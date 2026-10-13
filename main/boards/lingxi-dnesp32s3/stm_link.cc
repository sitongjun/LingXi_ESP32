#include "stm_link.h"

#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#define TAG "StmLink"

static constexpr size_t kMaxPayload = 1024;
static constexpr TickType_t kFrameGapTimeout = pdMS_TO_TICKS(50);

uint16_t StmLink::Crc16(const uint8_t* data, size_t len, uint16_t crc) {
    while (len--) {
        crc ^= static_cast<uint16_t>(*data++) << 8;
        for (int i = 0; i < 8; i++) {
            crc = (crc & 0x8000) ? static_cast<uint16_t>((crc << 1) ^ 0x1021) : static_cast<uint16_t>(crc << 1);
        }
    }
    return crc;
}

void StmLink::Start(uart_port_t uart, gpio_num_t tx, gpio_num_t rx, int baud) {
    uart_ = uart;
    uart_config_t cfg = {};
    cfg.baud_rate = baud;
    cfg.data_bits = UART_DATA_8_BITS;
    cfg.parity = UART_PARITY_DISABLE;
    cfg.stop_bits = UART_STOP_BITS_1;
    cfg.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
    cfg.source_clk = UART_SCLK_DEFAULT;
    ESP_ERROR_CHECK(uart_driver_install(uart_, 4096, 0, 0, nullptr, 0));
    ESP_ERROR_CHECK(uart_param_config(uart_, &cfg));
    ESP_ERROR_CHECK(uart_set_pin(uart_, tx, rx, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    gpio_set_pull_mode(rx, GPIO_PULLUP_ONLY);   // STM32 未连接时避免噪声

    xTaskCreate([](void* arg) {
        static_cast<StmLink*>(arg)->RxTask();
    }, "stm_link_rx", 4096, this, 5, nullptr);
    ESP_LOGI(TAG, "UART%d started, TX=%d RX=%d, %d bps", uart_, tx, rx, baud);
}

void StmLink::RxTask() {
    enum { H0, H1, LEN0, LEN1, CMD, SEQ, DATA, CRC0, CRC1 } state = H0;
    uint8_t hdr[4] = {};
    uint16_t len = 0, idx = 0, crc_rx = 0;
    std::vector<uint8_t> data(kMaxPayload);
    uint8_t buf[256];
    TickType_t last = 0;

    while (true) {
        int n = uart_read_bytes(uart_, buf, sizeof(buf), pdMS_TO_TICKS(20));
        if (n <= 0) {
            if (state != H0 && xTaskGetTickCount() - last > kFrameGapTimeout) {
                state = H0;   // 帧内超时，丢弃半帧
            }
            continue;
        }
        last = xTaskGetTickCount();
        for (int i = 0; i < n; i++) {
            uint8_t c = buf[i];
            switch (state) {
            case H0:
                if (c == 0xAA) state = H1;
                break;
            case H1:
                state = (c == 0x55) ? LEN0 : (c == 0xAA ? H1 : H0);
                break;
            case LEN0:
                hdr[0] = c;
                state = LEN1;
                break;
            case LEN1:
                hdr[1] = c;
                len = hdr[0] | (c << 8);
                state = len > kMaxPayload ? H0 : CMD;
                break;
            case CMD:
                hdr[2] = c;
                state = SEQ;
                break;
            case SEQ:
                hdr[3] = c;
                idx = 0;
                state = len ? DATA : CRC0;
                break;
            case DATA:
                data[idx++] = c;
                if (idx >= len) state = CRC0;
                break;
            case CRC0:
                crc_rx = c;
                state = CRC1;
                break;
            case CRC1:
                crc_rx |= c << 8;
                state = H0;
                if (Crc16(data.data(), len, Crc16(hdr, 4)) == crc_rx) {
                    OnFrame(hdr[2], hdr[3], data.data(), len);
                } else {
                    ESP_LOGW(TAG, "CRC error, cmd=0x%02X", hdr[2]);
                }
                break;
            }
        }
    }
}

void StmLink::OnFrame(uint8_t cmd, uint8_t seq, const uint8_t* data, size_t len) {
    if (cmd == kEvent) {
        std::string json(reinterpret_cast<const char*>(data), len);
        ESP_LOGI(TAG, "event: %s", json.c_str());
        if (event_cb_) {
            event_cb_(json);
        }
        return;
    }
    std::lock_guard<std::mutex> lock(resp_mutex_);
    if (cmd == wait_cmd_ && seq == wait_seq_ && !resp_ready_) {
        resp_data_.assign(data, data + len);
        resp_ready_ = true;
        resp_cv_.notify_all();
    }
}

bool StmLink::Request(uint8_t cmd, const std::vector<uint8_t>& payload, std::vector<uint8_t>& response,
                      int timeout_ms, int retries) {
    std::lock_guard<std::mutex> req_lock(request_mutex_);
    for (int attempt = 0; attempt < retries; attempt++) {
        uint8_t seq = ++seq_;
        std::vector<uint8_t> frame = {0xAA, 0x55,
                                      static_cast<uint8_t>(payload.size() & 0xFF),
                                      static_cast<uint8_t>(payload.size() >> 8), cmd, seq};
        frame.insert(frame.end(), payload.begin(), payload.end());
        uint16_t crc = Crc16(frame.data() + 2, frame.size() - 2);
        frame.push_back(crc & 0xFF);
        frame.push_back(crc >> 8);

        {
            std::lock_guard<std::mutex> lock(resp_mutex_);
            wait_cmd_ = cmd | 0x80;
            wait_seq_ = seq;
            resp_ready_ = false;
        }
        uart_write_bytes(uart_, frame.data(), frame.size());

        std::unique_lock<std::mutex> lock(resp_mutex_);
        if (resp_cv_.wait_for(lock, std::chrono::milliseconds(timeout_ms), [this] { return resp_ready_; })) {
            response = std::move(resp_data_);
            wait_cmd_ = 0;
            return true;
        }
        ESP_LOGW(TAG, "cmd 0x%02X timeout (attempt %d/%d)", cmd, attempt + 1, retries);
    }
    std::lock_guard<std::mutex> lock(resp_mutex_);
    wait_cmd_ = 0;
    return false;
}

bool StmLink::Ping(uint32_t* uptime_ms) {
    std::vector<uint8_t> resp;
    if (!Request(kPing, {}, resp) || resp.size() != 4) {
        return false;
    }
    if (uptime_ms) {
        *uptime_ms = resp[0] | (resp[1] << 8) | (resp[2] << 16) | (static_cast<uint32_t>(resp[3]) << 24);
    }
    return true;
}

bool StmLink::GetCaps(std::string& json) {
    std::vector<uint8_t> resp;
    if (!Request(kGetCaps, {}, resp, 500)) {
        return false;
    }
    json.assign(resp.begin(), resp.end());
    return true;
}

bool StmLink::GetState(std::string& json) {
    std::vector<uint8_t> resp;
    if (!Request(kGetState, {}, resp)) {
        return false;
    }
    json.assign(resp.begin(), resp.end());
    return true;
}

bool StmLink::Set(uint8_t dev_id, int16_t value, uint8_t& status, int16_t& new_value) {
    std::vector<uint8_t> resp;
    std::vector<uint8_t> payload = {dev_id, static_cast<uint8_t>(value & 0xFF),
                                    static_cast<uint8_t>((value >> 8) & 0xFF)};
    if (!Request(kSet, payload, resp) || resp.size() != 4) {
        return false;
    }
    status = resp[0];
    new_value = static_cast<int16_t>(resp[2] | (resp[3] << 8));
    return true;
}
