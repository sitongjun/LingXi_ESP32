#ifndef _STM_LINK_H_
#define _STM_LINK_H_

/*
 * 与 STM32 家居主控通信的帧协议（与 LingXi_STM32/APP/link/link.h 保持一致）
 *
 *   | 0xAA | 0x55 | LEN_L | LEN_H | CMD | SEQ | PAYLOAD | CRC_L | CRC_H |
 *   CRC16-CCITT(0x1021, init 0xFFFF)，范围 LEN_L .. PAYLOAD 末尾
 *   应答 CMD = 请求 CMD | 0x80，SEQ 相同；超时自动重发
 */

#include <driver/uart.h>
#include <driver/gpio.h>

#include <condition_variable>
#include <cstdint>
#include <functional>
#include <mutex>
#include <string>
#include <vector>

class StmLink {
public:
    enum Cmd : uint8_t {
        kPing = 0x01,
        kGetCaps = 0x02,
        kSet = 0x03,
        kGetState = 0x04,
        kEvent = 0x40,
    };

    enum SetStatus : uint8_t {
        kOk = 0,
        kNoDevice = 1,
        kOutOfRange = 2,
        kBadLength = 3,
    };

    using EventCallback = std::function<void(const std::string& json)>;

    void Start(uart_port_t uart, gpio_num_t tx, gpio_num_t rx, int baud);
    void OnEvent(EventCallback cb) { event_cb_ = cb; }

    // 发送请求并等待应答，超时自动重发；成功返回 true
    bool Request(uint8_t cmd, const std::vector<uint8_t>& payload, std::vector<uint8_t>& response,
                 int timeout_ms = 300, int retries = 3);

    bool Ping(uint32_t* uptime_ms = nullptr);
    bool GetCaps(std::string& json);
    bool GetState(std::string& json);
    bool Set(uint8_t dev_id, int16_t value, uint8_t& status, int16_t& new_value);

    static uint16_t Crc16(const uint8_t* data, size_t len, uint16_t crc = 0xFFFF);

private:
    uart_port_t uart_ = UART_NUM_1;
    uint8_t seq_ = 0;
    EventCallback event_cb_;

    std::mutex request_mutex_;          // 同一时间只允许一个请求在途
    std::mutex resp_mutex_;
    std::condition_variable resp_cv_;
    bool resp_ready_ = false;
    uint8_t wait_cmd_ = 0;
    uint8_t wait_seq_ = 0;
    std::vector<uint8_t> resp_data_;

    void RxTask();
    void OnFrame(uint8_t cmd, uint8_t seq, const uint8_t* data, size_t len);
};

#endif // _STM_LINK_H_
