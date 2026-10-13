/*
 * 灵犀居家管家 —— DNESP32S3 语音终端板型
 * 基于 atk-dnesp32s3 板型（去掉摄像头），增加与 STM32 家居主控的串口连接：
 * 启动时读取 STM32 的“能力清单”，为每个设备动态注册 MCP 工具，供大模型调用。
 */
#include "wifi_board.h"
#include "codecs/es8388_audio_codec.h"
#include "display/lcd_display.h"
#include "application.h"
#include "button.h"
#include "config.h"
#include "i2c_device.h"
#include "led/single_led.h"
#include "mcp_server.h"
#include "stm_link.h"
#include "assets/lang_config.h"

#include <esp_log.h>
#include <esp_lcd_panel_vendor.h>
#include <driver/i2c_master.h>
#include <driver/spi_common.h>
#include <wifi_station.h>
#include <cJSON.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <map>

#define TAG "lingxi_dnesp32s3"

LV_FONT_DECLARE(font_puhui_20_4);
LV_FONT_DECLARE(font_awesome_20_4);

class XL9555 : public I2cDevice {
public:
    XL9555(i2c_master_bus_handle_t i2c_bus, uint8_t addr) : I2cDevice(i2c_bus, addr) {
        WriteReg(0x06, 0x03);
        WriteReg(0x07, 0xF0);
    }

    void SetOutputState(uint8_t bit, uint8_t level) {
        uint16_t data;
        int index = bit;

        if (bit < 8) {
            data = ReadReg(0x02);
        } else {
            data = ReadReg(0x03);
            index -= 8;
        }

        data = (data & ~(1 << index)) | (level << index);

        if (bit < 8) {
            WriteReg(0x02, data);
        } else {
            WriteReg(0x03, data);
        }
    }
};

class lingxi_dnesp32s3 : public WifiBoard {
private:
    i2c_master_bus_handle_t i2c_bus_;
    Button boot_button_;
    LcdDisplay* display_;
    XL9555* xl9555_;
    StmLink stm_link_;
    std::vector<uint8_t> heartbeat_resp_;
    std::map<std::string, std::string> labels_;   // 设备英文名 -> 中文名

    void InitializeI2c() {
        i2c_master_bus_config_t i2c_bus_cfg = {
            .i2c_port = (i2c_port_t)I2C_NUM_0,
            .sda_io_num = AUDIO_CODEC_I2C_SDA_PIN,
            .scl_io_num = AUDIO_CODEC_I2C_SCL_PIN,
            .clk_source = I2C_CLK_SRC_DEFAULT,
            .glitch_ignore_cnt = 7,
            .intr_priority = 0,
            .trans_queue_depth = 0,
            .flags = {
                .enable_internal_pullup = 1,
            },
        };
        ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_bus_cfg, &i2c_bus_));

        xl9555_ = new XL9555(i2c_bus_, 0x20);
    }

    void InitializeSpi() {
        spi_bus_config_t buscfg = {};
        buscfg.mosi_io_num = LCD_MOSI_PIN;
        buscfg.miso_io_num = GPIO_NUM_NC;
        buscfg.sclk_io_num = LCD_SCLK_PIN;
        buscfg.quadwp_io_num = GPIO_NUM_NC;
        buscfg.quadhd_io_num = GPIO_NUM_NC;
        buscfg.max_transfer_sz = DISPLAY_WIDTH * DISPLAY_HEIGHT * sizeof(uint16_t);
        ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO));
    }

    void InitializeButtons() {
        boot_button_.OnClick([this]() {
            auto& app = Application::GetInstance();
            if (app.GetDeviceState() == kDeviceStateStarting && !WifiStation::GetInstance().IsConnected()) {
                ResetWifiConfiguration();
            }
            app.ToggleChatState();
        });
    }

    void InitializeSt7789Display() {
        esp_lcd_panel_io_handle_t panel_io = nullptr;
        esp_lcd_panel_handle_t panel = nullptr;
        esp_lcd_panel_io_spi_config_t io_config = {};
        io_config.cs_gpio_num = LCD_CS_PIN;
        io_config.dc_gpio_num = LCD_DC_PIN;
        io_config.spi_mode = 0;
        io_config.pclk_hz = 20 * 1000 * 1000;
        io_config.trans_queue_depth = 7;
        io_config.lcd_cmd_bits = 8;
        io_config.lcd_param_bits = 8;
        esp_lcd_new_panel_io_spi(SPI2_HOST, &io_config, &panel_io);

        esp_lcd_panel_dev_config_t panel_config = {};
        panel_config.reset_gpio_num = GPIO_NUM_NC;
        panel_config.rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB;
        panel_config.bits_per_pixel = 16;
        panel_config.data_endian = LCD_RGB_DATA_ENDIAN_BIG,
        esp_lcd_new_panel_st7789(panel_io, &panel_config, &panel);

        esp_lcd_panel_reset(panel);
        xl9555_->SetOutputState(8, 1);
        xl9555_->SetOutputState(2, 0);

        esp_lcd_panel_init(panel);
        esp_lcd_panel_invert_color(panel, DISPLAY_BACKLIGHT_OUTPUT_INVERT);
        esp_lcd_panel_swap_xy(panel, DISPLAY_SWAP_XY);
        esp_lcd_panel_mirror(panel, DISPLAY_MIRROR_X, DISPLAY_MIRROR_Y);
        display_ = new SpiLcdDisplay(panel_io, panel,
                                    DISPLAY_WIDTH, DISPLAY_HEIGHT, DISPLAY_OFFSET_X, DISPLAY_OFFSET_Y, DISPLAY_MIRROR_X, DISPLAY_MIRROR_Y, DISPLAY_SWAP_XY,
                                    {
                                        .text_font = &font_puhui_20_4,
                                        .icon_font = &font_awesome_20_4,
                                        #if CONFIG_USE_WECHAT_MESSAGE_STYLE
                                            .emoji_font = font_emoji_32_init(),
                                        #else
                                            .emoji_font = DISPLAY_HEIGHT >= 240 ? font_emoji_64_init() : font_emoji_32_init(),
                                        #endif
                                    });
    }

    // ---------------- 与 STM32 家居主控的连接 ----------------

    void InitializeHomeLink() {
        stm_link_.Start(STM_LINK_UART, STM_LINK_TX_PIN, STM_LINK_RX_PIN, STM_LINK_BAUD);
        stm_link_.OnEvent([this](const std::string& json) {
            HandleHomeEvent(json);
        });

        RegisterStateTool();

        std::string caps;
        if (stm_link_.GetCaps(caps)) {
            ESP_LOGI(TAG, "STM32 caps: %s", caps.c_str());
            RegisterDeviceTools(caps);
        } else {
            ESP_LOGW(TAG, "STM32 home controller not responding, home device tools not registered");
        }

        // 心跳：每 5 秒 PING 一次，STM32 据此在屏幕上显示“AI 在线”
        xTaskCreate([](void* arg) {
            auto* self = static_cast<lingxi_dnesp32s3*>(arg);
            while (true) {
                vTaskDelay(pdMS_TO_TICKS(5000));
                self->stm_link_.Request(StmLink::kPing, {}, self->heartbeat_resp_, 300, 1);
            }
        }, "stm_heartbeat", 3072, this, 3, nullptr);
    }

    void RegisterStateTool() {
        McpServer::GetInstance().AddTool("self.home.get_state",
            "Get the real-time state of all home devices (lights, alarm, ...) from the STM32 home controller.\n"
            "Call this before answering questions about the home, or before changing a device whose current state is unknown.",
            PropertyList(),
            [this](const PropertyList& properties) -> ReturnValue {
                std::string state;
                if (!stm_link_.GetState(state)) {
                    return "{\"success\": false, \"message\": \"Home controller is offline\"}";
                }
                return state;
            });
    }

    // 根据 STM32 上报的能力清单，为每个设备生成一个 MCP 工具（设备能力自描述）
    void RegisterDeviceTools(const std::string& caps) {
        cJSON* root = cJSON_Parse(caps.c_str());
        if (root == nullptr) {
            ESP_LOGE(TAG, "Invalid caps JSON");
            return;
        }
        cJSON* devices = cJSON_GetObjectItem(root, "devices");
        cJSON* dev = nullptr;
        cJSON_ArrayForEach(dev, devices) {
            cJSON* id = cJSON_GetObjectItem(dev, "id");
            cJSON* name = cJSON_GetObjectItem(dev, "name");
            cJSON* desc = cJSON_GetObjectItem(dev, "desc");
            cJSON* label = cJSON_GetObjectItem(dev, "label");
            cJSON* type = cJSON_GetObjectItem(dev, "type");
            cJSON* min = cJSON_GetObjectItem(dev, "min");
            cJSON* max = cJSON_GetObjectItem(dev, "max");
            if (!cJSON_IsNumber(id) || !cJSON_IsString(name) || !cJSON_IsString(desc) || !cJSON_IsString(type)) {
                continue;
            }

            uint8_t dev_id = id->valueint;
            std::string dev_name = name->valuestring;
            bool is_switch = strcmp(type->valuestring, "switch") == 0;
            std::string tool_name = "self.home." + dev_name + ".set";
            std::string dev_label = cJSON_IsString(label) ? label->valuestring : dev_name;
            labels_[dev_name] = dev_label;
            std::string description = "[" + dev_label + "] " + desc->valuestring + "\n";
            PropertyList props;
            if (is_switch) {
                description += "Args:\n  `on`: true to turn on, false to turn off.";
                props.AddProperty(Property("on", kPropertyTypeBoolean));
            } else {
                int lo = cJSON_IsNumber(min) ? min->valueint : 0;
                int hi = cJSON_IsNumber(max) ? max->valueint : 100;
                description += "Args:\n  `level`: target level (" + std::to_string(lo) + " - " + std::to_string(hi) + ").";
                props.AddProperty(Property("level", kPropertyTypeInteger, lo, hi));
            }

            McpServer::GetInstance().AddTool(tool_name, description, props,
                [this, dev_id, dev_name, is_switch](const PropertyList& properties) -> ReturnValue {
                    int16_t value = is_switch ? (properties["on"].value<bool>() ? 1 : 0)
                                              : static_cast<int16_t>(properties["level"].value<int>());
                    uint8_t status = 0;
                    int16_t new_value = 0;
                    if (!stm_link_.Set(dev_id, value, status, new_value)) {
                        return "{\"success\": false, \"message\": \"Home controller is offline\"}";
                    }
                    if (status != StmLink::kOk) {
                        return "{\"success\": false, \"message\": \"Rejected by home controller, status " +
                               std::to_string(status) + "\"}";
                    }
                    return "{\"success\": true, \"device\": \"" + dev_name + "\", \"value\": " +
                           std::to_string(new_value) + "}";
                });
        }
        cJSON_Delete(root);
    }

    // STM32 主动上报的事件：按键操作在屏幕上提示；自动报警时语音终端同步告警
    void HandleHomeEvent(const std::string& json) {
        cJSON* root = cJSON_Parse(json.c_str());
        if (root == nullptr) {
            return;
        }
        cJSON* src = cJSON_GetObjectItem(root, "src");
        cJSON* dev = cJSON_GetObjectItem(root, "dev");
        cJSON* label = cJSON_GetObjectItem(root, "label");
        cJSON* value = cJSON_GetObjectItem(root, "value");
        if (cJSON_IsString(dev) && cJSON_IsNumber(value)) {
            std::string name = cJSON_IsString(label) ? label->valuestring : dev->valuestring;
            bool on = value->valueint != 0;
            bool is_auto = cJSON_IsString(src) && strcmp(src->valuestring, "auto") == 0;
            bool is_alarm = strcmp(dev->valuestring, "alarm") == 0;

            if (is_auto && is_alarm && on) {
                cJSON* t = cJSON_GetObjectItem(root, "temperature");
                cJSON* h = cJSON_GetObjectItem(root, "humidity");
                bool over_t = cJSON_IsTrue(cJSON_GetObjectItem(root, "over_temp"));
                bool over_h = cJSON_IsTrue(cJSON_GetObjectItem(root, "over_humi"));
                std::string msg = over_t && over_h ? "温湿度超限" : (over_t ? "温度过高" : "湿度过高");
                if (cJSON_IsNumber(t) && cJSON_IsNumber(h)) {
                    msg += "  " + std::to_string(t->valueint) + "°C / " + std::to_string(h->valueint) + "%";
                }
                Application::GetInstance().Schedule([msg]() {
                    Application::GetInstance().Alert("警告", msg.c_str(), "sad", Lang::Sounds::OGG_EXCLAMATION);
                });
            } else if (is_auto && is_alarm) {
                Application::GetInstance().Schedule([this]() {
                    Application::GetInstance().DismissAlert();
                    display_->ShowNotification("温湿度已恢复正常");
                });
            } else {
                std::string text = std::string(is_auto ? "自动：" : "按键：") + name + (on ? " 已开启" : " 已关闭");
                Application::GetInstance().Schedule([this, text]() {
                    display_->ShowNotification(text);
                });
            }
        }
        cJSON_Delete(root);
    }

public:
    lingxi_dnesp32s3() : boot_button_(BOOT_BUTTON_GPIO) {
        InitializeI2c();
        InitializeSpi();
        InitializeSt7789Display();
        InitializeButtons();
        InitializeHomeLink();
    }

    virtual Led* GetLed() override {
        static SingleLed led(BUILTIN_LED_GPIO);
        return &led;
    }

    virtual AudioCodec* GetAudioCodec() override {
        static Es8388AudioCodec audio_codec(
            i2c_bus_,
            I2C_NUM_0,
            AUDIO_INPUT_SAMPLE_RATE,
            AUDIO_OUTPUT_SAMPLE_RATE,
            AUDIO_I2S_GPIO_MCLK,
            AUDIO_I2S_GPIO_BCLK,
            AUDIO_I2S_GPIO_WS,
            AUDIO_I2S_GPIO_DOUT,
            AUDIO_I2S_GPIO_DIN,
            GPIO_NUM_NC,
            AUDIO_CODEC_ES8388_ADDR
        );
        return &audio_codec;
    }

    virtual Display* GetDisplay() override {
        return display_;
    }
};

DECLARE_BOARD(lingxi_dnesp32s3);
