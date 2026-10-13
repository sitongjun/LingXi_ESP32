# 灵犀居家管家 —— DNESP32S3 语音终端固件

本仓库基于开源项目 [xiaozhi-esp32](https://github.com/78/xiaozhi-esp32)（MIT 许可）二次开发，
语音链路（唤醒、录音、编解码、服务器通信）沿用小智框架，新增内容见下表。

主仓库（文档、STM32 工程）：<https://github.com/sitongjun/LingXi>

## 新增 / 修改内容

| 文件 | 说明 |
|---|---|
| `main/boards/lingxi-dnesp32s3/` | 新板型：正点原子 DNESP32S3（去掉摄像头）+ STM32 家居主控 |
| `main/boards/lingxi-dnesp32s3/stm_link.*` | 与 STM32 的串口帧协议驱动（CRC16、应答、超时重发） |
| `main/boards/lingxi-dnesp32s3/lingxi_dnesp32s3.cc` | 启动时读取 STM32 能力清单，动态注册 MCP 工具 |
| `main/CMakeLists.txt`、`main/Kconfig.projbuild` | 注册新板型 |
| `main/idf_component.yml`、`local_components/image_player` | `image_player 1.1.0~1` 已从组件仓库下架，改用本地副本 |

## 硬件连接

| DNESP32S3（EX IO 排针 P1） | STM32（普中 F103ZET6） |
|---|---|
| IO17（TX） | PB11（USART3_RX） |
| IO18（RX） | PB10（USART3_TX） |
| GND | GND |

## 编译

环境：ESP-IDF **v5.4.2**。

```powershell
idf.py set-target esp32s3
idf.py menuconfig    # Xiaozhi Assistant -> Board Type -> 灵犀居家管家 (DNESP32S3 + STM32)
idf.py build
idf.py -p COMx flash
```

## 动态 MCP 工具

ESP32 启动后向 STM32 发送 `GET_CAPS`，按返回的设备列表注册：

- `self.home.get_state`：查询全部设备状态
- `self.home.<设备名>.set`：控制单个设备（`switch` 类型参数 `on`，`level` 类型参数 `level`）

STM32 端新增设备只需修改其设备表，ESP32 固件无需改动。
