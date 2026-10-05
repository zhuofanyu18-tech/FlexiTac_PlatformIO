# FlexiTac PlatformIO / Arduino Nano

目标硬件：经典 Arduino Nano，ATmega328P，16 MHz，5 V。不是 Nano Every / Nano ESP32 / Nano 33 系列。
核对日期：2026-10-02。此工程为复刻后的软件接入起点，未在用户实体硬件上烧录验证。

## 文件

- `src/main.cpp`：由 PyFlexiTac 固件模板转换，补充 Arduino.h 和函数声明。
- `platformio.ini`：12 行 / 16 行及新旧 bootloader 四种环境。
- `host/heatmap.py`：新增 Python OpenCV 上位机，支持 12/16 行、原始读数、重新校准。
- `reference/`：官方原版 16×32 ino 和 Python 文件，仅作参考，不参与 PlatformIO 编译。

## VS Code / PlatformIO

安装 PlatformIO IDE 扩展。解压本工程，在 VS Code 中 Open Folder，打开包含 platformio.ini 的目录。
如自行新建项目，Board 选 Arduino Nano ATmega328，Framework 选 Arduino，再替换 ini 和 src/main.cpp。

默认 `nano12_old`：12×32，mux 通道 4–15，旧 bootloader。其他环境：

| 环境 | 分辨率 | MUX offset | bootloader |
| --- | --- | --- | --- |
| nano12_old | 12×32 | 4 | old |
| nano12_new | 12×32 | 4 | new |
| nano16_old | 16×32 | 0 | old |
| nano16_new | 16×32 | 0 | new |

在 PlatformIO 的 Project Tasks 中选环境，再 Build / Upload；或在 PlatformIO Terminal 中：

```sh
pio run -e nano12_old
pio run -e nano12_old -t upload
```

新 bootloader 用 nano12_new。若无法自动找到端口，可在对应环境下加入实际 `upload_port = /dev/cu.xxx` 或 COM 端口。
USB 串口芯片为 CH340 并不能单独证明是旧 bootloader。烧录前关闭上位机和 Serial Monitor。
monitor_speed=2000000 是运行时的数据串口速率；烧录速率由所选 board 的 bootloader 配置决定。

## 上位机（macOS 示例，Windows 可将 port 换为 COM5 等）

```sh
python3 -m venv .venv
source .venv/bin/activate
python -m pip install -r host/requirements.txt
python -m serial.tools.list_ports
python host/heatmap.py --port /dev/cu.usbserial-XXXX --rows 12
```

XXXX 必须替换为上一条命令列出的真实设备。首次 30 帧保持无压力；R 重新校准，Q / Esc 退出。
如烧录 nano16_*，上位机改为 `--rows 16`。排查读数可附加 `--raw`。
显示的是 ADC / 基线差值，不是经过力标定的牛顿值。阈值 15 沿用原例，可通过 --threshold 修改。

## 12-pin FPC 与 16-pin 接口

教程的 Top FPC Gerber 有 12 个 0.5 mm 间距金手指；Bottom FPC 有 32 个。
教程读出板的 FPC1 为 16 路，连接 CD74HC4067 的 I0–I15，FPC2 为 32 路。
所以传感器有效阵列 12×32，而读出板容量 16×32。多出的 4 路并不代表传感器缺失或需要补上电极。
PyFlexiTac 的标准接线使用 mux 通道 4–15（这来自该驱动的默认配置，仍需核对你实体转接后的接线）。按官方原理图，FPC1 脚号 n 对应 mux I(n-1)，因此这组通道对应 FPC1 的信号脚 5–16。

这不等于“任何 12-pin FPC 都能直接插进 16-pin 插座”。教程采用 FFC extender / 转接，然后以 16-pin FFC 连读出板。
应确认间距、触点朝向、排线两端同面/异面以及编号方向；使用转接或定位以保证对齐。
断电，用万用表通断档确认 12 条线实际到达 FPC1 的哪些脚；别仅凭左右或居中猜位置。
若实际连到 I0–I11，则将 12 行环境的 `-DMUX_CHANNEL_OFFSET=4` 改为 0；若到 I2–I13，改为 2。
如果次序反向，热图会反向，需要对应反转行显示或扫描顺序；offset 不能修正乱序接线。
原理图中 FPC1 的 17/18、FPC2 的 33/34 是额外固定焊脚，不能算作传感器信号 pin。

## 引脚与协议

| Nano 引脚 | 作用 |
| --- | --- |
| A0 | HC4067 公共端 ADC 输入 |
| D2 | 74HC595 SER 数据 |
| D3 | 74HC595 SRCLK / RCLK 时钟 |
| D4–D7 | HC4067 S0–S3 |
| D8 | HC4067 E#，低电平使能 |
| D9 | 原固件为扩展保留，此单 mux 原理图未使用 |
| 5V / GND | 共电源 / 共地 |

运行波特率 2,000,000；每帧 AA 55，然后 rows×32 个 uint8。
12 行：384 数据字节，整帧 386 字节；16 行：512 数据字节，整帧 514 字节。
不要在这个串口中添加 Serial.println 调试文本；它会混入二进制数据。
固件保留上游高速 ADC 分频设置，这会牺牲一定 ADC 精度；若需定量测力，应另做采样与力标定验证。

## 来源与许可

- 官方硬件教程：https://docs.google.com/document/d/1bvz6AL7BUkhj4Dj7n9DFXTjnGIX4-ziN8-smiCKpVZU/edit
- 官方代码：https://github.com/FlexiTac/FlexiTac_Hardware_Repo
- 官方 ino：https://github.com/FlexiTac/FlexiTac_Hardware_Repo/blob/main/32_16_fast.ino
- 官方上位机：https://github.com/FlexiTac/FlexiTac_Hardware_Repo/blob/main/fast_32_16.py
- PyFlexiTac：https://github.com/WT-MM/PyFlexiTac （默认 12 行、offset=4）
- CPP 转换：https://docs.platformio.org/en/latest/faq/ino-to-cpp.html
- Nano 环境：https://docs.platformio.org/en/latest/boards/atmelavr/nanoatmega328.html
- Nano 新 bootloader：https://docs.platformio.org/en/latest/boards/atmelavr/nanoatmega328new.html

reference 中的官方代码来源项目声明 CC BY-NC 4.0，归 Binghao Huang / Yunzhu Li / Columbia University，
许可：https://creativecommons.org/licenses/by-nc/4.0/ 。
src/main.cpp 由 WT-MM/PyFlexiTac MIT 模板改编，其上游声明源于 Binghao Huang 固件；原 MIT 许可见 reference/PyFlexiTac_LICENSE。
新增 host/heatmap.py 为本次编写的辅助程序。请尊重原硬件/代码项目的署名和非商业许可要求。
