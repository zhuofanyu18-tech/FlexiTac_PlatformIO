# FlexiTac PlatformIO

把开源柔性触觉传感器 [FlexiTac](https://github.com/FlexiTac/FlexiTac_Hardware_Repo) 的 Arduino 固件移植成 **PlatformIO 工程**，适配我们手上的 **12×32 触觉阵列 + 经典 Arduino Nano 读出板**，并配套 Python 上位机：2D 热力图、MuJoCo 3D 连续地形，以及数据采集诊断工具。

```
触觉传感器 (12×32) ──FPC──► 读出板 (HC4067 + 74HC595, 容量 16×32) ──USB──► PC
                                     Arduino Nano (ATmega328P)          host/ 上位机
```

## 我们做了什么

- **固件移植到 PlatformIO**：基于 PyFlexiTac 的 `template.ino`（MIT），改写为 `src/main.cpp`：补了 `Arduino.h` 和函数声明，改用寄存器操作，保留上游的高速 ADC 设置。
- **适配 12 行传感器**：传感器有效阵列是 12×32，读出板容量是 16×32。行数 `ROW_COUNT` 和 mux 起始通道 `MUX_CHANNEL_OFFSET` 都可以在编译期配置，默认扫描 mux 通道 4–15。
- **多套编译环境**：`platformio.ini` 中提供 12 行 / 16 行、新 / 旧 bootloader，以及 3 个诊断固件（快 ADC、慢 ADC、单点文本探针）。
- **上位机热力图** `host/heatmap/`：OpenCV 实时显示，支持基线自动校准、原始值模式、逐秒统计和录制，运行中可用 `+`/`-` 调亮度。帧解析器会在每个帧边界校验帧头，丢字节后能自动重新同步。
- **MuJoCo 3D 地形** `host/mujoco_viewer/`：触觉数据经插值和平滑后写入 MuJoCo 高度场，显示为连续起伏的"山脉"，颜色随按压大小变化（蓝 → 绿 → 黄 → 红）。
- **诊断工具** `host/diagnose/`：无界面采集数据并录制成 `.npz/.csv/.json`，可以对比“松开 / 按压”两段录制，帮助定位接线和通道映射。
- **单元测试** `tests/`：覆盖帧解析（分包、伪帧头、数据中含 `AA 55`）、录制保真和对比逻辑。

**当前状态**（2026-10-06）：Nano 已烧录 12 行二进制固件，从 `/dev/ttyUSB0` 能稳定读到 12×32 帧，约 123 fps；单元测试 6/6 通过；热力图和 MuJoCo 地形都已在实机串口上跑通。读数是 ADC 相对值，**没有做力标定**（不是牛顿值）。

## 目录结构

| 路径 | 说明 |
| --- | --- |
| `src/main.cpp` | Nano 固件（二进制扫描 / 文本诊断两种模式） |
| `platformio.ini` | 编译 / 烧录环境 |
| `host/heatmap/heatmap.py` | 上位机：2D 实时热力图 |
| `host/mujoco_viewer/mujoco_viewer.py` | 上位机：MuJoCo 3D 连续地形 |
| `host/mujoco_viewer/terrain.py` | 地形生成：插值、平滑、高度场与按高度着色 |
| `host/diagnose/diagnose.py` | 上位机：无界面采集与松开 / 按压对比 |
| `host/common/` | 共用模块：串口帧解析 `frame_reader.py`、基线与亮度处理 `processing.py`、统计录制 `diagnostics.py` |
| `host/requirements.txt` | Python 依赖：numpy、pyserial、opencv-python、mujoco |
| `tests/` | 单元测试 |
| `reference/` | 官方原版 ino / py、读出板原理图 PDF，仅作参考，不参与编译 |
| `VALIDATION.md` | 验证记录 |
| `plan/` | 后续计划：[力标定（称重传感器 + ESP32-S3）](plan/force_calibration.md) |

## 硬件要求

- **经典 Arduino Nano**（ATmega328P，16 MHz，5 V）。Nano Every、Nano ESP32、Nano 33 系列都不支持（固件里有编译期检查）。
- FlexiTac 读出板：CD74HC4067 负责行，74HC595 级联负责 32 列。
- USB 串口芯片一般是 CH340，Linux 下对应 `/dev/ttyUSB0`。

---

## 快速开始（Linux）

```sh
# 0. 确认串口
ls -l /dev/ttyUSB*                 # 例：/dev/ttyUSB0

# 1. 烧录 12 行固件（新 bootloader）
pio run -e nano12_new -t upload --upload-port /dev/ttyUSB0

# 2. 打开 2D 热力图
conda activate rebot-mujoco        # 或任意装好 requirements.txt 的环境
python host/heatmap/heatmap.py --port /dev/ttyUSB0 --rows 12

# 3. 或者打开 MuJoCo 3D 地形
python host/mujoco_viewer/mujoco_viewer.py --port /dev/ttyUSB0 --rows 12
```

前 30 帧不要碰传感器，等它做基线校准，终端会提示 `Calibration complete.`，之后按压就能看到变化。

---

## 一、烧录固件

### 1. 安装 PlatformIO

- **VS Code**：安装 PlatformIO IDE 扩展（打开本工程时会自动推荐），然后用 Open Folder 打开包含 `platformio.ini` 的目录。
- **命令行**：扩展自带 CLI，路径是 `~/.platformio/penv/bin/pio`。如果终端里找不到 `pio`，可以加到 PATH：

  ```sh
  echo 'export PATH="$HOME/.platformio/penv/bin:$PATH"' >> ~/.bashrc && source ~/.bashrc
  ```

### 2. 选择环境

| 环境 | 用途 | 分辨率 | MUX 通道 | bootloader | 上位机参数 |
| --- | --- | --- | --- | --- | --- |
| `nano12_new` **（默认）** | 正常使用 | 12×32 | 4–15 | new | `--rows 12` |
| `nano12_old` | 正常使用 | 12×32 | 4–15 | old | `--rows 12` |
| `nano16_new` | 扫描全部通道 | 16×32 | 0–15 | new | `--rows 16` |
| `nano16_old` | 扫描全部通道 | 16×32 | 0–15 | old | `--rows 16` |
| `nano_diag16_fast` | 诊断：快 ADC（与正常固件相同） | 16×32 | 0–15 | new | `--rows 16` |
| `nano_diag16_slow` | 诊断：慢 ADC + 稳定延时 + 丢弃首次采样 | 16×32 | 0–15 | new | `--rows 16` |
| `nano_diag_fixed` | 诊断：固定单个行列点，输出文本 | — | 可调 | new | 用 miniterm，见下文 |

**新 / 旧 bootloader 怎么判断**：先用 `*_new` 烧录。如果报 `stk500_recv(): programmer is not responding` 或 `not in sync`，就换成 `*_old`。CH340 芯片不能说明是旧 bootloader。

### 3. 编译与烧录

```sh
pio run -e nano12_new                                        # 只编译
pio run -e nano12_new -t upload --upload-port /dev/ttyUSB0   # 编译并烧录
```

在 VS Code 里：左下角状态栏选择环境 → 点 ✓（Build）→ 点 →（Upload）。

注意：

- **烧录前关掉上位机、Serial Monitor 和其他占用串口的程序**，否则会报串口被占用。
- 不要用 PlatformIO 的 Serial Monitor 看二进制固件的输出，看到的只会是乱码。数据请用上位机看。
- `monitor_speed = 2000000` 是运行时的数据串口速率；烧录速率由 bootloader 决定，两者无关。

---

## 二、上位机

### 1. 安装依赖

```sh
# 方式 A：conda（当前使用的 rebot-mujoco 环境已装好）
conda activate rebot-mujoco
pip install -r host/requirements.txt

# 方式 B：venv
python3 -m venv .venv && source .venv/bin/activate
pip install -r host/requirements.txt
```

查看可用串口：

```sh
python -m serial.tools.list_ports
```

### 2. 2D 实时热力图 `heatmap/heatmap.py`

```sh
# 常用：12 行固件 + 自动基线校准
python host/heatmap/heatmap.py --port /dev/ttyUSB0 --rows 12

# 更亮：缩小满亮度量程、降低阈值（运行中也可以按 + / - 调）
python host/heatmap/heatmap.py --port /dev/ttyUSB0 --rows 12 --scale 20 --threshold 3

# 换色表
python host/heatmap/heatmap.py --port /dev/ttyUSB0 --rows 12 --cmap turbo

# 看原始 ADC 值（不做基线扣除，用来排查接线和通道）
python host/heatmap/heatmap.py --port /dev/ttyUSB0 --rows 12 --raw

# 每秒打印行列统计，并把这段数据录到 captures/
python host/heatmap/heatmap.py --port /dev/ttyUSB0 --rows 12 --diagnostics --record captures/test1 --seconds 20
```

**`--rows` 必须和烧录的固件一致**（`nano12_*` 对应 12，`nano16_*` / `nano_diag16_*` 对应 16）。不一致时，8 秒后会报 `No valid frames`。

键盘操作（焦点在热力图窗口上时有效）：

| 按键 | 作用 |
| --- | --- |
| `R` | 重新校准基线（先松开传感器） |
| `+` / `-` | 调亮 / 调暗（每次把满亮度量程 `scale` 乘以 0.8 / 1.25） |
| `1` / `2` / `3` | 把当前状态标注为 released / pressed / moving（只写进录制数据） |
| `Q` / `Esc` | 退出 |

常用参数：

| 参数 | 默认 | 说明 |
| --- | --- | --- |
| `--port` | 必填 | Linux `/dev/ttyUSB0`，macOS `/dev/cu.usbserial-*`，Windows `COM5` |
| `--rows` | 12 | 12 或 16，与固件一致 |
| `--threshold` | 4 | 平滑后的增量低于该值不显示（静止噪声约 ±3） |
| `--scale` | 40 | 满亮度对应的 ADC 增量，**越小越亮** |
| `--gamma` | 0.6 | 小于 1 时提亮轻按，1 为线性 |
| `--alpha` | 0.3 | 时间平滑系数，越大响应越快、噪声越多 |
| `--cmap` | viridis | 色表：viridis / turbo / inferno / jet / hot |
| `--raw` | 关 | 原始值模式，跳过校准和滤波 |
| `--raw-scale` | 64 | 原始值模式下的满亮度值 |
| `--diagnostics` | 关 | 每秒打印逐行 / 逐列统计 |
| `--record PREFIX` | — | 录制为 `PREFIX.npz/.csv/.json` |
| `--seconds` | 0 | 采集多少秒后自动退出，0 表示直到按 Q |
| `--mux-offset` | 自动 | 只影响打印的通道标签，不会修改固件 |

终端每秒会输出一行状态，例如 `fps_avg=123.5 raw_min=0 raw_max=9 max_at=(7,22) mux=11 ...`。其中 `max_at` 是当前读数最大的位置（行, 列），从 0 开始计数。

显示原理（`common/processing.py`，热力图和 MuJoCo 共用）：增量 = 当前值 − 基线 → 一阶低通平滑（`alpha`）→ `clip((平滑值 − threshold) / scale, 0, 1) ^ gamma`。先平滑再减阈值，偶发的单帧尖峰不会闪出来。

### 3. MuJoCo 3D 地形 `mujoco_viewer/mujoco_viewer.py`

把 12×32 的触觉矩阵显示成连续起伏的地形：按得越重，山越高，颜色从蓝 → 青 → 绿 → 黄 → 红变化。

```sh
# 实机
python host/mujoco_viewer/mujoco_viewer.py --port /dev/ttyUSB0 --rows 12

# 没接硬件时用模拟数据看效果
python host/mujoco_viewer/mujoco_viewer.py --demo

# 更灵敏、山更高、更平滑
python host/mujoco_viewer/mujoco_viewer.py --port /dev/ttyUSB0 --scale 20 --height-mm 45 --blur 1.0
```

实现方式：每帧把矩阵四周补一圈 0，双三次插值放大 `upsample` 倍，再做高斯平滑，得到连续曲面后写入 MuJoCo 的 heightfield（`hfield`）；同时用色表按高度生成一张同分辨率的纹理贴到曲面上。所以高度和颜色都由同一个压力值决定，相邻传感点之间是平滑过渡，没有断层。

操作：鼠标左键拖动旋转，右键平移，滚轮缩放；`Enter` 重新校准，`=` / `+` 更灵敏，`-` 更迟钝；关闭窗口即退出。

| 参数 | 默认 | 说明 |
| --- | --- | --- |
| `--port` / `--demo` | — | 二选一：串口或模拟数据 |
| `--rows` | 12 | 与固件一致 |
| `--threshold` / `--scale` / `--gamma` / `--alpha` | 4 / 40 / 0.6 / 0.3 | 与热力图含义相同 |
| `--cmap` | turbo | 高度色表：turbo / jet / viridis / inferno / plasma / hot |
| `--height-mm` | 30 | 满量程山峰的显示高度 |
| `--pitch-mm` | 5 | 相邻传感点的显示间距 |
| `--upsample` | 8 | 每个传感点细分成多少个网格顶点，越大越细腻 |
| `--blur` | 0.7 | 高斯平滑半径（单位：传感点），0 表示只做插值 |
| `--fps` | 60 | 渲染刷新上限 |
| `--snapshot PNG` | — | 不开窗口，离屏渲染一张图后退出（配合 `--seconds`） |

终端每秒输出 `render_fps`、已接收帧数、当前最高点 `peak`（0–1）和原始最大值位置。

### 4. 无界面采集与对比 `diagnose/diagnose.py`

适合定位“按哪里、哪一行哪一列在变”：

```sh
# 松开状态录 10 秒
python host/diagnose/diagnose.py --port /dev/ttyUSB0 --rows 12 --label released --record captures/rel --seconds 10
# 持续按住某一点录 10 秒
python host/diagnose/diagnose.py --port /dev/ttyUSB0 --rows 12 --label pressed  --record captures/press --seconds 10
# 对比两段录制：打印带符号的中位数差矩阵和变化最大的 10 个点
python host/diagnose/diagnose.py --compare captures/rel.npz captures/press.npz
```

`diagnose.py` 的 `--rows` 默认值是 **16**，用 12 行固件时记得加 `--rows 12`。`captures/` 已在 `.gitignore` 中。

### 5. 单点文本探针 `nano_diag_fixed`

把行和列固定在某一个点上，用文本输出 10-bit ADC 读数，方便对着万用表或按压逐点检查：

```sh
pio run -e nano_diag_fixed -t upload --upload-port /dev/ttyUSB0
python -m serial.tools.miniterm /dev/ttyUSB0 115200
```

在 miniterm 中输入命令并回车：`m 4`（选 mux 通道 0–15）、`c 0`（选列 0–31）、`e 0/1`（mux 使能）、`d 0/1`（列驱动）、`h`（帮助）。用完后要重新烧录 `nano12_new`，才能回到热力图模式。

### 6. 运行测试

```sh
python -m unittest discover -s tests
```

---

## 常见问题

| 现象 | 原因 / 处理 |
| --- | --- |
| `No valid frames for 8 seconds` | `--rows` 与固件不一致；或者烧的是 `nano_diag_fixed` 文本固件；或者串口选错 |
| `Permission denied: /dev/ttyUSB0` | `sudo usermod -aG dialout $USER`，然后重新登录 |
| `Could not exclusively lock port` / 烧录时串口忙 | 关掉其他占用串口的上位机、miniterm 或 Serial Monitor |
| 插上 Nano 后看不到 `/dev/ttyUSB0`（Ubuntu） | `brltty` 抢占了 CH340：`sudo apt remove brltty`，然后重新插拔 |
| 烧录报 `not in sync` / `not responding` | 换 `*_old` 环境 |
| 热力图太暗 / 全黑 | 按 `+` 调亮，或用 `--scale 20 --threshold 3`；如果校准时手按着传感器，松开后按 `R`；也可以用 `--raw` 看原始值 |
| MuJoCo 地形不起伏 | 同上，调小 `--scale`；校准时按着传感器的话，松开后按 `Enter` |
| `sync_losses` 持续缓慢增长 | 当前硬件在 2 Mbaud 下每秒约丢 3 帧（约 2%），解析器会自动重新同步，不影响显示；如果增长很快，检查 USB 线质量 |
| 图像上下颠倒或错位 | 接线次序反了或 offset 不对，见下方“FPC 接线” |

---

## 技术细节

### 引脚

| Nano 引脚 | 作用 |
| --- | --- |
| A0 | HC4067 公共端 ADC 输入 |
| D2 | 74HC595 SER 数据 |
| D3 | 74HC595 SRCLK / RCLK（共用时钟） |
| D4–D7 | HC4067 S0–S3 |
| D8 | HC4067 E#，低电平使能 |
| D9 | 原固件为扩展保留，单 mux 原理图未使用 |
| 5V / GND | 共电源 / 共地 |

### 串口协议

- 波特率 2,000,000，8N1。
- 每帧：`AA 55` 帧头，接着是 `rows × 32` 个 `uint8`，按行优先排列。每个值是 10-bit ADC 读数右移 2 位。
- 12 行：384 数据字节，整帧 386 字节；16 行：512 数据字节，整帧 514 字节。
- 协议没有 CRC 和序号。上位机通过“下一帧帧头也必须对齐”来判断帧边界。
- **不要在二进制固件里加 `Serial.print` 调试**，否则调试文本会混进数据流。需要调试时用 `nano_diag_fixed`。
- 默认 ADC 分频为 16（1 MHz ADC 时钟），速度快但会损失一些精度；需要定量测力时，可以对比 `nano_diag16_slow` 并另做力标定。

### 编译宏

可以在 `platformio.ini` 的 `build_flags` 里调整：

| 宏 | 默认 | 说明 |
| --- | --- | --- |
| `ROW_COUNT` | 12 | 扫描行数 |
| `MUX_CHANNEL_OFFSET` | 4 | 第 0 行对应的 HC4067 通道；要求 `ROW_COUNT + OFFSET ≤ 16` |
| `ADC_PRESCALER` | 16 | 16 或 128 |
| `SAMPLE_SETTLE_US` | 0 | 切换通道后的稳定延时（微秒） |
| `ADC_DISCARD_FIRST` | 0 | 1 = 每点先丢弃一次采样 |
| `DIAGNOSTIC_TEXT` | 0 | 1 = 单点文本探针模式（115200） |
| `FIXED_MUX_CHANNEL` / `FIXED_COLUMN` | 4 / 0 | 文本探针的初始行列 |

### FPC 接线：12 行传感器接 16 路读出板

- 传感器 Top FPC 有 12 根 0.5 mm 间距金手指（行），Bottom FPC 有 32 根（列）。
- 读出板 FPC1 有 16 路，接 HC4067 的 I0–I15；FPC2 有 32 路。所以有效阵列是 12×32，读出板有 4 路空闲，这是正常的。
- PyFlexiTac 的标准接法使用 mux 通道 4–15。按官方原理图，FPC1 的第 n 脚对应 I(n−1)，也就是 FPC1 的信号脚 5–16。
- 教程使用 FFC 转接板接到 16-pin 排线，**不是**把 12-pin 直接插进 16-pin 插座。需要确认间距、触点朝向、排线同面 / 异面以及编号方向。
- 断电后，用万用表通断档确认 12 根线实际到达 FPC1 的哪些脚：
  - 接到 I0–I11 → 把 `-DMUX_CHANNEL_OFFSET=4` 改为 `0`；接到 I2–I13 → 改为 `2`。
  - 次序反了时，热图会上下颠倒，需要在显示或扫描顺序上反转；offset 只能平移，修不了乱序。
- 原理图中 FPC1 的 17/18 脚和 FPC2 的 33/34 脚是固定焊脚，不是信号脚。
- 不确定接法时，可以先烧 `nano16_new`，用 `--rows 16 --raw` 看哪 12 行有响应。

---

## 来源与许可

- 官方硬件教程：https://docs.google.com/document/d/1bvz6AL7BUkhj4Dj7n9DFXTjnGIX4-ziN8-smiCKpVZU/edit
- 官方仓库：https://github.com/FlexiTac/FlexiTac_Hardware_Repo
  （[32_16_fast.ino](https://github.com/FlexiTac/FlexiTac_Hardware_Repo/blob/main/32_16_fast.ino)、[fast_32_16.py](https://github.com/FlexiTac/FlexiTac_Hardware_Repo/blob/main/fast_32_16.py)）
- PyFlexiTac：https://github.com/WT-MM/PyFlexiTac （默认 12 行、offset = 4）
- PlatformIO：[ino → cpp](https://docs.platformio.org/en/latest/faq/ino-to-cpp.html)、[nanoatmega328](https://docs.platformio.org/en/latest/boards/atmelavr/nanoatmega328.html)、[nanoatmega328new](https://docs.platformio.org/en/latest/boards/atmelavr/nanoatmega328new.html)

`reference/` 中的官方代码以 **CC BY-NC 4.0** 发布，版权归 Binghao Huang / Yunzhu Li / Columbia University（[许可](https://creativecommons.org/licenses/by-nc/4.0/)）。
`src/main.cpp` 改编自 WT-MM/PyFlexiTac 的 MIT 模板，其上游声明源自 Binghao Huang 的固件；原 MIT 许可见 `reference/PyFlexiTac_LICENSE`。
`host/` 下的脚本为本项目新写的辅助程序。请遵守原硬件和代码项目的署名与非商业许可要求。
