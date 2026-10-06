# host 上位机

所有命令都在工程根目录运行。`--rows` 必须和烧录的固件一致（`nano12_*` → 12）。完整参数说明见根目录 [README.md](../README.md)。

```
host/
├── common/            共用模块
│   ├── frame_reader.py    串口帧解析（AA 55 + rows×32 字节）
│   ├── processing.py      基线校准、平滑、阈值、亮度
│   └── diagnostics.py     逐秒统计与录制（.npz/.csv/.json）
├── heatmap/           2D 热力图（OpenCV）
├── mujoco_viewer/     3D 连续地形（MuJoCo）
│   ├── mujoco_viewer.py   入口：串口线程 + 渲染循环
│   └── terrain.py         插值平滑 → 高度场 + 按高度着色
└── diagnose/          无界面采集与松开 / 按压对比
```

```sh
pip install -r host/requirements.txt

# 2D 热力图（+/- 调亮度，R 重新校准，Q 退出）
python host/heatmap/heatmap.py --port /dev/ttyUSB0 --rows 12

# MuJoCo 3D 地形（Enter 重新校准，=/- 调灵敏度，关闭窗口退出）
python host/mujoco_viewer/mujoco_viewer.py --port /dev/ttyUSB0 --rows 12
python host/mujoco_viewer/mujoco_viewer.py --demo        # 不接硬件，用模拟数据

# 采集与对比
python host/diagnose/diagnose.py --port /dev/ttyUSB0 --rows 12 --label released --record captures/rel --seconds 10
python host/diagnose/diagnose.py --port /dev/ttyUSB0 --rows 12 --label pressed  --record captures/press --seconds 10
python host/diagnose/diagnose.py --compare captures/rel.npz captures/press.npz
```

启动后前 30 帧不要碰传感器（基线校准）。
