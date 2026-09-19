# -*- coding: utf-8 -*-
"""
bt_connect.py — BT04 连接助手（兼容入口，实现已并入 link_probe.py）

P3 开关收口后，链路探测只保留一个实现：`tools/link_probe.py`（同时认识
USB CDC 主链与 BT04 备链，逐口隔离探测，避免失联 SPP 口挂死脚本）。

本文件仅作为**兼容壳**存在：文档与既有习惯里的

    python tools/bt_connect.py <秒数> [--pair]

继续可用，等价于

    python tools/link_probe.py --wait <秒数> --bt04 [--pair]

注：等待模式下只要任意串口应答 PONG 即认定发现设备（优先标注 BT04 归类），
`--bt04` 在等待模式下不改变筛选行为，仅作为语义标注传入。
"""
import os
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ARGS = sys.argv[1:]
DURATION = next((a for a in ARGS if a.isdigit()), "120")

CMD = [sys.executable, os.path.join(HERE, "link_probe.py"),
       "--wait", DURATION, "--bt04"]
if "--pair" in ARGS:
    CMD.append("--pair")

sys.exit(subprocess.call(CMD))
