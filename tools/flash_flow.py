# -*- coding: utf-8 -*-
"""
flash_flow.py — 烧录 + 免拔插协调 + 自动验证（F103 软复位不重枚举的对策）

背景（2026-09-19 实测结论，详 doc/调试总结.md §27）:
  STM32F103 经 SWD 软复位（-rst）后 **USB 不再枚举**：主机侧保留旧设备节点，
  打开串口报 "设备无响应(31)/设备描述符请求失败(43)"。固件侧尝试（boot 时保持
  USB 断电窗口 250ms 制造"拔出"事件）**在本板无效** → 只能靠物理拔插（总线复位）。
  → 本脚本把"烧录 → 等人工拔插 → 自动验证"串成一条命令，省掉来回确认。

用法:
  python tools/flash_flow.py --flash            # 编译 + 烧录 + 等端口 + PING 验证
  python tools/flash_flow.py --verify           # 跳过烧录，只等端口 + 验证（拔插后复核）
  python tools/flash_flow.py --flash --wait 600 # 自定义等待上限（秒，默认 300）

判定:
  端口重现 + PING->PONG  ⇒ 退出码 0，并打印当前 owner（LINK ?）与警告计数（OLED 页 7 人眼项）
  超时                   ⇒ 退出码 1，提示手动检查接线/供电

依赖: pyserial（上位机侧工具同款）
"""
import argparse
import subprocess
import sys
import time

import serial
import serial.tools.list_ports

ST_VID = 0x0483
ST_PID = 0x5740          # ST CDC（Virtual COM Port）
BAUD = 9600
DEFAULT_WAIT = 300       # 秒


def find_port():
    """返回当前存在的 ST CDC 端口名（无则 None）"""
    for p in serial.tools.list_ports.comports():
        if p.vid == ST_VID and p.pid == ST_PID:
            return p.device
    return None


def probe(port):
    """打开端口并发 PING；成功返回 (True, 应答串)，失败返回 (False, 原因)"""
    try:
        s = serial.Serial(port, BAUD, timeout=0.5, write_timeout=2.0)
    except Exception as e:
        return False, str(e)[:70]
    try:
        time.sleep(0.3)
        s.reset_input_buffer()
        s.write(b"PING\n")
        time.sleep(0.6)
        r = s.read(100)
        if b"PONG" not in r:
            return False, "无 PONG（设备无响应，需拔插 USB）"
        # 顺带取仲裁状态与 ringbuf 溢出/OLED 失败计数不可远程读，仅报 owner
        s.reset_input_buffer()
        s.write(b"LINK ?\n")
        time.sleep(0.4)
        own = s.read(60).strip().decode(errors="replace")
        return True, "PONG  |  " + own
    finally:
        s.close()


def wait_and_verify(port_hint, wait_s):
    """轮询等待端口重现并验证；期间周期打印进度（便于后台运行看日志）"""
    t0 = time.time()
    last_note = 0.0
    while True:
        el = time.time() - t0
        if el > wait_s:
            print("[超时 %.0fs] 端口仍未就绪 —— 请检查: ① USB 线是否插回 ② 板子供电 ③ 是否需换口" % wait_s)
            return 1
        port = port_hint or find_port()
        if port:
            ok, info = probe(port)
            if ok:
                print("[OK] %.1fs 后 %s 通信恢复 ✅  %s" % (el, port, info))
                return 0
            if el - last_note > 10:
                print("  [%.0fs] 发现端口 %s 但 %s —— 请拔插一次 USB（等总线复位）" % (el, port, info))
                last_note = el
        elif el - last_note > 10:
            print("  [%.0fs] 等待 ST CDC 端口出现…（请拔插一次 USB）" % el)
            last_note = el
        time.sleep(0.5)


def run(cmd, **kw):
    print("$ " + " ".join(cmd))
    return subprocess.call(cmd, **kw)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--flash", action="store_true", help="先编译+烧录再等待验证")
    ap.add_argument("--verify", action="store_true", help="只等待+验证（跳过烧录）")
    ap.add_argument("--elf", default="build/Release/BluePill_Car.elf")
    ap.add_argument("--preset", default="Release")
    ap.add_argument("--port", default=None, help="指定端口（默认按 VID:PID 自动发现）")
    ap.add_argument("--wait", type=int, default=DEFAULT_WAIT, help="等待端口重现的上限秒数")
    a = ap.parse_args()

    if not (a.flash or a.verify):
        ap.print_help()
        return 2

    if a.flash:
        if run(["cmake", "--build", "build/" + a.preset]) != 0:
            print("[失败] 编译不通过")
            return 1
        if run(["STM32_Programmer_CLI.exe", "-c", "port=SWD",
                "-w", a.elf, "0x08000000", "-rst"]) != 0:
            print("[失败] 烧录不通过（ST-LINK 未接？）")
            return 1
        print()
        print("=" * 66)
        print("  固件已烧录（软复位）。F103 软复位后不重枚举 —— 现在请【拔插一次 USB】")
        print("  本脚本会自动等待端口重现并验证，无需再手动确认。")
        print("=" * 66)

    return wait_and_verify(a.port, a.wait)


if __name__ == "__main__":
    sys.exit(main())
