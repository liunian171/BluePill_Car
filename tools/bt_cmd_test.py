#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""BluePill_Car BT04 无线串口命令测试脚本 (开发跟踪 W2-W6 验证工具)

用法:
  python bt_cmd_test.py COM7 --auto          # 跑文本用例全表 (W4 架空状态执行)
  python bt_cmd_test.py COM7 --ping          # 只测链路 (W3)
  python bt_cmd_test.py COM7 --bin           # 二进制帧用例 (W5)
  python bt_cmd_test.py COM7                 # 交互模式, 手动输命令

依赖: pip install pyserial
说明: BT04 透传波特率 9600 (与 main.c 一致); 文本命令行尾自动补 CRLF
"""
import argparse
import struct
import sys
import time

try:
    import serial
except ImportError:
    sys.exit("需要 pyserial: pip install pyserial")

# (命令, 期望应答片段, 用例说明)
CASES_TEXT = [
    ("PING",   "PONG",    "链路心跳"),
    ("STOP",   "STOP OK", "双轮急停"),
    ("M0 60",  "M0:60",   "左轮 60RPM"),
    ("M0 0",   "M0:0",    "左轮 0速停车(应物理刹停)"),
    ("B0",     "BRAKE",   "左轮刹车"),
    ("M1 60",  "M1:60",   "右轮 60RPM"),
    ("M1 0",   "M1:0",    "右轮 0速停车"),
    ("B1",     "BRAKE",   "右轮刹车"),
    ("L 0",    "OFF",     "禁用巡线"),
    ("L 1",    "ON",      "启用巡线"),
    ("GK 6",   "GK:6",    "巡线 Kp 下发"),
    ("GS 30",  "GS:30",   "巡线速度下发"),
    ("STOP",   "STOP OK", "运行中急停(收尾安全)"),
]

# (原始字节, 期望应答片段(None=只看不判), 说明)
CASES_BIN = [
    (bytes([0xAA, 0xF0, 0xFF, 0xFF]), "PONG", "二进制 PING"),
    (bytes([0xAA, 0x01, 0x00]) + struct.pack("<f", 100.0) + bytes([0xFF, 0xFF]),
     "M0:100", "二进制 M0=100RPM"),
    (bytes([0xAA, 0x03, 0x00, 0xFF, 0xFF]), "BRAKE", "二进制 M0 刹车"),
    # W5 健壮性: 非法 id=5, 期望不越界(回应 '?' 或静默), 固件不死机
    (bytes([0xAA, 0x01, 0x05]) + struct.pack("<f", 100.0) + bytes([0xFF, 0xFF]),
     None, "非法 id=5 越界保护"),
    # W5 健壮性: 截断帧(无帧尾), 100ms 后应超时丢弃, 后续命令正常
    (bytes([0xAA, 0x01, 0x00]) + struct.pack("<f", 50.0), None,
     "截断帧(无帧尾)"),
    (bytes([0xAA, 0xF0, 0xFF, 0xFF]), "PONG", "截断帧后恢复(重同步)"),
]


def send_and_read(ser, payload, expect, timeout=2.0):
    """发送 payload(bytes/str), 等待 expect 片段, 返回 (命中?, 时延ms, 原始应答)"""
    if isinstance(payload, str):
        payload = (payload + "\r\n").encode()
    ser.reset_input_buffer()
    t0 = time.perf_counter()
    ser.write(payload)
    buf = b""
    deadline = time.perf_counter() + timeout
    while time.perf_counter() < deadline:
        n = ser.in_waiting
        if n:
            buf += ser.read(n)
            if expect and expect.encode() in buf:
                break
        time.sleep(0.01)
    dt = (time.perf_counter() - t0) * 1000
    return (expect is None) or (expect.encode() in buf), dt, buf.decode(errors="replace").strip()


def run_cases(ser, cases):
    passed = 0
    latencies = []
    for payload, expect, desc in cases:
        ok, dt, resp = send_and_read(ser, payload, expect)
        tag = "PASS" if ok else "FAIL"
        if ok and expect:
            latencies.append(dt)
        lat = f"{dt:6.0f}ms" if expect else "    --"
        print(f"[{tag}] {desc:<28} 应答: {resp[:60]!r}  {lat}")
        if ok:
            passed += 1
    if latencies:
        print(f"\n统计: {passed}/{len(cases)} 通过, 应答时延 "
              f"min/avg/max = {min(latencies):.0f}/"
              f"{sum(latencies)/len(latencies):.0f}/{max(latencies):.0f} ms")
    else:
        print(f"\n统计: {passed}/{len(cases)} 通过")


def main():
    ap = argparse.ArgumentParser(description="BT04 无线串口命令测试")
    ap.add_argument("port", help="蓝牙串口 COM 口, 如 COM7")
    ap.add_argument("--baud", type=int, default=9600)
    ap.add_argument("--ping", action="store_true", help="只测链路心跳")
    ap.add_argument("--auto", action="store_true", help="跑文本用例全表")
    ap.add_argument("--bin", action="store_true", help="跑二进制帧用例")
    args = ap.parse_args()

    ser = serial.Serial(args.port, args.baud, timeout=0.1)
    print(f"已打开 {args.port} @ {args.baud} (BT04 透传)")
    time.sleep(0.5)
    if ser.in_waiting:
        boot = ser.read(ser.in_waiting).decode(errors="replace")
        print(f"开机残留数据: {boot[:80]!r}")

    if args.ping:
        run_cases(ser, [("PING", "PONG", "链路心跳")])
        return

    if args.auto:
        run_cases(ser, CASES_TEXT)
        return

    if args.bin:
        run_cases(ser, CASES_BIN)
        return

    # 交互模式
    print("交互模式: 输入命令回车发送, Ctrl+C 退出")
    while True:
        try:
            cmd = input("> ").strip()
        except (EOFError, KeyboardInterrupt):
            break
        if not cmd:
            continue
        ok, dt, resp = send_and_read(ser, cmd, None, timeout=1.0)
        print(f"  <- {resp!r}  ({dt:.0f}ms)")
    ser.close()


if __name__ == "__main__":
    main()
