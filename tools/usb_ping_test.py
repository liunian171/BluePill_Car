# -*- coding: utf-8 -*-
"""
usb_ping_test.py — USB CDC 独立链路测试 (BluePill_Car, USB_ECHO_TEST 固件配套)

前置:
  1. 固件已用 -DUSB_ECHO_TEST=1 配置编译并烧录 (flash.bat)
  2. 烧录后【拔插一次 USB】再跑本脚本 (F103 SWD 复位后 USB 不重枚举)
  3. 设备管理器出现 "USB 串行设备 (COMx)" (VID=0x0483 PID=0x5740)

用法:
  python usb_ping_test.py            # 自动扫描 ST CDC 口
  python usb_ping_test.py COM17      # 指定端口

测试项:
  A. PING -> PONG 往返 x10 (协议语义 + 延迟统计)
  B. 随机数据回显 x20 (64~256B, 按 <=60B 分块往返, 规避 CDC 单包 64B 上限
     与裸回显的 USBD_BUSY 丢包; 本项目协议帧 <=32B, 分块测试即真实工况)

依赖: pyserial (pip install pyserial)
"""
import sys
import time
import random
import serial
import serial.tools.list_ports

ST_CDC_VID = 0x0483
ST_CDC_PID = 0x5740
TIMEOUT_S = 0.5
CHUNK_MAX = 60   # USB FS bulk 单包上限 64B, 留余量


def find_port():
    """优先找 ST CDC (VID:PID=0483:5740), 找不到列出所有串口让用户挑"""
    for p in serial.tools.list_ports.comports():
        if p.vid == ST_CDC_VID and p.pid == ST_CDC_PID:
            return p.device, "ST CDC 自动识别"
    print("[!] 未找到 VID=0483 PID=5740 的 CDC 设备")
    print("[!] 当前所有串口:")
    for p in serial.tools.list_ports.comports():
        print("   ", p.device, "-", p.description)
    print("[!] 确认: ① 固件是 USB_ECHO_TEST 版? ② 烧录后拔插过 USB? ③ 设备管理器有 COMx?")
    sys.exit(1)


def read_exact(ser, n):
    """精确收 n 字节 (超时返回已收到的部分)"""
    buf = bytearray()
    while len(buf) < n:
        chunk = ser.read(n - len(buf))
        if not chunk:
            break
        buf.extend(chunk)
    return bytes(buf)


def main():
    if len(sys.argv) > 1:
        port, how = sys.argv[1], "命令行指定"
    else:
        port, how = find_port()

    print("=" * 56)
    print("USB CDC 独立链路测试   端口: %s (%s)" % (port, how))
    print("=" * 56)

    with serial.Serial(port, timeout=TIMEOUT_S, write_timeout=TIMEOUT_S) as ser:
        ser.reset_input_buffer()
        ser.reset_output_buffer()

        # ---- 测试 A: PING -> PONG x10 ----
        ok, lat = 0, []
        for i in range(10):
            ser.reset_input_buffer()
            t0 = time.perf_counter()
            ser.write(b"PING\r\n")
            rsp = read_exact(ser, 6)
            dt = (time.perf_counter() - t0) * 1000
            if rsp == b"PONG\r\n":
                ok += 1
                lat.append(dt)
            else:
                print("  [%02d] FAIL  收到: %r" % (i + 1, rsp))
        print("[A] PING->PONG: %d/10 通过" % ok)
        if lat:
            lat.sort()
            print("    往返延迟 min/中位/max = %.1f / %.1f / %.1f ms"
                  % (lat[0], lat[len(lat) // 2], lat[-1]))

        # ---- 测试 B: 随机回显 x20 (分块 <=60B 往返) ----
        ok_b, lost = 0, 0
        random.seed(20260919)
        for i in range(20):
            n = random.randint(64, 256)
            payload = bytes(random.getrandbits(8) for _ in range(n))
            good = True
            for off in range(0, n, CHUNK_MAX):
                chunk = payload[off:off + CHUNK_MAX]
                ser.reset_input_buffer()
                ser.write(chunk)
                echo = read_exact(ser, len(chunk))
                if echo != chunk:
                    good = False
                    lost += len(chunk) - len(echo)
                    print("  [%02d] 块偏移 %4d: 发 %dB 收 %dB"
                          % (i + 1, off, len(chunk), len(echo)))
            if good:
                ok_b += 1
        print("[B] 随机回显:   %d/20 通过 (分块 %dB, 共丢 %d 字节)"
              % (ok_b, CHUNK_MAX, lost))

        # ---- 总结 ----
        total = ok + ok_b
        print("-" * 56)
        if total == 30:
            print("结论: PASS — USB CDC 链路可用, 硬件/固件验证通过")
        elif total >= 25:
            print("结论: MARGINAL — 链路通但有丢包, 检查 USB 线/供电/R10 上拉")
        else:
            print("结论: FAIL — 链路基本不通, 走硬件排查 (D+/PA12, D-/PA11, R10=1.5k)")


if __name__ == "__main__":
    main()
