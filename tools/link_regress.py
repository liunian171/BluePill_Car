# -*- coding: utf-8 -*-
"""
link_regress.py — 双链路真机回归（USB 主链 + BT04 备链）

对应 `doc/双链路仲裁设计文档.md` §8 的真机测试点，一条命令跑完常跑项：

    A 段（单链路体检，需 USB）      ：PING / ITEL（IMU 有数 + 200ms 节拍）/ ODOM 20Hz + ATT 10Hz / 突发 40 条零丢
    T1  （双链路并发应答）          ：两侧 PING 均 PONG + `LINK ?` owner 一致
    T2  （owner 切换三语义）        ：BT04 发 LINK UART 接管 → USB 侧确认 → USB 发 LINK USB 抢回
    T5  （BUSY 门控）               ：非 owner 链路发运动命令 → 回 `BUSY:<owner>`
    T7  （STOP 任意链路）           ：备链发 STOP 直达生效
    收尾                            ：`STOP` + owner 还原为 UART（避免下次联调莫名 BUSY）

用法（仓库根目录）:
    python tools/link_regress.py                 # 自动发现两条链路（link_probe 归类）
    python tools/link_regress.py --usb COM17     # 指定 USB 口
    python tools/link_regress.py --usb COM17 --bt COM12
    python tools/link_regress.py --usb-only      # 只跑 A 段（BT04 不在线时）

⚠️ 前置条件（本工具替你处理，但值得知道）：**非 owner 链路的命令会被拒**
（回 `BUSY:<owner>`）——所以 A 段会先取指挥权，收尾再还原；这也是本工具存在的原因：
手工发命令时很容易忘记这条前置条件，把"门控正确工作"误读成"遥测坏了"。

退出码 = 失败项数（0 = 全过）。依赖: pyserial
"""
import argparse
import os
import struct
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import serial  # noqa: E402

try:
    from link_probe import all_ports, classify
except Exception:                                     # 允许单独拷贝本文件使用
    def all_ports():
        import serial.tools.list_ports as lp
        return list(lp.comports())

    def classify(p):
        return "USB" if (p.vid == 0x0483 and p.pid == 0x5740) else "其他"

try:
    sys.stdout.reconfigure(encoding="utf-8")
except Exception:
    pass

RESULT = {"pass": 0, "fail": 0}


def note(tag, cond, detail=""):
    if cond:
        RESULT["pass"] += 1
        print("  [PASS] %-28s %s" % (tag, detail))
    else:
        RESULT["fail"] += 1
        print("  [FAIL] %-28s %s" % (tag, detail))


def openp(port):
    s = serial.Serial(port, 9600, timeout=0.05, write_timeout=2.0)
    time.sleep(0.35)
    s.reset_input_buffer()
    return s


def cmd(s, c, w=0.5):
    """发一条命令并读回一行应答（逐字节读，避免 read(n) 等满超时）"""
    s.write(c)
    time.sleep(w)
    d = b""
    end = time.time() + 0.7
    while time.time() < end:
        x = s.read(1)
        if x:
            d += x
            if d.endswith(b"\n"):
                break
        elif d:
            break
    return d.strip().decode(errors="replace")


def capture(s, seconds, chunk=4096):
    buf = bytearray()
    t0 = time.time()
    while time.time() - t0 < seconds:
        buf += s.read(chunk)
    return buf


def seg_a(usb, port):
    print("=" * 64)
    print("A 段：单链路体检（%s）" % port)
    print("=" * 64)
    r = cmd(usb, b"LINK USB\n")
    own = cmd(usb, b"LINK ?\n")
    if "OWNER:USB" not in own:
        note("取指挥权", False, "LINK USB→%s / owner→%s" % (r, own))
        return
    note("取指挥权", True, "%s / %s" % (r, own))

    note("PING→PONG", "PONG" in cmd(usb, b"PING\n"), "")

    # ITEL：IMU 有数 + 节拍（设计 200ms = 2×IRATE 100ms）
    usb.reset_input_buffer()
    usb.write(b"ITEL 1\n")
    time.sleep(0.5)
    rows = [l for l in capture(usb, 2.5).split(b"\r\n") if l.count(b",") >= 8]
    tk = []
    for l in rows:
        try:
            tk.append(int(l.split(b",")[0]))
        except Exception:
            pass
    d = [tk[i + 1] - tk[i] for i in range(len(tk) - 1)]
    nz = sum(1 for l in rows if l.split(b",")[1:4] != [b"0", b"0", b"0"])
    note("ITEL 有数+节拍 200ms", len(rows) >= 3 and nz > 0 and all(x == 200 for x in d[:3]),
         "行=%d 间隔=%s 陀螺非零=%d" % (len(rows), d[:3], nz))
    cmd(usb, b"ITEL 0\n")

    # ODOM / ATT
    usb.write(b"ODOM 1\n")
    time.sleep(0.5)
    b2 = capture(usb, 10.0)
    ts, n52, i = [], 0, 0
    while i < len(b2) - 19:
        if b2[i] == 0xAA and b2[i + 1] == 0x51:
            ts.append(struct.unpack("<I", bytes(b2[i + 14:i + 18]))[0])
            i += 20
        elif b2[i] == 0xAA and b2[i + 1] == 0x52:
            n52 += 1
            i += 20
        else:
            i += 1
    dd = sorted(set(ts[k + 1] - ts[k] for k in range(len(ts) - 1))) if len(ts) > 1 else []
    hz = 1000.0 / (sum(dd) / len(dd)) if dd else 0.0
    note("ODOM 20Hz + ATT 10Hz", len(ts) >= 180 and n52 >= 90 and abs(hz - 20) < 0.5,
         "0x51=%d帧 ts=%s → %.2fHz; 0x52=%d帧" % (len(ts), dd, hz, n52))
    cmd(usb, b"ODOM 0\n")

    # 突发零丢（容量 239B → 40 条 = 200B 在容量内）
    usb.reset_input_buffer()
    usb.write(b"PING\n" * 40)
    got = capture(usb, 1.5).count(b"PONG")
    note("突发 40 条零丢", got == 40, "PONG %d/40" % got)


def seg_arb(usb, b, port_bt):
    print("=" * 64)
    print("T 段：双链路指挥权（BT04=%s）" % port_bt)
    print("=" * 64)

    note("T1 双链路均应答", "PONG" in cmd(usb, b"PING\n") and "PONG" in cmd(b, b"PING\n", 0.7),
         "两侧 PING→PONG")
    ou, ob = cmd(usb, b"LINK ?\n"), cmd(b, b"LINK ?\n", 0.7)
    note("T1 两链路 owner 一致", bool(ou) and ou == ob, "USB→%s  BT04→%s" % (ou, ob))

    r = cmd(b, b"LINK UART\n", 0.7)
    note("T2 BT04 接管", "LINK:UART OK" in r, r)
    r2 = cmd(usb, b"LINK ?\n")
    note("T2 USB 侧确认", "OWNER:UART" in r2, r2)
    r3 = cmd(usb, b"M0 30\n")
    note("T5 非 owner 被拒", "BUSY" in r3, r3)

    r4 = cmd(usb, b"LINK USB\n")
    note("T2 USB 抢回", "LINK:USB OK" in r4, r4)
    r5 = cmd(b, b"M0 30\n", 0.7)
    note("T5 反向被拒", "BUSY" in r5, r5)

    r6 = cmd(b, b"STOP\n", 0.8)
    note("T7 备链 STOP 直达", "STOP OK" in r6, r6)
    b.close()


def main():
    ap = argparse.ArgumentParser(description="双链路真机回归（USB 主链 + BT04 备链）")
    ap.add_argument("--usb", help="USB CDC 口（默认自动发现 0483:5740）")
    ap.add_argument("--bt", help="BT04 SPP 口（默认自动发现）")
    ap.add_argument("--usb-only", action="store_true", help="只跑 A 段")
    a = ap.parse_args()

    up = a.usb or next((p.device for p in all_ports() if classify(p) == "USB"), None)
    bp = a.bt or next((p.device for p in all_ports() if classify(p) == "BT04"), None)
    if not up:
        print("[!] 未找到 USB CDC 口（0483:5740）—— 烧录后需拔插一次 USB，见 tools/flash_flow.py")
        return 1
    print("USB=%s  BT04=%s" % (up, bp or "（未发现，跳过 T 段）"))

    usb = openp(up)
    try:
        seg_a(usb, up)
        if bp and not a.usb_only:
            bt = None
            try:
                bt = openp(bp)
                seg_arb(usb, bt, bp)
            except Exception as e:
                note("T 段", False, "备链异常: %s" % str(e)[:60])
            finally:
                if bt:
                    bt.close()
        # 收尾：急停 + owner 还原（否则下次联调会莫名 BUSY）
        cmd(usb, b"STOP\n", 0.6)
        cmd(usb, b"LINK UART\n", 0.5)
        print("收尾: STOP + owner→%s" % cmd(usb, b"LINK ?\n"))
    finally:
        usb.close()

    print("-" * 64)
    print("小结: %d PASS / %d FAIL" % (RESULT["pass"], RESULT["fail"]))
    return RESULT["fail"]


if __name__ == "__main__":
    sys.exit(main())
