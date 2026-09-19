# -*- coding: utf-8 -*-
"""
link_probe.py — 双链路统一探测（P3 开关收口）

取代原先"各覆盖一半"的两个脚本（usb_ping_test.py / bt_connect.py）：
本脚本同时认识 USB CDC 主链与 BT04 SPP 备链，一条命令给出双链路体检报告。

用法（在仓库根目录执行）:
    python tools/link_probe.py                  # 扫全部串口 → 归类 → 逐口隔离探测 + 报 owner
    python tools/link_probe.py --port COM17     # 只测指定口
    python tools/link_probe.py --wait 120       # 轮询等待直到某口应答（BT04 连接场景）
    python tools/link_probe.py --pair           # Windows: 打开蓝牙设置做配对引导（PIN 1234）
    python tools/link_probe.py --echo           # 追加 USB 回显用例（需 USB_ECHO_TEST 固件）
    python tools/link_probe.py --bt04           # 等价旧 bt_connect 默认行为（只关心 BT04）

行为要点:
  · **逐口隔离探测**：每个口在独立子进程里探测并带超时 —— 失联的 SPP 口会让
    pyserial 的 write 永久阻塞（蓝牙栈等连接），同进程探测会连带挂死整个脚本。
  · 端口归类：VID:PID = 0483:5740 → USB CDC；hwid 含 BTHENUM / 本机 BT04 MAC
    (98DA20045F4F) → BT04；其余 → 其他。
  · 探测内容：`PING` → `PONG`（含首字节延迟）+ `LINK ?` → `OWNER:<链路>`。
  · 存档：结果写 `tools/data/link_probe.txt`；BT04 应答时另写 `tools/data/bt_port.txt`
    （下游采集脚本沿用该文件）。

依赖: pyserial
"""
import argparse
import os
import subprocess
import sys
import time

try:
    sys.stdout.reconfigure(encoding="utf-8")   # 经管道/日志重定向时不至于变乱码
except Exception:
    pass

ST_VID, ST_PID = 0x0483, 0x5740
BT04_MAC = "98DA20045F4F"          # BT04-A 经典 SPP（用户提供，见 .workbuddy/memory/MEMORY.md）
PORT_FILE = os.path.join("tools", "data", "bt_port.txt")
RESULT_FILE = os.path.join("tools", "data", "link_probe.txt")
PING_CMD = b"PING\n"

# ---- 子进程探测体（隔离 + 超时；输出一行结果给父进程） ----
CHILD = r'''
import sys, time
import serial
port = sys.argv[1]
s = None
try:
    # timeout 取小值: pyserial 的 read(n) 会一直等到"n 字节或超时"，
    # 所以延迟测量必须逐字节读（read(1)），否则量到的是超时值而非真实延迟
    s = serial.Serial(port, 9600, timeout=0.05, write_timeout=2.0)
except Exception as e:
    print("OPENFAIL|%s" % type(e).__name__)
    raise SystemExit(0)
try:
    time.sleep(0.3)
    s.reset_input_buffer()
    t0 = time.perf_counter()
    s.write(b"PING\n")
    d = b""
    first = 0.0
    end = time.time() + 1.5
    while time.time() < end and b"PONG" not in d:
        c = s.read(1)
        if c:
            if not d:
                first = (time.perf_counter() - t0) * 1000.0
            d += c
    if b"PONG" not in d:
        print("NOPONG|%s" % d[:24].hex())
        raise SystemExit(0)
    dt = first if first else (time.perf_counter() - t0) * 1000.0
    own = "-"
    try:
        s.reset_input_buffer()
        s.write(b"LINK ?\n")
        o = b""
        end = time.time() + 1.2
        while time.time() < end:
            c = s.read(1)
            if c:
                o += c
                if b"OWNER:" in o and b"\n" in o:      # 读到整行才算数
                    break
            elif b"OWNER:" in o:
                break                                   # 只剩行尾没到, 不必再等
        for line in o.decode(errors="ignore").splitlines():
            if "OWNER:" in line:
                own = line.strip()
                break
    except Exception:
        pass
    print("OK|%.1f|%s" % (dt, own))
except Exception as e:
    print("ERR|%s" % type(e).__name__)
finally:
    try:
        if s:
            s.close()
    except Exception:
        pass
'''


def all_ports():
    import serial.tools.list_ports as lp
    return list(lp.comports())


def classify(p):
    """USB = 本项目 USB CDC 主链; BT04 = 本项目 BT04（按 MAC 精确识别）;
    蓝牙? = 其它蓝牙虚拟串口（配对过的手机/耳机等，多半不通，只作参考）"""
    if p.vid == ST_VID and p.pid == ST_PID:
        return "USB"
    hw = ((p.hwid or "") + " " + (p.description or "")).upper().replace(":", "")
    if BT04_MAC in hw:
        return "BT04"
    if "BTHENUM" in hw or "BLUETOOTH" in hw:
        return "蓝牙?"
    return "其他"


# 探测超时（秒）：非本车蓝牙口挂死概率高，给短超时；两条目标链路给足
PROBE_TIMEOUT = {"USB": 8, "BT04": 12, "蓝牙?": 6, "其他": 6}


def probe_port(port, timeout=12):
    """子进程隔离探测 → (状态, 延迟ms 或 备注, owner)"""
    try:
        r = subprocess.run([sys.executable, "-u", "-c", CHILD, port],
                           capture_output=True, text=True, timeout=timeout,
                           encoding="utf-8", errors="replace")
    except subprocess.TimeoutExpired:
        return ("HANG", "-", "-")     # 口被占/链路挂死（SPP 常见）
    lines = [l for l in (r.stdout or "").strip().splitlines() if "|" in l]
    if not lines:
        return ("EMPTY", "-", "-")
    f = lines[-1].split("|")
    if f[0] == "OK":
        return ("PONG", f[1], f[2] if len(f) > 2 else "-")
    if f[0] == "NOPONG":
        return ("无应答", f[1] if len(f) > 1 else "-", "-")
    return (f[0], f[1] if len(f) > 1 else "-", "-")


def archive(lines, bt_port=None):
    os.makedirs(os.path.dirname(RESULT_FILE), exist_ok=True)
    with open(RESULT_FILE, "w", encoding="utf-8") as f:
        f.write("link_probe @ %s\n" % time.strftime("%Y-%m-%d %H:%M:%S"))
        for l in lines:
            f.write(l + "\n")
    print("已写 %s" % RESULT_FILE)
    if bt_port:
        with open(PORT_FILE, "w", encoding="utf-8") as f:
            f.write(bt_port + "\n")
        print("已写 %s (下游采集脚本默认端口)" % PORT_FILE)


def scan(only_port=None, bt_only=False, echo=False):
    ports = all_ports()
    if only_port:
        ports = [p for p in ports if p.device.upper() == only_port.upper()]
        if not ports:
            print("[!] 指定端口 %s 不在当前串口列表（拔插后重试）" % only_port)
            return 1
    rows, ok_any, bt_hit = [], False, None
    print("=" * 62)
    print("链路探测  (USB CDC = 0483:5740 主链 / BT04 SPP = 备链)")
    print("=" * 62)
    for p in ports:
        kind = classify(p)
        if kind == "其他":
            continue                      # 与本项目两条链路无关
        if bt_only and kind != "BT04":
            continue
        st, info, own = probe_port(p.device, PROBE_TIMEOUT.get(kind, 8))
        if st == "PONG":
            ok_any = True
            if kind == "BT04":
                bt_hit = p.device
            desc = "PONG  首字节 %sms  %s" % (info, own)
        elif st == "无应答":
            desc = "无应答 (口在, 但固件不回 PONG: 未上电 / 不是本车 / 链路未通)"
        elif st == "HANG":
            desc = "探测超时 (口被其它软件占用, 或 SPP 链路挂死)"
        elif st == "OPENFAIL":
            desc = "打不开 (%s — 被占用 / 端口失效; USB 口见 31=需拔插复位枚举)" % info
        else:
            desc = "%s %s" % (st, info)
        line = "%-8s %-8s %s" % (p.device, kind, desc)
        print(line)
        rows.append(line)
    print("-" * 62)
    if not rows:
        print("未发现 USB CDC / BT04 端口 —— 检查供电与接线（烧录后 USB 需拔插一次）")
    elif not ok_any:
        print("两条链路均未应答 —— 见上方逐口原因")
    if echo:
        usb = [p for p in all_ports() if classify(p) == "USB"]
        if not usb:
            print("[echo] 未找到 USB CDC 口，跳过")
        else:
            run_echo(usb[0].device)
    archive(rows, bt_hit)
    return 0 if ok_any else 1


def run_echo(port):
    """USB 回显用例（需 -DUSB_ECHO_TEST=1 固件；正常固件只应通过 A 段 PING）"""
    import random
    import serial
    print("[echo] %s — 回显用例（需 USB_ECHO_TEST 固件，正常固件会在 B 段全 FAIL）" % port)
    with serial.Serial(port, 9600, timeout=0.5, write_timeout=2.0) as s:
        s.reset_input_buffer()
        ok, lat = 0, []
        for _ in range(10):
            s.reset_input_buffer()
            t0 = time.perf_counter()
            s.write(b"PING\r\n")
            r = s.read(6)
            if r == b"PONG\r\n":
                ok += 1
                lat.append((time.perf_counter() - t0) * 1000)
        lat.sort()
        print("[A] PING->PONG %d/10" % ok, end="")
        if lat:
            print("  延迟 min/中位/max = %.1f / %.1f / %.1f ms"
                  % (lat[0], lat[len(lat) // 2], lat[-1]))
        else:
            print()
        okb, lost = 0, 0
        random.seed(20260919)
        for _ in range(20):
            n = random.randint(64, 256)
            payload = bytes(random.getrandbits(8) for _ in range(n))
            good = True
            for off in range(0, n, 60):
                chunk = payload[off:off + 60]
                s.reset_input_buffer()
                s.write(chunk)
                got = bytearray()
                while len(got) < len(chunk):
                    c = s.read(len(chunk) - len(got))
                    if not c:
                        break
                    got.extend(c)
                if bytes(got) != chunk:
                    good = False
                    lost += len(chunk) - len(got)
            if good:
                okb += 1
        print("[B] 随机回显 %d/20（分块 60B，丢 %d 字节）" % (okb, lost))


def wait_for_pong(seconds, pair=False):
    """轮询等待某口应答 PONG（旧 bt_connect 语义）；成功写 bt_port.txt"""
    if pair:
        print("== 配对引导：请在打开的窗口点选 BT04 →（若要求配对码）输入 1234 → 连接 ==")
        try:
            os.startfile("ms-settings:bluetooth")
        except Exception as e:
            print("   (自动打开蓝牙设置失败: %s，请手动打开)" % e)
        time.sleep(2)
    t_end = time.time() + seconds
    tried = set()
    rnd = 0
    print("== 轮询等待 PONG（%ds，端口随对配变化，出现即自动发现）==" % seconds, flush=True)
    while time.time() < t_end:
        rnd += 1
        for p in all_ports():
            if p.device in tried:
                continue
            tried.add(p.device)
            st, info, own = probe_port(p.device)
            if st == "PONG":
                print("\nFOUND: %s 应答 PONG — %s 链路就绪（%s）"
                      % (p.device, classify(p), own))
                if classify(p) == "BT04":
                    archive(["FOUND %s BT04 %s" % (p.device, own)], p.device)
                else:
                    archive(["FOUND %s USB %s" % (p.device, own)])
                return 0
            print("  [r%d] %s %s → %s" % (rnd, p.device, classify(p), st), flush=True)
        time.sleep(3)
    print("TIMEOUT: 未发现应答 PONG 的口（查供电/BT04 指示灯/是否需重连）")
    return 1


def main():
    ap = argparse.ArgumentParser(description="双链路统一探测（USB CDC + BT04 SPP）")
    ap.add_argument("--port", help="只测指定串口（如 COM17）")
    ap.add_argument("--wait", type=int, metavar="SEC", help="轮询等待直到某口应答 PONG")
    ap.add_argument("--pair", action="store_true", help="Windows: 打开蓝牙设置做配对引导")
    ap.add_argument("--echo", action="store_true", help="追加 USB 回显用例（需 USB_ECHO_TEST 固件）")
    ap.add_argument("--bt04", action="store_true", help="只关心 BT04 链路（等价旧 bt_connect）")
    a = ap.parse_args()
    if a.wait is not None:
        return wait_for_pong(a.wait, a.pair)
    return scan(a.port, a.bt04, a.echo)


if __name__ == "__main__":
    sys.exit(main())
