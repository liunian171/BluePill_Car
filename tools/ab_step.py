# -*- coding: utf-8 -*-
"""
ab_step.py — PID 参数 A/B 对照实验 (REC/DUMP 机内记录版, 对抗无线丢行)

用法:
    envs/default/Scripts/python.exe tools/ab_step.py [port]

三组配置 (轮子悬空):
    A 现状基线 : FF 30  + PID 24 13
    B 只改前馈 : FF 100 + PID 24 13
    C 理论全套 : FF 100 + PID 80 40
每组: 设参 → REC 1 → M0 30 → 3s → M0 0 → REC 0 → DUMP 重放(丢行自动重试补全)
指标 (20Hz 机内采样): 90% 上升时间 / 超调% / ±5% 整定时间 / 稳态误差
"""
import serial, time, os, sys

def port_default():
    if os.path.exists("tools/data/bt_port.txt"):
        return open("tools/data/bt_port.txt").read().strip()
    return "COM15"

def main():
    port = sys.argv[1] if len(sys.argv) > 1 else port_default()
    ser = serial.Serial(port, 9600, timeout=0.1)
    time.sleep(0.5); ser.reset_input_buffer()

    def send(c): ser.write((c + "\n").encode()); time.sleep(0.05)

    def send_expect(cmd, expect, timeout=1.5, retries=4):
        """发送命令并等待应答确认, 弱链路丢字节自动重发"""
        for _ in range(retries):
            ser.reset_input_buffer()
            ser.write((cmd + "\n").encode())
            t_end = time.time() + timeout
            buf = ""
            while time.time() < t_end:
                buf += ser.read(200).decode(errors="ignore")
                if expect in buf:
                    return True
        print(f"  WARN: '{cmd}' 未确认 (期望 {expect!r})")
        return False

    def dump_records(timeout=15.0):
        """DUMP 并按首行条数校验补全, 失败自动重试"""
        expect = None
        rows = {}
        ser.reset_input_buffer(); send("DUMP")   # 初始 DUMP 命令 (此前版本遗漏, 已修)
        t_end = time.time() + timeout
        passes = 0
        while time.time() < t_end:
            for raw in ser.readlines():
                l = raw.decode(errors="ignore").strip()
                if l.startswith("DUMP ") and "END" not in l:
                    expect = int(l.split()[1])
                elif l == "DUMP END":
                    passes += 1
                    print(f"    [dump 第{passes}轮] 应有 {expect}, 已收 {len(rows)}")
                    if expect is not None and len(rows) >= expect:
                        return [(k, rows[k]) for k in sorted(rows)]
                    ser.reset_input_buffer(); send("DUMP")
                    t_end = time.time() + timeout
                elif l.count(",") == 4:
                    try:
                        p = [int(x) for x in l.split(",")]
                        rows[p[0]] = (p[1], p[2])
                    except ValueError: pass
        print(f"    [dump 超时] 应有 {expect}, 已收 {len(rows)}, 共 {passes} 轮")
        return None

    send("PING"); time.sleep(0.8)
    if b"PONG" not in ser.read(300):
        print("FAIL: 链路不通 — 运行 tools/bt_connect.py 重连"); ser.close(); return 1
    send("LA 0"); send("L 0"); time.sleep(0.3); ser.reset_input_buffer()
    print(f"连接 OK ({port}), 三组对照 (轮子应悬空)\n")

    sets = [("A 现状基线(ff0.3)",      30, 24, 13),
            ("B 只改前馈(ff1.0)",      100, 24, 13),
            ("C 理论全套(ff1.0+SIMC)", 100, 80, 40)]
    curves = {}
    for name, ff100, kp, ki in sets:
        ok = send_expect(f"FF {ff100}", "FF:")
        ok &= send_expect(f"P0 {kp} {ki} 0", "PID0:")
        time.sleep(0.3); ser.reset_input_buffer()
        ok &= send_expect("REC 1", "REC:1")
        ok &= send_expect("M0 30", "M0:30RPM")
        if not ok:
            curves[name] = []; continue
        time.sleep(3.0)
        send_expect("M0 0", "M0:0RPM")
        time.sleep(0.6)
        send_expect("REC 0", "REC:0")
        time.sleep(0.3); ser.reset_input_buffer()
        recs = dump_records()
        if recs is None:
            print(f"[{name}] DUMP 补全失败"); curves[name] = []; continue
        step = [(t, g, r) for (t, (g, r)) in recs if g >= 15]
        curves[name] = step
        print(f"[{name}] 机内记录 {len(recs)} 帧, 阶跃段 {len(step)} 点")

    ser.close()
    os.makedirs("tools/data", exist_ok=True)
    print(f"\n{'组':<24}{'上升90%':>10}{'超调%':>8}{'±5%整定':>10}{'稳态误差':>9}")
    for name, step in curves.items():
        if len(step) < 3:
            print(f"{name:<24} 数据不足"); continue
        t0, tgt = step[0][0], step[0][1]
        if tgt <= 0:
            print(f"{name:<24} 目标值异常"); continue
        rel = [(t - t0, r) for t, g, r in step]
        rise = next((t for t, r in rel if r >= 0.9 * tgt), None)
        rise_s = f"{rise}ms" if rise is not None else ">3s"
        over = max((r / tgt - 1) for _, r in rel) * 100
        bad = [t for t, r in rel if abs(r - tgt) > 0.05 * tgt]
        settle = f"{bad[-1]}ms" if bad else "<100ms"
        ess = sum(r for _, r in rel[-5:]) / min(5, len(rel[-5:])) - tgt
        print(f"{name:<24}{rise_s:>10}{over:>8.1f}{settle:>10}{ess:>+9.1f}")
        with open(f"tools/data/ab_{name[0]}.csv", "w") as f:
            f.write("rel_ms,tgt,rpm\n"); f.write("\n".join(f"{t},{tgt},{r}" for t, r in rel) + "\n")
    print("\n曲线已存 tools/data/ab_A/B/C.csv")
    return 0

if __name__ == "__main__":
    sys.exit(main())
