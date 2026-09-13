# -*- coding: utf-8 -*-
"""
tune_smoke.py — 调参工具链冒烟测试 (B3 接入契约 v0 的 PC 端首用例)

用法:
    envs/default/Scripts/python.exe tools/tune_smoke.py [port]
    port 缺省 COM15 (BT04 SPP, 9600-8N1)

流程:
    1. 连接 → PING 握手 (预期 PONG)
    2. TEL 1 → 采集 2s 闭环遥测 → TEL 0        (行格式: tick,tgt0,rpm0,tgt1,rpm1)
    3. STEP 0 300 2000 → 采集阶跃全程           (行格式: t_ms,rate_0E3,rpm_x10)
    4. 数据存 tools/data/step_m0_300.csv, 摘要打印
"""
import sys, time, serial

PORT = sys.argv[1] if len(sys.argv) > 1 else "COM15"
BAUD = 9600

def open_port():
    ser = serial.Serial(PORT, BAUD, timeout=0.1)
    time.sleep(0.5)
    ser.reset_input_buffer()
    return ser

def send(ser, cmd):
    ser.write((cmd + "\n").encode())
    time.sleep(0.05)

def read_lines(ser, seconds):
    """读 N 秒, 返回非空行列表"""
    lines, t_end = [], time.time() + seconds
    while time.time() < t_end:
        chunk = ser.readlines()
        for raw in chunk:
            s = raw.decode(errors="ignore").strip()
            if s:
                lines.append(s)
    return lines

def main():
    ser = open_port()
    print(f"[1] 已连接 {PORT}@{BAUD}, PING 握手...")
    send(ser, "PING")
    pong = read_lines(ser, 1.0)
    ok = any("PONG" in l for l in pong)
    print(f"    PING -> {pong}  {'OK' if ok else 'FAIL'}")
    if not ok:
        ser.close(); return 1

    print("[2] TEL 1 采集 2s 闭环遥测...")
    send(ser, "TEL 1")
    tel = [l for l in read_lines(ser, 2.0) if l.count(",") == 4]
    send(ser, "TEL 0")
    time.sleep(0.3)
    ser.reset_input_buffer()
    print(f"    收到 {len(tel)} 行 (预期 ~20)")
    if tel:
        print(f"    首行: {tel[0]}")
        print(f"    末行: {tel[-1]}")

    print("[3] STEP 0 300 2000 (轮子应悬空!)...")
    send(ser, "STEP 0 300 2000")
    step = []
    t_end = time.time() + 4.0
    ended = False
    while time.time() < t_end and not ended:
        for l in read_lines(ser, 0.2):
            step.append(l)
            if "STEP END" in l:
                ended = True
    data = [l for l in step if l.count(",") == 2 and not l.startswith("STEP")]
    print(f"    数据 {len(data)} 行 (预期 ~40), 结束标记: {'收到' if ended else '未收到!'}")
    if data:
        print(f"    首行: {data[0]}")
        print(f"    末行: {data[-1]}")
        import os
        os.makedirs("tools/data", exist_ok=True)
        with open("tools/data/step_m0_300.csv", "w") as f:
            f.write("t_ms,rate_0E3,rpm_x10\n")
            f.write("\n".join(data) + "\n")
        print("    已存 tools/data/step_m0_300.csv")
        rpm = [int(l.split(",")[2]) / 10.0 for l in data]
        print(f"    转速: 首={rpm[0]} 末={rpm[-1]} 峰={max(rpm)} (单位 RPM)")
    ser.close()
    print("[4] 完成")
    return 0

if __name__ == "__main__":
    sys.exit(main())
