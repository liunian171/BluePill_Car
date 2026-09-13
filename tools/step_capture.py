# -*- coding: utf-8 -*-
"""
step_capture.py — 开环阶跃数据采集 (B3 接入契约 v0)

用法:
    envs/default/Scripts/python.exe tools/step_capture.py <port> <motor> <rate0E3> <ms> <outfile>
例:
    ... step_capture.py COM15 0 300 5000 tools/data/step_m0_300_5s.csv

流程: 连接 → PING 握手 → 停自动巡线 → STEP 采集至 STEP END → 存 CSV
"""
import sys, time, os, serial

def main():
    port, motor, rate, dur, outfile = (sys.argv[1], int(sys.argv[2]), int(sys.argv[3]),
                                       int(sys.argv[4]), sys.argv[5])
    div = int(sys.argv[6]) if len(sys.argv) > 6 else 1   # 遥测分频: 2=10Hz (弱链路防丢行)
    ser = serial.Serial(port, 9600, timeout=0.1)
    time.sleep(0.5); ser.reset_input_buffer()

    def send(c): ser.write((c + "\n").encode()); time.sleep(0.05)

    send("PING"); time.sleep(0.8)
    banner = ser.read(300).decode(errors="ignore")
    if "PONG" not in banner:
        print("FAIL: 无 PONG, 链路不通"); ser.close(); return 1
    print("连接 OK (PONG)")

    send("LA 0"); send("L 0"); time.sleep(0.3); ser.reset_input_buffer()

    send(f"STEP {motor} {rate} {dur} {div}")
    data, ended, t_end = [], False, time.time() + dur / 1000.0 + 5.0
    while time.time() < t_end and not ended:
        for raw in ser.readlines():
            l = raw.decode(errors="ignore").strip()
            if not l: continue
            if "STEP END" in l: ended = True
            elif l.count(",") == 2 and l[0].isdigit(): data.append(l)
    ser.close()

    if not ended: print("WARN: 未收到 STEP END (超时)")
    os.makedirs(os.path.dirname(outfile) or ".", exist_ok=True)
    with open(outfile, "w") as f:
        f.write("t_ms,rate_0E3,rpm_x10\n"); f.write("\n".join(data) + "\n")
    print(f"采集 {len(data)} 行 -> {outfile}")
    return 0 if ended else 1

if __name__ == "__main__":
    sys.exit(main())
