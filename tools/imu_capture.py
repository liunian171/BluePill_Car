# -*- coding: utf-8 -*-
"""
imu_capture.py — IMU 姿态遥测采集 (ITEL CSV, B3 契约同风格)

用法:
    envs/default/Scripts/python.exe tools/imu_capture.py [port] [seconds] [outfile]
    port 缺省读 tools/data/bt_port.txt; seconds 缺省 10

ITEL 行格式 (固件 2026-09-15, 2 分频默认 5Hz):
    tick,gx10,gy10,gz10,roll10,pitch10,yaw10,stable,driftz100
    角速度 °/s×10, 欧拉角 °×10, stable=静止标志, driftz=漂移补偿°/s×100
"""
import sys, time, os, serial

def port_default():
    f = os.path.join("tools", "data", "bt_port.txt")
    return open(f).read().strip() if os.path.exists(f) else "COM15"

def main():
    port = sys.argv[1] if len(sys.argv) > 1 else port_default()
    secs = int(sys.argv[2]) if len(sys.argv) > 2 else 10
    outfile = sys.argv[3] if len(sys.argv) > 3 else "tools/data/imu_capture.csv"
    ser = serial.Serial(port, 9600, timeout=0.1)
    time.sleep(0.5); ser.reset_input_buffer()

    def send(c): ser.write((c + "\n").encode()); time.sleep(0.05)

    send("PING"); time.sleep(0.8)
    if "PONG" not in ser.read(300).decode(errors="ignore"):
        print("FAIL: 无 PONG, 链路不通"); ser.close(); return 1
    print("连接 OK (PONG)")

    send("ITEL 1")
    rows, t_end = [], time.time() + secs
    print(f"采集 {secs}s ...")
    while time.time() < t_end:
        for raw in ser.readlines():
            l = raw.decode(errors="ignore").strip()
            p = l.split(",")
            if len(p) == 9 and l[0].isdigit():
                rows.append(l)
    send("ITEL 0")
    ser.close()

    os.makedirs(os.path.dirname(outfile) or ".", exist_ok=True)
    with open(outfile, "w") as f:
        f.write("tick,gx10,gy10,gz10,roll10,pitch10,yaw10,stable,driftz100\n")
        f.write("\n".join(rows) + "\n")
    print(f"采集 {len(rows)} 行 -> {outfile}")
    return 0 if rows else 1

if __name__ == "__main__":
    sys.exit(main())
