# -*- coding: utf-8 -*-
"""
bt_connect.py — BT04 蓝牙串口连接助手 (配对 PIN: 1234, 仅配对时需手动输入一次)

用法:
    envs/default/Scripts/python.exe tools/bt_connect.py [持续秒数] [--pair]

行为:
    1. 扫描所有 COM 口, 逐个探测 PONG (每个口独立子进程+超时, 防蓝牙口挂死)
    2. --pair 模式: 先自动打开系统蓝牙设置窗口,
       用户点选 BT04 → 输入 PIN 1234 → 连接 (仅需一次, 配对持久)
    3. 轮询直到某口应答 PONG, 写入 tools/data/bt_port.txt 供采集脚本默认使用
"""
import subprocess, sys, time, os

PY = sys.executable
PORT_FILE = os.path.join("tools", "data", "bt_port.txt")

PROBE = (
    "import serial,time,sys\n"
    "try:\n"
    "    s=serial.Serial(sys.argv[1],9600,timeout=0.3)\n"
    "    time.sleep(0.4); s.reset_input_buffer()\n"
    "    s.write(b'PING\\n'); time.sleep(0.7)\n"
    "    d=s.read(300).decode(errors='ignore'); s.close()\n"
    "    print('PONG' if 'PONG' in d else 'NO')\n"
    "except Exception:\n"
    "    print('ERR')\n"
)

def com_ports():
    import serial.tools.list_ports as p
    return [x.device for x in p.comports()]

def probe(port):
    try:
        r = subprocess.run([PY, "-u", "-c", PROBE, port],
                           capture_output=True, text=True, timeout=10)
        return "PONG" in (r.stdout or "")
    except subprocess.TimeoutExpired:
        return False

def main():
    args = sys.argv[1:]
    duration = int(args[0]) if args and args[0].isdigit() else 120
    do_pair = "--pair" in args

    if do_pair:
        print("== 配对模式: 已尝试打开系统蓝牙设置窗口 ==")
        print("   请在窗口中: 点选 BT04 → (若要求配对码) 输入 1234 → 连接")
        try:
            os.startfile("ms-settings:bluetooth")   # Windows-only
        except Exception as e:
            print(f"   (自动打开失败: {e}; 请手动打开 蓝牙和其他设备)")
        print(f"== 后台轮询 {duration}s, 连接成功会自动发现端口 ==\n", flush=True)
        time.sleep(2)

    t_end = time.time() + duration
    tried, round_no = set(), 0
    while time.time() < t_end:
        round_no += 1
        for port in com_ports():
            if port in tried:
                continue
            tried.add(port)
            print(f"[r{round_no}] 探测 {port} ...", flush=True)
            if probe(port):
                print(f"\nFOUND: {port} 应答 PONG — BT04 就绪")
                os.makedirs(os.path.dirname(PORT_FILE), exist_ok=True)
                with open(PORT_FILE, "w") as f:
                    f.write(port + "\n")
                print(f"已写入 {PORT_FILE} (采集脚本将默认使用此口)")
                return 0
        time.sleep(3)
    print("TIMEOUT: 未发现应答 PONG 的串口 (检查 BT04 电源/指示灯, 或重新运行 --pair)")
    return 1

if __name__ == "__main__":
    sys.exit(main())
