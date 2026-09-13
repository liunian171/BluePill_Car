# -*- coding: utf-8 -*-
"""
step_ident.py — FOPDT 辨识 + SIMC 整定建议 (纯 python, 无第三方依赖)

输入: STEP 命令产出的 CSV (B3 接入契约 v0: t_ms,rate_0E3,rpm_x10)
用法:
    ... python.exe tools/step_ident.py <csv> [csv2 ...]      # 支持多文件(多工作点)

方法 (SIMC/Skogestad):
    K  = Δy∞/Δu            稳态增益 (RPM/千分比)
    θ  = 首响应时刻         滞后 (采样分辨率 50ms, 即 1 帧)
    τ  = 到 63.2% 的时间    时间常数
    未达稳态时按积分环节近似: K' = 末段斜率, SIMC 积分规则
输出:
    K/τ/θ + 拟合 RMSE + SIMC 三档建议 (τc=θ/1.5θ/3θ) 换算为本工程
    增量式 PID (Ts=50ms): Kp=Kc, 每帧积分系数 = Kc·Ts/Ti, 前馈系数 = 1/K
"""
import sys

TS = 0.05  # 控制帧周期 (s)

def load_csv(path):
    ts, us, ys = [], [], []
    for i, line in enumerate(open(path, encoding="utf-8")):
        line = line.strip()
        if not line or i == 0 and line.startswith("t_ms"): continue
        p = line.split(",")
        if len(p) != 3: continue
        try:
            ts.append(int(p[0]) / 1000.0); us.append(int(p[1])); ys.append(int(p[2]) / 10.0)
        except ValueError:
            continue
    return ts, us, ys

def linfit_slope(xs, ys):
    n = len(xs)
    if n < 3: return 0.0
    mx, my = sum(xs) / n, sum(ys) / n
    num = sum((x - mx) * (y - my) for x, y in zip(xs, ys))
    den = sum((x - mx) ** 2 for x in xs)
    return num / den if den else 0.0

def identify(ts, us, ys, label):
    u = us[len(us) // 2]
    t0, y0 = ts[0], ys[0]
    tail_t = [t for t in ts if t >= ts[-1] - 0.5]
    tail_y = ys[len(ys) - len(tail_t):]
    y_inf = sum(tail_y) / len(tail_y)
    slope = linfit_slope(tail_t, tail_y)
    settled = abs(slope) < 0.02 * max(abs(y_inf - y0), 1.0)  # 末段变化 <2%/0.5s 视为稳态

    dy = y_inf - y0
    # θ: 首次超过 5% Δy 的时刻 (20Hz 采样, 分辨率=1 帧)
    theta = t0
    for t, y in zip(ts, ys):
        if abs(y - y0) >= 0.05 * abs(dy): theta = t; break

    report = {"label": label, "u": u, "y0": y0, "y_inf": y_inf,
              "slope_tail": slope, "settled": settled}

    if settled:
        K = dy / u
        y63 = y0 + 0.632 * dy
        tau = ts[-1] - theta
        for t, y in zip(ts, ys):
            if t > theta and ((dy > 0 and y >= y63) or (dy < 0 and y <= y63)):
                tau = t - theta; break
        Kp_int = None
        report.update(K=K, tau=tau, theta=theta)
    else:
        # 积分环节近似 (lag-dominant): K' = 末段斜率 (RPM/s per 千分比)
        Kp_int = slope / u
        report.update(Kp_int=Kp_int, theta=theta)
    report["Kp_int"] = Kp_int

    # 模型拟合 RMSE
    if settled:
        K, tau, th = report["K"], report["tau"], report["theta"]
        se = n = 0
        for t, y in zip(ts, ys):
            m = y0 + K * u * (1.0 - pow(2.718281828, -max(t - th, 0.0) / tau)) if t > th else y0
            se += (y - m) ** 2; n += 1
        report["rmse"] = (se / n) ** 0.5 if n else None
    return report

def simc_table(r, ts_frame=TS):
    rows = []
    if r.get("settled"):
        K, tau, th = r["K"], r["tau"], r["theta"]
        for name, tc in [("快", th), ("平衡", 1.5 * th), ("稳", 3 * th)]:
            Kc = tau / (K * (tc + th))
            Ti = min(tau, 4 * (tc + th))
            ki_frame = Kc * ts_frame / Ti
            rows.append((name, tc, Kc, Ti, ki_frame, 1.0 / K))
    else:
        Kp = r["Kp_int"]; th = r["theta"]
        for name, tc in [("快", th), ("平衡", 1.5 * th), ("稳", 3 * th)]:
            Kc = 1.0 / (Kp * (tc + th))
            Ti = 4 * (tc + th)
            ki_frame = Kc * ts_frame / Ti
            rows.append((name, tc, Kc, Ti, ki_frame, None))
    return rows

def main():
    if len(sys.argv) < 2:
        print(__doc__); return 1
    for path in sys.argv[1:]:
        ts, us, ys = load_csv(path)
        if len(ts) < 10:
            print(f"[{path}] 数据不足 ({len(ts)} 行)"); continue
        r = identify(ts, us, ys, path)
        print(f"\n===== {path} =====")
        print(f"  阶跃量 u = {r['u']} 千分比 | 初速 {r['y0']:.1f} -> 末速 {r['y_inf']:.1f} RPM")
        print(f"  末段斜率 {r['slope_tail']:.2f} RPM/s | 稳态判定: {'已稳态' if r['settled'] else '未达稳态(按积分环节近似)'}")
        if r["settled"]:
            print(f"  K  = {r['K']:.4f} RPM/千分比  (满占空外推 {1000*r['K']:.0f} RPM)")
            print(f"  τ  = {r['tau']*1000:.0f} ms")
        else:
            print(f"  K' = {r['Kp_int']:.4f} (RPM/s)/千分比  [积分近似]")
        print(f"  θ  = {r['theta']*1000:.0f} ms  (采样分辨率 {TS*1000:.0f} ms)")
        if "rmse" in r and r["rmse"] is not None:
            print(f"  FOPDT 拟合 RMSE = {r['rmse']:.2f} RPM")
        print(f"  SIMC 整定建议 (增量式, Ts={TS*1000:.0f}ms):")
        print(f"    档位   τc(s)    Kp      Ti(s)   每帧Ki   前馈(‰/RPM)")
        for name, tc, Kc, Ti, ki, ff in simc_table(r):
            ff_s = f"{ff:.3f}" if ff else "  -  "
            print(f"    {name: <4}  {tc:5.3f}  {Kc:7.4f}  {Ti:5.3f}  {ki:7.5f}  {ff_s}")
        rec = simc_table(r)[1]
        print(f"  → 推荐档位[平衡]: PID 命令值 Kp100={int(rec[2]*100)} Ki100={int(rec[4]*100)} Kd100=0"
              f" | 前馈系数 ff={rec[5]:.3f}" if rec[5] else
              f"  → 推荐档位[平衡]: PID 命令值 Kp100={int(rec[2]*100)} Ki100={int(rec[4]*100)} Kd100=0")
    return 0

if __name__ == "__main__":
    sys.exit(main())
