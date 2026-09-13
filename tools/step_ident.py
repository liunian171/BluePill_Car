# -*- coding: utf-8 -*-
"""
step_ident.py — FOPDT 辨识 + SIMC 整定建议 (纯 python, 无第三方依赖)

输入: STEP 命令产出的 CSV (B3 接入契约 v0: t_ms,rate_0E3,rpm_x10)
用法:
    ... python.exe tools/step_ident.py <csv> [csv2 ...]      # 多文件 = 多工作点

处理:
    1. 剔除 STEP END 后的滑行帧 (帧距 > 1.8x 中位帧距)
    2. K  = 稳态中位数 / 阶跃量;  τ = ln 拟合 (10%~80% 上升段);  θ = 首响应时刻
    3. 响应快于采样分辨率时给出 τ 上界结论 (首帧占比 > 63% 判定)
    4. SIMC 三档 (τc = θ/1.5θ/3θ) + 固件域换算:
       固件 PID 输出 1 单位 = 319 分之 1000 千分比 (max_rpm=319), 增益需除以 3.1348
       前馈系数 = 1/K (千分比每 RPM) -> 固件域 = 1/K * 319/1000 (乘在 target RPM 上)
"""
import sys

TS = 0.05      # 控制帧周期 (s)
MAX_RPM = 319  # 固件 map 常量: 1000‰ ↔ max_rpm

def load_csv(path):
    ts, us, ys = [], [], []
    for i, line in enumerate(open(path, encoding="utf-8")):
        line = line.strip()
        if not line or (i == 0 and line.startswith("t_ms")): continue
        p = line.split(",")
        if len(p) != 3: continue
        try:
            ts.append(int(p[0]) / 1000.0); us.append(int(p[1])); ys.append(int(p[2]) / 10.0)
        except ValueError:
            continue
    return ts, us, ys

def median(v):
    s = sorted(v); n = len(s)
    return s[n // 2] if n % 2 else 0.5 * (s[n // 2 - 1] + s[n // 2])

def trim_coast(ts, ys):
    """剔除末尾滑行帧: END 后 dt 变长, 帧距 > 1.8x 中位帧距即截断"""
    dts = [ts[i + 1] - ts[i] for i in range(len(ts) - 1)]
    med = median(dts)
    n = len(ts)
    while n > 3 and (ts[n - 1] - ts[n - 2]) > 1.8 * med:
        n -= 1
    return ts[:n], ys[:n]

def identify(ts, us, ys):
    u = median(us)
    ts, ys = trim_coast(ts, ys)
    # 基线 = 0: STEP 从静止开始 (capture 脚本先行刹停), 首帧采样可能已是响应中段
    y0 = 0.0
    tail = [y for t, y in zip(ts, ys) if t >= ts[-1] - 1.0]
    y_inf = median(tail)
    dy = y_inf - y0

    # θ: 首次超过 10% Δy (受采样分辨率限制, 实为上界)
    theta = ts[0]
    for t, y in zip(ts, ys):
        if abs(y - y0) >= 0.10 * abs(dy): theta = t; break

    # τ: 对 10%~80% 上升段做 ln 线性拟合; 仅 1 点时用单点估计 -t/ln(1-frac)
    tau, fit_n = None, 0
    xs, zs = [], []
    for t, y in zip(ts, ys):
        frac = (y - y0) / dy if abs(dy) > 1e-6 else 0
        if 0.10 < frac < 0.80:
            xs.append(t - theta)
            zs.append(-__import__("math").log(1.0 - frac))
    if len(xs) >= 2:
        n = len(xs); mx = sum(xs) / n; mz = sum(zs) / n
        num = sum((x - mx) * (z - mz) for x, z in zip(xs, zs))
        den = sum((x - mx) ** 2 for x in xs)
        if den > 1e-9 and num > 0:
            tau = 1.0 / (num / den)
            fit_n = n
    elif len(xs) == 1:
        tau = xs[0] / zs[0] if zs[0] > 1e-6 else None
        fit_n = 1
    return {"u": u, "y0": y0, "y_inf": y_inf, "dy": dy, "theta": theta,
            "tau": tau, "fit_n": fit_n}

def simc_rows(K, tau, theta):
    rows = []
    for name, tc in [("快", theta), ("平衡", 1.5 * theta), ("稳", 3 * theta)]:
        Kc = tau / (K * (tc + theta))            # 千分比域: ‰ 每 RPM
        Ti = min(tau, 4 * (tc + theta))
        ki_f = Kc * TS / Ti                       # 千分比域: ‰ 每帧
        fw = 1000.0 / MAX_RPM                     # 固件 1 单位 = 3.1348 ‰
        rows.append((name, tc, Kc, Ti, ki_f, Kc / fw, ki_f / fw, 1.0 / K * 319 / 1000))
    return rows

def main():
    if len(sys.argv) < 2:
        print(__doc__); return 1
    Ks, taus = [], []
    for path in sys.argv[1:]:
        ts, us, ys = load_csv(path)
        if len(ts) < 8:
            print(f"[{path}] 数据不足 ({len(ts)} 行)"); continue
        r = identify(ts, us, ys)
        K = r["dy"] / r["u"]
        Ks.append(K)
        print(f"\n===== {path} =====")
        print(f"  u = {r['u']:.0f} 千分比 | {r['y0']:.1f} -> {r['y_inf']:.1f} RPM (已剔除滑行帧)")
        print(f"  K  = {K:.4f} RPM/千分比   (满占空外推 {1000*K:.0f} RPM, 固件常量 {MAX_RPM})")
        if r["tau"]:
            print(f"  τ  = {r['tau']*1000:.0f} ms  (ln 拟合 {r['fit_n']} 点)")
            taus.append(r["tau"])
        else:
            print(f"  τ  = 首帧已超过 10% 阶跃 — 响应接近采样分辨率, 建议按 τ≈{TS*2*1000:.0f}ms 上界处理")
        print(f"  θ  = {r['theta']*1000:.0f} ms  (受采样分辨率限制)")
        if r["tau"]:
            print(f"  SIMC 整定 (固件域换算已含, max_rpm={MAX_RPM}):")
            print(f"    档位   τc(s)   Kp(‰/RPM)  Kp(固件)  Ki每帧(固件)  Ki100  前馈系数(固件)")
            for name, tc, Kc, Ti, ki, kfw, kifw, ffw in simc_rows(K, r["tau"], r["theta"]):
                print(f"    {name:<4}  {tc:5.3f}  {Kc:8.3f}  {kfw:7.3f}  {kifw:9.4f}  {int(kifw*100):5d}  {ffw:6.3f}")
    if Ks:
        Kavg = sum(Ks) / len(Ks)
        print(f"\n----- 多工作点汇总 -----")
        print(f"  K 均值 = {Kavg:.4f} RPM/千分比 (离散度 {100*(max(Ks)-min(Ks))/Kavg:.1f}%)")
        if taus:
            tavg = sum(taus) / len(taus)
            print(f"  τ 均值 = {tavg*1000:.0f} ms")
            print(f"  前馈修正建议: ff = 1/K = {1.0/Kavg*319/1000:.3f} (固件域, 乘在 target RPM 上; 当前固件 0.3)")
            rows = simc_rows(Kavg, tavg, 0.05)
            rec = rows[1]
            print(f"  → 推荐[平衡]档: PID {rec[3]:.3f} {rec[4]:.3f} 0   (即 Kp100={int(rec[3]*100)} Ki100={int(rec[4]*100)})")
    return 0

if __name__ == "__main__":
    sys.exit(main())
