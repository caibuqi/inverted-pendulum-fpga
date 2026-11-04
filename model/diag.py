# -*- coding: utf-8 -*-
"""诊断: 1) 动力学与能量是否自洽  2) 起摆为什么失败"""
import math
import numpy as np
import pendulum_design as pd

p = pd.P
pn = pd.make_params(b_arm=0.0, b_pen=0.0, tc_arm=0.0, tc_pen=0.0)
np.set_printoptions(precision=4, suppress=True)

print("=" * 70)
print("[诊断1] 动力学 <-> 能量 自洽性: dE/dt 应等于电机输入功率 tau*phidot")
print("=" * 70)
for x in (np.array([0.3, 1.0, 2.0, 3.0]),
          np.array([0.0, 0.7, 0.0, 0.0]),
          np.array([0.5, 2.5, -4.0, 6.0]),
          np.array([0.0, math.pi, 3.0, 0.0])):
    for u in (0.0, 0.4, -0.7):
        f = pd.deriv(x, u, pn)
        eps = 1e-7
        dE = (pd.energy(x + eps * f, pn) - pd.energy(x - eps * f, pn)) / (2 * eps)
        want = pn["Kt"] * u * x[2]
        print(f"x={x} u={u:+.1f}:  dE/dt(数值)={dE:+12.6f}   tau*phidot={want:+12.6f}"
              f"   差={dE - want:+.3e}")

print()
print("=" * 70)
print("[诊断2] 起摆过程 trace (初始: 摆杆自然下垂)")
print("=" * 70)
K = np.array([-5.4772, -593.5716, -9.2704, -22.4685])   # 上一轮 LQR 结果
ctrl = pd.Controller(K, p, k_sw=2.0)


def trace(k_sw=2.0, th_bal=0.35, dth_bal=6.0, T=8.0, x0=None, E_dead=0.05):
    c = pd.Controller(K, p, th_bal=th_bal, k_sw=k_sw, E_dead=E_dead)
    c.dth_bal = dth_bal
    x = np.array(x0 if x0 is not None else [0.0, math.pi - 0.02, 0.0, 0.0])
    dt, nsub = 1e-4, 10
    Estar = p["m"] * p["g"] * p["l"]
    print(f"  k_sw={k_sw} th_bal={th_bal} E_dead={E_dead}")
    print("     t      th(deg)   dth     phi(deg)  dphi    E-E*(mJ)  mode   u")
    u = 0.0
    for i in range(int(T / dt)):
        if i % nsub == 0:
            u, d = c(x, 0.0, 1e-3)
        if i % int(0.25 / dt) == 0:
            print(f"  {i*dt:6.2f}  {math.degrees(x[1]):9.1f} {x[3]:7.2f} "
                  f"{math.degrees(x[0]):9.1f} {x[2]:7.2f} "
                  f"{(pd.energy(x, p) - Estar)*1000:9.3f}   {d['mode']:3d}  {u:+.3f}")
        x = pd.rk4(x, u, dt, p)
    return x


trace(k_sw=2.0)
print()
trace(k_sw=8.0)
print()
trace(k_sw=8.0, th_bal=0.5, E_dead=0.01)
