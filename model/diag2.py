# -*- coding: utf-8 -*-
"""诊断: 平衡环 ±2.2deg 极限环是不是摩擦死区引起的 (逐个关掉摩擦做隔离实验)"""
import math
import numpy as np
import pendulum_design as pd

p = pd.P
Aa, Ba = pd.lin_accel(p)
Qa = np.diag([40.0, 500.0, 1.0, 10.0])
best = None
for amax in (3, 5, 8, 12, 20, 30, 50, 80, 150, 300):
    R = np.array([[1.0 / amax ** 2]])
    K, _ = pd.lqr(Aa, Ba, Qa, R)
    K = K.ravel()
    fp = float(np.max(np.abs(np.linalg.eigvals(Aa - Ba @ K.reshape(1, -1)))))
    if best is None or abs(fp - 20) < best[0]:
        best = (abs(fp - 20), R, K)
_, Ra, Ka = best
print(f"Ka = {np.round(Ka,2).tolist()}\n")

print(f"摆杆库仑摩擦造成的角度死区 = "
      f"{math.degrees(p['tc_pen']/(p['m']*p['g']*p['l'])):.2f} deg")
print(f"摆臂库仑摩擦需要的启动占空比 = {p['tc_arm']/p['Kt']:.4f}\n")

print(f"{'工况':<18}{'后4s|theta|max':>16}{'均方':>10}{'|u|max':>9}{'末phi(deg)':>12}")
for tag, over in [("全摩擦(基准)", {}),
                  ("无摆杆库仑摩擦", dict(tc_pen=0.0)),
                  ("无摆臂库仑摩擦", dict(tc_arm=0.0)),
                  ("无库仑摩擦", dict(tc_arm=0.0, tc_pen=0.0)),
                  ("完全无摩擦", dict(tc_arm=0.0, tc_pen=0.0, b_arm=0.0, b_pen=0.0))]:
    pp = pd.make_params(**over)
    c = pd.Controller(Ka, pp)
    v = pd.VelocityLoop(pp)
    ts, xs, us, modes, cmds = pd.simulate([0.0, 0.12, 0.0, 0.0], T=8.0,
                                          ctrl=c, vloop=v, p=pp)
    seg = slice(int(4.0 / 1e-4), None)
    th = np.degrees(xs[seg, 1])
    print(f"{tag:<18}{np.abs(th).max():16.4f}{np.sqrt(np.mean(th**2)):10.4f}"
          f"{np.abs(us).max():9.3f}{math.degrees(xs[-1,0]):12.2f}")

# --- 摆杆静摩擦灵敏度扫描 (决定平衡质量的关键机械参数) ---
print("\n摆杆库仑摩擦 -> 平衡极限环幅度 (决定'无明显震荡'能不能达标):")
print(f"{'tc_pen(mN*m)':>14}{'死区(deg)':>12}{'后4s|theta|max':>16}{'均方':>10}")
for tcp in (0.0, 0.0002, 0.0005, 0.001, 0.002, 0.004):
    pp = pd.make_params(tc_pen=tcp)
    c = pd.Controller(Ka, pp)
    v = pd.VelocityLoop(pp)
    ts, xs, us, modes, cmds = pd.simulate([0.0, 0.12, 0.0, 0.0], T=8.0,
                                          ctrl=c, vloop=v, p=pp)
    seg = slice(int(4.0 / 1e-4), None)
    th = np.degrees(xs[seg, 1])
    dz = math.degrees(tcp / (pp["m"] * pp["g"] * pp["l"]))
    print(f"{tcp*1000:14.2f}{dz:12.3f}{np.abs(th).max():16.4f}"
          f"{np.sqrt(np.mean(th**2)):10.4f}")

# --- 起摆参数扫描 (k_sw=1500, a_sw_max=80: 重点看摆臂是否越界 90deg) ---
print("\n起摆扫描 (初始自然下垂且在摩擦死区内, dt=2e-4, T=12s):")
pp = pd.make_params()
print(f"{'k_damp':>8}{'dphi_max':>10}{'起摆时间(s)':>13}{'末theta(deg)':>14}"
      f"{'|phi|max':>10}{'|phidot|max':>12}{'后期切换':>10}")
for kd in (4.0, 15.0, 30.0, 60.0):
    for dpm in (5.0, 12.0):
        c = pd.Controller(Ka, pp, alpha_sw_max=40.0, k_center=10.0,
                          k_damp=kd, sw_dphi_max=dpm, phi_wall=1.6)
        v = pd.VelocityLoop(pp)
        ts, xs, us, modes, cmds = pd.simulate([0.0, math.pi - 0.02, 0.0, 0.0],
                                              T=20.0, dt=2e-4, ctrl=c, vloop=v, p=pp)
        pmax = math.degrees(np.abs(xs[:, 0]).max())
        nlate = int(np.sum(np.diff((modes[int(15.0 / 2e-4):] > 0).astype(int)) != 0))
        if (modes > 0).any():
            tb = ts[int(np.argmax(modes > 0))]
            print(f"{kd:8.1f}{dpm:10.1f}{tb:13.2f}"
                  f"{math.degrees(xs[-1,1]):14.3f}{pmax:10.1f}"
                  f"{np.abs(xs[:,2]).max():12.2f}{nlate:10d}")
        else:
            print(f"{kd:8.1f}{dpm:10.1f}{'失败':>13}"
                  f"{math.degrees(xs[-1,1]):14.3f}{pmax:10.1f}"
                  f"{np.abs(xs[:,2]).max():12.2f}{nlate:10d}")





