# -*- coding: utf-8 -*-
"""直立环输出级: 旧写法 vs 新写法, 以及"PID 够不够"的物理核算.
参数取自 params.vh 头部的实测值."""
import io
import sys

sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding="utf-8")

m, l, A, r = 0.0294, 0.061, 1.4479e-4, 0.070
Kt, g = 0.373, 9.81
J = 2.0e-4 + m * r * r                    # J_arm 是估的, 有 2~3 倍不确定
c = m * r * l

mgl = m * g * l
w2 = mgl / A                              # 倒立不稳定极点^2  (s^-2)
thdd_per_duty = (c / A) * (Kt / J)        # 每单位占空比 -> 摆杆角加速度
K = 9362.0 * 100.0 / 651.9                 # Kp 的换算: u% = e_cnt*Kp/9362

print("一、按 params.vh 实测参数算的物理需求")
print(f"   m*g*l/A             = {w2:.1f} s^-2  (不稳定极点 {w2**0.5:.2f} rad/s)")
print(f"   1 单位占空比 -> 摆杆 {thdd_per_duty:.0f} s^-2")
print(f"   (J = J_arm + m*r^2 = {J:.2e}, J_arm 是估的)")
print()
print(f"   刚好抵消发散:      Kp = {w2*K/thdd_per_duty:.0f}")
print(f"   3 倍余量 (能拉住): Kp = {3*w2*K/thdd_per_duty:.0f}")
print(f"   5 倍余量:          Kp = {5*w2*K/thdd_per_duty:.0f}")
print(f"   >>> 现在是 Kp=136, 只提供 {136/K*thdd_per_duty:.0f} s^-2/rad,"
      f" 连发散 ({w2:.0f}) 都盖不住")

print()
print("二、Kp=136 时截断的后果")
p_deg = 136 * 11.38 / 9362.0
print(f"   P: 1 度误差 = {p_deg:.3f} % 占空比")
print(f"      旧写法最小一档 0.25 %, 要偏 {0.25/p_deg:.1f} 度才出第一档")
thr = 16384 / (75 * 7)
print(f"   D: 旧写法要 10 ms 内变化 {thr:.0f} 个计数 ({thr*0.0879:.1f} 度)"
      f" 才够一档 -> 微分项被截没了")

print()
print("三、输出级对比 (Kp=136)")
print(f"   {'误差(度)':>9}{'旧u_q':>9}{'旧占空比%':>11}{'新u_q':>9}{'新占空比%':>11}")


def old_u(e_deg, kp=136):
    e_cnt = e_deg * 11.38
    ang = int(round(e_cnt * kp * 7 / 16384.0))
    ang = max(-400, min(400, ang))
    return ang * 25


def new_u(e_deg, kp=136):
    u = int(round(e_deg * 11.38 * kp * 7 * 25 / 16384.0))
    return max(-10000, min(10000, u))


for d in (0.1, 0.3, 0.5, 1.0, 1.5, 2.0, 3.0, 5.0, 10.0):
    o, n = old_u(d), new_u(d)
    print(f"   {d:9.1f}{o:9d}{o/100.0:11.3f}{n:9d}{n/100.0:11.3f}")

print()
print("四、三个增益整体 x4 (Kp=550 Ki=40 Kd=300) 后的输出")
for d in (0.3, 1.0, 2.0, 5.0, 10.0, 20.0):
    n = new_u(d, 550)
    print(f"   {d:5.1f} 度 -> u_q {n:6d}  ({n/100.0:.3f} %)")
print(f"   反算: Kp=550 提供 {550/K*thdd_per_duty:.0f} s^-2/rad"
      f"  (需求 {w2:.0f}, 余量 {550/K*thdd_per_duty/w2:.1f} 倍)")
