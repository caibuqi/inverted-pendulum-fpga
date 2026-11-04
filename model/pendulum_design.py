# -*- coding: utf-8 -*-
"""
旋转倒立摆 建模 / LQR 设计 / 非线性仿真 / 定点量化验证   (v2: 级联控制架构)
目标平台: 高云 GW1N-9C (GW1N-UV9EQ144C6/I5), J280 姿态控制系统竞赛套件

v2 相对 v1 的关键改动
--------------------
摆臂惯量很小(Kt/J ~ 500 rad/s^2), 摆杆自然频率只有 ~10 rad/s, 摆臂相当于高速
"飞轮"。直接用力矩做能量法起摆会因为相位失配而失败(v1 实测: 能量赤字卡在
-130mJ 不动)。因此改为级联:

    外环(1kHz)  -> alpha_des (角加速度需求) -> 积分 -> dphi_cmd (角速度给定)
    内环(1kHz)  -> PI + 反电动势前馈 -> u (占空比) -> 电机

平衡环用"角加速度输入"模型设计 LQR:
    phi''  = alpha
    theta''= (m*g*l/A)*theta - (c/A)*alpha - (b_pen/A)*theta'
好处: 增益不依赖电机参数(J, Kt, Ke), 只依赖机械参数, 对电机参数不确定更鲁棒,
      而且天然限制了摆臂速度, 满足赛题"旋转臂转角<=90度"的约束。

能量自洽性说明: 电机反电动势 Ke*phidot 是耗散项, 因此
    dE/dt = (Kt*u - Ke*phidot)*phidot,  而不是 Kt*u*phidot
"""

import json
import math
import os

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

plt.rcParams["font.sans-serif"] = ["Microsoft YaHei", "SimHei", "DejaVu Sans"]
plt.rcParams["axes.unicode_minus"] = False

OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "out")
os.makedirs(OUT, exist_ok=True)
TWOPI = 2.0 * math.pi


def fold_pi(a):
    """把角度折叠到 [-pi, pi] (0 = 正立)。报告时必须折叠, 否则摆杆转过整圈
    后原始角度会累加到几千度, 看起来像发散, 其实稳稳立在顶点。"""
    return (a + math.pi) % TWOPI - math.pi


def deg_fold(a):
    return math.degrees(fold_pi(a))


# =====================================================================
# 1. 参数 (★ 首轮估计值, 必须用实测替换)
# =====================================================================
def make_params(**over):
    p = dict(
        g=9.81,
        r=0.090,        # 摆臂长度: 电机轴心 -> 摆杆转轴 (m)
        L=0.260,        # 摆杆全长 (m)
        m_rod=0.040,    # 摆杆质量 (kg)
        m_tip=0.010,    # 摆杆末端等效附加质量 (kg)
        J_arm=2.0e-4,   # 摆臂 + 电机转子折算到竖轴的转动惯量 (kg*m^2)
        Kt=0.150,       # 满占空比堵转力矩 (N*m)
        w0=31.4,        # 空载角速度 (rad/s)
        b_arm=1.5e-4,   # 摆臂粘性阻尼 (N*m*s/rad)
        b_pen=1.0e-4,   # 摆杆粘性阻尼
        tc_arm=0.010,   # 摆臂库仑摩擦 (N*m)
        tc_pen=0.002,   # 摆杆库仑摩擦 (N*m)
        # --- 传感器 (★ 待确认) ---
        enc_cpr=2000,   # 编码器 4 倍频后每转计数 (500线*4)
        sa100_res=0.088 * math.pi / 180.0,  # SA100 角度分辨率 (rad), 暂按 12bit/360deg
        sa100_noise=0.05 * math.pi / 180.0,  # SA100 噪声标准差 (rad)
        twin=5e-3,      # 测速窗口 (s)
    )
    p.update(over)
    m = p["m_rod"] + p["m_tip"]
    l = (p["m_rod"] * p["L"] / 2 + p["m_tip"] * p["L"]) / m
    A = p["m_rod"] * p["L"] ** 2 / 3 + p["m_tip"] * p["L"] ** 2
    p.update(m=m, l=l, A=A, c=m * p["r"] * l,
             J=p["J_arm"] + m * p["r"] ** 2, Ke=p["Kt"] / p["w0"])
    p["Delta"] = p["J"] * p["A"] - p["c"] ** 2
    return p


P = make_params()


# =====================================================================
# 2. 非线性动力学
# =====================================================================
def motor_torque(dphi, u, p=P):
    """电机输出力矩 (含反电动势阻尼)"""
    return p["Kt"] * u - p["Ke"] * dphi


def deriv(x, u, p=P, dtau=0.0):
    """x = [phi, theta, phidot, thetadot]; dtau = 施加在摆杆上的外部扰动力矩"""
    phi, th, dphi, dth = x
    m, l, A, J, c, g = p["m"], p["l"], p["A"], p["J"], p["c"], p["g"]
    st, ct = math.sin(th), math.cos(th)

    tau = motor_torque(dphi, u, p)
    tau -= p["b_arm"] * dphi + p["tc_arm"] * math.tanh(dphi / 0.05)
    fpen = -(p["b_pen"] * dth + p["tc_pen"] * math.tanh(dth / 0.05))

    M11 = J + m * l * l * st * st
    M12 = c * ct
    M22 = A
    F1 = tau - 2.0 * m * l * l * st * ct * dth * dphi + c * st * dth * dth
    F2 = m * l * l * st * ct * dphi * dphi + m * g * l * st + fpen + dtau

    det = M11 * M22 - M12 * M12
    return np.array([dphi, dth,
                     (M22 * F1 - M12 * F2) / det,
                     (-M12 * F1 + M11 * F2) / det])


def energy_pend(x, p=P):
    """摆杆能量(摆臂随动坐标系内) E = 1/2*A*theta'^2 + m*g*l*cos(theta)
    这是起摆控制真正要控制的量:
      dE/dt = -c*cos(theta)*theta'*phi'' + m*l^2*sin*cos*phi'^2*theta'
    第一项是可用的泵, 第二项在摆臂速度受控时有界可忽略。
    E* = m*g*l (正立静止), E(下垂静止) = -m*g*l。"""
    _, th, _, dth = x
    return 0.5 * p["A"] * dth ** 2 + p["m"] * p["g"] * p["l"] * math.cos(th)


def energy_total(x, p=P):
    """系统总机械能 —— 用于自检 dE_total/dt = (Kt*u - Ke*phi')*phi'"""
    _, th, dphi, _ = x
    m, l, J, c, g = p["m"], p["l"], p["J"], p["c"], p["g"]
    T = (0.5 * J * dphi ** 2 + 0.5 * p["A"] * x[3] ** 2
         + 0.5 * m * l * l * math.sin(th) ** 2 * dphi ** 2
         + c * math.cos(th) * dphi * x[3])
    return T + m * g * l * math.cos(th)


energy = energy_pend          # 起摆控制器使用的别名


def rk4(x, u, dt, p=P, dtau=0.0):
    k1 = deriv(x, u, p, dtau)
    k2 = deriv(x + 0.5 * dt * k1, u, p, dtau)
    k3 = deriv(x + 0.5 * dt * k2, u, p, dtau)
    k4 = deriv(x + dt * k3, u, p, dtau)
    return x + dt / 6.0 * (k1 + 2 * k2 + 2 * k3 + k4)


# =====================================================================
# 3. LQR (numpy 自己解 CARE, 不依赖 scipy)
# =====================================================================
def lqr(A, B, Q, R):
    n = A.shape[0]
    Rinv = np.linalg.inv(R)
    H = np.block([[A, -B @ Rinv @ B.T], [-Q, -A.T]])
    w, v = np.linalg.eig(H)
    Vs = v[:, np.argsort(w.real)[:n]]
    U1, U2 = Vs[:n, :], Vs[n:, :]
    Pm = np.real(U2 @ np.linalg.inv(U1))
    Pm = (Pm + Pm.T) / 2.0
    return Rinv @ B.T @ Pm, Pm


def ctrb_rank(A, B):
    n = A.shape[0]
    C = B.copy()
    for _ in range(n - 1):
        C = np.hstack([C, A @ C])
    return int(np.linalg.matrix_rank(C)), n


def lin_torque(p=P, eps=1e-6):
    """力矩输入模型 (仅用于参考对比)"""
    x0 = np.zeros(4)
    Ac = np.zeros((4, 4))
    for i in range(4):
        d = np.zeros(4)
        d[i] = eps
        Ac[:, i] = (deriv(x0 + d, 0.0, p) - deriv(x0 - d, 0.0, p)) / (2 * eps)
    Bc = ((deriv(x0, eps, p) - deriv(x0, -eps, p)) / (2 * eps)).reshape(4, 1)
    return Ac, Bc


def lin_accel(p=P):
    """角加速度输入模型: phi''=alpha, theta''=(mgl/A)theta-(b_pen/A)theta'-(c/A)alpha"""
    Ac = np.zeros((4, 4))
    Ac[0, 2] = 1.0
    Ac[1, 3] = 1.0
    Ac[3, 1] = p["m"] * p["g"] * p["l"] / p["A"]
    Ac[3, 3] = -p["b_pen"] / p["A"]
    Bc = np.zeros((4, 1))
    Bc[2, 0] = 1.0
    Bc[3, 0] = -p["c"] / p["A"]
    return Ac, Bc


# =====================================================================
# 4. 传感器模型 (编码器 + SA100)
# =====================================================================
class Sensors:
    def __init__(self, p=P, seed=1):
        self.p = p
        self.rng = np.random.default_rng(seed)
        self.enc_q = TWOPI / p["enc_cpr"]
        self.nwin = max(1, int(round(p["twin"] * 1000)))
        self.buf = []
        self.th_prev = None
        self.dphi_lp = 0.0

    def measure(self, x_true, k):
        p = self.p
        phi, th, dphi, dth = x_true
        phi_q = round(phi / self.enc_q) * self.enc_q
        self.buf.append(phi_q)
        if len(self.buf) > self.nwin:
            self.buf.pop(0)
        if len(self.buf) >= 2:
            dphi_m = (self.buf[-1] - self.buf[0]) / ((len(self.buf) - 1) * 1e-3)
        else:
            dphi_m = 0.0
        self.dphi_lp += 0.35 * (dphi_m - self.dphi_lp)     # 一阶低通
        th_m = round(th / p["sa100_res"]) * p["sa100_res"]
        th_m += self.rng.normal(0.0, p["sa100_noise"])
        dth_m = 0.0 if self.th_prev is None else (th_m - self.th_prev) / 1e-3
        dth_m = max(-60.0, min(60.0, dth_m))
        self.th_prev = th_m
        return np.array([phi_q, th_m, self.dphi_lp, dth_m])


# =====================================================================
# 5. 内环: 摆臂角速度 PI + 反电动势前馈
# =====================================================================
class VelocityLoop:
    def __init__(self, p=P, kp=0.25, ki=3.0, ff=True, dphi_max=35.0,
                 dead_margin=1.3, dead_thresh=0.05):
        self.p = p
        self.kp, self.ki = kp, ki
        self.ff = ff
        self.dphi_max = dphi_max
        # 库仑摩擦死区补偿: 电机必须输出 > tc_arm 才转得动。
        # 不做这个补偿, 起摆启动时会被卡在下垂点(实测 |u|=0.054 < 0.067 起不来)
        self.u_dead = p["tc_arm"] / p["Kt"] * dead_margin
        self.dead_thresh = dead_thresh
        self.integ = 0.0

    def reset(self, dphi=0.0):
        self.integ = 0.0

    def __call__(self, dphi_cmd, dphi_meas, dt=1e-3):
        p = self.p
        cmd = max(-self.dphi_max, min(self.dphi_max, dphi_cmd))
        e = cmd - dphi_meas
        self.integ += self.ki * e * dt
        self.integ = max(-1.0, min(1.0, self.integ))
        u = self.kp * e + self.integ
        if self.ff:
            u += (p["Ke"] + p["b_arm"]) / p["Kt"] * cmd
        if abs(cmd) > self.dead_thresh:                     # 死区补偿
            u += self.u_dead if cmd > 0 else -self.u_dead
        return max(-1.0, min(1.0, u))


# =====================================================================
# 6. 外环: 起摆(能量法) + 平衡(加速度 LQR) + 位置环
# =====================================================================
class Controller:
    def __init__(self, Ka, p=P, th_bal=0.25, th_sw=0.50, dth_bal=3.0,
                 k_sw=1500.0, sw_dphi_max=12.0, k_center=10.0, k_damp=4.0,
                 alpha_sw_max=40.0, phi_wall=1.60, E_margin=0.03,
                 th_amp_target=0.20, dth_pump_max=14.0,
                 alpha_max=400.0, kick_amp=4.0, kick_f=3.0, kick_dth=0.5):
        self.Ka = np.asarray(Ka).ravel()
        self.p = p
        self.th_bal = th_bal
        self.th_sw = th_sw
        self.dth_bal = dth_bal        # 捕获时允许的最大 theta'
        self.k_sw = k_sw              # 起摆增益 (rad/s^2)/J
        self.sw_dphi_max = sw_dphi_max
        self.k_center = k_center
        self.k_damp = k_damp
        self.alpha_sw_max = alpha_sw_max
        self.phi_wall = phi_wall      # 摆臂软墙 (rad)
        self.E_margin = E_margin
        self.th_amp_target = th_amp_target   # 起摆停止泵的振幅阈值(距顶点 rad)
        self.dth_pump_max = dth_pump_max     # 摆杆超过此转速就停泵(防甩飞)
        self.alpha_max = alpha_max
        # 起振激励: 摆杆静止时它被转轴静摩擦"粘"在死区里(实测死区 1.5deg,
        # 重力力矩 1.5mN*m < 静摩擦 2mN*m), 而能量律在 theta'=0 时输出恒为 0,
        # 会永久死锁。所以先用方波抖动摆臂把它晃出来。
        self.kick_amp = kick_amp
        self.kick_f = kick_f
        self.kick_dth = kick_dth
        self.reset()

    def reset(self):
        self.mode = "swingup"
        self.phi_ref = 0.0
        self.phi_ref_cmd = 0.0
        self.dphi_cmd = 0.0
        self.started = False        # 是否已摆脱静摩擦
        self.kick_cnt = 0
        self.amp = math.pi          # 摆杆摆幅(距顶点角), 返回点处更新
        self.dth_prev = 0.0

    def __call__(self, xm, phi_ref_cmd=None, dt=1e-3):
        phi, th, dphi, dth = xm
        p = self.p
        thw = (th - math.pi) % TWOPI - math.pi     # 折叠到 [-pi,pi], 0 = 正立

        if self.mode == "swingup":
            if abs(thw) < self.th_bal and abs(dth) < self.dth_bal:
                self.mode = "balance"
                self.phi_ref = phi
                self.dphi_cmd = dphi
        else:
            if abs(thw) > self.th_sw:
                self.mode = "swingup"
                self.started = False
                self.kick_cnt = 0
                self.amp = math.pi
                self.dth_prev = 0.0

        if phi_ref_cmd is not None:
            self.phi_ref_cmd = phi_ref_cmd
        step = 1.2 * dt
        self.phi_ref += max(-step, min(step, self.phi_ref_cmd - self.phi_ref))

        if self.mode == "balance":
            e = np.array([phi - self.phi_ref, thw, dphi, dth])
            alpha = float(-self.Ka @ e)
            alpha = max(-self.alpha_max, min(self.alpha_max, alpha))
            self.dphi_cmd += alpha * dt
            self.dphi_cmd = max(-35.0, min(35.0, self.dphi_cmd))
        else:
            if not self.started and abs(dth) > self.kick_dth:
                self.started = True
            if self.dth_prev * dth < 0.0:          # 返回点: 记录摆幅(距顶点角)
                self.amp = abs(thw)
            self.dth_prev = dth
            if not self.started:
                # 起振方波: 摆臂来回摆, 靠加速度把摆杆从静摩擦死区里拽出来
                half = max(1, int(round(0.5 / (self.kick_f * dt))))
                sign = 1.0 if (self.kick_cnt // half) % 2 == 0 else -1.0
                self.kick_cnt += 1
                self.dphi_cmd = self.kick_amp * sign - 2.0 * phi
            else:
                if self.amp > self.th_amp_target and abs(dth) < self.dth_pump_max:
                    # 满功率泵能量:  alpha = -A*sign(cos(theta)*theta')
                    #   => dE/dt = +A*c*|cos(theta)*theta'| > 0  单调增
                    # 用"振幅判据"而不是能量判据: 摆臂速度快时耦合项
                    # m*l^2*sin*cos*phi'^2*theta' 可达泵功率的 5 倍, 会把能量估计
                    # 搅乱; 而返回点(theta'=0)处的 |theta| 就是干净的振幅, 无污染。
                    # 另加 |theta'| 护栏: 摆杆已经转太快就停泵, 否则会连转好几圈
                    # 才被平衡环抓住(实测多了 4 圈)。
                    s = math.cos(thw) * dth
                    alpha = -self.alpha_sw_max * (1.0 if s > 0 else -1.0)
                else:
                    alpha = 0.0            # 振幅已到位 / 转速过高, 停止泵能量
                alpha -= self.k_damp * dphi + self.k_center * phi
                alpha = max(-self.alpha_sw_max, min(self.alpha_sw_max, alpha))
                if abs(phi) > self.phi_wall:   # 软墙: 无条件全力回推
                    alpha = -self.alpha_sw_max * (1.0 if phi > 0 else -1.0)
                self.dphi_cmd += alpha * dt
            self.dphi_cmd = max(-self.sw_dphi_max,
                                min(self.sw_dphi_max, self.dphi_cmd))
        return self.dphi_cmd, dict(mode=1 if self.mode == "balance" else 0)


# =====================================================================
# 7. 闭环仿真
# =====================================================================
def simulate(x0, T=12.0, dt=1e-4, Tctl=1e-3, ctrl=None, vloop=None,
             phi_ref_traj=None, dist=None, sensors=False, p=P):
    n = int(T / dt)
    nsub = max(1, int(round(Tctl / dt)))
    xs = np.zeros((n + 1, 4))
    us = np.zeros(n + 1)
    modes = np.zeros(n + 1)
    cmds = np.zeros(n + 1)
    ts = np.arange(n + 1) * dt
    x = np.array(x0, dtype=float)
    xs[0] = x
    sens = Sensors(p) if sensors else None
    u = 0.0
    for i in range(n):
        if i % nsub == 0:
            ref = None if phi_ref_traj is None else phi_ref_traj(ts[i])
            xm = sens.measure(x, i) if sens is not None else x
            dphi_cmd, d = ctrl(xm, ref, Tctl)
            u = vloop(dphi_cmd, xm[2], Tctl)
            modes[i] = d["mode"]
            cmds[i] = dphi_cmd
        dtau = 0.0 if dist is None else float(dist(ts[i]))
        x = rk4(x, u, dt, p, dtau)
        us[i] = u
        xs[i + 1] = x
    us[n] = u
    modes[n] = modes[n - 1] if n else 0
    cmds[n] = cmds[n - 1]
    return ts, xs, us, modes, cmds


# =====================================================================
# 8. 定点实现 (为 GW1N-9C 里的 Verilog 定标)
# =====================================================================
SX = np.array([1e-4, 1e-4, 1e-3, 1e-3])   # [phi,theta,phidot,thetadot] 每 LSB
SA = 1e-1                                  # alpha 每 LSB (rad/s^2)


class FixedVelocityLoop:
    """定点内环。定标: u_q=u/1e-4(duty), e_q=e/1e-3(rad/s)
    u = kp*e            -> u_q = kp*e_q*(1e-3/1e-4)      = kp*e_q*10
    integ += ki*e*dt    -> d(integ_q) = ki*e_q*dt*10     (dt=1e-3 时 = ki*e_q*0.01)
    u += kff*cmd        -> u_q = kff*cmd_q*10
    即统一乘 (1e-3/1e-4)=10, 再乘 2^sh。"""

    def __init__(self, vl, sh=12, dt=1e-3):
        self.p = vl.p
        self.sh = sh
        scale = (1e-3 / 1e-4)                 # = 10
        self.kp_q = int(round(vl.kp * scale * (1 << sh)))
        self.ki_q = int(round(vl.ki * scale * dt * (1 << sh)))
        self.kff_q = int(round((vl.p["Ke"] + vl.p["b_arm"]) / vl.p["Kt"]
                               * scale * (1 << sh)))
        self.udead_q = int(round(vl.u_dead / 1e-4))
        self.dead_thresh_q = int(round(vl.dead_thresh / 1e-3))
        self.reset()

    def reset(self):
        self.integ_q = 0

    def __call__(self, dphi_cmd, dphi_meas, dt=1e-3):
        cmd_q = int(round(dphi_cmd / 1e-3))
        e_q = cmd_q - int(round(dphi_meas / 1e-3))
        self.integ_q += (self.ki_q * e_q) >> self.sh
        self.integ_q = max(-10000, min(10000, self.integ_q))
        acc = self.kp_q * e_q + self.kff_q * cmd_q + (self.integ_q << self.sh)
        acc += (1 << (self.sh - 1)) if acc >= 0 else -(1 << (self.sh - 1))
        uq = acc >> self.sh
        if abs(cmd_q) > self.dead_thresh_q:
            uq += self.udead_q if cmd_q > 0 else -self.udead_q
        return float(max(-10000, min(10000, uq)) * 1e-4)


class FixedController:
    def __init__(self, ctrl, sh=12):
        self.p = ctrl.p
        self.sh = sh
        self.Kq = np.round(np.asarray(ctrl.Ka).ravel() * SX / SA
                           * (1 << sh)).astype(np.int64)
        self.th_bal = int(round(ctrl.th_bal / SX[1]))
        self.th_sw = int(round(ctrl.th_sw / SX[1]))
        self.dth_bal = int(round(ctrl.dth_bal / SX[3]))
        self.phi_wall = ctrl.phi_wall
        self.k_sw = ctrl.k_sw
        self.sw_dphi_max_q = int(round(ctrl.sw_dphi_max / SX[2]))
        self.k_center = ctrl.k_center
        self.k_damp = ctrl.k_damp
        self.alpha_sw_max = ctrl.alpha_sw_max
        self.E_margin = ctrl.E_margin
        self.alpha_max = int(round(ctrl.alpha_max / SA))
        self.kick_amp_q = int(round(ctrl.kick_amp / SX[2]))
        self.kick_half = max(1, int(round(0.5 / (ctrl.kick_f * 1e-3))))
        self.kick_dth_q = int(round(ctrl.kick_dth / SX[3]))
        self.th_amp_target_q = int(round(ctrl.th_amp_target / SX[1]))
        self.dth_pump_max_q = int(round(ctrl.dth_pump_max / SX[3]))
        self.reset()

    def reset(self):
        self.mode = 0
        self.phi_ref_q = 0
        self.dphi_cmd_q = 0
        self.dphi_cmd = 0.0
        self.started = False
        self.kick_cnt = 0
        self.amp_q = int(round(math.pi / SX[1]))
        self.dth_prev_q = 0

    def __call__(self, xm, phi_ref_cmd=None, dt=1e-3):
        p = self.p
        phi_q, th_q, dphi_q, dth_q = (np.round(np.asarray(xm) / SX)).astype(np.int64)
        PI_Q = int(round(math.pi / SX[1]))
        thw_q = ((th_q + PI_Q) % (2 * PI_Q)) - PI_Q
        if self.mode == 0:
            if abs(thw_q) < self.th_bal and abs(dth_q) < self.dth_bal:
                self.mode = 1
                self.phi_ref_q = int(phi_q)
                self.dphi_cmd_q = int(dphi_q)
        else:
            if abs(thw_q) > self.th_sw:
                self.mode = 0
                self.started = False
                self.kick_cnt = 0
                self.amp_q = int(round(math.pi / SX[1]))
                self.dth_prev_q = 0
        if phi_ref_cmd is not None:
            step_q = max(1, int(round(1.2 * dt / SX[0])))
            tgt = int(round(phi_ref_cmd / SX[0]))
            d = tgt - self.phi_ref_q
            self.phi_ref_q += max(-step_q, min(step_q, d))

        if self.mode == 1:
            e = np.array([phi_q - self.phi_ref_q, thw_q, dphi_q, dth_q],
                         dtype=np.int64)
            acc = int(np.dot(self.Kq, e))
            acc += (1 << (self.sh - 1)) if acc >= 0 else -(1 << (self.sh - 1))
            aq = -(acc >> self.sh)
            aq = max(-self.alpha_max, min(self.alpha_max, aq))
            self.dphi_cmd_q += int(round(aq * SA * dt / SX[2]))
            self.dphi_cmd_q = max(-35000, min(35000, self.dphi_cmd_q))
        else:
            if not self.started and abs(dth_q) > self.kick_dth_q:
                self.started = True
            if self.dth_prev_q * dth_q < 0:
                self.amp_q = abs(thw_q)
            self.dth_prev_q = int(dth_q)
            if not self.started:
                sign = 1 if (self.kick_cnt // self.kick_half) % 2 == 0 else -1
                self.kick_cnt += 1
                self.dphi_cmd_q = self.kick_amp_q * sign \
                    - int(round(2.0 * phi_q * SX[0] / SX[2]))
            else:
                thf, dthf, dphif = th_q * SX[1], dth_q * SX[3], dphi_q * SX[2]
                phif = phi_q * SX[0]
                thw_f = ((thf - math.pi) % TWOPI) - math.pi
                if self.amp_q > self.th_amp_target_q \
                        and abs(dth_q) < self.dth_pump_max_q:
                    s = math.cos(thw_f) * dthf
                    alpha = -self.alpha_sw_max * (1.0 if s > 0 else -1.0)
                else:
                    alpha = 0.0                      # 振幅到位/转速过高, 停止泵
                alpha -= self.k_damp * dphif + self.k_center * phif
                alpha = max(-self.alpha_sw_max, min(self.alpha_sw_max, alpha))
                if abs(phif) > self.phi_wall:
                    alpha = -self.alpha_sw_max * (1.0 if phif > 0 else -1.0)
                aq = int(round(alpha / SA))
                self.dphi_cmd_q += int(round(aq * SA * 1e-3 / SX[2]))
            self.dphi_cmd_q = max(-self.sw_dphi_max_q,
                                  min(self.sw_dphi_max_q, self.dphi_cmd_q))
        self.dphi_cmd = self.dphi_cmd_q * SX[2]
        return self.dphi_cmd, dict(mode=self.mode)


# =====================================================================
# 9. main
# =====================================================================
def main():
    p = P
    np.set_printoptions(precision=4, suppress=True)
    print("=" * 74)
    print("旋转倒立摆参数 (★首轮估计值, 待实测替换)")
    print("=" * 74)
    for k in ["r", "L", "m_rod", "m_tip", "m", "l", "A", "J_arm", "J", "c",
              "Delta", "Kt", "Ke", "w0", "enc_cpr", "twin"]:
        print(f"  {k:9s} = {p[k]:.6g}")

    print("\n" + "=" * 74)
    print("[自检] 总机械能 dE_total/dt 应等于 (电机功率 - 摩擦耗散)")
    print("=" * 74)
    pf = make_params(b_arm=0.0, b_pen=0.0, tc_arm=0.0, tc_pen=0.0)  # 无摩擦
    ok = True
    for x in (np.array([0.3, 1.0, 2.0, 3.0]), np.array([0.5, 2.5, -4.0, 6.0]),
              np.array([0.0, math.pi, 3.0, 0.0])):
        for u in (0.0, 0.4, -0.7):
            f = deriv(x, u, pf)
            eps = 1e-7
            dE = (energy_total(x + eps * f, pf)
                  - energy_total(x - eps * f, pf)) / (2 * eps)
            want = motor_torque(x[2], u, pf) * x[2]
            ok &= abs(dE - want) < 1e-6
            print(f"  x={x} u={u:+.1f}: dE/dt={dE:+12.6f} 期望={want:+12.6f} "
                  f"差={dE - want:+.2e}")
    print("  => 动力学与能量自洽 OK" if ok else "  => 不自洽 !!")
    # 有摩擦时的残差应正好等于摩擦耗散功率 (只打印核对)
    for x in (np.array([0.3, 1.0, 2.0, 3.0]),):
        f = deriv(x, 0.0, p)
        eps = 1e-7
        dE = (energy_total(x + eps * f, p) - energy_total(x - eps * f, p)) / (2 * eps)
        pfric = (p["b_arm"] * x[2] ** 2 + p["tc_arm"] * abs(x[2])
                 + p["b_pen"] * x[3] ** 2 + p["tc_pen"] * abs(x[3]))
        want = motor_torque(x[2], 0.0, p) * x[2] - pfric
        print(f"  [含摩擦核对] dE/dt={dE:+.6f} 期望={want:+.6f} 差={dE-want:+.2e}")
    x = np.array([0.0, 1.2, 0.0, 0.0])
    pc = make_params(Kt=0.0, b_arm=0.0, b_pen=0.0, tc_arm=0.0, tc_pen=0.0)
    E0 = energy_total(x, pc)
    xc = x.copy()
    for _ in range(20000):
        xc = rk4(xc, 0.0, 1e-5, pc)
    print(f"\n[自检] 无电机无摩擦总能量守恒相对误差 = "
          f"{abs(energy_total(xc, pc) - E0) / abs(E0):.2e}")

    print("\n" + "=" * 74)
    print("[设计] LQR")
    print("=" * 74)
    At, Bt = lin_torque(p)
    print(f"力矩模型能控性秩 = {ctrb_rank(At, Bt)[0]}/4")
    Aa, Ba = lin_accel(p)
    print(f"加速度模型能控性秩 = {ctrb_rank(Aa, Ba)[0]}/4")

    Qa = np.diag([40.0, 500.0, 1.0, 10.0])
    print(f"\n[加速度输入模型] A =\n{Aa}\n  B = {Ba.ravel()}")
    print(f"  Q = diag{np.diag(Qa).tolist()}")
    print("\n  R 扫描 —— 必须让最快闭环极点明显小于内环速度环带宽(33~132 rad/s),")
    print("  否则等于要求执行机构做它做不到的事(增益再大也没用, 只会抖/饱和)")
    target = 20.0
    best = None
    for amax in (3, 5, 8, 12, 20, 30, 50, 80, 150, 300):
        R = np.array([[1.0 / amax ** 2]])
        K_, _ = lqr(Aa, Ba, Qa, R)
        K_ = K_.ravel()
        cl_ = np.linalg.eigvals(Aa - Ba @ K_.reshape(1, -1))
        fp = float(np.max(np.abs(cl_)))
        print(f"   alpha_max={amax:6.1f}  R={R[0,0]:9.2e}  最快极点={fp:8.1f} rad/s  "
              f"Ka_th={K_[1]:9.1f}  Ka_dphi={K_[2]:9.1f}  Ka_dth={K_[3]:9.1f}")
        if best is None or abs(fp - target) < best[0]:
            best = (abs(fp - target), R, K_, cl_)
    _, Ra, Ka, cl = best
    print(f"\n  选中: R = {Ra[0,0]:.3e}  (alpha_max = {1/math.sqrt(Ra[0,0]):.1f} rad/s^2)")
    print(f"  Ka = {np.round(Ka, 3).tolist()}")
    print(f"  alpha_des = -({Ka[0]:.2f}*(phi-phiref) + {Ka[1]:.2f}*theta "
          f"+ {Ka[2]:.2f}*phidot + {Ka[3]:.2f}*thetadot)")
    print(f"  闭环极点 = {np.round(cl, 3).tolist()}")
    print(f"  人话: theta 项 {Ka[1] / 57.2958:.1f} (rad/s^2)/deg, "
          f"thetadot 项 {Ka[3] / 57.2958:.2f} per (deg/s)")
    print(f"  余量: 1 deg 倾角 -> alpha={abs(Ka[1])*math.pi/180:.0f} rad/s^2")

    dphi_max = p["Kt"] / (p["Ke"] + p["b_arm"])
    print(f"\n[内环] 摆臂开环稳态 phidot/u = {dphi_max:.1f} rad/s, "
          f"时间常数 {p['J'] / (p['Ke'] + p['b_arm']) * 1000:.1f} ms")
    for kp in (0.10, 0.25, 0.50):
        tau_cl = p["J"] / (p["Kt"] * kp + p["Ke"] + p["b_arm"])
        print(f"  kp={kp:4.2f} -> 速度环闭环时间常数 {tau_cl*1000:5.1f} ms "
              f"(带宽 {1/tau_cl:.0f} rad/s)")

    print("\n" + "=" * 74)
    print("[仿真] 场景")
    print("=" * 74)

    def run(x0, T, ref=None, dist=None, sensors=False, kp=0.25, ki=3.0,
            k_sw=60.0, th_bal=0.30):
        c = Controller(Ka, p, k_sw=k_sw, th_bal=th_bal)
        v = VelocityLoop(p, kp=kp, ki=ki)
        return (c, v) + simulate(x0, T=T, ctrl=c, vloop=v, phi_ref_traj=ref,
                                 dist=dist, sensors=sensors, p=p)

    def trace_table(ts, xs, us, cmds, modes, t0, t1, step=0.5, dt=1e-4):
        print("      t    theta(deg)  phi(deg)  phidot  dphi_cmd   thetadot   u   mode")
        t = t0
        while t <= t1:
            i = int(round(t / dt))
            if i < len(ts):
                print(f"   {ts[i]:5.2f}  {deg_fold(xs[i,1]):+9.2f} "
                      f"{math.degrees(xs[i,0]):+8.2f} {xs[i,2]:+7.2f} "
                      f"{cmds[i]:+8.2f} {xs[i,3]:+9.2f} {us[i]:+6.3f} "
                      f"{'平衡' if modes[i] > 0 else '起摆'}")
            t += step

    c1, v1, ts, xs, us, modes, cmds = run([0.0, 0.12, 0.0, 0.0], 6.0)
    thf1 = np.abs(np.array([deg_fold(a) for a in xs[:, 1]]))
    print(f"[场景1 平衡] 末态 theta={deg_fold(xs[-1,1]):+.3f} deg, "
          f"后半程|theta|max={thf1[len(thf1)//2:].max():.3f} deg, "
          f"|u|max={np.abs(us).max():.3f}, |phidot|max={np.abs(xs[:,2]).max():.1f}")
    trace_table(ts, xs, us, cmds, modes, 0.0, 5.0, 0.5)

    def ref(t):
        return 0.9 if t > 8.0 else 0.0

    TH_DD_PUSH = 25.0        # 外部推力 -> 摆杆角加速度 (rad/s^2), 约合 2 rad/s 冲量

    def dist(t):
        return p["A"] * TH_DD_PUSH if 6.0 <= t < 6.08 else 0.0

    c2, v2, ts2, xs2, us2, modes2, cmds2 = run([0.0, math.pi - 0.02, 0.0, 0.0], 13.0,
                                               ref=ref, dist=dist, k_sw=60.0)
    if (modes2 > 0).any():
        tb = ts2[int(np.argmax(modes2 > 0))]
        seg = slice(int(8.5 / 1e-4), None)
        thf2 = np.abs(np.array([deg_fold(a) for a in xs2[seg, 1]]))
        print(f"[场景2 起摆] 进入平衡 t={tb:.2f} s; 末态 theta="
              f"{deg_fold(xs2[-1,1]):+.3f} deg (折叠后), phi="
              f"{math.degrees(xs2[-1,0]):+.2f} deg (目标 51.57), "
              f"|u|max={np.abs(us2).max():.3f}")
        print(f"             位置到位后 |theta|max={thf2.max():.3f} deg, "
              f"|phidot|max={np.abs(xs2[seg,2]).max():.2f} rad/s")
        # 注意: modes 只在控制节拍(i % nsub == 0)赋值, 其余采样点为 0,
        # 所以统计切换次数必须按控制节拍抽样, 否则每个节拍都会误算 2 次。
        nsub = 10
        m10 = (modes2[::nsub] > 0).astype(int)
        nsw = int(np.sum(np.diff(m10) != 0))
        m10_late = (modes2[int(8.0 / 1e-4)::nsub] > 0).astype(int)
        nsw_late = int(np.sum(np.diff(m10_late) != 0))
        print(f"             模式切换次数 = {nsw} (其中最后 5s 只有 {nsw_late} 次; "
              f"后期还频繁切换说明平衡环抓不住)")
        trace_table(ts2, xs2, us2, cmds2, modes2, 0.0, 11.0, 0.5)
    else:
        print("[场景2 起摆] ★ 失败: 从未进入平衡态")
        trace_table(ts2, xs2, us2, cmds2, modes2, 0.0, 11.0, 0.5)

    c3, v3, ts3, xs3, us3, modes3, cmds3 = run([0.0, math.pi - 0.02, 0.0, 0.0], 13.0,
                                               ref=ref, dist=dist, sensors=True)
    if (modes3 > 0).any():
        seg = slice(int(8.5 / 1e-4), None)
        thf3 = np.abs(np.array([deg_fold(a) for a in xs3[seg, 1]]))
        print(f"[场景3 带传感器] 末态 theta={deg_fold(xs3[-1,1]):+.3f} deg, "
              f"位置到位后 |theta|max={thf3.max():.3f} deg")
        print(f"             注: 传感器量化="
              f"{math.degrees(p['sa100_res']):.3f} deg, 噪声="
              f"{math.degrees(p['sa100_noise']):.3f} deg, "
              f"测速窗口={p['twin']*1000:.0f} ms")
    else:
        print("[场景3 带传感器] ★ 失败: 从未进入平衡态")

    fc = FixedController(Controller(Ka, p), sh=12)
    fv = FixedVelocityLoop(VelocityLoop(p), sh=12)
    ts4, xs4, us4, modes4, cmds4 = simulate([0.0, 0.12, 0.0, 0.0], T=6.0,
                                            ctrl=fc, vloop=fv, p=p)
    print(f"[场景4 定点] 末态 theta={deg_fold(xs4[-1,1]):+.3f} deg "
          f"(浮点 {deg_fold(xs[-1,1]):+.3f}); max|Kq|={abs(fc.Kq).max()} "
          f"({int(abs(fc.Kq).max()).bit_length()}bit), kp_q={fv.kp_q}, "
          f"ki_q={fv.ki_q}, kff_q={fv.kff_q}, udead_q={fv.udead_q}")

    fig, ax = plt.subplots(5, 1, figsize=(12, 12), sharex=True)
    ax[0].plot(ts2, np.degrees(xs2[:, 1]), lw=1.0)
    ax[0].axhline(0, color="k", lw=0.5)
    for y in (17.2, -17.2):
        ax[0].axhline(y, color="r", ls=":", lw=0.8)
    ax[0].set_ylabel("theta (deg)")
    ax[0].set_title("摆杆倾角 (红虚线=+-17.2deg 捕获窗口)")
    ax[0].grid(alpha=.3)
    ax[1].plot(ts2, np.degrees(xs2[:, 0]), lw=1.0, color="tab:orange")
    ax[1].plot(ts2, np.degrees(np.clip((ts2 - 8.0) * 1.2, 0.0, 0.9)), "r--", lw=0.9)
    for y in (90, -90):
        ax[1].axhline(y, color="r", ls=":", lw=0.8)
    ax[1].set_ylabel("phi (deg)")
    ax[1].set_title("摆臂转角 (红虚线=+-90deg 赛题约束)")
    ax[1].grid(alpha=.3)
    ax[2].plot(ts2, xs2[:, 2], lw=1.0, color="tab:red", label="phidot")
    ax[2].plot(ts2, cmds2, lw=0.8, color="k", alpha=.6, label="phidot_cmd")
    ax[2].set_ylabel("rad/s")
    ax[2].legend()
    ax[2].grid(alpha=.3)
    ax[3].plot(ts2, xs2[:, 3], lw=1.0, color="tab:green")
    ax[3].set_ylabel("thetadot (rad/s)")
    ax[3].grid(alpha=.3)
    ax[4].plot(ts2, us2, lw=1.0, color="tab:purple", label="u")
    ax[4].plot(ts2, modes2 * 0.8 - 0.9, lw=0.8, color="gray", label="mode(0起摆/1平衡)")
    ax[4].set_ylabel("占空比")
    ax[4].set_xlabel("t (s)")
    ax[4].legend()
    ax[4].grid(alpha=.3)
    fig.suptitle("旋转倒立摆 v2 级联控制: 起摆 -> 平衡 -> 定点移动 -> 抗扰")
    fig.tight_layout()
    f1 = os.path.join(OUT, "cascade_run.png")
    fig.savefig(f1, dpi=120)

    fig2, ax2 = plt.subplots(3, 1, figsize=(12, 7), sharex=True)
    ax2[0].plot(ts3, np.degrees(xs3[:, 1]), lw=0.8)
    ax2[0].set_ylabel("theta(deg) 带噪声")
    ax2[0].grid(alpha=.3)
    ax2[1].plot(ts3, np.degrees(xs3[:, 0]), lw=0.8, color="tab:orange")
    ax2[1].set_ylabel("phi(deg)")
    ax2[1].grid(alpha=.3)
    ax2[2].plot(ts3, us3, lw=0.8, color="tab:purple")
    ax2[2].set_ylabel("u")
    ax2[2].set_xlabel("t (s)")
    ax2[2].grid(alpha=.3)
    fig2.suptitle("加入编码器量化 / SA100 量化噪声后的表现")
    fig2.tight_layout()
    f2 = os.path.join(OUT, "with_sensors.png")
    fig2.savefig(f2, dpi=120)

    gains = dict(
        note="v2 级联架构; 首轮估计参数算出, 实测参数后必须重算",
        params={k: (float(v) if isinstance(v, (int, float)) else v)
                for k, v in p.items()},
        Ka=Ka.tolist(), Qa=np.diag(Qa).tolist(), Ra=float(Ra[0, 0]),
        cl_poles=[[float(z.real), float(z.imag)] for z in cl],
        SX=SX.tolist(), SA=SA,
        Kq=fc.Kq.tolist(), kp_q=fv.kp_q, ki_q=fv.ki_q, kff_q=fv.kff_q,
    )
    with open(os.path.join(OUT, "gains.json"), "w", encoding="utf-8") as f:
        json.dump(gains, f, indent=2, ensure_ascii=False)
    print(f"\n输出: {f1}\n      {f2}\n      {os.path.join(OUT, 'gains.json')}")


if __name__ == "__main__":
    main()
