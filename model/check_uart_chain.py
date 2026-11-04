# -*- coding: utf-8 -*-
"""
位精确复刻 RTL 的两段逻辑, 用来验证"串口调好的参数填进 params.vh 后效果不一致":

  1) uart.v 的包解析状态机 (P_IDLE .. P_GET)
  2) ctrl.v 的角度链 + 直立环 PID -> u_q

所有取整/移位/位宽都按 Verilog 规则做 (>>> 算术右移, 截断到固定位宽)。
"""

WRAP = 62832

# ---------------------------------------------------------------- 串口解析
P_IDLE, P_S, P_COM1, P_BRCH, P_US, P_K, P_KIND, P_COM2, P_NUM, \
    P_T2, P_T3, P_T4, P_T5, P_T6, P_COM3, P_GET = range(16)


def s18(v):
    """截断到 18 bit 有符号 (Verilog 赋值时的隐式截断)"""
    v &= (1 << 18) - 1
    return v - (1 << 18) if v & (1 << 17) else v


def u18(v):
    return v & ((1 << 18) - 1)


class Parser:
    def __init__(self, kp=136, ki=10, kd=75, tgt=1810):
        self.ps = P_IDLE
        self.kind = 0
        self.isw = 0
        self.v10 = 0
        self.dot = 0
        self.frac_n = 0
        self.blen = 0
        # 复位默认值 (uart.v 第 349-355 行), 增益内部一律 x7
        self.gain_kp = u18(kp * 7)
        self.gain_kd = u18(kd * 7)
        self.gain_ki = u18(ki * 7)
        self.gain_sgn = 0
        self.gain_wkp = 0
        self.gain_wkd = 0
        self.tgt_q = tgt
        self.kpt_q, self.kit_q, self.kdt_q = kp, ki, kd
        self.echo = None
        self.rep_req = False

    def feed(self, data):
        """喂一个字节; 返回 True 表示该字节被当作包内容接受"""
        self.echo = None
        self.rep_req = False
        if data == ord('['):
            self.blen = 1
        elif self.blen < 16:
            self.blen += 1

        c = chr(data)
        ps = self.ps
        if ps == P_IDLE:
            if c == '[':
                self.ps = P_S
                self.v10 = 0
                self.dot = 0
                self.frac_n = 0
        elif ps == P_S:
            self.ps = P_COM1 if c == 's' else P_IDLE
        elif ps == P_COM1:
            self.ps = P_BRCH if c == ',' else P_IDLE
        elif ps == P_BRCH:
            if c == 'Z':
                self.isw = 0; self.ps = P_US
            elif c == 'W':
                self.isw = 1; self.ps = P_US
            elif c == 'G':
                self.ps = P_GET
            elif c == 't':
                self.ps = P_T2
            else:
                self.ps = P_IDLE
        elif ps == P_US:
            self.ps = P_K if c == '_' else P_IDLE
        elif ps == P_K:
            self.ps = P_KIND if c == 'K' else P_IDLE
        elif ps == P_KIND:
            if c == 'P':
                self.kind = 5 if self.isw else 1; self.ps = P_COM2
            elif c == 'D':
                self.kind = 6 if self.isw else 3; self.ps = P_COM2
            elif self.isw:
                self.ps = P_IDLE
            elif c == 'I':
                self.kind = 2; self.ps = P_COM2
            elif c == 'S':
                self.kind = 0; self.ps = P_COM2
            else:
                self.ps = P_IDLE
        elif ps == P_T2:
            self.ps = P_T3 if c == 'a' else P_IDLE
        elif ps == P_T3:
            self.ps = P_T4 if c == 'r' else P_IDLE
        elif ps == P_T4:
            self.ps = P_T5 if c == 'g' else P_IDLE
        elif ps == P_T5:
            self.ps = P_T6 if c == 'e' else P_IDLE
        elif ps == P_T6:
            if c == 't':
                self.kind = 4; self.ps = P_COM3
            else:
                self.ps = P_IDLE
        elif ps == P_COM2:
            self.ps = P_NUM if c == ',' else P_IDLE
        elif ps == P_COM3:
            self.ps = P_NUM if c == ',' else P_IDLE
        elif ps == P_GET:
            if c == ']':
                self.rep_req = True
                self.ps = P_IDLE
        elif ps == P_NUM:
            if c == ']':
                self.ps = P_IDLE
                self._commit()
            elif c == '.':
                self.dot = 1
            elif '0' <= c <= '9':
                if (not self.dot) or self.frac_n != 2:
                    self.v10 = (self.v10 * 10 + int(c)) & 0xFFFFFF
                    if self.dot:
                        self.frac_n += 1
            elif c == ',':
                self.ps = P_NUM
            else:
                self.ps = P_IDLE
        else:
            self.ps = P_IDLE
        return self.blen >= 1

    def _commit(self):
        gval = self.v10
        gval7 = ((gval << 3) - gval) & 0xFFFFFF
        v10x = self.v10 if self.dot else self.v10 * 10
        if self.kind == 4:                       # target
            self.tgt_q = min(v10x, 3600)
        elif self.kind == 5:                     # W_KP
            if gval <= 65535:
                self.gain_wkp = u18(gval)
        elif self.kind == 6:                     # W_KD
            if gval <= 65535:
                self.gain_wkd = u18(gval)
        elif gval <= 10000:
            if self.kind == 0:
                self.gain_sgn = 1 if gval else 0
            elif self.kind == 1:
                self.gain_kp = u18(gval7); self.kpt_q = gval & 0xFFFF
            elif self.kind == 2:
                self.gain_ki = u18(gval7); self.kit_q = gval & 0xFFFF
            elif self.kind == 3:
                self.gain_kd = u18(gval7); self.kdt_q = gval & 0xFFFF
        self.echo = self.blen + 1

    # ---- GET 读回 (uart.v 265-286 行) ----
    def get_report(self):
        # 已按修复后的 RTL: target 回满 16 bit (原来是 & 0x3FF, 只回 10 bit)
        vals = [self.kpt_q, self.kit_q, self.kdt_q, self.gain_sgn,
                self.tgt_q & 0xFFFF]
        names = "ABCDE"
        out = []
        for n, v in zip(names, vals):
            out.append("%s%04X" % (n, v & 0xFFFF))
        return " ".join(out)


def run(pkt, p=None):
    p = p or Parser()
    for ch in pkt.encode():
        p.feed(ch)
    return p


# ---------------------------------------------------------------- 直立环链
KI_DIV, ANG_INT_MAX = 5, 10000
POS_LIM = 100


def chain(deg_disp, gain_kp, gain_ki, gain_kd, tgt=1810, loc_out=0,
          int_ang=0, e_prev=0, d_ok=True, sgn=0):
    """返回 (s_x10, d_x10, guard, d_cnt, e_cnt, u_q)"""
    thd_m = int(round(deg_disp * 174.533))          # 显示角 -> 1e-4 rad
    s_x10 = (thd_m * 59) >> 10                       # 0.1 度
    d = s_x10 - tgt
    if d > 1800:
        d -= 3600
    if d < -1800:
        d += 3600
    guard = -450 < d < 450
    d_cnt = (d * 1165) >> 10
    e_cnt = s18(-loc_out - d_cnt)
    de = s18(e_cnt - e_prev) if d_ok else 0
    gkp = gain_kp & 0x1FFFF            # ctrl.v: gain_kp[16:0]  <- 只取 17 bit
    gki = gain_ki & 0x1FFFF
    gkd = gain_kd & 0x1FFFF
    ang_sum = e_cnt * gkp + int_ang * gki + de * gkd
    ang_out = ang_sum >> 14
    ang_out = max(-400, min(400, ang_out))
    u_bal = ang_out * 25 if sgn else -(ang_out * 25)
    u_q = u_bal if guard else 0
    return thd_m, s_x10, d, guard, d_cnt, e_cnt, u_q


# ---------------------------------------------------------------- 自检输出
def main():
    print("=" * 72)
    print("1. 文档里写的串口包, 逐个喂给 RTL 解析器")
    print("=" * 72)
    base = Parser()
    groups = []
    for pkt in ("[s,Z_KP,136]", "[s,Z_KI,10]", "[s,Z_KD,75]",
                "[s,target,181]", "[s,Z_SGN,1]", "[s,W_KP,1638]",
                "[s,W_KD,16384]", "[s,GET,1]"):
        p = run(pkt, base)          # 累加, 模拟同一块板连续调参
        base = p
        ok = "接受" if p.echo is not None or p.rep_req else "!! 被丢弃 !!"
        print("  %-18s -> %-10s  gain_kp=%d gain_ki=%d gain_kd=%d "
              "sgn=%d tgt=%d wkp=%d wkd=%d"
              % (pkt, ok, p.gain_kp, p.gain_ki, p.gain_kd,
                 p.gain_sgn, p.tgt_q, p.gain_wkp, p.gain_wkd))

    print()
    print("=" * 72)
    print("2. GET 读回 (参数从板子上抄回来)")
    print("=" * 72)
    print("  板子实际: Kp=136 Ki=10 Kd=75 sgn=0 target=1810")
    print("  GET 回一行: %s" % base.get_report())
    print("  E 字段应该是 0712 (=1810); 修复前只回 10 bit -> 0312 (=786)")

    print()
    print("=" * 72)
    print("3. 串口填值 vs 写进 params.vh, 增益是否一致")
    print("=" * 72)
    p = Parser()
    for pkt in ("[s,Z_KP,136]", "[s,Z_KI,10]", "[s,Z_KD,75]"):
        p = run(pkt, p)
    code = Parser()          # params.vh 默认值走复位路径
    print("  串口调: kp=%d ki=%d kd=%d" % (p.gain_kp, p.gain_ki, p.gain_kd))
    print("  代码里: kp=%d ki=%d kd=%d" % (code.gain_kp, code.gain_ki,
                                           code.gain_kd))
    print("  ctrl 实际取 [16:0]: 串口 %d/%d/%d  vs 代码 %d/%d/%d"
          % (p.gain_kp & 0x1FFFF, p.gain_ki & 0x1FFFF, p.gain_kd & 0x1FFFF,
             code.gain_kp & 0x1FFFF, code.gain_ki & 0x1FFFF,
             code.gain_kd & 0x1FFFF))

    print()
    print("  -- 大增益会怎样 (>9362, 串口允许到 10000) --")
    for n in (5000, 9362, 9363, 10000):
        q = Parser()
        q = run("[s,Z_KP,%d]" % n, q)
        used = q.gain_kp & 0x1FFFF
        eff = n * 7
        print("    Z_KP=%-6d -> 寄存器 %-6d (7x=%-6d), ctrl 只用低17位 = %-6d%s"
              % (n, q.gain_kp, eff, used,
                 "   <-- 回绕!" if used != eff else ""))

    print()
    print("=" * 72)
    print("4. 直立环整条链的数值 (Kp=136 Ki=10 Kd=75, 目标181.0度)")
    print("=" * 72)
    print("  摆角(显示)  thd_m   s_x10  d_x10  保护  d_cnt  e_cnt   u_q(1e-4)  %")
    for deg in (176, 178, 180, 181, 182, 184, 186, 190, 200, 270):
        t, s, d, g, dc, e, u = chain(deg, 952, 70, 525)
        print("   %6.1f    %6d  %5d  %5d  %3s  %5d  %5d   %7d  %5.2f"
              % (deg, t, s, d, "Y" if g else "N", dc, e, u, u / 100.0))


if __name__ == "__main__":
    main()
