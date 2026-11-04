# -*- coding: utf-8 -*-
"""放大看直立环输出量化 (ang_out*25 + >>>14 截断) 造成的不对称死区.
稳态 (角度不变) 时 de=0, 只看 P 项经过 >>>14 再 *25 的结果."""
import io
import sys

sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding="utf-8")
from check_uart_chain import chain

KP, KI, KD = 952, 70, 525          # params.vh 136 / 10 / 75, 内部 x7


def steady(deg):
    """稳态: e_prev = e_cnt  -> de = 0"""
    _, s, d, g, dc, e, _ = chain(deg, KP, KI, KD, e_prev=0)
    return chain(deg, KP, KI, KD, e_prev=e, d_ok=True)


print("稳态扫描 (微分项=0), 目标 181.0 度")
print("显示角   d_x10  d_cnt  e_cnt   u_q   占空比%")
for i in range(1780, 1831, 5):
    deg = i / 10.0
    _, s, d, g, dc, e, u = steady(deg)
    print(" %6.1f   %5d  %5d  %6d  %5d  %7.2f" % (deg, d, dc, e, u, u / 100.0))

print()
print("死区边界 (输出第一次离开 0):")
for tag, rng in (("偏小侧", range(1810, 1740, -1)),
                 ("偏大侧", range(1810, 1880))):
    for i in rng:
        deg = i / 10.0
        _, s, d, g, dc, e, u = steady(deg)
        if u != 0:
            print("  %s: 偏离 %.1f 度 -> u_q=%d (%.2f%%)"
                  % (tag, abs(deg - 181.0), u, u / 100.0))
            break

print()
print("对照: 同样的误差, 直通 (round 而不是 floor) 会输出多少")
print("  e_cnt=+17 (偏小 1.5 度): 现在 %d, round 后 %d"
      % (steady(179.5)[6], round(17 * 952 / 16384) * 25))
print("  e_cnt=-17 (偏大 1.5 度): 现在 %d, round 后 %d"
      % (steady(182.5)[6], -round(17 * 952 / 16384) * 25))
