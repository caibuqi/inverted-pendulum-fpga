def chain(deg, kp=19, tgt=1810, guard=450):
    thd_m = int(round(deg * 174.533))      # display angle, 1e-4 rad
    s_x10 = (thd_m * 59) >> 10             # -> 0.1 deg units
    e = tgt - s_x10
    if e > 1800:
        e -= 3600
    if e < -1800:
        e += 3600
    guard_ok = (-guard < e < guard)
    pid = e * kp
    out = -pid                             # gain_sgn = 0 -> negate
    out = max(-10000, min(10000, out))
    return thd_m, s_x10, e, guard_ok, out


print("angle  thd_m  s_x10  e(0.1deg)  guard   PWM")
for d in (0, 90, 136, 150, 176, 181, 186, 200, 226, 270, 359):
    thd_m, s, e, g, o = chain(d)
    print("%5d  %6d  %5d  %9d  %5s  %5d" % (d, thd_m, s, e, str(g), o))
