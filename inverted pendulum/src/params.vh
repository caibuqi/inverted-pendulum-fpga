// =====================================================================
//  params.vh  --  every tunable constant of the pendulum controller
//  Platform: Pocket Lab-F1 / GW1N-UV9EQ144C6 / 50 MHz
//  ---------------------------------------------------------------------
//  Fixed point conventions
//      phi, theta        : 1e-4 rad   per LSB   (pi = 31416)
//      duty              : 1e-4       per LSB   (10000 = 100 %)
//  Measured machine data
//      r = 0.070 m   m = 0.0294 kg   l = 0.061 m   A = 1.4479e-4 kg m^2
//      Kt = 0.373 N m   Ke = 0.006125   gear = 20
//      encoder 13 PPR -> 1040 counts per arm revolution
//      SA100 period 1024 us
// =====================================================================
`ifndef PARAMS_VH
`define PARAMS_VH

// ---------------- clock and timing ----------------
`define TICK_CLK      50000        // 1 kHz control tick  @50 MHz
`define PWM_CLK       2500         // 20 kHz PWM          @50 MHz

// ---------------- run switch (DIP 1) ----------------
// manual table 3-12: switch UP = high, switch DOWN = low.
// K1 on this board does not register, so the DIP switches take over:
//      SW1 (pin 75) UP -> 启摆 + 平衡
//      SW2 (pin 76) UP -> 只平衡, 不起摆 (手扶到竖直再推)
`define SW_LOW_ACTIVE 0            // 1 -> invert if the board reads the other way

// ---------------- serial port (Type-C / CH340, table 3-13) ----------------
//   FPGA_RXD = pin 49,  FPGA_TXD = pin 48
//   115200 8N1  ->  50 MHz / 115200 = 434 clocks per bit
`define BAUD_DIV      434

// ---------------- sensor scaling ----------------
// phi(1e-4 rad) = pos * ENC_K >> ENC_SH     (2*pi/1040 = 60.407 per count)
`define ENC_K         20'sd61857
`define ENC_SH        10
// SA100 watchdog: no PWM edge for this many clocks -> sensor lost
`define SA_TIMEOUT    200000       // 4 ms with no PWM -> sensor lost

// =====================================================================
//  平衡控制  --  完全照抄 江协科技《16-倒立摆-自动启摆》的 main.c
// ---------------------------------------------------------------------
//  示例的角度传感器是 12 位 AD (0~4095 = 一整圈), 所以
//      1 个计数 = 360/4096 = 0.0879 度
//  我们这边统一用 0.1 度, 进 ctrl.v 时换算:
//      d_cnt = (0.1 度偏差 * 1165) >> 10
//
//  示例 PID 输出范围 +-100 -> 占空比 %; 这里 *100 -> PWM 值 (10000 = 100%)
//
//  ---- 内环 角度环, 每 5 ms ----
//      e    = 目标 - 实际                 [计数]
//      Out  = Kp*e + Ki*∫e + Kd*(e - e上次)     限幅 +-100
//  ---- 外环 位置环, 每 50 ms ----
//      Loc  = 从进入平衡起手臂转过的编码器计数 (408 计数 = 一圈)
//      Out2 = Kp2*e2 + Kd2*(e2 - e2上次)        限幅 +-100 计数 (= +-8.8 度)
//      角度环目标 = 中心角度 + Out2   (位置环输出, 同号; 见 ctrl.v 的 e_cnt)
//
//  串口里的增益一律填【整数】, 和本文件里的 KP_ANG/KI_ANG/KD_ANG 是同一个数,
//  两个地方填一样的数字, 效果就完全一样 (内部都会再 x7, 见 ctrl.v):
//      [s,Z_KP,140]   <->  `define KP_ANG 32'sd140
//      [s,Z_KI,5]     <->  `define KI_ANG 32'sd5
//      [s,Z_KD,26]    <->  `define KD_ANG 32'sd26
//      换算: 实际增益 = 数字 / 9362  (Kp: 140 -> 0.0149 占空比%/计数)
//      格式: "[" "s" "," 名字 "," 数字 "]"  —— 数字后面不要加逗号
//      范围 0 ~ 10000, 打 "140.0" 也等价 (小数点后最多 2 位)
// =====================================================================
`define ANG_MS        10           // 角度环周期 (ms)  10 ms = 100 Hz
`define POS_MS        50           // 位置环周期 (ms)
// ---- 直立环增益 ------------------------------------------------------
// ★ 现在的值是现场串口调出来的 (Z_KP=140 / Z_KI=5 / Z_KD=26),
//   填的就是串口里那个"整数", 内部再 x7, 不要自己先乘.
// 单位换算: 1 度误差 -> 占空比% = Kp*11.38/9362 = Kp/823
//      Kp=140 -> 0.170 %/度
// 参考 (按本机实测参数 m=0.0294 l=0.061 A=1.4479e-4 Kt=0.373 反算):
//      倒立发散强度 m*g*l/A          = 122 s^-2/rad
//      每 1 % 占空比给出的矫正加速度 = 9.4 s^-2/rad
//      -> 抵消发散约需 Kp=186; 3 倍余量约 557.  (J_arm 没实测, 有 2~3 倍
//         不确定, 所以这条只当参考; 现场能站住的数就是对的)
`define KP_ANG        32'sd140     // 角度环 Kp
`define KI_ANG        32'sd5       // 角度环 Ki
`define KD_ANG        32'sd26      // 角度环 Kd
// 积分限幅 (单位: 计数 x 次). 它对占空比的贡献 = ANG_INT_MAX * Ki * 7 * 25
//   / 16384 / 100 %.  按现在的 Ki=5:
//        1000 -> 0.53 %      3000 -> 1.60 %      5000 -> 2.67 %
//       10000 -> 5.34 %  (= Kp=140 下 31 度误差的作用, 基本等于不限幅)
//   积分只用来消静差, 不该有主导权, 所以取 3000 (约等于 9 度误差的 P 作用).
//   在 +-20 度里 e_cnt 最大约 228, 每 50 ms 加一次 -> 约 0.7 s 才顶到 3000.
`define ANG_INT_MAX   20'sd3000    // 角度环积分限幅 (计数 x 次)
`define KI_DIV        5            // Ki 削弱: 每 KI_DIV 个角度环周期才累加一次
                                   //   ANG_MS=10 -> 每 50 ms 积分一次
                                   //   (保持和原来 5ms/10 分频一样的作用)
`define KP_POS_P      32'sd0       // 位置环 Kp2 = 0.00  (Q12, 置 0)
`define KD_POS_P      32'sd0       // 位置环 Kd2 = 0.00  (Q12, 置 0)
`define POS_LIM       18'sd100     // 位置环输出限幅 +-100 计数 (约 +-8.8 度)
`define POS_STEP_S    18'sd51      // 按键微调 1/8 圈 (45 度)
`define POS_STEP_L    18'sd408     // 按键微调 1 圈
`define POS_TGT_LIM   18'sd4080    // 位置目标限幅 +-10 圈

// ---------------- 直立中心 / 启摆 (判断状态) ----------------
// 示例: CENTER_ANGLE = 2010 (直立), CENTER_RANGE = 500 计数
//       500 计数 = 44.0 度  -> 这里 439 (0.1 度)
`define BAL_TARGET_Q  18'sd1810    // 直立中心 = 摆杆 181.0 度 (0.1 度单位)
`define BAL_RANGE     18'sd439     // 启摆用: 找最高点的区间 +-43.9 度
// ---- 起摆 -> 平衡 的交棒窗口 (显示角 150 度 ~ 210 度) ----
//   d_x10 是相对直立中心的偏差 (0.1 度), 显示角 = 180 + d_x10/10.056,
//   所以 +-302 就是显示角 150 度 ~ 210 度 (+-30 度).
//   注意: GUARD_DEG 必须不小于 CAP_RANGE, 否则抓到的那一瞬间保护就把
//   PWM 清零了, 等于白抓.
`define CAP_RANGE     18'sd302     // 交棒窗口 +-30.0 度 (显示 150 ~ 210)
// ---- 积分窗口 (Z_KI 只在 +-10 度内积累) ----
// 光"进了 mode 2"还不足以积分: 交棒可能在 30 度, 那时 e_cnt 有 340 个计数,
// 积分会积累得很快, 等摆杆被拉回直立位会猛踢一脚.
// 所以再加一层: 只有摆杆进到 +-INT_WIN 才允许积累积分, 出了窗口立刻清零.
`define INT_WIN       18'sd100     // +-10.0 度: Z_KI 只在这个范围内积累
`define BAL_GUARD_EN  1            // 保护开关: 1 = 打开, 0 = 关闭
`define GUARD_DEG     18'sd650     // 保护阈值: 直立环"输入"(摆杆相对直立中心的
                                   //   差值)绝对值超过 65.0 度 -> PWM 输出 0;
                                   //   差值回到 65 度以内自动恢复出力 (不停机)
                                   //   必须 >= CAP_RANGE, 否则交棒后立刻被清零
`define SW_SAMPLE_MS  40           // 判断状态: 每 40 ms 采样一次角度
`define CAP_CONFIRM_MS 5            // 起摆->平衡的交棒确认时间:
                                   //   连续 5 ms 都在中心区间内就切 PID.
                                   //   (以前只在每 40 ms 采样点上判, 而且踢腿
                                   //    的 200 ms 里不采样 -> 摆过窗口经常抓不到)
`define SW_PUSH_DUTY  18'sd3500    // 起摆力度 35 % (示例 START_PWM = 35)
`define SW_PUSH_MS    100          // 第一脚 100 ms (示例 START_TIME = 100)
`define SW_BACK_MS    100          // 回位脚 100 ms
`define SW_RETRY_MS   15000        // 15 s 还没起来就放弃 (只起摆一次)
`define SW_IDLE_MS    600          // 摆杆一直不动 600 ms -> 先踢一脚脱离死点

// ---------------- SA100 zero ----------------
// Measured on this machine:  hanging = 303 deg,  upright = 123 deg
//   theta = raw - 123 deg      (123 deg = 21468 in 1e-4 rad)
// SA_AUTOCAL 1 = learn the offset at power-up instead (the pendulum must
// then hang still while the board boots).
`define SA_ZERO_Q     21468       // 123.0 deg (used only when SA_AUTOCAL = 0)
`define SA_AUTOCAL    1           // 1 = learn the zero at power-up from the
                                   //     hanging position (pendulum must hang
                                   //     still for 500 ms after reset)

// ---------------- look after the buttons / bench test ----------------
// open-loop jog level used in display mode 3, in 1e-4 duty units
`define JOG_DUTY      3000         // 30 %

// ---------------- polarity switches (measure on hardware, then set) ----
// If the closed loop pushes the pendulum the WRONG way, exactly one of
// these is inverted.  Change it to 1 and rebuild (20 s) - no re-wiring.
`define MOTOR_SWAP    0    // 1 = invert motor direction
`define ENC_SWAP      0    // 1 = swap encoder A/B
`define THETA_SWAP    0    // 1 = invert SA100 angle sign (balance + swing-up + display)

// ---------------- balance output sign ----------------
// 串口 [s,Z_SGN,1] 改的就是这个方向位. 以前它没有编译期默认值,
// 所以"串口里翻转过的方向"一重编就丢了 (回到 0) —— 调好的效果复现不出来.
// 串口里确认好方向后, 把 1 抄到这里, 重新编译就固定下来了.
`define BAL_SGN       0    // 0 = 默认 (摆杆偏向哪边, 手臂就往哪边追)

// ---------------- display ----------------
// SMG_DIG1 (pin 137) is the LEFTMOST digit on this board, so the scan
// order has to be reversed to read left-to-right.  Set to 0 if it ever
// comes out the other way round.
`define SEG_DIG_REV   1

// ---------------- position (preset display) ----------------
// preset lives in [0, 2*pi) and steps wrap around, so the control error
// computed as fold(phi - phi_ref) always takes the shortest way round.
`define STEP5         20'sd873     //  5 deg = 0.0873 rad
`define STEP30        20'sd5236    // 30 deg = 0.5236 rad
`define REF_STEP      10           // phi_ref slew, 10 units per ms = 1 rad/s

`endif
