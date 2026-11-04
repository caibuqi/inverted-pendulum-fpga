// =====================================================================
//  ctrl.v  --  control core + 20 kHz PWM
//  ---------------------------------------------------------------------
//  结构完全照抄 江协科技《16-倒立摆-自动启摆》的 main.c
//
//      mode 0  停止        电机 PWM = 0
//      mode 1  判断 / 启摆   每 40 ms 采样一次角度, 连续 3 次都在同一侧,
//                          且中间那次是极值 -> 摆杆到了那一侧的最高点:
//                              先朝另一侧踢一脚 (SW_PUSH_MS)
//                              再反向回位       (SW_BACK_MS)
//                          连续 2 次采样落进中心区间 -> 进入平衡
//      mode 2  平衡        5 ms 角度环 + 50 ms 位置环 (串级)
//
//  角度单位: SA100 0.1 度, 进平衡环换算成示例的"计数" (360/4096 度 = 1 计数)
//  输出单位: u_q 是 PWM 数值, 10000 = 100 % 占空比
// =====================================================================
`include "params.vh"


module ctrl_core (
    input  wire               clk,
    input  wire               rst_n,
    input  wire signed [17:0] phi_q,        // arm angle
    input  wire        [17:0] th_raw,       // SA100 raw, 0..2pi
    input  wire               th_ok,
    input  wire               ev_start,      // run switch rising edge (SW1 or SW2)
    input  wire               run_sw,        // 1 = run (SW1 or SW2 is UP)
    input  wire               mode_sw,       // 1 = SW2 UP: BALANCE ONLY (no swing-up)
    input  wire               ev_up5,
    input  wire               ev_dn5,
    input  wire               ev_up30,
    input  wire               ev_dn30,
    input  wire               ev_save,
    input  wire               ev_reset,
    input  wire               jog_up,        // open-loop jog, +JOG_DUTY (mode 3)
    input  wire               jog_dn,        // open-loop jog, -JOG_DUTY
    input  wire        [17:0] gain_kp,       // 角度环 Kp  (串口, x1000 -> 数字)
    input  wire        [17:0] gain_kd,       // 角度环 Kd
    input  wire        [17:0] gain_ki,       // 角度环 Ki
    input  wire        [17:0] gain_wkp,      // 位置环 Kp2 (串口, Q12)
    input  wire        [17:0] gain_wkd,      // 位置环 Kd2 (串口, Q12)
    input  wire        [17:0] gain_sgn,      // bit0: 翻转平衡输出方向
    input  wire        [17:0] bal_target,    // 直立中心角度 (串口, 0.1 度)
    output reg  signed [17:0] u_q,          // duty, signed
    output reg         [1:0]  mode,         // 0 STOP, 1 SWING-UP, 2 BALANCE
    output wire        dbg_guard,     // 1 = 摆杆还在中心区间里
    output wire        dbg_cal,       // 定零是否完成
    output wire [17:0] dbg_ang,       // 控制器看到的摆杆角度 (0.1 度)
    output reg  signed [17:0] preset_q,
    output reg         [17:0] th_disp_q     // theta for the display:
                                            //   0 deg at the HANGING position
                                            // 180 deg at the upright position
);

    // =================================================================
    //  helper: signed saturation
    // =================================================================
    function signed [17:0] sat18;
        input signed [39:0] v;
        input signed [17:0] lim;
        begin
            if (v > lim)          sat18 = lim;
            else if (v < -lim)    sat18 = -lim;
            else                  sat18 = v[17:0];
        end
    endfunction

    // =================================================================
    //  1 kHz tick
    // =================================================================
    reg [15:0] tick_div;
    reg        tick;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            tick_div <= 16'd0;
            tick     <= 1'b0;
        end else if (tick_div == `TICK_CLK - 1) begin
            tick_div <= 16'd0;
            tick     <= 1'b1;
        end else begin
            tick_div <= tick_div + 16'd1;
            tick     <= 1'b0;
        end
    end

    // =================================================================
    //  SA100 zero.
    //  上电后摆杆静止 500 ms -> 把当前位置学成"下垂位 0 度"。
    //  SW1 起摆时再学一次; SW2 (只平衡) 不动零点; K8 重新学。
    // =================================================================
    reg [11:0] cal_ms;
    reg        cal_done;
    reg [17:0] offset;      // raw reading when the pendulum is UPRIGHT
    reg [17:0] raw_bot;     // raw reading when the pendulum HANGS DOWN

    // (th_raw + pi) mod 2*pi  -- keeps a learned offset in [0, 2*pi)
    wire [18:0] off_sum = {1'b0, th_raw} + 19'd31416;
    wire [18:0] off_mod = (off_sum >= 19'd62832) ? (off_sum - 19'd62832) : off_sum;

    // 定零采样参考值 (用来判断摆杆是否真的停稳了)
    reg [17:0] cal_ref;
    reg        cal_ref_v;
    reg [9:0]  cal_ms2;
    // |th_raw - cal_ref| 折到 +-180 度
    wire signed [19:0] cal_dev = $signed({2'b00, th_raw}) - $signed({2'b00, cal_ref});
    reg  signed [19:0] cal_dev_w;
    always @(*) begin
        cal_dev_w = cal_dev;
        if (cal_dev_w >  20'sd31416) cal_dev_w = cal_dev_w - 20'sd62832;
        if (cal_dev_w < -20'sd31416) cal_dev_w = cal_dev_w + 20'sd62832;
    end
    // 500 ms 内变化 < 5 度 就认为停稳了
    wire cal_still = (cal_dev_w < 20'sd873) && (cal_dev_w > -20'sd873);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            cal_ms2   <= 10'd0;
            cal_ref   <= 18'd0;
            cal_ref_v <= 1'b0;
            cal_done  <= 1'b0;
            offset    <= 18'd0;
            raw_bot   <= 18'd0;
        end else if (ev_reset) begin
            cal_ms2   <= 10'd0;
            cal_ref   <= 18'd0;
            cal_ref_v <= 1'b0;
            cal_done  <= 1'b0;
            offset    <= 18'd0;
            raw_bot   <= 18'd0;
        end else if (ev_start) begin
            // SW1 (起摆模式): 以当前摆位作为下垂参考 0 度
            // SW2 (只平衡):   不动零点, 沿用上电学到的
            if (!mode_sw) begin
                cal_done <= 1'b1;
                offset   <= off_mod[17:0];
                raw_bot  <= th_raw;
            end
        end else if (`SA_AUTOCAL && tick && !cal_done && th_ok) begin
            // ---------------- 上电自动定零 (静止检测) ----------------
            if (!cal_ref_v) begin
                cal_ref   <= th_raw;
                cal_ref_v <= 1'b1;
                cal_ms2   <= 10'd0;
            end else if (cal_ms2 >= 10'd500) begin
                cal_ms2 <= 10'd0;
                if (cal_still) begin
                    cal_done <= 1'b1;
                    offset   <= off_mod[17:0];
                    raw_bot  <= th_raw;
                end else begin
                    cal_ref <= th_raw;      // 不稳定 -> 重新取样
                end
            end else begin
                cal_ms2 <= cal_ms2 + 10'd1;
            end
        end
    end

    // ---- angle for the display: 0 at hanging, 180 at upright ----
    wire signed [19:0] thd_d = $signed({{2{th_raw[17]}},  th_raw })
                             - $signed({{2{raw_bot[17]}}, raw_bot});
    reg  signed [19:0] thd_m;
    always @(*) begin
        thd_m = thd_d;
        if (thd_m < 0) thd_m = thd_m + 20'sd62832;
    end
    wire [17:0] thd_out = `THETA_SWAP ? (18'd62832 - thd_m[17:0]) : thd_m[17:0];

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)         th_disp_q <= 18'd0;
        else if (ev_reset)  th_disp_q <= 18'd0;
        else if (tick)      th_disp_q <= thd_out;
    end

    // =================================================================
    //  preset register (按键), 只用于显示
    // =================================================================
    function [17:0] wrap2pi;
        input signed [19:0] v;
        reg signed [19:0] t;
        begin
            t = v;
            if (t >= 20'sd62832) t = t - 20'sd62832;
            if (t <  20'sd0)     t = t + 20'sd62832;
            wrap2pi = t[17:0];
        end
    endfunction

    wire signed [19:0] pre_up5  = $signed({2'b00, preset_q}) +  `STEP5;
    wire signed [19:0] pre_dn5  = $signed({2'b00, preset_q}) -  `STEP5;
    wire signed [19:0] pre_up30 = $signed({2'b00, preset_q}) + `STEP30;
    wire signed [19:0] pre_dn30 = $signed({2'b00, preset_q}) - `STEP30;

    // NOTE: the ev_* pulses are ONE clock wide (20 ns), handled on every
    // clock - never inside the 1 kHz tick.
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            preset_q <= 18'sd0;
        end else if (ev_reset) begin
            preset_q <= 18'sd0;
        end else begin
            if      (ev_up5)  preset_q <= wrap2pi(pre_up5);
            else if (ev_dn5)  preset_q <= wrap2pi(pre_dn5);
            else if (ev_up30) preset_q <= wrap2pi(pre_up30);
            else if (ev_dn30) preset_q <= wrap2pi(pre_dn30);
        end
    end

    // =================================================================
    //  run switch latch
    // =================================================================
    reg start_run;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)        start_run <= 1'b0;
        else if (ev_reset) start_run <= 1'b0;
        else if (tick)     start_run <= run_sw;
    end

    // SA100 有效信号的保持: 偶尔丢一帧不打断平衡
    reg [9:0] th_hold;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)         th_hold <= 10'd0;
        else if (ev_reset)  th_hold <= 10'd0;
        else if (tick) begin
            if (th_ok)                  th_hold <= 10'd500;      // 有效 -> 续 500 ms
            else if (th_hold != 10'd0)  th_hold <= th_hold - 10'd1;
        end
    end
    wire th_valid = (th_hold != 10'd0);

    // =================================================================
    //  摆杆角度 (0.1 度) 与相对直立中心的偏差
    // =================================================================
    reg  signed [17:0] s_x10;
    reg  signed [17:0] tgt_base;
    reg  signed [17:0] d_x10;           // 摆杆偏差, 折到 +-180.0 度

    wire signed [19:0] s_lsb = $signed({2'b00, thd_m[17:0]});       // 1e-4 rad
    wire [17:0] tgt_use = (bal_target > 18'd100 && bal_target < 18'd3500)
                        ? bal_target : `BAL_TARGET_Q;

    always @(*) begin
        // 注意: 乘法必须在 32 位里做!
        //   s_lsb 最大 62832, x59 = 3.7e6, 20 位装不下会溢出 ->
        //   以前写成 (s_lsb * 20'sd59) 只有 20 位, 角度一大就变成乱数,
        //   于是 d_x10 乱掉 -> bal_win 假 -> 平衡模式立刻被判"倒下" -> 电机不动
        s_x10    = (s_lsb * 32'sd59) >>> 10;                        // 0.1 度
        tgt_base = $signed(tgt_use);
        d_x10    = s_x10 - tgt_base;
        if (d_x10 >  18'sd1800) d_x10 = d_x10 - 18'sd3600;
        if (d_x10 < -18'sd1800) d_x10 = d_x10 + 18'sd3600;
    end

    // ---- 保护: 直立环的"输入"(摆杆相对直立中心的差值)绝对值超过 45 度
    //      就认为没必要再出力了, PWM 直接给 0; 差回到 45 度以内自动恢复 ----
    wire bal_win  = (`BAL_GUARD_EN == 0) ? 1'b1
                  : ((d_x10 < `GUARD_DEG) && (d_x10 > -`GUARD_DEG));
    wire bot_win  = (d_x10 >  (18'sd1800 - `BAL_RANGE)) ||
                    (d_x10 < -(18'sd1800 - `BAL_RANGE));   // 摆杆基本在下垂位

    assign dbg_guard = bal_win;
    assign dbg_cal   = cal_done;
    assign dbg_ang   = s_x10[17:0];

    // 0.1 度 -> 示例计数 (1 计数 = 360/4096 度):  x1165>>10
    wire signed [31:0] d_cnt32 = (d_x10 * 32'sd1165) >>> 10;
    wire signed [17:0] d_cnt   = d_cnt32[17:0];

    // =================================================================
    //  位置环 (外环)  --  输入: 手臂从"进入平衡"起转过的编码器计数
    //                      408 计数 = 一圈 (示例的编码器刻度)
    //                   输出: 角度环目标的偏移, 限幅 +-100 计数
    // =================================================================
    reg  signed [19:0] phi_cap;         // 进入平衡那一刻的手臂角度
    reg  signed [17:0] pos_tgt;         // 位置目标 (计数), 按键可调
    reg  signed [17:0] e_loc_prev;

    wire signed [19:0] dphi_raw = $signed({2'b00, phi_q}) - phi_cap;
    reg  signed [19:0] dphi_w;
    always @(*) begin
        dphi_w = dphi_raw;
        if (dphi_w >  20'sd31416) dphi_w = dphi_w - 20'sd62832;
        if (dphi_w < -20'sd31416) dphi_w = dphi_w + 20'sd62832;
    end
    // (1e-4 rad) -> 计数:  408/62832 = 0.0064935 ; 213/32768 = 0.0065002
    wire signed [31:0] loc32   = (dphi_w * 32'sd213) >>> 15;
    wire signed [17:0] loc_cnt = loc32[17:0];

    wire signed [17:0] e_loc = pos_tgt - loc_cnt;
    wire signed [17:0] de_loc = e_loc - e_loc_prev;

    // ---- 位置环: 硬件乘法器只有一个, 分两拍算 ----
    //   (位置环 50 ms 才更新一次, 分两拍完全来得及;
    //    这样省下一个 DSP, 否则整个设计放不进 GW1N-9)
    reg  [1:0]         pos_st;
    reg  [5:0]         div50;
    reg  signed [17:0] pm_a, pm_b;
    reg  signed [39:0] pos_acc;
    reg  signed [17:0] loc_out;
    wire signed [35:0] pm_now = pm_a * pm_b;        // 位置环唯一的乘法器

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pos_st   <= 2'd0;
            div50    <= 6'd0;
            pm_a     <= 18'sd0;
            pm_b     <= 18'sd0;
            pos_acc  <= 40'sd0;
            loc_out  <= 18'sd0;
        end else if (ev_reset) begin
            pos_st   <= 2'd0;
            div50    <= 6'd0;
            pm_a     <= 18'sd0;
            pm_b     <= 18'sd0;
            pos_acc  <= 40'sd0;
            loc_out  <= 18'sd0;
        end else if (tick) begin
            if (mode != 2'd2) begin
                pos_st     <= 2'd0;
                div50      <= 6'd0;
                loc_out    <= 18'sd0;
                e_loc_prev <= 18'sd0;
            end else begin
                case (pos_st)
                2'd0: if (div50 >= `POS_MS - 1) begin
                          div50  <= 6'd0;
                          pm_a   <= e_loc;
                          pm_b   <= $signed({1'b0, gain_wkp[16:0]});
                          pos_st <= 2'd1;
                      end else begin
                          div50 <= div50 + 6'd1;
                      end
                2'd1: begin
                          // pm_now = e_loc * Kp2
                          pos_acc <= {{4{pm_now[35]}}, pm_now};
                          pm_a    <= de_loc;
                          pm_b    <= $signed({1'b0, gain_wkd[16:0]});
                          pos_st  <= 2'd2;
                      end
                2'd2: begin
                          // pm_now = de_loc * Kd2   -> 加起来出结果
                          loc_out    <= sat18((pos_acc + {{4{pm_now[35]}}, pm_now})
                                              >>> 12, `POS_LIM);
                          e_loc_prev <= e_loc;      // 供下一轮算 50ms 差分
                          pos_st     <= 2'd0;
                      end
                default: pos_st <= 2'd0;
                endcase
            end
        end
    end

    // 按键微调位置目标 (K3/K4 = 1/8 圈, K5/K6 = 1 圈)
    wire signed [17:0] ptgt_up5 = pos_tgt + `POS_STEP_S;
    wire signed [17:0] ptgt_dn5 = pos_tgt - `POS_STEP_S;
    wire signed [17:0] ptgt_upL = pos_tgt + `POS_STEP_L;
    wire signed [17:0] ptgt_dnL = pos_tgt - `POS_STEP_L;
    function signed [17:0] clamp_tgt;
        input signed [17:0] v;
        begin
            if (v >  `POS_TGT_LIM) clamp_tgt =  `POS_TGT_LIM;
            else if (v < -`POS_TGT_LIM) clamp_tgt = -`POS_TGT_LIM;
            else clamp_tgt = v;
        end
    endfunction

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pos_tgt <= 18'sd0;
        end else if (ev_reset) begin
            pos_tgt <= 18'sd0;
        end else if (ev_save) begin
            pos_tgt <= 18'sd0;              // K7: 目标回到 0 (原地)
        end else begin
            if      (ev_up5)  pos_tgt <= clamp_tgt(ptgt_up5);
            else if (ev_dn5)  pos_tgt <= clamp_tgt(ptgt_dn5);
            else if (ev_up30) pos_tgt <= clamp_tgt(ptgt_upL);
            else if (ev_dn30) pos_tgt <= clamp_tgt(ptgt_dnL);
        end
    end

    // =================================================================
    //  角度环 (内环)
    //  增益: 串口收到的数字已经在这里之前 x7 了 (uart.v 里移位减法做的),
    //        所以这里直接用: 实际增益 = 串口数字 x 7 / 65536 = 数字 / 9362
    // =================================================================
    wire signed [17:0] gkp16 = $signed({1'b0, gain_kp[16:0]});   // 串口里已经 x7 了
    wire signed [17:0] gkd16 = $signed({1'b0, gain_kd[16:0]});
    wire signed [17:0] gki16 = $signed({1'b0, gain_ki[16:0]});

    // ---- 位置环输出如何进入角度环: 同号, 不能取反 ----
    //   e_cnt = 角度给定 - 角度实际, 而"角度给定"就是 loc_out 本身.
    //   为什么同号:
    //     要手臂往 +phi 走  ->  摆臂必须加速 +phi
    //     由 theta'' = (m*g*l/A)*theta - (c/A)*phi'' 可知, 加速 +phi 需要
    //     摆杆先偏 +theta; 而直立环对 +theta 恰好给正占空比 (才有正 phi'')
    //     => 想让手臂往哪边走, 角度给定就要往哪边偏.
    //   以前写成 (-loc_out) - d_cnt, 符号取反了: 手臂离目标越远, 给定偏得
    //   越反, 结果被推得离目标更远 (W_KP 一加上就往外跑).
    wire signed [17:0] e_cnt = loc_out - d_cnt;         // 目标 - 实际
    reg  signed [17:0] e_cnt_prev;
    // 进入平衡后的第一拍不算微分 (否则 e_cnt_prev 还是旧值, 会踢一大脚)
    reg               d_ok;
    wire signed [17:0] de_cnt = d_ok ? (e_cnt - e_cnt_prev) : 18'sd0;

    reg  signed [19:0] int_ang;                         // 积分 (每 5 ms 加一次)

    // ---- 积分门 ----
    //   mode 0/1 本来就不积分 (见下面的状态机), 但进入平衡后的头几百毫秒
    //   摆杆可能还在 40~60 度的位置, 这时 e_cnt 很大, 积分会迅速顶满.
    //   所以只允许在 +-INT_WIN 内积分, 出窗口立刻清零 (不是冻结).
    wire int_ok = (d_x10 < `INT_WIN) && (d_x10 > -`INT_WIN);
    // 积分累加后的值 (21 bit, 先加再钳用; 直接加在 int_ang 上会丢溢出)
    wire signed [20:0] int_next = {{1{int_ang[19]}}, int_ang}
                                + {{3{e_cnt[17]}},  e_cnt};

    wire signed [39:0] ang_sum =
          (e_cnt  * gkp16)
        + (int_ang * gki16)
        + (de_cnt * gkd16);

    // -----------------------------------------------------------------
    //  输出级: 直接算出 u_q, 中间不再截成整数台阶
    // -----------------------------------------------------------------
    //  旧写法是两步:
    //        ang_out = ang_sum >>> 14          <- 先截成整数 (丢小数)
    //        u_q     = ang_out * 25            <- 每 1 档 = 0.25 % 占空比
    //  这等于把 PID 输出量化成 0, ±0.25 %, ±0.50 %, ... 一档一档跳.
    //  后果, 按 Kp=136 / Ki=10 / Kd=75 算:
    //        P 项: 1 度误差只有 0.165 %, 要偏到 1.5 度才够跳第一档
    //              -> 小角度时输出恒为 0, 电机不推 (表现为"PID 太小")
    //        D 项: 10 ms 内角度变化要 > 2.7 度 (270 度/秒) 才够跳一档
    //              -> 微分项基本被截没了, 等于 Kd 白给
    //  现在一次性算完: u_q = 25 * ang_sum / 16384, 1 LSB = 1e-4 占空比
    //  (0.01 %), 增益一点没变, 但小输出真的出得来了.
    //
    //  注意 >>> 是【向负无穷取整】, 直接截会让输出不对称:
    //     ang_sum = -5000 -> >>>14 = -1 -> 一侧恒定多推 0.25 %, 另一侧不推,
    //     平衡点被拖偏 + 极限环. 加半个 LSB 再移 = 四舍五入, 两边对称.
    wire signed [47:0] u_scaled = ang_sum * 48'sd25;                // 40+5 bit, 不会溢出
    wire signed [17:0] u_pid    = sat18((u_scaled + 48'sd8192) >>> 14, 18'sd10000);
    // 输出方向: 默认取负 (方向反了串口发 [s,Z_SGN,1])
    wire signed [17:0] u_bal    = gain_sgn[0] ? u_pid : -u_pid;

    // =================================================================
    //  判断 / 启摆状态  (每 40 ms 采样 3 次找最高点)
    //      display 0..180 度  ->  摆杆在左侧  (d_x10 < -BAL_RANGE)
    //      display 180..360   ->  摆杆在右侧  (d_x10 > +BAL_RANGE)
    //  左侧最高点: 中间那次最大 -> 先向右踢 (+占空比) 再向左回位
    //  右侧最高点: 中间那次最小 -> 先向左踢 (-占空比) 再向右回位
    // =================================================================
    reg signed [17:0] S0, S1;
    reg [5:0]  sw_cnt;
    reg [1:0]  push_phase;      // 0 idle, 1 kick, 2 return kick
    reg [6:0]  push_t;
    reg signed [17:0] push_duty;
    reg [13:0] sw_retry;
    reg [9:0]  idle_ms;
    reg        cap_flag;
    reg [3:0]  cap_ms;          // 连续落在捕获窗口里的毫秒数
    reg        sw_give_up;

    wire push_on = (push_phase != 2'd0);

    // 采样瞬间的三次角度: n0 = 本次, S0 = 上次, S1 = 上上次
    //   (和示例 Angle0/Angle1/Angle2 一致: 移完位后马上判断)
    wire        n_l_all= (d_x10 < -`BAL_RANGE) && (S0 < -`BAL_RANGE) && (S1 < -`BAL_RANGE);
    wire        n_r_all= (d_x10 >  `BAL_RANGE) && (S0 >  `BAL_RANGE) && (S1 >  `BAL_RANGE);
    wire        n_l_pk = n_l_all && (S0 > d_x10) && (S0 > S1);   // 左侧最高点
    wire        n_r_pk = n_r_all && (S0 < d_x10) && (S0 < S1);   // 右侧最高点

    // ---- 捕获窗口: 1 ms 判一次, 和踢腿相位无关 ----
    //   以前是在"空闲相位每 40 ms 采样"里判 n_win, 而踢腿的 SW_PUSH_MS +
    //   SW_BACK_MS = 200 ms 内根本不采样, 摆杆从窗口穿过去也看不见, 因此
    //   看着像"起摆完不会自己进 PID". 现在改成连续计时:
    //     连续 `CAP_CONFIRM_MS 个 1 ms tick 都落在窗口内 -> cap_flag
    //   窗口用 CAP_RANGE (显示角 120~240 度), 和找最高点用的 BAL_RANGE 分开,
    //   两者互不影响.
    wire in_cap = (d_x10 < `CAP_RANGE) && (d_x10 > -`CAP_RANGE);

    // ---- 放弃标志: 只由起摆超时决定 ----
    //      (保护只关 PWM, 不再停机 -> 差值回到 45 度以内会自动恢复)
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)          sw_give_up <= 1'b0;
        else if (ev_reset)   sw_give_up <= 1'b0;
        else if (ev_start)   sw_give_up <= 1'b0;
        else if (tick && !start_run) sw_give_up <= 1'b0;
        else if (tick && (mode == 2'd1) && (sw_retry >= `SW_RETRY_MS))
                             sw_give_up <= 1'b1;
    end

    // ---- 状态机 ----
    reg [1:0] mode_nxt;
    always @(*) begin
        if (!start_run || !cal_done || !th_valid) mode_nxt = 2'd0;
        else if (mode == 2'd0)
            mode_nxt = sw_give_up ? 2'd0 : (mode_sw ? 2'd2 : 2'd1);
        else if (mode == 2'd1)
            mode_nxt = cap_flag ? 2'd2 : (sw_give_up ? 2'd0 : 2'd1);
        else
            mode_nxt = 2'd2;            // 平衡不会自己退出
    end
    wire enter_bal = (mode_nxt == 2'd2) && (mode != 2'd2);
    wire enter_sw  = (mode_nxt == 2'd1) && (mode != 2'd1);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)        mode <= 2'd0;
        else if (ev_reset) mode <= 2'd0;
        else if (tick)     mode <= mode_nxt;
    end

    // ---- 采样 / 踢腿 ----
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            S0 <= 18'sd0; S1 <= 18'sd0;
            sw_cnt <= 6'd0; push_phase <= 2'd0; push_t <= 7'd0;
            push_duty <= 18'sd0; sw_retry <= 14'd0; idle_ms <= 10'd0;
            cap_flag <= 1'b0; cap_ms <= 4'd0;
        end else if (ev_reset) begin
            S0 <= 18'sd0; S1 <= 18'sd0;
            sw_cnt <= 6'd0; push_phase <= 2'd0; push_t <= 7'd0;
            push_duty <= 18'sd0; sw_retry <= 14'd0; idle_ms <= 10'd0;
            cap_flag <= 1'b0; cap_ms <= 4'd0;
        end else if (tick) begin
            if (enter_bal) begin
                cap_flag   <= 1'b0;
                cap_ms     <= 4'd0;
                push_phase <= 2'd0;
                push_t     <= 7'd0;
                push_duty  <= 18'sd0;
            end else if (enter_sw) begin
                // 刚进入启摆: 立刻踢第一脚 (示例里按 K1 后立即进状态 21)
                sw_retry   <= 14'd0;
                idle_ms    <= 10'd0;
                sw_cnt     <= 6'd0;
                S0 <= 18'sd0; S1 <= 18'sd0;
                cap_flag   <= 1'b0;
                cap_ms     <= 4'd0;
                push_phase <= 2'd1;
                push_t     <= 7'd0;
                push_duty  <= `SW_PUSH_DUTY;
            end else if (mode == 2'd1) begin
                // ---- 捕获: 连续 `CAP_CONFIRM_MS 个 tick 在窗口内就交棒 ----
                //      这一段故意放在踢腿状态机之外, 踢腿期间也照判,
                //      这样"摆上去了"能立刻进 PID, 不用等空闲相位.
                if (in_cap) begin
                    if (cap_ms < 4'd15) cap_ms <= cap_ms + 4'd1;
                    if (cap_ms >= `CAP_CONFIRM_MS - 1) cap_flag <= 1'b1;
                end else begin
                    cap_ms <= 4'd0;
                end

                // ---- 计时 ----
                if (sw_retry < `SW_RETRY_MS) sw_retry <= sw_retry + 14'd1;

                if (push_phase == 2'd1) begin
                    // ---- 第一脚 ----
                    if (push_t >= `SW_PUSH_MS) begin
                        push_phase <= 2'd2;
                        push_t     <= 7'd0;
                        push_duty  <= -push_duty;       // 反向回位
                    end else push_t <= push_t + 7'd1;
                end else if (push_phase == 2'd2) begin
                    // ---- 回位脚 ----
                    if (push_t >= `SW_BACK_MS) begin
                        push_phase <= 2'd0;
                        push_t     <= 7'd0;
                        push_duty  <= 18'sd0;
                        // 扔掉旧数据, 免得用踢腿之前的角度误判最高点
                        S0 <= 18'sd0; S1 <= 18'sd0;
                    end else push_t <= push_t + 7'd1;
                end else begin
                    // ---- 空闲: 每 40 ms 采样一次 ----
                    if (idle_ms < 10'h3FF) idle_ms <= idle_ms + 10'd1;

                    if (idle_ms >= `SW_IDLE_MS && bot_win) begin
                        // 摆杆一直不动 -> 先踢一脚, 让它离开死点
                        idle_ms    <= 10'd0;
                        sw_cnt     <= 6'd0;
                        S0 <= 18'sd0; S1 <= 18'sd0;
                        push_phase <= 2'd1;
                        push_t     <= 7'd0;
                        push_duty  <= `SW_PUSH_DUTY;
                    end else if (sw_cnt >= `SW_SAMPLE_MS) begin
                        sw_cnt <= 6'd0;
                        S1 <= S0;
                        S0 <= d_x10;

                        // (交棒判定已移到上面, 每 1 ms 连续判, 不再用采样点)

                        if (n_l_pk) begin
                            // 左侧最高点 -> 先向右踢 (duty > 0)
                            idle_ms    <= 10'd0;
                            push_phase <= 2'd1;
                            push_t     <= 7'd0;
                            push_duty  <= `SW_PUSH_DUTY;
                        end else if (n_r_pk) begin
                            // 右侧最高点 -> 先向左踢 (duty < 0)
                            idle_ms    <= 10'd0;
                            push_phase <= 2'd1;
                            push_t     <= 7'd0;
                            push_duty  <= -(`SW_PUSH_DUTY);
                        end
                    end else begin
                        sw_cnt <= sw_cnt + 6'd1;
                    end
                end
            end else begin
                sw_cnt     <= 6'd0;
                push_phase <= 2'd0;
                push_t     <= 7'd0;
                push_duty  <= 18'sd0;
                sw_retry   <= 14'd0;
                idle_ms    <= 10'd0;
                S0 <= 18'sd0; S1 <= 18'sd0;
            end
        end
    end

    // =================================================================
    //  电机输出:  mode 0 停止 / mode 1 踢腿 / mode 2 平衡 (每 5 ms)
    // =================================================================
    reg [3:0] div5;         // 0..ANG_MS-1  (ANG_MS = 10 需要 4 位!)
    reg [3:0] div_i;        // Ki 的 10 倍削弱: 每 KI_DIV 个角度环周期才积分一次

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            u_q <= 18'sd0; div5 <= 3'd0; div_i <= 4'd0;
            int_ang <= 20'sd0; e_cnt_prev <= 18'sd0; d_ok <= 1'b0;
            phi_cap <= 20'sd0;
        end else if (ev_reset) begin
            u_q <= 18'sd0; div5 <= 3'd0; div_i <= 4'd0;
            int_ang <= 20'sd0; e_cnt_prev <= 18'sd0; d_ok <= 1'b0;
            phi_cap <= 20'sd0;
        end else if (ev_start) begin
            // ---- 每次推上起摆开关 (SW4 / SW2 的上升沿) 立刻清 Ki 积分 ----
            //   严格说 mode 1 的分支里本来就每 1 ms 清一次, 但那是"下一个
            //   tick 才生效", 而且如果这次是"平衡中 -> 停 -> 再起摆", 中间
            //   会有一拍带着旧积分. 这里在边沿上直接清, 保证每次起摆都是
            //   从 0 积分开始, 不会带着上一轮的积压去踢第一脚.
            int_ang <= 20'sd0;
            div_i   <= 4'd0;
            u_q     <= 18'sd0;
        end else if (tick) begin
            if (enter_bal) begin
                // 进入平衡: 清积分, 手臂位置从这里算起
                phi_cap    <= $signed({2'b00, phi_q});
                int_ang    <= 20'sd0;
                e_cnt_prev <= e_cnt;
                d_ok       <= 1'b0;
                div5       <= 3'd0;
                div_i      <= 4'd0;
                u_q        <= 18'sd0;
            end else if (mode == 2'd0) begin
                // 停止 (显示模式 3 下可以用 K5/K6 开环点动)
                if      (jog_up) u_q <=  `JOG_DUTY;
                else if (jog_dn) u_q <= -`JOG_DUTY;
                else             u_q <=  18'sd0;
                int_ang <= 20'sd0; div5 <= 3'd0;
            end else if (mode == 2'd1) begin
                // 启摆: 开环踢腿
                u_q <= push_on ? push_duty : 18'sd0;
                int_ang <= 20'sd0; div5 <= 3'd0;
            end else begin
                // ---------------- 平衡 ----------------
                // 角度环: 每 ANG_MS = 10 ms
                if (div5 >= `ANG_MS - 1) begin
                    div5       <= 3'd0;
                    // 保护: 直立环输入(偏差)绝对值超过 45 度 -> PWM = 0
                    u_q        <= bal_win ? u_bal : 18'sd0;
                    e_cnt_prev <= e_cnt;
                    d_ok       <= 1'b1;
                    // 示例的做法: Ki = 0 时不积分, 直接把积分清零
                    //   (否则 Ki 一直是 0 的时候积分会越攒越大,
                    //    等哪天把 Ki 调上去, 积分项会猛地一脚)
                    // 另外两个条件也清零 (不是冻结):
                    //   !bal_win  : 超出保护窗口 (GUARD_DEG)
                    //   !int_ok   : 还没进到 +-INT_WIN 的线性区
                    //               -> 刚交棒时摆杆还在 40~60 度, 不许攒积分
                    // Ki 单独再削弱: 每 KI_DIV 个角度环周期才累加一次
                    if (gki16 == 18'sd0 || !bal_win || !int_ok) begin
                        int_ang <= 20'sd0;
                        div_i   <= 4'd0;
                    end else if (div_i >= `KI_DIV - 1) begin
                        div_i <= 4'd0;
                        // 积分限幅: 先加再钳, 保证 |int_ang| 永远 <= ANG_INT_MAX.
                        //   (以前是"先钳后加", 加完还能超出限幅一个 e_cnt)
                        //   ANG_INT_MAX 的单位是"计数 x 次"; 它对占空比的贡献 =
                        //   ANG_INT_MAX * (Ki*7) * 25 / 16384 / 100  %.
                        if (int_next >  `ANG_INT_MAX)      int_ang <=  `ANG_INT_MAX;
                        else if (int_next < -`ANG_INT_MAX) int_ang <= -`ANG_INT_MAX;
                        else                               int_ang <= int_next[19:0];
                    end else begin
                        div_i <= div_i + 4'd1;
                    end
                end else begin
                    div5 <= div5 + 3'd1;
                end

                // 位置环的 50 ms 分频在它自己的 always 块里 (pos_st 状态机)
            end
        end
    end

endmodule


// ---------------------------------------------------------------------
//  20 kHz sign/magnitude PWM.  u_q is 1e-4 duty, +/-10000.
//  Period = 2500 clocks, so threshold = |u| / 4.
// ---------------------------------------------------------------------
module pwm_gen (
    input  wire               clk,
    input  wire               rst_n,
    input  wire signed [17:0] u_q,
    output reg                pwm,
    output reg                dir
);
    reg [11:0] cnt;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) cnt <= 12'd0;
        else if (cnt == `PWM_CLK - 1) cnt <= 12'd0;
        else cnt <= cnt + 12'd1;
    end

    wire [17:0] mag = u_q[17] ? (-u_q) : u_q;
    wire [11:0] thr = (mag > 18'd10000) ? 12'd2500 : mag[13:2];

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pwm <= 1'b0;
            dir <= 1'b0;
        end else begin
            pwm <= (cnt < thr);
            dir <= u_q[17];
        end
    end
endmodule
