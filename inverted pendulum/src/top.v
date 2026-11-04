// =====================================================================
//  top.v  --  rotary inverted pendulum, Pocket Lab-F1 / GW1N-UV9EQ144C6
//  ---------------------------------------------------------------------
//  Run control
//      DIP SW1 (75) : RUN      up = start / swing up / balance, down = stop
//                     (K1 on this board does not register, so SW1 replaces it)
//
//  Buttons
//      K2 (59) : cycle display mode
//      K3 (60) : preset +5 deg      K4 (61) : preset -5 deg
//      K5 (62) : preset +30 deg     K6 (63) : preset -30 deg
//      K7 (64) : save preset and execute
//      K8 (65) : reset everything
//
//  Display modes (K2), LED1..LED4 show which one is active
//      0 : arm angle phi    (deg, from the encoder)  <- power-on default
//      1 : preset arm angle (deg)
//      2 : pendulum angle theta (deg, 0 = hanging, 180 = upright)
//      3 : motor duty       (%)   <- K5/K6 jog the motor here
//
//  External wiring (J4 header) : see top.cst
// =====================================================================
`include "params.vh"

module top (
    input  wire       clk_50m,
    input  wire [7:0] key_n,
    input  wire       sw_run,       // DIP 4 (pin 79): UP = 起摆 + 平衡
    input  wire       sw_mode,      // DIP 2 (pin 76): UP = 只平衡 (不起摆)
    output wire [7:0] seg_n,
    output wire [3:0] dig_n,
    output wire [3:0] led,          // LED1..LED4 indicate the display mode
    output wire       beep,         // buzzer - disabled, always low
    output wire       motor_pwm,
    output wire       motor_dir,
    output wire       motor_dir_n,
    input  wire       enc_a,
    input  wire       enc_b,
    input  wire       sa100_pwm,
    input  wire       uart_rx,      // Type-C / CH340 (pin 49)
    output wire       uart_tx       //                 (pin 48)
);

    // =================================================================
    //  power-on reset, ~1.3 ms
    // =================================================================
    reg [15:0] por;
    always @(posedge clk_50m)
        if (por != 16'hFFFF) por <= por + 16'd1;
    wire rst_n = (por == 16'hFFFF);

    // =================================================================
    //  buttons
    // =================================================================
    wire [7:0] key_lvl;
    wire [7:0] key_prs;

    key_debounce u_key (
        .clk   (clk_50m),
        .rst_n (rst_n),
        .key_n (key_n),
        .level (key_lvl),
        .press (key_prs)
    );

    // =================================================================
    //  1 ms tick (shared by the debounce filters)
    // =================================================================
    reg [15:0] t_ms_div;
    reg        t_ms_tick;
    always @(posedge clk_50m or negedge rst_n) begin
        if (!rst_n) begin
            t_ms_div  <= 16'd0;
            t_ms_tick <= 1'b0;
        end else if (t_ms_div == `TICK_CLK - 1) begin
            t_ms_div  <= 16'd0;
            t_ms_tick <= 1'b1;
        end else begin
            t_ms_div  <= t_ms_div + 16'd1;
            t_ms_tick <= 1'b0;
        end
    end

    // =================================================================
    //  DIP switches (K1 on this board does not register)
    //      SW1 (pin 75) UP -> MODE 1 : swing-up, then balance
    //      SW2 (pin 76) UP -> MODE 2 : BALANCE ONLY, no swing-up
    //                        (hold the pendulum upright by hand, then push)
    //      both DOWN       -> stop, motor off
    //  3-FF synchroniser + 20 ms filter per switch + edge detect.
    //  `SW_LOW_ACTIVE` flips the polarity if the board reads the other way.
    // =================================================================
    wire [1:0] sw_raw = `SW_LOW_ACTIVE ? ~{sw_run, sw_mode} : {sw_run, sw_mode};

    reg [2:0] sw_sync0, sw_sync1;
    always @(posedge clk_50m or negedge rst_n) begin
        if (!rst_n) begin
            sw_sync0 <= 3'b000;
            sw_sync1 <= 3'b000;
        end else begin
            sw_sync0 <= {sw_sync0[1:0], sw_raw[0]};
            sw_sync1 <= {sw_sync1[1:0], sw_raw[1]};
        end
    end

    reg [4:0] sw_same0, sw_same1;
    reg       sw_last0, sw_last1;
    reg       sw_lvl0,  sw_lvl1;
    always @(posedge clk_50m or negedge rst_n) begin
        if (!rst_n) begin
            sw_same0 <= 5'd0; sw_last0 <= 1'b0; sw_lvl0 <= 1'b0;
            sw_same1 <= 5'd0; sw_last1 <= 1'b0; sw_lvl1 <= 1'b0;
        end else if (t_ms_tick) begin
            if (sw_sync0[2] == sw_last0) begin
                if (sw_same0 == 5'd20) sw_lvl0 <= sw_last0;   // stable 20 ms
                else                   sw_same0 <= sw_same0 + 5'd1;
            end else begin
                sw_same0 <= 5'd0;
                sw_last0 <= sw_sync0[2];
            end
            if (sw_sync1[2] == sw_last1) begin
                if (sw_same1 == 5'd20) sw_lvl1 <= sw_last1;
                else                   sw_same1 <= sw_same1 + 5'd1;
            end else begin
                sw_same1 <= 5'd0;
                sw_last1 <= sw_sync1[2];
            end
        end
    end

    reg [1:0] sw_lvl_d;
    always @(posedge clk_50m or negedge rst_n) begin
        if (!rst_n) sw_lvl_d <= 2'b00;
        else        sw_lvl_d <= {sw_lvl1, sw_lvl0};
    end

    // =================================================================
    //  启动开关
    //    拨码 SW1 (pin 75) 坏了 -> 启动改用拨码 SW4 (pin 79)
    //      SW4 推上 = 启动 (起摆 + 平衡)
    //      SW2 推上 = 启动 (只平衡, 不起摆)
    //  注意: sw_raw = {sw_run, sw_mode}
    //        sw_raw[0] = sw_mode = DIP2 (pin 76)  -> sw_lvl0
    //        sw_raw[1] = sw_run  = DIP4 (pin 79)  -> sw_lvl1
    // =================================================================
    wire sw_mode_lvl = sw_lvl0;                     // DIP2 (pin 76): 只平衡
    wire sw_run_lvl  = sw_lvl1;                     // DIP4 (pin 79): 起摆+平衡
    wire run_req     = sw_run_lvl | sw_mode_lvl;    // 任意一个推上 = 运行
    wire sw_rise0 =  sw_lvl0 & ~sw_lvl_d[0];        // DIP2 推上
    wire sw_rise1 =  sw_lvl1 & ~sw_lvl_d[1];        // DIP4 推上
    wire ev_start_dip = sw_rise0 | sw_rise1;        // 任意一个推上就启动

    // =================================================================
    //  display mode selector (K2).  In mode 3 (duty) K5/K6 become an
    //  open-loop jog so motor and encoder polarity can be checked.
    // =================================================================
    reg [2:0] disp_mode;                // 0..4  (4 = 调试状态)
    always @(posedge clk_50m or negedge rst_n) begin
        if (!rst_n)          disp_mode <= 3'd0;
        else if (key_prs[1]) disp_mode <= (disp_mode == 3'd4) ? 3'd0 : disp_mode + 3'd1;
    end
    wire jog_mode = (disp_mode == 3'd3);

    // =================================================================
    //  encoder -> phi (1e-4 rad)
    //  (the speed measurement in sense.v is not used by this controller)
    // =================================================================
    wire signed [31:0] enc_pos;
    wire               enc_step;
    wire               enc_dir;

    enc_decode u_enc (
        .clk      (clk_50m),
        .rst_n    (rst_n),
        .a        (`ENC_SWAP ? enc_b : enc_a),
        .b        (`ENC_SWAP ? enc_a : enc_b),
        .pos      (enc_pos),
        .step     (enc_step),
        .step_dir (enc_dir)
    );

    wire signed [19:0] pos_s   = enc_pos[19:0];              // +-500 revolutions
    wire signed [35:0] phi_raw = $signed(pos_s) * `ENC_K;
    wire signed [25:0] phi_sh  = phi_raw >>> `ENC_SH;
    wire signed [17:0] phi_q   = (phi_sh >  26'sd131071) ?  18'sd131071 :
                                 (phi_sh < -26'sd131071) ? -18'sd131071 :
                                 phi_sh[17:0];

    // phi wrapped into [0, 2*pi) : used for display AND for the position
    // loop, so the reference error always takes the shortest way round
    wire signed [19:0] phi_e  = $signed({{2{phi_q[17]}}, phi_q});
    wire signed [19:0] phi_m1 = (phi_e < -20'sd62832) ? phi_e + 20'sd125664 :
                                (phi_e >  20'sd62832) ? phi_e - 20'sd125664 :
                                                        phi_e;
    wire signed [19:0] phi_m2 = (phi_m1 < 0) ? phi_m1 + 20'sd62832 : phi_m1;
    wire        [17:0] phi_mod = phi_m2[17:0];

    // =================================================================
    //  SA100 -> theta raw (1e-4 rad, 0..2pi)
    // =================================================================
    wire [17:0] th_raw;
    wire        th_ok;

    sa100_capture u_sa (
        .clk     (clk_50m),
        .rst_n   (rst_n),
        .pwm_in  (sa100_pwm),
        .angle_q (th_raw),
        .valid   (th_ok)
    );

    // =================================================================
    //  serial port : on-line gain tuning
    //      [s,Z_KP,164.0]  [s,Z_KD,115.0]  [s,Z_KI,0.0]
    //      every accepted packet is echoed back + CRLF
    // =================================================================
    wire [17:0] gain_kp;
    wire [17:0] gain_kd;
    wire [17:0] gain_ki;
    wire [17:0] gain_sgn;
    wire [17:0] gain_wkp;       // 位置环 Kp2 (串口 W_KP, Q12)
    wire [17:0] gain_wkd;       // 位置环 Kd2 (串口 W_KD, Q12)
    wire [17:0] tgt_q;          // 直立环目标角度 (串口可调)

    tune_rx u_tune (
        .clk     (clk_50m),
        .rst_n   (rst_n),
        .rx_pin  (uart_rx),
        .tx_pin  (uart_tx),
        .gain_kp (gain_kp),
        .gain_kd (gain_kd),
        .gain_ki (gain_ki),
        .gain_sgn(gain_sgn),
        .gain_wkp(gain_wkp),
        .gain_wkd(gain_wkd),
        .tgt_q   (tgt_q)
    );

    // =================================================================
    //  control core
    // =================================================================
    wire signed [17:0] u_q;
    wire        [1:0]  mode;
    wire signed [17:0] preset_q;
    wire        [17:0] th_disp;         // 0 deg at hanging, 180 at upright
    wire               dbg_guard;       // 保护窗口允许输出
    wire               dbg_cal;         // 定零完成
    wire        [17:0] dbg_ang;         // 控制器看到的摆角 (0.1 度)

    ctrl_core u_ctrl (
        .clk        (clk_50m),
        .rst_n      (rst_n),
        .phi_q      (phi_mod),
        .th_raw     (th_raw),
        .th_ok      (th_ok),
        .ev_start   (ev_start_dip),
        .run_sw     (run_req),
        .mode_sw    (sw_mode_lvl),
        .ev_up5     (key_prs[2]),
        .ev_dn5     (key_prs[3]),
        .ev_up30    (jog_mode ? 1'b0 : key_prs[4]),
        .ev_dn30    (jog_mode ? 1'b0 : key_prs[5]),
        .ev_save    (key_prs[6]),
        .ev_reset   (key_prs[7]),
        .jog_up     (jog_mode & key_lvl[4]),
        .jog_dn     (jog_mode & key_lvl[5]),
        .gain_kp    (gain_kp),
        .gain_kd    (gain_kd),
        .gain_ki    (gain_ki),
        .gain_sgn   (gain_sgn),
        .gain_wkp   (gain_wkp),
        .gain_wkd   (gain_wkd),
        .bal_target (tgt_q),
        .u_q        (u_q),
        .mode       (mode),
        .preset_q   (preset_q),
        .dbg_guard  (dbg_guard),
        .dbg_cal    (dbg_cal),
        .dbg_ang    (dbg_ang),
        .th_disp_q  (th_disp)
    );

    // =================================================================
    //  PWM + direction for the TB6612
    // =================================================================
    wire pwm_out;
    wire dir_out;

    pwm_gen u_pwm (
        .clk   (clk_50m),
        .rst_n (rst_n),
        .u_q   (u_q),
        .pwm   (pwm_out),
        .dir   (dir_out)
    );

    assign motor_pwm   = pwm_out;
    // If the arm turns the wrong way for a positive duty, set MOTOR_SWAP = 1
    // in params.vh (no re-wiring needed).
    assign motor_dir   = `MOTOR_SWAP ? ~dir_out :  dir_out;
    assign motor_dir_n = `MOTOR_SWAP ?  dir_out : ~dir_out;

    // =================================================================
    //  mode indicator: one LED per display mode (LED1..LED4, active high)
    // =================================================================
    assign led = (disp_mode == 3'd4) ? 4'b1111 : (4'b0001 << disp_mode[1:0]);

    // =================================================================
    //  buzzer : disabled (not needed any more, pin kept low)
    // =================================================================
    assign beep = 1'b0;


    // =================================================================
    //  display value selection
    // =================================================================
    // ---- display value: every angle is shown as 0..360, no sign digit ----
    // mode 0 : arm angle phi  (encoder)   <- power-on default
    // mode 1 : preset arm angle           <- K3..K6 adjust here
    // mode 2 : pendulum angle theta       (0 deg when hanging, 180 upright)
    // mode 3 : motor duty in percent      <- K5/K6 jog the motor
    reg [17:0] disp_uv;                 // angle in 1e-4 rad, 0..62832
    always @(*) begin
        case (disp_mode)
            3'd0:    disp_uv = phi_mod;     // arm angle from the encoder
            3'd1:    disp_uv = preset_q;    // preset arm angle
            3'd2:    disp_uv = th_disp;     // pendulum angle, 0 = hanging
            default: disp_uv = 18'd0;
        endcase
    end

    wire [17:0] u_mag = u_q[17] ? (-u_q) : u_q;

    // degrees: (1e-4 rad)  * 180/pi -> *376  >> 16   (error 0.13 %)
    // percent: (1e-4 duty) / 100    -> *10486 >> 20  (error 0.002 %)
    wire [31:0] duty_pct = (u_mag * 32'd10486) >> 20;          // 0..100
    wire [15:0] duty_sh  = (duty_pct > 32'd100) ? 16'd100 : duty_pct[15:0];

    wire [31:0] scaled = (disp_mode == 3'd3) ? duty_pct
                                             : ((disp_uv * 32'd376) >> 16);

    wire [15:0] shown = (disp_mode == 3'd3)
                      ? duty_sh
                      : ((scaled > 32'd359) ? 16'd359 : scaled[15:0]);

    wire [15:0] bcd_lo;
    bin2bcd u_bcd (
        .bin_in (shown),
        .bcd    (bcd_lo)
    );

    // register it so the optimiser cannot sweep the converter away
    reg [15:0] bcd_r;
    always @(posedge clk_50m or negedge rst_n) begin
        if (!rst_n) bcd_r <= 16'hFFFF;
        else        bcd_r <= bcd_lo;
    end
    // top digit: '-' when the motor duty is negative.
    wire dneg = (disp_mode == 3'd3) && u_q[17];

    // 第 5 档调试: [定零完成][控制器看到的摆角: 3 位整数度]
    //   乘法要在 32 位里做: 3600 x 6554 = 2.4e7, 18 位装不下会溢出
    wire [31:0] dbg_deg32 = (dbg_ang * 32'd6554) >> 16;        // 0.1 度 -> 整数度
    wire [15:0] dbg_deg16 = (dbg_deg32 > 32'd359) ? 16'd359 : dbg_deg32[15:0];
    wire [15:0] bcd_ang;
    bin2bcd u_bcd3 (
        .bin_in (dbg_deg16),
        .bcd    (bcd_ang)
    );
    reg [15:0] bcd_ang_r;
    always @(posedge clk_50m or negedge rst_n) begin
        if (!rst_n) bcd_ang_r <= 16'h0000;
        else        bcd_ang_r <= bcd_ang;
    end

    // 第 1 位 = 定零完成 (1 = 学好了, 0 = 还在等摆杆静止)
    // 后 3 位 = 控制器看到的摆杆整数度
    wire [15:0] disp_bcd = (disp_mode == 3'd4)
                         ? { 3'b000, dbg_cal, bcd_ang_r[11:0] }
                         : { dneg ? 4'd10 : 4'd15, bcd_r[11:0] };

    // leading-zero blanking: 0 shows as "0", 15 as "15", 180 as "180"
    wire [3:0] blank_mask = (disp_mode == 3'd4) ? 4'b0000
                                               : { 1'b0,
                                                   (shown < 16'd100),
                                                   (shown < 16'd10),
                                                   1'b0 };

    wire [7:0] seg_raw;

    seg7 u_seg (
        .clk   (clk_50m),
        .rst_n (rst_n),
        .bcd   (disp_bcd),
        .blank (blank_mask),
        .seg_n (seg_raw),
        .dig_n (dig_n)
    );

    // decimal points lit = the motor is being driven by the control loop.
    // Quick way to see whether K1 actually did something.
    assign seg_n = (mode != 2'd0) ? (seg_raw & 8'h7F) : seg_raw;

endmodule
