// =====================================================================
//  uart.v  --  serial port for on-line gain tuning
//  ---------------------------------------------------------------------
//      uart_rx  : 8N1 receiver            (FPGA_RXD, pin 49)
//      uart_tx  : 8N1 transmitter         (FPGA_TXD, pin 48)
//      tune_rx  : packet parser + the three run-time gains
//
//  Packet format (ASCII, one decimal digit):
//      [s,Z_KP,164.0]      -> 直立环 P
//      [s,Z_KD,115.0]      -> 直立环 D
//      [s,Z_KI,0.0]        -> 直立环 I
//      [s,Z_SGN,1.0]       -> 1 = invert the balance direction (live!)
//
//  's' is a fixed character, the field name is fixed, only the number
//  changes.  Anything that does not match the pattern is dropped, byte by
//  byte, without disturbing the parser (a new '[' always restarts it).
//  Every accepted packet is echoed back followed by CR LF, so the PC side
//  can see exactly what the board received.
//
//  The number is the INTERNAL gain, exactly the value that is written in
//  params.vh (Q12), so it can be tuned live and copied back into the file
//  once it is good:
//      直立环 P (rad/s^2/rad)  = KP  x 0.4096
//      直立环 D (rad/s^2/(rad/s)) = KD  x 4.096
// =====================================================================
`include "params.vh"


// ---------------------------------------------------------------------
//  8N1 receiver, mid-bit sampling
// ---------------------------------------------------------------------
module uart_rx #(
    parameter integer DIV = `BAUD_DIV
)(
    input  wire       clk,
    input  wire       rst_n,
    input  wire       rx_pin,
    output reg  [7:0] data,
    output reg        done
);
    localparam [1:0] IDLE = 2'd0, START = 2'd1, DATA = 2'd2, STOP = 2'd3;

    reg [2:0] s;
    always @(posedge clk) s <= {s[1:0], rx_pin};
    wire rx = s[1];

    reg [1:0]  st;
    reg [16:0] cnt;
    reg [2:0]  bi;
    reg [7:0]  sh;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            st   <= IDLE;
            cnt  <= 17'd0;
            bi   <= 3'd0;
            sh   <= 8'd0;
            data <= 8'd0;
            done <= 1'b0;
        end else begin
            done <= 1'b0;
            case (st)
            IDLE: begin
                cnt <= 17'd0;
                if (!rx) st <= START;               // start bit
            end
            START: begin
                if (cnt == DIV/2 - 1) begin         // middle of the start bit
                    cnt <= 17'd0;
                    if (rx) st <= IDLE;             // glitch, not a start bit
                    else begin
                        st <= DATA;
                        bi <= 3'd0;
                    end
                end else begin
                    cnt <= cnt + 17'd1;
                end
            end
            DATA: begin
                if (cnt == DIV - 1) begin           // one bit time later
                    cnt <= 17'd0;
                    sh  <= {rx, sh[7:1]};           // LSB first
                    if (bi == 3'd7) st <= STOP;
                    else            bi <= bi + 3'd1;
                end else begin
                    cnt <= cnt + 17'd1;
                end
            end
            STOP: begin
                if (cnt == DIV/2 - 1) begin
                    cnt  <= 17'd0;
                    data <= sh;
                    done <= 1'b1;
                    st   <= IDLE;
                end else begin
                    cnt <= cnt + 17'd1;
                end
            end
            endcase
        end
    end
endmodule


// ---------------------------------------------------------------------
//  8N1 transmitter
// ---------------------------------------------------------------------
module uart_tx #(
    parameter integer DIV = `BAUD_DIV
)(
    input  wire       clk,
    input  wire       rst_n,
    input  wire       send,          // one-clock pulse
    input  wire [7:0] data,
    output reg        tx,
    output reg        busy
);
    localparam [1:0] IDLE = 2'd0, START = 2'd1, DATA = 2'd2, STOP = 2'd3;

    reg [1:0]  st;
    reg [16:0] cnt;
    reg [2:0]  bi;
    reg [7:0]  sh;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            st   <= IDLE;
            cnt  <= 17'd0;
            bi   <= 3'd0;
            sh   <= 8'd0;
            tx   <= 1'b1;
            busy <= 1'b0;
        end else begin
            case (st)
            IDLE: begin
                tx   <= 1'b1;
                busy <= 1'b0;
                cnt  <= 17'd0;
                if (send) begin
                    sh   <= data;
                    bi   <= 3'd0;
                    tx   <= 1'b0;                   // start bit
                    busy <= 1'b1;
                    st   <= START;
                end
            end
            START: begin
                if (cnt == DIV - 1) begin cnt <= 17'd0; st <= DATA; end
                else                cnt <= cnt + 17'd1;
            end
            DATA: begin
                if (cnt == DIV - 1) begin
                    cnt <= 17'd0;
                    tx  <= sh[0];
                    sh  <= {1'b0, sh[7:1]};
                    if (bi == 3'd7) st <= STOP;
                    else            bi <= bi + 3'd1;
                end else begin
                    cnt <= cnt + 17'd1;
                end
            end
            STOP: begin
                tx <= 1'b1;
                if (cnt == DIV - 1) begin cnt <= 17'd0; st <= IDLE; end
                else                cnt <= cnt + 17'd1;
            end
            endcase
        end
    end
endmodule


// ---------------------------------------------------------------------
//  packet parser + the three run-time gains
//      input  : "[s,Z_K<X>,<number>]"      X = P / I / D
//      output : gain_kp / gain_kd / gain_ki  (Q12, 0..99999)
// ---------------------------------------------------------------------
module tune_rx (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        rx_pin,
    output wire        tx_pin,
    output reg  [17:0] gain_kp,
    output reg  [17:0] gain_kd,
    output reg  [17:0] gain_ki,
    output reg  [17:0] gain_sgn,
    output reg  [17:0] gain_wkp,     // 位置环 Kp2 [s,W_KP,..] (Q12)
    output reg  [17:0] gain_wkd,     // 位置环 Kd2 [s,W_KD,..] (Q12)
    output reg  [17:0] tgt_q       // 直立环目标角度 (1e-4 rad, 0..2pi)
);
    // ---------------- byte level ----------------
    wire [7:0] rx_data;
    wire       rx_done;

    uart_rx u_rx (
        .clk    (clk),
        .rst_n  (rst_n),
        .rx_pin (rx_pin),
        .data   (rx_data),
        .done   (rx_done)
    );

    reg        tx_send;
    reg  [7:0] tx_byte;
    wire       tx_busy;

    uart_tx u_tx (
        .clk   (clk),
        .rst_n (rst_n),
        .send  (tx_send),
        .data  (tx_byte),
        .tx    (tx_pin),
        .busy  (tx_busy)
    );

    // ---------------- parser state machine ----------------
    // ---------------- parser state machine ----------------
    //   [s,Z_KP,19.0]     [s,Z_KI,0.7]   [s,Z_KD,1.1]   [s,Z_SGN,1.0]
    //   [s,target,181.0]
    localparam [4:0] P_IDLE  = 5'd0,  P_S   = 5'd1,  P_COM1 = 5'd2,
                     P_BRCH  = 5'd3,  P_US  = 5'd4,  P_K    = 5'd5,
                     P_KIND  = 5'd6,  P_COM2= 5'd7,  P_NUM  = 5'd8,
                     P_T2    = 5'd9,  P_T3  = 5'd10, P_T4   = 5'd11,
                     P_T5    = 5'd12, P_T6  = 5'd13, P_COM3 = 5'd14,
                     P_GET   = 5'd15, P_SG  = 5'd16, P_SGN  = 5'd17;

    reg [4:0]  ps;
    reg [2:0]  kind;        // 0=SGN 1=P 2=I 3=D 4=target 5=W_KP 6=W_KD
    reg        isw;         // 1 = 这个包是 W_xx (位置环), 0 = Z_xx (直立环)
    reg [23:0] v10;         // digits collected, x10 when a . was seen
    reg        dot;
    reg [1:0]  frac_n;      // 已收到的小数位数 (最多 2 位)
    reg [4:0]  blen;        // characters stored for the echo
    reg [7:0]  pkt [0:15];

    // ---- 增益: 串口里填的就是"整数"寄存器值 ----
    //   ctrl.v 里 ×7 (Q16) 变成实际增益:  实际增益 = 数字 / 9362
    //       100  -> 0.011     300  -> 0.032     1000 -> 0.107     3000 -> 0.320
    //   直接填整数, 不用打小数点:
    //       [s,Z_KP,1000]   [s,Z_KI,100]   [s,Z_KD,3000]
    //   (范围 0 ~ 10000)
    wire [23:0] gval = v10;
    // 直立环增益内部要 x7 (=8-1, 移位减法, 不占乘法器)
    wire [23:0] gval7 = (gval << 3) - gval;
    // target 用: 度 -> 0.1 度   ("181" -> 1810, "181.0" -> 1810)
    wire [31:0] v10x = dot ? {8'd0, v10} : ({8'd0, v10} * 32'd10);

    // ---------------- echo sequencer ----------------
    reg        echo_start;      // pulse from the parser: send the packet back
    reg        echo_run;
    reg [4:0]  echo_i;
    reg [4:0]  echo_len;

    // ---------------- 参数读回 [s,GET,1] ----------------
    //   回一行 5 组, 每组 = 组号字母 + 4 位十六进制数:
    //     A=Z_KP  B=Z_KI  C=Z_KD  D=Z_SGN  E=target
    //   例:  A0088 B000A C004B D0000 E0712
    //        (KP=0x88=136, KI=0x0A=10, KD=0x4B=75, SGN=0, 目标=0x712=1810)
    reg        rep_req;
    reg        rep_run;
    reg [2:0]  rep_gi;          // 0..4 组号, 5 = 结尾
    reg [2:0]  rep_ci;          // 组内第几个字节 0..4
    reg [15:0] kpt_q, kit_q, kdt_q;     // 用户填的原始数字 (未 x7)

    wire [15:0] rep_val = (rep_gi == 3'd0) ? kpt_q :
                          (rep_gi == 3'd1) ? kit_q :
                          (rep_gi == 3'd2) ? kdt_q :
                          (rep_gi == 3'd3) ? {14'd0, gain_sgn} :
                                            tgt_q[15:0];
    // NOTE: 以前这里写的是 {6'd0, tgt_q[9:0]}, 只回 10 bit.
    //       目标 181.0 度 (0x712=1810) 会被截成 0x312=786, 抄回代码就变成
    //       78.6 度 —— 平衡点直接错掉, 是"串口调好了、填进代码就不对"的元凶之一。
    wire [3:0]  rep_nib = (rep_ci == 3'd1) ? rep_val[15:12] :
                          (rep_ci == 3'd2) ? rep_val[11:8]  :
                          (rep_ci == 3'd3) ? rep_val[7:4]   :
                                            rep_val[3:0];
    reg  [7:0]  rep_byte;
    always @(*) begin
        case (rep_ci)
        3'd0: rep_byte = (rep_gi == 3'd0) ? "A" :
                         (rep_gi == 3'd1) ? "B" :
                         (rep_gi == 3'd2) ? "C" :
                         (rep_gi == 3'd3) ? "D" : "E";
        3'd1, 3'd2, 3'd3, 3'd4:
              rep_byte = (rep_nib < 4'd10) ? (8'h30 + {4'd0, rep_nib})
                                           : (8'h37 + {4'd0, rep_nib});
        default: rep_byte = 8'h20;
        endcase
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            tx_send   <= 1'b0;
            tx_byte   <= 8'h00;
            echo_run  <= 1'b0;
            echo_i    <= 5'd0;
            rep_run   <= 1'b0;
            rep_gi    <= 3'd0;
            rep_ci    <= 3'd0;
        end else begin
            tx_send <= 1'b0;
            // 读回优先: 它跑的时候不做回显
            if (rep_req && !rep_run) begin
                rep_run <= 1'b1;
                rep_gi  <= 3'd0;
                rep_ci  <= 3'd0;
            end else if (rep_run) begin
                if (!tx_busy) begin
                    tx_send <= 1'b1;
                    if (rep_gi == 3'd5)
                        tx_byte <= (rep_ci == 3'd0) ? 8'h0D : 8'h0A;   // CR LF
                    else
                        tx_byte <= rep_byte;
                    if (rep_gi == 3'd5) begin
                        if (rep_ci == 3'd0) rep_ci <= 3'd1;
                        else begin rep_run <= 1'b0; rep_ci <= 3'd0; rep_gi <= 3'd0; end
                    end else if (rep_ci == 3'd4) begin
                        rep_ci <= 3'd0;
                        rep_gi <= rep_gi + 3'd1;
                    end else begin
                        rep_ci <= rep_ci + 3'd1;
                    end
                end
            end else if (echo_start && !echo_run) begin
                echo_run <= 1'b1;
                echo_i   <= 5'd0;
            end else if (echo_run && !tx_busy) begin
                tx_send <= 1'b1;
                if (echo_i < echo_len)       tx_byte <= pkt[echo_i[3:0]];
                else if (echo_i == echo_len) tx_byte <= 8'h0D;   // CR
                else                         tx_byte <= 8'h0A;   // LF
                if (echo_i >= echo_len + 5'd1) echo_run <= 1'b0;
                echo_i <= echo_i + 5'd1;
            end
        end
    end

    // ---------------- parser ----------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            ps        <= P_IDLE;
            kind      <= 2'd0;
            v10       <= 24'd0;
            dot       <= 1'b0;
            frac_n    <= 2'd0;
            blen      <= 5'd0;
            rep_req   <= 1'b0;
            kpt_q     <= `KP_ANG;
            kit_q     <= `KI_ANG;
            kdt_q     <= `KD_ANG;
            // defaults from params.vh (32 -> 16 bit truncation is fine)
            gain_kp   <= `KP_ANG * 7;      // 内部一律 x7 (见 ctrl.v)
            gain_kd   <= `KD_ANG * 7;
            gain_ki   <= `KI_ANG * 7;
            gain_sgn  <= `BAL_SGN;          // 串口 [s,Z_SGN,x] 的编译期默认值
            gain_wkp  <= `KP_POS_P;         // 位置环 Kp2 默认 (Q12)
            gain_wkd  <= `KD_POS_P;         // 位置环 Kd2 默认 (Q12)
            tgt_q     <= `BAL_TARGET_Q;    // 0.1 度单位, 1810 = 181.0 度
        end else begin
          // ---- 每拍默认清零: 让 echo_start / rep_req 变成"1 拍脉冲" ----
          //      (以前只在收到下一个字节时才清零, 导致回显无限重复发送)
          echo_start <= 1'b0;
          rep_req    <= 1'b0;
          if (rx_done) begin
            // ---- keep a copy for the echo ----
            if (rx_data == 8'h5B) begin            // '[' starts a new packet
                blen     <= 5'd1;
                pkt[4'd0] <= rx_data;
            end else if (blen < 5'd16) begin
                pkt[blen[3:0]] <= rx_data;
                blen <= blen + 5'd1;
            end

            // ---- parse one character ----
            case (ps)
            P_IDLE: if (rx_data == 8'h5B) begin    // '['
                        ps <= P_S;
                        v10 <= 24'd0;
                        dot <= 1'b0;
                        frac_n <= 2'd0;
                    end
            P_S:    ps <= (rx_data == "s") ? P_COM1 : P_IDLE;
            P_COM1: ps <= (rx_data == ",") ? P_BRCH : P_IDLE;
            P_BRCH: if      (rx_data == "Z") begin isw <= 1'b0; ps <= P_US; end  // Z_Kx
                    else if (rx_data == "W") begin isw <= 1'b1; ps <= P_US; end  // W_Kx
                    else if (rx_data == "G") ps <= P_GET;     // [s,GET,...] 读回参数
                    else if (rx_data == "t") ps <= P_T2;      // target
                    else                     ps <= P_IDLE;
            P_US:   ps <= (rx_data == "_") ? P_K : P_IDLE;
            P_K:    ps <= (rx_data == "K") ? P_KIND : P_IDLE;
            P_KIND: if (rx_data == "P")      begin kind <= isw ? 3'd5 : 3'd1; ps <= P_COM2; end
                    else if (rx_data == "D") begin kind <= isw ? 3'd6 : 3'd3; ps <= P_COM2; end
                    else if (isw)            ps <= P_IDLE;   // W_ 只支持 P / D
                    else if (rx_data == "I") begin kind <= 3'd2; ps <= P_COM2; end
                    else if (rx_data == "S") begin kind <= 3'd0; ps <= P_SG; end
                    else                     ps <= P_IDLE;
            // ---- [s,Z_SGN,1] : "S" 后面还有 "GN", 必须跟着走完 ----
            //      以前 'S' 直接进 P_COM2, 下一个字符 'G' 不是 ',' -> 整个包
            //      被丢掉, 于是文档里"串口翻转方向"这条永远不起作用.
            P_SG:   ps <= (rx_data == "G") ? P_SGN : P_IDLE;
            P_SGN:  ps <= (rx_data == "N") ? P_COM2 : P_IDLE;
            // ---- [s,target,180.5] : spell out "target" ----
            P_T2:   ps <= (rx_data == "a") ? P_T3 : P_IDLE;
            P_T3:   ps <= (rx_data == "r") ? P_T4 : P_IDLE;
            P_T4:   ps <= (rx_data == "g") ? P_T5 : P_IDLE;
            P_T5:   ps <= (rx_data == "e") ? P_T6 : P_IDLE;
            P_T6:   if (rx_data == "t") begin kind <= 3'd4; ps <= P_COM3; end
                    else ps <= P_IDLE;
            P_COM2: ps <= (rx_data == ",") ? P_NUM : P_IDLE;
            P_COM3: ps <= (rx_data == ",") ? P_NUM : P_IDLE;
            // [s,GET,...] : 其余字符一律吞掉, 遇到 ] 就把当前参数打出去
            P_GET:  if (rx_data == 8'h5D) begin
                        rep_req <= 1'b1;
                        ps      <= P_IDLE;
                    end
            P_NUM: begin
                if (rx_data == 8'h5D) begin        // ']' -> packet complete
                    ps <= P_IDLE;
                    if (kind == 3'd4) begin
                        // target angle in DEGREES -> 1e-4 rad LSB
                        //   0.1 度 = 17.453 LSB  ->  x 2234 >> 7
                        //   度 -> 0.1 度:  x10  (v10x 已经是 x10 : "181.0" -> 1810)
                        if (v10x > 32'd3600) tgt_q <= 18'd3600;    // 钳到 360.0 度
                        else                 tgt_q <= v10x[17:0];
                        // 2234/128 = 17.453 LSB per 0.1 度
                    end else if (kind == 3'd5) begin           // W_KP (Q12)
                        if (gval <= 24'd65535) gain_wkp <= gval[17:0];
                    end else if (kind == 3'd6) begin           // W_KD (Q12)
                        if (gval <= 24'd65535) gain_wkd <= gval[17:0];
                    end else if (gval <= 24'd10000) begin     // 增益上限 10000
                        if (kind == 3'd0)      gain_sgn <= (gval != 24'd0) ? 18'd1 : 18'd0;
                        // ×7 在这里用移位减法做 (gval*8 - gval), 不占硬件乘法器
                        else if (kind == 3'd1) begin gain_kp <= gval7[17:0]; kpt_q <= gval[15:0]; end
                        else if (kind == 3'd2) begin gain_ki <= gval7[17:0]; kit_q <= gval[15:0]; end
                        else if (kind == 3'd3) begin gain_kd <= gval7[17:0]; kdt_q <= gval[15:0]; end
                    end
                    // echo the whole packet back + CR LF  (blen already
                    // counts the ']' being stored right now)
                    echo_len   <= blen + 5'd1;
                    echo_start <= 1'b1;
                end else if (rx_data == ".") begin
                    dot <= 1'b1;
                end else if (rx_data >= "0" && rx_data <= "9") begin
                    if (!dot || (frac_n != 2'd2)) begin
                        v10 <= v10 * 24'd10 + {16'd0, (rx_data - 8'd48)};
                        if (dot) frac_n <= frac_n + 2'd1;
                    end
                end else if (rx_data == ",") begin
                    // 容错: 数字后面多打一个逗号也认 ("[s,Z_KP,30,]")
                    ps <= P_NUM;
                end else begin
                    ps <= P_IDLE;                  // garbage -> drop packet
                end
            end
            default: ps <= P_IDLE;
            endcase
          end          // if (rx_done)
        end            // else begin
    end
endmodule
