// =====================================================================
//  sense.v  --  sensor front end
//      enc_decode    : 4x quadrature decode of the MG513 Hall encoder
//      udiv32        : 32/24 unsigned divider (used by sa100_capture)
//      sa100_capture : SA100 PWM -> angle (1e-4 rad, 0..2pi)
// =====================================================================
`include "params.vh"


// ---------------------------------------------------------------------
//  4x quadrature decode. pos counts 1040 per arm revolution.
//  step      : one-clock pulse on every edge
//  step_dir  : 1 = pos increased, 0 = pos decreased  (the velocity loop
//              NEEDS this - without a direction the speed measurement is
//              always positive and the velocity loop pushes one way for
//              ever)
// ---------------------------------------------------------------------
module enc_decode (
    input  wire              clk,
    input  wire              rst_n,
    input  wire              a,
    input  wire              b,
    output reg  signed [31:0] pos,
    output reg               step,
    output reg               step_dir
);
    reg [2:0] a_s, b_s;
    always @(posedge clk) begin
        a_s <= {a_s[1:0], a};
        b_s <= {b_s[1:0], b};
    end

    wire [1:0] ab = {a_s[1], b_s[1]};
    reg  [1:0] ab_d;
    always @(posedge clk) ab_d <= ab;

    wire [3:0] tr = {ab_d, ab};

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pos      <= 32'sd0;
            step     <= 1'b0;
            step_dir <= 1'b1;
        end else begin
            step <= 1'b0;
            case (tr)
                4'b00_01, 4'b01_11, 4'b11_10, 4'b10_00: begin
                    pos <= pos + 32'sd1; step <= 1'b1; step_dir <= 1'b1;
                end
                4'b00_10, 4'b10_11, 4'b11_01, 4'b01_00: begin
                    pos <= pos - 32'sd1; step <= 1'b1; step_dir <= 1'b0;
                end
                default: ;
            endcase
        end
    end
endmodule

// ---------------------------------------------------------------------
//  unsigned 32 / 24 restoring divider, 32 clocks, result in q
//  (used by sa100_capture to normalise by the measured PWM period)
// ---------------------------------------------------------------------
module udiv32 (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        start,
    input  wire [31:0] a,
    input  wire [23:0] b,
    output reg  [31:0] q,
    output reg         done
);
    reg [24:0] rem;
    reg [31:0] dvd;
    reg [23:0] dvs;
    reg [5:0]  i;
    reg        run;

    wire [24:0] shifted  = {rem[23:0], dvd[31]};
    wire [24:0] divisor  = {1'b0, dvs};

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            run <= 1'b0; done <= 1'b0; q <= 32'd0;
            rem <= 25'd0; dvd <= 32'd0; dvs <= 24'd1; i <= 6'd0;
        end else begin
            done <= 1'b0;
            if (start && !run) begin
                dvd <= a;
                dvs <= (b == 24'd0) ? 24'd1 : b;
                rem <= 25'd0;
                q   <= 32'd0;
                i   <= 6'd31;
                run <= 1'b1;
            end else if (run) begin
                if (shifted >= divisor) begin
                    rem <= shifted - divisor;
                    q   <= {q[30:0], 1'b1};
                end else begin
                    rem <= shifted;
                    q   <= {q[30:0], 1'b0};
                end
                dvd <= {dvd[30:0], 1'b0};
                if (i == 6'd0) begin
                    run  <= 1'b0;
                    done <= 1'b1;
                end else begin
                    i <= i - 6'd1;
                end
            end
        end
    end
endmodule


// ---------------------------------------------------------------------
//  SA100 : PWM period 1024 us, high time encodes 0..360 deg.
//  angle(1e-4 rad) = high_clk * 360/1024us * ... = (high * 1257) >> 10
//  Latched on every falling edge.
// ---------------------------------------------------------------------
module sa100_capture (
    input  wire               clk,
    input  wire               rst_n,
    input  wire               pwm_in,
    output reg  [17:0]        angle_q,
    output reg                valid
);
    reg [2:0] s;
    always @(posedge clk) s <= {s[1:0], pwm_in};
    wire p = s[1];

    reg [19:0] cnt;      // clocks since the last edge
    reg [19:0] ton;      // high time of the last period
    reg [19:0] per;      // last full period (rising to rising)
    reg        hi;
    reg        start_div;

    // The SA100 datasheet says the PWM frequency is ~976 Hz but varies from
    // chip to chip and with temperature, and it explicitly recommends
    // measuring the period instead of assuming it.  So latch BOTH ton and per.
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            cnt       <= 20'd0;
            ton       <= 20'd0;
            per       <= 20'd51200;
            hi        <= 1'b0;
            start_div <= 1'b0;
        end else begin
            start_div <= 1'b0;
            cnt <= cnt + 20'd1;
            if (p && !hi) begin                    // rising edge -> period
                hi  <= 1'b1;
                per <= (cnt < 20'd1000) ? 20'd51200 : cnt;
                cnt <= 20'd0;
            end else if (!p && hi) begin           // falling edge -> high time
                hi        <= 1'b0;
                ton       <= cnt;
                // NOTE: do NOT clear cnt here!  It has to keep counting so
                // that the next rising edge measures the FULL period
                // (ton + toff).  Clearing it here made "per" equal to the
                // low phase only, which inflated every angle by 1/(1-duty).
                start_div <= 1'b1;                 // trigger the divide
            end
        end
    end

    // ---- datasheet formula:  position = ton*4098/period - 1  (0..4095) ----
    wire [31:0] dvd = {12'd0, ton} * 32'd4098;
    wire [31:0] quo;
    wire        done;

    udiv32 u_div (
        .clk   (clk),
        .rst_n (rst_n),
        .start (start_div),
        .a     (dvd),
        .b     ({4'd0, per}),
        .q     (quo),
        .done  (done)
    );

    // ---- angle in 1e-4 rad:  position * 2pi/4096 = position*15708 >> 10 ----
    reg  [15:0] pos;
    wire [31:0] ang = ({16'd0, pos} * 32'd15708) >> 10;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pos     <= 16'd0;
            angle_q <= 18'd0;
            valid   <= 1'b0;
        end else if (done) begin
            if      (quo >= 32'd4097) pos <= 16'd4095;
            else if (quo == 32'd0)    pos <= 16'd0;
            else                      pos <= quo[15:0] - 16'd1;
            valid <= 1'b1;
        end else if (cnt > `SA_TIMEOUT) begin
            valid <= 1'b0;                         // sensor lost
        end else begin
            angle_q <= (ang > 32'd62832) ? 18'd62832 : ang[17:0];
        end
    end
endmodule
