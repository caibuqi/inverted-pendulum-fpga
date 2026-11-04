// =====================================================================
//  ui.v  --  buttons and 4-digit display
//      key_debounce : 20 ms debounce, one-clock press pulses
//      seg7         : 4-digit common-anode dynamic scan + decode
//      bin2bcd      : binary -> 4 BCD digits (saturating at 9999)
// =====================================================================
`include "params.vh"


// ---------------------------------------------------------------------
//  20 ms debounce for 8 active-low keys.
//  press[i] is a one-clock pulse on the rising edge of the debounced
//  level (i.e. the moment the key is actually pressed).
// ---------------------------------------------------------------------
module key_debounce (
    input  wire       clk,
    input  wire       rst_n,
    input  wire [7:0] key_n,
    output reg  [7:0] level,        // debounced, 1 = pressed
    output reg  [7:0] press         // one-clock pulse per press
);
    wire [7:0] raw = ~key_n;

    // ---- 1 ms tick ----
    reg [15:0] ms_div;
    reg        ms_tick;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            ms_div  <= 16'd0;
            ms_tick <= 1'b0;
        end else if (ms_div == `TICK_CLK - 1) begin
            ms_div  <= 16'd0;
            ms_tick <= 1'b1;
        end else begin
            ms_div  <= ms_div + 16'd1;
            ms_tick <= 1'b0;
        end
    end

    reg [4:0] same;                 // consecutive equal 1 ms samples
    reg [7:0] last;
    reg [7:0] level_r;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            same    <= 5'd0;
            last    <= 8'd0;
            level_r <= 8'd0;
            level   <= 8'd0;
            press   <= 8'd0;
        end else begin
            level <= level_r;
            press <= level_r & ~level;

            if (ms_tick) begin
                if (raw == last) begin
                    if (same != 5'd20) same <= same + 5'd1;
                    else               level_r <= last;
                end else begin
                    same <= 5'd0;
                    last <= raw;
                end
            end
        end
    end
endmodule


// ---------------------------------------------------------------------
//  4-digit scan, 250 us per digit, common anode, ACTIVE LOW.
// ---------------------------------------------------------------------
module seg7 (
    input  wire        clk,
    input  wire        rst_n,
    input  wire [15:0] bcd,         // {d3,d2,d1,d0}
    input  wire [3:0]  blank,       // 1 = blank that digit
    output wire [7:0]  seg_n,       // {dp,g,f,e,d,c,b,a}
    output wire [3:0]  dig_n
);
    reg [13:0] sc;
    reg [1:0]  sel;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            sc  <= 14'd0;
            sel <= 2'd0;
        end else if (sc == 14'd12499) begin
            sc  <= 14'd0;
            sel <= sel + 2'd1;
        end else begin
            sc <= sc + 14'd1;
        end
    end

    // SMG_DIG1 is the leftmost digit, so sel=0 must show the MOST
    // significant digit for the number to read left-to-right.
    // `~sel` on 2 bits == 3-sel.
    wire [1:0] bidx = `SEG_DIG_REV ? (~sel) : sel;

    reg [3:0] cur;
    always @(*) begin
        case (bidx)
            2'd0:    cur = bcd[3:0];
            2'd1:    cur = bcd[7:4];
            2'd2:    cur = bcd[11:8];
            default: cur = bcd[15:12];
        endcase
    end

    // active HIGH pattern {dp,g,f,e,d,c,b,a}
    function [7:0] pat;
        input [3:0] v;
        begin
            case (v)
                4'd0:    pat = 8'h3F;
                4'd1:    pat = 8'h06;
                4'd2:    pat = 8'h5B;
                4'd3:    pat = 8'h4F;
                4'd4:    pat = 8'h66;
                4'd5:    pat = 8'h6D;
                4'd6:    pat = 8'h7D;
                4'd7:    pat = 8'h07;
                4'd8:    pat = 8'h7F;
                4'd9:    pat = 8'h6F;
                4'd10:   pat = 8'h40;      // '-'
                4'd11:   pat = 8'h76;      // 'H'
                4'd12:   pat = 8'h3E;      // 'U'
                4'd13:   pat = 8'h79;      // 'E'
                4'd14:   pat = 8'h38;      // 'L'
                4'd15:   pat = 8'h00;      // blank
                default: pat = 8'h00;
            endcase
        end
    endfunction

    assign seg_n = (~pat(cur)) | {8{blank[bidx]}};
    assign dig_n = ~(4'b0001 << sel);
endmodule


// ---------------------------------------------------------------------
//  binary -> 4 BCD digits, saturating at 9999 (double dabble)
// ---------------------------------------------------------------------
module bin2bcd (
    input  wire [15:0] bin_in,
    output reg  [15:0] bcd
);
    wire [15:0] bin = (bin_in > 16'd9999) ? 16'd9999 : bin_in;

    reg [31:0] sh;
    integer i, k;

    always @(*) begin
        sh = {16'd0, bin};
        for (i = 0; i < 16; i = i + 1) begin
            for (k = 0; k < 4; k = k + 1) begin
                if (sh[16 + k*4 +: 4] >= 4'd5)
                    sh[16 + k*4 +: 4] = sh[16 + k*4 +: 4] + 4'd3;
            end
            sh = sh << 1;
        end
        bcd = sh[31:16];
    end
endmodule
