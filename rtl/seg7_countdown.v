// =============================================================================
// Module     : seg7_countdown
// Project    : TLC-RFID v4.4
// Description: Countdown display driver for 8-digit 7-segment (2 strips × 4 digits)
//              driven by daisy-chained 74HC595 shift registers.
//
//              Strip 1 (Digits 1-4): NS axis countdown (seconds)
//              Strip 2 (Digits 5-8): EW axis countdown (seconds)
//
//              Hiển thị:
//              - Pha XANH:  đếm ngược số giây còn lại của pha xanh
//              - Pha VÀNG:  đếm ngược số giây còn lại của pha vàng
//              - Pha ĐỎ:   đếm ngược tổng giây chờ đến khi được xanh
//              - Pha ƯU TIÊN: hiển thị "P---"
//
//              Multiplexed scanning: 8 digits qua 2 × 74HC595
//              Byte 1 (shift first): Segment data (a-g + dp)
//              Byte 2 (shift second): Digit select (active-high, one-hot)
// =============================================================================

module seg7_countdown #(
    // Thời lượng mỗi pha tính bằng giây (phải khớp với traffic_fsm)
    `ifdef SIMULATION
        parameter [6:0] SEC_GREEN_NS = 7'd3,
        parameter [6:0] SEC_GREEN_EW = 7'd2,
        parameter [6:0] SEC_YELLOW   = 7'd1,
        parameter [6:0] SEC_ALL_RED  = 7'd1,
        parameter [6:0] SEC_PRIO_CLR = 7'd1,
        parameter [6:0] SEC_PRIO_TMO = 7'd5
    `else
        parameter [6:0] SEC_GREEN_NS = 7'd30,
        parameter [6:0] SEC_GREEN_EW = 7'd25,
        parameter [6:0] SEC_YELLOW   = 7'd5,
        parameter [6:0] SEC_ALL_RED  = 7'd2,
        parameter [6:0] SEC_PRIO_CLR = 7'd5,
        parameter [6:0] SEC_PRIO_TMO = 7'd60
    `endif
)(
    input  wire       clk,          // 50 MHz
    input  wire       rst_n,
    input  wire [3:0] cur_state,    // From traffic_fsm
    output reg        seg7_dio,     // Serial data to 74HC595
    output reg        seg7_sclk,    // Shift clock
    output reg        seg7_rclk     // Latch clock
);

    // =========================================================================
    // FSM State Encodings (must match traffic_fsm)
    // =========================================================================
    localparam S_NS_GREEN   = 4'd0;
    localparam S_NS_YELLOW  = 4'd1;
    localparam S_ALL_RED_1  = 4'd2;
    localparam S_EW_GREEN   = 4'd3;
    localparam S_EW_YELLOW  = 4'd4;
    localparam S_ALL_RED_2  = 4'd5;
    localparam S_PRIO_CLEAR = 4'd6;
    localparam S_PRIO_HOLD  = 4'd7;
    localparam S_PRIO_EXIT  = 4'd8;
    localparam S_PRIO_RECOV = 4'd9;

    // =========================================================================
    // 7-Segment Encoding (Common Cathode: 1 = segment ON)
    //    a
    //   ---
    // f|   |b
    //   ---  <- g
    // e|   |c
    //   ---
    //    d      dp
    // Bit order: {dp, g, f, e, d, c, b, a}
    // =========================================================================
    function [7:0] seg7_encode;
        input [3:0] digit;
        begin
            case (digit)
                4'd0: seg7_encode = 8'b0_0111111; // 0x3F
                4'd1: seg7_encode = 8'b0_0000110; // 0x06
                4'd2: seg7_encode = 8'b0_1011011; // 0x5B
                4'd3: seg7_encode = 8'b0_1001111; // 0x4F
                4'd4: seg7_encode = 8'b0_1100110; // 0x66
                4'd5: seg7_encode = 8'b0_1101101; // 0x6D
                4'd6: seg7_encode = 8'b0_1111101; // 0x7D
                4'd7: seg7_encode = 8'b0_0000111; // 0x07
                4'd8: seg7_encode = 8'b0_1111111; // 0x7F
                4'd9: seg7_encode = 8'b0_1101111; // 0x6F
                default: seg7_encode = 8'b0_0000000; // blank
            endcase
        end
    endfunction

    // Special patterns
    localparam SEG_P     = 8'b0_1110011; // "P"
    localparam SEG_DASH  = 8'b0_1000000; // "-"
    localparam SEG_BLANK = 8'b0_0000000; // blank

    // =========================================================================
    // 1-Second Tick Generator
    // =========================================================================
    `ifdef SIMULATION
        localparam [25:0] ONE_SEC_COUNT = 26'd100; // Short for simulation
    `else
        localparam [25:0] ONE_SEC_COUNT = 26'd49_999_999; // 50M - 1
    `endif

    reg [25:0] sec_cnt;
    reg        sec_tick;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            sec_cnt  <= 26'd0;
            sec_tick <= 1'b0;
        end else begin
            if (sec_cnt >= ONE_SEC_COUNT) begin
                sec_cnt  <= 26'd0;
                sec_tick <= 1'b1;
            end else begin
                sec_cnt  <= sec_cnt + 1;
                sec_tick <= 1'b0;
            end
        end
    end

    // =========================================================================
    // State Change Detector
    // =========================================================================
    reg [3:0] prev_state;
    wire state_changed = (cur_state != prev_state);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            prev_state <= S_NS_GREEN;
        else
            prev_state <= cur_state;
    end

    // =========================================================================
    // NS / EW Countdown Counters (in seconds)
    // =========================================================================
    reg [6:0] ns_seconds;
    reg [6:0] ew_seconds;

    // Lookup: how many seconds to load for each axis when state changes
    // NS axis: shows own countdown when active, shows wait-time when inactive
    // EW axis: same logic, mirrored

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            ns_seconds <= SEC_GREEN_NS;
            ew_seconds <= SEC_GREEN_NS + SEC_YELLOW + SEC_ALL_RED;
        end else begin
            if (state_changed) begin
                // ─── Load new countdown values on state transitions ───
                case (cur_state)
                    S_NS_GREEN: begin
                        ns_seconds <= SEC_GREEN_NS;
                        ew_seconds <= SEC_GREEN_NS + SEC_YELLOW + SEC_ALL_RED;
                    end
                    S_NS_YELLOW: begin
                        ns_seconds <= SEC_YELLOW;
                        ew_seconds <= SEC_YELLOW + SEC_ALL_RED;
                    end
                    S_ALL_RED_1: begin
                        ns_seconds <= SEC_ALL_RED;
                        ew_seconds <= SEC_ALL_RED;
                    end
                    S_EW_GREEN: begin
                        ns_seconds <= SEC_GREEN_EW + SEC_YELLOW + SEC_ALL_RED;
                        ew_seconds <= SEC_GREEN_EW;
                    end
                    S_EW_YELLOW: begin
                        ns_seconds <= SEC_YELLOW + SEC_ALL_RED;
                        ew_seconds <= SEC_YELLOW;
                    end
                    S_ALL_RED_2: begin
                        ns_seconds <= SEC_ALL_RED;
                        ew_seconds <= SEC_ALL_RED;
                    end
                    S_PRIO_CLEAR: begin
                        ns_seconds <= 7'd0;
                        ew_seconds <= 7'd0;
                    end
                    S_PRIO_HOLD: begin
                        ns_seconds <= SEC_PRIO_TMO;
                        ew_seconds <= SEC_PRIO_TMO;
                    end
                    S_PRIO_EXIT: begin
                        ns_seconds <= SEC_YELLOW;
                        ew_seconds <= SEC_YELLOW;
                    end
                    S_PRIO_RECOV: begin
                        ns_seconds <= SEC_ALL_RED;
                        ew_seconds <= SEC_ALL_RED;
                    end
                    default: begin
                        ns_seconds <= 7'd0;
                        ew_seconds <= 7'd0;
                    end
                endcase
            end else if (sec_tick) begin
                // ─── Decrement every second ───
                if (ns_seconds > 0) ns_seconds <= ns_seconds - 1;
                if (ew_seconds > 0) ew_seconds <= ew_seconds - 1;
            end
        end
    end

    // =========================================================================
    // BCD Conversion (0-99 → tens and ones digits)
    // =========================================================================
    wire [3:0] ns_tens = ns_seconds / 10;
    wire [3:0] ns_ones = ns_seconds % 10;
    wire [3:0] ew_tens = ew_seconds / 10;
    wire [3:0] ew_ones = ew_seconds % 10;

    // =========================================================================
    // 8-Digit Segment Data Preparation
    // Digit layout: [D1 D2 D3 D4] [D5 D6 D7 D8]
    //               <── Strip 1 ──> <── Strip 2 ──>
    //               NS axis         EW axis
    // =========================================================================
    wire is_prio = (cur_state >= S_PRIO_CLEAR && cur_state <= S_PRIO_RECOV);

    reg [7:0] digit_seg [0:7]; // Segment data for each of 8 digits

    always @(*) begin
        if (is_prio) begin
            // Priority mode: show "P--" on both strips
            digit_seg[0] = SEG_P;
            digit_seg[1] = SEG_DASH;
            digit_seg[2] = seg7_encode(ns_tens);
            digit_seg[3] = seg7_encode(ns_ones);
            digit_seg[4] = SEG_P;
            digit_seg[5] = SEG_DASH;
            digit_seg[6] = seg7_encode(ew_tens);
            digit_seg[7] = seg7_encode(ew_ones);
        end else begin
            // Normal mode: show countdown
            // Strip 1: NS axis
            digit_seg[0] = SEG_BLANK;
            digit_seg[1] = SEG_BLANK;
            digit_seg[2] = (ns_tens == 0) ? SEG_BLANK : seg7_encode(ns_tens);
            digit_seg[3] = seg7_encode(ns_ones);
            // Strip 2: EW axis
            digit_seg[4] = SEG_BLANK;
            digit_seg[5] = SEG_BLANK;
            digit_seg[6] = (ew_tens == 0) ? SEG_BLANK : seg7_encode(ew_tens);
            digit_seg[7] = seg7_encode(ew_ones);
        end
    end

    // =========================================================================
    // 74HC595 Shift-Out & Multiplexing FSM
    //
    // Protocol per digit:
    //   1. Shift out 16 bits: {digit_select[7:0], segment_data[7:0]}
    //      - Segment data shifts into first 74HC595
    //      - Digit select shifts into second 74HC595
    //   2. Pulse RCLK to latch outputs
    //   3. Move to next digit
    //
    // Scanning: cycle through digits 0-7 continuously
    // SCLK timing: toggle every SHIFT_DIV clocks (~1MHz at div=25)
    // =========================================================================
    localparam SHIFT_DIV = 6'd25; // SCLK = 50MHz / (2*25) = 1MHz

    localparam SH_IDLE    = 2'd0;
    localparam SH_SHIFT   = 2'd1;
    localparam SH_LATCH   = 2'd2;
    localparam SH_PAUSE   = 2'd3;

    reg [1:0]  sh_state;
    reg [2:0]  scan_digit;     // Current digit being displayed (0-7)
    reg [4:0]  bit_idx;        // Current bit being shifted (0-15)
    reg [5:0]  clk_div_cnt;    // Clock divider for SCLK
    reg        sclk_phase;     // 0=set data, 1=pulse SCLK
    reg [15:0] shift_data;     // {digit_select[7:0], segment_data[7:0]}
    reg [5:0]  pause_cnt;      // Brief pause between digits

    // Digit select pattern (one-hot, active-high)
    wire [7:0] digit_sel = (8'd1 << scan_digit);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            sh_state    <= SH_IDLE;
            scan_digit  <= 3'd0;
            bit_idx     <= 5'd0;
            clk_div_cnt <= 6'd0;
            sclk_phase  <= 1'b0;
            shift_data  <= 16'd0;
            pause_cnt   <= 6'd0;
            seg7_dio    <= 1'b0;
            seg7_sclk   <= 1'b0;
            seg7_rclk   <= 1'b0;
        end else begin
            case (sh_state)
                // ─── Prepare shift data for current digit ───
                SH_IDLE: begin
                    shift_data  <= {digit_sel, digit_seg[scan_digit]};
                    bit_idx     <= 5'd15; // MSB first
                    sclk_phase  <= 1'b0;
                    clk_div_cnt <= 6'd0;
                    seg7_rclk   <= 1'b0;
                    sh_state    <= SH_SHIFT;
                end

                // ─── Shift out 16 bits ───
                SH_SHIFT: begin
                    clk_div_cnt <= clk_div_cnt + 1;
                    if (clk_div_cnt >= SHIFT_DIV) begin
                        clk_div_cnt <= 6'd0;
                        if (!sclk_phase) begin
                            // Phase 0: Set DIO to current bit
                            seg7_dio  <= shift_data[bit_idx];
                            seg7_sclk <= 1'b0;
                            sclk_phase <= 1'b1;
                        end else begin
                            // Phase 1: Rising edge of SCLK (data latched by 74HC595)
                            seg7_sclk  <= 1'b1;
                            sclk_phase <= 1'b0;
                            if (bit_idx == 0) begin
                                // All 16 bits shifted
                                sh_state <= SH_LATCH;
                            end else begin
                                bit_idx <= bit_idx - 1;
                            end
                        end
                    end
                end

                // ─── Latch: pulse RCLK ───
                SH_LATCH: begin
                    clk_div_cnt <= clk_div_cnt + 1;
                    if (clk_div_cnt < SHIFT_DIV) begin
                        seg7_sclk <= 1'b0;
                        seg7_rclk <= 1'b1; // Rising edge latches data
                    end else begin
                        seg7_rclk <= 1'b0;
                        pause_cnt <= 6'd0;
                        sh_state  <= SH_PAUSE;
                    end
                end

                // ─── Brief pause, then next digit ───
                SH_PAUSE: begin
                    pause_cnt <= pause_cnt + 1;
                    if (pause_cnt >= 6'd10) begin
                        scan_digit <= (scan_digit == 3'd7) ? 3'd0 : scan_digit + 1;
                        sh_state   <= SH_IDLE;
                    end
                end

                default: sh_state <= SH_IDLE;
            endcase
        end
    end

endmodule
