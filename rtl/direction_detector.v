// =============================================================================
// Module     : direction_detector
// Project    : TLC-RFID v4.4
// Description: Optimized vehicle direction detector. Tracks the sequence of
//              tag_detected falling edges without heavy 32-bit comparators.
//              Latching the last active falling edge reader yields exit direction.
//              Tie-break priority: N(3) > E(1) > S(2) > W(0)
// =============================================================================

module direction_detector (
    input  wire       clk,
    input  wire       rst_n,
    input  wire [3:0] tag_detected,   // [3]=N [2]=S [1]=E [0]=W
    input  wire       any_detected,   // OR of tag_detected
    output reg  [1:0] dir_out,        // 00=N 01=S 10=E 11=W
    output reg        dir_valid,      // 1-cycle pulse when direction confirmed
    output reg        dir_confidence  // 1=normal, 0=low confidence/timeout
);

    // Direction timeout: 10s @ 50 MHz = 500,000,000 cycles
    parameter [31:0] CNT_DIR_TIMEOUT = 32'd500_000_000;

    // FSM States
    localparam S_IDLE     = 2'd0;
    localparam S_TRACKING = 2'd1;
    localparam S_DECIDE   = 2'd2;

    reg [1:0]  state;
    reg [3:0]  prev_tag;                // Previous tag_detected for falling edge
    reg [3:0]  seen_mask;               // Which readers have seen the tag
    reg [31:0] timeout_cnt;             // Direction timeout counter
    reg        timeout_flag;

    // Falling edge detection: bit i = 1 when reader i just lost the tag
    wire [3:0] falling = prev_tag & ~tag_detected;

    // Count bits in seen_mask (popcount)
    wire [2:0] seen_count = seen_mask[0] + seen_mask[1] + seen_mask[2] + seen_mask[3];

    // ─── Incremental exit direction register ───
    // Captures the latest reader that generated a falling edge.
    // Tie-break priority is implemented via if-else priority (North is highest).
    reg [1:0] last_falling_dir;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            last_falling_dir <= 2'b11; // Default to West
        end else begin
            if (falling[3])      last_falling_dir <= 2'b00; // NORTH (Highest priority)
            else if (falling[1]) last_falling_dir <= 2'b10; // EAST
            else if (falling[2]) last_falling_dir <= 2'b01; // SOUTH
            else if (falling[0]) last_falling_dir <= 2'b11; // WEST
        end
    end

    // ─── Track previous tag_detected ───
    always @(posedge clk) begin
        if (!rst_n)
            prev_tag <= 4'b0000;
        else
            prev_tag <= tag_detected;
    end

    // ─── Main FSM ───
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state          <= S_IDLE;
            dir_out        <= 2'b00;
            dir_valid      <= 1'b0;
            dir_confidence <= 1'b0;
            seen_mask      <= 4'b0000;
            timeout_cnt    <= 32'd0;
            timeout_flag   <= 1'b0;
        end else begin
            // Default: clear pulse
            dir_valid <= 1'b0;

            case (state)
                // ─────────────────────────────────────
                S_IDLE: begin
                    if (any_detected) begin
                        seen_mask    <= tag_detected;
                        timeout_cnt  <= 32'd0;
                        timeout_flag <= 1'b0;
                        state        <= S_TRACKING;
                    end
                end

                // ─────────────────────────────────────
                S_TRACKING: begin
                    // Update seen_mask
                    seen_mask <= seen_mask | tag_detected;

                    // Transition conditions
                    if (tag_detected == 4'b0000) begin
                        // All readers lost → decide exit direction
                        state <= S_DECIDE;
                    end else if (timeout_cnt >= CNT_DIR_TIMEOUT) begin
                        // Timeout — vehicle stuck > 10s
                        timeout_flag <= 1'b1;
                        state        <= S_DECIDE;
                    end else begin
                        timeout_cnt <= timeout_cnt + 1;
                    end
                end

                // ─────────────────────────────────────
                S_DECIDE: begin
                    dir_out        <= last_falling_dir;
                    dir_valid      <= 1'b1;
                    dir_confidence <= (seen_count >= 3'd2 && !timeout_flag) ? 1'b1 : 1'b0;
                    state          <= S_IDLE;
                end

                default: state <= S_IDLE;
            endcase
        end
    end

endmodule
