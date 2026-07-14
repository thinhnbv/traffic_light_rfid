// =============================================================================
// Module     : direction_detector
// Project    : TLC-RFID v4.4
// Description: 3-state FSM that infers vehicle travel direction by tracking
//              the sequence of tag_detected falling edges. The last reader
//              to lose signal = exit direction.
//              Tie-break: N > E > S > W
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
    reg [31:0] tick_counter;            // Monotonic counter
    reg [31:0] lost_time [0:3];         // Timestamp when each reader lost tag
    reg [3:0]  seen_mask;               // Which readers have seen the tag
    reg [31:0] timeout_cnt;             // Direction timeout counter
    reg        timeout_flag;

    // Falling edge detection: bit i = 1 when reader i just lost the tag
    wire [3:0] falling = prev_tag & ~tag_detected;

    // Count bits in seen_mask (popcount) — simple for 4 bits
    wire [2:0] seen_count = seen_mask[0] + seen_mask[1] + seen_mask[2] + seen_mask[3];

    // ─── Argmax: find reader with largest lost_time (last to lose) ───
    // Tie-break priority: N(3) > E(1) > S(2) > W(0)
    // Check in order W → S → E → N so that later (higher priority) wins on >=
    reg [1:0]  max_dir;
    reg [31:0] max_val;

    always @(*) begin
        // Start with W (lowest priority)
        max_val = lost_time[0];
        max_dir = 2'b11;  // WEST

        // East
        if (lost_time[1] >= max_val) begin
            max_val = lost_time[1];
            max_dir = 2'b10;  // EAST
        end
        // South
        if (lost_time[2] >= max_val) begin
            max_val = lost_time[2];
            max_dir = 2'b01;  // SOUTH
        end
        // North (highest priority — checked last so wins on tie)
        if (lost_time[3] >= max_val) begin
            max_val = lost_time[3];
            max_dir = 2'b00;  // NORTH
        end
    end

    // ─── Track previous tag_detected ───
    always @(posedge clk) begin
        if (!rst_n)
            prev_tag <= 4'b0000;
        else
            prev_tag <= tag_detected;
    end

    // ─── Tick counter — runs continuously ───
    always @(posedge clk) begin
        if (!rst_n)
            tick_counter <= 32'd0;
        else
            tick_counter <= tick_counter + 1;
    end

    // ─── Main FSM ───
    always @(posedge clk) begin
        if (!rst_n) begin
            state          <= S_IDLE;
            dir_out        <= 2'b00;
            dir_valid      <= 1'b0;
            dir_confidence <= 1'b0;
            seen_mask      <= 4'b0000;
            timeout_cnt    <= 32'd0;
            timeout_flag   <= 1'b0;
            lost_time[0]   <= 32'd0;
            lost_time[1]   <= 32'd0;
            lost_time[2]   <= 32'd0;
            lost_time[3]   <= 32'd0;
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
                        lost_time[0] <= 32'd0;
                        lost_time[1] <= 32'd0;
                        lost_time[2] <= 32'd0;
                        lost_time[3] <= 32'd0;
                        state        <= S_TRACKING;
                    end
                end

                // ─────────────────────────────────────
                S_TRACKING: begin
                    // Update seen_mask
                    seen_mask <= seen_mask | tag_detected;

                    // Record falling edges
                    if (falling[0]) lost_time[0] <= tick_counter;
                    if (falling[1]) lost_time[1] <= tick_counter;
                    if (falling[2]) lost_time[2] <= tick_counter;
                    if (falling[3]) lost_time[3] <= tick_counter;

                    // Transition conditions
                    if (tag_detected == 4'b0000) begin
                        // All readers lost → decide
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
                    dir_out        <= max_dir;
                    dir_valid      <= 1'b1;
                    dir_confidence <= (seen_count >= 3'd2 && !timeout_flag) ? 1'b1 : 1'b0;
                    state          <= S_IDLE;
                end

                default: state <= S_IDLE;
            endcase
        end
    end

endmodule
