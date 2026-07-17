// =============================================================================
// Module     : traffic_fsm
// Project    : TLC-RFID v4.4
// Description: 10-state Moore FSM for traffic light control with priority
//              preemption. Instantiates phase_timer internally.
//              Matches spec Sec 7 and 9.2 port list exactly.
//              No pedestrian signals (removed in v4.0+).
// =============================================================================

module traffic_fsm (
    input  wire       clk,
    input  wire       rst_n,
    input  wire       ev_detect_ns,     // From priority_arbiter
    input  wire       ev_detect_ew,     // From priority_arbiter
    input  wire [2:0] prio_class,       // Winner class from arbiter
    input  wire       cascade_preclear, // From cascade_ctrl
    output reg        ns_r,
    output reg        ns_y,
    output reg        ns_g,
    output reg        ew_r,
    output reg        ew_y,
    output reg        ew_g,
    output wire       prio_active,
    output wire [3:0] cur_state,
    output reg        timer_load,
    output reg [31:0] timer_value
);

    // =========================================================================
    // Phase Timing Parameters (cycles @ 50 MHz)
    // =========================================================================
    `ifdef SIMULATION
        parameter [31:0] CNT_GREEN_NS = 32'd100;
        parameter [31:0] CNT_GREEN_EW = 32'd80;
        parameter [31:0] CNT_YELLOW   = 32'd50;
        parameter [31:0] CNT_ALL_RED  = 32'd20;
        parameter [31:0] CNT_PRIO_CLR = 32'd50;
        parameter [31:0] CNT_PRIO_TMO = 32'd600;
    `else
        parameter [31:0] CNT_GREEN_NS = 32'd1_500_000_000;  // 30s
        parameter [31:0] CNT_GREEN_EW = 32'd1_250_000_000;  // 25s
        parameter [31:0] CNT_YELLOW   = 32'd250_000_000;    // 5s
        parameter [31:0] CNT_ALL_RED  = 32'd100_000_000;    // 2s
        parameter [31:0] CNT_PRIO_CLR = 32'd250_000_000;    // 5s
        parameter [31:0] CNT_PRIO_TMO = 32'd3_000_000_000;  // 60s
    `endif

    // =========================================================================
    // State Encoding
    // =========================================================================
    localparam [3:0] S_NS_GREEN   = 4'd0;
    localparam [3:0] S_NS_YELLOW  = 4'd1;
    localparam [3:0] S_ALL_RED_1  = 4'd2;
    localparam [3:0] S_EW_GREEN   = 4'd3;
    localparam [3:0] S_EW_YELLOW  = 4'd4;
    localparam [3:0] S_ALL_RED_2  = 4'd5;
    localparam [3:0] S_PRIO_CLEAR = 4'd6;
    localparam [3:0] S_PRIO_HOLD  = 4'd7;
    localparam [3:0] S_PRIO_EXIT  = 4'd8;
    localparam [3:0] S_PRIO_RECOV = 4'd9;

    // =========================================================================
    // Internal Signals
    // =========================================================================
    reg [3:0] state, next_state;
    reg [3:0] prev_state;
    reg       prio_dir_reg;       // 0=NS priority, 1=EW priority (latched at PRIO_CLEAR)
    reg       ev_pending;         // Priority event detected during yellow
    reg       ev_pending_dir;     // Direction of pending event (0=NS, 1=EW)

    // ─── Phase Timer Instance ───
    wire timer_expired;

    phase_timer u_timer (
        .clk     (clk),
        .rst_n   (rst_n),
        .load    (timer_load),
        .value   (timer_value),
        .expired (timer_expired)
    );

    // ─── Status Outputs ───
    assign prio_active = (state == S_PRIO_CLEAR) || (state == S_PRIO_HOLD) ||
                         (state == S_PRIO_EXIT)  || (state == S_PRIO_RECOV);
    assign cur_state = state;

    // Priority input helpers
    wire ev_any = ev_detect_ns || ev_detect_ew;

    // =========================================================================
    // State Register
    // =========================================================================
    always @(posedge clk) begin
        if (!rst_n) begin
            state      <= S_NS_GREEN;
            prev_state <= S_NS_GREEN;
        end else begin
            prev_state <= state;
            state      <= next_state;
        end
    end

    // =========================================================================
    // Timer Load — "load on entry" pattern
    // =========================================================================
    wire state_changed = (state != prev_state);

    always @(posedge clk) begin
        if (!rst_n) begin
            timer_load  <= 1'b1;
            timer_value <= CNT_GREEN_NS;
        end else begin
            timer_load <= 1'b0;  // default

            if (state_changed) begin
                timer_load <= 1'b1;
                case (state)
                    S_NS_GREEN:   timer_value <= CNT_GREEN_NS;
                    S_NS_YELLOW:  timer_value <= CNT_YELLOW;
                    S_ALL_RED_1:  timer_value <= CNT_ALL_RED;
                    S_EW_GREEN:   timer_value <= CNT_GREEN_EW;
                    S_EW_YELLOW:  timer_value <= CNT_YELLOW;
                    S_ALL_RED_2:  timer_value <= CNT_ALL_RED;
                    S_PRIO_CLEAR: timer_value <= CNT_PRIO_CLR;
                    S_PRIO_HOLD:  timer_value <= CNT_PRIO_TMO;
                    S_PRIO_EXIT:  timer_value <= CNT_YELLOW;
                    S_PRIO_RECOV: timer_value <= CNT_ALL_RED;
                    default:      timer_value <= CNT_GREEN_NS;
                endcase
            end
        end
    end

    // =========================================================================
    // Priority Direction Latch + Pending Event Logic
    // =========================================================================
    always @(posedge clk) begin
        if (!rst_n) begin
            prio_dir_reg   <= 1'b0;
            ev_pending     <= 1'b0;
            ev_pending_dir <= 1'b0;
        end else begin
            // Latch priority direction when entering PRIO_CLEAR
            if (state != S_PRIO_CLEAR && next_state == S_PRIO_CLEAR) begin
                if (ev_pending) begin
                    prio_dir_reg <= ev_pending_dir;
                    ev_pending   <= 1'b0;
                end else begin
                    prio_dir_reg <= ev_detect_ew ? 1'b1 : 1'b0;
                end
            end

            // Record pending events during yellow phases
            if ((state == S_NS_YELLOW || state == S_EW_YELLOW) && ev_any) begin
                ev_pending     <= 1'b1;
                ev_pending_dir <= ev_detect_ew ? 1'b1 : 1'b0;
            end

            // Clear pending when entering normal green
            if (state == S_NS_GREEN || state == S_EW_GREEN)
                ev_pending <= 1'b0;
        end
    end

    // =========================================================================
    // Next State Logic (Combinational)
    // =========================================================================
    always @(*) begin
        next_state = state;  // Default: stay

        case (state)
            S_NS_GREEN: begin
                if (ev_detect_ew && !ev_detect_ns)
                    next_state = S_NS_YELLOW;  // Interrupt: go yellow first
                else if (ev_detect_ns)
                    next_state = S_PRIO_CLEAR; // Same axis: can go direct
                else if (cascade_preclear)
                    next_state = S_NS_YELLOW;  // Cascade request
                else if (timer_expired)
                    next_state = S_NS_YELLOW;
            end

            S_NS_YELLOW: begin
                if (timer_expired)
                    next_state = S_ALL_RED_1;
            end

            S_ALL_RED_1: begin
                if (timer_expired) begin
                    if (ev_pending || ev_any)
                        next_state = S_PRIO_CLEAR;
                    else
                        next_state = S_EW_GREEN;
                end
            end

            S_EW_GREEN: begin
                if (ev_detect_ns && !ev_detect_ew)
                    next_state = S_EW_YELLOW;
                else if (ev_detect_ew)
                    next_state = S_PRIO_CLEAR;
                else if (cascade_preclear)
                    next_state = S_EW_YELLOW;
                else if (timer_expired)
                    next_state = S_EW_YELLOW;
            end

            S_EW_YELLOW: begin
                if (timer_expired)
                    next_state = S_ALL_RED_2;
            end

            S_ALL_RED_2: begin
                if (timer_expired) begin
                    if (ev_pending || ev_any)
                        next_state = S_PRIO_CLEAR;
                    else
                        next_state = S_NS_GREEN;
                end
            end

            S_PRIO_CLEAR: begin
                if (timer_expired)
                    next_state = S_PRIO_HOLD;
            end

            S_PRIO_HOLD: begin
                if (timer_expired)
                    next_state = S_PRIO_EXIT;  // Timeout 60s
                else if (prio_dir_reg == 1'b0 && !ev_detect_ns)
                    next_state = S_PRIO_EXIT;  // NS tag lost
                else if (prio_dir_reg == 1'b1 && !ev_detect_ew)
                    next_state = S_PRIO_EXIT;  // EW tag lost
            end

            S_PRIO_EXIT: begin
                if (timer_expired)
                    next_state = S_PRIO_RECOV;
            end

            S_PRIO_RECOV: begin
                if (timer_expired)
                    next_state = S_NS_GREEN;  // Return to normal cycle
            end

            default: next_state = S_NS_GREEN;
        endcase
    end

    // =========================================================================
    // Moore Output Logic (based on state only)
    // =========================================================================
    always @(*) begin
        // Defaults: all red
        ns_r = 1'b1;  ns_y = 1'b0;  ns_g = 1'b0;
        ew_r = 1'b1;  ew_y = 1'b0;  ew_g = 1'b0;

        case (state)
            S_NS_GREEN: begin
                ns_r = 1'b0;  ns_g = 1'b1;
            end
            S_NS_YELLOW: begin
                ns_r = 1'b0;  ns_y = 1'b1;
            end
            S_ALL_RED_1: begin
                // All red — defaults
            end
            S_EW_GREEN: begin
                ew_r = 1'b0;  ew_g = 1'b1;
            end
            S_EW_YELLOW: begin
                ew_r = 1'b0;  ew_y = 1'b1;
            end
            S_ALL_RED_2: begin
                // All red — defaults
            end
            S_PRIO_CLEAR: begin
                // All red — clearing intersection
            end
            S_PRIO_HOLD: begin
                if (prio_dir_reg == 1'b0) begin
                    ns_r = 1'b0;  ns_g = 1'b1;  // NS green
                end else begin
                    ew_r = 1'b0;  ew_g = 1'b1;  // EW green
                end
            end
            S_PRIO_EXIT: begin
                if (prio_dir_reg == 1'b0) begin
                    ns_r = 1'b0;  ns_y = 1'b1;  // NS yellow
                end else begin
                    ew_r = 1'b0;  ew_y = 1'b1;  // EW yellow
                end
            end
            S_PRIO_RECOV: begin
                // All red — recovery
            end
            default: begin
                // All red — safe default
            end
        endcase
    end

endmodule
