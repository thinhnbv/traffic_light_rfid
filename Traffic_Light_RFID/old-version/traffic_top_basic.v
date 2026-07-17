// =============================================================================
// Module     : traffic_top_basic
// Project    : Traffic Light Controller with RFID + Green Wave
// Description: Basic FPGA top-level for initial testing on Gowin ACG525.
//              Maps traffic light FSM outputs to onboard LEDs.
//              Priority inputs directly from buttons/switches for manual test.
//              No RFID, no cascade — just pure FSM + LEDs.
//
// Target     : Gowin ACG525 (GW5A-25A)
// Clock      : Onboard oscillator → PLL → 50 MHz
//
// Usage      :
//   - Flash to FPGA, observe LED cycling (R/Y/G)
//   - Press priority buttons to test preemption
//   - DIP switches select priority class
//
// Pin assignments: Update traffic_top_basic.cst for your specific board
// =============================================================================

module traffic_top_basic (
    input        clk_in,       // Onboard oscillator (27MHz or 50MHz)
    input        rst_btn_n,    // Push button (active low)

    // Priority test inputs (buttons + switches)
    input        btn_ev_ns,    // Button: simulate EV on NS axis
    input        btn_ev_ew,    // Button: simulate EV on EW axis
    input  [2:0] sw_prio_class,// DIP switches: priority class (1-5)

    // NS Traffic lights
    output       led_ns_red,
    output       led_ns_yellow,
    output       led_ns_green,

    // EW Traffic lights
    output       led_ew_red,
    output       led_ew_yellow,
    output       led_ew_green,

    // Pedestrian indicators
    output       led_ped_ns,
    output       led_ped_ew,

    // Status
    output       led_priority,  // Priority mode active
    output [3:0] debug_state    // FSM state (optional, to header pins)
);

    // =========================================================================
    // Clock Management
    // If onboard oscillator is NOT 50 MHz, use PLL here.
    // For now, assume direct 50 MHz input or adjust CLK_FREQ parameter.
    // =========================================================================
    wire clk = clk_in;  // Direct connection (or PLL output)

    // =========================================================================
    // Button Debouncing
    // Simple 2-FF synchronizer + edge detector for buttons
    // =========================================================================
    reg [2:0] rst_sync;
    reg [2:0] ev_ns_sync;
    reg [2:0] ev_ew_sync;

    always @(posedge clk) begin
        rst_sync  <= {rst_sync[1:0], rst_btn_n};
        ev_ns_sync <= {ev_ns_sync[1:0], btn_ev_ns};
        ev_ew_sync <= {ev_ew_sync[1:0], btn_ev_ew};
    end

    wire rst_n     = rst_sync[2];       // Debounced reset (active low)
    wire ev_ns_raw = ev_ns_sync[2];     // Debounced NS button
    wire ev_ew_raw = ev_ew_sync[2];     // Debounced EW button

    // =========================================================================
    // Simple Button Debouncer with Hold Detection
    // Button must be held for ~20ms to register (1M cycles @50MHz)
    // =========================================================================
    reg [19:0] debounce_ns, debounce_ew;
    reg        ev_ns_stable, ev_ew_stable;

    always @(posedge clk) begin
        if (!rst_n) begin
            debounce_ns <= 20'd0;
            debounce_ew <= 20'd0;
            ev_ns_stable <= 1'b0;
            ev_ew_stable <= 1'b0;
        end else begin
            // NS button debounce
            if (ev_ns_raw) begin
                if (debounce_ns < 20'hF_FFFF)
                    debounce_ns <= debounce_ns + 1'b1;
                else
                    ev_ns_stable <= 1'b1;
            end else begin
                debounce_ns  <= 20'd0;
                ev_ns_stable <= 1'b0;
            end

            // EW button debounce
            if (ev_ew_raw) begin
                if (debounce_ew < 20'hF_FFFF)
                    debounce_ew <= debounce_ew + 1'b1;
                else
                    ev_ew_stable <= 1'b1;
            end else begin
                debounce_ew  <= 20'd0;
                ev_ew_stable <= 1'b0;
            end
        end
    end

    // =========================================================================
    // Priority Class from DIP Switches
    // Apply same class to both NS and EW (selected by which button pressed)
    // =========================================================================
    wire [2:0] ev_class_ns = ev_ns_stable ? sw_prio_class : 3'b000;
    wire [2:0] ev_class_ew = ev_ew_stable ? sw_prio_class : 3'b000;

    // =========================================================================
    // Traffic FSM Instantiation
    // =========================================================================
    wire       ns_red, ns_yellow, ns_green;
    wire       ew_red, ew_yellow, ew_green;
    wire       ped_ns_walk, ped_ew_walk;
    wire       priority_active;
    wire       priority_dir;
    wire [3:0] fsm_state;

    traffic_fsm #(
        .CLK_FREQ     (50_000_000),
        .GREEN_NS_SEC (30),
        .GREEN_EW_SEC (25),
        .YELLOW_SEC   (5),
        .ALL_RED_SEC  (2),
        .PRIO_CLR_SEC (5),
        .PRIO_TMO_SEC (60),
        .SIM_MODE     (0)
    ) u_fsm (
        .clk            (clk),
        .rst_n          (rst_n),
        .ev_detect_ns   (ev_ns_stable),
        .ev_detect_ew   (ev_ew_stable),
        .ev_class_ns    (ev_class_ns),
        .ev_class_ew    (ev_class_ew),
        .ns_red         (ns_red),
        .ns_yellow      (ns_yellow),
        .ns_green       (ns_green),
        .ew_red         (ew_red),
        .ew_yellow      (ew_yellow),
        .ew_green       (ew_green),
        .ped_ns_walk    (ped_ns_walk),
        .ped_ew_walk    (ped_ew_walk),
        .priority_active(priority_active),
        .priority_dir   (priority_dir),
        .current_state  (fsm_state)
    );

    // =========================================================================
    // Output Assignments
    // Active-high LEDs (invert if your board uses active-low LEDs)
    // =========================================================================
    assign led_ns_red    = ns_red;
    assign led_ns_yellow = ns_yellow;
    assign led_ns_green  = ns_green;
    assign led_ew_red    = ew_red;
    assign led_ew_yellow = ew_yellow;
    assign led_ew_green  = ew_green;
    assign led_ped_ns    = ped_ns_walk;
    assign led_ped_ew    = ped_ew_walk;
    assign led_priority  = priority_active;
    assign debug_state   = fsm_state;

endmodule
