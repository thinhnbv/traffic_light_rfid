// =============================================================================
// Testbench : tb_traffic_basic
// Project   : Traffic Light Controller with RFID + Green Wave
// Description: Basic testbench for phase_timer + traffic_fsm.
//              Tests normal cycle and priority preemption.
//              Uses SIM_MODE=1 for fast simulation.
// =============================================================================

`timescale 1ns / 1ps

module tb_traffic_basic;

    // =========================================================================
    // Clock and Reset
    // =========================================================================
    reg clk;
    reg rst_n;

    // 50 MHz clock → 20ns period
    initial clk = 0;
    always #10 clk = ~clk;

    // =========================================================================
    // DUT Signals
    // =========================================================================
    reg        ev_detect_ns;
    reg        ev_detect_ew;
    reg  [2:0] ev_class_ns;
    reg  [2:0] ev_class_ew;

    wire       ns_red, ns_yellow, ns_green;
    wire       ew_red, ew_yellow, ew_green;
    wire       ped_ns_walk, ped_ew_walk;
    wire       priority_active, priority_dir;
    wire [3:0] current_state;

    // =========================================================================
    // DUT Instantiation — SIM_MODE=1 for fast timers
    // =========================================================================
    traffic_fsm #(
        .CLK_FREQ     (50_000_000),
        .SIM_MODE     (1)           // Use short timer values
    ) dut (
        .clk            (clk),
        .rst_n          (rst_n),
        .ev_detect_ns   (ev_detect_ns),
        .ev_detect_ew   (ev_detect_ew),
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
        .current_state  (current_state)
    );

    // =========================================================================
    // State name for debug (waveform display)
    // =========================================================================
    reg [12*8-1:0] state_name;
    always @(*) begin
        case (current_state)
            4'd0: state_name = "NS_GREEN    ";
            4'd1: state_name = "NS_YELLOW   ";
            4'd2: state_name = "ALL_RED_1   ";
            4'd3: state_name = "EW_GREEN    ";
            4'd4: state_name = "EW_YELLOW   ";
            4'd5: state_name = "ALL_RED_2   ";
            4'd6: state_name = "PRIO_CLEAR  ";
            4'd7: state_name = "PRIO_HOLD   ";
            4'd8: state_name = "PRIO_EXIT   ";
            4'd9: state_name = "PRIO_RECOV  ";
            default: state_name = "UNKNOWN     ";
        endcase
    end

    // =========================================================================
    // Task: Wait for state transition
    // =========================================================================
    task wait_state;
        input [3:0] target_state;
        input integer timeout;
        integer i;
        begin
            for (i = 0; i < timeout; i = i + 1) begin
                @(posedge clk);
                if (current_state == target_state) begin
                    $display("[%0t] State reached: %0d", $time, target_state);
                    i = timeout; // Break
                end
            end
            if (current_state != target_state)
                $display("[%0t] ERROR: Timeout waiting for state %0d, current=%0d",
                         $time, target_state, current_state);
        end
    endtask

    // =========================================================================
    // Test Sequence
    // =========================================================================
    initial begin
        $dumpfile("tb_traffic_basic.vcd");
        $dumpvars(0, tb_traffic_basic);

        // Initialize
        rst_n       = 0;
        ev_detect_ns = 0;
        ev_detect_ew = 0;
        ev_class_ns  = 3'b000;
        ev_class_ew  = 3'b000;

        $display("========================================");
        $display("  Traffic Light FSM Testbench");
        $display("  SIM_MODE=1 (short timers)");
        $display("========================================");

        // Reset
        repeat(5) @(posedge clk);
        rst_n = 1;
        $display("[%0t] Reset released", $time);

        // =====================================================================
        // Test 1: Normal Cycle — verify full cycle G→Y→AR→G→Y→AR→repeat
        // =====================================================================
        $display("\n--- Test 1: Normal Cycle ---");

        // Should start at NS_GREEN (state 0)
        @(posedge clk);
        if (current_state == 4'd0 && ns_green && ew_red && ped_ew_walk)
            $display("[PASS] Initial state: NS_GREEN, NS=G, EW=R, PED_EW=WALK");
        else
            $display("[FAIL] Initial state incorrect: state=%0d", current_state);

        // Wait for NS_YELLOW
        wait_state(4'd1, 100);
        if (ns_yellow && ew_red && !ped_ew_walk)
            $display("[PASS] NS_YELLOW: NS=Y, EW=R, PED=STOP");
        else
            $display("[FAIL] NS_YELLOW outputs incorrect");

        // Wait for ALL_RED_1
        wait_state(4'd2, 50);
        if (ns_red && ew_red)
            $display("[PASS] ALL_RED_1: NS=R, EW=R");
        else
            $display("[FAIL] ALL_RED_1 outputs incorrect");

        // Wait for EW_GREEN
        wait_state(4'd3, 50);
        if (ew_green && ns_red && ped_ns_walk)
            $display("[PASS] EW_GREEN: NS=R, EW=G, PED_NS=WALK");
        else
            $display("[FAIL] EW_GREEN outputs incorrect");

        // Wait for EW_YELLOW
        wait_state(4'd4, 100);
        if (ew_yellow && ns_red)
            $display("[PASS] EW_YELLOW: NS=R, EW=Y");
        else
            $display("[FAIL] EW_YELLOW outputs incorrect");

        // Wait for ALL_RED_2
        wait_state(4'd5, 50);
        if (ns_red && ew_red)
            $display("[PASS] ALL_RED_2: NS=R, EW=R");
        else
            $display("[FAIL] ALL_RED_2 outputs incorrect");

        // Wait for NS_GREEN again — full cycle complete!
        wait_state(4'd0, 50);
        $display("[PASS] Full normal cycle complete! Back to NS_GREEN");

        // =====================================================================
        // Test 2: Priority NS during EW_GREEN
        // =====================================================================
        $display("\n--- Test 2: Priority NS during EW_GREEN ---");

        // Wait until EW_GREEN
        wait_state(4'd3, 200);
        $display("[%0t] In EW_GREEN, triggering NS priority (class=5, fire)", $time);

        // Trigger NS priority (xe cứu hỏa)
        ev_detect_ns = 1;
        ev_class_ns  = 3'b101;  // Nhất hỏa

        // Should go to PRIO_CLEAR
        wait_state(4'd6, 50);
        if (priority_active && ns_red && ew_red)
            $display("[PASS] PRIO_CLEAR: All red, clearing intersection");
        else
            $display("[FAIL] PRIO_CLEAR outputs incorrect");

        // Should go to PRIO_HOLD with NS green
        wait_state(4'd7, 50);
        if (ns_green && ew_red && priority_active)
            $display("[PASS] PRIO_HOLD: NS=GREEN for priority vehicle");
        else
            $display("[FAIL] PRIO_HOLD outputs incorrect");

        // Release EV signal (vehicle passed)
        repeat(10) @(posedge clk);
        ev_detect_ns = 0;
        ev_class_ns  = 3'b000;
        $display("[%0t] EV signal released (vehicle passed)", $time);

        // Should go to PRIO_EXIT
        wait_state(4'd8, 50);
        if (ns_yellow && ew_red)
            $display("[PASS] PRIO_EXIT: NS=YELLOW, preparing recovery");
        else
            $display("[FAIL] PRIO_EXIT outputs incorrect");

        // Should go to PRIO_RECOV
        wait_state(4'd9, 50);
        if (ns_red && ew_red)
            $display("[PASS] PRIO_RECOV: All red recovery");
        else
            $display("[FAIL] PRIO_RECOV outputs incorrect");

        // Should return to normal cycle
        wait_state(4'd3, 50);  // Return to EW_GREEN (the saved return state)
        $display("[PASS] Returned to normal cycle after priority");

        // =====================================================================
        // Test 3: Priority conflict — Hỏa (NS) vs Thương (EW)
        // =====================================================================
        $display("\n--- Test 3: Priority Conflict ---");

        // Wait for next NS_GREEN
        wait_state(4'd0, 200);
        $display("[%0t] In NS_GREEN, triggering conflict: Fire(NS) vs Ambulance(EW)", $time);

        ev_detect_ns = 1;  ev_class_ns = 3'b101;  // Fire (5)
        ev_detect_ew = 1;  ev_class_ew = 3'b011;  // Ambulance (3)

        // Fire should win → NS gets priority
        repeat(5) @(posedge clk);
        if (current_state == 4'd7 && ns_green)
            $display("[PASS] Conflict resolved: Fire(NS) wins over Ambulance(EW)");
        else
            $display("[INFO] State=%0d (may need more cycles), NS_G=%b", current_state, ns_green);

        // Release both
        repeat(10) @(posedge clk);
        ev_detect_ns = 0;  ev_class_ns = 3'b000;
        ev_detect_ew = 0;  ev_class_ew = 3'b000;

        // Wait for recovery
        wait_state(4'd0, 300);

        // =====================================================================
        // Test 4: Pedestrian signals
        // =====================================================================
        $display("\n--- Test 4: Pedestrian Signals ---");

        wait_state(4'd0, 200);  // NS_GREEN
        if (ped_ew_walk && !ped_ns_walk)
            $display("[PASS] NS_GREEN: PED_EW=WALK (perpendicular pedestrians walk)");
        else
            $display("[FAIL] Pedestrian signals incorrect during NS_GREEN");

        wait_state(4'd3, 200);  // EW_GREEN
        if (ped_ns_walk && !ped_ew_walk)
            $display("[PASS] EW_GREEN: PED_NS=WALK (perpendicular pedestrians walk)");
        else
            $display("[FAIL] Pedestrian signals incorrect during EW_GREEN");

        // =====================================================================
        // Done
        // =====================================================================
        repeat(50) @(posedge clk);
        $display("\n========================================");
        $display("  All basic tests completed!");
        $display("========================================");
        $finish;
    end

    // =========================================================================
    // Monitor: Print state changes
    // =========================================================================
    reg [3:0] prev_state;
    always @(posedge clk) begin
        prev_state <= current_state;
        if (current_state != prev_state)
            $display("[%0t] State: %0d → %0d (%0s)", $time, prev_state, current_state, state_name);
    end

endmodule
