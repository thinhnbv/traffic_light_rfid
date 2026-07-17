// =============================================================================
// Testbench : tb_traffic_light_simple
// Description: Testbench cho đèn giao thông đơn giản
//              Override timing ngắn lại để sim nhanh
// =============================================================================

`timescale 1ns / 1ps

module tb_traffic_light_simple;

    // Clock & Reset
    reg clk;
    reg rst_n;

    // 50 MHz → 20ns period
    initial clk = 0;
    always #10 clk = ~clk;

    // Outputs
    wire ns_red, ns_yellow, ns_green;
    wire ew_red, ew_yellow, ew_green;

    // DUT — override timing ngắn để sim nhanh (5/2/1 cycles thay vì giây)
    traffic_light_simple #(
        .CLK_FREQ     (1),    // 1 Hz giả → mỗi "giây" = 1 clock cycle
        .GREEN_TIME   (5),    // 5 cycles xanh
        .YELLOW_TIME  (2),    // 2 cycles vàng
        .RED_TIME     (1)     // 1 cycle all-red
    ) dut (
        .clk       (clk),
        .rst_n     (rst_n),
        .ns_red    (ns_red),
        .ns_yellow (ns_yellow),
        .ns_green  (ns_green),
        .ew_red    (ew_red),
        .ew_yellow (ew_yellow),
        .ew_green  (ew_green)
    );

    // State name cho waveform
    reg [11*8-1:0] state_name;
    always @(*) begin
        case (dut.state)
            3'd0: state_name = "NS_GREEN   ";
            3'd1: state_name = "NS_YELLOW  ";
            3'd2: state_name = "ALL_RED_1  ";
            3'd3: state_name = "EW_GREEN   ";
            3'd4: state_name = "EW_YELLOW  ";
            3'd5: state_name = "ALL_RED_2  ";
            default: state_name = "UNKNOWN    ";
        endcase
    end

    // Monitor state changes
    reg [2:0] prev_state;
    always @(posedge clk) begin
        prev_state <= dut.state;
        if (dut.state != prev_state)
            $display("[%0t] %s  | NS: R=%b Y=%b G=%b | EW: R=%b Y=%b G=%b",
                     $time, state_name,
                     ns_red, ns_yellow, ns_green,
                     ew_red, ew_yellow, ew_green);
    end

    // Test
    integer pass_count;
    integer fail_count;

    initial begin
        $dumpfile("tb_traffic_light_simple.vcd");
        $dumpvars(0, tb_traffic_light_simple);

        pass_count = 0;
        fail_count = 0;

        $display("==========================================");
        $display("  TB: traffic_light_simple");
        $display("  Timing: GREEN=5, YELLOW=2, RED=1 cycles");
        $display("==========================================");

        // Reset
        rst_n = 0;
        repeat(3) @(posedge clk);
        rst_n = 1;
        @(posedge clk);

        // --- Check 1: Start at NS_GREEN ---
        check("Start NS_GREEN", ns_green && ew_red && !ns_red && !ew_green);

        // --- Wait through NS_GREEN (5 cycles) ---
        repeat(5) @(posedge clk);

        // --- Check 2: NS_YELLOW ---
        check("NS_YELLOW", ns_yellow && ew_red && !ns_green && !ns_red);

        // --- Wait through NS_YELLOW (2 cycles) ---
        repeat(2) @(posedge clk);

        // --- Check 3: ALL_RED_1 ---
        check("ALL_RED_1", ns_red && ew_red && !ns_green && !ew_green);

        // --- Wait through ALL_RED_1 (1 cycle) ---
        repeat(1) @(posedge clk);

        // --- Check 4: EW_GREEN ---
        check("EW_GREEN", ew_green && ns_red && !ew_red && !ns_green);

        // --- Wait through EW_GREEN (5 cycles) ---
        repeat(5) @(posedge clk);

        // --- Check 5: EW_YELLOW ---
        check("EW_YELLOW", ew_yellow && ns_red && !ew_green && !ew_red);

        // --- Wait through EW_YELLOW (2 cycles) ---
        repeat(2) @(posedge clk);

        // --- Check 6: ALL_RED_2 ---
        check("ALL_RED_2", ns_red && ew_red && !ns_green && !ew_green);

        // --- Wait through ALL_RED_2 (1 cycle) ---
        repeat(1) @(posedge clk);

        // --- Check 7: Back to NS_GREEN (full cycle!) ---
        check("Cycle back NS_GREEN", ns_green && ew_red);

        // --- Run one more full cycle to confirm ---
        repeat(16) @(posedge clk);  // 5+2+1+5+2+1 = 16
        check("2nd cycle NS_GREEN", ns_green && ew_red);

        // --- Check 8: Reset mid-cycle ---
        repeat(3) @(posedge clk);
        rst_n = 0;
        repeat(2) @(posedge clk);
        rst_n = 1;
        @(posedge clk);
        check("Reset returns to NS_GREEN", ns_green && ew_red);

        // --- Results ---
        repeat(3) @(posedge clk);
        $display("==========================================");
        $display("  PASS: %0d / FAIL: %0d", pass_count, fail_count);
        if (fail_count == 0)
            $display("  ALL TESTS PASSED!");
        else
            $display("  SOME TESTS FAILED!");
        $display("==========================================");
        $finish;
    end

    // Check task
    task check;
        input [30*8-1:0] name;
        input condition;
        begin
            if (condition) begin
                $display("[PASS] %0s", name);
                pass_count = pass_count + 1;
            end else begin
                $display("[FAIL] %0s | NS: R=%b Y=%b G=%b | EW: R=%b Y=%b G=%b",
                         name, ns_red, ns_yellow, ns_green, ew_red, ew_yellow, ew_green);
                fail_count = fail_count + 1;
            end
        end
    endtask

endmodule
