`timescale 1ns/1ps

import uvm_pkg::*;
import traffic_uvm_pkg::*;
`include "uvm_macros.svh"

// Top-level UVM testbench for traffic_fsm.
// Instantiates the DUT, virtual interface, clock, and launches the UVM test.
module tb_traffic_top_uvm;

    // Clock
    logic clk = 1'b0;
    always #10 clk = ~clk;  // 50 MHz

    // Virtual interface instance (clock driven from top-level clk)
    traffic_if vif(clk);

    // DUT: traffic_fsm with short SIMULATION timer values (matches tb_traffic_top.v)
    traffic_fsm #(
        .CNT_GREEN_NS (32'd100),
        .CNT_GREEN_EW (32'd80),
        .CNT_YELLOW   (32'd50),
        .CNT_ALL_RED  (32'd20),
        .CNT_PRIO_CLR (32'd50),
        .CNT_PRIO_TMO (32'd600)
    ) u_fsm (
        .clk              (vif.clk),
        .rst_n            (vif.rst_n),
        .ev_detect_ns     (vif.ev_detect_ns),
        .ev_detect_ew     (vif.ev_detect_ew),
        .prio_class       (vif.prio_class),
        .cascade_preclear (vif.cascade_preclear),
        .ns_r             (vif.ns_r),
        .ns_y             (vif.ns_y),
        .ns_g             (vif.ns_g),
        .ew_r             (vif.ew_r),
        .ew_y             (vif.ew_y),
        .ew_g             (vif.ew_g),
        .prio_active      (vif.prio_active),
        .cur_state        (vif.cur_state),
        .timer_load       (vif.timer_load),
        .timer_value      (vif.timer_value)
    );

    // Launch UVM test
    initial begin
        uvm_config_db#(virtual traffic_if)::set(null, "*", "vif", vif);
        run_test("traffic_test");
    end

    // VCD dump for waveform debug
    initial begin
        $dumpfile("tb_traffic_top_uvm.vcd");
        $dumpvars(0, tb_traffic_top_uvm);
    end

    // Watchdog (long enough for 200 cycles @ 20ns = ~4us + drain; failsafe at 20ms)
    initial begin
        #(20 * 1000000);
        $display("\n[WATCHDOG] Simulation exceeded max time — stopping.");
        $finish;
    end

endmodule
