`timescale 1ns/1ps

// Virtual interface connecting UVM testbench to the traffic_fsm DUT.
// Mirrors the traffic_fsm port list exactly (spec Sec 9.2).
interface traffic_if(input logic clk);
    logic        rst_n;
    logic        ev_detect_ns;
    logic        ev_detect_ew;
    logic [2:0]  prio_class;
    logic        cascade_preclear;
    logic        ns_r, ns_y, ns_g;
    logic        ew_r, ew_y, ew_g;
    logic        prio_active;
    logic [3:0]  cur_state;
    logic        timer_load;
    logic [31:0] timer_value;
endinterface
