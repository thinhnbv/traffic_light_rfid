`timescale 1ns/1ps

package traffic_uvm_pkg;

    import uvm_pkg::*;
    `include "uvm_macros.svh"

    // ============================================================================
    // Single UVM package for traffic_fsm verification.
    // Compiled as one unit so all cross-class references resolve in ModelSim.
    // ============================================================================

    // ------------------------------------------------------------
    // Sequence item
    // ------------------------------------------------------------
    class traffic_sequence_item extends uvm_sequence_item;

        rand bit        ev_detect_ns;
        rand bit        ev_detect_ew;
        rand bit [2:0]  prio_class;
        rand bit        cascade_preclear;
        rand bit        do_reset;

        bit        rst_n;
        bit [2:0]  expect_state;
        bit        timer_load;
        bit [31:0] timer_value;

        constraint c_normal_ops {
            ev_detect_ns dist {1:/30, 0:/70};
            ev_detect_ew dist {1:/30, 0:/70};
            prio_class   dist {0:/40, 1:/10, 2:/10, 3:/15, 4:/15, 5:/10};
            cascade_preclear dist {1:/10, 0:/90};
            do_reset     dist {1:/5,  0:/95};
        }

        constraint c_no_same_axis_conflict {
            !(ev_detect_ns && ev_detect_ew);
        }

        `uvm_object_utils_begin(traffic_sequence_item)
            `uvm_field_int(ev_detect_ns,     UVM_ALL_ON)
            `uvm_field_int(ev_detect_ew,     UVM_ALL_ON)
            `uvm_field_int(prio_class,       UVM_ALL_ON)
            `uvm_field_int(cascade_preclear, UVM_ALL_ON)
            `uvm_field_int(do_reset,         UVM_ALL_ON)
            `uvm_field_int(rst_n,            UVM_ALL_ON)
            `uvm_field_int(expect_state,     UVM_ALL_ON)
            `uvm_field_int(timer_load,       UVM_ALL_ON)
            `uvm_field_int(timer_value,      UVM_ALL_ON)
        `uvm_object_utils_end

        function new(string name = "traffic_sequence_item");
            super.new(name);
        endfunction

    endclass

    // ------------------------------------------------------------
    // Driver
    // ------------------------------------------------------------
    class traffic_driver extends uvm_driver #(traffic_sequence_item);

        virtual traffic_if vif;

        `uvm_component_utils(traffic_driver)

        function new(string name = "traffic_driver", uvm_component parent = null);
            super.new(name, parent);
        endfunction

        virtual function void build_phase(uvm_phase phase);
            super.build_phase(phase);
            if (!uvm_config_db#(virtual traffic_if)::get(this, "", "vif", vif))
                `uvm_fatal("NOVIF", "virtual traffic_if not set")
        endfunction

        virtual task run_phase(uvm_phase phase);
            `uvm_info("DRV", "driver run_phase started", UVM_MEDIUM)
            forever begin
                `uvm_info("DRV", "waiting get_next_item", UVM_MEDIUM)
                seq_item_port.get_next_item(req);
                `uvm_info("DRV", "got item, driving", UVM_MEDIUM)
                drive_item(req);
                `uvm_info("DRV", "driven, item_done", UVM_MEDIUM)
                seq_item_port.item_done();
            end
        endtask

        virtual task drive_item(traffic_sequence_item item);
            vif.ev_detect_ns     = item.ev_detect_ns;
            vif.ev_detect_ew     = item.ev_detect_ew;
            vif.prio_class       = item.prio_class;
            vif.cascade_preclear = item.cascade_preclear;

            if (item.do_reset) begin
                vif.rst_n <= 1'b0;
                @(posedge vif.clk);
                vif.rst_n <= 1'b1;
            end else begin
                vif.rst_n <= 1'b1;
                @(posedge vif.clk);
            end
        endtask

    endclass

    // ------------------------------------------------------------
    // Monitor
    // ------------------------------------------------------------
    class traffic_monitor extends uvm_monitor;

        virtual traffic_if vif;
        uvm_analysis_port #(traffic_sequence_item) ap;

        `uvm_component_utils(traffic_monitor)

        function new(string name = "traffic_monitor", uvm_component parent = null);
            super.new(name, parent);
            ap = new("ap", this);
        endfunction

        virtual function void build_phase(uvm_phase phase);
            super.build_phase(phase);
            if (!uvm_config_db#(virtual traffic_if)::get(this, "", "vif", vif))
                `uvm_fatal("NOVIF", "virtual traffic_if not set")
        endfunction

        virtual task run_phase(uvm_phase phase);
            traffic_sequence_item item;
            forever begin
                @(posedge vif.clk);
                item = traffic_sequence_item::type_id::create("mon_item");
                item.ev_detect_ns     = vif.ev_detect_ns;
                item.ev_detect_ew     = vif.ev_detect_ew;
                item.prio_class       = vif.prio_class;
                item.cascade_preclear = vif.cascade_preclear;
                item.rst_n            = vif.rst_n;
                item.expect_state     = vif.cur_state;
                item.timer_load       = vif.timer_load;
                item.timer_value      = vif.timer_value;
                ap.write(item);
            end
        endtask

    endclass

    // ------------------------------------------------------------
    // Scoreboard with reference model + coverage
    // ------------------------------------------------------------
    class traffic_scoreboard extends uvm_scoreboard;

        uvm_analysis_imp #(traffic_sequence_item, traffic_scoreboard) imp;

        logic [3:0] item_state;
        logic [2:0] item_winner;

        covergroup fsm_cov;
            state_cp: coverpoint item_state {
                bins S_NS_GREEN   = {4'd0};
                bins S_NS_YELLOW  = {4'd1};
                bins S_ALL_RED_1  = {4'd2};
                bins S_EW_GREEN   = {4'd3};
                bins S_EW_YELLOW  = {4'd4};
                bins S_ALL_RED_2  = {4'd5};
                bins S_PRIO_CLEAR = {4'd6};
                bins S_PRIO_HOLD  = {4'd7};
                bins S_PRIO_EXIT  = {4'd8};
                bins S_PRIO_RECOV = {4'd9};
            }
        endgroup
        covergroup prio_cov;
            winner_cp: coverpoint item_winner {
                bins C0 = {3'd0};
                bins C1 = {3'd1};
                bins C2 = {3'd2};
                bins C3 = {3'd3};
                bins C4 = {3'd4};
                bins C5 = {3'd5};
                bins C67 = {3'd6, 3'd7};
            }
        endgroup

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

        logic [3:0] state;
        logic [3:0] next_state;
        logic [3:0] prev_state;      // for state-entry detection (timer reload)
        logic [31:0] model_counter;  // phase_timer counter model

        // SIMULATION timer values (match traffic_fsm parameters)
        localparam [31:0] CNT_GREEN_NS  = 32'd100;
        localparam [31:0] CNT_GREEN_EW  = 32'd80;
        localparam [31:0] CNT_YELLOW    = 32'd50;
        localparam [31:0] CNT_ALL_RED   = 32'd20;
        localparam [31:0] CNT_PRIO_CLR  = 32'd50;
        localparam [31:0] CNT_PRIO_TMO  = 32'd600;

        logic        ev_detect_ns, ev_detect_ew, cascade_preclear, rst_n;
        logic        timer_load;       // sampled from DUT interface
        logic [31:0] timer_value;      // sampled from DUT interface
        logic [2:0]  prio_class;

        int errors;
        int checks;
        int warmup;          // skip first few startup samples until model/DUT in sync

        `uvm_component_utils(traffic_scoreboard)

        function new(string name = "traffic_scoreboard", uvm_component parent = null);
            super.new(name, parent);
            imp = new("imp", this);
            fsm_cov = new();
            prio_cov = new();
            state = S_NS_GREEN;
            next_state = S_NS_GREEN;
            prev_state = S_NS_GREEN;
            model_counter = CNT_GREEN_NS;   // match phase_timer's initial load (CNT_GREEN_NS)
            warmup = 3;                     // startup transients (t=0/30/50) not real mismatches
            errors = 0;
            checks = 0;
        endfunction

        virtual function void write(traffic_sequence_item item);
            bit expired_this_cycle;
            item_state    = item.expect_state;
            item_winner   = item.prio_class;
            ev_detect_ns  = item.ev_detect_ns;
            ev_detect_ew  = item.ev_detect_ew;
            prio_class    = item.prio_class;
            cascade_preclear = item.cascade_preclear;
            rst_n         = item.rst_n;
            timer_load    = item.timer_load;
            timer_value   = item.timer_value;

            fsm_cov.sample();
            prio_cov.sample();

            // DUT's phase_timer evaluates expired = (counter==0) && !load with
            // the counter observed BEFORE this cycle's non-blocking update, so
            // check expiry pre-update to mirror the hardware exactly (avoids a
            // one-cycle-expiry skew vs the DUT).
            expired_this_cycle = (model_counter == 32'd0) && !timer_load;

            if (!rst_n) begin
                state = S_NS_GREEN;
                next_state = S_NS_GREEN;
                prev_state = S_NS_GREEN;
                model_counter = 32'd0;
                return;
            end

            // Check: previously-predicted next_state vs DUT cur_state sampled
            // at the same edge. Pipeline skew: driver applies inputs at
            // posedge clk, monitor samples cur_state at the same posedge, so
            // cur_state still reflects previous inputs; next_state holds the
            // prediction for THIS edge (computed from last cycle).
            // Skip first few startup samples until model/DUT are in sync.
            if (warmup > 0)
                warmup--;
            else begin
                checks++;
                if (next_state !== item_state) begin
                    `uvm_error("STATE_MISMATCH",
                        $sformatf("PRED next_state=%0d vs DUT cur_state=%0d  (ev_ns=%0b ev_ew=%0b cls=%0d)",
                                  next_state, item_state, ev_detect_ns, ev_detect_ew, prio_class))
                    errors++;
                end else begin
                    `uvm_info("STATE_OK",
                        $sformatf("state=%0d  (ev_ns=%0b ev_ew=%0b cls=%0d)", item_state,
                                  ev_detect_ns, ev_detect_ew, prio_class), UVM_MEDIUM)
                end
            end

            prev_state = state;
            state = item_state;
            compute_next_state(expired_this_cycle);

            // Track model phase_timer counter (mirrors phase_timer.v):
            //   !rst_n -> 0 ; load -> timer_value ; counter!=0 -> counter-1 ; else 0
            if (timer_load)
                model_counter = timer_value;
            else if (model_counter != 0)
                model_counter = model_counter - 1;
        endfunction

        virtual function void compute_next_state(bit expired);
            logic ev_any = ev_detect_ns || ev_detect_ew;
            next_state = state;

            case (state)
                S_NS_GREEN: begin
                    if (ev_detect_ew && !ev_detect_ns)
                        next_state = S_NS_YELLOW;
                    else if (ev_detect_ns)
                        next_state = S_PRIO_CLEAR;
                    else if (cascade_preclear)
                        next_state = S_NS_YELLOW;
                end
                S_NS_YELLOW: if (expired) next_state = S_ALL_RED_1;
                S_ALL_RED_1: if (expired)
                    next_state = (ev_any) ? S_PRIO_CLEAR : S_EW_GREEN;
                S_EW_GREEN: begin
                    if (ev_detect_ns && !ev_detect_ew)
                        next_state = S_EW_YELLOW;
                    else if (ev_detect_ew)
                        next_state = S_PRIO_CLEAR;
                    else if (cascade_preclear)
                        next_state = S_EW_YELLOW;
                    else if (expired)
                        next_state = S_EW_YELLOW;
                end
                S_EW_YELLOW: if (expired) next_state = S_ALL_RED_2;
                S_ALL_RED_2: if (expired)
                    next_state = (ev_any) ? S_PRIO_CLEAR : S_NS_GREEN;
                S_PRIO_CLEAR: if (expired) next_state = S_PRIO_HOLD;
                S_PRIO_HOLD: begin
                    if (expired)
                        next_state = S_PRIO_EXIT;
                    else if (!ev_detect_ns)
                        next_state = S_PRIO_EXIT;
                    else if (!ev_detect_ew)
                        next_state = S_PRIO_EXIT;
                end
                S_PRIO_EXIT: if (expired) next_state = S_PRIO_RECOV;
                S_PRIO_RECOV: if (expired) next_state = S_NS_GREEN;
                default: next_state = S_NS_GREEN;
            endcase
        endfunction

        virtual function void report_phase(uvm_phase phase);
            super.report_phase(phase);
            `uvm_info("SCOREBOARD",
                $sformatf("Checks=%0d  Errors=%0d", checks, errors), UVM_LOW)
        endfunction

    endclass

    // ------------------------------------------------------------
    // Agent
    // ------------------------------------------------------------
    class traffic_agent extends uvm_agent;

        traffic_driver                      drv;
        traffic_monitor                     mon;
        uvm_sequencer #(traffic_sequence_item) seqr;

        `uvm_component_utils(traffic_agent)

        function new(string name = "traffic_agent", uvm_component parent = null);
            super.new(name, parent);
        endfunction

        virtual function void build_phase(uvm_phase phase);
            super.build_phase(phase);
            drv = traffic_driver::type_id::create("drv", this);
            mon = traffic_monitor::type_id::create("mon", this);
            seqr = uvm_sequencer #(traffic_sequence_item)::type_id::create("seqr", this);
        endfunction

        virtual function void connect_phase(uvm_phase phase);
            super.connect_phase(phase);
            drv.seq_item_port.connect(seqr.seq_item_export);
        endfunction

    endclass

    // ------------------------------------------------------------
    // Environment
    // ------------------------------------------------------------
    class traffic_env extends uvm_env;

        traffic_agent   agt;
        traffic_scoreboard scb;

        `uvm_component_utils(traffic_env)

        function new(string name = "traffic_env", uvm_component parent = null);
            super.new(name, parent);
        endfunction

        virtual function void build_phase(uvm_phase phase);
            super.build_phase(phase);
            agt = traffic_agent::type_id::create("agt", this);
            scb = traffic_scoreboard::type_id::create("scb", this);
        endfunction

        virtual function void connect_phase(uvm_phase phase);
            super.connect_phase(phase);
            agt.mon.ap.connect(scb.imp);
        endfunction

    endclass

    // ------------------------------------------------------------
    // Sequence (constrained-random stimulus, 200 cycles)
    // ------------------------------------------------------------
    class traffic_sequence extends uvm_sequence #(traffic_sequence_item);

        `uvm_object_utils(traffic_sequence)

        function new(string name = "traffic_sequence");
            super.new(name);
        endfunction

        virtual task body();
            `uvm_info("SEQ", "sequence body starting (200 random items)", UVM_MEDIUM)
            repeat (200) begin
                traffic_sequence_item item = traffic_sequence_item::type_id::create("item");
                start_item(item);
                assert(item.randomize());
                finish_item(item);
            end
            `uvm_info("SEQ", "sequence body done (200 items sent)", UVM_MEDIUM)
        endtask

    endclass

    // ------------------------------------------------------------
    // Test
    // ------------------------------------------------------------
    class traffic_test extends uvm_test;

        traffic_env env;

        `uvm_component_utils(traffic_test)

        function new(string name = "traffic_test", uvm_component parent = null);
            super.new(name, parent);
        endfunction

        virtual function void build_phase(uvm_phase phase);
            super.build_phase(phase);
            env = traffic_env::type_id::create("env", this);
        endfunction

        virtual task run_phase(uvm_phase phase);
            traffic_sequence seq;
            phase.raise_objection(this);
            seq = traffic_sequence::type_id::create("seq");
            seq.start(env.agt.seqr);
            #(200);
            phase.drop_objection(this);
        endtask

    endclass

endpackage
