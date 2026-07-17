// =============================================================================
// Testbench : tb_rc522_sim
// Project   : Traffic Light Controller with RFID + Green Wave
// Description: Testbench with a mock SPI Slave simulating MFRC522.
//              Tests S50 Card read (Straight route preemption)
//              and Key Tag read (Left Turn route preemption).
// =============================================================================

`timescale 1ns/1ps

module tb_rc522_sim;

    // Inputs
    reg clk;
    reg rst_btn_n;
    reg rfid_miso;

    // Outputs
    wire rfid_sclk;
    wire rfid_mosi;
    wire rfid_cs_n;

    wire led_ns_red;
    wire led_ns_yellow;
    wire led_ns_green;
    wire led_ew_red;
    wire led_ew_yellow;
    wire led_ew_green;
    wire led_ped_ns;
    wire led_ped_ew;
    wire led_priority;
    wire led_straight_prio;
    wire led_left_prio;

    // Instantiate UUT
    // Override SIM_MODE = 1 in FSM via hierarchy parameter override
    traffic_top_rfid uut (
        .clk_in            (clk),
        .rst_btn_n         (rst_btn_n),
        .rfid_sclk         (rfid_sclk),
        .rfid_mosi         (rfid_mosi),
        .rfid_miso         (rfid_miso),
        .rfid_cs_n         (rfid_cs_n),
        .led_ns_red        (led_ns_red),
        .led_ns_yellow     (led_ns_yellow),
        .led_ns_green      (led_ns_green),
        .led_ew_red        (led_ew_red),
        .led_ew_yellow     (led_ew_yellow),
        .led_ew_green      (led_ew_green),
        .led_ped_ns        (led_ped_ns),
        .led_ped_ew        (led_ped_ew),
        .led_priority      (led_priority),
        .led_straight_prio (led_straight_prio),
        .led_left_prio     (led_left_prio)
    );

    // Override SIM_MODE for fast FSM simulation
    defparam uut.u_fsm.SIM_MODE = 1;

    // =========================================================================
    // Clock Generator (50MHz -> 20ns period)
    // =========================================================================
    always #10 clk = ~clk;

    // =========================================================================
    // Mock MFRC522 SPI Slave Behavioral Model
    // =========================================================================
    reg [7:0]  shift_in;
    reg [7:0]  shift_out;
    reg [3:0]  bit_cnt;
    reg        is_read;
    reg [5:0]  cmd_addr;
    reg        byte_index; // 0 = address byte, 1 = data byte

    // Mock RC522 internal registers/FIFO
    reg [7:0]  reg_command;
    reg [7:0]  reg_bit_framing;
    reg [7:0]  mock_fifo [7:0];
    reg [3:0]  mock_fifo_wp;
    reg [3:0]  mock_fifo_rp;
    reg [7:0]  mock_fifo_level;

    // Test stimulus variables
    reg [31:0] active_mock_uid;
    reg        tag_in_rf_field;

    // SPI Slave logic
    always @(posedge rfid_sclk or posedge rfid_cs_n) begin
        if (rfid_cs_n) begin
            bit_cnt    <= 4'd0;
            byte_index <= 1'b0;
            shift_in   <= 8'h00;
            rfid_miso  <= 1'bz;
        end else begin
            shift_in <= {shift_in[6:0], rfid_mosi};
            bit_cnt  <= bit_cnt + 1'b1;

            if (bit_cnt == 4'd7) begin
                bit_cnt    <= 4'd0;
                byte_index <= ~byte_index;
                
                if (byte_index == 1'b0) begin
                    // Address byte completed
                    is_read  <= shift_in[7];
                    cmd_addr <= shift_in[6:1];
                    
                    if (shift_in[7]) begin // Read request
                        case (shift_in[6:1])
                            6'h0A: shift_out <= mock_fifo_level;
                            6'h09: begin
                                shift_out    <= mock_fifo[mock_fifo_rp];
                                mock_fifo_rp <= mock_fifo_rp + 1'b1;
                                if (mock_fifo_level > 0)
                                    mock_fifo_level <= mock_fifo_level - 1'b1;
                            end
                            default: shift_out <= 8'h00;
                        endcase
                    end
                end else begin
                    // Data byte completed
                    if (!is_read) begin // Write request
                        case (cmd_addr)
                            6'h01: begin
                                reg_command <= shift_in;
                                if (shift_in == 8'h0C) begin
                                    // TRANSCEIVE command triggered!
                                    // Simulate transceive response if card is in field
                                    if (tag_in_rf_field) begin
                                        if (reg_bit_framing == 8'h87) begin
                                            // REQA sent (7 bits). Load 2-byte ATQA (0x04, 0x00)
                                            mock_fifo[0]    <= 8'h04;
                                            mock_fifo[1]    <= 8'h00;
                                            mock_fifo_level <= 8'd2;
                                            mock_fifo_rp    <= 4'd0;
                                            mock_fifo_wp    <= 4'd2;
                                        end else if (reg_bit_framing == 8'h80) begin
                                            // ANTICOLLISION SELECT sent. Load 4-byte UID + BCC check byte
                                            mock_fifo[0]    <= active_mock_uid[31:24];
                                            mock_fifo[1]    <= active_mock_uid[23:16];
                                            mock_fifo[2]    <= active_mock_uid[15:8];
                                            mock_fifo[3]    <= active_mock_uid[7:0];
                                            mock_fifo[4]    <= active_mock_uid[31:24] ^ active_mock_uid[23:16] ^ 
                                                               active_mock_uid[15:8]  ^ active_mock_uid[7:0]; // BCC
                                            mock_fifo_level <= 8'd5;
                                            mock_fifo_rp    <= 4'd0;
                                            mock_fifo_wp    <= 4'd5;
                                        end
                                    end
                                end
                            end
                            6'h0A: begin
                                if (shift_in[7]) begin // FIFO Clear
                                    mock_fifo_level <= 8'd0;
                                    mock_fifo_rp    <= 4'd0;
                                    mock_fifo_wp    <= 4'd0;
                                end
                            end
                            6'h0D: reg_bit_framing <= shift_in;
                            default: ;
                        endcase
                    end
                end
            end
        end
    end

    // MISO Drive on falling edge (Mode 0, CPHA=0 output change)
    always @(negedge rfid_sclk or posedge rfid_cs_n) begin
        if (rfid_cs_n) begin
            rfid_miso <= 1'bz;
        end else begin
            if (byte_index == 1'b0 && bit_cnt == 4'd0) begin
                // Drive first bit of read transaction
                if (is_read) begin
                    rfid_miso <= shift_out[7];
                    shift_out <= {shift_out[6:0], 1'b0};
                end else begin
                    rfid_miso <= 1'b0;
                end
            end else if (is_read) begin
                rfid_miso <= shift_out[7];
                shift_out <= {shift_out[6:0], 1'b0};
            end else begin
                rfid_miso <= 1'b0;
            end
        end
    end

    // =========================================================================
    // Test Scenario Stimulus
    // =========================================================================
    initial begin
        // Initialize signals
        clk             = 1'b0;
        rst_btn_n       = 1'b0;
        rfid_miso       = 1'bz;
        tag_in_rf_field = 1'b0;
        active_mock_uid = 32'h00000000;

        // Reset system
        #100;
        rst_btn_n = 1'b1;
        #100;

        $display("[TB] Reset complete. Running normal traffic cycle...");
        #200000; // Let the normal cycle run for a bit (SIM_MODE=1 makes it fast)

        // ---------------------------------------------------------------------
        // SCENARIO 1: Present S50 Card (Straight Priority -> NS)
        // ---------------------------------------------------------------------
        $display("[TB] Scenario 1: Present S50 Card (UID: 0xA1B2C3D4)...");
        active_mock_uid = 32'hA1B2C3D4; // S50 Card UID
        tag_in_rf_field = 1'b1;

        // Wait to detect and preempt
        @(posedge led_straight_prio);
        $display("[TB] SUCCESS: S50 Card detected! Straight priority route active.");
        #100000;

        // Verify NS green is held
        if (led_ns_green === 1'b1 && led_ew_red === 1'b1) begin
            $display("[TB] SUCCESS: NS axis green light held for priority vehicle.");
        end else begin
            $display("[TB] ERROR: NS green was not correctly held.");
        end

        #200000;
        // Remove S50 Card
        $display("[TB] Scenario 1: Remove S50 Card...");
        tag_in_rf_field = 1'b0;
        
        // Wait for preemption to release
        @(negedge led_priority);
        $display("[TB] SUCCESS: Priority released, system returning to normal.");
        #300000;

        // ---------------------------------------------------------------------
        // SCENARIO 2: Present Key Tag (Left Turn Priority -> EW)
        // ---------------------------------------------------------------------
        $display("[TB] Scenario 2: Present Key Tag (UID: 0xE5F6A7B8)...");
        active_mock_uid = 32'hE5F6A7B8; // Key Tag UID
        tag_in_rf_field = 1'b1;

        // Wait to detect and preempt
        @(posedge led_left_prio);
        $display("[TB] SUCCESS: Key Tag detected! Left-turn priority route active.");
        #100000;

        // Verify EW green is held
        if (led_ew_green === 1'b1 && led_ns_red === 1'b1) begin
            $display("[TB] SUCCESS: EW axis green light held for left-turning priority vehicle.");
        end else begin
            $display("[TB] ERROR: EW green was not correctly held.");
        end

        #200000;
        // Remove Key Tag
        $display("[TB] Scenario 2: Remove Key Tag...");
        tag_in_rf_field = 1'b0;

        // Wait for preemption to release
        @(negedge led_priority);
        $display("[TB] SUCCESS: Priority released, system returning to normal.");
        #300000;

        $display("[TB] Simulation completed successfully!");
        $finish;
    end

endmodule
