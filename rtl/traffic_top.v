// =============================================================================
// Module     : traffic_top
// Project    : TLC-RFID v4.4
// Description: FPGA top-level module. Instantiates PLL (placeholder),
//              rst_sync, and traffic_node. All ports are physical FPGA pins.
//              PLL generates 50 MHz CLK_SYS from onboard oscillator.
// =============================================================================

module traffic_top (
    input  wire       clk_osc,      // Onboard oscillator input
    input  wire       rst_btn_n,    // Push button reset (active-low, raw async)
    input  wire [1:0] node_id,      // DIP switch [1:0]
    // SPI
    input  wire       spi_miso,
    output wire       spi_sck,
    output wire       spi_mosi,
    output wire [3:0] cs_n,
    // RS485 LEFT
    input  wire       left_rx,
    output wire       left_tx,
    output wire       left_de,
    // RS485 RIGHT
    input  wire       right_rx,
    output wire       right_tx,
    output wire       right_de,
    // LEDs
    output wire [2:0] led_ns,       // {R, Y, G}
    output wire [2:0] led_ew,       // {R, Y, G}
    output wire       led_prio,
    // 7-Segment Display (74HC595)
    output wire       seg7_dio,     // Serial data
    output wire       seg7_sclk,    // Shift clock
    output wire       seg7_rclk     // Latch clock
);

    // =========================================================================
    // PLL — Generate 50 MHz CLK_SYS
    // =========================================================================
    // NOTE: Replace with Gowin PLL IP instantiation from Gowin IDE.
    // Use IP Generator → PLL → configure input freq → output 50 MHz.
    // For now, direct passthrough (assumes oscillator is 50 MHz):
    wire clk_sys;
    wire pll_locked;

    `ifdef SIMULATION
        // In simulation, bypass PLL
        assign clk_sys    = clk_osc;
        assign pll_locked = 1'b1;
    `else
        // ═══ REPLACE THIS WITH GOWIN PLL IP ═══
        // Example (will vary by board):
        // Gowin_PLL u_pll (
        //     .clkin  (clk_osc),
        //     .clkout (clk_sys),
        //     .lock   (pll_locked)
        // );
        assign clk_sys    = clk_osc;    // PLACEHOLDER — replace with PLL!
        assign pll_locked = 1'b1;
    `endif

    // =========================================================================
    // Reset Synchronizer
    // =========================================================================
    wire rst_sync_n;

    rst_sync u_rst (
        .clk         (clk_sys),
        .rst_async_n (rst_btn_n & pll_locked),  // Reset if PLL not locked
        .rst_sync_n  (rst_sync_n)
    );

    // =========================================================================
    // Traffic Node
    // =========================================================================
    wire ns_r, ns_y, ns_g;
    wire [3:0] fsm_cur_state;
    wire ew_r, ew_y, ew_g;
    wire prio_active;

    traffic_node u_node (
        .clk             (clk_sys),
        .rst_n           (rst_sync_n),
        .node_id         (node_id),
        .spi_miso        (spi_miso),
        .spi_sck         (spi_sck),
        .spi_mosi        (spi_mosi),
        .cs_n            (cs_n),
        .left_rx         (left_rx),
        .left_tx         (left_tx),
        .left_de         (left_de),
        .right_rx        (right_rx),
        .right_tx        (right_tx),
        .right_de        (right_de),
        .ns_r            (ns_r),
        .ns_y            (ns_y),
        .ns_g            (ns_g),
        .ew_r            (ew_r),
        .ew_y            (ew_y),
        .ew_g            (ew_g),
        .prio_active_led (prio_active),
        .cur_state_out   (fsm_cur_state)
    );

    // =========================================================================
    // LED Output Mapping
    // =========================================================================
    assign led_ns   = {ns_r, ns_y, ns_g};
    assign led_ew   = {ew_r, ew_y, ew_g};
    assign led_prio = prio_active;

    // =========================================================================
    // 7-Segment Countdown Display
    // =========================================================================
    seg7_countdown u_seg7 (
        .clk        (clk_sys),
        .rst_n      (rst_sync_n),
        .cur_state  (fsm_cur_state),
        .seg7_dio   (seg7_dio),
        .seg7_sclk  (seg7_sclk),
        .seg7_rclk  (seg7_rclk)
    );

endmodule
