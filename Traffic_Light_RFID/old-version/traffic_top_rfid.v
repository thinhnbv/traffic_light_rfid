// =============================================================================
// Module     : traffic_top_rfid
// Project    : Traffic Light Controller with RFID + Green Wave
// Description: Top-level module integrating RFID RC522 reader with
//              the traffic light FSM. Maps S50 Card to NS priority (Straight)
//              and Key Tag to EW priority (Left turn).
// =============================================================================

module traffic_top_rfid (
    input        clk_in,            // Onboard oscillator (50MHz)
    input        rst_btn_n,         // Push button (active low)

    // RC522 SPI pins
    output       rfid_sclk,
    output       rfid_mosi,
    input        rfid_miso,
    output       rfid_cs_n,

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

    // Route Status LEDs
    output       led_priority,      // Preemption active indicator
    output       led_straight_prio, // Latch for Straight priority route
    output       led_left_prio      // Latch for Left-turn priority route
);

    // =========================================================================
    // Clock & Reset Synchronization
    // =========================================================================
    wire clk = clk_in;
    reg [2:0] rst_sync;
    always @(posedge clk) begin
        rst_sync <= {rst_sync[1:0], rst_btn_n};
    end
    wire rst_n = rst_sync[2];

    // =========================================================================
    // RFID Driver Instantiation
    // =========================================================================
    wire        tag_detected;
    wire [31:0] tag_uid;

    rc522_driver u_rc522 (
        .clk          (clk),
        .rst_n        (rst_n),
        .sclk         (rfid_sclk),
        .mosi         (rfid_mosi),
        .miso         (rfid_miso),
        .cs_n         (rfid_cs_n),
        .tag_detected (tag_detected),
        .tag_uid      (tag_uid)
    );

    // =========================================================================
    // Tag Priority Lookup Table Instantiation
    // =========================================================================
    wire       rfid_prio_active;
    wire [2:0] rfid_prio_class;
    wire [1:0] rfid_turn_dir;

    tag_priority_lut #(
        .UID_CARD    (32'hA1B2C3D4), // UID of the S50 Card
        .UID_KEYTAG  (32'hE5F6A7B8)  // UID of the Key Tag
    ) u_lut (
        .clk             (clk),
        .rst_n           (rst_n),
        .tag_detected    (tag_detected),
        .tag_uid         (tag_uid),
        .priority_active (rfid_prio_active),
        .priority_class  (rfid_prio_class),
        .turn_direction  (rfid_turn_dir)
    );

    // =========================================================================
    // Route Mapping Logic
    // - S50 Card (turn_direction = 00) -> triggers NS Priority (Straight)
    // - Key Tag (turn_direction = 01)  -> triggers EW Priority (Left Turn)
    // =========================================================================
    wire ev_detect_ns = rfid_prio_active && (rfid_turn_dir == 2'b00);
    wire ev_detect_ew = rfid_prio_active && (rfid_turn_dir == 2'b01);
    
    wire [2:0] ev_class_ns = ev_detect_ns ? rfid_prio_class : 3'b000;
    wire [2:0] ev_class_ew = ev_detect_ew ? rfid_prio_class : 3'b000;

    assign led_straight_prio = ev_detect_ns;
    assign led_left_prio     = ev_detect_ew;

    // =========================================================================
    // Traffic FSM Instantiation
    // =========================================================================
    wire ns_red, ns_yellow, ns_green;
    wire ew_red, ew_yellow, ew_green;
    wire ped_ns_walk, ped_ew_walk;
    wire priority_active;

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
        .clk             (clk),
        .rst_n           (rst_n),
        .ev_detect_ns    (ev_detect_ns),
        .ev_detect_ew    (ev_detect_ew),
        .ev_class_ns     (ev_class_ns),
        .ev_class_ew     (ev_class_ew),
        .ns_red          (ns_red),
        .ns_yellow       (ns_yellow),
        .ns_green        (ns_green),
        .ew_red          (ew_red),
        .ew_yellow       (ew_yellow),
        .ew_green        (ew_green),
        .ped_ns_walk     (ped_ns_walk),
        .ped_ew_walk     (ped_ew_walk),
        .priority_active (priority_active),
        .priority_dir    (), // Unused at top level
        .current_state   ()  // Unused at top level
    );

    // =========================================================================
    // Output Pin Connections (Active High)
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

endmodule
