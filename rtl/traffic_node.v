// =============================================================================
// Module     : traffic_node
// Project    : TLC-RFID v4.4
// Description: Top-level wrapper for 1 intersection. Instantiates all
//              sub-modules and wires them together. Structural only —
//              no internal logic. All signals are in CLK_SYS domain.
// =============================================================================

module traffic_node (
    input  wire       clk,
    input  wire       rst_n,
    input  wire [1:0] node_id,
    // SPI to RC522
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
    output wire       ns_r,
    output wire       ns_y,
    output wire       ns_g,
    output wire       ew_r,
    output wire       ew_y,
    output wire       ew_g,
    output wire       prio_active_led,
    output wire [3:0] cur_state_out
);

    // =========================================================================
    // Internal Wires
    // =========================================================================

    // ─── RFID Driver → LUT / Arbiter / Direction ───
    wire [3:0]  rfid_tag_detected;
    wire [31:0] rfid_uid_n, rfid_uid_s, rfid_uid_e, rfid_uid_w;

    // ─── Tag Priority LUT outputs ───
    wire [2:0] lut_class_n, lut_class_s, lut_class_e, lut_class_w;

    // ─── Priority Arbiter outputs ───
    wire       arb_ev_detect_ns, arb_ev_detect_ew;
    wire [2:0] arb_winner_class;

    // ─── Direction Detector outputs ───
    wire [1:0] dir_out;
    wire       dir_valid;
    wire       dir_confidence;

    // ─── Traffic FSM outputs ───
    wire       fsm_prio_active;
    wire [3:0] fsm_cur_state;
    wire       fsm_timer_load;
    wire [31:0] fsm_timer_value;

    // ─── Cascade Control outputs ───
    wire       cascade_preclear;

    // LEFT cascade chain wires
    wire       left_uart_tx_serial, left_uart_tx_busy, left_uart_tx_done;
    wire [7:0] left_uart_tx_data;
    wire       left_uart_tx_start;
    wire [7:0] left_uart_rx_data;
    wire       left_uart_rx_valid, left_uart_rx_error;
    wire       left_cas_tx_done;
    wire       left_pkt_valid, left_pkt_flag_valid, left_pkt_flag_dir;
    wire       left_pkt_flag_clear, left_pkt_error;
    wire [4:0] left_pkt_src_id;
    wire [2:0] left_pkt_class;
    wire [7:0] left_pkt_eta;
    wire       left_rs485_de, left_rs485_tx_enable;

    // RIGHT cascade chain wires
    wire       right_uart_tx_serial, right_uart_tx_busy, right_uart_tx_done;
    wire [7:0] right_uart_tx_data;
    wire       right_uart_tx_start;
    wire [7:0] right_uart_rx_data;
    wire       right_uart_rx_valid, right_uart_rx_error;
    wire       right_cas_tx_done;
    wire       right_pkt_valid, right_pkt_flag_valid, right_pkt_flag_dir;
    wire       right_pkt_flag_clear, right_pkt_error;
    wire [4:0] right_pkt_src_id;
    wire [2:0] right_pkt_class;
    wire [7:0] right_pkt_eta;
    wire       right_rs485_de, right_rs485_tx_enable;

    // Cascade ctrl → cascade_tx wires
    wire       ctrl_left_tx_send, ctrl_left_tx_valid, ctrl_left_tx_dir, ctrl_left_tx_clear;
    wire [2:0] ctrl_left_tx_class;
    wire [7:0] ctrl_left_tx_eta;
    wire       ctrl_right_tx_send, ctrl_right_tx_valid, ctrl_right_tx_dir, ctrl_right_tx_clear;
    wire [2:0] ctrl_right_tx_class;
    wire [7:0] ctrl_right_tx_eta;

    // any_detected for direction_detector
    wire any_detected = |rfid_tag_detected;

    // =========================================================================
    // Module Instances
    // =========================================================================

    // ─── RC522 RFID Driver ───
    rc522_driver_x4 u_rfid (
        .clk          (clk),
        .rst_n        (rst_n),
        .spi_miso     (spi_miso),
        .spi_sck      (spi_sck),
        .spi_mosi     (spi_mosi),
        .cs_n         (cs_n),
        .tag_detected (rfid_tag_detected),
        .tag_uid_n    (rfid_uid_n),
        .tag_uid_s    (rfid_uid_s),
        .tag_uid_e    (rfid_uid_e),
        .tag_uid_w    (rfid_uid_w)
    );

    // ─── Tag Priority LUTs (4 instances) ───
    tag_priority_lut u_lut_n (.uid(rfid_uid_n), .priority_class(lut_class_n));
    tag_priority_lut u_lut_s (.uid(rfid_uid_s), .priority_class(lut_class_s));
    tag_priority_lut u_lut_e (.uid(rfid_uid_e), .priority_class(lut_class_e));
    tag_priority_lut u_lut_w (.uid(rfid_uid_w), .priority_class(lut_class_w));

    // Register buffering to break the critical path
    reg [2:0] reg_class_n, reg_class_s, reg_class_e, reg_class_w;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            reg_class_n <= 3'd0;
            reg_class_s <= 3'd0;
            reg_class_e <= 3'd0;
            reg_class_w <= 3'd0;
        end else begin
            reg_class_n <= lut_class_n;
            reg_class_s <= lut_class_s;
            reg_class_e <= lut_class_e;
            reg_class_w <= lut_class_w;
        end
    end

    // ─── Priority Arbiter ───
    priority_arbiter u_arb (
        .tag_detected (rfid_tag_detected),
        .class_n      (reg_class_n),
        .class_s      (reg_class_s),
        .class_e      (reg_class_e),
        .class_w      (reg_class_w),
        .cur_state    (fsm_cur_state),
        .ev_detect_ns (arb_ev_detect_ns),
        .ev_detect_ew (arb_ev_detect_ew),
        .winner_class (arb_winner_class)
    );

    // ─── Direction Detector ───
    direction_detector u_dir (
        .clk            (clk),
        .rst_n          (rst_n),
        .tag_detected   (rfid_tag_detected),
        .any_detected   (any_detected),
        .dir_out        (dir_out),
        .dir_valid      (dir_valid),
        .dir_confidence (dir_confidence)
    );

    // ─── Traffic FSM ───
    traffic_fsm u_fsm (
        .clk              (clk),
        .rst_n            (rst_n),
        .ev_detect_ns     (arb_ev_detect_ns),
        .ev_detect_ew     (arb_ev_detect_ew),
        .prio_class       (arb_winner_class),
        .cascade_preclear (cascade_preclear),
        .ns_r             (ns_r),
        .ns_y             (ns_y),
        .ns_g             (ns_g),
        .ew_r             (ew_r),
        .ew_y             (ew_y),
        .ew_g             (ew_g),
        .prio_active      (fsm_prio_active),
        .cur_state        (fsm_cur_state),
        .timer_load       (fsm_timer_load),
        .timer_value      (fsm_timer_value)
    );

    assign prio_active_led = fsm_prio_active;
    assign cur_state_out   = fsm_cur_state;

    // =========================================================================
    // LEFT Cascade Chain
    // =========================================================================
    uart_rx_core u_left_uart_rx (
        .clk       (clk),
        .rst_n     (rst_n),
        .rx_serial (left_rx),
        .rx_data   (left_uart_rx_data),
        .rx_valid  (left_uart_rx_valid),
        .rx_error  (left_uart_rx_error)
    );

    cascade_rx u_left_cas_rx (
        .clk            (clk),
        .rst_n          (rst_n),
        .uart_rx_data   (left_uart_rx_data),
        .uart_rx_valid  (left_uart_rx_valid),
        .pkt_valid      (left_pkt_valid),
        .pkt_flag_valid (left_pkt_flag_valid),
        .pkt_flag_dir   (left_pkt_flag_dir),
        .pkt_flag_clear (left_pkt_flag_clear),
        .pkt_src_id     (left_pkt_src_id),
        .pkt_class      (left_pkt_class),
        .pkt_eta        (left_pkt_eta),
        .pkt_error      (left_pkt_error)
    );

    uart_tx_core u_left_uart_tx (
        .clk       (clk),
        .rst_n     (rst_n),
        .tx_data   (left_uart_tx_data),
        .tx_start  (left_uart_tx_start),
        .tx_serial (left_uart_tx_serial),
        .tx_busy   (left_uart_tx_busy),
        .tx_done   (left_uart_tx_done)
    );

    cascade_tx u_left_cas_tx (
        .clk           (clk),
        .rst_n         (rst_n),
        .send          (ctrl_left_tx_send),
        .valid         (ctrl_left_tx_valid),
        .direction     (ctrl_left_tx_dir),
        .clear         (ctrl_left_tx_clear),
        .src_node_id   ({3'b000, node_id}),
        .prio_class    (ctrl_left_tx_class),
        .eta           (ctrl_left_tx_eta),
        .uart_tx_busy  (left_uart_tx_busy),
        .uart_tx_data  (left_uart_tx_data),
        .uart_tx_start (left_uart_tx_start),
        .tx_done       (left_cas_tx_done)
    );

    rs485_dir_ctrl u_left_rs485 (
        .clk       (clk),
        .rst_n     (rst_n),
        .tx_start  (ctrl_left_tx_send),
        .tx_done   (left_cas_tx_done),
        .rs485_de  (left_rs485_de),
        .tx_enable (left_rs485_tx_enable)
    );

    assign left_tx = left_uart_tx_serial;
    assign left_de = left_rs485_de;

    // =========================================================================
    // RIGHT Cascade Chain
    // =========================================================================
    uart_rx_core u_right_uart_rx (
        .clk       (clk),
        .rst_n     (rst_n),
        .rx_serial (right_rx),
        .rx_data   (right_uart_rx_data),
        .rx_valid  (right_uart_rx_valid),
        .rx_error  (right_uart_rx_error)
    );

    cascade_rx u_right_cas_rx (
        .clk            (clk),
        .rst_n          (rst_n),
        .uart_rx_data   (right_uart_rx_data),
        .uart_rx_valid  (right_uart_rx_valid),
        .pkt_valid      (right_pkt_valid),
        .pkt_flag_valid (right_pkt_flag_valid),
        .pkt_flag_dir   (right_pkt_flag_dir),
        .pkt_flag_clear (right_pkt_flag_clear),
        .pkt_src_id     (right_pkt_src_id),
        .pkt_class      (right_pkt_class),
        .pkt_eta        (right_pkt_eta),
        .pkt_error      (right_pkt_error)
    );

    uart_tx_core u_right_uart_tx (
        .clk       (clk),
        .rst_n     (rst_n),
        .tx_data   (right_uart_tx_data),
        .tx_start  (right_uart_tx_start),
        .tx_serial (right_uart_tx_serial),
        .tx_busy   (right_uart_tx_busy),
        .tx_done   (right_uart_tx_done)
    );

    cascade_tx u_right_cas_tx (
        .clk           (clk),
        .rst_n         (rst_n),
        .send          (ctrl_right_tx_send),
        .valid         (ctrl_right_tx_valid),
        .direction     (ctrl_right_tx_dir),
        .clear         (ctrl_right_tx_clear),
        .src_node_id   ({3'b000, node_id}),
        .prio_class    (ctrl_right_tx_class),
        .eta           (ctrl_right_tx_eta),
        .uart_tx_busy  (right_uart_tx_busy),
        .uart_tx_data  (right_uart_tx_data),
        .uart_tx_start (right_uart_tx_start),
        .tx_done       (right_cas_tx_done)
    );

    rs485_dir_ctrl u_right_rs485 (
        .clk       (clk),
        .rst_n     (rst_n),
        .tx_start  (ctrl_right_tx_send),
        .tx_done   (right_cas_tx_done),
        .rs485_de  (right_rs485_de),
        .tx_enable (right_rs485_tx_enable)
    );

    assign right_tx = right_uart_tx_serial;
    assign right_de = right_rs485_de;

    // =========================================================================
    // Cascade Controller
    // =========================================================================
    cascade_ctrl u_cascade (
        .clk                (clk),
        .rst_n              (rst_n),
        .node_id            (node_id),
        .prio_active        (fsm_prio_active),
        .winner_class       (arb_winner_class),
        .dir_out            (dir_out),
        .dir_valid          (dir_valid),
        .dir_confidence     (dir_confidence),
        // LEFT RX
        .left_pkt_valid     (left_pkt_valid),
        .left_pkt_flag_dir  (left_pkt_flag_dir),
        .left_pkt_flag_clear(left_pkt_flag_clear),
        .left_pkt_class     (left_pkt_class),
        .left_pkt_eta       (left_pkt_eta),
        // RIGHT RX
        .right_pkt_valid     (right_pkt_valid),
        .right_pkt_flag_dir  (right_pkt_flag_dir),
        .right_pkt_flag_clear(right_pkt_flag_clear),
        .right_pkt_class     (right_pkt_class),
        .right_pkt_eta       (right_pkt_eta),
        // LEFT TX
        .left_tx_send       (ctrl_left_tx_send),
        .left_tx_valid      (ctrl_left_tx_valid),
        .left_tx_dir        (ctrl_left_tx_dir),
        .left_tx_clear      (ctrl_left_tx_clear),
        .left_tx_class      (ctrl_left_tx_class),
        .left_tx_eta        (ctrl_left_tx_eta),
        // RIGHT TX
        .right_tx_send      (ctrl_right_tx_send),
        .right_tx_valid     (ctrl_right_tx_valid),
        .right_tx_dir       (ctrl_right_tx_dir),
        .right_tx_clear     (ctrl_right_tx_clear),
        .right_tx_class     (ctrl_right_tx_class),
        .right_tx_eta       (ctrl_right_tx_eta),
        // Pre-clear
        .preclear_req       (cascade_preclear)
    );

endmodule
