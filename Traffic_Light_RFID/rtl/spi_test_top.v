// =============================================================================
// Module     : spi_test_top
// Project    : TLC-RFID v4.4
// Description: Test top-level to verify RC522 RFID connection and scan logic.
//              Maps the 4 reader 'tag_detected' status signals directly
//              to the 4 onboard LEDs.
//              LED0 -> West Reader
//              LED1 -> East Reader
//              LED2 -> South Reader
//              LED3 -> North Reader
// =============================================================================

module spi_test_top (
    input  wire       clk_osc,      // T9 (50MHz clock)
    input  wire       rst_btn_n,    // B16 (Reset)
    // SPI Pins to RC522
    input  wire       spi_miso,     // D18
    output wire       spi_sck,      // G14
    output wire       spi_mosi,     // D17
    output wire [3:0] cs_n,         // cs_n[3]=G16, cs_n[2]=G18, cs_n[1]=F17, cs_n[0]=F18
    // Onboard LEDs
    output wire [3:0] led_onboard   // [3]=A9 (LED3), [2]=B9 (LED2), [1]=C14 (LED1), [0]=D14 (LED0)
);

    // Reset Synchronizer
    wire rst_n;
    rst_sync u_rst (
        .clk         (clk_osc),
        .rst_async_n (rst_btn_n),
        .rst_sync_n  (rst_n)
    );

    // Instantiate 4-Reader RFID Driver
    wire [3:0] tag_detected;
    wire [31:0] uid_n, uid_s, uid_e, uid_w;

    rc522_driver_x4 u_rfid_driver (
        .clk          (clk_osc),
        .rst_n        (rst_n),
        .spi_miso     (spi_miso),
        .spi_sck      (spi_sck),
        .spi_mosi     (spi_mosi),
        .cs_n         (cs_n),
        .tag_detected (tag_detected),
        .tag_uid_n    (uid_n),
        .tag_uid_s    (uid_s),
        .tag_uid_e    (uid_e),
        .tag_uid_w    (uid_w)
    );

    // Ánh xạ tag_detected ra LED onboard (mức 1 -> LED sáng, phụ thuộc active-low/high của board)
    // Thông thường trên ACG525, xuất mức '0' ra LED onboard thì LED sáng, '1' là tắt.
    // Nếu board của bạn tích cực cao (1=sáng), hãy bỏ dấu phủ định '~'.
    assign led_onboard[0] = ~tag_detected[0]; // LED0 (D14) hiển thị West Reader
    assign led_onboard[1] = ~tag_detected[1]; // LED1 (C14) hiển thị East Reader
    assign led_onboard[2] = ~tag_detected[2]; // LED2 (B9)  hiển thị South Reader
    assign led_onboard[3] = ~tag_detected[3]; // LED3 (A9)  hiển thị North Reader

endmodule
