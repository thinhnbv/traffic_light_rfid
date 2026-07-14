// =============================================================================
// Module     : spi_test_top
// Project    : Traffic Light Controller with RFID + Green Wave
// Description: Top-level module for physical SPI master loopback testing 
//              on Gowin FPGA (GW5A-25A / ACG525).
//              Nối tắt (jumper) chân MOSI sang MISO để chạy loopback test.
// =============================================================================

module spi_test_top (
    input        clk_in,       // Xung clock 50MHz từ board
    input        btn_rst_n,    // Nút nhấn Reset (tích cực thấp)
    input        btn_start_n,  // Nút nhấn kích hoạt truyền SPI (tích cực thấp)

    // Các chân SPI vật lý
    output       spi_sclk,
    output       spi_mosi,
    input        spi_miso,     // Nhận dữ liệu loopback

    // Hiển thị kết quả
    output [7:0] led_rx_data,  // 8 bóng LED hiển thị byte rx_data nhận về
    output       led_done,     // LED báo truyền xong
    output       led_busy      // LED báo đang bận truyền
);

    // =========================================================================
    // 1. Bộ Debounce và Phát hiện Cạnh cho nút nhấn Start
    // =========================================================================
    reg [19:0] debounce_cnt;
    reg        btn_start_sync;
    reg        btn_start_stable;
    reg        btn_start_stable_d;

    // Lọc nhiễu nút nhấn (20ms ở tần số 50MHz)
    always @(posedge clk_in or negedge btn_rst_n) begin
        if (!btn_rst_n) begin
            debounce_cnt       <= 20'd0;
            btn_start_sync     <= 1'b1;
            btn_start_stable   <= 1'b1;
            btn_start_stable_d <= 1'b1;
        end else begin
            btn_start_sync <= btn_start_n;
            
            if (btn_start_sync == btn_start_stable) begin
                debounce_cnt <= 20'd0;
            end else begin
                debounce_cnt <= debounce_cnt + 1'b1;
                if (debounce_cnt == 20'd1_000_000) begin // 20ms
                    btn_start_stable <= btn_start_sync;
                end
            end
            btn_start_stable_d <= btn_start_stable;
        end
    end

    // Xung kích hoạt truyền SPI (khi nút nhấn được nhấn xuống - cạnh xuống)
    wire spi_start = (btn_start_stable_d && !btn_start_stable);

    // =========================================================================
    // 2. Khởi tạo SPI Master
    // =========================================================================
    wire [7:0] rx_data_raw;
    wire       spi_done_sig;
    
    spi_master #(
        .CLK_DIV(8) // 50MHz / (2 * 8) = 3.125 MHz
    ) u_spi_master (
        .clk(clk_in),
        .rst_n(btn_rst_n),
        .start(spi_start),
        .tx_data(8'hFF), // Gửi thử byte test 0xA5 (10100101 nhị phân)
        .rx_data(rx_data_raw),
        .done(spi_done_sig),
        .busy(led_busy),
        .sclk(spi_sclk),
        .mosi(spi_mosi),
        .miso(spi_miso)
    );

    // =========================================================================
    // 3. Chốt dữ liệu nhận được lên LED khi truyền xong
    // =========================================================================
    reg [7:0] rx_data_latched;
    reg       done_latched;

    always @(posedge clk_in or negedge btn_rst_n) begin
        if (!btn_rst_n) begin
            rx_data_latched <= 8'd0;
            done_latched    <= 1'b0;
        end else begin
            if (spi_done_sig) begin
                rx_data_latched <= rx_data_raw;
                done_latched    <= 1'b1;
            end else if (spi_start) begin
                done_latched    <= 1'b0; // Reset trạng thái done khi có lượt truyền mới
            end
        end
    end

    // Gán dữ liệu ra LED (Tùy thuộc vào thiết kế board: Active-High hay Active-Low)
    // Ở đây mặc định là Active-High (LED sáng khi chân ra mức 1)
    assign led_rx_data = rx_data_latched;
    assign led_done    = done_latched;

endmodule
