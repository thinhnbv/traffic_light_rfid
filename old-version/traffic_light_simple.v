// =============================================================================
// Module     : traffic_light_simple
// Description: Đèn giao thông đơn giản nhất — 2 hướng NS/EW
//              Tự động cycle: Green → Yellow → Red, không priority, không gì hết
//
// Target     : Gowin ACG525 (GW5A-25A)
// Inputs     : clk (onboard), rst_n (button)
// Outputs    : 6 LEDs (ns_r, ns_y, ns_g, ew_r, ew_y, ew_g)
// =============================================================================

module traffic_light_simple (
    input        clk,       // Onboard clock
    input        rst_n,     // Reset button (active low)

    output reg   ns_red,    // NS đỏ
    output reg   ns_yellow, // NS vàng
    output reg   ns_green,  // NS xanh
    output reg   ew_red,    // EW đỏ
    output reg   ew_yellow, // EW vàng
    output reg   ew_green   // EW xanh
);

    // =========================================================================
    // Timing — Đổi ở đây để chỉnh thời gian mỗi phase
    // Với clock 50MHz: 50_000_000 = 1 giây
    // =========================================================================
    parameter CLK_FREQ = 50_000_000;    // (50MHz)

    parameter GREEN_TIME  = CLK_FREQ * 5;   // 5 giây xanh
    parameter YELLOW_TIME = CLK_FREQ * 2;   // 2 giây vàng
    parameter RED_TIME    = CLK_FREQ * 1;   // 1 giây all-red 

    // =========================================================================
    // States — chỉ 6 trạng thái đơn giản
    // =========================================================================
    parameter S_NS_GREEN  = 3'd0;   // NS xanh, EW đỏ
    parameter S_NS_YELLOW = 3'd1;   // NS vàng, EW đỏ
    parameter S_ALL_RED_1 = 3'd2;   // Tất cả đỏ (chuyển tiếp)
    parameter S_EW_GREEN  = 3'd3;   // EW xanh, NS đỏ
    parameter S_EW_YELLOW = 3'd4;   // EW vàng, NS đỏ
    parameter S_ALL_RED_2 = 3'd5;   // Tất cả đỏ (chuyển tiếp)

    // =========================================================================
    // Registers
    // =========================================================================
    reg [2:0]  state;
    reg [31:0] counter;

    // =========================================================================
    // Counter + State Machine
    // =========================================================================
    always @(posedge clk) begin
        if (!rst_n) begin
            state   <= S_NS_GREEN;
            counter <= GREEN_TIME - 1;
        end else begin
            if (counter == 0) begin
                // Counter hết → chuyển state
                case (state)
                    S_NS_GREEN: begin
                        state   <= S_NS_YELLOW;
                        counter <= YELLOW_TIME - 1;
                    end
                    S_NS_YELLOW: begin
                        state   <= S_ALL_RED_1;
                        counter <= RED_TIME - 1;
                    end
                    S_ALL_RED_1: begin
                        state   <= S_EW_GREEN;
                        counter <= GREEN_TIME - 1;
                    end
                    S_EW_GREEN: begin
                        state   <= S_EW_YELLOW;
                        counter <= YELLOW_TIME - 1;
                    end
                    S_EW_YELLOW: begin
                        state   <= S_ALL_RED_2;
                        counter <= RED_TIME - 1;
                    end
                    S_ALL_RED_2: begin
                        state   <= S_NS_GREEN;
                        counter <= GREEN_TIME - 1;
                    end
                    default: begin
                        state   <= S_NS_GREEN;
                        counter <= GREEN_TIME - 1;
                    end
                endcase
            end else begin
                counter <= counter - 1;
            end
        end
    end

    // =========================================================================
    // Output Logic — bảng đầu ra theo state
    // =========================================================================
    always @(*) begin
        // Default: tất cả tắt
        ns_red = 0; ns_yellow = 0; ns_green = 0;
        ew_red = 0; ew_yellow = 0; ew_green = 0;

        case (state)
            S_NS_GREEN: begin
                ns_green = 1;   // NS xanh
                ew_red   = 1;   // EW đỏ
            end
            S_NS_YELLOW: begin
                ns_yellow = 1;  // NS vàng
                ew_red    = 1;  // EW đỏ
            end
            S_ALL_RED_1: begin
                ns_red = 1;     // Tất cả đỏ
                ew_red = 1;
            end
            S_EW_GREEN: begin
                ns_red   = 1;   // NS đỏ
                ew_green = 1;   // EW xanh
            end
            S_EW_YELLOW: begin
                ns_red    = 1;  // NS đỏ
                ew_yellow = 1;  // EW vàng
            end
            S_ALL_RED_2: begin
                ns_red = 1;     // Tất cả đỏ
                ew_red = 1;
            end
            default: begin
                ns_red = 1;
                ew_red = 1;
            end
        endcase
    end

endmodule
