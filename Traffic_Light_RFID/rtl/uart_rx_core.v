// =============================================================================
// Module     : uart_rx_core
// Project    : TLC-RFID v4.4
// Description: 9600 bps 8N1 UART receiver with mid-bit sampling.
//              Detects start bit via falling edge, samples at bit center.
// =============================================================================

module uart_rx_core (
    input  wire       clk,
    input  wire       rst_n,
    input  wire       rx_serial,
    output reg  [7:0] rx_data,
    output reg        rx_valid,
    output reg        rx_error
);

    // Baud rate parameters (50 MHz / 9600 = 5208 clocks per bit)
    parameter CLKS_PER_BIT = 5208;
    parameter HALF_BIT     = CLKS_PER_BIT / 2;  // 2604 — center of bit

    // FSM States
    localparam S_IDLE  = 2'd0;
    localparam S_START = 2'd1;
    localparam S_DATA  = 2'd2;
    localparam S_STOP  = 2'd3;

    reg [1:0]  state;
    reg [12:0] baud_cnt;        // max 5208 → needs 13 bits
    reg [2:0]  bit_idx;         // 0-7
    reg [7:0]  rx_shift;        // shift register
    reg        rx_prev;         // previous rx_serial for falling edge detect

    // Falling edge detection on rx_serial
    wire rx_falling = rx_prev & ~rx_serial;

    // Track previous rx_serial value
    always @(posedge clk) begin
        if (!rst_n)
            rx_prev <= 1'b1;
        else
            rx_prev <= rx_serial;
    end

    // Main FSM
    always @(posedge clk) begin
        if (!rst_n) begin
            state    <= S_IDLE;
            baud_cnt <= 0;
            bit_idx  <= 0;
            rx_shift <= 8'd0;
            rx_data  <= 8'd0;
            rx_valid <= 1'b0;
            rx_error <= 1'b0;
        end else begin
            // Default: clear pulses
            rx_valid <= 1'b0;
            rx_error <= 1'b0;

            case (state)
                // ─────────────────────────────────────────────
                S_IDLE: begin
                    if (rx_falling) begin
                        // Potential start bit detected
                        baud_cnt <= 0;
                        state    <= S_START;
                    end
                end

                // ─────────────────────────────────────────────
                S_START: begin
                    // Wait until center of start bit
                    if (baud_cnt == HALF_BIT - 1) begin
                        if (rx_serial == 1'b0) begin
                            // Confirmed start bit — move to data
                            baud_cnt <= 0;
                            bit_idx  <= 0;
                            state    <= S_DATA;
                        end else begin
                            // False start — go back to idle
                            state <= S_IDLE;
                        end
                    end else begin
                        baud_cnt <= baud_cnt + 1;
                    end
                end

                // ─────────────────────────────────────────────
                S_DATA: begin
                    if (baud_cnt == CLKS_PER_BIT - 1) begin
                        // Sample at center of data bit
                        baud_cnt <= 0;
                        rx_shift <= {rx_serial, rx_shift[7:1]};  // LSB first, shift right
                        if (bit_idx == 3'd7) begin
                            state <= S_STOP;
                        end else begin
                            bit_idx <= bit_idx + 1;
                        end
                    end else begin
                        baud_cnt <= baud_cnt + 1;
                    end
                end

                // ─────────────────────────────────────────────
                S_STOP: begin
                    if (baud_cnt == CLKS_PER_BIT - 1) begin
                        // Sample stop bit at center
                        if (rx_serial == 1'b1) begin
                            rx_data  <= rx_shift;
                            rx_valid <= 1'b1;
                        end else begin
                            rx_error <= 1'b1;  // Framing error — stop bit != 1
                        end
                        state <= S_IDLE;
                    end else begin
                        baud_cnt <= baud_cnt + 1;
                    end
                end

                default: state <= S_IDLE;
            endcase
        end
    end

endmodule
