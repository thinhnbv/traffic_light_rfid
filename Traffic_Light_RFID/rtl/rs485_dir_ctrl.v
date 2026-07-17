// =============================================================================
// Module     : rs485_dir_ctrl
// Project    : TLC-RFID v4.4
// Description: MAX485 DE (Driver Enable) controller.
//              Asserts DE before TX starts, holds during transmission,
//              keeps DE HIGH for 2 baud cycles after tx_done to prevent
//              truncating the stop bit on the physical line.
// =============================================================================

module rs485_dir_ctrl (
    input  wire clk,
    input  wire rst_n,
    input  wire tx_start,     // From cascade_tx: about to transmit
    input  wire tx_done,      // From uart_tx_core: stop bit completed
    output reg  rs485_de,     // To MAX485 DE pin: HIGH=transmit, LOW=receive
    output reg  tx_enable     // Gate for uart_tx_start — HIGH when DE settled
);

    // Guard delay = 2 × baud_period = 2 × (50MHz / 9600) = 10416 cycles
    // Needs 14-bit counter (2^14 = 16384)
    parameter GUARD_CYCLES = 14'd10416;

    // FSM States
    localparam S_IDLE      = 2'd0;
    localparam S_TX_ACTIVE = 2'd1;
    localparam S_GUARD     = 2'd2;

    reg [1:0]  state;
    reg [13:0] guard_cnt;

    always @(posedge clk) begin
        if (!rst_n) begin
            state     <= S_IDLE;
            rs485_de  <= 1'b0;
            tx_enable <= 1'b0;
            guard_cnt <= 14'd0;
        end else begin
            case (state)
                // ─────────────────────────────────────
                S_IDLE: begin
                    rs485_de  <= 1'b0;   // RX mode
                    tx_enable <= 1'b0;
                    if (tx_start) begin
                        rs485_de <= 1'b1;  // Assert DE immediately
                        state    <= S_TX_ACTIVE;
                    end
                end

                // ─────────────────────────────────────
                S_TX_ACTIVE: begin
                    rs485_de  <= 1'b1;
                    tx_enable <= 1'b1;   // Allow UART TX
                    if (tx_done) begin
                        tx_enable <= 1'b0;
                        guard_cnt <= 14'd0;
                        state     <= S_GUARD;
                    end
                end

                // ─────────────────────────────────────
                S_GUARD: begin
                    rs485_de  <= 1'b1;   // Keep DE HIGH during guard
                    tx_enable <= 1'b0;
                    if (guard_cnt >= GUARD_CYCLES) begin
                        state <= S_IDLE;
                    end else begin
                        guard_cnt <= guard_cnt + 1;
                    end
                end

                default: begin
                    state    <= S_IDLE;
                    rs485_de <= 1'b0;
                end
            endcase
        end
    end

endmodule
