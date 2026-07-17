// =============================================================================
// Module     : cascade_tx
// Project    : TLC-RFID v4.4
// Description: Packs cascade data into 5-byte packet and sends via uart_tx_core.
//              Calculates XOR checksum automatically.
//              Packet: [0xCA][Flags][Class][ETA][Checksum]
// =============================================================================

module cascade_tx (
    input  wire       clk,
    input  wire       rst_n,
    input  wire       send,           // Strobe: start transmitting packet
    input  wire       valid,          // Flags[7]: packet carries valid data
    input  wire       direction,      // Flags[6]: 0=NS, 1=EW
    input  wire       clear,          // Flags[5]: vehicle has passed
    input  wire [4:0] src_node_id,    // Flags[4:0]: source node ID
    input  wire [2:0] prio_class,     // Byte 2: priority class
    input  wire [7:0] eta,            // Byte 3: ETA in seconds
    input  wire       uart_tx_busy,   // From uart_tx_core
    output reg  [7:0] uart_tx_data,   // To uart_tx_core
    output reg        uart_tx_start,  // Strobe to uart_tx_core
    output reg        tx_done         // Pulse when all 5 bytes sent
);

    // FSM States
    localparam S_IDLE      = 2'd0;
    localparam S_SEND      = 2'd1;
    localparam S_WAIT_BYTE = 2'd2;
    localparam S_DONE      = 2'd3;

    reg [1:0]  state;
    reg [2:0]  byte_idx;
    reg [7:0]  pkt_buf [0:4];

    always @(posedge clk) begin
        if (!rst_n) begin
            state         <= S_IDLE;
            byte_idx      <= 3'd0;
            uart_tx_data  <= 8'd0;
            uart_tx_start <= 1'b0;
            tx_done       <= 1'b0;
        end else begin
            uart_tx_start <= 1'b0;  // default
            tx_done       <= 1'b0;

            case (state)
                // ─────────────────────────────────────
                S_IDLE: begin
                    if (send) begin
                        // Assemble packet
                        pkt_buf[0] <= 8'hCA;
                        pkt_buf[1] <= {valid, direction, clear, src_node_id};
                        pkt_buf[2] <= {5'b00000, prio_class};
                        pkt_buf[3] <= eta;
                        pkt_buf[4] <= 8'hCA ^
                                      {valid, direction, clear, src_node_id} ^
                                      {5'b00000, prio_class} ^
                                      eta;
                        byte_idx <= 3'd0;
                        state    <= S_SEND;
                    end
                end

                // ─────────────────────────────────────
                S_SEND: begin
                    if (!uart_tx_busy) begin
                        uart_tx_data  <= pkt_buf[byte_idx];
                        uart_tx_start <= 1'b1;
                        state         <= S_WAIT_BYTE;
                    end
                end

                // ─────────────────────────────────────
                S_WAIT_BYTE: begin
                    // Wait for UART to finish this byte
                    if (!uart_tx_busy) begin
                        if (byte_idx == 3'd4) begin
                            state <= S_DONE;
                        end else begin
                            byte_idx <= byte_idx + 1;
                            state    <= S_SEND;
                        end
                    end
                end

                // ─────────────────────────────────────
                S_DONE: begin
                    tx_done <= 1'b1;
                    state   <= S_IDLE;
                end

                default: state <= S_IDLE;
            endcase
        end
    end

endmodule
