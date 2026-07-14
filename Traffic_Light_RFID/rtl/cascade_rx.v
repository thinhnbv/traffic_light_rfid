// =============================================================================
// Module     : cascade_rx
// Project    : TLC-RFID v4.4
// Description: Receives 5-byte cascade packet from uart_rx_core.
//              Verifies header (0xCA) and XOR checksum.
//              Parses flag fields on valid packet.
// =============================================================================

module cascade_rx (
    input  wire       clk,
    input  wire       rst_n,
    input  wire [7:0] uart_rx_data,
    input  wire       uart_rx_valid,   // Strobe: new byte from uart_rx_core
    output reg        pkt_valid,       // Pulse: valid packet received
    output reg        pkt_flag_valid,  // Flags[7] — parsed
    output reg        pkt_flag_dir,    // Flags[6] — 0=NS, 1=EW
    output reg        pkt_flag_clear,  // Flags[5] — vehicle passed
    output reg  [4:0] pkt_src_id,      // Flags[4:0]
    output reg  [2:0] pkt_class,       // Byte 2: priority_class
    output reg  [7:0] pkt_eta,         // Byte 3: ETA
    output reg        pkt_error        // Pulse: checksum or header error
);

    // FSM States
    localparam S_WAIT_HEADER = 2'd0;
    localparam S_COLLECT     = 2'd1;
    localparam S_VERIFY      = 2'd2;

    // Packet timeout — if 5 bytes don't arrive within ~10ms
    // At 9600 bps, 5 bytes ≈ 5.2ms. Use ~15ms = 750,000 cycles
    parameter [19:0] PKT_TIMEOUT = 20'd750_000;

    reg [1:0]  state;
    reg [2:0]  byte_cnt;          // 0-4
    reg [7:0]  pkt_buf [0:4];
    reg [19:0] timeout_cnt;

    always @(posedge clk) begin
        if (!rst_n) begin
            state          <= S_WAIT_HEADER;
            byte_cnt       <= 3'd0;
            timeout_cnt    <= 20'd0;
            pkt_valid      <= 1'b0;
            pkt_error      <= 1'b0;
            pkt_flag_valid <= 1'b0;
            pkt_flag_dir   <= 1'b0;
            pkt_flag_clear <= 1'b0;
            pkt_src_id     <= 5'd0;
            pkt_class      <= 3'b000;
            pkt_eta        <= 8'd0;
        end else begin
            // Default: clear pulses
            pkt_valid <= 1'b0;
            pkt_error <= 1'b0;

            case (state)
                // ─────────────────────────────────────
                S_WAIT_HEADER: begin
                    if (uart_rx_valid) begin
                        if (uart_rx_data == 8'hCA) begin
                            pkt_buf[0] <= 8'hCA;
                            byte_cnt   <= 3'd1;
                            timeout_cnt <= 20'd0;
                            state      <= S_COLLECT;
                        end
                        // Else: ignore non-header bytes
                    end
                end

                // ─────────────────────────────────────
                S_COLLECT: begin
                    if (uart_rx_valid) begin
                        pkt_buf[byte_cnt] <= uart_rx_data;
                        timeout_cnt <= 20'd0;  // Reset timeout on each byte
                        if (byte_cnt == 3'd4) begin
                            state <= S_VERIFY;
                        end else begin
                            byte_cnt <= byte_cnt + 1;
                        end
                    end else begin
                        // Timeout protection
                        if (timeout_cnt >= PKT_TIMEOUT) begin
                            pkt_error <= 1'b1;
                            state     <= S_WAIT_HEADER;
                        end else begin
                            timeout_cnt <= timeout_cnt + 1;
                        end
                    end
                end

                // ─────────────────────────────────────
                S_VERIFY: begin
                    // Check XOR checksum
                    if ((pkt_buf[0] ^ pkt_buf[1] ^ pkt_buf[2] ^ pkt_buf[3]) == pkt_buf[4]) begin
                        // Valid packet — parse fields
                        pkt_flag_valid <= pkt_buf[1][7];
                        pkt_flag_dir   <= pkt_buf[1][6];
                        pkt_flag_clear <= pkt_buf[1][5];
                        pkt_src_id     <= pkt_buf[1][4:0];
                        pkt_class      <= pkt_buf[2][2:0];
                        pkt_eta        <= pkt_buf[3];
                        pkt_valid      <= 1'b1;
                    end else begin
                        pkt_error <= 1'b1;
                    end
                    state <= S_WAIT_HEADER;
                end

                default: state <= S_WAIT_HEADER;
            endcase
        end
    end

endmodule
