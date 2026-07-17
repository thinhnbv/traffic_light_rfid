// =============================================================================
// Module     : rc522_driver
// Project    : Traffic Light Controller with RFID + Green Wave
// Description: Synthesizable MFRC522 (RC522) RFID Reader Driver.
//              Handles soft-reset, initialization, antenna control, 
//              and cyclic polling (REQA + Anticollision) to read 4-byte UID.
//              Communicates with RC522 via spi_master.
// =============================================================================

module rc522_driver (
    input             clk,          // System clock (50 MHz)
    input             rst_n,        // Active-low reset

    // SPI Interface
    output            sclk,
    output            mosi,
    input             miso,
    output reg        cs_n,

    // Status / Data outputs
    output reg        tag_detected, // Asserted when card is present
    output reg [31:0] tag_uid       // Latch of the 4-byte UID
);

    // =========================================================================
    // RC522 Registers & Commands
    // =========================================================================
    localparam REG_COMMAND      = 6'h01;
    localparam REG_COM_IEN      = 6'h02;
    localparam REG_COM_IRQ      = 6'h04;
    localparam REG_FIFO_DATA    = 6'h09;
    localparam REG_FIFO_LEVEL   = 6'h0A;
    localparam REG_BIT_FRAMING  = 6'h0D;
    localparam REG_MODE         = 6'h11;
    localparam REG_TX_CONTROL   = 6'h14;
    localparam REG_TX_ASK       = 6'h15;
    localparam REG_T_MODE       = 6'h2A;
    localparam REG_T_PRESCALER  = 6'h2B;
    localparam REG_T_RELOAD_H   = 6'h2C;
    localparam REG_T_RELOAD_L   = 6'h2D;

    localparam CMD_IDLE         = 8'h00;
    localparam CMD_TRANSCEIVE   = 8'h0C;
    localparam CMD_SOFTRESET    = 8'h0F;

    localparam REQA_CMD         = 8'h26;
    localparam SELECT_CMD       = 8'h93;
    localparam NVB_CMD          = 8'h20; // 20h = 2 bytes (CMD + NVB)

    // =========================================================================
    // Master State Machine States
    // =========================================================================
    localparam S_HW_RESET       = 4'd0;  // Initial reset write
    localparam S_HW_RESET_WAIT  = 4'd1;  // Wait for reset to execute (~10ms)
    localparam S_INIT_REGS      = 4'd2;  // Write configuration registers
    localparam S_POLL_DELAY     = 4'd3;  // Delay between polling cycles (~100ms)
    (
        // REQA sequence
        S_REQA_PREP       = 4'd4,
        S_REQA_WRITE_FIFO = 4'd5,
        S_REQA_EXEC       = 4'd6,
        S_REQA_WAIT_RX    = 4'd7,
        S_REQA_CHECK      = 4'd8
    );
    localparam S_ANTICOL_PREP   = 4'd9;
    localparam S_ANTICOL_WRITE  = 4'd10;
    localparam S_ANTICOL_EXEC   = 4'd11;
    localparam S_ANTICOL_WAIT   = 4'd12;
    localparam S_ANTICOL_READ   = 4'd13;

    // =========================================================================
    // SPI Sub-FSM Signals
    // =========================================================================
    reg        spi_start;
    reg  [7:0] spi_tx_data;
    wire [7:0] spi_rx_data;
    wire       spi_done;
    wire       spi_busy;

    // SPI transaction manager
    reg  [2:0] spi_step;
    reg        spi_op_write;   // 1 = Write, 0 = Read
    reg  [5:0] spi_op_addr;
    reg  [7:0] spi_op_data;
    reg        spi_op_trigger;
    reg        spi_op_busy;
    reg  [7:0] spi_op_rx_val;

    // SPI controller
    always @(posedge clk) begin
        if (!rst_n) begin
            spi_start     <= 1'b0;
            spi_tx_data   <= 8'h00;
            cs_n          <= 1'b1;
            spi_step      <= 3'd0;
            spi_op_busy   <= 1'b0;
            spi_op_rx_val <= 8'h00;
        end else begin
            if (spi_op_trigger && !spi_op_busy) begin
                spi_op_busy <= 1'b1;
                spi_step    <= 3'd0;
            end else if (spi_op_busy) begin
                case (spi_step)
                    3'd0: begin
                        cs_n        <= 1'b0;
                        spi_tx_data <= spi_op_write ? ((spi_op_addr << 1) & 8'h7E) : 
                                                     (((spi_op_addr << 1) & 8'h7E) | 8'h80);
                        spi_start   <= 1'b1;
                        spi_step    <= 3'd1;
                    end
                    3'd1: begin
                        spi_start <= 1'b0;
                        if (spi_done) begin
                            spi_step <= 3'd2;
                        end
                    end
                    3'd2: begin
                        // Second byte (Data to write / Dummy byte for read)
                        spi_tx_data <= spi_op_write ? spi_op_data : 8'h00;
                        spi_start   <= 1'b1;
                        spi_step    <= 3'd3;
                    end
                    3'd3: begin
                        spi_start <= 1'b0;
                        if (spi_done) begin
                            spi_op_rx_val <= spi_rx_data;
                            cs_n          <= 1'b1;
                            spi_op_busy   <= 1'b0;
                            spi_step      <= 3'd4;
                        end
                    end
                endcase
            end
        end
    end

    // Instantiation of the byte-level SPI Master
    spi_master #(.CLK_DIV(8)) u_spi (
        .clk     (clk),
        .rst_n   (rst_n),
        .start   (spi_start),
        .tx_data (spi_tx_data),
        .rx_data (spi_rx_data),
        .done    (spi_done),
        .busy    (spi_busy),
        .sclk    (sclk),
        .mosi    (mosi),
        .miso    (miso)
    );

    // =========================================================================
    // Driver Main State Machine
    // =========================================================================
    reg [3:0]  state;
    reg [31:0] delay_cnt;
    reg [3:0]  init_index;
    reg [2:0]  read_index;
    reg [7:0]  fifo_level;
    reg [7:0]  uid_bytes [3:0];
    reg [1:0]  miss_cnt;

    // Registers initialization list: {RegAddress, Data}
    reg [13:0] init_rom [5:0];
    initial begin
        init_rom[0] = {REG_T_MODE,      8'h8D}; // TMode
        init_rom[1] = {REG_T_PRESCALER, 8'h3E}; // TPrescaler
        init_rom[2] = {REG_T_RELOAD_H,  8'h00}; // TReloadHigh
        init_rom[3] = {REG_T_RELOAD_L,  8'h1E}; // TReloadLow
        init_rom[4] = {REG_Tx_ASK,      8'h40}; // TxASK (Force 100% ASK)
        init_rom[5] = {REG_TX_CONTROL,  8'h83}; // TxControl (Turn on Antenna)
    end

    always @(posedge clk) begin
        if (!rst_n) begin
            state          <= S_HW_RESET;
            delay_cnt      <= 32'd0;
            init_index     <= 4'd0;
            read_index     <= 3'd0;
            fifo_level     <= 8'd0;
            miss_cnt       <= 2'd0;
            tag_detected   <= 1'b0;
            tag_uid        <= 32'h00000000;
            spi_op_trigger <= 1'b0;
            spi_op_write   <= 1'b0;
            spi_op_addr    <= 6'd0;
            spi_op_data    <= 8'h00;
            uid_bytes[0]   <= 8'd0;
            uid_bytes[1]   <= 8'd0;
            uid_bytes[2]   <= 8'd0;
            uid_bytes[3]   <= 8'd0;
        end else begin
            spi_op_trigger <= 1'b0; // Default pulse

            case (state)
                // -------------------------------------------------------------
                // RESET AND INITIALIZATION
                // -------------------------------------------------------------
                S_HW_RESET: begin
                    if (!spi_op_busy && !spi_op_trigger) begin
                        spi_op_write   <= 1'b1;
                        spi_op_addr    <= REG_COMMAND;
                        spi_op_data    <= CMD_SOFTRESET;
                        spi_op_trigger <= 1'b1;
                        state          <= S_HW_RESET_WAIT;
                        delay_cnt      <= 32'd500_000; // ~10ms wait
                    end
                end

                S_HW_RESET_WAIT: begin
                    if (delay_cnt != 32'd0) begin
                        delay_cnt <= delay_cnt - 1'b1;
                    end else begin
                        state      <= S_INIT_REGS;
                        init_index <= 4'd0;
                    end
                end

                S_INIT_REGS: begin
                    if (!spi_op_busy && !spi_op_trigger) begin
                        if (init_index < 4'd6) begin
                            spi_op_write   <= 1'b1;
                            spi_op_addr    <= init_rom[init_index][13:8];
                            spi_op_data    <= init_rom[init_index][7:0];
                            spi_op_trigger <= 1'b1;
                            init_index     <= init_index + 1'b1;
                        end else begin
                            state     <= S_POLL_DELAY;
                            delay_cnt <= 32'd5_000_000; // ~100ms delay between polls
                        end
                    end
                end

                // -------------------------------------------------------------
                // POLLING DELAY
                // -------------------------------------------------------------
                S_POLL_DELAY: begin
                    if (delay_cnt != 32'd0) begin
                        delay_cnt <= delay_cnt - 1'b1;
                    end else begin
                        state <= S_REQA_PREP;
                    end
                end

                // -------------------------------------------------------------
                // REQA CARD REQUEST SEQUENCE
                // -------------------------------------------------------------
                S_REQA_PREP: begin
                    // Clear interrupts (ComIrqReg = 0x7F) and reset FIFO (FIFOLevelReg = 0x80)
                    if (!spi_op_busy && !spi_op_trigger) begin
                        spi_op_write   <= 1'b1;
                        spi_op_addr    <= REG_COM_IRQ;
                        spi_op_data    <= 8'h7F;
                        spi_op_trigger <= 1'b1;
                        state          <= S_REQA_WRITE_FIFO;
                    end
                end

                S_REQA_WRITE_FIFO: begin
                    if (!spi_op_busy && !spi_op_trigger) begin
                        // Clear FIFO
                        spi_op_write   <= 1'b1;
                        spi_op_addr    <= REG_FIFO_LEVEL;
                        spi_op_data    <= 8'h80;
                        spi_op_trigger <= 1'b1;
                        // In the next step, write REQA to FIFO data
                        state          <= S_REQA_EXEC;
                    end
                end

                S_REQA_EXEC: begin
                    if (!spi_op_busy && !spi_op_trigger) begin
                        // Write REQA cmd to FIFO
                        spi_op_write   <= 1'b1;
                        spi_op_addr    <= REG_FIFO_DATA;
                        spi_op_data    <= REQA_CMD;
                        spi_op_trigger <= 1'b1;
                        
                        // Execute CMD_TRANSCEIVE and configure BitFraming to start transmission
                        // Tên gọi chuyển sang state RX
                        state          <= S_REQA_WAIT_RX;
                        delay_cnt      <= 32'd100_000; // ~2ms wait for RF
                    end
                end

                S_REQA_WAIT_RX: begin
                    // We need to write TRANSCEIVE cmd and start transceive via BitFramingReg
                    if (!spi_op_busy && !spi_op_trigger) begin
                        // 1. Write TRANSCEIVE to CMD
                        spi_op_write   <= 1'b1;
                        spi_op_addr    <= REG_COMMAND;
                        spi_op_data    <= CMD_TRANSCEIVE;
                        spi_op_trigger <= 1'b1;
                        
                        state          <= S_REQA_CHECK;
                    end
                end

                S_REQA_CHECK: begin
                    // 2. Write 0x87 to BitFramingReg (StartSend = 1, 7 bits in last byte)
                    if (!spi_op_busy && !spi_op_trigger) begin
                        spi_op_write   <= 1'b1;
                        spi_op_addr    <= REG_BIT_FRAMING;
                        spi_op_data    <= 8'h87; // REQA is 7 bits
                        spi_op_trigger <= 1'b1;
                        
                        state          <= S_ANTICOL_PREP; // Go wait/read results
                        delay_cnt      <= 32'd150_000;    // ~3ms wait
                    end
                end

                // -------------------------------------------------------------
                // ANTICOLLISION SEQUENCE (READ UID)
                // -------------------------------------------------------------
                S_ANTICOL_PREP: begin
                    if (delay_cnt != 32'd0) begin
                        delay_cnt <= delay_cnt - 1'b1;
                    end else begin
                        // Read FIFO level to check if card responded to REQA
                        if (!spi_op_busy && !spi_op_trigger) begin
                            spi_op_write   <= 1'b0;
                            spi_op_addr    <= REG_FIFO_LEVEL;
                            spi_op_trigger <= 1'b1;
                            state          <= S_ANTICOL_WRITE;
                        end
                    end
                end

                S_ANTICOL_WRITE: begin
                    if (!spi_op_busy) begin
                        fifo_level <= spi_op_rx_val;
                        
                        if (spi_op_rx_val >= 8'd2) begin
                            // Card present! Clear FIFO & execute Anticollision
                            spi_op_write   <= 1'b1;
                            spi_op_addr    <= REG_FIFO_LEVEL;
                            spi_op_data    <= 8'h80; // Clear FIFO
                            spi_op_trigger <= 1'b1;
                            state          <= S_ANTICOL_EXEC;
                        end else begin
                            // No card detected, increment miss count
                            if (miss_cnt < 2'd3) begin
                                miss_cnt <= miss_cnt + 1'b1;
                            end else begin
                                tag_detected <= 1'b0; // No tag present
                            end
                            state     <= S_POLL_DELAY;
                            delay_cnt <= 32'd5_000_000;
                        end
                    end
                end

                S_ANTICOL_EXEC: begin
                    if (!spi_op_busy && !spi_op_trigger) begin
                        // Write SELECT Cascade 1 (0x93) to FIFO
                        spi_op_write   <= 1'b1;
                        spi_op_addr    <= REG_FIFO_DATA;
                        spi_op_data    <= SELECT_CMD;
                        spi_op_trigger <= 1'b1;
                        state          <= S_ANTICOL_WAIT;
                    end
                end

                S_ANTICOL_WAIT: begin
                    if (!spi_op_busy && !spi_op_trigger) begin
                        // Write NVB = 20h to FIFO
                        spi_op_write   <= 1'b1;
                        spi_op_addr    <= REG_FIFO_DATA;
                        spi_op_data    <= NVB_CMD;
                        spi_op_trigger <= 1'b1;
                        state          <= S_ANTICOL_READ;
                    end
                end

                S_ANTICOL_READ: begin
                    // Trigger TRANSCEIVE and send SELECT command via BitFramingReg
                    if (!spi_op_busy && !spi_op_trigger) begin
                        // Write TRANSCEIVE to COMMAND
                        spi_op_write   <= 1'b1;
                        spi_op_addr    <= REG_COMMAND;
                        spi_op_data    <= CMD_TRANSCEIVE;
                        spi_op_trigger <= 1'b1;
                        
                        state          <= 4'd14; // Sub-state to set BitFraming
                    end
                end

                4'd14: begin
                    if (!spi_op_busy && !spi_op_trigger) begin
                        // Write 0x80 to BitFramingReg (StartSend = 1)
                        spi_op_write   <= 1'b1;
                        spi_op_addr    <= REG_BIT_FRAMING;
                        spi_op_data    <= 8'h80;
                        spi_op_trigger <= 1'b1;
                        
                        state          <= 4'd15;
                        delay_cnt      <= 32'd200_000; // ~4ms wait for card response
                    end
                end

                4'd15: begin
                    if (delay_cnt != 32'd0) begin
                        delay_cnt <= delay_cnt - 1'b1;
                    end else begin
                        // Read FIFO level: should be 5 bytes (4 bytes UID + 1 check byte)
                        if (!spi_op_busy && !spi_op_trigger) begin
                            spi_op_write   <= 1'b0;
                            spi_op_addr    <= REG_FIFO_LEVEL;
                            spi_op_trigger <= 1'b1;
                            state          <= 4'd11; // Reuse for UID read sequence
                        end
                    end
                end

                // Sequential UID read states (States 11, 12, 13, 10 are reused for reading bytes)
                4'd11: begin
                    if (!spi_op_busy) begin
                        fifo_level <= spi_op_rx_val;
                        
                        if (spi_op_rx_val >= 8'd5) begin
                            // Start reading UID bytes sequentially from FIFODataReg
                            spi_op_write   <= 1'b0;
                            spi_op_addr    <= REG_FIFO_DATA;
                            spi_op_trigger <= 1'b1;
                            read_index     <= 3'd0;
                            state          <= 4'd12;
                        end else begin
                            // Anticollision failed
                            if (miss_cnt < 2'd3) begin
                                miss_cnt <= miss_cnt + 1'b1;
                            end else begin
                                tag_detected <= 1'b0;
                            end
                            state     <= S_POLL_DELAY;
                            delay_cnt <= 32'd5_000_000;
                        end
                    end
                end

                4'd12: begin
                    if (!spi_op_busy) begin
                        uid_bytes[read_index] <= spi_op_rx_val;
                        if (read_index < 3'd3) begin
                            read_index     <= read_index + 1'b1;
                            spi_op_write   <= 1'b0;
                            spi_op_addr    <= REG_FIFO_DATA;
                            spi_op_trigger <= 1'b1;
                        end else begin
                            // Finished reading 4-byte UID! (BCC check byte is left in FIFO or ignored)
                            state          <= 4'd13;
                        end
                    end
                end

                4'd13: begin
                    // Latch outputs
                    tag_detected <= 1'b1;
                    tag_uid      <= {uid_bytes[0], uid_bytes[1], uid_bytes[2], uid_bytes[3]};
                    miss_cnt     <= 2'd0; // Reset miss count

                    // Return to poll delay
                    state     <= S_POLL_DELAY;
                    delay_cnt <= 32'd5_000_000;
                end

                default: begin
                    state <= S_POLL_DELAY;
                end
            endcase
        end
    end

endmodule
