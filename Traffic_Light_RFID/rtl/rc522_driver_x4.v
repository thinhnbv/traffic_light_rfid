// =============================================================================
// Module     : rc522_driver_x4
// Project    : TLC-RFID v4.4
// Description: Polls 4 RC522 RFID readers via shared SPI bus.
//              INIT runs once after reset (write TxMode/RxMode/TxControl regs).
//              Then round-robin poll: REQA → ANTICOL → read 4-byte UID.
//              Uses ROM-based SPI command sequences for simplicity.
//
//              All logic runs on CLK_SYS — spi_sck is a toggled output, not
//              a separate clock domain. No CDC needed.
// =============================================================================

module rc522_driver_x4 (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        spi_miso,
    output wire        spi_sck,
    output wire        spi_mosi,
    output reg  [3:0]  cs_n,           // [3]=N [2]=S [1]=E [0]=W, active-low
    output reg  [3:0]  tag_detected,   // 1 when reader i has valid UID
    output reg  [31:0] tag_uid_n,
    output reg  [31:0] tag_uid_s,
    output reg  [31:0] tag_uid_e,
    output reg  [31:0] tag_uid_w
);

    // SPI clock divider: clk_div=4 → SPI = 50MHz / (2*(4+1)) = 5 MHz
    parameter [2:0] SPI_CLK_DIV = 3'd4;

    // Timeouts
    parameter [15:0] REQA_TIMEOUT = 16'd50000;  // ~1ms @ 50MHz
    parameter [15:0] CS_DESELECT_WAIT = 16'd50; // ~1us between reader switches

    // =========================================================================
    // SPI Master Instance
    // =========================================================================
    reg        spi_start;
    reg  [7:0] spi_tx_data;
    wire [7:0] spi_rx_data;
    wire       spi_done;

    spi_master u_spi (
        .clk      (clk),
        .rst_n    (rst_n),
        .start    (spi_start),
        .tx_data  (spi_tx_data),
        .clk_div  (SPI_CLK_DIV),
        .spi_miso (spi_miso),
        .spi_sck  (spi_sck),
        .spi_mosi (spi_mosi),
        .rx_data  (spi_rx_data),
        .done     (spi_done)
    );

    // =========================================================================
    // RC522 Register Addresses (shifted for SPI format)
    // RC522 SPI: byte = {R/W, addr[5:0], 0}
    //   Write: bit7=0, Read: bit7=1
    // =========================================================================
    // Write address format: {1'b0, addr[5:0], 1'b0}
    localparam ADDR_W_COMMAND     = 8'h02;  // CommandReg (0x01)
    localparam ADDR_W_FIFO_DATA   = 8'h12;  // FIFODataReg (0x09)
    localparam ADDR_W_FIFO_LEVEL  = 8'h14;  // FIFOLevelReg (0x0A)
    localparam ADDR_W_BIT_FRAMING = 8'h1A;  // BitFramingReg (0x0D)
    localparam ADDR_W_TX_MODE     = 8'h24;  // TxModeReg (0x12)
    localparam ADDR_W_RX_MODE     = 8'h26;  // RxModeReg (0x13)
    localparam ADDR_W_TX_CONTROL  = 8'h28;  // TxControlReg (0x14)

    // Read address format: {1'b1, addr[5:0], 1'b0}
    localparam ADDR_R_FIFO_DATA   = 8'h92;  // FIFODataReg (0x09)
    localparam ADDR_R_FIFO_LEVEL  = 8'h94;  // FIFOLevelReg (0x0A)
    localparam ADDR_R_STATUS1     = 8'h8E;  // Status1Reg (0x07)
    localparam ADDR_R_ERROR       = 8'h8C;  // ErrorReg (0x06)
    localparam ADDR_R_COM_IRQ     = 8'h88;  // ComIrqReg (0x04)

    // RC522 Commands
    localparam CMD_IDLE       = 8'h00;
    localparam CMD_TRANSCEIVE = 8'h0C;
    localparam REQA_CMD       = 8'h26;  // REQA (7-bit, ISO 14443-3)
    localparam SEL_CL1        = 8'h93;  // SELECT Cascade Level 1
    localparam NVB_FULL       = 8'h20;  // 2 bytes sent

    // =========================================================================
    // Main FSM States
    // =========================================================================
    localparam M_RESET        = 4'd0;
    localparam M_INIT_START   = 4'd1;
    localparam M_INIT_EXEC    = 4'd2;
    localparam M_INIT_NEXT    = 4'd3;
    localparam M_POLL_SELECT  = 4'd4;
    localparam M_POLL_REQA    = 4'd5;
    localparam M_POLL_WAIT    = 4'd6;
    localparam M_POLL_ANTICOL = 4'd7;
    localparam M_POLL_READ_UID= 4'd8;
    localparam M_POLL_NEXT    = 4'd9;
    localparam M_CS_DESELECT  = 4'd10;

    // SPI Command Sub-FSM States
    localparam C_IDLE      = 3'd0;
    localparam C_SEND_ADDR = 3'd1;
    localparam C_WAIT_ADDR = 3'd2;
    localparam C_SEND_DATA = 3'd3;
    localparam C_WAIT_DATA = 3'd4;
    localparam C_DONE      = 3'd5;

    // =========================================================================
    // Internal Registers
    // =========================================================================
    reg [3:0]  main_state;
    reg [2:0]  cmd_state;
    reg [1:0]  reader_idx;         // Current reader 0-3
    reg        init_done;
    reg [3:0]  init_step;          // Step within INIT sequence (0-5: 3 writes × 2)
    reg [3:0]  poll_step;          // Step within POLL sequence
    reg [15:0] timeout_cnt;
    reg [15:0] wait_cnt;           // Generic wait counter
    reg [2:0]  uid_byte_cnt;       // UID byte counter (0-3)
    reg [31:0] uid_temp;           // Temp UID accumulator
    reg [7:0]  cmd_addr;           // SPI address byte for sub-FSM
    reg [7:0]  cmd_data;           // SPI data byte for sub-FSM
    reg        cmd_trigger;        // Trigger sub-FSM
    reg        cmd_is_read;        // 1=read, 0=write
    wire       cmd_done;           // Sub-FSM completed
    reg [7:0]  cmd_rx_byte;        // Byte received from read

    assign cmd_done = (cmd_state == C_DONE);

    // =========================================================================
    // INIT Sequence ROM — 3 register writes per reader (6 SPI bytes)
    // Each entry: {addr_byte, data_byte}
    // =========================================================================
    reg [7:0] init_addr [0:2];
    reg [7:0] init_data [0:2];

    // Using initial block (synthesizable on Gowin with BSRAM inference)
    initial begin
        init_addr[0] = ADDR_W_TX_MODE;     init_data[0] = 8'h00;
        init_addr[1] = ADDR_W_RX_MODE;     init_data[1] = 8'h00;
        init_addr[2] = ADDR_W_TX_CONTROL;  init_data[2] = 8'h03;
    end

    // =========================================================================
    // CS_N Control — only 1 LOW at a time
    // =========================================================================
    always @(*) begin
        case (reader_idx)
            2'd0: cs_n = 4'b1110;  // W selected
            2'd1: cs_n = 4'b1101;  // E selected
            2'd2: cs_n = 4'b1011;  // S selected
            2'd3: cs_n = 4'b0111;  // N selected
            default: cs_n = 4'b1111;
        endcase
        // Override: deselect all in certain states
        if (main_state == M_RESET || main_state == M_POLL_SELECT ||
            main_state == M_CS_DESELECT || main_state == M_POLL_NEXT)
            cs_n = 4'b1111;
    end

    // =========================================================================
    // SPI Command Sub-FSM
    // Executes one RC522 register read or write (2 SPI bytes)
    // =========================================================================
    always @(posedge clk) begin
        if (!rst_n) begin
            cmd_state   <= C_IDLE;
            spi_start   <= 1'b0;
            spi_tx_data <= 8'd0;
            cmd_rx_byte <= 8'd0;
        end else begin
            spi_start <= 1'b0;  // default: no start

            case (cmd_state)
                C_IDLE: begin
                    if (cmd_trigger) begin
                        spi_tx_data <= cmd_addr;
                        spi_start   <= 1'b1;
                        cmd_state   <= C_WAIT_ADDR;
                    end
                end

                C_WAIT_ADDR: begin
                    if (spi_done) begin
                        // Address byte done — send data byte
                        spi_tx_data <= cmd_is_read ? 8'h00 : cmd_data;
                        spi_start   <= 1'b1;
                        cmd_state   <= C_WAIT_DATA;
                    end
                end

                C_WAIT_DATA: begin
                    if (spi_done) begin
                        if (cmd_is_read)
                            cmd_rx_byte <= spi_rx_data;
                        cmd_state <= C_DONE;
                    end
                end

                C_DONE: begin
                    // Stays here 1 cycle for main FSM to detect
                    cmd_state <= C_IDLE;
                end

                default: cmd_state <= C_IDLE;
            endcase
        end
    end

    // =========================================================================
    // Main FSM
    // =========================================================================
    always @(posedge clk) begin
        if (!rst_n) begin
            main_state   <= M_RESET;
            reader_idx   <= 2'd0;
            init_done    <= 1'b0;
            init_step    <= 4'd0;
            poll_step    <= 4'd0;
            timeout_cnt  <= 16'd0;
            wait_cnt     <= 16'd0;
            uid_byte_cnt <= 3'd0;
            uid_temp     <= 32'd0;
            tag_detected <= 4'b0000;
            tag_uid_n    <= 32'd0;
            tag_uid_s    <= 32'd0;
            tag_uid_e    <= 32'd0;
            tag_uid_w    <= 32'd0;
            cmd_trigger  <= 1'b0;
            cmd_addr     <= 8'd0;
            cmd_data     <= 8'd0;
            cmd_is_read  <= 1'b0;
        end else begin
            cmd_trigger <= 1'b0;  // default

            case (main_state)
                // ─────────────────────────────────────
                M_RESET: begin
                    reader_idx <= 2'd0;
                    init_step  <= 4'd0;
                    main_state <= M_INIT_START;
                end

                // ─────────────────────────────────────
                // INIT: Write 3 registers per reader, 4 readers total
                // ─────────────────────────────────────
                M_INIT_START: begin
                    if (init_step < 4'd3) begin
                        cmd_addr    <= init_addr[init_step];
                        cmd_data    <= init_data[init_step];
                        cmd_is_read <= 1'b0;
                        cmd_trigger <= 1'b1;
                        main_state  <= M_INIT_EXEC;
                    end else begin
                        // All 3 registers written for this reader
                        main_state <= M_INIT_NEXT;
                    end
                end

                M_INIT_EXEC: begin
                    if (cmd_done) begin
                        init_step  <= init_step + 1;
                        main_state <= M_INIT_START;
                    end
                end

                M_INIT_NEXT: begin
                    if (reader_idx == 2'd3) begin
                        // All 4 readers initialized
                        init_done  <= 1'b1;
                        reader_idx <= 2'd0;
                        main_state <= M_POLL_SELECT;
                    end else begin
                        reader_idx <= reader_idx + 1;
                        init_step  <= 4'd0;
                        // Brief CS deselect between readers
                        wait_cnt   <= 16'd0;
                        main_state <= M_CS_DESELECT;
                    end
                end

                // ─────────────────────────────────────
                // Brief wait with CS all deselected
                // ─────────────────────────────────────
                M_CS_DESELECT: begin
                    if (wait_cnt >= CS_DESELECT_WAIT) begin
                        if (init_done)
                            main_state <= M_POLL_REQA;
                        else begin
                            init_step  <= 4'd0;
                            main_state <= M_INIT_START;
                        end
                    end else begin
                        wait_cnt <= wait_cnt + 1;
                    end
                end

                // ─────────────────────────────────────
                // POLL: Round-robin poll each reader
                // ─────────────────────────────────────
                M_POLL_SELECT: begin
                    reader_idx <= 2'd0;
                    main_state <= M_POLL_REQA;
                    poll_step  <= 4'd0;
                end

                // ─── REQA Sequence ───
                // Step 0: Write CommandReg = IDLE (cancel previous)
                // Step 1: Write FIFOLevelReg = 0x80 (flush FIFO)
                // Step 2: Write FIFODataReg = REQA (0x26)
                // Step 3: Write BitFramingReg = 0x07 (7 bits)
                // Step 4: Write CommandReg = TRANSCEIVE (0x0C)
                // Step 5: Wait for response / timeout
                M_POLL_REQA: begin
                    case (poll_step)
                        4'd0: begin
                            cmd_addr <= ADDR_W_COMMAND; cmd_data <= CMD_IDLE;
                            cmd_is_read <= 1'b0; cmd_trigger <= 1'b1;
                            main_state <= M_POLL_WAIT; // wait for SPI done
                        end
                        4'd1: begin
                            cmd_addr <= ADDR_W_FIFO_LEVEL; cmd_data <= 8'h80;
                            cmd_is_read <= 1'b0; cmd_trigger <= 1'b1;
                            main_state <= M_POLL_WAIT;
                        end
                        4'd2: begin
                            cmd_addr <= ADDR_W_FIFO_DATA; cmd_data <= REQA_CMD;
                            cmd_is_read <= 1'b0; cmd_trigger <= 1'b1;
                            main_state <= M_POLL_WAIT;
                        end
                        4'd3: begin
                            cmd_addr <= ADDR_W_BIT_FRAMING; cmd_data <= 8'h87;
                            // 0x87 = StartSend bit [7] + 7 TX bits [2:0]
                            cmd_is_read <= 1'b0; cmd_trigger <= 1'b1;
                            main_state <= M_POLL_WAIT;
                        end
                        4'd4: begin
                            cmd_addr <= ADDR_W_COMMAND; cmd_data <= CMD_TRANSCEIVE;
                            cmd_is_read <= 1'b0; cmd_trigger <= 1'b1;
                            main_state <= M_POLL_WAIT;
                        end
                        4'd5: begin
                            // Poll ComIrqReg for RxIRq (bit 5)
                            cmd_addr <= ADDR_R_COM_IRQ; cmd_data <= 8'h00;
                            cmd_is_read <= 1'b1; cmd_trigger <= 1'b1;
                            main_state <= M_POLL_WAIT;
                        end
                        4'd6: begin
                            // Check response
                            if (cmd_rx_byte[5]) begin
                                // RxIRq set — tag responded to REQA
                                poll_step  <= 4'd0;
                                main_state <= M_POLL_ANTICOL;
                            end else if (timeout_cnt >= REQA_TIMEOUT) begin
                                // Timeout — no tag at this reader
                                tag_detected[reader_idx] <= 1'b0;
                                main_state <= M_POLL_NEXT;
                            end else begin
                                // Keep polling status
                                timeout_cnt <= timeout_cnt + 1;
                                poll_step   <= 4'd5;  // re-read ComIrqReg
                                main_state  <= M_POLL_REQA;
                            end
                        end
                        default: main_state <= M_POLL_NEXT;
                    endcase
                end

                // ─── Wait for SPI command to complete ───
                M_POLL_WAIT: begin
                    if (cmd_done) begin
                        poll_step  <= poll_step + 1;
                        main_state <= M_POLL_REQA;
                    end
                end

                // ─── ANTICOLLISION Sequence ───
                // Step 0: Write CommandReg = IDLE
                // Step 1: Write FIFOLevelReg = 0x80 (flush)
                // Step 2: Write FIFODataReg = SEL (0x93)
                // Step 3: Write FIFODataReg = NVB (0x20)
                // Step 4: Write BitFramingReg = 0x00 (full bytes)
                // Step 5: Write CommandReg = TRANSCEIVE
                // Step 6: Wait for RxIRq
                // Step 7: Check & start UID read
                M_POLL_ANTICOL: begin
                    case (poll_step)
                        4'd0: begin
                            cmd_addr <= ADDR_W_COMMAND; cmd_data <= CMD_IDLE;
                            cmd_is_read <= 1'b0; cmd_trigger <= 1'b1;
                            poll_step <= poll_step + 1; main_state <= M_POLL_ANTICOL;
                        end
                        4'd1: begin
                            if (cmd_done) begin
                                cmd_addr <= ADDR_W_FIFO_LEVEL; cmd_data <= 8'h80;
                                cmd_is_read <= 1'b0; cmd_trigger <= 1'b1;
                                poll_step <= poll_step + 1;
                            end
                        end
                        4'd2: begin
                            if (cmd_done) begin
                                cmd_addr <= ADDR_W_FIFO_DATA; cmd_data <= SEL_CL1;
                                cmd_is_read <= 1'b0; cmd_trigger <= 1'b1;
                                poll_step <= poll_step + 1;
                            end
                        end
                        4'd3: begin
                            if (cmd_done) begin
                                cmd_addr <= ADDR_W_FIFO_DATA; cmd_data <= NVB_FULL;
                                cmd_is_read <= 1'b0; cmd_trigger <= 1'b1;
                                poll_step <= poll_step + 1;
                            end
                        end
                        4'd4: begin
                            if (cmd_done) begin
                                cmd_addr <= ADDR_W_BIT_FRAMING; cmd_data <= 8'h80;
                                // 0x80 = StartSend, full bytes
                                cmd_is_read <= 1'b0; cmd_trigger <= 1'b1;
                                poll_step <= poll_step + 1;
                            end
                        end
                        4'd5: begin
                            if (cmd_done) begin
                                cmd_addr <= ADDR_W_COMMAND; cmd_data <= CMD_TRANSCEIVE;
                                cmd_is_read <= 1'b0; cmd_trigger <= 1'b1;
                                poll_step <= poll_step + 1;
                                timeout_cnt <= 16'd0;
                            end
                        end
                        4'd6: begin
                            // Wait for RxIRq
                            if (cmd_done) begin
                                if (cmd_rx_byte[5]) begin
                                    // Response received — read UID
                                    uid_byte_cnt <= 3'd0;
                                    uid_temp     <= 32'd0;
                                    main_state   <= M_POLL_READ_UID;
                                end else if (timeout_cnt >= REQA_TIMEOUT) begin
                                    tag_detected[reader_idx] <= 1'b0;
                                    main_state <= M_POLL_NEXT;
                                end else begin
                                    timeout_cnt <= timeout_cnt + 1;
                                    // Re-read ComIrqReg
                                    cmd_addr <= ADDR_R_COM_IRQ;
                                    cmd_is_read <= 1'b1; cmd_trigger <= 1'b1;
                                end
                            end else if (cmd_state == C_IDLE) begin
                                // First read of ComIrqReg
                                cmd_addr <= ADDR_R_COM_IRQ;
                                cmd_is_read <= 1'b1; cmd_trigger <= 1'b1;
                            end
                        end
                        default: main_state <= M_POLL_NEXT;
                    endcase
                end

                // ─── Read 4 UID bytes from FIFO ───
                M_POLL_READ_UID: begin
                    if (cmd_state == C_IDLE && !cmd_trigger) begin
                        if (uid_byte_cnt < 3'd4) begin
                            cmd_addr    <= ADDR_R_FIFO_DATA;
                            cmd_is_read <= 1'b1;
                            cmd_trigger <= 1'b1;
                        end else begin
                            // All 4 bytes read — store UID
                            tag_detected[reader_idx] <= 1'b1;
                            case (reader_idx)
                                2'd0: tag_uid_w <= uid_temp;
                                2'd1: tag_uid_e <= uid_temp;
                                2'd2: tag_uid_s <= uid_temp;
                                2'd3: tag_uid_n <= uid_temp;
                            endcase
                            main_state <= M_POLL_NEXT;
                        end
                    end else if (cmd_done) begin
                        // Accumulate UID byte
                        uid_temp <= {uid_temp[23:0], cmd_rx_byte};
                        uid_byte_cnt <= uid_byte_cnt + 1;
                    end
                end

                // ─── Move to next reader ───
                M_POLL_NEXT: begin
                    timeout_cnt <= 16'd0;
                    poll_step   <= 4'd0;
                    if (reader_idx == 2'd3) begin
                        // Completed full cycle — restart
                        reader_idx <= 2'd0;
                        wait_cnt   <= 16'd0;
                        main_state <= M_CS_DESELECT;
                    end else begin
                        reader_idx <= reader_idx + 1;
                        wait_cnt   <= 16'd0;
                        main_state <= M_CS_DESELECT;
                    end
                end

                default: main_state <= M_RESET;
            endcase
        end
    end

endmodule
