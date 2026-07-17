// =============================================================================
// Module     : rc522_driver_x4
// Project    : TLC-RFID v4.4
// Description: Polls 4 RC522 RFID readers via shared SPI bus.
//              Full INIT: soft reset → wait POR → timer config → gain → antenna.
//              Poll: REQA → ANTICOL → read 4-byte UID.
//
//              All logic runs on CLK_SYS (50 MHz).
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
    parameter [15:0] REQA_TIMEOUT    = 16'd50000;  // ~1ms @ 50MHz
    parameter [15:0] CS_DESELECT_WAIT = 16'd50;    // ~1us between readers
    parameter [23:0] POR_WAIT        = 24'd2_500_000; // 50ms Power-On-Reset wait

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
    // RC522 Register Addresses (SPI format: {R/W, addr[5:0], 0})
    //   Write: bit7=0, Read: bit7=1
    // =========================================================================
    localparam ADDR_W_COMMAND      = 8'h02;  // CommandReg      (0x01)
    localparam ADDR_W_COM_IEN      = 8'h04;  // ComIEnReg       (0x02)
    localparam ADDR_W_COM_IRQ      = 8'h08;  // ComIrqReg       (0x04)
    localparam ADDR_W_FIFO_DATA    = 8'h12;  // FIFODataReg     (0x09)
    localparam ADDR_W_FIFO_LEVEL   = 8'h14;  // FIFOLevelReg    (0x0A)
    localparam ADDR_W_BIT_FRAMING  = 8'h1A;  // BitFramingReg   (0x0D)
    localparam ADDR_W_MODE         = 8'h16;  // ModeReg         (0x0B)
    localparam ADDR_W_TX_MODE      = 8'h24;  // TxModeReg       (0x12)
    localparam ADDR_W_RX_MODE      = 8'h26;  // RxModeReg       (0x13)
    localparam ADDR_W_TX_CONTROL   = 8'h28;  // TxControlReg    (0x14)
    localparam ADDR_W_TX_ASK       = 8'h2A;  // TxASKReg        (0x15)
    localparam ADDR_W_TMODE        = 8'h4A;  // TModeReg        (0x25)
    localparam ADDR_W_TPRESCALER   = 8'h4C;  // TPrescalerReg   (0x26)
    localparam ADDR_W_TRELOAD_H    = 8'h4E;  // TReloadRegH     (0x27)
    localparam ADDR_W_TRELOAD_L    = 8'h50;  // TReloadRegL     (0x28)
    localparam ADDR_W_RF_CFG       = 8'h4C;  // Incorrect; fix below
    // RFCfgReg is at address 0x26... wait, that's RxModeReg
    // Let me recalculate: RFCfgReg = 0x26? No.
    // RC522 registers: RFCfgReg = 0x26 in datasheet? Let me check.
    // Actually: RFCfgReg = register address 0x26 → SPI write = {0, 100110, 0} = 0x4C
    // But TPrescalerReg is also 0x4C... That's wrong.
    // Let me recalculate properly:
    //   TModeReg     = 0x2A → SPI: {0, 101010, 0} = 0101_0100 = 0x54  ← FIX
    //   TPrescalerReg= 0x2B → SPI: {0, 101011, 0} = 0101_0110 = 0x56  ← FIX
    //   TReloadRegH  = 0x2C → SPI: {0, 101100, 0} = 0101_1000 = 0x58  ← FIX
    //   TReloadRegL  = 0x2D → SPI: {0, 101101, 0} = 0101_1010 = 0x5A  ← FIX
    //   RFCfgReg     = 0x26 → SPI: {0, 100110, 0} = 0100_1100 = 0x4C
    // Hmm, wait. Let me re-derive all addresses carefully.
    // RC522 SPI address byte = {(read?1:0), addr[5], addr[4], addr[3], addr[2], addr[1], addr[0], 0}

    // ─── CORRECTED Register Addresses ───
    // Reg 0x01 CommandReg:    W={0,000001,0}=0x02  R={1,000001,0}=0x82
    // Reg 0x02 ComIEnReg:     W={0,000010,0}=0x04
    // Reg 0x04 ComIrqReg:     W={0,000100,0}=0x08  R={1,000100,0}=0x88
    // Reg 0x09 FIFODataReg:   W={0,001001,0}=0x12  R={1,001001,0}=0x92
    // Reg 0x0A FIFOLevelReg:  W={0,001010,0}=0x14  R={1,001010,0}=0x94
    // Reg 0x0B ModeReg:       W={0,001011,0}=0x16
    // Reg 0x0D BitFramingReg: W={0,001101,0}=0x1A
    // Reg 0x11 TxASKReg:      W={0,010001,0}=0x22  ← CORRECTED
    // Reg 0x12 TxModeReg:     W={0,010010,0}=0x24
    // Reg 0x13 RxModeReg:     W={0,010011,0}=0x26
    // Reg 0x14 TxControlReg:  W={0,010100,0}=0x28
    // Reg 0x26 RFCfgReg:      W={0,100110,0}=0x4C
    // Reg 0x2A TModeReg:      W={0,101010,0}=0x54
    // Reg 0x2B TPrescalerReg: W={0,101011,0}=0x56
    // Reg 0x2C TReloadRegH:   W={0,101100,0}=0x58
    // Reg 0x2D TReloadRegL:   W={0,101101,0}=0x5A

    // Read addresses
    localparam ADDR_R_FIFO_DATA  = 8'h92;  // FIFODataReg  (0x09)
    localparam ADDR_R_COM_IRQ    = 8'h88;  // ComIrqReg    (0x04)

    // RC522 Commands
    localparam CMD_IDLE       = 8'h00;
    localparam CMD_SOFT_RESET = 8'h0F;
    localparam CMD_TRANSCEIVE = 8'h0C;
    localparam REQA_CMD       = 8'h26;  // REQA (7-bit, ISO 14443-3)
    localparam SEL_CL1        = 8'h93;  // SELECT Cascade Level 1
    localparam NVB_FULL       = 8'h20;  // 2 bytes sent (NVB for ANTICOL)

    // =========================================================================
    // Main FSM States
    // =========================================================================
    localparam M_POR_WAIT     = 4'd0;
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
    localparam M_ANTICOL_WAIT = 4'd11;

    // SPI Command Sub-FSM States
    localparam C_IDLE      = 3'd0;
    localparam C_WAIT_ADDR = 3'd1;
    localparam C_WAIT_DATA = 3'd2;
    localparam C_DONE      = 3'd3;

    // =========================================================================
    // Internal Registers
    // =========================================================================
    reg [3:0]  main_state;
    reg [2:0]  cmd_state;
    reg [1:0]  reader_idx;         // Current reader 0-3
    reg        init_done;
    reg [3:0]  init_step;          // Step within INIT sequence
    reg [3:0]  poll_step;          // Step within POLL sequence
    reg [15:0] timeout_cnt;
    reg [15:0] wait_cnt;           // Generic wait counter
    reg [23:0] por_cnt;            // POR wait counter
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
    // INIT Sequence ROM — Full RC522 initialization
    // Writes per reader: soft_reset, then config registers
    // =========================================================================
    // Step 0:  CommandReg     = 0x0F  (Soft Reset)
    // Step 1:  TModeReg       = 0x8D  (TAuto=1, TPrescaler high bits)
    // Step 2:  TPrescalerReg  = 0x3E  (TPrescaler=0xD3E → ~25ms timeout)
    // Step 3:  TReloadRegH    = 0x00  (Timer reload = 30)
    // Step 4:  TReloadRegL    = 0x1E
    // Step 5:  TxASKReg       = 0x40  (Force 100% ASK modulation)
    // Step 6:  ModeReg        = 0x3D  (CRC preset 6363h, Tx/Rx timing)
    // Step 7:  TxModeReg      = 0x00  (Reset value)
    // Step 8:  RxModeReg      = 0x00  (Reset value)
    // Step 9:  RFCfgReg       = 0x70  (Receiver gain = 48 dB, max)
    // Step 10: TxControlReg   = 0x03  (Enable TX1 and TX2 antenna drivers)
    // Step 11: ComIrqReg      = 0x7F  (Clear all interrupts)
    localparam INIT_STEPS = 4'd12;

    reg [7:0] init_rom_addr;
    reg [7:0] init_rom_data;

    always @(*) begin
        case (init_step)
            4'd0:  begin init_rom_addr = 8'h02; init_rom_data = CMD_SOFT_RESET; end  // CommandReg = SoftReset
            4'd1:  begin init_rom_addr = 8'h54; init_rom_data = 8'h8D; end  // TModeReg
            4'd2:  begin init_rom_addr = 8'h56; init_rom_data = 8'h3E; end  // TPrescalerReg
            4'd3:  begin init_rom_addr = 8'h58; init_rom_data = 8'h00; end  // TReloadRegH
            4'd4:  begin init_rom_addr = 8'h5A; init_rom_data = 8'h1E; end  // TReloadRegL
            4'd5:  begin init_rom_addr = 8'h22; init_rom_data = 8'h40; end  // TxASKReg
            4'd6:  begin init_rom_addr = 8'h16; init_rom_data = 8'h3D; end  // ModeReg
            4'd7:  begin init_rom_addr = 8'h24; init_rom_data = 8'h00; end  // TxModeReg
            4'd8:  begin init_rom_addr = 8'h26; init_rom_data = 8'h00; end  // RxModeReg
            4'd9:  begin init_rom_addr = 8'h4C; init_rom_data = 8'h70; end  // RFCfgReg (max gain)
            4'd10: begin init_rom_addr = 8'h28; init_rom_data = 8'h03; end  // TxControlReg (antenna on)
            4'd11: begin init_rom_addr = 8'h08; init_rom_data = 8'h7F; end  // ComIrqReg (clear all)
            default: begin init_rom_addr = 8'h02; init_rom_data = 8'h00; end
        endcase
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
        if (main_state == M_POR_WAIT || main_state == M_POLL_SELECT ||
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
            main_state   <= M_POR_WAIT;
            reader_idx   <= 2'd0;
            init_done    <= 1'b0;
            init_step    <= 4'd0;
            poll_step    <= 4'd0;
            timeout_cnt  <= 16'd0;
            wait_cnt     <= 16'd0;
            por_cnt      <= 24'd0;
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
                // Wait 50ms after power-on for RC522 oscillator startup
                // ─────────────────────────────────────
                M_POR_WAIT: begin
                    if (por_cnt >= POR_WAIT) begin
                        reader_idx <= 2'd0;
                        init_step  <= 4'd0;
                        main_state <= M_INIT_START;
                    end else begin
                        por_cnt <= por_cnt + 1;
                    end
                end

                // ─────────────────────────────────────
                // INIT: Write 12 registers per reader, 4 readers total
                // ─────────────────────────────────────
                M_INIT_START: begin
                    if (init_step < INIT_STEPS) begin
                        cmd_addr    <= init_rom_addr;
                        cmd_data    <= init_rom_data;
                        cmd_is_read <= 1'b0;
                        cmd_trigger <= 1'b1;
                        main_state  <= M_INIT_EXEC;
                    end else begin
                        // All registers written for this reader
                        main_state <= M_INIT_NEXT;
                    end
                end

                M_INIT_EXEC: begin
                    if (cmd_done) begin
                        // After soft reset (step 0), wait extra time
                        if (init_step == 4'd0) begin
                            wait_cnt   <= 16'd0;
                            init_step  <= init_step + 1;
                            main_state <= M_CS_DESELECT;  // reuse wait state for POR delay
                        end else begin
                            init_step  <= init_step + 1;
                            main_state <= M_INIT_START;
                        end
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
                    poll_step  <= 4'd0;
                    main_state <= M_POLL_REQA;
                end

                // ─── REQA Sequence ───
                // Step 0: Write CommandReg = IDLE (cancel previous)
                // Step 1: Write ComIrqReg = 0x7F (clear all interrupts)
                // Step 2: Write FIFOLevelReg = 0x80 (flush FIFO)
                // Step 3: Write FIFODataReg = REQA (0x26)
                // Step 4: Write BitFramingReg = 0x07 (7 TX bits, NO StartSend yet)
                // Step 5: Write CommandReg = TRANSCEIVE (0x0C)
                // Step 6: Write BitFramingReg = 0x87 (set StartSend bit)
                // Step 7: Read ComIrqReg — poll for RxIRq
                // Step 8: Check response
                M_POLL_REQA: begin
                    case (poll_step)
                        4'd0: begin
                            cmd_addr <= 8'h02; cmd_data <= CMD_IDLE;
                            cmd_is_read <= 1'b0; cmd_trigger <= 1'b1;
                            main_state <= M_POLL_WAIT;
                        end
                        4'd1: begin
                            // Clear all interrupt flags
                            cmd_addr <= 8'h08; cmd_data <= 8'h7F;
                            cmd_is_read <= 1'b0; cmd_trigger <= 1'b1;
                            main_state <= M_POLL_WAIT;
                        end
                        4'd2: begin
                            // Flush FIFO
                            cmd_addr <= 8'h14; cmd_data <= 8'h80;
                            cmd_is_read <= 1'b0; cmd_trigger <= 1'b1;
                            main_state <= M_POLL_WAIT;
                        end
                        4'd3: begin
                            // Write REQA command to FIFO
                            cmd_addr <= 8'h12; cmd_data <= REQA_CMD;
                            cmd_is_read <= 1'b0; cmd_trigger <= 1'b1;
                            main_state <= M_POLL_WAIT;
                        end
                        4'd4: begin
                            // Set BitFramingReg = 0x07 (7 TX bits, StartSend=0)
                            cmd_addr <= 8'h1A; cmd_data <= 8'h07;
                            cmd_is_read <= 1'b0; cmd_trigger <= 1'b1;
                            main_state <= M_POLL_WAIT;
                        end
                        4'd5: begin
                            // Start TRANSCEIVE command
                            cmd_addr <= 8'h02; cmd_data <= CMD_TRANSCEIVE;
                            cmd_is_read <= 1'b0; cmd_trigger <= 1'b1;
                            main_state <= M_POLL_WAIT;
                        end
                        4'd6: begin
                            // NOW set StartSend bit: BitFramingReg = 0x87
                            cmd_addr <= 8'h1A; cmd_data <= 8'h87;
                            cmd_is_read <= 1'b0; cmd_trigger <= 1'b1;
                            timeout_cnt <= 16'd0;
                            main_state <= M_POLL_WAIT;
                        end
                        4'd7: begin
                            // Read ComIrqReg to check RxIRq
                            cmd_addr <= ADDR_R_COM_IRQ; cmd_data <= 8'h00;
                            cmd_is_read <= 1'b1; cmd_trigger <= 1'b1;
                            main_state <= M_POLL_WAIT;
                        end
                        4'd8: begin
                            // Evaluate response
                            if (cmd_rx_byte[5]) begin
                                // RxIRq set — tag responded to REQA!
                                poll_step  <= 4'd0;
                                main_state <= M_POLL_ANTICOL;
                            end else if (timeout_cnt >= REQA_TIMEOUT) begin
                                // Timeout — no tag at this reader
                                tag_detected[reader_idx] <= 1'b0;
                                main_state <= M_POLL_NEXT;
                            end else begin
                                // Keep polling ComIrqReg
                                timeout_cnt <= timeout_cnt + 1;
                                poll_step   <= 4'd7;  // re-read ComIrqReg
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
                // Step 1: Write ComIrqReg = 0x7F (clear interrupts)
                // Step 2: Write FIFOLevelReg = 0x80 (flush)
                // Step 3: Write FIFODataReg = SEL (0x93)
                // Step 4: Write FIFODataReg = NVB (0x20)
                // Step 5: Write BitFramingReg = 0x00 (full bytes, no StartSend)
                // Step 6: Write CommandReg = TRANSCEIVE
                // Step 7: Write BitFramingReg = 0x80 (set StartSend)
                // Step 8: Read ComIrqReg
                // Step 9: Check & start UID read
                M_POLL_ANTICOL: begin
                    case (poll_step)
                        4'd0: begin
                            cmd_addr <= 8'h02; cmd_data <= CMD_IDLE;
                            cmd_is_read <= 1'b0; cmd_trigger <= 1'b1;
                            main_state <= M_ANTICOL_WAIT;
                        end
                        4'd1: begin
                            // Clear interrupts
                            cmd_addr <= 8'h08; cmd_data <= 8'h7F;
                            cmd_is_read <= 1'b0; cmd_trigger <= 1'b1;
                            main_state <= M_ANTICOL_WAIT;
                        end
                        4'd2: begin
                            // Flush FIFO
                            cmd_addr <= 8'h14; cmd_data <= 8'h80;
                            cmd_is_read <= 1'b0; cmd_trigger <= 1'b1;
                            main_state <= M_ANTICOL_WAIT;
                        end
                        4'd3: begin
                            // Write SEL to FIFO
                            cmd_addr <= 8'h12; cmd_data <= SEL_CL1;
                            cmd_is_read <= 1'b0; cmd_trigger <= 1'b1;
                            main_state <= M_ANTICOL_WAIT;
                        end
                        4'd4: begin
                            // Write NVB to FIFO
                            cmd_addr <= 8'h12; cmd_data <= NVB_FULL;
                            cmd_is_read <= 1'b0; cmd_trigger <= 1'b1;
                            main_state <= M_ANTICOL_WAIT;
                        end
                        4'd5: begin
                            // BitFramingReg = 0x00 (full bytes)
                            cmd_addr <= 8'h1A; cmd_data <= 8'h00;
                            cmd_is_read <= 1'b0; cmd_trigger <= 1'b1;
                            main_state <= M_ANTICOL_WAIT;
                        end
                        4'd6: begin
                            // Start TRANSCEIVE
                            cmd_addr <= 8'h02; cmd_data <= CMD_TRANSCEIVE;
                            cmd_is_read <= 1'b0; cmd_trigger <= 1'b1;
                            main_state <= M_ANTICOL_WAIT;
                        end
                        4'd7: begin
                            // Set StartSend
                            cmd_addr <= 8'h1A; cmd_data <= 8'h80;
                            cmd_is_read <= 1'b0; cmd_trigger <= 1'b1;
                            timeout_cnt <= 16'd0;
                            main_state <= M_ANTICOL_WAIT;
                        end
                        4'd8: begin
                            // Read ComIrqReg
                            cmd_addr <= ADDR_R_COM_IRQ; cmd_data <= 8'h00;
                            cmd_is_read <= 1'b1; cmd_trigger <= 1'b1;
                            main_state <= M_ANTICOL_WAIT;
                        end
                        4'd9: begin
                            // Check response
                            if (cmd_rx_byte[5]) begin
                                // Response received — read UID from FIFO
                                uid_byte_cnt <= 3'd0;
                                uid_temp     <= 32'd0;
                                main_state   <= M_POLL_READ_UID;
                            end else if (timeout_cnt >= REQA_TIMEOUT) begin
                                tag_detected[reader_idx] <= 1'b0;
                                main_state <= M_POLL_NEXT;
                            end else begin
                                timeout_cnt <= timeout_cnt + 1;
                                poll_step   <= 4'd8;  // re-read ComIrqReg
                                main_state  <= M_POLL_ANTICOL;
                            end
                        end
                        default: main_state <= M_POLL_NEXT;
                    endcase
                end

                // ─── Wait for ANTICOL SPI command to complete ───
                M_ANTICOL_WAIT: begin
                    if (cmd_done) begin
                        poll_step  <= poll_step + 1;
                        main_state <= M_POLL_ANTICOL;
                    end
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
                        // Accumulate UID byte (MSB first)
                        uid_temp <= {uid_temp[23:0], cmd_rx_byte};
                        uid_byte_cnt <= uid_byte_cnt + 1;
                    end
                end

                // ─── Move to next reader ───
                M_POLL_NEXT: begin
                    timeout_cnt <= 16'd0;
                    poll_step   <= 4'd0;
                    if (reader_idx == 2'd3) begin
                        // Completed full cycle — restart from reader 0
                        reader_idx <= 2'd0;
                        wait_cnt   <= 16'd0;
                        main_state <= M_CS_DESELECT;
                    end else begin
                        reader_idx <= reader_idx + 1;
                        wait_cnt   <= 16'd0;
                        main_state <= M_CS_DESELECT;
                    end
                end

                default: main_state <= M_POR_WAIT;
            endcase
        end
    end

endmodule
