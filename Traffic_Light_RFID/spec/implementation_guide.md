# Hướng Dẫn Code Từng Module — TLC-RFID v4.4

> [!IMPORTANT]
> Thứ tự code: leaf modules trước → tích hợp sau. Mỗi module code xong nên test riêng trước khi đi tiếp.

---

## Module 1: `rst_sync.v` — Reset Synchronizer

**Mục đích**: Chuyển tín hiệu reset async (nút nhấn) thành sync release trên `clk`.

**Độ khó**: ⭐ (đã có code trong spec Sec 3.3)

**Cách làm**:
- Copy code từ Sec 3.3 của spec ra file `rst_sync.v`
- 2 flip-flop cascade: `rst_meta` → `rst_sync_n`
- Sensitivity list: `posedge clk or negedge rst_async_n`
- Khi `rst_async_n = 0` (nút nhấn): cả 2 FF → 0 ngay lập tức (async assert)
- Khi `rst_async_n = 1` (nhả nút): `rst_meta` → 1 ở clock edge đầu, `rst_sync_n` → 1 ở clock edge sau (sync release, trễ 2 cycle)

**Logic cốt lõi**:
```
if (!rst_async_n)
    rst_meta    <= 0
    rst_sync_n  <= 0
else
    rst_meta    <= 1
    rst_sync_n  <= rst_meta    // pipeline 2 stage
```

**Verify nhanh**: Trong testbench, kéo `rst_async_n` xuống 0 ở thời điểm bất kỳ (giữa clock edge), kiểm tra `rst_sync_n` deassert đúng 2 rising edge sau khi nhả `rst_async_n`.

---

## Module 2: `phase_timer.v` — 32-bit Countdown Timer

**Mục đích**: Đếm ngược từ giá trị nạp, pulse `expired` khi chạm 0.

**Độ khó**: ⭐

**Cách làm**:

1. Khai báo `reg [31:0] counter`
2. Logic:

```
always @(posedge clk) begin
    if (!rst_n)
        counter <= 0
    else if (load)
        counter <= value          // nạp giá trị mới
    else if (counter != 0)
        counter <= counter - 1    // đếm xuống
end
```

3. Output expired — **2 cách**, chọn 1:

```
// Cách 1: Combinational (đơn giản, nhưng expired = 1 liên tục khi counter = 0)
assign expired = (counter == 32'd0);

// Cách 2: Pulse 1 cycle (chính xác hơn theo spec)
// expired = 1 đúng 1 cycle khi counter chuyển từ 1 → 0
assign expired = (counter == 32'd1) && !load;
// Hoặc dùng registered version:
// Detect transition: counter was != 0 last cycle, now == 0
```

> [!TIP]
> **Khuyến nghị dùng Cách 2** — pulse 1 cycle. FSM sẽ dùng `expired` như trigger chuyển state, nếu nó HIGH liên tục thì FSM có thể skip state.
> 
> Trick đơn giản nhất: `assign expired = (counter == 32'd1);` — vì counter giảm mỗi cycle, nên giá trị 1 chỉ tồn tại đúng 1 cycle trước khi về 0. Nhưng nhớ thêm điều kiện `&& !load` để tránh false trigger khi load value = 1.

**Lưu ý `CNT_PRIO_TMO`**: Giá trị 3,000,000,000 > 2³¹. Verilog literal `32'd3000000000` sẽ hoạt động đúng vì `reg [31:0]` là unsigned by default. Nhưng nếu bạn khai báo parameter là `integer` thì nó sẽ là signed → sai. Luôn dùng:
```
parameter [31:0] CNT_PRIO_TMO = 32'd3_000_000_000;
```

**Test**: Load value = 5, kiểm tra counter: 5→4→3→2→1→0, `expired` pulse đúng 1 cycle.

---

## Module 3: `tag_priority_lut.v` — UID → Priority Lookup

**Mục đích**: Tra bảng UID → `priority_class[2:0]`.

**Độ khó**: ⭐

**Cách làm**:

```
// Pure combinational — không cần clk, rst
module tag_priority_lut (
    input  wire [31:0] uid,
    output reg  [2:0]  priority_class
);

always @(*) begin
    case (uid)
        32'hA1B2C3D4: priority_class = 3'b101;  // Tier 5 - Chữa cháy
        32'hE5F6A7B8: priority_class = 3'b100;  // Tier 4 - Quân sự/Công an
        32'h12345678: priority_class = 3'b011;  // Tier 3 - Cứu thương
        32'h9ABCDEF0: priority_class = 3'b010;  // Tier 2 - Hộ đê/thiên tai
        32'h11223344: priority_class = 3'b001;  // Tier 1 - Xe tang
        default:      priority_class = 3'b000;  // Không ưu tiên
    endcase
end

endmodule
```

**Lưu ý**: Module này sẽ được instantiate 4 lần trong `traffic_node.v` (1 per reader direction).

**Test**: Không cần testbench riêng — verify bằng mắt hoặc inline trong testbench tổng.

---

## Module 4: `cdc_sync.v` — 2-FF Synchronizer (Utility)

**Mục đích**: Generic CDC synchronizer. Không dùng trong design hiện tại nhưng giữ lại.

**Độ khó**: ⭐

**Cách làm**:

```
module cdc_sync #(parameter W = 1) (
    input  wire         clk_dst,
    input  wire         rst_n,
    input  wire [W-1:0] d,
    output reg  [W-1:0] q
);

reg [W-1:0] meta;

always @(posedge clk_dst or negedge rst_n) begin
    if (!rst_n) begin
        meta <= {W{1'b0}};
        q    <= {W{1'b0}};
    end else begin
        meta <= d;
        q    <= meta;
    end
end

endmodule
```

> [!WARNING]
> Chỉ dùng W=1 cho 1-bit signal (valid, strobe). **KHÔNG** dùng cho multi-bit data bus — multi-bit cần handshake protocol hoặc async FIFO.

---

## Module 5: `spi_master.v` — SPI Mode 0 Controller (Reuse)

**Mục đích**: SPI full-duplex 8-bit, Mode 0.

**Độ khó**: ⭐⭐ (reuse từ ECG, chỉ cần review)

**Nếu cần code lại**, đây là logic:

**Registers cần có**:
- `reg [7:0] shift_reg` — shift register cho TX/RX
- `reg [2:0] bit_cnt` — đếm 0→7 (8 bit)
- `reg [2:0] clk_cnt` — đếm clock divider
- `reg sck_internal` — SPI clock toggle
- `reg running` — đang trong transfer

**FSM đơn giản** — 2 state: IDLE, TRANSFER

**IDLE**:
- `spi_sck = 0` (CPOL=0)
- Khi `start = 1`: latch `shift_reg = tx_data`, `bit_cnt = 7`, chuyển TRANSFER

**TRANSFER**:
- Đếm `clk_cnt` từ 0 đến `clk_div`
- Khi `clk_cnt` hết:
  - **Rising edge** (`sck 0→1`): Sample MISO → `shift_reg[0]`, tăng sck
  - **Falling edge** (`sck 1→0`): Shift out MSB → MOSI, shift trái, giảm sck
- Khi `bit_cnt = 0` và hoàn thành cả rising + falling: `done = 1`, `rx_data = shift_reg`, về IDLE

**Timing Mode 0 (CPOL=0, CPHA=0)**:
```
SCK:    ___/‾‾‾\___/‾‾‾\___/‾‾‾\___
MOSI:   X[D7]------[D6]------[D5]---
MISO:   -----[D7]------[D6]------[D5]
             ↑ sample   ↑ sample
```
- Data thay đổi trên **falling edge** (hoặc trước rising edge đầu tiên)
- Data sample trên **rising edge**

**Tham số clock**: `clk_div = 4` → SPI freq = 50MHz / (2 × (4+1)) = 5 MHz. Kiểm tra lại formula trong code ECG của bạn có khớp spec không.

> [!IMPORTANT]
> Module này **KHÔNG quản lý CS**. CS do `rc522_driver_x4` điều khiển riêng. `spi_master` chỉ shift 8 bit rồi báo `done`.

---

## Module 6: `uart_tx_core.v` — UART Transmitter (Reuse)

**Mục đích**: Truyền 1 byte 8N1 @ 9600 bps.

**Độ khó**: ⭐⭐ (reuse từ ECG)

**Nếu cần code lại**:

**Tham số**: `CLKS_PER_BIT = 50_000_000 / 9600 = 5208`

**FSM**: IDLE → START → DATA → STOP → DONE

```
IDLE:
    tx_serial = 1 (idle high)
    Khi tx_start = 1: latch tx_data, baud_cnt = 0, chuyển START

START:
    tx_serial = 0 (start bit)
    Đếm baud_cnt đến CLKS_PER_BIT-1, chuyển DATA, bit_idx = 0

DATA:
    tx_serial = tx_data_reg[bit_idx]   // LSB first
    Đếm baud_cnt đến CLKS_PER_BIT-1
    bit_idx++, nếu bit_idx = 8 → chuyển STOP

STOP:
    tx_serial = 1 (stop bit)
    Đếm baud_cnt đến CLKS_PER_BIT-1
    tx_done = 1 (pulse), chuyển IDLE
```

**Lưu ý**: `tx_busy = 1` từ khi nhận `tx_start` đến hết STOP bit. `cascade_tx` sẽ dùng `tx_busy` để biết khi nào gửi byte tiếp theo.

---

## Module 7: `uart_rx_core.v` — UART Receiver

**Mục đích**: Nhận 1 byte 8N1 @ 9600 bps, 16× oversampling.

**Độ khó**: ⭐⭐⭐

**Đây là module code mới** — cần cẩn thận.

**Tham số**:
```
CLKS_PER_BIT = 5208          // 50MHz / 9600
HALF_BIT     = 5208 / 2 = 2604   // để sample giữa bit
```

**FSM**: IDLE → START → DATA → STOP

**Logic chi tiết**:

```
IDLE:
    Detect falling edge trên rx_serial (1→0 = start bit bắt đầu)
    Cần register rx_prev để detect: falling = rx_prev & ~rx_serial
    Khi falling edge: baud_cnt = 0, chuyển START

START:
    Đếm baud_cnt đến HALF_BIT (= giữa start bit)
    Sample rx_serial tại giữa start bit
    Nếu rx_serial = 0: đúng là start bit → reset baud_cnt, chuyển DATA, bit_idx = 0
    Nếu rx_serial = 1: false start → quay về IDLE

DATA:
    Đếm baud_cnt đến CLKS_PER_BIT (= 1 full bit period từ mid-bit trước)
    Sample rx_serial → rx_data_reg[bit_idx]   // LSB first
    bit_idx++
    Nếu bit_idx = 8: chuyển STOP, reset baud_cnt

STOP:
    Đếm baud_cnt đến CLKS_PER_BIT
    Sample rx_serial tại giữa stop bit
    Nếu rx_serial = 1: rx_valid = 1, rx_data = rx_data_reg  (hợp lệ)
    Nếu rx_serial = 0: rx_error = 1  (framing error - stop bit không phải 1)
    Quay về IDLE
```

> [!TIP]
> **16× oversampling** trong spec nghĩa là sample ở giữa bit để tránh sampling ở transition edge. Cách đơn giản nhất: dùng `HALF_BIT` để align vào giữa start bit, sau đó cứ mỗi `CLKS_PER_BIT` sample 1 lần — tự động rơi vào giữa mỗi data bit.
>
> Nếu muốn robust hơn: dùng majority voting (sample 3 lần gần giữa bit, lấy đa số). Nhưng cho prototype thì mid-bit sampling là đủ.

**Test**: Tạo task trong testbench gửi 1 byte:
```
task send_byte(input [7:0] data);
    integer i;
    begin
        rx_serial = 0;                    // start bit
        #(CLKS_PER_BIT * CLK_PERIOD);
        for (i = 0; i < 8; i = i + 1) begin
            rx_serial = data[i];          // LSB first
            #(CLKS_PER_BIT * CLK_PERIOD);
        end
        rx_serial = 1;                    // stop bit
        #(CLKS_PER_BIT * CLK_PERIOD);
    end
endtask
```

---

## Module 8: `rc522_driver_x4.v` — 4-Reader RFID Driver

**Mục đích**: Poll 4 RC522 qua shared SPI, output `tag_detected[3:0]` và 4× `tag_uid`.

**Độ khó**: ⭐⭐⭐⭐⭐ (phức tạp nhất trong project)

### Kiến trúc nội bộ

```
rc522_driver_x4
├── spi_master (instance)           // 1 instance SPI, chia sẻ
├── FSM chính (main_state)          // Điều phối INIT và POLL
└── FSM con (cmd_state)             // Thực hiện SPI command sequence
```

### Registers cần có

```
reg [3:0] main_state;          // FSM chính
reg [3:0] cmd_state;           // FSM con (SPI command sequence)
reg [1:0] reader_idx;           // Reader đang xử lý (0-3)
reg       init_done;            // Flag: INIT đã xong
reg [3:0] tag_detected_reg;     // Internal register
reg [31:0] uid_reg [0:3];       // 4 UID registers (mỗi reader 1 UID)
reg [2:0] byte_cnt;             // Đếm byte trong UID (0-3)
reg [7:0] spi_tx_byte;          // Byte chuẩn bị gửi SPI
reg [15:0] timeout_cnt;         // Timeout cho REQA response
```

### Main FSM

```
S_IDLE:
    init_done = 0, reader_idx = 0
    → S_INIT_READER

S_INIT_READER:
    cs_n = ~(1 << reader_idx)     // activate current reader
    Gọi cmd sequence: Write TxModeReg (addr 0x12, data 0x00)
                      Write RxModeReg (addr 0x13, data 0x00)
                      Write TxControlReg (addr 0x14, data 0x03)
    Mỗi write = 2 SPI transfer: [addr_byte] [data_byte]
    Khi cmd sequence done:
        reader_idx++
        Nếu reader_idx = 3 done → init_done = 1, → S_POLL_SELECT
        Ngược lại → lặp S_INIT_READER

S_POLL_SELECT:
    reader_idx = 0
    → S_POLL_REQA

S_POLL_REQA:
    cs_n = ~(1 << reader_idx)
    Gửi REQA command qua SPI:
        Write CommandReg (0x01) = 0x0C (Transceive)
        Write FIFODataReg (0x09) = 0x26 (REQA)
        Write BitFramingReg (0x0D) = 0x07 (7 bits)
        Write CommandReg (0x01) = 0x0C (start transceive)
    Đợi response: read Status1Reg, check RxDataReady bit
    Timeout ~1ms (50,000 cycles):
        Nếu timeout: tag_detected[reader_idx] = 0, → S_POLL_NEXT
        Nếu response OK: → S_POLL_ANTICOL

S_POLL_ANTICOL:
    Gửi ANTICOLLISION command:
        Write FIFODataReg = 0x93 (SEL)
        Write FIFODataReg = 0x20 (NVB = 0x20 = full frame)
        Start transceive
    Read 4 bytes UID từ FIFODataReg:
        uid_reg[reader_idx] = {byte3, byte2, byte1, byte0}
    tag_detected[reader_idx] = 1
    → S_POLL_NEXT

S_POLL_NEXT:
    cs_n = 4'b1111 (deassert all)
    reader_idx++
    Nếu reader_idx overflow (0 lại) → S_POLL_SELECT (new cycle)
    Ngược lại → S_POLL_REQA
```

### SPI Command Sub-FSM

Mỗi RC522 SPI transaction = 2 bytes:
- **Write**: Byte 1 = `{0, addr[5:0], 0}`, Byte 2 = data
- **Read**: Byte 1 = `{1, addr[5:0], 0}`, Byte 2 = 0x00 (dummy), response ở `rx_data`

```
CMD_IDLE:
    Đợi trigger từ main FSM

CMD_SEND_ADDR:
    spi_tx_data = addr_byte
    spi_start = 1
    Đợi spi_done → CMD_SEND_DATA

CMD_SEND_DATA:
    spi_tx_data = data_byte (write) hoặc 0x00 (read)
    spi_start = 1
    Đợi spi_done → lưu rx_data nếu read → CMD_DONE

CMD_DONE:
    Báo main FSM xong 1 SPI transaction
    → CMD_IDLE
```

> [!WARNING]
> **Đây là module khó nhất**. Một số tips:
> - Đừng cố code hết 1 lần. Code INIT trước, test INIT bằng cách xem SPI waveform trên ModelSim.
> - RC522 SPI address format: bit 7 = R/W (1=read, 0=write), bit 6:1 = address, bit 0 = 0.
> - Giữa 2 SPI transfer cùng 1 reader: CS vẫn LOW. CS chỉ HIGH khi chuyển reader.
> - `tag_uid` output: map từ `uid_reg[0]` → `tag_uid_w`, `uid_reg[1]` → `tag_uid_e`, `uid_reg[2]` → `tag_uid_s`, `uid_reg[3]` → `tag_uid_n`. Thứ tự CS bit: `[3]=N [2]=S [1]=E [0]=W`.

> [!TIP]
> **Đơn giản hóa cho prototype**: Nếu RC522 command sequence quá phức tạp, có thể hardcode toàn bộ SPI byte sequence thành một ROM array rồi dùng pointer chạy tuần tự. Ví dụ:
> ```
> // INIT sequence cho 1 reader: 6 bytes (3 writes × 2 bytes)
> reg [7:0] init_rom [0:5];
> initial begin
>     init_rom[0] = 8'h24;  // TxModeReg addr (write)
>     init_rom[1] = 8'h00;  // data
>     init_rom[2] = 8'h26;  // RxModeReg addr
>     init_rom[3] = 8'h00;
>     init_rom[4] = 8'h28;  // TxControlReg addr
>     init_rom[5] = 8'h03;
> end
> ```

---

## Module 9: `direction_detector.v` — Direction Inference

**Mục đích**: Xác định hướng xe đi dựa trên thứ tự reader mất tín hiệu.

**Độ khó**: ⭐⭐⭐

### Registers cần có

```
reg [1:0] state;                    // IDLE, TRACKING, DECIDE
reg [3:0] prev_tag;                 // tag_detected ở cycle trước (detect falling edge)
reg [31:0] tick_counter;            // Monotonic counter (tăng liên tục)
reg [31:0] lost_time [0:3];         // Thời điểm mất tín hiệu của reader i
reg [3:0] seen_mask;                // Bit i = 1 nếu reader i đã từng thấy tag
reg [2:0] lost_count;               // Số reader đã mất tín hiệu
reg timeout_flag;                   // = 1 nếu bị timeout
```

### Logic chi tiết

```
// Falling edge detection — mỗi cycle
wire [3:0] falling = prev_tag & ~tag_detected;   // bit i = 1 khi reader i vừa mất tag

// Tick counter — chạy liên tục
always @(posedge clk)
    if (!rst_n) tick_counter <= 0;
    else tick_counter <= tick_counter + 1;

// State machine
IDLE:
    Nếu any_detected = 1 (có reader thấy tag):
        Reset lost_time[0..3] = 0
        seen_mask = tag_detected   // ghi nhận reader nào đang thấy
        timeout_cnt = 0
        → TRACKING

TRACKING:
    // Cập nhật seen_mask: bất kỳ reader nào từng thấy tag
    seen_mask = seen_mask | tag_detected

    // Detect falling edge cho từng reader
    for i = 0 to 3:
        if falling[i]:
            lost_time[i] = tick_counter
            lost_count++

    // Transition conditions:
    if (tag_detected == 4'b0000):       // tất cả reader đều mất → DECIDE
        → DECIDE
    else if (timeout_cnt >= CNT_DIR_TIMEOUT):  // 10s timeout
        timeout_flag = 1
        → DECIDE
    else:
        timeout_cnt++

DECIDE (1 cycle):
    // Tìm reader có lost_time lớn nhất = mất cuối cùng = hướng xe đi
    // So sánh lost_time[3], [2], [1], [0]
    // Tie-break: N(3) > E(1) > S(2) > W(0)
    
    dir_out = index có lost_time max
    dir_valid = 1   // pulse 1 cycle
    dir_confidence = (popcount(seen_mask) >= 2) && !timeout_flag ? 1 : 0
    → IDLE
```

### Tìm argmax — Cách code

```
// Combinational — tìm reader mất cuối cùng
// Ưu tiên N > E > S > W khi tie
reg [1:0] max_idx;
reg [31:0] max_val;

always @(*) begin
    max_idx = 2'b11;  // W = default thấp nhất
    max_val = lost_time[0];  // W

    // So sánh S (index 2 in tag_detected, but dir encoding 01)
    if (lost_time[1] >= max_val) begin  // E
        max_val = lost_time[1];
        max_idx = 2'b10;  // EAST
    end
    if (lost_time[2] >= max_val) begin  // S
        max_val = lost_time[2];
        max_idx = 2'b01;  // SOUTH
    end
    if (lost_time[3] >= max_val) begin  // N — highest priority
        max_val = lost_time[3];
        max_idx = 2'b00;  // NORTH
    end
    // Vì dùng >=, reader sau sẽ thắng khi tie → N check cuối = N thắng tie
end
```

> [!IMPORTANT]
> **Mapping giữa `tag_detected` bit index và direction encoding**:
> - `tag_detected[3]` = North → `dir_out = 2'b00`
> - `tag_detected[2]` = South → `dir_out = 2'b01`
> - `tag_detected[1]` = East → `dir_out = 2'b10`
> - `tag_detected[0]` = West → `dir_out = 2'b11`
>
> Cẩn thận không nhầm index! `lost_time[0]` nên map với `tag_detected[0]` = West.

---

## Module 10: `priority_arbiter.v` — Priority Conflict Resolution

**Mục đích**: Merge 4 hướng thành NS/EW, so sánh class, output grant.

**Độ khó**: ⭐⭐

**Toàn bộ module là combinational** (không cần `clk`):

```
// Step 1: Merge thành NS / EW
wire [2:0] class_ns = (class_n > class_s) ? class_n : class_s;
wire [2:0] class_ew = (class_e > class_w) ? class_e : class_w;

// Step 2: Chỉ tính nếu có tag thật sự
wire ns_has_prio = (tag_detected[3] && class_n != 0) || 
                   (tag_detected[2] && class_s != 0);
wire ew_has_prio = (tag_detected[1] && class_e != 0) ||
                   (tag_detected[0] && class_w != 0);

// Step 3: So sánh và grant
always @(*) begin
    ev_detect_ns = 0;
    ev_detect_ew = 0;
    winner_class = 3'b000;

    if (ns_has_prio || ew_has_prio) begin
        if (class_ns > class_ew) begin
            ev_detect_ns = 1;
            winner_class = class_ns;
        end
        else if (class_ew > class_ns) begin
            ev_detect_ew = 1;
            winner_class = class_ew;
        end
        else begin  // cùng class → tiebreak bằng cur_state
            winner_class = class_ns;  // bằng nhau thì lấy cái nào cũng được
            if (cur_state == 4'd0)       // S_NS_GREEN
                ev_detect_ns = 1;
            else if (cur_state == 4'd3)  // S_EW_GREEN
                ev_detect_ew = 1;
            else
                ev_detect_ns = 1;        // default
        end
    end
end
```

> [!TIP]
> Module này đơn giản nhưng cần test kỹ case tie-break. Tạo testbench thử:
> - N = class 5, E = class 3 → NS wins
> - N = class 3, E = class 5 → EW wins
> - N = class 3, E = class 3, FSM đang NS_GREEN → NS wins
> - N = class 3, E = class 3, FSM đang EW_GREEN → EW wins

---

## Module 11: `traffic_fsm.v` — 10-State Moore FSM

**Mục đích**: Điều khiển chu kỳ đèn + xử lý preemption.

**Độ khó**: ⭐⭐⭐⭐

### Kiến trúc

```
traffic_fsm
└── phase_timer (instance nội bộ)
```

### Registers

```
reg [3:0] state, next_state;
reg prio_dir_reg;           // 0 = NS được priority, 1 = EW
                            // Latch khi vào S_PRIO_CLEAR
reg [3:0] saved_state;      // State trước khi bị preempt (optional, cho recovery)
```

### State Transitions

```
S_NS_GREEN (4'd0):
    Load timer = CNT_GREEN_NS
    Output: NS_G=1, EW_R=1
    if (ev_detect_ew && !prio_active):
        // EW preemption — phải finish green rồi mới đi
        // Vẫn chờ timer expired, sau đó vào yellow rồi mới prio
        // HOẶC: interrupt ngay → S_NS_YELLOW
        → S_NS_YELLOW (interrupt cách)
    if (timer_expired):
        → S_NS_YELLOW

S_NS_YELLOW (4'd1):
    Load timer = CNT_YELLOW
    Output: NS_Y=1, EW_R=1
    if (timer_expired):
        if (ev_detect_ns || ev_detect_ew):
            → S_ALL_RED_1 → S_PRIO_CLEAR   // preempt sau yellow
        else:
            → S_ALL_RED_1

S_ALL_RED_1 (4'd2):
    Load timer = CNT_ALL_RED
    Output: NS_R=1, EW_R=1
    if (timer_expired):
        if (pending_prio):   // có preempt request đang chờ
            → S_PRIO_CLEAR
        else:
            → S_EW_GREEN

S_EW_GREEN (4'd3):
    // Tương tự S_NS_GREEN nhưng cho EW
    ...

S_EW_YELLOW (4'd4):
    // Tương tự S_NS_YELLOW
    ...

S_ALL_RED_2 (4'd5):
    // Tương tự S_ALL_RED_1, next = S_NS_GREEN hoặc S_PRIO_CLEAR
    ...

// ═══ PRIORITY STATES ═══

S_PRIO_CLEAR (4'd6):
    Load timer = CNT_PRIO_CLR (5s)
    Output: NS_R=1, EW_R=1, PRIO=1
    // LATCH direction tại đây!
    prio_dir_reg <= ev_detect_ns ? 0 : 1;
    if (timer_expired):
        → S_PRIO_HOLD

S_PRIO_HOLD (4'd7):
    Load timer = CNT_PRIO_TMO (60s timeout)
    Output: PRIO=1
        if (prio_dir_reg == 0): NS_G=1, EW_R=1
        else:                   NS_R=1, EW_G=1
    // Giữ green cho đến khi:
    if (timer_expired):                          // timeout 60s
        → S_PRIO_EXIT
    if (prio_dir_reg == 0 && !ev_detect_ns):     // tag mất (NS)
        → S_PRIO_EXIT
    if (prio_dir_reg == 1 && !ev_detect_ew):     // tag mất (EW)
        → S_PRIO_EXIT

S_PRIO_EXIT (4'd8):
    Load timer = CNT_YELLOW (5s)
    Output: PRIO=1
        if (prio_dir_reg == 0): NS_Y=1, EW_R=1
        else:                   NS_R=1, EW_Y=1
    if (timer_expired):
        → S_PRIO_RECOV

S_PRIO_RECOV (4'd9):
    Load timer = CNT_ALL_RED (2s)
    Output: NS_R=1, EW_R=1, PRIO=1
    if (timer_expired):
        → S_NS_GREEN   // hoặc khôi phục về saved_state
```

### Timer Loading Pattern

```
// Mỗi khi chuyển state, load timer 1 lần
// Dùng "load on entry" pattern:
reg [3:0] prev_state;

wire state_changed = (state != prev_state);

always @(posedge clk) begin
    prev_state <= state;
    timer_load <= 0;  // default
    
    if (state_changed) begin
        timer_load <= 1;
        case (state)
            S_NS_GREEN:   timer_value <= CNT_GREEN_NS;
            S_NS_YELLOW:  timer_value <= CNT_YELLOW;
            S_ALL_RED_1:  timer_value <= CNT_ALL_RED;
            S_EW_GREEN:   timer_value <= CNT_GREEN_EW;
            S_EW_YELLOW:  timer_value <= CNT_YELLOW;
            S_ALL_RED_2:  timer_value <= CNT_ALL_RED;
            S_PRIO_CLEAR: timer_value <= CNT_PRIO_CLR;
            S_PRIO_HOLD:  timer_value <= CNT_PRIO_TMO;
            S_PRIO_EXIT:  timer_value <= CNT_YELLOW;
            S_PRIO_RECOV: timer_value <= CNT_ALL_RED;
        endcase
    end
end
```

> [!WARNING]
> **Preemption từ Yellow state**: Spec nói "Complete yellow, then S_PRIO_CLEAR". Nghĩa là khi đang Yellow mà nhận preempt request → phải chờ hết Yellow → All-Red → rồi mới PRIO_CLEAR. **KHÔNG** được nhảy thẳng từ Yellow → PRIO_CLEAR.
>
> Cách implement: set flag `pending_prio = 1` khi nhận preempt ở Yellow, check flag ở All-Red transition.

---

## Module 12: `cascade_tx.v` — Cascade Packet Transmitter

**Mục đích**: Đóng gói 5 byte → gửi tuần tự qua `uart_tx_core`.

**Độ khó**: ⭐⭐

### FSM: 6 states

```
S_IDLE:
    Đợi send = 1
    Khi send: tính checksum, byte_idx = 0 → S_SEND

S_SEND:
    if (!uart_tx_busy):
        uart_tx_data = packet[byte_idx]
        uart_tx_start = 1 (pulse)
        → S_WAIT_DONE

S_WAIT_DONE:
    Đợi uart_tx_busy = 0 (byte đã gửi xong)
    byte_idx++
    if (byte_idx == 5): → S_DONE
    else: → S_SEND

S_DONE:
    tx_done = 1 (pulse)
    → S_IDLE
```

### Packet Assembly

```
// Trước khi gửi, tính sẵn 5 byte:
wire [7:0] byte0 = 8'hCA;                                        // header
wire [7:0] byte1 = {valid, direction, clear, src_node_id};        // flags
wire [7:0] byte2 = {5'b0, prio_class};                           // class
wire [7:0] byte3 = eta;                                          // ETA
wire [7:0] byte4 = byte0 ^ byte1 ^ byte2 ^ byte3;               // checksum

// Lưu vào array hoặc dùng case:
reg [7:0] pkt_buf [0:4];

// Khi send = 1:
pkt_buf[0] <= byte0;
pkt_buf[1] <= byte1;
pkt_buf[2] <= byte2;
pkt_buf[3] <= byte3;
pkt_buf[4] <= byte0 ^ byte1 ^ byte2 ^ byte3;
```

---

## Module 13: `cascade_rx.v` — Cascade Packet Receiver

**Mục đích**: Nhận 5 byte, verify header + checksum, parse fields.

**Độ khó**: ⭐⭐

### FSM

```
reg [2:0] byte_cnt;         // 0-4
reg [7:0] pkt_buf [0:4];    // buffer nhận

S_WAIT_HEADER:
    Đợi uart_rx_valid = 1
    Nếu uart_rx_data == 8'hCA:
        pkt_buf[0] = 8'hCA
        byte_cnt = 1
        → S_COLLECT

    Nếu uart_rx_data != 8'hCA:
        Bỏ qua (garbage byte), ở lại S_WAIT_HEADER

S_COLLECT:
    Đợi uart_rx_valid = 1
    pkt_buf[byte_cnt] = uart_rx_data
    byte_cnt++
    Nếu byte_cnt == 5: → S_VERIFY

S_VERIFY (1 cycle):
    checksum = pkt_buf[0] ^ pkt_buf[1] ^ pkt_buf[2] ^ pkt_buf[3]
    if (checksum == pkt_buf[4]):
        // Parse fields
        pkt_flag_valid = pkt_buf[1][7]
        pkt_flag_dir   = pkt_buf[1][6]
        pkt_flag_clear = pkt_buf[1][5]
        pkt_src_id     = pkt_buf[1][4:0]
        pkt_class      = pkt_buf[2][2:0]
        pkt_eta        = pkt_buf[3]
        pkt_valid = 1   // pulse
    else:
        pkt_error = 1   // pulse
    → S_WAIT_HEADER
```

> [!TIP]
> Thêm **timeout**: nếu nhận header 0xCA nhưng 4 byte sau không đến trong vòng ~10ms (time cho 5 bytes @ 9600bps ≈ 5.2ms), reset về S_WAIT_HEADER. Tránh bị stuck ở S_COLLECT nếu line bị noise.

---

## Module 14: `rs485_dir_ctrl.v` — MAX485 DE Controller

**Mục đích**: Quản lý chân DE: assert trước TX, giữ 2 baud cycles sau TX done.

**Độ khó**: ⭐⭐

### FSM: 3 states

```
parameter GUARD_CYCLES = 10416;  // 2 × 5208

S_IDLE:
    rs485_de = 0     // RX mode
    tx_enable = 0
    Khi tx_start = 1:
        rs485_de = 1  // Assert DE ngay
        → S_TX_ACTIVE

S_TX_ACTIVE:
    rs485_de = 1
    tx_enable = 1     // Gate cho uart_tx
    Khi tx_done = 1:  // UART đã gửi xong stop bit
        guard_cnt = 0
        → S_GUARD

S_GUARD:
    rs485_de = 1      // Vẫn giữ DE HIGH
    tx_enable = 0     // Không cho TX thêm
    guard_cnt++
    Khi guard_cnt >= GUARD_CYCLES:
        → S_IDLE
```

> [!NOTE]
> `tx_enable` nối vào `cascade_tx` để gate `uart_tx_start`. Nghĩa là `cascade_tx` chỉ được gửi byte khi `tx_enable = 1`. Nhưng thực tế có thể đơn giản hơn: `rs485_dir_ctrl` chỉ quản lý DE pin, còn `cascade_tx` tự quản lý gửi byte. Chỉ cần đảm bảo:
> 1. DE HIGH trước khi byte đầu tiên bắt đầu
> 2. DE HIGH suốt quá trình TX
> 3. DE HIGH thêm 2 baud sau byte cuối

---

## Module 15: `cascade_ctrl.v` — Green Wave Coordinator

**Mục đích**: Phối hợp green wave giữa 4 node.

**Độ khó**: ⭐⭐⭐

### Logic chính

```
parameter INTER_NODE_DELAY = 8'd5;  // 5 giây giữa các node

// ═══ LOCAL EVENT → TX ═══
// Khi local node detect xe ưu tiên → gửi packet cho cả LEFT và RIGHT

always @(posedge clk) begin
    if (prio_active && dir_valid) begin
        // Gửi packet thông báo cho neighbors
        // Direction: nếu xe đi EAST → gửi RIGHT (downstream)
        //            nếu xe đi WEST → gửi LEFT (downstream)
        //            nếu xe đi NS → gửi cả hai (broadcast)
        
        case (dir_out)
            2'b10: begin  // EAST → gửi RIGHT
                right_tx_send  <= 1;
                right_tx_valid <= 1;
                right_tx_dir   <= dir_out[0]; // 0=NS behavior, 1=EW
                right_tx_class <= winner_class;
                right_tx_eta   <= INTER_NODE_DELAY;
            end
            2'b11: begin  // WEST → gửi LEFT
                left_tx_send  <= 1;
                // ... tương tự
            end
            default: begin  // N/S → broadcast
                left_tx_send  <= 1;
                right_tx_send <= 1;
                // ...
            end
        endcase
    end
end

// ═══ RX PACKET → PROCESS ═══

// Khi nhận packet từ LEFT hoặc RIGHT:
// 1. Nếu clear flag = 1: xe đã qua → có thể recover
// 2. Nếu valid flag = 1: xe đang đến → pre-clear

always @(posedge clk) begin
    preclear_req <= 0;  // default
    
    if (left_pkt_valid && left_pkt_flag_valid_bit) begin
        if (left_pkt_flag_clear)
            // Xe đã qua node bên trái — có thể recover
            preclear_req <= 0;
        else begin
            // Xe đang đến từ bên trái — pre-clear!
            preclear_req <= 1;
            // Forward packet sang RIGHT (nếu xe đi left→right)
            right_tx_send  <= 1;
            right_tx_eta   <= left_pkt_eta + INTER_NODE_DELAY;
            right_tx_class <= left_pkt_class;
            // ...
        end
    end

    // Tương tự cho right_pkt_valid
end
```

> [!WARNING]
> **ETA overflow**: `left_pkt_eta + INTER_NODE_DELAY` có thể > 255 (8-bit). Cần saturate: `min(eta + delay, 255)`.

---

## Module 16: `traffic_node.v` — 1-Intersection Wrapper

**Mục đích**: Kết nối tất cả module cho 1 ngã tư.

**Độ khó**: ⭐⭐ (structural — chỉ wiring, không có logic)

### Cách tiếp cận

1. **Vẽ sơ đồ kết nối trên giấy trước** — đối chiếu port list từ Sec 9
2. Instantiate theo thứ tự dependency:
   ```
   rc522_driver_x4  u_rfid (.clk(clk), .rst_n(rst_n), ...);
   
   tag_priority_lut u_lut_n (.uid(tag_uid_n), .priority_class(class_n));
   tag_priority_lut u_lut_s (.uid(tag_uid_s), .priority_class(class_s));
   tag_priority_lut u_lut_e (.uid(tag_uid_e), .priority_class(class_e));
   tag_priority_lut u_lut_w (.uid(tag_uid_w), .priority_class(class_w));
   
   priority_arbiter u_arb (...);
   direction_detector u_dir (...);
   traffic_fsm u_fsm (...);
   
   // LEFT cascade chain
   uart_tx_core   u_left_uart_tx (...);
   uart_rx_core   u_left_uart_rx (.rx_serial(left_rx), ...);
   cascade_tx     u_left_cas_tx (...);
   cascade_rx     u_left_cas_rx (...);
   rs485_dir_ctrl u_left_rs485 (.rs485_de(left_de), ...);
   
   // RIGHT cascade chain (tương tự)
   ...
   
   cascade_ctrl u_cascade (...);
   ```

3. Khai báo internal wires cho tất cả kết nối giữa module

> [!TIP]
> **Naming convention**: Đặt tên wire = `<src_module>_<port_name>`. Ví dụ: `wire [3:0] rfid_tag_detected`, `wire [2:0] arb_winner_class`. Giúp debug dễ hơn nhiều khi nhìn waveform.

---

## Module 17: `traffic_top.v` — FPGA Top-Level

**Mục đích**: Top level có PLL + reset sync + traffic_node.

**Độ khó**: ⭐⭐

```
module traffic_top (
    input  wire       clk_osc,      // Onboard oscillator
    input  wire       rst_btn_n,    // Push button
    input  wire [1:0] node_id,
    // ... tất cả physical pins theo Sec 9.17
);

// ═══ PLL ═══
wire clk_sys, pll_locked;
// Gowin PLL IP — instantiate từ Gowin IDE IP Generator
// Input: clk_osc, Output: clk_sys = 50 MHz
Gowin_PLL u_pll (
    .clkin(clk_osc),
    .clkout(clk_sys),
    .lock(pll_locked)
);

// ═══ Reset Sync ═══
wire rst_sync_n;
rst_sync u_rst (
    .clk(clk_sys),
    .rst_async_n(rst_btn_n & pll_locked),  // Reset nếu PLL chưa lock
    .rst_sync_n(rst_sync_n)
);

// ═══ Traffic Node ═══
traffic_node u_node (
    .clk(clk_sys),
    .rst_n(rst_sync_n),
    .node_id(node_id),
    // ... wire tất cả pin
);

// ═══ LED Output Mapping ═══
assign led_ns = {ns_r, ns_y, ns_g};
assign led_ew = {ew_r, ew_y, ew_g};
assign led_prio = prio_active_led;

endmodule
```

> [!IMPORTANT]
> PLL instantiation phụ thuộc vào Gowin IDE. Bạn cần:
> 1. Mở Gowin IDE → IP Generator → chọn PLL
> 2. Cấu hình input freq (oscillator freq của board) → output 50 MHz
> 3. Generate → copy instance template vào code

---

## Module 18: `tb_traffic_top.v` + `run_sim.do`

**Độ khó**: ⭐⭐⭐

### Tips quan trọng

**1. Giảm counter cho simulation**:
```verilog
// Đừng dùng giá trị thật — simulation sẽ chạy cả ngày
`ifdef SIMULATION
    parameter CNT_GREEN_NS = 32'd100;     // thay vì 1,500,000,000
    parameter CNT_YELLOW   = 32'd50;
    parameter CNT_ALL_RED  = 32'd20;
    // ...
`else
    parameter CNT_GREEN_NS = 32'd1_500_000_000;
    // ...
`endif
```

**2. Mock SPI response**:
```verilog
// Task giả lập RC522 response trên spi_miso
// Khi testbench "thấy" cs_n[i] = 0 và spi_sck rising → trả về byte
task automatic rc522_respond;
    input [7:0] response_byte;
    integer i;
    begin
        for (i = 7; i >= 0; i = i - 1) begin
            @(negedge spi_sck);
            spi_miso = response_byte[i];
        end
    end
endtask
```

**3. 14 test scenarios structure**:
```verilog
initial begin
    // Setup
    rst_btn_n = 0;
    #1000;
    rst_btn_n = 1;
    
    // ═══ Test 1: Normal cycle ═══
    $display("=== TEST 1: Normal Cycle ===");
    // Đợi FSM chạy qua: NS_G → NS_Y → AR1 → EW_G → EW_Y → AR2 → NS_G
    // Verify timing match CNT_*
    wait(uut.u_node.u_fsm.state == 4'd0);  // NS_GREEN
    // ... verify LED outputs
    
    // ═══ Test 2: Priority NS during EW green ═══
    $display("=== TEST 2: Priority NS during EW Green ===");
    wait(uut.u_node.u_fsm.state == 4'd3);  // đợi EW_GREEN
    // Inject tag ở reader North
    force uut.u_node.u_rfid.tag_detected_reg[3] = 1;
    force uut.u_node.u_rfid.uid_reg[3] = 32'hA1B2C3D4;  // class 5
    // Đợi FSM chuyển qua: EW_Y → AR → PRIO_CLEAR → PRIO_HOLD
    // Verify NS_G = 1 trong PRIO_HOLD
    
    // ... tiếp tục 12 test còn lại
    
    $display("ALL TESTS PASSED");
    $finish;
end
```

**4. `run_sim.do` cho ModelSim**:
```tcl
# Compile
vlog +define+SIMULATION rst_sync.v
vlog +define+SIMULATION phase_timer.v
vlog +define+SIMULATION tag_priority_lut.v
# ... tất cả module
vlog +define+SIMULATION tb_traffic_top.v

# Simulate
vsim -t 1ns work.tb_traffic_top

# Add waves
add wave -r /*

# Run
run -all
```

---

## 📌 Checklist Tổng

| # | Module | File | Depends On | Status |
|---|--------|------|-----------|--------|
| 1 | Reset Sync | `rst_sync.v` | — | ☐ |
| 2 | Phase Timer | `phase_timer.v` | — | ☐ |
| 3 | Priority LUT | `tag_priority_lut.v` | — | ☐ |
| 4 | CDC Sync | `cdc_sync.v` | — | ☐ |
| 5 | SPI Master | `spi_master.v` | — (reuse) | ☐ |
| 6 | UART TX | `uart_tx_core.v` | — (reuse) | ☐ |
| 7 | UART RX | `uart_rx_core.v` | — | ☐ |
| 8 | RC522 Driver | `rc522_driver_x4.v` | spi_master | ☐ |
| 9 | Direction Det. | `direction_detector.v` | — | ☐ |
| 10 | Priority Arb. | `priority_arbiter.v` | — | ☐ |
| 11 | Traffic FSM | `traffic_fsm.v` | phase_timer | ☐ |
| 12 | Cascade TX | `cascade_tx.v` | uart_tx_core | ☐ |
| 13 | Cascade RX | `cascade_rx.v` | uart_rx_core | ☐ |
| 14 | RS485 DE Ctrl | `rs485_dir_ctrl.v` | — | ☐ |
| 15 | Cascade Ctrl | `cascade_ctrl.v` | — | ☐ |
| 16 | Traffic Node | `traffic_node.v` | ALL above | ☐ |
| 17 | Traffic Top | `traffic_top.v` | rst_sync, traffic_node | ☐ |
| 18 | Testbench | `tb_traffic_top.v` | ALL | ☐ |
| 19 | Sim Script | `run_sim.do` | — | ☐ |
