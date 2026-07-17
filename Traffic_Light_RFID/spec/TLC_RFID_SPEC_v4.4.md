**DESIGN SPECIFICATION**

**Traffic Light Controller**

RFID Priority Preemption · Direction Detection · Green Wave Cascade

| **Document No.**     | TLC-RFID-SPEC-006 rev 4.4                                                    |
| -------------------- | ---------------------------------------------------------------------------- |
| **Status**           | **RELEASED**                                                                 |
| **FPGA**             | Gowin ACG525 (GW5A-25A) · 55 nm SRAM · 23,040 LUT4                           |
| **RFID**             | 16× NXP MFRC522 · 4 nodes × 4 readers (N/S/E/W) · Shared SPI bus (4 CS pins) |
| **Direction Detect** | Sequence-based timeout (last reader to lose signal = exit direction)         |
| **Cascade Link**     | RS485 (MAX485) · 9600 bps UART · 5-byte packet                               |
| **Priority Law**     | 5-tier hierarchy - Luật TTATGT đường bộ 2024, Điều 27                        |
| **Version**          | 4.4 - Technical Design Specification                                         |
| **Date**             | 2026-05-31                                                                   |
| **Project Path**     | D:\\Dev\\Code\\Traffic_Light_RFID\\                                          |

## Revision History

| **Ver.** | **Date**       | **Changes**                                                                                                                                                                                                                                                                                                                                                                                                                  |
| -------- | -------------- | ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| 3.3      | 2026-05-31     | Pure technical spec. No academic content.                                                                                                                                                                                                                                                                                                                                                                                    |
| 4.0      | 2026-05-31     | 16 RC522 (4 nodes × 4 readers). Shared SPI. direction_detector.v added. Removed pedestrian.                                                                                                                                                                                                                                                                                                                                  |
| **4.1**  | **2026-05-31** | Fixes: (1) INIT separated from poll loop - latency 400ms→20ms. (2) CDC: only sync tag_detected\[i\], not data bus. (3) Direction algorithm: sequence-based tracking, not 1111 assumption. (4) rs485_dir_ctrl: DE guard delay 2 baud cycles. (5) Law citation: Luật TTATGT 2024 Điều 27. (6) All modules: full port list added.                                                                                               |
| **4.2**  | **2026-05-31** | **Fixes: (1) Priority table corrected per Luật TTATGT 2024 Điều 27 - Tier 4 merges quân sự/công an/kiểm sát + đoàn xe dẫn đường; Tier 2 updated to xe hộ đê/khắc phục thiên tai/dịch bệnh. (2) CDC simplified: rc522_driver_x4 runs on CLK_SYS - no CDC needed for RFID signals; only rst_sync remains for async external input. (3) direction_detector: added dir_confidence output (1=normal, 0=low confidence/timeout).** |
| **4.3**  | **2026-05-31** | **Consistency fixes: (1) Sec 4.2 UID table: Tier 2 and 4 descriptions updated to match Sec 6.1. (2) All tag_detected_sync → tag_detected (no CDC needed). (3) rc522_driver_x4: removed stale CLK_SPI/cdc_sync comments. (4) SPI timing: corrected MISO tmax per RC522 datasheet (~70ns), max SPI clock reduced to 5 MHz to ensure margin. (5) Test case 14 updated.**                                                        |
| **4.4**  | **2026-05-31** | **RELEASE CANDIDATE. Final 2 fixes: (1) Sec 2.2 pin table: SPI clock updated to max 5 MHz (clk_div=4). (2) Sec 2.3 timing constraint: create_generated_clock -divide_by corrected from 5 to 10 (50MHz÷10=5MHz).**                                                                                                                                                                                                            |

# 1 System Overview

## 1.1 Objectives

Thiết kế bộ điều khiển đèn giao thông tổng hợp được (synthesizable RTL) cho hành lang 4 ngã tư liên tiếp. Mỗi ngã tư có 4 RFID reader (N/S/E/W) để phát hiện xe ưu tiên và xác định hướng đi. Cơ chế Green Wave cascade phối hợp pre-clearance giữa các node.

| **#** | **Tính năng**                           | **Hiện thực**                                        |
| ----- | --------------------------------------- | ---------------------------------------------------- |
| 1     | Phát hiện xe ưu tiên - 4 hướng/node     | 4× RC522 · Shared SPI · Round-robin poll ~20ms/cycle |
| 2     | Xác định hướng đi (Direction Detection) | direction_detector.v · Sequence-based timeout        |
| 3     | FSM ưu tiên - 10 trạng thái             | Moore FSM · Verilog HDL · Synthesizable              |
| 4     | 5-tier priority - Luật TTATGT VN 2024   | priority_arbiter.v · priority_class\[2:0\]           |
| 5     | Green Wave cascade 4 node               | RS485 · MAX485 · 9600 bps · 5-byte packet            |

## 1.2 Prototype vs. Deployment

| **Thành phần**      | **Prototype (Lab)**        | **Deployment (Thực tế)**                                 |
| ------------------- | -------------------------- | -------------------------------------------------------- |
| RFID technology     | RC522 · 13.56 MHz HF       | UHF passive RFID · 860-960 MHz · EPC Gen2 · tầm 100-300m |
| Direction detection | Timeout sequence (digital) | RSSI comparison (analog output từ UHF reader)            |
| Cascade link        | RS485 · ≤1.2 km            | LoRa / Fiber optic - không đổi logic                     |
| Interface to FPGA   | **KHÔNG ĐỔI**              | **KHÔNG ĐỔI**                                            |

## 1.3 Design Flow

Step 1 RTL Coding Verilog HDL → 17 modules (15 synth + 1 TB + 1 script)

Step 2 Functional Sim ModelSim → 14 directed test scenarios

Step 3 Synthesis Gowin IDE → LUT / FF / BSRAM utilization report

Step 4 P&R + STA Gowin IDE → fmax, setup/hold slack

Step 5 Bitstream GowinProgrammer → flash GW5A-25A

Step 6 On-Board Verify GAO + LED check

## 1.4 Scope

| **Component**        | **Status** | **Notes**                                                                  |
| -------------------- | ---------- | -------------------------------------------------------------------------- |
| phase_timer.v        | **SYNTH**  | 32-bit countdown timer                                                     |
| traffic_fsm.v        | **SYNTH**  | 10-state Moore FSM                                                         |
| priority_arbiter.v   | **SYNTH**  | 5-tier VN traffic law                                                      |
| direction_detector.v | **SYNTH**  | Sequence-based direction inference                                         |
| cdc_sync.v           | **UTIL**   | Utility - không dùng trong design hiện tại. Giữ lại cho mở rộng tương lai. |
| rst_sync.v           | **SYNTH**  | Async assert / sync release                                                |
| spi_master.v         | **REUSE**  | Reused from ECG project                                                    |
| rc522_driver_x4.v    | **SYNTH**  | 4 readers · shared SPI · INIT once                                         |
| tag_priority_lut.v   | **SYNTH**  | UID\[31:0\] → priority_class\[2:0\]                                        |
| uart_tx_core.v       | **REUSE**  | Reused from ECG project                                                    |
| uart_rx_core.v       | **SYNTH**  | 9600 bps RX · 16× oversampling                                             |
| cascade_tx.v         | **SYNTH**  | Pack 5-byte cascade packet                                                 |
| cascade_rx.v         | **SYNTH**  | Parse + verify cascade packet                                              |
| cascade_ctrl.v       | **SYNTH**  | Green Wave coordinator                                                     |
| rs485_dir_ctrl.v     | **SYNTH**  | MAX485 DE · 2-baud guard delay                                             |
| traffic_node.v       | **SYNTH**  | 1-intersection wrapper                                                     |
| traffic_top.v        | **SYNTH**  | FPGA top-level + PLL                                                       |
| tb_traffic_top.v     | **TB**     | 14 scenarios · ModelSim                                                    |
| run_sim.do           | **SCRIPT** | ModelSim automation                                                        |

# 2 FPGA Target - Gowin GW5A-25A

## 2.1 Device Specifications

| **Resource**  | **GW5A-25A Total** | **Estimated (1 Node)**  |
| ------------- | ------------------ | ----------------------- |
| LUT4          | 23,040             | ~3,500-4,500 (≈20%)     |
| Flip-Flops    | 23,040             | ~2,500-3,500 (≈15%)     |
| BSRAM (18 Kb) | 56 blocks          | 2-4 blocks              |
| DSP (27×18)   | 28                 | 0                       |
| PLL           | 6                  | 1 - 50 MHz system clock |

**ⓘ** Update after running Gowin IDE synthesis.

## 2.2 Pin Assignment (1 Node)

| **Function**              | **Resource** | **Notes**                                   |
| ------------------------- | ------------ | ------------------------------------------- |
| NS / EW R, Y, G           | 6 LEDs       | Direct LED drive                            |
| Priority Active           | 1 LED        | HIGH during S_PRIO_HOLD                     |
| RC522 SPI (SCK/MOSI/MISO) | 3 GPIO       | Shared by 4 readers · max 5 MHz (clk_div=4) |
| CS_N\[3:0\] (N, S, E, W)  | 4 GPIO       | Active-low · one per reader                 |
| RS485 LEFT (TX/RX/DE)     | 3 GPIO       | MAX485 half-duplex                          |
| RS485 RIGHT (TX/RX/DE)    | 3 GPIO       | MAX485 half-duplex                          |
| Reset                     | Push button  | Active-low · async assert / sync release    |
| Node ID                   | DIP × 2-bit  | node_id\[1:0\]                              |
| System Clock              | Onboard OSC  | OSC → PLL → 50 MHz                          |

## 2.3 Timing Constraints

create_clock -name clk_sys -period 20.000 \[get_ports {clk}\]

create_generated_clock -name spi_sck -source \[get_ports {clk}\] \\

\-divide_by 10 \[get_nets {spi_sck_int}\]

set_false_path -from \[get_ports {rst_btn_n}\] -to \[get_clocks {clk_sys}\]

# 3 Clock Domain Crossing & Reset Strategy

## 3.1 Clock Domain Analysis - Single Domain

Toàn bộ thiết kế chạy trên một clock domain duy nhất: CLK_SYS = 50 MHz từ PLL. Không tồn tại CDC crossing nội bộ thật sự.

| **Signal**                   | **Domain** | **Giải thích**                                                                                                                                                                                       |
| ---------------------------- | ---------- | ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| spi_sck                      | CLK_SYS    | spi_sck là output signal được toggle từ logic CLK_SYS - KHÔNG phải clock domain riêng. Tất cả logic trong rc522_driver_x4 và spi_master đều clocked bởi clk 50MHz.                                   |
| tag_detected\[3:0\], tag_uid | CLK_SYS    | Output của rc522_driver_x4 đã ở CLK_SYS. Không cần CDC. spi_miso được sample đồng bộ bởi spi_master trên rising edge của spi_sck - spi_sck được generate từ CLK_SYS nên việc capture là synchronous. |
| cascade_pkt_valid, rx_data   | CLK_SYS    | uart_rx_core dùng baud rate counter nội bộ chạy bởi clk 50MHz. Tất cả output đã ở CLK_SYS.                                                                                                           |
| rst_btn_n                    | **ASYNC**  | Push button là nguồn async duy nhất. Xử lý bởi rst_sync.v.                                                                                                                                           |

**ⓘ** Kết luận: cdc_sync.v không cần thiết trong thiết kế này. Module rst_sync.v là đủ để xử lý toàn bộ vấn đề đồng bộ hóa. cdc_sync.v có thể giữ lại trong codebase như utility module cho mở rộng tương lai (ví dụ: tích hợp module external clock ngoài).

## 3.2 spi_miso Input Stability

spi_miso là tín hiệu external từ RC522, cần constraint để đảm bảo timing closure. Theo RC522 datasheet, MISO output valid trong vòng ~70ns sau SCK falling edge (tại 3.3V). Do đó:

| **Thông số**                  | **Giá trị & Lý do**                                     |
| ----------------------------- | ------------------------------------------------------- |
| MISO output valid (tDOV)      | ≤ 70 ns sau SCK falling edge (RC522 datasheet, 3.3V)    |
| SPI clock tối đa an toàn      | 5 MHz → half-period = 100 ns > 70 ns → margin = 30 ns ✓ |
| SPI clock 10 MHz (KHÔNG dùng) | **half-period = 50 ns < 70 ns → vi phạm setup time ✗**  |
| clk_div trong spi_master      | clk_div = 4 → SPI freq = 50MHz / (2×5) = 5 MHz          |

\# spi_miso timing constraint (Gowin IDE .sdc)

\# MISO captured on spi_sck rising edge, valid ~70ns after spi_sck falling

\# At 5MHz: half-period = 100ns, margin = 30ns

set_input_delay -clock spi_sck -max 70 \[get_ports {spi_miso}\]

set_input_delay -clock spi_sck -min 5 \[get_ports {spi_miso}\]

**ⓘ** KHÔNG dùng SPI clock 10 MHz với RC522 ở 3.3V. MISO valid time ~70ns > half-period 50ns → setup violation. Giữ clk_div ≥ 4 (SPI ≤ 5MHz).

## 3.3 Reset Strategy

// rst_sync.v - async assert / synchronous release

module rst_sync (

input wire clk,

input wire rst_async_n, // push button active-low

output reg rst_sync_n

);

reg rst_meta;

always @(posedge clk or negedge rst_async_n) begin

if (!rst_async_n) begin rst_meta <= 1'b0; rst_sync_n <= 1'b0; end

else begin rst_meta <= 1'b1; rst_sync_n <= rst_meta; end

end

endmodule

# 4 RFID Interface - 4× RC522 per Node

## 4.1 Shared SPI Bus Architecture

Bốn RC522 chia sẻ chung SCK, MOSI, MISO. Mỗi reader có CS_N riêng. rc522_driver_x4.v poll tuần tự round-robin. INIT chỉ chạy 1 lần sau reset - poll loop chỉ thực hiện REQA + ANTICOL.

Sau reset: INIT tất cả 4 reader (ghi TxModeReg, RxModeReg, TxControlReg)

→ ~10ms mỗi reader → ~40ms tổng (một lần duy nhất)

Poll loop: CS_N\[0\]=0 → REQA+ANTICOL → CS_N\[0\]=1 (~2-5ms)

CS_N\[1\]=0 → REQA+ANTICOL → CS_N\[1\]=1 (~2-5ms)

CS_N\[2\]=0 → REQA+ANTICOL → CS_N\[2\]=1 (~2-5ms)

CS_N\[3\]=0 → REQA+ANTICOL → CS_N\[3\]=1 (~2-5ms)

→ Tổng ~8-20ms/cycle

## 4.2 Tag UID → Priority Class

| **Tag UID (hex)** | **priority_class** | **Vehicle - Luật TTATGT 2024, Điều 27**                                                                        |
| ----------------- | ------------------ | -------------------------------------------------------------------------------------------------------------- |
| 0xA1B2C3D4        | **3'b101**         | Xe chữa cháy đang thực hiện nhiệm vụ (Tier 5 - cao nhất)                                                       |
| 0xE5F6A7B8        | **3'b100**         | Xe quân sự, công an, kiểm sát đang làm nhiệm vụ khẩn cấp; đoàn xe có xe công an/quân sự dẫn đường (Tier 4)     |
| 0x12345678        | **3'b011**         | Xe cứu thương, xe cứu nạn đang thực hiện nhiệm vụ (Tier 3)                                                     |
| 0x9ABCDEF0        | **3'b010**         | Xe hộ đê; xe làm nhiệm vụ khắc phục sự cố thiên tai, dịch bệnh, tai nạn hoặc tình huống khẩn cấp khác (Tier 2) |
| 0x11223344        | 3'b001             | Xe tang (Tier 1 - thấp nhất)                                                                                   |
| Any other UID     | 3'b000             | Xe thường - không ưu tiên                                                                                      |

# 5 Direction Detection - direction_detector.v

## 5.1 Nguyên lý hoạt động

Với RC522 tầm 3-5cm, tag sẽ không kích hoạt cả 4 reader cùng lúc. Thay vào đó tag kích hoạt lần lượt các reader theo lộ trình di chuyển. Module theo dõi thứ tự mất tín hiệu (falling edge của tag_detected\[i\]) và kết luận hướng đi dựa trên reader mất tín hiệu cuối cùng.

Ví dụ - xe vào từ SOUTH, rẽ sang EAST:

t=0ms: tag_detected = 4'b0100 (chỉ reader S thấy tag - xe đang vào từ Nam)

t=50ms: tag_detected = 4'b0110 (S + E cùng thấy - xe đang qua trung tâm)

t=100ms: tag_detected = 4'b0010 (chỉ E còn thấy - xe rẽ sang Đông)

t=150ms: tag_detected = 4'b0000 (E mất tín hiệu cuối cùng)

last_falling_edge = E → dir_out = 2'b10 (EAST) → dir_valid = 1

## 5.2 State Machine - 3 States

| **State** | **Trigger**                           | **Action**                                                                                                                                                 |
| --------- | ------------------------------------- | ---------------------------------------------------------------------------------------------------------------------------------------------------------- |
| IDLE      | any tag_detected\[i\] → 1             | Khởi tạo lost_time\[3:0\] = 0, seen_count = 0. Chuyển sang TRACKING.                                                                                       |
| TRACKING  | any_detected = 0 hoặc CNT_DIR_TIMEOUT | Mỗi khi tag_detected\[i\] có falling edge (1→0): lưu lost_time\[i\] = current_tick, tăng seen_count. Đợi đến khi all zeros hoặc timeout.                   |
| DECIDE    | 1 cycle                               | dir_out = argmax(lost_time). dir_confidence = 1 nếu seen_count ≥ 2 VÀ không phải timeout; ngược lại dir_confidence = 0. Pulse dir_valid = 1. Quay về IDLE. |

**ⓘ** Tie-break nếu 2 reader mất tín hiệu cùng tick: ưu tiên N > E > S > W.

**ⓘ** CNT_DIR_TIMEOUT = 10s = 500,000,000 cycles @ 50MHz. Nếu xe dừng giữa ngã tư > 10s: dir_confidence = 0, dir_out = hướng đang green (fallback an toàn).

## 5.3 Direction Encoding

| **dir_out\[1:0\]** | **Hướng** | **Ý nghĩa**                     |
| ------------------ | --------- | ------------------------------- |
| 2'b00              | NORTH     | Reader N mất tín hiệu cuối cùng |
| 2'b01              | SOUTH     | Reader S mất tín hiệu cuối cùng |
| 2'b10              | EAST      | Reader E mất tín hiệu cuối cùng |
| 2'b11              | WEST      | Reader W mất tín hiệu cuối cùng |

# 6 Priority System - Luật TTATGT 2024

## 6.1 5-Tier Hierarchy

| **Tier** | **Tên**    | **priority_class** | **Phương tiện (Điều 27, Luật TTATGT 2024)**                                                                      |
| -------- | ---------- | ------------------ | ---------------------------------------------------------------------------------------------------------------- |
| **5**    | Nhất hỏa   | 3'b101             | Xe chữa cháy đang thực hiện nhiệm vụ (ưu tiên tuyệt đối)                                                         |
| 4        | Nhì công   | 3'b100             | Xe quân sự, công an, kiểm sát nhân dân đang thực hiện nhiệm vụ khẩn cấp; đoàn xe có xe công an/quân sự dẫn đường |
| 3        | Tam thương | 3'b011             | Xe cứu thương, xe cứu nạn đang thực hiện nhiệm vụ                                                                |
| 2        | Tứ khẩn    | 3'b010             | Xe hộ đê; xe đang làm nhiệm vụ khắc phục sự cố thiên tai, dịch bệnh, tai nạn hoặc tình huống khẩn cấp khác       |
| 1        | Ngũ tang   | 3'b001             | Đoàn xe tang                                                                                                     |

## 6.2 Conflict Resolution

// priority_arbiter.v

// Merge 4 directions → NS = max(class_N, class_S), EW = max(class_E, class_W)

if (class_ns > class_ew) grant_ns = 1;

else if (class_ew > class_ns) grant_ew = 1;

else begin // cùng bậc - tiebreak

if (cur_state == S_NS_GREEN) grant_ns = 1;

else if (cur_state == S_EW_GREEN) grant_ew = 1;

else grant_ns = 1; // default

end

# 7 Traffic Light FSM - traffic_fsm.v

## 7.1 State Encoding

| **State**    | **Encoding** | **Group**    | **Description**                                            |
| ------------ | ------------ | ------------ | ---------------------------------------------------------- |
| S_NS_GREEN   | 4'd0         | **Normal**   | NS green, EW red                                           |
| S_NS_YELLOW  | 4'd1         | **Normal**   | NS yellow - chuyển pha                                     |
| S_ALL_RED_1  | 4'd2         | **Normal**   | All-red clearance (2s)                                     |
| S_EW_GREEN   | 4'd3         | **Normal**   | EW green, NS red                                           |
| S_EW_YELLOW  | 4'd4         | **Normal**   | EW yellow - chuyển pha                                     |
| S_ALL_RED_2  | 4'd5         | **Normal**   | All-red clearance (2s)                                     |
| S_PRIO_CLEAR | 4'd6         | **Priority** | All-red hold (5s) - dọn đường                              |
| S_PRIO_HOLD  | 4'd7         | **Priority** | Green cho hướng ưu tiên - giữ đến khi tag mất hoặc timeout |
| S_PRIO_EXIT  | 4'd8         | **Priority** | Yellow hướng ưu tiên (5s) - thoát ưu tiên                  |
| S_PRIO_RECOV | 4'd9         | **Priority** | All-red (2s) - khôi phục về Normal                         |

## 7.2 Moore Output Table

| **State**    | **NS_R** | **NS_Y** | **NS_G** | **EW_R** | **EW_Y** | **EW_G** | **PRIO** |
| ------------ | -------- | -------- | -------- | -------- | -------- | -------- | -------- |
| S_NS_GREEN   | **0**    | **0**    | **1**    | **1**    | **0**    | **0**    | **0**    |
| S_NS_YELLOW  | **0**    | **1**    | **0**    | **1**    | **0**    | **0**    | **0**    |
| S_ALL_RED_1  | **1**    | **0**    | **0**    | **1**    | **0**    | **0**    | **0**    |
| S_EW_GREEN   | **1**    | **0**    | **0**    | **0**    | **0**    | **1**    | **0**    |
| S_EW_YELLOW  | **1**    | **0**    | **0**    | **0**    | **1**    | **0**    | **0**    |
| S_ALL_RED_2  | **1**    | **0**    | **0**    | **1**    | **0**    | **0**    | **0**    |
| S_PRIO_CLEAR | **1**    | **0**    | **0**    | **1**    | **0**    | **0**    | **1**    |
| PRIO_HOLD†   | **-**    | **-**    | **-**    | **-**    | **-**    | **-**    | **1**    |
| PRIO_EXIT†   | **-**    | **-**    | **-**    | **-**    | **-**    | **-**    | **1**    |
| S_PRIO_RECOV | **1**    | **0**    | **0**    | **1**    | **0**    | **0**    | **1**    |

**ⓘ** † PRIO_HOLD/EXIT: chọn NS_G/NS_Y hoặc EW_G/EW_Y theo prio_dir_reg (latch khi vào S_PRIO_CLEAR).

## 7.3 Phase Timing

| **Phase**         | **Duration** | **Cycles @ 50 MHz** | **Parameter (32-bit)** |
| ----------------- | ------------ | ------------------- | ---------------------- |
| NS Green          | 30 s         | 1,500,000,000       | CNT_GREEN_NS           |
| Yellow            | 5 s          | 250,000,000         | CNT_YELLOW             |
| All-Red           | 2 s          | 100,000,000         | CNT_ALL_RED            |
| EW Green          | 25 s         | 1,250,000,000       | CNT_GREEN_EW           |
| Priority Clear    | 5 s          | 250,000,000         | CNT_PRIO_CLR           |
| Priority Timeout  | 60 s         | 3,000,000,000       | CNT_PRIO_TMO           |
| Direction Timeout | 10 s         | 500,000,000         | CNT_DIR_TIMEOUT        |

**ⓘ** CNT_PRIO_TMO = 3,000,000,000 > 2^31. Tất cả counter phải khai báo reg \[31:0\].

# 8 Green Wave - Cascade Protocol

## 8.1 Cascade Packet (5 bytes, 9600 bps 8N1)

| **Byte 0** | **Byte 1 - Flags** | **Byte 2**            | **Byte 3**    | **Byte 4**   |
| ---------- | ------------------ | --------------------- | ------------- | ------------ |
| 0xCA       | Bit-field          | priority_class\[2:0\] | ETA (0-255 s) | XOR checksum |

Flags \[7:0\]:

\[7\] valid - 1 = packet mang dữ liệu xe ưu tiên

\[6\] direction - 0 = NS, 1 = EW (từ dir_out của direction_detector)

\[5\] clear - 1 = xe đã qua, downstream có thể recover

\[4:0\] src_node_id - node ID nguồn (0-3)

Checksum = Byte0 ^ Byte1 ^ Byte2 ^ Byte3

## 8.2 RS485 DE Guard Delay

Vấn đề: nếu kéo DE xuống LOW ngay khi UART TX done, bit Stop chưa thoát khỏi đường truyền vật lý → node nhận bị Frame Error. rs485_dir_ctrl.v phải đợi 2 baud cycle sau tx_done mới hạ DE.

// Timing - 9600 bps: 1 baud cycle = 104.2 μs = 5208 cycles @ 50MHz

//

// TX_START ─────┐ ┌──────

// DE ─────┘ (assert DE 1 cycle before first bit) │

// UART_TX ──\[S\]\[D0\]\[D1\]...\[D7\]\[STOP\]────────────────── │

// tx_done ─────────────────────────────────────────┐ │

// DE (wait 2×baud)└────┘

// DE LOW here

# 9 Module Specifications - Port List

Mỗi module được mô tả đầy đủ: mô tả chức năng, port list đầy đủ (tên, chiều, độ rộng, mô tả), và các ghi chú thiết kế quan trọng.

### 9.1 phase_timer.v

32-bit countdown timer. Khi load = 1, nạp value vào counter. Đếm xuống mỗi clock. Pulse expired = 1 khi counter = 0.

| **Port Name** | **Dir** | **Width** | **Description**                                     |
| ------------- | ------- | --------- | --------------------------------------------------- |
| clk           | **IN**  | 1         | System clock 50 MHz                                 |
| rst_n         | **IN**  | 1         | Active-low synchronous reset                        |
| load          | **IN**  | 1         | Load pulse - nạp value vào counter (1-cycle strobe) |
| value         | **IN**  | 32        | Giá trị đếm xuống (số clock cycles)                 |
| expired       | **OUT** | 1         | Pulse 1 cycle khi counter chạm 0                    |

### 9.2 traffic_fsm.v

10-state Moore FSM. Instantiates phase_timer internally. Latch prio_dir_reg khi vào S_PRIO_CLEAR.

| **Port Name**    | **Dir** | **Width** | **Description**                                       |
| ---------------- | ------- | --------- | ----------------------------------------------------- |
| clk              | **IN**  | 1         | System clock                                          |
| rst_n            | **IN**  | 1         | Active-low reset                                      |
| ev_detect_ns     | **IN**  | 1         | Từ priority_arbiter - có xe ưu tiên hướng NS          |
| ev_detect_ew     | **IN**  | 1         | Từ priority_arbiter - có xe ưu tiên hướng EW          |
| prio_class       | **IN**  | 3         | Bậc ưu tiên của xe thắng từ arbiter                   |
| cascade_preclear | **IN**  | 1         | Từ cascade_ctrl - yêu cầu pre-clear từ upstream       |
| ns_r             | **OUT** | 1         | NS Red LED                                            |
| ns_y             | **OUT** | 1         | NS Yellow LED                                         |
| ns_g             | **OUT** | 1         | NS Green LED                                          |
| ew_r             | **OUT** | 1         | EW Red LED                                            |
| ew_y             | **OUT** | 1         | EW Yellow LED                                         |
| ew_g             | **OUT** | 1         | EW Green LED                                          |
| prio_active      | **OUT** | 1         | HIGH trong S_PRIO_CLEAR / HOLD / EXIT / RECOV         |
| cur_state        | **OUT** | 4         | State hiện tại - dùng bởi priority_arbiter (tiebreak) |
| timer_load       | **OUT** | 1         | Load pulse đến phase_timer                            |
| timer_value      | **OUT** | 32        | Giá trị nạp vào phase_timer                           |

### 9.3 priority_arbiter.v

Combinational. Merge 4 directions → NS/EW. So sánh class. Output grant cho FSM.

| **Port Name** | **Dir** | **Width** | **Description**                                                                                          |
| ------------- | ------- | --------- | -------------------------------------------------------------------------------------------------------- |
| tag_detected  | **IN**  | 4         | tag_detected\[3:0\] từ rc522_driver_x4 - \[3\]=N \[2\]=S \[1\]=E \[0\]=W (CLK_SYS domain, không cần CDC) |
| class_n       | **IN**  | 3         | priority_class từ LUT cho reader North                                                                   |
| class_s       | **IN**  | 3         | priority_class từ LUT cho reader South                                                                   |
| class_e       | **IN**  | 3         | priority_class từ LUT cho reader East                                                                    |
| class_w       | **IN**  | 3         | priority_class từ LUT cho reader West                                                                    |
| cur_state     | **IN**  | 4         | FSM state hiện tại - dùng cho tiebreak                                                                   |
| ev_detect_ns  | **OUT** | 1         | Có xe ưu tiên hướng NS → kích FSM                                                                        |
| ev_detect_ew  | **OUT** | 1         | Có xe ưu tiên hướng EW → kích FSM                                                                        |
| winner_class  | **OUT** | 3         | Bậc ưu tiên của xe thắng - NS = max(N,S), EW = max(E,W)                                                  |

### 9.4 direction_detector.v

3-state FSM: IDLE → TRACKING → DECIDE. Theo dõi falling edge của từng bit tag_detected, kết luận hướng xe từ reader mất tín hiệu cuối cùng.

| **Port Name**  | **Dir** | **Width** | **Description**                                                                                                                                                                |
| -------------- | ------- | --------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| clk            | **IN**  | 1         | System clock                                                                                                                                                                   |
| rst_n          | **IN**  | 1         | Active-low reset                                                                                                                                                               |
| tag_detected   | **IN**  | 4         | tag_detected\[3:0\] từ rc522_driver_x4 (CLK_SYS domain)                                                                                                                        |
| any_detected   | **IN**  | 1         | OR của tag_detected - HIGH khi có ít nhất 1 reader thấy tag                                                                                                                    |
| dir_out        | **OUT** | 2         | Hướng xe: 00=N 01=S 10=E 11=W                                                                                                                                                  |
| dir_valid      | **OUT** | 1         | 1-cycle pulse khi direction confirmed (kèm dir_confidence)                                                                                                                     |
| dir_confidence | **OUT** | 1         | 1 = phát hiện bình thường (seen_count ≥ 2, không timeout). 0 = độ tin cậy thấp (chỉ 1 reader thấy tag, hoặc timeout, hoặc tie). cascade_ctrl nên log nhưng vẫn forward packet. |

### 9.5 cdc_sync.v (Utility - không dùng trong design hiện tại)

Generic 2-FF synchronizer. Toàn bộ thiết kế chạy CLK_SYS duy nhất nên module này không được instantiate. Giữ lại trong codebase phòng trường hợp mở rộng tích hợp external clock domain (ví dụ: GPS module, LoRa module có clock riêng). Nếu dùng: chỉ instantiate cho W=1 (1-bit valid/strobe). KHÔNG dùng cho data bus nhiều bit.

| **Port Name** | **Dir** | **Width** | **Description**                          |
| ------------- | ------- | --------- | ---------------------------------------- |
| clk_dst       | **IN**  | 1         | Destination domain clock                 |
| rst_n         | **IN**  | 1         | Active-low reset                         |
| d             | **IN**  | W         | Input signal (source domain) - W=1       |
| q             | **OUT** | W         | Synchronized output (destination domain) |

### 9.6 rst_sync.v

Async assert / synchronous release. Một instance duy nhất trong traffic_top.v, rst_sync_n kết nối đến tất cả module.

| **Port Name** | **Dir** | **Width** | **Description**                                   |
| ------------- | ------- | --------- | ------------------------------------------------- |
| clk           | **IN**  | 1         | System clock                                      |
| rst_async_n   | **IN**  | 1         | Push button reset (active-low, asynchronous)      |
| rst_sync_n    | **OUT** | 1         | Synchronized reset đến tất cả module (active-low) |

### 9.7 spi_master.v (Reused - ECG Project)

SPI Mode 0 (CPOL=0, CPHA=0), 8-bit full-duplex, configurable clock divider. CS không được điều khiển từ module này - CS_N\[3:0\] được rc522_driver_x4 quản lý.

| **Port Name** | **Dir** | **Width** | **Description**                                   |
| ------------- | ------- | --------- | ------------------------------------------------- |
| clk           | **IN**  | 1         | System clock                                      |
| rst_n         | **IN**  | 1         | Active-low reset                                  |
| start         | **IN**  | 1         | Start 8-bit transfer (1-cycle strobe)             |
| tx_data       | **IN**  | 8         | Byte to transmit (MOSI)                           |
| clk_div       | **IN**  | 3         | Clock divider: SPI freq = clk / (2 × (clk_div+1)) |
| spi_miso      | **IN**  | 1         | MISO từ RC522                                     |
| spi_sck       | **OUT** | 1         | SPI clock đến RC522                               |
| spi_mosi      | **OUT** | 1         | MOSI đến RC522                                    |
| rx_data       | **OUT** | 8         | Received byte (MISO)                              |
| done          | **OUT** | 1         | 1-cycle pulse khi transfer hoàn thành             |

### 9.8 rc522_driver_x4.v

Poll 4 RC522 reader qua shared SPI bus. INIT chạy 1 lần sau reset. Poll loop: REQA → ANTICOL → UID → chuyển CS. Giữ tag_uid stable giữa các poll cycle. Toàn bộ logic chạy trên CLK_SYS - spi_sck là output signal toggle, không phải clock domain riêng. Output tag_detected và tag_uid đã ở CLK_SYS, không cần CDC.

| **Port Name** | **Dir** | **Width** | **Description**                                                                                                                    |
| ------------- | ------- | --------- | ---------------------------------------------------------------------------------------------------------------------------------- |
| clk           | **IN**  | 1         | System clock (cũng dùng để drive spi_master)                                                                                       |
| rst_n         | **IN**  | 1         | Active-low reset - trigger INIT sequence                                                                                           |
| spi_miso      | **IN**  | 1         | MISO chung từ 4 RC522                                                                                                              |
| spi_sck       | **OUT** | 1         | SCK chung đến 4 RC522                                                                                                              |
| spi_mosi      | **OUT** | 1         | MOSI chung đến 4 RC522                                                                                                             |
| cs_n          | **OUT** | 4         | CS_N\[3:0\] - \[3\]=N \[2\]=S \[1\]=E \[0\]=W. Active-low. Chỉ 1 bit LOW tại một thời điểm.                                        |
| tag_detected  | **OUT** | 4         | tag_detected\[i\]=1 khi reader i đọc được UID hợp lệ. Đã ở CLK_SYS - kết nối trực tiếp vào priority_arbiter và direction_detector. |
| tag_uid_n     | **OUT** | 32        | UID từ reader North (stable giữa polls)                                                                                            |
| tag_uid_s     | **OUT** | 32        | UID từ reader South                                                                                                                |
| tag_uid_e     | **OUT** | 32        | UID từ reader East                                                                                                                 |
| tag_uid_w     | **OUT** | 32        | UID từ reader West                                                                                                                 |

**ⓘ** tag_uid_n/s/e/w giữ stable trong toàn bộ thời gian tag hiện diện. CLK_SYS domain sample trực tiếp sau khi tag_detected\[i\] = 1.

### 9.9 tag_priority_lut.v

Combinational lookup table. Dùng 4 lần (1 instance/reader) trong traffic_node.v.

| **Port Name**  | **Dir** | **Width** | **Description**                         |
| -------------- | ------- | --------- | --------------------------------------- |
| uid            | **IN**  | 32        | Tag UID từ rc522_driver_x4              |
| priority_class | **OUT** | 3         | Bậc ưu tiên (3'b000 nếu UID không khớp) |

### 9.10 uart_tx_core.v (Reused - ECG Project)

9600 bps, 8N1 UART transmitter.

| **Port Name** | **Dir** | **Width** | **Description**                                |
| ------------- | ------- | --------- | ---------------------------------------------- |
| clk           | **IN**  | 1         | System clock                                   |
| rst_n         | **IN**  | 1         | Active-low reset                               |
| tx_data       | **IN**  | 8         | Byte cần truyền                                |
| tx_start      | **IN**  | 1         | Strobe bắt đầu truyền (1-cycle)                |
| tx_serial     | **OUT** | 1         | UART TX line → MAX485 DI                       |
| tx_busy       | **OUT** | 1         | HIGH trong khi đang truyền                     |
| tx_done       | **OUT** | 1         | 1-cycle pulse khi truyền xong (kể cả Stop bit) |

### 9.11 uart_rx_core.v

9600 bps, 8N1 UART receiver. 16× oversampling, mid-bit sampling.

| **Port Name** | **Dir** | **Width** | **Description**                           |
| ------------- | ------- | --------- | ----------------------------------------- |
| clk           | **IN**  | 1         | System clock                              |
| rst_n         | **IN**  | 1         | Active-low reset                          |
| rx_serial     | **IN**  | 1         | UART RX line từ MAX485 RO                 |
| rx_data       | **OUT** | 8         | Received byte                             |
| rx_valid      | **OUT** | 1         | 1-cycle pulse khi nhận xong 1 byte hợp lệ |
| rx_error      | **OUT** | 1         | 1-cycle pulse khi phát hiện framing error |

### 9.12 cascade_tx.v

Đóng gói cascade data → 5-byte packet → uart_tx_core. Tính XOR checksum tự động.

| **Port Name** | **Dir** | **Width** | **Description**                              |
| ------------- | ------- | --------- | -------------------------------------------- |
| clk           | **IN**  | 1         | System clock                                 |
| rst_n         | **IN**  | 1         | Active-low reset                             |
| send          | **IN**  | 1         | Strobe: bắt đầu truyền packet (1-cycle)      |
| valid         | **IN**  | 1         | Flags\[7\]: packet mang dữ liệu hợp lệ       |
| direction     | **IN**  | 1         | Flags\[6\]: 0=NS, 1=EW                       |
| clear         | **IN**  | 1         | Flags\[5\]: xe đã qua                        |
| src_node_id   | **IN**  | 5         | Flags\[4:0\]: node ID nguồn                  |
| prio_class    | **IN**  | 3         | Byte 2: bậc ưu tiên                          |
| eta           | **IN**  | 8         | Byte 3: ETA (giây)                           |
| uart_tx_busy  | **IN**  | 1         | Từ uart_tx_core                              |
| uart_tx_data  | **OUT** | 8         | Byte đẩy vào uart_tx_core                    |
| uart_tx_start | **OUT** | 1         | Strobe đến uart_tx_core                      |
| tx_done       | **OUT** | 1         | 1-cycle pulse khi toàn bộ 5 byte đã được gửi |

### 9.13 cascade_rx.v

Nhận 5 byte từ uart_rx_core. Kiểm tra header 0xCA và XOR checksum. Output parsed packet.

| **Port Name**  | **Dir** | **Width** | **Description**                                      |
| -------------- | ------- | --------- | ---------------------------------------------------- |
| clk            | **IN**  | 1         | System clock                                         |
| rst_n          | **IN**  | 1         | Active-low reset                                     |
| uart_rx_data   | **IN**  | 8         | Received byte từ uart_rx_core                        |
| uart_rx_valid  | **IN**  | 1         | Strobe: byte mới từ uart_rx_core                     |
| pkt_valid      | **OUT** | 1         | Pulse: packet hợp lệ đã nhận đủ 5 byte + checksum OK |
| pkt_flag_valid | **OUT** | 1         | Flags\[7\] - parsed                                  |
| pkt_flag_dir   | **OUT** | 1         | Flags\[6\] - 0=NS, 1=EW                              |
| pkt_flag_clear | **OUT** | 1         | Flags\[5\] - xe đã qua                               |
| pkt_src_id     | **OUT** | 5         | Flags\[4:0\]                                         |
| pkt_class      | **OUT** | 3         | Byte 2: priority_class                               |
| pkt_eta        | **OUT** | 8         | Byte 3: ETA                                          |
| pkt_error      | **OUT** | 1         | Pulse: checksum mismatch hoặc header sai             |

### 9.14 cascade_ctrl.v

Green Wave coordinator. Nhận local prio_active + dir_valid. Nhận packet từ LEFT/RIGHT port. Quyết định pre-clear request và forward packet với ETA cộng thêm inter-node delay.

| **Port Name**        | **Dir** | **Width** | **Description**                                                                                  |
| -------------------- | ------- | --------- | ------------------------------------------------------------------------------------------------ |
| clk                  | **IN**  | 1         | System clock                                                                                     |
| rst_n                | **IN**  | 1         | Active-low reset                                                                                 |
| node_id              | **IN**  | 2         | DIP switch - ID của node này (0-3)                                                               |
| prio_active          | **IN**  | 1         | Từ traffic_fsm - FSM đang trong priority states                                                  |
| winner_class         | **IN**  | 3         | Từ priority_arbiter - bậc xe đang ưu tiên                                                        |
| dir_out              | **IN**  | 2         | Từ direction_detector - hướng xe đi                                                              |
| dir_valid            | **IN**  | 1         | Từ direction_detector - strobe xác nhận hướng                                                    |
| dir_confidence       | **IN**  | 1         | Từ direction_detector - 1=tin cậy cao, 0=fallback. Khi 0: vẫn forward packet nhưng log để debug. |
| left_pkt_valid       | **IN**  | 1         | Packet hợp lệ từ LEFT cascade_rx                                                                 |
| left_pkt_flag_dir    | **IN**  | 1         | Direction từ LEFT packet                                                                         |
| left_pkt_flag_clear  | **IN**  | 1         | Clear flag từ LEFT packet                                                                        |
| left_pkt_class       | **IN**  | 3         | Class từ LEFT packet                                                                             |
| left_pkt_eta         | **IN**  | 8         | ETA từ LEFT packet                                                                               |
| right_pkt_valid      | **IN**  | 1         | Packet hợp lệ từ RIGHT cascade_rx                                                                |
| right_pkt_flag_dir   | **IN**  | 1         | Direction từ RIGHT packet                                                                        |
| right_pkt_flag_clear | **IN**  | 1         | Clear flag từ RIGHT packet                                                                       |
| right_pkt_class      | **IN**  | 3         | Class từ RIGHT packet                                                                            |
| right_pkt_eta        | **IN**  | 8         | ETA từ RIGHT packet                                                                              |
| left_tx_send         | **OUT** | 1         | Strobe đến LEFT cascade_tx                                                                       |
| left_tx_valid        | **OUT** | 1         | Flags\[7\] cho LEFT packet                                                                       |
| left_tx_dir          | **OUT** | 1         | Direction cho LEFT packet                                                                        |
| left_tx_clear        | **OUT** | 1         | Clear flag cho LEFT packet                                                                       |
| left_tx_class        | **OUT** | 3         | Class cho LEFT packet                                                                            |
| left_tx_eta          | **OUT** | 8         | ETA cho LEFT packet (= nhận + INTER_NODE_DELAY)                                                  |
| right_tx_send        | **OUT** | 1         | Strobe đến RIGHT cascade_tx                                                                      |
| right_tx_valid       | **OUT** | 1         | Flags\[7\] cho RIGHT packet                                                                      |
| right_tx_dir         | **OUT** | 1         | Direction cho RIGHT packet                                                                       |
| right_tx_clear       | **OUT** | 1         | Clear flag cho RIGHT packet                                                                      |
| right_tx_class       | **OUT** | 3         | Class cho RIGHT packet                                                                           |
| right_tx_eta         | **OUT** | 8         | ETA cho RIGHT packet                                                                             |
| preclear_req         | **OUT** | 1         | Yêu cầu pre-clear đến traffic_fsm (cascade_preclear)                                             |

### 9.15 rs485_dir_ctrl.v

Điều khiển chân DE của MAX485. Assert DE trước khi TX, giữ DE HIGH trong suốt frame, deassert sau 2 baud cycles post tx_done. Có 2 instance: LEFT port và RIGHT port.

| **Port Name** | **Dir** | **Width** | **Description**                                         |
| ------------- | ------- | --------- | ------------------------------------------------------- |
| clk           | **IN**  | 1         | System clock                                            |
| rst_n         | **IN**  | 1         | Active-low reset                                        |
| tx_start      | **IN**  | 1         | Từ cascade_tx: chuẩn bị truyền (assert DE ngay lập tức) |
| tx_done       | **IN**  | 1         | Từ uart_tx_core: đã truyền xong bit Stop                |
| rs485_de      | **OUT** | 1         | Đến MAX485 DE pin - HIGH=transmit, LOW=receive          |
| tx_enable     | **OUT** | 1         | Gate cho uart_tx_start - chỉ HIGH khi DE đã settled     |

**ⓘ** Guard delay = 2 × baud_period = 2 × (50MHz / 9600) = 10,416 clock cycles. Dùng counter 14-bit.

### 9.16 traffic_node.v

Top-level wrapper cho 1 ngã tư. Kết nối tất cả module. Expose flat I/O cho pin constraint. Không có CDC crossing nội bộ - tất cả signal đã ở CLK_SYS.

| **Port Name**   | **Dir** | **Width** | **Description**                |
| --------------- | ------- | --------- | ------------------------------ |
| clk             | **IN**  | 1         | System clock 50 MHz            |
| rst_n           | **IN**  | 1         | Synchronized reset từ rst_sync |
| node_id         | **IN**  | 2         | DIP switch - node ID           |
| spi_miso        | **IN**  | 1         | MISO từ 4 RC522                |
| spi_sck         | **OUT** | 1         | SCK đến 4 RC522                |
| spi_mosi        | **OUT** | 1         | MOSI đến 4 RC522               |
| cs_n            | **OUT** | 4         | CS_N\[3:0\] - N/S/E/W          |
| left_rx         | **IN**  | 1         | RS485 RX từ LEFT neighbor      |
| left_tx         | **OUT** | 1         | RS485 TX đến LEFT neighbor     |
| left_de         | **OUT** | 1         | MAX485 DE cho LEFT port        |
| right_rx        | **IN**  | 1         | RS485 RX từ RIGHT neighbor     |
| right_tx        | **OUT** | 1         | RS485 TX đến RIGHT neighbor    |
| right_de        | **OUT** | 1         | MAX485 DE cho RIGHT port       |
| ns_r            | **OUT** | 1         | NS Red LED                     |
| ns_y            | **OUT** | 1         | NS Yellow LED                  |
| ns_g            | **OUT** | 1         | NS Green LED                   |
| ew_r            | **OUT** | 1         | EW Red LED                     |
| ew_y            | **OUT** | 1         | EW Yellow LED                  |
| ew_g            | **OUT** | 1         | EW Green LED                   |
| prio_active_led | **OUT** | 1         | Priority Active indicator LED  |

### 9.17 traffic_top.v

FPGA top-level. Instantiates PLL, rst_sync, và traffic_node. Tất cả port là physical FPGA pin.

| **Port Name** | **Dir** | **Width** | **Description**                           |
| ------------- | ------- | --------- | ----------------------------------------- |
| clk_osc       | **IN**  | 1         | Onboard oscillator input                  |
| rst_btn_n     | **IN**  | 1         | Push button reset (active-low, raw async) |
| node_id       | **IN**  | 2         | DIP switch \[1:0\]                        |
| spi_miso      | **IN**  | 1         | Physical pin - MISO                       |
| spi_sck       | **OUT** | 1         | Physical pin - SCK                        |
| spi_mosi      | **OUT** | 1         | Physical pin - MOSI                       |
| cs_n          | **OUT** | 4         | Physical pins - CS_N\[3:0\]               |
| left_rx       | **IN**  | 1         | Physical pin - LEFT RS485 RX              |
| left_tx       | **OUT** | 1         | Physical pin - LEFT RS485 TX              |
| left_de       | **OUT** | 1         | Physical pin - LEFT MAX485 DE             |
| right_rx      | **IN**  | 1         | Physical pin - RIGHT RS485 RX             |
| right_tx      | **OUT** | 1         | Physical pin - RIGHT RS485 TX             |
| right_de      | **OUT** | 1         | Physical pin - RIGHT MAX485 DE            |
| led_ns        | **OUT** | 3         | NS LEDs \[R,Y,G\]                         |
| led_ew        | **OUT** | 3         | EW LEDs \[R,Y,G\]                         |
| led_prio      | **OUT** | 1         | Priority Active LED                       |

# 10 Verification Plan

## 10.1 ModelSim - 14 Scenarios

| **#** | **Test Case**                                  | **Pass Criterion**                                                                                                                                                                                                                                               |
| ----- | ---------------------------------------------- | ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| 1     | Normal cycle                                   | G(30s)→Y(5s)→AR(2s)→G(25s)→Y(5s)→AR(2s). Timer values match CNT\_\*.                                                                                                                                                                                             |
| 2     | Priority NS during EW green                    | EW→Y→AR→S_PRIO_CLEAR→S_PRIO_HOLD(NS_G)→hold while tag detected                                                                                                                                                                                                   |
| 3     | Priority EW during NS green                    | NS→Y→AR→S_PRIO_CLEAR→S_PRIO_HOLD(EW_G)                                                                                                                                                                                                                           |
| 4     | Priority during Yellow                         | Complete yellow, then S_PRIO_CLEAR                                                                                                                                                                                                                               |
| 5     | Priority timeout 60s                           | Auto S_PRIO_EXIT sau CNT_PRIO_TMO                                                                                                                                                                                                                                |
| 6     | Tag lost mid-hold                              | HOLD→EXIT(Y,5s)→RECOV(AR,2s)→Normal                                                                                                                                                                                                                              |
| 7     | Conflict class 5 vs class 3                    | Class 5 thắng                                                                                                                                                                                                                                                    |
| 8     | Conflict same class                            | Green direction giữ nguyên (tiebreak)                                                                                                                                                                                                                            |
| 9     | Direction: NORTH exit                          | tag sequence 0100→0110→1010→1000→0000 → dir=NORTH                                                                                                                                                                                                                |
| 10    | Direction: EAST exit                           | sequence kết thúc bằng E last → dir=EAST                                                                                                                                                                                                                         |
| 11    | Direction tie                                  | 2 reader cùng mất tick → N>E>S>W tiebreak                                                                                                                                                                                                                        |
| 12    | Direction timeout                              | any_detected stuck 10s → default, dir_valid pulse                                                                                                                                                                                                                |
| 13    | Green Wave 2 nodes                             | Node A detect → TX packet → Node B pre-clear ≤5ms                                                                                                                                                                                                                |
| 14    | SPI capture timing + reset synchronizer verify | (a) Confirm spi_sck = 5MHz (clk_div=4) on GAO. (b) rst_sync: assert rst_btn_n async → rst_sync_n deasserts synchronously within 2 clk_sys rising edges after rst_btn_n release. (c) After rst_sync_n releases, rc522_driver_x4 enters INIT state within 1 cycle. |

## 10.2 FPGA On-Board

| **#** | **Test**            | **Method**                                                            |
| ----- | ------------------- | --------------------------------------------------------------------- |
| 1     | Normal LED cycle    | Visual verify R/Y/G sequencing                                        |
| 2     | RC522 tag scan      | Tap card → priority LED, FSM enters preemption                        |
| 3     | Direction detection | Move tag toward one reader → dir_out correct trên GAO                 |
| 4     | Green Wave 4 boards | Scan tại Board A → B, C, D pre-clear tuần tự                          |
| 5     | Priority conflict   | 2 thẻ khác class → higher class wins                                  |
| 6     | GAO probe           | Probe: state\[3:0\], dir_out\[1:0\], dir_valid, DE timing, cascade TX |

# 11 Bill of Materials

## 11.1 Per Node (1 Intersection)

| **#** | **Component**           | **Qty**      | **Cost**  | **Notes**                   |
| ----- | ----------------------- | ------------ | --------- | --------------------------- |
| 1     | Gowin ACG525 (GW5A-25A) | 1            | \$30-50   | FPGA board + USB programmer |
| 2     | MFRC522 (RC522) module  | 4            | \$1-2 ea  | N/S/E/W · shared SPI · 3.3V |
| 3     | MIFARE Classic 1K card  | 5            | \$0.10 ea | 1 card/priority class       |
| 4     | MAX485 / SP3485 IC      | 2            | \$0.50 ea | LEFT + RIGHT cascade port   |
| 5     | Twisted pair cable      | ~2m          | \$0.50    | RS485 A+/B−                 |
| 6     | LEDs R/Y/G + resistors  | 7+           | ~\$1      | NS + EW + priority          |
| 7     | Breadboard / PCB        | 1            | ~\$3      | RC522 mounting              |
| -     |                         | **Per node** |           | **≈ \$45-68**               |

## 11.2 Total - 4 Nodes

| **Item**      | **Qty (total)**         | **Cost (total)** |
| ------------- | ----------------------- | ---------------- |
| Gowin ACG525  | 4                       | \$120-200        |
| RC522 modules | 16                      | \$16-32          |
| MIFARE cards  | 5 (shared)              | \$0.50           |
| MAX485 ICs    | 8                       | \$4              |
| Cable + misc  | -                       | ~\$10            |
| **TOTAL**     | **4 boards · 16 RC522** | **≈ \$180-292**  |