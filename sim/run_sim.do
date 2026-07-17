# Tạo thư mục thư viện làm việc nếu chưa có
vlib work
vmap work work

# Compile các module Verilog từ thư mục rtl
vlog -work work ../rtl/rst_sync.v
vlog -work work ../rtl/cdc_sync.v
vlog -work work ../rtl/phase_timer.v
vlog -work work ../rtl/spi_master.v
vlog -work work ../rtl/tag_priority_lut.v
vlog -work work ../rtl/priority_arbiter.v
vlog -work work ../rtl/direction_detector.v
vlog -work work ../rtl/rc522_driver_x4.v
vlog -work work ../rtl/uart_tx_core.v
vlog -work work ../rtl/uart_rx_core.v
vlog -work work ../rtl/cascade_tx.v
vlog -work work ../rtl/cascade_rx.v
vlog -work work ../rtl/cascade_ctrl.v
vlog -work work ../rtl/rs485_dir_ctrl.v
vlog -work work ../rtl/traffic_fsm.v
vlog -work work ../rtl/traffic_node.v
vlog -work work ../rtl/traffic_top.v

# Compile Testbench (ở thư mục cha hoặc thư mục con tương ứng)
vlog -work work ../tb/tb_traffic_top.v


# Khởi chạy mô phỏng với macro SIMULATION để rút ngắn thời gian timer
vsim -voptargs="+acc" -t 1ns +define+SIMULATION work.tb_traffic_top

# Thêm các tín hiệu cần quan sát vào Waveform
add wave -noupdate -divider {System Clock & Reset}
add wave -noupdate /tb_traffic_top/clk
add wave -noupdate /tb_traffic_top/rst_n

add wave -noupdate -divider {FSM State & Decoded Name}
add wave -noupdate -radix hexadecimal /tb_traffic_top/cur_state
add wave -noupdate -radix ascii /tb_traffic_top/state_name

add wave -noupdate -divider {LED Outputs}
add wave -noupdate -color {Red} /tb_traffic_top/ns_r
add wave -noupdate -color {Yellow} /tb_traffic_top/ns_y
add wave -noupdate -color {Green} /tb_traffic_top/ns_g
add wave -noupdate -color {Red} /tb_traffic_top/ew_r
add wave -noupdate -color {Yellow} /tb_traffic_top/ew_y
add wave -noupdate -color {Green} /tb_traffic_top/ew_g
add wave -noupdate -color {Cyan} /tb_traffic_top/led_prio

add wave -noupdate -divider {RFID Stimulus Inputs}
add wave -noupdate -radix binary /tb_traffic_top/sim_tag_detected
add wave -noupdate -radix hexadecimal /tb_traffic_top/sim_uid_n
add wave -noupdate -radix hexadecimal /tb_traffic_top/sim_uid_s
add wave -noupdate -radix hexadecimal /tb_traffic_top/sim_uid_e
add wave -noupdate -radix hexadecimal /tb_traffic_top/sim_uid_w

add wave -noupdate -divider {Priority Resolution}
add wave -noupdate /tb_traffic_top/arb_ev_ns
add wave -noupdate /tb_traffic_top/arb_ev_ew
add wave -noupdate -radix unsigned /tb_traffic_top/arb_winner

add wave -noupdate -divider {Direction Inference}
add wave -noupdate -radix binary /tb_traffic_top/u_dir/falling
add wave -noupdate -radix unsigned /tb_traffic_top/dir_out
add wave -noupdate /tb_traffic_top/dir_valid
add wave -noupdate /tb_traffic_top/dir_confidence

# Cấu hình định dạng hiển thị waveform
configure wave -timelineunits ns

# Chạy mô phỏng toàn bộ
run -all
