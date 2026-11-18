# UVM simulation for traffic_fsm (QuestaSim)
# Run:  vsim -do run_questa.do   (from sim/ directory)

# 1. Work library
vlib work
vmap work work

# 2. UVM package (Questa 10.x ships uvm-1.1d); UVM_NO_DPI avoids the Questa-10.2c
#    DPI TFDB version-read failure (uvm_dpi.svh pulls dpi functions not built here).
vlog -work work +define+UVM_NO_DPI +incdir+C:/questasim64_10.2c/verilog_src/uvm-1.1d/src \
  +incdir+C:/questasim64_10.2c/verilog_src/uvm-1.1d/src/base \
  +incdir+C:/questasim64_10.2c/verilog_src/uvm-1.1d/src/comps \
  +incdir+C:/questasim64_10.2c/verilog_src/uvm-1.1d/src/seq \
  +incdir+C:/questasim64_10.2c/verilog_src/uvm-1.1d/src/tlm1 \
  +incdir+C:/questasim64_10.2c/verilog_src/uvm-1.1d/src/tlm2 \
  +incdir+C:/questasim64_10.2c/verilog_src/uvm-1.1d/src/dpi \
  +incdir+C:/questasim64_10.2c/verilog_src/uvm-1.1d/src/reg \
  C:/questasim64_10.2c/verilog_src/uvm-1.1d/src/uvm_pkg.sv

# 3. Virtual interface (SystemVerilog)
vlog -sv -work work +incdir+C:/questasim64_10.2c/verilog_src/uvm-1.1d/src \
  ../tb/uvm/traffic_if.sv

# 4. Single UVM package containing all testbench classes
vlog -sv -work work +incdir+C:/questasim64_10.2c/verilog_src/uvm-1.1d/src \
  ../tb/uvm/traffic_uvm_pkg.sv

# 5. RTL modules (Verilog)
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

# 6. Top-level testbench
vlog -sv -work work +incdir+C:/questasim64_10.2c/verilog_src/uvm-1.1d/src \
  ../tb/uvm/tb_traffic_top_uvm.sv

# After compile, run:  vsim -voptargs="+acc" -t 1ns -coverage +define+SIMULATION +uvm work.tb_traffic_top_uvm -c -do "run -all; quit"
