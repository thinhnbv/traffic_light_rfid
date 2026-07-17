# TLC-RFID: An FPGA-Based Traffic Light Controller with Multi-Tier RFID Preemption and Green Wave Cascade

[![Verilog](https://img.shields.io/badge/Language-Verilog%20HDL-orange.svg)](#)
[![Toolchain](https://img.shields.io/badge/FPGA-Gowin%20GW5A--25A-blue.svg)](#)
[![Simulation](https://img.shields.io/badge/Simulation-ModelSim%20%2F%20Questa-green.svg)](#)
[![License](https://img.shields.io/badge/License-MIT-brightgreen.svg)](#)

A high-performance, real-time, FPGA-based intelligent Traffic Light Controller (TLC) incorporating multi-tier RFID emergency preemption, priority collision arbitration, direction-aware vehicle exit detection, dual-axis countdown displays, and RS485-based Green Wave synchronization.

Designed and verified for safety-critical edge deployment on the **Xiaomeige ACG525 (Gowin GW5A-LV25MG121)** FPGA platform.

---

## 🌟 Key Features

1. **Deterministic 10-State FSM Control**: Handles regular phases (Green, Yellow, All-Red) along with dedicated preemptive clear, hold, and recovery states.
2. **Multi-Reader Shared SPI Driver**: Multiplexes 4 x RC522 RFID readers over a single SPI bus using active-low Chip Selects (`cs_n`).
3. **Multi-Tier Priority Arbitration**: Classifies emergency vehicle UIDs into 5 tiers (Fire Truck, Police/Military, Ambulance, Disaster Relief, Funeral) and resolves multi-axis conflict using real-time FSM context.
4. **Direction Detection**: Tracks logical vehicle progression (entry vs. exit) by reading sequential tag detection histories from physical direction readers.
5. **Daisy-Chained 74HC595 Display Driver**: Multi-plexes dual 4-digit 7-segment displays to show countdown timers for both North-South (NS) and East-West (EW) axes simultaneously.
6. **Green Wave Cascade**: Communicates emergency preemption status to neighboring intersections via customized UART-over-RS485 packet structures.

---

## 📂 Project Structure

```
Traffic_Light_RFID/
├── rtl/                        # Source Verilog RTL Files
│   ├── traffic_top.v           # Physical top-level (Gowin target wrapper)
│   ├── traffic_node.v          # Functional top-level for 1 intersection
│   ├── traffic_fsm.v           # 10-state TLC state machine logic
│   ├── phase_timer.v           # Generic down-counter for phase timing
│   ├── rc522_driver_x4.v       # Shared SPI reader driver for 4x RC522
│   ├── spi_master.v            # Standard SPI Master core
│   ├── tag_priority_lut.v      # Priority tier LUT for registered tag UIDs
│   ├── priority_arbiter.v      # Resolves multi-axis priority conflicts
│   ├── direction_detector.v    # Estimates vehicle entrance/exit confidence
│   ├── seg7_countdown.v        # 74HC595 shift register 7-segment display driver
│   ├── cascade_ctrl.v          # Coordination logic for RS485 data packets
│   ├── cascade_tx.v            # Packet assembly and serialization
│   ├── cascade_rx.v            # Packet parsing and deserialization
│   ├── uart_tx_core.v          # Customized UART Transmitter
│   ├── uart_rx_core.v          # Customized UART Receiver
│   ├── rs485_dir_ctrl.v        # Automates MAX485 driver enable direction
│   ├── rst_sync.v              # Hardware Reset Synchronizer
│   └── cdc_sync.v              # Clock Domain Crossing Synchronizer (for simulation)
├── tb/                         # Testbench Suite
│   └── tb_traffic_top.v        # Comprehensive self-checking testbench (15 Scenarios)
├── sim/                        # Simulation Directory
│   └── run_sim.do              # ModelSim automated simulation script
├── traffic_top.cst             # Physical Pin Mapping constraint file (Gowin)
├── uid.txt                     # List of registered RFID card UIDs
└── README.md                   # This documentation
```

---

## 🔧 Physical Pin Constraints (ACG525 Board)

The pin assignments mapped in the `traffic_top.cst` file target the Xiaomeige ACG525 FPGA development board:

* **System Clock**: `clk_osc` -> `T9` (50MHz crystal oscillator)
* **Hardware Reset**: `rst_btn_n` -> `B16` (KEY0 active-low push button)
* **SPI Bus (RC522)**:
  * SCK: `G14`
  * MOSI: `D17` (Connect to RC522 MOSI)
  * MISO: `D18` (Connect to RC522 MISO)
  * CS_N: West=`F18`, East=`F17`, South=`G18`, North=`G16`
* **7-Segment Display (74HC595)**:
  * DIO: `F4`
  * RCLK: `F3`
  * SCLK: `H4`

---

## 🧪 Simulation and Verification

The system is validated through a self-checking testbench (`tb_traffic_top.v`) consisting of **15 testcases** verifying:
1. Normal cycle transitions (Green $\rightarrow$ Yellow $\rightarrow$ Red).
2. Priority preemption on inactive axis.
3. Preemption overlap on active axis.
4. Emergency override during Yellow phase buffer.
5. Priority timeout (60 seconds escape window).
6. Rapid tag loss recovery logic.
7. Arbitration priority conflicts (Class 5 over Class 3).
8. Same Class tiebreak resolving.
9. Direction confidence calculations.
10. Reset recovery and Green Wave Cascade packets.

### How to Run Simulation

1. Open **ModelSim / QuestaSim**.
2. Navigate to the `sim` folder:
   ```tcl
   cd <your_project_path>/Traffic_Light_RFID/sim
   ```
3. Execute the automated simulation script:
   ```tcl
   do run_sim.do
   ```
4. Check the simulator transcript output for `TEST SUMMARY` (PASS/FAIL indicators).

---

## 🚀 How to Push to GitHub

Follow these steps to push this project to your existing GitHub repository:

1. **Open your terminal** (Git Bash, Command Prompt, or PowerShell) and navigate to the project directory:
   ```bash
   cd D:/Dev/Code/Traffic_Light_RFID
   ```

2. **Initialize Git repository** (if not already done):
   ```bash
   git init
   ```

3. **Stage all files** for commit (creates a snapshot):
   ```bash
   git add .
   ```

4. **Commit the files** with a descriptive message:
   ```bash
   git commit -m "Initial commit of TLC-RFID v4.4 with 7-segment countdown display"
   ```

5. **Set the default branch to main**:
   ```bash
   git branch -M main
   ```

6. **Add the remote origin URL** (replace `YOUR_GITHUB_URL` with your actual repository link, e.g., `https://github.com/username/repo-name.git`):
   ```bash
   git remote add origin YOUR_GITHUB_URL
   ```
   *(Note: If you get an error saying remote origin already exists, run `git remote set-url origin YOUR_GITHUB_URL` instead)*

7. **Push to GitHub**:
   ```bash
   git push -u origin main
   ```

---

## 📄 License
This project is licensed under the MIT License - see the LICENSE file for details.
