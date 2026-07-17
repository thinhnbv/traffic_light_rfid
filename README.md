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
├── tb/                         # Testbench Suite
├── sim/                        # Simulation Workspace for ModelSim
├── spec/                       # System Specifications & Documentation
├── old-version/                # Legacy codes and configurations
├── work/                       # Gowin Synthesizer build metadata
├── traffic_top.cst             # Primary Pin Constraint file (Gowin)
├── uid.txt                     # List of registered RFID card UIDs
└── README.md                   # This documentation
```

---

## 🧪 Simulation and Verification

The system is validated through a self-checking testbench (`tb/tb_traffic_top.v`) consisting of **15 testcases** verifying:

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

## 📊 Synthesis & Hardware Footprint (Target)

* **Target Device**: Gowin GW5A-LV25MG121
* **Expected Synthesis Frequency (Fmax)**: > 100 MHz (easily satisfying the 50 MHz clock requirement)
* **Estimated Logic Footprint**: Low utilization (< 5% of device logic elements), making it highly optimized for edge deployment or integration into a larger SoC.

---

## 📄 License

This project is licensed under the MIT License - see the LICENSE file for details.
