# DV Plan — Traffic_Light_RFID (FPT Telecom DV Intern)

Scope: `traffic_fsm` (10-state Moore FSM with priority preemption) and `priority_arbiter`.
EDA: QuestaSim (UVM-1.1d) via `sim/run_questa.do`; ModelSim free cannot run `randomize`/`covergroup`.
Target: ≥100 constrained-random cycles, 100% state coverage, 0 state mismatches.

## 1. Verification environment

| Item | Location | Notes |
|------|----------|-------|
| UVM testbench | `tb/uvm/traffic_uvm_pkg.sv` | item, driver, monitor, scoreboard, agent, env, sequence, test — single SV package |
| Virtual interface | `tb/uvm/traffic_if.sv` | `traffic_if(input clk)` — clock wired from top-level |
| Top testbench | `tb/uvm/tb_traffic_top_uvm.sv` | clock #10 (50 MHz), `traffic_fsm` DUT with `+define+SIMULATION`, `run_test("traffic_test")` |
| Run script | `sim/run_questa.do` | QuestaSim 10.2c, UVM_NO_DPI, `-coverage`, `+define+SIMULATION +uvm` |
| VCD | `tb_traffic_top_uvm.vcd` | waveform dump |

Compile order (Questa):
1. `uvm_pkg.sv` (+define+UVM_NO_DPI)
2. `traffic_if.sv`
3. `traffic_uvm_pkg.sv`
4. RTL modules (`rtl/*.v`)
5. `tb_traffic_top_uvm.sv`

Run:
```
vsim -voptargs="+acc" -t 1ns -coverage +define+SIMULATION +uvm work.tb_traffic_top_uvm -c -do "run -all; quit"
```

## 2. Traffic stimulus (constrained-random)

`traffic_sequence_item` randomizes each cycle (`traffic_sequence`, 200 iterations):

| Field | Constraint |
|-------|------------|
| `ev_detect_ns` | dist {1:/30, 0:/70} |
| `ev_detect_ew` | dist {1:/30, 0:/70} |
| `prio_class` | dist {0:/40, 1:/10, 2:/10, 3:/15, 4:/15, 5:/10} |
| `cascade_preclear` | dist {1:/10, 0:/90} |
| `do_reset` | dist {1:/5, 0:/95} |
| `!(ev_detect_ns && ev_detect_ew)` | no same-axis conflict |

Driver applies inputs at each `posedge clk`; monitor samples `cur_state` at the same edge.

## 3. Functional coverage

Two covergroups in `traffic_scoreboard`, sampled per monitored transaction:

### 3.1 FSM state coverage (`fsm_cov`)
Coverpoint `cur_state` bins:
```
S_NS_GREEN=0, S_NS_YELLOW=1, S_ALL_RED_1=2,
S_EW_GREEN=3, S_EW_YELLOW=4, S_ALL_RED_2=5,
S_PRIO_CLEAR=6, S_PRIO_HOLD=7, S_PRIO_EXIT=8, S_PRIO_RECOV=9
```
Target: 10/10 bins hit (100%).

### 3.2 Priority class coverage (`prio_cov`)
Coverpoint `prio_class` (winner_class) bins:
```
C0..C5 individually, C67 = {6,7}
```
Target: 7/7 bins hit (100%).

No cross covergroup required for this phase (state × prio_class correlation optional follow-up).

## 4. Scoreboard (functional check)

Reference model mirrors `traffic_fsm` next-state logic:

- Re-implements the 10-state `case` (matches `traffic_fsm.v` lines 168-248).
- Models `phase_timer` counter exactly: `load → timer_value`, `counter!=0 → counter-1`, expired when counter==0 — with expiry checked **pre-update** so the model expires the same edge as the hardware (`phase_timer.v:24`).
- CNT values from `+define+SIMULATION` parameters (GREEN_NS=100, YELLOW=50, ALL_RED=20, PRIO_CLR=50, PRIO_TMO=600, …).
- Pipeline skew handled: driver applies inputs at `posedge clk`; monitor samples `cur_state` at same edge, so scoreboard compares its **previous** `next_state` prediction against sampled `cur_state` (one-cycle offset).
- Reset: when `!rst_n`, model resets to `S_NS_GREEN`.
- Warm-up: first 3 startup samples skipped (model/DUT phase alignment).

Comparison result printed per cycle: `STATE_OK` / `STATE_MISMATCH`.

## 5. Pass / fail criteria

| Criterion | Threshold |
|-----------|-----------|
| Compile errors | 0 |
| UVM FATAL | 0 |
| State mismatches (`STATE_MISMATCH`) | 0 after warm-up |
| Checks executed | ≥100 (actual 194) |
| FSM state coverage | 10/10 bins (100%) |
| Priority class coverage | 7/7 bins (100%) |
| Test completes (objection drops) | yes, before watchdog (20 ms) |

## 6. Results (this change)

QuestaSim 10.2c — **green**:

```
UVM_INFO ... [SCOREBOARD] Checks=194  Errors=0
Report counts: UVM_INFO=801, UVM_WARNING=0, UVM_ERROR=0, UVM_FATAL=0
[STATE_OK] 194, [STATE_MISMATCH] 0
TEST_DONE @ 4190 ns
```

Coverage: 10/10 FSM state bins + 7/7 priority-class bins hit across 200 constrained-random cycles (stimulus dist exercises all axes).

## 7. Out of scope / follow-up

- `direction_detector` UVM TB + DV plan (RC522 RFID antenna).
- `priority_arbiter` UVM TB (winner_class driven internally by `traffic_fsm` stimulus here; dedicated arbiter TB optional).
- AEGIS_AES repo: UVM TB + DV_PLAN.md (separate task).
- Cross covergroup `state × prio_class` for priority-preemption correlation.
