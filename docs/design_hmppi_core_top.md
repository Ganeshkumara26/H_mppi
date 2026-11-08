# Design: `hmppi_core_top.sv` — H-MPPI Accelerator Top-Level Integration

**Spec Reference:** §3 (H-MPPI Architecture), §3.7 (Top-Level Integration)
**Target:** Standalone H-MPPI & Integrated Monolithic SoC
**Hardware Cost:** 87 DSP48E1, 23,200 LUTs, 12,400 Flip-Flops (full accelerator)

---

## 1. Purpose

`hmppi_core_top.sv` is the structural top-level that integrates all H-MPPI accelerator submodules into a single port-clean entity. From the SoC's perspective, it is a black box with an AHB-Lite slave port, a control vector output bus, and a done interrupt. Internally, it contains the entire MPPI pipeline: 4 replicated rollout lanes (each with kinematics, slosh, and cost), the Sobol QMC generator, the parallel reduction tree, the exponential weight normalizer, and the control arbiter.

Like `tide_top`, this module contains **no behavioral logic** — only instances, wires, and the configuration register decode. All computation happens inside the submodules.

## 2. Interface

```systemverilog
module hmppi_core_top (
    input  logic        clk_i,             // 50 MHz system clock (or 500 MHz for ASAP7)
    input  logic        rst_ni,            // Active-low synchronous reset

    // === AHB-Lite Slave Interface (CPU access) ===
    input  logic        hsel_i,
    input  logic [11:0] haddr_i,           // 4 KB decode space
    input  logic        hwrite_i,
    input  logic [31:0] hwdata_i,
    input  logic [1:0]  htrans_i,
    input  logic [2:0]  hsize_i,
    output logic [31:0] hrdata_o,
    output logic        hready_o,
    output logic        hresp_o,

    // === Initial State Input (from TIDE EKF, directly wired or via registers) ===
    input  logic signed [15:0] tide_pos_x_i, tide_pos_y_i, tide_pos_z_i,
    input  logic signed [15:0] tide_vel_x_i, tide_vel_y_i, tide_vel_z_i,
    input  logic signed [15:0] tide_omega_x_i, tide_omega_y_i, tide_omega_z_i,
    input  logic        tide_state_valid_i,

    // === Control Output (to motor mixer / actuators) ===
    output logic signed [15:0] ctrl_vx_o, ctrl_vy_o, ctrl_vz_o, ctrl_dpsi_o,
    output logic        ctrl_valid_o,

    // === Interrupt & Status ===
    output logic        irq_done_o,        // MPPI frame complete
    output logic        busy_o,            // Accelerator processing
    output logic [15:0] frame_cycles_o     // Actual cycle count of last frame (diagnostic)
);
```

## 3. Submodule Instantiation Hierarchy

```
hmppi_core_top
├── hmppi_control_arbiter          (FSM scheduler, weighted average)
├── sobol_qmc_4d                   (4D quasi-random perturbation generator)
├── rollout_lane[0]                (Lane 0)
│   ├── attitude_kinematics_engine (6-DoF propagator)
│   ├── slosh_chebyshev_evaluator  (Chebyshev slosh model)
│   └── trajectory_cost_accumulator(Multi-objective cost)
├── rollout_lane[1]                (Lane 1, identical to Lane 0)
├── rollout_lane[2]                (Lane 2)
├── rollout_lane[3]                (Lane 3)
├── parallel_reduction_tree        (128→1 minimum finder)
├── fexp2_lut_taylor               (Base-2 exponential + normalizer)
├── hmppi_cost_buffer              (128×16 cost storage RAM)
├── hmppi_pert_buffer              (128×64 perturbation storage RAM)
├── hmppi_ahb_regs                 (AHB slave, config register decode)
└── hmppi_nom_traj_buf             (16×64 nominal trajectory buffer)
```

### Lane Replication

The 4 rollout lanes are identical instances. They share:
- Configuration constants (gyroscopic coefficients, Chebyshev constants, cost weights) — broadcast from configuration registers.
- Sobol perturbation outputs — the Sobol module generates 4 independent perturbation vectors per cycle.
- The parallel reduction tree — receives 4 cost values per batch.

They do NOT share:
- State registers (each lane propagates an independent trajectory).
- DSP48E1 slices (each lane has its own multipliers).

## 4. AHB-Lite Register Map

| Offset | Register | Access | Description |
|---|---|---|---|
| `0x000` | `CTRL` | W | bit 0 = frame_start, bit 1 = abort |
| `0x004` | `STATUS` | R | bit 0 = busy, bit 1 = done, bit 2 = error |
| `0x008` | `FRAME_CYCLES` | R | Actual cycle count of last frame |
| `0x00C` | `BEST_TRAJ_IDX` | R | Index of lowest-cost trajectory (7-bit) |
| `0x010` | `BEST_COST` | R | Minimum cost ρ (Q4.12) |
| `0x014` | `CTRL_VX` | R | Output control Vx (Q4.12) |
| `0x018` | `CTRL_VY` | R | Output control Vy |
| `0x01C` | `CTRL_VZ` | R | Output control Vz |
| `0x020` | `CTRL_DPSI` | R | Output control yaw rate |
| `0x030–0x03F` | `INIT_STATE` | R/W | Initial state vector (loaded from TIDE or manually) |
| `0x040–0x04F` | `GOAL_POS` | R/W | Goal position (x, y, z) |
| `0x050–0x05F` | `OBS_0_POS` | R/W | Obstacle 0 position (x, y, z) |
| `0x060–0x06F` | `OBS_1_POS` | R/W | Obstacle 1 position |
| `0x070–0x07F` | `OBS_2_POS` | R/W | Obstacle 2 position |
| `0x080–0x08F` | `OBS_3_POS` | R/W | Obstacle 3 position |
| `0x090` | `OBS_VALID` | R/W | Obstacle validity mask (4-bit) |
| `0x094` | `R_SAFE_SQ` | R/W | Safety radius² (Q4.12) |
| `0x098` | `R_OUTER_SQ` | R/W | Outer radius² (Q4.12) |
| `0x0A0` | `W_POS` | R/W | Position cost weight |
| `0x0A4` | `W_SLOSH` | R/W | Slosh penalty weight |
| `0x0A8` | `W_CTRL` | R/W | Control effort weight |
| `0x0AC` | `W_OBS` | R/W | Obstacle avoidance weight |
| `0x0B0` | `LAMBDA` | R/W | Temperature parameter λ (Q4.12) |
| `0x0B4` | `SIGMA` | R/W | Exploration noise scaling σ (Q4.12) |
| `0x0C0–0x0CF` | `KIN_COEFF` | R/W | Kinematics coefficients (Kx, Ky, Kz, Gx, Gy, Gz, dt, dt²/2) |
| `0x0D0–0x0DF` | `SLOSH_COEFF` | R/W | Chebyshev slosh coefficients (A, B, ωn², 2ζωn, 1/l, torque_gain, mass_scale) |
| `0x100–0x1FF` | `NOM_TRAJ` | R/W | Nominal control trajectory (16 steps × 4 values × 16-bit) |

## 5. Data Flow

The complete H-MPPI data flow for one frame:

```
                    ┌──────────────────────────────────────────────────┐
                    │              hmppi_core_top                      │
                    │                                                  │
  frame_start ─────►│  ┌─────────┐                                    │
                    │  │ Arbiter  │──► sobol_qmc_4d ──► δu[k,t]       │
                    │  │  (FSM)   │                       │            │
                    │  └────┬─────┘                       ▼            │
                    │       │        ┌──────────────────────────┐      │
  init_state ──────►│       │        │ 4 × Rollout Lane         │      │
                    │       │        │  kin → slosh → cost      │      │
                    │       │        └──────────┬───────────────┘      │
                    │       │                   │ S[k] (×4 per batch)  │
                    │       │                   ▼                      │
                    │       │        ┌──────────────────────┐          │
                    │       │        │ cost_buffer (128×16)  │          │
                    │       │        └──────────┬───────────┘          │
                    │       │                   │                      │
                    │       │        ┌──────────▼───────────┐          │
                    │       │        │ parallel_reduction    │──► ρ    │
                    │       │        └──────────────────────┘   │      │
                    │       │                                   │      │
                    │       │        ┌──────────────────────┐   │      │
                    │       │        │ fexp2_weight_norm     │◄──┘     │
                    │       │        └──────────┬───────────┘          │
                    │       │                   │ ŵ[k]                 │
                    │       │                   ▼                      │
                    │       │        ┌──────────────────────┐          │
                    │       └───────►│ weighted_average     │          │
                    │                └──────────┬───────────┘          │
                    │                           │ u*                   │
  ctrl_out ◄────────│───────────────────────────┘                     │
                    │                                                  │
                    └──────────────────────────────────────────────────┘
```

## 6. Cost Buffer Architecture

The cost buffer stores 128 accumulated trajectory costs for two purposes:
1. The reduction tree reads them to find $\rho$ (though the reduction tree actually processes costs streaming, not from the buffer).
2. The weight normalizer reads them during the softmax phase to compute $S_k - \rho$ for each trajectory.

Implementation: 128 × 16-bit dual-port RAM. Write during rollout (one cost per batch completion = 4 costs per batch write). Read during softmax (sequential scan of all 128 costs).

On Artix-7: 1 BRAM18 (256 bytes, only 256 of 2,048 bytes used). On ASIC: compiled SRAM or register file.

## 7. TIDE-to-MPPI State Transfer

The initial state for MPPI comes from the TIDE EKF's latest estimation output. Two transfer modes are supported:

### 7.1 Direct Wire (Monolithic SoC)
In the integrated `tide_hmppi_soc_top`, the TIDE register file's state registers (r0–r5) are directly wired to H-MPPI's initial state inputs. When the TIDE temporal controller signals `done_o`, the H-MPPI arbiter captures the state on the next cycle and begins its frame.

### 7.2 Register-Mediated (Standalone)
When H-MPPI is used standalone (e.g., with a different EKF), the initial state is loaded via MMIO registers (`INIT_STATE` at offset 0x030). The firmware writes the 9 state values (3 position + 3 velocity + 3 angular velocity) and then triggers `frame_start`.

## 8. Clock and Reset

All submodules share a single clock domain. On FPGA (50 MHz), the total frame time is ~27.3 µs. On ASAP7 ASIC (500 MHz target), the frame time drops to ~2.73 µs — well within the 50 Hz control loop's 20 ms budget.

Reset clears all FSMs to IDLE, all accumulators to zero, and all control outputs to zero. The Sobol generator's state is reset to $\mathbf{X}_0 = \mathbf{0}$.

## 9. Resource Summary

### Artix-7 XC7A35T FPGA

| Resource | Count | % of Device |
|---|---|---|
| LUTs | 23,200 | 69.7% |
| FFs | 12,400 | 18.6% |
| DSP48E1 | 87 | 96.7% |
| BRAM36 | 3 | 6.0% |
| BRAM18 | 4 | 8.0% |

### ASAP7 7nm ASIC

| Metric | Value |
|---|---|
| Cell count | 402,673 (monolithic SoC including TIDE + CPU) |
| Estimated core area | ~0.12 mm² (H-MPPI only) |
| Target clock | 500 MHz |
| Dynamic power (estimated) | ~15 mW at 500 MHz |

### SCL 180nm ASIC (Standalone H-MPPI)

| Metric | Value |
|---|---|
| Gate count | ~180,000 |
| Estimated core area | ~2.1 mm² |
| Target clock | 50 MHz |

## 10. End-to-End Loop Timing

The complete sensor-to-actuator loop through TIDE + H-MPPI:

| Phase | Cycles | Time @ 50 MHz | Time @ 500 MHz |
|---|---|---|---|
| TIDE EKF predict + update | ~800 | 16.0 µs | 1.6 µs |
| State transfer (TIDE → MPPI) | ~4 | 0.08 µs | 0.008 µs |
| H-MPPI rollout (128 traj × 16 steps) | ~1,216 | 24.3 µs | 2.43 µs |
| H-MPPI reduction + softmax | ~268 | 5.36 µs | 0.54 µs |
| H-MPPI weighted average + output | ~12 | 0.24 µs | 0.024 µs |
| **Total end-to-end** | **~2,300** | **46.0 µs** | **4.6 µs** |

This is well within the 20 ms (50 Hz) control loop budget, leaving ~19.95 ms for firmware processing, telemetry, and other tasks.

## 11. Verification Plan

| Test | Method | Pass Criterion |
|---|---|---|
| AHB register read/write: all config registers | Verilator | Correct data round-trip |
| Frame start → done: measure cycle count | Verilator | Matches schedule (~1,366 cycles) |
| Control output: all costs equal → u* = v_nom | Verilator | Control output = nominal |
| Control output: single best trajectory → u* ≈ best | Verilator | Matches best perturbation |
| Full golden model: random scenario → compare u* | Verilator + Python | Within 2 LSBs of golden model |
| Obstacle avoidance: trajectory through obstacle → high cost | Verilator | Trajectory avoided in output |
| TIDE integration: TIDE done → MPPI starts automatically | Verilator | Correct state transfer, valid output |
| Back-to-back frames: frame N+1 starts after frame N | Verilator | No state corruption |
| Power-on reset: all outputs zero, FSM in IDLE | Verilator | Clean initialization |
| Interrupt: irq_done_o pulse at frame completion | Verilator | Correct timing |

## 12. Implementation Notes for the Coder

1. **This module is purely structural.** No `always` blocks. Only `wire`, `assign`, and module instances.

2. **Use `generate` blocks for the 4 rollout lanes.** Each lane is identical with only the `LANE_ID` parameter differing (used for debug identification).

3. **The AHB register decode (`hmppi_ahb_regs`)** should be a separate submodule, not inline logic. Keep the top-level clean.

4. **DSP sharing:** In the current design, each lane owns its DSP slices exclusively. If area is critical, consider time-sharing DSPs across lanes (lane 0 uses DSPs on even cycles, lane 1 on odd cycles). This halves DSP count but doubles frame time.

5. **The `frame_cycles_o` diagnostic counter** should be a free-running 16-bit counter that starts at 0 on `frame_start_i` and freezes on `frame_done_o`. This is invaluable for performance validation.

6. **Import `hmppi_pkg`** for all parameters. The top-level should re-export key parameters as module parameters for the SoC integrator.

7. **Reset behavior:** All FSMs in IDLE, all outputs zero, `busy_o = 0`, `irq_done_o = 0`.
