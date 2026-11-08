# H-MPPI: High-Throughput Hardware Model Predictive Path Integral Guidance Core

## Overview
H-MPPI is a high-throughput, deterministic hardware accelerator implementing real-time Model Predictive Path Integral (MPPI) stochastic optimal control and trajectory guidance for autonomous aerial vehicles and agile robotics operating in highly non-convex, obstacle-dense, and dynamically shifting environments (e.g., aerial delivery vehicles carrying fluid payloads with severe propellant slosh dynamics).

Traditional MPPI implementations running on general-purpose CPUs or embedded GPUs suffer from nondeterministic execution latencies, heavy thermal throttling, and massive memory bandwidth overhead when sampling thousands of Monte Carlo trajectories. H-MPPI executes **4,096 parallel trajectory rollouts** per $2.0\text{ ms}$ control step in dedicated silicon, achieving sub-millisecond dispatch latency and deterministic real-time control guarantees.

---

## Mathematical Formulation & Algorithm

MPPI formulates continuous optimal control under stochastic disturbance by minimizing the cost functional:
$$J(u) = \mathbb{E}_{\mathbb{Q}} \left[ \phi(x_T) + \int_0^T \left( q(x_t) + \frac{\lambda}{2} u_t^T R^{-1} u_t \right) dt \right]$$

The optimal control sequence $u^*(t)$ is computed via importance sampling over $K = 4,096$ perturbed rollouts:
$$u^*(t) = u_{\text{nom}}(t) + \sum_{k=1}^K w^{(k)} \cdot \epsilon^{(k)}(t)$$

Where the importance weights $w^{(k)}$ are evaluated via the Gibbs distribution:
$$w^{(k)} = \frac{\exp\left( -\frac{1}{\lambda} \tilde{S}^{(k)} \right)}{\sum_{j=1}^K \exp\left( -\frac{1}{\lambda} \tilde{S}^{(j)} \right)}, \quad \tilde{S}^{(k)} = S(\tau^{(k)}) - \min_{j} S(\tau^{(j)})$$

---

## Hardware Architecture & Submodules (`rtl/`)

```
                  +-------------------------------------------------------------+
                  |                     H-MPPI GUIDANCE CORE                    |
                  |                                                             |
  Initial State   |  +---------------------+        +-------------------------+ | Optimal Control
  x0, P0          |  |  4D Sobol QMC Core  |=======>|   MPPI Rollout Lanes    | | Output
  ===============>|  |  (Low-Discrepancy)  | eps(k) |  (Predictive Pipelining)|=====> u*(t)
                  |  +---------------------+        +-------------------------+ |
                  |                                              |              |
                  |  +---------------------+                     v              |
                  |  | Chebyshev Slosh Eval|<=======[ State Dynamic Updates ]   |
                  |  | (Liquid Damping)    |                     |              |
                  |  +---------------------+                     v              |
                  |                                 +-------------------------+ |
                  |                                 |   Fast Exp2 & Softmax   | |
                  |                                 |  (Importance Weights)   | |
                  |                                 +-------------------------+ |
                  +-------------------------------------------------------------+
```

### 1. 4D Sobol Quasi-Monte Carlo Sampler (`rtl/sobol_qmc_4d.sv`)
- Generates low-discrepancy 4-dimensional perturbation vectors $\epsilon \in \mathbb{R}^4$ across acceleration and attitude rate channels.
- Employs bitwise Gray code counter stepping ($x_{n+1} = x_n \oplus v_{c}$) with precomputed direction numbers $v_i$, providing superior sample space coverage and faster error convergence ($O((\log N)^d / N)$) compared to standard pseudo-random number generators ($O(1/\sqrt{N})$).

### 2. Chebyshev Slosh Dynamics Evaluator (`rtl/chebyshev_slosh_eval.sv`)
- Models coupled fluid slosh dynamics without computationally prohibitive numerical ODE integration.
- Evaluates orthogonal Chebyshev polynomials $T_n(x)$ via pipelined recurrence relations ($T_{n+1}(x) = 2x T_n(x) - T_{n-1}(x)$), computing liquid pendulum deflection $\theta_{\text{slosh}}$ and lateral reaction forces in 3 clock cycles.

### 3. Fast Exponential Unit (`rtl/fexp2_unit.sv`)
- Evaluates base-2 exponential functions ($2^{-x}$) for trajectory likelihood weighting.
- Implements mantissa range reduction and 5th-order minimax polynomial approximation, guaranteeing $< 1\text{ ULP}$ error across the entire cost dynamic range.

### 4. Rollout Lane Pipeline (`rtl/mppi_rollout_lane.sv`)
- Deeply pipelined state integration pipeline evaluating velocity propagation, position updates, obstacle corridor distance checks, and terminal cost accumulation.

### 5. Top-Level Core (`rtl/hmppi_core_top.sv`)
- Coordinates rollout dispatch, maximum cost tracking, softmax accumulation, and minimum-variance control vector dispatch.

---

## Physical Design Progress & Sign-off Status

### 1. ASAP7 7nm FinFET Predictive ASIC Implementation (`asap7/`)
- **Physical Layout Sign-off**: Complete routed layout stream `asap7/hmppi_core_top.gds` ($62.7\text{ MB}$).
- **Target Operating Frequency**: $500.00\text{ MHz}$ ($2.00\text{ ns}$ clock period), static timing closed across fast, slow, and typical corners.
- **Execution Latency**: Complete 16-step trajectory evaluation frame finishes in $1,218\text{ clock cycles}$ ($2.436\ \mu\text{s}$ total hardware compute latency).
- **Standard Cell Area & Utilization**:
  - Cell Count: $63,405\text{ standard cells}$
  - Standard Cell Area: $8,214.93\ \mu\text{m}^2$ ($43.79\%$ core placement density)
- **Power Dissipation**:
  - Total Active Power: $32.47\text{ mW}$ at $500\text{ MHz}$ nominal voltage (detailed breakdown in `asap7/reports/power/typical.rpt`)
- **Detailed Routing Metrics**:
  - Routed Nets: 61,348 nets cleanly routed with 0 antenna/DRC violations and 0 floating pins/nets
  - Via Count: 499,870 vias across 7nm BEOL metal stack
  - Total Wirelength: $178,513\ \mu\text{m}$
- **Synthesized Gate Netlist**: `asap7/netlist/hmppi_core_top.vg` ($8.0\text{ MB}$)

### 2. Xilinx Artix-7 FPGA Implementation (`fpga/`)
- **Target Devices**: Xilinx Artix-7 `xc7a200t-fbg676-2` & `xc7a35t-cpg236-1`
- **Clock Frequency**: $50.00\text{ MHz}$ clean timing closure
- **FPGA Resource Utilization (`fpga/reports/`)**:
  - Slice LUTs: 18,420 (14.0% of xc7a200t)
  - Slice Registers: 15,110 (5.7% of xc7a200t)
  - DSP48E1 Slices: 48 (6.5% of xc7a200t)
  - Total On-Chip Power: $214\text{ mW}$

---

## Technical Documentation & Engineering Reports

Comprehensive micro-architectural design documents, detailed RTL development narratives capturing engineering decisions, and physical implementation reports are cataloged in [`docs/`](docs/):

### Architectural Design Specifications
- [`docs/design_mppi_module_index.md`](docs/design_mppi_module_index.md) — Master H-MPPI hardware accelerator architectural index and dataflow pipelines.
- [`docs/design_sobol_qmc_4d.md`](docs/design_sobol_qmc_4d.md) — 4D Gray-code low-discrepancy Quasi-Monte Carlo perturbation generator.
- [`docs/design_mppi_attitude_kinematics.md`](docs/design_mppi_attitude_kinematics.md) — 6-DoF rigid body attitude kinematics and trajectory rollout datapath.
- [`docs/design_chebyshev_slosh_eval.md`](docs/design_chebyshev_slosh_eval.md) — Orthogonal Chebyshev polynomial fuel-slosh dynamics evaluator.
- [`docs/design_fexp2_unit.md`](docs/design_fexp2_unit.md) — Pipelined base-2 exponential evaluation unit and Softmax normalizer.
- [`docs/design_hmppi_core_top.md`](docs/design_hmppi_core_top.md) — Top-level H-MPPI guidance and control accelerator integration.

### RTL Implementation & Engineering Thought Process
- [`docs/rtl_sobol_qmc_4d.md`](docs/rtl_sobol_qmc_4d.md) — Bit-level Gray code XOR datapath, primitive polynomials, and direction vector LUT pipelining.
- [`docs/rtl_chebyshev_slosh_eval.md`](docs/rtl_chebyshev_slosh_eval.md) — Pipelined Horner evaluation scheme, fixed-point precision analysis, and DSP mapping.
- [`docs/rtl_fexp2_unit.md`](docs/rtl_fexp2_unit.md) — Range reduction arithmetic, Taylor polynomial expansion, and normalization pipeline.
- [`docs/rtl_mppi_rollout_lane.md`](docs/rtl_mppi_rollout_lane.md) — Monte Carlo parallel rollout datapath, obstacle cost accumulation, and hazard mitigation.
- [`docs/rtl_hmppi_core_top.md`](docs/rtl_hmppi_core_top.md) — Inter-lane synchronization, tournament reduction tree, and control output staging.

### Physical Implementation Sign-Off
- [`docs/physical_implementation_asap7_hmppi.md`](docs/physical_implementation_asap7_hmppi.md) — ASAP7 7nm FinFET physical design sign-off: 63,405 cells, 500 MHz timing closure, 32.47 mW typical power dissipation, and 61,348 routed nets.

---

## Repository Structure
```
H_mppi/
├── .gitignore
├── README.md
├── docs/                              # Engineering documentation suite
│   ├── design_*.md                    # Micro-architectural design specifications
│   ├── rtl_*.md                       # Comprehensive RTL implementation narratives
│   └── physical_implementation_*.md   # ASAP7 7nm physical implementation report
├── rtl/
│   ├── chebyshev_slosh_eval.sv        # Orthogonal polynomial slosh dynamics evaluator
│   ├── fexp2_unit.sv                  # Fast pipelined base-2 exponential evaluation unit
│   ├── hmppi_core_top.sv              # H-MPPI guidance top-level engine
│   ├── mppi_rollout_lane.sv           # Pipelined Monte Carlo rollout evaluation lane
│   └── sobol_qmc_4d.sv                # 4D low-discrepancy Sobol QMC sampling engine
├── sim/
│   ├── mppi_golden_model.py           # Bit-accurate mathematical reference model
│   ├── tb_hmppi_netlist.sv            # Post-synthesis gate-level simulation testbench
│   ├── tb_hmppi_top.sv                # Top-level functional verification testbench
│   └── results/
│       └── simulation_report.log      # RTL full core simulation log (1,220 cycles, 0 errors)
├── fpga/
│   ├── constraints/
│   │   └── hmppi_core_top.xdc         # Artix-7 timing & physical pinout constraints
│   ├── netlist/
│   │   ├── hmppi_core_top.json        # Structural JSON netlist
│   │   └── hmppi_core_top_series7_netlist.v # Gate-level mapped netlist
│   └── reports/
│       ├── power_analysis_artix7.rpt  # Vivado switching power analysis report
│       ├── timing_summary_50mhz.rpt   # Static timing analysis summary
│       ├── utilization_artix7_xc7a35t.rpt  # xc7a35t device utilization report
│       └── utilization_artix7_xc7a200t.rpt # xc7a200t device utilization report
└── asap7/
    ├── asap7_ppa_report.json          # 7nm FinFET Power-Performance-Area sign-off metrics
    ├── hmppi_core_top.gds             # Sign-off GDSII layout database (62.7 MB)
    ├── hmppi_core_top.sdc             # Synopsys Design Constraints (500 MHz target)
    ├── netlist/
    │   └── hmppi_core_top.vg          # Gate-level netlist in ASAP7 standard cells (8.0 MB)
    └── reports/
        ├── metrics.json               # OpenROAD run metrics
        ├── checks/                    # Floating net and pin connection reports
        ├── clocks/                    # Clock tree synthesis (CTS) reports
        ├── design/                    # Standard cell usage & logic depth reports
        ├── power/                     # Typical (32.47 mW), Fast, and Slow power reports
        └── timing/                    # Setup slack, endpoints, and multi-scenario reports
```

---

## Running Verification

### 1. Functional SystemVerilog RTL Simulation
```bash
# Compile and run H-MPPI RTL testbench with Icarus Verilog
iverilog -g2012 -o hmppi_sim.vvp rtl/*.sv sim/tb_hmppi_top.sv
vvp hmppi_sim.vvp
```

### 2. Mathematical Golden Model Comparison
```bash
# Validate hardware QMC and softmax weights against floating-point golden model
python3 sim/mppi_golden_model.py --rollouts 4096 --verify
```
