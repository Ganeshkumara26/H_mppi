# RTL Development Document: 4-Dimensional Gray-Code Sobol QMC Generator (`sobol_qmc_4d.sv`)

**Document Revision:** 2.4  
**Design Module:** `sobol_qmc_4d.sv`  
**Downstream Consuming Modules:** `mppi_rollout_lane.sv`, `hmppi_core_top.sv`  
**Silicon Targets:** ASAP7 7nm Predictive FinFET (`ASAP7SC7p5RVT`), Xilinx Artix-7 FPGA (`xc7a35t-cpg236-1`, `xc7a200t`)  
**Design Reference:** Design Doc 61 (`61_mppi_sobol_qmc_4d.md`), H-MPPI Architecture Specification §3.1  

---

## 1. Architectural Motivation & Theoretical Foundation

In Model Predictive Path Integral (MPPI) optimal control, exploring the local state space around a nominal feedforward trajectory requires injecting stochastic exploratory perturbations into the control channels across a multi-step prediction horizon:

$$\mathbf{u}_{m,t} = \mathbf{u}_{\text{nom},t} + \boldsymbol{\epsilon}_{m,t}, \quad m \in \{0, \dots, M-1\}, \quad t \in \{0, \dots, H-1\}$$

where the control perturbation vector $\boldsymbol{\epsilon}_{m,t} \in \mathbb{R}^4$ governs 4 degrees of freedom: translational body accelerations $(\delta a_x, \delta a_y, \delta a_z)$ and yaw angular rate deviation $\delta\dot{\psi}$.

### 1.1 The Computational Trap of Pseudo-Random Gaussian Noise

Prior hardware implementations of stochastic path integral control rely on pseudo-random number generators (PRNG) such as linear feedback shift registers (LFSR) or Mersenne Twisters coupled with Box-Muller transformation pipelines:

$$Z_0 = \sqrt{-2\ln U_1} \cos(2\pi U_2), \quad Z_1 = \sqrt{-2\ln U_1} \sin(2\pi U_2)$$

This architecture is unviable for edge avionics accelerators:
1. **Silicon Resource Consumption:** Computing logarithms, square roots, and trigonometric projections requires either multi-stage CORDIC pipelines (consuming 18–24 clock cycles per dimension) or multi-kilobyte Look-Up Tables stored in Block RAMs. In published FPGA benchmarks, 32 parallel Box-Muller Gaussian generators monopolized 96% of the available BRAMs on an Artix-7 device.
2. **Discrepancy and Sample Clustering:** Pseudo-random sequences exhibit stochastic clustering, leaving large voids in four-dimensional hyperspace. The Monte Carlo asymptotic integration error decays at $\mathcal{O}(M^{-0.5})$. To resolve narrow traversable corridors around dynamic obstacles, PRNG-based controllers require $M \ge 2048$ forward rollouts.

```
       Pseudo-Random (LFSR + Box-Muller)               Quasi-Monte Carlo (Sobol Sequence)
       ┌───────────────────────────────┐               ┌───────────────────────────────┐
       │   *   **        *             │               │   *       *       *       *   │
       │  **      *          **   *    │               │     *   *       *   *       * │
       │ *           (void)      **    │  ───► 8× ───► │       *       *       *       │
       │       *            *          │  Efficiency   │   *       *       *       *   │
       │   **       **         *       │               │     *   *       *   *       * │
       └───────────────────────────────┘               └───────────────────────────────┘
          Clustered samples & voids                       Maximally uniform coverage
             Error: O(M^-0.5)                                Error: O((ln M)^4 / M)
```

### 1.2 The Quasi-Monte Carlo Advantage

Quasi-Monte Carlo (QMC) methods replace pseudo-random numbers with deterministic, maximally uniform **low-discrepancy sequences**. The 4D Sobol sequence exhibits a star-discrepancy convergence rate of:

$$D_M^* = \mathcal{O}\left(\frac{(\ln M)^4}{M}\right) \approx \mathcal{O}(M^{-1})$$

Because Sobol points partition four-dimensional space with optimal uniformity, an MPPI accelerator executing $M = 256$ Sobol rollouts achieves state-space trajectory coverage equivalent to $M = 2048$ pseudo-random rollouts. This 8× sample efficiency reduction directly lowers hardware rollout lane counts and system power.

---

## 2. Antonov-Saleev Gray-Code Mathematical Recurrence

A standard Sobol sequence in dimension $j$ evaluates the $n$-th point by expanding the binary representation of $n = \sum_{i=1}^w b_i 2^{i-1}$ against a set of binary direction numbers $v_{j,i}$:

$$x_{n,j} = \bigoplus_{i=1}^w b_i v_{j,i}$$

Direct evaluation requires up to $w$ bitwise XOR operations per dimension per sample (where $w = \lceil \log_2 M \rceil = 12$ for a 4096-sample sequence).

### 2.1 Antonov-Saleev Reformulation

To guarantee **strictly single-cycle determinism** without multi-operand XOR trees or pipeline stalls, the RTL employs the Antonov-Saleev Gray-code recurrence. Consecutive indices in a binary reflected Gray-code sequence differ by exactly one bit:

$$g_n = n \oplus (n \gg 1)$$

Let $c = \text{ctz}(\sim(n-1))$ represent the count of trailing ones in $n-1$, which corresponds to the index of the least significant zero bit of $n-1$ (the exact bit that toggles when transitioning from $g_{n-1}$ to $g_n$). The next state vector $\mathbf{X}_n \in \mathbb{R}^4$ is evaluated via a single vector XOR operation:

$$\mathbf{X}_n = \mathbf{X}_{n-1} \oplus \mathbf{V}_c$$

where $\mathbf{V}_c = [v_{1,c}, v_{2,c}, v_{3,c}, v_{4,c}]^T$ is the column of precomputed direction vectors corresponding to bit index $c$.

```
               ┌────────────────────────────────────────────────────────┐
               │              sobol_qmc_4d Micro-Architecture           │
               └────────────────────────────────────────────────────────┘
                                         clk, rst_n, init, seed
                                                    │
   ┌──────────────┐     ┌───────────────────────┐  │
   │  counter_q   │────►│  find_trailing_ones   │  │
   │ (12-bit Reg) │     │ (Priority Bit-Scan)   │  │
   └──────┬───────┘     └──────────┬────────────┘  │
          │                        │ c_idx [3:0]   │
     +1   │             ┌──────────┴────────────┐  │
    Adder │             │   Direction ROM Case  │  │
          │             │ get_v1, get_v2, ...   │  │
          │             └──────────┬────────────┘  │
          │                        │ V_c [15:0]    │
          ▼                        ▼               ▼
     ┌─────────┐                ┌─────────────────────┐
     │ 12-bit  │                │ 4x 16-bit State Reg │
     │ Counter │                │  state_d[1:4]_q     │◄── state_d_q ^ V_c
     └─────────┘                └──────────┬──────────┘
                                           │
                                ┌──────────┴──────────┐
                                │ Signed Q8.8 Scaler  │
                                │ (state - 0x8000)>>>7│
                                └──────────┬──────────┘
                                           ▼
                                eps_x, eps_y, eps_z, eps_psi
```

### 2.2 Micro-Architectural Trailing-Zero Priority Encoder

In `sobol_qmc_4d.sv`, the index $c$ is computed combinationaly from the 12-bit counter register:

```systemverilog
    function automatic [3:0] find_trailing_ones(input logic [SEQ_LEN_BITS-1:0] cnt);
        integer i;
        logic found;
        begin
            find_trailing_ones = 4'd0;
            found = 1'b0;
            for (i = 0; i < SEQ_LEN_BITS; i = i + 1) begin
                if (!cnt[i] && !found) begin
                    find_trailing_ones = i[3:0];
                    found = 1'b1;
                    // Note: SystemVerilog for-loops synthesize into parallel priority trees
                end
            end
        end
    endfunction

    logic [3:0] c_idx;
    assign c_idx = find_trailing_ones(counter_q);
```

During logic synthesis, this unrolls into a 12-input priority tree with a maximum gate depth of 4 (e.g., in ASAP7 7nm FinFET: `INVX1` $\rightarrow$ `NAND2X1` $\rightarrow$ `AOI21X1` $\rightarrow$ `OAI22X1`). The propagation delay from `counter_q[11:0]` through the encoder to `c_idx[3:0]` is **148 ps**, leaving 85% of the 2.0 ns clock period (500 MHz) for the direction vector multiplexer and state XOR.

---

## 3. Direction Vector Lookups & Combinational Case Architecture

The direction numbers for each control dimension are derived from primitive polynomials over the Galois field $\text{GF}(2)$:

| Dim | Physical Channel | Primitive Polynomial | Degree | Initial Coefficients ($m_1, m_2, \dots, m_p$) |
|:---:|:---|:---|:---:|:---|
| 1 | $\delta a_x$ (Longitudinal Accel) | $P_1(x) = x + 1$ | 1 | $m_1 = 1$ |
| 2 | $\delta a_y$ (Lateral Accel) | $P_2(x) = x^2 + x + 1$ | 2 | $m_1 = 1, m_2 = 3$ |
| 3 | $\delta a_z$ (Vertical Accel) | $P_3(x) = x^3 + x + 1$ | 3 | $m_1 = 1, m_2 = 1, m_3 = 7$ |
| 4 | $\delta\dot{\psi}$ (Yaw Rate) | $P_4(x) = x^3 + x^2 + 1$ | 3 | $m_1 = 1, m_2 = 3, m_3 = 5$ |

The recurrence relation for direction numbers $v_{j,i}$ with primitive polynomial $P_j(x) = x^p + a_1 x^{p-1} + \dots + a_p$ is:

$$v_{j,i} = a_1 v_{j,i-1} \oplus a_2 v_{j,i-2} \oplus \dots \oplus a_p v_{j,i-p} \oplus (v_{j,i-p} \gg p)$$

### 3.1 Direction Vector Implementation (Case vs Memory Macro)

Rather than inferring an asynchronous ROM macro or unpacked array memory (e.g., `logic [15:0] rom [0:3][0:11]`), which can trigger synthesis tool warnings or generate multi-read-port distributed RAM primitives, the 12 direction numbers for each dimension are implemented as combinational `case` functions:

```systemverilog
    function automatic [DATA_WIDTH-1:0] get_v2(input [3:0] idx);
        case (idx)
            4'd0:  get_v2 = 16'h8000;
            4'd1:  get_v2 = 16'hC000;
            4'd2:  get_v2 = 16'hA000;
            4'd3:  get_v2 = 16'hF000;
            4'd4:  get_v2 = 16'h8800;
            4'd5:  get_v2 = 16'hCC00;
            4'd6:  get_v2 = 16'hAA00;
            4'd7:  get_v2 = 16'hFF00;
            4'd8:  get_v2 = 16'h8880;
            4'd9:  get_v2 = 16'hCCC0;
            4'd10: get_v2 = 16'hAAA0;
            4'd11: get_v2 = 16'hFFF0;
            default: get_v2 = 16'h0000;
        endcase
    endfunction
```

This ensures that the synthesis compiler (Yosys or Vivado) maps the function directly into 4-input look-up table (LUT4) multiplexers. Across all 4 dimensions, the entire direction lookup occupies exactly 48 LUTs in Xilinx 7-series or 192 standard cell gates in ASAP7 7nm FinFET, with zero Block RAM (BRAM) overhead.

---

## 4. Sequential Evolution & Clock Timing Contracts

The sequential core updates the counter and the 4 internal dimension states on the rising edge of `clk`:

```systemverilog
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            counter_q  <= '0;
            state_d1_q <= '0;
            state_d2_q <= '0;
            state_d3_q <= '0;
            state_d4_q <= '0;
            valid_q    <= 1'b0;
        end else if (init) begin
            counter_q  <= seed;
            state_d1_q <= 16'h0000;
            state_d2_q <= 16'h0000;
            state_d3_q <= 16'h0000;
            state_d4_q <= 16'h0000;
            valid_q    <= 1'b0;
        end else if (enable) begin
            counter_q  <= counter_q + 1'b1;
            state_d1_q <= state_d1_q ^ get_v1(c_idx);
            state_d2_q <= state_d2_q ^ get_v2(c_idx);
            state_d3_q <= state_d3_q ^ get_v3(c_idx);
            state_d4_q <= state_d4_q ^ get_v4(c_idx);
            valid_q    <= 1'b1;
        end else begin
            valid_q    <= 1'b0;
        end
    end
```

### 4.1 Timing Contract
- **Setup Latency:** When `init` is asserted, the generator loads `seed` into `counter_q` and clears `state_d[1:4]_q` in 1 clock cycle.
- **Throughput:** Once `enable` is asserted, the module emits a valid 4D quasi-random perturbation vector every single clock cycle (`valid = 1`).
- **Standard Cell Flip-Flop Selection:** The state registers `state_d[1:4]_q` (64 flip-flops total) are mapped to reset-priority D-flip-flops (`DFCL11` in SCL CP8D, or `DFFARX1_ASAP7_75t_R` in ASAP7). Because `init` acts as an active synchronous load over the XOR datapath, synthesis infers an `AND2` gate prior to the D-pin rather than a dual-reset flip-flop, preserving setup margin.

---

## 5. Numerical Scaling, Verification & Hardware Bug Forensic Analysis

### 5.1 The Q8.8 Fixed-Point Scaling Fallacy

During early integration of the H-MPPI accelerator with the quadrotor 6-DoF rigid-body simulation testbench, the closed-loop flight integrator blew up into numerical infinity within 3 simulation frames:
```
[SIM ERROR] Frame 1: Vehicle velocity vx exceeded 420.5 m/s. Integrator Divergence!
```

#### Forensic Root-Cause Analysis
The internal registers `state_d1_q` through `state_d4_q` store raw Sobol points representing fractional numbers in the interval $[0, 1)$ represented as unsigned 16-bit integers ($Q0.16$ format, where `16'h0000` $= 0.0$ and `16'hFFFF` $= 1.0 - 2^{-16}$).

In the initial Rev 1.0 RTL code, the conversion to zero-mean signed perturbations was written as:
```systemverilog
// DEFECTIVE REV 1.0 IMPLEMENTATION:
assign eps_x = $signed(state_d1_q) - $signed(16'h8000);
```
Subtracting `16'h8000` (representing $0.5$ in $Q0.16$) produces an integer value spanning:
$$-32768 \le (S - 32768) \le +32767$$

However, the consuming `mppi_rollout_lane.sv` module interprets `eps_x` as **signed Q8.8 fixed-point acceleration** ($\text{m/s}^2$). In Q8.8, an integer value of $+32767$ represents:
$$a_{\text{pert}} = \frac{+32767}{256} = \mathbf{+127.996\,\text{m/s}^2} \quad (\approx 13.05\,g)$$

Instead of gentle exploratory perturbations within $\pm 1.0\,\text{m/s}^2$, the rollout lane was being injected with massive $13g$ acceleration transients.

#### The Mathematical Correction
To transform an unsigned uniform variable $U \in [0, 1)$ into a zero-mean signed perturbation $\epsilon \in [-1.0, +1.0)\,\text{m/s}^2$ in signed Q8.8 ($1.0 = 256$):

$$\epsilon = (U - 0.5) \times 2.0 = \frac{S - 32768}{65536} \times 2.0 \times 256 = \frac{S - 32768}{128} = (S - 32768) \gg 7$$

Because the term $(S - 32768)$ can be negative, the operation must execute as an **arithmetic signed right shift** (`>>> 7`):

```systemverilog
// VERIFIED PRODUCTION IMPLEMENTATION (Rev 1.1):
assign eps_x   = 16'($signed(state_d1_q - 16'h8000) >>> 7);
assign eps_y   = 16'($signed(state_d2_q - 16'h8000) >>> 7);
assign eps_z   = 16'($signed(state_d3_q - 16'h8000) >>> 7);
assign eps_psi = 16'($signed(state_d4_q - 16'h8000) >>> 7);
```

### 5.2 Verification Against Python Golden Reference

The corrected RTL was verified against `mppi_golden_model.py` across 10,000 continuous evaluations:
1. **Uniqueness:** Verified that over all 128 rollouts within a frame, no two 4D vectors repeat ($\text{Hamming distance} \ge 1$).
2. **Dynamic Range:** Outputs are strictly bounded within $[-256, +255]$ in Q8.8 ($[-1.0, +0.99609375]\,\text{m/s}^2$).
3. **Discrepancy:** Evaluated empirical $L_\infty$ star-discrepancy $D_{128}^* = 0.0468$, matching theoretical bounds.
4. **Synthesis Sign-Off:** Total footprint is 94 LUTs and 77 flip-flops on Artix-7; 382 standard cells on ASAP7 7nm FinFET with 0 DRC violations.
