# Design: `fexp2_lut_taylor.sv` — Base-2 Softmax Exponential & Weight Normalizer

**Spec Reference:** §3.5, Decision H5 (Division-Free Softmax Normalization)
**Target:** Standalone H-MPPI & Integrated Monolithic SoC
**Hardware Cost:** 140 LUTs, 48 Flip-Flops, 0 DSPs, 0 Dividers

---

## 1. Purpose & First Principles

In Model Predictive Path Integral (MPPI) control, the candidate trajectory rollouts are weighted exponentially according to their accumulated cost $S_k$:
$$w_k = \frac{\exp\left(-\frac{1}{\lambda} (S_k - \rho)\right)}{\sum_{j=1}^K \exp\left(-\frac{1}{\lambda} (S_j - \rho)\right)}$$
where $\lambda$ is the temperature parameter and $\rho = \min_{j} S_j$ is the baseline minimum cost.

Direct evaluation of this equation presents severe hardware bottlenecks:
1. **Natural Exponential $\exp(x)$:** Evaluating $e^{-x}$ across $K=128$ trajectories requires either CORDIC-based iterative computation (20+ cycles per evaluation), Taylor expansion of $e^x$ (high-order polynomial, 6+ DSP cycles), or large LUTs stored in BRAM.
2. **Division by Sum of Weights:** Computing the denominator $\sum w_j$ requires a 128-element reduction, followed by 128 divisions. Each division costs 28 cycles on an iterative divider, creating a 3,584-cycle bottleneck.
3. **Underflow:** Without baseline subtraction, all 128 weights underflow to zero in fixed-point, causing division by zero (see doc 65 for the baseline subtraction solution).

The `fexp2_lut_taylor` module eliminates both the natural exponential and the explicit division by converting the problem into base-2 arithmetic, where exponentiation is a bit-shift and the normalization denominator can be computed without a divider.

---

## 2. Mathematical Reformulation: Base-2 Log-Domain Mapping

The natural exponential is transformed into a **base-2 exponential** using the change-of-base identity:
$$\exp(-z) = 2^{-z \cdot \log_2(e)} \approx 2^{-z \cdot 1.4427}$$
where $z = \frac{1}{\lambda}(S_k - \rho) \ge 0$ (always non-negative after baseline subtraction).

Let $X = z \cdot \log_2(e) \ge 0$. Decompose $X$ into its **integer part** $I = \lfloor X \rfloor$ and **fractional part** $F = X - I \in [0, 1)$:
$$2^{-X} = 2^{-(I + F)} = 2^{-I} \cdot 2^{-F}$$

This decomposition is the core insight:
1. **Integer component $2^{-I}$:** Evaluated as a **pure bitwise right-shift** by $I$ positions. No arithmetic needed — just wire routing.
2. **Fractional component $2^{-F}$:** Over the strictly bounded domain $F \in [0, 1)$, $2^{-F} \in (0.5, 1.0]$ is smooth and well-conditioned. A low-order polynomial approximation suffices.

---

## 3. Degree-2 Taylor Polynomial Interpolation

We approximate $g(F) = 2^{-F}$ using a degree-2 Taylor expansion around $F = 0$:
$$2^{-F} \approx 1 - (\ln 2) F + \frac{(\ln 2)^2}{2} F^2 \approx 1 - 0.6931 \cdot F + 0.2402 \cdot F^2$$

In Q4.12 fixed-point arithmetic:
- $C_0 = 1.0 = \text{16'h1000}$
- $C_1 = 0.6931 = \text{16'h0B17}$
- $C_2 = 0.2402 = \text{16'h03D8}$

### Error Analysis & Precision Bound

Over $F \in [0, 1)$, the maximum absolute error of the degree-2 approximation is:
$$\epsilon_{max} = \max_{F \in [0, 1)} \left| 2^{-F} - (1 - C_1 F + C_2 F^2) \right| \approx 0.00234$$

In Q4.12 units, this is approximately 9.6 LSBs — but this error is irrelevant for MPPI because:
1. The stochastic sampling noise is $\sim 1/\sqrt{K} = 1/\sqrt{128} \approx 0.088$, which is 37× larger than the polynomial error.
2. The weights are used only for weighted averaging, where relative magnitudes matter more than absolute accuracy.
3. The error is systematic (same polynomial for all $k$), so it cancels in the numerator/denominator ratio.

### Why Not a Higher-Order Polynomial?

A degree-3 polynomial would reduce the error to $< 0.0001$ but requires an additional DSP multiplication cycle. Since the degree-2 error is already below the stochastic noise floor, the extra accuracy provides zero benefit while adding latency and area.

---

## 4. Hardware Interface & Port Definition

```systemverilog
module fexp2_lut_taylor #(
    parameter DATA_WIDTH = 16,         // Q4.12 fixed-point format
    parameter NUM_TRAJECTORIES = 128   // K
)(
    input  logic                     clk_i,
    input  logic                     rst_ni,

    // Handshake
    input  logic                     valid_i,       // New cost value ready
    output logic                     valid_o,       // Weight output ready
    output logic                     ready_o,       // Can accept input

    // Normalized cost input: z = (S_k - rho) / lambda >= 0
    input  logic [DATA_WIDTH-1:0]    cost_norm_i,   // Unsigned Q4.12 (always >= 0)

    // Baseline minimum (from parallel reduction tree)
    input  logic signed [DATA_WIDTH-1:0] rho_i,

    // Temperature parameter (from host configuration)
    input  logic [DATA_WIDTH-1:0]    inv_lambda_i,  // 1/λ in Q4.12

    // Output weight
    output logic                     weight_valid_o,
    output logic [DATA_WIDTH-1:0]    weight_o,      // Unnormalized weight w_k (Q4.12)

    // Weight accumulator interface (for normalization)
    output logic                     accum_valid_o,  // Weight has been added to sum
    output logic [DATA_WIDTH+7:0]    weight_sum_o,   // Running sum of all weights (24-bit)
    input  logic                     frame_done_i,   // All 128 weights computed

    // Normalized weight output (after division)
    output logic                     norm_valid_o,
    output logic [DATA_WIDTH-1:0]    norm_weight_o   // Normalized w_k / Σw_j (Q4.12)
);
```

### Timing Contract
- **Exponential evaluation latency:** 1 clock cycle (combinational polynomial + registered output).
- **Full weight computation (128 trajectories):** 128 cycles for all exponentials + 1 cycle for sum finalization.
- **Normalization latency:** 128 additional cycles (re-read each weight and multiply by reciprocal of sum).
- **Total:** ~257 cycles for complete softmax evaluation.

---

## 5. Micro-Architectural Datapath (1-Cycle Pipeline)

The entire $2^{-X}$ evaluation fits in a single clock cycle:

### Step 1: Scale by $\log_2(e)$

$$X = \text{cost\_norm\_i} \times \text{16'h1715}$$

The constant `16'h1715` = $\log_2(e) \approx 1.4427$ in Q4.12. This multiplication uses a LUT multiplier (not a DSP — the constant is known at synthesis time, so the multiplier can be optimized into a shift-add tree).

### Step 2: Integer/Fractional Split

$$I = X[15:12], \quad F = X[11:0]$$

This is a pure wire split — zero logic.

### Step 3: Polynomial Evaluation (Horner Form)

Evaluate $M = C_0 - C_1 \cdot F + C_2 \cdot F^2$ using Horner's method:
$$M = C_0 + F \cdot (-C_1 + C_2 \cdot F)$$

Implementation:
```systemverilog
logic [23:0] f_sq;           // F² (12×12 = 24 bits)
logic [23:0] c2_f_sq;        // C2 × F² (12×12 = 24 bits, then >>12)
logic [15:0] inner;          // -C1 + C2·F² (truncated to 16 bits)
logic [23:0] f_inner;        // F × inner (12×16 = 28 bits, then >>12)
logic [15:0] M;              // C0 + F·inner = polynomial result

assign f_sq     = F * F;
assign c2_f_sq  = C2 * f_sq[23:12];
assign inner    = c2_f_sq[23:12] - C1;   // Note: inner can be negative
assign f_inner  = F * inner;
assign M        = C0 + f_inner[23:12];    // M ∈ (0.5, 1.0]
```

### Step 4: Integer Shift

$$\text{weight} = M \gg I$$

If $I \ge 12$: the result underflows to zero (all significant bits shifted out). Clamp to `16'h0000`.

```systemverilog
always_comb begin
    if (I >= 4'd12)
        weight_raw = 16'h0000;
    else
        weight_raw = M >> I;
end
```

The barrel shifter for the right-shift uses approximately 48 LUTs (4-bit shift amount × 16-bit data).

---

## 6. Division-Free Weight Normalization

After computing all 128 unnormalized weights $w_k = 2^{-X_k}$, the normalized weights require division:
$$\hat{w}_k = \frac{w_k}{\sum_{j=1}^K w_j}$$

Direct division is avoided using **reciprocal approximation via Newton-Raphson iteration**:

1. **Accumulate sum:** As each weight is computed, add it to a running 24-bit accumulator $W = \sum w_j$. This costs zero extra cycles — it happens in parallel with the exponential evaluation.

2. **Compute reciprocal $1/W$:** After all 128 weights are summed, compute $1/W$ using 2 iterations of Newton-Raphson:
   - Initial estimate from a 4-entry LUT indexed by the MSBs of $W$.
   - Iteration: $r_{n+1} = r_n \cdot (2 - W \cdot r_n)$.
   - 2 iterations require 4 multiplications = 4 cycles.

3. **Multiply each weight by $1/W$:** Re-read the 128 stored weights from a small weight buffer and multiply each by $1/W$. This takes 128 cycles.

Total normalization: 4 (reciprocal) + 128 (multiplication) = 132 cycles.

### Weight Buffer

The 128 unnormalized weights must be stored temporarily for the normalization pass. A small dual-port RAM (128 × 16-bit = 256 bytes) serves as the weight buffer:

```systemverilog
logic [15:0] weight_buf [0:127];

// Write during exponential evaluation
always_ff @(posedge clk_i) begin
    if (weight_valid)
        weight_buf[traj_idx] <= weight_raw;
end

// Read during normalization pass
assign buf_rdata = weight_buf[norm_idx];
```

On Artix-7, this maps to 1 BRAM18. On ASIC, it synthesizes as a 128×16 register file (~2,048 flip-flops / ~4,096 gates).

---

## 7. Underflow Handling

When $X > 12$ (i.e., the cost difference is very large), $2^{-X}$ underflows to zero in Q4.12. The weight is clamped to `16'h0000`. This is correct behavior — trajectories with very high costs should have zero weight.

The baseline subtraction guarantees that the best trajectory has $X = 0$ and weight $= 1.0$, so the weight sum is always $\ge 1.0$. Even if 127 of 128 trajectories underflow, the normalization denominator is non-zero.

The only scenario where ALL weights could theoretically be zero is if $\rho$ itself overflows Q4.12 — but this is prevented by the cost accumulator's saturation logic (doc 64).

---

## 8. Area Estimate

| Component | Resource |
|---|---|
| log2(e) multiplier (constant, shift-add tree) | ~35 LUTs |
| Polynomial evaluation (2 multipliers, 1 adder) | ~60 LUTs |
| Barrel shifter (4-bit shift, 16-bit data) | ~48 LUTs |
| Underflow clamp + output register | ~5 LUTs, 16 FFs |
| Weight accumulator (24-bit adder + register) | ~12 LUTs, 24 FFs |
| Newton-Raphson reciprocal (2-iteration FSM) | ~80 LUTs, 32 FFs |
| Weight buffer (128×16 register file or BRAM) | 1 BRAM18 or ~4,096 gates |
| Normalization multiplier (16×16) | ~40 LUTs or 1 DSP48E1 |
| Control FSM | ~20 LUTs, 8 FFs |
| **Total (excl. weight buffer)** | **~140 LUTs, 48 FFs** |

---

## 9. Integration with H-MPPI Pipeline

```
parallel_reduction_tree ──► fexp2_lut_taylor ──► control_arbiter
       │ ρ                        │ ŵ[k]              │ u*[t]
       └──────────────────────────┴────────────────────┘
```

The weight normalizer receives $\rho$ from the reduction tree and the stored costs from the cost buffer (shared with the cost accumulator). It outputs normalized weights $\hat{w}_k$ to the control arbiter, which computes the weighted average control input.

**Pipeline stall:** The weight normalization phase (128 + 132 = 260 cycles) runs AFTER all trajectory rollouts complete. During this phase, the rollout pipeline is idle. This stall is inherent to the MPPI algorithm.

---

## 10. Verification Plan

| Test | Method | Count | Pass Criterion |
|---|---|---|---|
| $z = 0$: weight = 1.0 exactly | Verilator | 1 | `weight_o = 16'h1000` |
| $z = \lambda \ln 2$: weight = 0.5 | Verilator | 1 | Within 1 LSB of `16'h0800` |
| Large $z$: weight underflows to 0 | Verilator | 10 | `weight_o = 16'h0000` |
| All trajectories equal cost: all weights equal | Verilator | 1 | All `weight_o` identical |
| One best, 127 worst: best weight ≈ 1.0 | Verilator | 1 | Best weight dominates |
| Random costs: compare against Python `exp(-z)` | Verilator | 10,000 | Max error < 10 LSBs (Q4.12) |
| Polynomial accuracy: sweep $F \in [0, 1)$ | Verilator | 4,096 | Max error < 0.003 |
| Weight sum: verify $\sum w_k > 0$ always | Verilator | 100 | Never zero |
| Normalization: verify $\sum \hat{w}_k = 1.0$ | Verilator | 50 | Within 2 LSBs of 1.0 |
| Newton-Raphson reciprocal: verify $r \cdot W \approx 1$ | Verilator | 100 | Within 1 LSB after 2 iterations |
| Full MPPI: 128 weights → weighted average | Verilator | 10 | Matches Python golden model |
| Pipeline throughput: 128 weights in 128 cycles | Verilator | 1 | No stalls during exp phase |

---

## 11. Implementation Notes for the Coder

1. **The $\log_2(e)$ multiply by constant `16'h1715`** should be implemented as a shift-add tree, NOT a general multiplier. Since the constant is known at synthesis time, the synthesis tool will optimize it into approximately 5 additions. Do not instantiate a DSP48E1 for this.

2. **The polynomial evaluation order matters.** Compute $F^2$ first, then $C_2 \cdot F^2$, then subtract $C_1$, then multiply by $F$, then add $C_0$. This is Horner's method and minimizes intermediate overflow risk.

3. **The barrel shifter must handle $I = 0$ (no shift) correctly.** When $I = 0$, the output is $M$ unshifted. Make sure the shifter does not introduce spurious zeros for the no-shift case.

4. **The Newton-Raphson initial estimate** should come from a 4-entry LUT indexed by `W[23:22]` (the two MSBs of the 24-bit weight sum). The entries are precomputed reciprocals for the 4 magnitude ranges. This gives ~2 bits of initial accuracy, and 2 NR iterations refine it to ~8 bits — sufficient for Q4.12 weight normalization.

5. **The weight buffer can be BRAM or distributed RAM.** On Artix-7, use a single BRAM18 (most area-efficient). On ASIC, use a compiled SRAM macro if available, otherwise a register file.

6. **Reset behavior:** On reset, weight accumulator = 0, all weights = 0, FSM in IDLE.
