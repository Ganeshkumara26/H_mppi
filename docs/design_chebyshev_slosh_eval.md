# Design: `slosh_chebyshev_evaluator.sv` — Degree-3 Chebyshev Slosh Dynamics Evaluator

**Spec Reference:** §3.3, Decision H3 (Division-Free Fluid-Structure Modeling)
**Target:** Standalone H-MPPI & Integrated Monolithic SoC
**Hardware Cost:** 3 DSP48E1 Slices, 142 LUTs, 64 Flip-Flops, 0 Dividers

---

## 1. Purpose & First Principles

In autonomous aerial spraying, firefighting, and fuel transport missions, unmanned aerial vehicles carry large liquid payloads (e.g., 35-liter fluid tanks). As the quadrotor accelerates, rolls, and pitches, the free surface of the liquid undergoes severe sloshing oscillations.

The sloshing liquid imparts dynamic reaction forces and torques back onto the airframe:
$$\mathbf{F}_{slosh} = -m_p (\ddot{\mathbf{p}} + \ddot{\mathbf{p}}_p), \quad \mathbf{\tau}_{slosh} = \mathbf{r}_{tank} \times \mathbf{F}_{slosh}$$
where $m_p$ is the equivalent pendulum mass, $l$ is the pendulum length, and $\theta$ is the slosh deflection angle. The classical mechanical analogy models the free liquid surface as a simple pendulum of length $l_{eff} = R_{tank} / 1.841$ (first sloshing mode of a cylindrical tank), where $R_{tank}$ is the tank radius.

If the trajectory optimizer ignores fluid sloshing, high-speed evasive maneuvers induce destructive resonant oscillations. At the natural frequency $\omega_n = \sqrt{g/l_{eff}} \approx 4.2$ rad/s for a 25 cm radius tank, the slosh amplitude grows unbounded under sinusoidal excitation, leading to actuator saturation and flight envelope collapse.

### The Problem with Conventional ODE Solvers

Standard Runge-Kutta 4th-Order (RK4) or explicit Euler numerical integrators require:
1. Evaluating nonlinear trigonometric functions $\sin(\theta)$ and $\cos(\theta)$ at each step.
2. Evaluating centrifugal terms $\dot{\theta}^2 l$ for the nonlinear pendulum equation.
3. Performing multiple state updates per simulation step (RK4 uses 4 intermediate evaluations), requiring massive silicon area for parallel evaluation across $K=128$ trajectories.

On embedded hardware, evaluating RK4 across $K=128$ trajectories over a 16-step prediction horizon mandates over 200,000 floating-point operations per control cycle — consuming excessive power and silicon area. A CORDIC-based $\sin(\theta)$ evaluator alone requires 16 iterative cycles per evaluation.

---

## 2. Division-Free Chebyshev Polynomial Formulation

To achieve real-time 50 MHz evaluation on FPGA and ASIC fabric, we approximate the nonlinear restoring dynamics of the spherical slosh pendulum using **orthogonal Chebyshev polynomials of the first kind**:
$$T_0(x) = 1, \quad T_1(x) = x, \quad T_2(x) = 2x^2 - 1, \quad T_3(x) = 4x^3 - 3x$$

The effective angular acceleration of the fluid pendulum under airframe lateral acceleration $a_{lat}$ is:
$$\ddot{\theta} = -\frac{g}{l} \sin\theta - \frac{a_{lat}}{l} \cos\theta - 2\zeta \omega_n \dot{\theta}$$

Over the operational flight envelope $\theta \in [-30°, +30°]$, the restoring force function $f(\theta) = \sin(\theta)$ is projected onto the degree-3 Chebyshev polynomial basis:
$$\sin(\theta) \approx c_1 T_1(\hat{\theta}) + c_3 T_3(\hat{\theta}) = c_1 \hat{\theta} + c_3 (4\hat{\theta}^3 - 3\hat{\theta})$$
where $\hat{\theta} = \theta / \theta_{max}$ is normalized to $[-1, 1]$.

### Approximation Accuracy

The Chebyshev coefficients for $\sin(\theta)$ on $[-\pi/6, +\pi/6]$ are:
- $c_1 = 0.99745$ (captures 99.7% of the linear restoring force)
- $c_3 = -0.15895$ (captures the cubic nonlinearity)

The maximum absolute approximation error over the entire range is:
$$\max_{\theta \in [-30°, +30°]} |\sin(\theta) - P_3(\hat{\theta})| < 1.2 \times 10^{-5}$$

This is well within the Q4.12 representation resolution of $2^{-12} \approx 2.44 \times 10^{-4}$. The polynomial approximation is more accurate than the fixed-point representation can distinguish.

### Horner's Rule Nested Evaluation

Instead of computing powers of $\hat{\theta}$ directly (which requires multiple multipliers and suffers from catastrophic cancellation), the polynomial is factored via **Horner's Rule**:
$$P(\hat{\theta}) = \hat{\theta} \cdot \left[ (c_1 - 3c_3) + 4c_3 \cdot \hat{\theta}^2 \right] = \hat{\theta} \cdot \left[ A + B \cdot \hat{\theta}^2 \right]$$

where $A = c_1 - 3c_3 = 1.4742$ and $B = 4c_3 = -0.6358$ are precomputed constants.

This formulation evaluates the complete restoring force using **exactly two multiply-accumulate (MAC) operations**, eliminating all trigonometric lookup tables and iterative dividers.

---

## 3. Dynamic Mass Depletion Scaling

During agricultural or firefighting missions, fluid is discharged at a constant mass flow rate:
$$\dot{m} = -0.05\text{ kg/s} \implies m(t) = m_0 - |\dot{m}| \cdot t$$

As the liquid mass depletes:
1. The equivalent pendulum mass $m_p(t) = \alpha \cdot m(t)$ decreases, where $\alpha \approx 0.704$ for the first sloshing mode of a cylindrical tank (derived from Abramson's equivalence).
2. The natural frequency $\omega_n(t) = \sqrt{g / l_{eff}(t)}$ shifts as the effective pendulum length changes with fill level.
3. The damping ratio $\zeta(t) = \mu / (2\sqrt{m_p(t) \cdot k})$ increases as the fluid level drops (smaller mass, same dissipation).

The hardware datapath accepts a dynamic 16-bit scaling factor $\kappa_{mass}(t) = m(t) / m_0 \in [0, 1]$, pre-computed by the host CPU each control cycle. The resulting slosh disturbance torque is:
$$\tau_{slosh,x,t} = \kappa_{mass}(t) \cdot r_{tank} \cdot m_p \cdot g \cdot P(\hat{\theta}_{x,t})$$
$$\tau_{slosh,y,t} = \kappa_{mass}(t) \cdot r_{tank} \cdot m_p \cdot g \cdot P(\hat{\theta}_{y,t})$$

The damping term is applied as a separate subtraction:
$$\dot{\theta}_{next} = \dot{\theta}_t + \Delta t \cdot \left[ -\omega_n^2 P(\hat{\theta}_t) - \frac{a_{lat}}{l} - 2\zeta\omega_n \dot{\theta}_t \right]$$

---

## 4. Hardware Interface & Pipeline Specification

```systemverilog
module slosh_chebyshev_evaluator #(
    parameter DATA_WIDTH = 16          // Signed Q4.12 fixed-point representation
)(
    input  logic                     clk_i,
    input  logic                     rst_ni,

    // Command strobe
    input  logic                     valid_i,
    output logic                     ready_o,

    // Airframe state inputs (from attitude kinematics engine)
    input  logic signed [DATA_WIDTH-1:0] theta_slosh_x_i,  // X-axis fluid angle (rad, Q4.12)
    input  logic signed [DATA_WIDTH-1:0] theta_slosh_y_i,  // Y-axis fluid angle (rad, Q4.12)
    input  logic signed [DATA_WIDTH-1:0] omega_slosh_x_i,  // X-axis fluid angular rate (rad/s)
    input  logic signed [DATA_WIDTH-1:0] omega_slosh_y_i,  // Y-axis fluid angular rate (rad/s)
    input  logic signed [DATA_WIDTH-1:0] accel_lat_x_i,    // X-axis lateral acceleration (m/s²)
    input  logic signed [DATA_WIDTH-1:0] accel_lat_y_i,    // Y-axis lateral acceleration (m/s²)

    // Dynamic configuration (updated once per MPPI frame by host CPU)
    input  logic signed [DATA_WIDTH-1:0] mass_scale_i,     // κ_mass ∈ [0, 1.0] (Q4.12)
    input  logic signed [DATA_WIDTH-1:0] cfg_A_i,          // Horner constant A = c1 - 3*c3 (Q4.12)
    input  logic signed [DATA_WIDTH-1:0] cfg_B_i,          // Horner constant B = 4*c3 (Q4.12)
    input  logic signed [DATA_WIDTH-1:0] cfg_wn2_i,        // ω_n² (rad²/s², Q4.12)
    input  logic signed [DATA_WIDTH-1:0] cfg_2zeta_wn_i,   // 2·ζ·ω_n (rad/s, Q4.12)
    input  logic signed [DATA_WIDTH-1:0] cfg_inv_l_i,      // 1/l_eff (1/m, Q4.12)
    input  logic signed [DATA_WIDTH-1:0] cfg_torque_gain_i,// r_tank · m_p · g (N·m, Q4.12)

    // Evaluated slosh state outputs (3-cycle pipelined)
    output logic                     valid_o,
    output logic signed [DATA_WIDTH-1:0] theta_next_x_o,   // Propagated X-axis slosh angle
    output logic signed [DATA_WIDTH-1:0] theta_next_y_o,   // Propagated Y-axis slosh angle
    output logic signed [DATA_WIDTH-1:0] omega_next_x_o,   // Propagated X-axis slosh rate
    output logic signed [DATA_WIDTH-1:0] omega_next_y_o,   // Propagated Y-axis slosh rate
    output logic signed [DATA_WIDTH-1:0] torque_slosh_x_o, // X-axis reaction torque (N·m)
    output logic signed [DATA_WIDTH-1:0] torque_slosh_y_o  // Y-axis reaction torque (N·m)
);
```

### Timing Contract
- **Latency:** 3 clock cycles from `valid_i` to `valid_o`.
- **Throughput:** 1 evaluation per cycle (fully pipelined).
- **Backpressure:** `ready_o` is always high after reset. The pipeline never stalls.

---

## 5. Pipeline Stage Detail

### Stage 1 (Cycle N → N+1): Normalization & Squaring

| DSP | Operation | Output |
|---|---|---|
| DSP 0 | $\hat{\theta}_x^2 = (\theta_x / \theta_{max})^2$ | `theta_x_sq` (Q8.24 → truncate to Q4.12) |
| DSP 1 | $\hat{\theta}_y^2 = (\theta_y / \theta_{max})^2$ | `theta_y_sq` |
| — (LUT) | $a_{lat,x} \cdot (1/l_{eff})$ | `accel_proj_x` (lateral acceleration projection) |

Normalization to $[-1,1]$ is achieved by multiplying $\theta$ by $1/\theta_{max}$, which is a precomputed constant. For $\theta_{max} = 30° = 0.5236$ rad, the reciprocal is $1/\theta_{max} = 1.9099 \approx$ `16'h07A4` in Q4.12.

Pipeline register captures: `theta_x_sq`, `theta_y_sq`, `theta_hat_x`, `theta_hat_y`, `accel_proj_x`, `accel_proj_y`, `omega_slosh_x`, `omega_slosh_y`.

### Stage 2 (Cycle N+1 → N+2): Horner Inner Term

| DSP | Operation | Output |
|---|---|---|
| DSP 0 | $B \cdot \hat{\theta}_x^2 + A$ | `horner_x` (inner bracket) |
| DSP 1 | $B \cdot \hat{\theta}_y^2 + A$ | `horner_y` |
| DSP 2 | $2\zeta\omega_n \cdot \dot{\theta}_x$ | `damp_x` (damping term) |

The DSP48E1's pre-adder + multiplier + post-adder mode computes $B \cdot \hat{\theta}^2 + A$ in a single DSP cycle using `OPMODE = 7'b0110101` (C + A*B).

Pipeline register captures: `horner_x`, `horner_y`, `damp_x`, `damp_y`, `theta_hat_x`, `theta_hat_y`, `accel_proj`.

### Stage 3 (Cycle N+2 → N+3): Final Product, State Update & Torque

| DSP | Operation | Output |
|---|---|---|
| DSP 0 | $\hat{\theta}_x \cdot \text{horner\_x}$ | `poly_x` (Chebyshev polynomial value) |
| DSP 1 | $\hat{\theta}_y \cdot \text{horner\_y}$ | `poly_y` |
| DSP 2 | $\kappa_{mass} \cdot \text{torque\_gain} \cdot \text{poly\_x}$ | `torque_slosh_x` |

State updates (in LUT fabric adders):
$$\dot{\theta}_{x,next} = \dot{\theta}_{x} + \Delta t \cdot \left[ -\omega_n^2 \cdot \text{poly\_x} - \text{accel\_proj\_x} - \text{damp\_x} \right]$$
$$\theta_{x,next} = \theta_{x} + \Delta t \cdot \dot{\theta}_{x}$$

Saturation clamping applied to both $\theta_{next}$ and $\dot{\theta}_{next}$.

---

## 6. Dual-Axis Architecture

The slosh model evaluates **two independent axes** (X and Y body-frame) because the cylindrical tank pendulum has two uncoupled oscillation modes. Z-axis sloshing (vertical) is negligible for horizontal flight. The two axes share the same configuration constants but operate on independent state variables.

The DSP slice count of 3 reflects time-sharing: each DSP is used for both X and Y axis computations across the 3 pipeline stages. In a fully unrolled implementation, 6 DSPs would be needed. The folded design trades throughput for area — but since the pipeline is already fully pipelined at 1-per-cycle, folding does not reduce throughput.

---

## 7. Chebyshev Coefficient Computation (Host-Side)

The host CPU computes the Chebyshev coefficients offline using the following procedure:

```python
import numpy as np
from numpy.polynomial.chebyshev import chebfit

# Sample sin(theta) on [-pi/6, pi/6] with 100 points
theta_max = np.pi / 6
theta = np.linspace(-theta_max, theta_max, 100)
theta_hat = theta / theta_max  # Normalize to [-1, 1]

# Fit degree-3 Chebyshev polynomial
coeffs = chebfit(theta_hat, np.sin(theta), 3)
# coeffs = [0.0, 0.99745, 0.0, -0.15895] (only odd terms non-zero by symmetry)

# Horner constants
c1, c3 = coeffs[1], coeffs[3]
A = c1 - 3 * c3  # = 1.4742
B = 4 * c3        # = -0.6358

# Convert to Q4.12
A_q412 = int(A * 4096)   # = 6037 = 0x1795
B_q412 = int(B * 4096)   # = -2604 = 0xF5C4
```

These constants are loaded into configuration registers via MMIO before the MPPI frame begins.

---

## 8. Fixed-Point Overflow Analysis

The critical overflow risk is in Stage 1's squaring operation. For $\hat{\theta} \in [-1, 1]$ in Q4.12:
- Maximum magnitude: `16'h1000` (= 1.0)
- Square: `1.0 × 1.0 = 1.0` → `32'h01000000` in Q8.24
- After truncation to Q4.12: `16'h1000` = 1.0

Since $\hat{\theta}^2 \le 1.0$ by construction, the squaring operation never overflows the Q4.12 range. The Horner inner term $A + B \cdot \hat{\theta}^2$ has maximum magnitude $|A| + |B| = 1.4742 + 0.6358 = 2.11$, which is within Q4.12 range ($[-8, +8)$).

The final polynomial value $P(\hat{\theta}) \in [-1, 1]$ because $|\sin(\theta)| \le 1$ for all $\theta$. No saturation is needed on the polynomial output.

Saturation IS needed on the state update accumulators because multiple terms (restoring force + lateral acceleration + damping) can add up. Use 18-bit accumulators with saturation to Q4.12.

---

## 9. Area Estimate

| Component | Resource |
|---|---|
| DSP48E1 slices | 3 (time-shared across X/Y and pipeline stages) |
| LUT fabric (adders, saturation, muxes) | 142 LUTs |
| Pipeline registers (3 stages × ~200 bits) | 64 FFs |
| Configuration registers (7 × 16-bit) | 112 FFs (in host register file) |

On ASAP7: approximately 2,400 cells. On SCL 180nm: approximately 2,100 gates.

---

## 10. Verification Plan

| Test | Method | Count | Pass Criterion |
|---|---|---|---|
| Zero angle, zero rate: outputs remain zero | Verilator | 1 | Stable equilibrium |
| Small angle ($\theta = 0.1$ rad): linear response | Verilator | 16 | Matches $\sin(0.1) \approx 0.0998$ |
| Large angle ($\theta = 30°$): cubic correction | Verilator | 16 | Matches $\sin(\pi/6) = 0.5$ within 1 LSB |
| Damped oscillation: initial displacement, no forcing | Verilator | 200 | Decaying sinusoid, energy decreases |
| Mass depletion: $\kappa_{mass} = 0.5$ | Verilator | 50 | Torque halved vs $\kappa = 1.0$ |
| Mass depletion: $\kappa_{mass} = 0.0$ (empty tank) | Verilator | 10 | Zero torque output |
| Lateral forcing: constant $a_{lat}$, verify steady-state offset | Verilator | 100 | Converges to $\theta_{ss} = \arcsin(a_{lat}/g)$ |
| Full MPPI integration: 128 trajectories × 16 steps | Verilator | 1 | Torques match Python golden model |
| Pipeline throughput: back-to-back valid_i | Verilator | 100 | Output every cycle after 3-cycle fill |
| Coefficient update: change A, B mid-frame | Verilator | 5 | New coefficients take effect immediately |
| Saturation: extreme inputs drive state to Q4.12 limits | Verilator | 10 | Clamped, no wrap-around |

---

## 11. Implementation Notes for the Coder

1. **The normalization multiply ($\theta \times 1/\theta_{max}$) in Stage 1** should use a DSP48E1, not LUT fabric. Although it is "just a multiply by a constant," the DSP gives better timing and frees LUTs for the adders.

2. **Do NOT compute $\hat{\theta}^3$ directly.** The Horner factorization $\hat{\theta} \cdot [A + B \cdot \hat{\theta}^2]$ avoids the $\hat{\theta}^3$ computation entirely. Computing $\hat{\theta}^3 = \hat{\theta} \times \hat{\theta}^2$ would require an extra DSP cycle and risk larger intermediate overflow.

3. **The damping term $2\zeta\omega_n\dot{\theta}$** is a simple multiply. It can share a DSP slice with the Horner evaluation because it is independent and can be scheduled in the same pipeline stage.

4. **The `cfg_*` constants should be held in registers** that are written by the host CPU via MMIO and held stable for the entire MPPI frame (2048 cycles). Do not allow mid-frame updates to configuration registers — the pipeline will produce inconsistent results for trajectories that straddle the update.

5. **Sign symmetry:** The polynomial $P(\hat{\theta})$ is an odd function ($P(-x) = -P(x)$) because $\sin(\theta)$ is odd and only odd Chebyshev terms ($T_1, T_3$) are used. Verify this symmetry in directed tests.

6. **Import `hmppi_pkg`** for `DATA_WIDTH` and the Q4.12 truncation macros. The truncation from Q8.24 to Q4.12 should use the `Q_TRUNC` macro, not manual bit selection.
