# Design: `attitude_kinematics_engine.sv` — 6-DoF Rigid-Body Attitude & Velocity Propagator

**Spec Reference:** §3.2, Decision H2 (Fixed-Point Rigid-Body Dynamics)
**Target:** Standalone H-MPPI & Integrated Monolithic SoC
**Hardware Cost:** 8 DSP48E1 Slices, 210 LUTs, 96 Flip-Flops

---

## 1. Purpose & First Principles

In agile quadrotor trajectory planning, attitude dynamics cannot be treated as decoupled, linear second-order systems. When executing aggressive collision-avoidance maneuvers with a suspended sloshing payload, the airframe experiences high angular velocities ($\omega > 3.0\text{ rad/s}$), causing significant non-linear gyroscopic coupling:
$$\mathbf{J} \dot{\mathbf{\omega}} + \mathbf{\omega} \times (\mathbf{J}\mathbf{\omega}) = \mathbf{\tau}_{ctrl} + \mathbf{\tau}_{slosh}$$

where $\mathbf{J} = \text{diag}(I_{xx}, I_{yy}, I_{zz})$ is the vehicle's moment of inertia tensor, $\mathbf{\tau}_{ctrl}$ is the rotor actuation torque, and $\mathbf{\tau}_{slosh}$ is the reactionary torque transmitted by the fluid payload.

### Why Not Quaternion Propagation?

Standard attitude integration propagates a unit quaternion $\mathbf{q} = [q_w, q_x, q_y, q_z]^T$:
$$\dot{\mathbf{q}} = \frac{1}{2} \mathbf{\Omega}(\mathbf{\omega}) \mathbf{q}$$

However, integrating quaternions over a 16-step prediction horizon accumulates floating-point normalization drift. Enforcing the unit-norm constraint $\lVert \mathbf{q} \rVert = 1$ requires evaluating an inverse square root $(q_w^2 + q_x^2 + q_y^2 + q_z^2)^{-1/2}$ at every horizon step. On fixed-point hardware, this requires either a Newton-Raphson iterative square root (3 multiplications × 3 iterations = 9 DSP cycles) or a CORDIC pipeline (20+ cycles). Both approaches consume critical DSP resources and add pipeline latency that breaks the 1,218-cycle frame budget.

### The Euler-Rate Forward Integration Solution

For local prediction horizons of $T = 16$ steps ($\Delta t = 0.020\text{ s}$, total horizon $0.32\text{ s}$), the quadrotor body attitude remains within the non-singular flight envelope ($|\theta| < 70°$). Under this assumption, we reformulate the attitude dynamics in body-fixed angular velocity coordinates, directly updating angular rates using forward Euler integration:

$$\omega_{x,t+1} = \omega_{x,t} + \Delta t \cdot \frac{1}{I_{xx}} \left( \tau_{ctrl,x,t} + \tau_{slosh,x,t} - (I_{zz} - I_{yy}) \omega_{y,t} \omega_{z,t} \right)$$
$$\omega_{y,t+1} = \omega_{y,t} + \Delta t \cdot \frac{1}{I_{yy}} \left( \tau_{ctrl,y,t} + \tau_{slosh,y,t} - (I_{xx} - I_{zz}) \omega_{x,t} \omega_{z,t} \right)$$
$$\omega_{z,t+1} = \omega_{z,t} + \Delta t \cdot \frac{1}{I_{zz}} \left( \tau_{ctrl,z,t} - (I_{yy} - I_{xx}) \omega_{x,t} \omega_{y,t} \right)$$

The translational dynamics use 2nd-order Taylor expansion for position:
$$\mathbf{p}_{t+1} = \mathbf{p}_t + \Delta t \cdot \mathbf{v}_t + \frac{1}{2} \Delta t^2 \cdot \mathbf{a}_t$$
$$\mathbf{v}_{t+1} = \mathbf{v}_t + \Delta t \cdot \mathbf{a}_t$$

where $\mathbf{a}_t$ is the body-frame acceleration computed from thrust and gravity.

This eliminates all square roots, trigonometric functions, and quaternion normalization from the dynamics pipeline, reducing the entire attitude propagation to **multiply-accumulate chains** that map directly onto DSP48E1 slices.

---

## 2. Micro-Architecture & Cross-Product Acceleration

### 2.1 Gyroscopic Moment Decomposition

The cross-product vector $\mathbf{\omega} \times (\mathbf{J}\mathbf{\omega})$ expands to three scalar gyroscopic moments:

$$M_x = (I_{zz} - I_{yy}) \cdot \omega_y \cdot \omega_z$$
$$M_y = (I_{xx} - I_{zz}) \cdot \omega_x \cdot \omega_z$$
$$M_z = (I_{yy} - I_{xx}) \cdot \omega_x \cdot \omega_y$$

By pre-computing the normalized gyroscopic coefficients as fixed-point configuration constants:
$$K_x = \frac{(I_{zz} - I_{yy})}{I_{xx}} \cdot \Delta t, \quad K_y = \frac{(I_{xx} - I_{zz})}{I_{yy}} \cdot \Delta t, \quad K_z = \frac{(I_{yy} - I_{xx})}{I_{zz}} \cdot \Delta t$$

and the torque scaling constants:
$$G_x = \frac{\Delta t}{I_{xx}}, \quad G_y = \frac{\Delta t}{I_{yy}}, \quad G_z = \frac{\Delta t}{I_{zz}}$$

the entire angular velocity update collapses to six fixed-point multiplications and three additions:
$$\omega_{x,t+1} = \omega_{x,t} + K_x \cdot \omega_{y,t} \cdot \omega_{z,t} + G_x \cdot (\tau_{ctrl,x,t} + \tau_{slosh,x,t})$$

### 2.2 Pipeline Schedule (2-Stage)

The pipeline is structured to maximize DSP48E1 utilization across the 8 available slices:

**Stage 1 (cycle N → N+1): Pairwise Products**

| DSP | Operation | Result |
|---|---|---|
| DSP 0 | $\omega_y \times \omega_z$ | `prod_yz` (32-bit, Q8.24 intermediate) |
| DSP 1 | $\omega_x \times \omega_z$ | `prod_xz` |
| DSP 2 | $\omega_x \times \omega_y$ | `prod_xy` |
| DSP 3 | $v_x \times \Delta t$ | `dpos_x` (position delta) |
| DSP 4 | $v_y \times \Delta t$ | `dpos_y` |
| DSP 5 | $v_z \times \Delta t$ | `dpos_z` |
| DSP 6 | $a_x \times \frac{\Delta t^2}{2}$ | `dpos2_x` (2nd-order position term) |
| DSP 7 | $a_y \times \frac{\Delta t^2}{2}$ | `dpos2_y` |

**Stage 2 (cycle N+1 → N+2): Scaled Accumulation**

| DSP | Operation | Result |
|---|---|---|
| DSP 0 | $K_x \times \text{prod\_yz}$ | Gyroscopic moment X (Q4.12 output after truncation) |
| DSP 1 | $K_y \times \text{prod\_xz}$ | Gyroscopic moment Y |
| DSP 2 | $K_z \times \text{prod\_xy}$ | Gyroscopic moment Z |
| DSP 3 | $G_x \times (\tau_{ctrl,x} + \tau_{slosh,x})$ | Torque contribution X |
| DSP 4 | $G_y \times (\tau_{ctrl,y} + \tau_{slosh,y})$ | Torque contribution Y |
| DSP 5 | $G_z \times \tau_{ctrl,z}$ | Torque contribution Z (no Z-axis slosh) |
| DSP 6 | $a_z \times \frac{\Delta t^2}{2}$ | `dpos2_z` (deferred from Stage 1) |
| DSP 7 | — | Idle (or used for saturation check) |

Final accumulation is done in fabric LUTs (not DSPs) using carry-chain adders:
$$\omega_{x,next} = \text{sat}(\omega_{x} + \text{gyro\_x} + \text{torque\_x})$$
$$p_{x,next} = \text{sat}(p_x + \text{dpos\_x} + \text{dpos2\_x})$$

---

## 3. Hardware Interface & Port Definition

```systemverilog
module attitude_kinematics_engine #(
    parameter DATA_WIDTH = 16,         // Q4.12 fixed-point representation
    parameter COEFF_WIDTH = 16         // Configuration coefficient width
)(
    input  logic                     clk_i,
    input  logic                     rst_ni,

    // Handshake
    input  logic                     valid_i,       // New state + control inputs ready
    output logic                     valid_o,       // Propagated state ready (2-cycle latency)
    output logic                     ready_o,       // Can accept new input

    // Current angular velocity state (body-frame, rad/s in Q4.12)
    input  logic signed [DATA_WIDTH-1:0] omega_x_i, omega_y_i, omega_z_i,

    // Control torques (from mixer, N·m in Q4.12)
    input  logic signed [DATA_WIDTH-1:0] tau_ctrl_x_i, tau_ctrl_y_i, tau_ctrl_z_i,

    // Slosh reaction torques (from Chebyshev slosh model, N·m in Q4.12)
    input  logic signed [DATA_WIDTH-1:0] tau_slosh_x_i, tau_slosh_y_i,
    // Note: Z-axis slosh torque is zero (pendulum model is planar X/Y only)

    // Current translational state (NED frame, m and m/s in Q4.12)
    input  logic signed [DATA_WIDTH-1:0] pos_x_i, pos_y_i, pos_z_i,
    input  logic signed [DATA_WIDTH-1:0] vel_x_i, vel_y_i, vel_z_i,
    input  logic signed [DATA_WIDTH-1:0] acc_x_i, acc_y_i, acc_z_i,

    // Propagated angular velocity outputs (2-cycle latency)
    output logic signed [DATA_WIDTH-1:0] omega_x_next_o, omega_y_next_o, omega_z_next_o,

    // Propagated translational state outputs (2-cycle latency)
    output logic signed [DATA_WIDTH-1:0] pos_x_next_o, pos_y_next_o, pos_z_next_o,
    output logic signed [DATA_WIDTH-1:0] vel_x_next_o, vel_y_next_o, vel_z_next_o,

    // Configuration coefficients (loaded by host CPU, held constant during frame)
    input  logic signed [COEFF_WIDTH-1:0] cfg_Kx_i, cfg_Ky_i, cfg_Kz_i,    // Gyroscopic coefficients
    input  logic signed [COEFF_WIDTH-1:0] cfg_Gx_i, cfg_Gy_i, cfg_Gz_i,    // Torque gain coefficients
    input  logic signed [COEFF_WIDTH-1:0] cfg_dt_i,                         // Time step (Q4.12)
    input  logic signed [COEFF_WIDTH-1:0] cfg_dt2_half_i                    // dt²/2 (Q4.12)
);
```

### Timing Contract
- **Latency:** 2 clock cycles from `valid_i` to `valid_o`.
- **Throughput:** 1 state propagation per cycle (fully pipelined).
- **Backpressure:** `ready_o` deasserts only during reset. The pipeline never stalls under normal operation.

---

## 4. Fixed-Point Arithmetic & Overflow Protection

### Q4.12 Format

All state variables and intermediate products use signed Q4.12 fixed-point:
- 1 sign bit + 3 integer bits + 12 fractional bits = 16 bits.
- Range: $[-8.0, +7.999755859375]$.
- Resolution: $2^{-12} \approx 0.000244$.

Angular velocities in the range $[-7.99, +7.99]$ rad/s cover all physically realizable quadrotor maneuvers (maximum angular rate for most racing drones is ~5 rad/s).

### Intermediate Product Width

DSP48E1 multiplies 18×25 → 48-bit result. For Q4.12 × Q4.12:
- Full product: 32 bits (Q8.24).
- After truncation back to Q4.12: take bits [27:12] of the 32-bit product (right-shift by 12, preserving sign).

```systemverilog
// Q4.12 × Q4.12 → Q4.12 with truncation
logic signed [31:0] full_product;
logic signed [15:0] truncated;

assign full_product = omega_y * omega_z;  // 16×16 = 32 bits
assign truncated = full_product[27:12];    // Truncate fractional bits
```

### Saturation Clamping

After the final accumulation, each output is saturated to the Q4.12 range to prevent wrap-around:

```systemverilog
function automatic logic signed [15:0] saturate(logic signed [17:0] val);
    if (val > 18'sh07FFF)      return 16'sh7FFF;   // Positive clamp
    else if (val < 18'shF8000) return 16'sh8000;    // Negative clamp
    else                       return val[15:0];
endfunction
```

The accumulation uses 18-bit intermediates (2 extra bits for headroom) to detect overflow before clamping.

---

## 5. Configuration Register Precomputation

The host CPU computes the following constants before starting the MPPI frame and writes them to the configuration registers via MMIO:

| Register | Formula | Typical Value (Q4.12) | Physical Meaning |
|---|---|---|---|
| `cfg_Kx` | $(I_{zz} - I_{yy})/I_{xx} \cdot \Delta t$ | `16'h0003` (~0.00073) | Roll gyroscopic coupling |
| `cfg_Ky` | $(I_{xx} - I_{zz})/I_{yy} \cdot \Delta t$ | `16'hFFFD` (~-0.00073) | Pitch gyroscopic coupling |
| `cfg_Kz` | $(I_{yy} - I_{xx})/I_{zz} \cdot \Delta t$ | `16'h0000` (~0.0) | Yaw gyroscopic coupling (near-zero for symmetric frames) |
| `cfg_Gx` | $\Delta t / I_{xx}$ | `16'h0051` (~0.0199) | Roll torque sensitivity |
| `cfg_Gy` | $\Delta t / I_{yy}$ | `16'h0051` (~0.0199) | Pitch torque sensitivity |
| `cfg_Gz` | $\Delta t / I_{zz}$ | `16'h0029` (~0.0101) | Yaw torque sensitivity |
| `cfg_dt` | $\Delta t$ | `16'h0052` (0.0200) | Integration time step |
| `cfg_dt2_half` | $\Delta t^2 / 2$ | `16'h0001` (0.0002) | 2nd-order position term |

These are computed once per MPPI frame (~every 20 ms) and remain constant across all 128 trajectories and 16 horizon steps. The CPU computes them in floating-point and converts to Q4.12 before writing.

---

## 6. Integration with H-MPPI Pipeline

The attitude kinematics engine sits at pipeline stage 2 in the H-MPPI dataflow:

```
Stage 1: sobol_qmc_4d → Generate perturbation δu[k]
Stage 2: attitude_kinematics_engine → Propagate ω[k,t+1], p[k,t+1], v[k,t+1]
Stage 3: slosh_chebyshev → Compute θ_slosh from body angular rates
Stage 4: trajectory_cost_accumulator → Accumulate S[k] += L(x, u)
```

The engine receives the perturbed control input $\mathbf{u}_{k,t} = \mathbf{v}_{nom,t} + \delta\mathbf{u}_{k,t}$ as its `tau_ctrl` inputs (after the control allocation/mixer lookup). The propagated state flows to the slosh model and cost accumulator.

In the folded pipeline (4 lanes processing 128 trajectories), the kinematics engine is replicated **4 times** — one per lane. Each instance processes an independent trajectory sample with no data dependencies between lanes.

---

## 7. Area Estimate

| Component | Resource |
|---|---|
| DSP48E1 slices (8 multiplications) | 8 (shared with other pipeline stages via time-division) |
| LUT fabric (adders, saturation, muxes) | 210 LUTs |
| Pipeline registers (2 stages × ~300 bits) | 96 FFs |
| Configuration registers (8 × 16-bit) | 128 FFs (in host-side register file, not counted here) |

On ASAP7: approximately 3,200 cells. On SCL 180nm: approximately 2,800 gates.

---

## 8. Verification Plan

| Test | Method | Count | Pass Criterion |
|---|---|---|---|
| Zero torque, zero angular velocity | Verilator | 1 | All outputs remain zero |
| Constant torque, zero initial ω | Verilator | 16 | Linear ramp in ω, quadratic position |
| Gyroscopic coupling: ωx=ωy=1 rad/s, ωz=0 | Verilator | 16 | ωz develops from cross-coupling |
| Saturation: large torque drives ω to Q4.12 limit | Verilator | 5 | Output clamped, no wrap-around |
| Slosh torque injection: verify τ_slosh adds to ω | Verilator | 10 | Correct additive effect |
| Random inputs: compare against Python golden model | Verilator | 10,000 | Max error < 1 LSB (Q4.12) |
| Pipeline throughput: back-to-back valid_i | Verilator | 100 | Output every cycle after 2-cycle fill |
| Configuration change: update Kx mid-frame | Verilator | 5 | New coefficient takes effect next cycle |
| Position 2nd-order: verify quadratic term | Verilator | 20 | Position matches $p + vt + \frac{1}{2}at^2$ |

---

## 9. Implementation Notes for the Coder

1. **Use the DSP48E1 `OPMODE` to chain multiply-accumulate** where possible. For example, Stage 2's `Kx × prod_yz + omega_x` can be done in a single DSP cycle using the pre-adder and accumulator. Consult Xilinx UG479 for OPMODE patterns.

2. **The Z-axis slosh torque is hardwired to zero.** The pendulum slosh model (doc 63) only produces X and Y torques. `tau_slosh_z` does not exist as a port. The Z-axis angular velocity update uses only the control torque and gyroscopic moment.

3. **Sign extension is critical.** When truncating a 32-bit product to 16 bits, take `full_product[27:12]` — NOT `[31:16]`. The latter loses the fractional precision. If `full_product[31] != full_product[27]`, overflow has occurred and saturation must be applied.

4. **Pipeline valid propagation:** Use a 2-stage shift register for `valid_o`. No stalling, no backpressure, no bubbles.

5. **The acceleration input `acc_i`** is computed externally from thrust and gravity: $\mathbf{a} = T/m \cdot \hat{z}_{body} - g\hat{z}_{NED}$. This module does NOT compute acceleration — it only propagates position and velocity using the provided acceleration.

6. **Import `hmppi_pkg`** for `DATA_WIDTH`, `COEFF_WIDTH`, and the Q4.12 scaling constants. Do not hardcode `16` or `12` in the truncation logic.
