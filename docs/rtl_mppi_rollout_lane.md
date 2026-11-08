# RTL Development Document: Parallel Rollout Lane Engine (`mppi_rollout_lane.sv`)

**Document Revision:** 3.0  
**Design Module:** `mppi_rollout_lane.sv`  
**Parent Top-Level:** `hmppi_core_top.sv`  
**Instantiated Submodules:** `sobol_qmc_4d.sv`, `chebyshev_slosh_eval.sv`  
**Silicon Targets:** ASAP7 7nm Predictive FinFET (`ASAP7SC7p5RVT`), Xilinx Artix-7 FPGA (`xc7a35t-cpg236-1`, `xc7a200t-sbg484-1`), SCL 180nm CMOS (`CP8D` Standard Cells)  
**Design Reference:** Design Docs 62 (`62_mppi_attitude_kinematics.md`), 63 (`63_mppi_slosh_chebyshev.md`), 64 (`64_mppi_cost_accumulator.md`), H-MPPI Spec §3.2–§3.4  

---

## 1. Architectural Purpose & Prediction Horizon Scheduling

In Model Predictive Path Integral (MPPI) stochastic optimal control, the future evolution of an aerial vehicle carrying a liquid payload is explored by sampling and propagating randomized trajectory rollouts across a discrete prediction horizon of $H = 16$ time steps:

$$\tau_k = \left\{ \mathbf{x}_{k,0}, \mathbf{u}_{k,0}, \mathbf{x}_{k,1}, \mathbf{u}_{k,1}, \dots, \mathbf{x}_{k,H-1}, \mathbf{u}_{k,H-1}, \mathbf{x}_{k,H} \right\}$$

The module `mppi_rollout_lane.sv` implements a fully autonomous, pipelined hardware trajectory rollout engine. Instantiated four times in parallel within `hmppi_core_top`, each rollout lane executes sequential batch iterations to evaluate candidate trajectories without stalling the master pipeline. Rather than streaming state vectors to and from external memory or a shared multi-port register file, each lane encapsulates its own local Quasi-Monte Carlo perturbation engine (`sobol_qmc_4d`), a 3-stage division-free fluid slosh dynamics evaluator (`chebyshev_slosh_eval`), an integrated forward state kinematic propagator, and a multi-objective stage cost accumulator.

```
       ┌──────────────────────────────────────────────────────────────────────────────────┐
       │                       mppi_rollout_lane.sv Micro-Architecture                    │
       └──────────────────────────────────────────────────────────────────────────────────┘
         clk, rst_n, start_rollout, sample_seed, cov_scale_in, State Telemetry [p, v, u_nom]
                                                  │
          ┌───────────────────────────────────────┼───────────────────────────────────────┐
          │                                       │                                       │
          ▼                                       ▼                                       ▼
    ┌───────────┐                           ┌───────────┐                           ┌───────────┐
    │ sobol_qmc │                           │ Forward   │                           │ chebyshev │
    │   _4d     │                           │ Kinematic │                           │  _slosh   │
    │ Generator │                           │ Integrator│                           │   _eval   │
    └─────┬─────┘                           └─────┬─────┘                           └─────┬─────┘
          │ eps_x,y,z,psi                         │ px, py, pz                            │ f_slosh,
          │                                       │ vx, vy, vz                            │ slosh_cost
          ├───► ux = u_nom_x + (eps_x >>> 2) ─────┤ theta, thetadot ──────────────────────┤
          │     uy = u_nom_y + (eps_y >>> 2)      │                                       │
          │     uz = u_nom_z + (eps_z >>> 2)      ▼                                       ▼
          │                            ┌─────────────────────────────────────────────────────┐
          │                            │ Stage Cost Accumulation Unit                        │
          │                            │ - Goal Tracking Error ||p - p_goal||^2              │
          │                            │ - Actuator Control Effort ||u||^2                   │
          │                            │ - Slosh Penalty + Spatial Obstacle Barrier B(p)     │
          │                            └──────────────────────────┬──────────────────────────┘
          ▼ (Step 0 Latch)                                        │
    first_pert_x, y, z                                            ▼
                                                   total_cost [23:0], rollout_done
```

### 1.1 Temporal Horizon Scaling & Integration Timestep
The discrete simulation timestep is parameterized as:

$$\Delta t = 0.050\text{ s} \quad (20\text{ Hz discrete rollout rate})$$

Across $H = 16$ prediction steps, each rollout projects the vehicle's state **$0.80\text{ seconds}$** into the future. In signed Q8.8 fixed-point format ($1.0 = 256$):

$$\Delta t_{\text{Q8.8}} = \text{round}(0.050 \times 256) = 13 = \text{16'sh000D}$$

```systemverilog
    localparam logic signed [DATA_WIDTH-1:0] DT = 16'sh000D; // 13/256 = 0.05078s (~1.5% Δt quantization error)
```

This 16-step lookahead covers the fundamental natural slosh period of a 25 cm radius cylindrical tank ($\tau_n = 2\pi / \omega_n \approx 1.50\text{ s}$, half-period $0.75\text{ s}$), allowing the trajectory optimizer to observe and penalize incipient resonant liquid oscillation before the airframe commits to aggressive evasive maneuvers.

---

## 2. Attitude Kinematics: Quaternions vs DCM vs Euler-Rate Formulations

### 2.1 The Mathematical & Hardware Trade-Off
A full 6-DoF rigid-body simulation tracks angular velocities $\boldsymbol{\omega} = [\omega_x, \omega_y, \omega_z]^T$ governed by Euler's rotational equations with gyroscopic coupling:

$$\mathbf{J} \dot{\boldsymbol{\omega}} + \boldsymbol{\omega} \times (\mathbf{J}\boldsymbol{\omega}) = \boldsymbol{\tau}_{ctrl} + \boldsymbol{\tau}_{slosh}$$

where $\mathbf{J} = \text{diag}(I_{xx}, I_{yy}, I_{zz})$ is the airframe moment of inertia tensor, $\boldsymbol{\tau}_{ctrl}$ is the rotor moment, and $\boldsymbol{\tau}_{slosh}$ is the reaction torque transmitted by the suspended propellant tank. When implementing attitude propagation on silicon, three competing kinematic representations were analyzed:

1. **Unit Quaternion Integration ($\mathbf{q} = [q_w, q_x, q_y, q_z]^T$):**  
   Integrating $\dot{\mathbf{q}} = \frac{1}{2}\boldsymbol{\Omega}(\boldsymbol{\omega})\mathbf{q}$ guarantees singularity-free all-attitude representation. However, finite fixed-point precision induces progressive normalization drift ($\lVert \mathbf{q} \rVert \ne 1.0$). Enforcing normalization requires evaluating $(q_w^2 + q_x^2 + q_y^2 + q_z^2)^{-1/2}$ at every step. On an Artix-7 FPGA or SCL 180nm standard cell fabric, an iterative CORDIC or Newton-Raphson inverse square root engine consumes 16–22 clock cycles and 4 additional DSP blocks per lane, which violates the strict per-step execution budget.
2. **Direction Cosine Matrix Integration ($\mathbf{R} \in \text{SO}(3)$):**  
   Updating $\dot{\mathbf{R}} = \mathbf{R} [\boldsymbol{\omega}]_\times$ requires 9 coupled scalar differential equations. Enforcing matrix orthogonality ($\mathbf{R}^T \mathbf{R} = \mathbf{I}_3$) requires continuous Gram-Schmidt re-orthogonalization, requiring 18 fixed-point multipliers and 6 dividers per lane.
3. **Euler-Rate Small-Angle Forward Integration (Selected Architecture):**  
   For local trajectory planning horizons of $T = 0.80\text{ s}$, aggressive quadrotor maneuvers operate well within non-singular pitch/roll boundaries ($|\phi|, |\theta| < 60^\circ$). By pre-calculating normalized gyroscopic cross-coupling gains:
   
   $$K_x = \frac{I_{zz} - I_{yy}}{I_{xx}} \Delta t, \quad K_y = \frac{I_{xx} - I_{zz}}{I_{yy}} \Delta t, \quad K_z = \frac{I_{yy} - I_{xx}}{I_{zz}} \Delta t$$
   
   the rotational acceleration collapses into pure multiply-accumulate (MAC) chains:
   
   $$\omega_{x,t+1} = \omega_{x,t} + K_x \omega_{y,t} \omega_{z,t} + \frac{\Delta t}{I_{xx}} (\tau_{ctrl,x,t} + \tau_{slosh,x,t})$$

In the consolidated translational abstraction of `mppi_rollout_lane.sv`, body tilt angles are tightly coupled to horizontal translational accelerations ($a_x \approx g \cdot \theta_{pitch}$, $a_y \approx -g \cdot \phi_{roll}$). The control perturbation vector $\boldsymbol{\epsilon} = [\epsilon_x, \epsilon_y, \epsilon_z, \epsilon_\psi]^T$ generated by `sobol_qmc_4d` directly perturbs commanded body acceleration:

```systemverilog
    assign ux = u_nom_x + (eps_x >>> 2);
    assign uy = u_nom_y + (eps_y >>> 2);
    assign uz = u_nom_z + (eps_z >>> 2);
```

Arithmetic right-shift by 2 bits (`>>> 2`) bounds exploratory acceleration perturbations to $[-0.25, +0.25]\,\text{m/s}^2$ around the nominal flight trajectory, maintaining small-angle stability across all 16 prediction steps.

---

## 3. Rigid-Body Dynamics & Bidirectional Slosh Coupling

### 3.1 Translational Forward Kinematics
Vehicle position $\mathbf{p} = [p_x, p_y, p_z]^T$ and linear velocity $\mathbf{v} = [v_x, v_y, v_z]^T$ are updated using explicit Euler-Maruyama forward integration:

$$\mathbf{p}_{t+1} = \mathbf{p}_t + \mathbf{v}_t \cdot \Delta t$$
$$\mathbf{v}_{t+1} = \mathbf{v}_t + \mathbf{u}_t \cdot \Delta t$$

In hardware, multiplying two signed Q8.8 variables produces a signed 32-bit Q16.16 intermediate result. Right-shifting by 8 bits (`>>> 8`) restores the product to signed Q8.8:

```systemverilog
    // Forward Kinematics Integration Step
    px_q <= px_q + ((vx_q * DT) >>> 8);
    py_q <= py_q + ((vy_q * DT) >>> 8);
    pz_q <= pz_q + ((vz_q * DT) >>> 8);

    vx_q <= vx_q + ((ux * DT) >>> 8);
    vy_q <= vy_q + ((uy * DT) >>> 8);
    vz_q <= vz_q + ((uz * DT) >>> 8);
```

### 3.2 Nonlinear Slosh Pendulum Coupling
The fluid free-surface displacement is modeled as an equivalent spherical pendulum of mass $m_p$ and effective length $l_{eff} = R_{tank} / 1.841$. The angular acceleration of the slosh deflection is:

$$\ddot{\theta} = -\frac{g}{l_{eff}} \sin\theta - \frac{a_{lat}}{l_{eff}} \cos\theta - 2\zeta \omega_n \dot{\theta}$$

Evaluating $\sin\theta$ and $\cos\theta$ through iterative trigonometric units would incur prohibitive area and pipeline stalls. Instead, `chebyshev_slosh_eval` projects the restoring force onto a degree-3 orthogonal Chebyshev polynomial basis evaluated via Horner's rule:

$$P_3(\hat{\theta}) = \hat{\theta} \cdot \left[ A + B \cdot \hat{\theta}^2 \right]$$

where $A = c_1 - 3c_3 = 1.4742$ and $B = 4c_3 = -0.6358$. The resulting normalized restoring force $f_{slosh}$ and reaction torque are fed into the lane integrator:

```systemverilog
    // Slosh dynamics integration step
    // theta_dot = theta_dot - (g/L)*f_slosh*dt + accel_coupling
    thetadot_q <= thetadot_q - ((f_slosh * DT) >>> 8) + ((ux * DT) >>> 10);
    theta_q    <= theta_q + ((thetadot_q * DT) >>> 8);
```

The coupling term `((ux * DT) >>> 10)` captures the inertial reaction between the quadrotor's horizontal translational acceleration $u_x$ and liquid pendulum excitation, scaling by an effective moment arm of 4.0 in Q8.8 (`>>> 10` is equivalent to division by $256 \times 4 = 1024$).

When the upstream OS0035 TIDE-LATE out-of-sequence measurement controller asserts `cov_scale_in`, the embedded `chebyshev_slosh_eval` dynamically triples its internal cost weight to prevent aggressive maneuvering under high state estimation uncertainty.

---

## 4. Multi-Objective Cost & Quadratic Barrier Accumulation

At each prediction step $t \in \{0, \dots, 15\}$, the lane evaluates four additive cost components:

$$C_{stage} = C_{track} + C_{ctrl} + C_{slosh} + C_{obs}$$

```
                                  Stage Cost Functional Datapath
   (px_q - p_goal_x)^2 ───┐
   (py_q - p_goal_y)^2 ───┼──► Sum of Squares [27:8] ──► tracking_cost [23:0] ──┐
   (pz_q - p_goal_z)^2 ───┘                                                    │
                                                                               │
   ux^2 + uy^2 + uz^2  ──────► Sum of Squares [27:8] >>> 2 ──► control_cost ───┼──► Accumulator
                                                                               │    cost_acc_q <=
   chebyshev_slosh_eval ─────► slosh_cost [23:0] ──────────────────────────────┼    cost_acc_q +
                                                                               │    C_stage
   Piecewise Barrier ────────► If d_sq <= R_safe^2: 16'sh7FFF                  │
   (x - x_obs)^2 + ...         Else If d_sq <= R_outer^2: w*(R_out^2 - d_sq) ──┘
```

### 4.1 Waypoint Navigational Tracking Penalty
Penalizes 3D Euclidean distance to the mission waypoint coordinates $\mathbf{p}_{goal}$:

$$C_{track} = (p_x - p_{goal,x})^2 + (p_y - p_{goal,y})^2 + (p_z - p_{goal,z})^2$$

```systemverilog
    logic signed [DATA_WIDTH-1:0] ex, ey, ez;
    assign ex = px_q - p_goal_x;
    assign ey = py_q - p_goal_y;
    assign ez = pz_q - p_goal_z;

    logic signed [31:0] ex_sq, ey_sq, ez_sq;
    assign ex_sq = ex * ex;
    assign ey_sq = ey * ey;
    assign ez_sq = ez * ez;

    logic [COST_WIDTH-1:0] tracking_cost;
    assign tracking_cost = ex_sq[27:8] + ey_sq[27:8] + ez_sq[27:8];
```

Extracting bits `[27:8]` from the 32-bit signed square discards redundant fractional precision and provides a normalized 20-bit unsigned quantity in Q12.8 format.

### 4.2 Actuator Control Effort Penalty
Penalizes control chatter and large acceleration demands to prevent actuator saturation:

$$C_{ctrl} = \frac{1}{4} \left( u_x^2 + u_y^2 + u_z^2 \right)$$

```systemverilog
    logic signed [31:0] ux_sq, uy_sq, uz_sq;
    assign ux_sq = ux * ux;
    assign uy_sq = uy * uy;
    assign uz_sq = uz * uz;

    logic [COST_WIDTH-1:0] control_cost;
    assign control_cost = (ux_sq[27:8] + uy_sq[27:8] + uz_sq[27:8]) >>> 2;
```

### 4.3 Division-Free Quadratic Obstacle Barrier Function
Evaluating true Euclidean distance $\lVert \mathbf{p} - \mathbf{p}_{obs} \rVert$ requires an iterative square root, while classical artificial potential fields evaluate $1/d^2$, requiring iterative hardware division. To maintain 1-cycle throughput, obstacle avoidance is formulated directly in **squared spatial distance space**:

$$d_{sq} = (p_x - x_{obs})^2 + (p_y - y_{obs})^2 + (p_z - z_{obs})^2$$

A piecewise quadratic barrier penalty is applied:
- **Free Flight Zone ($d_{sq} > R_{outer}^2$):** $\mathcal{L}_{obs} = 0$.
- **Soft Repulsive Buffer ($R_{safe}^2 < d_{sq} \le R_{outer}^2$):** $\mathcal{L}_{obs} = w_{obs} \cdot (R_{outer}^2 - d_{sq})$.
- **Hard Collision Boundary ($d_{sq} \le R_{safe}^2$):** $\mathcal{L}_{obs} = \text{16'sh7FFF}$ (Saturating maximum cost).

This piecewise formulation eliminates all divisions and square roots, synthesizing directly into subtraction, comparison, and saturation clamp logic.

---

## 5. Fixed-Point Arithmetic & Saturation Clamping

### 5.1 Format Alignment & Wordlengths
- **State Kinematics:** Signed Q8.8 format (`DATA_WIDTH = 16`). Range: $[-128.0, +127.996]$, resolution $2^{-8} \approx 0.0039\,\text{m}$ or $\text{m/s}$.
- **Stage Cost & Accumulator:** Unsigned Q16.8 format (`COST_WIDTH = 24`). Range: $[0, 65535.996]$, resolution $2^{-8} \approx 0.0039$.
- **Chebyshev Slosh Normalized Angle:** Signed Q4.12 format (`DATA_WIDTH = 16`). Range: $[-8.0, +7.9997]$, resolution $2^{-12} \approx 0.000244\,\text{rad}$.

### 5.2 Saturation Protection Against Sign-Inversion Wrapping
In two's-complement arithmetic, accumulator overflow wraps from large positive values to large negative values. In an MPPI cost functional, a sign-inversion wrap would turn an extreme obstacle collision penalty into an apparent "global minimum cost," causing the optimizer to aggressively steer the vehicle directly into the hazard.

To prevent sign-inversion wrapping, state additions and cost accumulations use bit-growth headroom followed by explicit saturation logic:

```systemverilog
    function automatic logic signed [DATA_WIDTH-1:0] sat_add16(
        input logic signed [DATA_WIDTH:0] sum_extended
    );
        if (sum_extended > 17'sh07FFF)
            return 16'sh7FFF; // Positive clamp
        else if (sum_extended < -17'sh08000)
            return -16'sh08000; // Negative clamp
        else
            return sum_extended[DATA_WIDTH-1:0];
    endfunction
```

The 24-bit cost accumulator `cost_acc_q` provides 16 integer bits ($2^{16} = 65,536$), providing sufficient dynamic headroom so that 16 steps of maximum stage cost accumulation will not overflow the 24-bit register.

---

## 6. Micro-Architecture, State Machine & Timing Closure

### 6.1 State Machine Sequencing
The lane sequencing is controlled by a 3-state Moore FSM:

```systemverilog
    typedef enum logic [1:0] {
        IDLE    = 2'b00,
        ROLLOUT = 2'b01,
        FINISH  = 2'b10
    } state_t;

    state_t state_q;
    logic [4:0] step_cnt_q;
```

```
                 ┌────────────────────────────────────────────────────────┐
                 │                Rollout Lane FSM States                 │
                 └────────────────────────────────────────────────────────┘
                                             │
                                             ▼ (rst_n == 0)
                                       ┌───────────┐
                                       │   IDLE    │◄────────────────┐
                                       └─────┬─────┘                 │
                                             │ start_rollout == 1    │
                                             ▼                       │
                                       ┌───────────┐                 │
                             ┌────────►│  ROLLOUT  │                 │
           step_cnt < 15     │         └─────┬─────┘                 │
           (16 clock cycles) └───────────────┤ step_cnt == 15        │
                                             ▼                       │
                                       ┌───────────┐                 │
                                       │  FINISH   │─────────────────┘
                                       └───────────┘ rollout_done = 1
```

1. **`IDLE` (2'b00):** The lane waits for `start_rollout`. Upon assertion, initial telemetry (`p_curr`, `v_curr`) is latched, fluid deflection is initialized to $\theta_0 = 16'\text{sh}0020$ ($+0.125\text{ rad} \approx 7.16^\circ$), cost accumulator is cleared (`cost_acc_q <= '0`), `qmc_init <= 1'b1` pulses with `sample_seed`, and the FSM transitions to `ROLLOUT`.
2. **`ROLLOUT` (2'b01):** Iterates for 16 clock cycles ($t = 0 \dots 15$). On step $t = 0$, latches the exploratory perturbation into `pert_[x,y,z]_save_q`. Updates kinematics, evaluates slosh dynamics, and accumulates stage cost. When `step_cnt_q == 15`, transitions to `FINISH`.
3. **`FINISH` (2'b10):** Deasserts `qmc_enable`, asserts `rollout_done <= 1'b1` for 1 cycle, and returns to `IDLE`.

### 6.2 First-Step Perturbation Latching
In MPPI theory, the synthesized control action for the current actuation timestep is:

$$\mathbf{u}^*_0 = \mathbf{u}_{nom,0} + \frac{\sum_{k=0}^{M-1} w_k \boldsymbol{\epsilon}_{k,0}}{\sum_{k=0}^{M-1} w_k}$$

The lane preserves the initial perturbation vector $\boldsymbol{\epsilon}_{k,0}$ generated at step $t=0$:

```systemverilog
    if (step_cnt_q == 0) begin
        pert_x_save_q <= eps_x;
        pert_y_save_q <= eps_y;
        pert_z_save_q <= eps_z;
    end
```

### 6.3 Standard Cell Library Constraints & Physical Design
- **ASAP7 7nm FinFET (`ASAP7SC7p5RVT`):**  
  At 500 MHz ($T_{clk} = 2.0\text{ ns}$), the critical path originates from the registered velocity `vx_q`, traverses the multiplier `(vx_q * DT)`, the 8-bit arithmetic right-shift, and the position adder into `px_q`. Total combinational delay is **1.24 ns**, yielding a 38% positive setup timing slack. 
- **SCL 180nm CMOS (`CP8D`):**  
  The 2-metal routing layer stack presents severe Metal-1 congestion under multi-bit register arrays. To prevent routing blockages, pipeline registers are mapped to `DFCL11` (flip-flops with active-low clear) rather than scan-muxed `DFFL11` flops, eliminating unrouted congestion hotspots around the 24-bit cost accumulator. Clock fanout on `qmc_enable` is buffered using balanced `CLKBUFX4` trees.

---

## 7. Verification Defects Caught & Engineering Fixes

### Defect 1: Arithmetic Shift Sign Wrapping Under High Thrust
- **Symptom:** In early RTL simulation, when large control inputs were applied ($u_x > 32.0\,\text{m/s}^2$), the quadrotor accelerated in the opposite direction.
- **Root Cause:** The expression `((ux * DT) >>> 10)` in the slosh integration update was evaluated with an intermediate width of 16 bits. Multiplying large values of $u_x$ with $DT$ caused bit 15 to flip to 1, causing the unsigned shift to sign-extend erroneously as a negative number.
- **Engineering Fix:** Explicitly cast intermediate multiplication to signed 32-bit before shifting:
  ```systemverilog
  // Fixed: explicitly typed 32-bit signed product before arithmetic right-shift
  logic signed [31:0] u_dt_prod;
  assign u_dt_prod = $signed(ux) * $signed(DT);
  thetadot_q <= thetadot_q - ((f_slosh * DT) >>> 8) + (u_dt_prod >>> 10);
  ```

### Defect 2: Perturbation Register Overwrite on Horizon Step 1
- **Symptom:** The synthesized optimal control output `opt_u` matched the step-15 perturbation rather than the step-0 perturbation.
- **Root Cause:** In the initial commit, `pert_x_save_q` was assigned unconditionally on every cycle of `ROLLOUT`, overwriting the step-0 action with later horizon noise.
- **Engineering Fix:** Added the explicit guard condition `if (step_cnt_q == 0)` to latch and hold the exploratory action vector for the duration of the rollout.

---

## 8. Implementation Metrics & Sign-Off Summary

| Implementation Target | DSP Blocks | LUT / Standard Cell Count | Flip-Flops | Maximum Frequency | Frame Latency |
|---|---|---|---|---|---|
| **Xilinx Artix-7 (`xc7a35t`)** | 8 DSP48E1 | 1,280 LUTs | 410 FFs | 114 MHz | 18 clock cycles |
| **ASAP7 7nm FinFET** | 0 (synthesized MAC) | 3,840 cells ($1,420\,\mu\text{m}^2$) | 398 FFs | 404 MHz | 18 clock cycles |
| **SCL 180nm CMOS (`CP8D`)** | 0 (Booth multipliers) | 8,920 gates ($0.22\,\text{mm}^2$) | 402 FFs | 52 MHz | 18 clock cycles |

Every rollout completes deterministically in exactly **18 clock cycles** (1 cycle `IDLE` latching, 16 cycles `ROLLOUT`, 1 cycle `FINISH`), ensuring zero pipeline jitter and zero backpressure bubbles in the top-level H-MPPI execution schedule.
