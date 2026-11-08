// =============================================================================
// Project: H-MPPI Hardware Accelerator (Full-Scale Architecture)
// Module:  mppi_rollout_lane.sv
// Description:
//   Pipelined Trajectory Rollout Processing Lane.
//   Executes H=16 rollout steps per sample, evaluating quadrotor kinematics,
//   dynamic obstacle distance penalty, and Chebyshev fluid slosh dynamics.
// =============================================================================

`timescale 1ns / 1ps

module mppi_rollout_lane #(
    parameter DATA_WIDTH = 16,    // Signed Q8.8
    parameter COST_WIDTH = 24,    // Unsigned Q16.8
    parameter HORIZON    = 16
)(
    input  logic                             clk,
    input  logic                             rst_n,
    input  logic                             start_rollout,
    input  logic [11:0]                      sample_seed,
    
    // Initial vehicle state (from TIDE EKF coprocessor)
    input  logic signed [DATA_WIDTH-1:0]     p_curr_x,
    input  logic signed [DATA_WIDTH-1:0]     p_curr_y,
    input  logic signed [DATA_WIDTH-1:0]     p_curr_z,
    input  logic signed [DATA_WIDTH-1:0]     v_curr_x,
    input  logic signed [DATA_WIDTH-1:0]     v_curr_y,
    input  logic signed [DATA_WIDTH-1:0]     v_curr_z,
    
    // Target goal waypoint
    input  logic signed [DATA_WIDTH-1:0]     p_goal_x,
    input  logic signed [DATA_WIDTH-1:0]     p_goal_y,
    input  logic signed [DATA_WIDTH-1:0]     p_goal_z,
    
    // Nominal baseline control input
    input  logic signed [DATA_WIDTH-1:0]     u_nom_x,
    input  logic signed [DATA_WIDTH-1:0]     u_nom_y,
    input  logic signed [DATA_WIDTH-1:0]     u_nom_z,
    
    // Covariance scaling signal from Die 2 (OS0035 OOSM controller)
    input  logic                             cov_scale_in,
    
    // Rollout output
    output logic [COST_WIDTH-1:0]            total_cost,
    output logic signed [DATA_WIDTH-1:0]     first_pert_x,
    output logic signed [DATA_WIDTH-1:0]     first_pert_y,
    output logic signed [DATA_WIDTH-1:0]     first_pert_z,
    output logic                             rollout_done
);

    // FSM States
    typedef enum logic [1:0] {
        IDLE    = 2'b00,
        ROLLOUT = 2'b01,
        FINISH  = 2'b10
    } state_t;

    state_t state_q;
    logic [4:0] step_cnt_q;

    // State registers for simulated kinematics
    logic signed [DATA_WIDTH-1:0] px_q, py_q, pz_q;
    logic signed [DATA_WIDTH-1:0] vx_q, vy_q, vz_q;
    logic signed [DATA_WIDTH-1:0] theta_q, thetadot_q;
    logic [COST_WIDTH-1:0]        cost_acc_q;

    logic signed [DATA_WIDTH-1:0] pert_x_save_q;
    logic signed [DATA_WIDTH-1:0] pert_y_save_q;
    logic signed [DATA_WIDTH-1:0] pert_z_save_q;

    // Sobol QMC instantiation for this lane
    logic qmc_enable, qmc_init;
    logic signed [DATA_WIDTH-1:0] eps_x, eps_y, eps_z, eps_psi;
    logic qmc_valid;

    sobol_qmc_4d #(
        .DATA_WIDTH(DATA_WIDTH),
        .SEQ_LEN_BITS(12)
    ) u_qmc (
        .clk(clk),
        .rst_n(rst_n),
        .enable(qmc_enable),
        .init(qmc_init),
        .seed(sample_seed),
        .eps_x(eps_x),
        .eps_y(eps_y),
        .eps_z(eps_z),
        .eps_psi(eps_psi),
        .valid(qmc_valid)
    );

    // Chebyshev Slosh Evaluator instantiation
    logic signed [DATA_WIDTH-1:0] f_slosh;
    logic [COST_WIDTH-1:0]        slosh_cost;
    logic                         slosh_valid;

    chebyshev_slosh_eval #(
        .DATA_WIDTH(DATA_WIDTH),
        .COST_WIDTH(COST_WIDTH)
    ) u_slosh (
        .clk(clk),
        .rst_n(rst_n),
        .enable(state_q == ROLLOUT),
        .theta_norm(theta_q),
        .theta_dot(thetadot_q),
        .cov_scale_in(cov_scale_in),
        .f_slosh(f_slosh),
        .slosh_cost(slosh_cost),
        .valid_out(slosh_valid)
    );

    // Step cost calculations
    // Tracking error e = p - p_goal
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

    // Control effort cost ||u_nom + eps||^2
    logic signed [DATA_WIDTH-1:0] ux, uy, uz;
    assign ux = u_nom_x + (eps_x >>> 2);
    assign uy = u_nom_y + (eps_y >>> 2);
    assign uz = u_nom_z + (eps_z >>> 2);

    logic signed [31:0] ux_sq, uy_sq, uz_sq;
    assign ux_sq = ux * ux;
    assign uy_sq = uy * uy;
    assign uz_sq = uz * uz;

    logic [COST_WIDTH-1:0] control_cost;
    assign control_cost = (ux_sq[27:8] + uy_sq[27:8] + uz_sq[27:8]) >>> 2;

    // Time step dt = 0.05s (~13 in Q8.8)
    localparam logic signed [DATA_WIDTH-1:0] DT = 16'sh000D;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state_q         <= IDLE;
            step_cnt_q      <= '0;
            px_q            <= '0;
            py_q            <= '0;
            pz_q            <= '0;
            vx_q            <= '0;
            vy_q            <= '0;
            vz_q            <= '0;
            theta_q         <= '0;
            thetadot_q      <= '0;
            cost_acc_q      <= '0;
            pert_x_save_q   <= '0;
            pert_y_save_q   <= '0;
            pert_z_save_q   <= '0;
            qmc_enable      <= 1'b0;
            qmc_init        <= 1'b0;
            rollout_done    <= 1'b0;
        end else begin
            case (state_q)
                IDLE: begin
                    rollout_done <= 1'b0;
                    if (start_rollout) begin
                        state_q       <= ROLLOUT;
                        step_cnt_q    <= '0;
                        px_q          <= p_curr_x;
                        py_q          <= p_curr_y;
                        pz_q          <= p_curr_z;
                        vx_q          <= v_curr_x;
                        vy_q          <= v_curr_y;
                        vz_q          <= v_curr_z;
                        theta_q       <= 16'sh0020; // Initial small slosh angle
                        thetadot_q    <= 16'sh0000;
                        cost_acc_q    <= '0;
                        qmc_init      <= 1'b1;
                        qmc_enable    <= 1'b1;
                    end else begin
                        qmc_init      <= 1'b0;
                        qmc_enable    <= 1'b0;
                    end
                end

                ROLLOUT: begin
                    qmc_init   <= 1'b0;
                    qmc_enable <= 1'b1;

                    if (step_cnt_q == 0) begin
                        pert_x_save_q <= eps_x;
                        pert_y_save_q <= eps_y;
                        pert_z_save_q <= eps_z;
                    end

                    // Accumulate tracking cost + control effort + slosh cost
                    cost_acc_q <= cost_acc_q + tracking_cost + control_cost + slosh_cost;

                    // Kinematics integration step
                    // p = p + v * dt
                    px_q <= px_q + ((vx_q * DT) >>> 8);
                    py_q <= py_q + ((vy_q * DT) >>> 8);
                    pz_q <= pz_q + ((vz_q * DT) >>> 8);

                    // v = v + u * dt
                    vx_q <= vx_q + ((ux * DT) >>> 8);
                    vy_q <= vy_q + ((uy * DT) >>> 8);
                    vz_q <= vz_q + ((uz * DT) >>> 8);

                    // Slosh dynamics integration step
                    // theta_dot = theta_dot - (g/L)*f_slosh*dt + accel_coupling
                    thetadot_q <= thetadot_q - ((f_slosh * DT) >>> 8) + ((ux * DT) >>> 10);
                    theta_q    <= theta_q + ((thetadot_q * DT) >>> 8);

                    if (step_cnt_q == HORIZON - 1) begin
                        state_q <= FINISH;
                    end else begin
                        step_cnt_q <= step_cnt_q + 1'b1;
                    end
                end

                FINISH: begin
                    qmc_enable   <= 1'b0;
                    rollout_done <= 1'b1;
                    state_q      <= IDLE;
                end
            endcase
        end
    end

    assign total_cost   = cost_acc_q;
    assign first_pert_x = pert_x_save_q;
    assign first_pert_y = pert_y_save_q;
    assign first_pert_z = pert_z_save_q;

endmodule

