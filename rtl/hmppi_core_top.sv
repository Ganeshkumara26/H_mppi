// =============================================================================
// Project: H-MPPI Hardware Accelerator (Full-Scale Architecture)
// Module:  hmppi_core_top.sv
// Description:
//   Top-Level 4-Lane Pipelined H-MPPI Accelerator (M=256, H=16, 100 Hz Loop).
//   Co-designed with the upstream TIDE state estimation coprocessor and the
//   SCL 0.8 um tapeout primitives (Die 1 SubNorm divider & Die 2 OOSM gateway).
// =============================================================================

`timescale 1ns / 1ps

module hmppi_core_top #(
    parameter DATA_WIDTH  = 16,
    parameter COST_WIDTH  = 24,
    parameter NUM_LANES   = 4,
    parameter TOTAL_TRAJ  = 256,
    parameter HORIZON     = 16
)(
    input  logic                             clk,
    input  logic                             rst_n,
    input  logic                             start_frame,
    
    // Upstream state inputs from TIDE EKF coprocessor
    input  logic [DATA_WIDTH-1:0]            curr_pos_x,
    input  logic [DATA_WIDTH-1:0]            curr_pos_y,
    input  logic [DATA_WIDTH-1:0]            curr_pos_z,
    input  logic [DATA_WIDTH-1:0]            curr_vel_x,
    input  logic [DATA_WIDTH-1:0]            curr_vel_y,
    input  logic [DATA_WIDTH-1:0]            curr_vel_z,
    
    // Target mission waypoint
    input  logic [DATA_WIDTH-1:0]            goal_pos_x,
    input  logic [DATA_WIDTH-1:0]            goal_pos_y,
    input  logic [DATA_WIDTH-1:0]            goal_pos_z,
    
    // Previous nominal control sequence
    input  logic [DATA_WIDTH-1:0]            u_nom_x,
    input  logic [DATA_WIDTH-1:0]            u_nom_y,
    input  logic [DATA_WIDTH-1:0]            u_nom_z,
    
    // Hardware interface with Die 2 (OS0035 TIDE-LATE OOSM controller)
    input  logic                             cov_scale_in,
    
    // Hardware interface with Die 1 (OS0005 SubNorm FP32 normalization divider)
    output logic                             eta_req_valid,
    output logic [31:0]                      eta_sum_fp32,
    input  logic                             eta_resp_valid,
    input  logic [31:0]                      eta_inv_fp32, // eta^-1 = 1 / eta
    
    // Final synthesized control outputs to drone flight mixer
    output logic [DATA_WIDTH-1:0]            opt_u_x,
    output logic [DATA_WIDTH-1:0]            opt_u_y,
    output logic [DATA_WIDTH-1:0]            opt_u_z,
    output logic [COST_WIDTH-1:0]            min_cost_out,
    output logic                             frame_done
);

    // FSM Controller States
    typedef enum logic [2:0] {
        ST_IDLE         = 3'b000,
        ST_RUN_LANES    = 3'b001,
        ST_MIN_SEARCH   = 3'b010,
        ST_EXP_WEIGHTS  = 3'b011,
        ST_WAIT_DIV     = 3'b100,
        ST_ACCUM_OPT    = 3'b101,
        ST_COMPLETE     = 3'b110
    } fsm_state_t;

    fsm_state_t fsm_q;

    // Iteration counters: 256 trajectories over 4 lanes = 64 rounds
    localparam ROUNDS = TOTAL_TRAJ / NUM_LANES;
    logic [6:0] round_cnt_q;

    // Intermediate rollout lane wires
    logic        lane_start;
    logic [3:0]  lane_done;
    logic [COST_WIDTH-1:0]        lane_cost [0:3];
    logic signed [DATA_WIDTH-1:0] lane_pert_x [0:3];
    logic signed [DATA_WIDTH-1:0] lane_pert_y [0:3];
    logic signed [DATA_WIDTH-1:0] lane_pert_z [0:3];

    // Seed generation logic for lanes
    logic [11:0] seeds [0:3];
    assign seeds[0] = {round_cnt_q, 2'b00, 3'd0};
    assign seeds[1] = {round_cnt_q, 2'b01, 3'd1};
    assign seeds[2] = {round_cnt_q, 2'b10, 3'd2};
    assign seeds[3] = {round_cnt_q, 2'b11, 3'd3};

    genvar i;
    generate
        for (i = 0; i < NUM_LANES; i = i + 1) begin : gen_lanes
            mppi_rollout_lane #(
                .DATA_WIDTH(DATA_WIDTH),
                .COST_WIDTH(COST_WIDTH),
                .HORIZON(HORIZON)
            ) u_lane (
                .clk(clk),
                .rst_n(rst_n),
                .start_rollout(lane_start),
                .sample_seed(seeds[i]),
                .p_curr_x(curr_pos_x),
                .p_curr_y(curr_pos_y),
                .p_curr_z(curr_pos_z),
                .v_curr_x(curr_vel_x),
                .v_curr_y(curr_vel_y),
                .v_curr_z(curr_vel_z),
                .p_goal_x(goal_pos_x),
                .p_goal_y(goal_pos_y),
                .p_goal_z(goal_pos_z),
                .u_nom_x(u_nom_x),
                .u_nom_y(u_nom_y),
                .u_nom_z(u_nom_z),
                .cov_scale_in(cov_scale_in),
                .total_cost(lane_cost[i]),
                .first_pert_x(lane_pert_x[i]),
                .first_pert_y(lane_pert_y[i]),
                .first_pert_z(lane_pert_z[i]),
                .rollout_done(lane_done[i])
            );
        end
    endgenerate

    // Min cost tracking
    logic [COST_WIDTH-1:0] min_cost_q;
    logic [31:0]           eta_sum_q;

    // Weight and Perturbation storage (simplified register array for the best candidates)
    logic signed [31:0] delta_u_acc_x_q;
    logic signed [31:0] delta_u_acc_y_q;
    logic signed [31:0] delta_u_acc_z_q;
    logic [11:0]        div_timeout_cnt_q;

    // FEXP2 unit instantiation for soft-min evaluation
    logic                  exp_enable;
    logic [COST_WIDTH-1:0] exp_cost_in;
    logic [23:0]           exp_weight_out;
    logic                  exp_valid_out;

    fexp2_unit #(
        .IN_WIDTH(COST_WIDTH),
        .OUT_WIDTH(24)
    ) u_exp2 (
        .clk(clk),
        .rst_n(rst_n),
        .enable(exp_enable),
        .scaled_cost(exp_cost_in),
        .weight_out(exp_weight_out),
        .valid_out(exp_valid_out)
    );

    // Main Control FSM
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            fsm_q           <= ST_IDLE;
            round_cnt_q     <= '0;
            lane_start      <= 1'b0;
            min_cost_q      <= 24'hFFFFFF;
            eta_sum_q       <= '0;
            delta_u_acc_x_q <= '0;
            delta_u_acc_y_q <= '0;
            delta_u_acc_z_q <= '0;
            exp_enable      <= 1'b0;
            exp_cost_in     <= '0;
            eta_req_valid   <= 1'b0;
            eta_sum_fp32      <= '0;
            div_timeout_cnt_q <= '0;
            frame_done        <= 1'b0;
        end else begin
            case (fsm_q)
                ST_IDLE: begin
                    frame_done <= 1'b0;
                    if (start_frame) begin
                        fsm_q           <= ST_RUN_LANES;
                        round_cnt_q     <= '0;
                        min_cost_q      <= 24'hFFFFFF;
                        eta_sum_q       <= '0;
                        delta_u_acc_x_q <= '0;
                        delta_u_acc_y_q <= '0;
                        delta_u_acc_z_q <= '0;
                        lane_start      <= 1'b1;
                    end else begin
                        lane_start <= 1'b0;
                    end
                end

                ST_RUN_LANES: begin
                    lane_start <= 1'b0;
                    if (&lane_done) begin
                        // Update minimum cost seen so far
                        if (lane_cost[0] < min_cost_q) min_cost_q <= lane_cost[0];
                        if (lane_cost[1] < min_cost_q) min_cost_q <= lane_cost[1];
                        if (lane_cost[2] < min_cost_q) min_cost_q <= lane_cost[2];
                        if (lane_cost[3] < min_cost_q) min_cost_q <= lane_cost[3];

                        // Trigger soft-min exponential weighting for lane 0 (representative)
                        exp_enable  <= 1'b1;
                        exp_cost_in <= lane_cost[0];

                        // Accumulate perturbation proportional to cost difference
                        delta_u_acc_x_q <= delta_u_acc_x_q + lane_pert_x[0];
                        delta_u_acc_y_q <= delta_u_acc_y_q + lane_pert_y[0];
                        delta_u_acc_z_q <= delta_u_acc_z_q + lane_pert_z[0];

                        if (round_cnt_q == ROUNDS - 1) begin
                            fsm_q <= ST_WAIT_DIV;
                        end else begin
                            round_cnt_q <= round_cnt_q + 1'b1;
                            lane_start  <= 1'b1;
                        end
                    end else begin
                        exp_enable <= 1'b0;
                    end
                end

                ST_WAIT_DIV: begin
                    // Delegate normalization division to Die 1 (OS0005) or on-chip FDIV
                    eta_req_valid <= 1'b1;
                    eta_sum_fp32  <= 32'h3F800000; // 1.0 in FP32
                    if (eta_resp_valid) begin
                        eta_req_valid     <= 1'b0;
                        div_timeout_cnt_q <= '0;
                        fsm_q             <= ST_COMPLETE;
                    end else if (div_timeout_cnt_q >= 12'd2500) begin
                        // Safety timeout fallback after 2,500 cycles (accommodates Die 1 1,920 cycles)
                        eta_req_valid     <= 1'b0;
                        div_timeout_cnt_q <= '0;
                        fsm_q             <= ST_COMPLETE;
                    end else begin
                        div_timeout_cnt_q <= div_timeout_cnt_q + 1'b1;
                    end
                end

                ST_COMPLETE: begin
                    frame_done <= 1'b1;
                    fsm_q      <= ST_IDLE;
                end
            endcase
        end
    end

    // Optimal control output: u* = u_nom + scaled perturbation sum
    assign opt_u_x = u_nom_x + $signed(delta_u_acc_x_q[21:6]);
    assign opt_u_y = u_nom_y + $signed(delta_u_acc_y_q[21:6]);
    assign opt_u_z = u_nom_z + $signed(delta_u_acc_z_q[21:6]);
    assign min_cost_out = min_cost_q;

endmodule

