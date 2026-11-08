// =============================================================================
// Project: H-MPPI Hardware Accelerator (Full-Scale Architecture)
// Module:  chebyshev_slosh_eval.sv
// Description:
//   Division-Free Degree-3 Chebyshev Polynomial Evaluator for Spherical Pendulum
//   Fluid Slosh Dynamics in Agile Aerial Systems.
//   Evaluates slosh angle deflection theta, angular rate theta_dot, and calculates
//   the nonlinear restoration force and penalty cost without CORDIC or dividers.
//   Formula: T_0(x)=1, T_1(x)=x, T_2(x)=2x^2-1, T_3(x)=2x*T_2(x)-T_1(x)
//   F_slosh = c1*T_1(x) + c2*T_2(x) + c3*T_3(x)
// =============================================================================

`timescale 1ns / 1ps

module chebyshev_slosh_eval #(
    parameter DATA_WIDTH = 16,    // Signed Q8.8 fixed-point format
    parameter COST_WIDTH = 24     // Extended precision for cost accumulation
)(
    input  logic                             clk,
    input  logic                             rst_n,
    input  logic                             enable,
    
    // Slosh state inputs (deflection angle theta normalized to [-1.0, 1.0], and angular rate)
    input  logic signed [DATA_WIDTH-1:0]     theta_norm,      // x in [-1, 1]
    input  logic signed [DATA_WIDTH-1:0]     theta_dot,       // Angular rate in rad/s
    input  logic                             cov_scale_in,    // From Die 2 OOSM uncertainty gateway!
    
    // Outputs
    output logic signed [DATA_WIDTH-1:0]     f_slosh,         // Restoring force
    output logic [COST_WIDTH-1:0]            slosh_cost,      // Trajectory stage slosh penalty
    output logic                             valid_out
);

    // Chebyshev expansion coefficients in Q8.8 for nonlinear fluid pendulum restoration
    // c1 = 1.0 (256 in Q8.8), c2 = -0.1667 (-43 in Q8.8), c3 = 0.05 (13 in Q8.8)
    localparam logic signed [DATA_WIDTH-1:0] C1_COEFF = 16'sh0100; // +1.000 in Q8.8
    localparam logic signed [DATA_WIDTH-1:0] C2_COEFF = -16'sh002B; // -0.168 in Q8.8
    localparam logic signed [DATA_WIDTH-1:0] C3_COEFF = 16'sh000D; // +0.051 in Q8.8

    // Damping penalty weighting
    localparam logic signed [DATA_WIDTH-1:0] RATE_WEIGHT = 16'sh0080; // 0.5 in Q8.8

    // Pipeline Stage 1: Compute T_1 and T_2
    // x = theta_norm
    // T_1 = x
    // x^2 = (x * x) >>> 8
    // T_2 = 2*x^2 - 1 = (x^2 <<< 1) - 1.0
    logic signed [DATA_WIDTH-1:0] x_s1;
    logic signed [DATA_WIDTH-1:0] x_dot_s1;
    logic signed [31:0]           x_sq_full;
    logic signed [DATA_WIDTH-1:0] x_sq_s1;
    logic signed [DATA_WIDTH-1:0] t1_s1;
    logic signed [DATA_WIDTH-1:0] t2_s1;
    logic                         cov_scale_s1;
    logic                         valid_s1;

    assign x_sq_full = theta_norm * theta_norm;
    assign x_sq_s1   = x_sq_full[23:8]; // Truncate from Q16.16 to Q8.8

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            x_s1         <= '0;
            x_dot_s1     <= '0;
            t1_s1        <= '0;
            t2_s1        <= '0;
            cov_scale_s1 <= 1'b0;
            valid_s1     <= 1'b0;
        end else if (enable) begin
            x_s1         <= theta_norm;
            x_dot_s1     <= theta_dot;
            t1_s1        <= theta_norm;
            // T_2 = 2*x^2 - 1.0 (1.0 = 16'sh0100)
            t2_s1        <= (x_sq_s1 <<< 1) - 16'sh0100;
            cov_scale_s1 <= cov_scale_in;
            valid_s1     <= 1'b1;
        end else begin
            valid_s1     <= 1'b0;
        end
    end

    // Pipeline Stage 2: Compute T_3 and Assemble Force
    // T_3 = 2*x*T_2 - T_1
    // F_slosh = c1*T_1 + c2*T_2 + c3*T_3
    logic signed [31:0]           x_t2_full;
    logic signed [DATA_WIDTH-1:0] x_t2;
    logic signed [DATA_WIDTH-1:0] t3_s2;
    logic signed [31:0]           term1, term2, term3;
    logic signed [DATA_WIDTH-1:0] f_force_s2;
    logic signed [DATA_WIDTH-1:0] x_dot_s2;
    logic                         cov_scale_s2;
    logic                         valid_s2;

    assign x_t2_full = x_s1 * t2_s1;
    assign x_t2      = x_t2_full[23:8];
    assign t3_s2     = (x_t2 <<< 1) - t1_s1;

    assign term1 = C1_COEFF * t1_s1;
    assign term2 = C2_COEFF * t2_s1;
    assign term3 = C3_COEFF * t3_s2;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            f_force_s2   <= '0;
            x_dot_s2     <= '0;
            cov_scale_s2 <= 1'b0;
            valid_s2     <= 1'b0;
        end else if (valid_s1) begin
            f_force_s2   <= (term1[23:8] + term2[23:8] + term3[23:8]);
            x_dot_s2     <= x_dot_s1;
            cov_scale_s2 <= cov_scale_s1;
            valid_s2     <= 1'b1;
        end else begin
            valid_s2     <= 1'b0;
        end
    end

    // Pipeline Stage 3: Compute Quadratic Slosh Cost
    // Cost = (F_slosh^2) + RATE_WEIGHT * (x_dot^2)
    // If cov_scale_in is asserted (from Die 2 OOSM uncertainty gateway),
    // we boost the slosh penalty by 3x (damps MPPI aggressive control authority by >60%)!
    logic signed [31:0]    f_sq_full;
    logic signed [31:0]    rate_sq_full;
    logic [COST_WIDTH-1:0] base_cost;
    logic [COST_WIDTH-1:0] final_cost_q;
    logic signed [DATA_WIDTH-1:0] f_out_q;
    logic                  valid_out_q;

    assign f_sq_full    = f_force_s2 * f_force_s2;
    assign rate_sq_full = x_dot_s2 * x_dot_s2;
    assign base_cost    = f_sq_full[31:8] + ((rate_sq_full[31:8] * RATE_WEIGHT) >>> 8);

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            f_out_q      <= '0;
            final_cost_q <= '0;
            valid_out_q  <= 1'b0;
        end else if (valid_s2) begin
            f_out_q      <= f_force_s2;
            if (cov_scale_s2) begin
                // 3x penalty scaling when sensor uncertainty is elevated
                final_cost_q <= (base_cost <<< 1) + base_cost;
            end else begin
                final_cost_q <= base_cost;
            end
            valid_out_q  <= 1'b1;
        end else begin
            valid_out_q  <= 1'b0;
        end
    end

    assign f_slosh    = f_out_q;
    assign slosh_cost = final_cost_q;
    assign valid_out  = valid_out_q;

endmodule

