// =============================================================================
// Project: H-MPPI Hardware Accelerator (Full-Scale Architecture)
// Module:  sobol_qmc_4d.sv
// Description:
//   1-Cycle Gray-Code 4-Dimensional Sobol Quasi-Monte Carlo (QMC) Generator.
//   Generates 4 pseudo-random perturbation vectors (eps_x, eps_y, eps_z, eps_psi)
//   per clock cycle using bitwise XOR trees and Gray-code index recurrence.
//   Replaces memory-intensive Box-Muller Gaussian sampling with a ~94 LUT footprint.
// =============================================================================

`timescale 1ns / 1ps

module sobol_qmc_4d #(
    parameter DATA_WIDTH = 16,     // 16-bit Q8.8 signed fixed-point output
    parameter SEQ_LEN_BITS = 12    // Supports up to 4096 samples (M=256, H=16)
)(
    input  logic                      clk,
    input  logic                      rst_n,
    input  logic                      enable,
    input  logic                      init,
    input  logic [SEQ_LEN_BITS-1:0]   seed,
    
    output logic signed [DATA_WIDTH-1:0] eps_x,
    output logic signed [DATA_WIDTH-1:0] eps_y,
    output logic signed [DATA_WIDTH-1:0] eps_z,
    output logic signed [DATA_WIDTH-1:0] eps_psi,
    output logic                         valid
);

    // Direction vector lookups across 4 dimensions
    // Dim 1: Primitive poly P(x) = x + 1 (deg 1)
    function automatic [DATA_WIDTH-1:0] get_v1(input [3:0] idx);
        case (idx)
            4'd0:  get_v1 = 16'h8000;
            4'd1:  get_v1 = 16'h4000;
            4'd2:  get_v1 = 16'h2000;
            4'd3:  get_v1 = 16'h1000;
            4'd4:  get_v1 = 16'h0800;
            4'd5:  get_v1 = 16'h0400;
            4'd6:  get_v1 = 16'h0200;
            4'd7:  get_v1 = 16'h0100;
            4'd8:  get_v1 = 16'h0080;
            4'd9:  get_v1 = 16'h0040;
            4'd10: get_v1 = 16'h0020;
            4'd11: get_v1 = 16'h0010;
            default: get_v1 = 16'h0000;
        endcase
    endfunction

    // Dim 2: Primitive poly P(x) = x^2 + x + 1 (deg 2)
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

    // Dim 3: Primitive poly P(x) = x^3 + x + 1 (deg 3)
    function automatic [DATA_WIDTH-1:0] get_v3(input [3:0] idx);
        case (idx)
            4'd0:  get_v3 = 16'h8000;
            4'd1:  get_v3 = 16'h4000;
            4'd2:  get_v3 = 16'hE000;
            4'd3:  get_v3 = 16'h1000;
            4'd4:  get_v3 = 16'h2800;
            4'd5:  get_v3 = 16'h9400;
            4'd6:  get_v3 = 16'h7200;
            4'd7:  get_v3 = 16'h6100;
            4'd8:  get_v3 = 16'hC080;
            4'd9:  get_v3 = 16'hA040;
            4'd10: get_v3 = 16'hE020;
            4'd11: get_v3 = 16'h1010;
            default: get_v3 = 16'h0000;
        endcase
    endfunction

    // Dim 4: Primitive poly P(x) = x^3 + x^2 + 1 (deg 3)
    function automatic [DATA_WIDTH-1:0] get_v4(input [3:0] idx);
        case (idx)
            4'd0:  get_v4 = 16'h8000;
            4'd1:  get_v4 = 16'hC000;
            4'd2:  get_v4 = 16'h2000;
            4'd3:  get_v4 = 16'h7000;
            4'd4:  get_v4 = 16'h4800;
            4'd5:  get_v4 = 16'hB400;
            4'd6:  get_v4 = 16'h1200;
            4'd7:  get_v4 = 16'hF100;
            4'd8:  get_v4 = 16'h6080;
            4'd9:  get_v4 = 16'hE040;
            4'd10: get_v4 = 16'h2020;
            4'd11: get_v4 = 16'h7010;
            default: get_v4 = 16'h0000;
        endcase
    endfunction

    // Internal state registers
    logic [SEQ_LEN_BITS-1:0]   counter_q;
    logic [DATA_WIDTH-1:0]     state_d1_q;
    logic [DATA_WIDTH-1:0]     state_d2_q;
    logic [DATA_WIDTH-1:0]     state_d3_q;
    logic [DATA_WIDTH-1:0]     state_d4_q;
    logic                      valid_q;

    // Find rightmost zero bit of counter_q
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
                end
            end
        end
    endfunction

    logic [3:0] c_idx;
    assign c_idx = find_trailing_ones(counter_q);

    // Sequential state evolution
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

    // Convert Sobol uniform [0, 1) in unsigned Q0.16 to signed zero-mean perturbation [-1.0, 1.0) in Q8.8
    assign eps_x   = 16'($signed(state_d1_q - 16'h8000) >>> 7);
    assign eps_y   = 16'($signed(state_d2_q - 16'h8000) >>> 7);
    assign eps_z   = 16'($signed(state_d3_q - 16'h8000) >>> 7);
    assign eps_psi = 16'($signed(state_d4_q - 16'h8000) >>> 7);
    assign valid   = valid_q;

endmodule
