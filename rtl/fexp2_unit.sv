// =============================================================================
// Project: H-MPPI Hardware Accelerator (Full-Scale Architecture)
// Module:  fexp2_unit.sv
// Description:
//   Pipelined 1-Cycle Base-2 Exponential Unit for MPPI Trajectory Weighting.
//   Evaluates w = 2^(-x) = 2^(-k) * 2^(-f) where:
//     x = S * (log2(e) / lambda) >= 0
//     k = integer part, f = fractional part in [0, 1)
//   Uses 64-entry mantissa lookup table and bit-shift in ~140 LUT footprint,
//   replacing iterative Taylor series with single-cycle deterministic execution.
// =============================================================================

`timescale 1ns / 1ps

module fexp2_unit #(
    parameter IN_WIDTH  = 24,    // Input scaled cost in unsigned Q16.8 format
    parameter OUT_WIDTH = 24     // Output normalized weight in unsigned Q0.24 format
)(
    input  logic                   clk,
    input  logic                   rst_n,
    input  logic                   enable,
    input  logic [IN_WIDTH-1:0]    scaled_cost, // x = (S - S_min) * (log2(e) / lambda)
    
    output logic [OUT_WIDTH-1:0]   weight_out,  // w in (0, 1.0]
    output logic                   valid_out
);

    // Decomposition of input x into integer k (upper bits) and fractional f (lower bits)
    // Here scaled_cost is in Q16.8:
    // Bits [23:8] represent integer part k (0 .. 65535)
    // Bits [7:2] (top 6 bits of fraction) address a 64-entry lookup table for 2^(-f)
    logic [15:0] k_int;
    logic [5:0]  f_idx;
    logic [1:0]  f_rem;

    assign k_int = scaled_cost[23:8];
    assign f_idx = scaled_cost[7:2];
    assign f_rem = scaled_cost[1:0];

    // 64-entry ROM for 2^(-f) where f = idx / 64 for idx in [0, 63]
    // Values stored in 16-bit unsigned Q0.16 format (1.0 = 16'hFFFF or ~65536)
    // At f = 0, 2^0 = 1.0 (16'hFFFF)
    // At f = 0.5, 2^(-0.5) = 0.7071 (16'hB504)
    // At f = 63/64 = 0.984, 2^(-0.984) = 0.505 (16'h8160)
    logic [15:0] lut_val;

    function automatic [15:0] exp2_lut(input [5:0] idx);
        case (idx)
            6'd0:  exp2_lut = 16'hFFFF;
            6'd1:  exp2_lut = 16'hFC5F;
            6'd2:  exp2_lut = 16'hF8D1;
            6'd3:  exp2_lut = 16'hF554;
            6'd4:  exp2_lut = 16'hF1E9;
            6'd5:  exp2_lut = 16'hEE8E;
            6'd6:  exp2_lut = 16'hEB43;
            6'd7:  exp2_lut = 16'hE808;
            6'd8:  exp2_lut = 16'hE4DD;
            6'd9:  exp2_lut = 16'hE1C1;
            6'd10: exp2_lut = 16'hDEB5;
            6'd11: exp2_lut = 16'hDBB8;
            6'd12: exp2_lut = 16'hD8CA;
            6'd13: exp2_lut = 16'hD5EB;
            6'd14: exp2_lut = 16'hD31A;
            6'd15: exp2_lut = 16'hD058;
            6'd16: exp2_lut = 16'hCDA4;
            6'd17: exp2_lut = 16'hCAFF;
            6'd18: exp2_lut = 16'hC867;
            6'd19: exp2_lut = 16'hC5DD;
            6'd20: exp2_lut = 16'hC361;
            6'd21: exp2_lut = 16'hC0F2;
            6'd22: exp2_lut = 16'hBE91;
            6'd23: exp2_lut = 16'hBC3D;
            6'd24: exp2_lut = 16'hB9F6;
            6'd25: exp2_lut = 16'hB7BC;
            6'd26: exp2_lut = 16'hB58F;
            6'd27: exp2_lut = 16'hB36E;
            6'd28: exp2_lut = 16'hB15A;
            6'd29: exp2_lut = 16'hAF52;
            6'd30: exp2_lut = 16'hAD56;
            6'd31: exp2_lut = 16'hAB66;
            6'd32: exp2_lut = 16'hA982;
            6'd33: exp2_lut = 16'hA7A9;
            6'd34: exp2_lut = 16'hA5DC;
            6'd35: exp2_lut = 16'hA41B;
            6'd36: exp2_lut = 16'hA264;
            6'd37: exp2_lut = 16'hA0B8;
            6'd38: exp2_lut = 16'h9F17;
            6'd39: exp2_lut = 16'h9D80;
            6'd40: exp2_lut = 16'h9BF4;
            6'd41: exp2_lut = 16'h9A71;
            6'd42: exp2_lut = 16'h98F9;
            6'd43: exp2_lut = 16'h978A;
            6'd44: exp2_lut = 16'h9626;
            6'd45: exp2_lut = 16'h94CB;
            6'd46: exp2_lut = 16'h937A;
            6'd47: exp2_lut = 16'h9233;
            6'd48: exp2_lut = 16'h90F6;
            6'd49: exp2_lut = 16'h8FC2;
            6'd50: exp2_lut = 16'h8E98;
            6'd51: exp2_lut = 16'h8D77;
            6'd52: exp2_lut = 16'h8C5F;
            6'd53: exp2_lut = 16'h8B50;
            6'd54: exp2_lut = 16'h8A4B;
            6'd55: exp2_lut = 16'h894F;
            6'd56: exp2_lut = 16'h885B;
            6'd57: exp2_lut = 16'h8771;
            6'd58: exp2_lut = 16'h8690;
            6'd59: exp2_lut = 16'h85B8;
            6'd60: exp2_lut = 16'h84E9;
            6'd61: exp2_lut = 16'h8422;
            6'd62: exp2_lut = 16'h8365;
            6'd63: exp2_lut = 16'h82B0;
        endcase
    endfunction

    assign lut_val = exp2_lut(f_idx);

    // Apply integer exponent shift 2^(-k)
    // Zero-extend 16-bit LUT output to 24-bit Q0.24, then shift right by k_int
    logic [OUT_WIDTH-1:0] shifted_weight;
    assign shifted_weight = (k_int >= 24) ? {OUT_WIDTH{1'b0}} :
                            ({lut_val, 8'h00} >> k_int[4:0]);

    // Pipeline register
    logic [OUT_WIDTH-1:0] weight_q;
    logic                 valid_q;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            weight_q <= '0;
            valid_q  <= 1'b0;
        end else if (enable) begin
            weight_q <= shifted_weight;
            valid_q  <= 1'b1;
        end else begin
            valid_q  <= 1'b0;
        end
    end

    assign weight_out = weight_q;
    assign valid_out  = valid_q;

endmodule

