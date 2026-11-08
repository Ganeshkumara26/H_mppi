# ==============================================================================
# Timing and Physical Constraints for H-MPPI Guidance Accelerator Core
# Top Module : hmppi_core_top
# Target FPGA: Xilinx Artix-7 (xc7a35t-cpg236-1 / xc7a200tfbg676-2)
# Clock Spec : 50.0 MHz (Period: 20.000 ns, 50% duty cycle)
# ==============================================================================

# Primary System Clock
create_clock -period 20.000 -name sys_clk -waveform {0.000 10.000} [get_ports clk]

# Clock Uncertainty & Jitter
set_clock_uncertainty 0.200 [get_clocks sys_clk]

# Asynchronous Reset False Path
set_false_path -from [get_ports rst_n]

# Input / Output Timing Budget (3.0 ns max / 0.5 ns min interface budget)
set_input_delay -clock [get_clocks sys_clk] -max 3.000 [get_ports {start cov_scale_in u_nom* p_goal* x0* v0* theta0* omega0* m0* eta_resp_valid eta_inv_fp32*}]
set_input_delay -clock [get_clocks sys_clk] -min 0.500 [get_ports {start cov_scale_in u_nom* p_goal* x0* v0* theta0* omega0* m0* eta_resp_valid eta_inv_fp32*}]

set_output_delay -clock [get_clocks sys_clk] -max 3.000 [get_ports {busy done u_opt* best_cost* eta_req_valid eta_sum_fp32*}]
set_output_delay -clock [get_clocks sys_clk] -min 0.500 [get_ports {busy done u_opt* best_cost* eta_req_valid eta_sum_fp32*}]

# Configuration / Multicycle Paths if applicable
# Die 1 handshake is multi-cycle asynchronous coprocessor exchange:
set_multicycle_path 2 -from [get_ports eta_resp_valid] -to [all_registers] -setup
set_multicycle_path 1 -from [get_ports eta_resp_valid] -to [all_registers] -hold
