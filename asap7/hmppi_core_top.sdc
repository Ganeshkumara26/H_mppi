# SDC timing constraints for H-MPPI Accelerator on ASAP7 7nm FinFET
# Target Clock: 500 MHz (2.0 ns period)

set clk_period 2.0
set clk_uncertainty 0.05
set clk_transition 0.05

create_clock -name clk -period $clk_period [get_ports clk]
set_clock_uncertainty $clk_uncertainty [get_clocks clk]
set_clock_transition $clk_transition [get_clocks clk]

# Constrain inputs and outputs relative to clk
set in_delay 0.4
set out_delay 0.4

# Target data inputs specifically
set_input_delay $in_delay -clock clk [get_ports {rst_n start_frame curr_pos_* curr_vel_* goal_pos_* u_nom_* cov_scale_in eta_resp_valid eta_inv_fp32*}]
set_output_delay $out_delay -clock clk [get_ports {eta_req_valid eta_sum_fp32* opt_u_* min_cost_out* frame_done}]

# False paths
set_false_path -from [get_ports rst_n]
