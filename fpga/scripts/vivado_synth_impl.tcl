# ==============================================================================
# Vivado Full Flow Script: H-MPPI Accelerator Core for Xilinx Artix-7
# Targets: Artix-7 xc7a35t-cpg236-1 (Primary) / xc7a200tfbg676-2 (High-Capacity)
# Flow   : Project Creation -> RTL Read -> Synthesis -> Implementation (Opt, Place, Route) -> Sign-off Reports
# ==============================================================================

set script_dir [file dirname [file normalize [info script]]]
set fpga_dir   [file normalize [file join $script_dir ".."]]
set top_dir    [file normalize [file join $fpga_dir ".."]]
set rtl_dir    [file join $top_dir "rtl"]
set xdc_file   [file join $fpga_dir "constraints" "hmppi_core_top.xdc"]
set reports_dir [file join $fpga_dir "reports"]
set dcp_dir    [file join $fpga_dir "checkpoints"]

file mkdir $reports_dir
file mkdir $dcp_dir

# Configuration Parameters
set top_module "hmppi_core_top"
set target_part_35t  "xc7a35tcpg236-1"
set target_part_200t "xc7a200tfbg676-2"

# Select Part (Default to xc7a35tcpg236-1)
if {[info exists ::env(TARGET_PART)]} {
    set part $::env(TARGET_PART)
} else {
    set part $target_part_35t
}

puts "========================================================================"
puts "  H-MPPI Artix-7 FPGA Implementation Flow"
puts "  Target Part   : $part"
puts "  Top Module    : $top_module"
puts "  RTL Directory : $rtl_dir"
puts "  Reports Dir   : $reports_dir"
puts "========================================================================"

# Create in-memory project
create_project -in_memory -part $part

# Read SystemVerilog RTL Files
read_verilog -sv [file join $rtl_dir "chebyshev_slosh_eval.sv"]
read_verilog -sv [file join $rtl_dir "fexp2_unit.sv"]
read_verilog -sv [file join $rtl_dir "sobol_qmc_4d.sv"]
read_verilog -sv [file join $rtl_dir "mppi_rollout_lane.sv"]
read_verilog -sv [file join $rtl_dir "hmppi_core_top.sv"]

# Read Timing Constraints
read_xdc $xdc_file

# Synthesis Phase
puts "------------------------------------------------------------------------"
puts "  Stage 1: Logic Synthesis (Target Clock: 50 MHz / 20.0 ns)"
puts "------------------------------------------------------------------------"
synth_design -top $top_module -part $part -flatten_hierarchy rebuilt -keep_equivalent_registers

# Write Post-Synthesis Checkpoint & Reports
write_checkpoint -force [file join $dcp_dir "${top_module}_post_synth.dcp"]
report_utilization -file [file join $reports_dir "synth_utilization.rpt"] -pb [file join $reports_dir "synth_utilization.pb"]
report_timing_summary -file [file join $reports_dir "synth_timing_summary.rpt"] -max_paths 10

# Implementation Phase: Optimization & Placement
puts "------------------------------------------------------------------------"
puts "  Stage 2: Logic Optimization & Physical Placement"
puts "------------------------------------------------------------------------"
opt_design
place_design
report_clock_utilization -file [file join $reports_dir "impl_clock_utilization.rpt"]
report_utilization -file [file join $reports_dir "impl_utilization_placed.rpt"]

# Physical Optimization
phys_opt_design

# Routing Phase
puts "------------------------------------------------------------------------"
puts "  Stage 3: Detailed Routing"
puts "------------------------------------------------------------------------"
route_design

# Write Post-Route Checkpoint
write_checkpoint -force [file join $dcp_dir "${top_module}_routed.dcp"]

# Sign-off Reports
puts "------------------------------------------------------------------------"
puts "  Stage 4: Sign-off Verification Reports"
puts "------------------------------------------------------------------------"
report_utilization -file [file join $reports_dir "impl_utilization_routed.rpt"] -pb [file join $reports_dir "impl_utilization_routed.pb"]
report_timing_summary -delay_type min_max -report_unconstrained -check_timing_verbose -max_paths 20 -input_pins -routable_nets -file [file join $reports_dir "impl_timing_summary_routed.rpt"]
report_power -file [file join $reports_dir "impl_power_routed.rpt"]
report_drc -file [file join $reports_dir "impl_drc_routed.rpt"]

puts "========================================================================"
puts "  H-MPPI FPGA Flow Complete Successfully"
puts "========================================================================"
