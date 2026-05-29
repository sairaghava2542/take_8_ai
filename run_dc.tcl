# ============================================================================
# run_dc.tcl
#
# Final clean Design Compiler NXT low-power synthesis script for accel_top.
#
# Run from this folder:
#   cd ~/Downloads/final_project_ai
#   dc_shell -f run_dc.tcl
#
# Outputs:
#   syn/accel_top.lp.syn.v
#   syn/accel_top.ddc
#   syn/accel_top.syn.sdc
#   syn/accel_top.syn.upf
#
# Reports:
#   reports/dc/*.rpt
#
# Notes:
#   - Default clock is 6.20 ns because the previous 5.30 ns run had
#     setup WNS around -0.72 ns.
#   - Hold uses smaller hold uncertainty than setup uncertainty.
#   - Max transition is set to 0.60 ns so the user constraint does not
#     over-constrain reset/control nets beyond the library rule.
# ============================================================================

set_app_var sh_continue_on_error false

set DESIGN_NAME accel_top
set RTL_DIR     ./rtl
set UPF_IN      ./accel_top.upf
set OUT_DIR     ./syn
set RPT_DIR     ./reports/dc

set CLOCK_PERIOD      6.20
set SETUP_UNCERTAINTY 0.250
set HOLD_UNCERTAINTY  0.050
set IO_DELAY          1.00
set OUT_LOAD          0.05
set MAX_TRANSITION    0.60
set MAX_FANOUT        10

file mkdir $OUT_DIR
file mkdir $RPT_DIR

redirect -file $RPT_DIR/run_settings.rpt {
    puts "DESIGN_NAME       = $DESIGN_NAME"
    puts "CLOCK_PERIOD      = $CLOCK_PERIOD"
    puts "SETUP_UNCERTAINTY = $SETUP_UNCERTAINTY"
    puts "HOLD_UNCERTAINTY  = $HOLD_UNCERTAINTY"
    puts "IO_DELAY          = $IO_DELAY"
    puts "OUT_LOAD          = $OUT_LOAD"
    puts "MAX_TRANSITION    = $MAX_TRANSITION"
    puts "MAX_FANOUT        = $MAX_FANOUT"
}

# ----------------------------------------------------------------------------
# SAED32 library setup
# ----------------------------------------------------------------------------
set SAED32_ROOT /data/pdk/pdk32nm/SAED32_EDK
set RVT_DB      $SAED32_ROOT/lib/stdcell_rvt/db_ccs
set HVT_DB      $SAED32_ROOT/lib/stdcell_hvt/db_ccs
set LVT_DB      $SAED32_ROOT/lib/stdcell_lvt/db_ccs

proc pick_one_db {pattern} {
    set matches [lsort [glob -nocomplain $pattern]]
    if {[llength $matches] == 0} {
        error "No .db matched pattern: $pattern"
    }
    return [lindex $matches 0]
}

proc pick_first_db {patterns} {
    foreach pattern $patterns {
        set matches [lsort [glob -nocomplain $pattern]]
        if {[llength $matches] > 0} {
            return [lindex $matches 0]
        }
    }
    error "No .db matched any pattern: $patterns"
}

set LIB_RVT_AON_TT      [pick_one_db $RVT_DB/saed32rvt_tt1p05v25c.db]
set LIB_RVT_COMP_TT     [pick_one_db $RVT_DB/saed32rvt_tt0p78v25c.db]
set LIB_HVT_AON_TT      [pick_one_db $HVT_DB/saed32hvt_tt1p05v25c.db]
set LIB_HVT_COMP_TT     [pick_one_db $HVT_DB/saed32hvt_tt0p78v25c.db]
set LIB_LVT_AON_TT      [pick_one_db $LVT_DB/saed32lvt_tt1p05v25c.db]
set LIB_LVT_COMP_TT     [pick_one_db $LVT_DB/saed32lvt_tt0p78v25c.db]
set LIB_RVT_PG_TT       [pick_one_db $RVT_DB/saed32rvt_pg_tt1p05v25c.db]
set LIB_RVT_ULVL_TT     [pick_one_db $RVT_DB/saed32rvt_ulvl_tt1p05v25c_*0p78v.db]
set LIB_RVT_DLVL_TT     [pick_first_db [list \
    $RVT_DB/saed32rvt_dlvl_tt0p78v25c_i1p05v.db \
    $RVT_DB/saed32rvt_dlvl_tt0p78v25c_*1p05v.db \
    $RVT_DB/saed32rvt_dlvl_tt1p05v25c_*1p05v.db \
]]

set_app_var search_path [list . $RTL_DIR $RVT_DB $HVT_DB $LVT_DB]

set target_library [list \
    $LIB_RVT_AON_TT \
    $LIB_RVT_COMP_TT \
    $LIB_RVT_PG_TT \
    $LIB_RVT_ULVL_TT \
    $LIB_RVT_DLVL_TT \
    $LIB_LVT_AON_TT \
    $LIB_LVT_COMP_TT \
]

set link_library [concat "*" $target_library [list \
    $LIB_HVT_AON_TT \
    $LIB_HVT_COMP_TT \
]]

set_app_var target_library $target_library
set_app_var link_library   $link_library

# Preserve useful hierarchy for UPF and ICC2 handoff.
set_app_var compile_ultra_ungroup_dw false
set_app_var hdlin_preserve_sequential true

# ----------------------------------------------------------------------------
# RTL read/elaboration
# ----------------------------------------------------------------------------
analyze -format verilog [list \
    $RTL_DIR/systolic_pe.v \
    $RTL_DIR/systolic_array_8x8.v \
    $RTL_DIR/accel_regs.v \
    $RTL_DIR/accel_top.v \
]

elaborate $DESIGN_NAME
current_design $DESIGN_NAME
link
uniquify

set_ungroup [get_cells u_regs]    false
set_ungroup [get_cells u_compute] false

# ----------------------------------------------------------------------------
# Timing constraints
# ----------------------------------------------------------------------------
create_clock -name clk -period $CLOCK_PERIOD [get_ports clk]
set_clock_uncertainty -setup $SETUP_UNCERTAINTY [get_clocks clk]
set_clock_uncertainty -hold  $HOLD_UNCERTAINTY  [get_clocks clk]

set_input_delay $IO_DELAY -clock [get_clocks clk] \
    [remove_from_collection [all_inputs] [get_ports clk]]
set_output_delay $IO_DELAY -clock [get_clocks clk] [all_outputs]

set_max_fanout $MAX_FANOUT [current_design]
set_max_transition $MAX_TRANSITION [current_design]
set_load $OUT_LOAD [all_outputs]

# ----------------------------------------------------------------------------
# UPF low-power intent
# ----------------------------------------------------------------------------
load_upf $UPF_IN

set_voltage 1.05 -object_list {VDD_ALW}
set_voltage 0.78 -object_list {VDD_COMP VDD_COMP_SW}
set_voltage 0.00 -object_list {VSS}

check_mv_design -verbose > $RPT_DIR/check_mv_design.pre_compile.rpt

# ----------------------------------------------------------------------------
# Clock gating and compile
# ----------------------------------------------------------------------------
set_clock_gating_style \
    -positive_edge_logic integrated \
    -negative_edge_logic integrated \
    -control_point before \
    -max_fanout 32

set_fix_hold [get_clocks clk]
compile_ultra -gate_clock -timing_high_effort_script
compile_ultra -incremental -gate_clock

# ----------------------------------------------------------------------------
# Reports
# ----------------------------------------------------------------------------
check_design > $RPT_DIR/check_design.rpt
check_timing > $RPT_DIR/check_timing.rpt
check_mv_design -verbose > $RPT_DIR/check_mv_design.post_compile.rpt

report_qor > $RPT_DIR/qor.rpt
report_constraint -all_violators > $RPT_DIR/constraints_violators.rpt
report_area -hierarchy > $RPT_DIR/area_hier.rpt
report_power -hierarchy > $RPT_DIR/power_hier.rpt
report_clock_gating > $RPT_DIR/clock_gating.rpt
report_timing -delay_type max -path full -max_paths 50 -nets -transition_time -capacitance > $RPT_DIR/timing_max.rpt
report_timing -delay_type min -path full -max_paths 50 -nets -transition_time -capacitance > $RPT_DIR/timing_min.rpt
report_power_domain > $RPT_DIR/power_domains.rpt
report_isolation > $RPT_DIR/isolation.rpt
report_isolation_cell > $RPT_DIR/iso_cells.rpt
report_level_shifter > $RPT_DIR/level_shifters.rpt

# ----------------------------------------------------------------------------
# Deliverables
# ----------------------------------------------------------------------------
change_names -rules verilog -hierarchy

write -format verilog -hierarchy -output $OUT_DIR/${DESIGN_NAME}.lp.syn.v
write -format ddc     -hierarchy -output $OUT_DIR/${DESIGN_NAME}.ddc
write_sdc $OUT_DIR/${DESIGN_NAME}.syn.sdc
save_upf  $OUT_DIR/${DESIGN_NAME}.syn.upf

puts "============================================================"
puts "DC synthesis complete"
puts "Reports : $RPT_DIR"
puts "Netlist : $OUT_DIR/${DESIGN_NAME}.lp.syn.v"
puts "DDC     : $OUT_DIR/${DESIGN_NAME}.ddc"
puts "SDC     : $OUT_DIR/${DESIGN_NAME}.syn.sdc"
puts "UPF     : $OUT_DIR/${DESIGN_NAME}.syn.upf"
puts "============================================================"
