set DESIGN_NAME $::env(DESIGN_NAME)
set ASIC_PROCESS $::env(ASIC_PROCESS)


set RTL_SOURCE_FILES $::env(INPUT_VERILOG_FILE)

foreach RTL_SOURCE_FILE ${RTL_SOURCE_FILES} {
    yosys read_verilog "${RTL_SOURCE_FILE}"
}

# check design hierarchy
yosys hierarchy -top ${DESIGN_NAME}

# translate processes (always blocks)
yosys proc
yosys opt

# detect and optimize FSM encodings
yosys fsm
yosys opt

# implement memories (arrays)
yosys memory
yosys opt

# convert to gate logic
yosys techmap
yosys opt

if {$ASIC_PROCESS eq "asap7"} {
    set ASAP7_IP $::env(ASAP7_IP)
    # mapping logic to ASAP7 library
    yosys abc -dff -liberty "${ASAP7_IP}/asap7libs_24/lib/asap_7nm.lib" -D 200

    # mapping flip-flops to ASAP7 library
    yosys dfflibmap -liberty "${ASAP7_IP}/asap7libs_24/lib/asap7sc7p5t_24_SEQ_RVT_TT.lib"
} elseif {$ASIC_PROCESS eq "nan45"} {
    set NAN45_IP $::env(NAN45_IP)
    # mapping logic to NAN45 library
    yosys abc -dff -liberty "${NAN45_IP}/Front_End/Liberty/NLDM/NangateOpenCellLibrary_typical.lib" -D 200

    # mapping flip-flops to NAN45 library
    yosys dfflibmap -liberty "${NAN45_IP}/Front_End/Liberty/NLDM/NangateOpenCellLibrary_typical.lib"
}

set RESULTS_DIR "results"
if {![file exists ${RESULTS_DIR}]} {
    file mkdir $RESULTS_DIR
}

set DCRM_FINAL_VERILOG_OUTPUT_FILE  ${DESIGN_NAME}.mapped.v

yosys write_verilog ${RESULTS_DIR}/${DCRM_FINAL_VERILOG_OUTPUT_FILE}
