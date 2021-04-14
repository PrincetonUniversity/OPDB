yosys read_verilog $::env(INPUT_VERILOG_FILE)
yosys synth -flatten -top $::env(TOP_MODULE)
yosys abc -dff -g AND
yosys write_aiger  $::env(OUTPUT_AIGER_FILE)


