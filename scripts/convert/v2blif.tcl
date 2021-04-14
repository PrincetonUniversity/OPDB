yosys read_verilog $::env(INPUT_VERILOG_FILE)
yosys hierarchy -check -top $::env(TOP_MODULE)
yosys proc
yosys opt
yosys memory
yosys opt
yosys fsm
yosys opt
yosys techmap
yosys opt
yosys write_blif $::env(OUTPUT_BLIF_FILE)
