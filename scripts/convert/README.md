# Convert from Verilog to other formats

The scripts in this directory are skeletons for converting the pickled Verilog files to a BLIF and AIGER formats for further processing.
The scripts use Yosys and ABC.

- v2blif.tcl:  Verilog to BLIF
- v2aiger.tcl: Verilog to AIGER

To use the scripts you will need to define the following environment variables:

- INPUT_VERILOG_FILE
- TOP_MODULE: see toplevel field in floorplan.json
- OUTPUT_BLIF_FILE: You can set it to `${INPUT_VERILOG_FILE%.v}.blif`
- OUTPUT_AIGER_FILE: You can set it to `${INPUT_VERILOG_FILE%.v}.aag`
