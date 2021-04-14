#!/bin/bash


export DESIGN_NAME="dynamic_node_top_wrap"
export INPUT_VERILOG_FILE="../../modules/dynamic_node_2dmesh/NETWORK_2dmesh/dynamic_node_2dmesh.pickle.v"
export ASAP7_IP="<Path to ASAP7_PDKandLIB_v1p5>"
export NAN45_IP="<Path to NangateOpenCellLibrary_PDKv1_3_v2010_12>"
export ASIC_PROCESS="nan45"
#export ASIC_PROCESS="asap7"
export MAPPED_FLIST="Flist.mapped.${ASIC_PROCESS}"

# Yosys script for mapping to specific library
yosys synth.tcl

# Compile mapped design together with playback module
iverilog \
    -c ${MAPPED_FLIST} \
    -s cmp_top \
    -o example

echo "Running gatesim simulation"

./example \
    +stim_file=./stimuli.txt
