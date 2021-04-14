= Gate-level simulation

The current folder contains scripts and files that can get you started with gate-level simulation and verification using pickled files and open-source tools and
technology libraries. To run the example you will need to obtain either the ASAP7 or Nangate 45 libraries. The libraries are not included in these repository.


- example.sh: Top level script for this example.
- synth.tcl: TCL script that uses yosys to created a mapped version of the pickled file. The script can be configured through environment variables
- stimuli.txt: Stimuli files for the gate-level verification
- define.tmp.h:  part of the OpenPiton project included in this folder for convenience
- network_define.v: part of the OpenPiton project included in this folder for convenience
- Flist.mapped.* : Flist files


The example.sh script contains a number of environment variables that need to be set correctly before running the example.
```
export ASAP7_IP="<Path to ASAP7_PDKandLIB_v1p5>"
export NAN45_IP="<Path to NangateOpenCellLibrary_PDKv1_3_v2010_12>"
export ASIC_PROCESS="nan45"; # Can be nan45 or asap7
```

Furthermore, the Flists need to be enhanced with the correct paths to the node libraries.