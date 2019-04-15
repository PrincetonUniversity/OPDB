# OpenPiton Design Benchmark
OpenPiton Design Benchmark adapted from [OpenPiton](https://github.com/PrincetonUniversity/openpiton).

## Find out more

- More information about OpenPiton is available at www.openpiton.org
- [Follow us on Twitter!](https://www.twitter.com/openpiton)
- Get help from others by joining our [Google Group](https://groups.google.com/group/openpiton)
- Keep up-to-date with the latest releases at the [OpenPiton Blog](https://openpiton-blog.princeton.edu)
- [Floorplan and sdc files](#floorplan-and-sdc-files)

## Benchmark

The following table presents which modules from OpenPiton are included on OPDB, the name of the
top level module for each pickled module, and which attributes affect each module.


| Module name        | Top module                    | Topology | L1-I | L1-D | L1.5 | L2 |
|--------------------|--------------------------     |----------|------|------|------|----|
| chip               | chip                          | X        | X    | X    | X    | X  |
| chip_bridge        | chip_bridge                   |          |      |      |      |    |
| dynamic_node       | [*](#dynamic-node-top-module) | X        |      |      |      |    |
| fpga_bridge_rcv_32 | fpga_bridge_rcv_32            |          |      |      |      |    |
| fpu                | fpu                           |          |      |      |      |    |
| ifu_esl            | sparc_ifu_esl                 |          |      |      |      |    |
| ifu_esl_counter    | sparc_ifu_esl_counter         |          |      |      |      |    |
| ifu_esl_fsm        | sparc_ifu_esl_fsm             |          |      |      |      |    |
| ifu_esl_htsm       | sparc_ifu_esl_htsm            |          |      |      |      |    |
| ifu_esl_lfsr       | sparc_ifu_esl_lfsr            |          |      |      |      |    |
| ifu_esl_rtsm       | sparc_ifu_esl_rtsm            |          |      |      |      |    |
| ifu_esl_shiftreg   | sparc_ifu_esl_shiftreg        |          |      |      |      |    |
| ifu_esl_stsm       | sparc_ifu_esl_stsm            |          |      |      |      |    |
| l15                | l15                           | X        |      | X    | X    | X  |
| l2                 | l2                            | X        |      |      | X    | X  |
| pico               | picorv32                      |          |      |      |      |    |
| sparc_core         | sparc_core                    |          | X    | X    |      |    |
| sparc_exu          | sparc_exu_wrap                |          |      |      |      |    |
| sparc_ffu          | sparc_ffu_nospu_wrap          |          |      |      |      |    |
| sparc_ifu          | sparc_ifu                     |          | X    |      |      |    |
| sparc_lsu          | lsu                           |          |      | X    |      |    |
| sparc_mul          | sparc_mul_top_nospu_wrap      |          |      |      |      |    |
| sparc_tlu          | tlu_nospu_wrap                |          |      |      |      |    |
| tile               | tile                          | X        | X    | X    | X    | X  |

### Dynamic node top module
The top module of dynamic node depends on the network topology:

| Topology | Top module                 |
|----------|----------------------------|
| 2dmesh   | dynamic_node_top_wrap      |
| xbar     | dynamic_node_top_wrap_para |

## Configuration naming convention



`<attr1_id>_<attr1_val1>_..._<attr1_valM>__...__<attrN_id>_<attrN_val1>_..._<attrN_valM>`

Attribute            | Attribute ID | Value 1  | Value 2       |
---------------------|--------------|----------|---------------|
L1 data cache        | L1D          | size     | associativity |
L1 instruction cache | L1I          | size     | associativity |
L1.5 cache           | L15          | size     | associativity |
L2 cache             | L2           | size     | associativity |
Network topology     | NETWORK      | topology | -             |


### Organization

```
+ modules/
 \
  + <module name>
  |\
  | + <configuration>
  | |\
  | | + <module name>.pickle.v
  | | + floorplan.json
  | |
  | + <configuration>
  ...
  |
  + <module name>
```
#### Configuration folder

Each configuration folder will contain a pickled file of verilog code and a floorplan json file.
The floorplan json file will provide the top level module and additional information.

### Floorplan and sdc files

Specific configurations include floorplan and sdc files. The current ones are:

* modules/dynamic_node
* modules/sparc_core/L1I_16384_4__L1D_8192_4