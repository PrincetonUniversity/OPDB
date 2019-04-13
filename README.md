# OpenPiton Design Benchmark
OpenPiton Design Benchmark adapted from [OpenPiton](https://github.com/PrincetonUniversity/openpiton).

## Find out more

- More information about OpenPiton is available at www.openpiton.org
- [Follow us on Twitter!](https://www.twitter.com/openpiton)
- Get help from others by joining our [Google Group](https://groups.google.com/group/openpiton)
- Keep up-to-date with the latest releases at the [OpenPiton Blog](https://openpiton-blog.princeton.edu)

## Benchmark

### Supported modules

| Module name        | Topology | L1-I | L1-D | L1.5 | L2 |
|--------------------|----------|------|------|------|----|
| chip_bridge        |          |      |      |      |    |
| dynamic_node       | X        |      |      |      |    |
| fpu                |          |      |      |      |    |
| ifu_esl            |          |      |      |      |    |
| ifu_esl_counter    |          |      |      |      |    |
| ifu_esl_fsm        |          |      |      |      |    |
| ifu_esl_htsm       |          |      |      |      |    |
| ifu_esl_lfsr       |          |      |      |      |    |
| ifu_esl_rtsm       |          |      |      |      |    |
| ifu_esl_shiftreg   |          |      |      |      |    |
| ifu_esl_stsm       |          |      |      |      |    |
| l15                | X        |      | X    | X    | X  |
| l2                 | X        |      |      | X    | X  |
| pico               |          |      |      |      |    |
| sparc_core         |          | X    | X    |      |    |
| sparc_exu          |          |      |      |      |    |
| sparc_ffu          |          |      |      |      |    |
| sparc_ifu          |          | X    |      |      |    |
| sparc_lsu          |          |      | X    |      |    |
| sparc_mul          |          |      |      |      |    |
| sparc_srams        |          | X    | X    |      |    |
| sparc_tlu          |          |      |      |      |    |
| fpga_bridge_rcv_32 |          |      |      |      |    |
| tile               | X        | X    | X    | X    | X  |
| chip               | X        | X    | X    | X    | X  |

### Organization

```
+ modules/
 \
  + <module name>
  |\
  | + <configuration>
  | |\
  | | + <module name>.pickle.v
  | |
  | + <configuration>
  ...
  |
  + <module name>
```

### Configuration naming convention

`<attr1_id>_<attr1_val1>_..._<attr1_valM>__...__<attrN_id>_<attrN_val1>_..._<attrN_valM>`

| Attribute | Value 1  | Value 2       |
|-----------|----------|---------------|
| NETWORK   | topology | -             |
| L1D       | size     | associativity |
| L1I       | size     | associativity |
| L15       | size     | associativity |
| L2        | size     | associativity |

### Floorplan and sdc files

Specific configurations include floorplan and sdc files. The current ones are:

* modules/dynamic_node
* modules/sparc_core/L1I_16384_4__L1D_8192_4