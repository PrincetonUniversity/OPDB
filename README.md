# OpenPiton Design Benchmark
OpenPiton Design Benchmark adapted from [OpenPiton](https://github.com/PrincetonUniversity/openpiton).

## Find out more

- More information about OpenPiton is available at www.openpiton.org
- [Follow us on Twitter!](https://www.twitter.com/openpiton)
- Get help from others by joining our [Google Group](https://groups.google.com/group/openpiton)
- Keep up-to-date with the latest releases at the [OpenPiton Blog](https://openpiton-blog.princeton.edu)
- [Floorplan and sdc files](#floorplan-and-sdc-files)
- [Licenses](#licenses)

## Benchmark

The following table presents which modules from OpenPiton are included on OPDB, the name of the
top level module for each pickled module, and which attributes affect each module.


| Module name        | Top module                    | Has Macros         | X-dimension        | Y-dimension        | Topology           | L1-I               | L1-D               | L1.5               | L2                 |
|--------------------|-------------------------------|--------------------|--------------------|--------------------|--------------------|--------------------|--------------------|--------------------|--------------------|
| chip               | chip                          | :heavy_check_mark: | :heavy_check_mark: | :heavy_check_mark: | :heavy_check_mark: | :heavy_check_mark: | :heavy_check_mark: | :heavy_check_mark: | :heavy_check_mark: |
| chip_bridge        | chip_bridge                   |                    |                    |                    |                    |                    |                    |                    |                    |
| dynamic_node       | [*](#dynamic-node-top-module) |                    |                    |                    | :heavy_check_mark: |                    |                    |                    |                    |
| fpga_bridge_rcv_32 | fpga_bridge_rcv_32            |                    |                    |                    |                    |                    |                    |                    |                    |
| fpu                | fpu                           |                    |                    |                    |                    |                    |                    |                    |                    |
| ifu_esl            | sparc_ifu_esl                 |                    |                    |                    |                    |                    |                    |                    |                    |
| ifu_esl_counter    | sparc_ifu_esl_counter         |                    |                    |                    |                    |                    |                    |                    |                    |
| ifu_esl_fsm        | sparc_ifu_esl_fsm             |                    |                    |                    |                    |                    |                    |                    |                    |
| ifu_esl_htsm       | sparc_ifu_esl_htsm            |                    |                    |                    |                    |                    |                    |                    |                    |
| ifu_esl_lfsr       | sparc_ifu_esl_lfsr            |                    |                    |                    |                    |                    |                    |                    |                    |
| ifu_esl_rtsm       | sparc_ifu_esl_rtsm            |                    |                    |                    |                    |                    |                    |                    |                    |
| ifu_esl_shiftreg   | sparc_ifu_esl_shiftreg        |                    |                    |                    |                    |                    |                    |                    |                    |
| ifu_esl_stsm       | sparc_ifu_esl_stsm            |                    |                    |                    |                    |                    |                    |                    |                    |
| l15                | l15_wrap                      | :heavy_check_mark: |                    |                    | :heavy_check_mark: |                    | :heavy_check_mark: | :heavy_check_mark: |                    |
| l2                 | l2                            | :heavy_check_mark: |                    |                    | :heavy_check_mark: |                    |                    |                    | :heavy_check_mark: |
| MIAOW (GPGPU)      | neko                          | :heavy_check_mark: |                    |                    |                    |                    |                    |                    |                    |
| pico               | picorv32                      |                    |                    |                    |                    |                    |                    |                    |                    |
| sparc_core         | sparc_core                    | :heavy_check_mark: |                    |                    |                    | :heavy_check_mark: | :heavy_check_mark: |                    |                    |
| sparc_exu          | sparc_exu_wrap                |                    |                    |                    |                    |                    |                    |                    |                    |
| sparc_ffu          | sparc_ffu_nospu_wrap          | :heavy_check_mark: |                    |                    |                    |                    |                    |                    |                    |
| sparc_ifu          | sparc_ifu                     | :heavy_check_mark: |                    |                    |                    | :heavy_check_mark: |                    |                    |                    |
| sparc_lsu          | lsu                           | :heavy_check_mark: |                    |                    |                    |                    | :heavy_check_mark: |                    |                    |
| sparc_mul          | sparc_mul_top_nospu_wrap      |                    |                    |                    |                    |                    |                    |                    |                    |
| sparc_tlu          | tlu_nospu_wrap                |                    |                    |                    |                    |                    |                    |                    |                    |
| tile               | tile                          | :heavy_check_mark: |                    |                    | :heavy_check_mark: | :heavy_check_mark: | :heavy_check_mark: | :heavy_check_mark: | :heavy_check_mark: |
| FFT                | fftmain                       |                    |                    |                    |                    |                    |                    |                    |                    |
| GNG                | gng                           |                    |                    |                    |                    |                    |                    |                    |                    |

### Dynamic node top module
The top module of dynamic node depends on the network topology:

| Topology | Top module                 |
|----------|----------------------------|
| 2dmesh   | dynamic_node_top_wrap      |
| xbar     | dynamic_node_top_wrap_para |

## Configuration naming convention



`<attr1_id>_<attr1_val1>_..._<attr1_valM>__...__<attrN_id>_<attrN_val1>_..._<attrN_valM>`

Attribute              | Attribute ID | Value 1  | Value 2       |
-----------------------|--------------|----------|---------------|
L1 data cache          | L1D          | size     | associativity |
L1 instruction cache   | L1I          | size     | associativity |
L1.5 cache             | L15          | size     | associativity |
L2 cache               | L2           | size     | associativity |
Network topology       | NETWORK      | topology | -             |
\# tiles in X dimension | X            | width    | -             |
\# tiles in y dimension | Y            | width    | -             |


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

* [modules/dynamic_node/NETWORK_2dmesh](modules/dynamic_node/NETWORK_2dmesh)
* [modules/sparc_core/L1I_16384_4__L1D_8192_4](modules/sparc_core/L1I_16384_4__L1D_8192_4)

## Licenses

For details on the license of the MIAOW GPGPU see [here](https://github.com/VerticalResearchGroup/miaow)

For details on the license of the OpenSPARC T1 core see [here](https://github.com/PrincetonUniversity/openpiton/blob/openpiton/piton/GPLv2_License_OpenSPARCT1.txt)

The Ariane RISC-V CPU is licensed under the [SolderPad Hardware License](https://github.com/pulp-platform/ariane/blob/master/LICENSE)

The PicoRV32 core is licensed under the [ISC license](http://en.wikipedia.org/wiki/ISC_license)

The GNG core is licensed under [LGPL 2.1](https://www.gnu.org/licenses/old-licenses/lgpl-2.1.en.html) 

The FFT core is licensed under [LGPL 3](https://www.gnu.org/licenses/lgpl-3.0.en.html)

The OpenPiton project is licensed under the following 3-clause BSD license:

```
Copyright (c) Princeton University
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
    * Neither the name of Princeton University nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.
    
THIS SOFTWARE IS PROVIDED BY PRINCETON UNIVERSITY "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL PRINCETON UNIVERSITY BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
```

