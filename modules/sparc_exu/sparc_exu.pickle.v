// Modified by Princeton Unviersity on June 9th, 2015
// ========== Copyright Header Begin ==========================================
// 
// OpenSPARC T1 Processor File: bw_r_irf_register.v
// Copyright (c) 2006 Sun Microsystems, Inc.  All Rights Reserved.
// DO NOT ALTER OR REMOVE COPYRIGHT NOTICES.
// 
// The above named program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public
// License version 2 as published by the Free Software Foundation.
// 
// The above named program is distributed in the hope that it will be 
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// General Public License for more details.
// 
// You should have received a copy of the GNU General Public
// License along with this work; if not, write to the Free Software
// Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA 02110-1301, USA.
// 
// ========== Copyright Header End ============================================















































































































































 //FPGA_SYN_1THREAD




module bw_r_irf_register(clk, reset_l, wrens, save, save_addr, restore, restore_addr, wr_data0, wr_data1, rd_thread, rd_data);
    parameter NUM_WINDOW_ENTRIES = 16;
    parameter SHIFT_ADDR = 0;
	input		clk;
	input		reset_l;
	input	[1:0]	wrens;
	input		save;
	input	[3:0]	save_addr;
	input		restore;
	input	[3:0]	restore_addr;
	input	[71:0]	wr_data0;
	input	[71:0]	wr_data1;
	input		rd_thread;
	output	[71:0]	rd_data;



    reg	[71:0]	window[NUM_WINDOW_ENTRIES-1:0]/* synthesis syn_ramstyle = block_ram  syn_ramstyle = no_rw_check */;

reg	[71:0]	reg_th0, reg_th1;

reg	[3:0]	rd_addr;
reg	[3:0]	wr_addr;
reg		save_d;

// wentzlaff I disabled the initialization block. We are replacing the reset
// of reg_th0 and reg_th1 with a functional reset to zero done below
// initial begin
//  reg_th0 = 72'b0;
//  reg_th1 = 72'b0;
// end

bw_r_irf_72_2x1_mux mux2_1(
	.sel(rd_thread),
	.x0(reg_th0),
	.x1(reg_th1),
	.y(rd_data)
	);

  always @(negedge clk) begin
    rd_addr = restore_addr;
  end

  wire [71:0] restore_data = window[{(rd_addr[3:2]>>SHIFT_ADDR),rd_addr[1:0]}];

  always @(posedge clk) begin
    wr_addr <= save_addr;
  end
  always @(posedge clk) begin
    save_d <= save;
  end

  wire [71:0] save_data;

  bw_r_irf_72_2x1_mux mux2_2(
        .sel(wr_addr[3]),
        .x0(reg_th0),
        .x1(reg_th1),
        .y(save_data)
        );

  always @(negedge clk) begin
    if(save_d) window[{(wr_addr[3:2]>>SHIFT_ADDR),wr_addr[1:0]}] <= save_data;
  end

//Register implementation for 2 threads / 2 write & 1 restore port

  wire [1:0] restores = (1'b1 << rd_addr[3]) & {2{restore}};
  //wire [1:0] wren1s = (1'b1 << wr1_th) & {2{wren1}};
  //wire [1:0] wren2s = (1'b1 << wr2_th) & {2{wren2}};

  wire [71:0] wrdata0, wrdata1;

  bw_r_irf_72_2x1_mux mux2_5(
        .sel(restores[0]),
        .x0(wr_data0),
        .x1(restore_data),
        .y(wrdata0)
        );

  bw_r_irf_72_2x1_mux mux2_6(
        .sel(restores[1]),
        .x0(wr_data1),
        .x1(restore_data),
        .y(wrdata1)
        );

  //wire [1:0] wr_en = wren1s | wren2s | (restores & {2{(wr_addr[3:0] != rd_addr[3:0])}});
  wire [1:0] wr_en = wrens | (restores & {2{(wr_addr[3:0] != rd_addr[3:0])}});

  //144 Flops
  always @(posedge clk) begin
    if(!reset_l)
    begin
      reg_th0 <= 72'd0;
      reg_th1 <= 72'd0;
    end
    else
    begin
      if(wr_en[0]) reg_th0 <= wrdata0;
      if(wr_en[1]) reg_th1 <= wrdata1;
    end
  end
    
endmodule

























































































































































module bw_r_irf_72_2x1_mux(sel, y, x0, x1);
	input		sel;
	input	[71:0]	x0;
	input	[71:0]	x1;
	output	[71:0] y;
	reg	[71:0] y;

	always @(sel or x0 or x1)
		case(sel)
		  1'b0: y = x0;
		  1'b1: y = x1;
		endcase

endmodule
	

// Copyright (c) 2015 Princeton University
// All rights reserved.
// 
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of Princeton University nor the
//       names of its contributors may be used to endorse or promote products
//       derived from this software without specific prior written permission.
// 
// THIS SOFTWARE IS PROVIDED BY PRINCETON UNIVERSITY "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL PRINCETON UNIVERSITY BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

// Wraps bw_r_irf_register with specific parameteres


module bw_r_irf_register16
(
    input               clk,
    input               reset_l,
    input   [1:0]       wrens,
    input               save,
    input   [3:0]       save_addr,
    input               restore,
    input   [3:0]       restore_addr,
    input   [71:0]      wr_data0,
    input   [71:0]      wr_data1,
    input               rd_thread,
    output  [71:0]      rd_data
);















    bw_r_irf_register bw_r_irf_register
    (
        .clk                (clk),
        .reset_l            (reset_l),
        .wrens              (wrens),
        .save               (save),
        .save_addr          (save_addr),
        .restore            (restore),
        .restore_addr       (restore_addr),
        .wr_data0           (wr_data0),
        .wr_data1           (wr_data1),
        .rd_thread          (rd_thread),
        .rd_data            (rd_data)
    );


endmodule
// Copyright (c) 2015 Princeton University
// All rights reserved.
// 
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of Princeton University nor the
//       names of its contributors may be used to endorse or promote products
//       derived from this software without specific prior written permission.
// 
// THIS SOFTWARE IS PROVIDED BY PRINCETON UNIVERSITY "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL PRINCETON UNIVERSITY BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

// Wraps bw_r_irf_register with specific parameteres


module bw_r_irf_register8
(
    input               clk,
    input               reset_l,
    input   [1:0]       wrens,
    input               save,
    input   [3:0]       save_addr,
    input               restore,
    input   [3:0]       restore_addr,
    input   [71:0]      wr_data0,
    input   [71:0]      wr_data1,
    input               rd_thread,
    output  [71:0]      rd_data
);















    bw_r_irf_register #(.NUM_WINDOW_ENTRIES(8), .SHIFT_ADDR(1)) bw_r_irf_register
    (
        .clk                (clk),
        .reset_l            (reset_l),
        .wrens              (wrens),
        .save               (save),
        .save_addr          (save_addr),
        .restore            (restore),
        .restore_addr       (restore_addr),
        .wr_data0           (wr_data0),
        .wr_data1           (wr_data1),
        .rd_thread          (rd_thread),
        .rd_data            (rd_data)
    );


endmodule
/*
Copyright (c) 2015 Princeton University
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of Princeton University nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY PRINCETON UNIVERSITY "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL PRINCETON UNIVERSITY BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

//==================================================================================================
//  Filename      : clk_gating_latch.v
//  Created On    : 2015-01-26 14:10:43
//  Last Modified : 2019-04-17 11:56:55
//  Revision      :
//  Author        : Tri Nguyen
//  Company       : Princeton University
//  Email         : trin@princeton.edu
//
//  Description   : Latch for glitchless clock gating
//==================================================================================================

module clk_gating_latch (
    input wire clk,
    input wire clk_en,
    output wire clk_out
);

// use clock buffer on FPGA
// note that not all FPGAs have enough of these available
// so we use the latch as a fallback on certain boards (e.g., vc707)




 // PITON_FPGA_SYNTH

  wire clk_en_sync;
  reg clk_en_sync_latch;

  assign clk_out = clk & clk_en_sync_latch;

  synchronizer sync(
      .clk            (clk),
      .presyncdata    (clk_en),
      .syncdata       (clk_en_sync)
  );

  // if possible, replace this with a native clock gate from the std cell lib
  // clk_en_sync_latch changes only on the negative duty of the cycle
  always @ (clk or clk_en_sync)
      if (~clk) clk_en_sync_latch = clk_en_sync;



endmodule // clk_gating_latch
// Copyright (c) 2015 Princeton University
// All rights reserved.
// 
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of Princeton University nor the
//       names of its contributors may be used to endorse or promote products
//       derived from this software without specific prior written permission.
// 
// THIS SOFTWARE IS PROVIDED BY PRINCETON UNIVERSITY "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL PRINCETON UNIVERSITY BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

/****************************************************************************
 *
 *   FILE: credit_to_valrdy.v
 *
 *   Modified: Yaosheng Fu
 *   Date: May 2 2014

 ***************************************************************************/

/*
Copyright (c) 2015 Princeton University
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of Princeton University nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY PRINCETON UNIVERSITY "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL PRINCETON UNIVERSITY BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

/////////////////////////////////////////////////////////////////////////////////////////////
// 63         50 49      42 41      34 33           30 29      22 21                 0   
// ------------------------------------------------------------------------------------
// |            |          |          |               |          |                    |
// |  Chip ID   |  Dest X  |  Dest Y  |  Final Route  |  Length  |    Header Payload  | 
// |            |          |          |               |          |                    |
// ------------------------------------------------------------------------------------
////////////////////////////////////////////////////////////////////////////////////////////////











 //whether the routing is based on chipid or x y position
 //`define    ROUTING_CHIP_ID
 

 //defines for different topology, only one should be active
 //`define    NETWORK_TOPO_2D_MESH
 //`define    NETWORK_TOPO_3D_MESH
 

module credit_to_valrdy (
   clk,
   reset,
   //credit based interface	
   data_in,
   valid_in,
   yummy_in,
            
   //val/rdy interface
   data_out,
   valid_out,
   ready_out
);

   input	 clk;
   input	 reset;
   input [64-1:0]	 data_in;
   input	 valid_in;
   input     ready_out;
    
   output	 yummy_in;
   output	 valid_out;
   output [64-1:0] data_out;
   
   wire	 thanksIn;

   wire valid_out_temp;

   assign valid_out = valid_out_temp;

   network_input_blk_multi_out #(.LOG2_NUMBER_FIFO_ELEMENTS(2)) data(
      .clk(clk),
      .reset(reset),
      .data_in(data_in),
      .valid_in(valid_in),

      .thanks_in(valid_out & ready_out),

      .yummy_out(yummy_in),
      .data_val(data_out),
      .data_val1(/*not used*/),
      .data_avail(valid_out_temp));

endmodule



// Modified by Princeton University on June 9th, 2015
// ========== Copyright Header Begin ==========================================
// 
// OpenSPARC T1 Processor File: m1.behV
// Copyright (c) 2006 Sun Microsystems, Inc.  All Rights Reserved.
// DO NOT ALTER OR REMOVE COPYRIGHT NOTICES.
// 
// The above named program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public
// License version 2 as published by the Free Software Foundation.
// 
// The above named program is distributed in the hope that it will be 
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// General Public License for more details.
// 
// You should have received a copy of the GNU General Public
// License along with this work; if not, write to the Free Software
// Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA 02110-1301, USA.
// 
// ========== Copyright Header End ============================================
////////////////////////////////////////////////////////////////////////
// 64 bit nor gate with first 32 bits out

module zznor64_32 ( znor64, znor32, a );
  input  [63:0] a;
  output        znor64;
  output        znor32;

  assign znor32 =  ~(a[0]  | a[1]  | a[2]  | a[3]  | a[4]  | a[5]  | a[6]  | a[7]
		   | a[8]  | a[9]  | a[10] | a[11] | a[12] | a[13] | a[14] | a[15]
		   | a[16] | a[17] | a[18] | a[19] | a[20] | a[21] | a[22] | a[23]
		   | a[24] | a[25] | a[26] | a[27] | a[28] | a[29] | a[30] | a[31]); 

  assign znor64 =  ~(a[0]  | a[1]  | a[2]  | a[3]  | a[4]  | a[5]  | a[6]  | a[7]
		   | a[8]  | a[9]  | a[10] | a[11] | a[12] | a[13] | a[14] | a[15]
		   | a[16] | a[17] | a[18] | a[19] | a[20] | a[21] | a[22] | a[23]
		   | a[24] | a[25] | a[26] | a[27] | a[28] | a[29] | a[30] | a[31] 
		   | a[32] | a[33] | a[34] | a[35] | a[36] | a[37] | a[38] | a[39] 
		   | a[40] | a[41] | a[42] | a[43] | a[44] | a[45] | a[46] | a[47] 
		   | a[48] | a[49] | a[50] | a[51] | a[52] | a[53] | a[54] | a[55] 
		   | a[56] | a[57] | a[58] | a[59] | a[60] | a[61] | a[62] | a[63]);

endmodule // zznor64_32



////////////////////////////////////////////////////////////////////////////////
// 36 bit or gate

module zzor36 ( z, a );
  input  [35:0] a;
  output        z;

  assign z =  (a[0]  | a[1]  | a[2]  | a[3]  | a[4]  | a[5]  | a[6]  | a[7]
	     | a[8]  | a[9]  | a[10] | a[11] | a[12] | a[13] | a[14] | a[15]
	     | a[16] | a[17] | a[18] | a[19] | a[20] | a[21] | a[22] | a[23]
	     | a[24] | a[25] | a[26] | a[27] | a[28] | a[29] | a[30] | a[31]
	     | a[32] | a[33] | a[34] | a[35]); 
   
endmodule // zzor36



////////////////////////////////////////////////////////////////////////////////
// 32 bit or gate

module zzor32 ( z, a );
  input  [31:0] a;
  output        z;

  assign z =  (a[0]  | a[1]  | a[2]  | a[3]  | a[4]  | a[5]  | a[6]  | a[7]
	     | a[8]  | a[9]  | a[10] | a[11] | a[12] | a[13] | a[14] | a[15]
	     | a[16] | a[17] | a[18] | a[19] | a[20] | a[21] | a[22] | a[23]
	     | a[24] | a[25] | a[26] | a[27] | a[28] | a[29] | a[30] | a[31]); 

endmodule // zzor32



////////////////////////////////////////////////////////////////////////////////
// 24 bit nor gate

module zznor24 ( z, a );
  input  [23:0] a;
  output        z;

  assign z =  ~(a[0]  | a[1]  | a[2]  | a[3]  | a[4]  | a[5]  | a[6]  | a[7]
	      | a[8]  | a[9]  | a[10] | a[11] | a[12] | a[13] | a[14] | a[15]
	      | a[16] | a[17] | a[18] | a[19] | a[20] | a[21] | a[22] | a[23]); 

endmodule // zznor24



////////////////////////////////////////////////////////////////////////////////
// 16 bit nor gate

module zznor16 ( z, a );
  input  [15:0] a;
  output        z;

  assign z =  ~(a[0] | a[1] | a[2]  | a[3]  | a[4]  | a[5]  | a[6]  | a[7]
	      | a[8] | a[9] | a[10] | a[11] | a[12] | a[13] | a[14] | a[15]); 

endmodule // zznor16



////////////////////////////////////////////////////////////////////////////////
// 8 bit or gate

module zzor8 ( z, a );
  input  [7:0] a;
  output       z;

  assign z =  (a[0] | a[1] | a[2] | a[3] | a[4] | a[5] | a[6] | a[7]); 
   
endmodule // zzor8




////////////////////////////////////////////////////////////////////////////////
//  Description:	This block implements the adder for the sparc FPU.
//  			It takes two operands and a carry bit.  It adds them together
//			and sends the output to adder_out. 

module zzadd13 ( rs1_data, rs2_data, cin, adder_out );

  input  [12:0] rs1_data;   // 1st input operand
  input  [12:0] rs2_data;   // 2nd input operand
  input         cin;        // carry in

  output [12:0] adder_out;  // result of adder

  assign adder_out = rs1_data + rs2_data + cin;

endmodule // zzadd13



////////////////////////////////////////////////////////////////////////////////
//  Description:	This block implements the adder for the sparc FPU.
//  			It takes two operands and a carry bit.  It adds them together
//			and sends the output to adder_out. 

module zzadd56 ( rs1_data, rs2_data, cin, adder_out );

  input  [55:0] rs1_data;   // 1st input operand
  input  [55:0] rs2_data;   // 2nd input operand
  input         cin;        // carry in

  output [55:0] adder_out;  // result of adder

  assign adder_out = rs1_data + rs2_data + cin;

endmodule // zzadd56



////////////////////////////////////////////////////////////////////////////////

module zzadd48 ( rs1_data, rs2_data, cin, adder_out );

  input  [47:0] rs1_data;   // 1st input operand
  input  [47:0] rs2_data;   // 2nd input operand
  input         cin;        // carry in

  output [47:0] adder_out;  // result of adder

  assign adder_out = rs1_data + rs2_data + cin;

endmodule // zzadd48



////////////////////////////////////////////////////////////////////////////////
//  This adder is primarily used in the multiplier.
//  The cin to out path is optimized.

module zzadd34c ( rs1_data, rs2_data, cin, adder_out );

  input  [33:0] rs1_data;
  input  [33:0] rs2_data;
  input         cin;

  output [33:0] adder_out;

  assign adder_out = rs1_data + rs2_data + cin;


endmodule // zzadd34c



////////////////////////////////////////////////////////////////////////////////

module zzadd32 ( rs1_data, rs2_data, cin, adder_out, cout );

  input  [31:0] rs1_data;   // 1st input operand
  input  [31:0] rs2_data;   // 2nd input operand
  input         cin;        // carry in

  output [31:0] adder_out;  // result of adder
  output 	cout;       // carry out

  assign {cout, adder_out} = rs1_data + rs2_data + cin;

endmodule // zzadd32



////////////////////////////////////////////////////////////////////////////////

module zzadd18 ( rs1_data, rs2_data, cin, adder_out, cout );

  input  [17:0] rs1_data;   // 1st input operand
  input  [17:0] rs2_data;   // 2nd input operand
  input         cin;        // carry in

  output [17:0] adder_out;  // result of adder
  output 	cout;       // carry out

  assign {cout, adder_out} = rs1_data + rs2_data + cin;

endmodule // zzadd18



////////////////////////////////////////////////////////////////////////////////

module zzadd8 ( rs1_data, rs2_data, cin, adder_out, cout );

  input  [7:0] rs1_data;   // 1st input operand
  input  [7:0] rs2_data;   // 2nd input operand
  input        cin;        // carry in

  output [7:0] adder_out;  // result of add & decrement
  output       cout;       // carry out

  assign {cout, adder_out} = rs1_data + rs2_data + cin;

endmodule // zzadd8



////////////////////////////////////////////////////////////////////////////////
// Special 4-operand 32b adder used in spu_shamd5
//  Description:        This block implements the 4-operand 32-bit adder for SPU
//			It takes four 32-bit operands. It add them together and
//			output the 32-bit results to adder_out. The overflow of
//			32th bit and higher will be ignored.

module zzadd32op4 ( rs1_data, rs2_data, rs3_data, rs4_data, adder_out );

  input  [31:0] rs1_data;   // 1st input operand
  input  [31:0] rs2_data;   // 2nd input operand
  input  [31:0] rs3_data;   // 3rd input operand
  input  [31:0] rs4_data;   // 4th input operand

  output [31:0] adder_out;  // result of add

  assign adder_out = rs1_data + rs2_data + rs3_data + rs4_data;

endmodule // zzadd32op4


////////////////////////////////////////////////////////////////////////////////
//  Description:	This block implements the adder for the sparc alu.
//  			It takes two operands and a carry bit.  It adds them together
//			and sends the output to adder_out.  It outputs the overflow
//			and carry condition codes for both 64 bit and 32 bit operations.

module zzadd64 ( rs1_data, rs2_data, cin, adder_out, cout32, cout64 );

   input [63:0]  rs1_data;   // 1st input operand
   input [63:0]  rs2_data;   // 2nd input operand
   input         cin;        // carry in

   output [63:0] adder_out;  // result of adder
   output        cout32;     // carry out from lower 32 bit add
   output        cout64;     // carry out from 64 bit add

   assign {cout32, adder_out[31:0]}  = rs1_data[31:0]  + rs2_data[31:0]  + cin;
   assign {cout64, adder_out[63:32]} = rs1_data[63:32] + rs2_data[63:32] + cout32;

endmodule // zzadd64



///////////////////////////////////////////////////////////////////////
/*
//      Description: This is the ffu VIS adder.  It can do either
//                              2 16 bit adds or 1 32 bit add.
*/

module zzadd32v (/*AUTOARG*/
   // Outputs
   z,
   // Inputs
   a, b, cin, add32
   ) ;
   input [31:0] a;
   input [31:0] b;
   input        cin;
   input        add32;

   output [31:0] z;

   wire          cout15; // carry out from lower 16 bit add
   wire          cin16; // carry in to the upper 16 bit add
   wire          cout31; // carry out from the upper 16 bit add

   assign        cin16 = (add32)? cout15: cin;

   assign      {cout15, z[15:0]} = a[15:0]+b[15:0]+ cin;
   assign      {cout31, z[31:16]} = a[31:16]+b[31:16]+ cin16;

endmodule // zzadd32v




////////////////////////////////////////////////////////////////////////////////
// 64-bit incrementer

module zzinc64 ( in, out );

  input  [63:0] in;

  output [63:0] out;   // result of increment

  assign out = in + 1'b1;

endmodule // zzinc64


////////////////////////////////////////////////////////////////////////////////
// 48-bit incrementer

module zzinc48 ( in, out, overflow );

  input  [47:0] in;

  output [47:0] out;      // result of increment
  output        overflow; // overflow

  assign out      = in + 1'b1;
  assign overflow = ~in[47] & out[47];

endmodule // zzinc48


////////////////////////////////////////////////////////////////////////////////
// 32-bit incrementer

module zzinc32 ( in, out );

  input  [31:0] in;

  output [31:0] out;   // result of increment

  assign out = in + 1'b1;

endmodule // zzinc32


////////////////////////////////////////////////////////////////////////////////

module zzecc_exu_chkecc2 ( q,ce, ue, ne, d, p, vld );
   input [63:0] d;
   input [7:0]  p;
   input        vld;
   output [6:0] q;
   output       ce,
                ue,
                ne;

   wire       parity;

   assign     ce = vld & parity;

   assign ue = vld & ~parity & (q[6] | q[5] | q[4] | q[3] | q[2] | q[1] | q[0]);

   assign ne = ~vld | ~(parity | q[6] | q[5] | q[4] | q[3] | q[2] | q[1] | q[0]);


   assign q[0] = d[0]  ^ d[1]  ^ d[3]  ^ d[4]  ^ d[6]  ^ d[8]  ^ d[10]
               ^ d[11] ^ d[13] ^ d[15] ^ d[17] ^ d[19] ^ d[21] ^ d[23]
               ^ d[25] ^ d[26] ^ d[28] ^ d[30] ^ d[32] ^ d[34] ^ d[36]
               ^ d[38] ^ d[40] ^ d[42] ^ d[44] ^ d[46] ^ d[48] ^ d[50]
               ^ d[52] ^ d[54] ^ d[56] ^ d[57] ^ d[59] ^ d[61] ^ d[63]
               ^ p[0]  ;

   assign q[1] = d[0]  ^ d[2]  ^ d[3]  ^ d[5]  ^ d[6]  ^ d[9]  ^ d[10]
               ^ d[12] ^ d[13] ^ d[16] ^ d[17] ^ d[20] ^ d[21] ^ d[24]
               ^ d[25] ^ d[27] ^ d[28] ^ d[31] ^ d[32] ^ d[35] ^ d[36]
               ^ d[39] ^ d[40] ^ d[43] ^ d[44] ^ d[47] ^ d[48] ^ d[51]
               ^ d[52] ^ d[55] ^ d[56] ^ d[58] ^ d[59] ^ d[62] ^ d[63]
               ^ p[1]  ;

   assign q[2] = d[1]  ^ d[2]  ^ d[3]  ^ d[7]  ^ d[8]  ^ d[9]  ^ d[10]
               ^ d[14] ^ d[15] ^ d[16] ^ d[17] ^ d[22] ^ d[23] ^ d[24]
               ^ d[25] ^ d[29] ^ d[30] ^ d[31] ^ d[32] ^ d[37] ^ d[38]
               ^ d[39] ^ d[40] ^ d[45] ^ d[46] ^ d[47] ^ d[48] ^ d[53]
               ^ d[54] ^ d[55] ^ d[56] ^ d[60] ^ d[61] ^ d[62] ^ d[63]
               ^ p[2]  ;

   assign q[3] = d[4]  ^ d[5]  ^ d[6]  ^ d[7]  ^ d[8]  ^ d[9]  ^ d[10]
               ^ d[18] ^ d[19] ^ d[20] ^ d[21] ^ d[22] ^ d[23] ^ d[24]
               ^ d[25] ^ d[33] ^ d[34] ^ d[35] ^ d[36] ^ d[37] ^ d[38]
               ^ d[39] ^ d[40] ^ d[49] ^ d[50] ^ d[51] ^ d[52] ^ d[53]
               ^ d[54] ^ d[55] ^ d[56] ^ p[3]  ;

   assign q[4] = d[11] ^ d[12] ^ d[13] ^ d[14] ^ d[15] ^ d[16] ^ d[17]
               ^ d[18] ^ d[19] ^ d[20] ^ d[21] ^ d[22] ^ d[23] ^ d[24]
               ^ d[25] ^ d[41] ^ d[42] ^ d[43] ^ d[44] ^ d[45] ^ d[46]
               ^ d[47] ^ d[48] ^ d[49] ^ d[50] ^ d[51] ^ d[52] ^ d[53]
               ^ d[54] ^ d[55] ^ d[56] ^ p[4]  ;

   assign q[5] = d[26] ^ d[27] ^ d[28] ^ d[29] ^ d[30] ^ d[31] ^ d[32]
               ^ d[33] ^ d[34] ^ d[35] ^ d[36] ^ d[37] ^ d[38] ^ d[39]
               ^ d[40] ^ d[41] ^ d[42] ^ d[43] ^ d[44] ^ d[45] ^ d[46]
               ^ d[47] ^ d[48] ^ d[49] ^ d[50] ^ d[51] ^ d[52] ^ d[53]
               ^ d[54] ^ d[55] ^ d[56] ^ p[5]  ;

   assign q[6] = d[57] ^ d[58] ^ d[59] ^ d[60] ^ d[61] ^ d[62] ^ d[63] ^ p[6] ;

   assign parity = d[0]  ^ d[1]  ^ d[2]  ^ d[3]  ^ d[4]  ^ d[5]  ^ d[6]  ^ d[7]
                 ^ d[8]  ^ d[9]  ^ d[10] ^ d[11] ^ d[12] ^ d[13] ^ d[14] ^ d[15]
                 ^ d[16] ^ d[17] ^ d[18] ^ d[19] ^ d[20] ^ d[21] ^ d[22] ^ d[23]
                 ^ d[24] ^ d[25] ^ d[26] ^ d[27] ^ d[28] ^ d[29] ^ d[30] ^ d[31]
                 ^ d[32] ^ d[33] ^ d[34] ^ d[35] ^ d[36] ^ d[37] ^ d[38] ^ d[39]
                 ^ d[40] ^ d[41] ^ d[42] ^ d[43] ^ d[44] ^ d[45] ^ d[46] ^ d[47]
                 ^ d[48] ^ d[49] ^ d[50] ^ d[51] ^ d[52] ^ d[53] ^ d[54] ^ d[55]
                 ^ d[56] ^ d[57] ^ d[58] ^ d[59] ^ d[60] ^ d[61] ^ d[62] ^ d[63]
                 ^ p[0]  ^ p[1]  ^ p[2]  ^ p[3]  ^ p[4]  ^ p[5]  ^ p[6]  ^ p[7];

endmodule // zzecc_exu_chkecc2



////////////////////////////////////////////////////////////////////////////////

module zzecc_sctag_24b_gen ( din, dout, parity ) ;

// Input Ports
input  [23:0] din ;

// Output Ports
output [23:0] dout ;
output [5:0]  parity ;

wire   [23:0] dout ;
wire   [5:0]  parity ;

// Local Reg and Wires
wire          p1 ;
wire          p2 ;
wire          p4 ;
wire          p8 ;
wire          p16 ;
wire          p30 ;


//----|--|--|--|--|--|--|--|--|--|--|--|--|--|--|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
//    |1 |2 |3 |4 |5 |6 |7 |8 |9 |10|11|12|13|14|15 |16 |17 |18 |19 |20 |21 |22 |23 |24 |25 |26 |27 |28 |29 |30 |
//    |P1|P2|D0|P4|D1|D2|D3|P8|D4|D5|D6|D7|D8|D9|D10|P16|D11|D12|D13|D14|D15|D16|D17|D18|D19|D20|D21|D22|D23|P30|
//----|--|--|--|--|--|--|--|--|--|--|--|--|--|--|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
//P1  |  |  |* |  |* |  |* |  |* |  |* |  |* |  | * |   | * |   | * |   | * |   | * |   | * |   | * |   | * |   |
//P2  |  |  |* |  |  |* |* |  |  |* |* |  |  |* | * |   |   | * | * |   |   | * | * |   |   | * | * |   |   |   |
//P4  |  |  |  |  |* |* |* |  |  |  |  |* |* |* | * |   |   |   |   | * | * | * | * |   |   |   |   | * | * |   |
//P8  |  |  |  |  |  |  |  |  |* |* |* |* |* |* | * |   |   |   |   |   |   |   |   | * | * | * | * | * | * |   |
//P16 |  |  |  |  |  |  |  |  |  |  |  |  |  |  |   |   | * | * | * | * | * | * | * | * | * | * | * | * | * |   |
//----|--|--|--|--|--|--|--|--|--|--|--|--|--|--|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
//p30 |  |  |* |  |* |* |  |  |* |* |  |* |  |  | * |   | * | * |   | * |   |   | * | * |   |   | * |   | * |   |
//----|--|--|--|--|--|--|--|--|--|--|--|--|--|--|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|


assign p1  = din[0]  ^ din[1]  ^ din[3]  ^ din[4]  ^ din[6]  ^ din[8]  ^
             din[10] ^ din[11] ^ din[13] ^ din[15] ^ din[17] ^ din[19] ^
             din[21] ^ din[23] ;

assign p2  = din[0]  ^ din[2]  ^ din[3]  ^ din[5]  ^ din[6]  ^ din[9]  ^
             din[10] ^ din[12] ^ din[13] ^ din[16] ^ din[17] ^ din[20] ^
             din[21] ;

assign p4  = din[1]  ^ din[2]  ^ din[3]  ^ din[7]  ^ din[8]  ^ din[9]  ^
             din[10] ^ din[14] ^ din[15] ^ din[16] ^ din[17] ^ din[22] ^
             din[23] ;

assign p8  = din[4]  ^ din[5]  ^ din[6]  ^ din[7]  ^ din[8]  ^ din[9]  ^
             din[10] ^ din[18] ^ din[19] ^ din[20] ^ din[21] ^ din[22] ^
             din[23] ;

assign p16 = din[11] ^ din[12] ^ din[13] ^ din[14] ^ din[15] ^ din[16] ^
             din[17] ^ din[18] ^ din[19] ^ din[20] ^ din[21] ^ din[22] ^
             din[23] ;

assign p30 = din[0]  ^ din[1]  ^ din[2]  ^ din[4]  ^ din[5]  ^
             din[7]  ^ din[10] ^ din[11] ^ din[12] ^ din[14] ^
             din[17] ^ din[18] ^ din[21] ^ din[23] ;

assign dout   = din ;
assign parity = {p30, p16, p8, p4, p2, p1} ;

endmodule



////////////////////////////////////////////////////////////////////////////////

module zzecc_sctag_30b_cor ( din, parity, dout, corrected_bit ) ;

// Input Ports
input  [23:0] din ;
input  [4:0]  parity ;

// Output Ports
output [23:0] dout ;
output [4:0]  corrected_bit ;

wire   [23:0] dout ;
wire   [4:0]  corrected_bit ;

// Local Reg and Wires
wire          p1 ;
wire          p2 ;
wire          p4 ;
wire          p8 ;
wire          p16 ;
wire [23:0]   error_bit ;


//----|--|--|--|--|--|--|--|--|--|--|--|--|--|--|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
//    |1 |2 |3 |4 |5 |6 |7 |8 |9 |10|11|12|13|14|15 |16 |17 |18 |19 |20 |21 |22 |23 |24 |25 |26 |27 |28 |29 |30 |
//    |P1|P2|D0|P4|D1|D2|D3|P8|D4|D5|D6|D7|D8|D9|D10|P16|D11|D12|D13|D14|D15|D16|D17|D18|D19|D20|D21|D22|D23|P30|
//----|--|--|--|--|--|--|--|--|--|--|--|--|--|--|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
//P1  |* |  |* |  |* |  |* |  |* |  |* |  |* |  | * |   | * |   | * |   | * |   | * |   | * |   | * |   | * |   |
//P2  |  |* |* |  |  |* |* |  |  |* |* |  |  |* | * |   |   | * | * |   |   | * | * |   |   | * | * |   |   |   |
//P4  |  |  |  |* |* |* |* |  |  |  |  |* |* |* | * |   |   |   |   | * | * | * | * |   |   |   |   | * | * |   |
//P8  |  |  |  |  |  |  |  |* |* |* |* |* |* |* | * |   |   |   |   |   |   |   |   | * | * | * | * | * | * |   |
//P16 |  |  |  |  |  |  |  |  |  |  |  |  |  |  |   | * | * | * | * | * | * | * | * | * | * | * | * | * | * |   |
//----|--|--|--|--|--|--|--|--|--|--|--|--|--|--|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
//p30 |* |* |* |* |* |* |* |* |* |* |* |* |* |* | * | * | * | * | * | * | * | * | * | * | * | * | * | * | * | * |
//----|--|--|--|--|--|--|--|--|--|--|--|--|--|--|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|


assign p1  = parity[0] ^
             din[0]  ^ din[1]  ^ din[3]  ^ din[4]  ^ din[6]  ^ din[8]  ^
             din[10] ^ din[11] ^ din[13] ^ din[15] ^ din[17] ^ din[19] ^
             din[21] ^ din[23] ;

assign p2  = parity[1] ^
             din[0]  ^ din[2]  ^ din[3]  ^ din[5]  ^ din[6]  ^ din[9]  ^
             din[10] ^ din[12] ^ din[13] ^ din[16] ^ din[17] ^ din[20] ^
             din[21] ;

assign p4  = parity[2] ^
             din[1]  ^ din[2]  ^ din[3]  ^ din[7]  ^ din[8]  ^ din[9]  ^
             din[10] ^ din[14] ^ din[15] ^ din[16] ^ din[17] ^ din[22] ^
             din[23] ;

assign p8  = parity[3] ^
             din[4]  ^ din[5]  ^ din[6]  ^ din[7]  ^ din[8]  ^ din[9]  ^
             din[10] ^ din[18] ^ din[19] ^ din[20] ^ din[21] ^ din[22] ^
             din[23] ;

assign p16 = parity[4] ^
             din[11] ^ din[12] ^ din[13] ^ din[14] ^ din[15] ^ din[16] ^
             din[17] ^ din[18] ^ din[19] ^ din[20] ^ din[21] ^ din[22] ^
             din[23] ;

assign  error_bit[0]  = !p16 & !p8 & !p4 &  p2 &  p1 ; // 3
assign  error_bit[1]  = !p16 & !p8 &  p4 & !p2 &  p1 ; // 5
assign  error_bit[2]  = !p16 & !p8 &  p4 &  p2 & !p1 ; // 6
assign  error_bit[3]  = !p16 & !p8 &  p4 &  p2 &  p1 ; // 7
assign  error_bit[4]  = !p16 &  p8 & !p4 & !p2 &  p1 ; // 9
assign  error_bit[5]  = !p16 &  p8 & !p4 &  p2 & !p1 ; // 10
assign  error_bit[6]  = !p16 &  p8 & !p4 &  p2 &  p1 ; // 11
assign  error_bit[7]  = !p16 &  p8 &  p4 & !p2 & !p1 ; // 12
assign  error_bit[8]  = !p16 &  p8 &  p4 & !p2 &  p1 ; // 13
assign  error_bit[9]  = !p16 &  p8 &  p4 &  p2 & !p1 ; // 14
assign  error_bit[10] = !p16 &  p8 &  p4 &  p2 &  p1 ; // 15
assign  error_bit[11] =  p16 & !p8 & !p4 & !p2 &  p1 ; // 17
assign  error_bit[12] =  p16 & !p8 & !p4 &  p2 & !p1 ; // 18
assign  error_bit[13] =  p16 & !p8 & !p4 &  p2 &  p1 ; // 19
assign  error_bit[14] =  p16 & !p8 &  p4 & !p2 & !p1 ; // 20
assign  error_bit[15] =  p16 & !p8 &  p4 & !p2 &  p1 ; // 21
assign  error_bit[16] =  p16 & !p8 &  p4 &  p2 & !p1 ; // 22
assign  error_bit[17] =  p16 & !p8 &  p4 &  p2 &  p1 ; // 23
assign  error_bit[18] =  p16 &  p8 & !p4 & !p2 & !p1 ; // 24
assign  error_bit[19] =  p16 &  p8 & !p4 & !p2 &  p1 ; // 25
assign  error_bit[20] =  p16 &  p8 & !p4 &  p2 & !p1 ; // 26
assign  error_bit[21] =  p16 &  p8 & !p4 &  p2 &  p1 ; // 27
assign  error_bit[22] =  p16 &  p8 &  p4 & !p2 & !p1 ; // 28
assign  error_bit[23] =  p16 &  p8 &  p4 & !p2 &  p1 ; // 29

assign  dout          = din ^ error_bit ;
assign  corrected_bit = {p16, p8, p4, p2, p1} ;

endmodule



////////////////////////////////////////////////////////////////////////////////
//Module Name: zzecc_sctag_ecc39
//Function: Error Detection and Correction
//
//

module zzecc_sctag_ecc39 ( dout, cflag, pflag, parity, din);

   //Output: 32bit corrected data
   output[31:0] dout;
   output [5:0] cflag;
   output 	pflag;
   
   //Input: 32bit data din
   input [31:0] din;
   input [6:0]	parity;

   wire 	c0,c1,c2,c3,c4,c5;
   wire [31:0] 	err_bit_pos;

   //refer to the comments in parity_gen_32b.v for the position description
   
   assign c0= parity[0]^(din[0]^din[1])^(din[3]^din[4])^(din[6]^din[8])
                     ^(din[10]^din[11])^(din[13]^din[15])^(din[17]^din[19])
		     ^(din[21]^din[23])^(din[25]^din[26])^(din[28]^din[30]);
   
   assign c1= parity[1]^(din[0]^din[2])^(din[3]^din[5])^(din[6]^din[9])
                     ^(din[10]^din[12])^(din[13]^din[16])^(din[17]^din[20])
		     ^(din[21]^din[24])^(din[25]^din[27])^(din[28]^din[31]);
   
   assign c2= parity[2]^(din[1]^din[2])^(din[3]^din[7])^(din[8]^din[9])
                     ^(din[10]^din[14])^(din[15]^din[16])^(din[17]^din[22])
		     ^(din[23]^din[24])^(din[25]^din[29])^(din[30]^din[31]);
   
   assign c3= parity[3]^(din[4]^din[5])^(din[6]^din[7])^(din[8]^din[9])
                     ^(din[10]^din[18])^(din[19]^din[20])^(din[21]^din[22])
		     ^(din[23]^din[24])^din[25];
   
   assign c4= parity[4]^(din[11]^din[12])^(din[13]^din[14])^
                    (din[15]^din[16])^(din[17]^din[18])^(din[19]^din[20])^
                    (din[21]^din[22])^(din[23]^din[24])^din[25];

   assign c5= parity[5]^(din[26]^din[27])^(din[28]^din[29])^
		    (din[30]^din[31]);

   //generate total parity flag
   assign pflag= c0 ^
		(( (((parity[1]^parity[2])^(parity[3]^parity[4])) ^
		 ((parity[5]^parity[6])^(din[2]^din[5]))) ^		 
		 (((din[7]^din[9])^(din[12]^din[14])) ^
		 ((din[16]^din[18])^(din[20]^din[22]))) ) ^
		 ((din[24]^din[27])^(din[29]^din[31])) );
   
   assign cflag= {c5,c4,c3,c2,c1,c0};
   
   //6 to 32 decoder
   assign err_bit_pos[0] = (c0)&(c1)&(~c2)&(~c3)&(~c4)&(~c5);
   assign err_bit_pos[1] = (c0)&(~c1)&(c2)&(~c3)&(~c4)&(~c5);
   assign err_bit_pos[2] = (~c0)&(c1)&(c2)&(~c3)&(~c4)&(~c5);
   assign err_bit_pos[3] = (c0)&(c1)&(c2)&(~c3)&(~c4)&(~c5);
   assign err_bit_pos[4] = (c0)&(~c1)&(~c2)&(c3)&(~c4)&(~c5);
   assign err_bit_pos[5] = (~c0)&(c1)&(~c2)&(c3)&(~c4)&(~c5);
   assign err_bit_pos[6] = (c0)&(c1)&(~c2)&(c3)&(~c4)&(~c5);
   assign err_bit_pos[7] = (~c0)&(~c1)&(c2)&(c3)&(~c4)&(~c5);
   assign err_bit_pos[8] = (c0)&(~c1)&(c2)&(c3)&(~c4)&(~c5);
   assign err_bit_pos[9] = (~c0)&(c1)&(c2)&(c3)&(~c4)&(~c5);
   assign err_bit_pos[10] = (c0)&(c1)&(c2)&(c3)&(~c4)&(~c5);
   assign err_bit_pos[11] = (c0)&(~c1)&(~c2)&(~c3)&(c4)&(~c5);
   assign err_bit_pos[12] = (~c0)&(c1)&(~c2)&(~c3)&(c4)&(~c5);
   assign err_bit_pos[13] = (c0)&(c1)&(~c2)&(~c3)&(c4)&(~c5);
   assign err_bit_pos[14] = (~c0)&(~c1)&(c2)&(~c3)&(c4)&(~c5);
   assign err_bit_pos[15] = (c0)&(~c1)&(c2)&(~c3)&(c4)&(~c5);
   assign err_bit_pos[16] = (~c0)&(c1)&(c2)&(~c3)&(c4)&(~c5);
   assign err_bit_pos[17] = (c0)&(c1)&(c2)&(~c3)&(c4)&(~c5);
   assign err_bit_pos[18] = (~c0)&(~c1)&(~c2)&(c3)&(c4)&(~c5);
   assign err_bit_pos[19] = (c0)&(~c1)&(~c2)&(c3)&(c4)&(~c5);
   assign err_bit_pos[20] = (~c0)&(c1)&(~c2)&(c3)&(c4)&(~c5);
   assign err_bit_pos[21] = (c0)&(c1)&(~c2)&(c3)&(c4)&(~c5);
   assign err_bit_pos[22] = (~c0)&(~c1)&(c2)&(c3)&(c4)&(~c5);
   assign err_bit_pos[23] = (c0)&(~c1)&(c2)&(c3)&(c4)&(~c5);
   assign err_bit_pos[24] = (~c0)&(c1)&(c2)&(c3)&(c4)&(~c5);
   assign err_bit_pos[25] = (c0)&(c1)&(c2)&(c3)&(c4)&(~c5);
   assign err_bit_pos[26] = (c0)&(~c1)&(~c2)&(~c3)&(~c4)&(c5);
   assign err_bit_pos[27] = (~c0)&(c1)&(~c2)&(~c3)&(~c4)&(c5);
   assign err_bit_pos[28] = (c0)&(c1)&(~c2)&(~c3)&(~c4)&(c5);
   assign err_bit_pos[29] = (~c0)&(~c1)&(c2)&(~c3)&(~c4)&(c5);
   assign err_bit_pos[30] = (c0)&(~c1)&(c2)&(~c3)&(~c4)&(c5);
   assign err_bit_pos[31] = (~c0)&(c1)&(c2)&(~c3)&(~c4)&(c5);

   //correct the error bit, it can only correct one error bit.
   
   assign dout = din ^ err_bit_pos;

endmodule // zzecc_sctag_ecc39


////////////////////////////////////////////////////////////////////////////////
//Module Name: zzecc_sctag_pgen_32b
//Function: Generate 7 parity bits for 32bits input data
//

module zzecc_sctag_pgen_32b ( dout, parity, din);

   //Output: 32bit dout and 7bit parity bit
   output[31:0] dout;
   output [6:0] parity;

   //Input: 32bit data din
   input [31:0] din;

   //input data passing through this module
   assign dout = din ;

   //generate parity bits based on the hamming codes
   //the method to generate parity bit is shown as follows
   //1   2  3  4  5  6  7  8  9 10 11 12 13 14  15  16  17  18  19
   //P1 P2 d0 P4 d1 d2 d3 P8 d4 d5 d6 d7 d8 d9 d10 P16 d11 d12 d13 
   //
   // 20  21  22  23  24  25  26  27  28  29  30  31  32  33  34  35 
   //d14 d15 d16 d17 d18 d19 d20 d21 d22 d23 d24 d25 P32 d26 d27 d28
   //
   // 36  37  38       
   //d29 d30 d31
   //For binary numbers B1-B2-B3-B4-B5-B6:
   //B1=1 for (1,3,5,7,9,11,13,15,17,19,21,23,25,27,29,31,33,35,37,39,...)
   //B2=1 for (2,3,6,7,10,11,14,15,18,19,22,23,26,27,30,31,34,35,38,39...)
   //B3=1 for (4,5,6,7,12,13,14,15,20,21,22,23,28,29,30,31,36,37,38,39....)
   //B4=1 for (8,9,10,11,12,13,14,15,24,25,26,27,28,29,30,31,40,41,42,....)
   //B5=1 for (16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31,48,49,...)
   //B6=1 for (32,33,34,35,36,37,38,39,40,41,42,43,44,45,46,47,48,49...)
   //Parity bit P1,P2,P4,P8,P16,P32 can be generated from the above group of
   //bits B1=1,B2=1,B3=1,B4=1,B5=1,B6=1 respectively.

   //use parity[5:0] to stand for P1,P2,P4,P8,P16,P32
   assign parity[0] = (din[0]^din[1])^(din[3]^din[4])^(din[6]^din[8])
                     ^(din[10]^din[11])^(din[13]^din[15])^(din[17]^din[19])
		     ^(din[21]^din[23])^(din[25]^din[26])^(din[28]^din[30]);
   //
   assign parity[1] = (din[0]^din[2])^(din[3]^din[5])^(din[6]^din[9])
                     ^(din[10]^din[12])^(din[13]^din[16])^(din[17]^din[20])
		     ^(din[21]^din[24])^(din[25]^din[27])^(din[28]^din[31]);
   //
   assign parity[2] = (din[1]^din[2])^(din[3]^din[7])^(din[8]^din[9])
                     ^(din[10]^din[14])^(din[15]^din[16])^(din[17]^din[22])
		     ^(din[23]^din[24])^(din[25]^din[29])^(din[30]^din[31]);
   //
   assign parity[3] = (din[4]^din[5])^(din[6]^din[7])^(din[8]^din[9])
                     ^(din[10]^din[18])^(din[19]^din[20])^(din[21]^din[22])
		     ^(din[23]^din[24])^din[25];
   //
   assign parity[4] = (din[11]^din[12])^(din[13]^din[14])^(din[15]^din[16])
                     ^(din[17]^din[18])^(din[19]^din[20])^(din[21]^din[22])
		     ^(din[23]^din[24])^din[25];
   //
   assign parity[5] = (din[26]^din[27])^(din[28]^din[29])^(din[30]^din[31]);

   //the last parity bit is the xor of all 38bits
   //assign parity[6] = (^din)^(^parity[5:0]);
   //it can be further simplified as:
   //din= d0  d1  d2  d3  d4  d5  d6  d7  d8  d9 d10 d11 d12 d13 d14 d15 
   //p0 =  x   x       x   x       x       x       x   x       x       x
   //p1 =  x       x   x       x   x           x   x       x   x
   //p2 =      x   x   x               x   x   x   x               x   x
   //p3 =                  x   x   x   x   x   x   x  
   //p4 =                                              x   x   x   x   x
   //p5 =
   //-------------------------------------------------------------------
   //Total 3   3   3   4   3   3   4   3   4   4   5   3   3   4   3   4 
   //
   //din=d16 d17 d18 d19 d20 d21 d22 d23 d24 d25 d26 d27 d28 d29 d30 d31 
   //p0=       x       x       x       x       x   x       x       x    
   //p1=   x   x           x   x           x   x       x   x           x
   //p2=   x   x                   x   x   x   x               x   x   x
   //p3=           x   x   x   x   x   x   x   x
   //p4=   x   x   x   x   x   x   x   x   x   x
   //p5=                                           x   x   x   x   x   x
   //-------------------------------------------------------------------
   //total 4   5   3   4   4   5   4   5   5   6   3   3   4   3   4   4

   //so total=even number, the corresponding bit will not show up in the
   //final xor tree.
   assign parity[6] =  din[0] ^ din[1]  ^ din[2]  ^ din[4]  ^ din[5] ^ din[7]
		    ^ din[10] ^ din[11] ^ din[12] ^ din[14] ^ din[17]
		    ^ din[18] ^ din[21] ^ din[23] ^ din[24] ^ din[26]
		    ^ din[27] ^ din[29];
   
endmodule // zzecc_sctag_pgen_32b

////////////////////////////////////////////////////////////////////////////////
// 34 bit parity tree

module zzpar34 ( z, d );
   input  [33:0] d;
   output        z;

   assign  z =  d[0]  ^ d[1]  ^ d[2]  ^ d[3]  ^ d[4]  ^ d[5]  ^ d[6]  ^ d[7]
	      ^ d[8]  ^ d[9]  ^ d[10] ^ d[11] ^ d[12] ^ d[13] ^ d[14] ^ d[15]
	      ^ d[16] ^ d[17] ^ d[18] ^ d[19] ^ d[20] ^ d[21] ^ d[22] ^ d[23]
	      ^ d[24] ^ d[25] ^ d[26] ^ d[27] ^ d[28] ^ d[29] ^ d[30] ^ d[31]
	      ^ d[32] ^ d[33]; 

endmodule // zzpar34



////////////////////////////////////////////////////////////////////////////////
// 32 bit parity tree

module zzpar32 ( z, d );
   input  [31:0] d;
   output        z;

   assign  z =  d[0]  ^ d[1]  ^ d[2]  ^ d[3]  ^ d[4]  ^ d[5]  ^ d[6]  ^ d[7]
	      ^ d[8]  ^ d[9]  ^ d[10] ^ d[11] ^ d[12] ^ d[13] ^ d[14] ^ d[15]
	      ^ d[16] ^ d[17] ^ d[18] ^ d[19] ^ d[20] ^ d[21] ^ d[22] ^ d[23]
	      ^ d[24] ^ d[25] ^ d[26] ^ d[27] ^ d[28] ^ d[29] ^ d[30] ^ d[31]; 

endmodule // zzpar32



////////////////////////////////////////////////////////////////////////////////
// 28 bit parity tree

module zzpar28 ( z, d );
   input  [27:0] d;
   output        z;

   assign  z =  d[0]  ^ d[1]  ^ d[2]  ^ d[3]  ^ d[4]  ^ d[5]  ^ d[6]  ^ d[7]
	      ^ d[8]  ^ d[9]  ^ d[10] ^ d[11] ^ d[12] ^ d[13] ^ d[14] ^ d[15]
	      ^ d[16] ^ d[17] ^ d[18] ^ d[19] ^ d[20] ^ d[21] ^ d[22] ^ d[23]
	      ^ d[24] ^ d[25] ^ d[26] ^ d[27]; 

endmodule // zzpar28



////////////////////////////////////////////////////////////////////////////////
// 16 bit parity tree

module zzpar16 ( z, d );
   input  [15:0] d;
   output        z;

   assign z = d[0] ^ d[1] ^ d[2]  ^ d[3]  ^ d[4]  ^ d[5]  ^ d[6]  ^ d[7]
	    ^ d[8] ^ d[9] ^ d[10] ^ d[11] ^ d[12] ^ d[13] ^ d[14] ^ d[15]; 
   
endmodule // zzpar16



////////////////////////////////////////////////////////////////////////////////
// 8 bit parity tree

module zzpar8 ( z, d );
   input  [7:0] d;
   output       z;

   assign  z =  d[0] ^ d[1] ^ d[2] ^ d[3] ^ d[4] ^ d[5] ^ d[6] ^ d[7]; 

endmodule // zzpar8



////////////////////////////////////////////////////////////////////////////////
//    64 -> 6 priority encoder
//    Bit 63 has the highest priority

module zzpenc64 (/*AUTOARG*/
   // Outputs
   z, 
   // Inputs
  a 
   );

   input [63:0] a;
   output [5:0] z;

   integer      i;
   reg  [5:0]   z;

     always @ (a)
     begin
          z = 6'b0;
          for (i=0;i<64;i=i+1)
               if (a[i])
                      z = i;
     end

endmodule // zzpenc64

////////////////////////////////////////////////////////////////////////////////
//    4-bit 60x buffers

module zzbufh_60x4 (/*AUTOARG*/
   // Outputs
   z,
   // Inputs
  a
   );

   input [3:0] a;
   output [3:0] z;

   assign z = a;

endmodule //zzbufh_60x4

// LVT modules added below

module zzadd64_lv ( rs1_data, rs2_data, cin, adder_out, cout32, cout64 );

   input [63:0]  rs1_data;   // 1st input operand
   input [63:0]  rs2_data;   // 2nd input operand
   input         cin;        // carry in

   output [63:0] adder_out;  // result of adder
   output        cout32;     // carry out from lower 32 bit add
   output        cout64;     // carry out from 64 bit add

   assign {cout32, adder_out[31:0]}  = rs1_data[31:0]  + rs2_data[31:0]  + cin;
   assign {cout64, adder_out[63:32]} = rs1_data[63:32] + rs2_data[63:32] + cout32;

endmodule // zzadd64_lv

module zzpar8_lv ( z, d );
   input  [7:0] d;
   output       z;

   assign  z =  d[0] ^ d[1] ^ d[2] ^ d[3] ^ d[4] ^ d[5] ^ d[6] ^ d[7]; 

endmodule // zzpar8_lv


module zzpar32_lv ( z, d );
   input  [31:0] d;
   output        z;

   assign  z =  d[0]  ^ d[1]  ^ d[2]  ^ d[3]  ^ d[4]  ^ d[5]  ^ d[6]  ^ d[7]
              ^ d[8]  ^ d[9]  ^ d[10] ^ d[11] ^ d[12] ^ d[13] ^ d[14] ^ d[15]
              ^ d[16] ^ d[17] ^ d[18] ^ d[19] ^ d[20] ^ d[21] ^ d[22] ^ d[23]
              ^ d[24] ^ d[25] ^ d[26] ^ d[27] ^ d[28] ^ d[29] ^ d[30] ^ d[31];

endmodule // zzpar32_lv



module zznor64_32_lv ( znor64, znor32, a );
  input  [63:0] a;
  output        znor64;
  output        znor32;

  assign znor32 =  ~(a[0]  | a[1]  | a[2]  | a[3]  | a[4]  | a[5]  | a[6]  | a[7]
		   | a[8]  | a[9]  | a[10] | a[11] | a[12] | a[13] | a[14] | a[15]
		   | a[16] | a[17] | a[18] | a[19] | a[20] | a[21] | a[22] | a[23]
		   | a[24] | a[25] | a[26] | a[27] | a[28] | a[29] | a[30] | a[31]); 

  assign znor64 =  ~(a[0]  | a[1]  | a[2]  | a[3]  | a[4]  | a[5]  | a[6]  | a[7]
		   | a[8]  | a[9]  | a[10] | a[11] | a[12] | a[13] | a[14] | a[15]
		   | a[16] | a[17] | a[18] | a[19] | a[20] | a[21] | a[22] | a[23]
		   | a[24] | a[25] | a[26] | a[27] | a[28] | a[29] | a[30] | a[31] 
		   | a[32] | a[33] | a[34] | a[35] | a[36] | a[37] | a[38] | a[39] 
		   | a[40] | a[41] | a[42] | a[43] | a[44] | a[45] | a[46] | a[47] 
		   | a[48] | a[49] | a[50] | a[51] | a[52] | a[53] | a[54] | a[55] 
		   | a[56] | a[57] | a[58] | a[59] | a[60] | a[61] | a[62] | a[63]);

endmodule // zznor64_32_lv

////////////////////////////////////////////////////////////////////////////////
//    64 -> 6 priority encoder
//    Bit 63 has the highest priority
//    LVT version

module zzpenc64_lv (/*AUTOARG*/
   // Outputs
   z,
   // Inputs
  a
   );

   input [63:0] a;
   output [5:0] z;

   integer      i;
   reg  [5:0]   z;

     always @ (a)
     begin
          z = 6'b0;
          for (i=0;i<64;i=i+1)
               if (a[i])
                      z = i;
     end

endmodule // zzpenc64_lv

////////////////////////////////////////////////////////////////////////////////
// 36 bit or gate
// LVT version

module zzor36_lv ( z, a );
  input  [35:0] a;
  output        z;

  assign z =  (a[0]  | a[1]  | a[2]  | a[3]  | a[4]  | a[5]  | a[6]  | a[7]
             | a[8]  | a[9]  | a[10] | a[11] | a[12] | a[13] | a[14] | a[15]
             | a[16] | a[17] | a[18] | a[19] | a[20] | a[21] | a[22] | a[23]
             | a[24] | a[25] | a[26] | a[27] | a[28] | a[29] | a[30] | a[31]
             | a[32] | a[33] | a[34] | a[35]);

endmodule // zzor36_lv

////////////////////////////////////////////////////////////////////////////////
// 34 bit parity tree
// LVT version

module zzpar34_lv ( z, d );
   input  [33:0] d;
   output        z;

   assign  z =  d[0]  ^ d[1]  ^ d[2]  ^ d[3]  ^ d[4]  ^ d[5]  ^ d[6]  ^ d[7]
              ^ d[8]  ^ d[9]  ^ d[10] ^ d[11] ^ d[12] ^ d[13] ^ d[14] ^ d[15]
              ^ d[16] ^ d[17] ^ d[18] ^ d[19] ^ d[20] ^ d[21] ^ d[22] ^ d[23]
              ^ d[24] ^ d[25] ^ d[26] ^ d[27] ^ d[28] ^ d[29] ^ d[30] ^ d[31]
              ^ d[32] ^ d[33];

endmodule // zzpar34_lv


// Modified by Princeton University on June 9th, 2015
// ========== Copyright Header Begin ==========================================
// 
// OpenSPARC T1 Processor File: mul64.v
// Copyright (c) 2006 Sun Microsystems, Inc.  All Rights Reserved.
// DO NOT ALTER OR REMOVE COPYRIGHT NOTICES.
// 
// The above named program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public
// License version 2 as published by the Free Software Foundation.
// 
// The above named program is distributed in the hope that it will be 
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// General Public License for more details.
// 
// You should have received a copy of the GNU General Public
// License along with this work; if not, write to the Free Software
// Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA 02110-1301, USA.
// 
// ========== Copyright Header End ============================================
/*//////////////////////////////////////////////////////////////////////
//
//  Module Name: mul64
//  Description:        *This block implements the multiplier used in the modular multiplier
//                       unit (MUL) and be shared by sparc EXU and the streaming unit (SPU).
//                       It is also used as the 54x54 multiplier in the FPU.
//                      *It takes two 64-bit unsign data and accumulated operand and do the
//                       64x64 MAC operation at two cycle thruput and 5 cycle latency.
//                      *The mul_valid signal indicate the beginning of a new operation.
//                       It MUST be dis-asserted at the next cycle to have the proper 2-cycle
//                       latency operation in the csa array. If there are two back-to-back
//                       cycle operation, the first operation result will be incorrect.
//                      *Results are avaliable on the 5th cycle of the mul_valid as shows
//
//			*Following inputs should tie to "0" when used as a 64x64 multiplier
//			 - areg 
//			 - accreg 
//			 - x2
//
//                         Cycle-0  | Cycle-1 | Cycle-2 | Cycle-3 | Cycle-4 | Cycle-5
//                       1st        *         |         |         |         |
//                       rs1, rs2   ^         |         |         |         | 1st results
//                       valid=1    | valid=0 |         *         |         | avaliable
//                                1st         | 2nd OP  ^         |         |
//                                setup       | valid=1 |         |         |
//                                            |        2nd        |         |
//                                            |       setup       |         |
//
*/

//PITON_PROTO enables all FPGA related modifications



























































module mul64 (rs1_l, rs2, valid, areg, accreg, x2, out, rclk, si, so, se, mul_rst_l, mul_step);

input  [63:0]  	rs1_l;			// op1
input  [63:0]  	rs2;			// op2
input	       	valid;			// begin of the MUL operation
input  [96:0]  	areg;			// accumulated input for ACCUM
input  [135:129] accreg;		// direct input from ACCUM [135:129]
input	       	x2;			// for op1*op2*2
input	       	rclk, si, se, mul_rst_l, mul_step;
output  	so;
output [135:0] 	out;

wire	       	cyc1, cyc2, cyc3;	// cycle stage of MUL
wire [2:0]	b0, b1, b2,  b3,  b4,  b5,  b6,  b7;
wire [2:0]	b8, b9, b10, b11, b12, b13, b14, b15;
wire	    	b16;
wire [63:0]	op1_l, op1;
wire [81:0]	a0sum, a1sum, a0s, a1s; 
wire [81:4]	a0cout, a1cout, a0c, a1c;
wire		pcoutx2, psumx2;
wire 		x2_c1, x2_c2, x2_c3, x2_c2c3;

wire [98:0]	psum, pcout;
wire [98:30]	pcout_in, pc;
wire [98:31]	psum_in, ps;
wire [96:0]	ary2_cout, addin_cout;
wire [97:0]	ary2_sum,  addin_sum ;
wire		add_cin, addin_cin, add_co31, add_co96;
wire [103:0]	addout;
wire		clk_enb0, clk_enb1;
wire 		rst;
wire		clk;
wire		tm_l;

  assign clk = rclk;
  assign rst = ~mul_rst_l; 
  assign tm_l = ~se;

  clken_buf	ckbuf_0(.clk(clk_enb0), .rclk(clk), .enb_l(~mul_step), .tmb_l(tm_l));

  /////////////////////////////////////////////////////////////////////
  // 	States count
  /////////////////////////////////////////////////////////////////////
  dffr_s  cyc1_dff(.din(valid), .clk(clk_enb0), .q(cyc1), .rst(rst), .se(se), .si(), .so());
  dffr_s  cyc2_dff(.din(cyc1),  .clk(clk_enb0), .q(cyc2), .rst(rst), .se(se), .si(), .so());
  dffr_s  cyc3_dff(.din(cyc2),  .clk(clk_enb0), .q(cyc3), .rst(rst), .se(se), .si(), .so());
  dffr_s  x2c1_dff(.din(x2),    .clk(clk_enb0), .q(x2_c1), .rst(rst), .se(se), .si(), .so());
  dffr_s  x2c2_dff(.din(x2_c1), .clk(clk_enb0), .q(x2_c2), .rst(rst), .se(se), .si(), .so());
  dffr_s  x2c3_dff(.din(x2_c2), .clk(clk_enb0), .q(x2_c3), .rst(rst), .se(se), .si(), .so());

  assign x2_c2c3 =  x2_c2 | x2_c3 ;
	
  /////////////////////////////////////////////////////////////////////
  // 	Enable flops for op1
  /////////////////////////////////////////////////////////////////////
  clken_buf	ckbuf_1(.clk(clk_enb1), .rclk(clk), .enb_l(~(valid & mul_step)), .tmb_l(tm_l));
  dff_s #(64)  	ffrs1  (.din(rs1_l[63:0]), .clk(clk_enb1), .q(op1_l[63:0]),
			.se(se), .si(), .so());




  assign op1[63:0] = ~op1_l[63:0];

  mul_booth	 booth (.head (valid),
			.b_in (rs2),
			.b0   (b0),
			.b1   (b1),
			.b2   (b2),
			.b3   (b3),
			.b4   (b4),
			.b5   (b5),
			.b6   (b6),
			.b7   (b7),
			.b8   (b8),
			.b9   (b9),
			.b10  (b10),
			.b11  (b11),
			.b12  (b12),
			.b13  (b13),
			.b14  (b14),
			.b15  (b15),
			.b16  (b16),
			.clk  (clk), .se(se), .si(), .so(), .mul_step(mul_step), .tm_l(tm_l));
			
  /////////////////////////////////////////////////////////////////////
  // 	Two Array1 inst ary1_a0 & ary1_a1 with the ouput flops 
  /////////////////////////////////////////////////////////////////////
  mul_array1	ary1_a0(.cout (a0cout[81:4]),
			.sum  (a0sum[81:0]),
			.a    (op1),
			.b0   (b0),
			.b1   (b1),
			.b2   (b2),
			.b3   (b3),
			.b4   (b4),
			.b5   (b5),
			.b6   (b6),
			.b7   (b7),
			.b8   (3'b000),
			.head (cyc1),
			.bot  (1'b0)); //array a is never at the bottom of 33-pp rows
 
  dff_s #(78)  a0cot_dff (.din(a0cout[81:4]), .clk(clk_enb0), .q(a0c[81:4]),
			.se(se), .si(), .so());
  dff_s #(82)  a0sum_dff (.din(a0sum[81:0]), .clk(clk_enb0), .q(a0s[81:0]),
			.se(se), .si(), .so());

  mul_array1	ary1_a1(.cout (a1cout[81:4]),
			.sum  (a1sum[81:0]),
			.a    (op1),
			.b0   (b8),
			.b1   (b9),
			.b2   (b10),
			.b3   (b11),
			.b4   (b12),
			.b5   (b13),
			.b6   (b14),
			.b7   (b15),
			.b8   ({1'b0,b16,1'b0}),
			.head (1'b0),	//array b is never at the head of 33-pp rows
			.bot  (cyc2)); 

  dff_s #(78)  a1cot_dff (.din(a1cout[81:4]), .clk(clk_enb0), .q(a1c[81:4]),
			.se(se), .si(), .so());
  dff_s #(82)  a1sum_dff (.din(a1sum[81:0]), .clk(clk_enb0), .q(a1s[81:0]),
			.se(se), .si(), .so());

  /////////////////////////////////////////////////////////////////////
  // 	Array2 with the reorder output mux-flops
  /////////////////////////////////////////////////////////////////////
  mul_array2 	 array2(.pcoutx2 (pcoutx2),
			.psumx2  (psumx2),
			.pcout 	 (pcout[98:0]),
			.psum    (psum[98:0]), 
			.a0c     (a0c[81:4]),
			.a0s     (a0s[81:0]),
			.a1c     (a1c[81:4]),
			.a1s     (a1s[81:0]),
			.pc	 (pc[98:30]),
			.ps	 (ps[98:31]),
			.areg    (areg[96:0]),
			.bot     (cyc3),
			.x2      (x2_c2c3));
 
  //// Outpput re-order muxes and flops	////
  dp_mux2es #(97)  ary2_cmux (.dout(ary2_cout[96:0]),
                              .in0(pcout[96:0]),
                              .in1({pcout[95:0],pcoutx2}),
                              .sel(x2_c2c3));
  dff_s #(97)  a2cot_dff (.din(ary2_cout[96:0]), .clk(clk_enb0), .q(addin_cout[96:0]), 
              		.se(se), .si(), .so());

  dp_mux2es #(98) ary2_smux (.dout(ary2_sum[97:0]),
                             .in0(psum[97:0]),
                             .in1({psum[96:0],psumx2}),
                             .sel(x2_c2c3));
  dff_s #(98)  a2sum_dff (.din(ary2_sum[97:0]), .clk(clk_enb0), .q(addin_sum[97:0]), 
			.se(se), .si(), .so());

  //// Pseudo sum & cout logic and flops ////
  assign psum_in[98:32]  = psum[98:32] & {67{cyc2}} ;
  assign psum_in[31]     = psum[31] & x2_c2 ;

  assign pcout_in[98:31] = pcout[98:31] & {68{cyc2}} ;
  assign pcout_in[30]    = pcout[30] & x2_c2 ;
  
  dff_s #(68)  psum_dff  (.din(psum_in[98:31]), .clk(clk_enb0), .q(ps[98:31]),
                	.se(se), .si(), .so());
  dff_s #(69)  pcout_dff (.din(pcout_in[98:30]), .clk(clk_enb0), .q(pc[98:30]),
            		.se(se), .si(), .so());

  /////////////////////////////////////////////////////////////////////
  // 	Adder (104-bit) 
  /////////////////////////////////////////////////////////////////////

  assign 	add_cin = add_co31 & cyc3 ;

  assign {add_co31,addout[31:0]} =   {{1'b0},addin_sum[31:0]} 
		     		   + {{1'b0},addin_cout[30:0],addin_cin} ;


  assign {add_co96,addout[96:32]} =  addin_sum[97:32]	
				  + addin_cout[96:31]
				  + {{65'b0},add_co31} ;

  assign 	addout[103:97] =  accreg[135:129] + {{6'b0},add_co96} ;

  /////////////////////////////////////////////////////////////////////
  // 	Pipe adder outputs  
  /////////////////////////////////////////////////////////////////////

  dff_s  	      co31_dff (.din(add_cin), .clk(clk_enb0), .q(addin_cin),
       			.se(se), .si(), .so());

  dff_s #(104)   out_dff (.din(addout[103:0]), .clk(clk_enb0), .q(out[135:32]),
              		.se(se), .si(), .so());

  dff_s #(32)    pip_dff (.din(out[63:32]), .clk(clk_enb0), .q(out[31:0]),
               		.se(se), .si(), .so());

endmodule // mul64




////////////////////////////////////////////////////////////////////////
//	Sub-moudle for mul64
////////////////////////////////////////////////////////////////////////

module mul_array1 ( cout, sum, a, b0, b1, b2, b3, b4, b5, b6, b7, b8,
     bot, head );

input  bot, head;
output [81:4]  cout;
output [81:0]  sum;
input [2:0]  b6;
input [2:0]  b3;
input [2:0]  b8;
input [2:0]  b2;
input [2:0]  b1;
input [2:0]  b7;
input [63:0]  a;
input [2:0]  b0;
input [2:0]  b4;
input [2:0]  b5;

// Buses in the design

wire  [1:0]  b5n;
wire  [1:0]  b2n;
wire  [68:1]  c0;
wire  [69:0]  s1;
wire  [68:1]  c1;
wire  [69:0]  s2;
wire  [68:1]  c2;
wire  [70:4]  s_1;
wire  [69:2]  s0;
wire  [76:10]  s_2;
wire  [70:2]  c_1;
wire  [76:10]  c_2;
wire  [75:11]  co;

mul_negen p1n ( .b(b5[2:0]), .n1(b5n[1]), .n0(b5n[0]));
mul_negen p0n ( .b(b2[2:0]), .n1(b2n[1]), .n0(b2n[0]));
mul_csa42  sc3_71_ ( .c(s_2[71]), .cin(co[70]), .a(c_1[70]),
     .b(c_2[70]), .cout(co[71]), .sum(sum[71]), .d(s1[65]),
     .carry(cout[71]));
mul_csa42  sc3_75_ ( .c(s_2[75]), .cin(co[74]), .a(1'b0),
     .b(c_2[74]), .cout(co[75]), .sum(sum[75]), .d(s1[69]),
     .carry(cout[75]));
mul_csa42  sc3_74_ ( .c(s_2[74]), .cin(co[73]), .a(1'b0),
     .b(c_2[73]), .cout(co[74]), .sum(sum[74]), .d(s1[68]),
     .carry(cout[74]));
mul_csa42  sc3_73_ ( .c(s_2[73]), .cin(co[72]), .a(1'b0),
     .b(c_2[72]), .cout(co[73]), .sum(sum[73]), .d(s1[67]),
     .carry(cout[73]));
mul_csa42  sc3_72_ ( .c(s_2[72]), .cin(co[71]), .a(1'b0),
     .b(c_2[71]), .cout(co[72]), .sum(sum[72]), .d(s1[66]),
     .carry(cout[72]));
mul_csa42  sc3_76_ ( .c(s_2[76]), .cin(co[75]), .a(1'b0),
     .b(c_2[75]), .cout(), .sum(sum[76]), .d(1'b0),
     .carry(cout[76]));
mul_csa42  sc3_70_ ( .c(s_2[70]), .cin(co[69]), .a(c_1[69]),
     .b(c_2[69]), .cout(co[70]), .sum(sum[70]), .d(s_1[70]),
     .carry(cout[70]));
mul_csa42  sc3_69_ ( .c(s_2[69]), .cin(co[68]), .a(c_1[68]),
     .b(c_2[68]), .cout(co[69]), .sum(sum[69]), .d(s_1[69]),
     .carry(cout[69]));
mul_csa42  sc3_68_ ( .c(s_2[68]), .cin(co[67]), .a(c_1[67]),
     .b(c_2[67]), .cout(co[68]), .sum(sum[68]), .d(s_1[68]),
     .carry(cout[68]));
mul_csa42  sc3_67_ ( .c(s_2[67]), .cin(co[66]), .a(c_1[66]),
     .b(c_2[66]), .cout(co[67]), .sum(sum[67]), .d(s_1[67]),
     .carry(cout[67]));
mul_csa42  sc3_66_ ( .c(s_2[66]), .cin(co[65]), .a(c_1[65]),
     .b(c_2[65]), .cout(co[66]), .sum(sum[66]), .d(s_1[66]),
     .carry(cout[66]));
mul_csa42  sc3_65_ ( .c(s_2[65]), .cin(co[64]), .a(c_1[64]),
     .b(c_2[64]), .cout(co[65]), .sum(sum[65]), .d(s_1[65]),
     .carry(cout[65]));
mul_csa42  sc3_64_ ( .c(s_2[64]), .cin(co[63]), .a(c_1[63]),
     .b(c_2[63]), .cout(co[64]), .sum(sum[64]), .d(s_1[64]),
     .carry(cout[64]));
mul_csa42  sc3_63_ ( .c(s_2[63]), .cin(co[62]), .a(c_1[62]),
     .b(c_2[62]), .cout(co[63]), .sum(sum[63]), .d(s_1[63]),
     .carry(cout[63]));
mul_csa42  sc3_62_ ( .c(s_2[62]), .cin(co[61]), .a(c_1[61]),
     .b(c_2[61]), .cout(co[62]), .sum(sum[62]), .d(s_1[62]),
     .carry(cout[62]));
mul_csa42  sc3_61_ ( .c(s_2[61]), .cin(co[60]), .a(c_1[60]),
     .b(c_2[60]), .cout(co[61]), .sum(sum[61]), .d(s_1[61]),
     .carry(cout[61]));
mul_csa42  sc3_60_ ( .c(s_2[60]), .cin(co[59]), .a(c_1[59]),
     .b(c_2[59]), .cout(co[60]), .sum(sum[60]), .d(s_1[60]),
     .carry(cout[60]));
mul_csa42  sc3_59_ ( .c(s_2[59]), .cin(co[58]), .a(c_1[58]),
     .b(c_2[58]), .cout(co[59]), .sum(sum[59]), .d(s_1[59]),
     .carry(cout[59]));
mul_csa42  sc3_58_ ( .c(s_2[58]), .cin(co[57]), .a(c_1[57]),
     .b(c_2[57]), .cout(co[58]), .sum(sum[58]), .d(s_1[58]),
     .carry(cout[58]));
mul_csa42  sc3_57_ ( .c(s_2[57]), .cin(co[56]), .a(c_1[56]),
     .b(c_2[56]), .cout(co[57]), .sum(sum[57]), .d(s_1[57]),
     .carry(cout[57]));
mul_csa42  sc3_56_ ( .c(s_2[56]), .cin(co[55]), .a(c_1[55]),
     .b(c_2[55]), .cout(co[56]), .sum(sum[56]), .d(s_1[56]),
     .carry(cout[56]));
mul_csa42  sc3_55_ ( .c(s_2[55]), .cin(co[54]), .a(c_1[54]),
     .b(c_2[54]), .cout(co[55]), .sum(sum[55]), .d(s_1[55]),
     .carry(cout[55]));
mul_csa42  sc3_54_ ( .c(s_2[54]), .cin(co[53]), .a(c_1[53]),
     .b(c_2[53]), .cout(co[54]), .sum(sum[54]), .d(s_1[54]),
     .carry(cout[54]));
mul_csa42  sc3_53_ ( .c(s_2[53]), .cin(co[52]), .a(c_1[52]),
     .b(c_2[52]), .cout(co[53]), .sum(sum[53]), .d(s_1[53]),
     .carry(cout[53]));
mul_csa42  sc3_52_ ( .c(s_2[52]), .cin(co[51]), .a(c_1[51]),
     .b(c_2[51]), .cout(co[52]), .sum(sum[52]), .d(s_1[52]),
     .carry(cout[52]));
mul_csa42  sc3_51_ ( .c(s_2[51]), .cin(co[50]), .a(c_1[50]),
     .b(c_2[50]), .cout(co[51]), .sum(sum[51]), .d(s_1[51]),
     .carry(cout[51]));
mul_csa42  sc3_50_ ( .c(s_2[50]), .cin(co[49]), .a(c_1[49]),
     .b(c_2[49]), .cout(co[50]), .sum(sum[50]), .d(s_1[50]),
     .carry(cout[50]));
mul_csa42  sc3_49_ ( .c(s_2[49]), .cin(co[48]), .a(c_1[48]),
     .b(c_2[48]), .cout(co[49]), .sum(sum[49]), .d(s_1[49]),
     .carry(cout[49]));
mul_csa42  sc3_48_ ( .c(s_2[48]), .cin(co[47]), .a(c_1[47]),
     .b(c_2[47]), .cout(co[48]), .sum(sum[48]), .d(s_1[48]),
     .carry(cout[48]));
mul_csa42  sc3_47_ ( .c(s_2[47]), .cin(co[46]), .a(c_1[46]),
     .b(c_2[46]), .cout(co[47]), .sum(sum[47]), .d(s_1[47]),
     .carry(cout[47]));
mul_csa42  sc3_46_ ( .c(s_2[46]), .cin(co[45]), .a(c_1[45]),
     .b(c_2[45]), .cout(co[46]), .sum(sum[46]), .d(s_1[46]),
     .carry(cout[46]));
mul_csa42  sc3_45_ ( .c(s_2[45]), .cin(co[44]), .a(c_1[44]),
     .b(c_2[44]), .cout(co[45]), .sum(sum[45]), .d(s_1[45]),
     .carry(cout[45]));
mul_csa42  sc3_44_ ( .c(s_2[44]), .cin(co[43]), .a(c_1[43]),
     .b(c_2[43]), .cout(co[44]), .sum(sum[44]), .d(s_1[44]),
     .carry(cout[44]));
mul_csa42  sc3_43_ ( .c(s_2[43]), .cin(co[42]), .a(c_1[42]),
     .b(c_2[42]), .cout(co[43]), .sum(sum[43]), .d(s_1[43]),
     .carry(cout[43]));
mul_csa42  sc3_42_ ( .c(s_2[42]), .cin(co[41]), .a(c_1[41]),
     .b(c_2[41]), .cout(co[42]), .sum(sum[42]), .d(s_1[42]),
     .carry(cout[42]));
mul_csa42  sc3_41_ ( .c(s_2[41]), .cin(co[40]), .a(c_1[40]),
     .b(c_2[40]), .cout(co[41]), .sum(sum[41]), .d(s_1[41]),
     .carry(cout[41]));
mul_csa42  sc3_40_ ( .c(s_2[40]), .cin(co[39]), .a(c_1[39]),
     .b(c_2[39]), .cout(co[40]), .sum(sum[40]), .d(s_1[40]),
     .carry(cout[40]));
mul_csa42  sc3_39_ ( .c(s_2[39]), .cin(co[38]), .a(c_1[38]),
     .b(c_2[38]), .cout(co[39]), .sum(sum[39]), .d(s_1[39]),
     .carry(cout[39]));
mul_csa42  sc3_38_ ( .c(s_2[38]), .cin(co[37]), .a(c_1[37]),
     .b(c_2[37]), .cout(co[38]), .sum(sum[38]), .d(s_1[38]),
     .carry(cout[38]));
mul_csa42  sc3_37_ ( .c(s_2[37]), .cin(co[36]), .a(c_1[36]),
     .b(c_2[36]), .cout(co[37]), .sum(sum[37]), .d(s_1[37]),
     .carry(cout[37]));
mul_csa42  sc3_36_ ( .c(s_2[36]), .cin(co[35]), .a(c_1[35]),
     .b(c_2[35]), .cout(co[36]), .sum(sum[36]), .d(s_1[36]),
     .carry(cout[36]));
mul_csa42  sc3_35_ ( .c(s_2[35]), .cin(co[34]), .a(c_1[34]),
     .b(c_2[34]), .cout(co[35]), .sum(sum[35]), .d(s_1[35]),
     .carry(cout[35]));
mul_csa42  sc3_34_ ( .c(s_2[34]), .cin(co[33]), .a(c_1[33]),
     .b(c_2[33]), .cout(co[34]), .sum(sum[34]), .d(s_1[34]),
     .carry(cout[34]));
mul_csa42  sc3_33_ ( .c(s_2[33]), .cin(co[32]), .a(c_1[32]),
     .b(c_2[32]), .cout(co[33]), .sum(sum[33]), .d(s_1[33]),
     .carry(cout[33]));
mul_csa42  sc3_32_ ( .c(s_2[32]), .cin(co[31]), .a(c_1[31]),
     .b(c_2[31]), .cout(co[32]), .sum(sum[32]), .d(s_1[32]),
     .carry(cout[32]));
mul_csa42  sc3_31_ ( .c(s_2[31]), .cin(co[30]), .a(c_1[30]),
     .b(c_2[30]), .cout(co[31]), .sum(sum[31]), .d(s_1[31]),
     .carry(cout[31]));
mul_csa42  sc3_30_ ( .c(s_2[30]), .cin(co[29]), .a(c_1[29]),
     .b(c_2[29]), .cout(co[30]), .sum(sum[30]), .d(s_1[30]),
     .carry(cout[30]));
mul_csa42  sc3_29_ ( .c(s_2[29]), .cin(co[28]), .a(c_1[28]),
     .b(c_2[28]), .cout(co[29]), .sum(sum[29]), .d(s_1[29]),
     .carry(cout[29]));
mul_csa42  sc3_28_ ( .c(s_2[28]), .cin(co[27]), .a(c_1[27]),
     .b(c_2[27]), .cout(co[28]), .sum(sum[28]), .d(s_1[28]),
     .carry(cout[28]));
mul_csa42  sc3_27_ ( .c(s_2[27]), .cin(co[26]), .a(c_1[26]),
     .b(c_2[26]), .cout(co[27]), .sum(sum[27]), .d(s_1[27]),
     .carry(cout[27]));
mul_csa42  sc3_26_ ( .c(s_2[26]), .cin(co[25]), .a(c_1[25]),
     .b(c_2[25]), .cout(co[26]), .sum(sum[26]), .d(s_1[26]),
     .carry(cout[26]));
mul_csa42  sc3_25_ ( .c(s_2[25]), .cin(co[24]), .a(c_1[24]),
     .b(c_2[24]), .cout(co[25]), .sum(sum[25]), .d(s_1[25]),
     .carry(cout[25]));
mul_csa42  sc3_24_ ( .c(s_2[24]), .cin(co[23]), .a(c_1[23]),
     .b(c_2[23]), .cout(co[24]), .sum(sum[24]), .d(s_1[24]),
     .carry(cout[24]));
mul_csa42  sc3_23_ ( .c(s_2[23]), .cin(co[22]), .a(c_1[22]),
     .b(c_2[22]), .cout(co[23]), .sum(sum[23]), .d(s_1[23]),
     .carry(cout[23]));
mul_csa42  sc3_22_ ( .c(s_2[22]), .cin(co[21]), .a(c_1[21]),
     .b(c_2[21]), .cout(co[22]), .sum(sum[22]), .d(s_1[22]),
     .carry(cout[22]));
mul_csa42  sc3_21_ ( .c(s_2[21]), .cin(co[20]), .a(c_1[20]),
     .b(c_2[20]), .cout(co[21]), .sum(sum[21]), .d(s_1[21]),
     .carry(cout[21]));
mul_csa42  sc3_20_ ( .c(s_2[20]), .cin(co[19]), .a(c_1[19]),
     .b(c_2[19]), .cout(co[20]), .sum(sum[20]), .d(s_1[20]),
     .carry(cout[20]));
mul_csa42  sc3_19_ ( .c(s_2[19]), .cin(co[18]), .a(c_1[18]),
     .b(c_2[18]), .cout(co[19]), .sum(sum[19]), .d(s_1[19]),
     .carry(cout[19]));
mul_csa42  sc3_18_ ( .c(s_2[18]), .cin(co[17]), .a(c_1[17]),
     .b(c_2[17]), .cout(co[18]), .sum(sum[18]), .d(s_1[18]),
     .carry(cout[18]));
mul_csa42  sc3_17_ ( .c(s_2[17]), .cin(co[16]), .a(c_1[16]),
     .b(c_2[16]), .cout(co[17]), .sum(sum[17]), .d(s_1[17]),
     .carry(cout[17]));
mul_csa42  sc3_16_ ( .c(s_2[16]), .cin(co[15]), .a(c_1[15]),
     .b(c_2[15]), .cout(co[16]), .sum(sum[16]), .d(s_1[16]),
     .carry(cout[16]));
mul_csa42  sc3_15_ ( .c(s_2[15]), .cin(co[14]), .a(c_1[14]),
     .b(c_2[14]), .cout(co[15]), .sum(sum[15]), .d(s_1[15]),
     .carry(cout[15]));
mul_csa42  sc3_14_ ( .c(s_2[14]), .cin(co[13]), .a(c_1[13]),
     .b(c_2[13]), .cout(co[14]), .sum(sum[14]), .d(s_1[14]),
     .carry(cout[14]));
mul_csa42  sc3_13_ ( .c(s_2[13]), .cin(co[12]), .a(c_1[12]),
     .b(c_2[12]), .cout(co[13]), .sum(sum[13]), .d(s_1[13]),
     .carry(cout[13]));
mul_csa42  sc3_12_ ( .c(s_2[12]), .cin(co[11]), .a(c_1[11]),
     .b(c_2[11]), .cout(co[12]), .sum(sum[12]), .d(s_1[12]),
     .carry(cout[12]));
mul_csa42  sc3_11_ ( .c(s_2[11]), .cin(1'b0),
     .a(c_1[10]), .b(c_2[10]), .cout(co[11]), .sum(sum[11]),
     .d(s_1[11]), .carry(cout[11]));
mul_csa32  sc2_2_70_ ( .c(c1[63]), .b(c2[57]), .a(s2[58]),
     .cout(c_2[70]), .sum(s_2[70]));
mul_csa32  sc2_2_69_ ( .c(c1[62]), .b(c2[56]), .a(s2[57]),
     .cout(c_2[69]), .sum(s_2[69]));
mul_csa32  sc2_2_68_ ( .c(c1[61]), .b(c2[55]), .a(s2[56]),
     .cout(c_2[68]), .sum(s_2[68]));
mul_csa32  sc2_2_67_ ( .c(c1[60]), .b(c2[54]), .a(s2[55]),
     .cout(c_2[67]), .sum(s_2[67]));
mul_csa32  sc2_2_66_ ( .c(c1[59]), .b(c2[53]), .a(s2[54]),
     .cout(c_2[66]), .sum(s_2[66]));
mul_csa32  sc2_2_65_ ( .c(c1[58]), .b(c2[52]), .a(s2[53]),
     .cout(c_2[65]), .sum(s_2[65]));
mul_csa32  sc2_2_64_ ( .c(c1[57]), .b(c2[51]), .a(s2[52]),
     .cout(c_2[64]), .sum(s_2[64]));
mul_csa32  sc2_2_63_ ( .c(c1[56]), .b(c2[50]), .a(s2[51]),
     .cout(c_2[63]), .sum(s_2[63]));
mul_csa32  sc2_2_62_ ( .c(c1[55]), .b(c2[49]), .a(s2[50]),
     .cout(c_2[62]), .sum(s_2[62]));
mul_csa32  sc2_2_61_ ( .c(c1[54]), .b(c2[48]), .a(s2[49]),
     .cout(c_2[61]), .sum(s_2[61]));
mul_csa32  sc2_2_60_ ( .c(c1[53]), .b(c2[47]), .a(s2[48]),
     .cout(c_2[60]), .sum(s_2[60]));
mul_csa32  sc2_2_59_ ( .c(c1[52]), .b(c2[46]), .a(s2[47]),
     .cout(c_2[59]), .sum(s_2[59]));
mul_csa32  sc2_2_58_ ( .c(c1[51]), .b(c2[45]), .a(s2[46]),
     .cout(c_2[58]), .sum(s_2[58]));
mul_csa32  sc2_2_57_ ( .c(c1[50]), .b(c2[44]), .a(s2[45]),
     .cout(c_2[57]), .sum(s_2[57]));
mul_csa32  sc2_2_56_ ( .c(c1[49]), .b(c2[43]), .a(s2[44]),
     .cout(c_2[56]), .sum(s_2[56]));
mul_csa32  sc2_2_55_ ( .c(c1[48]), .b(c2[42]), .a(s2[43]),
     .cout(c_2[55]), .sum(s_2[55]));
mul_csa32  sc2_2_54_ ( .c(c1[47]), .b(c2[41]), .a(s2[42]),
     .cout(c_2[54]), .sum(s_2[54]));
mul_csa32  sc2_2_53_ ( .c(c1[46]), .b(c2[40]), .a(s2[41]),
     .cout(c_2[53]), .sum(s_2[53]));
mul_csa32  sc2_2_52_ ( .c(c1[45]), .b(c2[39]), .a(s2[40]),
     .cout(c_2[52]), .sum(s_2[52]));
mul_csa32  sc2_2_51_ ( .c(c1[44]), .b(c2[38]), .a(s2[39]),
     .cout(c_2[51]), .sum(s_2[51]));
mul_csa32  sc2_2_50_ ( .c(c1[43]), .b(c2[37]), .a(s2[38]),
     .cout(c_2[50]), .sum(s_2[50]));
mul_csa32  sc2_2_49_ ( .c(c1[42]), .b(c2[36]), .a(s2[37]),
     .cout(c_2[49]), .sum(s_2[49]));
mul_csa32  sc2_2_48_ ( .c(c1[41]), .b(c2[35]), .a(s2[36]),
     .cout(c_2[48]), .sum(s_2[48]));
mul_csa32  sc2_2_47_ ( .c(c1[40]), .b(c2[34]), .a(s2[35]),
     .cout(c_2[47]), .sum(s_2[47]));
mul_csa32  sc2_2_46_ ( .c(c1[39]), .b(c2[33]), .a(s2[34]),
     .cout(c_2[46]), .sum(s_2[46]));
mul_csa32  sc2_2_45_ ( .c(c1[38]), .b(c2[32]), .a(s2[33]),
     .cout(c_2[45]), .sum(s_2[45]));
mul_csa32  sc2_2_44_ ( .c(c1[37]), .b(c2[31]), .a(s2[32]),
     .cout(c_2[44]), .sum(s_2[44]));
mul_csa32  sc2_2_43_ ( .c(c1[36]), .b(c2[30]), .a(s2[31]),
     .cout(c_2[43]), .sum(s_2[43]));
mul_csa32  sc2_2_42_ ( .c(c1[35]), .b(c2[29]), .a(s2[30]),
     .cout(c_2[42]), .sum(s_2[42]));
mul_csa32  sc2_2_41_ ( .c(c1[34]), .b(c2[28]), .a(s2[29]),
     .cout(c_2[41]), .sum(s_2[41]));
mul_csa32  sc2_2_40_ ( .c(c1[33]), .b(c2[27]), .a(s2[28]),
     .cout(c_2[40]), .sum(s_2[40]));
mul_csa32  sc2_2_39_ ( .c(c1[32]), .b(c2[26]), .a(s2[27]),
     .cout(c_2[39]), .sum(s_2[39]));
mul_csa32  sc2_2_38_ ( .c(c1[31]), .b(c2[25]), .a(s2[26]),
     .cout(c_2[38]), .sum(s_2[38]));
mul_csa32  sc2_2_37_ ( .c(c1[30]), .b(c2[24]), .a(s2[25]),
     .cout(c_2[37]), .sum(s_2[37]));
mul_csa32  sc2_2_36_ ( .c(c1[29]), .b(c2[23]), .a(s2[24]),
     .cout(c_2[36]), .sum(s_2[36]));
mul_csa32  sc2_2_35_ ( .c(c1[28]), .b(c2[22]), .a(s2[23]),
     .cout(c_2[35]), .sum(s_2[35]));
mul_csa32  sc2_2_34_ ( .c(c1[27]), .b(c2[21]), .a(s2[22]),
     .cout(c_2[34]), .sum(s_2[34]));
mul_csa32  sc2_2_33_ ( .c(c1[26]), .b(c2[20]), .a(s2[21]),
     .cout(c_2[33]), .sum(s_2[33]));
mul_csa32  sc2_2_32_ ( .c(c1[25]), .b(c2[19]), .a(s2[20]),
     .cout(c_2[32]), .sum(s_2[32]));
mul_csa32  sc2_2_31_ ( .c(c1[24]), .b(c2[18]), .a(s2[19]),
     .cout(c_2[31]), .sum(s_2[31]));
mul_csa32  sc2_2_30_ ( .c(c1[23]), .b(c2[17]), .a(s2[18]),
     .cout(c_2[30]), .sum(s_2[30]));
mul_csa32  sc2_2_29_ ( .c(c1[22]), .b(c2[16]), .a(s2[17]),
     .cout(c_2[29]), .sum(s_2[29]));
mul_csa32  sc2_2_28_ ( .c(c1[21]), .b(c2[15]), .a(s2[16]),
     .cout(c_2[28]), .sum(s_2[28]));
mul_csa32  sc2_2_27_ ( .c(c1[20]), .b(c2[14]), .a(s2[15]),
     .cout(c_2[27]), .sum(s_2[27]));
mul_csa32  sc2_2_26_ ( .c(c1[19]), .b(c2[13]), .a(s2[14]),
     .cout(c_2[26]), .sum(s_2[26]));
mul_csa32  sc2_2_25_ ( .c(c1[18]), .b(c2[12]), .a(s2[13]),
     .cout(c_2[25]), .sum(s_2[25]));
mul_csa32  sc2_2_24_ ( .c(c1[17]), .b(c2[11]), .a(s2[12]),
     .cout(c_2[24]), .sum(s_2[24]));
mul_csa32  sc2_2_23_ ( .c(c1[16]), .b(c2[10]), .a(s2[11]),
     .cout(c_2[23]), .sum(s_2[23]));
mul_csa32  sc2_2_22_ ( .c(c1[15]), .b(c2[9]), .a(s2[10]),
     .cout(c_2[22]), .sum(s_2[22]));
mul_csa32  sc2_2_21_ ( .c(c1[14]), .b(c2[8]), .a(s2[9]),
     .cout(c_2[21]), .sum(s_2[21]));
mul_csa32  sc2_2_20_ ( .c(c1[13]), .b(c2[7]), .a(s2[8]),
     .cout(c_2[20]), .sum(s_2[20]));
mul_csa32  sc2_2_19_ ( .c(c1[12]), .b(c2[6]), .a(s2[7]),
     .cout(c_2[19]), .sum(s_2[19]));
mul_csa32  sc2_2_18_ ( .c(c1[11]), .b(c2[5]), .a(s2[6]),
     .cout(c_2[18]), .sum(s_2[18]));
mul_csa32  sc2_2_17_ ( .c(c1[10]), .b(c2[4]), .a(s2[5]),
     .cout(c_2[17]), .sum(s_2[17]));
mul_csa32  sc2_2_16_ ( .c(c1[9]), .b(c2[3]), .a(s2[4]),
     .cout(c_2[16]), .sum(s_2[16]));
mul_csa32  sc2_2_15_ ( .c(c1[8]), .b(c2[2]), .a(s2[3]),
     .cout(c_2[15]), .sum(s_2[15]));
mul_csa32  sc2_2_14_ ( .c(c1[7]), .b(c2[1]), .a(s2[2]),
     .cout(c_2[14]), .sum(s_2[14]));
mul_csa32  sc2_2_13_ ( .c(c1[6]), .b(s1[7]), .a(s2[1]),
     .cout(c_2[13]), .sum(s_2[13]));
mul_csa32  sc2_2_12_ ( .c(c1[5]), .b(s1[6]), .a(s2[0]),
     .cout(c_2[12]), .sum(s_2[12]));
mul_csa32  sc2_2_11_ ( .c(c1[4]), .b(s1[5]), .a(b5n[1]),
     .cout(c_2[11]), .sum(s_2[11]));
mul_csa32  sc2_2_10_ ( .c(c1[3]), .b(s1[4]), .a(b5n[0]),
     .cout(c_2[10]), .sum(s_2[10]));
mul_csa32  sc2_2_76_ ( .c(1'b1), .b(c2[63]), .a(s2[64]),
     .cout(c_2[76]), .sum(s_2[76]));
mul_csa32  sc2_2_77_ ( .c(c_2[76]), .b(c2[64]), .a(s2[65]),
     .cout(cout[77]), .sum(sum[77]));
mul_csa32  sc2_1_9_ ( .c(s1[3]), .b(c0[8]), .a(s0[9]), .cout(c_1[9]),
     .sum(s_1[9]));
mul_csa32  sc2_1_8_ ( .c(s1[2]), .b(c0[7]), .a(s0[8]), .cout(c_1[8]),
     .sum(s_1[8]));
mul_csa32  sc2_1_3_ ( .c(c_1[2]), .b(c0[2]), .a(s0[3]),
     .cout(c_1[3]), .sum(sum[3]));
mul_csa32  sc3_10_ ( .c(s_2[10]), .b(s_1[10]), .a(c_1[9]),
     .cout(cout[10]), .sum(sum[10]));
mul_csa32  sc3_9_ ( .c(c1[2]), .sum(sum[9]), .cout(cout[9]),
     .a(c_1[8]), .b(s_1[9]));
mul_csa32  sc3_8_ ( .c(c1[1]), .sum(sum[8]), .cout(cout[8]),
     .a(c_1[7]), .b(s_1[8]));
mul_csa32  sc2_2_71_ ( .c(c1[64]), .b(c2[58]), .a(s2[59]),
     .cout(c_2[71]), .sum(s_2[71]));
mul_csa32  sc2_2_75_ ( .c(c1[68]), .b(c2[62]), .a(s2[63]),
     .cout(c_2[75]), .sum(s_2[75]));
mul_csa32  sc2_2_74_ ( .c(c1[67]), .b(c2[61]), .a(s2[62]),
     .cout(c_2[74]), .sum(s_2[74]));
mul_csa32  sc2_2_73_ ( .c(c1[66]), .b(c2[60]), .a(s2[61]),
     .cout(c_2[73]), .sum(s_2[73]));
mul_csa32  sc2_2_72_ ( .c(c1[65]), .b(c2[59]), .a(s2[60]),
     .cout(c_2[72]), .sum(s_2[72]));
mul_csa32  sc2_1_69_ ( .c(s1[63]), .sum(s_1[69]), .cout(c_1[69]),
     .a(s0[69]), .b(c0[68]));
mul_csa32  sc2_1_68_ ( .c(s1[62]), .sum(s_1[68]), .cout(c_1[68]),
     .a(s0[68]), .b(c0[67]));
mul_csa32  sc2_1_67_ ( .c(s1[61]), .sum(s_1[67]), .cout(c_1[67]),
     .a(s0[67]), .b(c0[66]));
mul_csa32  sc2_1_66_ ( .c(s1[60]), .sum(s_1[66]), .cout(c_1[66]),
     .a(s0[66]), .b(c0[65]));
mul_csa32  sc2_1_65_ ( .c(s1[59]), .sum(s_1[65]), .cout(c_1[65]),
     .a(s0[65]), .b(c0[64]));
mul_csa32  sc2_1_64_ ( .c(s1[58]), .sum(s_1[64]), .cout(c_1[64]),
     .a(s0[64]), .b(c0[63]));
mul_csa32  sc2_1_63_ ( .c(s1[57]), .sum(s_1[63]), .cout(c_1[63]),
     .a(s0[63]), .b(c0[62]));
mul_csa32  sc2_1_62_ ( .c(s1[56]), .sum(s_1[62]), .cout(c_1[62]),
     .a(s0[62]), .b(c0[61]));
mul_csa32  sc2_1_61_ ( .c(s1[55]), .sum(s_1[61]), .cout(c_1[61]),
     .a(s0[61]), .b(c0[60]));
mul_csa32  sc2_1_60_ ( .c(s1[54]), .sum(s_1[60]), .cout(c_1[60]),
     .a(s0[60]), .b(c0[59]));
mul_csa32  sc2_1_59_ ( .c(s1[53]), .sum(s_1[59]), .cout(c_1[59]),
     .a(s0[59]), .b(c0[58]));
mul_csa32  sc2_1_58_ ( .c(s1[52]), .sum(s_1[58]), .cout(c_1[58]),
     .a(s0[58]), .b(c0[57]));
mul_csa32  sc2_1_57_ ( .c(s1[51]), .sum(s_1[57]), .cout(c_1[57]),
     .a(s0[57]), .b(c0[56]));
mul_csa32  sc2_1_56_ ( .c(s1[50]), .sum(s_1[56]), .cout(c_1[56]),
     .a(s0[56]), .b(c0[55]));
mul_csa32  sc2_1_55_ ( .c(s1[49]), .sum(s_1[55]), .cout(c_1[55]),
     .a(s0[55]), .b(c0[54]));
mul_csa32  sc2_1_54_ ( .c(s1[48]), .sum(s_1[54]), .cout(c_1[54]),
     .a(s0[54]), .b(c0[53]));
mul_csa32  sc2_1_53_ ( .c(s1[47]), .sum(s_1[53]), .cout(c_1[53]),
     .a(s0[53]), .b(c0[52]));
mul_csa32  sc2_1_52_ ( .c(s1[46]), .sum(s_1[52]), .cout(c_1[52]),
     .a(s0[52]), .b(c0[51]));
mul_csa32  sc2_1_51_ ( .c(s1[45]), .sum(s_1[51]), .cout(c_1[51]),
     .a(s0[51]), .b(c0[50]));
mul_csa32  sc2_1_50_ ( .c(s1[44]), .sum(s_1[50]), .cout(c_1[50]),
     .a(s0[50]), .b(c0[49]));
mul_csa32  sc2_1_49_ ( .c(s1[43]), .sum(s_1[49]), .cout(c_1[49]),
     .a(s0[49]), .b(c0[48]));
mul_csa32  sc2_1_48_ ( .c(s1[42]), .sum(s_1[48]), .cout(c_1[48]),
     .a(s0[48]), .b(c0[47]));
mul_csa32  sc2_1_47_ ( .c(s1[41]), .sum(s_1[47]), .cout(c_1[47]),
     .a(s0[47]), .b(c0[46]));
mul_csa32  sc2_1_46_ ( .c(s1[40]), .sum(s_1[46]), .cout(c_1[46]),
     .a(s0[46]), .b(c0[45]));
mul_csa32  sc2_1_45_ ( .c(s1[39]), .sum(s_1[45]), .cout(c_1[45]),
     .a(s0[45]), .b(c0[44]));
mul_csa32  sc2_1_44_ ( .c(s1[38]), .sum(s_1[44]), .cout(c_1[44]),
     .a(s0[44]), .b(c0[43]));
mul_csa32  sc2_1_43_ ( .c(s1[37]), .sum(s_1[43]), .cout(c_1[43]),
     .a(s0[43]), .b(c0[42]));
mul_csa32  sc2_1_42_ ( .c(s1[36]), .sum(s_1[42]), .cout(c_1[42]),
     .a(s0[42]), .b(c0[41]));
mul_csa32  sc2_1_41_ ( .c(s1[35]), .sum(s_1[41]), .cout(c_1[41]),
     .a(s0[41]), .b(c0[40]));
mul_csa32  sc2_1_40_ ( .c(s1[34]), .sum(s_1[40]), .cout(c_1[40]),
     .a(s0[40]), .b(c0[39]));
mul_csa32  sc2_1_39_ ( .c(s1[33]), .sum(s_1[39]), .cout(c_1[39]),
     .a(s0[39]), .b(c0[38]));
mul_csa32  sc2_1_38_ ( .c(s1[32]), .sum(s_1[38]), .cout(c_1[38]),
     .a(s0[38]), .b(c0[37]));
mul_csa32  sc2_1_37_ ( .c(s1[31]), .sum(s_1[37]), .cout(c_1[37]),
     .a(s0[37]), .b(c0[36]));
mul_csa32  sc2_1_36_ ( .c(s1[30]), .sum(s_1[36]), .cout(c_1[36]),
     .a(s0[36]), .b(c0[35]));
mul_csa32  sc2_1_35_ ( .c(s1[29]), .sum(s_1[35]), .cout(c_1[35]),
     .a(s0[35]), .b(c0[34]));
mul_csa32  sc2_1_34_ ( .c(s1[28]), .sum(s_1[34]), .cout(c_1[34]),
     .a(s0[34]), .b(c0[33]));
mul_csa32  sc2_1_33_ ( .c(s1[27]), .sum(s_1[33]), .cout(c_1[33]),
     .a(s0[33]), .b(c0[32]));
mul_csa32  sc2_1_32_ ( .c(s1[26]), .sum(s_1[32]), .cout(c_1[32]),
     .a(s0[32]), .b(c0[31]));
mul_csa32  sc2_1_31_ ( .c(s1[25]), .sum(s_1[31]), .cout(c_1[31]),
     .a(s0[31]), .b(c0[30]));
mul_csa32  sc2_1_30_ ( .c(s1[24]), .sum(s_1[30]), .cout(c_1[30]),
     .a(s0[30]), .b(c0[29]));
mul_csa32  sc2_1_29_ ( .c(s1[23]), .sum(s_1[29]), .cout(c_1[29]),
     .a(s0[29]), .b(c0[28]));
mul_csa32  sc2_1_28_ ( .c(s1[22]), .sum(s_1[28]), .cout(c_1[28]),
     .a(s0[28]), .b(c0[27]));
mul_csa32  sc2_1_27_ ( .c(s1[21]), .sum(s_1[27]), .cout(c_1[27]),
     .a(s0[27]), .b(c0[26]));
mul_csa32  sc2_1_26_ ( .c(s1[20]), .sum(s_1[26]), .cout(c_1[26]),
     .a(s0[26]), .b(c0[25]));
mul_csa32  sc2_1_25_ ( .c(s1[19]), .sum(s_1[25]), .cout(c_1[25]),
     .a(s0[25]), .b(c0[24]));
mul_csa32  sc2_1_24_ ( .c(s1[18]), .sum(s_1[24]), .cout(c_1[24]),
     .a(s0[24]), .b(c0[23]));
mul_csa32  sc2_1_23_ ( .c(s1[17]), .sum(s_1[23]), .cout(c_1[23]),
     .a(s0[23]), .b(c0[22]));
mul_csa32  sc2_1_22_ ( .c(s1[16]), .sum(s_1[22]), .cout(c_1[22]),
     .a(s0[22]), .b(c0[21]));
mul_csa32  sc2_1_21_ ( .c(s1[15]), .sum(s_1[21]), .cout(c_1[21]),
     .a(s0[21]), .b(c0[20]));
mul_csa32  sc2_1_20_ ( .c(s1[14]), .sum(s_1[20]), .cout(c_1[20]),
     .a(s0[20]), .b(c0[19]));
mul_csa32  sc2_1_19_ ( .c(s1[13]), .sum(s_1[19]), .cout(c_1[19]),
     .a(s0[19]), .b(c0[18]));
mul_csa32  sc2_1_18_ ( .c(s1[12]), .sum(s_1[18]), .cout(c_1[18]),
     .a(s0[18]), .b(c0[17]));
mul_csa32  sc2_1_17_ ( .c(s1[11]), .sum(s_1[17]), .cout(c_1[17]),
     .a(s0[17]), .b(c0[16]));
mul_csa32  sc2_1_16_ ( .c(s1[10]), .sum(s_1[16]), .cout(c_1[16]),
     .a(s0[16]), .b(c0[15]));
mul_csa32  sc2_1_15_ ( .c(s1[9]), .sum(s_1[15]), .cout(c_1[15]),
     .a(s0[15]), .b(c0[14]));
mul_csa32  sc2_1_14_ ( .c(s1[8]), .sum(s_1[14]), .cout(c_1[14]),
     .a(s0[14]), .b(c0[13]));
mul_csa32  sc2_1_7_ ( .c(s1[1]), .b(c0[6]), .a(s0[7]), .cout(c_1[7]),
     .sum(s_1[7]));
mul_csa32  sc2_1_6_ ( .c(s1[0]), .b(c0[5]), .a(s0[6]), .cout(c_1[6]),
     .sum(s_1[6]));
mul_csa32  sc2_1_5_ ( .c(b2n[1]), .b(c0[4]), .a(s0[5]),
     .cout(c_1[5]), .sum(s_1[5]));
mul_csa32  sc2_1_4_ ( .c(b2n[0]), .b(c0[3]), .a(s0[4]),
     .cout(c_1[4]), .sum(s_1[4]));
mul_ha sc2_1_10_ ( .sum(s_1[10]), .cout(c_1[10]), .a(s0[10]),
     .b(c0[9]));
mul_ha sc3_7_ ( .sum(sum[7]), .cout(cout[7]), .a(c_1[6]),
     .b(s_1[7]));
mul_ha sc3_6_ ( .sum(sum[6]), .cout(cout[6]), .a(c_1[5]),
     .b(s_1[6]));
mul_ha sc3_5_ ( .sum(sum[5]), .cout(cout[5]), .a(c_1[4]),
     .b(s_1[5]));
mul_ha sc3_4_ ( .sum(sum[4]), .cout(cout[4]), .a(c_1[3]),
     .b(s_1[4]));
mul_ha sc2_2_81_ ( .sum(sum[81]), .cout(cout[81]), .a(s2[69]),
     .b(c2[68]));
mul_ha sc2_2_80_ ( .sum(sum[80]), .cout(cout[80]), .a(s2[68]),
     .b(c2[67]));
mul_ha sc2_2_79_ ( .sum(sum[79]), .cout(cout[79]), .a(s2[67]),
     .b(c2[66]));
mul_ha sc2_2_78_ ( .sum(sum[78]), .cout(cout[78]), .a(s2[66]),
     .b(c2[65]));
mul_ha sc2_1_70_ ( .sum(s_1[70]), .cout(c_1[70]),
     .a(1'b1), .b(s1[64]));
mul_ha sc2_1_2_ ( .sum(sum[2]), .cout(c_1[2]), .a(s0[2]), .b(c0[1]));
mul_ha sc2_1_13_ ( .sum(s_1[13]), .cout(c_1[13]), .a(s0[13]),
     .b(c0[12]));
mul_ha sc2_1_12_ ( .sum(s_1[12]), .cout(c_1[12]), .a(s0[12]),
     .b(c0[11]));
mul_ha sc2_1_11_ ( .sum(s_1[11]), .cout(c_1[11]), .a(s0[11]),
     .b(c0[10]));
mul_ppgenrow3 I2 ( .head(1'b0), .bot(bot), .b2(b8[2:0]),
     .b1(b7[2:0]), .b0(b6[2:0]), .a(a[63:0]), .sum(s2[69:0]),
     .cout(c2[68:1]));
mul_ppgenrow3 I1 ( .head(1'b0), .bot(1'b1),
     .b2(b5[2:0]), .b1(b4[2:0]), .b0(b3[2:0]), .a(a[63:0]),
     .sum(s1[69:0]), .cout(c1[68:1]));
mul_ppgenrow3 I0 ( .head(head), .bot(1'b1), .b2(b2[2:0]),
     .b1(b1[2:0]), .b0(b0[2:0]), .a(a[63:0]), .sum({s0[69:2],
     sum[1:0]}), .cout(c0[68:1]));

endmodule // mul_array1

module mul_array2 ( pcout, pcoutx2, psum, psumx2, a0c, a0s, a1c, a1s,
     areg, bot, pc, ps, x2 );

output  pcoutx2, psumx2;
input  bot, x2;
output [98:0]  psum;
output [98:0]  pcout;
input [81:4]  a1c;
input [98:30]  pc;
input [98:31]  ps;
input [81:0]  a0s;
input [96:0]  areg;
input [81:0]  a1s;
input [81:4]  a0c;

// Buses in the design
wire  [81:15]  s3;
wire  [81:15]  c3;
wire  [96:0]  ain;
wire  [67:20]  co;
wire  [82:0]  s1;
wire  [96:0]  c2;
wire  [82:0]  c1;
wire  [96:0]  s2;
wire	      ainx2, s1x2, c1x2;

mul_mux2 sh_82_ ( .d1(areg[83]), .z(ain[82]), .d0(areg[82]), .s(x2));
mul_mux2 sh_68_ ( .d1(areg[69]), .z(ain[68]), .d0(areg[68]), .s(x2));
mul_mux2 sh_67_ ( .d1(areg[68]), .z(ain[67]), .d0(areg[67]), .s(x2));
mul_mux2 sh_66_ ( .d1(areg[67]), .z(ain[66]), .d0(areg[66]), .s(x2));
mul_mux2 sh_65_ ( .d1(areg[66]), .z(ain[65]), .d0(areg[65]), .s(x2));
mul_mux2 sh_64_ ( .d1(areg[65]), .z(ain[64]), .d0(areg[64]), .s(x2));
mul_mux2 sh_63_ ( .d1(areg[64]), .z(ain[63]), .d0(areg[63]), .s(x2));
mul_mux2 sh_62_ ( .d1(areg[63]), .z(ain[62]), .d0(areg[62]), .s(x2));
mul_mux2 sh_61_ ( .d1(areg[62]), .z(ain[61]), .d0(areg[61]), .s(x2));
mul_mux2 sh_60_ ( .d1(areg[61]), .z(ain[60]), .d0(areg[60]), .s(x2));
mul_mux2 sh_59_ ( .d1(areg[60]), .z(ain[59]), .d0(areg[59]), .s(x2));
mul_mux2 sh_58_ ( .d1(areg[59]), .z(ain[58]), .d0(areg[58]), .s(x2));
mul_mux2 sh_57_ ( .d1(areg[58]), .z(ain[57]), .d0(areg[57]), .s(x2));
mul_mux2 sh_56_ ( .d1(areg[57]), .z(ain[56]), .d0(areg[56]), .s(x2));
mul_mux2 sh_55_ ( .d1(areg[56]), .z(ain[55]), .d0(areg[55]), .s(x2));
mul_mux2 sh_54_ ( .d1(areg[55]), .z(ain[54]), .d0(areg[54]), .s(x2));
mul_mux2 sh_53_ ( .d1(areg[54]), .z(ain[53]), .d0(areg[53]), .s(x2));
mul_mux2 sh_52_ ( .d1(areg[53]), .z(ain[52]), .d0(areg[52]), .s(x2));
mul_mux2 sh_51_ ( .d1(areg[52]), .z(ain[51]), .d0(areg[51]), .s(x2));
mul_mux2 sh_50_ ( .d1(areg[51]), .z(ain[50]), .d0(areg[50]), .s(x2));
mul_mux2 sh_49_ ( .d1(areg[50]), .z(ain[49]), .d0(areg[49]), .s(x2));
mul_mux2 sh_48_ ( .d1(areg[49]), .z(ain[48]), .d0(areg[48]), .s(x2));
mul_mux2 sh_47_ ( .d1(areg[48]), .z(ain[47]), .d0(areg[47]), .s(x2));
mul_mux2 sh_46_ ( .d1(areg[47]), .z(ain[46]), .d0(areg[46]), .s(x2));
mul_mux2 sh_45_ ( .d1(areg[46]), .z(ain[45]), .d0(areg[45]), .s(x2));
mul_mux2 sh_44_ ( .d1(areg[45]), .z(ain[44]), .d0(areg[44]), .s(x2));
mul_mux2 sh_43_ ( .d1(areg[44]), .z(ain[43]), .d0(areg[43]), .s(x2));
mul_mux2 sh_42_ ( .d1(areg[43]), .z(ain[42]), .d0(areg[42]), .s(x2));
mul_mux2 sh_41_ ( .d1(areg[42]), .z(ain[41]), .d0(areg[41]), .s(x2));
mul_mux2 sh_40_ ( .d1(areg[41]), .z(ain[40]), .d0(areg[40]), .s(x2));
mul_mux2 sh_39_ ( .d1(areg[40]), .z(ain[39]), .d0(areg[39]), .s(x2));
mul_mux2 sh_38_ ( .d1(areg[39]), .z(ain[38]), .d0(areg[38]), .s(x2));
mul_mux2 sh_37_ ( .d1(areg[38]), .z(ain[37]), .d0(areg[37]), .s(x2));
mul_mux2 sh_36_ ( .d1(areg[37]), .z(ain[36]), .d0(areg[36]), .s(x2));
mul_mux2 sh_35_ ( .d1(areg[36]), .z(ain[35]), .d0(areg[35]), .s(x2));
mul_mux2 sh_34_ ( .d1(areg[35]), .z(ain[34]), .d0(areg[34]), .s(x2));
mul_mux2 sh_33_ ( .d1(areg[34]), .z(ain[33]), .d0(areg[33]), .s(x2));
mul_mux2 sh_32_ ( .d1(areg[33]), .z(ain[32]), .d0(areg[32]), .s(x2));
mul_mux2 sh_31_ ( .d1(areg[32]), .z(ain[31]), .d0(areg[31]), .s(x2));
mul_mux2 sh_30_ ( .d1(areg[31]), .z(ain[30]), .d0(areg[30]), .s(x2));
mul_mux2 sh_29_ ( .d1(areg[30]), .z(ain[29]), .d0(areg[29]), .s(x2));
mul_mux2 sh_28_ ( .d1(areg[29]), .z(ain[28]), .d0(areg[28]), .s(x2));
mul_mux2 sh_27_ ( .d1(areg[28]), .z(ain[27]), .d0(areg[27]), .s(x2));
mul_mux2 sh_26_ ( .d1(areg[27]), .z(ain[26]), .d0(areg[26]), .s(x2));
mul_mux2 sh_25_ ( .d1(areg[26]), .z(ain[25]), .d0(areg[25]), .s(x2));
mul_mux2 sh_24_ ( .d1(areg[25]), .z(ain[24]), .d0(areg[24]), .s(x2));
mul_mux2 sh_23_ ( .d1(areg[24]), .z(ain[23]), .d0(areg[23]), .s(x2));
mul_mux2 sh_22_ ( .d1(areg[23]), .z(ain[22]), .d0(areg[22]), .s(x2));
mul_mux2 sh_21_ ( .d1(areg[22]), .z(ain[21]), .d0(areg[21]), .s(x2));
mul_mux2 sh_20_ ( .d1(areg[21]), .z(ain[20]), .d0(areg[20]), .s(x2));
mul_mux2 sh_96_ ( .d1(1'b0), .z(ain[96]), .d0(areg[96]),
     .s(x2));
mul_mux2 sh_95_ ( .d1(areg[96]), .z(ain[95]), .d0(areg[95]), .s(x2));
mul_mux2 sh_94_ ( .d1(areg[95]), .z(ain[94]), .d0(areg[94]), .s(x2));
mul_mux2 sh_93_ ( .d1(areg[94]), .z(ain[93]), .d0(areg[93]), .s(x2));
mul_mux2 sh_92_ ( .d1(areg[93]), .z(ain[92]), .d0(areg[92]), .s(x2));
mul_mux2 sh_91_ ( .d1(areg[92]), .z(ain[91]), .d0(areg[91]), .s(x2));
mul_mux2 sh_90_ ( .d1(areg[91]), .z(ain[90]), .d0(areg[90]), .s(x2));
mul_mux2 sh_89_ ( .d1(areg[90]), .z(ain[89]), .d0(areg[89]), .s(x2));
mul_mux2 sh_88_ ( .d1(areg[89]), .z(ain[88]), .d0(areg[88]), .s(x2));
mul_mux2 sh_87_ ( .d1(areg[88]), .z(ain[87]), .d0(areg[87]), .s(x2));
mul_mux2 sh_86_ ( .d1(areg[87]), .z(ain[86]), .d0(areg[86]), .s(x2));
mul_mux2 sh_85_ ( .d1(areg[86]), .z(ain[85]), .d0(areg[85]), .s(x2));
mul_mux2 sh_84_ ( .d1(areg[85]), .z(ain[84]), .d0(areg[84]), .s(x2));
mul_mux2 sh_0_ ( .d1(areg[1]), .z(ain[0]), .d0(areg[0]), .s(x2));
mul_mux2 sh_81_ ( .d1(areg[82]), .z(ain[81]), .d0(areg[81]), .s(x2));
mul_mux2 sh_80_ ( .d1(areg[81]), .z(ain[80]), .d0(areg[80]), .s(x2));
mul_mux2 sh_79_ ( .d1(areg[80]), .z(ain[79]), .d0(areg[79]), .s(x2));
mul_mux2 sh_78_ ( .d1(areg[79]), .z(ain[78]), .d0(areg[78]), .s(x2));
mul_mux2 sh_77_ ( .d1(areg[78]), .z(ain[77]), .d0(areg[77]), .s(x2));
mul_mux2 sh_76_ ( .d1(areg[77]), .z(ain[76]), .d0(areg[76]), .s(x2));
mul_mux2 sh_75_ ( .d1(areg[76]), .z(ain[75]), .d0(areg[75]), .s(x2));
mul_mux2 sh_74_ ( .d1(areg[75]), .z(ain[74]), .d0(areg[74]), .s(x2));
mul_mux2 sh_73_ ( .d1(areg[74]), .z(ain[73]), .d0(areg[73]), .s(x2));
mul_mux2 sh_72_ ( .d1(areg[73]), .z(ain[72]), .d0(areg[72]), .s(x2));
mul_mux2 sh_71_ ( .d1(areg[72]), .z(ain[71]), .d0(areg[71]), .s(x2));
mul_mux2 sh_70_ ( .d1(areg[71]), .z(ain[70]), .d0(areg[70]), .s(x2));
mul_mux2 sh_69_ ( .d1(areg[70]), .z(ain[69]), .d0(areg[69]), .s(x2));
mul_mux2 sh_19_ ( .d1(areg[20]), .z(ain[19]), .d0(areg[19]), .s(x2));
mul_mux2 sh_18_ ( .d1(areg[19]), .z(ain[18]), .d0(areg[18]), .s(x2));
mul_mux2 sh_17_ ( .d1(areg[18]), .z(ain[17]), .d0(areg[17]), .s(x2));
mul_mux2 sh_16_ ( .d1(areg[17]), .z(ain[16]), .d0(areg[16]), .s(x2));
mul_mux2 sh_15_ ( .d1(areg[16]), .z(ain[15]), .d0(areg[15]), .s(x2));
mul_mux2 sh_4_ ( .d1(areg[5]), .z(ain[4]), .d0(areg[4]), .s(x2));
mul_mux2 sh_3_ ( .d1(areg[4]), .z(ain[3]), .d0(areg[3]), .s(x2));
mul_mux2 sh_2_ ( .d1(areg[3]), .z(ain[2]), .d0(areg[2]), .s(x2));
mul_mux2 sh_1_ ( .d1(areg[2]), .z(ain[1]), .d0(areg[1]), .s(x2));
mul_mux2 shx2 ( .d1(areg[0]), .z(ainx2), .d0(1'b0),
     .s(x2));
mul_mux2 sh_83_ ( .d1(areg[84]), .z(ain[83]), .d0(areg[83]), .s(x2));
mul_mux2 sh_14_ ( .d1(areg[15]), .z(ain[14]), .d0(areg[14]), .s(x2));
mul_mux2 sh_13_ ( .d1(areg[14]), .z(ain[13]), .d0(areg[13]), .s(x2));
mul_mux2 sh_12_ ( .d1(areg[13]), .z(ain[12]), .d0(areg[12]), .s(x2));
mul_mux2 sh_11_ ( .d1(areg[12]), .z(ain[11]), .d0(areg[11]), .s(x2));
mul_mux2 sh_10_ ( .d1(areg[11]), .z(ain[10]), .d0(areg[10]), .s(x2));
mul_mux2 sh_9_ ( .d1(areg[10]), .z(ain[9]), .d0(areg[9]), .s(x2));
mul_mux2 sh_8_ ( .d1(areg[9]), .z(ain[8]), .d0(areg[8]), .s(x2));
mul_mux2 sh_7_ ( .d1(areg[8]), .z(ain[7]), .d0(areg[7]), .s(x2));
mul_mux2 sh_6_ ( .d1(areg[7]), .z(ain[6]), .d0(areg[6]), .s(x2));
mul_mux2 sh_5_ ( .d1(areg[6]), .z(ain[5]), .d0(areg[5]), .s(x2));
mul_csa42  sc3_68_ ( .cin(co[67]), .d(1'b0),
     .carry(c3[68]), .c(c2[67]), .b(s2[68]), .a(1'b0),
     .cout(), .sum(s3[68]));
mul_csa42  sc3_67_ ( .cin(co[66]), .d(1'b0),
     .carry(c3[67]), .c(c2[66]), .b(s2[67]), .a(s1[67]), .cout(co[67]),
     .sum(s3[67]));
mul_csa42  sc3_66_ ( .cin(co[65]), .d(c1[65]), .carry(c3[66]),
     .c(c2[65]), .b(s2[66]), .a(s1[66]), .cout(co[66]), .sum(s3[66]));
mul_csa42  sc3_65_ ( .cin(co[64]), .d(c1[64]), .carry(c3[65]),
     .c(c2[64]), .b(s2[65]), .a(s1[65]), .cout(co[65]), .sum(s3[65]));
mul_csa42  sc3_64_ ( .cin(co[63]), .d(c1[63]), .carry(c3[64]),
     .c(c2[63]), .b(s2[64]), .a(s1[64]), .cout(co[64]), .sum(s3[64]));
mul_csa42  sc3_63_ ( .cin(co[62]), .d(c1[62]), .carry(c3[63]),
     .c(c2[62]), .b(s2[63]), .a(s1[63]), .cout(co[63]), .sum(s3[63]));
mul_csa42  sc3_62_ ( .cin(co[61]), .d(c1[61]), .carry(c3[62]),
     .c(c2[61]), .b(s2[62]), .a(s1[62]), .cout(co[62]), .sum(s3[62]));
mul_csa42  sc3_61_ ( .cin(co[60]), .d(c1[60]), .carry(c3[61]),
     .c(c2[60]), .b(s2[61]), .a(s1[61]), .cout(co[61]), .sum(s3[61]));
mul_csa42  sc3_60_ ( .cin(co[59]), .d(c1[59]), .carry(c3[60]),
     .c(c2[59]), .b(s2[60]), .a(s1[60]), .cout(co[60]), .sum(s3[60]));
mul_csa42  sc3_59_ ( .cin(co[58]), .d(c1[58]), .carry(c3[59]),
     .c(c2[58]), .b(s2[59]), .a(s1[59]), .cout(co[59]), .sum(s3[59]));
mul_csa42  sc3_58_ ( .cin(co[57]), .d(c1[57]), .carry(c3[58]),
     .c(c2[57]), .b(s2[58]), .a(s1[58]), .cout(co[58]), .sum(s3[58]));
mul_csa42  sc3_57_ ( .cin(co[56]), .d(c1[56]), .carry(c3[57]),
     .c(c2[56]), .b(s2[57]), .a(s1[57]), .cout(co[57]), .sum(s3[57]));
mul_csa42  sc3_56_ ( .cin(co[55]), .d(c1[55]), .carry(c3[56]),
     .c(c2[55]), .b(s2[56]), .a(s1[56]), .cout(co[56]), .sum(s3[56]));
mul_csa42  sc3_55_ ( .cin(co[54]), .d(c1[54]), .carry(c3[55]),
     .c(c2[54]), .b(s2[55]), .a(s1[55]), .cout(co[55]), .sum(s3[55]));
mul_csa42  sc3_54_ ( .cin(co[53]), .d(c1[53]), .carry(c3[54]),
     .c(c2[53]), .b(s2[54]), .a(s1[54]), .cout(co[54]), .sum(s3[54]));
mul_csa42  sc3_53_ ( .cin(co[52]), .d(c1[52]), .carry(c3[53]),
     .c(c2[52]), .b(s2[53]), .a(s1[53]), .cout(co[53]), .sum(s3[53]));
mul_csa42  sc3_52_ ( .cin(co[51]), .d(c1[51]), .carry(c3[52]),
     .c(c2[51]), .b(s2[52]), .a(s1[52]), .cout(co[52]), .sum(s3[52]));
mul_csa42  sc3_51_ ( .cin(co[50]), .d(c1[50]), .carry(c3[51]),
     .c(c2[50]), .b(s2[51]), .a(s1[51]), .cout(co[51]), .sum(s3[51]));
mul_csa42  sc3_50_ ( .cin(co[49]), .d(c1[49]), .carry(c3[50]),
     .c(c2[49]), .b(s2[50]), .a(s1[50]), .cout(co[50]), .sum(s3[50]));
mul_csa42  sc3_49_ ( .cin(co[48]), .d(c1[48]), .carry(c3[49]),
     .c(c2[48]), .b(s2[49]), .a(s1[49]), .cout(co[49]), .sum(s3[49]));
mul_csa42  sc3_48_ ( .cin(co[47]), .d(c1[47]), .carry(c3[48]),
     .c(c2[47]), .b(s2[48]), .a(s1[48]), .cout(co[48]), .sum(s3[48]));
mul_csa42  sc3_47_ ( .cin(co[46]), .d(c1[46]), .carry(c3[47]),
     .c(c2[46]), .b(s2[47]), .a(s1[47]), .cout(co[47]), .sum(s3[47]));
mul_csa42  sc3_46_ ( .cin(co[45]), .d(c1[45]), .carry(c3[46]),
     .c(c2[45]), .b(s2[46]), .a(s1[46]), .cout(co[46]), .sum(s3[46]));
mul_csa42  sc3_45_ ( .cin(co[44]), .d(c1[44]), .carry(c3[45]),
     .c(c2[44]), .b(s2[45]), .a(s1[45]), .cout(co[45]), .sum(s3[45]));
mul_csa42  sc3_44_ ( .cin(co[43]), .d(c1[43]), .carry(c3[44]),
     .c(c2[43]), .b(s2[44]), .a(s1[44]), .cout(co[44]), .sum(s3[44]));
mul_csa42  sc3_43_ ( .cin(co[42]), .d(c1[42]), .carry(c3[43]),
     .c(c2[42]), .b(s2[43]), .a(s1[43]), .cout(co[43]), .sum(s3[43]));
mul_csa42  sc3_42_ ( .cin(co[41]), .d(c1[41]), .carry(c3[42]),
     .c(c2[41]), .b(s2[42]), .a(s1[42]), .cout(co[42]), .sum(s3[42]));
mul_csa42  sc3_41_ ( .cin(co[40]), .d(c1[40]), .carry(c3[41]),
     .c(c2[40]), .b(s2[41]), .a(s1[41]), .cout(co[41]), .sum(s3[41]));
mul_csa42  sc3_40_ ( .cin(co[39]), .d(c1[39]), .carry(c3[40]),
     .c(c2[39]), .b(s2[40]), .a(s1[40]), .cout(co[40]), .sum(s3[40]));
mul_csa42  sc3_39_ ( .cin(co[38]), .d(c1[38]), .carry(c3[39]),
     .c(c2[38]), .b(s2[39]), .a(s1[39]), .cout(co[39]), .sum(s3[39]));
mul_csa42  sc3_38_ ( .cin(co[37]), .d(c1[37]), .carry(c3[38]),
     .c(c2[37]), .b(s2[38]), .a(s1[38]), .cout(co[38]), .sum(s3[38]));
mul_csa42  sc3_37_ ( .cin(co[36]), .d(c1[36]), .carry(c3[37]),
     .c(c2[36]), .b(s2[37]), .a(s1[37]), .cout(co[37]), .sum(s3[37]));
mul_csa42  sc3_36_ ( .cin(co[35]), .d(c1[35]), .carry(c3[36]),
     .c(c2[35]), .b(s2[36]), .a(s1[36]), .cout(co[36]), .sum(s3[36]));
mul_csa42  sc3_35_ ( .cin(co[34]), .d(c1[34]), .carry(c3[35]),
     .c(c2[34]), .b(s2[35]), .a(s1[35]), .cout(co[35]), .sum(s3[35]));
mul_csa42  sc3_34_ ( .cin(co[33]), .d(c1[33]), .carry(c3[34]),
     .c(c2[33]), .b(s2[34]), .a(s1[34]), .cout(co[34]), .sum(s3[34]));
mul_csa42  sc3_33_ ( .cin(co[32]), .d(c1[32]), .carry(c3[33]),
     .c(c2[32]), .b(s2[33]), .a(s1[33]), .cout(co[33]), .sum(s3[33]));
mul_csa42  sc3_32_ ( .cin(co[31]), .d(c1[31]), .carry(c3[32]),
     .c(c2[31]), .b(s2[32]), .a(s1[32]), .cout(co[32]), .sum(s3[32]));
mul_csa42  sc3_31_ ( .cin(co[30]), .d(c1[30]), .carry(c3[31]),
     .c(c2[30]), .b(s2[31]), .a(s1[31]), .cout(co[31]), .sum(s3[31]));
mul_csa42  sc3_30_ ( .cin(co[29]), .d(c1[29]), .carry(c3[30]),
     .c(c2[29]), .b(s2[30]), .a(s1[30]), .cout(co[30]), .sum(s3[30]));
mul_csa42  sc3_29_ ( .cin(co[28]), .d(c1[28]), .carry(c3[29]),
     .c(c2[28]), .b(s2[29]), .a(s1[29]), .cout(co[29]), .sum(s3[29]));
mul_csa42  sc3_28_ ( .cin(co[27]), .d(c1[27]), .carry(c3[28]),
     .c(c2[27]), .b(s2[28]), .a(s1[28]), .cout(co[28]), .sum(s3[28]));
mul_csa42  sc3_27_ ( .cin(co[26]), .d(c1[26]), .carry(c3[27]),
     .c(c2[26]), .b(s2[27]), .a(s1[27]), .cout(co[27]), .sum(s3[27]));
mul_csa42  sc3_26_ ( .cin(co[25]), .d(c1[25]), .carry(c3[26]),
     .c(c2[25]), .b(s2[26]), .a(s1[26]), .cout(co[26]), .sum(s3[26]));
mul_csa42  sc3_25_ ( .cin(co[24]), .d(c1[24]), .carry(c3[25]),
     .c(c2[24]), .b(s2[25]), .a(s1[25]), .cout(co[25]), .sum(s3[25]));
mul_csa42  sc3_24_ ( .cin(co[23]), .d(c1[23]), .carry(c3[24]),
     .c(c2[23]), .b(s2[24]), .a(s1[24]), .cout(co[24]), .sum(s3[24]));
mul_csa42  sc3_23_ ( .cin(co[22]), .d(c1[22]), .carry(c3[23]),
     .c(c2[22]), .b(s2[23]), .a(s1[23]), .cout(co[23]), .sum(s3[23]));
mul_csa42  sc3_22_ ( .cin(co[21]), .d(c1[21]), .carry(c3[22]),
     .c(c2[21]), .b(s2[22]), .a(s1[22]), .cout(co[22]), .sum(s3[22]));
mul_csa42  sc3_21_ ( .cin(co[20]), .d(c1[20]), .carry(c3[21]),
     .c(c2[20]), .b(s2[21]), .a(s1[21]), .cout(co[21]), .sum(s3[21]));
mul_csa42  sc3_20_ ( .cin(1'b0), .d(c1[19]),
     .carry(c3[20]), .c(c2[19]), .b(s2[20]), .a(s1[20]), .cout(co[20]),
     .sum(s3[20]));
mul_csa32  sc4_82_ ( .c(c3[81]), .b(s2[82]), .a(ain[82]),
     .cout(pcout[82]), .sum(psum[82]));
mul_csa32  sc4_68_ ( .c(c3[67]), .b(s3[68]), .a(ain[68]),
     .cout(pcout[68]), .sum(psum[68]));
mul_csa32  sc4_67_ ( .c(c3[66]), .b(s3[67]), .a(ain[67]),
     .cout(pcout[67]), .sum(psum[67]));
mul_csa32  sc4_66_ ( .c(c3[65]), .b(s3[66]), .a(ain[66]),
     .cout(pcout[66]), .sum(psum[66]));
mul_csa32  sc4_65_ ( .c(c3[64]), .b(s3[65]), .a(ain[65]),
     .cout(pcout[65]), .sum(psum[65]));
mul_csa32  sc4_64_ ( .c(c3[63]), .b(s3[64]), .a(ain[64]),
     .cout(pcout[64]), .sum(psum[64]));
mul_csa32  sc4_63_ ( .c(c3[62]), .b(s3[63]), .a(ain[63]),
     .cout(pcout[63]), .sum(psum[63]));
mul_csa32  sc4_62_ ( .c(c3[61]), .b(s3[62]), .a(ain[62]),
     .cout(pcout[62]), .sum(psum[62]));
mul_csa32  sc4_61_ ( .c(c3[60]), .b(s3[61]), .a(ain[61]),
     .cout(pcout[61]), .sum(psum[61]));
mul_csa32  sc4_60_ ( .c(c3[59]), .b(s3[60]), .a(ain[60]),
     .cout(pcout[60]), .sum(psum[60]));
mul_csa32  sc4_59_ ( .c(c3[58]), .b(s3[59]), .a(ain[59]),
     .cout(pcout[59]), .sum(psum[59]));
mul_csa32  sc4_58_ ( .c(c3[57]), .b(s3[58]), .a(ain[58]),
     .cout(pcout[58]), .sum(psum[58]));
mul_csa32  sc4_57_ ( .c(c3[56]), .b(s3[57]), .a(ain[57]),
     .cout(pcout[57]), .sum(psum[57]));
mul_csa32  sc4_56_ ( .c(c3[55]), .b(s3[56]), .a(ain[56]),
     .cout(pcout[56]), .sum(psum[56]));
mul_csa32  sc4_55_ ( .c(c3[54]), .b(s3[55]), .a(ain[55]),
     .cout(pcout[55]), .sum(psum[55]));
mul_csa32  sc4_54_ ( .c(c3[53]), .b(s3[54]), .a(ain[54]),
     .cout(pcout[54]), .sum(psum[54]));
mul_csa32  sc4_53_ ( .c(c3[52]), .b(s3[53]), .a(ain[53]),
     .cout(pcout[53]), .sum(psum[53]));
mul_csa32  sc4_52_ ( .c(c3[51]), .b(s3[52]), .a(ain[52]),
     .cout(pcout[52]), .sum(psum[52]));
mul_csa32  sc4_51_ ( .c(c3[50]), .b(s3[51]), .a(ain[51]),
     .cout(pcout[51]), .sum(psum[51]));
mul_csa32  sc4_50_ ( .c(c3[49]), .b(s3[50]), .a(ain[50]),
     .cout(pcout[50]), .sum(psum[50]));
mul_csa32  sc4_49_ ( .c(c3[48]), .b(s3[49]), .a(ain[49]),
     .cout(pcout[49]), .sum(psum[49]));
mul_csa32  sc4_48_ ( .c(c3[47]), .b(s3[48]), .a(ain[48]),
     .cout(pcout[48]), .sum(psum[48]));
mul_csa32  sc4_47_ ( .c(c3[46]), .b(s3[47]), .a(ain[47]),
     .cout(pcout[47]), .sum(psum[47]));
mul_csa32  sc4_46_ ( .c(c3[45]), .b(s3[46]), .a(ain[46]),
     .cout(pcout[46]), .sum(psum[46]));
mul_csa32  sc4_45_ ( .c(c3[44]), .b(s3[45]), .a(ain[45]),
     .cout(pcout[45]), .sum(psum[45]));
mul_csa32  sc4_44_ ( .c(c3[43]), .b(s3[44]), .a(ain[44]),
     .cout(pcout[44]), .sum(psum[44]));
mul_csa32  sc4_43_ ( .c(c3[42]), .b(s3[43]), .a(ain[43]),
     .cout(pcout[43]), .sum(psum[43]));
mul_csa32  sc4_42_ ( .c(c3[41]), .b(s3[42]), .a(ain[42]),
     .cout(pcout[42]), .sum(psum[42]));
mul_csa32  sc4_41_ ( .c(c3[40]), .b(s3[41]), .a(ain[41]),
     .cout(pcout[41]), .sum(psum[41]));
mul_csa32  sc4_40_ ( .c(c3[39]), .b(s3[40]), .a(ain[40]),
     .cout(pcout[40]), .sum(psum[40]));
mul_csa32  sc4_39_ ( .c(c3[38]), .b(s3[39]), .a(ain[39]),
     .cout(pcout[39]), .sum(psum[39]));
mul_csa32  sc4_38_ ( .c(c3[37]), .b(s3[38]), .a(ain[38]),
     .cout(pcout[38]), .sum(psum[38]));
mul_csa32  sc4_37_ ( .c(c3[36]), .b(s3[37]), .a(ain[37]),
     .cout(pcout[37]), .sum(psum[37]));
mul_csa32  sc4_36_ ( .c(c3[35]), .b(s3[36]), .a(ain[36]),
     .cout(pcout[36]), .sum(psum[36]));
mul_csa32  sc4_35_ ( .c(c3[34]), .b(s3[35]), .a(ain[35]),
     .cout(pcout[35]), .sum(psum[35]));
mul_csa32  sc4_34_ ( .c(c3[33]), .b(s3[34]), .a(ain[34]),
     .cout(pcout[34]), .sum(psum[34]));
mul_csa32  sc4_33_ ( .c(c3[32]), .b(s3[33]), .a(ain[33]),
     .cout(pcout[33]), .sum(psum[33]));
mul_csa32  sc4_32_ ( .c(c3[31]), .b(s3[32]), .a(ain[32]),
     .cout(pcout[32]), .sum(psum[32]));
mul_csa32  sc4_31_ ( .c(c3[30]), .b(s3[31]), .a(ain[31]),
     .cout(pcout[31]), .sum(psum[31]));
mul_csa32  sc4_30_ ( .c(c3[29]), .b(s3[30]), .a(ain[30]),
     .cout(pcout[30]), .sum(psum[30]));
mul_csa32  sc4_29_ ( .c(c3[28]), .b(s3[29]), .a(ain[29]),
     .cout(pcout[29]), .sum(psum[29]));
mul_csa32  sc4_28_ ( .c(c3[27]), .b(s3[28]), .a(ain[28]),
     .cout(pcout[28]), .sum(psum[28]));
mul_csa32  sc4_27_ ( .c(c3[26]), .b(s3[27]), .a(ain[27]),
     .cout(pcout[27]), .sum(psum[27]));
mul_csa32  sc4_26_ ( .c(c3[25]), .b(s3[26]), .a(ain[26]),
     .cout(pcout[26]), .sum(psum[26]));
mul_csa32  sc4_25_ ( .c(c3[24]), .b(s3[25]), .a(ain[25]),
     .cout(pcout[25]), .sum(psum[25]));
mul_csa32  sc4_24_ ( .c(c3[23]), .b(s3[24]), .a(ain[24]),
     .cout(pcout[24]), .sum(psum[24]));
mul_csa32  sc4_23_ ( .c(c3[22]), .b(s3[23]), .a(ain[23]),
     .cout(pcout[23]), .sum(psum[23]));
mul_csa32  sc4_22_ ( .c(c3[21]), .b(s3[22]), .a(ain[22]),
     .cout(pcout[22]), .sum(psum[22]));
mul_csa32  sc4_21_ ( .c(c3[20]), .b(s3[21]), .a(ain[21]),
     .cout(pcout[21]), .sum(psum[21]));
mul_csa32  sc4_20_ ( .c(c3[19]), .b(s3[20]), .a(ain[20]),
     .cout(pcout[20]), .sum(psum[20]));
mul_csa32  sc4_96_ ( .c(c2[95]), .b(s2[96]), .a(ain[96]),
     .cout(pcout[96]), .sum(psum[96]));
mul_csa32  sc4_95_ ( .c(c2[94]), .b(s2[95]), .a(ain[95]),
     .cout(pcout[95]), .sum(psum[95]));
mul_csa32  sc4_94_ ( .c(c2[93]), .b(s2[94]), .a(ain[94]),
     .cout(pcout[94]), .sum(psum[94]));
mul_csa32  sc4_93_ ( .c(c2[92]), .b(s2[93]), .a(ain[93]),
     .cout(pcout[93]), .sum(psum[93]));
mul_csa32  sc4_92_ ( .c(c2[91]), .b(s2[92]), .a(ain[92]),
     .cout(pcout[92]), .sum(psum[92]));
mul_csa32  sc4_91_ ( .c(c2[90]), .b(s2[91]), .a(ain[91]),
     .cout(pcout[91]), .sum(psum[91]));
mul_csa32  sc4_90_ ( .c(c2[89]), .b(s2[90]), .a(ain[90]),
     .cout(pcout[90]), .sum(psum[90]));
mul_csa32  sc4_89_ ( .c(c2[88]), .b(s2[89]), .a(ain[89]),
     .cout(pcout[89]), .sum(psum[89]));
mul_csa32  sc4_88_ ( .c(c2[87]), .b(s2[88]), .a(ain[88]),
     .cout(pcout[88]), .sum(psum[88]));
mul_csa32  sc4_87_ ( .c(c2[86]), .b(s2[87]), .a(ain[87]),
     .cout(pcout[87]), .sum(psum[87]));
mul_csa32  sc4_86_ ( .c(c2[85]), .b(s2[86]), .a(ain[86]),
     .cout(pcout[86]), .sum(psum[86]));
mul_csa32  sc4_85_ ( .c(c2[84]), .b(s2[85]), .a(ain[85]),
     .cout(pcout[85]), .sum(psum[85]));
mul_csa32  sc4_84_ ( .c(c2[83]), .b(s2[84]), .a(ain[84]),
     .cout(pcout[84]), .sum(psum[84]));
mul_csa32  sc4_81_ ( .c(c3[80]), .b(s3[81]), .a(ain[81]),
     .cout(pcout[81]), .sum(psum[81]));
mul_csa32  sc4_80_ ( .c(c3[79]), .b(s3[80]), .a(ain[80]),
     .cout(pcout[80]), .sum(psum[80]));
mul_csa32  sc4_79_ ( .c(c3[78]), .b(s3[79]), .a(ain[79]),
     .cout(pcout[79]), .sum(psum[79]));
mul_csa32  sc4_78_ ( .c(c3[77]), .b(s3[78]), .a(ain[78]),
     .cout(pcout[78]), .sum(psum[78]));
mul_csa32  sc4_77_ ( .c(c3[76]), .b(s3[77]), .a(ain[77]),
     .cout(pcout[77]), .sum(psum[77]));
mul_csa32  sc4_76_ ( .c(c3[75]), .b(s3[76]), .a(ain[76]),
     .cout(pcout[76]), .sum(psum[76]));
mul_csa32  sc4_75_ ( .c(c3[74]), .b(s3[75]), .a(ain[75]),
     .cout(pcout[75]), .sum(psum[75]));
mul_csa32  sc4_74_ ( .c(c3[73]), .b(s3[74]), .a(ain[74]),
     .cout(pcout[74]), .sum(psum[74]));
mul_csa32  sc4_73_ ( .c(c3[72]), .b(s3[73]), .a(ain[73]),
     .cout(pcout[73]), .sum(psum[73]));
mul_csa32  sc4_72_ ( .c(c3[71]), .b(s3[72]), .a(ain[72]),
     .cout(pcout[72]), .sum(psum[72]));
mul_csa32  sc4_71_ ( .c(c3[70]), .b(s3[71]), .a(ain[71]),
     .cout(pcout[71]), .sum(psum[71]));
mul_csa32  sc4_70_ ( .c(c3[69]), .b(s3[70]), .a(ain[70]),
     .cout(pcout[70]), .sum(psum[70]));
mul_csa32  sc4_69_ ( .c(c3[68]), .b(s3[69]), .a(ain[69]),
     .cout(pcout[69]), .sum(psum[69]));
mul_csa32  acc_4_ ( .c(c2[3]), .sum(psum[4]), .cout(pcout[4]),
     .a(ain[4]), .b(s2[4]));
mul_csa32  acc_3_ ( .c(c2[2]), .sum(psum[3]), .cout(pcout[3]),
     .a(ain[3]), .b(s2[3]));
mul_csa32  acc_2_ ( .c(c2[1]), .sum(psum[2]), .cout(pcout[2]),
     .a(ain[2]), .b(s2[2]));
mul_csa32  acc_1_ ( .c(c2[0]), .sum(psum[1]), .cout(pcout[1]),
     .a(ain[1]), .b(s2[1]));
mul_csa32  sc3_97_ ( .c(c2[96]), .sum(psum[97]), .cout(pcout[97]),
     .a(a1s[81]), .b(a1c[80]));
mul_csa32  sc1_19_ ( .c(a1s[3]), .b(pc[50]), .a(ps[51]),
     .cout(c1[19]), .sum(s1[19]));
mul_csa32  sc1_18_ ( .c(a1s[2]), .b(pc[49]), .a(ps[50]),
     .cout(c1[18]), .sum(s1[18]));
mul_csa32  sc1_17_ ( .c(a1s[1]), .b(pc[48]), .a(ps[49]),
     .cout(c1[17]), .sum(s1[17]));
mul_csa32  sc1_16_ ( .c(a1s[0]), .b(pc[47]), .a(ps[48]),
     .cout(c1[16]), .sum(s1[16]));
mul_csa32  sc1_15_ ( .c(1'b0), .b(pc[46]), .a(ps[47]),
     .cout(c1[15]), .sum(s1[15]));
mul_csa32  sc4_83_ ( .c(c2[82]), .b(s2[83]), .a(ain[83]),
     .cout(pcout[83]), .sum(psum[83]));
mul_csa32  sc2_83_ ( .c(c1[82]), .b(a1c[66]), .a(a1s[67]),
     .cout(c2[83]), .sum(s2[83]));
mul_csa32  sc2_19_ ( .c(a0c[18]), .b(a0s[19]), .a(s1[19]),
     .cout(c2[19]), .sum(s2[19]));
mul_csa32  sc2_18_ ( .c(a0c[17]), .b(a0s[18]), .a(s1[18]),
     .cout(c2[18]), .sum(s2[18]));
mul_csa32  sc2_17_ ( .c(a0c[16]), .b(a0s[17]), .a(s1[17]),
     .cout(c2[17]), .sum(s2[17]));
mul_csa32  sc2_16_ ( .c(a0c[15]), .b(a0s[16]), .a(s1[16]),
     .cout(c2[16]), .sum(s2[16]));
mul_csa32  sc2_15_ ( .c(a0c[14]), .b(a0s[15]), .a(s1[15]),
     .cout(c2[15]), .sum(s2[15]));
mul_csa32  sc1_81_ ( .c(a0s[81]), .b(a1c[64]), .a(a1s[65]),
     .cout(c1[81]), .sum(s1[81]));
mul_csa32  sc1_80_ ( .c(a0s[80]), .b(a1c[63]), .a(a1s[64]),
     .cout(c1[80]), .sum(s1[80]));
mul_csa32  sc1_79_ ( .c(a0s[79]), .b(a1c[62]), .a(a1s[63]),
     .cout(c1[79]), .sum(s1[79]));
mul_csa32  sc1_78_ ( .c(a0s[78]), .b(a1c[61]), .a(a1s[62]),
     .cout(c1[78]), .sum(s1[78]));
mul_csa32  sc1_77_ ( .c(a0s[77]), .b(a1c[60]), .a(a1s[61]),
     .cout(c1[77]), .sum(s1[77]));
mul_csa32  sc1_76_ ( .c(a0s[76]), .b(a1c[59]), .a(a1s[60]),
     .cout(c1[76]), .sum(s1[76]));
mul_csa32  sc1_75_ ( .c(a0s[75]), .b(a1c[58]), .a(a1s[59]),
     .cout(c1[75]), .sum(s1[75]));
mul_csa32  sc1_74_ ( .c(a0s[74]), .b(a1c[57]), .a(a1s[58]),
     .cout(c1[74]), .sum(s1[74]));
mul_csa32  sc1_73_ ( .c(a0s[73]), .b(a1c[56]), .a(a1s[57]),
     .cout(c1[73]), .sum(s1[73]));
mul_csa32  sc1_72_ ( .c(a0s[72]), .b(a1c[55]), .a(a1s[56]),
     .cout(c1[72]), .sum(s1[72]));
mul_csa32  sc1_71_ ( .c(a0s[71]), .b(a1c[54]), .a(a1s[55]),
     .cout(c1[71]), .sum(s1[71]));
mul_csa32  sc1_70_ ( .c(a0s[70]), .b(a1c[53]), .a(a1s[54]),
     .cout(c1[70]), .sum(s1[70]));
mul_csa32  sc1_69_ ( .c(a0s[69]), .b(a1c[52]), .a(a1s[53]),
     .cout(c1[69]), .sum(s1[69]));
mul_csa32  sc1_68_ ( .c(a0s[68]), .b(a1c[51]), .a(a1s[52]),
     .cout(c1[68]), .sum(s1[68]));
mul_csa32  sc3_19_ ( .c(c2[18]), .b(c1[18]), .a(s2[19]),
     .cout(c3[19]), .sum(s3[19]));
mul_csa32  sc3_18_ ( .c(c2[17]), .b(c1[17]), .a(s2[18]),
     .cout(c3[18]), .sum(s3[18]));
mul_csa32  sc3_17_ ( .c(c2[16]), .b(c1[16]), .a(s2[17]),
     .cout(c3[17]), .sum(s3[17]));
mul_csa32  sc3_16_ ( .c(c2[15]), .b(c1[15]), .a(s2[16]),
     .cout(c3[16]), .sum(s3[16]));
mul_csa32  sc3_15_ ( .c(c2[14]), .b(c1[14]), .a(s2[15]),
     .cout(c3[15]), .sum(s3[15]));
mul_csa32  sc1_82_ ( .c(a0c[81]), .b(a1c[65]), .a(a1s[66]),
     .cout(c1[82]), .sum(s1[82]));
mul_csa32  acc_14_ ( .c(c2[13]), .sum(psum[14]), .cout(pcout[14]),
     .a(ain[14]), .b(s2[14]));
mul_csa32  acc_13_ ( .c(c2[12]), .sum(psum[13]), .cout(pcout[13]),
     .a(ain[13]), .b(s2[13]));
mul_csa32  acc_12_ ( .c(c2[11]), .sum(psum[12]), .cout(pcout[12]),
     .a(ain[12]), .b(s2[12]));
mul_csa32  acc_11_ ( .c(c2[10]), .sum(psum[11]), .cout(pcout[11]),
     .a(ain[11]), .b(s2[11]));
mul_csa32  acc_10_ ( .c(c2[9]), .sum(psum[10]), .cout(pcout[10]),
     .a(ain[10]), .b(s2[10]));
mul_csa32  acc_9_ ( .c(c2[8]), .sum(psum[9]), .cout(pcout[9]),
     .a(ain[9]), .b(s2[9]));
mul_csa32  acc_8_ ( .c(c2[7]), .sum(psum[8]), .cout(pcout[8]),
     .a(ain[8]), .b(s2[8]));
mul_csa32  acc_7_ ( .c(c2[6]), .sum(psum[7]), .cout(pcout[7]),
     .a(ain[7]), .b(s2[7]));
mul_csa32  acc_6_ ( .c(c2[5]), .sum(psum[6]), .cout(pcout[6]),
     .a(ain[6]), .b(s2[6]));
mul_csa32  acc_5_ ( .c(c2[4]), .sum(psum[5]), .cout(pcout[5]),
     .a(ain[5]), .b(s2[5]));
mul_csa32  sc2_67_ ( .c(a0c[66]), .b(c1[66]), .a(a0s[67]),
     .cout(c2[67]), .sum(s2[67]));
mul_csa32  sc1_14_ ( .c(a0s[14]), .b(pc[45]), .a(ps[46]),
     .cout(c1[14]), .sum(s1[14]));
mul_csa32  sc1_13_ ( .c(a0s[13]), .b(pc[44]), .a(ps[45]),
     .cout(c1[13]), .sum(s1[13]));
mul_csa32  sc1_12_ ( .c(a0s[12]), .b(pc[43]), .a(ps[44]),
     .cout(c1[12]), .sum(s1[12]));
mul_csa32  sc1_11_ ( .c(a0s[11]), .b(pc[42]), .a(ps[43]),
     .cout(c1[11]), .sum(s1[11]));
mul_csa32  sc1_10_ ( .c(a0s[10]), .b(pc[41]), .a(ps[42]),
     .cout(c1[10]), .sum(s1[10]));
mul_csa32  sc1_9_ ( .c(a0s[9]), .b(pc[40]), .a(ps[41]), .cout(c1[9]),
     .sum(s1[9]));
mul_csa32  sc1_8_ ( .c(a0s[8]), .b(pc[39]), .a(ps[40]), .cout(c1[8]),
     .sum(s1[8]));
mul_csa32  sc1_7_ ( .c(a0s[7]), .b(pc[38]), .a(ps[39]), .cout(c1[7]),
     .sum(s1[7]));
mul_csa32  sc1_6_ ( .c(a0s[6]), .b(pc[37]), .a(ps[38]), .cout(c1[6]),
     .sum(s1[6]));
mul_csa32  sc1_5_ ( .c(a0s[5]), .b(pc[36]), .a(ps[37]), .cout(c1[5]),
     .sum(s1[5]));
mul_csa32  sc2_14_ ( .c(a0c[13]), .b(c1[13]), .a(s1[14]),
     .cout(c2[14]), .sum(s2[14]));
mul_csa32  sc2_13_ ( .c(a0c[12]), .b(c1[12]), .a(s1[13]),
     .cout(c2[13]), .sum(s2[13]));
mul_csa32  sc2_12_ ( .c(a0c[11]), .b(c1[11]), .a(s1[12]),
     .cout(c2[12]), .sum(s2[12]));
mul_csa32  sc2_11_ ( .c(a0c[10]), .b(c1[10]), .a(s1[11]),
     .cout(c2[11]), .sum(s2[11]));
mul_csa32  sc2_10_ ( .c(a0c[9]), .b(c1[9]), .a(s1[10]),
     .cout(c2[10]), .sum(s2[10]));
mul_csa32  sc2_9_ ( .c(a0c[8]), .b(c1[8]), .a(s1[9]), .cout(c2[9]),
     .sum(s2[9]));
mul_csa32  sc2_8_ ( .c(a0c[7]), .b(c1[7]), .a(s1[8]), .cout(c2[8]),
     .sum(s2[8]));
mul_csa32  sc2_7_ ( .c(a0c[6]), .b(c1[6]), .a(s1[7]), .cout(c2[7]),
     .sum(s2[7]));
mul_csa32  sc2_6_ ( .c(a0c[5]), .b(c1[5]), .a(s1[6]), .cout(c2[6]),
     .sum(s2[6]));
mul_csa32  sc2_5_ ( .c(a0c[4]), .b(c1[4]), .a(s1[5]), .cout(c2[5]),
     .sum(s2[5]));
mul_csa32  sc2_82_ ( .c(c2[81]), .b(c1[81]), .a(s1[82]),
     .cout(c2[82]), .sum(s2[82]));
mul_csa32  sc1_4_ ( .c(a0s[4]), .b(pc[35]), .a(ps[36]), .cout(c1[4]),
     .sum(s1[4]));
mul_csa32  sc1_3_ ( .c(a0s[3]), .b(pc[34]), .a(ps[35]), .cout(c1[3]),
     .sum(s1[3]));
mul_csa32  sc1_2_ ( .c(a0s[2]), .b(pc[33]), .a(ps[34]), .cout(c1[2]),
     .sum(s1[2]));
mul_csa32  sc1_1_ ( .c(a0s[1]), .b(pc[32]), .a(ps[33]), .cout(c1[1]),
     .sum(s1[1]));
mul_csa32  sc2_66_ ( .c(a0c[65]), .b(a0s[66]), .a(a1c[49]),
     .cout(c2[66]), .sum(s2[66]));
mul_csa32  sc2_65_ ( .c(a0c[64]), .b(a0s[65]), .a(a1c[48]),
     .cout(c2[65]), .sum(s2[65]));
mul_csa32  sc2_64_ ( .c(a0c[63]), .b(a0s[64]), .a(a1c[47]),
     .cout(c2[64]), .sum(s2[64]));
mul_csa32  sc2_63_ ( .c(a0c[62]), .b(a0s[63]), .a(a1c[46]),
     .cout(c2[63]), .sum(s2[63]));
mul_csa32  sc2_62_ ( .c(a0c[61]), .b(a0s[62]), .a(a1c[45]),
     .cout(c2[62]), .sum(s2[62]));
mul_csa32  sc2_61_ ( .c(a0c[60]), .b(a0s[61]), .a(a1c[44]),
     .cout(c2[61]), .sum(s2[61]));
mul_csa32  sc2_60_ ( .c(a0c[59]), .b(a0s[60]), .a(a1c[43]),
     .cout(c2[60]), .sum(s2[60]));
mul_csa32  sc2_59_ ( .c(a0c[58]), .b(a0s[59]), .a(a1c[42]),
     .cout(c2[59]), .sum(s2[59]));
mul_csa32  sc2_58_ ( .c(a0c[57]), .b(a0s[58]), .a(a1c[41]),
     .cout(c2[58]), .sum(s2[58]));
mul_csa32  sc2_57_ ( .c(a0c[56]), .b(a0s[57]), .a(a1c[40]),
     .cout(c2[57]), .sum(s2[57]));
mul_csa32  sc2_56_ ( .c(a0c[55]), .b(a0s[56]), .a(a1c[39]),
     .cout(c2[56]), .sum(s2[56]));
mul_csa32  sc2_55_ ( .c(a0c[54]), .b(a0s[55]), .a(a1c[38]),
     .cout(c2[55]), .sum(s2[55]));
mul_csa32  sc2_54_ ( .c(a0c[53]), .b(a0s[54]), .a(a1c[37]),
     .cout(c2[54]), .sum(s2[54]));
mul_csa32  sc2_53_ ( .c(a0c[52]), .b(a0s[53]), .a(a1c[36]),
     .cout(c2[53]), .sum(s2[53]));
mul_csa32  sc2_52_ ( .c(a0c[51]), .b(a0s[52]), .a(a1c[35]),
     .cout(c2[52]), .sum(s2[52]));
mul_csa32  sc2_51_ ( .c(a0c[50]), .b(a0s[51]), .a(a1c[34]),
     .cout(c2[51]), .sum(s2[51]));
mul_csa32  sc2_50_ ( .c(a0c[49]), .b(a0s[50]), .a(a1c[33]),
     .cout(c2[50]), .sum(s2[50]));
mul_csa32  sc2_49_ ( .c(a0c[48]), .b(a0s[49]), .a(a1c[32]),
     .cout(c2[49]), .sum(s2[49]));
mul_csa32  sc2_48_ ( .c(a0c[47]), .b(a0s[48]), .a(a1c[31]),
     .cout(c2[48]), .sum(s2[48]));
mul_csa32  sc2_47_ ( .c(a0c[46]), .b(a0s[47]), .a(a1c[30]),
     .cout(c2[47]), .sum(s2[47]));
mul_csa32  sc2_46_ ( .c(a0c[45]), .b(a0s[46]), .a(a1c[29]),
     .cout(c2[46]), .sum(s2[46]));
mul_csa32  sc2_45_ ( .c(a0c[44]), .b(a0s[45]), .a(a1c[28]),
     .cout(c2[45]), .sum(s2[45]));
mul_csa32  sc2_44_ ( .c(a0c[43]), .b(a0s[44]), .a(a1c[27]),
     .cout(c2[44]), .sum(s2[44]));
mul_csa32  sc2_43_ ( .c(a0c[42]), .b(a0s[43]), .a(a1c[26]),
     .cout(c2[43]), .sum(s2[43]));
mul_csa32  sc2_42_ ( .c(a0c[41]), .b(a0s[42]), .a(a1c[25]),
     .cout(c2[42]), .sum(s2[42]));
mul_csa32  sc2_41_ ( .c(a0c[40]), .b(a0s[41]), .a(a1c[24]),
     .cout(c2[41]), .sum(s2[41]));
mul_csa32  sc2_40_ ( .c(a0c[39]), .b(a0s[40]), .a(a1c[23]),
     .cout(c2[40]), .sum(s2[40]));
mul_csa32  sc2_39_ ( .c(a0c[38]), .b(a0s[39]), .a(a1c[22]),
     .cout(c2[39]), .sum(s2[39]));
mul_csa32  sc2_38_ ( .c(a0c[37]), .b(a0s[38]), .a(a1c[21]),
     .cout(c2[38]), .sum(s2[38]));
mul_csa32  sc2_37_ ( .c(a0c[36]), .b(a0s[37]), .a(a1c[20]),
     .cout(c2[37]), .sum(s2[37]));
mul_csa32  sc2_36_ ( .c(a0c[35]), .b(a0s[36]), .a(a1c[19]),
     .cout(c2[36]), .sum(s2[36]));
mul_csa32  sc2_35_ ( .c(a0c[34]), .b(a0s[35]), .a(a1c[18]),
     .cout(c2[35]), .sum(s2[35]));
mul_csa32  sc2_34_ ( .c(a0c[33]), .b(a0s[34]), .a(a1c[17]),
     .cout(c2[34]), .sum(s2[34]));
mul_csa32  sc2_33_ ( .c(a0c[32]), .b(a0s[33]), .a(a1c[16]),
     .cout(c2[33]), .sum(s2[33]));
mul_csa32  sc2_32_ ( .c(a0c[31]), .b(a0s[32]), .a(a1c[15]),
     .cout(c2[32]), .sum(s2[32]));
mul_csa32  sc2_31_ ( .c(a0c[30]), .b(a0s[31]), .a(a1c[14]),
     .cout(c2[31]), .sum(s2[31]));
mul_csa32  sc2_30_ ( .c(a0c[29]), .b(a0s[30]), .a(a1c[13]),
     .cout(c2[30]), .sum(s2[30]));
mul_csa32  sc2_29_ ( .c(a0c[28]), .b(a0s[29]), .a(a1c[12]),
     .cout(c2[29]), .sum(s2[29]));
mul_csa32  sc2_28_ ( .c(a0c[27]), .b(a0s[28]), .a(a1c[11]),
     .cout(c2[28]), .sum(s2[28]));
mul_csa32  sc2_27_ ( .c(a0c[26]), .b(a0s[27]), .a(a1c[10]),
     .cout(c2[27]), .sum(s2[27]));
mul_csa32  sc2_26_ ( .c(a0c[25]), .b(a0s[26]), .a(a1c[9]),
     .cout(c2[26]), .sum(s2[26]));
mul_csa32  sc2_25_ ( .c(a0c[24]), .b(a0s[25]), .a(a1c[8]),
     .cout(c2[25]), .sum(s2[25]));
mul_csa32  sc2_24_ ( .c(a0c[23]), .b(a0s[24]), .a(a1c[7]),
     .cout(c2[24]), .sum(s2[24]));
mul_csa32  sc2_23_ ( .c(a0c[22]), .b(a0s[23]), .a(a1c[6]),
     .cout(c2[23]), .sum(s2[23]));
mul_csa32  sc2_22_ ( .c(a0c[21]), .b(a0s[22]), .a(a1c[5]),
     .cout(c2[22]), .sum(s2[22]));
mul_csa32  sc2_21_ ( .c(a0c[20]), .b(a0s[21]), .a(a1c[4]),
     .cout(c2[21]), .sum(s2[21]));
mul_csa32  sc2_20_ ( .c(a0c[19]), .b(a0s[20]), .a(1'b0),
     .cout(c2[20]), .sum(s2[20]));
mul_csa32  sc1_66_ ( .c(a1s[50]), .b(pc[97]), .a(ps[98]),
     .cout(c1[66]), .sum(s1[66]));
mul_csa32  sc1_65_ ( .c(a1s[49]), .b(pc[96]), .a(ps[97]),
     .cout(c1[65]), .sum(s1[65]));
mul_csa32  sc1_64_ ( .c(a1s[48]), .b(pc[95]), .a(ps[96]),
     .cout(c1[64]), .sum(s1[64]));
mul_csa32  sc1_63_ ( .c(a1s[47]), .b(pc[94]), .a(ps[95]),
     .cout(c1[63]), .sum(s1[63]));
mul_csa32  sc1_62_ ( .c(a1s[46]), .b(pc[93]), .a(ps[94]),
     .cout(c1[62]), .sum(s1[62]));
mul_csa32  sc1_61_ ( .c(a1s[45]), .b(pc[92]), .a(ps[93]),
     .cout(c1[61]), .sum(s1[61]));
mul_csa32  sc1_60_ ( .c(a1s[44]), .b(pc[91]), .a(ps[92]),
     .cout(c1[60]), .sum(s1[60]));
mul_csa32  sc1_59_ ( .c(a1s[43]), .b(pc[90]), .a(ps[91]),
     .cout(c1[59]), .sum(s1[59]));
mul_csa32  sc1_58_ ( .c(a1s[42]), .b(pc[89]), .a(ps[90]),
     .cout(c1[58]), .sum(s1[58]));
mul_csa32  sc1_57_ ( .c(a1s[41]), .b(pc[88]), .a(ps[89]),
     .cout(c1[57]), .sum(s1[57]));
mul_csa32  sc1_56_ ( .c(a1s[40]), .b(pc[87]), .a(ps[88]),
     .cout(c1[56]), .sum(s1[56]));
mul_csa32  sc1_55_ ( .c(a1s[39]), .b(pc[86]), .a(ps[87]),
     .cout(c1[55]), .sum(s1[55]));
mul_csa32  sc1_54_ ( .c(a1s[38]), .b(pc[85]), .a(ps[86]),
     .cout(c1[54]), .sum(s1[54]));
mul_csa32  sc1_53_ ( .c(a1s[37]), .b(pc[84]), .a(ps[85]),
     .cout(c1[53]), .sum(s1[53]));
mul_csa32  sc1_52_ ( .c(a1s[36]), .b(pc[83]), .a(ps[84]),
     .cout(c1[52]), .sum(s1[52]));
mul_csa32  sc1_51_ ( .c(a1s[35]), .b(pc[82]), .a(ps[83]),
     .cout(c1[51]), .sum(s1[51]));
mul_csa32  sc1_50_ ( .c(a1s[34]), .b(pc[81]), .a(ps[82]),
     .cout(c1[50]), .sum(s1[50]));
mul_csa32  sc1_49_ ( .c(a1s[33]), .b(pc[80]), .a(ps[81]),
     .cout(c1[49]), .sum(s1[49]));
mul_csa32  sc1_48_ ( .c(a1s[32]), .b(pc[79]), .a(ps[80]),
     .cout(c1[48]), .sum(s1[48]));
mul_csa32  sc1_47_ ( .c(a1s[31]), .b(pc[78]), .a(ps[79]),
     .cout(c1[47]), .sum(s1[47]));
mul_csa32  sc1_46_ ( .c(a1s[30]), .b(pc[77]), .a(ps[78]),
     .cout(c1[46]), .sum(s1[46]));
mul_csa32  sc1_45_ ( .c(a1s[29]), .b(pc[76]), .a(ps[77]),
     .cout(c1[45]), .sum(s1[45]));
mul_csa32  sc1_44_ ( .c(a1s[28]), .b(pc[75]), .a(ps[76]),
     .cout(c1[44]), .sum(s1[44]));
mul_csa32  sc1_43_ ( .c(a1s[27]), .b(pc[74]), .a(ps[75]),
     .cout(c1[43]), .sum(s1[43]));
mul_csa32  sc1_42_ ( .c(a1s[26]), .b(pc[73]), .a(ps[74]),
     .cout(c1[42]), .sum(s1[42]));
mul_csa32  sc1_41_ ( .c(a1s[25]), .b(pc[72]), .a(ps[73]),
     .cout(c1[41]), .sum(s1[41]));
mul_csa32  sc1_40_ ( .c(a1s[24]), .b(pc[71]), .a(ps[72]),
     .cout(c1[40]), .sum(s1[40]));
mul_csa32  sc1_39_ ( .c(a1s[23]), .b(pc[70]), .a(ps[71]),
     .cout(c1[39]), .sum(s1[39]));
mul_csa32  sc1_38_ ( .c(a1s[22]), .b(pc[69]), .a(ps[70]),
     .cout(c1[38]), .sum(s1[38]));
mul_csa32  sc1_37_ ( .c(a1s[21]), .b(pc[68]), .a(ps[69]),
     .cout(c1[37]), .sum(s1[37]));
mul_csa32  sc1_36_ ( .c(a1s[20]), .b(pc[67]), .a(ps[68]),
     .cout(c1[36]), .sum(s1[36]));
mul_csa32  sc1_35_ ( .c(a1s[19]), .b(pc[66]), .a(ps[67]),
     .cout(c1[35]), .sum(s1[35]));
mul_csa32  sc1_34_ ( .c(a1s[18]), .b(pc[65]), .a(ps[66]),
     .cout(c1[34]), .sum(s1[34]));
mul_csa32  sc1_33_ ( .c(a1s[17]), .b(pc[64]), .a(ps[65]),
     .cout(c1[33]), .sum(s1[33]));
mul_csa32  sc1_32_ ( .c(a1s[16]), .b(pc[63]), .a(ps[64]),
     .cout(c1[32]), .sum(s1[32]));
mul_csa32  sc1_31_ ( .c(a1s[15]), .b(pc[62]), .a(ps[63]),
     .cout(c1[31]), .sum(s1[31]));
mul_csa32  sc1_30_ ( .c(a1s[14]), .b(pc[61]), .a(ps[62]),
     .cout(c1[30]), .sum(s1[30]));
mul_csa32  sc1_29_ ( .c(a1s[13]), .b(pc[60]), .a(ps[61]),
     .cout(c1[29]), .sum(s1[29]));
mul_csa32  sc1_28_ ( .c(a1s[12]), .b(pc[59]), .a(ps[60]),
     .cout(c1[28]), .sum(s1[28]));
mul_csa32  sc1_27_ ( .c(a1s[11]), .b(pc[58]), .a(ps[59]),
     .cout(c1[27]), .sum(s1[27]));
mul_csa32  sc1_26_ ( .c(a1s[10]), .b(pc[57]), .a(ps[58]),
     .cout(c1[26]), .sum(s1[26]));
mul_csa32  sc1_25_ ( .c(a1s[9]), .b(pc[56]), .a(ps[57]),
     .cout(c1[25]), .sum(s1[25]));
mul_csa32  sc1_24_ ( .c(a1s[8]), .b(pc[55]), .a(ps[56]),
     .cout(c1[24]), .sum(s1[24]));
mul_csa32  sc1_23_ ( .c(a1s[7]), .b(pc[54]), .a(ps[55]),
     .cout(c1[23]), .sum(s1[23]));
mul_csa32  sc1_22_ ( .c(a1s[6]), .b(pc[53]), .a(ps[54]),
     .cout(c1[22]), .sum(s1[22]));
mul_csa32  sc1_21_ ( .c(a1s[5]), .b(pc[52]), .a(ps[53]),
     .cout(c1[21]), .sum(s1[21]));
mul_csa32  sc1_20_ ( .c(a1s[4]), .b(pc[51]), .a(ps[52]),
     .cout(c1[20]), .sum(s1[20]));
mul_csa32  sc2_81_ ( .c(a0c[80]), .b(c1[80]), .a(s1[81]),
     .cout(c2[81]), .sum(s2[81]));
mul_csa32  sc2_80_ ( .c(a0c[79]), .b(c1[79]), .a(s1[80]),
     .cout(c2[80]), .sum(s2[80]));
mul_csa32  sc2_79_ ( .c(a0c[78]), .b(c1[78]), .a(s1[79]),
     .cout(c2[79]), .sum(s2[79]));
mul_csa32  sc2_78_ ( .c(a0c[77]), .b(c1[77]), .a(s1[78]),
     .cout(c2[78]), .sum(s2[78]));
mul_csa32  sc2_77_ ( .c(a0c[76]), .b(c1[76]), .a(s1[77]),
     .cout(c2[77]), .sum(s2[77]));
mul_csa32  sc2_76_ ( .c(a0c[75]), .b(c1[75]), .a(s1[76]),
     .cout(c2[76]), .sum(s2[76]));
mul_csa32  sc2_75_ ( .c(a0c[74]), .b(c1[74]), .a(s1[75]),
     .cout(c2[75]), .sum(s2[75]));
mul_csa32  sc2_74_ ( .c(a0c[73]), .b(c1[73]), .a(s1[74]),
     .cout(c2[74]), .sum(s2[74]));
mul_csa32  sc2_73_ ( .c(a0c[72]), .b(c1[72]), .a(s1[73]),
     .cout(c2[73]), .sum(s2[73]));
mul_csa32  sc2_72_ ( .c(a0c[71]), .b(c1[71]), .a(s1[72]),
     .cout(c2[72]), .sum(s2[72]));
mul_csa32  sc2_71_ ( .c(a0c[70]), .b(c1[70]), .a(s1[71]),
     .cout(c2[71]), .sum(s2[71]));
mul_csa32  sc2_70_ ( .c(a0c[69]), .b(c1[69]), .a(s1[70]),
     .cout(c2[70]), .sum(s2[70]));
mul_csa32  sc2_69_ ( .c(a0c[68]), .b(c1[68]), .a(s1[69]),
     .cout(c2[69]), .sum(s2[69]));
mul_csa32  sc2_68_ ( .c(a0c[67]), .b(c1[67]), .a(s1[68]),
     .cout(c2[68]), .sum(s2[68]));
mul_csa32  acc_19_ ( .c(c3[18]), .b(s3[19]), .a(ain[19]),
     .cout(pcout[19]), .sum(psum[19]));
mul_csa32  acc_18_ ( .c(c3[17]), .b(s3[18]), .a(ain[18]),
     .cout(pcout[18]), .sum(psum[18]));
mul_csa32  acc_17_ ( .c(c3[16]), .b(s3[17]), .a(ain[17]),
     .cout(pcout[17]), .sum(psum[17]));
mul_csa32  acc_16_ ( .c(c3[15]), .b(s3[16]), .a(ain[16]),
     .cout(pcout[16]), .sum(psum[16]));
mul_csa32  acc_15_ ( .c(1'b0), .b(s3[15]), .a(ain[15]),
     .cout(pcout[15]), .sum(psum[15]));
mul_csa32  sc1_0_ ( .c(a0s[0]), .sum(s1[0]), .cout(c1[0]),
     .a(ps[32]), .b(pc[31]));
mul_csa32  sc1_67_ ( .c(a1c[50]), .b(pc[98]), .a(a1s[51]),
     .cout(c1[67]), .sum(s1[67]));
mul_ha acc_0_ ( .sum(psum[0]), .cout(pcout[0]), .a(ain[0]),
     .b(s2[0]));
mul_ha sc3_98_ ( .sum(psum[98]), .cout(pcout[98]), .a(bot),
     .b(a1c[81]));
mul_ha sc2_96_ ( .b(a1c[79]), .a(a1s[80]), .cout(c2[96]),
     .sum(s2[96]));
mul_ha sc2_95_ ( .b(a1c[78]), .a(a1s[79]), .cout(c2[95]),
     .sum(s2[95]));
mul_ha sc2_94_ ( .b(a1c[77]), .a(a1s[78]), .cout(c2[94]),
     .sum(s2[94]));
mul_ha sc2_93_ ( .b(a1c[76]), .a(a1s[77]), .cout(c2[93]),
     .sum(s2[93]));
mul_ha sc2_92_ ( .b(a1c[75]), .a(a1s[76]), .cout(c2[92]),
     .sum(s2[92]));
mul_ha sc2_91_ ( .b(a1c[74]), .a(a1s[75]), .cout(c2[91]),
     .sum(s2[91]));
mul_ha sc2_90_ ( .b(a1c[73]), .a(a1s[74]), .cout(c2[90]),
     .sum(s2[90]));
mul_ha sc2_89_ ( .b(a1c[72]), .a(a1s[73]), .cout(c2[89]),
     .sum(s2[89]));
mul_ha sc2_88_ ( .b(a1c[71]), .a(a1s[72]), .cout(c2[88]),
     .sum(s2[88]));
mul_ha sc2_87_ ( .b(a1c[70]), .a(a1s[71]), .cout(c2[87]),
     .sum(s2[87]));
mul_ha sc2_86_ ( .b(a1c[69]), .a(a1s[70]), .cout(c2[86]),
     .sum(s2[86]));
mul_ha sc2_85_ ( .b(a1c[68]), .a(a1s[69]), .cout(c2[85]),
     .sum(s2[85]));
mul_ha sc2_84_ ( .b(a1c[67]), .a(a1s[68]), .cout(c2[84]),
     .sum(s2[84]));
mul_ha sc3_81_ ( .b(c2[80]), .a(s2[81]), .cout(c3[81]),
     .sum(s3[81]));
mul_ha sc3_80_ ( .b(c2[79]), .a(s2[80]), .cout(c3[80]),
     .sum(s3[80]));
mul_ha sc3_79_ ( .b(c2[78]), .a(s2[79]), .cout(c3[79]),
     .sum(s3[79]));
mul_ha sc3_78_ ( .b(c2[77]), .a(s2[78]), .cout(c3[78]),
     .sum(s3[78]));
mul_ha sc3_77_ ( .b(c2[76]), .a(s2[77]), .cout(c3[77]),
     .sum(s3[77]));
mul_ha sc3_76_ ( .b(c2[75]), .a(s2[76]), .cout(c3[76]),
     .sum(s3[76]));
mul_ha sc3_75_ ( .b(c2[74]), .a(s2[75]), .cout(c3[75]),
     .sum(s3[75]));
mul_ha sc3_74_ ( .b(c2[73]), .a(s2[74]), .cout(c3[74]),
     .sum(s3[74]));
mul_ha sc3_73_ ( .b(c2[72]), .a(s2[73]), .cout(c3[73]),
     .sum(s3[73]));
mul_ha sc3_72_ ( .b(c2[71]), .a(s2[72]), .cout(c3[72]),
     .sum(s3[72]));
mul_ha sc3_71_ ( .b(c2[70]), .a(s2[71]), .cout(c3[71]),
     .sum(s3[71]));
mul_ha sc3_70_ ( .b(c2[69]), .a(s2[70]), .cout(c3[70]),
     .sum(s3[70]));
mul_ha sc3_69_ ( .b(c2[68]), .a(s2[69]), .cout(c3[69]),
     .sum(s3[69]));
mul_ha accx2 ( .sum(psumx2), .cout(pcoutx2), .a(ainx2), .b(s1x2));
mul_ha sc2_4_ ( .sum(s2[4]), .cout(c2[4]), .a(s1[4]), .b(c1[3]));
mul_ha sc2_3_ ( .sum(s2[3]), .cout(c2[3]), .a(s1[3]), .b(c1[2]));
mul_ha sc2_2_ ( .sum(s2[2]), .cout(c2[2]), .a(s1[2]), .b(c1[1]));
mul_ha sc2_1_ ( .sum(s2[1]), .cout(c2[1]), .a(s1[1]), .b(c1[0]));
mul_ha sc2_0_ ( .sum(s2[0]), .cout(c2[0]), .a(s1[0]), .b(c1x2));
mul_ha sc1x2 ( .sum(s1x2), .cout(c1x2), .a(ps[31]), .b(pc[30]));

endmodule //mul_array2

module mul_csa32 (sum, cout, a, b, c);

output sum, cout;
input a, b, c;

wire x, y0, y1, y2;

assign x = a ^ b;
assign sum = c ^ x;

assign y0 = a & b ;
assign y1 = a & c ;
assign y2 = b & c ;

assign cout = y0 | y1 | y2 ;

endmodule //mul_csa32

module mul_csa42 (sum, carry, cout, a, b, c, d, cin);

output sum, carry, cout;
input a, b, c, d, cin;

wire x, y, z;

assign x = a ^ b;
assign y = c ^ d;
assign z = x ^ y;

assign sum = z ^ cin ;

assign carry = (b & ~z) | (cin & z);

assign cout = (d & ~y) | (a & y);

endmodule // mul_csa42

module mul_ha ( cout, sum, a, b );
output  cout, sum;
input  a, b;

assign sum = a ^ b;
assign cout = a & b ;

endmodule //mul_ha

module mul_negen ( n0, n1, b );
output  n0, n1;
input [2:0]  b;

assign n0 = b[2] & b[1] & ~b[0] ;
assign n1 = b[2] & b[1] & b[0] ;

endmodule //mul_negen

module mul_ppgen3lsb4 (cout, p0_l, p1_l, sum, a, b0, b1 );

output  p0_l, p1_l;
output [3:0]  sum;
output [3:1]  cout;
input [3:0]  a;
input [2:0]  b0;
input [2:0]  b1;

wire b0n, b0n_0, b0n_1, b1n_0, b1n_1;
wire p0_0, p0_1, p0_2, p0_3, p1_2, p1_3;
wire p0_l_0, p0_l_1, p0_l_2, p1_l_2;

assign b0n = b0n_1 | (b0n_0 & p0_0) ;
assign sum[0] = b0n_0 ^ p0_0 ;

mul_negen p0n ( .b(b0[2:0]), .n1(b0n_1), .n0(b0n_0));
mul_negen p1n ( .b(b1[2:0]), .n1(b1n_1), .n0(b1n_0));
mul_csa32  sc1_2_ ( .c(b1n_0), .sum(sum[2]), .cout(cout[2]),
     .a(p0_2), .b(p1_2));
mul_csa32  sc1_3_ ( .c(b1n_1), .sum(sum[3]), .cout(cout[3]),
     .a(p0_3), .b(p1_3));
mul_ha sc1_1_ ( .sum(sum[1]), .cout(cout[1]), .a(p0_1),
     .b(b0n));
mul_ppgen p0_3_ ( .pm1_l(p0_l_2), .p_l(p0_l), .b(b0[2:0]), .a(a[3]),
     .z(p0_3));
mul_ppgen p1_3_ ( .pm1_l(p1_l_2), .p_l(p1_l), .b(b1[2:0]), .a(a[1]),
     .z(p1_3));
mul_ppgen p0_2_ ( .pm1_l(p0_l_1), .p_l(p0_l_2), .b(b0[2:0]),
     .a(a[2]), .z(p0_2));
mul_ppgen p0_1_ ( .pm1_l(p0_l_0), .p_l(p0_l_1), .b(b0[2:0]),
     .a(a[1]), .z(p0_1));
mul_ppgen p0_0_ ( .pm1_l(1'b1), .p_l(p0_l_0),
     .b(b0[2:0]), .a(a[0]), .z(p0_0));
mul_ppgen p1_2_ ( .pm1_l(1'b1), .p_l(p1_l_2),
     .b(b1[2:0]), .a(a[0]), .z(p1_2));

endmodule // mul_ppgen3lsb4

module mul_ppgen3sign ( cout, sum, am1, am2, am3, am4, b0, b1, b2,
     bot, head, p0m1_l, p1m1_l, p2m1_l );
input  am1, am2, am3, am4;
input  bot, head, p0m1_l, p1m1_l, p2m1_l;
output [5:0]  sum;
output [4:0]  cout;
input [2:0]  b0;
input [2:0]  b2;
input [2:0]  b1;

wire net37, net42, net075, net088, net0117; 
wire net47, net073, net38, net0118, net078, net8, net15, net43, net48, net35;
wire p2_l_67, p2_l_66, p2_l_65, p2_l_64; 
wire p1_l_65, p1_l_64; 

assign sum[5] = bot & net075 ;
assign net0117 = head & net088 ; 
assign net37 = ~net0117 ;
assign net42 = head ^ net088 ;

mul_ppgensign p0_64_ ( .b(b0[2:0]), .z(net47), .p_l(net088),
     .pm1_l(p0m1_l));
mul_ppgensign p2_68_ ( .pm1_l(p2_l_67), .b(b2[2:0]), .z(net073),
     .p_l(net075));
mul_ppgensign p1_66_ ( .pm1_l(p1_l_65), .b(b1[2:0]), .z(net38),
     .p_l(net0118));
mul_ha sc1_68_ ( .b(net073), .a(1'b1), .cout(cout[4]),
     .sum(sum[4]));
mul_ppgen p2_67_ ( .pm1_l(p2_l_66), .b(b2[2:0]), .a(am1), .z(net078),
     .p_l(p2_l_67));
mul_ppgen p2_66_ ( .pm1_l(p2_l_65), .b(b2[2:0]), .a(am2), .z(net8),
     .p_l(p2_l_66));
mul_ppgen p2_65_ ( .pm1_l(p2_l_64), .p_l(p2_l_65), .b(b2[2:0]),
     .a(am3), .z(net15));
mul_ppgen p1_65_ ( .pm1_l(p1_l_64), .p_l(p1_l_65), .b(b1[2:0]),
     .a(am1), .z(net43));
mul_ppgen p1_64_ ( .pm1_l(p1m1_l), .p_l(p1_l_64), .b(b1[2:0]),
     .a(am2), .z(net48));
mul_ppgen p2_64_ ( .pm1_l(p2m1_l), .p_l(p2_l_64), .b(b2[2:0]),
     .a(am4), .z(net35));
mul_csa32  sc1_67_ ( .c(net078), .b(net0117), .a(net0118),
     .cout(cout[3]), .sum(sum[3]));
mul_csa32  sc1_66_ ( .c(net8), .b(net37), .a(net38), .cout(cout[2]),
     .sum(sum[2]));
mul_csa32  sc1_65_ ( .c(net15), .b(net42), .a(net43), .cout(cout[1]),
     .sum(sum[1]));
mul_csa32  sc1_64_ ( .c(net35), .b(net47), .a(net48), .cout(cout[0]),
     .sum(sum[0]));

endmodule //mul_ppgen3sign

module mul_ppgen3 ( cout, p0_l, p1_l, p2_l, sum, am2, am4,
     a, b0, b1, b2, p0m1_l, p1m1_l, p2m1_l );
output  cout, p0_l, p1_l, p2_l, sum;
input  am2, am4;
input  a, p0m1_l, p1m1_l, p2m1_l;
input [2:0]  b0;
input [2:0]  b2;
input [2:0]  b1;

wire net046, net32, net043;

mul_csa32  sc1 ( .a(net046), .b(net32), .cout(cout), .sum(sum),
     .c(net043));
mul_ppgen p2 ( .pm1_l(p2m1_l), .p_l(p2_l), .b(b2[2:0]), .a(am4),
     .z(net043));
mul_ppgen p1 ( .pm1_l(p1m1_l), .p_l(p1_l), .b(b1[2:0]), .a(am2),
     .z(net046));
mul_ppgen p0 ( .pm1_l(p0m1_l), .p_l(p0_l), .b(b0[2:0]), .a(a),
     .z(net32));

endmodule // mul_ppgen3

module mul_ppgenrow3 ( cout, sum, a, b0, b1, b2, bot, head );

output [68:1]  cout;
output [69:0]  sum;
input [63:0]  a;
input [2:0]  b2;
input [2:0]  b0;
input [2:0]  b1;
input  bot, head;

// Buses in the design
wire  [63:4]  p2_l;
wire  [63:3]  p1_l;
wire  [63:3]  p0_l;

mul_ppgen3sign I2 ( .am4(a[60]), .am3(a[61]), .am2(a[62]),
     .am1(a[63]), .p2m1_l(p2_l[63]), .p1m1_l(p1_l[63]),
     .p0m1_l(p0_l[63]), .b2(b2[2:0]), .head(head), .bot(bot),
     .sum(sum[69:64]), .cout(cout[68:64]), .b1(b1[2:0]), .b0(b0[2:0]));
mul_ppgen3 I1_63_ ( .p2_l(p2_l[63]), .b2(b2[2:0]),
     .am2(a[61]), .a(a[63]), .p2m1_l(p2_l[62]),
     .p1m1_l(p1_l[62]), .p0m1_l(p0_l[62]), .am4(a[59]), .sum(sum[63]),
     .cout(cout[63]), .b1(b1[2:0]), .b0(b0[2:0]), .p1_l(p1_l[63]),
     .p0_l(p0_l[63]));
mul_ppgen3 I1_62_ ( .p2_l(p2_l[62]), .b2(b2[2:0]), 
     .am2(a[60]), .a(a[62]), .p2m1_l(p2_l[61]),
     .p1m1_l(p1_l[61]), .p0m1_l(p0_l[61]), .am4(a[58]), .sum(sum[62]),
     .cout(cout[62]), .b1(b1[2:0]), .b0(b0[2:0]), .p1_l(p1_l[62]),
     .p0_l(p0_l[62]));
mul_ppgen3 I1_61_ ( .p2_l(p2_l[61]), .b2(b2[2:0]), 
     .am2(a[59]), .a(a[61]), .p2m1_l(p2_l[60]),
     .p1m1_l(p1_l[60]), .p0m1_l(p0_l[60]), .am4(a[57]), .sum(sum[61]),
     .cout(cout[61]), .b1(b1[2:0]), .b0(b0[2:0]), .p1_l(p1_l[61]),
     .p0_l(p0_l[61]));
mul_ppgen3 I1_60_ ( .p2_l(p2_l[60]), .b2(b2[2:0]), 
     .am2(a[58]), .a(a[60]), .p2m1_l(p2_l[59]),
     .p1m1_l(p1_l[59]), .p0m1_l(p0_l[59]), .am4(a[56]), .sum(sum[60]),
     .cout(cout[60]), .b1(b1[2:0]), .b0(b0[2:0]), .p1_l(p1_l[60]),
     .p0_l(p0_l[60]));
mul_ppgen3 I1_59_ ( .p2_l(p2_l[59]), .b2(b2[2:0]), 
     .am2(a[57]), .a(a[59]), .p2m1_l(p2_l[58]),
     .p1m1_l(p1_l[58]), .p0m1_l(p0_l[58]), .am4(a[55]), .sum(sum[59]),
     .cout(cout[59]), .b1(b1[2:0]), .b0(b0[2:0]), .p1_l(p1_l[59]),
     .p0_l(p0_l[59]));
mul_ppgen3 I1_58_ ( .p2_l(p2_l[58]), .b2(b2[2:0]), 
     .am2(a[56]), .a(a[58]), .p2m1_l(p2_l[57]),
     .p1m1_l(p1_l[57]), .p0m1_l(p0_l[57]), .am4(a[54]), .sum(sum[58]),
     .cout(cout[58]), .b1(b1[2:0]), .b0(b0[2:0]), .p1_l(p1_l[58]),
     .p0_l(p0_l[58]));
mul_ppgen3 I1_57_ ( .p2_l(p2_l[57]), .b2(b2[2:0]), 
     .am2(a[55]), .a(a[57]), .p2m1_l(p2_l[56]),
     .p1m1_l(p1_l[56]), .p0m1_l(p0_l[56]), .am4(a[53]), .sum(sum[57]),
     .cout(cout[57]), .b1(b1[2:0]), .b0(b0[2:0]), .p1_l(p1_l[57]),
     .p0_l(p0_l[57]));
mul_ppgen3 I1_56_ ( .p2_l(p2_l[56]), .b2(b2[2:0]), 
     .am2(a[54]), .a(a[56]), .p2m1_l(p2_l[55]),
     .p1m1_l(p1_l[55]), .p0m1_l(p0_l[55]), .am4(a[52]), .sum(sum[56]),
     .cout(cout[56]), .b1(b1[2:0]), .b0(b0[2:0]), .p1_l(p1_l[56]),
     .p0_l(p0_l[56]));
mul_ppgen3 I1_55_ ( .p2_l(p2_l[55]), .b2(b2[2:0]), 
     .am2(a[53]), .a(a[55]), .p2m1_l(p2_l[54]),
     .p1m1_l(p1_l[54]), .p0m1_l(p0_l[54]), .am4(a[51]), .sum(sum[55]),
     .cout(cout[55]), .b1(b1[2:0]), .b0(b0[2:0]), .p1_l(p1_l[55]),
     .p0_l(p0_l[55]));
mul_ppgen3 I1_54_ ( .p2_l(p2_l[54]), .b2(b2[2:0]), 
     .am2(a[52]), .a(a[54]), .p2m1_l(p2_l[53]),
     .p1m1_l(p1_l[53]), .p0m1_l(p0_l[53]), .am4(a[50]), .sum(sum[54]),
     .cout(cout[54]), .b1(b1[2:0]), .b0(b0[2:0]), .p1_l(p1_l[54]),
     .p0_l(p0_l[54]));
mul_ppgen3 I1_53_ ( .p2_l(p2_l[53]), .b2(b2[2:0]), 
     .am2(a[51]), .a(a[53]), .p2m1_l(p2_l[52]),
     .p1m1_l(p1_l[52]), .p0m1_l(p0_l[52]), .am4(a[49]), .sum(sum[53]),
     .cout(cout[53]), .b1(b1[2:0]), .b0(b0[2:0]), .p1_l(p1_l[53]),
     .p0_l(p0_l[53]));
mul_ppgen3 I1_52_ ( .p2_l(p2_l[52]), .b2(b2[2:0]), 
     .am2(a[50]), .a(a[52]), .p2m1_l(p2_l[51]),
     .p1m1_l(p1_l[51]), .p0m1_l(p0_l[51]), .am4(a[48]), .sum(sum[52]),
     .cout(cout[52]), .b1(b1[2:0]), .b0(b0[2:0]), .p1_l(p1_l[52]),
     .p0_l(p0_l[52]));
mul_ppgen3 I1_51_ ( .p2_l(p2_l[51]), .b2(b2[2:0]), 
     .am2(a[49]), .a(a[51]), .p2m1_l(p2_l[50]),
     .p1m1_l(p1_l[50]), .p0m1_l(p0_l[50]), .am4(a[47]), .sum(sum[51]),
     .cout(cout[51]), .b1(b1[2:0]), .b0(b0[2:0]), .p1_l(p1_l[51]),
     .p0_l(p0_l[51]));
mul_ppgen3 I1_50_ ( .p2_l(p2_l[50]), .b2(b2[2:0]), 
     .am2(a[48]), .a(a[50]), .p2m1_l(p2_l[49]),
     .p1m1_l(p1_l[49]), .p0m1_l(p0_l[49]), .am4(a[46]), .sum(sum[50]),
     .cout(cout[50]), .b1(b1[2:0]), .b0(b0[2:0]), .p1_l(p1_l[50]),
     .p0_l(p0_l[50]));
mul_ppgen3 I1_49_ ( .p2_l(p2_l[49]), .b2(b2[2:0]), 
     .am2(a[47]), .a(a[49]), .p2m1_l(p2_l[48]),
     .p1m1_l(p1_l[48]), .p0m1_l(p0_l[48]), .am4(a[45]), .sum(sum[49]),
     .cout(cout[49]), .b1(b1[2:0]), .b0(b0[2:0]), .p1_l(p1_l[49]),
     .p0_l(p0_l[49]));
mul_ppgen3 I1_48_ ( .p2_l(p2_l[48]), .b2(b2[2:0]), 
     .am2(a[46]), .a(a[48]), .p2m1_l(p2_l[47]),
     .p1m1_l(p1_l[47]), .p0m1_l(p0_l[47]), .am4(a[44]), .sum(sum[48]),
     .cout(cout[48]), .b1(b1[2:0]), .b0(b0[2:0]), .p1_l(p1_l[48]),
     .p0_l(p0_l[48]));
mul_ppgen3 I1_47_ ( .p2_l(p2_l[47]), .b2(b2[2:0]), 
     .am2(a[45]), .a(a[47]), .p2m1_l(p2_l[46]),
     .p1m1_l(p1_l[46]), .p0m1_l(p0_l[46]), .am4(a[43]), .sum(sum[47]),
     .cout(cout[47]), .b1(b1[2:0]), .b0(b0[2:0]), .p1_l(p1_l[47]),
     .p0_l(p0_l[47]));
mul_ppgen3 I1_46_ ( .p2_l(p2_l[46]), .b2(b2[2:0]), 
     .am2(a[44]), .a(a[46]), .p2m1_l(p2_l[45]),
     .p1m1_l(p1_l[45]), .p0m1_l(p0_l[45]), .am4(a[42]), .sum(sum[46]),
     .cout(cout[46]), .b1(b1[2:0]), .b0(b0[2:0]), .p1_l(p1_l[46]),
     .p0_l(p0_l[46]));
mul_ppgen3 I1_45_ ( .p2_l(p2_l[45]), .b2(b2[2:0]), 
     .am2(a[43]), .a(a[45]), .p2m1_l(p2_l[44]),
     .p1m1_l(p1_l[44]), .p0m1_l(p0_l[44]), .am4(a[41]), .sum(sum[45]),
     .cout(cout[45]), .b1(b1[2:0]), .b0(b0[2:0]), .p1_l(p1_l[45]),
     .p0_l(p0_l[45]));
mul_ppgen3 I1_44_ ( .p2_l(p2_l[44]), .b2(b2[2:0]), 
     .am2(a[42]), .a(a[44]), .p2m1_l(p2_l[43]),
     .p1m1_l(p1_l[43]), .p0m1_l(p0_l[43]), .am4(a[40]), .sum(sum[44]),
     .cout(cout[44]), .b1(b1[2:0]), .b0(b0[2:0]), .p1_l(p1_l[44]),
     .p0_l(p0_l[44]));
mul_ppgen3 I1_43_ ( .p2_l(p2_l[43]), .b2(b2[2:0]), 
     .am2(a[41]), .a(a[43]), .p2m1_l(p2_l[42]),
     .p1m1_l(p1_l[42]), .p0m1_l(p0_l[42]), .am4(a[39]), .sum(sum[43]),
     .cout(cout[43]), .b1(b1[2:0]), .b0(b0[2:0]), .p1_l(p1_l[43]),
     .p0_l(p0_l[43]));
mul_ppgen3 I1_42_ ( .p2_l(p2_l[42]), .b2(b2[2:0]), 
     .am2(a[40]), .a(a[42]), .p2m1_l(p2_l[41]),
     .p1m1_l(p1_l[41]), .p0m1_l(p0_l[41]), .am4(a[38]), .sum(sum[42]),
     .cout(cout[42]), .b1(b1[2:0]), .b0(b0[2:0]), .p1_l(p1_l[42]),
     .p0_l(p0_l[42]));
mul_ppgen3 I1_41_ ( .p2_l(p2_l[41]), .b2(b2[2:0]), 
     .am2(a[39]), .a(a[41]), .p2m1_l(p2_l[40]),
     .p1m1_l(p1_l[40]), .p0m1_l(p0_l[40]), .am4(a[37]), .sum(sum[41]),
     .cout(cout[41]), .b1(b1[2:0]), .b0(b0[2:0]), .p1_l(p1_l[41]),
     .p0_l(p0_l[41]));
mul_ppgen3 I1_40_ ( .p2_l(p2_l[40]), .b2(b2[2:0]), 
     .am2(a[38]), .a(a[40]), .p2m1_l(p2_l[39]),
     .p1m1_l(p1_l[39]), .p0m1_l(p0_l[39]), .am4(a[36]), .sum(sum[40]),
     .cout(cout[40]), .b1(b1[2:0]), .b0(b0[2:0]), .p1_l(p1_l[40]),
     .p0_l(p0_l[40]));
mul_ppgen3 I1_39_ ( .p2_l(p2_l[39]), .b2(b2[2:0]), 
     .am2(a[37]), .a(a[39]), .p2m1_l(p2_l[38]),
     .p1m1_l(p1_l[38]), .p0m1_l(p0_l[38]), .am4(a[35]), .sum(sum[39]),
     .cout(cout[39]), .b1(b1[2:0]), .b0(b0[2:0]), .p1_l(p1_l[39]),
     .p0_l(p0_l[39]));
mul_ppgen3 I1_38_ ( .p2_l(p2_l[38]), .b2(b2[2:0]), 
     .am2(a[36]), .a(a[38]), .p2m1_l(p2_l[37]),
     .p1m1_l(p1_l[37]), .p0m1_l(p0_l[37]), .am4(a[34]), .sum(sum[38]),
     .cout(cout[38]), .b1(b1[2:0]), .b0(b0[2:0]), .p1_l(p1_l[38]),
     .p0_l(p0_l[38]));
mul_ppgen3 I1_37_ ( .p2_l(p2_l[37]), .b2(b2[2:0]), 
     .am2(a[35]), .a(a[37]), .p2m1_l(p2_l[36]),
     .p1m1_l(p1_l[36]), .p0m1_l(p0_l[36]), .am4(a[33]), .sum(sum[37]),
     .cout(cout[37]), .b1(b1[2:0]), .b0(b0[2:0]), .p1_l(p1_l[37]),
     .p0_l(p0_l[37]));
mul_ppgen3 I1_36_ ( .p2_l(p2_l[36]), .b2(b2[2:0]), 
     .am2(a[34]), .a(a[36]), .p2m1_l(p2_l[35]),
     .p1m1_l(p1_l[35]), .p0m1_l(p0_l[35]), .am4(a[32]), .sum(sum[36]),
     .cout(cout[36]), .b1(b1[2:0]), .b0(b0[2:0]), .p1_l(p1_l[36]),
     .p0_l(p0_l[36]));
mul_ppgen3 I1_35_ ( .p2_l(p2_l[35]), .b2(b2[2:0]), 
     .am2(a[33]), .a(a[35]), .p2m1_l(p2_l[34]),
     .p1m1_l(p1_l[34]), .p0m1_l(p0_l[34]), .am4(a[31]), .sum(sum[35]),
     .cout(cout[35]), .b1(b1[2:0]), .b0(b0[2:0]), .p1_l(p1_l[35]),
     .p0_l(p0_l[35]));
mul_ppgen3 I1_34_ ( .p2_l(p2_l[34]), .b2(b2[2:0]), 
     .am2(a[32]), .a(a[34]), .p2m1_l(p2_l[33]),
     .p1m1_l(p1_l[33]), .p0m1_l(p0_l[33]), .am4(a[30]), .sum(sum[34]),
     .cout(cout[34]), .b1(b1[2:0]), .b0(b0[2:0]), .p1_l(p1_l[34]),
     .p0_l(p0_l[34]));
mul_ppgen3 I1_33_ ( .p2_l(p2_l[33]), .b2(b2[2:0]), 
     .am2(a[31]), .a(a[33]), .p2m1_l(p2_l[32]),
     .p1m1_l(p1_l[32]), .p0m1_l(p0_l[32]), .am4(a[29]), .sum(sum[33]),
     .cout(cout[33]), .b1(b1[2:0]), .b0(b0[2:0]), .p1_l(p1_l[33]),
     .p0_l(p0_l[33]));
mul_ppgen3 I1_32_ ( .p2_l(p2_l[32]), .b2(b2[2:0]), 
     .am2(a[30]), .a(a[32]), .p2m1_l(p2_l[31]),
     .p1m1_l(p1_l[31]), .p0m1_l(p0_l[31]), .am4(a[28]), .sum(sum[32]),
     .cout(cout[32]), .b1(b1[2:0]), .b0(b0[2:0]), .p1_l(p1_l[32]),
     .p0_l(p0_l[32]));
mul_ppgen3 I1_31_ ( .p2_l(p2_l[31]), .b2(b2[2:0]), 
     .am2(a[29]), .a(a[31]), .p2m1_l(p2_l[30]),
     .p1m1_l(p1_l[30]), .p0m1_l(p0_l[30]), .am4(a[27]), .sum(sum[31]),
     .cout(cout[31]), .b1(b1[2:0]), .b0(b0[2:0]), .p1_l(p1_l[31]),
     .p0_l(p0_l[31]));
mul_ppgen3 I1_30_ ( .p2_l(p2_l[30]), .b2(b2[2:0]), 
     .am2(a[28]), .a(a[30]), .p2m1_l(p2_l[29]),
     .p1m1_l(p1_l[29]), .p0m1_l(p0_l[29]), .am4(a[26]), .sum(sum[30]),
     .cout(cout[30]), .b1(b1[2:0]), .b0(b0[2:0]), .p1_l(p1_l[30]),
     .p0_l(p0_l[30]));
mul_ppgen3 I1_29_ ( .p2_l(p2_l[29]), .b2(b2[2:0]), 
     .am2(a[27]), .a(a[29]), .p2m1_l(p2_l[28]),
     .p1m1_l(p1_l[28]), .p0m1_l(p0_l[28]), .am4(a[25]), .sum(sum[29]),
     .cout(cout[29]), .b1(b1[2:0]), .b0(b0[2:0]), .p1_l(p1_l[29]),
     .p0_l(p0_l[29]));
mul_ppgen3 I1_28_ ( .p2_l(p2_l[28]), .b2(b2[2:0]), 
     .am2(a[26]), .a(a[28]), .p2m1_l(p2_l[27]),
     .p1m1_l(p1_l[27]), .p0m1_l(p0_l[27]), .am4(a[24]), .sum(sum[28]),
     .cout(cout[28]), .b1(b1[2:0]), .b0(b0[2:0]), .p1_l(p1_l[28]),
     .p0_l(p0_l[28]));
mul_ppgen3 I1_27_ ( .p2_l(p2_l[27]), .b2(b2[2:0]), 
     .am2(a[25]), .a(a[27]), .p2m1_l(p2_l[26]),
     .p1m1_l(p1_l[26]), .p0m1_l(p0_l[26]), .am4(a[23]), .sum(sum[27]),
     .cout(cout[27]), .b1(b1[2:0]), .b0(b0[2:0]), .p1_l(p1_l[27]),
     .p0_l(p0_l[27]));
mul_ppgen3 I1_26_ ( .p2_l(p2_l[26]), .b2(b2[2:0]), 
     .am2(a[24]), .a(a[26]), .p2m1_l(p2_l[25]),
     .p1m1_l(p1_l[25]), .p0m1_l(p0_l[25]), .am4(a[22]), .sum(sum[26]),
     .cout(cout[26]), .b1(b1[2:0]), .b0(b0[2:0]), .p1_l(p1_l[26]),
     .p0_l(p0_l[26]));
mul_ppgen3 I1_25_ ( .p2_l(p2_l[25]), .b2(b2[2:0]), 
     .am2(a[23]), .a(a[25]), .p2m1_l(p2_l[24]),
     .p1m1_l(p1_l[24]), .p0m1_l(p0_l[24]), .am4(a[21]), .sum(sum[25]),
     .cout(cout[25]), .b1(b1[2:0]), .b0(b0[2:0]), .p1_l(p1_l[25]),
     .p0_l(p0_l[25]));
mul_ppgen3 I1_24_ ( .p2_l(p2_l[24]), .b2(b2[2:0]), 
     .am2(a[22]), .a(a[24]), .p2m1_l(p2_l[23]),
     .p1m1_l(p1_l[23]), .p0m1_l(p0_l[23]), .am4(a[20]), .sum(sum[24]),
     .cout(cout[24]), .b1(b1[2:0]), .b0(b0[2:0]), .p1_l(p1_l[24]),
     .p0_l(p0_l[24]));
mul_ppgen3 I1_23_ ( .p2_l(p2_l[23]), .b2(b2[2:0]), 
     .am2(a[21]), .a(a[23]), .p2m1_l(p2_l[22]),
     .p1m1_l(p1_l[22]), .p0m1_l(p0_l[22]), .am4(a[19]), .sum(sum[23]),
     .cout(cout[23]), .b1(b1[2:0]), .b0(b0[2:0]), .p1_l(p1_l[23]),
     .p0_l(p0_l[23]));
mul_ppgen3 I1_22_ ( .p2_l(p2_l[22]), .b2(b2[2:0]), 
     .am2(a[20]), .a(a[22]), .p2m1_l(p2_l[21]),
     .p1m1_l(p1_l[21]), .p0m1_l(p0_l[21]), .am4(a[18]), .sum(sum[22]),
     .cout(cout[22]), .b1(b1[2:0]), .b0(b0[2:0]), .p1_l(p1_l[22]),
     .p0_l(p0_l[22]));
mul_ppgen3 I1_21_ ( .p2_l(p2_l[21]), .b2(b2[2:0]), 
     .am2(a[19]), .a(a[21]), .p2m1_l(p2_l[20]),
     .p1m1_l(p1_l[20]), .p0m1_l(p0_l[20]), .am4(a[17]), .sum(sum[21]),
     .cout(cout[21]), .b1(b1[2:0]), .b0(b0[2:0]), .p1_l(p1_l[21]),
     .p0_l(p0_l[21]));
mul_ppgen3 I1_20_ ( .p2_l(p2_l[20]), .b2(b2[2:0]), 
     .am2(a[18]), .a(a[20]), .p2m1_l(p2_l[19]),
     .p1m1_l(p1_l[19]), .p0m1_l(p0_l[19]), .am4(a[16]), .sum(sum[20]),
     .cout(cout[20]), .b1(b1[2:0]), .b0(b0[2:0]), .p1_l(p1_l[20]),
     .p0_l(p0_l[20]));
mul_ppgen3 I1_19_ ( .p2_l(p2_l[19]), .b2(b2[2:0]), 
     .am2(a[17]), .a(a[19]), .p2m1_l(p2_l[18]),
     .p1m1_l(p1_l[18]), .p0m1_l(p0_l[18]), .am4(a[15]), .sum(sum[19]),
     .cout(cout[19]), .b1(b1[2:0]), .b0(b0[2:0]), .p1_l(p1_l[19]),
     .p0_l(p0_l[19]));
mul_ppgen3 I1_18_ ( .p2_l(p2_l[18]), .b2(b2[2:0]), 
     .am2(a[16]), .a(a[18]), .p2m1_l(p2_l[17]),
     .p1m1_l(p1_l[17]), .p0m1_l(p0_l[17]), .am4(a[14]), .sum(sum[18]),
     .cout(cout[18]), .b1(b1[2:0]), .b0(b0[2:0]), .p1_l(p1_l[18]),
     .p0_l(p0_l[18]));
mul_ppgen3 I1_17_ ( .p2_l(p2_l[17]), .b2(b2[2:0]), 
     .am2(a[15]), .a(a[17]), .p2m1_l(p2_l[16]),
     .p1m1_l(p1_l[16]), .p0m1_l(p0_l[16]), .am4(a[13]), .sum(sum[17]),
     .cout(cout[17]), .b1(b1[2:0]), .b0(b0[2:0]), .p1_l(p1_l[17]),
     .p0_l(p0_l[17]));
mul_ppgen3 I1_16_ ( .p2_l(p2_l[16]), .b2(b2[2:0]), 
     .am2(a[14]), .a(a[16]), .p2m1_l(p2_l[15]),
     .p1m1_l(p1_l[15]), .p0m1_l(p0_l[15]), .am4(a[12]), .sum(sum[16]),
     .cout(cout[16]), .b1(b1[2:0]), .b0(b0[2:0]), .p1_l(p1_l[16]),
     .p0_l(p0_l[16]));
mul_ppgen3 I1_15_ ( .p2_l(p2_l[15]), .b2(b2[2:0]), 
     .am2(a[13]), .a(a[15]), .p2m1_l(p2_l[14]),
     .p1m1_l(p1_l[14]), .p0m1_l(p0_l[14]), .am4(a[11]), .sum(sum[15]),
     .cout(cout[15]), .b1(b1[2:0]), .b0(b0[2:0]), .p1_l(p1_l[15]),
     .p0_l(p0_l[15]));
mul_ppgen3 I1_14_ ( .p2_l(p2_l[14]), .b2(b2[2:0]), 
     .am2(a[12]), .a(a[14]), .p2m1_l(p2_l[13]),
     .p1m1_l(p1_l[13]), .p0m1_l(p0_l[13]), .am4(a[10]), .sum(sum[14]),
     .cout(cout[14]), .b1(b1[2:0]), .b0(b0[2:0]), .p1_l(p1_l[14]),
     .p0_l(p0_l[14]));
mul_ppgen3 I1_13_ ( .p2_l(p2_l[13]), .b2(b2[2:0]), 
     .am2(a[11]), .a(a[13]), .p2m1_l(p2_l[12]),
     .p1m1_l(p1_l[12]), .p0m1_l(p0_l[12]), .am4(a[9]), .sum(sum[13]),
     .cout(cout[13]), .b1(b1[2:0]), .b0(b0[2:0]), .p1_l(p1_l[13]),
     .p0_l(p0_l[13]));
mul_ppgen3 I1_12_ ( .p2_l(p2_l[12]), .b2(b2[2:0]), 
     .am2(a[10]), .a(a[12]), .p2m1_l(p2_l[11]),
     .p1m1_l(p1_l[11]), .p0m1_l(p0_l[11]), .am4(a[8]), .sum(sum[12]),
     .cout(cout[12]), .b1(b1[2:0]), .b0(b0[2:0]), .p1_l(p1_l[12]),
     .p0_l(p0_l[12]));
mul_ppgen3 I1_11_ ( .p2_l(p2_l[11]), .b2(b2[2:0]), 
     .am2(a[9]), .a(a[11]), .p2m1_l(p2_l[10]),
     .p1m1_l(p1_l[10]), .p0m1_l(p0_l[10]), .am4(a[7]), .sum(sum[11]),
     .cout(cout[11]), .b1(b1[2:0]), .b0(b0[2:0]), .p1_l(p1_l[11]),
     .p0_l(p0_l[11]));
mul_ppgen3 I1_10_ ( .p2_l(p2_l[10]), .b2(b2[2:0]), 
     .am2(a[8]), .a(a[10]), .p2m1_l(p2_l[9]),
     .p1m1_l(p1_l[9]), .p0m1_l(p0_l[9]), .am4(a[6]), .sum(sum[10]),
     .cout(cout[10]), .b1(b1[2:0]), .b0(b0[2:0]), .p1_l(p1_l[10]),
     .p0_l(p0_l[10]));
mul_ppgen3 I1_9_ ( .p2_l(p2_l[9]), .b2(b2[2:0]), 
     .am2(a[7]), .a(a[9]), .p2m1_l(p2_l[8]),
     .p1m1_l(p1_l[8]), .p0m1_l(p0_l[8]), .am4(a[5]), .sum(sum[9]),
     .cout(cout[9]), .b1(b1[2:0]), .b0(b0[2:0]), .p1_l(p1_l[9]),
     .p0_l(p0_l[9]));
mul_ppgen3 I1_8_ ( .p2_l(p2_l[8]), .b2(b2[2:0]), 
     .am2(a[6]), .a(a[8]), .p2m1_l(p2_l[7]),
     .p1m1_l(p1_l[7]), .p0m1_l(p0_l[7]), .am4(a[4]), .sum(sum[8]),
     .cout(cout[8]), .b1(b1[2:0]), .b0(b0[2:0]), .p1_l(p1_l[8]),
     .p0_l(p0_l[8]));
mul_ppgen3 I1_7_ ( .p2_l(p2_l[7]), .b2(b2[2:0]), 
     .am2(a[5]), .a(a[7]), .p2m1_l(p2_l[6]),
     .p1m1_l(p1_l[6]), .p0m1_l(p0_l[6]), .am4(a[3]), .sum(sum[7]),
     .cout(cout[7]), .b1(b1[2:0]), .b0(b0[2:0]), .p1_l(p1_l[7]),
     .p0_l(p0_l[7]));
mul_ppgen3 I1_6_ ( .p2_l(p2_l[6]), .b2(b2[2:0]), 
     .am2(a[4]), .a(a[6]), .p2m1_l(p2_l[5]),
     .p1m1_l(p1_l[5]), .p0m1_l(p0_l[5]), .am4(a[2]), .sum(sum[6]),
     .cout(cout[6]), .b1(b1[2:0]), .b0(b0[2:0]), .p1_l(p1_l[6]),
     .p0_l(p0_l[6]));
mul_ppgen3 I1_5_ ( .p2_l(p2_l[5]), .b2(b2[2:0]), 
     .am2(a[3]), .a(a[5]), .p2m1_l(p2_l[4]),
     .p1m1_l(p1_l[4]), .p0m1_l(p0_l[4]), .am4(a[1]), .sum(sum[5]),
     .cout(cout[5]), .b1(b1[2:0]), .b0(b0[2:0]), .p1_l(p1_l[5]),
     .p0_l(p0_l[5]));
mul_ppgen3 I1_4_ ( .p2_l(p2_l[4]), .b2(b2[2:0]), 
     .am2(a[2]), .a(a[4]), .p2m1_l(1'b1),
     .p1m1_l(p1_l[3]), .p0m1_l(p0_l[3]), .am4(a[0]), .sum(sum[4]),
     .cout(cout[4]), .b1(b1[2:0]), .b0(b0[2:0]), .p1_l(p1_l[4]),
     .p0_l(p0_l[4]));
mul_ppgen3lsb4 I0 ( .cout(cout[3:1]), .a(a[3:0]), .sum(sum[3:0]),
     .p1_l(p1_l[3]), .p0_l(p0_l[3]), .b1(b1[2:0]), .b0(b0[2:0]));

endmodule //mul_ppgenrow3

module mul_ppgensign ( p_l, z, b, pm1_l );
output  p_l, z;
input  pm1_l;
input [2:0]  b;

assign p_l = ~(b[1] & b[2]);
assign z = b[0] ? ~pm1_l : ~p_l ;

endmodule //mul_ppgensign

module mul_ppgen ( p_l, z, a, b, pm1_l );
output  p_l, z;
input  a, pm1_l;
input [2:0]  b;

assign p_l = ~((a ^ b[2]) & b[1]) ;
assign z = b[0] ? ~pm1_l : ~p_l ;

endmodule //mul_ppgen

module mul_mux2 ( z, d0, d1, s );
output  z;
input  d0, d1, s;

assign z = s ? d1 : d0 ;

endmodule // mul_mux2 

module mul_booth(
	head,
        b_in,
        b0, b1, b2, b3, b4, b5, b6, b7,
	b8, b9, b10, b11, b12, b13, b14, b15, b16,
	clk, se, si, so, mul_step, tm_l
	);
input		head;		// begin of the MUL operation
input   [63:0] 	b_in;
input		clk, se, si, mul_step, tm_l;
output  [2:0]  	b0, b1, b2, b3, b4, b5, b6, b7, b8, b9, b10, b11, b12, b13, b14, b15;
output 		b16;
output 		so;

wire  [63:31] 	b;
wire [2:0] 	b0_in0, b1_in0,  b2_in0,  b3_in0,  b4_in0,  b5_in0,  b6_in0,  b7_in0 ;
wire [2:0] 	b8_in0, b9_in0, b10_in0, b11_in0, b12_in0, b13_in0, b14_in0, b15_in0 ;
wire [2:0] 	b0_in1, b1_in1,  b2_in1,  b3_in1,  b4_in1,  b5_in1,  b6_in1,  b7_in1 ;
wire [2:0] 	b8_in1, b9_in1, b10_in1, b11_in1, b12_in1, b13_in1, b14_in1, b15_in1 ;
wire 	   	b16_in1;

wire [2:0] 	b0_outmx, b1_outmx, b2_outmx, b3_outmx, b4_outmx, b5_outmx, b6_outmx;
wire [2:0] 	b7_outmx, b8_outmx, b9_outmx, b10_outmx, b11_outmx, b12_outmx, b13_outmx;
wire [2:0] 	b14_outmx, b15_outmx;
wire 	   	b16_outmx;
wire		clk_enb0, clk_enb1;


  mul_bodec 		encode0_a(
				.x  (1'b0),
				.b  (b_in[15:0]),
				.b0 (b0_in0),
				.b1 (b1_in0),
				.b2 (b2_in0),
				.b3 (b3_in0),
				.b4 (b4_in0),
				.b5 (b5_in0),
				.b6 (b6_in0),
				.b7 (b7_in0)
				);
				//remove 16th row since it's never the last row
				//b8_in0 = 3'b010; 
  mul_bodec		encode0_b(
				.x  (b_in[15]),
				.b  (b_in[31:16]),
				.b0 (b8_in0),
				.b1 (b9_in0),
				.b2 (b10_in0),
				.b3 (b11_in0),
				.b4 (b12_in0),
				.b5 (b13_in0),
				.b6 (b14_in0),
				.b7 (b15_in0)
				);
				// remove 32th row since it's never the last row 
				// b16_in0 = 3'b010 ;

  // Pipe picked address [63:31] and hold flop

  clken_buf     ckbuf_0(.clk(clk_enb0), .rclk(clk), .enb_l(~mul_step), .tmb_l(tm_l));
  clken_buf     ckbuf_1(.clk(clk_enb1), .rclk(clk), .enb_l(~(head & mul_step)), .tmb_l(tm_l));

  dff_s 			hld_dff0(.din(b_in[31]), .clk(clk_enb1), .q(b[31]),
                        	.se(se), .si(), .so());
  dff_s #(32) 		hld_dff(.din(b_in[63:32]), .clk(clk_enb1), .q(b[63:32]),
				.se(se), .si(), .so());

  mul_bodec     	encode1_a(
                        	.x  (b[31]),
                        	.b  (b[47:32]),
                        	.b0 (b0_in1),
                        	.b1 (b1_in1),
                        	.b2 (b2_in1),
                        	.b3 (b3_in1),
                        	.b4 (b4_in1),
                        	.b5 (b5_in1),
                        	.b6 (b6_in1),
                        	.b7 (b7_in1)
                        	);
                        	//remove 16th row since it's never the last row
                        	//b8_in1 = 3'b010;
  mul_bodec     	encode1_b(
                        	.x  (b[47]),
                        	.b  (b[63:48]),
                        	.b0 (b8_in1),
                        	.b1 (b9_in1),
                        	.b2 (b10_in1),
                        	.b3 (b11_in1),
                        	.b4 (b12_in1),
                        	.b5 (b13_in1),
                        	.b6 (b14_in1),
                        	.b7 (b15_in1)
                        	);
				assign b16_in1 = b[63] ;

// Select booth encoded b outputs and flop based on the cycle0 and cycle1 

  dp_mux2es #(3)    out_mux0(.dout(b0_outmx[2:0]),
                        .in0(b0_in0[2:0]),
                        .in1(b0_in1[2:0]),
                        .sel(~head));
  dp_mux2es #(3)    out_mux1(.dout(b1_outmx[2:0]),
                        .in0(b1_in0[2:0]),
                        .in1(b1_in1[2:0]),
                        .sel(~head));
  dp_mux2es #(3)    out_mux2(.dout(b2_outmx[2:0]),
                        .in0(b2_in0[2:0]),
                        .in1(b2_in1[2:0]),
                        .sel(~head));
  dp_mux2es #(3)    out_mux3(.dout(b3_outmx[2:0]),
                        .in0(b3_in0[2:0]),
                        .in1(b3_in1[2:0]),
                        .sel(~head));
  dp_mux2es #(3)    out_mux4(.dout(b4_outmx[2:0]),
                        .in0(b4_in0[2:0]),
                        .in1(b4_in1[2:0]),
                        .sel(~head));
  dp_mux2es #(3)    out_mux5(.dout(b5_outmx[2:0]),
                        .in0(b5_in0[2:0]),
                        .in1(b5_in1[2:0]),
                        .sel(~head));
  dp_mux2es #(3)    out_mux6(.dout(b6_outmx[2:0]),
                        .in0(b6_in0[2:0]),
                        .in1(b6_in1[2:0]),
                        .sel(~head));
  dp_mux2es #(3)    out_mux7(.dout(b7_outmx[2:0]),
                        .in0(b7_in0[2:0]),
                        .in1(b7_in1[2:0]),
                        .sel(~head));
  dp_mux2es #(3)    out_mux8(.dout(b8_outmx[2:0]),
                        .in0(b8_in0[2:0]),
                        .in1(b8_in1[2:0]),
                        .sel(~head));
  dp_mux2es #(3)    out_mux9(.dout(b9_outmx[2:0]),
                        .in0(b9_in0[2:0]),
                        .in1(b9_in1[2:0]),
                        .sel(~head));
  dp_mux2es #(3)    out_mux10(.dout(b10_outmx[2:0]),
                        .in0(b10_in0[2:0]),
                        .in1(b10_in1[2:0]),
                        .sel(~head));
  dp_mux2es #(3)    out_mux11(.dout(b11_outmx[2:0]),
                        .in0(b11_in0[2:0]),
                        .in1(b11_in1[2:0]),
                        .sel(~head));
  dp_mux2es #(3)    out_mux12(.dout(b12_outmx[2:0]),
                        .in0(b12_in0[2:0]),
                        .in1(b12_in1[2:0]),
                        .sel(~head));
  dp_mux2es #(3)    out_mux13(.dout(b13_outmx[2:0]),
                        .in0(b13_in0[2:0]),
                        .in1(b13_in1[2:0]),
                        .sel(~head));
  dp_mux2es #(3)    out_mux14(.dout(b14_outmx[2:0]),
                        .in0(b14_in0[2:0]),
                        .in1(b14_in1[2:0]),
                        .sel(~head));
  dp_mux2es #(3)    out_mux15(.dout(b15_outmx[2:0]),
                        .in0(b15_in0[2:0]),
                        .in1(b15_in1[2:0]),
                        .sel(~head));
  dp_mux2es         out_mux16(.dout(b16_outmx),
                        .in0(1'b0),
                        .in1(b16_in1),
                        .sel(~head));

  dff_s #(3)    out_dff0 (.din(b0_outmx[2:0]), .clk(clk_enb0), .q(b0[2:0]),
			.se(se), .si(), .so());
  dff_s #(3)    out_dff1 (.din(b1_outmx[2:0]), .clk(clk_enb0), .q(b1[2:0]),
			.se(se), .si(), .so());
  dff_s #(3)    out_dff2 (.din(b2_outmx[2:0]), .clk(clk_enb0), .q(b2[2:0]),
			.se(se), .si(), .so());
  dff_s #(3)    out_dff3 (.din(b3_outmx[2:0]), .clk(clk_enb0), .q(b3[2:0]),
			.se(se), .si(), .so());
  dff_s #(3)    out_dff4 (.din(b4_outmx[2:0]), .clk(clk_enb0), .q(b4[2:0]),
			.se(se), .si(), .so());
  dff_s #(3)    out_dff5 (.din(b5_outmx[2:0]), .clk(clk_enb0), .q(b5[2:0]),
			.se(se), .si(), .so());
  dff_s #(3)    out_dff6 (.din(b6_outmx[2:0]), .clk(clk_enb0), .q(b6[2:0]),
			.se(se), .si(), .so());
  dff_s #(3)    out_dff7 (.din(b7_outmx[2:0]), .clk(clk_enb0), .q(b7[2:0]),
			.se(se), .si(), .so());
  dff_s #(3)    out_dff8 (.din(b8_outmx[2:0]), .clk(clk_enb0), .q(b8[2:0]),
			.se(se), .si(), .so());
  dff_s #(3)    out_dff9 (.din(b9_outmx[2:0]), .clk(clk_enb0), .q(b9[2:0]),
			.se(se), .si(), .so());
  dff_s #(3)    out_dff10 (.din(b10_outmx[2:0]), .clk(clk_enb0), .q(b10[2:0]),
			.se(se), .si(), .so());
  dff_s #(3)    out_dff11 (.din(b11_outmx[2:0]), .clk(clk_enb0), .q(b11[2:0]),
			.se(se), .si(), .so());
  dff_s #(3)    out_dff12 (.din(b12_outmx[2:0]), .clk(clk_enb0), .q(b12[2:0]),
			.se(se), .si(), .so());
  dff_s #(3)    out_dff13 (.din(b13_outmx[2:0]), .clk(clk_enb0), .q(b13[2:0]),
			.se(se), .si(), .so());
  dff_s #(3)    out_dff14 (.din(b14_outmx[2:0]), .clk(clk_enb0), .q(b14[2:0]),
			.se(se), .si(), .so());
  dff_s #(3)    out_dff15 (.din(b15_outmx[2:0]), .clk(clk_enb0), .q(b15[2:0]),
			.se(se), .si(), .so());
  dff_s 	      out_dff16 (.din(b16_outmx), .clk(clk_enb0), .q(b16),
			.se(se), .si(), .so());
endmodule //mul_booth

module mul_bodec (x, b,  
        b0, b1, b2, b3, b4, b5, b6, b7);

input	x;
input   [15:0] 	b;
output  [2:0] 	b0, b1, b2, b3, b4, b5, b6, b7; 

assign b0[2] = b[1];
assign b0[1] = ~((b[1] & b[0] & x) | (~b[1] & ~b[0] & ~x)) ;
assign b0[0] = (~b[1] & b[0] & x) | (b[1] & ~b[0] & ~x) ;

assign b1[2] = b[3]; 
assign b1[1] = ~((b[3] & b[2] & b[1]) | (~b[3] & ~b[2] & ~b[1])) ;
assign b1[0] = (~b[3] & b[2] & b[1]) | (b[3] & ~b[2] & ~b[1]) ;

assign b2[2] = b[5]; 
assign b2[1] = ~((b[5] & b[4] & b[3]) | (~b[5] & ~b[4] & ~b[3])) ;
assign b2[0] = (~b[5] & b[4] & b[3]) | (b[5] & ~b[4] & ~b[3]) ;

assign b3[2] = b[7] ;
assign b3[1] = ~((b[7] & b[6] & b[5]) | (~b[7] & ~b[6] & ~b[5])) ;
assign b3[0] = (~b[7] & b[6] & b[5]) | (b[7] & ~b[6] & ~b[5]) ;

assign b4[2] = b[9] ;
assign b4[1] = ~((b[9] & b[8] & b[7]) | (~b[9] & ~b[8] & ~b[7])) ;
assign b4[0] = (~b[9] & b[8] & b[7]) | (b[9] & ~b[8] & ~b[7]) ;

assign b5[2] = b[11] ;
assign b5[1] = ~((b[11] & b[10] & b[9]) | (~b[11] & ~b[10] & ~b[9])) ;
assign b5[0] = (~b[11] & b[10] & b[9]) | (b[11] & ~b[10] & ~b[9]) ;

assign b6[2] = b[13] ;
assign b6[1] = ~((b[13] & b[12] & b[11]) | (~b[13] & ~b[12] & ~b[11])) ;
assign b6[0] = (~b[13] & b[12] & b[11]) | (b[13] & ~b[12] & ~b[11]) ;

assign b7[2] = b[15] ;
assign b7[1] = ~((b[15] & b[14] & b[13]) | (~b[15] & ~b[14] & ~b[13])) ;
assign b7[0] = (~b[15] & b[14] & b[13]) | (b[15] & ~b[14] & ~b[13]) ;

endmodule // mul_bodec

// Modified by Princeton University on June 9th, 2015
// ========== Copyright Header Begin ==========================================
// 
// OpenSPARC T1 Processor File: swrvr_clib.v
// Copyright (c) 2006 Sun Microsystems, Inc.  All Rights Reserved.
// DO NOT ALTER OR REMOVE COPYRIGHT NOTICES.
// 
// The above named program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public
// License version 2 as published by the Free Software Foundation.
// 
// The above named program is distributed in the hope that it will be 
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// General Public License for more details.
// 
// You should have received a copy of the GNU General Public
// License along with this work; if not, write to the Free Software
// Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA 02110-1301, USA.
// 
// ========== Copyright Header End ============================================
///////////////////////////////////////////////////////////////////////
/*
//
//  Module Name: swrvr_clib.v
//      Description: Design control behavioural library
*/                 





// POSITVE-EDGE TRIGGERED FLOP with SCAN
module dff_s (din, clk, q, se, si, so);
// synopsys template

parameter SIZE = 1;

input	[SIZE-1:0]	din ;	// data in
input			clk ;	// clk or scan clk

output	[SIZE-1:0]	q ;	// output

input			se ;	// scan-enable
input	[SIZE-1:0]	si ;	// scan-input
output	[SIZE-1:0]	so ;	// scan-output

reg 	[SIZE-1:0]	q ;


always @ (posedge clk)
  q[SIZE-1:0]  <= din[SIZE-1:0] ;










endmodule // dff_s

// POSITVE-EDGE TRIGGERED FLOP with SCAN for Shadow-scan
module dff_sscan (din, clk, q, se, si, so);
// synopsys template

parameter SIZE = 1;

input	[SIZE-1:0]	din ;	// data in
input			clk ;	// clk or scan clk

output	[SIZE-1:0]	q ;	// output

input			se ;	// scan-enable
input	[SIZE-1:0]	si ;	// scan-input
output	[SIZE-1:0]	so ;	// scan-output

reg 	[SIZE-1:0]	q ;










always @ (posedge clk)
  q[SIZE-1:0]  <= din[SIZE-1:0] ;

assign so={SIZE{1'b0}};


endmodule // dff_sscan

// POSITVE-EDGE TRIGGERED FLOP without SCAN
module dff_ns (din, clk, q);
// synopsys template

parameter SIZE = 1;

input	[SIZE-1:0]	din ;	// data in
input			clk ;	// clk

output	[SIZE-1:0]	q ;	// output

reg 	[SIZE-1:0]	q ;

always @ (posedge clk)

	q[SIZE-1:0]  <= din[SIZE-1:0] ;

endmodule // dff_ns

// POSITIVE-EDGE TRIGGERED FLOP with SCAN, RESET
module dffr_s (din, clk, rst, q, se, si, so);
// synopsys template

parameter SIZE = 1;

input	[SIZE-1:0]	din ;	// data in
input			clk ;	// clk or scan clk
input			rst ;	// reset

output	[SIZE-1:0]	q ;	// output

input			se ;	// scan-enable
input	[SIZE-1:0]	si ;	// scan-input
output	[SIZE-1:0]	so ;	// scan-output

reg 	[SIZE-1:0]	q ;


always @ (posedge clk)
	q[SIZE-1:0]  <= ((rst) ? {SIZE{1'b0}}  : din[SIZE-1:0] );










endmodule // dffr_s

// POSITIVE-EDGE TRIGGERED FLOP with SCAN, RESET_L
module dffrl_s (din, clk, rst_l, q, se, si, so);
// synopsys template

parameter SIZE = 1;

input	[SIZE-1:0]	din ;	// data in
input			clk ;	// clk or scan clk
input			rst_l ;	// reset

output	[SIZE-1:0]	q ;	// output

input			se ;	// scan-enable
input	[SIZE-1:0]	si ;	// scan-input
output	[SIZE-1:0]	so ;	// scan-output

reg 	[SIZE-1:0]	q ;


always @ (posedge clk)
	q[SIZE-1:0]  <= rst_l ? din[SIZE-1:0] : {SIZE{1'b0}};










endmodule // dffrl_s

// POSITIVE-EDGE TRIGGERED FLOP with RESET, without SCAN
module dffr_ns (din, clk, rst, q);
// synopsys template

parameter SIZE = 1;

input	[SIZE-1:0]	din ;	// data in
input			clk ;	// clk
input			rst ;	// reset

output	[SIZE-1:0]	q ;	// output

reg 	[SIZE-1:0]	q ;

// synopsys sync_set_reset "rst"
always @ (posedge clk)
  q[SIZE-1:0] <= rst ? {SIZE{1'b0}} : din[SIZE-1:0];
   
endmodule // dffr_ns

// POSITIVE-EDGE TRIGGERED FLOP with RESET_L, without SCAN
module dffrl_ns (din, clk, rst_l, q);
// synopsys template

parameter SIZE = 1;

input	[SIZE-1:0]	din ;	// data in
input			clk ;	// clk
input			rst_l ;	// reset

output	[SIZE-1:0]	q ;	// output

reg 	[SIZE-1:0]	q ;

// synopsys sync_set_reset "rst_l"
always @ (posedge clk)
  q[SIZE-1:0] <= rst_l ? din[SIZE-1:0] : {SIZE{1'b0}};

endmodule // dffrl_ns

// POSITIVE-EDGE TRIGGERED FLOP with SCAN and FUNCTIONAL ENABLE
module dffe_s (din, en, clk, q, se, si, so);
// synopsys template

parameter SIZE = 1;

input	[SIZE-1:0]	din ;	// data in
input			en ;	// functional enable
input			clk ;	// clk or scan clk

output	[SIZE-1:0]	q ;	// output

input			se ;	// scan-enable
input	[SIZE-1:0]	si ;	// scan-input
output	[SIZE-1:0]	so ;	// scan-output

reg 	[SIZE-1:0]	q ;

// Enable Interpretation. Ultimate interpretation depends on design
// 
// en	se	out
//------------------
// x	1	sin ; scan dominates
// 1  	0	din
// 0 	0	q
//


always @ (posedge clk)
	q[SIZE-1:0]  <= ((en) ? din[SIZE-1:0] : q[SIZE-1:0]) ;









endmodule // dffe_s

// POSITIVE-EDGE TRIGGERED FLOP with enable, without SCAN
module dffe_ns (din, en, clk, q);
// synopsys template

parameter SIZE = 1;

input	[SIZE-1:0]	din ;	// data in
input			en ;	// functional enable
input			clk ;	// clk

output	[SIZE-1:0]	q ;	// output

reg 	[SIZE-1:0]	q ;

always @ (posedge clk)
  q[SIZE-1:0] <= en ? din[SIZE-1:0] : q[SIZE-1:0];

endmodule // dffe_ns

// POSITIVE-EDGE TRIGGERED FLOP with RESET, FUNCTIONAL ENABLE, SCAN.
module dffre_s (din, rst, en, clk, q, se, si, so);
// synopsys template

parameter SIZE = 1;

input	[SIZE-1:0]	din ;	// data in
input			en ;	// functional enable
input			rst ;	// reset
input			clk ;	// clk or scan clk

output	[SIZE-1:0]	q ;	// output

input			se ;	// scan-enable
input	[SIZE-1:0]	si ;	// scan-input
output	[SIZE-1:0]	so ;	// scan-output

reg 	[SIZE-1:0]	q ;

// Enable Interpretation. Ultimate interpretation depends on design
// 
// rst	en	se	out
//------------------
// 1	x	x	0   ; reset dominates
// 0	x	1	sin ; scan dominates
// 0	1  	0	din
// 0 	0 	0	q
//


always @ (posedge clk)
	q[SIZE-1:0]  <= (rst ? {SIZE{1'b0}} : ((en) ? din[SIZE-1:0] : q[SIZE-1:0])) ;











endmodule // dffre_s

// POSITIVE-EDGE TRIGGERED FLOP with RESET_L, FUNCTIONAL ENABLE, SCAN.
module dffrle_s (din, rst_l, en, clk, q, se, si, so);
// synopsys template

parameter SIZE = 1;

input	[SIZE-1:0]	din ;	// data in
input			en ;	// functional enable
input			rst_l ;	// reset
input			clk ;	// clk or scan clk

output	[SIZE-1:0]	q ;	// output

input			se ;	// scan-enable
input	[SIZE-1:0]	si ;	// scan-input
output	[SIZE-1:0]	so ;	// scan-output

reg 	[SIZE-1:0]	q ;

// Enable Interpretation. Ultimate interpretation depends on design
// 
// rst	en	se	out
//------------------
// 0	x	x	0   ; reset dominates
// 1	x	1	sin ; scan dominates
// 1	1  	0	din
// 1 	0 	0	q
//


always @ (posedge clk)
	 q[SIZE-1:0]  <= (rst_l ? ((en) ? din[SIZE-1:0] : q[SIZE-1:0]) : {SIZE{1'b0}}) ;










endmodule // dffrle_s

// POSITIVE-EDGE TRIGGERED FLOP with RESET, ENABLE, without SCAN.
module dffre_ns (din, rst, en, clk, q);
// synopsys template

parameter SIZE = 1;

input	[SIZE-1:0]	din ;	// data in
input			en ;	// functional enable
input			rst ;	// reset
input			clk ;	// clk

output	[SIZE-1:0]	q ;	// output

reg 	[SIZE-1:0]	q ;

// Enable Interpretation. Ultimate interpretation depends on design
// 
// rst	en	out
//------------------
// 1	x	0   ; reset dominates
// 0	1  	din
// 0 	0 	q
//

// synopsys sync_set_reset "rst"
always @ (posedge clk)
  q[SIZE-1:0] <= rst ? {SIZE{1'b0}} : ((en) ? din[SIZE-1:0] : q[SIZE-1:0]);

endmodule // dffre_ns

// POSITIVE-EDGE TRIGGERED FLOP with RESET_L, ENABLE, without SCAN.
module dffrle_ns (din, rst_l, en, clk, q);
// synopsys template

parameter SIZE = 1;

input	[SIZE-1:0]	din ;	// data in
input			en ;	// functional enable
input			rst_l ;	// reset
input			clk ;	// clk

output	[SIZE-1:0]	q ;	// output

reg 	[SIZE-1:0]	q ;

// Enable Interpretation. Ultimate interpretation depends on design
// 
// rst	en	out
//------------------
// 0	x	0   ; reset dominates
// 1	1  	din
// 1 	0 	q
//

// synopsys sync_set_reset "rst_l"
always @ (posedge clk)
  q[SIZE-1:0] <= rst_l ? ((en) ? din[SIZE-1:0] : q[SIZE-1:0]) : {SIZE{1'b0}} ;

endmodule // dffrle_ns

// POSITIVE-EDGE TRIGGERED FLOP with SCAN, and ASYNC RESET
module dffr_async (din, clk, rst, q, se, si, so);
// synopsys template

parameter SIZE = 1;

input   [SIZE-1:0]      din ;   // data in
input                   clk ;   // clk or scan clk
input                   rst ;   // reset

output  [SIZE-1:0]      q ;     // output

input                   se ;    // scan-enable
input   [SIZE-1:0]      si ;    // scan-input
output  [SIZE-1:0]      so ;    // scan-output

reg     [SIZE-1:0]      q ;


always @ (posedge clk or posedge rst)
	q[SIZE-1:0]  <= rst ? {SIZE{1'b0}} : din[SIZE-1:0];










endmodule // dffr_async

// POSITIVE-EDGE TRIGGERED FLOP with SCAN, and ASYNC RESET_L
module dffrl_async (din, clk, rst_l, q, se, si, so);
// synopsys template

parameter SIZE = 1;

input   [SIZE-1:0]      din ;   // data in
input                   clk ;   // clk or scan clk
input                   rst_l ;   // reset

output  [SIZE-1:0]      q ;     // output

input                   se ;    // scan-enable
input   [SIZE-1:0]      si ;    // scan-input
output  [SIZE-1:0]      so ;    // scan-output

reg     [SIZE-1:0]      q ;


always @ (posedge clk or negedge rst_l)
	q[SIZE-1:0]  <= (!rst_l) ? {SIZE{1'b0}} : din[SIZE-1:0];










endmodule // dffrl_async

// POSITIVE-EDGE TRIGGERED FLOP with ASYNC RESET, without SCAN
//module dffr_async_ns (din, clk, rst, q);
//// synopsys template
//parameter SIZE = 1;
//input   [SIZE-1:0]      din ;   // data in
//input                   clk ;   // clk or scan clk
//input                   rst ;   // reset
//output  [SIZE-1:0]      q ;     // output
//reg     [SIZE-1:0]      q ;
// Reset dominates
//// synopsys async_set_reset "rst"
//always @ (posedge clk or posedge rst)
//        if(rst) q[SIZE-1:0]  <= {SIZE{1'b0}};
//        else if(clk) q[SIZE-1:0]  <= din[SIZE-1:0];
//endmodule // dffr_async_ns

// POSITIVE-EDGE TRIGGERED FLOP with ASYNC RESET_L, without SCAN
module dffrl_async_ns (din, clk, rst_l, q);
// synopsys template

parameter SIZE = 1;

input   [SIZE-1:0]      din ;   // data in
input                   clk ;   // clk or scan clk
input                   rst_l ;   // reset

output  [SIZE-1:0]      q ;     // output

// Reset dominates
// synopsys async_set_reset "rst_l"
 reg [SIZE-1:0] q;   
always @ (posedge clk or negedge rst_l)
  q[SIZE-1:0] <= ~rst_l ?  {SIZE{1'b0}} : ({SIZE{rst_l}} & din[SIZE-1:0]);

//   reg  [SIZE-1:0]   qm, qs, qm_l, qs_l, qm_f, qs_f;
//   wire              s_l;
//   assign            s_l = 1'b1;
//
//   always @ (rst_l or qm)   qm_l = ~(qm & {SIZE{rst_l}});
//   always @ (s_l or qs)   qs_l = ~(qs & {SIZE{s_l}});
//   always @ (s_l or qm_l) qm_f = ~(qm_l & {SIZE{s_l}});
//   always @ (rst_l or qs_l) qs_f = ~(qs_l & {SIZE{rst_l}});
//
//   always @ (clk or din or qm_f)
//      qm <= clk ? qm_f : din;
//
//   always @ (clk or qm_l or qs_f)
//      qs <= clk ? qm_l : qs_f;
//
//   assign q  = ~qs;

endmodule // dffrl_async_ns

// 2:1 MUX WITH DECODED SELECTS
module mux2ds (dout, in0, in1, sel0, sel1) ;
// synopsys template

parameter SIZE = 1;

output 	[SIZE-1:0] 	dout;
input	[SIZE-1:0]	in0;
input	[SIZE-1:0]	in1;
input			sel0;
input			sel1;

// reg declaration does not imply state being maintained
// across cycles. Used to construct case statement and
// always updated by inputs every cycle.
reg	[SIZE-1:0]	dout ;

// priority encoding takes care of mutex'ing selects.




wire [1:0] sel = {sel1, sel0}; // 0in one_hot
   
always @ (sel0 or sel1 or in0 or in1)

	case ({sel1,sel0}) // synopsys infer_mux
		2'b01 :	dout = in0 ;
		2'b10 : dout = in1 ;
		2'b11 : dout = {SIZE{1'bx}} ;
		2'b00 : dout = {SIZE{1'bx}} ;
			// 2'b00 : // E.g. 4state vs. 2state modelling.
			// begin
			//	`ifdef FOUR_STATE
			//		dout = {SIZE{1'bx}};
			//	`else
			//		begin
			//		dout = {SIZE{1'b0}};
			//		$error();
			//		end
			//	`endif
			// end
		default : dout = {SIZE{1'bx}};
	endcase

endmodule // mux2ds

// 3:1 MUX WITH DECODED SELECTS
module mux3ds (dout, in0, in1, in2, sel0, sel1, sel2) ;
// synopsys template

parameter SIZE = 1;

output 	[SIZE-1:0] 	dout;
input	[SIZE-1:0]	in0;
input	[SIZE-1:0]	in1;
input	[SIZE-1:0]	in2;
input			sel0;
input			sel1;
input			sel2;

// reg declaration does not imply state being maintained
// across cycles. Used to construct case statement and
// always updated by inputs every cycle.
reg	[SIZE-1:0]	dout ;





wire [2:0] sel = {sel2,sel1,sel0}; // 0in one_hot
   
// priority encoding takes care of mutex'ing selects.
always @ (sel0 or sel1 or sel2 or in0 or in1 or in2)

	case ({sel2,sel1,sel0}) 
		3'b001 : dout = in0 ;
		3'b010 : dout = in1 ;
		3'b100 : dout = in2 ;
		3'b000 : dout = {SIZE{1'bx}} ;
		3'b011 : dout = {SIZE{1'bx}} ;
		3'b101 : dout = {SIZE{1'bx}} ;
		3'b110 : dout = {SIZE{1'bx}} ;
		3'b111 : dout = {SIZE{1'bx}} ;
		default : dout = {SIZE{1'bx}};
			// two state vs four state modelling will be added.
	endcase

endmodule // mux3ds

// 4:1 MUX WITH DECODED SELECTS
module mux4ds (dout, in0, in1, in2, in3, sel0, sel1, sel2, sel3) ;
// synopsys template

parameter SIZE = 1;

output 	[SIZE-1:0] 	dout;
input	[SIZE-1:0]	in0;
input	[SIZE-1:0]	in1;
input	[SIZE-1:0]	in2;
input	[SIZE-1:0]	in3;
input			sel0;
input			sel1;
input			sel2;
input			sel3;

// reg declaration does not imply state being maintained
// across cycles. Used to construct case statement and
// always updated by inputs every cycle.
reg	[SIZE-1:0]	dout ;




   
wire [3:0] sel = {sel3,sel2,sel1,sel0}; // 0in one_hot
   
// priority encoding takes care of mutex'ing selects.
always @ (sel0 or sel1 or sel2 or sel3 or in0 or in1 or in2 or in3)

	case ({sel3,sel2,sel1,sel0}) 
		4'b0001 : dout = in0 ;
		4'b0010 : dout = in1 ;
		4'b0100 : dout = in2 ;
		4'b1000 : dout = in3 ;
		4'b0000 : dout = {SIZE{1'bx}} ;
		4'b0011 : dout = {SIZE{1'bx}} ;
		4'b0101 : dout = {SIZE{1'bx}} ;
		4'b0110 : dout = {SIZE{1'bx}} ;
		4'b0111 : dout = {SIZE{1'bx}} ;
		4'b1001 : dout = {SIZE{1'bx}} ;
		4'b1010 : dout = {SIZE{1'bx}} ;
		4'b1011 : dout = {SIZE{1'bx}} ;
		4'b1100 : dout = {SIZE{1'bx}} ;
		4'b1101 : dout = {SIZE{1'bx}} ;
		4'b1110 : dout = {SIZE{1'bx}} ;
		4'b1111 : dout = {SIZE{1'bx}} ;
		default : dout = {SIZE{1'bx}};
			// two state vs four state modelling will be added.
	endcase

endmodule // mux4ds

// SINK FOR UNLOADED INPUT PORTS
module sink (in);
// synopsys template

parameter SIZE = 1;

input [SIZE-1:0] in;

// Alexey
// `ifdef PITON_PROTO
   // As of version 8.2 XST does not remove this module without the
   // following additional dead code

   wire    a;

   assign		a = | in;

// `endif

endmodule //sink

// SOURCE FOR UNDRIVEN OUTPUT PORTS
module source (out) ;
// synopsys template

parameter SIZE = 1;

output  [SIZE-1:0] out;
// 
// Once 4state/2state model established
// then enable check.
// `ifdef FOUR_STATE
// leda check for x_or_z_in rhs_of assign turned off
// assign  out = {SIZE{1'bx}};
//`else
assign  out = {SIZE{1'b0}};
//`endif

endmodule //source

// 2:1 MUX WITH PRIORITY ENCODED SELECTS
//module mux2es (dout, in0, in1, sel0, sel1) ;
//
//parameter SIZE = 1;
//
//output 	[SIZE-1:0] 	dout;
//input	[SIZE-1:0]	in0;
//input	[SIZE-1:0]	in1;
//input			sel0;
//input			sel1;
//
//// reg declaration does not imply state being maintained
//// across cycles. Used to construct case statement and
//// always updated by inputs every cycle.
//reg	[SIZE-1:0]	dout ;
//
//// must take into account lack of mutex selects.
//// there is no reason for handling of x and z conditions.
//// This will be dictated by design.
//always @ (sel0 or sel1 or in0 or in1)
//
//	case ({sel1,sel0})
//		2'b1x : dout = in1 ; // 10(in1),11(z) 
//		2'b0x :	dout = in0 ; // 01(in0),00(x)
//	endcase
//
//endmodule // mux2es

// CLK Header for gating off the clock of
// a FF.
// clk - output of the clk header
// rclk - input clk
// enb_l - Active low clock enable
// tmb_l  - Active low clock enable ( in scan mode, this input is !se )

module clken_buf (clk, rclk, enb_l, tmb_l);
output clk;
input  rclk, enb_l, tmb_l;
reg    clken;

  always @ (rclk or enb_l or tmb_l)
    if (!rclk)  //latch opens on rclk low phase
      clken = !enb_l | !tmb_l;
  assign clk = clken & rclk;

endmodule



// The following flops are maintained and used in ENET , MAC IP  ONLY
// -- Mimi X61467

// POSITIVE-EDGE TRIGGERED FLOP with SET_L, without SCAN.

module dffsl_ns (din, clk, set_l, q);
// synopsys template
parameter SIZE = 1;

input   [SIZE-1:0]      din ;   // data in
input                   clk ;   // clk or scan clk
input                   set_l ; // set

output  [SIZE-1:0]      q ;     // output

reg     [SIZE-1:0]      q ;

// synopsys sync_set_reset "set_l"
always @ (posedge clk)
  q[SIZE-1:0] <= set_l ? din[SIZE-1:0] : {SIZE{1'b1}};

endmodule // dffsl_ns

// POSITIVE-EDGE TRIGGERED FLOP with SET_L, without SCAN.

module dffsl_async_ns (din, clk, set_l, q);
// synopsys template
parameter SIZE = 1;

input   [SIZE-1:0]      din ;   // data in
input                   clk ;   // clk or scan clk
input                   set_l ; // set

output  [SIZE-1:0]      q ;     // output

reg     [SIZE-1:0]      q ;

// synopsys async_set_reset "set_l"
always @ (posedge clk or negedge set_l)
  q[SIZE-1:0] <= ~set_l ? {SIZE{1'b1}} : ({SIZE{~set_l}} | din[SIZE-1:0]);

endmodule // dffsl_async_ns

// POSITIVE-EDGE TRIGGERED FLOP WITH SET_H , without SCAN.

module dffr_ns_r1 (din, clk, rst, q);
// synopsys template
parameter SIZE = 1;

input   [SIZE-1:0]      din ;   // data in
input                   clk ;   // clk or scan clk
input                   rst ;   // reset

output  [SIZE-1:0]      q ;     // output

reg     [SIZE-1:0]      q ;

// Set to 1
// synopsys sync_set_reset "rst"
always @ (posedge clk)
  q[SIZE-1:0] <= rst ? {SIZE{1'b1}} : din[SIZE-1:0];

endmodule // dffr_ns_r1

// POSITIVE-EDGE TRIGGERED ASYNC RESET_H FLOP , without SCAN.

module dffr_async_ns (din, clk, rst, q);
// synopsys template

parameter SIZE = 1;

input   [SIZE-1:0]      din ;   // data in
input                   clk ;   // clk or scan clk
input                   rst;   // reset

output  [SIZE-1:0]      q ;     // output

reg     [SIZE-1:0]      q ;

// Reset dominates
// synopsys async_set_reset "rst"
always @ (posedge clk or posedge rst)
  q[SIZE-1:0] <= rst ? {SIZE{1'b0}} : din[SIZE-1:0];

endmodule // dffr_async_ns

// POSITIVE-EDGE TRIGGERED ASYNC SET_H FLOP , without SCAN.

module dffr_async_ns_r1 (din, clk, rst, q);
// synopsys template

parameter SIZE = 1;

input   [SIZE-1:0]      din ;   // data in
input                   clk ;   // clk or scan clk
input                   rst;   // reset

output  [SIZE-1:0]      q ;     // output

reg     [SIZE-1:0]      q ;

// Reset to 1
// synopsys async_set_reset "rst"
always @ (posedge clk or posedge rst)
  q[SIZE-1:0] <= rst ? {SIZE{1'b1}} : din[SIZE-1:0];

endmodule // dffr_async_ns_r1


// NEGATIVE-EDGE TRIGGERED ASYNC SET_H FLOP , without SCAN.

module dffr_async_ns_cl_r1 (din, clkl, rst, q);
// synopsys template
parameter SIZE = 1;

input   [SIZE-1:0]      din ;   // data in
input                   clkl ;  // clk or scan clk
input                   rst ;   // reset

output  [SIZE-1:0]      q ;     // output

reg     [SIZE-1:0]      q ;

// Set to 1
// synopsys sync_set_reset "rst"
always @ (negedge clkl or posedge rst)
  q[SIZE-1:0] <= rst ? {SIZE{1'b1}} : din[SIZE-1:0];

endmodule // dffr_async_ns_cl_r1

// Modified by Princeton University on June 9th, 2015
// ========== Copyright Header Begin ==========================================
// 
// OpenSPARC T1 Processor File: swrvr_dlib.v
// Copyright (c) 2006 Sun Microsystems, Inc.  All Rights Reserved.
// DO NOT ALTER OR REMOVE COPYRIGHT NOTICES.
// 
// The above named program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public
// License version 2 as published by the Free Software Foundation.
// 
// The above named program is distributed in the hope that it will be 
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// General Public License for more details.
// 
// You should have received a copy of the GNU General Public
// License along with this work; if not, write to the Free Software
// Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA 02110-1301, USA.
// 
// ========== Copyright Header End ============================================
// DP library

// 2:1 MUX WITH ENCODED SELECT
module dp_mux2es (dout, in0, in1, sel) ;
// synopsys template

parameter SIZE = 1;

output 	[SIZE-1:0] 	dout;
input	[SIZE-1:0]	in0;
input	[SIZE-1:0]	in1;
input			sel;

reg	[SIZE-1:0]	dout ;

always @ (sel or in0 or in1)

 begin
	   case (sel)
	     1'b1: dout = in1 ; 
	     1'b0: dout = in0;
	     default: 
         begin
            if (in0 == in1) begin
               dout = in0;
            end
            else
              dout = {SIZE{1'bx}};
         end
	   endcase // case(sel)
 end

endmodule // dp_mux2es

// ----------------------------------------------------------------------


// 4:1 MUX WITH DECODED SELECTS
module dp_mux4ds (dout, in0, in1, in2, in3, 
		     sel0_l, sel1_l, sel2_l, sel3_l) ;
// synopsys template

parameter SIZE = 1;

output 	[SIZE-1:0] 	dout;
input	[SIZE-1:0]	in0;
input	[SIZE-1:0]	in1;
input	[SIZE-1:0]	in2;
input	[SIZE-1:0]	in3;
input			sel0_l;
input			sel1_l;
input			sel2_l;
input			sel3_l;

// reg declaration does not imply state being maintained
// across cycles. Used to construct case statement and
// always updated by inputs every cycle.
reg	[SIZE-1:0]	dout ;





wire [3:0] sel = {sel3_l,sel2_l,sel1_l,sel0_l}; // 0in one_cold
   
always @ (sel0_l or sel1_l or sel2_l or sel3_l or in0 or in1 or in2 or in3)

	case ({sel3_l,sel2_l,sel1_l,sel0_l})
		4'b1110 : dout = in0 ;
		4'b1101 : dout = in1 ;
		4'b1011 : dout = in2 ;
		4'b0111 : dout = in3 ;
		4'b1111 : dout = {SIZE{1'bx}} ;
		default : dout = {SIZE{1'bx}} ;
	endcase

endmodule // dp_mux4ds

// ----------------------------------------------------------------------


// 5:1 MUX WITH DECODED SELECTS
module dp_mux5ds (dout, in0, in1, in2, in3,  in4,
		     sel0_l, sel1_l, sel2_l, sel3_l, sel4_l) ;
// synopsys template

parameter SIZE = 1;

output 	[SIZE-1:0] 	dout;
input	[SIZE-1:0]	in0;
input	[SIZE-1:0]	in1;
input	[SIZE-1:0]	in2;
input	[SIZE-1:0]	in3;
input	[SIZE-1:0]	in4;
input			sel0_l;
input			sel1_l;
input			sel2_l;
input			sel3_l;
input			sel4_l;

// reg declaration does not imply state being maintained
// across cycles. Used to construct case statement and
// always updated by inputs every cycle.
reg	[SIZE-1:0]	dout ;




   
wire [4:0] sel = {sel4_l,sel3_l,sel2_l,sel1_l,sel0_l}; // 0in one_cold

always @ (sel0_l or sel1_l or sel2_l or sel3_l or sel4_l or
		in0 or in1 or in2 or in3 or in4)

	case ({sel4_l,sel3_l,sel2_l,sel1_l,sel0_l})
		5'b11110 : dout = in0 ;
		5'b11101 : dout = in1 ;
		5'b11011 : dout = in2 ;
		5'b10111 : dout = in3 ;
		5'b01111 : dout = in4 ;
		5'b11111 : dout = {SIZE{1'bx}} ;
		default : dout = {SIZE{1'bx}} ;
	endcase

endmodule // dp_mux5ds

// --------------------------------------------------------------------

// 8:1 MUX WITH DECODED SELECTS
module dp_mux8ds (dout, in0, in1, in2, in3, 
			in4, in5, in6, in7,
		     sel0_l, sel1_l, sel2_l, sel3_l,
		     sel4_l, sel5_l, sel6_l, sel7_l) ;
// synopsys template

parameter SIZE = 1;

output 	[SIZE-1:0] 	dout;
input	[SIZE-1:0]	in0;
input	[SIZE-1:0]	in1;
input	[SIZE-1:0]	in2;
input	[SIZE-1:0]	in3;
input	[SIZE-1:0]	in4;
input	[SIZE-1:0]	in5;
input	[SIZE-1:0]	in6;
input	[SIZE-1:0]	in7;
input			sel0_l;
input			sel1_l;
input			sel2_l;
input			sel3_l;
input			sel4_l;
input			sel5_l;
input			sel6_l;
input			sel7_l;

// reg declaration does not imply state being maintained
// across cycles. Used to construct case statement and
// always updated by inputs every cycle.
reg	[SIZE-1:0]	dout ;






wire [7:0] sel = {sel7_l,sel6_l,sel5_l,sel4_l,
                  sel3_l,sel2_l,sel1_l,sel0_l}; // 0in one_cold

always @ (sel0_l or sel1_l or sel2_l or sel3_l or in0 or in1 or in2 or in3 or
	  sel4_l or sel5_l or sel6_l or sel7_l or in4 or in5 or in6 or in7)

	case ({sel7_l,sel6_l,sel5_l,sel4_l,sel3_l,sel2_l,sel1_l,sel0_l})
		8'b11111110 : dout = in0 ;
		8'b11111101 : dout = in1 ;
		8'b11111011 : dout = in2 ;
		8'b11110111 : dout = in3 ;
		8'b11101111 : dout = in4 ;
		8'b11011111 : dout = in5 ;
		8'b10111111 : dout = in6 ;
		8'b01111111 : dout = in7 ;
		8'b11111111 : dout = {SIZE{1'bx}} ;
		default : dout = {SIZE{1'bx}} ;
	endcase

endmodule // dp_mux8ds


// ----------------------------------------------------------------------


// 3:1 MUX WITH DECODED SELECTS
module dp_mux3ds (dout, in0, in1, in2, 
		     sel0_l, sel1_l, sel2_l);
// synopsys template

parameter SIZE = 1;

output 	[SIZE-1:0] 	dout;
input	[SIZE-1:0]	in0;
input	[SIZE-1:0]	in1;
input	[SIZE-1:0]	in2;
input			sel0_l;
input			sel1_l;
input			sel2_l;

// reg declaration does not imply state being maintained
// across cycles. Used to construct case statement and
// always updated by inputs every cycle.
reg	[SIZE-1:0]	dout ;





wire [2:0] sel = {sel2_l,sel1_l,sel0_l}; // 0in one_cold
   
always @ (sel0_l or sel1_l or sel2_l or in0 or in1 or in2)

	case ({sel2_l,sel1_l,sel0_l})
		3'b110 : dout = in0 ;
		3'b101 : dout = in1 ;
		3'b011 : dout = in2 ;
	        default : dout = {SIZE{1'bx}} ;
	endcase

endmodule // dp_mux3ds

// ----------------------------------------------------------------------


// 2:1 MUX WITH DECODED SELECTS
module dp_mux2ds (dout, in0, in1,
             sel0_l, sel1_l);
// synopsys template

parameter SIZE = 1;

output  [SIZE-1:0]  dout;
input   [SIZE-1:0]  in0;
input   [SIZE-1:0]  in1;
input           sel0_l;
input           sel1_l;

// reg declaration does not imply state being maintained
// across cycles. Used to construct case statement and
// always updated by inputs every cycle.
reg [SIZE-1:0]  dout ;





wire [1:0] sel = {sel1_l,sel0_l}; // 0in one_cold

always @ (sel0_l or sel1_l or in0 or in1)

    case ({sel1_l,sel0_l})
        3'b10 : dout = in0 ;
        3'b01 : dout = in1 ;
            default : dout = {SIZE{1'bx}} ;
    endcase

endmodule // dp_mux3ds

// ---------------------------------------------------------------------


module dp_buffer(dout, in);
// synopsys template

parameter SIZE = 1;

output 	[SIZE-1:0] 	dout;
input	[SIZE-1:0]	in;

assign dout = in;

endmodule // dp_buffer









// Modified by Princeton University on June 9th, 2015
// ========== Copyright Header Begin ==========================================
// 
// OpenSPARC T1 Processor File: test_stub_scan.v
// Copyright (c) 2006 Sun Microsystems, Inc.  All Rights Reserved.
// DO NOT ALTER OR REMOVE COPYRIGHT NOTICES.
// 
// The above named program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public
// License version 2 as published by the Free Software Foundation.
// 
// The above named program is distributed in the hope that it will be 
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// General Public License for more details.
// 
// You should have received a copy of the GNU General Public
// License along with this work; if not, write to the Free Software
// Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA 02110-1301, USA.
// 
// ========== Copyright Header End ============================================
// ____________________________________________________________________________
//
//  test_stub_bist - Test Stub with Scan Support
// ____________________________________________________________________________
//
// Description: DBB interface for test signal generation
// ____________________________________________________________________________

module test_stub_scan (/*AUTOARG*/
// Outputs
mux_drive_disable, mem_write_disable, sehold, se, testmode_l, 
mem_bypass, so_0, so_1, so_2, 
// Inputs
ctu_tst_pre_grst_l, arst_l, global_shift_enable, 
ctu_tst_scan_disable, ctu_tst_scanmode, ctu_tst_macrotest, 
ctu_tst_short_chain, long_chain_so_0, short_chain_so_0, 
long_chain_so_1, short_chain_so_1, long_chain_so_2, short_chain_so_2
);

   input        ctu_tst_pre_grst_l;
   input        arst_l;                // no longer used
   input        global_shift_enable;
   input        ctu_tst_scan_disable;  // redefined as pin_based_scan
   input        ctu_tst_scanmode;
   input 	ctu_tst_macrotest;
   input 	ctu_tst_short_chain;
   input 	long_chain_so_0;
   input 	short_chain_so_0;
   input 	long_chain_so_1;
   input 	short_chain_so_1;
   input 	long_chain_so_2;
   input 	short_chain_so_2;
   
   output 	mux_drive_disable;
   output 	mem_write_disable;
   output 	sehold;
   output 	se;
   output 	testmode_l;
   output 	mem_bypass;
   output 	so_0;
   output 	so_1;
   output 	so_2;

   wire         pin_based_scan;
   wire         short_chain_en;
   wire         short_chain_select;

   // INTERNAL CLUSTER CONNECTIONS
   //
   // Scan Chain Hookup
   // =================
   //
   // Scan chains have two configurations: long and short.
   // The short chain is typically the first tenth of the
   // long chain. The short chain should contain memory
   // collar flops for deep arrays. The CTU determines
   // which configuration is selected. Up to three chains
   // are supported.
   //
   // The scanout connections from the long and short
   // chains connect to the following inputs:
   //
   // long_chain_so_0, short_chain_so_0 (mandatory)
   // long_chain_so_1, short_chain_so_1 (optional)
   // long_chain_so_2, short_chain_so_2 (optional)
   //
   // The test stub outputs should connect directly to the
   // scanout port(s) of the cluster:
   //
   // so_0 (mandatory), so_1 (optional), so_2 (optional)
   //
   //
   // Static Output Signals
   // =====================
   //
   // testmode_l
   //
   // Local testmode control for overriding gated
   // clocks, asynchronous resets, etc. Asserted
   // for all shift-based test modes.
   //
   // mem_bypass
   //
   // Memory bypass control for arrays without output
   // flops. Allows testing of shadow logic. Asserted
   // for scan test; de-asserted for macrotest.
   //
   //
   // Dynamic Output Signals
   // ======================
   //
   // sehold
   //
   // The sehold signal needs to be set for macrotest
   // to allow holding flops in the array collars
   // to retain their shifted data during capture.
   // Inverted version of scan enable during macrotest.
   //
   // mux_drive_disable (for mux/long chain protection)
   //
   // Activate one-hot mux protection circuitry during
   // scan shift and reset. Formerly known as rst_tri_en.
   // Also used by long chain memories with embedded
   // control.
   //
   // mem_write_disable (for short chain protection)
   //
   // Protects contents of short chain memories during
   // shift and POR.
   //
   // se

   assign  mux_drive_disable  = ~ctu_tst_pre_grst_l | short_chain_select | se;
   assign  mem_write_disable  = ~ctu_tst_pre_grst_l | se;
   assign  sehold             = ctu_tst_macrotest & ~se;
   assign  se                 = global_shift_enable;
   assign  testmode_l         = ~ctu_tst_scanmode;
   assign  mem_bypass         = ~ctu_tst_macrotest & ~testmode_l;
   assign  pin_based_scan     = ctu_tst_scan_disable;
   assign  short_chain_en     = ~(pin_based_scan & se);
   assign  short_chain_select = ctu_tst_short_chain & ~testmode_l & short_chain_en;
   assign  so_0               = short_chain_select ? short_chain_so_0 : long_chain_so_0;
   assign  so_1               = short_chain_select ? short_chain_so_1 : long_chain_so_1;
   assign  so_2               = short_chain_select ? short_chain_so_2 : long_chain_so_2;
   
endmodule // test_stub_scan
// Modified by Princeton University on June 9th, 2015
// ========== Copyright Header Begin ==========================================
// 
// OpenSPARC T1 Processor File: u1.behV
// Copyright (c) 2006 Sun Microsystems, Inc.  All Rights Reserved.
// DO NOT ALTER OR REMOVE COPYRIGHT NOTICES.
// 
// The above named program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public
// License version 2 as published by the Free Software Foundation.
// 
// The above named program is distributed in the hope that it will be 
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// General Public License for more details.
// 
// You should have received a copy of the GNU General Public
// License along with this work; if not, write to the Free Software
// Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA 02110-1301, USA.
// 
// ========== Copyright Header End ============================================
////////////////////////////////////////////////////////////////////////
//
// basic gates {
//
////////////////////////////////////////////////////////////////////////


//bw_u1_inv_0p6x
//
//

module bw_u1_inv_0p6x (
    z,
    a );

    output z;
    input  a;

    assign z = ~( a );

endmodule


//bw_u1_inv_1x
//
//

module bw_u1_inv_1x (
    z,
    a );

    output z;
    input  a;

    assign z = ~( a );

endmodule


//bw_u1_inv_1p4x
//
//

module bw_u1_inv_1p4x (
    z,
    a );

    output z;
    input  a;

    assign z = ~( a );

endmodule


//bw_u1_inv_2x
//
//

module bw_u1_inv_2x (
    z,
    a );

    output z;
    input  a;

    assign z = ~( a );

endmodule


//bw_u1_inv_3x
//
//

module bw_u1_inv_3x (
    z,
    a );

    output z;
    input  a;

    assign z = ~( a );

endmodule


//bw_u1_inv_4x
//
//

module bw_u1_inv_4x (
    z,
    a );

    output z;
    input  a;

    assign z = ~( a );

endmodule



//bw_u1_inv_5x
//
//

module bw_u1_inv_5x (
    z,
    a );

    output z;
    input  a;

    assign z = ~( a );

endmodule


//bw_u1_inv_8x
//
//

module bw_u1_inv_8x (
    z,
    a );

    output z;
    input  a;

    assign z = ~( a );

endmodule


//bw_u1_inv_10x
//
//

module bw_u1_inv_10x (
    z,
    a );

    output z;
    input  a;

    assign z = ~( a );

endmodule


//bw_u1_inv_15x
//
//

module bw_u1_inv_15x (
    z,
    a );

    output z;
    input  a;

    assign z = ~( a );

endmodule


//bw_u1_inv_20x
//
//

module bw_u1_inv_20x (
    z,
    a );

    output z;
    input  a;

    assign z = ~( a );

endmodule


//bw_u1_inv_30x
//
//

module bw_u1_inv_30x (
    z,
    a );

    output z;
    input  a;

    assign z = ~( a );

endmodule


//bw_u1_inv_40x
//
//

module bw_u1_inv_40x (
    z,
    a );

    output z;
    input  a;

    assign z = ~( a );

endmodule

//bw_u1_invh_15x
//
//

module bw_u1_invh_15x (
    z,
    a );

    output z;
    input  a;

    assign z = ~( a );

endmodule

//bw_u1_invh_25x
//
//

module bw_u1_invh_25x (
    z,
    a );

    output z;
    input  a;

    assign z = ~( a );

endmodule


//bw_u1_invh_30x
//
//

module bw_u1_invh_30x (
    z,
    a );

    output z;
    input  a;

    assign z = ~( a );

endmodule


//bw_u1_invh_50x
//
//

module bw_u1_invh_50x (
    z,
    a );

    output z;
    input  a;

    assign z = ~( a );

endmodule


//bw_u1_invh_60x
//
//

module bw_u1_invh_60x (
    z,
    a );

    output z;
    input  a;

    assign z = ~( a );

endmodule




//bw_u1_nand2_0p4x
//
//
module bw_u1_nand2_0p4x (
    z,
    a,
    b );

    output z;
    input  a;
    input  b;

    assign z = ~( a & b );

endmodule


//bw_u1_nand2_0p6x
//
//
module bw_u1_nand2_0p6x (
    z,
    a,
    b );

    output z;
    input  a;
    input  b;

    assign z = ~( a & b );

endmodule


//bw_u1_nand2_1x
//
//
module bw_u1_nand2_1x (
    z,
    a,
    b );

    output z;
    input  a;
    input  b;

    assign z = ~( a & b );

endmodule


//bw_u1_nand2_1p4x
//
//
module bw_u1_nand2_1p4x (
    z,
    a,
    b );

    output z;
    input  a;
    input  b;

    assign z = ~( a & b );

endmodule


//bw_u1_nand2_2x
//
//
module bw_u1_nand2_2x (
    z,
    a,
    b );

    output z;
    input  a;
    input  b;

    assign z = ~( a & b );

endmodule


//bw_u1_nand2_3x
//
//
module bw_u1_nand2_3x (
    z,
    a,
    b );

    output z;
    input  a;
    input  b;

    assign z = ~( a & b );

endmodule


//bw_u1_nand2_4x
//
//
module bw_u1_nand2_4x (
    z,
    a,
    b );

    output z;
    input  a;
    input  b;

    assign z = ~( a & b );

endmodule


//bw_u1_nand2_5x
//
//
module bw_u1_nand2_5x (
    z,
    a,
    b );

    output z;
    input  a;
    input  b;

    assign z = ~( a & b );

endmodule


//bw_u1_nand2_7x
//
//
module bw_u1_nand2_7x (
    z,
    a,
    b );

    output z;
    input  a;
    input  b;

    assign z = ~( a & b );

endmodule


//bw_u1_nand2_10x
//
//
module bw_u1_nand2_10x (
    z,
    a,
    b );

    output z;
    input  a;
    input  b;

    assign z = ~( a & b );

endmodule


//bw_u1_nand2_15x
//
//
module bw_u1_nand2_15x (
    z,
    a,
    b );

    output z;
    input  a;
    input  b;

    assign z = ~( a & b );

endmodule


//bw_u1_nand3_0p4x
//
//
module bw_u1_nand3_0p4x (
    z,
    a,
    b,
    c );

    output z;
    input  a;
    input  b;
    input  c;

    assign z = ~( a & b & c );

endmodule




//bw_u1_nand3_0p6x
//
//
module bw_u1_nand3_0p6x (
    z,
    a,
    b,
    c );

    output z;
    input  a;
    input  b;
    input  c;

    assign z = ~( a & b & c );

endmodule



//bw_u1_nand3_1x

//
//
module bw_u1_nand3_1x (
    z,
    a,  
    b,  
    c );
    
    output z;
    input  a;
    input  b;
    input  c;
    
    assign z = ~( a & b & c );

endmodule


//bw_u1_nand3_1p4x

//
//
module bw_u1_nand3_1p4x (
    z,
    a,  
    b,  
    c );
    
    output z;
    input  a;
    input  b;
    input  c;
    
    assign z = ~( a & b & c );

endmodule


//bw_u1_nand3_2x

//
//
module bw_u1_nand3_2x (
    z,
    a,  
    b,  
    c );
    
    output z;
    input  a;
    input  b;
    input  c;
    
    assign z = ~( a & b & c );

endmodule


//bw_u1_nand3_3x

//
//
module bw_u1_nand3_3x (
    z,
    a,  
    b,  
    c );
    
    output z;
    input  a;
    input  b;
    input  c;
    
    assign z = ~( a & b & c );

endmodule


//bw_u1_nand3_4x

//
//
module bw_u1_nand3_4x (
    z,
    a,  
    b,  
    c );
    
    output z;
    input  a;
    input  b;
    input  c;
    
    assign z = ~( a & b & c );

endmodule


//bw_u1_nand3_5x

//
//
module bw_u1_nand3_5x (
    z,
    a,  
    b,  
    c );
    
    output z;
    input  a;
    input  b;
    input  c;
    
    assign z = ~( a & b & c );

endmodule


//bw_u1_nand3_7x

//
//
module bw_u1_nand3_7x (
    z,
    a,  
    b,  
    c );
    
    output z;
    input  a;
    input  b;
    input  c;
    
    assign z = ~( a & b & c );

endmodule


//bw_u1_nand3_10x

//
//
module bw_u1_nand3_10x (
    z,
    a,  
    b,  
    c );
    
    output z;
    input  a;
    input  b;
    input  c;
    
    assign z = ~( a & b & c );

endmodule


//bw_u1_nand4_0p6x

//
//
module bw_u1_nand4_0p6x (
    z,
    a,  
    b,  
    c,  
    d );
    
    output z;
    input  a;
    input  b;
    input  c;
    input  d;
    
    assign z = ~( a & b & c & d );

endmodule


//bw_u1_nand4_1x
//
//
module bw_u1_nand4_1x (
    z,
    a,
    b,
    c,
    d );

    output z;
    input  a;
    input  b;
    input  c;
    input  d;

    assign z = ~( a & b & c & d );

endmodule


//bw_u1_nand4_1p4x
//
//
module bw_u1_nand4_1p4x (
    z,
    a,
    b,
    c,
    d );

    output z;
    input  a;
    input  b;
    input  c;
    input  d;

    assign z = ~( a & b & c & d );

endmodule


//bw_u1_nand4_2x
//
//
module bw_u1_nand4_2x (
    z,
    a,
    b,
    c,
    d );

    output z;
    input  a;
    input  b;
    input  c;
    input  d;

    assign z = ~( a & b & c & d );

endmodule


//bw_u1_nand4_3x
//
//
module bw_u1_nand4_3x (
    z,
    a,
    b,
    c,
    d );

    output z;
    input  a;
    input  b;
    input  c;
    input  d;

    assign z = ~( a & b & c & d );

endmodule


//bw_u1_nand4_4x
//
//
module bw_u1_nand4_4x (
    z,
    a,
    b,
    c,
    d );

    output z;
    input  a;
    input  b;
    input  c;
    input  d;

    assign z = ~( a & b & c & d );

endmodule


//bw_u1_nand4_6x
//
//

module bw_u1_nand4_6x (
    z,
    a,
    b,
    c,
    d );

    output z;
    input  a;
    input  b;
    input  c;
    input  d;


    nand( z, a, b,c,d);

endmodule

//bw_u1_nand4_8x
//
//

module bw_u1_nand4_8x (
    z,
    a,
    b,
    c,
    d );

    output z;
    input  a;
    input  b;
    input  c;
    input  d;


    nand( z, a, b,c,d);

endmodule

//bw_u1_nor2_0p6x
//
//

module bw_u1_nor2_0p6x (
    z,
    a,
    b );

    output z;
    input  a;
    input  b;

    assign z = ~( a | b );

endmodule


//bw_u1_nor2_1x
//
//

module bw_u1_nor2_1x (
    z,
    a,
    b );

    output z;
    input  a;
    input  b;

    assign z = ~( a | b );

endmodule


//bw_u1_nor2_1p4x
//
//

module bw_u1_nor2_1p4x (
    z,
    a,
    b );

    output z;
    input  a;
    input  b;

    assign z = ~( a | b );

endmodule


//bw_u1_nor2_2x
//
//

module bw_u1_nor2_2x (
    z,
    a,
    b );

    output z;
    input  a;
    input  b;

    assign z = ~( a | b );

endmodule


//bw_u1_nor2_3x
//
//

module bw_u1_nor2_3x (
    z,
    a,
    b );

    output z;
    input  a;
    input  b;

    assign z = ~( a | b );

endmodule


//bw_u1_nor2_4x
//
//

module bw_u1_nor2_4x (
    z,
    a,
    b );

    output z;
    input  a;
    input  b;

    assign z = ~( a | b );

endmodule


//bw_u1_nor2_6x
//
//

module bw_u1_nor2_6x (
    z,
    a,
    b );

    output z;
    input  a;
    input  b;

    assign z = ~( a | b );

endmodule


//bw_u1_nor2_8x
//
//

module bw_u1_nor2_8x (
    z,
    a,
    b );

    output z;
    input  a;
    input  b;

    assign z = ~( a | b );

endmodule


//bw_u1_nor2_12x
//
//

module bw_u1_nor2_12x (
    z,
    a,
    b );

    output z;
    input  a;
    input  b;

    assign z = ~( a | b );

endmodule




//bw_u1_nor3_0p6x
//
//

module bw_u1_nor3_0p6x (
    z,
    a,
    b,
    c );

    output z;
    input  a;
    input  b;
    input  c;

    assign z = ~( a | b | c );

endmodule


//bw_u1_nor3_1x
//
//

module bw_u1_nor3_1x (
    z,
    a,
    b,
    c );

    output z;
    input  a;
    input  b;
    input  c;

    assign z = ~( a | b | c );

endmodule


//bw_u1_nor3_1p4x
//
//

module bw_u1_nor3_1p4x (
    z,
    a,
    b,
    c );

    output z;
    input  a;
    input  b;
    input  c;

    assign z = ~( a | b | c );

endmodule


//bw_u1_nor3_2x
//
//

module bw_u1_nor3_2x (
    z,
    a,
    b,
    c );

    output z;
    input  a;
    input  b;
    input  c;

    assign z = ~( a | b | c );

endmodule


//bw_u1_nor3_3x
//
//

module bw_u1_nor3_3x (
    z,
    a,
    b,
    c );

    output z;
    input  a;
    input  b;
    input  c;

    assign z = ~( a | b | c );

endmodule


//bw_u1_nor3_4x
//
//

module bw_u1_nor3_4x (
    z,
    a,
    b,
    c );

    output z;
    input  a;
    input  b;
    input  c;

    assign z = ~( a | b | c );

endmodule


//bw_u1_nor3_6x
//
//

module bw_u1_nor3_6x (
    z,
    a,
    b,
    c );

    output z;
    input  a;
    input  b;
    input  c;

    assign z = ~( a | b | c );

endmodule


//bw_u1_nor3_8x
//
//

module bw_u1_nor3_8x (
    z,
    a,
    b,
    c );

    output z;
    input  a;
    input  b;
    input  c;

    assign z = ~( a | b | c );

endmodule


//bw_u1_aoi21_0p4x
//
// 
module bw_u1_aoi21_0p4x (
    z,
    b1,
    b2,
    a );

    output z;
    input  b1;
    input  b2;
    input  a;

    assign z = ~(( b1 & b2 ) | ( a ));

endmodule
//bw_u1_aoi21_1x
//
// 
module bw_u1_aoi21_1x (

    z,
    b1,
    b2,
    a );

    output z;
    input  b1;
    input  b2;
    input  a;

    assign z = ~(( b1 & b2 ) | ( a  ));

endmodule
//bw_u1_aoi21_2x
//
// 
module bw_u1_aoi21_2x (
    z,
    b1,
    b2,
    a );

    output z;
    input  b1;
    input  b2;
    input  a;

    assign z = ~(( b1 & b2 ) | ( a ));

endmodule
//bw_u1_aoi21_4x
//
// 
module bw_u1_aoi21_4x (
    z,
    b1,
    b2,
    a );

    output z;
    input  b1;
    input  b2;
    input  a;

    assign z = ~(( b1 & b2 ) | ( a ));

endmodule
//bw_u1_aoi21_8x
//
// 
module bw_u1_aoi21_8x (
    z,
    b1,
    b2,
    a );

    output z;
    input  b1;
    input  b2;
    input  a;

    assign z = ~(( b1 & b2 ) | ( a ));

endmodule
//bw_u1_aoi21_12x
//
// 
module bw_u1_aoi21_12x (
    z,
    b1,
    b2,
    a );

    output z;
    input  b1;
    input  b2;
    input  a;

    assign z = ~(( b1 & b2 ) | ( a ));

endmodule
//bw_u1_aoi22_0p4x
//
// 
module bw_u1_aoi22_0p4x (
    z,
    a1,
    a2,
    b1,
    b2 );

    output z;
    input  a1;
    input  a2;
    input  b1;
    input  b2;

    assign z = ~(( a1 & a2 ) | ( b1 & b2 ));

endmodule
//bw_u1_aoi22_1x
//
// 
module bw_u1_aoi22_1x (
    z,
    b1,
    b2,
    a1,
    a2 );

    output z;
    input  b1;
    input  b2;
    input  a1;
    input  a2;


    assign z = ~(( a1 & a2 ) | ( b1 & b2 ));

endmodule
//bw_u1_aoi22_2x
//
// 
module bw_u1_aoi22_2x (


    z,
    b1,
    b2,
    a1,
    a2 );

    output z;
    input  b1;
    input  b2;
    input  a1;
    input  a2;
 
    assign z = ~(( a1 & a2 ) | ( b1 & b2 ));

endmodule
//bw_u1_aoi22_4x
//
// 
module bw_u1_aoi22_4x (

    z,
    b1,
    b2,
    a1,
    a2 );

    output z;
    input  b1;
    input  b2;
    input  a1;
    input  a2;

    assign z = ~(( a1 & a2 ) | ( b1 & b2 ));

endmodule
//bw_u1_aoi22_8x
//
// 
module bw_u1_aoi22_8x (

    z,
    b1,
    b2,
    a1,
    a2 );

    output z;
    input  b1;
    input  b2;
    input  a1;
    input  a2;

    assign z = ~(( a1 & a2 ) | ( b1 & b2 ));

endmodule
//bw_u1_aoi211_0p3x
//
// 
module bw_u1_aoi211_0p3x (

    z,
    c1,
    c2,
    b,
    a );

    output z;
    input  c1;
    input  c2;
    input  b;
    input  a;

    assign z = ~(( c1 & c2 ) | (a)| (b));

endmodule

//bw_u1_aoi211_1x
//
// 
module bw_u1_aoi211_1x (

    z,
    c1,
    c2,
    b,
    a );

    output z;
    input  c1;
    input  c2;
    input  b;
    input  a;

    assign z = ~(( c1 & c2 ) | (a)| (b));

endmodule

//bw_u1_aoi211_2x
//
// 
module bw_u1_aoi211_2x (



    z,
    c1,
    c2,
    b, 
    a );

    output z;
    input  c1;
    input  c2;
    input  b;
    input  a;
 

    assign z = ~(( c1 & c2 ) | (a)| (b));

endmodule

//bw_u1_aoi211_4x
//
// 
module bw_u1_aoi211_4x (


    z,
    c1,
    c2,
    b, 
    a );

    output z;
    input  c1;
    input  c2;
    input  b;
    input  a;
 


    assign z = ~(( c1 & c2 ) | (a)| (b));

endmodule

//bw_u1_aoi211_8x
//
// 
module bw_u1_aoi211_8x (


    z,
    c1,
    c2,
    b, 
    a );

    output z;
    input  c1;
    input  c2;
    input  b;
    input  a;
 


    assign z = ~(( c1 & c2 ) | (a)| (b));

endmodule

//bw_u1_oai21_0p4x
//
//
module bw_u1_oai21_0p4x (
    z,
    b1,
    b2,
    a );

    output z;
    input  b1;
    input  b2;
    input  a;

    assign z = ~(( b1 | b2 ) & ( a ));

endmodule



//bw_u1_oai21_1x
//
//
module bw_u1_oai21_1x (
    z,
    b1,
    b2,
    a );

    output z;
    input  b1;
    input  b2;
    input  a;

    assign z = ~(( b1 | b2 ) & ( a ));

endmodule



//bw_u1_oai21_2x
//
//
module bw_u1_oai21_2x (
    z,
    b1,
    b2,
    a );

    output z;
    input  b1;
    input  b2;
    input  a;

    assign z = ~(( b1 | b2 ) & ( a ));

endmodule



//bw_u1_oai21_4x
//
//
module bw_u1_oai21_4x (
    z,
    b1,
    b2,
    a );

    output z;
    input  b1;
    input  b2;
    input  a;

    assign z = ~(( b1 | b2 ) & ( a ));

endmodule



//bw_u1_oai21_8x
//
//
module bw_u1_oai21_8x (
    z,
    b1,
    b2,
    a );

    output z;
    input  b1;
    input  b2;
    input  a;

    assign z = ~(( b1 | b2 ) & ( a ));

endmodule



//bw_u1_oai21_12x
//
//
module bw_u1_oai21_12x (
    z,
    b1,
    b2,
    a );

    output z;
    input  b1;
    input  b2;
    input  a;

    assign z = ~(( b1 | b2 ) & ( a ));

endmodule



//bw_u1_oai22_0p4x
// 
module bw_u1_oai22_0p4x (
    z,
    a1,
    a2,
    b1,
    b2 );

    output z;
    input  a1;
    input  a2;
    input  b1;
    input  b2;

    assign z = ~(( a1 | a2 ) & ( b1 | b2 ));

endmodule

//bw_u1_oai22_1x
// 
module bw_u1_oai22_1x (
    z,
    a1,
    a2,
    b1,
    b2 );

    output z;
    input  a1;
    input  a2;
    input  b1;
    input  b2;

    assign z = ~(( a1 | a2 ) & ( b1 | b2 ));

endmodule

//bw_u1_oai22_2x
// 
module bw_u1_oai22_2x (
    z,
    a1,
    a2,
    b1,
    b2 );

    output z;
    input  a1;
    input  a2;
    input  b1;
    input  b2;

    assign z = ~(( a1 | a2 ) & ( b1 | b2 ));

endmodule

//bw_u1_oai22_4x
// 
module bw_u1_oai22_4x (
    z,
    a1,
    a2,
    b1,
    b2 );

    output z;
    input  a1;
    input  a2;
    input  b1;
    input  b2;

    assign z = ~(( a1 | a2 ) & ( b1 | b2 ));

endmodule

//bw_u1_oai22_8x
// 
module bw_u1_oai22_8x (
    z,
    a1,
    a2,
    b1,
    b2 );

    output z;
    input  a1;
    input  a2;
    input  b1;
    input  b2;

    assign z = ~(( a1 | a2 ) & ( b1 | b2 ));

endmodule

//bw_u1_oai211_0p3x
//
//
module bw_u1_oai211_0p3x (
    z,
    c1,
    c2,
    b,
    a );

    output z;
    input  c1;
    input  c2;
    input  b;
    input  a;

    assign z = ~(( c1 | c2 ) & ( a ) & (b));

endmodule

//bw_u1_oai211_1x
//
//
module bw_u1_oai211_1x (
    z,
    c1,
    c2,
    b,
    a );

    output z;
    input  c1;
    input  c2;
    input  b;
    input  a;

    assign z = ~(( c1 | c2 ) & ( a ) & (b));

endmodule

//bw_u1_oai211_2x
//
//
module bw_u1_oai211_2x (
    z,
    c1,
    c2,
    b,
    a );

    output z;
    input  c1;
    input  c2;
    input  b;
    input  a;

    assign z = ~(( c1 | c2 ) & ( a ) & (b));

endmodule

//bw_u1_oai211_4x
//
//
module bw_u1_oai211_4x (
    z,
    c1,
    c2,
    b,
    a );

    output z;
    input  c1;
    input  c2;
    input  b;
    input  a;

    assign z = ~(( c1 | c2 ) & ( a ) & (b));

endmodule

//bw_u1_oai211_8x
//
//
module bw_u1_oai211_8x (
    z,
    c1,
    c2,
    b,
    a );

    output z;
    input  c1;
    input  c2;
    input  b;
    input  a;

    assign z = ~(( c1 | c2 ) & ( a ) & (b));

endmodule

//bw_u1_aoi31_1x
//
// 
module bw_u1_aoi31_1x (


    z,
    b1,
    b2,
    b3,
    a );

    output z;
    input  b1;
    input  b2;
    input  b3;
    input  a;

    assign z = ~(( b1 & b2&b3 ) | ( a ));

endmodule
//bw_u1_aoi31_2x
//
// 
module bw_u1_aoi31_2x (

    z, 
    b1,
    b2, 
    b3, 
    a );
    
    output z; 
    input  b1;
    input  b2;
    input  b3;
    input  a;

    assign z = ~(( b1 & b2&b3 ) | ( a ));

endmodule
//bw_u1_aoi31_4x
//
// 
module bw_u1_aoi31_4x (
    z, 
    b1,
    b2, 
    b3, 
    a );
    
    output z; 
    input  b1;
    input  b2;
    input  b3;
    input  a;

    assign z = ~(( b1 & b2&b3 ) | ( a ));

endmodule
//bw_u1_aoi31_8x
//
// 
module bw_u1_aoi31_8x (

    z, 
    b1,
    b2, 
    b3, 
    a );
    
    output z; 
    input  b1;
    input  b2;
    input  b3;
    input  a;

    assign z = ~(( b1 & b2&b3 ) | ( a ));

endmodule
//bw_u1_aoi32_1x
//
// 
module bw_u1_aoi32_1x (
    z,
    b1,
    b2,
    b3,
    a1,
    a2 );

    output z;
    input  b1;
    input  b2;
    input  b3;
    input  a1;
    input  a2;

    assign z = ~(( b1 & b2&b3 ) | ( a1 & a2 ));

endmodule

//bw_u1_aoi32_2x
//
// 
module bw_u1_aoi32_2x (
    z,
    b1, 
    b2,
    b3,
    a1,
    a2 );

    output z;
    input  b1; 
    input  b2; 
    input  b3; 
    input  a1;
    input  a2;

 

    assign z = ~(( b1 & b2&b3 ) | ( a1 & a2 ));

endmodule

//bw_u1_aoi32_4x
//
// 
module bw_u1_aoi32_4x (

    z,
    b1, 
    b2,
    b3,
    a1,
    a2 );

    output z;
    input  b1; 
    input  b2; 
    input  b3; 
    input  a1;
    input  a2;

 

    assign z = ~(( b1 & b2&b3 ) | ( a1 & a2 ));

endmodule

//bw_u1_aoi32_8x
//
// 
module bw_u1_aoi32_8x (

    z,
    b1, 
    b2,
    b3,
    a1,
    a2 );

    output z;
    input  b1; 
    input  b2; 
    input  b3; 
    input  a1;
    input  a2;

 
    assign z = ~(( b1 & b2&b3 ) | ( a1 & a2 ));

endmodule

//bw_u1_aoi33_1x
//
//
module bw_u1_aoi33_1x (




    z,
    b1,
    b2,
    b3,
    a1,
    a2,
    a3 );

    output z;
    input  b1;
    input  b2;
    input  b3;
    input  a1;
    input  a2;
    input  a3;

    assign z = ~(( b1 & b2&b3 ) | ( a1&a2&a3 ));

endmodule


//bw_u1_aoi33_2x
//
//
module bw_u1_aoi33_2x (

       
    z, 
    b1, 
    b2,  
    b3,  
    a1,  
    a2,  
    a3 );
    
    output z;
    input  b1;
    input  b2;
    input  b3;
    input  a1;
    input  a2;
    input  a3;
    

    assign z = ~(( b1 & b2&b3 ) | ( a1&a2&a3 ));

endmodule


//bw_u1_aoi33_4x
//
//
module bw_u1_aoi33_4x (

       
    z, 
    b1, 
    b2,  
    b3,  
    a1,  
    a2,  
    a3 );
    
    output z;
    input  b1;
    input  b2;
    input  b3;
    input  a1;
    input  a2;
    input  a3;
    


    assign z = ~(( b1 & b2&b3 ) | ( a1&a2&a3 ));

endmodule


//bw_u1_aoi33_8x
//
//
module bw_u1_aoi33_8x (
       
    z, 
    b1, 
    b2,  
    b3,  
    a1,  
    a2,  
    a3 );
    
    output z;
    input  b1;
    input  b2;
    input  b3;
    input  a1;
    input  a2;
    input  a3;
    


    assign z = ~(( b1 & b2&b3 ) | ( a1&a2&a3 ));

endmodule


//bw_u1_aoi221_1x
//
// 
module bw_u1_aoi221_1x (

    z,
    c1,
    c2,
    b1,
    b2,
    a );

    output z;
    input  c1;
    input  c2;
    input  b1;
    input  b2;
    input  a;

    assign z = ~(( c1 & c2 ) | (b1&b2)| (a));

endmodule


//bw_u1_aoi221_2x
//
// 
module bw_u1_aoi221_2x (

    z,
    c1,
    c2,
    b1,
    b2,
    a );

    output z;
    input  c1;
    input  c2;
    input  b1;
    input  b2;
    input  a; 


    assign z = ~(( c1 & c2 ) | (b1&b2)| (a));

endmodule


//bw_u1_aoi221_4x
//
// 
module bw_u1_aoi221_4x (



    z,
    c1,
    c2,
    b1,
    b2,
    a );

    output z;
    input  c1;
    input  c2;
    input  b1;
    input  b2;
    input  a; 


    assign z = ~(( c1 & c2 ) | (b1&b2)| (a));

endmodule


//bw_u1_aoi221_8x
//
// 
module bw_u1_aoi221_8x (
    z,
    c1,
    c2,
    b1,
    b2,
    a );

    output z;
    input  c1;
    input  c2;
    input  b1;
    input  b2;
    input  a; 


    assign z = ~(( c1 & c2 ) | (b1&b2)| (a));

endmodule


//bw_u1_aoi222_1x
//
//
module bw_u1_aoi222_1x (

    z,
    a1,
    a2,
    b1,
    b2,
    c1,
    c2 );

    output z;
    input  a1;
    input  a2;
    input  b1;
    input  b2;
    input  c1;
    input  c2;

    assign z = ~(( c1 & c2 ) | (b1&b2)| (a1& a2));

endmodule

//bw_u1_aoi222_2x
//
//
module bw_u1_aoi222_2x (

    z,
    a1,
    a2,
    b1,
    b2,
    c1,
    c2 );

    output z;
    input  a1;
    input  a2;
    input  b1;
    input  b2;
    input  c1;
    input  c2;

    assign z = ~(( c1 & c2 ) | (b1&b2)| (a1& a2));

endmodule


//bw_u1_aoi222_4x
//
//
module bw_u1_aoi222_4x (

    z,
    a1,
    a2,
    b1,
    b2,
    c1,
    c2 );

    output z;
    input  a1;
    input  a2;
    input  b1;
    input  b2;
    input  c1;
    input  c2;

    assign z = ~(( c1 & c2 ) | (b1&b2)| (a1& a2));

endmodule


//bw_u1_aoi311_1x
//
//
module bw_u1_aoi311_1x (

    z,
    c1,
    c2,
    c3,
    b, 
    a );

    output z;
    input  c1;
    input  c2;
    input  c3;
    input  b;
    input  a;

    assign z = ~(( c1 & c2& c3 ) | (a)| (b));

endmodule




//bw_u1_aoi311_2x
//
//
module bw_u1_aoi311_2x (
    z,
    c1,
    c2,
    c3,
    b, 
    a );

    output z;
    input  c1;
    input  c2;
    input  c3;
    input  b;
    input  a;

    assign z = ~(( c1 & c2& c3 ) | (a)| (b));

endmodule




//bw_u1_aoi311_4x
//
//
module bw_u1_aoi311_4x (
    z,
    c1,
    c2,
    c3,
    b, 
    a );

    output z;
    input  c1;
    input  c2;
    input  c3;
    input  b;
    input  a;


    assign z = ~(( c1 & c2& c3 ) | (a)| (b));

endmodule




//bw_u1_aoi311_8x
//
//
module bw_u1_aoi311_8x (
    z,
    c1,
    c2,
    c3,
    b, 
    a );

    output z;
    input  c1;
    input  c2;
    input  c3;
    input  b;
    input  a;

    assign z = ~(( c1 & c2& c3 ) | (a)| (b));

endmodule




//bw_u1_oai31_1x
//
//
module bw_u1_oai31_1x (
    z,
    b1,
    b2,
    b3,
    a );

    output z;
    input  b1;
    input  b2;
    input  b3;
    input  a;

    assign z = ~(( b1 | b2|b3 ) & ( a ));

endmodule




//bw_u1_oai31_2x
//
//
module bw_u1_oai31_2x (
    z,
    b1,
    b2,
    b3,
    a );

    output z;
    input  b1;
    input  b2;
    input  b3;
    input  a;

    assign z = ~(( b1 | b2|b3 ) & ( a ));

endmodule




//bw_u1_oai31_4x
//
//
module bw_u1_oai31_4x (
    z,
    b1,
    b2,
    b3,
    a );

    output z;
    input  b1;
    input  b2;
    input  b3;
    input  a;

    assign z = ~(( b1 | b2|b3 ) & ( a ));

endmodule




//bw_u1_oai31_8x
//
//
module bw_u1_oai31_8x (
    z,
    b1,
    b2,
    b3,
    a );

    output z;
    input  b1;
    input  b2;
    input  b3;
    input  a;

    assign z = ~(( b1 | b2|b3 ) & ( a ));

endmodule




//bw_u1_oai32_1x
//
//
module bw_u1_oai32_1x (
    z,
    b1,
    b2,
    b3,
    a1,
    a2 );

    output z;
    input  b1;
    input  b2;
    input  b3;
    input  a1;
    input  a2;

    assign z = ~(( b1 | b2 | b3 ) & ( a1 | a2 ));

endmodule



//bw_u1_oai32_2x
//
//
module bw_u1_oai32_2x (
    z,
    b1,
    b2,
    b3,
    a1,
    a2 );

    output z;
    input  b1;
    input  b2;
    input  b3;
    input  a1;
    input  a2;

    assign z = ~(( b1 | b2 | b3 ) & ( a1 | a2 ));

endmodule



//bw_u1_oai32_4x
//
//
module bw_u1_oai32_4x (
    z,
    b1,
    b2,
    b3,
    a1,
    a2 );

    output z;
    input  b1;
    input  b2;
    input  b3;
    input  a1;
    input  a2;

    assign z = ~(( b1 | b2 | b3 ) & ( a1 | a2 ));

endmodule



//bw_u1_oai32_8x
//
//
module bw_u1_oai32_8x (
    z,
    b1,
    b2,
    b3,
    a1,
    a2 );

    output z;
    input  b1;
    input  b2;
    input  b3;
    input  a1;
    input  a2;

    assign z = ~(( b1 | b2 | b3 ) & ( a1 | a2 ));

endmodule



//bw_u1_oai33_1x
//
//
module bw_u1_oai33_1x (
    z,
    b1,
    b2,
    b3,
    a1,
    a2,
    a3 );

    output z;
    input  b1;
    input  b2;
    input  b3;
    input  a1;
    input  a2;
    input  a3;

    assign z = ~(( b1 | b2|b3 ) & ( a1|a2|a3 ));

endmodule


//bw_u1_oai33_2x
//
//
module bw_u1_oai33_2x (
    z,
    b1,
    b2,
    b3,
    a1,
    a2,
    a3 );

    output z;
    input  b1;
    input  b2;
    input  b3;
    input  a1;
    input  a2;
    input  a3;

    assign z = ~(( b1 | b2|b3 ) & ( a1|a2|a3 ));

endmodule


//bw_u1_oai33_4x
//
//
module bw_u1_oai33_4x (
    z,
    b1,
    b2,
    b3,
    a1,
    a2,
    a3 );

    output z;
    input  b1;
    input  b2;
    input  b3;
    input  a1;
    input  a2;
    input  a3;

    assign z = ~(( b1 | b2|b3 ) & ( a1|a2|a3 ));

endmodule


//bw_u1_oai33_8x
//
//
module bw_u1_oai33_8x (
    z,
    b1,
    b2,
    b3,
    a1,
    a2,
    a3 );

    output z;
    input  b1;
    input  b2;
    input  b3;
    input  a1;
    input  a2;
    input  a3;

    assign z = ~(( b1 | b2|b3 ) & ( a1|a2|a3 ));

endmodule


//bw_u1_oai221_1x
//
//
module bw_u1_oai221_1x (
    z,
    c1,
    c2,
    b1,
    b2,
    a );

    output z;
    input  c1;
    input  c2;
    input  b1;
    input  b2;
    input  a;

    assign z = ~(( c1 | c2 ) & ( a ) & (b1|b2));

endmodule

//bw_u1_oai221_2x
//
//
module bw_u1_oai221_2x (
    z,
    c1,
    c2,
    b1,
    b2,
    a );

    output z;
    input  c1;
    input  c2;
    input  b1;
    input  b2;
    input  a;

    assign z = ~(( c1 | c2 ) & ( a ) & (b1|b2));

endmodule

//bw_u1_oai221_4x
//
//
module bw_u1_oai221_4x (
    z,
    c1,
    c2,
    b1,
    b2,
    a );

    output z;
    input  c1;
    input  c2;
    input  b1;
    input  b2;
    input  a;

    assign z = ~(( c1 | c2 ) & ( a ) & (b1|b2));

endmodule

//bw_u1_oai221_8x
//
//
module bw_u1_oai221_8x (
    z,
    c1,
    c2,
    b1,
    b2,
    a );

    output z;
    input  c1;
    input  c2;
    input  b1;
    input  b2;
    input  a;

    assign z = ~(( c1 | c2 ) & ( a ) & (b1|b2));

endmodule

//bw_u1_oai222_1x
//
//
module bw_u1_oai222_1x (
    z,
    c1,
    c2,
    b1,
    b2,
    a1,
    a2 );

    output z;
    input  c1;
    input  c2;
    input  b1;
    input  b2;
    input  a1;
    input  a2;

    assign z = ~(( c1 | c2 ) & ( a1|a2 ) & (b1|b2));

endmodule


//bw_u1_oai222_2x
//
//
module bw_u1_oai222_2x (
    z,
    c1,
    c2,
    b1,
    b2,
    a1,
    a2 );

    output z;
    input  c1;
    input  c2;
    input  b1;
    input  b2;
    input  a1;
    input  a2;

    assign z = ~(( c1 | c2 ) & ( a1|a2 ) & (b1|b2));

endmodule


//bw_u1_oai222_4x
//
//
module bw_u1_oai222_4x (
    z,
    c1,
    c2,
    b1,
    b2,
    a1,
    a2 );

    output z;
    input  c1;
    input  c2;
    input  b1;
    input  b2;
    input  a1;
    input  a2;

    assign z = ~(( c1 | c2 ) & ( a1|a2 ) & (b1|b2));

endmodule


//bw_u1_oai311_1x
//
//
module bw_u1_oai311_1x (
    z,
    c1,
    c2,
    c3,
    b,
    a );

    output z;
    input  c1;
    input  c2;
    input  c3;
    input  b;
    input  a;

    assign z = ~(( c1 | c2|c3 ) & ( a ) & (b));

endmodule


//bw_u1_oai311_2x
//
//
module bw_u1_oai311_2x (
    z,
    c1,
    c2,
    c3,
    b,
    a );

    output z;
    input  c1;
    input  c2;
    input  c3;
    input  b;
    input  a;

    assign z = ~(( c1 | c2|c3 ) & ( a ) & (b));

endmodule


//bw_u1_oai311_4x
//
//
module bw_u1_oai311_4x (
    z,
    c1,
    c2,
    c3,
    b,
    a );

    output z;
    input  c1;
    input  c2;
    input  c3;
    input  b;
    input  a;

    assign z = ~(( c1 | c2 | c3 ) & ( a ) & (b));

endmodule


//bw_u1_oai311_8x
//
//
module bw_u1_oai311_8x (
    z,
    c1,
    c2,
    c3,
    b,
    a );

    output z;
    input  c1;
    input  c2;
    input  c3;
    input  b;
    input  a;

    assign z = ~(( c1 | c2|c3 ) & ( a ) & (b));

endmodule


//bw_u1_muxi21_0p6x



module bw_u1_muxi21_0p6x (z, d0, d1, s);
output z;
input  d0, d1, s;

    assign z = s ? ~d1 : ~d0;
endmodule


//bw_u1_muxi21_1x



module bw_u1_muxi21_1x (z, d0, d1, s);
output z;
input  d0, d1, s;

    assign z = s ? ~d1 : ~d0;
endmodule







//bw_u1_muxi21_2x



module bw_u1_muxi21_2x (z, d0, d1, s);
output z;
input  d0, d1, s;

    assign z = s ? ~d1 : ~d0;
endmodule


//bw_u1_muxi21_4x



module bw_u1_muxi21_4x (z, d0, d1, s);
output z;
input  d0, d1, s;

    assign z = s ? ~d1 : ~d0;
endmodule




//bw_u1_muxi21_6x


module bw_u1_muxi21_6x (z, d0, d1, s);
output z;
input  d0, d1, s;

    assign z = s ? ~d1 : ~d0;
endmodule

//bw_u1_muxi31d_4x
//

module bw_u1_muxi31d_4x (z, d0, d1, d2, s0, s1, s2);
output z;
input  d0, d1, d2, s0, s1, s2;
        zmuxi31d_prim i0 ( z, d0, d1, d2, s0, s1, s2 );
endmodule

//bw_u1_muxi41d_4x
//

module bw_u1_muxi41d_4x (z, d0, d1, d2, d3, s0, s1, s2, s3);
output z;
input  d0, d1, d2, d3, s0, s1, s2, s3;
        zmuxi41d_prim i0 ( z, d0, d1, d2, d3, s0, s1, s2, s3 );
endmodule

//bw_u1_muxi41d_6x
//

module bw_u1_muxi41d_6x (z, d0, d1, d2, d3, s0, s1, s2, s3);
output z;
input  d0, d1, d2, d3, s0, s1, s2, s3;
        zmuxi41d_prim i0 ( z, d0, d1, d2, d3, s0, s1, s2, s3 );
endmodule
 

//bw_u1_xor2_0p6x
//
// 
module bw_u1_xor2_0p6x (
    z,
    a,
    b );

    output z;
    input  a;
    input  b;

    assign z = ( a ^ b );

endmodule
//bw_u1_xor2_1x
//
// 
module bw_u1_xor2_1x (
    z,
    a,
    b );

    output z;
    input  a;
    input  b;

    assign z = ( a ^ b );

endmodule
//bw_u1_xor2_2x
//
// 
module bw_u1_xor2_2x (
    z,
    a,
    b );

    output z;
    input  a;
    input  b;

    assign z = ( a ^ b );

endmodule
//bw_u1_xor2_4x
//
// 
module bw_u1_xor2_4x (
    z,
    a,
    b );

    output z;
    input  a;
    input  b;

    assign z = ( a ^ b );

endmodule
//bw_u1_xnor2_0p6x
//
// 
module bw_u1_xnor2_0p6x (
    z,
    a,
    b );

    output z;
    input  a;
    input  b;

    assign z = ~( a ^ b );

endmodule
//bw_u1_xnor2_1x
//
// 
module bw_u1_xnor2_1x (
    z,
    a,
    b );

    output z;
    input  a;
    input  b;

    assign z = ~( a ^ b );

endmodule
//bw_u1_xnor2_2x
//
// 
module bw_u1_xnor2_2x (
    z,
    a,
    b );

    output z;
    input  a;
    input  b;

    assign z = ~( a ^ b );

endmodule
//bw_u1_xnor2_4x
//
// 
module bw_u1_xnor2_4x (
    z,
    a,
    b );

    output z;
    input  a;
    input  b;

    assign z = ~( a ^ b );

endmodule

//bw_u1_buf_1x
//

module bw_u1_buf_1x (
    z,
    a );

    output z;
    input  a;

    assign z = ( a );

endmodule

//bw_u1_buf_5x
//

module bw_u1_buf_5x (
    z,
    a );

    output z;
    input  a;

    assign z = ( a );

endmodule


//bw_u1_buf_10x
//

module bw_u1_buf_10x (
    z,
    a );

    output z;
    input  a;

    assign z = ( a );

endmodule


//bw_u1_buf_15x
//

module bw_u1_buf_15x (
    z,
    a );

    output z;
    input  a;

    assign z = ( a );

endmodule


//bw_u1_buf_20x
//

module bw_u1_buf_20x (
    z,
    a );

    output z;
    input  a;

    assign z = ( a );

endmodule


//bw_u1_buf_30x
//

module bw_u1_buf_30x (
    z,
    a );

    output z;
    input  a;

    assign z = ( a );

endmodule


//bw_u1_buf_40x
//

module bw_u1_buf_40x (
    z,
    a );

    output z;
    input  a;

    assign z = ( a );

endmodule


//bw_u1_ao2222_1x
//
//
module bw_u1_ao2222_1x (

    z,
    a1,
    a2,
    b1,
    b2,
    c1,
    c2,
    d1,
    d2 );

    output z;
    input  a1;
    input  a2;
    input  b1;
    input  b2;
    input  c1;
    input  c2;
    input  d1;
    input  d2;

    assign z = ((d1&d2) | ( c1 & c2 ) | (b1&b2)| (a1& a2));

endmodule


//bw_u1_ao2222_2x
//
//
module bw_u1_ao2222_2x (

    z,
    a1,
    a2,
    b1,
    b2,
    c1,
    c2,
    d1,
    d2 );

    output z;
    input  a1;
    input  a2;
    input  b1;
    input  b2;
    input  c1;
    input  c2;
    input  d1;
    input  d2;

    assign z = ((d1&d2) | ( c1 & c2 ) | (b1&b2)| (a1& a2));

endmodule

//bw_u1_ao2222_4x
//
//
module bw_u1_ao2222_4x (

    z,
    a1,
    a2,
    b1,
    b2,
    c1,
    c2,
    d1,
    d2 );

    output z;
    input  a1;
    input  a2;
    input  b1;
    input  b2;
    input  c1;
    input  c2;
    input  d1;
    input  d2;

    assign z = ((d1&d2) | ( c1 & c2 ) | (b1&b2)| (a1& a2));

endmodule

////////////////////////////////////////////////////////////////////////
//
// flipflops {
//
////////////////////////////////////////////////////////////////////////

//      scanable D-flipflop with scanout

module bw_u1_soff_1x (q, so, ck, d, se, sd);
output q, so;
input  ck, d, se, sd;
        zsoff_prim i0 ( q, so, ck, d, se, sd );
endmodule

module bw_u1_soff_2x (q, so, ck, d, se, sd);
output q, so;
input  ck, d, se, sd;
        zsoff_prim i0 ( q, so, ck, d, se, sd );
endmodule

module bw_u1_soff_4x (q, so, ck, d, se, sd);
output q, so;
input  ck, d, se, sd;
        zsoff_prim i0 ( q, so, ck, d, se, sd );
endmodule

module bw_u1_soff_8x (q, so, ck, d, se, sd);
output q, so;
input  ck, d, se, sd;
        zsoff_prim i0 ( q, so, ck, d, se, sd );
endmodule

//      fast scanable D-flipflop with scanout with inverted Q output

module bw_u1_soffi_4x (q_l, so, ck, d, se, sd);
output q_l, so;
input  ck, d, se, sd;
        zsoffi_prim i0 ( q_l, so, ck, d, se, sd );
endmodule
  
module bw_u1_soffi_8x (q_l, so, ck, d, se, sd);
output q_l, so;
input  ck, d, se, sd;
        zsoffi_prim i0 ( q_l, so, ck, d, se, sd );
endmodule

//      scanable D-flipflop with scanout with 2-to-1 input mux

module bw_u1_soffm2_4x (q, so, ck, d0, d1, s, se, sd);
output q, so;
input  ck, d0, d1, s, se, sd;
        zsoffm2_prim i0 ( q, so, ck, d0, d1, s, se, sd );
endmodule

module bw_u1_soffm2_8x (q, so, ck, d0, d1, s, se, sd);
output q, so;
input  ck, d0, d1, s, se, sd;
        zsoffm2_prim i0 ( q, so, ck, d0, d1, s, se, sd );
endmodule

//      scanable D-flipflop with scanout with sync reset-bar

module bw_u1_soffr_2x (q, so, ck, d, se, sd, r_l);
output q, so;
input  ck, d, se, sd, r_l;
        zsoffr_prim i0 ( q, so, ck, d, se, sd, r_l );
endmodule
  
module bw_u1_soffr_4x (q, so, ck, d, se, sd, r_l);
output q, so;
input  ck, d, se, sd, r_l;
        zsoffr_prim i0 ( q, so, ck, d, se, sd, r_l );
endmodule

module bw_u1_soffr_8x (q, so, ck, d, se, sd, r_l);
output q, so;
input  ck, d, se, sd, r_l;
        zsoffr_prim i0 ( q, so, ck, d, se, sd, r_l );
endmodule

//bw_u1_soffasr_2x

module bw_u1_soffasr_2x (q, so, ck, d, r_l, s_l, se, sd);
output q, so;
input  ck, d, r_l, s_l, se, sd;
        zsoffasr_prim i0 (q, so, ck, d, r_l, s_l, se, sd);
endmodule


//bw_u1_ckbuf_1p5x


module bw_u1_ckbuf_1p5x  (clk, rclk);
output clk;
input  rclk;
        buf (clk, rclk);
endmodule


//bw_u1_ckbuf_3x


module bw_u1_ckbuf_3x  (clk, rclk);
output clk;
input  rclk;
        buf (clk, rclk);
endmodule

//bw_u1_ckbuf_4p5x


module bw_u1_ckbuf_4p5x  (clk, rclk);
output clk;
input  rclk;
        buf (clk, rclk);
endmodule


//bw_u1_ckbuf_6x


module bw_u1_ckbuf_6x  (clk, rclk);
output clk;
input  rclk;
        buf (clk, rclk);
endmodule

//bw_u1_ckbuf_7x
//

module bw_u1_ckbuf_7x  (clk, rclk);
output clk;
input  rclk;
        buf (clk, rclk);
endmodule

//bw_u1_ckbuf_8x
//
module bw_u1_ckbuf_8x  (clk, rclk);
output clk;
input  rclk;
        buf (clk, rclk);
endmodule


//bw_u1_ckbuf_11x
//

module bw_u1_ckbuf_11x (clk, rclk);
output clk;
input  rclk;

    assign clk = ( rclk );

endmodule

//bw_u1_ckbuf_14x
//

module bw_u1_ckbuf_14x (clk, rclk);
output clk;
input  rclk;

    assign clk = ( rclk );

endmodule

//bw_u1_ckbuf_17x
//

module bw_u1_ckbuf_17x (clk, rclk);
output clk;
input  rclk;

    assign clk = ( rclk );

endmodule




//bw_u1_ckbuf_19x
//

module bw_u1_ckbuf_19x (clk, rclk);
output clk;
input  rclk;

    assign clk = ( rclk );

endmodule




//bw_u1_ckbuf_22x
//

module bw_u1_ckbuf_22x (clk, rclk);
output clk;
input  rclk;

    assign clk = ( rclk );

endmodule

//bw_u1_ckbuf_25x
//

module bw_u1_ckbuf_25x (clk, rclk);
output clk;
input  rclk;

    assign clk = ( rclk );

endmodule


//bw_u1_ckbuf_28x
//

module bw_u1_ckbuf_28x (clk, rclk);
output clk;
input  rclk;

    assign clk = ( rclk );

endmodule


//bw_u1_ckbuf_30x
//

module bw_u1_ckbuf_30x (clk, rclk);
output clk;
input  rclk;

    assign clk = ( rclk );

endmodule

//bw_u1_ckbuf_33x
//

module bw_u1_ckbuf_33x (clk, rclk);
output clk;
input  rclk;

    assign clk = ( rclk );

endmodule

//bw_u1_ckbuf_40x
//

module bw_u1_ckbuf_40x (clk, rclk);
output clk;
input  rclk;

    assign clk = ( rclk );

endmodule


// gated clock buffers


module bw_u1_ckenbuf_6x  (clk, rclk, en_l, tm_l);
output clk;
input  rclk, en_l, tm_l;
        zckenbuf_prim i0 ( clk, rclk, en_l, tm_l );
endmodule 

module bw_u1_ckenbuf_14x (clk, rclk, en_l, tm_l);
output clk;
input  rclk, en_l, tm_l;
        zckenbuf_prim i0 ( clk, rclk, en_l, tm_l );
endmodule   

////////////////////////////////////////////////////////////////////////
//
// half cells
//
////////////////////////////////////////////////////////////////////////



module bw_u1_zhinv_0p6x (z, a);
output z;
input  a;
        not (z, a);
endmodule


module bw_u1_zhinv_1x (z, a);
output z;
input  a;
        not (z, a);
endmodule



module bw_u1_zhinv_1p4x (z, a);
output z;
input  a;
        not (z, a);
endmodule


module bw_u1_zhinv_2x (z, a);
output z;
input  a;
        not (z, a);
endmodule



module bw_u1_zhinv_3x (z, a);
output z;
input  a;
        not (z, a);
endmodule



module bw_u1_zhinv_4x (z, a);
output z;
input  a;
        not (z, a);
endmodule



module bw_u1_zhnand2_0p4x (z, a, b);
output z;
input  a, b;
        nand (z, a, b);
endmodule


module bw_u1_zhnand2_0p6x (z, a, b);
output z;   
input  a, b;
        nand (z, a, b);
endmodule   


module bw_u1_zhnand2_1x (z, a, b);
output z;   
input  a, b;
        nand (z, a, b);
endmodule   


module bw_u1_zhnand2_1p4x (z, a, b);
output z;   
input  a, b;
        nand (z, a, b);
endmodule   


module bw_u1_zhnand2_2x (z, a, b);
output z;   
input  a, b;
        nand (z, a, b);
endmodule   


module bw_u1_zhnand2_3x (z, a, b);
output z;   
input  a, b;
        nand (z, a, b);
endmodule   


module bw_u1_zhnand3_0p6x (z, a, b, c);
output z;
input  a, b, c;
        nand (z, a, b, c);
endmodule

module bw_u1_zhnand3_1x (z, a, b, c);
output z;
input  a, b, c;
        nand (z, a, b, c);
endmodule

module bw_u1_zhnand3_2x (z, a, b, c);
output z;
input  a, b, c;
        nand (z, a, b, c);
endmodule


module bw_u1_zhnand4_0p6x (z, a, b, c, d);
output z;
input  a, b, c, d;
        nand (z, a, b, c, d);
endmodule

module bw_u1_zhnand4_1x (z, a, b, c, d);
output z;
input  a, b, c, d;
        nand (z, a, b, c, d);
endmodule

module bw_u1_zhnand4_2x (z, a, b, c, d);
output z;
input  a, b, c, d;
        nand (z, a, b, c, d);
endmodule


        
module bw_u1_zhnor2_0p6x (z, a, b);
output z;
input  a, b;
        nor (z, a, b);
endmodule

module bw_u1_zhnor2_1x (z, a, b);
output z;   
input  a, b;
        nor (z, a, b);
endmodule

module bw_u1_zhnor2_2x (z, a, b);
output z;   
input  a, b;
        nor (z, a, b);
endmodule



module bw_u1_zhnor3_0p6x (z, a, b, c);
output z;
input  a, b, c;
        nor (z, a, b, c);
endmodule


module bw_u1_zhaoi21_0p4x (z,b1,b2,a);

    output z;   
    input  b1;
    input  b2;
    input  a;

    assign z = ~(( b1 & b2 ) | ( a ));
    
endmodule



module bw_u1_zhaoi21_1x (z, a, b1, b2);

    output z;
    input  b1;
    input  b2;
    input  a;

    assign z = ~(( b1 & b2 ) | ( a ));

endmodule



module bw_u1_zhoai21_1x (z,b1,b2,a );
    
    output z;
    input  b1;
    input  b2;  
    input  a;
  
    assign z = ~(( b1 | b2 ) & ( a ));
      
endmodule




module bw_u1_zhoai211_0p3x (z, a, b, c1, c2);
    output z; 
    input  c1;  
    input  c2;
    input  b;
    input  a;
      
    assign z = ~(( c1 | c2 ) & ( a ) & (b));
       
endmodule





module bw_u1_zhoai211_1x (z, a, b, c1, c2);
output z;
input  a, b, c1, c2;
    assign z = ~(( c1 | c2 ) & ( a ) & (b));
       
endmodule





/////////////// Scan data lock up latch ///////////////

module bw_u1_scanlg_2x (so, sd, ck, se);
output so;
input sd, ck, se;

reg so_l;

    assign so = ~so_l;
    always @ ( ck or sd or se )
       if (~ck) so_l <= ~(sd & se) ;

endmodule

module bw_u1_scanl_2x (so, sd, ck);
output so;
input sd, ck;

reg so_l;

    assign so = ~so_l;
    always @ ( ck or sd )
       if (~ck) so_l <= ~sd ;

endmodule



////////////////// Synchronizer ////////////////

module bw_u1_syncff_4x (q, so, ck, d, se, sd);
output q, so;
input  ck, d, se, sd;

reg    q_r;
  always @ (posedge ck)
      q_r <= se ? sd : d;
  assign q  = q_r;
  assign so = q_r;

endmodule




////////////////////////////////////////////////////////////////////////
//
// non library cells
// 
////////////////////////////////////////////////////////////////////////

// These cells are used only in custom DP macros
// Do not use in any block design without prior permission


module bw_u1_zzeccxor2_5x (z, a, b); 
 output z; 
 input a, b;
    assign z = ( a ^ b );

endmodule



module bw_u1_zzmulcsa42_5x (sum, carry, cout, a, b, c, d, cin);
output sum, carry, cout;
input  a, b, c, d, cin;
wire and_cin_b, or_cin_b, xor_a_c_d, and_or_cin_b_xor_a_c_d;
wire and_a_c, and_a_d, and_c_d;
        assign sum   = cin ^ a ^ b ^ c ^ d;
        assign carry = cin & b | (cin | b) & (a ^ c ^ d);
        assign cout  = a & c | a & d | c & d;
endmodule



module bw_u1_zzmulcsa32_5x (sum, cout, a, b, c);
output sum, cout;
input  a, b, c;
wire and_a_b, and_a_c, and_b_c;
        assign sum  = a ^ b ^ c ;
        assign cout = a & b | a & c | b & c ;
endmodule



module bw_u1_zzmulppmuxi21_2x ( z, d0, d1, s );
output  z;
input  d0, d1, s;
    assign z = s ? ~d1 : ~d0;
endmodule



module bw_u1_zzmulnand2_2x ( z, a, b );
output z;
input  a;
input  b;
    assign z = ~( a & b );
endmodule



// Primitives




module zmuxi31d_prim (z, d0, d1, d2, s0, s1, s2);
output z;
input  d0, d1, d2, s0, s1, s2;
// for Blacktie



wire [2:0] sel = {s0,s1,s2}; // 0in one_hot
reg z;
    always @ (s2 or d2 or s1 or d1 or s0 or d0)
        casez ({s2,d2,s1,d1,s0,d0})
            6'b0?0?10: z = 1'b1;  
            6'b0?0?11: z = 1'b0;  
            6'b0?100?: z = 1'b1;  
            6'b0?110?: z = 1'b0;  
            6'b0?1010: z = 1'b1;  
            6'b0?1111: z = 1'b0;  
            6'b100?0?: z = 1'b1;  
            6'b110?0?: z = 1'b0;  
            6'b100?10: z = 1'b1;  
            6'b110?11: z = 1'b0;  
            6'b10100?: z = 1'b1;  
            6'b11110?: z = 1'b0;  
            6'b101010: z = 1'b1;  
            6'b111111: z = 1'b0;  
            default: z = 1'bx;
        endcase
endmodule







module zmuxi41d_prim (z, d0, d1, d2, d3, s0, s1, s2, s3);
output z;
input  d0, d1, d2, d3, s0, s1, s2, s3;
// for Blacktie



wire [3:0] sel = {s0,s1,s2,s3}; // 0in one_hot
reg z;
    always @ (s3 or d3 or s2 or d2 or s1 or d1 or s0 or d0)
        casez ({s3,d3,s2,d2,s1,d1,s0,d0})
            8'b0?0?0?10: z = 1'b1;
            8'b0?0?0?11: z = 1'b0;
            8'b0?0?100?: z = 1'b1;
            8'b0?0?110?: z = 1'b0;
            8'b0?0?1010: z = 1'b1;
            8'b0?0?1111: z = 1'b0;
            8'b0?100?0?: z = 1'b1;
            8'b0?110?0?: z = 1'b0;
            8'b0?100?10: z = 1'b1;
            8'b0?110?11: z = 1'b0;
            8'b0?10100?: z = 1'b1;
            8'b0?11110?: z = 1'b0;
            8'b0?101010: z = 1'b1;
            8'b0?111111: z = 1'b0;
            8'b100?0?0?: z = 1'b1;
            8'b110?0?0?: z = 1'b0;
            8'b100?0?10: z = 1'b1;
            8'b110?0?11: z = 1'b0;
            8'b100?100?: z = 1'b1;
            8'b110?110?: z = 1'b0;
            8'b100?1010: z = 1'b1;
            8'b110?1111: z = 1'b0;
            8'b10100?0?: z = 1'b1;
            8'b11110?0?: z = 1'b0;
            8'b10100?10: z = 1'b1;
            8'b11110?11: z = 1'b0;
            8'b1010100?: z = 1'b1;
            8'b1111110?: z = 1'b0;
            8'b10101010: z = 1'b1;
            8'b11111111: z = 1'b0;
            default: z = 1'bx;
        endcase   
endmodule



module zsoff_prim (q, so, ck, d, se, sd);
output q, so;
input  ck, d, se, sd;
reg    q_r;
  always @ (posedge ck)
      q_r <= se ? sd : d;
  assign q  = q_r;
  assign so = q_r ;
endmodule


module zsoffr_prim (q, so, ck, d, se, sd, r_l);
output q, so;
input  ck, d, se, sd, r_l;
reg    q_r;
  always @ (posedge ck)
      q_r <= se ? sd : (d & r_l) ;
  assign q  = q_r;
  assign so = q_r;
endmodule


module zsoffi_prim (q_l, so, ck, d, se, sd);
output q_l, so;
input  ck, d, se, sd;
reg    q_r;
  always @ (posedge ck)
      q_r <= se ? sd : d;
  assign q_l = ~q_r;
  assign so  = q_r;
endmodule



module zsoffm2_prim (q, so, ck, d0, d1, s, se, sd);
output q, so;
input  ck, d0, d1, s, se, sd;
reg    q_r;
  always @ (posedge ck)
      q_r <= se ? sd : (s ? d1 : d0) ;
  assign q  = q_r;
  assign so = q_r;
endmodule

module zsoffasr_prim (q, so, ck, d, r_l, s_l, se, sd);
  output q, so;
  input ck, d, r_l, s_l, se, sd;

  // asynchronous reset and asynchronous set
  // (priority: r_l > s_l > se > d)
  reg q;
  wire so;

  always @ (posedge ck or negedge r_l or negedge s_l) begin
		if(~r_l) q <= 1'b0;
		else if (~s_l) q <= r_l;
		else if (se) q <= r_l & s_l & sd;
		else q <= r_l & s_l & (~se) & d;
  end

  assign so = q | ~se;

endmodule



module zckenbuf_prim (clk, rclk, en_l, tm_l);
output clk;
input  rclk, en_l, tm_l;
reg    clken;

  always @ (rclk or en_l or tm_l)
    if (!rclk)  //latch opens on rclk low phase
      clken <= ~en_l | ~tm_l;
  assign clk = clken & rclk;

endmodule

module bw_mckbuf_40x (clk, rclk, en);
output clk;
input  rclk;
input  en;

    assign clk = rclk & en ;

endmodule

module bw_mckbuf_33x (clk, rclk, en);
output clk;
input  rclk;
input  en;

    assign clk = rclk & en ;

endmodule

module bw_mckbuf_30x (clk, rclk, en);
output clk;
input  rclk;
input  en;

    assign clk = rclk & en ;

endmodule

module bw_mckbuf_28x (clk, rclk, en);
output clk;
input  rclk;
input  en;

    assign clk = rclk & en ;

endmodule

module bw_mckbuf_25x (clk, rclk, en);
output clk;
input  rclk;
input  en;

    assign clk = rclk & en ;

endmodule

module bw_mckbuf_22x (clk, rclk, en);
output clk;
input  rclk;
input  en;

    assign clk = rclk & en ;

endmodule

module bw_mckbuf_19x (clk, rclk, en);
output clk;
input  rclk;
input  en;

    assign clk = rclk & en ;

endmodule

module bw_mckbuf_17x (clk, rclk, en);
output clk;
input  rclk;
input  en;

    assign clk = rclk & en ;

endmodule

module bw_mckbuf_14x (clk, rclk, en);
output clk;
input  rclk;
input  en;

    assign clk = rclk & en ;

endmodule

module bw_mckbuf_11x (clk, rclk, en);
output clk;
input  rclk;
input  en;

    assign clk = rclk & en ;

endmodule

module bw_mckbuf_8x (clk, rclk, en);
output clk;
input  rclk;
input  en;

    assign clk = rclk & en ;

endmodule

module bw_mckbuf_7x (clk, rclk, en);
output clk;
input  rclk;
input  en;

    assign clk = rclk & en ;

endmodule

module bw_mckbuf_6x (clk, rclk, en);
output clk;
input  rclk;
input  en;

    assign clk = rclk & en ;

endmodule

module bw_mckbuf_4p5x (clk, rclk, en);
output clk;
input  rclk;
input  en;

    assign clk = rclk & en ;

endmodule

module bw_mckbuf_3x (clk, rclk, en);
output clk;
input  rclk;
input  en;

    assign clk = rclk & en ;

endmodule

module bw_mckbuf_1p5x (clk, rclk, en);
output clk;
input  rclk;
input  en;

    assign clk = rclk & en ;

endmodule

//bw_u1_minbuf_1x
//

module bw_u1_minbuf_1x (
    z,
    a );

    output z;
    input  a;

    assign z = ( a );

endmodule

//bw_u1_minbuf_4x
//

module bw_u1_minbuf_4x (
    z,
    a );

    output z;
    input  a;

    assign z = ( a );

endmodule

//bw_u1_minbuf_5x
//

module bw_u1_minbuf_5x (
    z,
    a );

    output z;
    input  a;

    assign z = ( a );

endmodule

module bw_u1_ckenbuf_4p5x  (clk, rclk, en_l, tm_l);
output clk;
input  rclk, en_l, tm_l;
        zckenbuf_prim i0 ( clk, rclk, en_l, tm_l );
endmodule 

// dummy fill modules to get rid of DFT "CAP" property errors (bug 5487)

module bw_u1_fill_1x(\vdd! );
input \vdd! ;
endmodule

module bw_u1_fill_2x(\vdd! );
input \vdd! ;
endmodule

module bw_u1_fill_3x(\vdd! );
input \vdd! ;
endmodule

module bw_u1_fill_4x(\vdd! );
input \vdd! ;
endmodule
// Modified by Princeton University on June 9th, 2015
// ========== Copyright Header Begin ==========================================
//
// OpenSPARC T1 Processor File: ucb_bus_in.v
// Copyright (c) 2006 Sun Microsystems, Inc.  All Rights Reserved.
// DO NOT ALTER OR REMOVE COPYRIGHT NOTICES.
//
// The above named program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public
// License version 2 as published by the Free Software Foundation.
//
// The above named program is distributed in the hope that it will be
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// General Public License for more details.
//
// You should have received a copy of the GNU General Public
// License along with this work; if not, write to the Free Software
// Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA 02110-1301, USA.
//
// ========== Copyright Header End ============================================
////////////////////////////////////////////////////////////////////////
/*
//  Module Name:	ucb_bus_in (ucb bus inbound interface block)
//  Description:	This interface block is instaniated by the
//                      UCB modules and IO Bridge to receive packets
//                      on the UCB bus.
*/
////////////////////////////////////////////////////////////////////////
// Global header file includes
////////////////////////////////////////////////////////////////////////
// Modified by Princeton University on June 9th, 2015
/*
* ========== Copyright Header Begin ==========================================
* 
* OpenSPARC T1 Processor File: sys.h
* Copyright (c) 2006 Sun Microsystems, Inc.  All Rights Reserved.
* DO NOT ALTER OR REMOVE COPYRIGHT NOTICES.
* 
* The above named program is free software; you can redistribute it and/or
* modify it under the terms of the GNU General Public
* License version 2 as published by the Free Software Foundation.
* 
* The above named program is distributed in the hope that it will be 
* useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
* General Public License for more details.
* 
* You should have received a copy of the GNU General Public
* License along with this work; if not, write to the Free Software
* Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA 02110-1301, USA.
* 
* ========== Copyright Header End ============================================
*/
// -*- verilog -*-
////////////////////////////////////////////////////////////////////////
/*
//
// Description:		Global header file that contain definitions that 
//                      are common/shared at the systme level
*/
////////////////////////////////////////////////////////////////////////
//
// Setting the time scale
// If the timescale changes, JP_TIMESCALE may also have to change.
`timescale	1ps/1ps
`default_nettype wire

//
// Number of threads in a core
// ===========================
//

//`define CONFIG_NUM_THREADS // This must be defined for any of below to work
//`define THREADS_1
//`define THREADS_2
//`define THREADS_3


//
// JBUS clock
// =========
//
// `define SYSCLK_PERIOD   5000


// Afara Link Defines
// ==================

// Reliable Link




// Afara Link Objects


// Afara Link Object Format - Reliable Link










// Afara Link Object Format - Congestion



  







// Afara Link Object Format - Acknowledge











// Afara Link Object Format - Request

















// Afara Link Object Format - Message



// Acknowledge Types




// Request Types





// Afara Link Frame



//
// UCB Packet Type
// ===============
//

















//
// UCB Data Packet Format
// ======================
//






























// Size encoding for the UCB_SIZE_HI/LO field
// 000 - byte
// 001 - half-word
// 010 - word
// 011 - double-word
// 111 - quad-word







//
// UCB Interrupt Packet Format
// ===========================
//










//`define UCB_THR_HI             9      // (6) cpu/thread ID shared with
//`define UCB_THR_LO             4             data packet format
//`define UCB_PKT_HI             3      // (4) packet type shared with
//`define UCB_PKT_LO             0      //     data packet format







//
// FCRAM Bus Widths
// ================
//






//
// ENET clock periods
// ==================
//
// `define AXGRMII_CLK_PERIOD          6400 // 312.5MHz/2
// `define ENET_GMAC_CLK_PERIOD        8000 // 125MHz


//
// JBus Bridge defines
// =================
//
// `define      SYS_UPA_CLK        `SYS.upa_clk
// `define      SYS_J_CLK          `SYS.j_clk
// `define      SYS_P_CLK          `SYS.p_clk
// `define      SYS_G_CLK          `SYS.g_clk
// `define      JP_TIMESCALE       `timescale 1 ps / 1 ps
// `define      PCI_CLK_PERIOD     15152                  //  66 MHz
// `define      UPA_RD_CLK_PERIOD  6666                   // 150 MHz
// `define      UPA_REF_CLK_PERIOD 7576                   // 132 MHz
// `define      ICHIP_CLK_PERIOD   30304                  //  33 MHz


//
// PCI Device Address Configuration
// ================================
//























// system level definition file which contains the
			// time scale definition

////////////////////////////////////////////////////////////////////////
// Local header file includes / local defines
////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////
// Interface signal list declarations
////////////////////////////////////////////////////////////////////////
module ucb_bus_in (/*AUTOARG*/
   // Outputs
   stall, indata_buf_vld, indata_buf,
   // Inputs
   rst_l, clk, vld, data, stall_a1
   );

   // synopsys template

   parameter UCB_BUS_WIDTH = 32;
   parameter REG_WIDTH = 64;


////////////////////////////////////////////////////////////////////////
// Signal declarations
////////////////////////////////////////////////////////////////////////
   // Global interface
   input                     rst_l;
   input 		     clk;


   // UCB bus interface
   input 		     vld;
   input [UCB_BUS_WIDTH-1:0] data;
   output 		     stall;


   // Local interface
   output 		     indata_buf_vld;
   output [REG_WIDTH+63:0]   indata_buf;
   input 		     stall_a1; // would this prevent indata_buf to change?


   // Internal signals
   wire 		     vld_d1;
   wire 		     stall_d1;
   wire [UCB_BUS_WIDTH-1:0]  data_d1;
   wire 		     skid_buf0_en;
   wire 		     vld_buf0;
   wire [UCB_BUS_WIDTH-1:0]  data_buf0;
   wire 		     skid_buf1_en;
   wire 		     vld_buf1;
   wire [UCB_BUS_WIDTH-1:0]  data_buf1;
   wire 		     skid_buf0_sel;
   wire 		     skid_buf1_sel;
   wire 		     vld_mux;
   wire [UCB_BUS_WIDTH-1:0]  data_mux;
   wire [(REG_WIDTH+64)/UCB_BUS_WIDTH-1:0] indata_vec_next;
   wire [(REG_WIDTH+64)/UCB_BUS_WIDTH-1:0] indata_vec;
   wire [REG_WIDTH+63:0]     indata_buf_next;
   wire 		     indata_vec0_d1;


////////////////////////////////////////////////////////////////////////
// Code starts here
////////////////////////////////////////////////////////////////////////
   /************************************************************
    * UCB bus interface flops
    * This is to make signals going between IOB and UCB flop-to-flop
    * to improve timing.
    ************************************************************/
   dffrle_ns #(1) vld_d1_ff (.din(vld),
			     .rst_l(rst_l),
			     .en(~stall_d1),
			     .clk(clk),
			     .q(vld_d1));

   dffe_ns #(UCB_BUS_WIDTH) data_d1_ff (.din(data),
					.en(~stall_d1),
					.clk(clk),
					.q(data_d1));

   dffrl_ns #(1) stall_ff (.din(stall_a1),
			   .clk(clk),
			   .rst_l(rst_l),
			   .q(stall));

   dffrl_ns #(1) stall_d1_ff (.din(stall),
			      .clk(clk),
			      .rst_l(rst_l),
			      .q(stall_d1));


   /************************************************************
    * Skid buffer
    * We need a two deep skid buffer to handle stalling.
    ************************************************************/
   // Assertion: stall has to be deasserted for more than 1 cycle
   //            ie time between two separate stalls has to be
   //            at least two cycles.  Otherwise, contents from
   //            skid buffer will be lost.

   // Buffer 0
   assign 	 skid_buf0_en = stall_a1 & ~stall;

   dffrle_ns #(1) vld_buf0_ff (.din(vld_d1),
			       .rst_l(rst_l),
			       .en(skid_buf0_en),
			       .clk(clk),
			       .q(vld_buf0));

   dffe_ns #(UCB_BUS_WIDTH) data_buf0_ff (.din(data_d1),
					  .en(skid_buf0_en),
					  .clk(clk),
					  .q(data_buf0));

   // Buffer 1
   dffrl_ns #(1) skid_buf1_en_ff (.din(skid_buf0_en),
				  .clk(clk),
				  .rst_l(rst_l),
				  .q(skid_buf1_en));

   dffrle_ns #(1) vld_buf1_ff (.din(vld_d1),
			       .rst_l(rst_l),
			       .en(skid_buf1_en),
			       .clk(clk),
			       .q(vld_buf1));

   dffe_ns #(UCB_BUS_WIDTH) data_buf1_ff (.din(data_d1),
					  .en(skid_buf1_en),
					  .clk(clk),
					  .q(data_buf1));


   /************************************************************
    * Mux between skid buffer and interface flop
    ************************************************************/
   // Assertion: stall has to be deasserted for more than 1 cycle
   //            ie time between two separate stalls has to be
   //            at least two cycles.  Otherwise, contents from
   //            skid buffer will be lost.

   assign 	 skid_buf0_sel = ~stall_a1 & stall;

   dffrl_ns #(1) skid_buf1_sel_ff (.din(skid_buf0_sel),
				   .clk(clk),
				   .rst_l(rst_l),
				   .q(skid_buf1_sel));

   assign 	 vld_mux = skid_buf0_sel ? vld_buf0 :
		           skid_buf1_sel ? vld_buf1 :
		                           vld_d1;

   assign 	 data_mux = skid_buf0_sel ? data_buf0 :
		            skid_buf1_sel ? data_buf1 :
		                            data_d1;


   /************************************************************
    * Assemble inbound data
    ************************************************************/
   // valid vector
   assign 	 indata_vec_next = {vld_mux,
				    indata_vec[(REG_WIDTH+64)/UCB_BUS_WIDTH-1:1]};
   dffrle_ns #((REG_WIDTH+64)/UCB_BUS_WIDTH) indata_vec_ff (.din(indata_vec_next),
							    .en(~stall_a1),
							    .rst_l(rst_l),
							    .clk(clk),
							    .q(indata_vec));

   // data buffer
   assign 	 indata_buf_next = {data_mux,
				    indata_buf[REG_WIDTH+63:UCB_BUS_WIDTH]};
   dffe_ns #(REG_WIDTH+64) indata_buf_ff (.din(indata_buf_next),
					  .en(~stall_a1),
					  .clk(clk),
					  .q(indata_buf));

   // detect a new packet
   dffrle_ns #(1) indata_vec0_d1_ff (.din(indata_vec[0]),
				     .rst_l(rst_l),
				     .en(~stall_a1),
				     .clk(clk),
				     .q(indata_vec0_d1));

   assign        indata_buf_vld = indata_vec[0] & ~indata_vec0_d1;


endmodule // ucb_bus_in
// Modified by Princeton University on June 9th, 2015
// ========== Copyright Header Begin ==========================================
//
// OpenSPARC T1 Processor File: ucb_bus_out.v
// Copyright (c) 2006 Sun Microsystems, Inc.  All Rights Reserved.
// DO NOT ALTER OR REMOVE COPYRIGHT NOTICES.
//
// The above named program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public
// License version 2 as published by the Free Software Foundation.
//
// The above named program is distributed in the hope that it will be
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// General Public License for more details.
//
// You should have received a copy of the GNU General Public
// License along with this work; if not, write to the Free Software
// Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA 02110-1301, USA.
//
// ========== Copyright Header End ============================================
////////////////////////////////////////////////////////////////////////
/*
//  Module Name:        ucb_bus_out (ucb bus outbound interface block)
//	Description:	This interface block is instantiated by the
//                      UCB modules and IO Bridge to transmit packets
//                      on the UCB bus.
*/
////////////////////////////////////////////////////////////////////////
// Global header file includes
////////////////////////////////////////////////////////////////////////
// Modified by Princeton University on June 9th, 2015
/*
* ========== Copyright Header Begin ==========================================
* 
* OpenSPARC T1 Processor File: sys.h
* Copyright (c) 2006 Sun Microsystems, Inc.  All Rights Reserved.
* DO NOT ALTER OR REMOVE COPYRIGHT NOTICES.
* 
* The above named program is free software; you can redistribute it and/or
* modify it under the terms of the GNU General Public
* License version 2 as published by the Free Software Foundation.
* 
* The above named program is distributed in the hope that it will be 
* useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
* General Public License for more details.
* 
* You should have received a copy of the GNU General Public
* License along with this work; if not, write to the Free Software
* Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA 02110-1301, USA.
* 
* ========== Copyright Header End ============================================
*/
// -*- verilog -*-
////////////////////////////////////////////////////////////////////////
/*
//
// Description:		Global header file that contain definitions that 
//                      are common/shared at the systme level
*/
////////////////////////////////////////////////////////////////////////
//
// Setting the time scale
// If the timescale changes, JP_TIMESCALE may also have to change.
`timescale	1ps/1ps
`default_nettype wire

//
// Number of threads in a core
// ===========================
//

//`define CONFIG_NUM_THREADS // This must be defined for any of below to work
//`define THREADS_1
//`define THREADS_2
//`define THREADS_3


//
// JBUS clock
// =========
//
// `define SYSCLK_PERIOD   5000


// Afara Link Defines
// ==================

// Reliable Link




// Afara Link Objects


// Afara Link Object Format - Reliable Link










// Afara Link Object Format - Congestion



  







// Afara Link Object Format - Acknowledge











// Afara Link Object Format - Request

















// Afara Link Object Format - Message



// Acknowledge Types




// Request Types





// Afara Link Frame



//
// UCB Packet Type
// ===============
//

















//
// UCB Data Packet Format
// ======================
//






























// Size encoding for the UCB_SIZE_HI/LO field
// 000 - byte
// 001 - half-word
// 010 - word
// 011 - double-word
// 111 - quad-word







//
// UCB Interrupt Packet Format
// ===========================
//










//`define UCB_THR_HI             9      // (6) cpu/thread ID shared with
//`define UCB_THR_LO             4             data packet format
//`define UCB_PKT_HI             3      // (4) packet type shared with
//`define UCB_PKT_LO             0      //     data packet format







//
// FCRAM Bus Widths
// ================
//






//
// ENET clock periods
// ==================
//
// `define AXGRMII_CLK_PERIOD          6400 // 312.5MHz/2
// `define ENET_GMAC_CLK_PERIOD        8000 // 125MHz


//
// JBus Bridge defines
// =================
//
// `define      SYS_UPA_CLK        `SYS.upa_clk
// `define      SYS_J_CLK          `SYS.j_clk
// `define      SYS_P_CLK          `SYS.p_clk
// `define      SYS_G_CLK          `SYS.g_clk
// `define      JP_TIMESCALE       `timescale 1 ps / 1 ps
// `define      PCI_CLK_PERIOD     15152                  //  66 MHz
// `define      UPA_RD_CLK_PERIOD  6666                   // 150 MHz
// `define      UPA_REF_CLK_PERIOD 7576                   // 132 MHz
// `define      ICHIP_CLK_PERIOD   30304                  //  33 MHz


//
// PCI Device Address Configuration
// ================================
//























// system level definition file which
                        // contains the time scale definition

////////////////////////////////////////////////////////////////////////
// Local header file includes / local defines
////////////////////////////////////////////////////////////////////////

module ucb_bus_out (/*AUTOARG*/
   // Outputs
   vld, data, outdata_buf_busy,
   // Inputs
   clk, rst_l, stall, outdata_buf_in, outdata_vec_in, outdata_buf_wr
   );

   // synopsys template

   parameter UCB_BUS_WIDTH = 32;
   parameter REG_WIDTH = 64;            // maximum data bits that needs to
                                        // be sent.  Set to 64 or 128

   // Globals
   input                                clk;
   input 				rst_l;


   // UCB bus interface
   output 				vld;
   output [UCB_BUS_WIDTH-1:0] 		data;
   input 				stall;


   // Local interface
   output 				outdata_buf_busy;  // busy outputting, can't accept data into buffer
   input [REG_WIDTH+63:0] 		outdata_buf_in;
   input [(REG_WIDTH+64)/UCB_BUS_WIDTH-1:0] outdata_vec_in; // indicating how much data to send
   input 				outdata_buf_wr;


   // Local signals
   wire 				stall_d1;
   wire [(REG_WIDTH+64)/UCB_BUS_WIDTH-1:0] 	outdata_vec;
   wire [(REG_WIDTH+64)/UCB_BUS_WIDTH-1:0] 	outdata_vec_next;
   wire [REG_WIDTH+63:0] 		outdata_buf;
   reg [REG_WIDTH+63:0] 		outdata_buf_next;
   wire 				load_outdata;
   wire 				shift_outdata;


////////////////////////////////////////////////////////////////////////
// Code starts here
////////////////////////////////////////////////////////////////////////
   /************************************************************
    * UCB bus interface flops
    ************************************************************/
   assign 	 vld = outdata_vec[0];
   // assign 	 data = vld ? outdata_buf[UCB_BUS_WIDTH-1:0] : `UCB_BUS_WIDTH'b0;
   assign    data = outdata_buf[UCB_BUS_WIDTH-1:0];

   dffrl_ns #(1) stall_d1_ff (.din(stall),
                              .clk(clk),
                              .rst_l(rst_l),
                              .q(stall_d1));


   /************************************************************
    * Outbound Data
    ************************************************************/
   // accept new data only if there is none being processed
   assign 	 load_outdata = outdata_buf_wr & ~outdata_buf_busy;

   assign 	 outdata_buf_busy = outdata_vec[0] | stall_d1;

   // only shifts when then input vector is a straight valids
   assign 	 shift_outdata = outdata_vec[0] & ~stall_d1;

   assign 	 outdata_vec_next =
		 load_outdata  ? outdata_vec_in:
		 shift_outdata ? outdata_vec >> 1:
	                         outdata_vec;
   dffrl_ns #((REG_WIDTH+64)/UCB_BUS_WIDTH) outdata_vec_ff (.din(outdata_vec_next),
							    .clk(clk),
							    .rst_l(rst_l),
							    .q(outdata_vec));

   // assign 	 outdata_buf_next =
		 // load_outdata  ? outdata_buf_in:
		 // shift_outdata ? (outdata_buf >> UCB_BUS_WIDTH):
	  //                        outdata_buf;
   always @ *
   begin
      if (load_outdata)
         outdata_buf_next = outdata_buf_in;
      else if (shift_outdata)
      begin
         outdata_buf_next = outdata_buf >> UCB_BUS_WIDTH;
         if (outdata_vec[1] == 1'b0)
            outdata_buf_next[UCB_BUS_WIDTH-1:0] = 0;
      end
      else
         outdata_buf_next = outdata_buf; // no shifting
   end

   dff_ns #(REG_WIDTH+64) outdata_buf_ff (.din(outdata_buf_next),
					  .clk(clk),
					  .q(outdata_buf));


endmodule // ucb_bus_out






// Copyright (c) 2015 Princeton University
// All rights reserved.
// 
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of Princeton University nor the
//       names of its contributors may be used to endorse or promote products
//       derived from this software without specific prior written permission.
// 
// THIS SOFTWARE IS PROVIDED BY PRINCETON UNIVERSITY "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL PRINCETON UNIVERSITY BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

//File: valrdy_to_credit.v (modified from space_avail_top.v)
//
//Modified: Yaosheng Fu
//May 2, 2014
//
//Function: This module keeps track of how many spots are free in the NIB that
//	we are sending to
//
//State: count_f, yummy_out_f, valid_in_f
//
//Instantiates: 
//

module valrdy_to_credit (
            clk,
            reset,
                
            //val/rdy interface
            data_in,
            valid_in,
            ready_in,

			//credit based interface	
            data_out,
            valid_out,
		    yummy_out);

parameter BUFFER_SIZE = 4;
parameter BUFFER_BITS = 3;
   
input clk;
input reset;

 
input [64-1:0]	 data_in;
 input valid_in;			// sending data to the output
 input yummy_out;			// output consumed data

output [64-1:0]  data_out;
 output valid_out;
 output ready_in;		// is there space available?


//This is the state
 reg yummy_out_f;
 reg valid_temp_f;
 reg [BUFFER_BITS-1:0] count_f;

reg is_one_f;
 reg is_two_or_more_f;

//wires
 wire [BUFFER_BITS-1:0] count_plus_1;
 wire [BUFFER_BITS-1:0] count_minus_1;
 wire up;
 wire down;

 wire valid_temp;

//wire regs
  reg [BUFFER_BITS-1:0] count_temp;


//assigns
assign data_out = data_in;
assign valid_temp = valid_in & ready_in;
assign valid_out = valid_temp;

assign count_plus_1 = count_f + 1'b1;
assign count_minus_1 = count_f - 1'b1;
assign ready_in = is_two_or_more_f;
assign up = yummy_out_f & ~valid_temp_f;
assign down = ~yummy_out_f & valid_temp_f;

always @ (count_f or count_plus_1 or count_minus_1 or up or down)
begin
	case (count_f)
	0:
		begin
			if(up)
			begin
				count_temp <= count_plus_1;
			end
			else
			begin
				count_temp <= count_f;
			end
		end
	BUFFER_SIZE:
		begin
			if(down)
			begin
				count_temp <= count_minus_1;
			end
			else
			begin
				count_temp <= count_f;
			end
		end
	default:
		begin
			case ({up, down})
				2'b10:	count_temp <= count_plus_1;
				2'b01:	count_temp <= count_minus_1;
				default:	count_temp <= count_f;
			endcase
		end
	endcase
end

//wire top_bits_zero_temp = ~| count_temp[BUFFER_BITS-1:1];
 wire top_bits_zero_temp = count_temp < 3 ? 1 : 0;

always @ (posedge clk)
begin
	if(reset)
	begin
	   count_f <= BUFFER_SIZE;
	   yummy_out_f <= 1'b0;
	   valid_temp_f <= 1'b0;
	   is_one_f <= (BUFFER_SIZE == 1);
	   is_two_or_more_f <= (BUFFER_SIZE >= 2);
	end
	else
	begin
	   count_f <= count_temp;
	   yummy_out_f <= yummy_out;
	   valid_temp_f <= valid_temp;
	   is_one_f         <= top_bits_zero_temp & count_temp[0];
   	   is_two_or_more_f <= ~top_bits_zero_temp;
	end
end

endmodule
      
/*
Copyright (c) 2015 Princeton University
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of Princeton University nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY PRINCETON UNIVERSITY "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL PRINCETON UNIVERSITY BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

/*
Copyright (c) 2015 Princeton University
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of Princeton University nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY PRINCETON UNIVERSITY "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL PRINCETON UNIVERSITY BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

//l15.h
// Copyright (c) 2015 Princeton University
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of Princeton University nor the
//       names of its contributors may be used to endorse or promote products
//       derived from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY PRINCETON UNIVERSITY "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL PRINCETON UNIVERSITY BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

//==================================================================================================
//  Filename      : define.h
//  Created On    : 2014-02-20
//  Last Modified : 2018-11-16 17:14:11
//  Revision      :
//  Author        : Yaosheng Fu
//  Company       : Princeton University
//  Email         : yfu@princeton.edu
//
//  Description   : main header file defining global architecture parameters
//
//
//==================================================================================================




// Uncomment to define USE_GENERIC_SRAM_IMPLEMENTATION to use the old unsynthesizable BRAM
// `define USE_GENERIC_SRAM_IMPLEMENTATION




/*
Copyright (c) 2015 Princeton University
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of Princeton University nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY PRINCETON UNIVERSITY "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL PRINCETON UNIVERSITY BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

/////////////////////////////////////////////////////////////////////////////////////////////
// 63         50 49      42 41      34 33           30 29      22 21                 0   
// ------------------------------------------------------------------------------------
// |            |          |          |               |          |                    |
// |  Chip ID   |  Dest X  |  Dest Y  |  Final Route  |  Length  |    Header Payload  | 
// |            |          |          |               |          |                    |
// ------------------------------------------------------------------------------------
////////////////////////////////////////////////////////////////////////////////////////////////











 //whether the routing is based on chipid or x y position
 //`define    ROUTING_CHIP_ID
 

 //defines for different topology, only one should be active
 //`define    NETWORK_TOPO_2D_MESH
 //`define    NETWORK_TOPO_3D_MESH
 

// Tile config

// devices.xml





// NoC interface





















// NodeID decomposition








//========================
//Packet format
//=========================

//Header decomposition































// these shifted fields are added for convienience
// HEADER 2








// HEADER 3








//NoC header information










// Width of MSG_ADDR field - you're probably looking for PHY_ADDR_WIDTH


//Coherence information





//Requests from L15 to L2
// Should always make #0 an error








//condition satisfied

//condition not satisfied

//Both SWAP and LDSTUB are the same for L2









//RISC-V AMO requests









//RISC-V AMO L2-internal phase 1









//RISC-V AMO L2-internal phase 2












//Forward requests from L2 to L15







//Memory requests from L2 to DRAM






//Forward acks from L15 to L2







//Memory acks from memory to L2









//Acks from L2 to L15


//TODO



//Only exist within L2





//`define MSG_TYPE_LOAD_REQ           8'd31 if this is enabled, don't use 31





// These should be defined in l2.vh, not the global defines











//Physical address










//Transition data size












//`define HOME_ID_MASK_X          10:10
//Additional fields for Sharer Domain ID and Logical Sharer ID
//For coherence domain restriction only


// Tri: dynamically adjust these parameters based on how many tiles are available
//  Assumption: 8x8 topology























































//`define DMBR_TAG_WIDTH 4

//Clumpy Shared Memory






////////////////////////////////////////////
// SOME CONFIGURATION REGISTERS DEFINES
////////////////////////////////////////////
// example: read/write to csm_en would be 0xba_0000_0100

// `define ASI_ADDRESS_MASK    `L15_ADDR_TYPE
// `define CONFIG_ASI_ADDRESS  `L15_ADDR_TYPE_WIDTH'hba










// DMBR Config register 1 fields















// DMBR Config register 2 fields



//Home allocation method






//Additional fields for Sharer Domain ID and Logical Sharer ID
//For coherence domain restriction only

































//`define TTE_CSM_WIDTH           64
//`define TTE_CSM                 63:0
//`define TTE_CSM_VALID           63
//`define TTE_CSM_SZL             62:61
//`define TTE_CSM_NFO             60
//`define TTE_CSM_IE              59
//`define TTE_CSM_SOFT2           58:49
//`define TTE_CSM_SZH             48
//`define TTE_CSM_DIAG            47:40
//`define TTE_CSM_RES1            39
//`define TTE_CSM_SDID            38:29
//`define TTE_CSM_HDID            28:19
//`define TTE_CSM_LSID            18:13
//`define TTE_CSM_SOFT            12:8
//`define TTE_CSM_RES2            7
//`define TTE_CSM_LOCK            6
//`define TTE_CSM_CP              5
//`define TTE_CSM_CV              4
//`define TTE_CSM_E               3
//`define TTE_CSM_P               2
//`define TTE_CSM_W               1
//`define TTE_CSM_RES3            0












// Packet format for home id





/////////////////////////////////////
// BIST
/////////////////////////////////////

// the data width from tap to individual sram wrappers



//deprecated































/////////////////////////////////////
// IDs for JTAG-Core interface
/////////////////////////////////////

// 48b for writing the PC reset vector

// 94b for reading the sscan data











// Execution Drafting Synchronization Method Values





// Execution Drafting timeout counter bit width


// Configuration registers












// Execution Drafting configuration register bit positions








// Execution Drafting configuration register default values
// ED disabled, STSM sync method, LFSR seed = 16'b0, LFSR load = 1'b0,
// Counter Timeout = 16'd32



//Clumpy sharer memory configuration registers
























// Modified by Princeton University on June 9th, 2015
/*
* ========== Copyright Header Begin ==========================================
* 
* OpenSPARC T1 Processor File: iop.h
* Copyright (c) 2006 Sun Microsystems, Inc.  All Rights Reserved.
* DO NOT ALTER OR REMOVE COPYRIGHT NOTICES.
* 
* The above named program is free software; you can redistribute it and/or
* modify it under the terms of the GNU General Public
* License version 2 as published by the Free Software Foundation.
* 
* The above named program is distributed in the hope that it will be 
* useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
* General Public License for more details.
* 
* You should have received a copy of the GNU General Public
* License along with this work; if not, write to the Free Software
* Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA 02110-1301, USA.
* 
* ========== Copyright Header End ============================================
*/
//-*- verilog -*-
////////////////////////////////////////////////////////////////////////
/*
//
//  Description:	Global header file that contain definitions that 
//                      are common/shared at the IOP chip level
*/
////////////////////////////////////////////////////////////////////////


// Address Map Defines
// ===================




// CMP space



// IOP space




                               //`define ENET_ING_CSR     8'h84
                               //`define ENET_EGR_CMD_CSR 8'h85















// L2 space



// More IOP space





//Cache Crossbar Width and Field Defines
//======================================













































//bits 133:128 are shared by different fields
//for different packet types.
























//`define CPX_INV_PA_HI   116
//`define CPX_INV_PA_LO   112






// cache invalidation format
// `define CPX_INV_DCACHE_WORD0_VAL 0
// `define CPX_INV_ICACHE_WORD0_VAL 1
// `define CPX_INV_WORD0_WAY 5:2
// `define CPX_INV_DCACHE_WORD0_VAL 6
// `define CPX_INV_ICACHE_WORD0_VAL 7
// `define CPX_INV_WORD0_WAY 11:8
// `define CPX_INV_DCACHE_WORD0_VAL 12
// // `define CPX_INV_ICACHE_WORD0_VAL 13
// `define CPX_INV_WORD0_WAY 17:14
// `define CPX_INV_DCACHE_WORD0_VAL 18
// // `define CPX_INV_ICACHE_WORD0_VAL 19
// `define CPX_INV_WORD0_WAY 23:20




// 4 extra bits for bigger icache/dcache
// up to 512KB l1 icache, 256KB l1 dcache

































//Pico defines













//End cache crossbar defines


// Number of COS supported by EECU 



// 
// BSC bus sizes
// =============
//

// General




// CTags













// reinstated temporarily




// CoS






// L2$ Bank



// L2$ Req













// L2$ Ack








// Enet Egress Command Unit














// Enet Egress Packet Unit













// This is cleaved in between Egress Datapath Ack's








// Enet Egress Datapath
















// In-Order / Ordered Queue: EEPU
// Tag is: TLEN, SOF, EOF, QID = 15






// Nack + Tag Info + CTag




// ENET Ingress Queue Management Req












// ENET Ingress Queue Management Ack








// Enet Ingress Packet Unit












// ENET Ingress Packet Unit Ack







// In-Order / Ordered Queue: PCI
// Tag is: CTAG





// PCI-X Request











// PCI_X Acknowledge











//
// BSC array sizes
//================
//












// ECC syndrome bits per memory element




//
// BSC Port Definitions
// ====================
//
// Bits 7 to 4 of curr_port_id








// Number of ports of each type


// Bits needed to represent above


// How wide the linked list pointers are
// 60b for no payload (2CoS)
// 80b for payload (2CoS)

//`define BSC_OBJ_PTR   80
//`define BSC_HD1_HI    69
//`define BSC_HD1_LO    60
//`define BSC_TL1_HI    59
//`define BSC_TL1_LO    50
//`define BSC_CT1_HI    49
//`define BSC_CT1_LO    40
//`define BSC_HD0_HI    29
//`define BSC_HD0_LO    20
//`define BSC_TL0_HI    19
//`define BSC_TL0_LO    10
//`define BSC_CT0_HI     9
//`define BSC_CT0_LO     0


































// I2C STATES in DRAMctl







//
// IOB defines
// ===========
//



















//`define IOB_INT_STAT_WIDTH   32
//`define IOB_INT_STAT_HI      31
//`define IOB_INT_STAT_LO       0

















































// fixme - double check address mapping
// CREG in `IOB_INT_CSR space










// CREG in `IOB_MAN_CSR space





































// Address map for TAP access of SPARC ASI













//
// CIOP UCB Bus Width
// ==================
//
//`define IOB_EECU_WIDTH       16  // ethernet egress command
//`define EECU_IOB_WIDTH       16

//`define IOB_NRAM_WIDTH       16  // NRAM (RLDRAM previously)
//`define NRAM_IOB_WIDTH        4




//`define IOB_ENET_ING_WIDTH   32  // ethernet ingress
//`define ENET_ING_IOB_WIDTH    8

//`define IOB_ENET_EGR_WIDTH    4  // ethernet egress
//`define ENET_EGR_IOB_WIDTH    4

//`define IOB_ENET_MAC_WIDTH    4  // ethernet MAC
//`define ENET_MAC_IOB_WIDTH    4




//`define IOB_BSC_WIDTH         4  // BSC
//`define BSC_IOB_WIDTH         4







//`define IOB_CLSP_WIDTH        4  // clk spine unit
//`define CLSP_IOB_WIDTH        4





//
// CIOP UCB Buf ID Type
// ====================
//



//
// Interrupt Device ID
// ===================
//
// Caution: DUMMY_DEV_ID has to be 9 bit wide
//          for fields to line up properly in the IOB.



//
// Soft Error related definitions 
// ==============================
//



//
// CMP clock
// =========
//




//
// NRAM/IO Interface
// =================
//










//
// NRAM/ENET Interface
// ===================
//







//
// IO/FCRAM Interface
// ==================
//






//
// PCI Interface
// ==================
// Load/store size encodings
// -------------------------
// Size encoding
// 000 - byte
// 001 - half-word
// 010 - word
// 011 - double-word
// 100 - quad






//
// JBI<->SCTAG Interface
// =======================
// Outbound Header Format



























// Inbound Header Format




















//
// JBI->IOB Mondo Header Format
// ============================
//














// JBI->IOB Mondo Bus Width/Cycle
// ==============================
// Cycle  1 Header[15:8]
// Cycle  2 Header[ 7:0]
// Cycle  3 J_AD[127:120]
// Cycle  4 J_AD[119:112]
// .....
// Cycle 18 J_AD[  7:  0]






// `define L15_CACHELINE_WIDTH 128

















// devices.xml
// this is used in the ariane SV packages to derive the parameterization

















































// 7

// 7




// 16B cache lines

// 10




// 40 - 4 (16B line) - 7 (index width) = 29

// 11

// 39


// this need to be defined when L1.5 has more sets than L1D
// for correct operations
// `define L15_WMT_EXTENDED_ALIAS


















// `define L15_WMT_ENTRY_0_MASK 1*`L15_WMT_ENTRY_WIDTH-1 -: `L15_WMT_ENTRY_WIDTH
// `define L15_WMT_ENTRY_1_MASK 2*`L15_WMT_ENTRY_WIDTH-1 -: `L15_WMT_ENTRY_WIDTH
// `define L15_WMT_ENTRY_2_MASK 3*`L15_WMT_ENTRY_WIDTH-1 -: `L15_WMT_ENTRY_WIDTH
// `define L15_WMT_ENTRY_3_MASK 4*`L15_WMT_ENTRY_WIDTH-1 -: `L15_WMT_ENTRY_WIDTH
// `define L15_WMT_ENTRY_0_VALID_MASK 1*`L15_WMT_ENTRY_WIDTH-1
// `define L15_WMT_ENTRY_1_VALID_MASK 2*`L15_WMT_ENTRY_WIDTH-1
// `define L15_WMT_ENTRY_2_VALID_MASK 3*`L15_WMT_ENTRY_WIDTH-1
// `define L15_WMT_ENTRY_3_VALID_MASK 4*`L15_WMT_ENTRY_WIDTH-1


  
  


  
  







// LRU array storage
// keeps 6 bits per cache set: 4 "used" bits, 1 each cache line, and 2 bits for wayid round robin (4w)





// source





// MSHR






// controls how many mshr there are
// `define L15_MSHR_COUNT 10
// should be more than the count above when 2^n











// pipeline OPs




// `define L15_MSHR_ALLOCATE_TYPE_WIDTH 2
// `define L15_MSHR_ALLOCATE_TYPE_LD 2'd1
// `define L15_MSHR_ALLOCATE_TYPE_ST 2'd2
// `define L15_MSHR_ALLOCATE_TYPE_IFILL 2'd3









































































































// `define PCX_REQ_SIZE_WIDTH 4


























// define the width of the flattened, native L15 interface, used for ARIANE_RV64 option



























// [L15_DTAG_OP_WIDTH-1:0]































//`define L15_S3_MESI_WRITE_TAGCHECK_WAY_M_IF_LRSC_SET 3'd7






























































// `define L15_NOC1_DUMMY_GEN_NOC1_CREDIT 5'd11










































// L2 shared states


// `define L15_NOC2_ACK_STATE_WIDTH 2
// `define L15_NOC2_ACK_STATE_S 2'd1
// `define L15_NOC2_ACK_STATE_E 2'd2
// `define L15_NOC2_ACK_STATE_M 2'd3

// `define L15_NOC1_REQTYPE_WIDTH `MSG_TYPE_WIDTH
// `define L15_NOC1_REQTYPE_WRITEBACK_GUARD `MSG_TYPE_WBGUARD_REQ
// `define L15_NOC1_REQTYPE_LD_REQUEST `MSG_TYPE_LOAD_REQ
// `define L15_NOC1_REQTYPE_LD_PREFETCH_REQUEST `MSG_TYPE_PREFETCH_REQ
// `define L15_NOC1_REQTYPE_LD_NC_REQUEST `MSG_TYPE_NC_LOAD_REQ
// `define L15_NOC1_REQTYPE_IFILL_REQUEST `MSG_TYPE_LOAD_REQ
// // `define L15_NOC1_REQTYPE_WRITETHROUGH_REQUEST 6'd4
// `define L15_NOC1_REQTYPE_ST_REQUEST `MSG_TYPE_STORE_REQ
// `define L15_NOC1_REQTYPE_BLK_ST_REQUEST `MSG_TYPE_BLK_STORE_REQ
// `define L15_NOC1_REQTYPE_BLK_ST_INIT_REQUEST `MSG_TYPE_BLKINIT_STORE_REQ
// // `define L15_NOC1_REQTYPE__REQUEST
// // `define L15_NOC1_REQTYPE__REQUEST
// // `define L15_NOC1_REQTYPE_ST_UPGRADE_REQUEST 6
// // `define L15_NOC1_REQTYPE_ST_FILL_REQUEST 6'd6
// `define L15_NOC1_REQTYPE_CAS_REQUEST `MSG_TYPE_CAS_REQ
// `define L15_NOC1_REQTYPE_SWAP_REQUEST `MSG_TYPE_SWAP_REQ

// `define L15_NOC3_REQTYPE_WIDTH `MSG_TYPE_WIDTH
// `define L15_NOC3_REQTYPE_WRITEBACK `MSG_TYPE_WB_REQ
// `define L15_NOC3_REQTYPE_NO_DATA_FWD_ACK `MSG_TYPE_NODATA_ACK
// `define L15_NOC3_REQTYPE_DATA_FWD_ACK `MSG_TYPE_DATA_ACK

// `define L2_REQTYPE_WIDTH `MSG_TYPE_WIDTH
// `define L2_REQTYPE_INVALIDATE 6'd1
// `define L2_REQTYPE_DOWNGRADE 6'd2
// `define L2_REQTYPE_ACKDT_LD_NC 6'd3
// `define L2_REQTYPE_ACKDT_IFILL 6'd4
// `define L2_REQTYPE_ACKDT_LD 6'd5
// `define L2_REQTYPE_ACKDT_ST_IM 6'd6
// `define L2_REQTYPE_ACKDT_ST_SM 6'd7
// `define L2_REQTYPE_ACK_WRITETHROUGH 6'd8
// `define L2_REQTYPE_ACK_ATOMIC 6'd9






































// NOC1 ENCODER











// NOC3 ENCODER










// NOC2 BUFFER





// DMBR
// put this here for now, should be moved to a more appropriate location


// NOC1 CREDIT MANAGEMENT
// becareful, the noc1buffer module assumes these are power of two
// also, please change the corresponding pyv value in noc1buffer.v.pyv























// `define L15_NOC1BUFFER_BLKSTORE_LO  `L15_NOC1BUFFER_PREFETCH_HI + 1
// `define L15_NOC1BUFFER_BLKSTORE_HI  `L15_NOC1BUFFER_BLKSTORE_LO + 1 - 1
// `define L15_NOC1BUFFER_BLKINITSTORE_LO  `L15_NOC1BUFFER_BLKSTORE_HI + 1
// `define L15_NOC1BUFFER_BLKINITSTORE_HI  `L15_NOC1BUFFER_BLKINITSTORE_LO + 1 - 1
// `define L15_NOC1BUFFER_DATA_INDEX_LO  `L15_NOC1BUFFER_BLKINITSTORE_HI + 1
































// `define L15_CPUID_ADDRESS 40'h9800000900






////////////////
// CSM
////////////////



// Tri: save space on l15 sram






















//HMC array
































//Special addresses for HMC


//`define L15_ADDR_TYPE_TAG_ACCESS     8'hb4 // later
//`define L15_ADDR_TYPE_STATE_ACCESS   8'hb6 // later
//`define L15_ADDR_TYPE_DIR_ACCESS     8'hb1 // later
//`define L15_ADDR_TYPE_CTRL_REG       8'hb9
//`define L15_ADDR_TYPE_DIS_FLUSH      8'hbc, 8'hbd, 8'hbe, 8'hbf
// `define L15_ADDR_TYPE_HMT_BASE_REG      8'hb7


















// Copyright (c) 2015 Princeton University
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of Princeton University nor the
//       names of its contributors may be used to endorse or promote products
//       derived from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY PRINCETON UNIVERSITY "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL PRINCETON UNIVERSITY BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

//==================================================================================================
//  Filename      : define.h
//  Created On    : 2014-02-20
//  Last Modified : 2018-11-16 17:14:11
//  Revision      :
//  Author        : Yaosheng Fu
//  Company       : Princeton University
//  Email         : yfu@princeton.edu
//
//  Description   : main header file defining global architecture parameters
//
//
//==================================================================================================






































































































































































































































































































































































































































































































































































































// devices.xml



module flat_id_to_xy(
    input  [(6-1):0] flat_id,
    output reg [(8-1):0] x_coord,
    output reg [(8-1):0] y_coord
);

    always @*
    begin
        case (flat_id)
        
//(`NOC_Y_WIDTH+`NOC_X_WIDTH)'d0: 
6'd0: 
begin
    x_coord = 3'd0;
    y_coord = 3'd0;
end

        default:
        begin
            x_coord = 3'dX;
            y_coord = 3'dX;
        end
        endcase
    end
endmodule
// Copyright (c) 2015 Princeton University
// All rights reserved.
// 
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of Princeton University nor the
//       names of its contributors may be used to endorse or promote products
//       derived from this software without specific prior written permission.
// 
// THIS SOFTWARE IS PROVIDED BY PRINCETON UNIVERSITY "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL PRINCETON UNIVERSITY BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

// Wraps the EXU to tie unused signals when no scan chain is present                                              

// Copyright (c) 2015 Princeton University
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of Princeton University nor the
//       names of its contributors may be used to endorse or promote products
//       derived from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY PRINCETON UNIVERSITY "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL PRINCETON UNIVERSITY BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

//==================================================================================================
//  Filename      : define.h
//  Created On    : 2014-02-20
//  Last Modified : 2018-11-16 17:14:11
//  Revision      :
//  Author        : Yaosheng Fu
//  Company       : Princeton University
//  Email         : yfu@princeton.edu
//
//  Description   : main header file defining global architecture parameters
//
//
//==================================================================================================






































































































































































































































































































































































































































































































































































































module bw_r_irf_wrap
(
    input           rclk,
    input           reset_l,
    input           sehold,
    input           rst_tri_en,
    input           ifu_exu_tid_s2,
    input   [4:0]   ifu_exu_rs1_s,
    input   [4:0]   ifu_exu_rs2_s,
    input   [4:0]   ifu_exu_rs3_s,
    input           ifu_exu_ren1_s,
    input           ifu_exu_ren2_s,
    input           ifu_exu_ren3_s,
    input           ecl_irf_wen_w,
    input           ecl_irf_wen_w2,
    input   [4:0]   ecl_irf_rd_m,
    input   [4:0]   ecl_irf_rd_g,
    input   [71:0]  byp_irf_rd_data_w,
    input   [71:0]  byp_irf_rd_data_w2,
    input   [1:0]   ecl_irf_tid_m,
    input   [1:0]   ecl_irf_tid_g,
    input   [2:0]   rml_irf_old_lo_cwp_e,
    input   [2:0]   rml_irf_new_lo_cwp_e,
    input   [2:1]   rml_irf_old_e_cwp_e,
    input   [2:1]   rml_irf_new_e_cwp_e,
    input           rml_irf_swap_even_e,
    input           rml_irf_swap_odd_e,
    input           rml_irf_swap_local_e,
    input           rml_irf_kill_restore_w,
    input           rml_irf_cwpswap_tid_e,
    input   [1:0]   rml_irf_old_agp,
    input   [1:0]   rml_irf_new_agp,
    input           rml_irf_swap_global,
    input           rml_irf_global_tid,

    output  [71:0]  irf_byp_rs1_data_d_l,
    output  [71:0]  irf_byp_rs2_data_d_l,
    output  [71:0]  irf_byp_rs3_data_d_l,
    output  [31:0]  irf_byp_rs3h_data_d_l,

    output [94-1:0] core_rtap_data,
    input wire rtap_core_val,
    input wire rtap_core_threadid,
    input wire [4-1:0]  rtap_core_id,
    input wire [4:0] rtap_core_data
);

bw_r_irf irf
(
    .rclk (rclk),
    .reset_l (reset_l),
    .si (1'bx),
    .se (1'b0),
    .sehold (sehold),
    .rst_tri_en (rst_tri_en),
    .ifu_exu_tid_s2 ({1'bx, ifu_exu_tid_s2}),
    .ifu_exu_rs1_s (ifu_exu_rs1_s),
    .ifu_exu_rs2_s (ifu_exu_rs2_s),
    .ifu_exu_rs3_s (ifu_exu_rs3_s),
    .ifu_exu_ren1_s (ifu_exu_ren1_s),
    .ifu_exu_ren2_s (ifu_exu_ren2_s),
    .ifu_exu_ren3_s (ifu_exu_ren3_s),
    .ecl_irf_wen_w (ecl_irf_wen_w),
    .ecl_irf_wen_w2 (ecl_irf_wen_w2),
    .ecl_irf_rd_m (ecl_irf_rd_m),
    .ecl_irf_rd_g (ecl_irf_rd_g),
    .byp_irf_rd_data_w (byp_irf_rd_data_w),
    .byp_irf_rd_data_w2 (byp_irf_rd_data_w2),
    .ecl_irf_tid_m (ecl_irf_tid_m),
    .ecl_irf_tid_g (ecl_irf_tid_g),
    .rml_irf_old_lo_cwp_e (rml_irf_old_lo_cwp_e),
    .rml_irf_new_lo_cwp_e (rml_irf_new_lo_cwp_e),
    .rml_irf_old_e_cwp_e (rml_irf_old_e_cwp_e),
    .rml_irf_new_e_cwp_e (rml_irf_new_e_cwp_e),
    .rml_irf_swap_even_e (rml_irf_swap_even_e),
    .rml_irf_swap_odd_e (rml_irf_swap_odd_e),
    .rml_irf_swap_local_e (rml_irf_swap_local_e),
    .rml_irf_kill_restore_w (rml_irf_kill_restore_w),
    .rml_irf_cwpswap_tid_e ({1'bx, rml_irf_cwpswap_tid_e}),
    .rml_irf_old_agp (rml_irf_old_agp),
    .rml_irf_new_agp (rml_irf_new_agp),
    .rml_irf_swap_global (rml_irf_swap_global),
    .rml_irf_global_tid ({1'bx, rml_irf_global_tid}),
    .so (),
    .irf_byp_rs1_data_d_l (irf_byp_rs1_data_d_l),
    .irf_byp_rs2_data_d_l (irf_byp_rs2_data_d_l),
    .irf_byp_rs3_data_d_l (irf_byp_rs3_data_d_l),
    .irf_byp_rs3h_data_d_l (irf_byp_rs3h_data_d_l)
    
    
    
    ,
    .core_rtap_data (core_rtap_data),
    .rtap_core_val (rtap_core_val),
    .rtap_core_threadid ({1'bx, rtap_core_threadid}),
    .rtap_core_id (rtap_core_id),
    .rtap_core_data ({89'bx, rtap_core_data}) 
    
    
);

endmodule
// Modified by Princeton University on June 9th, 2015
// ========== Copyright Header Begin ==========================================
//
// OpenSPARC T1 Processor File: bw_r_irf.v
// Copyright (c) 2006 Sun Microsystems, Inc.  All Rights Reserved.
// DO NOT ALTER OR REMOVE COPYRIGHT NOTICES.
//
// The above named program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public
// License version 2 as published by the Free Software Foundation.
//
// The above named program is distributed in the hope that it will be
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// General Public License for more details.
//
// You should have received a copy of the GNU General Public
// License along with this work; if not, write to the Free Software
// Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA 02110-1301, USA.
//
// ========== Copyright Header End ============================================
////////////////////////////////////////////////////////////////////////
/*
//  Module Name: bw_r_irf
//	Description: Register file with 3 read ports and 2 write ports.  Has
//				32 registers per thread with 4 threads.  Reading and writing
//				the same register concurrently produces x.
*/

//PITON_PROTO enables all FPGA related modifications






// Copyright (c) 2015 Princeton University
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of Princeton University nor the
//       names of its contributors may be used to endorse or promote products
//       derived from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY PRINCETON UNIVERSITY "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL PRINCETON UNIVERSITY BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

//==================================================================================================
//  Filename      : define.h
//  Created On    : 2014-02-20
//  Last Modified : 2018-11-16 17:14:11
//  Revision      :
//  Author        : Yaosheng Fu
//  Company       : Princeton University
//  Email         : yfu@princeton.edu
//
//  Description   : main header file defining global architecture parameters
//
//
//==================================================================================================






































































































































































































































































































































































































































































































































































































module bw_r_irf(so, irf_byp_rs1_data_d_l, irf_byp_rs2_data_d_l,
	irf_byp_rs3_data_d_l, irf_byp_rs3h_data_d_l, rclk, reset_l, si, se,
	sehold, rst_tri_en, ifu_exu_tid_s2, ifu_exu_rs1_s, ifu_exu_rs2_s,
	ifu_exu_rs3_s, ifu_exu_ren1_s, ifu_exu_ren2_s, ifu_exu_ren3_s,
	ecl_irf_wen_w, ecl_irf_wen_w2, ecl_irf_rd_m, ecl_irf_rd_g,
	byp_irf_rd_data_w, byp_irf_rd_data_w2, ecl_irf_tid_m, ecl_irf_tid_g,
	rml_irf_old_lo_cwp_e, rml_irf_new_lo_cwp_e, rml_irf_old_e_cwp_e,
	rml_irf_new_e_cwp_e, rml_irf_swap_even_e, rml_irf_swap_odd_e,
	rml_irf_swap_local_e, rml_irf_kill_restore_w, rml_irf_cwpswap_tid_e,
	rml_irf_old_agp, rml_irf_new_agp, rml_irf_swap_global,
	rml_irf_global_tid,

	// trin jtag read port

   core_rtap_data,
   rtap_core_val,
   rtap_core_threadid,
   rtap_core_id,
   rtap_core_data
	);

	input			rclk;
	input			reset_l;
	input			si;
	input			se;
	input			sehold;
	input			rst_tri_en;
	input	[1:0]		ifu_exu_tid_s2;
	input	[4:0]		ifu_exu_rs1_s;
	input	[4:0]		ifu_exu_rs2_s;
	input	[4:0]		ifu_exu_rs3_s;
	input			ifu_exu_ren1_s;
	input			ifu_exu_ren2_s;
	input			ifu_exu_ren3_s;
	input			ecl_irf_wen_w;
	input			ecl_irf_wen_w2;
	input	[4:0]		ecl_irf_rd_m;
	input	[4:0]		ecl_irf_rd_g;
	input	[71:0]		byp_irf_rd_data_w;
	input	[71:0]		byp_irf_rd_data_w2;
	input	[1:0]		ecl_irf_tid_m;
	input	[1:0]		ecl_irf_tid_g;
	input	[2:0]		rml_irf_old_lo_cwp_e;
	input	[2:0]		rml_irf_new_lo_cwp_e;
	input	[2:1]		rml_irf_old_e_cwp_e;
	input	[2:1]		rml_irf_new_e_cwp_e;
	input			rml_irf_swap_even_e;
	input			rml_irf_swap_odd_e;
	input			rml_irf_swap_local_e;
	input			rml_irf_kill_restore_w;
	input	[1:0]		rml_irf_cwpswap_tid_e;
	input	[1:0]		rml_irf_old_agp;
	input	[1:0]		rml_irf_new_agp;
	input			rml_irf_swap_global;
	input	[1:0]		rml_irf_global_tid;
	output			so;
	output	[71:0]		irf_byp_rs1_data_d_l;
	output	[71:0]		irf_byp_rs2_data_d_l;
	output	[71:0]		irf_byp_rs3_data_d_l;
	output	[31:0]		irf_byp_rs3h_data_d_l;

output reg [94-1:0] core_rtap_data;
input wire rtap_core_val;
input wire [1:0] rtap_core_threadid;
input wire [4-1:0]  rtap_core_id;
input wire [94-1:0] rtap_core_data;


	wire	[71:0]		irf_byp_rs1_data_d;
	wire	[71:0]		irf_byp_rs2_data_d;
	wire	[71:0]		irf_byp_rs3_data_d;
	wire	[71:0]		irf_byp_rs3h_data_d;
	wire	[1:0]		ecl_irf_tid_w;
	wire	[1:0]		ecl_irf_tid_w2;
	wire	[4:0]		ecl_irf_rd_w;
	wire	[4:0]		ecl_irf_rd_w2;
	wire	[1:0]		ifu_exu_thr_d;
	wire			ifu_exu_ren1_d;
	wire			ifu_exu_ren2_d;
	wire			ifu_exu_ren3_d;
	wire	[4:0]		ifu_exu_rs1_d;
	wire	[4:0]		ifu_exu_rs2_d;
	wire	[4:0]		ifu_exu_rs3_d;
	wire	[6:0]		thr_rs1;
	wire	[6:0]		thr_rs2;
	wire	[6:0]		thr_rs3;
	wire	[6:0]		thr_rs3h;
	wire	[6:0]		thr_rd_w;
	wire	[6:0]		thr_rd_w2;
	reg	[1:0]		cwpswap_tid_m;
	reg	[1:0]		cwpswap_tid_w;
	reg	[2:0]		old_lo_cwp_m;
	reg	[2:0]		new_lo_cwp_m;
	reg	[2:0]		new_lo_cwp_w;
	reg	[1:0]		old_e_cwp_m;
	reg	[1:0]		new_e_cwp_m;
	reg	[1:0]		new_e_cwp_w;
	reg			swap_local_m;
	reg			swap_local_w;
	reg			swap_even_m;
	reg			swap_even_w;
	reg			swap_odd_m;
	reg			swap_odd_w;
	reg			kill_restore_d1;
	reg			swap_global_d1;
	reg			swap_global_d2;
	reg	[1:0]		global_tid_d1;
	reg	[1:0]		global_tid_d2;
	reg	[1:0]		old_agp_d1;
	reg	[1:0]		new_agp_d1;
	reg	[1:0]		new_agp_d2;
	reg	[71:0]		active_win_thr_rd_w_neg;
	reg	[71:0]		active_win_thr_rd_w2_neg;
	reg	[6:0]		thr_rd_w_neg;
	reg	[6:0]		thr_rd_w2_neg;
	reg			active_win_thr_rd_w_neg_wr_en;
	reg			active_win_thr_rd_w2_neg_wr_en;
	reg			rst_tri_en_neg;
	wire			clk;
	wire			ren1_s;
	wire			ren2_s;
	wire			ren3_s;
	wire	[4:0]		rs1_s;
	wire	[4:0]		rs2_s;
	wire	[4:0]		rs3_s;
	wire	[1:0]		tid_s;
	wire	[1:0]		tid_g;
	wire	[1:0]		tid_m;
	wire	[4:0]		rd_m;
	wire	[4:0]		rd_g;
	wire			kill_restore_w;
	wire			swap_global_d1_vld;
	wire			swap_local_m_vld;
	wire			swap_even_m_vld;
	wire			swap_odd_m_vld;
	wire			wr_en;
	wire			wr_en2;

	assign clk = rclk;
	assign {ren1_s, ren2_s, ren3_s, rs1_s[4:0], rs2_s[4:0], rs3_s[4:0],
		tid_s[1:0], tid_g[1:0], tid_m[1:0], rd_m[4:0], rd_g[4:0]} = (
		sehold ? {ifu_exu_ren1_d, ifu_exu_ren2_d, ifu_exu_ren3_d,
		ifu_exu_rs1_d[4:0], ifu_exu_rs2_d[4:0], ifu_exu_rs3_d[4:0],
		ifu_exu_thr_d[1:0], ecl_irf_tid_w2[1:0], ecl_irf_tid_w[1:0],
		ecl_irf_rd_w[4:0], ecl_irf_rd_w2[4:0]} : {ifu_exu_ren1_s,
		ifu_exu_ren2_s, ifu_exu_ren3_s, ifu_exu_rs1_s[4:0],
		ifu_exu_rs2_s[4:0], ifu_exu_rs3_s[4:0], ifu_exu_tid_s2[1:0],
		ecl_irf_tid_g[1:0], ecl_irf_tid_m[1:0], ecl_irf_rd_m[4:0],
		ecl_irf_rd_g[4:0]});
	assign thr_rs1[6:0] = {ifu_exu_thr_d, ifu_exu_rs1_d};
	assign thr_rs2[6:0] = {ifu_exu_thr_d, ifu_exu_rs2_d};
	assign thr_rs3[6:0] = {ifu_exu_thr_d, ifu_exu_rs3_d[4:0]};
	assign thr_rs3h[6:0] = {ifu_exu_thr_d[1:0], ifu_exu_rs3_d[4:1], 1'b1};
	assign thr_rd_w[6:0] = {ecl_irf_tid_w, ecl_irf_rd_w};
	assign thr_rd_w2[6:0] = {ecl_irf_tid_w2, ecl_irf_rd_w2};
	assign irf_byp_rs1_data_d_l[71:0] = (~irf_byp_rs1_data_d[71:0]);
	assign irf_byp_rs2_data_d_l[71:0] = (~irf_byp_rs2_data_d[71:0]);
	assign irf_byp_rs3_data_d_l[71:0] = (~irf_byp_rs3_data_d[71:0]);
	assign irf_byp_rs3h_data_d_l[31:0] = (~irf_byp_rs3h_data_d[31:0]);
	assign kill_restore_w = (sehold ? kill_restore_d1 :
		rml_irf_kill_restore_w);
	assign swap_local_m_vld = (swap_local_m & (~rst_tri_en));
	assign swap_odd_m_vld = (swap_odd_m & (~rst_tri_en));
	assign swap_even_m_vld = (swap_even_m & (~rst_tri_en));
	assign swap_global_d1_vld = (swap_global_d1 & (~rst_tri_en));
	assign wr_en = (active_win_thr_rd_w_neg_wr_en & ((~rst_tri_en) | (~
		rst_tri_en_neg)));
	assign wr_en2 = (active_win_thr_rd_w2_neg_wr_en & ((~rst_tri_en) | (~
		rst_tri_en_neg)));

	// trin
	reg ren1_s_muxed;
	reg ren1_s_muxed_f;
	// reg ren1_s_muxed_ff;
	reg [4:0] rs1_s_muxed;
	reg [1:0] tid_s_muxed;
	always @ *
	begin
	    ren1_s_muxed = ren1_s;
	    rs1_s_muxed = rs1_s;
	    tid_s_muxed = tid_s;
	    if (rtap_core_val && rtap_core_id == 4'd6)
	    begin
	    	ren1_s_muxed = 1'b1;
	    	rs1_s_muxed = rtap_core_data[4:0];
	    	tid_s_muxed = rtap_core_threadid[1:0];
	    end

	    core_rtap_data = 0;
	    if (ren1_s_muxed_f)
	    	core_rtap_data = irf_byp_rs1_data_d;
	end

	always @ (posedge clk) ren1_s_muxed_f <= ren1_s_muxed;
	// always @ (posedge clk) ren1_s_muxed_ff <= ren1_s_muxed_f;

	dff_s dff_ren1_s2d(
		.din				(ren1_s_muxed),
		.clk				(clk),
		.q				(ifu_exu_ren1_d),
		.si(),
		.so(),
		.se				(se));
	dff_s dff_ren2_s2d(
		.din				(ren2_s),
		.clk				(clk),
		.q				(ifu_exu_ren2_d),
		.si(),
		.so(),
		.se				(se));
	dff_s dff_ren3_s2d(
		.din				(ren3_s),
		.clk				(clk),
		.q				(ifu_exu_ren3_d),
		.si(),
		.so(),
		.se				(se));
	dff_s #(5) dff_rs1_s2d(
		.din				(rs1_s_muxed[4:0]),
		.clk				(clk),
		.q				(ifu_exu_rs1_d[4:0]),
		.si(),
		.so(),
		.se				(se));
	dff_s #(5) dff_rs2_s2d(
		.din				(rs2_s[4:0]),
		.clk				(clk),
		.q				(ifu_exu_rs2_d[4:0]),
		.si(),
		.so(),
		.se				(se));
	dff_s #(5) dff_rs3_s2d(
		.din				(rs3_s[4:0]),
		.clk				(clk),
		.q				(ifu_exu_rs3_d[4:0]),
		.si(),
		.so(),
		.se				(se));
	dff_s #(2) dff_thr_s2d(
		.din				(tid_s_muxed[1:0]),
		.clk				(clk),
		.q				(ifu_exu_thr_d[1:0]),
		.si(),
		.so(),
		.se				(se));
	dff_s #(2) dff_thr_g2w2(
		.din				(tid_g[1:0]),
		.clk				(clk),
		.q				(ecl_irf_tid_w2[1:0]),
		.si(),
		.so(),
		.se				(se));
	dff_s #(2) dff_thr_m2w(
		.din				(tid_m[1:0]),
		.clk				(clk),
		.q				(ecl_irf_tid_w[1:0]),
		.si(),
		.so(),
		.se				(se));
	dff_s #(5) dff_rd_m2w(
		.din				(rd_m[4:0]),
		.clk				(clk),
		.q				(ecl_irf_rd_w[4:0]),
		.si(),
		.so(),
		.se				(se));
	dff_s #(5) dff_rd_g2w2(
		.din				(rd_g[4:0]),
		.clk				(clk),
		.q				(ecl_irf_rd_w2[4:0]),
		.si(),
		.so(),
		.se				(se));
	bw_r_irf_core bw_r_irf_core(
		.clk				(clk),
		.reset_l			(reset_l),
		.ifu_exu_ren1_d			(ifu_exu_ren1_d),
		.ifu_exu_ren2_d			(ifu_exu_ren2_d),
		.ifu_exu_ren3_d			(ifu_exu_ren3_d),
		.thr_rs1			(thr_rs1),
		.thr_rs2			(thr_rs2),
		.thr_rs3			(thr_rs3),
		.thr_rs3h			(thr_rs3h),
		.irf_byp_rs1_data_d		(irf_byp_rs1_data_d),
		.irf_byp_rs2_data_d		(irf_byp_rs2_data_d),
		.irf_byp_rs3_data_d		(irf_byp_rs3_data_d),
		.irf_byp_rs3h_data_d		(irf_byp_rs3h_data_d),
		.wr_en				(wr_en),
		.wr_en2				(wr_en2),
		.active_win_thr_rd_w_neg	(active_win_thr_rd_w_neg),
		.active_win_thr_rd_w2_neg	(active_win_thr_rd_w2_neg),
		.thr_rd_w_neg			(thr_rd_w_neg),
		.thr_rd_w2_neg			(thr_rd_w2_neg),
		.swap_global_d1_vld		(swap_global_d1_vld),
		.swap_global_d2			(swap_global_d2),
		.global_tid_d1			(global_tid_d1),
		.global_tid_d2			(global_tid_d2),
		.old_agp_d1			(old_agp_d1),
		.new_agp_d2			(new_agp_d2),
		.swap_local_m_vld		(swap_local_m_vld),
		.swap_local_w			(swap_local_w),
		.old_lo_cwp_m			(old_lo_cwp_m),
		.new_lo_cwp_w			(new_lo_cwp_w),
		.swap_even_m_vld		(swap_even_m_vld),
		.swap_even_w			(swap_even_w),
		.old_e_cwp_m			(old_e_cwp_m),
		.new_e_cwp_w			(new_e_cwp_w),
		.swap_odd_m_vld			(swap_odd_m_vld),
		.swap_odd_w			(swap_odd_w),
		.cwpswap_tid_m			(cwpswap_tid_m),
		.cwpswap_tid_w			(cwpswap_tid_w),
		.kill_restore_w			(kill_restore_w));

	always @(negedge clk) begin
	  rst_tri_en_neg <= rst_tri_en;
	  if ((ecl_irf_wen_w & ecl_irf_wen_w2) & (thr_rd_w[6:0] ==
		  thr_rd_w2[6:0])) begin
	    active_win_thr_rd_w_neg <= {72 {1'bx}};
	    thr_rd_w_neg <= thr_rd_w;
	    active_win_thr_rd_w_neg_wr_en <= 1'b1;
	    active_win_thr_rd_w2_neg_wr_en <= 1'b0;
	  end
	  else
	    begin
	      if (ecl_irf_wen_w & (thr_rd_w[4:0] != 5'b0)) begin
		active_win_thr_rd_w_neg <= byp_irf_rd_data_w;
		thr_rd_w_neg <= thr_rd_w;
		active_win_thr_rd_w_neg_wr_en <= 1'b1;
	      end
	      else begin
		active_win_thr_rd_w_neg_wr_en <= 1'b0;
	      end
	      if (ecl_irf_wen_w2 & (thr_rd_w2[4:0] != 5'b0)) begin
		active_win_thr_rd_w2_neg <= byp_irf_rd_data_w2;
		thr_rd_w2_neg <= thr_rd_w2;
		active_win_thr_rd_w2_neg_wr_en <= 1'b1;
	      end
	      else begin
		active_win_thr_rd_w2_neg_wr_en <= 1'b0;
	      end
	    end
	end
	always @(posedge clk) begin
	  cwpswap_tid_m[1:0] <= (sehold ? cwpswap_tid_m[1:0] :
		  rml_irf_cwpswap_tid_e[1:0]);
	  cwpswap_tid_w[1:0] <= cwpswap_tid_m[1:0];
	  old_lo_cwp_m[2:0] <= (sehold ? old_lo_cwp_m[2:0] :
		  rml_irf_old_lo_cwp_e[2:0]);
	  new_lo_cwp_m[2:0] <= (sehold ? new_lo_cwp_m[2:0] :
		  rml_irf_new_lo_cwp_e[2:0]);
	  new_lo_cwp_w[2:0] <= new_lo_cwp_m[2:0];
	  old_e_cwp_m[1:0] <= (sehold ? old_e_cwp_m[1:0] :
		  rml_irf_old_e_cwp_e[2:1]);
	  new_e_cwp_m[1:0] <= (sehold ? new_e_cwp_m[1:0] :
		  rml_irf_new_e_cwp_e[2:1]);
	  new_e_cwp_w[1:0] <= new_e_cwp_m[1:0];
	  swap_local_m <= (sehold ? (swap_local_m & rst_tri_en) :
		  rml_irf_swap_local_e);
	  swap_local_w <= swap_local_m_vld;
	  swap_odd_m <= (sehold ? (swap_odd_m & rst_tri_en) : rml_irf_swap_odd_e
		  );
	  swap_odd_w <= swap_odd_m_vld;
	  swap_even_m <= (sehold ? (swap_even_m & rst_tri_en) :
		  rml_irf_swap_even_e);
	  swap_even_w <= swap_even_m_vld;
	  kill_restore_d1 <= kill_restore_w;
	end
	always @(posedge clk) begin
	  swap_global_d1 <= (sehold ? (swap_global_d1 & rst_tri_en) :
		  rml_irf_swap_global);
	  swap_global_d2 <= swap_global_d1_vld;
	  global_tid_d1[1:0] <= (sehold ? global_tid_d1[1:0] :
		  rml_irf_global_tid[1:0]);
	  global_tid_d2[1:0] <= global_tid_d1[1:0];
	  old_agp_d1[1:0] <= (sehold ? old_agp_d1[1:0] : rml_irf_old_agp[1:0]);
	  new_agp_d1[1:0] <= (sehold ? new_agp_d1[1:0] : rml_irf_new_agp[1:0]);
	  new_agp_d2[1:0] <= new_agp_d1[1:0];
	end
/*
	always @(posedge clk) begin
	  if (wr_en) begin
	    $display("Write Port 1: %h %h", active_win_thr_rd_w_neg,
		    thr_rd_w_neg);
	  end
	  if (wr_en2) begin
	    $display("Write Port 2: %h %h", active_win_thr_rd_w2_neg,
		    thr_rd_w2_neg);
	  end
	  if (ifu_exu_ren1_d) begin
	    @(posedge clk) ;
	    $display("Read Port 1: %h %h", irf_byp_rs1_data_d, thr_rs1);
	  end
	  if (ifu_exu_ren2_d) begin
	    @(posedge clk) ;
	    $display("Read Port 2: %h %h", irf_byp_rs2_data_d, thr_rs2);
	  end
	  if (ifu_exu_ren3_d) begin
	    @(posedge clk) ;
	    $display("Read Port 3: %h %h", irf_byp_rs3_data_d, thr_rs3);
	  end
	end
*/
endmodule

module bw_r_irf_core(clk, reset_l, ifu_exu_ren1_d, ifu_exu_ren2_d, ifu_exu_ren3_d,
	thr_rs1, thr_rs2, thr_rs3, thr_rs3h, irf_byp_rs1_data_d,
	irf_byp_rs2_data_d, irf_byp_rs3_data_d, irf_byp_rs3h_data_d, wr_en,
	wr_en2, active_win_thr_rd_w_neg, active_win_thr_rd_w2_neg, thr_rd_w_neg,
	thr_rd_w2_neg, swap_global_d1_vld, swap_global_d2, global_tid_d1,
	global_tid_d2, old_agp_d1, new_agp_d2, swap_local_m_vld, swap_local_w,
	old_lo_cwp_m, new_lo_cwp_w, swap_even_m_vld, swap_even_w, old_e_cwp_m,
	new_e_cwp_w, swap_odd_m_vld, swap_odd_w, cwpswap_tid_m, cwpswap_tid_w,
	kill_restore_w);

	input			clk;
	input			reset_l;
	input			ifu_exu_ren1_d;
	input			ifu_exu_ren2_d;
	input			ifu_exu_ren3_d;
	input	[6:0]		thr_rs1;
	input	[6:0]		thr_rs2;
	input	[6:0]		thr_rs3;
	input	[6:0]		thr_rs3h;
	output	[71:0]		irf_byp_rs1_data_d;
	output	[71:0]		irf_byp_rs2_data_d;
	output	[71:0]		irf_byp_rs3_data_d;
	output	[71:0]		irf_byp_rs3h_data_d;
	input			wr_en;
	input			wr_en2;
	input	[71:0]		active_win_thr_rd_w_neg;
	input	[71:0]		active_win_thr_rd_w2_neg;
	input	[6:0]		thr_rd_w_neg;
	input	[6:0]		thr_rd_w2_neg;
	input			swap_global_d1_vld;
	input			swap_global_d2;
	input	[1:0]		global_tid_d1;
	input	[1:0]		global_tid_d2;
	input	[1:0]		old_agp_d1;
	input	[1:0]		new_agp_d2;
	input			swap_local_m_vld;
	input			swap_local_w;
	input	[2:0]		old_lo_cwp_m;
	input	[2:0]		new_lo_cwp_w;
	input			swap_even_m_vld;
	input			swap_even_w;
	input	[1:0]		old_e_cwp_m;
	input	[1:0]		new_e_cwp_w;
	input			swap_odd_m_vld;
	input			swap_odd_w;
	input	[1:0]		cwpswap_tid_m;
	input	[1:0]		cwpswap_tid_w;
	input			kill_restore_w;

	reg	[71:0]		irf_byp_rs1_data_d;
	reg	[71:0]		irf_byp_rs2_data_d;
	reg	[71:0]		irf_byp_rs3_data_d;
	reg	[71:0]		irf_byp_rs3h_data_d;
	wire	[71:0]		rd_data00;
	wire	[71:0]		rd_data01;
	wire	[71:0]		rd_data02;
	wire	[71:0]		rd_data03;
	wire	[71:0]		rd_data04;
	wire	[71:0]		rd_data05;
	wire	[71:0]		rd_data06;
	wire	[71:0]		rd_data07;
	wire	[71:0]		rd_data08;
	wire	[71:0]		rd_data09;
	wire	[71:0]		rd_data10;
	wire	[71:0]		rd_data11;
	wire	[71:0]		rd_data12;
	wire	[71:0]		rd_data13;
	wire	[71:0]		rd_data14;
	wire	[71:0]		rd_data15;
	wire	[71:0]		rd_data16;
	wire	[71:0]		rd_data17;
	wire	[71:0]		rd_data18;
	wire	[71:0]		rd_data19;
	wire	[71:0]		rd_data20;
	wire	[71:0]		rd_data21;
	wire	[71:0]		rd_data22;
	wire	[71:0]		rd_data23;
	wire	[71:0]		rd_data24;
	wire	[71:0]		rd_data25;
	wire	[71:0]		rd_data26;
	wire	[71:0]		rd_data27;
	wire	[71:0]		rd_data28;
	wire	[71:0]		rd_data29;
	wire	[71:0]		rd_data30;
	wire	[71:0]		rd_data31;
	wire			wren;
	wire	[4:0]		wr_addr;
	wire	[71:0]		wr_data;


	wire 	[127:0]	wr_en1s = (wr_en << {thr_rd_w_neg[4:0],thr_rd_w_neg[6:5]});
	wire	[127:0] wr_en2s = (wr_en2 << {thr_rd_w2_neg[4:0],thr_rd_w2_neg[6:5]});
	wire	[127:0]	wrens = wr_en1s | wr_en2s;

	wire	[3:0]	wr_th1 = wr_en << thr_rd_w_neg[6:5];

	wire	[71:0]	wr_data0 = wr_th1[0] ? active_win_thr_rd_w_neg : active_win_thr_rd_w2_neg;
	wire	[71:0]	wr_data1 = wr_th1[1] ? active_win_thr_rd_w_neg : active_win_thr_rd_w2_neg;
	wire	[71:0]	wr_data2 = wr_th1[2] ? active_win_thr_rd_w_neg : active_win_thr_rd_w2_neg;
	wire	[71:0]	wr_data3 = wr_th1[3] ? active_win_thr_rd_w_neg : active_win_thr_rd_w2_neg;


// this doesn't do anything.  we are replacing it with a zero on rd_data00
//	bw_r_irf_register16 register00(
//		.clk				(clk),
//		.wrens				(wrens[3:0]),
//		.save				(swap_global_d1_vld),
//		.save_addr			({global_tid_d1, 1'b0, old_agp_d1[1:0]}),
//		.restore			(swap_global_d2),
//		.restore_addr			({global_tid_d2, 1'b0, new_agp_d2[1:0]}),
//		.wr_data0			(72'b0),
//		.wr_data1			(72'b0),
//		.wr_data2			(72'b0),
//		.wr_data3			(72'b0),
//		.rd_thread			(thr_rs1[6:5]),
//		.rd_data			(rd_data00));
    assign rd_data00 = 72'd0;
	bw_r_irf_register8 register01(
		.clk				(clk),
		.reset_l			(reset_l),
		.wrens				(wrens[5:4]),
		.save				(swap_global_d1_vld),
		.save_addr			({global_tid_d1[0], 1'b0, old_agp_d1[1:0]}),
		.restore			(swap_global_d2),
		.restore_addr			({global_tid_d2[0], 1'b0, new_agp_d2[1:0]}),
		.wr_data0			(wr_data0),
		.wr_data1			(wr_data1),
		.rd_thread			(thr_rs1[5]),
		.rd_data			(rd_data01));
	bw_r_irf_register8 register02(
		.clk				(clk),
		.reset_l			(reset_l),
		.wrens				(wrens[9:8]),
		.save				(swap_global_d1_vld),
		.save_addr			({global_tid_d1[0], 1'b0, old_agp_d1[1:0]}),
		.restore			(swap_global_d2),
		.restore_addr			({global_tid_d2[0], 1'b0, new_agp_d2[1:0]}),
		.wr_data0			(wr_data0),
		.wr_data1			(wr_data1),
		.rd_thread			(thr_rs1[5]),
		.rd_data			(rd_data02));
	bw_r_irf_register8 register03(
		.clk				(clk),
		.reset_l			(reset_l),
		.wrens				(wrens[13:12]),
		.save				(swap_global_d1_vld),
		.save_addr			({global_tid_d1[0], 1'b0, old_agp_d1[1:0]}),
		.restore			(swap_global_d2),
		.restore_addr			({global_tid_d2[0], 1'b0, new_agp_d2[1:0]}),
		.wr_data0			(wr_data0),
		.wr_data1			(wr_data1),
		.rd_thread			(thr_rs1[5]),
		.rd_data			(rd_data03));
	bw_r_irf_register8 register04(
		.clk				(clk),
		.reset_l			(reset_l),
		.wrens				(wrens[17:16]),
		.save				(swap_global_d1_vld),
		.save_addr			({global_tid_d1[0], 1'b0, old_agp_d1[1:0]}),
		.restore			(swap_global_d2),
		.restore_addr			({global_tid_d2[0], 1'b0, new_agp_d2[1:0]}),
		.wr_data0			(wr_data0),
		.wr_data1			(wr_data1),
		.rd_thread			(thr_rs1[5]),
		.rd_data			(rd_data04));
	bw_r_irf_register8 register05(
		.clk				(clk),
		.reset_l			(reset_l),
		.wrens				(wrens[21:20]),
		.save				(swap_global_d1_vld),
		.save_addr			({global_tid_d1[0], 1'b0, old_agp_d1[1:0]}),
		.restore			(swap_global_d2),
		.restore_addr			({global_tid_d2[0], 1'b0, new_agp_d2[1:0]}),
		.wr_data0			(wr_data0),
		.wr_data1			(wr_data1),
		.rd_thread			(thr_rs1[5]),
		.rd_data			(rd_data05));
	bw_r_irf_register8 register06(
		.clk				(clk),
		.reset_l			(reset_l),
		.wrens				(wrens[25:24]),
		.save				(swap_global_d1_vld),
		.save_addr			({global_tid_d1[0], 1'b0, old_agp_d1[1:0]}),
		.restore			(swap_global_d2),
		.restore_addr			({global_tid_d2[0], 1'b0, new_agp_d2[1:0]}),
		.wr_data0			(wr_data0),
		.wr_data1			(wr_data1),
		.rd_thread			(thr_rs1[5]),
		.rd_data			(rd_data06));
	bw_r_irf_register8 register07(
		.clk				(clk),
		.reset_l			(reset_l),
		.wrens				(wrens[29:28]),
		.save				(swap_global_d1_vld),
		.save_addr			({global_tid_d1[0], 1'b0, old_agp_d1[1:0]}),
		.restore			(swap_global_d2),
		.restore_addr			({global_tid_d2[0], 1'b0, new_agp_d2[1:0]}),
		.wr_data0			(wr_data0),
		.wr_data1			(wr_data1),
		.rd_thread			(thr_rs1[5]),
		.rd_data			(rd_data07));

	bw_r_irf_register8 register08(
		.clk				(clk),
		.reset_l			(reset_l),
		.wrens				(wrens[33:32]),
		.save				(swap_odd_m_vld),
		.save_addr			({cwpswap_tid_m[0], 1'b0, old_lo_cwp_m[2:1]}),
		.restore			((swap_odd_w & (~kill_restore_w))),
		.restore_addr			({cwpswap_tid_w[0],1'b0, new_lo_cwp_w[2:1]}),
		.wr_data0			(wr_data0),
		.wr_data1			(wr_data1),
		.rd_thread			(thr_rs1[5]),
		.rd_data			(rd_data08));
	bw_r_irf_register8 register09(
		.clk				(clk),
		.reset_l			(reset_l),
		.wrens				(wrens[37:36]),
		.save				(swap_odd_m_vld),
		.save_addr			({cwpswap_tid_m[0], 1'b0, old_lo_cwp_m[2:1]}),
		.restore			((swap_odd_w & (~kill_restore_w))),
		.restore_addr			({cwpswap_tid_w[0],1'b0, new_lo_cwp_w[2:1]}),
		.wr_data0			(wr_data0),
		.wr_data1			(wr_data1),
		.rd_thread			(thr_rs1[5]),
		.rd_data			(rd_data09));
	bw_r_irf_register8 register10(
		.clk				(clk),
		.reset_l			(reset_l),
		.wrens				(wrens[41:40]),
		.save				(swap_odd_m_vld),
		.save_addr			({cwpswap_tid_m[0], 1'b0, old_lo_cwp_m[2:1]}),
		.restore			((swap_odd_w & (~kill_restore_w))),
		.restore_addr			({cwpswap_tid_w[0],1'b0, new_lo_cwp_w[2:1]}),
		.wr_data0			(wr_data0),
		.wr_data1			(wr_data1),
		.rd_thread			(thr_rs1[5]),
		.rd_data			(rd_data10));
	bw_r_irf_register8 register11(
		.clk				(clk),
		.reset_l			(reset_l),
		.wrens				(wrens[45:44]),
		.save				(swap_odd_m_vld),
		.save_addr			({cwpswap_tid_m[0], 1'b0, old_lo_cwp_m[2:1]}),
		.restore			((swap_odd_w & (~kill_restore_w))),
		.restore_addr			({cwpswap_tid_w[0],1'b0, new_lo_cwp_w[2:1]}),
		.wr_data0			(wr_data0),
		.wr_data1			(wr_data1),
		.rd_thread			(thr_rs1[5]),
		.rd_data			(rd_data11));
	bw_r_irf_register8 register12(
		.clk				(clk),
		.reset_l			(reset_l),
		.wrens				(wrens[49:48]),
		.save				(swap_odd_m_vld),
		.save_addr			({cwpswap_tid_m[0], 1'b0, old_lo_cwp_m[2:1]}),
		.restore			((swap_odd_w & (~kill_restore_w))),
		.restore_addr			({cwpswap_tid_w[0],1'b0, new_lo_cwp_w[2:1]}),
		.wr_data0			(wr_data0),
		.wr_data1			(wr_data1),
		.rd_thread			(thr_rs1[5]),
		.rd_data			(rd_data12));
	bw_r_irf_register8 register13(
		.clk				(clk),
		.reset_l			(reset_l),
		.wrens				(wrens[53:52]),
		.save				(swap_odd_m_vld),
		.save_addr			({cwpswap_tid_m[0], 1'b0, old_lo_cwp_m[2:1]}),
		.restore			((swap_odd_w & (~kill_restore_w))),
		.restore_addr			({cwpswap_tid_w[0],1'b0, new_lo_cwp_w[2:1]}),
		.wr_data0			(wr_data0),
		.wr_data1			(wr_data1),
		.rd_thread			(thr_rs1[5]),
		.rd_data			(rd_data13));
	bw_r_irf_register8 register14(
		.clk				(clk),
		.reset_l			(reset_l),
		.wrens				(wrens[57:56]),
		.save				(swap_odd_m_vld),
		.save_addr			({cwpswap_tid_m[0], 1'b0, old_lo_cwp_m[2:1]}),
		.restore			((swap_odd_w & (~kill_restore_w))),
		.restore_addr			({cwpswap_tid_w[0],1'b0, new_lo_cwp_w[2:1]}),
		.wr_data0			(wr_data0),
		.wr_data1			(wr_data1),
		.rd_thread			(thr_rs1[5]),
		.rd_data			(rd_data14));
	bw_r_irf_register8 register15(
		.clk				(clk),
		.reset_l			(reset_l),
		.wrens				(wrens[61:60]),
		.save				(swap_odd_m_vld),
		.save_addr			({cwpswap_tid_m[0], 1'b0, old_lo_cwp_m[2:1]}),
		.restore			((swap_odd_w & (~kill_restore_w))),
		.restore_addr			({cwpswap_tid_w[0],1'b0, new_lo_cwp_w[2:1]}),
		.wr_data0			(wr_data0),
		.wr_data1			(wr_data1),
		.rd_thread			(thr_rs1[5]),
		.rd_data			(rd_data15));

	bw_r_irf_register16 register16(
		.clk				(clk),
		.reset_l			(reset_l),
		.wrens				(wrens[65:64]),
		.save				(swap_local_m_vld),
		.save_addr			({cwpswap_tid_m[0], old_lo_cwp_m[2:0]}),
		.restore			((swap_local_w & (~kill_restore_w))),
		.restore_addr			({cwpswap_tid_w[0], new_lo_cwp_w[2:0]}),
		.wr_data0			(wr_data0),
		.wr_data1			(wr_data1),
		.rd_thread			(thr_rs1[5]),
		.rd_data			(rd_data16));
	bw_r_irf_register16 register17(
		.clk				(clk),
		.reset_l			(reset_l),
		.wrens				(wrens[69:68]),
		.save				(swap_local_m_vld),
		.save_addr			({cwpswap_tid_m[0], old_lo_cwp_m[2:0]}),
		.restore			((swap_local_w & (~kill_restore_w))),
		.restore_addr			({cwpswap_tid_w[0], new_lo_cwp_w[2:0]}),
		.wr_data0			(wr_data0),
		.wr_data1			(wr_data1),
		.rd_thread			(thr_rs1[5]),
		.rd_data			(rd_data17));
	bw_r_irf_register16 register18(
		.clk				(clk),
		.reset_l			(reset_l),
		.wrens				(wrens[73:72]),
		.save				(swap_local_m_vld),
		.save_addr			({cwpswap_tid_m[0], old_lo_cwp_m[2:0]}),
		.restore			((swap_local_w & (~kill_restore_w))),
		.restore_addr			({cwpswap_tid_w[0], new_lo_cwp_w[2:0]}),
		.wr_data0			(wr_data0),
		.wr_data1			(wr_data1),
		.rd_thread			(thr_rs1[5]),
		.rd_data			(rd_data18));
	bw_r_irf_register16 register19(
		.clk				(clk),
		.reset_l			(reset_l),
		.wrens				(wrens[77:76]),
		.save				(swap_local_m_vld),
		.save_addr			({cwpswap_tid_m[0], old_lo_cwp_m[2:0]}),
		.restore			((swap_local_w & (~kill_restore_w))),
		.restore_addr			({cwpswap_tid_w[0], new_lo_cwp_w[2:0]}),
		.wr_data0			(wr_data0),
		.wr_data1			(wr_data1),
		.rd_thread			(thr_rs1[5]),
		.rd_data			(rd_data19));
	bw_r_irf_register16 register20(
		.clk				(clk),
		.reset_l			(reset_l),
		.wrens				(wrens[81:80]),
		.save				(swap_local_m_vld),
		.save_addr			({cwpswap_tid_m[0], old_lo_cwp_m[2:0]}),
		.restore			((swap_local_w & (~kill_restore_w))),
		.restore_addr			({cwpswap_tid_w[0], new_lo_cwp_w[2:0]}),
		.wr_data0			(wr_data0),
		.wr_data1			(wr_data1),
		.rd_thread			(thr_rs1[5]),
		.rd_data			(rd_data20));
	bw_r_irf_register16 register21(
		.clk				(clk),
		.reset_l			(reset_l),
		.wrens				(wrens[85:84]),
		.save				(swap_local_m_vld),
		.save_addr			({cwpswap_tid_m[0], old_lo_cwp_m[2:0]}),
		.restore			((swap_local_w & (~kill_restore_w))),
		.restore_addr			({cwpswap_tid_w[0], new_lo_cwp_w[2:0]}),
		.wr_data0			(wr_data0),
		.wr_data1			(wr_data1),
		.rd_thread			(thr_rs1[5]),
		.rd_data			(rd_data21));
	bw_r_irf_register16 register22(
		.clk				(clk),
		.reset_l			(reset_l),
		.wrens				(wrens[89:88]),
		.save				(swap_local_m_vld),
		.save_addr			({cwpswap_tid_m[0], old_lo_cwp_m[2:0]}),
		.restore			((swap_local_w & (~kill_restore_w))),
		.restore_addr			({cwpswap_tid_w[0], new_lo_cwp_w[2:0]}),
		.wr_data0			(wr_data0),
		.wr_data1			(wr_data1),
		.rd_thread			(thr_rs1[5]),
		.rd_data			(rd_data22));
	bw_r_irf_register16 register23(
		.clk				(clk),
		.reset_l			(reset_l),
		.wrens				(wrens[93:92]),
		.save				(swap_local_m_vld),
		.save_addr			({cwpswap_tid_m[0], old_lo_cwp_m[2:0]}),
		.restore			((swap_local_w & (~kill_restore_w))),
		.restore_addr			({cwpswap_tid_w[0], new_lo_cwp_w[2:0]}),
		.wr_data0			(wr_data0),
		.wr_data1			(wr_data1),
		.rd_thread			(thr_rs1[5]),
		.rd_data			(rd_data23));

	bw_r_irf_register8 register24(
		.clk				(clk),
		.reset_l			(reset_l),
		.wrens				(wrens[97:96]),
		.save				(swap_even_m_vld),
		.save_addr			({cwpswap_tid_m[0], 1'b0, old_e_cwp_m[1:0]}),
		.restore			((swap_even_w & (~kill_restore_w))),
		.restore_addr			({cwpswap_tid_w[0], 1'b0, new_e_cwp_w[1:0]}),
		.wr_data0			(wr_data0),
		.wr_data1			(wr_data1),
		.rd_thread			(thr_rs1[5]),
		.rd_data			(rd_data24));
	bw_r_irf_register8 register25(
		.clk				(clk),
		.reset_l			(reset_l),
		.wrens				(wrens[101:100]),
		.save				(swap_even_m_vld),
		.save_addr			({cwpswap_tid_m[0], 1'b0, old_e_cwp_m[1:0]}),
		.restore			((swap_even_w & (~kill_restore_w))),
		.restore_addr			({cwpswap_tid_w[0], 1'b0, new_e_cwp_w[1:0]}),
		.wr_data0			(wr_data0),
		.wr_data1			(wr_data1),
		.rd_thread			(thr_rs1[5]),
		.rd_data			(rd_data25));
	bw_r_irf_register8 register26(
		.clk				(clk),
		.reset_l			(reset_l),
		.wrens				(wrens[105:104]),
		.save				(swap_even_m_vld),
		.save_addr			({cwpswap_tid_m[0], 1'b0, old_e_cwp_m[1:0]}),
		.restore			((swap_even_w & (~kill_restore_w))),
		.restore_addr			({cwpswap_tid_w[0], 1'b0, new_e_cwp_w[1:0]}),
		.wr_data0			(wr_data0),
		.wr_data1			(wr_data1),
		.rd_thread			(thr_rs1[5]),
		.rd_data			(rd_data26));
	bw_r_irf_register8 register27(
		.clk				(clk),
		.reset_l			(reset_l),
		.wrens				(wrens[109:108]),
		.save				(swap_even_m_vld),
		.save_addr			({cwpswap_tid_m[0], 1'b0, old_e_cwp_m[1:0]}),
		.restore			((swap_even_w & (~kill_restore_w))),
		.restore_addr			({cwpswap_tid_w[0], 1'b0, new_e_cwp_w[1:0]}),
		.wr_data0			(wr_data0),
		.wr_data1			(wr_data1),
		.rd_thread			(thr_rs1[5]),
		.rd_data			(rd_data27));
	bw_r_irf_register8 register28(
		.clk				(clk),
		.reset_l			(reset_l),
		.wrens				(wrens[113:112]),
		.save				(swap_even_m_vld),
		.save_addr			({cwpswap_tid_m[0], 1'b0, old_e_cwp_m[1:0]}),
		.restore			((swap_even_w & (~kill_restore_w))),
		.restore_addr			({cwpswap_tid_w[0], 1'b0, new_e_cwp_w[1:0]}),
		.wr_data0			(wr_data0),
		.wr_data1			(wr_data1),
		.rd_thread			(thr_rs1[5]),
		.rd_data			(rd_data28));
	bw_r_irf_register8 register29(
		.clk				(clk),
		.reset_l			(reset_l),
		.wrens				(wrens[117:116]),
		.save				(swap_even_m_vld),
		.save_addr			({cwpswap_tid_m[0], 1'b0, old_e_cwp_m[1:0]}),
		.restore			((swap_even_w & (~kill_restore_w))),
		.restore_addr			({cwpswap_tid_w[0], 1'b0, new_e_cwp_w[1:0]}),
		.wr_data0			(wr_data0),
		.wr_data1			(wr_data1),
		.rd_thread			(thr_rs1[5]),
		.rd_data			(rd_data29));
	bw_r_irf_register8 register30(
		.clk				(clk),
		.reset_l			(reset_l),
		.wrens				(wrens[121:120]),
		.save				(swap_even_m_vld),
		.save_addr			({cwpswap_tid_m[0], 1'b0, old_e_cwp_m[1:0]}),
		.restore			((swap_even_w & (~kill_restore_w))),
		.restore_addr			({cwpswap_tid_w[0], 1'b0, new_e_cwp_w[1:0]}),
		.wr_data0			(wr_data0),
		.wr_data1			(wr_data1),
		.rd_thread			(thr_rs1[5]),
		.rd_data			(rd_data30));
	bw_r_irf_register8 register31(
		.clk				(clk),
		.reset_l			(reset_l),
		.wrens				(wrens[125:124]),
		.save				(swap_even_m_vld),
		.save_addr			({cwpswap_tid_m[0], 1'b0, old_e_cwp_m[1:0]}),
		.restore			((swap_even_w & (~kill_restore_w))),
		.restore_addr			({cwpswap_tid_w[0], 1'b0, new_e_cwp_w[1:0]}),
		.wr_data0			(wr_data0),
		.wr_data1			(wr_data1),
		.rd_thread			(thr_rs1[5]),
		.rd_data			(rd_data31));


	always @(negedge clk) if (ifu_exu_ren1_d) begin
	  case (thr_rs1[4:0])
	    5'b0:
	      irf_byp_rs1_data_d <= rd_data00;
	    5'b1:
	      irf_byp_rs1_data_d <= rd_data01;
	    5'b00010:
	      irf_byp_rs1_data_d <= rd_data02;
	    5'b00011:
	      irf_byp_rs1_data_d <= rd_data03;
	    5'b00100:
	      irf_byp_rs1_data_d <= rd_data04;
	    5'b00101:
	      irf_byp_rs1_data_d <= rd_data05;
	    5'b00110:
	      irf_byp_rs1_data_d <= rd_data06;
	    5'b00111:
	      irf_byp_rs1_data_d <= rd_data07;
	    5'b01000:
	      irf_byp_rs1_data_d <= rd_data08;
	    5'b01001:
	      irf_byp_rs1_data_d <= rd_data09;
	    5'b01010:
	      irf_byp_rs1_data_d <= rd_data10;
	    5'b01011:
	      irf_byp_rs1_data_d <= rd_data11;
	    5'b01100:
	      irf_byp_rs1_data_d <= rd_data12;
	    5'b01101:
	      irf_byp_rs1_data_d <= rd_data13;
	    5'b01110:
	      irf_byp_rs1_data_d <= rd_data14;
	    5'b01111:
	      irf_byp_rs1_data_d <= rd_data15;
	    5'b10000:
	      irf_byp_rs1_data_d <= rd_data16;
	    5'b10001:
	      irf_byp_rs1_data_d <= rd_data17;
	    5'b10010:
	      irf_byp_rs1_data_d <= rd_data18;
	    5'b10011:
	      irf_byp_rs1_data_d <= rd_data19;
	    5'b10100:
	      irf_byp_rs1_data_d <= rd_data20;
	    5'b10101:
	      irf_byp_rs1_data_d <= rd_data21;
	    5'b10110:
	      irf_byp_rs1_data_d <= rd_data22;
	    5'b10111:
	      irf_byp_rs1_data_d <= rd_data23;
	    5'b11000:
	      irf_byp_rs1_data_d <= rd_data24;
	    5'b11001:
	      irf_byp_rs1_data_d <= rd_data25;
	    5'b11010:
	      irf_byp_rs1_data_d <= rd_data26;
	    5'b11011:
	      irf_byp_rs1_data_d <= rd_data27;
	    5'b11100:
	      irf_byp_rs1_data_d <= rd_data28;
	    5'b11101:
	      irf_byp_rs1_data_d <= rd_data29;
	    5'b11110:
	      irf_byp_rs1_data_d <= rd_data30;
	    5'b11111:
	      irf_byp_rs1_data_d <= rd_data31;
	  endcase
	end
	always @(negedge clk) if (ifu_exu_ren2_d) begin
	  case (thr_rs2[4:0])
	    5'b0:
	      irf_byp_rs2_data_d <= rd_data00;
	    5'b1:
	      irf_byp_rs2_data_d <= rd_data01;
	    5'b00010:
	      irf_byp_rs2_data_d <= rd_data02;
	    5'b00011:
	      irf_byp_rs2_data_d <= rd_data03;
	    5'b00100:
	      irf_byp_rs2_data_d <= rd_data04;
	    5'b00101:
	      irf_byp_rs2_data_d <= rd_data05;
	    5'b00110:
	      irf_byp_rs2_data_d <= rd_data06;
	    5'b00111:
	      irf_byp_rs2_data_d <= rd_data07;
	    5'b01000:
	      irf_byp_rs2_data_d <= rd_data08;
	    5'b01001:
	      irf_byp_rs2_data_d <= rd_data09;
	    5'b01010:
	      irf_byp_rs2_data_d <= rd_data10;
	    5'b01011:
	      irf_byp_rs2_data_d <= rd_data11;
	    5'b01100:
	      irf_byp_rs2_data_d <= rd_data12;
	    5'b01101:
	      irf_byp_rs2_data_d <= rd_data13;
	    5'b01110:
	      irf_byp_rs2_data_d <= rd_data14;
	    5'b01111:
	      irf_byp_rs2_data_d <= rd_data15;
	    5'b10000:
	      irf_byp_rs2_data_d <= rd_data16;
	    5'b10001:
	      irf_byp_rs2_data_d <= rd_data17;
	    5'b10010:
	      irf_byp_rs2_data_d <= rd_data18;
	    5'b10011:
	      irf_byp_rs2_data_d <= rd_data19;
	    5'b10100:
	      irf_byp_rs2_data_d <= rd_data20;
	    5'b10101:
	      irf_byp_rs2_data_d <= rd_data21;
	    5'b10110:
	      irf_byp_rs2_data_d <= rd_data22;
	    5'b10111:
	      irf_byp_rs2_data_d <= rd_data23;
	    5'b11000:
	      irf_byp_rs2_data_d <= rd_data24;
	    5'b11001:
	      irf_byp_rs2_data_d <= rd_data25;
	    5'b11010:
	      irf_byp_rs2_data_d <= rd_data26;
	    5'b11011:
	      irf_byp_rs2_data_d <= rd_data27;
	    5'b11100:
	      irf_byp_rs2_data_d <= rd_data28;
	    5'b11101:
	      irf_byp_rs2_data_d <= rd_data29;
	    5'b11110:
	      irf_byp_rs2_data_d <= rd_data30;
	    5'b11111:
	      irf_byp_rs2_data_d <= rd_data31;
	  endcase
	end
	always @(negedge clk) if (ifu_exu_ren3_d) begin
	  case (thr_rs3[4:0])
	    5'b0:
	      irf_byp_rs3_data_d <= rd_data00;
	    5'b1:
	      irf_byp_rs3_data_d <= rd_data01;
	    5'b00010:
	      irf_byp_rs3_data_d <= rd_data02;
	    5'b00011:
	      irf_byp_rs3_data_d <= rd_data03;
	    5'b00100:
	      irf_byp_rs3_data_d <= rd_data04;
	    5'b00101:
	      irf_byp_rs3_data_d <= rd_data05;
	    5'b00110:
	      irf_byp_rs3_data_d <= rd_data06;
	    5'b00111:
	      irf_byp_rs3_data_d <= rd_data07;
	    5'b01000:
	      irf_byp_rs3_data_d <= rd_data08;
	    5'b01001:
	      irf_byp_rs3_data_d <= rd_data09;
	    5'b01010:
	      irf_byp_rs3_data_d <= rd_data10;
	    5'b01011:
	      irf_byp_rs3_data_d <= rd_data11;
	    5'b01100:
	      irf_byp_rs3_data_d <= rd_data12;
	    5'b01101:
	      irf_byp_rs3_data_d <= rd_data13;
	    5'b01110:
	      irf_byp_rs3_data_d <= rd_data14;
	    5'b01111:
	      irf_byp_rs3_data_d <= rd_data15;
	    5'b10000:
	      irf_byp_rs3_data_d <= rd_data16;
	    5'b10001:
	      irf_byp_rs3_data_d <= rd_data17;
	    5'b10010:
	      irf_byp_rs3_data_d <= rd_data18;
	    5'b10011:
	      irf_byp_rs3_data_d <= rd_data19;
	    5'b10100:
	      irf_byp_rs3_data_d <= rd_data20;
	    5'b10101:
	      irf_byp_rs3_data_d <= rd_data21;
	    5'b10110:
	      irf_byp_rs3_data_d <= rd_data22;
	    5'b10111:
	      irf_byp_rs3_data_d <= rd_data23;
	    5'b11000:
	      irf_byp_rs3_data_d <= rd_data24;
	    5'b11001:
	      irf_byp_rs3_data_d <= rd_data25;
	    5'b11010:
	      irf_byp_rs3_data_d <= rd_data26;
	    5'b11011:
	      irf_byp_rs3_data_d <= rd_data27;
	    5'b11100:
	      irf_byp_rs3_data_d <= rd_data28;
	    5'b11101:
	      irf_byp_rs3_data_d <= rd_data29;
	    5'b11110:
	      irf_byp_rs3_data_d <= rd_data30;
	    5'b11111:
	      irf_byp_rs3_data_d <= rd_data31;
	  endcase
	end
	always @(negedge clk) if (ifu_exu_ren3_d) begin
	  case (thr_rs3h[4:1])
	    4'b0:
	      irf_byp_rs3h_data_d <= rd_data01;
	    4'b1:
	      irf_byp_rs3h_data_d <= rd_data03;
	    4'b0010:
	      irf_byp_rs3h_data_d <= rd_data05;
	    4'b0011:
	      irf_byp_rs3h_data_d <= rd_data07;
	    4'b0100:
	      irf_byp_rs3h_data_d <= rd_data09;
	    4'b0101:
	      irf_byp_rs3h_data_d <= rd_data11;
	    4'b0110:
	      irf_byp_rs3h_data_d <= rd_data13;
	    4'b0111:
	      irf_byp_rs3h_data_d <= rd_data15;
	    4'b1000:
	      irf_byp_rs3h_data_d <= rd_data17;
	    4'b1001:
	      irf_byp_rs3h_data_d <= rd_data19;
	    4'b1010:
	      irf_byp_rs3h_data_d <= rd_data21;
	    4'b1011:
	      irf_byp_rs3h_data_d <= rd_data23;
	    4'b1100:
	      irf_byp_rs3h_data_d <= rd_data25;
	    4'b1101:
	      irf_byp_rs3h_data_d <= rd_data27;
	    4'b1110:
	      irf_byp_rs3h_data_d <= rd_data29;
	    4'b1111:
	      irf_byp_rs3h_data_d <= rd_data31;
	  endcase
	end
endmodule





























































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































 // CONFIG_NUM_THREADS
// Modified by Princeton University on June 9th, 2015
// ========== Copyright Header Begin ==========================================
// 
// OpenSPARC T1 Processor File: sparc_exu.v
// Copyright (c) 2006 Sun Microsystems, Inc.  All Rights Reserved.
// DO NOT ALTER OR REMOVE COPYRIGHT NOTICES.
// 
// The above named program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public
// License version 2 as published by the Free Software Foundation.
// 
// The above named program is distributed in the hope that it will be 
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// General Public License for more details.
// 
// You should have received a copy of the GNU General Public
// License along with this work; if not, write to the Free Software
// Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA 02110-1301, USA.
// 
// ========== Copyright Header End ============================================
////////////////////////////////////////////////////////////////////////
/*
//  Module Name: sparc_exu
//	Description: Execution unit containing register file(IRF),
//			execution control (ECL), ALU, shifting (SHFT).
*/

// Copyright (c) 2015 Princeton University
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of Princeton University nor the
//       names of its contributors may be used to endorse or promote products
//       derived from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY PRINCETON UNIVERSITY "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL PRINCETON UNIVERSITY BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

//==================================================================================================
//  Filename      : define.h
//  Created On    : 2014-02-20
//  Last Modified : 2018-11-16 17:14:11
//  Revision      :
//  Author        : Yaosheng Fu
//  Company       : Princeton University
//  Email         : yfu@princeton.edu
//
//  Description   : main header file defining global architecture parameters
//
//
//==================================================================================================





































































































































































































































































































































































































































































































































































































// Modified by Princeton University on June 9th, 2015
/*
* ========== Copyright Header Begin ==========================================
* 
* OpenSPARC T1 Processor File: lsu.h
* Copyright (c) 2006 Sun Microsystems, Inc.  All Rights Reserved.
* DO NOT ALTER OR REMOVE COPYRIGHT NOTICES.
* 
* The above named program is free software; you can redistribute it and/or
* modify it under the terms of the GNU General Public
* License version 2 as published by the Free Software Foundation.
* 
* The above named program is distributed in the hope that it will be 
* useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
* General Public License for more details.
* 
* You should have received a copy of the GNU General Public
* License along with this work; if not, write to the Free Software
* Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA 02110-1301, USA.
* 
* ========== Copyright Header End ============================================
*/

// devices.xml






// 1:0


// 128

// 32


// 10

// 7


// 6:0



// 29 + 1 parity





// 144




















//`define STB_PCX_WY_HI   107
//`define STB_PCX_WY_LO   106



















































































// TLB Tag and Data Format
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	

	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	

	
	
	
	
	
	
	
	
	
	
	
	
	
	


// I-TLB version - lsu_tlb only.
























// // Invalidate Format
// //addr<5:4>=00
// `define CPX_A00_C0_LO	0
// `define CPX_A00_C0_HI	3
// `define CPX_A00_C1_LO	4
// `define CPX_A00_C1_HI	7
// `define CPX_A00_C2_LO	8
// `define CPX_A00_C2_HI	11
// `define CPX_A00_C3_LO	12
// `define CPX_A00_C3_HI	15
// `define CPX_A00_C4_LO	16
// `define CPX_A00_C4_HI	19
// `define CPX_A00_C5_LO	20
// `define CPX_A00_C5_HI	23
// `define CPX_A00_C6_LO	24
// `define CPX_A00_C6_HI	27
// `define CPX_A00_C7_LO	28
// `define CPX_A00_C7_HI	31

// //addr<5:4>=01
// `define CPX_A01_C0_LO	32
// `define CPX_A01_C0_HI	34
// `define CPX_A01_C1_LO	35
// `define CPX_A01_C1_HI	37
// `define CPX_A01_C2_LO	38
// `define CPX_A01_C2_HI	40
// `define CPX_A01_C3_LO	41
// `define CPX_A01_C3_HI	43
// `define CPX_A01_C4_LO	44
// `define CPX_A01_C4_HI	46
// `define CPX_A01_C5_LO	47
// `define CPX_A01_C5_HI	49
// `define CPX_A01_C6_LO	50
// `define CPX_A01_C6_HI	52
// `define CPX_A01_C7_LO	53
// `define CPX_A01_C7_HI	55

// //addr<5:4>=10
// `define CPX_A10_C0_LO	56
// `define CPX_A10_C0_HI	59
// `define CPX_A10_C1_LO	60
// `define CPX_A10_C1_HI	63
// `define CPX_A10_C2_LO	64
// `define CPX_A10_C2_HI	67
// `define CPX_A10_C3_LO	68
// `define CPX_A10_C3_HI	71
// `define CPX_A10_C4_LO	72
// `define CPX_A10_C4_HI	75
// `define CPX_A10_C5_LO	76
// `define CPX_A10_C5_HI	79
// `define CPX_A10_C6_LO	80
// `define CPX_A10_C6_HI	83
// `define CPX_A10_C7_LO	84
// `define CPX_A10_C7_HI	87

// //addr<5:4>=11
// `define CPX_A11_C0_LO	88
// `define CPX_A11_C0_HI	90
// `define CPX_A11_C1_LO	91
// `define CPX_A11_C1_HI	93
// `define CPX_A11_C2_LO	94
// `define CPX_A11_C2_HI	96
// `define CPX_A11_C3_LO	97
// `define CPX_A11_C3_HI	99
// `define CPX_A11_C4_LO	100
// `define CPX_A11_C4_HI	102
// `define CPX_A11_C5_LO	103
// `define CPX_A11_C5_HI	105
// `define CPX_A11_C6_LO	106
// `define CPX_A11_C6_HI	108
// `define CPX_A11_C7_LO	109
// `define CPX_A11_C7_HI	111

// cpuid - 4b



// CPUany, addr<5:4>=00,10
// `define CPX_AX0_INV_DVLD 0
// `define CPX_AX0_INV_IVLD 1
// `define CPX_AX0_INV_WY_LO 2
// `define CPX_AX0_INV_WY_HI 3

// CPUany, addr<5:4>=01,11
// `define CPX_AX1_INV_DVLD 0
// `define CPX_AX1_INV_WY_LO 1
// `define CPX_AX1_INV_WY_HI 2

// CPUany, addr<5:4>=01,11
// `define CPX_AX1_INV_DVLD 0
// `define CPX_AX1_INV_WY_LO 1
// `define CPX_AX1_INV_WY_HI 2

// DTAG parity error Invalidate




// CPX BINIT STORE


module sparc_exu (/*AUTOARG*/
   // Outputs
   exu_tlu_wsr_data_m, exu_tlu_va_oor_m, exu_tlu_va_oor_jl_ret_m, 
   exu_tlu_ue_trap_m, exu_tlu_ttype_vld_m, exu_tlu_ttype_m, 
   exu_tlu_spill_wtype, exu_tlu_spill_tid, exu_tlu_spill_other, 
   exu_tlu_spill, exu_tlu_misalign_addr_jmpl_rtn_m, 
   exu_tlu_cwp_retry, exu_tlu_cwp_cmplt_tid, exu_tlu_cwp_cmplt, 
   exu_tlu_cwp3_w, exu_tlu_cwp2_w, exu_tlu_cwp1_w, exu_tlu_cwp0_w, 
   exu_tlu_ccr3_w, exu_tlu_ccr2_w, exu_tlu_ccr1_w, exu_tlu_ccr0_w, 
   exu_spu_rs3_data_e, exu_mul_rs2_data, exu_mul_rs1_data, 
   exu_mul_input_vld, exu_mmu_early_va_e, exu_lsu_rs3_data_e, 
   exu_lsu_rs2_data_e, exu_lsu_priority_trap_m, exu_lsu_ldst_va_e, 
   exu_lsu_early_va_e, exu_ifu_va_oor_m, exu_ifu_spill_e, 
   exu_ifu_regz_e, exu_ifu_regn_e, exu_ifu_oddwin_s, 
   exu_ifu_longop_done_g, exu_ifu_inj_ack, exu_ifu_err_reg_m, 
   exu_ifu_ecc_ue_m, exu_ifu_ecc_ce_m, exu_ifu_cc_d, exu_ifu_brpc_e, 
   exu_ffu_wsr_inst_e, short_so0, short_so1, so0, exu_ifu_err_synd_m, 
   // Inputs
   tlu_exu_rsr_data_m, tlu_exu_priv_trap_m, tlu_exu_pic_twobelow_m, 
   tlu_exu_pic_onebelow_m, tlu_exu_cwpccr_update_m, 
   tlu_exu_cwp_retry_m, tlu_exu_cwp_m, tlu_exu_ccr_m, 
   tlu_exu_agp_tid, tlu_exu_agp_swap, tlu_exu_agp, sehold, se, rclk, 
   mul_exu_data_g, mul_exu_ack, lsu_exu_thr_m, 
   lsu_exu_st_dtlb_perr_g, lsu_exu_rd_m, lsu_exu_ldxa_m, 
   lsu_exu_ldxa_data_g, lsu_exu_ldst_miss_g2, lsu_exu_flush_pipe_w, 
   lsu_exu_dfill_vld_g, lsu_exu_dfill_data_g, ifu_tlu_wsr_inst_d, 
   ifu_tlu_sraddr_d, ifu_tlu_flush_m, ifu_exu_wen_d, 
   ifu_exu_useimm_d, ifu_exu_usecin_d, ifu_exu_use_rsr_e_l, 
   ifu_exu_tv_d, ifu_exu_ttype_vld_m, ifu_exu_tid_s2, ifu_exu_tcc_e, 
   ifu_exu_tagop_d, ifu_exu_shiftop_d, ifu_exu_sethi_inst_d, 
   ifu_exu_setcc_d, ifu_exu_saved_e, ifu_exu_save_d, 
   ifu_exu_rs3o_vld_d, ifu_exu_rs3e_vld_d, ifu_exu_rs3_s, 
   ifu_exu_rs2_vld_d, ifu_exu_rs2_s, ifu_exu_rs1_vld_d, 
   ifu_exu_rs1_s, ifu_exu_return_d, ifu_exu_restored_e, 
   ifu_exu_restore_d, ifu_exu_ren3_s, ifu_exu_ren2_s, ifu_exu_ren1_s, 
   ifu_exu_rd_ifusr_e, ifu_exu_rd_ffusr_e, ifu_exu_rd_exusr_e, 
   ifu_exu_rd_d, ifu_exu_range_check_other_d, 
   ifu_exu_range_check_jlret_d, ifu_exu_pcver_e, ifu_exu_pc_d, 
   ifu_exu_nceen_e, ifu_exu_muls_d, ifu_exu_muldivop_d, 
   ifu_exu_kill_e, ifu_exu_invert_d, ifu_exu_inst_vld_w, 
   ifu_exu_inst_vld_e, ifu_exu_inj_irferr, ifu_exu_imm_data_d, 
   ifu_exu_ialign_d, ifu_exu_flushw_e, ifu_exu_enshift_d, 
   ifu_exu_ecc_mask, ifu_exu_dontmv_regz1_e, ifu_exu_dontmv_regz0_e, 
   ifu_exu_disable_ce_e, ifu_exu_dbrinst_d, ifu_exu_casa_d, 
   ifu_exu_aluop_d, ifu_exu_addr_mask_d, grst_l, ffu_exu_rsr_data_m, 
   arst_l, 
   // mux_drive_disable, mem_write_disable, 
   short_si0, 
   short_si1, si0,

   core_rtap_data,
   rtap_core_val,
   rtap_core_threadid,
   rtap_core_id,
   rtap_core_data
   ) ;

output wire [94-1:0] core_rtap_data;
input wire rtap_core_val;
input wire [1:0] rtap_core_threadid;
input wire [4-1:0]  rtap_core_id;
input wire [94-1:0] rtap_core_data;

   // input mux_drive_disable;
   // input mem_write_disable;
   // wire mux_drive_disable = 0;
   // wire mem_write_disable = 0;
   input short_si0;
   input short_si1;
   input si0;
   output short_so0;
   output short_so1;
   output so0;
   /*AUTOINPUT*/
   // Beginning of automatic inputs (from unused autoinst inputs)
   input                arst_l;                 // To ecl of sparc_exu_ecl.v, ...
   input [63:0]         ffu_exu_rsr_data_m;     // To bypass of sparc_exu_byp.v
   input                grst_l;                 // To ecl of sparc_exu_ecl.v, ...
   input                ifu_exu_addr_mask_d;    // To ecl of sparc_exu_ecl.v
   input [2:0]          ifu_exu_aluop_d;        // To ecl of sparc_exu_ecl.v
   input                ifu_exu_casa_d;         // To ecl of sparc_exu_ecl.v
   input                ifu_exu_dbrinst_d;      // To ecl of sparc_exu_ecl.v
   input                ifu_exu_disable_ce_e;   // To ecl of sparc_exu_ecl.v
   input                ifu_exu_dontmv_regz0_e; // To ecl of sparc_exu_ecl.v
   input                ifu_exu_dontmv_regz1_e; // To ecl of sparc_exu_ecl.v
   input [7:0]          ifu_exu_ecc_mask;       // To ecl of sparc_exu_ecl.v
   input                ifu_exu_enshift_d;      // To ecl of sparc_exu_ecl.v
   input                ifu_exu_flushw_e;       // To rml of sparc_exu_rml.v
   input                ifu_exu_ialign_d;       // To ecl of sparc_exu_ecl.v
   input [31:0]         ifu_exu_imm_data_d;     // To bypass of sparc_exu_byp.v
   input                ifu_exu_inj_irferr;     // To ecl of sparc_exu_ecl.v
   input                ifu_exu_inst_vld_e;     // To ecl of sparc_exu_ecl.v
   input                ifu_exu_inst_vld_w;     // To ecl of sparc_exu_ecl.v
   input                ifu_exu_invert_d;       // To ecl of sparc_exu_ecl.v, ...
   input                ifu_exu_kill_e;         // To ecl of sparc_exu_ecl.v
   input [4:0]          ifu_exu_muldivop_d;     // To ecl of sparc_exu_ecl.v
   input                ifu_exu_muls_d;         // To ecl of sparc_exu_ecl.v
   input                ifu_exu_nceen_e;        // To ecl of sparc_exu_ecl.v
   input [47:0]         ifu_exu_pc_d;           // To bypass of sparc_exu_byp.v
   input [63:0]         ifu_exu_pcver_e;        // To bypass of sparc_exu_byp.v
   input                ifu_exu_range_check_jlret_d;// To ecl of sparc_exu_ecl.v
   input                ifu_exu_range_check_other_d;// To ecl of sparc_exu_ecl.v
   input [4:0]          ifu_exu_rd_d;           // To ecl of sparc_exu_ecl.v
   input                ifu_exu_rd_exusr_e;     // To ecl of sparc_exu_ecl.v
   input                ifu_exu_rd_ffusr_e;     // To ecl of sparc_exu_ecl.v
   input                ifu_exu_rd_ifusr_e;     // To ecl of sparc_exu_ecl.v
   input                ifu_exu_ren1_s;         // To irf of bw_r_irf.v
   input                ifu_exu_ren2_s;         // To irf of bw_r_irf.v
   input                ifu_exu_ren3_s;         // To irf of bw_r_irf.v
   input                ifu_exu_restore_d;      // To ecl of sparc_exu_ecl.v, ...
   input                ifu_exu_restored_e;     // To rml of sparc_exu_rml.v
   input                ifu_exu_return_d;       // To ecl of sparc_exu_ecl.v
   input [4:0]          ifu_exu_rs1_s;          // To irf of bw_r_irf.v, ...
   input                ifu_exu_rs1_vld_d;      // To ecl of sparc_exu_ecl.v
   input [4:0]          ifu_exu_rs2_s;          // To irf of bw_r_irf.v, ...
   input                ifu_exu_rs2_vld_d;      // To ecl of sparc_exu_ecl.v
   input [4:0]          ifu_exu_rs3_s;          // To irf of bw_r_irf.v, ...
   input                ifu_exu_rs3e_vld_d;     // To ecl of sparc_exu_ecl.v
   input                ifu_exu_rs3o_vld_d;     // To ecl of sparc_exu_ecl.v
   input                ifu_exu_save_d;         // To ecl of sparc_exu_ecl.v, ...
   input                ifu_exu_saved_e;        // To rml of sparc_exu_rml.v
   input                ifu_exu_setcc_d;        // To ecl of sparc_exu_ecl.v
   input                ifu_exu_sethi_inst_d;   // To ecl of sparc_exu_ecl.v
   input [2:0]          ifu_exu_shiftop_d;      // To ecl of sparc_exu_ecl.v
   input                ifu_exu_tagop_d;        // To ecl of sparc_exu_ecl.v
   input                ifu_exu_tcc_e;          // To ecl of sparc_exu_ecl.v
   input [1:0]          ifu_exu_tid_s2;         // To irf of bw_r_irf.v, ...
   input                ifu_exu_ttype_vld_m;    // To ecl of sparc_exu_ecl.v
   input                ifu_exu_tv_d;           // To ecl of sparc_exu_ecl.v
   input                ifu_exu_use_rsr_e_l;    // To ecl of sparc_exu_ecl.v
   input                ifu_exu_usecin_d;       // To ecl of sparc_exu_ecl.v
   input                ifu_exu_useimm_d;       // To ecl of sparc_exu_ecl.v
   input                ifu_exu_wen_d;          // To ecl of sparc_exu_ecl.v
   input                ifu_tlu_flush_m;        // To ecl of sparc_exu_ecl.v
   input [6:0]          ifu_tlu_sraddr_d;       // To ecl of sparc_exu_ecl.v
   input                ifu_tlu_wsr_inst_d;     // To ecl of sparc_exu_ecl.v
   input [63:0]         lsu_exu_dfill_data_g;   // To bypass of sparc_exu_byp.v
   input                lsu_exu_dfill_vld_g;    // To ecl of sparc_exu_ecl.v
   input                lsu_exu_flush_pipe_w;   // To ecl of sparc_exu_ecl.v
   input                lsu_exu_ldst_miss_g2;   // To ecl of sparc_exu_ecl.v
   input [63:0]         lsu_exu_ldxa_data_g;    // To bypass of sparc_exu_byp.v
   input                lsu_exu_ldxa_m;         // To ecl of sparc_exu_ecl.v
   input [4:0]          lsu_exu_rd_m;           // To ecl of sparc_exu_ecl.v
   input                lsu_exu_st_dtlb_perr_g; // To ecl of sparc_exu_ecl.v
   input [1:0]          lsu_exu_thr_m;          // To ecl of sparc_exu_ecl.v
   input                mul_exu_ack;            // To ecl of sparc_exu_ecl.v
   input [63:0]         mul_exu_data_g;         // To div of sparc_exu_div.v
   input                rclk;                   // To irf of bw_r_irf.v, ...
   input                se;                     // To irf of bw_r_irf.v, ...
   input                sehold;                 // To irf of bw_r_irf.v, ...
   input [1:0]          tlu_exu_agp;            // To rml of sparc_exu_rml.v
   input                tlu_exu_agp_swap;       // To rml of sparc_exu_rml.v
   input [1:0]          tlu_exu_agp_tid;        // To rml of sparc_exu_rml.v
   input [7:0]          tlu_exu_ccr_m;          // To ecl of sparc_exu_ecl.v
   input [2:0]          tlu_exu_cwp_m;          // To rml of sparc_exu_rml.v
   input                tlu_exu_cwp_retry_m;    // To rml of sparc_exu_rml.v
   input                tlu_exu_cwpccr_update_m;// To ecl of sparc_exu_ecl.v, ...
   input                tlu_exu_pic_onebelow_m; // To ecl of sparc_exu_ecl.v
   input                tlu_exu_pic_twobelow_m; // To ecl of sparc_exu_ecl.v
   input                tlu_exu_priv_trap_m;    // To ecl of sparc_exu_ecl.v
   input [63:0]         tlu_exu_rsr_data_m;     // To bypass of sparc_exu_byp.v
   // End of automatics
   /*AUTOOUTPUT*/
   // Beginning of automatic outputs (from unused autoinst outputs)
   output               exu_ffu_wsr_inst_e;     // From ecl of sparc_exu_ecl.v
   output [47:0]        exu_ifu_brpc_e;         // From alu of sparc_exu_alu.v
   output [7:0]         exu_ifu_cc_d;           // From ecl of sparc_exu_ecl.v
   output               exu_ifu_ecc_ce_m;       // From ecl of sparc_exu_ecl.v
   output               exu_ifu_ecc_ue_m;       // From ecl of sparc_exu_ecl.v
   output [7:0]         exu_ifu_err_reg_m;      // From ecl of sparc_exu_ecl.v
   output               exu_ifu_inj_ack;        // From ecl of sparc_exu_ecl.v
   output [3:0]         exu_ifu_longop_done_g;  // From ecl of sparc_exu_ecl.v
   output [3:0]         exu_ifu_oddwin_s;       // From rml of sparc_exu_rml.v
   output               exu_ifu_regn_e;         // From alu of sparc_exu_alu.v
   output               exu_ifu_regz_e;         // From alu of sparc_exu_alu.v
   output               exu_ifu_spill_e;        // From rml of sparc_exu_rml.v
   output               exu_ifu_va_oor_m;       // From ecl of sparc_exu_ecl.v
   output [(6 + 4):3]        exu_lsu_early_va_e;     // From alu of sparc_exu_alu.v
   output [47:0]        exu_lsu_ldst_va_e;      // From alu of sparc_exu_alu.v
   output               exu_lsu_priority_trap_m;// From ecl of sparc_exu_ecl.v
   output [63:0]        exu_lsu_rs2_data_e;     // From bypass of sparc_exu_byp.v
   output [63:0]        exu_lsu_rs3_data_e;     // From bypass of sparc_exu_byp.v
   output [7:0]         exu_mmu_early_va_e;     // From alu of sparc_exu_alu.v
   output               exu_mul_input_vld;      // From ecl of sparc_exu_ecl.v
   output [63:0]        exu_mul_rs1_data;       // From div of sparc_exu_div.v
   output [63:0]        exu_mul_rs2_data;       // From div of sparc_exu_div.v
   output [63:0]        exu_spu_rs3_data_e;     // From bypass of sparc_exu_byp.v
   output [7:0]         exu_tlu_ccr0_w;         // From ecl of sparc_exu_ecl.v
   output [7:0]         exu_tlu_ccr1_w;         // From ecl of sparc_exu_ecl.v
   output [7:0]         exu_tlu_ccr2_w;         // From ecl of sparc_exu_ecl.v
   output [7:0]         exu_tlu_ccr3_w;         // From ecl of sparc_exu_ecl.v
   output [2:0]         exu_tlu_cwp0_w;         // From rml of sparc_exu_rml.v
   output [2:0]         exu_tlu_cwp1_w;         // From rml of sparc_exu_rml.v
   output [2:0]         exu_tlu_cwp2_w;         // From rml of sparc_exu_rml.v
   output [2:0]         exu_tlu_cwp3_w;         // From rml of sparc_exu_rml.v
   output               exu_tlu_cwp_cmplt;      // From rml of sparc_exu_rml.v
   output [1:0]         exu_tlu_cwp_cmplt_tid;  // From rml of sparc_exu_rml.v
   output               exu_tlu_cwp_retry;      // From rml of sparc_exu_rml.v
   output               exu_tlu_misalign_addr_jmpl_rtn_m;// From ecl of sparc_exu_ecl.v
   output               exu_tlu_spill;          // From rml of sparc_exu_rml.v
   output               exu_tlu_spill_other;    // From rml of sparc_exu_rml.v
   output [1:0]         exu_tlu_spill_tid;      // From rml of sparc_exu_rml.v
   output [2:0]         exu_tlu_spill_wtype;    // From rml of sparc_exu_rml.v
   output [8:0]         exu_tlu_ttype_m;        // From ecl of sparc_exu_ecl.v
   output               exu_tlu_ttype_vld_m;    // From ecl of sparc_exu_ecl.v
   output               exu_tlu_ue_trap_m;      // From ecl of sparc_exu_ecl.v
   output               exu_tlu_va_oor_jl_ret_m;// From ecl of sparc_exu_ecl.v
   output               exu_tlu_va_oor_m;       // From ecl of sparc_exu_ecl.v
   output [63:0]        exu_tlu_wsr_data_m;     // From bypass of sparc_exu_byp.v
   // End of automatics
   /*AUTOWIRE*/
   // Beginning of automatic wires (for undeclared instantiated-module outputs)
   wire [63:0]          alu_byp_rd_data_e;      // From alu of sparc_exu_alu.v
   wire                 alu_ecl_add_n32_e;      // From alu of sparc_exu_alu.v
   wire                 alu_ecl_add_n64_e;      // From alu of sparc_exu_alu.v
   wire                 alu_ecl_adder_out_63_e; // From alu of sparc_exu_alu.v
   wire                 alu_ecl_adderin2_31_e;  // From alu of sparc_exu_alu.v
   wire                 alu_ecl_adderin2_63_e;  // From alu of sparc_exu_alu.v
   wire                 alu_ecl_cout32_e;       // From alu of sparc_exu_alu.v
   wire                 alu_ecl_cout64_e_l;     // From alu of sparc_exu_alu.v
   wire                 alu_ecl_log_n32_e;      // From alu of sparc_exu_alu.v
   wire                 alu_ecl_log_n64_e;      // From alu of sparc_exu_alu.v
   wire                 alu_ecl_mem_addr_invalid_e_l;// From alu of sparc_exu_alu.v
   wire                 alu_ecl_zhigh_e;        // From alu of sparc_exu_alu.v
   wire                 alu_ecl_zlow_e;         // From alu of sparc_exu_alu.v
   wire [63:0]          byp_alu_rcc_data_e;     // From bypass of sparc_exu_byp.v
   wire [63:0]          byp_alu_rs1_data_e;     // From bypass of sparc_exu_byp.v
   wire [63:0]          byp_alu_rs2_data_e_l;   // From bypass of sparc_exu_byp.v
   wire [63:0]          byp_ecc_rcc_data_e;     // From bypass of sparc_exu_byp.v
   wire [7:0]           byp_ecc_rs1_synd_d;     // From bypass of sparc_exu_byp.v
   wire [7:0]           byp_ecc_rs2_synd_d;     // From bypass of sparc_exu_byp.v
   wire [63:0]          byp_ecc_rs3_data_e;     // From bypass of sparc_exu_byp.v
   wire [7:0]           byp_ecc_rs3_synd_d;     // From bypass of sparc_exu_byp.v
   wire [2:0]           byp_ecl_rs1_2_0_e;      // From bypass of sparc_exu_byp.v
   wire                 byp_ecl_rs1_31_e;       // From bypass of sparc_exu_byp.v
   wire                 byp_ecl_rs1_63_e;       // From bypass of sparc_exu_byp.v
   wire                 byp_ecl_rs2_31_e;       // From bypass of sparc_exu_byp.v
   wire [3:0]           byp_ecl_rs2_3_0_e;      // From bypass of sparc_exu_byp.v
   wire [71:0]          byp_irf_rd_data_w;      // From bypass of sparc_exu_byp.v
   wire [71:0]          byp_irf_rd_data_w2;     // From bypass of sparc_exu_byp.v
   wire [63:0]          div_byp_muldivout_g;    // From div of sparc_exu_div.v
   wire [31:0]          div_byp_yreg_e;         // From div of sparc_exu_div.v
   wire                 div_ecl_adder_out_31;   // From div of sparc_exu_div.v
   wire                 div_ecl_cout32;         // From div of sparc_exu_div.v
   wire                 div_ecl_cout64;         // From div of sparc_exu_div.v
   wire                 div_ecl_d_62;           // From div of sparc_exu_div.v
   wire                 div_ecl_d_msb;          // From div of sparc_exu_div.v
   wire                 div_ecl_detect_zero_high;// From div of sparc_exu_div.v
   wire                 div_ecl_detect_zero_low;// From div of sparc_exu_div.v
   wire                 div_ecl_dividend_msb;   // From div of sparc_exu_div.v
   wire                 div_ecl_gencc_in_31;    // From div of sparc_exu_div.v
   wire                 div_ecl_gencc_in_msb_l; // From div of sparc_exu_div.v
   wire                 div_ecl_low32_nonzero;  // From div of sparc_exu_div.v
   wire                 div_ecl_upper32_equal;  // From div of sparc_exu_div.v
   wire                 div_ecl_x_msb;          // From div of sparc_exu_div.v
   wire                 div_ecl_xin_msb_l;      // From div of sparc_exu_div.v
   wire [3:0]           div_ecl_yreg_0_l;       // From div of sparc_exu_div.v
   wire [63:0]          ecc_byp_ecc_result_m;   // From ecc of sparc_exu_ecc.v
   wire                 ecc_ecl_rs1_ce;         // From ecc of sparc_exu_ecc.v
   wire                 ecc_ecl_rs1_ue;         // From ecc of sparc_exu_ecc.v
   wire                 ecc_ecl_rs2_ce;         // From ecc of sparc_exu_ecc.v
   wire                 ecc_ecl_rs2_ue;         // From ecc of sparc_exu_ecc.v
   wire                 ecc_ecl_rs3_ce;         // From ecc of sparc_exu_ecc.v
   wire                 ecc_ecl_rs3_ue;         // From ecc of sparc_exu_ecc.v
   wire                 ecl_alu_cin_e;          // From ecl of sparc_exu_ecl.v
   wire                 ecl_alu_log_sel_and_e;  // From ecl of sparc_exu_ecl.v
   wire                 ecl_alu_log_sel_move_e; // From ecl of sparc_exu_ecl.v
   wire                 ecl_alu_log_sel_or_e;   // From ecl of sparc_exu_ecl.v
   wire                 ecl_alu_log_sel_xor_e;  // From ecl of sparc_exu_ecl.v
   wire                 ecl_alu_out_sel_logic_e_l;// From ecl of sparc_exu_ecl.v
   wire                 ecl_alu_out_sel_rs3_e_l;// From ecl of sparc_exu_ecl.v
   wire                 ecl_alu_out_sel_shift_e_l;// From ecl of sparc_exu_ecl.v
   wire                 ecl_alu_out_sel_sum_e_l;// From ecl of sparc_exu_ecl.v
   wire                 ecl_alu_sethi_inst_e;   // From ecl of sparc_exu_ecl.v
   wire [2:0]           ecl_byp_3lsb_m;         // From ecl of sparc_exu_ecl.v
   wire [7:0]           ecl_byp_ecc_mask_m_l;   // From ecl of sparc_exu_ecl.v
   wire [7:0]           ecl_byp_eclpr_e;        // From ecl of sparc_exu_ecl.v
   wire                 ecl_byp_ldxa_g;         // From ecl of sparc_exu_ecl.v
   wire                 ecl_byp_rcc_mux1_sel_m; // From ecl of sparc_exu_ecl.v
   wire                 ecl_byp_rcc_mux1_sel_other;// From ecl of sparc_exu_ecl.v
   wire                 ecl_byp_rcc_mux1_sel_w; // From ecl of sparc_exu_ecl.v
   wire                 ecl_byp_rcc_mux1_sel_w2;// From ecl of sparc_exu_ecl.v
   wire                 ecl_byp_rcc_mux2_sel_e; // From ecl of sparc_exu_ecl.v
   wire                 ecl_byp_rcc_mux2_sel_ld;// From ecl of sparc_exu_ecl.v
   wire                 ecl_byp_rcc_mux2_sel_rf;// From ecl of sparc_exu_ecl.v
   wire                 ecl_byp_rcc_mux2_sel_usemux1;// From ecl of sparc_exu_ecl.v
   wire                 ecl_byp_restore_m;      // From ecl of sparc_exu_ecl.v
   wire                 ecl_byp_rs1_longmux_sel_g2;// From ecl of sparc_exu_ecl.v
   wire                 ecl_byp_rs1_longmux_sel_ldxa;// From ecl of sparc_exu_ecl.v
   wire                 ecl_byp_rs1_longmux_sel_w2;// From ecl of sparc_exu_ecl.v
   wire                 ecl_byp_rs1_mux1_sel_m; // From ecl of sparc_exu_ecl.v
   wire                 ecl_byp_rs1_mux1_sel_other;// From ecl of sparc_exu_ecl.v
   wire                 ecl_byp_rs1_mux1_sel_w; // From ecl of sparc_exu_ecl.v
   wire                 ecl_byp_rs1_mux1_sel_w2;// From ecl of sparc_exu_ecl.v
   wire                 ecl_byp_rs1_mux2_sel_e; // From ecl of sparc_exu_ecl.v
   wire                 ecl_byp_rs1_mux2_sel_ld;// From ecl of sparc_exu_ecl.v
   wire                 ecl_byp_rs1_mux2_sel_rf;// From ecl of sparc_exu_ecl.v
   wire                 ecl_byp_rs1_mux2_sel_usemux1;// From ecl of sparc_exu_ecl.v
   wire                 ecl_byp_rs2_longmux_sel_g2;// From ecl of sparc_exu_ecl.v
   wire                 ecl_byp_rs2_longmux_sel_ldxa;// From ecl of sparc_exu_ecl.v
   wire                 ecl_byp_rs2_longmux_sel_w2;// From ecl of sparc_exu_ecl.v
   wire                 ecl_byp_rs2_mux1_sel_m; // From ecl of sparc_exu_ecl.v
   wire                 ecl_byp_rs2_mux1_sel_other;// From ecl of sparc_exu_ecl.v
   wire                 ecl_byp_rs2_mux1_sel_w; // From ecl of sparc_exu_ecl.v
   wire                 ecl_byp_rs2_mux1_sel_w2;// From ecl of sparc_exu_ecl.v
   wire                 ecl_byp_rs2_mux2_sel_e; // From ecl of sparc_exu_ecl.v
   wire                 ecl_byp_rs2_mux2_sel_ld;// From ecl of sparc_exu_ecl.v
   wire                 ecl_byp_rs2_mux2_sel_rf;// From ecl of sparc_exu_ecl.v
   wire                 ecl_byp_rs2_mux2_sel_usemux1;// From ecl of sparc_exu_ecl.v
   wire                 ecl_byp_rs3_longmux_sel_g2;// From ecl of sparc_exu_ecl.v
   wire                 ecl_byp_rs3_longmux_sel_ldxa;// From ecl of sparc_exu_ecl.v
   wire                 ecl_byp_rs3_longmux_sel_w2;// From ecl of sparc_exu_ecl.v
   wire                 ecl_byp_rs3_mux1_sel_m; // From ecl of sparc_exu_ecl.v
   wire                 ecl_byp_rs3_mux1_sel_other;// From ecl of sparc_exu_ecl.v
   wire                 ecl_byp_rs3_mux1_sel_w; // From ecl of sparc_exu_ecl.v
   wire                 ecl_byp_rs3_mux1_sel_w2;// From ecl of sparc_exu_ecl.v
   wire                 ecl_byp_rs3_mux2_sel_e; // From ecl of sparc_exu_ecl.v
   wire                 ecl_byp_rs3_mux2_sel_ld;// From ecl of sparc_exu_ecl.v
   wire                 ecl_byp_rs3_mux2_sel_rf;// From ecl of sparc_exu_ecl.v
   wire                 ecl_byp_rs3_mux2_sel_usemux1;// From ecl of sparc_exu_ecl.v
   wire                 ecl_byp_rs3h_longmux_sel_g2;// From ecl of sparc_exu_ecl.v
   wire                 ecl_byp_rs3h_longmux_sel_ldxa;// From ecl of sparc_exu_ecl.v
   wire                 ecl_byp_rs3h_longmux_sel_w2;// From ecl of sparc_exu_ecl.v
   wire                 ecl_byp_rs3h_mux1_sel_m;// From ecl of sparc_exu_ecl.v
   wire                 ecl_byp_rs3h_mux1_sel_other;// From ecl of sparc_exu_ecl.v
   wire                 ecl_byp_rs3h_mux1_sel_w;// From ecl of sparc_exu_ecl.v
   wire                 ecl_byp_rs3h_mux1_sel_w2;// From ecl of sparc_exu_ecl.v
   wire                 ecl_byp_rs3h_mux2_sel_e;// From ecl of sparc_exu_ecl.v
   wire                 ecl_byp_rs3h_mux2_sel_ld;// From ecl of sparc_exu_ecl.v
   wire                 ecl_byp_rs3h_mux2_sel_rf;// From ecl of sparc_exu_ecl.v
   wire                 ecl_byp_rs3h_mux2_sel_usemux1;// From ecl of sparc_exu_ecl.v
   wire                 ecl_byp_sel_alu_e;      // From ecl of sparc_exu_ecl.v
   wire                 ecl_byp_sel_ecc_m;      // From ecl of sparc_exu_ecl.v
   wire                 ecl_byp_sel_eclpr_e;    // From ecl of sparc_exu_ecl.v
   wire                 ecl_byp_sel_ffusr_m;    // From ecl of sparc_exu_ecl.v
   wire                 ecl_byp_sel_ifex_m;     // From ecl of sparc_exu_ecl.v
   wire                 ecl_byp_sel_ifusr_e;    // From ecl of sparc_exu_ecl.v
   wire                 ecl_byp_sel_load_g;     // From ecl of sparc_exu_ecl.v
   wire                 ecl_byp_sel_load_m;     // From ecl of sparc_exu_ecl.v
   wire                 ecl_byp_sel_muldiv_g;   // From ecl of sparc_exu_ecl.v
   wire                 ecl_byp_sel_pipe_m;     // From ecl of sparc_exu_ecl.v
   wire                 ecl_byp_sel_restore_g;  // From ecl of sparc_exu_ecl.v
   wire                 ecl_byp_sel_restore_m;  // From ecl of sparc_exu_ecl.v
   wire                 ecl_byp_sel_tlusr_m;    // From ecl of sparc_exu_ecl.v
   wire                 ecl_byp_sel_yreg_e;     // From ecl of sparc_exu_ecl.v
   wire                 ecl_byp_std_e_l;        // From ecl of sparc_exu_ecl.v
   wire                 ecl_div_almostlast_cycle;// From ecl of sparc_exu_ecl.v
   wire                 ecl_div_cin;            // From ecl of sparc_exu_ecl.v
   wire                 ecl_div_div64;          // From ecl of sparc_exu_ecl.v
   wire                 ecl_div_dividend_sign;  // From ecl of sparc_exu_ecl.v
   wire                 ecl_div_keep_d;         // From ecl of sparc_exu_ecl.v
   wire                 ecl_div_keepx;          // From ecl of sparc_exu_ecl.v
   wire                 ecl_div_last_cycle;     // From ecl of sparc_exu_ecl.v
   wire                 ecl_div_ld_inputs;      // From ecl of sparc_exu_ecl.v
   wire                 ecl_div_mul_get_32bit_data;// From ecl of sparc_exu_ecl.v
   wire                 ecl_div_mul_get_new_data;// From ecl of sparc_exu_ecl.v
   wire                 ecl_div_mul_keep_data;  // From ecl of sparc_exu_ecl.v
   wire                 ecl_div_mul_sext_rs1_e; // From ecl of sparc_exu_ecl.v
   wire                 ecl_div_mul_sext_rs2_e; // From ecl of sparc_exu_ecl.v
   wire                 ecl_div_mul_wen;        // From ecl of sparc_exu_ecl.v
   wire                 ecl_div_muls;           // From ecl of sparc_exu_ecl.v
   wire                 ecl_div_muls_rs1_31_e_l;// From ecl of sparc_exu_ecl.v
   wire                 ecl_div_newq;           // From ecl of sparc_exu_ecl.v
   wire                 ecl_div_sel_64b;        // From ecl of sparc_exu_ecl.v
   wire                 ecl_div_sel_adder;      // From ecl of sparc_exu_ecl.v
   wire                 ecl_div_sel_div;        // From ecl of sparc_exu_ecl.v
   wire                 ecl_div_sel_neg32;      // From ecl of sparc_exu_ecl.v
   wire                 ecl_div_sel_pos32;      // From ecl of sparc_exu_ecl.v
   wire                 ecl_div_sel_u32;        // From ecl of sparc_exu_ecl.v
   wire                 ecl_div_subtract_l;     // From ecl of sparc_exu_ecl.v
   wire [3:0]           ecl_div_thr_e;          // From ecl of sparc_exu_ecl.v
   wire                 ecl_div_upper32_zero;   // From ecl of sparc_exu_ecl.v
   wire                 ecl_div_upper33_one;    // From ecl of sparc_exu_ecl.v
   wire                 ecl_div_upper33_zero;   // From ecl of sparc_exu_ecl.v
   wire                 ecl_div_xinmask;        // From ecl of sparc_exu_ecl.v
   wire                 ecl_div_yreg_data_31_g; // From ecl of sparc_exu_ecl.v
   wire [3:0]           ecl_div_yreg_shift_g;   // From ecl of sparc_exu_ecl.v
   wire [3:0]           ecl_div_yreg_wen_g;     // From ecl of sparc_exu_ecl.v
   wire [3:0]           ecl_div_yreg_wen_l;     // From ecl of sparc_exu_ecl.v
   wire [3:0]           ecl_div_yreg_wen_w;     // From ecl of sparc_exu_ecl.v
   wire                 ecl_div_zero_rs2_e;     // From ecl of sparc_exu_ecl.v
   wire                 ecl_ecc_log_rs1_m;      // From ecl of sparc_exu_ecl.v
   wire                 ecl_ecc_log_rs2_m;      // From ecl of sparc_exu_ecl.v
   wire                 ecl_ecc_log_rs3_m;      // From ecl of sparc_exu_ecl.v
   wire                 ecl_ecc_rs1_use_rf_e;   // From ecl of sparc_exu_ecl.v
   wire                 ecl_ecc_rs2_use_rf_e;   // From ecl of sparc_exu_ecl.v
   wire                 ecl_ecc_rs3_use_rf_e;   // From ecl of sparc_exu_ecl.v
   wire                 ecl_ecc_sel_rs1_m_l;    // From ecl of sparc_exu_ecl.v
   wire                 ecl_ecc_sel_rs2_m_l;    // From ecl of sparc_exu_ecl.v
   wire                 ecl_ecc_sel_rs3_m_l;    // From ecl of sparc_exu_ecl.v
   wire [4:0]           ecl_irf_rd_g;           // From ecl of sparc_exu_ecl.v
   wire [4:0]           ecl_irf_rd_m;           // From ecl of sparc_exu_ecl.v
   wire [1:0]           ecl_irf_tid_g;          // From ecl of sparc_exu_ecl.v
   wire [1:0]           ecl_irf_tid_m;          // From ecl of sparc_exu_ecl.v
   wire                 ecl_irf_wen_w;          // From ecl of sparc_exu_ecl.v
   wire                 ecl_irf_wen_w2;         // From ecl of sparc_exu_ecl.v
   wire                 ecl_rml_canrestore_wen_w;// From ecl of sparc_exu_ecl.v
   wire                 ecl_rml_cansave_wen_w;  // From ecl of sparc_exu_ecl.v
   wire                 ecl_rml_cleanwin_wen_w; // From ecl of sparc_exu_ecl.v
   wire                 ecl_rml_cwp_wen_e;      // From ecl of sparc_exu_ecl.v
   wire                 ecl_rml_early_flush_w;  // From ecl of sparc_exu_ecl.v
   wire                 ecl_rml_inst_vld_w;     // From ecl of sparc_exu_ecl.v
   wire                 ecl_rml_kill_e;         // From ecl of sparc_exu_ecl.v
   wire                 ecl_rml_kill_w;         // From ecl of sparc_exu_ecl.v
   wire                 ecl_rml_otherwin_wen_w; // From ecl of sparc_exu_ecl.v
   wire [3:0]           ecl_rml_thr_m;          // From ecl of sparc_exu_ecl.v
   wire [3:0]           ecl_rml_thr_w;          // From ecl of sparc_exu_ecl.v
   wire                 ecl_rml_wstate_wen_w;   // From ecl of sparc_exu_ecl.v
   wire [2:0]           ecl_rml_xor_data_e;     // From ecl of sparc_exu_ecl.v
   wire                 ecl_shft_enshift_e_l;   // From ecl of sparc_exu_ecl.v
   wire                 ecl_shft_extend32bit_e_l;// From ecl of sparc_exu_ecl.v
   wire                 ecl_shft_extendbit_e;   // From ecl of sparc_exu_ecl.v
   wire                 ecl_shft_lshift_e_l;    // From ecl of sparc_exu_ecl.v
   wire                 ecl_shft_op32_e;        // From ecl of sparc_exu_ecl.v
   wire [3:0]           ecl_shft_shift1_e;      // From ecl of sparc_exu_ecl.v
   wire [3:0]           ecl_shft_shift4_e;      // From ecl of sparc_exu_ecl.v
   wire [71:0]          irf_byp_rs1_data_d_l;   // From irf of bw_r_irf.v
   wire [71:0]          irf_byp_rs2_data_d_l;   // From irf of bw_r_irf.v
   wire [71:0]          irf_byp_rs3_data_d_l;   // From irf of bw_r_irf.v
   wire [31:0]          irf_byp_rs3h_data_d_l;  // From irf of bw_r_irf.v
   wire [2:0]           rml_ecl_canrestore_d;   // From rml of sparc_exu_rml.v
   wire [2:0]           rml_ecl_cansave_d;      // From rml of sparc_exu_rml.v
   wire                 rml_ecl_clean_window_e; // From rml of sparc_exu_rml.v
   wire [2:0]           rml_ecl_cleanwin_d;     // From rml of sparc_exu_rml.v
   wire [2:0]           rml_ecl_cwp_d;          // From rml of sparc_exu_rml.v
   wire                 rml_ecl_fill_e;         // From rml of sparc_exu_rml.v
   wire [1:0]           rml_ecl_gl_e;           // From rml of sparc_exu_rml.v
   wire                 rml_ecl_kill_m;         // From rml of sparc_exu_rml.v
   wire                 rml_ecl_other_e;        // From rml of sparc_exu_rml.v
   wire [2:0]           rml_ecl_otherwin_d;     // From rml of sparc_exu_rml.v
   wire                 rml_ecl_rmlop_done_e;   // From rml of sparc_exu_rml.v
   wire [3:0]           rml_ecl_swap_done;      // From rml of sparc_exu_rml.v
   wire [5:0]           rml_ecl_wstate_d;       // From rml of sparc_exu_rml.v
   wire [2:0]           rml_ecl_wtype_e;        // From rml of sparc_exu_rml.v
   wire [1:0]           rml_irf_cwpswap_tid_e;  // From rml of sparc_exu_rml.v
   wire [1:0]           rml_irf_global_tid;     // From rml of sparc_exu_rml.v
   wire                 rml_irf_kill_restore_w; // From rml of sparc_exu_rml.v
   wire [1:0]           rml_irf_new_agp;        // From rml of sparc_exu_rml.v
   wire [2:0]           rml_irf_new_lo_cwp_e;   // From rml of sparc_exu_rml.v
   wire [1:0]           rml_irf_old_agp;        // From rml of sparc_exu_rml.v
   wire [2:0]           rml_irf_old_lo_cwp_e;   // From rml of sparc_exu_rml.v
   wire                 rml_irf_swap_even_e;    // From rml of sparc_exu_rml.v
   wire                 rml_irf_swap_global;    // From rml of sparc_exu_rml.v
   wire                 rml_irf_swap_local_e;   // From rml of sparc_exu_rml.v
   wire                 rml_irf_swap_odd_e;     // From rml of sparc_exu_rml.v
   wire [63:0]          shft_alu_shift_out_e;   // From shft of sparc_exu_shft.v
   // End of automatics
   wire                 short_scan0_1;
   wire                 scan0_1,scan0_2,scan0_3;

   wire                 ecl_alu_casa_e;
   wire [63:0]          byp_alu_rs2_data_e;
   output [7:0]         exu_ifu_err_synd_m;
   wire [1:0]           rml_irf_old_e_cwp_e;
   wire [1:0]           rml_irf_new_e_cwp_e;

wire mux_drive_disable = ~grst_l;
wire mem_write_disable = ~grst_l;
 
   bw_r_irf_wrap irf(
                .reset_l (arst_l),
                .rst_tri_en             (mem_write_disable),
                .rml_irf_old_e_cwp_e    (rml_irf_old_e_cwp_e[1:0]),
                .rml_irf_new_e_cwp_e    (rml_irf_new_e_cwp_e[1:0]),
                /*AUTOINST*/
                // Outputs
                .irf_byp_rs1_data_d_l   (irf_byp_rs1_data_d_l[71:0]),
                .irf_byp_rs2_data_d_l   (irf_byp_rs2_data_d_l[71:0]),
                .irf_byp_rs3_data_d_l   (irf_byp_rs3_data_d_l[71:0]),
                .irf_byp_rs3h_data_d_l  (irf_byp_rs3h_data_d_l[31:0]),
                // Inputs
                .rclk                   (rclk),
                .sehold                 (sehold),
                .ifu_exu_tid_s2         (ifu_exu_tid_s2[0]),
                .ifu_exu_rs1_s          (ifu_exu_rs1_s[4:0]),
                .ifu_exu_rs2_s          (ifu_exu_rs2_s[4:0]),
                .ifu_exu_rs3_s          (ifu_exu_rs3_s[4:0]),
                .ifu_exu_ren1_s         (ifu_exu_ren1_s),
                .ifu_exu_ren2_s         (ifu_exu_ren2_s),
                .ifu_exu_ren3_s         (ifu_exu_ren3_s),
                .ecl_irf_wen_w          (ecl_irf_wen_w),
                .ecl_irf_wen_w2         (ecl_irf_wen_w2),
                .ecl_irf_rd_m           (ecl_irf_rd_m[4:0]),
                .ecl_irf_rd_g           (ecl_irf_rd_g[4:0]),
                .byp_irf_rd_data_w      (byp_irf_rd_data_w[71:0]),
                .byp_irf_rd_data_w2     (byp_irf_rd_data_w2[71:0]),
                .ecl_irf_tid_m          (ecl_irf_tid_m[1:0]),
                .ecl_irf_tid_g          (ecl_irf_tid_g[1:0]),
                .rml_irf_old_lo_cwp_e   (rml_irf_old_lo_cwp_e[2:0]),
                .rml_irf_new_lo_cwp_e   (rml_irf_new_lo_cwp_e[2:0]),
                .rml_irf_swap_even_e    (rml_irf_swap_even_e),
                .rml_irf_swap_odd_e     (rml_irf_swap_odd_e),
                .rml_irf_swap_local_e   (rml_irf_swap_local_e),
                .rml_irf_kill_restore_w (rml_irf_kill_restore_w),
                .rml_irf_cwpswap_tid_e  (rml_irf_cwpswap_tid_e[0]),
                .rml_irf_old_agp        (rml_irf_old_agp[1:0]),
                .rml_irf_new_agp        (rml_irf_new_agp[1:0]),
                .rml_irf_swap_global    (rml_irf_swap_global),
                .rml_irf_global_tid     (rml_irf_global_tid[0]),

                .core_rtap_data          (core_rtap_data),
                .rtap_core_val         (rtap_core_val),
                .rtap_core_threadid         (rtap_core_threadid[0]),
                .rtap_core_id         (rtap_core_id),
                .rtap_core_data         (rtap_core_data[4:0])
                );
   
   sparc_exu_byp bypass(
                        .so             (short_so1),
                        .si             (short_si1),
                        .byp_alu_rs2_data_e(byp_alu_rs2_data_e[63:0]),
                        /*AUTOINST*/
                        // Outputs
                        .byp_alu_rs1_data_e(byp_alu_rs1_data_e[63:0]),
                        .byp_alu_rs2_data_e_l(byp_alu_rs2_data_e_l[63:0]),
                        .exu_lsu_rs3_data_e(exu_lsu_rs3_data_e[63:0]),
                        .exu_spu_rs3_data_e(exu_spu_rs3_data_e[63:0]),
                        .exu_lsu_rs2_data_e(exu_lsu_rs2_data_e[63:0]),
                        .byp_alu_rcc_data_e(byp_alu_rcc_data_e[63:0]),
                        .byp_irf_rd_data_w(byp_irf_rd_data_w[71:0]),
                        .exu_tlu_wsr_data_m(exu_tlu_wsr_data_m[63:0]),
                        .byp_irf_rd_data_w2(byp_irf_rd_data_w2[71:0]),
                        .byp_ecc_rs3_data_e(byp_ecc_rs3_data_e[63:0]),
                        .byp_ecc_rcc_data_e(byp_ecc_rcc_data_e[63:0]),
                        .byp_ecl_rs2_31_e(byp_ecl_rs2_31_e),
                        .byp_ecl_rs1_31_e(byp_ecl_rs1_31_e),
                        .byp_ecl_rs1_63_e(byp_ecl_rs1_63_e),
                        .byp_ecl_rs1_2_0_e(byp_ecl_rs1_2_0_e[2:0]),
                        .byp_ecl_rs2_3_0_e(byp_ecl_rs2_3_0_e[3:0]),
                        .byp_ecc_rs1_synd_d(byp_ecc_rs1_synd_d[7:0]),
                        .byp_ecc_rs2_synd_d(byp_ecc_rs2_synd_d[7:0]),
                        .byp_ecc_rs3_synd_d(byp_ecc_rs3_synd_d[7:0]),
                        // Inputs
                        .rclk           (rclk),
                        .se             (se),
                        .sehold         (sehold),
                        .ecl_byp_rs1_mux2_sel_e(ecl_byp_rs1_mux2_sel_e),
                        .ecl_byp_rs1_mux2_sel_rf(ecl_byp_rs1_mux2_sel_rf),
                        .ecl_byp_rs1_mux2_sel_ld(ecl_byp_rs1_mux2_sel_ld),
                        .ecl_byp_rs1_mux2_sel_usemux1(ecl_byp_rs1_mux2_sel_usemux1),
                        .ecl_byp_rs1_mux1_sel_m(ecl_byp_rs1_mux1_sel_m),
                        .ecl_byp_rs1_mux1_sel_w(ecl_byp_rs1_mux1_sel_w),
                        .ecl_byp_rs1_mux1_sel_w2(ecl_byp_rs1_mux1_sel_w2),
                        .ecl_byp_rs1_mux1_sel_other(ecl_byp_rs1_mux1_sel_other),
                        .ecl_byp_rcc_mux2_sel_e(ecl_byp_rcc_mux2_sel_e),
                        .ecl_byp_rcc_mux2_sel_rf(ecl_byp_rcc_mux2_sel_rf),
                        .ecl_byp_rcc_mux2_sel_ld(ecl_byp_rcc_mux2_sel_ld),
                        .ecl_byp_rcc_mux2_sel_usemux1(ecl_byp_rcc_mux2_sel_usemux1),
                        .ecl_byp_rcc_mux1_sel_m(ecl_byp_rcc_mux1_sel_m),
                        .ecl_byp_rcc_mux1_sel_w(ecl_byp_rcc_mux1_sel_w),
                        .ecl_byp_rcc_mux1_sel_w2(ecl_byp_rcc_mux1_sel_w2),
                        .ecl_byp_rcc_mux1_sel_other(ecl_byp_rcc_mux1_sel_other),
                        .ecl_byp_rs2_mux2_sel_e(ecl_byp_rs2_mux2_sel_e),
                        .ecl_byp_rs2_mux2_sel_rf(ecl_byp_rs2_mux2_sel_rf),
                        .ecl_byp_rs2_mux2_sel_ld(ecl_byp_rs2_mux2_sel_ld),
                        .ecl_byp_rs2_mux2_sel_usemux1(ecl_byp_rs2_mux2_sel_usemux1),
                        .ecl_byp_rs2_mux1_sel_m(ecl_byp_rs2_mux1_sel_m),
                        .ecl_byp_rs2_mux1_sel_w(ecl_byp_rs2_mux1_sel_w),
                        .ecl_byp_rs2_mux1_sel_w2(ecl_byp_rs2_mux1_sel_w2),
                        .ecl_byp_rs2_mux1_sel_other(ecl_byp_rs2_mux1_sel_other),
                        .ecl_byp_rs3_mux2_sel_e(ecl_byp_rs3_mux2_sel_e),
                        .ecl_byp_rs3_mux2_sel_rf(ecl_byp_rs3_mux2_sel_rf),
                        .ecl_byp_rs3_mux2_sel_ld(ecl_byp_rs3_mux2_sel_ld),
                        .ecl_byp_rs3_mux2_sel_usemux1(ecl_byp_rs3_mux2_sel_usemux1),
                        .ecl_byp_rs3_mux1_sel_m(ecl_byp_rs3_mux1_sel_m),
                        .ecl_byp_rs3_mux1_sel_w(ecl_byp_rs3_mux1_sel_w),
                        .ecl_byp_rs3_mux1_sel_w2(ecl_byp_rs3_mux1_sel_w2),
                        .ecl_byp_rs3_mux1_sel_other(ecl_byp_rs3_mux1_sel_other),
                        .ecl_byp_rs3h_mux2_sel_e(ecl_byp_rs3h_mux2_sel_e),
                        .ecl_byp_rs3h_mux2_sel_rf(ecl_byp_rs3h_mux2_sel_rf),
                        .ecl_byp_rs3h_mux2_sel_ld(ecl_byp_rs3h_mux2_sel_ld),
                        .ecl_byp_rs3h_mux2_sel_usemux1(ecl_byp_rs3h_mux2_sel_usemux1),
                        .ecl_byp_rs3h_mux1_sel_m(ecl_byp_rs3h_mux1_sel_m),
                        .ecl_byp_rs3h_mux1_sel_w(ecl_byp_rs3h_mux1_sel_w),
                        .ecl_byp_rs3h_mux1_sel_w2(ecl_byp_rs3h_mux1_sel_w2),
                        .ecl_byp_rs3h_mux1_sel_other(ecl_byp_rs3h_mux1_sel_other),
                        .ecl_byp_rs1_longmux_sel_g2(ecl_byp_rs1_longmux_sel_g2),
                        .ecl_byp_rs1_longmux_sel_w2(ecl_byp_rs1_longmux_sel_w2),
                        .ecl_byp_rs1_longmux_sel_ldxa(ecl_byp_rs1_longmux_sel_ldxa),
                        .ecl_byp_rs2_longmux_sel_g2(ecl_byp_rs2_longmux_sel_g2),
                        .ecl_byp_rs2_longmux_sel_w2(ecl_byp_rs2_longmux_sel_w2),
                        .ecl_byp_rs2_longmux_sel_ldxa(ecl_byp_rs2_longmux_sel_ldxa),
                        .ecl_byp_rs3_longmux_sel_g2(ecl_byp_rs3_longmux_sel_g2),
                        .ecl_byp_rs3_longmux_sel_w2(ecl_byp_rs3_longmux_sel_w2),
                        .ecl_byp_rs3_longmux_sel_ldxa(ecl_byp_rs3_longmux_sel_ldxa),
                        .ecl_byp_rs3h_longmux_sel_g2(ecl_byp_rs3h_longmux_sel_g2),
                        .ecl_byp_rs3h_longmux_sel_w2(ecl_byp_rs3h_longmux_sel_w2),
                        .ecl_byp_rs3h_longmux_sel_ldxa(ecl_byp_rs3h_longmux_sel_ldxa),
                        .ecl_byp_sel_load_m(ecl_byp_sel_load_m),
                        .ecl_byp_sel_pipe_m(ecl_byp_sel_pipe_m),
                        .ecl_byp_sel_ecc_m(ecl_byp_sel_ecc_m),
                        .ecl_byp_sel_muldiv_g(ecl_byp_sel_muldiv_g),
                        .ecl_byp_sel_load_g(ecl_byp_sel_load_g),
                        .ecl_byp_sel_restore_g(ecl_byp_sel_restore_g),
                        .ecl_byp_std_e_l(ecl_byp_std_e_l),
                        .ecl_byp_ldxa_g (ecl_byp_ldxa_g),
                        .alu_byp_rd_data_e(alu_byp_rd_data_e[63:0]),
                        .ifu_exu_imm_data_d(ifu_exu_imm_data_d[31:0]),
                        .irf_byp_rs1_data_d_l(irf_byp_rs1_data_d_l[71:0]),
                        .irf_byp_rs2_data_d_l(irf_byp_rs2_data_d_l[71:0]),
                        .irf_byp_rs3_data_d_l(irf_byp_rs3_data_d_l[71:0]),
                        .irf_byp_rs3h_data_d_l(irf_byp_rs3h_data_d_l[31:0]),
                        .lsu_exu_dfill_data_g(lsu_exu_dfill_data_g[63:0]),
                        .lsu_exu_ldxa_data_g(lsu_exu_ldxa_data_g[63:0]),
                        .div_byp_muldivout_g(div_byp_muldivout_g[63:0]),
                        .ecc_byp_ecc_result_m(ecc_byp_ecc_result_m[63:0]),
                        .ecl_byp_ecc_mask_m_l(ecl_byp_ecc_mask_m_l[7:0]),
                        .ifu_exu_pc_d   (ifu_exu_pc_d[47:0]),
                        .ecl_byp_3lsb_m (ecl_byp_3lsb_m[2:0]),
                        .ecl_byp_restore_m(ecl_byp_restore_m),
                        .ecl_byp_sel_restore_m(ecl_byp_sel_restore_m),
                        .ecl_byp_eclpr_e(ecl_byp_eclpr_e[7:0]),
                        .div_byp_yreg_e (div_byp_yreg_e[31:0]),
                        .ifu_exu_pcver_e(ifu_exu_pcver_e[63:0]),
                        .tlu_exu_rsr_data_m(tlu_exu_rsr_data_m[63:0]),
                        .ffu_exu_rsr_data_m(ffu_exu_rsr_data_m[63:0]),
                        .ecl_byp_sel_yreg_e(ecl_byp_sel_yreg_e),
                        .ecl_byp_sel_eclpr_e(ecl_byp_sel_eclpr_e),
                        .ecl_byp_sel_ifusr_e(ecl_byp_sel_ifusr_e),
                        .ecl_byp_sel_alu_e(ecl_byp_sel_alu_e),
                        .ecl_byp_sel_ifex_m(ecl_byp_sel_ifex_m),
                        .ecl_byp_sel_ffusr_m(ecl_byp_sel_ffusr_m),
                        .ecl_byp_sel_tlusr_m(ecl_byp_sel_tlusr_m));

   sparc_exu_ecc ecc(
                     .so                (scan0_1),
                     .si                (si0),
                     .byp_alu_rs2_data_e(byp_alu_rs2_data_e[63:0]),
                     /*AUTOINST*/
                     // Outputs
                     .ecc_ecl_rs1_ce    (ecc_ecl_rs1_ce),
                     .ecc_ecl_rs1_ue    (ecc_ecl_rs1_ue),
                     .ecc_ecl_rs2_ce    (ecc_ecl_rs2_ce),
                     .ecc_ecl_rs2_ue    (ecc_ecl_rs2_ue),
                     .ecc_ecl_rs3_ce    (ecc_ecl_rs3_ce),
                     .ecc_ecl_rs3_ue    (ecc_ecl_rs3_ue),
                     .ecc_byp_ecc_result_m(ecc_byp_ecc_result_m[63:0]),
                     .exu_ifu_err_synd_m(exu_ifu_err_synd_m[6:0]),
                     // Inputs
                     .rclk              (rclk),
                     .se                (se),
                     .byp_ecc_rcc_data_e(byp_ecc_rcc_data_e[63:0]),
                     .ecl_ecc_rs1_use_rf_e(ecl_ecc_rs1_use_rf_e),
                     .byp_ecc_rs1_synd_d(byp_ecc_rs1_synd_d[7:0]),
                     .ecl_ecc_rs2_use_rf_e(ecl_ecc_rs2_use_rf_e),
                     .byp_ecc_rs2_synd_d(byp_ecc_rs2_synd_d[7:0]),
                     .byp_ecc_rs3_data_e(byp_ecc_rs3_data_e[63:0]),
                     .ecl_ecc_rs3_use_rf_e(ecl_ecc_rs3_use_rf_e),
                     .byp_ecc_rs3_synd_d(byp_ecc_rs3_synd_d[7:0]),
                     .ecl_ecc_sel_rs1_m_l(ecl_ecc_sel_rs1_m_l),
                     .ecl_ecc_sel_rs2_m_l(ecl_ecc_sel_rs2_m_l),
                     .ecl_ecc_sel_rs3_m_l(ecl_ecc_sel_rs3_m_l),
                     .ecl_ecc_log_rs1_m (ecl_ecc_log_rs1_m),
                     .ecl_ecc_log_rs2_m (ecl_ecc_log_rs2_m),
                     .ecl_ecc_log_rs3_m (ecl_ecc_log_rs3_m));
   
   sparc_exu_ecl ecl(
                     .so                (short_so0),
                     .si                (short_scan0_1),
                     .rst_tri_en        (mux_drive_disable),
                     .byp_ecl_wrccr_data_w(byp_irf_rd_data_w[7:0]),
                     .alu_ecl_adder_out_31_e(exu_ifu_brpc_e[31]),
                     .byp_ecl_rd_data_3lsb_m(exu_tlu_wsr_data_m[2:0]),                     
                     .alu_ecl_adder_out_7_0_e(exu_ifu_brpc_e[7:0]),
                     .exu_ifu_regz_e    (exu_ifu_regz_e),
                     .ecl_alu_casa_e    (ecl_alu_casa_e),
                     .exu_ifu_err_synd_7_m (exu_ifu_err_synd_m[7]),
                     /*AUTOINST*/
                     // Outputs
                     .ecl_byp_ecc_mask_m_l(ecl_byp_ecc_mask_m_l[7:0]),
                     .ecl_byp_eclpr_e   (ecl_byp_eclpr_e[7:0]),
                     .ecl_byp_sel_load_g(ecl_byp_sel_load_g),
                     .ecl_byp_sel_load_m(ecl_byp_sel_load_m),
                     .ecl_byp_sel_muldiv_g(ecl_byp_sel_muldiv_g),
                     .ecl_byp_sel_pipe_m(ecl_byp_sel_pipe_m),
                     .ecl_byp_sel_restore_g(ecl_byp_sel_restore_g),
                     .ecl_byp_sel_restore_m(ecl_byp_sel_restore_m),
                     .ecl_div_almostlast_cycle(ecl_div_almostlast_cycle),
                     .ecl_div_cin       (ecl_div_cin),
                     .ecl_div_dividend_sign(ecl_div_dividend_sign),
                     .ecl_div_keep_d    (ecl_div_keep_d),
                     .ecl_div_keepx     (ecl_div_keepx),
                     .ecl_div_last_cycle(ecl_div_last_cycle),
                     .ecl_div_mul_get_32bit_data(ecl_div_mul_get_32bit_data),
                     .ecl_div_mul_get_new_data(ecl_div_mul_get_new_data),
                     .ecl_div_mul_keep_data(ecl_div_mul_keep_data),
                     .ecl_div_mul_sext_rs1_e(ecl_div_mul_sext_rs1_e),
                     .ecl_div_mul_sext_rs2_e(ecl_div_mul_sext_rs2_e),
                     .ecl_div_newq      (ecl_div_newq),
                     .ecl_div_sel_64b   (ecl_div_sel_64b),
                     .ecl_div_sel_adder (ecl_div_sel_adder),
                     .ecl_div_sel_neg32 (ecl_div_sel_neg32),
                     .ecl_div_sel_pos32 (ecl_div_sel_pos32),
                     .ecl_div_sel_u32   (ecl_div_sel_u32),
                     .ecl_div_subtract_l(ecl_div_subtract_l),
                     .ecl_div_upper32_zero(ecl_div_upper32_zero),
                     .ecl_div_upper33_one(ecl_div_upper33_one),
                     .ecl_div_upper33_zero(ecl_div_upper33_zero),
                     .ecl_div_xinmask   (ecl_div_xinmask),
                     .ecl_div_yreg_shift_g(ecl_div_yreg_shift_g[3:0]),
                     .ecl_div_yreg_wen_g(ecl_div_yreg_wen_g[3:0]),
                     .ecl_div_yreg_wen_l(ecl_div_yreg_wen_l[3:0]),
                     .ecl_div_yreg_wen_w(ecl_div_yreg_wen_w[3:0]),
                     .ecl_ecc_log_rs1_m (ecl_ecc_log_rs1_m),
                     .ecl_ecc_log_rs2_m (ecl_ecc_log_rs2_m),
                     .ecl_ecc_log_rs3_m (ecl_ecc_log_rs3_m),
                     .ecl_ecc_sel_rs1_m_l(ecl_ecc_sel_rs1_m_l),
                     .ecl_ecc_sel_rs2_m_l(ecl_ecc_sel_rs2_m_l),
                     .ecl_ecc_sel_rs3_m_l(ecl_ecc_sel_rs3_m_l),
                     .ecl_rml_canrestore_wen_w(ecl_rml_canrestore_wen_w),
                     .ecl_rml_cansave_wen_w(ecl_rml_cansave_wen_w),
                     .ecl_rml_cleanwin_wen_w(ecl_rml_cleanwin_wen_w),
                     .ecl_rml_cwp_wen_e (ecl_rml_cwp_wen_e),
                     .ecl_rml_otherwin_wen_w(ecl_rml_otherwin_wen_w),
                     .ecl_rml_wstate_wen_w(ecl_rml_wstate_wen_w),
                     .exu_ffu_wsr_inst_e(exu_ffu_wsr_inst_e),
                     .exu_ifu_ecc_ce_m  (exu_ifu_ecc_ce_m),
                     .exu_ifu_ecc_ue_m  (exu_ifu_ecc_ue_m),
                     .exu_ifu_err_reg_m (exu_ifu_err_reg_m[7:0]),
                     .exu_ifu_inj_ack   (exu_ifu_inj_ack),
                     .exu_ifu_longop_done_g(exu_ifu_longop_done_g[3:0]),
                     .exu_mul_input_vld (exu_mul_input_vld),
                     .exu_tlu_ccr0_w    (exu_tlu_ccr0_w[7:0]),
                     .exu_tlu_ccr1_w    (exu_tlu_ccr1_w[7:0]),
                     .exu_tlu_ccr2_w    (exu_tlu_ccr2_w[7:0]),
                     .exu_tlu_ccr3_w    (exu_tlu_ccr3_w[7:0]),
                     .ecl_byp_sel_alu_e (ecl_byp_sel_alu_e),
                     .ecl_byp_sel_eclpr_e(ecl_byp_sel_eclpr_e),
                     .ecl_byp_sel_yreg_e(ecl_byp_sel_yreg_e),
                     .ecl_byp_sel_ifusr_e(ecl_byp_sel_ifusr_e),
                     .ecl_byp_sel_ffusr_m(ecl_byp_sel_ffusr_m),
                     .ecl_byp_sel_ifex_m(ecl_byp_sel_ifex_m),
                     .ecl_byp_sel_tlusr_m(ecl_byp_sel_tlusr_m),
                     .exu_ifu_va_oor_m  (exu_ifu_va_oor_m),
                     .ecl_alu_out_sel_sum_e_l(ecl_alu_out_sel_sum_e_l),
                     .ecl_alu_out_sel_rs3_e_l(ecl_alu_out_sel_rs3_e_l),
                     .ecl_alu_out_sel_shift_e_l(ecl_alu_out_sel_shift_e_l),
                     .ecl_alu_out_sel_logic_e_l(ecl_alu_out_sel_logic_e_l),
                     .ecl_alu_log_sel_and_e(ecl_alu_log_sel_and_e),
                     .ecl_alu_log_sel_or_e(ecl_alu_log_sel_or_e),
                     .ecl_alu_log_sel_xor_e(ecl_alu_log_sel_xor_e),
                     .ecl_alu_log_sel_move_e(ecl_alu_log_sel_move_e),
                     .ecl_alu_sethi_inst_e(ecl_alu_sethi_inst_e),
                     .ecl_alu_cin_e     (ecl_alu_cin_e),
                     .ecl_shft_lshift_e_l(ecl_shft_lshift_e_l),
                     .ecl_shft_op32_e   (ecl_shft_op32_e),
                     .ecl_shft_shift4_e (ecl_shft_shift4_e[3:0]),
                     .ecl_shft_shift1_e (ecl_shft_shift1_e[3:0]),
                     .ecl_shft_enshift_e_l(ecl_shft_enshift_e_l),
                     .ecl_byp_restore_m (ecl_byp_restore_m),
                     .ecl_byp_rs1_mux2_sel_e(ecl_byp_rs1_mux2_sel_e),
                     .ecl_byp_rs1_mux2_sel_rf(ecl_byp_rs1_mux2_sel_rf),
                     .ecl_byp_rs1_mux2_sel_ld(ecl_byp_rs1_mux2_sel_ld),
                     .ecl_byp_rs1_mux2_sel_usemux1(ecl_byp_rs1_mux2_sel_usemux1),
                     .ecl_byp_rs1_mux1_sel_m(ecl_byp_rs1_mux1_sel_m),
                     .ecl_byp_rs1_mux1_sel_w(ecl_byp_rs1_mux1_sel_w),
                     .ecl_byp_rs1_mux1_sel_w2(ecl_byp_rs1_mux1_sel_w2),
                     .ecl_byp_rs1_mux1_sel_other(ecl_byp_rs1_mux1_sel_other),
                     .ecl_byp_rcc_mux2_sel_e(ecl_byp_rcc_mux2_sel_e),
                     .ecl_byp_rcc_mux2_sel_rf(ecl_byp_rcc_mux2_sel_rf),
                     .ecl_byp_rcc_mux2_sel_ld(ecl_byp_rcc_mux2_sel_ld),
                     .ecl_byp_rcc_mux2_sel_usemux1(ecl_byp_rcc_mux2_sel_usemux1),
                     .ecl_byp_rcc_mux1_sel_m(ecl_byp_rcc_mux1_sel_m),
                     .ecl_byp_rcc_mux1_sel_w(ecl_byp_rcc_mux1_sel_w),
                     .ecl_byp_rcc_mux1_sel_w2(ecl_byp_rcc_mux1_sel_w2),
                     .ecl_byp_rcc_mux1_sel_other(ecl_byp_rcc_mux1_sel_other),
                     .ecl_byp_rs2_mux2_sel_e(ecl_byp_rs2_mux2_sel_e),
                     .ecl_byp_rs2_mux2_sel_rf(ecl_byp_rs2_mux2_sel_rf),
                     .ecl_byp_rs2_mux2_sel_ld(ecl_byp_rs2_mux2_sel_ld),
                     .ecl_byp_rs2_mux2_sel_usemux1(ecl_byp_rs2_mux2_sel_usemux1),
                     .ecl_byp_rs2_mux1_sel_m(ecl_byp_rs2_mux1_sel_m),
                     .ecl_byp_rs2_mux1_sel_w(ecl_byp_rs2_mux1_sel_w),
                     .ecl_byp_rs2_mux1_sel_w2(ecl_byp_rs2_mux1_sel_w2),
                     .ecl_byp_rs2_mux1_sel_other(ecl_byp_rs2_mux1_sel_other),
                     .ecl_byp_rs3_mux2_sel_e(ecl_byp_rs3_mux2_sel_e),
                     .ecl_byp_rs3_mux2_sel_rf(ecl_byp_rs3_mux2_sel_rf),
                     .ecl_byp_rs3_mux2_sel_ld(ecl_byp_rs3_mux2_sel_ld),
                     .ecl_byp_rs3_mux2_sel_usemux1(ecl_byp_rs3_mux2_sel_usemux1),
                     .ecl_byp_rs3_mux1_sel_m(ecl_byp_rs3_mux1_sel_m),
                     .ecl_byp_rs3_mux1_sel_w(ecl_byp_rs3_mux1_sel_w),
                     .ecl_byp_rs3_mux1_sel_w2(ecl_byp_rs3_mux1_sel_w2),
                     .ecl_byp_rs3_mux1_sel_other(ecl_byp_rs3_mux1_sel_other),
                     .ecl_byp_rs3h_mux2_sel_e(ecl_byp_rs3h_mux2_sel_e),
                     .ecl_byp_rs3h_mux2_sel_rf(ecl_byp_rs3h_mux2_sel_rf),
                     .ecl_byp_rs3h_mux2_sel_ld(ecl_byp_rs3h_mux2_sel_ld),
                     .ecl_byp_rs3h_mux2_sel_usemux1(ecl_byp_rs3h_mux2_sel_usemux1),
                     .ecl_byp_rs3h_mux1_sel_m(ecl_byp_rs3h_mux1_sel_m),
                     .ecl_byp_rs3h_mux1_sel_w(ecl_byp_rs3h_mux1_sel_w),
                     .ecl_byp_rs3h_mux1_sel_w2(ecl_byp_rs3h_mux1_sel_w2),
                     .ecl_byp_rs3h_mux1_sel_other(ecl_byp_rs3h_mux1_sel_other),
                     .ecl_byp_rs1_longmux_sel_g2(ecl_byp_rs1_longmux_sel_g2),
                     .ecl_byp_rs1_longmux_sel_w2(ecl_byp_rs1_longmux_sel_w2),
                     .ecl_byp_rs1_longmux_sel_ldxa(ecl_byp_rs1_longmux_sel_ldxa),
                     .ecl_byp_rs2_longmux_sel_g2(ecl_byp_rs2_longmux_sel_g2),
                     .ecl_byp_rs2_longmux_sel_w2(ecl_byp_rs2_longmux_sel_w2),
                     .ecl_byp_rs2_longmux_sel_ldxa(ecl_byp_rs2_longmux_sel_ldxa),
                     .ecl_byp_rs3_longmux_sel_g2(ecl_byp_rs3_longmux_sel_g2),
                     .ecl_byp_rs3_longmux_sel_w2(ecl_byp_rs3_longmux_sel_w2),
                     .ecl_byp_rs3_longmux_sel_ldxa(ecl_byp_rs3_longmux_sel_ldxa),
                     .ecl_byp_rs3h_longmux_sel_g2(ecl_byp_rs3h_longmux_sel_g2),
                     .ecl_byp_rs3h_longmux_sel_w2(ecl_byp_rs3h_longmux_sel_w2),
                     .ecl_byp_rs3h_longmux_sel_ldxa(ecl_byp_rs3h_longmux_sel_ldxa),
                     .ecl_byp_std_e_l   (ecl_byp_std_e_l),
                     .ecl_byp_ldxa_g    (ecl_byp_ldxa_g),
                     .ecl_byp_3lsb_m    (ecl_byp_3lsb_m[2:0]),
                     .ecl_ecc_rs1_use_rf_e(ecl_ecc_rs1_use_rf_e),
                     .ecl_ecc_rs2_use_rf_e(ecl_ecc_rs2_use_rf_e),
                     .ecl_ecc_rs3_use_rf_e(ecl_ecc_rs3_use_rf_e),
                     .ecl_irf_rd_m      (ecl_irf_rd_m[4:0]),
                     .ecl_irf_tid_m     (ecl_irf_tid_m[1:0]),
                     .ecl_irf_wen_w     (ecl_irf_wen_w),
                     .ecl_irf_wen_w2    (ecl_irf_wen_w2),
                     .ecl_irf_rd_g      (ecl_irf_rd_g[4:0]),
                     .ecl_irf_tid_g     (ecl_irf_tid_g[1:0]),
                     .ecl_div_thr_e     (ecl_div_thr_e[3:0]),
                     .ecl_rml_thr_m     (ecl_rml_thr_m[3:0]),
                     .ecl_rml_thr_w     (ecl_rml_thr_w[3:0]),
                     .ecl_rml_xor_data_e(ecl_rml_xor_data_e[2:0]),
                     .ecl_div_ld_inputs (ecl_div_ld_inputs),
                     .ecl_div_sel_div   (ecl_div_sel_div),
                     .ecl_div_div64     (ecl_div_div64),
                     .exu_ifu_cc_d      (exu_ifu_cc_d[7:0]),
                     .ecl_shft_extendbit_e(ecl_shft_extendbit_e),
                     .ecl_shft_extend32bit_e_l(ecl_shft_extend32bit_e_l),
                     .ecl_div_zero_rs2_e(ecl_div_zero_rs2_e),
                     .ecl_div_muls_rs1_31_e_l(ecl_div_muls_rs1_31_e_l),
                     .ecl_div_yreg_data_31_g(ecl_div_yreg_data_31_g),
                     .exu_tlu_va_oor_m  (exu_tlu_va_oor_m),
                     .exu_tlu_va_oor_jl_ret_m(exu_tlu_va_oor_jl_ret_m),
                     .ecl_rml_kill_e    (ecl_rml_kill_e),
                     .ecl_rml_kill_w    (ecl_rml_kill_w),
                     .ecl_byp_sel_ecc_m (ecl_byp_sel_ecc_m),
                     .exu_tlu_ttype_m   (exu_tlu_ttype_m[8:0]),
                     .exu_tlu_ttype_vld_m(exu_tlu_ttype_vld_m),
                     .exu_tlu_ue_trap_m (exu_tlu_ue_trap_m),
                     .exu_tlu_misalign_addr_jmpl_rtn_m(exu_tlu_misalign_addr_jmpl_rtn_m),
                     .exu_lsu_priority_trap_m(exu_lsu_priority_trap_m),
                     .ecl_div_mul_wen   (ecl_div_mul_wen),
                     .ecl_div_muls      (ecl_div_muls),
                     .ecl_rml_early_flush_w(ecl_rml_early_flush_w),
                     .ecl_rml_inst_vld_w(ecl_rml_inst_vld_w),
                     // Inputs
                     .div_ecl_adder_out_31(div_ecl_adder_out_31),
                     .div_ecl_cout32    (div_ecl_cout32),
                     .div_ecl_cout64    (div_ecl_cout64),
                     .div_ecl_d_62      (div_ecl_d_62),
                     .div_ecl_d_msb     (div_ecl_d_msb),
                     .div_ecl_detect_zero_high(div_ecl_detect_zero_high),
                     .div_ecl_detect_zero_low(div_ecl_detect_zero_low),
                     .div_ecl_dividend_msb(div_ecl_dividend_msb),
                     .div_ecl_gencc_in_31(div_ecl_gencc_in_31),
                     .div_ecl_gencc_in_msb_l(div_ecl_gencc_in_msb_l),
                     .div_ecl_low32_nonzero(div_ecl_low32_nonzero),
                     .div_ecl_upper32_equal(div_ecl_upper32_equal),
                     .div_ecl_x_msb     (div_ecl_x_msb),
                     .div_ecl_xin_msb_l (div_ecl_xin_msb_l),
                     .ecc_ecl_rs1_ce    (ecc_ecl_rs1_ce),
                     .ecc_ecl_rs1_ue    (ecc_ecl_rs1_ue),
                     .ecc_ecl_rs2_ce    (ecc_ecl_rs2_ce),
                     .ecc_ecl_rs2_ue    (ecc_ecl_rs2_ue),
                     .ecc_ecl_rs3_ce    (ecc_ecl_rs3_ce),
                     .ecc_ecl_rs3_ue    (ecc_ecl_rs3_ue),
                     .ifu_exu_disable_ce_e(ifu_exu_disable_ce_e),
                     .ifu_exu_ecc_mask  (ifu_exu_ecc_mask[7:0]),
                     .ifu_exu_inj_irferr(ifu_exu_inj_irferr),
                     .ifu_exu_inst_vld_e(ifu_exu_inst_vld_e),
                     .ifu_exu_inst_vld_w(ifu_exu_inst_vld_w),
                     .ifu_exu_muldivop_d(ifu_exu_muldivop_d[4:0]),
                     .ifu_exu_return_d  (ifu_exu_return_d),
                     .ifu_tlu_sraddr_d  (ifu_tlu_sraddr_d[6:0]),
                     .ifu_tlu_wsr_inst_d(ifu_tlu_wsr_inst_d),
                     .lsu_exu_ldst_miss_g2(lsu_exu_ldst_miss_g2),
                     .mul_exu_ack       (mul_exu_ack),
                     .rml_ecl_canrestore_d(rml_ecl_canrestore_d[2:0]),
                     .rml_ecl_cansave_d (rml_ecl_cansave_d[2:0]),
                     .rml_ecl_cleanwin_d(rml_ecl_cleanwin_d[2:0]),
                     .rml_ecl_cwp_d     (rml_ecl_cwp_d[2:0]),
                     .rml_ecl_gl_e      (rml_ecl_gl_e[1:0]),
                     .rml_ecl_kill_m    (rml_ecl_kill_m),
                     .rml_ecl_otherwin_d(rml_ecl_otherwin_d[2:0]),
                     .rml_ecl_rmlop_done_e(rml_ecl_rmlop_done_e),
                     .rml_ecl_swap_done (rml_ecl_swap_done[3:0]),
                     .rml_ecl_wstate_d  (rml_ecl_wstate_d[5:0]),
                     .sehold            (sehold),
                     .tlu_exu_ccr_m     (tlu_exu_ccr_m[7:0]),
                     .tlu_exu_cwpccr_update_m(tlu_exu_cwpccr_update_m),
                     .rclk              (rclk),
                     .se                (se),
                     .grst_l            (grst_l),
                     .arst_l            (arst_l),
                     .ifu_exu_dbrinst_d (ifu_exu_dbrinst_d),
                     .ifu_exu_aluop_d   (ifu_exu_aluop_d[2:0]),
                     .ifu_exu_shiftop_d (ifu_exu_shiftop_d[2:0]),
                     .ifu_exu_invert_d  (ifu_exu_invert_d),
                     .ifu_exu_usecin_d  (ifu_exu_usecin_d),
                     .ifu_exu_enshift_d (ifu_exu_enshift_d),
                     .byp_ecl_rs2_3_0_e (byp_ecl_rs2_3_0_e[3:0]),
                     .byp_ecl_rs1_2_0_e (byp_ecl_rs1_2_0_e[2:0]),
                     .ifu_exu_use_rsr_e_l(ifu_exu_use_rsr_e_l),
                     .ifu_exu_rd_exusr_e(ifu_exu_rd_exusr_e),
                     .ifu_exu_rd_ifusr_e(ifu_exu_rd_ifusr_e),
                     .ifu_exu_rd_ffusr_e(ifu_exu_rd_ffusr_e),
                     .ifu_exu_rs1_vld_d (ifu_exu_rs1_vld_d),
                     .ifu_exu_rs2_vld_d (ifu_exu_rs2_vld_d),
                     .ifu_exu_rs3e_vld_d(ifu_exu_rs3e_vld_d),
                     .ifu_exu_rs3o_vld_d(ifu_exu_rs3o_vld_d),
                     .ifu_exu_dontmv_regz0_e(ifu_exu_dontmv_regz0_e),
                     .ifu_exu_dontmv_regz1_e(ifu_exu_dontmv_regz1_e),
                     .ifu_exu_rd_d      (ifu_exu_rd_d[4:0]),
                     .ifu_exu_tid_s2    (ifu_exu_tid_s2[1:0]),
                     .ifu_exu_kill_e    (ifu_exu_kill_e),
                     .ifu_exu_wen_d     (ifu_exu_wen_d),
                     .ifu_exu_ialign_d  (ifu_exu_ialign_d),
                     .alu_ecl_add_n64_e (alu_ecl_add_n64_e),
                     .alu_ecl_add_n32_e (alu_ecl_add_n32_e),
                     .alu_ecl_log_n64_e (alu_ecl_log_n64_e),
                     .alu_ecl_log_n32_e (alu_ecl_log_n32_e),
                     .alu_ecl_zhigh_e   (alu_ecl_zhigh_e),
                     .alu_ecl_zlow_e    (alu_ecl_zlow_e),
                     .ifu_exu_setcc_d   (ifu_exu_setcc_d),
                     .lsu_exu_dfill_vld_g(lsu_exu_dfill_vld_g),
                     .lsu_exu_rd_m      (lsu_exu_rd_m[4:0]),
                     .lsu_exu_thr_m     (lsu_exu_thr_m[1:0]),
                     .lsu_exu_ldxa_m    (lsu_exu_ldxa_m),
                     .byp_ecl_rs1_31_e  (byp_ecl_rs1_31_e),
                     .byp_ecl_rs2_31_e  (byp_ecl_rs2_31_e),
                     .byp_ecl_rs1_63_e  (byp_ecl_rs1_63_e),
                     .alu_ecl_cout64_e_l(alu_ecl_cout64_e_l),
                     .alu_ecl_cout32_e  (alu_ecl_cout32_e),
                     .alu_ecl_adder_out_63_e(alu_ecl_adder_out_63_e),
                     .alu_ecl_adderin2_63_e(alu_ecl_adderin2_63_e),
                     .alu_ecl_adderin2_31_e(alu_ecl_adderin2_31_e),
                     .ifu_exu_rs1_s     (ifu_exu_rs1_s[4:0]),
                     .ifu_exu_rs2_s     (ifu_exu_rs2_s[4:0]),
                     .ifu_exu_rs3_s     (ifu_exu_rs3_s[4:0]),
                     .ifu_exu_tagop_d   (ifu_exu_tagop_d),
                     .ifu_exu_tv_d      (ifu_exu_tv_d),
                     .ifu_exu_muls_d    (ifu_exu_muls_d),
                     .div_ecl_yreg_0_l  (div_ecl_yreg_0_l[3:0]),
                     .alu_ecl_mem_addr_invalid_e_l(alu_ecl_mem_addr_invalid_e_l),
                     .ifu_exu_range_check_jlret_d(ifu_exu_range_check_jlret_d),
                     .ifu_exu_range_check_other_d(ifu_exu_range_check_other_d),
                     .ifu_exu_addr_mask_d(ifu_exu_addr_mask_d),
                     .ifu_exu_save_d    (ifu_exu_save_d),
                     .ifu_exu_restore_d (ifu_exu_restore_d),
                     .ifu_exu_casa_d    (ifu_exu_casa_d),
                     .rml_ecl_clean_window_e(rml_ecl_clean_window_e),
                     .rml_ecl_fill_e    (rml_ecl_fill_e),
                     .rml_ecl_other_e   (rml_ecl_other_e),
                     .rml_ecl_wtype_e   (rml_ecl_wtype_e[2:0]),
                     .ifu_exu_tcc_e     (ifu_exu_tcc_e),
                     .ifu_exu_useimm_d  (ifu_exu_useimm_d),
                     .ifu_exu_nceen_e   (ifu_exu_nceen_e),
                     .ifu_tlu_flush_m   (ifu_tlu_flush_m),
                     .ifu_exu_ttype_vld_m(ifu_exu_ttype_vld_m),
                     .tlu_exu_priv_trap_m(tlu_exu_priv_trap_m),
                     .tlu_exu_pic_onebelow_m(tlu_exu_pic_onebelow_m),
                     .tlu_exu_pic_twobelow_m(tlu_exu_pic_twobelow_m),
                     .lsu_exu_flush_pipe_w(lsu_exu_flush_pipe_w),
                     .ifu_exu_sethi_inst_d(ifu_exu_sethi_inst_d),
                     .lsu_exu_st_dtlb_perr_g(lsu_exu_st_dtlb_perr_g));
   
   sparc_exu_alu alu(
                     .byp_alu_rs3_data_e(exu_lsu_rs3_data_e[63:0]),
                     .so                (scan0_2),
                     .si                (scan0_1),
                     .ifu_lsu_casa_e (ecl_alu_casa_e),
                     /*AUTOINST*/
                     // Outputs
                     .alu_byp_rd_data_e (alu_byp_rd_data_e[63:0]),
                     .exu_ifu_brpc_e    (exu_ifu_brpc_e[47:0]),
                     .exu_lsu_ldst_va_e (exu_lsu_ldst_va_e[47:0]),
                     .exu_lsu_early_va_e(exu_lsu_early_va_e[(6 + 4):3]),
                     .exu_mmu_early_va_e(exu_mmu_early_va_e[7:0]),
                     .alu_ecl_add_n64_e (alu_ecl_add_n64_e),
                     .alu_ecl_add_n32_e (alu_ecl_add_n32_e),
                     .alu_ecl_log_n64_e (alu_ecl_log_n64_e),
                     .alu_ecl_log_n32_e (alu_ecl_log_n32_e),
                     .alu_ecl_zhigh_e   (alu_ecl_zhigh_e),
                     .alu_ecl_zlow_e    (alu_ecl_zlow_e),
                     .exu_ifu_regz_e    (exu_ifu_regz_e),
                     .exu_ifu_regn_e    (exu_ifu_regn_e),
                     .alu_ecl_adderin2_63_e(alu_ecl_adderin2_63_e),
                     .alu_ecl_adderin2_31_e(alu_ecl_adderin2_31_e),
                     .alu_ecl_adder_out_63_e(alu_ecl_adder_out_63_e),
                     .alu_ecl_cout32_e  (alu_ecl_cout32_e),
                     .alu_ecl_cout64_e_l(alu_ecl_cout64_e_l),
                     .alu_ecl_mem_addr_invalid_e_l(alu_ecl_mem_addr_invalid_e_l),
                     // Inputs
                     .rclk              (rclk),
                     .se                (se),
                     .byp_alu_rs1_data_e(byp_alu_rs1_data_e[63:0]),
                     .byp_alu_rs2_data_e_l(byp_alu_rs2_data_e_l[63:0]),
                     .byp_alu_rcc_data_e(byp_alu_rcc_data_e[63:0]),
                     .ecl_alu_cin_e     (ecl_alu_cin_e),
                     .ifu_exu_invert_d  (ifu_exu_invert_d),
                     .ecl_alu_log_sel_and_e(ecl_alu_log_sel_and_e),
                     .ecl_alu_log_sel_or_e(ecl_alu_log_sel_or_e),
                     .ecl_alu_log_sel_xor_e(ecl_alu_log_sel_xor_e),
                     .ecl_alu_log_sel_move_e(ecl_alu_log_sel_move_e),
                     .ecl_alu_out_sel_sum_e_l(ecl_alu_out_sel_sum_e_l),
                     .ecl_alu_out_sel_rs3_e_l(ecl_alu_out_sel_rs3_e_l),
                     .ecl_alu_out_sel_shift_e_l(ecl_alu_out_sel_shift_e_l),
                     .ecl_alu_out_sel_logic_e_l(ecl_alu_out_sel_logic_e_l),
                     .shft_alu_shift_out_e(shft_alu_shift_out_e[63:0]),
                     .ecl_alu_sethi_inst_e(ecl_alu_sethi_inst_e));
   sparc_exu_shft shft(/*AUTOINST*/
                       // Outputs
                       .shft_alu_shift_out_e(shft_alu_shift_out_e[63:0]),
                       // Inputs
                       .ecl_shft_lshift_e_l(ecl_shft_lshift_e_l),
                       .ecl_shft_op32_e (ecl_shft_op32_e),
                       .ecl_shft_shift4_e(ecl_shft_shift4_e[3:0]),
                       .ecl_shft_shift1_e(ecl_shft_shift1_e[3:0]),
                       .byp_alu_rs1_data_e(byp_alu_rs1_data_e[63:0]),
                       .byp_alu_rs2_data_e(byp_alu_rs2_data_e[5:4]),
                       .ecl_shft_enshift_e_l(ecl_shft_enshift_e_l),
                       .ecl_shft_extendbit_e(ecl_shft_extendbit_e),
                       .ecl_shft_extend32bit_e_l(ecl_shft_extend32bit_e_l));

   sparc_exu_div div(
                     .so                (scan0_3),
                     .si                (scan0_2),
                     .byp_div_rs1_data_e(byp_alu_rs1_data_e[63:0]),
                     .byp_div_rs2_data_e(byp_alu_rs2_data_e[63:0]),
                     .byp_div_yreg_data_w(byp_irf_rd_data_w[31:0]),
                     /*AUTOINST*/
                     // Outputs
                     .div_ecl_xin_msb_l (div_ecl_xin_msb_l),
                     .div_ecl_x_msb     (div_ecl_x_msb),
                     .div_ecl_d_msb     (div_ecl_d_msb),
                     .div_ecl_cout64    (div_ecl_cout64),
                     .div_ecl_cout32    (div_ecl_cout32),
                     .div_ecl_gencc_in_msb_l(div_ecl_gencc_in_msb_l),
                     .div_ecl_gencc_in_31(div_ecl_gencc_in_31),
                     .div_ecl_upper32_equal(div_ecl_upper32_equal),
                     .div_ecl_low32_nonzero(div_ecl_low32_nonzero),
                     .div_ecl_dividend_msb(div_ecl_dividend_msb),
                     .div_byp_muldivout_g(div_byp_muldivout_g[63:0]),
                     .div_byp_yreg_e    (div_byp_yreg_e[31:0]),
                     .div_ecl_yreg_0_l  (div_ecl_yreg_0_l[3:0]),
                     .exu_mul_rs1_data  (exu_mul_rs1_data[63:0]),
                     .exu_mul_rs2_data  (exu_mul_rs2_data[63:0]),
                     .div_ecl_adder_out_31(div_ecl_adder_out_31),
                     .div_ecl_detect_zero_low(div_ecl_detect_zero_low),
                     .div_ecl_detect_zero_high(div_ecl_detect_zero_high),
                     .div_ecl_d_62      (div_ecl_d_62),
                     // Inputs
                     .ecl_div_thr_e     (ecl_div_thr_e[3:0]),
                     .ecl_div_yreg_data_31_g(ecl_div_yreg_data_31_g),
                     .ecl_div_yreg_shift_g(ecl_div_yreg_shift_g[3:0]),
                     .ecl_div_yreg_wen_g(ecl_div_yreg_wen_g[3:0]),
                     .ecl_div_yreg_wen_l(ecl_div_yreg_wen_l[3:0]),
                     .ecl_div_yreg_wen_w(ecl_div_yreg_wen_w[3:0]),
                     .rclk              (rclk),
                     .se                (se),
                     .ecl_div_keep_d    (ecl_div_keep_d),
                     .ecl_div_ld_inputs (ecl_div_ld_inputs),
                     .ecl_div_sel_adder (ecl_div_sel_adder),
                     .ecl_div_last_cycle(ecl_div_last_cycle),
                     .ecl_div_almostlast_cycle(ecl_div_almostlast_cycle),
                     .ecl_div_div64     (ecl_div_div64),
                     .ecl_div_sel_u32   (ecl_div_sel_u32),
                     .ecl_div_sel_pos32 (ecl_div_sel_pos32),
                     .ecl_div_sel_neg32 (ecl_div_sel_neg32),
                     .ecl_div_sel_64b   (ecl_div_sel_64b),
                     .ecl_div_upper32_zero(ecl_div_upper32_zero),
                     .ecl_div_upper33_one(ecl_div_upper33_one),
                     .ecl_div_upper33_zero(ecl_div_upper33_zero),
                     .mul_exu_data_g    (mul_exu_data_g[63:0]),
                     .ecl_div_sel_div   (ecl_div_sel_div),
                     .ecl_div_mul_wen   (ecl_div_mul_wen),
                     .ecl_div_dividend_sign(ecl_div_dividend_sign),
                     .ecl_div_subtract_l(ecl_div_subtract_l),
                     .ecl_div_cin       (ecl_div_cin),
                     .ecl_div_newq      (ecl_div_newq),
                     .ecl_div_xinmask   (ecl_div_xinmask),
                     .ecl_div_keepx     (ecl_div_keepx),
                     .ecl_div_mul_get_new_data(ecl_div_mul_get_new_data),
                     .ecl_div_mul_keep_data(ecl_div_mul_keep_data),
                     .ecl_div_mul_get_32bit_data(ecl_div_mul_get_32bit_data),
                     .ecl_div_mul_sext_rs2_e(ecl_div_mul_sext_rs2_e),
                     .ecl_div_mul_sext_rs1_e(ecl_div_mul_sext_rs1_e),
                     .ecl_div_muls_rs1_31_e_l(ecl_div_muls_rs1_31_e_l),
                     .ecl_div_muls      (ecl_div_muls),
                     .ecl_div_zero_rs2_e(ecl_div_zero_rs2_e));

   sparc_exu_rml rml(
                     .so                (so0),
                     .si                (scan0_3),
                  .rst_tri_en        (mux_drive_disable),
                     .exu_tlu_wsr_data_w(byp_irf_rd_data_w[5:0]),
                     .rml_irf_old_e_cwp_e(rml_irf_old_e_cwp_e[1:0]),
                     .rml_irf_new_e_cwp_e(rml_irf_new_e_cwp_e[1:0]),
                     /*AUTOINST*/
                     // Outputs
                     .exu_tlu_cwp0_w    (exu_tlu_cwp0_w[2:0]),
                     .exu_tlu_cwp1_w    (exu_tlu_cwp1_w[2:0]),
                     .exu_tlu_cwp2_w    (exu_tlu_cwp2_w[2:0]),
                     .exu_tlu_cwp3_w    (exu_tlu_cwp3_w[2:0]),
                     .exu_tlu_cwp_retry (exu_tlu_cwp_retry),
                     .exu_tlu_spill_other(exu_tlu_spill_other),
                     .exu_tlu_spill_wtype(exu_tlu_spill_wtype[2:0]),
                     .exu_tlu_cwp_cmplt (exu_tlu_cwp_cmplt),
                     .exu_tlu_cwp_cmplt_tid(exu_tlu_cwp_cmplt_tid[1:0]),
                     .rml_ecl_cwp_d     (rml_ecl_cwp_d[2:0]),
                     .rml_ecl_cansave_d (rml_ecl_cansave_d[2:0]),
                     .rml_ecl_canrestore_d(rml_ecl_canrestore_d[2:0]),
                     .rml_ecl_otherwin_d(rml_ecl_otherwin_d[2:0]),
                     .rml_ecl_wstate_d  (rml_ecl_wstate_d[5:0]),
                     .rml_ecl_cleanwin_d(rml_ecl_cleanwin_d[2:0]),
                     .rml_ecl_fill_e    (rml_ecl_fill_e),
                     .rml_ecl_clean_window_e(rml_ecl_clean_window_e),
                     .rml_ecl_other_e   (rml_ecl_other_e),
                     .rml_ecl_wtype_e   (rml_ecl_wtype_e[2:0]),
                     .exu_ifu_spill_e   (exu_ifu_spill_e),
                     .rml_ecl_gl_e      (rml_ecl_gl_e[1:0]),
                     .rml_irf_old_lo_cwp_e(rml_irf_old_lo_cwp_e[2:0]),
                     .rml_irf_new_lo_cwp_e(rml_irf_new_lo_cwp_e[2:0]),
                     .rml_irf_swap_even_e(rml_irf_swap_even_e),
                     .rml_irf_swap_odd_e(rml_irf_swap_odd_e),
                     .rml_irf_swap_local_e(rml_irf_swap_local_e),
                     .rml_irf_kill_restore_w(rml_irf_kill_restore_w),
                     .rml_irf_cwpswap_tid_e(rml_irf_cwpswap_tid_e[1:0]),
                     .rml_ecl_swap_done (rml_ecl_swap_done[3:0]),
                     .rml_ecl_rmlop_done_e(rml_ecl_rmlop_done_e),
                     .exu_ifu_oddwin_s  (exu_ifu_oddwin_s[3:0]),
                     .exu_tlu_spill     (exu_tlu_spill),
                     .exu_tlu_spill_tid (exu_tlu_spill_tid[1:0]),
                     .rml_ecl_kill_m    (rml_ecl_kill_m),
                     .rml_irf_old_agp   (rml_irf_old_agp[1:0]),
                     .rml_irf_new_agp   (rml_irf_new_agp[1:0]),
                     .rml_irf_swap_global(rml_irf_swap_global),
                     .rml_irf_global_tid(rml_irf_global_tid[1:0]),
                     // Inputs
                     .rclk              (rclk),
                     .se                (se),
                     .grst_l            (grst_l),
                     .arst_l            (arst_l),
                     .ifu_exu_tid_s2    (ifu_exu_tid_s2[1:0]),
                     .ifu_exu_save_d    (ifu_exu_save_d),
                     .ifu_exu_restore_d (ifu_exu_restore_d),
                     .ifu_exu_saved_e   (ifu_exu_saved_e),
                     .ifu_exu_restored_e(ifu_exu_restored_e),
                     .ifu_exu_flushw_e  (ifu_exu_flushw_e),
                     .ecl_rml_thr_m     (ecl_rml_thr_m[3:0]),
                     .ecl_rml_thr_w     (ecl_rml_thr_w[3:0]),
                     .ecl_rml_cwp_wen_e (ecl_rml_cwp_wen_e),
                     .ecl_rml_cansave_wen_w(ecl_rml_cansave_wen_w),
                     .ecl_rml_canrestore_wen_w(ecl_rml_canrestore_wen_w),
                     .ecl_rml_otherwin_wen_w(ecl_rml_otherwin_wen_w),
                     .ecl_rml_wstate_wen_w(ecl_rml_wstate_wen_w),
                     .ecl_rml_cleanwin_wen_w(ecl_rml_cleanwin_wen_w),
                     .ecl_rml_xor_data_e(ecl_rml_xor_data_e[2:0]),
                     .ecl_rml_kill_e    (ecl_rml_kill_e),
                     .ecl_rml_kill_w    (ecl_rml_kill_w),
                     .ecl_rml_early_flush_w(ecl_rml_early_flush_w),
                     .tlu_exu_agp       (tlu_exu_agp[1:0]),
                     .tlu_exu_agp_swap  (tlu_exu_agp_swap),
                     .tlu_exu_agp_tid   (tlu_exu_agp_tid[1:0]),
                     .tlu_exu_cwp_m     (tlu_exu_cwp_m[2:0]),
                     .tlu_exu_cwpccr_update_m(tlu_exu_cwpccr_update_m),
                     .ecl_rml_inst_vld_w(ecl_rml_inst_vld_w),
                     .tlu_exu_cwp_retry_m(tlu_exu_cwp_retry_m));
endmodule // sparc_exu
// Local Variables:
// verilog-library-directories:("." "../../../srams/rtl")
// End:
// Modified by Princeton University on June 9th, 2015
// ========== Copyright Header Begin ==========================================
//
// OpenSPARC T1 Processor File: sparc_exu_alu.v
// Copyright (c) 2006 Sun Microsystems, Inc.  All Rights Reserved.
// DO NOT ALTER OR REMOVE COPYRIGHT NOTICES.
//
// The above named program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public
// License version 2 as published by the Free Software Foundation.
//
// The above named program is distributed in the hope that it will be
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// General Public License for more details.
//
// You should have received a copy of the GNU General Public
// License along with this work; if not, write to the Free Software
// Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA 02110-1301, USA.
//
// ========== Copyright Header End ============================================
////////////////////////////////////////////////////////////////////////
/*
//  Module Name: sparc_exu_alu
*/

module sparc_exu_alu
(
 /*AUTOARG*/
   // Outputs
   so, alu_byp_rd_data_e, exu_ifu_brpc_e, exu_lsu_ldst_va_e,
   exu_lsu_early_va_e, exu_mmu_early_va_e, alu_ecl_add_n64_e,
   alu_ecl_add_n32_e, alu_ecl_log_n64_e, alu_ecl_log_n32_e,
   alu_ecl_zhigh_e, alu_ecl_zlow_e, exu_ifu_regz_e, exu_ifu_regn_e,
   alu_ecl_adderin2_63_e, alu_ecl_adderin2_31_e,
   alu_ecl_adder_out_63_e, alu_ecl_cout32_e, alu_ecl_cout64_e_l,
   alu_ecl_mem_addr_invalid_e_l,
   // Inputs
   rclk, se, si, byp_alu_rs1_data_e, byp_alu_rs2_data_e_l,
   byp_alu_rs3_data_e, byp_alu_rcc_data_e, ecl_alu_cin_e,
   ifu_exu_invert_d, ecl_alu_log_sel_and_e, ecl_alu_log_sel_or_e,
   ecl_alu_log_sel_xor_e, ecl_alu_log_sel_move_e,
   ecl_alu_out_sel_sum_e_l, ecl_alu_out_sel_rs3_e_l,
   ecl_alu_out_sel_shift_e_l, ecl_alu_out_sel_logic_e_l,
   shft_alu_shift_out_e, ecl_alu_sethi_inst_e, ifu_lsu_casa_e
   );
   input rclk;
   input se;
   input si;
   input [63:0] byp_alu_rs1_data_e;   // source operand 1
   input [63:0] byp_alu_rs2_data_e_l;  // source operand 2
   input [63:0] byp_alu_rs3_data_e;  // source operand 3
   input [63:0] byp_alu_rcc_data_e;  // source operand for reg condition codes
   input        ecl_alu_cin_e;            // cin for adder
   input        ifu_exu_invert_d;
   input  ecl_alu_log_sel_and_e;// These 4 wires are select lines for the logic
   input  ecl_alu_log_sel_or_e;// block mux.  They are active high and choose the
   input  ecl_alu_log_sel_xor_e;// output they describe.
   input  ecl_alu_log_sel_move_e;
   input  ecl_alu_out_sel_sum_e_l;// The following 4 are select lines for
   input  ecl_alu_out_sel_rs3_e_l;// the output stage mux.  They are active high
   input  ecl_alu_out_sel_shift_e_l;// and choose the output of the respective block.
   input  ecl_alu_out_sel_logic_e_l;
   input [63:0] shft_alu_shift_out_e;// result from shifter
   input        ecl_alu_sethi_inst_e;
   input        ifu_lsu_casa_e;

   output       so;
   output [63:0] alu_byp_rd_data_e;          // alu result
   output [47:0] exu_ifu_brpc_e;// branch pc output
   output [47:0] exu_lsu_ldst_va_e; // address for lsu
   output [(6 + 4):3] exu_lsu_early_va_e; // faster bits for cache
   output [7:0]  exu_mmu_early_va_e;
   output        alu_ecl_add_n64_e;
   output        alu_ecl_add_n32_e;
   output        alu_ecl_log_n64_e;
   output        alu_ecl_log_n32_e;
   output        alu_ecl_zhigh_e;
   output        alu_ecl_zlow_e;
   output    exu_ifu_regz_e;              // rs1_data == 0
   output    exu_ifu_regn_e;
   output    alu_ecl_adderin2_63_e;
   output    alu_ecl_adderin2_31_e;
   output    alu_ecl_adder_out_63_e;
   output    alu_ecl_cout32_e;       // To ecl of sparc_exu_ecl.v
   output    alu_ecl_cout64_e_l;       // To ecl of sparc_exu_ecl.v
   output    alu_ecl_mem_addr_invalid_e_l;// adder_out[63:48] not all 1 or all 0

   wire         clk;
   wire [63:0] logic_out;       // result of logic block
   wire [63:0] adder_out;       // result of adder
   wire [63:0] spr_out;         // result of sum predict
   wire [63:0] zcomp_in;        // result going to zcompare
   wire [63:0] va_e;            // complete va
   wire [63:0] byp_alu_rs2_data_e;
   wire        invert_e;
   wire        ecl_alu_out_sel_sum_e;
   wire        ecl_alu_out_sel_rs3_e;
   wire        ecl_alu_out_sel_shift_e;
   wire        ecl_alu_out_sel_logic_e;
   assign      clk = rclk;
   assign      byp_alu_rs2_data_e[63:0] = ~byp_alu_rs2_data_e_l[63:0];
   assign      ecl_alu_out_sel_sum_e = ~ecl_alu_out_sel_sum_e_l;
   assign      ecl_alu_out_sel_rs3_e = ~ecl_alu_out_sel_rs3_e_l;
   assign      ecl_alu_out_sel_shift_e = ~ecl_alu_out_sel_shift_e_l;
   assign      ecl_alu_out_sel_logic_e = ~ecl_alu_out_sel_logic_e_l;

   // Zero comparison for exu_ifu_regz_e
   // Tri: adds dummy output zero32 to get rid of the vcs warning
   sparc_exu_aluzcmp64 regzcmp(.in(byp_alu_rcc_data_e[63:0]), .zero64(exu_ifu_regz_e), .zero32());
   assign     exu_ifu_regn_e = byp_alu_rcc_data_e[63];

   // mux between adder output and rs1 (for casa) for lsu va
   dp_mux2es #(64)  lsu_va_mux(.dout(va_e[63:0]),
                               .in0(adder_out[63:0]),
                               .in1(byp_alu_rs1_data_e[63:0]),
                               .sel(ifu_lsu_casa_e));
   assign     exu_lsu_ldst_va_e[47:0] = va_e[47:0];
   // for bits 10:4 we have a separate bus that is not used for cas
   assign     exu_lsu_early_va_e[(6 + 4):3] = adder_out[(6 + 4):3];
   // mmu needs bits 7:0
   assign     exu_mmu_early_va_e[7:0] = adder_out[7:0];


   // Adder
   assign     exu_ifu_brpc_e[47:0] = adder_out[47:0];
   assign     alu_ecl_adder_out_63_e = adder_out[63];
   sparc_exu_aluaddsub addsub(.adder_out(adder_out[63:0]),
                              /*AUTOINST*/
                              // Outputs
                              .spr_out  (spr_out[63:0]),
                              .alu_ecl_cout64_e_l(alu_ecl_cout64_e_l),
                              .alu_ecl_cout32_e(alu_ecl_cout32_e),
                              .alu_ecl_adderin2_63_e(alu_ecl_adderin2_63_e),
                              .alu_ecl_adderin2_31_e(alu_ecl_adderin2_31_e),
                              // Inputs
                              .clk      (clk),
                              .se       (se),
                              .byp_alu_rs1_data_e(byp_alu_rs1_data_e[63:0]),
                              .byp_alu_rs2_data_e(byp_alu_rs2_data_e[63:0]),
                              .ecl_alu_cin_e(ecl_alu_cin_e),
                              .ifu_exu_invert_d(ifu_exu_invert_d));

   // Logic/pass rs2_data
   dff_s invert_d2e(.din(ifu_exu_invert_d), .clk(clk), .q(invert_e), .se(se), .si(), .so());
   sparc_exu_alulogic logic_mod(.rs1_data(byp_alu_rs1_data_e[63:0]),
                            .rs2_data(byp_alu_rs2_data_e[63:0]),
                            .isand(ecl_alu_log_sel_and_e),
                            .isor(ecl_alu_log_sel_or_e),
                            .isxor(ecl_alu_log_sel_xor_e),
                            .pass_rs2_data(ecl_alu_log_sel_move_e),
                            .inv_logic(invert_e), .logic_out(logic_out[63:0]),
                            .ifu_exu_sethi_inst_e(ecl_alu_sethi_inst_e));

   // Mux between sum predict and logic outputs for zcc
   dp_mux2es #(64)  zcompmux(.dout(zcomp_in[63:0]),
                           .in0(logic_out[63:0]),
                           .in1(spr_out[63:0]),
                           .sel(ecl_alu_out_sel_sum_e));

   // Zero comparison for zero cc
//   sparc_exu_aluzcmp64 zcccmp(.in(zcomp_in[63:0]), .zero64(alu_ecl_z64_e),
//                          .zero32(alu_ecl_z32_e));
   assign        alu_ecl_zlow_e = ~(|zcomp_in[31:0]);
   assign        alu_ecl_zhigh_e = ~(|zcomp_in[63:32]);

   // Get Negative ccs
   assign   alu_ecl_add_n64_e = adder_out[63];
   assign   alu_ecl_add_n32_e = adder_out[31];
   assign   alu_ecl_log_n64_e = logic_out[63];
   assign   alu_ecl_log_n32_e = logic_out[31];


   // Mux for output
   mux4ds #(64) output_mux(.dout(alu_byp_rd_data_e[63:0]),
                         .in0(adder_out[63:0]),
                         .in1(byp_alu_rs3_data_e[63:0]),
                         .in2(shft_alu_shift_out_e[63:0]),
                         .in3(logic_out[63:0]),
                         .sel0(ecl_alu_out_sel_sum_e),
                         .sel1(ecl_alu_out_sel_rs3_e),
                         .sel2(ecl_alu_out_sel_shift_e),
                         .sel3(ecl_alu_out_sel_logic_e));

   // memory address checks
   sparc_exu_alu_16eql chk_mem_addr(.equal(alu_ecl_mem_addr_invalid_e_l),
                                    .in(va_e[63:47]));

endmodule  // sparc_exu_alu
// Modified by Princeton University on June 9th, 2015
// ========== Copyright Header Begin ==========================================
// 
// OpenSPARC T1 Processor File: sparc_exu_alu_16eql.v
// Copyright (c) 2006 Sun Microsystems, Inc.  All Rights Reserved.
// DO NOT ALTER OR REMOVE COPYRIGHT NOTICES.
// 
// The above named program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public
// License version 2 as published by the Free Software Foundation.
// 
// The above named program is distributed in the hope that it will be 
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// General Public License for more details.
// 
// You should have received a copy of the GNU General Public
// License along with this work; if not, write to the Free Software
// Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA 02110-1301, USA.
// 
// ========== Copyright Header End ============================================
////////////////////////////////////////////////////////////////////////
/*
//  Module Name: sparc_exu_alu_16eql
//	Description: Takes a 17 bit input and generates an active low output
//			signifying that all 17 bits have the same value.
*/
module sparc_exu_alu_16eql (/*AUTOARG*/
   // Outputs
   equal, 
   // Inputs
   in
   ) ;
   input [16:0] in;

   output       equal;

   wire [15:0]  inxor;
   wire [7:0]   nor1;
   wire [1:0]   nand2;

   assign inxor[0] = in[15] ^ in[14];
   assign inxor[1] = in[14] ^ in[13];
   assign inxor[2] = in[13] ^ in[12];
   assign inxor[3] = in[12] ^ in[11];
   assign inxor[4] = in[11] ^ in[10];
   assign inxor[5] = in[10] ^ in[9];
   assign inxor[6] = in[9] ^ in[8];
   assign inxor[7] = in[8] ^ in[7];
   assign inxor[8] = in[7] ^ in[6];
   assign inxor[9] = in[6] ^ in[5];
   assign inxor[10] = in[5] ^ in[4];
   assign inxor[11] = in[4] ^ in[3];
   assign inxor[12] = in[3] ^ in[2];
   assign inxor[13] = in[2] ^ in[1];
   assign inxor[14] = in[1] ^ in[0];
   assign inxor[15] = in[16] ^ in[15];

   assign nor1[0] = ~(inxor[15] | inxor[14]);
   assign       nor1[1] = ~(inxor[1] | inxor[0]);
   assign       nor1[2] = ~(inxor[3] | inxor[2]);
   assign       nor1[3] = ~(inxor[5] | inxor[4]);
   assign       nor1[4] = ~(inxor[7] | inxor[6]);
   assign       nor1[5] = ~(inxor[9] | inxor[8]);
   assign       nor1[6] = ~(inxor[11] | inxor[10]);
   assign       nor1[7] = ~(inxor[13] | inxor[12]);

   assign       nand2[0] = ~(nor1[1] & nor1[2] & nor1[3] & nor1[4]);
   assign       nand2[1] = ~(nor1[5] & nor1[6] & nor1[7] & nor1[0]);

   assign       equal = ~(nand2[1] | nand2[0]);
   
endmodule // sparc_exu_div_32eql
// Modified by Princeton University on June 9th, 2015
// ========== Copyright Header Begin ==========================================
// 
// OpenSPARC T1 Processor File: sparc_exu_aluadder64.v
// Copyright (c) 2006 Sun Microsystems, Inc.  All Rights Reserved.
// DO NOT ALTER OR REMOVE COPYRIGHT NOTICES.
// 
// The above named program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public
// License version 2 as published by the Free Software Foundation.
// 
// The above named program is distributed in the hope that it will be 
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// General Public License for more details.
// 
// You should have received a copy of the GNU General Public
// License along with this work; if not, write to the Free Software
// Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA 02110-1301, USA.
// 
// ========== Copyright Header End ============================================
////////////////////////////////////////////////////////////////////////
/*
//  Module Name: sparc_exu_aluadder64
//	Description:		This block implements the adder for the sparc alu.
//            It takes two operands and a carry bit.  It adds them together
//						and sends the output to adder_out.  It outputs the overflow
//						and carry condition codes for both 64 bit and 32 bit operations.
*/

module sparc_exu_aluadder64
  (
   rs1_data,
   rs2_data,
   cin,
   adder_out,
   cout32,
   cout64
   );

   input [63:0]  rs1_data;   // 1st input operand
   input [63:0]  rs2_data;   // 2nd input operand
   input         cin;           // carry in

   output [63:0] adder_out; // result of adder
   output         cout32;         // Cout from lower 32 bit add
   output         cout64;         // cout from 64 bit add


////////////////////////////////////////////
//  Module implementation
////////////////////////////////////////////

   assign      {cout32, adder_out[31:0]} = rs1_data[31:0]+rs2_data[31:0]+
                                           cin;
   assign      {cout64, adder_out[63:32]} = rs1_data[63:32] 
               + rs2_data[63:32] + cout32;

endmodule // sparc_exu_aluadder64




// Modified by Princeton University on June 9th, 2015
// ========== Copyright Header Begin ==========================================
// 
// OpenSPARC T1 Processor File: sparc_exu_aluaddsub.v
// Copyright (c) 2006 Sun Microsystems, Inc.  All Rights Reserved.
// DO NOT ALTER OR REMOVE COPYRIGHT NOTICES.
// 
// The above named program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public
// License version 2 as published by the Free Software Foundation.
// 
// The above named program is distributed in the hope that it will be 
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// General Public License for more details.
// 
// You should have received a copy of the GNU General Public
// License along with this work; if not, write to the Free Software
// Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA 02110-1301, USA.
// 
// ========== Copyright Header End ============================================
////////////////////////////////////////////////////////////////////////
/*
//  Module Name: sparc_exu_aluaddsub
//	Description:		This block implements addition and subtraction.
//            It takes two operands, a carry_in, plus two control signals
//            (subtract and use_cin).  If subtract is high, then rs2_data
//            is subtracted from rs1_data.  If use_cin is high, then
//            carry_in is added to the sum (addition) or subtracted from
//            the result (subtraction).  It outputs the result of the 
//            specified operation.  To keep the cin calculation from
//	      being in the critical path, it is moved into the d-stage.
//	      All other calculations are in the e-stage.
*/

module sparc_exu_aluaddsub
  (/*AUTOARG*/
   // Outputs
   adder_out, spr_out, alu_ecl_cout64_e_l, alu_ecl_cout32_e, 
   alu_ecl_adderin2_63_e, alu_ecl_adderin2_31_e, 
   // Inputs
   clk, se, byp_alu_rs1_data_e, byp_alu_rs2_data_e, ecl_alu_cin_e, 
   ifu_exu_invert_d
   );
   input clk;
   input se;
   input [63:0] byp_alu_rs1_data_e;   // 1st input operand
   input [63:0]  byp_alu_rs2_data_e;   // 2nd input operand
   input         ecl_alu_cin_e;           // carry in
   input         ifu_exu_invert_d;     // subtract used by adder

   output [63:0] adder_out; // result of adder
   output [63:0] spr_out;   // result of sum predict
   output         alu_ecl_cout64_e_l;
   output         alu_ecl_cout32_e;
   output       alu_ecl_adderin2_63_e;
   output       alu_ecl_adderin2_31_e;
   
   wire [63:0]  rs2_data;       // 2nd input to adder
   wire [63:0]  rs1_data;       // 1st input to adder
   wire [63:0]  subtract_d;
   wire [63:0]  subtract_e;
   wire         cout64_e;
   
////////////////////////////////////////////
//  Module implementation
////////////////////////////////////////////
   assign       subtract_d[63:0] = {64{ifu_exu_invert_d}};
   dff_s #(64) sub_dff(.din(subtract_d[63:0]), .clk(clk), .q(subtract_e[63:0]), .se(se),
                     .si(), .so());

   assign       rs1_data[63:0] = byp_alu_rs1_data_e[63:0];

   assign       rs2_data[63:0] = byp_alu_rs2_data_e[63:0] ^ subtract_e[63:0];
   
   assign      alu_ecl_adderin2_63_e = rs2_data[63];
   assign      alu_ecl_adderin2_31_e = rs2_data[31];
   sparc_exu_aluadder64 adder(.rs1_data(rs1_data[63:0]), .rs2_data(rs2_data[63:0]),
                              .cin(ecl_alu_cin_e), .adder_out(adder_out[63:0]),
                              .cout32(alu_ecl_cout32_e), .cout64(cout64_e));
   assign      alu_ecl_cout64_e_l = ~cout64_e;


   // sum predict
   sparc_exu_aluspr spr(.rs1_data(rs1_data[63:0]), .rs2_data(rs2_data[63:0]), .cin(ecl_alu_cin_e),
                        .spr_out(spr_out[63:0]));

endmodule // sparc_exu_aluaddsub




// Modified by Princeton University on June 9th, 2015
// ========== Copyright Header Begin ==========================================
// 
// OpenSPARC T1 Processor File: sparc_exu_alulogic.v
// Copyright (c) 2006 Sun Microsystems, Inc.  All Rights Reserved.
// DO NOT ALTER OR REMOVE COPYRIGHT NOTICES.
// 
// The above named program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public
// License version 2 as published by the Free Software Foundation.
// 
// The above named program is distributed in the hope that it will be 
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// General Public License for more details.
// 
// You should have received a copy of the GNU General Public
// License along with this work; if not, write to the Free Software
// Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA 02110-1301, USA.
// 
// ========== Copyright Header End ============================================
////////////////////////////////////////////////////////////////////////
/*
//
//  Module Name: sparc_exu_alulogic
//	Description: This block implements and, or, xor, xnor, nand, nor
//		and pass_rs2_data.  And, or, Xor and pass are muxed together
//		and then xored with an inversion signal to create
//		xnor, nand and nor.  Both inputs are buffered before being
//		used and the rs2_data signal is buffered again before going
//		to the mux.
*/

module sparc_exu_alulogic (/*AUTOARG*/
   // Outputs
   logic_out, 
   // Inputs
   rs1_data, rs2_data, isand, isor, isxor, pass_rs2_data, inv_logic, 
   ifu_exu_sethi_inst_e
   );

input [63:0] rs1_data;             // 1st input operand
input [63:0] rs2_data;             // 2nd input operand
input isand;
input isor;
input isxor;
input pass_rs2_data;
input inv_logic;
   input ifu_exu_sethi_inst_e;       // zero out top half of rs2 on mov

output [63:0] logic_out;      // output of logic block

wire [63:0] rs1_data_bf1;                 // buffered rs1_data
wire [63:0] rs2_data_bf1;                 // buffered rs2_data
   wire [63:0] mov_data;
wire [63:0] result_and;              // rs1_data & rs2_data
wire [63:0] result_or;               // rs1_data | rs2_data
wire [63:0] result_xor;              // rs1_data ^ rs2_data
wire [63:0] rs2_xor_invert;           // output of mux between various results


// mux between various results
   mux4ds #(64) logic_mux(.dout(logic_out[63:0]),
                        .in0(result_and[63:0]), 
                        .in1(result_or[63:0]),
                        .in2(result_xor[63:0]), 
                        .in3(mov_data[63:0]), 
                        .sel0(isand),
                        .sel1(isor), 
                        .sel2(isxor),
                        .sel3(pass_rs2_data));

// buffer inputs
dp_buffer #(64) rs1_data_buf(.dout(rs1_data_bf1[63:0]), .in(rs1_data[63:0]));
dp_buffer #(64) rs2_data_buf(.dout(rs2_data_bf1[63:0]), .in(rs2_data[63:0]));

   // zero out top of rs2 for sethi_inst
  assign   mov_data[63:32] = rs2_data_bf1[63:32] & {32{~ifu_exu_sethi_inst_e}};
   dp_buffer #(32) rs2_data_buf2(.dout(mov_data[31:0]), .in(rs2_data_bf1[31:0]));

// invert input2 for andn, orn, xnor
assign rs2_xor_invert[63:0] = rs2_data_bf1[63:0] ^ {64{inv_logic}};
   
// do boolean ops
assign result_and = rs1_data_bf1 & rs2_xor_invert;
assign result_or = rs1_data_bf1 | rs2_xor_invert;
assign result_xor = rs1_data_bf1 ^ rs2_xor_invert;

endmodule

// Modified by Princeton University on June 9th, 2015
// ========== Copyright Header Begin ==========================================
// 
// OpenSPARC T1 Processor File: sparc_exu_aluor32.v
// Copyright (c) 2006 Sun Microsystems, Inc.  All Rights Reserved.
// DO NOT ALTER OR REMOVE COPYRIGHT NOTICES.
// 
// The above named program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public
// License version 2 as published by the Free Software Foundation.
// 
// The above named program is distributed in the hope that it will be 
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// General Public License for more details.
// 
// You should have received a copy of the GNU General Public
// License along with this work; if not, write to the Free Software
// Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA 02110-1301, USA.
// 
// ========== Copyright Header End ============================================
////////////////////////////////////////////////////////////////////////
/*
//  Module Name: sparc_exu_aluor32
//	Description: This block performs a 32 bit OR of the input source.
//			The result is the output nonzero.
*/


module sparc_exu_aluor32
  (/*AUTOARG*/
   // Outputs
   out, 
   // Inputs
   in
   );

   input [31:0] in;         // input to be compared to zero

   output       out;       // or of input bits

   wire         nor1_1;
   wire         nor1_2;
   wire         nor1_3;
   wire         nor1_4;
   wire         nor1_5;
   wire         nor1_6;
   wire         nor1_7;
   wire         nor1_8;
   wire         nor1_9;
   wire         nor1_10;
   wire         nor1_11;
   wire         nor1_12;
   wire         nor1_13;
   wire         nor1_14;
   wire         nor1_15;
   wire         nor1_16;
   wire         nand2_1;
   wire         nand2_2;
   wire         nand2_3;
   wire         nand2_4;
   wire         inv3_1;
   wire         inv3_2;
   wire         inv3_3;
   wire         inv3_4;

   assign       nor1_1 = ~(in[1] | in[0]);
   assign       nor1_2 = ~(in[3] | in[2]);
   assign       nor1_3 = ~(in[5] | in[4]);
   assign       nor1_4 = ~(in[7] | in[6]);
   assign       nor1_5 = ~(in[9] | in[8]);
   assign       nor1_6 = ~(in[11] | in[10]);
   assign       nor1_7 = ~(in[13] | in[12]);
   assign       nor1_8 = ~(in[15] | in[14]);
   assign       nor1_9 = ~(in[17] | in[16]);
   assign       nor1_10 = ~(in[19] | in[18]);
   assign       nor1_11 = ~(in[21] | in[20]);
   assign       nor1_12 = ~(in[23] | in[22]);
   assign       nor1_13 = ~(in[25] | in[24]);
   assign       nor1_14 = ~(in[27] | in[26]);
   assign       nor1_15 = ~(in[29] | in[28]);
   assign       nor1_16 = ~(in[31] | in[30]);

   assign       nand2_1 = ~(nor1_1 & nor1_2 & nor1_3 & nor1_4);
   assign       nand2_2 = ~(nor1_5 & nor1_6 & nor1_7 & nor1_8);
   assign       nand2_3 = ~(nor1_9 & nor1_10 & nor1_11 & nor1_12);
   assign       nand2_4 = ~(nor1_13 & nor1_14 & nor1_15 & nor1_16);

   assign       inv3_1 = ~nand2_1;
   assign       inv3_2 = ~nand2_2;
   assign       inv3_3 = ~nand2_3;
   assign       inv3_4 = ~nand2_4;

   assign       out = ~(inv3_1 & inv3_2 & inv3_3 & inv3_4);

endmodule // sparc_exu_aluor32

   

   
                           

                            
                            
                          
// Modified by Princeton University on June 9th, 2015
// ========== Copyright Header Begin ==========================================
// 
// OpenSPARC T1 Processor File: sparc_exu_aluspr.v
// Copyright (c) 2006 Sun Microsystems, Inc.  All Rights Reserved.
// DO NOT ALTER OR REMOVE COPYRIGHT NOTICES.
// 
// The above named program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public
// License version 2 as published by the Free Software Foundation.
// 
// The above named program is distributed in the hope that it will be 
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// General Public License for more details.
// 
// You should have received a copy of the GNU General Public
// License along with this work; if not, write to the Free Software
// Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA 02110-1301, USA.
// 
// ========== Copyright Header End ============================================
////////////////////////////////////////////////////////////////////////
/*
//  Module Name: sparc_exu_aluspr
//	Description:		This block implements the sum predict for the sparc alu.
//            It takes two operands and produces the correct result if the
//            sum is zero.  If not, the output is undefined, but non-zero.
*/

module sparc_exu_aluspr(/*AUTOARG*/
   // Outputs
   spr_out, 
   // Inputs
   rs1_data, rs2_data, cin
   );

input [63:0] rs1_data;
input [63:0] rs2_data;
   input     cin;
output [63:0] spr_out;

wire [63:0] rs1_data_xor_rs2_data;
wire [62:0] rs1_data_or_rs2_data;
wire [63:0] shift_or;

assign rs1_data_xor_rs2_data[63:0] = rs1_data[63:0] ^ rs2_data[63:0];
assign rs1_data_or_rs2_data[62:0] = rs1_data[62:0] | rs2_data[62:0];
assign shift_or[63:0] = {rs1_data_or_rs2_data[62:0],cin};

assign spr_out[63:0] = rs1_data_xor_rs2_data[63:0] ^ shift_or[63:0];

endmodule  // sparc_exu_aluspr
// Modified by Princeton University on June 9th, 2015
// ========== Copyright Header Begin ==========================================
// 
// OpenSPARC T1 Processor File: sparc_exu_aluzcmp64.v
// Copyright (c) 2006 Sun Microsystems, Inc.  All Rights Reserved.
// DO NOT ALTER OR REMOVE COPYRIGHT NOTICES.
// 
// The above named program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public
// License version 2 as published by the Free Software Foundation.
// 
// The above named program is distributed in the hope that it will be 
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// General Public License for more details.
// 
// You should have received a copy of the GNU General Public
// License along with this work; if not, write to the Free Software
// Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA 02110-1301, USA.
// 
// ========== Copyright Header End ============================================
////////////////////////////////////////////////////////////////////////
/*
//
//  Module Name: sparc_exu_aluzcmp64
//	Description: This block determines if the input 'source' is zero.
//		It provides to outputs.  zero64 is high if all 64 bits
//		are zero, while zero32 is high if the lower 32 bits are
//		zero.  It uses 2 32 bit or gates and then NORs them.
*/

module sparc_exu_aluzcmp64
  (
   in,
   zero64,
   zero32
   );

   input [63:0] in;         // input operand

   output zero64;               // true if input is zero
   output zero32;               // true if lower 32 bits are zero

   wire   low_nonzero;					// low 32 is nonzero
   wire   high_nonzero;         // high 32 is nonzero

   // evaluate each half of the input
   sparc_exu_aluor32 lowcmp(.in(in[31:0]), .out(low_nonzero));
   sparc_exu_aluor32 highcmp(.in(in[63:32]), .out(high_nonzero));

   assign zero32 = ~low_nonzero;
   assign zero64 = ~(low_nonzero | high_nonzero);

endmodule // sparc_exu_aluzcmp64
   
// Modified by Princeton University on June 9th, 2015
// ========== Copyright Header Begin ==========================================
// 
// OpenSPARC T1 Processor File: sparc_exu_byp.v
// Copyright (c) 2006 Sun Microsystems, Inc.  All Rights Reserved.
// DO NOT ALTER OR REMOVE COPYRIGHT NOTICES.
// 
// The above named program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public
// License version 2 as published by the Free Software Foundation.
// 
// The above named program is distributed in the hope that it will be 
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// General Public License for more details.
// 
// You should have received a copy of the GNU General Public
// License along with this work; if not, write to the Free Software
// Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA 02110-1301, USA.
// 
// ========== Copyright Header End ============================================
////////////////////////////////////////////////////////////////////////
/*
//  Module Name: sparc_exu_byp
//	Description: This block includes the muxes for the bypassing for all
//		3 register outputs.  It also includes the pipeline registers 
//		for the output of the ALU.  All other operands come from
// 		outside the bypass block.  Rs1_data chooses between the normal
//		bypassing paths and the PC.  Rs2_data chooses between the normal
//		bypassing paths and the immediate.
*/

//PITON_PROTO enables all FPGA related modifications





module sparc_exu_byp
( /*AUTOARG*/
   // Outputs
   so, byp_alu_rs1_data_e, byp_alu_rs2_data_e_l, byp_alu_rs2_data_e, 
   exu_lsu_rs3_data_e, exu_spu_rs3_data_e, exu_lsu_rs2_data_e, 
   byp_alu_rcc_data_e, byp_irf_rd_data_w, exu_tlu_wsr_data_m, 
   byp_irf_rd_data_w2, byp_ecc_rs3_data_e, byp_ecc_rcc_data_e, 
   byp_ecl_rs2_31_e, byp_ecl_rs1_31_e, byp_ecl_rs1_63_e, 
   byp_ecl_rs1_2_0_e, byp_ecl_rs2_3_0_e, byp_ecc_rs1_synd_d, 
   byp_ecc_rs2_synd_d, byp_ecc_rs3_synd_d, 
   // Inputs
   rclk, se, si, sehold, ecl_byp_rs1_mux2_sel_e, 
   ecl_byp_rs1_mux2_sel_rf, ecl_byp_rs1_mux2_sel_ld, 
   ecl_byp_rs1_mux2_sel_usemux1, ecl_byp_rs1_mux1_sel_m, 
   ecl_byp_rs1_mux1_sel_w, ecl_byp_rs1_mux1_sel_w2, 
   ecl_byp_rs1_mux1_sel_other, ecl_byp_rcc_mux2_sel_e, 
   ecl_byp_rcc_mux2_sel_rf, ecl_byp_rcc_mux2_sel_ld, 
   ecl_byp_rcc_mux2_sel_usemux1, ecl_byp_rcc_mux1_sel_m, 
   ecl_byp_rcc_mux1_sel_w, ecl_byp_rcc_mux1_sel_w2, 
   ecl_byp_rcc_mux1_sel_other, ecl_byp_rs2_mux2_sel_e, 
   ecl_byp_rs2_mux2_sel_rf, ecl_byp_rs2_mux2_sel_ld, 
   ecl_byp_rs2_mux2_sel_usemux1, ecl_byp_rs2_mux1_sel_m, 
   ecl_byp_rs2_mux1_sel_w, ecl_byp_rs2_mux1_sel_w2, 
   ecl_byp_rs2_mux1_sel_other, ecl_byp_rs3_mux2_sel_e, 
   ecl_byp_rs3_mux2_sel_rf, ecl_byp_rs3_mux2_sel_ld, 
   ecl_byp_rs3_mux2_sel_usemux1, ecl_byp_rs3_mux1_sel_m, 
   ecl_byp_rs3_mux1_sel_w, ecl_byp_rs3_mux1_sel_w2, 
   ecl_byp_rs3_mux1_sel_other, ecl_byp_rs3h_mux2_sel_e, 
   ecl_byp_rs3h_mux2_sel_rf, ecl_byp_rs3h_mux2_sel_ld, 
   ecl_byp_rs3h_mux2_sel_usemux1, ecl_byp_rs3h_mux1_sel_m, 
   ecl_byp_rs3h_mux1_sel_w, ecl_byp_rs3h_mux1_sel_w2, 
   ecl_byp_rs3h_mux1_sel_other, ecl_byp_rs1_longmux_sel_g2, 
   ecl_byp_rs1_longmux_sel_w2, ecl_byp_rs1_longmux_sel_ldxa, 
   ecl_byp_rs2_longmux_sel_g2, ecl_byp_rs2_longmux_sel_w2, 
   ecl_byp_rs2_longmux_sel_ldxa, ecl_byp_rs3_longmux_sel_g2, 
   ecl_byp_rs3_longmux_sel_w2, ecl_byp_rs3_longmux_sel_ldxa, 
   ecl_byp_rs3h_longmux_sel_g2, ecl_byp_rs3h_longmux_sel_w2, 
   ecl_byp_rs3h_longmux_sel_ldxa, ecl_byp_sel_load_m, 
   ecl_byp_sel_pipe_m, ecl_byp_sel_ecc_m, ecl_byp_sel_muldiv_g, 
   ecl_byp_sel_load_g, ecl_byp_sel_restore_g, ecl_byp_std_e_l, 
   ecl_byp_ldxa_g, alu_byp_rd_data_e, ifu_exu_imm_data_d, 
   irf_byp_rs1_data_d_l, irf_byp_rs2_data_d_l, irf_byp_rs3_data_d_l, 
   irf_byp_rs3h_data_d_l, lsu_exu_dfill_data_g, lsu_exu_ldxa_data_g, 
   div_byp_muldivout_g, ecc_byp_ecc_result_m, ecl_byp_ecc_mask_m_l, 
   ifu_exu_pc_d, ecl_byp_3lsb_m, ecl_byp_restore_m, 
   ecl_byp_sel_restore_m, ecl_byp_eclpr_e, div_byp_yreg_e, 
   ifu_exu_pcver_e, tlu_exu_rsr_data_m, ffu_exu_rsr_data_m, 
   ecl_byp_sel_yreg_e, ecl_byp_sel_eclpr_e, ecl_byp_sel_ifusr_e, 
   ecl_byp_sel_alu_e, ecl_byp_sel_ifex_m, ecl_byp_sel_ffusr_m, 
   ecl_byp_sel_tlusr_m
   );

   input rclk;
   input se;                    // scan enable
   input si;
   input sehold;
   input ecl_byp_rs1_mux2_sel_e;// select lines for bypass muxes for rs1
   input ecl_byp_rs1_mux2_sel_rf;
   input ecl_byp_rs1_mux2_sel_ld;
   input ecl_byp_rs1_mux2_sel_usemux1;
   input ecl_byp_rs1_mux1_sel_m;
   input ecl_byp_rs1_mux1_sel_w;
   input ecl_byp_rs1_mux1_sel_w2;
   input ecl_byp_rs1_mux1_sel_other;
   input ecl_byp_rcc_mux2_sel_e;// select lines for bypass muxes for reg condition code
   input ecl_byp_rcc_mux2_sel_rf;
   input ecl_byp_rcc_mux2_sel_ld;
   input ecl_byp_rcc_mux2_sel_usemux1;
   input ecl_byp_rcc_mux1_sel_m;
   input ecl_byp_rcc_mux1_sel_w;
   input ecl_byp_rcc_mux1_sel_w2;
   input ecl_byp_rcc_mux1_sel_other;
   input ecl_byp_rs2_mux2_sel_e;// select lines for bypass muxes for rs2
   input ecl_byp_rs2_mux2_sel_rf;
   input ecl_byp_rs2_mux2_sel_ld;
   input ecl_byp_rs2_mux2_sel_usemux1;
   input ecl_byp_rs2_mux1_sel_m;
   input ecl_byp_rs2_mux1_sel_w;
   input ecl_byp_rs2_mux1_sel_w2;
   input ecl_byp_rs2_mux1_sel_other;
   input ecl_byp_rs3_mux2_sel_e;// select lines for bypass muxes for rs3
   input ecl_byp_rs3_mux2_sel_rf;
   input ecl_byp_rs3_mux2_sel_ld;
   input ecl_byp_rs3_mux2_sel_usemux1;
   input ecl_byp_rs3_mux1_sel_m;
   input ecl_byp_rs3_mux1_sel_w;
   input ecl_byp_rs3_mux1_sel_w2;
   input ecl_byp_rs3_mux1_sel_other;
   input ecl_byp_rs3h_mux2_sel_e;// select lines for bypass muxes for rs3 double
   input ecl_byp_rs3h_mux2_sel_rf;
   input ecl_byp_rs3h_mux2_sel_ld;
   input ecl_byp_rs3h_mux2_sel_usemux1;
   input ecl_byp_rs3h_mux1_sel_m;
   input ecl_byp_rs3h_mux1_sel_w;
   input ecl_byp_rs3h_mux1_sel_w2;
   input ecl_byp_rs3h_mux1_sel_other;
   input ecl_byp_rs1_longmux_sel_g2;
   input ecl_byp_rs1_longmux_sel_w2;
   input ecl_byp_rs1_longmux_sel_ldxa;
   input ecl_byp_rs2_longmux_sel_g2;
   input ecl_byp_rs2_longmux_sel_w2;
   input ecl_byp_rs2_longmux_sel_ldxa;
   input ecl_byp_rs3_longmux_sel_g2;
   input ecl_byp_rs3_longmux_sel_w2;
   input ecl_byp_rs3_longmux_sel_ldxa;
   input ecl_byp_rs3h_longmux_sel_g2;
   input ecl_byp_rs3h_longmux_sel_w2;
   input ecl_byp_rs3h_longmux_sel_ldxa;
   input ecl_byp_sel_load_m;        // m instruction uses load in w1 port
   input ecl_byp_sel_pipe_m;
   input ecl_byp_sel_ecc_m;
   input ecl_byp_sel_muldiv_g;
   input ecl_byp_sel_load_g;
   input ecl_byp_sel_restore_g;
   input ecl_byp_std_e_l;
   input ecl_byp_ldxa_g;
   input [63:0] alu_byp_rd_data_e;           // data from alu for bypass
   input [31:0] ifu_exu_imm_data_d;     // immediate
   input [71:0] irf_byp_rs1_data_d_l;  // RF rs1_data
   input [71:0] irf_byp_rs2_data_d_l;  // RF rs2_data
   input [71:0] irf_byp_rs3_data_d_l;  // RF rs3_data
   input [31:0] irf_byp_rs3h_data_d_l;// RF rs3 double data
   input [63:0] lsu_exu_dfill_data_g; // load data
   input [63:0] lsu_exu_ldxa_data_g;
   input [63:0] div_byp_muldivout_g;
   input [63:0] ecc_byp_ecc_result_m;// result from ecc
   input [7:0]  ecl_byp_ecc_mask_m_l;
   input [47:0]  ifu_exu_pc_d;
   input [2:0]   ecl_byp_3lsb_m;
   input         ecl_byp_restore_m;
   input         ecl_byp_sel_restore_m;
   input [7:0]   ecl_byp_eclpr_e;
   input [31:0]  div_byp_yreg_e;
   input [63:0]  ifu_exu_pcver_e;
   input [63:0]  tlu_exu_rsr_data_m;
   input [63:0]  ffu_exu_rsr_data_m;
   input         ecl_byp_sel_yreg_e;
   input         ecl_byp_sel_eclpr_e;
   input         ecl_byp_sel_ifusr_e;
   input         ecl_byp_sel_alu_e;
   input         ecl_byp_sel_ifex_m;
   input         ecl_byp_sel_ffusr_m;
   input         ecl_byp_sel_tlusr_m;
 
   output        so;
   output [63:0] byp_alu_rs1_data_e; // rs1_data operand for alu
   output [63:0] byp_alu_rs2_data_e_l; // rs2_data operand for alu
   output [63:0] byp_alu_rs2_data_e;
   output [63:0] exu_lsu_rs3_data_e; // rs3_data operand for lsu
   output [63:0] exu_spu_rs3_data_e;// rs3 data for spu
   output [63:0]  exu_lsu_rs2_data_e;
   output [63:0]  byp_alu_rcc_data_e;// data for reg condition codes
   output [71:0] byp_irf_rd_data_w;
   output [63:0] exu_tlu_wsr_data_m;          // data for writeback
   output [71:0] byp_irf_rd_data_w2;
   output [63:0] byp_ecc_rs3_data_e;
   output [63:0] byp_ecc_rcc_data_e;
   output        byp_ecl_rs2_31_e;
   output        byp_ecl_rs1_31_e;
   output        byp_ecl_rs1_63_e;
   output [2:0]  byp_ecl_rs1_2_0_e;
   output [3:0]  byp_ecl_rs2_3_0_e;
   output [7:0]  byp_ecc_rs1_synd_d;
   output [7:0]  byp_ecc_rs2_synd_d;
   output [7:0]  byp_ecc_rs3_synd_d;

   wire          clk;
   wire          sehold_clk;
   wire [63:0] irf_byp_rs1_data_d;  // RF rs1_data
   wire [63:0] irf_byp_rs2_data_d;  // RF rs2_data
   wire [63:0] irf_byp_rs3_data_d;  // RF rs3_data
   wire [31:0] irf_byp_rs3h_data_d;  // RF rs3_data double
   wire [63:0] byp_alu_rs1_data_d; // rs1 operand for alu
   wire [63:0] byp_alu_rcc_data_d; // rcc operand for alu
   wire [63:0] byp_alu_rs2_data_d; // rs2_data operand for alu
   wire [63:0]   rd_data_e;          // e stage rd_data
   wire [63:0]   rd_data_m;          // m stage non-load rd_data
   wire [63:0]   full_rd_data_m;          // m stage non-load rd_data including rdsr
   wire [63:0]   rd_data_g;
   wire [63:0]   byp_irf_rd_data_m;// m stage rd_data
   wire [63:0]   rs1_data_btwn_mux;  // intermediate net for rs1_data muxes
   wire [63:0]   rcc_data_btwn_mux;  // intermediate net for rs1_data muxes
   wire [63:0]   rs2_data_btwn_mux;  // intermediate net for rs2_data muxes
   wire [63:0]   rs3_data_btwn_mux;  // intermediate net for rs3_data muxes
   wire [31:0]   rs3h_data_btwn_mux;  // intermediate net for rs3h_data muxes
   wire [63:0]   rs3_data_d;
   wire [63:0]   rs3_data_e;
   wire [31:0]   rs3h_data_d;
   wire [31:0]   rs3h_data_e;
   wire [63:0]   restore_rd_data;
   wire [63:0]   restore_rd_data_next;
   wire [63:0]   dfill_data_g;
   wire [63:0]   dfill_data_g2;
   wire          ecl_byp_std_e;
   wire [7:0]    rd_synd_w_l;
   wire [7:0]    rd_synd_w2_l;

   assign        clk = rclk;


   clken_buf irf_write_clkbuf (	
                                .rclk   (clk),
                                .enb_l  (sehold),
                                .tmb_l  (~se),
                                .clk    (sehold_clk)
                                ) ;

   
   
   assign        byp_ecc_rs1_synd_d[7:0] = ~irf_byp_rs1_data_d_l[71:64];
   assign        byp_ecc_rs2_synd_d[7:0] = ~irf_byp_rs2_data_d_l[71:64];
   assign        byp_ecc_rs3_synd_d[7:0] = ~irf_byp_rs3_data_d_l[71:64];
   /////////////////////////////////////////
   // Load returns go straight into a flop after mux with ldxa_data
   /////////////////////////////////////////
   dp_mux2es #(64) dfill_data_mux (.dout(dfill_data_g[63:0]),
                                   .in0(lsu_exu_dfill_data_g[63:0]),
                                   .in1(lsu_exu_ldxa_data_g[63:0]),
                                   .sel(ecl_byp_ldxa_g));
   dff_s #(64) dfill_data_dff (.din(dfill_data_g[63:0]), .clk(clk),
                             .q(dfill_data_g2[63:0]), .se(se), .si(), .so());
   
   //////////////////////////////////////////////////
   // RD of PR or SR
   //////////////////////////////////////////////////
   
   // Mux outputs for rdpr/rdsr
   mux4ds #(64) ifu_exu_sr_mux(.dout(rd_data_e[63:0]),
                               .in0({32'b0, div_byp_yreg_e[31:0]}),
                               .in1({56'b0, ecl_byp_eclpr_e[7:0]}),
                               .in2(ifu_exu_pcver_e[63:0]),
                               .in3(alu_byp_rd_data_e[63:0]),
                               .sel0(ecl_byp_sel_yreg_e),
                               .sel1(ecl_byp_sel_eclpr_e),
                               .sel2(ecl_byp_sel_ifusr_e),
                               .sel3(ecl_byp_sel_alu_e));
   
   // mux in the rdsr data from ffu and tlu
   mux3ds #(64) sr_out_mux(.dout(full_rd_data_m[63:0]),
                           .in0({rd_data_m[63:3], ecl_byp_3lsb_m[2:0]}),
                           .in1(ffu_exu_rsr_data_m[63:0]),
                           .in2(tlu_exu_rsr_data_m[63:0]),
                           .sel0(ecl_byp_sel_ifex_m),
                           .sel1(ecl_byp_sel_ffusr_m),
                           .sel2(ecl_byp_sel_tlusr_m));
   
   // Pipeline registers for rd_data
   dff_s #(64) dff_rd_data_e2m(.din(rd_data_e[63:0]), .clk(clk), .q(rd_data_m[63:0]),
                           .se(se), .si(), .so());
   dp_buffer #(64) wsr_data_buf(.dout(exu_tlu_wsr_data_m[63:0]), .in(rd_data_m[63:0]));
   
   // Flop for storing result from restore
   dp_mux2es #(64) restore_buf_mux(.dout(restore_rd_data_next[63:0]),
                                   .in0(restore_rd_data[63:0]),
                                   .in1(rd_data_m[63:0]),
                                   .sel(ecl_byp_restore_m));
   dff_s #(64) dff_restore_buf(.din(restore_rd_data_next[63:0]),
                             .q(restore_rd_data[63:0]), .clk(clk),
                             .se(se), .si(), .so());
   // Mux for rd_data_m between ALU and load data and ECC result and restore result
   mux4ds #(64) rd_data_m_mux(.dout(byp_irf_rd_data_m[63:0]), 
                              .in0(full_rd_data_m[63:0]),
                              .in1(dfill_data_g2[63:0]),
                              .in2(ecc_byp_ecc_result_m[63:0]),
                              .in3(restore_rd_data[63:0]),
                              .sel0(ecl_byp_sel_pipe_m), 
                              .sel1(ecl_byp_sel_load_m),
                              .sel2(ecl_byp_sel_ecc_m),
                              .sel3(ecl_byp_sel_restore_m));




   dff_s #(64) dff_rd_data_m2w(.din(byp_irf_rd_data_m[63:0]), .clk(sehold_clk), .q(byp_irf_rd_data_w[63:0]),
                           .se(se), .si(), .so());


   // W2 flop




   dff_s #(64) dff_rd_data_g2w(.din(rd_data_g[63:0]), .clk(sehold_clk), .q(byp_irf_rd_data_w2[63:0]),
                           .se(se), .si(), .so());

   
   
   // D-E pipeline registers for rs_data
   dff_s #(64) rs1_data_dff(.din(byp_alu_rs1_data_d[63:0]), .clk(clk),
                        .q(byp_alu_rs1_data_e[63:0]), .se(se),
                        .si(), .so());
   dff_s #(64) rs2_data_dff(.din(byp_alu_rs2_data_d[63:0]), .clk(clk), 
                        .q(byp_alu_rs2_data_e[63:0]), .se(se),
                        .si(), .so());
   assign        byp_alu_rs2_data_e_l[63:0] = ~byp_alu_rs2_data_e[63:0];
   assign        byp_ecl_rs2_31_e = byp_alu_rs2_data_e[31];
   assign        byp_ecl_rs1_63_e = byp_alu_rs1_data_e[63];
   assign        byp_ecl_rs1_31_e = byp_alu_rs1_data_e[31];
   assign        byp_ecl_rs1_2_0_e[2:0] = byp_alu_rs1_data_e[2:0];
   assign        byp_ecl_rs2_3_0_e[3:0] = byp_alu_rs2_data_e[3:0];
   

   dff_s #(64) rs3_data_dff(.din(rs3_data_d[63:0]), .clk(clk), 
                        .q(rs3_data_e[63:0]), .se(se),
                        .si(), .so());
   dff_s #(32) rs3h_data_dff(.din(rs3h_data_d[31:0]), .clk(clk), 
                           .q(rs3h_data_e[31:0]), .se(se),
                           .si(), .so());
   dff_s #(64) rcc_data_dff(.din(byp_alu_rcc_data_d[63:0]), .clk(clk), 
                        .q(byp_alu_rcc_data_e[63:0]), .se(se),
                        .si(), .so());

   assign        ecl_byp_std_e = ~ecl_byp_std_e_l;
   dp_mux2es #(64) rs2_data_out_mux(.dout(exu_lsu_rs2_data_e[63:0]),
                                    .in0(byp_alu_rs2_data_e[63:0]),
                                    .in1(rs3_data_e[63:0]),
                                    .sel(ecl_byp_std_e));
   dp_mux2es #(64) rs3_data_out_mux(.dout(exu_lsu_rs3_data_e[63:0]),
                                    .in0(rs3_data_e[63:0]),
                                    .in1({32'b0,rs3h_data_e[31:0]}),
                                    .sel(ecl_byp_std_e));
   // part of rs3 goes to spu.  Buffer off to help timing/loading
   assign        exu_spu_rs3_data_e[63:0] = rs3_data_e[63:0];
   
   assign        byp_ecc_rs3_data_e[63:0] = rs3_data_e[63:0];
   assign        byp_ecc_rcc_data_e[63:0] = byp_alu_rcc_data_e[63:0];
   
   // Forwarding Muxes
   // Select lines are as follows:
   // mux1[M, W, W2, OTHER(optional)]
   // mux2[mux1, RF, E, LD]
   assign        irf_byp_rs1_data_d[63:0] = ~irf_byp_rs1_data_d_l[63:0];
   assign        irf_byp_rs2_data_d[63:0] = ~irf_byp_rs2_data_d_l[63:0];
   assign        irf_byp_rs3_data_d[63:0] = ~irf_byp_rs3_data_d_l[63:0];
   assign        irf_byp_rs3h_data_d[31:0] = ~irf_byp_rs3h_data_d_l[31:0];

/* -----\/----- EXCLUDED -----\/-----
   // the w2 bypass path is either what is being written that cycle
   // or the load result that will be written next cycle.
 -----/\----- EXCLUDED -----/\----- */
   wire [63:0]   rs1_data_w2;
   wire [63:0]   rs2_data_w2;
   wire [63:0]   rs3_data_w2;
   wire [31:0]   rs3h_data_w2;
   mux3ds #(64) rs1_w2_mux(.dout(rs1_data_w2[63:0]),
                           .in0(byp_irf_rd_data_w2[63:0]),
                           .in1(dfill_data_g2[63:0]),
                           .in2(lsu_exu_ldxa_data_g[63:0]),
                           .sel0(ecl_byp_rs1_longmux_sel_w2),
                           .sel1(ecl_byp_rs1_longmux_sel_g2),
                           .sel2(ecl_byp_rs1_longmux_sel_ldxa));
   mux3ds #(64) rs2_w2_mux(.dout(rs2_data_w2[63:0]),
                           .in0(byp_irf_rd_data_w2[63:0]),
                           .in1(dfill_data_g2[63:0]),
                           .in2(lsu_exu_ldxa_data_g[63:0]),
                           .sel0(ecl_byp_rs2_longmux_sel_w2),
                           .sel1(ecl_byp_rs2_longmux_sel_g2),
                           .sel2(ecl_byp_rs2_longmux_sel_ldxa));
   mux3ds #(64) rs3_w2_mux(.dout(rs3_data_w2[63:0]),
                           .in0(byp_irf_rd_data_w2[63:0]),
                           .in1(dfill_data_g2[63:0]),
                           .in2(lsu_exu_ldxa_data_g[63:0]),
                           .sel0(ecl_byp_rs3_longmux_sel_w2),
                           .sel1(ecl_byp_rs3_longmux_sel_g2),
                           .sel2(ecl_byp_rs3_longmux_sel_ldxa));
   mux3ds #(32) rs3h_w2_mux(.dout(rs3h_data_w2[31:0]),
                            .in0(byp_irf_rd_data_w2[31:0]),
                            .in1(dfill_data_g2[31:0]),
                            .in2(lsu_exu_ldxa_data_g[31:0]),
                            .sel0(ecl_byp_rs3h_longmux_sel_w2),
                            .sel1(ecl_byp_rs3h_longmux_sel_g2),
                            .sel2(ecl_byp_rs3h_longmux_sel_ldxa));
                              
   
   // rs1_data muxes: RF and E are critical paths
   mux4ds #(64) mux_rs1_data_1(.dout(rs1_data_btwn_mux[63:0]), 
                               .in0(rd_data_m[63:0]),
                               .in1(byp_irf_rd_data_w[63:0]),
                               .in2(rs1_data_w2[63:0]), 
                               .in3({{16{ifu_exu_pc_d[47]}}, ifu_exu_pc_d[47:0]}),
                             .sel0(ecl_byp_rs1_mux1_sel_m),
                             .sel1(ecl_byp_rs1_mux1_sel_w),
                             .sel2(ecl_byp_rs1_mux1_sel_w2),
                             .sel3(ecl_byp_rs1_mux1_sel_other));
   mux4ds #(64) mux_rs1_data_2(.dout(byp_alu_rs1_data_d[63:0]),
                             .in0(rs1_data_btwn_mux[63:0]),
                             .in1(irf_byp_rs1_data_d[63:0]), 
                             .in2(alu_byp_rd_data_e[63:0]),
                             .in3(lsu_exu_dfill_data_g[63:0]),
                             .sel0(ecl_byp_rs1_mux2_sel_usemux1),
                             .sel1(ecl_byp_rs1_mux2_sel_rf),
                             .sel2(ecl_byp_rs1_mux2_sel_e),
                             .sel3(ecl_byp_rs1_mux2_sel_ld));
   
   // rcc_data muxes: RF and E are critical paths
   mux4ds #(64) mux_rcc_data_1(.dout(rcc_data_btwn_mux[63:0]), 
                               .in0(rd_data_m[63:0]),
                               .in1(byp_irf_rd_data_w[63:0]),
                               .in2(rs1_data_w2[63:0]), 
                               .in3({64{1'b0}}),
                             .sel0(ecl_byp_rcc_mux1_sel_m),
                             .sel1(ecl_byp_rcc_mux1_sel_w),
                             .sel2(ecl_byp_rcc_mux1_sel_w2),
                             .sel3(ecl_byp_rcc_mux1_sel_other));
   mux4ds #(64) mux_rcc_data_2(.dout(byp_alu_rcc_data_d[63:0]),
                             .in0(rcc_data_btwn_mux[63:0]),
                             .in1(irf_byp_rs1_data_d[63:0]), 
                             .in2(alu_byp_rd_data_e[63:0]),
                             .in3(lsu_exu_dfill_data_g[63:0]),
                             .sel0(ecl_byp_rcc_mux2_sel_usemux1),
                             .sel1(ecl_byp_rcc_mux2_sel_rf),
                             .sel2(ecl_byp_rcc_mux2_sel_e),
                             .sel3(ecl_byp_rcc_mux2_sel_ld));

   // rs2_data muxes: RF and E are critical paths, optional is imm
   mux4ds #(64) mux_rs2_data_1(.dout(rs2_data_btwn_mux[63:0]), 
                             .in0(rd_data_m[63:0]),
                             .in1(byp_irf_rd_data_w[63:0]),
                             .in2(rs2_data_w2[63:0]),
                             .in3({{32{ifu_exu_imm_data_d[31]}},
                                   ifu_exu_imm_data_d[31:0]}),
                             .sel0(ecl_byp_rs2_mux1_sel_m),
                             .sel1(ecl_byp_rs2_mux1_sel_w),
                             .sel2(ecl_byp_rs2_mux1_sel_w2),
                             .sel3(ecl_byp_rs2_mux1_sel_other));
   mux4ds #(64) mux_rs2_data_2(.dout(byp_alu_rs2_data_d[63:0]),
                             .in0(rs2_data_btwn_mux[63:0]),
                             .in1(irf_byp_rs2_data_d[63:0]), 
                             .in2(alu_byp_rd_data_e[63:0]),
                             .in3(lsu_exu_dfill_data_g[63:0]),
                             .sel0(ecl_byp_rs2_mux2_sel_usemux1),
                             .sel1(ecl_byp_rs2_mux2_sel_rf),
                             .sel2(ecl_byp_rs2_mux2_sel_e),
                             .sel3(ecl_byp_rs2_mux2_sel_ld));
   
   // rs3_data muxes: RF and E are critical paths, no optional
   mux4ds #(64) mux_rs3_data_1(.dout(rs3_data_btwn_mux[63:0]), 
                                .in0(rd_data_m[63:0]),
                             .in1(byp_irf_rd_data_w[63:0]),
                             .in2(rs3_data_w2[63:0]), .in3({64{1'b0}}),
                             .sel0(ecl_byp_rs3_mux1_sel_m),
                             .sel1(ecl_byp_rs3_mux1_sel_w),
                             .sel2(ecl_byp_rs3_mux1_sel_w2),
                             .sel3(ecl_byp_rs3_mux1_sel_other));
   mux4ds #(64) mux_rs3_data_2(.dout(rs3_data_d[63:0]), 
                                .in0(rs3_data_btwn_mux[63:0]),
                                .in1(irf_byp_rs3_data_d[63:0]), 
                                .in2(alu_byp_rd_data_e[63:0]),
                                .in3(lsu_exu_dfill_data_g[63:0]),
                                .sel0(ecl_byp_rs3_mux2_sel_usemux1),
                                .sel1(ecl_byp_rs3_mux2_sel_rf),
                                .sel2(ecl_byp_rs3_mux2_sel_e),
                                .sel3(ecl_byp_rs3_mux2_sel_ld));
   
   // rs3_data muxes: RF and E are critical paths, no optional
   mux4ds #(32) mux_rs3h_data_1(.dout(rs3h_data_btwn_mux[31:0]), 
                                .in0(rd_data_m[31:0]),
                             .in1(byp_irf_rd_data_w[31:0]),
                             .in2(rs3h_data_w2[31:0]), .in3({32{1'b0}}),
                             .sel0(ecl_byp_rs3h_mux1_sel_m),
                             .sel1(ecl_byp_rs3h_mux1_sel_w),
                             .sel2(ecl_byp_rs3h_mux1_sel_w2),
                             .sel3(ecl_byp_rs3h_mux1_sel_other));
   mux4ds #(32) mux_rs3h_data_2(.dout(rs3h_data_d[31:0]), 
                                .in0(rs3h_data_btwn_mux[31:0]),
                                .in1(irf_byp_rs3h_data_d[31:0]), 
                                .in2(alu_byp_rd_data_e[31:0]),
                                .in3(lsu_exu_dfill_data_g[31:0]),
                                .sel0(ecl_byp_rs3h_mux2_sel_usemux1),
                                .sel1(ecl_byp_rs3h_mux2_sel_rf),
                                .sel2(ecl_byp_rs3h_mux2_sel_e),
                                .sel3(ecl_byp_rs3h_mux2_sel_ld));
 
   // ECC for W1






   sparc_exu_byp_eccgen w1_eccgen(.d(byp_irf_rd_data_m[63:0]),
                                   .msk(ecl_byp_ecc_mask_m_l[7:0]),
                                   .p(rd_synd_w_l[7:0]),
                                  .clk(sehold_clk), .se(se));

   assign        byp_irf_rd_data_w[71:64] = ~rd_synd_w_l[7:0];
   
   ////////////////////////
   // G arbitration muxes and W2 ECC
   ////////////////////////
   mux3ds #(64) mux_w2_data(.dout(rd_data_g[63:0]),
                            .in0(div_byp_muldivout_g[63:0]),
                            .in1(dfill_data_g2[63:0]),
                            .in2(restore_rd_data[63:0]),
                            .sel0(ecl_byp_sel_muldiv_g),
                            .sel1(ecl_byp_sel_load_g),
                            .sel2(ecl_byp_sel_restore_g));






   sparc_exu_byp_eccgen w2_eccgen(.d(rd_data_g[63:0]),
                                   .msk(ecl_byp_ecc_mask_m_l[7:0]),
                                  .p(rd_synd_w2_l[7:0]),
                                  .clk(sehold_clk), .se(se));

   assign        byp_irf_rd_data_w2[71:64] = ~rd_synd_w2_l[7:0];
   
endmodule // sparc_exu_byp

   
// Modified by Princeton University on June 9th, 2015
// ========== Copyright Header Begin ==========================================
// 
// OpenSPARC T1 Processor File: sparc_exu_byp_eccgen.v
// Copyright (c) 2006 Sun Microsystems, Inc.  All Rights Reserved.
// DO NOT ALTER OR REMOVE COPYRIGHT NOTICES.
// 
// The above named program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public
// License version 2 as published by the Free Software Foundation.
// 
// The above named program is distributed in the hope that it will be 
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// General Public License for more details.
// 
// You should have received a copy of the GNU General Public
// License along with this work; if not, write to the Free Software
// Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA 02110-1301, USA.
// 
// ========== Copyright Header End ============================================
////////////////////////////////////////////////////////////////////////////////
//  Module Name: zzecc_exu_byp_eccgen2
//      Description: This block generates the 8 bit ecc for a 64 bit input
//                              It is split over 2 cycles to accomodate the timing requirements of 
//                              the other blocks.

module sparc_exu_byp_eccgen ( p, d, msk, clk, se);
   input [63:0] d;
   input [7:0]  msk;
   input        clk;
   input        se;
   output [7:0] p;

   wire [7:0]   p0_g;
   wire [7:0]   p0_w;
   wire [7:0]   p1_g;
   wire [7:0]   p1_w;
   wire [7:0]   p2_g;
   wire [7:0]   p2_w;
   wire [7:0]   p3_g;
   wire [7:0]   p3_w;
   wire [3:0]   p4_g;
   wire [3:0]   p4_w;
   wire [1:0]   p5_g;
   wire [1:0]   p5_w;
   wire [1:0]   p6_g;
   wire [1:0]   p6_w;
   wire [7:0]   p7_g;
   wire [7:0]   p7_w;
   wire 	msk_w5;
   wire 	msk_w4;

   // Flops to store intermediate results
  dff_s Imsk_5_  ( .q(msk_w5),  .din(msk[5]),  .clk(clk), .se(se), .si(), .so());
  dff_s Imsk_4_  ( .q(msk_w4),  .din(msk[4]),  .clk(clk), .se(se), .si(), .so());
  dff_s Ip0ff_7_ ( .q(p0_w[7]), .din(p0_g[7]), .clk(clk), .se(se), .si(), .so());
  dff_s Ip0ff_6_ ( .q(p0_w[6]), .din(p0_g[6]), .clk(clk), .se(se), .si(), .so());
  dff_s Ip0ff_5_ ( .q(p0_w[5]), .din(p0_g[5]), .clk(clk), .se(se), .si(), .so());
  dff_s Ip0ff_4_ ( .q(p0_w[4]), .din(p0_g[4]), .clk(clk), .se(se), .si(), .so());
  dff_s Ip0ff_3_ ( .q(p0_w[3]), .din(p0_g[3]), .clk(clk), .se(se), .si(), .so());
  dff_s Ip0ff_2_ ( .q(p0_w[2]), .din(p0_g[2]), .clk(clk), .se(se), .si(), .so());
  dff_s Ip0ff_1_ ( .q(p0_w[1]), .din(p0_g[1]), .clk(clk), .se(se), .si(), .so());
  dff_s Ip0ff_0_ ( .q(p0_w[0]), .din(p0_g[0]), .clk(clk), .se(se), .si(), .so());
  dff_s Ip1ff_7_ ( .q(p1_w[7]), .din(p1_g[7]), .clk(clk), .se(se), .si(), .so());
  dff_s Ip1ff_6_ ( .q(p1_w[6]), .din(p1_g[6]), .clk(clk), .se(se), .si(), .so());
  dff_s Ip1ff_5_ ( .q(p1_w[5]), .din(p1_g[5]), .clk(clk), .se(se), .si(), .so());
  dff_s Ip1ff_4_ ( .q(p1_w[4]), .din(p1_g[4]), .clk(clk), .se(se), .si(), .so());
  dff_s Ip1ff_3_ ( .q(p1_w[3]), .din(p1_g[3]), .clk(clk), .se(se), .si(), .so());
  dff_s Ip1ff_2_ ( .q(p1_w[2]), .din(p1_g[2]), .clk(clk), .se(se), .si(), .so());
  dff_s Ip1ff_1_ ( .q(p1_w[1]), .din(p1_g[1]), .clk(clk), .se(se), .si(), .so());
  dff_s Ip1ff_0_ ( .q(p1_w[0]), .din(p1_g[0]), .clk(clk), .se(se), .si(), .so());
  dff_s Ip2ff_7_ ( .q(p2_w[7]), .din(p2_g[7]), .clk(clk), .se(se), .si(), .so());
  dff_s Ip2ff_6_ ( .q(p2_w[6]), .din(p2_g[6]), .clk(clk), .se(se), .si(), .so());
  dff_s Ip2ff_5_ ( .q(p2_w[5]), .din(p2_g[5]), .clk(clk), .se(se), .si(), .so());
  dff_s Ip2ff_4_ ( .q(p2_w[4]), .din(p2_g[4]), .clk(clk), .se(se), .si(), .so());
  dff_s Ip2ff_3_ ( .q(p2_w[3]), .din(p2_g[3]), .clk(clk), .se(se), .si(), .so());
  dff_s Ip2ff_2_ ( .q(p2_w[2]), .din(p2_g[2]), .clk(clk), .se(se), .si(), .so());
  dff_s Ip2ff_1_ ( .q(p2_w[1]), .din(p2_g[1]), .clk(clk), .se(se), .si(), .so());
  dff_s Ip2ff_0_ ( .q(p2_w[0]), .din(p2_g[0]), .clk(clk), .se(se), .si(), .so());
  dff_s Ip3ff_7_ ( .q(p3_w[7]), .din(p3_g[7]), .clk(clk), .se(se), .si(), .so());
  dff_s Ip3ff_6_ ( .q(p3_w[6]), .din(p3_g[6]), .clk(clk), .se(se), .si(), .so());
  dff_s Ip3ff_5_ ( .q(p3_w[5]), .din(p3_g[5]), .clk(clk), .se(se), .si(), .so());
  dff_s Ip3ff_4_ ( .q(p3_w[4]), .din(p3_g[4]), .clk(clk), .se(se), .si(), .so());
  dff_s Ip3ff_3_ ( .q(p3_w[3]), .din(p3_g[3]), .clk(clk), .se(se), .si(), .so());
  dff_s Ip3ff_2_ ( .q(p3_w[2]), .din(p3_g[2]), .clk(clk), .se(se), .si(), .so());
  dff_s Ip3ff_1_ ( .q(p3_w[1]), .din(p3_g[1]), .clk(clk), .se(se), .si(), .so());
  dff_s Ip3ff_0_ ( .q(p3_w[0]), .din(p3_g[0]), .clk(clk), .se(se), .si(), .so());
  dff_s Ip4ff_3_ ( .q(p4_w[3]), .din(p4_g[3]), .clk(clk), .se(se), .si(), .so());
  dff_s Ip4ff_2_ ( .q(p4_w[2]), .din(p4_g[2]), .clk(clk), .se(se), .si(), .so());
  dff_s Ip4ff_1_ ( .q(p4_w[1]), .din(p4_g[1]), .clk(clk), .se(se), .si(), .so());
  dff_s Ip4ff_0_ ( .q(p4_w[0]), .din(p4_g[0]), .clk(clk), .se(se), .si(), .so());
  dff_s Ip5ff_1_ ( .q(p5_w[1]), .din(p5_g[1]), .clk(clk), .se(se), .si(), .so());
  dff_s Ip5ff_0_ ( .q(p5_w[0]), .din(p5_g[0]), .clk(clk), .se(se), .si(), .so());
  dff_s Ip6ff_1_ ( .q(p6_w[1]), .din(p6_g[1]), .clk(clk), .se(se), .si(), .so());
  dff_s Ip6ff_0_ ( .q(p6_w[0]), .din(p6_g[0]), .clk(clk), .se(se), .si(), .so());
  dff_s Ip7ff_7_ ( .q(p7_w[7]), .din(p7_g[7]), .clk(clk), .se(se), .si(), .so());
  dff_s Ip7ff_6_ ( .q(p7_w[6]), .din(p7_g[6]), .clk(clk), .se(se), .si(), .so());
  dff_s Ip7ff_5_ ( .q(p7_w[5]), .din(p7_g[5]), .clk(clk), .se(se), .si(), .so());
  dff_s Ip7ff_4_ ( .q(p7_w[4]), .din(p7_g[4]), .clk(clk), .se(se), .si(), .so());
  dff_s Ip7ff_3_ ( .q(p7_w[3]), .din(p7_g[3]), .clk(clk), .se(se), .si(), .so());
  dff_s Ip7ff_2_ ( .q(p7_w[2]), .din(p7_g[2]), .clk(clk), .se(se), .si(), .so());
  dff_s Ip7ff_1_ ( .q(p7_w[1]), .din(p7_g[1]), .clk(clk), .se(se), .si(), .so());
  dff_s Ip7ff_0_ ( .q(p7_w[0]), .din(p7_g[0]), .clk(clk), .se(se), .si(), .so());


   // p[0]
   assign p[0] = p0_w[0] ^ p0_w[1] ^ p0_w[2] ^ p0_w[3] ^ p0_w[4] ^ p0_w[5] ^ p0_w[6] ^ p0_w[7]; 
   assign p0_g[0] = d[0]  ^ d[1]  ^ d[3]  ^ d[4]; 
   assign p0_g[1] = d[6]  ^ d[8]  ^ d[10] ^ d[11];
   assign p0_g[2] = d[13] ^ d[15] ^ d[17] ^ d[19];
   assign p0_g[3] = d[21] ^ d[23] ^ d[25] ^ d[26];
   assign p0_g[4] = d[28] ^ d[30] ^ d[32] ^ d[34];
   assign p0_g[5] = d[36] ^ d[38] ^ d[40] ^ d[42];
   assign p0_g[6] = d[44] ^ d[46] ^ d[48] ^ d[50];
   assign p0_g[7] = d[52] ^ d[54] ^ d[56] ^ d[57] ^ d[59] ^ d[61] ^ d[63] ^ msk[0];

   // p[1]
   assign p[1] = p1_w[0] ^ p1_w[1] ^ p1_w[2] ^ p1_w[3] ^ p1_w[4] ^ p1_w[5] ^ p1_w[6] ^ p1_w[7];
   assign p1_g[0] = d[0]  ^ d[2]  ^ d[3]  ^ d[5]; 
   assign p1_g[1] = d[6]  ^ d[9]  ^ d[10] ^ d[12];
   assign p1_g[2] = d[13] ^ d[16] ^ d[17] ^ d[20];
   assign p1_g[3] = d[21] ^ d[24] ^ d[25] ^ d[27];
   assign p1_g[4] = d[28] ^ d[31] ^ d[32] ^ d[35];
   assign p1_g[5] = d[36] ^ d[39] ^ d[40] ^ d[43];
   assign p1_g[6] = d[44] ^ d[47] ^ d[48] ^ d[51];
   assign p1_g[7] = d[52] ^ d[55] ^ d[56] ^ d[58] ^ d[59] ^ d[62] ^ d[63] ^ msk[1];

   // p[2]
   assign p[2] = p2_w[0] ^ p2_w[1] ^ p2_w[2] ^ p2_w[3] ^ p2_w[4] ^ p2_w[5] ^ p2_w[6] ^ p2_w[7];
   assign p2_g[0] = d[1]  ^ d[2]  ^ d[3]  ^ d[7]; 
   assign p2_g[1] = d[8]  ^ d[9]  ^ d[10] ^ d[14];
   assign p2_g[2] = d[15] ^ d[16] ^ d[17] ^ d[22];
   assign p2_g[3] = d[23] ^ d[24] ^ d[25] ^ d[29];
   assign p2_g[4] = d[30] ^ d[31] ^ d[32] ^ d[37];
   assign p2_g[5] = d[38] ^ d[39] ^ d[40] ^ d[45];
   assign p2_g[6] = d[46] ^ d[47] ^ d[48] ^ d[53];
   assign p2_g[7] = d[54] ^ d[55] ^ d[56] ^ d[60] ^ d[61] ^ d[62] ^ d[63] ^ msk[2];
   
   // p[3]
   assign p[3] =  p3_w[0] ^ p3_w[1] ^ p3_w[2] ^ p3_w[3] ^ p3_w[4] ^ p3_w[5] ^ p3_w[6] ^ p3_w[7];
   assign p3_g[0] = d[4]  ^ d[5]  ^ d[6]  ^ d[7]; 
   assign p3_g[1] = d[8]  ^ d[9]  ^ d[10] ^ d[18];
   assign p3_g[2] = d[19] ^ d[20] ^ d[21] ^ d[22];
   assign p3_g[3] = d[23] ^ d[24] ^ d[25] ^ d[33];
   assign p3_g[4] = d[34] ^ d[35] ^ d[36] ^ d[37];
   assign p3_g[5] = d[38] ^ d[39] ^ d[40] ^ d[49];
   assign p3_g[6] = d[50] ^ d[51] ^ d[52] ^ d[53];
   assign p3_g[7] = d[54] ^ d[55] ^ d[56] ^ msk[3];

   // p[4]
   assign p[4] =  p4_w[0] ^ p4_w[1] ^ p4_w[2] ^ p4_w[3] ^ msk_w4;

   assign p4_g[0] = d[11] ^ d[12] ^ d[13] ^ d[14] ^ d[15] ^ d[16] ^ d[17] ^ d[18];
   assign p4_g[1] = d[19] ^ d[20] ^ d[21] ^ d[22] ^ d[23] ^ d[24] ^ d[25] ^ d[41];
   assign p4_g[2] = d[42] ^ d[43] ^ d[44] ^ d[45] ^ d[46] ^ d[47] ^ d[48] ^ d[49];
   assign p4_g[3] = d[50] ^ d[51] ^ d[52] ^ d[53] ^ d[54] ^ d[55] ^ d[56];

   // p[5]
   assign p[5] =  p5_w[0] ^ p5_w[1] ^ p4_w[2] ^ p4_w[3] ^ msk_w5;
   assign p5_g[0] = d[26] ^ d[27] ^ d[28] ^ d[29] ^ d[30] ^ d[31] ^ d[32] ^ d[33];
   assign p5_g[1] = d[34] ^ d[35] ^ d[36] ^ d[37] ^ d[38] ^ d[39] ^ d[40] ^ d[41];
/* -----\/----- EXCLUDED -----\/-----
   assign p5_g[2] = (d[42]  ^ d[43]  ^ d[44]  ^ d[45]  ^
                     d[46]  ^ d[47]  ^ d[48]  ^ d[49]);
   assign p5_g[3] = (d[50]  ^ d[51]  ^ d[52]  ^ d[53]  ^
                     d[54]  ^ d[55]  ^ d[56]); 
 -----/\----- EXCLUDED -----/\----- */

   // p[6]
   assign p[6] =  p6_w[0] ^ p6_w[1];
   assign p6_g[0] = d[57] ^ d[58] ^ d[59] ^ d[60];
   assign p6_g[1] = d[61] ^ d[62] ^ d[63] ^ msk[6]; 

   // p[7]
   assign p[7] = p7_w[0] ^ p7_w[1] ^ p7_w[2] ^ p7_w[3] ^ p7_w[4] ^ p7_w[5] ^ p7_w[6] ^ p7_w[7];
   assign p7_g[0] = d[0]  ^ d[1]  ^ d[2]  ^ d[4]; 
   assign p7_g[1] = d[5]  ^ d[7]  ^ d[10] ^ d[11];
   assign p7_g[2] = d[12] ^ d[14] ^ d[17] ^ d[18];
   assign p7_g[3] = d[21] ^ d[23] ^ d[24] ^ d[26];
   assign p7_g[4] = d[27] ^ d[29] ^ d[32] ^ d[33];
   assign p7_g[5] = d[36] ^ d[38] ^ d[39] ^ d[41];
   assign p7_g[6] = d[44] ^ d[46] ^ d[47] ^ d[50];
   assign p7_g[7] = d[51] ^ d[53] ^ d[56] ^ d[57] ^ d[58] ^ d[60] ^ d[63] ^ msk[7];
       
endmodule // zzecc_exu_byp_eccgen3

// Modified by Princeton University on June 9th, 2015
// ========== Copyright Header Begin ==========================================
// 
// OpenSPARC T1 Processor File: sparc_exu_div.v
// Copyright (c) 2006 Sun Microsystems, Inc.  All Rights Reserved.
// DO NOT ALTER OR REMOVE COPYRIGHT NOTICES.
// 
// The above named program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public
// License version 2 as published by the Free Software Foundation.
// 
// The above named program is distributed in the hope that it will be 
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// General Public License for more details.
// 
// You should have received a copy of the GNU General Public
// License along with this work; if not, write to the Free Software
// Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA 02110-1301, USA.
// 
// ========== Copyright Header End ============================================
////////////////////////////////////////////////////////////////////////
/*
//  Module Name: sparc_exu_div
*/
module sparc_exu_div (/*AUTOARG*/
   // Outputs
   so, div_ecl_xin_msb_l, div_ecl_x_msb, div_ecl_d_msb, 
   div_ecl_cout64, div_ecl_cout32, div_ecl_gencc_in_msb_l, 
   div_ecl_gencc_in_31, div_ecl_upper32_equal, div_ecl_low32_nonzero, 
   div_ecl_dividend_msb, div_byp_muldivout_g, div_byp_yreg_e, 
   div_ecl_yreg_0_l, exu_mul_rs1_data, exu_mul_rs2_data, 
   div_ecl_adder_out_31, div_ecl_detect_zero_low, 
   div_ecl_detect_zero_high, div_ecl_d_62, 
   // Inputs
   ecl_div_yreg_wen_w, ecl_div_yreg_wen_l, ecl_div_yreg_wen_g, 
   ecl_div_yreg_shift_g, ecl_div_yreg_data_31_g, ecl_div_thr_e, 
   byp_div_yreg_data_w, rclk, se, si, ecl_div_keep_d, 
   ecl_div_ld_inputs, ecl_div_sel_adder, ecl_div_last_cycle, 
   ecl_div_almostlast_cycle, ecl_div_div64, ecl_div_sel_u32, 
   ecl_div_sel_pos32, ecl_div_sel_neg32, ecl_div_sel_64b, 
   ecl_div_upper32_zero, ecl_div_upper33_one, ecl_div_upper33_zero, 
   mul_exu_data_g, ecl_div_sel_div, ecl_div_mul_wen, 
   ecl_div_dividend_sign, ecl_div_subtract_l, ecl_div_cin, 
   ecl_div_newq, ecl_div_xinmask, ecl_div_keepx, 
   ecl_div_mul_get_new_data, ecl_div_mul_keep_data, 
   ecl_div_mul_get_32bit_data, ecl_div_mul_sext_rs2_e, 
   ecl_div_mul_sext_rs1_e, byp_div_rs1_data_e, byp_div_rs2_data_e, 
   ecl_div_muls_rs1_31_e_l, ecl_div_muls, ecl_div_zero_rs2_e
   ) ;
   /*AUTOINPUT*/
   // Beginning of automatic inputs (from unused autoinst inputs)
   input [31:0]         byp_div_yreg_data_w;    // To yreg of sparc_exu_div_yreg.v
   input [3:0]          ecl_div_thr_e;          // To yreg of sparc_exu_div_yreg.v
   input                ecl_div_yreg_data_31_g; // To yreg of sparc_exu_div_yreg.v
   input [3:0]          ecl_div_yreg_shift_g;   // To yreg of sparc_exu_div_yreg.v
   input [3:0]          ecl_div_yreg_wen_g;     // To yreg of sparc_exu_div_yreg.v
   input [3:0]          ecl_div_yreg_wen_l;     // To yreg of sparc_exu_div_yreg.v
   input [3:0]          ecl_div_yreg_wen_w;     // To yreg of sparc_exu_div_yreg.v
   // End of automatics
   input rclk;
   input se;
   input si;
   input        ecl_div_keep_d; // d should store (w/ overflow calcs)
   input        ecl_div_ld_inputs;// load in d and x
   input        ecl_div_sel_adder;// d should use adder output
   input        ecl_div_last_cycle;// last cycle of computations
   input         ecl_div_almostlast_cycle;// 2nd to last cycle of div
   input   ecl_div_div64;
   input         ecl_div_sel_u32;
   input         ecl_div_sel_pos32;
   input         ecl_div_sel_neg32;
   input         ecl_div_sel_64b;
   input         ecl_div_upper32_zero;
   input         ecl_div_upper33_one;
   input         ecl_div_upper33_zero;
   input [63:0]  mul_exu_data_g;
   input         ecl_div_sel_div;
   input         ecl_div_mul_wen;
   input         ecl_div_dividend_sign;
   input         ecl_div_subtract_l;     // add/subtract to adder
   input         ecl_div_cin;
   input  ecl_div_newq;         // newest q bit
   input         ecl_div_xinmask;
   input  ecl_div_keepx;
   input         ecl_div_mul_get_new_data;
   input         ecl_div_mul_keep_data;
   input         ecl_div_mul_get_32bit_data;
   input         ecl_div_mul_sext_rs2_e;
   input         ecl_div_mul_sext_rs1_e;
   input [63:0]  byp_div_rs1_data_e;
   input [63:0]  byp_div_rs2_data_e;
   input         ecl_div_muls_rs1_31_e_l;
   input         ecl_div_muls;
   input         ecl_div_zero_rs2_e;
   
   output        so;
   output div_ecl_xin_msb_l;
   output div_ecl_x_msb;
   output div_ecl_d_msb;
   output div_ecl_cout64;       // cout from adder
   output div_ecl_cout32;       // cout from adder
   output        div_ecl_gencc_in_msb_l;
   output        div_ecl_gencc_in_31;
   output        div_ecl_upper32_equal;
   output        div_ecl_low32_nonzero;
   output        div_ecl_dividend_msb;
   output [63:0] div_byp_muldivout_g;
   output [31:0] div_byp_yreg_e;
   output [3:0]  div_ecl_yreg_0_l;
   output [63:0]          exu_mul_rs1_data; 
   output [63:0]          exu_mul_rs2_data;
   output                 div_ecl_adder_out_31; 
   output        div_ecl_detect_zero_low;
   output        div_ecl_detect_zero_high;
   output        div_ecl_d_62;

   /*AUTOWIRE*/
   // Beginning of automatic wires (for undeclared instantiated-module outputs)
   wire [31:0]          yreg_mdq_y_e;           // From yreg of sparc_exu_div_yreg.v
   // End of automatics
   wire                 clk;
   wire [127:0]  din;           // sign extended dividend
   wire [127:0]  d;             // current dividend/quotient
   wire [63:0]   adder_out;     // output of adder
   wire [127:0]  dnext;         // input to d flop
   wire [127:0]  adder_dnext;   // combination of adder out and quotient
   wire [63:0]   x;             // divisor
   wire [63:0]   xin;           // sign extended (for 32bit) divisor
   wire [63:0]   xnext;         // input to divisor flop
   wire [63:0]   adderin1;      // first input to adder
   wire [63:0]   adderin2;      // 2nd input to adder
   
   wire [63:0]   curr_q;        // current quotient
   wire [63:0]   out64;         // 64 bit result
   wire [63:0]   pos32;         // positive 32 bit result w/ ovfl
   wire [63:0]   neg32;         // negative 32 bit result w/ ovfl
   wire [63:0]   u32;           // unsigned 32 bit result w/ ovfl
   wire [63:0]   gencc_in;
   wire [63:0]   mul_result;
   wire [63:0]   mul_result_next;
   wire [127:0]  input_data_e;
   wire [63:0]   dividend;
   wire [63:0]   divisor;
   wire [127:0]  next_mul_data;
   wire [127:0]  mul_data_out;
   wire [127:0]  mul32_input_data_e;
   wire          subtract;
   wire [63:0]   spr_out;
   wire [63:0]   z_in;

   assign        clk = rclk;
   ///////////////////////////////////////
   // Input masking for 32 bit operations
   ///////////////////////////////////////
   dp_buffer #(128) buf_input_data(.dout(input_data_e[127:0]), 
                                   .in({byp_div_rs2_data_e[63:0], byp_div_rs1_data_e[63:0]}));
   // Mux in yreg into upper 32 bits on 32 bit divides
   dp_mux2es #(32) dividendmux(.dout(dividend[63:32]),
                             .in0(yreg_mdq_y_e[31:0]),
                             .in1(input_data_e[63:32]),
                             .sel(ecl_div_div64));
   assign        dividend[31:0] = input_data_e[31:0];
   assign        divisor[63:0] = input_data_e[127:64];

   
   /////////////////////
   // Output assignment
   /////////////////////
   dp_mux2es #(64) output_mux(.dout(div_byp_muldivout_g[63:0]), .in1(d[63:0]),
                         .in0(mul_result[63:0]),
                         .sel(ecl_div_sel_div));
   ///////////////////////////
   // Generate Condition Codes and divide by zero exception and overflow
   ///////////////////////////
   dp_mux2es #(64) gencc_mux(.dout(gencc_in[63:0]), 
                          .in0(mul_result[63:0]),
                          .in1(curr_q[63:0]),
                          .sel(ecl_div_sel_div));
   sparc_exu_div_32eql u32eql(.in(gencc_in[63:32]), .equal(div_ecl_upper32_equal));
   sparc_exu_aluor32 low32or(// Outputs
                             .out  (div_ecl_low32_nonzero),
                             // Inputs
                             .in    (gencc_in[31:0]));  
   assign        div_ecl_gencc_in_msb_l = ~gencc_in[63];
   assign        div_ecl_gencc_in_31 = gencc_in[31];
   
   
   // Division overflow calculations
   assign        curr_q = d[127:64];
   assign        u32 = {32'b0, (curr_q[31:0] | {32{~ecl_div_upper32_zero}})}; 
   assign        pos32 = {33'b0, (curr_q[30:0] | {31{~ecl_div_upper33_zero}})}; 
   assign        neg32 = {{33{1'b1}}, (curr_q[30:0] & {31{ecl_div_upper33_one}})}; 
   
   mux4ds #(64) result_mux(.dout(out64[63:0]), .in0(curr_q[63:0]), .in1(u32[63:0]),
                         .in2(pos32[63:0]), .in3(neg32[63:0]), .sel0(ecl_div_sel_64b),
                         .sel1(ecl_div_sel_u32), .sel2(ecl_div_sel_pos32),
                         .sel3(ecl_div_sel_neg32));
   
   //////////////////////////
   // Logic for D (dividend)
   //////////////////////////
   
   // If signed div sign extend dividend to 127 bits
   assign        div_ecl_dividend_msb = dividend[63];
   assign        din[62:0] = dividend[62:0];
   dp_mux2es #(32) din_mux(.dout(din[94:63]),
                           .in0({{31{ecl_div_dividend_sign}}, dividend[63]}),
                           .in1({~ecl_div_muls_rs1_31_e_l, dividend[31:1]}),
                           .sel(ecl_div_muls));
   assign        din[127:95] = {33{ecl_div_dividend_sign}};
//   assign        din = {{64{ecl_div_dividend_sign}}, dividend[63:0]};


   // Select input to FF for d
   mux3ds #(128) d_mux(.dout(dnext[127:0]), .in0({d[127:64], out64[63:0]}),
                     .in1(adder_dnext[127:0]), .in2(din[127:0]),
                     .sel0(ecl_div_keep_d),
                     .sel1(ecl_div_sel_adder),
                     .sel2(ecl_div_ld_inputs));
   assign        div_ecl_d_62 = d[62];

   // FF for d
   dff_s #(128) d_dff(.din(dnext[127:0]), .clk(clk), .q(d[127:0]), .se(se), .si(), .so());

   ////////////////////////////
   // Logic for X (divisor)
   ////////////////////////////
   // if signed div and 32 bits sign extend to upper 32 bits
   dp_mux2es #(32) xin_mux(.dout(xin[63:32]), .in1(divisor[63:32]),
                      .in0({32{ecl_div_xinmask}}),
                      .sel(ecl_div_div64));
   assign        xin[31:0] = divisor[31:0] & {32{~ecl_div_zero_rs2_e}};
   //assign xin[31:0] = divisor[31:0];

   // Pick between x and divisor and 1 (use divisor on first cycle, 1 last cycle)
   mux3ds #(64) x_mux(.dout(xnext[63:0]), .in0(x[63:0]), .in1(xin[63:0]), .in2({64'b0}),
                    .sel0(ecl_div_keepx),
                    .sel1(ecl_div_ld_inputs),
                    .sel2(ecl_div_almostlast_cycle));

   // FF for x
   dff_s #(64) x_dff(.din(xnext[63:0]), .clk(clk), .q(x[63:0]), .se(se), .si(), .so());


   ///////////////////////////
   // Logic for inputs to adder
   //////////////////////////
   assign div_ecl_xin_msb_l = ~xin[63];
   assign div_ecl_x_msb = x[63];
   assign div_ecl_d_msb = d[127];
   dp_mux2es #(64) in1_mux(.dout(adderin1[63:0]), .in0(d[126:63]),
                      .in1({d[62:0], ecl_div_newq}), .sel(ecl_div_last_cycle));

   assign subtract = ~ecl_div_subtract_l;
   assign        adderin2[63:0] = x[63:0] ^ {64{subtract}};

   //////////////////////////
   //  Adder
   /////////////////////////
   sparc_exu_aluadder64 add64(// Outputs
                              .adder_out(adder_out[63:0]),
                              .cout32   (div_ecl_cout32),
                              .cout64   (div_ecl_cout64),
                              // Inputs
                              .rs1_data (adderin1[63:0]),
                              .rs2_data (adderin2[63:0]),
                              .cin      (ecl_div_cin));

   assign        adder_dnext = {adder_out[63:0], d[62:0], ecl_div_newq};
   assign        div_ecl_adder_out_31 = adder_out[31];

   // sum predict and zero detection
   sparc_exu_aluspr spr(.rs1_data(adderin1[63:0]), .rs2_data(adderin2[63:0]), .cin(ecl_div_cin),
                        .spr_out(spr_out[63:0]));   
   dp_mux2es #(64) zero_detect_mux(.dout(z_in[63:0]),
                                   .in0(spr_out[63:0]),
                                   .in1(xin[63:0]),
                                   .sel(ecl_div_ld_inputs));
   //sparc_exu_aluzcmp64 regzcmp(.in(z_in[63:0]), .zero64(div_ecl_detect_zero));
   assign        div_ecl_detect_zero_low = ~(|z_in[31:0]);
   assign        div_ecl_detect_zero_high = ~(|z_in[63:32]);
   

   // y register
   assign        div_byp_yreg_e = yreg_mdq_y_e;
   sparc_exu_div_yreg yreg(.mul_div_yreg_data_g(mul_exu_data_g[63:32]),
                           /*AUTOINST*/
                           // Outputs
                           .yreg_mdq_y_e(yreg_mdq_y_e[31:0]),
                           .div_ecl_yreg_0_l(div_ecl_yreg_0_l[3:0]),
                           // Inputs
                           .clk         (clk),
                           .se          (se),
                           .byp_div_yreg_data_w(byp_div_yreg_data_w[31:0]),
                           .ecl_div_thr_e(ecl_div_thr_e[3:0]),
                           .ecl_div_yreg_wen_w(ecl_div_yreg_wen_w[3:0]),
                           .ecl_div_yreg_wen_g(ecl_div_yreg_wen_g[3:0]),
                           .ecl_div_yreg_wen_l(ecl_div_yreg_wen_l[3:0]),
                           .ecl_div_yreg_data_31_g(ecl_div_yreg_data_31_g),
                           .ecl_div_yreg_shift_g(ecl_div_yreg_shift_g[3:0]));
   
   
   //////////////////////////////////
   // MULTIPLIER inputs
   //////////////////////////////////                  
   assign        mul32_input_data_e[127:64] = {{32{ecl_div_mul_sext_rs2_e}}, input_data_e[95:64]};
   assign        mul32_input_data_e[63:0] = {{32{ecl_div_mul_sext_rs1_e}}, input_data_e[31:0]};
   mux3ds #(128) mul_data_mux(.dout(next_mul_data[127:0]),
                              .in0(input_data_e[127:0]),
                              .in1(mul32_input_data_e[127:0]),
                              .in2(mul_data_out[127:0]),
                              .sel0(ecl_div_mul_get_new_data),
                              .sel1(ecl_div_mul_get_32bit_data),
                              .sel2(ecl_div_mul_keep_data));
   dff_s #(128) mul_data_dff(.din(next_mul_data[127:0]), .clk(clk), .q(mul_data_out[127:0]),
                           .se(se), .si(), .so());
   assign        exu_mul_rs1_data = mul_data_out[63:0];
   assign        exu_mul_rs2_data = mul_data_out[127:64];

   ///////////////////////////////////
   // Store output from mul
   //////////////////////////////////
   dp_mux2es #(64) mul_result_mux(.dout(mul_result_next[63:0]), .in0(mul_result[63:0]),
                           .in1(mul_exu_data_g[63:0]),
                           .sel(ecl_div_mul_wen));
   dff_s #(64) mul_result_dff(.din(mul_result_next[63:0]), .clk(clk), .q(mul_result[63:0]),
                        .se(se), .si(), .so());

   
endmodule // sparc_exu_div
// Modified by Princeton University on June 9th, 2015
// ========== Copyright Header Begin ==========================================
// 
// OpenSPARC T1 Processor File: sparc_exu_div_32eql.v
// Copyright (c) 2006 Sun Microsystems, Inc.  All Rights Reserved.
// DO NOT ALTER OR REMOVE COPYRIGHT NOTICES.
// 
// The above named program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public
// License version 2 as published by the Free Software Foundation.
// 
// The above named program is distributed in the hope that it will be 
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// General Public License for more details.
// 
// You should have received a copy of the GNU General Public
// License along with this work; if not, write to the Free Software
// Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA 02110-1301, USA.
// 
// ========== Copyright Header End ============================================
module sparc_exu_div_32eql (/*AUTOARG*/
   // Outputs
   equal, 
   // Inputs
   in
   ) ;
   input [31:0] in;

   output       equal;

   wire [31:0]  inxor;
   wire         notequal;

   assign       inxor[0] = 1'b0;
   assign inxor[1] = in[31] ^ in[30];
   assign inxor[2] = in[30] ^ in[29];
   assign inxor[3] = in[29] ^ in[28];
   assign inxor[4] = in[28] ^ in[27];
   assign inxor[5] = in[27] ^ in[26];
   assign inxor[6] = in[26] ^ in[25];
   assign inxor[7] = in[25] ^ in[24];
   assign inxor[8] = in[24] ^ in[23];
   assign inxor[9] = in[23] ^ in[22];
   assign inxor[10] = in[22] ^ in[21];
   assign inxor[11] = in[21] ^ in[20];
   assign inxor[12] = in[20] ^ in[19];
   assign inxor[13] = in[19] ^ in[18];
   assign inxor[14] = in[18] ^ in[17];
   assign inxor[15] = in[17] ^ in[16];
   assign inxor[16] = in[16] ^ in[15];
   assign inxor[17] = in[15] ^ in[14];
   assign inxor[18] = in[14] ^ in[13];
   assign inxor[19] = in[13] ^ in[12];
   assign inxor[20] = in[12] ^ in[11];
   assign inxor[21] = in[11] ^ in[10];
   assign inxor[22] = in[10] ^ in[9];
   assign inxor[23] = in[9] ^ in[8];
   assign inxor[24] = in[8] ^ in[7];
   assign inxor[25] = in[7] ^ in[6];
   assign inxor[26] = in[6] ^ in[5];
   assign inxor[27] = in[5] ^ in[4];
   assign inxor[28] = in[4] ^ in[3];
   assign inxor[29] = in[3] ^ in[2];
   assign inxor[30] = in[2] ^ in[1];
   assign inxor[31] = in[1] ^ in[0];

   assign equal = ~notequal;
   sparc_exu_aluor32 or32(// Outputs
                          .out     (notequal),
                          // Inputs
                          .in       (inxor[31:0]));
   
endmodule // sparc_exu_div_32eql
// Modified by Princeton University on June 9th, 2015
// ========== Copyright Header Begin ==========================================
// 
// OpenSPARC T1 Processor File: sparc_exu_div_yreg.v
// Copyright (c) 2006 Sun Microsystems, Inc.  All Rights Reserved.
// DO NOT ALTER OR REMOVE COPYRIGHT NOTICES.
// 
// The above named program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public
// License version 2 as published by the Free Software Foundation.
// 
// The above named program is distributed in the hope that it will be 
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// General Public License for more details.
// 
// You should have received a copy of the GNU General Public
// License along with this work; if not, write to the Free Software
// Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA 02110-1301, USA.
// 
// ========== Copyright Header End ============================================
////////////////////////////////////////////////////////////////////////
/*
//  Module Name: sparc_exu_div_yreg
//	Description: The 4 32 bit y registers.  It can be written to
// 		twice each cycle because by definition the writes must come
//		from different threads.  There is no bypassing because wry switches out.
*/
// Modified by Princeton University on June 9th, 2015
/*
* ========== Copyright Header Begin ==========================================
* 
* OpenSPARC T1 Processor File: sys.h
* Copyright (c) 2006 Sun Microsystems, Inc.  All Rights Reserved.
* DO NOT ALTER OR REMOVE COPYRIGHT NOTICES.
* 
* The above named program is free software; you can redistribute it and/or
* modify it under the terms of the GNU General Public
* License version 2 as published by the Free Software Foundation.
* 
* The above named program is distributed in the hope that it will be 
* useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
* General Public License for more details.
* 
* You should have received a copy of the GNU General Public
* License along with this work; if not, write to the Free Software
* Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA 02110-1301, USA.
* 
* ========== Copyright Header End ============================================
*/
// -*- verilog -*-
////////////////////////////////////////////////////////////////////////
/*
//
// Description:		Global header file that contain definitions that 
//                      are common/shared at the systme level
*/
////////////////////////////////////////////////////////////////////////
//
// Setting the time scale
// If the timescale changes, JP_TIMESCALE may also have to change.
`timescale	1ps/1ps
`default_nettype wire

//
// Number of threads in a core
// ===========================
//

//`define CONFIG_NUM_THREADS // This must be defined for any of below to work
//`define THREADS_1
//`define THREADS_2
//`define THREADS_3


//
// JBUS clock
// =========
//
// `define SYSCLK_PERIOD   5000


// Afara Link Defines
// ==================

// Reliable Link




// Afara Link Objects


// Afara Link Object Format - Reliable Link










// Afara Link Object Format - Congestion



  







// Afara Link Object Format - Acknowledge











// Afara Link Object Format - Request

















// Afara Link Object Format - Message



// Acknowledge Types




// Request Types





// Afara Link Frame



//
// UCB Packet Type
// ===============
//

















//
// UCB Data Packet Format
// ======================
//






























// Size encoding for the UCB_SIZE_HI/LO field
// 000 - byte
// 001 - half-word
// 010 - word
// 011 - double-word
// 111 - quad-word







//
// UCB Interrupt Packet Format
// ===========================
//










//`define UCB_THR_HI             9      // (6) cpu/thread ID shared with
//`define UCB_THR_LO             4             data packet format
//`define UCB_PKT_HI             3      // (4) packet type shared with
//`define UCB_PKT_LO             0      //     data packet format







//
// FCRAM Bus Widths
// ================
//






//
// ENET clock periods
// ==================
//
// `define AXGRMII_CLK_PERIOD          6400 // 312.5MHz/2
// `define ENET_GMAC_CLK_PERIOD        8000 // 125MHz


//
// JBus Bridge defines
// =================
//
// `define      SYS_UPA_CLK        `SYS.upa_clk
// `define      SYS_J_CLK          `SYS.j_clk
// `define      SYS_P_CLK          `SYS.p_clk
// `define      SYS_G_CLK          `SYS.g_clk
// `define      JP_TIMESCALE       `timescale 1 ps / 1 ps
// `define      PCI_CLK_PERIOD     15152                  //  66 MHz
// `define      UPA_RD_CLK_PERIOD  6666                   // 150 MHz
// `define      UPA_REF_CLK_PERIOD 7576                   // 132 MHz
// `define      ICHIP_CLK_PERIOD   30304                  //  33 MHz


//
// PCI Device Address Configuration
// ================================
//
























module sparc_exu_div_yreg (/*AUTOARG*/
   // Outputs
   yreg_mdq_y_e, div_ecl_yreg_0_l, 
   // Inputs
   clk, se, byp_div_yreg_data_w, mul_div_yreg_data_g, ecl_div_thr_e, 
   ecl_div_yreg_wen_w, ecl_div_yreg_wen_g, ecl_div_yreg_wen_l, 
   ecl_div_yreg_data_31_g, ecl_div_yreg_shift_g
   ) ;
   input clk;
   input se;
   input [31:0] byp_div_yreg_data_w;
   input [31:0] mul_div_yreg_data_g;
   input [3:0]  ecl_div_thr_e;
   input [3:0]  ecl_div_yreg_wen_w;
   input [3:0]  ecl_div_yreg_wen_g;
   input [3:0]  ecl_div_yreg_wen_l;// w or w2
   input        ecl_div_yreg_data_31_g;// bit shifted in on muls
   input [3:0]  ecl_div_yreg_shift_g;// yreg should be shifted
   
   output [31:0] yreg_mdq_y_e;
   output [3:0]  div_ecl_yreg_0_l;

   wire [31:0]   next_yreg_thr0;// next value for yreg
   wire [31:0]   next_yreg_thr1;
   wire [31:0]   next_yreg_thr2;
   wire [31:0]   next_yreg_thr3;
   wire [31:0]   yreg_thr0;     // current value of yreg
   wire [31:0]   yreg_thr1;
   wire [31:0]   yreg_thr2;
   wire [31:0]   yreg_thr3;
   wire [3:0]    div_ecl_yreg_0;
   wire [31:0]   yreg_data_w1;


   //////////////////////////////////
   //  Output selection for yreg
   //////////////////////////////////
   // output the LSB of all 4 regs
   assign        div_ecl_yreg_0[3:0] = {yreg_thr3[0],yreg_thr2[0],yreg_thr1[0],yreg_thr0[0]};
   assign        div_ecl_yreg_0_l[3:0] = ~div_ecl_yreg_0[3:0];

 // Use two threads unless this is defined

   // mux between the 2 yregs
   mux2ds #(32) mux_yreg_out(.dout(yreg_mdq_y_e[31:0]), .sel0(ecl_div_thr_e[0]),
                         .sel1(ecl_div_thr_e[1]), .in0(yreg_thr0[31:0]),
                         .in1(yreg_thr1[31:0])); 




































 // `ifndef CONFIG_NUM_THREADS
   
   //////////////////////////////////////
   //  Storage of yreg
   //////////////////////////////////////
   // pass along yreg w to w2 (for control signal timing)
   dff_s #(32) yreg_dff_w2w2(.din(byp_div_yreg_data_w[31:0]), .clk(clk), .q(yreg_data_w1[31:0]),
                           .se(se), .si(), .so());


   // mux between yreg_w, yreg_g, old value
   mux4ds #(32) mux_yregin0(.dout(next_yreg_thr0[31:0]), 
                          .sel0(ecl_div_yreg_wen_w[0]),
                          .sel1(ecl_div_yreg_wen_g[0]), 
                          .sel2(ecl_div_yreg_wen_l[0]),
                          .sel3(ecl_div_yreg_shift_g[0]),
                          .in0(yreg_data_w1[31:0]),
                          .in1(mul_div_yreg_data_g[31:0]), 
                          .in2(yreg_thr0[31:0]),
                          .in3({ecl_div_yreg_data_31_g, yreg_thr0[31:1]}));

 // Use two threads unless this is defined

   mux4ds #(32) mux_yregin1(.dout(next_yreg_thr1[31:0]),
                          .sel0(ecl_div_yreg_wen_w[1]),
                          .sel1(ecl_div_yreg_wen_g[1]),
                          .sel2(ecl_div_yreg_wen_l[1]),
                          .sel3(ecl_div_yreg_shift_g[1]),
                          .in0(yreg_data_w1[31:0]),
                          .in1(mul_div_yreg_data_g[31:0]),
                          .in2(yreg_thr1[31:0]),
                          .in3({ecl_div_yreg_data_31_g, yreg_thr1[31:1]}));
   assign    next_yreg_thr2[31:0] = yreg_data_w1[31:0];
   assign    next_yreg_thr3[31:0] = yreg_data_w1[31:0];



















































































 // `ifndef CONFIG_NUM_THREADS
   
   // store new value
   dff_s #(32) dff_yreg_thr0(.din(next_yreg_thr0[31:0]), .clk(clk), .q(yreg_thr0[31:0]),
                       .se(se), .si(), .so());
   dff_s #(32) dff_yreg_thr1(.din(next_yreg_thr1[31:0]), .clk(clk), .q(yreg_thr1[31:0]),
                       .se(se), .si(), .so());
   dff_s #(32) dff_yreg_thr2(.din(next_yreg_thr2[31:0]), .clk(clk), .q(yreg_thr2[31:0]),
                       .se(se), .si(), .so());
   dff_s #(32) dff_yreg_thr3(.din(next_yreg_thr3[31:0]), .clk(clk), .q(yreg_thr3[31:0]),
                       .se(se), .si(), .so());
   
   
endmodule // sparc_exu_div_yreg
// Modified by Princeton University on June 9th, 2015
// ========== Copyright Header Begin ==========================================
// 
// OpenSPARC T1 Processor File: sparc_exu_ecc.v
// Copyright (c) 2006 Sun Microsystems, Inc.  All Rights Reserved.
// DO NOT ALTER OR REMOVE COPYRIGHT NOTICES.
// 
// The above named program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public
// License version 2 as published by the Free Software Foundation.
// 
// The above named program is distributed in the hope that it will be 
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// General Public License for more details.
// 
// You should have received a copy of the GNU General Public
// License along with this work; if not, write to the Free Software
// Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA 02110-1301, USA.
// 
// ========== Copyright Header End ============================================
////////////////////////////////////////////////////////////////////////
/*
//  Module Name: sparc_exu_ecc
//	Description: This block performs the ecc check and correction as well as
// 			doing the w2 write port arbitration and the w2 ecc generation.
*/
module sparc_exu_ecc (/*AUTOARG*/
   // Outputs
   so, ecc_ecl_rs1_ce, ecc_ecl_rs1_ue, ecc_ecl_rs2_ce, 
   ecc_ecl_rs2_ue, ecc_ecl_rs3_ce, ecc_ecl_rs3_ue, 
   ecc_byp_ecc_result_m, exu_ifu_err_synd_m, 
   // Inputs
   rclk, se, si, byp_ecc_rcc_data_e, ecl_ecc_rs1_use_rf_e, 
   byp_ecc_rs1_synd_d, byp_alu_rs2_data_e, ecl_ecc_rs2_use_rf_e, 
   byp_ecc_rs2_synd_d, byp_ecc_rs3_data_e, ecl_ecc_rs3_use_rf_e, 
   byp_ecc_rs3_synd_d, ecl_ecc_sel_rs1_m_l, ecl_ecc_sel_rs2_m_l, 
   ecl_ecc_sel_rs3_m_l, ecl_ecc_log_rs1_m, ecl_ecc_log_rs2_m, 
   ecl_ecc_log_rs3_m
   ) ;
   input rclk;
   input se;
   input si;
   input [63:0] byp_ecc_rcc_data_e;
   input        ecl_ecc_rs1_use_rf_e;
   input [7:0]  byp_ecc_rs1_synd_d;
   input [63:0] byp_alu_rs2_data_e;
   input        ecl_ecc_rs2_use_rf_e;
   input [7:0]  byp_ecc_rs2_synd_d;
   input [63:0] byp_ecc_rs3_data_e;
   input        ecl_ecc_rs3_use_rf_e;
   input [7:0]  byp_ecc_rs3_synd_d;
   input        ecl_ecc_sel_rs1_m_l;
   input        ecl_ecc_sel_rs2_m_l;
   input        ecl_ecc_sel_rs3_m_l;
   input        ecl_ecc_log_rs1_m;
   input        ecl_ecc_log_rs2_m;
   input        ecl_ecc_log_rs3_m;

   output       so;
   output       ecc_ecl_rs1_ce;
   output       ecc_ecl_rs1_ue;
   output       ecc_ecl_rs2_ce;
   output       ecc_ecl_rs2_ue;
   output       ecc_ecl_rs3_ce;
   output       ecc_ecl_rs3_ue;

   output [63:0] ecc_byp_ecc_result_m;
   output [6:0]  exu_ifu_err_synd_m;

   wire          clk;
   wire         sel_rs1_m;
   wire         sel_rs2_m;
   wire         sel_rs3_m;
   wire [7:0]   rs1_ecc_e;
   wire [6:0]   rs1_err_e;      // syndrome generated by checker
   wire [6:0]   rs1_err_m;      // syndrome generated by checker
   wire [7:0]   rs2_ecc_e;
   wire [6:0]   rs2_err_e;      // syndrome generated by checker
   wire [6:0]   rs2_err_m;      // syndrome generated by checker
   wire [7:0]   rs3_ecc_e;
   wire [6:0]   rs3_err_e;      // syndrome generated by checker
   wire [6:0]   rs3_err_m;      // syndrome generated by checker
   wire [6:0]   err_m;
   wire [63:0]  ecc_datain_m;
   wire [63:0]  byp_ecc_rcc_data_m;
   wire [63:0]  byp_alu_rs2_data_m;
   wire [63:0]  exu_lsu_rs3_data_m;
   wire [63:0]  error_data_m;

   assign       clk = rclk;
   // Pass along ecc parity bits from RF
   dff_s #(8) rs1_ecc_d2e(.din(byp_ecc_rs1_synd_d[7:0]), .clk(clk), .q(rs1_ecc_e[7:0]),
                      .se(se), .si(), .so());
   dff_s #(8) rs2_ecc_d2e(.din(byp_ecc_rs2_synd_d[7:0]), .clk(clk), .q(rs2_ecc_e[7:0]),
                      .se(se), .si(), .so());
   dff_s #(8) rs3_ecc_d2e(.din(byp_ecc_rs3_synd_d[7:0]), .clk(clk), .q(rs3_ecc_e[7:0]),
                      .se(se), .si(), .so());
   
   // Check the ecc for all 4 outputs from RF
   zzecc_exu_chkecc2 chk_rs1(.d(byp_ecc_rcc_data_e[63:0]),
                            .vld(ecl_ecc_rs1_use_rf_e),
                            .p(rs1_ecc_e[7:0]),
                            .q(rs1_err_e[6:0]),
                            .ce(ecc_ecl_rs1_ce), .ue(ecc_ecl_rs1_ue), .ne());
   zzecc_exu_chkecc2 chk_rs2(.d(byp_alu_rs2_data_e[63:0]),
                            .vld(ecl_ecc_rs2_use_rf_e),
                            .p(rs2_ecc_e[7:0]),
                            .q(rs2_err_e[6:0]),
                            .ce(ecc_ecl_rs2_ce), .ue(ecc_ecl_rs2_ue), .ne());
   zzecc_exu_chkecc2 chk_rs3(.d(byp_ecc_rs3_data_e[63:0]),
                                .vld(ecl_ecc_rs3_use_rf_e),
                                .p(rs3_ecc_e[7:0]),
                                .q(rs3_err_e[6:0]),
                                .ce(ecc_ecl_rs3_ce), .ue(ecc_ecl_rs3_ue), .ne());

   // Put results from checkers into flops
   dff_s #(7) rs1_err_e2m(.din(rs1_err_e[6:0]), .clk(clk), .q(rs1_err_m[6:0]),
                      .se(se), .si(), .so());
   dff_s #(7) rs2_err_e2m(.din(rs2_err_e[6:0]), .clk(clk), .q(rs2_err_m[6:0]),
                      .se(se), .si(), .so());
   dff_s #(7) rs3o_err_e2m(.din(rs3_err_e[6:0]), .clk(clk), .q(rs3_err_m[6:0]),
                      .se(se), .si(), .so());

   // Pass along RF data to M stage
   dff_s #(64) rs1_data_e2m(.din(byp_ecc_rcc_data_e[63:0]), .clk(clk), .q(byp_ecc_rcc_data_m[63:0]),
                        .se(se), .si(), .so());
   dff_s #(64) rs2_data_e2m(.din(byp_alu_rs2_data_e[63:0]), .clk(clk), .q(byp_alu_rs2_data_m[63:0]),
                        .se(se), .si(), .so());
   dff_s #(64) rs3_data_e2m(.din(byp_ecc_rs3_data_e[63:0]), .clk(clk), 
                         .q(exu_lsu_rs3_data_m[63:0]),
                         .se(se), .si(), .so());

   // Mux between 3 different ports for syndrome and data
   assign       sel_rs1_m = ~ecl_ecc_sel_rs1_m_l;
   assign       sel_rs2_m = ~ecl_ecc_sel_rs2_m_l;
   assign       sel_rs3_m = ~ecl_ecc_sel_rs3_m_l;
   mux3ds #(7) syn_mux(.dout(err_m[6:0]),
                     .in0(rs1_err_m[6:0]),
                     .in1(rs2_err_m[6:0]),
                     .in2(rs3_err_m[6:0]),
                     .sel0(sel_rs1_m),
                     .sel1(sel_rs2_m),
                     .sel2(sel_rs3_m));
   mux3ds #(64) data_m_mux(.dout(ecc_datain_m[63:0]),
                     .in0(byp_ecc_rcc_data_m[63:0]),
                     .in1(byp_alu_rs2_data_m[63:0]),
                     .in2(exu_lsu_rs3_data_m[63:0]),
                     .sel0(sel_rs1_m),
                     .sel1(sel_rs2_m),
                     .sel2(sel_rs3_m));

   mux3ds #(7) syn_log_mux(.dout(exu_ifu_err_synd_m[6:0]),
                           .in0(rs1_err_m[6:0]),
                           .in1(rs2_err_m[6:0]),
                           .in2(rs3_err_m[6:0]),
                           .sel0(ecl_ecc_log_rs1_m),
                           .sel1(ecl_ecc_log_rs2_m),
                           .sel2(ecl_ecc_log_rs3_m));
   // Decode syndrome from checker
   sparc_exu_ecc_dec decode(.e          (error_data_m[63:0]),
                            .q          (err_m[6:0]));
   assign       ecc_byp_ecc_result_m[63:0] = ecc_datain_m[63:0] ^ error_data_m[63:0];


endmodule // sparc_exu_ecc
// Modified by Princeton University on June 9th, 2015
// ========== Copyright Header Begin ==========================================
// 
// OpenSPARC T1 Processor File: sparc_exu_ecc_dec.v
// Copyright (c) 2006 Sun Microsystems, Inc.  All Rights Reserved.
// DO NOT ALTER OR REMOVE COPYRIGHT NOTICES.
// 
// The above named program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public
// License version 2 as published by the Free Software Foundation.
// 
// The above named program is distributed in the hope that it will be 
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// General Public License for more details.
// 
// You should have received a copy of the GNU General Public
// License along with this work; if not, write to the Free Software
// Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA 02110-1301, USA.
// 
// ========== Copyright Header End ============================================
////////////////////////////////////////////////////////////////////////
/*
//  Module Name: sparc_exu_ecc_dec
//	Description:  Decodes the result from the ecc checking block
// 			into a 64 bit value that is used to correct single bit errors.
//			Correction is performed by e ^ data.
*/
module sparc_exu_ecc_dec (/*AUTOARG*/
   // Outputs
   e, 
   // Inputs
   q
   ) ; 
   input [6:0] q;
   output [63:0] e;

   assign e[0] = ~q[6] & ~q[5] & ~q[4] & ~q[3] & ~q[2] & q[1] & q[0];
   assign e[1] = ~q[6] & ~q[5] & ~q[4] & ~q[3] & q[2] & ~q[1] & q[0];
   assign e[2] = ~q[6] & ~q[5] & ~q[4] & ~q[3] & q[2] & q[1] & ~q[0];
   assign e[3] = ~q[6] & ~q[5] & ~q[4] & ~q[3] & q[2] & q[1] & q[0];
   assign e[4] = ~q[6] & ~q[5] & ~q[4] & q[3] & ~q[2] & ~q[1] & q[0];
   assign e[5] = ~q[6] & ~q[5] & ~q[4] & q[3] & ~q[2] & q[1] & ~q[0];
   assign e[6] = ~q[6] & ~q[5] & ~q[4] & q[3] & ~q[2] & q[1] & q[0];
   assign e[7] = ~q[6] & ~q[5] & ~q[4] & q[3] & q[2] & ~q[1] & ~q[0];
   assign e[8] = ~q[6] & ~q[5] & ~q[4] & q[3] & q[2] & ~q[1] & q[0];
   assign e[9] = ~q[6] & ~q[5] & ~q[4] & q[3] & q[2] & q[1] & ~q[0];
   assign e[10] = ~q[6] & ~q[5] & ~q[4] & q[3] & q[2] & q[1] & q[0];
   assign e[11] = ~q[6] & ~q[5] & q[4] & ~q[3] & ~q[2] & ~q[1] & q[0];
   assign e[12] = ~q[6] & ~q[5] & q[4] & ~q[3] & ~q[2] & q[1] & ~q[0];
   assign e[13] = ~q[6] & ~q[5] & q[4] & ~q[3] & ~q[2] & q[1] & q[0];
   assign e[14] = ~q[6] & ~q[5] & q[4] & ~q[3] & q[2] & ~q[1] & ~q[0];
   assign e[15] = ~q[6] & ~q[5] & q[4] & ~q[3] & q[2] & ~q[1] & q[0];
   assign e[16] = ~q[6] & ~q[5] & q[4] & ~q[3] & q[2] & q[1] & ~q[0];
   assign e[17] = ~q[6] & ~q[5] & q[4] & ~q[3] & q[2] & q[1] & q[0];
   assign e[18] = ~q[6] & ~q[5] & q[4] & q[3] & ~q[2] & ~q[1] & ~q[0];
   assign e[19] = ~q[6] & ~q[5] & q[4] & q[3] & ~q[2] & ~q[1] & q[0];
   assign e[20] = ~q[6] & ~q[5] & q[4] & q[3] & ~q[2] & q[1] & ~q[0];
   assign e[21] = ~q[6] & ~q[5] & q[4] & q[3] & ~q[2] & q[1] & q[0];
   assign e[22] = ~q[6] & ~q[5] & q[4] & q[3] & q[2] & ~q[1] & ~q[0];
   assign e[23] = ~q[6] & ~q[5] & q[4] & q[3] & q[2] & ~q[1] & q[0];
   assign e[24] = ~q[6] & ~q[5] & q[4] & q[3] & q[2] & q[1] & ~q[0];
   assign e[25] = ~q[6] & ~q[5] & q[4] & q[3] & q[2] & q[1] & q[0];
   assign e[26] = ~q[6] & q[5] & ~q[4] & ~q[3] & ~q[2] & ~q[1] & q[0];
   assign e[27] = ~q[6] & q[5] & ~q[4] & ~q[3] & ~q[2] & q[1] & ~q[0];
   assign e[28] = ~q[6] & q[5] & ~q[4] & ~q[3] & ~q[2] & q[1] & q[0];
   assign e[29] = ~q[6] & q[5] & ~q[4] & ~q[3] & q[2] & ~q[1] & ~q[0];
   assign e[30] = ~q[6] & q[5] & ~q[4] & ~q[3] & q[2] & ~q[1] & q[0];
   assign e[31] = ~q[6] & q[5] & ~q[4] & ~q[3] & q[2] & q[1] & ~q[0];
   assign e[32] = ~q[6] & q[5] & ~q[4] & ~q[3] & q[2] & q[1] & q[0];
   assign e[33] = ~q[6] & q[5] & ~q[4] & q[3] & ~q[2] & ~q[1] & ~q[0];
   assign e[34] = ~q[6] & q[5] & ~q[4] & q[3] & ~q[2] & ~q[1] & q[0];
   assign e[35] = ~q[6] & q[5] & ~q[4] & q[3] & ~q[2] & q[1] & ~q[0];
   assign e[36] = ~q[6] & q[5] & ~q[4] & q[3] & ~q[2] & q[1] & q[0];
   assign e[37] = ~q[6] & q[5] & ~q[4] & q[3] & q[2] & ~q[1] & ~q[0];
   assign e[38] = ~q[6] & q[5] & ~q[4] & q[3] & q[2] & ~q[1] & q[0];
   assign e[39] = ~q[6] & q[5] & ~q[4] & q[3] & q[2] & q[1] & ~q[0];
   assign e[40] = ~q[6] & q[5] & ~q[4] & q[3] & q[2] & q[1] & q[0];
   assign e[41] = ~q[6] & q[5] & q[4] & ~q[3] & ~q[2] & ~q[1] & ~q[0];
   assign e[42] = ~q[6] & q[5] & q[4] & ~q[3] & ~q[2] & ~q[1] & q[0];
   assign e[43] = ~q[6] & q[5] & q[4] & ~q[3] & ~q[2] & q[1] & ~q[0];
   assign e[44] = ~q[6] & q[5] & q[4] & ~q[3] & ~q[2] & q[1] & q[0];
   assign e[45] = ~q[6] & q[5] & q[4] & ~q[3] & q[2] & ~q[1] & ~q[0];
   assign e[46] = ~q[6] & q[5] & q[4] & ~q[3] & q[2] & ~q[1] & q[0];
   assign e[47] = ~q[6] & q[5] & q[4] & ~q[3] & q[2] & q[1] & ~q[0];
   assign e[48] = ~q[6] & q[5] & q[4] & ~q[3] & q[2] & q[1] & q[0];
   assign e[49] = ~q[6] & q[5] & q[4] & q[3] & ~q[2] & ~q[1] & ~q[0];
   assign e[50] = ~q[6] & q[5] & q[4] & q[3] & ~q[2] & ~q[1] & q[0];
   assign e[51] = ~q[6] & q[5] & q[4] & q[3] & ~q[2] & q[1] & ~q[0];
   assign e[52] = ~q[6] & q[5] & q[4] & q[3] & ~q[2] & q[1] & q[0];
   assign e[53] = ~q[6] & q[5] & q[4] & q[3] & q[2] & ~q[1] & ~q[0];
   assign e[54] = ~q[6] & q[5] & q[4] & q[3] & q[2] & ~q[1] & q[0];
   assign e[55] = ~q[6] & q[5] & q[4] & q[3] & q[2] & q[1] & ~q[0];
   assign e[56] = ~q[6] & q[5] & q[4] & q[3] & q[2] & q[1] & q[0];
   assign e[57] = q[6] & ~q[5] & ~q[4] & ~q[3] & ~q[2] & ~q[1] & q[0];
   assign e[58] = q[6] & ~q[5] & ~q[4] & ~q[3] & ~q[2] & q[1] & ~q[0];
   assign e[59] = q[6] & ~q[5] & ~q[4] & ~q[3] & ~q[2] & q[1] & q[0];
   assign e[60] = q[6] & ~q[5] & ~q[4] & ~q[3] & q[2] & ~q[1] & ~q[0];
   assign e[61] = q[6] & ~q[5] & ~q[4] & ~q[3] & q[2] & ~q[1] & q[0];
   assign e[62] = q[6] & ~q[5] & ~q[4] & ~q[3] & q[2] & q[1] & ~q[0];
   assign e[63] = q[6] & ~q[5] & ~q[4] & ~q[3] & q[2] & q[1] & q[0];
   
endmodule // sparc_exu_ecc_dec
// Modified by Princeton University on June 9th, 2015
// ========== Copyright Header Begin ==========================================
// 
// OpenSPARC T1 Processor File: sparc_exu_ecl.v
// Copyright (c) 2006 Sun Microsystems, Inc.  All Rights Reserved.
// DO NOT ALTER OR REMOVE COPYRIGHT NOTICES.
// 
// The above named program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public
// License version 2 as published by the Free Software Foundation.
// 
// The above named program is distributed in the hope that it will be 
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// General Public License for more details.
// 
// You should have received a copy of the GNU General Public
// License along with this work; if not, write to the Free Software
// Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA 02110-1301, USA.
// 
// ========== Copyright Header End ============================================
////////////////////////////////////////////////////////////////////////
/*
//  Module Name: sparc_exu_ecl
//	Description:  Implements all the control logic for the exu.
//		This includes: bypass logic, ccr control and ccr bypassing,
//			  w2 arbitration logic, mux selects for alu and shift.
//				Also implements the ccrs.
*/

module sparc_exu_ecl
(/*AUTOARG*/
   // Outputs
   exu_tlu_ccr3_w, exu_tlu_ccr2_w, exu_tlu_ccr1_w, exu_tlu_ccr0_w, 
   exu_mul_input_vld, exu_ifu_longop_done_g, exu_ifu_inj_ack, 
   exu_ifu_err_synd_7_m, exu_ifu_err_reg_m, exu_ifu_ecc_ue_m, 
   exu_ifu_ecc_ce_m, exu_ffu_wsr_inst_e, ecl_rml_wstate_wen_w, 
   ecl_rml_otherwin_wen_w, ecl_rml_cwp_wen_e, ecl_rml_cleanwin_wen_w, 
   ecl_rml_cansave_wen_w, ecl_rml_canrestore_wen_w, 
   ecl_ecc_sel_rs3_m_l, ecl_ecc_sel_rs2_m_l, ecl_ecc_sel_rs1_m_l, 
   ecl_ecc_log_rs3_m, ecl_ecc_log_rs2_m, ecl_ecc_log_rs1_m, 
   ecl_div_yreg_wen_w, ecl_div_yreg_wen_l, ecl_div_yreg_wen_g, 
   ecl_div_yreg_shift_g, ecl_div_xinmask, ecl_div_upper33_zero, 
   ecl_div_upper33_one, ecl_div_upper32_zero, ecl_div_subtract_l, 
   ecl_div_sel_u32, ecl_div_sel_pos32, ecl_div_sel_neg32, 
   ecl_div_sel_adder, ecl_div_sel_64b, ecl_div_newq, 
   ecl_div_mul_sext_rs2_e, ecl_div_mul_sext_rs1_e, 
   ecl_div_mul_keep_data, ecl_div_mul_get_new_data, 
   ecl_div_mul_get_32bit_data, ecl_div_last_cycle, ecl_div_keepx, 
   ecl_div_keep_d, ecl_div_dividend_sign, ecl_div_cin, 
   ecl_div_almostlast_cycle, ecl_byp_sel_restore_m, 
   ecl_byp_sel_restore_g, ecl_byp_sel_pipe_m, ecl_byp_sel_muldiv_g, 
   ecl_byp_sel_load_m, ecl_byp_sel_load_g, ecl_byp_eclpr_e, 
   ecl_byp_ecc_mask_m_l, so, ecl_byp_sel_alu_e, ecl_byp_sel_eclpr_e, 
   ecl_byp_sel_yreg_e, ecl_byp_sel_ifusr_e, ecl_byp_sel_ffusr_m, 
   ecl_byp_sel_ifex_m, ecl_byp_sel_tlusr_m, exu_ifu_va_oor_m, 
   ecl_alu_out_sel_sum_e_l, ecl_alu_out_sel_rs3_e_l, 
   ecl_alu_out_sel_shift_e_l, ecl_alu_out_sel_logic_e_l, 
   ecl_alu_log_sel_and_e, ecl_alu_log_sel_or_e, 
   ecl_alu_log_sel_xor_e, ecl_alu_log_sel_move_e, 
   ecl_alu_sethi_inst_e, ecl_alu_cin_e, ecl_shft_lshift_e_l, 
   ecl_shft_op32_e, ecl_shft_shift4_e, ecl_shft_shift1_e, 
   ecl_shft_enshift_e_l, ecl_byp_restore_m, ecl_byp_rs1_mux2_sel_e, 
   ecl_byp_rs1_mux2_sel_rf, ecl_byp_rs1_mux2_sel_ld, 
   ecl_byp_rs1_mux2_sel_usemux1, ecl_byp_rs1_mux1_sel_m, 
   ecl_byp_rs1_mux1_sel_w, ecl_byp_rs1_mux1_sel_w2, 
   ecl_byp_rs1_mux1_sel_other, ecl_byp_rcc_mux2_sel_e, 
   ecl_byp_rcc_mux2_sel_rf, ecl_byp_rcc_mux2_sel_ld, 
   ecl_byp_rcc_mux2_sel_usemux1, ecl_byp_rcc_mux1_sel_m, 
   ecl_byp_rcc_mux1_sel_w, ecl_byp_rcc_mux1_sel_w2, 
   ecl_byp_rcc_mux1_sel_other, ecl_byp_rs2_mux2_sel_e, 
   ecl_byp_rs2_mux2_sel_rf, ecl_byp_rs2_mux2_sel_ld, 
   ecl_byp_rs2_mux2_sel_usemux1, ecl_byp_rs2_mux1_sel_m, 
   ecl_byp_rs2_mux1_sel_w, ecl_byp_rs2_mux1_sel_w2, 
   ecl_byp_rs2_mux1_sel_other, ecl_byp_rs3_mux2_sel_e, 
   ecl_byp_rs3_mux2_sel_rf, ecl_byp_rs3_mux2_sel_ld, 
   ecl_byp_rs3_mux2_sel_usemux1, ecl_byp_rs3_mux1_sel_m, 
   ecl_byp_rs3_mux1_sel_w, ecl_byp_rs3_mux1_sel_w2, 
   ecl_byp_rs3_mux1_sel_other, ecl_byp_rs3h_mux2_sel_e, 
   ecl_byp_rs3h_mux2_sel_rf, ecl_byp_rs3h_mux2_sel_ld, 
   ecl_byp_rs3h_mux2_sel_usemux1, ecl_byp_rs3h_mux1_sel_m, 
   ecl_byp_rs3h_mux1_sel_w, ecl_byp_rs3h_mux1_sel_w2, 
   ecl_byp_rs3h_mux1_sel_other, ecl_byp_rs1_longmux_sel_g2, 
   ecl_byp_rs1_longmux_sel_w2, ecl_byp_rs1_longmux_sel_ldxa, 
   ecl_byp_rs2_longmux_sel_g2, ecl_byp_rs2_longmux_sel_w2, 
   ecl_byp_rs2_longmux_sel_ldxa, ecl_byp_rs3_longmux_sel_g2, 
   ecl_byp_rs3_longmux_sel_w2, ecl_byp_rs3_longmux_sel_ldxa, 
   ecl_byp_rs3h_longmux_sel_g2, ecl_byp_rs3h_longmux_sel_w2, 
   ecl_byp_rs3h_longmux_sel_ldxa, ecl_byp_std_e_l, ecl_byp_ldxa_g, 
   ecl_byp_3lsb_m, ecl_ecc_rs1_use_rf_e, ecl_ecc_rs2_use_rf_e, 
   ecl_ecc_rs3_use_rf_e, ecl_irf_rd_m, ecl_irf_tid_m, ecl_irf_wen_w, 
   ecl_irf_wen_w2, ecl_irf_rd_g, ecl_irf_tid_g, ecl_div_thr_e, 
   ecl_rml_thr_m, ecl_rml_thr_w, ecl_rml_xor_data_e, 
   ecl_div_ld_inputs, ecl_div_sel_div, ecl_div_div64, exu_ifu_cc_d, 
   ecl_shft_extendbit_e, ecl_shft_extend32bit_e_l, 
   ecl_div_zero_rs2_e, ecl_div_muls_rs1_31_e_l, 
   ecl_div_yreg_data_31_g, exu_tlu_va_oor_m, exu_tlu_va_oor_jl_ret_m, 
   ecl_rml_kill_e, ecl_rml_kill_w, ecl_byp_sel_ecc_m, 
   exu_tlu_ttype_m, exu_tlu_ttype_vld_m, exu_tlu_ue_trap_m, 
   exu_tlu_misalign_addr_jmpl_rtn_m, exu_lsu_priority_trap_m, 
   ecl_div_mul_wen, ecl_div_muls, ecl_rml_early_flush_w, 
   ecl_rml_inst_vld_w, ecl_alu_casa_e, 
   // Inputs
   tlu_exu_cwpccr_update_m, tlu_exu_ccr_m, sehold, rst_tri_en, 
   rml_ecl_wstate_d, rml_ecl_swap_done, rml_ecl_rmlop_done_e, 
   rml_ecl_otherwin_d, rml_ecl_kill_m, rml_ecl_gl_e, rml_ecl_cwp_d, 
   rml_ecl_cleanwin_d, rml_ecl_cansave_d, rml_ecl_canrestore_d, 
   mul_exu_ack, lsu_exu_ldst_miss_g2, ifu_tlu_wsr_inst_d, 
   ifu_tlu_sraddr_d, ifu_exu_return_d, ifu_exu_muldivop_d, 
   ifu_exu_inst_vld_w, ifu_exu_inst_vld_e, ifu_exu_inj_irferr, 
   ifu_exu_ecc_mask, ifu_exu_disable_ce_e, ecc_ecl_rs3_ue, 
   ecc_ecl_rs3_ce, ecc_ecl_rs2_ue, ecc_ecl_rs2_ce, ecc_ecl_rs1_ue, 
   ecc_ecl_rs1_ce, div_ecl_xin_msb_l, div_ecl_x_msb, 
   div_ecl_upper32_equal, div_ecl_low32_nonzero, 
   div_ecl_gencc_in_msb_l, div_ecl_gencc_in_31, div_ecl_dividend_msb, 
   div_ecl_detect_zero_low, div_ecl_detect_zero_high, div_ecl_d_msb, 
   div_ecl_d_62, div_ecl_cout64, div_ecl_cout32, 
   div_ecl_adder_out_31, byp_ecl_wrccr_data_w, rclk, se, si, grst_l, 
   arst_l, ifu_exu_dbrinst_d, ifu_exu_aluop_d, ifu_exu_shiftop_d, 
   ifu_exu_invert_d, ifu_exu_usecin_d, ifu_exu_enshift_d, 
   byp_ecl_rs2_3_0_e, byp_ecl_rs1_2_0_e, byp_ecl_rd_data_3lsb_m, 
   ifu_exu_use_rsr_e_l, ifu_exu_rd_exusr_e, ifu_exu_rd_ifusr_e, 
   ifu_exu_rd_ffusr_e, ifu_exu_rs1_vld_d, ifu_exu_rs2_vld_d, 
   ifu_exu_rs3e_vld_d, ifu_exu_rs3o_vld_d, ifu_exu_dontmv_regz0_e, 
   ifu_exu_dontmv_regz1_e, ifu_exu_rd_d, ifu_exu_tid_s2, 
   ifu_exu_kill_e, ifu_exu_wen_d, ifu_exu_ialign_d, exu_ifu_regz_e, 
   alu_ecl_add_n64_e, alu_ecl_add_n32_e, alu_ecl_log_n64_e, 
   alu_ecl_log_n32_e, alu_ecl_zhigh_e, alu_ecl_zlow_e, 
   ifu_exu_setcc_d, lsu_exu_dfill_vld_g, lsu_exu_rd_m, lsu_exu_thr_m, 
   lsu_exu_ldxa_m, byp_ecl_rs1_31_e, byp_ecl_rs2_31_e, 
   byp_ecl_rs1_63_e, alu_ecl_cout64_e_l, alu_ecl_cout32_e, 
   alu_ecl_adder_out_63_e, alu_ecl_adder_out_31_e, 
   alu_ecl_adderin2_63_e, alu_ecl_adderin2_31_e, ifu_exu_rs1_s, 
   ifu_exu_rs2_s, ifu_exu_rs3_s, ifu_exu_tagop_d, ifu_exu_tv_d, 
   ifu_exu_muls_d, div_ecl_yreg_0_l, alu_ecl_mem_addr_invalid_e_l, 
   ifu_exu_range_check_jlret_d, ifu_exu_range_check_other_d, 
   ifu_exu_addr_mask_d, ifu_exu_save_d, ifu_exu_restore_d, 
   ifu_exu_casa_d, rml_ecl_clean_window_e, rml_ecl_fill_e, 
   rml_ecl_other_e, rml_ecl_wtype_e, ifu_exu_tcc_e, 
   alu_ecl_adder_out_7_0_e, ifu_exu_useimm_d, ifu_exu_nceen_e, 
   ifu_tlu_flush_m, ifu_exu_ttype_vld_m, tlu_exu_priv_trap_m, 
   tlu_exu_pic_onebelow_m, tlu_exu_pic_twobelow_m, 
   lsu_exu_flush_pipe_w, ifu_exu_sethi_inst_d, 
   lsu_exu_st_dtlb_perr_g
   );
   
/*AUTOINPUT*/
// Beginning of automatic inputs (from unused autoinst inputs)
input [7:0]             byp_ecl_wrccr_data_w;   // To ccr of sparc_exu_eclccr.v
input                   div_ecl_adder_out_31;   // To divcntl of sparc_exu_ecl_divcntl.v
input                   div_ecl_cout32;         // To divcntl of sparc_exu_ecl_divcntl.v
input                   div_ecl_cout64;         // To divcntl of sparc_exu_ecl_divcntl.v
input                   div_ecl_d_62;           // To divcntl of sparc_exu_ecl_divcntl.v
input                   div_ecl_d_msb;          // To divcntl of sparc_exu_ecl_divcntl.v
input                   div_ecl_detect_zero_high;// To divcntl of sparc_exu_ecl_divcntl.v, ...
input                   div_ecl_detect_zero_low;// To divcntl of sparc_exu_ecl_divcntl.v, ...
input                   div_ecl_dividend_msb;   // To divcntl of sparc_exu_ecl_divcntl.v
input                   div_ecl_gencc_in_31;    // To divcntl of sparc_exu_ecl_divcntl.v
input                   div_ecl_gencc_in_msb_l; // To divcntl of sparc_exu_ecl_divcntl.v
input                   div_ecl_low32_nonzero;  // To divcntl of sparc_exu_ecl_divcntl.v
input                   div_ecl_upper32_equal;  // To divcntl of sparc_exu_ecl_divcntl.v
input                   div_ecl_x_msb;          // To divcntl of sparc_exu_ecl_divcntl.v
input                   div_ecl_xin_msb_l;      // To divcntl of sparc_exu_ecl_divcntl.v
input                   ecc_ecl_rs1_ce;         // To eccctl of sparc_exu_ecl_eccctl.v
input                   ecc_ecl_rs1_ue;         // To eccctl of sparc_exu_ecl_eccctl.v
input                   ecc_ecl_rs2_ce;         // To eccctl of sparc_exu_ecl_eccctl.v
input                   ecc_ecl_rs2_ue;         // To eccctl of sparc_exu_ecl_eccctl.v
input                   ecc_ecl_rs3_ce;         // To eccctl of sparc_exu_ecl_eccctl.v
input                   ecc_ecl_rs3_ue;         // To eccctl of sparc_exu_ecl_eccctl.v
input                   ifu_exu_disable_ce_e;   // To eccctl of sparc_exu_ecl_eccctl.v
input [7:0]             ifu_exu_ecc_mask;       // To eccctl of sparc_exu_ecl_eccctl.v
input                   ifu_exu_inj_irferr;     // To eccctl of sparc_exu_ecl_eccctl.v
input                   ifu_exu_inst_vld_e;     // To writeback of sparc_exu_ecl_wb.v, ...
input                   ifu_exu_inst_vld_w;     // To ccr of sparc_exu_eclccr.v, ...
input [4:0]             ifu_exu_muldivop_d;     // To mdqctl of sparc_exu_ecl_mdqctl.v
input                   ifu_exu_return_d;       // To writeback of sparc_exu_ecl_wb.v
input [6:0]             ifu_tlu_sraddr_d;       // To writeback of sparc_exu_ecl_wb.v
input                   ifu_tlu_wsr_inst_d;     // To writeback of sparc_exu_ecl_wb.v
input                   lsu_exu_ldst_miss_g2;   // To writeback of sparc_exu_ecl_wb.v
input                   mul_exu_ack;            // To mdqctl of sparc_exu_ecl_mdqctl.v
input [2:0]             rml_ecl_canrestore_d;   // To writeback of sparc_exu_ecl_wb.v
input [2:0]             rml_ecl_cansave_d;      // To writeback of sparc_exu_ecl_wb.v
input [2:0]             rml_ecl_cleanwin_d;     // To writeback of sparc_exu_ecl_wb.v
input [2:0]             rml_ecl_cwp_d;          // To writeback of sparc_exu_ecl_wb.v, ...
input [1:0]             rml_ecl_gl_e;           // To eccctl of sparc_exu_ecl_eccctl.v
input                   rml_ecl_kill_m;         // To writeback of sparc_exu_ecl_wb.v
input [2:0]             rml_ecl_otherwin_d;     // To writeback of sparc_exu_ecl_wb.v
input                   rml_ecl_rmlop_done_e;   // To writeback of sparc_exu_ecl_wb.v
input [3:0]             rml_ecl_swap_done;      // To writeback of sparc_exu_ecl_wb.v
input [5:0]             rml_ecl_wstate_d;       // To writeback of sparc_exu_ecl_wb.v
input                   rst_tri_en;             // To eccctl of sparc_exu_ecl_eccctl.v
input                   sehold;                 // To writeback of sparc_exu_ecl_wb.v, ...
input [7:0]             tlu_exu_ccr_m;          // To ccr of sparc_exu_eclccr.v
input                   tlu_exu_cwpccr_update_m;// To ccr of sparc_exu_eclccr.v
// End of automatics
   input 				rclk;
   input        se;
   input        si;
   input        grst_l;
   input        arst_l;
   input        ifu_exu_dbrinst_d;// rs1 bypass should use pc
   input [2:0]  ifu_exu_aluop_d;// partially decoded op for exu operation
   input [2:0]  ifu_exu_shiftop_d;
   input        ifu_exu_invert_d;       // invert logic output
   input        ifu_exu_usecin_d;        // use cin for add ops
   input        ifu_exu_enshift_d;     // enable shifter
   input [3:0]  byp_ecl_rs2_3_0_e;
   input [2:0]  byp_ecl_rs1_2_0_e;
   input [2:0]  byp_ecl_rd_data_3lsb_m;
   input        ifu_exu_use_rsr_e_l;      // e stage instruction uses sr
   input        ifu_exu_rd_exusr_e;
   input        ifu_exu_rd_ifusr_e;
   input        ifu_exu_rd_ffusr_e;
   input        ifu_exu_rs1_vld_d;
   input        ifu_exu_rs2_vld_d;
   input        ifu_exu_rs3e_vld_d;
   input        ifu_exu_rs3o_vld_d;
   input        ifu_exu_dontmv_regz0_e;// a move instruction got killed
   input        ifu_exu_dontmv_regz1_e;
   input [4:0]  ifu_exu_rd_d;           // destination register
   input [1:0]  ifu_exu_tid_s2;          // thread of inst in s stage
   input        ifu_exu_kill_e;         // kill instruction in e-stage
   input        ifu_exu_wen_d;  // instruction in d-stage writes to regfile
   input        ifu_exu_ialign_d;// instruction is alignaddress
   input        exu_ifu_regz_e;
   input        alu_ecl_add_n64_e;
   input        alu_ecl_add_n32_e;
   input        alu_ecl_log_n64_e;
   input        alu_ecl_log_n32_e;
   input        alu_ecl_zhigh_e;
   input        alu_ecl_zlow_e;
   input        ifu_exu_setcc_d;
   input        lsu_exu_dfill_vld_g; // load data is valid
   input [4:0]  lsu_exu_rd_m;  // load destination register
   input [1:0]  lsu_exu_thr_m; // load thread
   input        lsu_exu_ldxa_m;
   input  byp_ecl_rs1_31_e;
   input  byp_ecl_rs2_31_e;
   input  byp_ecl_rs1_63_e;
   input       alu_ecl_cout64_e_l;
   input       alu_ecl_cout32_e;
   input       alu_ecl_adder_out_63_e;
   input       alu_ecl_adder_out_31_e;
   input       alu_ecl_adderin2_63_e;
   input       alu_ecl_adderin2_31_e;
   input [4:0]  ifu_exu_rs1_s;  // source addresses
   input [4:0]  ifu_exu_rs2_s;
   input [4:0]  ifu_exu_rs3_s;
   input        ifu_exu_tagop_d;// add or sub sets icc.v with tagged overflow
   input        ifu_exu_tv_d;   // 32 bit overflow causes exception
   input  ifu_exu_muls_d;
   input  [3:0] div_ecl_yreg_0_l;
   input  alu_ecl_mem_addr_invalid_e_l;
   input  ifu_exu_range_check_jlret_d;
   input  ifu_exu_range_check_other_d;
   input  ifu_exu_addr_mask_d;
   input      ifu_exu_save_d;
   input      ifu_exu_restore_d;
   input      ifu_exu_casa_d;
   input  rml_ecl_clean_window_e;
   input  rml_ecl_fill_e;
   input  rml_ecl_other_e;
   input  [2:0] rml_ecl_wtype_e;
   input        ifu_exu_tcc_e;
   input [7:0]  alu_ecl_adder_out_7_0_e;
   input       ifu_exu_useimm_d;
   input       ifu_exu_nceen_e;
   input       ifu_tlu_flush_m;
   input       ifu_exu_ttype_vld_m;
   input        tlu_exu_priv_trap_m;
   input        tlu_exu_pic_onebelow_m;
   input        tlu_exu_pic_twobelow_m;
   input       lsu_exu_flush_pipe_w;
   input       ifu_exu_sethi_inst_d;
   input       lsu_exu_st_dtlb_perr_g;
  
   /*AUTOOUTPUT*/
   // Beginning of automatic outputs (from unused autoinst outputs)
   output [7:0]         ecl_byp_ecc_mask_m_l;   // From eccctl of sparc_exu_ecl_eccctl.v
   output [7:0]         ecl_byp_eclpr_e;        // From writeback of sparc_exu_ecl_wb.v
   output               ecl_byp_sel_load_g;     // From writeback of sparc_exu_ecl_wb.v
   output               ecl_byp_sel_load_m;     // From writeback of sparc_exu_ecl_wb.v
   output               ecl_byp_sel_muldiv_g;   // From writeback of sparc_exu_ecl_wb.v
   output               ecl_byp_sel_pipe_m;     // From writeback of sparc_exu_ecl_wb.v
   output               ecl_byp_sel_restore_g;  // From writeback of sparc_exu_ecl_wb.v
   output               ecl_byp_sel_restore_m;  // From writeback of sparc_exu_ecl_wb.v
   output               ecl_div_almostlast_cycle;// From divcntl of sparc_exu_ecl_divcntl.v
   output               ecl_div_cin;            // From divcntl of sparc_exu_ecl_divcntl.v
   output               ecl_div_dividend_sign;  // From divcntl of sparc_exu_ecl_divcntl.v
   output               ecl_div_keep_d;         // From divcntl of sparc_exu_ecl_divcntl.v
   output               ecl_div_keepx;          // From divcntl of sparc_exu_ecl_divcntl.v
   output               ecl_div_last_cycle;     // From divcntl of sparc_exu_ecl_divcntl.v
   output               ecl_div_mul_get_32bit_data;// From mdqctl of sparc_exu_ecl_mdqctl.v
   output               ecl_div_mul_get_new_data;// From mdqctl of sparc_exu_ecl_mdqctl.v
   output               ecl_div_mul_keep_data;  // From mdqctl of sparc_exu_ecl_mdqctl.v
   output               ecl_div_mul_sext_rs1_e; // From mdqctl of sparc_exu_ecl_mdqctl.v
   output               ecl_div_mul_sext_rs2_e; // From mdqctl of sparc_exu_ecl_mdqctl.v
   output               ecl_div_newq;           // From divcntl of sparc_exu_ecl_divcntl.v
   output               ecl_div_sel_64b;        // From divcntl of sparc_exu_ecl_divcntl.v
   output               ecl_div_sel_adder;      // From divcntl of sparc_exu_ecl_divcntl.v
   output               ecl_div_sel_neg32;      // From divcntl of sparc_exu_ecl_divcntl.v
   output               ecl_div_sel_pos32;      // From divcntl of sparc_exu_ecl_divcntl.v
   output               ecl_div_sel_u32;        // From divcntl of sparc_exu_ecl_divcntl.v
   output               ecl_div_subtract_l;     // From divcntl of sparc_exu_ecl_divcntl.v
   output               ecl_div_upper32_zero;   // From divcntl of sparc_exu_ecl_divcntl.v
   output               ecl_div_upper33_one;    // From divcntl of sparc_exu_ecl_divcntl.v
   output               ecl_div_upper33_zero;   // From divcntl of sparc_exu_ecl_divcntl.v
   output               ecl_div_xinmask;        // From divcntl of sparc_exu_ecl_divcntl.v
   output [3:0]         ecl_div_yreg_shift_g;   // From writeback of sparc_exu_ecl_wb.v
   output [3:0]         ecl_div_yreg_wen_g;     // From writeback of sparc_exu_ecl_wb.v
   output [3:0]         ecl_div_yreg_wen_l;     // From writeback of sparc_exu_ecl_wb.v
   output [3:0]         ecl_div_yreg_wen_w;     // From writeback of sparc_exu_ecl_wb.v
   output               ecl_ecc_log_rs1_m;      // From eccctl of sparc_exu_ecl_eccctl.v
   output               ecl_ecc_log_rs2_m;      // From eccctl of sparc_exu_ecl_eccctl.v
   output               ecl_ecc_log_rs3_m;      // From eccctl of sparc_exu_ecl_eccctl.v
   output               ecl_ecc_sel_rs1_m_l;    // From eccctl of sparc_exu_ecl_eccctl.v
   output               ecl_ecc_sel_rs2_m_l;    // From eccctl of sparc_exu_ecl_eccctl.v
   output               ecl_ecc_sel_rs3_m_l;    // From eccctl of sparc_exu_ecl_eccctl.v
   output               ecl_rml_canrestore_wen_w;// From writeback of sparc_exu_ecl_wb.v
   output               ecl_rml_cansave_wen_w;  // From writeback of sparc_exu_ecl_wb.v
   output               ecl_rml_cleanwin_wen_w; // From writeback of sparc_exu_ecl_wb.v
   output               ecl_rml_cwp_wen_e;      // From writeback of sparc_exu_ecl_wb.v
   output               ecl_rml_otherwin_wen_w; // From writeback of sparc_exu_ecl_wb.v
   output               ecl_rml_wstate_wen_w;   // From writeback of sparc_exu_ecl_wb.v
   output               exu_ffu_wsr_inst_e;     // From writeback of sparc_exu_ecl_wb.v
   output               exu_ifu_ecc_ce_m;       // From eccctl of sparc_exu_ecl_eccctl.v
   output               exu_ifu_ecc_ue_m;       // From eccctl of sparc_exu_ecl_eccctl.v
   output [7:0]         exu_ifu_err_reg_m;      // From eccctl of sparc_exu_ecl_eccctl.v
   output               exu_ifu_err_synd_7_m;   // From eccctl of sparc_exu_ecl_eccctl.v
   output               exu_ifu_inj_ack;        // From eccctl of sparc_exu_ecl_eccctl.v
   output [3:0]         exu_ifu_longop_done_g;  // From writeback of sparc_exu_ecl_wb.v
   output               exu_mul_input_vld;      // From mdqctl of sparc_exu_ecl_mdqctl.v
   output [7:0]         exu_tlu_ccr0_w;         // From ccr of sparc_exu_eclccr.v
   output [7:0]         exu_tlu_ccr1_w;         // From ccr of sparc_exu_eclccr.v
   output [7:0]         exu_tlu_ccr2_w;         // From ccr of sparc_exu_eclccr.v
   output [7:0]         exu_tlu_ccr3_w;         // From ccr of sparc_exu_eclccr.v
   // End of automatics
   output               so;
   output               ecl_byp_sel_alu_e;
   output               ecl_byp_sel_eclpr_e;
   output               ecl_byp_sel_yreg_e;
   output               ecl_byp_sel_ifusr_e;
   output               ecl_byp_sel_ffusr_m;
   output               ecl_byp_sel_ifex_m;
   output               ecl_byp_sel_tlusr_m;
   output   exu_ifu_va_oor_m;
   output ecl_alu_out_sel_sum_e_l;
   output ecl_alu_out_sel_rs3_e_l;
   output ecl_alu_out_sel_shift_e_l;
   output ecl_alu_out_sel_logic_e_l;
   output ecl_alu_log_sel_and_e;
   output ecl_alu_log_sel_or_e;
   output ecl_alu_log_sel_xor_e;
   output ecl_alu_log_sel_move_e;
   output ecl_alu_sethi_inst_e;
   output ecl_alu_cin_e;    // cin for add/sub operations
   output  ecl_shft_lshift_e_l;  // if 0 do left shift.  else right shift
   output  ecl_shft_op32_e;      // indicates 32 bit operation so upper 32 = 0
   //output [3:0] ecl_shft_shift16_e;// [48, 32, 16, 0] shift
   output [3:0] ecl_shft_shift4_e;// [12, 8, 4, 0] shift
   output [3:0] ecl_shft_shift1_e;// [3, 2, 1, 0] shift
   output        ecl_shft_enshift_e_l;// enables inputs to shifter
   output        ecl_byp_restore_m;
   output ecl_byp_rs1_mux2_sel_e;// select lines for bypass muxes for rs1
   output ecl_byp_rs1_mux2_sel_rf;
   output ecl_byp_rs1_mux2_sel_ld;
   output ecl_byp_rs1_mux2_sel_usemux1;
   output ecl_byp_rs1_mux1_sel_m;
   output ecl_byp_rs1_mux1_sel_w;
   output ecl_byp_rs1_mux1_sel_w2;
   output ecl_byp_rs1_mux1_sel_other;
   output ecl_byp_rcc_mux2_sel_e;// select lines for bypass muxes for rcc
   output ecl_byp_rcc_mux2_sel_rf;
   output ecl_byp_rcc_mux2_sel_ld;
   output ecl_byp_rcc_mux2_sel_usemux1;
   output ecl_byp_rcc_mux1_sel_m;
   output ecl_byp_rcc_mux1_sel_w;
   output ecl_byp_rcc_mux1_sel_w2;
   output ecl_byp_rcc_mux1_sel_other;
   output ecl_byp_rs2_mux2_sel_e;// select lines for bypass muxes for rs2
   output ecl_byp_rs2_mux2_sel_rf;
   output ecl_byp_rs2_mux2_sel_ld;
   output ecl_byp_rs2_mux2_sel_usemux1;
   output ecl_byp_rs2_mux1_sel_m;
   output ecl_byp_rs2_mux1_sel_w;
   output ecl_byp_rs2_mux1_sel_w2;
   output ecl_byp_rs2_mux1_sel_other;
   output ecl_byp_rs3_mux2_sel_e; // select lines for bypass muxes for rs3
   output ecl_byp_rs3_mux2_sel_rf;
   output ecl_byp_rs3_mux2_sel_ld;
   output ecl_byp_rs3_mux2_sel_usemux1;
   output ecl_byp_rs3_mux1_sel_m;
   output ecl_byp_rs3_mux1_sel_w;
   output ecl_byp_rs3_mux1_sel_w2;
   output ecl_byp_rs3_mux1_sel_other;
   output ecl_byp_rs3h_mux2_sel_e; // select lines for bypass muxes for rs3 double
   output ecl_byp_rs3h_mux2_sel_rf;
   output ecl_byp_rs3h_mux2_sel_ld;
   output ecl_byp_rs3h_mux2_sel_usemux1;
   output ecl_byp_rs3h_mux1_sel_m;
   output ecl_byp_rs3h_mux1_sel_w;
   output ecl_byp_rs3h_mux1_sel_w2;
   output ecl_byp_rs3h_mux1_sel_other;
   output ecl_byp_rs1_longmux_sel_g2;
   output ecl_byp_rs1_longmux_sel_w2;
   output ecl_byp_rs1_longmux_sel_ldxa;
   output ecl_byp_rs2_longmux_sel_g2;
   output ecl_byp_rs2_longmux_sel_w2;
   output ecl_byp_rs2_longmux_sel_ldxa;
   output ecl_byp_rs3_longmux_sel_g2;
   output ecl_byp_rs3_longmux_sel_w2;
   output ecl_byp_rs3_longmux_sel_ldxa;
   output ecl_byp_rs3h_longmux_sel_g2;
   output ecl_byp_rs3h_longmux_sel_w2;
   output ecl_byp_rs3h_longmux_sel_ldxa;
   output ecl_byp_std_e_l;
   output ecl_byp_ldxa_g;       // use the ldxa return data
   output [2:0] ecl_byp_3lsb_m;
   output                ecl_ecc_rs1_use_rf_e;
   output                ecl_ecc_rs2_use_rf_e;
   output                ecl_ecc_rs3_use_rf_e;
   output  [4:0] ecl_irf_rd_m;
   output  [1:0] ecl_irf_tid_m;
   output        ecl_irf_wen_w;
   output        ecl_irf_wen_w2;// write enable for w2
   output  [4:0] ecl_irf_rd_g; // w2 destination register
   output [1:0]  ecl_irf_tid_g;     // thread of inst in long w stage
   output [3:0]  ecl_div_thr_e;
   output [3:0] ecl_rml_thr_m;
   output [3:0] ecl_rml_thr_w;
   output [2:0] ecl_rml_xor_data_e;
   output        ecl_div_ld_inputs;
   output        ecl_div_sel_div;
   output        ecl_div_div64;
   output [7:0]  exu_ifu_cc_d;
   output ecl_shft_extendbit_e;     // bit that gets appended on right shifts
   output ecl_shft_extend32bit_e_l;   // bit that gets appended on 32 bit right shfts
   output         ecl_div_zero_rs2_e;// used on muls ops
   output         ecl_div_muls_rs1_31_e_l;
   output         ecl_div_yreg_data_31_g;
   output         exu_tlu_va_oor_m;
   output         exu_tlu_va_oor_jl_ret_m;
   output        ecl_rml_kill_e;
   output        ecl_rml_kill_w;
   output        ecl_byp_sel_ecc_m;
   output [8:0] exu_tlu_ttype_m;
   output       exu_tlu_ttype_vld_m;
   output       exu_tlu_ue_trap_m;
   output   exu_tlu_misalign_addr_jmpl_rtn_m;
   output   exu_lsu_priority_trap_m;
   output   ecl_div_mul_wen;
   output   ecl_div_muls;
   output   ecl_rml_early_flush_w;
   output   ecl_rml_inst_vld_w;
   output   ecl_alu_casa_e;

   
   /*AUTOWIRE*/
   // Beginning of automatic wires (for undeclared instantiated-module outputs)
   wire                 bypass_m;               // From writeback of sparc_exu_ecl_wb.v
   wire                 bypass_w;               // From writeback of sparc_exu_ecl_wb.v
   wire [7:0]           divcntl_ccr_cc_w2;      // From divcntl of sparc_exu_ecl_divcntl.v
   wire                 divcntl_wb_req_g;       // From divcntl of sparc_exu_ecl_divcntl.v
   wire [4:0]           eccctl_wb_rd_m;         // From eccctl of sparc_exu_ecl_eccctl.v
   wire                 ecl_div_signed_div;     // From mdqctl of sparc_exu_ecl_mdqctl.v
   wire                 mdqctl_divcntl_input_vld;// From mdqctl of sparc_exu_ecl_mdqctl.v
   wire                 mdqctl_divcntl_muldone; // From mdqctl of sparc_exu_ecl_mdqctl.v
   wire                 mdqctl_divcntl_reset_div;// From mdqctl of sparc_exu_ecl_mdqctl.v
   wire [4:0]           mdqctl_wb_divrd_g;      // From mdqctl of sparc_exu_ecl_mdqctl.v
   wire                 mdqctl_wb_divsetcc_g;   // From mdqctl of sparc_exu_ecl_mdqctl.v
   wire [1:0]           mdqctl_wb_divthr_g;     // From mdqctl of sparc_exu_ecl_mdqctl.v
   wire [4:0]           mdqctl_wb_mulrd_g;      // From mdqctl of sparc_exu_ecl_mdqctl.v
   wire                 mdqctl_wb_mulsetcc_g;   // From mdqctl of sparc_exu_ecl_mdqctl.v
   wire [1:0]           mdqctl_wb_multhr_g;     // From mdqctl of sparc_exu_ecl_mdqctl.v
   wire                 mdqctl_wb_yreg_shift_g; // From mdqctl of sparc_exu_ecl_mdqctl.v
   wire                 mdqctl_wb_yreg_wen_g;   // From mdqctl of sparc_exu_ecl_mdqctl.v
   wire [4:0]           wb_byplog_rd_g2;        // From writeback of sparc_exu_ecl_wb.v
   wire [4:0]           wb_byplog_rd_w2;        // From writeback of sparc_exu_ecl_wb.v
   wire [1:0]           wb_byplog_tid_w2;       // From writeback of sparc_exu_ecl_wb.v
   wire                 wb_byplog_wen_g2;       // From writeback of sparc_exu_ecl_wb.v
   wire                 wb_byplog_wen_w2;       // From writeback of sparc_exu_ecl_wb.v
   wire                 wb_ccr_setcc_g;         // From writeback of sparc_exu_ecl_wb.v
   wire                 wb_ccr_wrccr_w;         // From writeback of sparc_exu_ecl_wb.v
   wire                 wb_divcntl_ack_g;       // From writeback of sparc_exu_ecl_wb.v
   wire                 wb_e;                   // From writeback of sparc_exu_ecl_wb.v
   wire                 wb_eccctl_spec_wen_next;// From writeback of sparc_exu_ecl_wb.v
   // End of automatics
   wire                 clk;
   wire                 reset;
   wire                 ecl_reset_l;
   wire                ecl_byp_rs1_mux2_sel_rf;// To eccctl of sparc_exu_ecl_eccctl.v
   wire                ecl_byp_rs2_mux2_sel_rf;// To eccctl of sparc_exu_ecl_eccctl.v
   wire                ecl_byp_rs3_mux2_sel_rf;// To eccctl of sparc_exu_ecl_eccctl.v
   wire                ldxa_g;
   wire                ecl_byp_ldxa_g;
   wire                rs1_vld_e;
   wire                rs2_vld_e;
   wire                std_d;
   wire                std_e;
   wire                rs3_vld_d;
   wire                rs3_vld_e;
   wire                cancel_rs3_ecc_e;
   wire [4:0]  ifu_exu_rs1_d;  // source addresses
   wire [4:0]  ifu_exu_rs2_d;
   wire [4:0]  ifu_exu_rs3_d;
   wire [2:0]   ifu_exu_aluop_e;
   wire [2:0]   shiftop_d;
   wire [2:0]   shiftop_e;
   wire         enshift_e;
   wire         sel_sum_d;
   wire         sel_sum_e;
   wire         sub_e;        // Do subtraction for add ops
   wire         shft_sext_e;     // sign extend for R shift.  must be 0 for left
   wire         is_logic_e;       // opcode is for logic op
   wire         dont_move_e;
   wire         sethi_e;
   wire [4:0]   rd_e;
   wire  [4:0] rd_m;
   wire  [4:0] ecl_irf_rd_w;
   wire [1:0]  tid_d;
   wire [3:0] thr_d;
   wire [1:0]  tid_e;
   wire  [1:0] tid_m;
   wire [1:0]  tid_w;
   wire [1:0]  tid_w1;
   wire  [1:0] ecl_irf_tid_w;
   wire [3:0]  thr_m;
   wire [3:0]  ecl_rml_thr_w;
   wire        ecl_irf_wen_w;
   wire          extend64bit;   // bit that gets appended on 64 bit right shfts
   wire         c_used_d;       // actual c_in calculated in d_stage
   wire [1:0]  adder_xcc;
   wire [1:0]  adder_icc;
   wire        cc_e_3;          // cc_e for muls
   wire        cc_e_1;
   wire [3:0]  alu_xcc_e; // 64 bit ccs NZVC
   wire [3:0]  alu_icc_e; // 32 bit ccs NZVC
   wire        ialign_e;
   wire        ialign_m;
   wire        ifu_exu_tv_e;
   wire        ifu_exu_tagop_e;
   wire        tag_overflow;    // tag overflow has occured
   wire     tag_overflow_trap_e;   
   wire           ifu_exu_range_check_jlret_e;
   wire           ifu_exu_range_check_other_e;
   wire           addr_mask_e;
   wire           valid_range_check_jlret_e;
   wire           ifu_exu_range_check_jlret_m;
   wire           ifu_exu_range_check_other_m;
   wire           alu_ecl_mem_addr_invalid_m_l;
   wire           misalign_addr_e;
   wire           muls_rs1_31_m_l;
   wire           rs2_data_31_m;
   wire       save_e;
   wire       restore_e;
   wire [4:0] real_rd_e;
   wire       ifu_tlu_flush_w;
   wire          flush_w;
   wire          flush_w1;
   wire          part_early_flush_m;
   wire          part_early_flush_w;
   wire          pic_trap_m;
   wire          inst_vld_w1;
   wire          tlu_priv_trap_w;
   wire          early_flush_w;
   wire          thr_match_ew;
   wire          thr_match_mw1;
   wire          thr_match_mw;
   wire          thr_match_sd;
   wire          thr_match_de;
   wire          thr_match_se;
   wire          thr_match_dm;
   wire          ld_thr_match_sm;
   wire          ld_thr_match_dg;
   wire          ld_thr_match_sg;
   wire          ld_thr_match_dg2;
   wire	  	     ecl_exu_kill_m;
   wire	  	     kill_rml_m;
   wire          kill_rml_w;
   wire [3:0]    perr_store_next;
   wire [3:0]    perr_store;
   wire [3:0]    perr_kill;
   wire [4:0]    ld_rd_g;
   wire [1:0]    ld_tid_g;

   wire          read_yreg_e;
   wire          read_ffusr_e;
   wire          read_tlusr_e;
   wire          read_ffusr_m;
   wire          read_tlusr_m;
   
   // trap logic
   wire          ue_trap_m;
   wire [8:0]    early1_ttype_e;
   wire [8:0]    early2_ttype_e;
   wire [8:0]    early_ttype_m;
   wire          early_ttype_vld_e;
   wire          early_ttype_vld_m;
   wire          pick_not_aligned;
   wire          pick_tcc;
   wire          pick_normal_ttype;   
   wire          fill_trap_e;
   wire          fill_trap_m;
   wire       next_yreg_data_31;
   wire       muls_e;
   wire       zero_rs2_d;
   wire       div_e;
   wire       div_zero_m;


   wire [4:0]  ifu_exu_rs1_e;
   wire [4:0]  ifu_exu_rs1_m;
   wire [4:0]  ifu_exu_rs2_e;
   wire [4:0]  ifu_exu_rs2_m;
   wire [4:0]  ifu_exu_rs3_e;
   wire [4:0]  ifu_exu_rs3_m;
   wire [3:0]  div_ecl_yreg_0;
   wire   div_ecl_yreg_0_d;
   
   assign clk = rclk;
   // Reset flop
    dffrl_async rstff(.din (grst_l),
                        .q   (ecl_reset_l),
                        .clk (clk),
                        .rst_l (arst_l), .se(se), .si(), .so());
   assign reset = ~ecl_reset_l;
   
   // Pipeline flops for irf control signals
   dff_s #(5) dff_rs1_s2d(.din(ifu_exu_rs1_s[4:0]), .clk(clk), .q(ifu_exu_rs1_d[4:0]), .se(se),
                      .si(),.so());
   dff_s #(5) dff_rs2_s2d(.din(ifu_exu_rs2_s[4:0]), .clk(clk), .q(ifu_exu_rs2_d[4:0]), .se(se),
                      .si(),.so());
   dff_s #(5) dff_rs3_s2d(.din(ifu_exu_rs3_s[4:0]), .clk(clk), .q(ifu_exu_rs3_d[4:0]), .se(se),
                      .si(),.so());
   dff_s #(5) dff_rs1_d2e(.din(ifu_exu_rs1_d[4:0]), .clk(clk), .q(ifu_exu_rs1_e[4:0]), .se(se),
                      .si(),.so());
   dff_s #(5) dff_rs2_d2e(.din(ifu_exu_rs2_d[4:0]), .clk(clk), .q(ifu_exu_rs2_e[4:0]), .se(se),
                      .si(),.so());
   dff_s #(5) dff_rs3_d2e(.din(ifu_exu_rs3_d[4:0]), .clk(clk), .q(ifu_exu_rs3_e[4:0]), .se(se),
                      .si(),.so());
   dff_s #(5) dff_rs1_e2m(.din(ifu_exu_rs1_e[4:0]), .clk(clk), .q(ifu_exu_rs1_m[4:0]), .se(se),
                      .si(),.so());
   dff_s #(5) dff_rs2_e2m(.din(ifu_exu_rs2_e[4:0]), .clk(clk), .q(ifu_exu_rs2_m[4:0]), .se(se),
                      .si(),.so());
   dff_s #(5) dff_rs3_e2m(.din(ifu_exu_rs3_e[4:0]), .clk(clk), .q(ifu_exu_rs3_m[4:0]), .se(se),
                      .si(),.so());
   dff_s #(5) dff_ld_rd_m2g(.din(lsu_exu_rd_m[4:0]), .clk(clk), .q(ld_rd_g[4:0]), .se(se), .si(), .so());  
   dff_s #(2) dff_ld_tid_m2g(.din(lsu_exu_thr_m[1:0]), .clk(clk), .q(ld_tid_g[1:0]), .se(se), .si(), .so());
   
   // Pipeline flops for control signals
   dff_s #(3) dff_aluop_d2e(.din(ifu_exu_aluop_d[2:0]), .clk(clk), .q(ifu_exu_aluop_e[2:0]),
                        .se(se), .si(), .so());
   dff_s #(3) dff_shiftop_d2e(.din(shiftop_d[2:0]), .clk(clk),
                          .q(shiftop_e[2:0]), .se(se),
                          .si(), .so());
   dff_s dff_enshift_d2e(.din(ifu_exu_enshift_d), .clk(clk), .q(enshift_e),
                       .se(se), .si(), .so());
   dff_s dff_sel_sum_d2e(.din(sel_sum_d), .clk(clk), .q(sel_sum_e),
                       .se(se), .si(), .so());
   dff_s dff_tv_d2e(.din(ifu_exu_tv_d), .clk(clk), .q(ifu_exu_tv_e),
                  .se(se), .si(), .so());
   dff_s dff_tagop_d2e(.din(ifu_exu_tagop_d), .clk(clk), .q(ifu_exu_tagop_e),
                  .se(se), .si(), .so());
   dff_s dff_ialign_d2e(.din(ifu_exu_ialign_d), .clk(clk), .q(ialign_e),
                      .se(se), .si(), .so());
   dff_s dff_ialign_e2m(.din(ialign_e), .clk(clk), .q(ialign_m),
                      .se(se), .si(), .so());
   dff_s ldxa_dff(.din(lsu_exu_ldxa_m), .clk(clk), .q(ldxa_g), .se(se), .si(), .so());
   dff_s sethi_d2e(.din(ifu_exu_sethi_inst_d), .clk(clk), .q(sethi_e), .se(se), .si(), .so());
   dff_s rs1_vld_d2e(.din(ifu_exu_rs1_vld_d), .clk(clk), .q(rs1_vld_e), .se(se), .si(), .so());
   dff_s rs2_vld_d2e(.din(ifu_exu_rs2_vld_d), .clk(clk), .q(rs2_vld_e), .se(se), .si(), .so());
   assign rs3_vld_d = ifu_exu_rs3e_vld_d | ifu_exu_rs3o_vld_d;
   dff_s rs3_vld_d2e(.din(rs3_vld_d), .q(rs3_vld_e), .clk(clk), .se(se), .si(), .so());
   dff_s casa_d2e(.din(ifu_exu_casa_d), .q(ecl_alu_casa_e), .clk(clk), .se(se), .si(), .so());
   
   ///////////////////////////////
   // ALU Control
   ///////////////////////////////
   // Decode opcode for ALU
   // aluop: [move, log1, log0]
   // ADD = 00, AND = 01, OR = 10, XOR = 11
   // log_sel: [and, or, xor, pass]
   // out_sel: [sum, logic, shift]
   assign ecl_alu_log_sel_and_e = 
          (~ifu_exu_aluop_e[2] & ~ifu_exu_aluop_e[1] & ifu_exu_aluop_e[0]);
   assign ecl_alu_log_sel_or_e = (~ifu_exu_aluop_e[2] & ifu_exu_aluop_e[1]
                                     & ~ifu_exu_aluop_e[0]);
   assign ecl_alu_log_sel_xor_e = (~ifu_exu_aluop_e[2] & ifu_exu_aluop_e[1]
                                      & ifu_exu_aluop_e[0]);
   assign ecl_alu_log_sel_move_e = 
          (ifu_exu_aluop_e[2] | ~(ifu_exu_aluop_e[1] | ifu_exu_aluop_e[0]));

   assign is_logic_e = ifu_exu_aluop_e[2] | ifu_exu_aluop_e[1] |
          ifu_exu_aluop_e[0];

   assign ecl_alu_sethi_inst_e = sethi_e;// | ifu_exu_sethi_inst_e;

   assign dont_move_e = (exu_ifu_regz_e)? ifu_exu_dontmv_regz1_e:ifu_exu_dontmv_regz0_e;

   assign sel_sum_d = ~(ifu_exu_enshift_d | ifu_exu_aluop_d[2] |ifu_exu_aluop_d[1] |ifu_exu_aluop_d[0]); 
   assign ecl_alu_out_sel_sum_e_l = ~sel_sum_e;
   assign ecl_alu_out_sel_shift_e_l = ~(~is_logic_e & ~sel_sum_e);
   assign ecl_alu_out_sel_logic_e_l = ~(is_logic_e & ~dont_move_e & ~sel_sum_e);
   assign ecl_alu_out_sel_rs3_e_l = ~(is_logic_e & dont_move_e & ~sel_sum_e);// dontmove includes is_logic

   assign ecl_byp_sel_alu_e = ifu_exu_use_rsr_e_l;
   assign ecl_byp_sel_ifusr_e = ~ifu_exu_use_rsr_e_l & ifu_exu_rd_ifusr_e;
   assign ecl_byp_sel_yreg_e = ~ifu_exu_use_rsr_e_l & ~ifu_exu_rd_ifusr_e & read_yreg_e;
   assign ecl_byp_sel_eclpr_e = ~ifu_exu_use_rsr_e_l & ~ifu_exu_rd_ifusr_e & ~read_yreg_e;
        
   assign read_ffusr_e = ~ifu_exu_use_rsr_e_l & ifu_exu_rd_ffusr_e;
   assign read_tlusr_e = ~ifu_exu_use_rsr_e_l & ~ifu_exu_rd_ffusr_e & ~ifu_exu_rd_ifusr_e & ~ifu_exu_rd_exusr_e;
   assign ecl_byp_sel_ffusr_m = read_ffusr_m;
   assign ecl_byp_sel_tlusr_m = read_tlusr_m & ~read_ffusr_m;
   assign ecl_byp_sel_ifex_m = ~read_tlusr_m & ~read_ffusr_m;

   dff_s #(2) rsr_e2m(.din({read_ffusr_e, read_tlusr_e}), .clk(clk),
                    .q({read_ffusr_m, read_tlusr_m}), .se(se), .si(), .so());
 
   // ecc checking on rs3 will be cancelled if mov happens
   assign cancel_rs3_ecc_e = ~dont_move_e & is_logic_e;
   
   // compute xor for write to cwp
   assign ecl_rml_xor_data_e = byp_ecl_rs1_2_0_e[2:0] ^ byp_ecl_rs2_3_0_e[2:0];
   // Logic for muls control signals
   // icc.v ^ icc.n
   assign         ecl_div_muls_rs1_31_e_l = ~(cc_e_3 ^ cc_e_1);
   assign div_ecl_yreg_0[3:0] = ~div_ecl_yreg_0_l[3:0];
   mux4ds yreg0_mux(.dout(div_ecl_yreg_0_d),
                    .in0(div_ecl_yreg_0[0]),
                    .in1(div_ecl_yreg_0[1]),
                    .in2(div_ecl_yreg_0[2]),
                    .in3(div_ecl_yreg_0[3]),
                    .sel0(thr_d[0]),
                    .sel1(thr_d[1]),
                    .sel2(thr_d[2]),
                    .sel3(thr_d[3]));
                       
   assign zero_rs2_d = ifu_exu_muls_d & ~div_ecl_yreg_0_d;

   assign next_yreg_data_31 = (muls_e)? byp_ecl_rs1_2_0_e[0]:ecl_div_yreg_data_31_g; 
   dff_s dff_rs1_b0_m2w(.din(next_yreg_data_31), .clk(clk), .q(ecl_div_yreg_data_31_g),
                .se(se), .si(), .so());
   
   // Logic for carryin and subtract
   assign      c_used_d = ~(ifu_exu_invert_d ^ ~(exu_ifu_cc_d[0] & ifu_exu_usecin_d));
   // Pipeline flops
   dff_s sub_dff(.din(ifu_exu_invert_d), .clk(clk), .q(sub_e), .se(se),
               .si(), .so());
   dff_s c_used_dff(.din(c_used_d), .clk(clk), .q(ecl_alu_cin_e), .se(se),
                 .si(), .so());
   dff_s dff_muls_d2e(.din(ifu_exu_muls_d), .clk(clk), .q(muls_e),
                .se(se), .si(), .so());
   dff_s zero_rs2_dff(.din(zero_rs2_d), .clk(clk), .q(ecl_div_zero_rs2_e),
                    .se(se), .si(), .so());
   dff_s #(2) cc_d2e(.din({exu_ifu_cc_d[3],exu_ifu_cc_d[1]}), .clk(clk), .q({cc_e_3,cc_e_1}),
                   .se(se), .si(), .so());
   dff_s mulsrs131_e2m(.din(ecl_div_muls_rs1_31_e_l), .clk(clk),
                     .q(muls_rs1_31_m_l),
                     .se(se), .si(), .so());
   dff_s rs2_31_e2m(.din(byp_ecl_rs2_31_e), .clk(clk),
                  .q(rs2_data_31_m), .se(se), .si(), .so());
   
   dff_s save_dff(.din(ifu_exu_save_d), .clk(clk), .q(save_e), .se(se),
                .si(), .so());
   dff_s restore_dff(.din(ifu_exu_restore_d), .clk(clk), .q(restore_e), .se(se),
                .si(), .so());
   
   // Condition code generation
   assign      adder_xcc[0] = (~alu_ecl_cout64_e_l ^ sub_e) & sel_sum_e;
   assign      adder_icc[0] = (alu_ecl_cout32_e ^ sub_e) & sel_sum_e;
/* -----\/----- EXCLUDED -----\/-----
   assign adder_xcc[1] = ((byp_ecl_rs1_63_e & alu_ecl_adderin2_63_e & 
                             ~alu_ecl_adder_out_63_e) |
                           (~byp_ecl_rs1_63_e & ~alu_ecl_adderin2_63_e &
                             alu_ecl_adder_out_63_e));
   assign adder_icc[1] = ((byp_ecl_rs1_31_e & alu_ecl_adderin2_31_e & 
                             ~alu_ecl_adder_out_31_e) |
                           (~byp_ecl_rs1_31_e & ~alu_ecl_adderin2_31_e &
                             alu_ecl_adder_out_31_e));
 -----/\----- EXCLUDED -----/\----- */
   assign adder_xcc[1] = (alu_ecl_adder_out_63_e) ? (~byp_ecl_rs1_63_e & ~alu_ecl_adderin2_63_e & sel_sum_e):
                                                      (byp_ecl_rs1_63_e & alu_ecl_adderin2_63_e & sel_sum_e);
   assign adder_icc[1] = (alu_ecl_adder_out_31_e) ? ((~byp_ecl_rs1_31_e & ~alu_ecl_adderin2_31_e | tag_overflow) 
                                                     & sel_sum_e):
                                                      ((byp_ecl_rs1_31_e & alu_ecl_adderin2_31_e | tag_overflow)
                                                       & sel_sum_e);
   // Tagged overflow
   assign tag_overflow = (byp_ecl_rs1_2_0_e[0] | byp_ecl_rs1_2_0_e[1] |
                          byp_ecl_rs2_3_0_e[0] | byp_ecl_rs2_3_0_e[1]) & ifu_exu_tagop_e;

   // Set V C ccs assuming they are 0s for logic and shifting
   assign alu_xcc_e[3] = (sel_sum_e)? alu_ecl_add_n64_e: alu_ecl_log_n64_e;
   assign alu_xcc_e[2] = alu_ecl_zlow_e & alu_ecl_zhigh_e;
   assign alu_xcc_e[1:0] = adder_xcc[1:0]; // includes sel_sum
   
   assign alu_icc_e[3] = (sel_sum_e)? alu_ecl_add_n32_e: alu_ecl_log_n32_e;
   assign alu_icc_e[2] = alu_ecl_zlow_e;
   assign alu_icc_e[1:0] = adder_icc[1:0]; // includes sel_sum

   // Tag overflow exception on TV instruction with icc.v
   assign   tag_overflow_trap_e = ifu_exu_tv_e & adder_icc[1];
   
   // Mem address exception generation and flops
   assign   misalign_addr_e = (alu_ecl_adder_out_7_0_e[1] | alu_ecl_adder_out_7_0_e[0]) & ifu_exu_range_check_jlret_e;
   // jlret is used for misalign (E stage) and va hole (M stage).
   // if address mask is on then the va hole is not checked
   assign   valid_range_check_jlret_e = ifu_exu_range_check_jlret_e & ~addr_mask_e;
   assign   exu_ifu_va_oor_m = ~alu_ecl_mem_addr_invalid_m_l;
   assign exu_tlu_va_oor_m = (~alu_ecl_mem_addr_invalid_m_l &
                              ifu_exu_range_check_other_m);
   assign exu_tlu_va_oor_jl_ret_m = (~alu_ecl_mem_addr_invalid_m_l &
                                     ifu_exu_range_check_jlret_m);
   dff_s dff_addr_mask_d2e (.din(ifu_exu_addr_mask_d), .clk(clk), .q(addr_mask_e),
                          .se(se), .si(), .so());
   dff_s dff_mem_invalid_e2m(.din(alu_ecl_mem_addr_invalid_e_l), .clk(clk),
                           .q(alu_ecl_mem_addr_invalid_m_l), .se(se),
                           .si(), .so());
   dff_s dff_misalign_addr_e2m(.din(misalign_addr_e), .clk(clk),
                           .q(exu_tlu_misalign_addr_jmpl_rtn_m), .se(se),
                           .si(), .so());
   dff_s dff_range_check_jlret_d2e(.din(ifu_exu_range_check_jlret_d), .clk(clk),
                            .q(ifu_exu_range_check_jlret_e), .se(se),
                            .si(), .so());
   dff_s dff_range_check_jlret_e2m(.din(valid_range_check_jlret_e), .clk(clk),
                            .q(ifu_exu_range_check_jlret_m), .se(se),
                            .si(), .so());
   dff_s dff_range_check_other_d2e(.din(ifu_exu_range_check_other_d), .clk(clk),
                            .q(ifu_exu_range_check_other_e), .se(se),
                            .si(), .so());
   dff_s dff_range_check_other_e2m(.din(ifu_exu_range_check_other_e), .clk(clk),
                            .q(ifu_exu_range_check_other_m), .se(se),
                            .si(), .so());

   // 3lsbs can be zeroes for ialign
   assign ecl_byp_3lsb_m[2:0] = (ialign_m)? 3'b0: byp_ecl_rd_data_3lsb_m[2:0];

   /////////////////////////////
   // Generate Shift control
   /////////////////////////////
   assign shiftop_d[2:0] = ifu_exu_shiftop_d[2:0] & {3{ifu_exu_enshift_d}};
   // shiftop:
   //   2 = 64bit shift
   //   1 = Rshift (1), LShift (0)
   //   0 = arithmetic shift
   assign ecl_shft_lshift_e_l = shiftop_e[1];
   assign shft_sext_e = shiftop_e[0];
   assign ecl_shft_op32_e = ~shiftop_e[2];
   assign ecl_shft_enshift_e_l = ~enshift_e;
   // decide what sign extension for right shifts should be (in parallel w/
   // masking operation)
   assign ecl_shft_extend32bit_e_l = ~(ecl_shft_op32_e & byp_ecl_rs1_31_e
                                   & shft_sext_e);
   assign extend64bit = shft_sext_e & byp_ecl_rs1_63_e &
          ~ecl_shft_op32_e;
   assign ecl_shft_extendbit_e = (extend64bit | ~ecl_shft_extend32bit_e_l);
   
   // Get rid of top bit for 32 bit instructions
   //assign mod_shiftby_e[5]  = shiftop_e[2] & byp_ecl_rs2_3_0_e[5];
   // decode shiftby input into mux control signals
   //assign ecl_shft_shift16_e[0] = (~mod_shiftby_e[5] & ~mod_shiftby_e[4]);
   //assign ecl_shft_shift16_e[1] = (~mod_shiftby_e[5] & mod_shiftby_e[4]);
   //assign ecl_shft_shift16_e[2] = (mod_shiftby_e[5] & ~mod_shiftby_e[4]);
   //assign ecl_shft_shift16_e[3] = (mod_shiftby_e[5] & mod_shiftby_e[4]);

   assign ecl_shft_shift4_e[0] = (~byp_ecl_rs2_3_0_e[3] & ~byp_ecl_rs2_3_0_e[2]);
   assign ecl_shft_shift4_e[1] = (~byp_ecl_rs2_3_0_e[3] & byp_ecl_rs2_3_0_e[2]);
   assign ecl_shft_shift4_e[2] = (byp_ecl_rs2_3_0_e[3] & ~byp_ecl_rs2_3_0_e[2]);
   assign ecl_shft_shift4_e[3] = (byp_ecl_rs2_3_0_e[3] & byp_ecl_rs2_3_0_e[2]);

   assign ecl_shft_shift1_e[0] = (~byp_ecl_rs2_3_0_e[1] & ~byp_ecl_rs2_3_0_e[0]);
   assign ecl_shft_shift1_e[1] = (~byp_ecl_rs2_3_0_e[1] & byp_ecl_rs2_3_0_e[0]);
   assign ecl_shft_shift1_e[2] = (byp_ecl_rs2_3_0_e[1] & ~byp_ecl_rs2_3_0_e[0]);
   assign ecl_shft_shift1_e[3] = (byp_ecl_rs2_3_0_e[1] & byp_ecl_rs2_3_0_e[0]);


   // pipeline flops for bypass data
   dff_s #(5) dff_rd_d2e(.din(ifu_exu_rd_d[4:0]), .clk(clk), .q(rd_e[4:0]), .se(se),
                     .si(), .so());
   // account for switch of ins outs on save/restore
   assign real_rd_e[4] = rd_e[4] ^ (rd_e[3] & (save_e | restore_e));
   assign real_rd_e[3:0] = rd_e[3:0];
   dff_s #(5) dff_rd_e2m(.din(real_rd_e[4:0]), .clk(clk), .q(rd_m[4:0]), .se(se),
                     .si(), .so());
   dff_s #(5) dff_rd_m2w(.din(ecl_irf_rd_m[4:0]), .clk(clk), .q(ecl_irf_rd_w[4:0]), .se(se),
                     .si(), .so());
   dff_s #(2) dff_thr_s2d(.din(ifu_exu_tid_s2[1:0]), .clk(clk), .q(tid_d[1:0]), .se(se),
                      .si(), .so());
   dff_s #(2) dff_tid_d2e(.din(tid_d[1:0]), .clk(clk), .q(tid_e[1:0]), .se(se),
                      .si(), .so());
   dff_s #(2) dff_thr_e2m(.din(tid_e[1:0]), .clk(clk), .q(tid_m[1:0]), .se(se),
                      .si(), .so());
   // Need the original thr and the one with ld thr muxed in
   dff_s #(2) dff_tid_m2w(.din(tid_m[1:0]), .clk(clk), .q(tid_w[1:0]), .se(se),
                      .si(), .so());
   dff_s #(2) dff_tid_w2w1(.din(tid_w[1:0]), .clk(clk), .q(tid_w1[1:0]), .se(se),
                      .si(), .so());
   dff_s #(2) dff_irf_thr_m2w(.din(ecl_irf_tid_m[1:0]), .clk(clk), .q(ecl_irf_tid_w[1:0]), .se(se),
                      .si(), .so());

   // Thread decode
   // decode tid_d
   assign        thr_d[0] = ~tid_d[1] & ~tid_d[0];
   assign        thr_d[1] = ~tid_d[1] & tid_d[0];
   assign        thr_d[2] = tid_d[1] & ~tid_d[0];
   assign        thr_d[3] = tid_d[1] & tid_d[0];

   // decode thr_e
   assign        ecl_div_thr_e[0] = ~tid_e[1] & ~tid_e[0];
   assign        ecl_div_thr_e[1] = ~tid_e[1] & tid_e[0];
   assign        ecl_div_thr_e[2] = tid_e[1] & ~tid_e[0];
   assign        ecl_div_thr_e[3] = tid_e[1] & tid_e[0];
   
   // decode thr_m
   assign        thr_m[0] = ~tid_m[1] & ~tid_m[0];
   assign        thr_m[1] = ~tid_m[1] & tid_m[0];
   assign        thr_m[2] = tid_m[1] & ~tid_m[0];
   assign        thr_m[3] = tid_m[1] & tid_m[0];
   assign        ecl_rml_thr_m[3:0] = thr_m[3:0];
   // decode tid_w
   assign        ecl_rml_thr_w[0] = ~tid_w[1] & ~tid_w[0];
   assign        ecl_rml_thr_w[1] = ~tid_w[1] & tid_w[0];
   assign        ecl_rml_thr_w[2] = tid_w[1] & ~tid_w[0];
   assign        ecl_rml_thr_w[3] = tid_w[1] & tid_w[0];

   //////////////////////////////////////
   // Kill logic
   //////////////////////////////////////
   // a parity error on a store should kill the next instruction on that thread
   // perr_store_w sets the bit.  perr_kill_m says that the instruction in M should
   // be killed.  However, it does not check inst_vld or flush so it might be killing
   // an invalid instruction.  Therefore perr_store does not get cleared until W.  This
   // might cause an extra perr_kill_m, but that is OK because subsequent instructions will
   // be killed until the trap is taken.
   wire [3:0]    perr_store_w;
   wire [3:0]    perr_clear_w;
   wire          perr_kill_m;
   assign        perr_store_w[3] = tid_w[1] & tid_w[0] & lsu_exu_st_dtlb_perr_g;
   assign        perr_store_w[2] = tid_w[1] & ~tid_w[0] & lsu_exu_st_dtlb_perr_g;
   assign        perr_store_w[1] = ~tid_w[1] & tid_w[0] & lsu_exu_st_dtlb_perr_g;
   assign        perr_store_w[0] = ~tid_w[1] & ~tid_w[0] & lsu_exu_st_dtlb_perr_g;
   assign        perr_store_next[3] = perr_store_w[3] | perr_store[3] & ~perr_clear_w[3];
   assign        perr_store_next[2] = perr_store_w[2] | perr_store[2] & ~perr_clear_w[2];
   assign        perr_store_next[1] = perr_store_w[1] | perr_store[1] & ~perr_clear_w[1];
   assign        perr_store_next[0] = perr_store_w[0] | perr_store[0] & ~perr_clear_w[0];
   assign        perr_kill[3] = tid_m[1] & tid_m[0] & perr_store[3];
   assign        perr_kill[2] = tid_m[1] & ~tid_m[0] & perr_store[2];
   assign        perr_kill[1] = ~tid_m[1] & tid_m[0] & perr_store[1];
   assign        perr_kill[0] = ~tid_m[1] & ~tid_m[0] & perr_store[0];
   assign        perr_kill_m = |perr_kill[3:0] | lsu_exu_st_dtlb_perr_g & thr_match_mw;
   assign        perr_clear_w[3] = tid_w[1] & tid_w[0] & perr_store[3] & ifu_exu_inst_vld_w & ~ifu_tlu_flush_w;
   assign        perr_clear_w[2] = tid_w[1] & ~tid_w[0] & perr_store[2] & ifu_exu_inst_vld_w & ~ifu_tlu_flush_w;
   assign        perr_clear_w[1] = ~tid_w[1] & tid_w[0] & perr_store[1] & ifu_exu_inst_vld_w & ~ifu_tlu_flush_w;
   assign        perr_clear_w[0] = ~tid_w[1] & ~tid_w[0] & perr_store[0] & ifu_exu_inst_vld_w & ~ifu_tlu_flush_w;
   
   dffr_s #(4) perr_dff(.din(perr_store_next[3:0]), .clk(clk), .q(perr_store[3:0]), .si(), .so(), .se(se), .rst(reset));
   
   // calculate an early flush for killing writes in W
   // the pic trap occurs if there are too many instructions on a given thread.
   dff_s inst_vld_ww1(.din(ifu_exu_inst_vld_w), .clk(clk), .q(inst_vld_w1), .se(se), .si(), .so());
   assign pic_trap_m = ((tlu_exu_pic_onebelow_m & (thr_match_mw & ifu_exu_inst_vld_w | 
                                                   thr_match_mw1 & inst_vld_w1)) | 
                        (tlu_exu_pic_twobelow_m & thr_match_mw & ifu_exu_inst_vld_w &
                         thr_match_mw1 & inst_vld_w1));
   assign        part_early_flush_m = (exu_tlu_ttype_vld_m | ifu_exu_ttype_vld_m | exu_tlu_va_oor_jl_ret_m | 
                                       perr_kill_m | pic_trap_m);
   dff_s priv_trap_dff(.din(tlu_exu_priv_trap_m), .clk(clk), .q(tlu_priv_trap_w), .se(se), .si(), .so());
   dff_s early_flush_dff(.din(part_early_flush_m), .clk(clk), .q(part_early_flush_w), .se(se), .si(), .so());
   assign        early_flush_w = part_early_flush_w | tlu_priv_trap_w;
   assign        ecl_rml_early_flush_w = early_flush_w;
   
   // buffer this off so it only sees one load from the ifu
   assign        ecl_rml_inst_vld_w = ifu_exu_inst_vld_w & ~ifu_tlu_flush_w;

   dff_s flush_m2w(.din(ifu_tlu_flush_m), .clk(clk), .q(ifu_tlu_flush_w), .se(se), .si(), .so());
   assign        flush_w = ifu_tlu_flush_w | lsu_exu_flush_pipe_w;
   dff_s flush_w_dff(.din(flush_w), .clk(clk), .q(flush_w1), .se(se), .si(), .so());
   // allow misalign address on returns to kill the cwp switch
   // ttype[7] is a fill_trap so the return misalign should be ignored
   // UE trap should kill window ops.  This check is needed here because the
   // window traps will override the flush_W signals.
   assign        kill_rml_m = (ue_trap_m | ifu_exu_ttype_vld_m | perr_kill_m | pic_trap_m |
                               (exu_tlu_misalign_addr_jmpl_rtn_m & ~exu_tlu_ttype_m[7]));
   dff_s kill_rml_mw(.din(kill_rml_m), .clk(clk) , .q(kill_rml_w), .se(se), .si(), .so());
   // include tlu_priv_trap to cancel window traps
   assign        ecl_rml_kill_w = tlu_priv_trap_w | kill_rml_w;

   // pass kill_e through to the rml
   assign        ecl_rml_kill_e = ifu_exu_kill_e;
   
   assign        ecl_exu_kill_m = thr_match_mw1 & flush_w1;
   assign thr_match_mw = ~((tid_w[1] ^ tid_m[1]) |
                           (tid_w[0] ^ tid_m[0]));
   assign thr_match_ew = ~((tid_e[1] ^ tid_w[1]) |
                           (tid_e[0] ^ tid_w[0]));
   dff_s thr_match_ew_dff(.din(thr_match_ew), .clk(clk), .q(thr_match_mw1), .se(se), .si(), .so());

   // ldxa needs to check inst_vld and prior flushes
   assign ecl_byp_ldxa_g = ldxa_g & ifu_exu_inst_vld_w;   
   
   // controls for outputs to lsu
   assign std_d = ifu_exu_rs3e_vld_d & ifu_exu_rs3o_vld_d;
   dff_s std_d2e (.din(std_d), .q(std_e), .clk(clk), .se(se), .si(), .so());
   assign ecl_byp_std_e_l = ~std_e;

  
   //////////////////////////////////////
   // Trap output logic
   //-----------------------
   // In pipe traps (with priority order):
   // 029h: uncorrected ecc trap
   // 0C0h-0FFh: Fill trap
   // 024h: clean window trap
   // 034h: mem_address_not_aligned
   // 023h: Tag Overflow
   // 028h: Div by zero
   // 100h-17Fh: Trap instruction
   //////////////////////////////////////
   // ecc traps must be enabled
   assign fill_trap_e = rml_ecl_fill_e;
   
   assign early_ttype_vld_e = (rml_ecl_clean_window_e | rml_ecl_fill_e | 
                                 tag_overflow_trap_e | ifu_exu_tcc_e |
                                 misalign_addr_e);
   // This ttype includes clean window, fill, tag overflow, tcc, misalign address, and div zero.
   // Note that this will be div_zero on any divide instruction.  The valid will only be asserted if
   // div_zero is detected though.
   assign early1_ttype_e[8] = 1'b0;
   assign early1_ttype_e[7] = fill_trap_e;
   assign early1_ttype_e[6] = fill_trap_e;
   assign early1_ttype_e[5] = (rml_ecl_fill_e & rml_ecl_other_e) | 
                                (~rml_ecl_fill_e & (rml_ecl_clean_window_e | tag_overflow_trap_e | div_e));
   assign early1_ttype_e[4] = fill_trap_e & rml_ecl_wtype_e[2];
   assign early1_ttype_e[3] = (rml_ecl_fill_e & rml_ecl_wtype_e[1]) |
                           (~rml_ecl_fill_e & ~rml_ecl_clean_window_e & ~tag_overflow_trap_e & div_e);
   assign early1_ttype_e[2] = (fill_trap_e & rml_ecl_wtype_e[0]) |
                                 (~rml_ecl_fill_e & rml_ecl_clean_window_e);
   assign early1_ttype_e[1] = ~rml_ecl_fill_e & ~rml_ecl_clean_window_e & tag_overflow_trap_e;
   assign early1_ttype_e[0] = (~rml_ecl_fill_e & ~rml_ecl_clean_window_e & tag_overflow_trap_e);
   
   // mux together the ttypes
   // tcc only can be combined with an ue which is caught later so it isn't qualified by other traps
   assign pick_normal_ttype = ~pick_not_aligned & ~ifu_exu_tcc_e;
   assign pick_tcc = ifu_exu_tcc_e;
   assign pick_not_aligned = ~(rml_ecl_fill_e | rml_ecl_clean_window_e) & misalign_addr_e & ~ifu_exu_tcc_e;
   
   // the ue ttype is muxed in after the flop because it is so late
   mux3ds #(9) ttype_mux(.dout(early2_ttype_e[8:0]),
                         .in0(early1_ttype_e[8:0]),
                         .in1({1'b1, alu_ecl_adder_out_7_0_e[7:0]}),
                         .in2(9'h034),
                         .sel0(pick_normal_ttype),
                         .sel1(pick_tcc),
                         .sel2(pick_not_aligned));
   assign exu_tlu_ttype_m[8:0] = (ue_trap_m)? 9'h029: early_ttype_m[8:0];
   assign exu_tlu_ttype_vld_m = early_ttype_vld_m | ue_trap_m | div_zero_m;
   assign exu_tlu_ue_trap_m = ue_trap_m;
   
   dff_s ttype_vld_e2m(.din(early_ttype_vld_e), .clk(clk), .q(early_ttype_vld_m),
                     .se(se), .si(), .so());
   dff_s #(9) ttype_e2m(.din(early2_ttype_e[8:0]), .clk(clk), .q(early_ttype_m[8:0]),
                    .se(se), .si(), .so());
   // lsu needs to know about spill and ue traps for squashing sfsr writes
   dff_s fill_e2m(.din(fill_trap_e), .clk(clk), .q(fill_trap_m), .se(se), .si(), .so());
   assign exu_lsu_priority_trap_m = fill_trap_m | ue_trap_m;

   // Condition code Register
   sparc_exu_eclccr ccr(.wb_ccr_thr_g(ecl_irf_tid_g[1:0]),
                        .thrdec_d  (thr_d[3:0]),
                        .thr_w  (ecl_rml_thr_w[3:0]),
                       /*AUTOINST*/
                        // Outputs
                        .exu_ifu_cc_d   (exu_ifu_cc_d[7:0]),
                        .exu_tlu_ccr0_w (exu_tlu_ccr0_w[7:0]),
                        .exu_tlu_ccr1_w (exu_tlu_ccr1_w[7:0]),
                        .exu_tlu_ccr2_w (exu_tlu_ccr2_w[7:0]),
                        .exu_tlu_ccr3_w (exu_tlu_ccr3_w[7:0]),
                        // Inputs
                        .clk            (clk),
                        .se             (se),
                        .alu_xcc_e      (alu_xcc_e[3:0]),
                        .alu_icc_e      (alu_icc_e[3:0]),
                        .tid_d          (tid_d[1:0]),
                        .thr_match_dm   (thr_match_dm),
                        .thr_match_de   (thr_match_de),
                        .tid_w          (tid_w[1:0]),
                        .ifu_exu_kill_e (ifu_exu_kill_e),
                        .ifu_exu_setcc_d(ifu_exu_setcc_d),
                        .byp_ecl_wrccr_data_w(byp_ecl_wrccr_data_w[7:0]),
                        .wb_ccr_wrccr_w (wb_ccr_wrccr_w),
                        .wb_ccr_setcc_g (wb_ccr_setcc_g),
                        .divcntl_ccr_cc_w2(divcntl_ccr_cc_w2[7:0]),
                        .tlu_exu_cwpccr_update_m(tlu_exu_cwpccr_update_m),
                        .tlu_exu_ccr_m  (tlu_exu_ccr_m[7:0]),
                        .ifu_exu_inst_vld_w(ifu_exu_inst_vld_w),
                        .ifu_tlu_flush_w(ifu_tlu_flush_w),
                        .early_flush_w  (early_flush_w));
   
   // Writeback control logic
   sparc_exu_ecl_wb writeback(
                              .read_yreg_e(read_yreg_e),
                              /*AUTOINST*/
                              // Outputs
                              .wb_ccr_wrccr_w(wb_ccr_wrccr_w),
                              .ecl_rml_cwp_wen_e(ecl_rml_cwp_wen_e),
                              .ecl_rml_cansave_wen_w(ecl_rml_cansave_wen_w),
                              .ecl_rml_canrestore_wen_w(ecl_rml_canrestore_wen_w),
                              .ecl_rml_otherwin_wen_w(ecl_rml_otherwin_wen_w),
                              .ecl_rml_wstate_wen_w(ecl_rml_wstate_wen_w),
                              .ecl_rml_cleanwin_wen_w(ecl_rml_cleanwin_wen_w),
                              .ecl_byp_sel_load_m(ecl_byp_sel_load_m),
                              .ecl_byp_sel_restore_m(ecl_byp_sel_restore_m),
                              .ecl_byp_sel_pipe_m(ecl_byp_sel_pipe_m),
                              .ecl_byp_restore_m(ecl_byp_restore_m),
                              .ecl_irf_tid_m(ecl_irf_tid_m[1:0]),
                              .ecl_irf_rd_m(ecl_irf_rd_m[4:0]),
                              .ecl_irf_rd_g(ecl_irf_rd_g[4:0]),
                              .ecl_irf_wen_w2(ecl_irf_wen_w2),
                              .ecl_irf_tid_g(ecl_irf_tid_g[1:0]),
                              .wb_e     (wb_e),
                              .bypass_m (bypass_m),
                              .ecl_irf_wen_w(ecl_irf_wen_w),
                              .ecl_byp_sel_load_g(ecl_byp_sel_load_g),
                              .ecl_byp_sel_muldiv_g(ecl_byp_sel_muldiv_g),
                              .ecl_byp_sel_restore_g(ecl_byp_sel_restore_g),
                              .wb_divcntl_ack_g(wb_divcntl_ack_g),
                              .wb_ccr_setcc_g(wb_ccr_setcc_g),
                              .ecl_byp_eclpr_e(ecl_byp_eclpr_e[7:0]),
                              .exu_ifu_longop_done_g(exu_ifu_longop_done_g[3:0]),
                              .ecl_div_yreg_wen_w(ecl_div_yreg_wen_w[3:0]),
                              .ecl_div_yreg_wen_g(ecl_div_yreg_wen_g[3:0]),
                              .ecl_div_yreg_shift_g(ecl_div_yreg_shift_g[3:0]),
                              .ecl_div_yreg_wen_l(ecl_div_yreg_wen_l[3:0]),
                              .wb_eccctl_spec_wen_next(wb_eccctl_spec_wen_next),
                              .bypass_w (bypass_w),
                              .wb_byplog_rd_w2(wb_byplog_rd_w2[4:0]),
                              .wb_byplog_tid_w2(wb_byplog_tid_w2[1:0]),
                              .wb_byplog_wen_w2(wb_byplog_wen_w2),
                              .wb_byplog_rd_g2(wb_byplog_rd_g2[4:0]),
                              .wb_byplog_wen_g2(wb_byplog_wen_g2),
                              .exu_ffu_wsr_inst_e(exu_ffu_wsr_inst_e),
                              // Inputs
                              .clk      (clk),
                              .se       (se),
                              .reset    (reset),
                              .sehold   (sehold),
                              .ld_rd_g  (ld_rd_g[4:0]),
                              .ld_tid_g (ld_tid_g[1:0]),
                              .lsu_exu_dfill_vld_g(lsu_exu_dfill_vld_g),
                              .lsu_exu_ldst_miss_g2(lsu_exu_ldst_miss_g2),
                              .rd_m     (rd_m[4:0]),
                              .tid_m    (tid_m[1:0]),
                              .thr_m    (thr_m[3:0]),
                              .tid_w1   (tid_w1[1:0]),
                              .ifu_exu_wen_d(ifu_exu_wen_d),
                              .ifu_exu_kill_e(ifu_exu_kill_e),
                              .ecl_exu_kill_m(ecl_exu_kill_m),
                              .rml_ecl_kill_m(rml_ecl_kill_m),
                              .ifu_tlu_flush_w(ifu_tlu_flush_w),
                              .flush_w1 (flush_w1),
                              .divcntl_wb_req_g(divcntl_wb_req_g),
                              .mdqctl_wb_divrd_g(mdqctl_wb_divrd_g[4:0]),
                              .mdqctl_wb_divthr_g(mdqctl_wb_divthr_g[1:0]),
                              .mdqctl_wb_mulrd_g(mdqctl_wb_mulrd_g[4:0]),
                              .mdqctl_wb_multhr_g(mdqctl_wb_multhr_g[1:0]),
                              .mdqctl_wb_divsetcc_g(mdqctl_wb_divsetcc_g),
                              .mdqctl_wb_mulsetcc_g(mdqctl_wb_mulsetcc_g),
                              .ecl_div_sel_div(ecl_div_sel_div),
                              .ifu_tlu_wsr_inst_d(ifu_tlu_wsr_inst_d),
                              .ifu_tlu_sraddr_d(ifu_tlu_sraddr_d[6:0]),
                              .rml_ecl_cwp_d(rml_ecl_cwp_d[2:0]),
                              .rml_ecl_cansave_d(rml_ecl_cansave_d[2:0]),
                              .rml_ecl_canrestore_d(rml_ecl_canrestore_d[2:0]),
                              .rml_ecl_otherwin_d(rml_ecl_otherwin_d[2:0]),
                              .rml_ecl_wstate_d(rml_ecl_wstate_d[5:0]),
                              .rml_ecl_cleanwin_d(rml_ecl_cleanwin_d[2:0]),
                              .exu_ifu_cc_d(exu_ifu_cc_d[7:0]),
                              .rml_ecl_swap_done(rml_ecl_swap_done[3:0]),
                              .rml_ecl_rmlop_done_e(rml_ecl_rmlop_done_e),
                              .mdqctl_wb_yreg_wen_g(mdqctl_wb_yreg_wen_g),
                              .mdqctl_wb_yreg_shift_g(mdqctl_wb_yreg_shift_g),
                              .ecl_byp_sel_ecc_m(ecl_byp_sel_ecc_m),
                              .eccctl_wb_rd_m(eccctl_wb_rd_m[4:0]),
                              .ifu_exu_inst_vld_e(ifu_exu_inst_vld_e),
                              .ifu_exu_inst_vld_w(ifu_exu_inst_vld_w),
                              .ifu_exu_return_d(ifu_exu_return_d),
                              .restore_e(restore_e),
                              .rml_ecl_fill_e(rml_ecl_fill_e),
                              .early_flush_w(early_flush_w),
                              .ecl_byp_ldxa_g(ecl_byp_ldxa_g));

   ////////////////////////
   // ECC control logic
   ////////////////////////
   sparc_exu_ecl_eccctl eccctl(
                               .ue_trap_m(ue_trap_m),
                               /*AUTOINST*/
                               // Outputs
                               .ecl_ecc_sel_rs1_m_l(ecl_ecc_sel_rs1_m_l),
                               .ecl_ecc_sel_rs2_m_l(ecl_ecc_sel_rs2_m_l),
                               .ecl_ecc_sel_rs3_m_l(ecl_ecc_sel_rs3_m_l),
                               .ecl_ecc_log_rs1_m(ecl_ecc_log_rs1_m),
                               .ecl_ecc_log_rs2_m(ecl_ecc_log_rs2_m),
                               .ecl_ecc_log_rs3_m(ecl_ecc_log_rs3_m),
                               .ecl_byp_sel_ecc_m(ecl_byp_sel_ecc_m),
                               .ecl_ecc_rs1_use_rf_e(ecl_ecc_rs1_use_rf_e),
                               .ecl_ecc_rs2_use_rf_e(ecl_ecc_rs2_use_rf_e),
                               .ecl_ecc_rs3_use_rf_e(ecl_ecc_rs3_use_rf_e),
                               .eccctl_wb_rd_m(eccctl_wb_rd_m[4:0]),
                               .exu_ifu_ecc_ce_m(exu_ifu_ecc_ce_m),
                               .exu_ifu_ecc_ue_m(exu_ifu_ecc_ue_m),
                               .exu_ifu_err_reg_m(exu_ifu_err_reg_m[7:0]),
                               .ecl_byp_ecc_mask_m_l(ecl_byp_ecc_mask_m_l[7:0]),
                               .exu_ifu_inj_ack(exu_ifu_inj_ack),
                               .exu_ifu_err_synd_7_m(exu_ifu_err_synd_7_m),
                               // Inputs
                               .clk     (clk),
                               .se      (se),
                               .rst_tri_en(rst_tri_en),
                               .ecc_ecl_rs1_ce(ecc_ecl_rs1_ce),
                               .ecc_ecl_rs1_ue(ecc_ecl_rs1_ue),
                               .ecc_ecl_rs2_ce(ecc_ecl_rs2_ce),
                               .ecc_ecl_rs2_ue(ecc_ecl_rs2_ue),
                               .ecc_ecl_rs3_ce(ecc_ecl_rs3_ce),
                               .ecc_ecl_rs3_ue(ecc_ecl_rs3_ue),
                               .ecl_byp_rcc_mux2_sel_rf(ecl_byp_rcc_mux2_sel_rf),
                               .ecl_byp_rs2_mux2_sel_rf(ecl_byp_rs2_mux2_sel_rf),
                               .ecl_byp_rs3_mux2_sel_rf(ecl_byp_rs3_mux2_sel_rf),
                               .rs1_vld_e(rs1_vld_e),
                               .rs2_vld_e(rs2_vld_e),
                               .rs3_vld_e(rs3_vld_e),
                               .ifu_exu_rs1_m(ifu_exu_rs1_m[4:0]),
                               .ifu_exu_rs2_m(ifu_exu_rs2_m[4:0]),
                               .ifu_exu_rs3_m(ifu_exu_rs3_m[4:0]),
                               .rml_ecl_cwp_d(rml_ecl_cwp_d[2:0]),
                               .ifu_exu_ecc_mask(ifu_exu_ecc_mask[7:0]),
                               .ifu_exu_inj_irferr(ifu_exu_inj_irferr),
                               .ifu_exu_disable_ce_e(ifu_exu_disable_ce_e),
                               .wb_eccctl_spec_wen_next(wb_eccctl_spec_wen_next),
                               .ifu_exu_nceen_e(ifu_exu_nceen_e),
                               .ifu_exu_inst_vld_e(ifu_exu_inst_vld_e),
                               .rml_ecl_gl_e(rml_ecl_gl_e[1:0]),
                               .cancel_rs3_ecc_e(cancel_rs3_ecc_e));
   // Bypass logic
   // Precalculate some of the matching logic to help timing
   assign thr_match_sd =  ~((ifu_exu_tid_s2[1] ^ tid_d[1]) |
                           (ifu_exu_tid_s2[0] ^ tid_d[0]));
   dff_s thr_match_sd_dff(.din(thr_match_sd), .clk(clk), .q(thr_match_de),
                        .se(se), .si(), .so());
   assign thr_match_se =  ~((ifu_exu_tid_s2[1] ^ tid_e[1]) |
                           (ifu_exu_tid_s2[0] ^ tid_e[0]));
   dff_s thr_match_se_dff(.din(thr_match_se), .clk(clk), .q(thr_match_dm),
                        .se(se), .si(), .so());
   assign ld_thr_match_sm = ~((ifu_exu_tid_s2[1] ^ lsu_exu_thr_m[1]) |
                           (ifu_exu_tid_s2[0] ^ lsu_exu_thr_m[0]));
   dff_s ld_thr_match_sm_dff(.din(ld_thr_match_sm), .clk(clk), .q(ld_thr_match_dg), .se(se),
                           .si(), .so());
   assign ld_thr_match_sg = ~((ifu_exu_tid_s2[1] ^ ld_tid_g[1]) |
                           (ifu_exu_tid_s2[0] ^ ld_tid_g[0]));
   dff_s ld_thr_match_sg_dff(.din(ld_thr_match_sg), .clk(clk), .q(ld_thr_match_dg2), .se(se),
                           .si(), .so());
   sparc_exu_eclbyplog_rs1 byplog_rs1(.rs_sel_mux1_m(ecl_byp_rs1_mux1_sel_m),
                                  .rs_sel_mux1_w(ecl_byp_rs1_mux1_sel_w),
                                  .rs_sel_mux1_w2(ecl_byp_rs1_mux1_sel_w2),
                                  .rs_sel_mux1_other(ecl_byp_rs1_mux1_sel_other),
                                  .rs_sel_mux2_e(ecl_byp_rs1_mux2_sel_e),
                                  .rs_sel_mux2_rf(ecl_byp_rs1_mux2_sel_rf),
                                  .rs_sel_mux2_ld(ecl_byp_rs1_mux2_sel_ld),
                                  .rs_sel_mux2_usemux1(ecl_byp_rs1_mux2_sel_usemux1),
                                  .rs_sel_longmux_g2(ecl_byp_rs1_longmux_sel_g2),
                                  .rs_sel_longmux_w2(ecl_byp_rs1_longmux_sel_w2),
                                  .rs_sel_longmux_ldxa(ecl_byp_rs1_longmux_sel_ldxa),
                                  .rs   (ifu_exu_rs1_d[4:0]),
                                  .use_other(ifu_exu_dbrinst_d),
                                  /*AUTOINST*/
                                      // Outputs
                                      .ecl_byp_rcc_mux1_sel_m(ecl_byp_rcc_mux1_sel_m),
                                      .ecl_byp_rcc_mux1_sel_w(ecl_byp_rcc_mux1_sel_w),
                                      .ecl_byp_rcc_mux1_sel_w2(ecl_byp_rcc_mux1_sel_w2),
                                      .ecl_byp_rcc_mux1_sel_other(ecl_byp_rcc_mux1_sel_other),
                                      .ecl_byp_rcc_mux2_sel_usemux1(ecl_byp_rcc_mux2_sel_usemux1),
                                      .ecl_byp_rcc_mux2_sel_rf(ecl_byp_rcc_mux2_sel_rf),
                                      .ecl_byp_rcc_mux2_sel_e(ecl_byp_rcc_mux2_sel_e),
                                      .ecl_byp_rcc_mux2_sel_ld(ecl_byp_rcc_mux2_sel_ld),
                                      // Inputs
                                      .sehold(sehold),
                                      .rd_e(rd_e[4:0]),
                                      .rd_m(rd_m[4:0]),
                                      .ecl_irf_rd_w(ecl_irf_rd_w[4:0]),
                                      .ld_rd_g(ld_rd_g[4:0]),
                                      .wb_byplog_rd_w2(wb_byplog_rd_w2[4:0]),
                                      .wb_byplog_rd_g2(wb_byplog_rd_g2[4:0]),
                                      .tid_d(tid_d[1:0]),
                                      .thr_match_de(thr_match_de),
                                      .thr_match_dm(thr_match_dm),
                                      .ecl_irf_tid_w(ecl_irf_tid_w[1:0]),
                                      .ld_thr_match_dg(ld_thr_match_dg),
                                      .wb_byplog_tid_w2(wb_byplog_tid_w2[1:0]),
                                      .ld_thr_match_dg2(ld_thr_match_dg2),
                                      .ifu_exu_kill_e(ifu_exu_kill_e),
                                      .wb_e(wb_e),
                                      .bypass_m(bypass_m),
                                      .lsu_exu_dfill_vld_g(lsu_exu_dfill_vld_g),
                                      .bypass_w(bypass_w),
                                      .wb_byplog_wen_w2(wb_byplog_wen_w2),
                                      .wb_byplog_wen_g2(wb_byplog_wen_g2),
                                      .ecl_byp_ldxa_g(ecl_byp_ldxa_g));

   sparc_exu_eclbyplog byplog_rs2(.rs_sel_mux1_m(ecl_byp_rs2_mux1_sel_m),
                                  .rs_sel_mux1_w(ecl_byp_rs2_mux1_sel_w),
                                  .rs_sel_mux1_w2(ecl_byp_rs2_mux1_sel_w2),
                                  .rs_sel_mux1_other(ecl_byp_rs2_mux1_sel_other),
                                  .rs_sel_mux2_e(ecl_byp_rs2_mux2_sel_e),
                                  .rs_sel_mux2_rf(ecl_byp_rs2_mux2_sel_rf),
                                  .rs_sel_mux2_ld(ecl_byp_rs2_mux2_sel_ld),
                                  .rs_sel_mux2_usemux1(ecl_byp_rs2_mux2_sel_usemux1),
                                  .rs_sel_longmux_g2(ecl_byp_rs2_longmux_sel_g2),
                                  .rs_sel_longmux_w2(ecl_byp_rs2_longmux_sel_w2),
                                  .rs_sel_longmux_ldxa(ecl_byp_rs2_longmux_sel_ldxa),
                                  .rs   (ifu_exu_rs2_d[4:0]),
                                  .use_other(ifu_exu_useimm_d),
                                  /*AUTOINST*/
                                  // Inputs
                                  .sehold(sehold),
                                  .rd_e (rd_e[4:0]),
                                  .rd_m (rd_m[4:0]),
                                  .ecl_irf_rd_w(ecl_irf_rd_w[4:0]),
                                  .ld_rd_g(ld_rd_g[4:0]),
                                  .wb_byplog_rd_w2(wb_byplog_rd_w2[4:0]),
                                  .wb_byplog_rd_g2(wb_byplog_rd_g2[4:0]),
                                  .tid_d(tid_d[1:0]),
                                  .thr_match_de(thr_match_de),
                                  .thr_match_dm(thr_match_dm),
                                  .ecl_irf_tid_w(ecl_irf_tid_w[1:0]),
                                  .ld_thr_match_dg(ld_thr_match_dg),
                                  .wb_byplog_tid_w2(wb_byplog_tid_w2[1:0]),
                                  .ld_thr_match_dg2(ld_thr_match_dg2),
                                  .ifu_exu_kill_e(ifu_exu_kill_e),
                                  .wb_e (wb_e),
                                  .bypass_m(bypass_m),
                                  .lsu_exu_dfill_vld_g(lsu_exu_dfill_vld_g),
                                  .bypass_w(bypass_w),
                                  .wb_byplog_wen_w2(wb_byplog_wen_w2),
                                  .wb_byplog_wen_g2(wb_byplog_wen_g2),
                                  .ecl_byp_ldxa_g(ecl_byp_ldxa_g));
   sparc_exu_eclbyplog byplog_rs3(.rs_sel_mux1_m(ecl_byp_rs3_mux1_sel_m),
                                  .rs_sel_mux1_w(ecl_byp_rs3_mux1_sel_w),
                                  .rs_sel_mux1_w2(ecl_byp_rs3_mux1_sel_w2),
                                  .rs_sel_mux1_other(ecl_byp_rs3_mux1_sel_other),
                                  .rs_sel_mux2_e(ecl_byp_rs3_mux2_sel_e),
                                  .rs_sel_mux2_rf(ecl_byp_rs3_mux2_sel_rf),
                                  .rs_sel_mux2_ld(ecl_byp_rs3_mux2_sel_ld),
                                  .rs_sel_mux2_usemux1(ecl_byp_rs3_mux2_sel_usemux1),
                                  .rs_sel_longmux_g2(ecl_byp_rs3_longmux_sel_g2),
                                  .rs_sel_longmux_w2(ecl_byp_rs3_longmux_sel_w2),
                                  .rs_sel_longmux_ldxa(ecl_byp_rs3_longmux_sel_ldxa),
                                  .rs   ({ifu_exu_rs3_d[4:0]}),
                                  .use_other(1'b0),
                                  /*AUTOINST*/
                                  // Inputs
                                  .sehold(sehold),
                                  .rd_e (rd_e[4:0]),
                                  .rd_m (rd_m[4:0]),
                                  .ecl_irf_rd_w(ecl_irf_rd_w[4:0]),
                                  .ld_rd_g(ld_rd_g[4:0]),
                                  .wb_byplog_rd_w2(wb_byplog_rd_w2[4:0]),
                                  .wb_byplog_rd_g2(wb_byplog_rd_g2[4:0]),
                                  .tid_d(tid_d[1:0]),
                                  .thr_match_de(thr_match_de),
                                  .thr_match_dm(thr_match_dm),
                                  .ecl_irf_tid_w(ecl_irf_tid_w[1:0]),
                                  .ld_thr_match_dg(ld_thr_match_dg),
                                  .wb_byplog_tid_w2(wb_byplog_tid_w2[1:0]),
                                  .ld_thr_match_dg2(ld_thr_match_dg2),
                                  .ifu_exu_kill_e(ifu_exu_kill_e),
                                  .wb_e (wb_e),
                                  .bypass_m(bypass_m),
                                  .lsu_exu_dfill_vld_g(lsu_exu_dfill_vld_g),
                                  .bypass_w(bypass_w),
                                  .wb_byplog_wen_w2(wb_byplog_wen_w2),
                                  .wb_byplog_wen_g2(wb_byplog_wen_g2),
                                  .ecl_byp_ldxa_g(ecl_byp_ldxa_g));
   sparc_exu_eclbyplog byplog_rs3h(.rs_sel_mux1_m(ecl_byp_rs3h_mux1_sel_m),
                                  .rs_sel_mux1_w(ecl_byp_rs3h_mux1_sel_w),
                                  .rs_sel_mux1_w2(ecl_byp_rs3h_mux1_sel_w2),
                                  .rs_sel_mux1_other(ecl_byp_rs3h_mux1_sel_other),
                                  .rs_sel_mux2_e(ecl_byp_rs3h_mux2_sel_e),
                                  .rs_sel_mux2_rf(ecl_byp_rs3h_mux2_sel_rf),
                                  .rs_sel_mux2_ld(ecl_byp_rs3h_mux2_sel_ld),
                                  .rs_sel_mux2_usemux1(ecl_byp_rs3h_mux2_sel_usemux1),
                                  .rs_sel_longmux_g2(ecl_byp_rs3h_longmux_sel_g2),
                                  .rs_sel_longmux_w2(ecl_byp_rs3h_longmux_sel_w2),
                                  .rs_sel_longmux_ldxa(ecl_byp_rs3h_longmux_sel_ldxa),
                                  .rs   ({ifu_exu_rs3_d[4:1],1'b1}),
                                  .use_other(1'b0),
                                  /*AUTOINST*/
                                   // Inputs
                                   .sehold(sehold),
                                   .rd_e(rd_e[4:0]),
                                   .rd_m(rd_m[4:0]),
                                   .ecl_irf_rd_w(ecl_irf_rd_w[4:0]),
                                   .ld_rd_g(ld_rd_g[4:0]),
                                   .wb_byplog_rd_w2(wb_byplog_rd_w2[4:0]),
                                   .wb_byplog_rd_g2(wb_byplog_rd_g2[4:0]),
                                   .tid_d(tid_d[1:0]),
                                   .thr_match_de(thr_match_de),
                                   .thr_match_dm(thr_match_dm),
                                   .ecl_irf_tid_w(ecl_irf_tid_w[1:0]),
                                   .ld_thr_match_dg(ld_thr_match_dg),
                                   .wb_byplog_tid_w2(wb_byplog_tid_w2[1:0]),
                                   .ld_thr_match_dg2(ld_thr_match_dg2),
                                   .ifu_exu_kill_e(ifu_exu_kill_e),
                                   .wb_e(wb_e),
                                   .bypass_m(bypass_m),
                                   .lsu_exu_dfill_vld_g(lsu_exu_dfill_vld_g),
                                   .bypass_w(bypass_w),
                                   .wb_byplog_wen_w2(wb_byplog_wen_w2),
                                   .wb_byplog_wen_g2(wb_byplog_wen_g2),
                                   .ecl_byp_ldxa_g(ecl_byp_ldxa_g));

   /////////////////////////
   // Division control logic
   /////////////////////////
   sparc_exu_ecl_divcntl divcntl(
                                 .div_ecl_divisorin_31(byp_ecl_rs2_31_e),                                 
                                 /*AUTOINST*/
                                 // Outputs
                                 .ecl_div_xinmask(ecl_div_xinmask),
                                 .ecl_div_keep_d(ecl_div_keep_d),
                                 .ecl_div_ld_inputs(ecl_div_ld_inputs),
                                 .ecl_div_sel_adder(ecl_div_sel_adder),
                                 .ecl_div_last_cycle(ecl_div_last_cycle),
                                 .ecl_div_almostlast_cycle(ecl_div_almostlast_cycle),
                                 .ecl_div_sel_div(ecl_div_sel_div),
                                 .divcntl_wb_req_g(divcntl_wb_req_g),
                                 .divcntl_ccr_cc_w2(divcntl_ccr_cc_w2[7:0]),
                                 .ecl_div_sel_64b(ecl_div_sel_64b),
                                 .ecl_div_sel_u32(ecl_div_sel_u32),
                                 .ecl_div_sel_pos32(ecl_div_sel_pos32),
                                 .ecl_div_sel_neg32(ecl_div_sel_neg32),
                                 .ecl_div_upper32_zero(ecl_div_upper32_zero),
                                 .ecl_div_upper33_one(ecl_div_upper33_one),
                                 .ecl_div_upper33_zero(ecl_div_upper33_zero),
                                 .ecl_div_dividend_sign(ecl_div_dividend_sign),
                                 .ecl_div_newq(ecl_div_newq),
                                 .ecl_div_subtract_l(ecl_div_subtract_l),
                                 .ecl_div_keepx(ecl_div_keepx),
                                 .ecl_div_cin(ecl_div_cin),
                                 // Inputs
                                 .clk   (clk),
                                 .se    (se),
                                 .reset (reset),
                                 .mdqctl_divcntl_input_vld(mdqctl_divcntl_input_vld),
                                 .wb_divcntl_ack_g(wb_divcntl_ack_g),
                                 .mdqctl_divcntl_reset_div(mdqctl_divcntl_reset_div),
                                 .div_ecl_gencc_in_msb_l(div_ecl_gencc_in_msb_l),
                                 .div_ecl_gencc_in_31(div_ecl_gencc_in_31),
                                 .div_ecl_upper32_equal(div_ecl_upper32_equal),
                                 .div_ecl_low32_nonzero(div_ecl_low32_nonzero),
                                 .ecl_div_signed_div(ecl_div_signed_div),
                                 .div_ecl_dividend_msb(div_ecl_dividend_msb),
                                 .div_ecl_xin_msb_l(div_ecl_xin_msb_l),
                                 .div_ecl_x_msb(div_ecl_x_msb),
                                 .div_ecl_d_msb(div_ecl_d_msb),
                                 .div_ecl_cout64(div_ecl_cout64),
                                 .ecl_div_div64(ecl_div_div64),
                                 .mdqctl_divcntl_muldone(mdqctl_divcntl_muldone),
                                 .ecl_div_muls(ecl_div_muls),
                                 .div_ecl_adder_out_31(div_ecl_adder_out_31),
                                 .muls_rs1_31_m_l(muls_rs1_31_m_l),
                                 .div_ecl_cout32(div_ecl_cout32),
                                 .rs2_data_31_m(rs2_data_31_m),
                                 .div_ecl_detect_zero_high(div_ecl_detect_zero_high),
                                 .div_ecl_detect_zero_low(div_ecl_detect_zero_low),
                                 .div_ecl_d_62(div_ecl_d_62));

   assign div_e = mdqctl_divcntl_input_vld;
   sparc_exu_ecl_mdqctl mdqctl(.div_zero_m(div_zero_m),
                               .byp_alu_rs1_data_31_e(byp_ecl_rs1_31_e),
                               .byp_alu_rs2_data_31_e(byp_ecl_rs2_31_e),
                               /*AUTOINST*/
                               // Outputs
                               .mdqctl_divcntl_input_vld(mdqctl_divcntl_input_vld),
                               .mdqctl_divcntl_reset_div(mdqctl_divcntl_reset_div),
                               .mdqctl_divcntl_muldone(mdqctl_divcntl_muldone),
                               .ecl_div_div64(ecl_div_div64),
                               .ecl_div_signed_div(ecl_div_signed_div),
                               .ecl_div_muls(ecl_div_muls),
                               .mdqctl_wb_divthr_g(mdqctl_wb_divthr_g[1:0]),
                               .mdqctl_wb_divrd_g(mdqctl_wb_divrd_g[4:0]),
                               .mdqctl_wb_multhr_g(mdqctl_wb_multhr_g[1:0]),
                               .mdqctl_wb_mulrd_g(mdqctl_wb_mulrd_g[4:0]),
                               .mdqctl_wb_divsetcc_g(mdqctl_wb_divsetcc_g),
                               .mdqctl_wb_mulsetcc_g(mdqctl_wb_mulsetcc_g),
                               .mdqctl_wb_yreg_shift_g(mdqctl_wb_yreg_shift_g),
                               .exu_mul_input_vld(exu_mul_input_vld),
                               .mdqctl_wb_yreg_wen_g(mdqctl_wb_yreg_wen_g),
                               .ecl_div_mul_sext_rs1_e(ecl_div_mul_sext_rs1_e),
                               .ecl_div_mul_sext_rs2_e(ecl_div_mul_sext_rs2_e),
                               .ecl_div_mul_get_new_data(ecl_div_mul_get_new_data),
                               .ecl_div_mul_keep_data(ecl_div_mul_keep_data),
                               .ecl_div_mul_get_32bit_data(ecl_div_mul_get_32bit_data),
                               .ecl_div_mul_wen(ecl_div_mul_wen),
                               // Inputs
                               .clk     (clk),
                               .se      (se),
                               .reset   (reset),
                               .ifu_exu_muldivop_d(ifu_exu_muldivop_d[4:0]),
                               .tid_d   (tid_d[1:0]),
                               .ifu_exu_rd_d(ifu_exu_rd_d[4:0]),
                               .tid_w1  (tid_w1[1:0]),
                               .flush_w1(flush_w1),
                               .ifu_exu_inst_vld_w(ifu_exu_inst_vld_w),
                               .wb_divcntl_ack_g(wb_divcntl_ack_g),
                               .divcntl_wb_req_g(divcntl_wb_req_g),
                               .mul_exu_ack(mul_exu_ack),
                               .ecl_div_sel_div(ecl_div_sel_div),
                               .ifu_exu_muls_d(ifu_exu_muls_d),
                               .div_ecl_detect_zero_high(div_ecl_detect_zero_high),
                               .div_ecl_detect_zero_low(div_ecl_detect_zero_low),
                               .ifu_tlu_flush_w(ifu_tlu_flush_w),
                               .early_flush_w(early_flush_w));

endmodule // sparc_exu_ecl


// Modified by Princeton University on June 9th, 2015
// ========== Copyright Header Begin ==========================================
// 
// OpenSPARC T1 Processor File: sparc_exu_ecl_cnt6.v
// Copyright (c) 2006 Sun Microsystems, Inc.  All Rights Reserved.
// DO NOT ALTER OR REMOVE COPYRIGHT NOTICES.
// 
// The above named program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public
// License version 2 as published by the Free Software Foundation.
// 
// The above named program is distributed in the hope that it will be 
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// General Public License for more details.
// 
// You should have received a copy of the GNU General Public
// License along with this work; if not, write to the Free Software
// Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA 02110-1301, USA.
// 
// ========== Copyright Header End ============================================
////////////////////////////////////////////////////////////////////////
/*
//  Module Name: sparc_exu_cnt6
//	Description: 6 bit binary counter
*/
module sparc_exu_ecl_cnt6 (/*AUTOARG*/
   // Outputs
   cntr, 
   // Inputs
   reset, clk, se
   ) ;
   input reset;
   input clk;
   input se;
   
   output [5:0] cntr;

   wire [5:0]   next_cntr;
   wire         tog1;
   wire         tog2;
   wire         tog3;
   wire         tog4;
   wire         tog5;

   assign       tog1 = cntr[0];
   assign       tog2 = cntr[0] & cntr[1];
   assign       tog3 = cntr[0] & cntr[1] & cntr[2];
   assign       tog4 = cntr[0] & cntr[1] & cntr[2] & cntr[3];
   assign       tog5 = cntr[0] & cntr[1] & cntr[2] & cntr[3] & cntr[4];
   assign next_cntr[0] = ~reset & ~cntr[0];
   assign next_cntr[1] = ~reset & ((~cntr[1] & tog1) | (cntr[1] & ~tog1)); 
   assign next_cntr[2] = ~reset & ((~cntr[2] & tog2) | (cntr[2] & ~tog2)); 
   assign next_cntr[3] = ~reset & ((~cntr[3] & tog3) | (cntr[3] & ~tog3)); 
   assign next_cntr[4] = ~reset & ((~cntr[4] & tog4) | (cntr[4] & ~tog4)); 
   assign next_cntr[5] = ~reset & ((~cntr[5] & tog5) | (cntr[5] & ~tog5)); 
   

   // counter flop
   dff_s #(6) cntr_dff(.din(next_cntr[5:0]), .clk(clk), .q(cntr[5:0]), .se(se), .si(), .so());
endmodule // sparc_exu_ecl_cnt6
// Modified by Princeton University on June 9th, 2015
// ========== Copyright Header Begin ==========================================
// 
// OpenSPARC T1 Processor File: sparc_exu_ecl_divcntl.v
// Copyright (c) 2006 Sun Microsystems, Inc.  All Rights Reserved.
// DO NOT ALTER OR REMOVE COPYRIGHT NOTICES.
// 
// The above named program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public
// License version 2 as published by the Free Software Foundation.
// 
// The above named program is distributed in the hope that it will be 
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// General Public License for more details.
// 
// You should have received a copy of the GNU General Public
// License along with this work; if not, write to the Free Software
// Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA 02110-1301, USA.
// 
// ========== Copyright Header End ============================================
////////////////////////////////////////////////////////////////////////
/*
//  Module Name: sparc_exu_divcntl
//	Description: Control block for div.  Division takes 1 cycle to load
//		the values, 65 cycles to calculate the result, and 1 cycle to 
//    calculate the ccs and check for overflow.
//   	Controlled by a one hot state machine and a 6 bit counter.
*/








module sparc_exu_ecl_divcntl (/*AUTOARG*/
   // Outputs
   ecl_div_xinmask, ecl_div_keep_d, ecl_div_ld_inputs, 
   ecl_div_sel_adder, ecl_div_last_cycle, ecl_div_almostlast_cycle, 
   ecl_div_sel_div, divcntl_wb_req_g, divcntl_ccr_cc_w2, 
   ecl_div_sel_64b, ecl_div_sel_u32, ecl_div_sel_pos32, 
   ecl_div_sel_neg32, ecl_div_upper32_zero, ecl_div_upper33_one, 
   ecl_div_upper33_zero, ecl_div_dividend_sign, ecl_div_newq, 
   ecl_div_subtract_l, ecl_div_keepx, ecl_div_cin, 
   // Inputs
   clk, se, reset, mdqctl_divcntl_input_vld, wb_divcntl_ack_g, 
   mdqctl_divcntl_reset_div, div_ecl_gencc_in_msb_l, 
   div_ecl_gencc_in_31, div_ecl_upper32_equal, div_ecl_low32_nonzero, 
   ecl_div_signed_div, div_ecl_dividend_msb, div_ecl_xin_msb_l, 
   div_ecl_x_msb, div_ecl_d_msb, div_ecl_cout64, 
   div_ecl_divisorin_31, ecl_div_div64, mdqctl_divcntl_muldone, 
   ecl_div_muls, div_ecl_adder_out_31, muls_rs1_31_m_l, 
   div_ecl_cout32, rs2_data_31_m, div_ecl_detect_zero_high, 
   div_ecl_detect_zero_low, div_ecl_d_62
   ) ;
   input     clk;
   input     se;
   input     reset;
   input     mdqctl_divcntl_input_vld;
   input     wb_divcntl_ack_g;
   input     mdqctl_divcntl_reset_div;
   input     div_ecl_gencc_in_msb_l;
   input     div_ecl_gencc_in_31;
   input     div_ecl_upper32_equal;
   input     div_ecl_low32_nonzero;
   input     ecl_div_signed_div;
   input     div_ecl_dividend_msb;
   input     div_ecl_xin_msb_l;
   input     div_ecl_x_msb;
   input     div_ecl_d_msb;
   input     div_ecl_cout64;
   input     div_ecl_divisorin_31;
   input     ecl_div_div64;
   input     mdqctl_divcntl_muldone;
   input     ecl_div_muls;
   input  div_ecl_adder_out_31;
   input  muls_rs1_31_m_l;
   input  div_ecl_cout32;
   input  rs2_data_31_m;
   input         div_ecl_detect_zero_high;
   input         div_ecl_detect_zero_low;
   input         div_ecl_d_62;
   
   output    ecl_div_xinmask;
   output    ecl_div_keep_d;
   output    ecl_div_ld_inputs;
   output    ecl_div_sel_adder;
   output    ecl_div_last_cycle;   // last cycle of calculation
   output    ecl_div_almostlast_cycle;//
   output    ecl_div_sel_div;
   output    divcntl_wb_req_g;
   output [7:0] divcntl_ccr_cc_w2;
   output       ecl_div_sel_64b;
   output       ecl_div_sel_u32;
   output       ecl_div_sel_pos32;
   output       ecl_div_sel_neg32;
   output       ecl_div_upper32_zero;
   output       ecl_div_upper33_one;
   output       ecl_div_upper33_zero;
   output       ecl_div_dividend_sign;
   output       ecl_div_newq;
   output       ecl_div_subtract_l;
   output       ecl_div_keepx;
   output        ecl_div_cin;
   
   wire         firstq;
   wire         q_next;        // next q bit
   wire         adderin1_64;   // msbs for adder
   wire         adderin2_64;
   wire         firstlast_sub; // subtract for first and last cycle
   wire         sub_next;      // next cycle will subtract
   wire         subtract;
   wire         bit64_halfadd; // partial result for qpredict
   wire         partial_qpredict;
   wire [1:0]   q_next_nocout;
   wire [1:0]   sub_next_nocout;
   wire         partial_qpredict_l;
   wire          divisor_sign;
   wire          detect_zero;
   wire          new_zero_rem_with_zero;
   wire          new_zero_rem_no_zero;
   wire          zero_rem_d;
   wire          zero_rem_q;
   wire          last_cin_with_zero;
   wire          last_cin_no_zero;
   wire          last_cin;
   wire          last_cin_next;
   
   // overflow correction wires
   wire          upper32_equal_d1;
   wire          gencc_in_msb_l_d1;
   wire          gencc_in_31_d1;
   wire          sel_div_d1;
   wire          low32_nonzero_d1;
   
   // Condition code generation wires
   wire [3:0]   xcc;
   wire [3:0]   icc;
   wire         unsign_ovfl;
   wire         pos_ovfl;
   wire         neg_ovfl;
   wire         muls_c;
   wire         next_muls_c;
   wire         muls_v;
   wire         next_muls_v;
   wire         muls_rs1_data_31_m;
   wire         div_adder_out_31_w;
   wire         rs2_data_31_w;
   wire         muls_rs1_data_31_w;
   wire         ovfl_32;
   wire         div_v;
   
   wire [5:0]   div_state;
   wire [5:0]   next_state;
   wire         go_idle,
                stay_idle,
                go_run,
                stay_run,
                go_last_calc,
                go_chk_ovfl,
                go_fix_ovfl,
                go_done,
                stay_done;


   wire         reset_cnt;
   wire [5:0]   cntr;
   wire         cntris63;

   /////////////////////////////////
   // G arbitration between MUL/DIV
   /////////////////////////////////
   assign        divcntl_wb_req_g = div_state[5] | 
                      (~(div_state[5] | div_state[3] | div_state[4]) &mdqctl_divcntl_muldone);
   assign        ecl_div_sel_div = ~(~(div_state[5] | div_state[3] | div_state[4]) & 
                                    mdqctl_divcntl_muldone);
   
   // state flop
   dff_s #(6) divstate_dff(.din(next_state[5:0]), .clk(clk), .q(div_state[5:0]), .se(se), .si(),
                    .so());

   // output logic and state decode
   assign        ecl_div_almostlast_cycle = go_last_calc & ~ecl_div_ld_inputs;
   assign        ecl_div_sel_adder = (div_state[1] | div_state[2]) & ~ecl_div_ld_inputs;
   assign        ecl_div_last_cycle = div_state[2];
   assign        ecl_div_ld_inputs = mdqctl_divcntl_input_vld;
   assign        ecl_div_keep_d = ~(ecl_div_sel_adder | ecl_div_ld_inputs);
   assign        reset_cnt = ~div_state[1];
   
   // next state logic
   assign        stay_idle = div_state[0] & ~mdqctl_divcntl_input_vld;
   assign        go_idle = div_state[5] & wb_divcntl_ack_g;
   assign        next_state[0] = go_idle | stay_idle | mdqctl_divcntl_reset_div | reset;

   assign        stay_run = div_state[1] & ~cntris63 & ~ecl_div_muls;
   assign        go_run = (div_state[0] & mdqctl_divcntl_input_vld);
   assign        next_state[1] = (go_run | stay_run) & 
                                      ~mdqctl_divcntl_reset_div & ~reset;

   assign        go_last_calc = div_state[1] & (cntris63);
   assign        next_state[2] = go_last_calc & ~mdqctl_divcntl_reset_div & ~reset;

   // chk_ovfl and fix_ovfl are place holders to guarantee that the overflow checking
   // takes place on the result.  No special logic occurs in them compared to the done state.
   assign        go_chk_ovfl = div_state[2];
   assign        next_state[3] = go_chk_ovfl & ~mdqctl_divcntl_reset_div & ~reset;

   assign        go_fix_ovfl = div_state[3] | (div_state[1] & ecl_div_muls);
   assign        next_state[4] = go_fix_ovfl & ~mdqctl_divcntl_reset_div & ~reset;

   assign        go_done = div_state[4];
   assign        stay_done = div_state[5] & ~wb_divcntl_ack_g;
   assign        next_state[5] = (go_done | stay_done) & ~mdqctl_divcntl_reset_div & ~reset;
   
   // counter
   sparc_exu_ecl_cnt6 cnt6(.reset       (reset_cnt),
                           /*AUTOINST*/
                           // Outputs
                           .cntr        (cntr[5:0]),
                           // Inputs
                           .clk         (clk),
                           .se          (se));

   assign        cntris63 = cntr[5] & cntr[4] & cntr[3] & cntr[2] & cntr[1] & cntr[0];


   ///////////////////////////////
   // Random logic for divider
   ///////////////////////////////
   // Generation of sign extension of dividend and divisor
   assign        ecl_div_dividend_sign = ecl_div_signed_div & div_ecl_dividend_msb;
   assign        ecl_div_xinmask = div_ecl_divisorin_31 & ecl_div_signed_div;

   assign        divisor_sign = div_ecl_x_msb & ecl_div_signed_div;
   
   // Generation of next bit of quotient
   ////////////////////////////////////////////////////////////////
   //	Calculate the next q.  Requires calculating the result
   // of the 65th bit of the adder and xoring it with the sign of
   // the divisor.  The order of these xors is switched for critical
   // path considerations.
   ////////////////////////////////////////////////////////////////
   assign        adderin1_64 = div_ecl_d_msb;
   assign        adderin2_64 = (ecl_div_signed_div & div_ecl_x_msb) ^ subtract;
   assign        bit64_halfadd = adderin1_64 ^ adderin2_64;
   assign        partial_qpredict = bit64_halfadd ^ ~(div_ecl_x_msb & ecl_div_signed_div);
   assign        partial_qpredict_l = ~partial_qpredict;
   //assign        qpredict = partial_qpredict ^ div_ecl_cout64;
   //assign        firstq = ~ecl_div_signed_div | div_ecl_xin_msb_l; 
   assign        firstq = ecl_div_dividend_sign;

   mux2ds #(2) qnext_mux(.dout(q_next_nocout[1:0]), 
                            .in0({partial_qpredict, partial_qpredict_l}),
                            .in1({2{firstq}}),
                            .sel0(~ecl_div_ld_inputs),
                            .sel1(ecl_div_ld_inputs));
   dp_mux2es qnext_cout_mux(.dout(q_next),
                            .in0(q_next_nocout[1]),
                            .in1(q_next_nocout[0]),
                            .sel(div_ecl_cout64));

   dff_s q_dff(.din(q_next), .clk(clk), .q(ecl_div_newq), .se(se), .si(),
               .so());


   ////////////////////////////
   // Subtraction logic and subtract flop
   //-------------------------------------
   // To take the subtraction calc out of the critical path,
   // it is done in the previous cycle and part is done with a
   // mux.  The result is put into a flop.
   ////////////////////////////
   assign firstlast_sub = ~ecl_div_almostlast_cycle & ~ecl_div_muls &
          (~ecl_div_signed_div | ~(div_ecl_dividend_msb ^ ~div_ecl_xin_msb_l));
                                                       
   assign        ecl_div_keepx = ~(ecl_div_ld_inputs |
                                  ecl_div_almostlast_cycle);

   mux2ds #(2) subnext_mux(.dout(sub_next_nocout[1:0]), 
                              .in0({2{firstlast_sub}}),
                              .in1({partial_qpredict, partial_qpredict_l}),
                              .sel0(~ecl_div_keepx),
                              .sel1(ecl_div_keepx));
   dp_mux2es subtract_cout_mux(.dout(sub_next),
                            .in0(sub_next_nocout[1]),
                            .in1(sub_next_nocout[0]),
                            .sel(div_ecl_cout64));
   
   dff_s sub_dff(.din(sub_next), .clk(clk), .q(subtract), .se(se), .si(),
               .so());

   assign        ecl_div_subtract_l = ~subtract;


   /////////////////////////////////////////////
   // Carry in logic
   //--------------------------------------------
   // The carry is usually just subtract.  The
   // quotient correction for signed division
   // sometimes has to adjust it though.
   /////////////////////////////////////////////
   assign        detect_zero = div_ecl_detect_zero_low & div_ecl_detect_zero_high;

   assign ecl_div_cin = (ecl_div_last_cycle)? last_cin: subtract;
   // stores if the partial remainder was ever zero.
/* -----\/----- EXCLUDED -----\/-----
   // changed for timing
    assign        zero_rem_d = ~ecl_div_ld_inputs & (div_ecl_detect_zero | zero_rem_q) & 
                                                     (~div_ecl_d_62 | ecl_div_almostlast_cycle);
 -----/\----- EXCLUDED -----/\----- */
   assign new_zero_rem_with_zero = ~ecl_div_ld_inputs & (~div_ecl_d_62 | ecl_div_almostlast_cycle);
   assign new_zero_rem_no_zero = zero_rem_q & new_zero_rem_with_zero;
   assign zero_rem_d = (detect_zero)? new_zero_rem_with_zero: new_zero_rem_no_zero;
   dff_s zero_rem_dff(.din(zero_rem_d), .clk(clk), .q(zero_rem_q),
                    .se(se), .si(), .so());
   
/* -----\/----- EXCLUDED -----\/-----
   // changed for timing
   assign last_cin_next = ecl_div_signed_div & (divisor_sign & ~div_ecl_d_62 | 
                                                ~divisor_sign &div_ecl_d_62&~zero_rem_d |
                                                divisor_sign &div_ecl_d_62&zero_rem_d);
 -----/\----- EXCLUDED -----/\----- */
   assign last_cin_with_zero = ecl_div_signed_div & (divisor_sign & ~div_ecl_d_62 | 
                                                ~divisor_sign &div_ecl_d_62&~new_zero_rem_with_zero |
                                                divisor_sign &div_ecl_d_62&new_zero_rem_with_zero);
   assign last_cin_no_zero = ecl_div_signed_div & (divisor_sign & ~div_ecl_d_62 | 
                                                ~divisor_sign &div_ecl_d_62&~new_zero_rem_no_zero |
                                                divisor_sign &div_ecl_d_62&new_zero_rem_no_zero);
   assign last_cin_next = (detect_zero)? last_cin_with_zero: last_cin_no_zero;
   dff_s last_cin_dff(.din(last_cin_next), .clk(clk), .q(last_cin),
                    .se(se), .si(), .so());
   
   ///////////////////////////////
   // Condition code generation
   ///////////////////////////////
   // There is a special case:
   // For 64 bit signed division largest neg/-1 = largest neg
   // However for 32 bit division this will give us positive overflow.
   // This is detected by a sign switch on this case.
   wire   inputs_neg_d;
   wire   inputs_neg_q;
   wire   large_neg_ovfl;
   assign inputs_neg_d = div_ecl_dividend_msb & div_ecl_divisorin_31;
   assign large_neg_ovfl = inputs_neg_q & ~gencc_in_msb_l_d1;
   dffe_s inputs_neg_dff(.din(inputs_neg_d), .clk(clk), .q(inputs_neg_q), 
                       .en(ecl_div_ld_inputs), .se(se), .si(), .so());
   dff_s #(5) cc_sig_dff(.din({div_ecl_upper32_equal, div_ecl_gencc_in_msb_l,
                             div_ecl_gencc_in_31, ecl_div_sel_div, div_ecl_low32_nonzero}),
                         .q({upper32_equal_d1, gencc_in_msb_l_d1,
                             gencc_in_31_d1, sel_div_d1, low32_nonzero_d1}),
                         .clk(clk), .se(se), .si(), .so());
   // selects for correcting divide overflow
   assign        ecl_div_sel_64b = ecl_div_div64 | ecl_div_muls;
   assign        ecl_div_sel_u32 = ~ecl_div_sel_64b & ~ecl_div_signed_div;
   assign 			 ecl_div_sel_pos32 = (~ecl_div_sel_64b & ecl_div_signed_div & 
                                      (gencc_in_msb_l_d1 | large_neg_ovfl));
   assign        ecl_div_sel_neg32 = (~ecl_div_sel_64b & ecl_div_signed_div & 
                                      ~gencc_in_msb_l_d1 & ~large_neg_ovfl);

   // results of checking are staged one cycle for timing reasons
   // this is the reason for the chk and fix ovfl states
   assign        ecl_div_upper32_zero = upper32_equal_d1 & gencc_in_msb_l_d1;
   assign        ecl_div_upper33_zero = (upper32_equal_d1 & gencc_in_msb_l_d1 & 
                                         ~gencc_in_31_d1);
   assign        ecl_div_upper33_one = (upper32_equal_d1 & ~gencc_in_msb_l_d1 & 
                                        gencc_in_31_d1);

   // divide overflow
   assign        unsign_ovfl = ecl_div_sel_u32 & ~ecl_div_upper32_zero & sel_div_d1;
   assign        pos_ovfl = ecl_div_sel_pos32 & ~ecl_div_upper33_zero & sel_div_d1;
   assign        neg_ovfl = ecl_div_sel_neg32 & ~ecl_div_upper33_one & sel_div_d1;
   assign        div_v = pos_ovfl | unsign_ovfl | neg_ovfl;

   // muls carry and overflow
   assign next_muls_c = (div_state[1]) ? div_ecl_cout32: muls_c;

   assign        muls_rs1_data_31_m = ~muls_rs1_31_m_l;
   dff_s #(3) muls_overlow_dff(.din({muls_rs1_data_31_m, rs2_data_31_m, div_ecl_adder_out_31}),
                             .q({muls_rs1_data_31_w, rs2_data_31_w, div_adder_out_31_w}),
                             .clk(clk), .se(se), .si(), .so());
   assign ovfl_32 = ((muls_rs1_data_31_w & rs2_data_31_w & ~div_adder_out_31_w) |
                     (~muls_rs1_data_31_w & ~rs2_data_31_w & div_adder_out_31_w));
   assign next_muls_v = (div_state[4]) ? ovfl_32: muls_v;
   dff_s muls_c_dff(.din(next_muls_c), .clk(clk), .q(muls_c),
                  .se(se), .si(), .so());
   dff_s muls_v_dff(.din(next_muls_v), .clk(clk), .q(muls_v),
                  .se(se), .si(), .so());
  
   // negative
   assign xcc[3] = ~gencc_in_msb_l_d1 & ~unsign_ovfl & ~pos_ovfl;
   assign icc[3] = (gencc_in_31_d1 & ~pos_ovfl) | neg_ovfl | unsign_ovfl;
   // zero
   assign xcc[2] = upper32_equal_d1 & gencc_in_msb_l_d1 & ~low32_nonzero_d1;
   assign icc[2] = ~low32_nonzero_d1 & ~div_v; // nonzero checks before ovfl
   //overflow
   assign xcc[1] = 1'b0;
   assign icc[1] = (ecl_div_muls & sel_div_d1) ? muls_v: div_v;
   // carry
   assign xcc[0] = 1'b0;
   assign icc[0] = ecl_div_muls & sel_div_d1 & muls_c;

   assign divcntl_ccr_cc_w2 = {xcc, icc};

endmodule // sparc_exu_divcntl
// Modified by Princeton University on June 9th, 2015
// ========== Copyright Header Begin ==========================================
// 
// OpenSPARC T1 Processor File: sparc_exu_ecl_eccctl.v
// Copyright (c) 2006 Sun Microsystems, Inc.  All Rights Reserved.
// DO NOT ALTER OR REMOVE COPYRIGHT NOTICES.
// 
// The above named program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public
// License version 2 as published by the Free Software Foundation.
// 
// The above named program is distributed in the hope that it will be 
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// General Public License for more details.
// 
// You should have received a copy of the GNU General Public
// License along with this work; if not, write to the Free Software
// Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA 02110-1301, USA.
// 
// ========== Copyright Header End ============================================
////////////////////////////////////////////////////////////////////////
/*
//  Module Name: sparc_exu_ecl_eccctl
//	Description:  Implements the control logic for ecc checking.
//		This includes picking which error to fix (only one fixed per instruction),
//		enabling the checks, and signalling the errors.
*/

module sparc_exu_ecl_eccctl (/*AUTOARG*/
   // Outputs
   ue_trap_m, ecl_ecc_sel_rs1_m_l, ecl_ecc_sel_rs2_m_l, 
   ecl_ecc_sel_rs3_m_l, ecl_ecc_log_rs1_m, ecl_ecc_log_rs2_m, 
   ecl_ecc_log_rs3_m, ecl_byp_sel_ecc_m, ecl_ecc_rs1_use_rf_e, 
   ecl_ecc_rs2_use_rf_e, ecl_ecc_rs3_use_rf_e, eccctl_wb_rd_m, 
   exu_ifu_ecc_ce_m, exu_ifu_ecc_ue_m, exu_ifu_err_reg_m, 
   ecl_byp_ecc_mask_m_l, exu_ifu_inj_ack, exu_ifu_err_synd_7_m, 
   // Inputs
   clk, se, rst_tri_en, ecc_ecl_rs1_ce, ecc_ecl_rs1_ue, 
   ecc_ecl_rs2_ce, ecc_ecl_rs2_ue, ecc_ecl_rs3_ce, ecc_ecl_rs3_ue, 
   ecl_byp_rcc_mux2_sel_rf, ecl_byp_rs2_mux2_sel_rf, 
   ecl_byp_rs3_mux2_sel_rf, rs1_vld_e, rs2_vld_e, rs3_vld_e, 
   ifu_exu_rs1_m, ifu_exu_rs2_m, ifu_exu_rs3_m, rml_ecl_cwp_d, 
   ifu_exu_ecc_mask, ifu_exu_inj_irferr, ifu_exu_disable_ce_e, 
   wb_eccctl_spec_wen_next, ifu_exu_nceen_e, ifu_exu_inst_vld_e, 
   rml_ecl_gl_e, cancel_rs3_ecc_e
   ) ;
   input clk;
   input se;
   input rst_tri_en;
   input       ecc_ecl_rs1_ce;
   input       ecc_ecl_rs1_ue;
   input       ecc_ecl_rs2_ce;
   input       ecc_ecl_rs2_ue;
   input       ecc_ecl_rs3_ce;
   input       ecc_ecl_rs3_ue;
   input       ecl_byp_rcc_mux2_sel_rf;
   input       ecl_byp_rs2_mux2_sel_rf;
   input       ecl_byp_rs3_mux2_sel_rf;
   input       rs1_vld_e;
   input       rs2_vld_e;
   input       rs3_vld_e;
   input [4:0] ifu_exu_rs1_m;
   input [4:0] ifu_exu_rs2_m;
   input [4:0] ifu_exu_rs3_m;
   input [2:0] rml_ecl_cwp_d;
   input [7:0] ifu_exu_ecc_mask;
   input       ifu_exu_inj_irferr;
   input       ifu_exu_disable_ce_e;
   input       wb_eccctl_spec_wen_next;
   input       ifu_exu_nceen_e;
   input       ifu_exu_inst_vld_e;
   input [1:0] rml_ecl_gl_e;
   input       cancel_rs3_ecc_e;
   
   output      ue_trap_m;
   output      ecl_ecc_sel_rs1_m_l;
   output      ecl_ecc_sel_rs2_m_l;
   output      ecl_ecc_sel_rs3_m_l;
   output      ecl_ecc_log_rs1_m;
   output      ecl_ecc_log_rs2_m;
   output      ecl_ecc_log_rs3_m;
   output      ecl_byp_sel_ecc_m;
   output      ecl_ecc_rs1_use_rf_e;
   output      ecl_ecc_rs2_use_rf_e;
   output      ecl_ecc_rs3_use_rf_e;
   output [4:0] eccctl_wb_rd_m;
   output       exu_ifu_ecc_ce_m;
   output       exu_ifu_ecc_ue_m;
   output [7:0] exu_ifu_err_reg_m;
   output [7:0] ecl_byp_ecc_mask_m_l;
   output       exu_ifu_inj_ack;
   output    exu_ifu_err_synd_7_m;
   
   wire      sel_rs1_e;
   wire      sel_rs2_e;
   wire      sel_rs3_e;
   wire        sel_rs1_m;
   wire        sel_rs2_m;
   wire        sel_rs3_m;
   wire        safe_sel_rs1_m;
   wire        safe_sel_rs2_m;
   wire        safe_sel_rs3_m;
   wire [2:0]  cwp_e;
   wire [2:0]  cwp_m;
   wire [1:0]  gl_m;
   wire        inj_irferr_m;
   wire        inj_irferr_w;
   wire        detect_ce_e;
   wire        detect_ue_e;
   wire        flag_ecc_ce_e;
   wire        flag_ecc_ue_e;
   wire [4:0]  log_rs_m;
   wire        rs1_ce_m;
   wire        rs1_ue_m;
   wire        rs2_ce_m;
   wire        rs2_ue_m;
   wire        rs3_ue_m;
   wire        rs1_sel_rf_e;
   wire        rs2_sel_rf_e;
   wire        rs3_sel_rf_e;
   wire        vld_rs3_ce_e;
   wire        vld_rs3_ue_e;
   wire        nceen_m;
   
   // Store whether rf value was used for ecc checking
   assign      ecl_ecc_rs1_use_rf_e = rs1_sel_rf_e & rs1_vld_e & ifu_exu_inst_vld_e;
   assign      ecl_ecc_rs2_use_rf_e = rs2_sel_rf_e & rs2_vld_e & ifu_exu_inst_vld_e;
   assign      ecl_ecc_rs3_use_rf_e = rs3_sel_rf_e & rs3_vld_e & ifu_exu_inst_vld_e; 

   dff_s rs1_rf_dff(.din(ecl_byp_rcc_mux2_sel_rf), .clk(clk),
                  .q(rs1_sel_rf_e), .se(se), .si(), .so());
   dff_s rs2_rf_dff(.din(ecl_byp_rs2_mux2_sel_rf), .clk(clk),
                  .q(rs2_sel_rf_e), .se(se), .si(), .so());
   dff_s rs3_rf_dff(.din(ecl_byp_rs3_mux2_sel_rf), .clk(clk),
                  .q(rs3_sel_rf_e), .se(se), .si(), .so());

   assign      vld_rs3_ce_e = ecc_ecl_rs3_ce & ~cancel_rs3_ecc_e;
   assign      vld_rs3_ue_e = ecc_ecl_rs3_ue & ~cancel_rs3_ecc_e;
   assign    detect_ce_e = (ecc_ecl_rs1_ce | ecc_ecl_rs2_ce | vld_rs3_ce_e);
   assign    detect_ue_e = (ecc_ecl_rs1_ue | ecc_ecl_rs2_ue | vld_rs3_ue_e);
   // Generate trap signals
   assign    flag_ecc_ue_e = (detect_ue_e | 
                                    detect_ce_e & ifu_exu_disable_ce_e); // convert ce to ue
   assign    flag_ecc_ce_e = detect_ce_e & ~ifu_exu_disable_ce_e;

   // Pass along signal to fix errors
   dff_s byp_sel_ecc_e2m(.din(flag_ecc_ce_e), .clk(clk), .q(ecl_byp_sel_ecc_m),
                       .se(se), .si(), .so());
   dff_s ecc_ue_e2m(.din(flag_ecc_ue_e), .clk(clk), .q(exu_ifu_ecc_ue_m),
                  .se(se), .si(), .so());
   dff_s nceen_e2m(.din(ifu_exu_nceen_e), .clk(clk), .q(nceen_m), .se(se), .si(), .so());
   assign    ue_trap_m = exu_ifu_ecc_ue_m & nceen_m;
   // only report ce (and replay) if no ue
   assign      exu_ifu_ecc_ce_m = ecl_byp_sel_ecc_m & ~exu_ifu_ecc_ue_m;
   // if globals then report %gl.  otherwise log %cwp
   assign      exu_ifu_err_reg_m[7:5] = (~log_rs_m[4] & ~log_rs_m[3])? {1'b0,gl_m[1:0]}: cwp_m[2:0];
   assign      exu_ifu_err_reg_m[4:0] = log_rs_m[4:0];
   
   // Control for mux to ecc decoder (just ce)
   assign      sel_rs1_e = ecc_ecl_rs1_ce;
   assign      sel_rs2_e = ~ecc_ecl_rs1_ce & ecc_ecl_rs2_ce;
   assign      sel_rs3_e = ~(ecc_ecl_rs1_ce | ecc_ecl_rs2_ce);
   
   dff_s ecc_sel_rs1_dff(.din(sel_rs1_e), .clk(clk), .q(sel_rs1_m),
                       .se(se), .si(), .so());
   dff_s ecc_sel_rs2_dff(.din(sel_rs2_e), .clk(clk), .q(sel_rs2_m),
                       .se(se), .si(), .so());
   dff_s ecc_sel_rs3_dff(.din(sel_rs3_e), .clk(clk), .q(sel_rs3_m),
                       .se(se), .si(), .so());
   // Make selects one hot
   assign      safe_sel_rs1_m = sel_rs1_m | rst_tri_en;
   assign      safe_sel_rs2_m = sel_rs2_m & ~rst_tri_en;
   assign      safe_sel_rs3_m = sel_rs3_m & ~rst_tri_en;
   assign      ecl_ecc_sel_rs1_m_l = ~safe_sel_rs1_m;
   assign      ecl_ecc_sel_rs2_m_l = ~safe_sel_rs2_m;
   assign      ecl_ecc_sel_rs3_m_l = ~safe_sel_rs3_m;

   // Mux to generate the rd for fixed value
   mux3ds #(5) ecc_rd_mux(.dout(eccctl_wb_rd_m[4:0]),
                          .in0(ifu_exu_rs1_m[4:0]),
                          .in1(ifu_exu_rs2_m[4:0]),
                          .in2(ifu_exu_rs3_m[4:0]),
                          .sel0(safe_sel_rs1_m),
                          .sel1(safe_sel_rs2_m),
                          .sel2(safe_sel_rs3_m));

   // Control for muxes for logging errors
   assign      ecl_ecc_log_rs1_m = rs1_ue_m | (rs1_ce_m & ~rs2_ue_m & ~rs3_ue_m);
   assign      ecl_ecc_log_rs2_m = (rs2_ue_m & ~rs1_ue_m) | (rs2_ce_m & ~rs1_ue_m & ~rs1_ce_m & ~rs3_ue_m);
   assign      ecl_ecc_log_rs3_m = ~(ecl_ecc_log_rs1_m | ecl_ecc_log_rs2_m);
   // Mux to generate the rs for error_logging
   mux3ds #(5) ecc_rdlog_mux(.dout(log_rs_m[4:0]),
                          .in0(ifu_exu_rs1_m[4:0]),
                          .in1(ifu_exu_rs2_m[4:0]),
                          .in2(ifu_exu_rs3_m[4:0]),
                          .sel0(ecl_ecc_log_rs1_m),
                          .sel1(ecl_ecc_log_rs2_m),
                          .sel2(ecl_ecc_log_rs3_m));

   dff_s #(3) cwp_d2e(.din(rml_ecl_cwp_d[2:0]), .clk(clk), .q(cwp_e[2:0]),
                    .se(se), .si(), .so());
   dff_s #(3) cwp_e2m(.din(cwp_e[2:0]), .clk(clk), .q(cwp_m[2:0]),
                    .se(se), .si(), .so());
   dff_s #(2) gl_e2m(.din(rml_ecl_gl_e[1:0]), .clk(clk), .q(gl_m[1:0]),
                   .se(se), .si(), .so());

   // Syndrome needs to know if it was really a ce or ue
   mux3ds ecc_synd7_mux(.dout(exu_ifu_err_synd_7_m),
                        .in0(rs1_ce_m),
                        .in1(rs2_ce_m),
                        .in2(~rs3_ue_m),
                        .sel0(ecl_ecc_log_rs1_m),
                        .sel1(ecl_ecc_log_rs2_m),
                        .sel2(ecl_ecc_log_rs3_m));


   // signals for injecting errors
   // inject error if it is enabled and a write will probably happen
   // (don't bother to check kill_w
   assign      inj_irferr_m = wb_eccctl_spec_wen_next & ifu_exu_inj_irferr;
   assign      ecl_byp_ecc_mask_m_l = ~(ifu_exu_ecc_mask[7:0] & {8{inj_irferr_m}});
   dff_s inj_irferr_m2w(.din(inj_irferr_m), .clk(clk), .q(inj_irferr_w),
                      .se(se), .si(), .so());
   assign      exu_ifu_inj_ack = inj_irferr_w;

   // Pipeline Flops
   dff_s rs1_ue_e2m(.din(ecc_ecl_rs1_ue), .clk(clk), .q(rs1_ue_m), .se(se), .si(), .so());
   dff_s rs1_ce_e2m(.din(ecc_ecl_rs1_ce), .clk(clk), .q(rs1_ce_m), .se(se), .si(), .so());
   dff_s rs2_ue_e2m(.din(ecc_ecl_rs2_ue), .clk(clk), .q(rs2_ue_m), .se(se), .si(), .so());
   dff_s rs2_ce_e2m(.din(ecc_ecl_rs2_ce), .clk(clk), .q(rs2_ce_m), .se(se), .si(), .so());
   dff_s rs3_ue_e2m(.din(vld_rs3_ue_e), .clk(clk), .q(rs3_ue_m), .se(se), .si(), .so());
   
endmodule // sparc_exu_ecl_eccctl
// Modified by Princeton University on June 9th, 2015
// ========== Copyright Header Begin ==========================================
// 
// OpenSPARC T1 Processor File: sparc_exu_ecl_mdqctl.v
// Copyright (c) 2006 Sun Microsystems, Inc.  All Rights Reserved.
// DO NOT ALTER OR REMOVE COPYRIGHT NOTICES.
// 
// The above named program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public
// License version 2 as published by the Free Software Foundation.
// 
// The above named program is distributed in the hope that it will be 
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// General Public License for more details.
// 
// You should have received a copy of the GNU General Public
// License along with this work; if not, write to the Free Software
// Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA 02110-1301, USA.
// 
// ========== Copyright Header End ============================================
////////////////////////////////////////////////////////////////////////
/*
//  Module Name: sparc_exu_ecl_mdqctl
//	Description:  This block is the control logic for the multiply/divide
// 	input buffer.  It generates the select lines for both the output
//	to mul and div, as well as for moving the data within the buffer.
//	There are 4 slots in the buffer, which is a modified FIFO.
//	It will output 1 MUL and 1 DIV every cycle, as well as whether those
// 	outputs are valid.  If none of the slots contain a valid entry, it
//	will pass through the input to the output.  If a kill comes through
//	and invalidates an entry, it will show up on the valid bit coming out
//	of the mdq, but may cause a lost cycle as the kill won't affect the logic
//	which chooses the output until the next cycle.  The block also
//	stores the thr, rd, setcc and other control bits for each entry.
*/






module sparc_exu_ecl_mdqctl (/*AUTOARG*/
   // Outputs
   mdqctl_divcntl_input_vld, mdqctl_divcntl_reset_div, 
   mdqctl_divcntl_muldone, ecl_div_div64, ecl_div_signed_div, 
   ecl_div_muls, mdqctl_wb_divthr_g, mdqctl_wb_divrd_g, 
   mdqctl_wb_multhr_g, mdqctl_wb_mulrd_g, mdqctl_wb_divsetcc_g, 
   mdqctl_wb_mulsetcc_g, mdqctl_wb_yreg_shift_g, exu_mul_input_vld, 
   mdqctl_wb_yreg_wen_g, ecl_div_mul_sext_rs1_e, 
   ecl_div_mul_sext_rs2_e, ecl_div_mul_get_new_data, 
   ecl_div_mul_keep_data, ecl_div_mul_get_32bit_data, 
   ecl_div_mul_wen, div_zero_m, 
   // Inputs
   clk, se, reset, ifu_exu_muldivop_d, tid_d, ifu_exu_rd_d, tid_w1, 
   flush_w1, ifu_exu_inst_vld_w, wb_divcntl_ack_g, divcntl_wb_req_g, 
   byp_alu_rs1_data_31_e, byp_alu_rs2_data_31_e, mul_exu_ack, 
   ecl_div_sel_div, ifu_exu_muls_d, div_ecl_detect_zero_high, 
   div_ecl_detect_zero_low, ifu_tlu_flush_w, early_flush_w
   ) ;
   input clk;
   input se;
   input reset;
   input [4:0] ifu_exu_muldivop_d;
   input [1:0] tid_d;
   input [4:0] ifu_exu_rd_d;
   input [1:0] tid_w1;
   input       flush_w1;
   input       ifu_exu_inst_vld_w;
   input       wb_divcntl_ack_g;
   input       divcntl_wb_req_g;
   input       byp_alu_rs1_data_31_e;
   input       byp_alu_rs2_data_31_e;
   input       mul_exu_ack;
   input       ecl_div_sel_div;
   input       ifu_exu_muls_d;
   input       div_ecl_detect_zero_high;
   input       div_ecl_detect_zero_low;
   input       ifu_tlu_flush_w;
   input       early_flush_w;

   
   output      mdqctl_divcntl_input_vld;
   output      mdqctl_divcntl_reset_div;
   output      mdqctl_divcntl_muldone;
   output      ecl_div_div64;
   output      ecl_div_signed_div;
   output      ecl_div_muls;
   output [1:0] mdqctl_wb_divthr_g;
   output [4:0] mdqctl_wb_divrd_g;
   output [1:0] mdqctl_wb_multhr_g;
   output [4:0] mdqctl_wb_mulrd_g;
   output       mdqctl_wb_divsetcc_g;
   output       mdqctl_wb_mulsetcc_g;
   output       mdqctl_wb_yreg_shift_g;

   
   output       exu_mul_input_vld;
   output       mdqctl_wb_yreg_wen_g;
   output       ecl_div_mul_sext_rs1_e;
   output       ecl_div_mul_sext_rs2_e;
   output       ecl_div_mul_get_new_data;
   output       ecl_div_mul_keep_data;
   output       ecl_div_mul_get_32bit_data;
   output       ecl_div_mul_wen;
   output   div_zero_m;

   wire [11:0] div_data_next;
   wire [11:0] div_data;
   wire        new_div_vld;
   wire        curr_div_vld;
   wire [11:0] div_input_data_d;
   wire [9:0] mul_input_data_d;
   wire [9:0] mul_data;
   wire [9:0] mul_data_next;
   wire        new_mul_d;
   wire        kill_thr_mul;
   wire        mul_kill;
   wire        invalid_mul_w;
   wire        div_kill;
   wire        kill_thr_div;
   
   wire        mul_ready_next;
   wire        mul_ready;
   wire        mul_done_valid_c0;
   wire        mul_done_valid_c1;
   wire        mul_done_ack;
   wire        mul_done_c0;
   wire        mul_done_c1;
   wire        mul_done_c2;
   wire        mul_done_c3;

   wire        isdiv_e_valid;
   wire        isdiv_m_valid;
   wire        ismul_e_valid;
   wire        ismul_m_valid;
   wire        isdiv_e;
   wire        isdiv_m;
   wire        isdiv_w;
   wire        ismul_e;
   wire        ismul_m;
   wire        ismul_w;
   
   wire        div_used;
   wire        invalid_div_w;
   wire        div_zero_e;

   // Mul result state wires
   wire        go_mul_done;
   wire        stay_mul_done;
   wire        mul_done;
   wire        next_mul_done;
   
   
   ////////////////////////
   // Divide  output DATAPATH
   ////////////////////////
   // store control signals
   assign div_used = divcntl_wb_req_g & wb_divcntl_ack_g & ecl_div_sel_div;

   assign new_div_vld = ifu_exu_muls_d | ifu_exu_muldivop_d[3];
   
   assign div_input_data_d[11:0] = {1'b1, // isdiv
                                    ifu_exu_muls_d,
                                    ifu_exu_muldivop_d[2], // 64bit
                                    ifu_exu_muldivop_d[1], // signed
                                    ifu_exu_muldivop_d[0], // setcc
                                    ifu_exu_rd_d[4:0],
                                    tid_d[1:0]};
   mux2ds #(12) div_data_mux(.dout(div_data_next[11:0]),
                                .in0({curr_div_vld, div_data[10:0]}),
                                .in1(div_input_data_d[11:0]),
                                .sel0(~new_div_vld),
                                .sel1(new_div_vld));

   dffr_s #(12) div_data_dff(.din(div_data_next[11:0]), .clk(clk), .q(div_data[11:0]),
                          .se(se), .si(), .so(), .rst(reset));

   //div  kill logic (kills on div by zero exception or if there isn't an outstanding div)
   assign div_zero_e = isdiv_e & div_ecl_detect_zero_high & div_ecl_detect_zero_low & ~div_data[10];
   assign invalid_div_w = isdiv_w & (~ifu_exu_inst_vld_w | ifu_tlu_flush_w | early_flush_w);
   assign kill_thr_div = ~(div_data[1] ^ tid_w1[1]) & ~(div_data[0] ^ tid_w1[0]);
   assign div_kill = (flush_w1 & kill_thr_div) | invalid_div_w | new_div_vld;
   assign curr_div_vld = div_data[11] & ~div_zero_m & ~div_kill & ~div_used;

   wire   div_zero_unqual_m;
   assign div_zero_m = div_zero_unqual_m & isdiv_m;
   dff_s div_zero_e2m(.din(div_zero_e), .clk(clk), .q(div_zero_unqual_m), .se(se), .si(), .so());
   
   // pipeling for divide valid signal (for inst_vld checking)
   dff_s isdiv_d2e(.din(new_div_vld), .clk(clk), .q(isdiv_e),
                 .se(se), .si(), .so());
   dff_s isdiv_e2m(.din(isdiv_e_valid), .clk(clk), .q(isdiv_m),
                 .se(se), .si(), .so());
   dff_s isdiv_m2w(.din(isdiv_m_valid), .clk(clk), .q(isdiv_w),
                 .se(se), .si(), .so());
   assign        isdiv_e_valid = isdiv_e & ~div_kill;
   assign        isdiv_m_valid = isdiv_m & ~div_kill;

   // control for div state machine
   assign mdqctl_divcntl_reset_div = (~div_data[11] | div_kill);
   assign mdqctl_divcntl_input_vld = isdiv_e;

   // control signals for div
   assign ecl_div_div64 = div_data[9];
   assign ecl_div_signed_div = div_data[8];
   assign ecl_div_muls = div_data[10];
   
   // control for writeback on completion
   assign mdqctl_wb_divrd_g[4:0] = div_data[6:2];
   assign mdqctl_wb_divthr_g[1:0] = div_data[1:0];
   assign mdqctl_wb_divsetcc_g = div_data[7] | div_data[10];
   assign mdqctl_wb_yreg_shift_g = div_used & div_data[10];

   
   ////////////////////////////////////////////////////////////////////////////
   // Multiply control
   //----------------------
   // The multiply will drop the current operation if a new request is issued.
   // This requires addition checking to make sure that the kills are for the
   // proper operation.
   ////////////////////////////////////////////////////////////////////////////
   dff_s ismul_d2e(.din(ifu_exu_muldivop_d[4]), .clk(clk), .q(ismul_e),
                 .se(se), .si(), .so());
   dff_s ismul_e2m(.din(ismul_e_valid), .clk(clk), .q(ismul_m),
                 .se(se), .si(), .so());
   dff_s ismul_m2w(.din(ismul_m_valid), .clk(clk), .q(ismul_w),
                 .se(se), .si(), .so());
   assign ismul_e_valid = ismul_e & ~mul_kill;
   assign        ismul_m_valid = ismul_m & ~mul_kill & ~ismul_e;
   
   // store control signals
  //   assign mul_used = divcntl_wb_req_g & wb_divcntl_ack_g & ~ecl_div_sel_div;
   assign new_mul_d = ifu_exu_muldivop_d[4];
   
   assign mul_input_data_d[9:0] = {ifu_exu_muldivop_d[2], // 64bit
                                    ifu_exu_muldivop_d[1], // signed
                                    ifu_exu_muldivop_d[0], // setcc
                                    ifu_exu_rd_d[4:0],
                                    tid_d[1:0]};
   assign mul_data_next[9:0] = (new_mul_d)? mul_input_data_d[9:0]: mul_data[9:0];

   dff_s #(10) mul_data_dff(.din(mul_data_next[9:0]), .clk(clk), .q(mul_data[9:0]),
                          .se(se), .si(), .so());

   // mul kill logic
   assign kill_thr_mul = ~(mul_data[1] ^ tid_w1[1]) & ~(mul_data[0] ^ tid_w1[0]);
   assign mul_kill = (flush_w1 & kill_thr_mul) | reset;
   assign invalid_mul_w = ismul_w & ~ifu_exu_inst_vld_w;
   
   // control signals for mul data in div unit
   assign      ecl_div_mul_keep_data = ~ismul_e;
   assign      ecl_div_mul_get_new_data = ismul_e & mul_data[9];
   assign      ecl_div_mul_get_32bit_data = ismul_e & ~mul_data[9];
   assign      ecl_div_mul_sext_rs1_e = byp_alu_rs1_data_31_e & mul_data[8];
   assign      ecl_div_mul_sext_rs2_e = byp_alu_rs2_data_31_e & mul_data[8];

   // control for writeback on completion
   assign      mdqctl_wb_yreg_wen_g = ~mul_data[9] & ecl_div_mul_wen;
   assign      mdqctl_wb_multhr_g[1:0] = mul_data[1:0];
   assign      mdqctl_wb_mulsetcc_g = mul_data[7];
   assign      mdqctl_wb_mulrd_g[4:0] = mul_data[6:2];

   // interface with mul and state of pending mul
   assign      mul_ready_next = ismul_e_valid | (mul_ready & ~mul_exu_ack & ~mul_kill & ~ismul_e & ~invalid_mul_w);
   dff_s mul_ready_dff(.din(mul_ready_next), .clk(clk), .q(mul_ready), .se(se), .si(), .so());
   
   assign      exu_mul_input_vld = mul_ready;
   
   // If there was a valid request and an ack then start passing down pipe
   assign      mul_done_ack = mul_ready & ~mul_kill & ~ismul_e & mul_exu_ack & ~invalid_mul_w;
   dff_s dff_done_ack2c0(.din(mul_done_ack), .clk(clk), .q(mul_done_c0),
                       .se(se), .si(), .so());
   // need to check here cause this could be w
   assign        mul_done_valid_c0 = mul_done_c0 & ~mul_kill & ~invalid_mul_w & ~ismul_e;
   dff_s dff_done_c02c1(.din(mul_done_valid_c0), .clk(clk), .q(mul_done_c1),
                       .se(se), .si(), .so());
   // need to check here cause this could be w1
   assign        mul_done_valid_c1 = mul_done_c1 & ~mul_kill & ~ismul_e;
   dff_s dff_done_c1c2(.din(mul_done_valid_c1), .clk(clk), .q(mul_done_c2),
                       .se(se), .si(), .so());
   dff_s dff_done_c22c3(.din(mul_done_c2), .clk(clk), .q(mul_done_c3),
                       .se(se), .si(), .so());
   dff_s dff_done_c32c4(.din(mul_done_c3), .clk(clk), .q(ecl_div_mul_wen),
                       .se(se), .si(), .so());

   // Mul result state machine
   assign        go_mul_done = ~mul_done & ecl_div_mul_wen;
   assign        stay_mul_done = mul_done & (~wb_divcntl_ack_g | ecl_div_sel_div);
   assign        next_mul_done = ~reset & (go_mul_done | stay_mul_done);

   assign        mdqctl_divcntl_muldone = mul_done;

   // mul state flop
   dff_s  mulstate_dff(.din(next_mul_done), .clk(clk), .q(mul_done), .se(se), .si(),
                     .so());
   
   /////////////////////////////////////////
   // Pipeline registers for control signals
   /////////////////////////////////////////

   
endmodule // sparc_exu_ecl_mdqctl
// Modified by Princeton University on June 9th, 2015
// ========== Copyright Header Begin ==========================================
// 
// OpenSPARC T1 Processor File: sparc_exu_ecl_wb.v
// Copyright (c) 2006 Sun Microsystems, Inc.  All Rights Reserved.
// DO NOT ALTER OR REMOVE COPYRIGHT NOTICES.
// 
// The above named program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public
// License version 2 as published by the Free Software Foundation.
// 
// The above named program is distributed in the hope that it will be 
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// General Public License for more details.
// 
// You should have received a copy of the GNU General Public
// License along with this work; if not, write to the Free Software
// Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA 02110-1301, USA.
// 
// ========== Copyright Header End ============================================
////////////////////////////////////////////////////////////////////////
/*
//  Module Name: sparc_exu_ecl_wb
//	Description:  Implements the writeback logic for the exu.
//		This includes the control signals for the w1 and w2 input 
//   	muxes as well as keeping track of the wen signal for ALU ops.
*/

module sparc_exu_ecl_wb (/*AUTOARG*/
   // Outputs
   wb_ccr_wrccr_w, ecl_rml_cwp_wen_e, ecl_rml_cansave_wen_w, 
   ecl_rml_canrestore_wen_w, ecl_rml_otherwin_wen_w, 
   ecl_rml_wstate_wen_w, ecl_rml_cleanwin_wen_w, ecl_byp_sel_load_m, 
   ecl_byp_sel_restore_m, ecl_byp_sel_pipe_m, ecl_byp_restore_m, 
   ecl_irf_tid_m, ecl_irf_rd_m, ecl_irf_rd_g, ecl_irf_wen_w2, 
   ecl_irf_tid_g, wb_e, bypass_m, ecl_irf_wen_w, ecl_byp_sel_load_g, 
   ecl_byp_sel_muldiv_g, ecl_byp_sel_restore_g, wb_divcntl_ack_g, 
   wb_ccr_setcc_g, ecl_byp_eclpr_e, exu_ifu_longop_done_g, 
   ecl_div_yreg_wen_w, ecl_div_yreg_wen_g, ecl_div_yreg_shift_g, 
   ecl_div_yreg_wen_l, wb_eccctl_spec_wen_next, bypass_w, 
   wb_byplog_rd_w2, wb_byplog_tid_w2, wb_byplog_wen_w2, 
   wb_byplog_rd_g2, wb_byplog_wen_g2, read_yreg_e, 
   exu_ffu_wsr_inst_e, 
   // Inputs
   clk, se, reset, sehold, ld_rd_g, ld_tid_g, lsu_exu_dfill_vld_g, 
   lsu_exu_ldst_miss_g2, rd_m, tid_m, thr_m, tid_w1, ifu_exu_wen_d, 
   ifu_exu_kill_e, ecl_exu_kill_m, rml_ecl_kill_m, ifu_tlu_flush_w, 
   flush_w1, divcntl_wb_req_g, mdqctl_wb_divrd_g, mdqctl_wb_divthr_g, 
   mdqctl_wb_mulrd_g, mdqctl_wb_multhr_g, mdqctl_wb_divsetcc_g, 
   mdqctl_wb_mulsetcc_g, ecl_div_sel_div, ifu_tlu_wsr_inst_d, 
   ifu_tlu_sraddr_d, rml_ecl_cwp_d, rml_ecl_cansave_d, 
   rml_ecl_canrestore_d, rml_ecl_otherwin_d, rml_ecl_wstate_d, 
   rml_ecl_cleanwin_d, exu_ifu_cc_d, rml_ecl_swap_done, 
   rml_ecl_rmlop_done_e, mdqctl_wb_yreg_wen_g, 
   mdqctl_wb_yreg_shift_g, ecl_byp_sel_ecc_m, eccctl_wb_rd_m, 
   ifu_exu_inst_vld_e, ifu_exu_inst_vld_w, ifu_exu_return_d, 
   restore_e, rml_ecl_fill_e, early_flush_w, ecl_byp_ldxa_g
   ) ;
   input clk;
   input se;
   input reset;
   input sehold;
   input [4:0] ld_rd_g;
   input [1:0] ld_tid_g;
   input       lsu_exu_dfill_vld_g;
   input        lsu_exu_ldst_miss_g2;
   input [4:0]  rd_m;
   input [1:0]  tid_m;
   input [3:0]  thr_m;
   input [1:0]  tid_w1;
   input        ifu_exu_wen_d;
   input        ifu_exu_kill_e;
   input        ecl_exu_kill_m;
   input        rml_ecl_kill_m; // kill from spill or fill trap
   input        ifu_tlu_flush_w;
   input        flush_w1;
   input        divcntl_wb_req_g;
   input [4:0]  mdqctl_wb_divrd_g;
   input [1:0]  mdqctl_wb_divthr_g;
   input [4:0]  mdqctl_wb_mulrd_g;
   input [1:0]  mdqctl_wb_multhr_g;
   input        mdqctl_wb_divsetcc_g;
   input        mdqctl_wb_mulsetcc_g;
   input        ecl_div_sel_div;
   input        ifu_tlu_wsr_inst_d;
   input [6:0] ifu_tlu_sraddr_d;
   input [2:0] rml_ecl_cwp_d;
   input [2:0] rml_ecl_cansave_d;
   input [2:0] rml_ecl_canrestore_d;
   input [2:0] rml_ecl_otherwin_d;
   input [5:0] rml_ecl_wstate_d;
   input [2:0] rml_ecl_cleanwin_d;
   input [7:0] exu_ifu_cc_d;
   input [3:0] rml_ecl_swap_done;
   input       rml_ecl_rmlop_done_e;
   input         mdqctl_wb_yreg_wen_g;
   input         mdqctl_wb_yreg_shift_g;
   input         ecl_byp_sel_ecc_m;
   input  [4:0] eccctl_wb_rd_m;
   input        ifu_exu_inst_vld_e;
   input        ifu_exu_inst_vld_w;
   input        ifu_exu_return_d;
   input  restore_e;
   input  rml_ecl_fill_e;
   input  early_flush_w;
   input        ecl_byp_ldxa_g;
   
   output      wb_ccr_wrccr_w;
   output      ecl_rml_cwp_wen_e;
   output      ecl_rml_cansave_wen_w;
   output      ecl_rml_canrestore_wen_w;
   output      ecl_rml_otherwin_wen_w;
   output      ecl_rml_wstate_wen_w;
   output      ecl_rml_cleanwin_wen_w;
   output      ecl_byp_sel_load_m;
   output      ecl_byp_sel_restore_m;
   output      ecl_byp_sel_pipe_m;
   output      ecl_byp_restore_m;
   output [1:0] ecl_irf_tid_m;
   output [4:0] ecl_irf_rd_m;
   output [4:0] ecl_irf_rd_g;
   output       ecl_irf_wen_w2;
   output [1:0] ecl_irf_tid_g;
   output       wb_e;
   output       bypass_m;
   output       ecl_irf_wen_w;
   output       ecl_byp_sel_load_g;
   output       ecl_byp_sel_muldiv_g;
   output       ecl_byp_sel_restore_g;
   output       wb_divcntl_ack_g;
   output       wb_ccr_setcc_g;
   output [7:0] ecl_byp_eclpr_e;
   output [3:0]  exu_ifu_longop_done_g;
   output [3:0]  ecl_div_yreg_wen_w;
   output [3:0]  ecl_div_yreg_wen_g;
   output [3:0]  ecl_div_yreg_shift_g;
   output [3:0]  ecl_div_yreg_wen_l;// w or w2 or shift
   output        wb_eccctl_spec_wen_next;
   output        bypass_w;
   output [4:0] wb_byplog_rd_w2;
   output [1:0] wb_byplog_tid_w2;
   output       wb_byplog_wen_w2;
   output [4:0] wb_byplog_rd_g2;
   output       wb_byplog_wen_g2;
   output       read_yreg_e;
   output       exu_ffu_wsr_inst_e;

   wire          wb_e;
   wire          wb_m;
   wire          wb_w;
   wire          inst_vld_noflush_wen_m;
   wire          inst_vld_noflush_wen_w;
   wire       ecl_irf_wen_g;
   wire      yreg_wen_w;
   wire      yreg_wen_w1;
   wire      yreg_wen_w1_vld;
   wire      wen_no_inst_vld_m;         // load or restore or ce wen
   wire        wen_no_inst_vld_w;
   wire        wen_w_inst_vld;
   wire        valid_e;
   wire        valid_m;
   wire    valid_w;
   wire    ecl_sel_mul_g;
   wire    ecl_sel_div_g;
   wire [1:0] muldiv_tid;
   wire        setcc_g;        // without wen from divcntl
   wire    wrsr_e;
   wire    wrsr_m;
   wire    wrsr_w;
   wire    [6:0] sraddr_e;
   wire    [6:0] sraddr_m;
   wire    [6:0] sraddr_w;
   wire    sraddr_ccr_w;
   wire    sraddr_y_w;
   wire    sraddr_cwp_e;
   wire    sraddr_cansave_w;
   wire    sraddr_canrestore_w;
   wire    sraddr_cleanwin_w;
   wire    sraddr_otherwin_w;
   wire    sraddr_wstate_w;
   wire    sel_cleanwin_d;
   wire    sel_otherwin_d;
   wire    sel_wstate_d;
   wire    sel_canrestore_d;
   wire    sel_ccr_d;
   wire    sel_cansave_d;
   wire    sel_cwp_d;
   wire    sel_rdpr_mux1_d;
   wire [2:0] rdpr_mux1_out;
   wire [7:0] rdpr_mux2_out;
   wire [3:0] muldiv_done_g;
   wire [3:0]    multhr_dec_g;
   wire [3:0]    divthr_dec_g;
   wire [3:0]    thrdec_w1;
   wire   short_longop_done_e;
   wire   short_longop_done_m;
   wire [3:0] short_longop_done;
   wire       return_e;
   wire   restore_m;
   wire   restore_w;
   wire   vld_restore_e;
   wire   vld_restore_w;
   wire   restore_request;
   wire   restore_wen;
   wire   restore_ready;
   wire   restore_ready_next;
   wire   restore_picked;
   wire [3:0]  restore_done;
   wire [1:0] restore_tid;
   wire [4:0] restore_rd;
   wire [3:0] restore_thr;
   wire [3:0] ecl_longop_done_kill_m;
   wire [3:0] ecl_longop_done_nokill_m;
   wire       dfill_vld_g2;
   wire       ld_g;
   wire       ld_g2;
   wire [1:0] dfill_tid_g2;
   wire [4:0] dfill_rd_g2;
   wire       kill_ld_g2;
   wire [1:0] tid_w2;
   wire [4:0] rd_w2;
   
   ////////////////////////////////////////////
   // Pass along result of load for one cycle
   ////////////////////////////////////////////
   assign     ld_g = lsu_exu_dfill_vld_g | ecl_byp_ldxa_g;
   dff_s dfill_vld_dff (.din(ld_g), .clk(clk), .q(ld_g2),
                      .se(se), .si(), .so());
   assign     kill_ld_g2 = flush_w1 & (dfill_tid_g2[1:0] == tid_w1[1:0]);
   assign     dfill_vld_g2 = ld_g2 & ~kill_ld_g2 & ~lsu_exu_ldst_miss_g2;
   dff_s #(2) dfill_tid_dff(.din(ld_tid_g[1:0]), .clk(clk), .q(dfill_tid_g2[1:0]),
                          .se(se), .si(), .so());
   dff_s #(5) dfill_rd_dff(.din(ld_rd_g[4:0]), .clk(clk), .q(dfill_rd_g2[4:0]),
                         .se(se), .si(), .so());

   ///////////////////////////////////////////
   // Help with bypassing of long latency ops
   ///////////////////////////////////////////
   assign       wb_byplog_rd_w2[4:0] = rd_w2[4:0];
   assign       wb_byplog_wen_w2 = ecl_irf_wen_w2;
   assign       wb_byplog_tid_w2[1:0] = tid_w2[1:0];
   assign       wb_byplog_rd_g2[4:0] = dfill_rd_g2[4:0];
   assign       wb_byplog_wen_g2 = ld_g2;
   
   
   ////////////////////////////////////////////////////////////////
   // G selection logic (picks between LOAD and MUL/DIV outputs)
   ////////////////////////////////////////////////////////////////
   // select signals: priority LOAD, RESTORE, MUL, DIV
   assign      ecl_byp_sel_load_g = (ld_g2 & (wb_m | wrsr_m | ecl_byp_sel_ecc_m));
   assign      ecl_byp_sel_restore_g = restore_request & ((wb_m | wrsr_m | ecl_byp_sel_ecc_m) ^ ld_g2);
   assign      ecl_byp_sel_muldiv_g = ~(ecl_byp_sel_load_g | ecl_byp_sel_restore_g);
   assign      ecl_sel_mul_g = ~ecl_div_sel_div & ecl_byp_sel_muldiv_g;
   assign      ecl_sel_div_g = ecl_div_sel_div & ecl_byp_sel_muldiv_g;
   assign      wb_divcntl_ack_g = ecl_byp_sel_muldiv_g;

   assign      muldiv_tid[1:0] = (ecl_div_sel_div)? mdqctl_wb_divthr_g[1:0]: mdqctl_wb_multhr_g[1:0];
   assign muldiv_done_g[3] = ((wb_divcntl_ack_g & divcntl_wb_req_g) & 
                              muldiv_tid[1] & muldiv_tid[0]); 
   assign muldiv_done_g[2] = ((wb_divcntl_ack_g & divcntl_wb_req_g) &
                              muldiv_tid[1] & ~muldiv_tid[0]); 
   assign muldiv_done_g[1] = ((wb_divcntl_ack_g & divcntl_wb_req_g) &
                              ~muldiv_tid[1] & muldiv_tid[0]); 
   assign muldiv_done_g[0] = ((wb_divcntl_ack_g & divcntl_wb_req_g) &
                              ~muldiv_tid[1] & ~muldiv_tid[0]); 

   assign ecl_irf_wen_g = (sehold)? ecl_irf_wen_w2: 
                                   (ecl_byp_sel_load_g & dfill_vld_g2 |
                                    (ecl_byp_sel_restore_g & restore_wen) |
                                    (ecl_byp_sel_muldiv_g & divcntl_wb_req_g));

   dff_s wen_w2_dff(.din(ecl_irf_wen_g), .clk(clk), .q(ecl_irf_wen_w2),
                  .se(se), .si(), .so());
   mux4ds #(5) rd_g_mux(.dout(ecl_irf_rd_g[4:0]), .in0(dfill_rd_g2[4:0]),
                       .in1(mdqctl_wb_divrd_g[4:0]),
                       .in2(mdqctl_wb_mulrd_g[4:0]),
                        .in3(restore_rd[4:0]),
                       .sel0(ecl_byp_sel_load_g),
                       .sel1(ecl_sel_div_g),
                        .sel2(ecl_sel_mul_g),
                        .sel3(ecl_byp_sel_restore_g));
   mux4ds #(2) thr_g_mux(.dout(ecl_irf_tid_g[1:0]), .in0(dfill_tid_g2[1:0]),
                        .in1(mdqctl_wb_divthr_g[1:0]),
                        .in2(mdqctl_wb_multhr_g[1:0]),
                         .in3(restore_tid[1:0]),
                        .sel0(ecl_byp_sel_load_g),
                        .sel1(ecl_sel_div_g),
                         .sel2(ecl_sel_mul_g),
                         .sel3(ecl_byp_sel_restore_g));
   mux2ds setcc_g_mux(.dout(setcc_g),
                         .in0(mdqctl_wb_mulsetcc_g),
                         .in1(mdqctl_wb_divsetcc_g),
                         .sel0(~ecl_div_sel_div),
                         .sel1(ecl_div_sel_div));
   dff_s #(2) dff_thr_g2w2(.din(ecl_irf_tid_g[1:0]), .clk(clk), .q(tid_w2[1:0]), .se(se),
                      .si(), .so());
   dff_s #(5) dff_rd_g2w2(.din(ecl_irf_rd_g[4:0]), .clk(clk), .q(rd_w2[4:0]), .se(se),
                     .si(), .so());
   // needs wen to setcc
   assign wb_ccr_setcc_g = wb_divcntl_ack_g & divcntl_wb_req_g & setcc_g;
   

   ///////////////////
   // W1 port control
   ///////////////////
   // sehold will turn off in pipe writes and put the hold functionality through
   // the non inst_vld part
   // Mux between load and ALU for rd, thr, and wen
   assign      ecl_byp_sel_load_m = ~(wb_m | wrsr_m | ecl_byp_sel_ecc_m) & ld_g2;
   assign      ecl_byp_sel_pipe_m = (wb_m | wrsr_m) & ~ecl_byp_sel_ecc_m; 
   assign      ecl_byp_sel_restore_m = ~(wb_m | wrsr_m | ld_g2 | ecl_byp_sel_ecc_m);
   assign      wen_no_inst_vld_m = (sehold)? ecl_irf_wen_w: 
                                             ((dfill_vld_g2 & ecl_byp_sel_load_m) |
                                              (ecl_byp_sel_restore_m & restore_wen));
   dff_s dff_lsu_wen_m2w(.din(wen_no_inst_vld_m), .clk(clk), .q(wen_no_inst_vld_w), .se(se), .si(),
                       .so());
   // ecc_wen must be kept separate because it needs to check inst_vld but not flush
   assign      inst_vld_noflush_wen_m = ecl_byp_sel_ecc_m & ~sehold;
   dff_s ecc_wen_m2w(.din(inst_vld_noflush_wen_m), .clk(clk), .q(inst_vld_noflush_wen_w), .se(se), .si(), .so());
   
   assign ecl_irf_tid_m[1:0] = ((ecl_byp_sel_load_m)? dfill_tid_g2[1:0]:
                                (ecl_byp_sel_restore_m)? restore_tid[1:0]:
                                tid_m[1:0]);

   mux4ds #(5) rd_mux(.dout(ecl_irf_rd_m[4:0]), 
                      .in0(rd_m[4:0]), 
                      .in1(dfill_rd_g2[4:0]),
                      .in2(eccctl_wb_rd_m[4:0]),
                      .in3(restore_rd[4:0]),
                      .sel0(ecl_byp_sel_pipe_m), 
                      .sel1(ecl_byp_sel_load_m),
                      .sel2(ecl_byp_sel_ecc_m),
                      .sel3(ecl_byp_sel_restore_m));
   assign wen_w_inst_vld = valid_w | inst_vld_noflush_wen_w;
   assign ecl_irf_wen_w = ifu_exu_inst_vld_w & wen_w_inst_vld | wen_no_inst_vld_w;

   // bypass valid logic and flops
   dff_s dff_wb_d2e(.din(ifu_exu_wen_d), .clk(clk), .q(wb_e), .se(se),
                  .si(), .so());
   dff_s dff_wb_e2m(.din(valid_e), .clk(clk), .q(wb_m), .se(se),
                  .si(), .so());
   dffr_s dff_wb_m2w(.din(valid_m), .clk(clk), .q(wb_w), .se(se),
                  .si(), .so(), .rst(reset));
   assign  valid_e = wb_e & ~ifu_exu_kill_e & ~restore_e & ~wrsr_e;// restore doesn't finish on time
   assign  bypass_m = wb_m;// bypass doesn't need to check for traps or sehold
   assign  valid_m = bypass_m & ~rml_ecl_kill_m & ~sehold;// sehold turns off writes from this path
   assign  valid_w = (wb_w & ~early_flush_w & ~ifu_tlu_flush_w);// check inst_vld later
   // don't check flush for bypass
   assign  bypass_w = wb_w | inst_vld_noflush_wen_w | wen_no_inst_vld_w;

   // speculative wen for ecc injection
   assign  wb_eccctl_spec_wen_next = valid_m | dfill_vld_g2 | restore_request |  divcntl_wb_req_g;

   ///////////////////////////////////////////////////////
   // Priviledged register read and write flops and logic
   ///////////////////////////////////////////////////////
/* -----\/----- EXCLUDED -----\/-----
   Decoded sraddr
   sraddr[5] = 1-priv, 0-state
   Y -   0
   CCR - 2
   CWP - 9
   CANSAVE - a
   CARESTORE - b
   CLEANWIN - c
   OTHERWIN - d
   WSTATE - e 
   GSR - 0x13
 -----/\----- EXCLUDED -----/\----- */
   assign  ecl_rml_cwp_wen_e = sraddr_cwp_e & wrsr_e;
   assign  sraddr_cwp_e = ~sraddr_e[6] & sraddr_e[5] & ~sraddr_e[4] & sraddr_e[3] & ~sraddr_e[2] &
           ~sraddr_e[1] & sraddr_e[0];

   assign  sraddr_y_w = ~sraddr_w[6] & ~sraddr_w[5] & ~sraddr_w[4] & ~sraddr_w[3] & ~sraddr_w[2] & 
           ~sraddr_w[1] & ~sraddr_w[0];
   assign  sraddr_ccr_w = ~sraddr_w[6] & ~sraddr_w[5] & ~sraddr_w[4] & ~sraddr_w[3] & ~sraddr_w[2] &
           sraddr_w[1] & ~sraddr_w[0];
   assign  sraddr_cansave_w = ~sraddr_w[6] & sraddr_w[5] & ~sraddr_w[4] & sraddr_w[3] & ~sraddr_w[2] &
           sraddr_w[1] & ~sraddr_w[0];
   assign  sraddr_canrestore_w = ~sraddr_w[6] & sraddr_w[5] & ~sraddr_w[4] & sraddr_w[3] & ~sraddr_w[2] &
           sraddr_w[1] & sraddr_w[0];
   assign  sraddr_cleanwin_w = ~sraddr_w[6] & sraddr_w[5] & ~sraddr_w[4] & sraddr_w[3] & sraddr_w[2] &
           ~sraddr_w[1] & ~sraddr_w[0];
   assign  sraddr_otherwin_w = ~sraddr_w[6] & sraddr_w[5] & ~sraddr_w[4] & sraddr_w[3] & sraddr_w[2] &
           ~sraddr_w[1] & sraddr_w[0];
   assign  sraddr_wstate_w = ~sraddr_w[6] & sraddr_w[5] & ~sraddr_w[4] & sraddr_w[3] & sraddr_w[2] &
           sraddr_w[1] & ~sraddr_w[0];

   // yreg writes cycle after w and checks flush in that cycle
   assign  yreg_wen_w = sraddr_y_w & wrsr_w & ifu_exu_inst_vld_w;
   assign  yreg_wen_w1_vld = yreg_wen_w1 & ~flush_w1;

   // controls for all other writes (and flush checks) are in their respective blocks
   assign  wb_ccr_wrccr_w = sraddr_ccr_w & wrsr_w;
   assign  ecl_rml_cansave_wen_w = sraddr_cansave_w & wrsr_w;
   assign  ecl_rml_canrestore_wen_w = sraddr_canrestore_w & wrsr_w;
   assign  ecl_rml_cleanwin_wen_w = sraddr_cleanwin_w & wrsr_w;
   assign  ecl_rml_otherwin_wen_w = sraddr_otherwin_w & wrsr_w;
   assign  ecl_rml_wstate_wen_w = sraddr_wstate_w & wrsr_w;
   

   dff_s dff_wrsr_d2e(.din(ifu_tlu_wsr_inst_d), .clk(clk), .q(wrsr_e), .se(se),
                   .si(), .so());
   assign  exu_ffu_wsr_inst_e = wrsr_e;
   dff_s dff_wrsr_e2m(.din(wrsr_e), .clk(clk), .q(wrsr_m), .se(se),
                   .si(), .so());
   dff_s dff_wrsr_m2w(.din(wrsr_m), .clk(clk), .q(wrsr_w), .se(se),
                   .si(), .so());
   dff_s #(7) dff_sraddr_d2e(.din(ifu_tlu_sraddr_d[6:0]), .clk(clk), .q(sraddr_e[6:0]), .se(se),
                       .si(), .so());
   dff_s #(7) dff_sraddr_e2m(.din(sraddr_e[6:0]), .clk(clk), .q(sraddr_m[6:0]), .se(se),
                       .si(), .so());
   dff_s #(7) dff_sraddr_m2w(.din(sraddr_m[6:0]), .clk(clk), .q(sraddr_w[6:0]), .se(se),
                       .si(), .so());
   dff_s dff_yreg_wen_w2w1(.din(yreg_wen_w), .clk(clk), .q(yreg_wen_w1), .se(se), .si(), .so());
   
   // Logic for rdpr/rdsr
   // This mux takes advantage of the fact that these 4 encodings don't overlap
   assign sel_cleanwin_d = ~ifu_tlu_sraddr_d[1] & ~ifu_tlu_sraddr_d[0];
   assign sel_otherwin_d = ~ifu_tlu_sraddr_d[1] & ifu_tlu_sraddr_d[0];
   assign sel_cansave_d = ifu_tlu_sraddr_d[1] & ~ifu_tlu_sraddr_d[0];
   assign sel_canrestore_d = ifu_tlu_sraddr_d[1] & ifu_tlu_sraddr_d[0];
   mux4ds #(3) rdpr_mux1(.dout(rdpr_mux1_out[2:0]),
                       .in0(rml_ecl_canrestore_d[2:0]),
                       .in1(rml_ecl_cleanwin_d[2:0]),
                       .in2(rml_ecl_cansave_d[2:0]),
                       .in3(rml_ecl_otherwin_d[2:0]),
                       .sel0(sel_canrestore_d),
                       .sel1(sel_cleanwin_d),
                       .sel2(sel_cansave_d),
                       .sel3(sel_otherwin_d));
   assign sel_ccr_d = ~ifu_tlu_sraddr_d[3];
   assign sel_cwp_d = ifu_tlu_sraddr_d[3] & ~ifu_tlu_sraddr_d[2] & ~ifu_tlu_sraddr_d[1] & ifu_tlu_sraddr_d[0];
   assign sel_wstate_d = ifu_tlu_sraddr_d[3] & ifu_tlu_sraddr_d[2] & ifu_tlu_sraddr_d[1] & ~ifu_tlu_sraddr_d[0];
   assign sel_rdpr_mux1_d = ~(sel_ccr_d | sel_cwp_d | sel_wstate_d);
   mux4ds #(8) rdpr_mux2(.dout(rdpr_mux2_out[7:0]),
                       .in0(exu_ifu_cc_d[7:0]),
                       .in1({5'b0, rml_ecl_cwp_d[2:0]}),
                       .in2({2'b0, rml_ecl_wstate_d[5:0]}),
                       .in3({5'b0, rdpr_mux1_out[2:0]}),
                       .sel0(sel_ccr_d),
                       .sel1(sel_cwp_d),
                       .sel2(sel_wstate_d),
                       .sel3(sel_rdpr_mux1_d));

   assign read_yreg_e = ~(sraddr_e[3] | sraddr_e[1]);
   dff_s #(8) rdpr_dff(.din(rdpr_mux2_out[7:0]), .clk(clk), .q(ecl_byp_eclpr_e[7:0]),
                   .se(se), .si(), .so());


   ///////////////////////////////
   // YREG write enable logic
   ///////////////////////////////
   // decode thr_g for mux select
   assign multhr_dec_g[0] = ~mdqctl_wb_multhr_g[1] & ~mdqctl_wb_multhr_g[0];
   assign multhr_dec_g[1] = ~mdqctl_wb_multhr_g[1] & mdqctl_wb_multhr_g[0];
   assign multhr_dec_g[2] = mdqctl_wb_multhr_g[1] & ~mdqctl_wb_multhr_g[0];
   assign multhr_dec_g[3] = mdqctl_wb_multhr_g[1] & mdqctl_wb_multhr_g[0];

   assign divthr_dec_g[0] = ~mdqctl_wb_divthr_g[1] & ~mdqctl_wb_divthr_g[0];
   assign divthr_dec_g[1] = ~mdqctl_wb_divthr_g[1] & mdqctl_wb_divthr_g[0];
   assign divthr_dec_g[2] = mdqctl_wb_divthr_g[1] & ~mdqctl_wb_divthr_g[0];
   assign divthr_dec_g[3] = mdqctl_wb_divthr_g[1] & mdqctl_wb_divthr_g[0];

   assign thrdec_w1[0] = ~tid_w1[1] & ~tid_w1[0];
   assign thrdec_w1[1] = ~tid_w1[1] & tid_w1[0];
   assign thrdec_w1[2] = tid_w1[1] & ~tid_w1[0];
   assign thrdec_w1[3] = tid_w1[1] & tid_w1[0];

   // enable input for each thread
   
   assign ecl_div_yreg_shift_g[0] = divthr_dec_g[0] & mdqctl_wb_yreg_shift_g;
   assign ecl_div_yreg_wen_w[0] = (thrdec_w1[0] & yreg_wen_w1_vld &
                                   ~ecl_div_yreg_shift_g[0] &
                                   ~ecl_div_yreg_wen_g[0]);
   assign ecl_div_yreg_wen_g[0] = (multhr_dec_g[0] & mdqctl_wb_yreg_wen_g & 
                                   ~ecl_div_yreg_shift_g[0]);
   assign ecl_div_yreg_wen_l[0] = ~(ecl_div_yreg_wen_w[0] | ecl_div_yreg_wen_g[0]
                                    | ecl_div_yreg_shift_g[0]);
   assign ecl_div_yreg_shift_g[1] = divthr_dec_g[1] & mdqctl_wb_yreg_shift_g;
   assign ecl_div_yreg_wen_w[1] = (thrdec_w1[1] & yreg_wen_w1_vld &
                                   ~ecl_div_yreg_shift_g[1] &
                                   ~ecl_div_yreg_wen_g[1]);
   assign ecl_div_yreg_wen_g[1] = (multhr_dec_g[1] & mdqctl_wb_yreg_wen_g & 
                                   ~ecl_div_yreg_shift_g[1]);
   assign ecl_div_yreg_wen_l[1] = ~(ecl_div_yreg_wen_w[1] | ecl_div_yreg_wen_g[1]
                                    | ecl_div_yreg_shift_g[1]);
   assign ecl_div_yreg_shift_g[2] = divthr_dec_g[2] & mdqctl_wb_yreg_shift_g;
   assign ecl_div_yreg_wen_w[2] = (thrdec_w1[2] & yreg_wen_w1_vld &
                                   ~ecl_div_yreg_shift_g[2] &
                                   ~ecl_div_yreg_wen_g[2]);
   assign ecl_div_yreg_wen_g[2] = (multhr_dec_g[2] & mdqctl_wb_yreg_wen_g & 
                                   ~ecl_div_yreg_shift_g[2]);
   assign ecl_div_yreg_wen_l[2] = ~(ecl_div_yreg_wen_w[2] | ecl_div_yreg_wen_g[2]
                                    | ecl_div_yreg_shift_g[2]);
   assign ecl_div_yreg_shift_g[3] = divthr_dec_g[3] & mdqctl_wb_yreg_shift_g;
   assign ecl_div_yreg_wen_w[3] = (thrdec_w1[3] & yreg_wen_w1_vld &
                                   ~ecl_div_yreg_shift_g[3] &
                                   ~ecl_div_yreg_wen_g[3]);
   assign ecl_div_yreg_wen_g[3] = (multhr_dec_g[3] & mdqctl_wb_yreg_wen_g & 
                                   ~ecl_div_yreg_shift_g[3]);
   assign ecl_div_yreg_wen_l[3] = ~(ecl_div_yreg_wen_w[3] | ecl_div_yreg_wen_g[3]
                                    | ecl_div_yreg_shift_g[3]);

   //////////////////////////////////////////////////////////
   // Completion logic for restore
   //////////////////////////////////////////////////////////

   // only worry about restores.  Returns are automatically switched back in
   assign ecl_byp_restore_m = restore_m;
   assign vld_restore_e = restore_e & wb_e & ~return_e & ~rml_ecl_fill_e & ifu_exu_inst_vld_e;
   assign vld_restore_w = (restore_w & ~ifu_tlu_flush_w & ~early_flush_w 
                           & ifu_exu_inst_vld_w & ~reset);

   assign restore_request = restore_w | restore_ready;
   assign restore_wen = vld_restore_w | restore_ready;
   assign restore_picked = ecl_byp_sel_restore_m | ecl_byp_sel_restore_g;
   assign restore_done[3:0] = restore_thr[3:0] & {4{restore_picked & restore_request}};
   // restore request waits for kills in the w stage.  they
   // won't start until after the flop
   assign restore_ready_next = (vld_restore_w  | restore_ready) & ~restore_picked;

   dffe_s #(2) restore_tid_dff(.din(tid_m[1:0]), .clk(clk), .q(restore_tid[1:0]),
                             .se(se), .si(), .so(), .en(restore_m));
   dffe_s #(5) restore_rd_dff(.din(rd_m[4:0]), .clk(clk), .q(restore_rd[4:0]),
                            .se(se), .si(), .so(), .en(restore_m));
   dff_s return_d2e(.din(ifu_exu_return_d), .clk(clk), .q(return_e),
                   .se(se), .si(), .so());
   dff_s restore_e2m(.din(vld_restore_e), .clk(clk), .q(restore_m),
                   .se(se), .si(), .so());
   dff_s restore_m2w(.din(restore_m), .clk(clk), .q(restore_w),
                   .se(se), .si(), .so());
   dff_s restore_ready_dff(.din(restore_ready_next), .q(restore_ready),
                         .clk(clk), .se(se), .so(), .si());

   //////////////////////////////////////////////////////////
   // Completion logic for non integer-pipeline operations
   //////////////////////////////////////////////////////////
   // short_longops must check inst_vld_e to protect against invalid completion signal
   assign short_longop_done_e = (rml_ecl_rmlop_done_e | (restore_e & ~wb_e & ~return_e)) & 
                                  ifu_exu_inst_vld_e & ~ifu_exu_kill_e;
   dff_s longop_done_e2m (.din(short_longop_done_e), .clk(clk), .q(short_longop_done_m), .se(se), .si(), .so());
   assign short_longop_done[3:0] = thr_m[3:0] & {4{short_longop_done_m}};
   
   assign ecl_longop_done_nokill_m[3:0] = (muldiv_done_g[3:0] | restore_done[3:0] | short_longop_done[3:0] | 
                                           rml_ecl_swap_done[3:0]);
   assign ecl_longop_done_kill_m[3:0] = (muldiv_done_g[3:0] | restore_done[3:0] | rml_ecl_swap_done[3:0]);
   assign exu_ifu_longop_done_g[3:0] = (ecl_exu_kill_m)? ecl_longop_done_kill_m[3:0]: ecl_longop_done_nokill_m[3:0];
   

   // decode tid
   assign restore_thr[3] = restore_tid[1] & restore_tid[0];
   assign restore_thr[2] = restore_tid[1] & ~restore_tid[0];
   assign restore_thr[1] = ~restore_tid[1] & restore_tid[0];
   assign restore_thr[0] = ~restore_tid[1] & ~restore_tid[0];

endmodule // sparc_exu_ecl_wb
// Modified by Princeton University on June 9th, 2015
// ========== Copyright Header Begin ==========================================
// 
// OpenSPARC T1 Processor File: sparc_exu_eclbyplog.v
// Copyright (c) 2006 Sun Microsystems, Inc.  All Rights Reserved.
// DO NOT ALTER OR REMOVE COPYRIGHT NOTICES.
// 
// The above named program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public
// License version 2 as published by the Free Software Foundation.
// 
// The above named program is distributed in the hope that it will be 
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// General Public License for more details.
// 
// You should have received a copy of the GNU General Public
// License along with this work; if not, write to the Free Software
// Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA 02110-1301, USA.
// 
// ========== Copyright Header End ============================================
////////////////////////////////////////////////////////////////////////
/*
//  Module Name: sparc_exu_eclbyplog
//	Description: This block implements the bypass logic for a single
//	operand.  It takes the destination registers of all
//	four forwarding sources and the rs.  It also has the
//	thread for the instruction in each stage and whether
//	the instruction writes to the register file.  It won't
//	bypass if rs =0.
*/
module sparc_exu_eclbyplog (/*AUTOARG*/
   // Outputs
   rs_sel_mux1_m, rs_sel_mux1_w, rs_sel_mux1_w2, rs_sel_mux1_other, 
   rs_sel_mux2_usemux1, rs_sel_mux2_rf, rs_sel_mux2_e, 
   rs_sel_mux2_ld, rs_sel_longmux_g2, rs_sel_longmux_w2, 
   rs_sel_longmux_ldxa, 
   // Inputs
   sehold, use_other, rs, rd_e, rd_m, ecl_irf_rd_w, ld_rd_g, 
   wb_byplog_rd_w2, wb_byplog_rd_g2, tid_d, thr_match_de, 
   thr_match_dm, ecl_irf_tid_w, ld_thr_match_dg, wb_byplog_tid_w2, 
   ld_thr_match_dg2, ifu_exu_kill_e, wb_e, bypass_m, 
   lsu_exu_dfill_vld_g, bypass_w, wb_byplog_wen_w2, wb_byplog_wen_g2, 
   ecl_byp_ldxa_g
   ) ;
   input sehold;
   input use_other;
   input [4:0] rs;              // source register
   input [4:0] rd_e;            // destination regs for all stages
   input [4:0] rd_m;
   input [4:0] ecl_irf_rd_w;
   input [4:0] ld_rd_g;
   input [4:0] wb_byplog_rd_w2;
   input [4:0] wb_byplog_rd_g2;
   input [1:0] tid_d;
   input       thr_match_de;
   input       thr_match_dm;
   input [1:0] ecl_irf_tid_w;
   input       ld_thr_match_dg;
   input [1:0] wb_byplog_tid_w2;
   input       ld_thr_match_dg2;
   input       ifu_exu_kill_e;
   input       wb_e;            // whether each stage writes to reg
   input       bypass_m;            // file
   input       lsu_exu_dfill_vld_g;
   input       bypass_w;
   input       wb_byplog_wen_w2;
   input       wb_byplog_wen_g2;
   input       ecl_byp_ldxa_g;

   output      rs_sel_mux1_m;
   output      rs_sel_mux1_w;
   output      rs_sel_mux1_w2;
   output      rs_sel_mux1_other;
   output      rs_sel_mux2_usemux1;
   output      rs_sel_mux2_rf;
   output      rs_sel_mux2_e;
   output      rs_sel_mux2_ld;
   output      rs_sel_longmux_g2;
   output      rs_sel_longmux_w2;
   output      rs_sel_longmux_ldxa;

   wire        use_e, use_m, use_w, use_w2, use_rf, use_ld, use_ldxa;
   wire         match_e, match_m, match_w, match_w2, match_ld; // outputs of comparison
   wire         match_g2;
   wire         bypass;         // boolean that allows bypassing
   wire         rs_is_nonzero;

   // Don't bypass if rs == 0 or we are supposed to use other
   assign       rs_is_nonzero = rs[0]|rs[1]|rs[2]|rs[3]|rs[4];
   assign       bypass = rs_is_nonzero & ~use_other & ~sehold;

   // Normal pipe priority: E, M, W, RF
   // Ld priority: LD, RF
   // W2 priority: E, M, W2, RF
   assign       use_e = match_e & wb_e & ~ifu_exu_kill_e;
   assign       use_m = match_m & bypass_m & ~use_e;
   assign       use_w = match_w & bypass_w & ~use_m & ~use_e;
   assign       use_ld = match_ld & lsu_exu_dfill_vld_g & ~ecl_byp_ldxa_g;
   assign       use_ldxa = match_ld & ecl_byp_ldxa_g;
   assign       use_w2 = (match_w2 & wb_byplog_wen_w2 | match_g2 & wb_byplog_wen_g2) & ~use_e & ~use_m;
   assign       use_rf = ~use_w2 & ~use_w & ~use_m & ~use_e & ~use_ld & ~use_ldxa;

   // mux1[M, W, W2, OTHER(optional)]
   // mux2[mux1, RF, E, LD]
   assign       rs_sel_mux2_e = (use_e & bypass);
   assign       rs_sel_mux2_rf = ((use_rf | ~bypass) & ~(use_other & ~sehold));
   assign       rs_sel_mux2_ld = (use_ld & ~use_e & ~use_w & ~use_m & ~use_w2 & bypass);
   assign       rs_sel_mux2_usemux1 = (use_other & ~sehold) | (~rs_sel_mux1_other & ~use_e);
   assign rs_sel_mux1_other = ~((use_m | use_w | use_w2 | use_ldxa) & bypass);
   assign rs_sel_mux1_w2 = ((use_w2 | use_ldxa) & bypass);
   assign rs_sel_mux1_w = (use_w & ~use_w2 & ~use_ldxa & bypass);
   assign rs_sel_mux1_m = (use_m & ~use_w2 & ~use_ldxa & bypass);

   assign rs_sel_longmux_ldxa = use_ldxa;
   assign rs_sel_longmux_g2 = match_g2 & wb_byplog_wen_g2 & ~use_ldxa;
   assign rs_sel_longmux_w2 = ~use_ldxa & ~(match_g2 & wb_byplog_wen_g2);
   
   // Comparisons
   assign match_e = thr_match_de & (rs[4:0] == rd_e[4:0]);
//   sparc_exu_eclcomp7 e_comp7(.out(match_e), .in1({tid_d[1:0],rs[4:0]}),
//                              .in2({ecl_rml_tid_e[1:0],rd_e[4:0]}));
   assign match_m = thr_match_dm & (rs[4:0] == rd_m[4:0]);
//   sparc_exu_eclcomp7 m_comp7(.out(match_m), .in1({tid_d[1:0],rs[4:0]}),
//                              .in2({tid_m[1:0],rd_m[4:0]}));
   sparc_exu_eclcomp7 w_comp7(.out(match_w), .in1({tid_d[1:0],rs[4:0]}),
                              .in2({ecl_irf_tid_w[1:0],ecl_irf_rd_w[4:0]}));
   sparc_exu_eclcomp7 w2_comp7(.out(match_w2), .in1({tid_d[1:0],rs[4:0]}),
                               .in2({wb_byplog_tid_w2[1:0],wb_byplog_rd_w2[4:0]}));
   assign match_ld = ld_thr_match_dg & (rs[4:0] == ld_rd_g[4:0]);
   assign match_g2 = ld_thr_match_dg2 & (rs[4:0] == wb_byplog_rd_g2[4:0]);
/* -----\/----- EXCLUDED -----\/-----
   sparc_exu_eclcomp7 ld_comp7(.out(match_ld), .in1({tid_d[1:0],rs[4:0]}),
                               .in2({ld_tid_g[1:0],ld_rd_g[4:0]}));
   sparc_exu_eclcomp7 g2_comp7(.out(match_g2), .in1({tid_d[1:0],rs[4:0]}),
                               .in2({wb_byplog_tid_g2[1:0],wb_byplog_rd_g2[4:0]}));
 -----/\----- EXCLUDED -----/\----- */

   
endmodule // sparc_exu_eclbyplog
// Modified by Princeton University on June 9th, 2015
// ========== Copyright Header Begin ==========================================
// 
// OpenSPARC T1 Processor File: sparc_exu_eclbyplog_rs1.v
// Copyright (c) 2006 Sun Microsystems, Inc.  All Rights Reserved.
// DO NOT ALTER OR REMOVE COPYRIGHT NOTICES.
// 
// The above named program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public
// License version 2 as published by the Free Software Foundation.
// 
// The above named program is distributed in the hope that it will be 
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// General Public License for more details.
// 
// You should have received a copy of the GNU General Public
// License along with this work; if not, write to the Free Software
// Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA 02110-1301, USA.
// 
// ========== Copyright Header End ============================================
////////////////////////////////////////////////////////////////////////
/*
//  Module Name: sparc_exu_eclbyplog_rs1
//	Description: This block implements the bypass logic for a single
//	operand.  It takes the destination registers of all
//	four forwarding sources and the rs.  It also has the
//	thread for the instruction in each stage and whether
//	the instruction writes to the register file.  It won't
//	bypass if bypass_enable is low or rs =0.  This is for the
//	special case of rs1 which has two bypass sets.  One uses
//	the pc as an input (other) and one does not.  
*/
module sparc_exu_eclbyplog_rs1 (/*AUTOARG*/
   // Outputs
   rs_sel_mux1_m, rs_sel_mux1_w, rs_sel_mux1_w2, rs_sel_mux1_other, 
   rs_sel_mux2_usemux1, rs_sel_mux2_rf, rs_sel_mux2_e, 
   rs_sel_mux2_ld, rs_sel_longmux_g2, rs_sel_longmux_w2, 
   rs_sel_longmux_ldxa, ecl_byp_rcc_mux1_sel_m, 
   ecl_byp_rcc_mux1_sel_w, ecl_byp_rcc_mux1_sel_w2, 
   ecl_byp_rcc_mux1_sel_other, ecl_byp_rcc_mux2_sel_usemux1, 
   ecl_byp_rcc_mux2_sel_rf, ecl_byp_rcc_mux2_sel_e, 
   ecl_byp_rcc_mux2_sel_ld, 
   // Inputs
   sehold, use_other, rs, rd_e, rd_m, ecl_irf_rd_w, ld_rd_g, 
   wb_byplog_rd_w2, wb_byplog_rd_g2, tid_d, thr_match_de, 
   thr_match_dm, ecl_irf_tid_w, ld_thr_match_dg, wb_byplog_tid_w2, 
   ld_thr_match_dg2, ifu_exu_kill_e, wb_e, bypass_m, 
   lsu_exu_dfill_vld_g, bypass_w, wb_byplog_wen_w2, wb_byplog_wen_g2, 
   ecl_byp_ldxa_g
   ) ;
   input sehold;
   input use_other;
   input [4:0] rs;              // source register
   input [4:0] rd_e;            // destination regs for all stages
   input [4:0] rd_m;
   input [4:0] ecl_irf_rd_w;
   input [4:0] ld_rd_g;
   input [4:0] wb_byplog_rd_w2;
   input [4:0] wb_byplog_rd_g2;
   input [1:0] tid_d;
   input       thr_match_de;
   input       thr_match_dm;
   input [1:0] ecl_irf_tid_w;
   input       ld_thr_match_dg;
   input [1:0] wb_byplog_tid_w2;
   input       ld_thr_match_dg2;
   input       ifu_exu_kill_e;
   input       wb_e;            // whether each stage writes to reg
   input       bypass_m;            // file
   input       lsu_exu_dfill_vld_g;
   input       bypass_w;
   input       wb_byplog_wen_w2;
   input       wb_byplog_wen_g2;
   input       ecl_byp_ldxa_g;

   output      rs_sel_mux1_m;
   output      rs_sel_mux1_w;
   output      rs_sel_mux1_w2;
   output      rs_sel_mux1_other;
   output      rs_sel_mux2_usemux1;
   output      rs_sel_mux2_rf;
   output      rs_sel_mux2_e;
   output      rs_sel_mux2_ld;
   output      rs_sel_longmux_g2;
   output      rs_sel_longmux_w2;
   output      rs_sel_longmux_ldxa;
   output      ecl_byp_rcc_mux1_sel_m;
   output      ecl_byp_rcc_mux1_sel_w;
   output      ecl_byp_rcc_mux1_sel_w2;
   output      ecl_byp_rcc_mux1_sel_other;
   output      ecl_byp_rcc_mux2_sel_usemux1;
   output      ecl_byp_rcc_mux2_sel_rf;
   output      ecl_byp_rcc_mux2_sel_e;
   output      ecl_byp_rcc_mux2_sel_ld;
   

   wire         use_e, use_m, use_w, use_w2, use_rf, use_ld, use_ldxa;
   wire         match_e, match_m, match_w, match_w2, match_ld; // outputs of comparison
   wire         match_g2;
   wire         bypass;         // boolean that allows bypassing
   wire         rs_is_nonzero;
   wire   rcc_bypass;

   // Don't bypass if rs == 0 or we are supposed to use other
   assign       rs_is_nonzero = rs[0]|rs[1]|rs[2]|rs[3]|rs[4];
   assign       bypass = rs_is_nonzero & ~use_other & ~sehold;

   // Normal pipe priority: E, M, W, RF
   // Ld priority: LD, RF
   // W2 priority: W2, RF
   assign       use_e = match_e & wb_e & ~ifu_exu_kill_e;
   assign       use_m = match_m & bypass_m & ~use_e;
   assign       use_w = match_w & bypass_w & ~use_m & ~use_e;
   assign       use_ld = match_ld & lsu_exu_dfill_vld_g & ~ecl_byp_ldxa_g;
   assign       use_ldxa = match_ld & ecl_byp_ldxa_g;
   assign       use_w2 = (match_w2 & wb_byplog_wen_w2 | match_g2 & wb_byplog_wen_g2) & ~use_e & ~use_m;
   assign       use_rf = ~use_w2 & ~use_w & ~use_m & ~use_e & ~use_ld & ~use_ldxa;

   // mux1[M, W, W2, OTHER(optional)]
   // mux2[mux1, RF, E, LD]
   assign       rs_sel_mux2_e = (use_e & bypass);
   assign       rs_sel_mux2_rf = ((use_rf | ~bypass) & ~use_other);
   assign       rs_sel_mux2_ld = (use_ld & ~use_e  & ~use_w & ~use_m & ~use_w2 & bypass);
   assign       rs_sel_mux2_usemux1 = (use_other & ~sehold) | (~rs_sel_mux1_other & ~use_e);
   assign rs_sel_mux1_other = ~((use_m | use_w | use_w2 | use_ldxa) & bypass);
   assign rs_sel_mux1_w2 = ((use_ldxa | use_w2) & bypass);
   assign rs_sel_mux1_w = (use_w & ~use_w2 & ~use_ldxa & bypass);
   assign rs_sel_mux1_m = (use_m & ~use_w2 & ~use_ldxa & bypass);

   assign rs_sel_longmux_ldxa = use_ldxa;
   assign rs_sel_longmux_g2 = match_g2 & wb_byplog_wen_g2 & ~use_ldxa;
   assign rs_sel_longmux_w2 = ~use_ldxa & ~(match_g2 & wb_byplog_wen_g2);
   
   // Bypassing for cc generation (don't use other input)
   assign rcc_bypass = rs_is_nonzero;
   assign ecl_byp_rcc_mux2_sel_e = use_e & rcc_bypass;
   assign ecl_byp_rcc_mux2_sel_rf = use_rf | ~rcc_bypass;
   assign ecl_byp_rcc_mux2_sel_ld = use_ld & ~use_e  & ~use_w & ~use_m & ~use_w2 & rcc_bypass;
   assign ecl_byp_rcc_mux2_sel_usemux1 = (use_m | use_w | use_w2 | use_ldxa) & rcc_bypass & ~use_e;
   assign ecl_byp_rcc_mux1_sel_other = ~(use_m | use_w | use_w2 | use_ldxa);
   assign ecl_byp_rcc_mux1_sel_w2 = use_w2 | use_ldxa;
   assign ecl_byp_rcc_mux1_sel_w = use_w & ~use_w2 & ~use_ldxa;
   assign ecl_byp_rcc_mux1_sel_m = use_m & ~use_w2 & ~use_ldxa;
   
   // Comparisons
   assign match_e = thr_match_de & (rs[4:0] == rd_e[4:0]);
//   sparc_exu_eclcomp7 e_comp7(.out(match_e), .in1({tid_d[1:0],rs[4:0]}),
//                              .in2({ecl_rml_tid_e[1:0],rd_e[4:0]}));
   assign match_m = thr_match_dm & (rs[4:0] == rd_m[4:0]);
//   sparc_exu_eclcomp7 m_comp7(.out(match_m), .in1({tid_d[1:0],rs[4:0]}),
//                              .in2({tid_m[1:0],rd_m[4:0]}));
   sparc_exu_eclcomp7 w_comp7(.out(match_w), .in1({tid_d[1:0],rs[4:0]}),
                              .in2({ecl_irf_tid_w[1:0],ecl_irf_rd_w[4:0]}));
   sparc_exu_eclcomp7 w2_comp7(.out(match_w2), .in1({tid_d[1:0],rs[4:0]}),
                               .in2({wb_byplog_tid_w2[1:0],wb_byplog_rd_w2[4:0]}));
   assign match_ld = ld_thr_match_dg & (rs[4:0] == ld_rd_g[4:0]);
   assign match_g2 = ld_thr_match_dg2 & (rs[4:0] == wb_byplog_rd_g2[4:0]);
/* -----\/----- EXCLUDED -----\/-----
   sparc_exu_eclcomp7 ld_comp7(.out(match_ld), .in1({tid_d[1:0],rs[4:0]}),
                               .in2({ld_tid_g[1:0],ld_rd_g[4:0]}));
   sparc_exu_eclcomp7 g2_comp7(.out(match_g2), .in1({tid_d[1:0],rs[4:0]}),
                               .in2({wb_byplog_tid_g2[1:0],wb_byplog_rd_g2[4:0]}));
 -----/\----- EXCLUDED -----/\----- */

   
endmodule // sparc_exu_eclbyplog
// Modified by Princeton University on June 9th, 2015
// ========== Copyright Header Begin ==========================================
// 
// OpenSPARC T1 Processor File: sparc_exu_eclccr.v
// Copyright (c) 2006 Sun Microsystems, Inc.  All Rights Reserved.
// DO NOT ALTER OR REMOVE COPYRIGHT NOTICES.
// 
// The above named program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public
// License version 2 as published by the Free Software Foundation.
// 
// The above named program is distributed in the hope that it will be 
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// General Public License for more details.
// 
// You should have received a copy of the GNU General Public
// License along with this work; if not, write to the Free Software
// Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA 02110-1301, USA.
// 
// ========== Copyright Header End ============================================
////////////////////////////////////////////////////////////////////////
/*
//  Module Name: sparc_exu_eclccr
//	Description: 4 bit condition code registers with forwarding.  Takes
//	the e_stage result and writes on the w stage.
*/
// Modified by Princeton University on June 9th, 2015
/*
* ========== Copyright Header Begin ==========================================
* 
* OpenSPARC T1 Processor File: sys.h
* Copyright (c) 2006 Sun Microsystems, Inc.  All Rights Reserved.
* DO NOT ALTER OR REMOVE COPYRIGHT NOTICES.
* 
* The above named program is free software; you can redistribute it and/or
* modify it under the terms of the GNU General Public
* License version 2 as published by the Free Software Foundation.
* 
* The above named program is distributed in the hope that it will be 
* useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
* General Public License for more details.
* 
* You should have received a copy of the GNU General Public
* License along with this work; if not, write to the Free Software
* Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA 02110-1301, USA.
* 
* ========== Copyright Header End ============================================
*/
// -*- verilog -*-
////////////////////////////////////////////////////////////////////////
/*
//
// Description:		Global header file that contain definitions that 
//                      are common/shared at the systme level
*/
////////////////////////////////////////////////////////////////////////
//
// Setting the time scale
// If the timescale changes, JP_TIMESCALE may also have to change.
`timescale	1ps/1ps
`default_nettype wire

//
// Number of threads in a core
// ===========================
//

//`define CONFIG_NUM_THREADS // This must be defined for any of below to work
//`define THREADS_1
//`define THREADS_2
//`define THREADS_3


//
// JBUS clock
// =========
//
// `define SYSCLK_PERIOD   5000


// Afara Link Defines
// ==================

// Reliable Link




// Afara Link Objects


// Afara Link Object Format - Reliable Link










// Afara Link Object Format - Congestion



  







// Afara Link Object Format - Acknowledge











// Afara Link Object Format - Request

















// Afara Link Object Format - Message



// Acknowledge Types




// Request Types





// Afara Link Frame



//
// UCB Packet Type
// ===============
//

















//
// UCB Data Packet Format
// ======================
//






























// Size encoding for the UCB_SIZE_HI/LO field
// 000 - byte
// 001 - half-word
// 010 - word
// 011 - double-word
// 111 - quad-word







//
// UCB Interrupt Packet Format
// ===========================
//










//`define UCB_THR_HI             9      // (6) cpu/thread ID shared with
//`define UCB_THR_LO             4             data packet format
//`define UCB_PKT_HI             3      // (4) packet type shared with
//`define UCB_PKT_LO             0      //     data packet format







//
// FCRAM Bus Widths
// ================
//






//
// ENET clock periods
// ==================
//
// `define AXGRMII_CLK_PERIOD          6400 // 312.5MHz/2
// `define ENET_GMAC_CLK_PERIOD        8000 // 125MHz


//
// JBus Bridge defines
// =================
//
// `define      SYS_UPA_CLK        `SYS.upa_clk
// `define      SYS_J_CLK          `SYS.j_clk
// `define      SYS_P_CLK          `SYS.p_clk
// `define      SYS_G_CLK          `SYS.g_clk
// `define      JP_TIMESCALE       `timescale 1 ps / 1 ps
// `define      PCI_CLK_PERIOD     15152                  //  66 MHz
// `define      UPA_RD_CLK_PERIOD  6666                   // 150 MHz
// `define      UPA_REF_CLK_PERIOD 7576                   // 132 MHz
// `define      ICHIP_CLK_PERIOD   30304                  //  33 MHz


//
// PCI Device Address Configuration
// ================================
//
























module sparc_exu_eclccr (/*AUTOARG*/
   // Outputs
   exu_ifu_cc_d, exu_tlu_ccr0_w, exu_tlu_ccr1_w, exu_tlu_ccr2_w, 
   exu_tlu_ccr3_w, 
   // Inputs
   clk, se, alu_xcc_e, alu_icc_e, tid_d, thrdec_d, thr_match_dm, 
   thr_match_de, tid_w, thr_w, ifu_exu_kill_e, ifu_exu_setcc_d, 
   byp_ecl_wrccr_data_w, wb_ccr_wrccr_w, wb_ccr_setcc_g, 
   divcntl_ccr_cc_w2, wb_ccr_thr_g, tlu_exu_cwpccr_update_m, 
   tlu_exu_ccr_m, ifu_exu_inst_vld_w, ifu_tlu_flush_w, early_flush_w
   ) ;
   input clk;
   input se;
   input [3:0] alu_xcc_e;    // condition codes from the alu
   input [3:0] alu_icc_e;
   input [1:0] tid_d;   // thread for each stage
   input [3:0] thrdec_d;   // decoded tid_d for mux select
   input       thr_match_dm;
   input       thr_match_de;
   input [1:0] tid_w;
   input [3:0] thr_w;        // decoded tid_w
   input       ifu_exu_kill_e;
   input       ifu_exu_setcc_d;
   input [7:0] byp_ecl_wrccr_data_w;// for the WRCCR operation (LSBs of
   input       wb_ccr_wrccr_w; // ALU result) + wen signal
   input       wb_ccr_setcc_g;
   input [7:0] divcntl_ccr_cc_w2;
   input [1:0] wb_ccr_thr_g;
   input       tlu_exu_cwpccr_update_m;
   input [7:0] tlu_exu_ccr_m;
   input       ifu_exu_inst_vld_w;
   input       ifu_tlu_flush_w;
   input       early_flush_w;
   
   output [7:0] exu_ifu_cc_d;   // condition codes for current thread
   output [7:0] exu_tlu_ccr0_w;
   output [7:0] exu_tlu_ccr1_w;
   output [7:0] exu_tlu_ccr2_w;
   output [7:0] exu_tlu_ccr3_w;

   wire [7:0]   partial_cc_d;   // partial bypassed ccr
   wire [7:0]   alu_cc_e;   // alu combined condition codes
   wire [7:0]   alu_cc_m;   // m stage alu ccs
   wire [7:0]   alu_cc_w; 
   wire [7:0]   exu_ifu_cc_w;   // writeback data
   wire         setcc_e;        // from previous stage
   wire         setcc_m;
   wire         setcc_w;
   wire         valid_setcc_e;  // after comparing with kill
   wire         valid_setcc_m;
   wire         valid_setcc_w;
   wire         setcc_w2;
   wire [7:0]   ccrin_thr0;
   wire [7:0]   ccrin_thr1;
   wire [7:0]   ccrin_thr2;
   wire [7:0]   ccrin_thr3;
   wire [7:0]   ccr_d;
   wire [7:0]   ccr_thr0;
   wire [7:0]   ccr_thr1;
   wire [7:0]   ccr_thr2;
   wire [7:0]   ccr_thr3;
   wire         use_alu_cc;
   wire         use_ccr;
   wire         use_cc_e;
   wire         use_cc_m;
   wire         use_cc_w;
   wire  [1:0]   tid_dxorw;
   wire         thr_match_de;
   wire         thrmatch_w;
   wire [1:0]   thr_w2;
   wire          thr0_w2;
   wire          thr1_w2;
   wire          thr2_w2;
   wire          thr3_w2;
   wire          wen_thr0_w;    // write enable for each input/thread
   wire          wen_thr0_w2;
   wire          wen_thr1_w;
   wire          wen_thr1_w2;
   wire          wen_thr2_w;
   wire          wen_thr2_w2;
   wire          wen_thr3_w;
   wire          wen_thr3_w2;
   wire          wen_thr0_l;      // overall write enable for each thread
   wire          wen_thr1_l;
   wire          wen_thr2_l;
   wire          wen_thr3_l;
   wire          bypass_cc_w;

   wire [7:0]    ccr_m;


   // D2E flops
   dff_s dff_setcc_d2e(.din(ifu_exu_setcc_d), .clk(clk), .q(setcc_e),
                     .se(se), .si(), .so());
   
   // E stage
   assign       alu_cc_e = {alu_xcc_e, alu_icc_e};
   assign       valid_setcc_e = setcc_e & ~ifu_exu_kill_e;

   dff_s #(8) dff_cc_e2m(.din(alu_cc_e[7:0]), .clk(clk), .q(alu_cc_m[7:0]),
                  .se(se), .si(), .so());
   dff_s dff_setcc_e2m(.din(valid_setcc_e), .clk(clk), .q(setcc_m),
                     .se(se), .si(), .so());
   
   // M stage
   assign       valid_setcc_m = setcc_m | tlu_exu_cwpccr_update_m;
   mux2ds #(8) mux_ccr_m(.dout(ccr_m[7:0]),
                            .in0(alu_cc_m[7:0]),
                            .in1(tlu_exu_ccr_m[7:0]),
                            .sel0(~tlu_exu_cwpccr_update_m),
                            .sel1(tlu_exu_cwpccr_update_m));

   dff_s #(8) dff_cc_m2w(.din(ccr_m[7:0]), .clk(clk), .q(alu_cc_w[7:0]),
                  .se(se), .si(), .so());
   dff_s dff_setcc_m2w(.din(valid_setcc_m), .clk(clk), .q(setcc_w),
                     .se(se), .si(), .so());

   // W stage
   assign bypass_cc_w = ifu_exu_inst_vld_w & setcc_w;
   assign valid_setcc_w = ~ifu_tlu_flush_w & ~early_flush_w & ifu_exu_inst_vld_w & (setcc_w | wb_ccr_wrccr_w);

   // mux with wrccr
   assign        use_alu_cc = ~(wb_ccr_wrccr_w);
   mux2ds #(8) mux_ccrin_cc(.dout(exu_ifu_cc_w[7:0]), .sel0(wb_ccr_wrccr_w),
                          .sel1(use_alu_cc),
                          .in0(byp_ecl_wrccr_data_w[7:0]),
                          .in1(alu_cc_w[7:0]));

   dff_s #(3) setcc_g2w2 (.din({wb_ccr_setcc_g, wb_ccr_thr_g[1:0]}), .clk(clk), 
                        .q({setcc_w2, thr_w2[1:0]}),
                        .se(se), .si(), .so());

   
   /////////////////////////
   // Storage of ccr
   /////////////////////////

 // Use two threads unless this is defined

   // decode thr_w2 for mux select
   assign        thr0_w2 = ~thr_w2[1] & ~thr_w2[0];
   assign        thr1_w2 = ~thr_w2[1] & thr_w2[0];
   // enable input for each thread
   assign        wen_thr0_w = (thr_w[0] & valid_setcc_w & ~wen_thr0_w2);
   assign        wen_thr0_w2 = thr0_w2 & setcc_w2;
   assign        wen_thr0_l = ~(wen_thr0_w | wen_thr0_w2);
   assign        wen_thr1_w = (thr_w[1] & valid_setcc_w & ~wen_thr1_w2);
   assign        wen_thr1_w2 = (thr1_w2 & setcc_w2);
   assign        wen_thr1_l = ~(wen_thr1_w | wen_thr1_w2);

   // mux between cc_w, cc_w2, old value, tlu value
   mux3ds #(8) mux_ccrin0(.dout(ccrin_thr0[7:0]), .sel0(wen_thr0_w),
                          .sel1(wen_thr0_w2), .sel2(wen_thr0_l),
                          .in0(exu_ifu_cc_w[7:0]),
                          .in1(divcntl_ccr_cc_w2[7:0]), .in2(ccr_thr0[7:0]));
   mux3ds #(8) mux_ccrin1(.dout(ccrin_thr1[7:0]), .sel0(wen_thr1_w),
                          .sel1(wen_thr1_w2), .sel2(wen_thr1_l),
                          .in0(exu_ifu_cc_w[7:0]),
                          .in1(divcntl_ccr_cc_w2[7:0]), .in2(ccr_thr1[7:0]));

    // store new value
   dff_s #(8) dff_ccr_thr0(.din(ccrin_thr0[7:0]), .clk(clk), .q(ccr_thr0[7:0]),
                       .se(se), .si(), .so());
   dff_s #(8) dff_ccr_thr1(.din(ccrin_thr1[7:0]), .clk(clk), .q(ccr_thr1[7:0]),
                       .se(se), .si(), .so());

   // mux between the 2 sets of ccrs
   mux2ds #(8) mux_ccr_out(.dout(ccr_d[7:0]), .sel0(thrdec_d[0]),
                         .sel1(thrdec_d[1]), .in0(ccr_thr0[7:0]),
                         .in1(ccr_thr1[7:0]));














































































































































































 // `ifndef CONFIG_NUM_THREADS
   
   // bypass the ccs to the output.  Only alu result needs to be bypassed
   assign        exu_ifu_cc_d[7:0] = (use_cc_e)? alu_cc_e[7:0]: partial_cc_d[7:0]; 
   mux3ds #(8) mux_ccr_bypass1(.dout(partial_cc_d[7:0]), 
                               .sel0(use_ccr),
                               .sel1(use_cc_m),
                               .sel2(use_cc_w), 
                               .in0(ccr_d[7:0]),
                               .in1(alu_cc_m[7:0]),
                               .in2(alu_cc_w[7:0]));

   assign        use_cc_e = valid_setcc_e & thr_match_de;
   assign        use_cc_m = setcc_m & thr_match_dm;
   assign        use_cc_w = bypass_cc_w & thrmatch_w & ~use_cc_m;
   assign        use_ccr = ~(use_cc_m | use_cc_w);

   assign        tid_dxorw = tid_w ^ tid_d;
   
   assign        thrmatch_w = ~(tid_dxorw[1] | tid_dxorw[0]);

   // generate ccr_w for the tlu
   assign        exu_tlu_ccr0_w[7:0] = ccr_thr0[7:0];
   assign        exu_tlu_ccr1_w[7:0] = ccr_thr1[7:0];
   assign        exu_tlu_ccr2_w[7:0] = ccr_thr2[7:0];
   assign        exu_tlu_ccr3_w[7:0] = ccr_thr3[7:0];

   
endmodule // sparc_exu_eclccr
// Modified by Princeton University on June 9th, 2015
// ========== Copyright Header Begin ==========================================
// 
// OpenSPARC T1 Processor File: sparc_exu_eclcomp7.v
// Copyright (c) 2006 Sun Microsystems, Inc.  All Rights Reserved.
// DO NOT ALTER OR REMOVE COPYRIGHT NOTICES.
// 
// The above named program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public
// License version 2 as published by the Free Software Foundation.
// 
// The above named program is distributed in the hope that it will be 
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// General Public License for more details.
// 
// You should have received a copy of the GNU General Public
// License along with this work; if not, write to the Free Software
// Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA 02110-1301, USA.
// 
// ========== Copyright Header End ============================================
////////////////////////////////////////////////////////////////////////
/*
//  Module Name: sparc_exu_eclcomp7
//	Description: This block is a 7 bit comparator.  It takes 2 inputs
// 		and outputs a 1 on out if they are equal.
*/

module sparc_exu_eclcomp7 (/*AUTOARG*/
   // Outputs
   out, 
   // Inputs
   in1, in2
   ) ;
   input [6:0] in1;
   input [6:0] in2;
   output      out;

   wire [6:0]  in1xorin2;
   wire nor1out;
   wire nor2out;
   wire nor3out;
   wire nandout;
   
   assign in1xorin2 = in1 ^ in2;
   assign nor1out = ~(in1xorin2[0] | in1xorin2[1]);
   assign nor2out = ~(in1xorin2[2] | in1xorin2[3]);
   assign nor3out = ~(in1xorin2[4] | in1xorin2[5]);
   assign nandout = ~(nor1out & nor2out & nor3out);
   assign out = ~(in1xorin2[6] | nandout);
   
   
endmodule // sparc_exu_eclcomp7
// Modified by Princeton University on June 9th, 2015
// ========== Copyright Header Begin ==========================================
// 
// OpenSPARC T1 Processor File: sparc_exu_reg.v
// Copyright (c) 2006 Sun Microsystems, Inc.  All Rights Reserved.
// DO NOT ALTER OR REMOVE COPYRIGHT NOTICES.
// 
// The above named program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public
// License version 2 as published by the Free Software Foundation.
// 
// The above named program is distributed in the hope that it will be 
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// General Public License for more details.
// 
// You should have received a copy of the GNU General Public
// License along with this work; if not, write to the Free Software
// Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA 02110-1301, USA.
// 
// ========== Copyright Header End ============================================
// Modified by Princeton University on June 9th, 2015
/*
* ========== Copyright Header Begin ==========================================
* 
* OpenSPARC T1 Processor File: sys.h
* Copyright (c) 2006 Sun Microsystems, Inc.  All Rights Reserved.
* DO NOT ALTER OR REMOVE COPYRIGHT NOTICES.
* 
* The above named program is free software; you can redistribute it and/or
* modify it under the terms of the GNU General Public
* License version 2 as published by the Free Software Foundation.
* 
* The above named program is distributed in the hope that it will be 
* useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
* General Public License for more details.
* 
* You should have received a copy of the GNU General Public
* License along with this work; if not, write to the Free Software
* Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA 02110-1301, USA.
* 
* ========== Copyright Header End ============================================
*/
// -*- verilog -*-
////////////////////////////////////////////////////////////////////////
/*
//
// Description:		Global header file that contain definitions that 
//                      are common/shared at the systme level
*/
////////////////////////////////////////////////////////////////////////
//
// Setting the time scale
// If the timescale changes, JP_TIMESCALE may also have to change.
`timescale	1ps/1ps
`default_nettype wire

//
// Number of threads in a core
// ===========================
//

//`define CONFIG_NUM_THREADS // This must be defined for any of below to work
//`define THREADS_1
//`define THREADS_2
//`define THREADS_3


//
// JBUS clock
// =========
//
// `define SYSCLK_PERIOD   5000


// Afara Link Defines
// ==================

// Reliable Link




// Afara Link Objects


// Afara Link Object Format - Reliable Link










// Afara Link Object Format - Congestion



  







// Afara Link Object Format - Acknowledge











// Afara Link Object Format - Request

















// Afara Link Object Format - Message



// Acknowledge Types




// Request Types





// Afara Link Frame



//
// UCB Packet Type
// ===============
//

















//
// UCB Data Packet Format
// ======================
//






























// Size encoding for the UCB_SIZE_HI/LO field
// 000 - byte
// 001 - half-word
// 010 - word
// 011 - double-word
// 111 - quad-word







//
// UCB Interrupt Packet Format
// ===========================
//










//`define UCB_THR_HI             9      // (6) cpu/thread ID shared with
//`define UCB_THR_LO             4             data packet format
//`define UCB_PKT_HI             3      // (4) packet type shared with
//`define UCB_PKT_LO             0      //     data packet format







//
// FCRAM Bus Widths
// ================
//






//
// ENET clock periods
// ==================
//
// `define AXGRMII_CLK_PERIOD          6400 // 312.5MHz/2
// `define ENET_GMAC_CLK_PERIOD        8000 // 125MHz


//
// JBus Bridge defines
// =================
//
// `define      SYS_UPA_CLK        `SYS.upa_clk
// `define      SYS_J_CLK          `SYS.j_clk
// `define      SYS_P_CLK          `SYS.p_clk
// `define      SYS_G_CLK          `SYS.g_clk
// `define      JP_TIMESCALE       `timescale 1 ps / 1 ps
// `define      PCI_CLK_PERIOD     15152                  //  66 MHz
// `define      UPA_RD_CLK_PERIOD  6666                   // 150 MHz
// `define      UPA_REF_CLK_PERIOD 7576                   // 132 MHz
// `define      ICHIP_CLK_PERIOD   30304                  //  33 MHz


//
// PCI Device Address Configuration
// ================================
//
























module sparc_exu_reg (/*AUTOARG*/
   // Outputs
   data_out, 
   // Inputs
   clk, se, thr_out, wen_w, thr_w, data_in_w
   ) ;
   parameter SIZE = 3;

   input     clk;
   input     se;
   input [3:0]       thr_out;
   input             wen_w;
   input [3:0]       thr_w;
   input [SIZE -1:0] data_in_w;

   output [SIZE-1:0] data_out;

   wire [SIZE-1:0]   data_thr0;
   wire [SIZE-1:0]   data_thr1;
   wire [SIZE-1:0]   data_thr2;
   wire [SIZE-1:0]   data_thr3;
   wire [SIZE-1:0]   data_thr0_next;
   wire [SIZE-1:0]   data_thr1_next;
   wire [SIZE-1:0]   data_thr2_next;
   wire [SIZE-1:0]   data_thr3_next;

   wire          wen_thr0_w;
   wire          wen_thr1_w;
   wire          wen_thr2_w;
   wire          wen_thr3_w;

   //////////////////////////////////
   //  Output selection for reg
   //////////////////////////////////

 // Use two threads unless this is defined

   // mux between the 2 regs
   mux2ds #(SIZE) mux_data_out1(.dout(data_out[SIZE -1:0]), .sel0(thr_out[0]),
                               .sel1(thr_out[1]), .in0(data_thr0[SIZE -1:0]),
                               .in1(data_thr1[SIZE -1:0]));
   //////////////////////////////////////
   //  Storage of reg
   //////////////////////////////////////
   // enable input for each thread
   assign        wen_thr0_w = (thr_w[0] & wen_w);
   assign        wen_thr1_w = (thr_w[1] & wen_w);

   // mux between new and current value
   mux2ds #(SIZE) data_next0_mux(.dout(data_thr0_next[SIZE -1:0]),
                               .in0(data_thr0[SIZE -1:0]),
                               .in1(data_in_w[SIZE -1:0]),
                               .sel0(~wen_thr0_w),
                               .sel1(wen_thr0_w));
   mux2ds #(SIZE) data_next1_mux(.dout(data_thr1_next[SIZE -1:0]),
                               .in0(data_thr1[SIZE -1:0]),
                               .in1(data_in_w[SIZE -1:0]),
                               .sel0(~wen_thr1_w),
                               .sel1(wen_thr1_w));

   // store new value
   dff_s #(SIZE) dff_reg_thr0(.din(data_thr0_next[SIZE -1:0]), .clk(clk), .q(data_thr0[SIZE -1:0]),
                       .se(se), .si(), .so());
   dff_s #(SIZE) dff_reg_thr1(.din(data_thr1_next[SIZE -1:0]), .clk(clk), .q(data_thr1[SIZE -1:0]),
                       .se(se), .si(), .so());























































































































































 // `ifndef CONFIG_NUM_THREADS
   
endmodule // sparc_exu_reg
// Modified by Princeton University on June 9th, 2015
// ========== Copyright Header Begin ==========================================
// 
// OpenSPARC T1 Processor File: sparc_exu_rml.v
// Copyright (c) 2006 Sun Microsystems, Inc.  All Rights Reserved.
// DO NOT ALTER OR REMOVE COPYRIGHT NOTICES.
// 
// The above named program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public
// License version 2 as published by the Free Software Foundation.
// 
// The above named program is distributed in the hope that it will be 
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// General Public License for more details.
// 
// You should have received a copy of the GNU General Public
// License along with this work; if not, write to the Free Software
// Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA 02110-1301, USA.
// 
// ========== Copyright Header End ============================================
////////////////////////////////////////////////////////////////////////
/*
//  Module Name: sparc_exu_rml
//	Description: Register management logic.  Contains CWP, CANSAVE, CANRESTORE
//		and other window management registers.  Generates RF related traps
//  		and switches the global registers to alternate globals.  All the registers
//		are written in the W stage (there is no bypassing so they must
//		swap out) and will either get a new value generated by a window management
//		Instruction or by a WRPS instruction.  The following traps can be generated:
//			Fill: restore with canrestore == 0
//			clean_window: save with cleanwin-canrestore == 0
//			spill: flushw with cansave != nwindows -2 or
//				save with cansave == 0
//		It is assumed that the contents of the new window will get squashed
//		on a clean_window or fill trap so the save or restore gets executed
//		normally.  Spill traps or WRCWPs mean that all 16 windowed registers
//		must be saved and restored (a 4 cycle operation).
*/
// Modified by Princeton University on June 9th, 2015
/*
* ========== Copyright Header Begin ==========================================
* 
* OpenSPARC T1 Processor File: sys.h
* Copyright (c) 2006 Sun Microsystems, Inc.  All Rights Reserved.
* DO NOT ALTER OR REMOVE COPYRIGHT NOTICES.
* 
* The above named program is free software; you can redistribute it and/or
* modify it under the terms of the GNU General Public
* License version 2 as published by the Free Software Foundation.
* 
* The above named program is distributed in the hope that it will be 
* useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
* General Public License for more details.
* 
* You should have received a copy of the GNU General Public
* License along with this work; if not, write to the Free Software
* Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA 02110-1301, USA.
* 
* ========== Copyright Header End ============================================
*/
// -*- verilog -*-
////////////////////////////////////////////////////////////////////////
/*
//
// Description:		Global header file that contain definitions that 
//                      are common/shared at the systme level
*/
////////////////////////////////////////////////////////////////////////
//
// Setting the time scale
// If the timescale changes, JP_TIMESCALE may also have to change.
`timescale	1ps/1ps
`default_nettype wire

//
// Number of threads in a core
// ===========================
//

//`define CONFIG_NUM_THREADS // This must be defined for any of below to work
//`define THREADS_1
//`define THREADS_2
//`define THREADS_3


//
// JBUS clock
// =========
//
// `define SYSCLK_PERIOD   5000


// Afara Link Defines
// ==================

// Reliable Link




// Afara Link Objects


// Afara Link Object Format - Reliable Link










// Afara Link Object Format - Congestion



  







// Afara Link Object Format - Acknowledge











// Afara Link Object Format - Request

















// Afara Link Object Format - Message



// Acknowledge Types




// Request Types





// Afara Link Frame



//
// UCB Packet Type
// ===============
//

















//
// UCB Data Packet Format
// ======================
//






























// Size encoding for the UCB_SIZE_HI/LO field
// 000 - byte
// 001 - half-word
// 010 - word
// 011 - double-word
// 111 - quad-word







//
// UCB Interrupt Packet Format
// ===========================
//










//`define UCB_THR_HI             9      // (6) cpu/thread ID shared with
//`define UCB_THR_LO             4             data packet format
//`define UCB_PKT_HI             3      // (4) packet type shared with
//`define UCB_PKT_LO             0      //     data packet format







//
// FCRAM Bus Widths
// ================
//






//
// ENET clock periods
// ==================
//
// `define AXGRMII_CLK_PERIOD          6400 // 312.5MHz/2
// `define ENET_GMAC_CLK_PERIOD        8000 // 125MHz


//
// JBus Bridge defines
// =================
//
// `define      SYS_UPA_CLK        `SYS.upa_clk
// `define      SYS_J_CLK          `SYS.j_clk
// `define      SYS_P_CLK          `SYS.p_clk
// `define      SYS_G_CLK          `SYS.g_clk
// `define      JP_TIMESCALE       `timescale 1 ps / 1 ps
// `define      PCI_CLK_PERIOD     15152                  //  66 MHz
// `define      UPA_RD_CLK_PERIOD  6666                   // 150 MHz
// `define      UPA_REF_CLK_PERIOD 7576                   // 132 MHz
// `define      ICHIP_CLK_PERIOD   30304                  //  33 MHz


//
// PCI Device Address Configuration
// ================================
//
























module sparc_exu_rml (/*AUTOARG*/
   // Outputs
   exu_tlu_spill_wtype, exu_tlu_spill_other, exu_tlu_cwp_retry, 
   exu_tlu_cwp3_w, exu_tlu_cwp2_w, exu_tlu_cwp1_w, exu_tlu_cwp0_w, 
   so, exu_tlu_cwp_cmplt, exu_tlu_cwp_cmplt_tid, rml_ecl_cwp_d, 
   rml_ecl_cansave_d, rml_ecl_canrestore_d, rml_ecl_otherwin_d, 
   rml_ecl_wstate_d, rml_ecl_cleanwin_d, rml_ecl_fill_e, 
   rml_ecl_clean_window_e, rml_ecl_other_e, rml_ecl_wtype_e, 
   exu_ifu_spill_e, rml_ecl_gl_e, rml_irf_old_lo_cwp_e, 
   rml_irf_new_lo_cwp_e, rml_irf_old_e_cwp_e, rml_irf_new_e_cwp_e, 
   rml_irf_swap_even_e, rml_irf_swap_odd_e, rml_irf_swap_local_e, 
   rml_irf_kill_restore_w, rml_irf_cwpswap_tid_e, rml_ecl_swap_done, 
   rml_ecl_rmlop_done_e, exu_ifu_oddwin_s, exu_tlu_spill, 
   exu_tlu_spill_tid, rml_ecl_kill_m, rml_irf_old_agp, 
   rml_irf_new_agp, rml_irf_swap_global, rml_irf_global_tid, 
   // Inputs
   tlu_exu_cwp_retry_m, rst_tri_en, rclk, se, si, grst_l, arst_l, 
   ifu_exu_tid_s2, ifu_exu_save_d, ifu_exu_restore_d, 
   ifu_exu_saved_e, ifu_exu_restored_e, ifu_exu_flushw_e, 
   ecl_rml_thr_m, ecl_rml_thr_w, ecl_rml_cwp_wen_e, 
   ecl_rml_cansave_wen_w, ecl_rml_canrestore_wen_w, 
   ecl_rml_otherwin_wen_w, ecl_rml_wstate_wen_w, 
   ecl_rml_cleanwin_wen_w, ecl_rml_xor_data_e, ecl_rml_kill_e, 
   ecl_rml_kill_w, ecl_rml_early_flush_w, exu_tlu_wsr_data_w, 
   tlu_exu_agp, tlu_exu_agp_swap, tlu_exu_agp_tid, tlu_exu_cwp_m, 
   tlu_exu_cwpccr_update_m, ecl_rml_inst_vld_w
   ) ;
   input rclk;
   input se;
   input si;
   input grst_l;
   input arst_l;
   input [1:0] ifu_exu_tid_s2;
   input       ifu_exu_save_d;
   input       ifu_exu_restore_d;
   input       ifu_exu_saved_e;
   input       ifu_exu_restored_e;
   input       ifu_exu_flushw_e;
   input [3:0] ecl_rml_thr_m;
   input [3:0] ecl_rml_thr_w;
   input       ecl_rml_cwp_wen_e;
   input       ecl_rml_cansave_wen_w;
   input       ecl_rml_canrestore_wen_w;
   input       ecl_rml_otherwin_wen_w;
   input       ecl_rml_wstate_wen_w;
   input       ecl_rml_cleanwin_wen_w;
   input [2:0] ecl_rml_xor_data_e;
   input       ecl_rml_kill_e;// needed for oddwin updates
   input       ecl_rml_kill_w;
   input       ecl_rml_early_flush_w;
   input [5:0] exu_tlu_wsr_data_w; // for wstate
   input [1:0]   tlu_exu_agp;   // alternate global pointer
   input         tlu_exu_agp_swap;// switch globals
   input [1:0]   tlu_exu_agp_tid;// thread that agp refers to
   input [2:0] tlu_exu_cwp_m;   // for switching cwp on return from trap
   input       tlu_exu_cwpccr_update_m;
   input       ecl_rml_inst_vld_w;
   /*AUTOINPUT*/
   // Beginning of automatic inputs (from unused autoinst inputs)
   input                rst_tri_en;             // To cwp of sparc_exu_rml_cwp.v
   input                tlu_exu_cwp_retry_m;    // To cwp of sparc_exu_rml_cwp.v
   // End of automatics

   /*AUTOOUTPUT*/
   // Beginning of automatic outputs (from unused autoinst outputs)
   output [2:0]         exu_tlu_cwp0_w;         // From cwp of sparc_exu_rml_cwp.v
   output [2:0]         exu_tlu_cwp1_w;         // From cwp of sparc_exu_rml_cwp.v
   output [2:0]         exu_tlu_cwp2_w;         // From cwp of sparc_exu_rml_cwp.v
   output [2:0]         exu_tlu_cwp3_w;         // From cwp of sparc_exu_rml_cwp.v
   output               exu_tlu_cwp_retry;      // From cwp of sparc_exu_rml_cwp.v
   output               exu_tlu_spill_other;    // From cwp of sparc_exu_rml_cwp.v
   output [2:0]         exu_tlu_spill_wtype;    // From cwp of sparc_exu_rml_cwp.v
   // End of automatics
   output               so;
   output      exu_tlu_cwp_cmplt;
   output [1:0] exu_tlu_cwp_cmplt_tid;
   output [2:0]  rml_ecl_cwp_d;
   output [2:0]  rml_ecl_cansave_d;
   output [2:0]  rml_ecl_canrestore_d;
   output [2:0]  rml_ecl_otherwin_d;
   output [5:0]  rml_ecl_wstate_d;
   output [2:0]  rml_ecl_cleanwin_d;
   output        rml_ecl_fill_e;
   output        rml_ecl_clean_window_e;
   output        rml_ecl_other_e;
   output [2:0] rml_ecl_wtype_e;
   output       exu_ifu_spill_e;
   output [1:0] rml_ecl_gl_e;

   output [2:0]  rml_irf_old_lo_cwp_e;  // current window pointer for locals and odds
   output [2:0]  rml_irf_new_lo_cwp_e;  // current window pointer for locals and odd
   output [1:0]  rml_irf_old_e_cwp_e;  // current window pointer for evens
   output [1:0]  rml_irf_new_e_cwp_e;  // current window pointer for evens
   output        rml_irf_swap_even_e;
   output        rml_irf_swap_odd_e;
   output        rml_irf_swap_local_e;
   output        rml_irf_kill_restore_w;
   output [1:0]  rml_irf_cwpswap_tid_e;

   output [3:0] rml_ecl_swap_done;
   output       rml_ecl_rmlop_done_e;   
   output [3:0] exu_ifu_oddwin_s;
   output       exu_tlu_spill;
   output [1:0] exu_tlu_spill_tid;
   output       rml_ecl_kill_m;
   
   output [1:0]  rml_irf_old_agp; // alternate global pointer
   output [1:0]  rml_irf_new_agp; // alternate global pointer
   output        rml_irf_swap_global;
   output [1:0]  rml_irf_global_tid;

   wire          clk;
   wire [1:0]    tid_d;
   wire [3:0]    thr_d;
   wire [1:0]    tid_e;
   wire          rml_reset_l;
   wire          reset;
   wire          save_e;
   wire          save_m;
   wire          restore_e;
   wire          swap_e;
   wire          agp_wen;
   wire [1:0]    agp_thr0;
   wire [1:0]    agp_thr1;
   wire [1:0]    agp_thr2;
   wire [1:0]    agp_thr3;
   wire [1:0]    agp_thr0_next;
   wire [1:0]    agp_thr1_next;
   wire [1:0]    agp_thr2_next;
   wire [1:0]    agp_thr3_next;
   wire          agp_wen_thr0_w;
   wire          agp_wen_thr1_w;
   wire          agp_wen_thr2_w;
   wire          agp_wen_thr3_w;
   wire [1:0]    new_agp;   
   wire [1:0]    agp_tid;
   wire [3:0]    agp_thr;
   wire        full_swap_e;
   wire   did_restore_m;
   wire   did_restore_w;
   wire   kill_restore_m;
   wire   kill_restore_w;

   wire [2:0]  rml_ecl_cwp_e;
   wire [2:0]  rml_ecl_cansave_e;
   wire [2:0]  rml_ecl_canrestore_e;
   wire [2:0]  rml_ecl_otherwin_e;
   wire [2:0]  rml_ecl_cleanwin_e;

   wire [2:0]  rml_next_cwp_e;        
   wire [2:0]  rml_next_cansave_e;// e-stage of rml generated new data
   wire [2:0]  rml_next_canrestore_e;
   wire [2:0]  rml_next_otherwin_e;
   wire [2:0]  rml_next_cleanwin_e;
   
   wire [2:0]  next_cwp_e;      
   wire [2:0]  next_cansave_e;  // e-stage of new data
   wire [2:0]  next_canrestore_e;
   wire [2:0]  next_otherwin_e;
   wire [2:0]  next_cleanwin_e;
   wire [2:0]  next_cwp_m;      // m-stage of new data
   wire [2:0]  next_cansave_m;
   wire [2:0]  next_canrestore_m;
   wire [2:0]  next_otherwin_m;
   wire [2:0]  next_cleanwin_m;
   wire [2:0]  next_cansave_w;// w-stage of new data
   wire [2:0]  next_canrestore_w;
   wire [2:0]  next_otherwin_w;
   wire [2:0]  next_cleanwin_w;
   wire [2:0]  next_cwp_noreset_w;
   wire [2:0]  next_cwp_w;

   wire   rml_cwp_wen_e;        // wen for cwp from rml
   wire   rml_cwp_wen_m;        // wen for cwp from rml
   wire [2:0] spill_cwp_e;      // next cwp if there is a spill trap 
   wire       spill_cwp_carry0; // carry bit from spill cwp computations
   wire       spill_cwp_carry1;
   wire       next_cwp_sel_inc; // select line to next_cwp mux

   wire        rml_cansave_wen_w;// rml generated wen
   wire        rml_canrestore_wen_w;
   wire        rml_otherwin_wen_w;
   wire        rml_cleanwin_wen_w;

   wire        cansave_wen_w;// wen to registers
   wire        canrestore_wen_w;
   wire        otherwin_wen_w;
   wire        cleanwin_wen_w;
   wire        cwp_wen_nokill_w;
   wire        cwp_wen_w;
   wire        wstate_wen_w;

   wire        cwp_wen_m;       // rml generated wen w/o kills
   wire        cansave_wen_m;
   wire        canrestore_wen_m;
   wire        otherwin_wen_m;
   wire        cleanwin_wen_m;
   wire        cansave_wen_valid_m;	// rml generated wen w/ kills
   wire        canrestore_wen_valid_m;
   wire        otherwin_wen_valid_m;
   wire        cleanwin_wen_valid_m;

   wire      	 cwp_wen_e;       // rml generated wen_e
   wire        cansave_wen_e;
   wire        canrestore_wen_e;
   wire        otherwin_wen_e;
   wire        cleanwin_wen_e;

   wire        cansave_inc_e;
   wire        canrestore_inc_e;

   wire        spill_trap_save;
   wire        spill_trap_flush;
   wire        spill_m;
   wire [2:0]  cleanwin_xor_canrestore;

   wire        otherwin_is0_e;
   wire        cansave_is0_e;
   wire        canrestore_is0_e;

   wire        swap_locals_ins;
   wire        swap_outs;
   wire [2:0]  old_cwp_e;
   wire [2:0]  new_cwp_e;

   wire [2:0]   rml_ecl_wtype_d;
   wire [2:0]   rml_ecl_wtype_e;
   wire         rml_ecl_other_d;
   wire         rml_ecl_other_e;
   wire        exu_tlu_spill_e;
   wire         rml_ecl_kill_e;
   wire         rml_kill_w;
   wire         vld_w;
   wire         win_trap_e;
   wire         win_trap_m;
   wire         win_trap_w;

   assign       clk = rclk;
   // Reset flop
    dffrl_async rstff(.din (grst_l),
                        .q   (rml_reset_l),
                        .clk (clk),
                        .rst_l (arst_l), .se(se), .si(), .so());
   assign       reset = ~rml_reset_l;
 
   dff_s #(2) tid_s2d(.din(ifu_exu_tid_s2[1:0]), .clk(clk), .q(tid_d[1:0]), .se(se), .si(), .so());
   dff_s #(2) tid_d2e(.din(tid_d[1:0]), .clk(clk), .q(tid_e[1:0]), .se(se), .si(), .so());
   assign       thr_d[3] = tid_d[1] & tid_d[0];
   assign       thr_d[2] = tid_d[1] & ~tid_d[0];
   assign       thr_d[1] = ~tid_d[1] & tid_d[0];
   assign       thr_d[0] = ~tid_d[1] & ~tid_d[0];
   
   dff_s save_d2e(.din(ifu_exu_save_d), .clk(clk), .q(save_e), .se(se), .si(), .so());
   dff_s save_e2m(.din(save_e), .clk(clk), .q(save_m), .se(se), .si(), .so());
   dff_s restore_d2e(.din(ifu_exu_restore_d), .clk(clk), .q(restore_e), .se(se), .si(), .so());

   // don't check flush_pipe in w if caused by rml trap.  Things with a higher priority
   // than a window trap have been accumulated into ecl_rml_kill_w
   assign       vld_w = ecl_rml_inst_vld_w & (~ecl_rml_early_flush_w | win_trap_w);
   assign     rml_kill_w = ecl_rml_kill_w | ~vld_w;

   assign     win_trap_e = rml_ecl_fill_e | exu_tlu_spill_e | rml_ecl_clean_window_e;
   dff_s win_trap_e2m(.din(win_trap_e), .clk(clk), .q(win_trap_m), .se(se), .si(), .so());
   dff_s win_trap_m2w(.din(win_trap_m), .clk(clk), .q(win_trap_w), .se(se), .si(), .so());
   
   assign canrestore_is0_e = (~rml_ecl_canrestore_e[0] & ~rml_ecl_canrestore_e[1] 
                              & ~rml_ecl_canrestore_e[2]);
   assign cansave_is0_e = (~rml_ecl_cansave_e[0] & ~rml_ecl_cansave_e[1] & 
                           ~rml_ecl_cansave_e[2]);
   assign otherwin_is0_e = ~rml_ecl_other_e;

   ///////////////////////////////////////
   // Signals that operations are done
   // restore/return is not signalled here
   // because it depends on the write to the
   // irf (computed in ecl_wb)
   ////////////////////////////////////////
   assign rml_ecl_rmlop_done_e = (ifu_exu_saved_e | ifu_exu_restored_e |
                                  (ifu_exu_flushw_e & ~spill_trap_flush));
   
   //////////////////////////
   // Trap generation
   //////////////////////////
   // Fill trap generated on restore and canrestore == 0
   assign rml_ecl_fill_e = restore_e & canrestore_is0_e; 
   
   // Spill trap on save with cansave == 0
   assign spill_trap_save = save_e & cansave_is0_e;
   assign exu_ifu_spill_e = spill_trap_save;
   // Spill trap on wflush with cansave != (NWINDOWS - 2 = 6)
   assign spill_trap_flush = (ifu_exu_flushw_e & ~(rml_ecl_cansave_e[2] &
                                                 rml_ecl_cansave_e[1] & 
                                                 ~rml_ecl_cansave_e[0]));
   assign exu_tlu_spill_e = (spill_trap_save | spill_trap_flush);
   dff_s spill_e2m(.din(exu_tlu_spill_e), .clk(clk), .q(spill_m), .se(se), .si(), .so());

   // Clean window trap on save w/ cleanwin - canrestore == 0
   // or cleanwin == canrestore
   // (not signalled on spill traps because spill is higher priority)
   assign cleanwin_xor_canrestore = rml_ecl_cleanwin_e ^ rml_ecl_canrestore_e;
   assign rml_ecl_clean_window_e = ~(cleanwin_xor_canrestore[2] |
                                cleanwin_xor_canrestore[1] |
                                cleanwin_xor_canrestore[0]) & save_e & ~exu_tlu_spill_e;

   // Kill signal for w1 wen bit (all others don't care)
   assign rml_ecl_kill_e = rml_ecl_fill_e | exu_tlu_spill_e;
   dff_s rml_kill_e2m(.din(rml_ecl_kill_e), .clk(clk), .q(rml_ecl_kill_m),
                    .se(se), .si(), .so());
   

   // WTYPE generation
   assign rml_ecl_other_d = (rml_ecl_otherwin_d[0] | rml_ecl_otherwin_d[1] 
                            | rml_ecl_otherwin_d[2]);
   dff_s other_d2e(.din(rml_ecl_other_d), .clk(clk), .q(rml_ecl_other_e), .se(se),
                 .si(), .so());
   mux2ds #(3) wtype_mux(.dout(rml_ecl_wtype_d[2:0]),
                          .in0(rml_ecl_wstate_d[2:0]),
                          .in1(rml_ecl_wstate_d[5:3]),
                          .sel0(~rml_ecl_other_d),
                          .sel1(rml_ecl_other_d));
   dff_s #(3) wtype_d2e(.din(rml_ecl_wtype_d[2:0]), .clk(clk), .q(rml_ecl_wtype_e[2:0]),
                    .se(se), .si(), .so());


   ////////////////////////////
   // Interface with IRF
   ////////////////////////////
   assign rml_irf_old_lo_cwp_e[2:0] = old_cwp_e[2:0];
   assign rml_irf_new_lo_cwp_e[2:0] = new_cwp_e[2:0];
   assign rml_irf_old_e_cwp_e[1:0] = (old_cwp_e[0])? old_cwp_e[2:1] + 2'b01: old_cwp_e[2:1];
   assign rml_irf_new_e_cwp_e[1:0] = (new_cwp_e[0])? new_cwp_e[2:1] + 2'b01: new_cwp_e[2:1];
   
   assign rml_irf_swap_local_e = (swap_e | swap_locals_ins);
   assign rml_irf_swap_odd_e = ((save_e | ecl_rml_cwp_wen_e | spill_trap_flush | swap_locals_ins) & old_cwp_e[0]) | 
                                 ((restore_e | swap_outs) & ~old_cwp_e[0]);
   assign rml_irf_swap_even_e = ((save_e | ecl_rml_cwp_wen_e | spill_trap_flush | swap_locals_ins) & ~old_cwp_e[0]) |
                                  ((restore_e | swap_outs) & old_cwp_e[0]);

   assign swap_e = save_e | restore_e | ecl_rml_cwp_wen_e | spill_trap_flush;
   dff_s dff_did_restore_e2m(.din(swap_e), .clk(clk),
                       .q(did_restore_m), .se(se),
                       .si(), .so());
   dff_s dff_did_restore_m2w(.din(did_restore_m), .clk(clk),
                       .q(did_restore_w), .se(se),
                       .si(), .so());
   // kill restore on all saves (except those that spill) and any swaps that
   // get kill signals
   assign kill_restore_m = (~spill_m & save_m);
   dff_s dff_kill_restore_m2w(.din(kill_restore_m), .clk(clk), .q(kill_restore_w),
                            .se(se), .si(), .so());
   assign rml_irf_kill_restore_w = kill_restore_w | (did_restore_w & rml_kill_w);


   ///////////////////////////////
   // CWP logic
   ///////////////////////////////
   // Logic to compute next_cwp on spill trap.
   //  CWP = CWP + CANSAVE + 2
   assign spill_cwp_e[0] = rml_ecl_cwp_e[0] ^ rml_ecl_cansave_e[0];
   assign spill_cwp_carry0 = rml_ecl_cwp_e[0] & rml_ecl_cansave_e[0];
   assign spill_cwp_e[1] = rml_ecl_cwp_e[1] ^ rml_ecl_cansave_e[1] ^ ~spill_cwp_carry0;
   assign spill_cwp_carry1 = (rml_ecl_cwp_e[1] | rml_ecl_cansave_e[1] |
                              spill_cwp_carry0) & ~(rml_ecl_cwp_e[1] &
                                                    rml_ecl_cansave_e[1] &
                                                    spill_cwp_carry0);
   assign spill_cwp_e[2] = rml_ecl_cwp_e[2] ^ rml_ecl_cansave_e[2] ^ spill_cwp_carry1;

   assign rml_cwp_wen_e = (save_e | restore_e) & ~exu_tlu_spill_e;
   assign cwp_wen_e = (rml_cwp_wen_e | ecl_rml_cwp_wen_e) & ~ecl_rml_kill_e;
   sparc_exu_rml_inc3 cwp_inc(.dout(rml_next_cwp_e[2:0]), .din(rml_ecl_cwp_e[2:0]),
                                  .inc(save_e));

   assign     next_cwp_sel_inc = ~(ecl_rml_cwp_wen_e | exu_tlu_spill_e);
   mux3ds #(3) next_cwp_mux(.dout(next_cwp_e[2:0]), 
                          .in0(rml_next_cwp_e[2:0]),
                          .in1(ecl_rml_xor_data_e[2:0]),
                          .in2(spill_cwp_e[2:0]),
                          .sel0(next_cwp_sel_inc),
                          .sel1(ecl_rml_cwp_wen_e),
                          .sel2(exu_tlu_spill_e));

   dff_s cwp_wen_e2m(.din(cwp_wen_e), .clk(clk), .q(rml_cwp_wen_m),
                       .se(se), .si(), .so());
   dff_s #(3) next_cwp_e2m(.din(next_cwp_e[2:0]), .clk(clk), .q(next_cwp_m[2:0]),
                           .se(se), .si(), .so());
   assign     cwp_wen_m = rml_cwp_wen_m;
   dff_s #(3) next_cwp_m2w(.din(next_cwp_m[2:0]), .clk(clk), .q(next_cwp_noreset_w[2:0]),
                         .se(se), .si(), .so());
   dff_s cwp_wen_m2w(.din(cwp_wen_m), .clk(clk), .q(cwp_wen_nokill_w),
                       .se(se), .si(), .so());
   assign cwp_wen_w = cwp_wen_nokill_w & ~rml_kill_w;
   assign next_cwp_w[2:0] = next_cwp_noreset_w[2:0];

   assign full_swap_e = (exu_tlu_spill_e | ecl_rml_cwp_wen_e);


   // oddwin signal for ifu needs bypass from w.  It is done in M and staged for timing.
   // This is possible because the thread is switched out so there is only one bypass condition.
   // Only save/return will switch in fast enough for a bypass so this is the only write condition
   // we need to check
   wire [3:0] oddwin_m;
   wire [3:0] oddwin_w;
   assign     oddwin_m[3] = (cwp_wen_m & ecl_rml_thr_m[3])? next_cwp_m[0]: oddwin_w[3];
   assign     oddwin_m[2] = (cwp_wen_m & ecl_rml_thr_m[2])? next_cwp_m[0]: oddwin_w[2];
   assign     oddwin_m[1] = (cwp_wen_m & ecl_rml_thr_m[1])? next_cwp_m[0]: oddwin_w[1];
   assign     oddwin_m[0] = (cwp_wen_m & ecl_rml_thr_m[0])? next_cwp_m[0]: oddwin_w[0];
   dff_s #(4) oddwin_dff(.din(oddwin_m[3:0]), .clk(clk), .q(exu_ifu_oddwin_s[3:0]),
                       .se(se), .si(), .so());

   sparc_exu_rml_cwp cwp(
                         .swap_outs     (swap_outs),
                         .swap_locals_ins(swap_locals_ins),
                         .rml_ecl_cwp_e (rml_ecl_cwp_e[2:0]),
                         .old_cwp_e     (old_cwp_e[2:0]),
                         .new_cwp_e     (new_cwp_e[2:0]),
                         .oddwin_w     (oddwin_w[3:0]),
                         /*AUTOINST*/
                         // Outputs
                         .rml_ecl_cwp_d (rml_ecl_cwp_d[2:0]),
                         .exu_tlu_cwp0_w(exu_tlu_cwp0_w[2:0]),
                         .exu_tlu_cwp1_w(exu_tlu_cwp1_w[2:0]),
                         .exu_tlu_cwp2_w(exu_tlu_cwp2_w[2:0]),
                         .exu_tlu_cwp3_w(exu_tlu_cwp3_w[2:0]),
                         .rml_irf_cwpswap_tid_e(rml_irf_cwpswap_tid_e[1:0]),
                         .exu_tlu_spill (exu_tlu_spill),
                         .exu_tlu_spill_wtype(exu_tlu_spill_wtype[2:0]),
                         .exu_tlu_spill_other(exu_tlu_spill_other),
                         .exu_tlu_spill_tid(exu_tlu_spill_tid[1:0]),
                         .rml_ecl_swap_done(rml_ecl_swap_done[3:0]),
                         .exu_tlu_cwp_cmplt(exu_tlu_cwp_cmplt),
                         .exu_tlu_cwp_cmplt_tid(exu_tlu_cwp_cmplt_tid[1:0]),
                         .exu_tlu_cwp_retry(exu_tlu_cwp_retry),
                         // Inputs
                         .clk           (clk),
                         .se            (se),
                         .reset         (reset),
                         .rst_tri_en    (rst_tri_en),
                         .rml_ecl_wtype_e(rml_ecl_wtype_e[2:0]),
                         .rml_ecl_other_e(rml_ecl_other_e),
                         .exu_tlu_spill_e(exu_tlu_spill_e),
                         .tlu_exu_cwpccr_update_m(tlu_exu_cwpccr_update_m),
                         .tlu_exu_cwp_retry_m(tlu_exu_cwp_retry_m),
                         .tlu_exu_cwp_m (tlu_exu_cwp_m[2:0]),
                         .thr_d         (thr_d[3:0]),
                         .ecl_rml_thr_m (ecl_rml_thr_m[3:0]),
                         .ecl_rml_thr_w (ecl_rml_thr_w[3:0]),
                         .tid_e         (tid_e[1:0]),
                         .next_cwp_w    (next_cwp_w[2:0]),
                         .next_cwp_e    (next_cwp_e[2:0]),
                         .cwp_wen_w     (cwp_wen_w),
                         .save_e        (save_e),
                         .restore_e     (restore_e),
                         .ifu_exu_flushw_e(ifu_exu_flushw_e),
                         .ecl_rml_cwp_wen_e(ecl_rml_cwp_wen_e),
                         .full_swap_e   (full_swap_e),
                         .rml_kill_w    (rml_kill_w));

   ///////////////////////////////
   // Cansave logic
   ///////////////////////////////
   assign cansave_wen_e = ((save_e & ~cansave_is0_e & ~rml_ecl_clean_window_e) |
                           ifu_exu_saved_e |
                           (restore_e & ~canrestore_is0_e) |
                           (ifu_exu_restored_e & otherwin_is0_e));
   sparc_exu_rml_inc3 cansave_inc(.dout(rml_next_cansave_e[2:0]), .din(rml_ecl_cansave_e[2:0]),
                                  .inc(cansave_inc_e));
   assign cansave_inc_e = restore_e | ifu_exu_saved_e;

   mux2ds #(3) next_cansave_mux(.dout(next_cansave_e[2:0]),
                              .in0(ecl_rml_xor_data_e[2:0]),
                              .in1(rml_next_cansave_e[2:0]),
                              .sel0(~cansave_wen_e),
                              .sel1(cansave_wen_e));
   dff_s cansave_wen_e2m(.din(cansave_wen_e), .clk(clk), .q(cansave_wen_m),
                       .se(se), .si(), .so());
   dff_s #(3) next_cansave_e2m(.din(next_cansave_e[2:0]), .clk(clk), .q(next_cansave_m[2:0]),
                           .se(se), .si(), .so());
   assign cansave_wen_valid_m = cansave_wen_m;
   dff_s cansave_wen_m2w(.din(cansave_wen_valid_m), .clk(clk), .q(rml_cansave_wen_w),
                       .se(se), .si(), .so());
   dff_s #(3) next_cansave_m2w(.din(next_cansave_m[2:0]), .clk(clk), .q(next_cansave_w[2:0]),
                           .se(se), .si(), .so());
   assign cansave_wen_w = (rml_cansave_wen_w | ecl_rml_cansave_wen_w) & ~rml_kill_w;

   ///////////////////////////////
   // Canrestore logic
   ///////////////////////////////
   assign canrestore_wen_e = ((save_e & ~cansave_is0_e & ~rml_ecl_clean_window_e) |
                              ifu_exu_restored_e |
                              (restore_e & ~canrestore_is0_e) |
                              (ifu_exu_saved_e & otherwin_is0_e));
   sparc_exu_rml_inc3 canrestore_inc(.dout(rml_next_canrestore_e[2:0]),
                                     .din(rml_ecl_canrestore_e[2:0]),
                                     .inc(canrestore_inc_e));
   assign canrestore_inc_e = ifu_exu_restored_e | save_e;
   
   mux2ds #(3) next_canrestore_mux(.dout(next_canrestore_e[2:0]),
                                    .in0(ecl_rml_xor_data_e[2:0]),
                                    .in1(rml_next_canrestore_e[2:0]),
                                    .sel0(~canrestore_wen_e),
                                    .sel1(canrestore_wen_e));
   dff_s canrestore_wen_e2m(.din(canrestore_wen_e), .clk(clk), .q(canrestore_wen_m),
                       .se(se), .si(), .so());
   dff_s #(3) next_canrestore_e2m(.din(next_canrestore_e[2:0]), .clk(clk), .q(next_canrestore_m[2:0]),
                           .se(se), .si(), .so());
   assign canrestore_wen_valid_m = canrestore_wen_m;
   dff_s canrestore_wen_m2w(.din(canrestore_wen_valid_m), .clk(clk), .q(rml_canrestore_wen_w),
                       .se(se), .si(), .so());
   dff_s #(3) next_canrestore_m2w(.din(next_canrestore_m[2:0]), .clk(clk), .q(next_canrestore_w[2:0]),
                           .se(se), .si(), .so());
   assign canrestore_wen_w = (rml_canrestore_wen_w | ecl_rml_canrestore_wen_w) & ~rml_kill_w;

   ///////////////////////////////
   // Otherwin logic
   ///////////////////////////////
   // Decrements on saved or restored if otherwin != 0
   assign otherwin_wen_e = ((ifu_exu_saved_e | ifu_exu_restored_e) 
                            & ~otherwin_is0_e);
   assign rml_next_otherwin_e[2] = ((rml_ecl_otherwin_e[2] & rml_ecl_otherwin_e[1]) |
                                (rml_ecl_otherwin_e[2] & rml_ecl_otherwin_e[0]));
   assign rml_next_otherwin_e[1] = rml_ecl_otherwin_e[1] ^ ~rml_ecl_otherwin_e[0];
   assign rml_next_otherwin_e[0] = ~rml_ecl_otherwin_e[0];

   mux2ds #(3) next_otherwin_mux(.dout(next_otherwin_e[2:0]),
                               .in0(ecl_rml_xor_data_e[2:0]),
                               .in1(rml_next_otherwin_e[2:0]),
                               .sel0(~otherwin_wen_e),
                               .sel1(otherwin_wen_e));
   dff_s otherwin_wen_e2m(.din(otherwin_wen_e), .clk(clk), .q(otherwin_wen_m),
                       .se(se), .si(), .so());
   dff_s #(3) next_otherwin_e2m(.din(next_otherwin_e[2:0]), .clk(clk), .q(next_otherwin_m[2:0]),
                           .se(se), .si(), .so());
   assign otherwin_wen_valid_m = otherwin_wen_m;
   dff_s otherwin_wen_m2w(.din(otherwin_wen_valid_m), .clk(clk), .q(rml_otherwin_wen_w),
                       .se(se), .si(), .so());
   dff_s #(3) next_otherwin_m2w(.din(next_otherwin_m[2:0]), .clk(clk), .q(next_otherwin_w[2:0]),
                           .se(se), .si(), .so());
   assign otherwin_wen_w = (rml_otherwin_wen_w | ecl_rml_otherwin_wen_w) & ~rml_kill_w;

   ///////////////////////////////
   // Cleanwin logic
   ///////////////////////////////
   // increments on restored if cleanwin != 7
   assign cleanwin_wen_e = (ifu_exu_restored_e &
                            ~(rml_ecl_cleanwin_e[2] & rml_ecl_cleanwin_e[1] 
                              & rml_ecl_cleanwin_e[0]));
   assign rml_next_cleanwin_e[2] = ((~rml_ecl_cleanwin_e[2] & rml_ecl_cleanwin_e[1] 
                                 & rml_ecl_cleanwin_e[0]) | rml_ecl_cleanwin_e[2]);
   assign rml_next_cleanwin_e[1] = rml_ecl_cleanwin_e[1] ^ rml_ecl_cleanwin_e[0];
   assign rml_next_cleanwin_e[0] = ~rml_ecl_cleanwin_e[0];
   
   mux2ds #(3) next_cleanwin_mux(.dout(next_cleanwin_e[2:0]),
                                  .in0(ecl_rml_xor_data_e[2:0]),
                                  .in1(rml_next_cleanwin_e[2:0]),
                                  .sel0(~cleanwin_wen_e),
                                  .sel1(cleanwin_wen_e));
   dff_s cleanwin_wen_e2m(.din(cleanwin_wen_e), .clk(clk), .q(cleanwin_wen_m),
                       .se(se), .si(), .so());
   dff_s #(3) next_cleanwin_e2m(.din(next_cleanwin_e[2:0]), .clk(clk), .q(next_cleanwin_m[2:0]),
                           .se(se), .si(), .so());
   assign cleanwin_wen_valid_m = cleanwin_wen_m;
   dff_s cleanwin_wen_m2w(.din(cleanwin_wen_valid_m), .clk(clk), .q(rml_cleanwin_wen_w),
                       .se(se), .si(), .so());
   dff_s #(3) next_cleanwin_m2w(.din(next_cleanwin_m[2:0]), .clk(clk), .q(next_cleanwin_w[2:0]),
                           .se(se), .si(), .so());
   assign cleanwin_wen_w = (rml_cleanwin_wen_w | ecl_rml_cleanwin_wen_w) & ~rml_kill_w;

   ///////////////////////////////
   // WSTATE logic
   ///////////////////////////////
   assign wstate_wen_w = ecl_rml_wstate_wen_w & ~rml_kill_w;

   ///////////////////////////////
   // Storage of other WMRs
   ///////////////////////////////
   sparc_exu_reg  cansave_reg(.clk(clk), .se(se),
                                .data_out(rml_ecl_cansave_d[2:0]), .thr_out(thr_d[3:0]), 
                                .thr_w(ecl_rml_thr_w[3:0]),
                              .wen_w(cansave_wen_w), .data_in_w(next_cansave_w[2:0]));
   dff_s #(3) cansave_d2e(.din(rml_ecl_cansave_d[2:0]), .clk(clk), .q(rml_ecl_cansave_e[2:0]), .se(se),
                  .si(), .so());
   sparc_exu_reg  canrestore_reg(.clk(clk), .se(se),
                                   .data_out(rml_ecl_canrestore_d[2:0]), .thr_out(thr_d[3:0]),
                                   .thr_w(ecl_rml_thr_w[3:0]),
                                   .wen_w(canrestore_wen_w),
                                   .data_in_w(next_canrestore_w[2:0]));
   dff_s #(3) canrestore_d2e(.din(rml_ecl_canrestore_d[2:0]), .clk(clk), .q(rml_ecl_canrestore_e[2:0]),
                         .se(se), .si(), .so());
   sparc_exu_reg  otherwin_reg(.clk(clk), .se(se),
                                 .data_out(rml_ecl_otherwin_d[2:0]), .thr_out(thr_d[3:0]),
                                 .thr_w(ecl_rml_thr_w[3:0]),
                                 .wen_w(otherwin_wen_w), .data_in_w(next_otherwin_w[2:0]));
   dff_s #(3) otherwin_d2e(.din(rml_ecl_otherwin_d[2:0]), .clk(clk), .q(rml_ecl_otherwin_e[2:0]),
                       .se(se), .si(), .so());
   sparc_exu_reg  cleanwin_reg(.clk(clk), .se(se),
                                 .data_out(rml_ecl_cleanwin_d[2:0]), .thr_out(thr_d[3:0]),
                                 .thr_w(ecl_rml_thr_w[3:0]),
                                 .wen_w(cleanwin_wen_w), .data_in_w(next_cleanwin_w[2:0]));
   dff_s #(3) cleanwin_d2e(.din(rml_ecl_cleanwin_d[2:0]), .clk(clk), .q(rml_ecl_cleanwin_e[2:0]),
                       .se(se), .si(), .so());
   sparc_exu_reg hi_wstate_reg(.clk(clk), .se(se),
                               .data_out(rml_ecl_wstate_d[5:3]), .thr_out(thr_d[3:0]),
                               .thr_w(ecl_rml_thr_w[3:0]),
                               .wen_w(wstate_wen_w), 
                               .data_in_w(exu_tlu_wsr_data_w[5:3]));
   sparc_exu_reg lo_wstate_reg(.clk(clk), .se(se),
                               .data_out(rml_ecl_wstate_d[2:0]), .thr_out(thr_d[3:0]),
                               .thr_w(ecl_rml_thr_w[3:0]),
                               .wen_w(wstate_wen_w), 
                               .data_in_w(exu_tlu_wsr_data_w[2:0]));


   /////////////////////////////////
   // Alternate Globals control
   //----------------------------
   /////////////////////////////////
   assign rml_irf_new_agp[1:0] = tlu_exu_agp[1:0];
   assign agp_tid[1:0] = tlu_exu_agp_tid[1:0];

 // Use two threads unless this is defined

   //  Output selection for current agp
   mux2ds #(2) mux_agp_out1(.dout(rml_irf_old_agp[1:0]),
                            .sel0(agp_thr[0]),
                            .sel1(agp_thr[1]),
                            .in0(agp_thr0[1:0]),
                            .in1(agp_thr1[1:0]));

   //////////////////////////////////////
   //  Storage of agp
   //////////////////////////////////////

   // enable input for each thread
   assign        agp_wen_thr0_w = (agp_thr[0] & agp_wen) | reset;
   assign        agp_wen_thr1_w = (agp_thr[1] & agp_wen) | reset;

   // mux between new and current value
   mux2ds #(2) agp_next0_mux(.dout(agp_thr0_next[1:0]),
                               .in0(agp_thr0[1:0]),
                               .in1(new_agp[1:0]),
                               .sel0(~agp_wen_thr0_w),
                               .sel1(agp_wen_thr0_w));
   mux2ds #(2) agp_next1_mux(.dout(agp_thr1_next[1:0]),
                               .in0(agp_thr1[1:0]),
                               .in1(new_agp[1:0]),
                               .sel0(~agp_wen_thr1_w),
                               .sel1(agp_wen_thr1_w));

   // store new value
   dff_s #(2) dff_agp_thr0(.din(agp_thr0_next[1:0]), .clk(clk), .q(agp_thr0[1:0]),
                       .se(se), .si(), .so());
   dff_s #(2) dff_agp_thr1(.din(agp_thr1_next[1:0]), .clk(clk), .q(agp_thr1[1:0]),
                       .se(se), .si(), .so());

   // generation of controls
   assign        agp_wen = tlu_exu_agp_swap;
   assign        rml_irf_swap_global = agp_wen;
   assign        rml_irf_global_tid[1:0] = agp_tid[1:0];

   // decode tids
   assign        agp_thr[0] = ~agp_tid[1] & ~agp_tid[0];
   assign        agp_thr[1] = ~agp_tid[1] & agp_tid[0];

   // Decode agp input
   assign new_agp[1:0] = rml_irf_new_agp[1:0] | {2{reset}};

   // send current global level to ecl for error logging
   assign rml_ecl_gl_e[1:0] = ((tid_e[1:0] == 2'b00)? agp_thr0[1:0]:
                                                      agp_thr1[1:0]);


















































































































































































































































 // `ifndef CONFIG_NUM_THREADS
   
endmodule // sparc_exu_rml
// Modified by Princeton University on June 9th, 2015
// ========== Copyright Header Begin ==========================================
// 
// OpenSPARC T1 Processor File: sparc_exu_rml_cwp.v
// Copyright (c) 2006 Sun Microsystems, Inc.  All Rights Reserved.
// DO NOT ALTER OR REMOVE COPYRIGHT NOTICES.
// 
// The above named program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public
// License version 2 as published by the Free Software Foundation.
// 
// The above named program is distributed in the hope that it will be 
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// General Public License for more details.
// 
// You should have received a copy of the GNU General Public
// License along with this work; if not, write to the Free Software
// Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA 02110-1301, USA.
// 
// ========== Copyright Header End ============================================
////////////////////////////////////////////////////////////////////////
/*
//  Module Name: sparc_exu_rml_cwp
//	Description: Register management logic.  Contains CWP, CANSAVE, CANRESTORE
//		and other window management registers.  Generates RF related traps
//  		and switches the global registers to alternate globals.  All the registers
//		are written in the W stage (there is no bypassing so they must
//		swap out) and will either get a new value generated by a window management
//		Instruction or by a WRPS instruction.  The following traps can be generated:
//			Fill: restore with canrestore == 0
//			clean_window: save with cleanwin-canrestore == 0
//			spill: flushw with cansave != nwindows -2 or
//				save with cansave == 0
//		It is assumed that the contents of the new window will get squashed
//		on a clean_window or fill trap so the save or restore gets executed
//		normally.  Spill traps or WRCWPs mean that all 16 windowed registers
//		must be saved and restored (a 4 cycle operation).
*/
module sparc_exu_rml_cwp (/*AUTOARG*/
   // Outputs
   rml_ecl_cwp_d, rml_ecl_cwp_e, exu_tlu_cwp0_w, exu_tlu_cwp1_w, 
   exu_tlu_cwp2_w, exu_tlu_cwp3_w, rml_irf_cwpswap_tid_e, old_cwp_e, 
   new_cwp_e, swap_locals_ins, swap_outs, exu_tlu_spill, 
   exu_tlu_spill_wtype, exu_tlu_spill_other, exu_tlu_spill_tid, 
   rml_ecl_swap_done, exu_tlu_cwp_cmplt, exu_tlu_cwp_cmplt_tid, 
   exu_tlu_cwp_retry, oddwin_w, 
   // Inputs
   clk, se, reset, rst_tri_en, rml_ecl_wtype_e, rml_ecl_other_e, 
   exu_tlu_spill_e, tlu_exu_cwpccr_update_m, tlu_exu_cwp_retry_m, 
   tlu_exu_cwp_m, thr_d, ecl_rml_thr_m, ecl_rml_thr_w, tid_e, 
   next_cwp_w, next_cwp_e, cwp_wen_w, save_e, restore_e, 
   ifu_exu_flushw_e, ecl_rml_cwp_wen_e, full_swap_e, rml_kill_w
   ) ;
   input clk;
   input se;
   input reset;
   input rst_tri_en;
   input [2:0] rml_ecl_wtype_e;
   input       rml_ecl_other_e;
   input       exu_tlu_spill_e;
   input       tlu_exu_cwpccr_update_m;
   input       tlu_exu_cwp_retry_m;
   input [2:0] tlu_exu_cwp_m; // for switching cwp on return from trap
   input [3:0] thr_d;
   input [3:0] ecl_rml_thr_m;
   input [3:0] ecl_rml_thr_w;
   input [1:0] tid_e;
   input [2:0] next_cwp_w;
   input [2:0] next_cwp_e;
   input       cwp_wen_w;
   input       save_e;
   input       restore_e;
   input       ifu_exu_flushw_e;
   input       ecl_rml_cwp_wen_e;
   input       full_swap_e;
   input       rml_kill_w;

   output [2:0] rml_ecl_cwp_d;
   output [2:0] rml_ecl_cwp_e;
   output [2:0] exu_tlu_cwp0_w;
   output [2:0] exu_tlu_cwp1_w;
   output [2:0] exu_tlu_cwp2_w;
   output [2:0] exu_tlu_cwp3_w;
   output [1:0] rml_irf_cwpswap_tid_e;
   output [2:0] old_cwp_e;
   output [2:0] new_cwp_e;
   output       swap_locals_ins;
   output       swap_outs;
   output      exu_tlu_spill;
   output [2:0] exu_tlu_spill_wtype;
   output       exu_tlu_spill_other;
   output [1:0] exu_tlu_spill_tid;
   output [3:0] rml_ecl_swap_done;
   output       exu_tlu_cwp_cmplt;
   output [1:0] exu_tlu_cwp_cmplt_tid;
   output       exu_tlu_cwp_retry;
   output [3:0] oddwin_w;
   
   wire         can_swap;
   wire         swapping;
   wire         just_swapped;
   wire         full_swap_m;
   wire         full_swap_w;
   wire [3:0]   swap_done_next_cycle;
   wire [3:0] swap_sel_input;
   wire [3:0] swap_sel_tlu;
   wire [3:0] swap_keep_value;
   wire [2:0]  trap_old_cwp_m;
   wire   tlu_cwp_no_change;
   wire [2:0] tlu_cwp_xor;
   wire   cwp_cmplt_next;
   wire [1:0] cwp_cmplt_tid_next;
   wire       cwp_retry_next;
   wire   cwp_fastcmplt_m;
   wire   cwp_fastcmplt_w;
   wire   cwpccr_update_w;
   wire   valid_tlu_swap_w;
   wire [2:0] tlu_exu_cwp_w;
   wire       tlu_exu_cwp_retry_w;

   wire [3:0] swap_thr;
   wire [1:0] swap_tid;
   wire [3:0] swap_req_vec;
   wire       kill_swap_slot_w;
   wire [3:0] thr_e;
   
   wire [1:0] swap_slot0_state;
   wire [1:0] swap_slot1_state;
   wire [1:0] swap_slot2_state;
   wire [1:0] swap_slot3_state;
   wire [1:0] swap_slot0_state_valid;
   wire [1:0] swap_slot1_state_valid;
   wire [1:0] swap_slot2_state_valid;
   wire [1:0] swap_slot3_state_valid;
   wire [1:0] next_slot0_state;
   wire [1:0] next_slot1_state;
   wire [1:0] next_slot2_state;
   wire [1:0] next_slot3_state;
   wire [3:0] swap_keep_state;
   wire [3:0] swap_next_state;
   wire [1:0] swap_state;

   wire [3:0] next_swap_thr;
   wire [12:0] swap_data;
   wire [12:0] tlu_swap_data;
   wire [12:0] swap_input_data;
   wire [12:0] next_slot0_data;
   wire [12:0] next_slot1_data;
   wire [12:0] next_slot2_data;
   wire [12:0] next_slot3_data;
   wire [12:0] swap_slot0_data;
   wire [12:0] swap_slot1_data;
   wire [12:0] swap_slot2_data;
   wire [12:0] swap_slot3_data;

   wire        new_cwp_sel_swap;
   wire [2:0]  old_swap_cwp;
   wire [2:0]  new_swap_cwp;

   
   // wires for cwp register
   wire [2:0]   cwp_thr0;
   wire [2:0]   cwp_thr1;
   wire [2:0]   cwp_thr2;
   wire [2:0]   cwp_thr3;
   wire [2:0]   cwp_thr0_next;
   wire [2:0]   cwp_thr1_next;
   wire [2:0]   cwp_thr2_next;
   wire [2:0]   cwp_thr3_next;
   wire          cwp_wen_thr0_w;
   wire          cwp_wen_thr1_w;
   wire          cwp_wen_thr2_w;
   wire          cwp_wen_thr3_w;
   wire [3:0]    cwp_wen_tlu_w;
   wire [3:0] cwp_wen_spill;
   wire [2:0] spill_cwp;
   wire [3:0]    cwp_wen_l;
   wire [2:0]    old_cwp_w;
   wire        spill_next;
   wire [1:0]  spill_tid_next;
   wire        spill_other_next;
   wire [2:0]  spill_wtype_next;

   // decode thr_e
   assign        thr_e[0] = ~tid_e[1] & ~tid_e[0];
   assign        thr_e[1] = ~tid_e[1] & tid_e[0];
   assign        thr_e[2] = tid_e[1] & ~tid_e[0];
   assign        thr_e[3] = tid_e[1] & tid_e[0];
   
   /////////////////////////////////
   // CWP output to IRF
   /////////////////////////////////
   // Output current_d thr on saves or restores
   mux2ds #(2) irf_thr_mux(.dout(rml_irf_cwpswap_tid_e[1:0]),
                              .in0(tid_e[1:0]),
                              .in1(swap_tid[1:0]),
                              .sel0(~can_swap),
                              .sel1(can_swap));
   // Output cwp_e for save, restore, flushw
   // and swap_cwp from queue for swap restores (default)
   // Need to have an incremented cwp for swap of outs
   assign        old_swap_cwp[2:0] = swap_data[2:0];
   assign        new_swap_cwp[2:0] = swap_data[5:3];
   
   assign        new_cwp_sel_swap = can_swap;

   assign new_cwp_e[2:0] = (new_cwp_sel_swap)?  new_swap_cwp[2:0]: next_cwp_e[2:0];
   assign old_cwp_e[2:0] = (new_cwp_sel_swap)?  old_swap_cwp[2:0]: rml_ecl_cwp_e[2:0];
   
 
   /////////////////////////////////
   // CWP register
   /////////////////////////////////
   assign exu_tlu_cwp0_w[2:0] = cwp_thr0[2:0];
   assign exu_tlu_cwp1_w[2:0] = cwp_thr1[2:0];
   assign exu_tlu_cwp2_w[2:0] = cwp_thr2[2:0];
   assign exu_tlu_cwp3_w[2:0] = cwp_thr3[2:0];
   
   mux4ds #(3) mux_cwp_old_w(.dout(old_cwp_w[2:0]), .sel0(ecl_rml_thr_w[0]),
                             .sel1(ecl_rml_thr_w[1]), .sel2(ecl_rml_thr_w[2]),
                             .sel3(ecl_rml_thr_w[3]), .in0(cwp_thr0[2:0]),
                             .in1(cwp_thr1[2:0]), .in2(cwp_thr2[2:0]),
                             .in3(cwp_thr3[2:0]));

   //  Output selection for reg
   mux4ds #(3) mux_cwp_out_d(.dout(rml_ecl_cwp_d[2:0]), .sel0(thr_d[0]),
                             .sel1(thr_d[1]), .sel2(thr_d[2]),
                             .sel3(thr_d[3]), .in0(cwp_thr0[2:0]),
                             .in1(cwp_thr1[2:0]), .in2(cwp_thr2[2:0]),
                             .in3(cwp_thr3[2:0]));
   mux4ds #(3) mux_cwp_out_e(.dout(rml_ecl_cwp_e[2:0]), .sel0(thr_e[0]),
                             .sel1(thr_e[1]), .sel2(thr_e[2]),
                             .sel3(thr_e[3]), .in0(cwp_thr0[2:0]),
                             .in1(cwp_thr1[2:0]), .in2(cwp_thr2[2:0]),
                             .in3(cwp_thr3[2:0]));
   mux4ds #(3) mux_cwp_trap(.dout(trap_old_cwp_m[2:0]), .sel0(ecl_rml_thr_m[0]),
                             .sel1(ecl_rml_thr_m[1]), .sel2(ecl_rml_thr_m[2]),
                             .sel3(ecl_rml_thr_m[3]), .in0(cwp_thr0[2:0]),
                             .in1(cwp_thr1[2:0]), .in2(cwp_thr2[2:0]),
                             .in3(cwp_thr3[2:0]));

   //////////////////////////////////////
   //  Storage of cwp
   //////////////////////////////////////
   // enable input for each thread
   assign     cwp_wen_spill[3:0] = swap_thr[3:0] & {4{spill_next}};
   assign        cwp_wen_thr0_w = ((ecl_rml_thr_w[0] & cwp_wen_w)) & ~cwp_wen_spill[0];
   assign        cwp_wen_thr1_w = ((ecl_rml_thr_w[1] & cwp_wen_w)) & ~cwp_wen_spill[1];
   assign        cwp_wen_thr2_w = ((ecl_rml_thr_w[2] & cwp_wen_w)) & ~cwp_wen_spill[2];
   assign        cwp_wen_thr3_w = ((ecl_rml_thr_w[3] & cwp_wen_w)) & ~cwp_wen_spill[3];
   assign        cwp_wen_tlu_w[3:0] = ecl_rml_thr_w[3:0] & {4{valid_tlu_swap_w}} & ~cwp_wen_spill &
                                       {~cwp_wen_thr3_w,~cwp_wen_thr2_w,~cwp_wen_thr1_w,~cwp_wen_thr0_w};
   assign        cwp_wen_l[3:0] = ~(cwp_wen_tlu_w[3:0] | cwp_wen_spill[3:0] |
                                    {cwp_wen_thr3_w,cwp_wen_thr2_w, cwp_wen_thr1_w,cwp_wen_thr0_w});

   // oddwin_w is the new value of cwp[0]
   assign        oddwin_w[3:0] = {cwp_thr3_next[0],cwp_thr2_next[0],cwp_thr1_next[0],cwp_thr0_next[0]};
   // mux between new and current value
   mux4ds #(3) cwp_next0_mux(.dout(cwp_thr0_next[2:0]),
                             .in0(cwp_thr0[2:0]),
                             .in1(next_cwp_w[2:0]),
                             .in2(tlu_exu_cwp_w[2:0]),
                             .in3(spill_cwp[2:0]),
                             .sel0(cwp_wen_l[0]),
                             .sel1(cwp_wen_thr0_w),
                             .sel2(cwp_wen_tlu_w[0]),
                             .sel3(cwp_wen_spill[0]));
   mux4ds #(3) cwp_next1_mux(.dout(cwp_thr1_next[2:0]),
                             .in0(cwp_thr1[2:0]),
                             .in1(next_cwp_w[2:0]),
                             .in2(tlu_exu_cwp_w[2:0]),
                             .in3(spill_cwp[2:0]),
                             .sel0(cwp_wen_l[1]),
                             .sel1(cwp_wen_thr1_w),
                             .sel2(cwp_wen_tlu_w[1]),
                             .sel3(cwp_wen_spill[1]));
   mux4ds #(3) cwp_next2_mux(.dout(cwp_thr2_next[2:0]),
                             .in0(cwp_thr2[2:0]),
                             .in1(next_cwp_w[2:0]),
                             .in2(tlu_exu_cwp_w[2:0]),
                             .in3(spill_cwp[2:0]),
                             .sel0(cwp_wen_l[2]),
                             .sel1(cwp_wen_thr2_w),
                             .sel2(cwp_wen_tlu_w[2]),
                             .sel3(cwp_wen_spill[2]));
   mux4ds #(3) cwp_next3_mux(.dout(cwp_thr3_next[2:0]),
                             .in0(cwp_thr3[2:0]),
                             .in1(next_cwp_w[2:0]),
                             .in2(tlu_exu_cwp_w[2:0]),
                             .in3(spill_cwp[2:0]),
                             .sel0(cwp_wen_l[3]),
                             .sel1(cwp_wen_thr3_w),
                             .sel2(cwp_wen_tlu_w[3]),
                             .sel3(cwp_wen_spill[3]));

   // store new value
   dff_s #(3) dff_cwp_thr0(.din(cwp_thr0_next[2:0]), .clk(clk), .q(cwp_thr0[2:0]),
                       .se(se), .si(), .so());
   dff_s #(3) dff_cwp_thr1(.din(cwp_thr1_next[2:0]), .clk(clk), .q(cwp_thr1[2:0]),
                       .se(se), .si(), .so());
   dff_s #(3) dff_cwp_thr2(.din(cwp_thr2_next[2:0]), .clk(clk), .q(cwp_thr2[2:0]),
                       .se(se), .si(), .so());
   dff_s #(3) dff_cwp_thr3(.din(cwp_thr3_next[2:0]), .clk(clk), .q(cwp_thr3[2:0]),
                       .se(se), .si(), .so());



   ////////////////////////////////////////////
   // Queue for full window swaps
   ////////////////////////////////////////////
   // A full swap of the current window requires a 2 cycle operation.
   // Each cycle must make sure that
   // there isn't another instruction trying to save or restore on top of it.
   // The same thread also cannot issue a swap to irf in back-to-back cycles.
   // Data is stored as follows:
   //   2:0 - CWP
   //   5:3 - NewCWP
   //   6   - !WRCWP/SPILL
   //   7   - Trap return
   //   8   - OTHER (for spill trap)
   //   11:9- WTYPE (for spill trap)
   //		12  - Retry (for trap return)
   dff_s full_swap_e2m(.din(full_swap_e), .clk(clk), .q(full_swap_m), .se(se), .si(), .so());
   dff_s full_swap_m2w(.din(full_swap_m), .clk(clk), .q(full_swap_w), .se(se), .si(), .so());
   assign     swap_input_data = {1'b0, rml_ecl_wtype_e[2:0], rml_ecl_other_e, 1'b0, exu_tlu_spill_e, 
                                 next_cwp_e[2:0],rml_ecl_cwp_e[2:0]};
   assign     tlu_swap_data = {tlu_exu_cwp_retry_w, 4'b0, 1'b1, 1'b0, tlu_exu_cwp_w[2:0], old_cwp_w[2:0]};


   assign     swap_sel_input[3:0] = thr_e[3:0] & {4{full_swap_e}};
   assign     swap_sel_tlu[3:0] = ecl_rml_thr_w[3:0] & {4{cwpccr_update_w}} 
                                    & ~swap_sel_input[3:0];
   assign     swap_keep_value[3:0] = ~(swap_sel_tlu[3:0] | swap_sel_input[3:0]);
   assign     swap_keep_state[3:0] = ~(swap_sel_tlu[3:0] | swap_sel_input[3:0]) & 
                                        ~(swap_thr[3:0] & {4{can_swap}});
   assign     swap_next_state[3:0] = ~(swap_sel_tlu[3:0] | swap_sel_input[3:0]) 
                                         & (swap_thr[3:0] & {4{can_swap}});
   mux3ds #(13) slot0_data_mux(.dout(next_slot0_data[12:0]),
                               .in0(swap_input_data[12:0]),
                               .in1(tlu_swap_data[12:0]),
                               .in2(swap_slot0_data[12:0]),
                               .sel0(swap_sel_input[0]),
                               .sel1(swap_sel_tlu[0]),
                               .sel2(swap_keep_value[0]));
   mux3ds #(13) slot1_data_mux(.dout(next_slot1_data[12:0]),
                               .in0(swap_input_data[12:0]),
                               .in1(tlu_swap_data[12:0]),
                               .in2(swap_slot1_data[12:0]),
                               .sel0(swap_sel_input[1]),
                               .sel1(swap_sel_tlu[1]),
                               .sel2(swap_keep_value[1]));
   mux3ds #(13) slot2_data_mux(.dout(next_slot2_data[12:0]),
                               .in0(swap_input_data[12:0]),
                               .in1(tlu_swap_data[12:0]),
                               .in2(swap_slot2_data[12:0]),
                               .sel0(swap_sel_input[2]),
                               .sel1(swap_sel_tlu[2]),
                               .sel2(swap_keep_value[2]));
   mux3ds #(13) slot3_data_mux(.dout(next_slot3_data[12:0]),
                               .in0(swap_input_data[12:0]),
                               .in1(tlu_swap_data[12:0]),
                               .in2(swap_slot3_data[12:0]),
                               .sel0(swap_sel_input[3]),
                               .sel1(swap_sel_tlu[3]),
                               .sel2(swap_keep_value[3]));

   // Muxes for slot state.
   // There are 2 possible states:
   // No swap done (01)
   // Swap locals/ins done (10)
   mux4ds #(2) slot0_state_mux(.dout(next_slot0_state[1:0]),
                               .in0(2'b10),
                               .in1({1'b0, valid_tlu_swap_w}),
                               .in2(swap_slot0_state_valid[1:0]),
                               .in3({swap_slot0_state_valid[0], 1'b0}),
                               .sel0(swap_sel_input[0]),
                               .sel1(swap_sel_tlu[0]),
                               .sel2(swap_keep_state[0]),
                               .sel3(swap_next_state[0]));
   mux4ds #(2) slot1_state_mux(.dout(next_slot1_state[1:0]),
                               .in0(2'b10),
                               .in1({1'b0, valid_tlu_swap_w}),
                               .in2(swap_slot1_state_valid[1:0]),
                               .in3({swap_slot1_state_valid[0], 1'b0}),
                               .sel0(swap_sel_input[1]),
                               .sel1(swap_sel_tlu[1]),
                               .sel2(swap_keep_state[1]),
                               .sel3(swap_next_state[1]));
   mux4ds #(2) slot2_state_mux(.dout(next_slot2_state[1:0]),
                               .in0(2'b10),
                               .in1({1'b0, valid_tlu_swap_w}),
                               .in2(swap_slot2_state_valid[1:0]),
                               .in3({swap_slot2_state_valid[0], 1'b0}),
                               .sel0(swap_sel_input[2]),
                               .sel1(swap_sel_tlu[2]),
                               .sel2(swap_keep_state[2]),
                               .sel3(swap_next_state[2]));
   mux4ds #(2) slot3_state_mux(.dout(next_slot3_state[1:0]),
                               .in0(2'b10),
                               .in1({1'b0, valid_tlu_swap_w}),
                               .in2(swap_slot3_state_valid[1:0]),
                               .in3({swap_slot3_state_valid[0], 1'b0}),
                               .sel0(swap_sel_input[3]),
                               .sel1(swap_sel_tlu[3]),
                               .sel2(swap_keep_state[3]),
                               .sel3(swap_next_state[3]));

   // The kill is only assessed in w1 because back to back swaps are not allowed.
   // This means that a swap cannot start in the M or W stage.
   assign     kill_swap_slot_w = rml_kill_w & full_swap_w;

   assign     swap_slot0_state_valid[1:0] = {(swap_slot0_state[1] & ~(kill_swap_slot_w & ecl_rml_thr_w[0])),
                                             (swap_slot0_state[0])};
   assign     swap_slot1_state_valid[1:0] = {(swap_slot1_state[1] & ~(kill_swap_slot_w & ecl_rml_thr_w[1])),
                                             (swap_slot1_state[0])};
   assign     swap_slot2_state_valid[1:0] = {(swap_slot2_state[1] & ~(kill_swap_slot_w & ecl_rml_thr_w[2])),
                                             (swap_slot2_state[0])};
   assign     swap_slot3_state_valid[1:0] = {(swap_slot3_state[1] & ~(kill_swap_slot_w & ecl_rml_thr_w[3])),
                                             (swap_slot3_state[0])};
   
   // Flops for cwp_swap data
   dffr_s #(15) slot0_data_dff(.din({next_slot0_state[1:0], next_slot0_data[12:0]}), .clk(clk), 
                            .q({swap_slot0_state[1:0], swap_slot0_data[12:0]}), .rst(reset),
                            .se(se), .si(), .so());
   dffr_s #(15) slot1_data_dff(.din({next_slot1_state[1:0], next_slot1_data[12:0]}), .clk(clk), 
                            .q({swap_slot1_state[1:0], swap_slot1_data[12:0]}), .rst(reset),
                            .se(se), .si(), .so());
   dffr_s #(15) slot2_data_dff(.din({next_slot2_state[1:0], next_slot2_data[12:0]}), .clk(clk), 
                            .q({swap_slot2_state[1:0], swap_slot2_data[12:0]}), .rst(reset),
                            .se(se), .si(), .so());
   dffr_s #(15) slot3_data_dff(.din({next_slot3_state[1:0], next_slot3_data[12:0]}), .clk(clk), 
                            .q({swap_slot3_state[1:0], swap_slot3_data[12:0]}), .rst(reset),
                            .se(se), .si(), .so());

   ////////////////////////////
   // Control for queue output
   //	==========================
   //	The queue results go into a flop
   //	so that they can meet timing.
   ////////////////////////////
   assign     swap_req_vec[0] = (swap_slot0_state[1] | swap_slot0_state[0]);
   assign     swap_req_vec[1] = (swap_slot1_state[1] | swap_slot1_state[0]);
   assign     swap_req_vec[2] = (swap_slot2_state[1] | swap_slot2_state[0]);
   assign     swap_req_vec[3] = (swap_slot3_state[1] | swap_slot3_state[0]);
   
   sparc_exu_rndrob cwp_output_queue(// Outputs
                                     .grant_vec(next_swap_thr[3:0]),
                                     // Inputs
                                     .clk(clk),
                                     .reset(reset),
                                     .se(se),
                                     .req_vec(swap_req_vec[3:0]),
                                     .advance(can_swap));
   dff_s #(4) dff_swap_thr(.din(next_swap_thr[3:0]), .clk(clk), .q(swap_thr[3:0]),
                         .se(se), .si(), .so());
   assign     swap_tid[1] = swap_thr[3] | swap_thr[2];
   assign     swap_tid[0] = swap_thr[3] | swap_thr[1];

   // make selects one hot
   wire [3:0] swap_sel;
   assign swap_sel[0] = ~(swap_thr[1] | swap_thr[2] | swap_thr[3]) | rst_tri_en;
   assign swap_sel[3:1] = swap_thr[3:1] & {3{~rst_tri_en}};

   mux4ds #(15) cwp_output_mux(.dout({swap_state[1:0], swap_data[12:0]}),
                               .in0({swap_slot0_state[1:0], swap_slot0_data[12:0]}),
                               .in1({swap_slot1_state[1:0], swap_slot1_data[12:0]}),
                               .in2({swap_slot2_state[1:0], swap_slot2_data[12:0]}),
                               .in3({swap_slot3_state[1:0], swap_slot3_data[12:0]}),
                               .sel0(swap_sel[0]),
                               .sel1(swap_sel[1]),
                               .sel2(swap_sel[2]),
                               .sel3(swap_sel[3]));

   // To prevent back to back swap requests on the same thread, the queue cannot swap
   // 2 cycles in a row.  Also swaps can't start in M or W to allow flush to be checked
   dffr_s can_swap_flop(.din(swapping), .clk(clk), .q(just_swapped), .rst(reset), .se(se), .si(), .so());
   assign     can_swap = ~(save_e | restore_e | ifu_exu_flushw_e | ecl_rml_cwp_wen_e | just_swapped);
   assign      swap_locals_ins = can_swap & swap_state[0];
   assign      swap_outs = can_swap & swap_state[1];
   assign      swapping = (can_swap & |swap_state[1:0]) | full_swap_e | full_swap_m;

   ///////////////////////////////////
   // Signals for completion of swaps
   ///////////////////////////////////
   assign spill_next = swap_data[6] & ~swap_data[7] & swap_outs;
   assign spill_tid_next[1:0] = swap_tid[1:0];
   //assign exu_tlu_spill_ttype[8:0] = {3'b010, swap_data[8], swap_data[11:9], 2'b00};
   assign spill_other_next = swap_data[8];
   assign spill_wtype_next[2:0] = swap_data[11:9];
   dff_s #(7) spill_dff(.din({spill_next,spill_tid_next[1:0], spill_other_next, spill_wtype_next[2:0]}),
                      .q({exu_tlu_spill,exu_tlu_spill_tid[1:0], exu_tlu_spill_other, exu_tlu_spill_wtype[2:0]}),
                      .clk(clk), .se(se), .si(), .so());
   assign spill_cwp[2:0] = swap_data[5:3];
/* -----\/----- EXCLUDED -----\/-----
   dff_s #(3) spill_cwp_dff(.din(swap_data[5:3]), .clk(clk), .q(spill_cwp[2:0]),
                          .se(se), .si(), .so());
 -----/\----- EXCLUDED -----/\----- */
   assign swap_done_next_cycle[3] = (swap_outs & ~swap_data[6] & ~swap_data[7] &
                                     swap_tid[1] & swap_tid[0]); 
   assign swap_done_next_cycle[2] = (swap_outs & ~swap_data[6] & ~swap_data[7] &
                                     swap_tid[1] & ~swap_tid[0]); 
   assign swap_done_next_cycle[1] = (swap_outs & ~swap_data[6] & ~swap_data[7] &
                                     ~swap_tid[1] & swap_tid[0]); 
   assign swap_done_next_cycle[0] = (swap_outs & ~swap_data[6] & ~swap_data[7] &
                                     ~swap_tid[1] & ~swap_tid[0]); 

   dff_s #(4) swap_done_dff(.din(swap_done_next_cycle[3:0]), .clk(clk),
                        .q(rml_ecl_swap_done[3:0]), .se(se), .si(), .so());

   dff_s #(4) cwp_cmplt_dff(.din({cwp_cmplt_next, cwp_cmplt_tid_next[1:0], cwp_retry_next}),
                          .q({exu_tlu_cwp_cmplt,exu_tlu_cwp_cmplt_tid[1:0], exu_tlu_cwp_retry}),
                          .clk(clk), .si(), .so(), .se(se));
   assign cwp_cmplt_next = swap_outs & swap_data[7];
   assign cwp_cmplt_tid_next[1:0] = swap_tid[1:0];
   assign cwp_retry_next = swap_data[12];

   assign tlu_cwp_xor[2:0] = trap_old_cwp_m[2:0] ^ tlu_exu_cwp_m[2:0];
   assign tlu_cwp_no_change = ~(tlu_cwp_xor[2] | tlu_cwp_xor[1] | tlu_cwp_xor[0]); 
   assign cwp_fastcmplt_m = tlu_exu_cwpccr_update_m & tlu_cwp_no_change;

   dff_s fastcmplt_dff(.din(cwp_fastcmplt_m), .clk(clk),
                     .q(cwp_fastcmplt_w), .se(se), .si(), .so());

   ///////////////////////////////////////////////////////////
   // Pipe along tlu_exu_done/retry so inst_vld can be caught
   ///////////////////////////////////////////////////////////
   dff_s #(5) tlu_data_dff(.q({cwpccr_update_w,tlu_exu_cwp_w[2:0],tlu_exu_cwp_retry_w}),
                         .din({tlu_exu_cwpccr_update_m,tlu_exu_cwp_m[2:0],tlu_exu_cwp_retry_m}),
                         .clk(clk), .se(se), .si(), .so());
   assign valid_tlu_swap_w = cwpccr_update_w & ~rml_kill_w & ~cwp_fastcmplt_w;
   
endmodule // sparc_exu_rml_cwp
// Modified by Princeton University on June 9th, 2015
// ========== Copyright Header Begin ==========================================
// 
// OpenSPARC T1 Processor File: sparc_exu_rml_inc3.v
// Copyright (c) 2006 Sun Microsystems, Inc.  All Rights Reserved.
// DO NOT ALTER OR REMOVE COPYRIGHT NOTICES.
// 
// The above named program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public
// License version 2 as published by the Free Software Foundation.
// 
// The above named program is distributed in the hope that it will be 
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// General Public License for more details.
// 
// You should have received a copy of the GNU General Public
// License along with this work; if not, write to the Free Software
// Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA 02110-1301, USA.
// 
// ========== Copyright Header End ============================================
module sparc_exu_rml_inc3 (/*AUTOARG*/
   // Outputs
   dout, 
   // Inputs
   din, inc
   ) ;
   input [2:0] din;
   input       inc;
   output [2:0] dout;

   assign       dout[2] = ((~din[2] & ~din[1] & ~din[0] & ~inc) |
                           (~din[2] & din[1] & din[0] & inc) |
                           (din[2] & din[1] & ~din[0]) |
                           (din[2] & ~din[1] & inc) |
                           (din[2] & din[0] & ~inc));
   assign dout[1] = ((~din[1] & ~din[0] & ~inc) |
                     (din[1] & ~din[0] & inc) |
                     (~din[1] & din[0] & inc) |
                     (din[1] & din[0] & ~inc));
   assign dout[0] = ~din[0];
   
endmodule // sparc_exu_rml_inc3
// Modified by Princeton University on June 9th, 2015
// ========== Copyright Header Begin ==========================================
// 
// OpenSPARC T1 Processor File: sparc_exu_rndrob.v
// Copyright (c) 2006 Sun Microsystems, Inc.  All Rights Reserved.
// DO NOT ALTER OR REMOVE COPYRIGHT NOTICES.
// 
// The above named program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public
// License version 2 as published by the Free Software Foundation.
// 
// The above named program is distributed in the hope that it will be 
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// General Public License for more details.
// 
// You should have received a copy of the GNU General Public
// License along with this work; if not, write to the Free Software
// Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA 02110-1301, USA.
// 
// ========== Copyright Header End ============================================
////////////////////////////////////////////////////////////////////////
/*
//  Module Name: sparc_exu_rndrob
//  Description:	
//  Round robin scheduler.  Least priority to the last granted
//  customer.  If no requests, the priority remains the same. 
*/
////////////////////////////////////////////////////////////////////////

module sparc_exu_rndrob(/*AUTOARG*/
   // Outputs
   grant_vec, 
   // Inputs
   clk, reset, se, req_vec, advance
   );

   input     clk, reset, se;

   input  [3:0] req_vec;
   input 	advance;
   
   output [3:0] grant_vec;
   
   wire [3:0] 	pv,
		next_pv,
		park_vec;
   
   
   assign  next_pv =  advance ? grant_vec : park_vec;
   
   dffr_s #(4)  park_reg(.din  (next_pv[3:0]),
		    .clk  (clk),
		    .q    (pv[3:0]),
		    .rst  (reset),
		    .se   (se), .si(), .so());

   assign  park_vec = pv;

   // if noone requests, don't advance, otherwise we'll go back to 0
   // and will not be fair to other requestors
   assign grant_vec[0] =  park_vec[3] & req_vec[0] |
		  park_vec[2] & ~req_vec[3] & req_vec[0] |
		  park_vec[1] & ~req_vec[2] & ~req_vec[3] & req_vec[0] |
	          ~req_vec[1] & ~req_vec[2] & ~req_vec[3];
   
   assign grant_vec[1] = park_vec[0] & req_vec[1] |
		  park_vec[3] & ~req_vec[0] & req_vec[1] |
		  park_vec[2] & ~req_vec[3] & ~req_vec[0] & req_vec[1] |
	          req_vec[1] & ~req_vec[2] & ~req_vec[3] & ~req_vec[0];

   assign grant_vec[2] = park_vec[1] & req_vec[2] |
		  park_vec[0] & ~req_vec[1] & req_vec[2] |
		  park_vec[3] & ~req_vec[0] & ~req_vec[1] & req_vec[2] |
		  req_vec[2] & ~req_vec[3] & ~req_vec[0] & ~req_vec[1];

   assign grant_vec[3] = park_vec[2] & req_vec[3] |
		  park_vec[1] & ~req_vec[2] & req_vec[3] |
		  park_vec[0] & ~req_vec[1] & ~req_vec[2] & req_vec[3] |
		  req_vec[3] & ~req_vec[0] & ~req_vec[1] & ~req_vec[2];

endmodule // sparc_exu_rndrob

   
   
// Copyright (c) 2015 Princeton University
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of Princeton University nor the
//       names of its contributors may be used to endorse or promote products
//       derived from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY PRINCETON UNIVERSITY "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL PRINCETON UNIVERSITY BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

// Wraps the EXU to tie unused signals when no scan chain is present

// Copyright (c) 2015 Princeton University
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of Princeton University nor the
//       names of its contributors may be used to endorse or promote products
//       derived from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY PRINCETON UNIVERSITY "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL PRINCETON UNIVERSITY BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

//==================================================================================================
//  Filename      : define.h
//  Created On    : 2014-02-20
//  Last Modified : 2018-11-16 17:14:11
//  Revision      :
//  Author        : Yaosheng Fu
//  Company       : Princeton University
//  Email         : yfu@princeton.edu
//
//  Description   : main header file defining global architecture parameters
//
//
//==================================================================================================





































































































































































































































































































































































































































































































































































































// Modified by Princeton University on June 9th, 2015
/*
* ========== Copyright Header Begin ==========================================
* 
* OpenSPARC T1 Processor File: lsu.h
* Copyright (c) 2006 Sun Microsystems, Inc.  All Rights Reserved.
* DO NOT ALTER OR REMOVE COPYRIGHT NOTICES.
* 
* The above named program is free software; you can redistribute it and/or
* modify it under the terms of the GNU General Public
* License version 2 as published by the Free Software Foundation.
* 
* The above named program is distributed in the hope that it will be 
* useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
* General Public License for more details.
* 
* You should have received a copy of the GNU General Public
* License along with this work; if not, write to the Free Software
* Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA 02110-1301, USA.
* 
* ========== Copyright Header End ============================================
*/

// devices.xml






// 1:0


// 128

// 32


// 10

// 7


// 6:0



// 29 + 1 parity





// 144




















//`define STB_PCX_WY_HI   107
//`define STB_PCX_WY_LO   106



















































































// TLB Tag and Data Format
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	

	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	

	
	
	
	
	
	
	
	
	
	
	
	
	
	


// I-TLB version - lsu_tlb only.
























// // Invalidate Format
// //addr<5:4>=00
// `define CPX_A00_C0_LO	0
// `define CPX_A00_C0_HI	3
// `define CPX_A00_C1_LO	4
// `define CPX_A00_C1_HI	7
// `define CPX_A00_C2_LO	8
// `define CPX_A00_C2_HI	11
// `define CPX_A00_C3_LO	12
// `define CPX_A00_C3_HI	15
// `define CPX_A00_C4_LO	16
// `define CPX_A00_C4_HI	19
// `define CPX_A00_C5_LO	20
// `define CPX_A00_C5_HI	23
// `define CPX_A00_C6_LO	24
// `define CPX_A00_C6_HI	27
// `define CPX_A00_C7_LO	28
// `define CPX_A00_C7_HI	31

// //addr<5:4>=01
// `define CPX_A01_C0_LO	32
// `define CPX_A01_C0_HI	34
// `define CPX_A01_C1_LO	35
// `define CPX_A01_C1_HI	37
// `define CPX_A01_C2_LO	38
// `define CPX_A01_C2_HI	40
// `define CPX_A01_C3_LO	41
// `define CPX_A01_C3_HI	43
// `define CPX_A01_C4_LO	44
// `define CPX_A01_C4_HI	46
// `define CPX_A01_C5_LO	47
// `define CPX_A01_C5_HI	49
// `define CPX_A01_C6_LO	50
// `define CPX_A01_C6_HI	52
// `define CPX_A01_C7_LO	53
// `define CPX_A01_C7_HI	55

// //addr<5:4>=10
// `define CPX_A10_C0_LO	56
// `define CPX_A10_C0_HI	59
// `define CPX_A10_C1_LO	60
// `define CPX_A10_C1_HI	63
// `define CPX_A10_C2_LO	64
// `define CPX_A10_C2_HI	67
// `define CPX_A10_C3_LO	68
// `define CPX_A10_C3_HI	71
// `define CPX_A10_C4_LO	72
// `define CPX_A10_C4_HI	75
// `define CPX_A10_C5_LO	76
// `define CPX_A10_C5_HI	79
// `define CPX_A10_C6_LO	80
// `define CPX_A10_C6_HI	83
// `define CPX_A10_C7_LO	84
// `define CPX_A10_C7_HI	87

// //addr<5:4>=11
// `define CPX_A11_C0_LO	88
// `define CPX_A11_C0_HI	90
// `define CPX_A11_C1_LO	91
// `define CPX_A11_C1_HI	93
// `define CPX_A11_C2_LO	94
// `define CPX_A11_C2_HI	96
// `define CPX_A11_C3_LO	97
// `define CPX_A11_C3_HI	99
// `define CPX_A11_C4_LO	100
// `define CPX_A11_C4_HI	102
// `define CPX_A11_C5_LO	103
// `define CPX_A11_C5_HI	105
// `define CPX_A11_C6_LO	106
// `define CPX_A11_C6_HI	108
// `define CPX_A11_C7_LO	109
// `define CPX_A11_C7_HI	111

// cpuid - 4b



// CPUany, addr<5:4>=00,10
// `define CPX_AX0_INV_DVLD 0
// `define CPX_AX0_INV_IVLD 1
// `define CPX_AX0_INV_WY_LO 2
// `define CPX_AX0_INV_WY_HI 3

// CPUany, addr<5:4>=01,11
// `define CPX_AX1_INV_DVLD 0
// `define CPX_AX1_INV_WY_LO 1
// `define CPX_AX1_INV_WY_HI 2

// CPUany, addr<5:4>=01,11
// `define CPX_AX1_INV_DVLD 0
// `define CPX_AX1_INV_WY_LO 1
// `define CPX_AX1_INV_WY_HI 2

// DTAG parity error Invalidate




// CPX BINIT STORE


module sparc_exu_wrap
(
    input                arst_l,
    input [63:0]         ffu_exu_rsr_data_m,
    input                grst_l,
    input                ifu_exu_addr_mask_d,
    input [2:0]          ifu_exu_aluop_d,
    input                ifu_exu_casa_d,
    input                ifu_exu_dbrinst_d,
    input                ifu_exu_disable_ce_e,
    input                ifu_exu_dontmv_regz0_e,
    input                ifu_exu_dontmv_regz1_e,
    input [7:0]          ifu_exu_ecc_mask,
    input                ifu_exu_enshift_d,
    input                ifu_exu_flushw_e,
    input                ifu_exu_ialign_d,
    input [31:0]         ifu_exu_imm_data_d,
    input                ifu_exu_inj_irferr,
    input                ifu_exu_inst_vld_e,
    input                ifu_exu_inst_vld_w,
    input                ifu_exu_invert_d,
    input                ifu_exu_kill_e,
    input [4:0]          ifu_exu_muldivop_d,
    input                ifu_exu_muls_d,
    input                ifu_exu_nceen_e,
    input [47:0]         ifu_exu_pc_d,
    input [63:0]         ifu_exu_pcver_e,
    input                ifu_exu_range_check_jlret_d,
    input                ifu_exu_range_check_other_d,
    input [4:0]          ifu_exu_rd_d,
    input                ifu_exu_rd_exusr_e,
    input                ifu_exu_rd_ffusr_e,
    input                ifu_exu_rd_ifusr_e,
    input                ifu_exu_ren1_s,
    input                ifu_exu_ren2_s,
    input                ifu_exu_ren3_s,
    input                ifu_exu_restore_d,
    input                ifu_exu_restored_e,
    input                ifu_exu_return_d,
    input [4:0]          ifu_exu_rs1_s,
    input                ifu_exu_rs1_vld_d,
    input [4:0]          ifu_exu_rs2_s,
    input                ifu_exu_rs2_vld_d,
    input [4:0]          ifu_exu_rs3_s,
    input                ifu_exu_rs3e_vld_d,
    input                ifu_exu_rs3o_vld_d,
    input                ifu_exu_save_d,
    input                ifu_exu_saved_e,
    input                ifu_exu_setcc_d,
    input                ifu_exu_sethi_inst_d,
    input [2:0]          ifu_exu_shiftop_d,
    input                ifu_exu_tagop_d,
    input                ifu_exu_tcc_e,
    input [1:0]          ifu_exu_tid_s2,
    input                ifu_exu_ttype_vld_m,
    input                ifu_exu_tv_d,
    input                ifu_exu_use_rsr_e_l,
    input                ifu_exu_usecin_d,
    input                ifu_exu_useimm_d,
    input                ifu_exu_wen_d,
    input                ifu_tlu_flush_m,
    input [6:0]          ifu_tlu_sraddr_d,
    input                ifu_tlu_wsr_inst_d,
    input [63:0]         lsu_exu_dfill_data_g,
    input                lsu_exu_dfill_vld_g,
    input                lsu_exu_flush_pipe_w,
    input                lsu_exu_ldst_miss_g2,
    input [63:0]         lsu_exu_ldxa_data_g,
    input                lsu_exu_ldxa_m,
    input [4:0]          lsu_exu_rd_m,
    input                lsu_exu_st_dtlb_perr_g,
    input [1:0]          lsu_exu_thr_m,
    input                mul_exu_ack,
    input [63:0]         mul_exu_data_g,
    input                rclk,
    input [1:0]          tlu_exu_agp,
    input                tlu_exu_agp_swap,
    input [1:0]          tlu_exu_agp_tid,
    input [7:0]          tlu_exu_ccr_m,
    input [2:0]          tlu_exu_cwp_m,
    input                tlu_exu_cwp_retry_m,
    input                tlu_exu_cwpccr_update_m,
    input                tlu_exu_pic_onebelow_m,
    input                tlu_exu_pic_twobelow_m,
    input                tlu_exu_priv_trap_m,
    input [63:0]         tlu_exu_rsr_data_m,
    output               exu_ffu_wsr_inst_e,
    output [47:0]        exu_ifu_brpc_e,
    output [7:0]         exu_ifu_cc_d,
    output               exu_ifu_ecc_ce_m,
    output               exu_ifu_ecc_ue_m,
    output [7:0]         exu_ifu_err_reg_m,
    output               exu_ifu_inj_ack,
    output [3:0]         exu_ifu_longop_done_g,
    output [3:0]         exu_ifu_oddwin_s,
    output               exu_ifu_regn_e,
    output               exu_ifu_regz_e,
    output               exu_ifu_spill_e,
    output               exu_ifu_va_oor_m,
    output [(6 + 4):3]        exu_lsu_early_va_e,
    output [47:0]        exu_lsu_ldst_va_e,
    output               exu_lsu_priority_trap_m,
    output [63:0]        exu_lsu_rs2_data_e,
    output [63:0]        exu_lsu_rs3_data_e,
    output [7:0]         exu_mmu_early_va_e,
    output               exu_mul_input_vld,
    output [63:0]        exu_mul_rs1_data,
    output [63:0]        exu_mul_rs2_data,
    output [7:0]         exu_tlu_ccr0_w,
    output [7:0]         exu_tlu_ccr1_w,
    output [7:0]         exu_tlu_ccr2_w,
    output [7:0]         exu_tlu_ccr3_w,
    output [2:0]         exu_tlu_cwp0_w,
    output [2:0]         exu_tlu_cwp1_w,
    output [2:0]         exu_tlu_cwp2_w,
    output [2:0]         exu_tlu_cwp3_w,
    output               exu_tlu_cwp_cmplt,
    output [1:0]         exu_tlu_cwp_cmplt_tid,
    output               exu_tlu_cwp_retry,
    output               exu_tlu_misalign_addr_jmpl_rtn_m,
    output               exu_tlu_spill,
    output               exu_tlu_spill_other,
    output [1:0]         exu_tlu_spill_tid,
    output [2:0]         exu_tlu_spill_wtype,
    output [8:0]         exu_tlu_ttype_m,
    output               exu_tlu_ttype_vld_m,
    output               exu_tlu_va_oor_jl_ret_m,
    output               exu_tlu_va_oor_m,
    output [63:0]        exu_tlu_wsr_data_m,
    output [7:0]         exu_ifu_err_synd_m,

    // jtag debug
    output wire [94-1:0] core_rtap_data,
    input wire rtap_core_val,
    input wire rtap_core_threadid_0,
    input wire [4-1:0]  rtap_core_id,
    input wire [4:0] rtap_core_data_4_0
);

    sparc_exu exu   (
                 .short_si0              (1'bx),
                 .short_so0              (),
                 .short_si1              (1'bx),
                 .short_so1              (),
                 .si0                    (1'bx),
                 .so0                    (),
                 // reset stuff
                 .grst_l                (grst_l),
                 .arst_l                (arst_l),
                 .mul_exu_data_g        (mul_exu_data_g),
                 .ifu_tlu_wsr_inst_d    (ifu_tlu_wsr_inst_d),
                 //
                 .exu_tlu_ue_trap_m     (),

                     /*AUTOINST*/
                 // Outputs
                 .exu_ffu_wsr_inst_e    (exu_ffu_wsr_inst_e),
                 .exu_ifu_brpc_e        (exu_ifu_brpc_e),
                 .exu_ifu_cc_d          (exu_ifu_cc_d),
                 .exu_ifu_ecc_ce_m      (exu_ifu_ecc_ce_m),
                 .exu_ifu_ecc_ue_m      (exu_ifu_ecc_ue_m),
                 .exu_ifu_err_reg_m     (exu_ifu_err_reg_m),
                 .exu_ifu_inj_ack       (exu_ifu_inj_ack),
                 .exu_ifu_longop_done_g (exu_ifu_longop_done_g),
                 .exu_ifu_oddwin_s      (exu_ifu_oddwin_s),
                 .exu_ifu_regn_e        (exu_ifu_regn_e),
                 .exu_ifu_regz_e        (exu_ifu_regz_e),
                 .exu_ifu_spill_e       (exu_ifu_spill_e),
                 .exu_ifu_va_oor_m      (exu_ifu_va_oor_m),
                 .exu_lsu_early_va_e    (exu_lsu_early_va_e),
                 .exu_lsu_ldst_va_e     (exu_lsu_ldst_va_e),
                 .exu_lsu_priority_trap_m(exu_lsu_priority_trap_m),
                 .exu_lsu_rs2_data_e    (exu_lsu_rs2_data_e),
                 .exu_lsu_rs3_data_e    (exu_lsu_rs3_data_e),
                 .exu_mmu_early_va_e    (exu_mmu_early_va_e),
                 .exu_mul_input_vld     (exu_mul_input_vld),
                 .exu_mul_rs1_data      (exu_mul_rs1_data),
                 .exu_mul_rs2_data      (exu_mul_rs2_data),
                 .exu_spu_rs3_data_e    (),
                 .exu_tlu_ccr0_w        (exu_tlu_ccr0_w),
                 .exu_tlu_ccr1_w        (exu_tlu_ccr1_w),
                 .exu_tlu_ccr2_w        (exu_tlu_ccr2_w),
                 .exu_tlu_ccr3_w        (exu_tlu_ccr3_w),
                 .exu_tlu_cwp0_w        (exu_tlu_cwp0_w),
                 .exu_tlu_cwp1_w        (exu_tlu_cwp1_w),
                 .exu_tlu_cwp2_w        (exu_tlu_cwp2_w),
                 .exu_tlu_cwp3_w        (exu_tlu_cwp3_w),
                 .exu_tlu_cwp_cmplt     (exu_tlu_cwp_cmplt),
                 .exu_tlu_cwp_cmplt_tid (exu_tlu_cwp_cmplt_tid),
                 .exu_tlu_cwp_retry     (exu_tlu_cwp_retry),
                 .exu_tlu_misalign_addr_jmpl_rtn_m(exu_tlu_misalign_addr_jmpl_rtn_m),
                 .exu_tlu_spill         (exu_tlu_spill),
                 .exu_tlu_spill_other   (exu_tlu_spill_other),
                 .exu_tlu_spill_tid     (exu_tlu_spill_tid),
                 .exu_tlu_spill_wtype   (exu_tlu_spill_wtype),
                 .exu_tlu_ttype_m       (exu_tlu_ttype_m),
                 .exu_tlu_ttype_vld_m   (exu_tlu_ttype_vld_m),
                 .exu_tlu_va_oor_jl_ret_m(exu_tlu_va_oor_jl_ret_m),
                 .exu_tlu_va_oor_m      (exu_tlu_va_oor_m),
                 .exu_tlu_wsr_data_m    (exu_tlu_wsr_data_m),
                 .exu_ifu_err_synd_m    (exu_ifu_err_synd_m),
                 // Inputs
                 .ffu_exu_rsr_data_m    (ffu_exu_rsr_data_m),
                 .ifu_exu_addr_mask_d   (ifu_exu_addr_mask_d),
                 .ifu_exu_aluop_d       (ifu_exu_aluop_d),
                 .ifu_exu_casa_d        (ifu_exu_casa_d),
                 .ifu_exu_dbrinst_d     (ifu_exu_dbrinst_d),
                 .ifu_exu_disable_ce_e  (ifu_exu_disable_ce_e),
                 .ifu_exu_dontmv_regz0_e(ifu_exu_dontmv_regz0_e),
                 .ifu_exu_dontmv_regz1_e(ifu_exu_dontmv_regz1_e),
                 .ifu_exu_ecc_mask      (ifu_exu_ecc_mask),
                 .ifu_exu_enshift_d     (ifu_exu_enshift_d),
                 .ifu_exu_flushw_e      (ifu_exu_flushw_e),
                 .ifu_exu_ialign_d      (ifu_exu_ialign_d),
                 .ifu_exu_imm_data_d    (ifu_exu_imm_data_d),
                 .ifu_exu_inj_irferr    (ifu_exu_inj_irferr),
                 .ifu_exu_inst_vld_e    (ifu_exu_inst_vld_e),
                 .ifu_exu_inst_vld_w    (ifu_exu_inst_vld_w),
                 .ifu_exu_invert_d      (ifu_exu_invert_d),
                 .ifu_exu_kill_e        (ifu_exu_kill_e),
                 .ifu_exu_muldivop_d    (ifu_exu_muldivop_d),
                 .ifu_exu_muls_d        (ifu_exu_muls_d),
                 .ifu_exu_nceen_e       (ifu_exu_nceen_e),
                 .ifu_exu_pc_d          (ifu_exu_pc_d),
                 .ifu_exu_pcver_e       (ifu_exu_pcver_e),
                 .ifu_exu_range_check_jlret_d(ifu_exu_range_check_jlret_d),
                 .ifu_exu_range_check_other_d(ifu_exu_range_check_other_d),
                 .ifu_exu_rd_d          (ifu_exu_rd_d),
                 .ifu_exu_rd_exusr_e    (ifu_exu_rd_exusr_e),
                 .ifu_exu_rd_ffusr_e    (ifu_exu_rd_ffusr_e),
                 .ifu_exu_rd_ifusr_e    (ifu_exu_rd_ifusr_e),
                 .ifu_exu_ren1_s        (ifu_exu_ren1_s),
                 .ifu_exu_ren2_s        (ifu_exu_ren2_s),
                 .ifu_exu_ren3_s        (ifu_exu_ren3_s),
                 .ifu_exu_restore_d     (ifu_exu_restore_d),
                 .ifu_exu_restored_e    (ifu_exu_restored_e),
                 .ifu_exu_return_d      (ifu_exu_return_d),
                 .ifu_exu_rs1_s         (ifu_exu_rs1_s),
                 .ifu_exu_rs1_vld_d     (ifu_exu_rs1_vld_d),
                 .ifu_exu_rs2_s         (ifu_exu_rs2_s),
                 .ifu_exu_rs2_vld_d     (ifu_exu_rs2_vld_d),
                 .ifu_exu_rs3_s         (ifu_exu_rs3_s),
                 .ifu_exu_rs3e_vld_d    (ifu_exu_rs3e_vld_d),
                 .ifu_exu_rs3o_vld_d    (ifu_exu_rs3o_vld_d),
                 .ifu_exu_save_d        (ifu_exu_save_d),
                 .ifu_exu_saved_e       (ifu_exu_saved_e),
                 .ifu_exu_setcc_d       (ifu_exu_setcc_d),
                 .ifu_exu_sethi_inst_d  (ifu_exu_sethi_inst_d),
                 .ifu_exu_shiftop_d     (ifu_exu_shiftop_d),
                 .ifu_exu_tagop_d       (ifu_exu_tagop_d),
                 .ifu_exu_tcc_e         (ifu_exu_tcc_e),
                 .ifu_exu_tid_s2        (ifu_exu_tid_s2),
                 .ifu_exu_ttype_vld_m   (ifu_exu_ttype_vld_m),
                 .ifu_exu_tv_d          (ifu_exu_tv_d),
                 .ifu_exu_use_rsr_e_l   (ifu_exu_use_rsr_e_l),
                 .ifu_exu_usecin_d      (ifu_exu_usecin_d),
                 .ifu_exu_useimm_d      (ifu_exu_useimm_d),
                 .ifu_exu_wen_d         (ifu_exu_wen_d),
                 .ifu_tlu_flush_m       (ifu_tlu_flush_m),
                 .ifu_tlu_sraddr_d      (ifu_tlu_sraddr_d),
                 .lsu_exu_dfill_data_g  (lsu_exu_dfill_data_g),
                 .lsu_exu_dfill_vld_g   (lsu_exu_dfill_vld_g),
                 .lsu_exu_flush_pipe_w  (lsu_exu_flush_pipe_w),
                 .lsu_exu_ldst_miss_g2  (lsu_exu_ldst_miss_g2),
                 .lsu_exu_ldxa_data_g   (lsu_exu_ldxa_data_g),
                 .lsu_exu_ldxa_m        (lsu_exu_ldxa_m),
                 .lsu_exu_rd_m          (lsu_exu_rd_m),
                 .lsu_exu_st_dtlb_perr_g(lsu_exu_st_dtlb_perr_g),
                 .lsu_exu_thr_m         (lsu_exu_thr_m),
                 .mul_exu_ack           (mul_exu_ack),
                 .rclk                  (rclk),
                 .se                    (1'b0),
                 .sehold                (1'b0),
                 .tlu_exu_agp           (tlu_exu_agp),
                 .tlu_exu_agp_swap      (tlu_exu_agp_swap),
                 .tlu_exu_agp_tid       (tlu_exu_agp_tid),
                 .tlu_exu_ccr_m         (tlu_exu_ccr_m),
                 .tlu_exu_cwp_m         (tlu_exu_cwp_m),
                 .tlu_exu_cwp_retry_m   (tlu_exu_cwp_retry_m),
                 .tlu_exu_cwpccr_update_m(tlu_exu_cwpccr_update_m),
                 .tlu_exu_pic_onebelow_m(tlu_exu_pic_onebelow_m),
                 .tlu_exu_pic_twobelow_m(tlu_exu_pic_twobelow_m),
                 .tlu_exu_priv_trap_m   (tlu_exu_priv_trap_m),
                 .tlu_exu_rsr_data_m    (tlu_exu_rsr_data_m),

                 .core_rtap_data          (core_rtap_data),
                 .rtap_core_val         (rtap_core_val),
                 .rtap_core_threadid         ({1'bx, rtap_core_threadid_0}),
                 .rtap_core_id         (rtap_core_id),
                 .rtap_core_data         ({{(94-5){1'bx}}, rtap_core_data_4_0})
                 );

endmodule
// Modified by Princeton University on June 9th, 2015
// ========== Copyright Header Begin ==========================================
// 
// OpenSPARC T1 Processor File: sparc_exu_shft.v
// Copyright (c) 2006 Sun Microsystems, Inc.  All Rights Reserved.
// DO NOT ALTER OR REMOVE COPYRIGHT NOTICES.
// 
// The above named program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public
// License version 2 as published by the Free Software Foundation.
// 
// The above named program is distributed in the hope that it will be 
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// General Public License for more details.
// 
// You should have received a copy of the GNU General Public
// License along with this work; if not, write to the Free Software
// Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA 02110-1301, USA.
// 
// ========== Copyright Header End ============================================
////////////////////////////////////////////////////////////////////////
/*
//  Module Name: sparc_exu_shft
//	Description: This block implements right and left shifting of any amount
//								from 0 to 63.
*/


module sparc_exu_shft (/*AUTOARG*/
   // Outputs
   shft_alu_shift_out_e, 
   // Inputs
   ecl_shft_lshift_e_l, ecl_shft_op32_e, ecl_shft_shift4_e, 
   ecl_shft_shift1_e, byp_alu_rs1_data_e, byp_alu_rs2_data_e, 
   ecl_shft_enshift_e_l, ecl_shft_extendbit_e, 
   ecl_shft_extend32bit_e_l
   ) ;
   input 	ecl_shft_lshift_e_l;    // if 0 do left shift.  else right shift
   input  ecl_shft_op32_e;      // indicates 32 bit operation so upper 32 = 0
   //input [3:0] ecl_shft_shift16_e;// [48, 32, 16, 0] shift
   input [3:0] ecl_shft_shift4_e;// [12, 8, 4, 0] shift
   input [3:0] ecl_shft_shift1_e;// [3, 2, 1, 0] shift
   input [63:0] byp_alu_rs1_data_e;
   input [5:4] byp_alu_rs2_data_e;
   input        ecl_shft_enshift_e_l;// enables inputs to shifter
   input        ecl_shft_extendbit_e;
   input    ecl_shft_extend32bit_e_l;
   
   output [63:0] shft_alu_shift_out_e;

   wire [63:0]   shifter_input; // enabled input
   wire [63:0]   shifter_input_b1;// buffered input
   wire [63:0]   rshifterinput; // masked for 32-bit operation
   wire [63:0]   rshifterinput_b1; // masked for 32-bit operation
   wire [63:0]   lshift16;      // output of the respective mux
   wire [63:0]   rshift16;
   wire [63:0]   lshift4;
   wire [63:0]   rshift4;
   wire [63:0]   lshift1;
   wire [63:0]   rshift1;
   wire [63:0]   lshift16_b1;      // buffed output of the respective mux
   wire [63:0]   rshift16_b1;
   wire [63:0]   lshift4_b1;
   wire [63:0]   rshift4_b1;
   wire [47:0]   shft_extendbit_e;
   wire [3:0]    shift16_e;
   wire          shiftby_msb;
   wire          extend32bit_e;

   assign        shiftby_msb = byp_alu_rs2_data_e[5] & ~ecl_shft_op32_e;
   assign        shift16_e[0] = ~shiftby_msb & ~byp_alu_rs2_data_e[4];
   assign        shift16_e[1] = ~shiftby_msb & byp_alu_rs2_data_e[4];
   assign        shift16_e[2] = shiftby_msb & ~byp_alu_rs2_data_e[4];
   assign        shift16_e[3] = shiftby_msb & byp_alu_rs2_data_e[4];
   // enable inputs
   assign   shifter_input[63:0] = byp_alu_rs1_data_e[63:0] & {64{~ecl_shft_enshift_e_l}};
   
   // mux between left and right shifts
   dp_mux2es #(64) mux_shiftout(.dout(shft_alu_shift_out_e[63:0]), .in0(lshift1[63:0]),
                           .in1(rshift1[63:0]),
                           .sel(ecl_shft_lshift_e_l));
   
   // mask out top for r_shift 32bit
   assign   extend32bit_e = ~ecl_shft_extend32bit_e_l;
   dp_mux2es #(32) mux_rshift_extend(.dout(rshifterinput[63:32]),
                                     .in0(byp_alu_rs1_data_e[63:32]),
                                     .in1({32{extend32bit_e}}),
                                     .sel(ecl_shft_op32_e));
   assign rshifterinput[31:0] = shifter_input[31:0];

   assign shft_extendbit_e[47:0] = {48{ecl_shft_extendbit_e}};

   // right shift muxes
   mux4ds #(64) mux_right16(.dout(rshift16[63:0]),
                          .in0({shft_extendbit_e[47:0], rshifterinput_b1[63:48]}),
                          .in1({shft_extendbit_e[47:16], rshifterinput_b1[63:32]}),
                          .in2({shft_extendbit_e[47:32], rshifterinput_b1[63:16]}),
                          .in3(rshifterinput_b1[63:0]),
                          .sel0(shift16_e[3]),
                          .sel1(shift16_e[2]),
                          .sel2(shift16_e[1]),
                          .sel3(shift16_e[0]));
   mux4ds #(64) mux_right4(.dout(rshift4[63:0]),
                         .in0({shft_extendbit_e[47:36], rshift16_b1[63:12]}),
                         .in1({shft_extendbit_e[47:40], rshift16_b1[63:8]}),
                         .in2({shft_extendbit_e[47:44], rshift16_b1[63:4]}),
                         .in3(rshift16_b1[63:0]),
                         .sel0(ecl_shft_shift4_e[3]),
                         .sel1(ecl_shft_shift4_e[2]),
                         .sel2(ecl_shft_shift4_e[1]),
                         .sel3(ecl_shft_shift4_e[0]));
   mux4ds #(64) mux_right1(.dout(rshift1[63:0]),
                         .in0({shft_extendbit_e[47:45], rshift4_b1[63:3]}),
                         .in1({shft_extendbit_e[47:46], rshift4_b1[63:2]}),
                         .in2({shft_extendbit_e[47], rshift4_b1[63:1]}),
                         .in3(rshift4_b1[63:0]),
                         .sel0(ecl_shft_shift1_e[3]),
                         .sel1(ecl_shft_shift1_e[2]),
                         .sel2(ecl_shft_shift1_e[1]),
                         .sel3(ecl_shft_shift1_e[0]));

   // buffer signals to right muxes
   dp_buffer #(64) buf_rshiftin(.dout(rshifterinput_b1[63:0]), .in(rshifterinput[63:0]));
   dp_buffer #(64) buf_rshift16(.dout(rshift16_b1[63:0]), .in(rshift16[63:0]));
   dp_buffer #(64) buf_rshift4(.dout(rshift4_b1[63:0]), .in(rshift4[63:0]));

   // left shift muxes
   mux4ds #(64) mux_left16(.dout(lshift16[63:0]),
                         .in0({shifter_input_b1[15:0], {48{1'b0}}}),
                         .in1({shifter_input_b1[31:0], {32{1'b0}}}),
                         .in2({shifter_input_b1[47:0], {16{1'b0}}}),
                         .in3(shifter_input_b1[63:0]),
                         .sel0(shift16_e[3]),
                         .sel1(shift16_e[2]),
                         .sel2(shift16_e[1]),
                         .sel3(shift16_e[0]));
   mux4ds #(64) mux_left4(.dout(lshift4[63:0]),
                        .in0({lshift16_b1[51:0], {12{1'b0}}}),
                        .in1({lshift16_b1[55:0], {8{1'b0}}}),
                        .in2({lshift16_b1[59:0], {4{1'b0}}}),
                        .in3(lshift16_b1[63:0]),
                        .sel0(ecl_shft_shift4_e[3]),
                        .sel1(ecl_shft_shift4_e[2]),
                        .sel2(ecl_shft_shift4_e[1]),
                        .sel3(ecl_shft_shift4_e[0]));
   mux4ds #(64) mux_left1(.dout(lshift1[63:0]),
                        .in0({lshift4_b1[60:0], {3{1'b0}}}),
                        .in1({lshift4_b1[61:0], {2{1'b0}}}),
                        .in2({lshift4_b1[62:0], {1{1'b0}}}),
                        .in3(lshift4_b1[63:0]),
                        .sel0(ecl_shft_shift1_e[3]),
                        .sel1(ecl_shft_shift1_e[2]),
                        .sel2(ecl_shft_shift1_e[1]),
                        .sel3(ecl_shft_shift1_e[0]));

   // buffer signals to left muxes
   dp_buffer #(64) buf_lshiftin(.dout(shifter_input_b1[63:0]), .in(shifter_input[63:0]));
   dp_buffer #(64) buf_lshift16(.dout(lshift16_b1[63:0]), .in(lshift16[63:0]));
   dp_buffer #(64) buf_lshift4(.dout(lshift4_b1[63:0]), .in(lshift4[63:0]));

    
endmodule // sparc_exu_shft
