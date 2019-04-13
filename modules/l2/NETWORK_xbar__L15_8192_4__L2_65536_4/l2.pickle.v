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

/********************************************************************
 * Author: Sam Payne
 * 
 * Module encapsulates an asynchronus FIFO used for bridging signals
 * across clock domains, parameters allow for different sized modules
 * compatible with wide range of frequencies.
 *
 * * *******************************************************************/



module async_fifo 
#(
	parameter DSIZE = 64,
	parameter ASIZE = 5,
	parameter MEMSIZE = 16 // should be 2 ^ (ASIZE-1)
)
(
	rdata, 
	rempty,
	rclk,
	ren,
	wdata,
	wfull,
	wclk,
	wval,
	wreset,
	rreset
	);

//Inputs and Outputs
output  [DSIZE-1:0] 	rdata;
output			rempty;
output 			wfull;
input	[DSIZE-1:0]	wdata;
input			wval;
input			ren;
input			rclk;
input			wclk;
input 			wreset;
input			rreset;

//Internal Registers
reg	[ASIZE-1:0]	g_wptr;
reg	[ASIZE-1:0]	g_rptr;

reg	[ASIZE-1:0]	g_rsync1, g_rsync2;
reg	[ASIZE-1:0]	g_wsync1, g_wsync2;

//Memory
reg	[DSIZE-1:0] 	fifo[MEMSIZE-1:0];

wire [ASIZE-1:0] b_wptr;
wire [ASIZE-1:0] b_wptr_next;
wire [ASIZE-1:0] g_wptr_next;
wire [ASIZE-1:0] b_rptr;
wire [ASIZE-1:0] b_rptr_next;
wire [ASIZE-1:0] g_rptr_next;

/********************************************************************
COMBINATIONAL LOGIC
********************************************************************/

//convert gray to binary
assign b_wptr[ASIZE-1:0] = ({1'b0, b_wptr[ASIZE-1:1]} ^ g_wptr[ASIZE-1:0]);
assign b_rptr[ASIZE-1:0] = ({1'b0, b_rptr[ASIZE-1:1]} ^ g_rptr[ASIZE-1:0]);

//increment
assign b_wptr_next = b_wptr + 1;
assign b_rptr_next = b_rptr + 1;

//convert binary to gray
assign g_wptr_next[ASIZE-1:0] = {1'b0, b_wptr_next[ASIZE-1:1]} ^ b_wptr_next[ASIZE-1:0];
assign g_rptr_next[ASIZE-1:0] = {1'b0, b_rptr_next[ASIZE-1:1]} ^ b_rptr_next[ASIZE-1:0];

//full and empty signals
assign wfull =  (g_wptr[ASIZE-1]   != g_rsync2[ASIZE-1]  ) && 
		(g_wptr[ASIZE-2]   != g_rsync2[ASIZE-2]  ) &&
		(g_wptr[ASIZE-3:0] == g_rsync2[ASIZE-3:0]) ||
		(wreset || rreset);

assign rempty =  (g_wsync2[ASIZE-1:0] == g_rptr[ASIZE-1:0]) ||
	         (wreset || rreset);

//output values
assign rdata = fifo[b_rptr[ASIZE-2:0]];

/********************************************************************
SEQUENTIAL LOGIC
********************************************************************/

//transfer register values
always @(posedge rclk) begin
	if (rreset) begin
		g_rptr <= 0;
	end
	else if (ren && !rempty) begin
		g_rptr <= g_rptr_next;
	end

	g_wsync1 <= g_wptr;
	g_wsync2 <= g_wsync1;
end

always @(posedge wclk) begin
	if (wreset) begin
		g_wptr <= 0;
	end
	else if (wval && !wfull) begin
		fifo[b_wptr[ASIZE-2:0]] <= wdata;
		g_wptr <= g_wptr_next;
	end

	g_rsync1 <= g_rptr;
	g_rsync2 <= g_rsync1;

	
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

// Author:      Alexey Lavrov, Tri Nguyen
// Description: A simple wrapper to infer Xilinx BRAMs for SRAMs, modified to be synthesizable

module bram_1r1w_wrapper 
#(parameter NAME="", DEPTH=1, ADDR_WIDTH=1, BITMASK_WIDTH=1, DATA_WIDTH=1)
(
  input wire MEMCLK,
  input wire RESET_N,
  input wire CEA,
  input wire [ADDR_WIDTH-1:0] AA,
  input wire [ADDR_WIDTH-1:0] AB,

  input wire RDWENA,
  input wire CEB,
  input wire RDWENB,
  input wire [DATA_WIDTH-1:0] BWA,
  input wire [DATA_WIDTH-1:0] DINA,
  output reg [DATA_WIDTH-1:0] DOUTA,
  input wire [DATA_WIDTH-1:0] BWB,
  input wire [DATA_WIDTH-1:0] DINB,
  output wire [DATA_WIDTH-1:0] DOUTB
  // input wire [`BIST_OP_WIDTH-1:0] BIST_COMMAND,
  // input wire [`SRAM_WRAPPER_BUS_WIDTH-1:0] BIST_DIN,
  // output reg [`SRAM_WRAPPER_BUS_WIDTH-1:0] BIST_DOUT,
  // input wire [`BIST_ID_WIDTH-1:0] SRAMID
);

wire                            write_enable_in;
wire                            read_enable_in;

// Temporary storage for write data
reg                             write_enable_in_reg;
reg   [ADDR_WIDTH-1:0    ]      WRITE_ADDRESS_REG;
reg   [ADDR_WIDTH-1:0    ]      WRITE_ADDRESS_REG_muxed;
reg   [BITMASK_WIDTH-1:0 ]      WRITE_BIT_MASK_REG;
reg   [DATA_WIDTH-1:0    ]      DIN_r;
// reg   [DATA_WIDTH-1:0    ]      DOUTB_r;

reg                             read_enable_in_reg;

reg   [DATA_WIDTH-1:0    ]      bram_data_in_r;

wire                            bram_write_en;
reg                            bram_write_en_muxed;
wire                            bram_read_en;
wire                            bram_write_read_en;
reg  [DATA_WIDTH-1:0    ]      bram_data_write_read_out_reg;
reg  [DATA_WIDTH-1:0    ]      bram_data_read_out_reg;
reg  [DATA_WIDTH-1:0    ]      bram_data_in;
reg  [DATA_WIDTH-1:0    ]      bram_data_in_muxed;
wire  [DATA_WIDTH-1:0    ]      last_wrote_data;
wire                            rw_conflict;
reg                             rw_conflict_r;
wire                            ww_conflict;
reg                             ww_conflict_r;

/* renaming signals */
assign read_enable_in    = CEA & (RDWENA == 1'b1);
assign write_enable_in   = CEB & (RDWENB == 1'b0);
wire [ADDR_WIDTH-1:0    ] READ_ADDRESS = AA;
wire [ADDR_WIDTH-1:0    ] WRITE_ADDRESS = AB;
wire [BITMASK_WIDTH-1:0    ] WRITE_BIT_MASK = BWB;

// Intermediate logic for write processing
always @(posedge MEMCLK) begin
  write_enable_in_reg <= write_enable_in;
  WRITE_ADDRESS_REG   <= WRITE_ADDRESS;
  WRITE_BIT_MASK_REG  <= WRITE_BIT_MASK;
  DIN_r <= DINB;
  read_enable_in_reg  <= read_enable_in;
  bram_data_in_r <= bram_data_in;
  rw_conflict_r  <= rw_conflict;
  ww_conflict_r  <= ww_conflict;
  // DOUTB_r  <= DOUTB;
end

// determining read-write and write-write conflict for data bypassing
assign rw_conflict      = write_enable_in_reg & read_enable_in & (WRITE_ADDRESS_REG == READ_ADDRESS);
assign ww_conflict      = write_enable_in_reg & write_enable_in & (WRITE_ADDRESS_REG == WRITE_ADDRESS);
assign DOUTB = {DATA_WIDTH{1'bx}}; // port B is always used for write

// calculate the correct read and write data after accoutning for conflicts
always @ * begin
  bram_data_in = (DIN_r & WRITE_BIT_MASK_REG);
  if (ww_conflict_r)
    bram_data_in = bram_data_in | (bram_data_in_r & ~WRITE_BIT_MASK_REG);
  else
    bram_data_in = bram_data_in | (bram_data_write_read_out_reg & ~WRITE_BIT_MASK_REG);
  

  // note: DOUT retains value if read enable is not asserted
  // which is why default value is not set for DOUT
  if (read_enable_in_reg) begin
    DOUTA = bram_data_read_out_reg; 
    if (rw_conflict_r) begin
      DOUTA = bram_data_in_r;
    end
  end
end

// synthesizable BRAM
assign bram_write_en      = write_enable_in_reg;
assign bram_read_en         = (read_enable_in) & ~rw_conflict;             // do not read in case of a conflict
assign bram_write_read_en         = (write_enable_in) & ~ww_conflict;             // do not read in case of a conflict

reg [DATA_WIDTH-1:0] ram [DEPTH-1:0];
// reg [%d-1:0] bram_data_write_read_out_reg;
always @(posedge MEMCLK) begin
  if (bram_write_en_muxed) begin
    ram[WRITE_ADDRESS_REG_muxed] <= bram_data_in_muxed;
  end
  if (bram_read_en) begin
    bram_data_read_out_reg <= ram[READ_ADDRESS];
  end
  if (bram_write_read_en) begin
    bram_data_write_read_out_reg <= ram[WRITE_ADDRESS];
  end
end
// END BRAM


/* BIST logic for resetting RAM content to 0s on reset*/
localparam INIT_STATE = 1'd0;
localparam DONE_STATE  = 1'd1;

reg [ADDR_WIDTH-1:0] bist_index;
reg [ADDR_WIDTH-1:0] bist_index_next;
reg init_done;
reg init_done_next;

always @ (posedge MEMCLK)
begin
   if (!RESET_N)
   begin
      bist_index <= 0;
      init_done <= 0;
   end
   else
   begin
      bist_index <= bist_index_next;
      init_done <= init_done_next;
   end
end

always @ *
begin
   bist_index_next = init_done ? bist_index : bist_index + 1;
   init_done_next = ((|(~bist_index)) == 0) | init_done;
end

// MUX for BIST
always @ *
begin
   if (!init_done)
   begin
      WRITE_ADDRESS_REG_muxed = bist_index;
      bram_write_en_muxed = 1'b1;
      bram_data_in_muxed = {DATA_WIDTH{1'b0}};
   end
   else
   begin
      WRITE_ADDRESS_REG_muxed = WRITE_ADDRESS_REG;
      bram_write_en_muxed = bram_write_en;
      bram_data_in_muxed = bram_data_in;
   end
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

// Author:      Alexey Lavrov, Tri Nguyen
// Description: A simple wrapper to infer Xilinx BRAMs for SRAMs

module bram_1rw_wrapper 
#(parameter NAME="", DEPTH=1, ADDR_WIDTH=1, BITMASK_WIDTH=1, DATA_WIDTH=1)
(
    input                         MEMCLK,
    input wire RESET_N,
    input                         CE,
    input   [ADDR_WIDTH-1:0]      A,
    input                         RDWEN,
    input   [BITMASK_WIDTH-1:0]   BW,
    input   [DATA_WIDTH-1:0]      DIN,
    output  [DATA_WIDTH-1:0]      DOUT
);

wire                            write_en;
wire                            read_en;

// Temporary storage for write data
reg                             wen_r;
reg   [ADDR_WIDTH-1:0    ]      A_r;
reg   [BITMASK_WIDTH-1:0 ]      BW_r;
reg   [DATA_WIDTH-1:0    ]      DIN_r;
reg   [DATA_WIDTH-1:0    ]      DOUT_r;

reg                             ren_r;

reg   [DATA_WIDTH-1:0    ]      bram_data_in_r;

wire                            bram_wen;
wire                            bram_ren;
reg  [DATA_WIDTH-1:0    ]      bram_data_out;
wire  [DATA_WIDTH-1:0    ]      bram_data_in;
wire  [DATA_WIDTH-1:0    ]      up_to_date_data;
wire                            rw_conflict;
reg                             rw_conflict_r;



reg   [ADDR_WIDTH-1:0    ]      WRITE_ADDRESS_REG_muxed;
reg                            bram_write_en_muxed;
reg  [DATA_WIDTH-1:0    ]      bram_data_in_muxed;

assign write_en   = CE & (RDWEN == 1'b0);
assign read_en    = CE & (RDWEN == 1'b1);


// Intermediate logic for write processing
always @(posedge MEMCLK) begin
   wen_r <= write_en;
   A_r   <= A;
   BW_r  <= BW;
   DIN_r <= DIN;
end

always @(posedge MEMCLK) begin
  ren_r  <= read_en;
end

always @(posedge MEMCLK)
   bram_data_in_r <= bram_data_in;

always @(posedge MEMCLK)
   rw_conflict_r  <= rw_conflict;

always @(posedge MEMCLK)
  DOUT_r  <= DOUT;

assign bram_data_in = (up_to_date_data & ~BW_r) | (DIN_r & BW_r);

// processing of read in case if it just in the next cycle after read to the same address
assign rw_conflict      = wen_r & CE & (A_r == A);                         // read or write to the same address
assign up_to_date_data  = rw_conflict_r ? bram_data_in_r : bram_data_out;  // delay of mem is 1 cycle
assign bram_ren         = (read_en | write_en) & ~rw_conflict;             // do not read in case of a conflict
                                                                        // to make behaviour of a memory robust
assign bram_wen      = wen_r;

assign DOUT          = ren_r ? up_to_date_data : DOUT_r;

// BRAM
reg [DATA_WIDTH-1:0] ram [DEPTH-1:0];
// reg [%d-1:0] bram_data_out;
always @(posedge MEMCLK) begin
  if (bram_write_en_muxed) begin
      ram[WRITE_ADDRESS_REG_muxed] <= bram_data_in_muxed;
  end
  if (bram_ren) begin
    bram_data_out <= ram[A];
  end
end
// END BRAM

 // undefined by default

/* BIST logic for resetting RAM content to 0s on reset*/
localparam INIT_STATE = 1'd0;
localparam DONE_STATE  = 1'd1;

reg [ADDR_WIDTH-1:0] bist_index;
reg [ADDR_WIDTH-1:0] bist_index_next;
reg init_done;
reg init_done_next;

always @ (posedge MEMCLK)
begin
   if (!RESET_N)
   begin
      bist_index <= 0;
      init_done <= 0;
   end
   else
   begin
      bist_index <= bist_index_next;
      init_done <= init_done_next;
   end
end

always @ *
begin
   bist_index_next = init_done ? bist_index : bist_index + 1;
   init_done_next = ((|(~bist_index)) == 0) | init_done;
end

// MUX for BIST
always @ *
begin
   if (!init_done)
   begin
      WRITE_ADDRESS_REG_muxed = bist_index;
      bram_write_en_muxed = 1'b1;
      bram_data_in_muxed = {DATA_WIDTH{1'b0}};
   end
   else
   begin
      WRITE_ADDRESS_REG_muxed = A_r;
      bram_write_en_muxed = bram_wen;
      bram_data_in_muxed = bram_data_in;
   end
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

//==================================================================================================
//  Filename      : synchronizer.v
//  Created On    : 2014-01-31 12:52:57
//  Last Modified : 2018-11-29 17:02:47
//  Revision      :
//  Author        : Tri Nguyen
//  Company       : Princeton University
//  Email         : trin@princeton.edu
//
//  Description   :
//==================================================================================================





module synchronizer (
    clk,
    presyncdata,
    syncdata
    );

// synopsys template
parameter SIZE = 1;

input wire clk;
input wire [SIZE-1:0] presyncdata;
output reg [SIZE-1:0] syncdata;













  reg [SIZE-1:0] presyncdata_tmp;



    // bw_u1_syncff_4x u_synchronizer_syncff [SIZE-1:0](.q(presyncdata_tmp),
    //                  .so(),
    //                  .ck(clk),
    //                  .d(presyncdata),
    //                  .sd(),
    //                  .se(1'b0)
    //                  );

    // bw_u1_soff_2x u_synchronizer_ff[SIZE-1:0] (.q(syncdata),
    //                  .so(),
    //                  .ck(clk),
    //                  .d(presyncdata_tmp),
    //                  .sd(),
    //                  .se(1'b0)
    //                  );

always @ (posedge clk)
begin
    presyncdata_tmp <= presyncdata;
    syncdata        <= presyncdata_tmp;
end

endmodule

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























// Modified by Princeton University on June 9th, 2015
/*
* ========== Copyright Header Begin ==========================================
* 
* OpenSPARC T1 Processor File: tlu.h
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
// ifu trap types





















//
// modified for hypervisor support
//
























//


// modified due to bug 2588
// `define	TSA_PSTATE_VRANGE2_LO 16 


//











//
// added due to Niagara SRAMs methodology
// The following defines have been replaced due
// the memory macro replacement from:
// bw_r_rf32x144 -> 2x bw_r_rf32x80
/*
`define	TSA_MEM_WIDTH     144 
`define	TSA_HTSTATE_HI    142 //  3 bits 
`define	TSA_HTSTATE_LO    140 
`define	TSA_TPC_HI        138 // 47 bits 
`define	TSA_TPC_LO         92
`define	TSA_TNPC_HI        90 // 47 bits
`define	TSA_TNPC_LO        44 
`define	TSA_TSTATE_HI      40 // 29 bits 
`define	TSA_TSTATE_LO      12 
`define	TSA_TTYPE_HI        8 //  9 bits
`define	TSA_TTYPE_LO        0
`define	TSA_MEM_CWP_LO	   12
`define	TSA_MEM_CWP_HI	   14
`define	TSA_MEM_PSTATE_LO  15
`define	TSA_MEM_PSTATE_HI  22
`define	TSA_MEM_ASI_LO	   23
`define	TSA_MEM_ASI_HI	   30
`define	TSA_MEM_CCR_LO	   31
`define	TSA_MEM_CCR_HI	   38
`define	TSA_MEM_GL_LO	   39 
`define	TSA_MEM_GL_HI	   40 
*/











//











// HPSTATE position definitions within wsr






// TSTATE postition definitions within wsr







// modified due to bug 2588


// added for bug 2584 




//







//
// tick_cmp and stick_cmp definitions





//
// PIB WRAP



// HPSTATE postition definitions






// HTBA definitions




// TBA definitions




















//
// added for the hypervisor support


// modified due to bug 2588
















//
// compressed PSTATE WSR definitions














//
// ASI_QUEUE for hypervisor
// Queues are: CPU_MONODO
//             DEV_MONODO
//             RESUMABLE_ERROR
//             NON_RESUMABLE_ERROR
//







// for address range checking
















//
// Niagara scratch-pads
// VA address of 0x20 and 0x28 are exclusive to hypervisor
// 







//
// range checking 







// PIB related definitions
// Bit definition for events









// 
// PIB related definitions
// PCR and PIC address definitions



// 
// PCR bit definitions







//









// PIC definitions








// PIC  mask bit position definitions










// added define from sparc_tlu_int.v 










//
// shadow scan related definitions 

// modified due to logic redistribution
// `define TCL_SSCAN_WIDTH 12 





// `define TCL_SSCAN_LO 51 




// 
// position definitions - TDP






// 
// position definitions - TCL




// 
// To speedup POR for verification purposes

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
//  Filename      : jtag.vh
//  Created On    : 2014-01-31 12:52:57
//  Last Modified : 2015-01-28 16:54:05
//  Revision      :
//  Author        : Tri Nguyen
//  Company       : Princeton University
//  Email         : trin@princeton.edu
//
//  Description   : Parallel JTAG/debug controller defines
//==================================================================================================


/////////////////////////////////
// Chip ID in JTAG
/////////////////////////////////







/////////////////////////////////
// JTAG TAP
/////////////////////////////////
// copied from ctu.h








// `define TAP_CREG_ADDR          6'h08
// `define TAP_CREG_WDATA         6'h09
// `define TAP_CREG_RDATA         6'h0a
// `define TAP_CREG_SCRATCH       6'h0b
// `define TAP_IOB_WR             6'h0c
// `define TAP_IOB_RD             6'h0d
// `define TAP_IOB_WADDR          6'h0e
// `define TAP_IOB_WDATA          6'h0f
// `define TAP_IOB_RADDR          6'h10




// `define TAP_CREG_SCRATCH       6'h0b
// `define TAP_IOB_WR             6'h0c
// `define TAP_IOB_RD             6'h0d
// `define TAP_IOB_WADDR          6'h0e
// `define TAP_IOB_WDATA          6'h0f
// `define TAP_IOB_RADDR          6'h10

// `define TAP_MBIST_SERIAL       6'h14
// `define TAP_MBIST_PARALLEL     6'h15
// `define TAP_MBIST_RESULT       6'h16
// `define TAP_MBIST_ABORT        6'h17

// `define TAP_PLL_BYPASS         6'h18

// `define TAP_CLK_STOP_ID        6'h1a
// `define TAP_CLK_SEL            6'h1b  //mask ff00 for ck src

// `define TAP_SSCAN_T0           6'h1c
// `define TAP_SSCAN_T1           6'h1d
// `define TAP_SSCAN_T2           6'h1e
// `define TAP_SSCAN_T3           6'h1f

// `define TAP_SCAN_PARALLEL      6'h20
// `define TAP_SCAN_SERIAL        6'h21
// `define TAP_SCAN_MTEST_LONG    6'h22
// `define TAP_SCAN_MTEST_SHORT   6'h23
// `define TAP_SCAN_BYPASS_EN     6'h24
// `define TAP_SCAN_NSTEP         6'h25
// `define TAP_SCAN_DUMP          6'h26

// `define TAP_EFC_READ           6'h28 
// `define TAP_EFC_BYPASS_DATA    6'h29 
// `define TAP_EFC_BYPASS         6'h2a 
// `define TAP_EFC_READ_MODE      6'h2b 
// `define TAP_EFC_COL_ADDR       6'h2c
// `define TAP_EFC_ROW_ADDR       6'h2d
// `define TAP_EFC_DEST_SAMPLE    6'h2e





/////////////////////////////////
// CTAP register select defines
/////////////////////////////////

// `define CTAP_DATA_REG_WIDTH 64






/////////////////////////////////
// JTAG instructions
/////////////////////////////////

// header


// lengths of header





// definitions of operations
// `define JTAG_REQ_OP_READ_SHADOWSCAN 8'd1

// `define JTAG_REQ_OP_STALL_CORE 8'd3


// `define JTAG_REQ_OP_WRITE_PC 8'd6
// `define JTAG_REQ_OP_WRITE_THREADSTATE 8'd7
// `define JTAG_REQ_OP_CPX_INTERRUPT 8'd8








// definitions of misc field for read/write rtap











// `define JTAG_RTAP_ID__REG 16'd


// definitions of tileids
// `define CTAP_ID_BROADCAST 6'b111111



// mask of header




// misc is used for stall bit at bit 0

// address reg





// masks in address reg




// data reg



// // From CTAP to RTAP
// // these valid vector assumes 4b bus. so 32b vector would be 128b
// `define CTAP_REQ_VEC_WHOLE_PACKET 32'hffffffff
// // header has first 32b
// `define CTAP_REQ_VEC_HEADER 32'h000000ff
// // half is 64b, includes the addresses
// `define CTAP_REQ_VEC_HALF 32'h0000ffff





// RTAP returns






// RTAP states












// ORAM specifics



/////////////////////////////////
// UCB related
/////////////////////////////////




// CTAP_UCB_TILEID_MASK

// RTAP_INSTRUCTION_MASK
// RTAP_INSTRUCTION_RETURN_SHADOWSCAN
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

// 02/06/2015 14:58:59
// This file is auto-generated
// Author: Tri Nguyen

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
//  Filename      : l2.h.pyv
//  Created On    : 2014-02-20
//  Revision      :
//  Author        : Yaosheng Fu
//  Company       : Princeton University
//  Email         : yfu@princeton.edu
//
//  Description   : header file for the L2 cache
//
//
//==================================================================================================

// devices.xml



// Input buffer for pipeline1













// Input buffer for pipeline2














// Output buffer















// L2 cache configuration







//`define L2_SIZE                  65536
//`define L2_SIZE_WIDTH            16






//`define L2_LINE_SIZE             64
//`define L2_LINE_SIZE_WIDTH       6













//`define L2_WAYS                  4
//`define L2_WAYS_WIDTH            2
//`define L2_WAY_0                 2'b00
//`define L2_WAY_1                 2'b01
//`define L2_WAY_2                 2'b10
//`define L2_WAY_3                 2'b11





// Tag array







//`define L2_TAG_INDEX_WIDTH      8
//`define L2_TAG_WIDTH            26
//`define L2_TAG_WAY_WIDTH        26
//`define L2_TAG_ARRAY_WIDTH      104


// Tag Address decomposition 




//`define L2_TAG_INDEX            13:6
//`define L2_TAG                  39:14


//Data array












//`define L2_DATA_INDEX_WIDTH         12 








// Data Address decomposition 







//Dir array



//`define L2_DIR_INDEX_WIDTH      10 





//State array

















//Whether the cache line is in Icaches or Dcaches
















//Round Robin selection














// State decomposition 












//MSHR array



































//`define L2_MSHR_CMP_ADDR        13:6
//`define L2_MSHR_ADDR            39:0
//`define L2_MSHR_WAY             41:40
//`define L2_MSHR_MSHRID          49:42
//`define L2_MSHR_CACHE_TYPE      50
//`define L2_MSHR_DATA_SIZE       53:51
//`define L2_MSHR_MSG_TYPE        61:54
//`define L2_MSHR_L2_MISS         62
//`define L2_MSHR_SRC_CHIPID      76:63
//`define L2_MSHR_SRC_X           84:77
//`define L2_MSHR_SRC_Y           92:85
//`define L2_MSHR_SRC_FBITS       96:93
//`define L2_MSHR_SDID            106:97
//`define L2_MSHR_LSID            112:107      
//`define L2_MSHR_MISS_LSID       118:113
//`define L2_MSHR_SMC_MISS        119
//`define L2_MSHR_RECYCLED        120
//`define L2_MSHR_INV_FWD_PENDING 121


//SMC array


































//Message destination_type







//L2 public sharer beyond the maximum clump size



//L2 registers



//Special addresses





















//L2 core ID





//L2 dir array reuse







//Control Signal 








//AMO ALU OP macros












//Control Signal in Stage 1























































// devices.xml

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



























module sram_l2_data
(
input wire MEMCLK,
input wire RESET_N,
input wire CE,
input wire [12-1:0] A,
input wire RDWEN,
input wire [144-1:0] BW,
input wire [144-1:0] DIN,
output wire [144-1:0] DOUT,
input wire [4-1:0] BIST_COMMAND,
input wire [4-1:0] BIST_DIN,
output reg [4-1:0] BIST_DOUT,
input wire [8-1:0] SRAMID
);


wire [144-1:0] DOUT_bram;
assign DOUT = DOUT_bram;

bram_1rw_wrapper #(
   .NAME          (""             ),
   .DEPTH         (4096),
   .ADDR_WIDTH    (12),
   .BITMASK_WIDTH (144),
   .DATA_WIDTH    (144)
)   sram_l2_data (
   .MEMCLK        (MEMCLK     ),
   .RESET_N        (RESET_N     ),
   .CE            (CE         ),
   .A             (A          ),
   .RDWEN         (RDWEN      ),
   .BW            (BW         ),
   .DIN           (DIN        ),
   .DOUT          (DOUT_bram       )
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

// 02/06/2015 14:58:59
// This file is auto-generated
// Author: Tri Nguyen

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
//  Filename      : l2.h.pyv
//  Created On    : 2014-02-20
//  Revision      :
//  Author        : Yaosheng Fu
//  Company       : Princeton University
//  Email         : yfu@princeton.edu
//
//  Description   : header file for the L2 cache
//
//
//==================================================================================================

// devices.xml



// Input buffer for pipeline1













// Input buffer for pipeline2














// Output buffer















// L2 cache configuration







//`define L2_SIZE                  65536
//`define L2_SIZE_WIDTH            16






//`define L2_LINE_SIZE             64
//`define L2_LINE_SIZE_WIDTH       6













//`define L2_WAYS                  4
//`define L2_WAYS_WIDTH            2
//`define L2_WAY_0                 2'b00
//`define L2_WAY_1                 2'b01
//`define L2_WAY_2                 2'b10
//`define L2_WAY_3                 2'b11





// Tag array







//`define L2_TAG_INDEX_WIDTH      8
//`define L2_TAG_WIDTH            26
//`define L2_TAG_WAY_WIDTH        26
//`define L2_TAG_ARRAY_WIDTH      104


// Tag Address decomposition 




//`define L2_TAG_INDEX            13:6
//`define L2_TAG                  39:14


//Data array












//`define L2_DATA_INDEX_WIDTH         12 








// Data Address decomposition 







//Dir array



//`define L2_DIR_INDEX_WIDTH      10 





//State array

















//Whether the cache line is in Icaches or Dcaches
















//Round Robin selection














// State decomposition 












//MSHR array



































//`define L2_MSHR_CMP_ADDR        13:6
//`define L2_MSHR_ADDR            39:0
//`define L2_MSHR_WAY             41:40
//`define L2_MSHR_MSHRID          49:42
//`define L2_MSHR_CACHE_TYPE      50
//`define L2_MSHR_DATA_SIZE       53:51
//`define L2_MSHR_MSG_TYPE        61:54
//`define L2_MSHR_L2_MISS         62
//`define L2_MSHR_SRC_CHIPID      76:63
//`define L2_MSHR_SRC_X           84:77
//`define L2_MSHR_SRC_Y           92:85
//`define L2_MSHR_SRC_FBITS       96:93
//`define L2_MSHR_SDID            106:97
//`define L2_MSHR_LSID            112:107      
//`define L2_MSHR_MISS_LSID       118:113
//`define L2_MSHR_SMC_MISS        119
//`define L2_MSHR_RECYCLED        120
//`define L2_MSHR_INV_FWD_PENDING 121


//SMC array


































//Message destination_type







//L2 public sharer beyond the maximum clump size



//L2 registers



//Special addresses





















//L2 core ID





//L2 dir array reuse







//Control Signal 








//AMO ALU OP macros












//Control Signal in Stage 1























































// devices.xml

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







































































































































































































































































































































































































































































































































































































module sram_l2_dir
(
input wire MEMCLK,
input wire RESET_N,
input wire CE,
input wire [10-1:0] A,
input wire RDWEN,
input wire [64-1:0] BW,
input wire [64-1:0] DIN,
output wire [64-1:0] DOUT,
input wire [4-1:0] BIST_COMMAND,
input wire [4-1:0] BIST_DIN,
output reg [4-1:0] BIST_DOUT,
input wire [8-1:0] SRAMID
);


wire [64-1:0] DOUT_bram;
assign DOUT = DOUT_bram;

bram_1rw_wrapper #(
   .NAME          (""             ),
   .DEPTH         (1024),
   .ADDR_WIDTH    (10),
   .BITMASK_WIDTH (64),
   .DATA_WIDTH    (64)
)   sram_l2_dir (
   .MEMCLK        (MEMCLK     ),
   .RESET_N        (RESET_N     ),
   .CE            (CE         ),
   .A             (A          ),
   .RDWEN         (RDWEN      ),
   .BW            (BW         ),
   .DIN           (DIN        ),
   .DOUT          (DOUT_bram       )
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

// 02/06/2015 14:58:59
// This file is auto-generated
// Author: Tri Nguyen

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
//  Filename      : l2.h.pyv
//  Created On    : 2014-02-20
//  Revision      :
//  Author        : Yaosheng Fu
//  Company       : Princeton University
//  Email         : yfu@princeton.edu
//
//  Description   : header file for the L2 cache
//
//
//==================================================================================================

// devices.xml



// Input buffer for pipeline1













// Input buffer for pipeline2














// Output buffer















// L2 cache configuration







//`define L2_SIZE                  65536
//`define L2_SIZE_WIDTH            16






//`define L2_LINE_SIZE             64
//`define L2_LINE_SIZE_WIDTH       6













//`define L2_WAYS                  4
//`define L2_WAYS_WIDTH            2
//`define L2_WAY_0                 2'b00
//`define L2_WAY_1                 2'b01
//`define L2_WAY_2                 2'b10
//`define L2_WAY_3                 2'b11





// Tag array







//`define L2_TAG_INDEX_WIDTH      8
//`define L2_TAG_WIDTH            26
//`define L2_TAG_WAY_WIDTH        26
//`define L2_TAG_ARRAY_WIDTH      104


// Tag Address decomposition 




//`define L2_TAG_INDEX            13:6
//`define L2_TAG                  39:14


//Data array












//`define L2_DATA_INDEX_WIDTH         12 








// Data Address decomposition 







//Dir array



//`define L2_DIR_INDEX_WIDTH      10 





//State array

















//Whether the cache line is in Icaches or Dcaches
















//Round Robin selection














// State decomposition 












//MSHR array



































//`define L2_MSHR_CMP_ADDR        13:6
//`define L2_MSHR_ADDR            39:0
//`define L2_MSHR_WAY             41:40
//`define L2_MSHR_MSHRID          49:42
//`define L2_MSHR_CACHE_TYPE      50
//`define L2_MSHR_DATA_SIZE       53:51
//`define L2_MSHR_MSG_TYPE        61:54
//`define L2_MSHR_L2_MISS         62
//`define L2_MSHR_SRC_CHIPID      76:63
//`define L2_MSHR_SRC_X           84:77
//`define L2_MSHR_SRC_Y           92:85
//`define L2_MSHR_SRC_FBITS       96:93
//`define L2_MSHR_SDID            106:97
//`define L2_MSHR_LSID            112:107      
//`define L2_MSHR_MISS_LSID       118:113
//`define L2_MSHR_SMC_MISS        119
//`define L2_MSHR_RECYCLED        120
//`define L2_MSHR_INV_FWD_PENDING 121


//SMC array


































//Message destination_type







//L2 public sharer beyond the maximum clump size



//L2 registers



//Special addresses





















//L2 core ID





//L2 dir array reuse







//Control Signal 








//AMO ALU OP macros












//Control Signal in Stage 1























































// devices.xml

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







































































































































































































































































































































































































































































































































































































module sram_l2_state
(
input wire MEMCLK,
input wire RESET_N,
input wire CEA,
input wire [8-1:0] AA,
input wire RDWENA,
input wire CEB,
input wire [8-1:0] AB,
input wire RDWENB,
input wire [15*4+2+4-1:0] BWA,
input wire [15*4+2+4-1:0] DINA,
output wire [15*4+2+4-1:0] DOUTA,
input wire [15*4+2+4-1:0] BWB,
input wire [15*4+2+4-1:0] DINB,
output wire [15*4+2+4-1:0] DOUTB,
input wire [4-1:0] BIST_COMMAND,
input wire [4-1:0] BIST_DIN,
output reg [4-1:0] BIST_DOUT,
input wire [8-1:0] SRAMID
);
  

wire [15*4+2+4-1:0] DOUTA_bram;
wire [15*4+2+4-1:0] DOUTB_bram;
assign DOUTA = DOUTA_bram;
assign DOUTB = DOUTB_bram;

bram_1r1w_wrapper #(
   .NAME          (""             ),
   .DEPTH         (256),
   .ADDR_WIDTH    (8),
   .BITMASK_WIDTH (15*4+2+4),
   .DATA_WIDTH    (15*4+2+4)
)   sram_l2_state (
   .MEMCLK        (MEMCLK     ),
   .RESET_N        (RESET_N     ),
   .CEA        (CEA     ),
   .AA        (AA     ),
   .AB        (AB     ),
   .RDWENA        (RDWENA     ),
   .CEB        (CEB     ),
   .RDWENB        (RDWENB     ),
   .BWA        (BWA     ),
   .DINA        (DINA     ),
   .DOUTA        (DOUTA_bram     ),
   .BWB        (BWB     ),
   .DINB        (DINB     ),
   .DOUTB        (DOUTB_bram     )
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

// 02/06/2015 14:58:59
// This file is auto-generated
// Author: Tri Nguyen

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
//  Filename      : l2.h.pyv
//  Created On    : 2014-02-20
//  Revision      :
//  Author        : Yaosheng Fu
//  Company       : Princeton University
//  Email         : yfu@princeton.edu
//
//  Description   : header file for the L2 cache
//
//
//==================================================================================================

// devices.xml



// Input buffer for pipeline1













// Input buffer for pipeline2














// Output buffer















// L2 cache configuration







//`define L2_SIZE                  65536
//`define L2_SIZE_WIDTH            16






//`define L2_LINE_SIZE             64
//`define L2_LINE_SIZE_WIDTH       6













//`define L2_WAYS                  4
//`define L2_WAYS_WIDTH            2
//`define L2_WAY_0                 2'b00
//`define L2_WAY_1                 2'b01
//`define L2_WAY_2                 2'b10
//`define L2_WAY_3                 2'b11





// Tag array







//`define L2_TAG_INDEX_WIDTH      8
//`define L2_TAG_WIDTH            26
//`define L2_TAG_WAY_WIDTH        26
//`define L2_TAG_ARRAY_WIDTH      104


// Tag Address decomposition 




//`define L2_TAG_INDEX            13:6
//`define L2_TAG                  39:14


//Data array












//`define L2_DATA_INDEX_WIDTH         12 








// Data Address decomposition 







//Dir array



//`define L2_DIR_INDEX_WIDTH      10 





//State array

















//Whether the cache line is in Icaches or Dcaches
















//Round Robin selection














// State decomposition 












//MSHR array



































//`define L2_MSHR_CMP_ADDR        13:6
//`define L2_MSHR_ADDR            39:0
//`define L2_MSHR_WAY             41:40
//`define L2_MSHR_MSHRID          49:42
//`define L2_MSHR_CACHE_TYPE      50
//`define L2_MSHR_DATA_SIZE       53:51
//`define L2_MSHR_MSG_TYPE        61:54
//`define L2_MSHR_L2_MISS         62
//`define L2_MSHR_SRC_CHIPID      76:63
//`define L2_MSHR_SRC_X           84:77
//`define L2_MSHR_SRC_Y           92:85
//`define L2_MSHR_SRC_FBITS       96:93
//`define L2_MSHR_SDID            106:97
//`define L2_MSHR_LSID            112:107      
//`define L2_MSHR_MISS_LSID       118:113
//`define L2_MSHR_SMC_MISS        119
//`define L2_MSHR_RECYCLED        120
//`define L2_MSHR_INV_FWD_PENDING 121


//SMC array


































//Message destination_type







//L2 public sharer beyond the maximum clump size



//L2 registers



//Special addresses





















//L2 core ID





//L2 dir array reuse







//Control Signal 








//AMO ALU OP macros












//Control Signal in Stage 1























































// devices.xml

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







































































































































































































































































































































































































































































































































































































module sram_l2_tag
(
input wire MEMCLK,
input wire RESET_N,
input wire CE,
input wire [8-1:0] A,
input wire RDWEN,
input wire [104-1:0] BW,
input wire [104-1:0] DIN,
output wire [104-1:0] DOUT,
input wire [4-1:0] BIST_COMMAND,
input wire [4-1:0] BIST_DIN,
output reg [4-1:0] BIST_DOUT,
input wire [8-1:0] SRAMID
);


wire [104-1:0] DOUT_bram;
assign DOUT = DOUT_bram;

bram_1rw_wrapper #(
   .NAME          (""             ),
   .DEPTH         (256),
   .ADDR_WIDTH    (8),
   .BITMASK_WIDTH (104),
   .DATA_WIDTH    (104)
)   sram_l2_tag (
   .MEMCLK        (MEMCLK     ),
   .RESET_N        (RESET_N     ),
   .CE            (CE         ),
   .A             (A          ),
   .RDWEN         (RDWEN      ),
   .BW            (BW         ),
   .DIN           (DIN        ),
   .DOUT          (DOUT_bram       )
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
//  Filename      : l2.v
//  Created On    : 2014-02-20
//  Revision      :
//  Author        : Yaosheng Fu
//  Company       : Princeton University
//  Email         : yfu@princeton.edu
//
//  Description   : main verilog file for the L2 cache
//
//
//==================================================================================================

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
//  Filename      : l2.h.pyv
//  Created On    : 2014-02-20
//  Revision      :
//  Author        : Yaosheng Fu
//  Company       : Princeton University
//  Email         : yfu@princeton.edu
//
//  Description   : header file for the L2 cache
//
//
//==================================================================================================

// devices.xml



// Input buffer for pipeline1













// Input buffer for pipeline2














// Output buffer















// L2 cache configuration







//`define L2_SIZE                  65536
//`define L2_SIZE_WIDTH            16






//`define L2_LINE_SIZE             64
//`define L2_LINE_SIZE_WIDTH       6













//`define L2_WAYS                  4
//`define L2_WAYS_WIDTH            2
//`define L2_WAY_0                 2'b00
//`define L2_WAY_1                 2'b01
//`define L2_WAY_2                 2'b10
//`define L2_WAY_3                 2'b11





// Tag array







//`define L2_TAG_INDEX_WIDTH      8
//`define L2_TAG_WIDTH            26
//`define L2_TAG_WAY_WIDTH        26
//`define L2_TAG_ARRAY_WIDTH      104


// Tag Address decomposition 




//`define L2_TAG_INDEX            13:6
//`define L2_TAG                  39:14


//Data array












//`define L2_DATA_INDEX_WIDTH         12 








// Data Address decomposition 







//Dir array



//`define L2_DIR_INDEX_WIDTH      10 





//State array

















//Whether the cache line is in Icaches or Dcaches
















//Round Robin selection














// State decomposition 












//MSHR array



































//`define L2_MSHR_CMP_ADDR        13:6
//`define L2_MSHR_ADDR            39:0
//`define L2_MSHR_WAY             41:40
//`define L2_MSHR_MSHRID          49:42
//`define L2_MSHR_CACHE_TYPE      50
//`define L2_MSHR_DATA_SIZE       53:51
//`define L2_MSHR_MSG_TYPE        61:54
//`define L2_MSHR_L2_MISS         62
//`define L2_MSHR_SRC_CHIPID      76:63
//`define L2_MSHR_SRC_X           84:77
//`define L2_MSHR_SRC_Y           92:85
//`define L2_MSHR_SRC_FBITS       96:93
//`define L2_MSHR_SDID            106:97
//`define L2_MSHR_LSID            112:107      
//`define L2_MSHR_MISS_LSID       118:113
//`define L2_MSHR_SMC_MISS        119
//`define L2_MSHR_RECYCLED        120
//`define L2_MSHR_INV_FWD_PENDING 121


//SMC array


































//Message destination_type







//L2 public sharer beyond the maximum clump size



//L2 registers



//Special addresses





















//L2 core ID





//L2 dir array reuse







//Control Signal 








//AMO ALU OP macros












//Control Signal in Stage 1























































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





































































































































































































































































































































































































































































































































































































module l2(

    input wire clk,
    input wire rst_n,

    input wire [14-1:0] chipid,
    input wire [8-1:0] coreid_x,
    input wire [8-1:0] coreid_y,

    input wire noc1_valid_in,
    input wire [64-1:0] noc1_data_in,
    output wire noc1_ready_in,


    input wire noc3_valid_in,
    input wire [64-1:0] noc3_data_in,
    output wire noc3_ready_in,

    output wire noc2_valid_out,
    output wire [64-1:0] noc2_data_out,
    input wire noc2_ready_out,

    // sram interface
    output wire [4-1:0] srams_rtap_data,
    input wire  [4-1:0] rtap_srams_bist_command,
    input wire  [4-1:0] rtap_srams_bist_data
);

// sram
wire [4-1:0] data_rtap_data;
wire [4-1:0] dir_rtap_data;
wire [4-1:0] tag_rtap_data;
wire [4-1:0] state_rtap_data;
assign srams_rtap_data = data_rtap_data
                            | dir_rtap_data
                            | tag_rtap_data
                            | state_rtap_data;

localparam y = 1'b1;
localparam n = 1'b0;


wire mshr_cam_en_p1;
wire mshr_wr_state_en_p1;
wire mshr_wr_data_en_p1;
wire mshr_pending_ready_p1;
wire [2-1:0] mshr_state_in_p1;
wire [120+2-1:0] mshr_data_in_p1;
wire [120+2-1:0] mshr_data_mask_in_p1;
wire [3-1:0] mshr_inv_counter_rd_index_in_p1;
wire [3-1:0] mshr_wr_index_in_p1;
wire [8-1:0] mshr_addr_in_p1;

wire mshr_rd_en_p2;
wire mshr_wr_state_en_p2;
wire mshr_wr_data_en_p2;
wire mshr_inc_counter_en_p2;
wire [2-1:0] mshr_state_in_p2;
wire [120+2-1:0] mshr_data_in_p2;
wire [120+2-1:0] mshr_data_mask_in_p2;
wire [3-1:0] mshr_rd_index_in_p2;
wire [3-1:0] mshr_wr_index_in_p2;

wire mshr_hit;
wire [3-1:0] mshr_hit_index;

wire [2-1:0] mshr_state_out;
wire [120+2-1:0] mshr_data_out;








wire [6-1:0] mshr_inv_counter_out;
wire [3:0] mshr_empty_slots;
wire mshr_pending;
wire [3-1:0] mshr_pending_index;
wire [3-1:0] mshr_empty_index;

wire state_rd_en_p1;
wire state_wr_en_p1;
wire [8-1:0] state_rd_addr_p1;
wire [8-1:0] state_wr_addr_p1;
wire [15*4+2+4-1:0] state_data_in_p1;
wire [15*4+2+4-1:0] state_data_mask_in_p1;

wire state_rd_en_p2;
wire state_wr_en_p2;
wire [8-1:0] state_rd_addr_p2;
wire [8-1:0] state_wr_addr_p2;
wire [15*4+2+4-1:0] state_data_in_p2;
wire [15*4+2+4-1:0] state_data_mask_in_p2;

wire [15*4+2+4-1:0] state_data_out;

wire tag_clk_en_p1;
wire tag_rdw_en_p1;
wire [8-1:0] tag_addr_p1;
wire [104-1:0] tag_data_in_p1;
wire [104-1:0] tag_data_mask_in_p1;

wire tag_clk_en_p2;
wire tag_rdw_en_p2;
wire [8-1:0] tag_addr_p2;
wire [104-1:0] tag_data_in_p2;
wire [104-1:0] tag_data_mask_in_p2;

wire [104-1:0] tag_data_out;

wire dir_clk_en_p1;
wire dir_rdw_en_p1;
wire [8+2-1:0] dir_addr_p1;
wire [64-1:0] dir_data_in_p1;
wire [64-1:0] dir_data_mask_in_p1;

wire dir_clk_en_p2;
wire dir_rdw_en_p2;
wire [8+2-1:0] dir_addr_p2;
wire [64-1:0] dir_data_in_p2;
wire [64-1:0] dir_data_mask_in_p2;

wire [64-1:0] dir_data_out;

wire data_clk_en_p1;
wire data_rdw_en_p1;
wire [8+2+2-1:0] data_addr_p1;
wire [144-1:0] data_data_in_p1;
wire [144-1:0] data_data_mask_in_p1;

wire data_clk_en_p2;
wire data_rdw_en_p2;
wire [8+2+2-1:0] data_addr_p2;
wire [144-1:0] data_data_in_p2;
wire [144-1:0] data_data_mask_in_p2;
wire [144-1:0] data_data_out;


wire smc_rd_en;
wire smc_rd_diag_en;
wire smc_wr_diag_en;
wire smc_flush_en;
wire [2-1:0] smc_addr_op;
wire [16-1:0] smc_rd_addr_in;

wire smc_wr_en_p1;
wire [16-1:0] smc_wr_addr_in_p1;
wire [128-1:0] smc_data_in_p1;

wire smc_wr_en_p2;
wire [16-1:0] smc_wr_addr_in_p2;
wire [128-1:0] smc_data_in_p2;

wire [2-1:0] broadcast_counter_op_p1;
wire broadcast_counter_op_val_p1;

wire [2-1:0] broadcast_counter_op_p2;
wire broadcast_counter_op_val_p2;

wire smc_hit;
wire [30-1:0] smc_data_out;
wire [4-1:0] smc_valid_out;
wire [14-1:0] smc_tag_out;

wire broadcast_counter_zero1;
wire broadcast_counter_max1;
wire broadcast_counter_avail1;
wire [14-1:0] broadcast_chipid_out1;
wire [8-1:0] broadcast_x_out1;
wire [8-1:0] broadcast_y_out1;

wire broadcast_counter_zero2;
wire broadcast_counter_max2;
wire broadcast_counter_avail2;
wire [14-1:0] broadcast_chipid_out2;
wire [8-1:0] broadcast_x_out2;
wire [8-1:0] broadcast_y_out2;


wire reg_rd_en;
wire reg_wr_en;
wire [8-1:0] reg_rd_addr_type;
wire [8-1:0] reg_wr_addr_type;
wire [64-1:0] reg_data_out;
wire [64-1:0] reg_data_in;
wire l2_access_valid;
wire l2_miss_valid;
wire data_ecc_corr_error;
wire data_ecc_uncorr_error;
wire [8+2+2-1:0] data_ecc_addr;
wire [40-1:0] error_addr;
wire [34-1:0] my_nodeid;
wire [14+8+8-1:0] core_max;

wire csm_en;

wire [22-1:0] smt_base_addr;


wire pipe2_valid_S1;
wire pipe2_valid_S2;
wire pipe2_valid_S3;



wire [8-1:0] pipe2_msg_type_S1;
wire [8-1:0] pipe2_msg_type_S2;
wire [8-1:0] pipe2_msg_type_S3;

wire [40-1:0] pipe2_addr_S1;
wire [40-1:0] pipe2_addr_S2;
wire [40-1:0] pipe2_addr_S3;

wire active_S1;
wire active_S2;
wire active_S3;





l2_config_regs config_regs(

    .clk                    (clk),
    .rst_n                  (rst_n),
    .chipid                 (chipid),
    .coreid_x               (coreid_x),
    .coreid_y               (coreid_y),
    .l2_access_valid        (l2_access_valid),
    .l2_miss_valid          (l2_miss_valid),
    .data_ecc_corr_error    (data_ecc_corr_error),
    .data_ecc_uncorr_error  (data_ecc_uncorr_error),
    .data_ecc_addr          (data_ecc_addr),
    .error_addr             (error_addr),
    .reg_rd_en              (reg_rd_en),
    .reg_wr_en              (reg_wr_en),
    .reg_rd_addr_type       (reg_rd_addr_type),
    .reg_wr_addr_type       (reg_wr_addr_type),
    .reg_data_in            (reg_data_in),

    .reg_data_out           (reg_data_out),
    .my_nodeid              (my_nodeid),
    .core_max               (core_max),
    
    .csm_en                 (csm_en),
    


    .smt_base_addr          (smt_base_addr)

);




l2_mshr_wrap mshr_wrap(
    .clk                    (clk),
    .rst_n                  (rst_n),

    .pipe_rd_sel            (active_S1),

    .pipe_wr_sel            (active_S3),


    .rd_en1                 (1'b0),

    .cam_en1                (mshr_cam_en_p1),
    .wr_state_en1           (mshr_wr_state_en_p1),
    .wr_data_en1            (mshr_wr_data_en_p1),
    .pending_ready1         (mshr_pending_ready_p1),
    .state_in1              (mshr_state_in_p1),
    .data_in1               (mshr_data_in_p1),
    .data_mask_in1          (mshr_data_mask_in_p1),

    .rd_index_in1           ({3{1'b0}}),

    .inv_counter_rd_index_in1(mshr_inv_counter_rd_index_in_p1),
    .wr_index_in1           (mshr_wr_index_in_p1),
    .addr_in1               (mshr_addr_in_p1),


    .rd_en2                 (mshr_rd_en_p2),
    .cam_en2                (1'b0),

    .wr_state_en2           (mshr_wr_state_en_p2),
    .wr_data_en2            (mshr_wr_data_en_p2),

    .pending_ready2         (1'b0), 

    .inc_counter_en2        (mshr_inc_counter_en_p2),
    .state_in2              (mshr_state_in_p2),
    .data_in2               (mshr_data_in_p2),
    .data_mask_in2          (mshr_data_mask_in_p2),
    .rd_index_in2           (mshr_rd_index_in_p2),
    .wr_index_in2           (mshr_wr_index_in_p2),

    .addr_in2               ({8{1'b0}}),


    .hit                    (mshr_hit),
    .hit_index              (mshr_hit_index),

    .state_out              (mshr_state_out),
    .data_out               (mshr_data_out),







    .inv_counter_out        (mshr_inv_counter_out), 
    .empty_slots            (mshr_empty_slots),
    .pending                (mshr_pending),
    .pending_index          (mshr_pending_index),
    .empty_index            (mshr_empty_index)
);

l2_state_wrap state_wrap(
    .clk                    (clk),
    .rst_n                  (rst_n),
    .pdout_en               (1'b0),
    .deepsleep              (1'b0),
    .pipe_rd_sel            (active_S1),
    .pipe_wr_sel            (active_S3),

    .rd_en1                 (state_rd_en_p1),
    .wr_en1                 (state_wr_en_p1),
    .rd_addr1               (state_rd_addr_p1),
    .wr_addr1               (state_wr_addr_p1),
    .data_in1               (state_data_in_p1),
    .data_mask_in1          (state_data_mask_in_p1),

    .rd_en2                 (state_rd_en_p2),
    .wr_en2                 (state_wr_en_p2),
    .rd_addr2               (state_rd_addr_p2),
    .wr_addr2               (state_wr_addr_p2),
    .data_in2               (state_data_in_p2),
    .data_mask_in2          (state_data_mask_in_p2),

    .data_out               (state_data_out),
    .pdata_out              (),

    // sram interfaces
    .srams_rtap_data (state_rtap_data),
    .rtap_srams_bist_command (rtap_srams_bist_command),
    .rtap_srams_bist_data (rtap_srams_bist_data)
);

l2_tag_wrap tag_wrap(
    .clk                    (clk),
    .rst_n                  (rst_n),
    .pdout_en               (1'b0),
    .deepsleep              (1'b0),
    .pipe_sel               (active_S1),

    .clk_en1                (tag_clk_en_p1),
    .rdw_en1                (tag_rdw_en_p1),
    .addr1                  (tag_addr_p1),
    .data_in1               (tag_data_in_p1),
    .data_mask_in1          (tag_data_mask_in_p1),

    .clk_en2                (tag_clk_en_p2),
    .rdw_en2                (tag_rdw_en_p2),
    .addr2                  (tag_addr_p2),
    .data_in2               (tag_data_in_p2),
    .data_mask_in2          (tag_data_mask_in_p2),


    .data_out               (tag_data_out),
    .pdata_out              (),

    // sram interfaces
    .srams_rtap_data (tag_rtap_data),
    .rtap_srams_bist_command (rtap_srams_bist_command),
    .rtap_srams_bist_data (rtap_srams_bist_data)
);

l2_dir_wrap dir_wrap(
    .clk                    (clk),
    .rst_n                  (rst_n),
    .pdout_en               (1'b0),
    .deepsleep              (1'b0),
    .pipe_sel               (active_S2),

    .clk_en1                (dir_clk_en_p1),
    .rdw_en1                (dir_rdw_en_p1),
    .addr1                  (dir_addr_p1),
    .data_in1               (dir_data_in_p1),
    .data_mask_in1          (dir_data_mask_in_p1),

    .clk_en2                (dir_clk_en_p2),
    .rdw_en2                (dir_rdw_en_p2),
    .addr2                  (dir_addr_p2),
    .data_in2               (dir_data_in_p2),
    .data_mask_in2          (dir_data_mask_in_p2),


    .data_out               (dir_data_out),
    .pdata_out              (),

    // sram interfaces
    .srams_rtap_data (dir_rtap_data),
    .rtap_srams_bist_command (rtap_srams_bist_command),
    .rtap_srams_bist_data (rtap_srams_bist_data)
);

l2_data_wrap data_wrap(
    .clk                    (clk),
    .rst_n                  (rst_n),
    .pdout_en               (1'b0),
    .deepsleep              (1'b0),
    .pipe_sel               (active_S2),

    .clk_en1                (data_clk_en_p1),
    .rdw_en1                (data_rdw_en_p1),
    .addr1                  (data_addr_p1),
    .data_in1               (data_data_in_p1),
    .data_mask_in1          (data_data_mask_in_p1),

    .clk_en2                (data_clk_en_p2),
    .rdw_en2                (data_rdw_en_p2),
    .addr2                  (data_addr_p2),
    .data_in2               (data_data_in_p2),
    .data_mask_in2          (data_data_mask_in_p2),


    .data_out               (data_data_out),
    .pdata_out              (),

    // sram interfaces
    .srams_rtap_data (data_rtap_data),
    .rtap_srams_bist_command (rtap_srams_bist_command),
    .rtap_srams_bist_data (rtap_srams_bist_data)
);


l2_smc_wrap smc_wrap(
    .clk                    (clk),
    .rst_n                  (rst_n),
    .pipe_sel               (active_S2),
    .rd_en                  (smc_rd_en),
    .rd_diag_en             (smc_rd_diag_en),
    .flush_en               (smc_flush_en),
    .addr_op                (smc_addr_op),
    .rd_addr_in             (smc_rd_addr_in),

    .wr_en1                 (smc_wr_en_p1),
    .wr_addr_in1            (smc_wr_addr_in_p1),
    .data_in1               (smc_data_in_p1),
    .wr_diag_en1            (smc_wr_diag_en),

    .wr_en2                 (smc_wr_en_p2),
    .wr_addr_in2            (smc_wr_addr_in_p2),
    .data_in2               (smc_data_in_p2),
    .wr_diag_en2            (1'b0),

    .hit                    (smc_hit),
    .data_out               (smc_data_out),
    .valid_out              (smc_valid_out),
    .tag_out                (smc_tag_out)
);

l2_broadcast_counter_wrap l2_broadcast_counter_wrap(
    .clk                    (clk),
    .rst_n                  (rst_n),
    .chipid_max             (core_max[29:16]),
    .x_max                  (core_max[7:0]),
    .y_max                  (core_max[15:8]),
    .pipe_sel               (active_S2),

    .counter_op1            (broadcast_counter_op_p1),
    .counter_op_val1        (broadcast_counter_op_val_p1),

    .counter_op2            (broadcast_counter_op_p2),
    .counter_op_val2        (broadcast_counter_op_val_p2),

    .zero1                  (broadcast_counter_zero1),
    .max1                   (broadcast_counter_max1),
    .avail1                 (broadcast_counter_avail1),
    .chipid_out1            (broadcast_chipid_out1),
    .x_out1                 (broadcast_x_out1),
    .y_out1                 (broadcast_y_out1),

    .zero2                  (broadcast_counter_zero2),
    .max2                   (broadcast_counter_max2),
    .avail2                 (broadcast_counter_avail2),
    .chipid_out2            (broadcast_chipid_out2),
    .x_out2                 (broadcast_x_out2),
    .y_out2                 (broadcast_y_out2)
);



l2_pipe1 pipe1(
    .clk                    (clk),
    .rst_n                  (rst_n),
    .my_nodeid              (my_nodeid),
    
    .csm_en                 (csm_en),
    
    .smt_base_addr          (smt_base_addr),

    .noc_valid_in           (noc1_valid_in),
    .noc_data_in            (noc1_data_in),
    .noc_ready_in           (noc1_ready_in),

    .noc_valid_out          (noc2_valid_out),
    .noc_data_out           (noc2_data_out),
    .noc_ready_out          (noc2_ready_out),


    .pipe2_valid_S1         (pipe2_valid_S1),
    .pipe2_valid_S2         (pipe2_valid_S2),
    .pipe2_valid_S3         (pipe2_valid_S3),
    .pipe2_msg_type_S1      (pipe2_msg_type_S1),
    .pipe2_msg_type_S2      (pipe2_msg_type_S2),
    .pipe2_msg_type_S3      (pipe2_msg_type_S3),
    .pipe2_addr_S1          (pipe2_addr_S1),
    .pipe2_addr_S2          (pipe2_addr_S2),
    .pipe2_addr_S3          (pipe2_addr_S3),
    .global_stall_S1        (active_S1),
    .global_stall_S2        (active_S2),
    .global_stall_S4        (active_S3),

    .mshr_hit               (mshr_hit),

    .mshr_data_out          (mshr_data_out),




    .mshr_inv_counter_out   (mshr_inv_counter_out),
    .mshr_empty_slots       (mshr_empty_slots),
    .mshr_pending           (mshr_pending),
    .mshr_pending_index     (mshr_pending_index),
    .mshr_empty_index       (mshr_empty_index),

    
    .broadcast_counter_zero (broadcast_counter_zero1),
    .broadcast_counter_max  (broadcast_counter_max1),
    .broadcast_counter_avail(broadcast_counter_avail1),
    .broadcast_chipid_out   (broadcast_chipid_out1),
    .broadcast_x_out        (broadcast_x_out1),
    .broadcast_y_out        (broadcast_y_out1),
    

    .state_data_out         (state_data_out),
    .tag_data_out           (tag_data_out),
    .dir_data_out           (dir_data_out),
    .data_data_out          (data_data_out),

    .l2_access_valid        (l2_access_valid),
    .l2_miss_valid          (l2_miss_valid),
    .data_ecc_corr_error    (data_ecc_corr_error),
    .data_ecc_uncorr_error  (data_ecc_uncorr_error),
    .data_ecc_addr          (data_ecc_addr),
    .error_addr             (error_addr),

    .reg_rd_en              (reg_rd_en),
    .reg_wr_en              (reg_wr_en),
    .reg_rd_addr_type       (reg_rd_addr_type),
    .reg_wr_addr_type       (reg_wr_addr_type),

    .reg_data_out           (reg_data_out),
    .reg_data_in            (reg_data_in),

    .mshr_cam_en            (mshr_cam_en_p1),
    .mshr_wr_state_en       (mshr_wr_state_en_p1),
    .mshr_wr_data_en        (mshr_wr_data_en_p1),
    .mshr_pending_ready     (mshr_pending_ready_p1),
    .mshr_state_in          (mshr_state_in_p1),
    .mshr_data_in           (mshr_data_in_p1),
    .mshr_data_mask_in      (mshr_data_mask_in_p1),
    .mshr_inv_counter_rd_index_in(mshr_inv_counter_rd_index_in_p1),
    .mshr_wr_index_in       (mshr_wr_index_in_p1),
    .mshr_addr_in           (mshr_addr_in_p1),

    .state_rd_en            (state_rd_en_p1),
    .state_wr_en            (state_wr_en_p1),
    .state_rd_addr          (state_rd_addr_p1),
    .state_wr_addr          (state_wr_addr_p1),
    .state_data_in          (state_data_in_p1),
    .state_data_mask_in     (state_data_mask_in_p1),

    .tag_clk_en             (tag_clk_en_p1),
    .tag_rdw_en             (tag_rdw_en_p1),
    .tag_addr               (tag_addr_p1),
    .tag_data_in            (tag_data_in_p1),
    .tag_data_mask_in       (tag_data_mask_in_p1),

    .dir_clk_en             (dir_clk_en_p1),
    .dir_rdw_en             (dir_rdw_en_p1),
    .dir_addr               (dir_addr_p1),
    .dir_data_in            (dir_data_in_p1),
    .dir_data_mask_in       (dir_data_mask_in_p1),

    
    .broadcast_counter_op   (broadcast_counter_op_p1),
    .broadcast_counter_op_val(broadcast_counter_op_val_p1),

    .smc_rd_en              (smc_rd_en),
    .smc_rd_addr_in         (smc_rd_addr_in),
    .smc_rd_diag_en         (smc_rd_diag_en),
    .smc_flush_en           (smc_flush_en),
    .smc_addr_op            (smc_addr_op),
    .smc_wr_diag_en         (smc_wr_diag_en),
    .smc_wr_en              (smc_wr_en_p1),
    .smc_wr_addr_in         (smc_wr_addr_in_p1),
    .smc_data_in            (smc_data_in_p1),
    .smc_hit                (smc_hit),
    .smc_data_out           (smc_data_out),
    .smc_valid_out          (smc_valid_out),
    .smc_tag_out            (smc_tag_out),
    

    .data_clk_en            (data_clk_en_p1),
    .data_rdw_en            (data_rdw_en_p1),
    .data_addr              (data_addr_p1),
    .data_data_in           (data_data_in_p1),
    .data_data_mask_in      (data_data_mask_in_p1)

);



l2_pipe2 pipe2(
    .clk                    (clk),
    .rst_n                  (rst_n),
    
    .csm_en                 (csm_en),
    
    .noc_valid_in           (noc3_valid_in),
    .noc_data_in            (noc3_data_in),
    .noc_ready_in           (noc3_ready_in),


    .mshr_state_out         (mshr_state_out),
    .mshr_data_out          (mshr_data_out),





    
    .broadcast_counter_zero (broadcast_counter_zero2),
    .broadcast_counter_max  (broadcast_counter_max2),
    .broadcast_chipid_out   (broadcast_chipid_out2),
    .broadcast_x_out        (broadcast_x_out2),
    .broadcast_y_out        (broadcast_y_out2),
    

    .state_data_out         (state_data_out),
    .tag_data_out           (tag_data_out),
    .dir_data_out           (dir_data_out),

    .mshr_rd_en             (mshr_rd_en_p2),
    .mshr_wr_state_en       (mshr_wr_state_en_p2),
    .mshr_wr_data_en        (mshr_wr_data_en_p2),
    .mshr_inc_counter_en    (mshr_inc_counter_en_p2),
    .mshr_state_in          (mshr_state_in_p2),
    .mshr_data_in           (mshr_data_in_p2),
    .mshr_data_mask_in      (mshr_data_mask_in_p2),
    .mshr_rd_index_in       (mshr_rd_index_in_p2),
    .mshr_wr_index_in       (mshr_wr_index_in_p2),

    .state_rd_en            (state_rd_en_p2),
    .state_wr_en            (state_wr_en_p2),
    .state_rd_addr          (state_rd_addr_p2),
    .state_wr_addr          (state_wr_addr_p2),
    .state_data_in          (state_data_in_p2),
    .state_data_mask_in     (state_data_mask_in_p2),

    .tag_clk_en             (tag_clk_en_p2),
    .tag_rdw_en             (tag_rdw_en_p2),
    .tag_addr               (tag_addr_p2),
    .tag_data_in            (tag_data_in_p2),
    .tag_data_mask_in       (tag_data_mask_in_p2),

    .dir_clk_en             (dir_clk_en_p2),
    .dir_rdw_en             (dir_rdw_en_p2),
    .dir_addr               (dir_addr_p2),
    .dir_data_in            (dir_data_in_p2),
    .dir_data_mask_in       (dir_data_mask_in_p2),

    .data_clk_en            (data_clk_en_p2),
    .data_rdw_en            (data_rdw_en_p2),
    .data_addr              (data_addr_p2),
    .data_data_in           (data_data_in_p2),
    .data_data_mask_in      (data_data_mask_in_p2),


    
    .broadcast_counter_op   (broadcast_counter_op_p2),
    .broadcast_counter_op_val(broadcast_counter_op_val_p2),

    .smc_wr_en              (smc_wr_en_p2),
    .smc_wr_addr_in         (smc_wr_addr_in_p2),
    .smc_data_in            (smc_data_in_p2),
    

    .valid_S1               (pipe2_valid_S1),
    .valid_S2               (pipe2_valid_S2),
    .valid_S3               (pipe2_valid_S3),
    .msg_type_S1            (pipe2_msg_type_S1),
    .msg_type_S2            (pipe2_msg_type_S2),
    .msg_type_S3            (pipe2_msg_type_S3),
    .addr_S1                (pipe2_addr_S1),
    .addr_S2                (pipe2_addr_S2),
    .addr_S3                (pipe2_addr_S3),
    .active_S1              (active_S1),
    .active_S2              (active_S2),
    .active_S3              (active_S3)
);

endmodule
/*
Copyright (c) 2018 Princeton University
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

//==================================================================================================
//  Filename      : l2.h.pyv
//  Created On    : 2014-02-20
//  Revision      :
//  Author        : Yaosheng Fu
//  Company       : Princeton University
//  Email         : yfu@princeton.edu
//
//  Description   : header file for the L2 cache
//
//
//==================================================================================================

// devices.xml



// Input buffer for pipeline1













// Input buffer for pipeline2














// Output buffer















// L2 cache configuration







//`define L2_SIZE                  65536
//`define L2_SIZE_WIDTH            16






//`define L2_LINE_SIZE             64
//`define L2_LINE_SIZE_WIDTH       6













//`define L2_WAYS                  4
//`define L2_WAYS_WIDTH            2
//`define L2_WAY_0                 2'b00
//`define L2_WAY_1                 2'b01
//`define L2_WAY_2                 2'b10
//`define L2_WAY_3                 2'b11





// Tag array







//`define L2_TAG_INDEX_WIDTH      8
//`define L2_TAG_WIDTH            26
//`define L2_TAG_WAY_WIDTH        26
//`define L2_TAG_ARRAY_WIDTH      104


// Tag Address decomposition 




//`define L2_TAG_INDEX            13:6
//`define L2_TAG                  39:14


//Data array












//`define L2_DATA_INDEX_WIDTH         12 








// Data Address decomposition 







//Dir array



//`define L2_DIR_INDEX_WIDTH      10 





//State array

















//Whether the cache line is in Icaches or Dcaches
















//Round Robin selection














// State decomposition 












//MSHR array



































//`define L2_MSHR_CMP_ADDR        13:6
//`define L2_MSHR_ADDR            39:0
//`define L2_MSHR_WAY             41:40
//`define L2_MSHR_MSHRID          49:42
//`define L2_MSHR_CACHE_TYPE      50
//`define L2_MSHR_DATA_SIZE       53:51
//`define L2_MSHR_MSG_TYPE        61:54
//`define L2_MSHR_L2_MISS         62
//`define L2_MSHR_SRC_CHIPID      76:63
//`define L2_MSHR_SRC_X           84:77
//`define L2_MSHR_SRC_Y           92:85
//`define L2_MSHR_SRC_FBITS       96:93
//`define L2_MSHR_SDID            106:97
//`define L2_MSHR_LSID            112:107      
//`define L2_MSHR_MISS_LSID       118:113
//`define L2_MSHR_SMC_MISS        119
//`define L2_MSHR_RECYCLED        120
//`define L2_MSHR_INV_FWD_PENDING 121


//SMC array


































//Message destination_type







//L2 public sharer beyond the maximum clump size



//L2 registers



//Special addresses





















//L2 core ID





//L2 dir array reuse







//Control Signal 








//AMO ALU OP macros












//Control Signal in Stage 1
























































module l2_amo_alu #(
  parameter SWAP_ENDIANESS = 1
) (
  input      [4-1:0] amo_alu_op,
  input      [40-1:0]      address,
  input      [3-1:0] data_size,
  input      [128-1:0]  memory_operand,
  input      [128-1:0]  cpu_operand,
  output reg [128-1:0]  amo_result
);

wire [63:0] amo_operand_a_mux, amo_operand_b_mux;
wire [63:0] amo_operand_a_swp, amo_operand_b_swp;
reg  [63:0] amo_operand_a, amo_operand_b;
reg  [63:0] amo_64b_tmp, amo_64b_result;
reg  [64:0] adder_operand_a, adder_operand_b;
wire [64:0] adder_sum;

// note: the L2_DATA_DATA_WIDTH_LOG2 is calculated for a bit width
// so we have to subtract 6 from it in order to get the a dword index
wire [7-7:0] dword_offset;


// select dword to operate on
assign dword_offset      = address[7-4:3];
assign amo_operand_a_mux = memory_operand[dword_offset*64 +: 64];
assign amo_operand_b_mux = cpu_operand[dword_offset*64 +: 64];

// endianess swap (if needed)
generate
  if (SWAP_ENDIANESS) begin : g_swap_in
    assign amo_operand_a_swp = {amo_operand_a_mux[ 0 +:8],
                                amo_operand_a_mux[ 8 +:8],
                                amo_operand_a_mux[16 +:8],
                                amo_operand_a_mux[24 +:8],
                                amo_operand_a_mux[32 +:8],
                                amo_operand_a_mux[40 +:8],
                                amo_operand_a_mux[48 +:8],
                                amo_operand_a_mux[56 +:8]};

    assign amo_operand_b_swp = {amo_operand_b_mux[ 0 +:8],
                                amo_operand_b_mux[ 8 +:8],
                                amo_operand_b_mux[16 +:8],
                                amo_operand_b_mux[24 +:8],
                                amo_operand_b_mux[32 +:8],
                                amo_operand_b_mux[40 +:8],
                                amo_operand_b_mux[48 +:8],
                                amo_operand_b_mux[56 +:8]};
  end else begin : g_swap_in
    assign amo_operand_a_swp = amo_operand_a_mux;
    assign amo_operand_b_swp = amo_operand_b_mux;
  end
endgenerate

// operand word/byte select
always @* begin
  amo_operand_a = 64'h0;
  amo_operand_b = 64'h0;

  case (data_size)
    3'b001: begin
      amo_operand_a[56 +: 8]     = amo_operand_a_swp[address[2:0]*8 +: 8];
      amo_operand_b[56 +: 8]     = amo_operand_b_swp[address[2:0]*8 +: 8];
    end
    3'b010: begin
        amo_operand_a[48 +: 16]  = amo_operand_a_swp[address[2:1]*16 +: 16];
        amo_operand_b[48 +: 16]  = amo_operand_b_swp[address[2:1]*16 +: 16];
     end
    3'b011: begin
        amo_operand_a[32 +: 32]  = amo_operand_a_swp[address[2:2]*32 +: 32];
        amo_operand_b[32 +: 32]  = amo_operand_b_swp[address[2:2]*32 +: 32];
    end
    3'b100: begin
        amo_operand_a  = amo_operand_a_swp;
        amo_operand_b  = amo_operand_b_swp;
    end
    default: ;
  endcase // data_size
end


// main ALU
assign adder_sum     = adder_operand_a + adder_operand_b;

always @*
begin
    adder_operand_a = $signed(amo_operand_a);
    adder_operand_b = $signed(amo_operand_b);

    amo_64b_tmp     = amo_operand_a;

    case (amo_alu_op)
        4'd0: ;
        4'd1: amo_64b_tmp = adder_sum[63:0];
        4'd2: amo_64b_tmp = amo_operand_a & amo_operand_b;
        4'd3:  amo_64b_tmp = amo_operand_a | amo_operand_b;
        4'd4: amo_64b_tmp = amo_operand_a ^ amo_operand_b;
        4'd5: begin
            adder_operand_b = -$signed(amo_operand_b);
            amo_64b_tmp = adder_sum[64] ? amo_operand_b : amo_operand_a;
        end
        4'd6: begin
            adder_operand_a = $unsigned(amo_operand_a);
            adder_operand_b = -$unsigned(amo_operand_b);
            amo_64b_tmp = adder_sum[64] ? amo_operand_b : amo_operand_a;
        end
        4'd7: begin
            adder_operand_b = -$signed(amo_operand_b);
            amo_64b_tmp = adder_sum[64] ? amo_operand_a : amo_operand_b;
        end
        4'd8: begin
            adder_operand_a = $unsigned(amo_operand_a);
            adder_operand_b = -$unsigned(amo_operand_b);
            amo_64b_tmp = adder_sum[64] ? amo_operand_a : amo_operand_b;
        end
        default: ;
    endcase
end


// operand select and endianess swap
always @* begin
  // first read-modify-write 64bit word
  amo_64b_result = amo_operand_a_swp;
  case (data_size)
    3'b001: begin
      amo_64b_result[address[2:0]*8 +: 8]     = amo_64b_tmp[56 +: 8];
    end
    3'b010: begin
        amo_64b_result[address[2:1]*16 +: 16]  = amo_64b_tmp[48 +: 16];
     end
    3'b011: begin
        amo_64b_result[address[2:2]*32 +: 32]  = amo_64b_tmp[32 +: 32];
    end
    3'b100: begin
        amo_64b_result  = amo_64b_tmp;
    end
    default: ;
  endcase // data_size

  // merge back into memory line
  amo_result     = memory_operand;
  if (SWAP_ENDIANESS) begin
    amo_result[dword_offset*64 +: 64] = {amo_64b_result[ 0 +:8],
                                         amo_64b_result[ 8 +:8],
                                         amo_64b_result[16 +:8],
                                         amo_64b_result[24 +:8],
                                         amo_64b_result[32 +:8],
                                         amo_64b_result[40 +:8],
                                         amo_64b_result[48 +:8],
                                         amo_64b_result[56 +:8]};
  end else begin
    amo_result[dword_offset*64 +: 64] = amo_64b_result;
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

//==================================================================================================
//  Filename      : l2_broadcast_counter.v
//  Created On    : 2014-08-14
//  Revision      
//  Author        : Yaosheng Fu
//  Company       : Princeton University
//  Email         : yfu@princeton.edu
//
//  Description   : The broadcast counter in the L2 cache
//
//
//==================================================================================================

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
//  Filename      : l2.h.pyv
//  Created On    : 2014-02-20
//  Revision      :
//  Author        : Yaosheng Fu
//  Company       : Princeton University
//  Email         : yfu@princeton.edu
//
//  Description   : header file for the L2 cache
//
//
//==================================================================================================

// devices.xml



// Input buffer for pipeline1













// Input buffer for pipeline2














// Output buffer















// L2 cache configuration







//`define L2_SIZE                  65536
//`define L2_SIZE_WIDTH            16






//`define L2_LINE_SIZE             64
//`define L2_LINE_SIZE_WIDTH       6













//`define L2_WAYS                  4
//`define L2_WAYS_WIDTH            2
//`define L2_WAY_0                 2'b00
//`define L2_WAY_1                 2'b01
//`define L2_WAY_2                 2'b10
//`define L2_WAY_3                 2'b11





// Tag array







//`define L2_TAG_INDEX_WIDTH      8
//`define L2_TAG_WIDTH            26
//`define L2_TAG_WAY_WIDTH        26
//`define L2_TAG_ARRAY_WIDTH      104


// Tag Address decomposition 




//`define L2_TAG_INDEX            13:6
//`define L2_TAG                  39:14


//Data array












//`define L2_DATA_INDEX_WIDTH         12 








// Data Address decomposition 







//Dir array



//`define L2_DIR_INDEX_WIDTH      10 





//State array

















//Whether the cache line is in Icaches or Dcaches
















//Round Robin selection














// State decomposition 












//MSHR array



































//`define L2_MSHR_CMP_ADDR        13:6
//`define L2_MSHR_ADDR            39:0
//`define L2_MSHR_WAY             41:40
//`define L2_MSHR_MSHRID          49:42
//`define L2_MSHR_CACHE_TYPE      50
//`define L2_MSHR_DATA_SIZE       53:51
//`define L2_MSHR_MSG_TYPE        61:54
//`define L2_MSHR_L2_MISS         62
//`define L2_MSHR_SRC_CHIPID      76:63
//`define L2_MSHR_SRC_X           84:77
//`define L2_MSHR_SRC_Y           92:85
//`define L2_MSHR_SRC_FBITS       96:93
//`define L2_MSHR_SDID            106:97
//`define L2_MSHR_LSID            112:107      
//`define L2_MSHR_MISS_LSID       118:113
//`define L2_MSHR_SMC_MISS        119
//`define L2_MSHR_RECYCLED        120
//`define L2_MSHR_INV_FWD_PENDING 121


//SMC array


































//Message destination_type







//L2 public sharer beyond the maximum clump size



//L2 registers



//Special addresses





















//L2 core ID





//L2 dir array reuse







//Control Signal 








//AMO ALU OP macros












//Control Signal in Stage 1























































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





































































































































































































































































































































































































































































































































































































module l2_broadcast_counter(

    input wire clk,
    input wire rst_n,
    input wire [14-1:0] chipid_max,
    input wire [8-1:0] x_max,
    input wire [8-1:0] y_max,

    input wire [2-1:0] counter_op,
    input wire counter_op_val,
    
    output reg zero,
    output reg max,
    output reg [14-1:0] chipid_out,
    output reg [8-1:0] x_out,
    output reg [8-1:0] y_out

);


reg [14-1:0] chipid_f;
reg [14-1:0] chipid_next;
reg [8-1:0] x_f;
reg [8-1:0] x_next;
reg [8-1:0] y_f;
reg [8-1:0] y_next;



always @ *
begin
    if (!rst_n)
    begin
        chipid_next = 0;
        x_next = 0;
        y_next = 0;
    end
    else if (counter_op_val)
    begin
        if (counter_op == 2'd1)
        begin
            chipid_next = chipid_max;
            x_next = x_max;
            y_next = y_max;
        end
        else if (counter_op == 2'd0)
        begin
            chipid_next = 0;
            x_next = 0;
            y_next = 0;
        end
        else if (counter_op == 2'd2)
        begin
            if (x_f == x_max)
            begin
                if (y_f == y_max)
                begin
                    if(chipid_f == chipid_max)
                    begin
                        chipid_next = 0;
                        x_next = 0;
                        y_next = 0;
                    end
                    else        
                    begin
                        chipid_next = chipid_f + 1;
                        x_next = 0;
                        y_next = 0;
                    end
                end
                else
                begin
                    chipid_next = chipid_f;
                    x_next = 0;
                    y_next = y_f + 1;
                end
            end
            else
            begin
                chipid_next = chipid_f;
                x_next = x_f + 1;
                y_next = y_f;
            end
        end
        else
        begin
            chipid_next = chipid_f;
            x_next = x_f;
            y_next = y_f;
        end
    end
    else
    begin
        chipid_next = chipid_f;
        x_next = x_f;
        y_next = y_f;
    end
end


always @ (posedge clk)
begin
    chipid_f <= chipid_next;
    x_f <= x_next;
    y_f <= y_next;
end


always @ *
begin
    zero = (x_f == 0) && (y_f == 0) && (chipid_f == 0);
    max = (x_f == x_max) && (y_f == y_max) && (chipid_f == chipid_max);
end



always @ *
begin
    chipid_out = chipid_f;
    x_out = x_f;
    y_out = y_f;
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

//==================================================================================================
//  Filename      : l2_broadcast_counter_wrap.v
//  Created On    : 2014-08-14
//  Revision      
//  Author        : Yaosheng Fu
//  Company       : Princeton University
//  Email         : yfu@princeton.edu
//
//  Description   : The wrap module for the broadcast counter in the L2 cache
//
//
//==================================================================================================

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
//  Filename      : l2.h.pyv
//  Created On    : 2014-02-20
//  Revision      :
//  Author        : Yaosheng Fu
//  Company       : Princeton University
//  Email         : yfu@princeton.edu
//
//  Description   : header file for the L2 cache
//
//
//==================================================================================================

// devices.xml



// Input buffer for pipeline1













// Input buffer for pipeline2














// Output buffer















// L2 cache configuration







//`define L2_SIZE                  65536
//`define L2_SIZE_WIDTH            16






//`define L2_LINE_SIZE             64
//`define L2_LINE_SIZE_WIDTH       6













//`define L2_WAYS                  4
//`define L2_WAYS_WIDTH            2
//`define L2_WAY_0                 2'b00
//`define L2_WAY_1                 2'b01
//`define L2_WAY_2                 2'b10
//`define L2_WAY_3                 2'b11





// Tag array







//`define L2_TAG_INDEX_WIDTH      8
//`define L2_TAG_WIDTH            26
//`define L2_TAG_WAY_WIDTH        26
//`define L2_TAG_ARRAY_WIDTH      104


// Tag Address decomposition 




//`define L2_TAG_INDEX            13:6
//`define L2_TAG                  39:14


//Data array












//`define L2_DATA_INDEX_WIDTH         12 








// Data Address decomposition 







//Dir array



//`define L2_DIR_INDEX_WIDTH      10 





//State array

















//Whether the cache line is in Icaches or Dcaches
















//Round Robin selection














// State decomposition 












//MSHR array



































//`define L2_MSHR_CMP_ADDR        13:6
//`define L2_MSHR_ADDR            39:0
//`define L2_MSHR_WAY             41:40
//`define L2_MSHR_MSHRID          49:42
//`define L2_MSHR_CACHE_TYPE      50
//`define L2_MSHR_DATA_SIZE       53:51
//`define L2_MSHR_MSG_TYPE        61:54
//`define L2_MSHR_L2_MISS         62
//`define L2_MSHR_SRC_CHIPID      76:63
//`define L2_MSHR_SRC_X           84:77
//`define L2_MSHR_SRC_Y           92:85
//`define L2_MSHR_SRC_FBITS       96:93
//`define L2_MSHR_SDID            106:97
//`define L2_MSHR_LSID            112:107      
//`define L2_MSHR_MISS_LSID       118:113
//`define L2_MSHR_SMC_MISS        119
//`define L2_MSHR_RECYCLED        120
//`define L2_MSHR_INV_FWD_PENDING 121


//SMC array


































//Message destination_type







//L2 public sharer beyond the maximum clump size



//L2 registers



//Special addresses





















//L2 core ID





//L2 dir array reuse







//Control Signal 








//AMO ALU OP macros












//Control Signal in Stage 1























































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





































































































































































































































































































































































































































































































































































































module l2_broadcast_counter_wrap(

    input wire clk,
    input wire rst_n,
    input wire [14-1:0] chipid_max,
    input wire [8-1:0] x_max,
    input wire [8-1:0] y_max,
    input wire pipe_sel,

    input wire [2-1:0] counter_op1,
    input wire counter_op_val1,
   

    input wire [2-1:0] counter_op2,
    input wire counter_op_val2,
 
    output wire zero1,
    output wire max1,
    output reg  avail1,
    output wire [14-1:0] chipid_out1,
    output wire [8-1:0] x_out1,
    output wire [8-1:0] y_out1,

    output wire zero2,
    output wire max2,
    output reg  avail2,
    output wire [14-1:0] chipid_out2,
    output wire [8-1:0] x_out2,
    output wire [8-1:0] y_out2
);


reg state_f;
reg state_next;

always @ *
begin
    if (!rst_n)
    begin
        state_next = 1'b0;
    end
    else
    begin
        if (counter_op_val1 && (counter_op1 == 2'd2))
        begin
            state_next = 1'b1;
        end
        else if (counter_op_val2 && (counter_op2 == 2'd0))
        begin
            state_next = 1'b0;
        end
        else
        begin
            state_next = state_f;
        end
    end
end


always @ (posedge clk)
begin
    state_f <= state_next;
end


always @ *
begin
    avail1 = (state_f == 1'b0);
    avail2 = (state_f == 1'b0);
end


l2_broadcast_counter l2_broadcast_counter1(
    .clk                (clk),
    .rst_n              (rst_n), 
    .chipid_max         (chipid_max),
    .x_max              (x_max),
    .y_max              (y_max),
    .counter_op         (counter_op1),
    .counter_op_val     (counter_op_val1),
    .zero               (zero1),
    .max                (max1),
    .chipid_out         (chipid_out1),
    .x_out              (x_out1),
    .y_out              (y_out1)

);

l2_broadcast_counter l2_broadcast_counter2(
    .clk                (clk),
    .rst_n              (rst_n), 
    .chipid_max         (chipid_max),
    .x_max              (x_max),
    .y_max              (y_max),
    .counter_op         (counter_op2),
    .counter_op_val     (counter_op_val2),
    .zero               (zero2),
    .max                (max2),
    .chipid_out         (chipid_out2),
    .x_out              (x_out2),
    .y_out              (y_out2)

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
//  Filename      : l2_config_regs.v
//  Created On    : 2015-01-02
//  Revision      :
//  Author        : Yaosheng Fu
//  Company       : Princeton University
//  Email         : yfu@princeton.edu
//
//  Description   : Config registers for the L2 cache
//
//
//==================================================================================================

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
//  Filename      : l2.h.pyv
//  Created On    : 2014-02-20
//  Revision      :
//  Author        : Yaosheng Fu
//  Company       : Princeton University
//  Email         : yfu@princeton.edu
//
//  Description   : header file for the L2 cache
//
//
//==================================================================================================

// devices.xml



// Input buffer for pipeline1













// Input buffer for pipeline2














// Output buffer















// L2 cache configuration







//`define L2_SIZE                  65536
//`define L2_SIZE_WIDTH            16






//`define L2_LINE_SIZE             64
//`define L2_LINE_SIZE_WIDTH       6













//`define L2_WAYS                  4
//`define L2_WAYS_WIDTH            2
//`define L2_WAY_0                 2'b00
//`define L2_WAY_1                 2'b01
//`define L2_WAY_2                 2'b10
//`define L2_WAY_3                 2'b11





// Tag array







//`define L2_TAG_INDEX_WIDTH      8
//`define L2_TAG_WIDTH            26
//`define L2_TAG_WAY_WIDTH        26
//`define L2_TAG_ARRAY_WIDTH      104


// Tag Address decomposition 




//`define L2_TAG_INDEX            13:6
//`define L2_TAG                  39:14


//Data array












//`define L2_DATA_INDEX_WIDTH         12 








// Data Address decomposition 







//Dir array



//`define L2_DIR_INDEX_WIDTH      10 





//State array

















//Whether the cache line is in Icaches or Dcaches
















//Round Robin selection














// State decomposition 












//MSHR array



































//`define L2_MSHR_CMP_ADDR        13:6
//`define L2_MSHR_ADDR            39:0
//`define L2_MSHR_WAY             41:40
//`define L2_MSHR_MSHRID          49:42
//`define L2_MSHR_CACHE_TYPE      50
//`define L2_MSHR_DATA_SIZE       53:51
//`define L2_MSHR_MSG_TYPE        61:54
//`define L2_MSHR_L2_MISS         62
//`define L2_MSHR_SRC_CHIPID      76:63
//`define L2_MSHR_SRC_X           84:77
//`define L2_MSHR_SRC_Y           92:85
//`define L2_MSHR_SRC_FBITS       96:93
//`define L2_MSHR_SDID            106:97
//`define L2_MSHR_LSID            112:107      
//`define L2_MSHR_MISS_LSID       118:113
//`define L2_MSHR_SMC_MISS        119
//`define L2_MSHR_RECYCLED        120
//`define L2_MSHR_INV_FWD_PENDING 121


//SMC array


































//Message destination_type







//L2 public sharer beyond the maximum clump size



//L2 registers



//Special addresses





















//L2 core ID





//L2 dir array reuse







//Control Signal 








//AMO ALU OP macros












//Control Signal in Stage 1























































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





































































































































































































































































































































































































































































































































































































module l2_config_regs(

    input wire clk,
    input wire rst_n,

    input wire [14-1:0] chipid,
    input wire [8-1:0] coreid_x,
    input wire [8-1:0] coreid_y,
    input wire l2_access_valid,
    input wire l2_miss_valid,
    input wire data_ecc_corr_error,
    input wire data_ecc_uncorr_error,
    input wire [8+2+2-1:0] data_ecc_addr,
    input wire [40-1:0] error_addr,
    input wire reg_rd_en,
    input wire reg_wr_en,
    input wire [8-1:0] reg_rd_addr_type,
    input wire [8-1:0] reg_wr_addr_type,
    input wire [64-1:0] reg_data_in,

    output reg [64-1:0] reg_data_out,
    output reg [34-1:0] my_nodeid,
    output reg [14+8+8-1:0] core_max,
    output reg csm_en,
    output reg [22-1:0] smt_base_addr

);


reg [64-1:0] ctrl_reg_f;
reg [64-1:0] coreid_reg_f;
reg [64-1:0] l2_access_counter_reg_f;
reg [64-1:0] l2_miss_counter_reg_f;
reg [64-1:0] error_status_reg_f;

reg ctrl_reg_wr_en;
reg coreid_reg_wr_en;
reg l2_access_counter_reg_wr_en;
reg l2_miss_counter_reg_wr_en;
reg error_status_reg_wr_en;
reg error_status_en;
reg l2_access_counter_inc_en;
reg l2_miss_counter_inc_en;

always @ *
begin
    ctrl_reg_wr_en = reg_wr_en && (reg_wr_addr_type == 8'ha9); 
    coreid_reg_wr_en = reg_wr_en && (reg_wr_addr_type == 8'ha7); 
    l2_access_counter_reg_wr_en = reg_wr_en && (reg_wr_addr_type == 8'haa); 
    l2_miss_counter_reg_wr_en = reg_wr_en && (reg_wr_addr_type == 8'hab); 
    error_status_reg_wr_en = reg_wr_en && (reg_wr_addr_type == 8'ha8); 
    
end

always @ (posedge clk)
begin
    if (!rst_n)
    begin
        ctrl_reg_f <= 0;
    end
    else if (ctrl_reg_wr_en)
    begin
        ctrl_reg_f <= reg_data_in; 
    end
end

always @ (posedge clk)
begin
    if (!rst_n)
    begin
        coreid_reg_f <= {{(64-34){1'b0}},chipid, coreid_x, coreid_y, 4'd0};
    end
    else if (coreid_reg_wr_en)
    begin
        coreid_reg_f <= reg_data_in; 
    end
end

reg l2_access_counter_inc_en_f;
always @ (posedge clk)
begin
    if (!rst_n)
    begin
        l2_access_counter_reg_f <= 0;
    end
    else if (l2_access_counter_reg_wr_en)
    begin
        l2_access_counter_reg_f <= reg_data_in; 
    end
    // else if (l2_access_counter_inc_en && l2_access_valid)
    // begin
    //     l2_access_counter_reg_f <= l2_access_counter_reg_f + 1;
    // end 
    // trin: pipeline addition for timing
    l2_access_counter_inc_en_f <= l2_access_counter_inc_en && l2_access_valid;
    if (l2_access_counter_inc_en_f) begin
        l2_access_counter_reg_f <= l2_access_counter_reg_f + 1;
    end
end


reg l2_miss_counter_inc_en_f;
always @ (posedge clk)
begin
    if (!rst_n)
    begin
        l2_miss_counter_reg_f <= 0;
    end
    else if (l2_miss_counter_reg_wr_en)
    begin
        l2_miss_counter_reg_f <= reg_data_in; 
    end
    // else if (l2_miss_counter_inc_en && l2_miss_valid)
    // begin
    //     l2_miss_counter_reg_f <= l2_miss_counter_reg_f + 1;
    // end 
    // trin: pipeline addition for timing
    l2_miss_counter_inc_en_f <= l2_miss_counter_inc_en && l2_miss_valid;
    if (l2_miss_counter_inc_en_f) begin
        l2_miss_counter_reg_f <= l2_miss_counter_reg_f + 1;
    end
end

always @ (posedge clk)
begin
    if (!rst_n)
    begin
        error_status_reg_f <= 0;
    end
    else if (error_status_reg_wr_en)
    begin
        error_status_reg_f <= reg_data_in; 
    end
    else if (error_status_en && data_ecc_corr_error)
    begin
        if (|error_status_reg_f[1:0])
        begin
            error_status_reg_f <= {error_status_reg_f[63:55], error_addr, data_ecc_addr, 1'b1, 1'b0, 1'b1};
        end
        else
        begin
            error_status_reg_f <= {error_status_reg_f[63:55], error_addr, data_ecc_addr, 1'b0, 1'b0, 1'b1};
        end
    end
    else if (error_status_en && data_ecc_uncorr_error)
    begin
        if (|error_status_reg_f[1:0])
        begin
            error_status_reg_f <= {error_status_reg_f[63:55], error_addr, data_ecc_addr, 1'b1, 1'b1, 1'b0};
        end
        else
        begin
            error_status_reg_f <= {error_status_reg_f[63:55], error_addr, data_ecc_addr, 1'b0, 1'b1, 1'b0};
        end
    end

end


always @ * 
begin
    if (reg_rd_en)
    begin
        if (reg_rd_addr_type == 8'ha9)
        begin
            reg_data_out = ctrl_reg_f;
        end
        else if (reg_rd_addr_type == 8'ha7)
        begin
            reg_data_out = coreid_reg_f;
        end
        else if (reg_rd_addr_type == 8'haa)
        begin
            reg_data_out = l2_access_counter_reg_f;
        end
        else if (reg_rd_addr_type == 8'hab)
        begin
            reg_data_out = l2_miss_counter_reg_f;
        end
        else if (reg_rd_addr_type == 8'ha8)
        begin
            reg_data_out = error_status_reg_f;
        end
        else
        begin
            reg_data_out = 0;
        end
    end
    else
    begin
        reg_data_out = 0;
    end
end

always @ * 
begin
    csm_en = ctrl_reg_f[0];
    error_status_en = ctrl_reg_f[1];
    l2_access_counter_inc_en = ctrl_reg_f[2];
    l2_miss_counter_inc_en = ctrl_reg_f[3];
    smt_base_addr = ctrl_reg_f[22+32-1 : 32];
end

always @ * 
begin
    my_nodeid = coreid_reg_f[34-1 : 0]; 
    core_max = coreid_reg_f[14+8+8+34-1 : 34];
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

//==================================================================================================
//  Filename      : l2_data.v
//  Created On    : 2014-02-24
//  Revision      :
//  Author        : Yaosheng Fu
//  Company       : Princeton University
//  Email         : yfu@princeton.edu
//
//  Description   : The data array in the L2 cache
//
//
//==================================================================================================

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
//  Filename      : l2.h.pyv
//  Created On    : 2014-02-20
//  Revision      :
//  Author        : Yaosheng Fu
//  Company       : Princeton University
//  Email         : yfu@princeton.edu
//
//  Description   : header file for the L2 cache
//
//
//==================================================================================================

// devices.xml



// Input buffer for pipeline1













// Input buffer for pipeline2














// Output buffer















// L2 cache configuration







//`define L2_SIZE                  65536
//`define L2_SIZE_WIDTH            16






//`define L2_LINE_SIZE             64
//`define L2_LINE_SIZE_WIDTH       6













//`define L2_WAYS                  4
//`define L2_WAYS_WIDTH            2
//`define L2_WAY_0                 2'b00
//`define L2_WAY_1                 2'b01
//`define L2_WAY_2                 2'b10
//`define L2_WAY_3                 2'b11





// Tag array







//`define L2_TAG_INDEX_WIDTH      8
//`define L2_TAG_WIDTH            26
//`define L2_TAG_WAY_WIDTH        26
//`define L2_TAG_ARRAY_WIDTH      104


// Tag Address decomposition 




//`define L2_TAG_INDEX            13:6
//`define L2_TAG                  39:14


//Data array












//`define L2_DATA_INDEX_WIDTH         12 








// Data Address decomposition 







//Dir array



//`define L2_DIR_INDEX_WIDTH      10 





//State array

















//Whether the cache line is in Icaches or Dcaches
















//Round Robin selection














// State decomposition 












//MSHR array



































//`define L2_MSHR_CMP_ADDR        13:6
//`define L2_MSHR_ADDR            39:0
//`define L2_MSHR_WAY             41:40
//`define L2_MSHR_MSHRID          49:42
//`define L2_MSHR_CACHE_TYPE      50
//`define L2_MSHR_DATA_SIZE       53:51
//`define L2_MSHR_MSG_TYPE        61:54
//`define L2_MSHR_L2_MISS         62
//`define L2_MSHR_SRC_CHIPID      76:63
//`define L2_MSHR_SRC_X           84:77
//`define L2_MSHR_SRC_Y           92:85
//`define L2_MSHR_SRC_FBITS       96:93
//`define L2_MSHR_SDID            106:97
//`define L2_MSHR_LSID            112:107      
//`define L2_MSHR_MISS_LSID       118:113
//`define L2_MSHR_SMC_MISS        119
//`define L2_MSHR_RECYCLED        120
//`define L2_MSHR_INV_FWD_PENDING 121


//SMC array


































//Message destination_type







//L2 public sharer beyond the maximum clump size



//L2 registers



//Special addresses





















//L2 core ID





//L2 dir array reuse







//Control Signal 








//AMO ALU OP macros












//Control Signal in Stage 1























































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





































































































































































































































































































































































































































































































































































































module l2_data(

    input wire clk,
    input wire rst_n,
    input wire clk_en,
    input wire rdw_en,
    input wire pdout_en,
    input wire deepsleep,

    input wire [8+2+2-1:0] addr,
    input wire [144-1:0] data_in,
    input wire [144-1:0] data_mask_in,

    output wire [144-1:0] data_out,
    output wire [144-1:0] pdata_out,

    // sram interface
    output wire [4-1:0] srams_rtap_data,
    input wire  [4-1:0] rtap_srams_bist_command,
    input wire  [4-1:0] rtap_srams_bist_data
);

/*
localparam reset = 2'd0;
localparam init  = 2'd1;
localparam done  = 2'd2;

reg [1:0] init_state_f;
reg [1:0] init_state_next;
reg [`L2_DATA_INDEX_WIDTH-1:0] init_counter_f;
reg [`L2_DATA_INDEX_WIDTH-1:0] init_counter_next;

reg [`L2_DATA_INDEX_WIDTH-1:0] addr_real;
reg rdw_en_real;
reg clk_en_real;
reg [`L2_DATA_ARRAY_WIDTH-1:0] data_in_real;
reg [`L2_DATA_ARRAY_WIDTH-1:0] data_mask_in_real;

always @ *
begin
    if (!rst_n)
    begin
        init_state_next = reset;
    end
    else
    begin
        if (init_state_f == reset)
        begin
            init_state_next = init;
        end
        else if ((init_state_f == init) && (init_counter_f == {`L2_DATA_INDEX_WIDTH{1'b1}}))
        begin
            init_state_next = done;
        end
        else
        begin
            init_state_next = init_state_f;
        end
    end
end

always @ (posedge clk)
begin
    init_state_f <= init_state_next;
end

always @ *
begin
    if ((init_state_f == reset) || (init_state_f == done))
    begin
        init_counter_next = {`L2_DATA_INDEX_WIDTH{1'b0}};
    end
    else
    begin
        init_counter_next = init_counter_f + 1;
    end
end


always @ (posedge clk)
begin
    init_counter_f <= init_counter_next;
end


always @ *
begin
    init_done = (init_state_f == done);
end


always @ *
begin
    init_done = (init_state_f == done);
end


always @ *
begin
    if (init_state_f == init)
    begin
        clk_en_real = 1'b1;
        rdw_en_real = 1'b0;
        addr_real = init_counter_f;
        data_in_real = {`L2_DATA_ARRAY_WIDTH{1'b0}};
        data_mask_in_real = {`L2_DATA_ARRAY_WIDTH{1'b1}};
    end
    else
    begin
        clk_en_real = clk_en;
        rdw_en_real = rdw_en;
        addr_real = addr;
        data_in_real = data_in;
        data_mask_in_real = data_mask_in;
    end
end

*/

// sram_1rw_4096x144 l2_data_array(
sram_l2_data l2_data_array(
    .MEMCLK     (clk),
    .RESET_N(rst_n),
    .CE         (clk_en),

    .A          (addr),
    .DIN        (data_in),
    .RDWEN      (rdw_en),
    .BW         (data_mask_in),
    .DOUT       (data_out),
    .BIST_COMMAND(rtap_srams_bist_command),
    .BIST_DIN(rtap_srams_bist_data),
    .BIST_DOUT(srams_rtap_data),
    .SRAMID(8'd13)
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
//  Filename      : l2_data_wrap.v
//  Created On    : 2014-02-24
//  Revision      :
//  Author        : Yaosheng Fu
//  Company       : Princeton University
//  Email         : yfu@princeton.edu
//
//  Description   : The wrap module for data array in the L2 cache
//
//
//==================================================================================================


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
//  Filename      : l2.h.pyv
//  Created On    : 2014-02-20
//  Revision      :
//  Author        : Yaosheng Fu
//  Company       : Princeton University
//  Email         : yfu@princeton.edu
//
//  Description   : header file for the L2 cache
//
//
//==================================================================================================

// devices.xml



// Input buffer for pipeline1













// Input buffer for pipeline2














// Output buffer















// L2 cache configuration







//`define L2_SIZE                  65536
//`define L2_SIZE_WIDTH            16






//`define L2_LINE_SIZE             64
//`define L2_LINE_SIZE_WIDTH       6













//`define L2_WAYS                  4
//`define L2_WAYS_WIDTH            2
//`define L2_WAY_0                 2'b00
//`define L2_WAY_1                 2'b01
//`define L2_WAY_2                 2'b10
//`define L2_WAY_3                 2'b11





// Tag array







//`define L2_TAG_INDEX_WIDTH      8
//`define L2_TAG_WIDTH            26
//`define L2_TAG_WAY_WIDTH        26
//`define L2_TAG_ARRAY_WIDTH      104


// Tag Address decomposition 




//`define L2_TAG_INDEX            13:6
//`define L2_TAG                  39:14


//Data array












//`define L2_DATA_INDEX_WIDTH         12 








// Data Address decomposition 







//Dir array



//`define L2_DIR_INDEX_WIDTH      10 





//State array

















//Whether the cache line is in Icaches or Dcaches
















//Round Robin selection














// State decomposition 












//MSHR array



































//`define L2_MSHR_CMP_ADDR        13:6
//`define L2_MSHR_ADDR            39:0
//`define L2_MSHR_WAY             41:40
//`define L2_MSHR_MSHRID          49:42
//`define L2_MSHR_CACHE_TYPE      50
//`define L2_MSHR_DATA_SIZE       53:51
//`define L2_MSHR_MSG_TYPE        61:54
//`define L2_MSHR_L2_MISS         62
//`define L2_MSHR_SRC_CHIPID      76:63
//`define L2_MSHR_SRC_X           84:77
//`define L2_MSHR_SRC_Y           92:85
//`define L2_MSHR_SRC_FBITS       96:93
//`define L2_MSHR_SDID            106:97
//`define L2_MSHR_LSID            112:107      
//`define L2_MSHR_MISS_LSID       118:113
//`define L2_MSHR_SMC_MISS        119
//`define L2_MSHR_RECYCLED        120
//`define L2_MSHR_INV_FWD_PENDING 121


//SMC array


































//Message destination_type







//L2 public sharer beyond the maximum clump size



//L2 registers



//Special addresses





















//L2 core ID





//L2 dir array reuse







//Control Signal 








//AMO ALU OP macros












//Control Signal in Stage 1























































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





































































































































































































































































































































































































































































































































































































module l2_data_wrap(

    input wire clk,
    input wire rst_n,
    input wire clk_en1,
    input wire clk_en2,
    input wire rdw_en1,
    input wire rdw_en2,
    input wire pdout_en,
    input wire deepsleep,
    input wire pipe_sel,

    input wire [8+2+2-1:0] addr1,
    input wire [144-1:0] data_in1,
    input wire [144-1:0] data_mask_in1,

    input wire [8+2+2-1:0] addr2,
    input wire [144-1:0] data_in2,
    input wire [144-1:0] data_mask_in2,

    output wire [144-1:0] data_out,
    output wire [144-1:0] pdata_out,

    // sram interface
    output wire [4-1:0] srams_rtap_data,
    input wire  [4-1:0] rtap_srams_bist_command,
    input wire  [4-1:0] rtap_srams_bist_data

);

reg clk_en;
reg rdw_en;
reg [8+2+2-1:0] addr;
reg [144-1:0] data_in;
reg [144-1:0] data_mask_in;

always @ *
begin
    if (pipe_sel)
    begin
        clk_en = clk_en2;
        rdw_en = rdw_en2;
        addr = addr2;
        data_in = data_in2;
        data_mask_in = data_mask_in2;
    end
    else
    begin
        clk_en = clk_en1;
        rdw_en = rdw_en1;
        addr = addr1;
        data_in = data_in1;
        data_mask_in = data_mask_in1;
    end
end

l2_data l2_data(

    .clk            (clk),
    .rst_n          (rst_n),
    .clk_en         (clk_en),
    .rdw_en         (rdw_en),
    .pdout_en       (pdout_en),
    .deepsleep      (deepsleep),
    .addr           (addr),
    .data_in        (data_in),
    .data_mask_in   (data_mask_in),
    .data_out       (data_out),
    .pdata_out      (pdata_out),

    // sram interfaces
    .srams_rtap_data (srams_rtap_data),
    .rtap_srams_bist_command (rtap_srams_bist_command),
    .rtap_srams_bist_data (rtap_srams_bist_data)
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
//  Filename      : l2_decoder.v
//  Created On    : 2014-02-25
//  Revision      :
//  Author        : Yaosheng Fu
//  Company       : Princeton University
//  Email         : yfu@princeton.edu
//
//  Description   : The message decoder in the L2 cache
//
//
//====================================================================================================

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
//  Filename      : l2.h.pyv
//  Created On    : 2014-02-20
//  Revision      :
//  Author        : Yaosheng Fu
//  Company       : Princeton University
//  Email         : yfu@princeton.edu
//
//  Description   : header file for the L2 cache
//
//
//==================================================================================================

// devices.xml



// Input buffer for pipeline1













// Input buffer for pipeline2














// Output buffer















// L2 cache configuration







//`define L2_SIZE                  65536
//`define L2_SIZE_WIDTH            16






//`define L2_LINE_SIZE             64
//`define L2_LINE_SIZE_WIDTH       6













//`define L2_WAYS                  4
//`define L2_WAYS_WIDTH            2
//`define L2_WAY_0                 2'b00
//`define L2_WAY_1                 2'b01
//`define L2_WAY_2                 2'b10
//`define L2_WAY_3                 2'b11





// Tag array







//`define L2_TAG_INDEX_WIDTH      8
//`define L2_TAG_WIDTH            26
//`define L2_TAG_WAY_WIDTH        26
//`define L2_TAG_ARRAY_WIDTH      104


// Tag Address decomposition 




//`define L2_TAG_INDEX            13:6
//`define L2_TAG                  39:14


//Data array












//`define L2_DATA_INDEX_WIDTH         12 








// Data Address decomposition 







//Dir array



//`define L2_DIR_INDEX_WIDTH      10 





//State array

















//Whether the cache line is in Icaches or Dcaches
















//Round Robin selection














// State decomposition 












//MSHR array



































//`define L2_MSHR_CMP_ADDR        13:6
//`define L2_MSHR_ADDR            39:0
//`define L2_MSHR_WAY             41:40
//`define L2_MSHR_MSHRID          49:42
//`define L2_MSHR_CACHE_TYPE      50
//`define L2_MSHR_DATA_SIZE       53:51
//`define L2_MSHR_MSG_TYPE        61:54
//`define L2_MSHR_L2_MISS         62
//`define L2_MSHR_SRC_CHIPID      76:63
//`define L2_MSHR_SRC_X           84:77
//`define L2_MSHR_SRC_Y           92:85
//`define L2_MSHR_SRC_FBITS       96:93
//`define L2_MSHR_SDID            106:97
//`define L2_MSHR_LSID            112:107      
//`define L2_MSHR_MISS_LSID       118:113
//`define L2_MSHR_SMC_MISS        119
//`define L2_MSHR_RECYCLED        120
//`define L2_MSHR_INV_FWD_PENDING 121


//SMC array


































//Message destination_type







//L2 public sharer beyond the maximum clump size



//L2 registers



//Special addresses





















//L2 core ID





//L2 dir array reuse







//Control Signal 








//AMO ALU OP macros












//Control Signal in Stage 1























































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





































































































































































































































































































































































































































































































































































































module l2_decoder(

    input wire [192-1:0] msg_header,

    output reg [8-1:0] msg_type,
    output reg [8-1:0] msg_length,
    output reg [8-1:0] msg_mshrid,
    output reg [3-1:0] msg_data_size,
    output reg [1-1:0] msg_cache_type,
    output reg [4-1:0] msg_subline_vector,
    output reg [2-1:0] msg_mesi,
    output reg [1-1:0] msg_l2_miss,
    output reg [2-1:0] msg_subline_id,
    output reg [1-1:0] msg_last_subline,
    output reg [40-1:0] msg_addr,
    output reg [14-1:0] msg_src_chipid,
    output reg [8-1:0] msg_src_x,
    output reg [8-1:0] msg_src_y,
    output reg [4-1:0] msg_src_fbits,
    output reg [10-1:0] msg_sdid,
    output reg [6-1:0] msg_lsid
);

always @ *
begin
    msg_type = msg_header[21:14];
    msg_length = msg_header[29:22];
    msg_mshrid = msg_header[13:6];
    msg_data_size = msg_header[74:72];
    msg_cache_type = msg_header[75];
    msg_subline_vector = msg_header[79:76];
    msg_mesi = msg_header[5:4];
    msg_l2_miss = msg_header[3];
    msg_subline_id = msg_header[2:1];
    msg_last_subline = msg_header[0];
    msg_addr = msg_header[119:80];
    msg_src_chipid = msg_header[191:178];
    msg_src_x = msg_header[177:170];
    msg_src_y = msg_header[169:162];
    msg_src_fbits = msg_header[161:158];
    msg_sdid = msg_header[157:148];
    msg_lsid = msg_header[147:142];
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

//==================================================================================================
//  Filename      : l2_dir.v
//  Created On    : 2014-02-24
//  Revision      :
//  Author        : Yaosheng Fu
//  Company       : Princeton University
//  Email         : yfu@princeton.edu
//
//  Description   : The directory array in the L2 cache
//
//
//==================================================================================================

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
//  Filename      : l2.h.pyv
//  Created On    : 2014-02-20
//  Revision      :
//  Author        : Yaosheng Fu
//  Company       : Princeton University
//  Email         : yfu@princeton.edu
//
//  Description   : header file for the L2 cache
//
//
//==================================================================================================

// devices.xml



// Input buffer for pipeline1













// Input buffer for pipeline2














// Output buffer















// L2 cache configuration







//`define L2_SIZE                  65536
//`define L2_SIZE_WIDTH            16






//`define L2_LINE_SIZE             64
//`define L2_LINE_SIZE_WIDTH       6













//`define L2_WAYS                  4
//`define L2_WAYS_WIDTH            2
//`define L2_WAY_0                 2'b00
//`define L2_WAY_1                 2'b01
//`define L2_WAY_2                 2'b10
//`define L2_WAY_3                 2'b11





// Tag array







//`define L2_TAG_INDEX_WIDTH      8
//`define L2_TAG_WIDTH            26
//`define L2_TAG_WAY_WIDTH        26
//`define L2_TAG_ARRAY_WIDTH      104


// Tag Address decomposition 




//`define L2_TAG_INDEX            13:6
//`define L2_TAG                  39:14


//Data array












//`define L2_DATA_INDEX_WIDTH         12 








// Data Address decomposition 







//Dir array



//`define L2_DIR_INDEX_WIDTH      10 





//State array

















//Whether the cache line is in Icaches or Dcaches
















//Round Robin selection














// State decomposition 












//MSHR array



































//`define L2_MSHR_CMP_ADDR        13:6
//`define L2_MSHR_ADDR            39:0
//`define L2_MSHR_WAY             41:40
//`define L2_MSHR_MSHRID          49:42
//`define L2_MSHR_CACHE_TYPE      50
//`define L2_MSHR_DATA_SIZE       53:51
//`define L2_MSHR_MSG_TYPE        61:54
//`define L2_MSHR_L2_MISS         62
//`define L2_MSHR_SRC_CHIPID      76:63
//`define L2_MSHR_SRC_X           84:77
//`define L2_MSHR_SRC_Y           92:85
//`define L2_MSHR_SRC_FBITS       96:93
//`define L2_MSHR_SDID            106:97
//`define L2_MSHR_LSID            112:107      
//`define L2_MSHR_MISS_LSID       118:113
//`define L2_MSHR_SMC_MISS        119
//`define L2_MSHR_RECYCLED        120
//`define L2_MSHR_INV_FWD_PENDING 121


//SMC array


































//Message destination_type







//L2 public sharer beyond the maximum clump size



//L2 registers



//Special addresses





















//L2 core ID





//L2 dir array reuse







//Control Signal 








//AMO ALU OP macros












//Control Signal in Stage 1























































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





































































































































































































































































































































































































































































































































































































module l2_dir(

    input wire clk,
    input wire rst_n,
    input wire clk_en,
    input wire rdw_en,
    input wire pdout_en,
    input wire deepsleep,

    input wire [8+2-1:0] addr,
    input wire [64-1:0] data_in,
    input wire [64-1:0] data_mask_in,

    output wire [64-1:0] data_out,
    output wire [64-1:0] pdata_out,

    // sram interface
    output wire [4-1:0] srams_rtap_data,
    input wire  [4-1:0] rtap_srams_bist_command,
    input wire  [4-1:0] rtap_srams_bist_data

);
/*
localparam reset = 2'd0;
localparam init  = 2'd1;
localparam done  = 2'd2;

reg [1:0] init_state_f;
reg [1:0] init_state_next;
reg [`L2_DIR_INDEX_WIDTH-1:0] init_counter_f;
reg [`L2_DIR_INDEX_WIDTH-1:0] init_counter_next;

reg [`L2_DIR_INDEX_WIDTH-1:0] addr_real;
reg rdw_en_real;
reg clk_en_real;
reg [`L2_DIR_ARRAY_WIDTH-1:0] data_in_real;
reg [`L2_DIR_ARRAY_WIDTH-1:0] data_mask_in_real;

always @ *
begin
    if (!rst_n)
    begin
        init_state_next = reset;
    end
    else
    begin
        if (init_state_f == reset)
        begin
            init_state_next = init;
        end
        else if ((init_state_f == init) && (init_counter_f == {`L2_DIR_INDEX_WIDTH{1'b1}}))
        begin
            init_state_next = done;
        end
        else
        begin
            init_state_next = init_state_f;
        end
    end
end

always @ (posedge clk)
begin
    init_state_f <= init_state_next;
end

always @ *
begin
    if ((init_state_f == reset) || (init_state_f == done))
    begin
        init_counter_next = {`L2_DIR_INDEX_WIDTH{1'b0}};
    end
    else
    begin
        init_counter_next = init_counter_f + 1;
    end
end


always @ (posedge clk)
begin
    init_counter_f <= init_counter_next;
end


always @ *
begin
    init_done = (init_state_f == done);
end


always @ *
begin
    init_done = (init_state_f == done);
end


always @ *
begin
    if (init_state_f == init)
    begin
        clk_en_real = 1'b1;
        rdw_en_real = 1'b0;
        addr_real = init_counter_f;
        data_in_real = {`L2_DIR_ARRAY_WIDTH{1'b0}};
        data_mask_in_real = {`L2_DIR_ARRAY_WIDTH{1'b1}};
    end
    else
    begin
        clk_en_real = clk_en;
        rdw_en_real = rdw_en;
        addr_real = addr;
        data_in_real = data_in;
        data_mask_in_real = data_mask_in;
    end
end
*/


// sram_1rw_1024x64 l2_dir_array(
sram_l2_dir l2_dir_array(
    .MEMCLK     (clk),
    .RESET_N(rst_n),
    .CE         (clk_en),

    .A          (addr),
    .DIN        (data_in),
    .RDWEN      (rdw_en),
    .BW         (data_mask_in),

    .DOUT       (data_out),
    .BIST_COMMAND(rtap_srams_bist_command),
    .BIST_DIN(rtap_srams_bist_data),
    .BIST_DOUT(srams_rtap_data),
    .SRAMID(8'd14)
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
//  Filename      : l2_dir_wrap.v
//  Created On    : 2014-02-24
//  Revision      :
//  Author        : Yaosheng Fu
//  Company       : Princeton University
//  Email         : yfu@princeton.edu
//
//  Description   : The wrap module for directory array in the L2 cache
//
//
//==================================================================================================

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
//  Filename      : l2.h.pyv
//  Created On    : 2014-02-20
//  Revision      :
//  Author        : Yaosheng Fu
//  Company       : Princeton University
//  Email         : yfu@princeton.edu
//
//  Description   : header file for the L2 cache
//
//
//==================================================================================================

// devices.xml



// Input buffer for pipeline1













// Input buffer for pipeline2














// Output buffer















// L2 cache configuration







//`define L2_SIZE                  65536
//`define L2_SIZE_WIDTH            16






//`define L2_LINE_SIZE             64
//`define L2_LINE_SIZE_WIDTH       6













//`define L2_WAYS                  4
//`define L2_WAYS_WIDTH            2
//`define L2_WAY_0                 2'b00
//`define L2_WAY_1                 2'b01
//`define L2_WAY_2                 2'b10
//`define L2_WAY_3                 2'b11





// Tag array







//`define L2_TAG_INDEX_WIDTH      8
//`define L2_TAG_WIDTH            26
//`define L2_TAG_WAY_WIDTH        26
//`define L2_TAG_ARRAY_WIDTH      104


// Tag Address decomposition 




//`define L2_TAG_INDEX            13:6
//`define L2_TAG                  39:14


//Data array












//`define L2_DATA_INDEX_WIDTH         12 








// Data Address decomposition 







//Dir array



//`define L2_DIR_INDEX_WIDTH      10 





//State array

















//Whether the cache line is in Icaches or Dcaches
















//Round Robin selection














// State decomposition 












//MSHR array



































//`define L2_MSHR_CMP_ADDR        13:6
//`define L2_MSHR_ADDR            39:0
//`define L2_MSHR_WAY             41:40
//`define L2_MSHR_MSHRID          49:42
//`define L2_MSHR_CACHE_TYPE      50
//`define L2_MSHR_DATA_SIZE       53:51
//`define L2_MSHR_MSG_TYPE        61:54
//`define L2_MSHR_L2_MISS         62
//`define L2_MSHR_SRC_CHIPID      76:63
//`define L2_MSHR_SRC_X           84:77
//`define L2_MSHR_SRC_Y           92:85
//`define L2_MSHR_SRC_FBITS       96:93
//`define L2_MSHR_SDID            106:97
//`define L2_MSHR_LSID            112:107      
//`define L2_MSHR_MISS_LSID       118:113
//`define L2_MSHR_SMC_MISS        119
//`define L2_MSHR_RECYCLED        120
//`define L2_MSHR_INV_FWD_PENDING 121


//SMC array


































//Message destination_type







//L2 public sharer beyond the maximum clump size



//L2 registers



//Special addresses





















//L2 core ID





//L2 dir array reuse







//Control Signal 








//AMO ALU OP macros












//Control Signal in Stage 1























































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






































































































































































































































































































































































































































































































































































































module l2_dir_wrap(

    input wire clk,
    input wire rst_n,
    input wire clk_en1,
    input wire clk_en2,
    input wire rdw_en1,
    input wire rdw_en2,
    input wire pdout_en,
    input wire deepsleep,
    input wire pipe_sel,

    input wire [8+2-1:0] addr1,
    input wire [64-1:0] data_in1,
    input wire [64-1:0] data_mask_in1,

    input wire [8+2-1:0] addr2,
    input wire [64-1:0] data_in2,
    input wire [64-1:0] data_mask_in2,

    output wire [64-1:0] data_out,
    output wire [64-1:0] pdata_out,

    // sram interface
    output wire [4-1:0] srams_rtap_data,
    input wire  [4-1:0] rtap_srams_bist_command,
    input wire  [4-1:0] rtap_srams_bist_data

);

reg clk_en;
reg rdw_en;
reg [8+2-1:0] addr;
reg [64-1:0] data_in;
reg [64-1:0] data_mask_in;

always @ *
begin
    if (pipe_sel)
    begin
        clk_en = clk_en2;
        rdw_en = rdw_en2;
        addr = addr2;
        data_in = data_in2;
        data_mask_in = data_mask_in2;
    end
    else
    begin
        clk_en = clk_en1;
        rdw_en = rdw_en1;
        addr = addr1;
        data_in = data_in1;
        data_mask_in = data_mask_in1;
    end
end

l2_dir l2_dir(

    .clk            (clk),
    .rst_n          (rst_n),
    .clk_en         (clk_en),
    .rdw_en         (rdw_en),
    .pdout_en       (pdout_en),
    .deepsleep      (deepsleep),
    .addr           (addr),
    .data_in        (data_in),
    .data_mask_in   (data_mask_in),
    .data_out       (data_out),
    .pdata_out      (pdata_out),

    // sram interfaces
    .srams_rtap_data (srams_rtap_data),
    .rtap_srams_bist_command (rtap_srams_bist_command),
    .rtap_srams_bist_data (rtap_srams_bist_data)
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
//  Filename      : l2_encoder.v
//  Created On    : 2014-03-03
//  Revision      :
//  Author        : Yaosheng Fu
//  Company       : Princeton University
//  Email         : yfu@princeton.edu
//
//  Description   : The message encoder in the L2 cache
//
//
//====================================================================================================


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
//  Filename      : l2.h.pyv
//  Created On    : 2014-02-20
//  Revision      :
//  Author        : Yaosheng Fu
//  Company       : Princeton University
//  Email         : yfu@princeton.edu
//
//  Description   : header file for the L2 cache
//
//
//==================================================================================================

// devices.xml



// Input buffer for pipeline1













// Input buffer for pipeline2














// Output buffer















// L2 cache configuration







//`define L2_SIZE                  65536
//`define L2_SIZE_WIDTH            16






//`define L2_LINE_SIZE             64
//`define L2_LINE_SIZE_WIDTH       6













//`define L2_WAYS                  4
//`define L2_WAYS_WIDTH            2
//`define L2_WAY_0                 2'b00
//`define L2_WAY_1                 2'b01
//`define L2_WAY_2                 2'b10
//`define L2_WAY_3                 2'b11





// Tag array







//`define L2_TAG_INDEX_WIDTH      8
//`define L2_TAG_WIDTH            26
//`define L2_TAG_WAY_WIDTH        26
//`define L2_TAG_ARRAY_WIDTH      104


// Tag Address decomposition 




//`define L2_TAG_INDEX            13:6
//`define L2_TAG                  39:14


//Data array












//`define L2_DATA_INDEX_WIDTH         12 








// Data Address decomposition 







//Dir array



//`define L2_DIR_INDEX_WIDTH      10 





//State array

















//Whether the cache line is in Icaches or Dcaches
















//Round Robin selection














// State decomposition 












//MSHR array



































//`define L2_MSHR_CMP_ADDR        13:6
//`define L2_MSHR_ADDR            39:0
//`define L2_MSHR_WAY             41:40
//`define L2_MSHR_MSHRID          49:42
//`define L2_MSHR_CACHE_TYPE      50
//`define L2_MSHR_DATA_SIZE       53:51
//`define L2_MSHR_MSG_TYPE        61:54
//`define L2_MSHR_L2_MISS         62
//`define L2_MSHR_SRC_CHIPID      76:63
//`define L2_MSHR_SRC_X           84:77
//`define L2_MSHR_SRC_Y           92:85
//`define L2_MSHR_SRC_FBITS       96:93
//`define L2_MSHR_SDID            106:97
//`define L2_MSHR_LSID            112:107      
//`define L2_MSHR_MISS_LSID       118:113
//`define L2_MSHR_SMC_MISS        119
//`define L2_MSHR_RECYCLED        120
//`define L2_MSHR_INV_FWD_PENDING 121


//SMC array


































//Message destination_type







//L2 public sharer beyond the maximum clump size



//L2 registers



//Special addresses





















//L2 core ID





//L2 dir array reuse







//Control Signal 








//AMO ALU OP macros












//Control Signal in Stage 1























































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





































































































































































































































































































































































































































































































































































































module l2_encoder(

    input wire [14-1:0] msg_dst_chipid,
    input wire [8-1:0] msg_dst_x,
    input wire [8-1:0] msg_dst_y,
    input wire [4-1:0] msg_dst_fbits,
    input wire [8-1:0] msg_length,
    input wire [8-1:0] msg_type,
    input wire [8-1:0] msg_mshrid,
    input wire [3-1:0] msg_data_size,
    input wire [1-1:0] msg_cache_type,
    input wire [4-1:0] msg_subline_vector,
    input wire [2-1:0] msg_mesi,
    input wire [1-1:0] msg_l2_miss,
    input wire [1-1:0] msg_last_subline,
    input wire [2-1:0] msg_subline_id,
    input wire [40-1:0] msg_addr,
    input wire [14-1:0] msg_src_chipid,
    input wire [8-1:0] msg_src_x,
    input wire [8-1:0] msg_src_y,
    input wire [4-1:0] msg_src_fbits,
    input wire [10-1:0] msg_sdid,
    input wire [6-1:0] msg_lsid,


    output reg [192-1:0] msg_header
);

always @ *
begin
    msg_header = {msg_src_chipid,
                  msg_src_x,
                  msg_src_y,
                  msg_src_fbits,
                  msg_sdid,
                  msg_lsid,
                  14'd0,

                  8'd0,
                  msg_addr,
                  msg_subline_vector,
                  msg_cache_type,
                  msg_data_size,
                  8'd0,

                  msg_dst_chipid,
                  msg_dst_x,
                  msg_dst_y,
                  msg_dst_fbits,
                  msg_length,
                  msg_type,
                  msg_mshrid,
                  msg_mesi,
                  msg_l2_miss,
                  msg_subline_id,
                  msg_last_subline};
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

//==================================================================================================
//  Filename      : l2_mshr_decoder.v
//  Created On    : 2014-03-03
//  Revision      :
//  Author        : Yaosheng Fu
//  Company       : Princeton University
//  Email         : yfu@princeton.edu
//
//  Description   : The mshr decoder in the L2 cache
//
//
//====================================================================================================


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
//  Filename      : l2.h.pyv
//  Created On    : 2014-02-20
//  Revision      :
//  Author        : Yaosheng Fu
//  Company       : Princeton University
//  Email         : yfu@princeton.edu
//
//  Description   : header file for the L2 cache
//
//
//==================================================================================================

// devices.xml



// Input buffer for pipeline1













// Input buffer for pipeline2














// Output buffer















// L2 cache configuration







//`define L2_SIZE                  65536
//`define L2_SIZE_WIDTH            16






//`define L2_LINE_SIZE             64
//`define L2_LINE_SIZE_WIDTH       6













//`define L2_WAYS                  4
//`define L2_WAYS_WIDTH            2
//`define L2_WAY_0                 2'b00
//`define L2_WAY_1                 2'b01
//`define L2_WAY_2                 2'b10
//`define L2_WAY_3                 2'b11





// Tag array







//`define L2_TAG_INDEX_WIDTH      8
//`define L2_TAG_WIDTH            26
//`define L2_TAG_WAY_WIDTH        26
//`define L2_TAG_ARRAY_WIDTH      104


// Tag Address decomposition 




//`define L2_TAG_INDEX            13:6
//`define L2_TAG                  39:14


//Data array












//`define L2_DATA_INDEX_WIDTH         12 








// Data Address decomposition 







//Dir array



//`define L2_DIR_INDEX_WIDTH      10 





//State array

















//Whether the cache line is in Icaches or Dcaches
















//Round Robin selection














// State decomposition 












//MSHR array



































//`define L2_MSHR_CMP_ADDR        13:6
//`define L2_MSHR_ADDR            39:0
//`define L2_MSHR_WAY             41:40
//`define L2_MSHR_MSHRID          49:42
//`define L2_MSHR_CACHE_TYPE      50
//`define L2_MSHR_DATA_SIZE       53:51
//`define L2_MSHR_MSG_TYPE        61:54
//`define L2_MSHR_L2_MISS         62
//`define L2_MSHR_SRC_CHIPID      76:63
//`define L2_MSHR_SRC_X           84:77
//`define L2_MSHR_SRC_Y           92:85
//`define L2_MSHR_SRC_FBITS       96:93
//`define L2_MSHR_SDID            106:97
//`define L2_MSHR_LSID            112:107      
//`define L2_MSHR_MISS_LSID       118:113
//`define L2_MSHR_SMC_MISS        119
//`define L2_MSHR_RECYCLED        120
//`define L2_MSHR_INV_FWD_PENDING 121


//SMC array


































//Message destination_type







//L2 public sharer beyond the maximum clump size



//L2 registers



//Special addresses





















//L2 core ID





//L2 dir array reuse







//Control Signal 








//AMO ALU OP macros












//Control Signal in Stage 1























































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





































































































































































































































































































































































































































































































































































































module l2_mshr_decoder(

    input wire [120+2-1:0] data_in,

    output reg [40-1:0] addr_out,
    output reg [2-1:0] way_out,
    output reg [8-1:0] mshrid_out,
    output reg [1-1:0] cache_type_out,
    output reg [3-1:0] data_size_out,
    output reg [8-1:0] msg_type_out,
    output reg [1-1:0] msg_l2_miss_out,
    output reg [14-1:0] src_chipid_out,
    output reg [8-1:0] src_x_out,
    output reg [8-1:0] src_y_out,
    output reg [4-1:0] src_fbits_out,
    output reg [10-1:0] sdid_out,
    output reg [6-1:0] lsid_out,
    output reg [6-1:0] miss_lsid_out,
    output reg smc_miss_out,
    output reg recycled,
    output reg inv_fwd_pending

);


always @ *
begin
    addr_out = data_in[39:0];
    way_out = data_in[39+2:40];
    mshrid_out = data_in[47+2:40+2];
    cache_type_out = data_in[48+2];
    data_size_out = data_in[51+2:49+2];
    msg_type_out = data_in[59+2:52+2];
    msg_l2_miss_out = data_in[60+2];
    src_chipid_out = data_in[74+2:61+2];
    src_x_out = data_in[82+2:75+2];
    src_y_out = data_in[90+2:83+2];
    src_fbits_out = data_in[94+2:91+2];
    sdid_out = data_in[104+2:95+2];
    lsid_out = data_in[110+2:105+2];
    miss_lsid_out = data_in[116+2:111+2];
    smc_miss_out = data_in[117+2];
    recycled = data_in[118+2];
    inv_fwd_pending = data_in[119+2];
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

//==================================================================================================
//  Filename      : l2_pipe1.v
//  Created On    : 2014-02-20
//  Revision      :
//  Author        : Yaosheng Fu
//  Company       : Princeton University
//  Email         : yfu@princeton.edu
//
//  Description   : Pipeline 1 for the L2 cache
//
//
//====================================================================================================


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
//  Filename      : l2.h.pyv
//  Created On    : 2014-02-20
//  Revision      :
//  Author        : Yaosheng Fu
//  Company       : Princeton University
//  Email         : yfu@princeton.edu
//
//  Description   : header file for the L2 cache
//
//
//==================================================================================================

// devices.xml



// Input buffer for pipeline1













// Input buffer for pipeline2














// Output buffer















// L2 cache configuration







//`define L2_SIZE                  65536
//`define L2_SIZE_WIDTH            16






//`define L2_LINE_SIZE             64
//`define L2_LINE_SIZE_WIDTH       6













//`define L2_WAYS                  4
//`define L2_WAYS_WIDTH            2
//`define L2_WAY_0                 2'b00
//`define L2_WAY_1                 2'b01
//`define L2_WAY_2                 2'b10
//`define L2_WAY_3                 2'b11





// Tag array







//`define L2_TAG_INDEX_WIDTH      8
//`define L2_TAG_WIDTH            26
//`define L2_TAG_WAY_WIDTH        26
//`define L2_TAG_ARRAY_WIDTH      104


// Tag Address decomposition 




//`define L2_TAG_INDEX            13:6
//`define L2_TAG                  39:14


//Data array












//`define L2_DATA_INDEX_WIDTH         12 








// Data Address decomposition 







//Dir array



//`define L2_DIR_INDEX_WIDTH      10 





//State array

















//Whether the cache line is in Icaches or Dcaches
















//Round Robin selection














// State decomposition 












//MSHR array



































//`define L2_MSHR_CMP_ADDR        13:6
//`define L2_MSHR_ADDR            39:0
//`define L2_MSHR_WAY             41:40
//`define L2_MSHR_MSHRID          49:42
//`define L2_MSHR_CACHE_TYPE      50
//`define L2_MSHR_DATA_SIZE       53:51
//`define L2_MSHR_MSG_TYPE        61:54
//`define L2_MSHR_L2_MISS         62
//`define L2_MSHR_SRC_CHIPID      76:63
//`define L2_MSHR_SRC_X           84:77
//`define L2_MSHR_SRC_Y           92:85
//`define L2_MSHR_SRC_FBITS       96:93
//`define L2_MSHR_SDID            106:97
//`define L2_MSHR_LSID            112:107      
//`define L2_MSHR_MISS_LSID       118:113
//`define L2_MSHR_SMC_MISS        119
//`define L2_MSHR_RECYCLED        120
//`define L2_MSHR_INV_FWD_PENDING 121


//SMC array


































//Message destination_type







//L2 public sharer beyond the maximum clump size



//L2 registers



//Special addresses





















//L2 core ID





//L2 dir array reuse







//Control Signal 








//AMO ALU OP macros












//Control Signal in Stage 1























































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





































































































































































































































































































































































































































































































































































































module l2_pipe1(

    input wire clk,
    input wire rst_n,
    input wire [34-1:0] my_nodeid,
    
    input wire csm_en,
    
    input wire [22-1:0] smt_base_addr,
  
   //inputs from NOC1
   
    input wire noc_valid_in,
    input wire [64-1:0] noc_data_in,
    output wire noc_ready_in,
    

    //outputs to NOC2
   
    output wire noc_valid_out,
    output wire [64-1:0] noc_data_out,
    input wire  noc_ready_out,

    input wire pipe2_valid_S1,
    input wire pipe2_valid_S2,
    input wire pipe2_valid_S3,
    input wire [8-1:0] pipe2_msg_type_S1,
    input wire [8-1:0] pipe2_msg_type_S2,
    input wire [8-1:0] pipe2_msg_type_S3,
    input wire [40-1:0] pipe2_addr_S1,
    input wire [40-1:0] pipe2_addr_S2,
    input wire [40-1:0] pipe2_addr_S3,
    input wire global_stall_S1,
    input wire global_stall_S2,
    input wire global_stall_S4,

    input wire mshr_hit,

    input wire [120+2-1:0] mshr_data_out,



 // L2_CAM_MSHR
    input wire [6-1:0] mshr_inv_counter_out,
    input wire [3:0] mshr_empty_slots,
    input wire mshr_pending,
    input wire [3-1:0] mshr_pending_index,
    input wire [3-1:0] mshr_empty_index,

    
    input wire broadcast_counter_zero,
    input wire broadcast_counter_max,
    input wire broadcast_counter_avail,
    input wire [14-1:0] broadcast_chipid_out,
    input wire [8-1:0] broadcast_x_out,
    input wire [8-1:0] broadcast_y_out,
    

    input wire [15*4+2+4-1:0] state_data_out,
    
    input wire [104-1:0] tag_data_out,

    input wire [64-1:0] dir_data_out,

    input wire [144-1:0] data_data_out,

    
    input wire smc_hit,
    input wire [30-1:0] smc_data_out,
    input wire [4-1:0] smc_valid_out,
    input wire [14-1:0] smc_tag_out,
    

    input wire [64-1:0] reg_data_out,

    output wire mshr_cam_en,
    output wire mshr_wr_state_en,
    output wire mshr_wr_data_en,
    output wire mshr_pending_ready,
    output wire [2-1:0] mshr_state_in,
    output wire [120+2-1:0] mshr_data_in,
    output wire [120+2-1:0] mshr_data_mask_in,
    output wire [3-1:0] mshr_inv_counter_rd_index_in,
    output wire [3-1:0] mshr_wr_index_in,
    output wire [8-1:0] mshr_addr_in,

    output wire state_rd_en,
    output wire state_wr_en,
    output wire [8-1:0] state_rd_addr,
    output wire [8-1:0] state_wr_addr,
    output wire [15*4+2+4-1:0] state_data_in,
    output wire [15*4+2+4-1:0] state_data_mask_in,


    output wire tag_clk_en,
    output wire tag_rdw_en,
    output wire [8-1:0] tag_addr,
    output wire [104-1:0] tag_data_in,
    output wire [104-1:0] tag_data_mask_in,

    output wire dir_clk_en,
    output wire dir_rdw_en,
    output wire [8+2-1:0] dir_addr,
    output wire [64-1:0] dir_data_in,
    output wire [64-1:0] dir_data_mask_in,

    output wire data_clk_en,
    output wire data_rdw_en,
    output wire [8+2+2-1:0] data_addr,
    output wire [144-1:0] data_data_in,
    output wire [144-1:0] data_data_mask_in,

    
    output wire [2-1:0] broadcast_counter_op,
    output wire broadcast_counter_op_val,
    

    
    output wire smc_rd_en,
    output wire [16-1:0] smc_rd_addr_in,
    output wire smc_rd_diag_en,
    output wire smc_flush_en,
    output wire [2-1:0] smc_addr_op,
    output wire smc_wr_en,
    output wire smc_wr_diag_en,
    output wire [16-1:0] smc_wr_addr_in,
    output wire [128-1:0] smc_data_in,
    

    output wire l2_access_valid,
    output wire l2_miss_valid,
    output wire data_ecc_corr_error,
    output wire data_ecc_uncorr_error,
    output wire [8+2+2-1:0] data_ecc_addr,
    output wire [40-1:0] error_addr,

    output wire reg_rd_en,
    output wire reg_wr_en,
    output wire [8-1:0] reg_rd_addr_type,
    output wire [8-1:0] reg_wr_addr_type,
    output wire [64-1:0] reg_data_in

);


wire [8-1:0] msg_type;
wire [8-1:0] msg_length;
wire [8-1:0] msg_mshrid;
wire [3-1:0] msg_data_size;
wire [1-1:0] msg_cache_type;
wire [40-1:0] msg_addr;
wire [14-1:0] msg_src_chipid;
wire [8-1:0] msg_src_x;
wire [8-1:0] msg_src_y;
wire [4-1:0] msg_src_fbits;
wire [10-1:0] msg_sdid;
wire [6-1:0] msg_lsid;


wire [8-1:0] mshr_msg_type;
wire [8-1:0] mshr_mshrid;
wire [3-1:0] mshr_data_size;
wire [1-1:0] mshr_cache_type;
wire [40-1:0] mshr_addr;
wire [2-1:0] mshr_way;
wire [1-1:0] mshr_l2_miss;
wire [14-1:0] mshr_src_chipid;
wire [8-1:0] mshr_src_x;
wire [8-1:0] mshr_src_y;
wire [4-1:0] mshr_src_fbits;
wire [10-1:0] mshr_sdid;
wire [6-1:0] mshr_lsid;
wire [6-1:0] mshr_miss_lsid;















 // L2_CAM_MSHR


wire mshr_smc_miss;


 // L2_CAM_MSHR


wire mshr_recycled;





















 // L2_CAM_MSHR

wire msg_header_valid;
wire [192-1:0] msg_header;
wire msg_header_ready;

wire msg_data_valid;
wire [64-1:0] msg_data;
wire msg_data_ready;


wire valid_S1; 
wire stall_S1;  
wire msg_from_mshr_S1;
wire [40-1:0] addr_S1;
wire dis_flush_S1;


wire [4-1:0] amo_alu_op_S2;

wire valid_S2; 
wire stall_S2;  
wire stall_before_S2; 
wire stall_real_S2; 
wire msg_from_mshr_S2;
wire [8-1:0] msg_type_S2;
wire [3-1:0] data_size_S2;
wire [1-1:0] cache_type_S2;
wire state_owner_en_S2;
wire [2-1:0] state_owner_op_S2;
wire state_subline_en_S2;
wire [2-1:0] state_subline_op_S2;
wire state_di_en_S2;
wire state_vd_en_S2;
wire [2-1:0] state_vd_S2;
wire state_mesi_en_S2;
wire [2-1:0] state_mesi_S2;
wire state_lru_en_S2;
wire [1-1:0] state_lru_op_S2;
wire state_rb_en_S2;
wire l2_ifill_S2;
wire [2-1:0] l2_load_data_subline_S2;
wire [40-1:0] addr_S2;
wire l2_tag_hit_S2;
wire l2_evict_S2;
wire l2_wb_S2;
wire [2-1:0] l2_way_state_mesi_S2;
wire [2-1:0] l2_way_state_vd_S2;
wire [1-1:0] l2_way_state_cache_type_S2;
wire [4-1:0] l2_way_state_subline_S2;
wire [2-1:0] dir_op_S2;
wire req_from_owner_S2;
wire addr_l2_aligned_S2;
wire special_addr_type_S2;
wire [6-1:0] lsid_S2;
wire state_load_sdid_S2;


wire valid_S3; 
wire stall_S3;  
wire stall_before_S3; 
wire [40-1:0] addr_S3;

wire valid_S4;    
wire stall_S4;
wire stall_before_S4; 
wire [6-1:0] dir_sharer_S4;
wire [6-1:0] dir_sharer_counter_S4;
wire cas_cmp_en_S4;
wire [3-1:0] cas_cmp_data_size_S4;
wire [40-1:0] addr_S4;
wire l2_evict_S4;
wire l2_tag_hit_S4;
wire [2-1:0] l2_way_state_mesi_S4;
wire [6-1:0] l2_way_state_owner_S4;
wire [2-1:0] l2_way_state_vd_S4;
wire [4-1:0] l2_way_state_subline_S4;
wire [1-1:0] l2_way_state_cache_type_S4;
wire [8-1:0] mshrid_S4;
wire req_from_owner_S4;
wire cas_cmp_S4;
wire atomic_read_data_en_S4;
wire [8-1:0] msg_type_S4;
wire [3-1:0] data_size_S4;
wire [1-1:0] cache_type_S4;
wire [1-1:0] l2_miss_S4;
wire [6-1:0] mshr_miss_lsid_S4;
wire [6-1:0] lsid_S4;
wire special_addr_type_S4;
wire state_wr_sel_S4;
wire [64-1:0] dir_data_S4;
wire [64-1:0] dir_data_sel_S4;

wire smc_miss_S4;
wire stall_smc_buf_S4;
    
wire msg_from_mshr_S4;
wire req_recycle_S4;
wire inv_fwd_pending_S4;

wire msg_send_valid;
wire msg_send_ready;
wire [3-1:0] msg_send_mode;
wire [8-1:0] msg_send_type;
wire [8-1:0] msg_send_type_pre;
wire [8-1:0] msg_send_length;
wire [3-1:0] msg_send_data_size;
wire [1-1:0] msg_send_cache_type;
wire [1-1:0] msg_send_l2_miss;
wire [2-1:0] msg_send_mesi;
wire [8-1:0] msg_send_mshrid;
wire [4-1:0] msg_send_subline_vector;
wire [40-1:0] msg_send_addr;
wire [14-1:0] msg_send_dst_chipid;
wire [8-1:0] msg_send_dst_x;
wire [8-1:0] msg_send_dst_y;
wire [4-1:0] msg_send_dst_fbits;
wire [64*2-1:0] msg_send_data;
wire [64*3-1:0] msg_send_header;


assign error_addr = addr_S4;

l2_pipe1_buf_in buf_in(
    .clk                    (clk),
    .rst_n                  (rst_n),

    .valid_in               (noc_valid_in),
    .data_in                (noc_data_in),
    .ready_in               (noc_ready_in),

    .msg_header_valid_out   (msg_header_valid),
    .msg_header_out         (msg_header),
    .msg_header_ready_out   (msg_header_ready),

    .msg_data_valid_out     (msg_data_valid),
    .msg_data_out           (msg_data),
    .msg_data_ready_out     (msg_data_ready)
);


l2_decoder decoder(
    .msg_header         (msg_header),
    .msg_type           (msg_type),
    .msg_length         (msg_length),
    .msg_mshrid         (msg_mshrid),
    .msg_data_size      (msg_data_size),
    .msg_cache_type     (msg_cache_type),
    .msg_subline_vector (),
    .msg_mesi           (),
    .msg_l2_miss        (),
    .msg_subline_id     (),
    .msg_last_subline   (),
    .msg_addr           (msg_addr),
    .msg_src_chipid     (msg_src_chipid),
    .msg_src_x          (msg_src_x),
    .msg_src_y          (msg_src_y),
    .msg_src_fbits      (msg_src_fbits),
    .msg_sdid           (msg_sdid),
    .msg_lsid           (msg_lsid)
);


l2_mshr_decoder mshr_decoder(

    .data_in            (mshr_data_out),
    .addr_out           (mshr_addr),
    .way_out            (mshr_way),
    .mshrid_out         (mshr_mshrid),
    .cache_type_out     (mshr_cache_type),
    .data_size_out      (mshr_data_size),
    .msg_type_out       (mshr_msg_type),
    .msg_l2_miss_out    (mshr_l2_miss),
    .src_chipid_out     (mshr_src_chipid),
    .src_x_out          (mshr_src_x),
    .src_y_out          (mshr_src_y),
    .src_fbits_out      (mshr_src_fbits),
    .sdid_out           (mshr_sdid),
    .lsid_out           (mshr_lsid),
    .miss_lsid_out      (mshr_miss_lsid),
    
    .smc_miss_out       (mshr_smc_miss),
    


    .recycled           (mshr_recycled),
    .inv_fwd_pending    ()
);
































































 // L2_CAM_MSHR

l2_pipe1_ctrl ctrl(

    .clk                        (clk),
    .rst_n                      (rst_n),
    
    .csm_en                     (csm_en),
    

    .pipe2_valid_S1             (pipe2_valid_S1),
    .pipe2_valid_S2             (pipe2_valid_S2),
    .pipe2_valid_S3             (pipe2_valid_S3),
    .pipe2_msg_type_S1          (pipe2_msg_type_S1),
    .pipe2_msg_type_S2          (pipe2_msg_type_S2),
    .pipe2_msg_type_S3          (pipe2_msg_type_S3),
    .pipe2_addr_S1              (pipe2_addr_S1),
    .pipe2_addr_S2              (pipe2_addr_S2),
    .pipe2_addr_S3              (pipe2_addr_S3),

    .global_stall_S1            (global_stall_S1),
    .msg_header_valid_S1        (msg_header_valid),
    .msg_type_S1                (msg_type),
    .msg_data_size_S1           (msg_data_size),
    .msg_cache_type_S1          (msg_cache_type),
    .mshr_hit_S1                (mshr_hit),

    .mshr_msg_type_S1           (mshr_msg_type),
    .mshr_l2_miss_S1            (mshr_l2_miss),
    .mshr_data_size_S1          (mshr_data_size),
    .mshr_cache_type_S1         (mshr_cache_type),





 // L2_CAM_MSHR
    .mshr_pending_S1            (mshr_pending),
    .mshr_pending_index_S1      (mshr_pending_index),
    .mshr_empty_slots_S1        (mshr_empty_slots),
    

    .mshr_smc_miss_S1           (mshr_smc_miss),


 // L2_CAM_MSHR
    








 // L2_CAM_MSHR
    .msg_data_valid_S1          (msg_data_valid),
    .addr_S1                    (addr_S1),
   
    .global_stall_S2            (global_stall_S2),
    .l2_tag_hit_S2              (l2_tag_hit_S2),
    .l2_evict_S2                (l2_evict_S2),
    .l2_wb_S2                   (l2_wb_S2),
    .l2_way_state_mesi_S2       (l2_way_state_mesi_S2),
    .l2_way_state_vd_S2         (l2_way_state_vd_S2),
    .l2_way_state_cache_type_S2 (l2_way_state_cache_type_S2),
    .l2_way_state_subline_S2    (l2_way_state_subline_S2),
    .req_from_owner_S2          (req_from_owner_S2),
    .addr_l2_aligned_S2         (addr_l2_aligned_S2),
    .lsid_S2                    (lsid_S2),
    .msg_data_valid_S2          (msg_data_valid),
    .addr_S2                    (addr_S2),

    .dir_data_S3                (dir_data_out),
    .addr_S3                    (addr_S3),

    .global_stall_S4            (global_stall_S4),
    .l2_evict_S4                (l2_evict_S4),
    .l2_tag_hit_S4              (l2_tag_hit_S4),
    .l2_way_state_mesi_S4       (l2_way_state_mesi_S4),
    .l2_way_state_owner_S4      (l2_way_state_owner_S4),
    .l2_way_state_vd_S4         (l2_way_state_vd_S4),
    .l2_way_state_subline_S4    (l2_way_state_subline_S4),
    .l2_way_state_cache_type_S4 (l2_way_state_cache_type_S4),
    .mshrid_S4                  (mshrid_S4),
    .req_from_owner_S4          (req_from_owner_S4),
    .mshr_miss_lsid_S4          (mshr_miss_lsid_S4),
    .lsid_S4                    (lsid_S4),
    .addr_S4                    (addr_S4),
    .cas_cmp_S4                 (cas_cmp_S4),
    .msg_send_ready_S4          (msg_send_ready),
    .mshr_empty_index_S4        (mshr_empty_index),
    
    
    .smc_hit_S4                 (smc_hit),
    .broadcast_counter_zero_S4  (broadcast_counter_zero),
    .broadcast_counter_max_S4   (broadcast_counter_max),
    .broadcast_counter_avail_S4 (broadcast_counter_avail),
    .broadcast_chipid_out_S4    (broadcast_chipid_out),
    .broadcast_x_out_S4         (broadcast_x_out),
    .broadcast_y_out_S4         (broadcast_y_out),
    

    .valid_S1                   (valid_S1),  
    .stall_S1                   (stall_S1),    
    .msg_from_mshr_S1           (msg_from_mshr_S1), 
    .dis_flush_S1               (dis_flush_S1),
    .mshr_cam_en_S1             (mshr_cam_en),
    .mshr_pending_ready_S1      (mshr_pending_ready),
    .msg_header_ready_S1        (msg_header_ready),
    .tag_clk_en_S1              (tag_clk_en),
    .tag_rdw_en_S1              (tag_rdw_en),
    .state_rd_en_S1             (state_rd_en),
    .reg_wr_en_S1               (reg_wr_en),
    .reg_wr_addr_type_S1        (reg_wr_addr_type),


    .valid_S2                   (valid_S2),    
    .stall_S2                   (stall_S2), 
    .stall_before_S2            (stall_before_S2), 
    .stall_real_S2              (stall_real_S2),
    .msg_type_S2                (msg_type_S2),
    .msg_from_mshr_S2           (msg_from_mshr_S2),
    .special_addr_type_S2       (special_addr_type_S2),
    .dir_clk_en_S2              (dir_clk_en),
    .dir_rdw_en_S2              (dir_rdw_en),
    .dir_op_S2                  (dir_op_S2),
    .data_clk_en_S2             (data_clk_en),
    .data_rdw_en_S2             (data_rdw_en),
    .amo_alu_op_S2              (amo_alu_op_S2),
    .data_size_S2               (data_size_S2),
    .cache_type_S2              (cache_type_S2),
    .state_owner_en_S2          (state_owner_en_S2),
    .state_owner_op_S2          (state_owner_op_S2),
    .state_subline_en_S2        (state_subline_en_S2),
    .state_subline_op_S2        (state_subline_op_S2),   
    .state_di_en_S2             (state_di_en_S2),
    .state_vd_en_S2             (state_vd_en_S2),
    .state_vd_S2                (state_vd_S2),
    .state_mesi_en_S2           (state_mesi_en_S2),
    .state_mesi_S2              (state_mesi_S2),
    .state_lru_en_S2            (state_lru_en_S2),
    .state_lru_op_S2            (state_lru_op_S2),
    .state_rb_en_S2             (state_rb_en_S2),
    .state_load_sdid_S2         (state_load_sdid_S2),
    .l2_ifill_S2                (l2_ifill_S2),
    .l2_load_data_subline_S2    (l2_load_data_subline_S2),
    .msg_data_ready_S2          (msg_data_ready),
    
    .smc_wr_en_S2               (smc_wr_en),
    .smc_wr_diag_en_S2          (smc_wr_diag_en),
    .smc_flush_en_S2            (smc_flush_en),
    .smc_addr_op_S2             (smc_addr_op),
        

    .valid_S3                   (valid_S3),    
    .stall_S3                   (stall_S3), 
    .stall_before_S3            (stall_before_S3), 

    .valid_S4                   (valid_S4),    
    .stall_S4                   (stall_S4), 
    .stall_before_S4            (stall_before_S4),
     
    .stall_smc_buf_S4           (stall_smc_buf_S4),
    
    .msg_from_mshr_S4           (msg_from_mshr_S4),
    .req_recycle_S4             (req_recycle_S4),
    .inv_fwd_pending_S4         (inv_fwd_pending_S4),
    .dir_sharer_S4              (dir_sharer_S4),
    .dir_sharer_counter_S4      (dir_sharer_counter_S4),
    .cas_cmp_en_S4              (cas_cmp_en_S4),
    .atomic_read_data_en_S4     (atomic_read_data_en_S4),
    .cas_cmp_data_size_S4       (cas_cmp_data_size_S4),
    .msg_send_valid_S4          (msg_send_valid),
    .msg_send_mode_S4           (msg_send_mode),
    .msg_send_type_S4           (msg_send_type),
    .msg_send_type_pre_S4       (msg_send_type_pre),
    .msg_send_length_S4         (msg_send_length),
    .msg_send_data_size_S4      (msg_send_data_size),
    .msg_send_cache_type_S4     (msg_send_cache_type),
    .msg_send_mesi_S4           (msg_send_mesi),
    .msg_send_l2_miss_S4        (msg_send_l2_miss),
    .msg_send_mshrid_S4         (msg_send_mshrid),
    .msg_send_subline_vector_S4 (msg_send_subline_vector),
    .special_addr_type_S4       (special_addr_type_S4),
    .dir_data_sel_S4            (dir_data_sel_S4),
    .dir_data_S4                (dir_data_S4),
    .msg_type_S4                (msg_type_S4),
    .data_size_S4               (data_size_S4),
    .cache_type_S4              (cache_type_S4),
    .l2_miss_S4                 (l2_miss_S4),
    
    .smc_miss_S4                (smc_miss_S4),
    
    .mshr_wr_data_en_S4         (mshr_wr_data_en),
    .mshr_wr_state_en_S4        (mshr_wr_state_en),
    .mshr_state_in_S4           (mshr_state_in),
    .mshr_wr_index_in_S4        (mshr_wr_index_in),    
    .mshr_inv_counter_rd_index_in_S4(mshr_inv_counter_rd_index_in),    
    .state_wr_sel_S4            (state_wr_sel_S4),
    .state_wr_en_S4             (state_wr_en),
    
    .broadcast_counter_op_S4    (broadcast_counter_op),
    .broadcast_counter_op_val_S4(broadcast_counter_op_val),
    
    
    
    .smc_rd_diag_en_buf_S4      (smc_rd_diag_en),
    .smc_rd_en_buf_S4           (smc_rd_en),
    

    .l2_access_valid_S4         (l2_access_valid),
    .l2_miss_valid_S4           (l2_miss_valid),
    .reg_rd_en_S4               (reg_rd_en),
    .reg_rd_addr_type_S4        (reg_rd_addr_type)


);


l2_pipe1_dpath dpath(
    .clk                        (clk),
    .rst_n                      (rst_n),
    
    .csm_en                     (csm_en),
    
    .smt_base_addr              (smt_base_addr),
    

    .mshr_addr_S1               (mshr_addr),
    .mshr_mshrid_S1             (mshr_mshrid),
    .mshr_way_S1                (mshr_way),
    .mshr_src_chipid_S1         (mshr_src_chipid),
    .mshr_src_x_S1              (mshr_src_x),
    .mshr_src_y_S1              (mshr_src_y),
    .mshr_src_fbits_S1          (mshr_src_fbits),
    .mshr_sdid_S1               (mshr_sdid),
    .mshr_lsid_S1               (mshr_lsid),
    .mshr_miss_lsid_S1          (mshr_miss_lsid),
    .mshr_recycled_S1           (mshr_recycled),

























 // L2_CAM_MSHR

    .dis_flush_S1               (dis_flush_S1),
    .msg_addr_S1                (msg_addr),
    .msg_mshrid_S1              (msg_mshrid),
    .msg_src_chipid_S1          (msg_src_chipid),
    .msg_src_x_S1               (msg_src_x),
    .msg_src_y_S1               (msg_src_y),
    .msg_src_fbits_S1           (msg_src_fbits),
    .msg_sdid_S1                (msg_sdid),
    .msg_lsid_S1                (msg_lsid),
    .msg_data_S1                (msg_data),
    .valid_S1                   (valid_S1),
    .stall_S1                   (stall_S1),
    .msg_from_mshr_S1           (msg_from_mshr_S1), 


    .state_data_S2              (state_data_out),
    .tag_data_S2                (tag_data_out),
    .msg_data_S2                (msg_data),
    .msg_type_S2                (msg_type_S2),
    .msg_from_mshr_S2           (msg_from_mshr_S2),
    .special_addr_type_S2       (special_addr_type_S2),
    .data_size_S2               (data_size_S2),
    .cache_type_S2              (cache_type_S2),
    .state_owner_en_S2          (state_owner_en_S2),
    .state_owner_op_S2          (state_owner_op_S2), 
    .state_subline_en_S2        (state_subline_en_S2),
    .state_subline_op_S2        (state_subline_op_S2),
    .state_di_en_S2             (state_di_en_S2),
    .state_vd_en_S2             (state_vd_en_S2),
    .state_vd_S2                (state_vd_S2),
    .state_mesi_en_S2           (state_mesi_en_S2),
    .state_mesi_S2              (state_mesi_S2),
    .state_lru_en_S2            (state_lru_en_S2),
    .state_lru_op_S2            (state_lru_op_S2),
    .state_rb_en_S2             (state_rb_en_S2),
    .state_load_sdid_S2         (state_load_sdid_S2),
    .dir_op_S2                  (dir_op_S2),
    .l2_ifill_S2                (l2_ifill_S2),
    .l2_load_data_subline_S2    (l2_load_data_subline_S2),
    .valid_S2                   (valid_S2),
    .stall_S2                   (stall_S2),
    .stall_before_S2            (stall_before_S2), 
    .data_clk_en_S2             (data_clk_en),
    .stall_real_S2              (stall_real_S2),
    .amo_alu_op_S2              (amo_alu_op_S2),

    .valid_S3                   (valid_S3),
    .stall_S3                   (stall_S3),
    .stall_before_S3            (stall_before_S3), 
    .data_data_S3               (data_data_out),

    .valid_S4                   (valid_S4),
    .stall_S4                   (stall_S4),
    .stall_before_S4            (stall_before_S4),
     
    .stall_smc_buf_S4           (stall_smc_buf_S4),
    
    .msg_from_mshr_S4           (msg_from_mshr_S4),
    .req_recycle_S4             (req_recycle_S4),
    .inv_fwd_pending_S4         (inv_fwd_pending_S4),
    .cas_cmp_en_S4              (cas_cmp_en_S4),    
    .atomic_read_data_en_S4     (atomic_read_data_en_S4),
    .cas_cmp_data_size_S4       (cas_cmp_data_size_S4),
    .dir_sharer_S4              (dir_sharer_S4),
    .dir_sharer_counter_S4      (dir_sharer_counter_S4),
    .mshr_inv_counter_out_S4    (mshr_inv_counter_out),
    .special_addr_type_S4       (special_addr_type_S4),
    .dir_data_sel_S4            (dir_data_sel_S4),
    .dir_data_S4                (dir_data_S4),
    .msg_send_type_S4           (msg_send_type),
    .my_nodeid_chipid_S4        (my_nodeid[33:20]),
    .my_nodeid_x_S4             (my_nodeid[19:12]),
    .my_nodeid_y_S4             (my_nodeid[11:4]),
    .state_wr_sel_S4            (state_wr_sel_S4),
    .msg_type_S4                (msg_type_S4),
    .msg_send_type_pre_S4       (msg_send_type_pre),
    .data_size_S4               (data_size_S4),
    .cache_type_S4              (cache_type_S4),
    .l2_miss_S4                 (l2_miss_S4),

    
    .smc_miss_S4                (smc_miss_S4),
    .smc_data_out_S4            (smc_data_out),
    .smc_valid_out_S4           (smc_valid_out),
    .smc_tag_out_S4             (smc_tag_out),
    
    .reg_data_out_S4            (reg_data_out),
    
    .broadcast_chipid_out_S4    (broadcast_chipid_out),
    .broadcast_x_out_S4         (broadcast_x_out),
    .broadcast_y_out_S4         (broadcast_y_out),
    
 
    .addr_S1                    (addr_S1),
    .mshr_addr_in_S1            (mshr_addr_in),
    .tag_addr_S1                (tag_addr),
    .tag_data_in_S1             (tag_data_in),  
    .tag_data_mask_in_S1        (tag_data_mask_in),
    .state_rd_addr_S1           (state_rd_addr),
    .reg_data_in_S1             (reg_data_in),

    .addr_S2                    (addr_S2),
    .l2_tag_hit_S2              (l2_tag_hit_S2),
    .l2_evict_S2                (l2_evict_S2),
    .l2_wb_S2                   (l2_wb_S2),
    .l2_way_state_mesi_S2       (l2_way_state_mesi_S2),
    .l2_way_state_vd_S2         (l2_way_state_vd_S2),    
    .l2_way_state_cache_type_S2 (l2_way_state_cache_type_S2),
    .l2_way_state_subline_S2    (l2_way_state_subline_S2),
    .req_from_owner_S2          (req_from_owner_S2),
    .addr_l2_aligned_S2         (addr_l2_aligned_S2),
    .lsid_S2                    (lsid_S2),
    .dir_addr_S2                (dir_addr),
    .dir_data_in_S2             (dir_data_in),
    .dir_data_mask_in_S2        (dir_data_mask_in),
    .data_addr_S2               (data_addr),
    .data_data_in_S2            (data_data_in),
    .data_data_mask_in_S2       (data_data_mask_in),

    
    .smc_wr_addr_in_S2          (smc_wr_addr_in),
    .smc_data_in_S2             (smc_data_in),
    

    .addr_S3                    (addr_S3),

    .addr_S4                    (addr_S4),
    .data_addr_S4               (data_ecc_addr),
    .l2_evict_S4                (l2_evict_S4),
    .l2_tag_hit_S4              (l2_tag_hit_S4),
    .l2_way_state_mesi_S4       (l2_way_state_mesi_S4),
    .l2_way_state_owner_S4      (l2_way_state_owner_S4),
    .l2_way_state_vd_S4         (l2_way_state_vd_S4),
    .l2_way_state_subline_S4    (l2_way_state_subline_S4),
    .l2_way_state_cache_type_S4 (l2_way_state_cache_type_S4),
    .mshrid_S4                  (mshrid_S4),
    .req_from_owner_S4          (req_from_owner_S4),
    .mshr_miss_lsid_S4          (mshr_miss_lsid_S4),
    .lsid_S4                    (lsid_S4),
    .corr_error_S4              (data_ecc_corr_error),
    .uncorr_error_S4            (data_ecc_uncorr_error),
    .cas_cmp_S4                 (cas_cmp_S4),
    .msg_send_addr_S4           (msg_send_addr),
    .msg_send_dst_chipid_S4     (msg_send_dst_chipid),
    .msg_send_dst_x_S4          (msg_send_dst_x),
    .msg_send_dst_y_S4          (msg_send_dst_y),
    .msg_send_dst_fbits_S4      (msg_send_dst_fbits),
    .msg_send_data_S4           (msg_send_data),
    .mshr_data_in_S4            (mshr_data_in),
    .mshr_data_mask_in_S4       (mshr_data_mask_in),

    
    .smc_rd_addr_in_buf_S4      (smc_rd_addr_in),
    

    .state_wr_addr_S4           (state_wr_addr),
    .state_data_in_S4           (state_data_in),
    .state_data_mask_in_S4      (state_data_mask_in)

);

l2_encoder encoder(
    .msg_dst_chipid             (msg_send_dst_chipid),
    .msg_dst_x                  (msg_send_dst_x),
    .msg_dst_y                  (msg_send_dst_y),
    .msg_dst_fbits              (msg_send_dst_fbits),
    .msg_length                 (msg_send_length),
    .msg_type                   (msg_send_type),
    .msg_mshrid                 (msg_send_mshrid),
    .msg_data_size              (msg_send_data_size),
    .msg_cache_type             (msg_send_cache_type),
    .msg_subline_vector         (msg_send_subline_vector),
    .msg_mesi                   (msg_send_mesi),
    .msg_l2_miss                (msg_send_l2_miss),
    .msg_subline_id             ({2{1'b0}}),
    .msg_last_subline           ({1{1'b0}}),
    .msg_addr                   (msg_send_addr),
    .msg_src_chipid             (my_nodeid[33:20]),
    .msg_src_x                  (my_nodeid[19:12]),
    .msg_src_y                  (my_nodeid[11:4]),
    .msg_src_fbits              (my_nodeid[3:0]),
    .msg_sdid                   ({10{1'b0}}),
    .msg_lsid                   ({6{1'b0}}),
    .msg_header                 (msg_send_header)
);



l2_pipe1_buf_out buf_out(
    .clk                (clk),
    .rst_n              (rst_n),
    .mode_in            (msg_send_mode),
    .valid_in           (msg_send_valid),
    .data_in            ({msg_send_data, msg_send_header}),
    .ready_in           (msg_send_ready),
    .valid_out          (noc_valid_out),
    .data_out           (noc_data_out),
    .ready_out          (noc_ready_out)
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
//  Filename      : l2_pipe2.v
//  Created On    : 2014-04-03
//  Revision      :
//  Author        : Yaosheng Fu
//  Company       : Princeton University
//  Email         : yfu@princeton.edu
//
//  Description   : Pipeline 2 for the L2 cache
//
//
//====================================================================================================


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
//  Filename      : l2.h.pyv
//  Created On    : 2014-02-20
//  Revision      :
//  Author        : Yaosheng Fu
//  Company       : Princeton University
//  Email         : yfu@princeton.edu
//
//  Description   : header file for the L2 cache
//
//
//==================================================================================================

// devices.xml



// Input buffer for pipeline1













// Input buffer for pipeline2














// Output buffer















// L2 cache configuration







//`define L2_SIZE                  65536
//`define L2_SIZE_WIDTH            16






//`define L2_LINE_SIZE             64
//`define L2_LINE_SIZE_WIDTH       6













//`define L2_WAYS                  4
//`define L2_WAYS_WIDTH            2
//`define L2_WAY_0                 2'b00
//`define L2_WAY_1                 2'b01
//`define L2_WAY_2                 2'b10
//`define L2_WAY_3                 2'b11





// Tag array







//`define L2_TAG_INDEX_WIDTH      8
//`define L2_TAG_WIDTH            26
//`define L2_TAG_WAY_WIDTH        26
//`define L2_TAG_ARRAY_WIDTH      104


// Tag Address decomposition 




//`define L2_TAG_INDEX            13:6
//`define L2_TAG                  39:14


//Data array












//`define L2_DATA_INDEX_WIDTH         12 








// Data Address decomposition 







//Dir array



//`define L2_DIR_INDEX_WIDTH      10 





//State array

















//Whether the cache line is in Icaches or Dcaches
















//Round Robin selection














// State decomposition 












//MSHR array



































//`define L2_MSHR_CMP_ADDR        13:6
//`define L2_MSHR_ADDR            39:0
//`define L2_MSHR_WAY             41:40
//`define L2_MSHR_MSHRID          49:42
//`define L2_MSHR_CACHE_TYPE      50
//`define L2_MSHR_DATA_SIZE       53:51
//`define L2_MSHR_MSG_TYPE        61:54
//`define L2_MSHR_L2_MISS         62
//`define L2_MSHR_SRC_CHIPID      76:63
//`define L2_MSHR_SRC_X           84:77
//`define L2_MSHR_SRC_Y           92:85
//`define L2_MSHR_SRC_FBITS       96:93
//`define L2_MSHR_SDID            106:97
//`define L2_MSHR_LSID            112:107      
//`define L2_MSHR_MISS_LSID       118:113
//`define L2_MSHR_SMC_MISS        119
//`define L2_MSHR_RECYCLED        120
//`define L2_MSHR_INV_FWD_PENDING 121


//SMC array


































//Message destination_type







//L2 public sharer beyond the maximum clump size



//L2 registers



//Special addresses





















//L2 core ID





//L2 dir array reuse







//Control Signal 








//AMO ALU OP macros












//Control Signal in Stage 1























































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





































































































































































































































































































































































































































































































































































































module l2_pipe2(

    input wire clk,
    input wire rst_n,
    
    input wire csm_en,
    
    //inputs from NOC3
   
    input wire noc_valid_in,
    input wire [64-1:0] noc_data_in,
    output wire noc_ready_in,

    input wire [2-1:0] mshr_state_out,
    input wire [120+2-1:0] mshr_data_out,

    
    input wire broadcast_counter_zero,
    input wire broadcast_counter_max,
    input wire [14-1:0] broadcast_chipid_out,
    input wire [8-1:0] broadcast_x_out,
    input wire [8-1:0] broadcast_y_out,
    

    input wire [15*4+2+4-1:0] state_data_out,
    
    input wire [104-1:0] tag_data_out,

    input wire [64-1:0] dir_data_out,


    output wire mshr_rd_en,
    output wire mshr_wr_state_en,
    output wire mshr_wr_data_en,
    output wire [2-1:0] mshr_state_in,
    output wire [120+2-1:0] mshr_data_in,
    output wire [120+2-1:0] mshr_data_mask_in,
    output wire [3-1:0] mshr_rd_index_in,
    output wire [3-1:0] mshr_wr_index_in,
    output wire mshr_inc_counter_en,

    output wire state_rd_en,
    output wire state_wr_en,
    output wire [8-1:0] state_rd_addr,
    output wire [8-1:0] state_wr_addr,
    output wire [15*4+2+4-1:0] state_data_in,
    output wire [15*4+2+4-1:0] state_data_mask_in,


    output wire tag_clk_en,
    output wire tag_rdw_en,
    output wire [8-1:0] tag_addr,
    output wire [104-1:0] tag_data_in,
    output wire [104-1:0] tag_data_mask_in,

    output wire dir_clk_en,
    output wire dir_rdw_en,
    output wire [8+2-1:0] dir_addr,
    output wire [64-1:0] dir_data_in,
    output wire [64-1:0] dir_data_mask_in,

    output wire data_clk_en,
    output wire data_rdw_en,
    output wire [8+2+2-1:0] data_addr,
    output wire [144-1:0] data_data_in,
    output wire [144-1:0] data_data_mask_in,

    
    output wire [2-1:0] broadcast_counter_op,
    output wire broadcast_counter_op_val,

    output wire smc_wr_en,
    output wire [16-1:0] smc_wr_addr_in,
    output wire [128-1:0] smc_data_in,
    

    output wire valid_S1,
    output wire valid_S2,
    output wire valid_S3,

    output wire [8-1:0] msg_type_S1,
    output wire [8-1:0] msg_type_S2,
    output wire [8-1:0] msg_type_S3,

    output wire [40-1:0] addr_S1,
    output wire [40-1:0] addr_S2,
    output wire [40-1:0] addr_S3,

    output wire active_S1,
    output wire active_S2,
    output wire active_S3
);


wire [8-1:0] msg_type;
wire [8-1:0] msg_length;
wire [8-1:0] msg_mshrid;
wire [3-1:0] msg_data_size;
wire [1-1:0] msg_cache_type;
wire [2-1:0] msg_subline_id;
wire [1-1:0] msg_last_subline;
wire [2-1:0] msg_mesi;
wire [40-1:0] msg_addr;
wire [14-1:0] msg_src_chipid;
wire [8-1:0] msg_src_x;
wire [8-1:0] msg_src_y;
wire [4-1:0] msg_src_fbits;
wire [10-1:0] msg_sdid;
wire [6-1:0] msg_lsid;


wire [8-1:0] mshr_msg_type;
wire [8-1:0] mshr_mshrid;
wire [3-1:0] mshr_data_size;
wire [1-1:0] mshr_cache_type;
wire [40-1:0] mshr_addr;
wire [2-1:0] mshr_way;
wire [1-1:0] mshr_l2_miss;
wire [14-1:0] mshr_src_chipid;
wire [8-1:0] mshr_src_x;
wire [8-1:0] mshr_src_y;
wire [4-1:0] mshr_src_fbits;
wire [10-1:0] mshr_sdid;
wire [6-1:0] mshr_lsid;
wire [6-1:0] mshr_miss_lsid;

wire mshr_smc_miss;

wire mshr_recycled;
wire mshr_inv_fwd_pending;

wire msg_header_valid;
wire [192-1:0] msg_header;
wire msg_header_ready;

wire msg_data_valid;
wire [128-1:0] msg_data;
wire msg_data_ready;

//wire valid_S1;
wire stall_S1;
wire msg_from_mshr_S1;
//wire [`PHY_ADDR_WIDTH-1:0] addr_S1;
wire is_same_address_S1;


//wire valid_S2;
wire stall_S2;
wire stall_before_S2;
wire msg_from_mshr_S2;
wire [3-1:0] data_size_S2;
wire [1-1:0] cache_type_S2;
wire state_owner_en_S2;
wire [2-1:0] state_owner_op_S2;
wire state_subline_en_S2;
wire [2-1:0] state_subline_op_S2;
wire state_di_en_S2;
wire state_vd_en_S2;
wire [2-1:0] state_vd_S2;
wire state_mesi_en_S2;
wire [2-1:0] state_mesi_S2;
wire state_lru_en_S2;
wire [1-1:0] state_lru_op_S2;
wire state_rb_en_S2;
wire dir_clr_en_S2;
wire l2_load_64B_S2;
wire [2-1:0] l2_load_data_subline_S2;
//wire [`PHY_ADDR_WIDTH-1:0] addr_S2;
wire l2_tag_hit_S2;
wire [2-1:0] l2_way_sel_S2;
wire l2_evict_S2;
wire l2_wb_S2;
wire [6-1:0] l2_way_state_owner_S2;
wire [2-1:0] l2_way_state_mesi_S2;
wire [2-1:0] l2_way_state_vd_S2;
wire [4-1:0] l2_way_state_subline_S2;
wire [1-1:0] l2_way_state_cache_type_S2;
wire addr_l2_aligned_S2;
wire subline_valid_S2;
wire [6-1:0] lsid_S2;

//wire valid_S3;
wire stall_S3;
//wire [`PHY_ADDR_WIDTH-1:0] addr_S3;

assign msg_type_S1 = msg_type;


l2_pipe2_buf_in buf_in(
    .clk                    (clk),
    .rst_n                  (rst_n),

    .valid_in               (noc_valid_in),
    .data_in                (noc_data_in),
    .ready_in               (noc_ready_in),

    .msg_header_valid_out   (msg_header_valid),
    .msg_header_out         (msg_header),
    .msg_header_ready_out   (msg_header_ready),

    .msg_data_valid_out     (msg_data_valid),
    .msg_data_out           (msg_data),
    .msg_data_ready_out     (msg_data_ready)
);


l2_decoder decoder(
    .msg_header         (msg_header),
    .msg_type           (msg_type),
    .msg_length         (msg_length),
    .msg_mshrid         (msg_mshrid),
    .msg_data_size      (msg_data_size),
    .msg_cache_type     (msg_cache_type),
    .msg_subline_vector (),
    .msg_mesi           (msg_mesi),
    .msg_l2_miss        (),
    .msg_subline_id     (msg_subline_id),
    .msg_last_subline   (msg_last_subline),
    .msg_addr           (msg_addr),
    .msg_src_chipid     (msg_src_chipid),
    .msg_src_x          (msg_src_x),
    .msg_src_y          (msg_src_y),
    .msg_src_fbits      (msg_src_fbits),
    .msg_sdid           (msg_sdid),
    .msg_lsid           (msg_lsid)
);

l2_mshr_decoder mshr_decoder(

    .data_in            (mshr_data_out),
    .addr_out           (mshr_addr),
    .way_out            (mshr_way),
    .mshrid_out         (mshr_mshrid),
    .cache_type_out     (mshr_cache_type), 
    .data_size_out      (mshr_data_size),
    .msg_type_out       (mshr_msg_type),
    .msg_l2_miss_out    (mshr_l2_miss),
    .src_chipid_out     (mshr_src_chipid),
    .src_x_out          (mshr_src_x),
    .src_y_out          (mshr_src_y),
    .src_fbits_out      (mshr_src_fbits),
    .sdid_out           (mshr_sdid),
    .lsid_out           (mshr_lsid),
    .miss_lsid_out      (mshr_miss_lsid),
    
    .smc_miss_out       (mshr_smc_miss),
    

    
    .recycled           (mshr_recycled),
    .inv_fwd_pending    (mshr_inv_fwd_pending)

);


l2_pipe2_ctrl ctrl(

    .clk                        (clk),
    .rst_n                      (rst_n),
    
    .csm_en                     (csm_en),
    

    .msg_header_valid_S1        (msg_header_valid),
    .msg_type_S1                (msg_type),
    .msg_length_S1              (msg_length),
    .msg_data_size_S1           (msg_data_size),
    .msg_cache_type_S1          (msg_cache_type),
    .msg_last_subline_S1        (msg_last_subline),
    .msg_mesi_S1                (msg_mesi),
    .mshr_msg_type_S1           (mshr_msg_type),
    .mshr_l2_miss_S1            (mshr_l2_miss),
    .mshr_data_size_S1          (mshr_data_size),
    .mshr_cache_type_S1         (mshr_cache_type),
     
    .mshr_smc_miss_S1           (mshr_smc_miss),
    
    .mshr_state_out_S1          (mshr_state_out),
    .mshr_inv_fwd_pending_S1    (mshr_inv_fwd_pending),
    .addr_S1                    (addr_S1),
    .is_same_address_S1         (is_same_address_S1),
   
    .l2_tag_hit_S2              (l2_tag_hit_S2),
    .l2_way_sel_S2              (l2_way_sel_S2),
    .l2_wb_S2                   (l2_wb_S2),
    .l2_way_state_owner_S2      (l2_way_state_owner_S2),
    .l2_way_state_mesi_S2       (l2_way_state_mesi_S2),
    .l2_way_state_vd_S2         (l2_way_state_vd_S2),
    .l2_way_state_subline_S2    (l2_way_state_subline_S2),
    .l2_way_state_cache_type_S2 (l2_way_state_cache_type_S2),
    .addr_l2_aligned_S2         (addr_l2_aligned_S2),
    .subline_valid_S2           (subline_valid_S2),
    .msg_data_valid_S2          (msg_data_valid),
    
    .broadcast_counter_zero_S2  (broadcast_counter_zero),
    .broadcast_counter_max_S2   (broadcast_counter_max),
    .broadcast_chipid_out_S2    (broadcast_chipid_out),
    .broadcast_x_out_S2         (broadcast_x_out),
    .broadcast_y_out_S2         (broadcast_y_out),
    
    .lsid_S2                    (lsid_S2),
    .addr_S2                    (addr_S2),


    .addr_S3                    (addr_S3),

    .valid_S1                   (valid_S1),  
    .stall_S1                   (stall_S1), 
    .active_S1                  (active_S1),   
    .msg_from_mshr_S1           (msg_from_mshr_S1), 
    .mshr_rd_en_S1              (mshr_rd_en),
    .msg_header_ready_S1        (msg_header_ready),
    .tag_clk_en_S1              (tag_clk_en),
    .tag_rdw_en_S1              (tag_rdw_en),
    .state_rd_en_S1             (state_rd_en),

    .valid_S2                   (valid_S2),    
    .stall_S2                   (stall_S2), 
    .stall_before_S2            (stall_before_S2), 
    .active_S2                  (active_S2), 
    .msg_from_mshr_S2           (msg_from_mshr_S2),
    .msg_type_S2                (msg_type_S2),
    .data_size_S2               (data_size_S2),
    .cache_type_S2              (cache_type_S2),
    .dir_clk_en_S2              (dir_clk_en),
    .dir_rdw_en_S2              (dir_rdw_en),
    .dir_clr_en_S2              (dir_clr_en_S2),
    .data_clk_en_S2             (data_clk_en),
    .data_rdw_en_S2             (data_rdw_en),
    .state_owner_en_S2          (state_owner_en_S2),
    .state_owner_op_S2          (state_owner_op_S2),
    .state_subline_en_S2        (state_subline_en_S2),
    .state_subline_op_S2        (state_subline_op_S2),   
    .state_di_en_S2             (state_di_en_S2),
    .state_vd_en_S2             (state_vd_en_S2),
    .state_vd_S2                (state_vd_S2),
    .state_mesi_en_S2           (state_mesi_en_S2),
    .state_mesi_S2              (state_mesi_S2),
    .state_lru_en_S2            (state_lru_en_S2),
    .state_lru_op_S2            (state_lru_op_S2),
    .state_rb_en_S2             (state_rb_en_S2),
    .l2_load_64B_S2             (l2_load_64B_S2),
    .l2_load_data_subline_S2    (l2_load_data_subline_S2),
    .msg_data_ready_S2          (msg_data_ready),
    
    .smc_wr_en_S2               (smc_wr_en),
    .broadcast_counter_op_S2    (broadcast_counter_op),
    .broadcast_counter_op_val_S2(broadcast_counter_op_val),
    

    .valid_S3                   (valid_S3),    
    .stall_S3                   (stall_S3), 
    .active_S3                  (active_S3),   
    .msg_type_S3                (msg_type_S3),
    .mshr_wr_state_en_S3        (mshr_wr_state_en),
    .mshr_wr_data_en_S3         (mshr_wr_data_en),
    .mshr_state_in_S3           (mshr_state_in),
    .mshr_inc_counter_en_S3     (mshr_inc_counter_en),
    .state_wr_en_S3             (state_wr_en)
);


l2_pipe2_dpath dpath(
    .clk                        (clk),
    .rst_n                      (rst_n),
    .mshr_addr_S1               (mshr_addr),
    .mshr_mshrid_S1             (mshr_mshrid),
    .mshr_way_S1                (mshr_way),
    .mshr_src_chipid_S1         (mshr_src_chipid),
    .mshr_src_x_S1              (mshr_src_x),
    .mshr_src_y_S1              (mshr_src_y),
    .mshr_src_fbits_S1          (mshr_src_fbits),
    .mshr_sdid_S1               (mshr_sdid),
    .mshr_lsid_S1               (mshr_lsid),
    .mshr_miss_lsid_S1          (mshr_miss_lsid),
    .msg_addr_S1                (msg_addr),
    .msg_type_S1                (msg_type),
    .msg_subline_id_S1          (msg_subline_id),
    .msg_mshrid_S1              (msg_mshrid),
    .msg_src_chipid_S1          (msg_src_chipid),
    .msg_src_x_S1               (msg_src_x),
    .msg_src_y_S1               (msg_src_y),
    .msg_src_fbits_S1           (msg_src_fbits),
    .msg_sdid_S1                (msg_sdid),
    .msg_lsid_S1                (msg_lsid),
    .valid_S1                   (valid_S1),
    .stall_S1                   (stall_S1),
    .msg_from_mshr_S1           (msg_from_mshr_S1), 

    .state_data_S2              (state_data_out),
    .tag_data_S2                (tag_data_out),
    .msg_data_S2                (msg_data),
    .msg_from_mshr_S2           (msg_from_mshr_S2),
    .msg_type_S2                (msg_type_S2),
    .data_size_S2               (data_size_S2),
    .cache_type_S2              (cache_type_S2),
    .state_owner_en_S2          (state_owner_en_S2),
    .state_owner_op_S2          (state_owner_op_S2), 
    .state_subline_en_S2        (state_subline_en_S2),
    .state_subline_op_S2        (state_subline_op_S2),
    .state_di_en_S2             (state_di_en_S2),
    .state_vd_en_S2             (state_vd_en_S2),
    .state_vd_S2                (state_vd_S2),
    .state_mesi_en_S2           (state_mesi_en_S2),
    .state_mesi_S2              (state_mesi_S2),
    .state_lru_en_S2            (state_lru_en_S2),
    .state_lru_op_S2            (state_lru_op_S2),
    .state_rb_en_S2             (state_rb_en_S2),
    .dir_clr_en_S2              (dir_clr_en_S2),
    .l2_load_64B_S2             (l2_load_64B_S2),
    .l2_load_data_subline_S2    (l2_load_data_subline_S2),
    .valid_S2                   (valid_S2),
    .stall_S2                   (stall_S2),
    .stall_before_S2            (stall_before_S2), 


    .valid_S3                   (valid_S3),
    .stall_S3                   (stall_S3),

    .addr_S1                    (addr_S1),
    .mshr_rd_index_S1           (mshr_rd_index_in),
    .tag_addr_S1                (tag_addr),
    .state_rd_addr_S1           (state_rd_addr),
    .tag_data_in_S1             (tag_data_in),  
    .tag_data_mask_in_S1        (tag_data_mask_in),
    .is_same_address_S1         (is_same_address_S1),

    .addr_S2                    (addr_S2),
    .l2_tag_hit_S2              (l2_tag_hit_S2),
    .l2_way_sel_S2              (l2_way_sel_S2),
    .l2_wb_S2                   (l2_wb_S2),
    .l2_way_state_owner_S2      (l2_way_state_owner_S2),
    .l2_way_state_mesi_S2       (l2_way_state_mesi_S2),
    .l2_way_state_vd_S2         (l2_way_state_vd_S2),    
    .l2_way_state_subline_S2    (l2_way_state_subline_S2),
    .l2_way_state_cache_type_S2 (l2_way_state_cache_type_S2),
    .addr_l2_aligned_S2         (addr_l2_aligned_S2),
    .subline_valid_S2           (subline_valid_S2),
    .lsid_S2                    (lsid_S2),
    .dir_addr_S2                (dir_addr),
    .dir_data_in_S2             (dir_data_in),
    .dir_data_mask_in_S2        (dir_data_mask_in),
    .data_addr_S2               (data_addr),
    .data_data_in_S2            (data_data_in),
    .data_data_mask_in_S2       (data_data_mask_in),
    
    .smc_wr_addr_in_S2          (smc_wr_addr_in),
    .smc_data_in_S2             (smc_data_in),
    

    .addr_S3                    (addr_S3),
    .mshr_wr_index_S3           (mshr_wr_index_in),
    .mshr_data_in_S3            (mshr_data_in),
    .mshr_data_mask_in_S3       (mshr_data_mask_in),
    .state_wr_addr_S3           (state_wr_addr),
    .state_data_in_S3           (state_data_in),
    .state_data_mask_in_S3      (state_data_mask_in)
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
//  Filename      : l2_smc_wrap.v
//  Created On    : 2014-06-19
//  Revision      :
//  Author        : Yaosheng Fu
//  Company       : Princeton University
//  Email         : yfu@princeton.edu
//
//  Description   : The wrap module for smc in the L2 cache
//
//
//==================================================================================================

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
//  Filename      : l2.h.pyv
//  Created On    : 2014-02-20
//  Revision      :
//  Author        : Yaosheng Fu
//  Company       : Princeton University
//  Email         : yfu@princeton.edu
//
//  Description   : header file for the L2 cache
//
//
//==================================================================================================

// devices.xml



// Input buffer for pipeline1













// Input buffer for pipeline2














// Output buffer















// L2 cache configuration







//`define L2_SIZE                  65536
//`define L2_SIZE_WIDTH            16






//`define L2_LINE_SIZE             64
//`define L2_LINE_SIZE_WIDTH       6













//`define L2_WAYS                  4
//`define L2_WAYS_WIDTH            2
//`define L2_WAY_0                 2'b00
//`define L2_WAY_1                 2'b01
//`define L2_WAY_2                 2'b10
//`define L2_WAY_3                 2'b11





// Tag array







//`define L2_TAG_INDEX_WIDTH      8
//`define L2_TAG_WIDTH            26
//`define L2_TAG_WAY_WIDTH        26
//`define L2_TAG_ARRAY_WIDTH      104


// Tag Address decomposition 




//`define L2_TAG_INDEX            13:6
//`define L2_TAG                  39:14


//Data array












//`define L2_DATA_INDEX_WIDTH         12 








// Data Address decomposition 







//Dir array



//`define L2_DIR_INDEX_WIDTH      10 





//State array

















//Whether the cache line is in Icaches or Dcaches
















//Round Robin selection














// State decomposition 












//MSHR array



































//`define L2_MSHR_CMP_ADDR        13:6
//`define L2_MSHR_ADDR            39:0
//`define L2_MSHR_WAY             41:40
//`define L2_MSHR_MSHRID          49:42
//`define L2_MSHR_CACHE_TYPE      50
//`define L2_MSHR_DATA_SIZE       53:51
//`define L2_MSHR_MSG_TYPE        61:54
//`define L2_MSHR_L2_MISS         62
//`define L2_MSHR_SRC_CHIPID      76:63
//`define L2_MSHR_SRC_X           84:77
//`define L2_MSHR_SRC_Y           92:85
//`define L2_MSHR_SRC_FBITS       96:93
//`define L2_MSHR_SDID            106:97
//`define L2_MSHR_LSID            112:107      
//`define L2_MSHR_MISS_LSID       118:113
//`define L2_MSHR_SMC_MISS        119
//`define L2_MSHR_RECYCLED        120
//`define L2_MSHR_INV_FWD_PENDING 121


//SMC array


































//Message destination_type







//L2 public sharer beyond the maximum clump size



//L2 registers



//Special addresses





















//L2 core ID





//L2 dir array reuse







//Control Signal 








//AMO ALU OP macros












//Control Signal in Stage 1























































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





































































































































































































































































































































































































































































































































































































module l2_smc_wrap(

    input wire clk,
    input wire rst_n,
    input wire pipe_sel,
    input wire rd_en,
    input wire rd_diag_en,
    input wire flush_en,
    input wire [2-1:0] addr_op,
    input wire [16-1:0] rd_addr_in,

    input wire wr_en1,
    input wire [16-1:0] wr_addr_in1,
    input wire [128-1:0] data_in1,
    input wire wr_diag_en1,

    input wire wr_en2,
    input wire [16-1:0] wr_addr_in2,
    input wire [128-1:0] data_in2,
    input wire wr_diag_en2,

    output wire hit,
    output wire [30-1:0] data_out,
    output wire [4-1:0] valid_out,
    output wire [14-1:0] tag_out
);


reg wr_en;
reg [16-1:0] wr_addr_in;
reg [128-1:0] data_in;
reg wr_diag_en;

always @ *
begin
    if (pipe_sel)
    begin
        wr_en = wr_en2;
        wr_addr_in = wr_addr_in2;
        data_in = data_in2;
        wr_diag_en = wr_diag_en2;
    end
    else
    begin
        wr_en = wr_en1;
        wr_addr_in = wr_addr_in1;
        data_in = data_in1;
        wr_diag_en = wr_diag_en1;
    end
end

l2_smc l2_smc(
    .clk            (clk),
    .rst_n          (rst_n),
    .rd_en          (rd_en),
    .wr_en          (wr_en),
    .rd_diag_en     (rd_diag_en),
    .wr_diag_en     (wr_diag_en),
    .flush_en       (flush_en),
    .addr_op        (addr_op),
    .rd_addr_in     (rd_addr_in),
    .wr_addr_in     (wr_addr_in),
    .data_in        (data_in),
    .hit            (hit),
    .data_out       (data_out),
    .valid_out      (valid_out),
    .tag_out        (tag_out)
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
//  Filename      : l2_state_wrap.v
//  Created On    : 2014-02-24
//  Revision      :
//  Author        : Yaosheng Fu
//  Company       : Princeton University
//  Email         : yfu@princeton.edu
//
//  Description   : The wrap module for state array in the L2 cache
//
//
//==================================================================================================

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
//  Filename      : l2.h.pyv
//  Created On    : 2014-02-20
//  Revision      :
//  Author        : Yaosheng Fu
//  Company       : Princeton University
//  Email         : yfu@princeton.edu
//
//  Description   : header file for the L2 cache
//
//
//==================================================================================================

// devices.xml



// Input buffer for pipeline1













// Input buffer for pipeline2














// Output buffer















// L2 cache configuration







//`define L2_SIZE                  65536
//`define L2_SIZE_WIDTH            16






//`define L2_LINE_SIZE             64
//`define L2_LINE_SIZE_WIDTH       6













//`define L2_WAYS                  4
//`define L2_WAYS_WIDTH            2
//`define L2_WAY_0                 2'b00
//`define L2_WAY_1                 2'b01
//`define L2_WAY_2                 2'b10
//`define L2_WAY_3                 2'b11





// Tag array







//`define L2_TAG_INDEX_WIDTH      8
//`define L2_TAG_WIDTH            26
//`define L2_TAG_WAY_WIDTH        26
//`define L2_TAG_ARRAY_WIDTH      104


// Tag Address decomposition 




//`define L2_TAG_INDEX            13:6
//`define L2_TAG                  39:14


//Data array












//`define L2_DATA_INDEX_WIDTH         12 








// Data Address decomposition 







//Dir array



//`define L2_DIR_INDEX_WIDTH      10 





//State array

















//Whether the cache line is in Icaches or Dcaches
















//Round Robin selection














// State decomposition 












//MSHR array



































//`define L2_MSHR_CMP_ADDR        13:6
//`define L2_MSHR_ADDR            39:0
//`define L2_MSHR_WAY             41:40
//`define L2_MSHR_MSHRID          49:42
//`define L2_MSHR_CACHE_TYPE      50
//`define L2_MSHR_DATA_SIZE       53:51
//`define L2_MSHR_MSG_TYPE        61:54
//`define L2_MSHR_L2_MISS         62
//`define L2_MSHR_SRC_CHIPID      76:63
//`define L2_MSHR_SRC_X           84:77
//`define L2_MSHR_SRC_Y           92:85
//`define L2_MSHR_SRC_FBITS       96:93
//`define L2_MSHR_SDID            106:97
//`define L2_MSHR_LSID            112:107      
//`define L2_MSHR_MISS_LSID       118:113
//`define L2_MSHR_SMC_MISS        119
//`define L2_MSHR_RECYCLED        120
//`define L2_MSHR_INV_FWD_PENDING 121


//SMC array


































//Message destination_type







//L2 public sharer beyond the maximum clump size



//L2 registers



//Special addresses





















//L2 core ID





//L2 dir array reuse







//Control Signal 








//AMO ALU OP macros












//Control Signal in Stage 1























































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





































































































































































































































































































































































































































































































































































































module l2_state_wrap(


    input wire clk,
    input wire rst_n,
    input wire pipe_rd_sel,
    input wire pipe_wr_sel,
    input wire pdout_en,
    input wire deepsleep,

    input wire rd_en1,
    input wire wr_en1,
    input wire [8-1:0] rd_addr1,
    input wire [8-1:0] wr_addr1,
    input wire [15*4+2+4-1:0] data_in1,
    input wire [15*4+2+4-1:0] data_mask_in1,

    input wire rd_en2,
    input wire wr_en2,
    input wire [8-1:0] rd_addr2,
    input wire [8-1:0] wr_addr2,
    input wire [15*4+2+4-1:0] data_in2,
    input wire [15*4+2+4-1:0] data_mask_in2,

    output wire [15*4+2+4-1:0] data_out,
    output wire [15*4+2+4-1:0] pdata_out,

    // sram interface
    output wire [4-1:0] srams_rtap_data,
    input wire  [4-1:0] rtap_srams_bist_command,
    input wire  [4-1:0] rtap_srams_bist_data

);

reg rd_en;
reg wr_en;
reg [8-1:0] rd_addr;
reg [8-1:0] wr_addr;
reg [15*4+2+4-1:0] data_in;
reg [15*4+2+4-1:0] data_mask_in;

always @ *
begin
    if (pipe_rd_sel)
    begin
        rd_en = rd_en2;
        rd_addr = rd_addr2;
    end
    else
    begin
        rd_en = rd_en1;
        rd_addr = rd_addr1;
    end
end

always @ *
begin
    if (pipe_wr_sel)
    begin
        wr_en = wr_en2;
        wr_addr = wr_addr2;
        data_in = data_in2;
        data_mask_in = data_mask_in2;
     end
    else
    begin
        wr_en = wr_en1;
        wr_addr = wr_addr1;
        data_in = data_in1;
        data_mask_in = data_mask_in1;
    end
end


l2_state l2_state(

    .clk            (clk),
    .rst_n          (rst_n),
    .rd_en          (rd_en),
    .wr_en          (wr_en),
    .pdout_en       (pdout_en),
    .deepsleep      (deepsleep),
    .rd_addr        (rd_addr),
    .wr_addr        (wr_addr),
    .data_in        (data_in),
    .data_mask_in   (data_mask_in),
    .data_out       (data_out),
    .pdata_out      (pdata_out),

    // sram interfaces
    .srams_rtap_data (srams_rtap_data),
    .rtap_srams_bist_command (rtap_srams_bist_command),
    .rtap_srams_bist_data (rtap_srams_bist_data)
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
//  Filename      : l2_tag.v
//  Created On    : 2014-02-24
//  Revision      :
//  Author        : Yaosheng Fu
//  Company       : Princeton University
//  Email         : yfu@princeton.edu
//
//  Description   : The tag array in the L2 cache
//
//
//==================================================================================================

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
//  Filename      : l2.h.pyv
//  Created On    : 2014-02-20
//  Revision      :
//  Author        : Yaosheng Fu
//  Company       : Princeton University
//  Email         : yfu@princeton.edu
//
//  Description   : header file for the L2 cache
//
//
//==================================================================================================

// devices.xml



// Input buffer for pipeline1













// Input buffer for pipeline2














// Output buffer















// L2 cache configuration







//`define L2_SIZE                  65536
//`define L2_SIZE_WIDTH            16






//`define L2_LINE_SIZE             64
//`define L2_LINE_SIZE_WIDTH       6













//`define L2_WAYS                  4
//`define L2_WAYS_WIDTH            2
//`define L2_WAY_0                 2'b00
//`define L2_WAY_1                 2'b01
//`define L2_WAY_2                 2'b10
//`define L2_WAY_3                 2'b11





// Tag array







//`define L2_TAG_INDEX_WIDTH      8
//`define L2_TAG_WIDTH            26
//`define L2_TAG_WAY_WIDTH        26
//`define L2_TAG_ARRAY_WIDTH      104


// Tag Address decomposition 




//`define L2_TAG_INDEX            13:6
//`define L2_TAG                  39:14


//Data array












//`define L2_DATA_INDEX_WIDTH         12 








// Data Address decomposition 







//Dir array



//`define L2_DIR_INDEX_WIDTH      10 





//State array

















//Whether the cache line is in Icaches or Dcaches
















//Round Robin selection














// State decomposition 












//MSHR array



































//`define L2_MSHR_CMP_ADDR        13:6
//`define L2_MSHR_ADDR            39:0
//`define L2_MSHR_WAY             41:40
//`define L2_MSHR_MSHRID          49:42
//`define L2_MSHR_CACHE_TYPE      50
//`define L2_MSHR_DATA_SIZE       53:51
//`define L2_MSHR_MSG_TYPE        61:54
//`define L2_MSHR_L2_MISS         62
//`define L2_MSHR_SRC_CHIPID      76:63
//`define L2_MSHR_SRC_X           84:77
//`define L2_MSHR_SRC_Y           92:85
//`define L2_MSHR_SRC_FBITS       96:93
//`define L2_MSHR_SDID            106:97
//`define L2_MSHR_LSID            112:107      
//`define L2_MSHR_MISS_LSID       118:113
//`define L2_MSHR_SMC_MISS        119
//`define L2_MSHR_RECYCLED        120
//`define L2_MSHR_INV_FWD_PENDING 121


//SMC array


































//Message destination_type







//L2 public sharer beyond the maximum clump size



//L2 registers



//Special addresses





















//L2 core ID





//L2 dir array reuse







//Control Signal 








//AMO ALU OP macros












//Control Signal in Stage 1























































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





































































































































































































































































































































































































































































































































































































module l2_tag(

    input wire clk,
    input wire rst_n,
    input wire clk_en,

    //0 for write, 1 for read
    input wire rdw_en,
    input wire pdout_en,
    input wire deepsleep,

    input wire [8-1:0] addr,
    input wire [104-1:0] data_in,
    input wire [104-1:0] data_mask_in,

    output wire [104-1:0] data_out,
    output wire [104-1:0] pdata_out,

    // sram interface
    output wire [4-1:0] srams_rtap_data,
    input wire  [4-1:0] rtap_srams_bist_command,
    input wire  [4-1:0] rtap_srams_bist_data

);

/*
localparam reset = 2'd0;
localparam init  = 2'd1;
localparam done  = 2'd2;

reg [1:0] init_state_f;
reg [1:0] init_state_next;
reg [`L2_TAG_INDEX_WIDTH-1:0] init_counter_f;
reg [`L2_TAG_INDEX_WIDTH-1:0] init_counter_next;

reg [`L2_TAG_INDEX_WIDTH-1:0] addr_real;
reg rdw_en_real;
reg clk_en_real;
reg [`L2_TAG_ARRAY_WIDTH-1:0] data_in_real;
reg [`L2_TAG_ARRAY_WIDTH-1:0] data_mask_in_real;

always @ *
begin
    if (!rst_n)
    begin
        init_state_next = reset;
    end
    else
    begin
        if (init_state_f == reset)
        begin
            init_state_next = init;
        end
        else if ((init_state_f == init) && (init_counter_f == {`L2_TAG_INDEX_WIDTH{1'b1}}))
        begin
            init_state_next = done;
        end
        else
        begin
            init_state_next = init_state_f;
        end
    end
end

always @ (posedge clk)
begin
    init_state_f <= init_state_next;
end

always @ *
begin
    if ((init_state_f == reset) || (init_state_f == done))
    begin
        init_counter_next = {`L2_TAG_INDEX_WIDTH{1'b0}};
    end
    else
    begin
        init_counter_next = init_counter_f + 1;
    end
end


always @ (posedge clk)
begin
    init_counter_f <= init_counter_next;
end


always @ *
begin
    init_done = (init_state_f == done);
end


always @ *
begin
    init_done = (init_state_f == done);
end


always @ *
begin
    if (init_state_f == init)
    begin
        clk_en_real = 1'b1;
        rdw_en_real = 1'b0;
        addr_real = init_counter_f;
        data_in_real = {`L2_TAG_ARRAY_WIDTH{1'b0}};
        data_mask_in_real = {`L2_TAG_ARRAY_WIDTH{1'b1}};
    end
    else
    begin
        clk_en_real = clk_en;
        rdw_en_real = rdw_en;
        addr_real = addr;
        data_in_real = data_in;
        data_mask_in_real = data_mask_in;
    end
end
*/


// sram_1rw_256x104 l2_tag_array(
sram_l2_tag l2_tag_array(
    .MEMCLK     (clk),
    .RESET_N(rst_n),
    .CE         (clk_en),

    .A          (addr),
    .DIN        (data_in),
    .RDWEN      (rdw_en),
    .BW         (data_mask_in),
    .DOUT       (data_out),
    .BIST_COMMAND(rtap_srams_bist_command),
    .BIST_DIN(rtap_srams_bist_data),
    .BIST_DOUT(srams_rtap_data),
    .SRAMID(8'd16)
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
//  Filename      : l2_tag_wrap.v
//  Created On    : 2014-02-24
//  Revision      :
//  Author        : Yaosheng Fu
//  Company       : Princeton University
//  Email         : yfu@princeton.edu
//
//  Description   : The wrap module for tag array in the L2 cache
//
//
//==================================================================================================

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
//  Filename      : l2.h.pyv
//  Created On    : 2014-02-20
//  Revision      :
//  Author        : Yaosheng Fu
//  Company       : Princeton University
//  Email         : yfu@princeton.edu
//
//  Description   : header file for the L2 cache
//
//
//==================================================================================================

// devices.xml



// Input buffer for pipeline1













// Input buffer for pipeline2














// Output buffer















// L2 cache configuration







//`define L2_SIZE                  65536
//`define L2_SIZE_WIDTH            16






//`define L2_LINE_SIZE             64
//`define L2_LINE_SIZE_WIDTH       6













//`define L2_WAYS                  4
//`define L2_WAYS_WIDTH            2
//`define L2_WAY_0                 2'b00
//`define L2_WAY_1                 2'b01
//`define L2_WAY_2                 2'b10
//`define L2_WAY_3                 2'b11





// Tag array







//`define L2_TAG_INDEX_WIDTH      8
//`define L2_TAG_WIDTH            26
//`define L2_TAG_WAY_WIDTH        26
//`define L2_TAG_ARRAY_WIDTH      104


// Tag Address decomposition 




//`define L2_TAG_INDEX            13:6
//`define L2_TAG                  39:14


//Data array












//`define L2_DATA_INDEX_WIDTH         12 








// Data Address decomposition 







//Dir array



//`define L2_DIR_INDEX_WIDTH      10 





//State array

















//Whether the cache line is in Icaches or Dcaches
















//Round Robin selection














// State decomposition 












//MSHR array



































//`define L2_MSHR_CMP_ADDR        13:6
//`define L2_MSHR_ADDR            39:0
//`define L2_MSHR_WAY             41:40
//`define L2_MSHR_MSHRID          49:42
//`define L2_MSHR_CACHE_TYPE      50
//`define L2_MSHR_DATA_SIZE       53:51
//`define L2_MSHR_MSG_TYPE        61:54
//`define L2_MSHR_L2_MISS         62
//`define L2_MSHR_SRC_CHIPID      76:63
//`define L2_MSHR_SRC_X           84:77
//`define L2_MSHR_SRC_Y           92:85
//`define L2_MSHR_SRC_FBITS       96:93
//`define L2_MSHR_SDID            106:97
//`define L2_MSHR_LSID            112:107      
//`define L2_MSHR_MISS_LSID       118:113
//`define L2_MSHR_SMC_MISS        119
//`define L2_MSHR_RECYCLED        120
//`define L2_MSHR_INV_FWD_PENDING 121


//SMC array


































//Message destination_type







//L2 public sharer beyond the maximum clump size



//L2 registers



//Special addresses





















//L2 core ID





//L2 dir array reuse







//Control Signal 








//AMO ALU OP macros












//Control Signal in Stage 1























































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





































































































































































































































































































































































































































































































































































































module l2_tag_wrap(

    input wire clk,
    input wire rst_n,
    input wire clk_en1,
    input wire clk_en2,
    input wire rdw_en1,
    input wire rdw_en2,
    input wire pdout_en,
    input wire deepsleep,
    input wire pipe_sel,

    input wire [8-1:0] addr1,
    input wire [104-1:0] data_in1,
    input wire [104-1:0] data_mask_in1,


    input wire [8-1:0] addr2,
    input wire [104-1:0] data_in2,
    input wire [104-1:0] data_mask_in2,

    output wire [104-1:0] data_out,
    output wire [104-1:0] pdata_out,

    // sram interface
    output wire [4-1:0] srams_rtap_data,
    input wire  [4-1:0] rtap_srams_bist_command,
    input wire  [4-1:0] rtap_srams_bist_data

);

reg clk_en;
reg rdw_en;
reg [8-1:0] addr;
reg [104-1:0] data_in;
reg [104-1:0] data_mask_in;

always @ *
begin
    if (pipe_sel)
    begin
        clk_en = clk_en2;
        rdw_en = rdw_en2;
        addr = addr2;
        data_in = data_in2;
        data_mask_in = data_mask_in2;
    end
    else
    begin
        clk_en = clk_en1;
        rdw_en = rdw_en1;
        addr = addr1;
        data_in = data_in1;
        data_mask_in = data_mask_in1;
    end
end

l2_tag l2_tag(

    .clk            (clk),
    .rst_n          (rst_n),
    .clk_en         (clk_en),
    .rdw_en         (rdw_en),
    .pdout_en       (pdout_en),
    .deepsleep      (deepsleep),
    .addr           (addr),
    .data_in        (data_in),
    .data_mask_in   (data_mask_in),
    .data_out       (data_out),
    .pdata_out      (pdata_out),

    // sram interfaces
    .srams_rtap_data (srams_rtap_data),
    .rtap_srams_bist_command (rtap_srams_bist_command),
    .rtap_srams_bist_data (rtap_srams_bist_data)
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
//  Filename      : l2_data_ecc.v
//  Created On    : 2014-06-02
//  Revision      :
//  Author        : Yaosheng Fu
//  Company       : Princeton University
//  Email         : yfu@princeton.edu
//
//  Description   : L2 cache array error detection and correction
//
//
//==================================================================================================

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
//  Filename      : l2.h.pyv
//  Created On    : 2014-02-20
//  Revision      :
//  Author        : Yaosheng Fu
//  Company       : Princeton University
//  Email         : yfu@princeton.edu
//
//  Description   : header file for the L2 cache
//
//
//==================================================================================================

// devices.xml



// Input buffer for pipeline1













// Input buffer for pipeline2














// Output buffer















// L2 cache configuration







//`define L2_SIZE                  65536
//`define L2_SIZE_WIDTH            16






//`define L2_LINE_SIZE             64
//`define L2_LINE_SIZE_WIDTH       6













//`define L2_WAYS                  4
//`define L2_WAYS_WIDTH            2
//`define L2_WAY_0                 2'b00
//`define L2_WAY_1                 2'b01
//`define L2_WAY_2                 2'b10
//`define L2_WAY_3                 2'b11





// Tag array







//`define L2_TAG_INDEX_WIDTH      8
//`define L2_TAG_WIDTH            26
//`define L2_TAG_WAY_WIDTH        26
//`define L2_TAG_ARRAY_WIDTH      104


// Tag Address decomposition 




//`define L2_TAG_INDEX            13:6
//`define L2_TAG                  39:14


//Data array












//`define L2_DATA_INDEX_WIDTH         12 








// Data Address decomposition 







//Dir array



//`define L2_DIR_INDEX_WIDTH      10 





//State array

















//Whether the cache line is in Icaches or Dcaches
















//Round Robin selection














// State decomposition 












//MSHR array



































//`define L2_MSHR_CMP_ADDR        13:6
//`define L2_MSHR_ADDR            39:0
//`define L2_MSHR_WAY             41:40
//`define L2_MSHR_MSHRID          49:42
//`define L2_MSHR_CACHE_TYPE      50
//`define L2_MSHR_DATA_SIZE       53:51
//`define L2_MSHR_MSG_TYPE        61:54
//`define L2_MSHR_L2_MISS         62
//`define L2_MSHR_SRC_CHIPID      76:63
//`define L2_MSHR_SRC_X           84:77
//`define L2_MSHR_SRC_Y           92:85
//`define L2_MSHR_SRC_FBITS       96:93
//`define L2_MSHR_SDID            106:97
//`define L2_MSHR_LSID            112:107      
//`define L2_MSHR_MISS_LSID       118:113
//`define L2_MSHR_SMC_MISS        119
//`define L2_MSHR_RECYCLED        120
//`define L2_MSHR_INV_FWD_PENDING 121


//SMC array


































//Message destination_type







//L2 public sharer beyond the maximum clump size



//L2 registers



//Special addresses





















//L2 core ID





//L2 dir array reuse







//Control Signal 








//AMO ALU OP macros












//Control Signal in Stage 1























































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








































































































































































































































































































































































































































































































































































































module l2_data_ecc ( 
   input  [64-1:0]      din,
   input  [8-1:0]	 parity,

   output [64-1:0]      dout,
   output                                    corr_error,
   output                                    uncorr_error
);

   
wire [64-1:0] 	err_bit_pos;
wire [8-2:0]    cflag;
wire 	                                pflag;
assign cflag[0] = parity[0]  ^ din[0] ^ din[1] ^ din[3] ^ din[4] ^ din[6] ^ din[8] ^ din[10] ^ din[11] ^ din[13] ^ din[15] ^ din[17] ^ din[19] ^ din[21] ^ din[23] ^ din[25] ^ din[26] ^ din[28] ^ din[30] ^ din[32] ^ din[34] ^ din[36] ^ din[38] ^ din[40] ^ din[42] ^ din[44] ^ din[46] ^ din[48] ^ din[50] ^ din[52] ^ din[54] ^ din[56] ^ din[57] ^ din[59] ^ din[61] ^ din[63] ;

assign cflag[1] = parity[1]  ^ din[0] ^ din[2] ^ din[3] ^ din[5] ^ din[6] ^ din[9] ^ din[10] ^ din[12] ^ din[13] ^ din[16] ^ din[17] ^ din[20] ^ din[21] ^ din[24] ^ din[25] ^ din[27] ^ din[28] ^ din[31] ^ din[32] ^ din[35] ^ din[36] ^ din[39] ^ din[40] ^ din[43] ^ din[44] ^ din[47] ^ din[48] ^ din[51] ^ din[52] ^ din[55] ^ din[56] ^ din[58] ^ din[59] ^ din[62] ^ din[63] ;

assign cflag[2] = parity[2]  ^ din[1] ^ din[2] ^ din[3] ^ din[7] ^ din[8] ^ din[9] ^ din[10] ^ din[14] ^ din[15] ^ din[16] ^ din[17] ^ din[22] ^ din[23] ^ din[24] ^ din[25] ^ din[29] ^ din[30] ^ din[31] ^ din[32] ^ din[37] ^ din[38] ^ din[39] ^ din[40] ^ din[45] ^ din[46] ^ din[47] ^ din[48] ^ din[53] ^ din[54] ^ din[55] ^ din[56] ^ din[60] ^ din[61] ^ din[62] ^ din[63] ;

assign cflag[3] = parity[3]  ^ din[4] ^ din[5] ^ din[6] ^ din[7] ^ din[8] ^ din[9] ^ din[10] ^ din[18] ^ din[19] ^ din[20] ^ din[21] ^ din[22] ^ din[23] ^ din[24] ^ din[25] ^ din[33] ^ din[34] ^ din[35] ^ din[36] ^ din[37] ^ din[38] ^ din[39] ^ din[40] ^ din[49] ^ din[50] ^ din[51] ^ din[52] ^ din[53] ^ din[54] ^ din[55] ^ din[56] ;

assign cflag[4] = parity[4]  ^ din[11] ^ din[12] ^ din[13] ^ din[14] ^ din[15] ^ din[16] ^ din[17] ^ din[18] ^ din[19] ^ din[20] ^ din[21] ^ din[22] ^ din[23] ^ din[24] ^ din[25] ^ din[41] ^ din[42] ^ din[43] ^ din[44] ^ din[45] ^ din[46] ^ din[47] ^ din[48] ^ din[49] ^ din[50] ^ din[51] ^ din[52] ^ din[53] ^ din[54] ^ din[55] ^ din[56] ;

assign cflag[5] = parity[5]  ^ din[26] ^ din[27] ^ din[28] ^ din[29] ^ din[30] ^ din[31] ^ din[32] ^ din[33] ^ din[34] ^ din[35] ^ din[36] ^ din[37] ^ din[38] ^ din[39] ^ din[40] ^ din[41] ^ din[42] ^ din[43] ^ din[44] ^ din[45] ^ din[46] ^ din[47] ^ din[48] ^ din[49] ^ din[50] ^ din[51] ^ din[52] ^ din[53] ^ din[54] ^ din[55] ^ din[56] ;

assign cflag[6] = parity[6]  ^ din[57] ^ din[58] ^ din[59] ^ din[60] ^ din[61] ^ din[62] ^ din[63] ;

assign pflag = cflag[0]
 ^ parity[1]  ^ parity[2]  ^ parity[3]  ^ parity[4]  ^ parity[5]  ^ parity[6] 
^ din[2] ^ din[5] ^ din[7] ^ din[9] ^ din[12] ^ din[14] ^ din[16] ^ din[18] ^ din[20] ^ din[22] ^ din[24] ^ din[27] ^ din[29] ^ din[31] ^ din[33] ^ din[35] ^ din[37] ^ din[39] ^ din[41] ^ din[43] ^ din[45] ^ din[47] ^ din[49] ^ din[51] ^ din[53] ^ din[55] ^ din[58] ^ din[60] ^ din[62] ;

assign err_bit_pos[0] =  (cflag[0]) & (cflag[1]) & (~cflag[2]) & (~cflag[3]) & (~cflag[4]) & (~cflag[5]) & (~cflag[6]) ;

assign err_bit_pos[1] =  (cflag[0]) & (~cflag[1]) & (cflag[2]) & (~cflag[3]) & (~cflag[4]) & (~cflag[5]) & (~cflag[6]) ;

assign err_bit_pos[2] =  (~cflag[0]) & (cflag[1]) & (cflag[2]) & (~cflag[3]) & (~cflag[4]) & (~cflag[5]) & (~cflag[6]) ;

assign err_bit_pos[3] =  (cflag[0]) & (cflag[1]) & (cflag[2]) & (~cflag[3]) & (~cflag[4]) & (~cflag[5]) & (~cflag[6]) ;

assign err_bit_pos[4] =  (cflag[0]) & (~cflag[1]) & (~cflag[2]) & (cflag[3]) & (~cflag[4]) & (~cflag[5]) & (~cflag[6]) ;

assign err_bit_pos[5] =  (~cflag[0]) & (cflag[1]) & (~cflag[2]) & (cflag[3]) & (~cflag[4]) & (~cflag[5]) & (~cflag[6]) ;

assign err_bit_pos[6] =  (cflag[0]) & (cflag[1]) & (~cflag[2]) & (cflag[3]) & (~cflag[4]) & (~cflag[5]) & (~cflag[6]) ;

assign err_bit_pos[7] =  (~cflag[0]) & (~cflag[1]) & (cflag[2]) & (cflag[3]) & (~cflag[4]) & (~cflag[5]) & (~cflag[6]) ;

assign err_bit_pos[8] =  (cflag[0]) & (~cflag[1]) & (cflag[2]) & (cflag[3]) & (~cflag[4]) & (~cflag[5]) & (~cflag[6]) ;

assign err_bit_pos[9] =  (~cflag[0]) & (cflag[1]) & (cflag[2]) & (cflag[3]) & (~cflag[4]) & (~cflag[5]) & (~cflag[6]) ;

assign err_bit_pos[10] =  (cflag[0]) & (cflag[1]) & (cflag[2]) & (cflag[3]) & (~cflag[4]) & (~cflag[5]) & (~cflag[6]) ;

assign err_bit_pos[11] =  (cflag[0]) & (~cflag[1]) & (~cflag[2]) & (~cflag[3]) & (cflag[4]) & (~cflag[5]) & (~cflag[6]) ;

assign err_bit_pos[12] =  (~cflag[0]) & (cflag[1]) & (~cflag[2]) & (~cflag[3]) & (cflag[4]) & (~cflag[5]) & (~cflag[6]) ;

assign err_bit_pos[13] =  (cflag[0]) & (cflag[1]) & (~cflag[2]) & (~cflag[3]) & (cflag[4]) & (~cflag[5]) & (~cflag[6]) ;

assign err_bit_pos[14] =  (~cflag[0]) & (~cflag[1]) & (cflag[2]) & (~cflag[3]) & (cflag[4]) & (~cflag[5]) & (~cflag[6]) ;

assign err_bit_pos[15] =  (cflag[0]) & (~cflag[1]) & (cflag[2]) & (~cflag[3]) & (cflag[4]) & (~cflag[5]) & (~cflag[6]) ;

assign err_bit_pos[16] =  (~cflag[0]) & (cflag[1]) & (cflag[2]) & (~cflag[3]) & (cflag[4]) & (~cflag[5]) & (~cflag[6]) ;

assign err_bit_pos[17] =  (cflag[0]) & (cflag[1]) & (cflag[2]) & (~cflag[3]) & (cflag[4]) & (~cflag[5]) & (~cflag[6]) ;

assign err_bit_pos[18] =  (~cflag[0]) & (~cflag[1]) & (~cflag[2]) & (cflag[3]) & (cflag[4]) & (~cflag[5]) & (~cflag[6]) ;

assign err_bit_pos[19] =  (cflag[0]) & (~cflag[1]) & (~cflag[2]) & (cflag[3]) & (cflag[4]) & (~cflag[5]) & (~cflag[6]) ;

assign err_bit_pos[20] =  (~cflag[0]) & (cflag[1]) & (~cflag[2]) & (cflag[3]) & (cflag[4]) & (~cflag[5]) & (~cflag[6]) ;

assign err_bit_pos[21] =  (cflag[0]) & (cflag[1]) & (~cflag[2]) & (cflag[3]) & (cflag[4]) & (~cflag[5]) & (~cflag[6]) ;

assign err_bit_pos[22] =  (~cflag[0]) & (~cflag[1]) & (cflag[2]) & (cflag[3]) & (cflag[4]) & (~cflag[5]) & (~cflag[6]) ;

assign err_bit_pos[23] =  (cflag[0]) & (~cflag[1]) & (cflag[2]) & (cflag[3]) & (cflag[4]) & (~cflag[5]) & (~cflag[6]) ;

assign err_bit_pos[24] =  (~cflag[0]) & (cflag[1]) & (cflag[2]) & (cflag[3]) & (cflag[4]) & (~cflag[5]) & (~cflag[6]) ;

assign err_bit_pos[25] =  (cflag[0]) & (cflag[1]) & (cflag[2]) & (cflag[3]) & (cflag[4]) & (~cflag[5]) & (~cflag[6]) ;

assign err_bit_pos[26] =  (cflag[0]) & (~cflag[1]) & (~cflag[2]) & (~cflag[3]) & (~cflag[4]) & (cflag[5]) & (~cflag[6]) ;

assign err_bit_pos[27] =  (~cflag[0]) & (cflag[1]) & (~cflag[2]) & (~cflag[3]) & (~cflag[4]) & (cflag[5]) & (~cflag[6]) ;

assign err_bit_pos[28] =  (cflag[0]) & (cflag[1]) & (~cflag[2]) & (~cflag[3]) & (~cflag[4]) & (cflag[5]) & (~cflag[6]) ;

assign err_bit_pos[29] =  (~cflag[0]) & (~cflag[1]) & (cflag[2]) & (~cflag[3]) & (~cflag[4]) & (cflag[5]) & (~cflag[6]) ;

assign err_bit_pos[30] =  (cflag[0]) & (~cflag[1]) & (cflag[2]) & (~cflag[3]) & (~cflag[4]) & (cflag[5]) & (~cflag[6]) ;

assign err_bit_pos[31] =  (~cflag[0]) & (cflag[1]) & (cflag[2]) & (~cflag[3]) & (~cflag[4]) & (cflag[5]) & (~cflag[6]) ;

assign err_bit_pos[32] =  (cflag[0]) & (cflag[1]) & (cflag[2]) & (~cflag[3]) & (~cflag[4]) & (cflag[5]) & (~cflag[6]) ;

assign err_bit_pos[33] =  (~cflag[0]) & (~cflag[1]) & (~cflag[2]) & (cflag[3]) & (~cflag[4]) & (cflag[5]) & (~cflag[6]) ;

assign err_bit_pos[34] =  (cflag[0]) & (~cflag[1]) & (~cflag[2]) & (cflag[3]) & (~cflag[4]) & (cflag[5]) & (~cflag[6]) ;

assign err_bit_pos[35] =  (~cflag[0]) & (cflag[1]) & (~cflag[2]) & (cflag[3]) & (~cflag[4]) & (cflag[5]) & (~cflag[6]) ;

assign err_bit_pos[36] =  (cflag[0]) & (cflag[1]) & (~cflag[2]) & (cflag[3]) & (~cflag[4]) & (cflag[5]) & (~cflag[6]) ;

assign err_bit_pos[37] =  (~cflag[0]) & (~cflag[1]) & (cflag[2]) & (cflag[3]) & (~cflag[4]) & (cflag[5]) & (~cflag[6]) ;

assign err_bit_pos[38] =  (cflag[0]) & (~cflag[1]) & (cflag[2]) & (cflag[3]) & (~cflag[4]) & (cflag[5]) & (~cflag[6]) ;

assign err_bit_pos[39] =  (~cflag[0]) & (cflag[1]) & (cflag[2]) & (cflag[3]) & (~cflag[4]) & (cflag[5]) & (~cflag[6]) ;

assign err_bit_pos[40] =  (cflag[0]) & (cflag[1]) & (cflag[2]) & (cflag[3]) & (~cflag[4]) & (cflag[5]) & (~cflag[6]) ;

assign err_bit_pos[41] =  (~cflag[0]) & (~cflag[1]) & (~cflag[2]) & (~cflag[3]) & (cflag[4]) & (cflag[5]) & (~cflag[6]) ;

assign err_bit_pos[42] =  (cflag[0]) & (~cflag[1]) & (~cflag[2]) & (~cflag[3]) & (cflag[4]) & (cflag[5]) & (~cflag[6]) ;

assign err_bit_pos[43] =  (~cflag[0]) & (cflag[1]) & (~cflag[2]) & (~cflag[3]) & (cflag[4]) & (cflag[5]) & (~cflag[6]) ;

assign err_bit_pos[44] =  (cflag[0]) & (cflag[1]) & (~cflag[2]) & (~cflag[3]) & (cflag[4]) & (cflag[5]) & (~cflag[6]) ;

assign err_bit_pos[45] =  (~cflag[0]) & (~cflag[1]) & (cflag[2]) & (~cflag[3]) & (cflag[4]) & (cflag[5]) & (~cflag[6]) ;

assign err_bit_pos[46] =  (cflag[0]) & (~cflag[1]) & (cflag[2]) & (~cflag[3]) & (cflag[4]) & (cflag[5]) & (~cflag[6]) ;

assign err_bit_pos[47] =  (~cflag[0]) & (cflag[1]) & (cflag[2]) & (~cflag[3]) & (cflag[4]) & (cflag[5]) & (~cflag[6]) ;

assign err_bit_pos[48] =  (cflag[0]) & (cflag[1]) & (cflag[2]) & (~cflag[3]) & (cflag[4]) & (cflag[5]) & (~cflag[6]) ;

assign err_bit_pos[49] =  (~cflag[0]) & (~cflag[1]) & (~cflag[2]) & (cflag[3]) & (cflag[4]) & (cflag[5]) & (~cflag[6]) ;

assign err_bit_pos[50] =  (cflag[0]) & (~cflag[1]) & (~cflag[2]) & (cflag[3]) & (cflag[4]) & (cflag[5]) & (~cflag[6]) ;

assign err_bit_pos[51] =  (~cflag[0]) & (cflag[1]) & (~cflag[2]) & (cflag[3]) & (cflag[4]) & (cflag[5]) & (~cflag[6]) ;

assign err_bit_pos[52] =  (cflag[0]) & (cflag[1]) & (~cflag[2]) & (cflag[3]) & (cflag[4]) & (cflag[5]) & (~cflag[6]) ;

assign err_bit_pos[53] =  (~cflag[0]) & (~cflag[1]) & (cflag[2]) & (cflag[3]) & (cflag[4]) & (cflag[5]) & (~cflag[6]) ;

assign err_bit_pos[54] =  (cflag[0]) & (~cflag[1]) & (cflag[2]) & (cflag[3]) & (cflag[4]) & (cflag[5]) & (~cflag[6]) ;

assign err_bit_pos[55] =  (~cflag[0]) & (cflag[1]) & (cflag[2]) & (cflag[3]) & (cflag[4]) & (cflag[5]) & (~cflag[6]) ;

assign err_bit_pos[56] =  (cflag[0]) & (cflag[1]) & (cflag[2]) & (cflag[3]) & (cflag[4]) & (cflag[5]) & (~cflag[6]) ;

assign err_bit_pos[57] =  (cflag[0]) & (~cflag[1]) & (~cflag[2]) & (~cflag[3]) & (~cflag[4]) & (~cflag[5]) & (cflag[6]) ;

assign err_bit_pos[58] =  (~cflag[0]) & (cflag[1]) & (~cflag[2]) & (~cflag[3]) & (~cflag[4]) & (~cflag[5]) & (cflag[6]) ;

assign err_bit_pos[59] =  (cflag[0]) & (cflag[1]) & (~cflag[2]) & (~cflag[3]) & (~cflag[4]) & (~cflag[5]) & (cflag[6]) ;

assign err_bit_pos[60] =  (~cflag[0]) & (~cflag[1]) & (cflag[2]) & (~cflag[3]) & (~cflag[4]) & (~cflag[5]) & (cflag[6]) ;

assign err_bit_pos[61] =  (cflag[0]) & (~cflag[1]) & (cflag[2]) & (~cflag[3]) & (~cflag[4]) & (~cflag[5]) & (cflag[6]) ;

assign err_bit_pos[62] =  (~cflag[0]) & (cflag[1]) & (cflag[2]) & (~cflag[3]) & (~cflag[4]) & (~cflag[5]) & (cflag[6]) ;

assign err_bit_pos[63] =  (cflag[0]) & (cflag[1]) & (cflag[2]) & (~cflag[3]) & (~cflag[4]) & (~cflag[5]) & (cflag[6]) ;





//correct the error bit, it can only correct one error bit.

assign dout = din ^ err_bit_pos;

assign corr_error = pflag;

assign uncorr_error = |(cflag[8-2:0]) & ~pflag;


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
//  Filename      : l2_data_pgen.v
//  Created On    : 2014-06-02
//  Revision      :
//  Author        : Yaosheng Fu
//  Company       : Princeton University
//  Email         : yfu@princeton.edu
//
//  Description   : L2 cache array parity bits generation
//
//
//==================================================================================================

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
//  Filename      : l2.h.pyv
//  Created On    : 2014-02-20
//  Revision      :
//  Author        : Yaosheng Fu
//  Company       : Princeton University
//  Email         : yfu@princeton.edu
//
//  Description   : header file for the L2 cache
//
//
//==================================================================================================

// devices.xml



// Input buffer for pipeline1













// Input buffer for pipeline2














// Output buffer















// L2 cache configuration







//`define L2_SIZE                  65536
//`define L2_SIZE_WIDTH            16






//`define L2_LINE_SIZE             64
//`define L2_LINE_SIZE_WIDTH       6













//`define L2_WAYS                  4
//`define L2_WAYS_WIDTH            2
//`define L2_WAY_0                 2'b00
//`define L2_WAY_1                 2'b01
//`define L2_WAY_2                 2'b10
//`define L2_WAY_3                 2'b11





// Tag array







//`define L2_TAG_INDEX_WIDTH      8
//`define L2_TAG_WIDTH            26
//`define L2_TAG_WAY_WIDTH        26
//`define L2_TAG_ARRAY_WIDTH      104


// Tag Address decomposition 




//`define L2_TAG_INDEX            13:6
//`define L2_TAG                  39:14


//Data array












//`define L2_DATA_INDEX_WIDTH         12 








// Data Address decomposition 







//Dir array



//`define L2_DIR_INDEX_WIDTH      10 





//State array

















//Whether the cache line is in Icaches or Dcaches
















//Round Robin selection














// State decomposition 












//MSHR array



































//`define L2_MSHR_CMP_ADDR        13:6
//`define L2_MSHR_ADDR            39:0
//`define L2_MSHR_WAY             41:40
//`define L2_MSHR_MSHRID          49:42
//`define L2_MSHR_CACHE_TYPE      50
//`define L2_MSHR_DATA_SIZE       53:51
//`define L2_MSHR_MSG_TYPE        61:54
//`define L2_MSHR_L2_MISS         62
//`define L2_MSHR_SRC_CHIPID      76:63
//`define L2_MSHR_SRC_X           84:77
//`define L2_MSHR_SRC_Y           92:85
//`define L2_MSHR_SRC_FBITS       96:93
//`define L2_MSHR_SDID            106:97
//`define L2_MSHR_LSID            112:107      
//`define L2_MSHR_MISS_LSID       118:113
//`define L2_MSHR_SMC_MISS        119
//`define L2_MSHR_RECYCLED        120
//`define L2_MSHR_INV_FWD_PENDING 121


//SMC array


































//Message destination_type







//L2 public sharer beyond the maximum clump size



//L2 registers



//Special addresses





















//L2 core ID





//L2 dir array reuse







//Control Signal 








//AMO ALU OP macros












//Control Signal in Stage 1























































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








































































































































































































































































































































































































































































































































































































module l2_data_pgen ( 
   input    [64-1:0]   din,

   output   [8-1:0] parity
);


//generate parity bits based on the hamming codes
assign parity[0] =  din[0] ^ din[1] ^ din[3] ^ din[4] ^ din[6] ^ din[8] ^ din[10] ^ din[11] ^ din[13] ^ din[15] ^ din[17] ^ din[19] ^ din[21] ^ din[23] ^ din[25] ^ din[26] ^ din[28] ^ din[30] ^ din[32] ^ din[34] ^ din[36] ^ din[38] ^ din[40] ^ din[42] ^ din[44] ^ din[46] ^ din[48] ^ din[50] ^ din[52] ^ din[54] ^ din[56] ^ din[57] ^ din[59] ^ din[61] ^ din[63] ;

assign parity[1] =  din[0] ^ din[2] ^ din[3] ^ din[5] ^ din[6] ^ din[9] ^ din[10] ^ din[12] ^ din[13] ^ din[16] ^ din[17] ^ din[20] ^ din[21] ^ din[24] ^ din[25] ^ din[27] ^ din[28] ^ din[31] ^ din[32] ^ din[35] ^ din[36] ^ din[39] ^ din[40] ^ din[43] ^ din[44] ^ din[47] ^ din[48] ^ din[51] ^ din[52] ^ din[55] ^ din[56] ^ din[58] ^ din[59] ^ din[62] ^ din[63] ;

assign parity[2] =  din[1] ^ din[2] ^ din[3] ^ din[7] ^ din[8] ^ din[9] ^ din[10] ^ din[14] ^ din[15] ^ din[16] ^ din[17] ^ din[22] ^ din[23] ^ din[24] ^ din[25] ^ din[29] ^ din[30] ^ din[31] ^ din[32] ^ din[37] ^ din[38] ^ din[39] ^ din[40] ^ din[45] ^ din[46] ^ din[47] ^ din[48] ^ din[53] ^ din[54] ^ din[55] ^ din[56] ^ din[60] ^ din[61] ^ din[62] ^ din[63] ;

assign parity[3] =  din[4] ^ din[5] ^ din[6] ^ din[7] ^ din[8] ^ din[9] ^ din[10] ^ din[18] ^ din[19] ^ din[20] ^ din[21] ^ din[22] ^ din[23] ^ din[24] ^ din[25] ^ din[33] ^ din[34] ^ din[35] ^ din[36] ^ din[37] ^ din[38] ^ din[39] ^ din[40] ^ din[49] ^ din[50] ^ din[51] ^ din[52] ^ din[53] ^ din[54] ^ din[55] ^ din[56] ;

assign parity[4] =  din[11] ^ din[12] ^ din[13] ^ din[14] ^ din[15] ^ din[16] ^ din[17] ^ din[18] ^ din[19] ^ din[20] ^ din[21] ^ din[22] ^ din[23] ^ din[24] ^ din[25] ^ din[41] ^ din[42] ^ din[43] ^ din[44] ^ din[45] ^ din[46] ^ din[47] ^ din[48] ^ din[49] ^ din[50] ^ din[51] ^ din[52] ^ din[53] ^ din[54] ^ din[55] ^ din[56] ;

assign parity[5] =  din[26] ^ din[27] ^ din[28] ^ din[29] ^ din[30] ^ din[31] ^ din[32] ^ din[33] ^ din[34] ^ din[35] ^ din[36] ^ din[37] ^ din[38] ^ din[39] ^ din[40] ^ din[41] ^ din[42] ^ din[43] ^ din[44] ^ din[45] ^ din[46] ^ din[47] ^ din[48] ^ din[49] ^ din[50] ^ din[51] ^ din[52] ^ din[53] ^ din[54] ^ din[55] ^ din[56] ;

assign parity[6] =  din[57] ^ din[58] ^ din[59] ^ din[60] ^ din[61] ^ din[62] ^ din[63] ;

assign parity[7] =  din[0] ^ din[1] ^ din[2] ^ din[4] ^ din[5] ^ din[7] ^ din[10] ^ din[11] ^ din[12] ^ din[14] ^ din[17] ^ din[18] ^ din[21] ^ din[23] ^ din[24] ^ din[26] ^ din[27] ^ din[29] ^ din[32] ^ din[33] ^ din[36] ^ din[38] ^ din[39] ^ din[41] ^ din[44] ^ din[46] ^ din[47] ^ din[50] ^ din[51] ^ din[53] ^ din[56] ^ din[57] ^ din[58] ^ din[60] ^ din[63] ;




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
//  Filename      : l2_mshr.v
//  Created On    : 2014-02-24
//  Revision      :
//  Author        : Yaosheng Fu
//  Company       : Princeton University
//  Email         : yfu@princeton.edu
//
//  Description   : The MSHRs in the L2 cache
//
//
//==================================================================================================

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
//  Filename      : l2.h.pyv
//  Created On    : 2014-02-20
//  Revision      :
//  Author        : Yaosheng Fu
//  Company       : Princeton University
//  Email         : yfu@princeton.edu
//
//  Description   : header file for the L2 cache
//
//
//==================================================================================================

// devices.xml



// Input buffer for pipeline1













// Input buffer for pipeline2














// Output buffer















// L2 cache configuration







//`define L2_SIZE                  65536
//`define L2_SIZE_WIDTH            16






//`define L2_LINE_SIZE             64
//`define L2_LINE_SIZE_WIDTH       6













//`define L2_WAYS                  4
//`define L2_WAYS_WIDTH            2
//`define L2_WAY_0                 2'b00
//`define L2_WAY_1                 2'b01
//`define L2_WAY_2                 2'b10
//`define L2_WAY_3                 2'b11





// Tag array







//`define L2_TAG_INDEX_WIDTH      8
//`define L2_TAG_WIDTH            26
//`define L2_TAG_WAY_WIDTH        26
//`define L2_TAG_ARRAY_WIDTH      104


// Tag Address decomposition 




//`define L2_TAG_INDEX            13:6
//`define L2_TAG                  39:14


//Data array












//`define L2_DATA_INDEX_WIDTH         12 








// Data Address decomposition 







//Dir array



//`define L2_DIR_INDEX_WIDTH      10 





//State array

















//Whether the cache line is in Icaches or Dcaches
















//Round Robin selection














// State decomposition 












//MSHR array



































//`define L2_MSHR_CMP_ADDR        13:6
//`define L2_MSHR_ADDR            39:0
//`define L2_MSHR_WAY             41:40
//`define L2_MSHR_MSHRID          49:42
//`define L2_MSHR_CACHE_TYPE      50
//`define L2_MSHR_DATA_SIZE       53:51
//`define L2_MSHR_MSG_TYPE        61:54
//`define L2_MSHR_L2_MISS         62
//`define L2_MSHR_SRC_CHIPID      76:63
//`define L2_MSHR_SRC_X           84:77
//`define L2_MSHR_SRC_Y           92:85
//`define L2_MSHR_SRC_FBITS       96:93
//`define L2_MSHR_SDID            106:97
//`define L2_MSHR_LSID            112:107      
//`define L2_MSHR_MISS_LSID       118:113
//`define L2_MSHR_SMC_MISS        119
//`define L2_MSHR_RECYCLED        120
//`define L2_MSHR_INV_FWD_PENDING 121


//SMC array


































//Message destination_type







//L2 public sharer beyond the maximum clump size



//L2 registers



//Special addresses





















//L2 core ID





//L2 dir array reuse







//Control Signal 








//AMO ALU OP macros












//Control Signal in Stage 1























































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





































































































































































































































































































































































































































































































































































































module l2_mshr(

    input wire clk,
    input wire rst_n,

    //Read enable
    input wire rd_en,

    //cam enable
    input wire cam_en,

    //Write state enable
    input wire wr_state_en,


    //Write data enable
    input wire wr_data_en,

    //Reading out a pending entry
    input wire pending_ready,

    //increment counter which is used as a temporary place to count invalidated sharers
    input wire inc_counter_en,


    //Write state
    input wire [2-1:0] state_in,

    //Write data
    input wire [120+2-1:0] data_in,
    input wire [120+2-1:0] data_mask_in,

    //Read index
    input wire [3-1:0] rd_index_in,

    //Inv counter index
    input wire [3-1:0] inv_counter_rd_index_in,

    //Write/increment counter index
    input wire [3-1:0] wr_index_in,

    //CAM address
    input wire [8-1:0] addr_in,



    output reg hit,
    output reg [3-1:0] hit_index,
    
    //Read or pending output
    output reg [2-1:0] state_out,
    output reg [120+2-1:0] data_out,
    output reg [6-1:0] inv_counter_out,

    //Status output
    output reg [3:0] empty_slots,
    output reg pending,
    output reg [3-1:0] pending_index,
    output reg [3-1:0] empty_index
);



reg [2-1:0] state_mem_f [8-1:0];
reg [120+2-1:0] data_mem_f [8-1:0];
reg [6-1:0] counter_mem_f [8-1:0];
reg [3-1:0] wbg_counter_f;
reg [3-1:0] wbg_counter_next;

always @ *
begin
    empty_slots = 0;
    if (state_mem_f[0] == 2'd0)
    begin
        empty_slots = empty_slots + 1;
    end
    if (state_mem_f[1] == 2'd0)
    begin
        empty_slots = empty_slots + 1;
    end
    if (state_mem_f[2] == 2'd0)
    begin
        empty_slots = empty_slots + 1;
    end
    if (state_mem_f[3] == 2'd0)
    begin
        empty_slots = empty_slots + 1;
    end
    if (state_mem_f[4] == 2'd0)
    begin
        empty_slots = empty_slots + 1;
    end
    if (state_mem_f[5] == 2'd0)
    begin
        empty_slots = empty_slots + 1;
    end
    if (state_mem_f[6] == 2'd0)
    begin
        empty_slots = empty_slots + 1;
    end
    if (state_mem_f[7] == 2'd0)
    begin
        empty_slots = empty_slots + 1;
    end

end

always @ *
begin
    if (state_mem_f[0] == 2'd2)
    begin
        pending = 1'b1;
        pending_index = 3'd0;
    end
    else if (state_mem_f[1] == 2'd2)
    begin
        pending = 1'b1;
        pending_index = 3'd1;
    end
    else if (state_mem_f[2] == 2'd2)
    begin
        pending = 1'b1;
        pending_index = 3'd2;
    end
    else if (state_mem_f[3] == 2'd2)
    begin
        pending = 1'b1;
        pending_index = 3'd3;
    end
    else if (state_mem_f[4] == 2'd2)
    begin
        pending = 1'b1;
        pending_index = 3'd4;
    end
    else if (state_mem_f[5] == 2'd2)
    begin
        pending = 1'b1;
        pending_index = 3'd5;
    end
    else if (state_mem_f[6] == 2'd2)
    begin
        pending = 1'b1;
        pending_index = 3'd6;
    end
    else if (state_mem_f[7] == 2'd2)
    begin
        pending = 1'b1;
        pending_index = 3'd7;
    end
    else
    begin
        pending = 1'b0;
        pending_index = 3'd0;
    end

end

always @ *
begin
    if (state_mem_f[0] == 2'd0)
    begin
        empty_index = 3'd0;
    end
    else if (state_mem_f[1] == 2'd0)
    begin
        empty_index = 3'd1;
    end
    else if (state_mem_f[2] == 2'd0)
    begin
        empty_index = 3'd2;
    end
    else if (state_mem_f[3] == 2'd0)
    begin
        empty_index = 3'd3;
    end
    else if (state_mem_f[4] == 2'd0)
    begin
        empty_index = 3'd4;
    end
    else if (state_mem_f[5] == 2'd0)
    begin
        empty_index = 3'd5;
    end
    else if (state_mem_f[6] == 2'd0)
    begin
        empty_index = 3'd6;
    end
    else if (state_mem_f[7] == 2'd0)
    begin
        empty_index = 3'd7;
    end
    else
    begin
        empty_index = 3'd0;
    end

end

always @ *
begin
    if (rd_en)
    begin
        state_out = state_mem_f[rd_index_in];
        data_out = data_mem_f[rd_index_in];
    end
    else if (cam_en && hit)
    begin
        state_out = state_mem_f[hit_index];
        data_out = data_mem_f[hit_index];
    end
    else if (pending)
    begin
        state_out = state_mem_f[pending_index];
        data_out = data_mem_f[pending_index];
    end
    else
    begin
        state_out = 2'd0;
        data_out = 0;
    end
end

always @ *
begin
    inv_counter_out = counter_mem_f[inv_counter_rd_index_in];
end

always @ *
begin
    if(cam_en)
    begin
        if ((data_mem_f[0][6+8-1:6] == addr_in) && (state_mem_f[0] != 2'd0))
        begin
            hit = 1'b1;
            hit_index = 3'd0;
        end
        else if ((data_mem_f[1][6+8-1:6] == addr_in) && (state_mem_f[1] != 2'd0))
        begin
            hit = 1'b1;
            hit_index = 3'd1;
        end
        else if ((data_mem_f[2][6+8-1:6] == addr_in) && (state_mem_f[2] != 2'd0))
        begin
            hit = 1'b1;
            hit_index = 3'd2;
        end
        else if ((data_mem_f[3][6+8-1:6] == addr_in) && (state_mem_f[3] != 2'd0))
        begin
            hit = 1'b1;
            hit_index = 3'd3;
        end
        else if ((data_mem_f[4][6+8-1:6] == addr_in) && (state_mem_f[4] != 2'd0))
        begin
            hit = 1'b1;
            hit_index = 3'd4;
        end
        else if ((data_mem_f[5][6+8-1:6] == addr_in) && (state_mem_f[5] != 2'd0))
        begin
            hit = 1'b1;
            hit_index = 3'd5;
        end
        else if ((data_mem_f[6][6+8-1:6] == addr_in) && (state_mem_f[6] != 2'd0))
        begin
            hit = 1'b1;
            hit_index = 3'd6;
        end
        else if ((data_mem_f[7][6+8-1:6] == addr_in) && (state_mem_f[7] != 2'd0))
        begin
            hit = 1'b1;
            hit_index = 3'd7;
        end
        else
        begin
            hit = 1'b0;
            hit_index = 3'd0;
        end
    end
    else
    begin
        hit = 1'b0;
        hit_index = 3'd0;
    end

end




always @ (posedge clk)
begin
    if (!rst_n)
    begin
        state_mem_f[0] <= 2'd0;
        state_mem_f[1] <= 2'd0;
        state_mem_f[2] <= 2'd0;
        state_mem_f[3] <= 2'd0;
        state_mem_f[4] <= 2'd0;
        state_mem_f[5] <= 2'd0;
        state_mem_f[6] <= 2'd0;
        state_mem_f[7] <= 2'd0;

    end
    else if (wr_state_en)
    begin
        state_mem_f[wr_index_in] <= state_in;
        if (pending && pending_ready && (pending_index != wr_index_in))
        begin
            //SMC miss entries are locked in the mshr
            if (data_mem_f[pending_index][117+2])
            begin
                state_mem_f[pending_index] <= 2'd1;
            end
            else
            begin
                state_mem_f[pending_index] <= 2'd0;
            end
        end
    end
    else if (pending && pending_ready)
    begin
        if (data_mem_f[pending_index][117+2])
        begin
            state_mem_f[pending_index] <= 2'd1;
        end
        else
        begin
            state_mem_f[pending_index] <= 2'd0;
        end
    end
    else if (cam_en && hit && (data_mem_f[hit_index][59+2:52+2] == 8'd13))
    begin
        state_mem_f[hit_index] <= 2'd2;
    end
    //Clear entries with WB guard requests if they occupy more entries than the threshold
    else if (wbg_counter_f > 4)
    begin
        if ((state_mem_f[0] == 2'd1) && (data_mem_f[0][59+2:52+2] == 8'd13))
        begin
            state_mem_f[0] <= 2'd2;
        end
        if ((state_mem_f[1] == 2'd1) && (data_mem_f[1][59+2:52+2] == 8'd13))
        begin
            state_mem_f[1] <= 2'd2;
        end
        if ((state_mem_f[2] == 2'd1) && (data_mem_f[2][59+2:52+2] == 8'd13))
        begin
            state_mem_f[2] <= 2'd2;
        end
        if ((state_mem_f[3] == 2'd1) && (data_mem_f[3][59+2:52+2] == 8'd13))
        begin
            state_mem_f[3] <= 2'd2;
        end
        if ((state_mem_f[4] == 2'd1) && (data_mem_f[4][59+2:52+2] == 8'd13))
        begin
            state_mem_f[4] <= 2'd2;
        end
        if ((state_mem_f[5] == 2'd1) && (data_mem_f[5][59+2:52+2] == 8'd13))
        begin
            state_mem_f[5] <= 2'd2;
        end
        if ((state_mem_f[6] == 2'd1) && (data_mem_f[6][59+2:52+2] == 8'd13))
        begin
            state_mem_f[6] <= 2'd2;
        end
        if ((state_mem_f[7] == 2'd1) && (data_mem_f[7][59+2:52+2] == 8'd13))
        begin
            state_mem_f[7] <= 2'd2;
        end

    end
end



always @ *
begin
    if(wr_state_en && wr_data_en && (state_in == 2'd1) 
    && (data_in[59+2:52+2] == 8'd13) && (data_mask_in[59+2:52+2] == {8{1'b1}}))
    begin
        wbg_counter_next = wbg_counter_f + 1;
    end
    else if ((~wr_state_en) && (~(pending && pending_ready)) 
          && (cam_en && hit && (data_mem_f[hit_index][59+2:52+2] == 8'd13)))
    begin
        wbg_counter_next = wbg_counter_f - 1;
    end
    else
    begin
        wbg_counter_next = wbg_counter_f;
    end
end

always @ (posedge clk)
begin
    if (!rst_n)
    begin
        wbg_counter_f <= 0;
    end
    else   
    begin
        wbg_counter_f <= wbg_counter_next;
    end
end

always @ (posedge clk)
begin
    if (!rst_n)
    begin
        counter_mem_f[0] <= {6{1'b0}};
        counter_mem_f[1] <= {6{1'b0}};
        counter_mem_f[2] <= {6{1'b0}};
        counter_mem_f[3] <= {6{1'b0}};
        counter_mem_f[4] <= {6{1'b0}};
        counter_mem_f[5] <= {6{1'b0}};
        counter_mem_f[6] <= {6{1'b0}};
        counter_mem_f[7] <= {6{1'b0}};

    end
    else if (pending && pending_ready)
    begin
        counter_mem_f[pending_index] <= {6{1'b0}};
        if (inc_counter_en && (pending_index != wr_index_in))
        begin
            counter_mem_f[wr_index_in] <= counter_mem_f[wr_index_in] + 1;
        end
    end
    else if (inc_counter_en)
    begin
        counter_mem_f[wr_index_in] <= counter_mem_f[wr_index_in] + 1;
    end
end


always @ (posedge clk)
begin
    if (!rst_n)
    begin
        data_mem_f[0] <= {120+2{1'b0}};
        data_mem_f[1] <= {120+2{1'b0}};
        data_mem_f[2] <= {120+2{1'b0}};
        data_mem_f[3] <= {120+2{1'b0}};
        data_mem_f[4] <= {120+2{1'b0}};
        data_mem_f[5] <= {120+2{1'b0}};
        data_mem_f[6] <= {120+2{1'b0}};
        data_mem_f[7] <= {120+2{1'b0}};

    end
    else if (wr_data_en)
    begin
        data_mem_f[wr_index_in] <= (data_mem_f[wr_index_in] & (~data_mask_in))
                                 | (data_in & data_mask_in);
    end
    else
    begin
        data_mem_f[wr_index_in] <= data_mem_f[wr_index_in];
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

//==================================================================================================
//  Filename      : l2_mshr_wrap.v
//  Created On    : 2014-02-24
//  Revision      :
//  Author        : Yaosheng Fu
//  Company       : Princeton University
//  Email         : yfu@princeton.edu
//
//  Description   : The wrap module for MSHRs in the L2 cache
//
//
//==================================================================================================


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
//  Filename      : l2.h.pyv
//  Created On    : 2014-02-20
//  Revision      :
//  Author        : Yaosheng Fu
//  Company       : Princeton University
//  Email         : yfu@princeton.edu
//
//  Description   : header file for the L2 cache
//
//
//==================================================================================================

// devices.xml



// Input buffer for pipeline1













// Input buffer for pipeline2














// Output buffer















// L2 cache configuration







//`define L2_SIZE                  65536
//`define L2_SIZE_WIDTH            16






//`define L2_LINE_SIZE             64
//`define L2_LINE_SIZE_WIDTH       6













//`define L2_WAYS                  4
//`define L2_WAYS_WIDTH            2
//`define L2_WAY_0                 2'b00
//`define L2_WAY_1                 2'b01
//`define L2_WAY_2                 2'b10
//`define L2_WAY_3                 2'b11





// Tag array







//`define L2_TAG_INDEX_WIDTH      8
//`define L2_TAG_WIDTH            26
//`define L2_TAG_WAY_WIDTH        26
//`define L2_TAG_ARRAY_WIDTH      104


// Tag Address decomposition 




//`define L2_TAG_INDEX            13:6
//`define L2_TAG                  39:14


//Data array












//`define L2_DATA_INDEX_WIDTH         12 








// Data Address decomposition 







//Dir array



//`define L2_DIR_INDEX_WIDTH      10 





//State array

















//Whether the cache line is in Icaches or Dcaches
















//Round Robin selection














// State decomposition 












//MSHR array



































//`define L2_MSHR_CMP_ADDR        13:6
//`define L2_MSHR_ADDR            39:0
//`define L2_MSHR_WAY             41:40
//`define L2_MSHR_MSHRID          49:42
//`define L2_MSHR_CACHE_TYPE      50
//`define L2_MSHR_DATA_SIZE       53:51
//`define L2_MSHR_MSG_TYPE        61:54
//`define L2_MSHR_L2_MISS         62
//`define L2_MSHR_SRC_CHIPID      76:63
//`define L2_MSHR_SRC_X           84:77
//`define L2_MSHR_SRC_Y           92:85
//`define L2_MSHR_SRC_FBITS       96:93
//`define L2_MSHR_SDID            106:97
//`define L2_MSHR_LSID            112:107      
//`define L2_MSHR_MISS_LSID       118:113
//`define L2_MSHR_SMC_MISS        119
//`define L2_MSHR_RECYCLED        120
//`define L2_MSHR_INV_FWD_PENDING 121


//SMC array


































//Message destination_type







//L2 public sharer beyond the maximum clump size



//L2 registers



//Special addresses





















//L2 core ID





//L2 dir array reuse







//Control Signal 








//AMO ALU OP macros












//Control Signal in Stage 1























































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





































































































































































































































































































































































































































































































































































































module l2_mshr_wrap(

    input wire clk,
    input wire rst_n,

    input wire pipe_rd_sel,
 // L2_CAM_MSHR
    input wire pipe_wr_sel,


    input wire rd_en1,
 // L2_CAM_MSHR
    input wire cam_en1,
    input wire wr_state_en1,
    input wire wr_data_en1,
    input wire pending_ready1,
    input wire [2-1:0] state_in1,
    input wire [120+2-1:0] data_in1,
    input wire [120+2-1:0] data_mask_in1,

    input wire [3-1:0] rd_index_in1,
 // L2_CAM_MSHR
    input wire [3-1:0] inv_counter_rd_index_in1,
    input wire [3-1:0] wr_index_in1,
    input wire [8-1:0] addr_in1,


    input wire rd_en2,
    input wire cam_en2,
 // L2_CAM_MSHR
    input wire wr_state_en2,
    input wire wr_data_en2,

    input wire pending_ready2,
 // L2_CAM_MSHR
    input wire inc_counter_en2,
    input wire [2-1:0] state_in2,
    input wire [120+2-1:0] data_in2,
    input wire [120+2-1:0] data_mask_in2,
    input wire [3-1:0] rd_index_in2,
    input wire [3-1:0] wr_index_in2,

    input wire [8-1:0] addr_in2,
 // L2_CAM_MSHR


    output wire hit,
    output wire [3-1:0] hit_index,
    output wire [2-1:0] state_out,
    output wire [120+2-1:0] data_out,
    output wire [6-1:0] inv_counter_out,
    output wire [3:0] empty_slots,
    output wire pending,
    output wire [3-1:0] pending_index,
    output wire [3-1:0] empty_index














 // L2_CAM_MSHR
);


reg rd_en;
reg cam_en;
 // L2_CAM_MSHR
reg wr_state_en;
reg wr_data_en;

reg pending_ready;
 // L2_CAM_MSHR
reg [2-1:0] state_in;
reg [120+2-1:0] data_in;
reg [120+2-1:0] data_mask_in;

reg [3-1:0] rd_index_in;
 // L2_CAM_MSHR
reg [3-1:0] wr_index_in;

reg [8-1:0] addr_in;
 // L2_CAM_MSHR



always @ *
begin
    if (pipe_rd_sel)
    begin
        rd_en = rd_en2;
        cam_en = cam_en2;
        pending_ready = pending_ready2;
        rd_index_in = rd_index_in2;
        addr_in = addr_in2;
    end
    else
    begin
        rd_en = rd_en1;
        cam_en = cam_en1;
        pending_ready = pending_ready1;
        rd_index_in = rd_index_in1;
        addr_in = addr_in1;
    end
end
 // L2_CAM_MSHR

always @ *
begin
    if (pipe_wr_sel)
    begin
        wr_state_en = wr_state_en2;
        wr_data_en = wr_data_en2;
        state_in = state_in2;
        data_in = data_in2;
        data_mask_in = data_mask_in2;
        wr_index_in = wr_index_in2;
    end
    else
    begin
        wr_state_en = wr_state_en1;
        wr_data_en = wr_data_en1;
        state_in = state_in1;
        data_in = data_in1;
        data_mask_in = data_mask_in1;
        wr_index_in = wr_index_in1;
    end
end



l2_mshr l2_mshr(
    .clk                        (clk),
    .rst_n                      (rst_n),
    .rd_en                      (rd_en),
    .cam_en                     (cam_en),
    .wr_state_en                (wr_state_en),
    .wr_data_en                 (wr_data_en),
    .pending_ready              (pending_ready),
    .inc_counter_en             (inc_counter_en2),
    .state_in                   (state_in),
    .data_in                    (data_in),
    .data_mask_in               (data_mask_in),
    .rd_index_in                (rd_index_in),
    .inv_counter_rd_index_in    (inv_counter_rd_index_in1),
    .wr_index_in                (wr_index_in),
    .addr_in                    (addr_in),
    .hit                        (hit),
    .hit_index                  (hit_index),
    .state_out                  (state_out),
    .data_out                   (data_out),
    .inv_counter_out            (inv_counter_out), 
    .empty_slots                (empty_slots),
    .pending                    (pending),
    .pending_index              (pending_index),
    .empty_index                (empty_index)
);
















































































































































































































































































































































































































 // L2_CAM_MSHR

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
//  Filename      : l2_pipe1_buf_in.v
//  Created On    : 2014-04-06
//  Revision      :
//  Author        : Yaosheng Fu
//  Company       : Princeton University
//  Email         : yfu@princeton.edu
//
//  Description   : Input buffer for pipeline1
//
//
//==================================================================================================

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
//  Filename      : l2.h.pyv
//  Created On    : 2014-02-20
//  Revision      :
//  Author        : Yaosheng Fu
//  Company       : Princeton University
//  Email         : yfu@princeton.edu
//
//  Description   : header file for the L2 cache
//
//
//==================================================================================================

// devices.xml



// Input buffer for pipeline1













// Input buffer for pipeline2














// Output buffer















// L2 cache configuration







//`define L2_SIZE                  65536
//`define L2_SIZE_WIDTH            16






//`define L2_LINE_SIZE             64
//`define L2_LINE_SIZE_WIDTH       6













//`define L2_WAYS                  4
//`define L2_WAYS_WIDTH            2
//`define L2_WAY_0                 2'b00
//`define L2_WAY_1                 2'b01
//`define L2_WAY_2                 2'b10
//`define L2_WAY_3                 2'b11





// Tag array







//`define L2_TAG_INDEX_WIDTH      8
//`define L2_TAG_WIDTH            26
//`define L2_TAG_WAY_WIDTH        26
//`define L2_TAG_ARRAY_WIDTH      104


// Tag Address decomposition 




//`define L2_TAG_INDEX            13:6
//`define L2_TAG                  39:14


//Data array












//`define L2_DATA_INDEX_WIDTH         12 








// Data Address decomposition 







//Dir array



//`define L2_DIR_INDEX_WIDTH      10 





//State array

















//Whether the cache line is in Icaches or Dcaches
















//Round Robin selection














// State decomposition 












//MSHR array



































//`define L2_MSHR_CMP_ADDR        13:6
//`define L2_MSHR_ADDR            39:0
//`define L2_MSHR_WAY             41:40
//`define L2_MSHR_MSHRID          49:42
//`define L2_MSHR_CACHE_TYPE      50
//`define L2_MSHR_DATA_SIZE       53:51
//`define L2_MSHR_MSG_TYPE        61:54
//`define L2_MSHR_L2_MISS         62
//`define L2_MSHR_SRC_CHIPID      76:63
//`define L2_MSHR_SRC_X           84:77
//`define L2_MSHR_SRC_Y           92:85
//`define L2_MSHR_SRC_FBITS       96:93
//`define L2_MSHR_SDID            106:97
//`define L2_MSHR_LSID            112:107      
//`define L2_MSHR_MISS_LSID       118:113
//`define L2_MSHR_SMC_MISS        119
//`define L2_MSHR_RECYCLED        120
//`define L2_MSHR_INV_FWD_PENDING 121


//SMC array


































//Message destination_type







//L2 public sharer beyond the maximum clump size



//L2 registers



//Special addresses





















//L2 core ID





//L2 dir array reuse







//Control Signal 








//AMO ALU OP macros












//Control Signal in Stage 1























































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





































































































































































































































































































































































































































































































































































































module l2_pipe1_buf_in(

    input wire clk,
    input wire rst_n,

    input wire valid_in,
    input wire [64-1:0] data_in,
    output reg ready_in,

   
    output reg msg_header_valid_out,
    output reg [192-1:0] msg_header_out,
    input wire msg_header_ready_out,


    output reg msg_data_valid_out,
    output reg [64-1:0] msg_data_out,
    input wire msg_data_ready_out
);


localparam msg_data_state0 = 2'd0;
localparam msg_data_state1 = 2'd1;
localparam msg_data_state2 = 2'd2;

localparam msg_state_header0 = 3'd0;
localparam msg_state_header1 = 3'd1;
localparam msg_state_header2 = 3'd2;
localparam msg_state_data0 = 3'd3;
localparam msg_state_data1 = 3'd4;

reg [2:0] msg_state_f;
reg [2:0] msg_state_next;
reg real_ready_in;

reg [1:0] msg_data_state_f;
reg [1:0] msg_data_state_next;

//stall for interrupt packets with only 1 header flit
reg [2:0] msg_int_state_f;
reg [2:0] msg_int_state_next;
reg int_stall;

always @ *
begin
    if (!rst_n)
    begin
        msg_data_state_next = msg_data_state0;
    end
    else if((msg_state_f == msg_state_header0) && valid_in)
    begin
        if (data_in[21:14] == 8'd32)
        begin
            msg_data_state_next = msg_data_state1;
        end
        else
        begin
            msg_data_state_next = data_in[29:22] - 2;
        end
    end
    else
    begin
        msg_data_state_next = msg_data_state_f;
    end
end

always @ (posedge clk)
begin
    msg_data_state_f <= msg_data_state_next;
end

always @ *
begin
    if (!rst_n)
    begin
        msg_state_next = msg_state_header0;
    end
    else if (valid_in && real_ready_in)
    begin
        if ((msg_state_f == msg_state_header2) && (msg_data_state_f == msg_data_state0))
        begin
            msg_state_next = msg_state_header0;
        end
        else if ((msg_state_f == msg_state_data0) && (msg_data_state_f == msg_data_state1))
        begin
            msg_state_next = msg_state_header0;
        end
        else
        begin
            if (msg_state_f == msg_state_data1)
            begin
                msg_state_next = msg_state_header0;
            end
            else
            begin
                msg_state_next = msg_state_f + 3'd1;
            end
        end
    end
    else
    begin
        msg_state_next = msg_state_f;
    end 
end


always @ (posedge clk)
begin
    msg_state_f <= msg_state_next;
end

always @ *
begin
    if (!rst_n)
    begin
        msg_int_state_next = msg_state_header0;
    end
    else if (valid_in && real_ready_in && (data_in[21:14] == 8'd32) 
         && (msg_state_f == msg_state_header0))
    begin
        msg_int_state_next = msg_state_header1;
    end
    else if (valid_in && real_ready_in && (msg_int_state_f == msg_state_header1))
    begin
        msg_int_state_next = msg_state_header2;
    end
    else if (valid_in && real_ready_in && (msg_int_state_f == msg_state_header2))
    begin
        msg_int_state_next = msg_state_header0;
    end
    else
    begin
        msg_int_state_next = msg_int_state_f;
    end 
end

always @ (posedge clk)
begin
    msg_int_state_f <= msg_int_state_next;
end


always @ *
begin
    if ((valid_in && (data_in[21:14] == 8'd32) && (msg_state_f == msg_state_header0))
    || (msg_int_state_f == msg_state_header1))
    begin
        int_stall = 1'b1;
    end
    else
    begin
        int_stall = 1'b0;
    end
end




localparam msg_mux_header = 1'b0;
localparam msg_mux_data = 1'b1;

reg msg_mux_sel;

always @ *
begin
    if ((msg_state_f == msg_state_header0)
    ||  (msg_state_f == msg_state_header1)
    ||  (msg_state_f == msg_state_header2))
    begin
        msg_mux_sel = msg_mux_header;
    end
    else
        msg_mux_sel = msg_mux_data;
end


reg msg_header_valid_in;
reg [64-1:0] msg_header_in;
reg msg_header_ready_in;


reg msg_data_valid_in;
reg [64-1:0] msg_data_in;
reg msg_data_ready_in;


always @ *
begin
    if (msg_mux_sel == msg_mux_header)
    begin
        msg_header_valid_in = valid_in;
        msg_header_in = data_in;
    end
    else
    begin
        msg_header_valid_in = 0;
        msg_header_in = 0;
    end
end

always @ *
begin
    if (msg_mux_sel == msg_mux_data)
    begin
        msg_data_valid_in = valid_in;
        msg_data_in = data_in;
    end
    else
    begin
        msg_data_valid_in = 0;
        msg_data_in = 0;
    end
end

always @ *
begin
    if (msg_mux_sel == msg_mux_data)
    begin
        real_ready_in = msg_data_ready_in; 
    end
    else
    begin
        real_ready_in = msg_header_ready_in;
    end
end

always @ *
begin
    ready_in = real_ready_in && (!int_stall);
end


reg [64-1:0] header_buf_mem_f [8-1:0];
reg header_buf_empty;
reg header_buf_full;
reg [3:0] header_buf_counter_f;
reg [3:0] header_buf_counter_next;
reg [3-1:0] header_rd_ptr_f;
reg [3-1:0] header_rd_ptr_next;
reg [3-1:0] header_rd_ptr_plus1;
reg [3-1:0] header_rd_ptr_plus2;
reg [3-1:0] header_wr_ptr_f;
reg [3-1:0] header_wr_ptr_next;


always @ *
begin
    header_buf_empty = (header_buf_counter_f == 0);
    header_buf_full = (header_buf_counter_f ==  8);
end

always @ *
begin
    if ((msg_header_valid_in && msg_header_ready_in) && (msg_header_valid_out && msg_header_ready_out))
    begin
        header_buf_counter_next = header_buf_counter_f + 1 - 3;
    end
    else if (msg_header_valid_in && msg_header_ready_in)
    begin 
        header_buf_counter_next = header_buf_counter_f + 1;
    end
    else if (msg_header_valid_out && msg_header_ready_out)
    begin 
        header_buf_counter_next = header_buf_counter_f - 3;
    end
    else
    begin
        header_buf_counter_next = header_buf_counter_f;
    end
end


always @ (posedge clk)
begin
    if (!rst_n)
    begin
        header_buf_counter_f <= 0;
    end
    else
    begin
        header_buf_counter_f <= header_buf_counter_next;
    end
end

always @ *
begin
    if (!rst_n)
    begin   
        header_rd_ptr_next = 0;
    end
    else if (msg_header_valid_out && msg_header_ready_out)
    begin
        header_rd_ptr_next = header_rd_ptr_f + 3;
    end
    else
    begin
        header_rd_ptr_next = header_rd_ptr_f;
    end
end

always @ (posedge clk)
begin
    header_rd_ptr_f <= header_rd_ptr_next;
end

always @ *
begin
    if (!rst_n)
    begin   
        header_wr_ptr_next = 0;
    end
    else if (msg_header_valid_in && msg_header_ready_in)
    begin
        header_wr_ptr_next = header_wr_ptr_f + 1;
    end
    else
    begin
        header_wr_ptr_next = header_wr_ptr_f;
    end
end


always @ (posedge clk)
begin
    header_wr_ptr_f <= header_wr_ptr_next;
end


always @ *
begin
    header_rd_ptr_plus1 = header_rd_ptr_f + 1;
    header_rd_ptr_plus2 = header_rd_ptr_f + 2;
end


always @ *
begin
   msg_header_ready_in = !header_buf_full;
end


always @ *
begin
    msg_header_valid_out = (header_buf_counter_f >= 3);
    msg_header_out = {header_buf_mem_f[header_rd_ptr_plus2], 
                      header_buf_mem_f[header_rd_ptr_plus1], 
                      header_buf_mem_f[header_rd_ptr_f]};
end


always @ (posedge clk)
begin
    if (!rst_n)
    begin   
        header_buf_mem_f[0] <= 0;
        header_buf_mem_f[1] <= 0;
        header_buf_mem_f[2] <= 0;
        header_buf_mem_f[3] <= 0;
        header_buf_mem_f[4] <= 0;
        header_buf_mem_f[5] <= 0;
        header_buf_mem_f[6] <= 0;
        header_buf_mem_f[7] <= 0;

    end
    else if (msg_header_valid_in && msg_header_ready_in)
    begin
        header_buf_mem_f[header_wr_ptr_f] <= msg_header_in;
    end
    else
    begin 
        header_buf_mem_f[header_wr_ptr_f] <= header_buf_mem_f[header_wr_ptr_f];
    end
end




reg [64-1:0] data_buf_mem_f [4-1:0];
reg data_buf_empty;
reg data_buf_full;
reg [2:0] data_buf_counter_f;
reg [2:0] data_buf_counter_next;
reg [2-1:0] data_rd_ptr_f;
reg [2-1:0] data_rd_ptr_next;
reg [2-1:0] data_wr_ptr_f;
reg [2-1:0] data_wr_ptr_next;

always @ *
begin
    data_buf_empty = (data_buf_counter_f == 0);
    data_buf_full = (data_buf_counter_f ==  4);
end

always @ *
begin
    if (!rst_n)
    begin
        data_buf_counter_next = 0;
    end
    else if ((msg_data_valid_in && msg_data_ready_in) && (msg_data_valid_out && msg_data_ready_out))
    begin
        data_buf_counter_next = data_buf_counter_f + 1 - 1;
    end
    else if (msg_data_valid_in && msg_data_ready_in)
    begin 
        data_buf_counter_next = data_buf_counter_f + 1;
    end
    else if (msg_data_valid_out && msg_data_ready_out)
    begin 
        data_buf_counter_next = data_buf_counter_f - 1;
    end
    else
    begin
        data_buf_counter_next = data_buf_counter_f;
    end
end

always @ (posedge clk)
begin
    data_buf_counter_f <= data_buf_counter_next;
end

always @ *
begin
    if (!rst_n)
    begin   
        data_rd_ptr_next = 0;
    end
    else if (msg_data_valid_out && msg_data_ready_out)
    begin
        data_rd_ptr_next = data_rd_ptr_f + 1;
    end
    else
    begin
        data_rd_ptr_next = data_rd_ptr_f;
    end
end


always @ (posedge clk)
begin
    data_rd_ptr_f <= data_rd_ptr_next;
end

always @ *
begin
    if (!rst_n)
    begin   
        data_wr_ptr_next = 0;
    end
    else if (msg_data_valid_in && msg_data_ready_in)
    begin
        data_wr_ptr_next = data_wr_ptr_f + 1;
    end
    else
    begin
        data_wr_ptr_next = data_wr_ptr_f;
    end
end

always @ (posedge clk)
begin
    data_wr_ptr_f <= data_wr_ptr_next;
end



always @ *
begin
   msg_data_ready_in = !data_buf_full;
end


always @ *
begin
    msg_data_valid_out = !data_buf_empty;
    msg_data_out = data_buf_mem_f[data_rd_ptr_f]; 
end


always @ (posedge clk)
begin
    if (!rst_n)
    begin   
        data_buf_mem_f[0] <= 0;
        data_buf_mem_f[1] <= 0;
        data_buf_mem_f[2] <= 0;
        data_buf_mem_f[3] <= 0;

    end
    else if (msg_data_valid_in && msg_data_ready_in)
    begin
        data_buf_mem_f[data_wr_ptr_f] <= msg_data_in;
    end
    else
    begin 
        data_buf_mem_f[data_wr_ptr_f] <= data_buf_mem_f[data_wr_ptr_f];
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

//==================================================================================================
//  Filename      : l2_pipe1_buf_out.v
//  Created On    : 2014-04-06
//  Revision      :
//  Author        : Yaosheng Fu
//  Company       : Princeton University
//  Email         : yfu@princeton.edu
//
//  Description   : Output buffer for pipeline1
//
//
//==================================================================================================

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
//  Filename      : l2.h.pyv
//  Created On    : 2014-02-20
//  Revision      :
//  Author        : Yaosheng Fu
//  Company       : Princeton University
//  Email         : yfu@princeton.edu
//
//  Description   : header file for the L2 cache
//
//
//==================================================================================================

// devices.xml



// Input buffer for pipeline1













// Input buffer for pipeline2














// Output buffer















// L2 cache configuration







//`define L2_SIZE                  65536
//`define L2_SIZE_WIDTH            16






//`define L2_LINE_SIZE             64
//`define L2_LINE_SIZE_WIDTH       6













//`define L2_WAYS                  4
//`define L2_WAYS_WIDTH            2
//`define L2_WAY_0                 2'b00
//`define L2_WAY_1                 2'b01
//`define L2_WAY_2                 2'b10
//`define L2_WAY_3                 2'b11





// Tag array







//`define L2_TAG_INDEX_WIDTH      8
//`define L2_TAG_WIDTH            26
//`define L2_TAG_WAY_WIDTH        26
//`define L2_TAG_ARRAY_WIDTH      104


// Tag Address decomposition 




//`define L2_TAG_INDEX            13:6
//`define L2_TAG                  39:14


//Data array












//`define L2_DATA_INDEX_WIDTH         12 








// Data Address decomposition 







//Dir array



//`define L2_DIR_INDEX_WIDTH      10 





//State array

















//Whether the cache line is in Icaches or Dcaches
















//Round Robin selection














// State decomposition 












//MSHR array



































//`define L2_MSHR_CMP_ADDR        13:6
//`define L2_MSHR_ADDR            39:0
//`define L2_MSHR_WAY             41:40
//`define L2_MSHR_MSHRID          49:42
//`define L2_MSHR_CACHE_TYPE      50
//`define L2_MSHR_DATA_SIZE       53:51
//`define L2_MSHR_MSG_TYPE        61:54
//`define L2_MSHR_L2_MISS         62
//`define L2_MSHR_SRC_CHIPID      76:63
//`define L2_MSHR_SRC_X           84:77
//`define L2_MSHR_SRC_Y           92:85
//`define L2_MSHR_SRC_FBITS       96:93
//`define L2_MSHR_SDID            106:97
//`define L2_MSHR_LSID            112:107      
//`define L2_MSHR_MISS_LSID       118:113
//`define L2_MSHR_SMC_MISS        119
//`define L2_MSHR_RECYCLED        120
//`define L2_MSHR_INV_FWD_PENDING 121


//SMC array


































//Message destination_type







//L2 public sharer beyond the maximum clump size



//L2 registers



//Special addresses





















//L2 core ID





//L2 dir array reuse







//Control Signal 








//AMO ALU OP macros












//Control Signal in Stage 1























































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





































































































































































































































































































































































































































































































































































































module l2_pipe1_buf_out(

    input wire clk,
    input wire rst_n,

    input wire [3-1:0] mode_in,
    input wire valid_in,
    input wire [320 -1:0] data_in,
    output reg ready_in,

   
    output reg valid_out,
    output reg [64-1:0] data_out,
    input wire ready_out

);



reg [64-1:0] buf_mem_f [16-1:0];
reg buf_empty;
reg buf_full;
reg [4:0] buf_counter_f;
reg [4:0] buf_counter_next;
reg [4-1:0] rd_ptr_f;
reg [4-1:0] rd_ptr_next;
reg [4-1:0] wr_ptr_f;
reg [4-1:0] wr_ptr_next;
reg [4-1:0] wr_ptr_plus1;
reg [4-1:0] wr_ptr_plus2;
reg [4-1:0] wr_ptr_plus3;
reg [4-1:0] wr_ptr_plus4;
reg [4:0] buf_rd_flits;

always @ *
begin
    if (mode_in == 3'd1)
    begin
        buf_rd_flits = 1;
    end
    else if (mode_in == 3'd2)
    begin
        buf_rd_flits = 2;
    end
    else if (mode_in == 3'd3)
    begin
        buf_rd_flits = 3;
    end
    else if (mode_in == 3'd4)
    begin
        buf_rd_flits = 3;
    end
    else if (mode_in == 3'd5)
    begin
        buf_rd_flits = 4;
    end
    else if (mode_in == 3'd6)
    begin
        buf_rd_flits = 5;
    end
    else if (mode_in == 3'd7)
    begin
        buf_rd_flits = 2;
    end
    else
    begin
        buf_rd_flits = 0;
    end
end



always @ *
begin
   buf_empty = (buf_counter_f == 0);
   buf_full = (buf_counter_f ==  16);
end

always @ *
begin
    if (!rst_n)
    begin
        buf_counter_next = 0;
    end
    else if ((valid_in && ready_in) && (valid_out && ready_out))
    begin
        buf_counter_next = buf_counter_f + buf_rd_flits - 1;
    end
    else if (valid_in && ready_in)
    begin 
        buf_counter_next = buf_counter_f + buf_rd_flits;
    end
    else if (valid_out && ready_out)
    begin 
        buf_counter_next = buf_counter_f - 1;
    end
    else
    begin
        buf_counter_next = buf_counter_f;
    end
end



always @ (posedge clk)
begin
    buf_counter_f <= buf_counter_next;
end


always @ *
begin
    if (!rst_n)
    begin   
        rd_ptr_next = 0;
    end
    else if (valid_out && ready_out)
    begin
        rd_ptr_next = rd_ptr_f + 1;
    end
    else
    begin
        rd_ptr_next = rd_ptr_f;
    end
end



always @ (posedge clk)
begin
    rd_ptr_f <= rd_ptr_next;
end

always @ *
begin
    if (!rst_n)
    begin   
        wr_ptr_next = 0;
    end
    else if (valid_in && ready_in)
    begin
        wr_ptr_next = wr_ptr_f + buf_rd_flits;
    end
    else
    begin
        wr_ptr_next = wr_ptr_f;
    end
end

always @ (posedge clk)
begin
    wr_ptr_f <= wr_ptr_next;
end


always @ *
begin
    wr_ptr_plus1 = wr_ptr_f + 1;
    wr_ptr_plus2 = wr_ptr_f + 2;
    wr_ptr_plus3 = wr_ptr_f + 3;
    wr_ptr_plus4 = wr_ptr_f + 4;
end


always @ *
begin
    ready_in = (buf_counter_f <= 16 - buf_rd_flits);
end


always @ *
begin
    valid_out = !buf_empty;
    data_out = buf_mem_f[rd_ptr_f]; 
end



always @ (posedge clk)
begin
    if (!rst_n)
    begin  
        buf_mem_f[0] <= 0;
        buf_mem_f[1] <= 0;
        buf_mem_f[2] <= 0;
        buf_mem_f[3] <= 0;
        buf_mem_f[4] <= 0;
        buf_mem_f[5] <= 0;
        buf_mem_f[6] <= 0;
        buf_mem_f[7] <= 0;
        buf_mem_f[8] <= 0;
        buf_mem_f[9] <= 0;
        buf_mem_f[10] <= 0;
        buf_mem_f[11] <= 0;
        buf_mem_f[12] <= 0;
        buf_mem_f[13] <= 0;
        buf_mem_f[14] <= 0;
        buf_mem_f[15] <= 0;

    end
    else if (valid_in && ready_in)
    begin
        if (mode_in == 3'd1)
        begin
            buf_mem_f[wr_ptr_f] <= data_in[64-1:0];
        end
        else if (mode_in == 3'd2)
        begin
            buf_mem_f[wr_ptr_f] <= data_in[64-1:0];
            buf_mem_f[wr_ptr_plus1] <= data_in[64*4-1:64*3];
        end
        else if (mode_in == 3'd7)
        begin
            buf_mem_f[wr_ptr_f] <= data_in[64*4-1:64*3];
            buf_mem_f[wr_ptr_plus1] <= data_in[64*5-1:64*4];
        end
        else if (mode_in == 3'd4)
        begin
            buf_mem_f[wr_ptr_f] <= data_in[64-1:0];
            buf_mem_f[wr_ptr_plus1] <= data_in[64*2-1:64];
            buf_mem_f[wr_ptr_plus2] <= data_in[64*3-1:64*2];
        end
        else if (mode_in == 3'd3)
        begin
            buf_mem_f[wr_ptr_f] <= data_in[64-1:0];
            buf_mem_f[wr_ptr_plus1] <= data_in[64*4-1:64*3];
            buf_mem_f[wr_ptr_plus2] <= data_in[64*5-1:64*4];
        end
        else if (mode_in == 3'd5)
        begin
            buf_mem_f[wr_ptr_f] <= data_in[64-1:0];
            buf_mem_f[wr_ptr_plus1] <= data_in[64*2-1:64];
            buf_mem_f[wr_ptr_plus2] <= data_in[64*3-1:64*2];
            buf_mem_f[wr_ptr_plus3] <= data_in[64*4-1:64*3];
        end
        else if (mode_in == 3'd6)
        begin
            buf_mem_f[wr_ptr_f] <= data_in[64-1:0];
            buf_mem_f[wr_ptr_plus1] <= data_in[64*2-1:64];
            buf_mem_f[wr_ptr_plus2] <= data_in[64*3-1:64*2];
            buf_mem_f[wr_ptr_plus3] <= data_in[64*4-1:64*3];
            buf_mem_f[wr_ptr_plus4] <= data_in[64*5-1:64*4];
        end
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

//==================================================================================================
//  Filename      : l2_pipe1_ctrl.v
//  Created On    : 2014-02-24
//  Revision      :
//  Author        : Yaosheng Fu
//  Company       : Princeton University
//  Email         : yfu@princeton.edu
//
//  Description   : The control unit for pipeline1 in the L2 cache
//
//
//====================================================================================================

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
//  Filename      : l2.h.pyv
//  Created On    : 2014-02-20
//  Revision      :
//  Author        : Yaosheng Fu
//  Company       : Princeton University
//  Email         : yfu@princeton.edu
//
//  Description   : header file for the L2 cache
//
//
//==================================================================================================

// devices.xml



// Input buffer for pipeline1













// Input buffer for pipeline2














// Output buffer















// L2 cache configuration







//`define L2_SIZE                  65536
//`define L2_SIZE_WIDTH            16






//`define L2_LINE_SIZE             64
//`define L2_LINE_SIZE_WIDTH       6













//`define L2_WAYS                  4
//`define L2_WAYS_WIDTH            2
//`define L2_WAY_0                 2'b00
//`define L2_WAY_1                 2'b01
//`define L2_WAY_2                 2'b10
//`define L2_WAY_3                 2'b11





// Tag array







//`define L2_TAG_INDEX_WIDTH      8
//`define L2_TAG_WIDTH            26
//`define L2_TAG_WAY_WIDTH        26
//`define L2_TAG_ARRAY_WIDTH      104


// Tag Address decomposition 




//`define L2_TAG_INDEX            13:6
//`define L2_TAG                  39:14


//Data array












//`define L2_DATA_INDEX_WIDTH         12 








// Data Address decomposition 







//Dir array



//`define L2_DIR_INDEX_WIDTH      10 





//State array

















//Whether the cache line is in Icaches or Dcaches
















//Round Robin selection














// State decomposition 












//MSHR array



































//`define L2_MSHR_CMP_ADDR        13:6
//`define L2_MSHR_ADDR            39:0
//`define L2_MSHR_WAY             41:40
//`define L2_MSHR_MSHRID          49:42
//`define L2_MSHR_CACHE_TYPE      50
//`define L2_MSHR_DATA_SIZE       53:51
//`define L2_MSHR_MSG_TYPE        61:54
//`define L2_MSHR_L2_MISS         62
//`define L2_MSHR_SRC_CHIPID      76:63
//`define L2_MSHR_SRC_X           84:77
//`define L2_MSHR_SRC_Y           92:85
//`define L2_MSHR_SRC_FBITS       96:93
//`define L2_MSHR_SDID            106:97
//`define L2_MSHR_LSID            112:107      
//`define L2_MSHR_MISS_LSID       118:113
//`define L2_MSHR_SMC_MISS        119
//`define L2_MSHR_RECYCLED        120
//`define L2_MSHR_INV_FWD_PENDING 121


//SMC array


































//Message destination_type







//L2 public sharer beyond the maximum clump size



//L2 registers



//Special addresses





















//L2 core ID





//L2 dir array reuse







//Control Signal 








//AMO ALU OP macros












//Control Signal in Stage 1























































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





































































































































































































































































































































































































































































































































































































module l2_pipe1_ctrl(

    input wire clk,
    input wire rst_n,
    
    input wire csm_en,
    
    //Inputs to Stage 1

    input wire pipe2_valid_S1,
    input wire pipe2_valid_S2,
    input wire pipe2_valid_S3,
    input wire [8-1:0] pipe2_msg_type_S1,
    input wire [8-1:0] pipe2_msg_type_S2,
    input wire [8-1:0] pipe2_msg_type_S3,
    input wire [40-1:0] pipe2_addr_S1,
    input wire [40-1:0] pipe2_addr_S2,
    input wire [40-1:0] pipe2_addr_S3,

    //global stall from pipeline2
    input wire global_stall_S1,

    //input msg from the input buffer
    input wire msg_header_valid_S1,
    input wire [8-1:0] msg_type_S1,
    input wire [3-1:0] msg_data_size_S1,
    input wire [1-1:0] msg_cache_type_S1,

    //input from the mshr
    input wire mshr_hit_S1,

    input wire [8-1:0] mshr_msg_type_S1,
    input wire [1-1:0] mshr_l2_miss_S1,
    input wire [3-1:0] mshr_data_size_S1,
    input wire [1-1:0] mshr_cache_type_S1,
 // L2_CAM_MSHR
    input wire mshr_pending_S1,
    input wire [3-1:0] mshr_pending_index_S1,
    input wire [3:0] mshr_empty_slots_S1,
    

    input wire mshr_smc_miss_S1,
 // L2_CAM_MSHR
    



















 // L2_CAM_MSHR

    //data valid signal from the input buffer
    input wire msg_data_valid_S1,

    input wire [40-1:0] addr_S1,


    //Inputs to Stage 2

    //global stall from pipeline2
    input wire global_stall_S2,

    //tag and state info from dpath
    input wire l2_tag_hit_S2,
    input wire l2_evict_S2,
    input wire l2_wb_S2,
    input wire [2-1:0] l2_way_state_mesi_S2,
    input wire [2-1:0] l2_way_state_vd_S2,
    input wire [1-1:0] l2_way_state_cache_type_S2,
    input wire [4-1:0] l2_way_state_subline_S2,
    input wire req_from_owner_S2,
    input wire addr_l2_aligned_S2,
    input wire [6-1:0] lsid_S2,

    //data valid signal from the input buffer
    input wire msg_data_valid_S2,


    input wire [40-1:0] addr_S2,

    //Inputs to Stage 3

    //global stall from pipeline2
    //input wire global_stall_S3,
    //sharer list from the directory array
    input wire [64-1:0] dir_data_S3,
    input wire [40-1:0] addr_S3,


    //Inputs to Stage 4
    //global stall from pipeline2
    input wire global_stall_S4,

    //input signals from the mshr
    input wire [3-1:0] mshr_empty_index_S4,

    //pipelined tag and state info from dpath
    input wire l2_tag_hit_S4,
    input wire l2_evict_S4,
    input wire [2-1:0] l2_way_state_mesi_S4,
    input wire [6-1:0] l2_way_state_owner_S4,
    input wire [2-1:0] l2_way_state_vd_S4,
    input wire [4-1:0] l2_way_state_subline_S4,
    input wire [1-1:0] l2_way_state_cache_type_S4,
    input wire [8-1:0] mshrid_S4,
    input wire req_from_owner_S4,
    input wire [6-1:0] mshr_miss_lsid_S4,
    input wire [6-1:0] lsid_S4,

    
    input wire broadcast_counter_zero_S4,
    input wire broadcast_counter_max_S4,
    input wire broadcast_counter_avail_S4,
    input wire [14-1:0] broadcast_chipid_out_S4,
    input wire [8-1:0] broadcast_x_out_S4,
    input wire [8-1:0] broadcast_y_out_S4,
    

    input wire [40-1:0] addr_S4,
    //comparing result of CAS requests
    input wire cas_cmp_S4,

    //ready signal from the output buffer
    input wire msg_send_ready_S4,

    //inputs from the smc
    
    input wire smc_hit_S4,
    

    //Outputs from Stage 1

    output reg valid_S1,
    output reg stall_S1,
    output reg msg_from_mshr_S1,
    output reg dis_flush_S1,

    output reg mshr_cam_en_S1,
    output reg mshr_pending_ready_S1,

    output reg msg_header_ready_S1,

    output reg tag_clk_en_S1,
    output reg tag_rdw_en_S1,

    output reg state_rd_en_S1,

    output reg reg_wr_en_S1,
    output reg [8-1:0] reg_wr_addr_type_S1,


    //Outputs from Stage 2

    output reg valid_S2,
    output reg stall_S2,
    output reg stall_before_S2,
    output reg stall_real_S2,
    output reg [8-1:0] msg_type_S2,

    output reg msg_from_mshr_S2,
    output reg special_addr_type_S2,
    output wire state_load_sdid_S2,

    output reg dir_clk_en_S2,
    output reg dir_rdw_en_S2,
    output reg [2-1:0] dir_op_S2,


    output reg data_clk_en_S2,
    output reg data_rdw_en_S2,
    output reg [4-1:0] amo_alu_op_S2,

    output reg [3-1:0] data_size_S2,
    output reg [1-1:0] cache_type_S2,
    output reg state_owner_en_S2,
    output reg [2-1:0] state_owner_op_S2,
    output reg state_subline_en_S2,
    output reg [2-1:0] state_subline_op_S2,
    output reg state_di_en_S2,
    output reg state_vd_en_S2,
    output reg [2-1:0] state_vd_S2,
    output reg state_mesi_en_S2,
    output reg [2-1:0] state_mesi_S2,
    output reg state_lru_en_S2,
    output reg [1-1:0] state_lru_op_S2,
    output reg state_rb_en_S2,

    output reg [2-1:0] l2_load_data_subline_S2,
    output reg l2_ifill_S2,

    output reg msg_data_ready_S2,

    
    output reg smc_wr_en_S2,
    output reg smc_wr_diag_en_S2,
    output reg smc_flush_en_S2,
    output reg [2-1:0] smc_addr_op_S2,
    
    //Outputs from Stage 3

    output reg valid_S3,
    output reg stall_S3,
    output reg stall_before_S3,

    //Outputs from Stage 4

    output reg valid_S4,
    output reg stall_S4,
    output reg stall_before_S4,


    output reg [8-1:0] msg_type_S4,
    output reg [3-1:0] data_size_S4,
    output reg [1-1:0] cache_type_S4,
    output reg [1-1:0] l2_miss_S4,
    
    output reg smc_miss_S4,
    output reg stall_smc_buf_S4,
    
    output reg  msg_from_mshr_S4,
    output reg req_recycle_S4,
    output reg inv_fwd_pending_S4,

    output wire [6-1:0] dir_sharer_S4,
    output reg [6-1:0] dir_sharer_counter_S4,
    output reg [64-1:0] dir_data_sel_S4,
    output reg cas_cmp_en_S4,
    output reg atomic_read_data_en_S4,
    output reg [3-1:0] cas_cmp_data_size_S4,
    output reg [64-1:0] dir_data_S4,

    output reg msg_send_valid_S4,
    output reg [3-1:0] msg_send_mode_S4,
    output reg [8-1:0] msg_send_type_S4,
    output reg [8-1:0] msg_send_type_pre_S4,

    output reg [8-1:0] msg_send_length_S4,
    output reg [3-1:0] msg_send_data_size_S4,
    output reg [1-1:0] msg_send_cache_type_S4,
    output reg [2-1:0] msg_send_mesi_S4,
    output reg [1-1:0] msg_send_l2_miss_S4,
    output reg [8-1:0] msg_send_mshrid_S4,
    output reg [4-1:0] msg_send_subline_vector_S4,
    output reg special_addr_type_S4,

    output reg mshr_wr_data_en_S4,
    output reg mshr_wr_state_en_S4,
    output reg [2-1:0] mshr_state_in_S4,
    output reg [3-1:0] mshr_inv_counter_rd_index_in_S4,
    output reg [3-1:0] mshr_wr_index_in_S4,

    output reg state_wr_sel_S4,
    output reg state_wr_en_S4,

    
    output reg [2-1:0] broadcast_counter_op_S4,
    output reg broadcast_counter_op_val_S4,
    

    
    output reg smc_rd_diag_en_buf_S4,
    output reg smc_rd_en_buf_S4,
    

    output reg l2_access_valid_S4,
    output reg l2_miss_valid_S4,
    output reg reg_rd_en_S4,
    output reg [8-1:0] reg_rd_addr_type_S4

);




localparam y = 1'b1;
localparam n = 1'b0;


localparam rd = 1'b1;
localparam wr = 1'b0;


// pre-declare
reg [8-1:0] msg_type_S2_f;
reg msg_from_mshr_S2_f;
reg [8-1:0] msg_type_S4_f;

//============================
// Stage 1
//============================

reg stall_pre_S1;
reg stall_hazard_S1;
reg [8-1:0] msg_type_mux_S1;
reg [8-1:0] msg_type_trans_S1;
reg [3-1:0] data_size_S1;
reg [1-1:0] cache_type_S1;

reg msg_header_ready_real_S1;
reg msg_cas_cmp_S1_f;
reg msg_cas_cmp_S1_next;
reg msg_input_en_S1_f;
reg msg_input_en_S1_next;
reg [8-1:0] addr_type_S1;
reg [2-1:0] addr_op_S1;
reg special_addr_type_S1;
reg msg_data_rd_S1;

always @ *
begin
    valid_S1 = mshr_pending_S1 || (msg_header_valid_S1 && msg_input_en_S1_f);
end



always @ *
begin
    stall_pre_S1 = stall_S2 || global_stall_S1;
end


always @ *
begin
    mshr_pending_ready_S1 = mshr_pending_S1 && (!stall_S1);
end

always @ *
begin
    msg_from_mshr_S1 = mshr_pending_S1;
end

//msgs from mshr have higher priority than those from the input buffer
always @ *
begin
    if (msg_from_mshr_S1)
    begin

        msg_type_mux_S1 = mshr_msg_type_S1;


 // L2_CAM_MSHR
    end
    else
    begin
        msg_type_mux_S1 = msg_type_S1;
    end
end


always @ *
begin
     //Modified for timing
    //mshr_cam_en_S1 = (!mshr_pending_S1) && msg_header_valid_S1 && (!stall_pre_S1);
    // mshr_cam_en_S1 = (!mshr_pending_S1) && msg_header_valid_S1;
    mshr_cam_en_S1 = (!mshr_pending_S1) && msg_header_valid_S1 && (!global_stall_S1);
    // trinn
end


localparam atomic_state0 = 1'b0;
localparam atomic_state1 = 1'b1;
reg atomic_state_S1_f;
reg atomic_state_S1_next;

always @ *
begin
    if (!rst_n)
    begin
        atomic_state_S1_next = atomic_state0;
    end
    else if (valid_S1 && !stall_S1 && (!msg_from_mshr_S1) &&
       (msg_type_trans_S1 == 8'd6
        || msg_type_trans_S1 == 8'd10
        || msg_type_trans_S1 == 8'd44
        || msg_type_trans_S1 == 8'd45
        || msg_type_trans_S1 == 8'd46
        || msg_type_trans_S1 == 8'd47
        || msg_type_trans_S1 == 8'd48
        || msg_type_trans_S1 == 8'd49
        || msg_type_trans_S1 == 8'd50
        || msg_type_trans_S1 == 8'd51))
    begin
        atomic_state_S1_next = atomic_state1;
    end
    else if (valid_S1 && !stall_S1 && (!msg_from_mshr_S1) &&
            (msg_type_trans_S1 == 8'd7
            || msg_type_trans_S1 == 8'd8
            || msg_type_trans_S1 == 8'd11
            || msg_type_trans_S1 == 8'd52
            || msg_type_trans_S1 == 8'd53
            || msg_type_trans_S1 == 8'd54
            || msg_type_trans_S1 == 8'd55
            || msg_type_trans_S1 == 8'd56
            || msg_type_trans_S1 == 8'd57
            || msg_type_trans_S1 == 8'd58
            || msg_type_trans_S1 == 8'd59))
    begin
        atomic_state_S1_next = atomic_state0;
    end
    else
    begin
        atomic_state_S1_next = atomic_state_S1_f;
    end
end


always @ (posedge clk)
begin
    atomic_state_S1_f <= atomic_state_S1_next;
end

//translate atomic instructions into two subtypes based on two phases
always @ *
begin
    case (msg_type_mux_S1)
    8'd14:
    begin
        case (addr_type_S1)
        8'hac, 8'had, 8'hae, 8'haf:
        begin
            msg_type_trans_S1 = 8'd35;
        end
        8'ha3:
        begin
            msg_type_trans_S1 = 8'd34;
        end
        default:
        begin
            msg_type_trans_S1 = msg_type_mux_S1;
        end
        endcase
    end
    8'd5:
    begin
        if (atomic_state_S1_f == atomic_state0)
        begin
            msg_type_trans_S1 = 8'd6;
        end
        else
        begin
            if (msg_cas_cmp_S1_f)
            begin
                msg_type_trans_S1 = 8'd7;
            end
            else
            begin
                msg_type_trans_S1 = 8'd8;
            end
        end
    end
    8'd9:
    begin
        if (atomic_state_S1_f == atomic_state0)
        begin
            msg_type_trans_S1 = 8'd10;
        end
        else
        begin
            msg_type_trans_S1 = 8'd11;
        end
    end
    8'd36:
    begin
        if (atomic_state_S1_f == atomic_state0)
        begin
            msg_type_trans_S1 = 8'd44;
        end
        else
        begin
            msg_type_trans_S1 = 8'd52;
        end
    end
    8'd37:
    begin
        if (atomic_state_S1_f == atomic_state0)
        begin
            msg_type_trans_S1 = 8'd45;
        end
        else
        begin
            msg_type_trans_S1 = 8'd53;
        end
    end
    8'd38:
    begin
        if (atomic_state_S1_f == atomic_state0)
        begin
            msg_type_trans_S1 = 8'd46;
        end
        else
        begin
            msg_type_trans_S1 = 8'd54;
        end
    end
    8'd39:
    begin
        if (atomic_state_S1_f == atomic_state0)
        begin
            msg_type_trans_S1 = 8'd47;
        end
        else
        begin
            msg_type_trans_S1 = 8'd55;
        end
    end
    8'd40:
    begin
        if (atomic_state_S1_f == atomic_state0)
        begin
            msg_type_trans_S1 = 8'd48;
        end
        else
        begin
            msg_type_trans_S1 = 8'd56;
        end
    end
    8'd41:
    begin
        if (atomic_state_S1_f == atomic_state0)
        begin
            msg_type_trans_S1 = 8'd49;
        end
        else
        begin
            msg_type_trans_S1 = 8'd57;
        end
    end
    8'd42:
    begin
        if (atomic_state_S1_f == atomic_state0)
        begin
            msg_type_trans_S1 = 8'd50;
        end
        else
        begin
            msg_type_trans_S1 = 8'd58;
        end
    end
    8'd43:
    begin
        if (atomic_state_S1_f == atomic_state0)
        begin
            msg_type_trans_S1 = 8'd51;
        end
        else
        begin
            msg_type_trans_S1 = 8'd59;
        end
    end
    default:
    begin
        msg_type_trans_S1 = msg_type_mux_S1;
    end
    endcase
end


always @ *
begin
    dis_flush_S1 = (msg_type_trans_S1 == 8'd35) && ~msg_from_mshr_S1;
end

always @ *
begin
    addr_type_S1 = addr_S1[39:32];
    addr_op_S1 = addr_S1[31:30];
end


always @ *
begin
    if ((msg_type_trans_S1 == 8'd14 || msg_type_trans_S1 == 8'd15)
    &&  (addr_type_S1 == 8'ha0
    ||   addr_type_S1 == 8'ha4
    ||   addr_type_S1 == 8'ha6
    ||   addr_type_S1 == 8'ha1
    ||   addr_type_S1 == 8'ha2
    ||   addr_type_S1 == 8'ha5
    ||   addr_type_S1 == 8'haa
    ||   addr_type_S1 == 8'hab
    ||   addr_type_S1 == 8'ha7
    ||   addr_type_S1 == 8'ha8
    ||   addr_type_S1 == 8'ha9))
    begin
        special_addr_type_S1 = 1'b1;
    end
    else
    begin
        special_addr_type_S1 = 1'b0;
    end
end

always @ *
begin
    if (msg_from_mshr_S1)
    begin

        data_size_S1 = mshr_data_size_S1;


 // L2_CAM_MSHR
    end
    else
    begin
        data_size_S1 = msg_data_size_S1;
    end
end

always @ *
begin
    if (msg_from_mshr_S1)
    begin

        cache_type_S1 = mshr_cache_type_S1;


 // L2_CAM_MSHR
    end
    else
    begin
        cache_type_S1 = msg_cache_type_S1;
    end
end


reg [3-1:0] cs_S1;

always @ *
begin
    cs_S1 = {3{1'bx}};
    if (valid_S1)
    begin
        if (special_addr_type_S1)
        begin
            case (addr_type_S1)
                8'ha4:
                begin
                    if (msg_type_trans_S1 == 8'd14)
                    begin
                        //  tag_clk_en      tag_rdw_en    state_rd_en
                        cs_S1 = {y,             rd,       n};
                    end
                    else
                    begin
                        cs_S1 = {y,             wr,        n};
                    end
                end
                8'ha6:
                begin
                    if (msg_type_trans_S1 == 8'd14)
                    begin
                        //  tag_clk_en      tag_rdw_en   state_rd_en
                        cs_S1 = {y,             rd,      y};
                    end
                    else
                    begin
                        cs_S1 = {n,             rd,       n};
                    end
                end
                default:
                begin
                    cs_S1 = {n,             rd,           n};
                end
            endcase
        end
        else
        begin
            case (msg_type_trans_S1)
                8'd31, 8'd14, 8'd15, 8'd1,
                8'd2,
                8'd60,
                8'd13, 8'd7, 8'd11,
                8'd6, 8'd10, 8'd35,8'd34,
                8'd44, 8'd52,
                8'd45, 8'd53,
                8'd46, 8'd54,
                8'd47, 8'd55,
                8'd48, 8'd56,
                8'd49, 8'd57,
                8'd50, 8'd58,
                8'd51, 8'd59:
                begin
                //  tag_clk_en      tag_rdw_en      state_rd_en
                    cs_S1 = {y,             rd,     y};
                end
                8'd30, 8'd8, 8'd32:
                begin
                    cs_S1 = {n,             rd,          n};
                end
                default:
                begin
                    cs_S1 = {3{1'bx}};
                end
            endcase
        end
    end
    else
    begin
        cs_S1 = {3{1'b0}};
    end
end


//disable inputs from the input buffers for requests with stored data because the stored data cannot be buffered
//if those instructions cause misses
always @ *
begin
    if (!rst_n)
    begin
        msg_input_en_S1_next = y;
    end
    else if ((valid_S1 && !stall_S1)
           &&((msg_type_trans_S1 == 8'd7)
           || (msg_type_trans_S1 == 8'd11)
           || (msg_type_trans_S1 == 8'd52)
           || (msg_type_trans_S1 == 8'd53)
           || (msg_type_trans_S1 == 8'd54)
           || (msg_type_trans_S1 == 8'd55)
           || (msg_type_trans_S1 == 8'd56)
           || (msg_type_trans_S1 == 8'd57)
           || (msg_type_trans_S1 == 8'd58)
           || (msg_type_trans_S1 == 8'd59)
           || (msg_type_trans_S1 == 8'd15))
           && !msg_from_mshr_S1)
    begin
        msg_input_en_S1_next = n;
    end
    else if ((valid_S2 && !stall_S2)
           &&((((msg_type_S2_f == 8'd7)
           || (msg_type_S2_f == 8'd11)
           || (msg_type_S2_f == 8'd52)
           || (msg_type_S2_f == 8'd53)
           || (msg_type_S2_f == 8'd54)
           || (msg_type_S2_f == 8'd55)
           || (msg_type_S2_f == 8'd56)
           || (msg_type_S2_f == 8'd57)
           || (msg_type_S2_f == 8'd58)
           || (msg_type_S2_f == 8'd59))
                && l2_tag_hit_S2 && (l2_way_state_mesi_S2 == 2'b00))
            ||((msg_type_S2_f == 8'd15)
                && ((!l2_tag_hit_S2 && !msg_from_mshr_S2_f) || (l2_tag_hit_S2 && (l2_way_state_mesi_S2 == 2'b00) && (l2_way_state_vd_S2 == 2'b10))))))
    begin
        msg_input_en_S1_next = y;
    end
    else
    begin
        msg_input_en_S1_next = msg_input_en_S1_f;
    end
end


always @ (posedge clk)
begin
    msg_input_en_S1_f <= msg_input_en_S1_next;
end



always @ *
begin
    msg_data_rd_S1 = valid_S1 && (msg_type_trans_S1 == 8'd15)
                  && (addr_type_S1 == 8'ha7
                  ||  addr_type_S1 == 8'ha9
                  ||  addr_type_S1 == 8'ha8
                  ||  addr_type_S1 == 8'haa
                  ||  addr_type_S1 == 8'hab
                  ||  addr_type_S1 == 8'ha4);
end


always @ *
begin
    reg_wr_en_S1 = valid_S1 && ~stall_S1 && (msg_type_trans_S1 == 8'd15)
               && ((addr_type_S1 == 8'ha9)
                || (addr_type_S1 == 8'ha7)
                || (addr_type_S1 == 8'ha8)
                || (addr_type_S1 == 8'haa)
                || (addr_type_S1 == 8'hab));
end

always @ *
begin
    reg_wr_addr_type_S1 = addr_type_S1;
end


//write the comparing result from stage 3
always @ *
begin
    if (!rst_n)
    begin
        msg_cas_cmp_S1_next = n;
    end
    else if (msg_type_S4_f == 8'd6 && cas_cmp_en_S4)
    begin
        if (cas_cmp_S4)
        begin
            msg_cas_cmp_S1_next = y;
        end
        else
        begin
            msg_cas_cmp_S1_next = n;
        end
    end
    else
    begin
        msg_cas_cmp_S1_next = msg_cas_cmp_S1_f;
    end
end


always @ (posedge clk)
begin
    msg_cas_cmp_S1_f <= msg_cas_cmp_S1_next;
end


always @ *
begin
    stall_hazard_S1 = (valid_S2 && (addr_S1[6+8-1:6] == addr_S2[6+8-1:6]))
                   || (valid_S3 && (addr_S1[6+8-1:6] == addr_S3[6+8-1:6]))
                   || (valid_S4 && (addr_S1[6+8-1:6] == addr_S4[6+8-1:6]))
                   || (pipe2_valid_S1 && (addr_S1[39:6] == pipe2_addr_S1[39:6]))
                   || (pipe2_valid_S2 && (addr_S1[39:6] == pipe2_addr_S2[39:6]))
                   || (pipe2_valid_S3 && (addr_S1[39:6] == pipe2_addr_S3[39:6]));
end

reg stall_mshr_S1;

always @ *
begin
    // trinn
    // stall_mshr_S1 = ( (mshr_cam_en_S1 && !global_stall_S1) && mshr_hit_S1)
    stall_mshr_S1 = (mshr_cam_en_S1 && mshr_hit_S1)
                 || (~msg_from_mshr_S1
                 //can be optimized
                 &&((mshr_empty_slots_S1 <= 3 && (valid_S2 || valid_S3 || valid_S4))
                 ||  mshr_empty_slots_S1 == 0));
end

reg stall_msg_S1;

always @ *
begin
    stall_msg_S1 = msg_data_rd_S1 && ~msg_data_valid_S1;
end

always @ *
begin
    stall_S1 = valid_S1 && (stall_pre_S1 || stall_hazard_S1 || stall_mshr_S1 || stall_msg_S1);
end


always @ *
begin
    msg_header_ready_real_S1 = (!stall_S1) && (!msg_from_mshr_S1) && msg_input_en_S1_f;
end

always @ *
begin
    msg_header_ready_S1 = msg_header_ready_real_S1
        && ((msg_type_trans_S1 != 8'd6)
         && (msg_type_trans_S1 != 8'd10)
         && (msg_type_trans_S1 != 8'd44)
         && (msg_type_trans_S1 != 8'd45)
         && (msg_type_trans_S1 != 8'd46)
         && (msg_type_trans_S1 != 8'd47)
         && (msg_type_trans_S1 != 8'd48)
         && (msg_type_trans_S1 != 8'd49)
         && (msg_type_trans_S1 != 8'd50)
         && (msg_type_trans_S1 != 8'd51)
         );
end


always @ *
begin
    //for timing
    tag_clk_en_S1 = valid_S1 && cs_S1[2];
    //tag_clk_en_S1 = valid_S1 && !stall_S1 && cs_S1[`CS_TAG_CLK_EN_S1];
end

always @ *
begin
    //for timing
    tag_rdw_en_S1 = valid_S1 && cs_S1[1];
    //tag_rdw_en_S1 = valid_S1 && !stall_S1 && cs_S1[`CS_TAG_RDW_EN_S1];
end

always @ *
begin
    //for timing
    state_rd_en_S1 = valid_S1 && cs_S1[0];
    //state_rd_en_S1 = valid_S1 && !stall_S1 && cs_S1[`CS_STATE_RD_EN_S1];
end

reg l2_miss_S1;

always @ *
begin
    if (msg_from_mshr_S1)
    begin

        l2_miss_S1 = mshr_l2_miss_S1;


 // L2_CAM_MSHR
    end
    else
    begin
        l2_miss_S1 = 0;
    end
end


reg valid_S1_next;

always @ *
begin
    valid_S1_next = valid_S1 && !stall_S1;
end



//============================
// Stage 1 -> Stage 2
//============================

reg valid_S2_f;
reg [3-1:0] data_size_S2_f;
reg [1-1:0] cache_type_S2_f;
reg [1-1:0] l2_miss_S2_f;

reg mshr_smc_miss_S2_f;

reg [3-1:0] mshr_pending_index_S2_f;
reg special_addr_type_S2_f;
reg msg_data_rd_S2_f;

always @ (posedge clk)
begin
    if (!rst_n)
    begin
        valid_S2_f <= 1'b0;
        msg_type_S2_f <= 0;
        data_size_S2_f <= 0;
        cache_type_S2_f <= 0;
        l2_miss_S2_f <= 0;
        
        mshr_smc_miss_S2_f <= 0;
        
        msg_from_mshr_S2_f <= 1'b0;
        mshr_pending_index_S2_f <= 0;
        special_addr_type_S2_f <= 0;
        msg_data_rd_S2_f <= 0;
    end
    else if (!stall_S2)
    begin
        valid_S2_f <= valid_S1_next;
        msg_type_S2_f <= msg_type_trans_S1;
        data_size_S2_f <= data_size_S1;
        cache_type_S2_f <= cache_type_S1;
        l2_miss_S2_f <= l2_miss_S1;
        

        mshr_smc_miss_S2_f <= mshr_smc_miss_S1;



 // L2_CAM_MSHR
        
        msg_from_mshr_S2_f <= msg_from_mshr_S1;
        mshr_pending_index_S2_f <= mshr_pending_index_S1;
        special_addr_type_S2_f <= special_addr_type_S1;
        msg_data_rd_S2_f <= msg_data_rd_S1;
    end
end



//============================
// Stage 2
//============================

reg stall_pre_S2;
// reg stall_real_S2;
reg stall_before_S2_f;
reg stall_before_S2_next;
reg state_wr_en_S2;
reg [1-1:0] l2_miss_S2;
//re-execute this request
reg req_recycle_S2;
reg req_recycle_cur_S2;
reg req_recycle_buf_S2_f;
reg req_recycle_buf_S2_next;

reg mshr_wr_data_en_S2;
reg mshr_wr_state_en_S2;
reg [2-1:0] mshr_state_in_S2;

reg [8-1:0] addr_type_S2;
reg [2-1:0] addr_op_S2;


always @ *
begin
    valid_S2 = valid_S2_f;
    data_size_S2 = data_size_S2_f;
    cache_type_S2 = cache_type_S2_f;
    msg_from_mshr_S2 = msg_from_mshr_S2_f;
    stall_before_S2 = stall_before_S2_f;
    msg_type_S2 = msg_type_S2_f;
    special_addr_type_S2 = special_addr_type_S2_f;
end

always @ *
begin
    addr_type_S2 = addr_S2[39:32];
    addr_op_S2 = addr_S2[31:30];
end



always @ *
begin
    if (!rst_n)
    begin
        stall_before_S2_next = 0;
    end
    else
    begin
        stall_before_S2_next = stall_S2;
    end
end


always @ (posedge clk)
begin
    stall_before_S2_f <= stall_before_S2_next;
end


always @ *
begin
    stall_pre_S2 = stall_S3 || global_stall_S2;
end



reg [27-1:0] cs_S2;

always @ *
begin
    if (valid_S2)
    begin
        if (special_addr_type_S2_f)
        begin
            case (addr_type_S2)
                8'ha0:
                begin
                    if (msg_type_S2_f == 8'd14)
                    begin
                        //       amo_alu    dir        dir      dir     data    data           mshr     state      state       state       msg
                        //       op         clk_en     rdw_en   op      clk_en  rdw_en         wr_en    owner_en   owner_op    subline_en  data_ready
                        cs_S2 = {4'd0, n,         rd,      2'd0, y,      rd,            n,       n,         2'd0,     n,          n,
                        //       state       state   state   state          state    state           state   state
                        //       subline_op  di_en   vd_en   vd             mesi_en  mesi            lru_en  lru
                                 2'd0,    n,      n,      2'b01,  n,       2'b00, n,      1'b0};
                    end
                    else
                    begin
                        cs_S2 = {4'd0, n,         rd,      2'd0, y,      wr,            n,       n,         2'd0,     n,          y,
                                 2'd0,    n,      n,      2'b01,  n,       2'b00, n,      1'b0};
                    end
                end
                8'ha1:
                begin
                    if (msg_type_S2_f == 8'd14)
                    begin
                        cs_S2 = {4'd0, y,         rd,      2'd0, n,      rd,            n,       n,         2'd0,     n,          n,
                                 2'd0,    n,      n,      2'b01,  n,       2'b00, n,      1'b0};
                    end
                    else
                    begin
                        cs_S2 = {4'd0, y,         wr,      2'd0, n,      rd,            n,       n,         2'd0,     n,          y,
                                 2'd0,    n,      n,      2'b01,  n,       2'b00, n,      1'b0};
                    end
                end
                8'ha6:
                begin
                    if (msg_type_S2_f == 8'd15)
                    begin
                        cs_S2 = {4'd0, n,         rd,      2'd0, n,      rd,            n,       y,         2'd0,     y,          y,
                                 2'd0,    y,      y,      2'b00,  y,       2'b00, y,      1'b0};
                    end
                    else
                    begin
                        cs_S2 = {4'd0, n,         rd,      2'd0, n,      rd,            n,       n,         2'd0,     n,          n,
                                 2'd0,    n,      n,      2'b01,  n,       2'b00, n,      1'b0};
                    end
                end
                8'ha2:
                begin
                    if (msg_type_S2_f == 8'd15)
                    begin
                        cs_S2 = {4'd0, n,         rd,      2'd0, n,      rd,            n,       n,         2'd0,     n,          y,
                                 2'd0,    n,      n,      2'b01,  n,       2'b00, n,      1'b0};
                    end
                    else
                    begin
                        cs_S2 = {4'd0, n,         rd,      2'd0, n,      rd,            n,       n,         2'd0,     n,          n,
                                 2'd0,    n,      n,      2'b01,  n,       2'b00, n,      1'b0};
                    end
                end
                default:
                begin
                    cs_S2 = {4'd0, n,         rd,      2'd0, n,      rd,            n,       n,         2'd0,     n,          n,
                             2'd0,    n,      n,      2'b01,  n,       2'b00, n,      1'b0};
                end
            endcase
        end
        else if (req_recycle_S2)
        begin
            cs_S2 = {27{1'b0}};
        end
        else
        begin
            if ((msg_type_S2_f == 8'd8) || (msg_type_S2_f == 8'd32))
            begin
                cs_S2 = {4'd0, n,         rd,      2'd0, n,      rd,            n,       n,         2'd0,     n,          y,
                         2'd0,    n,      n,      2'b01,  n,       2'b00, n,      1'b0};
            end

            else if (l2_evict_S2)
            begin
                case (l2_way_state_mesi_S2)
                2'b01:
                begin
                    cs_S2 = {4'd0, y,         rd,      2'd0, n,      rd,            y,       n,         2'd0,    n,          n,
                             2'd0,   n,       n,      2'b01,  n,      2'b00, n,      1'b0};
                end
                
                2'b11:
                begin
                    cs_S2 = {4'd0, n,         rd,      2'd0, n,      rd,            y,       n,         2'd0,    n,          n,
                             2'd0,   n,       n,      2'b01,  n,      2'b00, n,      1'b0};
                end
                
                2'b10:
                begin
                    
                    if (csm_en)
                    begin
                        cs_S2 = {4'd0, y,         rd,      2'd0, n,      rd,            y,       n,         2'd0,    n,          n,
                                 2'd0,   n,       n,      2'b01,  n,      2'b00, n,      1'b0};
                    end
                    else
                    
                    begin
                        cs_S2 = {4'd0, n,         rd,      2'd0, n,      rd,            y,       n,         2'd0,    n,          n,
                                 2'd0,   n,       n,      2'b01,  n,      2'b00, n,      1'b0};
                    end
                end
                2'b00:
                begin
                    case (l2_way_state_vd_S2)
                    2'b10:
                    begin
                        if (msg_type_S2_f == 8'd34)
                        begin
                            cs_S2 = {4'd0, n,         rd,      2'd0, n,      rd,            n,       n,         2'd0,    n,          n,
                                    2'd0,   n,       y,      2'b00,  n,      2'b00, y,      1'b0};
                        end
                        else
                        begin
                            cs_S2 = {4'd0, n,         rd,      2'd0, n,      rd,            y,       n,         2'd0,    n,          n,
                                    2'd0,   n,       y,      2'b00,  n,      2'b00, y,      1'b0};
                        end
                    end
                    2'b11:
                    begin
                        if (msg_type_S2_f == 8'd34)
                        begin
                            cs_S2 = {4'd0, n,         rd,      2'd0, y,      rd,            n,       n,         2'd0,    n,          n,
                                    2'd0,   n,       y,      2'b00,  n,      2'b00, y,      1'b0};
                        end
                        else
                        begin
                            cs_S2 = {4'd0, n,         rd,      2'd0, y,      rd,            y,       n,         2'd0,    n,          n,
                                     2'd0,   n,       y,      2'b00,  n,      2'b00, y,      1'b0};
                        end
                    end
                    default:
                    begin
                        cs_S2 = {27{1'bx}};
                    end
                    endcase
                end
                default:
                begin
                    cs_S2 = {27{1'bx}};
                end
                endcase
            end

            else if (!l2_tag_hit_S2)
            begin
                if (msg_type_S2_f == 8'd15)
                begin
                    if (msg_from_mshr_S2_f)
                    begin
                        //       dir        dir      dir     data    data           mshr     state      state       state       msg
                        //       clk_en     rdw_en   op      clk_en  rdw_en         wr_en    owner_en   owner_op    subline_en  data_ready
                        cs_S2 = {4'd0, n,         rd,      2'd0, n,      rd,            n,       n,         2'd0,     n,          n,
                        //       state       state   state   state          state    state           state   state
                        //       subline_op  di_en   vd_en   vd             mesi_en  mesi            lru_en  lru
                                 2'd0,    n,      n,      2'b01,  n,       2'b00, n,      1'b0};
                    end
                    else
                    begin
                        cs_S2 = {4'd0, n,         rd,      2'd0, n,      rd,            y,       n,         2'd0,     n,          y,
                                 2'd0,    n,      n,      2'b01,  n,       2'b00, n,      1'b0};
                    end
                end
                else if (msg_type_S2_f == 8'd13
                 || msg_type_S2_f == 8'd34
                 || msg_type_S2_f == 8'd35)
                begin
                    cs_S2 = {4'd0, n,         rd,      2'd0, n,      rd,            n,       n,         2'd0,    n,          n,
                             2'd0,   n,       n,      2'b00,  n,      2'b00, n,      1'b0};
                end
                else
                begin
                    cs_S2 = {4'd0, n,         rd,      2'd0, n,      rd,            y,       n,         2'd0,    n,          n,
                             2'd0,   n,       n,      2'b00,  n,      2'b00, n,      1'b0};
                end
            end

            else begin
            case (msg_type_S2_f)

                8'd13:
                begin
                    
                    if (csm_en && (l2_way_state_mesi_S2 == 2'b10) && l2_way_state_subline_S2[addr_S2[5:4]])
                    begin
                        cs_S2 = {4'd0, y,         rd,      2'd0, n,      rd,            n,       n,         2'd0,    n,          n,
                                2'd0,   n,       n,      2'b00,  n,      2'b00, n,      1'b0};
                    end
                    else
                    
                    begin
                        cs_S2 = {4'd0, n,         rd,      2'd0, n,      rd,            n,       n,         2'd0,    n,          n,
                                2'd0,   n,       n,      2'b00,  n,      2'b00, n,      1'b0};
                    end
                end
                8'd35:
                begin
                    case (l2_way_state_mesi_S2)
                    2'b00:
                    begin
                        cs_S2 = {4'd0, n,         rd,      2'd0, y,      rd,            n,       n,         2'd0,    n,          n,
                                2'd0,   n,       y,      2'b00,  n,      2'b00, y,      1'b0};
                    end
                    2'b01:
                    begin
                        cs_S2 = {4'd0, y,         rd,      2'd0, n,      rd,            y,       n,         2'd0,    n,      n,
                                 2'd0,   n,       n,      2'b00,  n,      2'b00, n,      1'b0};
                    end
                    
                    2'b11:
                    begin
                        cs_S2 = {4'd0, n,         rd,      2'd0, n,      rd,            y,       n,         2'd0,    n,      n,
                                 2'd0,   n,       n,      2'b00,  n,      2'b00, n,      1'b0};
                    end
                    
                    2'b10:
                    begin
                        
                        if (csm_en)
                        begin
                            cs_S2 = {4'd0, y,         rd,      2'd0, n,      rd,            y,       n,         2'd0,    n,      n,
                                     2'd0,   n,       n,      2'b00,  n,      2'b00, n,      1'b0};
                        end
                        else
                        
                        begin
                            cs_S2 = {4'd0, n,         rd,      2'd0, n,      rd,            y,       n,         2'd0,    n,      n,
                                     2'd0,   n,       n,      2'b00,  n,      2'b00, n,      1'b0};
                        end
                    end
                    default:
                    begin
                        cs_S2 = {27{1'bx}};
                    end
                    endcase

                end
                8'd14:
                begin
                    case (l2_way_state_mesi_S2)
                    2'b00:
                    begin
                        case (l2_way_state_vd_S2)
                        2'b10:
                        begin
                            cs_S2 = {4'd0, n,         rd,      2'd0, y,      rd,            n,       n,         2'd0,    n,          n,
                                    2'd0,   n,       y,      2'b00,  n,      2'b00, y,      1'b0};
                        end
                        2'b11:
                        begin
                            cs_S2 = {4'd0, n,         rd,      2'd0, y,      rd,            y,       n,         2'd0,    n,          n,
                                    2'd0,   n,       y,      2'b10,  n,      2'b00, y,      1'b0};
                        end
                        default:
                        begin
                            cs_S2 = {27{1'bx}};
                        end
                        endcase
                    end
                    2'b01:
                    begin
                        cs_S2 = {4'd0, y,         rd,      2'd0, n,      rd,            y,       n,         2'd0,    n,      n,
                                 2'd0,   n,       n,      2'b00,  n,      2'b00, n,      1'b0};
                    end
                    
                    2'b11:
                    begin
                        cs_S2 = {4'd0, n,         rd,      2'd0, n,      rd,            y,       n,         2'd0,    n,      n,
                                 2'd0,   n,       n,      2'b00,  n,      2'b00, n,      1'b0};
                    end
                    
                    2'b10:
                    begin
                        
                        if (csm_en)
                        begin
                            cs_S2 = {4'd0, y,         rd,      2'd0, n,      rd,            y,       n,         2'd0,    n,      n,
                                     2'd0,   n,       n,      2'b00,  n,      2'b00, n,      1'b0};
                        end
                        else
                        
                        begin
                            cs_S2 = {4'd0, n,         rd,      2'd0, n,      rd,            y,       n,         2'd0,    n,      n,
                                     2'd0,   n,       n,      2'b00,  n,      2'b00, n,      1'b0};
                        end
                    end
                    default:
                    begin
                        cs_S2 = {27{1'bx}};
                    end
                    endcase
                end

                8'd15:
                begin
                    case (l2_way_state_mesi_S2)
                    2'b00:
                    begin
                        case (l2_way_state_vd_S2)
                        2'b10:
                        begin
                            cs_S2 = {4'd0, n,         rd,      2'd0, y,      rd,            y,       n,         2'd0,    n,          y,
                                    2'd0,   n,       y,      2'b00,  n,      2'b00, y,      1'b0};
                        end
                        2'b11:
                        begin
                            cs_S2 = {4'd0, n,         rd,      2'd0, y,      rd,            y,       n,         2'd0,    n,          n,
                                    2'd0,   n,       y,      2'b10,  n,      2'b00, y,      1'b0};
                        end
                        default:
                        begin
                            cs_S2 = {27{1'bx}};
                        end
                        endcase
                    end
                    2'b01:
                    begin
                        cs_S2 = {4'd0, y,         rd,      2'd0, n,      rd,            y,       n,         2'd0,    n,      n,
                                 2'd0,   n,       n,      2'b00,  n,      2'b00, n,      1'b0};
                    end
                    
                    2'b11:
                    begin
                        cs_S2 = {4'd0, n,         rd,      2'd0, n,      rd,            y,       n,         2'd0,    n,      n,
                                 2'd0,   n,       n,      2'b00,  n,      2'b00, n,      1'b0};
                    end
                    
                    2'b10:
                    begin
                        
                        if (csm_en)
                        begin
                            cs_S2 = {4'd0, y,         rd,      2'd0, n,      rd,            y,       n,         2'd0,    n,      n,
                                     2'd0,   n,       n,      2'b00,  n,      2'b00, n,      1'b0};
                        end
                        else
                        
                        begin
                            cs_S2 = {4'd0, n,         rd,      2'd0, n,      rd,            y,       n,         2'd0,    n,      n,
                                     2'd0,   n,       n,      2'b00,  n,      2'b00, n,      1'b0};
                        end
                    end
                    default:
                    begin
                        cs_S2 = {27{1'bx}};
                    end
                    endcase
                end

                8'd31:
                begin
                    case (l2_way_state_mesi_S2)
                    2'b00:
                    begin
                        if (cache_type_S2_f == 1'b0)
                        begin
                            
                            if (csm_en)
                            begin
                                cs_S2 = {4'd0, y,         wr,      2'd1, y,      rd,            n,       y,         2'd1,     y,      n,
                                         2'd2,   y,       n,      2'b00,  y,      2'b10,   y,     1'b1};
                            end
                            else
                            
                            begin
                                cs_S2 = {4'd0, n,         rd,      2'd0, y,      rd,            n,       y,         2'd1,     y,      n,
                                         2'd2,   y,       n,      2'b00,  y,      2'b10,   y,     1'b1};
                            end
                        end
                        else
                        begin
                            
                            if (csm_en)
                            begin
                                if (lsid_S2 == 6'd63)
                                begin
                                    cs_S2 = {4'd0, y,         wr,      2'd0, y,      rd,            n,       y,         2'd0,     y,      n,
                                            2'd1,   y,       n,      2'b00,  y,      2'b11,   y,     1'b1};
                                end
                                else
                                begin
                                    cs_S2 = {4'd0, y,         wr,      2'd0, y,      rd,            n,       y,         2'd1,     y,      n,
                                            2'd1,   y,       n,      2'b00,  y,      2'b01,   y,     1'b1};

                                end
                            end
                            else
                            
                            begin
                                cs_S2 = {4'd0, y,         wr,      2'd0, y,      rd,            n,       y,         2'd0,     y,      n,
                                         2'd2,   y,       n,      2'b00,  y,      2'b01,   y,     1'b1};
                            end
                        end
                    end
                    2'b01:
                    begin
                        if (cache_type_S2_f == l2_way_state_cache_type_S2)
                        begin
                            
                            if (csm_en)
                            begin
                                cs_S2 = {4'd0, y,         wr,      2'd0, y,      rd,            n,       n,         2'd0,    n,      n,
                                        2'd0,   n,       n,      2'b00,  n,      2'b00, y,     1'b1};
                            end
                            else
                            
                            begin
                                cs_S2 = {4'd0, y,         wr,      2'd0, y,      rd,            n,       n,         2'd0,    y,      n,
                                        2'd2,   n,       n,      2'b00,  n,      2'b00, y,     1'b1};
                            end
                        end
                        else
                        begin
                            cs_S2 = {4'd0, y,         rd,      2'd0, n,      rd,            y,       n,         2'd0,    n,      n,
                                     2'd0,   n,       n,      2'b00,  n,      2'b00, n,      1'b0};
                        end
                    end
                    
                    2'b11:
                    begin
                        if (cache_type_S2_f == l2_way_state_cache_type_S2)
                        begin
                            
                            if (csm_en)
                            begin
                                cs_S2 = {4'd0, n,         wr,      2'd0, y,      rd,            n,       n,         2'd0,    n,      n,
                                         2'd0,   n,       n,      2'b00,  n,      2'b00, y,     1'b1};
                            end
                            else
                            
                            begin
                                cs_S2 = {4'd0, n,         wr,      2'd0, y,      rd,            n,       n,         2'd0,    y,      n,
                                         2'd2,   n,       n,      2'b00,  n,      2'b00, y,     1'b1};
                            end
                        end
                        else
                        begin
                            cs_S2 = {4'd0, n,         rd,      2'd0, n,      rd,            y,       n,         2'd0,    n,      n,
                                     2'd0,   n,       n,      2'b00,  n,      2'b00, n,      1'b0};
                        end
                    end
                    
                    2'b10:
                    begin
                        if (req_from_owner_S2 && (cache_type_S2_f == l2_way_state_cache_type_S2))
                        begin
                            cs_S2 = {4'd0, n,         rd,      2'd0, y,      rd,            n,       n,         2'd0,    y,      n,
                                     2'd2,   n,       n,      2'b00,  n,      2'b00, y,      1'b1};
                        end
                        else
                        begin
                            
                            if (csm_en)
                            begin
                                cs_S2 = {4'd0, y,         rd,      2'd0, n,      rd,            y,       n,         2'd0,    n,      n,
                                         2'd0,   n,       n,      2'b00,  n,      2'b00, n,      1'b0};
                            end
                            else
                            
                            begin
                                cs_S2 = {4'd0, n,         rd,      2'd0, n,      rd,            y,       n,         2'd0,    n,      n,
                                         2'd0,   n,       n,      2'b00,  n,      2'b00, n,      1'b0};
                            end
                        end
                    end
                    default:
                    begin
                        cs_S2 = {27{1'bx}};
                    end
                    endcase
                end
                // Just like a STORE
                8'd60:
                begin
                    case (l2_way_state_mesi_S2)
                    2'b00:
                    begin
                        if (cache_type_S2_f == 1'b0)
                        begin
                            cs_S2 = {4'd0, n,         rd,      2'd0, y,      rd,            n,       y,         2'd1,     y,      n,
                                     2'd2,   y,       n,      2'b00,  y,      2'b10,   y,     1'b1};
                        end
                        else begin
                            cs_S2 = {27{1'bx}};
                        end
                    end
                    2'b01:   // change MESI state to I in pipe2
                    begin
                        cs_S2 = {4'd0, y,         rd,      2'd0, n,      rd,            y,       n,         2'd0,    n,          n,
                                 2'd0,   n,       n,      2'b00,  n,      2'b00, n,      1'b0};
                    end
                    2'b10:
                    begin
                        if (req_from_owner_S2 && (cache_type_S2_f == l2_way_state_cache_type_S2))
                        begin
                            cs_S2 = {4'd0, n,         rd,      2'd0, y,      rd,            n,       n,         2'd0,    y,      n,
                                     2'd2,   n,       n,      2'b00,  n,      2'b00, y,      1'b1};
                        end
                        else
                        begin
                            cs_S2 = {4'd0, n,         rd,      2'd0, n,      rd,            y,       n,         2'd0,    n,      n,
                                     2'd0,   n,       n,      2'b00,  n,      2'b00, n,      1'b0};
                        end
                    end
                    default:
                    begin
                        cs_S2 = {27{1'bx}};
                    end
                    endcase
                end
                8'd1:
                begin
                    cs_S2 = {4'd0, n,         rd,      2'd0, n,      rd,            n,       n,         2'd0,    n,      n,
                             2'd0,   n,       n,      2'b00,  n,      2'b00, y,      1'b1};
                end

                8'd2:
                begin
                    case (l2_way_state_mesi_S2)
                    2'b00:
                    begin
                        
                        if (csm_en)
                        begin
                            cs_S2 = {4'd0, y,         wr,      2'd1, y,      rd,            n,       y,         2'd1,     y,      n,
                                     2'd2,   y,       n,      2'b00,  y,      2'b10, y,     1'b1};
                        end
                        else
                        
                        begin
                            cs_S2 = {4'd0, n,         rd,      2'd0, y,      rd,            n,       y,         2'd1,     y,      n,
                                     2'd2,   y,       n,      2'b00,  y,      2'b10, y,     1'b1};
                        end
                    end
                    2'b01:
                    begin
                        cs_S2 = {4'd0, y,         rd,      2'd0, n,      rd,            y,       n,         2'd0,    n,          n,
                                 2'd0,   n,       n,      2'b00,  n,      2'b00, n,      1'b0};
                    end
                    
                    2'b11:
                    begin
                        cs_S2 = {4'd0, n,         rd,      2'd0, n,      rd,            y,       n,         2'd0,    n,          n,
                                 2'd0,   n,       n,      2'b00,  n,      2'b00, n,      1'b0};
                    end
                    
                    2'b10:
                    begin
                        if (req_from_owner_S2 && (cache_type_S2_f == l2_way_state_cache_type_S2))
                        begin
                            cs_S2 = {4'd0, n,         rd,      2'd0, y,      rd,            n,       n,         2'd0,    y,      n,
                                     2'd2,   n,       n,      2'b00,  n,      2'b00, y,      1'b1};
                        end
                        else
                        begin
                            
                            if (csm_en)
                            begin
                                cs_S2 = {4'd0, y,         rd,      2'd0, n,      rd,            y,       n,         2'd0,    n,      n,
                                         2'd0,   n,       n,      2'b00,  n,      2'b00, n,      1'b0};
                            end
                            else
                            
                            begin
                                cs_S2 = {4'd0, n,         rd,      2'd0, n,      rd,            y,       n,         2'd0,    n,      n,
                                         2'd0,   n,       n,      2'b00,  n,      2'b00, n,      1'b0};
                            end
                        end
                    end
                    default:
                    begin
                        cs_S2 = {27{1'bx}};
                    end
                    endcase
                end
                8'd6:
                begin
                    case (l2_way_state_mesi_S2)
                    2'b00:
                    begin
                        cs_S2 = {4'd0, n,         rd,      2'd0, y,      rd,            n,       n,         2'd0,     n,      y,
                                 2'd0,   n,       n,      2'b00,  n,      2'b00,   y,     1'b1};
                    end
                    2'b01:
                    begin
                        cs_S2 = {4'd0, y,         rd,      2'd0, n,      rd,            y,       n,         2'd0,    n,      n,
                                 2'd0,   n,       n,      2'b00,  n,      2'b00, n,      1'b0};
                    end
                    
                    2'b11:
                    begin
                        cs_S2 = {4'd0, n,         rd,      2'd0, n,      rd,            y,       n,         2'd0,    n,      n,
                                 2'd0,   n,       n,      2'b00,  n,      2'b00, n,      1'b0};
                    end
                    
                    2'b10:
                    begin
                        
                        if (csm_en)
                        begin
                            cs_S2 = {4'd0, y,         rd,      2'd0, n,      rd,            y,       n,         2'd0,    n,      n,
                                     2'd0,   n,       n,      2'b00,  n,      2'b00, n,      1'b0};
                        end
                        else
                        
                        begin
                            cs_S2 = {4'd0, n,         rd,      2'd0, n,      rd,            y,       n,         2'd0,    n,      n,
                                     2'd0,   n,       n,      2'b00,  n,      2'b00, n,      1'b0};
                        end
                    end
                    default:
                    begin
                        cs_S2 = {27{1'bx}};
                    end
                    endcase
                end
                8'd10,
                8'd44,
                8'd45,
                8'd46,
                8'd47,
                8'd48,
                8'd49,
                8'd50,
                8'd51:
                begin
                    case (l2_way_state_mesi_S2)
                    2'b00:
                    begin
                        cs_S2 = {4'd0, n,         rd,      2'd0, y,      rd,            n,       n,         2'd0,     n,      n,
                                 2'd0,   n,       n,      2'b00,  n,      2'b00,   y,     1'b1};
                    end
                    2'b01:
                    begin
                        cs_S2 = {4'd0, y,         rd,      2'd0, n,      rd,            y,       n,         2'd0,    n,      n,
                                 2'd0,   n,       n,      2'b00,  n,      2'b00, n,      1'b0};
                    end
                    
                    2'b11:
                    begin
                        cs_S2 = {4'd0, n,         rd,      2'd0, n,      rd,            y,       n,         2'd0,    n,      n,
                                 2'd0,   n,       n,      2'b00,  n,      2'b00, n,      1'b0};
                    end
                    
                    2'b10:
                    begin
                        
                        if (csm_en)
                        begin
                            cs_S2 = {4'd0, y,         rd,      2'd0, n,      rd,            y,       n,         2'd0,    n,      n,
                                     2'd0,   n,       n,      2'b00,  n,      2'b00, n,      1'b0};
                        end
                        else
                        
                        begin
                            cs_S2 = {4'd0, n,         rd,      2'd0, n,      rd,            y,       n,         2'd0,    n,      n,
                                     2'd0,   n,       n,      2'b00,  n,      2'b00, n,      1'b0};
                        end
                    end
                    default:
                    begin
                        cs_S2 = {27{1'bx}};
                    end
                    endcase
                end

                8'd7,
                8'd11,
                8'd52,
                8'd53,
                8'd54,
                8'd55,
                8'd56,
                8'd57,
                8'd58,
                8'd59:
                begin
                    case (l2_way_state_mesi_S2)
                    2'b00:
                    begin
                        case (msg_type_S2_f)
                        8'd7,
                        8'd11:
                        begin
                            cs_S2 = {4'd0, n,         rd,      2'd0, y,      wr,            n,       n,         2'd0,    n,      y,
                                 2'd0,   y,       y,      2'b11,  n,      2'b00, y,      1'b1};
                        end
                        8'd52:
                        begin
                            cs_S2 = {4'd1, n,         rd,      2'd0, y,      wr,            n,       n,         2'd0,    n,      y,
                                 2'd0,   y,       y,      2'b11,  n,      2'b00, y,      1'b1};
                        end
                        8'd53:
                        begin
                            cs_S2 = {4'd2, n,         rd,      2'd0, y,      wr,            n,       n,         2'd0,    n,      y,
                                 2'd0,   y,       y,      2'b11,  n,      2'b00, y,      1'b1};
                        end
                        8'd54:
                        begin
                            cs_S2 = {4'd3, n,         rd,      2'd0, y,      wr,            n,       n,         2'd0,    n,      y,
                                 2'd0,   y,       y,      2'b11,  n,      2'b00, y,      1'b1};
                        end
                        8'd55:
                        begin
                            cs_S2 = {4'd4, n,         rd,      2'd0, y,      wr,            n,       n,         2'd0,    n,      y,
                                 2'd0,   y,       y,      2'b11,  n,      2'b00, y,      1'b1};
                        end
                        8'd56:
                        begin
                            cs_S2 = {4'd5, n,         rd,      2'd0, y,      wr,            n,       n,         2'd0,    n,      y,
                                 2'd0,   y,       y,      2'b11,  n,      2'b00, y,      1'b1};
                        end
                        8'd57:
                        begin
                            cs_S2 = {4'd6, n,         rd,      2'd0, y,      wr,            n,       n,         2'd0,    n,      y,
                                 2'd0,   y,       y,      2'b11,  n,      2'b00, y,      1'b1};
                        end
                        8'd58:
                        begin
                            cs_S2 = {4'd7, n,         rd,      2'd0, y,      wr,            n,       n,         2'd0,    n,      y,
                                 2'd0,   y,       y,      2'b11,  n,      2'b00, y,      1'b1};
                        end
                        8'd59:
                        begin
                            cs_S2 = {4'd8, n,         rd,      2'd0, y,      wr,            n,       n,         2'd0,    n,      y,
                                 2'd0,   y,       y,      2'b11,  n,      2'b00, y,      1'b1};
                        end
                        endcase
                    end
                    2'b01:
                    begin
                        cs_S2 = {4'd0, y,         rd,      2'd0, n,      rd,            y,       n,         2'd0,    n,      n,
                                 2'd0,   n,       n,      2'b00,  n,      2'b00, n,      1'b0};
                    end
                    
                    2'b11:
                    begin
                        cs_S2 = {4'd0, n,         rd,      2'd0, n,      rd,            y,       n,         2'd0,    n,      n,
                                 2'd0,   n,       n,      2'b00,  n,      2'b00, n,      1'b0};
                    end
                    
                    2'b10:
                    begin
                        
                        if (csm_en)
                        begin
                            cs_S2 = {4'd0, y,         rd,      2'd0, n,      rd,            y,       n,         2'd0,    n,      n,
                                     2'd0,   n,       n,      2'b00,  n,      2'b00, n,      1'b0};
                        end
                        else
                        
                        begin
                            cs_S2 = {4'd0, n,         rd,      2'd0, n,      rd,            y,       n,         2'd0,    n,      n,
                                     2'd0,   n,       n,      2'b00,  n,      2'b00, n,      1'b0};
                        end
                    end
                    default:
                    begin
                        cs_S2 = {27{1'bx}};
                    end
                    endcase
                end

                default:
                begin
                    cs_S2 = {27{1'bx}};
                end
            endcase
            end
        end
    end
    else
    begin
        cs_S2 = {27{1'b0}};
    end
end



reg [2-1:0] l2_load_data_subline_S2_f;
reg [2-1:0] l2_load_data_subline_S2_next;


always @ *
begin

    dir_clk_en_S2 = !stall_S2 && cs_S2[22];


 // L2_CAM_MSHR
end

always @ *
begin

    dir_rdw_en_S2 = !stall_S2 && cs_S2[21];


 // L2_CAM_MSHR
end


always @ *
begin
    dir_op_S2 = cs_S2[20:19];
end

always @ *
begin

    data_clk_en_S2 = !stall_real_S2 && cs_S2[18];


 // L2_CAM_MSHR
end

always @ *
begin

    data_rdw_en_S2 = !stall_real_S2 && cs_S2[17];


 // L2_CAM_MSHR
end

always @ *
begin
    mshr_wr_data_en_S2 = !stall_S2 && cs_S2[16];
    mshr_wr_state_en_S2 = !stall_S2 && cs_S2[16];
end

always @ *
begin
    amo_alu_op_S2 = cs_S2[26:23];
end



always @ *
begin
    if ( l2_tag_hit_S2 && (msg_type_S2_f == 8'd14 || msg_type_S2_f == 8'd15)
    && (l2_way_state_mesi_S2 == 2'b00) && (l2_way_state_vd_S2 == 2'b11))
    begin
        mshr_state_in_S2 = 2'd2;
    end
    else
    begin
        mshr_state_in_S2 = 2'd1;
    end
end


always @ *
begin
    if (!l2_tag_hit_S2)
    begin
        l2_miss_S2 = 1;
    end
    else
    begin
        l2_miss_S2 = l2_miss_S2_f;
    end
end




always @ *
begin
    if (special_addr_type_S2_f && (addr_type_S2 == 8'ha2) && (msg_type_S2_f == 8'd15))
    begin
        smc_wr_en_S2 = 1'b1;
        smc_wr_diag_en_S2 = 1'b1;
    end
    else
    begin
        smc_wr_en_S2 = 1'b0;
        smc_wr_diag_en_S2 = 1'b0;
    end
end

always @ *
begin
    if (special_addr_type_S2_f && (addr_type_S2 == 8'ha5))
    begin
        smc_flush_en_S2 = 1'b1;
    end
    else
    begin
        smc_flush_en_S2 = 1'b0;
    end
end


always @ *
begin
    smc_addr_op_S2 = addr_op_S2;
end


always @ *
begin
    state_owner_en_S2 =  cs_S2[15];
end


always @ *
begin
    state_owner_op_S2 = cs_S2[14:13];
end


always @ *
begin
    state_subline_en_S2 =  cs_S2[12];
end

always @ *
begin
    state_subline_op_S2 = cs_S2[10:9];
end



assign state_load_sdid_S2 = csm_en && state_owner_en_S2 && (state_owner_op_S2 ==2'd1)
                                && state_subline_en_S2 && (state_subline_op_S2 == 2'd1);




always @ *
begin
    state_di_en_S2 = cs_S2[8];
end

always @ *
begin
    state_vd_en_S2 = cs_S2[7];
end

always @ *
begin
    state_vd_S2 = cs_S2[6:5];
end

always @ *
begin
    state_mesi_en_S2 = cs_S2[4];
end

always @ *
begin
    if (!cs_S2[4])
    begin
        state_mesi_S2 = l2_way_state_mesi_S2;
    end
    else
    begin
        state_mesi_S2 = cs_S2[3:2];
    end
end

always @ *
begin
    state_lru_en_S2 =  cs_S2[1];
end

always @ *
begin
    state_lru_op_S2 = cs_S2[0];
end


//TODO
always @ *
begin
    state_rb_en_S2 =  l2_evict_S2  && (l2_way_state_mesi_S2 == 2'b00)
                 && (msg_type_S2_f != 8'd8);
end


always @ *
begin
    state_wr_en_S2 = valid_S2 && !stall_S2 && (
                          cs_S2[15]
                       || cs_S2[12]
                       || cs_S2[7]
                       || cs_S2[8]
                       || cs_S2[4]
                       || cs_S2[1]
                       || (state_rb_en_S2));
end


always @ *
begin
    req_recycle_cur_S2 = valid_S2
    &&  (~special_addr_type_S2_f)
    &&  ((pipe2_valid_S1 && (pipe2_msg_type_S1 == 8'd12)
        && (addr_S2[39:6] == pipe2_addr_S1[39:6]))
    ||   (pipe2_valid_S2 && (pipe2_msg_type_S1 == 8'd12)
        && (addr_S2[39:6] == pipe2_addr_S2[39:6]))
    ||   (pipe2_valid_S3 && (pipe2_msg_type_S3 == 8'd12)
        && (addr_S2[39:6] == pipe2_addr_S3[39:6])));
end


always @ *
begin
    if (!rst_n)
    begin
        req_recycle_buf_S2_next = 1'b0;
    end
    else
    begin
        if (!stall_S2)
        begin
            req_recycle_buf_S2_next = 1'b0;
        end
        else if (req_recycle_cur_S2)
        begin
            req_recycle_buf_S2_next = 1'b1;
        end
        else
        begin
            req_recycle_buf_S2_next = req_recycle_buf_S2_f;
        end
    end
end


always @ (posedge clk)
begin
    req_recycle_buf_S2_f <= req_recycle_buf_S2_next;
end


always @ *
begin
    req_recycle_S2 = req_recycle_cur_S2 | req_recycle_buf_S2_f;
end


always @ *
begin
    msg_data_ready_S2 = valid_S2 && !stall_S2 && (cs_S2[11] || msg_data_rd_S2_f);
end


always @ *
begin
    if (special_addr_type_S2)
    begin
        l2_ifill_S2 = n;
    end
    else if (valid_S2 && l2_tag_hit_S2 && data_clk_en_S2 && (data_rdw_en_S2 == rd) && ~l2_wb_S2
    && (cache_type_S2_f == 1'b1))
    begin
        l2_ifill_S2 = y;
    end
    else
    begin
        l2_ifill_S2 = n;
    end
end



//writeback reads 64B and required 4 cycles to read out from the data array
//ifill loads 32B and required 2 cycles to read out from the data array
always @ *
begin
    if (!rst_n)
    begin
        l2_load_data_subline_S2_next = 2'd0;
    end
    else if (valid_S2 && !(stall_real_S2) && l2_ifill_S2 && (l2_load_data_subline_S2_f == 2'd1))
    begin
        l2_load_data_subline_S2_next = 2'd0;
    end
    else if (valid_S2 && !(stall_real_S2) && (l2_wb_S2 || l2_ifill_S2))
    begin
        l2_load_data_subline_S2_next = l2_load_data_subline_S2_f + 1;
    end
    else
    begin
        l2_load_data_subline_S2_next = l2_load_data_subline_S2_f;
    end
end


always @ (posedge clk)
begin
    l2_load_data_subline_S2_f <= l2_load_data_subline_S2_next;
end

reg stall_load_S2;

always @ *
begin
    if (l2_wb_S2)
    begin
        stall_load_S2 = (l2_load_data_subline_S2_f != 2'd3);
    end
    else if (l2_ifill_S2)
    begin
        stall_load_S2 = (l2_load_data_subline_S2_f != 2'd1);
    end
    else
    begin
        stall_load_S2 = n;
    end
end


always @ *
begin
    l2_load_data_subline_S2 = l2_load_data_subline_S2_f;
end

reg stall_msg_S2;

always @ *
begin
    stall_msg_S2 = (cs_S2[11] || msg_data_rd_S2_f) && ~msg_data_valid_S2;
end


always @ *
begin
    stall_real_S2 = valid_S2 && (stall_pre_S2 || stall_msg_S2);
 end

always @ *
begin
    stall_S2 = valid_S2 && (stall_real_S2 || stall_load_S2);
end

reg valid_S2_next;

always @ *
begin
    valid_S2_next = valid_S2 && !stall_real_S2;
end



//============================
// Stage 2 -> Stage 3
//============================

reg valid_S3_f;
reg [8-1:0] msg_type_S3_f;
reg [3-1:0] data_size_S3_f;
reg [1-1:0] cache_type_S3_f;
reg msg_from_mshr_S3_f;
reg [2-1:0] l2_load_data_subline_S3_f;
reg [2-1:0] state_mesi_S3_f;
reg [1-1:0] l2_miss_S3_f;

reg mshr_smc_miss_S3_f;

reg state_wr_en_S3_f;
reg mshr_wr_data_en_S3_f;
reg mshr_wr_state_en_S3_f;
reg [2-1:0] mshr_state_in_S3_f;
reg [3-1:0] mshr_pending_index_S3_f;
reg special_addr_type_S3_f;
reg req_recycle_S3_f;

always @ (posedge clk)
begin
    if (!rst_n)
    begin
        valid_S3_f <= 1'b0;
        msg_type_S3_f <= 0;
        data_size_S3_f <= 0;
        cache_type_S3_f <= 0;
        msg_from_mshr_S3_f <= 0;
        l2_load_data_subline_S3_f <= 0;
        state_mesi_S3_f <= 0;
        l2_miss_S3_f <= 0;
        
        mshr_smc_miss_S3_f <= 0;
        
        state_wr_en_S3_f <= 0;
        mshr_wr_data_en_S3_f <= 0;
        mshr_wr_state_en_S3_f <= 0;
        mshr_state_in_S3_f <= 0;
        mshr_pending_index_S3_f <= 0;
        special_addr_type_S3_f <= 0;
        req_recycle_S3_f <= 0;
    end
    else if (!stall_S3)
    begin
        valid_S3_f <= valid_S2_next;
        msg_type_S3_f <= msg_type_S2_f;
        data_size_S3_f <= data_size_S2_f;
        cache_type_S3_f <= cache_type_S2_f;
        msg_from_mshr_S3_f <= msg_from_mshr_S2_f;
        l2_load_data_subline_S3_f <= l2_load_data_subline_S2_f;
        state_mesi_S3_f <= state_mesi_S2;
        l2_miss_S3_f <= l2_miss_S2;
        
        mshr_smc_miss_S3_f <= mshr_smc_miss_S2_f;
        
        state_wr_en_S3_f <= state_wr_en_S2;
        mshr_wr_data_en_S3_f <= mshr_wr_data_en_S2;
        mshr_wr_state_en_S3_f <= mshr_wr_state_en_S2;
        mshr_state_in_S3_f <= mshr_state_in_S2;
        mshr_pending_index_S3_f <= mshr_pending_index_S2_f;
        special_addr_type_S3_f <= special_addr_type_S2_f;
        req_recycle_S3_f <= req_recycle_S2;
    end
end


//============================
// Stage 3
//============================

reg stall_pre_S3;
reg stall_before_S3_f;
reg stall_before_S3_next;
reg req_recycle_S3;
reg req_recycle_cur_S3;
reg req_recycle_buf_S3_f;
reg req_recycle_buf_S3_next;

always @ *
begin
    stall_before_S3 = stall_before_S3_f;
    valid_S3 = valid_S3_f;
end



always @ *
begin
    req_recycle_cur_S3 = valid_S3 && (req_recycle_S3_f
     || (state_wr_en_S3_f
        && ((pipe2_valid_S1 && (pipe2_msg_type_S1 == 8'd12)
            && (addr_S3[39:6] == pipe2_addr_S1[39:6]))
        ||  (pipe2_valid_S2 && (pipe2_msg_type_S2 == 8'd12)
            && (addr_S3[39:6] == pipe2_addr_S2[39:6]))
        ||  (pipe2_valid_S3 && (pipe2_msg_type_S3 == 8'd12)
            && (addr_S3[39:6] == pipe2_addr_S3[39:6])))));
end

always @ *
begin
    if (!rst_n)
    begin
        req_recycle_buf_S3_next = 1'b0;
    end
    else
    begin
        if (!stall_S3)
        begin
            req_recycle_buf_S3_next = 1'b0;
        end
        else if (req_recycle_cur_S3)
        begin
            req_recycle_buf_S3_next = 1'b1;
        end
        else
        begin
            req_recycle_buf_S3_next = req_recycle_buf_S3_f;
        end
    end
end


always @ (posedge clk)
begin
    req_recycle_buf_S3_f <= req_recycle_buf_S3_next;
end


always @ *
begin
    req_recycle_S3 = req_recycle_cur_S3 | req_recycle_buf_S3_f;
end




always @ *
begin
    stall_pre_S3 = stall_S4;
    //stall_pre_S3 = stall_S4 || global_stall_S3;
end

always @ *
begin
    if (!rst_n)
    begin
        stall_before_S3_next = 0;
    end
    else
    begin
        stall_before_S3_next = stall_S3;
    end
end

//used to switch buffered output from arrays
always @ (posedge clk)
begin
    stall_before_S3_f <= stall_before_S3_next;
end



always @ *
begin
    stall_S3 = stall_pre_S3;
end



reg valid_S3_next;

always @ *
begin
    valid_S3_next = valid_S3 && !stall_S3;
end


//============================
// Stage 3 -> Stage 4
//============================

reg valid_S4_f;
reg [3-1:0] data_size_S4_f;
reg [1-1:0] cache_type_S4_f;
reg msg_from_mshr_S4_f;
reg [2-1:0] l2_load_data_subline_S4_f;
reg [2-1:0] state_mesi_S4_f;
reg [1-1:0] l2_miss_S4_f;

reg mshr_smc_miss_S4_f;

reg state_wr_en_S4_f;
reg mshr_wr_data_en_S4_f;
reg mshr_wr_state_en_S4_f;
reg [2-1:0] mshr_state_in_S4_f;
reg [3-1:0] mshr_pending_index_S4_f;
reg special_addr_type_S4_f;
reg [64-1:0] dir_data_S4_f;
reg req_recycle_S4_f;


always @ (posedge clk)
begin
    if (!rst_n)
    begin
        valid_S4_f <= 1'b0;
        msg_type_S4_f <= 0;
        data_size_S4_f <= 0;
        cache_type_S4_f <= 0;
        msg_from_mshr_S4_f <= 0;
        l2_load_data_subline_S4_f <= 0;
        state_mesi_S4_f <= 0;
        l2_miss_S4_f <= 0;
        
        mshr_smc_miss_S4_f <= 0;
        
        state_wr_en_S4_f <= 0;
        mshr_wr_data_en_S4_f <= 0;
        mshr_wr_state_en_S4_f <= 0;
        mshr_state_in_S4_f <= 0;
        mshr_pending_index_S4_f <= 0;
        special_addr_type_S4_f <= 0;
        dir_data_S4_f <= 0;
        req_recycle_S4_f <= 0;
    end
    else if (!stall_S4)
    begin
        valid_S4_f <= valid_S3_next;
        msg_type_S4_f <= msg_type_S3_f;
        data_size_S4_f <= data_size_S3_f;
        cache_type_S4_f <= cache_type_S3_f;
        msg_from_mshr_S4_f <= msg_from_mshr_S3_f;
        l2_load_data_subline_S4_f <= l2_load_data_subline_S3_f;
        state_mesi_S4_f <= state_mesi_S3_f;
        l2_miss_S4_f <= l2_miss_S3_f;
        
        mshr_smc_miss_S4_f <= mshr_smc_miss_S3_f;
        
        state_wr_en_S4_f <= state_wr_en_S3_f;
        mshr_wr_data_en_S4_f <= mshr_wr_data_en_S3_f;
        mshr_wr_state_en_S4_f <= mshr_wr_state_en_S3_f;
        mshr_state_in_S4_f <= mshr_state_in_S3_f;
        mshr_pending_index_S4_f <= mshr_pending_index_S3_f;
        special_addr_type_S4_f <= special_addr_type_S3_f;
        dir_data_S4_f <= dir_data_S3;
        req_recycle_S4_f <= req_recycle_S3;
    end
end


//============================
// Stage 4
//============================

reg stall_before_S4_f;
reg stall_before_S4_next;
reg dir_data_stall_S4;
reg state_wr_en_real_S4;
reg stall_inv_counter_S4;
reg msg_stall_S4;
reg load_store_mem_S4;

reg smc_stall_S4;
reg broadcast_stall_S4;

reg req_recycle_cur_S4;
reg req_recycle_buf_S4_f;
reg req_recycle_buf_S4_next;

reg [8-1:0] addr_type_S4;
reg [2-1:0] addr_op_S4;

reg msg0_send_valid_S4;
reg [8-1:0] msg0_send_type_S4;
reg msg1_send_valid_S4;
reg [8-1:0] msg1_send_type_S4;
//reg msg_send_valid_pre_S4;
//reg [`MSG_TYPE_WIDTH-1:0] msg_send_type_pre_S4;


reg smc_rd_diag_en_S4;
reg smc_rd_en_S4;


reg mshr_inv_flag_S4;

always @ *
begin
    valid_S4 = valid_S4_f;
    stall_before_S4 = stall_before_S4_f;
    msg_type_S4 = msg_type_S4_f;
    data_size_S4 = data_size_S4_f;
    cache_type_S4 = cache_type_S4_f;
    l2_miss_S4 = l2_miss_S4_f;
    special_addr_type_S4 = special_addr_type_S4_f;
    dir_data_S4 = dir_data_S4_f;
    msg_from_mshr_S4 = msg_from_mshr_S4_f;
end


always @ *
begin
    if (~special_addr_type_S4_f && req_recycle_S4)
    begin
        mshr_state_in_S4 = 2'd2;
    end
    else if(mshr_inv_flag_S4)
    begin
        mshr_state_in_S4 = 2'd0;
    end
    else
    begin
        mshr_state_in_S4 = mshr_state_in_S4_f;
    end
end

always @ *
begin
    req_recycle_cur_S4 = valid_S4 && (req_recycle_S4_f
     || (state_wr_en_S4_f
        && ((pipe2_valid_S1 && (pipe2_msg_type_S1 == 8'd12)
            && (addr_S4[39:6] == pipe2_addr_S1[39:6]))
        ||  (pipe2_valid_S2 && (pipe2_msg_type_S2 == 8'd12)
            && (addr_S4[39:6] == pipe2_addr_S2[39:6]))
        ||  (pipe2_valid_S3 && (pipe2_msg_type_S3 == 8'd12)
            && (addr_S4[39:6] == pipe2_addr_S3[39:6])))));
end

always @ *
begin
    if (!rst_n)
    begin
        req_recycle_buf_S4_next = 1'b0;
    end
    else
    begin
        if (!stall_S4)
        begin
            req_recycle_buf_S4_next = 1'b0;
        end
        else if (req_recycle_cur_S4)
        begin
            req_recycle_buf_S4_next = 1'b1;
        end
        else
        begin
            req_recycle_buf_S4_next = req_recycle_buf_S4_f;
        end
    end
end


always @ (posedge clk)
begin
    req_recycle_buf_S4_f <= req_recycle_buf_S4_next;
end


always @ *
begin
    req_recycle_S4 = req_recycle_cur_S4 | req_recycle_buf_S4_f;
end




reg smc_rd_diag_en_buf_S4_next;
reg smc_rd_en_buf_S4_next;
reg smc_rd_diag_en_buf_S4_f;
reg smc_rd_en_buf_S4_f;

always @ *
begin
    if (!rst_n)
    begin
        smc_rd_diag_en_buf_S4_next = 0;
        smc_rd_en_buf_S4_next = 0;
    end
    else if (!stall_smc_buf_S4)
    begin
        smc_rd_diag_en_buf_S4_next = smc_rd_diag_en_S4;
        smc_rd_en_buf_S4_next = smc_rd_en_S4;
    end
    else
    begin
        smc_rd_diag_en_buf_S4_next = smc_rd_diag_en_buf_S4_f;
        smc_rd_en_buf_S4_next = smc_rd_en_buf_S4_f;
    end
end

always @ (posedge clk)
begin
    smc_rd_diag_en_buf_S4_f <= smc_rd_diag_en_buf_S4_next;
    smc_rd_en_buf_S4_f <= smc_rd_en_buf_S4_next;
end


always @ *
begin
    smc_rd_diag_en_buf_S4 = smc_rd_diag_en_buf_S4_f;
    smc_rd_en_buf_S4 = smc_rd_en_buf_S4_f;
end
/*
reg smc_hit_buf_S4_next;
reg smc_hit_buf_S4_f;

always @ *
begin
    if (!rst_n)
    begin
        smc_hit_buf_S4_next = 0;
    end
    else if (!stall_smc_buf_S4)
    begin
        smc_hit_buf_S4_next = smc_hit_S4;
    end
    else
    begin
        smc_hit_buf_S4_next = smc_hit_buf_S4_f;
    end
end

always @ (posedge clk)
begin
    smc_hit_buf_S4_f <= smc_hit_buf_S4_next;
end
*/



always @ *
begin
    if(valid_S4)
        begin
        if (~special_addr_type_S4_f && req_recycle_S4)
        begin
            mshr_wr_data_en_S4 = ~stall_S4;
            mshr_wr_state_en_S4 = ~stall_S4;
            mshr_inv_flag_S4 = 1'b0;
        end
        else if (load_store_mem_S4)
        begin
            mshr_wr_data_en_S4 = msg_send_valid_S4 && (msg_send_type_S4 == msg0_send_type_S4) && msg_send_ready_S4;
            mshr_wr_state_en_S4 = msg_send_valid_S4 && (msg_send_type_S4 == msg0_send_type_S4) && msg_send_ready_S4;
            mshr_inv_flag_S4 = 1'b0;
        end
        else if (msg_send_type_S4 == 8'd18)
        begin
            mshr_wr_data_en_S4 = ((msg_send_valid_S4 && msg_send_ready_S4 && (dir_sharer_counter_S4 == 1)) || (~stall_S4))
                              && mshr_wr_data_en_S4_f;
            mshr_wr_state_en_S4 = ((msg_send_valid_S4 && msg_send_ready_S4 && (dir_sharer_counter_S4 == 1)) || (~stall_S4))
                               && mshr_wr_state_en_S4_f;
            mshr_inv_flag_S4 = 1'b0;
        end
        else if ((msg_type_S4 == 8'd13) && l2_tag_hit_S4
              && (l2_way_state_mesi_S4 == 2'b10) && l2_way_state_subline_S4[addr_S4[5:4]]
              && req_from_owner_S4)
        begin
            mshr_wr_data_en_S4 = ~stall_S4;
            mshr_wr_state_en_S4 = ~stall_S4;
            mshr_inv_flag_S4 = 1'b0;
        end
        
        else if (csm_en && mshr_smc_miss_S4_f && (~mshr_wr_state_en_S4_f))
        begin
            mshr_wr_data_en_S4 = 1'b0;
            mshr_wr_state_en_S4 = ~stall_S4;
            mshr_inv_flag_S4 = 1'b1;
        end
        
        else
        begin
            mshr_wr_data_en_S4 = ~stall_S4 && mshr_wr_data_en_S4_f;
            mshr_wr_state_en_S4 = ~stall_S4 && mshr_wr_state_en_S4_f;
            mshr_inv_flag_S4 = 1'b0;
        end
    end
    else
    begin
        mshr_wr_data_en_S4 = 1'b0;
        mshr_wr_state_en_S4 = 1'b0;
        mshr_inv_flag_S4 = 1'b0;
    end
end

always @ *
begin
    if (valid_S4 && (!req_recycle_S4) && (msg_send_type_S4 == 8'd18)
     && (msg_send_valid_S4 && msg_send_ready_S4 && (dir_sharer_counter_S4 == 1) && stall_S4)
     && mshr_wr_data_en_S4_f)
    begin
        inv_fwd_pending_S4 = 1'b1;
    end
    else
    begin
        inv_fwd_pending_S4 = 1'b0;
    end
end

always @ *
begin
    addr_type_S4 = addr_S4[39:32];
    addr_op_S4 = addr_S4[31:30];
end



always @ *
begin
    if (!rst_n)
    begin
        stall_before_S4_next = 0;
    end
    else
    begin
        stall_before_S4_next = stall_S4;
    end
end

//used to switch buffered output from arrays
always @ (posedge clk)
begin
    stall_before_S4_f <= stall_before_S4_next;
end


reg [19-1:0] cs_S4;

always @ *
begin
    if (valid_S4)
    begin
        if (special_addr_type_S4_f)
        begin
            if (msg_type_S4_f == 8'd15)
            begin
                //       msg        msg0        msg0                   msg1        msg1
                //       send_fwd   send_en     send_type              send_en     send_type
                cs_S4 = {n,         y,          8'd28,  n,          8'd30};
            end
            else
            begin
                cs_S4 = {n,         y,          8'd29,  n,          8'd30};
            end
        end
        else if(req_recycle_S4)
        begin
            cs_S4 = {19{1'b0}};
        end
        else
        begin
            if (msg_type_S4_f == 8'd32)
            begin
                cs_S4 = {n,         y,          8'd33,    n,          8'd30};
            end
            else if (msg_type_S4_f == 8'd8)
            begin
                cs_S4 = {n,         n,          8'd30,    n,          8'd30};
            end
            else if (msg_type_S4_f == 8'd13)
            begin
                cs_S4 = {n,         n,          8'd30,    n,          8'd30};
            end
            else if (l2_evict_S4)
            begin
                begin
                    case (l2_way_state_mesi_S4)
                    2'b01, 2'b11:
                    begin
                        cs_S4 = {y,         y,          8'd18,    n,          8'd30};
                    end
                    2'b10:
                    begin
                        cs_S4 = {y,         y,          8'd17,    n,          8'd30};
                    end
                    2'b00:
                    begin
                        case (l2_way_state_vd_S4)
                        2'b10:
                        begin
                            if (msg_type_S4_f == 8'd34)
                            begin
                                cs_S4 = {n,         y,          8'd29,    n,          8'd30};
                            end
                            else if (msg_type_S4_f == 8'd14)
                            begin
                                //TODO
                                //cs_S4 = {y,          `MSG_TYPE_NC_LOAD_REQ,    n,          `MSG_TYPE_ERROR};
                                cs_S4 = {n,         y,          8'd19,    n,          8'd30};
                            end
                            else
                            begin
                                cs_S4 = {n,         y,          8'd19,    n,          8'd30};
                            end
                        end
                        2'b11:
                        begin
                            if (msg_type_S4_f == 8'd34)
                            begin
                                cs_S4 = {n,         y,          8'd29,    y,          8'd20};
                            end
                            else if (msg_type_S4_f == 8'd14)
                            begin
                                //TODO
                                //cs_S4 = {y,          `MSG_TYPE_NC_LOAD_REQ,    y,          `MSG_TYPE_STORE_MEM};
                                cs_S4 = {n,         y,          8'd19,    y,          8'd20};
                            end
                            else
                            begin
                                cs_S4 = {n,         y,          8'd19,    y,          8'd20};
                            end
                        end
                        default:
                        begin
                            cs_S4 = {19{1'bx}};
                        end
                        endcase
                    end
                    default:
                    begin
                        cs_S4 = {19{1'bx}};
                    end
                    endcase
                end
            end

            else if (!l2_tag_hit_S4)
            begin
                begin
                    if (msg_type_S4_f == 8'd35 || msg_type_S4_f == 8'd34)
                    begin
                        cs_S4 = {n,         y,          8'd29,    n,          8'd30};
                    end
                    else if (msg_type_S4_f == 8'd15)
                    begin
                        if (msg_from_mshr_S4_f)
                        begin
                            //       msg         msg0        msg0                   msg1        msg1
                            //       send_fwd    send_en     send_type              send_en     send_type
                            cs_S4 = {n,          y,          8'd28,  n,          8'd30};
                        end
                        else
                        begin
                            cs_S4 = {n,          y,          8'd15,n,          8'd30};
                        end
                    end
                    else if (msg_type_S4_f == 8'd14)
                    begin
                        //TODO
                        //cs_S4 = {y,          `MSG_TYPE_NC_LOAD_REQ,    n,          `MSG_TYPE_ERROR};
                        cs_S4 = {n,         y,          8'd19,    n,          8'd30};
                    end
                    else
                    begin
                        cs_S4 = {n,         y,          8'd19,    n,          8'd30};
                    end
                end
            end

            else begin
            case (msg_type_S4_f)
                //TODO
                8'd35, 8'd34:
                begin
                   case (l2_way_state_mesi_S4)
                    2'b00:
                    begin
                        case (l2_way_state_vd_S4)
                        2'b10:
                        begin
                            cs_S4 = {n,         y,          8'd29,    n,          8'd30};
                        end
                        2'b11:
                        begin
                            cs_S4 = {n,         y,          8'd29,    y,          8'd20};
                        end
                        default:
                        begin
                            cs_S4 = {19{1'bx}};
                        end
                        endcase
                    end
                    2'b01, 2'b11:
                    begin
                        cs_S4 = {y,         y,          8'd18,    n,          8'd30};
                    end
                    2'b10:
                    begin
                        cs_S4 = {y,         y,          8'd17,   n,          8'd30};
                    end
                    default:
                    begin
                        cs_S4 = {19{1'bx}};
                    end
                    endcase
                end
                8'd15:
                begin
                   case (l2_way_state_mesi_S4)
                    2'b00:
                    begin
                        case (l2_way_state_vd_S4)
                        2'b10:
                        begin
                            cs_S4 = {n,         y,          8'd15,    n,          8'd30};
                        end
                        2'b11:
                        begin
                            cs_S4 = {n,         y,          8'd20,    n,          8'd30};
                        end
                        default:
                        begin
                            cs_S4 = {19{1'bx}};
                        end
                        endcase
                    end
                    2'b01, 2'b11:
                    begin
                        cs_S4 = {y,         y,          8'd18,    n,          8'd30};
                    end
                    2'b10:
                    begin
                        cs_S4 = {y,         y,          8'd17,   n,          8'd30};
                    end
                    default:
                    begin
                        cs_S4 = {19{1'bx}};
                    end
                    endcase
                end

                8'd14:
                begin
                   case (l2_way_state_mesi_S4)
                    2'b00:
                    begin
                        case (l2_way_state_vd_S4)
                        2'b10:
                        begin
                            cs_S4 = {n,         y,          8'd29,    n,          8'd30};
                        end
                        2'b11:
                        begin
                            cs_S4 = {n,         y,          8'd20,    n,          8'd30};
                        end
                        default:
                        begin
                            cs_S4 = {19{1'bx}};
                        end
                        endcase
                    end
                    2'b01, 2'b11:
                    begin
                        cs_S4 = {y,         y,          8'd18,    n,          8'd30};
                    end
                    2'b10:
                    begin
                        cs_S4 = {y,         y,          8'd17,   n,          8'd30};
                    end
                    default:
                    begin
                        cs_S4 = {19{1'bx}};
                    end
                    endcase
                end
                8'd31:
                begin
                    case (l2_way_state_mesi_S4)
                    2'b00:
                    begin
                        cs_S4 = {n,         y,          8'd29,    n,          8'd30};
                    end
                    2'b01, 2'b11:
                    begin
                        if (cache_type_S4_f == l2_way_state_cache_type_S4)
                        begin
                            cs_S4 = {n,         y,          8'd29,    n,          8'd30};
                        end
                        else
                        begin
                            cs_S4 = {y,         y,          8'd18,    n,          8'd30};
                        end
                    end
                    2'b10:
                    begin
                        if (req_from_owner_S4 && (cache_type_S4_f == l2_way_state_cache_type_S4))
                        begin
                            cs_S4 = {n,         y,          8'd29,    n,          8'd30};
                        end
                        else if (cache_type_S4_f != l2_way_state_cache_type_S4)
                        begin
                            cs_S4 = {y,         y,          8'd17,   n,          8'd30};
                        end
                        else
                        begin
                            cs_S4 = {y,         y,          8'd16,   n,          8'd30};
                        end
                    end
                    default:
                    begin
                        cs_S4 = {19{1'bx}};
                    end
                    endcase
                end
                8'd60:   // Jsut same as a store req 
                begin
                    case (l2_way_state_mesi_S4)
                    2'b00:
                    begin
                        cs_S4 = {n,         y,          8'd29,    n,          8'd30};
                    end
                    2'b01:
                    begin
                        cs_S4 = {y,         y,          8'd18,    n,          8'd30};
                    end
                    2'b10:
                    begin
                        if (req_from_owner_S4 && (cache_type_S4_f == l2_way_state_cache_type_S4))
                        begin
                            cs_S4 = {n,         y,          8'd29,    n,          8'd30};
                        end
                        else
                        begin
                            cs_S4 = {y,         y,          8'd17,    n,          8'd30};
                        end
                    end
                    default:
                    begin
                        cs_S4 = {19{1'bx}};
                    end
                    endcase
                end
                8'd1:
                begin
                    cs_S4 = {n,         y,          8'd28,    n,          8'd30};
                end

                8'd2:
                begin
                    case (l2_way_state_mesi_S4)
                    2'b00:
                    begin
                        cs_S4 = {n,         y,          8'd29,    n,          8'd30};
                    end
                    2'b01, 2'b11:
                    begin
                        cs_S4 = {y,         y,          8'd18,    n,          8'd30};
                    end
                    2'b10:
                    begin
                        if (req_from_owner_S4 && (cache_type_S4_f == l2_way_state_cache_type_S4))
                        begin
                            cs_S4 = {n,         y,          8'd29,    n,          8'd30};
                        end
                        else
                        begin
                            cs_S4 = {y,         y,          8'd17,    n,          8'd30};
                        end
                    end
                    default:
                    begin
                        cs_S4 = {19{1'bx}};
                    end
                    endcase
                end
                8'd6, 8'd10,
                8'd44,
                8'd45,
                8'd46,
                8'd47,
                8'd48,
                8'd49,
                8'd50,
                8'd51:
                begin
                    case (l2_way_state_mesi_S4)
                    2'b00:
                    begin
                        cs_S4 = {n,         y,          8'd29,    n,          8'd30};
                    end
                    2'b01, 2'b11:
                    begin
                        cs_S4 = {y,         y,          8'd18,    n,          8'd30};
                    end
                    2'b10:
                    begin
                        cs_S4 = {y,         y,          8'd17,    n,          8'd30};
                    end
                    default:
                    begin
                        cs_S4 = {19{1'bx}};
                    end
                    endcase
                end

                8'd7, 8'd11,
                8'd52,
                8'd53,
                8'd54,
                8'd55,
                8'd56,
                8'd57,
                8'd58,
                8'd59:
                begin
                    case (l2_way_state_mesi_S4)
                    2'b00:
                    begin
                        cs_S4 = {n,         n,          8'd30,    n,          8'd30};
                    end
                    2'b01, 2'b11:
                    begin
                        cs_S4 = {y,         y,          8'd18,    n,          8'd30};
                    end
                    2'b10:
                    begin
                        cs_S4 = {y,         y,          8'd17,    n,          8'd30};
                    end
                    default:
                    begin
                        cs_S4 = {19{1'bx}};
                    end
                    endcase
                end

                default:
                begin
                    cs_S4 = {19{1'bx}};
                end
            endcase
            end
        end
    end
    else
    begin
        cs_S4 = {19{1'b0}};
    end
end




always @ *
begin
    msg0_send_valid_S4 = !(global_stall_S4 || stall_inv_counter_S4 || smc_stall_S4 || broadcast_stall_S4) && cs_S4[17];
    msg0_send_type_S4 = cs_S4[16:9];
    msg1_send_valid_S4 = !(global_stall_S4 || stall_inv_counter_S4|| smc_stall_S4 || broadcast_stall_S4) && cs_S4[8];
    msg1_send_type_S4 = cs_S4[7:0];
end











always @ *
begin
    load_store_mem_S4 = cs_S4[17] && cs_S4[8] &&
                       (msg0_send_type_S4 == 8'd19 || msg0_send_type_S4 == 8'd14)
                     &&(msg1_send_type_S4 == 8'd20);
end

localparam msg_state_0 = 1'b0;
localparam msg_state_1 = 1'b1;

reg msg_state_S4_f;
reg msg_state_S4_next;



always @ *
begin
    if (!rst_n)
    begin
        msg_state_S4_next = msg_state_0;
    end
    else if (msg0_send_valid_S4 && msg1_send_valid_S4 && valid_S4
    && !(dir_data_stall_S4 || smc_stall_S4 || (msg_send_valid_S4 && !msg_send_ready_S4) || global_stall_S4 || broadcast_stall_S4))
    begin
        if (msg_state_S4_f == msg_state_0)
        begin
            msg_state_S4_next = msg_state_1;
        end
        else
        begin
            if (l2_load_data_subline_S4_f == 2'd3)
            begin
                msg_state_S4_next = msg_state_0;
            end
            else
            begin
                msg_state_S4_next = msg_state_1;
            end
        end
    end
    else
    begin
        msg_state_S4_next = msg_state_S4_f;
    end
end



































always @ (posedge clk)
begin
    msg_state_S4_f <= msg_state_S4_next;
end

always @ *
begin
    if (msg_state_S4_f == msg_state_0)
    begin
        msg_send_valid_S4 = msg0_send_valid_S4;
        msg_send_type_pre_S4 = msg0_send_type_S4;
    end
    else
    begin
        msg_send_valid_S4 = msg1_send_valid_S4;
        msg_send_type_pre_S4 = msg1_send_type_S4;
    end
end


always @ *
begin
    if (smc_miss_S4)
    begin
        msg_send_type_S4 = 8'd14;
    end
    else
    begin
        msg_send_type_S4 = msg_send_type_pre_S4;
    end
end








//stall signal for one cycle delay of smc array read to meet timing
/*
always @ *
begin
    if (msg_state_S4_f == msg_state_0)
    begin
        msg_send_valid_pre_S4 = msg0_send_valid_S4;
        msg_send_type_S4 = msg0_send_type_S4;
    end
    else
    begin
        msg_send_valid_pre_S4 = msg1_send_valid_S4;
        msg_send_type_S4 = msg1_send_type_S4;
    end
end
*/


localparam smc_state_0 = 1'b0;
localparam smc_state_1 = 1'b1;

reg smc_state_S4_f;
reg smc_state_S4_next;

always @ *
begin
    if (!rst_n)
    begin
        smc_state_S4_next = smc_state_0;
    end
    else if (smc_rd_en_S4 && (~stall_smc_buf_S4))
    begin
        if (smc_state_S4_f == smc_state_0)
        begin
            smc_state_S4_next = smc_state_1;
        end
        else
        begin
            smc_state_S4_next = smc_state_0;
        end
    end
    else
    begin
        smc_state_S4_next = smc_state_S4_f;
    end
end

always @ (posedge clk)
begin
    smc_state_S4_f <= smc_state_S4_next;
end

always @ *
begin
    smc_stall_S4 = smc_rd_en_S4 && (smc_state_S4_f == smc_state_0);
end

/*
always @ *
begin
    msg_send_valid_S4 = msg_send_valid_pre_S4 && (~smc_stall_S4);
end
*/

reg [3-1:0] mshr_empty_index_buf_S4_f;
reg [3-1:0] mshr_empty_index_buf_S4_next;
reg [3-1:0] mshr_empty_index_sel_S4;

always @ *
begin
    if (stall_before_S4_f)
    begin
        mshr_empty_index_sel_S4 = mshr_empty_index_buf_S4_f;
    end
    else
    begin
        mshr_empty_index_sel_S4 = mshr_empty_index_S4;
    end
end

always @ *
begin
    if (!rst_n)
    begin
        mshr_empty_index_buf_S4_next = {3{1'b0}};
    end
    else if (stall_S4 && !stall_before_S4_f)
    begin
        mshr_empty_index_buf_S4_next = mshr_empty_index_S4;
    end
    else
    begin
        mshr_empty_index_buf_S4_next = mshr_empty_index_buf_S4_f;
    end
end


always @ (posedge clk)
begin
    mshr_empty_index_buf_S4_f <= mshr_empty_index_buf_S4_next;
end



always @ *
begin
    if (msg_send_valid_S4)
    begin
        case (msg_send_type_S4)
        8'd16, 8'd17, 8'd18:
        begin
            msg_send_mode_S4 = 3'd4;
            msg_send_length_S4 = 2;
            msg_send_data_size_S4 = 3'b101;
            msg_send_cache_type_S4 = l2_way_state_cache_type_S4;
            msg_send_mshrid_S4 = mshr_wr_index_in_S4;
        end
        8'd28:
        begin
            msg_send_mode_S4 = 3'd1;
            msg_send_length_S4 = 0;
            msg_send_data_size_S4 = 3'b000;
            msg_send_cache_type_S4 = cache_type_S4_f;
            msg_send_mshrid_S4 = mshrid_S4;
        end
        8'd29:
        begin
            if (special_addr_type_S4_f)
            begin
//TODO
/*
                msg_send_mode_S4 = `L2_P1_BUF_OUT_MODE_1H1D;
                msg_send_length_S4 = 1;
                msg_send_data_size_S4 = `MSG_DATA_SIZE_8B;
                msg_send_cache_type_S4 = cache_type_S4_f;
                msg_send_mshrid_S4 = mshrid_S4;
*/
                msg_send_mode_S4 = 3'd3;
                msg_send_length_S4 = 2;
                msg_send_data_size_S4 = data_size_S4_f;
                msg_send_cache_type_S4 = cache_type_S4_f;
                msg_send_mshrid_S4 = mshrid_S4;
            end
            else if (cache_type_S4_f == 1'b0)
            begin
                msg_send_mode_S4 = 3'd3;
                msg_send_length_S4 = 2;
                msg_send_data_size_S4 = 3'b101;
                msg_send_cache_type_S4 = cache_type_S4_f;
                msg_send_mshrid_S4 = mshrid_S4;
            end
            else
            begin
                msg_send_length_S4 = 4;
                msg_send_data_size_S4 = 3'b110;
                msg_send_cache_type_S4 = cache_type_S4_f;
                msg_send_mshrid_S4 = mshrid_S4;
                if (l2_load_data_subline_S4_f == 2'd0)
                begin
                    msg_send_mode_S4 = 3'd3;
                end
                else
                begin
                    msg_send_mode_S4 = 3'd7;
                end
            end

        end
        8'd19:
        begin
            msg_send_mode_S4 = 3'd4;
            msg_send_length_S4 = 2;



            msg_send_data_size_S4 = data_size_S4_f;

            msg_send_cache_type_S4 = 1'b0;
            msg_send_mshrid_S4 = mshr_wr_index_in_S4;
        end
        8'd14:
        begin
            msg_send_mode_S4 = 3'd4;
            msg_send_length_S4 = 2;



            msg_send_data_size_S4 = data_size_S4_f;

            msg_send_cache_type_S4 = cache_type_S4_f;
            msg_send_mshrid_S4 = mshr_wr_index_in_S4;
        end

        8'd15:
        begin
            msg_send_mode_S4 = 3'd5;
            msg_send_length_S4 = 3;
            msg_send_data_size_S4 = data_size_S4_f;
            msg_send_cache_type_S4 = cache_type_S4_f;
            msg_send_mshrid_S4 = mshr_wr_index_in_S4;
        end
        8'd33:
        begin
            msg_send_mode_S4 = 3'd2;
            msg_send_length_S4 = 1;
            msg_send_data_size_S4 = data_size_S4_f;
            msg_send_cache_type_S4 = cache_type_S4_f;
            msg_send_mshrid_S4 = mshrid_S4;
        end

        8'd20:
        begin
            msg_send_length_S4 = 10;



            msg_send_data_size_S4 = data_size_S4_f;

            msg_send_cache_type_S4 = 1'b0;
            msg_send_mshrid_S4 = mshr_wr_index_in_S4;
            if (l2_load_data_subline_S4_f == 2'd0)
            begin
                msg_send_mode_S4 = 3'd6;
            end
            else
            begin
                msg_send_mode_S4 = 3'd7;
            end
        end
        default:
        begin
            msg_send_mode_S4 = 3'd0;
            msg_send_length_S4 = 0;
            msg_send_data_size_S4 = 3'b000;
            msg_send_cache_type_S4 = 1'b0;
            msg_send_mshrid_S4 = mshrid_S4;
        end
        endcase
    end
    else
    begin
        msg_send_mode_S4 = 3'd0;
        msg_send_length_S4 = 0;
        msg_send_data_size_S4 = 3'b000;
        msg_send_cache_type_S4 = 1'b0;
        msg_send_mshrid_S4 = mshrid_S4;
    end
end

always @ *
begin
    msg_send_l2_miss_S4 = l2_miss_S4_f;
end

always @ *
begin
    if ((msg_send_type_S4 == 8'd33)
    || special_addr_type_S4_f
    || ((msg_type_S4_f == 8'd15) && !l2_tag_hit_S4))
    begin
        msg_send_subline_vector_S4 = {4{1'b0}};
    end
    else if (msg_send_type_S4 == 8'd18)
    begin
        msg_send_subline_vector_S4 = {4{1'b1}};
    end
    else
    begin
        msg_send_subline_vector_S4 = l2_way_state_subline_S4;
    end
end


always @ *
begin
    if ((msg_type_S4_f == 8'd2 || msg_type_S4_f == 8'd60) && msg_send_type_S4 == 8'd29)
    begin
        msg_send_mesi_S4 = 2'b11;
    end
    else if ((msg_send_type_S4 == 8'd33)
    || special_addr_type_S4_f
    || ((msg_type_S4_f == 8'd15) && !l2_tag_hit_S4))
    begin
        msg_send_mesi_S4 = 2'b00;
    end
    else
    begin
        if (state_mesi_S4_f == 2'b11)
        begin
            msg_send_mesi_S4 = 2'b01;
        end
        else
        begin
            msg_send_mesi_S4 = state_mesi_S4_f;
        end
    end
end

always @ *
begin
    l2_access_valid_S4 = valid_S4 && !stall_S4 && msg_send_valid_S4
                     && (msg_send_type_S4 == 8'd29 || msg_send_type_S4 == 8'd28);
end


always @ *
begin
    l2_miss_valid_S4 = l2_access_valid_S4 && msg_send_l2_miss_S4;
end


always @ *
begin
    msg_stall_S4 = msg0_send_valid_S4 && msg1_send_valid_S4
               && (msg_state_S4_f == msg_state_0);
end


reg [64-1:0] dir_data_buf_S4_f;
reg [64-1:0] dir_data_buf_S4_next;
reg [64-1:0] dir_data_trans_S4;

always @ *
begin
    if (stall_before_S4_f)
    begin
        dir_data_sel_S4 = dir_data_buf_S4_f;
    end
    else
    begin
        
        if(mshr_smc_miss_S4_f)
        begin
            //continue invalidation in the middle of the sharer list
            dir_data_sel_S4 = (dir_data_S4 >> mshr_miss_lsid_S4) << mshr_miss_lsid_S4;
        end
        else
        
        begin
            dir_data_sel_S4 = dir_data_S4;
        end
    end
end


always @ *
begin
    if (!rst_n)
    begin
        dir_data_buf_S4_next = {64{1'b0}};
    end
    else if ((stall_S4 && !stall_before_S4_f) && (msg_send_type_pre_S4 == 8'd18) && (l2_way_state_mesi_S4 != 2'b11))
    begin
        if (msg_stall_S4 || smc_stall_S4 || (msg_send_valid_S4 && !msg_send_ready_S4)
         || global_stall_S4 || stall_inv_counter_S4 )
        begin
            if(mshr_smc_miss_S4_f)
            begin
                dir_data_buf_S4_next = (dir_data_S4 >> mshr_miss_lsid_S4) << mshr_miss_lsid_S4;
            end
            else
            begin
                dir_data_buf_S4_next = dir_data_S4;
            end
        end
        else
        begin
            dir_data_buf_S4_next = dir_data_trans_S4;
        end
    end
    else if (!((msg_send_valid_S4 && !msg_send_ready_S4) || global_stall_S4 || smc_stall_S4 || broadcast_stall_S4
             || stall_inv_counter_S4) && dir_data_stall_S4)
    begin
        dir_data_buf_S4_next = dir_data_trans_S4;
    end
    else
    begin
        dir_data_buf_S4_next = dir_data_buf_S4_f;
    end
end































always @ (posedge clk)
begin
    dir_data_buf_S4_f <= dir_data_buf_S4_next;
end

wire [64-1:0] dir_sharer_mask_S4;
wire nonzero_sharer_S4;


l2_priority_encoder_6 priority_encoder_6bits( 

    .data_in        (dir_data_sel_S4),
    .data_out       (dir_sharer_S4),
    .data_out_mask  (dir_sharer_mask_S4),
    .nonzero_out    (nonzero_sharer_S4)
);

/*
always @ *
begin
    dir_sharer_mask_S4 = {`L2_DIR_ARRAY_WIDTH{1'b1}};
    dir_sharer_mask_S4[dir_sharer_S4] = 1'b0;
end
*/
//decode sharers from the sharer list with priority decoder
/*
always @ *
begin
    dir_sharer_S4 = {`L2_OWNER_BITS{1'b0}};
    dir_sharer_mask_S4 = {`L2_DIR_ARRAY_WIDTH{1'b0}};
    if (dir_data_sel_S4[0])
    begin
        dir_sharer_S4 = 6'd0;
        dir_sharer_mask_S4 = 64'b0000000000000000000000000000000000000000000000000000000000000001;
    end
    else if (dir_data_sel_S4[1])
    begin
        dir_sharer_S4 = 6'd1;
        dir_sharer_mask_S4 = 64'b0000000000000000000000000000000000000000000000000000000000000010;
    end
    else if (dir_data_sel_S4[2])
    begin
        dir_sharer_S4 = 6'd2;
        dir_sharer_mask_S4 = 64'b0000000000000000000000000000000000000000000000000000000000000100;
    end
    else if (dir_data_sel_S4[3])
    begin
        dir_sharer_S4 = 6'd3;
        dir_sharer_mask_S4 = 64'b0000000000000000000000000000000000000000000000000000000000001000;
    end
    else if (dir_data_sel_S4[4])
    begin
        dir_sharer_S4 = 6'd4;
        dir_sharer_mask_S4 = 64'b0000000000000000000000000000000000000000000000000000000000010000;
    end
    else if (dir_data_sel_S4[5])
    begin
        dir_sharer_S4 = 6'd5;
        dir_sharer_mask_S4 = 64'b0000000000000000000000000000000000000000000000000000000000100000;
    end
    else if (dir_data_sel_S4[6])
    begin
        dir_sharer_S4 = 6'd6;
        dir_sharer_mask_S4 = 64'b0000000000000000000000000000000000000000000000000000000001000000;
    end
    else if (dir_data_sel_S4[7])
    begin
        dir_sharer_S4 = 6'd7;
        dir_sharer_mask_S4 = 64'b0000000000000000000000000000000000000000000000000000000010000000;
    end
    else if (dir_data_sel_S4[8])
    begin
        dir_sharer_S4 = 6'd8;
        dir_sharer_mask_S4 = 64'b0000000000000000000000000000000000000000000000000000000100000000;
    end
    else if (dir_data_sel_S4[9])
    begin
        dir_sharer_S4 = 6'd9;
        dir_sharer_mask_S4 = 64'b0000000000000000000000000000000000000000000000000000001000000000;
    end
    else if (dir_data_sel_S4[10])
    begin
        dir_sharer_S4 = 6'd10;
        dir_sharer_mask_S4 = 64'b0000000000000000000000000000000000000000000000000000010000000000;
    end
    else if (dir_data_sel_S4[11])
    begin
        dir_sharer_S4 = 6'd11;
        dir_sharer_mask_S4 = 64'b0000000000000000000000000000000000000000000000000000100000000000;
    end
    else if (dir_data_sel_S4[12])
    begin
        dir_sharer_S4 = 6'd12;
        dir_sharer_mask_S4 = 64'b0000000000000000000000000000000000000000000000000001000000000000;
    end
    else if (dir_data_sel_S4[13])
    begin
        dir_sharer_S4 = 6'd13;
        dir_sharer_mask_S4 = 64'b0000000000000000000000000000000000000000000000000010000000000000;
    end
    else if (dir_data_sel_S4[14])
    begin
        dir_sharer_S4 = 6'd14;
        dir_sharer_mask_S4 = 64'b0000000000000000000000000000000000000000000000000100000000000000;
    end
    else if (dir_data_sel_S4[15])
    begin
        dir_sharer_S4 = 6'd15;
        dir_sharer_mask_S4 = 64'b0000000000000000000000000000000000000000000000001000000000000000;
    end
    else if (dir_data_sel_S4[16])
    begin
        dir_sharer_S4 = 6'd16;
        dir_sharer_mask_S4 = 64'b0000000000000000000000000000000000000000000000010000000000000000;
    end
    else if (dir_data_sel_S4[17])
    begin
        dir_sharer_S4 = 6'd17;
        dir_sharer_mask_S4 = 64'b0000000000000000000000000000000000000000000000100000000000000000;
    end
    else if (dir_data_sel_S4[18])
    begin
        dir_sharer_S4 = 6'd18;
        dir_sharer_mask_S4 = 64'b0000000000000000000000000000000000000000000001000000000000000000;
    end
    else if (dir_data_sel_S4[19])
    begin
        dir_sharer_S4 = 6'd19;
        dir_sharer_mask_S4 = 64'b0000000000000000000000000000000000000000000010000000000000000000;
    end
    else if (dir_data_sel_S4[20])
    begin
        dir_sharer_S4 = 6'd20;
        dir_sharer_mask_S4 = 64'b0000000000000000000000000000000000000000000100000000000000000000;
    end
    else if (dir_data_sel_S4[21])
    begin
        dir_sharer_S4 = 6'd21;
        dir_sharer_mask_S4 = 64'b0000000000000000000000000000000000000000001000000000000000000000;
    end
    else if (dir_data_sel_S4[22])
    begin
        dir_sharer_S4 = 6'd22;
        dir_sharer_mask_S4 = 64'b0000000000000000000000000000000000000000010000000000000000000000;
    end
    else if (dir_data_sel_S4[23])
    begin
        dir_sharer_S4 = 6'd23;
        dir_sharer_mask_S4 = 64'b0000000000000000000000000000000000000000100000000000000000000000;
    end
    else if (dir_data_sel_S4[24])
    begin
        dir_sharer_S4 = 6'd24;
        dir_sharer_mask_S4 = 64'b0000000000000000000000000000000000000001000000000000000000000000;
    end
    else if (dir_data_sel_S4[25])
    begin
        dir_sharer_S4 = 6'd25;
        dir_sharer_mask_S4 = 64'b0000000000000000000000000000000000000010000000000000000000000000;
    end
    else if (dir_data_sel_S4[26])
    begin
        dir_sharer_S4 = 6'd26;
        dir_sharer_mask_S4 = 64'b0000000000000000000000000000000000000100000000000000000000000000;
    end
    else if (dir_data_sel_S4[27])
    begin
        dir_sharer_S4 = 6'd27;
        dir_sharer_mask_S4 = 64'b0000000000000000000000000000000000001000000000000000000000000000;
    end
    else if (dir_data_sel_S4[28])
    begin
        dir_sharer_S4 = 6'd28;
        dir_sharer_mask_S4 = 64'b0000000000000000000000000000000000010000000000000000000000000000;
    end
    else if (dir_data_sel_S4[29])
    begin
        dir_sharer_S4 = 6'd29;
        dir_sharer_mask_S4 = 64'b0000000000000000000000000000000000100000000000000000000000000000;
    end
    else if (dir_data_sel_S4[30])
    begin
        dir_sharer_S4 = 6'd30;
        dir_sharer_mask_S4 = 64'b0000000000000000000000000000000001000000000000000000000000000000;
    end
    else if (dir_data_sel_S4[31])
    begin
        dir_sharer_S4 = 6'd31;
        dir_sharer_mask_S4 = 64'b0000000000000000000000000000000010000000000000000000000000000000;
    end
    else if (dir_data_sel_S4[32])
    begin
        dir_sharer_S4 = 6'd32;
        dir_sharer_mask_S4 = 64'b0000000000000000000000000000000100000000000000000000000000000000;
    end
    else if (dir_data_sel_S4[33])
    begin
        dir_sharer_S4 = 6'd33;
        dir_sharer_mask_S4 = 64'b0000000000000000000000000000001000000000000000000000000000000000;
    end
    else if (dir_data_sel_S4[34])
    begin
        dir_sharer_S4 = 6'd34;
        dir_sharer_mask_S4 = 64'b0000000000000000000000000000010000000000000000000000000000000000;
    end
    else if (dir_data_sel_S4[35])
    begin
        dir_sharer_S4 = 6'd35;
        dir_sharer_mask_S4 = 64'b0000000000000000000000000000100000000000000000000000000000000000;
    end
    else if (dir_data_sel_S4[36])
    begin
        dir_sharer_S4 = 6'd36;
        dir_sharer_mask_S4 = 64'b0000000000000000000000000001000000000000000000000000000000000000;
    end
    else if (dir_data_sel_S4[37])
    begin
        dir_sharer_S4 = 6'd37;
        dir_sharer_mask_S4 = 64'b0000000000000000000000000010000000000000000000000000000000000000;
    end
    else if (dir_data_sel_S4[38])
    begin
        dir_sharer_S4 = 6'd38;
        dir_sharer_mask_S4 = 64'b0000000000000000000000000100000000000000000000000000000000000000;
    end
    else if (dir_data_sel_S4[39])
    begin
        dir_sharer_S4 = 6'd39;
        dir_sharer_mask_S4 = 64'b0000000000000000000000001000000000000000000000000000000000000000;
    end
    else if (dir_data_sel_S4[40])
    begin
        dir_sharer_S4 = 6'd40;
        dir_sharer_mask_S4 = 64'b0000000000000000000000010000000000000000000000000000000000000000;
    end
    else if (dir_data_sel_S4[41])
    begin
        dir_sharer_S4 = 6'd41;
        dir_sharer_mask_S4 = 64'b0000000000000000000000100000000000000000000000000000000000000000;
    end
    else if (dir_data_sel_S4[42])
    begin
        dir_sharer_S4 = 6'd42;
        dir_sharer_mask_S4 = 64'b0000000000000000000001000000000000000000000000000000000000000000;
    end
    else if (dir_data_sel_S4[43])
    begin
        dir_sharer_S4 = 6'd43;
        dir_sharer_mask_S4 = 64'b0000000000000000000010000000000000000000000000000000000000000000;
    end
    else if (dir_data_sel_S4[44])
    begin
        dir_sharer_S4 = 6'd44;
        dir_sharer_mask_S4 = 64'b0000000000000000000100000000000000000000000000000000000000000000;
    end
    else if (dir_data_sel_S4[45])
    begin
        dir_sharer_S4 = 6'd45;
        dir_sharer_mask_S4 = 64'b0000000000000000001000000000000000000000000000000000000000000000;
    end
    else if (dir_data_sel_S4[46])
    begin
        dir_sharer_S4 = 6'd46;
        dir_sharer_mask_S4 = 64'b0000000000000000010000000000000000000000000000000000000000000000;
    end
    else if (dir_data_sel_S4[47])
    begin
        dir_sharer_S4 = 6'd47;
        dir_sharer_mask_S4 = 64'b0000000000000000100000000000000000000000000000000000000000000000;
    end
    else if (dir_data_sel_S4[48])
    begin
        dir_sharer_S4 = 6'd48;
        dir_sharer_mask_S4 = 64'b0000000000000001000000000000000000000000000000000000000000000000;
    end
    else if (dir_data_sel_S4[49])
    begin
        dir_sharer_S4 = 6'd49;
        dir_sharer_mask_S4 = 64'b0000000000000010000000000000000000000000000000000000000000000000;
    end
    else if (dir_data_sel_S4[50])
    begin
        dir_sharer_S4 = 6'd50;
        dir_sharer_mask_S4 = 64'b0000000000000100000000000000000000000000000000000000000000000000;
    end
    else if (dir_data_sel_S4[51])
    begin
        dir_sharer_S4 = 6'd51;
        dir_sharer_mask_S4 = 64'b0000000000001000000000000000000000000000000000000000000000000000;
    end
    else if (dir_data_sel_S4[52])
    begin
        dir_sharer_S4 = 6'd52;
        dir_sharer_mask_S4 = 64'b0000000000010000000000000000000000000000000000000000000000000000;
    end
    else if (dir_data_sel_S4[53])
    begin
        dir_sharer_S4 = 6'd53;
        dir_sharer_mask_S4 = 64'b0000000000100000000000000000000000000000000000000000000000000000;
    end
    else if (dir_data_sel_S4[54])
    begin
        dir_sharer_S4 = 6'd54;
        dir_sharer_mask_S4 = 64'b0000000001000000000000000000000000000000000000000000000000000000;
    end
    else if (dir_data_sel_S4[55])
    begin
        dir_sharer_S4 = 6'd55;
        dir_sharer_mask_S4 = 64'b0000000010000000000000000000000000000000000000000000000000000000;
    end
    else if (dir_data_sel_S4[56])
    begin
        dir_sharer_S4 = 6'd56;
        dir_sharer_mask_S4 = 64'b0000000100000000000000000000000000000000000000000000000000000000;
    end
    else if (dir_data_sel_S4[57])
    begin
        dir_sharer_S4 = 6'd57;
        dir_sharer_mask_S4 = 64'b0000001000000000000000000000000000000000000000000000000000000000;
    end
    else if (dir_data_sel_S4[58])
    begin
        dir_sharer_S4 = 6'd58;
        dir_sharer_mask_S4 = 64'b0000010000000000000000000000000000000000000000000000000000000000;
    end
    else if (dir_data_sel_S4[59])
    begin
        dir_sharer_S4 = 6'd59;
        dir_sharer_mask_S4 = 64'b0000100000000000000000000000000000000000000000000000000000000000;
    end
    else if (dir_data_sel_S4[60])
    begin
        dir_sharer_S4 = 6'd60;
        dir_sharer_mask_S4 = 64'b0001000000000000000000000000000000000000000000000000000000000000;
    end
    else if (dir_data_sel_S4[61])
    begin
        dir_sharer_S4 = 6'd61;
        dir_sharer_mask_S4 = 64'b0010000000000000000000000000000000000000000000000000000000000000;
    end
    else if (dir_data_sel_S4[62])
    begin
        dir_sharer_S4 = 6'd62;
        dir_sharer_mask_S4 = 64'b0100000000000000000000000000000000000000000000000000000000000000;
    end
    else if (dir_data_sel_S4[63])
    begin
        dir_sharer_S4 = 6'd63;
        dir_sharer_mask_S4 = 64'b1000000000000000000000000000000000000000000000000000000000000000;
    end

end
*/


reg [6-1:0] dir_sharer_counter_S4_f;
reg [6-1:0] dir_sharer_counter_S4_next;

always @ *
begin
    if (!rst_n)
    begin
        dir_sharer_counter_S4_next = 1;
    end
    else if (msg_send_valid_S4 && msg_send_ready_S4 && (msg_send_type_pre_S4 == 8'd18))
    begin
        if (dir_data_stall_S4)
        begin
            dir_sharer_counter_S4_next = dir_sharer_counter_S4_f + 1;
        end
        else
        begin
            dir_sharer_counter_S4_next = 1;
        end
    end
    else
    begin
        dir_sharer_counter_S4_next = dir_sharer_counter_S4_f;
    end
end


always @ (posedge clk)
begin
    dir_sharer_counter_S4_f <= dir_sharer_counter_S4_next;
end


always @ *
begin
    dir_sharer_counter_S4 = dir_sharer_counter_S4_f;
end

always @ *
begin
    dir_data_trans_S4 = dir_data_sel_S4 & (dir_sharer_mask_S4);
end


localparam broadcast_state_0 = 1'b0;
localparam broadcast_state_1 = 1'b1;

reg broadcast_state_S4_f;
reg broadcast_state_S4_next;

always @ *
begin
    if (!rst_n)
    begin
        broadcast_state_S4_next = broadcast_state_0;
    end
    else if (valid_S4 && (~stall_S4))
    begin
        broadcast_state_S4_next = broadcast_state_0;
    end
    else if (valid_S4 && (l2_way_state_mesi_S4 == 2'b11) && (msg_send_type_S4 == 8'd18)
         && (~(msg_stall_S4 || smc_stall_S4 || (msg_send_valid_S4 && !msg_send_ready_S4)
         || global_stall_S4 || stall_inv_counter_S4 || broadcast_stall_S4)))
    begin
        if (broadcast_state_S4_f == broadcast_state_0)
        begin
            broadcast_state_S4_next = broadcast_state_1;
        end
        else
        begin
            broadcast_state_S4_next = broadcast_state_S4_f;
        end
    end
    else
    begin
        broadcast_state_S4_next = broadcast_state_S4_f;
    end
end

always @ (posedge clk)
begin
    broadcast_state_S4_f <= broadcast_state_S4_next;
end


always @ *
begin
    broadcast_stall_S4 = (l2_way_state_mesi_S4 == 2'b11) && (msg_send_type_S4 == 8'd18)
                      && (broadcast_state_S4_f == broadcast_state_0) && (~broadcast_counter_avail_S4)
                      && (~(msg_from_mshr_S4_f && mshr_smc_miss_S4_f));
end



always @ *
begin
    broadcast_counter_op_val_S4 = valid_S4 && (~stall_smc_buf_S4) && (~smc_stall_S4) && (l2_way_state_mesi_S4 == 2'b11)
    && (msg_send_type_S4 == 8'd18) && (~((broadcast_state_S4_f == broadcast_state_1) && broadcast_counter_zero_S4));
end


always @ *
begin
    if (broadcast_counter_op_val_S4)
    begin
        broadcast_counter_op_S4 = 2'd2;
    end
    else
    begin
        broadcast_counter_op_S4 = 2'd0;
    end
end




always @ *
begin
    if (l2_way_state_mesi_S4 == 2'b11)
    begin
        dir_data_stall_S4 = (msg_send_type_S4 == 8'd18) && (~broadcast_counter_max_S4);
    end
    else
    begin
        dir_data_stall_S4 = (msg_send_type_S4 == 8'd18) && (| dir_data_trans_S4[64-1:0]);
    end
end









always @ *
begin
    state_wr_en_real_S4 = valid_S4 && !dir_data_stall_S4 &&  msg_send_valid_S4 && (msg_send_type_pre_S4 == 8'd18);
end

//write to the state array if either the enable signal of S3 or the one from S2 is true
always @ *
begin
    if (load_store_mem_S4)
    begin
        state_wr_en_S4 = msg_send_valid_S4 && (msg_send_type_S4 == msg0_send_type_S4) && msg_send_ready_S4 && ~(req_recycle_S4 && ~special_addr_type_S4_f);
    end
    else
    begin
        state_wr_en_S4 = !stall_S4 && (state_wr_en_real_S4 || state_wr_en_S4_f) && ~(req_recycle_S4 && ~special_addr_type_S4_f);
    end
end



always @ *
begin
    smc_rd_en_S4 = valid_S4 &&
                ((special_addr_type_S4_f
                && (addr_S4[39:32] == 8'ha2)
                && (msg_type_S4 == 8'd14))
                || (csm_en
//To break timing loop
//&& msg_send_valid_S4
//&& cs_S4[`CS_MSG_SEND_FWD_S4]);
                && (((msg_send_type_pre_S4 == 8'd18) && (l2_way_state_mesi_S4 != 2'b11)))));
           //     || ((msg_send_type_pre_S4 == `MSG_TYPE_LOAD_FWD || msg_send_type_pre_S4 == `MSG_TYPE_STORE_FWD)
           //        && (l2_way_state_owner_S4 != `L2_PUBLIC_SHARER))));
end

always @ *
begin
    smc_rd_diag_en_S4 =
                (special_addr_type_S4_f
                && (addr_S4[39:32] == 8'ha2)
                && (msg_type_S4 == 8'd14));
end

always @ *
begin
    smc_miss_S4 = smc_rd_en_S4  && ~smc_rd_diag_en_S4 && (~smc_hit_S4);
end


always @ *
begin
    if (msg_type_S4_f == 8'd6 && (msg_send_valid_S4 && msg_send_type_S4 == 8'd29)
    && valid_S4 && !stall_S4)
    begin
        cas_cmp_en_S4 = y;
    end
    else
    begin
        cas_cmp_en_S4 = n;
    end
end

always @ *
begin
    if ((msg_type_S4_f == 8'd6
        || msg_type_S4_f == 8'd10
        || msg_type_S4_f == 8'd44
        || msg_type_S4_f == 8'd45
        || msg_type_S4_f == 8'd46
        || msg_type_S4_f == 8'd47
        || msg_type_S4_f == 8'd48
        || msg_type_S4_f == 8'd49
        || msg_type_S4_f == 8'd50
        || msg_type_S4_f == 8'd51)
    && (msg_send_valid_S4 && msg_send_type_S4 == 8'd29)
    && valid_S4 && !stall_S4)
    begin
        atomic_read_data_en_S4 = y;
    end
    else
    begin
        atomic_read_data_en_S4 = n;
    end
end


always @ *
begin
    cas_cmp_data_size_S4 = data_size_S4_f;
end

always @ *
begin
    reg_rd_en_S4 = valid_S4  && (msg_type_S4 == 8'd14)
               && ((addr_type_S4 == 8'ha9)
                || (addr_type_S4 == 8'ha7)
                || (addr_type_S4 == 8'ha8)
                || (addr_type_S4 == 8'haa)
                || (addr_type_S4 == 8'hab));
end

always @ *
begin
    reg_rd_addr_type_S4 = addr_type_S4;
end

always @ *
begin
    if (state_wr_en_real_S4)
    begin
        state_wr_sel_S4 = 1'b1;
    end
    else
    begin
        state_wr_sel_S4 = 1'b0;
    end
end

always @ *
begin
    
    if (mshr_smc_miss_S4_f)
    begin
        mshr_wr_index_in_S4 = mshr_pending_index_S4_f;
    end
    else
    
    begin
        mshr_wr_index_in_S4 = mshr_empty_index_sel_S4;
    end
end

always @ *
begin
    mshr_inv_counter_rd_index_in_S4 = mshr_wr_index_in_S4;
end

//wait for inv_fwdack to write to mshr in pipeline2
always @ *
begin
    stall_inv_counter_S4 = valid_S4 && ((global_stall_S1 && (pipe2_msg_type_S1 == 8'd23))
                                     || (global_stall_S2 && (pipe2_msg_type_S2 == 8'd23)))
                       && (msg_send_type_pre_S4 == 8'd18);
                   //    && ~(dir_data_stall_S4|| msg_stall_S4 || global_stall_S4);
end



always @ *
begin
    stall_smc_buf_S4 = valid_S4 && (global_stall_S4
           || (msg_send_valid_S4 && !msg_send_ready_S4)
           || broadcast_stall_S4
           || stall_inv_counter_S4);
end


always @ *
begin
    stall_S4 = valid_S4 && (global_stall_S4 || msg_stall_S4 || dir_data_stall_S4
           || (msg_send_valid_S4 && !msg_send_ready_S4)
           || stall_inv_counter_S4
           || broadcast_stall_S4
           || smc_stall_S4);
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

//==================================================================================================
//  Filename      : l2_pipe1_dpath.v
//  Created On    : 2014-02-24
//  Revision      :
//  Author        : Yaosheng Fu
//  Company       : Princeton University
//  Email         : yfu@princeton.edu
//
//  Description   : The datapath for pipeline1 in the L2 cache
//
//
//==================================================================================================


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
//  Filename      : l2.h.pyv
//  Created On    : 2014-02-20
//  Revision      :
//  Author        : Yaosheng Fu
//  Company       : Princeton University
//  Email         : yfu@princeton.edu
//
//  Description   : header file for the L2 cache
//
//
//==================================================================================================

// devices.xml



// Input buffer for pipeline1













// Input buffer for pipeline2














// Output buffer















// L2 cache configuration







//`define L2_SIZE                  65536
//`define L2_SIZE_WIDTH            16






//`define L2_LINE_SIZE             64
//`define L2_LINE_SIZE_WIDTH       6













//`define L2_WAYS                  4
//`define L2_WAYS_WIDTH            2
//`define L2_WAY_0                 2'b00
//`define L2_WAY_1                 2'b01
//`define L2_WAY_2                 2'b10
//`define L2_WAY_3                 2'b11





// Tag array







//`define L2_TAG_INDEX_WIDTH      8
//`define L2_TAG_WIDTH            26
//`define L2_TAG_WAY_WIDTH        26
//`define L2_TAG_ARRAY_WIDTH      104


// Tag Address decomposition 




//`define L2_TAG_INDEX            13:6
//`define L2_TAG                  39:14


//Data array












//`define L2_DATA_INDEX_WIDTH         12 








// Data Address decomposition 







//Dir array



//`define L2_DIR_INDEX_WIDTH      10 





//State array

















//Whether the cache line is in Icaches or Dcaches
















//Round Robin selection














// State decomposition 












//MSHR array



































//`define L2_MSHR_CMP_ADDR        13:6
//`define L2_MSHR_ADDR            39:0
//`define L2_MSHR_WAY             41:40
//`define L2_MSHR_MSHRID          49:42
//`define L2_MSHR_CACHE_TYPE      50
//`define L2_MSHR_DATA_SIZE       53:51
//`define L2_MSHR_MSG_TYPE        61:54
//`define L2_MSHR_L2_MISS         62
//`define L2_MSHR_SRC_CHIPID      76:63
//`define L2_MSHR_SRC_X           84:77
//`define L2_MSHR_SRC_Y           92:85
//`define L2_MSHR_SRC_FBITS       96:93
//`define L2_MSHR_SDID            106:97
//`define L2_MSHR_LSID            112:107      
//`define L2_MSHR_MISS_LSID       118:113
//`define L2_MSHR_SMC_MISS        119
//`define L2_MSHR_RECYCLED        120
//`define L2_MSHR_INV_FWD_PENDING 121


//SMC array


































//Message destination_type







//L2 public sharer beyond the maximum clump size



//L2 registers



//Special addresses





















//L2 core ID





//L2 dir array reuse







//Control Signal 








//AMO ALU OP macros












//Control Signal in Stage 1























































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


module l2_pipe1_dpath(

    input wire clk,
    input wire rst_n,
    
    input wire csm_en,
    
    input wire [22-1:0] smt_base_addr,

    //Inputs to Stage 1   


    //inputs from the mshr
    input wire [40-1:0] mshr_addr_S1,
    input wire [8-1:0] mshr_mshrid_S1,
    input wire [2-1:0] mshr_way_S1,
    input wire [14-1:0] mshr_src_chipid_S1,
    input wire [8-1:0] mshr_src_x_S1,
    input wire [8-1:0] mshr_src_y_S1,
    input wire [4-1:0] mshr_src_fbits_S1,
    input wire [10-1:0] mshr_sdid_S1,
    input wire [6-1:0] mshr_lsid_S1,
    input wire [6-1:0] mshr_miss_lsid_S1,
    input wire mshr_recycled_S1,


























 // L2_CAM_MSHR
    input wire dis_flush_S1,

    //msg info from the input buffer
    input wire [40-1:0] msg_addr_S1,
    input wire [8-1:0] msg_mshrid_S1,
    input wire [14-1:0] msg_src_chipid_S1,
    input wire [8-1:0] msg_src_x_S1,
    input wire [8-1:0] msg_src_y_S1,
    input wire [4-1:0] msg_src_fbits_S1,
    input wire [10-1:0] msg_sdid_S1,
    input wire [6-1:0] msg_lsid_S1,
    
    input wire [64-1:0] msg_data_S1,

    //control signals from ctrl
    input wire valid_S1,
    input wire stall_S1,
    input wire msg_from_mshr_S1, 
    
 
    //Inputs to Stage 2   
    //input from the input buffer
    input wire [64-1:0] msg_data_S2,
   //input from the state array
    input wire [15*4+2+4-1:0] state_data_S2,
    
    //input from the tag array 
    input wire [104-1:0] tag_data_S2,


    //control signals from ctrl
    input wire msg_from_mshr_S2,
    input wire special_addr_type_S2,
    input wire [8-1:0] msg_type_S2,
    input wire [3-1:0] data_size_S2,
    input wire [1-1:0] cache_type_S2,
    input wire [2-1:0] dir_op_S2,
    input wire state_owner_en_S2,
    input wire [2-1:0] state_owner_op_S2,
    input wire state_subline_en_S2,
    input wire [2-1:0] state_subline_op_S2,
    input wire state_di_en_S2,
    input wire state_vd_en_S2,
    input wire [2-1:0] state_vd_S2,
    input wire state_mesi_en_S2,
    input wire [2-1:0] state_mesi_S2,
    input wire state_lru_en_S2,
    input wire [1-1:0] state_lru_op_S2,
    input wire state_rb_en_S2,
    input wire l2_ifill_S2,
    input wire [2-1:0] l2_load_data_subline_S2,
    input wire valid_S2,
    input wire stall_S2,
    input wire stall_before_S2,
    input wire state_load_sdid_S2, 
    input wire data_clk_en_S2,
    input wire stall_real_S2,
    input wire [4-1:0] amo_alu_op_S2,

    //Inputs to Stage 3

    //input from the data array
    input wire [144-1:0] data_data_S3,
    input wire valid_S3,
    input wire stall_S3,
    input wire stall_before_S3, 


    //Inputs to Stage 4
    //control signals from ctrl
    input wire valid_S4,
    input wire stall_S4,
    input wire stall_before_S4, 
    input wire cas_cmp_en_S4,
    input wire atomic_read_data_en_S4,
    input wire [3-1:0] cas_cmp_data_size_S4,
    input wire [6-1:0] dir_sharer_S4,
    input wire [6-1:0] dir_sharer_counter_S4,
    input wire [6-1:0] mshr_inv_counter_out_S4,
    input wire [8-1:0] msg_send_type_S4,
    input wire [8-1:0] msg_send_type_pre_S4,
    input wire state_wr_sel_S4,
    input wire [8-1:0] msg_type_S4,
    input wire [3-1:0] data_size_S4,
    input wire [1-1:0] cache_type_S4,
    input wire [1-1:0] l2_miss_S4,
    
    input wire smc_miss_S4,
    
    input wire special_addr_type_S4,
    input wire [64-1:0] dir_data_sel_S4,
    input wire [64-1:0] dir_data_S4,
    
    input wire stall_smc_buf_S4,
    
    input wire msg_from_mshr_S4,
    input wire req_recycle_S4,
    input wire inv_fwd_pending_S4,

    
    //input from the broadcast counter
    input wire [14-1:0] broadcast_chipid_out_S4,
    input wire [8-1:0] broadcast_x_out_S4,
    input wire [8-1:0] broadcast_y_out_S4,
    
    //input from the smc
    
    input wire [30-1:0] smc_data_out_S4,
    input wire [4-1:0] smc_valid_out_S4,
    input wire [14-1:0] smc_tag_out_S4,
    
    //node id from id register
    input wire [14-1:0] my_nodeid_chipid_S4,
    input wire [8-1:0] my_nodeid_x_S4,
    input wire [8-1:0] my_nodeid_y_S4,
    input wire [64-1:0] reg_data_out_S4,
    

    //Outputs from Stage 1
    
    output reg [40-1:0] addr_S1,
    output reg [8-1:0] mshr_addr_in_S1,
    output reg [8-1:0] tag_addr_S1,
    output reg [8-1:0] state_rd_addr_S1,
    output reg [64-1:0] reg_data_in_S1,

    output reg [104-1:0] tag_data_in_S1,
    output reg [104-1:0] tag_data_mask_in_S1,
    //Outputs from Stage 2
   
 
    output reg [40-1:0] addr_S2,
    output reg l2_tag_hit_S2,
    output reg l2_evict_S2,
    output reg l2_wb_S2,
    output reg [2-1:0] l2_way_state_mesi_S2,
    output reg [2-1:0] l2_way_state_vd_S2,
    output reg [1-1:0] l2_way_state_cache_type_S2,
    output reg [4-1:0] l2_way_state_subline_S2,
    output reg req_from_owner_S2,
    output reg addr_l2_aligned_S2,
    output reg [6-1:0] lsid_S2,

    output reg [8+2-1:0] dir_addr_S2,
    output reg [64-1:0] dir_data_in_S2,
    output reg [64-1:0] dir_data_mask_in_S2,

    output reg [8+2+2-1:0] data_addr_S2,
    output reg [144-1:0] data_data_in_S2,
    output reg [144-1:0] data_data_mask_in_S2,

    
    output reg [16-1:0] smc_wr_addr_in_S2,
    output reg [128-1:0] smc_data_in_S2,
    

    //Outputs from Stage 3
    output reg [40-1:0] addr_S3,

    //Outputs from Stage 4
    output reg [40-1:0] addr_S4,
    output reg [8+2+2-1:0] data_addr_S4,

    output reg l2_tag_hit_S4,
    output reg l2_evict_S4,
    output reg [2-1:0] l2_way_state_mesi_S4,
    output reg [6-1:0] l2_way_state_owner_S4,
    output reg [2-1:0] l2_way_state_vd_S4,
    output reg [4-1:0] l2_way_state_subline_S4,
    output reg [1-1:0] l2_way_state_cache_type_S4,
    output reg [8-1:0] mshrid_S4,
    output reg req_from_owner_S4,
    output reg cas_cmp_S4,
    output reg [6-1:0] mshr_miss_lsid_S4,
    output reg [6-1:0] lsid_S4,
    output reg corr_error_S4,
    output reg uncorr_error_S4,

    output reg [40-1:0] msg_send_addr_S4,
    output reg [14-1:0] msg_send_dst_chipid_S4,
    output reg [8-1:0] msg_send_dst_x_S4,
    output reg [8-1:0] msg_send_dst_y_S4,
    output reg [4-1:0] msg_send_dst_fbits_S4,
    output reg [128-1:0] msg_send_data_S4,

    output reg [120+2-1:0] mshr_data_in_S4,
    output wire [120+2-1:0] mshr_data_mask_in_S4,


    
    output reg [16-1:0] smc_rd_addr_in_buf_S4,
    

    output reg [8-1:0] state_wr_addr_S4,
    output reg [15*4+2+4-1:0] state_data_in_S4,
    output reg [15*4+2+4-1:0] state_data_mask_in_S4

);


localparam y = 1'b1;
localparam n = 1'b0;


//used by stage 1
wire [128-1:0] data_data_ecc_S4;
//============================
// Stage 1
//============================

reg [8-1:0] mshrid_S1;
reg [14-1:0] src_chipid_S1;
reg [8-1:0] src_x_S1;
reg [8-1:0] src_y_S1;
reg [4-1:0] src_fbits_S1;
reg [10-1:0] sdid_S1;
reg [6-1:0] lsid_S1;
reg [40-1:0] addr_trans_S1;
reg recycled_S1;

always @ *
begin
    if (msg_from_mshr_S1)
    begin

        addr_S1 = mshr_addr_S1;
        mshrid_S1 = mshr_mshrid_S1;
        src_chipid_S1 = mshr_src_chipid_S1;
        src_x_S1 = mshr_src_x_S1;   
        src_y_S1 = mshr_src_y_S1;   
        src_fbits_S1 = mshr_src_fbits_S1;
        sdid_S1 = mshr_sdid_S1;
        lsid_S1 = mshr_lsid_S1;
        recycled_S1 = mshr_recycled_S1;










 // L2_CAM_MSHR
    end
    else
    begin
        addr_S1 = msg_addr_S1;
        mshrid_S1 = msg_mshrid_S1;
        src_chipid_S1 = msg_src_chipid_S1;
        src_x_S1 = msg_src_x_S1;   
        src_y_S1 = msg_src_y_S1;   
        src_fbits_S1 = msg_src_fbits_S1;
        sdid_S1 = msg_sdid_S1;
        lsid_S1 = msg_lsid_S1;
        recycled_S1 = 1'b0;
    end
end

always @ *
begin
    if (dis_flush_S1)
    begin
        //address reorder for displacement flush
        addr_trans_S1 = {addr_S1[5:0],addr_S1[33:6],6'd0};
    end
    else
    begin
        addr_trans_S1 = addr_S1;
    end
end


always @ *
begin
    if (~msg_from_mshr_S1)
    begin
        mshr_addr_in_S1 = addr_trans_S1[6+8-1:6];
    end
    else 
    begin
        mshr_addr_in_S1 = {8{1'b0}};
    end
end

always @ *
begin
    tag_addr_S1 = addr_trans_S1[6+8-1:6];
end

always @ *
begin
    state_rd_addr_S1 = addr_trans_S1[6+8-1:6];
end


//the cache line read by the 1st phase of atomic instructions
reg [128-1:0] atomic_read_data_S1_f;
reg [128-1:0] atomic_read_data_S1_next;

always @ *
begin
    if (!rst_n)
    begin
        atomic_read_data_S1_next = 0;
    end
    else if (atomic_read_data_en_S4)
    begin
        atomic_read_data_S1_next = data_data_ecc_S4;
    end
    else
    begin
        atomic_read_data_S1_next = atomic_read_data_S1_f;
    end
end


always @ (posedge clk)
begin
    atomic_read_data_S1_f <= atomic_read_data_S1_next;
end

always @ *
begin
    reg_data_in_S1 = msg_data_S1;
end


always @ *
begin
    tag_data_in_S1 = {4{msg_data_S1[26-1:0]}};
end

always @ *
begin
    tag_data_mask_in_S1 = {{(4-1)*26{1'b0}},{26{1'b1}}} 
                       << (addr_trans_S1[6+8+2-1:6+8] * 26);
end


//============================
// Stage 1 -> Stage 2
//============================


reg [40-1:0] addr_S2_f;
reg [8-1:0] mshrid_S2_f;
reg [14-1:0] src_chipid_S2_f;
reg [8-1:0] src_x_S2_f;
reg [8-1:0] src_y_S2_f;
reg [4-1:0] src_fbits_S2_f;
reg [10-1:0] sdid_S2_f;
reg [6-1:0] lsid_S2_f;
reg [2-1:0] mshr_way_S2_f;
reg [6-1:0] mshr_miss_lsid_S2_f;
reg [128-1:0] atomic_read_data_S2_f;
reg recycled_S2_f;

always @ (posedge clk)
begin
    if (!rst_n)
    begin
        addr_S2_f <= 0; 
        mshrid_S2_f <= 0;
        src_chipid_S2_f <= 0;
        src_x_S2_f <= 0;
        src_y_S2_f <= 0;
        src_fbits_S2_f <= 0;
        sdid_S2_f <= 0;
        lsid_S2_f <= 0;
        mshr_way_S2_f <= 0;
        mshr_miss_lsid_S2_f <= 0;
        atomic_read_data_S2_f <= 0;
        recycled_S2_f <= 0;
    end
    else if (!stall_S2)
    begin
        addr_S2_f <= addr_trans_S1;
        mshrid_S2_f <= mshrid_S1;
        src_chipid_S2_f <= src_chipid_S1;
        src_x_S2_f <= src_x_S1;
        src_y_S2_f <= src_y_S1;
        src_fbits_S2_f <= src_fbits_S1;
        sdid_S2_f <= sdid_S1;
        lsid_S2_f <= lsid_S1;

        mshr_way_S2_f <= mshr_way_S1;
        mshr_miss_lsid_S2_f <= mshr_miss_lsid_S1;






 // L2_CAM_MSHR
        atomic_read_data_S2_f <= atomic_read_data_S1_f;
        recycled_S2_f <= recycled_S1;
    end
end


//============================
// Stage 2
//============================


reg [64-1:0] return_data_S2;
reg [2-1:0] l2_way_sel_S2;
reg [15*4+2+4-1:0] state_data_in_S2;
reg [15*4+2+4-1:0] state_data_mask_in_S2;


always @ *
begin
    addr_S2 = addr_S2_f;
    lsid_S2 = lsid_S2_f;
end

reg [104-1:0] tag_data_buf_S2_f;
reg [104-1:0] tag_data_buf_S2_next;
reg [104-1:0] tag_data_trans_S2;

always @ *
begin
    if (!rst_n)
    begin
        tag_data_buf_S2_next = 0;
    end
    else if (stall_S2 && !stall_before_S2)
    begin
        tag_data_buf_S2_next = tag_data_S2;
    end
    else
    begin
        tag_data_buf_S2_next = tag_data_buf_S2_f;
    end
end


always @ (posedge clk)
begin
    tag_data_buf_S2_f <= tag_data_buf_S2_next;
end


//choose between the direct output and buffered output of the tag array based on stall situation
always @ *
begin
    if (stall_before_S2)
    begin
        tag_data_trans_S2 = tag_data_buf_S2_f;
    end
    else
    begin
        tag_data_trans_S2 = tag_data_S2;
    end
end


reg [15*4+2+4-1:0] state_data_buf_S2_f;
reg [15*4+2+4-1:0] state_data_buf_S2_next;
reg [15*4+2+4-1:0] state_data_trans_S2;

always @ *
begin
    if (!rst_n)
    begin
        state_data_buf_S2_next = 0;
    end
    else if (stall_S2 && !stall_before_S2)
    begin
        state_data_buf_S2_next = state_data_S2;
    end
    else
    begin
        state_data_buf_S2_next = state_data_buf_S2_f;
    end
end


always @ (posedge clk)
begin
    state_data_buf_S2_f <= state_data_buf_S2_next;
end


//choose between the direct output and buffered output of the state array based on stall situation
always @ *
begin
    if (stall_before_S2)
    begin
        state_data_trans_S2 = state_data_buf_S2_f;
    end
    else
    begin
        state_data_trans_S2 = state_data_S2;
    end
end


wire [2-1:0] l2_hit_way_sel_S2;
reg [2-1:0] l2_evict_way_sel_S2;
reg [2-1:0] l2_rb_bits_S2;
reg [4-1:0] l2_lru_bits_S2;


always @ *
begin
    l2_rb_bits_S2 = state_data_trans_S2[15*4+2+4-1:15*4+4];
    l2_lru_bits_S2 = state_data_trans_S2[15*4+4-1:15*4];
end


reg [26 - 1:0] tag_data_way_S2 [3:0];


reg [3:0] tag_hit_way_S2;


reg [15 - 1:0] state_way_S2 [3:0];







always @ *
begin
    tag_data_way_S2[0] = tag_data_trans_S2[26 * 1 - 1: 26 * 0];
    tag_data_way_S2[1] = tag_data_trans_S2[26 * 2 - 1: 26 * 1];
    tag_data_way_S2[2] = tag_data_trans_S2[26 * 3 - 1: 26 * 2];
    tag_data_way_S2[3] = tag_data_trans_S2[26 * 4 - 1: 26 * 3];

end

always @ *
begin
    state_way_S2[0] = state_data_trans_S2[15 * 1 - 1: 
15 * 0];
    state_way_S2[1] = state_data_trans_S2[15 * 2 - 1: 
15 * 1];
    state_way_S2[2] = state_data_trans_S2[15 * 3 - 1: 
15 * 2];
    state_way_S2[3] = state_data_trans_S2[15 * 4 - 1: 
15 * 3];

end

always @ *
begin
    if ((addr_S2_f[39:6+8] == tag_data_way_S2[0]) && 
(state_way_S2[0][12:11] == 2'b10 || state_way_S2[0][12:11] == 2'b11 ))
    begin
        tag_hit_way_S2[0] = 1'b1;
    end
    else
    begin
        tag_hit_way_S2[0] = 1'b0;
    end
end
always @ *
begin
    if ((addr_S2_f[39:6+8] == tag_data_way_S2[1]) && 
(state_way_S2[1][12:11] == 2'b10 || state_way_S2[1][12:11] == 2'b11 ))
    begin
        tag_hit_way_S2[1] = 1'b1;
    end
    else
    begin
        tag_hit_way_S2[1] = 1'b0;
    end
end
always @ *
begin
    if ((addr_S2_f[39:6+8] == tag_data_way_S2[2]) && 
(state_way_S2[2][12:11] == 2'b10 || state_way_S2[2][12:11] == 2'b11 ))
    begin
        tag_hit_way_S2[2] = 1'b1;
    end
    else
    begin
        tag_hit_way_S2[2] = 1'b0;
    end
end
always @ *
begin
    if ((addr_S2_f[39:6+8] == tag_data_way_S2[3]) && 
(state_way_S2[3][12:11] == 2'b10 || state_way_S2[3][12:11] == 2'b11 ))
    begin
        tag_hit_way_S2[3] = 1'b1;
    end
    else
    begin
        tag_hit_way_S2[3] = 1'b0;
    end
end



wire l2_tag_cmp_hit_S2;


l2_priority_encoder_2 priority_encoder_tag_cmp_2bits( 

    .data_in        (tag_hit_way_S2),
    .data_out       (l2_hit_way_sel_S2),
    .data_out_mask  (),
    .nonzero_out    (l2_tag_cmp_hit_S2)
);



always @ *
begin
    if (special_addr_type_S2 || msg_type_S2 == 8'd34)
    begin
        l2_tag_hit_S2 = 1'b0;
    end
    else
    begin
        l2_tag_hit_S2 = l2_tag_cmp_hit_S2;
    end
end
/*
            l2_tag_hit_S2 = tag_hit_way_S2[0] || tag_hit_way_S2[1] || tag_hit_way_S2[2] || tag_hit_way_S2[3];

    end
end



always @ *
begin
    l2_hit_way_sel_S2 = {`L2_WAYS_WIDTH{1'b0}};
    if (tag_hit_way_S2[0])
    begin
        l2_hit_way_sel_S2 = `L2_WAY_0;
    end
    if (tag_hit_way_S2[1])
    begin
        l2_hit_way_sel_S2 = `L2_WAY_1;
    end
    if (tag_hit_way_S2[2])
    begin
        l2_hit_way_sel_S2 = `L2_WAY_2;
    end
    if (tag_hit_way_S2[3])
    begin
        l2_hit_way_sel_S2 = `L2_WAY_3;
    end

end
*/
 
//pseudo LRU algorithm 
always @ *
begin

     if (!state_way_S2[0][12:11]) 
        begin
            l2_evict_way_sel_S2 = 2'd0;
        end
     else if (!state_way_S2[1][12:11]) 
        begin
            l2_evict_way_sel_S2 = 2'd1;
        end
     else if (!state_way_S2[2][12:11]) 
        begin
            l2_evict_way_sel_S2 = 2'd2;
        end
     else if (!state_way_S2[3][12:11]) 
        begin
            l2_evict_way_sel_S2 = 2'd3;
        end

    else
    begin
    case (l2_rb_bits_S2)
    2'd0:
    begin
        if (!l2_lru_bits_S2[0])
        begin
            l2_evict_way_sel_S2 = 2'd0;
        end
        else if (!l2_lru_bits_S2[1])
        begin
            l2_evict_way_sel_S2 = 2'd1;
        end
        else if (!l2_lru_bits_S2[2])
        begin
            l2_evict_way_sel_S2 = 2'd2;
        end
        else
        begin
            l2_evict_way_sel_S2 = 2'd3;
        end
    end
    2'd1:
    begin
        if (!l2_lru_bits_S2[1])
        begin
            l2_evict_way_sel_S2 = 2'd1;
        end
        else if (!l2_lru_bits_S2[2])
        begin
            l2_evict_way_sel_S2 = 2'd2;
        end
        else if (!l2_lru_bits_S2[3])
        begin
            l2_evict_way_sel_S2 = 2'd3;
        end
        else
        begin
            l2_evict_way_sel_S2 = 2'd0;
        end
    end
    2'd2:
    begin
        if (!l2_lru_bits_S2[2])
        begin
            l2_evict_way_sel_S2 = 2'd2;
        end
        else if (!l2_lru_bits_S2[3])
        begin
            l2_evict_way_sel_S2 = 2'd3;
        end
        else if (!l2_lru_bits_S2[0])
        begin
            l2_evict_way_sel_S2 = 2'd0;
        end
        else
        begin
            l2_evict_way_sel_S2 = 2'd1;
        end
    end
    2'd3:
    begin
        if (!l2_lru_bits_S2[3])
        begin
            l2_evict_way_sel_S2 = 2'd3;
        end
        else if (!l2_lru_bits_S2[0])
        begin
            l2_evict_way_sel_S2 = 2'd0;
        end
        else if (!l2_lru_bits_S2[1])
        begin
            l2_evict_way_sel_S2 = 2'd1;
        end
        else
        begin
            l2_evict_way_sel_S2 = 2'd2;
        end
    end

    default:
    begin
        l2_evict_way_sel_S2 = 2'd0;
    end
    endcase
    end
end


always @ *
begin
/*
    if (msg_from_mshr_S2)
    begin
        l2_way_sel_S2 = mshr_way_S2_f;
    end
    else 
*/
    if (special_addr_type_S2 || msg_type_S2 == 8'd34)
    begin
        l2_way_sel_S2 = addr_S2[6+8+2-1:6+8];
    end
    else if (l2_tag_hit_S2)
    begin
        l2_way_sel_S2 = l2_hit_way_sel_S2;
    end
    else   
    begin
        l2_way_sel_S2 = l2_evict_way_sel_S2;
    end
end

always @ *
begin
    if (special_addr_type_S2 
     || msg_type_S2 == 8'd13
     || msg_type_S2 == 8'd15)
    begin
        l2_evict_S2 = 1'b0;
    end
    else if (!l2_tag_hit_S2 && (state_way_S2[l2_way_sel_S2][12:11] == 2'b10 || 
        state_way_S2[l2_way_sel_S2][12:11] == 2'b11))
    begin
        l2_evict_S2 = 1'b1;
    end
    else
    begin
        l2_evict_S2 = 1'b0;
    end
end

always @ *
begin
    if (special_addr_type_S2
     || msg_type_S2 == 8'd13)
    begin
        l2_wb_S2 = 1'b0;
    end
    else if (((!l2_tag_hit_S2 && (msg_type_S2 != 8'd15))
           || (msg_type_S2 == 8'd14 || msg_type_S2 == 8'd35)
           || (l2_tag_hit_S2 && msg_type_S2 == 8'd15))
    && (state_way_S2[l2_way_sel_S2][14:13] == 2'b00)
    && (state_way_S2[l2_way_sel_S2][12:11] == 2'b11))
    begin
        l2_wb_S2 = 1'b1;
    end
    else
    begin
        l2_wb_S2 = 1'b0;
    end
end


reg [6-1:0] l2_way_state_owner_S2;

always @ *
begin
    l2_way_state_mesi_S2 = state_way_S2[l2_way_sel_S2][14:13];
    l2_way_state_vd_S2 = state_way_S2[l2_way_sel_S2][12:11];
    l2_way_state_subline_S2 = state_way_S2[l2_way_sel_S2][9:6];
    l2_way_state_cache_type_S2 = state_way_S2[l2_way_sel_S2][10];
    l2_way_state_owner_S2 = state_way_S2[l2_way_sel_S2][5:0];
end



always @ *
begin
    
    if (csm_en)
    begin
        req_from_owner_S2 = (l2_way_state_owner_S2 == lsid_S2_f) && (lsid_S2_f != 6'd63);
    end
    else
    
    begin
        req_from_owner_S2 = (l2_way_state_owner_S2 == {src_y_S2_f[2:0], src_x_S2_f[2:0]});
    end
end


always @ *
begin
    dir_addr_S2 = {addr_S2_f[6+8-1:6],l2_way_sel_S2}; 
end

always @ *
begin
    if (l2_wb_S2)
    begin
        data_addr_S2 = {addr_S2_f[6+8-1:6],l2_way_sel_S2, l2_load_data_subline_S2};
    end
    else if (l2_ifill_S2)
    begin
        data_addr_S2 = {addr_S2_f[6+8-1:6],l2_way_sel_S2, 
                        addr_S2_f[5], l2_load_data_subline_S2[0]};
    end
    else
    begin
        data_addr_S2 = {addr_S2_f[6+8-1:6],l2_way_sel_S2, addr_S2_f[5:4]};
    end
end

reg [40-1:0] evict_addr_S2;

always @ *
begin
    evict_addr_S2 = {tag_data_way_S2[l2_way_sel_S2], addr_S2_f[6+8-1:6], {6{1'b0}}}; 
end



always @ *
begin
    addr_l2_aligned_S2 = (addr_S2_f[6-1:0] == {6{1'b0}}); 
end



always @ *
begin
    if (special_addr_type_S2)
    begin  
        if (addr_S2[39:32] == 8'ha4)
        begin
            return_data_S2 = {{(64 - 26){1'b0}}, tag_data_way_S2[l2_way_sel_S2]};
        end
        else if (addr_S2[39:32] == 8'ha6)
        begin
        // State access is only available when L2_WAYS is less than 8
            if (addr_S2[31:30] == {2{1'b0}})
            begin
                return_data_S2 = {{(64 - 15*4){1'b0}}, state_data_trans_S2[15*4-1:0]};
            end
            else
            begin
                return_data_S2 = {{(64 - 2 - 4){1'b0}}, l2_rb_bits_S2, l2_lru_bits_S2};
            end
        end
        else
        begin
            return_data_S2 = {64{1'b0}};
        end 
    end
    else
    begin
        return_data_S2 = {64{1'b0}};
    end
end




always @ *
begin
    if (special_addr_type_S2)
    begin  
        dir_data_in_S2 = msg_data_S2;  
    end
    else
    begin
        //track owner beyond the domain scope
        
        if (csm_en && (dir_op_S2 == 2'd1))
        begin
            dir_data_in_S2 = {sdid_S2_f, src_chipid_S2_f, src_y_S2_f,src_x_S2_f};  
        end
        else        
        
        begin
            dir_data_in_S2 = {64{1'b1}}; 
        end 
    end
end

always @ *
begin
    if (special_addr_type_S2)
    begin  
        dir_data_mask_in_S2 = {64{1'b1}};
    end
    else
    begin
        
        if (csm_en)
        begin
            if (dir_op_S2 == 2'd1)
            begin
                dir_data_mask_in_S2 = {64{1'b1}};
            end
            else
            begin
                dir_data_mask_in_S2 = {{(64-1){1'b0}},1'b1} << lsid_S2_f;
            end
        end
        else    
        
        begin
            dir_data_mask_in_S2 = {{(64-1){1'b0}},1'b1} << {src_y_S2_f[2:0], src_x_S2_f[2:0]};
        end
    end 
end

reg [128-1:0] msg_data_mask_in_S2;
reg [128-1:0] data_data_merge_S2;
wire [8-1:0] data_data_parity1_S2;
wire [8-1:0] data_data_parity2_S2;
wire [128-1:0] amo_result_S2;

always @ *
begin
    if (data_size_S2 == 3'b001)
    begin
        msg_data_mask_in_S2 = {{8{1'b1}}, {(128-8){1'b0}}};
        msg_data_mask_in_S2 = msg_data_mask_in_S2 >> (8*addr_S2_f[3:0]);
    end
    else if (data_size_S2 == 3'b010)
    begin
        msg_data_mask_in_S2 = {{16{1'b1}}, {(128-16){1'b0}}};
        msg_data_mask_in_S2 = msg_data_mask_in_S2 >> (16*addr_S2_f[3:1]);
    end
    else if (data_size_S2 == 3'b011)
    begin
        msg_data_mask_in_S2 = {{32{1'b1}}, {(128-32){1'b0}}};
        msg_data_mask_in_S2 = msg_data_mask_in_S2 >> (32*addr_S2_f[3:2]);
    end
    else
    begin   
        msg_data_mask_in_S2 = {128{1'b1}}; 
    end
    msg_data_mask_in_S2 = {msg_data_mask_in_S2[63:0], msg_data_mask_in_S2[127:64]};
end

l2_amo_alu l2_amo_alu (
    .amo_alu_op     (amo_alu_op_S2),
    .address        (addr_S2_f),
    .data_size      (data_size_S2),
    .memory_operand (atomic_read_data_S2_f),
    .cpu_operand    ({msg_data_S2, msg_data_S2}),
    .amo_result     (amo_result_S2)
);

always @ *
begin
    data_data_merge_S2 = ({msg_data_S2, msg_data_S2} & msg_data_mask_in_S2)
                       | (atomic_read_data_S2_f & ~msg_data_mask_in_S2); 

    if (amo_alu_op_S2 != 4'd0)
    begin
        data_data_merge_S2 = amo_result_S2;
    end
end

l2_data_pgen data_pgen1( 
    .din            (data_data_merge_S2[64-1:0]),
    .parity         (data_data_parity1_S2)
);

l2_data_pgen data_pgen2( 
    .din            (data_data_merge_S2[128-1:64]),
    .parity         (data_data_parity2_S2)
);


always @ *
begin
    if (special_addr_type_S2)
    begin
        data_data_in_S2 = {msg_data_S2[8-1:0], msg_data_S2,
                           msg_data_S2[8-1:0], msg_data_S2};
    end
    else
    begin
        data_data_in_S2 = {data_data_parity2_S2, data_data_merge_S2[127:64], data_data_parity1_S2, data_data_merge_S2[63:0]}; 
    end
end

always @ *
begin
    if (special_addr_type_S2)
    begin
        if (addr_S2_f[31:30] == {2{1'b0}})
        begin
            data_data_mask_in_S2 = {{(144-72){1'b0}},
                                    {8{1'b0}}, {64{1'b1}}};
        end
        else
        begin
            data_data_mask_in_S2 = {{(144-72){1'b0}},
                                    {8{1'b1}}, {64{1'b0}}};
        end
        data_data_mask_in_S2 = data_data_mask_in_S2 << (72*addr_S2_f[3]);
    end
    else if (data_size_S2 == 3'b001 || data_size_S2 == 3'b010
    ||  data_size_S2 == 3'b011 || data_size_S2 == 3'b100
    )
    begin
        data_data_mask_in_S2 = {{(144-72){1'b0}},{72{1'b1}}};
        data_data_mask_in_S2 = data_data_mask_in_S2 << (72*addr_S2_f[3]);
    end
    else
    begin   
        data_data_mask_in_S2 = {144{1'b1}}; 
    end
end




reg [6-1:0] state_owner_S2;
reg [4-1:0] state_subline_S2;
reg [2-1:0] state_rb_S2;
reg [4-1:0] state_lru_S2;

always @ *
begin
    state_owner_S2 = l2_way_state_owner_S2; 
    if (state_owner_op_S2 == 2'd1)
    begin
        
        if (csm_en)
        begin
            if (state_load_sdid_S2)
            begin
                state_owner_S2 = sdid_S2_f[5:0];
            end
            else
            begin
                state_owner_S2 = lsid_S2_f; 
            end
        end
        else
        
        begin
            state_owner_S2 = {src_y_S2_f[2:0], src_x_S2_f[2:0]}; 
        end
    end
    else if (state_owner_op_S2 == 2'd2)
    begin
        state_owner_S2 = l2_way_state_owner_S2 + 1; 
    end
    else if (state_owner_op_S2 == 2'd3)
    begin
        state_owner_S2 = l2_way_state_owner_S2 - 1; 
    end
    else if (state_owner_op_S2 == 2'd0)
    begin
        state_owner_S2 = 0; 
    end
end

reg [4-1:0] addr_subline_S2;

always @ *
begin
    if (cache_type_S2 == 1'b0)
    begin
        //addr_subline_S2= {1'b1, {(`L2_SUBLINE_BITS-1){1'b0}}} >> addr_S2_f[`L2_DATA_SUBLINE];
        addr_subline_S2= {{(4-1){1'b0}},1'b1} << addr_S2_f[5:4];
    end
    else
    begin
        addr_subline_S2= {{(4-2){1'b0}},2'b11} << (2*addr_S2_f[5]);
        //addr_subline_S2= {2'b11, {(`L2_SUBLINE_BITS-2){1'b0}}} >> (2*addr_S2_f[`L2_INS_SUBLINE]);
    end
end


always @ *
begin
    if (state_load_sdid_S2)
    begin
        state_subline_S2 = sdid_S2_f[9:6];
    end
    else if (state_subline_op_S2 == 2'd2)
    begin
        state_subline_S2 = l2_way_state_subline_S2 | addr_subline_S2;
    end
    else if (state_subline_op_S2 == 2'd3)
    begin
        state_subline_S2 = l2_way_state_subline_S2 & (~addr_subline_S2);
    end
    else if (state_subline_op_S2 == 2'd0)
    begin
        state_subline_S2 = {4{1'b0}};
    end
    else
    begin
        state_subline_S2 = {4{1'bx}};
    end
end

always @ *
begin
    state_rb_S2 = l2_rb_bits_S2 + 1; 
end


always @ *
begin
    if (state_lru_en_S2)
    begin
        if (state_lru_op_S2 == 1'b0)
        begin
            state_lru_S2 = l2_lru_bits_S2 & (~({{(4-1){1'b0}},1'b1} << l2_way_sel_S2));
        end
        else
        begin
            state_lru_S2 = l2_lru_bits_S2 | ({{(4-1){1'b0}},1'b1} << l2_way_sel_S2);
            //clear all lru bits if they are all set
            if (state_lru_S2 == {4{1'b1}})
            begin
                state_lru_S2 = {4{1'b0}};
            end
        end
    end
    else
    begin
        state_lru_S2 = l2_lru_bits_S2; 
    end
end


always @ *
begin
    if (special_addr_type_S2)
    begin   
        state_data_in_S2 = {msg_data_S2[2+4-1:0], msg_data_S2[15*4-1:0]};
    end
    else
    begin
        state_data_in_S2 = {state_rb_S2, state_lru_S2, 
        {4{state_mesi_S2, state_vd_S2, cache_type_S2, state_subline_S2, state_owner_S2}}};
    end
end

reg [15*4-1:0] state_way_data_mask_in_S2;



always @ *
begin
    state_way_data_mask_in_S2 = {{(4-1)*15{1'b0}},
                                {{2{state_mesi_en_S2}}, 
                                 {2{state_vd_en_S2}}, 
                                 {1{state_di_en_S2}}, 
                                 {4{state_subline_en_S2}}, 
                                 {6{state_owner_en_S2}}}} 
    << (l2_way_sel_S2 * 15); 
end


always @ *
begin
    if (special_addr_type_S2)
    begin
        if (addr_S2_f[31:30] == {2{1'b0}})
        begin
            state_data_mask_in_S2 = {{(2+4){1'b0}}, {15*4{1'b1}}};
        end
        else
        begin
            state_data_mask_in_S2 = {{(2+4){1'b1}}, {15*4{1'b0}}};
        end
    end
    else
    begin
        state_data_mask_in_S2 = {{2{state_rb_en_S2}}, 
                                {4{state_lru_en_S2}},
                                state_way_data_mask_in_S2}; 
    end
end


reg [64-1:0] msg_data_S2_next;

always @ *
begin
    if (special_addr_type_S2)
    begin
        msg_data_S2_next = return_data_S2;
    end
    else
    begin
        msg_data_S2_next = msg_data_S2;
    end
end


always @ *
begin
    smc_wr_addr_in_S2 = addr_S2[16+3:4];
end

always @ *
begin
    smc_data_in_S2 = {msg_data_S2, msg_data_S2};
end


//============================
// Stage 2 > Stage 3
//============================

reg [40-1:0] addr_S3_f;
reg [8-1:0] mshrid_S3_f;
reg [14-1:0] src_chipid_S3_f;
reg [8-1:0] src_x_S3_f;
reg [8-1:0] src_y_S3_f;
reg [4-1:0] src_fbits_S3_f;
reg [10-1:0] sdid_S3_f;
reg [6-1:0] lsid_S3_f;
reg [6-1:0] mshr_miss_lsid_S3_f;
reg [40-1:0] evict_addr_S3_f;
reg l2_tag_hit_S3_f;
reg l2_evict_S3_f;
reg [2-1:0] l2_way_sel_S3_f;
reg [6-1:0] l2_way_state_owner_S3_f;
reg [2-1:0] l2_way_state_mesi_S3_f;
reg [2-1:0] l2_way_state_vd_S3_f;
reg [4-1:0] l2_way_state_subline_S3_f;
reg [1-1:0] l2_way_state_cache_type_S3_f;
reg [64-1:0] msg_data_S3_f;
reg req_from_owner_S3_f;
reg [15*4+2+4-1:0] state_data_in_S3_f;
reg [15*4+2+4-1:0] state_data_mask_in_S3_f;
reg [8+2+2-1:0] data_addr_S3_f;
reg recycled_S3_f;
reg data_clk_en_S3_f; // trin: added for stalled skid buffer

always @ (posedge clk)
begin
    if (!rst_n)
    begin
        addr_S3_f <= 0;
        mshrid_S3_f <= 0;
        src_chipid_S3_f <= 0;
        src_x_S3_f <= 0;
        src_y_S3_f <= 0;
        src_fbits_S3_f <= 0;
        sdid_S3_f <= 0;
        lsid_S3_f <= 0;
        mshr_miss_lsid_S3_f <= 0;
        evict_addr_S3_f <= 0;
        l2_tag_hit_S3_f <= 0;
        l2_evict_S3_f <= 0;
        l2_way_sel_S3_f <= 0;
        l2_way_state_owner_S3_f <= 0;
        l2_way_state_mesi_S3_f <= 0;
        l2_way_state_vd_S3_f <= 0;
        l2_way_state_subline_S3_f <= 0;
        l2_way_state_cache_type_S3_f <= 0;
        msg_data_S3_f <= 0;
        req_from_owner_S3_f <= 0; 
        state_data_in_S3_f <= 0;
        state_data_mask_in_S3_f <= 0;
        data_addr_S3_f <= 0;
        recycled_S3_f <= 0;
        data_clk_en_S3_f <= 0;
    end
    else if (!stall_S3)
    begin
        addr_S3_f <= addr_S2;
        mshrid_S3_f <= mshrid_S2_f;
        src_chipid_S3_f <= src_chipid_S2_f;
        src_x_S3_f <= src_x_S2_f;
        src_y_S3_f <= src_y_S2_f;
        src_fbits_S3_f <= src_fbits_S2_f;
        sdid_S3_f <= sdid_S2_f;
        lsid_S3_f <= lsid_S2_f;
        mshr_miss_lsid_S3_f <= mshr_miss_lsid_S2_f;
        evict_addr_S3_f <= evict_addr_S2;
        l2_tag_hit_S3_f <= l2_tag_hit_S2;
        l2_evict_S3_f <= l2_evict_S2;
        l2_way_sel_S3_f <= l2_way_sel_S2;
        l2_way_state_owner_S3_f <= l2_way_state_owner_S2;
        l2_way_state_mesi_S3_f <= l2_way_state_mesi_S2;
        l2_way_state_vd_S3_f <= l2_way_state_vd_S2;
        l2_way_state_subline_S3_f <= l2_way_state_subline_S2;
        l2_way_state_cache_type_S3_f <= l2_way_state_cache_type_S2;
        msg_data_S3_f <= msg_data_S2_next;
        req_from_owner_S3_f <= req_from_owner_S2; 
        state_data_in_S3_f <= state_data_in_S2;
        state_data_mask_in_S3_f <= state_data_mask_in_S2;
        data_addr_S3_f <= data_addr_S2;
        recycled_S3_f <= recycled_S2_f;
    end
    data_clk_en_S3_f <= (data_clk_en_S2 && valid_S2 && !stall_real_S2); // note: should not be qualified by stall_S3
end


//============================
// Stage 3
//============================

always @ *
begin
    addr_S3 = addr_S3_f;
end
//============================
// Stage 3 > Stage 4
//============================

reg [40-1:0] addr_S4_f;
reg [8-1:0] mshrid_S4_f;
reg [14-1:0] src_chipid_S4_f;
reg [8-1:0] src_x_S4_f;
reg [8-1:0] src_y_S4_f;
reg [4-1:0] src_fbits_S4_f;
reg [10-1:0] sdid_S4_f;
reg [6-1:0] lsid_S4_f;
reg [6-1:0] mshr_miss_lsid_S4_f;
reg [40-1:0] evict_addr_S4_f;
reg l2_tag_hit_S4_f;
reg l2_evict_S4_f;
reg [2-1:0] l2_way_sel_S4_f;
reg [6-1:0] l2_way_state_owner_S4_f;
reg [2-1:0] l2_way_state_mesi_S4_f;
reg [2-1:0] l2_way_state_vd_S4_f;
reg [4-1:0] l2_way_state_subline_S4_f;
reg [1-1:0] l2_way_state_cache_type_S4_f;
reg [64-1:0] msg_data_S4_f;
reg req_from_owner_S4_f;
reg [15*4+2+4-1:0] state_data_in_S4_f;
reg [15*4+2+4-1:0] state_data_mask_in_S4_f;
reg [144-1:0] data_data_S4_f;
reg [8+2+2-1:0] data_addr_S4_f;
reg recycled_S4_f;

reg data_stalled_skid_buffer_en_S3_f;
reg [144-1:0] data_stalled_skid_buffer_S3_f;

always @ (posedge clk)
begin
    if (!rst_n)
    begin
        addr_S4_f <= 0;
        mshrid_S4_f <= 0;
        src_chipid_S4_f <= 0;
        src_x_S4_f <= 0;
        src_y_S4_f <= 0;
        src_fbits_S4_f <= 0;
        sdid_S4_f <= 0;
        lsid_S4_f <= 0;
        mshr_miss_lsid_S4_f <= 0;
        evict_addr_S4_f <= 0;
        l2_tag_hit_S4_f <= 0;
        l2_evict_S4_f <= 0;
        l2_way_sel_S4_f <= 0;
        l2_way_state_owner_S4_f <= 0;
        l2_way_state_mesi_S4_f <= 0;
        l2_way_state_vd_S4_f <= 0;
        l2_way_state_subline_S4_f <= 0;
        l2_way_state_cache_type_S4_f <= 0;
        msg_data_S4_f <= 0;
        req_from_owner_S4_f <= 0; 
        state_data_in_S4_f <= 0;
        state_data_mask_in_S4_f <= 0;
        data_data_S4_f <= 0;
        data_addr_S4_f <= 0;
        recycled_S4_f <= 0;
    end
    else if (!stall_S4)
    begin
        addr_S4_f <= addr_S3_f;
        mshrid_S4_f <= mshrid_S3_f;
        src_chipid_S4_f <= src_chipid_S3_f;
        src_x_S4_f <= src_x_S3_f;
        src_y_S4_f <= src_y_S3_f;
        src_fbits_S4_f <= src_fbits_S3_f;
        sdid_S4_f <= sdid_S3_f;
        lsid_S4_f <= lsid_S3_f;
        mshr_miss_lsid_S4_f <= mshr_miss_lsid_S3_f;
        evict_addr_S4_f <= evict_addr_S3_f;
        l2_tag_hit_S4_f <= l2_tag_hit_S3_f;
        l2_evict_S4_f <= l2_evict_S3_f;
        l2_way_sel_S4_f <= l2_way_sel_S3_f;
        l2_way_state_owner_S4_f <= l2_way_state_owner_S3_f;
        l2_way_state_mesi_S4_f <= l2_way_state_mesi_S3_f;
        l2_way_state_vd_S4_f <= l2_way_state_vd_S3_f;
        l2_way_state_subline_S4_f <= l2_way_state_subline_S3_f;
        l2_way_state_cache_type_S4_f <= l2_way_state_cache_type_S3_f;
        msg_data_S4_f <= msg_data_S3_f;
        req_from_owner_S4_f <= req_from_owner_S3_f; 
        state_data_in_S4_f <= state_data_in_S3_f;
        state_data_mask_in_S4_f <= state_data_mask_in_S3_f;
        data_data_S4_f <= data_data_S3;
        data_addr_S4_f <= data_addr_S3_f;
        recycled_S4_f <= recycled_S3_f;
        if (data_stalled_skid_buffer_en_S3_f) begin
            data_data_S4_f <= data_stalled_skid_buffer_S3_f;
        end
    end
end

// trin: skid buffer for data read (bug 7/11/18)
wire data_stalled_skid_buffer_en_S3 = data_clk_en_S3_f && valid_S3 && stall_S3;
wire data_stalled_skid_buffer_consume_S3 = valid_S3 && !stall_S3;

always @ (posedge clk) begin
    if (data_stalled_skid_buffer_en_S3) begin
        data_stalled_skid_buffer_S3_f <= data_data_S3;
        data_stalled_skid_buffer_en_S3_f <= 1'b1;
    end
    if (data_stalled_skid_buffer_consume_S3) begin
        data_stalled_skid_buffer_en_S3_f <= 1'b0;
    end
end




// monitor for checking race condition
always @ (posedge clk) begin
    if (data_stalled_skid_buffer_en_S3_f) begin
        if (data_stalled_skid_buffer_S3_f != data_data_S3) begin
            // check whether the saved data is equaled to the current data
            $display("Error: L2 pipe1 data access race condition!");
            $finish();
        end
    end
end
 // L2_CAM_MSHR



//============================
// Stage 4
//============================

reg [15*4+2+4-1:0] state_data_in_real_S4;
reg [15*4+2+4-1:0] state_data_mask_in_real_S4;

reg [16-1:0] smc_rd_addr_in_S4;

reg [144-1:0] data_data_S4;
reg [10-1:0] sdid_S4;

always @ *
begin
    addr_S4 = addr_S4_f;
    mshrid_S4 = mshrid_S4_f;
    l2_evict_S4 = l2_evict_S4_f;
    l2_tag_hit_S4 = l2_tag_hit_S4_f;
    l2_way_state_mesi_S4 = l2_way_state_mesi_S4_f;
    l2_way_state_owner_S4 = l2_way_state_owner_S4_f;
    l2_way_state_vd_S4 = l2_way_state_vd_S4_f;
    l2_way_state_subline_S4 = l2_way_state_subline_S4_f; 
    l2_way_state_cache_type_S4 = l2_way_state_cache_type_S4_f;
    mshr_miss_lsid_S4 = mshr_miss_lsid_S4_f;
    lsid_S4 = lsid_S4_f;
    data_data_S4 = data_data_S4_f;
    data_addr_S4 = data_addr_S4_f;
end


always @ *
begin
    
    if (csm_en && (!msg_from_mshr_S4 || recycled_S4_f) && l2_evict_S4)
    begin
        if (l2_way_state_mesi_S4 == 2'b10)
        begin
            sdid_S4 = dir_data_S4[14+8+8+10-1:14+8+8]; 
        end
        else if (l2_way_state_mesi_S4 == 2'b01 || l2_way_state_mesi_S4 == 2'b11)
        begin
            sdid_S4 = {l2_way_state_subline_S4, l2_way_state_owner_S4};
        end
        else
        begin
            sdid_S4 = sdid_S4_f;
        end
    end
    else
    
    begin
        sdid_S4 = sdid_S4_f;
    end
end

always @ *
begin
    
    if (csm_en && (msg_type_S4 == 8'd13) && l2_tag_hit_S4
     && (l2_way_state_mesi_S4 == 2'b10) && l2_way_state_subline_S4[addr_S4[5:4]])
    begin
        req_from_owner_S4 = (src_chipid_S4_f == dir_data_S4[29:16])
                         && (src_x_S4_f == dir_data_S4[7:0])
                         && (src_y_S4_f == dir_data_S4[15:8]);
    end
    else
    
    begin
        req_from_owner_S4 = req_from_owner_S4_f;
    end
end



reg [16-1:0] smc_rd_addr_in_buf_S4_next;
reg [16-1:0] smc_rd_addr_in_buf_S4_f;

always @ *
begin
    if (!rst_n)
    begin
        smc_rd_addr_in_buf_S4_next = 0;
    end
    else if (!stall_smc_buf_S4)
    begin
        smc_rd_addr_in_buf_S4_next = smc_rd_addr_in_S4;
    end
    else 
    begin
        smc_rd_addr_in_buf_S4_next = smc_rd_addr_in_buf_S4_f;
    end
end

always @ (posedge clk)
begin
    smc_rd_addr_in_buf_S4_f <= smc_rd_addr_in_buf_S4_next;
end

always @ *
begin
    smc_rd_addr_in_buf_S4 = smc_rd_addr_in_buf_S4_f;
end
/*
reg [`L2_SMC_DATA_OUT_WIDTH-1:0] smc_data_out_buf_S4_next;
reg [`L2_SMC_VALID_WIDTH-1:0] smc_valid_out_buf_S4_next;
reg [`L2_SMC_TAG_WIDTH-1:0] smc_tag_out_buf_S4_next;
reg [`L2_SMC_DATA_OUT_WIDTH-1:0] smc_data_out_buf_S4_f;
reg [`L2_SMC_VALID_WIDTH-1:0] smc_valid_out_buf_S4_f;
reg [`L2_SMC_TAG_WIDTH-1:0] smc_tag_out_buf_S4_f;


always @ *
begin
    if (!rst_n)
    begin
        smc_data_out_buf_S4_next = 0;
        smc_tag_out_buf_S4_next = 0;
        smc_valid_out_buf_S4_next = 0;
    end
    else if (!stall_smc_buf_S4)
    begin
        smc_data_out_buf_S4_next = smc_data_out_S4;
        smc_tag_out_buf_S4_next = smc_tag_out_S4;
        smc_valid_out_buf_S4_next = smc_valid_out_S4;
    end
    else 
    begin
        smc_data_out_buf_S4_next = smc_data_out_buf_S4_f;
        smc_tag_out_buf_S4_next = smc_tag_out_buf_S4_f;
        smc_valid_out_buf_S4_next = smc_valid_out_buf_S4_f;
    end
end

always @ (posedge clk)
begin
    smc_data_out_buf_S4_f <= smc_data_out_buf_S4_next;
    smc_tag_out_buf_S4_f <= smc_tag_out_buf_S4_next;
    smc_valid_out_buf_S4_f <= smc_valid_out_buf_S4_next;
end
*/


always @ *
begin
    state_wr_addr_S4 = addr_S4_f[6+8-1:6]; 
end

always @ *
begin
    //invalidations may be interrupted by smc misses so the number of sharers may    
    //not be the same as the total number. In addition, the counter needs to 
    //consider the case that inv_acks for early invs come back before later invs     
    //are sent out
    if(!msg_from_mshr_S4 || recycled_S4_f)
    begin
        
        if (smc_miss_S4)
        begin
            state_data_in_real_S4 = {{2{1'b0}}, {4{1'b0}}, 
            {4{{2{1'b0}}, {2{1'b0}}, {1{1'b0}}, {4{1'b0}}, 
            (dir_sharer_counter_S4 - mshr_inv_counter_out_S4 - 6'b1)}}};
        end
        else
        
        begin
            state_data_in_real_S4 = {{2{1'b0}}, {4{1'b0}}, 
            {4{{2{1'b0}}, {2{1'b0}}, {1{1'b0}}, {4{1'b0}}, 
            (dir_sharer_counter_S4 - mshr_inv_counter_out_S4)}}};
        end
    end
    else
    begin
        
        if (smc_miss_S4)
        begin
            state_data_in_real_S4 = {{2{1'b0}}, {4{1'b0}}, 
            {4{{2{1'b0}}, {2{1'b0}}, {1{1'b0}}, {4{1'b0}}, 
            (dir_sharer_counter_S4 + l2_way_state_owner_S4_f - mshr_inv_counter_out_S4 - 6'b1)}}};
        end
        else
        
        begin
            state_data_in_real_S4 = {{2{1'b0}}, {4{1'b0}}, 
            {4{{2{1'b0}}, {2{1'b0}}, {1{1'b0}}, {4{1'b0}}, 
            (dir_sharer_counter_S4 + l2_way_state_owner_S4_f - mshr_inv_counter_out_S4)}}};
        end
    end
end

reg [4*15-1:0] state_way_data_mask_in_S4;



always @ *
begin
    state_way_data_mask_in_S4 = {{(4-1)*15{1'b0}},
                                {{2{1'b0}}, 
                                 {2{1'b0}}, 
                                 {1{1'b0}}, 
                                 {4{1'b0}}, 
                                 {6{state_wr_sel_S4}}}} 
    << (l2_way_sel_S4_f * 15); 
end


always @ *
begin
    state_data_mask_in_real_S4 = {{2{1'b0}}, 
                                 {4{1'b0}},
                                 state_way_data_mask_in_S4}; 
end


always @ *
begin
    state_data_mask_in_S4 = state_data_mask_in_S4_f | state_data_mask_in_real_S4;
end

always @ *
begin
    state_data_in_S4 = (state_data_in_S4_f & state_data_mask_in_S4_f) | 
                       (state_data_in_real_S4 & state_data_mask_in_real_S4);
end

reg [144-1:0] data_data_buf_S4_f;
reg [144-1:0] data_data_buf_S4_next;
reg [144-1:0] data_data_trans_S4;
reg [128-1:0] data_data_shift_S4;
wire corr_error1_S4, corr_error2_S4;
wire uncorr_error1_S4, uncorr_error2_S4;

always @ *
begin
    if (!rst_n)
    begin
        data_data_buf_S4_next = 0;
    end
    else if (stall_S4 && !stall_before_S4)
    begin
        data_data_buf_S4_next = data_data_S4;
    end
    else
    begin
        data_data_buf_S4_next = data_data_buf_S4_f;
    end
end



always @ (posedge clk)
begin
    data_data_buf_S4_f <= data_data_buf_S4_next;
end


always @ *
begin
    if (stall_before_S4)
    begin
        data_data_trans_S4 = data_data_buf_S4_f;
    end
    else
    begin
        data_data_trans_S4 = data_data_S4;
    end
end

l2_data_ecc data_ecc1 ( 
    .din                (data_data_trans_S4[63:0]),
    .parity             (data_data_trans_S4[71:64]),        
    .dout               (data_data_ecc_S4[63:0]),
    .corr_error         (corr_error1_S4),
    .uncorr_error       (uncorr_error1_S4)
);

l2_data_ecc data_ecc2 ( 
    .din                (data_data_trans_S4[135:72]),
    .parity             (data_data_trans_S4[143:136]),        
    .dout               (data_data_ecc_S4[127:64]),
    .corr_error         (corr_error2_S4),
    .uncorr_error       (uncorr_error2_S4)
);


always @ *
begin
    corr_error_S4 = corr_error1_S4 | corr_error2_S4; 
    uncorr_error_S4 = uncorr_error1_S4 | uncorr_error2_S4; 
end


always @ *
begin
    case (cas_cmp_data_size_S4)
    3'b011:  
    begin
        data_data_shift_S4 = {data_data_ecc_S4[63:0],data_data_ecc_S4[127:64]} << 32*addr_S4_f[3:2];
    end
    3'b100:
    begin
        data_data_shift_S4 = {data_data_ecc_S4[63:0], data_data_ecc_S4[127:64]} << 64*addr_S4_f[3];
    end
    default:
    begin
        data_data_shift_S4 = {data_data_ecc_S4[63:0], data_data_ecc_S4[127:64]};
    end
    endcase
end



always @ *
begin
    if (cas_cmp_en_S4)
    begin
    case (cas_cmp_data_size_S4)
        3'b011:  
        begin
            if (data_data_shift_S4[127:96] == msg_data_S4_f[31:0])
            begin
                cas_cmp_S4 = y;
            end
            else
            begin
                cas_cmp_S4 = n;
            end
        end
        3'b100:
        begin
            if (data_data_shift_S4[127:64] == msg_data_S4_f[63:0])
            begin
                cas_cmp_S4 = y;
            end
            else
            begin
                cas_cmp_S4 = n;
            end
        end
        default:
        begin
            cas_cmp_S4 = n;
        end
    endcase
    end
    else
    begin
        cas_cmp_S4 = n;
    end
end

always @ *
begin
    if (l2_evict_S4 
    && (msg_send_type_S4 == 8'd20
    ||  msg_send_type_S4 == 8'd18
    ||  msg_send_type_S4 == 8'd17))
    begin
        msg_send_addr_S4 = evict_addr_S4_f;
    end
    else if (msg_send_type_S4 == 8'd17
         ||  msg_send_type_S4 == 8'd18
         ||  msg_send_type_S4 == 8'd16)
    begin
        msg_send_addr_S4 = {addr_S4_f[39:6+8], addr_S4_f[6+8-1:6], {6{1'b0}}}; 
    end
    else if (msg_send_type_S4 == 8'd14)
    begin
        
        if (csm_en)
        begin 
            if (smc_miss_S4)
            begin 
                msg_send_addr_S4 = {smt_base_addr, smc_rd_addr_in_S4[15:2], 4'd0};
            end
            else
            begin
                msg_send_addr_S4 = addr_S4_f;
            end
        end
        else
        
        begin
            msg_send_addr_S4 = addr_S4_f;
        end
    end
    else    
    begin
        msg_send_addr_S4 = addr_S4_f;
    end
end


always @ *
begin
    case (msg_send_type_S4)
    8'd16, 8'd17: 
    begin
        
        if (csm_en)
        begin
            msg_send_dst_chipid_S4 = dir_data_S4[29:16];
            msg_send_dst_x_S4 = dir_data_S4[7:0];
            msg_send_dst_y_S4 = dir_data_S4[15:8];
        end
        else    
        
        begin


      
            msg_send_dst_chipid_S4 = my_nodeid_chipid_S4;
      
            msg_send_dst_x_S4 = {{(8 - 3){1'b0}}, 
                                 l2_way_state_owner_S4_f[2:0]}; 
            msg_send_dst_y_S4 = {{(8 - 3){1'b0}},
                                 l2_way_state_owner_S4_f[5:3]};
        end
        msg_send_dst_fbits_S4 = 4'd0;
    end
    8'd18:
    begin
        
        if (csm_en)
        begin
            if (l2_way_state_mesi_S4_f == 2'b11)
            begin
                msg_send_dst_chipid_S4 = broadcast_chipid_out_S4;
                msg_send_dst_x_S4 = broadcast_x_out_S4;
                msg_send_dst_y_S4 = broadcast_y_out_S4;
            end
            else
            begin
                msg_send_dst_chipid_S4 = smc_data_out_S4[29:16];
                msg_send_dst_x_S4 = smc_data_out_S4[7:0];
                msg_send_dst_y_S4 = smc_data_out_S4[15:8];
            end
        end
        else
        
        begin
            msg_send_dst_chipid_S4 = my_nodeid_chipid_S4;
            msg_send_dst_x_S4 = {{(8 - 3){1'b0}}, dir_sharer_S4[2:0]}; 
            msg_send_dst_y_S4 = {{(8 - 3){1'b0}}, dir_sharer_S4[5:3]};
        end
        msg_send_dst_fbits_S4 = 4'd0;
    end
    8'd28, 8'd29:
    begin
        msg_send_dst_chipid_S4 = src_chipid_S4_f;
        msg_send_dst_x_S4 = src_x_S4_f; 
        msg_send_dst_y_S4 = src_y_S4_f;
        msg_send_dst_fbits_S4 = 4'd0;

    end
    8'd19, 8'd14, 8'd15, 8'd20:
    begin
        msg_send_dst_chipid_S4 = {1'b1, my_nodeid_chipid_S4[12:0]};
        msg_send_dst_x_S4 = 0; 
        msg_send_dst_y_S4 = 0;
        msg_send_dst_fbits_S4 = 4'd2;
    end
    8'd33:
    begin
        msg_send_dst_chipid_S4 = my_nodeid_chipid_S4;
        msg_send_dst_x_S4 = my_nodeid_x_S4; 
        msg_send_dst_y_S4 = my_nodeid_y_S4;
        msg_send_dst_fbits_S4 = 4'd0;
    end
    default:
    begin
        msg_send_dst_chipid_S4 = my_nodeid_chipid_S4;
        msg_send_dst_x_S4 = my_nodeid_x_S4; 
        msg_send_dst_y_S4 = my_nodeid_y_S4;
        msg_send_dst_fbits_S4 = 4'd0;
    end
    endcase
end

always @ *
begin
    if (special_addr_type_S4)
    begin  
        if (addr_S4[39:32] == 8'ha0)
        begin
            if (addr_S4[31:30] == {2{1'b0}})
            begin
                if (addr_S4[3] == 0)
                begin
                    msg_send_data_S4 = {2{data_data_trans_S4[63:0]}}; 
                end
                else
                begin
                    msg_send_data_S4 = {2{data_data_trans_S4[135:72]}}; 
                end
            end
            else
            begin
                if (addr_S4[3] == 0)
                begin
                    msg_send_data_S4 = {2{56'b0, data_data_trans_S4[71:64]}}; 
                end
                else
                begin
                    msg_send_data_S4 = {2{56'b0, data_data_trans_S4[143:136]}}; 
                end
            end
        end
        else if (addr_S4[39:32] == 8'ha1)
        begin
            msg_send_data_S4 = {2{dir_data_sel_S4}}; 
        end
        
        else if (addr_S4[39:32] == 8'ha2)
        begin
            case (addr_S4[31:30])
            2'd0:
            begin
                msg_send_data_S4 = {2{{(64-30){1'b0}}, smc_data_out_S4}}; 
            end
            2'd1:
            begin
                msg_send_data_S4 = {2{{(64-4){1'b0}}, smc_valid_out_S4}}; 
            end
            2'd2: 
            begin
                msg_send_data_S4 = {2{{(64-14){1'b0}}, smc_tag_out_S4}}; 
            end
            default:
            begin
                msg_send_data_S4 = {128{1'b0}}; 
            end
            endcase
        end
        
        else if (addr_S4[39:32] == 8'ha9 
              || addr_S4[39:32] == 8'ha7
              || addr_S4[39:32] == 8'ha8
              || addr_S4[39:32] == 8'haa
              || addr_S4[39:32] == 8'hab)
        begin
            msg_send_data_S4 = {2{reg_data_out_S4}}; 
        end
        else
        begin
            msg_send_data_S4 = {2{msg_data_S4_f}}; 
        end 
    end
    else if ((msg_type_S4 == 8'd34 || msg_type_S4 == 8'd35) 
         &&  (msg_send_type_S4 == 8'd29))
    begin
        msg_send_data_S4 = {128{1'b0}}; 
    end  
    else if (msg_send_type_S4 == 8'd15 || (msg_send_type_S4 == 8'd33))
    begin
        msg_send_data_S4 = {2{msg_data_S4_f}}; 
    end
    else    
    begin
        msg_send_data_S4 = data_data_ecc_S4;
    end
end



always @ *
begin
    if (special_addr_type_S4 && (addr_S4[39:32] == 8'ha2))
    begin
        smc_rd_addr_in_S4 = addr_S4[16+3:4];
    end
    else if (msg_send_type_pre_S4 == 8'd18)
    begin
        smc_rd_addr_in_S4 = {sdid_S4, dir_sharer_S4};
    end
    else
    begin
        smc_rd_addr_in_S4 = {sdid_S4, l2_way_state_owner_S4_f};
    end
end




always @ *
begin
    mshr_data_in_S4 = {inv_fwd_pending_S4,
                       (req_recycle_S4 && (!msg_from_mshr_S4 || recycled_S4_f)),
                       smc_miss_S4,
                       smc_rd_addr_in_S4[5:0],
                       lsid_S4_f, 
                       sdid_S4,
                       src_fbits_S4_f,
                       src_y_S4_f,
                       src_x_S4_f,
                       src_chipid_S4_f,
                       l2_miss_S4,
                       msg_type_S4,
                       data_size_S4,
                       cache_type_S4,
                       mshrid_S4_f,
                       l2_way_sel_S4_f,
                       addr_S4_f};
end























/*
always @ *
begin
    mshr_data_mask_in_S2 = {`L2_MSHR_ARRAY_WIDTH{1'b1}}; 
end
*/
assign mshr_data_mask_in_S4 = {120+2{1'b1}}; 



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
//  Filename      : l2_pipe2_buf_in.v
//  Created On    : 2014-04-06
//  Revision      :
//  Author        : Yaosheng Fu
//  Company       : Princeton University
//  Email         : yfu@princeton.edu
//
//  Description   : Input buffer for pipeline2
//
//
//==================================================================================================

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
//  Filename      : l2.h.pyv
//  Created On    : 2014-02-20
//  Revision      :
//  Author        : Yaosheng Fu
//  Company       : Princeton University
//  Email         : yfu@princeton.edu
//
//  Description   : header file for the L2 cache
//
//
//==================================================================================================

// devices.xml



// Input buffer for pipeline1













// Input buffer for pipeline2














// Output buffer















// L2 cache configuration







//`define L2_SIZE                  65536
//`define L2_SIZE_WIDTH            16






//`define L2_LINE_SIZE             64
//`define L2_LINE_SIZE_WIDTH       6













//`define L2_WAYS                  4
//`define L2_WAYS_WIDTH            2
//`define L2_WAY_0                 2'b00
//`define L2_WAY_1                 2'b01
//`define L2_WAY_2                 2'b10
//`define L2_WAY_3                 2'b11





// Tag array







//`define L2_TAG_INDEX_WIDTH      8
//`define L2_TAG_WIDTH            26
//`define L2_TAG_WAY_WIDTH        26
//`define L2_TAG_ARRAY_WIDTH      104


// Tag Address decomposition 




//`define L2_TAG_INDEX            13:6
//`define L2_TAG                  39:14


//Data array












//`define L2_DATA_INDEX_WIDTH         12 








// Data Address decomposition 







//Dir array



//`define L2_DIR_INDEX_WIDTH      10 





//State array

















//Whether the cache line is in Icaches or Dcaches
















//Round Robin selection














// State decomposition 












//MSHR array



































//`define L2_MSHR_CMP_ADDR        13:6
//`define L2_MSHR_ADDR            39:0
//`define L2_MSHR_WAY             41:40
//`define L2_MSHR_MSHRID          49:42
//`define L2_MSHR_CACHE_TYPE      50
//`define L2_MSHR_DATA_SIZE       53:51
//`define L2_MSHR_MSG_TYPE        61:54
//`define L2_MSHR_L2_MISS         62
//`define L2_MSHR_SRC_CHIPID      76:63
//`define L2_MSHR_SRC_X           84:77
//`define L2_MSHR_SRC_Y           92:85
//`define L2_MSHR_SRC_FBITS       96:93
//`define L2_MSHR_SDID            106:97
//`define L2_MSHR_LSID            112:107      
//`define L2_MSHR_MISS_LSID       118:113
//`define L2_MSHR_SMC_MISS        119
//`define L2_MSHR_RECYCLED        120
//`define L2_MSHR_INV_FWD_PENDING 121


//SMC array


































//Message destination_type







//L2 public sharer beyond the maximum clump size



//L2 registers



//Special addresses





















//L2 core ID





//L2 dir array reuse







//Control Signal 








//AMO ALU OP macros












//Control Signal in Stage 1























































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





































































































































































































































































































































































































































































































































































































module l2_pipe2_buf_in(

    input wire clk,
    input wire rst_n,

    input wire valid_in,
    input wire [64-1:0] data_in,
    output reg ready_in,

    output reg msg_header_valid_out,
    output reg [192-1:0] msg_header_out,
    input wire msg_header_ready_out,

    output reg msg_data_valid_out,
    output reg [128-1:0] msg_data_out,
    input wire msg_data_ready_out
);




localparam msg_data_state_0F = 2'd0;
localparam msg_data_state_2F = 2'd1;
localparam msg_data_state_8F = 2'd2;

localparam msg_state_header0 = 4'd0;
localparam msg_state_header1 = 4'd1;
localparam msg_state_header2 = 4'd2;
localparam msg_state_data0 = 4'd3;
localparam msg_state_data1 = 4'd4;
localparam msg_state_data2 = 4'd5;
localparam msg_state_data3 = 4'd6;
localparam msg_state_data4 = 4'd7;
localparam msg_state_data5 = 4'd8;
localparam msg_state_data6 = 4'd9;
localparam msg_state_data7 = 4'd10;

reg [3:0] msg_state_f;
reg [3:0] msg_state_next;

reg [1:0] msg_data_state_f;
reg [1:0] msg_data_state_next;

always @ *
begin
    if (!rst_n)
    begin
        msg_data_state_next = msg_data_state_0F;
    end
    else if((msg_state_f == msg_state_header0) && valid_in)
    begin
        if (data_in[29:22] == 8)
        begin
            msg_data_state_next = msg_data_state_8F;
        end
        else if (data_in[29:22] == 0)
        begin
            msg_data_state_next = msg_data_state_0F;
        end
        else    
        begin
            msg_data_state_next = msg_data_state_2F;
        end
    end
    else
    begin
        msg_data_state_next = msg_data_state_f;
    end
end

always @ (posedge clk)
begin
    msg_data_state_f <= msg_data_state_next;
end

always @ *
begin
    if (!rst_n)
    begin
        msg_state_next = msg_state_header0;
    end
    else if (valid_in && ready_in)
    begin
        if ((msg_state_f == msg_state_header0) && (data_in[21:14] != 8'd12))
        begin
            if (data_in[29:22] == 0)
            begin
                msg_state_next = msg_state_header0;
            end
            else
            begin
                msg_state_next = msg_state_data0;
            end
        end
        else if ((msg_state_f == msg_state_data1) && (msg_data_state_f == msg_data_state_2F))
        begin
            msg_state_next = msg_state_header0;
        end
        else
        begin
            if (msg_state_f == msg_state_data7)
            begin
                msg_state_next = msg_state_header0;
            end
            else
            begin
                msg_state_next = msg_state_f + 3'd1;
            end
        end
    end
    else
    begin
        msg_state_next = msg_state_f;
    end 
end


always @ (posedge clk)
begin
    msg_state_f <= msg_state_next;
end

localparam msg_mux_header = 1'b0;
localparam msg_mux_data = 1'b1;

reg msg_mux_sel;

always @ *
begin
    if ((msg_state_f == msg_state_header0)
    ||  (msg_state_f == msg_state_header1)
    ||  (msg_state_f == msg_state_header2))
    begin
        msg_mux_sel = msg_mux_header;
    end
    else
        msg_mux_sel = msg_mux_data;
end


reg msg_header_valid_in;
reg [64-1:0] msg_header_in;
reg msg_header_ready_in;


reg msg_data_valid_in;
reg [64-1:0] msg_data_in;
reg msg_data_ready_in;


always @ *
begin
    if (msg_mux_sel == msg_mux_header)
    begin
        msg_header_valid_in = valid_in;
        msg_header_in = data_in;
    end
    else
    begin
        msg_header_valid_in = 0;
        msg_header_in = 0;
    end
end

always @ *
begin
    if (msg_mux_sel == msg_mux_data)
    begin
        msg_data_valid_in = valid_in;
        msg_data_in = data_in;
    end
    else
    begin
        msg_data_valid_in = 0;
        msg_data_in = 0;
    end
end

always @ *
begin
    if (msg_mux_sel == msg_mux_data)
    begin
        ready_in = msg_data_ready_in; 
    end
    else
    begin
        ready_in = msg_header_ready_in;
    end
end



reg [64-1:0] header_buf_mem_f [4-1:0];
reg header_buf_empty;
reg header_buf_full;
reg [2:0] header_buf_counter_f;
reg [2:0] header_buf_counter_next;
reg [2-1:0] header_rd_ptr_f;
reg [2-1:0] header_rd_ptr_next;
reg [2-1:0] header_rd_ptr_plus1;
reg [2-1:0] header_rd_ptr_plus2;
reg [2-1:0] header_wr_ptr_f;
reg [2-1:0] header_wr_ptr_next;


always @ *
begin
    header_buf_empty = (header_buf_counter_f == 0);
    header_buf_full = (header_buf_counter_f ==  4);
end

reg [1:0] msg_header_flits;

always @ *
begin
    msg_header_flits = 1;
    if (header_buf_mem_f[header_rd_ptr_f][21:14] == 8'd12)
    begin
        msg_header_flits = 3;
    end
    else
    begin
        msg_header_flits = 1;
    end
end

always @ *
begin
   if (!rst_n)
    begin
        header_buf_counter_next = 0;
    end
    else if ((msg_header_valid_in && msg_header_ready_in) && (msg_header_valid_out && msg_header_ready_out))
    begin
        header_buf_counter_next = header_buf_counter_f + 1 - msg_header_flits;
    end
    else if (msg_header_valid_in && msg_header_ready_in)
    begin 
        header_buf_counter_next = header_buf_counter_f + 1;
    end
    else if (msg_header_valid_out && msg_header_ready_out)
    begin 
        header_buf_counter_next = header_buf_counter_f - msg_header_flits;
    end
    else
    begin
        header_buf_counter_next = header_buf_counter_f;
    end
end



always @ (posedge clk)
begin
    header_buf_counter_f <= header_buf_counter_next;
end

always @ *
begin
    if (!rst_n)
    begin   
        header_rd_ptr_next = 0;
    end
    else if (msg_header_valid_out && msg_header_ready_out)
    begin
        header_rd_ptr_next = header_rd_ptr_f + msg_header_flits;
    end
    else
    begin
        header_rd_ptr_next = header_rd_ptr_f;
    end
end


always @ (posedge clk)
begin
    header_rd_ptr_f <= header_rd_ptr_next;
end

always @ *
begin
    if (!rst_n)
    begin   
        header_wr_ptr_next = 0;
    end
    else if (msg_header_valid_in && msg_header_ready_in)
    begin
        header_wr_ptr_next = header_wr_ptr_f + 1;
    end
    else
    begin
        header_wr_ptr_next = header_wr_ptr_f;
    end
end

always @ (posedge clk)
begin
    header_wr_ptr_f <= header_wr_ptr_next;
end


always @ *
begin
    header_rd_ptr_plus1 = header_rd_ptr_f + 1;
    header_rd_ptr_plus2 = header_rd_ptr_f + 2;
end


always @ *
begin
   msg_header_ready_in = !header_buf_full;
end


always @ *
begin
    msg_header_valid_out = (!header_buf_empty) && (header_buf_counter_f >= msg_header_flits);
end

always @ *
begin
    if (msg_header_flits == 3)
    begin
        msg_header_out = {header_buf_mem_f[header_rd_ptr_plus2], 
                          header_buf_mem_f[header_rd_ptr_plus1], 
                          header_buf_mem_f[header_rd_ptr_f]};
    end
    else
    begin
        msg_header_out = {{(2*64){1'b0}},header_buf_mem_f[header_rd_ptr_f]}; 
    end
end


always @ (posedge clk)
begin
    if (!rst_n)
    begin   
        header_buf_mem_f[0] <= 0;
        header_buf_mem_f[1] <= 0;
        header_buf_mem_f[2] <= 0;
        header_buf_mem_f[3] <= 0;

    end
    else if (msg_header_valid_in && msg_header_ready_in)
    begin
        header_buf_mem_f[header_wr_ptr_f] <= msg_header_in;
    end
    else
    begin 
        header_buf_mem_f[header_wr_ptr_f] <= header_buf_mem_f[header_wr_ptr_f];
    end
end



reg [64-1:0] data_buf_mem_f [16-1:0];
reg data_buf_empty;
reg data_buf_full;
reg [4:0] data_buf_counter_f;
reg [4:0] data_buf_counter_next;
reg [4-1:0] data_rd_ptr_f;
reg [4-1:0] data_rd_ptr_next;
reg [4-1:0] data_rd_ptr_plus1;
reg [4-1:0] data_wr_ptr_f;
reg [4-1:0] data_wr_ptr_next;

always @ *
begin
    data_buf_empty = (data_buf_counter_f == 0);
    data_buf_full = (data_buf_counter_f ==  16);
end

always @ *
begin
    if ((msg_data_valid_in && msg_data_ready_in) && (msg_data_valid_out && msg_data_ready_out))
    begin
        data_buf_counter_next = data_buf_counter_f + 1 - 2;
    end
    else if (msg_data_valid_in && msg_data_ready_in)
    begin 
        data_buf_counter_next = data_buf_counter_f + 1;
    end
    else if (msg_data_valid_out && msg_data_ready_out)
    begin 
        data_buf_counter_next = data_buf_counter_f - 2;
    end
    else
    begin
        data_buf_counter_next = data_buf_counter_f;
    end
end


always @ (posedge clk)
begin
    if (!rst_n)
        data_buf_counter_f <= 0;
    else
        data_buf_counter_f <= data_buf_counter_next;
end

always @ *
begin
    if (!rst_n)
    begin   
        data_rd_ptr_next = 0;
    end
    else if (msg_data_valid_out && msg_data_ready_out)
    begin
        data_rd_ptr_next = data_rd_ptr_f + 2;
    end
    else
    begin
        data_rd_ptr_next = data_rd_ptr_f;
    end
end


always @ (posedge clk)
begin
    data_rd_ptr_f <= data_rd_ptr_next;
end

always @ *
begin
    if (!rst_n)
    begin   
        data_wr_ptr_next = 0;
    end
    else if (msg_data_valid_in && msg_data_ready_in)
    begin
        data_wr_ptr_next = data_wr_ptr_f + 1;
    end
    else
    begin
        data_wr_ptr_next = data_wr_ptr_f;
    end
end

always @ (posedge clk)
begin
    data_wr_ptr_f <= data_wr_ptr_next;
end


always @ *
begin
    data_rd_ptr_plus1 = data_rd_ptr_f + 1;
end

always @ *
begin
   msg_data_ready_in = !data_buf_full;
end


always @ *
begin
    msg_data_valid_out = (data_buf_counter_f >= 2);
    msg_data_out = {data_buf_mem_f[data_rd_ptr_plus1], data_buf_mem_f[data_rd_ptr_f]}; 
end


always @ (posedge clk)
begin
    if (!rst_n)
    begin   
        data_buf_mem_f[0] <= 0;
        data_buf_mem_f[1] <= 0;
        data_buf_mem_f[2] <= 0;
        data_buf_mem_f[3] <= 0;
        data_buf_mem_f[4] <= 0;
        data_buf_mem_f[5] <= 0;
        data_buf_mem_f[6] <= 0;
        data_buf_mem_f[7] <= 0;
        data_buf_mem_f[8] <= 0;
        data_buf_mem_f[9] <= 0;
        data_buf_mem_f[10] <= 0;
        data_buf_mem_f[11] <= 0;
        data_buf_mem_f[12] <= 0;
        data_buf_mem_f[13] <= 0;
        data_buf_mem_f[14] <= 0;
        data_buf_mem_f[15] <= 0;

    end
    else if (msg_data_valid_in && msg_data_ready_in)
    begin
        data_buf_mem_f[data_wr_ptr_f] <= msg_data_in;
    end
    else
    begin 
        data_buf_mem_f[data_wr_ptr_f] <=  data_buf_mem_f[data_wr_ptr_f];
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

//==================================================================================================
//  Filename      : l2_pipe2_ctrl.v
//  Created On    : 2014-04-03
//  Revision      :
//  Author        : Yaosheng Fu
//  Company       : Princeton University
//  Email         : yfu@princeton.edu
//
//  Description   : The control unit for pipeline2 in the L2 cache
//
//
//====================================================================================================


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
//  Filename      : l2.h.pyv
//  Created On    : 2014-02-20
//  Revision      :
//  Author        : Yaosheng Fu
//  Company       : Princeton University
//  Email         : yfu@princeton.edu
//
//  Description   : header file for the L2 cache
//
//
//==================================================================================================

// devices.xml



// Input buffer for pipeline1













// Input buffer for pipeline2














// Output buffer















// L2 cache configuration







//`define L2_SIZE                  65536
//`define L2_SIZE_WIDTH            16






//`define L2_LINE_SIZE             64
//`define L2_LINE_SIZE_WIDTH       6













//`define L2_WAYS                  4
//`define L2_WAYS_WIDTH            2
//`define L2_WAY_0                 2'b00
//`define L2_WAY_1                 2'b01
//`define L2_WAY_2                 2'b10
//`define L2_WAY_3                 2'b11





// Tag array







//`define L2_TAG_INDEX_WIDTH      8
//`define L2_TAG_WIDTH            26
//`define L2_TAG_WAY_WIDTH        26
//`define L2_TAG_ARRAY_WIDTH      104


// Tag Address decomposition 




//`define L2_TAG_INDEX            13:6
//`define L2_TAG                  39:14


//Data array












//`define L2_DATA_INDEX_WIDTH         12 








// Data Address decomposition 







//Dir array



//`define L2_DIR_INDEX_WIDTH      10 





//State array

















//Whether the cache line is in Icaches or Dcaches
















//Round Robin selection














// State decomposition 












//MSHR array



































//`define L2_MSHR_CMP_ADDR        13:6
//`define L2_MSHR_ADDR            39:0
//`define L2_MSHR_WAY             41:40
//`define L2_MSHR_MSHRID          49:42
//`define L2_MSHR_CACHE_TYPE      50
//`define L2_MSHR_DATA_SIZE       53:51
//`define L2_MSHR_MSG_TYPE        61:54
//`define L2_MSHR_L2_MISS         62
//`define L2_MSHR_SRC_CHIPID      76:63
//`define L2_MSHR_SRC_X           84:77
//`define L2_MSHR_SRC_Y           92:85
//`define L2_MSHR_SRC_FBITS       96:93
//`define L2_MSHR_SDID            106:97
//`define L2_MSHR_LSID            112:107      
//`define L2_MSHR_MISS_LSID       118:113
//`define L2_MSHR_SMC_MISS        119
//`define L2_MSHR_RECYCLED        120
//`define L2_MSHR_INV_FWD_PENDING 121


//SMC array


































//Message destination_type







//L2 public sharer beyond the maximum clump size



//L2 registers



//Special addresses





















//L2 core ID





//L2 dir array reuse







//Control Signal 








//AMO ALU OP macros












//Control Signal in Stage 1























































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





































































































































































































































































































































































































































































































































































































module l2_pipe2_ctrl(

    input wire clk,
    input wire rst_n,
    
    input wire csm_en,
    
    //Inputs to Stage 1   

 
    input wire msg_header_valid_S1,
    input wire [8-1:0] msg_type_S1,
    input wire [8-1:0] msg_length_S1,
    input wire [3-1:0] msg_data_size_S1,
    input wire [1-1:0] msg_cache_type_S1,
    input wire [1-1:0] msg_last_subline_S1,
    input wire [2-1:0] msg_mesi_S1,

    //input wire mshr_hit_S1,
    input wire [8-1:0] mshr_msg_type_S1,
    input wire [1-1:0] mshr_l2_miss_S1,
    input wire [3-1:0] mshr_data_size_S1,
    input wire [1-1:0] mshr_cache_type_S1, 
    
    input wire mshr_smc_miss_S1,
    
    input wire [2-1:0] mshr_state_out_S1,
    input wire mshr_inv_fwd_pending_S1,

    input wire [40-1:0] addr_S1,
    input wire is_same_address_S1,

    //Inputs to Stage 2
   
 
    input wire l2_tag_hit_S2,
    input wire [2-1:0] l2_way_sel_S2,
    input wire l2_wb_S2,
    input wire [6-1:0] l2_way_state_owner_S2,
    input wire [2-1:0] l2_way_state_mesi_S2,
    input wire [2-1:0] l2_way_state_vd_S2,
    input wire [4-1:0] l2_way_state_subline_S2,
    input wire [1-1:0] l2_way_state_cache_type_S2,
    input wire addr_l2_aligned_S2,
    input wire subline_valid_S2,
    input wire [6-1:0] lsid_S2,

    
    input wire broadcast_counter_zero_S2,
    input wire broadcast_counter_max_S2,
    input wire [14-1:0] broadcast_chipid_out_S2,
    input wire [8-1:0] broadcast_x_out_S2,
    input wire [8-1:0] broadcast_y_out_S2,
    

    input wire msg_data_valid_S2,
    
    input wire [40-1:0] addr_S2,


    //Inputs to Stage 3
    input wire [40-1:0] addr_S3,

    //Outputs from Stage 1

    output reg valid_S1,  
    output reg stall_S1,
    output reg active_S1, 
    output reg msg_from_mshr_S1, 
 
    output reg mshr_rd_en_S1,
    //output reg mshr_cam_en_S1,

    output reg msg_header_ready_S1,

    output reg tag_clk_en_S1,
    output reg tag_rdw_en_S1,

    output reg state_rd_en_S1,

    //Outputs from Stage 2

    output reg valid_S2,    
    output reg stall_S2,  
    output reg stall_before_S2,
    output reg active_S2, 

    output reg msg_from_mshr_S2,
    output reg [8-1:0] msg_type_S2,
    output reg [3-1:0] data_size_S2,
    output reg [1-1:0] cache_type_S2,

    output reg dir_clk_en_S2,
    output reg dir_rdw_en_S2,
    output reg dir_clr_en_S2,


    output reg data_clk_en_S2,
    output wire data_rdw_en_S2,

    
    output reg [2-1:0] broadcast_counter_op_S2,
    output reg broadcast_counter_op_val_S2,
    

    output reg state_owner_en_S2,
    output reg [2-1:0] state_owner_op_S2,
    output reg state_subline_en_S2,
    output reg [2-1:0] state_subline_op_S2,
    output reg state_di_en_S2,
    output reg state_vd_en_S2,
    output reg [2-1:0] state_vd_S2,
    output reg state_mesi_en_S2,
    output reg [2-1:0] state_mesi_S2,
    output reg state_lru_en_S2,
    output reg [1-1:0] state_lru_op_S2,
    output wire state_rb_en_S2,

    output reg l2_load_64B_S2, 
    output reg [2-1:0] l2_load_data_subline_S2,

    output reg msg_data_ready_S2,

    
    output reg smc_wr_en_S2,
    
    //Outputs from Stage 3
    output reg valid_S3,    
    output wire stall_S3,  
    output reg active_S3, 

    output reg [8-1:0] msg_type_S3,
    output reg mshr_wr_state_en_S3,
    output wire mshr_wr_data_en_S3,
    output reg [2-1:0] mshr_state_in_S3,
    output reg mshr_inc_counter_en_S3,
    output reg state_wr_en_S3

);




localparam y = 1'b1;
localparam n = 1'b0;


localparam rd = 1'b1;
localparam wr = 1'b0;


//============================
// Stage 1
//============================

reg stall_pre_S1;
reg [3-1:0] data_size_S1;
reg [1-1:0] cache_type_S1;

reg smc_miss_S1;

reg inv_fwd_pending_S1;

reg stall_hazard_S1;

always @ *
begin
    stall_hazard_S1 = (valid_S2 && (addr_S1[6+8-1:6] == addr_S2[6+8-1:6])) ||
                      (valid_S3 && (addr_S1[6+8-1:6] == addr_S3[6+8-1:6]));
end


always @ *
begin
    valid_S1 = msg_header_valid_S1;
end


always @ *
begin
    stall_pre_S1 = stall_S2; 
end


always @ *
begin
    mshr_rd_en_S1 = valid_S1 && (msg_type_S1 != 8'd12) && (msg_type_S1 != 8'd25);
end

/*
always @ *
begin
    mshr_cam_en_S1 = valid_S1 && (msg_type_S1 == `MSG_TYPE_WB_REQ);
end
*/
always @ *
begin
    msg_from_mshr_S1 = mshr_rd_en_S1
                    && (mshr_state_out_S1 != 2'd0); 
end


always @ *
begin
    if (msg_from_mshr_S1)
    begin
        data_size_S1 = mshr_data_size_S1;
    end
    else
    begin
        data_size_S1 = msg_data_size_S1;
    end
end

always @ *
begin
    if (msg_from_mshr_S1)
    begin
        cache_type_S1 = mshr_cache_type_S1;
    end
    else
    begin
        cache_type_S1 = msg_cache_type_S1;
    end
end

always @ *
begin
    if (msg_from_mshr_S1)
    begin
        inv_fwd_pending_S1 = mshr_inv_fwd_pending_S1;
    end
    else
    begin
        inv_fwd_pending_S1 = 1'b0;
    end
end


reg [3-1:0] cs_S1;

always @ *
begin
    cs_S1 = {3{1'bx}};
    if (valid_S1)
    begin
        case (msg_type_S1)
        8'd23:
        begin
            //       tag_clk_en      tag_rdw_en   state_rd_en
            cs_S1 = {n,              rd,           y};
        end
        8'd21, 8'd22:
        begin
            cs_S1 = {n,              rd,         y};
        end
        8'd24, 8'd26:
        begin
            
            if (smc_miss_S1)
            begin
                cs_S1 = {n,              rd,           n};
            end
            else
            
            begin
                cs_S1 = {y,              wr,         n};
            end
        end
        8'd25, 8'd27:
        begin
            cs_S1 = {n,              rd,         n};
        end
        8'd12:
        begin
            cs_S1 = {y,              rd,          y};
        end
        default:
        begin
            cs_S1 = {3{1'bx}};
        end
        endcase
    end
    else
    begin
        cs_S1 = {3{1'b0}};
    end
end





always @ *
begin
    stall_S1 = valid_S1 && (stall_pre_S1 || stall_hazard_S1);
end

always @ *
begin
    msg_header_ready_S1 = !stall_S1; 
end


always @ *
begin
    tag_clk_en_S1 = valid_S1 && !stall_S1 && cs_S1[2];
end

always @ *
begin
    tag_rdw_en_S1 = valid_S1 && !stall_S1 && cs_S1[1];
end

always @ *
begin
    state_rd_en_S1 =  valid_S1 && !stall_S1 && cs_S1[0];
end


always @ *
begin
    if (msg_from_mshr_S1)
    begin
        smc_miss_S1 = mshr_smc_miss_S1;
    end
    else
    begin
        smc_miss_S1 = 0;
    end
end


reg valid_next_S1;

always @ *
begin
    valid_next_S1 = valid_S1 && !stall_S1;
end

always @ *
begin
    active_S1 = valid_S1;
//             || (valid_S1 && msg_type_S1 == `MSG_TYPE_WB_REQ)
//             || (valid_S2 && msg_type_S2_f == `MSG_TYPE_WB_REQ);
end


//============================
// Stage 1 -> Stage 2
//============================

reg valid_S2_f;
reg [8-1:0] msg_length_S2_f;
reg [1-1:0] msg_last_subline_S2_f;
reg [3-1:0] data_size_S2_f;
reg [1-1:0] cache_type_S2_f;
reg msg_from_mshr_S2_f;
reg [2-1:0] msg_mesi_S2_f;

reg smc_miss_S2_f;

reg [8-1:0] msg_type_S2_f;
reg inv_fwd_pending_S2_f;

always @ (posedge clk)
begin
    if (!rst_n)
    begin
        valid_S2_f <= 1'b0;
        msg_type_S2_f <= 0;
        msg_length_S2_f <= 0;
        msg_last_subline_S2_f <= 0;
        data_size_S2_f <= 0;  
        cache_type_S2_f <= 0; 
        msg_from_mshr_S2_f <= 1'b0;
        msg_mesi_S2_f <= 0;
        
        smc_miss_S2_f <= 0;
        
        inv_fwd_pending_S2_f <= 0;
    end
    else if (!stall_S2)
    begin
        valid_S2_f <= valid_next_S1;
        msg_type_S2_f <= msg_type_S1;
        msg_length_S2_f <= msg_length_S1;
        msg_last_subline_S2_f <= msg_last_subline_S1;
        data_size_S2_f <= data_size_S1;
        cache_type_S2_f <= cache_type_S1;
        msg_from_mshr_S2_f <= msg_from_mshr_S1;
        msg_mesi_S2_f <= msg_mesi_S1;
        
        smc_miss_S2_f <= smc_miss_S1;
        
        inv_fwd_pending_S2_f <= inv_fwd_pending_S1;
    end
end

//============================
// Stage 2
//============================

reg stall_real_S2;
reg stall_load_S2;
reg stall_before_S2_f;
reg stall_before_S2_next;
reg state_wr_en_S2;
reg mshr_wr_state_en_S2;
reg [2-1:0] mshr_state_in_S2;

always @ *
begin
    valid_S2 = valid_S2_f;
    msg_type_S2 = msg_type_S2_f;
    msg_from_mshr_S2 = msg_from_mshr_S2_f;
    data_size_S2 = data_size_S2_f;
    cache_type_S2 = cache_type_S2_f;
    stall_before_S2 = stall_before_S2_f;
end

always @ *
begin
    if (!rst_n)
    begin
        stall_before_S2_next = 0;
    end
    else
    begin
        stall_before_S2_next = stall_S2;
    end
end

always @ (posedge clk)
begin
    stall_before_S2_f <= stall_before_S2_next;
end


reg is_last_subline_S2;


always @ *
begin
    is_last_subline_S2 = msg_last_subline_S2_f;
end




reg [19-1:0] cs_S2;

always @ *
begin
    if (valid_S2)
    begin
    case (msg_type_S2_f)
        8'd21:
        begin
            case (l2_way_state_mesi_S2)
            2'b10:
            begin
                if (is_last_subline_S2)
                begin
                    
                    if (csm_en)
                    begin
                        if (lsid_S2 == 6'd63)
                        begin
                            if (subline_valid_S2)   
                            begin
                                if (msg_length_S2_f != 0)
                                begin
                                    //       data   dir        dir         dir      state      state        state        state       state 
                                    //       clk_en clk_en     rdw_en      clr_en   owner_en   owner_op     subline_en   subline_op  di_en  
                                    cs_S2 = {y,     y,         wr,         y,        y,         2'd0,     n,           2'd0,    n, 
                                    //       state   state          state    state           state   state
                                    //       vd_en   vd             mesi_en  mesi            lru_en  lru
                                             y,      2'b11,  y,      2'b11,      n,      1'b0};  
                                end     
                                else
                                begin       
                                    cs_S2 = {n,     y,         wr,         y,        y,         2'd0,     n,           2'd0,    n, 
                                             y,      2'b11,  y,      2'b11,      n,      1'b0};  

                                end
                            end
                            else
                            begin
                                cs_S2 = {n,     n,         wr,         n,       y,         2'd0,     n,           2'd0,    n, 
                                         y,      2'b11,  y,      2'b11,      n,      1'b0};    
                            end
                        end
                        else
                        begin
                            if (subline_valid_S2)   
                            begin
                                if (msg_length_S2_f != 0)
                                begin
                                    //       data   dir        dir         dir      state      state        state        state       state 
                                    //       clk_en clk_en     rdw_en      clr_en   owner_en   owner_op     subline_en   subline_op  di_en  
                                    cs_S2 = {y,     y,         wr,         n,        y,         2'd1,     y,           2'd1,    n, 
                                    //       state   state          state    state           state   state
                                    //       vd_en   vd             mesi_en  mesi            lru_en  lru
                                             y,      2'b11,  y,      2'b01,      n,      1'b0};  
                                end     
                                else
                                begin       
                                    cs_S2 = {n,     y,         wr,         n,        y,         2'd1,     y,           2'd1,    n, 
                                             y,      2'b11,  y,      2'b01,      n,      1'b0};  

                                end
                            end
                            else
                            begin
                                cs_S2 = {n,     y,         wr,         n,       y,         2'd1,     y,           2'd1,    n, 
                                         y,      2'b11,  y,      2'b01,      n,      1'b0};    
                            end
                        end
                    end
                    else
                    
                    begin
                        if (subline_valid_S2)   
                        begin
                            if (msg_length_S2_f != 0)
                            begin
                                //       data   dir        dir         dir      state      state        state        state       state 
                                //       clk_en clk_en     rdw_en      clr_en   owner_en   owner_op     subline_en   subline_op  di_en  
                                cs_S2 = {y,     y,         wr,         n,        y,         2'd0,     n,           2'd0,    n, 
                                //       state   state          state    state           state   state
                                //       vd_en   vd             mesi_en  mesi            lru_en  lru
                                         y,      2'b11,  y,      2'b01,      n,      1'b0};  
                            end     
                            else
                            begin       
                                cs_S2 = {n,     y,         wr,         n,        y,         2'd0,     n,           2'd0,    n, 
                                         y,      2'b11,  y,      2'b01,      n,      1'b0};  

                            end
                        end
                        else
                        begin
                            cs_S2 = {n,     y,         wr,         n,       y,         2'd0,     n,           2'd0,    n, 
                                     y,      2'b11,  y,      2'b01,      n,      1'b0};    
                        end

                    end
                end
                else
                begin
                    if (subline_valid_S2)   
                    begin
                        if (msg_length_S2_f != 0)
                        begin
                            cs_S2 = {y,     n,         rd,         n,       n,         2'd0,     n,           2'd0,    n,       
                                     n,      2'b01,  n,      2'b00,  n,      1'b0};
                        end
                        else
                        begin
                            cs_S2 = {n,     n,         rd,         n,       n,         2'd0,     n,           2'd0,    n,       
                                     n,      2'b01,  n,      2'b00,  n,      1'b0};
                        end
                    end
                    else
                    begin
                        cs_S2 = {n,     n,         rd,         n,       n,         2'd0,     n,           2'd0,    n,       
                                 n,      2'b01,  n,      2'b00,  n,      1'b0};
                    end
                end  
            end
            2'b00:
            begin
                cs_S2 = {n,     n,         rd,         n,       n,         2'd0,      n,           2'd0,    n,       
                         n,      2'b01,  n,      2'b00,  n,      1'b0};
            end
            default:
            begin
                cs_S2 = {19{1'bx}};
            end
            endcase
        end
        8'd22:
        begin
            case (l2_way_state_mesi_S2)
            2'b10:
            begin
                if (is_last_subline_S2)
                begin
                    if (subline_valid_S2)   
                    begin
                        if (msg_length_S2_f != 0)
                        begin
                            cs_S2 = {y,     y,         wr,         y,       y,         2'd0,     y,           2'd0,    n, 
                                     y,      2'b11,  y,      2'b00,      n,      1'b0};  
                        end
                        else
                        begin
                            cs_S2 = {n,     y,         wr,         y,       y,         2'd0,     y,           2'd0,    n, 
                                     y,      2'b11,  y,      2'b00,      n,      1'b0};  
                        end
                        
                    end
                    else
                    begin
                        cs_S2 = {n,     y,         wr,         y,       y,         2'd0,     y,           2'd0,    n, 
                                 y,      2'b11,  y,      2'b00,      n,      1'b0};  
                    end  
                end
                else
                begin
                    if (subline_valid_S2)   
                    begin
                        if (msg_length_S2_f != 0)
                        begin
                            cs_S2 = {y,     n,         rd,         n,       n,         2'd0,     n,           2'd0,    n,       
                                     n,      2'b01,  n,      2'b00,  n,      1'b0};
                        end
                        else
                        begin
                            cs_S2 = {n,     n,         rd,         n,       n,         2'd0,     n,           2'd0,    n,       
                                     n,      2'b01,  n,      2'b00,  n,      1'b0};
                        end
                    end
                    else
                    begin
                        cs_S2 = {n,     n,         rd,         n,       n,         2'd0,     n,           2'd0,    n,       
                                 n,      2'b01,  n,      2'b00,  n,      1'b0};
                    end 
                end 
            end
            2'b00:
            begin
                cs_S2 = {n,     n,         rd,         n,       n,         2'd0,     n,           2'd0,    n,       
                         n,      2'b01,  n,      2'b00,  n,      1'b0};
            end
            default:
            begin
                cs_S2 = {19{1'bx}};
            end
            endcase
        end
        8'd23:
        begin
            if (is_last_subline_S2)
            begin
                
                if (l2_way_state_mesi_S2 == 2'b11)
                begin   
                    if (broadcast_counter_max_S2)
                    begin
                        cs_S2 = {n,     n,         rd,         n,       y,         2'd0,     y,           2'd0,    n, 
                                 n,      2'b01,  y,      2'b00,      n,      1'b0};    
                    end
                    else
                    begin
                        cs_S2 = {n,     n,         rd,         n,       y,         2'd3,     n,           2'd0,    n, 
                                 n,      2'b01,  n,      2'b00, n,      1'b0};    
                    end
                end
                else 
                
                begin
                    
                    if ((l2_way_state_owner_S2 == 1) && (~smc_miss_S2_f) && (~inv_fwd_pending_S2_f))
                    


                    begin
                        cs_S2 = {n,     n,         rd,         n,       y,         2'd0,     y,           2'd0,    n, 
                                 n,      2'b01,  y,      2'b00,      n,      1'b0};    
                    end
                    else
                    begin
                        cs_S2 = {n,     n,         rd,         n,       y,         2'd3,     n,           2'd0,    n, 
                                 n,      2'b01,  n,      2'b00, n,      1'b0};    
                    end
                end
            end
            else
            begin
                cs_S2 = {n,     n,         rd,         n,       n,         2'd0,     n,           2'd0,    n,       
                         n,      2'b01,  n,      2'b00,  n,      1'b0};
            end  
        end

        8'd24:
        begin
            cs_S2 = {y,     y,         wr,         y,       n,         2'd0,     y,           2'd0,    n,       
                     y,      2'b10,  n,      2'b00,  n,      1'b0};
        end
        8'd26:
        begin
            
            if (smc_miss_S2_f)
            begin
                cs_S2 = {n,     n,         rd,         n,       n,         2'd0,     n,           2'd0,    n,       
                         n,      2'b10,  n,      2'b00,  n,      1'b0};
            end
            else
            
            begin
                cs_S2 = {y,     n,         rd,         n,       n,         2'd0,     n,           2'd0,    n,       
                         y,      2'b10,  n,      2'b00,  n,      1'b0};
            end
        end
        8'd25, 8'd27:
        begin
            cs_S2 = {n,     n,         rd,         n,       n,         2'd0,     n,           2'd0,    n,       
                     n,      2'b01,  n,      2'b00,  n,      1'b0};
        end
        8'd12:
        begin
            begin
            //should be the last line
            if (l2_way_state_subline_S2 == ({{(4-1){1'b0}}, 1'b1} << addr_S2[5:4]))
            begin
                cs_S2 = {y,     y,         wr,         y,       n,         2'd0,     y,        2'd3,    n,       
                         y,     2'b11,  y,      2'b00,     n,      1'b0};
            end
            else
            begin
                cs_S2 = {y,     n,         rd,         n,       n,         2'd0,     y,        2'd3,    n,       
                         n,      2'b01,  n,      2'b00,     n,      1'b0};
            end
            end
         end
        default:
        begin
            cs_S2 = {19{1'bx}};
        end
    endcase
    end
    else    
    begin
        cs_S2 = {19{1'b0}};
    end
end





always @ *
begin
    broadcast_counter_op_val_S2 = !stall_S2 && valid_S2 && is_last_subline_S2 
                               && (msg_type_S2_f == 8'd23) && (l2_way_state_mesi_S2 == 2'b11);
end

always @ *
begin
    if (broadcast_counter_max_S2)
    begin
        broadcast_counter_op_S2 = 2'd0;
    end
    else
    begin
        broadcast_counter_op_S2 = 2'd2;
    end
end


always @ *
begin
    dir_clk_en_S2 = !stall_S2 && cs_S2[17];
end

always @ *
begin
    dir_rdw_en_S2 = cs_S2[16];
end


always @ *
begin
    dir_clr_en_S2 = cs_S2[15];
end

always @ *
begin
    data_clk_en_S2 = !stall_real_S2 && cs_S2[18];
end
/*
always @ *
begin
    data_rdw_en_S2 = wr;
end
*/
assign data_rdw_en_S2 = wr;


always @ *
begin
    if (msg_type_S2_f == 8'd12 || msg_type_S2_f == 8'd25)
    begin
        mshr_wr_state_en_S2 = n;
        mshr_state_in_S2 = 2'd0;
    end
    else if (msg_type_S2_f == 8'd23 
          || msg_type_S2_f == 8'd21
          || msg_type_S2_f == 8'd22)
    begin
        if (is_last_subline_S2)
        begin
             
            if (msg_type_S2_f == 8'd23 
            && ((l2_way_state_owner_S2 != 1) || smc_miss_S2_f || inv_fwd_pending_S2_f))
            



            begin
                mshr_wr_state_en_S2 = n;
                mshr_state_in_S2 = 2'd0;
            end
            else
            begin
                mshr_wr_state_en_S2 = !stall_S2;
                mshr_state_in_S2 = 2'd2;
            end
        end
        else
        begin
            mshr_wr_state_en_S2 = n;
            mshr_state_in_S2 = 2'd0;
        end
    end
    else
    begin
        mshr_wr_state_en_S2 = !stall_S2;
        mshr_state_in_S2 = 2'd2;
    end
end


always @ *
begin
    state_owner_en_S2 = !stall_S2 && cs_S2[14];
end


always @ *
begin
    state_owner_op_S2 = cs_S2[13:12];
end

always @ *
begin
    state_subline_en_S2 = !stall_S2 && cs_S2[11];
end

always @ *
begin
    state_subline_op_S2 = cs_S2[10:9];
end

always @ *
begin
    state_di_en_S2 = cs_S2[8];
end

always @ *
begin
    state_vd_en_S2 = !stall_S2 && cs_S2[7];
end

always @ *
begin
    state_vd_S2 = cs_S2[6:5];
end

always @ *
begin
    state_mesi_en_S2 = !stall_S2 && cs_S2[4];
end

always @ *
begin
    state_mesi_S2 = cs_S2[3:2];
end

always @ *
begin
    state_lru_en_S2 = !stall_S2 && cs_S2[1];
end

always @ *
begin
    state_lru_op_S2 = cs_S2[0];
end

always @ *
begin
    state_wr_en_S2 = !stall_S2 && (state_owner_en_S2 || state_subline_en_S2 || state_vd_en_S2
                  ||  state_di_en_S2 || state_mesi_en_S2 || state_lru_en_S2 || state_rb_en_S2);
end



always @ *
begin
    msg_data_ready_S2 = !stall_real_S2 && (data_clk_en_S2 || smc_wr_en_S2);
end






/*
always @ *
begin
    state_rb_en_S2 = n; 
end
*/

always @ *
begin
    smc_wr_en_S2 = valid_S2 && smc_miss_S2_f && (msg_type_S2_f == 8'd26);
end



assign state_rb_en_S2 = n;

always @ *
begin
    if (msg_type_S2_f == 8'd24)
    begin
        l2_load_64B_S2 = y;
    end
    else    
    begin
        l2_load_64B_S2 = n;
    end
end

reg [2-1:0] l2_load_data_subline_S2_f;
reg [2-1:0] l2_load_data_subline_S2_next;

always @ *
begin
    if (!rst_n)
    begin
        l2_load_data_subline_S2_next = 2'd0;
    end
    else if (valid_S2 && !stall_real_S2 && l2_load_64B_S2)
    begin
        l2_load_data_subline_S2_next = l2_load_data_subline_S2_f + 1;
    end
    else
    begin
        l2_load_data_subline_S2_next = l2_load_data_subline_S2_f;
    end
end

always @ (posedge clk)
begin
    l2_load_data_subline_S2_f <= l2_load_data_subline_S2_next;
end


always @ *
begin
    if (l2_load_64B_S2)
    begin
        stall_load_S2 = (l2_load_data_subline_S2_f != 2'd3);
    end
    else
    begin
        stall_load_S2 = n;
    end
end


always @ *
begin
    l2_load_data_subline_S2 = l2_load_data_subline_S2_f;
end


always @ *
begin
    stall_real_S2 = valid_S2 && ((cs_S2[18] || smc_wr_en_S2) && !msg_data_valid_S2);
end






always @ *
begin
    stall_S2 = valid_S2 && (stall_real_S2 || stall_load_S2);
end




always @ *
begin
    active_S2 = valid_S2;
end

reg valid_next_S2;

always @ *
begin
    valid_next_S2 = valid_S2 && !stall_S2;
end


//============================
// Stage 2 -> Stage 3
//============================

reg valid_S3_f;
reg state_wr_en_S3_f;
reg mshr_wr_state_en_S3_f;
reg [2-1:0] mshr_state_in_S3_f;

reg smc_miss_S3_f;

reg msg_from_mshr_S3_f;
reg [8-1:0] msg_type_S3_f;

always @ (posedge clk)
begin
    if (!rst_n)
    begin
        valid_S3_f <= 1'b0;
        state_wr_en_S3_f <= 1'b0;
        mshr_wr_state_en_S3_f <= 0;
        mshr_state_in_S3_f <= 0;
        
        smc_miss_S3_f <= 0;
        
        msg_from_mshr_S3_f <= 0;
        msg_type_S3_f <= 0;
    end
    else if (!stall_S3)
    begin
        valid_S3_f <= valid_next_S2;
        state_wr_en_S3_f <= state_wr_en_S2;
        mshr_wr_state_en_S3_f <= mshr_wr_state_en_S2;
        mshr_state_in_S3_f <= mshr_state_in_S2;
        
        smc_miss_S3_f <= smc_miss_S2_f;
        
        msg_from_mshr_S3_f <= msg_from_mshr_S2_f;
        msg_type_S3_f <= msg_type_S2_f;
    end
end

//============================
// Stage 3
//============================



always @ *
begin
    valid_S3 = valid_S3_f;
    state_wr_en_S3 = !stall_S3 && valid_S3 && state_wr_en_S3_f;
    mshr_wr_state_en_S3 = !stall_S3 && valid_S3 && mshr_wr_state_en_S3_f;
    mshr_state_in_S3 = mshr_state_in_S3_f;
    msg_type_S3 = msg_type_S3_f;
end

assign mshr_wr_data_en_S3 = 1'b0;


always @ *
begin
    mshr_inc_counter_en_S3 = valid_S3 && (msg_type_S3_f == 8'd23);
end

always @ *
begin
    active_S3 = valid_S3;
end

assign stall_S3 = 1'b0;

/*
//============================
// Debug
//============================

`ifndef SYNTHESIS


wire [15*8-1:0] msg_type_string_S1;
wire [15*8-1:0] msg_type_string_S2;

l2_msg_type_parse msg_type_parse_S1(
    .msg_type (msg_type_S1),
    .msg_type_string (msg_type_string_S1)
);

always @ (posedge clk)
begin
    if (valid_S1 && !stall_S1)
    begin
        $display("-------------------------------------");
        $display($time);
        $display("P2S1 msg type: %s, data_size: %b, cache_type: %b, last_subline: %b", msg_type_string_S1, data_size_S1, cache_type_S1, msg_last_subline_S1);
        $display("P2S1 valid: stall: %b, stall_pre: %b, stall_hazard: %b",
                  stall_S1, stall_pre_S1, stall_hazard_S1);
        $display("Control signals: %b", cs_S1);
        $display("Msg from mshr: %b", msg_from_mshr_S1);
    end
end


l2_msg_type_parse msg_type_parse_S2(
    .msg_type (msg_type_S2_f),
    .msg_type_string (msg_type_string_S2)
);

always @ (posedge clk)
begin
    if (valid_S2 && !stall_S2)
    begin
        $display("-------------------------------------");
        $display($time);
        $display("P2S2 msg type: %s, data_size: %b, cache_type: %b, last_subline: %b", msg_type_string_S2, data_size_S2_f, cache_type_S2_f,msg_last_subline_S2_f);
        $display("P2S2 valid: stall: %b, stall_real: %b, stall_load: %b",
                  stall_S2, stall_real_S2,stall_load_S2);
        $display("Control signals: %b", cs_S2);
        $display("Msg from mshr: %b", msg_from_mshr_S2);
    end
end




`endif
*/
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
//  Filename      : l2_pipe2_dpath.v
//  Created On    : 2014-04-03
//  Revision      :
//  Author        : Yaosheng Fu
//  Company       : Princeton University
//  Email         : yfu@princeton.edu
//
//  Description   : The datapath for pipeline2 in the L2 cache
//
//
//==================================================================================================


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
//  Filename      : l2.h.pyv
//  Created On    : 2014-02-20
//  Revision      :
//  Author        : Yaosheng Fu
//  Company       : Princeton University
//  Email         : yfu@princeton.edu
//
//  Description   : header file for the L2 cache
//
//
//==================================================================================================

// devices.xml



// Input buffer for pipeline1













// Input buffer for pipeline2














// Output buffer















// L2 cache configuration







//`define L2_SIZE                  65536
//`define L2_SIZE_WIDTH            16






//`define L2_LINE_SIZE             64
//`define L2_LINE_SIZE_WIDTH       6













//`define L2_WAYS                  4
//`define L2_WAYS_WIDTH            2
//`define L2_WAY_0                 2'b00
//`define L2_WAY_1                 2'b01
//`define L2_WAY_2                 2'b10
//`define L2_WAY_3                 2'b11





// Tag array







//`define L2_TAG_INDEX_WIDTH      8
//`define L2_TAG_WIDTH            26
//`define L2_TAG_WAY_WIDTH        26
//`define L2_TAG_ARRAY_WIDTH      104


// Tag Address decomposition 




//`define L2_TAG_INDEX            13:6
//`define L2_TAG                  39:14


//Data array












//`define L2_DATA_INDEX_WIDTH         12 








// Data Address decomposition 







//Dir array



//`define L2_DIR_INDEX_WIDTH      10 





//State array

















//Whether the cache line is in Icaches or Dcaches
















//Round Robin selection














// State decomposition 












//MSHR array



































//`define L2_MSHR_CMP_ADDR        13:6
//`define L2_MSHR_ADDR            39:0
//`define L2_MSHR_WAY             41:40
//`define L2_MSHR_MSHRID          49:42
//`define L2_MSHR_CACHE_TYPE      50
//`define L2_MSHR_DATA_SIZE       53:51
//`define L2_MSHR_MSG_TYPE        61:54
//`define L2_MSHR_L2_MISS         62
//`define L2_MSHR_SRC_CHIPID      76:63
//`define L2_MSHR_SRC_X           84:77
//`define L2_MSHR_SRC_Y           92:85
//`define L2_MSHR_SRC_FBITS       96:93
//`define L2_MSHR_SDID            106:97
//`define L2_MSHR_LSID            112:107      
//`define L2_MSHR_MISS_LSID       118:113
//`define L2_MSHR_SMC_MISS        119
//`define L2_MSHR_RECYCLED        120
//`define L2_MSHR_INV_FWD_PENDING 121


//SMC array


































//Message destination_type







//L2 public sharer beyond the maximum clump size



//L2 registers



//Special addresses





















//L2 core ID





//L2 dir array reuse







//Control Signal 








//AMO ALU OP macros












//Control Signal in Stage 1























































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


module l2_pipe2_dpath(

    input wire clk,
    input wire rst_n,

    //Inputs to Stage 1   

    input wire [40-1:0] mshr_addr_S1,
    input wire [8-1:0] mshr_mshrid_S1,
    input wire [2-1:0] mshr_way_S1,
    input wire [14-1:0] mshr_src_chipid_S1,
    input wire [8-1:0] mshr_src_x_S1,
    input wire [8-1:0] mshr_src_y_S1,
    input wire [4-1:0] mshr_src_fbits_S1,
    input wire [10-1:0] mshr_sdid_S1,
    input wire [6-1:0] mshr_lsid_S1,
    input wire [6-1:0] mshr_miss_lsid_S1,


    input wire [40-1:0] msg_addr_S1,
    input wire [8-1:0] msg_type_S1,
    input wire [2-1:0] msg_subline_id_S1,
    input wire [8-1:0] msg_mshrid_S1,
    input wire [14-1:0] msg_src_chipid_S1,
    input wire [8-1:0] msg_src_x_S1,
    input wire [8-1:0] msg_src_y_S1,
    input wire [4-1:0] msg_src_fbits_S1,
    input wire [10-1:0] msg_sdid_S1,
    input wire [6-1:0] msg_lsid_S1,

    input wire valid_S1,
    input wire stall_S1,
    input wire msg_from_mshr_S1, 
    
    //Inputs to Stage 2   

    input wire [15*4+2+4-1:0] state_data_S2,
    input wire [104-1:0] tag_data_S2,

    input wire [128-1:0] msg_data_S2,

    input wire msg_from_mshr_S2,
    input wire [8-1:0] msg_type_S2,
   
    input wire [3-1:0] data_size_S2,
    input wire [1-1:0] cache_type_S2,
    input wire state_owner_en_S2,
    input wire [2-1:0] state_owner_op_S2,
    input wire state_subline_en_S2,
    input wire [2-1:0] state_subline_op_S2,
    input wire state_di_en_S2,
    input wire state_vd_en_S2,
    input wire [2-1:0] state_vd_S2,
    input wire state_mesi_en_S2,
    input wire [2-1:0] state_mesi_S2,
    input wire state_lru_en_S2,
    input wire [1-1:0] state_lru_op_S2,
    input wire state_rb_en_S2,
    input wire dir_clr_en_S2,
   
    input wire l2_load_64B_S2, 
    input wire [2-1:0] l2_load_data_subline_S2,
 
    input wire valid_S2,
    input wire stall_S2,
    input wire stall_before_S2,

    //Inputs to Stage 3 
    input wire valid_S3,
    input wire stall_S3,
    
    //Outputs from Stage 1
    
    output reg [40-1:0] addr_S1,
    output reg [3-1:0] mshr_rd_index_S1,
    output reg [8-1:0] tag_addr_S1,
    output reg [8-1:0] state_rd_addr_S1,
    
    output reg [104-1:0] tag_data_in_S1,
    output reg [104-1:0] tag_data_mask_in_S1,
    output reg is_same_address_S1,

    //Outputs from Stage 2
   
 
    output reg [40-1:0] addr_S2,
    output reg l2_tag_hit_S2,
    output reg [2-1:0] l2_way_sel_S2,
    output reg l2_wb_S2,
    output reg [6-1:0] l2_way_state_owner_S2,
    output reg [2-1:0] l2_way_state_mesi_S2,
    output reg [2-1:0] l2_way_state_vd_S2,
    output reg [4-1:0] l2_way_state_subline_S2,
    output reg [1-1:0] l2_way_state_cache_type_S2,
    output reg addr_l2_aligned_S2,
    output reg subline_valid_S2, 
    output reg [6-1:0] lsid_S2,

    output reg [8+2-1:0] dir_addr_S2,
    output reg [64-1:0] dir_data_in_S2,
    output wire [64-1:0] dir_data_mask_in_S2,

    output reg [8+2+2-1:0] data_addr_S2,
    output reg [144-1:0] data_data_in_S2,
    output wire [144-1:0] data_data_mask_in_S2,

    
    output reg [16-1:0] smc_wr_addr_in_S2,
    output reg [128-1:0] smc_data_in_S2,
    

    //Outputs from Stage 3
    output reg [40-1:0] addr_S3,
    output reg [3-1:0] mshr_wr_index_S3,
    output wire [120+2-1:0] mshr_data_in_S3,
    output wire [120+2-1:0] mshr_data_mask_in_S3,
    output reg [8-1:0] state_wr_addr_S3,
    output reg [15*4+2+4-1:0] state_data_in_S3,
    output reg [15*4+2+4-1:0] state_data_mask_in_S3

);


//============================
// Stage 1
//============================

reg [8-1:0] mshrid_S1;
reg [14-1:0] src_chipid_S1;
reg [8-1:0] src_x_S1;
reg [8-1:0] src_y_S1;
reg [4-1:0] src_fbits_S1;
reg [10-1:0] sdid_S1;
reg [6-1:0] lsid_S1;

always @ *
begin
    if (msg_from_mshr_S1)
    begin
        addr_S1 = {mshr_addr_S1[39:6+8], mshr_addr_S1[6+8-1:6],
                   msg_subline_id_S1, mshr_addr_S1[3:0]};
        src_chipid_S1 = mshr_src_chipid_S1;
        src_x_S1 = mshr_src_x_S1;   
        src_y_S1 = mshr_src_y_S1;   
        src_fbits_S1 = mshr_src_fbits_S1;
        sdid_S1 = mshr_sdid_S1;
        lsid_S1 = mshr_lsid_S1;
    end
    else
    begin
        addr_S1 = msg_addr_S1;
        src_chipid_S1 = msg_src_chipid_S1;
        src_x_S1 = msg_src_x_S1;   
        src_y_S1 = msg_src_y_S1;   
        src_fbits_S1 = msg_src_fbits_S1;
        sdid_S1 = msg_sdid_S1;
        lsid_S1 = msg_lsid_S1;
    end
end

always @ *
begin
    is_same_address_S1 = (mshr_addr_S1 == msg_addr_S1);
end

always @ *
begin
    mshrid_S1 = msg_mshrid_S1;
end


always @ *
begin
    mshr_rd_index_S1 = msg_mshrid_S1;
end


always @ *
begin
    tag_addr_S1 = addr_S1[6+8-1:6];
end

always @ *
begin
    state_rd_addr_S1 = addr_S1[6+8-1:6];
end


always @ *
begin
    tag_data_in_S1 = {4{addr_S1[39:6+8]}};
end

always @ *
begin
    tag_data_mask_in_S1 = {{(4-1)*26{1'b0}},{26{1'b1}}} 
                       << (mshr_way_S1 * 26);
end


//============================
// Stage 1 -> Stage 2
//============================


reg [40-1:0] addr_S2_f;
reg [8-1:0] mshrid_S2_f;
reg [14-1:0] src_chipid_S2_f;
reg [8-1:0] src_x_S2_f;
reg [8-1:0] src_y_S2_f;
reg [4-1:0] src_fbits_S2_f;
reg [10-1:0] sdid_S2_f;
reg [6-1:0] lsid_S2_f;
reg [2-1:0] mshr_way_S2_f;
reg [2-1:0] msg_subline_id_S2_f;
reg [6-1:0] mshr_miss_lsid_S2_f;

always @ (posedge clk)
begin
    if (!rst_n)
    begin
        addr_S2_f <= 0; 
        mshrid_S2_f <= 0;
        src_chipid_S2_f <= 0;
        src_x_S2_f <= 0;
        src_y_S2_f <= 0;
        src_fbits_S2_f <= 0;
        sdid_S2_f <= 0;
        lsid_S2_f <= 0;
        mshr_way_S2_f <= 0;
        msg_subline_id_S2_f <= 0;
        mshr_miss_lsid_S2_f <= 0;
    end
    else if (!stall_S2)
    begin
        addr_S2_f <= addr_S1;
        mshrid_S2_f <= mshrid_S1;
        src_chipid_S2_f <= src_chipid_S1;
        src_x_S2_f <= src_x_S1;
        src_y_S2_f <= src_y_S1;
        src_fbits_S2_f <= src_fbits_S1;
        sdid_S2_f <= sdid_S1;
        lsid_S2_f <= lsid_S1;
        mshr_way_S2_f <= mshr_way_S1;
        msg_subline_id_S2_f <= msg_subline_id_S1;
        mshr_miss_lsid_S2_f <= mshr_miss_lsid_S1;
    end
end


//============================
// Stage 2
//============================


reg [15*4+2+4-1:0] state_data_in_S2;
reg [15*4+2+4-1:0] state_data_mask_in_S2;

always @ *
begin
    addr_S2 = addr_S2_f;
    lsid_S2 = lsid_S2_f;
end


reg [104-1:0] tag_data_buf_S2_f;
reg [104-1:0] tag_data_buf_S2_next;
reg [104-1:0] tag_data_trans_S2;

always @ *
begin
    if (!rst_n)
    begin
        tag_data_buf_S2_next = 0;
    end
    else if (stall_S2 && !stall_before_S2)
    begin
        tag_data_buf_S2_next = tag_data_S2;
    end
    else
    begin
        tag_data_buf_S2_next = tag_data_buf_S2_f;
    end
end

always @ (posedge clk)
begin
    tag_data_buf_S2_f <= tag_data_buf_S2_next;
end

always @ *
begin
    if (stall_before_S2)
    begin
        tag_data_trans_S2 = tag_data_buf_S2_f;
    end
    else
    begin
        tag_data_trans_S2 = tag_data_S2;
    end
end

reg [104-1:0] state_data_buf_S2_f;
reg [104-1:0] state_data_buf_S2_next;
reg [104-1:0] state_data_trans_S2;

always @ *
begin
    if (!rst_n)
    begin
        state_data_buf_S2_next = 0;
    end
    else if (stall_S2 && !stall_before_S2)
    begin
        state_data_buf_S2_next = state_data_S2;
    end
    else
    begin
        state_data_buf_S2_next = state_data_buf_S2_f;
    end
end

always @ (posedge clk)
begin
    state_data_buf_S2_f <= state_data_buf_S2_next;
end

always @ *
begin
    if (stall_before_S2)
    begin
        state_data_trans_S2 = state_data_buf_S2_f;
    end
    else
    begin
        state_data_trans_S2 = state_data_S2;
    end
end


reg [2-1:0] l2_hit_way_sel_S2;
reg [2-1:0] l2_rb_bits_S2;
reg [4-1:0] l2_lru_bits_S2;


always @ *
begin
    l2_rb_bits_S2 = state_data_trans_S2[15*4+2+4-1:15*4+4];
    l2_lru_bits_S2 = state_data_trans_S2[15*4+4-1:15*4];
end


reg [26 - 1:0] tag_data_way_S2 [3:0];


reg [3:0] tag_hit_way_S2;


reg [15 - 1:0] state_way_S2 [3:0];





always @ *
begin
    tag_data_way_S2[0] = tag_data_trans_S2[26 * 1 - 1: 26 * 0];
    tag_data_way_S2[1] = tag_data_trans_S2[26 * 2 - 1: 26 * 1];
    tag_data_way_S2[2] = tag_data_trans_S2[26 * 3 - 1: 26 * 2];
    tag_data_way_S2[3] = tag_data_trans_S2[26 * 4 - 1: 26 * 3];

end

always @ *
begin
    state_way_S2[0] = state_data_trans_S2[15 * 1 - 1: 
15 * 0];
    state_way_S2[1] = state_data_trans_S2[15 * 2 - 1: 
15 * 1];
    state_way_S2[2] = state_data_trans_S2[15 * 3 - 1: 
15 * 2];
    state_way_S2[3] = state_data_trans_S2[15 * 4 - 1: 
15 * 3];

end

always @ *
begin
    if ((addr_S2_f[39:6+8] == tag_data_way_S2[0]) && 
(state_way_S2[0][12:11] == 2'b10 || state_way_S2[0][12:11] == 2'b11 ))
    begin
        tag_hit_way_S2[0] = 1'b1;
    end
    else
    begin
        tag_hit_way_S2[0] = 1'b0;
    end
end
always @ *
begin
    if ((addr_S2_f[39:6+8] == tag_data_way_S2[1]) && 
(state_way_S2[1][12:11] == 2'b10 || state_way_S2[1][12:11] == 2'b11 ))
    begin
        tag_hit_way_S2[1] = 1'b1;
    end
    else
    begin
        tag_hit_way_S2[1] = 1'b0;
    end
end
always @ *
begin
    if ((addr_S2_f[39:6+8] == tag_data_way_S2[2]) && 
(state_way_S2[2][12:11] == 2'b10 || state_way_S2[2][12:11] == 2'b11 ))
    begin
        tag_hit_way_S2[2] = 1'b1;
    end
    else
    begin
        tag_hit_way_S2[2] = 1'b0;
    end
end
always @ *
begin
    if ((addr_S2_f[39:6+8] == tag_data_way_S2[3]) && 
(state_way_S2[3][12:11] == 2'b10 || state_way_S2[3][12:11] == 2'b11 ))
    begin
        tag_hit_way_S2[3] = 1'b1;
    end
    else
    begin
        tag_hit_way_S2[3] = 1'b0;
    end
end



always @ *
begin
    if (msg_from_mshr_S2)
    begin
        l2_tag_hit_S2 = tag_hit_way_S2[mshr_way_S2_f];
    end
    else
        l2_tag_hit_S2 = tag_hit_way_S2[0] || tag_hit_way_S2[1] || tag_hit_way_S2[2] || tag_hit_way_S2[3];

end

always @ *
begin
    l2_hit_way_sel_S2 = {2{1'bx}};
    if (tag_hit_way_S2[0])
    begin
        l2_hit_way_sel_S2 = 2'd0;
    end
    if (tag_hit_way_S2[1])
    begin
        l2_hit_way_sel_S2 = 2'd1;
    end
    if (tag_hit_way_S2[2])
    begin
        l2_hit_way_sel_S2 = 2'd2;
    end
    if (tag_hit_way_S2[3])
    begin
        l2_hit_way_sel_S2 = 2'd3;
    end

end


always @ *
begin
    if(valid_S2)
    begin
        if (msg_from_mshr_S2)
        begin
            l2_way_sel_S2 = mshr_way_S2_f;
        end
        else
        begin
            l2_way_sel_S2 = l2_hit_way_sel_S2;
        end
    end
    else
    begin
        l2_way_sel_S2 = 0;
    end
end


always @ *
begin
    if (!l2_tag_hit_S2 && (state_way_S2[l2_way_sel_S2][12:11] == 2'b11))
    begin
        l2_wb_S2 = 1'b1;
    end
    else
    begin
        l2_wb_S2 = 1'b0;
    end
end



always @ *
begin
    l2_way_state_mesi_S2 = state_way_S2[l2_way_sel_S2][14:13];
    l2_way_state_vd_S2 = state_way_S2[l2_way_sel_S2][12:11];
    l2_way_state_subline_S2 = state_way_S2[l2_way_sel_S2][9:6];
    l2_way_state_cache_type_S2 = state_way_S2[l2_way_sel_S2][10];
    l2_way_state_owner_S2 = state_way_S2[l2_way_sel_S2][5:0];
end





always @ *
begin
    dir_addr_S2 = {addr_S2_f[6+8-1:6],l2_way_sel_S2}; 
end

always @ *
begin
    if (l2_load_64B_S2)
    begin
        data_addr_S2 = {addr_S2_f[6+8-1:6],l2_way_sel_S2, l2_load_data_subline_S2};
    end
    else
    begin
        data_addr_S2 = {addr_S2_f[6+8-1:6],l2_way_sel_S2, addr_S2_f[5:4]};
    end
end



always @ *
begin
    addr_l2_aligned_S2 = (addr_S2_f[6-1:0] == {6{1'b0}}); 
end

/*
always @ *
begin
    dir_data_mask_in_S2 = {`L2_DIR_ARRAY_WIDTH{1'b1}}; 
end
*/
assign dir_data_mask_in_S2 = {64{1'b1}};

//TODO
always @ *
begin
    if (dir_clr_en_S2)
    begin
        dir_data_in_S2 = {(64){1'b0}}; 
    end
    else
    begin
        dir_data_in_S2 = {{(64-1){1'b0}},1'b1} << l2_way_state_owner_S2; 
    end
end


wire [8-1:0] msg_data_parity1_S2;
wire [8-1:0] msg_data_parity2_S2;

l2_data_pgen data_pgen1( 
    .din            (msg_data_S2[64-1:0]),
    .parity         (msg_data_parity1_S2)
);

l2_data_pgen data_pgen2( 
    .din            (msg_data_S2[128-1:64]),
    .parity         (msg_data_parity2_S2)
);


always @ *
begin
    data_data_in_S2 = {msg_data_parity2_S2, msg_data_S2[127:64], msg_data_parity1_S2, msg_data_S2[63:0]}; 
end

assign data_data_mask_in_S2 = {144{1'b1}}; 


reg [6-1:0] state_owner_S2;
reg [4-1:0] state_subline_S2;
reg [2-1:0] state_rb_S2;
reg [4-1:0] state_lru_S2;


always @ *
begin
    state_owner_S2 = l2_way_state_owner_S2; 
    if (state_owner_op_S2 == 2'd1)
    begin
        state_owner_S2 = sdid_S2_f[5:0]; 
    end
    else if (state_owner_op_S2 == 2'd2)
    begin
        state_owner_S2 = l2_way_state_owner_S2 + 1; 
    end
    else if (state_owner_op_S2 == 2'd3)
    begin
        state_owner_S2 = l2_way_state_owner_S2 - 1; 
    end
    else if (state_owner_op_S2 == 2'd0)
    begin
        state_owner_S2 = 0; 
    end
end

reg [4-1:0] addr_subline_S2;

always @ *
begin
    if (cache_type_S2 == 1'b0)
    begin
        //addr_subline_S2= {1'b1, {(`L2_SUBLINE_BITS-1){1'b0}}} >> addr_S2_f[`L2_DATA_SUBLINE];
        addr_subline_S2= {{(4-1){1'b0}},1'b1} << addr_S2_f[5:4];
    end
    else
    begin
        //addr_subline_S2= {2'b11, {(`L2_SUBLINE_BITS-2){1'b0}}} >> (2*addr_S2_f[`L2_INS_SUBLINE]);
        addr_subline_S2= {{(4-2){1'b0}},2'b11} << (2*addr_S2_f[5]);
    end
end


always @ *
begin
    if (state_subline_op_S2 == 2'd1)
    begin
        state_subline_S2 = sdid_S2_f[9:6];
    end
    else if (state_subline_op_S2 == 2'd2)
    begin
        state_subline_S2 = l2_way_state_subline_S2 | addr_subline_S2;
    end
    else if (state_subline_op_S2 == 2'd3)
    begin
        state_subline_S2 = l2_way_state_subline_S2 & (~addr_subline_S2);
    end
    else if (state_subline_op_S2 == 2'd0)
    begin
        state_subline_S2 = {4{1'b0}};
    end
    else
    begin
        state_subline_S2 = {4{1'bx}};
    end
end

always @ *
begin
    state_rb_S2 = l2_rb_bits_S2 + 1; 
end


always @ *
begin
    if (state_lru_en_S2)
    begin
        if (state_lru_op_S2 == 1'b0)
        begin
            state_lru_S2 = l2_lru_bits_S2 & (~({{(4-1){1'b0}},1'b1} << l2_way_sel_S2));
        end
        else
        begin
            state_lru_S2 = l2_lru_bits_S2 | ({{(4-1){1'b0}},1'b1} << l2_way_sel_S2);
            if (state_lru_S2 == {4{1'b1}})
            begin
                state_lru_S2 = {4{1'b0}};
            end
        end
    end
    else
    begin
        state_lru_S2 = l2_lru_bits_S2; 
    end
end


always @ *
begin
    subline_valid_S2 = l2_way_state_subline_S2[msg_subline_id_S2_f]; 
end



always @ *
begin
    state_data_in_S2 = {state_rb_S2, state_lru_S2, 
    {4{state_mesi_S2, state_vd_S2, cache_type_S2, state_subline_S2, state_owner_S2}}};
end

reg [4*15-1:0] state_way_data_mask_in_S2;


always @ *
begin
    state_way_data_mask_in_S2 = {{(4-1)*15{1'b0}},
                                {{2{state_mesi_en_S2}}, 
                                 {2{state_vd_en_S2}}, 
                                 {1{state_di_en_S2}}, 
                                 {4{state_subline_en_S2}}, 
                                 {6{state_owner_en_S2}}}} 
    << (l2_way_sel_S2 * 15); 
end

always @ *
begin
    state_data_mask_in_S2 = {{2{state_rb_en_S2}}, 
                             {4{state_lru_en_S2}},
                              state_way_data_mask_in_S2}; 
end


always @ *
begin
    smc_wr_addr_in_S2 = {sdid_S2_f, mshr_miss_lsid_S2_f}; 
end

always @ *
begin
    smc_data_in_S2 = msg_data_S2; 
end



//============================
// Stage 2 -> Stage 3
//============================


reg [40-1:0] addr_S3_f;
reg [15*4+2+4-1:0] state_data_in_S3_f;
reg [15*4+2+4-1:0] state_data_mask_in_S3_f;
reg [8-1:0] mshrid_S3_f;
reg [6-1:0] mshr_miss_lsid_S3_f;

always @ (posedge clk)
begin
    if (!rst_n)
    begin
        addr_S3_f <= 0; 
        state_data_in_S3_f <= 0;
        state_data_mask_in_S3_f <= 0;
        mshrid_S3_f <= 0;
        mshr_miss_lsid_S3_f <= 0;
    end
    else if (!stall_S3)
    begin
        addr_S3_f <= addr_S2_f;
        state_data_in_S3_f <= state_data_in_S2;
        state_data_mask_in_S3_f <= state_data_mask_in_S2;
        mshrid_S3_f <= mshrid_S2_f;
        mshr_miss_lsid_S3_f <= mshr_miss_lsid_S2_f;
    end
end


//============================
// Stage 3
//============================

always @ *
begin
    state_data_in_S3 = state_data_in_S3_f;
    state_data_mask_in_S3 = state_data_mask_in_S3_f;
    addr_S3 = addr_S3_f;
end


always @ *
begin
    state_wr_addr_S3 = addr_S3_f[6+8-1:6]; 
end

always @ *
begin
    mshr_wr_index_S3 = mshrid_S3_f; 
end

assign mshr_data_in_S3 = {120+2{1'b0}}; 
assign mshr_data_mask_in_S3 = {1'b1, {(120+2-1){1'b0}}}; 

/*
//============================
// Debug
//============================

`ifndef SYNTHESIS
    

always @ (posedge clk)
begin
    if (valid_S1 && !stall_S1)
    begin
        $display("-------------------------------------");
        $display($time);
        $display("P2S1 addr: 0x%h", addr_S1);
        $display("Mshr_rd_index: %b", mshr_rd_index_S1);
        $display("Tag_addr: 0x%h", tag_addr_S1,);
        $display("Tag_data_in: 0x%h", tag_data_in_S1,);
        $display("Tag_data_mask_in: 0x%h", tag_data_mask_in_S1,);
        $display("State_rd_addr: 0x%h",state_rd_addr_S1);
        $display("Msg from mshr: %b", msg_from_mshr_S1);
    end
end


always @ (posedge clk)
begin
    if (valid_S2 && !stall_S2)
    begin
        $display("-------------------------------------");
        $display($time);
        $display("P2S2 addr: 0x%h", addr_S2);
        $display("P2S2 valid: l2_way_sel: %b, l2_hit: %b, l2_wb: %b",
                  l2_way_sel_S2, l2_tag_hit_S2, l2_wb_S2);
        $display("state: mesi: %b, vd: %b, subline: %b, cache_type: %b, owner: %b",
                 l2_way_state_mesi_S2, l2_way_state_vd_S2, l2_way_state_subline_S2, l2_way_state_cache_type_S2, l2_way_state_owner_S2);
        $display("Msg from mshr: %b", msg_from_mshr_S2);
        $display("Mshr wr index: %b", mshr_wr_index_S2);
        $display("Dir addr: 0x%h", dir_addr_S2);
        $display("Dir data: 0x%h", dir_data_in_S2);
        $display("Dir data mask: 0x%h", dir_data_mask_in_S2);
        $display("Data addr: 0x%h", data_addr_S2);
        $display("Data data: 0x%h", data_data_in_S2);
        $display("Data data mask: 0x%h", data_data_mask_in_S2);
        $display("State wr addr: 0x%h", state_wr_addr_S2);
        $display("State data: 0x%h", state_data_in_S2);
        $display("State data mask: 0x%h", state_data_mask_in_S2);
    end
end





`endif
*/
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
//  Filename      : l2_priority_encoder.v
//  Created On    : 2014-07-09
//  Revision      :
//  Author        : Yaosheng Fu
//  Company       : Princeton University
//  Email         : yfu@princeton.edu
//
//  Description   : The priority encoder for pipeline1 in the L2 cache
//
//
//==================================================================================================


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
//  Filename      : l2.h.pyv
//  Created On    : 2014-02-20
//  Revision      :
//  Author        : Yaosheng Fu
//  Company       : Princeton University
//  Email         : yfu@princeton.edu
//
//  Description   : header file for the L2 cache
//
//
//==================================================================================================

// devices.xml



// Input buffer for pipeline1













// Input buffer for pipeline2














// Output buffer















// L2 cache configuration







//`define L2_SIZE                  65536
//`define L2_SIZE_WIDTH            16






//`define L2_LINE_SIZE             64
//`define L2_LINE_SIZE_WIDTH       6













//`define L2_WAYS                  4
//`define L2_WAYS_WIDTH            2
//`define L2_WAY_0                 2'b00
//`define L2_WAY_1                 2'b01
//`define L2_WAY_2                 2'b10
//`define L2_WAY_3                 2'b11





// Tag array







//`define L2_TAG_INDEX_WIDTH      8
//`define L2_TAG_WIDTH            26
//`define L2_TAG_WAY_WIDTH        26
//`define L2_TAG_ARRAY_WIDTH      104


// Tag Address decomposition 




//`define L2_TAG_INDEX            13:6
//`define L2_TAG                  39:14


//Data array












//`define L2_DATA_INDEX_WIDTH         12 








// Data Address decomposition 







//Dir array



//`define L2_DIR_INDEX_WIDTH      10 





//State array

















//Whether the cache line is in Icaches or Dcaches
















//Round Robin selection














// State decomposition 












//MSHR array



































//`define L2_MSHR_CMP_ADDR        13:6
//`define L2_MSHR_ADDR            39:0
//`define L2_MSHR_WAY             41:40
//`define L2_MSHR_MSHRID          49:42
//`define L2_MSHR_CACHE_TYPE      50
//`define L2_MSHR_DATA_SIZE       53:51
//`define L2_MSHR_MSG_TYPE        61:54
//`define L2_MSHR_L2_MISS         62
//`define L2_MSHR_SRC_CHIPID      76:63
//`define L2_MSHR_SRC_X           84:77
//`define L2_MSHR_SRC_Y           92:85
//`define L2_MSHR_SRC_FBITS       96:93
//`define L2_MSHR_SDID            106:97
//`define L2_MSHR_LSID            112:107      
//`define L2_MSHR_MISS_LSID       118:113
//`define L2_MSHR_SMC_MISS        119
//`define L2_MSHR_RECYCLED        120
//`define L2_MSHR_INV_FWD_PENDING 121


//SMC array


































//Message destination_type







//L2 public sharer beyond the maximum clump size



//L2 registers



//Special addresses





















//L2 core ID





//L2 dir array reuse







//Control Signal 








//AMO ALU OP macros












//Control Signal in Stage 1























































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








































































































































































































































































































































































































































































































































































































module l2_priority_encoder_1(
    input wire [1:0] data_in,
    output wire [0:0] data_out,
    output wire [1:0] data_out_mask,
    output wire nonzero_out
);

assign data_out = data_in[0] ? 1'b0 : 1'b1;
assign data_out_mask = data_in[0] ? 2'b10 : 2'b01;
assign nonzero_out = | (data_in[1:0]);
endmodule

module l2_priority_encoder_2(
    input wire [3:0] data_in,
    output wire [1:0] data_out,
    output wire [3:0] data_out_mask,
    output wire nonzero_out
);

wire [0:0] data_low;

wire [0:0] data_high;

wire [1:0] data_low_mask;

wire [1:0] data_high_mask;

wire nonzero_low;

wire nonzero_high;

l2_priority_encoder_1 encoder_high_1 (.data_in(data_in[3:2]), .data_out(data_high), .data_out_mask(data_high_mask), .nonzero_out(nonzero_high));

l2_priority_encoder_1 encoder_low_1(.data_in(data_in[1:0]), .data_out(data_low), .data_out_mask(data_low_mask), .nonzero_out(nonzero_low));

assign data_out = nonzero_low ? {1'b0, data_low} : {1'b1, data_high};

assign data_out_mask = nonzero_low ? {{2{1'b1}}, data_low_mask} : {data_high_mask,{2{1'b1}}};

assign nonzero_out = nonzero_low | nonzero_high;
endmodule

module l2_priority_encoder_3(
    input wire [7:0] data_in,
    output wire [2:0] data_out,
    output wire [7:0] data_out_mask,
    output wire nonzero_out
);

wire [1:0] data_low;

wire [1:0] data_high;

wire [3:0] data_low_mask;

wire [3:0] data_high_mask;

wire nonzero_low;

wire nonzero_high;

l2_priority_encoder_2 encoder_high_2 (.data_in(data_in[7:4]), .data_out(data_high), .data_out_mask(data_high_mask), .nonzero_out(nonzero_high));

l2_priority_encoder_2 encoder_low_2(.data_in(data_in[3:0]), .data_out(data_low), .data_out_mask(data_low_mask), .nonzero_out(nonzero_low));

assign data_out = nonzero_low ? {1'b0, data_low} : {1'b1, data_high};

assign data_out_mask = nonzero_low ? {{4{1'b1}}, data_low_mask} : {data_high_mask,{4{1'b1}}};

assign nonzero_out = nonzero_low | nonzero_high;
endmodule

module l2_priority_encoder_4(
    input wire [15:0] data_in,
    output wire [3:0] data_out,
    output wire [15:0] data_out_mask,
    output wire nonzero_out
);

wire [2:0] data_low;

wire [2:0] data_high;

wire [7:0] data_low_mask;

wire [7:0] data_high_mask;

wire nonzero_low;

wire nonzero_high;

l2_priority_encoder_3 encoder_high_3 (.data_in(data_in[15:8]), .data_out(data_high), .data_out_mask(data_high_mask), .nonzero_out(nonzero_high));

l2_priority_encoder_3 encoder_low_3(.data_in(data_in[7:0]), .data_out(data_low), .data_out_mask(data_low_mask), .nonzero_out(nonzero_low));

assign data_out = nonzero_low ? {1'b0, data_low} : {1'b1, data_high};

assign data_out_mask = nonzero_low ? {{8{1'b1}}, data_low_mask} : {data_high_mask,{8{1'b1}}};

assign nonzero_out = nonzero_low | nonzero_high;
endmodule

module l2_priority_encoder_5(
    input wire [31:0] data_in,
    output wire [4:0] data_out,
    output wire [31:0] data_out_mask,
    output wire nonzero_out
);

wire [3:0] data_low;

wire [3:0] data_high;

wire [15:0] data_low_mask;

wire [15:0] data_high_mask;

wire nonzero_low;

wire nonzero_high;

l2_priority_encoder_4 encoder_high_4 (.data_in(data_in[31:16]), .data_out(data_high), .data_out_mask(data_high_mask), .nonzero_out(nonzero_high));

l2_priority_encoder_4 encoder_low_4(.data_in(data_in[15:0]), .data_out(data_low), .data_out_mask(data_low_mask), .nonzero_out(nonzero_low));

assign data_out = nonzero_low ? {1'b0, data_low} : {1'b1, data_high};

assign data_out_mask = nonzero_low ? {{16{1'b1}}, data_low_mask} : {data_high_mask,{16{1'b1}}};

assign nonzero_out = nonzero_low | nonzero_high;
endmodule

module l2_priority_encoder_6(
    input wire [63:0] data_in,
    output wire [5:0] data_out,
    output wire [63:0] data_out_mask,
    output wire nonzero_out
);

wire [4:0] data_low;

wire [4:0] data_high;

wire [31:0] data_low_mask;

wire [31:0] data_high_mask;

wire nonzero_low;

wire nonzero_high;

l2_priority_encoder_5 encoder_high_5 (.data_in(data_in[63:32]), .data_out(data_high), .data_out_mask(data_high_mask), .nonzero_out(nonzero_high));

l2_priority_encoder_5 encoder_low_5(.data_in(data_in[31:0]), .data_out(data_low), .data_out_mask(data_low_mask), .nonzero_out(nonzero_low));

assign data_out = nonzero_low ? {1'b0, data_low} : {1'b1, data_high};

assign data_out_mask = nonzero_low ? {{32{1'b1}}, data_low_mask} : {data_high_mask,{32{1'b1}}};

assign nonzero_out = nonzero_low | nonzero_high;
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
//  Filename      : l2_smc.v
//  Created On    : 2014-06-07
//  Revision      :
//  Author        : Yaosheng Fu
//  Company       : Princeton University
//  Email         : yfu@princeton.edu
//
//  Description   : The SMC in the L2 cache
//
//
//==================================================================================================

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
//  Filename      : l2.h.pyv
//  Created On    : 2014-02-20
//  Revision      :
//  Author        : Yaosheng Fu
//  Company       : Princeton University
//  Email         : yfu@princeton.edu
//
//  Description   : header file for the L2 cache
//
//
//==================================================================================================

// devices.xml



// Input buffer for pipeline1













// Input buffer for pipeline2














// Output buffer















// L2 cache configuration







//`define L2_SIZE                  65536
//`define L2_SIZE_WIDTH            16






//`define L2_LINE_SIZE             64
//`define L2_LINE_SIZE_WIDTH       6













//`define L2_WAYS                  4
//`define L2_WAYS_WIDTH            2
//`define L2_WAY_0                 2'b00
//`define L2_WAY_1                 2'b01
//`define L2_WAY_2                 2'b10
//`define L2_WAY_3                 2'b11





// Tag array







//`define L2_TAG_INDEX_WIDTH      8
//`define L2_TAG_WIDTH            26
//`define L2_TAG_WAY_WIDTH        26
//`define L2_TAG_ARRAY_WIDTH      104


// Tag Address decomposition 




//`define L2_TAG_INDEX            13:6
//`define L2_TAG                  39:14


//Data array












//`define L2_DATA_INDEX_WIDTH         12 








// Data Address decomposition 







//Dir array



//`define L2_DIR_INDEX_WIDTH      10 





//State array

















//Whether the cache line is in Icaches or Dcaches
















//Round Robin selection














// State decomposition 












//MSHR array



































//`define L2_MSHR_CMP_ADDR        13:6
//`define L2_MSHR_ADDR            39:0
//`define L2_MSHR_WAY             41:40
//`define L2_MSHR_MSHRID          49:42
//`define L2_MSHR_CACHE_TYPE      50
//`define L2_MSHR_DATA_SIZE       53:51
//`define L2_MSHR_MSG_TYPE        61:54
//`define L2_MSHR_L2_MISS         62
//`define L2_MSHR_SRC_CHIPID      76:63
//`define L2_MSHR_SRC_X           84:77
//`define L2_MSHR_SRC_Y           92:85
//`define L2_MSHR_SRC_FBITS       96:93
//`define L2_MSHR_SDID            106:97
//`define L2_MSHR_LSID            112:107      
//`define L2_MSHR_MISS_LSID       118:113
//`define L2_MSHR_SMC_MISS        119
//`define L2_MSHR_RECYCLED        120
//`define L2_MSHR_INV_FWD_PENDING 121


//SMC array


































//Message destination_type







//L2 public sharer beyond the maximum clump size



//L2 registers



//Special addresses





















//L2 core ID





//L2 dir array reuse







//Control Signal 








//AMO ALU OP macros












//Control Signal in Stage 1























































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





































































































































































































































































































































































































































































































































































































module l2_smc(

    input wire clk,
    input wire rst_n,

    //Read enable
    input wire rd_en,

    //Write enable
    input wire wr_en,

    //Diagnostic read enable
    input wire rd_diag_en,

    //Diagnostic write enable
    input wire wr_diag_en,

    //Flush enable
    input wire flush_en,

    input wire [2-1:0] addr_op,

    //address
    input wire [16-1:0] rd_addr_in,
    input wire [16-1:0] wr_addr_in,

    //Write data
    input wire [128-1:0] data_in,

    output reg hit,
    
    //Read output
    output reg [30-1:0] data_out,
    output reg [4-1:0] valid_out,
    output reg [14-1:0] tag_out
);



reg [16-1:0] entry_used_f;
reg [16-1:0] entry_used_next;
reg [16-1:0] entry_used_and_mask;
reg [16-1:0] entry_used_or_mask;
reg [16-1:0] entry_locked_f;
reg [16-1:0] entry_locked_next;
reg [16-1:0] entry_locked_and_mask;
reg [16-1:0] entry_locked_or_mask;
reg [138-1:0] data_mem_f [16-1:0];

reg [14-1:0] smc_tag [16-1:0];
reg [4-1:0] smc_valid [16-1:0];
reg [120-1:0] smc_data [16-1:0];
reg [10-1:0] smc_sdid [16-1:0];
reg [14-1:0] rd_tag_in;
reg [14-1:0] wr_tag_in;
reg [4-1:0] rd_index_in;
reg [4-1:0] wr_index_in;
reg [2-1:0] rd_offset_in;
reg [2-1:0] wr_offset_in;
reg [10-1:0] wr_sdid_in;
reg [4-1:0] smc_valid_in;
reg [120-1:0] smc_data_in;
reg [4-1:0] hit_index;
reg [4-1:0] replace_index;
reg wr_hit;
reg [4-1:0] wr_hit_index;
reg [4-1:0] wr_index;


always @ *
begin
    smc_tag[0] = data_mem_f[0][137:124];
    smc_tag[1] = data_mem_f[1][137:124];
    smc_tag[2] = data_mem_f[2][137:124];
    smc_tag[3] = data_mem_f[3][137:124];
    smc_tag[4] = data_mem_f[4][137:124];
    smc_tag[5] = data_mem_f[5][137:124];
    smc_tag[6] = data_mem_f[6][137:124];
    smc_tag[7] = data_mem_f[7][137:124];
    smc_tag[8] = data_mem_f[8][137:124];
    smc_tag[9] = data_mem_f[9][137:124];
    smc_tag[10] = data_mem_f[10][137:124];
    smc_tag[11] = data_mem_f[11][137:124];
    smc_tag[12] = data_mem_f[12][137:124];
    smc_tag[13] = data_mem_f[13][137:124];
    smc_tag[14] = data_mem_f[14][137:124];
    smc_tag[15] = data_mem_f[15][137:124];

end

always @ *
begin
    smc_valid[0] = data_mem_f[0][123:120];
    smc_valid[1] = data_mem_f[1][123:120];
    smc_valid[2] = data_mem_f[2][123:120];
    smc_valid[3] = data_mem_f[3][123:120];
    smc_valid[4] = data_mem_f[4][123:120];
    smc_valid[5] = data_mem_f[5][123:120];
    smc_valid[6] = data_mem_f[6][123:120];
    smc_valid[7] = data_mem_f[7][123:120];
    smc_valid[8] = data_mem_f[8][123:120];
    smc_valid[9] = data_mem_f[9][123:120];
    smc_valid[10] = data_mem_f[10][123:120];
    smc_valid[11] = data_mem_f[11][123:120];
    smc_valid[12] = data_mem_f[12][123:120];
    smc_valid[13] = data_mem_f[13][123:120];
    smc_valid[14] = data_mem_f[14][123:120];
    smc_valid[15] = data_mem_f[15][123:120];

end

always @ *
begin
    smc_data[0] = data_mem_f[0][119:0];
    smc_data[1] = data_mem_f[1][119:0];
    smc_data[2] = data_mem_f[2][119:0];
    smc_data[3] = data_mem_f[3][119:0];
    smc_data[4] = data_mem_f[4][119:0];
    smc_data[5] = data_mem_f[5][119:0];
    smc_data[6] = data_mem_f[6][119:0];
    smc_data[7] = data_mem_f[7][119:0];
    smc_data[8] = data_mem_f[8][119:0];
    smc_data[9] = data_mem_f[9][119:0];
    smc_data[10] = data_mem_f[10][119:0];
    smc_data[11] = data_mem_f[11][119:0];
    smc_data[12] = data_mem_f[12][119:0];
    smc_data[13] = data_mem_f[13][119:0];
    smc_data[14] = data_mem_f[14][119:0];
    smc_data[15] = data_mem_f[15][119:0];

end

always @ *
begin
    smc_sdid[0] = data_mem_f[0][137:128];
    smc_sdid[1] = data_mem_f[1][137:128];
    smc_sdid[2] = data_mem_f[2][137:128];
    smc_sdid[3] = data_mem_f[3][137:128];
    smc_sdid[4] = data_mem_f[4][137:128];
    smc_sdid[5] = data_mem_f[5][137:128];
    smc_sdid[6] = data_mem_f[6][137:128];
    smc_sdid[7] = data_mem_f[7][137:128];
    smc_sdid[8] = data_mem_f[8][137:128];
    smc_sdid[9] = data_mem_f[9][137:128];
    smc_sdid[10] = data_mem_f[10][137:128];
    smc_sdid[11] = data_mem_f[11][137:128];
    smc_sdid[12] = data_mem_f[12][137:128];
    smc_sdid[13] = data_mem_f[13][137:128];
    smc_sdid[14] = data_mem_f[14][137:128];
    smc_sdid[15] = data_mem_f[15][137:128];

end


always @ *
begin
    rd_tag_in = rd_addr_in[15:2];
    rd_offset_in = rd_addr_in[1:0];
    rd_index_in = rd_addr_in[5:2];
end

always @ *
begin
    wr_tag_in = wr_addr_in[15:2];
    wr_offset_in = wr_addr_in[1:0];
    wr_index_in = wr_addr_in[5:2];
    wr_sdid_in = wr_addr_in[15:6];
end


always @ *
begin
    smc_valid_in = { data_in[127], data_in[95], data_in[63], data_in[31] };
    smc_data_in = { data_in[125:96], data_in[93:64], data_in[61:32], data_in[29:0] };

end


wire [4-1:0] tag_hit_index;
wire tag_hit;


reg [15:0] smc_tag_cmp;

always @ *
begin

    smc_tag_cmp[0] = (smc_tag[0] == rd_tag_in) && smc_valid[0][rd_offset_in];

    smc_tag_cmp[1] = (smc_tag[1] == rd_tag_in) && smc_valid[1][rd_offset_in];

    smc_tag_cmp[2] = (smc_tag[2] == rd_tag_in) && smc_valid[2][rd_offset_in];

    smc_tag_cmp[3] = (smc_tag[3] == rd_tag_in) && smc_valid[3][rd_offset_in];

    smc_tag_cmp[4] = (smc_tag[4] == rd_tag_in) && smc_valid[4][rd_offset_in];

    smc_tag_cmp[5] = (smc_tag[5] == rd_tag_in) && smc_valid[5][rd_offset_in];

    smc_tag_cmp[6] = (smc_tag[6] == rd_tag_in) && smc_valid[6][rd_offset_in];

    smc_tag_cmp[7] = (smc_tag[7] == rd_tag_in) && smc_valid[7][rd_offset_in];

    smc_tag_cmp[8] = (smc_tag[8] == rd_tag_in) && smc_valid[8][rd_offset_in];

    smc_tag_cmp[9] = (smc_tag[9] == rd_tag_in) && smc_valid[9][rd_offset_in];

    smc_tag_cmp[10] = (smc_tag[10] == rd_tag_in) && smc_valid[10][rd_offset_in];

    smc_tag_cmp[11] = (smc_tag[11] == rd_tag_in) && smc_valid[11][rd_offset_in];

    smc_tag_cmp[12] = (smc_tag[12] == rd_tag_in) && smc_valid[12][rd_offset_in];

    smc_tag_cmp[13] = (smc_tag[13] == rd_tag_in) && smc_valid[13][rd_offset_in];

    smc_tag_cmp[14] = (smc_tag[14] == rd_tag_in) && smc_valid[14][rd_offset_in];

    smc_tag_cmp[15] = (smc_tag[15] == rd_tag_in) && smc_valid[15][rd_offset_in];

end


l2_priority_encoder_4 priority_encoder_cmp_4bits( 

    .data_in        (smc_tag_cmp),
    .data_out       (tag_hit_index),
    .data_out_mask  (),
    .nonzero_out    (tag_hit)
);



always @ *
begin
    if (rd_en && rd_diag_en)
    begin
        hit = 1'b0;
        hit_index = rd_index_in;
    end
    else
    begin
        if(rd_en)
        begin
            hit = tag_hit;
            hit_index = tag_hit_index;
        end
        else
        begin
            hit = 1'b0;
            hit_index = 0;
        end
    end
end
/*
        if(rd_en)
    begin
        if ((smc_tag[0] == rd_tag_in) && smc_valid[0][rd_offset_in])
        begin
            hit = 1'b1;
            hit_index = 4'd0;
        end
        else if ((smc_tag[1] == rd_tag_in) && smc_valid[1][rd_offset_in])
        begin
            hit = 1'b1;
            hit_index = 4'd1;
        end
        else if ((smc_tag[2] == rd_tag_in) && smc_valid[2][rd_offset_in])
        begin
            hit = 1'b1;
            hit_index = 4'd2;
        end
        else if ((smc_tag[3] == rd_tag_in) && smc_valid[3][rd_offset_in])
        begin
            hit = 1'b1;
            hit_index = 4'd3;
        end
        else if ((smc_tag[4] == rd_tag_in) && smc_valid[4][rd_offset_in])
        begin
            hit = 1'b1;
            hit_index = 4'd4;
        end
        else if ((smc_tag[5] == rd_tag_in) && smc_valid[5][rd_offset_in])
        begin
            hit = 1'b1;
            hit_index = 4'd5;
        end
        else if ((smc_tag[6] == rd_tag_in) && smc_valid[6][rd_offset_in])
        begin
            hit = 1'b1;
            hit_index = 4'd6;
        end
        else if ((smc_tag[7] == rd_tag_in) && smc_valid[7][rd_offset_in])
        begin
            hit = 1'b1;
            hit_index = 4'd7;
        end
        else if ((smc_tag[8] == rd_tag_in) && smc_valid[8][rd_offset_in])
        begin
            hit = 1'b1;
            hit_index = 4'd8;
        end
        else if ((smc_tag[9] == rd_tag_in) && smc_valid[9][rd_offset_in])
        begin
            hit = 1'b1;
            hit_index = 4'd9;
        end
        else if ((smc_tag[10] == rd_tag_in) && smc_valid[10][rd_offset_in])
        begin
            hit = 1'b1;
            hit_index = 4'd10;
        end
        else if ((smc_tag[11] == rd_tag_in) && smc_valid[11][rd_offset_in])
        begin
            hit = 1'b1;
            hit_index = 4'd11;
        end
        else if ((smc_tag[12] == rd_tag_in) && smc_valid[12][rd_offset_in])
        begin
            hit = 1'b1;
            hit_index = 4'd12;
        end
        else if ((smc_tag[13] == rd_tag_in) && smc_valid[13][rd_offset_in])
        begin
            hit = 1'b1;
            hit_index = 4'd13;
        end
        else if ((smc_tag[14] == rd_tag_in) && smc_valid[14][rd_offset_in])
        begin
            hit = 1'b1;
            hit_index = 4'd14;
        end
        else if ((smc_tag[15] == rd_tag_in) && smc_valid[15][rd_offset_in])
        begin
            hit = 1'b1;
            hit_index = 4'd15;
        end
        else
        begin
            hit = 1'b0;
            hit_index = 4'd0;
        end
    end
    else
    begin
        hit = 1'b0;
        hit_index = 4'd0;
    end

    end
end
*/


wire [4-1:0] tag_wr_hit_index;
wire tag_wr_hit;


reg [15:0] smc_tag_wr_cmp;

always @ *
begin

    smc_tag_wr_cmp[0] = (smc_tag[0] == wr_tag_in) && (smc_valid[0] != 0);

    smc_tag_wr_cmp[1] = (smc_tag[1] == wr_tag_in) && (smc_valid[1] != 0);

    smc_tag_wr_cmp[2] = (smc_tag[2] == wr_tag_in) && (smc_valid[2] != 0);

    smc_tag_wr_cmp[3] = (smc_tag[3] == wr_tag_in) && (smc_valid[3] != 0);

    smc_tag_wr_cmp[4] = (smc_tag[4] == wr_tag_in) && (smc_valid[4] != 0);

    smc_tag_wr_cmp[5] = (smc_tag[5] == wr_tag_in) && (smc_valid[5] != 0);

    smc_tag_wr_cmp[6] = (smc_tag[6] == wr_tag_in) && (smc_valid[6] != 0);

    smc_tag_wr_cmp[7] = (smc_tag[7] == wr_tag_in) && (smc_valid[7] != 0);

    smc_tag_wr_cmp[8] = (smc_tag[8] == wr_tag_in) && (smc_valid[8] != 0);

    smc_tag_wr_cmp[9] = (smc_tag[9] == wr_tag_in) && (smc_valid[9] != 0);

    smc_tag_wr_cmp[10] = (smc_tag[10] == wr_tag_in) && (smc_valid[10] != 0);

    smc_tag_wr_cmp[11] = (smc_tag[11] == wr_tag_in) && (smc_valid[11] != 0);

    smc_tag_wr_cmp[12] = (smc_tag[12] == wr_tag_in) && (smc_valid[12] != 0);

    smc_tag_wr_cmp[13] = (smc_tag[13] == wr_tag_in) && (smc_valid[13] != 0);

    smc_tag_wr_cmp[14] = (smc_tag[14] == wr_tag_in) && (smc_valid[14] != 0);

    smc_tag_wr_cmp[15] = (smc_tag[15] == wr_tag_in) && (smc_valid[15] != 0);

end



l2_priority_encoder_4 priority_encoder_wr_cmp_4bits( 

    .data_in        (smc_tag_wr_cmp),
    .data_out       (tag_wr_hit_index),
    .data_out_mask  (),
    .nonzero_out    (tag_wr_hit)
);





//avoid redundant entries
always @ *
begin
    if(wr_en || (flush_en && (addr_op == 2'd1)))
    begin
        wr_hit = tag_wr_hit;
        wr_hit_index = tag_wr_hit_index;
    end
    else
    begin
        wr_hit = 1'b0;
        wr_hit_index = 0;
    end
end


/*
always @ *
begin
    if(wr_en || (flush_en && (addr_op == 2'd1)))
    begin
        if ((smc_tag[0] == wr_tag_in) && (smc_valid[0] != 0))
        begin
            wr_hit = 1'b1;
            wr_hit_index = 4'd0;
        end
        else if ((smc_tag[1] == wr_tag_in) && (smc_valid[1] != 0))
        begin
            wr_hit = 1'b1;
            wr_hit_index = 4'd1;
        end
        else if ((smc_tag[2] == wr_tag_in) && (smc_valid[2] != 0))
        begin
            wr_hit = 1'b1;
            wr_hit_index = 4'd2;
        end
        else if ((smc_tag[3] == wr_tag_in) && (smc_valid[3] != 0))
        begin
            wr_hit = 1'b1;
            wr_hit_index = 4'd3;
        end
        else if ((smc_tag[4] == wr_tag_in) && (smc_valid[4] != 0))
        begin
            wr_hit = 1'b1;
            wr_hit_index = 4'd4;
        end
        else if ((smc_tag[5] == wr_tag_in) && (smc_valid[5] != 0))
        begin
            wr_hit = 1'b1;
            wr_hit_index = 4'd5;
        end
        else if ((smc_tag[6] == wr_tag_in) && (smc_valid[6] != 0))
        begin
            wr_hit = 1'b1;
            wr_hit_index = 4'd6;
        end
        else if ((smc_tag[7] == wr_tag_in) && (smc_valid[7] != 0))
        begin
            wr_hit = 1'b1;
            wr_hit_index = 4'd7;
        end
        else if ((smc_tag[8] == wr_tag_in) && (smc_valid[8] != 0))
        begin
            wr_hit = 1'b1;
            wr_hit_index = 4'd8;
        end
        else if ((smc_tag[9] == wr_tag_in) && (smc_valid[9] != 0))
        begin
            wr_hit = 1'b1;
            wr_hit_index = 4'd9;
        end
        else if ((smc_tag[10] == wr_tag_in) && (smc_valid[10] != 0))
        begin
            wr_hit = 1'b1;
            wr_hit_index = 4'd10;
        end
        else if ((smc_tag[11] == wr_tag_in) && (smc_valid[11] != 0))
        begin
            wr_hit = 1'b1;
            wr_hit_index = 4'd11;
        end
        else if ((smc_tag[12] == wr_tag_in) && (smc_valid[12] != 0))
        begin
            wr_hit = 1'b1;
            wr_hit_index = 4'd12;
        end
        else if ((smc_tag[13] == wr_tag_in) && (smc_valid[13] != 0))
        begin
            wr_hit = 1'b1;
            wr_hit_index = 4'd13;
        end
        else if ((smc_tag[14] == wr_tag_in) && (smc_valid[14] != 0))
        begin
            wr_hit = 1'b1;
            wr_hit_index = 4'd14;
        end
        else if ((smc_tag[15] == wr_tag_in) && (smc_valid[15] != 0))
        begin
            wr_hit = 1'b1;
            wr_hit_index = 4'd15;
        end
        else
        begin
            wr_hit = 1'b0;
            wr_hit_index = 4'd0;
        end
    end
    else
    begin
        wr_hit = 1'b0;
        wr_hit_index = 4'd0;
    end

end
*/

always @ *
begin
    data_out = smc_data[hit_index]>>(rd_offset_in * 30);
    valid_out = smc_valid[hit_index];
    tag_out = smc_tag[hit_index];
end


always @ *
begin
    entry_locked_and_mask = {16{1'b1}};
    entry_locked_or_mask = {16{1'b0}};
    if (!rst_n)
    begin
        entry_locked_and_mask = {16{1'b0}};
    end
    else if (wr_en && ~wr_diag_en)
    begin
        if(smc_valid_in)
        begin
            entry_locked_or_mask[wr_index] = 1'b1;
        end
        else
        begin
            entry_locked_and_mask[wr_index] = 1'b0;
        end
        if (rd_en && ~rd_diag_en && hit && (wr_index != hit_index) && entry_locked_f[hit_index])
        begin
            entry_locked_and_mask[hit_index] = 1'b0;
        end 
    end
    else if (rd_en && ~rd_diag_en && hit && entry_locked_f[hit_index])
    begin
        entry_locked_and_mask[hit_index] = 1'b0;
    end
end

always @ *
begin
    entry_locked_next = (entry_locked_f & entry_locked_and_mask) | entry_locked_or_mask;
end


always @ (posedge clk)
begin
    entry_locked_f <= entry_locked_next;
end


always @ *
begin
    entry_used_and_mask = {16{1'b1}};
    entry_used_or_mask = {16{1'b0}};
    if (!rst_n)
    begin
        entry_used_and_mask = {16{1'b0}};
    end
    else if (wr_en && ~wr_diag_en)
    begin
        if(smc_valid_in)
        begin
            entry_used_or_mask[wr_index] = 1'b1;
        end
        else
        begin
            entry_used_and_mask[wr_index] = 1'b0;
        end
        if (rd_en && ~rd_diag_en && hit && (wr_index != hit_index))
        begin
            entry_used_or_mask[hit_index] = 1'b1;
        end 
    end
    else if (rd_en && ~rd_diag_en && hit)
    begin
        entry_used_or_mask[hit_index] = 1'b1;
    end
end

always @ *
begin
    entry_used_next = (entry_used_f & entry_used_and_mask) | entry_used_or_mask;
    if (entry_used_next == {16{1'b1}})
    begin
        entry_used_next = {16{1'b0}};
    end
end


always @ (posedge clk)
begin
    entry_used_f <= entry_used_next;
end


wire [4-1:0] entry_replace_index;
wire replace_hit;


reg [15:0] replace_cmp;

always @ *
begin

    replace_cmp[0] = (~entry_used_f[0] && ~entry_locked_f[0]);

    replace_cmp[1] = (~entry_used_f[1] && ~entry_locked_f[1]);

    replace_cmp[2] = (~entry_used_f[2] && ~entry_locked_f[2]);

    replace_cmp[3] = (~entry_used_f[3] && ~entry_locked_f[3]);

    replace_cmp[4] = (~entry_used_f[4] && ~entry_locked_f[4]);

    replace_cmp[5] = (~entry_used_f[5] && ~entry_locked_f[5]);

    replace_cmp[6] = (~entry_used_f[6] && ~entry_locked_f[6]);

    replace_cmp[7] = (~entry_used_f[7] && ~entry_locked_f[7]);

    replace_cmp[8] = (~entry_used_f[8] && ~entry_locked_f[8]);

    replace_cmp[9] = (~entry_used_f[9] && ~entry_locked_f[9]);

    replace_cmp[10] = (~entry_used_f[10] && ~entry_locked_f[10]);

    replace_cmp[11] = (~entry_used_f[11] && ~entry_locked_f[11]);

    replace_cmp[12] = (~entry_used_f[12] && ~entry_locked_f[12]);

    replace_cmp[13] = (~entry_used_f[13] && ~entry_locked_f[13]);

    replace_cmp[14] = (~entry_used_f[14] && ~entry_locked_f[14]);

    replace_cmp[15] = (~entry_used_f[15] && ~entry_locked_f[15]);

end


l2_priority_encoder_4 priority_encoder_replace_cmp_4bits( 

    .data_in        (replace_cmp),
    .data_out       (entry_replace_index),
    .data_out_mask  (),
    .nonzero_out    (replace_hit)
);


always @ *
begin
    if (replace_hit)
    begin
        replace_index = entry_replace_index;
    end
    else
    begin
        replace_index = {4{1'b0}};
    end

end

/*
always @ *
begin
    if (~entry_used_f[0] && ~entry_locked_f[0])
    begin
        replace_index = 4'd0;
    end
    else if (~entry_used_f[1] && ~entry_locked_f[1])
    begin
        replace_index = 4'd1;
    end
    else if (~entry_used_f[2] && ~entry_locked_f[2])
    begin
        replace_index = 4'd2;
    end
    else if (~entry_used_f[3] && ~entry_locked_f[3])
    begin
        replace_index = 4'd3;
    end
    else if (~entry_used_f[4] && ~entry_locked_f[4])
    begin
        replace_index = 4'd4;
    end
    else if (~entry_used_f[5] && ~entry_locked_f[5])
    begin
        replace_index = 4'd5;
    end
    else if (~entry_used_f[6] && ~entry_locked_f[6])
    begin
        replace_index = 4'd6;
    end
    else if (~entry_used_f[7] && ~entry_locked_f[7])
    begin
        replace_index = 4'd7;
    end
    else if (~entry_used_f[8] && ~entry_locked_f[8])
    begin
        replace_index = 4'd8;
    end
    else if (~entry_used_f[9] && ~entry_locked_f[9])
    begin
        replace_index = 4'd9;
    end
    else if (~entry_used_f[10] && ~entry_locked_f[10])
    begin
        replace_index = 4'd10;
    end
    else if (~entry_used_f[11] && ~entry_locked_f[11])
    begin
        replace_index = 4'd11;
    end
    else if (~entry_used_f[12] && ~entry_locked_f[12])
    begin
        replace_index = 4'd12;
    end
    else if (~entry_used_f[13] && ~entry_locked_f[13])
    begin
        replace_index = 4'd13;
    end
    else if (~entry_used_f[14] && ~entry_locked_f[14])
    begin
        replace_index = 4'd14;
    end
    else if (~entry_used_f[15] && ~entry_locked_f[15])
    begin
        replace_index = 4'd15;
    end
    else
    begin
        replace_index = 4'dx;
    end

end
*/

always @ *
begin
    if (wr_en && wr_diag_en)
    begin
        wr_index = wr_index_in;
    end
    else if ((flush_en || wr_en) && wr_hit)
    begin
        wr_index = wr_hit_index;
    end
    else
    begin
        wr_index = replace_index;
    end
end


always @ (posedge clk)
begin
    if (!rst_n)
    begin
        data_mem_f[0] <= {138{1'b0}};
        data_mem_f[1] <= {138{1'b0}};
        data_mem_f[2] <= {138{1'b0}};
        data_mem_f[3] <= {138{1'b0}};
        data_mem_f[4] <= {138{1'b0}};
        data_mem_f[5] <= {138{1'b0}};
        data_mem_f[6] <= {138{1'b0}};
        data_mem_f[7] <= {138{1'b0}};
        data_mem_f[8] <= {138{1'b0}};
        data_mem_f[9] <= {138{1'b0}};
        data_mem_f[10] <= {138{1'b0}};
        data_mem_f[11] <= {138{1'b0}};
        data_mem_f[12] <= {138{1'b0}};
        data_mem_f[13] <= {138{1'b0}};
        data_mem_f[14] <= {138{1'b0}};
        data_mem_f[15] <= {138{1'b0}};

    end
    else if (flush_en)
    begin
        case (addr_op)
        2'd0:
        begin
            data_mem_f[0][123:120] <= {4{1'b0}};
            data_mem_f[1][123:120] <= {4{1'b0}};
            data_mem_f[2][123:120] <= {4{1'b0}};
            data_mem_f[3][123:120] <= {4{1'b0}};
            data_mem_f[4][123:120] <= {4{1'b0}};
            data_mem_f[5][123:120] <= {4{1'b0}};
            data_mem_f[6][123:120] <= {4{1'b0}};
            data_mem_f[7][123:120] <= {4{1'b0}};
            data_mem_f[8][123:120] <= {4{1'b0}};
            data_mem_f[9][123:120] <= {4{1'b0}};
            data_mem_f[10][123:120] <= {4{1'b0}};
            data_mem_f[11][123:120] <= {4{1'b0}};
            data_mem_f[12][123:120] <= {4{1'b0}};
            data_mem_f[13][123:120] <= {4{1'b0}};
            data_mem_f[14][123:120] <= {4{1'b0}};
            data_mem_f[15][123:120] <= {4{1'b0}};

        end
        2'd1:
        begin
            if (wr_hit)
            begin
                data_mem_f[wr_index][120+wr_offset_in] <= 1'b0;
                
            end
        end
        2'd2:
        begin
            if ((smc_sdid[0] == wr_sdid_in) && (smc_valid[0] != 0))
                data_mem_f[0][123:120] <= {4{1'b0}};
            if ((smc_sdid[1] == wr_sdid_in) && (smc_valid[1] != 0))
                data_mem_f[1][123:120] <= {4{1'b0}};
            if ((smc_sdid[2] == wr_sdid_in) && (smc_valid[2] != 0))
                data_mem_f[2][123:120] <= {4{1'b0}};
            if ((smc_sdid[3] == wr_sdid_in) && (smc_valid[3] != 0))
                data_mem_f[3][123:120] <= {4{1'b0}};
            if ((smc_sdid[4] == wr_sdid_in) && (smc_valid[4] != 0))
                data_mem_f[4][123:120] <= {4{1'b0}};
            if ((smc_sdid[5] == wr_sdid_in) && (smc_valid[5] != 0))
                data_mem_f[5][123:120] <= {4{1'b0}};
            if ((smc_sdid[6] == wr_sdid_in) && (smc_valid[6] != 0))
                data_mem_f[6][123:120] <= {4{1'b0}};
            if ((smc_sdid[7] == wr_sdid_in) && (smc_valid[7] != 0))
                data_mem_f[7][123:120] <= {4{1'b0}};
            if ((smc_sdid[8] == wr_sdid_in) && (smc_valid[8] != 0))
                data_mem_f[8][123:120] <= {4{1'b0}};
            if ((smc_sdid[9] == wr_sdid_in) && (smc_valid[9] != 0))
                data_mem_f[9][123:120] <= {4{1'b0}};
            if ((smc_sdid[10] == wr_sdid_in) && (smc_valid[10] != 0))
                data_mem_f[10][123:120] <= {4{1'b0}};
            if ((smc_sdid[11] == wr_sdid_in) && (smc_valid[11] != 0))
                data_mem_f[11][123:120] <= {4{1'b0}};
            if ((smc_sdid[12] == wr_sdid_in) && (smc_valid[12] != 0))
                data_mem_f[12][123:120] <= {4{1'b0}};
            if ((smc_sdid[13] == wr_sdid_in) && (smc_valid[13] != 0))
                data_mem_f[13][123:120] <= {4{1'b0}};
            if ((smc_sdid[14] == wr_sdid_in) && (smc_valid[14] != 0))
                data_mem_f[14][123:120] <= {4{1'b0}};
            if ((smc_sdid[15] == wr_sdid_in) && (smc_valid[15] != 0))
                data_mem_f[15][123:120] <= {4{1'b0}};

        end
        default:
        begin
            data_mem_f[wr_index] <= data_mem_f[wr_index];
        end
        endcase
    end
    else if (wr_en)
    begin
        if (wr_diag_en)
        begin
            case (addr_op)
            2'd0:
            begin
                case (wr_offset_in)
                2'd0:
                begin
                    data_mem_f[wr_index][30-1:0] <= 
                    data_in[30-1:0];
                end
                2'd1:
                begin
                    data_mem_f[wr_index][30*2-1:30] <= 
                    data_in[30-1:0];
                end
                2'd2:
                begin
                    data_mem_f[wr_index][30*3-1:30*2] <= 
                    data_in[30-1:0];
                end
                2'd3:
                begin
                    data_mem_f[wr_index][30*4-1:30*3] <= 
                    data_in[30-1:0];
                end
                default:
                begin
                    data_mem_f[wr_index] <= data_mem_f[wr_index];
                end
                endcase
            end
            2'd1:
            begin
                data_mem_f[wr_index][123:120] <= data_in[4-1:0];
            end
            2'd2: 
            begin
                data_mem_f[wr_index][137:124] <= data_in[14-1:0];
            end
            default:
            begin
                data_mem_f[wr_index] <= data_mem_f[wr_index];
            end
            endcase
        end
        else
        begin
            data_mem_f[wr_index] <= {wr_tag_in, smc_valid_in, smc_data_in};
        end
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

//==================================================================================================
//  Filename      : l2_state.v
//  Created On    : 2014-02-24
//  Revision      :
//  Author        : Yaosheng Fu
//  Company       : Princeton University
//  Email         : yfu@princeton.edu
//
//  Description   : The state array in the L2 cache
//
//
//==================================================================================================

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
//  Filename      : l2.h.pyv
//  Created On    : 2014-02-20
//  Revision      :
//  Author        : Yaosheng Fu
//  Company       : Princeton University
//  Email         : yfu@princeton.edu
//
//  Description   : header file for the L2 cache
//
//
//==================================================================================================

// devices.xml



// Input buffer for pipeline1













// Input buffer for pipeline2














// Output buffer















// L2 cache configuration







//`define L2_SIZE                  65536
//`define L2_SIZE_WIDTH            16






//`define L2_LINE_SIZE             64
//`define L2_LINE_SIZE_WIDTH       6













//`define L2_WAYS                  4
//`define L2_WAYS_WIDTH            2
//`define L2_WAY_0                 2'b00
//`define L2_WAY_1                 2'b01
//`define L2_WAY_2                 2'b10
//`define L2_WAY_3                 2'b11





// Tag array







//`define L2_TAG_INDEX_WIDTH      8
//`define L2_TAG_WIDTH            26
//`define L2_TAG_WAY_WIDTH        26
//`define L2_TAG_ARRAY_WIDTH      104


// Tag Address decomposition 




//`define L2_TAG_INDEX            13:6
//`define L2_TAG                  39:14


//Data array












//`define L2_DATA_INDEX_WIDTH         12 








// Data Address decomposition 







//Dir array



//`define L2_DIR_INDEX_WIDTH      10 





//State array

















//Whether the cache line is in Icaches or Dcaches
















//Round Robin selection














// State decomposition 












//MSHR array



































//`define L2_MSHR_CMP_ADDR        13:6
//`define L2_MSHR_ADDR            39:0
//`define L2_MSHR_WAY             41:40
//`define L2_MSHR_MSHRID          49:42
//`define L2_MSHR_CACHE_TYPE      50
//`define L2_MSHR_DATA_SIZE       53:51
//`define L2_MSHR_MSG_TYPE        61:54
//`define L2_MSHR_L2_MISS         62
//`define L2_MSHR_SRC_CHIPID      76:63
//`define L2_MSHR_SRC_X           84:77
//`define L2_MSHR_SRC_Y           92:85
//`define L2_MSHR_SRC_FBITS       96:93
//`define L2_MSHR_SDID            106:97
//`define L2_MSHR_LSID            112:107      
//`define L2_MSHR_MISS_LSID       118:113
//`define L2_MSHR_SMC_MISS        119
//`define L2_MSHR_RECYCLED        120
//`define L2_MSHR_INV_FWD_PENDING 121


//SMC array


































//Message destination_type







//L2 public sharer beyond the maximum clump size



//L2 registers



//Special addresses





















//L2 core ID





//L2 dir array reuse







//Control Signal 








//AMO ALU OP macros












//Control Signal in Stage 1























































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





































































































































































































































































































































































































































































































































































































module l2_state(


    input wire clk,
    input wire rst_n,
    input wire pdout_en,
    input wire deepsleep,

    input wire rd_en,
    input wire wr_en,
    input wire [8-1:0] rd_addr,
    input wire [8-1:0] wr_addr,
    input wire [15*4+2+4-1:0] data_in,
    input wire [15*4+2+4-1:0] data_mask_in,

    output reg [15*4+2+4-1:0] data_out,
    output wire [15*4+2+4-1:0] pdata_out,

    // sram interface
    output wire [4-1:0] srams_rtap_data,
    input wire  [4-1:0] rtap_srams_bist_command,
    input wire  [4-1:0] rtap_srams_bist_data

);




//Need to bypass the read data if both read and write are valid for the same index in the same cycle


reg [15*4+2+4-1:0] data_in_buf;
reg [15*4+2+4-1:0] data_mask_in_buf;
wire [15*4+2+4-1:0] data_out_real;

always @ (posedge clk)
begin
    data_in_buf <= data_in;
    data_mask_in_buf <= data_mask_in;
end

reg bypass_f;
reg bypass_next;

always @ *
begin
    if (rd_en && wr_en && (rd_addr == wr_addr))
    begin
        bypass_next = 1'b1;
    end
    else
    begin
        bypass_next = 1'b0;
    end
end


always @ (posedge clk)
begin
    bypass_f <= bypass_next;
end

always @ *
begin
    if (bypass_f)
    begin

        data_out[0] = data_mask_in_buf[0] ? data_in_buf[0] : data_out_real[0];
    

        data_out[1] = data_mask_in_buf[1] ? data_in_buf[1] : data_out_real[1];
    

        data_out[2] = data_mask_in_buf[2] ? data_in_buf[2] : data_out_real[2];
    

        data_out[3] = data_mask_in_buf[3] ? data_in_buf[3] : data_out_real[3];
    

        data_out[4] = data_mask_in_buf[4] ? data_in_buf[4] : data_out_real[4];
    

        data_out[5] = data_mask_in_buf[5] ? data_in_buf[5] : data_out_real[5];
    

        data_out[6] = data_mask_in_buf[6] ? data_in_buf[6] : data_out_real[6];
    

        data_out[7] = data_mask_in_buf[7] ? data_in_buf[7] : data_out_real[7];
    

        data_out[8] = data_mask_in_buf[8] ? data_in_buf[8] : data_out_real[8];
    

        data_out[9] = data_mask_in_buf[9] ? data_in_buf[9] : data_out_real[9];
    

        data_out[10] = data_mask_in_buf[10] ? data_in_buf[10] : data_out_real[10];
    

        data_out[11] = data_mask_in_buf[11] ? data_in_buf[11] : data_out_real[11];
    

        data_out[12] = data_mask_in_buf[12] ? data_in_buf[12] : data_out_real[12];
    

        data_out[13] = data_mask_in_buf[13] ? data_in_buf[13] : data_out_real[13];
    

        data_out[14] = data_mask_in_buf[14] ? data_in_buf[14] : data_out_real[14];
    

        data_out[15] = data_mask_in_buf[15] ? data_in_buf[15] : data_out_real[15];
    

        data_out[16] = data_mask_in_buf[16] ? data_in_buf[16] : data_out_real[16];
    

        data_out[17] = data_mask_in_buf[17] ? data_in_buf[17] : data_out_real[17];
    

        data_out[18] = data_mask_in_buf[18] ? data_in_buf[18] : data_out_real[18];
    

        data_out[19] = data_mask_in_buf[19] ? data_in_buf[19] : data_out_real[19];
    

        data_out[20] = data_mask_in_buf[20] ? data_in_buf[20] : data_out_real[20];
    

        data_out[21] = data_mask_in_buf[21] ? data_in_buf[21] : data_out_real[21];
    

        data_out[22] = data_mask_in_buf[22] ? data_in_buf[22] : data_out_real[22];
    

        data_out[23] = data_mask_in_buf[23] ? data_in_buf[23] : data_out_real[23];
    

        data_out[24] = data_mask_in_buf[24] ? data_in_buf[24] : data_out_real[24];
    

        data_out[25] = data_mask_in_buf[25] ? data_in_buf[25] : data_out_real[25];
    

        data_out[26] = data_mask_in_buf[26] ? data_in_buf[26] : data_out_real[26];
    

        data_out[27] = data_mask_in_buf[27] ? data_in_buf[27] : data_out_real[27];
    

        data_out[28] = data_mask_in_buf[28] ? data_in_buf[28] : data_out_real[28];
    

        data_out[29] = data_mask_in_buf[29] ? data_in_buf[29] : data_out_real[29];
    

        data_out[30] = data_mask_in_buf[30] ? data_in_buf[30] : data_out_real[30];
    

        data_out[31] = data_mask_in_buf[31] ? data_in_buf[31] : data_out_real[31];
    

        data_out[32] = data_mask_in_buf[32] ? data_in_buf[32] : data_out_real[32];
    

        data_out[33] = data_mask_in_buf[33] ? data_in_buf[33] : data_out_real[33];
    

        data_out[34] = data_mask_in_buf[34] ? data_in_buf[34] : data_out_real[34];
    

        data_out[35] = data_mask_in_buf[35] ? data_in_buf[35] : data_out_real[35];
    

        data_out[36] = data_mask_in_buf[36] ? data_in_buf[36] : data_out_real[36];
    

        data_out[37] = data_mask_in_buf[37] ? data_in_buf[37] : data_out_real[37];
    

        data_out[38] = data_mask_in_buf[38] ? data_in_buf[38] : data_out_real[38];
    

        data_out[39] = data_mask_in_buf[39] ? data_in_buf[39] : data_out_real[39];
    

        data_out[40] = data_mask_in_buf[40] ? data_in_buf[40] : data_out_real[40];
    

        data_out[41] = data_mask_in_buf[41] ? data_in_buf[41] : data_out_real[41];
    

        data_out[42] = data_mask_in_buf[42] ? data_in_buf[42] : data_out_real[42];
    

        data_out[43] = data_mask_in_buf[43] ? data_in_buf[43] : data_out_real[43];
    

        data_out[44] = data_mask_in_buf[44] ? data_in_buf[44] : data_out_real[44];
    

        data_out[45] = data_mask_in_buf[45] ? data_in_buf[45] : data_out_real[45];
    

        data_out[46] = data_mask_in_buf[46] ? data_in_buf[46] : data_out_real[46];
    

        data_out[47] = data_mask_in_buf[47] ? data_in_buf[47] : data_out_real[47];
    

        data_out[48] = data_mask_in_buf[48] ? data_in_buf[48] : data_out_real[48];
    

        data_out[49] = data_mask_in_buf[49] ? data_in_buf[49] : data_out_real[49];
    

        data_out[50] = data_mask_in_buf[50] ? data_in_buf[50] : data_out_real[50];
    

        data_out[51] = data_mask_in_buf[51] ? data_in_buf[51] : data_out_real[51];
    

        data_out[52] = data_mask_in_buf[52] ? data_in_buf[52] : data_out_real[52];
    

        data_out[53] = data_mask_in_buf[53] ? data_in_buf[53] : data_out_real[53];
    

        data_out[54] = data_mask_in_buf[54] ? data_in_buf[54] : data_out_real[54];
    

        data_out[55] = data_mask_in_buf[55] ? data_in_buf[55] : data_out_real[55];
    

        data_out[56] = data_mask_in_buf[56] ? data_in_buf[56] : data_out_real[56];
    

        data_out[57] = data_mask_in_buf[57] ? data_in_buf[57] : data_out_real[57];
    

        data_out[58] = data_mask_in_buf[58] ? data_in_buf[58] : data_out_real[58];
    

        data_out[59] = data_mask_in_buf[59] ? data_in_buf[59] : data_out_real[59];
    

        data_out[60] = data_mask_in_buf[60] ? data_in_buf[60] : data_out_real[60];
    

        data_out[61] = data_mask_in_buf[61] ? data_in_buf[61] : data_out_real[61];
    

        data_out[62] = data_mask_in_buf[62] ? data_in_buf[62] : data_out_real[62];
    

        data_out[63] = data_mask_in_buf[63] ? data_in_buf[63] : data_out_real[63];
    

        data_out[64] = data_mask_in_buf[64] ? data_in_buf[64] : data_out_real[64];
    

        data_out[65] = data_mask_in_buf[65] ? data_in_buf[65] : data_out_real[65];
    

    end
    else
    begin
        data_out = data_out_real;
    end
end

 // sram_2rw_256x66 l2_state_array (
 sram_l2_state l2_state_array (
     .RESET_N(rst_n),
     .MEMCLK         (clk),

     .CEA            (rd_en),
     .RDWENA          (1'b1),
     .AA             (rd_addr),
     .BWA             (),
     .DINA            (),
     .DOUTA           (data_out_real),

     .CEB            (wr_en),
     .RDWENB            (1'b0),
     .AB             (wr_addr),
     .BWB             (data_mask_in),
     .DINB            (data_in),
     .DOUTB           (),

    .BIST_COMMAND(rtap_srams_bist_command),
    .BIST_DIN(rtap_srams_bist_data),
    .BIST_DOUT(srams_rtap_data),
    .SRAMID(8'd15)

 );





endmodule
