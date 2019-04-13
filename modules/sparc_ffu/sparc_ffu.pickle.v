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

//==================================================================================================
//  Filename      : clk_gating_latch.v
//  Created On    : 2015-01-26 14:10:43
//  Last Modified : 2015-01-26 15:13:40
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

wire clk_en_sync;
reg clk_en_sync_latch;

assign clk_out = clk & clk_en_sync_latch;

synchronizer sync(
    .clk            (clk),
    .presyncdata    (clk_en),
    .syncdata       (clk_en_sync)
);

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

	q[SIZE-1:0]  <= (se) ? si[SIZE-1:0]  : din[SIZE-1:0] ;

assign so[SIZE-1:0] = q[SIZE-1:0] ;



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






// Scan-Enable dominates
always @ (posedge clk)

	q[SIZE-1:0]  <= se ? si[SIZE-1:0] : ((rst) ? {SIZE{1'b0}}  : din[SIZE-1:0] );

assign so[SIZE-1:0] = q[SIZE-1:0] ;


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






// Reset dominates
always @ (posedge clk)

	q[SIZE-1:0]  <= rst_l ? ((se) ? si[SIZE-1:0]  : din[SIZE-1:0] ) : {SIZE{1'b0}};

assign so[SIZE-1:0] = q[SIZE-1:0] ;


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

	q[SIZE-1:0]  <= (se) ? si[SIZE-1:0]  : ((en) ? din[SIZE-1:0] : q[SIZE-1:0]) ;

assign so[SIZE-1:0] = q[SIZE-1:0] ;


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

//	q[SIZE-1:0]  <= rst ? {SIZE{1'b0}} : ((se) ? si[SIZE-1:0]  : ((en) ? din[SIZE-1:0] : q[SIZE-1:0])) ;
	q[SIZE-1:0]  <= se ? si[SIZE-1:0]  : (rst ? {SIZE{1'b0}} : ((en) ? din[SIZE-1:0] : q[SIZE-1:0])) ;

assign so[SIZE-1:0] = q[SIZE-1:0] ;



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

//	q[SIZE-1:0]  <= rst_l ? ((se) ? si[SIZE-1:0]  : ((en) ? din[SIZE-1:0] : q[SIZE-1:0])) : {SIZE{1'b0}} ;
	q[SIZE-1:0]  <= se ? si[SIZE-1:0]  : (rst_l ? ((en) ? din[SIZE-1:0] : q[SIZE-1:0]) : {SIZE{1'b0}}) ;

assign so[SIZE-1:0] = q[SIZE-1:0] ;


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






// Reset dominates
always @ (posedge clk or posedge rst)
  q[SIZE-1:0]  <= rst ? {SIZE{1'b0}} : ((se) ? si[SIZE-1:0]  : din[SIZE-1:0] );

assign so[SIZE-1:0] = q[SIZE-1:0] ;



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






// Reset dominates
always @ (posedge clk or negedge rst_l)
  q[SIZE-1:0]  <= (!rst_l) ? {SIZE{1'b0}} : ((se) ? si[SIZE-1:0]  : din[SIZE-1:0] );

assign so[SIZE-1:0] = q[SIZE-1:0] ;



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







































































































































































































































































































































































































































































































































































































module sram_1rw_128x78
(
input wire MEMCLK,
input wire RESET_N,
input wire CE,
input wire [7-1:0] A,
input wire RDWEN,
input wire [78-1:0] BW,
input wire [78-1:0] DIN,
output wire [78-1:0] DOUT,
input wire [4-1:0] BIST_COMMAND,
input wire [4-1:0] BIST_DIN,
output reg [4-1:0] BIST_DOUT,
input wire [8-1:0] SRAMID
);


wire [78-1:0] DOUT_bram;
assign DOUT = DOUT_bram;

bram_1rw_wrapper #(
   .NAME          (""             ),
   .DEPTH         (128),
   .ADDR_WIDTH    (7),
   .BITMASK_WIDTH (78),
   .DATA_WIDTH    (78)
)   sram_1rw_128x78 (
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
// Author: Tri Nguyen

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







module sram_l1d_data_piton
(
input wire MEMCLK,
input wire RESET_N,
input wire CE,
input wire [6:0] A,
input wire RDWEN,
input wire [287:0] BW,
input wire [287:0] DIN,
output wire [287:0] DOUT,
input wire [4-1:0] BIST_COMMAND,
input wire [4-1:0] BIST_DIN,
output reg [4-1:0] BIST_DOUT,
input wire [8-1:0] SRAMID
);
reg [287:0] cache [(256/2)-1:0];

integer i;
initial
begin
   for (i = 0; i < (256/2); i = i + 1)
   begin
      cache[i] = 0;
   end
end



   reg [287:0] dout_f;

   assign DOUT = dout_f;

   always @ (posedge MEMCLK)
   begin
      if (CE)
      begin
         if (RDWEN == 1'b0)
            cache[A] <= (DIN & BW) | (cache[A] & ~BW);
         else
            dout_f <= cache[A];
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

// 02/06/2015 14:58:59
// This file is auto-generated
// Author: Tri Nguyen

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







































































































































































































































































































































































































































































































































































































module sram_l1d_tag
(
input wire MEMCLK,
input wire RESET_N,
input wire CE,
input wire [(6+1)-1:0] A,
input wire RDWEN,
input wire [(33*2)-1:0] BW,
input wire [(33*2)-1:0] DIN,
output wire [(33*2)-1:0] DOUT,
input wire [4-1:0] BIST_COMMAND,
input wire [4-1:0] BIST_DIN,
output reg [4-1:0] BIST_DOUT,
input wire [8-1:0] SRAMID
);


wire [(33*2)-1:0] DOUT_bram;
assign DOUT = DOUT_bram;

bram_1rw_wrapper #(
   .NAME          (""             ),
   .DEPTH         ((256/2)),
   .ADDR_WIDTH    ((6+1)),
   .BITMASK_WIDTH ((33*2)),
   .DATA_WIDTH    ((33*2))
)   sram_l1d_tag (
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

// Modified by Princeton University on June 9th, 2015
/*
* ========== Copyright Header Begin ==========================================
* 
* OpenSPARC T1 Processor File: ifu.h
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
////////////////////////////////////////////////////////////////////////
/*
//
//  Module Name: ifu.h
//  Description:	
//  All ifu defines
*/

//--------------------------------------------
// Icache Values in IFU::ICD/ICV/ICT/FDP/IFQDP
//--------------------------------------------

// devices.xml








// `IC_WAY_MASK

// Set Values
// !!IMPORTANT!! a change to IC_LINE_SZ will mean a change to the code as
//   well.  Unfortunately this has not been properly parametrized.
//   Changing the IC_LINE_SZ param alone is *not* enough.
// `define IC_LINE_SZ  32



// !!IMPORTANT!! a change to IC_TAG_HI will mean a change to the code as
//   well.  Changing the IC_TAG_HI param alone is *not* enough to
//   change the PA range. 
// highest bit of PA


// Derived Values
// IC_IDX_HI = log(icache_size/4ways) - 1
// 11


// 4095
// `define IC_ARR_HI (`IC_SZ/`IC_NUM_WAY - 1)

// number of entries - 1 = 511

// 128 - 1

// 32


// 12


// 28

// `define IC_TAG_MASK_ALL ((`IC_TLB_TAG_SZ * `IC_NUM_WAY)-1):0


// 4

// `define IC_PARITY_PADDING (32 - `IC_TAG_SZ - 1)




// 7


// tags for all 4 ways + parity
// 116
// `define IC_TAG_ALL   ((`IC_TAG_SZ * `IC_NUM_WAY) + 4)

// 115
// `define IC_TAG_ALL_HI   ((`IC_TAG_SZ * `IC_NUM_WAY) + 3)

// physical implementation defines














// TLB
// `define IC_TLB_TAG_SZ 30
// tag + 1 bit parity



























//----------------------------------------------------------------------
// For thread scheduler in IFU::DTU::SWL
//----------------------------------------------------------------------
// thread states:  (thr_state[4:0])









// thread configuration register bit fields







//----------------------------------------------------------------------
// For MIL fsm in IFU::IFQ
//----------------------------------------------------------------------











//---------------------------------------------------
// Interrupt Block
//---------------------------------------------------







//-------------------------------------
// IFQ
//-------------------------------------
// valid bit plus ifill













//`ifdef SPARC_L2_64B


//`else
//`define BANK_ID_HI 8
//`define BANK_ID_LO 7
//`endif

//`define CPX_INV_PA_HI  116
//`define CPX_INV_PA_LO  112







//----------------------------------------
// IFU Traps
//----------------------------------------
// precise















// disrupting













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







































































































































































































































































































































































































































































































































































































module sram_l1i_data
(
input wire MEMCLK,
input wire RESET_N,
input wire CE,
input wire [6+2-1:0] A,
input wire RDWEN,
input wire [272-1:0] BW,
input wire [272-1:0] DIN,
output wire [272-1:0] DOUT,
input wire [4-1:0] BIST_COMMAND,
input wire [4-1:0] BIST_DIN,
output reg [4-1:0] BIST_DOUT,
input wire [8-1:0] SRAMID
);


wire [272-1:0] DOUT_bram;
assign DOUT = DOUT_bram;

bram_1rw_wrapper #(
   .NAME          (""             ),
   .DEPTH         (((8192/32/2))*2),
   .ADDR_WIDTH    (6+2),
   .BITMASK_WIDTH (272),
   .DATA_WIDTH    (272)
)   sram_l1i_data (
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

// Modified by Princeton University on June 9th, 2015
/*
* ========== Copyright Header Begin ==========================================
* 
* OpenSPARC T1 Processor File: ifu.h
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
////////////////////////////////////////////////////////////////////////
/*
//
//  Module Name: ifu.h
//  Description:	
//  All ifu defines
*/

//--------------------------------------------
// Icache Values in IFU::ICD/ICV/ICT/FDP/IFQDP
//--------------------------------------------

// devices.xml








// `IC_WAY_MASK

// Set Values
// !!IMPORTANT!! a change to IC_LINE_SZ will mean a change to the code as
//   well.  Unfortunately this has not been properly parametrized.
//   Changing the IC_LINE_SZ param alone is *not* enough.
// `define IC_LINE_SZ  32



// !!IMPORTANT!! a change to IC_TAG_HI will mean a change to the code as
//   well.  Changing the IC_TAG_HI param alone is *not* enough to
//   change the PA range. 
// highest bit of PA


// Derived Values
// IC_IDX_HI = log(icache_size/4ways) - 1
// 11


// 4095
// `define IC_ARR_HI (`IC_SZ/`IC_NUM_WAY - 1)

// number of entries - 1 = 511

// 128 - 1

// 32


// 12


// 28

// `define IC_TAG_MASK_ALL ((`IC_TLB_TAG_SZ * `IC_NUM_WAY)-1):0


// 4

// `define IC_PARITY_PADDING (32 - `IC_TAG_SZ - 1)




// 7


// tags for all 4 ways + parity
// 116
// `define IC_TAG_ALL   ((`IC_TAG_SZ * `IC_NUM_WAY) + 4)

// 115
// `define IC_TAG_ALL_HI   ((`IC_TAG_SZ * `IC_NUM_WAY) + 3)

// physical implementation defines














// TLB
// `define IC_TLB_TAG_SZ 30
// tag + 1 bit parity



























//----------------------------------------------------------------------
// For thread scheduler in IFU::DTU::SWL
//----------------------------------------------------------------------
// thread states:  (thr_state[4:0])









// thread configuration register bit fields







//----------------------------------------------------------------------
// For MIL fsm in IFU::IFQ
//----------------------------------------------------------------------











//---------------------------------------------------
// Interrupt Block
//---------------------------------------------------







//-------------------------------------
// IFQ
//-------------------------------------
// valid bit plus ifill













//`ifdef SPARC_L2_64B


//`else
//`define BANK_ID_HI 8
//`define BANK_ID_LO 7
//`endif

//`define CPX_INV_PA_HI  116
//`define CPX_INV_PA_LO  112







//----------------------------------------
// IFU Traps
//----------------------------------------
// precise















// disrupting













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







































































































































































































































































































































































































































































































































































































module sram_l1i_tag
(
input wire MEMCLK,
input wire RESET_N,
input wire CE,
input wire [6+1-1:0] A,
input wire RDWEN,
input wire [132-1:0] BW,
input wire [132-1:0] DIN,
output wire [132-1:0] DOUT,
input wire [4-1:0] BIST_COMMAND,
input wire [4-1:0] BIST_DIN,
output reg [4-1:0] BIST_DOUT,
input wire [8-1:0] SRAMID
);


wire [132-1:0] DOUT_bram;
assign DOUT = DOUT_bram;

bram_1rw_wrapper #(
   .NAME          (""             ),
   .DEPTH         (((8192/32/2))),
   .ADDR_WIDTH    (6+1),
   .BITMASK_WIDTH (132),
   .DATA_WIDTH    (132)
)   sram_l1i_tag (
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

// Modified by Princeton University on June 9th, 2015
// ========== Copyright Header Begin ==========================================
//
// OpenSPARC T1 Processor File: bw_r_frf.v
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
//  Module Name: bw_r_frf
// Description: This is the floating point register file.  It has one R/W port that is
//     78 bits (64 bits data, 14 bits ecc) wide.
*/


// trin 4/18/16: always use nowrapper implementation
//  all others will fail floating point tests



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





































































































































































































































































































































































































































































































































































































module bw_r_frf (/*AUTOARG*/
   // Outputs
   so, frf_dp_data,
   // Inputs
   rclk, si, se, sehold, rst_tri_en, ctl_frf_wen, ctl_frf_ren,
   dp_frf_data, ctl_frf_addr,


   // sram wrapper interface
   srams_rtap_data,
   rtap_srams_bist_command,
   rtap_srams_bist_data

   ) ;
   input rclk;
   input si;
   input se;
   input sehold;
   input rst_tri_en;
   input [1:0] ctl_frf_wen;
   input ctl_frf_ren;
   input [77:0] dp_frf_data;
   input [6:0]   ctl_frf_addr;

   output so;
   output [77:0] frf_dp_data;

   wire [7:0]    regfile_index;
  //  wire [7:0]   regfile_index_low;
    // wire [7:0] regfile_index_high;
   //
   reg [77:0] frf_dp_data_f;

   assign frf_dp_data = frf_dp_data_f;

  output [4-1:0] srams_rtap_data;
  input  [4-1:0] rtap_srams_bist_command;
  input  [4-1:0] rtap_srams_bist_data;

   // reg            rst_tri_en_negedge;
   // wire           ren_d1;
   // wire [6:0]     addr_d1;
   // wire [1:0]     wen_d1;
   // wire [77:0]    write_data_d1;
   // wire [77:0]    sehold_write_data;
   // wire [9:0]     sehold_cntl_data;

   // wire [9:0]     cntl_scan_data;
   // wire [38:0]    write_scan_data_hi;
   // wire [38:0]    write_scan_data_lo;
   // wire [38:0]    read_scan_data_hi;
   // wire [38:0]    read_scan_data_lo;

   // wire           real_se;
   // assign         real_se = se & ~sehold;

   // This is for sas comparisons
   assign        regfile_index[7:0] = {ctl_frf_addr[6:0], 1'b0};

   // assign        regfile_index_low[7:0] = {addr_d1[6:0], 1'b0};
   // assign        regfile_index_high[7:0] = {addr_d1[6:0], 1'b1};

   // assign         sehold_write_data[77:0] = (sehold)? write_data_d1[77:0]: dp_frf_data[77:0];
   // assign sehold_cntl_data[9:0] = (sehold)? {addr_d1[6:0],wen_d1[1:0], ren_d1}:
   //                                          {ctl_frf_addr[6:0],ctl_frf_wen[1:0],ctl_frf_ren};
   // All inputs go through flop
   // dff_s #(39) datain_dff1(.din(sehold_write_data[77:39]), .clk(rclk), .q(write_data_d1[77:39]),
   //                       .se(real_se), .si({cntl_scan_data[0],write_scan_data_lo[38:1]}),
   //                       .so(write_scan_data_hi[38:0]));
   // dff_s #(39) datain_dff2(.din(sehold_write_data[38:0]), .clk(rclk), .q(write_data_d1[38:0]),
   //                       .se(real_se), .si(write_scan_data_hi[38:0]), .so(write_scan_data_lo[38:0]));
   // dff_s #(10) controlin_dff(.din(sehold_cntl_data[9:0]),
   //                         .q({addr_d1[6:0],wen_d1[1:0],ren_d1}),
   //                         .clk(rclk), .se(real_se), .si({si,cntl_scan_data[9:1]}), .so(cntl_scan_data[9:0]));


   wire [77:0] write_mask = {{39{ctl_frf_wen[1]}},{39{ctl_frf_wen[0]}}};
   wire ctl_frf_wens = ctl_frf_wen[1] | ctl_frf_wen[0];

   wire [77:0] read_data;
sram_1rw_128x78 regfile
//sram_configurable_frf regfile
(
  .MEMCLK(rclk),
  .RESET_N(!rst_tri_en),
  .CE(ctl_frf_wens | ctl_frf_ren),
  .A(ctl_frf_addr),
  .DIN(dp_frf_data),
  .BW(write_mask),
  .RDWEN(~ctl_frf_wens),
  .DOUT(read_data),

  .BIST_COMMAND(rtap_srams_bist_command),
  .BIST_DIN(rtap_srams_bist_data),
  .BIST_DOUT(srams_rtap_data),
  .SRAMID(8'd5)
);

   always @ (posedge rclk)
   begin
      frf_dp_data_f <= read_data;
   end

   // dff_s #(39) dataout_dff1(.din(read_data[77:39]), .clk(rclk), .q(frf_dp_data[77:39]),
   //                        .se(real_se), .si(read_scan_data_lo[38:0]), .so(read_scan_data_hi[38:0]));
   // dff_s #(39) dataout_dff2(.din(read_data[38:0]), .clk(rclk), .q(frf_dp_data[38:0]),
   //                        .se(real_se), .si({read_scan_data_hi[37:0],write_scan_data_lo[0]}),
   //                        .so(read_scan_data_lo[38:0]));
   // assign so = read_scan_data_hi[38];


   // always @ (negedge rclk) begin
   //    // latch rst_tri_en
   //    rst_tri_en_negedge <= rst_tri_en;
   // end

endmodule // sparc_ffu_frf


































































































































// Modified by Princeton University on June 9th, 2015
// ========== Copyright Header Begin ==========================================
//
// OpenSPARC T1 Processor File: bw_r_icd.v
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
 //  Module Name:  bw_r_icd
 //  Description:
 //    The ICD contains the icache data.
 //    32B line size.
 //    Write BW: 16B
 //    Read BW: 16Bx2 (fetdata and topdata), collapsed to 4Bx2
 //    Associativity: 4
 //    Write boundary: 34b (32b inst + parity + predec bit)
 //    NOTES:
 //    1. No clock enable.  Rd/Wr enable is used to trigger the
 //    operation.
 //    2. 2:1 mux on address input.  Selects provided externally.
 //    3. 3:1 mux on data input.   Selects provided and guaranteed
 //    exclusive, externally.
 //
 */


////////////////////////////////////////////////////////////////////////
// Global header file includes
////////////////////////////////////////////////////////////////////////
//`include "sys.h" // system level definition file which contains the
// time scale definition


////////////////////////////////////////////////////////////////////////
// Local header file includes / local defines
////////////////////////////////////////////////////////////////////////

// Modified by Princeton University on June 9th, 2015
/*
* ========== Copyright Header Begin ==========================================
* 
* OpenSPARC T1 Processor File: ifu.h
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
////////////////////////////////////////////////////////////////////////
/*
//
//  Module Name: ifu.h
//  Description:	
//  All ifu defines
*/

//--------------------------------------------
// Icache Values in IFU::ICD/ICV/ICT/FDP/IFQDP
//--------------------------------------------

// devices.xml








// `IC_WAY_MASK

// Set Values
// !!IMPORTANT!! a change to IC_LINE_SZ will mean a change to the code as
//   well.  Unfortunately this has not been properly parametrized.
//   Changing the IC_LINE_SZ param alone is *not* enough.
// `define IC_LINE_SZ  32



// !!IMPORTANT!! a change to IC_TAG_HI will mean a change to the code as
//   well.  Changing the IC_TAG_HI param alone is *not* enough to
//   change the PA range. 
// highest bit of PA


// Derived Values
// IC_IDX_HI = log(icache_size/4ways) - 1
// 11


// 4095
// `define IC_ARR_HI (`IC_SZ/`IC_NUM_WAY - 1)

// number of entries - 1 = 511

// 128 - 1

// 32


// 12


// 28

// `define IC_TAG_MASK_ALL ((`IC_TLB_TAG_SZ * `IC_NUM_WAY)-1):0


// 4

// `define IC_PARITY_PADDING (32 - `IC_TAG_SZ - 1)




// 7


// tags for all 4 ways + parity
// 116
// `define IC_TAG_ALL   ((`IC_TAG_SZ * `IC_NUM_WAY) + 4)

// 115
// `define IC_TAG_ALL_HI   ((`IC_TAG_SZ * `IC_NUM_WAY) + 3)

// physical implementation defines














// TLB
// `define IC_TLB_TAG_SZ 30
// tag + 1 bit parity



























//----------------------------------------------------------------------
// For thread scheduler in IFU::DTU::SWL
//----------------------------------------------------------------------
// thread states:  (thr_state[4:0])









// thread configuration register bit fields







//----------------------------------------------------------------------
// For MIL fsm in IFU::IFQ
//----------------------------------------------------------------------











//---------------------------------------------------
// Interrupt Block
//---------------------------------------------------







//-------------------------------------
// IFQ
//-------------------------------------
// valid bit plus ifill













//`ifdef SPARC_L2_64B


//`else
//`define BANK_ID_HI 8
//`define BANK_ID_LO 7
//`endif

//`define CPX_INV_PA_HI  116
//`define CPX_INV_PA_LO  112







//----------------------------------------
// IFU Traps
//----------------------------------------
// precise















// disrupting






















//PITON_PROTO enables all FPGA related modifications






















































































































































































































































































































































































module bw_r_icd(

    // sram wrapper interface
    sram_icache_w10_rtap_data,
    sram_icache_w32_rtap_data,
    rtap_srams_bist_command,
    rtap_srams_bist_data,


    icd_wsel_fetdata_s1, icd_wsel_topdata_s1, icd_fuse_repair_value,
    icd_fuse_repair_en, so, rclk, se, si, reset_l, sehold, fdp_icd_index_bf,
    ifq_icd_index_bf, fcl_icd_index_sel_ifq_bf, ifq_icd_wrway_bf,
    ifq_icd_worden_bf, ifq_icd_wrdata_i2, fcl_icd_rdreq_bf,
    fcl_icd_wrreq_bf, bist_ic_data, rst_tri_en, ifq_icd_data_sel_old_i2,
    ifq_icd_data_sel_fill_i2, ifq_icd_data_sel_bist_i2, fuse_icd_wren,
    fuse_icd_rid, fuse_icd_repair_value, fuse_icd_repair_en,
    efc_spc_fuse_clk1);

    // sram wrapper interface
    output [4-1:0] sram_icache_w10_rtap_data;
    output [4-1:0] sram_icache_w32_rtap_data;
    input  [4-1:0] rtap_srams_bist_command;
    input  [4-1:0] rtap_srams_bist_data;

    input           rclk;
    input           se;
    input           si;
    input           reset_l;
    input           sehold;
    input   [(6 + 5):2]      fdp_icd_index_bf;
    input   [(6 + 5):2]      ifq_icd_index_bf;
    input           fcl_icd_index_sel_ifq_bf;
    input   [1:0]       ifq_icd_wrway_bf;
    input   [3:0]       ifq_icd_worden_bf;
    input   [135:0]     ifq_icd_wrdata_i2;
    input           fcl_icd_rdreq_bf;
    input           fcl_icd_wrreq_bf;
    input   [7:0]       bist_ic_data;
    input           rst_tri_en;
    input           ifq_icd_data_sel_old_i2;
    input           ifq_icd_data_sel_fill_i2;
    input           ifq_icd_data_sel_bist_i2;
    input           fuse_icd_wren;
    input   [3:0]       fuse_icd_rid;
    input   [7:0]       fuse_icd_repair_value;
    input   [1:0]       fuse_icd_repair_en;
    input           efc_spc_fuse_clk1;
    output  [135:0]     icd_wsel_fetdata_s1;
    output  [135:0]     icd_wsel_topdata_s1;
    output  [7:0]       icd_fuse_repair_value;
    output  [1:0]       icd_fuse_repair_en;
    output          so;

    reg [7:0]       icd_fuse_repair_value;
    reg [1:0]       icd_fuse_repair_en;
    reg [135:0]     fetdata_f;
    reg [135:0]     topdata_f;
    // reg [135:0]     fetdata_sa;
    // reg [135:0]     topdata_sa;
    reg [135:0]     fetdata_s1;
    reg [135:0]     topdata_s1;
    wire            clk;
    wire    [135:0]     next_wrdata_bf;
    reg    [135:0]     wrdata_f;
    wire    [135:0]     bist_data_expand;
    wire [(6 + 5):2]     index_bf;
    reg [(6 + 5):2]      index_f;
    reg [(6 + 5):2]      index_s1;
    // reg [`IC_IDX_HI:0]      wr_index0;
    // reg [`IC_IDX_HI:0]      wr_index1;
    // reg [`IC_IDX_HI:0]      wr_index2;
    // reg [`IC_IDX_HI:0]      wr_index3;
    // reg         rdreq_f;
    // reg         wrreq_f;
    // reg [3:0]       worden_f;
    reg [1:0]       wrway_f;


    assign clk = rclk;
    assign index_bf = (fcl_icd_index_sel_ifq_bf ? ifq_icd_index_bf :
        fdp_icd_index_bf);

    // wire [`IC_IDX_HI:2] top_index = {index_f[`IC_IDX_HI:3] , 1'b1};

    always @ (posedge clk)
    begin
        wrway_f <= ifq_icd_wrway_bf;
        index_f <= index_bf;
        index_s1 <= index_f;
    end

    assign bist_data_expand = 136'b0;
    assign icd_wsel_fetdata_s1 = fetdata_s1;
    assign icd_wsel_topdata_s1 = topdata_s1;

    // mux3ds #(136) icden_mux(
    //     .dout               (next_wrdata_bf),
    //     .in0                (wrdata_f),
    //     .in1                (ifq_icd_wrdata_i2),
    //     .in2                (bist_data_expand),
    //     .sel0               (ifq_icd_data_sel_old_i2),
    //     .sel1               (ifq_icd_data_sel_fill_i2),
    //     .sel2               (ifq_icd_data_sel_bist_i2));
    // dffe_s #(136) wrdata_reg(
    //     .din                (next_wrdata_bf),
    //     .clk                (clk),
    //     .q              (wrdata_f),
    //     .en             ((~sehold)),
    //     .se             (se));

    assign next_wrdata_bf = ifq_icd_data_sel_old_i2 ? wrdata_f :
                            ifq_icd_data_sel_fill_i2 ? ifq_icd_wrdata_i2 : bist_data_expand;

    always @ (posedge clk)
    begin
        wrdata_f <= next_wrdata_bf;
    end

    wire  [543:0] read_data_f;
    wire [543:0] wrdata_expanded_bf;
    reg  [543:0] wrmask_expanded_bf;
    wire [135:0] wrmask_bf;

sram_l1i_data icache_way_10
(
    .MEMCLK(rclk),
    .RESET_N(reset_l),
    .CE(fcl_icd_rdreq_bf | fcl_icd_wrreq_bf),
    .A(index_bf[(6 + 5):4]),
    .DIN(wrdata_expanded_bf[271:0]),
    .BW(wrmask_expanded_bf[271:0]),
    .RDWEN(~fcl_icd_wrreq_bf),
    .DOUT(read_data_f[271:0]),

    .BIST_COMMAND(rtap_srams_bist_command),
    .BIST_DIN(rtap_srams_bist_data),
    .BIST_DOUT(sram_icache_w10_rtap_data),
    .SRAMID(8'd1)
);


sram_l1i_data icache_way_32
(
    .MEMCLK(rclk),
    .RESET_N(reset_l),
    .CE(fcl_icd_rdreq_bf | fcl_icd_wrreq_bf),
    .A(index_bf[(6 + 5):4]),
    .DIN(wrdata_expanded_bf[543:272]),
    .BW(wrmask_expanded_bf[543:272]),
    .RDWEN(~fcl_icd_wrreq_bf),
    .DOUT(read_data_f[543:272]),

    .BIST_COMMAND(rtap_srams_bist_command),
    .BIST_DIN(rtap_srams_bist_data),
    .BIST_DOUT(sram_icache_w32_rtap_data),
    .SRAMID(8'd2)
);


    // reg  [543:0] read_data_s1;
    wire [33:0]     icdata_ary_00_00;
    wire [33:0]     icdata_ary_00_01;
    wire [33:0]     icdata_ary_00_10;
    wire [33:0]     icdata_ary_00_11;
    wire [33:0]     icdata_ary_01_00;
    wire [33:0]     icdata_ary_01_01;
    wire [33:0]     icdata_ary_01_10;
    wire [33:0]     icdata_ary_01_11;
    wire [33:0]     icdata_ary_10_00;
    wire [33:0]     icdata_ary_10_01;
    wire [33:0]     icdata_ary_10_10;
    wire [33:0]     icdata_ary_10_11;
    wire [33:0]     icdata_ary_11_00;
    wire [33:0]     icdata_ary_11_01;
    wire [33:0]     icdata_ary_11_10;
    wire [33:0]     icdata_ary_11_11;
    // reg [135:0]     fetdata_s1;
    // reg [135:0]     topdata_s1;

    assign {icdata_ary_11_00,icdata_ary_11_01,icdata_ary_11_10,icdata_ary_11_11,
            icdata_ary_10_00,icdata_ary_10_01,icdata_ary_10_10,icdata_ary_10_11,
            icdata_ary_01_00,icdata_ary_01_01,icdata_ary_01_10,icdata_ary_01_11,
            icdata_ary_00_00,icdata_ary_00_01,icdata_ary_00_10,icdata_ary_00_11} = read_data_f;

    // way, word

    always @ *
    begin
      case (index_f[3:2])
        2'b00:
        begin
          fetdata_f = {icdata_ary_11_00, icdata_ary_10_00, icdata_ary_01_00, icdata_ary_00_00};
          topdata_f = {icdata_ary_11_01, icdata_ary_10_01, icdata_ary_01_01, icdata_ary_00_01};
        end
        2'b01:
        begin
          fetdata_f = {icdata_ary_11_01, icdata_ary_10_01, icdata_ary_01_01, icdata_ary_00_01};
          topdata_f = {icdata_ary_11_01, icdata_ary_10_01, icdata_ary_01_01, icdata_ary_00_01};
        end
        2'b10:
        begin
          fetdata_f = {icdata_ary_11_10, icdata_ary_10_10, icdata_ary_01_10, icdata_ary_00_10};
          topdata_f = {icdata_ary_11_11, icdata_ary_10_11, icdata_ary_01_11, icdata_ary_00_11};
        end
        2'b11:
        begin
          fetdata_f = {icdata_ary_11_11, icdata_ary_10_11, icdata_ary_01_11, icdata_ary_00_11};
          topdata_f = {icdata_ary_11_11, icdata_ary_10_11, icdata_ary_01_11, icdata_ary_00_11};
        end
      endcase
    end

    // always @ (posedge clk)
    // begin
    // end

    ///WRITE

    assign wrdata_expanded_bf = {4{next_wrdata_bf}};
    assign wrmask_bf = {{34{ifq_icd_worden_bf[0]}},{34{ifq_icd_worden_bf[1]}},{34{ifq_icd_worden_bf[2]}},{34{ifq_icd_worden_bf[3]}}};

    always @ *
    begin
        case (ifq_icd_wrway_bf)
          2'b11:
          begin
            wrmask_expanded_bf = {wrmask_bf, 136'b0, 136'b0, 136'b0};
          end
          2'b10:
          begin
            wrmask_expanded_bf = {136'b0, wrmask_bf, 136'b0, 136'b0};
          end
          2'b01:
          begin
            wrmask_expanded_bf = {136'b0, 136'b0, wrmask_bf, 136'b0};
          end
          2'b00:
          begin
            wrmask_expanded_bf = {136'b0, 136'b0, 136'b0, wrmask_bf};
          end
        endcase
    end

    always @ (posedge clk)
    begin
        fetdata_s1 <= fetdata_f;
        topdata_s1 <= topdata_f;
    end


endmodule


 // IBM









// Modified by Princeton University on June 9th, 2015
// ========== Copyright Header Begin ==========================================
//
// OpenSPARC T1 Processor File: sram_l1i_val.v
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
 //  Module Name:  sram_l1i_val
 //  Description:
 //   1r1w array for icache and dcache valid bits.
 //   Modified to conform to naming convention
 //   Added 16 bit wr en
 //   Made bit_wen and din flopped inputs
 //   So all inputs are setup to flops in the stage before memory
 //   access.  The data output is available one cycle later (same
 //   stage as mem access)
 //
 //  IMPORTANT NOTE: This block has to work even in the case where
 //  there is contention between a read and write operation for the
 //  same address.  Based on ease of implementation, the behavior
 //  during contention is defined as follows.
 //    -- write always succeeds
 //    -- read data is (array_data & write_data)
 //       (i.e. old_data & new_data)
 //
 //   So read 0 always succeeds.  read 1 succeeds if the data being
 //   written is also a 1.  Otherwise it fails.
 //
 // new_data = 1, old_data = 0, does not give the expected or
 // predictable result in post layout, so the code has been modified
 // to be
 // old new rd_data
 // --- --- -------
 // 0    0     0
 // 0    1     X
 // 1    0     0
 // 1    1     1
 //
 // **The write still succeeds in ALL cases**
 */

////////////////////////////////////////////////////////////////////////
// Global header file includes
////////////////////////////////////////////////////////////////////////
//`include "sys.h" // system level definition file which contains the
// time scale definition

//`include "iop.h"
// Modified by Princeton University on June 9th, 2015
/*
* ========== Copyright Header Begin ==========================================
* 
* OpenSPARC T1 Processor File: ifu.h
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
////////////////////////////////////////////////////////////////////////
/*
//
//  Module Name: ifu.h
//  Description:	
//  All ifu defines
*/

//--------------------------------------------
// Icache Values in IFU::ICD/ICV/ICT/FDP/IFQDP
//--------------------------------------------

// devices.xml








// `IC_WAY_MASK

// Set Values
// !!IMPORTANT!! a change to IC_LINE_SZ will mean a change to the code as
//   well.  Unfortunately this has not been properly parametrized.
//   Changing the IC_LINE_SZ param alone is *not* enough.
// `define IC_LINE_SZ  32



// !!IMPORTANT!! a change to IC_TAG_HI will mean a change to the code as
//   well.  Changing the IC_TAG_HI param alone is *not* enough to
//   change the PA range. 
// highest bit of PA


// Derived Values
// IC_IDX_HI = log(icache_size/4ways) - 1
// 11


// 4095
// `define IC_ARR_HI (`IC_SZ/`IC_NUM_WAY - 1)

// number of entries - 1 = 511

// 128 - 1

// 32


// 12


// 28

// `define IC_TAG_MASK_ALL ((`IC_TLB_TAG_SZ * `IC_NUM_WAY)-1):0


// 4

// `define IC_PARITY_PADDING (32 - `IC_TAG_SZ - 1)




// 7


// tags for all 4 ways + parity
// 116
// `define IC_TAG_ALL   ((`IC_TAG_SZ * `IC_NUM_WAY) + 4)

// 115
// `define IC_TAG_ALL_HI   ((`IC_TAG_SZ * `IC_NUM_WAY) + 3)

// physical implementation defines














// TLB
// `define IC_TLB_TAG_SZ 30
// tag + 1 bit parity



























//----------------------------------------------------------------------
// For thread scheduler in IFU::DTU::SWL
//----------------------------------------------------------------------
// thread states:  (thr_state[4:0])









// thread configuration register bit fields







//----------------------------------------------------------------------
// For MIL fsm in IFU::IFQ
//----------------------------------------------------------------------











//---------------------------------------------------
// Interrupt Block
//---------------------------------------------------







//-------------------------------------
// IFQ
//-------------------------------------
// valid bit plus ifill













//`ifdef SPARC_L2_64B


//`else
//`define BANK_ID_HI 8
//`define BANK_ID_LO 7
//`endif

//`define CPX_INV_PA_HI  116
//`define CPX_INV_PA_LO  112







//----------------------------------------
// IFU Traps
//----------------------------------------
// precise















// disrupting













////////////////////////////////////////////////////////////////////////
// Local header file includes / local defines
////////////////////////////////////////////////////////////////////////


module sram_l1i_val (/*AUTOARG*/
   // Outputs
   dout, so,
   // Inputs
   rclk, se, si, reset_l, sehold, rst_tri_en, rd_adr1, rd_adr2,
   rd_adr1_sel, rd_en, wr_adr, wr_en, bit_wen, din,

   sramid,
   srams_rtap_data,
   rtap_srams_bist_command,
   rtap_srams_bist_data
   );


input        rclk;
input        se;
input        si;
input        reset_l;
input        sehold;          // scan enable hold
input        rst_tri_en;

// 11:5(I);10:4(D)
input [6:0]     rd_adr1 ;     // rd address-1
input [6:0]     rd_adr2 ;     // rd address-2

input        rd_adr1_sel ;  // sel rd addr 1
input        rd_en ;            // rd enable

// 11:7(I);10:6(D)
input [6:2]     wr_adr ;  // wr address

input        wr_en ;        // wr enable
input [15:0] bit_wen ;  // write enable with bit select
input        din ;        // write data

output reg [3:0]    dout ;    // valid bits for tag compare

output       so;

wire         clk;
assign       clk = rclk;


// sram wrapper interface
output reg [4-1:0] srams_rtap_data;
input  [4-1:0] rtap_srams_bist_command;
input  [4-1:0] rtap_srams_bist_data;
input  [8-1:0] sramid;

// interface to debug module
reg [15:0] debug_data_bmask;
reg [15:0] debug_data_in;
reg [15:0] debug_data_out;
reg [6:0] debug_address;
reg debug_rw;
reg debug_en;

// trin's reimplementation
reg [15:0] mem [0:((8192/32/2/4))-1];
wire rst_all = rst_tri_en | ~reset_l;

reg write_en_x;
reg write_bit_x;
reg [15:0] write_mask_x;
reg [6:2] write_address_x;
reg read_en_x;
reg [6:0] read_address_x;
reg debug_en_x;

reg write_en_y;
reg write_bit_y;
reg [15:0] write_mask_y;
reg [6:2] write_address_y;
reg read_en_y;
reg [6:0] read_address_y;
reg debug_en_y;

always @ *
begin

   write_en_x = wr_en; 
   write_bit_x = din;
   write_mask_x = bit_wen;
   write_address_x = wr_adr;

   read_en_x = rd_en;
   read_address_x = rd_adr1_sel ? rd_adr1 : rd_adr2;

   if (debug_en)
   begin
      write_en_x = debug_rw == 1'b1; 
      // write_bit_x = debug_data_in;
      write_mask_x = debug_data_in;
      write_address_x = 0;
      write_address_x[6:2] = debug_address[6-2:0];

      read_en_x = debug_rw == 1'b0;
      read_address_x = 0;
      read_address_x[6:2] = debug_address[6-2:0];
   end
   debug_en_x = debug_en;

end

always @ (posedge clk)
begin
   write_en_y <= write_en_x;
   write_bit_y <= write_bit_x;
   write_mask_y <= write_mask_x;
   write_address_y <= write_address_x;
   read_en_y <= read_en_x;
   debug_en_y <= debug_en_x;
   read_address_y <= read_address_x;
end

// WRITE
integer memidx;
reg [15:0] write_data_y;
always @ *
begin
   write_data_y = (mem[write_address_y[6:2]] & ~write_mask_y) | ({16{write_bit_y}} & write_mask_y);
   if (debug_en_y)
      write_data_y = write_mask_y;
end
always @ (posedge clk)
begin
   if (rst_all)
   begin
      // mem[0] <= 16'b0;
      // mem[1] <= 16'b0;
      // mem[2] <= 16'b0;
      // mem[3] <= 16'b0;
      // mem[4] <= 16'b0;
      // mem[5] <= 16'b0;
      // mem[6] <= 16'b0;
      // mem[7] <= 16'b0;
      // mem[8] <= 16'b0;
      // mem[9] <= 16'b0;
      // mem[10] <= 16'b0;
      // mem[11] <= 16'b0;
      // mem[12] <= 16'b0;
      // mem[13] <= 16'b0;
      // mem[14] <= 16'b0;
      // mem[15] <= 16'b0;
      // mem[16] <= 16'b0;
      // mem[17] <= 16'b0;
      // mem[18] <= 16'b0;
      // mem[19] <= 16'b0;
      // mem[20] <= 16'b0;
      // mem[21] <= 16'b0;
      // mem[22] <= 16'b0;
      // mem[23] <= 16'b0;
      // mem[24] <= 16'b0;
      // mem[25] <= 16'b0;
      // mem[26] <= 16'b0;
      // mem[27] <= 16'b0;
      // mem[28] <= 16'b0;
      // mem[29] <= 16'b0;
      // mem[30] <= 16'b0;
      // mem[31] <= 16'b0;
      
      // genvar memidx;
      // generate
      // for (memidx = 0; memidx < `IC_VAL_SET_COUNT; memidx = memidx + 1) begin
      //   mem[memidx] <= 16b'0;
      // end
      
      for (memidx = 0; memidx < ((8192/32/2/4)); memidx = memidx + 1) begin
        mem[memidx] <= 16'b0;
      end
   end
   else
   begin
      if (write_en_y)
      begin
         mem[write_address_y[6:2]] <= write_data_y;
      end
   end
end

// READ
reg [15:0] read1;
reg [15:0] read2;
reg [15:0] read3;
reg [15:0] write_to_0;
always @ *
begin
   read1 = mem[read_address_y[6:2]];

   // write/read conflict
   // basically, final_read = sram_read & write
   read2 = read1;
   write_to_0 = ~{16{write_bit_y}} & write_mask_y;
   if ((read_address_y[6:2] == write_address_y[6:2]) && write_en_y)
      read2 = read1 & ~write_to_0;

   // write_to_0 = 0;
   // if ((read_address_y[6:2] == write_address_y[6:2]) && write_en_y)
   //    write_to_0 = write_mask_y;

   // read2 = (~write_to_0 & read1 | write_to_0 & {16{write_bit_y}} & read1);
   debug_data_out = read2;

   case (read_address_y[1:0])
      2'b00:
         read3 = read2[3:0];
      2'b01:
         read3 = read2[7:4];
      2'b10:
         read3 = read2[11:8];
      2'b11:
         read3 = read2[15:12];
   endcase

   dout = 0;
   if (read_en_y)
      dout = read3;
end


///////////////////////////////////////////////////
// implements the standardized SRAM interface
///////////////////////////////////////////////////
localparam BIST_STATE_INIT = 1'b0;
localparam BIST_STATE_CAPTURE_READ = 1'b1;
localparam BIST_DATA_WIDTH = 16;

reg [15:0] bist_address_reg;
reg [3:0] bist_id_reg;
reg [BIST_DATA_WIDTH-1:0] bist_data_reg;
reg [7:0] bist_bsel_reg;
reg bist_state;

reg [15:0] bist_address_reg_next;
reg [3:0] bist_id_reg_next;
reg [BIST_DATA_WIDTH-1:0] bist_data_reg_next;
reg [7:0] bist_bsel_reg_next;
reg bist_state_next;

// wires
reg bist_en;
reg bist_rdwen;
reg bist_data_capture_read_en;
reg bist_address_shift_en;
reg bist_data_shift_en;
reg bist_bsel_shift_en;
reg bist_id_shift_en;
reg [4-1:0] bist_command;


always @ (posedge clk)
begin
   if (!reset_l)
   begin
      // only resetting important variables
      bist_id_reg <= 0;
      bist_state <= BIST_STATE_INIT;
   end
   else
   begin
      bist_data_reg <= bist_data_reg_next;
      bist_state <= bist_state_next;
      bist_address_reg <= bist_address_reg_next;
      bist_bsel_reg <= bist_bsel_reg_next;
      bist_id_reg <= bist_id_reg_next;
   end
end

always @ *
begin
   bist_data_reg_next = bist_data_reg;
   bist_bsel_reg_next = bist_bsel_reg;
   bist_address_reg_next = bist_address_reg;
   bist_id_reg_next = bist_id_reg;

   if (bist_data_capture_read_en)
   begin
      bist_data_reg_next = debug_data_out[BIST_DATA_WIDTH-1:0];
   end

   if (bist_data_shift_en)
   begin
      bist_data_reg_next = {bist_data_reg[BIST_DATA_WIDTH-1 - 4 : 0], rtap_srams_bist_data};
   end

   if (bist_address_shift_en)
   begin
      bist_address_reg_next = {bist_address_reg[15 - 4 : 0], rtap_srams_bist_data};
   end

   if (bist_bsel_shift_en)
   begin
      // assuming bus width is 4 and bsel is 8b
      bist_bsel_reg_next = {bist_bsel_reg[3:0], rtap_srams_bist_data[3:0]};
   end

   if (bist_id_shift_en)
   begin
      // assuming bus width is 4 and id is 4b
      bist_id_reg_next = rtap_srams_bist_data[3:0];
   end

   // assign output to be the most significant bits while shifting
   srams_rtap_data = bist_data_reg[BIST_DATA_WIDTH-1 -: 4];
end

// make/validate bist command
always @ *
begin
   // except for id shift, all commands need to have matching sramid to be activated
   bist_command = 0;
   if (bist_id_reg == sramid 
         || rtap_srams_bist_command == 4'd6
         || bist_id_reg == 8'b1111_1111)
      bist_command = rtap_srams_bist_command;
end
// combinational logics for states
always @ *
begin
   bist_state_next = BIST_STATE_INIT;
   // general processing of commands
   bist_data_capture_read_en = 1'b0;
   bist_en = 1'b0;
   bist_rdwen = 1'b0;
   bist_data_shift_en = 1'b0;
   bist_address_shift_en = 1'b0;
   bist_bsel_shift_en = 1'b0;
   bist_id_shift_en = 1'b0;
   case (bist_state)
      BIST_STATE_INIT:
      begin
         bist_state_next = BIST_STATE_INIT;
         if (bist_command == 4'd1)
         begin
            bist_state_next = BIST_STATE_CAPTURE_READ;
         end
      end
      BIST_STATE_CAPTURE_READ:
      begin
         bist_data_capture_read_en = 1'b1;
         bist_state_next = BIST_STATE_INIT;
      end
   endcase
   if (bist_command == 4'd1)
   begin
      bist_en = 1'b1;
      bist_rdwen = 1'b0;
   end
   if (bist_command == 4'd2)
   begin
      bist_en = 1'b1;
      bist_rdwen = 1'b1;
   end
   if (bist_command == 4'd4)
   begin
      bist_data_shift_en = 1'b1;
   end
   if (bist_command == 4'd5)
   begin
      bist_address_shift_en = 1'b1;
   end
   if (bist_command == 4'd6)
   begin
      bist_id_shift_en = 1'b1;
   end
   if (bist_command == 4'd7)
   begin
      bist_bsel_shift_en = 1'b1;
   end

   // TODO
   debug_en = bist_en;
   debug_rw = bist_rdwen;
   debug_address[6:0] = bist_address_reg[6:0];
   debug_data_in[15:0] = bist_data_reg[15:0];
   debug_data_bmask = ~(16'b0);
end


endmodule // sram_l1i_val












// Modified by Princeton University on June 9th, 2015
// ========== Copyright Header Begin ==========================================
//
// OpenSPARC T1 Processor File: bw_r_scm.v
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
//	Description:	Store Buffer of Load/Store Unit (CAM Side)
//		 - Physically divided into CAM and DATA RAMs.
//                              - CAM RAM has a single cam port and a single
//                              port for read/writes. The cam port is for loads,
//                              write for stores, read for test/diagnostic purposes.
//                              rd or write can be simultaneous with cam. can rd and cam
//                              a single entry simultaneously. cannot write and cam
//                              the same entry.
//                              - DATA RAM read occurs for a load raw match in the
//                              stb CAM RAM. DATA RAM write occurs a store. Both
//                              actions are architecturally guaranteed to be
//                              mutex.
//                              - Write occurs simultaneously to both arrays.
//                              - Reads are not necessarily simultaneous and are
//                              controlled by individual read signals.
//                              - Certain bits are maintained outside the array
//                              in the stb's control section, such as the valid
//                              bits.
//
*/

////////////////////////////////////////////////////////////////////////
// Local header file includes / local defines
////////////////////////////////////////////////////////////////////////

//PITON_PROTO enables all FPGA related modifications






module bw_r_scm (/*AUTOARG*/
   // Outputs

    stb_rcsm_ramc,

   stb_rdata_ramc, stb_ld_full_raw, stb_ld_partial_raw,
   stb_cam_hit_ptr, stb_cam_hit, stb_cam_mhit,
   // Inputs

   stb_cam_csm,stb_alt_wr_csm,

   stb_cam_data, stb_alt_wr_data, stb_camwr_data, stb_alt_wsel,
   stb_cam_vld, stb_cam_cm_tid, stb_cam_sqsh_msk, stb_cam_rw_ptr,
   stb_cam_wptr_vld, stb_cam_rptr_vld, stb_cam_rw_tid,
   stb_quad_ld_cam, rclk, rst_tri_en
   ) ;

parameter NUMENTRIES = 32 ;				// number of entries in stb

input	[44:15]		stb_cam_data ;	  // data for compare; disjoint msb
input	[44:15]		stb_alt_wr_data ;	  // data for compare; disjoint msb
input	[14:0]		stb_camwr_data ;  // data for compare/write; common lsb
input			stb_alt_wsel ;
input			stb_cam_vld ;	  // cam is required.
input	[1:0]		stb_cam_cm_tid ;  // thread id for cam operation.
input	[7:0]		stb_cam_sqsh_msk; // mask for squashing cam results.

input 	[2:0]		stb_cam_rw_ptr ;  // wr pointer for single port.
input 	     		stb_cam_wptr_vld ;// write pointer vld
input 	     		stb_cam_rptr_vld ;// write pointer vld
input	[1:0]		stb_cam_rw_tid ;  // thread id for rw.
input 			stb_quad_ld_cam ; // quad-ld cam.

input			rclk ;		  // clock

//input			scan_ena ;	  // no longer required !
//input	[7:0]		adj ;

input			rst_tri_en ;


input   [32:0] stb_cam_csm;
input   [32:0] stb_alt_wr_csm;

output  [32:0] stb_rcsm_ramc;


output	[44:0]		stb_rdata_ramc ;  // rd data from CAM RAM.
// raw output is muxed on a thread basis.
output	[7:0]		stb_ld_full_raw ; // ld with full raw.
output	[7:0]		stb_ld_partial_raw ; // ld with partial raw.
output	[2:0]		stb_cam_hit_ptr ;
output			stb_cam_hit ;	  // any hit in stb
output			stb_cam_mhit ;	  // multiple hits in stb


/*UTOREG*/
// Beginning of automatic regs (for this module's undeclared outputs)
// End of automatics
reg [44:0]		stb_rdata_ramc ;
reg [31:0]		rw_wdline ;
reg [44:0]		stb_ramc [NUMENTRIES-1:0] /* synthesis syn_ramstyle = block_ram  syn_ramstyle = no_rw_check */;
reg [44:0]		ramc_entry ;
reg [36:0]		cam_tag ;
reg [31:0]		ptag_hit ;
reg [7:0]		cam_bmask ;
reg [31:0]		byte_match ;
reg [31:0]		byte_overlap ;
reg [31:0]		ld_full_raw ;
reg [31:0]		ld_partial_raw ;
reg [44:15]		alt_wr_data ;
reg [44:15]		pipe_wr_data ;
reg [14:0]		camwr_data ;
reg			wptr_vld ;
reg			rptr_vld_tmp ;
reg [1:0]	  	cam_tid ;
reg [1:0]	  	cam_vld ;
reg			alt_wsel ;

wire		rptr_vld ;
wire 		ldq ;
wire	[7:0]	sqsh_msk ;
wire 	[7:0]	ld_full_raw_mx ;
wire    [7:0]	ld_partial_raw_mx ;
wire	[7:0]	ptag_hit_mx ;
wire	[7:0]	byte_overlap_mx ;
wire	[7:0]	byte_match_mx ;
wire	[7:0]	cam_hit ;
wire	[44:0]	wdata_ramc ;
wire	[44:0]	cam_data ;
wire	[44:15] wr_data ;

reg	[4:0]	stb_addr;



reg [32:0] stb_rcsm_ramc;
reg [32:0] stb_csm_ramc [NUMENTRIES-1:0];
reg [32:0] pipe_wr_csm;
wire [32:0] wcsm_ramc;
reg [32:0] alt_wr_csm;


integer	i,j,k,l ;


wire	scan_ena ;
assign	scan_ena = 1'b0 ;

//=========================================================================================
//	generate wordlines
//=========================================================================================

assign	sqsh_msk[7:0]	= stb_cam_sqsh_msk[7:0];

// cam_vld and cam_tid_tmp are set-up a phase earlier.
// Comment out - Now setup to posedge.
/*always @(negedge clk)
	begin
		cam_tid_tmp[1:0]	<= stb_cam_cm_tid[1:0] ;
		cam_vld_tmp		<= stb_cam_vld ;
	end */
















always @(posedge rclk)
	begin
		pipe_wr_data[44:15] <= stb_cam_data[44:15];
		alt_wr_data[44:15] <= stb_alt_wr_data[44:15];
		camwr_data[14:0] <= stb_camwr_data[14:0];
		wptr_vld 	<= stb_cam_wptr_vld ;
		rptr_vld_tmp 	<= stb_cam_rptr_vld ;
		cam_tid[1:0]	<= stb_cam_cm_tid[1:0] ;
		//cam_tid[1:0]	<= cam_tid_tmp[1:0] ;
 		//ldq 		<=  stb_quad_ld_cam ; Bug 2870
		alt_wsel 	<= stb_alt_wsel ;


		pipe_wr_csm <= stb_cam_csm;
        alt_wr_csm  <= stb_alt_wr_csm;


                stb_addr	<= {stb_cam_rw_tid[1:0],stb_cam_rw_ptr[2:0]};

	end

assign 	ldq =  stb_quad_ld_cam ;
assign  rptr_vld = rptr_vld_tmp | rst_tri_en ;

//=========================================================================================
//	write or read to/from memory
//=========================================================================================

// For blk-st, select out-of-pipe.
assign	wr_data[44:15] = alt_wsel ?
                alt_wr_data[44:15] : pipe_wr_data[44:15] ;

assign	wdata_ramc[44:0] = {wr_data[44:15],camwr_data[14:0]};

assign wcsm_ramc = alt_wsel ? alt_wr_csm : pipe_wr_csm;




// Write
always @ (negedge rclk)
	begin

	if(wptr_vld) begin
		if(~rst_tri_en) begin
			stb_ramc[stb_addr] <= wdata_ramc[44:0];
            stb_csm_ramc[stb_addr] <= wcsm_ramc;
			stb_rdata_ramc[44:0] <=  wdata_ramc[44:0];
            stb_rcsm_ramc <= wcsm_ramc;
                end else begin
			stb_rdata_ramc[44:0] <=  stb_ramc[stb_addr];
            stb_rcsm_ramc <= stb_csm_ramc[stb_addr];
		end
	end






















// Read

		if(rptr_vld & ~scan_ena) begin
			if (rptr_vld & wptr_vld & ~rst_tri_en) begin
				stb_rdata_ramc[44:0] <=  wdata_ramc[44:0];
                stb_rcsm_ramc <= wcsm_ramc;
			end
			else begin
				stb_rdata_ramc[44:0] <=  stb_ramc[stb_addr];
                stb_rcsm_ramc <= stb_csm_ramc[stb_addr];
			end
		end


















	end

























































//=========================================================================================
//	CAM contents of CAM RAM
//=========================================================================================

// - Generate full/partial raw for incoming load.
// - Output signals need to be qualified with per entry
// vlds before causing any subsequent event, the read of
// the DATA RAM specifically.
// - full_raw & vld will cause rd of DATA RAM.
// - partial_raw & vld will cause ld to follow corresponding
// st on way out to xbar.
// - logic to generate partial and full raws may be done outside
// but that would require an additional signal per entry to
// be output.

// Mapping of cam/write data
//
//	| 	40-3=37b(pa)	| 1b(stquad) 	|	8b(bytemask)	| <- use
//	|	45:9		| 8		|	7:0		| <- input port
//				**^ stquad rm'ed

assign	cam_data[44:0] = {stb_cam_data[44:15],stb_camwr_data[14:0]};

always @ (posedge rclk)
	begin

		for (l=0;l<NUMENTRIES;l=l+1)
				begin
				ramc_entry[44:0] = stb_ramc[l] ;

				cam_tag[36:0] = ramc_entry[44:8] ;
				cam_bmask[7:0] = ramc_entry[7:0] ;
				//stq =	ramc_entry[8] ;			// additional bit -stq

				// Prior to adding stb_quad_ld_cam port.
				/*ptag_hit[l] =
					((cam_tag[36:1] == cam_data[44:9]) &
						(((cam_tag[0] == cam_data[8]) & ~stq) | stq)) & stcam_vld_tmp & ~scan_ena ;*/
				// Modification.
				// * remove ramc_entry[8]. Or keep it but it won't be used.
				// * Instead we'll control this from outside.
				ptag_hit[l] =
					(cam_tag[36:1] == cam_data[44:9]) &
						(((cam_tag[0] == cam_data[8]) & ~ldq) | ldq) & stb_cam_vld & ~scan_ena ;
				byte_match[l] = |(cam_bmask[7:0] & cam_data[7:0]) & stb_cam_vld & ~scan_ena ;
				// Simplification :
				byte_overlap[l] = |(~cam_bmask[7:0] & cam_data[7:0]) & stb_cam_vld & ~scan_ena ;

				end
	end

// Mux the raw signals down to 8b quantities. Squash mask comes mid-way thru cycle.


assign	byte_overlap_mx[7:0] =
	(cam_tid[1:0] == 2'b00) ? byte_overlap[7:0] :
		(cam_tid[1:0] == 2'b01) ? byte_overlap[15:8] :
			(cam_tid[1:0] == 2'b10) ? byte_overlap[23:16] :
				(cam_tid[1:0] == 2'b11) ? byte_overlap[31:24] : 8'bxxxx_xxxx ;

assign	byte_match_mx[7:0] =
	(cam_tid[1:0] == 2'b00) ? byte_match[7:0] :
		(cam_tid[1:0] == 2'b01) ? byte_match[15:8] :
			(cam_tid[1:0] == 2'b10) ? byte_match[23:16] :
				(cam_tid[1:0] == 2'b11) ? byte_match[31:24] : 8'bxxxx_xxxx ;

assign	ptag_hit_mx[7:0] =
	(cam_tid[1:0] == 2'b00) ? ptag_hit[7:0] :
		(cam_tid[1:0] == 2'b01) ? ptag_hit[15:8] :
			(cam_tid[1:0] == 2'b10) ? ptag_hit[23:16] :
				(cam_tid[1:0] == 2'b11) ? ptag_hit[31:24] : 8'bxxxx_xxxx ;

assign	stb_ld_full_raw[7:0] =
	ptag_hit_mx[7:0] & byte_match_mx[7:0] & ~byte_overlap_mx[7:0] & ~sqsh_msk[7:0] ;
assign	stb_ld_partial_raw[7:0] =
	ptag_hit_mx[7:0] & byte_match_mx[7:0] &  byte_overlap_mx[7:0] & ~sqsh_msk[7:0] ;

assign	cam_hit[7:0] =
	ptag_hit_mx[7:0] & byte_match_mx[7:0] & ~sqsh_msk[7:0] ;
assign	stb_cam_hit = |(cam_hit[7:0]);

// The stb data is meant to be read for single hit full raw case. It may actually be read
// for full raw, partial raw or multiple hit case but the read output will be ignored for
// partial and multiple hit case. Multiple hits will not cause a hazard as the ptr is first
// encoded and then decoded to form the wdline for the stb-data
// Use cam_hit result to void false hits.
assign	stb_cam_hit_ptr[0] 	=  cam_hit[1] | cam_hit[3] | cam_hit[5] | cam_hit[7] ;
assign	stb_cam_hit_ptr[1] 	=  cam_hit[2] | cam_hit[3] | cam_hit[6] | cam_hit[7] ;
assign	stb_cam_hit_ptr[2] 	=  cam_hit[4] | cam_hit[5] | cam_hit[6] | cam_hit[7] ;

//Generating multiple hits
assign  stb_cam_mhit            =  (cam_hit[0]  & cam_hit[1]) | (cam_hit[2] & cam_hit[3])  |
                                   (cam_hit[4]  & cam_hit[5]) | (cam_hit[6] & cam_hit[7])  |
                                   ((cam_hit[0] | cam_hit[1]) & (cam_hit[2] | cam_hit[3])) |
                                   ((cam_hit[4] | cam_hit[5]) & (cam_hit[6] | cam_hit[7])) |
                                   ((|cam_hit[3:0]) & (|cam_hit[7:4]));

//--------------------------------------------------------------
// Error Checking.
//--------------------------------------------------------------

// 1. simultaneous rd/wr on single port - terminate
// 2. simultaneous cam and wr - terminate
// * PUT OUTSIDE OF SRAM RTL, AS RST NOT AVAILABLE. *

endmodule
// Modified by Princeton University on June 9th, 2015
// ========== Copyright Header Begin ==========================================
//
// OpenSPARC T1 Processor File: bw_r_rf32x152b.v
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
//      Description:    DCache Fill Queue of Load Store Unit.
//                      - Contains invalidates and loads.
//                      - loads will bypass and/or fill dcache.
//                      - Entry at head of queue may have to
//                      be held for multiple passes.
//
*/

////////////////////////////////////////////////////////////////////////
// Local header file includes / local defines
////////////////////////////////////////////////////////////////////////

module bw_r_rf32x152b(dout, so, rd_en, rd_adr, wr_en, wr_adr, din, si, se,
  sehold, rclk, rst_tri_en, reset_l);

  parameter   NUMENTRIES  = 32;

  input [4:0]   rd_adr;
  input     rd_en;
  input     wr_en;
  input [4:0]   wr_adr;
  input [151:0]   din;
  input     rclk;
  input     reset_l;
  input     rst_tri_en;
  input     sehold;
  input     si;
  input     se;
  output  [151:0]   dout;
  // reg [151:0]   dout;
  output      so;

  wire      clk;
  wire      wr_vld;

  reg [151:0]   mem[(NUMENTRIES - 1):0];  /* synthesis syn_ramstyle = block_ram  syn_ramstyle = no_rw_check */
  // reg rd_en_d1;
  reg [4:0]   rd_adr_d1;

  assign clk = rclk;
  assign wr_vld = ((wr_en & (~rst_tri_en)) & reset_l);

  always @(posedge clk) begin
    if (wr_vld) begin
      mem[wr_adr] = din;
    end
    // rd_en_d1 <= rd_en;
    rd_adr_d1 <= rd_adr;
  end

  assign dout = mem[rd_adr_d1];

endmodule

// Modified by Princeton University on June 9th, 2015
// ========== Copyright Header Begin ==========================================
//
// OpenSPARC T1 Processor File: sram_l1d_val.v
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
 //  Module Name:  sram_l1d_val
 //  Description:
 //   1r1w array for icache and dcache valid bits.
 //   Modified to conform to naming convention
 //   Added 16 bit wr en
 //   Made bit_wen and din flopped inputs
 //   So all inputs are setup to flops in the stage before memory
 //   access.  The data output is available one cycle later (same
 //   stage as mem access)
 //
 //  IMPORTANT NOTE: This block has to work even in the case where
 //  there is contention between a read and write operation for the
 //  same address.  Based on ease of implementation, the behavior
 //  during contention is defined as follows.
 //    -- write always succeeds
 //    -- read data is (array_data & write_data)
 //       (i.e. old_data & new_data)
 //
 //   So read 0 always succeeds.  read 1 succeeds if the data being
 //   written is also a 1.  Otherwise it fails.
 //
 // new_data = 1, old_data = 0, does not give the expected or
 // predictable result in post layout, so the code has been modified
 // to be
 // old new rd_data
 // --- --- -------
 // 0    0     0
 // 0    1     X
 // 1    0     0
 // 1    1     1
 //
 // **The write still succeeds in ALL cases**
 */

////////////////////////////////////////////////////////////////////////
// Global header file includes
////////////////////////////////////////////////////////////////////////
//`include "sys.h" // system level definition file which contains the
// time scale definition

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

//`include "iop.h"

////////////////////////////////////////////////////////////////////////
// Local header file includes / local defines
////////////////////////////////////////////////////////////////////////


module sram_l1d_val (/*AUTOARG*/
   // Outputs
   dout, so,
   // Inputs
   rclk, se, si, reset_l, sehold, rst_tri_en, rd_adr1, rd_adr2,
   rd_adr1_sel, rd_en, wr_adr, wr_en, bit_wen, din,

   sramid,
   srams_rtap_data,
   rtap_srams_bist_command,
   rtap_srams_bist_data
   );


input        rclk;
input        se;
input        si;
input        reset_l;
input        sehold;          // scan enable hold
input        rst_tri_en;

// 11:5(I);10:4(D)
input [6:0]     rd_adr1 ;     // rd address-1
input [6:0]     rd_adr2 ;     // rd address-2

input        rd_adr1_sel ;  // sel rd addr 1
input        rd_en ;            // rd enable

// 11:7(I);10:6(D)
input [6:2]     wr_adr ;  // wr address

input        wr_en ;        // wr enable
input [(4*2-1):0] bit_wen ;  // write enable with bit select
input        din ;        // write data

output reg [2-1:0]    dout ;    // valid bits for tag compare

output       so;

wire         clk;
assign       clk = rclk;


// sram wrapper interface
output reg [4-1:0] srams_rtap_data;
input  [4-1:0] rtap_srams_bist_command;
input  [4-1:0] rtap_srams_bist_data;
input  [8-1:0] sramid;

// interface to debug module
reg [(4*2-1):0] debug_data_bmask;
reg [(4*2-1):0] debug_data_in;
reg [(4*2-1):0] debug_data_out;
reg [6:0] debug_address;
reg debug_rw;
reg debug_en;

// trin's reimplementation
reg [(4*2-1):0] mem [0:((256/2)/4)-1];
wire rst_all = rst_tri_en | ~reset_l;

reg write_en_x;
reg write_bit_x;
reg [(4*2-1):0] write_mask_x;
reg [6:2] write_address_x;
reg read_en_x;
reg [6:0] read_address_x;
reg debug_en_x;

reg write_en_y;
reg write_bit_y;
reg [(4*2-1):0] write_mask_y;
reg [6:2] write_address_y;
reg read_en_y;
reg [6:0] read_address_y;
reg debug_en_y;

always @ *
begin

   write_en_x = wr_en; 
   write_bit_x = din;
   write_mask_x = bit_wen;
   write_address_x = wr_adr;

   read_en_x = rd_en;
   read_address_x = rd_adr1_sel ? rd_adr1 : rd_adr2;

   if (debug_en)
   begin
      write_en_x = debug_rw == 1'b1; 
      // write_bit_x = debug_data_in;
      write_mask_x = debug_data_in;
      write_address_x = 0;
      write_address_x[6:2] = debug_address[4:0];

      read_en_x = debug_rw == 1'b0;
      read_address_x = 0;
      read_address_x[6:2] = debug_address[4:0];
   end
   debug_en_x = debug_en;

end

always @ (posedge clk)
begin
   write_en_y <= write_en_x;
   write_bit_y <= write_bit_x;
   write_mask_y <= write_mask_x;
   write_address_y <= write_address_x;
   read_en_y <= read_en_x;
   debug_en_y <= debug_en_x;
   read_address_y <= read_address_x;
end

// WRITE
reg [(4*2-1):0] write_data_y;
always @ *
begin
   write_data_y = (mem[write_address_y[6:2]] & ~write_mask_y) | ({16{write_bit_y}} & write_mask_y);
   if (debug_en_y)
      write_data_y = write_mask_y;
end
always @ (posedge clk)
begin
   if (rst_all)
   begin
      mem[0] <= 16'b0;
      mem[1] <= 16'b0;
      mem[2] <= 16'b0;
      mem[3] <= 16'b0;
      mem[4] <= 16'b0;
      mem[5] <= 16'b0;
      mem[6] <= 16'b0;
      mem[7] <= 16'b0;
      mem[8] <= 16'b0;
      mem[9] <= 16'b0;
      mem[10] <= 16'b0;
      mem[11] <= 16'b0;
      mem[12] <= 16'b0;
      mem[13] <= 16'b0;
      mem[14] <= 16'b0;
      mem[15] <= 16'b0;
      mem[16] <= 16'b0;
      mem[17] <= 16'b0;
      mem[18] <= 16'b0;
      mem[19] <= 16'b0;
      mem[20] <= 16'b0;
      mem[21] <= 16'b0;
      mem[22] <= 16'b0;
      mem[23] <= 16'b0;
      mem[24] <= 16'b0;
      mem[25] <= 16'b0;
      mem[26] <= 16'b0;
      mem[27] <= 16'b0;
      mem[28] <= 16'b0;
      mem[29] <= 16'b0;
      mem[30] <= 16'b0;
      mem[31] <= 16'b0;
   end
   else
   begin
      if (write_en_y)
      begin
         mem[write_address_y[6:2]] <= write_data_y;
      end
   end
end

// READ
reg [(4*2-1):0] read1;
reg [(4*2-1):0] read2;
reg [(4*2-1):0] read3;
reg [(4*2-1):0] write_to_0;
always @ *
begin
   read1 = mem[read_address_y[6:2]];

   // write/read conflict
   // basically, final_read = sram_read & write
   read2 = read1;
   write_to_0 = ~{16{write_bit_y}} & write_mask_y;
   if ((read_address_y[6:2] == write_address_y[6:2]) && write_en_y)
      read2 = read1 & ~write_to_0;

   // write_to_0 = 0;
   // if ((read_address_y[`L1D_SET_IDX_HI:2] == write_address_y[`L1D_SET_IDX_HI:2]) && write_en_y)
   //    write_to_0 = write_mask_y;

   // read2 = (~write_to_0 & read1 | write_to_0 & {16{write_bit_y}} & read1);
   debug_data_out = read2;

   case (read_address_y[1:0])
      2'b00:
         read3 = read2[2*1-1 -: 2];
      2'b01:
         read3 = read2[2*2-1 -: 2];
      2'b10:
         read3 = read2[2*3-1 -: 2];
      2'b11:
         read3 = read2[2*4-1 -: 2];
   endcase

   dout = 0;
   if (read_en_y)
      dout = read3;
end


///////////////////////////////////////////////////
// implements the standardized SRAM interface
///////////////////////////////////////////////////
localparam BIST_STATE_INIT = 1'b0;
localparam BIST_STATE_CAPTURE_READ = 1'b1;
localparam BIST_DATA_WIDTH = 16;

reg [15:0] bist_address_reg;
reg [3:0] bist_id_reg;
reg [BIST_DATA_WIDTH-1:0] bist_data_reg;
reg [7:0] bist_bsel_reg;
reg bist_state;

reg [15:0] bist_address_reg_next;
reg [3:0] bist_id_reg_next;
reg [BIST_DATA_WIDTH-1:0] bist_data_reg_next;
reg [7:0] bist_bsel_reg_next;
reg bist_state_next;

// wires
reg bist_en;
reg bist_rdwen;
reg bist_data_capture_read_en;
reg bist_address_shift_en;
reg bist_data_shift_en;
reg bist_bsel_shift_en;
reg bist_id_shift_en;
reg [4-1:0] bist_command;


always @ (posedge clk)
begin
   if (!reset_l)
   begin
      // only resetting important variables
      bist_id_reg <= 0;
      bist_state <= BIST_STATE_INIT;
   end
   else
   begin
      bist_data_reg <= bist_data_reg_next;
      bist_state <= bist_state_next;
      bist_address_reg <= bist_address_reg_next;
      bist_bsel_reg <= bist_bsel_reg_next;
      bist_id_reg <= bist_id_reg_next;
   end
end

always @ *
begin
   bist_data_reg_next = bist_data_reg;
   bist_bsel_reg_next = bist_bsel_reg;
   bist_address_reg_next = bist_address_reg;
   bist_id_reg_next = bist_id_reg;

   if (bist_data_capture_read_en)
   begin
      bist_data_reg_next = debug_data_out[BIST_DATA_WIDTH-1:0]; // trin reconfig
   end

   if (bist_data_shift_en)
   begin
      bist_data_reg_next = {bist_data_reg[BIST_DATA_WIDTH-1 - 4 : 0], rtap_srams_bist_data};
   end

   if (bist_address_shift_en)
   begin
      bist_address_reg_next = {bist_address_reg[15 - 4 : 0], rtap_srams_bist_data};
   end

   if (bist_bsel_shift_en)
   begin
      // assuming bus width is 4 and bsel is 8b
      bist_bsel_reg_next = {bist_bsel_reg[3:0], rtap_srams_bist_data[3:0]};
   end

   if (bist_id_shift_en)
   begin
      // assuming bus width is 4 and id is 4b
      bist_id_reg_next = rtap_srams_bist_data[3:0];
   end

   // assign output to be the most significant bits while shifting
   srams_rtap_data = bist_data_reg[BIST_DATA_WIDTH-1 -: 4];
end

// make/validate bist command
always @ *
begin
   // except for id shift, all commands need to have matching sramid to be activated
   bist_command = 0;
   if (bist_id_reg == sramid 
         || rtap_srams_bist_command == 4'd6
         || bist_id_reg == 8'b1111_1111)
      bist_command = rtap_srams_bist_command;
end
// combinational logics for states
always @ *
begin
   bist_state_next = BIST_STATE_INIT;
   // general processing of commands
   bist_data_capture_read_en = 1'b0;
   bist_en = 1'b0;
   bist_rdwen = 1'b0;
   bist_data_shift_en = 1'b0;
   bist_address_shift_en = 1'b0;
   bist_bsel_shift_en = 1'b0;
   bist_id_shift_en = 1'b0;
   case (bist_state)
      BIST_STATE_INIT:
      begin
         bist_state_next = BIST_STATE_INIT;
         if (bist_command == 4'd1)
         begin
            bist_state_next = BIST_STATE_CAPTURE_READ;
         end
      end
      BIST_STATE_CAPTURE_READ:
      begin
         bist_data_capture_read_en = 1'b1;
         bist_state_next = BIST_STATE_INIT;
      end
   endcase
   if (bist_command == 4'd1)
   begin
      bist_en = 1'b1;
      bist_rdwen = 1'b0;
   end
   if (bist_command == 4'd2)
   begin
      bist_en = 1'b1;
      bist_rdwen = 1'b1;
   end
   if (bist_command == 4'd4)
   begin
      bist_data_shift_en = 1'b1;
   end
   if (bist_command == 4'd5)
   begin
      bist_address_shift_en = 1'b1;
   end
   if (bist_command == 4'd6)
   begin
      bist_id_shift_en = 1'b1;
   end
   if (bist_command == 4'd7)
   begin
      bist_bsel_shift_en = 1'b1;
   end

   // TODO
   debug_en = bist_en;
   debug_rw = bist_rdwen;
   debug_address[6:0] = bist_address_reg[6:0];
   debug_data_in[(4*2-1):0] = bist_data_reg[(4*2-1):0];
   debug_data_bmask = ~(16'b0);
end


endmodule // sram_l1d_val












// Modified by Princeton University on June 9th, 2015
// ========== Copyright Header Begin ==========================================
//
// OpenSPARC T1 Processor File: bw_r_rf32x80.v
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
//	Description:	Trap Stack Array
//			- Dual-Ported.
//			- Port1 - Write Port; Used by wrpr, trap insertion.
//			Write occurs in W Stage. (M1:M2:W).
//			- Port2 - Read Port; Used by rdpr, done/retry.
//			Read occurs in E Stage.
//			- Arranged as 6(trap-levels/thread) x 4 threads = 24 entries.
//			Trap-level and thread id used to index array.
*/
////////////////////////////////////////////////////////////////////////
// Local header file includes / local defines
////////////////////////////////////////////////////////////////////////









//PITON_PROTO enables all FPGA related modifications



















































































































































































































































module bw_r_rf32x80(/*AUTOARG*/
   // Outputs
   dout, so,
   // Inputs
   rd_en, rd_adr, wr_en, nib_wr_en, wr_adr, din,
   si, se, sehold, rclk, rst_tri_en, reset_l);

parameter NUM_TPL = 6 ;         // 6 supported trap levels.
parameter NUM_ENTRIES = 32 ;    // 8 entries per thread

/*AUTOINPUT*/
// Beginning of automatic inputs (from unused autoinst inputs)
// End of automatics
input [4:0]   rd_adr;     // read adr.
input         rd_en;      // read pointer
input         wr_en;      // write pointer vld
input [19:0]  nib_wr_en;  // enable write of a byte in tsa.
input [4:0]   wr_adr;     // write adr.
input [79:0] din;         // wr data for tsa.
input         rclk;       // clock
input         reset_l;    // active low reset
input         rst_tri_en; // reset and scan
input         sehold;     // scan hold
input         si;         // scan in
input         se;         // scan enable

/*AUTOOUTPUT*/
// Beginning of automatic outputs (from unused autoinst outputs)
// End of automatics
output  [79:0] dout ; // rd data for tsa.
output          so ;   // scan out write

   wire [159:0] write_mask; // write-mask
   reg [3:0] rd_adr_d1;
   reg [3:0] rd_adr_d2;
   reg read_en_d1;
   reg read_en_d2;

// for sastasks?
// reg [79:0]      temp_tlvl = 0;

   // memory
   reg [79:0] inq_ary [NUM_ENTRIES-1:0];

   // read
   assign dout = inq_ary[rd_adr_d1];

   // write
   always @ (posedge rclk)
   begin
      if (!reset_l)
      begin
         // assume flops
         inq_ary[00] <= 80'b0;
         inq_ary[01] <= 80'b0;
         inq_ary[02] <= 80'b0;
         inq_ary[03] <= 80'b0;
         inq_ary[04] <= 80'b0;
         inq_ary[05] <= 80'b0;
         inq_ary[06] <= 80'b0;
         inq_ary[07] <= 80'b0;
         inq_ary[08] <= 80'b0;
         inq_ary[09] <= 80'b0;
         inq_ary[10] <= 80'b0;
         inq_ary[11] <= 80'b0;
         inq_ary[12] <= 80'b0;
         inq_ary[13] <= 80'b0;
         inq_ary[14] <= 80'b0;
         inq_ary[15] <= 80'b0;
         inq_ary[16] <= 80'b0;
         inq_ary[17] <= 80'b0;
         inq_ary[18] <= 80'b0;
         inq_ary[19] <= 80'b0;
         inq_ary[20] <= 80'b0;
         inq_ary[21] <= 80'b0;
         inq_ary[22] <= 80'b0;
         inq_ary[23] <= 80'b0;
         inq_ary[24] <= 80'b0;
         inq_ary[25] <= 80'b0;
         inq_ary[26] <= 80'b0;
         inq_ary[27] <= 80'b0;
         inq_ary[28] <= 80'b0;
         inq_ary[29] <= 80'b0;
         inq_ary[30] <= 80'b0;
         inq_ary[31] <= 80'b0;
      end
      else
      begin
         if (wr_en)
         begin
            inq_ary[wr_adr] <= (din & write_mask) | (inq_ary[wr_adr] & ~write_mask);
         end
      end
   end

   // pipeline crap
   always @ (posedge rclk)
   begin
      // din_d1 <= din;
      rd_adr_d1 <= rd_adr;
      rd_adr_d2 <= rd_adr_d1;
      // wr_adr_d1 <= wr_adr;
      // wr_en_d1 <= wr_en;
      // word_wen_d1 <= word_wen;
      // byte_wen_d1 <= byte_wen;
      read_en_d1 <= rd_en;
      read_en_d2 <= read_en_d1;
   end

assign  write_mask[79:0] =
    {{4{nib_wr_en[19]}},
     {4{nib_wr_en[18]}},
     {4{nib_wr_en[17]}},
     {4{nib_wr_en[16]}},
     {4{nib_wr_en[15]}},
     {4{nib_wr_en[14]}},
     {4{nib_wr_en[13]}},
     {4{nib_wr_en[12]}},
     {4{nib_wr_en[11]}},
     {4{nib_wr_en[10]}},
     {4{nib_wr_en[9]}},
     {4{nib_wr_en[8]}},
     {4{nib_wr_en[7]}},
     {4{nib_wr_en[6]}},
     {4{nib_wr_en[5]}},
     {4{nib_wr_en[4]}},
     {4{nib_wr_en[3]}},
     {4{nib_wr_en[2]}},
     {4{nib_wr_en[1]}},
     {4{nib_wr_en[0]}}
    };

endmodule











// Modified by Princeton University on June 9th, 2015
// ========== Copyright Header Begin ==========================================
//
// OpenSPARC T1 Processor File: bw_r_dcd.v
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


// devices.xml


////////////////////////////////////////////////////////////////////////
/*
//  Module Name:
//  Description:  LSU Data Cache.
//      - Physically-Indexed Physically Tagged (PIPT)
//      - 8KB
//      - 4 way set-associative.
//      - 16B lines
//      - 2:1 column select by choosing either lower
//      or upper half of 16B line.
//      - Parity protected on a byte basis.
//      - Byte enables for byte-wide stores.
//
*/
////////////////////////////////////////////////////////////////////////
// Global header file includes
////////////////////////////////////////////////////////////////////////
//`include  "sys.h" // system level definition file which contains the
                  // time scale definition

//`include "iop.h"
//`include "fabric.h"











//PITON_PROTO enables all FPGA related modifications






////////////////////////////////////////////////////////////////////////
// Local header file includes / local defines
////////////////////////////////////////////////////////////////////////

module bw_r_dcd ( /*AUTOARG*/
   // Outputs
   so, dcache_rdata_wb, dcache_rparity_wb, dcache_rparity_err_wb,
   dcache_rdata_msb_m,
   // dcache_rdata_msb_w0_m, dcache_rdata_msb_w1_m,
   // dcache_rdata_msb_w2_m, dcache_rdata_msb_w3_m,
   dcd_fuse_repair_value, dcd_fuse_repair_en,
   // Inputs
   dcache_rd_addr_e, dcache_alt_addr_e, dcache_rvld_e, dcache_wvld_e,
   dcache_wdata_e, dcache_wr_rway_e, dcache_byte_wr_en_e,
   dcache_alt_rsel_way_e, dcache_rsel_way_wb, dcache_alt_mx_sel_e,
   si, se, sehold, rst_tri_en, arst_l, rclk, dcache_alt_data_w0_m,
   dcache_arry_data_sel_m, efc_spc_fuse_clk1, fuse_dcd_wren,
   fuse_dcd_rid, fuse_dcd_repair_value, fuse_dcd_repair_en,


   // sram wrapper interface
   sram_dcache_w01_rtap_data,
   sram_dcache_w23_rtap_data,
   rtap_srams_bist_command,
   rtap_srams_bist_data
   ) ;


output [4-1:0] sram_dcache_w01_rtap_data;
output [4-1:0] sram_dcache_w23_rtap_data;
input  [4-1:0] rtap_srams_bist_command;
input  [4-1:0] rtap_srams_bist_data;

input [(6 + 4):3]    dcache_rd_addr_e;     // read cache index [`L1D_ADDRESS_HI:4] + bit [3] offset
input [(6 + 4):3]    dcache_alt_addr_e;    // write/bist/diagnostic read cache index + offset

input           dcache_rvld_e;        // read accesses d$.
input           dcache_wvld_e;        // valid write setup to m-stage.

input [144-1:0]   dcache_wdata_e;       // write data - 16Bx8 + 8b parity.
input [2-1:0]     dcache_wr_rway_e;     // replacement way for load miss/store.
input [15:0]    dcache_byte_wr_en_e;  // 16b byte wr enable for stores.

input [2-1:0]     dcache_alt_rsel_way_e ; // bist/diagnostic read way select
input [2-1:0]     dcache_rsel_way_wb;     // load way select, connect to cache_way_hit
input           dcache_alt_mx_sel_e;

input           si;
input           se;
input           sehold;

output          so;

input   rst_tri_en ;

input           arst_l; // used for redundancy flops - do not reset on wrm reset.

input           rclk;

output reg [63:0]  dcache_rdata_wb;
output reg [7:0]   dcache_rparity_wb;
output reg         dcache_rparity_err_wb;

//=================================
//    dc_fill critical path
//=================================
   input [63:0] dcache_alt_data_w0_m; //from qdp1
   input        dcache_arry_data_sel_m;            //from dctl

   // output [7:0] dcache_rdata_msb_w0_m;    //to dcdp
   // output [7:0] dcache_rdata_msb_w1_m;    //to dcdp
   // output [7:0] dcache_rdata_msb_w2_m;    //to dcdp
   // output [7:0] dcache_rdata_msb_w3_m;    //to dcdp
  
  wire [7:0] dcache_rdata_msb_w0_m;    //to dcdp


  wire [7:0] dcache_rdata_msb_w1_m;    //to dcdp



   output [8*2-1:0] dcache_rdata_msb_m;    //to dcdp

//-----------------------------------------------------------------------------
// 32KB block fuse inputs
//-----------------------------------------------------------------------------
// efuse non ovl clks
input           efc_spc_fuse_clk1;

input           fuse_dcd_wren;          //redundancy register write enable, qualified
input [2:0]     fuse_dcd_rid;           //redundancy register id
input [7:0]     fuse_dcd_repair_value;  //data in for redundancy register
input [1:0]     fuse_dcd_repair_en;     //enable bits to turn on redundancy
output [7:0]    dcd_fuse_repair_value;  //data out for redundancy register
output [1:0]    dcd_fuse_repair_en;     //enable bits out

// Memory declaration.
wire  [(6 + 4):3]    dcache_rwaddr_e ;
wire  [(6 + 4):3]    dcache_raddr_e ;



// wire [`L1D_DATA_ENTRY_WIDTH-1:0]   temp_w0a;
// wire [`L1D_DATA_ENTRY_WIDTH-1:0]   temp_w1a;
// wire [`L1D_DATA_ENTRY_WIDTH-1:0]   temp_w2a;
// wire [`L1D_DATA_ENTRY_WIDTH-1:0]   temp_w3a;


  wire [144-1:0]   temp_w0a;


  wire [144-1:0]   temp_w1a;



// this particular macro contains 2 ways
// wire [`L1D_DATA_ENTRY_WIDTH-1:0] way_mask_e = { {8{dcache_byte_wr_en_e[0]}},
//                             {8{dcache_byte_wr_en_e[1]}},
//                             {8{dcache_byte_wr_en_e[2]}},
//                             {8{dcache_byte_wr_en_e[3]}},
//                             {8{dcache_byte_wr_en_e[4]}},
//                             {8{dcache_byte_wr_en_e[5]}},
//                             {8{dcache_byte_wr_en_e[6]}},
//                             {8{dcache_byte_wr_en_e[7]}},
//                             {8{dcache_byte_wr_en_e[8]}},
//                             {8{dcache_byte_wr_en_e[9]}},
//                             {8{dcache_byte_wr_en_e[10]}},
//                             {8{dcache_byte_wr_en_e[11]}},
//                             {8{dcache_byte_wr_en_e[12]}},
//                             {8{dcache_byte_wr_en_e[13]}},
//                             {8{dcache_byte_wr_en_e[14]}},
//                             {8{dcache_byte_wr_en_e[15]}}
//                           };

// Includes data(128b)+parity(16b).
wire [144-1:0] way_mask_e =
    {{8{dcache_byte_wr_en_e[15]}},{8{dcache_byte_wr_en_e[14]}},{8{dcache_byte_wr_en_e[13]}},
     {8{dcache_byte_wr_en_e[12]}},{8{dcache_byte_wr_en_e[11]}},{8{dcache_byte_wr_en_e[10]}},
     {8{dcache_byte_wr_en_e[9]}}, {8{dcache_byte_wr_en_e[8]}}, {8{dcache_byte_wr_en_e[7]}},
     {8{dcache_byte_wr_en_e[6]}}, {8{dcache_byte_wr_en_e[5]}}, {8{dcache_byte_wr_en_e[4]}},
     {8{dcache_byte_wr_en_e[3]}}, {8{dcache_byte_wr_en_e[2]}}, {8{dcache_byte_wr_en_e[1]}},
     {8{dcache_byte_wr_en_e[0]}}, dcache_byte_wr_en_e[15:0]} ;

// wire [`L1D_DATA_ENTRY_WIDTH-1:0] way0_write_mask = {144{dcache_wr_rway_e[0]}} & way_mask_e;
// wire [`L1D_DATA_ENTRY_WIDTH-1:0] way1_write_mask = {144{dcache_wr_rway_e[1]}} & way_mask_e;
// wire [`L1D_DATA_ENTRY_WIDTH-1:0] way2_write_mask = {144{dcache_wr_rway_e[2]}} & way_mask_e;
// wire [`L1D_DATA_ENTRY_WIDTH-1:0] way3_write_mask = {144{dcache_wr_rway_e[3]}} & way_mask_e;

// wire [`L1D_DATA_ENTRY_WIDTH-1:0]   temp_w0a_reg;
// wire [`L1D_DATA_ENTRY_WIDTH-1:0]   temp_w1a_reg;
// wire [`L1D_DATA_ENTRY_WIDTH-1:0]   temp_w2a_reg;
// wire [`L1D_DATA_ENTRY_WIDTH-1:0]   temp_w3a_reg;

  wire [144-1:0] way0_write_mask = {144{dcache_wr_rway_e[0]}} & way_mask_e;
  wire [144-1:0]   temp_w0a_reg;


  wire [144-1:0] way1_write_mask = {144{dcache_wr_rway_e[1]}} & way_mask_e;
  wire [144-1:0]   temp_w1a_reg;





// !MERGE_L1_DCACHE && SRAM_LSU_DCACHE


sram_l1d_data_piton dcache_way_01
(
  .MEMCLK(rclk),
  .RESET_N(arst_l),
  .CE(dcache_wvld_e | dcache_rvld_e),
  .A(dcache_rwaddr_e[(6 + 4):4]),
  .DIN({dcache_wdata_e, dcache_wdata_e}),
  .BW({way1_write_mask, way0_write_mask}),
  .RDWEN(~dcache_wvld_e),
  .DOUT({temp_w1a_reg, temp_w0a_reg}),

  .BIST_COMMAND(rtap_srams_bist_command),
  .BIST_DIN(rtap_srams_bist_data),
  .BIST_DOUT(sram_dcache_w01_rtap_data),
  .SRAMID(8'd3)
);

sram_l1d_data_piton dcache_way_23
(
  .MEMCLK(rclk),
  .RESET_N(arst_l),
  .CE(dcache_wvld_e | dcache_rvld_e),
  .A(dcache_rwaddr_e[(6 + 4):4]),
  .DIN({dcache_wdata_e, dcache_wdata_e}),
  .BW({way3_write_mask, way2_write_mask}),
  .RDWEN(~dcache_wvld_e),
  .DOUT({temp_w3a_reg, temp_w2a_reg}),

  .BIST_COMMAND(rtap_srams_bist_command),
  .BIST_DIN(rtap_srams_bist_data),
  .BIST_DOUT(sram_dcache_w23_rtap_data),
  .SRAMID(8'd4)
);










































 // MERGE_L1_DCACHE












































 // SRAM_LSU_DCACHE

reg [(6 + 4):3]    dcache_rwaddr_m ;
reg [(6 + 4):3]    dcache_raddr_m ;
reg           dcache_rvld_m ;
reg           wvld_m ;
reg [144-1:0]   dcache_wdata_m ;
reg [127:0]   rw_wdline ;
reg [2-1:0]     dcache_wr_rway_m ;

// reg [63:0]    dcache_rdata_w0_wb;   // way0 64b data.
// reg [63:0]    dcache_rdata_w1_wb;   // way1 64b data.
// reg [63:0]    dcache_rdata_w2_wb;   // way2 64b data.
// reg [63:0]    dcache_rdata_w3_wb;   // way3 64b data.
reg [15:0]    byte_wr_enable ;

reg           dcache_alt_mx_sel_m, dcache_alt_mx_sel_wb;
reg [2-1:0]     dcache_alt_rsel_way_m, dcache_alt_rsel_way_wb;


wire            dcache_wvld_m ;
// wire  [63:0]    dcache_rdata_w0_m;    // way0 64b data.
// wire  [63:0]    dcache_rdata_w1_m;    // way1 64b data.
// wire  [63:0]    dcache_rdata_w2_m;    // way2 64b data.
// wire  [63:0]    dcache_rdata_w3_m;    // way3 64b data.
// wire  [7:0]     dcache_rparity_w0_m;  // way0 8b parity.
// wire  [7:0]     dcache_rparity_w1_m;  // way1 8b parity.
// wire  [7:0]     dcache_rparity_w2_m;  // way2 8b parity.
// wire  [7:0]     dcache_rparity_w3_m;  // way3 8b parity.

   // wire [7:0]   rd_parity_err_w0_m;
   // wire [7:0]   rd_parity_err_w1_m;
   // wire [7:0]   rd_parity_err_w2_m;
   // wire [7:0]   rd_parity_err_w3_m;


wire  [144-1:0]   way_mask ;
wire  [144-1:0]   way_mask_inv ;

//calculated parity based on read-out data
// wire [7:0]  gen_dcache_parity_w0_m;
// wire [7:0]  gen_dcache_parity_w1_m;
// wire [7:0]  gen_dcache_parity_w2_m;
// wire [7:0]  gen_dcache_parity_w3_m;

   wire     clk;
   assign   clk = rclk;


    wire [7:0]  gen_dcache_parity_w0_m;
    wire [7:0]   rd_parity_err_w0_m;
    wire  [7:0]     dcache_rparity_w0_m;  // way0 8b parity.
    wire  [63:0]    dcache_rdata_w0_m;    // way0 64b data.
    reg [63:0]    dcache_rdata_w0_wb;   // way0 64b data.


    wire [7:0]  gen_dcache_parity_w1_m;
    wire [7:0]   rd_parity_err_w1_m;
    wire  [7:0]     dcache_rparity_w1_m;  // way1 8b parity.
    wire  [63:0]    dcache_rdata_w1_m;    // way1 64b data.
    reg [63:0]    dcache_rdata_w1_wb;   // way1 64b data.



//=========================================================================================
//  Staging
//=========================================================================================

// BIST Rd used fill address port.
assign  dcache_rwaddr_e[(6 + 4):3] =
  (dcache_alt_mx_sel_e) ? dcache_alt_addr_e[(6 + 4):3] : dcache_rd_addr_e[(6 + 4):3] ;

assign  dcache_raddr_e[(6 + 4):3] =
  (dcache_alt_mx_sel_e) ? dcache_alt_addr_e[(6 + 4):3] : dcache_rd_addr_e[(6 + 4):3] ;

always @(posedge clk)
  begin
    dcache_alt_mx_sel_m   <= sehold ? dcache_alt_mx_sel_m : dcache_alt_mx_sel_e;

    dcache_alt_rsel_way_m <= sehold ? dcache_alt_rsel_way_m : dcache_alt_rsel_way_e;

    dcache_rwaddr_m[(6 + 4):3] <= sehold ? dcache_rwaddr_m[(6 + 4):3] : dcache_rwaddr_e[(6 + 4):3] ;

    dcache_raddr_m[(6 + 4):3] <= sehold ? dcache_raddr_m[(6 + 4):3] : dcache_raddr_e[(6 + 4):3] ;

    dcache_rvld_m         <= sehold ? dcache_rvld_m  : dcache_rvld_e ;

    wvld_m                <= sehold ?  wvld_m : dcache_wvld_e ;

    dcache_wdata_m[144-1:0] <= sehold ?  dcache_wdata_m[144-1:0] : dcache_wdata_e[144-1:0] ;

    dcache_wr_rway_m[2-1:0] <= sehold ?  dcache_wr_rway_m[2-1:0] : dcache_wr_rway_e[2-1:0] ;

    byte_wr_enable[15:0]  <= sehold ? byte_wr_enable[15:0] : dcache_byte_wr_en_e[15:0] ;

  end

always @ (posedge clk)
  begin
// JC modified begin
//    dcache_alt_mx_sel_wb <= dcache_alt_mx_sel_m;
//    dcache_alt_rsel_way_wb  <= dcache_alt_rsel_way_m;
    dcache_alt_mx_sel_wb <= sehold ? dcache_alt_mx_sel_wb :dcache_alt_mx_sel_m;
    dcache_alt_rsel_way_wb  <= sehold ? dcache_alt_rsel_way_wb :dcache_alt_rsel_way_m;
// JC modified end
  end

assign  dcache_wvld_m = wvld_m & ~rst_tri_en ;




















 // `ifndef SRAM_LSU_DCACHE

//removed stablizer, zero out without read (for IBM sram too)
// assign  temp_w0a[`L1D_DATA_ENTRY_WIDTH-1:0] = dcache_rvld_m? temp_w0a_reg[`L1D_DATA_ENTRY_WIDTH-1:0]: 144'b0;
// assign  temp_w1a[`L1D_DATA_ENTRY_WIDTH-1:0] = dcache_rvld_m? temp_w1a_reg[`L1D_DATA_ENTRY_WIDTH-1:0]: 144'b0;
// assign  temp_w2a[`L1D_DATA_ENTRY_WIDTH-1:0] = dcache_rvld_m? temp_w2a_reg[`L1D_DATA_ENTRY_WIDTH-1:0]: 144'b0;
// assign  temp_w3a[`L1D_DATA_ENTRY_WIDTH-1:0] = dcache_rvld_m? temp_w3a_reg[`L1D_DATA_ENTRY_WIDTH-1:0]: 144'b0;


  assign  temp_w0a[144-1:0] = dcache_rvld_m? temp_w0a_reg[144-1:0]: 144'b0;


  assign  temp_w1a[144-1:0] = dcache_rvld_m? temp_w1a_reg[144-1:0]: 144'b0;



 // `ifndef SRAM_LSU_DCACHE

// Prior to SA, column mux (64(D)+8(P))x4 bits. Assume parity is
// at the end of the 144b line. Entry is wX||Parity

// // Select either upper or lower 64b from each of the 4 ways.
// assign  dcache_rdata_w0_m[63:0] = ~dcache_rwaddr_m[3] ? temp_w0a[`L1D_DATA_ENTRY_WIDTH-1:80] : temp_w0a[79:16] ;
// assign  dcache_rdata_w1_m[63:0] = ~dcache_rwaddr_m[3] ? temp_w1a[`L1D_DATA_ENTRY_WIDTH-1:80] : temp_w1a[79:16] ;
// assign  dcache_rdata_w2_m[63:0] = ~dcache_rwaddr_m[3] ? temp_w2a[`L1D_DATA_ENTRY_WIDTH-1:80] : temp_w2a[79:16] ;
// assign  dcache_rdata_w3_m[63:0] = ~dcache_rwaddr_m[3] ? temp_w3a[`L1D_DATA_ENTRY_WIDTH-1:80] : temp_w3a[79:16] ;


  assign  dcache_rdata_w0_m[63:0] = ~dcache_rwaddr_m[3] ? temp_w0a[144-1:80] : temp_w0a[79:16] ;


  assign  dcache_rdata_w1_m[63:0] = ~dcache_rwaddr_m[3] ? temp_w1a[144-1:80] : temp_w1a[79:16] ;



   wire [7:0] dcache_msb_w0_m;
   wire [7:0] dcache_alt_data_w0_msb_m;

//MSB sent out to dcdp in M stage
assign dcache_msb_w0_m[7:0]=
    {dcache_rdata_w0_m[63],
     dcache_rdata_w0_m[55],
     dcache_rdata_w0_m[47],
     dcache_rdata_w0_m[39],
     dcache_rdata_w0_m[31],
     dcache_rdata_w0_m[23],
     dcache_rdata_w0_m[15],
     dcache_rdata_w0_m[07]} ;

assign dcache_alt_data_w0_msb_m [7:0]=
    {dcache_alt_data_w0_m[63],
     dcache_alt_data_w0_m[55],
     dcache_alt_data_w0_m[47],
     dcache_alt_data_w0_m[39],
     dcache_alt_data_w0_m[31],
     dcache_alt_data_w0_m[23],
     dcache_alt_data_w0_m[15],
     dcache_alt_data_w0_m[07]} ;

//2-to-1 mux
assign dcache_rdata_msb_w0_m[7:0] = dcache_arry_data_sel_m ?
                                    dcache_msb_w0_m[7:0] :
                                    dcache_alt_data_w0_msb_m[7:0];

// assign dcache_rdata_msb_w1_m[7:0]=
//     {dcache_rdata_w1_m[63],
//      dcache_rdata_w1_m[55],
//      dcache_rdata_w1_m[47],
//      dcache_rdata_w1_m[39],
//      dcache_rdata_w1_m[31],
//      dcache_rdata_w1_m[23],
//      dcache_rdata_w1_m[15],
//      dcache_rdata_w1_m[07]} ;

// assign dcache_rdata_msb_w2_m[7:0]=
//     {dcache_rdata_w2_m[63],
//      dcache_rdata_w2_m[55],
//      dcache_rdata_w2_m[47],
//      dcache_rdata_w2_m[39],
//      dcache_rdata_w2_m[31],
//      dcache_rdata_w2_m[23],
//      dcache_rdata_w2_m[15],
//      dcache_rdata_w2_m[07]} ;

// assign dcache_rdata_msb_w3_m[7:0]=
//     {dcache_rdata_w3_m[63],
//      dcache_rdata_w3_m[55],
//      dcache_rdata_w3_m[47],
//      dcache_rdata_w3_m[39],
//      dcache_rdata_w3_m[31],
//      dcache_rdata_w3_m[23],
//      dcache_rdata_w3_m[15],
//      dcache_rdata_w3_m[07]} ;

// assign dcache_rdata_msb_w3_m[7:0]=
//     {dcache_rdata_w3_m[63],
//      dcache_rdata_w3_m[55],
//      dcache_rdata_w3_m[47],
//      dcache_rdata_w3_m[39],
//      dcache_rdata_w3_m[31],
//      dcache_rdata_w3_m[23],
//      dcache_rdata_w3_m[15],
//      dcache_rdata_w3_m[07]} ;


  assign dcache_rdata_msb_w1_m[7:0]=
    {dcache_rdata_w1_m[63],
     dcache_rdata_w1_m[55],
     dcache_rdata_w1_m[47],
     dcache_rdata_w1_m[39],
     dcache_rdata_w1_m[31],
     dcache_rdata_w1_m[23],
     dcache_rdata_w1_m[15],
     dcache_rdata_w1_m[07]} ;



assign dcache_rdata_msb_m[(8*(0+1))-1 -: 8] = dcache_rdata_msb_w0_m;
assign dcache_rdata_msb_m[(8*(1+1))-1 -: 8] = dcache_rdata_msb_w1_m;


   // wire [63:0] rdata_w0_m;
   // wire [63:0] rdata_w1_m;
   // wire [63:0] rdata_w2_m;
   // wire [63:0] rdata_w3_m;
wire [63:0] rdata_w0_m;
wire [63:0] rdata_w1_m;


//2-to-1 mux
//dcache_alt_mx_sel default 0001 (way 0) when not in MBIST mode (logic in qdp2)
assign rdata_w0_m[63:0] = dcache_arry_data_sel_m ?
                          dcache_rdata_w0_m[63:0] : dcache_alt_data_w0_m[63:0];

// //assign rdata_w0_m[63:0] = dcache_rdata_w0_m[63:0];
// assign rdata_w1_m[63:0] = dcache_rdata_w1_m[63:0];
// assign rdata_w2_m[63:0] = dcache_rdata_w2_m[63:0];
// assign rdata_w3_m[63:0] = dcache_rdata_w3_m[63:0];
   

  assign rdata_w1_m[63:0] = dcache_rdata_w1_m[63:0];



// // Select upper half or lower half of parity.
// assign  dcache_rparity_w0_m[7:0] = ~dcache_rwaddr_m[3] ? temp_w0a[15:8] : temp_w0a[7:0] ;
// assign  dcache_rparity_w1_m[7:0] = ~dcache_rwaddr_m[3] ? temp_w1a[15:8] : temp_w1a[7:0] ;
// assign  dcache_rparity_w2_m[7:0] = ~dcache_rwaddr_m[3] ? temp_w2a[15:8] : temp_w2a[7:0] ;
// assign  dcache_rparity_w3_m[7:0] = ~dcache_rwaddr_m[3] ? temp_w3a[15:8] : temp_w3a[7:0] ;

//    reg [7:0] dcache_rparity_w0_wb;
//    reg [7:0] dcache_rparity_w1_wb;
//    reg [7:0] dcache_rparity_w2_wb;
//    reg [7:0] dcache_rparity_w3_wb;

//    reg [7:0] rd_parity_err_w0_wb;
//    reg [7:0] rd_parity_err_w1_wb;
//    reg [7:0] rd_parity_err_w2_wb;
//    reg [7:0] rd_parity_err_w3_wb;


  assign  dcache_rparity_w0_m[7:0] = ~dcache_rwaddr_m[3] ? temp_w0a[15:8] : temp_w0a[7:0] ;
  reg [7:0] dcache_rparity_w0_wb;
  reg [7:0] rd_parity_err_w0_wb;


  assign  dcache_rparity_w1_m[7:0] = ~dcache_rwaddr_m[3] ? temp_w1a[15:8] : temp_w1a[7:0] ;
  reg [7:0] dcache_rparity_w1_wb;
  reg [7:0] rd_parity_err_w1_wb;




// Stage to WB
always  @(posedge clk)
  begin
    // dcache_rdata_w0_wb[63:0] <= rdata_w0_m[63:0] ;
    // dcache_rdata_w1_wb[63:0] <= rdata_w1_m[63:0] ;
    // dcache_rdata_w2_wb[63:0] <= rdata_w2_m[63:0] ;
    // dcache_rdata_w3_wb[63:0] <= rdata_w3_m[63:0] ;

    // dcache_rparity_w0_wb[7:0] <=  dcache_rparity_w0_m[7:0];
    // dcache_rparity_w1_wb[7:0] <=  dcache_rparity_w1_m[7:0];
    // dcache_rparity_w2_wb[7:0] <=  dcache_rparity_w2_m[7:0];
    // dcache_rparity_w3_wb[7:0] <=  dcache_rparity_w3_m[7:0];

    // rd_parity_err_w0_wb [7:0] <= rd_parity_err_w0_m[7:0];
    // rd_parity_err_w1_wb [7:0] <= rd_parity_err_w1_m[7:0];
    // rd_parity_err_w2_wb [7:0] <= rd_parity_err_w2_m[7:0];
    // rd_parity_err_w3_wb [7:0] <= rd_parity_err_w3_m[7:0];


  dcache_rdata_w0_wb[63:0] <= rdata_w0_m[63:0] ;
  dcache_rparity_w0_wb[7:0] <=  dcache_rparity_w0_m[7:0];
  rd_parity_err_w0_wb [7:0] <= rd_parity_err_w0_m[7:0];


  dcache_rdata_w1_wb[63:0] <= rdata_w1_m[63:0] ;
  dcache_rparity_w1_wb[7:0] <=  dcache_rparity_w1_m[7:0];
  rd_parity_err_w1_wb [7:0] <= rd_parity_err_w1_m[7:0];



  end

//parity calculation and check are done in M stage for 4 way data
   // wire rd_parity_err_w0;
   // wire rd_parity_err_w1;
   // wire rd_parity_err_w2;
   // wire rd_parity_err_w3;

// lsu_dc_parity_gen #(8,8)  parity_gen_w0 (
//                 .data_in        (dcache_rdata_w0_m[63:0]),
//                 .parity_out     (gen_dcache_parity_w0_m[7:0])
//         );

// assign rd_parity_err_w0_m[7:0] = dcache_rvld_m ? (dcache_rparity_w0_m[7:0] ^ gen_dcache_parity_w0_m[7:0]) :
//                                                   8'hff;


// lsu_dc_parity_gen #(8,8)  parity_gen_w1 (
//                 .data_in        (dcache_rdata_w1_m[63:0]),
//                 .parity_out     (gen_dcache_parity_w1_m[7:0])
//         );

// assign rd_parity_err_w1_m[7:0] = dcache_rvld_m ? (dcache_rparity_w1_m[7:0] ^ gen_dcache_parity_w1_m[7:0]) :
//                                                  8'hff;

// lsu_dc_parity_gen #(8,8)  parity_gen_w2 (
//                 .data_in        (dcache_rdata_w2_m[63:0]),
//                 .parity_out     (gen_dcache_parity_w2_m[7:0])
//         );
// assign rd_parity_err_w2_m[7:0] = dcache_rvld_m ? (dcache_rparity_w2_m[7:0] ^ gen_dcache_parity_w2_m[7:0])  :
//                                                    8'hff;

// lsu_dc_parity_gen #(8,8)  parity_gen_w3 (
//                 .data_in        (dcache_rdata_w3_m[63:0]),
//                 .parity_out     (gen_dcache_parity_w3_m[7:0])
//         );
// assign rd_parity_err_w3_m[7:0] =  dcache_rvld_m ? (dcache_rparity_w3_m[7:0] ^ gen_dcache_parity_w3_m[7:0]) :
//                                                   8'hff;


  wire rd_parity_err_w0;
  lsu_dc_parity_gen #(8,8)  parity_gen_w0 (
                  .data_in        (dcache_rdata_w0_m[63:0]),
                  .parity_out     (gen_dcache_parity_w0_m[7:0])
          );
  assign rd_parity_err_w0_m[7:0] = dcache_rvld_m ? (dcache_rparity_w0_m[7:0] ^ gen_dcache_parity_w0_m[7:0])  :
                                                     8'hff;


  wire rd_parity_err_w1;
  lsu_dc_parity_gen #(8,8)  parity_gen_w1 (
                  .data_in        (dcache_rdata_w1_m[63:0]),
                  .parity_out     (gen_dcache_parity_w1_m[7:0])
          );
  assign rd_parity_err_w1_m[7:0] = dcache_rvld_m ? (dcache_rparity_w1_m[7:0] ^ gen_dcache_parity_w1_m[7:0])  :
                                                     8'hff;



// way select mux on READ
// Select one of four ways from indexed cache set.

wire [2-1:0] dcache_rd_sel_way_wb;
assign dcache_rd_sel_way_wb[2-1:0] = dcache_alt_mx_sel_wb ? dcache_alt_rsel_way_wb[2-1:0] :
                                                          dcache_rsel_way_wb[2-1:0];

// assign  dcache_rdata_wb[63:0]  =
//  (dcache_rd_sel_way_wb[0] ? dcache_rdata_w0_wb[63:0] : 64'b0) |
//  (dcache_rd_sel_way_wb[1] ? dcache_rdata_w1_wb[63:0] : 64'b0) |
//  (dcache_rd_sel_way_wb[2] ? dcache_rdata_w2_wb[63:0] : 64'b0) |
//  (dcache_rd_sel_way_wb[3] ? dcache_rdata_w3_wb[63:0] : 64'b0);


always @ *
begin
dcache_rdata_wb[63:0] = 0;
if (dcache_rd_sel_way_wb[0])
   dcache_rdata_wb[63:0] = dcache_rdata_wb[63:0] | dcache_rdata_w0_wb[63:0];
if (dcache_rd_sel_way_wb[1])
   dcache_rdata_wb[63:0] = dcache_rdata_wb[63:0] | dcache_rdata_w1_wb[63:0];
end


//parity err in W-stage, cache_way_hit may not be one-hot
// assign rd_parity_err_w0 =   |(rd_parity_err_w0_wb[7:0]);
// assign rd_parity_err_w1 =   |(rd_parity_err_w1_wb[7:0]);
// assign rd_parity_err_w2 =   |(rd_parity_err_w2_wb[7:0]);
// assign rd_parity_err_w3 =   |(rd_parity_err_w3_wb[7:0]);


  assign rd_parity_err_w0 =   |(rd_parity_err_w0_wb[7:0]);


  assign rd_parity_err_w1 =   |(rd_parity_err_w1_wb[7:0]);



// assign dcache_rparity_err_wb = rd_parity_err_w3 & dcache_rd_sel_way_wb[3] |
//                                rd_parity_err_w2 & dcache_rd_sel_way_wb[2] |
//                                rd_parity_err_w1 & dcache_rd_sel_way_wb[1] |
//                                rd_parity_err_w0 & dcache_rd_sel_way_wb[0] ;

always @ *
begin
dcache_rparity_err_wb = 0;
if (dcache_rd_sel_way_wb[0])
   dcache_rparity_err_wb = dcache_rparity_err_wb | rd_parity_err_w0;
if (dcache_rd_sel_way_wb[1])
   dcache_rparity_err_wb = dcache_rparity_err_wb | rd_parity_err_w1;
end


//mux4ds #(64) dcache_rdata_wb_mx (
//  .in0  (dcache_rdata_w0_wb[63:0]),
//  .in1  (dcache_rdata_w1_wb[63:0]),
//  .in2  (dcache_rdata_w2_wb[63:0]),
//  .in3  (dcache_rdata_w3_wb[63:0]),
//  .sel0 (dcache_rd_sel_way_wb[0]),
//  .sel1 (dcache_rd_sel_way_wb[1]),
//  .sel2 (dcache_rd_sel_way_wb[2]),
//  .sel3 (dcache_rd_sel_way_wb[3]),
//  .dout (dcache_rdata_wb[63:0])
//);

// dcache_rparity_wb only used by MBIST
//mux4ds #(8) dcache_rparity_wb_mx (
//  .in0 (dcache_rparity_w0_wb[7:0]),
//  .in1 (dcache_rparity_w1_wb[7:0]),
//  .in2 (dcache_rparity_w2_wb[7:0]),
//  .in3 (dcache_rparity_w3_wb[7:0]),
//  .sel0(dcache_alt_rsel_way_wb[0]),
//  .sel1(dcache_alt_rsel_way_wb[1]),
//  .sel2(dcache_alt_rsel_way_wb[2]),
//  .sel3(dcache_alt_rsel_way_wb[3]),
//  .dout(dcache_rparity_wb[7:0])
//);

// assign  dcache_rparity_wb[7:0] =
// ( dcache_rd_sel_way_wb[0] ? dcache_rparity_w0_wb[7:0] : 8'b0 ) |
// ( dcache_rd_sel_way_wb[1] ? dcache_rparity_w1_wb[7:0] : 8'b0 ) |
// ( dcache_rd_sel_way_wb[2] ? dcache_rparity_w2_wb[7:0] : 8'b0 ) |
// ( dcache_rd_sel_way_wb[3] ? dcache_rparity_w3_wb[7:0] : 8'b0 ) ;
always @ *
begin
dcache_rparity_wb[7:0] = 0;
if (dcache_rd_sel_way_wb[0])
   dcache_rparity_wb[7:0] = dcache_rparity_wb[7:0] | dcache_rparity_w0_wb[7:0];
if (dcache_rd_sel_way_wb[1])
   dcache_rparity_wb[7:0] = dcache_rparity_wb[7:0] | dcache_rparity_w1_wb[7:0];
end













































 // `ifndef SRAM_LSU_DCACHE // FPGA

endmodule









// Modified by Princeton University on June 9th, 2015
// ========== Copyright Header Begin ==========================================
//
// OpenSPARC T1 Processor File: bw_r_dct.v
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
 //  Module Name:  bw_r_dct.v
 //  Description:
 //    Contains the RTL for the icache and dcache tag blocks.
 //    This is a 1RW 512 entry X 33b macro, with 132b rd and 132b wr,
 //    broken into 4 33b segments with its own write enable.
 //    Address and Control inputs are available the stage before
 //    array access, which is referred to as "_x".  Write data is
 //    available in the same stage as the write to the ram, referred
 //    to as "_y".  Read data is also read out and available in "_y".
 //
 //            X       |      Y
 //     index          |  ram access
 //     index sel      |  write_tag
 //     rd/wr req      |     -> read_tag
 //     way enable     |
 */

// devices.xml



////////////////////////////////////////////////////////////////////////
// Local header file includes / local defines
////////////////////////////////////////////////////////////////////////









//PITON_PROTO enables all FPGA related modifications






































































































































































module bw_r_dct(
  // rdtag_w0_y, rdtag_w1_y, rdtag_w2_y, 
  rdtag_y, 
  // rdtag_w3_y, 
  so, rclk, se,
  si, reset_l, sehold, rst_tri_en, index0_x, index1_x, index_sel_x,
  dec_wrway_x, rdreq_x, wrreq_x, 

  wrtag_x, wrtag_y,
  // wrtag_w0_y, wrtag_w1_y, wrtag_w2_y,
  // wrtag_w3_y, wrtag_w0_x, wrtag_w1_x, wrtag_w2_x,
  // wrtag_w3_x, 
  adj,

  // sram wrapper interface
  sramid,
  srams_rtap_data,
  rtap_srams_bist_command,
  rtap_srams_bist_data
  );

  input     rclk;
  input     se;
  input     si;
  input     reset_l;
  input     sehold;
  input     rst_tri_en;
  input [6:0]   index0_x;
  input [6:0]   index1_x;
  input     index_sel_x;
  input [2-1:0]   dec_wrway_x;
  input     rdreq_x;
  input     wrreq_x;

  input [33-1:0]    wrtag_y;
  input [33-1:0]    wrtag_x;
  input [2-1:0]   adj;


  // sram wrapper interface
  output [4-1:0] srams_rtap_data;
  input  [4-1:0] rtap_srams_bist_command;
  input  [4-1:0] rtap_srams_bist_data;
  input  [8-1:0] sramid;

  output  [(33*2)-1:0]    rdtag_y;
  output      so;

  wire      clk;
  wire  [6:0]   index_x;
  reg   [6:0]   index_y;
  wire  [2-1:0]   we;
  reg           wrreq_y;
  reg           rdreq_y;

  reg [131:0]  rdtag_sa_y; //for error_inject XMR

  assign clk = rclk;
  assign index_x = (index_sel_x ? index1_x : index0_x);
  assign we = ({2 {((wrreq_x & reset_l) & (~rst_tri_en))}} & dec_wrway_x);

  // wire [131:0] write_bus_x = {wrtag_x,wrtag_x,wrtag_x,wrtag_x};
  // wire [131:0] write_bus_mask_x = {{33{we[3]}}, {33{we[2]}}, {33{we[1]}}, {33{we[0]}}};

  wire [33*2-1:0] write_bus_x = {2{wrtag_x}};
  wire [33*2-1:0] write_bus_mask_x;
  
  assign write_bus_mask_x[33*(0+1)-1 -: 33] = {33{we[0]}};


  assign write_bus_mask_x[33*(1+1)-1 -: 33] = {33{we[1]}};




  always @ (posedge rclk)
  begin
    index_y <= index_x;
    wrreq_y <= wrreq_x;
    rdreq_y <= rdreq_x;
  end

  wire  [(33*2)-1:0]    rdtag_y;

sram_l1d_tag cache
(
  .MEMCLK(rclk),
    .RESET_N(reset_l),
  .CE(wrreq_x | rdreq_x),
  .A(index_x),
  .DIN(write_bus_x),
  .BW(write_bus_mask_x),
  .RDWEN(~wrreq_x),
  .DOUT(rdtag_y),

  // .BIST_COMMAND(rtap_srams_bist_command),
  // .BIST_DIN(rtap_srams_bist_data),
  // .BIST_DOUT(sram_dcache_w01_rtap_data),
  // .SRAMID(`BIST_ID_L1_DCACHE_W01)
  .BIST_COMMAND(rtap_srams_bist_command),
  .BIST_DIN(rtap_srams_bist_data),
  .BIST_DOUT(srams_rtap_data),
  .SRAMID(sramid)
);

  // assign rdtag_y = {rdtag_w3_y, rdtag_w2_y, rdtag_w1_y, rdtag_w0_y};




endmodule

 // IBM TAG









// Modified by Princeton University on June 9th, 2015
// ========== Copyright Header Begin ==========================================
// 
// OpenSPARC T1 Processor File: bw_r_dtlb.v
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
//	Description:	Common TLB for Instruction Fetch and Load/Stores
*/

// devices.xml


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







































































































































































































































































































































































































































































































































































































//PITON_PROTO enables all FPGA related modifications








//this macro will not be used




























































































































































































































































































































































































































































































































































































































































































































































































































































































































































































module bw_r_dtlb ( /*AUTOARG*/
   // Outputs

   
   tlb_rd_tte_csm, tlb_rd_tte_csm_crit,
   
   tlb_rd_tte_tag, tlb_rd_tte_data, tlb_pgnum, tlb_pgnum_crit, 
   tlb_cam_hit, cache_way_hit, cache_hit, so, 
   // Inputs

   
   tlb_wr_tte_csm_thrid, tlb_wr_tte_csm, tlb_rd_csm_vld, tlb_wr_csm_sel,

   cfg_csm_tlb_state, cfg_csm_tlb_chipid,cfg_csm_tlb_x,
   cfg_csm_tlb_y, cfg_csm_tlb_hdid,cfg_csm_tlb_hd_size,
   cfg_csm_tlb_sdid,cfg_csm_tlb_lsid,
   


   tlb_cam_vld, tlb_cam_key, tlb_cam_pid,  
   tlb_demap_key, tlb_addr_mask_l, tlb_ctxt, 
   tlb_wr_vld, tlb_wr_tte_tag, tlb_wr_tte_data, tlb_rd_tag_vld, 
   tlb_rd_data_vld, tlb_rw_index, tlb_rw_index_vld, tlb_demap, 
   tlb_demap_auto, tlb_demap_all, cache_ptag, 
   // cache_ptag_w0, cache_ptag_w1, 
   // cache_ptag_w2, cache_ptag_w3, 
   cache_set_vld, tlb_bypass_va, 
   tlb_bypass, se, si, hold, adj, arst_l, rst_soft_l, rclk,
   rst_tri_en
   ) ;	


input			tlb_cam_vld ;		// ld/st requires xlation. 
input	[40:0]		tlb_cam_key ;		// cam data for loads/stores;includes vld 
						// CHANGE : add real bit for cam.
input	[2:0]		tlb_cam_pid ;		// NEW: pid for cam. 
input	[40:0]		tlb_demap_key ;		// cam data for demap; includes vlds. 
						// CHANGE : add real bit for demap
input			tlb_addr_mask_l ;	// address masking occurs
input	[12:0]		tlb_ctxt ;		// context for cam xslate/demap. 
input			tlb_wr_vld;		// write to tlb. 
input	[58:0]		tlb_wr_tte_tag;		// CHANGE:tte tag to be written (55+4-1)
						// R(+1b),PID(+3b),G(-1b). 
input	[42:0]		tlb_wr_tte_data;	// tte data to be written.
						// No change(!!!) - G bit becomes spare
input			tlb_rd_tag_vld ;	// read tag
input			tlb_rd_data_vld ;	// read data
input	[5:0]		tlb_rw_index ;		// index to read/write tlb.
input			tlb_rw_index_vld ;	// indexed write else use algorithm.
input			tlb_demap ;		// demap : page/ctxt/all/auto.  
input			tlb_demap_auto ;	// demap is of type auto 
input			tlb_demap_all;		// demap-all operation : encoded separately.
input    [(33*2)+1-1:0]      cache_ptag;          // way3 30b(D)/29b(I) tag.
// input  	[29:0]    	cache_ptag_w0;       	// way1 30b(D)/29b(I) tag.
// input  	[29:0]    	cache_ptag_w1;       	// way2 30b(D)/29b(I) tag.
// input  	[29:0]     	cache_ptag_w2;       	// way0 30b(D)/29b(I) tag.
// input  	[29:0]     	cache_ptag_w3;       	// way3 30b(D)/29b(I) tag.
input	[2-1:0]		cache_set_vld;       	// set vld-4 ways
input	[12:10]		tlb_bypass_va;	   	// bypass va.other va bits from cam-data
input			tlb_bypass;		// bypass tlb xslation

input			se ;			// scan-enable ; unused
input			si ;			// scan data in ; unused
input			hold ;			// scan hold signal
input	[7:0]		adj ;			// self-time adjustment ; unused
input			arst_l ;		// synchronous for tlb ; unused	
input			rst_soft_l ;		// software reset - asi
input			rclk;
input			rst_tri_en ;

// input    [29:0]      cache_ptag_w0;          // way1 30b(D)/29b(I) tag.
// input    [29:0]      cache_ptag_w1;          // way2 30b(D)/29b(I) tag.
// input    [29:0]      cache_ptag_w2;          // way0 30b(D)/29b(I) tag.
// input    [29:0]      cache_ptag_w3;          // way3 30b(D)/29b(I) tag.

wire cache_ptag_va_10 = cache_ptag[(33*2)];


wire [33-1:0] cache_ptag_w0_tmp = cache_ptag[33*(0+1)-1 -: 33];
wire [(29+1)-1:0] cache_ptag_w0 = {cache_ptag_w0_tmp[(29+1)-2:0],cache_ptag_va_10};


wire [33-1:0] cache_ptag_w1_tmp = cache_ptag[33*(1+1)-1 -: 33];
wire [(29+1)-1:0] cache_ptag_w1 = {cache_ptag_w1_tmp[(29+1)-2:0],cache_ptag_va_10};




input [2-1:0]   cfg_csm_tlb_state;
input [14-1:0]      cfg_csm_tlb_chipid;
input [8-1:0]           cfg_csm_tlb_x;
input [8-1:0]           cfg_csm_tlb_y;
input [10-1:0]        cfg_csm_tlb_hdid;
input [6-1:0]        cfg_csm_tlb_hd_size;
input [10-1:0]        cfg_csm_tlb_sdid;
input [6-1:0]        cfg_csm_tlb_lsid;


input [1:0]         tlb_wr_tte_csm_thrid;
input [32:0]    tlb_wr_tte_csm;
input               tlb_rd_csm_vld;
input               tlb_wr_csm_sel;
output [32:0]   tlb_rd_tte_csm;
output [32:0]   tlb_rd_tte_csm_crit;



output	[58:0]		tlb_rd_tte_tag;		// CHANGE: tte tag read from tlb.
output	[42:0]		tlb_rd_tte_data;	// tte data read from tlb.
// Need two ports for tlb_pgnum - critical and non-critical.
output	[39:10]		tlb_pgnum ;		// bypass or xslated pgnum
output	[39:10]		tlb_pgnum_crit ;	// bypass or xslated pgnum - critical
output			tlb_cam_hit ;		// xlation hits in tlb.
output	[2-1:0]		cache_way_hit;		// tag comparison results.
output			cache_hit;		// tag comparison result - 'or' of above.

//output			tlb_writeable ;		// tlb can be written in current cycle.

output			so ;		// scan data out ; unused

wire	[53:0]		tlb_cam_data ;
wire	[58:0]		wr_tte_tag ;	// CHANGE
wire	[42:0]		wr_tte_data ;
wire	[29:3]		phy_pgnum_m;
wire	[29:0]		pgnum_m;
wire 	[16-1:0]		used ;
wire			tlb_not_writeable ;
wire	[40:25] 	tlb_cam_key_masked ;
wire	[26:0]		tlb_cam_comp_key ;
wire			cam_vld ;
wire			demap_other ;
wire	[2-1:0]   	cache_way_hit ;

reg			tlb_not_writeable_d1 ;
reg			tlb_writeable ;
reg	[58:0]		tte_tag_ram  [16-1:0] ;	// CHANGE
reg	[42:0]		tte_data_ram [16-1:0] ;
reg	[16-1:0]		tlb_entry_vld ;
reg	[16-1:0]		tlb_entry_locked ;
reg	[16-1:0]		ademap_hit ;
reg	[58:0]		rd_tte_tag ;	// CHANGE
reg	[42:0]		rd_tte_data ;	
reg	[58:0]		tlb_rd_tte_tag ; // CHANGE	
reg	[42:0]		tlb_rd_tte_data ;	
reg			cam_vld_tmp ;
reg	[2:0]		cam_pid ;
reg	[53:0]		cam_data ;
reg			demap_auto, demap_other_tmp, demap_all ;
reg	[16-1:0]		mismatch ;
reg	[16-1:0]		cam_hit ;
reg	[16-1:0]		demap_hit ;
reg	[16-1:0]		demap_all_but_locked_hit ;
reg	[16-1:0]		mismatch_va_b47_28 ;
reg	[16-1:0]		mismatch_va_b27_22 ;
reg	[16-1:0]		mismatch_va_b21_16 ;
reg	[16-1:0]		mismatch_va_b15_13 ;
reg	[16-1:0]		mismatch_ctxt ;
reg	[16-1:0]		mismatch_pid ;
reg	[58:0]		tag ;	// CHANGE
reg	[16-1:0]		rw_wdline ;
reg	[16-1:0]		tlb_entry_used ;
reg	[16-1:0]		tlb_entry_replace ;
reg	[16-1:0]		tlb_entry_replace_d2 ;
reg	[29:0]		pgnum_g ;
reg     [2-1:0]		cache_set_vld_g;
// reg	[29:0]		cache_ptag_w0_g,cache_ptag_w1_g;
// reg	[29:0]		cache_ptag_w2_g,cache_ptag_w3_g;
reg [29:0]      cache_ptag_w0_g;
reg [29:0]      cache_ptag_w1_g;

reg			wr_vld_tmp;
reg			rd_tag; 
reg			rd_data;
reg			rw_index_vld;
reg	[4-1:0]		rw_index;
reg	[16-1:0]		sat ;



wire	[32:0]		wr_tte_csm ;
reg	    [32:0]		tte_csm_buf [1:0];
reg	    [32:0]		tte_csm_ram [16-1:0];
reg	    [32:0]		rd_tte_csm ;	
reg	    [32:0]		rd_tte_csm_real ;	
reg	    [32:0]		tlb_rd_tte_csm ;	
reg			            rd_csm;
reg                     wr_csm_sel;
reg     [1:0]           wr_tte_csm_thrid;

wire	[29:0] 		vrtl_pgnum_m;
wire			bypass ;

wire			wr_vld ;

//integer	i,j,k,l,m,n,p,r,s,t,u,w;

/*AUTOWIRE*/
// Beginning of automatic wires (for undeclared instantiated-module outputs)
// End of automatics

// Some bits are removed from the tag and data. 
// 'U' must be defined as a '1' on a write.
// 'L' required for demap all function.
// Do not need an internal valid bit for va range 47:22.
// These bits are always valid for a page. 
// 
// TTE STLB_TAG
//
//`define	STLB_TAG_PID_HI		58	: NEW PID - bit2
//`define	STLB_TAG_PID_LO		56	: NEW PID - bit0
//`define	STLB_TAG_R		55	: NEW Real bit
//`define 	STLB_TAG_PARITY		54	// Parity kept in same posn to avoid having
//`define	STLB_TAG_VA_47_28_HI 	53	// to redo interface
//`define	STLB_TAG_VA_47_28_LO 	34
//`define	STLB_TAG_VA_27_22_HI 	33	
//`define	STLB_TAG_VA_27_22_LO 	28
//`define	STLB_TAG_27_22_V	27	
//`define	STLB_TAG_V		26	: valid for entry. Write of 0 resets it.
//`define	STLB_TAG_L		25
//`define	STLB_TAG_U		24	
//`define	STLB_TAG_VA_21_16_HI 	23
//`define	STLB_TAG_VA_21_16_LO  	18
//`define	STLB_TAG_VA_21_16_V  	17	  	
//`define	STLB_TAG_VA_15_13_HI 	16
//`define	STLB_TAG_VA_15_13_LO  	14
//`define	STLB_TAG_VA_15_13_V  	13
//`define	STLB_TAG_CTXT_12_0_HI  	12	// removed Global bit
//`define	STLB_TAG_CTXT_12_0_LO  	0
//// 				Total - 59b
////
//// TTE STLB_DATA
////
//// Soft[12:7] & Soft2[58:50] are removed.
//// Diag[49:41] are removed. Used bit used for Diag[0] on read.
//// CV is included for software correctness.
//// PA<40> is removed as it is not used.
//// G/L present in data even though present in tag : can't read out simultaneously.
//   (Unfortunately this is no longer correct. For data read, tag is also read
//   simultaneously to get valid bit, used bits).
//`define 	STLB_DATA_PARITY  	42 
//`define 	STLB_DATA_PA_39_28_HI 	41	// CHANGE
//`define 	STLB_DATA_PA_39_28_LO 	30
//`define 	STLB_DATA_PA_27_22_HI 	29	// CHANGE
//`define 	STLB_DATA_PA_27_22_LO 	24
//`define 	STLB_DATA_27_22_SEL	23
//`define 	STLB_DATA_PA_21_16_HI 	22	// CHANGE
//`define 	STLB_DATA_PA_21_16_LO 	17
//`define 	STLB_DATA_21_16_SEL	16
//`define 	STLB_DATA_PA_15_13_HI 	15	
//`define 	STLB_DATA_PA_15_13_LO 	13
//`define 	STLB_DATA_15_13_SEL	12
//`define 	STLB_DATA_V  		11	: static, does not get modified.
//`define 	STLB_DATA_NFO  		10
//`define 	STLB_DATA_IE   		9
//`define 	STLB_DATA_L 		8 	: added for read.
//`define 	STLB_DATA_CP 		7 
//`define 	STLB_DATA_CV 		6 
//`define 	STLB_DATA_E  		5 
//`define 	STLB_DATA_P  		4 
//`define 	STLB_DATA_W  		3 
//`define 	STLB_DATA_SPARE_HI  	2	: Global bit has been removed
//`define 	STLB_DATA_SPARE_LO	0  	 
// 				Total - 43b

// Valid bits for key(tlb_cam_key/tlb_demap_key).
// Total - 41b
//`define	CAM_VA_47_28_HI  	40
//`define	CAM_VA_47_28_LO  	21
//`define	CAM_VA_47_28_V  	20	// b47-28 participate in match
//`define	CAM_VA_27_22_HI  	19
//`define	CAM_VA_27_22_LO  	14
//`define	CAM_VA_27_22_V  	13	// b27-22 participate in match
//`define	CAM_VA_21_16_HI  	12
//`define	CAM_VA_21_16_LO  	7
//`define	CAM_VA_21_16_V  	6	// b21-16 participate in match
//`define	CAM_VA_15_13_HI 	5	
//`define	CAM_VA_15_13_LO 	3	
//`define	CAM_VA_15_13_V 	 	2	// b15-13 participate in match
//`define	CAM_CTXT_GK 		1	// Context participates in match
//`define	CAM_REAL_V 		0	// cam/demap applies to real mapping
					

// ctxt port is different from cam key port even though both are
// required for cam. (tlb_ctxt)
// If Gk is set then ctxt will not participate in match.
// Total - 14b




//=========================================================================================
//	What's Left :
//=========================================================================================

// Scan Insertion - scan to be ignored in formal verification for now.

//=========================================================================================
//	Design Notes.
//=========================================================================================

// - Supported Demap Operations - By Page, By Context, All But
// Locked, Autodemap, Invalidate-All i.e., reset. Demap Partition is
// not supported - it is mapped to demap-all by logic. 
// - Interpretation of demap inputs
//	- tlb_demap - this is used to signal demap by page, by ctxt
//	,all, and autodemap. 
//	- tlb_demap_ctxt - If a demap_by_ctxt operation is occuring then
//	this signal and tlb_demap must be active.
//	- tlb_demap_all - demap all operation. If a demap_all operation is
//	occuring, then tlb_demap_all must be asserted with tlb_demap. 
// - Reset is similar to demap-all except that *all* entries
// are invalidated. The action is initiated by software. The reset occurs
// on the negedge and is synchronous with the clk.
// - TTE Tag and Data
// 	- The TTE tag and data can be read together. Each will have its 
//	own bus and the muxing will occur externally. The tag needs to
//	be read on a data request to supply the valid bit.
// 	- The TTE tag and data can be written together.
// - The cam hit is a separate output signal based on the 
// the match signals.
// - Read/Write may occur based on supplied index. If not valid
// then use replacement way determined by algorithm to write.
// - Only write can use replacement way determined by algorithm.
// - Data is formatted appr. on read or write in the MMU. 
// - The TLB will generate a signal which reports whether the 
// tlb can be filled in the current cycle or not.
// **Physical Tag Comparison**
// For I-SIDE, comparison is of 28b, whereas for D-side, comparison is of 29b. The actual
// comparison, due to legacy, is for 30b.
// For the I-TLB, va[11:10] must be hardwired to the same value as the lsb of the 4 tags
// at the port level. Since the itag it only 28b, add two least significant bits to extend it to 30b.
// Similarly, for the dside, va[10] needs to be made same.	
// **Differentiating among Various TLB Operations**
// Valid bits are now associated with the key to allow selective incorporation of
// match results. The 5 valid bits are : v4(b47-28),v3(b27-22),v2(21-16),v1(b15-13)
// and Gk(G bit for auto-demap). The rules of use are :
//	- cam: v4-v1 are set high. G=~cam_real=0/1.
//	- demap_by_page : v4-v1 are set high. G=1. cam_real=0.
// 	- demap_by_ctxt : v4-v1 are low. G=1. cam_real=0
//	- demap_all : v4-v1 are don't-care. G=x. cam_real=x
//	- autodemap : v4-v1 are based on page size of incoming tte. G=~cam_real=0/1.
// Note : Gk is now used only to void a context match on a Real Translation.
// In general, if a valid bit is low then the corresponding va field will not take
// part in the match. Similarly, for the ctxt, if Gk=1, the ctxt will participate
// in the match.
//
// Demap Table (For Satya) :
// Note : To include a context match, Gk must be set to 1.
//--------------------------------------------------------------------------------------------------------
//tlb_demap tlb_demap_all  tlb_ctxt Gk	Vk4 Vk3	Vk2 Vk1 Real	Operation
//--------------------------------------------------------------------------------------------------------
//0		x		x   x	x   x	x   x   0	No demap operation
//1		0		0   1	1   1	1   1	0	Demap by page
//1		0		0   1	1   0	0   0	0/1	256M demap(auto demap)
//1		0		0   0	1   0	0   0	0	256M demap(auto demap) (*Illgl*)
//1		0		0   1	1   1	0   0	0/1	4M demap(auto demap)
//1		0		0   0	1   1	0   0	0	4M demap(auto demap) (*Illgl*)
//1		0		0   1	1   1	1   0	0/1	64k demap(auto demap)
//1		0		0   0	1   1	1   0	0	64k demap(auto demap) (*Illgl*)
//1		0		0   1	1   1	1   1	0/1	8k demap(auto demap)
//1		0		0   0	1   1	1   1	0	8k demap(auto demap) (*Illgl*)
//1		0		1   1	0   0	0   0	0	demap by ctxt
//1		1		x   x	x   x	x   x	0	demap_all
//------------------------------------------------------------------------------------------
//-----
//All other are illegal combinations
//
//=========================================================================================
//	Changes related to Hypervisor/Legacy Compatibility
//=========================================================================================
//
// - Add PID. PID does not effect demap-all. Otherwise it is included in cam, other demap
// operations and auto-demap.
// - Add R. Real translation ignores context. This is controlled externally by Gk.
// - Remove G bit for tte. Input remains in demap-key/cam-key to allow for disabling
//   of context match Real Translation  
// - Final Page Size support - 8KB,64KB,4M,256M
// - SPARC_HPV_EN has been defined to enable new tlb design support. 
// Issues : 
// -Max ptag size is now 28b. Satya, will this help the speed at all. I doubt it !

//=========================================================================================
//	Miscellaneous
//=========================================================================================
   wire clk;
   assign clk = rclk;
   
wire async_reset, sync_reset ;
assign	async_reset = ~arst_l ; 			// hardware
assign	sync_reset = (~rst_soft_l & ~rst_tri_en) ;	// software

wire rw_disable ;
// INNO - wr/rd gated off. Note required as rst_tri_en is
// asserted, but implemented in addition in schematic.
assign	rw_disable = ~arst_l | rst_tri_en ;

//=========================================================================================
// 	Stage Data
//=========================================================================================
// Apply address masking
assign	tlb_cam_key_masked[40:25]
	= {16{tlb_addr_mask_l}} & 
		tlb_cam_key[40:21+4] ;

// Reconstitute cam data CHANGE : add additional bit for real mapping
assign	tlb_cam_data[53:13] = tlb_demap ? 
	tlb_demap_key[40:0] :
	{tlb_cam_key_masked[40:25],tlb_cam_key[21+3:0]} ; 

assign tlb_cam_comp_key[26:0] = 
		tlb_demap ?
			{tlb_demap_key[32:21], tlb_demap_key[19:14],tlb_demap_key[12:7],
			tlb_demap_key[5:3]} :
			{tlb_cam_key_masked[32:25],tlb_cam_key[24:21],
			tlb_cam_key[19:14],tlb_cam_key[12:7],tlb_cam_key[5:3]} ;

assign	tlb_cam_data[12:0] = tlb_ctxt[12:0] ;

// These signals are flow-thru.
assign	wr_tte_tag[58:0] 	= tlb_wr_tte_tag[58:0] ;	// CHANGE
assign	wr_tte_data[42:0] 	= tlb_wr_tte_data[42:0] ;

assign wr_tte_csm = tlb_wr_tte_csm;



// CHANGE(SATYA) - Currently the rw_index/rw_index_vld are shared by both reads
// and writes. However, writes are done in the cycle of broadcast, whereas
// the reads are done a cycle later, as given in the model(incorrect) 
// They have to be treated uniformly. To make the model work, I've assumed the read/write 
// are done in the cycle the valids are broadcast. 
always @ (posedge clk)
	begin
	if (hold)
		begin
		cam_pid[2:0]		<= cam_pid[2:0] ;
		cam_vld_tmp		<= cam_vld_tmp ;
		cam_data[53:0] 		<= cam_data[53:0] ;
		demap_other_tmp		<= demap_other_tmp ;
		demap_auto		<= demap_auto ;
		demap_all		<= demap_all ;
		wr_vld_tmp 		<= wr_vld_tmp ;
		rd_tag 			<= rd_tag ;
		rd_data			<= rd_data ;
		rw_index_vld		<= rw_index_vld ;
		rw_index[4-1:0]		<= rw_index[4-1:0] ; 
            
        
        rd_csm          <= rd_csm;
        wr_csm_sel      <= wr_csm_sel;
        wr_tte_csm_thrid <= wr_tte_csm_thrid;
        
		end
	else
		begin
		cam_pid[2:0]		<= tlb_cam_pid[2:0] ;
		cam_vld_tmp		<= tlb_cam_vld ;
		cam_data[53:0] 		<= tlb_cam_data[53:0] ;
		demap_other_tmp		<= tlb_demap ;
		demap_auto		<= tlb_demap_auto ;
		demap_all		<= tlb_demap_all ;
		wr_vld_tmp 		<= tlb_wr_vld ;
		rd_tag 			<= tlb_rd_tag_vld ;
		rd_data			<= tlb_rd_data_vld ;
		rw_index_vld		<= tlb_rw_index_vld ;
		rw_index[4-1:0]		<= tlb_rw_index[4-1:0] ; 	
		
        
        rd_csm          <= tlb_rd_csm_vld;
        wr_csm_sel      <= tlb_wr_csm_sel;
        wr_tte_csm_thrid <= tlb_wr_tte_csm_thrid;
        
        end

	end

// INNO - gate cam,demap,wr with rst_tri_en.
reg rst_tri_en_lat;

 always        @ (clk)
 rst_tri_en_lat = rst_tri_en;

assign	cam_vld = cam_vld_tmp & ~rst_tri_en_lat ;
assign	demap_other = demap_other_tmp & ~rst_tri_en ;
assign	wr_vld = wr_vld_tmp & ~rst_tri_en ;

//=========================================================================================
//	Generate Write Wordlines
//=========================================================================================

// Based on static rw index	
// This generates the wordlines for a read/write to the tlb based on index. Wordlines for
// the write based on replacement alg. are muxed in later.


always	@ (/*AUTOSENSE*/rd_csm or rd_data or rd_tag or rw_index or rw_index_vld
           or wr_vld_tmp)
	begin
        if ((rw_index[4-1:0] == 0) & ((wr_vld_tmp & rw_index_vld) | rd_tag | rd_data | rd_csm))
         begin
            rw_wdline[0] = 1'b1;
         end
         else	
         begin
            rw_wdline[0] = 1'b0;
         end
        if ((rw_index[4-1:0] == 1) & ((wr_vld_tmp & rw_index_vld) | rd_tag | rd_data | rd_csm))
         begin
            rw_wdline[1] = 1'b1;
         end
         else	
         begin
            rw_wdline[1] = 1'b0;
         end
        if ((rw_index[4-1:0] == 2) & ((wr_vld_tmp & rw_index_vld) | rd_tag | rd_data | rd_csm))
         begin
            rw_wdline[2] = 1'b1;
         end
         else	
         begin
            rw_wdline[2] = 1'b0;
         end
        if ((rw_index[4-1:0] == 3) & ((wr_vld_tmp & rw_index_vld) | rd_tag | rd_data | rd_csm))
         begin
            rw_wdline[3] = 1'b1;
         end
         else	
         begin
            rw_wdline[3] = 1'b0;
         end
        if ((rw_index[4-1:0] == 4) & ((wr_vld_tmp & rw_index_vld) | rd_tag | rd_data | rd_csm))
         begin
            rw_wdline[4] = 1'b1;
         end
         else	
         begin
            rw_wdline[4] = 1'b0;
         end
        if ((rw_index[4-1:0] == 5) & ((wr_vld_tmp & rw_index_vld) | rd_tag | rd_data | rd_csm))
         begin
            rw_wdline[5] = 1'b1;
         end
         else	
         begin
            rw_wdline[5] = 1'b0;
         end
        if ((rw_index[4-1:0] == 6) & ((wr_vld_tmp & rw_index_vld) | rd_tag | rd_data | rd_csm))
         begin
            rw_wdline[6] = 1'b1;
         end
         else	
         begin
            rw_wdline[6] = 1'b0;
         end
        if ((rw_index[4-1:0] == 7) & ((wr_vld_tmp & rw_index_vld) | rd_tag | rd_data | rd_csm))
         begin
            rw_wdline[7] = 1'b1;
         end
         else	
         begin
            rw_wdline[7] = 1'b0;
         end
        if ((rw_index[4-1:0] == 8) & ((wr_vld_tmp & rw_index_vld) | rd_tag | rd_data | rd_csm))
         begin
            rw_wdline[8] = 1'b1;
         end
         else	
         begin
            rw_wdline[8] = 1'b0;
         end
        if ((rw_index[4-1:0] == 9) & ((wr_vld_tmp & rw_index_vld) | rd_tag | rd_data | rd_csm))
         begin
            rw_wdline[9] = 1'b1;
         end
         else	
         begin
            rw_wdline[9] = 1'b0;
         end
        if ((rw_index[4-1:0] == 10) & ((wr_vld_tmp & rw_index_vld) | rd_tag | rd_data | rd_csm))
         begin
            rw_wdline[10] = 1'b1;
         end
         else	
         begin
            rw_wdline[10] = 1'b0;
         end
        if ((rw_index[4-1:0] == 11) & ((wr_vld_tmp & rw_index_vld) | rd_tag | rd_data | rd_csm))
         begin
            rw_wdline[11] = 1'b1;
         end
         else	
         begin
            rw_wdline[11] = 1'b0;
         end
        if ((rw_index[4-1:0] == 12) & ((wr_vld_tmp & rw_index_vld) | rd_tag | rd_data | rd_csm))
         begin
            rw_wdline[12] = 1'b1;
         end
         else	
         begin
            rw_wdline[12] = 1'b0;
         end
        if ((rw_index[4-1:0] == 13) & ((wr_vld_tmp & rw_index_vld) | rd_tag | rd_data | rd_csm))
         begin
            rw_wdline[13] = 1'b1;
         end
         else	
         begin
            rw_wdline[13] = 1'b0;
         end
        if ((rw_index[4-1:0] == 14) & ((wr_vld_tmp & rw_index_vld) | rd_tag | rd_data | rd_csm))
         begin
            rw_wdline[14] = 1'b1;
         end
         else	
         begin
            rw_wdline[14] = 1'b0;
         end
        if ((rw_index[4-1:0] == 15) & ((wr_vld_tmp & rw_index_vld) | rd_tag | rd_data | rd_csm))
         begin
            rw_wdline[15] = 1'b1;
         end
         else	
         begin
            rw_wdline[15] = 1'b0;
         end

					
	end









































































































































//=========================================================================================
//	Write TLB
//=========================================================================================

reg	[58:0]	tmp_tag ;
reg	[42:0]	tmp_data ;


reg [32:0] tmp_csm;



// Currently TLB_TAG and TLB_DATA RAMs are written in the B phase. 
// Used bit is set on write in later code as it is also effected by read of tlb.
always	@ (negedge clk)
	begin
        if (((rw_index_vld & rw_wdline[0]) | (~rw_index_vld & tlb_entry_replace_d2[0])) & wr_vld_tmp & ~rw_disable)
		    begin
				if (~rst_tri_en)
					begin
                    if (wr_csm_sel)
                    begin
                        tte_csm_buf[wr_tte_csm_thrid] <= wr_tte_csm;
                    end
                    else
                    begin
                        tte_csm_ram[0] <= tte_csm_buf[wr_tte_csm_thrid];
                        tte_tag_ram[0] <= wr_tte_tag[58:0];	// CHANGE
                        tte_data_ram[0] <= wr_tte_data[42:0];
                        //tlb_entry_vld[j] <= wr_tte_tag[`STLB_TAG_V] ;
                        tlb_entry_used[0] <= wr_tte_tag[24] ;
                        tlb_entry_locked[0] = wr_tte_tag[25] ;
                    end
					// write-thru 
					rd_tte_tag[58:0]  <= wr_tte_tag[58:0] ;	// CHANGE 
					rd_tte_data[42:0] <=  wr_tte_data[42:0];
                    rd_tte_csm <= wr_tte_csm;
					end
				else
					begin
					tmp_tag[58:0]=tte_tag_ram[0]; // use non-blocking
					tmp_data[42:0]=tte_data_ram[0];
                    tmp_csm = tte_csm_ram[0];
					// INNO - read wins.
					rd_tte_tag[58:0] <=	
					{tmp_tag[58:27], tlb_entry_vld[0],tlb_entry_locked[0], 
					tlb_entry_used[0], tmp_tag[23:0]}  ;
					rd_tte_data[42:0] <= {tmp_data[42:12],tmp_data[11:0]} ;
                    rd_tte_csm <= tmp_csm;
					end
			
			end
        if (((rw_index_vld & rw_wdline[1]) | (~rw_index_vld & tlb_entry_replace_d2[1])) & wr_vld_tmp & ~rw_disable)
		    begin
				if (~rst_tri_en)
					begin
                    if (wr_csm_sel)
                    begin
                        tte_csm_buf[wr_tte_csm_thrid] <= wr_tte_csm;
                    end
                    else
                    begin
                        tte_csm_ram[1] <= tte_csm_buf[wr_tte_csm_thrid];
                        tte_tag_ram[1] <= wr_tte_tag[58:0];	// CHANGE
                        tte_data_ram[1] <= wr_tte_data[42:0];
                        //tlb_entry_vld[j] <= wr_tte_tag[`STLB_TAG_V] ;
                        tlb_entry_used[1] <= wr_tte_tag[24] ;
                        tlb_entry_locked[1] = wr_tte_tag[25] ;
                    end
					// write-thru 
					rd_tte_tag[58:0]  <= wr_tte_tag[58:0] ;	// CHANGE 
					rd_tte_data[42:0] <=  wr_tte_data[42:0];
                    rd_tte_csm <= wr_tte_csm;
					end
				else
					begin
					tmp_tag[58:0]=tte_tag_ram[1]; // use non-blocking
					tmp_data[42:0]=tte_data_ram[1];
                    tmp_csm = tte_csm_ram[1];
					// INNO - read wins.
					rd_tte_tag[58:0] <=	
					{tmp_tag[58:27], tlb_entry_vld[1],tlb_entry_locked[1], 
					tlb_entry_used[1], tmp_tag[23:0]}  ;
					rd_tte_data[42:0] <= {tmp_data[42:12],tmp_data[11:0]} ;
                    rd_tte_csm <= tmp_csm;
					end
			
			end
        if (((rw_index_vld & rw_wdline[2]) | (~rw_index_vld & tlb_entry_replace_d2[2])) & wr_vld_tmp & ~rw_disable)
		    begin
				if (~rst_tri_en)
					begin
                    if (wr_csm_sel)
                    begin
                        tte_csm_buf[wr_tte_csm_thrid] <= wr_tte_csm;
                    end
                    else
                    begin
                        tte_csm_ram[2] <= tte_csm_buf[wr_tte_csm_thrid];
                        tte_tag_ram[2] <= wr_tte_tag[58:0];	// CHANGE
                        tte_data_ram[2] <= wr_tte_data[42:0];
                        //tlb_entry_vld[j] <= wr_tte_tag[`STLB_TAG_V] ;
                        tlb_entry_used[2] <= wr_tte_tag[24] ;
                        tlb_entry_locked[2] = wr_tte_tag[25] ;
                    end
					// write-thru 
					rd_tte_tag[58:0]  <= wr_tte_tag[58:0] ;	// CHANGE 
					rd_tte_data[42:0] <=  wr_tte_data[42:0];
                    rd_tte_csm <= wr_tte_csm;
					end
				else
					begin
					tmp_tag[58:0]=tte_tag_ram[2]; // use non-blocking
					tmp_data[42:0]=tte_data_ram[2];
                    tmp_csm = tte_csm_ram[2];
					// INNO - read wins.
					rd_tte_tag[58:0] <=	
					{tmp_tag[58:27], tlb_entry_vld[2],tlb_entry_locked[2], 
					tlb_entry_used[2], tmp_tag[23:0]}  ;
					rd_tte_data[42:0] <= {tmp_data[42:12],tmp_data[11:0]} ;
                    rd_tte_csm <= tmp_csm;
					end
			
			end
        if (((rw_index_vld & rw_wdline[3]) | (~rw_index_vld & tlb_entry_replace_d2[3])) & wr_vld_tmp & ~rw_disable)
		    begin
				if (~rst_tri_en)
					begin
                    if (wr_csm_sel)
                    begin
                        tte_csm_buf[wr_tte_csm_thrid] <= wr_tte_csm;
                    end
                    else
                    begin
                        tte_csm_ram[3] <= tte_csm_buf[wr_tte_csm_thrid];
                        tte_tag_ram[3] <= wr_tte_tag[58:0];	// CHANGE
                        tte_data_ram[3] <= wr_tte_data[42:0];
                        //tlb_entry_vld[j] <= wr_tte_tag[`STLB_TAG_V] ;
                        tlb_entry_used[3] <= wr_tte_tag[24] ;
                        tlb_entry_locked[3] = wr_tte_tag[25] ;
                    end
					// write-thru 
					rd_tte_tag[58:0]  <= wr_tte_tag[58:0] ;	// CHANGE 
					rd_tte_data[42:0] <=  wr_tte_data[42:0];
                    rd_tte_csm <= wr_tte_csm;
					end
				else
					begin
					tmp_tag[58:0]=tte_tag_ram[3]; // use non-blocking
					tmp_data[42:0]=tte_data_ram[3];
                    tmp_csm = tte_csm_ram[3];
					// INNO - read wins.
					rd_tte_tag[58:0] <=	
					{tmp_tag[58:27], tlb_entry_vld[3],tlb_entry_locked[3], 
					tlb_entry_used[3], tmp_tag[23:0]}  ;
					rd_tte_data[42:0] <= {tmp_data[42:12],tmp_data[11:0]} ;
                    rd_tte_csm <= tmp_csm;
					end
			
			end
        if (((rw_index_vld & rw_wdline[4]) | (~rw_index_vld & tlb_entry_replace_d2[4])) & wr_vld_tmp & ~rw_disable)
		    begin
				if (~rst_tri_en)
					begin
                    if (wr_csm_sel)
                    begin
                        tte_csm_buf[wr_tte_csm_thrid] <= wr_tte_csm;
                    end
                    else
                    begin
                        tte_csm_ram[4] <= tte_csm_buf[wr_tte_csm_thrid];
                        tte_tag_ram[4] <= wr_tte_tag[58:0];	// CHANGE
                        tte_data_ram[4] <= wr_tte_data[42:0];
                        //tlb_entry_vld[j] <= wr_tte_tag[`STLB_TAG_V] ;
                        tlb_entry_used[4] <= wr_tte_tag[24] ;
                        tlb_entry_locked[4] = wr_tte_tag[25] ;
                    end
					// write-thru 
					rd_tte_tag[58:0]  <= wr_tte_tag[58:0] ;	// CHANGE 
					rd_tte_data[42:0] <=  wr_tte_data[42:0];
                    rd_tte_csm <= wr_tte_csm;
					end
				else
					begin
					tmp_tag[58:0]=tte_tag_ram[4]; // use non-blocking
					tmp_data[42:0]=tte_data_ram[4];
                    tmp_csm = tte_csm_ram[4];
					// INNO - read wins.
					rd_tte_tag[58:0] <=	
					{tmp_tag[58:27], tlb_entry_vld[4],tlb_entry_locked[4], 
					tlb_entry_used[4], tmp_tag[23:0]}  ;
					rd_tte_data[42:0] <= {tmp_data[42:12],tmp_data[11:0]} ;
                    rd_tte_csm <= tmp_csm;
					end
			
			end
        if (((rw_index_vld & rw_wdline[5]) | (~rw_index_vld & tlb_entry_replace_d2[5])) & wr_vld_tmp & ~rw_disable)
		    begin
				if (~rst_tri_en)
					begin
                    if (wr_csm_sel)
                    begin
                        tte_csm_buf[wr_tte_csm_thrid] <= wr_tte_csm;
                    end
                    else
                    begin
                        tte_csm_ram[5] <= tte_csm_buf[wr_tte_csm_thrid];
                        tte_tag_ram[5] <= wr_tte_tag[58:0];	// CHANGE
                        tte_data_ram[5] <= wr_tte_data[42:0];
                        //tlb_entry_vld[j] <= wr_tte_tag[`STLB_TAG_V] ;
                        tlb_entry_used[5] <= wr_tte_tag[24] ;
                        tlb_entry_locked[5] = wr_tte_tag[25] ;
                    end
					// write-thru 
					rd_tte_tag[58:0]  <= wr_tte_tag[58:0] ;	// CHANGE 
					rd_tte_data[42:0] <=  wr_tte_data[42:0];
                    rd_tte_csm <= wr_tte_csm;
					end
				else
					begin
					tmp_tag[58:0]=tte_tag_ram[5]; // use non-blocking
					tmp_data[42:0]=tte_data_ram[5];
                    tmp_csm = tte_csm_ram[5];
					// INNO - read wins.
					rd_tte_tag[58:0] <=	
					{tmp_tag[58:27], tlb_entry_vld[5],tlb_entry_locked[5], 
					tlb_entry_used[5], tmp_tag[23:0]}  ;
					rd_tte_data[42:0] <= {tmp_data[42:12],tmp_data[11:0]} ;
                    rd_tte_csm <= tmp_csm;
					end
			
			end
        if (((rw_index_vld & rw_wdline[6]) | (~rw_index_vld & tlb_entry_replace_d2[6])) & wr_vld_tmp & ~rw_disable)
		    begin
				if (~rst_tri_en)
					begin
                    if (wr_csm_sel)
                    begin
                        tte_csm_buf[wr_tte_csm_thrid] <= wr_tte_csm;
                    end
                    else
                    begin
                        tte_csm_ram[6] <= tte_csm_buf[wr_tte_csm_thrid];
                        tte_tag_ram[6] <= wr_tte_tag[58:0];	// CHANGE
                        tte_data_ram[6] <= wr_tte_data[42:0];
                        //tlb_entry_vld[j] <= wr_tte_tag[`STLB_TAG_V] ;
                        tlb_entry_used[6] <= wr_tte_tag[24] ;
                        tlb_entry_locked[6] = wr_tte_tag[25] ;
                    end
					// write-thru 
					rd_tte_tag[58:0]  <= wr_tte_tag[58:0] ;	// CHANGE 
					rd_tte_data[42:0] <=  wr_tte_data[42:0];
                    rd_tte_csm <= wr_tte_csm;
					end
				else
					begin
					tmp_tag[58:0]=tte_tag_ram[6]; // use non-blocking
					tmp_data[42:0]=tte_data_ram[6];
                    tmp_csm = tte_csm_ram[6];
					// INNO - read wins.
					rd_tte_tag[58:0] <=	
					{tmp_tag[58:27], tlb_entry_vld[6],tlb_entry_locked[6], 
					tlb_entry_used[6], tmp_tag[23:0]}  ;
					rd_tte_data[42:0] <= {tmp_data[42:12],tmp_data[11:0]} ;
                    rd_tte_csm <= tmp_csm;
					end
			
			end
        if (((rw_index_vld & rw_wdline[7]) | (~rw_index_vld & tlb_entry_replace_d2[7])) & wr_vld_tmp & ~rw_disable)
		    begin
				if (~rst_tri_en)
					begin
                    if (wr_csm_sel)
                    begin
                        tte_csm_buf[wr_tte_csm_thrid] <= wr_tte_csm;
                    end
                    else
                    begin
                        tte_csm_ram[7] <= tte_csm_buf[wr_tte_csm_thrid];
                        tte_tag_ram[7] <= wr_tte_tag[58:0];	// CHANGE
                        tte_data_ram[7] <= wr_tte_data[42:0];
                        //tlb_entry_vld[j] <= wr_tte_tag[`STLB_TAG_V] ;
                        tlb_entry_used[7] <= wr_tte_tag[24] ;
                        tlb_entry_locked[7] = wr_tte_tag[25] ;
                    end
					// write-thru 
					rd_tte_tag[58:0]  <= wr_tte_tag[58:0] ;	// CHANGE 
					rd_tte_data[42:0] <=  wr_tte_data[42:0];
                    rd_tte_csm <= wr_tte_csm;
					end
				else
					begin
					tmp_tag[58:0]=tte_tag_ram[7]; // use non-blocking
					tmp_data[42:0]=tte_data_ram[7];
                    tmp_csm = tte_csm_ram[7];
					// INNO - read wins.
					rd_tte_tag[58:0] <=	
					{tmp_tag[58:27], tlb_entry_vld[7],tlb_entry_locked[7], 
					tlb_entry_used[7], tmp_tag[23:0]}  ;
					rd_tte_data[42:0] <= {tmp_data[42:12],tmp_data[11:0]} ;
                    rd_tte_csm <= tmp_csm;
					end
			
			end
        if (((rw_index_vld & rw_wdline[8]) | (~rw_index_vld & tlb_entry_replace_d2[8])) & wr_vld_tmp & ~rw_disable)
		    begin
				if (~rst_tri_en)
					begin
                    if (wr_csm_sel)
                    begin
                        tte_csm_buf[wr_tte_csm_thrid] <= wr_tte_csm;
                    end
                    else
                    begin
                        tte_csm_ram[8] <= tte_csm_buf[wr_tte_csm_thrid];
                        tte_tag_ram[8] <= wr_tte_tag[58:0];	// CHANGE
                        tte_data_ram[8] <= wr_tte_data[42:0];
                        //tlb_entry_vld[j] <= wr_tte_tag[`STLB_TAG_V] ;
                        tlb_entry_used[8] <= wr_tte_tag[24] ;
                        tlb_entry_locked[8] = wr_tte_tag[25] ;
                    end
					// write-thru 
					rd_tte_tag[58:0]  <= wr_tte_tag[58:0] ;	// CHANGE 
					rd_tte_data[42:0] <=  wr_tte_data[42:0];
                    rd_tte_csm <= wr_tte_csm;
					end
				else
					begin
					tmp_tag[58:0]=tte_tag_ram[8]; // use non-blocking
					tmp_data[42:0]=tte_data_ram[8];
                    tmp_csm = tte_csm_ram[8];
					// INNO - read wins.
					rd_tte_tag[58:0] <=	
					{tmp_tag[58:27], tlb_entry_vld[8],tlb_entry_locked[8], 
					tlb_entry_used[8], tmp_tag[23:0]}  ;
					rd_tte_data[42:0] <= {tmp_data[42:12],tmp_data[11:0]} ;
                    rd_tte_csm <= tmp_csm;
					end
			
			end
        if (((rw_index_vld & rw_wdline[9]) | (~rw_index_vld & tlb_entry_replace_d2[9])) & wr_vld_tmp & ~rw_disable)
		    begin
				if (~rst_tri_en)
					begin
                    if (wr_csm_sel)
                    begin
                        tte_csm_buf[wr_tte_csm_thrid] <= wr_tte_csm;
                    end
                    else
                    begin
                        tte_csm_ram[9] <= tte_csm_buf[wr_tte_csm_thrid];
                        tte_tag_ram[9] <= wr_tte_tag[58:0];	// CHANGE
                        tte_data_ram[9] <= wr_tte_data[42:0];
                        //tlb_entry_vld[j] <= wr_tte_tag[`STLB_TAG_V] ;
                        tlb_entry_used[9] <= wr_tte_tag[24] ;
                        tlb_entry_locked[9] = wr_tte_tag[25] ;
                    end
					// write-thru 
					rd_tte_tag[58:0]  <= wr_tte_tag[58:0] ;	// CHANGE 
					rd_tte_data[42:0] <=  wr_tte_data[42:0];
                    rd_tte_csm <= wr_tte_csm;
					end
				else
					begin
					tmp_tag[58:0]=tte_tag_ram[9]; // use non-blocking
					tmp_data[42:0]=tte_data_ram[9];
                    tmp_csm = tte_csm_ram[9];
					// INNO - read wins.
					rd_tte_tag[58:0] <=	
					{tmp_tag[58:27], tlb_entry_vld[9],tlb_entry_locked[9], 
					tlb_entry_used[9], tmp_tag[23:0]}  ;
					rd_tte_data[42:0] <= {tmp_data[42:12],tmp_data[11:0]} ;
                    rd_tte_csm <= tmp_csm;
					end
			
			end
        if (((rw_index_vld & rw_wdline[10]) | (~rw_index_vld & tlb_entry_replace_d2[10])) & wr_vld_tmp & ~rw_disable)
		    begin
				if (~rst_tri_en)
					begin
                    if (wr_csm_sel)
                    begin
                        tte_csm_buf[wr_tte_csm_thrid] <= wr_tte_csm;
                    end
                    else
                    begin
                        tte_csm_ram[10] <= tte_csm_buf[wr_tte_csm_thrid];
                        tte_tag_ram[10] <= wr_tte_tag[58:0];	// CHANGE
                        tte_data_ram[10] <= wr_tte_data[42:0];
                        //tlb_entry_vld[j] <= wr_tte_tag[`STLB_TAG_V] ;
                        tlb_entry_used[10] <= wr_tte_tag[24] ;
                        tlb_entry_locked[10] = wr_tte_tag[25] ;
                    end
					// write-thru 
					rd_tte_tag[58:0]  <= wr_tte_tag[58:0] ;	// CHANGE 
					rd_tte_data[42:0] <=  wr_tte_data[42:0];
                    rd_tte_csm <= wr_tte_csm;
					end
				else
					begin
					tmp_tag[58:0]=tte_tag_ram[10]; // use non-blocking
					tmp_data[42:0]=tte_data_ram[10];
                    tmp_csm = tte_csm_ram[10];
					// INNO - read wins.
					rd_tte_tag[58:0] <=	
					{tmp_tag[58:27], tlb_entry_vld[10],tlb_entry_locked[10], 
					tlb_entry_used[10], tmp_tag[23:0]}  ;
					rd_tte_data[42:0] <= {tmp_data[42:12],tmp_data[11:0]} ;
                    rd_tte_csm <= tmp_csm;
					end
			
			end
        if (((rw_index_vld & rw_wdline[11]) | (~rw_index_vld & tlb_entry_replace_d2[11])) & wr_vld_tmp & ~rw_disable)
		    begin
				if (~rst_tri_en)
					begin
                    if (wr_csm_sel)
                    begin
                        tte_csm_buf[wr_tte_csm_thrid] <= wr_tte_csm;
                    end
                    else
                    begin
                        tte_csm_ram[11] <= tte_csm_buf[wr_tte_csm_thrid];
                        tte_tag_ram[11] <= wr_tte_tag[58:0];	// CHANGE
                        tte_data_ram[11] <= wr_tte_data[42:0];
                        //tlb_entry_vld[j] <= wr_tte_tag[`STLB_TAG_V] ;
                        tlb_entry_used[11] <= wr_tte_tag[24] ;
                        tlb_entry_locked[11] = wr_tte_tag[25] ;
                    end
					// write-thru 
					rd_tte_tag[58:0]  <= wr_tte_tag[58:0] ;	// CHANGE 
					rd_tte_data[42:0] <=  wr_tte_data[42:0];
                    rd_tte_csm <= wr_tte_csm;
					end
				else
					begin
					tmp_tag[58:0]=tte_tag_ram[11]; // use non-blocking
					tmp_data[42:0]=tte_data_ram[11];
                    tmp_csm = tte_csm_ram[11];
					// INNO - read wins.
					rd_tte_tag[58:0] <=	
					{tmp_tag[58:27], tlb_entry_vld[11],tlb_entry_locked[11], 
					tlb_entry_used[11], tmp_tag[23:0]}  ;
					rd_tte_data[42:0] <= {tmp_data[42:12],tmp_data[11:0]} ;
                    rd_tte_csm <= tmp_csm;
					end
			
			end
        if (((rw_index_vld & rw_wdline[12]) | (~rw_index_vld & tlb_entry_replace_d2[12])) & wr_vld_tmp & ~rw_disable)
		    begin
				if (~rst_tri_en)
					begin
                    if (wr_csm_sel)
                    begin
                        tte_csm_buf[wr_tte_csm_thrid] <= wr_tte_csm;
                    end
                    else
                    begin
                        tte_csm_ram[12] <= tte_csm_buf[wr_tte_csm_thrid];
                        tte_tag_ram[12] <= wr_tte_tag[58:0];	// CHANGE
                        tte_data_ram[12] <= wr_tte_data[42:0];
                        //tlb_entry_vld[j] <= wr_tte_tag[`STLB_TAG_V] ;
                        tlb_entry_used[12] <= wr_tte_tag[24] ;
                        tlb_entry_locked[12] = wr_tte_tag[25] ;
                    end
					// write-thru 
					rd_tte_tag[58:0]  <= wr_tte_tag[58:0] ;	// CHANGE 
					rd_tte_data[42:0] <=  wr_tte_data[42:0];
                    rd_tte_csm <= wr_tte_csm;
					end
				else
					begin
					tmp_tag[58:0]=tte_tag_ram[12]; // use non-blocking
					tmp_data[42:0]=tte_data_ram[12];
                    tmp_csm = tte_csm_ram[12];
					// INNO - read wins.
					rd_tte_tag[58:0] <=	
					{tmp_tag[58:27], tlb_entry_vld[12],tlb_entry_locked[12], 
					tlb_entry_used[12], tmp_tag[23:0]}  ;
					rd_tte_data[42:0] <= {tmp_data[42:12],tmp_data[11:0]} ;
                    rd_tte_csm <= tmp_csm;
					end
			
			end
        if (((rw_index_vld & rw_wdline[13]) | (~rw_index_vld & tlb_entry_replace_d2[13])) & wr_vld_tmp & ~rw_disable)
		    begin
				if (~rst_tri_en)
					begin
                    if (wr_csm_sel)
                    begin
                        tte_csm_buf[wr_tte_csm_thrid] <= wr_tte_csm;
                    end
                    else
                    begin
                        tte_csm_ram[13] <= tte_csm_buf[wr_tte_csm_thrid];
                        tte_tag_ram[13] <= wr_tte_tag[58:0];	// CHANGE
                        tte_data_ram[13] <= wr_tte_data[42:0];
                        //tlb_entry_vld[j] <= wr_tte_tag[`STLB_TAG_V] ;
                        tlb_entry_used[13] <= wr_tte_tag[24] ;
                        tlb_entry_locked[13] = wr_tte_tag[25] ;
                    end
					// write-thru 
					rd_tte_tag[58:0]  <= wr_tte_tag[58:0] ;	// CHANGE 
					rd_tte_data[42:0] <=  wr_tte_data[42:0];
                    rd_tte_csm <= wr_tte_csm;
					end
				else
					begin
					tmp_tag[58:0]=tte_tag_ram[13]; // use non-blocking
					tmp_data[42:0]=tte_data_ram[13];
                    tmp_csm = tte_csm_ram[13];
					// INNO - read wins.
					rd_tte_tag[58:0] <=	
					{tmp_tag[58:27], tlb_entry_vld[13],tlb_entry_locked[13], 
					tlb_entry_used[13], tmp_tag[23:0]}  ;
					rd_tte_data[42:0] <= {tmp_data[42:12],tmp_data[11:0]} ;
                    rd_tte_csm <= tmp_csm;
					end
			
			end
        if (((rw_index_vld & rw_wdline[14]) | (~rw_index_vld & tlb_entry_replace_d2[14])) & wr_vld_tmp & ~rw_disable)
		    begin
				if (~rst_tri_en)
					begin
                    if (wr_csm_sel)
                    begin
                        tte_csm_buf[wr_tte_csm_thrid] <= wr_tte_csm;
                    end
                    else
                    begin
                        tte_csm_ram[14] <= tte_csm_buf[wr_tte_csm_thrid];
                        tte_tag_ram[14] <= wr_tte_tag[58:0];	// CHANGE
                        tte_data_ram[14] <= wr_tte_data[42:0];
                        //tlb_entry_vld[j] <= wr_tte_tag[`STLB_TAG_V] ;
                        tlb_entry_used[14] <= wr_tte_tag[24] ;
                        tlb_entry_locked[14] = wr_tte_tag[25] ;
                    end
					// write-thru 
					rd_tte_tag[58:0]  <= wr_tte_tag[58:0] ;	// CHANGE 
					rd_tte_data[42:0] <=  wr_tte_data[42:0];
                    rd_tte_csm <= wr_tte_csm;
					end
				else
					begin
					tmp_tag[58:0]=tte_tag_ram[14]; // use non-blocking
					tmp_data[42:0]=tte_data_ram[14];
                    tmp_csm = tte_csm_ram[14];
					// INNO - read wins.
					rd_tte_tag[58:0] <=	
					{tmp_tag[58:27], tlb_entry_vld[14],tlb_entry_locked[14], 
					tlb_entry_used[14], tmp_tag[23:0]}  ;
					rd_tte_data[42:0] <= {tmp_data[42:12],tmp_data[11:0]} ;
                    rd_tte_csm <= tmp_csm;
					end
			
			end
        if (((rw_index_vld & rw_wdline[15]) | (~rw_index_vld & tlb_entry_replace_d2[15])) & wr_vld_tmp & ~rw_disable)
		    begin
				if (~rst_tri_en)
					begin
                    if (wr_csm_sel)
                    begin
                        tte_csm_buf[wr_tte_csm_thrid] <= wr_tte_csm;
                    end
                    else
                    begin
                        tte_csm_ram[15] <= tte_csm_buf[wr_tte_csm_thrid];
                        tte_tag_ram[15] <= wr_tte_tag[58:0];	// CHANGE
                        tte_data_ram[15] <= wr_tte_data[42:0];
                        //tlb_entry_vld[j] <= wr_tte_tag[`STLB_TAG_V] ;
                        tlb_entry_used[15] <= wr_tte_tag[24] ;
                        tlb_entry_locked[15] = wr_tte_tag[25] ;
                    end
					// write-thru 
					rd_tte_tag[58:0]  <= wr_tte_tag[58:0] ;	// CHANGE 
					rd_tte_data[42:0] <=  wr_tte_data[42:0];
                    rd_tte_csm <= wr_tte_csm;
					end
				else
					begin
					tmp_tag[58:0]=tte_tag_ram[15]; // use non-blocking
					tmp_data[42:0]=tte_data_ram[15];
                    tmp_csm = tte_csm_ram[15];
					// INNO - read wins.
					rd_tte_tag[58:0] <=	
					{tmp_tag[58:27], tlb_entry_vld[15],tlb_entry_locked[15], 
					tlb_entry_used[15], tmp_tag[23:0]}  ;
					rd_tte_data[42:0] <= {tmp_data[42:12],tmp_data[11:0]} ;
                    rd_tte_csm <= tmp_csm;
					end
			
			end

//=========================================================================================
//	Read STLB
//=========================================================================================

        if (rw_wdline[0] & (rd_tag | rd_data | rd_csm) & ~rw_disable)
				begin
					tmp_tag  = tte_tag_ram[0] ;
					tmp_data = tte_data_ram[0] ;
                    tmp_csm = tte_csm_ram[0];
					if (rd_tag)
						rd_tte_tag[58:0] <=	// CHANGE - Bug 2185
						{tmp_tag[58:27], tlb_entry_vld[0],tlb_entry_locked[0], 
						tlb_entry_used[0], tmp_tag[23:0]}  ;
					if (rd_data) begin
						rd_tte_data[42:0] <= {tmp_data[42:12],tmp_data[11:0]} ;
					end
					if (rd_csm) begin
						rd_tte_csm <= tmp_csm ;
					end

				end
        if (rw_wdline[1] & (rd_tag | rd_data | rd_csm) & ~rw_disable)
				begin
					tmp_tag  = tte_tag_ram[1] ;
					tmp_data = tte_data_ram[1] ;
                    tmp_csm = tte_csm_ram[1];
					if (rd_tag)
						rd_tte_tag[58:0] <=	// CHANGE - Bug 2185
						{tmp_tag[58:27], tlb_entry_vld[1],tlb_entry_locked[1], 
						tlb_entry_used[1], tmp_tag[23:0]}  ;
					if (rd_data) begin
						rd_tte_data[42:0] <= {tmp_data[42:12],tmp_data[11:0]} ;
					end
					if (rd_csm) begin
						rd_tte_csm <= tmp_csm ;
					end

				end
        if (rw_wdline[2] & (rd_tag | rd_data | rd_csm) & ~rw_disable)
				begin
					tmp_tag  = tte_tag_ram[2] ;
					tmp_data = tte_data_ram[2] ;
                    tmp_csm = tte_csm_ram[2];
					if (rd_tag)
						rd_tte_tag[58:0] <=	// CHANGE - Bug 2185
						{tmp_tag[58:27], tlb_entry_vld[2],tlb_entry_locked[2], 
						tlb_entry_used[2], tmp_tag[23:0]}  ;
					if (rd_data) begin
						rd_tte_data[42:0] <= {tmp_data[42:12],tmp_data[11:0]} ;
					end
					if (rd_csm) begin
						rd_tte_csm <= tmp_csm ;
					end

				end
        if (rw_wdline[3] & (rd_tag | rd_data | rd_csm) & ~rw_disable)
				begin
					tmp_tag  = tte_tag_ram[3] ;
					tmp_data = tte_data_ram[3] ;
                    tmp_csm = tte_csm_ram[3];
					if (rd_tag)
						rd_tte_tag[58:0] <=	// CHANGE - Bug 2185
						{tmp_tag[58:27], tlb_entry_vld[3],tlb_entry_locked[3], 
						tlb_entry_used[3], tmp_tag[23:0]}  ;
					if (rd_data) begin
						rd_tte_data[42:0] <= {tmp_data[42:12],tmp_data[11:0]} ;
					end
					if (rd_csm) begin
						rd_tte_csm <= tmp_csm ;
					end

				end
        if (rw_wdline[4] & (rd_tag | rd_data | rd_csm) & ~rw_disable)
				begin
					tmp_tag  = tte_tag_ram[4] ;
					tmp_data = tte_data_ram[4] ;
                    tmp_csm = tte_csm_ram[4];
					if (rd_tag)
						rd_tte_tag[58:0] <=	// CHANGE - Bug 2185
						{tmp_tag[58:27], tlb_entry_vld[4],tlb_entry_locked[4], 
						tlb_entry_used[4], tmp_tag[23:0]}  ;
					if (rd_data) begin
						rd_tte_data[42:0] <= {tmp_data[42:12],tmp_data[11:0]} ;
					end
					if (rd_csm) begin
						rd_tte_csm <= tmp_csm ;
					end

				end
        if (rw_wdline[5] & (rd_tag | rd_data | rd_csm) & ~rw_disable)
				begin
					tmp_tag  = tte_tag_ram[5] ;
					tmp_data = tte_data_ram[5] ;
                    tmp_csm = tte_csm_ram[5];
					if (rd_tag)
						rd_tte_tag[58:0] <=	// CHANGE - Bug 2185
						{tmp_tag[58:27], tlb_entry_vld[5],tlb_entry_locked[5], 
						tlb_entry_used[5], tmp_tag[23:0]}  ;
					if (rd_data) begin
						rd_tte_data[42:0] <= {tmp_data[42:12],tmp_data[11:0]} ;
					end
					if (rd_csm) begin
						rd_tte_csm <= tmp_csm ;
					end

				end
        if (rw_wdline[6] & (rd_tag | rd_data | rd_csm) & ~rw_disable)
				begin
					tmp_tag  = tte_tag_ram[6] ;
					tmp_data = tte_data_ram[6] ;
                    tmp_csm = tte_csm_ram[6];
					if (rd_tag)
						rd_tte_tag[58:0] <=	// CHANGE - Bug 2185
						{tmp_tag[58:27], tlb_entry_vld[6],tlb_entry_locked[6], 
						tlb_entry_used[6], tmp_tag[23:0]}  ;
					if (rd_data) begin
						rd_tte_data[42:0] <= {tmp_data[42:12],tmp_data[11:0]} ;
					end
					if (rd_csm) begin
						rd_tte_csm <= tmp_csm ;
					end

				end
        if (rw_wdline[7] & (rd_tag | rd_data | rd_csm) & ~rw_disable)
				begin
					tmp_tag  = tte_tag_ram[7] ;
					tmp_data = tte_data_ram[7] ;
                    tmp_csm = tte_csm_ram[7];
					if (rd_tag)
						rd_tte_tag[58:0] <=	// CHANGE - Bug 2185
						{tmp_tag[58:27], tlb_entry_vld[7],tlb_entry_locked[7], 
						tlb_entry_used[7], tmp_tag[23:0]}  ;
					if (rd_data) begin
						rd_tte_data[42:0] <= {tmp_data[42:12],tmp_data[11:0]} ;
					end
					if (rd_csm) begin
						rd_tte_csm <= tmp_csm ;
					end

				end
        if (rw_wdline[8] & (rd_tag | rd_data | rd_csm) & ~rw_disable)
				begin
					tmp_tag  = tte_tag_ram[8] ;
					tmp_data = tte_data_ram[8] ;
                    tmp_csm = tte_csm_ram[8];
					if (rd_tag)
						rd_tte_tag[58:0] <=	// CHANGE - Bug 2185
						{tmp_tag[58:27], tlb_entry_vld[8],tlb_entry_locked[8], 
						tlb_entry_used[8], tmp_tag[23:0]}  ;
					if (rd_data) begin
						rd_tte_data[42:0] <= {tmp_data[42:12],tmp_data[11:0]} ;
					end
					if (rd_csm) begin
						rd_tte_csm <= tmp_csm ;
					end

				end
        if (rw_wdline[9] & (rd_tag | rd_data | rd_csm) & ~rw_disable)
				begin
					tmp_tag  = tte_tag_ram[9] ;
					tmp_data = tte_data_ram[9] ;
                    tmp_csm = tte_csm_ram[9];
					if (rd_tag)
						rd_tte_tag[58:0] <=	// CHANGE - Bug 2185
						{tmp_tag[58:27], tlb_entry_vld[9],tlb_entry_locked[9], 
						tlb_entry_used[9], tmp_tag[23:0]}  ;
					if (rd_data) begin
						rd_tte_data[42:0] <= {tmp_data[42:12],tmp_data[11:0]} ;
					end
					if (rd_csm) begin
						rd_tte_csm <= tmp_csm ;
					end

				end
        if (rw_wdline[10] & (rd_tag | rd_data | rd_csm) & ~rw_disable)
				begin
					tmp_tag  = tte_tag_ram[10] ;
					tmp_data = tte_data_ram[10] ;
                    tmp_csm = tte_csm_ram[10];
					if (rd_tag)
						rd_tte_tag[58:0] <=	// CHANGE - Bug 2185
						{tmp_tag[58:27], tlb_entry_vld[10],tlb_entry_locked[10], 
						tlb_entry_used[10], tmp_tag[23:0]}  ;
					if (rd_data) begin
						rd_tte_data[42:0] <= {tmp_data[42:12],tmp_data[11:0]} ;
					end
					if (rd_csm) begin
						rd_tte_csm <= tmp_csm ;
					end

				end
        if (rw_wdline[11] & (rd_tag | rd_data | rd_csm) & ~rw_disable)
				begin
					tmp_tag  = tte_tag_ram[11] ;
					tmp_data = tte_data_ram[11] ;
                    tmp_csm = tte_csm_ram[11];
					if (rd_tag)
						rd_tte_tag[58:0] <=	// CHANGE - Bug 2185
						{tmp_tag[58:27], tlb_entry_vld[11],tlb_entry_locked[11], 
						tlb_entry_used[11], tmp_tag[23:0]}  ;
					if (rd_data) begin
						rd_tte_data[42:0] <= {tmp_data[42:12],tmp_data[11:0]} ;
					end
					if (rd_csm) begin
						rd_tte_csm <= tmp_csm ;
					end

				end
        if (rw_wdline[12] & (rd_tag | rd_data | rd_csm) & ~rw_disable)
				begin
					tmp_tag  = tte_tag_ram[12] ;
					tmp_data = tte_data_ram[12] ;
                    tmp_csm = tte_csm_ram[12];
					if (rd_tag)
						rd_tte_tag[58:0] <=	// CHANGE - Bug 2185
						{tmp_tag[58:27], tlb_entry_vld[12],tlb_entry_locked[12], 
						tlb_entry_used[12], tmp_tag[23:0]}  ;
					if (rd_data) begin
						rd_tte_data[42:0] <= {tmp_data[42:12],tmp_data[11:0]} ;
					end
					if (rd_csm) begin
						rd_tte_csm <= tmp_csm ;
					end

				end
        if (rw_wdline[13] & (rd_tag | rd_data | rd_csm) & ~rw_disable)
				begin
					tmp_tag  = tte_tag_ram[13] ;
					tmp_data = tte_data_ram[13] ;
                    tmp_csm = tte_csm_ram[13];
					if (rd_tag)
						rd_tte_tag[58:0] <=	// CHANGE - Bug 2185
						{tmp_tag[58:27], tlb_entry_vld[13],tlb_entry_locked[13], 
						tlb_entry_used[13], tmp_tag[23:0]}  ;
					if (rd_data) begin
						rd_tte_data[42:0] <= {tmp_data[42:12],tmp_data[11:0]} ;
					end
					if (rd_csm) begin
						rd_tte_csm <= tmp_csm ;
					end

				end
        if (rw_wdline[14] & (rd_tag | rd_data | rd_csm) & ~rw_disable)
				begin
					tmp_tag  = tte_tag_ram[14] ;
					tmp_data = tte_data_ram[14] ;
                    tmp_csm = tte_csm_ram[14];
					if (rd_tag)
						rd_tte_tag[58:0] <=	// CHANGE - Bug 2185
						{tmp_tag[58:27], tlb_entry_vld[14],tlb_entry_locked[14], 
						tlb_entry_used[14], tmp_tag[23:0]}  ;
					if (rd_data) begin
						rd_tte_data[42:0] <= {tmp_data[42:12],tmp_data[11:0]} ;
					end
					if (rd_csm) begin
						rd_tte_csm <= tmp_csm ;
					end

				end
        if (rw_wdline[15] & (rd_tag | rd_data | rd_csm) & ~rw_disable)
				begin
					tmp_tag  = tte_tag_ram[15] ;
					tmp_data = tte_data_ram[15] ;
                    tmp_csm = tte_csm_ram[15];
					if (rd_tag)
						rd_tte_tag[58:0] <=	// CHANGE - Bug 2185
						{tmp_tag[58:27], tlb_entry_vld[15],tlb_entry_locked[15], 
						tlb_entry_used[15], tmp_tag[23:0]}  ;
					if (rd_data) begin
						rd_tte_data[42:0] <= {tmp_data[42:12],tmp_data[11:0]} ;
					end
					if (rd_csm) begin
						rd_tte_csm <= tmp_csm ;
					end

				end

		if (cam_vld & ~rw_disable)
  		begin
    			//Checking for no hit and multiple hits
    			sat = {16{1'b0}};
             begin
      				if(cam_hit[0])
      				begin
        				sat = sat + {{(16-1){1'b0}}, 1'b1};
      				end
    			end
             begin
      				if(cam_hit[1])
      				begin
        				sat = sat + {{(16-1){1'b0}}, 1'b1};
      				end
    			end
             begin
      				if(cam_hit[2])
      				begin
        				sat = sat + {{(16-1){1'b0}}, 1'b1};
      				end
    			end
             begin
      				if(cam_hit[3])
      				begin
        				sat = sat + {{(16-1){1'b0}}, 1'b1};
      				end
    			end
             begin
      				if(cam_hit[4])
      				begin
        				sat = sat + {{(16-1){1'b0}}, 1'b1};
      				end
    			end
             begin
      				if(cam_hit[5])
      				begin
        				sat = sat + {{(16-1){1'b0}}, 1'b1};
      				end
    			end
             begin
      				if(cam_hit[6])
      				begin
        				sat = sat + {{(16-1){1'b0}}, 1'b1};
      				end
    			end
             begin
      				if(cam_hit[7])
      				begin
        				sat = sat + {{(16-1){1'b0}}, 1'b1};
      				end
    			end
             begin
      				if(cam_hit[8])
      				begin
        				sat = sat + {{(16-1){1'b0}}, 1'b1};
      				end
    			end
             begin
      				if(cam_hit[9])
      				begin
        				sat = sat + {{(16-1){1'b0}}, 1'b1};
      				end
    			end
             begin
      				if(cam_hit[10])
      				begin
        				sat = sat + {{(16-1){1'b0}}, 1'b1};
      				end
    			end
             begin
      				if(cam_hit[11])
      				begin
        				sat = sat + {{(16-1){1'b0}}, 1'b1};
      				end
    			end
             begin
      				if(cam_hit[12])
      				begin
        				sat = sat + {{(16-1){1'b0}}, 1'b1};
      				end
    			end
             begin
      				if(cam_hit[13])
      				begin
        				sat = sat + {{(16-1){1'b0}}, 1'b1};
      				end
    			end
             begin
      				if(cam_hit[14])
      				begin
        				sat = sat + {{(16-1){1'b0}}, 1'b1};
      				end
    			end
             begin
      				if(cam_hit[15])
      				begin
        				sat = sat + {{(16-1){1'b0}}, 1'b1};
      				end
    			end

			// Only one hit occur read the data
    			if(sat == {{(16-1){1'b0}}, 1'b1})
    			begin
                   begin
                    if (cam_hit[0])
                    begin
                            rd_tte_data[42:0] <= tte_data_ram[0] ;
                            rd_tte_csm <= tte_csm_ram[0] ;
                    end
				end
                   begin
                    if (cam_hit[1])
                    begin
                            rd_tte_data[42:0] <= tte_data_ram[1] ;
                            rd_tte_csm <= tte_csm_ram[1] ;
                    end
				end
                   begin
                    if (cam_hit[2])
                    begin
                            rd_tte_data[42:0] <= tte_data_ram[2] ;
                            rd_tte_csm <= tte_csm_ram[2] ;
                    end
				end
                   begin
                    if (cam_hit[3])
                    begin
                            rd_tte_data[42:0] <= tte_data_ram[3] ;
                            rd_tte_csm <= tte_csm_ram[3] ;
                    end
				end
                   begin
                    if (cam_hit[4])
                    begin
                            rd_tte_data[42:0] <= tte_data_ram[4] ;
                            rd_tte_csm <= tte_csm_ram[4] ;
                    end
				end
                   begin
                    if (cam_hit[5])
                    begin
                            rd_tte_data[42:0] <= tte_data_ram[5] ;
                            rd_tte_csm <= tte_csm_ram[5] ;
                    end
				end
                   begin
                    if (cam_hit[6])
                    begin
                            rd_tte_data[42:0] <= tte_data_ram[6] ;
                            rd_tte_csm <= tte_csm_ram[6] ;
                    end
				end
                   begin
                    if (cam_hit[7])
                    begin
                            rd_tte_data[42:0] <= tte_data_ram[7] ;
                            rd_tte_csm <= tte_csm_ram[7] ;
                    end
				end
                   begin
                    if (cam_hit[8])
                    begin
                            rd_tte_data[42:0] <= tte_data_ram[8] ;
                            rd_tte_csm <= tte_csm_ram[8] ;
                    end
				end
                   begin
                    if (cam_hit[9])
                    begin
                            rd_tte_data[42:0] <= tte_data_ram[9] ;
                            rd_tte_csm <= tte_csm_ram[9] ;
                    end
				end
                   begin
                    if (cam_hit[10])
                    begin
                            rd_tte_data[42:0] <= tte_data_ram[10] ;
                            rd_tte_csm <= tte_csm_ram[10] ;
                    end
				end
                   begin
                    if (cam_hit[11])
                    begin
                            rd_tte_data[42:0] <= tte_data_ram[11] ;
                            rd_tte_csm <= tte_csm_ram[11] ;
                    end
				end
                   begin
                    if (cam_hit[12])
                    begin
                            rd_tte_data[42:0] <= tte_data_ram[12] ;
                            rd_tte_csm <= tte_csm_ram[12] ;
                    end
				end
                   begin
                    if (cam_hit[13])
                    begin
                            rd_tte_data[42:0] <= tte_data_ram[13] ;
                            rd_tte_csm <= tte_csm_ram[13] ;
                    end
				end
                   begin
                    if (cam_hit[14])
                    begin
                            rd_tte_data[42:0] <= tte_data_ram[14] ;
                            rd_tte_csm <= tte_csm_ram[14] ;
                    end
				end
                   begin
                    if (cam_hit[15])
                    begin
                            rd_tte_data[42:0] <= tte_data_ram[15] ;
                            rd_tte_csm <= tte_csm_ram[15] ;
                    end
				end

			end
			else
			begin
				// INNO - just to keep the tool happy.
				// ram cell will not be corrupted.
               begin
					if (cam_hit[0])
                    begin
                        		tte_data_ram[0] <= 43'bx ;
                        		tte_csm_ram[0] <= {33{1'bx}} ;
                    end
				end
               begin
					if (cam_hit[1])
                    begin
                        		tte_data_ram[1] <= 43'bx ;
                        		tte_csm_ram[1] <= {33{1'bx}} ;
                    end
				end
               begin
					if (cam_hit[2])
                    begin
                        		tte_data_ram[2] <= 43'bx ;
                        		tte_csm_ram[2] <= {33{1'bx}} ;
                    end
				end
               begin
					if (cam_hit[3])
                    begin
                        		tte_data_ram[3] <= 43'bx ;
                        		tte_csm_ram[3] <= {33{1'bx}} ;
                    end
				end
               begin
					if (cam_hit[4])
                    begin
                        		tte_data_ram[4] <= 43'bx ;
                        		tte_csm_ram[4] <= {33{1'bx}} ;
                    end
				end
               begin
					if (cam_hit[5])
                    begin
                        		tte_data_ram[5] <= 43'bx ;
                        		tte_csm_ram[5] <= {33{1'bx}} ;
                    end
				end
               begin
					if (cam_hit[6])
                    begin
                        		tte_data_ram[6] <= 43'bx ;
                        		tte_csm_ram[6] <= {33{1'bx}} ;
                    end
				end
               begin
					if (cam_hit[7])
                    begin
                        		tte_data_ram[7] <= 43'bx ;
                        		tte_csm_ram[7] <= {33{1'bx}} ;
                    end
				end
               begin
					if (cam_hit[8])
                    begin
                        		tte_data_ram[8] <= 43'bx ;
                        		tte_csm_ram[8] <= {33{1'bx}} ;
                    end
				end
               begin
					if (cam_hit[9])
                    begin
                        		tte_data_ram[9] <= 43'bx ;
                        		tte_csm_ram[9] <= {33{1'bx}} ;
                    end
				end
               begin
					if (cam_hit[10])
                    begin
                        		tte_data_ram[10] <= 43'bx ;
                        		tte_csm_ram[10] <= {33{1'bx}} ;
                    end
				end
               begin
					if (cam_hit[11])
                    begin
                        		tte_data_ram[11] <= 43'bx ;
                        		tte_csm_ram[11] <= {33{1'bx}} ;
                    end
				end
               begin
					if (cam_hit[12])
                    begin
                        		tte_data_ram[12] <= 43'bx ;
                        		tte_csm_ram[12] <= {33{1'bx}} ;
                    end
				end
               begin
					if (cam_hit[13])
                    begin
                        		tte_data_ram[13] <= 43'bx ;
                        		tte_csm_ram[13] <= {33{1'bx}} ;
                    end
				end
               begin
					if (cam_hit[14])
                    begin
                        		tte_data_ram[14] <= 43'bx ;
                        		tte_csm_ram[14] <= {33{1'bx}} ;
                    end
				end
               begin
					if (cam_hit[15])
                    begin
                        		tte_data_ram[15] <= 43'bx ;
                        		tte_csm_ram[15] <= {33{1'bx}} ;
                    end
				end

                rd_tte_data[42:0] <= 43'bx ;
                rd_tte_csm <= {33{1'bx}};
			end
		end

                   begin
                                if (cam_hit[0])
                                        tlb_entry_used[0] <= 1'b1;
                        end
                   begin
                                if (cam_hit[1])
                                        tlb_entry_used[1] <= 1'b1;
                        end
                   begin
                                if (cam_hit[2])
                                        tlb_entry_used[2] <= 1'b1;
                        end
                   begin
                                if (cam_hit[3])
                                        tlb_entry_used[3] <= 1'b1;
                        end
                   begin
                                if (cam_hit[4])
                                        tlb_entry_used[4] <= 1'b1;
                        end
                   begin
                                if (cam_hit[5])
                                        tlb_entry_used[5] <= 1'b1;
                        end
                   begin
                                if (cam_hit[6])
                                        tlb_entry_used[6] <= 1'b1;
                        end
                   begin
                                if (cam_hit[7])
                                        tlb_entry_used[7] <= 1'b1;
                        end
                   begin
                                if (cam_hit[8])
                                        tlb_entry_used[8] <= 1'b1;
                        end
                   begin
                                if (cam_hit[9])
                                        tlb_entry_used[9] <= 1'b1;
                        end
                   begin
                                if (cam_hit[10])
                                        tlb_entry_used[10] <= 1'b1;
                        end
                   begin
                                if (cam_hit[11])
                                        tlb_entry_used[11] <= 1'b1;
                        end
                   begin
                                if (cam_hit[12])
                                        tlb_entry_used[12] <= 1'b1;
                        end
                   begin
                                if (cam_hit[13])
                                        tlb_entry_used[13] <= 1'b1;
                        end
                   begin
                                if (cam_hit[14])
                                        tlb_entry_used[14] <= 1'b1;
                        end
                   begin
                                if (cam_hit[15])
                                        tlb_entry_used[15] <= 1'b1;
                        end

// Clear on following edge if necessary.
// CHANGE(SATYA) : tlb_entry_used qualified with valid needs to be used to determine
// whether the Used bits are to be cleared. This allows invalid entries created
// by a demap to be used for replacement. Else we will ignore these entries
// for replacement

                //if (tlb_not_writeable)
                if (~tlb_writeable & ~cam_vld & ~wr_vld & ~rd_tag & ~rst_tri_en)
                        begin
                           begin
                                                //if (~tlb_entry_locked[t])
                                                if (~tlb_entry_locked[0] & ~cam_vld & ~wr_vld)
                                                        tlb_entry_used[0] <= 1'b0;
                                        end
                           begin
                                                //if (~tlb_entry_locked[t])
                                                if (~tlb_entry_locked[1] & ~cam_vld & ~wr_vld)
                                                        tlb_entry_used[1] <= 1'b0;
                                        end
                           begin
                                                //if (~tlb_entry_locked[t])
                                                if (~tlb_entry_locked[2] & ~cam_vld & ~wr_vld)
                                                        tlb_entry_used[2] <= 1'b0;
                                        end
                           begin
                                                //if (~tlb_entry_locked[t])
                                                if (~tlb_entry_locked[3] & ~cam_vld & ~wr_vld)
                                                        tlb_entry_used[3] <= 1'b0;
                                        end
                           begin
                                                //if (~tlb_entry_locked[t])
                                                if (~tlb_entry_locked[4] & ~cam_vld & ~wr_vld)
                                                        tlb_entry_used[4] <= 1'b0;
                                        end
                           begin
                                                //if (~tlb_entry_locked[t])
                                                if (~tlb_entry_locked[5] & ~cam_vld & ~wr_vld)
                                                        tlb_entry_used[5] <= 1'b0;
                                        end
                           begin
                                                //if (~tlb_entry_locked[t])
                                                if (~tlb_entry_locked[6] & ~cam_vld & ~wr_vld)
                                                        tlb_entry_used[6] <= 1'b0;
                                        end
                           begin
                                                //if (~tlb_entry_locked[t])
                                                if (~tlb_entry_locked[7] & ~cam_vld & ~wr_vld)
                                                        tlb_entry_used[7] <= 1'b0;
                                        end
                           begin
                                                //if (~tlb_entry_locked[t])
                                                if (~tlb_entry_locked[8] & ~cam_vld & ~wr_vld)
                                                        tlb_entry_used[8] <= 1'b0;
                                        end
                           begin
                                                //if (~tlb_entry_locked[t])
                                                if (~tlb_entry_locked[9] & ~cam_vld & ~wr_vld)
                                                        tlb_entry_used[9] <= 1'b0;
                                        end
                           begin
                                                //if (~tlb_entry_locked[t])
                                                if (~tlb_entry_locked[10] & ~cam_vld & ~wr_vld)
                                                        tlb_entry_used[10] <= 1'b0;
                                        end
                           begin
                                                //if (~tlb_entry_locked[t])
                                                if (~tlb_entry_locked[11] & ~cam_vld & ~wr_vld)
                                                        tlb_entry_used[11] <= 1'b0;
                                        end
                           begin
                                                //if (~tlb_entry_locked[t])
                                                if (~tlb_entry_locked[12] & ~cam_vld & ~wr_vld)
                                                        tlb_entry_used[12] <= 1'b0;
                                        end
                           begin
                                                //if (~tlb_entry_locked[t])
                                                if (~tlb_entry_locked[13] & ~cam_vld & ~wr_vld)
                                                        tlb_entry_used[13] <= 1'b0;
                                        end
                           begin
                                                //if (~tlb_entry_locked[t])
                                                if (~tlb_entry_locked[14] & ~cam_vld & ~wr_vld)
                                                        tlb_entry_used[14] <= 1'b0;
                                        end
                           begin
                                                //if (~tlb_entry_locked[t])
                                                if (~tlb_entry_locked[15] & ~cam_vld & ~wr_vld)
                                                        tlb_entry_used[15] <= 1'b0;
                                        end

                        end
	end


always	@ *
begin
    case (cfg_csm_tlb_state)
    2'd0:
    begin
        if (~bypass)
        begin
            rd_tte_csm_real = {1'b0, rd_tte_csm[31:0]};
        end
        else
        begin
            rd_tte_csm_real = {1'b0, cfg_csm_tlb_hdid, cfg_csm_tlb_hd_size,
                               cfg_csm_tlb_sdid, cfg_csm_tlb_lsid};
        end
    end
    2'd1:
    begin
        if (~bypass)
        begin
            rd_tte_csm_real = {1'b0, rd_tte_csm[31:0]};
        end
        else
        begin
            rd_tte_csm_real = {1'b1, 2'b0, cfg_csm_tlb_chipid, cfg_csm_tlb_x,
                               cfg_csm_tlb_y};
        end
    end
    2'd2:
    begin
        rd_tte_csm_real = {1'b0, cfg_csm_tlb_hdid, cfg_csm_tlb_hd_size,
                           cfg_csm_tlb_sdid, cfg_csm_tlb_lsid};
    end
    2'd3:
    begin
        rd_tte_csm_real = {1'b1, 2'b0, cfg_csm_tlb_chipid, cfg_csm_tlb_x,
                           cfg_csm_tlb_y};
    end
    default:
    begin
        rd_tte_csm_real = {1'b0, rd_tte_csm[31:0]};
    end
    endcase
end

// Stage to next cycle.
always	@ (posedge clk)
	begin
		tlb_rd_tte_tag[58:0] 	<= rd_tte_tag[58:0] ;	// CHANGE
		tlb_rd_tte_data[42:0] 	<= rd_tte_data[42:0] ;
		tlb_rd_tte_csm 	<= rd_tte_csm_real ;
	end
assign tlb_rd_tte_csm_crit = rd_tte_csm_real;





































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































//=========================================================================================
//	CAM/DEMAP STLB for xlation
//=========================================================================================

//  no_hit logic does not work because it is set in multiple clock
//  domains and is reset before ever having a chance to be effective
//reg	no_hit ;


// Demap and CAM operation are mutually exclusive.

always  @ ( negedge clk )
	begin
	
           begin
                        /*if (demap_all)  begin
                                if (demap_auto & demap_other) ademap_hit[n]   =
                                        (~mismatch[n] & demap_all_but_locked_hit[n] & demap_other
                                                & tlb_entry_vld[n]) ;
                                end
                        else    begin */
                                if (demap_auto & demap_other) ademap_hit[0]    =
                                        (~mismatch[0] & demap_other & tlb_entry_vld[0]) ;
                                //end
			end
           begin
                        /*if (demap_all)  begin
                                if (demap_auto & demap_other) ademap_hit[n]   =
                                        (~mismatch[n] & demap_all_but_locked_hit[n] & demap_other
                                                & tlb_entry_vld[n]) ;
                                end
                        else    begin */
                                if (demap_auto & demap_other) ademap_hit[1]    =
                                        (~mismatch[1] & demap_other & tlb_entry_vld[1]) ;
                                //end
			end
           begin
                        /*if (demap_all)  begin
                                if (demap_auto & demap_other) ademap_hit[n]   =
                                        (~mismatch[n] & demap_all_but_locked_hit[n] & demap_other
                                                & tlb_entry_vld[n]) ;
                                end
                        else    begin */
                                if (demap_auto & demap_other) ademap_hit[2]    =
                                        (~mismatch[2] & demap_other & tlb_entry_vld[2]) ;
                                //end
			end
           begin
                        /*if (demap_all)  begin
                                if (demap_auto & demap_other) ademap_hit[n]   =
                                        (~mismatch[n] & demap_all_but_locked_hit[n] & demap_other
                                                & tlb_entry_vld[n]) ;
                                end
                        else    begin */
                                if (demap_auto & demap_other) ademap_hit[3]    =
                                        (~mismatch[3] & demap_other & tlb_entry_vld[3]) ;
                                //end
			end
           begin
                        /*if (demap_all)  begin
                                if (demap_auto & demap_other) ademap_hit[n]   =
                                        (~mismatch[n] & demap_all_but_locked_hit[n] & demap_other
                                                & tlb_entry_vld[n]) ;
                                end
                        else    begin */
                                if (demap_auto & demap_other) ademap_hit[4]    =
                                        (~mismatch[4] & demap_other & tlb_entry_vld[4]) ;
                                //end
			end
           begin
                        /*if (demap_all)  begin
                                if (demap_auto & demap_other) ademap_hit[n]   =
                                        (~mismatch[n] & demap_all_but_locked_hit[n] & demap_other
                                                & tlb_entry_vld[n]) ;
                                end
                        else    begin */
                                if (demap_auto & demap_other) ademap_hit[5]    =
                                        (~mismatch[5] & demap_other & tlb_entry_vld[5]) ;
                                //end
			end
           begin
                        /*if (demap_all)  begin
                                if (demap_auto & demap_other) ademap_hit[n]   =
                                        (~mismatch[n] & demap_all_but_locked_hit[n] & demap_other
                                                & tlb_entry_vld[n]) ;
                                end
                        else    begin */
                                if (demap_auto & demap_other) ademap_hit[6]    =
                                        (~mismatch[6] & demap_other & tlb_entry_vld[6]) ;
                                //end
			end
           begin
                        /*if (demap_all)  begin
                                if (demap_auto & demap_other) ademap_hit[n]   =
                                        (~mismatch[n] & demap_all_but_locked_hit[n] & demap_other
                                                & tlb_entry_vld[n]) ;
                                end
                        else    begin */
                                if (demap_auto & demap_other) ademap_hit[7]    =
                                        (~mismatch[7] & demap_other & tlb_entry_vld[7]) ;
                                //end
			end
           begin
                        /*if (demap_all)  begin
                                if (demap_auto & demap_other) ademap_hit[n]   =
                                        (~mismatch[n] & demap_all_but_locked_hit[n] & demap_other
                                                & tlb_entry_vld[n]) ;
                                end
                        else    begin */
                                if (demap_auto & demap_other) ademap_hit[8]    =
                                        (~mismatch[8] & demap_other & tlb_entry_vld[8]) ;
                                //end
			end
           begin
                        /*if (demap_all)  begin
                                if (demap_auto & demap_other) ademap_hit[n]   =
                                        (~mismatch[n] & demap_all_but_locked_hit[n] & demap_other
                                                & tlb_entry_vld[n]) ;
                                end
                        else    begin */
                                if (demap_auto & demap_other) ademap_hit[9]    =
                                        (~mismatch[9] & demap_other & tlb_entry_vld[9]) ;
                                //end
			end
           begin
                        /*if (demap_all)  begin
                                if (demap_auto & demap_other) ademap_hit[n]   =
                                        (~mismatch[n] & demap_all_but_locked_hit[n] & demap_other
                                                & tlb_entry_vld[n]) ;
                                end
                        else    begin */
                                if (demap_auto & demap_other) ademap_hit[10]    =
                                        (~mismatch[10] & demap_other & tlb_entry_vld[10]) ;
                                //end
			end
           begin
                        /*if (demap_all)  begin
                                if (demap_auto & demap_other) ademap_hit[n]   =
                                        (~mismatch[n] & demap_all_but_locked_hit[n] & demap_other
                                                & tlb_entry_vld[n]) ;
                                end
                        else    begin */
                                if (demap_auto & demap_other) ademap_hit[11]    =
                                        (~mismatch[11] & demap_other & tlb_entry_vld[11]) ;
                                //end
			end
           begin
                        /*if (demap_all)  begin
                                if (demap_auto & demap_other) ademap_hit[n]   =
                                        (~mismatch[n] & demap_all_but_locked_hit[n] & demap_other
                                                & tlb_entry_vld[n]) ;
                                end
                        else    begin */
                                if (demap_auto & demap_other) ademap_hit[12]    =
                                        (~mismatch[12] & demap_other & tlb_entry_vld[12]) ;
                                //end
			end
           begin
                        /*if (demap_all)  begin
                                if (demap_auto & demap_other) ademap_hit[n]   =
                                        (~mismatch[n] & demap_all_but_locked_hit[n] & demap_other
                                                & tlb_entry_vld[n]) ;
                                end
                        else    begin */
                                if (demap_auto & demap_other) ademap_hit[13]    =
                                        (~mismatch[13] & demap_other & tlb_entry_vld[13]) ;
                                //end
			end
           begin
                        /*if (demap_all)  begin
                                if (demap_auto & demap_other) ademap_hit[n]   =
                                        (~mismatch[n] & demap_all_but_locked_hit[n] & demap_other
                                                & tlb_entry_vld[n]) ;
                                end
                        else    begin */
                                if (demap_auto & demap_other) ademap_hit[14]    =
                                        (~mismatch[14] & demap_other & tlb_entry_vld[14]) ;
                                //end
			end
           begin
                        /*if (demap_all)  begin
                                if (demap_auto & demap_other) ademap_hit[n]   =
                                        (~mismatch[n] & demap_all_but_locked_hit[n] & demap_other
                                                & tlb_entry_vld[n]) ;
                                end
                        else    begin */
                                if (demap_auto & demap_other) ademap_hit[15]    =
                                        (~mismatch[15] & demap_other & tlb_entry_vld[15]) ;
                                //end
			end

	end  // always

always	@ (/*AUTOSENSE*/ /*memory or*/ 
           cam_data or cam_pid or cam_vld or demap_all
           or demap_other or tlb_entry_vld)
	begin
	
		    	begin
			tag[58:0] = tte_tag_ram[0] ;	// CHANGE

			mismatch_va_b47_28[0] = 
			(tag[53:34] 
			!= cam_data[40+13:21+13]);

			mismatch_va_b27_22[0] = 
			(tag[33:28] 
			!= cam_data[19+13:14+13]);

			mismatch_va_b21_16[0] = 
			(tag[23:18]
			!= cam_data[12+13:7+13]) ;

			mismatch_va_b15_13[0] = 
			(tag[16:14]
			!= cam_data[5+13:3+13]) ;

			mismatch_ctxt[0] = 
			(tag[12:0] 
			!= cam_data[12:0]) ;
			
			mismatch_pid[0] = (tag[58:56] != cam_pid[2:0]) ;

			mismatch[0] =
			(mismatch_va_b47_28[0] & cam_data[20+13]) 				|
			(mismatch_va_b27_22[0] & tag[27] & cam_data[13+13]) 	|
			(mismatch_va_b21_16[0] & tag[17] & cam_data[6+13]) 	|
			(mismatch_va_b15_13[0] & tag[13] & cam_data[2+13]) 	|
			(mismatch_ctxt[0] & ~cam_data[1+13])	|
			// mismatch is request type not equal to entry type. types are real/virtual.
			((tag[55] ^ cam_data[0+13]) & ~demap_all)  	| 
			mismatch_pid[0] ;	// pid always included in mismatch calculations

			demap_all_but_locked_hit[0] = 
			~tag[25] & demap_all ;

			cam_hit[0] 	= 
				~mismatch[0] & cam_vld   & tlb_entry_vld[0] ;

                        if (demap_all)  begin
                                // Satya(10/3) - I've simplified the demap-all equation
                                // Pls confirm that this is okay. Otherwise we will nee
                                // qualifying bits for the pid and r fields.
                                demap_hit[0]    =
                                        (~mismatch[0] & demap_all_but_locked_hit[0] & demap_other
                                                & tlb_entry_vld[0]) ;
				// qualification with demap_auto to prevent ademap_hit from
				// being cleared. Satya-we could get rid of this.
                                end
                        else    begin
                                demap_hit[0]    =
                                        (~mismatch[0] & demap_other & tlb_entry_vld[0]) ;
				// qualification with demap_auto to prevent ademap_hit from
				// being cleared. Satya-this is the only one we need.
                                end
//			no_hit = cam_vld ;
			end
		    	begin
			tag[58:0] = tte_tag_ram[1] ;	// CHANGE

			mismatch_va_b47_28[1] = 
			(tag[53:34] 
			!= cam_data[40+13:21+13]);

			mismatch_va_b27_22[1] = 
			(tag[33:28] 
			!= cam_data[19+13:14+13]);

			mismatch_va_b21_16[1] = 
			(tag[23:18]
			!= cam_data[12+13:7+13]) ;

			mismatch_va_b15_13[1] = 
			(tag[16:14]
			!= cam_data[5+13:3+13]) ;

			mismatch_ctxt[1] = 
			(tag[12:0] 
			!= cam_data[12:0]) ;
			
			mismatch_pid[1] = (tag[58:56] != cam_pid[2:0]) ;

			mismatch[1] =
			(mismatch_va_b47_28[1] & cam_data[20+13]) 				|
			(mismatch_va_b27_22[1] & tag[27] & cam_data[13+13]) 	|
			(mismatch_va_b21_16[1] & tag[17] & cam_data[6+13]) 	|
			(mismatch_va_b15_13[1] & tag[13] & cam_data[2+13]) 	|
			(mismatch_ctxt[1] & ~cam_data[1+13])	|
			// mismatch is request type not equal to entry type. types are real/virtual.
			((tag[55] ^ cam_data[0+13]) & ~demap_all)  	| 
			mismatch_pid[1] ;	// pid always included in mismatch calculations

			demap_all_but_locked_hit[1] = 
			~tag[25] & demap_all ;

			cam_hit[1] 	= 
				~mismatch[1] & cam_vld   & tlb_entry_vld[1] ;

                        if (demap_all)  begin
                                // Satya(10/3) - I've simplified the demap-all equation
                                // Pls confirm that this is okay. Otherwise we will nee
                                // qualifying bits for the pid and r fields.
                                demap_hit[1]    =
                                        (~mismatch[1] & demap_all_but_locked_hit[1] & demap_other
                                                & tlb_entry_vld[1]) ;
				// qualification with demap_auto to prevent ademap_hit from
				// being cleared. Satya-we could get rid of this.
                                end
                        else    begin
                                demap_hit[1]    =
                                        (~mismatch[1] & demap_other & tlb_entry_vld[1]) ;
				// qualification with demap_auto to prevent ademap_hit from
				// being cleared. Satya-this is the only one we need.
                                end
//			no_hit = cam_vld ;
			end
		    	begin
			tag[58:0] = tte_tag_ram[2] ;	// CHANGE

			mismatch_va_b47_28[2] = 
			(tag[53:34] 
			!= cam_data[40+13:21+13]);

			mismatch_va_b27_22[2] = 
			(tag[33:28] 
			!= cam_data[19+13:14+13]);

			mismatch_va_b21_16[2] = 
			(tag[23:18]
			!= cam_data[12+13:7+13]) ;

			mismatch_va_b15_13[2] = 
			(tag[16:14]
			!= cam_data[5+13:3+13]) ;

			mismatch_ctxt[2] = 
			(tag[12:0] 
			!= cam_data[12:0]) ;
			
			mismatch_pid[2] = (tag[58:56] != cam_pid[2:0]) ;

			mismatch[2] =
			(mismatch_va_b47_28[2] & cam_data[20+13]) 				|
			(mismatch_va_b27_22[2] & tag[27] & cam_data[13+13]) 	|
			(mismatch_va_b21_16[2] & tag[17] & cam_data[6+13]) 	|
			(mismatch_va_b15_13[2] & tag[13] & cam_data[2+13]) 	|
			(mismatch_ctxt[2] & ~cam_data[1+13])	|
			// mismatch is request type not equal to entry type. types are real/virtual.
			((tag[55] ^ cam_data[0+13]) & ~demap_all)  	| 
			mismatch_pid[2] ;	// pid always included in mismatch calculations

			demap_all_but_locked_hit[2] = 
			~tag[25] & demap_all ;

			cam_hit[2] 	= 
				~mismatch[2] & cam_vld   & tlb_entry_vld[2] ;

                        if (demap_all)  begin
                                // Satya(10/3) - I've simplified the demap-all equation
                                // Pls confirm that this is okay. Otherwise we will nee
                                // qualifying bits for the pid and r fields.
                                demap_hit[2]    =
                                        (~mismatch[2] & demap_all_but_locked_hit[2] & demap_other
                                                & tlb_entry_vld[2]) ;
				// qualification with demap_auto to prevent ademap_hit from
				// being cleared. Satya-we could get rid of this.
                                end
                        else    begin
                                demap_hit[2]    =
                                        (~mismatch[2] & demap_other & tlb_entry_vld[2]) ;
				// qualification with demap_auto to prevent ademap_hit from
				// being cleared. Satya-this is the only one we need.
                                end
//			no_hit = cam_vld ;
			end
		    	begin
			tag[58:0] = tte_tag_ram[3] ;	// CHANGE

			mismatch_va_b47_28[3] = 
			(tag[53:34] 
			!= cam_data[40+13:21+13]);

			mismatch_va_b27_22[3] = 
			(tag[33:28] 
			!= cam_data[19+13:14+13]);

			mismatch_va_b21_16[3] = 
			(tag[23:18]
			!= cam_data[12+13:7+13]) ;

			mismatch_va_b15_13[3] = 
			(tag[16:14]
			!= cam_data[5+13:3+13]) ;

			mismatch_ctxt[3] = 
			(tag[12:0] 
			!= cam_data[12:0]) ;
			
			mismatch_pid[3] = (tag[58:56] != cam_pid[2:0]) ;

			mismatch[3] =
			(mismatch_va_b47_28[3] & cam_data[20+13]) 				|
			(mismatch_va_b27_22[3] & tag[27] & cam_data[13+13]) 	|
			(mismatch_va_b21_16[3] & tag[17] & cam_data[6+13]) 	|
			(mismatch_va_b15_13[3] & tag[13] & cam_data[2+13]) 	|
			(mismatch_ctxt[3] & ~cam_data[1+13])	|
			// mismatch is request type not equal to entry type. types are real/virtual.
			((tag[55] ^ cam_data[0+13]) & ~demap_all)  	| 
			mismatch_pid[3] ;	// pid always included in mismatch calculations

			demap_all_but_locked_hit[3] = 
			~tag[25] & demap_all ;

			cam_hit[3] 	= 
				~mismatch[3] & cam_vld   & tlb_entry_vld[3] ;

                        if (demap_all)  begin
                                // Satya(10/3) - I've simplified the demap-all equation
                                // Pls confirm that this is okay. Otherwise we will nee
                                // qualifying bits for the pid and r fields.
                                demap_hit[3]    =
                                        (~mismatch[3] & demap_all_but_locked_hit[3] & demap_other
                                                & tlb_entry_vld[3]) ;
				// qualification with demap_auto to prevent ademap_hit from
				// being cleared. Satya-we could get rid of this.
                                end
                        else    begin
                                demap_hit[3]    =
                                        (~mismatch[3] & demap_other & tlb_entry_vld[3]) ;
				// qualification with demap_auto to prevent ademap_hit from
				// being cleared. Satya-this is the only one we need.
                                end
//			no_hit = cam_vld ;
			end
		    	begin
			tag[58:0] = tte_tag_ram[4] ;	// CHANGE

			mismatch_va_b47_28[4] = 
			(tag[53:34] 
			!= cam_data[40+13:21+13]);

			mismatch_va_b27_22[4] = 
			(tag[33:28] 
			!= cam_data[19+13:14+13]);

			mismatch_va_b21_16[4] = 
			(tag[23:18]
			!= cam_data[12+13:7+13]) ;

			mismatch_va_b15_13[4] = 
			(tag[16:14]
			!= cam_data[5+13:3+13]) ;

			mismatch_ctxt[4] = 
			(tag[12:0] 
			!= cam_data[12:0]) ;
			
			mismatch_pid[4] = (tag[58:56] != cam_pid[2:0]) ;

			mismatch[4] =
			(mismatch_va_b47_28[4] & cam_data[20+13]) 				|
			(mismatch_va_b27_22[4] & tag[27] & cam_data[13+13]) 	|
			(mismatch_va_b21_16[4] & tag[17] & cam_data[6+13]) 	|
			(mismatch_va_b15_13[4] & tag[13] & cam_data[2+13]) 	|
			(mismatch_ctxt[4] & ~cam_data[1+13])	|
			// mismatch is request type not equal to entry type. types are real/virtual.
			((tag[55] ^ cam_data[0+13]) & ~demap_all)  	| 
			mismatch_pid[4] ;	// pid always included in mismatch calculations

			demap_all_but_locked_hit[4] = 
			~tag[25] & demap_all ;

			cam_hit[4] 	= 
				~mismatch[4] & cam_vld   & tlb_entry_vld[4] ;

                        if (demap_all)  begin
                                // Satya(10/3) - I've simplified the demap-all equation
                                // Pls confirm that this is okay. Otherwise we will nee
                                // qualifying bits for the pid and r fields.
                                demap_hit[4]    =
                                        (~mismatch[4] & demap_all_but_locked_hit[4] & demap_other
                                                & tlb_entry_vld[4]) ;
				// qualification with demap_auto to prevent ademap_hit from
				// being cleared. Satya-we could get rid of this.
                                end
                        else    begin
                                demap_hit[4]    =
                                        (~mismatch[4] & demap_other & tlb_entry_vld[4]) ;
				// qualification with demap_auto to prevent ademap_hit from
				// being cleared. Satya-this is the only one we need.
                                end
//			no_hit = cam_vld ;
			end
		    	begin
			tag[58:0] = tte_tag_ram[5] ;	// CHANGE

			mismatch_va_b47_28[5] = 
			(tag[53:34] 
			!= cam_data[40+13:21+13]);

			mismatch_va_b27_22[5] = 
			(tag[33:28] 
			!= cam_data[19+13:14+13]);

			mismatch_va_b21_16[5] = 
			(tag[23:18]
			!= cam_data[12+13:7+13]) ;

			mismatch_va_b15_13[5] = 
			(tag[16:14]
			!= cam_data[5+13:3+13]) ;

			mismatch_ctxt[5] = 
			(tag[12:0] 
			!= cam_data[12:0]) ;
			
			mismatch_pid[5] = (tag[58:56] != cam_pid[2:0]) ;

			mismatch[5] =
			(mismatch_va_b47_28[5] & cam_data[20+13]) 				|
			(mismatch_va_b27_22[5] & tag[27] & cam_data[13+13]) 	|
			(mismatch_va_b21_16[5] & tag[17] & cam_data[6+13]) 	|
			(mismatch_va_b15_13[5] & tag[13] & cam_data[2+13]) 	|
			(mismatch_ctxt[5] & ~cam_data[1+13])	|
			// mismatch is request type not equal to entry type. types are real/virtual.
			((tag[55] ^ cam_data[0+13]) & ~demap_all)  	| 
			mismatch_pid[5] ;	// pid always included in mismatch calculations

			demap_all_but_locked_hit[5] = 
			~tag[25] & demap_all ;

			cam_hit[5] 	= 
				~mismatch[5] & cam_vld   & tlb_entry_vld[5] ;

                        if (demap_all)  begin
                                // Satya(10/3) - I've simplified the demap-all equation
                                // Pls confirm that this is okay. Otherwise we will nee
                                // qualifying bits for the pid and r fields.
                                demap_hit[5]    =
                                        (~mismatch[5] & demap_all_but_locked_hit[5] & demap_other
                                                & tlb_entry_vld[5]) ;
				// qualification with demap_auto to prevent ademap_hit from
				// being cleared. Satya-we could get rid of this.
                                end
                        else    begin
                                demap_hit[5]    =
                                        (~mismatch[5] & demap_other & tlb_entry_vld[5]) ;
				// qualification with demap_auto to prevent ademap_hit from
				// being cleared. Satya-this is the only one we need.
                                end
//			no_hit = cam_vld ;
			end
		    	begin
			tag[58:0] = tte_tag_ram[6] ;	// CHANGE

			mismatch_va_b47_28[6] = 
			(tag[53:34] 
			!= cam_data[40+13:21+13]);

			mismatch_va_b27_22[6] = 
			(tag[33:28] 
			!= cam_data[19+13:14+13]);

			mismatch_va_b21_16[6] = 
			(tag[23:18]
			!= cam_data[12+13:7+13]) ;

			mismatch_va_b15_13[6] = 
			(tag[16:14]
			!= cam_data[5+13:3+13]) ;

			mismatch_ctxt[6] = 
			(tag[12:0] 
			!= cam_data[12:0]) ;
			
			mismatch_pid[6] = (tag[58:56] != cam_pid[2:0]) ;

			mismatch[6] =
			(mismatch_va_b47_28[6] & cam_data[20+13]) 				|
			(mismatch_va_b27_22[6] & tag[27] & cam_data[13+13]) 	|
			(mismatch_va_b21_16[6] & tag[17] & cam_data[6+13]) 	|
			(mismatch_va_b15_13[6] & tag[13] & cam_data[2+13]) 	|
			(mismatch_ctxt[6] & ~cam_data[1+13])	|
			// mismatch is request type not equal to entry type. types are real/virtual.
			((tag[55] ^ cam_data[0+13]) & ~demap_all)  	| 
			mismatch_pid[6] ;	// pid always included in mismatch calculations

			demap_all_but_locked_hit[6] = 
			~tag[25] & demap_all ;

			cam_hit[6] 	= 
				~mismatch[6] & cam_vld   & tlb_entry_vld[6] ;

                        if (demap_all)  begin
                                // Satya(10/3) - I've simplified the demap-all equation
                                // Pls confirm that this is okay. Otherwise we will nee
                                // qualifying bits for the pid and r fields.
                                demap_hit[6]    =
                                        (~mismatch[6] & demap_all_but_locked_hit[6] & demap_other
                                                & tlb_entry_vld[6]) ;
				// qualification with demap_auto to prevent ademap_hit from
				// being cleared. Satya-we could get rid of this.
                                end
                        else    begin
                                demap_hit[6]    =
                                        (~mismatch[6] & demap_other & tlb_entry_vld[6]) ;
				// qualification with demap_auto to prevent ademap_hit from
				// being cleared. Satya-this is the only one we need.
                                end
//			no_hit = cam_vld ;
			end
		    	begin
			tag[58:0] = tte_tag_ram[7] ;	// CHANGE

			mismatch_va_b47_28[7] = 
			(tag[53:34] 
			!= cam_data[40+13:21+13]);

			mismatch_va_b27_22[7] = 
			(tag[33:28] 
			!= cam_data[19+13:14+13]);

			mismatch_va_b21_16[7] = 
			(tag[23:18]
			!= cam_data[12+13:7+13]) ;

			mismatch_va_b15_13[7] = 
			(tag[16:14]
			!= cam_data[5+13:3+13]) ;

			mismatch_ctxt[7] = 
			(tag[12:0] 
			!= cam_data[12:0]) ;
			
			mismatch_pid[7] = (tag[58:56] != cam_pid[2:0]) ;

			mismatch[7] =
			(mismatch_va_b47_28[7] & cam_data[20+13]) 				|
			(mismatch_va_b27_22[7] & tag[27] & cam_data[13+13]) 	|
			(mismatch_va_b21_16[7] & tag[17] & cam_data[6+13]) 	|
			(mismatch_va_b15_13[7] & tag[13] & cam_data[2+13]) 	|
			(mismatch_ctxt[7] & ~cam_data[1+13])	|
			// mismatch is request type not equal to entry type. types are real/virtual.
			((tag[55] ^ cam_data[0+13]) & ~demap_all)  	| 
			mismatch_pid[7] ;	// pid always included in mismatch calculations

			demap_all_but_locked_hit[7] = 
			~tag[25] & demap_all ;

			cam_hit[7] 	= 
				~mismatch[7] & cam_vld   & tlb_entry_vld[7] ;

                        if (demap_all)  begin
                                // Satya(10/3) - I've simplified the demap-all equation
                                // Pls confirm that this is okay. Otherwise we will nee
                                // qualifying bits for the pid and r fields.
                                demap_hit[7]    =
                                        (~mismatch[7] & demap_all_but_locked_hit[7] & demap_other
                                                & tlb_entry_vld[7]) ;
				// qualification with demap_auto to prevent ademap_hit from
				// being cleared. Satya-we could get rid of this.
                                end
                        else    begin
                                demap_hit[7]    =
                                        (~mismatch[7] & demap_other & tlb_entry_vld[7]) ;
				// qualification with demap_auto to prevent ademap_hit from
				// being cleared. Satya-this is the only one we need.
                                end
//			no_hit = cam_vld ;
			end
		    	begin
			tag[58:0] = tte_tag_ram[8] ;	// CHANGE

			mismatch_va_b47_28[8] = 
			(tag[53:34] 
			!= cam_data[40+13:21+13]);

			mismatch_va_b27_22[8] = 
			(tag[33:28] 
			!= cam_data[19+13:14+13]);

			mismatch_va_b21_16[8] = 
			(tag[23:18]
			!= cam_data[12+13:7+13]) ;

			mismatch_va_b15_13[8] = 
			(tag[16:14]
			!= cam_data[5+13:3+13]) ;

			mismatch_ctxt[8] = 
			(tag[12:0] 
			!= cam_data[12:0]) ;
			
			mismatch_pid[8] = (tag[58:56] != cam_pid[2:0]) ;

			mismatch[8] =
			(mismatch_va_b47_28[8] & cam_data[20+13]) 				|
			(mismatch_va_b27_22[8] & tag[27] & cam_data[13+13]) 	|
			(mismatch_va_b21_16[8] & tag[17] & cam_data[6+13]) 	|
			(mismatch_va_b15_13[8] & tag[13] & cam_data[2+13]) 	|
			(mismatch_ctxt[8] & ~cam_data[1+13])	|
			// mismatch is request type not equal to entry type. types are real/virtual.
			((tag[55] ^ cam_data[0+13]) & ~demap_all)  	| 
			mismatch_pid[8] ;	// pid always included in mismatch calculations

			demap_all_but_locked_hit[8] = 
			~tag[25] & demap_all ;

			cam_hit[8] 	= 
				~mismatch[8] & cam_vld   & tlb_entry_vld[8] ;

                        if (demap_all)  begin
                                // Satya(10/3) - I've simplified the demap-all equation
                                // Pls confirm that this is okay. Otherwise we will nee
                                // qualifying bits for the pid and r fields.
                                demap_hit[8]    =
                                        (~mismatch[8] & demap_all_but_locked_hit[8] & demap_other
                                                & tlb_entry_vld[8]) ;
				// qualification with demap_auto to prevent ademap_hit from
				// being cleared. Satya-we could get rid of this.
                                end
                        else    begin
                                demap_hit[8]    =
                                        (~mismatch[8] & demap_other & tlb_entry_vld[8]) ;
				// qualification with demap_auto to prevent ademap_hit from
				// being cleared. Satya-this is the only one we need.
                                end
//			no_hit = cam_vld ;
			end
		    	begin
			tag[58:0] = tte_tag_ram[9] ;	// CHANGE

			mismatch_va_b47_28[9] = 
			(tag[53:34] 
			!= cam_data[40+13:21+13]);

			mismatch_va_b27_22[9] = 
			(tag[33:28] 
			!= cam_data[19+13:14+13]);

			mismatch_va_b21_16[9] = 
			(tag[23:18]
			!= cam_data[12+13:7+13]) ;

			mismatch_va_b15_13[9] = 
			(tag[16:14]
			!= cam_data[5+13:3+13]) ;

			mismatch_ctxt[9] = 
			(tag[12:0] 
			!= cam_data[12:0]) ;
			
			mismatch_pid[9] = (tag[58:56] != cam_pid[2:0]) ;

			mismatch[9] =
			(mismatch_va_b47_28[9] & cam_data[20+13]) 				|
			(mismatch_va_b27_22[9] & tag[27] & cam_data[13+13]) 	|
			(mismatch_va_b21_16[9] & tag[17] & cam_data[6+13]) 	|
			(mismatch_va_b15_13[9] & tag[13] & cam_data[2+13]) 	|
			(mismatch_ctxt[9] & ~cam_data[1+13])	|
			// mismatch is request type not equal to entry type. types are real/virtual.
			((tag[55] ^ cam_data[0+13]) & ~demap_all)  	| 
			mismatch_pid[9] ;	// pid always included in mismatch calculations

			demap_all_but_locked_hit[9] = 
			~tag[25] & demap_all ;

			cam_hit[9] 	= 
				~mismatch[9] & cam_vld   & tlb_entry_vld[9] ;

                        if (demap_all)  begin
                                // Satya(10/3) - I've simplified the demap-all equation
                                // Pls confirm that this is okay. Otherwise we will nee
                                // qualifying bits for the pid and r fields.
                                demap_hit[9]    =
                                        (~mismatch[9] & demap_all_but_locked_hit[9] & demap_other
                                                & tlb_entry_vld[9]) ;
				// qualification with demap_auto to prevent ademap_hit from
				// being cleared. Satya-we could get rid of this.
                                end
                        else    begin
                                demap_hit[9]    =
                                        (~mismatch[9] & demap_other & tlb_entry_vld[9]) ;
				// qualification with demap_auto to prevent ademap_hit from
				// being cleared. Satya-this is the only one we need.
                                end
//			no_hit = cam_vld ;
			end
		    	begin
			tag[58:0] = tte_tag_ram[10] ;	// CHANGE

			mismatch_va_b47_28[10] = 
			(tag[53:34] 
			!= cam_data[40+13:21+13]);

			mismatch_va_b27_22[10] = 
			(tag[33:28] 
			!= cam_data[19+13:14+13]);

			mismatch_va_b21_16[10] = 
			(tag[23:18]
			!= cam_data[12+13:7+13]) ;

			mismatch_va_b15_13[10] = 
			(tag[16:14]
			!= cam_data[5+13:3+13]) ;

			mismatch_ctxt[10] = 
			(tag[12:0] 
			!= cam_data[12:0]) ;
			
			mismatch_pid[10] = (tag[58:56] != cam_pid[2:0]) ;

			mismatch[10] =
			(mismatch_va_b47_28[10] & cam_data[20+13]) 				|
			(mismatch_va_b27_22[10] & tag[27] & cam_data[13+13]) 	|
			(mismatch_va_b21_16[10] & tag[17] & cam_data[6+13]) 	|
			(mismatch_va_b15_13[10] & tag[13] & cam_data[2+13]) 	|
			(mismatch_ctxt[10] & ~cam_data[1+13])	|
			// mismatch is request type not equal to entry type. types are real/virtual.
			((tag[55] ^ cam_data[0+13]) & ~demap_all)  	| 
			mismatch_pid[10] ;	// pid always included in mismatch calculations

			demap_all_but_locked_hit[10] = 
			~tag[25] & demap_all ;

			cam_hit[10] 	= 
				~mismatch[10] & cam_vld   & tlb_entry_vld[10] ;

                        if (demap_all)  begin
                                // Satya(10/3) - I've simplified the demap-all equation
                                // Pls confirm that this is okay. Otherwise we will nee
                                // qualifying bits for the pid and r fields.
                                demap_hit[10]    =
                                        (~mismatch[10] & demap_all_but_locked_hit[10] & demap_other
                                                & tlb_entry_vld[10]) ;
				// qualification with demap_auto to prevent ademap_hit from
				// being cleared. Satya-we could get rid of this.
                                end
                        else    begin
                                demap_hit[10]    =
                                        (~mismatch[10] & demap_other & tlb_entry_vld[10]) ;
				// qualification with demap_auto to prevent ademap_hit from
				// being cleared. Satya-this is the only one we need.
                                end
//			no_hit = cam_vld ;
			end
		    	begin
			tag[58:0] = tte_tag_ram[11] ;	// CHANGE

			mismatch_va_b47_28[11] = 
			(tag[53:34] 
			!= cam_data[40+13:21+13]);

			mismatch_va_b27_22[11] = 
			(tag[33:28] 
			!= cam_data[19+13:14+13]);

			mismatch_va_b21_16[11] = 
			(tag[23:18]
			!= cam_data[12+13:7+13]) ;

			mismatch_va_b15_13[11] = 
			(tag[16:14]
			!= cam_data[5+13:3+13]) ;

			mismatch_ctxt[11] = 
			(tag[12:0] 
			!= cam_data[12:0]) ;
			
			mismatch_pid[11] = (tag[58:56] != cam_pid[2:0]) ;

			mismatch[11] =
			(mismatch_va_b47_28[11] & cam_data[20+13]) 				|
			(mismatch_va_b27_22[11] & tag[27] & cam_data[13+13]) 	|
			(mismatch_va_b21_16[11] & tag[17] & cam_data[6+13]) 	|
			(mismatch_va_b15_13[11] & tag[13] & cam_data[2+13]) 	|
			(mismatch_ctxt[11] & ~cam_data[1+13])	|
			// mismatch is request type not equal to entry type. types are real/virtual.
			((tag[55] ^ cam_data[0+13]) & ~demap_all)  	| 
			mismatch_pid[11] ;	// pid always included in mismatch calculations

			demap_all_but_locked_hit[11] = 
			~tag[25] & demap_all ;

			cam_hit[11] 	= 
				~mismatch[11] & cam_vld   & tlb_entry_vld[11] ;

                        if (demap_all)  begin
                                // Satya(10/3) - I've simplified the demap-all equation
                                // Pls confirm that this is okay. Otherwise we will nee
                                // qualifying bits for the pid and r fields.
                                demap_hit[11]    =
                                        (~mismatch[11] & demap_all_but_locked_hit[11] & demap_other
                                                & tlb_entry_vld[11]) ;
				// qualification with demap_auto to prevent ademap_hit from
				// being cleared. Satya-we could get rid of this.
                                end
                        else    begin
                                demap_hit[11]    =
                                        (~mismatch[11] & demap_other & tlb_entry_vld[11]) ;
				// qualification with demap_auto to prevent ademap_hit from
				// being cleared. Satya-this is the only one we need.
                                end
//			no_hit = cam_vld ;
			end
		    	begin
			tag[58:0] = tte_tag_ram[12] ;	// CHANGE

			mismatch_va_b47_28[12] = 
			(tag[53:34] 
			!= cam_data[40+13:21+13]);

			mismatch_va_b27_22[12] = 
			(tag[33:28] 
			!= cam_data[19+13:14+13]);

			mismatch_va_b21_16[12] = 
			(tag[23:18]
			!= cam_data[12+13:7+13]) ;

			mismatch_va_b15_13[12] = 
			(tag[16:14]
			!= cam_data[5+13:3+13]) ;

			mismatch_ctxt[12] = 
			(tag[12:0] 
			!= cam_data[12:0]) ;
			
			mismatch_pid[12] = (tag[58:56] != cam_pid[2:0]) ;

			mismatch[12] =
			(mismatch_va_b47_28[12] & cam_data[20+13]) 				|
			(mismatch_va_b27_22[12] & tag[27] & cam_data[13+13]) 	|
			(mismatch_va_b21_16[12] & tag[17] & cam_data[6+13]) 	|
			(mismatch_va_b15_13[12] & tag[13] & cam_data[2+13]) 	|
			(mismatch_ctxt[12] & ~cam_data[1+13])	|
			// mismatch is request type not equal to entry type. types are real/virtual.
			((tag[55] ^ cam_data[0+13]) & ~demap_all)  	| 
			mismatch_pid[12] ;	// pid always included in mismatch calculations

			demap_all_but_locked_hit[12] = 
			~tag[25] & demap_all ;

			cam_hit[12] 	= 
				~mismatch[12] & cam_vld   & tlb_entry_vld[12] ;

                        if (demap_all)  begin
                                // Satya(10/3) - I've simplified the demap-all equation
                                // Pls confirm that this is okay. Otherwise we will nee
                                // qualifying bits for the pid and r fields.
                                demap_hit[12]    =
                                        (~mismatch[12] & demap_all_but_locked_hit[12] & demap_other
                                                & tlb_entry_vld[12]) ;
				// qualification with demap_auto to prevent ademap_hit from
				// being cleared. Satya-we could get rid of this.
                                end
                        else    begin
                                demap_hit[12]    =
                                        (~mismatch[12] & demap_other & tlb_entry_vld[12]) ;
				// qualification with demap_auto to prevent ademap_hit from
				// being cleared. Satya-this is the only one we need.
                                end
//			no_hit = cam_vld ;
			end
		    	begin
			tag[58:0] = tte_tag_ram[13] ;	// CHANGE

			mismatch_va_b47_28[13] = 
			(tag[53:34] 
			!= cam_data[40+13:21+13]);

			mismatch_va_b27_22[13] = 
			(tag[33:28] 
			!= cam_data[19+13:14+13]);

			mismatch_va_b21_16[13] = 
			(tag[23:18]
			!= cam_data[12+13:7+13]) ;

			mismatch_va_b15_13[13] = 
			(tag[16:14]
			!= cam_data[5+13:3+13]) ;

			mismatch_ctxt[13] = 
			(tag[12:0] 
			!= cam_data[12:0]) ;
			
			mismatch_pid[13] = (tag[58:56] != cam_pid[2:0]) ;

			mismatch[13] =
			(mismatch_va_b47_28[13] & cam_data[20+13]) 				|
			(mismatch_va_b27_22[13] & tag[27] & cam_data[13+13]) 	|
			(mismatch_va_b21_16[13] & tag[17] & cam_data[6+13]) 	|
			(mismatch_va_b15_13[13] & tag[13] & cam_data[2+13]) 	|
			(mismatch_ctxt[13] & ~cam_data[1+13])	|
			// mismatch is request type not equal to entry type. types are real/virtual.
			((tag[55] ^ cam_data[0+13]) & ~demap_all)  	| 
			mismatch_pid[13] ;	// pid always included in mismatch calculations

			demap_all_but_locked_hit[13] = 
			~tag[25] & demap_all ;

			cam_hit[13] 	= 
				~mismatch[13] & cam_vld   & tlb_entry_vld[13] ;

                        if (demap_all)  begin
                                // Satya(10/3) - I've simplified the demap-all equation
                                // Pls confirm that this is okay. Otherwise we will nee
                                // qualifying bits for the pid and r fields.
                                demap_hit[13]    =
                                        (~mismatch[13] & demap_all_but_locked_hit[13] & demap_other
                                                & tlb_entry_vld[13]) ;
				// qualification with demap_auto to prevent ademap_hit from
				// being cleared. Satya-we could get rid of this.
                                end
                        else    begin
                                demap_hit[13]    =
                                        (~mismatch[13] & demap_other & tlb_entry_vld[13]) ;
				// qualification with demap_auto to prevent ademap_hit from
				// being cleared. Satya-this is the only one we need.
                                end
//			no_hit = cam_vld ;
			end
		    	begin
			tag[58:0] = tte_tag_ram[14] ;	// CHANGE

			mismatch_va_b47_28[14] = 
			(tag[53:34] 
			!= cam_data[40+13:21+13]);

			mismatch_va_b27_22[14] = 
			(tag[33:28] 
			!= cam_data[19+13:14+13]);

			mismatch_va_b21_16[14] = 
			(tag[23:18]
			!= cam_data[12+13:7+13]) ;

			mismatch_va_b15_13[14] = 
			(tag[16:14]
			!= cam_data[5+13:3+13]) ;

			mismatch_ctxt[14] = 
			(tag[12:0] 
			!= cam_data[12:0]) ;
			
			mismatch_pid[14] = (tag[58:56] != cam_pid[2:0]) ;

			mismatch[14] =
			(mismatch_va_b47_28[14] & cam_data[20+13]) 				|
			(mismatch_va_b27_22[14] & tag[27] & cam_data[13+13]) 	|
			(mismatch_va_b21_16[14] & tag[17] & cam_data[6+13]) 	|
			(mismatch_va_b15_13[14] & tag[13] & cam_data[2+13]) 	|
			(mismatch_ctxt[14] & ~cam_data[1+13])	|
			// mismatch is request type not equal to entry type. types are real/virtual.
			((tag[55] ^ cam_data[0+13]) & ~demap_all)  	| 
			mismatch_pid[14] ;	// pid always included in mismatch calculations

			demap_all_but_locked_hit[14] = 
			~tag[25] & demap_all ;

			cam_hit[14] 	= 
				~mismatch[14] & cam_vld   & tlb_entry_vld[14] ;

                        if (demap_all)  begin
                                // Satya(10/3) - I've simplified the demap-all equation
                                // Pls confirm that this is okay. Otherwise we will nee
                                // qualifying bits for the pid and r fields.
                                demap_hit[14]    =
                                        (~mismatch[14] & demap_all_but_locked_hit[14] & demap_other
                                                & tlb_entry_vld[14]) ;
				// qualification with demap_auto to prevent ademap_hit from
				// being cleared. Satya-we could get rid of this.
                                end
                        else    begin
                                demap_hit[14]    =
                                        (~mismatch[14] & demap_other & tlb_entry_vld[14]) ;
				// qualification with demap_auto to prevent ademap_hit from
				// being cleared. Satya-this is the only one we need.
                                end
//			no_hit = cam_vld ;
			end
		    	begin
			tag[58:0] = tte_tag_ram[15] ;	// CHANGE

			mismatch_va_b47_28[15] = 
			(tag[53:34] 
			!= cam_data[40+13:21+13]);

			mismatch_va_b27_22[15] = 
			(tag[33:28] 
			!= cam_data[19+13:14+13]);

			mismatch_va_b21_16[15] = 
			(tag[23:18]
			!= cam_data[12+13:7+13]) ;

			mismatch_va_b15_13[15] = 
			(tag[16:14]
			!= cam_data[5+13:3+13]) ;

			mismatch_ctxt[15] = 
			(tag[12:0] 
			!= cam_data[12:0]) ;
			
			mismatch_pid[15] = (tag[58:56] != cam_pid[2:0]) ;

			mismatch[15] =
			(mismatch_va_b47_28[15] & cam_data[20+13]) 				|
			(mismatch_va_b27_22[15] & tag[27] & cam_data[13+13]) 	|
			(mismatch_va_b21_16[15] & tag[17] & cam_data[6+13]) 	|
			(mismatch_va_b15_13[15] & tag[13] & cam_data[2+13]) 	|
			(mismatch_ctxt[15] & ~cam_data[1+13])	|
			// mismatch is request type not equal to entry type. types are real/virtual.
			((tag[55] ^ cam_data[0+13]) & ~demap_all)  	| 
			mismatch_pid[15] ;	// pid always included in mismatch calculations

			demap_all_but_locked_hit[15] = 
			~tag[25] & demap_all ;

			cam_hit[15] 	= 
				~mismatch[15] & cam_vld   & tlb_entry_vld[15] ;

                        if (demap_all)  begin
                                // Satya(10/3) - I've simplified the demap-all equation
                                // Pls confirm that this is okay. Otherwise we will nee
                                // qualifying bits for the pid and r fields.
                                demap_hit[15]    =
                                        (~mismatch[15] & demap_all_but_locked_hit[15] & demap_other
                                                & tlb_entry_vld[15]) ;
				// qualification with demap_auto to prevent ademap_hit from
				// being cleared. Satya-we could get rid of this.
                                end
                        else    begin
                                demap_hit[15]    =
                                        (~mismatch[15] & demap_other & tlb_entry_vld[15]) ;
				// qualification with demap_auto to prevent ademap_hit from
				// being cleared. Satya-this is the only one we need.
                                end
//			no_hit = cam_vld ;
			end

	end  // always

assign	tlb_cam_hit = |cam_hit[16-1:0] ;

// Read on CAM hit occurs on negedge.
/* MOVED TO COMMON ALWAYS BLOCK
always @ (negedge clk)
	begin
		if (|cam_hit[63:0])	
			begin
			for (p=0;p<64;p=p+1)
				if (cam_hit[p])	
				begin
					rd_tte_data[42:0] <= tte_data_ram[p] ;
				end
//				no_hit = 1'b0 ;
			end
//		else	if (no_hit) begin
//			rd_tte_data[42:0] <= {43{1'bx}};
//			no_hit = 1'b0 ;
//			end
	end
*/
// Change tlb_entry_vld handling for multi-threaded tlb writes.
// A write is always preceeded by an autodemap. The intent is to make the result of autodemap
// (clearing of vld bit if hit) invisible until write occurs. In the same cycle that the write
// occurs, the vld bit for an entry will be cleared if there is an autodemap hit. The write
// and admp action may even be to same entry. The write must dominate. There is no need to
// clear the dmp latches after the write/clear has occurred as the subsequent admp will set
// up new state in the latches.

// Define valid bit based on write/demap/reset.


 

always @ (negedge clk)
	begin
        if (async_reset)
        begin
            tlb_entry_vld[0] <= 1'b0 ;
        end
        else if ((demap_hit[0] & ~demap_auto) | sync_reset)   // non-auto-demap, reset
            tlb_entry_vld[0] <= 1'b0 ; 
        else if (((rw_index_vld & rw_wdline[0]) | (~rw_index_vld & tlb_entry_replace_d2[0])) & 
            wr_vld & ~ wr_csm_sel & ~rw_disable)
                tlb_entry_vld[0] <= wr_tte_tag[26] ;	// write
        else	begin
            if (ademap_hit[0] & wr_vld & ~wr_csm_sel)			// autodemap specifically
                tlb_entry_vld[0] <= 1'b0 ;		
            end
	end
    

always @ (negedge clk)
	begin
        if (async_reset)
        begin
            tlb_entry_vld[1] <= 1'b0 ;
        end
        else if ((demap_hit[1] & ~demap_auto) | sync_reset)   // non-auto-demap, reset
            tlb_entry_vld[1] <= 1'b0 ; 
        else if (((rw_index_vld & rw_wdline[1]) | (~rw_index_vld & tlb_entry_replace_d2[1])) & 
            wr_vld & ~ wr_csm_sel & ~rw_disable)
                tlb_entry_vld[1] <= wr_tte_tag[26] ;	// write
        else	begin
            if (ademap_hit[1] & wr_vld & ~wr_csm_sel)			// autodemap specifically
                tlb_entry_vld[1] <= 1'b0 ;		
            end
	end
    

always @ (negedge clk)
	begin
        if (async_reset)
        begin
            tlb_entry_vld[2] <= 1'b0 ;
        end
        else if ((demap_hit[2] & ~demap_auto) | sync_reset)   // non-auto-demap, reset
            tlb_entry_vld[2] <= 1'b0 ; 
        else if (((rw_index_vld & rw_wdline[2]) | (~rw_index_vld & tlb_entry_replace_d2[2])) & 
            wr_vld & ~ wr_csm_sel & ~rw_disable)
                tlb_entry_vld[2] <= wr_tte_tag[26] ;	// write
        else	begin
            if (ademap_hit[2] & wr_vld & ~wr_csm_sel)			// autodemap specifically
                tlb_entry_vld[2] <= 1'b0 ;		
            end
	end
    

always @ (negedge clk)
	begin
        if (async_reset)
        begin
            tlb_entry_vld[3] <= 1'b0 ;
        end
        else if ((demap_hit[3] & ~demap_auto) | sync_reset)   // non-auto-demap, reset
            tlb_entry_vld[3] <= 1'b0 ; 
        else if (((rw_index_vld & rw_wdline[3]) | (~rw_index_vld & tlb_entry_replace_d2[3])) & 
            wr_vld & ~ wr_csm_sel & ~rw_disable)
                tlb_entry_vld[3] <= wr_tte_tag[26] ;	// write
        else	begin
            if (ademap_hit[3] & wr_vld & ~wr_csm_sel)			// autodemap specifically
                tlb_entry_vld[3] <= 1'b0 ;		
            end
	end
    

always @ (negedge clk)
	begin
        if (async_reset)
        begin
            tlb_entry_vld[4] <= 1'b0 ;
        end
        else if ((demap_hit[4] & ~demap_auto) | sync_reset)   // non-auto-demap, reset
            tlb_entry_vld[4] <= 1'b0 ; 
        else if (((rw_index_vld & rw_wdline[4]) | (~rw_index_vld & tlb_entry_replace_d2[4])) & 
            wr_vld & ~ wr_csm_sel & ~rw_disable)
                tlb_entry_vld[4] <= wr_tte_tag[26] ;	// write
        else	begin
            if (ademap_hit[4] & wr_vld & ~wr_csm_sel)			// autodemap specifically
                tlb_entry_vld[4] <= 1'b0 ;		
            end
	end
    

always @ (negedge clk)
	begin
        if (async_reset)
        begin
            tlb_entry_vld[5] <= 1'b0 ;
        end
        else if ((demap_hit[5] & ~demap_auto) | sync_reset)   // non-auto-demap, reset
            tlb_entry_vld[5] <= 1'b0 ; 
        else if (((rw_index_vld & rw_wdline[5]) | (~rw_index_vld & tlb_entry_replace_d2[5])) & 
            wr_vld & ~ wr_csm_sel & ~rw_disable)
                tlb_entry_vld[5] <= wr_tte_tag[26] ;	// write
        else	begin
            if (ademap_hit[5] & wr_vld & ~wr_csm_sel)			// autodemap specifically
                tlb_entry_vld[5] <= 1'b0 ;		
            end
	end
    

always @ (negedge clk)
	begin
        if (async_reset)
        begin
            tlb_entry_vld[6] <= 1'b0 ;
        end
        else if ((demap_hit[6] & ~demap_auto) | sync_reset)   // non-auto-demap, reset
            tlb_entry_vld[6] <= 1'b0 ; 
        else if (((rw_index_vld & rw_wdline[6]) | (~rw_index_vld & tlb_entry_replace_d2[6])) & 
            wr_vld & ~ wr_csm_sel & ~rw_disable)
                tlb_entry_vld[6] <= wr_tte_tag[26] ;	// write
        else	begin
            if (ademap_hit[6] & wr_vld & ~wr_csm_sel)			// autodemap specifically
                tlb_entry_vld[6] <= 1'b0 ;		
            end
	end
    

always @ (negedge clk)
	begin
        if (async_reset)
        begin
            tlb_entry_vld[7] <= 1'b0 ;
        end
        else if ((demap_hit[7] & ~demap_auto) | sync_reset)   // non-auto-demap, reset
            tlb_entry_vld[7] <= 1'b0 ; 
        else if (((rw_index_vld & rw_wdline[7]) | (~rw_index_vld & tlb_entry_replace_d2[7])) & 
            wr_vld & ~ wr_csm_sel & ~rw_disable)
                tlb_entry_vld[7] <= wr_tte_tag[26] ;	// write
        else	begin
            if (ademap_hit[7] & wr_vld & ~wr_csm_sel)			// autodemap specifically
                tlb_entry_vld[7] <= 1'b0 ;		
            end
	end
    

always @ (negedge clk)
	begin
        if (async_reset)
        begin
            tlb_entry_vld[8] <= 1'b0 ;
        end
        else if ((demap_hit[8] & ~demap_auto) | sync_reset)   // non-auto-demap, reset
            tlb_entry_vld[8] <= 1'b0 ; 
        else if (((rw_index_vld & rw_wdline[8]) | (~rw_index_vld & tlb_entry_replace_d2[8])) & 
            wr_vld & ~ wr_csm_sel & ~rw_disable)
                tlb_entry_vld[8] <= wr_tte_tag[26] ;	// write
        else	begin
            if (ademap_hit[8] & wr_vld & ~wr_csm_sel)			// autodemap specifically
                tlb_entry_vld[8] <= 1'b0 ;		
            end
	end
    

always @ (negedge clk)
	begin
        if (async_reset)
        begin
            tlb_entry_vld[9] <= 1'b0 ;
        end
        else if ((demap_hit[9] & ~demap_auto) | sync_reset)   // non-auto-demap, reset
            tlb_entry_vld[9] <= 1'b0 ; 
        else if (((rw_index_vld & rw_wdline[9]) | (~rw_index_vld & tlb_entry_replace_d2[9])) & 
            wr_vld & ~ wr_csm_sel & ~rw_disable)
                tlb_entry_vld[9] <= wr_tte_tag[26] ;	// write
        else	begin
            if (ademap_hit[9] & wr_vld & ~wr_csm_sel)			// autodemap specifically
                tlb_entry_vld[9] <= 1'b0 ;		
            end
	end
    

always @ (negedge clk)
	begin
        if (async_reset)
        begin
            tlb_entry_vld[10] <= 1'b0 ;
        end
        else if ((demap_hit[10] & ~demap_auto) | sync_reset)   // non-auto-demap, reset
            tlb_entry_vld[10] <= 1'b0 ; 
        else if (((rw_index_vld & rw_wdline[10]) | (~rw_index_vld & tlb_entry_replace_d2[10])) & 
            wr_vld & ~ wr_csm_sel & ~rw_disable)
                tlb_entry_vld[10] <= wr_tte_tag[26] ;	// write
        else	begin
            if (ademap_hit[10] & wr_vld & ~wr_csm_sel)			// autodemap specifically
                tlb_entry_vld[10] <= 1'b0 ;		
            end
	end
    

always @ (negedge clk)
	begin
        if (async_reset)
        begin
            tlb_entry_vld[11] <= 1'b0 ;
        end
        else if ((demap_hit[11] & ~demap_auto) | sync_reset)   // non-auto-demap, reset
            tlb_entry_vld[11] <= 1'b0 ; 
        else if (((rw_index_vld & rw_wdline[11]) | (~rw_index_vld & tlb_entry_replace_d2[11])) & 
            wr_vld & ~ wr_csm_sel & ~rw_disable)
                tlb_entry_vld[11] <= wr_tte_tag[26] ;	// write
        else	begin
            if (ademap_hit[11] & wr_vld & ~wr_csm_sel)			// autodemap specifically
                tlb_entry_vld[11] <= 1'b0 ;		
            end
	end
    

always @ (negedge clk)
	begin
        if (async_reset)
        begin
            tlb_entry_vld[12] <= 1'b0 ;
        end
        else if ((demap_hit[12] & ~demap_auto) | sync_reset)   // non-auto-demap, reset
            tlb_entry_vld[12] <= 1'b0 ; 
        else if (((rw_index_vld & rw_wdline[12]) | (~rw_index_vld & tlb_entry_replace_d2[12])) & 
            wr_vld & ~ wr_csm_sel & ~rw_disable)
                tlb_entry_vld[12] <= wr_tte_tag[26] ;	// write
        else	begin
            if (ademap_hit[12] & wr_vld & ~wr_csm_sel)			// autodemap specifically
                tlb_entry_vld[12] <= 1'b0 ;		
            end
	end
    

always @ (negedge clk)
	begin
        if (async_reset)
        begin
            tlb_entry_vld[13] <= 1'b0 ;
        end
        else if ((demap_hit[13] & ~demap_auto) | sync_reset)   // non-auto-demap, reset
            tlb_entry_vld[13] <= 1'b0 ; 
        else if (((rw_index_vld & rw_wdline[13]) | (~rw_index_vld & tlb_entry_replace_d2[13])) & 
            wr_vld & ~ wr_csm_sel & ~rw_disable)
                tlb_entry_vld[13] <= wr_tte_tag[26] ;	// write
        else	begin
            if (ademap_hit[13] & wr_vld & ~wr_csm_sel)			// autodemap specifically
                tlb_entry_vld[13] <= 1'b0 ;		
            end
	end
    

always @ (negedge clk)
	begin
        if (async_reset)
        begin
            tlb_entry_vld[14] <= 1'b0 ;
        end
        else if ((demap_hit[14] & ~demap_auto) | sync_reset)   // non-auto-demap, reset
            tlb_entry_vld[14] <= 1'b0 ; 
        else if (((rw_index_vld & rw_wdline[14]) | (~rw_index_vld & tlb_entry_replace_d2[14])) & 
            wr_vld & ~ wr_csm_sel & ~rw_disable)
                tlb_entry_vld[14] <= wr_tte_tag[26] ;	// write
        else	begin
            if (ademap_hit[14] & wr_vld & ~wr_csm_sel)			// autodemap specifically
                tlb_entry_vld[14] <= 1'b0 ;		
            end
	end
    

always @ (negedge clk)
	begin
        if (async_reset)
        begin
            tlb_entry_vld[15] <= 1'b0 ;
        end
        else if ((demap_hit[15] & ~demap_auto) | sync_reset)   // non-auto-demap, reset
            tlb_entry_vld[15] <= 1'b0 ; 
        else if (((rw_index_vld & rw_wdline[15]) | (~rw_index_vld & tlb_entry_replace_d2[15])) & 
            wr_vld & ~ wr_csm_sel & ~rw_disable)
                tlb_entry_vld[15] <= wr_tte_tag[26] ;	// write
        else	begin
            if (ademap_hit[15] & wr_vld & ~wr_csm_sel)			// autodemap specifically
                tlb_entry_vld[15] <= 1'b0 ;		
            end
	end
    





































































































































































































































































































// async reset.
//always  @ (async_reset) 
//	begin
//	for (l=0;l<64;l=l+1)
//		begin
//	  	tlb_entry_vld[l] <= 1'b0 ;
//		end
//	end

//=========================================================================================
//	TAG COMPARISON
//=========================================================================================

reg [30:0] va_tag_plus ;

// Stage to m
always @(posedge clk)
		begin
		// INNO - add hold to this input
		if (hold)
			va_tag_plus[30:0] <= va_tag_plus[30:0] ;
		else
			va_tag_plus[30:0] 
			<= {tlb_cam_comp_key[26:0],tlb_bypass_va[12:10],tlb_bypass}; 
		end
			
assign vrtl_pgnum_m[29:0] = va_tag_plus[30:1] ;
assign bypass = va_tag_plus[0] ;

// Mux to bypass va or form pa tag based on tte-data.

assign	phy_pgnum_m[29:3] = 
	{rd_tte_data[41:30],
		rd_tte_data[29:24],
			rd_tte_data[22:17],
				rd_tte_data[15:13]};

// Derive the tlb-based physical address.
assign pgnum_m[2:0] = vrtl_pgnum_m[2:0];
assign pgnum_m[5:3] = (~rd_tte_data[12] & ~bypass)
				? phy_pgnum_m[5:3] : vrtl_pgnum_m[5:3] ;
assign pgnum_m[11:6] = (~rd_tte_data[16] & ~bypass)  
				? phy_pgnum_m[11:6] : vrtl_pgnum_m[11:6] ;
assign pgnum_m[17:12] = (~rd_tte_data[23] & ~bypass)
				? phy_pgnum_m[17:12] : vrtl_pgnum_m[17:12] ;
assign pgnum_m[29:18] = ~bypass ? phy_pgnum_m[29:18] : vrtl_pgnum_m[29:18];

// Stage to g
// Flop tags in tlb itself and do comparison immediately after rising edge.
// Similarly stage va/pa tag to g
always @(posedge clk)
		begin
			pgnum_g[29:0] <= pgnum_m[29:0];
			// rm hold on these inputs.
			cache_set_vld_g[2-1:0]  	<= cache_set_vld[2-1:0] ;
			// cache_ptag_w0_g[29:0] 	<= cache_ptag_w0[29:0] ;
			// cache_ptag_w1_g[29:0] 	<= cache_ptag_w1[29:0] ;
			// cache_ptag_w2_g[29:0] 	<= cache_ptag_w2[29:0] ;
			// cache_ptag_w3_g[29:0] 	<= cache_ptag_w3[29:0] ;
         
 cache_ptag_w0_g[(29+1)-1:0]   <= cache_ptag_w0[(29+1)-1:0];


 cache_ptag_w1_g[(29+1)-1:0]   <= cache_ptag_w1[(29+1)-1:0];

 
		end


// Need to stage by a cycle where used.
assign	tlb_pgnum[39:10] = pgnum_g[29:0] ;
// Same cycle as cam - meant for one load on critical path
assign	tlb_pgnum_crit[39:10] = pgnum_m[29:0] ;


// assign	cache_way_hit[0] = 
// 	(cache_ptag_w0_g[29:0] == pgnum_g[29:0]) & cache_set_vld_g[0];
// assign	cache_way_hit[1] = 
// 	(cache_ptag_w1_g[29:0] == pgnum_g[29:0]) & cache_set_vld_g[1];
// assign	cache_way_hit[2] = 
// 	(cache_ptag_w2_g[29:0] == pgnum_g[29:0]) & cache_set_vld_g[2];
// assign	cache_way_hit[3] = 
// 	(cache_ptag_w3_g[29:0] == pgnum_g[29:0]) & cache_set_vld_g[3];


    assign  cache_way_hit[0] = 
       (cache_ptag_w0_g[(29+1)-1:0] == pgnum_g[(29+1)-1:0]) & cache_set_vld_g[0];


    assign  cache_way_hit[1] = 
       (cache_ptag_w1_g[(29+1)-1:0] == pgnum_g[(29+1)-1:0]) & cache_set_vld_g[1];



assign	cache_hit = |cache_way_hit[2-1:0];


//=========================================================================================
//	TLB ENTRY REPLACEMENT
//=========================================================================================

// A single Used bit is used to track the replacement state of each entry.
// Only an unused entry can be replaced.
// An Unused entry is :
//			- an invalid entry
//			- a valid entry which has had its Used bit cleared.
//				- on write of a valid entry, the Used bit is set.
//				- The Used bit of a valid entry is cleared if all
//				entries have their Used bits set and the entry itself is not Locked.
// A locked entry should always appear to be Used.
// A single priority-encoder is required to evaluate the used status. Priority is static
// and used entry0 is of the highest priority if unused.

// Timing :
// Used bit gets updated by cam-hit or hit on negedge.
// After Used bit gets updated off negedge, the replacement entry can be generated in
// Phase2. In parallel, it is determined whether all Used bits are set or not. If
// so, then they are cleared on the next negedge with the replacement entry generated
// in the related Phase1 

// Choosing replacement entry
// Replacement entry is integer k

assign	tlb_not_writeable = &used[16-1:0] ;
/*
// Used bit can be set because of write or because of cam-hit.
always @(negedge clk)
	begin
		for (s=0;s<64;s=s+1)
			begin
				if (cam_hit[s]) 
					tlb_entry_used[s] <= 1'b1;			
			end

// Clear on following edge if necessary.
// CHANGE(SATYA) : tlb_entry_used qualified with valid needs to be used to determine
// whether the Used bits are to be cleared. This allows invalid entries created
// by a demap to be used for replacement. Else we will ignore these entries
// for replacement

		if (tlb_not_writeable)
			begin
				for (t=0;t<64;t=t+1)
					begin
						if (~tlb_entry_locked[t])
							tlb_entry_used[t] <= 1'b0;
					end
			end
	end
*/

// Determine whether entry should be squashed.

assign	used[16-1:0] = tlb_entry_used[16-1:0] & tlb_entry_vld[16-1:0] ;

/*assign squash[0] = 1'b0 ;
assign squash[1] = ~used[0] ;
assign squash[2] = |(~used[1:0]) ;
assign squash[3] = |(~used[2:0]) ;
assign squash[4] = |(~used[3:0]) ;
assign squash[5] = |(~used[4:0]) ;
assign squash[6] = |(~used[5:0]) ;
assign squash[7] = |(~used[6:0]) ;
assign squash[8] = |(~used[7:0]) ;
assign squash[9] = |(~used[8:0]) ;
assign squash[10] = |(~used[9:0]) ;
assign squash[11] = |(~used[10:0]) ;
assign squash[12] = |(~used[11:0]) ;
assign squash[13] = |(~used[12:0]) ;
assign squash[14] = |(~used[13:0]) ;
assign squash[15] = |(~used[14:0]) ;
assign squash[16] = |(~used[15:0]) ;
assign squash[17] = |(~used[16:0]) ;
assign squash[18] = |(~used[17:0]) ;
assign squash[19] = |(~used[18:0]) ;
assign squash[20] = |(~used[19:0]) ;
assign squash[21] = |(~used[20:0]) ;
assign squash[22] = |(~used[21:0]) ;
assign squash[23] = |(~used[22:0]) ;
assign squash[24] = |(~used[23:0]) ;
assign squash[25] = |(~used[24:0]) ;
assign squash[26] = |(~used[25:0]) ;
assign squash[27] = |(~used[26:0]) ;
assign squash[28] = |(~used[27:0]) ;
assign squash[29] = |(~used[28:0]) ;
assign squash[30] = |(~used[29:0]) ;
assign squash[31] = |(~used[30:0]) ;
assign squash[32] = |(~used[31:0]) ;
assign squash[33] = |(~used[32:0]) ;
assign squash[34] = |(~used[33:0]) ;
assign squash[35] = |(~used[34:0]) ;
assign squash[36] = |(~used[35:0]) ;
assign squash[37] = |(~used[36:0]) ;
assign squash[38] = |(~used[37:0]) ;
assign squash[39] = |(~used[38:0]) ;
assign squash[40] = |(~used[39:0]) ;
assign squash[41] = |(~used[40:0]) ;
assign squash[42] = |(~used[41:0]) ;
assign squash[43] = |(~used[42:0]) ;
assign squash[44] = |(~used[43:0]) ;
assign squash[45] = |(~used[44:0]) ;
assign squash[46] = |(~used[45:0]) ;
assign squash[47] = |(~used[46:0]) ;
assign squash[48] = |(~used[47:0]) ;
assign squash[49] = |(~used[48:0]) ;
assign squash[50] = |(~used[49:0]) ;
assign squash[51] = |(~used[50:0]) ;
assign squash[52] = |(~used[51:0]) ;
assign squash[53] = |(~used[52:0]) ;
assign squash[54] = |(~used[53:0]) ;
assign squash[55] = |(~used[54:0]) ;
assign squash[56] = |(~used[55:0]) ;
assign squash[57] = |(~used[56:0]) ;
assign squash[58] = |(~used[57:0]) ;
assign squash[59] = |(~used[58:0]) ;
assign squash[60] = |(~used[59:0]) ;
assign squash[61] = |(~used[60:0]) ;
assign squash[62] = |(~used[61:0]) ;
assign squash[63] = |(~used[62:0]) ; */

// Based on updated Used state, generate replacement entry.
// So, replacement entries can be generated on a cycle-by-cycle basis. 
//always @(/*AUTOSENSE*/squash or used)
	reg	[16-1:0]	tlb_entry_replace_d1;
	always @ *
	begin
  	  tlb_entry_replace_d1 = {16{1'b0}};
  	  // Priority is given to entry0

                if(~used[0])
                begin
                  tlb_entry_replace_d1[0] = ~used[0] ;
                end
         

                else if(~used[1])
                begin
                  tlb_entry_replace_d1[1] = ~used[1] ;
                end
         

                else if(~used[2])
                begin
                  tlb_entry_replace_d1[2] = ~used[2] ;
                end
         

                else if(~used[3])
                begin
                  tlb_entry_replace_d1[3] = ~used[3] ;
                end
         

                else if(~used[4])
                begin
                  tlb_entry_replace_d1[4] = ~used[4] ;
                end
         

                else if(~used[5])
                begin
                  tlb_entry_replace_d1[5] = ~used[5] ;
                end
         

                else if(~used[6])
                begin
                  tlb_entry_replace_d1[6] = ~used[6] ;
                end
         

                else if(~used[7])
                begin
                  tlb_entry_replace_d1[7] = ~used[7] ;
                end
         

                else if(~used[8])
                begin
                  tlb_entry_replace_d1[8] = ~used[8] ;
                end
         

                else if(~used[9])
                begin
                  tlb_entry_replace_d1[9] = ~used[9] ;
                end
         

                else if(~used[10])
                begin
                  tlb_entry_replace_d1[10] = ~used[10] ;
                end
         

                else if(~used[11])
                begin
                  tlb_entry_replace_d1[11] = ~used[11] ;
                end
         

                else if(~used[12])
                begin
                  tlb_entry_replace_d1[12] = ~used[12] ;
                end
         

                else if(~used[13])
                begin
                  tlb_entry_replace_d1[13] = ~used[13] ;
                end
         

                else if(~used[14])
                begin
                  tlb_entry_replace_d1[14] = ~used[14] ;
                end
         

                else if(~used[15])
                begin
                  tlb_entry_replace_d1[15] = ~used[15] ;
                end
         

  	  else
      begin
      	     tlb_entry_replace_d1[16-1] = 1'b1;
 	  end
	end
	always @(posedge clk)
	begin
	  // named in this manner to keep arch model happy.
  	  tlb_entry_replace <= tlb_entry_replace_d1 ;
	end
	// INNO - 2 stage delay before update is visible
	always @(posedge clk)
	begin
  	  tlb_entry_replace_d2 <= tlb_entry_replace ;
	end



//=========================================================================================
//	TLB WRITEABLE DETECTION
//=========================================================================================

// 2-cycles later, tlb become writeable
always @(posedge clk)
	begin
		tlb_not_writeable_d1 <= tlb_not_writeable ;
	end

always @(posedge clk)
	begin
		tlb_writeable <= ~tlb_not_writeable_d1 ;
	end

endmodule



// Modified by Princeton University on June 9th, 2015
// ========== Copyright Header Begin ==========================================
//
// OpenSPARC T1 Processor File: bw_r_ict.v
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
 //  Module Name:  bw_r_ict.v
 //  Description:
 //    Contains the RTL for the icache and dcache tag blocks.
 //    This is a 1RW 512 entry X 33b macro, with 132b rd and 132b wr,
 //    broken into 4 33b segments with its own write enable.
 //    Address and Control inputs are available the stage before
 //    array access, which is referred to as "_x".  Write data is
 //    available in the same stage as the write to the ram, referred
 //    to as "_y".  Read data is also read out and available in "_y".
 //
 //            X       |      Y
 //     index          |  ram access
 //     index sel      |  write_tag
 //     rd/wr req      |     -> read_tag
 //     way enable     |
 */


////////////////////////////////////////////////////////////////////////
// Local header file includes / local defines
////////////////////////////////////////////////////////////////////////


// devices.xml












//PITON_PROTO enables all FPGA related modifications














































































































































module bw_r_ict(rdtag_y, so, rclk, se,
  si, reset_l, sehold, rst_tri_en, index0_x, index1_x, index_sel_x,
  dec_wrway_x, rdreq_x, wrreq_x, wrtag_y,
  wrtag_x, adj,

  // sram wrapper interface
  sramid,
  srams_rtap_data,
  rtap_srams_bist_command,
  rtap_srams_bist_data
  );

  input     rclk;
  input     se;
  input     si;
  input     reset_l;
  input     sehold;
  input     rst_tri_en;
  input [6:0]   index0_x;
  input [6:0]   index1_x;
  input     index_sel_x;
  input [2-1:0]   dec_wrway_x;
  input     rdreq_x;
  input     wrreq_x;
  input [((39 - (6 + 5))+1)-1:0] wrtag_x;
  input [((39 - (6 + 5))+1)-1:0] wrtag_y;
  input [2-1:0]   adj;


  // sram wrapper interface
  output [4-1:0] srams_rtap_data;
  input  [4-1:0] rtap_srams_bist_command;
  input  [4-1:0] rtap_srams_bist_data;
  input  [8-1:0] sramid;

  output  [((((39 - (6 + 5))+1) * 2)-1):0] rdtag_y;
  output      so;

  wire      clk;
  wire  [6:0]   index_x;
  reg   [6:0]   index_y;
  wire  [2-1:0]   we;
  reg           wrreq_y;
  reg           rdreq_y;

  reg [((((39 - (6 + 5))+1) * 2)-1):0]  rdtag_sa_y; //for error_inject XMR

  assign clk = rclk;
  assign index_x = (index_sel_x ? index1_x : index0_x);
  assign we = ({2 {((wrreq_x & reset_l) & (~rst_tri_en))}} & dec_wrway_x);

  // assign write_bus_x[`IC_PHYS_TAG_WAY0_MASK] = wrtag_x;
  // assign write_bus_x[`IC_PHYS_TAG_WAY1_MASK] = wrtag_x;
  // assign write_bus_x[`IC_PHYS_TAG_WAY2_MASK] = wrtag_x;
  // assign write_bus_x[`IC_PHYS_TAG_WAY3_MASK] = wrtag_x;

  always @ (posedge rclk)
  begin
    index_y <= index_x;
    wrreq_y <= wrreq_x;
    rdreq_y <= rdreq_x;
  end

// real SRAM instance
wire [33*2-1 : 0] write_bus_mask_x = {
{33{we[1]}},
{33{we[0]}}

};

  wire [32:0] wrtag_x_phys = wrtag_x;
  wire [33*2-1 : 0] write_bus_x_phys = {2{wrtag_x_phys}};
  // wire [`IC_PHYS_TAG_MASK_ALL] write_bus_x_phys = {wrtag_x_phys, wrtag_x_phys, wrtag_x_phys, wrtag_x_phys};
  wire [33*2-1 : 0] rdtag_y_phys;

  // assign rdtag_y[`IC_TLB_TAG_WAY0_MASK] = rdtag_y_phys[`IC_PHYS_TAG_WAY0_MASK];
  // assign rdtag_y[`IC_TLB_TAG_WAY1_MASK] = rdtag_y_phys[`IC_PHYS_TAG_WAY1_MASK];
  // assign rdtag_y[`IC_TLB_TAG_WAY2_MASK] = rdtag_y_phys[`IC_PHYS_TAG_WAY2_MASK];
  // assign rdtag_y[`IC_TLB_TAG_WAY3_MASK] = rdtag_y_phys[`IC_PHYS_TAG_WAY3_MASK];

  // truncate tags from 33 bits to appropriate size
  
  wire [32:0] rdtag_y_phys_WAY0 = rdtag_y_phys[33*1-1 -: 33];
  assign rdtag_y[((39 - (6 + 5))+1)*1-1 -: ((39 - (6 + 5))+1)] = rdtag_y_phys_WAY0[(((39 - (6 + 5))+1)-1):0];
  

  wire [32:0] rdtag_y_phys_WAY1 = rdtag_y_phys[33*2-1 -: 33];
  assign rdtag_y[((39 - (6 + 5))+1)*2-1 -: ((39 - (6 + 5))+1)] = rdtag_y_phys_WAY1[(((39 - (6 + 5))+1)-1):0];
  


  sram_l1i_tag cache
  (
    .MEMCLK(rclk),
      .RESET_N(reset_l),
    .CE(wrreq_x | rdreq_x),
    .A(index_x),
    .DIN(write_bus_x_phys),
    .BW(write_bus_mask_x),
    .RDWEN(~wrreq_x),
    .DOUT(rdtag_y_phys),

    .BIST_COMMAND(rtap_srams_bist_command),
    .BIST_DIN(rtap_srams_bist_data),
    .BIST_DOUT(srams_rtap_data),
    .SRAMID(sramid)
  );

endmodule

 // IBM TAG









// Modified by Princeton University on June 9th, 2015
// ========== Copyright Header Begin ==========================================
// 
// OpenSPARC T1 Processor File: bw_r_itlb.v
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

// devices.xml


///////////////////////////////////////////////////////////////////////
/*
//	Description:	Common TLB for Instruction Fetch and Load/Stores
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







































































































































































































































































































































































































































































































































































































//PITON_PROTO enables all FPGA related modifications








//this macro will not be used
































































































































































































































































































































































































































































































































































































































































































































































































































































































































 // is default implementation in openpiton







module bw_r_itlb ( /*AUTOARG*/
   // Outputs

   
   tlb_rd_tte_csm, tlb_rd_tte_csm_crit,
   
   tlb_rd_tte_tag, tlb_rd_tte_data, tlb_pgnum, tlb_pgnum_crit, 
   tlb_cam_hit, cache_way_hit, cache_hit, so, 
   // Inputs

   
   tlb_wr_tte_csm_thrid, tlb_wr_tte_csm, tlb_rd_csm_vld, tlb_wr_csm_sel,

   cfg_csm_tlb_state, cfg_csm_tlb_chipid,cfg_csm_tlb_x,
   cfg_csm_tlb_y, cfg_csm_tlb_hdid,cfg_csm_tlb_hd_size,
   cfg_csm_tlb_sdid,cfg_csm_tlb_lsid,
   


   tlb_cam_vld, tlb_cam_key, tlb_cam_pid,  
   tlb_demap_key, tlb_addr_mask_l, tlb_ctxt, 
   tlb_wr_vld, tlb_wr_tte_tag, tlb_wr_tte_data, tlb_rd_tag_vld, 
   tlb_rd_data_vld, tlb_rw_index, tlb_rw_index_vld, tlb_demap, 
   tlb_demap_auto, tlb_demap_all, cache_ptag, 
   cache_set_vld, tlb_bypass_va, 
   tlb_bypass, se, si, hold, adj, arst_l, rst_soft_l, rclk,
   rst_tri_en
   ) ;	


input			tlb_cam_vld ;		// ld/st requires xlation. 
input	[40:0]		tlb_cam_key ;		// cam data for loads/stores;includes vld 
						// CHANGE : add real bit for cam.
input	[2:0]		tlb_cam_pid ;		// NEW: pid for cam. 
input	[40:0]		tlb_demap_key ;		// cam data for demap; includes vlds. 
						// CHANGE : add real bit for demap
input			tlb_addr_mask_l ;	// address masking occurs
input	[12:0]		tlb_ctxt ;		// context for cam xslate/demap. 
input			tlb_wr_vld;		// write to tlb. 
input	[58:0]		tlb_wr_tte_tag;		// CHANGE:tte tag to be written (55+4-1)
						// R(+1b),PID(+3b),G(-1b). 
input	[42:0]		tlb_wr_tte_data;	// tte data to be written.
						// No change(!!!) - G bit becomes spare
input			tlb_rd_tag_vld ;	// read tag
input			tlb_rd_data_vld ;	// read data
input	[5:0]		tlb_rw_index ;		// index to read/write tlb.
input			tlb_rw_index_vld ;	// indexed write else use algorithm.
input			tlb_demap ;		// demap : page/ctxt/all/auto.  
input			tlb_demap_auto ;	// demap is of type auto 
input			tlb_demap_all;		// demap-all operation : encoded separately.
input  	[((((39 - (6 + 5))+1) * 2)-1):0]    	cache_ptag;       	// way1 30b(D)/29b(I) tag.
input	[2-1:0]		cache_set_vld;       	// set vld-4 ways
input	[12:10]		tlb_bypass_va;	   	// bypass va.other va bits from cam-data
input			tlb_bypass;		// bypass tlb xslation

input			se ;			// scan-enable ; unused
input			si ;			// scan data in ; unused
input			hold ;			// scan hold signal
input	[7:0]		adj ;			// self-time adjustment ; unused
input			arst_l ;		// synchronous for tlb ; unused	
input			rst_soft_l ;		// software reset - asi
input			rclk;
input			rst_tri_en ;



input [2-1:0]   cfg_csm_tlb_state;
input [14-1:0]      cfg_csm_tlb_chipid;
input [8-1:0]           cfg_csm_tlb_x;
input [8-1:0]           cfg_csm_tlb_y;
input [10-1:0]        cfg_csm_tlb_hdid;
input [6-1:0]        cfg_csm_tlb_hd_size;
input [10-1:0]        cfg_csm_tlb_sdid;
input [6-1:0]        cfg_csm_tlb_lsid;


input [1:0]         tlb_wr_tte_csm_thrid;
input [32:0]    tlb_wr_tte_csm;
input               tlb_rd_csm_vld;
input               tlb_wr_csm_sel;
output [32:0]   tlb_rd_tte_csm;
output [32:0]   tlb_rd_tte_csm_crit;



output	[58:0]		tlb_rd_tte_tag;		// CHANGE: tte tag read from tlb.
output	[42:0]		tlb_rd_tte_data;	// tte data read from tlb.
// Need two ports for tlb_pgnum - critical and non-critical.
output	[39:10]		tlb_pgnum ;		// bypass or xslated pgnum
output	[39:10]		tlb_pgnum_crit ;	// bypass or xslated pgnum - critical
output			tlb_cam_hit ;		// xlation hits in tlb.
output	[2-1:0]		cache_way_hit;		// tag comparison results.
output			cache_hit;		// tag comparison result - 'or' of above.

//output			tlb_writeable ;		// tlb can be written in current cycle.

output			so ;		// scan data out ; unused

wire	[53:0]		tlb_cam_data ;
wire	[58:0]		wr_tte_tag ;	// CHANGE
wire	[42:0]		wr_tte_data ;
wire	[29:3]		phy_pgnum_m;
wire	[29:0]		pgnum_m;
wire 	[16-1:0]		used ;
wire			tlb_not_writeable ;
wire	[40:25] 	tlb_cam_key_masked ;
wire	[26:0]		tlb_cam_comp_key ;
wire			cam_vld ;
wire			demap_other ;
wire	[2-1:0]   	cache_way_hit ;

reg			tlb_not_writeable_d1 ;
reg			tlb_writeable ;
reg	[58:0]		tte_tag_ram  [16-1:0] ;	// CHANGE
reg	[42:0]		tte_data_ram [16-1:0] ;
reg	[16-1:0]		tlb_entry_vld ;
reg	[16-1:0]		tlb_entry_locked ;
reg	[16-1:0]		ademap_hit ;
reg	[58:0]		rd_tte_tag ;	// CHANGE
reg	[42:0]		rd_tte_data ;	
reg	[58:0]		tlb_rd_tte_tag ; // CHANGE	
reg	[42:0]		tlb_rd_tte_data ;	
reg			cam_vld_tmp ;
reg	[2:0]		cam_pid ;
reg	[53:0]		cam_data ;
reg			demap_auto, demap_other_tmp, demap_all ;
reg	[16-1:0]		mismatch ;
reg	[16-1:0]		cam_hit ;
reg	[16-1:0]		demap_hit ;
reg	[16-1:0]		demap_all_but_locked_hit ;
reg	[16-1:0]		mismatch_va_b47_28 ;
reg	[16-1:0]		mismatch_va_b27_22 ;
reg	[16-1:0]		mismatch_va_b21_16 ;
reg	[16-1:0]		mismatch_va_b15_13 ;
reg	[16-1:0]		mismatch_ctxt ;
reg	[16-1:0]		mismatch_pid ;
reg	[58:0]		tag ;	// CHANGE
reg	[16-1:0]		rw_wdline ;
reg	[16-1:0]		tlb_entry_used ;
reg	[16-1:0]		tlb_entry_replace ;
reg	[16-1:0]		tlb_entry_replace_d2 ;
reg	[29:0]		pgnum_g ;
reg     [2-1:0]		cache_set_vld_g;
reg	[((((39 - (6 + 5))+1) * 2)-1):0]		cache_ptag_g;
reg			wr_vld_tmp;
reg			rd_tag; 
reg			rd_data;
reg			rw_index_vld;
reg	[4-1:0]		rw_index;
reg	[16-1:0]		sat ;



wire	[32:0]		wr_tte_csm ;
reg	    [32:0]		tte_csm_buf [1:0];
reg	    [32:0]		tte_csm_ram [16-1:0];
reg	    [32:0]		rd_tte_csm ;	
reg	    [32:0]		rd_tte_csm_real ;	
reg	    [32:0]		tlb_rd_tte_csm ;	
reg			            rd_csm;
reg                     wr_csm_sel;
reg     [1:0]           wr_tte_csm_thrid;

wire	[29:0] 		vrtl_pgnum_m;
wire			bypass ;

wire			wr_vld ;

//integer	i,j,k,l,m,n,p,r,s,t,u,w;

/*AUTOWIRE*/
// Beginning of automatic wires (for undeclared instantiated-module outputs)
// End of automatics

// Some bits are removed from the tag and data. 
// 'U' must be defined as a '1' on a write.
// 'L' required for demap all function.
// Do not need an internal valid bit for va range 47:22.
// These bits are always valid for a page. 
// 
// TTE STLB_TAG
//
//`define	STLB_TAG_PID_HI		58	: NEW PID - bit2
//`define	STLB_TAG_PID_LO		56	: NEW PID - bit0
//`define	STLB_TAG_R		55	: NEW Real bit
//`define 	STLB_TAG_PARITY		54	// Parity kept in same posn to avoid having
//`define	STLB_TAG_VA_47_28_HI 	53	// to redo interface
//`define	STLB_TAG_VA_47_28_LO 	34
//`define	STLB_TAG_VA_27_22_HI 	33	
//`define	STLB_TAG_VA_27_22_LO 	28
//`define	STLB_TAG_27_22_V	27	
//`define	STLB_TAG_V		26	: valid for entry. Write of 0 resets it.
//`define	STLB_TAG_L		25
//`define	STLB_TAG_U		24	
//`define	STLB_TAG_VA_21_16_HI 	23
//`define	STLB_TAG_VA_21_16_LO  	18
//`define	STLB_TAG_VA_21_16_V  	17	  	
//`define	STLB_TAG_VA_15_13_HI 	16
//`define	STLB_TAG_VA_15_13_LO  	14
//`define	STLB_TAG_VA_15_13_V  	13
//`define	STLB_TAG_CTXT_12_0_HI  	12	// removed Global bit
//`define	STLB_TAG_CTXT_12_0_LO  	0
//// 				Total - 59b
////
//// TTE STLB_DATA
////
//// Soft[12:7] & Soft2[58:50] are removed.
//// Diag[49:41] are removed. Used bit used for Diag[0] on read.
//// CV is included for software correctness.
//// PA<40> is removed as it is not used.
//// G/L present in data even though present in tag : can't read out simultaneously.
//   (Unfortunately this is no longer correct. For data read, tag is also read
//   simultaneously to get valid bit, used bits).
//`define 	STLB_DATA_PARITY  	42 
//`define 	STLB_DATA_PA_39_28_HI 	41	// CHANGE
//`define 	STLB_DATA_PA_39_28_LO 	30
//`define 	STLB_DATA_PA_27_22_HI 	29	// CHANGE
//`define 	STLB_DATA_PA_27_22_LO 	24
//`define 	STLB_DATA_27_22_SEL	23
//`define 	STLB_DATA_PA_21_16_HI 	22	// CHANGE
//`define 	STLB_DATA_PA_21_16_LO 	17
//`define 	STLB_DATA_21_16_SEL	16
//`define 	STLB_DATA_PA_15_13_HI 	15	
//`define 	STLB_DATA_PA_15_13_LO 	13
//`define 	STLB_DATA_15_13_SEL	12
//`define 	STLB_DATA_V  		11	: static, does not get modified.
//`define 	STLB_DATA_NFO  		10
//`define 	STLB_DATA_IE   		9
//`define 	STLB_DATA_L 		8 	: added for read.
//`define 	STLB_DATA_CP 		7 
//`define 	STLB_DATA_CV 		6 
//`define 	STLB_DATA_E  		5 
//`define 	STLB_DATA_P  		4 
//`define 	STLB_DATA_W  		3 
//`define 	STLB_DATA_SPARE_HI  	2	: Global bit has been removed
//`define 	STLB_DATA_SPARE_LO	0  	 
// 				Total - 43b

// Valid bits for key(tlb_cam_key/tlb_demap_key).
// Total - 41b
//`define	CAM_VA_47_28_HI  	40
//`define	CAM_VA_47_28_LO  	21
//`define	CAM_VA_47_28_V  	20	// b47-28 participate in match
//`define	CAM_VA_27_22_HI  	19
//`define	CAM_VA_27_22_LO  	14
//`define	CAM_VA_27_22_V  	13	// b27-22 participate in match
//`define	CAM_VA_21_16_HI  	12
//`define	CAM_VA_21_16_LO  	7
//`define	CAM_VA_21_16_V  	6	// b21-16 participate in match
//`define	CAM_VA_15_13_HI 	5	
//`define	CAM_VA_15_13_LO 	3	
//`define	CAM_VA_15_13_V 	 	2	// b15-13 participate in match
//`define	CAM_CTXT_GK 		1	// Context participates in match
//`define	CAM_REAL_V 		0	// cam/demap applies to real mapping
					

// ctxt port is different from cam key port even though both are
// required for cam. (tlb_ctxt)
// If Gk is set then ctxt will not participate in match.
// Total - 14b




//=========================================================================================
//	What's Left :
//=========================================================================================

// Scan Insertion - scan to be ignored in formal verification for now.

//=========================================================================================
//	Design Notes.
//=========================================================================================

// - Supported Demap Operations - By Page, By Context, All But
// Locked, Autodemap, Invalidate-All i.e., reset. Demap Partition is
// not supported - it is mapped to demap-all by logic. 
// - Interpretation of demap inputs
//	- tlb_demap - this is used to signal demap by page, by ctxt
//	,all, and autodemap. 
//	- tlb_demap_ctxt - If a demap_by_ctxt operation is occuring then
//	this signal and tlb_demap must be active.
//	- tlb_demap_all - demap all operation. If a demap_all operation is
//	occuring, then tlb_demap_all must be asserted with tlb_demap. 
// - Reset is similar to demap-all except that *all* entries
// are invalidated. The action is initiated by software. The reset occurs
// on the negedge and is synchronous with the clk.
// - TTE Tag and Data
// 	- The TTE tag and data can be read together. Each will have its 
//	own bus and the muxing will occur externally. The tag needs to
//	be read on a data request to supply the valid bit.
// 	- The TTE tag and data can be written together.
// - The cam hit is a separate output signal based on the 
// the match signals.
// - Read/Write may occur based on supplied index. If not valid
// then use replacement way determined by algorithm to write.
// - Only write can use replacement way determined by algorithm.
// - Data is formatted appr. on read or write in the MMU. 
// - The TLB will generate a signal which reports whether the 
// tlb can be filled in the current cycle or not.
// **Physical Tag Comparison**
// For I-SIDE, comparison is of 28b, whereas for D-side, comparison is of 29b. The actual
// comparison, due to legacy, is for 30b.
// For the I-TLB, va[11:10] must be hardwired to the same value as the lsb of the 4 tags
// at the port level. Since the itag it only 28b, add two least significant bits to extend it to 30b.
// Similarly, for the dside, va[10] needs to be made same.	
// **Differentiating among Various TLB Operations**
// Valid bits are now associated with the key to allow selective incorporation of
// match results. The 5 valid bits are : v4(b47-28),v3(b27-22),v2(21-16),v1(b15-13)
// and Gk(G bit for auto-demap). The rules of use are :
//	- cam: v4-v1 are set high. G=~cam_real=0/1.
//	- demap_by_page : v4-v1 are set high. G=1. cam_real=0.
// 	- demap_by_ctxt : v4-v1 are low. G=1. cam_real=0
//	- demap_all : v4-v1 are don't-care. G=x. cam_real=x
//	- autodemap : v4-v1 are based on page size of incoming tte. G=~cam_real=0/1.
// Note : Gk is now used only to void a context match on a Real Translation.
// In general, if a valid bit is low then the corresponding va field will not take
// part in the match. Similarly, for the ctxt, if Gk=1, the ctxt will participate
// in the match.
//
// Demap Table (For Satya) :
// Note : To include a context match, Gk must be set to 1.
//--------------------------------------------------------------------------------------------------------
//tlb_demap tlb_demap_all  tlb_ctxt Gk	Vk4 Vk3	Vk2 Vk1 Real	Operation
//--------------------------------------------------------------------------------------------------------
//0		x		x   x	x   x	x   x   0	No demap operation
//1		0		0   1	1   1	1   1	0	Demap by page
//1		0		0   1	1   0	0   0	0/1	256M demap(auto demap)
//1		0		0   0	1   0	0   0	0	256M demap(auto demap) (*Illgl*)
//1		0		0   1	1   1	0   0	0/1	4M demap(auto demap)
//1		0		0   0	1   1	0   0	0	4M demap(auto demap) (*Illgl*)
//1		0		0   1	1   1	1   0	0/1	64k demap(auto demap)
//1		0		0   0	1   1	1   0	0	64k demap(auto demap) (*Illgl*)
//1		0		0   1	1   1	1   1	0/1	8k demap(auto demap)
//1		0		0   0	1   1	1   1	0	8k demap(auto demap) (*Illgl*)
//1		0		1   1	0   0	0   0	0	demap by ctxt
//1		1		x   x	x   x	x   x	0	demap_all
//------------------------------------------------------------------------------------------
//-----
//All other are illegal combinations
//
//=========================================================================================
//	Changes related to Hypervisor/Legacy Compatibility
//=========================================================================================
//
// - Add PID. PID does not effect demap-all. Otherwise it is included in cam, other demap
// operations and auto-demap.
// - Add R. Real translation ignores context. This is controlled externally by Gk.
// - Remove G bit for tte. Input remains in demap-key/cam-key to allow for disabling
//   of context match Real Translation  
// - Final Page Size support - 8KB,64KB,4M,256M
// - SPARC_HPV_EN has been defined to enable new tlb design support. 
// Issues : 
// -Max ptag size is now 28b. Satya, will this help the speed at all. I doubt it !

//=========================================================================================
//	Miscellaneous
//=========================================================================================
   wire clk;
   assign clk = rclk;
   
wire async_reset, sync_reset ;
assign	async_reset = ~arst_l ; 			// hardware
assign	sync_reset = (~rst_soft_l & ~rst_tri_en) ;	// software

wire rw_disable ;
// INNO - wr/rd gated off. Note required as rst_tri_en is
// asserted, but implemented in addition in schematic.
assign	rw_disable = ~arst_l | rst_tri_en ;

//=========================================================================================
// 	Stage Data
//=========================================================================================
// Apply address masking
assign	tlb_cam_key_masked[40:25]
	= {16{tlb_addr_mask_l}} & 
		tlb_cam_key[40:21+4] ;

// Reconstitute cam data CHANGE : add additional bit for real mapping
assign	tlb_cam_data[53:13] = tlb_demap ? 
	tlb_demap_key[40:0] :
	{tlb_cam_key_masked[40:25],tlb_cam_key[21+3:0]} ; 

assign tlb_cam_comp_key[26:0] = 
		tlb_demap ?
			{tlb_demap_key[32:21], tlb_demap_key[19:14],tlb_demap_key[12:7],
			tlb_demap_key[5:3]} :
			{tlb_cam_key_masked[32:25],tlb_cam_key[24:21],
			tlb_cam_key[19:14],tlb_cam_key[12:7],tlb_cam_key[5:3]} ;

assign	tlb_cam_data[12:0] = tlb_ctxt[12:0] ;

// These signals are flow-thru.
assign	wr_tte_tag[58:0] 	= tlb_wr_tte_tag[58:0] ;	// CHANGE
assign	wr_tte_data[42:0] 	= tlb_wr_tte_data[42:0] ;

assign wr_tte_csm = tlb_wr_tte_csm;



// CHANGE(SATYA) - Currently the rw_index/rw_index_vld are shared by both reads
// and writes. However, writes are done in the cycle of broadcast, whereas
// the reads are done a cycle later, as given in the model(incorrect) 
// They have to be treated uniformly. To make the model work, I've assumed the read/write 
// are done in the cycle the valids are broadcast. 
always @ (posedge clk)
	begin
	if (hold)
		begin
		cam_pid[2:0]		<= cam_pid[2:0] ;
		cam_vld_tmp		<= cam_vld_tmp ;
		cam_data[53:0] 		<= cam_data[53:0] ;
		demap_other_tmp		<= demap_other_tmp ;
		demap_auto		<= demap_auto ;
		demap_all		<= demap_all ;
		wr_vld_tmp 		<= wr_vld_tmp ;
		rd_tag 			<= rd_tag ;
		rd_data			<= rd_data ;
		rw_index_vld		<= rw_index_vld ;
		rw_index[4-1:0]		<= rw_index[4-1:0] ; 
            
        
        rd_csm          <= rd_csm;
        wr_csm_sel      <= wr_csm_sel;
        wr_tte_csm_thrid <= wr_tte_csm_thrid;
        
		end
	else
		begin
		cam_pid[2:0]		<= tlb_cam_pid[2:0] ;
		cam_vld_tmp		<= tlb_cam_vld ;
		cam_data[53:0] 		<= tlb_cam_data[53:0] ;
		demap_other_tmp		<= tlb_demap ;
		demap_auto		<= tlb_demap_auto ;
		demap_all		<= tlb_demap_all ;
		wr_vld_tmp 		<= tlb_wr_vld ;
		rd_tag 			<= tlb_rd_tag_vld ;
		rd_data			<= tlb_rd_data_vld ;
		rw_index_vld		<= tlb_rw_index_vld ;
		rw_index[4-1:0]		<= tlb_rw_index[4-1:0] ; 	
		
        
        rd_csm          <= tlb_rd_csm_vld;
        wr_csm_sel      <= tlb_wr_csm_sel;
        wr_tte_csm_thrid <= tlb_wr_tte_csm_thrid;
        
        end

	end

// INNO - gate cam,demap,wr with rst_tri_en.
reg rst_tri_en_lat;

 always        @ (clk)
 rst_tri_en_lat = rst_tri_en;

assign	cam_vld = cam_vld_tmp & ~rst_tri_en_lat ;
assign	demap_other = demap_other_tmp & ~rst_tri_en ;
assign	wr_vld = wr_vld_tmp & ~rst_tri_en ;

//=========================================================================================
//	Generate Write Wordlines
//=========================================================================================

// Based on static rw index	
// This generates the wordlines for a read/write to the tlb based on index. Wordlines for
// the write based on replacement alg. are muxed in later.


always	@ (/*AUTOSENSE*/rd_csm or rd_data or rd_tag or rw_index or rw_index_vld
           or wr_vld_tmp)
	begin
        if ((rw_index[4-1:0] == 0) & ((wr_vld_tmp & rw_index_vld) | rd_tag | rd_data | rd_csm))
         begin
            rw_wdline[0] = 1'b1;
         end
         else	
         begin
            rw_wdline[0] = 1'b0;
         end
        if ((rw_index[4-1:0] == 1) & ((wr_vld_tmp & rw_index_vld) | rd_tag | rd_data | rd_csm))
         begin
            rw_wdline[1] = 1'b1;
         end
         else	
         begin
            rw_wdline[1] = 1'b0;
         end
        if ((rw_index[4-1:0] == 2) & ((wr_vld_tmp & rw_index_vld) | rd_tag | rd_data | rd_csm))
         begin
            rw_wdline[2] = 1'b1;
         end
         else	
         begin
            rw_wdline[2] = 1'b0;
         end
        if ((rw_index[4-1:0] == 3) & ((wr_vld_tmp & rw_index_vld) | rd_tag | rd_data | rd_csm))
         begin
            rw_wdline[3] = 1'b1;
         end
         else	
         begin
            rw_wdline[3] = 1'b0;
         end
        if ((rw_index[4-1:0] == 4) & ((wr_vld_tmp & rw_index_vld) | rd_tag | rd_data | rd_csm))
         begin
            rw_wdline[4] = 1'b1;
         end
         else	
         begin
            rw_wdline[4] = 1'b0;
         end
        if ((rw_index[4-1:0] == 5) & ((wr_vld_tmp & rw_index_vld) | rd_tag | rd_data | rd_csm))
         begin
            rw_wdline[5] = 1'b1;
         end
         else	
         begin
            rw_wdline[5] = 1'b0;
         end
        if ((rw_index[4-1:0] == 6) & ((wr_vld_tmp & rw_index_vld) | rd_tag | rd_data | rd_csm))
         begin
            rw_wdline[6] = 1'b1;
         end
         else	
         begin
            rw_wdline[6] = 1'b0;
         end
        if ((rw_index[4-1:0] == 7) & ((wr_vld_tmp & rw_index_vld) | rd_tag | rd_data | rd_csm))
         begin
            rw_wdline[7] = 1'b1;
         end
         else	
         begin
            rw_wdline[7] = 1'b0;
         end
        if ((rw_index[4-1:0] == 8) & ((wr_vld_tmp & rw_index_vld) | rd_tag | rd_data | rd_csm))
         begin
            rw_wdline[8] = 1'b1;
         end
         else	
         begin
            rw_wdline[8] = 1'b0;
         end
        if ((rw_index[4-1:0] == 9) & ((wr_vld_tmp & rw_index_vld) | rd_tag | rd_data | rd_csm))
         begin
            rw_wdline[9] = 1'b1;
         end
         else	
         begin
            rw_wdline[9] = 1'b0;
         end
        if ((rw_index[4-1:0] == 10) & ((wr_vld_tmp & rw_index_vld) | rd_tag | rd_data | rd_csm))
         begin
            rw_wdline[10] = 1'b1;
         end
         else	
         begin
            rw_wdline[10] = 1'b0;
         end
        if ((rw_index[4-1:0] == 11) & ((wr_vld_tmp & rw_index_vld) | rd_tag | rd_data | rd_csm))
         begin
            rw_wdline[11] = 1'b1;
         end
         else	
         begin
            rw_wdline[11] = 1'b0;
         end
        if ((rw_index[4-1:0] == 12) & ((wr_vld_tmp & rw_index_vld) | rd_tag | rd_data | rd_csm))
         begin
            rw_wdline[12] = 1'b1;
         end
         else	
         begin
            rw_wdline[12] = 1'b0;
         end
        if ((rw_index[4-1:0] == 13) & ((wr_vld_tmp & rw_index_vld) | rd_tag | rd_data | rd_csm))
         begin
            rw_wdline[13] = 1'b1;
         end
         else	
         begin
            rw_wdline[13] = 1'b0;
         end
        if ((rw_index[4-1:0] == 14) & ((wr_vld_tmp & rw_index_vld) | rd_tag | rd_data | rd_csm))
         begin
            rw_wdline[14] = 1'b1;
         end
         else	
         begin
            rw_wdline[14] = 1'b0;
         end
        if ((rw_index[4-1:0] == 15) & ((wr_vld_tmp & rw_index_vld) | rd_tag | rd_data | rd_csm))
         begin
            rw_wdline[15] = 1'b1;
         end
         else	
         begin
            rw_wdline[15] = 1'b0;
         end

					
	end









































































































































//=========================================================================================
//	Write TLB
//=========================================================================================

reg	[58:0]	tmp_tag ;
reg	[42:0]	tmp_data ;


reg [32:0] tmp_csm;



// Currently TLB_TAG and TLB_DATA RAMs are written in the B phase. 
// Used bit is set on write in later code as it is also effected by read of tlb.
always	@ (negedge clk)
	begin
        if (((rw_index_vld & rw_wdline[0]) | (~rw_index_vld & tlb_entry_replace_d2[0])) & wr_vld_tmp & ~rw_disable)
		    begin
				if (~rst_tri_en)
					begin
                    if (wr_csm_sel)
                    begin
                        tte_csm_buf[wr_tte_csm_thrid] <= wr_tte_csm;
                    end
                    else
                    begin
                        tte_csm_ram[0] <= tte_csm_buf[wr_tte_csm_thrid];
                        tte_tag_ram[0] <= wr_tte_tag[58:0];	// CHANGE
                        tte_data_ram[0] <= wr_tte_data[42:0];
                        //tlb_entry_vld[j] <= wr_tte_tag[`STLB_TAG_V] ;
                        tlb_entry_used[0] <= wr_tte_tag[24] ;
                        tlb_entry_locked[0] = wr_tte_tag[25] ;
                    end
					// write-thru 
					rd_tte_tag[58:0]  <= wr_tte_tag[58:0] ;	// CHANGE 
					rd_tte_data[42:0] <=  wr_tte_data[42:0];
                    rd_tte_csm <= wr_tte_csm;
					end
				else
					begin
					tmp_tag[58:0]=tte_tag_ram[0]; // use non-blocking
					tmp_data[42:0]=tte_data_ram[0];
                    tmp_csm = tte_csm_ram[0];
					// INNO - read wins.
					rd_tte_tag[58:0] <=	
					{tmp_tag[58:27], tlb_entry_vld[0],tlb_entry_locked[0], 
					tlb_entry_used[0], tmp_tag[23:0]}  ;
					rd_tte_data[42:0] <= {tmp_data[42:12],tmp_data[11:0]} ;
                    rd_tte_csm <= tmp_csm;
					end
			
			end
        if (((rw_index_vld & rw_wdline[1]) | (~rw_index_vld & tlb_entry_replace_d2[1])) & wr_vld_tmp & ~rw_disable)
		    begin
				if (~rst_tri_en)
					begin
                    if (wr_csm_sel)
                    begin
                        tte_csm_buf[wr_tte_csm_thrid] <= wr_tte_csm;
                    end
                    else
                    begin
                        tte_csm_ram[1] <= tte_csm_buf[wr_tte_csm_thrid];
                        tte_tag_ram[1] <= wr_tte_tag[58:0];	// CHANGE
                        tte_data_ram[1] <= wr_tte_data[42:0];
                        //tlb_entry_vld[j] <= wr_tte_tag[`STLB_TAG_V] ;
                        tlb_entry_used[1] <= wr_tte_tag[24] ;
                        tlb_entry_locked[1] = wr_tte_tag[25] ;
                    end
					// write-thru 
					rd_tte_tag[58:0]  <= wr_tte_tag[58:0] ;	// CHANGE 
					rd_tte_data[42:0] <=  wr_tte_data[42:0];
                    rd_tte_csm <= wr_tte_csm;
					end
				else
					begin
					tmp_tag[58:0]=tte_tag_ram[1]; // use non-blocking
					tmp_data[42:0]=tte_data_ram[1];
                    tmp_csm = tte_csm_ram[1];
					// INNO - read wins.
					rd_tte_tag[58:0] <=	
					{tmp_tag[58:27], tlb_entry_vld[1],tlb_entry_locked[1], 
					tlb_entry_used[1], tmp_tag[23:0]}  ;
					rd_tte_data[42:0] <= {tmp_data[42:12],tmp_data[11:0]} ;
                    rd_tte_csm <= tmp_csm;
					end
			
			end
        if (((rw_index_vld & rw_wdline[2]) | (~rw_index_vld & tlb_entry_replace_d2[2])) & wr_vld_tmp & ~rw_disable)
		    begin
				if (~rst_tri_en)
					begin
                    if (wr_csm_sel)
                    begin
                        tte_csm_buf[wr_tte_csm_thrid] <= wr_tte_csm;
                    end
                    else
                    begin
                        tte_csm_ram[2] <= tte_csm_buf[wr_tte_csm_thrid];
                        tte_tag_ram[2] <= wr_tte_tag[58:0];	// CHANGE
                        tte_data_ram[2] <= wr_tte_data[42:0];
                        //tlb_entry_vld[j] <= wr_tte_tag[`STLB_TAG_V] ;
                        tlb_entry_used[2] <= wr_tte_tag[24] ;
                        tlb_entry_locked[2] = wr_tte_tag[25] ;
                    end
					// write-thru 
					rd_tte_tag[58:0]  <= wr_tte_tag[58:0] ;	// CHANGE 
					rd_tte_data[42:0] <=  wr_tte_data[42:0];
                    rd_tte_csm <= wr_tte_csm;
					end
				else
					begin
					tmp_tag[58:0]=tte_tag_ram[2]; // use non-blocking
					tmp_data[42:0]=tte_data_ram[2];
                    tmp_csm = tte_csm_ram[2];
					// INNO - read wins.
					rd_tte_tag[58:0] <=	
					{tmp_tag[58:27], tlb_entry_vld[2],tlb_entry_locked[2], 
					tlb_entry_used[2], tmp_tag[23:0]}  ;
					rd_tte_data[42:0] <= {tmp_data[42:12],tmp_data[11:0]} ;
                    rd_tte_csm <= tmp_csm;
					end
			
			end
        if (((rw_index_vld & rw_wdline[3]) | (~rw_index_vld & tlb_entry_replace_d2[3])) & wr_vld_tmp & ~rw_disable)
		    begin
				if (~rst_tri_en)
					begin
                    if (wr_csm_sel)
                    begin
                        tte_csm_buf[wr_tte_csm_thrid] <= wr_tte_csm;
                    end
                    else
                    begin
                        tte_csm_ram[3] <= tte_csm_buf[wr_tte_csm_thrid];
                        tte_tag_ram[3] <= wr_tte_tag[58:0];	// CHANGE
                        tte_data_ram[3] <= wr_tte_data[42:0];
                        //tlb_entry_vld[j] <= wr_tte_tag[`STLB_TAG_V] ;
                        tlb_entry_used[3] <= wr_tte_tag[24] ;
                        tlb_entry_locked[3] = wr_tte_tag[25] ;
                    end
					// write-thru 
					rd_tte_tag[58:0]  <= wr_tte_tag[58:0] ;	// CHANGE 
					rd_tte_data[42:0] <=  wr_tte_data[42:0];
                    rd_tte_csm <= wr_tte_csm;
					end
				else
					begin
					tmp_tag[58:0]=tte_tag_ram[3]; // use non-blocking
					tmp_data[42:0]=tte_data_ram[3];
                    tmp_csm = tte_csm_ram[3];
					// INNO - read wins.
					rd_tte_tag[58:0] <=	
					{tmp_tag[58:27], tlb_entry_vld[3],tlb_entry_locked[3], 
					tlb_entry_used[3], tmp_tag[23:0]}  ;
					rd_tte_data[42:0] <= {tmp_data[42:12],tmp_data[11:0]} ;
                    rd_tte_csm <= tmp_csm;
					end
			
			end
        if (((rw_index_vld & rw_wdline[4]) | (~rw_index_vld & tlb_entry_replace_d2[4])) & wr_vld_tmp & ~rw_disable)
		    begin
				if (~rst_tri_en)
					begin
                    if (wr_csm_sel)
                    begin
                        tte_csm_buf[wr_tte_csm_thrid] <= wr_tte_csm;
                    end
                    else
                    begin
                        tte_csm_ram[4] <= tte_csm_buf[wr_tte_csm_thrid];
                        tte_tag_ram[4] <= wr_tte_tag[58:0];	// CHANGE
                        tte_data_ram[4] <= wr_tte_data[42:0];
                        //tlb_entry_vld[j] <= wr_tte_tag[`STLB_TAG_V] ;
                        tlb_entry_used[4] <= wr_tte_tag[24] ;
                        tlb_entry_locked[4] = wr_tte_tag[25] ;
                    end
					// write-thru 
					rd_tte_tag[58:0]  <= wr_tte_tag[58:0] ;	// CHANGE 
					rd_tte_data[42:0] <=  wr_tte_data[42:0];
                    rd_tte_csm <= wr_tte_csm;
					end
				else
					begin
					tmp_tag[58:0]=tte_tag_ram[4]; // use non-blocking
					tmp_data[42:0]=tte_data_ram[4];
                    tmp_csm = tte_csm_ram[4];
					// INNO - read wins.
					rd_tte_tag[58:0] <=	
					{tmp_tag[58:27], tlb_entry_vld[4],tlb_entry_locked[4], 
					tlb_entry_used[4], tmp_tag[23:0]}  ;
					rd_tte_data[42:0] <= {tmp_data[42:12],tmp_data[11:0]} ;
                    rd_tte_csm <= tmp_csm;
					end
			
			end
        if (((rw_index_vld & rw_wdline[5]) | (~rw_index_vld & tlb_entry_replace_d2[5])) & wr_vld_tmp & ~rw_disable)
		    begin
				if (~rst_tri_en)
					begin
                    if (wr_csm_sel)
                    begin
                        tte_csm_buf[wr_tte_csm_thrid] <= wr_tte_csm;
                    end
                    else
                    begin
                        tte_csm_ram[5] <= tte_csm_buf[wr_tte_csm_thrid];
                        tte_tag_ram[5] <= wr_tte_tag[58:0];	// CHANGE
                        tte_data_ram[5] <= wr_tte_data[42:0];
                        //tlb_entry_vld[j] <= wr_tte_tag[`STLB_TAG_V] ;
                        tlb_entry_used[5] <= wr_tte_tag[24] ;
                        tlb_entry_locked[5] = wr_tte_tag[25] ;
                    end
					// write-thru 
					rd_tte_tag[58:0]  <= wr_tte_tag[58:0] ;	// CHANGE 
					rd_tte_data[42:0] <=  wr_tte_data[42:0];
                    rd_tte_csm <= wr_tte_csm;
					end
				else
					begin
					tmp_tag[58:0]=tte_tag_ram[5]; // use non-blocking
					tmp_data[42:0]=tte_data_ram[5];
                    tmp_csm = tte_csm_ram[5];
					// INNO - read wins.
					rd_tte_tag[58:0] <=	
					{tmp_tag[58:27], tlb_entry_vld[5],tlb_entry_locked[5], 
					tlb_entry_used[5], tmp_tag[23:0]}  ;
					rd_tte_data[42:0] <= {tmp_data[42:12],tmp_data[11:0]} ;
                    rd_tte_csm <= tmp_csm;
					end
			
			end
        if (((rw_index_vld & rw_wdline[6]) | (~rw_index_vld & tlb_entry_replace_d2[6])) & wr_vld_tmp & ~rw_disable)
		    begin
				if (~rst_tri_en)
					begin
                    if (wr_csm_sel)
                    begin
                        tte_csm_buf[wr_tte_csm_thrid] <= wr_tte_csm;
                    end
                    else
                    begin
                        tte_csm_ram[6] <= tte_csm_buf[wr_tte_csm_thrid];
                        tte_tag_ram[6] <= wr_tte_tag[58:0];	// CHANGE
                        tte_data_ram[6] <= wr_tte_data[42:0];
                        //tlb_entry_vld[j] <= wr_tte_tag[`STLB_TAG_V] ;
                        tlb_entry_used[6] <= wr_tte_tag[24] ;
                        tlb_entry_locked[6] = wr_tte_tag[25] ;
                    end
					// write-thru 
					rd_tte_tag[58:0]  <= wr_tte_tag[58:0] ;	// CHANGE 
					rd_tte_data[42:0] <=  wr_tte_data[42:0];
                    rd_tte_csm <= wr_tte_csm;
					end
				else
					begin
					tmp_tag[58:0]=tte_tag_ram[6]; // use non-blocking
					tmp_data[42:0]=tte_data_ram[6];
                    tmp_csm = tte_csm_ram[6];
					// INNO - read wins.
					rd_tte_tag[58:0] <=	
					{tmp_tag[58:27], tlb_entry_vld[6],tlb_entry_locked[6], 
					tlb_entry_used[6], tmp_tag[23:0]}  ;
					rd_tte_data[42:0] <= {tmp_data[42:12],tmp_data[11:0]} ;
                    rd_tte_csm <= tmp_csm;
					end
			
			end
        if (((rw_index_vld & rw_wdline[7]) | (~rw_index_vld & tlb_entry_replace_d2[7])) & wr_vld_tmp & ~rw_disable)
		    begin
				if (~rst_tri_en)
					begin
                    if (wr_csm_sel)
                    begin
                        tte_csm_buf[wr_tte_csm_thrid] <= wr_tte_csm;
                    end
                    else
                    begin
                        tte_csm_ram[7] <= tte_csm_buf[wr_tte_csm_thrid];
                        tte_tag_ram[7] <= wr_tte_tag[58:0];	// CHANGE
                        tte_data_ram[7] <= wr_tte_data[42:0];
                        //tlb_entry_vld[j] <= wr_tte_tag[`STLB_TAG_V] ;
                        tlb_entry_used[7] <= wr_tte_tag[24] ;
                        tlb_entry_locked[7] = wr_tte_tag[25] ;
                    end
					// write-thru 
					rd_tte_tag[58:0]  <= wr_tte_tag[58:0] ;	// CHANGE 
					rd_tte_data[42:0] <=  wr_tte_data[42:0];
                    rd_tte_csm <= wr_tte_csm;
					end
				else
					begin
					tmp_tag[58:0]=tte_tag_ram[7]; // use non-blocking
					tmp_data[42:0]=tte_data_ram[7];
                    tmp_csm = tte_csm_ram[7];
					// INNO - read wins.
					rd_tte_tag[58:0] <=	
					{tmp_tag[58:27], tlb_entry_vld[7],tlb_entry_locked[7], 
					tlb_entry_used[7], tmp_tag[23:0]}  ;
					rd_tte_data[42:0] <= {tmp_data[42:12],tmp_data[11:0]} ;
                    rd_tte_csm <= tmp_csm;
					end
			
			end
        if (((rw_index_vld & rw_wdline[8]) | (~rw_index_vld & tlb_entry_replace_d2[8])) & wr_vld_tmp & ~rw_disable)
		    begin
				if (~rst_tri_en)
					begin
                    if (wr_csm_sel)
                    begin
                        tte_csm_buf[wr_tte_csm_thrid] <= wr_tte_csm;
                    end
                    else
                    begin
                        tte_csm_ram[8] <= tte_csm_buf[wr_tte_csm_thrid];
                        tte_tag_ram[8] <= wr_tte_tag[58:0];	// CHANGE
                        tte_data_ram[8] <= wr_tte_data[42:0];
                        //tlb_entry_vld[j] <= wr_tte_tag[`STLB_TAG_V] ;
                        tlb_entry_used[8] <= wr_tte_tag[24] ;
                        tlb_entry_locked[8] = wr_tte_tag[25] ;
                    end
					// write-thru 
					rd_tte_tag[58:0]  <= wr_tte_tag[58:0] ;	// CHANGE 
					rd_tte_data[42:0] <=  wr_tte_data[42:0];
                    rd_tte_csm <= wr_tte_csm;
					end
				else
					begin
					tmp_tag[58:0]=tte_tag_ram[8]; // use non-blocking
					tmp_data[42:0]=tte_data_ram[8];
                    tmp_csm = tte_csm_ram[8];
					// INNO - read wins.
					rd_tte_tag[58:0] <=	
					{tmp_tag[58:27], tlb_entry_vld[8],tlb_entry_locked[8], 
					tlb_entry_used[8], tmp_tag[23:0]}  ;
					rd_tte_data[42:0] <= {tmp_data[42:12],tmp_data[11:0]} ;
                    rd_tte_csm <= tmp_csm;
					end
			
			end
        if (((rw_index_vld & rw_wdline[9]) | (~rw_index_vld & tlb_entry_replace_d2[9])) & wr_vld_tmp & ~rw_disable)
		    begin
				if (~rst_tri_en)
					begin
                    if (wr_csm_sel)
                    begin
                        tte_csm_buf[wr_tte_csm_thrid] <= wr_tte_csm;
                    end
                    else
                    begin
                        tte_csm_ram[9] <= tte_csm_buf[wr_tte_csm_thrid];
                        tte_tag_ram[9] <= wr_tte_tag[58:0];	// CHANGE
                        tte_data_ram[9] <= wr_tte_data[42:0];
                        //tlb_entry_vld[j] <= wr_tte_tag[`STLB_TAG_V] ;
                        tlb_entry_used[9] <= wr_tte_tag[24] ;
                        tlb_entry_locked[9] = wr_tte_tag[25] ;
                    end
					// write-thru 
					rd_tte_tag[58:0]  <= wr_tte_tag[58:0] ;	// CHANGE 
					rd_tte_data[42:0] <=  wr_tte_data[42:0];
                    rd_tte_csm <= wr_tte_csm;
					end
				else
					begin
					tmp_tag[58:0]=tte_tag_ram[9]; // use non-blocking
					tmp_data[42:0]=tte_data_ram[9];
                    tmp_csm = tte_csm_ram[9];
					// INNO - read wins.
					rd_tte_tag[58:0] <=	
					{tmp_tag[58:27], tlb_entry_vld[9],tlb_entry_locked[9], 
					tlb_entry_used[9], tmp_tag[23:0]}  ;
					rd_tte_data[42:0] <= {tmp_data[42:12],tmp_data[11:0]} ;
                    rd_tte_csm <= tmp_csm;
					end
			
			end
        if (((rw_index_vld & rw_wdline[10]) | (~rw_index_vld & tlb_entry_replace_d2[10])) & wr_vld_tmp & ~rw_disable)
		    begin
				if (~rst_tri_en)
					begin
                    if (wr_csm_sel)
                    begin
                        tte_csm_buf[wr_tte_csm_thrid] <= wr_tte_csm;
                    end
                    else
                    begin
                        tte_csm_ram[10] <= tte_csm_buf[wr_tte_csm_thrid];
                        tte_tag_ram[10] <= wr_tte_tag[58:0];	// CHANGE
                        tte_data_ram[10] <= wr_tte_data[42:0];
                        //tlb_entry_vld[j] <= wr_tte_tag[`STLB_TAG_V] ;
                        tlb_entry_used[10] <= wr_tte_tag[24] ;
                        tlb_entry_locked[10] = wr_tte_tag[25] ;
                    end
					// write-thru 
					rd_tte_tag[58:0]  <= wr_tte_tag[58:0] ;	// CHANGE 
					rd_tte_data[42:0] <=  wr_tte_data[42:0];
                    rd_tte_csm <= wr_tte_csm;
					end
				else
					begin
					tmp_tag[58:0]=tte_tag_ram[10]; // use non-blocking
					tmp_data[42:0]=tte_data_ram[10];
                    tmp_csm = tte_csm_ram[10];
					// INNO - read wins.
					rd_tte_tag[58:0] <=	
					{tmp_tag[58:27], tlb_entry_vld[10],tlb_entry_locked[10], 
					tlb_entry_used[10], tmp_tag[23:0]}  ;
					rd_tte_data[42:0] <= {tmp_data[42:12],tmp_data[11:0]} ;
                    rd_tte_csm <= tmp_csm;
					end
			
			end
        if (((rw_index_vld & rw_wdline[11]) | (~rw_index_vld & tlb_entry_replace_d2[11])) & wr_vld_tmp & ~rw_disable)
		    begin
				if (~rst_tri_en)
					begin
                    if (wr_csm_sel)
                    begin
                        tte_csm_buf[wr_tte_csm_thrid] <= wr_tte_csm;
                    end
                    else
                    begin
                        tte_csm_ram[11] <= tte_csm_buf[wr_tte_csm_thrid];
                        tte_tag_ram[11] <= wr_tte_tag[58:0];	// CHANGE
                        tte_data_ram[11] <= wr_tte_data[42:0];
                        //tlb_entry_vld[j] <= wr_tte_tag[`STLB_TAG_V] ;
                        tlb_entry_used[11] <= wr_tte_tag[24] ;
                        tlb_entry_locked[11] = wr_tte_tag[25] ;
                    end
					// write-thru 
					rd_tte_tag[58:0]  <= wr_tte_tag[58:0] ;	// CHANGE 
					rd_tte_data[42:0] <=  wr_tte_data[42:0];
                    rd_tte_csm <= wr_tte_csm;
					end
				else
					begin
					tmp_tag[58:0]=tte_tag_ram[11]; // use non-blocking
					tmp_data[42:0]=tte_data_ram[11];
                    tmp_csm = tte_csm_ram[11];
					// INNO - read wins.
					rd_tte_tag[58:0] <=	
					{tmp_tag[58:27], tlb_entry_vld[11],tlb_entry_locked[11], 
					tlb_entry_used[11], tmp_tag[23:0]}  ;
					rd_tte_data[42:0] <= {tmp_data[42:12],tmp_data[11:0]} ;
                    rd_tte_csm <= tmp_csm;
					end
			
			end
        if (((rw_index_vld & rw_wdline[12]) | (~rw_index_vld & tlb_entry_replace_d2[12])) & wr_vld_tmp & ~rw_disable)
		    begin
				if (~rst_tri_en)
					begin
                    if (wr_csm_sel)
                    begin
                        tte_csm_buf[wr_tte_csm_thrid] <= wr_tte_csm;
                    end
                    else
                    begin
                        tte_csm_ram[12] <= tte_csm_buf[wr_tte_csm_thrid];
                        tte_tag_ram[12] <= wr_tte_tag[58:0];	// CHANGE
                        tte_data_ram[12] <= wr_tte_data[42:0];
                        //tlb_entry_vld[j] <= wr_tte_tag[`STLB_TAG_V] ;
                        tlb_entry_used[12] <= wr_tte_tag[24] ;
                        tlb_entry_locked[12] = wr_tte_tag[25] ;
                    end
					// write-thru 
					rd_tte_tag[58:0]  <= wr_tte_tag[58:0] ;	// CHANGE 
					rd_tte_data[42:0] <=  wr_tte_data[42:0];
                    rd_tte_csm <= wr_tte_csm;
					end
				else
					begin
					tmp_tag[58:0]=tte_tag_ram[12]; // use non-blocking
					tmp_data[42:0]=tte_data_ram[12];
                    tmp_csm = tte_csm_ram[12];
					// INNO - read wins.
					rd_tte_tag[58:0] <=	
					{tmp_tag[58:27], tlb_entry_vld[12],tlb_entry_locked[12], 
					tlb_entry_used[12], tmp_tag[23:0]}  ;
					rd_tte_data[42:0] <= {tmp_data[42:12],tmp_data[11:0]} ;
                    rd_tte_csm <= tmp_csm;
					end
			
			end
        if (((rw_index_vld & rw_wdline[13]) | (~rw_index_vld & tlb_entry_replace_d2[13])) & wr_vld_tmp & ~rw_disable)
		    begin
				if (~rst_tri_en)
					begin
                    if (wr_csm_sel)
                    begin
                        tte_csm_buf[wr_tte_csm_thrid] <= wr_tte_csm;
                    end
                    else
                    begin
                        tte_csm_ram[13] <= tte_csm_buf[wr_tte_csm_thrid];
                        tte_tag_ram[13] <= wr_tte_tag[58:0];	// CHANGE
                        tte_data_ram[13] <= wr_tte_data[42:0];
                        //tlb_entry_vld[j] <= wr_tte_tag[`STLB_TAG_V] ;
                        tlb_entry_used[13] <= wr_tte_tag[24] ;
                        tlb_entry_locked[13] = wr_tte_tag[25] ;
                    end
					// write-thru 
					rd_tte_tag[58:0]  <= wr_tte_tag[58:0] ;	// CHANGE 
					rd_tte_data[42:0] <=  wr_tte_data[42:0];
                    rd_tte_csm <= wr_tte_csm;
					end
				else
					begin
					tmp_tag[58:0]=tte_tag_ram[13]; // use non-blocking
					tmp_data[42:0]=tte_data_ram[13];
                    tmp_csm = tte_csm_ram[13];
					// INNO - read wins.
					rd_tte_tag[58:0] <=	
					{tmp_tag[58:27], tlb_entry_vld[13],tlb_entry_locked[13], 
					tlb_entry_used[13], tmp_tag[23:0]}  ;
					rd_tte_data[42:0] <= {tmp_data[42:12],tmp_data[11:0]} ;
                    rd_tte_csm <= tmp_csm;
					end
			
			end
        if (((rw_index_vld & rw_wdline[14]) | (~rw_index_vld & tlb_entry_replace_d2[14])) & wr_vld_tmp & ~rw_disable)
		    begin
				if (~rst_tri_en)
					begin
                    if (wr_csm_sel)
                    begin
                        tte_csm_buf[wr_tte_csm_thrid] <= wr_tte_csm;
                    end
                    else
                    begin
                        tte_csm_ram[14] <= tte_csm_buf[wr_tte_csm_thrid];
                        tte_tag_ram[14] <= wr_tte_tag[58:0];	// CHANGE
                        tte_data_ram[14] <= wr_tte_data[42:0];
                        //tlb_entry_vld[j] <= wr_tte_tag[`STLB_TAG_V] ;
                        tlb_entry_used[14] <= wr_tte_tag[24] ;
                        tlb_entry_locked[14] = wr_tte_tag[25] ;
                    end
					// write-thru 
					rd_tte_tag[58:0]  <= wr_tte_tag[58:0] ;	// CHANGE 
					rd_tte_data[42:0] <=  wr_tte_data[42:0];
                    rd_tte_csm <= wr_tte_csm;
					end
				else
					begin
					tmp_tag[58:0]=tte_tag_ram[14]; // use non-blocking
					tmp_data[42:0]=tte_data_ram[14];
                    tmp_csm = tte_csm_ram[14];
					// INNO - read wins.
					rd_tte_tag[58:0] <=	
					{tmp_tag[58:27], tlb_entry_vld[14],tlb_entry_locked[14], 
					tlb_entry_used[14], tmp_tag[23:0]}  ;
					rd_tte_data[42:0] <= {tmp_data[42:12],tmp_data[11:0]} ;
                    rd_tte_csm <= tmp_csm;
					end
			
			end
        if (((rw_index_vld & rw_wdline[15]) | (~rw_index_vld & tlb_entry_replace_d2[15])) & wr_vld_tmp & ~rw_disable)
		    begin
				if (~rst_tri_en)
					begin
                    if (wr_csm_sel)
                    begin
                        tte_csm_buf[wr_tte_csm_thrid] <= wr_tte_csm;
                    end
                    else
                    begin
                        tte_csm_ram[15] <= tte_csm_buf[wr_tte_csm_thrid];
                        tte_tag_ram[15] <= wr_tte_tag[58:0];	// CHANGE
                        tte_data_ram[15] <= wr_tte_data[42:0];
                        //tlb_entry_vld[j] <= wr_tte_tag[`STLB_TAG_V] ;
                        tlb_entry_used[15] <= wr_tte_tag[24] ;
                        tlb_entry_locked[15] = wr_tte_tag[25] ;
                    end
					// write-thru 
					rd_tte_tag[58:0]  <= wr_tte_tag[58:0] ;	// CHANGE 
					rd_tte_data[42:0] <=  wr_tte_data[42:0];
                    rd_tte_csm <= wr_tte_csm;
					end
				else
					begin
					tmp_tag[58:0]=tte_tag_ram[15]; // use non-blocking
					tmp_data[42:0]=tte_data_ram[15];
                    tmp_csm = tte_csm_ram[15];
					// INNO - read wins.
					rd_tte_tag[58:0] <=	
					{tmp_tag[58:27], tlb_entry_vld[15],tlb_entry_locked[15], 
					tlb_entry_used[15], tmp_tag[23:0]}  ;
					rd_tte_data[42:0] <= {tmp_data[42:12],tmp_data[11:0]} ;
                    rd_tte_csm <= tmp_csm;
					end
			
			end

//=========================================================================================
//	Read STLB
//=========================================================================================

        if (rw_wdline[0] & (rd_tag | rd_data | rd_csm) & ~rw_disable)
				begin
					tmp_tag  = tte_tag_ram[0] ;
					tmp_data = tte_data_ram[0] ;
                    tmp_csm = tte_csm_ram[0];
					if (rd_tag)
						rd_tte_tag[58:0] <=	// CHANGE - Bug 2185
						{tmp_tag[58:27], tlb_entry_vld[0],tlb_entry_locked[0], 
						tlb_entry_used[0], tmp_tag[23:0]}  ;
					if (rd_data) begin
						rd_tte_data[42:0] <= {tmp_data[42:12],tmp_data[11:0]} ;
					end
					if (rd_csm) begin
						rd_tte_csm <= tmp_csm ;
					end

				end
        if (rw_wdline[1] & (rd_tag | rd_data | rd_csm) & ~rw_disable)
				begin
					tmp_tag  = tte_tag_ram[1] ;
					tmp_data = tte_data_ram[1] ;
                    tmp_csm = tte_csm_ram[1];
					if (rd_tag)
						rd_tte_tag[58:0] <=	// CHANGE - Bug 2185
						{tmp_tag[58:27], tlb_entry_vld[1],tlb_entry_locked[1], 
						tlb_entry_used[1], tmp_tag[23:0]}  ;
					if (rd_data) begin
						rd_tte_data[42:0] <= {tmp_data[42:12],tmp_data[11:0]} ;
					end
					if (rd_csm) begin
						rd_tte_csm <= tmp_csm ;
					end

				end
        if (rw_wdline[2] & (rd_tag | rd_data | rd_csm) & ~rw_disable)
				begin
					tmp_tag  = tte_tag_ram[2] ;
					tmp_data = tte_data_ram[2] ;
                    tmp_csm = tte_csm_ram[2];
					if (rd_tag)
						rd_tte_tag[58:0] <=	// CHANGE - Bug 2185
						{tmp_tag[58:27], tlb_entry_vld[2],tlb_entry_locked[2], 
						tlb_entry_used[2], tmp_tag[23:0]}  ;
					if (rd_data) begin
						rd_tte_data[42:0] <= {tmp_data[42:12],tmp_data[11:0]} ;
					end
					if (rd_csm) begin
						rd_tte_csm <= tmp_csm ;
					end

				end
        if (rw_wdline[3] & (rd_tag | rd_data | rd_csm) & ~rw_disable)
				begin
					tmp_tag  = tte_tag_ram[3] ;
					tmp_data = tte_data_ram[3] ;
                    tmp_csm = tte_csm_ram[3];
					if (rd_tag)
						rd_tte_tag[58:0] <=	// CHANGE - Bug 2185
						{tmp_tag[58:27], tlb_entry_vld[3],tlb_entry_locked[3], 
						tlb_entry_used[3], tmp_tag[23:0]}  ;
					if (rd_data) begin
						rd_tte_data[42:0] <= {tmp_data[42:12],tmp_data[11:0]} ;
					end
					if (rd_csm) begin
						rd_tte_csm <= tmp_csm ;
					end

				end
        if (rw_wdline[4] & (rd_tag | rd_data | rd_csm) & ~rw_disable)
				begin
					tmp_tag  = tte_tag_ram[4] ;
					tmp_data = tte_data_ram[4] ;
                    tmp_csm = tte_csm_ram[4];
					if (rd_tag)
						rd_tte_tag[58:0] <=	// CHANGE - Bug 2185
						{tmp_tag[58:27], tlb_entry_vld[4],tlb_entry_locked[4], 
						tlb_entry_used[4], tmp_tag[23:0]}  ;
					if (rd_data) begin
						rd_tte_data[42:0] <= {tmp_data[42:12],tmp_data[11:0]} ;
					end
					if (rd_csm) begin
						rd_tte_csm <= tmp_csm ;
					end

				end
        if (rw_wdline[5] & (rd_tag | rd_data | rd_csm) & ~rw_disable)
				begin
					tmp_tag  = tte_tag_ram[5] ;
					tmp_data = tte_data_ram[5] ;
                    tmp_csm = tte_csm_ram[5];
					if (rd_tag)
						rd_tte_tag[58:0] <=	// CHANGE - Bug 2185
						{tmp_tag[58:27], tlb_entry_vld[5],tlb_entry_locked[5], 
						tlb_entry_used[5], tmp_tag[23:0]}  ;
					if (rd_data) begin
						rd_tte_data[42:0] <= {tmp_data[42:12],tmp_data[11:0]} ;
					end
					if (rd_csm) begin
						rd_tte_csm <= tmp_csm ;
					end

				end
        if (rw_wdline[6] & (rd_tag | rd_data | rd_csm) & ~rw_disable)
				begin
					tmp_tag  = tte_tag_ram[6] ;
					tmp_data = tte_data_ram[6] ;
                    tmp_csm = tte_csm_ram[6];
					if (rd_tag)
						rd_tte_tag[58:0] <=	// CHANGE - Bug 2185
						{tmp_tag[58:27], tlb_entry_vld[6],tlb_entry_locked[6], 
						tlb_entry_used[6], tmp_tag[23:0]}  ;
					if (rd_data) begin
						rd_tte_data[42:0] <= {tmp_data[42:12],tmp_data[11:0]} ;
					end
					if (rd_csm) begin
						rd_tte_csm <= tmp_csm ;
					end

				end
        if (rw_wdline[7] & (rd_tag | rd_data | rd_csm) & ~rw_disable)
				begin
					tmp_tag  = tte_tag_ram[7] ;
					tmp_data = tte_data_ram[7] ;
                    tmp_csm = tte_csm_ram[7];
					if (rd_tag)
						rd_tte_tag[58:0] <=	// CHANGE - Bug 2185
						{tmp_tag[58:27], tlb_entry_vld[7],tlb_entry_locked[7], 
						tlb_entry_used[7], tmp_tag[23:0]}  ;
					if (rd_data) begin
						rd_tte_data[42:0] <= {tmp_data[42:12],tmp_data[11:0]} ;
					end
					if (rd_csm) begin
						rd_tte_csm <= tmp_csm ;
					end

				end
        if (rw_wdline[8] & (rd_tag | rd_data | rd_csm) & ~rw_disable)
				begin
					tmp_tag  = tte_tag_ram[8] ;
					tmp_data = tte_data_ram[8] ;
                    tmp_csm = tte_csm_ram[8];
					if (rd_tag)
						rd_tte_tag[58:0] <=	// CHANGE - Bug 2185
						{tmp_tag[58:27], tlb_entry_vld[8],tlb_entry_locked[8], 
						tlb_entry_used[8], tmp_tag[23:0]}  ;
					if (rd_data) begin
						rd_tte_data[42:0] <= {tmp_data[42:12],tmp_data[11:0]} ;
					end
					if (rd_csm) begin
						rd_tte_csm <= tmp_csm ;
					end

				end
        if (rw_wdline[9] & (rd_tag | rd_data | rd_csm) & ~rw_disable)
				begin
					tmp_tag  = tte_tag_ram[9] ;
					tmp_data = tte_data_ram[9] ;
                    tmp_csm = tte_csm_ram[9];
					if (rd_tag)
						rd_tte_tag[58:0] <=	// CHANGE - Bug 2185
						{tmp_tag[58:27], tlb_entry_vld[9],tlb_entry_locked[9], 
						tlb_entry_used[9], tmp_tag[23:0]}  ;
					if (rd_data) begin
						rd_tte_data[42:0] <= {tmp_data[42:12],tmp_data[11:0]} ;
					end
					if (rd_csm) begin
						rd_tte_csm <= tmp_csm ;
					end

				end
        if (rw_wdline[10] & (rd_tag | rd_data | rd_csm) & ~rw_disable)
				begin
					tmp_tag  = tte_tag_ram[10] ;
					tmp_data = tte_data_ram[10] ;
                    tmp_csm = tte_csm_ram[10];
					if (rd_tag)
						rd_tte_tag[58:0] <=	// CHANGE - Bug 2185
						{tmp_tag[58:27], tlb_entry_vld[10],tlb_entry_locked[10], 
						tlb_entry_used[10], tmp_tag[23:0]}  ;
					if (rd_data) begin
						rd_tte_data[42:0] <= {tmp_data[42:12],tmp_data[11:0]} ;
					end
					if (rd_csm) begin
						rd_tte_csm <= tmp_csm ;
					end

				end
        if (rw_wdline[11] & (rd_tag | rd_data | rd_csm) & ~rw_disable)
				begin
					tmp_tag  = tte_tag_ram[11] ;
					tmp_data = tte_data_ram[11] ;
                    tmp_csm = tte_csm_ram[11];
					if (rd_tag)
						rd_tte_tag[58:0] <=	// CHANGE - Bug 2185
						{tmp_tag[58:27], tlb_entry_vld[11],tlb_entry_locked[11], 
						tlb_entry_used[11], tmp_tag[23:0]}  ;
					if (rd_data) begin
						rd_tte_data[42:0] <= {tmp_data[42:12],tmp_data[11:0]} ;
					end
					if (rd_csm) begin
						rd_tte_csm <= tmp_csm ;
					end

				end
        if (rw_wdline[12] & (rd_tag | rd_data | rd_csm) & ~rw_disable)
				begin
					tmp_tag  = tte_tag_ram[12] ;
					tmp_data = tte_data_ram[12] ;
                    tmp_csm = tte_csm_ram[12];
					if (rd_tag)
						rd_tte_tag[58:0] <=	// CHANGE - Bug 2185
						{tmp_tag[58:27], tlb_entry_vld[12],tlb_entry_locked[12], 
						tlb_entry_used[12], tmp_tag[23:0]}  ;
					if (rd_data) begin
						rd_tte_data[42:0] <= {tmp_data[42:12],tmp_data[11:0]} ;
					end
					if (rd_csm) begin
						rd_tte_csm <= tmp_csm ;
					end

				end
        if (rw_wdline[13] & (rd_tag | rd_data | rd_csm) & ~rw_disable)
				begin
					tmp_tag  = tte_tag_ram[13] ;
					tmp_data = tte_data_ram[13] ;
                    tmp_csm = tte_csm_ram[13];
					if (rd_tag)
						rd_tte_tag[58:0] <=	// CHANGE - Bug 2185
						{tmp_tag[58:27], tlb_entry_vld[13],tlb_entry_locked[13], 
						tlb_entry_used[13], tmp_tag[23:0]}  ;
					if (rd_data) begin
						rd_tte_data[42:0] <= {tmp_data[42:12],tmp_data[11:0]} ;
					end
					if (rd_csm) begin
						rd_tte_csm <= tmp_csm ;
					end

				end
        if (rw_wdline[14] & (rd_tag | rd_data | rd_csm) & ~rw_disable)
				begin
					tmp_tag  = tte_tag_ram[14] ;
					tmp_data = tte_data_ram[14] ;
                    tmp_csm = tte_csm_ram[14];
					if (rd_tag)
						rd_tte_tag[58:0] <=	// CHANGE - Bug 2185
						{tmp_tag[58:27], tlb_entry_vld[14],tlb_entry_locked[14], 
						tlb_entry_used[14], tmp_tag[23:0]}  ;
					if (rd_data) begin
						rd_tte_data[42:0] <= {tmp_data[42:12],tmp_data[11:0]} ;
					end
					if (rd_csm) begin
						rd_tte_csm <= tmp_csm ;
					end

				end
        if (rw_wdline[15] & (rd_tag | rd_data | rd_csm) & ~rw_disable)
				begin
					tmp_tag  = tte_tag_ram[15] ;
					tmp_data = tte_data_ram[15] ;
                    tmp_csm = tte_csm_ram[15];
					if (rd_tag)
						rd_tte_tag[58:0] <=	// CHANGE - Bug 2185
						{tmp_tag[58:27], tlb_entry_vld[15],tlb_entry_locked[15], 
						tlb_entry_used[15], tmp_tag[23:0]}  ;
					if (rd_data) begin
						rd_tte_data[42:0] <= {tmp_data[42:12],tmp_data[11:0]} ;
					end
					if (rd_csm) begin
						rd_tte_csm <= tmp_csm ;
					end

				end

		if (cam_vld & ~rw_disable)
  		begin
    			//Checking for no hit and multiple hits
    			sat = {16{1'b0}};
             begin
      				if(cam_hit[0])
      				begin
        				sat = sat + {{(16-1){1'b0}}, 1'b1};
      				end
    			end
             begin
      				if(cam_hit[1])
      				begin
        				sat = sat + {{(16-1){1'b0}}, 1'b1};
      				end
    			end
             begin
      				if(cam_hit[2])
      				begin
        				sat = sat + {{(16-1){1'b0}}, 1'b1};
      				end
    			end
             begin
      				if(cam_hit[3])
      				begin
        				sat = sat + {{(16-1){1'b0}}, 1'b1};
      				end
    			end
             begin
      				if(cam_hit[4])
      				begin
        				sat = sat + {{(16-1){1'b0}}, 1'b1};
      				end
    			end
             begin
      				if(cam_hit[5])
      				begin
        				sat = sat + {{(16-1){1'b0}}, 1'b1};
      				end
    			end
             begin
      				if(cam_hit[6])
      				begin
        				sat = sat + {{(16-1){1'b0}}, 1'b1};
      				end
    			end
             begin
      				if(cam_hit[7])
      				begin
        				sat = sat + {{(16-1){1'b0}}, 1'b1};
      				end
    			end
             begin
      				if(cam_hit[8])
      				begin
        				sat = sat + {{(16-1){1'b0}}, 1'b1};
      				end
    			end
             begin
      				if(cam_hit[9])
      				begin
        				sat = sat + {{(16-1){1'b0}}, 1'b1};
      				end
    			end
             begin
      				if(cam_hit[10])
      				begin
        				sat = sat + {{(16-1){1'b0}}, 1'b1};
      				end
    			end
             begin
      				if(cam_hit[11])
      				begin
        				sat = sat + {{(16-1){1'b0}}, 1'b1};
      				end
    			end
             begin
      				if(cam_hit[12])
      				begin
        				sat = sat + {{(16-1){1'b0}}, 1'b1};
      				end
    			end
             begin
      				if(cam_hit[13])
      				begin
        				sat = sat + {{(16-1){1'b0}}, 1'b1};
      				end
    			end
             begin
      				if(cam_hit[14])
      				begin
        				sat = sat + {{(16-1){1'b0}}, 1'b1};
      				end
    			end
             begin
      				if(cam_hit[15])
      				begin
        				sat = sat + {{(16-1){1'b0}}, 1'b1};
      				end
    			end

			// Only one hit occur read the data
    			if(sat == {{(16-1){1'b0}}, 1'b1})
    			begin
                   begin
                    if (cam_hit[0])
                    begin
                            rd_tte_data[42:0] <= tte_data_ram[0] ;
                            rd_tte_csm <= tte_csm_ram[0] ;
                    end
				end
                   begin
                    if (cam_hit[1])
                    begin
                            rd_tte_data[42:0] <= tte_data_ram[1] ;
                            rd_tte_csm <= tte_csm_ram[1] ;
                    end
				end
                   begin
                    if (cam_hit[2])
                    begin
                            rd_tte_data[42:0] <= tte_data_ram[2] ;
                            rd_tte_csm <= tte_csm_ram[2] ;
                    end
				end
                   begin
                    if (cam_hit[3])
                    begin
                            rd_tte_data[42:0] <= tte_data_ram[3] ;
                            rd_tte_csm <= tte_csm_ram[3] ;
                    end
				end
                   begin
                    if (cam_hit[4])
                    begin
                            rd_tte_data[42:0] <= tte_data_ram[4] ;
                            rd_tte_csm <= tte_csm_ram[4] ;
                    end
				end
                   begin
                    if (cam_hit[5])
                    begin
                            rd_tte_data[42:0] <= tte_data_ram[5] ;
                            rd_tte_csm <= tte_csm_ram[5] ;
                    end
				end
                   begin
                    if (cam_hit[6])
                    begin
                            rd_tte_data[42:0] <= tte_data_ram[6] ;
                            rd_tte_csm <= tte_csm_ram[6] ;
                    end
				end
                   begin
                    if (cam_hit[7])
                    begin
                            rd_tte_data[42:0] <= tte_data_ram[7] ;
                            rd_tte_csm <= tte_csm_ram[7] ;
                    end
				end
                   begin
                    if (cam_hit[8])
                    begin
                            rd_tte_data[42:0] <= tte_data_ram[8] ;
                            rd_tte_csm <= tte_csm_ram[8] ;
                    end
				end
                   begin
                    if (cam_hit[9])
                    begin
                            rd_tte_data[42:0] <= tte_data_ram[9] ;
                            rd_tte_csm <= tte_csm_ram[9] ;
                    end
				end
                   begin
                    if (cam_hit[10])
                    begin
                            rd_tte_data[42:0] <= tte_data_ram[10] ;
                            rd_tte_csm <= tte_csm_ram[10] ;
                    end
				end
                   begin
                    if (cam_hit[11])
                    begin
                            rd_tte_data[42:0] <= tte_data_ram[11] ;
                            rd_tte_csm <= tte_csm_ram[11] ;
                    end
				end
                   begin
                    if (cam_hit[12])
                    begin
                            rd_tte_data[42:0] <= tte_data_ram[12] ;
                            rd_tte_csm <= tte_csm_ram[12] ;
                    end
				end
                   begin
                    if (cam_hit[13])
                    begin
                            rd_tte_data[42:0] <= tte_data_ram[13] ;
                            rd_tte_csm <= tte_csm_ram[13] ;
                    end
				end
                   begin
                    if (cam_hit[14])
                    begin
                            rd_tte_data[42:0] <= tte_data_ram[14] ;
                            rd_tte_csm <= tte_csm_ram[14] ;
                    end
				end
                   begin
                    if (cam_hit[15])
                    begin
                            rd_tte_data[42:0] <= tte_data_ram[15] ;
                            rd_tte_csm <= tte_csm_ram[15] ;
                    end
				end

			end
			else
			begin
				// INNO - just to keep the tool happy.
				// ram cell will not be corrupted.
               begin
					if (cam_hit[0])
                    begin
                        		tte_data_ram[0] <= 43'bx ;
                        		tte_csm_ram[0] <= {33{1'bx}} ;
                    end
				end
               begin
					if (cam_hit[1])
                    begin
                        		tte_data_ram[1] <= 43'bx ;
                        		tte_csm_ram[1] <= {33{1'bx}} ;
                    end
				end
               begin
					if (cam_hit[2])
                    begin
                        		tte_data_ram[2] <= 43'bx ;
                        		tte_csm_ram[2] <= {33{1'bx}} ;
                    end
				end
               begin
					if (cam_hit[3])
                    begin
                        		tte_data_ram[3] <= 43'bx ;
                        		tte_csm_ram[3] <= {33{1'bx}} ;
                    end
				end
               begin
					if (cam_hit[4])
                    begin
                        		tte_data_ram[4] <= 43'bx ;
                        		tte_csm_ram[4] <= {33{1'bx}} ;
                    end
				end
               begin
					if (cam_hit[5])
                    begin
                        		tte_data_ram[5] <= 43'bx ;
                        		tte_csm_ram[5] <= {33{1'bx}} ;
                    end
				end
               begin
					if (cam_hit[6])
                    begin
                        		tte_data_ram[6] <= 43'bx ;
                        		tte_csm_ram[6] <= {33{1'bx}} ;
                    end
				end
               begin
					if (cam_hit[7])
                    begin
                        		tte_data_ram[7] <= 43'bx ;
                        		tte_csm_ram[7] <= {33{1'bx}} ;
                    end
				end
               begin
					if (cam_hit[8])
                    begin
                        		tte_data_ram[8] <= 43'bx ;
                        		tte_csm_ram[8] <= {33{1'bx}} ;
                    end
				end
               begin
					if (cam_hit[9])
                    begin
                        		tte_data_ram[9] <= 43'bx ;
                        		tte_csm_ram[9] <= {33{1'bx}} ;
                    end
				end
               begin
					if (cam_hit[10])
                    begin
                        		tte_data_ram[10] <= 43'bx ;
                        		tte_csm_ram[10] <= {33{1'bx}} ;
                    end
				end
               begin
					if (cam_hit[11])
                    begin
                        		tte_data_ram[11] <= 43'bx ;
                        		tte_csm_ram[11] <= {33{1'bx}} ;
                    end
				end
               begin
					if (cam_hit[12])
                    begin
                        		tte_data_ram[12] <= 43'bx ;
                        		tte_csm_ram[12] <= {33{1'bx}} ;
                    end
				end
               begin
					if (cam_hit[13])
                    begin
                        		tte_data_ram[13] <= 43'bx ;
                        		tte_csm_ram[13] <= {33{1'bx}} ;
                    end
				end
               begin
					if (cam_hit[14])
                    begin
                        		tte_data_ram[14] <= 43'bx ;
                        		tte_csm_ram[14] <= {33{1'bx}} ;
                    end
				end
               begin
					if (cam_hit[15])
                    begin
                        		tte_data_ram[15] <= 43'bx ;
                        		tte_csm_ram[15] <= {33{1'bx}} ;
                    end
				end

                rd_tte_data[42:0] <= 43'bx ;
                rd_tte_csm <= {33{1'bx}};
			end
		end

                   begin
                                if (cam_hit[0])
                                        tlb_entry_used[0] <= 1'b1;
                        end
                   begin
                                if (cam_hit[1])
                                        tlb_entry_used[1] <= 1'b1;
                        end
                   begin
                                if (cam_hit[2])
                                        tlb_entry_used[2] <= 1'b1;
                        end
                   begin
                                if (cam_hit[3])
                                        tlb_entry_used[3] <= 1'b1;
                        end
                   begin
                                if (cam_hit[4])
                                        tlb_entry_used[4] <= 1'b1;
                        end
                   begin
                                if (cam_hit[5])
                                        tlb_entry_used[5] <= 1'b1;
                        end
                   begin
                                if (cam_hit[6])
                                        tlb_entry_used[6] <= 1'b1;
                        end
                   begin
                                if (cam_hit[7])
                                        tlb_entry_used[7] <= 1'b1;
                        end
                   begin
                                if (cam_hit[8])
                                        tlb_entry_used[8] <= 1'b1;
                        end
                   begin
                                if (cam_hit[9])
                                        tlb_entry_used[9] <= 1'b1;
                        end
                   begin
                                if (cam_hit[10])
                                        tlb_entry_used[10] <= 1'b1;
                        end
                   begin
                                if (cam_hit[11])
                                        tlb_entry_used[11] <= 1'b1;
                        end
                   begin
                                if (cam_hit[12])
                                        tlb_entry_used[12] <= 1'b1;
                        end
                   begin
                                if (cam_hit[13])
                                        tlb_entry_used[13] <= 1'b1;
                        end
                   begin
                                if (cam_hit[14])
                                        tlb_entry_used[14] <= 1'b1;
                        end
                   begin
                                if (cam_hit[15])
                                        tlb_entry_used[15] <= 1'b1;
                        end

// Clear on following edge if necessary.
// CHANGE(SATYA) : tlb_entry_used qualified with valid needs to be used to determine
// whether the Used bits are to be cleared. This allows invalid entries created
// by a demap to be used for replacement. Else we will ignore these entries
// for replacement

                //if (tlb_not_writeable)
                if (~tlb_writeable & ~cam_vld & ~wr_vld & ~rd_tag & ~rst_tri_en)
                        begin
                           begin
                                                //if (~tlb_entry_locked[t])
                                                if (~tlb_entry_locked[0] & ~cam_vld & ~wr_vld)
                                                        tlb_entry_used[0] <= 1'b0;
                                        end
                           begin
                                                //if (~tlb_entry_locked[t])
                                                if (~tlb_entry_locked[1] & ~cam_vld & ~wr_vld)
                                                        tlb_entry_used[1] <= 1'b0;
                                        end
                           begin
                                                //if (~tlb_entry_locked[t])
                                                if (~tlb_entry_locked[2] & ~cam_vld & ~wr_vld)
                                                        tlb_entry_used[2] <= 1'b0;
                                        end
                           begin
                                                //if (~tlb_entry_locked[t])
                                                if (~tlb_entry_locked[3] & ~cam_vld & ~wr_vld)
                                                        tlb_entry_used[3] <= 1'b0;
                                        end
                           begin
                                                //if (~tlb_entry_locked[t])
                                                if (~tlb_entry_locked[4] & ~cam_vld & ~wr_vld)
                                                        tlb_entry_used[4] <= 1'b0;
                                        end
                           begin
                                                //if (~tlb_entry_locked[t])
                                                if (~tlb_entry_locked[5] & ~cam_vld & ~wr_vld)
                                                        tlb_entry_used[5] <= 1'b0;
                                        end
                           begin
                                                //if (~tlb_entry_locked[t])
                                                if (~tlb_entry_locked[6] & ~cam_vld & ~wr_vld)
                                                        tlb_entry_used[6] <= 1'b0;
                                        end
                           begin
                                                //if (~tlb_entry_locked[t])
                                                if (~tlb_entry_locked[7] & ~cam_vld & ~wr_vld)
                                                        tlb_entry_used[7] <= 1'b0;
                                        end
                           begin
                                                //if (~tlb_entry_locked[t])
                                                if (~tlb_entry_locked[8] & ~cam_vld & ~wr_vld)
                                                        tlb_entry_used[8] <= 1'b0;
                                        end
                           begin
                                                //if (~tlb_entry_locked[t])
                                                if (~tlb_entry_locked[9] & ~cam_vld & ~wr_vld)
                                                        tlb_entry_used[9] <= 1'b0;
                                        end
                           begin
                                                //if (~tlb_entry_locked[t])
                                                if (~tlb_entry_locked[10] & ~cam_vld & ~wr_vld)
                                                        tlb_entry_used[10] <= 1'b0;
                                        end
                           begin
                                                //if (~tlb_entry_locked[t])
                                                if (~tlb_entry_locked[11] & ~cam_vld & ~wr_vld)
                                                        tlb_entry_used[11] <= 1'b0;
                                        end
                           begin
                                                //if (~tlb_entry_locked[t])
                                                if (~tlb_entry_locked[12] & ~cam_vld & ~wr_vld)
                                                        tlb_entry_used[12] <= 1'b0;
                                        end
                           begin
                                                //if (~tlb_entry_locked[t])
                                                if (~tlb_entry_locked[13] & ~cam_vld & ~wr_vld)
                                                        tlb_entry_used[13] <= 1'b0;
                                        end
                           begin
                                                //if (~tlb_entry_locked[t])
                                                if (~tlb_entry_locked[14] & ~cam_vld & ~wr_vld)
                                                        tlb_entry_used[14] <= 1'b0;
                                        end
                           begin
                                                //if (~tlb_entry_locked[t])
                                                if (~tlb_entry_locked[15] & ~cam_vld & ~wr_vld)
                                                        tlb_entry_used[15] <= 1'b0;
                                        end

                        end
	end


always	@ *
begin
    case (cfg_csm_tlb_state)
    2'd0:
    begin
        if (~bypass)
        begin
            rd_tte_csm_real = {1'b0, rd_tte_csm[31:0]};
        end
        else
        begin
            rd_tte_csm_real = {1'b0, cfg_csm_tlb_hdid, cfg_csm_tlb_hd_size,
                               cfg_csm_tlb_sdid, cfg_csm_tlb_lsid};
        end
    end
    2'd1:
    begin
        if (~bypass)
        begin
            rd_tte_csm_real = {1'b0, rd_tte_csm[31:0]};
        end
        else
        begin
            rd_tte_csm_real = {1'b1, 2'b0, cfg_csm_tlb_chipid, cfg_csm_tlb_x,
                               cfg_csm_tlb_y};
        end
    end
    2'd2:
    begin
        rd_tte_csm_real = {1'b0, cfg_csm_tlb_hdid, cfg_csm_tlb_hd_size,
                           cfg_csm_tlb_sdid, cfg_csm_tlb_lsid};
    end
    2'd3:
    begin
        rd_tte_csm_real = {1'b1, 2'b0, cfg_csm_tlb_chipid, cfg_csm_tlb_x,
                           cfg_csm_tlb_y};
    end
    default:
    begin
        rd_tte_csm_real = {1'b0, rd_tte_csm[31:0]};
    end
    endcase
end

// Stage to next cycle.
always	@ (posedge clk)
	begin
		tlb_rd_tte_tag[58:0] 	<= rd_tte_tag[58:0] ;	// CHANGE
		tlb_rd_tte_data[42:0] 	<= rd_tte_data[42:0] ;
		tlb_rd_tte_csm 	<= rd_tte_csm_real ;
	end
assign tlb_rd_tte_csm_crit = rd_tte_csm_real;





































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































//=========================================================================================
//	CAM/DEMAP STLB for xlation
//=========================================================================================

//  no_hit logic does not work because it is set in multiple clock
//  domains and is reset before ever having a chance to be effective
//reg	no_hit ;


// Demap and CAM operation are mutually exclusive.

always  @ ( negedge clk )
	begin
	
           begin
                        /*if (demap_all)  begin
                                if (demap_auto & demap_other) ademap_hit[n]   =
                                        (~mismatch[n] & demap_all_but_locked_hit[n] & demap_other
                                                & tlb_entry_vld[n]) ;
                                end
                        else    begin */
                                if (demap_auto & demap_other) ademap_hit[0]    =
                                        (~mismatch[0] & demap_other & tlb_entry_vld[0]) ;
                                //end
			end
           begin
                        /*if (demap_all)  begin
                                if (demap_auto & demap_other) ademap_hit[n]   =
                                        (~mismatch[n] & demap_all_but_locked_hit[n] & demap_other
                                                & tlb_entry_vld[n]) ;
                                end
                        else    begin */
                                if (demap_auto & demap_other) ademap_hit[1]    =
                                        (~mismatch[1] & demap_other & tlb_entry_vld[1]) ;
                                //end
			end
           begin
                        /*if (demap_all)  begin
                                if (demap_auto & demap_other) ademap_hit[n]   =
                                        (~mismatch[n] & demap_all_but_locked_hit[n] & demap_other
                                                & tlb_entry_vld[n]) ;
                                end
                        else    begin */
                                if (demap_auto & demap_other) ademap_hit[2]    =
                                        (~mismatch[2] & demap_other & tlb_entry_vld[2]) ;
                                //end
			end
           begin
                        /*if (demap_all)  begin
                                if (demap_auto & demap_other) ademap_hit[n]   =
                                        (~mismatch[n] & demap_all_but_locked_hit[n] & demap_other
                                                & tlb_entry_vld[n]) ;
                                end
                        else    begin */
                                if (demap_auto & demap_other) ademap_hit[3]    =
                                        (~mismatch[3] & demap_other & tlb_entry_vld[3]) ;
                                //end
			end
           begin
                        /*if (demap_all)  begin
                                if (demap_auto & demap_other) ademap_hit[n]   =
                                        (~mismatch[n] & demap_all_but_locked_hit[n] & demap_other
                                                & tlb_entry_vld[n]) ;
                                end
                        else    begin */
                                if (demap_auto & demap_other) ademap_hit[4]    =
                                        (~mismatch[4] & demap_other & tlb_entry_vld[4]) ;
                                //end
			end
           begin
                        /*if (demap_all)  begin
                                if (demap_auto & demap_other) ademap_hit[n]   =
                                        (~mismatch[n] & demap_all_but_locked_hit[n] & demap_other
                                                & tlb_entry_vld[n]) ;
                                end
                        else    begin */
                                if (demap_auto & demap_other) ademap_hit[5]    =
                                        (~mismatch[5] & demap_other & tlb_entry_vld[5]) ;
                                //end
			end
           begin
                        /*if (demap_all)  begin
                                if (demap_auto & demap_other) ademap_hit[n]   =
                                        (~mismatch[n] & demap_all_but_locked_hit[n] & demap_other
                                                & tlb_entry_vld[n]) ;
                                end
                        else    begin */
                                if (demap_auto & demap_other) ademap_hit[6]    =
                                        (~mismatch[6] & demap_other & tlb_entry_vld[6]) ;
                                //end
			end
           begin
                        /*if (demap_all)  begin
                                if (demap_auto & demap_other) ademap_hit[n]   =
                                        (~mismatch[n] & demap_all_but_locked_hit[n] & demap_other
                                                & tlb_entry_vld[n]) ;
                                end
                        else    begin */
                                if (demap_auto & demap_other) ademap_hit[7]    =
                                        (~mismatch[7] & demap_other & tlb_entry_vld[7]) ;
                                //end
			end
           begin
                        /*if (demap_all)  begin
                                if (demap_auto & demap_other) ademap_hit[n]   =
                                        (~mismatch[n] & demap_all_but_locked_hit[n] & demap_other
                                                & tlb_entry_vld[n]) ;
                                end
                        else    begin */
                                if (demap_auto & demap_other) ademap_hit[8]    =
                                        (~mismatch[8] & demap_other & tlb_entry_vld[8]) ;
                                //end
			end
           begin
                        /*if (demap_all)  begin
                                if (demap_auto & demap_other) ademap_hit[n]   =
                                        (~mismatch[n] & demap_all_but_locked_hit[n] & demap_other
                                                & tlb_entry_vld[n]) ;
                                end
                        else    begin */
                                if (demap_auto & demap_other) ademap_hit[9]    =
                                        (~mismatch[9] & demap_other & tlb_entry_vld[9]) ;
                                //end
			end
           begin
                        /*if (demap_all)  begin
                                if (demap_auto & demap_other) ademap_hit[n]   =
                                        (~mismatch[n] & demap_all_but_locked_hit[n] & demap_other
                                                & tlb_entry_vld[n]) ;
                                end
                        else    begin */
                                if (demap_auto & demap_other) ademap_hit[10]    =
                                        (~mismatch[10] & demap_other & tlb_entry_vld[10]) ;
                                //end
			end
           begin
                        /*if (demap_all)  begin
                                if (demap_auto & demap_other) ademap_hit[n]   =
                                        (~mismatch[n] & demap_all_but_locked_hit[n] & demap_other
                                                & tlb_entry_vld[n]) ;
                                end
                        else    begin */
                                if (demap_auto & demap_other) ademap_hit[11]    =
                                        (~mismatch[11] & demap_other & tlb_entry_vld[11]) ;
                                //end
			end
           begin
                        /*if (demap_all)  begin
                                if (demap_auto & demap_other) ademap_hit[n]   =
                                        (~mismatch[n] & demap_all_but_locked_hit[n] & demap_other
                                                & tlb_entry_vld[n]) ;
                                end
                        else    begin */
                                if (demap_auto & demap_other) ademap_hit[12]    =
                                        (~mismatch[12] & demap_other & tlb_entry_vld[12]) ;
                                //end
			end
           begin
                        /*if (demap_all)  begin
                                if (demap_auto & demap_other) ademap_hit[n]   =
                                        (~mismatch[n] & demap_all_but_locked_hit[n] & demap_other
                                                & tlb_entry_vld[n]) ;
                                end
                        else    begin */
                                if (demap_auto & demap_other) ademap_hit[13]    =
                                        (~mismatch[13] & demap_other & tlb_entry_vld[13]) ;
                                //end
			end
           begin
                        /*if (demap_all)  begin
                                if (demap_auto & demap_other) ademap_hit[n]   =
                                        (~mismatch[n] & demap_all_but_locked_hit[n] & demap_other
                                                & tlb_entry_vld[n]) ;
                                end
                        else    begin */
                                if (demap_auto & demap_other) ademap_hit[14]    =
                                        (~mismatch[14] & demap_other & tlb_entry_vld[14]) ;
                                //end
			end
           begin
                        /*if (demap_all)  begin
                                if (demap_auto & demap_other) ademap_hit[n]   =
                                        (~mismatch[n] & demap_all_but_locked_hit[n] & demap_other
                                                & tlb_entry_vld[n]) ;
                                end
                        else    begin */
                                if (demap_auto & demap_other) ademap_hit[15]    =
                                        (~mismatch[15] & demap_other & tlb_entry_vld[15]) ;
                                //end
			end

	end  // always

always	@ (/*AUTOSENSE*/ /*memory or*/ 
           cam_data or cam_pid or cam_vld or demap_all
           or demap_other or tlb_entry_vld)
	begin
	
		    	begin
			tag[58:0] = tte_tag_ram[0] ;	// CHANGE

			mismatch_va_b47_28[0] = 
			(tag[53:34] 
			!= cam_data[40+13:21+13]);

			mismatch_va_b27_22[0] = 
			(tag[33:28] 
			!= cam_data[19+13:14+13]);

			mismatch_va_b21_16[0] = 
			(tag[23:18]
			!= cam_data[12+13:7+13]) ;

			mismatch_va_b15_13[0] = 
			(tag[16:14]
			!= cam_data[5+13:3+13]) ;

			mismatch_ctxt[0] = 
			(tag[12:0] 
			!= cam_data[12:0]) ;
			
			mismatch_pid[0] = (tag[58:56] != cam_pid[2:0]) ;

			mismatch[0] =
			(mismatch_va_b47_28[0] & cam_data[20+13]) 				|
			(mismatch_va_b27_22[0] & tag[27] & cam_data[13+13]) 	|
			(mismatch_va_b21_16[0] & tag[17] & cam_data[6+13]) 	|
			(mismatch_va_b15_13[0] & tag[13] & cam_data[2+13]) 	|
			(mismatch_ctxt[0] & ~cam_data[1+13])	|
			// mismatch is request type not equal to entry type. types are real/virtual.
			((tag[55] ^ cam_data[0+13]) & ~demap_all)  	| 
			mismatch_pid[0] ;	// pid always included in mismatch calculations

			demap_all_but_locked_hit[0] = 
			~tag[25] & demap_all ;

			cam_hit[0] 	= 
				~mismatch[0] & cam_vld   & tlb_entry_vld[0] ;

                        if (demap_all)  begin
                                // Satya(10/3) - I've simplified the demap-all equation
                                // Pls confirm that this is okay. Otherwise we will nee
                                // qualifying bits for the pid and r fields.
                                demap_hit[0]    =
                                        (~mismatch[0] & demap_all_but_locked_hit[0] & demap_other
                                                & tlb_entry_vld[0]) ;
				// qualification with demap_auto to prevent ademap_hit from
				// being cleared. Satya-we could get rid of this.
                                end
                        else    begin
                                demap_hit[0]    =
                                        (~mismatch[0] & demap_other & tlb_entry_vld[0]) ;
				// qualification with demap_auto to prevent ademap_hit from
				// being cleared. Satya-this is the only one we need.
                                end
//			no_hit = cam_vld ;
			end
		    	begin
			tag[58:0] = tte_tag_ram[1] ;	// CHANGE

			mismatch_va_b47_28[1] = 
			(tag[53:34] 
			!= cam_data[40+13:21+13]);

			mismatch_va_b27_22[1] = 
			(tag[33:28] 
			!= cam_data[19+13:14+13]);

			mismatch_va_b21_16[1] = 
			(tag[23:18]
			!= cam_data[12+13:7+13]) ;

			mismatch_va_b15_13[1] = 
			(tag[16:14]
			!= cam_data[5+13:3+13]) ;

			mismatch_ctxt[1] = 
			(tag[12:0] 
			!= cam_data[12:0]) ;
			
			mismatch_pid[1] = (tag[58:56] != cam_pid[2:0]) ;

			mismatch[1] =
			(mismatch_va_b47_28[1] & cam_data[20+13]) 				|
			(mismatch_va_b27_22[1] & tag[27] & cam_data[13+13]) 	|
			(mismatch_va_b21_16[1] & tag[17] & cam_data[6+13]) 	|
			(mismatch_va_b15_13[1] & tag[13] & cam_data[2+13]) 	|
			(mismatch_ctxt[1] & ~cam_data[1+13])	|
			// mismatch is request type not equal to entry type. types are real/virtual.
			((tag[55] ^ cam_data[0+13]) & ~demap_all)  	| 
			mismatch_pid[1] ;	// pid always included in mismatch calculations

			demap_all_but_locked_hit[1] = 
			~tag[25] & demap_all ;

			cam_hit[1] 	= 
				~mismatch[1] & cam_vld   & tlb_entry_vld[1] ;

                        if (demap_all)  begin
                                // Satya(10/3) - I've simplified the demap-all equation
                                // Pls confirm that this is okay. Otherwise we will nee
                                // qualifying bits for the pid and r fields.
                                demap_hit[1]    =
                                        (~mismatch[1] & demap_all_but_locked_hit[1] & demap_other
                                                & tlb_entry_vld[1]) ;
				// qualification with demap_auto to prevent ademap_hit from
				// being cleared. Satya-we could get rid of this.
                                end
                        else    begin
                                demap_hit[1]    =
                                        (~mismatch[1] & demap_other & tlb_entry_vld[1]) ;
				// qualification with demap_auto to prevent ademap_hit from
				// being cleared. Satya-this is the only one we need.
                                end
//			no_hit = cam_vld ;
			end
		    	begin
			tag[58:0] = tte_tag_ram[2] ;	// CHANGE

			mismatch_va_b47_28[2] = 
			(tag[53:34] 
			!= cam_data[40+13:21+13]);

			mismatch_va_b27_22[2] = 
			(tag[33:28] 
			!= cam_data[19+13:14+13]);

			mismatch_va_b21_16[2] = 
			(tag[23:18]
			!= cam_data[12+13:7+13]) ;

			mismatch_va_b15_13[2] = 
			(tag[16:14]
			!= cam_data[5+13:3+13]) ;

			mismatch_ctxt[2] = 
			(tag[12:0] 
			!= cam_data[12:0]) ;
			
			mismatch_pid[2] = (tag[58:56] != cam_pid[2:0]) ;

			mismatch[2] =
			(mismatch_va_b47_28[2] & cam_data[20+13]) 				|
			(mismatch_va_b27_22[2] & tag[27] & cam_data[13+13]) 	|
			(mismatch_va_b21_16[2] & tag[17] & cam_data[6+13]) 	|
			(mismatch_va_b15_13[2] & tag[13] & cam_data[2+13]) 	|
			(mismatch_ctxt[2] & ~cam_data[1+13])	|
			// mismatch is request type not equal to entry type. types are real/virtual.
			((tag[55] ^ cam_data[0+13]) & ~demap_all)  	| 
			mismatch_pid[2] ;	// pid always included in mismatch calculations

			demap_all_but_locked_hit[2] = 
			~tag[25] & demap_all ;

			cam_hit[2] 	= 
				~mismatch[2] & cam_vld   & tlb_entry_vld[2] ;

                        if (demap_all)  begin
                                // Satya(10/3) - I've simplified the demap-all equation
                                // Pls confirm that this is okay. Otherwise we will nee
                                // qualifying bits for the pid and r fields.
                                demap_hit[2]    =
                                        (~mismatch[2] & demap_all_but_locked_hit[2] & demap_other
                                                & tlb_entry_vld[2]) ;
				// qualification with demap_auto to prevent ademap_hit from
				// being cleared. Satya-we could get rid of this.
                                end
                        else    begin
                                demap_hit[2]    =
                                        (~mismatch[2] & demap_other & tlb_entry_vld[2]) ;
				// qualification with demap_auto to prevent ademap_hit from
				// being cleared. Satya-this is the only one we need.
                                end
//			no_hit = cam_vld ;
			end
		    	begin
			tag[58:0] = tte_tag_ram[3] ;	// CHANGE

			mismatch_va_b47_28[3] = 
			(tag[53:34] 
			!= cam_data[40+13:21+13]);

			mismatch_va_b27_22[3] = 
			(tag[33:28] 
			!= cam_data[19+13:14+13]);

			mismatch_va_b21_16[3] = 
			(tag[23:18]
			!= cam_data[12+13:7+13]) ;

			mismatch_va_b15_13[3] = 
			(tag[16:14]
			!= cam_data[5+13:3+13]) ;

			mismatch_ctxt[3] = 
			(tag[12:0] 
			!= cam_data[12:0]) ;
			
			mismatch_pid[3] = (tag[58:56] != cam_pid[2:0]) ;

			mismatch[3] =
			(mismatch_va_b47_28[3] & cam_data[20+13]) 				|
			(mismatch_va_b27_22[3] & tag[27] & cam_data[13+13]) 	|
			(mismatch_va_b21_16[3] & tag[17] & cam_data[6+13]) 	|
			(mismatch_va_b15_13[3] & tag[13] & cam_data[2+13]) 	|
			(mismatch_ctxt[3] & ~cam_data[1+13])	|
			// mismatch is request type not equal to entry type. types are real/virtual.
			((tag[55] ^ cam_data[0+13]) & ~demap_all)  	| 
			mismatch_pid[3] ;	// pid always included in mismatch calculations

			demap_all_but_locked_hit[3] = 
			~tag[25] & demap_all ;

			cam_hit[3] 	= 
				~mismatch[3] & cam_vld   & tlb_entry_vld[3] ;

                        if (demap_all)  begin
                                // Satya(10/3) - I've simplified the demap-all equation
                                // Pls confirm that this is okay. Otherwise we will nee
                                // qualifying bits for the pid and r fields.
                                demap_hit[3]    =
                                        (~mismatch[3] & demap_all_but_locked_hit[3] & demap_other
                                                & tlb_entry_vld[3]) ;
				// qualification with demap_auto to prevent ademap_hit from
				// being cleared. Satya-we could get rid of this.
                                end
                        else    begin
                                demap_hit[3]    =
                                        (~mismatch[3] & demap_other & tlb_entry_vld[3]) ;
				// qualification with demap_auto to prevent ademap_hit from
				// being cleared. Satya-this is the only one we need.
                                end
//			no_hit = cam_vld ;
			end
		    	begin
			tag[58:0] = tte_tag_ram[4] ;	// CHANGE

			mismatch_va_b47_28[4] = 
			(tag[53:34] 
			!= cam_data[40+13:21+13]);

			mismatch_va_b27_22[4] = 
			(tag[33:28] 
			!= cam_data[19+13:14+13]);

			mismatch_va_b21_16[4] = 
			(tag[23:18]
			!= cam_data[12+13:7+13]) ;

			mismatch_va_b15_13[4] = 
			(tag[16:14]
			!= cam_data[5+13:3+13]) ;

			mismatch_ctxt[4] = 
			(tag[12:0] 
			!= cam_data[12:0]) ;
			
			mismatch_pid[4] = (tag[58:56] != cam_pid[2:0]) ;

			mismatch[4] =
			(mismatch_va_b47_28[4] & cam_data[20+13]) 				|
			(mismatch_va_b27_22[4] & tag[27] & cam_data[13+13]) 	|
			(mismatch_va_b21_16[4] & tag[17] & cam_data[6+13]) 	|
			(mismatch_va_b15_13[4] & tag[13] & cam_data[2+13]) 	|
			(mismatch_ctxt[4] & ~cam_data[1+13])	|
			// mismatch is request type not equal to entry type. types are real/virtual.
			((tag[55] ^ cam_data[0+13]) & ~demap_all)  	| 
			mismatch_pid[4] ;	// pid always included in mismatch calculations

			demap_all_but_locked_hit[4] = 
			~tag[25] & demap_all ;

			cam_hit[4] 	= 
				~mismatch[4] & cam_vld   & tlb_entry_vld[4] ;

                        if (demap_all)  begin
                                // Satya(10/3) - I've simplified the demap-all equation
                                // Pls confirm that this is okay. Otherwise we will nee
                                // qualifying bits for the pid and r fields.
                                demap_hit[4]    =
                                        (~mismatch[4] & demap_all_but_locked_hit[4] & demap_other
                                                & tlb_entry_vld[4]) ;
				// qualification with demap_auto to prevent ademap_hit from
				// being cleared. Satya-we could get rid of this.
                                end
                        else    begin
                                demap_hit[4]    =
                                        (~mismatch[4] & demap_other & tlb_entry_vld[4]) ;
				// qualification with demap_auto to prevent ademap_hit from
				// being cleared. Satya-this is the only one we need.
                                end
//			no_hit = cam_vld ;
			end
		    	begin
			tag[58:0] = tte_tag_ram[5] ;	// CHANGE

			mismatch_va_b47_28[5] = 
			(tag[53:34] 
			!= cam_data[40+13:21+13]);

			mismatch_va_b27_22[5] = 
			(tag[33:28] 
			!= cam_data[19+13:14+13]);

			mismatch_va_b21_16[5] = 
			(tag[23:18]
			!= cam_data[12+13:7+13]) ;

			mismatch_va_b15_13[5] = 
			(tag[16:14]
			!= cam_data[5+13:3+13]) ;

			mismatch_ctxt[5] = 
			(tag[12:0] 
			!= cam_data[12:0]) ;
			
			mismatch_pid[5] = (tag[58:56] != cam_pid[2:0]) ;

			mismatch[5] =
			(mismatch_va_b47_28[5] & cam_data[20+13]) 				|
			(mismatch_va_b27_22[5] & tag[27] & cam_data[13+13]) 	|
			(mismatch_va_b21_16[5] & tag[17] & cam_data[6+13]) 	|
			(mismatch_va_b15_13[5] & tag[13] & cam_data[2+13]) 	|
			(mismatch_ctxt[5] & ~cam_data[1+13])	|
			// mismatch is request type not equal to entry type. types are real/virtual.
			((tag[55] ^ cam_data[0+13]) & ~demap_all)  	| 
			mismatch_pid[5] ;	// pid always included in mismatch calculations

			demap_all_but_locked_hit[5] = 
			~tag[25] & demap_all ;

			cam_hit[5] 	= 
				~mismatch[5] & cam_vld   & tlb_entry_vld[5] ;

                        if (demap_all)  begin
                                // Satya(10/3) - I've simplified the demap-all equation
                                // Pls confirm that this is okay. Otherwise we will nee
                                // qualifying bits for the pid and r fields.
                                demap_hit[5]    =
                                        (~mismatch[5] & demap_all_but_locked_hit[5] & demap_other
                                                & tlb_entry_vld[5]) ;
				// qualification with demap_auto to prevent ademap_hit from
				// being cleared. Satya-we could get rid of this.
                                end
                        else    begin
                                demap_hit[5]    =
                                        (~mismatch[5] & demap_other & tlb_entry_vld[5]) ;
				// qualification with demap_auto to prevent ademap_hit from
				// being cleared. Satya-this is the only one we need.
                                end
//			no_hit = cam_vld ;
			end
		    	begin
			tag[58:0] = tte_tag_ram[6] ;	// CHANGE

			mismatch_va_b47_28[6] = 
			(tag[53:34] 
			!= cam_data[40+13:21+13]);

			mismatch_va_b27_22[6] = 
			(tag[33:28] 
			!= cam_data[19+13:14+13]);

			mismatch_va_b21_16[6] = 
			(tag[23:18]
			!= cam_data[12+13:7+13]) ;

			mismatch_va_b15_13[6] = 
			(tag[16:14]
			!= cam_data[5+13:3+13]) ;

			mismatch_ctxt[6] = 
			(tag[12:0] 
			!= cam_data[12:0]) ;
			
			mismatch_pid[6] = (tag[58:56] != cam_pid[2:0]) ;

			mismatch[6] =
			(mismatch_va_b47_28[6] & cam_data[20+13]) 				|
			(mismatch_va_b27_22[6] & tag[27] & cam_data[13+13]) 	|
			(mismatch_va_b21_16[6] & tag[17] & cam_data[6+13]) 	|
			(mismatch_va_b15_13[6] & tag[13] & cam_data[2+13]) 	|
			(mismatch_ctxt[6] & ~cam_data[1+13])	|
			// mismatch is request type not equal to entry type. types are real/virtual.
			((tag[55] ^ cam_data[0+13]) & ~demap_all)  	| 
			mismatch_pid[6] ;	// pid always included in mismatch calculations

			demap_all_but_locked_hit[6] = 
			~tag[25] & demap_all ;

			cam_hit[6] 	= 
				~mismatch[6] & cam_vld   & tlb_entry_vld[6] ;

                        if (demap_all)  begin
                                // Satya(10/3) - I've simplified the demap-all equation
                                // Pls confirm that this is okay. Otherwise we will nee
                                // qualifying bits for the pid and r fields.
                                demap_hit[6]    =
                                        (~mismatch[6] & demap_all_but_locked_hit[6] & demap_other
                                                & tlb_entry_vld[6]) ;
				// qualification with demap_auto to prevent ademap_hit from
				// being cleared. Satya-we could get rid of this.
                                end
                        else    begin
                                demap_hit[6]    =
                                        (~mismatch[6] & demap_other & tlb_entry_vld[6]) ;
				// qualification with demap_auto to prevent ademap_hit from
				// being cleared. Satya-this is the only one we need.
                                end
//			no_hit = cam_vld ;
			end
		    	begin
			tag[58:0] = tte_tag_ram[7] ;	// CHANGE

			mismatch_va_b47_28[7] = 
			(tag[53:34] 
			!= cam_data[40+13:21+13]);

			mismatch_va_b27_22[7] = 
			(tag[33:28] 
			!= cam_data[19+13:14+13]);

			mismatch_va_b21_16[7] = 
			(tag[23:18]
			!= cam_data[12+13:7+13]) ;

			mismatch_va_b15_13[7] = 
			(tag[16:14]
			!= cam_data[5+13:3+13]) ;

			mismatch_ctxt[7] = 
			(tag[12:0] 
			!= cam_data[12:0]) ;
			
			mismatch_pid[7] = (tag[58:56] != cam_pid[2:0]) ;

			mismatch[7] =
			(mismatch_va_b47_28[7] & cam_data[20+13]) 				|
			(mismatch_va_b27_22[7] & tag[27] & cam_data[13+13]) 	|
			(mismatch_va_b21_16[7] & tag[17] & cam_data[6+13]) 	|
			(mismatch_va_b15_13[7] & tag[13] & cam_data[2+13]) 	|
			(mismatch_ctxt[7] & ~cam_data[1+13])	|
			// mismatch is request type not equal to entry type. types are real/virtual.
			((tag[55] ^ cam_data[0+13]) & ~demap_all)  	| 
			mismatch_pid[7] ;	// pid always included in mismatch calculations

			demap_all_but_locked_hit[7] = 
			~tag[25] & demap_all ;

			cam_hit[7] 	= 
				~mismatch[7] & cam_vld   & tlb_entry_vld[7] ;

                        if (demap_all)  begin
                                // Satya(10/3) - I've simplified the demap-all equation
                                // Pls confirm that this is okay. Otherwise we will nee
                                // qualifying bits for the pid and r fields.
                                demap_hit[7]    =
                                        (~mismatch[7] & demap_all_but_locked_hit[7] & demap_other
                                                & tlb_entry_vld[7]) ;
				// qualification with demap_auto to prevent ademap_hit from
				// being cleared. Satya-we could get rid of this.
                                end
                        else    begin
                                demap_hit[7]    =
                                        (~mismatch[7] & demap_other & tlb_entry_vld[7]) ;
				// qualification with demap_auto to prevent ademap_hit from
				// being cleared. Satya-this is the only one we need.
                                end
//			no_hit = cam_vld ;
			end
		    	begin
			tag[58:0] = tte_tag_ram[8] ;	// CHANGE

			mismatch_va_b47_28[8] = 
			(tag[53:34] 
			!= cam_data[40+13:21+13]);

			mismatch_va_b27_22[8] = 
			(tag[33:28] 
			!= cam_data[19+13:14+13]);

			mismatch_va_b21_16[8] = 
			(tag[23:18]
			!= cam_data[12+13:7+13]) ;

			mismatch_va_b15_13[8] = 
			(tag[16:14]
			!= cam_data[5+13:3+13]) ;

			mismatch_ctxt[8] = 
			(tag[12:0] 
			!= cam_data[12:0]) ;
			
			mismatch_pid[8] = (tag[58:56] != cam_pid[2:0]) ;

			mismatch[8] =
			(mismatch_va_b47_28[8] & cam_data[20+13]) 				|
			(mismatch_va_b27_22[8] & tag[27] & cam_data[13+13]) 	|
			(mismatch_va_b21_16[8] & tag[17] & cam_data[6+13]) 	|
			(mismatch_va_b15_13[8] & tag[13] & cam_data[2+13]) 	|
			(mismatch_ctxt[8] & ~cam_data[1+13])	|
			// mismatch is request type not equal to entry type. types are real/virtual.
			((tag[55] ^ cam_data[0+13]) & ~demap_all)  	| 
			mismatch_pid[8] ;	// pid always included in mismatch calculations

			demap_all_but_locked_hit[8] = 
			~tag[25] & demap_all ;

			cam_hit[8] 	= 
				~mismatch[8] & cam_vld   & tlb_entry_vld[8] ;

                        if (demap_all)  begin
                                // Satya(10/3) - I've simplified the demap-all equation
                                // Pls confirm that this is okay. Otherwise we will nee
                                // qualifying bits for the pid and r fields.
                                demap_hit[8]    =
                                        (~mismatch[8] & demap_all_but_locked_hit[8] & demap_other
                                                & tlb_entry_vld[8]) ;
				// qualification with demap_auto to prevent ademap_hit from
				// being cleared. Satya-we could get rid of this.
                                end
                        else    begin
                                demap_hit[8]    =
                                        (~mismatch[8] & demap_other & tlb_entry_vld[8]) ;
				// qualification with demap_auto to prevent ademap_hit from
				// being cleared. Satya-this is the only one we need.
                                end
//			no_hit = cam_vld ;
			end
		    	begin
			tag[58:0] = tte_tag_ram[9] ;	// CHANGE

			mismatch_va_b47_28[9] = 
			(tag[53:34] 
			!= cam_data[40+13:21+13]);

			mismatch_va_b27_22[9] = 
			(tag[33:28] 
			!= cam_data[19+13:14+13]);

			mismatch_va_b21_16[9] = 
			(tag[23:18]
			!= cam_data[12+13:7+13]) ;

			mismatch_va_b15_13[9] = 
			(tag[16:14]
			!= cam_data[5+13:3+13]) ;

			mismatch_ctxt[9] = 
			(tag[12:0] 
			!= cam_data[12:0]) ;
			
			mismatch_pid[9] = (tag[58:56] != cam_pid[2:0]) ;

			mismatch[9] =
			(mismatch_va_b47_28[9] & cam_data[20+13]) 				|
			(mismatch_va_b27_22[9] & tag[27] & cam_data[13+13]) 	|
			(mismatch_va_b21_16[9] & tag[17] & cam_data[6+13]) 	|
			(mismatch_va_b15_13[9] & tag[13] & cam_data[2+13]) 	|
			(mismatch_ctxt[9] & ~cam_data[1+13])	|
			// mismatch is request type not equal to entry type. types are real/virtual.
			((tag[55] ^ cam_data[0+13]) & ~demap_all)  	| 
			mismatch_pid[9] ;	// pid always included in mismatch calculations

			demap_all_but_locked_hit[9] = 
			~tag[25] & demap_all ;

			cam_hit[9] 	= 
				~mismatch[9] & cam_vld   & tlb_entry_vld[9] ;

                        if (demap_all)  begin
                                // Satya(10/3) - I've simplified the demap-all equation
                                // Pls confirm that this is okay. Otherwise we will nee
                                // qualifying bits for the pid and r fields.
                                demap_hit[9]    =
                                        (~mismatch[9] & demap_all_but_locked_hit[9] & demap_other
                                                & tlb_entry_vld[9]) ;
				// qualification with demap_auto to prevent ademap_hit from
				// being cleared. Satya-we could get rid of this.
                                end
                        else    begin
                                demap_hit[9]    =
                                        (~mismatch[9] & demap_other & tlb_entry_vld[9]) ;
				// qualification with demap_auto to prevent ademap_hit from
				// being cleared. Satya-this is the only one we need.
                                end
//			no_hit = cam_vld ;
			end
		    	begin
			tag[58:0] = tte_tag_ram[10] ;	// CHANGE

			mismatch_va_b47_28[10] = 
			(tag[53:34] 
			!= cam_data[40+13:21+13]);

			mismatch_va_b27_22[10] = 
			(tag[33:28] 
			!= cam_data[19+13:14+13]);

			mismatch_va_b21_16[10] = 
			(tag[23:18]
			!= cam_data[12+13:7+13]) ;

			mismatch_va_b15_13[10] = 
			(tag[16:14]
			!= cam_data[5+13:3+13]) ;

			mismatch_ctxt[10] = 
			(tag[12:0] 
			!= cam_data[12:0]) ;
			
			mismatch_pid[10] = (tag[58:56] != cam_pid[2:0]) ;

			mismatch[10] =
			(mismatch_va_b47_28[10] & cam_data[20+13]) 				|
			(mismatch_va_b27_22[10] & tag[27] & cam_data[13+13]) 	|
			(mismatch_va_b21_16[10] & tag[17] & cam_data[6+13]) 	|
			(mismatch_va_b15_13[10] & tag[13] & cam_data[2+13]) 	|
			(mismatch_ctxt[10] & ~cam_data[1+13])	|
			// mismatch is request type not equal to entry type. types are real/virtual.
			((tag[55] ^ cam_data[0+13]) & ~demap_all)  	| 
			mismatch_pid[10] ;	// pid always included in mismatch calculations

			demap_all_but_locked_hit[10] = 
			~tag[25] & demap_all ;

			cam_hit[10] 	= 
				~mismatch[10] & cam_vld   & tlb_entry_vld[10] ;

                        if (demap_all)  begin
                                // Satya(10/3) - I've simplified the demap-all equation
                                // Pls confirm that this is okay. Otherwise we will nee
                                // qualifying bits for the pid and r fields.
                                demap_hit[10]    =
                                        (~mismatch[10] & demap_all_but_locked_hit[10] & demap_other
                                                & tlb_entry_vld[10]) ;
				// qualification with demap_auto to prevent ademap_hit from
				// being cleared. Satya-we could get rid of this.
                                end
                        else    begin
                                demap_hit[10]    =
                                        (~mismatch[10] & demap_other & tlb_entry_vld[10]) ;
				// qualification with demap_auto to prevent ademap_hit from
				// being cleared. Satya-this is the only one we need.
                                end
//			no_hit = cam_vld ;
			end
		    	begin
			tag[58:0] = tte_tag_ram[11] ;	// CHANGE

			mismatch_va_b47_28[11] = 
			(tag[53:34] 
			!= cam_data[40+13:21+13]);

			mismatch_va_b27_22[11] = 
			(tag[33:28] 
			!= cam_data[19+13:14+13]);

			mismatch_va_b21_16[11] = 
			(tag[23:18]
			!= cam_data[12+13:7+13]) ;

			mismatch_va_b15_13[11] = 
			(tag[16:14]
			!= cam_data[5+13:3+13]) ;

			mismatch_ctxt[11] = 
			(tag[12:0] 
			!= cam_data[12:0]) ;
			
			mismatch_pid[11] = (tag[58:56] != cam_pid[2:0]) ;

			mismatch[11] =
			(mismatch_va_b47_28[11] & cam_data[20+13]) 				|
			(mismatch_va_b27_22[11] & tag[27] & cam_data[13+13]) 	|
			(mismatch_va_b21_16[11] & tag[17] & cam_data[6+13]) 	|
			(mismatch_va_b15_13[11] & tag[13] & cam_data[2+13]) 	|
			(mismatch_ctxt[11] & ~cam_data[1+13])	|
			// mismatch is request type not equal to entry type. types are real/virtual.
			((tag[55] ^ cam_data[0+13]) & ~demap_all)  	| 
			mismatch_pid[11] ;	// pid always included in mismatch calculations

			demap_all_but_locked_hit[11] = 
			~tag[25] & demap_all ;

			cam_hit[11] 	= 
				~mismatch[11] & cam_vld   & tlb_entry_vld[11] ;

                        if (demap_all)  begin
                                // Satya(10/3) - I've simplified the demap-all equation
                                // Pls confirm that this is okay. Otherwise we will nee
                                // qualifying bits for the pid and r fields.
                                demap_hit[11]    =
                                        (~mismatch[11] & demap_all_but_locked_hit[11] & demap_other
                                                & tlb_entry_vld[11]) ;
				// qualification with demap_auto to prevent ademap_hit from
				// being cleared. Satya-we could get rid of this.
                                end
                        else    begin
                                demap_hit[11]    =
                                        (~mismatch[11] & demap_other & tlb_entry_vld[11]) ;
				// qualification with demap_auto to prevent ademap_hit from
				// being cleared. Satya-this is the only one we need.
                                end
//			no_hit = cam_vld ;
			end
		    	begin
			tag[58:0] = tte_tag_ram[12] ;	// CHANGE

			mismatch_va_b47_28[12] = 
			(tag[53:34] 
			!= cam_data[40+13:21+13]);

			mismatch_va_b27_22[12] = 
			(tag[33:28] 
			!= cam_data[19+13:14+13]);

			mismatch_va_b21_16[12] = 
			(tag[23:18]
			!= cam_data[12+13:7+13]) ;

			mismatch_va_b15_13[12] = 
			(tag[16:14]
			!= cam_data[5+13:3+13]) ;

			mismatch_ctxt[12] = 
			(tag[12:0] 
			!= cam_data[12:0]) ;
			
			mismatch_pid[12] = (tag[58:56] != cam_pid[2:0]) ;

			mismatch[12] =
			(mismatch_va_b47_28[12] & cam_data[20+13]) 				|
			(mismatch_va_b27_22[12] & tag[27] & cam_data[13+13]) 	|
			(mismatch_va_b21_16[12] & tag[17] & cam_data[6+13]) 	|
			(mismatch_va_b15_13[12] & tag[13] & cam_data[2+13]) 	|
			(mismatch_ctxt[12] & ~cam_data[1+13])	|
			// mismatch is request type not equal to entry type. types are real/virtual.
			((tag[55] ^ cam_data[0+13]) & ~demap_all)  	| 
			mismatch_pid[12] ;	// pid always included in mismatch calculations

			demap_all_but_locked_hit[12] = 
			~tag[25] & demap_all ;

			cam_hit[12] 	= 
				~mismatch[12] & cam_vld   & tlb_entry_vld[12] ;

                        if (demap_all)  begin
                                // Satya(10/3) - I've simplified the demap-all equation
                                // Pls confirm that this is okay. Otherwise we will nee
                                // qualifying bits for the pid and r fields.
                                demap_hit[12]    =
                                        (~mismatch[12] & demap_all_but_locked_hit[12] & demap_other
                                                & tlb_entry_vld[12]) ;
				// qualification with demap_auto to prevent ademap_hit from
				// being cleared. Satya-we could get rid of this.
                                end
                        else    begin
                                demap_hit[12]    =
                                        (~mismatch[12] & demap_other & tlb_entry_vld[12]) ;
				// qualification with demap_auto to prevent ademap_hit from
				// being cleared. Satya-this is the only one we need.
                                end
//			no_hit = cam_vld ;
			end
		    	begin
			tag[58:0] = tte_tag_ram[13] ;	// CHANGE

			mismatch_va_b47_28[13] = 
			(tag[53:34] 
			!= cam_data[40+13:21+13]);

			mismatch_va_b27_22[13] = 
			(tag[33:28] 
			!= cam_data[19+13:14+13]);

			mismatch_va_b21_16[13] = 
			(tag[23:18]
			!= cam_data[12+13:7+13]) ;

			mismatch_va_b15_13[13] = 
			(tag[16:14]
			!= cam_data[5+13:3+13]) ;

			mismatch_ctxt[13] = 
			(tag[12:0] 
			!= cam_data[12:0]) ;
			
			mismatch_pid[13] = (tag[58:56] != cam_pid[2:0]) ;

			mismatch[13] =
			(mismatch_va_b47_28[13] & cam_data[20+13]) 				|
			(mismatch_va_b27_22[13] & tag[27] & cam_data[13+13]) 	|
			(mismatch_va_b21_16[13] & tag[17] & cam_data[6+13]) 	|
			(mismatch_va_b15_13[13] & tag[13] & cam_data[2+13]) 	|
			(mismatch_ctxt[13] & ~cam_data[1+13])	|
			// mismatch is request type not equal to entry type. types are real/virtual.
			((tag[55] ^ cam_data[0+13]) & ~demap_all)  	| 
			mismatch_pid[13] ;	// pid always included in mismatch calculations

			demap_all_but_locked_hit[13] = 
			~tag[25] & demap_all ;

			cam_hit[13] 	= 
				~mismatch[13] & cam_vld   & tlb_entry_vld[13] ;

                        if (demap_all)  begin
                                // Satya(10/3) - I've simplified the demap-all equation
                                // Pls confirm that this is okay. Otherwise we will nee
                                // qualifying bits for the pid and r fields.
                                demap_hit[13]    =
                                        (~mismatch[13] & demap_all_but_locked_hit[13] & demap_other
                                                & tlb_entry_vld[13]) ;
				// qualification with demap_auto to prevent ademap_hit from
				// being cleared. Satya-we could get rid of this.
                                end
                        else    begin
                                demap_hit[13]    =
                                        (~mismatch[13] & demap_other & tlb_entry_vld[13]) ;
				// qualification with demap_auto to prevent ademap_hit from
				// being cleared. Satya-this is the only one we need.
                                end
//			no_hit = cam_vld ;
			end
		    	begin
			tag[58:0] = tte_tag_ram[14] ;	// CHANGE

			mismatch_va_b47_28[14] = 
			(tag[53:34] 
			!= cam_data[40+13:21+13]);

			mismatch_va_b27_22[14] = 
			(tag[33:28] 
			!= cam_data[19+13:14+13]);

			mismatch_va_b21_16[14] = 
			(tag[23:18]
			!= cam_data[12+13:7+13]) ;

			mismatch_va_b15_13[14] = 
			(tag[16:14]
			!= cam_data[5+13:3+13]) ;

			mismatch_ctxt[14] = 
			(tag[12:0] 
			!= cam_data[12:0]) ;
			
			mismatch_pid[14] = (tag[58:56] != cam_pid[2:0]) ;

			mismatch[14] =
			(mismatch_va_b47_28[14] & cam_data[20+13]) 				|
			(mismatch_va_b27_22[14] & tag[27] & cam_data[13+13]) 	|
			(mismatch_va_b21_16[14] & tag[17] & cam_data[6+13]) 	|
			(mismatch_va_b15_13[14] & tag[13] & cam_data[2+13]) 	|
			(mismatch_ctxt[14] & ~cam_data[1+13])	|
			// mismatch is request type not equal to entry type. types are real/virtual.
			((tag[55] ^ cam_data[0+13]) & ~demap_all)  	| 
			mismatch_pid[14] ;	// pid always included in mismatch calculations

			demap_all_but_locked_hit[14] = 
			~tag[25] & demap_all ;

			cam_hit[14] 	= 
				~mismatch[14] & cam_vld   & tlb_entry_vld[14] ;

                        if (demap_all)  begin
                                // Satya(10/3) - I've simplified the demap-all equation
                                // Pls confirm that this is okay. Otherwise we will nee
                                // qualifying bits for the pid and r fields.
                                demap_hit[14]    =
                                        (~mismatch[14] & demap_all_but_locked_hit[14] & demap_other
                                                & tlb_entry_vld[14]) ;
				// qualification with demap_auto to prevent ademap_hit from
				// being cleared. Satya-we could get rid of this.
                                end
                        else    begin
                                demap_hit[14]    =
                                        (~mismatch[14] & demap_other & tlb_entry_vld[14]) ;
				// qualification with demap_auto to prevent ademap_hit from
				// being cleared. Satya-this is the only one we need.
                                end
//			no_hit = cam_vld ;
			end
		    	begin
			tag[58:0] = tte_tag_ram[15] ;	// CHANGE

			mismatch_va_b47_28[15] = 
			(tag[53:34] 
			!= cam_data[40+13:21+13]);

			mismatch_va_b27_22[15] = 
			(tag[33:28] 
			!= cam_data[19+13:14+13]);

			mismatch_va_b21_16[15] = 
			(tag[23:18]
			!= cam_data[12+13:7+13]) ;

			mismatch_va_b15_13[15] = 
			(tag[16:14]
			!= cam_data[5+13:3+13]) ;

			mismatch_ctxt[15] = 
			(tag[12:0] 
			!= cam_data[12:0]) ;
			
			mismatch_pid[15] = (tag[58:56] != cam_pid[2:0]) ;

			mismatch[15] =
			(mismatch_va_b47_28[15] & cam_data[20+13]) 				|
			(mismatch_va_b27_22[15] & tag[27] & cam_data[13+13]) 	|
			(mismatch_va_b21_16[15] & tag[17] & cam_data[6+13]) 	|
			(mismatch_va_b15_13[15] & tag[13] & cam_data[2+13]) 	|
			(mismatch_ctxt[15] & ~cam_data[1+13])	|
			// mismatch is request type not equal to entry type. types are real/virtual.
			((tag[55] ^ cam_data[0+13]) & ~demap_all)  	| 
			mismatch_pid[15] ;	// pid always included in mismatch calculations

			demap_all_but_locked_hit[15] = 
			~tag[25] & demap_all ;

			cam_hit[15] 	= 
				~mismatch[15] & cam_vld   & tlb_entry_vld[15] ;

                        if (demap_all)  begin
                                // Satya(10/3) - I've simplified the demap-all equation
                                // Pls confirm that this is okay. Otherwise we will nee
                                // qualifying bits for the pid and r fields.
                                demap_hit[15]    =
                                        (~mismatch[15] & demap_all_but_locked_hit[15] & demap_other
                                                & tlb_entry_vld[15]) ;
				// qualification with demap_auto to prevent ademap_hit from
				// being cleared. Satya-we could get rid of this.
                                end
                        else    begin
                                demap_hit[15]    =
                                        (~mismatch[15] & demap_other & tlb_entry_vld[15]) ;
				// qualification with demap_auto to prevent ademap_hit from
				// being cleared. Satya-this is the only one we need.
                                end
//			no_hit = cam_vld ;
			end

	end  // always

assign	tlb_cam_hit = |cam_hit[16-1:0] ;

// Read on CAM hit occurs on negedge.
/* MOVED TO COMMON ALWAYS BLOCK
always @ (negedge clk)
	begin
		if (|cam_hit[63:0])	
			begin
			for (p=0;p<64;p=p+1)
				if (cam_hit[p])	
				begin
					rd_tte_data[42:0] <= tte_data_ram[p] ;
				end
//				no_hit = 1'b0 ;
			end
//		else	if (no_hit) begin
//			rd_tte_data[42:0] <= {43{1'bx}};
//			no_hit = 1'b0 ;
//			end
	end
*/
// Change tlb_entry_vld handling for multi-threaded tlb writes.
// A write is always preceeded by an autodemap. The intent is to make the result of autodemap
// (clearing of vld bit if hit) invisible until write occurs. In the same cycle that the write
// occurs, the vld bit for an entry will be cleared if there is an autodemap hit. The write
// and admp action may even be to same entry. The write must dominate. There is no need to
// clear the dmp latches after the write/clear has occurred as the subsequent admp will set
// up new state in the latches.

// Define valid bit based on write/demap/reset.

 



always @ (negedge clk)
	begin
        if (async_reset)
        begin
            tlb_entry_vld[0] <= 1'b0 ;
        end
        else if ((demap_hit[0] & ~demap_auto) | sync_reset)   // non-auto-demap, reset
            tlb_entry_vld[0] <= 1'b0 ; 
        else if (((rw_index_vld & rw_wdline[0]) | (~rw_index_vld & tlb_entry_replace_d2[0])) & 
            wr_vld & ~ wr_csm_sel & ~rw_disable)
                tlb_entry_vld[0] <= wr_tte_tag[26] ;	// write
        else	begin
            if (ademap_hit[0] & wr_vld & ~wr_csm_sel)			// autodemap specifically
                tlb_entry_vld[0] <= 1'b0 ;		
            end
	end
    

always @ (negedge clk)
	begin
        if (async_reset)
        begin
            tlb_entry_vld[1] <= 1'b0 ;
        end
        else if ((demap_hit[1] & ~demap_auto) | sync_reset)   // non-auto-demap, reset
            tlb_entry_vld[1] <= 1'b0 ; 
        else if (((rw_index_vld & rw_wdline[1]) | (~rw_index_vld & tlb_entry_replace_d2[1])) & 
            wr_vld & ~ wr_csm_sel & ~rw_disable)
                tlb_entry_vld[1] <= wr_tte_tag[26] ;	// write
        else	begin
            if (ademap_hit[1] & wr_vld & ~wr_csm_sel)			// autodemap specifically
                tlb_entry_vld[1] <= 1'b0 ;		
            end
	end
    

always @ (negedge clk)
	begin
        if (async_reset)
        begin
            tlb_entry_vld[2] <= 1'b0 ;
        end
        else if ((demap_hit[2] & ~demap_auto) | sync_reset)   // non-auto-demap, reset
            tlb_entry_vld[2] <= 1'b0 ; 
        else if (((rw_index_vld & rw_wdline[2]) | (~rw_index_vld & tlb_entry_replace_d2[2])) & 
            wr_vld & ~ wr_csm_sel & ~rw_disable)
                tlb_entry_vld[2] <= wr_tte_tag[26] ;	// write
        else	begin
            if (ademap_hit[2] & wr_vld & ~wr_csm_sel)			// autodemap specifically
                tlb_entry_vld[2] <= 1'b0 ;		
            end
	end
    

always @ (negedge clk)
	begin
        if (async_reset)
        begin
            tlb_entry_vld[3] <= 1'b0 ;
        end
        else if ((demap_hit[3] & ~demap_auto) | sync_reset)   // non-auto-demap, reset
            tlb_entry_vld[3] <= 1'b0 ; 
        else if (((rw_index_vld & rw_wdline[3]) | (~rw_index_vld & tlb_entry_replace_d2[3])) & 
            wr_vld & ~ wr_csm_sel & ~rw_disable)
                tlb_entry_vld[3] <= wr_tte_tag[26] ;	// write
        else	begin
            if (ademap_hit[3] & wr_vld & ~wr_csm_sel)			// autodemap specifically
                tlb_entry_vld[3] <= 1'b0 ;		
            end
	end
    

always @ (negedge clk)
	begin
        if (async_reset)
        begin
            tlb_entry_vld[4] <= 1'b0 ;
        end
        else if ((demap_hit[4] & ~demap_auto) | sync_reset)   // non-auto-demap, reset
            tlb_entry_vld[4] <= 1'b0 ; 
        else if (((rw_index_vld & rw_wdline[4]) | (~rw_index_vld & tlb_entry_replace_d2[4])) & 
            wr_vld & ~ wr_csm_sel & ~rw_disable)
                tlb_entry_vld[4] <= wr_tte_tag[26] ;	// write
        else	begin
            if (ademap_hit[4] & wr_vld & ~wr_csm_sel)			// autodemap specifically
                tlb_entry_vld[4] <= 1'b0 ;		
            end
	end
    

always @ (negedge clk)
	begin
        if (async_reset)
        begin
            tlb_entry_vld[5] <= 1'b0 ;
        end
        else if ((demap_hit[5] & ~demap_auto) | sync_reset)   // non-auto-demap, reset
            tlb_entry_vld[5] <= 1'b0 ; 
        else if (((rw_index_vld & rw_wdline[5]) | (~rw_index_vld & tlb_entry_replace_d2[5])) & 
            wr_vld & ~ wr_csm_sel & ~rw_disable)
                tlb_entry_vld[5] <= wr_tte_tag[26] ;	// write
        else	begin
            if (ademap_hit[5] & wr_vld & ~wr_csm_sel)			// autodemap specifically
                tlb_entry_vld[5] <= 1'b0 ;		
            end
	end
    

always @ (negedge clk)
	begin
        if (async_reset)
        begin
            tlb_entry_vld[6] <= 1'b0 ;
        end
        else if ((demap_hit[6] & ~demap_auto) | sync_reset)   // non-auto-demap, reset
            tlb_entry_vld[6] <= 1'b0 ; 
        else if (((rw_index_vld & rw_wdline[6]) | (~rw_index_vld & tlb_entry_replace_d2[6])) & 
            wr_vld & ~ wr_csm_sel & ~rw_disable)
                tlb_entry_vld[6] <= wr_tte_tag[26] ;	// write
        else	begin
            if (ademap_hit[6] & wr_vld & ~wr_csm_sel)			// autodemap specifically
                tlb_entry_vld[6] <= 1'b0 ;		
            end
	end
    

always @ (negedge clk)
	begin
        if (async_reset)
        begin
            tlb_entry_vld[7] <= 1'b0 ;
        end
        else if ((demap_hit[7] & ~demap_auto) | sync_reset)   // non-auto-demap, reset
            tlb_entry_vld[7] <= 1'b0 ; 
        else if (((rw_index_vld & rw_wdline[7]) | (~rw_index_vld & tlb_entry_replace_d2[7])) & 
            wr_vld & ~ wr_csm_sel & ~rw_disable)
                tlb_entry_vld[7] <= wr_tte_tag[26] ;	// write
        else	begin
            if (ademap_hit[7] & wr_vld & ~wr_csm_sel)			// autodemap specifically
                tlb_entry_vld[7] <= 1'b0 ;		
            end
	end
    

always @ (negedge clk)
	begin
        if (async_reset)
        begin
            tlb_entry_vld[8] <= 1'b0 ;
        end
        else if ((demap_hit[8] & ~demap_auto) | sync_reset)   // non-auto-demap, reset
            tlb_entry_vld[8] <= 1'b0 ; 
        else if (((rw_index_vld & rw_wdline[8]) | (~rw_index_vld & tlb_entry_replace_d2[8])) & 
            wr_vld & ~ wr_csm_sel & ~rw_disable)
                tlb_entry_vld[8] <= wr_tte_tag[26] ;	// write
        else	begin
            if (ademap_hit[8] & wr_vld & ~wr_csm_sel)			// autodemap specifically
                tlb_entry_vld[8] <= 1'b0 ;		
            end
	end
    

always @ (negedge clk)
	begin
        if (async_reset)
        begin
            tlb_entry_vld[9] <= 1'b0 ;
        end
        else if ((demap_hit[9] & ~demap_auto) | sync_reset)   // non-auto-demap, reset
            tlb_entry_vld[9] <= 1'b0 ; 
        else if (((rw_index_vld & rw_wdline[9]) | (~rw_index_vld & tlb_entry_replace_d2[9])) & 
            wr_vld & ~ wr_csm_sel & ~rw_disable)
                tlb_entry_vld[9] <= wr_tte_tag[26] ;	// write
        else	begin
            if (ademap_hit[9] & wr_vld & ~wr_csm_sel)			// autodemap specifically
                tlb_entry_vld[9] <= 1'b0 ;		
            end
	end
    

always @ (negedge clk)
	begin
        if (async_reset)
        begin
            tlb_entry_vld[10] <= 1'b0 ;
        end
        else if ((demap_hit[10] & ~demap_auto) | sync_reset)   // non-auto-demap, reset
            tlb_entry_vld[10] <= 1'b0 ; 
        else if (((rw_index_vld & rw_wdline[10]) | (~rw_index_vld & tlb_entry_replace_d2[10])) & 
            wr_vld & ~ wr_csm_sel & ~rw_disable)
                tlb_entry_vld[10] <= wr_tte_tag[26] ;	// write
        else	begin
            if (ademap_hit[10] & wr_vld & ~wr_csm_sel)			// autodemap specifically
                tlb_entry_vld[10] <= 1'b0 ;		
            end
	end
    

always @ (negedge clk)
	begin
        if (async_reset)
        begin
            tlb_entry_vld[11] <= 1'b0 ;
        end
        else if ((demap_hit[11] & ~demap_auto) | sync_reset)   // non-auto-demap, reset
            tlb_entry_vld[11] <= 1'b0 ; 
        else if (((rw_index_vld & rw_wdline[11]) | (~rw_index_vld & tlb_entry_replace_d2[11])) & 
            wr_vld & ~ wr_csm_sel & ~rw_disable)
                tlb_entry_vld[11] <= wr_tte_tag[26] ;	// write
        else	begin
            if (ademap_hit[11] & wr_vld & ~wr_csm_sel)			// autodemap specifically
                tlb_entry_vld[11] <= 1'b0 ;		
            end
	end
    

always @ (negedge clk)
	begin
        if (async_reset)
        begin
            tlb_entry_vld[12] <= 1'b0 ;
        end
        else if ((demap_hit[12] & ~demap_auto) | sync_reset)   // non-auto-demap, reset
            tlb_entry_vld[12] <= 1'b0 ; 
        else if (((rw_index_vld & rw_wdline[12]) | (~rw_index_vld & tlb_entry_replace_d2[12])) & 
            wr_vld & ~ wr_csm_sel & ~rw_disable)
                tlb_entry_vld[12] <= wr_tte_tag[26] ;	// write
        else	begin
            if (ademap_hit[12] & wr_vld & ~wr_csm_sel)			// autodemap specifically
                tlb_entry_vld[12] <= 1'b0 ;		
            end
	end
    

always @ (negedge clk)
	begin
        if (async_reset)
        begin
            tlb_entry_vld[13] <= 1'b0 ;
        end
        else if ((demap_hit[13] & ~demap_auto) | sync_reset)   // non-auto-demap, reset
            tlb_entry_vld[13] <= 1'b0 ; 
        else if (((rw_index_vld & rw_wdline[13]) | (~rw_index_vld & tlb_entry_replace_d2[13])) & 
            wr_vld & ~ wr_csm_sel & ~rw_disable)
                tlb_entry_vld[13] <= wr_tte_tag[26] ;	// write
        else	begin
            if (ademap_hit[13] & wr_vld & ~wr_csm_sel)			// autodemap specifically
                tlb_entry_vld[13] <= 1'b0 ;		
            end
	end
    

always @ (negedge clk)
	begin
        if (async_reset)
        begin
            tlb_entry_vld[14] <= 1'b0 ;
        end
        else if ((demap_hit[14] & ~demap_auto) | sync_reset)   // non-auto-demap, reset
            tlb_entry_vld[14] <= 1'b0 ; 
        else if (((rw_index_vld & rw_wdline[14]) | (~rw_index_vld & tlb_entry_replace_d2[14])) & 
            wr_vld & ~ wr_csm_sel & ~rw_disable)
                tlb_entry_vld[14] <= wr_tte_tag[26] ;	// write
        else	begin
            if (ademap_hit[14] & wr_vld & ~wr_csm_sel)			// autodemap specifically
                tlb_entry_vld[14] <= 1'b0 ;		
            end
	end
    

always @ (negedge clk)
	begin
        if (async_reset)
        begin
            tlb_entry_vld[15] <= 1'b0 ;
        end
        else if ((demap_hit[15] & ~demap_auto) | sync_reset)   // non-auto-demap, reset
            tlb_entry_vld[15] <= 1'b0 ; 
        else if (((rw_index_vld & rw_wdline[15]) | (~rw_index_vld & tlb_entry_replace_d2[15])) & 
            wr_vld & ~ wr_csm_sel & ~rw_disable)
                tlb_entry_vld[15] <= wr_tte_tag[26] ;	// write
        else	begin
            if (ademap_hit[15] & wr_vld & ~wr_csm_sel)			// autodemap specifically
                tlb_entry_vld[15] <= 1'b0 ;		
            end
	end
    





































































































































































































































































































// async reset.
//always  @ (async_reset) 
//	begin
//	for (l=0;l<64;l=l+1)
//		begin
//	  	tlb_entry_vld[l] <= 1'b0 ;
//		end
//	end

//=========================================================================================
//	TAG COMPARISON
//=========================================================================================

reg [30:0] va_tag_plus ;

// Stage to m
always @(posedge clk)
		begin
		// INNO - add hold to this input
		if (hold)
			va_tag_plus[30:0] <= va_tag_plus[30:0] ;
		else
			va_tag_plus[30:0] 
			<= {tlb_cam_comp_key[26:0],tlb_bypass_va[12:10],tlb_bypass}; 
		end
			
assign vrtl_pgnum_m[29:0] = va_tag_plus[30:1] ;
assign bypass = va_tag_plus[0] ;

// Mux to bypass va or form pa tag based on tte-data.

assign	phy_pgnum_m[29:3] = 
	{rd_tte_data[41:30],
		rd_tte_data[29:24],
			rd_tte_data[22:17],
				rd_tte_data[15:13]};

// Derive the tlb-based physical address.
assign pgnum_m[2:0] = vrtl_pgnum_m[2:0];
assign pgnum_m[5:3] = (~rd_tte_data[12] & ~bypass)
				? phy_pgnum_m[5:3] : vrtl_pgnum_m[5:3] ;
assign pgnum_m[11:6] = (~rd_tte_data[16] & ~bypass)  
				? phy_pgnum_m[11:6] : vrtl_pgnum_m[11:6] ;
assign pgnum_m[17:12] = (~rd_tte_data[23] & ~bypass)
				? phy_pgnum_m[17:12] : vrtl_pgnum_m[17:12] ;
assign pgnum_m[29:18] = ~bypass ? phy_pgnum_m[29:18] : vrtl_pgnum_m[29:18];

// Stage to g
// Flop tags in tlb itself and do comparison immediately after rising edge.
// Similarly stage va/pa tag to g
always @(posedge clk)
		begin
			pgnum_g[29:0] <= pgnum_m[29:0];
			// rm hold on these inputs.
			cache_set_vld_g[2-1:0]  	<= cache_set_vld[2-1:0] ;
			cache_ptag_g[((((39 - (6 + 5))+1) * 2)-1):0] 	<= cache_ptag[((((39 - (6 + 5))+1) * 2)-1):0] ;
		end


// Need to stage by a cycle where used.
assign	tlb_pgnum[39:10] = pgnum_g[29:0] ;
// Same cycle as cam - meant for one load on critical path
assign	tlb_pgnum_crit[39:10] = pgnum_m[29:0] ;


   assign   cache_way_hit[0] = 
      (cache_ptag_g[((39 - (6 + 5))+1)*1-2 -: (39 - (6 + 5))] == pgnum_g[29 -: (39 - (6 + 5))]) & cache_set_vld_g[0];


   assign   cache_way_hit[1] = 
      (cache_ptag_g[((39 - (6 + 5))+1)*2-2 -: (39 - (6 + 5))] == pgnum_g[29 -: (39 - (6 + 5))]) & cache_set_vld_g[1];



assign	cache_hit = |cache_way_hit[2-1:0];


//=========================================================================================
//	TLB ENTRY REPLACEMENT
//=========================================================================================

// A single Used bit is used to track the replacement state of each entry.
// Only an unused entry can be replaced.
// An Unused entry is :
//			- an invalid entry
//			- a valid entry which has had its Used bit cleared.
//				- on write of a valid entry, the Used bit is set.
//				- The Used bit of a valid entry is cleared if all
//				entries have their Used bits set and the entry itself is not Locked.
// A locked entry should always appear to be Used.
// A single priority-encoder is required to evaluate the used status. Priority is static
// and used entry0 is of the highest priority if unused.

// Timing :
// Used bit gets updated by cam-hit or hit on negedge.
// After Used bit gets updated off negedge, the replacement entry can be generated in
// Phase2. In parallel, it is determined whether all Used bits are set or not. If
// so, then they are cleared on the next negedge with the replacement entry generated
// in the related Phase1 

// Choosing replacement entry
// Replacement entry is integer k

assign	tlb_not_writeable = &used[16-1:0] ;
/*
// Used bit can be set because of write or because of cam-hit.
always @(negedge clk)
	begin
		for (s=0;s<64;s=s+1)
			begin
				if (cam_hit[s]) 
					tlb_entry_used[s] <= 1'b1;			
			end

// Clear on following edge if necessary.
// CHANGE(SATYA) : tlb_entry_used qualified with valid needs to be used to determine
// whether the Used bits are to be cleared. This allows invalid entries created
// by a demap to be used for replacement. Else we will ignore these entries
// for replacement

		if (tlb_not_writeable)
			begin
				for (t=0;t<64;t=t+1)
					begin
						if (~tlb_entry_locked[t])
							tlb_entry_used[t] <= 1'b0;
					end
			end
	end
*/

// Determine whether entry should be squashed.

assign	used[16-1:0] = tlb_entry_used[16-1:0] & tlb_entry_vld[16-1:0] ;

/*assign squash[0] = 1'b0 ;
assign squash[1] = ~used[0] ;
assign squash[2] = |(~used[1:0]) ;
assign squash[3] = |(~used[2:0]) ;
assign squash[4] = |(~used[3:0]) ;
assign squash[5] = |(~used[4:0]) ;
assign squash[6] = |(~used[5:0]) ;
assign squash[7] = |(~used[6:0]) ;
assign squash[8] = |(~used[7:0]) ;
assign squash[9] = |(~used[8:0]) ;
assign squash[10] = |(~used[9:0]) ;
assign squash[11] = |(~used[10:0]) ;
assign squash[12] = |(~used[11:0]) ;
assign squash[13] = |(~used[12:0]) ;
assign squash[14] = |(~used[13:0]) ;
assign squash[15] = |(~used[14:0]) ;
assign squash[16] = |(~used[15:0]) ;
assign squash[17] = |(~used[16:0]) ;
assign squash[18] = |(~used[17:0]) ;
assign squash[19] = |(~used[18:0]) ;
assign squash[20] = |(~used[19:0]) ;
assign squash[21] = |(~used[20:0]) ;
assign squash[22] = |(~used[21:0]) ;
assign squash[23] = |(~used[22:0]) ;
assign squash[24] = |(~used[23:0]) ;
assign squash[25] = |(~used[24:0]) ;
assign squash[26] = |(~used[25:0]) ;
assign squash[27] = |(~used[26:0]) ;
assign squash[28] = |(~used[27:0]) ;
assign squash[29] = |(~used[28:0]) ;
assign squash[30] = |(~used[29:0]) ;
assign squash[31] = |(~used[30:0]) ;
assign squash[32] = |(~used[31:0]) ;
assign squash[33] = |(~used[32:0]) ;
assign squash[34] = |(~used[33:0]) ;
assign squash[35] = |(~used[34:0]) ;
assign squash[36] = |(~used[35:0]) ;
assign squash[37] = |(~used[36:0]) ;
assign squash[38] = |(~used[37:0]) ;
assign squash[39] = |(~used[38:0]) ;
assign squash[40] = |(~used[39:0]) ;
assign squash[41] = |(~used[40:0]) ;
assign squash[42] = |(~used[41:0]) ;
assign squash[43] = |(~used[42:0]) ;
assign squash[44] = |(~used[43:0]) ;
assign squash[45] = |(~used[44:0]) ;
assign squash[46] = |(~used[45:0]) ;
assign squash[47] = |(~used[46:0]) ;
assign squash[48] = |(~used[47:0]) ;
assign squash[49] = |(~used[48:0]) ;
assign squash[50] = |(~used[49:0]) ;
assign squash[51] = |(~used[50:0]) ;
assign squash[52] = |(~used[51:0]) ;
assign squash[53] = |(~used[52:0]) ;
assign squash[54] = |(~used[53:0]) ;
assign squash[55] = |(~used[54:0]) ;
assign squash[56] = |(~used[55:0]) ;
assign squash[57] = |(~used[56:0]) ;
assign squash[58] = |(~used[57:0]) ;
assign squash[59] = |(~used[58:0]) ;
assign squash[60] = |(~used[59:0]) ;
assign squash[61] = |(~used[60:0]) ;
assign squash[62] = |(~used[61:0]) ;
assign squash[63] = |(~used[62:0]) ; */

// Based on updated Used state, generate replacement entry.
// So, replacement entries can be generated on a cycle-by-cycle basis. 
//always @(/*AUTOSENSE*/squash or used)

	reg	[16-1:0]	tlb_entry_replace_d1;
	always @ *
	begin
  	  tlb_entry_replace_d1 = {16{1'b0}};
  	  // Priority is given to entry0

                if(~used[0])
                begin
                  tlb_entry_replace_d1[0] = ~used[0] ;
                end
         

                else if(~used[1])
                begin
                  tlb_entry_replace_d1[1] = ~used[1] ;
                end
         

                else if(~used[2])
                begin
                  tlb_entry_replace_d1[2] = ~used[2] ;
                end
         

                else if(~used[3])
                begin
                  tlb_entry_replace_d1[3] = ~used[3] ;
                end
         

                else if(~used[4])
                begin
                  tlb_entry_replace_d1[4] = ~used[4] ;
                end
         

                else if(~used[5])
                begin
                  tlb_entry_replace_d1[5] = ~used[5] ;
                end
         

                else if(~used[6])
                begin
                  tlb_entry_replace_d1[6] = ~used[6] ;
                end
         

                else if(~used[7])
                begin
                  tlb_entry_replace_d1[7] = ~used[7] ;
                end
         

                else if(~used[8])
                begin
                  tlb_entry_replace_d1[8] = ~used[8] ;
                end
         

                else if(~used[9])
                begin
                  tlb_entry_replace_d1[9] = ~used[9] ;
                end
         

                else if(~used[10])
                begin
                  tlb_entry_replace_d1[10] = ~used[10] ;
                end
         

                else if(~used[11])
                begin
                  tlb_entry_replace_d1[11] = ~used[11] ;
                end
         

                else if(~used[12])
                begin
                  tlb_entry_replace_d1[12] = ~used[12] ;
                end
         

                else if(~used[13])
                begin
                  tlb_entry_replace_d1[13] = ~used[13] ;
                end
         

                else if(~used[14])
                begin
                  tlb_entry_replace_d1[14] = ~used[14] ;
                end
         

                else if(~used[15])
                begin
                  tlb_entry_replace_d1[15] = ~used[15] ;
                end
         

  	  else
      begin
      	     tlb_entry_replace_d1[16-1] = 1'b1;
 	  end
	end
	always @(posedge clk)
	begin
	  // named in this manner to keep arch model happy.
  	  tlb_entry_replace <= tlb_entry_replace_d1 ;
	end
	// INNO - 2 stage delay before update is visible
	always @(posedge clk)
	begin
  	  tlb_entry_replace_d2 <= tlb_entry_replace ;
	end

//=========================================================================================
//	TLB WRITEABLE DETECTION
//=========================================================================================

// 2-cycles later, tlb become writeable
always @(posedge clk)
	begin
		tlb_not_writeable_d1 <= tlb_not_writeable ;
	end

always @(posedge clk)
	begin
		tlb_writeable <= ~tlb_not_writeable_d1 ;
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

// Wraps the SPU and ties unused signals 

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





































































































































































































































































































































































































































































































































































































module sparc_ffu_nospu_wrap
(
    input                rclk,
    
    input                arst_l,
    input                grst_l,

    input                lsu_ffu_st_dtlb_perr_g,
    input                exu_ffu_ist_e,    
    input [1:0]          ifu_ffu_tid_d,
    input [63:0]         cpx_fpu_data,
    input                cpx_vld,
    input                cpx_fcmp,
    input [3:0]          cpx_req,
    input [1:0]          cpx_fccval,
    input [4:0]          cpx_fpexc,
    input [31:0]         exu_ffu_gsr_mask_m,
    input [4:0]          exu_ffu_gsr_scale_m,
    input [2:0]          exu_ffu_gsr_align_m,
    input [2:0]          exu_ffu_gsr_rnd_m,
    input                ifu_ffu_ldst_single_d,
    // input                mux_drive_disable,
    // input                mem_write_disable,
    

    output [80:0]        ffu_lsu_data,
    output [3:0]         ffu_ifu_cc_vld_w2,
    output [7:0]         ffu_ifu_cc_w2,
    output               ffu_ifu_ecc_ce_w2,      
    output               ffu_ifu_ecc_ue_w2,
    output [5:0]         ffu_ifu_err_reg_w2,
    output [13:0]        ffu_ifu_err_synd_w2,
    output               ffu_ifu_fpop_done_w2,
    output               ffu_ifu_fst_ce_w,
    output               ffu_ifu_inj_ack,
    output               ffu_ifu_stallreq,
    output [1:0]         ffu_ifu_tid_w2,
    output               ffu_lsu_blk_st_e,
    output [5:3]         ffu_lsu_blk_st_va_e,
    output               ffu_lsu_fpop_rq_vld,
    output               ffu_lsu_kill_fst_w,
    output               ffu_tlu_fpu_cmplt,
    output [1:0]         ffu_tlu_fpu_tid,
    output               ffu_tlu_ill_inst_m,
    output               ffu_tlu_trap_ieee754,
    output               ffu_tlu_trap_other,
    output               ffu_tlu_trap_ue,
    output [63:0]        ffu_exu_rsr_data_m,
    input                exu_ffu_wsr_inst_e,
    input                ifu_exu_disable_ce_e,
    input [6:0]          ifu_exu_ecc_mask,
    input                ifu_exu_nceen_e,
    input [1:0]          ifu_ffu_fcc_num_d,
    input                ifu_ffu_fld_d,
    input                ifu_ffu_fpop1_d,
    input                ifu_ffu_fpop2_d,
    input [8:0]          ifu_ffu_fpopcode_d,
    input [4:0]          ifu_ffu_frd_d,
    input [4:0]          ifu_ffu_frs1_d,        
    input [4:0]          ifu_ffu_frs2_d,
    input                ifu_ffu_fst_d,
    input                ifu_ffu_inj_frferr,
    input                ifu_ffu_ldfsr_d,
    input                ifu_ffu_ldxfsr_d,
    input                ifu_ffu_mvcnd_m,
    input                ifu_ffu_quad_op_e,
    input                ifu_ffu_stfsr_d,
    input                ifu_ffu_visop_d,
    input                ifu_lsu_ld_inst_e,
    input                ifu_tlu_flsh_inst_e,  
    input                ifu_tlu_flush_w,
    input                ifu_tlu_inst_vld_w,
    input [6:0]          ifu_tlu_sraddr_d,
    input                lsu_ffu_ack,
    input [2:0]          lsu_ffu_bld_cnt_w,
    input                lsu_ffu_blk_asi_e,
    input                lsu_ffu_flush_pipe_w,
    input [63:0]         lsu_ffu_ld_data,
    input                lsu_ffu_ld_vld,
    input                lsu_ffu_stb_full0,
    input                lsu_ffu_stb_full1,
    input                lsu_ffu_stb_full2,      
    input                lsu_ffu_stb_full3,

    // sram wrapper interface
    output [4-1:0] srams_rtap_data,
    input  [4-1:0] rtap_srams_bist_command,
    input  [4-1:0] rtap_srams_bist_data
);

    sparc_ffu ffu(
                 .short_si0             (),
                 .short_so0             (),
                 .si                    (),
                 .so                    (),
                 // reset stuff
                 .grst_l                (grst_l),
                 .arst_l                (arst_l),
                 
       // eco 6529 .
       .lsu_ffu_st_dtlb_perr_g      (lsu_ffu_st_dtlb_perr_g),

                 .exu_ffu_ist_e         (exu_ffu_ist_e),
                 .ifu_ffu_tid_d         (ifu_ffu_tid_d),
                 .cpx_fpu_data          (cpx_fpu_data),
                 .cpx_vld             (cpx_vld),
                 .cpx_fcmp            (cpx_fcmp),
                 .cpx_req             (cpx_req),
                 .cpx_fccval          (cpx_fccval),
                 .cpx_fpexc           (cpx_fpexc),                   
                 .exu_ffu_gsr_mask_m  (exu_ffu_gsr_mask_m),
                 .exu_ffu_gsr_scale_m (exu_ffu_gsr_scale_m),
                 .exu_ffu_gsr_align_m (exu_ffu_gsr_align_m),
                 .exu_ffu_gsr_rnd_m   (exu_ffu_gsr_rnd_m),
                 .ifu_ffu_ldst_single_d   (ifu_ffu_ldst_single_d),
                     /*AUTOINST*/
                 // Outputs
                 .ffu_lsu_data          (ffu_lsu_data),
                 .ffu_ifu_cc_vld_w2     (ffu_ifu_cc_vld_w2),
                 .ffu_ifu_cc_w2         (ffu_ifu_cc_w2),
                 .ffu_ifu_ecc_ce_w2     (ffu_ifu_ecc_ce_w2),
                 .ffu_ifu_ecc_ue_w2     (ffu_ifu_ecc_ue_w2),
                 .ffu_ifu_err_reg_w2    (ffu_ifu_err_reg_w2),
                 .ffu_ifu_err_synd_w2   (ffu_ifu_err_synd_w2),
                 .ffu_ifu_fpop_done_w2  (ffu_ifu_fpop_done_w2),
                 .ffu_ifu_fst_ce_w      (ffu_ifu_fst_ce_w),
                 .ffu_ifu_inj_ack       (ffu_ifu_inj_ack),
                 .ffu_ifu_stallreq      (ffu_ifu_stallreq),
                 .ffu_ifu_tid_w2        (ffu_ifu_tid_w2),
                 .ffu_lsu_blk_st_e      (ffu_lsu_blk_st_e),
                 .ffu_lsu_blk_st_va_e   (ffu_lsu_blk_st_va_e),
                 .ffu_lsu_fpop_rq_vld   (ffu_lsu_fpop_rq_vld),
                 .ffu_lsu_kill_fst_w    (ffu_lsu_kill_fst_w),
                 .ffu_tlu_fpu_cmplt     (ffu_tlu_fpu_cmplt),
                 .ffu_tlu_fpu_tid       (ffu_tlu_fpu_tid),
                 .ffu_tlu_ill_inst_m    (ffu_tlu_ill_inst_m),
                 .ffu_tlu_trap_ieee754  (ffu_tlu_trap_ieee754),
                 .ffu_tlu_trap_other    (ffu_tlu_trap_other),
                 .ffu_tlu_trap_ue       (ffu_tlu_trap_ue),
                 .ffu_exu_rsr_data_m    (ffu_exu_rsr_data_m),
                 // Inputs
                 // .mux_drive_disable     (mux_drive_disable),
                 // .mem_write_disable     (mem_write_disable),
                 .exu_ffu_wsr_inst_e    (exu_ffu_wsr_inst_e),
                 .ifu_exu_disable_ce_e  (ifu_exu_disable_ce_e),
                 .ifu_exu_ecc_mask      (ifu_exu_ecc_mask),
                 .ifu_exu_nceen_e       (ifu_exu_nceen_e),
                 .ifu_ffu_fcc_num_d     (ifu_ffu_fcc_num_d),
                 .ifu_ffu_fld_d         (ifu_ffu_fld_d),
                 .ifu_ffu_fpop1_d       (ifu_ffu_fpop1_d),
                 .ifu_ffu_fpop2_d       (ifu_ffu_fpop2_d),
                 .ifu_ffu_fpopcode_d    (ifu_ffu_fpopcode_d),
                 .ifu_ffu_frd_d         (ifu_ffu_frd_d),
                 .ifu_ffu_frs1_d        (ifu_ffu_frs1_d),
                 .ifu_ffu_frs2_d        (ifu_ffu_frs2_d),
                 .ifu_ffu_fst_d         (ifu_ffu_fst_d),
                 .ifu_ffu_inj_frferr    (ifu_ffu_inj_frferr),
                 .ifu_ffu_ldfsr_d       (ifu_ffu_ldfsr_d),
                 .ifu_ffu_ldxfsr_d      (ifu_ffu_ldxfsr_d),
                 .ifu_ffu_mvcnd_m       (ifu_ffu_mvcnd_m),
                 .ifu_ffu_quad_op_e     (ifu_ffu_quad_op_e),
                 .ifu_ffu_stfsr_d       (ifu_ffu_stfsr_d),
                 .ifu_ffu_visop_d       (ifu_ffu_visop_d),
                 .ifu_lsu_ld_inst_e     (ifu_lsu_ld_inst_e),
                 .ifu_tlu_flsh_inst_e   (ifu_tlu_flsh_inst_e),
                 .ifu_tlu_flush_w       (ifu_tlu_flush_w),
                 .ifu_tlu_inst_vld_w    (ifu_tlu_inst_vld_w),
                 .ifu_tlu_sraddr_d      (ifu_tlu_sraddr_d),
                 .lsu_ffu_ack           (lsu_ffu_ack),
                 .lsu_ffu_bld_cnt_w     (lsu_ffu_bld_cnt_w),
                 .lsu_ffu_blk_asi_e     (lsu_ffu_blk_asi_e),
                 .lsu_ffu_flush_pipe_w  (lsu_ffu_flush_pipe_w),
                 .lsu_ffu_ld_data       (lsu_ffu_ld_data),
                 .lsu_ffu_ld_vld        (lsu_ffu_ld_vld),
                 .lsu_ffu_stb_full0     (lsu_ffu_stb_full0),
                 .lsu_ffu_stb_full1     (lsu_ffu_stb_full1),
                 .lsu_ffu_stb_full2     (lsu_ffu_stb_full2),
                 .lsu_ffu_stb_full3     (lsu_ffu_stb_full3),
                 .rclk                  (rclk),
                 .se                    (),
                 .sehold                (),
                 // interface to srams
                 .srams_rtap_data (srams_rtap_data),
                 .rtap_srams_bist_command (rtap_srams_bist_command),
                 .rtap_srams_bist_data (rtap_srams_bist_data)
       );

endmodule
// Modified by Princeton University on June 9th, 2015
// ========== Copyright Header Begin ==========================================
// 
// OpenSPARC T1 Processor File: sparc_ffu.v
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
//  Module Name: sparc_ffu
//  Description: This is the top level for the floating point frontend unit (ffu).
//  It instantiates the control (ffu_ctl), datapath (ffu_dp), and register file
//			(frf).
*/

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









module sparc_ffu (/*AUTOARG*/
   // Outputs
   so, ffu_tlu_trap_ue, ffu_tlu_trap_other, ffu_tlu_trap_ieee754, 
   ffu_tlu_ill_inst_m, ffu_tlu_fpu_tid, ffu_tlu_fpu_cmplt, 
   ffu_lsu_kill_fst_w, ffu_lsu_fpop_rq_vld, ffu_lsu_blk_st_va_e, 
   ffu_lsu_blk_st_e, ffu_ifu_tid_w2, ffu_ifu_stallreq, 
   ffu_ifu_inj_ack, ffu_ifu_fst_ce_w, ffu_ifu_fpop_done_w2, 
   ffu_ifu_err_synd_w2, ffu_ifu_err_reg_w2, ffu_ifu_ecc_ue_w2, 
   ffu_ifu_ecc_ce_w2, ffu_ifu_cc_w2, ffu_ifu_cc_vld_w2, ffu_lsu_data, 
   short_so0, ffu_exu_rsr_data_m, 
   // Inputs
   si, sehold, se, rclk, lsu_ffu_stb_full3, lsu_ffu_stb_full2, 
   lsu_ffu_stb_full1, lsu_ffu_stb_full0, lsu_ffu_ld_vld, 
   lsu_ffu_ld_data, lsu_ffu_flush_pipe_w, lsu_ffu_blk_asi_e, 
   lsu_ffu_bld_cnt_w, lsu_ffu_ack, ifu_tlu_sraddr_d, 
   ifu_tlu_inst_vld_w, ifu_tlu_flush_w, ifu_tlu_flsh_inst_e, 
   ifu_lsu_ld_inst_e, ifu_ffu_visop_d, ifu_ffu_tid_d, 
   ifu_ffu_stfsr_d, ifu_ffu_quad_op_e, ifu_ffu_mvcnd_m, 
   ifu_ffu_ldxfsr_d, ifu_ffu_ldst_single_d, ifu_ffu_ldfsr_d, 
   ifu_ffu_inj_frferr, ifu_ffu_fst_d, ifu_ffu_frs2_d, ifu_ffu_frs1_d, 
   ifu_ffu_frd_d, ifu_ffu_fpopcode_d, ifu_ffu_fpop2_d, 
   ifu_ffu_fpop1_d, ifu_ffu_fld_d, ifu_ffu_fcc_num_d, 
   ifu_exu_nceen_e, ifu_exu_ecc_mask, ifu_exu_disable_ce_e, grst_l, 
   exu_ffu_wsr_inst_e, exu_ffu_ist_e, exu_ffu_gsr_scale_m, 
   exu_ffu_gsr_rnd_m, exu_ffu_gsr_mask_m, exu_ffu_gsr_align_m, 
   cpx_vld, cpx_req, cpx_fpu_data, cpx_fpexc, cpx_fcmp, cpx_fccval, 
   arst_l, 
   // mux_drive_disable, mem_write_disable, 
   short_si0,
//sotheas,8/17/04: eco 6529
   lsu_ffu_st_dtlb_perr_g,
//////////////////////////////////
   // sram wrapper interface
   srams_rtap_data,
   rtap_srams_bist_command,
   rtap_srams_bist_data
   ) ;


  output [4-1:0] srams_rtap_data;
  wire [4-1:0] sram_frf_rtap_data;
  assign srams_rtap_data = sram_frf_rtap_data;
  input  [4-1:0] rtap_srams_bist_command;
  input  [4-1:0] rtap_srams_bist_data;

   output [80:0]        ffu_lsu_data;           // From dp of sparc_ffu_dp.v, ...
   output               short_so0;

   // input                mux_drive_disable;
   // input                mem_write_disable;
   input                short_si0;
   /*AUTOINPUT*/
   // Beginning of automatic inputs (from unused autoinst inputs)
   input                arst_l;                 // To ctl of sparc_ffu_ctl.v
   input [1:0]          cpx_fccval;             // To ctl of sparc_ffu_ctl.v
   input                cpx_fcmp;               // To ctl of sparc_ffu_ctl.v
   input [4:0]          cpx_fpexc;              // To ctl of sparc_ffu_ctl.v
   input [63:0]         cpx_fpu_data;           // To dp of sparc_ffu_dp.v
   input [3:0]          cpx_req;                // To ctl of sparc_ffu_ctl.v
   input                cpx_vld;                // To ctl of sparc_ffu_ctl.v
   input [2:0]          exu_ffu_gsr_align_m;    // To ctl of sparc_ffu_ctl.v
   input [31:0]         exu_ffu_gsr_mask_m;     // To ctl of sparc_ffu_ctl.v
   input [2:0]          exu_ffu_gsr_rnd_m;      // To ctl of sparc_ffu_ctl.v
   input [4:0]          exu_ffu_gsr_scale_m;    // To ctl of sparc_ffu_ctl.v
   input                exu_ffu_ist_e;          // To ctl of sparc_ffu_ctl.v
   input                exu_ffu_wsr_inst_e;     // To ctl of sparc_ffu_ctl.v
   input                grst_l;                 // To ctl of sparc_ffu_ctl.v
   input                ifu_exu_disable_ce_e;   // To ctl of sparc_ffu_ctl.v
   input [6:0]          ifu_exu_ecc_mask;       // To ctl of sparc_ffu_ctl.v
   input                ifu_exu_nceen_e;        // To ctl of sparc_ffu_ctl.v
   input [1:0]          ifu_ffu_fcc_num_d;      // To ctl of sparc_ffu_ctl.v
   input                ifu_ffu_fld_d;          // To ctl of sparc_ffu_ctl.v
   input                ifu_ffu_fpop1_d;        // To ctl of sparc_ffu_ctl.v
   input                ifu_ffu_fpop2_d;        // To ctl of sparc_ffu_ctl.v
   input [8:0]          ifu_ffu_fpopcode_d;     // To ctl of sparc_ffu_ctl.v
   input [4:0]          ifu_ffu_frd_d;          // To ctl of sparc_ffu_ctl.v
   input [4:0]          ifu_ffu_frs1_d;         // To ctl of sparc_ffu_ctl.v
   input [4:0]          ifu_ffu_frs2_d;         // To ctl of sparc_ffu_ctl.v
   input                ifu_ffu_fst_d;          // To ctl of sparc_ffu_ctl.v
   input                ifu_ffu_inj_frferr;     // To ctl of sparc_ffu_ctl.v
   input                ifu_ffu_ldfsr_d;        // To ctl of sparc_ffu_ctl.v
   input                ifu_ffu_ldst_single_d;  // To ctl of sparc_ffu_ctl.v
   input                ifu_ffu_ldxfsr_d;       // To ctl of sparc_ffu_ctl.v
   input                ifu_ffu_mvcnd_m;        // To ctl of sparc_ffu_ctl.v
   input                ifu_ffu_quad_op_e;      // To ctl of sparc_ffu_ctl.v
   input                ifu_ffu_stfsr_d;        // To ctl of sparc_ffu_ctl.v
   input [1:0]          ifu_ffu_tid_d;          // To ctl of sparc_ffu_ctl.v
   input                ifu_ffu_visop_d;        // To ctl of sparc_ffu_ctl.v
   input                ifu_lsu_ld_inst_e;      // To ctl of sparc_ffu_ctl.v
   input                ifu_tlu_flsh_inst_e;    // To ctl of sparc_ffu_ctl.v
   input                ifu_tlu_flush_w;        // To ctl of sparc_ffu_ctl.v
   input                ifu_tlu_inst_vld_w;     // To ctl of sparc_ffu_ctl.v
   input [6:0]          ifu_tlu_sraddr_d;       // To ctl of sparc_ffu_ctl.v
   input                lsu_ffu_ack;            // To ctl of sparc_ffu_ctl.v
   input [2:0]          lsu_ffu_bld_cnt_w;      // To ctl of sparc_ffu_ctl.v
   input                lsu_ffu_blk_asi_e;      // To ctl of sparc_ffu_ctl.v
   input                lsu_ffu_flush_pipe_w;   // To ctl of sparc_ffu_ctl.v
   input [63:0]         lsu_ffu_ld_data;        // To dp of sparc_ffu_dp.v
   input                lsu_ffu_ld_vld;         // To ctl of sparc_ffu_ctl.v
   input                lsu_ffu_stb_full0;      // To ctl of sparc_ffu_ctl.v
   input                lsu_ffu_stb_full1;      // To ctl of sparc_ffu_ctl.v
   input                lsu_ffu_stb_full2;      // To ctl of sparc_ffu_ctl.v
   input                lsu_ffu_stb_full3;      // To ctl of sparc_ffu_ctl.v
   input                lsu_ffu_st_dtlb_perr_g; // sotheas,8/17/04: fixed eco 6529, signal to sparc_ffu_ctl.v
   input                rclk;                   // To frf of bw_r_frf.v, ...
   input                se;                     // To frf of bw_r_frf.v, ...
   input                sehold;                 // To frf of bw_r_frf.v
   input                si;                     // To dp of sparc_ffu_dp.v
   // End of automatics
   /*AUTOOUTPUT*/
   // Beginning of automatic outputs (from unused autoinst outputs)
   output [3:0]         ffu_ifu_cc_vld_w2;      // From ctl of sparc_ffu_ctl.v
   output [7:0]         ffu_ifu_cc_w2;          // From ctl of sparc_ffu_ctl.v
   output               ffu_ifu_ecc_ce_w2;      // From ctl of sparc_ffu_ctl.v
   output               ffu_ifu_ecc_ue_w2;      // From ctl of sparc_ffu_ctl.v
   output [5:0]         ffu_ifu_err_reg_w2;     // From ctl of sparc_ffu_ctl.v
   output [13:0]        ffu_ifu_err_synd_w2;    // From ctl of sparc_ffu_ctl.v
   output               ffu_ifu_fpop_done_w2;   // From ctl of sparc_ffu_ctl.v
   output               ffu_ifu_fst_ce_w;       // From ctl of sparc_ffu_ctl.v
   output               ffu_ifu_inj_ack;        // From ctl of sparc_ffu_ctl.v
   output               ffu_ifu_stallreq;       // From ctl of sparc_ffu_ctl.v
   output [1:0]         ffu_ifu_tid_w2;         // From ctl of sparc_ffu_ctl.v
   output               ffu_lsu_blk_st_e;       // From ctl of sparc_ffu_ctl.v
   output [5:3]         ffu_lsu_blk_st_va_e;    // From ctl of sparc_ffu_ctl.v
   output               ffu_lsu_fpop_rq_vld;    // From ctl of sparc_ffu_ctl.v
   output               ffu_lsu_kill_fst_w;     // From ctl of sparc_ffu_ctl.v
   output               ffu_tlu_fpu_cmplt;      // From ctl of sparc_ffu_ctl.v
   output [1:0]         ffu_tlu_fpu_tid;        // From ctl of sparc_ffu_ctl.v
   output               ffu_tlu_ill_inst_m;     // From ctl of sparc_ffu_ctl.v
   output               ffu_tlu_trap_ieee754;   // From ctl of sparc_ffu_ctl.v
   output               ffu_tlu_trap_other;     // From ctl of sparc_ffu_ctl.v
   output               ffu_tlu_trap_ue;        // From ctl of sparc_ffu_ctl.v
   output               so;                     // From dp of sparc_ffu_dp.v
   // End of automatics
   /*AUTOWIRE*/
   // Beginning of automatic wires (for undeclared instantiated-module outputs)
   wire                 ctl_dp_ecc_sel_frf;     // From ctl of sparc_ffu_ctl.v
   wire [9:0]           ctl_dp_exc_w2;          // From ctl of sparc_ffu_ctl.v
   wire [7:0]           ctl_dp_fcc_w2;          // From ctl of sparc_ffu_ctl.v
   wire                 ctl_dp_flip_fpu;        // From ctl of sparc_ffu_ctl.v
   wire                 ctl_dp_flip_lsu;        // From ctl of sparc_ffu_ctl.v
   wire [3:0]           ctl_dp_fp_thr;          // From ctl of sparc_ffu_ctl.v
   wire [3:0]           ctl_dp_fsr_sel_fpu;     // From ctl of sparc_ffu_ctl.v
   wire [3:0]           ctl_dp_fsr_sel_ld;      // From ctl of sparc_ffu_ctl.v
   wire [3:0]           ctl_dp_fsr_sel_old;     // From ctl of sparc_ffu_ctl.v
   wire [2:0]           ctl_dp_ftt_w2;          // From ctl of sparc_ffu_ctl.v
   wire [3:0]           ctl_dp_gsr_wsr_w2;      // From ctl of sparc_ffu_ctl.v
   wire                 ctl_dp_new_rs1;         // From ctl of sparc_ffu_ctl.v
   wire                 ctl_dp_noflip_fpu;      // From ctl of sparc_ffu_ctl.v
   wire                 ctl_dp_noflip_lsu;      // From ctl of sparc_ffu_ctl.v
   wire                 ctl_dp_noshift64_frf;   // From ctl of sparc_ffu_ctl.v
   wire                 ctl_dp_output_sel_frf;  // From ctl of sparc_ffu_ctl.v
   wire                 ctl_dp_output_sel_fsr;  // From ctl of sparc_ffu_ctl.v
   wire                 ctl_dp_output_sel_rs1;  // From ctl of sparc_ffu_ctl.v
   wire                 ctl_dp_output_sel_rs2;  // From ctl of sparc_ffu_ctl.v
   wire                 ctl_dp_rd_ecc;          // From ctl of sparc_ffu_ctl.v
   wire                 ctl_dp_rs2_frf_read;    // From ctl of sparc_ffu_ctl.v
   wire                 ctl_dp_rs2_keep_data;   // From ctl of sparc_ffu_ctl.v
   wire                 ctl_dp_rs2_sel_fpu_lsu; // From ctl of sparc_ffu_ctl.v
   wire                 ctl_dp_rs2_sel_vis;     // From ctl of sparc_ffu_ctl.v
   wire                 ctl_dp_rst_l;           // From ctl of sparc_ffu_ctl.v
   wire                 ctl_dp_shift_frf_left;  // From ctl of sparc_ffu_ctl.v
   wire                 ctl_dp_shift_frf_right; // From ctl of sparc_ffu_ctl.v
   wire [1:0]           ctl_dp_sign;            // From ctl of sparc_ffu_ctl.v
   wire [3:0]           ctl_dp_thr_e;           // From ctl of sparc_ffu_ctl.v
   wire [36:0]          ctl_dp_wsr_data_w2;     // From ctl of sparc_ffu_ctl.v
   wire                 ctl_dp_zero_low32_frf;  // From ctl of sparc_ffu_ctl.v
   wire [6:0]           ctl_frf_addr;           // From ctl of sparc_ffu_ctl.v
   wire                 ctl_frf_ren;            // From ctl of sparc_ffu_ctl.v
   wire [1:0]           ctl_frf_wen;            // From ctl of sparc_ffu_ctl.v
   wire                 ctl_vis_add32;          // From ctl of sparc_ffu_ctl.v
   wire                 ctl_vis_align0;         // From ctl of sparc_ffu_ctl.v
   wire                 ctl_vis_align2;         // From ctl of sparc_ffu_ctl.v
   wire                 ctl_vis_align4;         // From ctl of sparc_ffu_ctl.v
   wire                 ctl_vis_align6;         // From ctl of sparc_ffu_ctl.v
   wire                 ctl_vis_align_odd;      // From ctl of sparc_ffu_ctl.v
   wire                 ctl_vis_cin;            // From ctl of sparc_ffu_ctl.v
   wire                 ctl_vis_log_constant;   // From ctl of sparc_ffu_ctl.v
   wire                 ctl_vis_log_invert_rs1; // From ctl of sparc_ffu_ctl.v
   wire                 ctl_vis_log_invert_rs2; // From ctl of sparc_ffu_ctl.v
   wire                 ctl_vis_log_pass_const; // From ctl of sparc_ffu_ctl.v
   wire                 ctl_vis_log_pass_rs1;   // From ctl of sparc_ffu_ctl.v
   wire                 ctl_vis_log_pass_rs2;   // From ctl of sparc_ffu_ctl.v
   wire                 ctl_vis_log_sel_nand;   // From ctl of sparc_ffu_ctl.v
   wire                 ctl_vis_log_sel_nor;    // From ctl of sparc_ffu_ctl.v
   wire                 ctl_vis_log_sel_pass;   // From ctl of sparc_ffu_ctl.v
   wire                 ctl_vis_log_sel_xor;    // From ctl of sparc_ffu_ctl.v
   wire                 ctl_vis_sel_add;        // From ctl of sparc_ffu_ctl.v
   wire                 ctl_vis_sel_align;      // From ctl of sparc_ffu_ctl.v
   wire                 ctl_vis_sel_log;        // From ctl of sparc_ffu_ctl.v
   wire                 ctl_vis_subtract;       // From ctl of sparc_ffu_ctl.v
   wire [4:0]           dp_ctl_fsr_aexc;        // From dp of sparc_ffu_dp.v
   wire [4:0]           dp_ctl_fsr_cexc;        // From dp of sparc_ffu_dp.v
   wire [7:0]           dp_ctl_fsr_fcc;         // From dp of sparc_ffu_dp.v
   wire [1:0]           dp_ctl_fsr_rnd;         // From dp of sparc_ffu_dp.v
   wire [4:0]           dp_ctl_fsr_tem;         // From dp of sparc_ffu_dp.v
   wire [31:0]          dp_ctl_gsr_mask_e;      // From dp of sparc_ffu_dp.v
   wire [4:0]           dp_ctl_gsr_scale_e;     // From dp of sparc_ffu_dp.v
   wire [7:0]           dp_ctl_ld_fcc;          // From dp of sparc_ffu_dp.v
   wire [1:0]           dp_ctl_rs2_sign;        // From dp of sparc_ffu_dp.v
   wire [6:0]           dp_ctl_synd_out_high;   // From dp of sparc_ffu_dp.v
   wire [6:0]           dp_ctl_synd_out_low;    // From dp of sparc_ffu_dp.v
   wire [63:0]          dp_vis_rs1_data;        // From dp of sparc_ffu_dp.v
   wire [63:0]          dp_vis_rs2_data;        // From dp of sparc_ffu_dp.v
   wire [77:0]          frf_dp_data;            // From frf of bw_r_frf.v
   wire [63:0]          vis_dp_rd_data;         // From vis of sparc_ffu_vis.v
   // End of automatics
   wire [77:0]          dp_frf_data;
   
   output [63:0]        ffu_exu_rsr_data_m;
   wire [31:0]          ffu_exu_rsr_data_hi_m;
   wire [2:0]          ffu_exu_rsr_data_mid_m;
   wire [7:0]          ffu_exu_rsr_data_lo_m;

   wire mux_drive_disable = ~grst_l;
   wire mem_write_disable = ~grst_l;

   wire                short_scan_1;
   
   assign              ffu_exu_rsr_data_m[63:0] = {ffu_exu_rsr_data_hi_m[31:0], 4'b0, 
                                                   ffu_exu_rsr_data_mid_m[2:0], 17'b0,
                                                   ffu_exu_rsr_data_lo_m[7:0]};

   bw_r_frf frf(
                .si(short_si0),
                .so(short_scan_1),
                .dp_frf_data            (dp_frf_data[77:0]),
                .rst_tri_en             (mem_write_disable),
                /*AUTOINST*/
                // Outputs
                .frf_dp_data            (frf_dp_data[77:0]),
                // Inputs
                .rclk                   (rclk),
                .se                     (se),
                .sehold                 (sehold),
                .ctl_frf_wen            (ctl_frf_wen[1:0]),
                .ctl_frf_ren            (ctl_frf_ren),
                .ctl_frf_addr           (ctl_frf_addr[6:0]),

                // debug interface
                .srams_rtap_data   (sram_frf_rtap_data),
                .rtap_srams_bist_command   (rtap_srams_bist_command),
                .rtap_srams_bist_data   (rtap_srams_bist_data)
               );

   sparc_ffu_dp dp(
                   .dp_frf_data         ({dp_frf_data[70:39],dp_frf_data[31:0]}),
                   /*AUTOINST*/
                   // Outputs
                   .so                  (so),
                   .ffu_lsu_data        (ffu_lsu_data[63:0]),
                   .dp_vis_rs1_data     (dp_vis_rs1_data[63:0]),
                   .dp_vis_rs2_data     (dp_vis_rs2_data[63:0]),
                   .dp_ctl_rs2_sign     (dp_ctl_rs2_sign[1:0]),
                   .dp_ctl_fsr_fcc      (dp_ctl_fsr_fcc[7:0]),
                   .dp_ctl_fsr_rnd      (dp_ctl_fsr_rnd[1:0]),
                   .dp_ctl_fsr_tem      (dp_ctl_fsr_tem[4:0]),
                   .dp_ctl_fsr_aexc     (dp_ctl_fsr_aexc[4:0]),
                   .dp_ctl_fsr_cexc     (dp_ctl_fsr_cexc[4:0]),
                   .dp_ctl_ld_fcc       (dp_ctl_ld_fcc[7:0]),
                   .dp_ctl_gsr_mask_e   (dp_ctl_gsr_mask_e[31:0]),
                   .dp_ctl_gsr_scale_e  (dp_ctl_gsr_scale_e[4:0]),
                   .dp_ctl_synd_out_low (dp_ctl_synd_out_low[6:0]),
                   .dp_ctl_synd_out_high(dp_ctl_synd_out_high[6:0]),
                   // Inputs
                   .rclk                (rclk),
                   .se                  (se),
                   .si                  (si),
                   .ctl_dp_rst_l        (ctl_dp_rst_l),
                   .frf_dp_data         (frf_dp_data[77:0]),
                   .cpx_fpu_data        (cpx_fpu_data[63:0]),
                   .lsu_ffu_ld_data     (lsu_ffu_ld_data[63:0]),
                   .vis_dp_rd_data      (vis_dp_rd_data[63:0]),
                   .ctl_dp_wsr_data_w2  (ctl_dp_wsr_data_w2[36:0]),
                   .ctl_dp_sign         (ctl_dp_sign[1:0]),
                   .ctl_dp_exc_w2       (ctl_dp_exc_w2[9:0]),
                   .ctl_dp_fcc_w2       (ctl_dp_fcc_w2[7:0]),
                   .ctl_dp_ftt_w2       (ctl_dp_ftt_w2[2:0]),
                   .ctl_dp_noshift64_frf(ctl_dp_noshift64_frf),
                   .ctl_dp_shift_frf_right(ctl_dp_shift_frf_right),
                   .ctl_dp_shift_frf_left(ctl_dp_shift_frf_left),
                   .ctl_dp_zero_low32_frf(ctl_dp_zero_low32_frf),
                   .ctl_dp_output_sel_rs1(ctl_dp_output_sel_rs1),
                   .ctl_dp_output_sel_rs2(ctl_dp_output_sel_rs2),
                   .ctl_dp_output_sel_frf(ctl_dp_output_sel_frf),
                   .ctl_dp_output_sel_fsr(ctl_dp_output_sel_fsr),
                   .ctl_dp_noflip_lsu   (ctl_dp_noflip_lsu),
                   .ctl_dp_flip_lsu     (ctl_dp_flip_lsu),
                   .ctl_dp_noflip_fpu   (ctl_dp_noflip_fpu),
                   .ctl_dp_flip_fpu     (ctl_dp_flip_fpu),
                   .ctl_dp_rs2_frf_read (ctl_dp_rs2_frf_read),
                   .ctl_dp_rs2_sel_vis  (ctl_dp_rs2_sel_vis),
                   .ctl_dp_rs2_sel_fpu_lsu(ctl_dp_rs2_sel_fpu_lsu),
                   .ctl_dp_rs2_keep_data(ctl_dp_rs2_keep_data),
                   .ctl_dp_rd_ecc       (ctl_dp_rd_ecc),
                   .ctl_dp_fp_thr       (ctl_dp_fp_thr[3:0]),
                   .ctl_dp_fsr_sel_old  (ctl_dp_fsr_sel_old[3:0]),
                   .ctl_dp_fsr_sel_ld   (ctl_dp_fsr_sel_ld[3:0]),
                   .ctl_dp_fsr_sel_fpu  (ctl_dp_fsr_sel_fpu[3:0]),
                   .ctl_dp_gsr_wsr_w2   (ctl_dp_gsr_wsr_w2[3:0]),
                   .ctl_dp_thr_e        (ctl_dp_thr_e[3:0]),
                   .ctl_dp_new_rs1      (ctl_dp_new_rs1),
                   .ctl_dp_ecc_sel_frf  (ctl_dp_ecc_sel_frf));

   sparc_ffu_ctl ctl(
                     .si(short_scan_1),
                     .so                (short_so0),
                     .ffu_exu_rsr_data_hi_m(ffu_exu_rsr_data_hi_m[31:0]),
                     .ffu_exu_rsr_data_lo_m(ffu_exu_rsr_data_lo_m[7:0]),
                     .ffu_exu_rsr_data_mid_m(ffu_exu_rsr_data_mid_m[2:0]),
                     .ctl_frf_write_synd({dp_frf_data[77:71],dp_frf_data[38:32]}),
                     .rst_tri_en        (mux_drive_disable),
                     /*AUTOINST*/
                     // Outputs
                     .ctl_dp_gsr_wsr_w2 (ctl_dp_gsr_wsr_w2[3:0]),
                     .ctl_dp_thr_e      (ctl_dp_thr_e[3:0]),
                     .ctl_dp_wsr_data_w2(ctl_dp_wsr_data_w2[36:0]),
                     .ctl_vis_add32     (ctl_vis_add32),
                     .ctl_vis_align0    (ctl_vis_align0),
                     .ctl_vis_align2    (ctl_vis_align2),
                     .ctl_vis_align4    (ctl_vis_align4),
                     .ctl_vis_align6    (ctl_vis_align6),
                     .ctl_vis_align_odd (ctl_vis_align_odd),
                     .ctl_vis_cin       (ctl_vis_cin),
                     .ctl_vis_log_constant(ctl_vis_log_constant),
                     .ctl_vis_log_invert_rs1(ctl_vis_log_invert_rs1),
                     .ctl_vis_log_invert_rs2(ctl_vis_log_invert_rs2),
                     .ctl_vis_log_pass_const(ctl_vis_log_pass_const),
                     .ctl_vis_log_pass_rs1(ctl_vis_log_pass_rs1),
                     .ctl_vis_log_pass_rs2(ctl_vis_log_pass_rs2),
                     .ctl_vis_log_sel_nand(ctl_vis_log_sel_nand),
                     .ctl_vis_log_sel_nor(ctl_vis_log_sel_nor),
                     .ctl_vis_log_sel_pass(ctl_vis_log_sel_pass),
                     .ctl_vis_log_sel_xor(ctl_vis_log_sel_xor),
                     .ctl_vis_sel_add   (ctl_vis_sel_add),
                     .ctl_vis_sel_align (ctl_vis_sel_align),
                     .ctl_vis_sel_log   (ctl_vis_sel_log),
                     .ctl_vis_subtract  (ctl_vis_subtract),
                     .ctl_dp_rst_l      (ctl_dp_rst_l),
                     .ffu_ifu_fpop_done_w2(ffu_ifu_fpop_done_w2),
                     .ffu_ifu_cc_vld_w2 (ffu_ifu_cc_vld_w2[3:0]),
                     .ffu_ifu_cc_w2     (ffu_ifu_cc_w2[7:0]),
                     .ffu_ifu_tid_w2    (ffu_ifu_tid_w2[1:0]),
                     .ffu_ifu_stallreq  (ffu_ifu_stallreq),
                     .ffu_ifu_ecc_ce_w2 (ffu_ifu_ecc_ce_w2),
                     .ffu_ifu_ecc_ue_w2 (ffu_ifu_ecc_ue_w2),
                     .ffu_ifu_err_reg_w2(ffu_ifu_err_reg_w2[5:0]),
                     .ffu_ifu_err_synd_w2(ffu_ifu_err_synd_w2[13:0]),
                     .ffu_ifu_fst_ce_w  (ffu_ifu_fst_ce_w),
                     .ffu_lsu_kill_fst_w(ffu_lsu_kill_fst_w),
                     .ffu_ifu_inj_ack   (ffu_ifu_inj_ack),
                     .ffu_lsu_data      (ffu_lsu_data[80:64]),
                     .ffu_lsu_fpop_rq_vld(ffu_lsu_fpop_rq_vld),
                     .ffu_lsu_blk_st_va_e(ffu_lsu_blk_st_va_e[5:3]),
                     .ffu_lsu_blk_st_e  (ffu_lsu_blk_st_e),
                     .ffu_tlu_trap_ieee754(ffu_tlu_trap_ieee754),
                     .ffu_tlu_trap_other(ffu_tlu_trap_other),
                     .ffu_tlu_trap_ue   (ffu_tlu_trap_ue),
                     .ffu_tlu_ill_inst_m(ffu_tlu_ill_inst_m),
                     .ffu_tlu_fpu_tid   (ffu_tlu_fpu_tid[1:0]),
                     .ffu_tlu_fpu_cmplt (ffu_tlu_fpu_cmplt),
                     .ctl_frf_ren       (ctl_frf_ren),
                     .ctl_frf_wen       (ctl_frf_wen[1:0]),
                     .ctl_frf_addr      (ctl_frf_addr[6:0]),
                     .ctl_dp_fp_thr     (ctl_dp_fp_thr[3:0]),
                     .ctl_dp_fcc_w2     (ctl_dp_fcc_w2[7:0]),
                     .ctl_dp_ftt_w2     (ctl_dp_ftt_w2[2:0]),
                     .ctl_dp_exc_w2     (ctl_dp_exc_w2[9:0]),
                     .ctl_dp_ecc_sel_frf(ctl_dp_ecc_sel_frf),
                     .ctl_dp_output_sel_rs1(ctl_dp_output_sel_rs1),
                     .ctl_dp_output_sel_rs2(ctl_dp_output_sel_rs2),
                     .ctl_dp_output_sel_frf(ctl_dp_output_sel_frf),
                     .ctl_dp_output_sel_fsr(ctl_dp_output_sel_fsr),
                     .ctl_dp_rs2_frf_read(ctl_dp_rs2_frf_read),
                     .ctl_dp_rs2_sel_vis(ctl_dp_rs2_sel_vis),
                     .ctl_dp_rs2_sel_fpu_lsu(ctl_dp_rs2_sel_fpu_lsu),
                     .ctl_dp_rs2_keep_data(ctl_dp_rs2_keep_data),
                     .ctl_dp_rd_ecc     (ctl_dp_rd_ecc),
                     .ctl_dp_fsr_sel_ld (ctl_dp_fsr_sel_ld[3:0]),
                     .ctl_dp_fsr_sel_fpu(ctl_dp_fsr_sel_fpu[3:0]),
                     .ctl_dp_fsr_sel_old(ctl_dp_fsr_sel_old[3:0]),
                     .ctl_dp_noshift64_frf(ctl_dp_noshift64_frf),
                     .ctl_dp_shift_frf_right(ctl_dp_shift_frf_right),
                     .ctl_dp_shift_frf_left(ctl_dp_shift_frf_left),
                     .ctl_dp_zero_low32_frf(ctl_dp_zero_low32_frf),
                     .ctl_dp_new_rs1    (ctl_dp_new_rs1),
                     .ctl_dp_sign       (ctl_dp_sign[1:0]),
                     .ctl_dp_flip_fpu   (ctl_dp_flip_fpu),
                     .ctl_dp_flip_lsu   (ctl_dp_flip_lsu),
                     .ctl_dp_noflip_fpu (ctl_dp_noflip_fpu),
                     .ctl_dp_noflip_lsu (ctl_dp_noflip_lsu),
                     // Inputs
                     .dp_ctl_gsr_mask_e (dp_ctl_gsr_mask_e[31:0]),
                     .dp_ctl_gsr_scale_e(dp_ctl_gsr_scale_e[4:0]),
                     .exu_ffu_gsr_align_m(exu_ffu_gsr_align_m[2:0]),
                     .exu_ffu_gsr_mask_m(exu_ffu_gsr_mask_m[31:0]),
                     .exu_ffu_gsr_rnd_m (exu_ffu_gsr_rnd_m[2:0]),
                     .exu_ffu_gsr_scale_m(exu_ffu_gsr_scale_m[4:0]),
                     .exu_ffu_wsr_inst_e(exu_ffu_wsr_inst_e),
                     .ifu_tlu_sraddr_d  (ifu_tlu_sraddr_d[6:0]),
                     .lsu_ffu_st_dtlb_perr_g  (lsu_ffu_st_dtlb_perr_g), //sotheas,8/17/04: fixed eco 6529
                     .rclk              (rclk),
                     .se                (se),
                     .grst_l            (grst_l),
                     .arst_l            (arst_l),
                     .dp_ctl_rs2_sign   (dp_ctl_rs2_sign[1:0]),
                     .cpx_vld           (cpx_vld),
                     .cpx_fcmp          (cpx_fcmp),
                     .cpx_req           (cpx_req[3:0]),
                     .cpx_fccval        (cpx_fccval[1:0]),
                     .cpx_fpexc         (cpx_fpexc[4:0]),
                     .dp_ctl_fsr_fcc    (dp_ctl_fsr_fcc[7:0]),
                     .dp_ctl_fsr_rnd    (dp_ctl_fsr_rnd[1:0]),
                     .dp_ctl_fsr_tem    (dp_ctl_fsr_tem[4:0]),
                     .dp_ctl_fsr_aexc   (dp_ctl_fsr_aexc[4:0]),
                     .dp_ctl_fsr_cexc   (dp_ctl_fsr_cexc[4:0]),
                     .dp_ctl_synd_out_low(dp_ctl_synd_out_low[6:0]),
                     .dp_ctl_synd_out_high(dp_ctl_synd_out_high[6:0]),
                     .ifu_ffu_fpop1_d   (ifu_ffu_fpop1_d),
                     .ifu_ffu_fpop2_d   (ifu_ffu_fpop2_d),
                     .ifu_ffu_visop_d   (ifu_ffu_visop_d),
                     .ifu_ffu_fpopcode_d(ifu_ffu_fpopcode_d[8:0]),
                     .ifu_ffu_frs1_d    (ifu_ffu_frs1_d[4:0]),
                     .ifu_ffu_frs2_d    (ifu_ffu_frs2_d[4:0]),
                     .ifu_ffu_frd_d     (ifu_ffu_frd_d[4:0]),
                     .ifu_ffu_fld_d     (ifu_ffu_fld_d),
                     .ifu_ffu_fst_d     (ifu_ffu_fst_d),
                     .ifu_ffu_ldst_single_d(ifu_ffu_ldst_single_d),
                     .ifu_ffu_tid_d     (ifu_ffu_tid_d[1:0]),
                     .ifu_ffu_fcc_num_d (ifu_ffu_fcc_num_d[1:0]),
                     .ifu_ffu_mvcnd_m   (ifu_ffu_mvcnd_m),
                     .ifu_ffu_inj_frferr(ifu_ffu_inj_frferr),
                     .ifu_exu_ecc_mask  (ifu_exu_ecc_mask[6:0]),
                     .ifu_ffu_ldfsr_d   (ifu_ffu_ldfsr_d),
                     .ifu_ffu_ldxfsr_d  (ifu_ffu_ldxfsr_d),
                     .ifu_ffu_stfsr_d   (ifu_ffu_stfsr_d),
                     .ifu_ffu_quad_op_e (ifu_ffu_quad_op_e),
                     .ifu_tlu_inst_vld_w(ifu_tlu_inst_vld_w),
                     .lsu_ffu_flush_pipe_w(lsu_ffu_flush_pipe_w),
                     .ifu_tlu_flush_w   (ifu_tlu_flush_w),
                     .lsu_ffu_ack       (lsu_ffu_ack),
                     .lsu_ffu_ld_vld    (lsu_ffu_ld_vld),
                     .lsu_ffu_bld_cnt_w (lsu_ffu_bld_cnt_w[2:0]),
                     .dp_ctl_ld_fcc     (dp_ctl_ld_fcc[7:0]),
                     .ifu_exu_nceen_e   (ifu_exu_nceen_e),
                     .ifu_exu_disable_ce_e(ifu_exu_disable_ce_e),
                     .lsu_ffu_blk_asi_e (lsu_ffu_blk_asi_e),
                     .exu_ffu_ist_e     (exu_ffu_ist_e),
                     .ifu_tlu_flsh_inst_e(ifu_tlu_flsh_inst_e),
                     .ifu_lsu_ld_inst_e (ifu_lsu_ld_inst_e),
                     .lsu_ffu_stb_full0 (lsu_ffu_stb_full0),
                     .lsu_ffu_stb_full1 (lsu_ffu_stb_full1),
                     .lsu_ffu_stb_full2 (lsu_ffu_stb_full2),
                     .lsu_ffu_stb_full3 (lsu_ffu_stb_full3));

   sparc_ffu_vis vis(/*AUTOINST*/
                     // Outputs
                     .vis_dp_rd_data    (vis_dp_rd_data[63:0]),
                     // Inputs
                     .dp_vis_rs1_data   (dp_vis_rs1_data[63:0]),
                     .dp_vis_rs2_data   (dp_vis_rs2_data[63:0]),
                     .ctl_vis_sel_add   (ctl_vis_sel_add),
                     .ctl_vis_sel_log   (ctl_vis_sel_log),
                     .ctl_vis_sel_align (ctl_vis_sel_align),
                     .ctl_vis_add32     (ctl_vis_add32),
                     .ctl_vis_subtract  (ctl_vis_subtract),
                     .ctl_vis_cin       (ctl_vis_cin),
                     .ctl_vis_align0    (ctl_vis_align0),
                     .ctl_vis_align2    (ctl_vis_align2),
                     .ctl_vis_align4    (ctl_vis_align4),
                     .ctl_vis_align6    (ctl_vis_align6),
                     .ctl_vis_align_odd (ctl_vis_align_odd),
                     .ctl_vis_log_sel_pass(ctl_vis_log_sel_pass),
                     .ctl_vis_log_sel_nand(ctl_vis_log_sel_nand),
                     .ctl_vis_log_sel_nor(ctl_vis_log_sel_nor),
                     .ctl_vis_log_sel_xor(ctl_vis_log_sel_xor),
                     .ctl_vis_log_invert_rs1(ctl_vis_log_invert_rs1),
                     .ctl_vis_log_invert_rs2(ctl_vis_log_invert_rs2),
                     .ctl_vis_log_constant(ctl_vis_log_constant),
                     .ctl_vis_log_pass_const(ctl_vis_log_pass_const),
                     .ctl_vis_log_pass_rs1(ctl_vis_log_pass_rs1),
                     .ctl_vis_log_pass_rs2(ctl_vis_log_pass_rs2));
endmodule // sparc_ffu
// Local Variables:
// verilog-library-directories:("." "../../../srams/rtl")
// End:

// Modified by Princeton University on June 9th, 2015
// ========== Copyright Header Begin ==========================================
// 
// OpenSPARC T1 Processor File: sparc_ffu_ctl.v
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
//  Module Name: sparc_ffu_ctl
//	Description: This is the ffu control block.
*/

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














module sparc_ffu_ctl (/*AUTOARG*/
   // Outputs
   ffu_exu_rsr_data_mid_m, ffu_exu_rsr_data_lo_m, 
   ffu_exu_rsr_data_hi_m, ctl_vis_subtract, ctl_vis_sel_log, 
   ctl_vis_sel_align, ctl_vis_sel_add, ctl_vis_log_sel_xor, 
   ctl_vis_log_sel_pass, ctl_vis_log_sel_nor, ctl_vis_log_sel_nand, 
   ctl_vis_log_pass_rs2, ctl_vis_log_pass_rs1, 
   ctl_vis_log_pass_const, ctl_vis_log_invert_rs2, 
   ctl_vis_log_invert_rs1, ctl_vis_log_constant, ctl_vis_cin, 
   ctl_vis_align_odd, ctl_vis_align6, ctl_vis_align4, ctl_vis_align2, 
   ctl_vis_align0, ctl_vis_add32, ctl_dp_wsr_data_w2, ctl_dp_thr_e, 
   ctl_dp_gsr_wsr_w2, so, ctl_dp_rst_l, ffu_ifu_fpop_done_w2, 
   ffu_ifu_cc_vld_w2, ffu_ifu_cc_w2, ffu_ifu_tid_w2, 
   ffu_ifu_stallreq, ffu_ifu_ecc_ce_w2, ffu_ifu_ecc_ue_w2, 
   ffu_ifu_err_reg_w2, ffu_ifu_err_synd_w2, ffu_ifu_fst_ce_w, 
   ffu_lsu_kill_fst_w, ffu_ifu_inj_ack, ffu_lsu_data, 
   ffu_lsu_fpop_rq_vld, ffu_lsu_blk_st_va_e, ffu_lsu_blk_st_e, 
   ffu_tlu_trap_ieee754, ffu_tlu_trap_other, ffu_tlu_trap_ue, 
   ffu_tlu_ill_inst_m, ffu_tlu_fpu_tid, ffu_tlu_fpu_cmplt, 
   ctl_frf_ren, ctl_frf_wen, ctl_frf_addr, ctl_dp_fp_thr, 
   ctl_dp_fcc_w2, ctl_dp_ftt_w2, ctl_dp_exc_w2, ctl_dp_ecc_sel_frf, 
   ctl_dp_output_sel_rs1, ctl_dp_output_sel_rs2, 
   ctl_dp_output_sel_frf, ctl_dp_output_sel_fsr, ctl_dp_rs2_frf_read, 
   ctl_dp_rs2_sel_vis, ctl_dp_rs2_sel_fpu_lsu, ctl_dp_rs2_keep_data, 
   ctl_dp_rd_ecc, ctl_dp_fsr_sel_ld, ctl_dp_fsr_sel_fpu, 
   ctl_dp_fsr_sel_old, ctl_dp_noshift64_frf, ctl_dp_shift_frf_right, 
   ctl_dp_shift_frf_left, ctl_dp_zero_low32_frf, ctl_dp_new_rs1, 
   ctl_dp_sign, ctl_dp_flip_fpu, ctl_dp_flip_lsu, ctl_dp_noflip_fpu, 
   ctl_dp_noflip_lsu, ctl_frf_write_synd, 
   // Inputs
   ifu_tlu_sraddr_d, exu_ffu_wsr_inst_e, exu_ffu_gsr_scale_m, 
   exu_ffu_gsr_rnd_m, exu_ffu_gsr_mask_m, exu_ffu_gsr_align_m, 
   dp_ctl_gsr_scale_e, dp_ctl_gsr_mask_e, rclk, si, se, grst_l, 
   arst_l, rst_tri_en, dp_ctl_rs2_sign, cpx_vld, cpx_fcmp, cpx_req, 
   cpx_fccval, cpx_fpexc, dp_ctl_fsr_fcc, dp_ctl_fsr_rnd, 
   dp_ctl_fsr_tem, dp_ctl_fsr_aexc, dp_ctl_fsr_cexc, 
   dp_ctl_synd_out_low, dp_ctl_synd_out_high, ifu_ffu_fpop1_d, 
   ifu_ffu_fpop2_d, ifu_ffu_visop_d, ifu_ffu_fpopcode_d, 
   ifu_ffu_frs1_d, ifu_ffu_frs2_d, ifu_ffu_frd_d, ifu_ffu_fld_d, 
   ifu_ffu_fst_d, ifu_ffu_ldst_single_d, ifu_ffu_tid_d, 
   ifu_ffu_fcc_num_d, ifu_ffu_mvcnd_m, ifu_ffu_inj_frferr, 
   ifu_exu_ecc_mask, ifu_ffu_ldfsr_d, ifu_ffu_ldxfsr_d, 
   ifu_ffu_stfsr_d, ifu_ffu_quad_op_e, ifu_tlu_inst_vld_w, 
   lsu_ffu_flush_pipe_w, ifu_tlu_flush_w, lsu_ffu_ack, 
   lsu_ffu_ld_vld, lsu_ffu_bld_cnt_w, dp_ctl_ld_fcc, ifu_exu_nceen_e, 
   ifu_exu_disable_ce_e, lsu_ffu_blk_asi_e, exu_ffu_ist_e, 
   ifu_tlu_flsh_inst_e, ifu_lsu_ld_inst_e, lsu_ffu_stb_full0, 
   lsu_ffu_stb_full1, lsu_ffu_stb_full2, lsu_ffu_stb_full3,
//sotheas,8/17/04: fixed eco 6529
   lsu_ffu_st_dtlb_perr_g
//////////////////////////////
   ) ;
   /*AUTOINPUT*/
   // Beginning of automatic inputs (from unused autoinst inputs)
   input [31:0]         dp_ctl_gsr_mask_e;      // To visctl of sparc_ffu_ctl_visctl.v
   input [4:0]          dp_ctl_gsr_scale_e;     // To visctl of sparc_ffu_ctl_visctl.v
   input [2:0]          exu_ffu_gsr_align_m;    // To visctl of sparc_ffu_ctl_visctl.v
   input [31:0]         exu_ffu_gsr_mask_m;     // To visctl of sparc_ffu_ctl_visctl.v
   input [2:0]          exu_ffu_gsr_rnd_m;      // To visctl of sparc_ffu_ctl_visctl.v
   input [4:0]          exu_ffu_gsr_scale_m;    // To visctl of sparc_ffu_ctl_visctl.v
   input                exu_ffu_wsr_inst_e;     // To visctl of sparc_ffu_ctl_visctl.v
   input [6:0]          ifu_tlu_sraddr_d;       // To visctl of sparc_ffu_ctl_visctl.v
   // End of automatics
   input rclk;
   input si;
   input se;
   input grst_l;
   input arst_l;
   input rst_tri_en;
   input [1:0] dp_ctl_rs2_sign;

   input          cpx_vld;
   input          cpx_fcmp;
   input [3:0]    cpx_req;
   input [1:0]    cpx_fccval;
   input [4:0]    cpx_fpexc;
   input [7:0] dp_ctl_fsr_fcc;
   input [1:0] dp_ctl_fsr_rnd;
   input [4:0] dp_ctl_fsr_tem;
	 input [4:0] dp_ctl_fsr_aexc;
	 input [4:0] dp_ctl_fsr_cexc;

   input [6:0] dp_ctl_synd_out_low;   // signals for ecc errors
   input [6:0] dp_ctl_synd_out_high;

   input       ifu_ffu_fpop1_d;
   input       ifu_ffu_fpop2_d;
   input       ifu_ffu_visop_d;
   input [8:0] ifu_ffu_fpopcode_d;
   input [4:0] ifu_ffu_frs1_d;
   input [4:0] ifu_ffu_frs2_d;
   input [4:0] ifu_ffu_frd_d;
   input       ifu_ffu_fld_d;
   input       ifu_ffu_fst_d;
   input       ifu_ffu_ldst_single_d;
   input [1:0] ifu_ffu_tid_d;
   input [1:0] ifu_ffu_fcc_num_d;
   input       ifu_ffu_mvcnd_m;

   input       ifu_ffu_inj_frferr;
   input [6:0] ifu_exu_ecc_mask;

   input       ifu_ffu_ldfsr_d,
	       ifu_ffu_ldxfsr_d,
	       ifu_ffu_stfsr_d;
   input       ifu_ffu_quad_op_e;
   
   input       ifu_tlu_inst_vld_w;
   input       lsu_ffu_flush_pipe_w;
   input       ifu_tlu_flush_w;
   
   input        lsu_ffu_ack;
   input        lsu_ffu_ld_vld;
   input [2:0]  lsu_ffu_bld_cnt_w;
   input [7:0]  dp_ctl_ld_fcc;

   input        ifu_exu_nceen_e;// enable ecc traps
   input        ifu_exu_disable_ce_e; // all ce are treated as ue
   input        lsu_ffu_blk_asi_e;
   input        exu_ffu_ist_e;
   input        ifu_tlu_flsh_inst_e;
   input        ifu_lsu_ld_inst_e;
   input        lsu_ffu_stb_full0;
   input        lsu_ffu_stb_full1;
   input        lsu_ffu_stb_full2;
   input        lsu_ffu_stb_full3;

   input        lsu_ffu_st_dtlb_perr_g; //sotheas,8/17/04: fixed eco 6529, when asserted terminated
                                        //                 block store

   /*AUTOOUTPUT*/
   // Beginning of automatic outputs (from unused autoinst outputs)
   output [3:0]         ctl_dp_gsr_wsr_w2;      // From visctl of sparc_ffu_ctl_visctl.v
   output [3:0]         ctl_dp_thr_e;           // From visctl of sparc_ffu_ctl_visctl.v
   output [36:0]        ctl_dp_wsr_data_w2;     // From visctl of sparc_ffu_ctl_visctl.v
   output               ctl_vis_add32;          // From visctl of sparc_ffu_ctl_visctl.v
   output               ctl_vis_align0;         // From visctl of sparc_ffu_ctl_visctl.v
   output               ctl_vis_align2;         // From visctl of sparc_ffu_ctl_visctl.v
   output               ctl_vis_align4;         // From visctl of sparc_ffu_ctl_visctl.v
   output               ctl_vis_align6;         // From visctl of sparc_ffu_ctl_visctl.v
   output               ctl_vis_align_odd;      // From visctl of sparc_ffu_ctl_visctl.v
   output               ctl_vis_cin;            // From visctl of sparc_ffu_ctl_visctl.v
   output               ctl_vis_log_constant;   // From visctl of sparc_ffu_ctl_visctl.v
   output               ctl_vis_log_invert_rs1; // From visctl of sparc_ffu_ctl_visctl.v
   output               ctl_vis_log_invert_rs2; // From visctl of sparc_ffu_ctl_visctl.v
   output               ctl_vis_log_pass_const; // From visctl of sparc_ffu_ctl_visctl.v
   output               ctl_vis_log_pass_rs1;   // From visctl of sparc_ffu_ctl_visctl.v
   output               ctl_vis_log_pass_rs2;   // From visctl of sparc_ffu_ctl_visctl.v
   output               ctl_vis_log_sel_nand;   // From visctl of sparc_ffu_ctl_visctl.v
   output               ctl_vis_log_sel_nor;    // From visctl of sparc_ffu_ctl_visctl.v
   output               ctl_vis_log_sel_pass;   // From visctl of sparc_ffu_ctl_visctl.v
   output               ctl_vis_log_sel_xor;    // From visctl of sparc_ffu_ctl_visctl.v
   output               ctl_vis_sel_add;        // From visctl of sparc_ffu_ctl_visctl.v
   output               ctl_vis_sel_align;      // From visctl of sparc_ffu_ctl_visctl.v
   output               ctl_vis_sel_log;        // From visctl of sparc_ffu_ctl_visctl.v
   output               ctl_vis_subtract;       // From visctl of sparc_ffu_ctl_visctl.v
   output [31:0]        ffu_exu_rsr_data_hi_m;  // From visctl of sparc_ffu_ctl_visctl.v
   output [7:0]         ffu_exu_rsr_data_lo_m;  // From visctl of sparc_ffu_ctl_visctl.v
   output [2:0]         ffu_exu_rsr_data_mid_m; // From visctl of sparc_ffu_ctl_visctl.v
   // End of automatics
   output               so;
   output               ctl_dp_rst_l;
   output       ffu_ifu_fpop_done_w2;
   output [3:0] ffu_ifu_cc_vld_w2;// one hot valid for each set of fcc
   output [7:0] ffu_ifu_cc_w2;// all 4 sets of fcc
   output [1:0] ffu_ifu_tid_w2;
   output       ffu_ifu_stallreq; // stall pipe so blk st can issue
   
   output       ffu_ifu_ecc_ce_w2;  // correctable ecc error
   output       ffu_ifu_ecc_ue_w2;  // uncorrectable ecc error
   output [5:0] ffu_ifu_err_reg_w2;
   output [13:0] ffu_ifu_err_synd_w2;
   output       ffu_ifu_fst_ce_w;
   output       ffu_lsu_kill_fst_w;
   output       ffu_ifu_inj_ack;

   output [80:64] ffu_lsu_data;
   output 	  ffu_lsu_fpop_rq_vld ;   // ffu dispatches fpop issue request.
   output [5:3]  ffu_lsu_blk_st_va_e;
   output        ffu_lsu_blk_st_e;

   output 	  ffu_tlu_trap_ieee754;
   output 	  ffu_tlu_trap_other;
   output     ffu_tlu_trap_ue;
   output     ffu_tlu_ill_inst_m;

   output [1:0] ffu_tlu_fpu_tid;
   output       ffu_tlu_fpu_cmplt;
   
   output       ctl_frf_ren;
   output [1:0] ctl_frf_wen;
   output [6:0] ctl_frf_addr;

   output [3:0] ctl_dp_fp_thr;
   
   output [7:0] ctl_dp_fcc_w2;
   output [2:0] ctl_dp_ftt_w2;
   output [9:0] ctl_dp_exc_w2;

   output ctl_dp_ecc_sel_frf;
   

   // mux selects
   output       ctl_dp_output_sel_rs1;
   output 	ctl_dp_output_sel_rs2;
   output 	ctl_dp_output_sel_frf;
   output 	ctl_dp_output_sel_fsr;
   
   output 	ctl_dp_rs2_frf_read;
   output 	ctl_dp_rs2_sel_vis;
   output 	ctl_dp_rs2_sel_fpu_lsu;
   output 	ctl_dp_rs2_keep_data;
   output   ctl_dp_rd_ecc;

   output [3:0] ctl_dp_fsr_sel_ld,
		ctl_dp_fsr_sel_fpu,
		ctl_dp_fsr_sel_old;
   
   output 	ctl_dp_noshift64_frf;
   output 	ctl_dp_shift_frf_right;
   output 	ctl_dp_shift_frf_left;
   output 	ctl_dp_zero_low32_frf;
   
   output ctl_dp_new_rs1;
   
   output [1:0] ctl_dp_sign;
   
   output       ctl_dp_flip_fpu;
   output       ctl_dp_flip_lsu;
   output       ctl_dp_noflip_fpu;
   output       ctl_dp_noflip_lsu;


   wire     clk;
   wire     reset;
   wire     ffu_reset_l;
   // FPOP is broken into parts:
   // [8:4] fpop_high
   // [3:2] fpop_mid
   // [1:0] fpop_size
   wire         fpop_size_0;    // 2 lsbs of fpop
   wire         fpop_size_1;

   wire         fpop_high_0;    // 4 msbs of fpop
   wire         fpop_high_2;
   wire         fpop_high_4;
   wire         fpop_high_5; 
   wire         fpop_high_6; 
   wire         fpop_high_8;
   wire         fpop_high_a;
   wire         fpop_high_c;
   wire         fpop_high_d;
   wire         fpop_high_e;
   wire         fpop_high_10;
   wire         fpop_high_18;
   wire    fpop_low_1;
   wire    fpop_low_2;
   wire    fpop_low_4;
   wire    fpop_low_5;
   wire    fpop_low_6;
   wire    fpop_low_8;
   wire    fpop_low_9;
   wire    fpop_low_a;
   wire    fpop_low_d;
   wire    fpop_low_e;

   wire         source_single_e;
   wire         source_single_next;
   wire         source_single;

   wire         dest_single_e;
   wire         dest_single_next;
   wire         dest_single;

   wire [4:0]   frs1_e;
   wire [5:0]   rs1_e;
   wire [5:0]   rs1_next;
   wire [5:0]   rs1;
   wire [4:0]   frs2_e;
   wire [5:0]   rs2_e;
   wire [4:0]   frd_e;
   wire         ldst_single_e;
   wire [5:0]   rd_e;
   wire [5:1]   st_rd_d;
   wire [5:1]   write_addr;

   wire [5:0] 	rs2_next;
   wire [5:0] 	rs2;
   wire [5:0] 	rd_next;
   wire [5:0] 	rd;
   wire [5:1]   blk_rd;

   wire         is_fpop_d;
   wire    shift_frf_rs2_m;
   wire    shift_frf_rs1_w;
   wire    shift_frf_right_next;
   wire    shift_frf_right;
   wire    shift_frf_left_next;
   wire    shift_frf_left;
   wire    noshift64_frf_next;
   wire    noshift64_frf;

   wire 	abs_w;
   wire 	neg_w;
   wire 	cond_move_e;
   wire 	cond_move_m;
   wire 	move_e;
   wire 	move_m;
   wire 	move_m_valid;
   wire 	move_w;
   wire   move_w_valid;
   wire   move_w2;
   wire   move_w2_vld;
   wire   move_wen_m;
   wire   move_wen_w;
   wire 	move_wen_w2;
   wire 	move_wen_w2_valid;

   wire   vis_nofrf_e;
   wire 	ren_rs2_e;
   wire 	ren_rs2_e_vld;
   wire 	ren_rs2_m;
   wire 	ren_rs2_m_vld;
   wire 	ren_rs2_w;
   wire 	ren_rs2_w2;
   wire 	ren_rs2_w3;
   wire 	ren_rs2_w4;
   wire 	ren_rs1_e;
   wire 	ren_rs1_m;
   wire 	ren_rs1_w;
   wire 	ren_rs1_w_vld;
   wire 	ren_rs1_w2_vld;
   wire 	ren_rs1_w2;
   wire 	ren_rs1_w3;
   wire 	ren_rs1_w4;
   wire 	ren_rs1_w5;
   wire 	read_rs1;
   wire 	read_rs2;
   wire   read_rd;
   wire 	read_bst;

   wire 	fpu_op_e;
   wire 	fpu_op_m;
   wire 	fpu_op_w_vld;
   wire 	fpu_op_w;
   wire 	fpu_op_w2;
   wire 	fpu_op_w2_vld;
   wire 	fpu_op_w3;
   wire 	fpu_op_w3_vld;
   wire 	any_op_e;
   wire 	any_op_m;
   wire 	any_op_w;
   wire 	any_op_w2;
   wire 	any_op_m_valid;
   wire   visop_e;
   wire   visop_m;
   wire   visop_w_vld;
   
   wire 	fld_e;
   wire 	fld_m;
   wire 	fst_e;
   wire 	fst_m;
   wire 	fst_w;
   wire 	zero_lower_data_next;

   wire   fpop1_e;
   wire   fpop2_e;
   wire   fpop1_next;
   wire   fpop1;
   wire   fpop2_next;
   wire   fpop2;
   wire   visop_next;
   wire   visop;
   wire 	kill_m;
   wire 	killed_w;
   wire 	kill_w;
   wire 	kill_unimpl_w;
   wire 	kill_fp;
   wire 	kill_eccchk_w;
   wire   flush_w;
   wire   flush_w2;
   
   wire [1:0] 	tid_next;
   wire [1:0]   tid;
   wire [1:0]   extra_tid;
   wire [8:0] 	opf_next;
   wire [8:0] 	opf;
   wire [5:1] 	early_frf_rnum;
   wire [5:1] 	frf_rnum;
   wire [1:0]   frf_tid;
   wire [1:0]   fpu_rnd;

   wire 	thr_match_mw2;
   wire 	thr_match_ww2;
   wire 	thr_match_fpw2;
   wire [1:0] 	tid_e,
		tid_m,
		tid_w,
                tid_w2;

   wire 	fpop1_ready_w2_next;
   wire 	fpop2_ready_w3_next;
   wire 	fpop1_ready_w2;
   wire 	fpop2_ready_w3;
   wire   issue_fpop2;
   
   wire 	ldfsr,
		ldxfsr,
		stfsr_e,
		stfsr_w,
		stfsr_qual_w,
		ldfsr_vld,
		ldxfsr_vld;
   wire   stfsr_w2;
   wire   stfsr_w2_vld;
   
   wire   clear_ftt;
   
   wire [1:0] 	ldfsr_next;
   
   wire 	is_fpu_result;
   
   wire   output_sel_rs1_next;
   wire   output_sel_frf_next;
   wire   output_sel_fsr_next;
   wire   output_sel_rs2_next;
   wire   output_sel_rs1;
   wire   output_sel_frf;
   wire   output_sel_fsr;
   wire   output_sel_rs2;

   wire   ffu_op_done_next;
   wire   ffu_op_done;
   wire   ffu_op_done_vld;
   wire 	external_wen_next;
   wire 	lsu_pkt_vld;
   wire [1:0] 	lsu_pkt_type;
   wire 	store_ready;
   wire 	load_pending;
   wire 	load_pending_next;
   wire   blk_ld_done;
   wire   blk_ld_m;
   wire 	blk_load_pending;
   wire 	blk_load_pending_next;
   wire 	fp_pending,
		fp_pending_next;

   wire [3:0] 	fcc_num_dec;
   wire [1:0] 	fcc_num,
		fcc_num_next;
   
   wire [7:0] 	fpu_fcc;
   wire [9:0] 	fp_exc_w2;

   wire         fcc_sel_fpu;
   wire         fcc_sel_ldx;
   wire         fcc_sel_ld;
   wire         fcc_sel_old;
   wire 	cc_changed;
   wire 	rf_wen;
   wire   rf_wen_next;
   wire   rf_ecc_gen_next;
   wire   rf_ecc_gen;
   wire   vis_wen_next;
   wire   vis_result;

   wire [4:0] fsr_tem_d1;
   wire       ieee_trap;
   wire  	take_ieee_trap;
   wire 	take_other_trap;
   wire [4:0] 	ieee_trap_vec;
   wire 	fpexc_nxc;
   wire   fpexc_ofc;
   wire   fpexc_ufc;
   
   wire [1:0] error_detected;
   wire [1:0] possible_ue;
   wire [1:0] ce;
   wire [1:0] prev_err_detected;
   wire [1:0] prev_poss_ue;
   wire       rollback_fst_m;
   wire       rollback_fst_w;
   wire       rollback_rs2_w2;
   wire       rollback_rs1_w3;
   wire       rollback_c1_next;
   wire   rollback_c1;
   wire   rollback_c1_vld;
   wire   rollback_c2;
   wire   rollback_c3;
   wire   rolled_back_next;
   wire   rolled_back;
   wire   chk_rs1_w2;
   wire 	check_ecc_next;
   wire [1:0] chk_ecc_m;
   wire [1:0] chk_ecc_w;
   wire [1:0] chk_ecc_w2;
   wire [1:0] chk_ecc;
   wire [1:0] chk_ecc_prev;
   wire   disable_ce_m;
   wire   disable_ce_w;
   wire 	fst_ce_w;
   wire 	fst_ue_w;
   wire 	fst_ce_w2;
   wire 	fst_ue_w2;
   wire 	rs2_ce_w2;
   wire 	rs2_ue_w2;
   wire 	rs2_fst_ce_w2_vld;
   wire 	rs2_fst_ue_w2_vld;
   wire 	rs2_fst_ce_w3;
   wire 	rs2_fst_ue_w3;
   wire   rs1_ce_w3;
   wire 	ce_w3;
   wire 	ue_w3;
   wire   nceen;
   wire   nceen_next;
   wire   ue_trap_w3;
   wire [1:0] 	previous_ce;
   wire         previous_ue;
   wire [1:0] 	ecc_wen_next;
   wire [1:0] 	ecc_wen_gen_next;
   wire [1:0] 	ecc_wen_gen;
   wire [1:0] 	ecc_wen;
   wire 	inject_err_next;
   wire [6:0] 	err_data;
   wire 	inject_err;
   wire   wen_rs1_ecc;
   wire   wen_rs2_ecc;
   wire   ecc_kill_rs2_w2;
   wire [13:0] new_err_synd;
   wire [13:0] err_synd_d1;
   wire [13:0] err_synd_next;
   wire [5:0]  new_err_reg;
   wire [5:0]  err_reg_next;
   wire [5:0]  err_reg_d1;
   wire        log_new_err;
   wire   kill_st_ce_w;
   wire   possible_kill_st_ue_m;
   wire   possible_kill_st_ce_m;
   wire   possible_kill_st_ue_w;
   wire   possible_kill_st_ce_w;
   
   wire 	unimpl_op_e,
		unimpl_op_all_e,
		unimpl_op_m,
		unimpl_op_w,
		unimpl_qual_w,
		unimpl_qual_w2,
		unimpl_op_w2;
   wire    illegal_vis_e;
   wire    illegal_vis_m;
   wire    illegal_blk_m;
   wire    illegal_rs1_e;
   wire    illegal_field_e;
   wire    convert_op_e;
   
   wire          cpx_vld_d1;
   wire          cpx_fcmp_d1;
   wire [3:0]    cpx_req_d1;
   wire [1:0]    cpx_fccval_d1;
   wire [4:0]    cpx_fpexc_d1;

   wire   bst_m;
   wire   bst_w;
   wire   st_dtlb_perr_w2_l;
   wire   can_issue_bst_c2;
   wire   other_mem_op_e;
   wire [5:0] bst_rs;
   wire [2:0] bst_cnt;
   wire [2:0] bst_cnt_next;
   wire       bst_read_req;
   wire       bst_issue_c1;
   wire       bst_issue_c2;
   wire       bst_issue_c3;
   wire       bst_issue_c4;
   wire       bst_issue_c5;
   wire       bst_issue_c6;
   wire       bst_issue_c1_next;
   wire       bst_issue_c2_next;
   wire       bst_issue_c3_next;
   wire       bst_issue_c4_next;
   wire       bst_issue_c5_next;
   wire       bst_issue_c6_next;
   wire       bst_done;
   wire [2:0] bld_cnt_d1;
   wire [2:0] bld_cnt_d2;
   wire [2:0] bld_cnt_d3;
   wire       stb_full0;
   wire       stb_full1;
   wire       stb_full2;
   wire       stb_full3;
   wire       stb_full_c2;
   wire [5:0] bst_stall_cnt;
   wire [5:0] bst_stall_cnt_next;
   wire       bst_stall_req;
   wire       bst_stall_req_next;
   wire       fld_done;
   wire       ld_ret;
   wire   bst_ce_c4;
   wire   bst_ue_c4;
   wire   fixed_bst_ce;
   wire   fixed_bst_ce_next;
   wire   blk_asi_m;
   
//
// Code begins here
//
   assign clk = rclk;
   // Reset flop
    dffrl_async rstff(.din (grst_l),
                        .q   (ffu_reset_l),
                        .clk (clk),
                        .rst_l (arst_l), .si(), .so(), .se(se));
   assign        ctl_dp_rst_l = ffu_reset_l;
   assign        reset = ~ffu_reset_l;
   // Stage cpx data by one cycle
   dff_s #(13) cpx_dff(.din({cpx_vld, cpx_fcmp, cpx_req[3:0], cpx_fccval[1:0], cpx_fpexc[4:0]}),
                     .q({cpx_vld_d1, cpx_fcmp_d1, cpx_req_d1[3:0], cpx_fccval_d1[1:0], cpx_fpexc_d1[4:0]}),
                     .clk(clk), .se(se), .si(), .so());
   dff_s #(3) lsu_bld_cnt1_dff(.din({lsu_ffu_bld_cnt_w[2:0]}), .clk(clk),
                         .q({bld_cnt_d1[2:0]}), .se(se), .si(), .so());
   dff_s #(3) lsu_bld_cnt2_dff(.din({bld_cnt_d1[2:0]}), .clk(clk),
                         .q({bld_cnt_d2[2:0]}), .se(se), .si(), .so());
   dff_s #(3) lsu_bld_cnt3_dff(.din({bld_cnt_d2[2:0]}), .clk(clk),
                         .q({bld_cnt_d3[2:0]}), .se(se), .si(), .so());

//----------------------------------------
// Decode Instruction From IFU
//----------------------------------------
   dff_s #(3) fpop_d2e(.din({ifu_ffu_fpop1_d, ifu_ffu_fpop2_d, ifu_ffu_visop_d}), .clk(clk),
                     .q({fpop1_e, fpop2_e, visop_e}), .se(se), .si(), .so());
   assign        fpop1_next = (any_op_e | reset)? fpop1_e: fpop1;
   assign        fpop2_next = (any_op_e | reset)? fpop2_e: fpop2;
   assign        visop_next = (any_op_e | reset)? visop_e: visop;
   dff_s #(3) fpop_dff(.din({fpop1_next,fpop2_next,visop_next}),
                     .q({fpop1,fpop2,visop}),
                     .clk(clk), .se(se), .si(), .so());
   
   assign  fpop_size_0 = ~opf[1] & ~opf[0];
   assign  fpop_size_1 = ~opf[1] & opf[0];

   assign  fpop_low_1 = ~opf[3] & ~opf[2] & ~opf[1] & opf[0];
   assign  fpop_low_2 = ~opf[3] & ~opf[2] & opf[1] & ~opf[0];
   assign  fpop_low_4 = ~opf[3] & opf[2] & ~opf[1] & ~opf[0];
   assign  fpop_low_5 = ~opf[3] & opf[2] & ~opf[1] & opf[0];
   assign  fpop_low_6 = ~opf[3] & opf[2] & opf[1] & ~opf[0];
   assign  fpop_low_8 = opf[3] & ~opf[2] & ~opf[1] & ~opf[0];
   assign  fpop_low_9 = opf[3] & ~opf[2] & ~opf[1] & opf[0];
   assign  fpop_low_a = opf[3] & ~opf[2] & opf[1] & ~opf[0];
   assign  fpop_low_d = opf[3] & opf[2] & ~opf[1] & opf[0];
   assign  fpop_low_e = opf[3] & opf[2] & opf[1] & ~opf[0];
   
   assign  fpop_high_0 = ~opf[8] & ~opf[7] & ~opf[6] & ~opf[5] & ~opf[4];
   assign  fpop_high_2 = ~opf[8] & ~opf[7] & ~opf[6] & opf[5] & ~opf[4];
   assign  fpop_high_4 = ~opf[8] & ~opf[7] & opf[6] & ~opf[5] & ~opf[4];
   assign  fpop_high_5 = ~opf[8] & ~opf[7] & opf[6] & ~opf[5] & opf[4];
   assign  fpop_high_6 = ~opf[8] & ~opf[7] & opf[6] & opf[5] & ~opf[4];
   assign  fpop_high_a = ~opf[8] & opf[7] & ~opf[6] & opf[5] & ~opf[4];
   assign  fpop_high_8 = ~opf[8] & opf[7] & ~opf[6] & ~opf[5] & ~opf[4];
   assign  fpop_high_c = ~opf[8] & opf[7] & opf[6] & ~opf[5] & ~opf[4];
   assign  fpop_high_d = ~opf[8] & opf[7] & opf[6] & ~opf[5] & opf[4];
   assign  fpop_high_e = ~opf[8] & opf[7] & opf[6] & opf[5] & ~opf[4];
   assign  fpop_high_10 = opf[8] & ~opf[7] & ~opf[6] & ~opf[5] & ~opf[4];
   assign  fpop_high_18 = opf[8] & opf[7] & ~opf[6] & ~opf[5] & ~opf[4];

   assign  unimpl_op_e = ~((fpop_low_1 | fpop_low_2) & (fpop_high_0 | fpop_high_4 | fpop_high_8 |
                                                        fpop1_e & fpop_high_d |
                                                        fpop2_e & (fpop_high_5 | fpop_high_c |
                                                                           fpop_high_10 | fpop_high_18)) |
                           (fpop_low_4 | fpop_low_8) & fpop1_e & (fpop_high_8 | fpop_high_c) |
                           (fpop_low_5 | fpop_low_6) & (fpop_high_4 |
                                                        fpop1_e & fpop_high_0 |
                                                        fpop2_e & (fpop_high_2 | fpop_high_5 |
                                                                           fpop_high_6 | fpop_high_a |
                                                                           fpop_high_c | fpop_high_e)) |
                           fpop_low_6 & fpop1_e & fpop_high_c |
                           fpop_low_9 & fpop1_e & (fpop_high_0 | fpop_high_4 | fpop_high_6 |
                                                           fpop_high_c) |
                           fpop_low_a & fpop1_e & (fpop_high_0 | fpop_high_4) |
                           (fpop_low_d | fpop_low_e) & fpop1_e & fpop_high_4) & (fpop1_e | fpop2_e);
   assign illegal_field_e = fpop2_e & (fpop_high_5 & |rd_e[4:2] |// bits 29:27 must be zero on fcmp
                                       ~opf[4] & ~opf[2] & rs1_e[4]);// bit 18 must be zero on fmovcc
                                       

   assign  convert_op_e = fpop1_e & opf[7];
   assign  illegal_rs1_e = (frs1_e[4:0] != 5'b00000) & (move_e & ~rollback_c3 | convert_op_e);
      
//
// Decode size of source and destination. don't care for unimplemented ops
//
   assign source_single_e = (fpop_high_c & fpop_size_0) | //32b int
                                 (opf[0]);// single (also quad but those are illegal

/* -----\/----- EXCLUDED -----\/-----
   assign  convert_op = (ifu_ffu_fpopcode_d[7] |fpop_high_6_d) & ifu_ffu_fpop1_d;
   assign dest_single_d = (is_fpop_d)? (fpop_size_1_d & ~convert_op) | // sgl and not conv
                                         (ifu_ffu_fpop1_d & ifu_ffu_fpopcode_d[7] & ~ifu_ffu_fpopcode_d[3] &
                                          (~fpop_high_8_d | ifu_ffu_fpopcode_d[2])) |// int to s or float to short int
                                           (ifu_ffu_visop_d & ifu_ffu_fpopcode_d[0]) :// vis single
                                        ifu_ffu_ldst_single_d;
 -----/\----- EXCLUDED -----/\----- */
   assign dest_single_e = (fpop1_e & (~opf[1] & opf[0] & ~(opf[7] |fpop_high_6) | // sgl and not conv
                                              opf[7] & ~opf[3] &
                                              (~fpop_high_8 | opf[2])) |// int to s or float to short int
                           fpop2_e & fpop_size_1 | 
                           visop_e & opf[0] | // vis single
                           (fst_e | fld_e) & ldst_single_e);

   assign unimpl_op_all_e = (unimpl_op_e | ifu_ffu_quad_op_e | 
                             illegal_rs1_e | illegal_field_e);
   
   dff_s #1 qopm_ff(.din (unimpl_op_all_e),
		  .q   (unimpl_op_m),
		  .clk (clk), .se(se), .si(), .so());
   
   dff_s #1 qopw_ff(.din (unimpl_op_m),
		  .q   (unimpl_op_w),
		  .clk (clk), .se(se), .si(), .so());
   assign unimpl_qual_w = unimpl_op_w & ~kill_unimpl_w;
   assign unimpl_qual_w2 = unimpl_op_w2 & ~flush_w2;

   dff_s #1 qopw2_ff(.din (unimpl_qual_w),
		   .q   (unimpl_op_w2),
		   .clk (clk), .se(se), .si(), .so());
   
   
   // Decode register encoding (bit[5] wrapped to bit[0] for non singles)
   // Also the storage is flopped around so odd regs are at even addresses 
   // in the regfile for singles.  this helps because everything external 
   // expects data to be [63:0] not [31:0, 63:32] on doubles.
   
   assign rs1_e[5] = frs1_e[0] & ~source_single_e;// zero for singles
   assign rs1_e[4:2] = frs1_e[4:2];
   assign rs1_e[1] = frs1_e[1];
   assign rs1_e[0] = frs1_e[0] & source_single_e;// only nonzro for sgl

   assign rs2_e[5] = frs2_e[0] & ~source_single_e;// zero for singles
   assign rs2_e[4:2] = frs2_e[4:2];
   assign rs2_e[1] = frs2_e[1];
   assign rs2_e[0] = frs2_e[0] & source_single_e;// only nonzro for sgl

   assign rd_e[5] = frd_e[0] & ~dest_single_e;// zero for singles
   assign rd_e[4:2] = frd_e[4:2];
   assign rd_e[1] = frd_e[1];
   assign rd_e[0] = frd_e[0] & dest_single_e;// only nonzro for sgl


   // Decode general type of operation
   assign is_fpop_d = ifu_ffu_fpop1_d | ifu_ffu_fpop2_d | ifu_ffu_visop_d;
	  
   // Do locally
   assign move_e = fpop_high_0 & (fpop1_e | (fpop1 & rollback_c3)) |
                                                (rollback_c3 & fpop2 & ~opf[4]);// rollback cond_move
   // cond moves don't get rollback because they either don't happen or become unconditional
   assign cond_move_e = fpop2_e & ~opf[4];

   assign abs_w = move_w & fpop_high_0 & opf[3];
   assign neg_w = move_w & fpop_high_0 & opf[2];

   // Send to FPU
   assign fpu_op_e = ((~fpop_high_0 & fpop1_next) | (fpop_high_5 & fpop2_next)) & (any_op_e | rollback_c3);

   // FRF read indication
   assign ren_rs2_e = (fpop1_e | fpop2_e | visop_e) | rollback_c3;
   assign ren_rs2_e_vld = ren_rs2_e & ~vis_nofrf_e;
   assign ren_rs2_m_vld = ren_rs2_m & (cond_move_m & ifu_ffu_mvcnd_m | ~cond_move_m);
   assign ren_rs1_e = ((~vis_nofrf_e & visop_e) | // all vis except siam read rs1
                       (rollback_c3 & visop) |
                       ((fpop2_next & opf[4]) | //FCMP
                        (fpop1_next & ~opf[7] & opf[6])) &	// add,sub,mul,div
                       (any_op_e | rollback_c3 & ~reset));
   assign ren_rs1_w_vld = ren_rs1_w & ~kill_eccchk_w;
   assign ren_rs1_w2_vld = ren_rs1_w2 & ~flush_w2;

   //------------------------------------
   // Store and wait for FPop to complete
   //------------------------------------
   // Storage of control signals

   // >>>>> added ~kill_fp
   // set these in e so that kill_fp doesn't kill spuriously
   assign load_pending_next = fld_e |          // set
	        (load_pending & ~lsu_ffu_ld_vld & ~kill_fp & ~blk_ld_m);
   assign fld_done = lsu_ffu_ld_vld & ~kill_fp & load_pending;
   dffr_s ldpend_dff(.din (load_pending_next), 
		               .q   (load_pending),
		               .clk (clk), 
		               .rst (reset),
		               .se(se), .si(), .so());

   assign blk_ld_m = fld_m & blk_asi_m;
   assign blk_ld_done = lsu_ffu_ld_vld & ~kill_fp & (bld_cnt_d1[2:0] == 3'b111) & blk_load_pending;
   assign blk_load_pending_next = (blk_ld_m & ~kill_m)  | // set
          (blk_load_pending & ~kill_fp & ~ffu_ifu_fpop_done_w2);
   dffr_s blk_ldpend_dff(.din(blk_load_pending_next),
                       .q(blk_load_pending),
                       .clk(clk),
                       .rst(reset),
                       .se(se), .si(), .so());
   
   assign fp_pending_next = fpu_op_e |   // set
	                    (fp_pending & ~is_fpu_result & ~kill_fp);
   dffr_s fppend_dff(.din(fp_pending_next), 
		               .q(fp_pending),
		               .clk(clk), 
		               .rst (reset),
		               .se(se), .si(), .so());

   // rs1
   dff_s #(5) rs1_d2e(.din(ifu_ffu_frs1_d[4:0]), .clk(clk), .q(frs1_e[4:0]), .se(se), .si(), .so());
   mux2ds #(6) rs1_mux(.dout (rs1_next[5:0]),
                          .in0  (rs1[5:0]),
                          .in1  (rs1_e[5:0]),
                          .sel0  (~any_op_e),
                          .sel1  (any_op_e));
   dff_s #(6) rs1_dff(.din(rs1_next[5:0]), 
		                .clk(clk), 
		                .q(rs1[5:0]),
                    .se(se), .si(), .so());
   // rs2
   dff_s #(5) rs2_d2e(.din(ifu_ffu_frs2_d[4:0]), .clk(clk), .q(frs2_e[4:0]), .se(se), .si(), .so());
   mux2ds #(6) rs2_mux(.dout(rs2_next[5:0]),
                       .in0 (rs2[5:0]),
                       .in1 (rs2_e[5:0]),
                       .sel0(~any_op_e),
                       .sel1 (any_op_e));
   dff_s #(6) rs2_dff(.din (rs2_next[5:0]), 
		                .clk (clk), 
		                .q   (rs2[5:0]),
                    .se(se), .si(), .so());
   // rd
   dff_s #(6) rd_d2e(.din({ifu_ffu_ldst_single_d,ifu_ffu_frd_d[4:0]}), .clk(clk), 
                   .q({ldst_single_e,frd_e[4:0]}),
                   .se(se), .si(), .so());
   mux2ds #(6) rd_mux(.dout (rd_next[5:0]),
                      .in0  (rd[5:0]),
                      .in1  (rd_e[5:0]),
                      .sel0 (~any_op_e),
                      .sel1  (any_op_e));
   dff_s #(6) rd_dff(.din (rd_next[5:0]), 
		               .clk (clk),
		               .q    (rd[5:0]),
                   .se  (se), .si(), .so());
   // rs size
   mux2ds source_single_mux(.dout (source_single_next),
                            .in0  (source_single),
                            .in1  (source_single_e),
                            .sel0(~any_op_e),
                            .sel1 (any_op_e));
   dff_s source_single_dff(.din(source_single_next), 
			                   .clk(clk), 
			                   .q(source_single),
                         .se(se), .si(), .so());
   // rd size
   assign dest_single_next = (any_op_e)? dest_single_e: dest_single;
   dff_s dest_single_dff(.din (dest_single_next), 
			                    .clk (clk), 
			                    .q   (dest_single),
                          .se  (se), .si(), .so());
   // thread
   mux2ds #(2) tid_mux(.dout (tid_next[1:0]),
                       .in0  (tid[1:0]),
                       .in1  (tid_e[1:0]),
                       .sel0 (~any_op_e),
                       .sel1  (any_op_e));
   dff_s #(2) tid_dff(.din(tid_next[1:0]), 
		                .clk(clk), 
		                .q(tid[1:0]),
                    .se(se), .si(), .so());
   // extra tid to help fanout for critical signals
   dff_s #(2) extra_tid_dff(.din(tid_next[1:0]),
                          .clk(clk), .q(extra_tid[1:0]), .se(se), .si(), .so());
   // fcc num
   mux2ds #(2) fcc_mux(.dout (fcc_num_next[1:0]),
                       .in0   (fcc_num[1:0]),
                       .in1   (ifu_ffu_fcc_num_d[1:0]),
                       .sel0 (~is_fpop_d),
                       .sel1   (is_fpop_d));
   dff_s #(2) fcc_dff(.din (fcc_num_next[1:0]), 
		                .clk (clk), 
		                .q   (fcc_num[1:0]),
                    .se  (se), .si(), .so());

   // ldfsr
   mux2ds #(2) ldfsr_mux(.dout (ldfsr_next[1:0]),
                         .in0  ({ldfsr, ldxfsr}),
                         .in1  ({ifu_ffu_ldfsr_d, ifu_ffu_ldxfsr_d}),
                         .sel0  (~ifu_ffu_fld_d),
                         .sel1  (ifu_ffu_fld_d));
   dff_s #(2) ldfsr_dff(.din (ldfsr_next[1:0]), 
		                  .clk (clk), 
		                  .q   ({ldfsr, ldxfsr}),
                      .se  (se), .si(), .so());

   // op code
   mux2ds #(9) opf_mux(.dout (opf_next[8:0]),
                       .in0  (opf[8:0]),
                       .in1  (ifu_ffu_fpopcode_d[8:0]),
                       .sel0  (~is_fpop_d),
                       .sel1  (is_fpop_d));
   dff_s #(9) opf_dff(.din(opf_next[8:0]), 
		                .clk(clk), 
		                .q(opf[8:0]),
                   .se(se), .si(), .so());
   //----------
   // FP Pipe
   //----------
   dff_s fop_e2m(.din(any_op_e), .clk(clk), 
	             .q(any_op_m), 
	             .se(se), .si(), .so());

   dff_s fop_m2w(.din(any_op_m_valid), .clk(clk), 
	             .q(any_op_w), 
	             .se(se), .si(), .so());
   dff_s fop_w2w2(.din(any_op_w), .clk(clk), 
	             .q(any_op_w2), 
	             .se(se), .si(), .so());

   dff_s fst_d2e(.din(ifu_ffu_fst_d),  .clk(clk),  
	             .q  (fst_e), 
	             .se(se), .si(), .so());
   dff_s fst_e2m(.din(fst_e), .clk(clk), 
	             .q  (fst_m), 
	             .se(se), .si(), .so());
   dff_s fst_m2w(.din(fst_m), .clk(clk), 
	             .q  (fst_w), 
	             .se(se), .si(), .so());
   dff_s fld_d2e(.din(ifu_ffu_fld_d),  .clk(clk),  
	             .q  (fld_e), 
	             .se(se), .si(), .so());
   dff_s fld_e2m(.din(fld_e),  .clk(clk),  
	             .q  (fld_m), 
	             .se(se), .si(), .so());

   dff_s ren_rs2_e2m(.din(ren_rs2_e_vld), .clk(clk), 
		               .q(ren_rs2_m), 
		               .se(se), .si(), .so());
   dff_s ren_rs2_m2w(.din(ren_rs2_m_vld), .clk(clk), 
		               .q(ren_rs2_w), 
		               .se(se), .si(), .so());
   dff_s ren_rs2_w2w2(.din(ren_rs2_w), .clk(clk), 
		                .q(ren_rs2_w2), 
		                .se(se), .si(), .so());
   dff_s ren_rs2_w22w3(.din(ren_rs2_w2), .clk(clk), 
		                .q(ren_rs2_w3), 
		                .se(se), .si(), .so());
   dff_s ren_rs2_w32w4(.din(ren_rs2_w3), .clk(clk), 
		                .q(ren_rs2_w4), 
		                .se(se), .si(), .so());
   
   dff_s ren_rs1_e2m(.din(ren_rs1_e), .clk(clk), 
		               .q(ren_rs1_m), 
		               .se(se), .si(), .so());
   dff_s ren_rs1_m2w(.din(ren_rs1_m), .clk(clk), 
		               .q(ren_rs1_w), 
		               .se(se), .si(), .so());
   dff_s ren_rs1_w2w2(.din(ren_rs1_w_vld), .clk(clk), 
		               .q(ren_rs1_w2), 
		               .se(se), .si(), .so());
   dff_s ren_rs1_w22w3(.din(ren_rs1_w2_vld), .clk(clk), 
		               .q(ren_rs1_w3), 
		               .se(se), .si(), .so());
   dff_s ren_rs1_w32w4(.din(ren_rs1_w3), .clk(clk), 
		               .q(ren_rs1_w4), 
		               .se(se), .si(), .so());
   dff_s ren_rs1_w42w5(.din(ren_rs1_w4), .clk(clk), 
		               .q(ren_rs1_w5), 
		               .se(se), .si(), .so());
  
   dff_s cond_move_e2m(.din(cond_move_e), .clk(clk), 
		                 .q(cond_move_m), 
		                 .se(se), .si(), .so());
   
   dff_s move_e2m(.din(move_e), .clk(clk), 
		            .q(move_m), 
		            .se(se), .si(), .so());
   dff_s move_m2w(.din(move_m_valid), .clk(clk), 
		            .q(move_w), 
		            .se(se), .si(), .so());
   dff_s move_wen_m2w(.din(move_wen_m), .clk(clk), .q(move_wen_w),
                    .se(se), .si(), .so());
   dff_s move_wen_w2w2(.din(move_wen_w), .clk(clk), .q(move_wen_w2),
                    .se(se), .si(), .so());
   dff_s move_wdff(.din(move_w_valid), .clk(clk), .q(move_w2), .se(se), .si(), .so());

   dff_s stfsr_d2e(.din(ifu_ffu_stfsr_d), 
		             .q(stfsr_e), 
		             .clk(clk), 
		             .se(se), .si(), .so());
  
   dff_s fpu_op_e2m(.din(fpu_op_e), .clk(clk), 
		              .q(fpu_op_m), 
		              .se(se), .si(), .so());
   dff_s fpu_op_m2w(.din(fpu_op_m), .clk(clk), 
		              .q(fpu_op_w), 
		              .se(se), .si(), .so());
   dff_s fpu_op_w2w2(.din(fpu_op_w_vld), .clk(clk), 
		              .q(fpu_op_w2), 
		              .se(se), .si(), .so());
   dff_s fpu_op_w22w3(.din(fpu_op_w2_vld), .clk(clk), 
		              .q(fpu_op_w3), 
		              .se(se), .si(), .so());

   dff_s #(2) tid_d2e(.din(ifu_ffu_tid_d[1:0]), 
		                .clk(clk), 
		                .q(tid_e[1:0]), 
		                .se(se), .si(), .so());
   
   dff_s #(2) tid_e2m(.din(tid_e[1:0]), 
		                .clk(clk), 
		                .q(tid_m[1:0]), 
		                .se(se), .si(), .so());
   
   dff_s #(2) tid_m2w(.din(tid_m[1:0]), 
		                .clk(clk), 
		                .q(tid_w[1:0]), 
		                .se(se), .si(), .so());
   
   dff_s #(2) tid_w2w2(.din(tid_w[1:0]), 
		                 .clk(clk), 
		                 .q(tid_w2[1:0]), 
		                 .se(se), .si(), .so());

   dff_s dff_killed_w(.din(kill_m), 
		                .clk(clk), 
		                .q(killed_w), 
		                .se(se), .si(), .so());

   dff_s dff_flush_w2(.din(flush_w), .clk(clk), .q(flush_w2), .se(se), .si(), .so());
   
   assign  thr_match_mw2 = ~((tid_m[1] ^ tid_w2[1]) |
                           (tid_m[0] ^ tid_w2[0]));
   assign  thr_match_ww2 = ~((tid_w[1] ^ tid_w2[1]) |
                           (tid_w[0] ^ tid_w2[0]));

   assign thr_match_fpw2 = ~((tid[1] ^ tid_w2[1]) |
                           (tid[0] ^ tid_w2[0]));

   // new fpops squash previous ones (only possible in m or w, but w will also have ifu_tlu_flush_w)
   // all kill_w signals do not include lsu_ffu_flush_pipe_w.  This must be included at the final destination
   assign flush_w = (lsu_ffu_flush_pipe_w | ifu_tlu_flush_w) & ~rolled_back;
   assign any_op_e = fpop1_e | fpop2_e | fst_e | fld_e | visop_e;
   assign any_op_m_valid = any_op_m & ~any_op_e;
   assign  kill_m = (thr_match_mw2 & flush_w2) | any_op_e;
   assign  kill_eccchk_w = (~ifu_tlu_inst_vld_w | killed_w | unimpl_op_w | any_op_e | 
                            (thr_match_ww2 & flush_w2)) & ~rolled_back;
   // unimplemented ops don't check rolled_back because they trap before rollback happens
   assign  kill_unimpl_w = (~ifu_tlu_inst_vld_w | killed_w | any_op_e 
	                  | (thr_match_ww2 & flush_w2));
   assign  kill_w = (~ifu_tlu_inst_vld_w | killed_w | any_op_e |
	                  unimpl_op_w | ffu_lsu_kill_fst_w | (thr_match_ww2 & flush_w2)) & ~rolled_back;
   // this kills the "pending" signals that are set in the E stage.
   // Since they are set in E all the kills can be delayed by one cycle without
   // squashing a new, valid op
   assign  kill_fp = (thr_match_fpw2 & flush_w2 | any_op_e |
		                  any_op_w & (~ifu_tlu_inst_vld_w | unimpl_op_w)) & ~rolled_back;
   

//----------------------------
// Control for muxes that manipulate data to/from FRF
//----------------------------
   // implement fmov/fmovcc
   assign  move_wen_m = move_m | (cond_move_m & ifu_ffu_mvcnd_m);
   assign  move_wen_w2_valid = move_wen_w2 & move_w2_vld;
   assign  move_m_valid = (move_m | cond_move_m);
   // used for updating fsr
   assign  move_w_valid = move_w & ~kill_w;
   assign  move_w2_vld = move_w2 & ~flush_w2 & ~rollback_rs2_w2 & ~(rs2_ue_w2 & nceen);

   // negation or absolute value happen to rs2 in the m_stage if needed
   assign  ctl_dp_sign[1] = (dp_ctl_rs2_sign[1] ^ neg_w) & ~abs_w;
   assign  ctl_dp_sign[0] = (source_single) ? 
			     (dp_ctl_rs2_sign[0] ^ neg_w) & ~abs_w : 
	                      dp_ctl_rs2_sign[0];

   //
   // Shifts to align sgl precision 32b data
   //
   // mux for moving around single data from frf
   
   // shift on moves or stores
   assign shift_frf_rs2_m = (rs2[0] ^ rd[0]) & (move_m | cond_move_m | visop_m) & ~fst_e;
   assign shift_frf_rs1_w = (rs1[0] ^ rd[0]) & visop_w_vld; //check for squash

   assign  shift_frf_right_next = (source_single & shift_frf_rs2_m & ~rs2[0]) |
                                      (source_single & shift_frf_rs1_w & ~rs1[0]) |
                                      (dest_single_e & fst_e & ~rd_e[0]);

   assign  shift_frf_left_next = ((source_single & rs2[0] & (shift_frf_rs2_m | fpu_op_m & ~fst_e) |
                                   source_single & rs1[0] & (shift_frf_rs1_w | fpu_op_w_vld)) 
                                  & ~shift_frf_right_next);

   assign  noshift64_frf_next = ~(shift_frf_right_next | shift_frf_left_next);
   
   assign  ctl_dp_shift_frf_right = shift_frf_right & ~rst_tri_en;
   assign  ctl_dp_shift_frf_left = shift_frf_left & ~rst_tri_en;
   assign  ctl_dp_noshift64_frf = noshift64_frf | rst_tri_en;

   // fpu expects lower 32 bits to be zero on single operands
   assign  zero_lower_data_next = ((source_single & ~rs1[0] & fpu_op_w_vld) | 
                                   (source_single & ~rs2[0] & fpu_op_m));
   
   dff_s shift_frf_right_dff(.din(shift_frf_right_next), .clk(clk), .q(shift_frf_right),
                           .se(se), .si(), .so());
   dff_s shift_frf_left_dff(.din(shift_frf_left_next), .clk(clk), .q(shift_frf_left),
                           .se(se), .si(), .so());
   dff_s noshift64_dff(.din(noshift64_frf_next), .clk(clk), .q(noshift64_frf),
                     .se(se), .si(), .so());
   dff_s noshift32_dff(.din(zero_lower_data_next), .clk(clk), .q(ctl_dp_zero_low32_frf),
                     .se(se), .si(), .so());
   
   wire    flip_fpu;
   wire    flip_lsu;
   // mux for rearranging data from fpu
   // data comes in with msb always at b63.  This means that singles with an odd
   // rd must be flipped so that the data ends up in the correct
   //  registers.
   assign  flip_fpu = (dest_single & rd[0]);// single with odd rd

   // mux for rearranging data from lsu
   // data comes in [63:0].  This means that singles with an even
   // rd must be flipped so that the data ends up in the correct
   //  registers.
   assign  flip_lsu = (dest_single & ~rd[0]);// single with even rd

   assign  ctl_dp_noflip_lsu = ld_ret & ~flip_lsu;
   assign  ctl_dp_flip_lsu = ld_ret & flip_lsu;
   assign  ctl_dp_noflip_fpu = ~ld_ret & ~flip_fpu & is_fpu_result & ~cpx_fcmp_d1;
   assign  ctl_dp_flip_fpu = ~ld_ret & ~ctl_dp_noflip_fpu;
   

//---------------------------------
// LSU Interface
//---------------------------------

   // Note that stores fit into the standard pipeline so they are automatically 
   // accepted and do not require an ACK.  The lsu will check for kills in m and w.
   
   assign  store_ready = fst_m | bst_issue_c3;
   assign  fpu_op_w_vld = fpu_op_w & ~kill_w;
   assign  fpu_op_w2_vld = fpu_op_w2 & ~flush_w2 & ~ecc_kill_rs2_w2;
   assign  fpu_op_w3_vld = fpu_op_w3 & ~ue_trap_w3 & ~rollback_rs1_w3;
   // don't qual with inst_vld since it takes too much time?
   // Resolved with Sanjay:
   //    Will never receive ack in the same cycle req was first made
   assign  fpop1_ready_w2_next = (fpu_op_w3_vld | 
				                         (fpop1_ready_w2 & ~lsu_ffu_ack));

   dffr_s #1 fpop1_w2_dff(.din (fpop1_ready_w2_next), 
		                   .q   (fpop1_ready_w2),
		                   .rst (reset),
		                   .clk (clk), .se(se), .si(), .so());

   // once op1 has been accepted, move to w2, and in the next cycle op2 is ready
   //
   // C1 -- recv ack, send op1 (fpop1_ready_w2)
   // C2 -- send op2 (fpop2_ready_w3)
   assign fpop2_ready_w3_next = fpop1_ready_w2 & lsu_ffu_ack;
   
   dff_s fpop2_w22w3(.din (fpop2_ready_w3_next), 
		               .q   (fpop2_ready_w3), 
		               .clk(clk), .se(se), .si(), .so());

   
   // request in W2 and wait till an ack is received
   //    Will never receive ack in the same cycle req was first made
   assign ffu_lsu_fpop_rq_vld = fpu_op_w3_vld;

   // valid pkt sent to lsu (after request)
   assign issue_fpop2 = fpop2_ready_w3 & ~opf[7];// not conversion op
   assign  lsu_pkt_vld = fpop1_ready_w2 | fpu_op_w3_vld | issue_fpop2 | store_ready;

   assign lsu_pkt_type[1:0] = {store_ready, fpop2_ready_w3};

   // Create packet for LSU: ffu_lsu_data[80:0]
   //     80 = vld
   //     79:78 = type (00 = fpu operand 1, 01 = fpu operand 2, 10 = fp store)
   //     77:76 = tid
   //     75:68 = floating point opcode
   //     67:66 = fcc
   //     65:64 = rounding mode
   //     63:0 = data
   assign ffu_lsu_data[80:64] = {lsu_pkt_vld, 
				 lsu_pkt_type[1:0], 
				 extra_tid[1:0], 
				 opf[7:0], 
				 fcc_num[1:0], 
				 fpu_rnd[1:0]};  // rounding mode

   // Select data to send to LSU.  This is calculated one cycle early and flopped
   assign output_sel_rs1_next = fpop2_ready_w3_next & ~fst_e & ~bst_issue_c3_next;     // rs2 is sent first (fpop1)
   assign output_sel_frf_next = fst_e & ~stfsr_e | bst_issue_c3_next;   // store data
   assign output_sel_fsr_next = fst_e & stfsr_e & ~bst_issue_c3_next;
   assign output_sel_rs2_next = ~(fpop2_ready_w3_next | fst_e | bst_issue_c3_next);
   assign ctl_dp_output_sel_rs1 = output_sel_rs1 & ~rst_tri_en;
   assign ctl_dp_output_sel_frf = output_sel_frf & ~rst_tri_en;
   assign ctl_dp_output_sel_fsr = output_sel_fsr & ~rst_tri_en;
   assign ctl_dp_output_sel_rs2 = output_sel_rs2 | rst_tri_en;
   dff_s #(4) output_sel_dff(.din({output_sel_rs1_next,output_sel_rs2_next,output_sel_frf_next,output_sel_fsr_next}),
                           .q({output_sel_rs1,output_sel_rs2,output_sel_frf,output_sel_fsr}),
                           .clk(clk), .se(se), .si(), .so());


   dff_s #1 sfsrw_ff(.din (ctl_dp_output_sel_fsr),
		               .q   (stfsr_w),
		               .clk (clk), .se(se), .si(), .so());
   dff_s stfsr_wdff(.din(stfsr_qual_w), .clk(clk), .q(stfsr_w2), .se(se), .si(), .so());
   assign stfsr_qual_w =  (stfsr_w & ~kill_w);
   assign stfsr_w2_vld = stfsr_w2 & ~flush_w2;


   //------------------------------------------------------
   //  Block Stores
   //------------------------------------------------------
   // interface with lsu.  bst packet issues in c3
   // check for stb_full so it doesn't confuse lsu.  Don't have to count inflight packets
   // because none exist by bst_issue_c2
   assign        ffu_lsu_blk_st_e = bst_issue_c2 & ~stb_full_c2;
   assign        ffu_lsu_blk_st_va_e[5:3] = bst_cnt[2:0];
   assign        bst_done = bst_issue_c4 & (bst_cnt[2:0] == 3'b111) & ~bst_ce_c4 & ~(bst_ue_c4 & nceen);

   mux4ds stb_full_mux (.dout(stb_full_c2),
                        .in0(stb_full0),
                        .in1(stb_full1),
                        .in2(stb_full2),
                        .in3(stb_full3),
                        .sel0(ctl_dp_fp_thr[0]),
                        .sel1(ctl_dp_fp_thr[1]),
                        .sel2(ctl_dp_fp_thr[2]),
                        .sel3(ctl_dp_fp_thr[3]));

   assign        other_mem_op_e = exu_ffu_ist_e | ifu_tlu_flsh_inst_e | ifu_lsu_ld_inst_e;
   assign        can_issue_bst_c2  = (~other_mem_op_e & ~stb_full_c2);
   assign bst_m = fst_m & blk_asi_m;

   assign bst_rs[5:0] = {rd[5:4], bst_cnt[2:0], 1'b0};

   // bst starts when bst hits w and is done when the 7th pckt has issued
   assign bst_issue_c1_next = ((bst_w & ~kill_w) | 
                               (bst_issue_c4 & ~(bst_cnt[2:0] == 3'b111) & ~bst_ce_c4 & 
                                ~(bst_ue_c4 & nceen)) | bst_issue_c6);

   // sotheas,9/14/04: fixed eco 6910, suppress block store start on dtlb perr
   //                  using registered version of lsu_ffu_st_dtlb_perr_g
   //   WAS:
   //        assign bst_issue_c2_next = ((bst_issue_c1 & ~(any_op_w2 & flush_w2)) | (bst_issue_c2 & ~can_issue_bst_c2)) & ~reset;
   //   IS:
   dff_s #1 st_dtlbperr_ff(.din (!lsu_ffu_st_dtlb_perr_g),
		               .q   (st_dtlb_perr_w2_l),
		               .clk (clk), .se(se), .si(), .so());
   assign bst_issue_c2_next = ((bst_issue_c1 & ~(any_op_w2 & flush_w2) & st_dtlb_perr_w2_l) 
                             | (bst_issue_c2 & ~can_issue_bst_c2)) & ~reset;
   assign bst_issue_c3_next = bst_issue_c2 & can_issue_bst_c2 & ~reset;
   assign bst_issue_c4_next = bst_issue_c3 & ~reset;
   assign bst_issue_c5_next = bst_issue_c4 & bst_ce_c4 & ~reset;
   assign bst_issue_c6_next = bst_issue_c5 & ~reset;

   // bst keeps reading in both c1 and c2 in case it stalls in c2
   assign bst_read_req = bst_issue_c1 | bst_issue_c2;
   // counter resets to 1 when bst hits w, increments when one is issued to lsu without ce
   assign bst_cnt_next[2:0] = (bst_w? 				3'b001:
                               (bst_issue_c4 & ~bst_ce_c4)?  (bst_cnt[2:0] + 3'b001):
                                               bst_cnt[2:0]);

   ///////////////////
   // bst starvation
   //----------------
   // when six bit counter saturates then a req to stall inst issue is made
   // The request stays high until a bst gets issued
   ///////////////////
   assign ffu_ifu_stallreq = bst_stall_req;
   assign bst_stall_req_next = ((bst_stall_cnt[5:0] == 6'b111111) & bst_issue_c2 & ~can_issue_bst_c2 |
                                bst_stall_req & other_mem_op_e);
   assign bst_stall_cnt_next[5:0] = (~bst_issue_c2)? 6'd0: bst_stall_cnt[5:0] + 6'd1;

   /////////////////////
   // bst ecc control
   /////////////////////
   // if a ce occurs even after correction then it is converted to a ue
   assign bst_ue_c4 = bst_issue_c4 & (previous_ue | (fixed_bst_ce & |previous_ce[1:0]));
   assign bst_ce_c4 = bst_issue_c4 & |previous_ce[1:0] & ~fixed_bst_ce & ~previous_ue;
   assign fixed_bst_ce_next = bst_ce_c4 | (fixed_bst_ce & ~bst_issue_c4);

   dff_s #(4) stb_full_dff(.din({lsu_ffu_stb_full0,lsu_ffu_stb_full1,lsu_ffu_stb_full2,lsu_ffu_stb_full3}),
                         .q({stb_full0, stb_full1, stb_full2, stb_full3}),
                         .clk(clk), .se(se), .si(), .so());
   dff_s blk_asi_dff(.din(lsu_ffu_blk_asi_e), .clk(clk), .q(blk_asi_m),
                   .se(se), .si(), .so());
   dffr_s bst_fix_ce_dff(.din(fixed_bst_ce_next), .clk(clk), .q(fixed_bst_ce), 
                       .se(se), .si(), .so(), .rst(reset));
   dff_s #(3) bst_cnt_dff(.din(bst_cnt_next[2:0]), .clk(clk), .q(bst_cnt[2:0]),
                        .se(se), .si(), .so());
   dff_s bst_m2w(.din(bst_m), .clk(clk), .q(bst_w), .se(se), .si(), .so());
   dff_s bst_issue_c1_dff(.din(bst_issue_c1_next), .clk(clk), .q(bst_issue_c1), .se(se),
                     .si(), .so());
   dff_s bst_issue_c2_dff(.din(bst_issue_c2_next), .clk(clk), .q(bst_issue_c2), .se(se),
                     .si(), .so());
   dff_s bst_issue_c3_dff(.din(bst_issue_c3_next), .clk(clk), .q(bst_issue_c3), .se(se),
                     .si(), .so());
   dff_s bst_issue_c4_dff(.din(bst_issue_c4_next), .clk(clk), .q(bst_issue_c4), .se(se),
                     .si(), .so());
   dff_s bst_issue_c5_dff(.din(bst_issue_c5_next), .clk(clk), .q(bst_issue_c5), .se(se),
                     .si(), .so());
   dff_s bst_issue_c6_dff(.din(bst_issue_c6_next), .clk(clk), .q(bst_issue_c6), .se(se),
                     .si(), .so());
   dff_s #(6) bst_stall_cntdff(.din(bst_stall_cnt_next[5:0]), .clk(clk), .q(bst_stall_cnt[5:0]),
                             .se(se), .si(), .so());
   dffr_s bst_stall_reqdff(.din(bst_stall_req_next), .clk(clk), .q(bst_stall_req),
                        .se(se), .si(), .so(), .rst(reset));
   
//----------------------------------------
// Decode Returning FPU/LSU packets
//----------------------------------------

   // FPU result pulled off of cpx
   assign is_fpu_result = (cpx_req_d1 == 4'b1000) ?
                           cpx_vld_d1 & fp_pending : 1'b0;
   assign ld_ret = lsu_ffu_ld_vld & ~(thr_match_fpw2 & flush_w2) & (blk_load_pending | load_pending);
	  
   // select frf write data
   // don't write data on Fcompares
   assign ctl_dp_rs2_sel_fpu_lsu = is_fpu_result & ~cpx_fcmp_d1 | ld_ret;
   assign ctl_dp_rs2_sel_vis = vis_result & ~ctl_dp_rs2_sel_fpu_lsu;
   assign ctl_dp_rs2_frf_read = (ren_rs2_w | ctl_dp_rd_ecc) & ~ctl_dp_rs2_sel_fpu_lsu & ~vis_result;
   assign ctl_dp_rs2_keep_data = ~(ren_rs2_w | ctl_dp_rd_ecc | vis_result | 
				   ctl_dp_rs2_sel_fpu_lsu);

   // selects for rs2 result mux
   assign ctl_dp_rd_ecc = fst_ce_w | rollback_rs2_w2 | bst_ce_c4 | rollback_rs1_w3;
   
   // Selects for rs1 mux
   assign ctl_dp_new_rs1 = ren_rs1_w2;

//----------------------------------------
// FRF Controls
//----------------------------------------   
   // WEN for frf from load, FPU result or mov
   assign external_wen_next = ld_ret & ~(ldfsr | ldxfsr) | (is_fpu_result & ~cpx_fcmp_d1 & ~take_ieee_trap);
   assign rf_ecc_gen_next = external_wen_next | vis_wen_next;
   
   dff_s rf_eccgen_dff(.din(rf_ecc_gen_next), .q(rf_ecc_gen), .clk(clk), .se(se), .si(), .so());
   dff_s rf_wen_dff(.din(rf_wen_next), .q(rf_wen), .clk(clk), .se(se), .si(), .so());
   // check for flush_pipe for moves
   assign rf_wen_next = rf_ecc_gen & ~(any_op_w2 & flush_w2) | move_wen_w2_valid;


   // REN and WEN must be mutually exclusive.  This works because WEN is always after W
   // if a new fpop has arrived to cancel it.
   // The even register is the upper half, odd is the lower half
   assign ctl_frf_wen[1] = ((rf_wen & ~rd[0]) | ecc_wen[1]) & ~ctl_frf_ren;   // double or even sgl
   assign ctl_frf_wen[0] = ((rf_wen & (~dest_single | rd[0])) | ecc_wen[0]) & ~ctl_frf_ren; // dbl or odd sgl

   // REN for frf  -- rd rs2 in D, rs1 in E
   assign read_rs2 = ren_rs2_e;
   assign read_rs1 = ren_rs1_m;
   assign read_rd = ifu_ffu_fst_d;
   assign read_bst = bst_read_req;
   // expanded out the terms for reading rs2 to help timing
   assign ctl_frf_ren = (read_rs1 | read_rs2 |
                         ifu_ffu_fst_d | bst_read_req);

   assign early_frf_rnum[5:1] = read_rs2?  rs2_next[5:1]:
                                read_rs1?  rs1[5:1]:
                                read_bst?  bst_rs[5:1]:
                                           write_addr[5:1];
   assign     st_rd_d[5:1] = {ifu_ffu_frd_d[0] & ~ifu_ffu_ldst_single_d, ifu_ffu_frd_d[4:1]};
   mux2ds #5 frf_rnum_mux(.dout(frf_rnum[5:1]),
                            .in0(early_frf_rnum[5:1]),
                            .in1(st_rd_d[5:1]),
                            .sel0(~read_rd),
                            .sel1(read_rd));

   assign frf_tid[1:0] = (read_rd)? ifu_ffu_tid_d[1:0]: tid_next[1:0];
                          
   assign wen_rs2_ecc = |ecc_wen[1:0] & ren_rs2_w4;
   assign wen_rs1_ecc = |ecc_wen[1:0] & ren_rs1_w5;
   assign blk_rd[5:1] = rd[5:1] + {2'b0, bld_cnt_d3[2:0]};
   assign write_addr[5:1] = wen_rs2_ecc?          rs2[5:1] :
                            wen_rs1_ecc?          rs1[5:1] :
                            bst_issue_c6?         bst_rs[5:1]:
                            blk_load_pending?     blk_rd[5:1]:
                                                  rd[5:1];

   // Address is combination of tid and rnum
   assign ctl_frf_addr[6:0] = {frf_tid[1:0], frf_rnum[5:1]}; 
  
//----------------------------------------
// Data from FPU forwarded to IFU
//----------------------------------------
   // Send thrid to IFU
   assign ffu_ifu_tid_w2[1:0] = tid;

   // completion is always signalled after the w-stage so that flush_pipe, etc.
   // can be checked.  For lds and fpops this is signalled after they write.
   // ecc_kill_rs2_w is checked for move and fst because the "compeletion"
   // is signalled over a separate interface so rollback can occur.
   // This is staged 2 cycles to allow for the cycle of ecc generation.
   //
   assign ffu_op_done_next = ((is_fpu_result & ~take_ieee_trap) | fld_done |
                              blk_ld_done | bst_done | vis_result |
                              fst_w & ~bst_w & ~kill_w);
   dff_s ffu_op_done_dff(.din(ffu_op_done_next), .clk(clk), .q(ffu_op_done),
                       .se(se), .si(), .so());
   // sotheas,9/14/04: fixed eco 6910, send done on dtlb perr for block store
   //                  using registered version of lsu_ffu_st_dtlb_perr_g
   //   WAS:
   //     assign ffu_op_done_vld = ffu_op_done & ~(any_op_w2 & flush_w2) | move_w2_vld;
   assign ffu_op_done_vld = (ffu_op_done | (bst_issue_c1 & !st_dtlb_perr_w2_l) ) 
                            & ~(any_op_w2 & flush_w2) | move_w2_vld;
   dff_s ffu_op_done2_dff(.din(ffu_op_done_vld), .clk(clk), .q(ffu_ifu_fpop_done_w2),
                        .se(se), .si(), .so());

//------------------------------------------
// FSR Controls
//------------------------------------------

   assign ctl_dp_fp_thr[0] = ~extra_tid[1] & ~extra_tid[0];
   assign ctl_dp_fp_thr[1] = ~extra_tid[1] &  extra_tid[0];
   assign ctl_dp_fp_thr[2] =  extra_tid[1] & ~extra_tid[0];
   assign ctl_dp_fp_thr[3] =  extra_tid[1] &  extra_tid[0];

   // CC's
   assign fcc_num_dec[0] = ~fcc_num[1] & ~fcc_num[0];
   assign fcc_num_dec[1] = ~fcc_num[1] & fcc_num[0];
   assign fcc_num_dec[2] = fcc_num[1] & ~fcc_num[0];
   assign fcc_num_dec[3] = fcc_num[1] & fcc_num[0];

   // selects to load next fsr from
   // stfsr or fmov always clear ftt
   assign clear_ftt = stfsr_w2_vld | move_w2_vld | is_fpu_result;
   
   assign ctl_dp_fsr_sel_fpu[3:0] = ({4{is_fpu_result | move_w2_vld | take_other_trap | stfsr_w2_vld}}
                                     & ctl_dp_fp_thr[3:0]);
   assign ctl_dp_fsr_sel_ld[3:0] =  ({4{ld_ret & (ldfsr | ldxfsr)}} &  
                                     ~ctl_dp_fsr_sel_fpu[3:0] & ctl_dp_fp_thr[3:0]);
   assign ctl_dp_fsr_sel_old[3:0] =  (~ctl_dp_fsr_sel_fpu[3:0] & ~ctl_dp_fsr_sel_ld[3:0]);
   
   // align fcc depending on which fcc_num was used
   mux4ds #8 fcc_ret_mux(.dout (fpu_fcc[7:0]),
			 .in0  ({dp_ctl_fsr_fcc[7:2], cpx_fccval_d1[1:0]}),
			 .in1  ({dp_ctl_fsr_fcc[7:4], cpx_fccval_d1[1:0], dp_ctl_fsr_fcc[1:0]}),
			 .in2  ({dp_ctl_fsr_fcc[7:6], cpx_fccval_d1[1:0], dp_ctl_fsr_fcc[3:0]}),
			 .in3  ({cpx_fccval_d1[1:0], dp_ctl_fsr_fcc[5:0]}),
			 .sel0 (fcc_num_dec[0]),
			 .sel1 (fcc_num_dec[1]),
			 .sel2 (fcc_num_dec[2]),
			 .sel3 (fcc_num_dec[3]));

   // set fcc if this was an fcmp instruction
   assign fcc_sel_fpu = cpx_fcmp_d1 & ~ieee_trap & is_fpu_result;
   assign fcc_sel_ld = ~is_fpu_result & ldfsr_vld;
   assign fcc_sel_ldx = ~is_fpu_result & ~ldfsr_vld & ldxfsr_vld;
   assign fcc_sel_old = ~fcc_sel_fpu & ~fcc_sel_ld & ~fcc_sel_ldx;
   mux4ds #8 fcc_set_mux(.dout (ctl_dp_fcc_w2[7:0]),
			                   .in0  (dp_ctl_fsr_fcc[7:0]),
			                   .in1  (fpu_fcc[7:0]),
			                   .in2  (dp_ctl_ld_fcc[7:0]),
			                   .in3  ({dp_ctl_fsr_fcc[7:2], dp_ctl_ld_fcc[1:0]}),
			                   .sel0 (fcc_sel_old),
			                   .sel1 (fcc_sel_fpu),
			                   .sel2 (fcc_sel_ldx),
			                   .sel3 (fcc_sel_ld));

   // get fcc's from ldfsr instruction
   assign ldfsr_vld = ldfsr & load_pending;
   assign ldxfsr_vld = ldxfsr & load_pending;

   wire   cc_changed_w2;

   // fcc set by fcmp, ldfsr or ldxfsr
   assign cc_changed = fcc_sel_fpu |
	                     fld_done & (ldfsr_vld | ldxfsr_vld);

   dff_s cc_changed_dff(.din(cc_changed), .clk(clk), .q(cc_changed_w2),
                      .se(se), .si(), .so());
   dff_s #(8) cc_next(.din(ctl_dp_fcc_w2[7:0]), .clk(clk), .q(ffu_ifu_cc_w2[7:0]),
                    .se(se), .si(), .so());

   assign ffu_ifu_cc_vld_w2[3:0] = ctl_dp_fp_thr[3:0] & {4{cc_changed_w2}};
   
//-----------------------------------
// Traps
//-----------------------------------   

   // illegal instruction if blk ld/st is not 8 dp regs aligned
   dff_s illegal_vis_e2m (.din(illegal_vis_e), .clk(clk), .q(illegal_vis_m),
                        .se(se), .si(), .so());
   assign illegal_blk_m = (blk_ld_m | bst_m) & (rd[0] | rd[1] | rd[2] | rd[3]) & ~dest_single;
   assign ffu_tlu_ill_inst_m = illegal_blk_m | illegal_vis_m;
   
   assign ctl_dp_ftt_w2 = take_ieee_trap      ? 3'b001 :
	                  unimpl_qual_w2        ? 3'b011 :
	                  clear_ftt           ? 3'b000 :
	                                        3'bxxx; 
   
// SPARC V9 Underflow, Overflow, Inexact behavior: 
//
//      Exception(s)             |            Current
//      Detected    Trap Enable  | fp_        Exception
//      in f.p.     Mask Bits    | exception_ Bits (in
//      operation   (in FSR.TEM) | ieee_754   FSR.cexc)   
//      ----------- ------------ | Trap       ----------- 
//      of  uf  nx  OFM UFM NXM  | Occurs?    ofc ufc nxc  Notes
//      --- --- --- --- --- ---  | ---------- --- --- ---  -----
//       -   -   -   x   x   x   |  no         0   0   0   
//       -   -   *   x   x   0   |  no         0   0   1 
//       -   *   *   x   0   0   |  no         0   1   1
//       *   -   *   0   x   0   |  no         1   0   1    (2)
//                               | 
//       -   -   *   x   x   1   |  yes        0   0   1 
//       -   *   *   x   0   1   |  yes        0   0   1 
//       -   *   -   x   1   x   |  yes        0   1   0
//       -   *   *   x   1   x   |  yes        0   1   0 
//       *   -   *   1   x   x   |  yes        1   0   0    (2)
//       *   -   *   0   x   1   |  yes        0   0   1    (2)
//
//      (2) Overflow is always accompanied by inexact.
//
//
// The FPU does not receive FSR.TEM bits. FSR.TEM bits are used within
// the FFU for the following cases:
// (1) fp_exception_ieee_754 trap detection
//     If a FPop generates an IEEE exception (nv, of, uf, dz, nx) for
//     which the corresponding trap enable (TEM) is set, then a
//     fp_exception_ieee_754 trap is caused. FSR.cexc field has one bit
//     set corresponding to the IEEE exception, and FSR.aexc field is
//     unchanged.
// (2) Clear FSR.nxc if an overflow (underflow) exception does trap
//     because FSR.OFM (FSR.UFM) is set, regardless of whether FSR.NXM
//     is set. Set FSR.ofc (FSR.ufc).
// (3) Clear FSR.ofc (FSR.ufc) if overflow (underflow) exception traps
//     and FSR.OFM (FSR.UFM) is not set and FSR.NXM is set. Set FSR.nxc.
// (4) Clear FSR.ufc if the result is exact (FSR.nxc is not set) and
//     FSR.UFM is not set. This case represents an exact denormalized
//     result.
//
// Note: - FPU will signal underflow to the FFU for all "tiny" results.
//       - FPU always reports inexact along with overflow.
   dff_s #(5) tem_dff(.din(dp_ctl_fsr_tem[4:0]), .clk(clk), .q(fsr_tem_d1),
                    .se(se), .si(), .so());
   assign fpexc_nxc =
          cpx_fpexc_d1[0] &
          ~(( fsr_tem_d1[3] & cpx_fpexc_d1[3]) |    // enabled  of
            ( fsr_tem_d1[2] & cpx_fpexc_d1[2])  );  // enabled  uf

   assign fpexc_ofc =
          cpx_fpexc_d1[3] &
          ~(~fsr_tem_d1[3] & fsr_tem_d1[0] & cpx_fpexc_d1[0]); // disabled of and enabled nx

   assign fpexc_ufc =
          cpx_fpexc_d1[2] &
          ~(~fsr_tem_d1[2] & ( fsr_tem_d1[0] & cpx_fpexc_d1[0])) & // disabled uf & enabled nx
          ~(~fsr_tem_d1[2] & ~cpx_fpexc_d1[0]) ; // disabled uf with no inexact
                                                               // (i.e. exact denorm w/ UFM=0)


   assign ieee_trap_vec[4:0] =
          ({cpx_fpexc_d1[4],
            fpexc_ofc,
            fpexc_ufc,
            cpx_fpexc_d1[1],
            fpexc_nxc       } & fsr_tem_d1[4:0]);


   // ieee trap has least priority.  Put through a flop for timing reasons
   assign ieee_trap = (|ieee_trap_vec[4:0]);
   assign take_ieee_trap = ieee_trap & is_fpu_result;
   dff_s trap_ieee_dff(.din(take_ieee_trap), .clk(clk), .q(ffu_tlu_trap_ieee754),
                     .se(se), .si(), .so());

   assign take_other_trap = unimpl_qual_w2;
   assign ffu_tlu_trap_other = take_other_trap;
   assign ffu_tlu_trap_ue = ue_trap_w3;
   
   // current exception
   assign fp_exc_w2[4:0] =
          ({cpx_fpexc_d1[4],
            fpexc_ofc,
            fpexc_ufc,
            cpx_fpexc_d1[1],
            fpexc_nxc       });



   
   // accrued exceptions
   // fp_exc_w2 = dp_ctl_fsr_aexc | cpx_fpexc_d1 & {5{~take_ieee_trap}};
   assign fp_exc_w2[9] = dp_ctl_fsr_aexc[4] | 
	                     is_fpu_result & cpx_fpexc_d1[4] & ~fsr_tem_d1[4];
   assign fp_exc_w2[6] = dp_ctl_fsr_aexc[1] | 
	                     is_fpu_result & cpx_fpexc_d1[1] & ~fsr_tem_d1[1];
   assign fp_exc_w2[7] = dp_ctl_fsr_aexc[2] | 
	                     is_fpu_result & fpexc_ufc & ~fsr_tem_d1[2]; 
   assign fp_exc_w2[8] = dp_ctl_fsr_aexc[3] | 
	                     is_fpu_result & fpexc_ofc & ~fsr_tem_d1[3];
   assign fp_exc_w2[5] = dp_ctl_fsr_aexc[0] |
	                     is_fpu_result & fpexc_nxc & ~fsr_tem_d1[0];
   
   assign ctl_dp_exc_w2[9:5] = fp_exc_w2[9:5];
   // move, abs, etc will clear cexc, fpu_results will update, all else will leave unchanged
   wire   update_cexc;
   assign update_cexc = is_fpu_result | move_w2_vld;
   assign ctl_dp_exc_w2[4:0] = ((update_cexc)? fp_exc_w2[4:0] & {5{is_fpu_result}}:
                                dp_ctl_fsr_cexc[4:0]);



   ////////////////////////////////
   // ECC control
   ////////////////////////////////
   // Generation of the parity bit for writes
   wire [13:0] gen_synd_d1;
   wire        gen_par_hi;
   wire        gen_par_low;
   wire [6:0]  error_inj_data;
   output [13:0] ctl_frf_write_synd;
   dff_s #(14) gen_synd_dff (.din({dp_ctl_synd_out_high[6:0],dp_ctl_synd_out_low[6:0]}),
                           .q(gen_synd_d1[13:0]), .clk(clk), .se(se), .si(), .so());
   assign  gen_par_hi = ^gen_synd_d1[13:7];
   assign  gen_par_low = ^gen_synd_d1[6:0];
   assign  ctl_frf_write_synd[13:0] = ({gen_par_hi,gen_synd_d1[12:7],gen_par_low,gen_synd_d1[5:0]} ^ 
                                       {error_inj_data[6:0],error_inj_data[6:0]});
   /////////////////////////////////
   // error injection
   /////////////////////////////////
   // injection doesn't check for flush on wen
   assign  inject_err_next = ifu_ffu_inj_frferr & rf_wen_next;
   assign error_inj_data[6:0] = {7{inject_err}} & err_data[6:0];
   dff_s #(7) err_data_dff(.din(ifu_exu_ecc_mask[6:0]),
                     .q(err_data[6:0]),
                     .clk(clk), .se(se), .si(), .so());
   dff_s err_cntl(.din(inject_err_next),
                .q(inject_err),
                .clk(clk), .se(se), .si(), .so());
   // speculate on error injection (don't check flush_pipe etc)
   assign ffu_ifu_inj_ack = inject_err;


   // check the upper half on a double or a single with an even reg num
   // check the lower half on a double or a single with an odd reg num
   // ecc block will run on frf input for reads
   // otherwise it will run on the rd_data
   assign check_ecc_next = ren_rs2_m | fst_e | ren_rs1_w | bst_issue_c3_next;
   dff_s check_ecc_dff(.din(check_ecc_next), .clk(clk), .q(ctl_dp_ecc_sel_frf),
                     .se(se), .si(), .so());
   // rs1 will not be checked if a ce was detected on rs2.  If there was a ue on rs2
   // rs1 will be checked and a ce will be corrected, but the error on rs2 will be logged
   assign chk_rs1_w2 = ren_rs1_w2;
   assign chk_ecc_m[1] = fst_m & ~rd[0] & ~output_sel_fsr;
   assign chk_ecc_m[0] = fst_m & (~dest_single | rd[0]) & ~output_sel_fsr;
   assign chk_ecc_w[1] = ren_rs2_w & ~kill_eccchk_w & ~rs2[0];
   assign chk_ecc_w[0] = ren_rs2_w & ~kill_eccchk_w & (~source_single | rs2[0]);
   assign chk_ecc_w2[1] = chk_rs1_w2 & ~rs1[0];
   assign chk_ecc_w2[0] = chk_rs1_w2 & (~source_single | rs1[0]);

   assign chk_ecc[1:0] = chk_ecc_m[1:0] | chk_ecc_w[1:0] | chk_ecc_w2[1:0] | {2{bst_issue_c3}};
   dff_s #(2) chk_ecc_dff(.din(chk_ecc[1:0]), .clk(clk), .q(chk_ecc_prev[1:0]),
                        .se(se), .si(), .so());

   assign     error_detected[1] = |dp_ctl_synd_out_high[5:0];
   assign     error_detected[0] = |dp_ctl_synd_out_low[5:0];
   
   assign     possible_ue[1] = ~dp_ctl_synd_out_high[6] & chk_ecc[1];
   assign     possible_ue[0] = ~dp_ctl_synd_out_low[6] & chk_ecc[0];
   assign     ce[1] = dp_ctl_synd_out_high[6] & chk_ecc[1];
   assign     ce[0] = dp_ctl_synd_out_low[6] & chk_ecc[0];

   assign rollback_fst_m = ((dp_ctl_synd_out_high[6] & chk_ecc_m[1] & ~disable_ce_m) |
                        (dp_ctl_synd_out_low[6] & chk_ecc_m[0] & ~disable_ce_m));
   dff_s rollback_m2w(.din(rollback_fst_m), .clk(clk), .q(rollback_fst_w), .se(se), .si(), .so());
   dff_s #(2) possible_ue_dff(.din(possible_ue[1:0]), .clk(clk), .q(prev_poss_ue[1:0]),
                            .se(se), .si(), .so());
   dff_s #(2) ce_dff(.din(ce[1:0]), .clk(clk), .q(previous_ce[1:0]),
                            .se(se), .si(), .so());
   dff_s #(2) err_det_dff(.din(error_detected[1:0]), .clk(clk), .q(prev_err_detected[1:0]),
                            .se(se), .si(), .so());
   assign previous_ue = |(prev_err_detected[1:0] & prev_poss_ue[1:0]);
   
   dff_s #(2) ecc_wen1_dff(.din(ecc_wen_gen_next[1:0]), .clk(clk), .q(ecc_wen_gen[1:0]),
                        .se(se), .si(), .so());
   dff_s #(2) ecc_wen2_dff(.din(ecc_wen_next[1:0]), .clk(clk), .q(ecc_wen[1:0]),
                        .se(se), .si(), .so());
   // if the ecc error was in the m stage we need to check for a kill
   // if the ecc error was in the w stage we need to check flush
   // ECC errors on rs1 will not be written back to the frf.  The data that is used will be corrected.
   assign ecc_wen_gen_next[1:0] = previous_ce[1:0] & 
          {2{bst_issue_c4 | fst_ce_w | rollback_rs2_w2 | rollback_rs1_w3}};
   assign ecc_wen_next = ecc_wen_gen[1:0] & {2{~(fst_ce_w2 & flush_w2)}};
   
   // pass along ce and ue so trap can be signalled to ffu and tlu
   // if disable_ce_m then don't tell ifu reissue ce.  Instead convert to a ue.
   dff_s disable_ce_e2m(.din(ifu_exu_disable_ce_e), .clk(clk), .q(disable_ce_m),
                      .se(se), .si(), .so());
   dff_s disable_ce_m2w(.din(disable_ce_m), .clk(clk), .q(disable_ce_w),
                      .se(se), .si(), .so());
   assign fst_ce_w = rollback_fst_w & ~kill_eccchk_w;
   assign fst_ue_w = fst_w & (previous_ue | (disable_ce_w & |(previous_ce[1:0])))  & ~kill_eccchk_w;
   assign rollback_rs2_w2 = (ren_rs2_w2 & ~flush_w2 & |previous_ce[1:0] 
                             & ~rolled_back);
   assign rs2_ce_w2 = ren_rs2_w2 & |previous_ce[1:0] & ~rolled_back & ~previous_ue;
   assign rs2_ue_w2 = ren_rs2_w2 & (previous_ue | (rolled_back & |previous_ce[1:0]));
   // must check for flush because eccchk doesn't do this
   assign rs2_fst_ce_w2_vld = (rs2_ce_w2 | fst_ce_w2) & ~flush_w2;
   assign rs2_fst_ue_w2_vld = (rs2_ue_w2 | fst_ue_w2) & ~flush_w2;
   dff_s ce_w2w2(.din(fst_ce_w), .clk(clk), .q(fst_ce_w2),
               .se(se), .si(), .so());
   dff_s ue_w2w2(.din(fst_ue_w), .clk(clk), .q(fst_ue_w2),
               .se(se), .si(), .so());
   dff_s ce_w22w3(.din(rs2_fst_ce_w2_vld), .clk(clk), .q(rs2_fst_ce_w3),
               .se(se), .si(), .so());
   dff_s ue_w22w3(.din(rs2_fst_ue_w2_vld), .clk(clk), .q(rs2_fst_ue_w3),
               .se(se), .si(), .so());

   assign rs1_ce_w3 = ren_rs1_w3 & |previous_ce[1:0] & ~previous_ue & ~rolled_back;
   assign rollback_rs1_w3 = rs1_ce_w3 & ~ue_trap_w3;
   assign ce_w3 = (rs1_ce_w3 | rs2_fst_ce_w3);
   assign ue_w3 = (ren_rs1_w3 & (previous_ue | (rolled_back & |previous_ce[1:0]))) | rs2_fst_ue_w3;

   assign ffu_ifu_ecc_ce_w2 = (ce_w3 | bst_ce_c4);
   assign ffu_ifu_ecc_ue_w2 = (ue_w3 | bst_ue_c4);
   
   // error logging signals.  The error register priority is fst, bst, rs1_ue, rs2_ue, rs1_ce, rs2_ce
   assign log_new_err = (ren_rs2_w2 | bst_issue_c4 | fst_w | 
                         (ren_rs1_w3 & previous_ue) | (rs1_ce_w3 & ~rs2_fst_ue_w3));
   assign new_err_reg[5:0] = fst_w ?        rd[5:0]:
                             bst_issue_c4 ? bst_rs[5:0]:
                             ren_rs2_w2 ?    rs2[5:0]:
                                            rs1[5:0];
   assign err_reg_next[5:0] = (log_new_err) ? new_err_reg[5:0] : err_reg_d1[5:0];
   dff_s #(6) err_reg_dff(.din(err_reg_next[5:0]), .clk(clk), .q(err_reg_d1[5:0]),
                        .se(se), .si(), .so());
   assign ffu_ifu_err_reg_w2[5:0] = err_reg_d1[5:0];
   
   // storage of error syndrome for logging
   // For singles the invalid half of the syndrome is zeroed out.
   // The syndrome reported to the ifu will be latched until a new error is detected
   assign      new_err_synd[13:7] = gen_synd_d1[13:7] & {7{chk_ecc_prev[1]}};
   assign      new_err_synd[6:0] = gen_synd_d1[6:0] & {7{chk_ecc_prev[0]}};
   assign      err_synd_next[13:0] = (log_new_err)? new_err_synd: err_synd_d1;
   dff_s #(14) err_synd_d1ff(.din(err_synd_next[13:0]), .clk(clk), .q(err_synd_d1[13:0]),
                          .se(se), .si(), .so());
   assign      ffu_ifu_err_synd_w2[13:0] = err_synd_d1[13:0];

   // kill moves and fpu ops
   assign ecc_kill_rs2_w2 = rollback_rs2_w2 | (rs2_ue_w2 & nceen);
   

   // pipe along enable signal for ue traps
   assign nceen_next = (any_op_e)? ifu_exu_nceen_e: (nceen & ~rollback_fst_w);
   dff_s nceen_dff(.din(nceen_next), .clk(clk), .q(nceen),
                 .se(se), .si(), .so());
   assign ue_trap_w3 = (ue_w3 | bst_ue_c4) & nceen;

   // signals for killing stores on ecc
   // use this to kill any rs2/rd ce   
   assign ffu_ifu_fst_ce_w = rollback_fst_w;

   // These signals kill the entry in the store buffer for any ce or trapping ue.  Very critical timing.
   assign possible_kill_st_ce_m = ((fst_m & ~output_sel_fsr & ~(disable_ce_m & ~nceen)) |
                                   (bst_issue_c3 & ~(fixed_bst_ce & ~nceen)));
   assign possible_kill_st_ue_m = (fst_m & ~output_sel_fsr & nceen | bst_issue_c3 & nceen);
   assign kill_st_ce_w = (|previous_ce[1:0]) & possible_kill_st_ce_w;
   assign ffu_lsu_kill_fst_w = (previous_ue)? possible_kill_st_ue_w: kill_st_ce_w;
   dff_s kill_fst_ce_dff(.din(possible_kill_st_ce_m), .clk(clk), .q(possible_kill_st_ce_w),
                       .se(se), .si(), .so());
   dff_s kill_fst_ue_dff(.din(possible_kill_st_ue_m), .clk(clk), .q(possible_kill_st_ue_w),
                       .se(se), .si(), .so());

   // rollback signals
   assign rollback_c1_next = rollback_rs2_w2 | rollback_rs1_w3;
   dffr_s #(3) rollback_dff(.din({rollback_c1_next,rollback_c1_vld,rollback_c2}),
                         .q({rollback_c1, rollback_c2, rollback_c3}),
                         .clk(clk), .se(se), .si(), .so(), .rst(reset));
   // if both rs1 and rs2 rollback then the state machine needs to start on rs1
   assign rollback_c1_vld = rollback_c1 & ~ue_trap_w3 & ~rollback_rs1_w3;

   assign rolled_back_next = rollback_c1_vld | rolled_back & ~any_op_e;
   dffr_s rollback_state(.din(rolled_back_next), .q(rolled_back),
                        .rst(reset), .clk(clk),
                        .se(se), .si(), .so());
   
   //////////////////////////////
   // Performance counter signals
   //////////////////////////////
   assign ffu_tlu_fpu_tid[1:0] = tid[1:0] & {2{is_fpu_result}}; // don't toggle wire if not needed
   assign ffu_tlu_fpu_cmplt = is_fpu_result;

   sparc_ffu_ctl_visctl visctl(
                               .illegal_vis_e(illegal_vis_e),                               
                               .vis_nofrf_e(vis_nofrf_e),
                               .visop_e (visop_e),
                               .visop_m (visop_m),
                               .visop_w_vld (visop_w_vld),
                               .vis_wen_next (vis_wen_next),
                               .ifu_ffu_rnd_e(frs2_e[2:0]),
                               .fpu_rnd (fpu_rnd[1:0]),
                               .vis_result(vis_result),
                               /*AUTOINST*/
                               // Outputs
                               .ctl_vis_sel_add(ctl_vis_sel_add),
                               .ctl_vis_sel_log(ctl_vis_sel_log),
                               .ctl_vis_sel_align(ctl_vis_sel_align),
                               .ctl_vis_add32(ctl_vis_add32),
                               .ctl_vis_subtract(ctl_vis_subtract),
                               .ctl_vis_cin(ctl_vis_cin),
                               .ctl_vis_align0(ctl_vis_align0),
                               .ctl_vis_align2(ctl_vis_align2),
                               .ctl_vis_align4(ctl_vis_align4),
                               .ctl_vis_align6(ctl_vis_align6),
                               .ctl_vis_align_odd(ctl_vis_align_odd),
                               .ctl_vis_log_sel_pass(ctl_vis_log_sel_pass),
                               .ctl_vis_log_sel_nand(ctl_vis_log_sel_nand),
                               .ctl_vis_log_sel_nor(ctl_vis_log_sel_nor),
                               .ctl_vis_log_sel_xor(ctl_vis_log_sel_xor),
                               .ctl_vis_log_invert_rs1(ctl_vis_log_invert_rs1),
                               .ctl_vis_log_invert_rs2(ctl_vis_log_invert_rs2),
                               .ctl_vis_log_constant(ctl_vis_log_constant),
                               .ctl_vis_log_pass_const(ctl_vis_log_pass_const),
                               .ctl_vis_log_pass_rs1(ctl_vis_log_pass_rs1),
                               .ctl_vis_log_pass_rs2(ctl_vis_log_pass_rs2),
                               .ffu_exu_rsr_data_hi_m(ffu_exu_rsr_data_hi_m[31:0]),
                               .ffu_exu_rsr_data_mid_m(ffu_exu_rsr_data_mid_m[2:0]),
                               .ffu_exu_rsr_data_lo_m(ffu_exu_rsr_data_lo_m[7:0]),
                               .ctl_dp_wsr_data_w2(ctl_dp_wsr_data_w2[36:0]),
                               .ctl_dp_gsr_wsr_w2(ctl_dp_gsr_wsr_w2[3:0]),
                               .ctl_dp_thr_e(ctl_dp_thr_e[3:0]),
                               // Inputs
                               .clk     (clk),
                               .se      (se),
                               .reset   (reset),
                               .opf     (opf[8:0]),
                               .tid_w2  (tid_w2[1:0]),
                               .tid_e   (tid_e[1:0]),
                               .tid     (tid[1:0]),
                               .kill_w  (kill_w),
                               .ifu_tlu_sraddr_d(ifu_tlu_sraddr_d[6:0]),
                               .exu_ffu_wsr_inst_e(exu_ffu_wsr_inst_e),
                               .exu_ffu_gsr_align_m(exu_ffu_gsr_align_m[2:0]),
                               .exu_ffu_gsr_rnd_m(exu_ffu_gsr_rnd_m[2:0]),
                               .exu_ffu_gsr_mask_m(exu_ffu_gsr_mask_m[31:0]),
                               .exu_ffu_gsr_scale_m(exu_ffu_gsr_scale_m[4:0]),
                               .dp_ctl_fsr_rnd(dp_ctl_fsr_rnd[1:0]),
                               .flush_w2(flush_w2),
                               .thr_match_mw2(thr_match_mw2),
                               .thr_match_ww2(thr_match_ww2),
                               .ifu_tlu_inst_vld_w(ifu_tlu_inst_vld_w),
                               .ue_trap_w3(ue_trap_w3),
                               .frs1_e  (frs1_e[4:0]),
                               .frs2_e  (frs2_e[4:0]),
                               .frd_e   (frd_e[4:0]),
                               .rollback_c3(rollback_c3),
                               .rollback_rs2_w2(rollback_rs2_w2),
                               .visop   (visop),
                               .rollback_rs1_w3(rollback_rs1_w3),
                               .dp_ctl_gsr_mask_e(dp_ctl_gsr_mask_e[31:0]),
                               .dp_ctl_gsr_scale_e(dp_ctl_gsr_scale_e[4:0]));

endmodule // sparc_ffu_ctl
// Modified by Princeton University on June 9th, 2015
// ========== Copyright Header Begin ==========================================
// 
// OpenSPARC T1 Processor File: sparc_ffu_ctl_visctl.v
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
//  Module Name: sparc_ffu_ctl_visctl
//	Description: This is the ffu vis control block.
*/
module sparc_ffu_ctl_visctl (/*AUTOARG*/
   // Outputs
   ctl_vis_sel_add, ctl_vis_sel_log, ctl_vis_sel_align, 
   ctl_vis_add32, ctl_vis_subtract, ctl_vis_cin, ctl_vis_align0, 
   ctl_vis_align2, ctl_vis_align4, ctl_vis_align6, ctl_vis_align_odd, 
   ctl_vis_log_sel_pass, ctl_vis_log_sel_nand, ctl_vis_log_sel_nor, 
   ctl_vis_log_sel_xor, ctl_vis_log_invert_rs1, 
   ctl_vis_log_invert_rs2, ctl_vis_log_constant, 
   ctl_vis_log_pass_const, ctl_vis_log_pass_rs1, 
   ctl_vis_log_pass_rs2, vis_result, illegal_vis_e, vis_nofrf_e, 
   visop_m, visop_w_vld, vis_wen_next, fpu_rnd, 
   ffu_exu_rsr_data_hi_m, ffu_exu_rsr_data_mid_m, 
   ffu_exu_rsr_data_lo_m, ctl_dp_wsr_data_w2, ctl_dp_gsr_wsr_w2, 
   ctl_dp_thr_e, 
   // Inputs
   clk, se, reset, opf, tid_w2, tid_e, tid, visop_e, kill_w, 
   ifu_tlu_sraddr_d, exu_ffu_wsr_inst_e, exu_ffu_gsr_align_m, 
   exu_ffu_gsr_rnd_m, exu_ffu_gsr_mask_m, exu_ffu_gsr_scale_m, 
   ifu_ffu_rnd_e, dp_ctl_fsr_rnd, flush_w2, thr_match_mw2, 
   thr_match_ww2, ifu_tlu_inst_vld_w, ue_trap_w3, frs1_e, frs2_e, 
   frd_e, rollback_c3, rollback_rs2_w2, visop, rollback_rs1_w3, 
   dp_ctl_gsr_mask_e, dp_ctl_gsr_scale_e
   ) ;
   input clk;
   input se;
   input reset;
   input [8:0] opf;
   input [1:0] tid_w2;
   input [1:0] tid_e;
   input [1:0] tid;
   input      visop_e;
   input       kill_w;
   input [6:0] ifu_tlu_sraddr_d; 
   input       exu_ffu_wsr_inst_e;
   input [2:0] exu_ffu_gsr_align_m;
   input [2:0] exu_ffu_gsr_rnd_m;
   input [31:0] exu_ffu_gsr_mask_m;
   input [4:0]  exu_ffu_gsr_scale_m;
   input [2:0] ifu_ffu_rnd_e;
   input [1:0] dp_ctl_fsr_rnd;
   input      flush_w2;
   input      thr_match_mw2;
   input      thr_match_ww2;
   input      ifu_tlu_inst_vld_w;
   input      ue_trap_w3;
   input [4:0] frs1_e;
   input [4:0] frs2_e;
   input [4:0] frd_e;
   input       rollback_c3;
   input       rollback_rs2_w2;
   input       visop;
   input       rollback_rs1_w3;
   input [31:0] dp_ctl_gsr_mask_e;
   input [4:0]  dp_ctl_gsr_scale_e;
   
   output      ctl_vis_sel_add;
   output      ctl_vis_sel_log;
   output      ctl_vis_sel_align;
   output      ctl_vis_add32;
   output      ctl_vis_subtract;
   output      ctl_vis_cin;
   output      ctl_vis_align0;
   output      ctl_vis_align2;
   output      ctl_vis_align4;
   output      ctl_vis_align6;
   output      ctl_vis_align_odd;
   output      ctl_vis_log_sel_pass;
   output      ctl_vis_log_sel_nand;
   output      ctl_vis_log_sel_nor;
   output      ctl_vis_log_sel_xor;
   output      ctl_vis_log_invert_rs1;
   output      ctl_vis_log_invert_rs2;
   output      ctl_vis_log_constant;
   output      ctl_vis_log_pass_const;
   output      ctl_vis_log_pass_rs1;
   output      ctl_vis_log_pass_rs2;
   output      vis_result;
   output      illegal_vis_e;
   output      vis_nofrf_e;
   output      visop_m;
   output      visop_w_vld;
   output      vis_wen_next;
   output [1:0] fpu_rnd;
   output [31:0] ffu_exu_rsr_data_hi_m;
   output [2:0]  ffu_exu_rsr_data_mid_m;
   output [7:0] ffu_exu_rsr_data_lo_m;

   output [36:0] ctl_dp_wsr_data_w2;
   output [3:0] ctl_dp_gsr_wsr_w2;
   output [3:0] ctl_dp_thr_e;
   
   wire         illegal_rs1_e;
   wire         illegal_rs2_e;
   wire         illegal_siam_e;
   wire         rs2_check_nonzero_e;
   wire         rs1_check_nonzero_e;
   wire        visop_e;
   wire        issue_visop_e;
   wire        visop_m;
   wire        visop_w;
   wire        visop_w_vld;
   wire        visop_w2_vld;
   wire        visop_w2;
   wire        visop_w3;
   wire        visop_w3_vld;
   wire        add;
   wire        align;
   wire        wlogic;
   wire        siam;
   wire        alignaddr;

   wire        opf_log_zero;
   wire        opf_log_one;
   wire        opf_log_src1;
   wire        opf_log_src2;
   wire        opf_log_not1;
   wire        opf_log_not2;
   wire        opf_log_or;
   wire        opf_log_nor;
   wire        opf_log_and;
   wire        opf_log_nand;
   wire        opf_log_xor;
   wire        opf_log_xnor;
   wire        opf_log_ornot1;
   wire        opf_log_ornot2;
   wire        opf_log_andnot1;
   wire        opf_log_andnot2;
   wire        invert_rs1_next;
   wire        invert_rs2_next;
   wire        log_pass_rs1_next;
   wire        log_pass_rs2_next;
   wire        log_pass_rs1;
   wire        log_pass_rs2;
   
   wire [2:0] t0_gsr_rnd;
   wire [2:0] t1_gsr_rnd;
   wire [2:0] t2_gsr_rnd;
   wire [2:0] t3_gsr_rnd;
   wire [2:0] t0_gsr_align;
   wire [2:0] t1_gsr_align;
   wire [2:0] t2_gsr_align;
   wire [2:0] t3_gsr_align;
   wire [2:0] t0_gsr_rnd_next;
   wire [2:0] t1_gsr_rnd_next;
   wire [2:0] t2_gsr_rnd_next;
   wire [2:0] t3_gsr_rnd_next;
   wire [2:0] t0_gsr_align_next;
   wire [2:0] t1_gsr_align_next;
   wire [2:0] t2_gsr_align_next;
   wire [2:0] t3_gsr_align_next;
   wire [2:0] gsr_rnd_e;
   wire [2:0] gsr_align_e;
   wire       t0_rnd_wen_l;
   wire       t0_gsr_wsr_w2;
   wire       t0_siam_w2;
   wire       t0_align_wen_l;
   wire       t0_alignaddr_w2;
   wire       t1_rnd_wen_l;
   wire       t1_gsr_wsr_w2;
   wire       t1_siam_w2;
   wire       t1_align_wen_l;
   wire       t1_alignaddr_w2;
   wire       t2_rnd_wen_l;
   wire       t2_gsr_wsr_w2;
   wire       t2_siam_w2;
   wire       t2_align_wen_l;
   wire       t2_alignaddr_w2;
   wire       t3_rnd_wen_l;
   wire       t3_gsr_wsr_w2;
   wire       t3_siam_w2;
   wire       t3_align_wen_l;
   wire       t3_alignaddr_w2;

   wire [2:0] siam_rnd;
   wire [3:0] thr_w2;
   wire [3:0] ctl_dp_thr_e;
   wire [3:0] thr_fp;
   wire       gsr_addr_d;
   wire       gsr_addr_e;
   wire       wgsr_e;
   wire       wgsr_m;
   wire       wgsr_w;
   wire       wgsr_vld_m;
   wire       wgsr_vld_w;
   wire       wgsr_vld_w2;
   wire       wgsr_w2;
   wire [2:0] gsr_rnd;
   wire [1:0] fpu_rnd_next;
   wire [2:0]  gsr_align;
   wire [2:0]  gsr_align_d1;

   wire [2:0] align_addr_data_w2;
   wire [2:0] wgsr_align_offset_w;
   wire [2:0] wgsr_rnd_w;
   wire [2:0] wgsr_align_offset_w2;
   wire [2:0] wgsr_rnd_w2;

   wire [36:0] wsr_data_m;
   wire [36:0] wsr_data_w;


   //////////////////////////////////////
   // VIS PIPELINE
   //------------------------------------
   // Note: rs2_ce, rs2_ue, rs1_ue will kill vis instruction
   //       in addition to any traps, etc.
   //       These are incorporated into the "kill" signals
   // E: ren rs2
   // M: ren rs1
   // W: rs2 data ready, check rs2 ecc
   // W2: rs1 data ready, check rs1 ecc
   // W3: execute vis operation (result written to rs2/rd flop)
   // W4: gen ecc and write to frf
   
   dff_s visop_e2m(.din(issue_visop_e), .clk(clk), .q(visop_m), .si(), .so(), .se(se));
   dff_s visop_m2w(.din(visop_m), .clk(clk), .q(visop_w), .si(), .so(), .se(se));
   dff_s visop_w2w2(.din(visop_w_vld), .clk(clk), .q(visop_w2), .si(), .so(), .se(se));
   dff_s visop_w22w3(.din(visop_w2_vld), .clk(clk), .q(visop_w3), .si(), .so(), .se(se));

   assign     issue_visop_e = visop_e | visop & rollback_c3;
   // only check kills in w since they are accumulated into kill_w
   assign      visop_w_vld = visop_w & ~kill_w;
   assign      visop_w2_vld = visop_w2 & ~flush_w2 & ~rollback_rs2_w2;
   assign      visop_w3_vld = visop_w3 & ~ue_trap_w3 & ~rollback_rs1_w3;

   assign      vis_result = visop_w3_vld;
   assign      vis_wen_next = vis_result & ~siam & ~alignaddr;

   ////////////////////////////////////
   // Decode opf
   ////////////////////////////////////
   assign      add = ~opf[8] & ~opf[7] & opf[6] & ~opf[5] & opf[4] & ~opf[3];
   assign      align = ~opf[8] & ~opf[7] & opf[6] & ~opf[5] & ~opf[4] & opf[3] & ~opf[2] & ~opf[1] & ~opf[0];
   assign      wlogic = ~opf[8] & ~opf[7] & opf[6] & opf[5];
   assign siam = ~opf[8] & opf[7] & ~opf[6] & ~opf[5] & ~opf[4] & ~opf[3] & ~opf[2] & ~opf[1] & opf[0];
   assign alignaddr = ~opf[8] & ~opf[7] & ~opf[6] & ~opf[5] & opf[4] & opf[3] & ~opf[2] & ~opf[0]; //alignaddress

   assign illegal_vis_e = (visop_e & ~(add | align | wlogic | siam | alignaddr) | 
                           illegal_rs1_e | illegal_rs2_e | illegal_siam_e);
   assign rs1_check_nonzero_e = visop_e & (siam | (wlogic & (opf_log_zero | opf_log_one | opf_log_src2 | opf_log_not2)));
   assign rs2_check_nonzero_e = visop_e & wlogic & (opf_log_zero | opf_log_one | opf_log_src1 | opf_log_not1);
   assign illegal_rs1_e = (frs1_e[4:0] != 5'b00000) & rs1_check_nonzero_e;
   assign illegal_rs2_e = (frs2_e[4:0] != 5'b00000) & rs2_check_nonzero_e;
   assign illegal_siam_e = ((frd_e[4:0] != 5'b00000) | frs2_e[4] | frs2_e[3]) & siam & visop_e;

   assign vis_nofrf_e = visop_e & (siam | alignaddr | opf_log_zero | opf_log_one);
   
   // controls for add
   // Make subtract come out of its own flop for loading purposes (very critical timing)
   dff_s sub_dff(.din(opf[2]), .clk(clk), .q(ctl_vis_subtract), .se(se), .si(), .so());
   assign ctl_vis_cin = opf[2];
   assign ctl_vis_add32 = opf[1];

   // controls for logic
   assign opf_log_zero = ~opf[4] & ~opf[3] & ~opf[2] & ~opf[1];
   assign opf_log_nor = ~opf[4] & ~opf[3] & ~opf[2] & opf[1];
   assign opf_log_andnot2 = ~opf[4] & ~opf[3] & opf[2] & ~opf[1];
   assign opf_log_not2 = ~opf[4] & ~opf[3] & opf[2] & opf[1];
   assign opf_log_andnot1 = ~opf[4] & opf[3] & ~opf[2] & ~opf[1];
   assign opf_log_not1 = ~opf[4] & opf[3] & ~opf[2] & opf[1];
   assign opf_log_xor = ~opf[4] & opf[3] & opf[2] & ~opf[1];
   assign opf_log_nand = ~opf[4] & opf[3] & opf[2] & opf[1];
   assign opf_log_and = opf[4] & ~opf[3] & ~opf[2] & ~opf[1];
   assign opf_log_xnor = opf[4] & ~opf[3] & ~opf[2] & opf[1];
   assign opf_log_src1 = opf[4] & ~opf[3] & opf[2] & ~opf[1];
   assign opf_log_ornot2 = opf[4] & ~opf[3] & opf[2] & opf[1];
   assign opf_log_src2 = opf[4] & opf[3] & ~opf[2] & ~opf[1];
   assign opf_log_ornot1 = opf[4] & opf[3] & ~opf[2] & opf[1];
   assign opf_log_or = opf[4] & opf[3] & opf[2] & ~opf[1];
   assign opf_log_one = opf[4] & opf[3] & opf[2] & opf[1];

   // selects for logic mux
   assign ctl_vis_log_sel_nand = opf_log_or | opf_log_nand | opf_log_ornot1 | opf_log_ornot2;
   assign ctl_vis_log_sel_xor = opf_log_xor | opf_log_xnor;
   assign ctl_vis_log_sel_nor = opf_log_and | opf_log_nor | opf_log_andnot1 | opf_log_andnot2;
   assign ctl_vis_log_sel_pass = (opf_log_zero | opf_log_one | opf_log_src1 | opf_log_src2 |
                                  opf_log_not1 | opf_log_not2);

   assign invert_rs1_next = (opf_log_not1 | opf_log_or | opf_log_and | 
                                    opf_log_ornot2 | opf_log_andnot2);
   assign invert_rs2_next = (opf_log_not2 | opf_log_or | opf_log_and | 
                                    opf_log_ornot1 | opf_log_andnot1 | opf_log_xnor);
   dff_s invert_rs1_dff(.din(invert_rs1_next), .clk(clk), .q(ctl_vis_log_invert_rs1),
                      .se(se), .si(), .so());
   dff_s invert_rs2_dff(.din(invert_rs2_next), .clk(clk), .q(ctl_vis_log_invert_rs2),
                      .se(se), .si(), .so());
   // precalculate to help timing
   assign log_pass_rs1_next = opf_log_src1 | opf_log_not1;
   assign log_pass_rs2_next = opf_log_src2 | opf_log_not2;
   dff_s #(2) log_pass_dff(.din({log_pass_rs1_next,log_pass_rs2_next}), .clk(clk),
                         .q({log_pass_rs1,log_pass_rs2}), .se(se), .si(), .so());
   
   assign ctl_vis_log_pass_rs1 = log_pass_rs1;
   assign ctl_vis_log_pass_rs2 = log_pass_rs2 & ~log_pass_rs1;
   assign ctl_vis_log_constant = opf_log_one;
   assign ctl_vis_log_pass_const = ~(ctl_vis_log_pass_rs1 | ctl_vis_log_pass_rs2);
   
   // controls for falign
   assign ctl_vis_align0 = ~gsr_align_d1[2] & ~gsr_align_d1[1];
   assign ctl_vis_align2 = ~gsr_align_d1[2] & gsr_align_d1[1];
   assign ctl_vis_align4 = gsr_align_d1[2] & ~gsr_align_d1[1];
   assign ctl_vis_align6 = gsr_align_d1[2] & gsr_align_d1[1];
   assign ctl_vis_align_odd = gsr_align_d1[0];

   // controls for output mux
   assign ctl_vis_sel_add = add;
   assign ctl_vis_sel_align = align;
   assign ctl_vis_sel_log = ~(add | align);

   ///////////////////////////////////////////////////////////
   // GSR.alignaddr_offset, GSR.IM, GSR.IRND
   ///////////////////////////////////////////////////////////

   mux4ds #(6) curr_gsr_mux(.dout({gsr_rnd[2:0], gsr_align[2:0]}),
                            .in0({t0_gsr_rnd[2:0], t0_gsr_align[2:0]}),
                            .in1({t1_gsr_rnd[2:0], t1_gsr_align[2:0]}),
                            .in2({t2_gsr_rnd[2:0], t2_gsr_align[2:0]}),
                            .in3({t3_gsr_rnd[2:0], t3_gsr_align[2:0]}),
                            .sel0(thr_fp[0]),
                            .sel1(thr_fp[1]),
                            .sel2(thr_fp[2]),
                            .sel3(thr_fp[3]));
   mux4ds #(6) gsr_e_mux(.dout({gsr_rnd_e[2:0], gsr_align_e[2:0]}),
                            .in0({t0_gsr_rnd[2:0], t0_gsr_align[2:0]}),
                            .in1({t1_gsr_rnd[2:0], t1_gsr_align[2:0]}),
                            .in2({t2_gsr_rnd[2:0], t2_gsr_align[2:0]}),
                            .in3({t3_gsr_rnd[2:0], t3_gsr_align[2:0]}),
                            .sel0(ctl_dp_thr_e[0]),
                            .sel1(ctl_dp_thr_e[1]),
                            .sel2(ctl_dp_thr_e[2]),
                            .sel3(ctl_dp_thr_e[3]));
   dff_s #(43) gsr_e2m(.din({dp_ctl_gsr_mask_e[31:0],gsr_rnd_e[2:0],
                          dp_ctl_gsr_scale_e[4:0],gsr_align_e[2:0]}), .clk(clk),
                    .q({ffu_exu_rsr_data_hi_m[31:0],ffu_exu_rsr_data_mid_m[2:0], ffu_exu_rsr_data_lo_m[7:0]}),
                    .se(se), .si(), .so());
   dff_s #(3) gsr_align_dff(.din(gsr_align[2:0]), .clk(clk), .q(gsr_align_d1[2:0]), .se(se), .si(), .so());

   // put in to help timing for sending to lsu
   dff_s #(2) fpu_rnd_dff(.din(fpu_rnd_next[1:0]), .clk(clk), .q(fpu_rnd[1:0]), .si(), .so(), .se(se));
   assign      fpu_rnd_next[1:0] = (gsr_rnd[2])? gsr_rnd[1:0]: dp_ctl_fsr_rnd[1:0];

   // if alignaddress_little then write the 2's complement
   assign     align_addr_data_w2[2:0] = (opf[1])? (~wgsr_align_offset_w2[2:0] + 3'b001):
                                                  wgsr_align_offset_w2[2:0];
   
   assign     gsr_addr_d = (ifu_tlu_sraddr_d[6:0] == 7'b0010011);
   assign     wgsr_e = exu_ffu_wsr_inst_e & gsr_addr_e;
   dff_s gsr_addr_d2e(.din(gsr_addr_d), .clk(clk), .q(gsr_addr_e), .se(se), .si(), .so());

   // need independent kill checks because this isn't killed by new fpop
   assign     wgsr_vld_m = wgsr_m & ~(thr_match_mw2 & flush_w2);
   assign     wgsr_vld_w = wgsr_w & ifu_tlu_inst_vld_w & ~(thr_match_ww2 & flush_w2);
   assign     wgsr_vld_w2 = wgsr_w2 & ~flush_w2;
   dff_s wgsr_e2m(.din(wgsr_e), .clk(clk), .q(wgsr_m), .si(), .so(), .se(se));
   dff_s wgsr_m2w(.din(wgsr_vld_m), .clk(clk), .q(wgsr_w), .si(), .so(), .se(se));
   dff_s wgsr_w2w2(.din(wgsr_vld_w), .clk(clk), .q(wgsr_w2), .si(), .so(), .se(se));

   assign     thr_w2[3] = (tid_w2[1:0] == 2'b11);
   assign     thr_w2[2] = (tid_w2[1:0] == 2'b10);
   assign     thr_w2[1] = (tid_w2[1:0] == 2'b01);
   assign     thr_w2[0] = (tid_w2[1:0] == 2'b00);
   assign     ctl_dp_thr_e[3] = (tid_e[1:0] == 2'b11);
   assign     ctl_dp_thr_e[2] = (tid_e[1:0] == 2'b10);
   assign     ctl_dp_thr_e[1] = (tid_e[1:0] == 2'b01);
   assign     ctl_dp_thr_e[0] = (tid_e[1:0] == 2'b00);
   assign     thr_fp[3] = (tid[1:0] == 2'b11);
   assign     thr_fp[2] = (tid[1:0] == 2'b10);
   assign     thr_fp[1] = (tid[1:0] == 2'b01);
   assign     thr_fp[0] = (tid[1:0] == 2'b00);
   
   assign     t0_siam_w2 = thr_fp[0] & siam & visop_w2_vld;
   assign     t0_gsr_wsr_w2 = thr_w2[0] & wgsr_vld_w2;
   assign     t0_alignaddr_w2 = thr_fp[0] & alignaddr & visop_w2_vld;
   assign     t0_rnd_wen_l = ~(t0_gsr_wsr_w2 | t0_siam_w2);
   assign     t0_align_wen_l = ~(t0_gsr_wsr_w2 | t0_alignaddr_w2);
   assign     t1_siam_w2 = thr_fp[1] & siam & visop_w2_vld;
   assign     t1_gsr_wsr_w2 = thr_w2[1] & wgsr_vld_w2;
   assign     t1_alignaddr_w2 = thr_fp[1] & alignaddr & visop_w2_vld;
   assign     t1_rnd_wen_l = ~(t1_gsr_wsr_w2 | t1_siam_w2);
   assign     t1_align_wen_l = ~(t1_gsr_wsr_w2 | t1_alignaddr_w2);
   assign     t2_siam_w2 = thr_fp[2] & siam & visop_w2_vld;
   assign     t2_gsr_wsr_w2 = thr_w2[2] & wgsr_vld_w2;
   assign     t2_alignaddr_w2 = thr_fp[2] & alignaddr & visop_w2_vld;
   assign     t2_rnd_wen_l = ~(t2_gsr_wsr_w2 | t2_siam_w2);
   assign     t2_align_wen_l = ~(t2_gsr_wsr_w2 | t2_alignaddr_w2);
   assign     t3_siam_w2 = thr_fp[3] & siam & visop_w2_vld;
   assign     t3_gsr_wsr_w2 = thr_w2[3] & wgsr_vld_w2;
   assign     t3_alignaddr_w2 = thr_fp[3] & alignaddr & visop_w2_vld;
   assign     t3_rnd_wen_l = ~(t3_gsr_wsr_w2 | t3_siam_w2);
   assign     t3_align_wen_l = ~(t3_gsr_wsr_w2 | t3_alignaddr_w2);

   assign     ctl_dp_gsr_wsr_w2[3:0] = {t3_gsr_wsr_w2,t2_gsr_wsr_w2,t1_gsr_wsr_w2,t0_gsr_wsr_w2};

   // Storage flops and muxes
   mux3ds #(3) t0_rnd_mux(.dout(t0_gsr_rnd_next[2:0]),
                          .in0(t0_gsr_rnd[2:0]),
                          .in1(wgsr_rnd_w2[2:0]),
                          .in2(siam_rnd[2:0]),
                          .sel0(t0_rnd_wen_l),
                          .sel1(t0_gsr_wsr_w2),
                          .sel2(t0_siam_w2));
   mux3ds #(3) t0_align_mux(.dout(t0_gsr_align_next[2:0]),
                            .in0(t0_gsr_align[2:0]),
                            .in1(wgsr_align_offset_w2[2:0]),
                            .in2(align_addr_data_w2[2:0]),
                            .sel0(t0_align_wen_l),
                            .sel1(t0_gsr_wsr_w2),
                            .sel2(t0_alignaddr_w2));
   mux3ds #(3) t1_rnd_mux(.dout(t1_gsr_rnd_next[2:0]),
                          .in0(t1_gsr_rnd[2:0]),
                          .in1(wgsr_rnd_w2[2:0]),
                          .in2(siam_rnd[2:0]),
                          .sel0(t1_rnd_wen_l),
                          .sel1(t1_gsr_wsr_w2),
                          .sel2(t1_siam_w2));
   mux3ds #(3) t1_align_mux(.dout(t1_gsr_align_next[2:0]),
                            .in0(t1_gsr_align[2:0]),
                            .in1(wgsr_align_offset_w2[2:0]),
                            .in2(align_addr_data_w2[2:0]),
                            .sel0(t1_align_wen_l),
                            .sel1(t1_gsr_wsr_w2),
                            .sel2(t1_alignaddr_w2));
   mux3ds #(3) t2_rnd_mux(.dout(t2_gsr_rnd_next[2:0]),
                          .in0(t2_gsr_rnd[2:0]),
                          .in1(wgsr_rnd_w2[2:0]),
                          .in2(siam_rnd[2:0]),
                          .sel0(t2_rnd_wen_l),
                          .sel1(t2_gsr_wsr_w2),
                          .sel2(t2_siam_w2));
   mux3ds #(3) t2_align_mux(.dout(t2_gsr_align_next[2:0]),
                            .in0(t2_gsr_align[2:0]),
                            .in1(wgsr_align_offset_w2[2:0]),
                            .in2(align_addr_data_w2[2:0]),
                            .sel0(t2_align_wen_l),
                            .sel1(t2_gsr_wsr_w2),
                            .sel2(t2_alignaddr_w2));
   mux3ds #(3) t3_rnd_mux(.dout(t3_gsr_rnd_next[2:0]),
                          .in0(t3_gsr_rnd[2:0]),
                          .in1(wgsr_rnd_w2[2:0]),
                          .in2(siam_rnd[2:0]),
                          .sel0(t3_rnd_wen_l),
                          .sel1(t3_gsr_wsr_w2),
                          .sel2(t3_siam_w2));
   mux3ds #(3) t3_align_mux(.dout(t3_gsr_align_next[2:0]),
                            .in0(t3_gsr_align[2:0]),
                            .in1(wgsr_align_offset_w2[2:0]),
                            .in2(align_addr_data_w2[2:0]),
                            .sel0(t3_align_wen_l),
                            .sel1(t3_gsr_wsr_w2),
                            .sel2(t3_alignaddr_w2));


   dffr_s #(6) t0_gsr_dff(.din({t0_gsr_rnd_next[2:0], t0_gsr_align_next[2:0]}), .clk(clk),
                       .q({t0_gsr_rnd[2:0], t0_gsr_align[2:0]}), .se(se),
                       .si(), .so(), .rst(reset));
   dffr_s #(6) t1_gsr_dff(.din({t1_gsr_rnd_next[2:0], t1_gsr_align_next[2:0]}), .clk(clk),
                       .q({t1_gsr_rnd[2:0], t1_gsr_align[2:0]}), .se(se),
                       .si(), .so(), .rst(reset));
   dffr_s #(6) t2_gsr_dff(.din({t2_gsr_rnd_next[2:0], t2_gsr_align_next[2:0]}), .clk(clk),
                       .q({t2_gsr_rnd[2:0], t2_gsr_align[2:0]}), .se(se),
                       .si(), .so(), .rst(reset));
   dffr_s #(6) t3_gsr_dff(.din({t3_gsr_rnd_next[2:0], t3_gsr_align_next[2:0]}), .clk(clk),
                       .q({t3_gsr_rnd[2:0], t3_gsr_align[2:0]}), .se(se),
                       .si(), .so(), .rst(reset));

   dffre_s #(3) siam_rnd_dff(.din(ifu_ffu_rnd_e[2:0]), .clk(clk),
                          .q(siam_rnd), .se(se), .si(), .so(),
                           .rst(reset), .en(visop_e));
   dff_s #(3) align_offset_dff1(.din(exu_ffu_gsr_align_m[2:0]), .clk(clk),
                             .q(wgsr_align_offset_w[2:0]), .se(se), .si(), .so());
   dff_s #(3) align_offset_dff2(.din(wgsr_align_offset_w[2:0]), .clk(clk),
                             .q(wgsr_align_offset_w2[2:0]), .se(se), .si(), .so());
   dff_s #(3) rnd_dff1(.din(exu_ffu_gsr_rnd_m[2:0]), .clk(clk),
                      .q(wgsr_rnd_w[2:0]), .se(se), .si(), .so());
   dff_s #(3) rnd_dff2(.din(wgsr_rnd_w[2:0]), .clk(clk),
                      .q(wgsr_rnd_w2[2:0]), .se(se), .si(), .so());
   assign     wsr_data_m[36:0] = {exu_ffu_gsr_mask_m[31:0], exu_ffu_gsr_scale_m[4:0]};
   dff_s #(37) wsr_data_m2w(.din(wsr_data_m[36:0]), .clk(clk), .q(wsr_data_w[36:0]),
                          .se(se), .si(), .so());
   dff_s #(37) wsr_data_w2w2(.din(wsr_data_w[36:0]), .clk(clk), .q(ctl_dp_wsr_data_w2[36:0]),
                          .se(se), .si(), .so());
           
   
endmodule // sparc_ffu_ctl_visctl
// Modified by Princeton University on June 9th, 2015
// ========== Copyright Header Begin ==========================================
// 
// OpenSPARC T1 Processor File: sparc_ffu_dp.v
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
//  Module Name: sparc_ffu_dp
//	Description: This is the ffu datapath.  It stores the 2 128 bit operands
//  	and the result (puts result in the 1st source to save space). 
*/

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
























module sparc_ffu_dp (/*AUTOARG*/
   // Outputs
   so, dp_frf_data, ffu_lsu_data, dp_vis_rs1_data, dp_vis_rs2_data, 
   dp_ctl_rs2_sign, dp_ctl_fsr_fcc, dp_ctl_fsr_rnd, dp_ctl_fsr_tem, 
   dp_ctl_fsr_aexc, dp_ctl_fsr_cexc, dp_ctl_ld_fcc, 
   dp_ctl_gsr_mask_e, dp_ctl_gsr_scale_e, dp_ctl_synd_out_low, 
   dp_ctl_synd_out_high, 
   // Inputs
   rclk, se, si, ctl_dp_rst_l, frf_dp_data, cpx_fpu_data, 
   lsu_ffu_ld_data, vis_dp_rd_data, ctl_dp_wsr_data_w2, ctl_dp_sign, 
   ctl_dp_exc_w2, ctl_dp_fcc_w2, ctl_dp_ftt_w2, ctl_dp_noshift64_frf, 
   ctl_dp_shift_frf_right, ctl_dp_shift_frf_left, 
   ctl_dp_zero_low32_frf, ctl_dp_output_sel_rs1, 
   ctl_dp_output_sel_rs2, ctl_dp_output_sel_frf, 
   ctl_dp_output_sel_fsr, ctl_dp_noflip_lsu, ctl_dp_flip_lsu, 
   ctl_dp_noflip_fpu, ctl_dp_flip_fpu, ctl_dp_rs2_frf_read, 
   ctl_dp_rs2_sel_vis, ctl_dp_rs2_sel_fpu_lsu, ctl_dp_rs2_keep_data, 
   ctl_dp_rd_ecc, ctl_dp_fp_thr, ctl_dp_fsr_sel_old, 
   ctl_dp_fsr_sel_ld, ctl_dp_fsr_sel_fpu, ctl_dp_gsr_wsr_w2, 
   ctl_dp_thr_e, ctl_dp_new_rs1, ctl_dp_ecc_sel_frf
   ) ;
   input rclk;
   input se;
   input si;
   input ctl_dp_rst_l;
   input [77:0] frf_dp_data;
   input [63:0]   cpx_fpu_data;
   input [63:0] lsu_ffu_ld_data;
   input [63:0] vis_dp_rd_data;

   input [36:0] ctl_dp_wsr_data_w2;

   input [1:0]   ctl_dp_sign;  // sign after abs or neg
   input [9:0] 	 ctl_dp_exc_w2;
   input [7:0] 	 ctl_dp_fcc_w2;
   input [2:0] 	 ctl_dp_ftt_w2;

   // mux selects
   input         ctl_dp_noshift64_frf;     // choose output from FRF
   input         ctl_dp_shift_frf_right;
   input         ctl_dp_shift_frf_left;

   input         ctl_dp_zero_low32_frf;
   
   input         ctl_dp_output_sel_rs1;  // choose output to lsu
   input         ctl_dp_output_sel_rs2;
   input         ctl_dp_output_sel_frf;
   input         ctl_dp_output_sel_fsr;

   input         ctl_dp_noflip_lsu;// inputs from lsu and fpu
   input         ctl_dp_flip_lsu;
   input         ctl_dp_noflip_fpu;
   input         ctl_dp_flip_fpu;
   
   input         ctl_dp_rs2_frf_read;    // choose r2
   input         ctl_dp_rs2_sel_vis;
   input         ctl_dp_rs2_sel_fpu_lsu;
   input         ctl_dp_rs2_keep_data;
   input         ctl_dp_rd_ecc;
   
   input [3:0] 	 ctl_dp_fp_thr;

   input [3:0] 	 ctl_dp_fsr_sel_old,     // choose what to update FSR with
		 ctl_dp_fsr_sel_ld,
		 ctl_dp_fsr_sel_fpu;

   input [3:0]   ctl_dp_gsr_wsr_w2;
   input [3:0]   ctl_dp_thr_e;
   

   // rs1 selects
   input         ctl_dp_new_rs1;
   
   // 2:1 mux selects
   input      ctl_dp_ecc_sel_frf;

   // outputs
   output       so;
   output [63:0] dp_frf_data;
   output [63:0] ffu_lsu_data;
   output [63:0] dp_vis_rs1_data;
   output [63:0] dp_vis_rs2_data;
   output [1:0]  dp_ctl_rs2_sign;  // sign for rs2
   output [7:0]  dp_ctl_fsr_fcc;
   output [1:0]  dp_ctl_fsr_rnd;
   output [4:0]  dp_ctl_fsr_tem;
   output [4:0]  dp_ctl_fsr_aexc;
   output [4:0]  dp_ctl_fsr_cexc;
   
   output [7:0]  dp_ctl_ld_fcc;

   output [31:0] dp_ctl_gsr_mask_e;
   output [4:0]  dp_ctl_gsr_scale_e;
   
   
   output [6:0] dp_ctl_synd_out_low;   // signals for ecc errors
   output [6:0] dp_ctl_synd_out_high;
 
   wire         clk;
   wire         reset;
   // local signals
   wire [63:0] 	 fpu_ffu_data;
   wire [63:0]   lsu_ffu_ld_data_d1;
   wire [63:0]   rs2_rd_data;  // stores both the rs2 and rd data
   wire [63:0]   rs2_rd_data_next;
   wire [63:0]   write_data;    // needed since block loads are pipelined
   wire [63:0]   rs2_data_changed;
   wire [63:0]   local_rd_data;
   wire [63:0]   rs1_data;
   wire [63:0]   rs1_data_next;
   wire [63:0]   shifted_frf_data;
   wire [63:0]   new_frf_data;
   wire [63:0]   lsu_fpu_data;
   wire [63:0]   frf_data_in;
   wire [6:0]    synd_in_low;   // input ecc for lower word
   wire [6:0]    synd_in_h;   // input ecc for upper word
   wire [63:0] corr_data_next;
   wire [63:0] corr_data;
   wire [63:0] ecc_data_in;

   wire [27:0] 	 current_fsr,
		 t0_fsr,
		 t1_fsr,
		 t2_fsr,
		 t3_fsr;
   wire [27:0] 	 t0_fsr_nxt,
		 t1_fsr_nxt,
		 t2_fsr_nxt,
		 t3_fsr_nxt;
   wire [27:0] 	 t0_ldfsr_data,
		             t0_fpufsr_data;
   wire [27:0] 	 t1_ldfsr_data,
		             t1_fpufsr_data;
   wire [27:0] 	 t2_ldfsr_data,
		             t2_fpufsr_data;
   wire [27:0] 	 t3_ldfsr_data,
		             t3_fpufsr_data;

   wire [36:0]   gsr_e;
   wire [36:0]   t0_gsr;
   wire [36:0]   t0_gsr_nxt;
   wire [36:0]   t1_gsr;
   wire [36:0]   t1_gsr_nxt;
   wire [36:0]   t2_gsr;
   wire [36:0]   t2_gsr_nxt;
   wire [36:0]   t3_gsr;
   wire [36:0]   t3_gsr_nxt;

   assign        reset = ~ctl_dp_rst_l;
   assign        clk= rclk;

   dff_s #(64) cpx_reg(.din(cpx_fpu_data[63:0]),
			               .q   (fpu_ffu_data[63:0]),
			               .clk (clk), .se(se), .si(), .so());

   // flop for lsu data.  the data is flopped in ffu, but the vld is flopped in the lsu.
   // This is for timing reasons on the valid bit and Sanjay didn't want to redo the
   // lsu dp for the data portion
   dff_s #(64) lsu_data_dff(.din(lsu_ffu_ld_data[63:0]), .clk(clk), .q(lsu_ffu_ld_data_d1[63:0]),
                          .se(se), .si(), .so());
   assign dp_ctl_ld_fcc[7:0] = {lsu_ffu_ld_data_d1[37:32], lsu_ffu_ld_data_d1[11:10]};
   
   ///////////////////////////////////////////////
   //	Input from FRF (shift as needed for singles)
   //	The data needs to be shifted around because these are 64 bit reads but
   //	the required data might be in either the upper or lower 32 bits for
   //	singles.  If it is a double then the data is left alone.
   //	If it is a single move and the source and target have the same alignment
   //	then no change happens.  If it is a single move and the source and target
   //	have different alignments the operands get moved into place for the write.
   //	If it is data that will be sent to the lsu the data is moved into the lower
   //	32 bits.  If the data will be sent to the fpu the data is moved to the upper
   //	32 bits (if not there already)
   ///////////////////////////////////////////////
   assign  frf_data_in[63:32] = frf_dp_data[70:39];
   assign  frf_data_in[31:0] = frf_dp_data[31:0];
   mux3ds #(64) frf_input_mux(.dout(shifted_frf_data[63:0]),
                              .in0(frf_data_in[63:0]),
                              .in1({32'b0, frf_data_in[63:32]}),
                              .in2({frf_data_in[31:0], 32'b0}),
                              .sel0(ctl_dp_noshift64_frf),
                              .sel1(ctl_dp_shift_frf_right),
                              .sel2(ctl_dp_shift_frf_left));
   assign  new_frf_data[63:32] = shifted_frf_data[63:32];
   assign  new_frf_data[31:0] = shifted_frf_data[31:0] & {32{~ctl_dp_zero_low32_frf}};

   mux4ds #(64) lsu_fpu_input_mux(.dout(lsu_fpu_data[63:0]),
                                  .in0(lsu_ffu_ld_data_d1[63:0]),
                                  .in1({lsu_ffu_ld_data_d1[31:0], 32'b0}),
                                  .in2(fpu_ffu_data[63:0]),
                                  .in3({32'b0, fpu_ffu_data[63:32]}),
                                  .sel0(ctl_dp_noflip_lsu),
                                  .sel1(ctl_dp_flip_lsu),
                                  .sel2(ctl_dp_noflip_fpu),
                                  .sel3(ctl_dp_flip_fpu));

   // Data to FRF
   dp_buffer #(64) frf_out_buf(.in(write_data[63:0]), .dout (dp_frf_data[63:0]));



   // Data to LSU
   // Mux for lsu data between two sets of data and the direct 
   // frf output for stores
   mux4ds #(64) output_mux(.dout (ffu_lsu_data[63:0]),
                           .in0  (rs2_rd_data[63:0]),
                           .in1  (rs1_data[63:0]),
                           .in2  (shifted_frf_data[63:0]),
			                     .in3  ({26'b0, current_fsr[27:20], 2'b0, current_fsr[19:15], 6'b0, current_fsr[14:12], 2'b0, current_fsr[11:0]}),
                           .sel0 (ctl_dp_output_sel_rs2),
                           .sel1 (ctl_dp_output_sel_rs1),
                           .sel2 (ctl_dp_output_sel_frf),
			                     .sel3 (ctl_dp_output_sel_fsr));

   // RS2 can take value from frf  (with modification to sign), from lsu 
   // or keep value
   // The modification to the sign bits allows for FABS and FNEG
   assign dp_ctl_rs2_sign[1:0] = {new_frf_data[63], new_frf_data[31]};
   
   assign rs2_data_changed[63:0] = {ctl_dp_sign[1], new_frf_data[62:32], 
				    ctl_dp_sign[0], new_frf_data[30:0]};

   dp_mux2es #(64) local_rd_mux(.dout(local_rd_data[63:0]),
                             .in0(rs2_data_changed[63:0]),
                             .in1(corr_data[63:0]),
                             .sel(ctl_dp_rd_ecc));

   mux4ds #(64) rs2_rd_mux(.dout (rs2_rd_data_next[63:0]),
                           .in0  (local_rd_data[63:0]),
                           .in1  (vis_dp_rd_data[63:0]),
                           .in2  (lsu_fpu_data[63:0]),
                           .in3  (rs2_rd_data[63:0]),
                           .sel0 (ctl_dp_rs2_frf_read),
                           .sel1 (ctl_dp_rs2_sel_vis),
                           .sel2 (ctl_dp_rs2_sel_fpu_lsu),
                           .sel3 (ctl_dp_rs2_keep_data));
   
   dff_s #(64) rs2_rd_dff(.din (rs2_rd_data_next[63:0]), 
			                  .q   (rs2_rd_data[63:0]),
			                  .clk (clk), .se(se), .si(), .so());
   assign dp_vis_rs2_data[63:0] = rs2_rd_data[63:0];
   dff_s #(64) write_data_dff(.din(rs2_rd_data[63:0]),
                            .q(write_data[63:0]),
                            .clk(clk), .se(se), .si(), .so());
   
   ////////////////////////////////////////////////////////
   //			RS1
   ////////////////////////////////////////////////////////
   // RS1 next either takes value from frf or keeps value
   dp_mux2es #(64) rs1_mux(.dout (rs1_data_next[63:0]),
                           .in0  (rs1_data[63:0]),
                           .in1  (new_frf_data[63:0]),
                           .sel  (ctl_dp_new_rs1));
   
   dff_s #(64) rs1_dff(.din (rs1_data_next[63:0]), 
		                 .q   (rs1_data[63:0]),
		                 .clk (clk), .se(se), .si(), .so());
   assign dp_vis_rs1_data[63:0] = rs1_data[63:0];


   /////////////////////////////////////////////////////////
   //			FSR
   /////////////////////////////////////////////////////////
   // FSR takes data from load
   // fsr is set by ldfsr, ldxfsr, or any fpu operation
   assign t0_ldfsr_data[27:0] = {ctl_dp_fcc_w2[7:2],     // fcc3,2,1
			                           lsu_ffu_ld_data_d1[31:30],   // RND mode
			                           //2'b0,                     // rsvd
			                           lsu_ffu_ld_data_d1[27:23],   // TEM
			                           //6'b0,                     // NS, rsvd, ver
			                           t0_fsr[14:12],       // ftt
			                           //2'b0,                     // qne, rsvd
			                           lsu_ffu_ld_data_d1[11:0]};   // fcc0, aexc, cexc
   
   assign t0_fpufsr_data[27:0] = {ctl_dp_fcc_w2[7:2],
			                            t0_fsr[21:20],  // rnd
			                            t0_fsr[19:15],  // TEM
			                            ctl_dp_ftt_w2[2:0],  // ftt
			                            ctl_dp_fcc_w2[1:0],
			                            ctl_dp_exc_w2[9:0]};

   assign t1_ldfsr_data[27:0] = {ctl_dp_fcc_w2[7:2],     // fcc3,2,1
			                           lsu_ffu_ld_data_d1[31:30],   // RND mode
			                           //2'b0,                     // rsvd
			                           lsu_ffu_ld_data_d1[27:23],   // TEM
			                           //6'b0,                     // NS, rsvd, ver
			                           t1_fsr[14:12],       // ftt
			                           //2'b0,                     // qne, rsvd
			                           lsu_ffu_ld_data_d1[11:0]};   // fcc0, aexc, cexc
   
   assign t1_fpufsr_data[27:0] = {ctl_dp_fcc_w2[7:2],
			                            t1_fsr[21:20],  // rnd
			                            t1_fsr[19:15],  // TEM
			                            ctl_dp_ftt_w2[2:0],  // ftt
			                            ctl_dp_fcc_w2[1:0],
			                            ctl_dp_exc_w2[9:0]};

   assign t2_ldfsr_data[27:0] = {ctl_dp_fcc_w2[7:2],     // fcc3,2,1
			                           lsu_ffu_ld_data_d1[31:30],   // RND mode
			                           //2'b0,                     // rsvd
			                           lsu_ffu_ld_data_d1[27:23],   // TEM
			                           //6'b0,                     // NS, rsvd, ver
			                           t2_fsr[14:12],       // ftt
			                           //2'b0,                     // qne, rsvd
			                           lsu_ffu_ld_data_d1[11:0]};   // fcc0, aexc, cexc
   
   assign t2_fpufsr_data[27:0] = {ctl_dp_fcc_w2[7:2],
			                            t2_fsr[21:20],  // rnd
			                            t2_fsr[19:15],  // TEM
			                            ctl_dp_ftt_w2[2:0],  // ftt
			                            ctl_dp_fcc_w2[1:0],
			                            ctl_dp_exc_w2[9:0]};

   assign t3_ldfsr_data[27:0] = {ctl_dp_fcc_w2[7:2],     // fcc3,2,1
			                           lsu_ffu_ld_data_d1[31:30],   // RND mode
			                           //2'b0,                     // rsvd
			                           lsu_ffu_ld_data_d1[27:23],   // TEM
			                           //6'b0,                     // NS, rsvd, ver
			                           t3_fsr[14:12],       // ftt
			                           //2'b0,                     // qne, rsvd
			                           lsu_ffu_ld_data_d1[11:0]};   // fcc0, aexc, cexc
   
   assign t3_fpufsr_data[27:0] = {ctl_dp_fcc_w2[7:2],
			                            t3_fsr[21:20],  // rnd
			                            t3_fsr[19:15],  // TEM
			                            ctl_dp_ftt_w2[2:0],  // ftt
			                            ctl_dp_fcc_w2[1:0],
			                            ctl_dp_exc_w2[9:0]};

 // Use two threads unless this is defined

   mux3ds #28 fsr0_mux(.dout (t0_fsr_nxt[27:0]),
               .in0  (t0_fsr[27:0]),
               .in1  (t0_ldfsr_data[27:0]),
               .in2  (t0_fpufsr_data[27:0]),
               .sel0 (ctl_dp_fsr_sel_old[0]),
               .sel1 (ctl_dp_fsr_sel_ld[0]),
               .sel2 (ctl_dp_fsr_sel_fpu[0]));
   mux3ds #28 fsr1_mux(.dout (t1_fsr_nxt[27:0]),
               .in0  (t1_fsr[27:0]),
               .in1  (t1_ldfsr_data[27:0]),
               .in2  (t1_fpufsr_data[27:0]),
               .sel0 (ctl_dp_fsr_sel_old[1]),
               .sel1 (ctl_dp_fsr_sel_ld[1]),
               .sel2 (ctl_dp_fsr_sel_fpu[1]));

   // FSR registers
   // need only 28 flops for FSR since rest are always 0
   dffr_s #28 fsr0_reg(.din (t0_fsr_nxt[27:0]),
                         .q   (t0_fsr[27:0]),
                     .rst(reset),
                         .clk (clk), .se(se), .si(), .so());
   dffr_s #28 fsr1_reg(.din (t1_fsr_nxt[27:0]),
                         .q   (t1_fsr[27:0]),
                     .rst(reset),
                         .clk (clk), .se(se), .si(), .so());

   // Current FSR
   mux2ds #28 curr_fsr_mux(.dout (current_fsr[27:0]),
               .in0  (t0_fsr[27:0]),
               .in1  (t1_fsr[27:0]),
               .sel0 (ctl_dp_fp_thr[0]),
               .sel1 (ctl_dp_fp_thr[1]));






















































































































































































 // `ifndef CONFIG_NUM_THREADS

   assign dp_ctl_fsr_fcc = {current_fsr[27:22], current_fsr[11:10]};
   assign dp_ctl_fsr_rnd = current_fsr[21:20];
   assign dp_ctl_fsr_tem = current_fsr[19:15];
   assign dp_ctl_fsr_aexc = current_fsr[9:5];
   assign dp_ctl_fsr_cexc = current_fsr[4:0];
   
   ////////////////////////////////////////////////////////////
   //		ECC generation and correction
   ////////////////////////////////////////////////////////////
   dp_mux2es #(64) ecc_mux(.dout(ecc_data_in[63:0]),
                           .in0(rs2_rd_data[63:0]),
                           .in1({frf_dp_data[70:39], frf_dp_data[31:0]}),
                           .sel(ctl_dp_ecc_sel_frf));

   assign     synd_in_low[6:0] = {7{ctl_dp_ecc_sel_frf}} & frf_dp_data[38:32];
   assign     synd_in_h[6:0] = {7{ctl_dp_ecc_sel_frf}} & frf_dp_data[77:71];
   
   zzecc_sctag_ecc39 ecccor_low(.din(ecc_data_in[31:0]),
                                .parity(synd_in_low[6:0]),
                                .dout(corr_data_next[31:0]),
                                .pflag(dp_ctl_synd_out_low[6]),
                                .cflag(dp_ctl_synd_out_low[5:0]));
   
   zzecc_sctag_ecc39 ecccor_high(.din(ecc_data_in[63:32]),
                                .parity(synd_in_h[6:0]),
                                .dout(corr_data_next[63:32]),
                                .pflag(dp_ctl_synd_out_high[6]),
                                .cflag(dp_ctl_synd_out_high[5:0]));

   
   dff_s #(64) ecc_corr_data(.din(corr_data_next[63:0]), .q(corr_data[63:0]),
                           .clk(clk), .se(se), .si(), .so());


   ////////////////////////////////////////////////
   // GSR Storage
   ////////////////////////////////////////////////
   // GSR registers
   // need only 37 flops for GSR since rest are always 0
   // and the align and rnd fields are in the ctl block
 // Use two threads unless this is defined

   dffr_s #37 gsr0_reg(.din (t0_gsr_nxt[36:0]),
                         .q   (t0_gsr[36:0]),
                     .rst(reset),
                         .clk (clk), .se(se), .si(), .so());
   dffr_s #37 gsr1_reg(.din (t1_gsr_nxt[36:0]),
                         .q   (t1_gsr[36:0]),
                     .rst(reset),
                         .clk (clk), .se(se), .si(), .so());

   dp_mux2es #(37) gsr0_mux(.dout(t0_gsr_nxt[36:0]),
                            .in0(t0_gsr[36:0]),
                            .in1(ctl_dp_wsr_data_w2[36:0]),
                            .sel(ctl_dp_gsr_wsr_w2[0]));
   dp_mux2es #(37) gsr1_mux(.dout(t1_gsr_nxt[36:0]),
                            .in0(t1_gsr[36:0]),
                            .in1(ctl_dp_wsr_data_w2[36:0]),
                            .sel(ctl_dp_gsr_wsr_w2[1]));

   // GSR_E
   mux2ds #37 curr_gsr_mux(.dout (gsr_e[36:0]),
               .in0  (t0_gsr[36:0]),
               .in1  (t1_gsr[36:0]),
               .sel0 (ctl_dp_thr_e[0]),
               .sel1 (ctl_dp_thr_e[1]));





































































































































 // `ifndef CONFIG_NUM_THREADS
   
   assign     dp_ctl_gsr_scale_e[4:0] = gsr_e[4:0];
   assign     dp_ctl_gsr_mask_e[31:0] = gsr_e[36:5];

   
   

endmodule // sparc_ffu_dp
// Modified by Princeton University on June 9th, 2015
// ========== Copyright Header Begin ==========================================
// 
// OpenSPARC T1 Processor File: sparc_ffu_part_add32.v
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
//  Module Name: sparc_ffu_part_add32
//      Description: This is the ffu VIS adder.  It can do either 
//				2 16 bit adds or 1 32 bit add.
*/
module sparc_ffu_part_add32 (/*AUTOARG*/
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

   assign        cin16 = (add32)? cout15: cin;

   assign      {cout15, z[15:0]} = a[15:0]+b[15:0]+{15'b0,cin};   
   assign      z[31:16] = a[31:16]+b[31:16]+{15'b0,cin16};   
   
endmodule // sparc_ffu_part_add32
// Modified by Princeton University on June 9th, 2015
// ========== Copyright Header Begin ==========================================
// 
// OpenSPARC T1 Processor File: sparc_ffu_vis.v
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
//  Module Name: sparc_ffu_vis
//      Description: This is the ffu VIS blk.
//	It implements FALIGN, partitioned add and logicals.
*/

module sparc_ffu_vis(/*AUTOARG*/
   // Outputs
   vis_dp_rd_data, 
   // Inputs
   dp_vis_rs1_data, dp_vis_rs2_data, ctl_vis_sel_add, 
   ctl_vis_sel_log, ctl_vis_sel_align, ctl_vis_add32, 
   ctl_vis_subtract, ctl_vis_cin, ctl_vis_align0, ctl_vis_align2, 
   ctl_vis_align4, ctl_vis_align6, ctl_vis_align_odd, 
   ctl_vis_log_sel_pass, ctl_vis_log_sel_nand, ctl_vis_log_sel_nor, 
   ctl_vis_log_sel_xor, ctl_vis_log_invert_rs1, 
   ctl_vis_log_invert_rs2, ctl_vis_log_constant, 
   ctl_vis_log_pass_const, ctl_vis_log_pass_rs1, 
   ctl_vis_log_pass_rs2
   );

   input [63:0] dp_vis_rs1_data;
   input [63:0] dp_vis_rs2_data;
   input        ctl_vis_sel_add;
   input        ctl_vis_sel_log;
   input        ctl_vis_sel_align;
   input        ctl_vis_add32;
   input        ctl_vis_subtract;
   input        ctl_vis_cin;
   input         ctl_vis_align0;
   input         ctl_vis_align2;
   input         ctl_vis_align4;
   input         ctl_vis_align6;
   input         ctl_vis_align_odd;
   input         ctl_vis_log_sel_pass;
   input         ctl_vis_log_sel_nand;
   input         ctl_vis_log_sel_nor;
   input         ctl_vis_log_sel_xor;
   input         ctl_vis_log_invert_rs1;
   input         ctl_vis_log_invert_rs2;
   input         ctl_vis_log_constant;
   input         ctl_vis_log_pass_const;
   input         ctl_vis_log_pass_rs1;
   input         ctl_vis_log_pass_rs2;

   output [63:0] vis_dp_rd_data;

   wire [71:0]   align_data1;
   wire [63:0]   align_rs1;
   wire [63:8]   align_rs2;

   wire [63:0]   add_out;
   wire [63:0]   log_out;
   wire [63:0]   align_out;
   wire [63:0]   add_in_rs1;
   wire [63:0]   add_in_rs2;

   wire [63:0]   logic_nor;
   wire [63:0]   logic_pass;
   wire [63:0]   logic_xor;
   wire [63:0]   logic_nand;
   wire [63:0]   logic_rs1;
   wire [63:0]   logic_rs2;
   
   /////////////////////////////////////////////////////////////////
   // Logic for partitioned addition.
   //----------------------------------
   // RS1 is normal RS1 data, RS2 is inverted by subtraction signal.
   /////////////////////////////////////////////////////////////////
   assign        add_in_rs1[63:0] = dp_vis_rs1_data[63:0];
   assign        add_in_rs2[63:0] = dp_vis_rs2_data[63:0] ^ {64{ctl_vis_subtract}};
   sparc_ffu_part_add32 part_adder_hi(.z(add_out[63:32]),
                                   .add32(ctl_vis_add32),
                                   .a(add_in_rs1[63:32]),
                                   .b(add_in_rs2[63:32]),
                                   .cin(ctl_vis_cin));
   sparc_ffu_part_add32 part_adder_lo(.z(add_out[31:0]),
                                   .add32(ctl_vis_add32),
                                   .a(add_in_rs1[31:0]),
                                   .b(add_in_rs2[31:0]),
                                   .cin(ctl_vis_cin));

   ///////////////////////////////////////////////////////////////////////////
   // Datapath for FALIGNDATA
   //---------------------------------------------------------------
   // FALIGNDATA concatenates rs1 and rs2 and shifts them by byte to create
   // an 8 byte value.  The first mux creates a 72 bit value and the
   // 2nd mux picks 64 bits out of these for the output.
   ///////////////////////////////////////////////////////////////////////////
   dp_buffer #(64) align_rs1_buf(.dout(align_rs1[63:0]), .in(dp_vis_rs1_data[63:0]));
   dp_buffer #(56) align_rs2_buf(.dout(align_rs2[63:8]), .in(dp_vis_rs2_data[63:8]));
   mux4ds #(72) falign_mux1(.dout(align_data1[71:0]),
                            .in0({align_rs1[63:0], align_rs2[63:56]}),
                            .in1({align_rs1[47:0], align_rs2[63:40]}),
                            .in2({align_rs1[31:0], align_rs2[63:24]}),
                            .in3({align_rs1[15:0], align_rs2[63:8]}),
                            .sel0(ctl_vis_align0),
                            .sel1(ctl_vis_align2),
                            .sel2(ctl_vis_align4),
                            .sel3(ctl_vis_align6));
   dp_mux2es #(64) falign_mux2(.dout(align_out[63:0]),
                              .in0(align_data1[71:8]),
                              .in1(align_data1[63:0]),
                              .sel(ctl_vis_align_odd));

   ///////////////////////////////////////////////////////////////////////////
   // Datapath for VIS logicals
   //-----------------------------------------------------------------------
   // VIS logicals perform 3 fundamental ops: NAND, NOR and XOR plus inverted
   // versions of the inputs to create the other versions.  These 3 outputs are
   // muxed with a choice of 1, 0, rs1 or rs2.
   ///////////////////////////////////////////////////////////////////////////

   // create inverted versions of data if desired
   assign        logic_rs1[63:0] = dp_vis_rs1_data[63:0] ^ {64{ctl_vis_log_invert_rs1}};
   assign        logic_rs2[63:0] = dp_vis_rs2_data[63:0] ^ {64{ctl_vis_log_invert_rs2}};

   // 3 basic logical operations
   assign        logic_nor[63:0] = ~(logic_rs1[63:0] | logic_rs2[63:0]);
   assign        logic_nand[63:0] = ~(logic_rs1[63:0] & logic_rs2[63:0]);
   assign        logic_xor[63:0] = (logic_rs1[63:0] ^ logic_rs2[63:0]);
   
   // mux for pass through data
   mux3ds #(64) pass_mux(.dout(logic_pass[63:0]),
                         .in0({64{ctl_vis_log_constant}}),
                         .in1(logic_rs1[63:0]),
                         .in2(logic_rs2[63:0]),
                         .sel0(ctl_vis_log_pass_const),
                         .sel1(ctl_vis_log_pass_rs1),
                         .sel2(ctl_vis_log_pass_rs2));

   // pick between logic outputs
   mux4ds #(64) logic_mux(.dout(log_out[63:0]),
                          .in0(logic_nor[63:0]),
                          .in1(logic_nand[63:0]),
                          .in2(logic_xor[63:0]),
                          .in3(logic_pass[63:0]),
                          .sel0(ctl_vis_log_sel_nor),
                          .sel1(ctl_vis_log_sel_nand),
                          .sel2(ctl_vis_log_sel_xor),
                          .sel3(ctl_vis_log_sel_pass));


   
   //////////////////////////////////////////////////////////
   // output mux
   //////////////////////////////////////////////////////////
   mux3ds #(64) output_mux(.dout(vis_dp_rd_data[63:0]),
                           .in0(add_out[63:0]),
                           .in1(log_out[63:0]),
                           .in2(align_out[63:0]),
                           .sel0(ctl_vis_sel_add),
                           .sel1(ctl_vis_sel_log),
                           .sel2(ctl_vis_sel_align));
   
endmodule // sparc_ffu_vis
