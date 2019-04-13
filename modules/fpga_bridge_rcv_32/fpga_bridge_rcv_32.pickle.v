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
 * Module encapsulates the chip to fpga receive interface for networks 
 * 1, 2 and 3. packets received are 64 bits wide, received in 2 
 * 32-bit payloads.
 *
 *******************************************************************/



`timescale 1ns/1ps

module fpga_bridge_rcv_32 (
    rst, 
    wr_clk,
    rd_clk,
    credit_rd_clk,
    bout_data_1,
    bout_val_1,
    bout_rdy_1,
    bout_data_2,
    bout_val_2,
    bout_rdy_2,
    bout_data_3,
    bout_val_3,
    bout_rdy_3,
    data_from_chip,
    data_channel,
    credit_to_chip
); 

input rst;
input wr_clk;
input rd_clk;
input credit_rd_clk;
input bout_rdy_1;
input bout_rdy_2;
input bout_rdy_3;
input [31:0] data_from_chip;
input [ 1:0] data_channel;

output [63:0]   bout_data_1;
output          bout_val_1;
output [63:0]   bout_data_2;
output          bout_val_2;
output [63:0]   bout_data_3;
output          bout_val_3;
output [2:0]    credit_to_chip;

wire sort_rdy_1;
wire sort_rdy_2;
wire sort_rdy_3;


wire [63:0] sort_data_1;
wire [63:0] sort_data_2;
wire [63:0] sort_data_3;

wire sort_val_1;
wire sort_val_2;
wire sort_val_3;

wire fifo1_full;
wire fifo2_full;
wire fifo3_full;
wire credit_fifo_full;

reg [31:0] data_from_chip_f /* synthesis iob = true */;
reg  [1:0] data_channel_f /* synthesis iob = true */;
reg [31:0] data_from_chip_ff; // Buffering these for timing
reg  [1:0] data_channel_ff; // Buffering  these for timing
reg [31:0] data_from_chip_fff; // Buffering these for timing
reg  [1:0] data_channel_fff; // Buffering  these for timing

reg [31:0] channel_buffer;
reg [0:0]  channel_buffer_count;
wire [63:0] buffered_data;
reg [1:0] buffered_channel;

reg wr_rst_f;
reg wr_rst_ff;
reg rd_rst_f;
reg rd_rst_ff;
reg credit_rd_rst_f;
reg credit_rd_rst_ff;

// assume reset is asynchronous, synchronize
always @ (posedge wr_clk)
begin
    wr_rst_f <= rst;
    wr_rst_ff <= wr_rst_f;
end

always @ (posedge rd_clk)
begin
    rd_rst_f <= rst;
    rd_rst_ff <= rd_rst_f;
end

always @ (posedge credit_rd_clk)
begin
    credit_rd_rst_f <= rst;
    credit_rd_rst_ff <= credit_rd_rst_f;
end

assign sort_data_1 = (buffered_channel == 2'b01 && buffered_channel == data_channel_fff && channel_buffer_count == 1'b1) ? buffered_data : 64'd0;
assign sort_data_2 = (buffered_channel == 2'b10 && buffered_channel == data_channel_fff && channel_buffer_count == 1'b1) ? buffered_data : 64'd0;
assign sort_data_3 = (buffered_channel == 2'b11 && buffered_channel == data_channel_fff && channel_buffer_count == 1'b1) ? buffered_data : 64'd0;


assign sort_val_1  = (buffered_channel == 2'b01 && buffered_channel == data_channel_fff && channel_buffer_count == 1'b1) ? 1'b1 : 1'b0;
assign sort_val_2  = (buffered_channel == 2'b10 && buffered_channel == data_channel_fff && channel_buffer_count == 1'b1) ? 1'b1 : 1'b0;
assign sort_val_3  = (buffered_channel == 2'b11 && buffered_channel == data_channel_fff && channel_buffer_count == 1'b1) ? 1'b1 : 1'b0;

/*********************************************************
//buffer broken up flits
*********************************************************/
assign buffered_data = {data_from_chip_fff, channel_buffer};

always @(posedge wr_clk) begin
    data_from_chip_f <= data_from_chip;
    data_from_chip_ff <= data_from_chip_f;
    data_from_chip_fff <= data_from_chip_ff;
end
always @(posedge wr_clk) begin
    if(wr_rst_ff) begin
        buffered_channel <= 2'd0;
        channel_buffer_count <= 0;
        data_channel_f <= 2'd0;
        data_channel_ff <= 2'd0;
        data_channel_fff <= 2'd0;
    end
    else begin
        if(!channel_buffer_count && data_channel_fff != 0) begin //do not store data if no channel is assigned
            channel_buffer <= data_from_chip_fff;
            buffered_channel <= data_channel_fff;
            channel_buffer_count <= channel_buffer_count + 1'b1;
        end
        else if (channel_buffer_count && buffered_channel == data_channel_fff) begin
            channel_buffer_count <= 1'b0;
            buffered_channel <= 2'b00;
        end
        data_channel_f <= data_channel;
        data_channel_ff <= data_channel_f;
        data_channel_fff <= data_channel_ff;
    end
end

//input-output fixed to 64 bits
/*********************************************************
//data FIFOs
*********************************************************/

wire          bout_val_pre1;
wire          bout_val_pre2;
wire          bout_val_pre3;
reg           bout_val_buf1;
reg           bout_val_buf2;
reg           bout_val_buf3;

//data is read out a cycle later after the rd_en signal, so the interface is differd_ent than val/rdy


wire fifo1_empty;














    async_fifo #(
    .DSIZE(64),
    .ASIZE(9),
    .MEMSIZE(256) )
    async_fifo_1(
        .rreset(rd_rst_ff),
        .wreset(wr_rst_ff),
        .wclk(wr_clk),
        .rclk(rd_clk),
        .ren(bout_rdy_1 & ~credit_fifo_full),
        .wval(sort_val_1),
        .wdata(sort_data_1),
        .rdata(bout_data_1),
        .wfull(fifo1_full), //credit system should prevent this from ever being full
        .rempty(fifo1_empty)
    );



assign bout_val_1 = ~fifo1_empty & ~credit_fifo_full;


wire fifo2_empty;














    async_fifo #(
    .DSIZE(64),
    .ASIZE(9),
    .MEMSIZE(256) )
    async_fifo_2(
        .rreset(rd_rst_ff),
        .wreset(wr_rst_ff),
        .wclk(wr_clk),
        .rclk(rd_clk),
        .ren(bout_rdy_2 & ~credit_fifo_full),
        .wval(sort_val_2),
        .wdata(sort_data_2),
        .rdata(bout_data_2),
        .wfull(fifo2_full), //credit system should prevent this from ever being full
        .rempty(fifo2_empty)
    );



assign bout_val_2 = ~fifo2_empty & ~credit_fifo_full;

wire fifo3_empty;














    async_fifo #(
    .DSIZE(64),
    .ASIZE(9),
    .MEMSIZE(256) )
    async_fifo_3(
        .rreset(rd_rst_ff),
        .wreset(wr_rst_ff),
        .wclk(wr_clk),
        .rclk(rd_clk),
        .ren(bout_rdy_3 & ~credit_fifo_full),
        .wval(sort_val_3),
        .wdata(sort_data_3),
        .rdata(bout_data_3),
        .wfull(fifo3_full), //credit system should prevent this from ever being full
        .rempty(fifo3_empty)
    );


assign bout_val_3 = ~fifo3_empty & ~credit_fifo_full;




/*********************************************************
//credit FIFO
*********************************************************/

wire [2:0] credit_gather;
wire credit_val;
wire [2:0] credit_fifo_out;

reg  [2:0] credit_to_chip_r /* synthesis iob = true */;

// mmckeown: adding two pipeline register after credit fifo valid signal for timing
reg  [2:0] credit_fifo_out_f;

//FIFO used for bridging credits between clock domains


wire credit_empty;














    async_fifo #(
    .DSIZE(3),
    .ASIZE(11),
    .MEMSIZE(1024) )
    async_credit_fifo(
        .rreset(credit_rd_rst_ff),
        .wreset(rd_rst_ff),
        .wclk(rd_clk),
        .rclk(credit_rd_clk),
        .ren(~credit_rd_rst_ff),
        .wval(~rd_rst_ff & (| credit_gather)),
        .wdata(credit_gather),
        .rdata(credit_fifo_out),
        .wfull(credit_fifo_full),   
        .rempty(credit_empty)
    );


assign credit_val = ~ credit_empty;


assign credit_to_chip = credit_to_chip_r;

assign credit_gather[0] = bout_val_1 & bout_rdy_1;
assign credit_gather[1] = bout_val_2 & bout_rdy_2;
assign credit_gather[2] = bout_val_3 & bout_rdy_3;

/****************************************************
 channel side sequential logic 
 ****************************************************/

// mmckeown: adding pipeline register after credit fifo valid signal for timing
always@(posedge credit_rd_clk) begin
   if(credit_rd_rst_ff) begin
        credit_to_chip_r <= 3'b000;
        credit_fifo_out_f <= 3'b000;
   end
   else begin
       credit_to_chip_r <= credit_fifo_out_f;
       if(credit_val) begin 
           credit_fifo_out_f <= credit_fifo_out;
       end
       else
           credit_fifo_out_f <= 3'b000;
   end 
end

endmodule
