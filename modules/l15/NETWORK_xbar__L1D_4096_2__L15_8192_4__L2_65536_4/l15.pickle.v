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








































































































































































































































































































































































































































































































































































































module sram_l15_data
(
input wire MEMCLK,
input wire RESET_N,
input wire CE,
input wire [9-1:0] A,
input wire RDWEN,
input wire [128-1:0] BW,
input wire [128-1:0] DIN,
output wire [128-1:0] DOUT,
input wire [4-1:0] BIST_COMMAND,
input wire [4-1:0] BIST_DIN,
output reg [4-1:0] BIST_DOUT,
input wire [8-1:0] SRAMID
);


wire [128-1:0] DOUT_bram;
assign DOUT = DOUT_bram;

bram_1rw_wrapper #(
   .NAME          (""             ),
   .DEPTH         (512),
   .ADDR_WIDTH    (9),
   .BITMASK_WIDTH (128),
   .DATA_WIDTH    (128)
)   sram_l15_data (
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








































































































































































































































































































































































































































































































































































































module sram_l15_hmt
(
input wire MEMCLK,
input wire RESET_N,
input wire CE,
input wire [9-1:0] A,
input wire RDWEN,
input wire [32-1:0] BW,
input wire [32-1:0] DIN,
output wire [32-1:0] DOUT,
input wire [4-1:0] BIST_COMMAND,
input wire [4-1:0] BIST_DIN,
output reg [4-1:0] BIST_DOUT,
input wire [8-1:0] SRAMID
);


wire [32-1:0] DOUT_bram;
assign DOUT = DOUT_bram;

bram_1rw_wrapper #(
   .NAME          (""             ),
   .DEPTH         (512),
   .ADDR_WIDTH    (9),
   .BITMASK_WIDTH (32),
   .DATA_WIDTH    (32)
)   sram_l15_hmt (
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








































































































































































































































































































































































































































































































































































































module sram_l15_tag
(
input wire MEMCLK,
input wire RESET_N,
input wire CE,
input wire [((9-2))-1:0] A,
input wire RDWEN,
input wire [33*4-1:0] BW,
input wire [33*4-1:0] DIN,
output wire [33*4-1:0] DOUT,
input wire [4-1:0] BIST_COMMAND,
input wire [4-1:0] BIST_DIN,
output reg [4-1:0] BIST_DOUT,
input wire [8-1:0] SRAMID
);


wire [33*4-1:0] DOUT_bram;
assign DOUT = DOUT_bram;

bram_1rw_wrapper #(
   .NAME          (""             ),
   .DEPTH         ((512/4)),
   .ADDR_WIDTH    (((9-2))),
   .BITMASK_WIDTH (33*4),
   .DATA_WIDTH    (33*4)
)   sram_l15_tag (
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
//  Filename      : l15.v
//  Created On    : 2014-01-31 12:52:57
//  Last Modified : 2015-01-22 17:39:39
//  Revision      :
//  Author        : Tri Nguyen
//  Company       : Princeton University
//  Email         : trin@princeton.edu
//
//  Description   : L1.5 top module
//
//
//==================================================================================================

//`timescale 1 ns / 10 ps
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





module l15 (
    input                                   clk,
    input                                   rst_n,
    
    input [4:0]                             transducer_l15_rqtype,
    input [4-1:0]           transducer_l15_amo_op,
    input                                   transducer_l15_nc,
    input [2:0]                             transducer_l15_size,
    input [0:0]              transducer_l15_threadid,
    input                                   transducer_l15_prefetch,
    input                                   transducer_l15_invalidate_cacheline,
    input                                   transducer_l15_blockstore,
    input                                   transducer_l15_blockinitstore,
    input [1:0]                             transducer_l15_l1rplway,
    input                                   transducer_l15_val,
    input [39:0]                            transducer_l15_address,
    input [63:0]                            transducer_l15_data,
    input [63:0]                            transducer_l15_data_next_entry,
    input [33-1:0]              transducer_l15_csm_data,

    output                                  l15_transducer_ack,
    output                                  l15_transducer_header_ack,

    output                                  l15_transducer_val,
    output [3:0]                            l15_transducer_returntype,
    output                                  l15_transducer_l2miss,
    output [1:0]                            l15_transducer_error,
    output                                  l15_transducer_noncacheable,
    output                                  l15_transducer_atomic,
    output [0:0]             l15_transducer_threadid,
    output                                  l15_transducer_prefetch,
    output                                  l15_transducer_f4b,
    output [63:0]                           l15_transducer_data_0,
    output [63:0]                           l15_transducer_data_1,
    output [63:0]                           l15_transducer_data_2,
    output [63:0]                           l15_transducer_data_3,
    output                                  l15_transducer_inval_icache_all_way,
    output                                  l15_transducer_inval_dcache_all_way,
    output [15:4]                           l15_transducer_inval_address_15_4,
    output                                  l15_transducer_cross_invalidate,
    output [1:0]                            l15_transducer_cross_invalidate_way,
    output                                  l15_transducer_inval_dcache_inval,
    output                                  l15_transducer_inval_icache_inval,
    output [1:0]                            l15_transducer_inval_way,
    output                                  l15_transducer_blockinitstore,

    input                                   transducer_l15_req_ack,

    input                                   noc1_out_rdy,
    input                                   noc2_in_val,
    input [64-1:0]             noc2_in_data,
    input                                   noc3_out_rdy,
    input                                   dmbr_l15_stall,
    input [14-1:0]           chipid,
    input [8-1:0]                coreid_x,
    input [8-1:0]                coreid_y,

    // input from config registers to pipeline
    input [63:0]                            config_l15_read_res_data_s3,
    input                                   config_csm_en,
    input [31:0]                            config_system_tile_count,
    input [2-1:0]    config_home_alloc_method, 
    input [22-1:0]    config_hmt_base,

    output                                  noc1_out_val,
    output [64-1:0]            noc1_out_data,
    output                                  noc2_in_rdy,
    output                                  noc3_out_val,
    output [64-1:0]            noc3_out_data,
    output                                  l15_dmbr_l1missIn,
    output [4-1:0]            l15_dmbr_l1missTag,
    output                                  l15_dmbr_l2responseIn,
    output                                  l15_dmbr_l2missIn,
    output [4-1:0]            l15_dmbr_l2missTag,

    // output to config registers to pipeline
    output                                  l15_config_req_val_s2,
    output                                  l15_config_req_rw_s2,
    output [63:0]                           l15_config_write_req_data_s2,
    output [15:8]       l15_config_req_address_s2,

    // sram interface
    output [4-1:0]    srams_rtap_data,
    input  [4-1:0]             rtap_srams_bist_command,
    input  [4-1:0]    rtap_srams_bist_data

);

// assigning sram return data
wire [4-1:0] dtag_rtap_data;
wire [4-1:0] dcache_rtap_data;


wire [4-1:0] hmt_rtap_data;
assign srams_rtap_data = dtag_rtap_data
                            | dcache_rtap_data
                            | hmt_rtap_data;





///////////////////////////////////
// CSM module
///////////////////////////////////

wire [40-1:0] l15_csm_req_address_s2;
wire l15_csm_req_val_s2;
wire l15_csm_stall_s3;
wire [3-1:0] l15_csm_req_ticket_s2;
// wire [`HOME_ID_WIDTH-1:0] l15_csm_clump_tile_count_s2;
wire  l15_csm_req_type_s2;
wire [127:0] l15_csm_req_data_s2;
wire  [33-1:0] l15_csm_req_pcx_data_s2;
wire csm_l15_res_val_s3;
wire [63:0] csm_l15_res_data_s3;

// read port in case of a cache miss
wire [3-1:0] l15_csm_read_ticket;
wire [3-1:0] l15_csm_clear_ticket;
wire l15_csm_clear_ticket_val;
wire [(14+8+8)-1:0] csm_l15_read_res_data;
wire csm_l15_read_res_val;

// with noc1
wire noc1encoder_csm_req_ack;
wire csm_noc1encoder_req_val;
wire [5-1:0] csm_noc1encoder_req_type;
wire [3-1:0] csm_noc1encoder_req_mshrid;
wire [40-1:0] csm_noc1encoder_req_address;
wire csm_noc1encoder_req_non_cacheable;
wire  [3-1:0] csm_noc1encoder_req_size;

l15_csm l15_csm(
    .clk(clk),
    .rst_n(rst_n),
    
    // interface with noc1 buffer
    .l15_csm_read_ticket(l15_csm_read_ticket),
    .l15_csm_clear_ticket(l15_csm_clear_ticket),
    .l15_csm_clear_ticket_val(l15_csm_clear_ticket_val),
    .csm_l15_read_res_data(csm_l15_read_res_data),
    .csm_l15_read_res_val(csm_l15_read_res_val),
    
    // config regs
    .l15_hmt_base_reg(config_hmt_base),
    .csm_en(config_csm_en),
    .system_tile_count(config_system_tile_count[6-1:0]),
    .home_alloc_method(config_home_alloc_method),
    
    // interface with pipeline
    .l15_csm_req_address_s2(l15_csm_req_address_s2),
    .l15_csm_req_val_s2(l15_csm_req_val_s2),
    .l15_csm_stall_s3(l15_csm_stall_s3),
    .l15_csm_req_ticket_s2(l15_csm_req_ticket_s2),
    //.l15_csm_clump_tile_count_s2(l15_csm_clump_tile_count_s2),
    .l15_csm_req_type_s2(l15_csm_req_type_s2),
    .l15_csm_req_data_s2(l15_csm_req_data_s2),
    .l15_csm_req_pcx_data_s2(l15_csm_req_pcx_data_s2),
    .csm_l15_res_val_s3(csm_l15_res_val_s3),
    .csm_l15_res_data_s3(csm_l15_res_data_s3),
    
    // interface with noc1
    .noc1encoder_csm_req_ack(noc1encoder_csm_req_ack),
    .csm_noc1encoder_req_val(csm_noc1encoder_req_val),
    .csm_noc1encoder_req_type(csm_noc1encoder_req_type),
    .csm_noc1encoder_req_mshrid(csm_noc1encoder_req_mshrid),
    .csm_noc1encoder_req_address(csm_noc1encoder_req_address),
    .csm_noc1encoder_req_non_cacheable(csm_noc1encoder_req_non_cacheable),
    .csm_noc1encoder_req_size(csm_noc1encoder_req_size)
);

/*
    NoC2 buffer gets all the flits of a packet before transmitting it to the L1.5
    TODO: optimize it so that full buffering is not needed,
            ie. we can have a header ready signal + data ready signal
*/

wire [511:0] noc2_data;
wire noc2_data_val;
wire noc2_data_ack;

simplenocbuffer simplenocbuffer(
    .clk(clk),
    .rst_n(rst_n),
    .noc_in_val(noc2_in_val),
    .noc_in_data(noc2_in_data),
    .msg_ack(noc2_data_ack),
    .noc_in_rdy(noc2_in_rdy),
    .msg(noc2_data),
    .msg_val(noc2_data_val)
);

wire l15_noc2decoder_ack;
wire l15_noc2decoder_header_ack;
wire noc2decoder_l15_val;
wire [2-1:0] noc2decoder_l15_mshrid;
wire noc2decoder_l15_l2miss;
wire noc2decoder_l15_icache_type;
wire noc2decoder_l15_f4b;
wire [8-1:0] noc2decoder_l15_reqtype;
wire [2-1:0] noc2decoder_l15_ack_state;
wire [63:0] noc2decoder_l15_data_0;
wire [63:0] noc2decoder_l15_data_1;
wire [63:0] noc2decoder_l15_data_2;
wire [63:0] noc2decoder_l15_data_3;
wire [39:0] noc2decoder_l15_address;
wire [3:0] noc2decoder_l15_fwd_subcacheline_vector;
wire [(14+8+8)-1:0] noc2decoder_l15_src_homeid;

wire [3-1:0] noc2decoder_l15_csm_mshrid;
wire [0:0] noc2decoder_l15_threadid;
wire noc2decoder_l15_hmc_fill;

/*
    noc2decoder takes the data from the buffer and decode it to meaningful signals
    to the l15
*/
noc2decoder noc2decoder(
    .clk(clk),
    .rst_n(rst_n),
    .noc2_data(noc2_data),
    .noc2_data_val(noc2_data_val),
    .l15_noc2decoder_ack(l15_noc2decoder_ack),
    .l15_noc2decoder_header_ack(l15_noc2decoder_header_ack),
    .noc2_data_ack(noc2_data_ack),
    .noc2decoder_l15_val(noc2decoder_l15_val),
    .noc2decoder_l15_mshrid(noc2decoder_l15_mshrid),
    .noc2decoder_l15_l2miss(noc2decoder_l15_l2miss),
    .noc2decoder_l15_icache_type(noc2decoder_l15_icache_type),
    .noc2decoder_l15_f4b(noc2decoder_l15_f4b),
    .noc2decoder_l15_reqtype(noc2decoder_l15_reqtype),
    .noc2decoder_l15_ack_state(noc2decoder_l15_ack_state),
    .noc2decoder_l15_data_0(noc2decoder_l15_data_0),
    .noc2decoder_l15_data_1(noc2decoder_l15_data_1),
    .noc2decoder_l15_data_2(noc2decoder_l15_data_2),
    .noc2decoder_l15_data_3(noc2decoder_l15_data_3),
    .noc2decoder_l15_address(noc2decoder_l15_address),
    .noc2decoder_l15_fwd_subcacheline_vector(noc2decoder_l15_fwd_subcacheline_vector),
    .noc2decoder_l15_src_homeid(noc2decoder_l15_src_homeid),
    .noc2decoder_l15_csm_mshrid(noc2decoder_l15_csm_mshrid),
    .noc2decoder_l15_threadid(noc2decoder_l15_threadid),
    .noc2decoder_l15_hmc_fill(noc2decoder_l15_hmc_fill),
    .l15_dmbr_l2missIn(l15_dmbr_l2missIn),
    .l15_dmbr_l2missTag(l15_dmbr_l2missTag),
    .l15_dmbr_l2responseIn(l15_dmbr_l2responseIn)
);

// noc1 signal declarations
wire noc1encoder_l15_req_ack;
wire noc1encoder_l15_req_sent;
wire l15_noc1buffer_req_val;
wire [2-1:0] noc1encoder_l15_req_data_sent;

wire [5-1:0] l15_noc1buffer_req_type;
wire [0:0] l15_noc1buffer_req_threadid;
wire [2-1:0] l15_noc1buffer_req_mshrid;
wire [39:0] l15_noc1buffer_req_address;
wire l15_noc1buffer_req_non_cacheable;
wire [2:0] l15_noc1buffer_req_size;
wire l15_noc1buffer_req_prefetch;
// wire l15_noc1buffer_req_blkstore;
// wire l15_noc1buffer_req_blkinitstore;
wire [63:0] l15_noc1buffer_req_data_0;
wire [63:0] l15_noc1buffer_req_data_1;
wire [33-1:0] l15_noc1buffer_req_csm_data;
// csm
wire [3-1:0] l15_noc1buffer_req_csm_ticket;
wire [(14+8+8)-1:0] l15_noc1buffer_req_homeid;
wire l15_noc1buffer_req_homeid_val;
wire [10-1:0] noc1buffer_noc1encoder_req_csm_sdid;
wire [6-1:0] noc1buffer_noc1encoder_req_csm_lsid;

wire [5-1:0] noc1buffer_noc1encoder_req_type;
wire [0:0] noc1buffer_noc1encoder_req_threadid;
wire [2-1:0] noc1buffer_noc1encoder_req_mshrid;
wire [39:0] noc1buffer_noc1encoder_req_address;
wire noc1buffer_noc1encoder_req_non_cacheable;
wire [2:0] noc1buffer_noc1encoder_req_size;
wire noc1buffer_noc1encoder_req_prefetch;
// wire noc1buffer_noc1encoder_req_blkstore;
// wire noc1buffer_noc1encoder_req_blkinitstore;
wire [63:0] noc1buffer_noc1encoder_req_data_0;
wire [63:0] noc1buffer_noc1encoder_req_data_1;
wire [(14+8+8)-1:0] noc1buffer_noc1encoder_req_homeid;

wire noc1encoder_noc1buffer_req_ack;
wire noc1buffer_noc1encoder_req_val;


// noc3 signal declarations
wire noc3encoder_l15_req_ack;
wire noc3encoder_noc3buffer_req_ack;

wire l15_noc3encoder_req_val;
wire noc3buffer_noc3encoder_req_val;
wire [3-1:0] l15_noc3encoder_req_type;
wire [63:0] l15_noc3encoder_req_data_0;
wire [63:0] l15_noc3encoder_req_data_1;
wire [2-1:0] l15_noc3encoder_req_mshrid;
wire [1:0] l15_noc3encoder_req_sequenceid;
wire [0:0] l15_noc3encoder_req_threadid;
wire [39:0] l15_noc3encoder_req_address;
wire l15_noc3encoder_req_with_data;
wire l15_noc3encoder_req_was_inval;
wire [3:0] l15_noc3encoder_req_fwdack_vector;
wire [(14+8+8)-1:0] l15_noc3encoder_req_homeid;

wire [3-1:0] noc3buffer_noc3encoder_req_type;
wire [63:0] noc3buffer_noc3encoder_req_data_0;
wire [63:0] noc3buffer_noc3encoder_req_data_1;
wire [2-1:0] noc3buffer_noc3encoder_req_mshrid;
wire [1:0] noc3buffer_noc3encoder_req_sequenceid;
wire [0:0] noc3buffer_noc3encoder_req_threadid;
wire [39:0] noc3buffer_noc3encoder_req_address;
wire noc3buffer_noc3encoder_req_with_data;
wire noc3buffer_noc3encoder_req_was_inval;
wire [3:0] noc3buffer_noc3encoder_req_fwdack_vector;
wire [(14+8+8)-1:0] noc3buffer_noc3encoder_req_homeid;



// DTAG
wire l15_dtag_val_s1;
wire l15_dtag_rw_s1;
wire [((9-2))-1:0] l15_dtag_index_s1;
wire [33*4-1:0] l15_dtag_write_data_s1;
wire [33*4-1:0] l15_dtag_write_mask_s1;
wire [33*4-1:0] dtag_l15_dout_s2;

sram_l15_tag dtag(
    .MEMCLK(clk),
    .RESET_N(rst_n),
    .CE(l15_dtag_val_s1),
    .A(l15_dtag_index_s1),
    .DIN(l15_dtag_write_data_s1),
    .BW(l15_dtag_write_mask_s1),
    .RDWEN(l15_dtag_rw_s1),
    .DOUT(dtag_l15_dout_s2),
    .BIST_COMMAND(rtap_srams_bist_command),
    .BIST_DIN(rtap_srams_bist_data),
    .BIST_DOUT(dtag_rtap_data),
    .SRAMID(8'd6)
);

// DCACHE
wire l15_dcache_val_s2;
wire l15_dcache_rw_s2;
wire [(((9-2))+2)-1:0] l15_dcache_index_s2;
wire [127:0] l15_dcache_write_data_s2;
wire [127:0] l15_dcache_write_mask_s2;
wire [127:0] dcache_l15_dout_s3;



wire [14 + 8 + 8-1:0] l15_hmt_write_data_s2;
wire [14 + 8 + 8-1:0] l15_hmt_write_mask_s2;
wire [14 + 8 + 8-1:0] hmt_l15_dout_s3;


// sram_1rw_512x128 dcache(
sram_l15_data dcache(
    .MEMCLK(clk),
    .RESET_N(rst_n),
    .CE(l15_dcache_val_s2),
    .A(l15_dcache_index_s2),
    .DIN({l15_dcache_write_data_s2}),
    .BW({l15_dcache_write_mask_s2}),
    .RDWEN(l15_dcache_rw_s2),
    .DOUT({dcache_l15_dout_s3}),
    .BIST_COMMAND(rtap_srams_bist_command),
    .BIST_DIN(rtap_srams_bist_data),
    .BIST_DOUT(dcache_rtap_data),
    .SRAMID(8'd7)
);

// wire [127:0] l15_hmt_write_data_s2_extended = l15_hmt_write_data_s2;
// wire [127:0] l15_hmt_write_mask_s2_extended = l15_hmt_write_mask_s2;
// wire [127:0] hmt_l15_dout_s3_extended;
// assign hmt_l15_dout_s3 = hmt_l15_dout_s3_extended;
// // home map table, is supposed to be merged with dcache but for now use this
// sram_1rw_512x128 hmt(
//    .MEMCLK(clk),
//    .RESET_N(rst_n),
//    .CE(l15_dcache_val_s2),
//    .A(l15_dcache_index_s2),
//    .DIN(l15_hmt_write_data_s2_extended),
//    .BW(l15_hmt_write_mask_s2_extended),
//    .RDWEN(l15_dcache_rw_s2),

//    .TESTEN(1'b0),
//    .TA(l15_dcache_index_s2),
//    .TDIN(l15_dcache_write_data_s2),
//    .TBW(l15_dcache_write_mask_s2),
//    .TRDWEN(l15_dcache_rw_s2),

//    .FUSE(`BIST_FUSE_WIDTH'b0),

//    .DOUT(hmt_l15_dout_s3_extended),
//    .TDOUT()
//    );


wire [31:0] l15_hmt_write_data_s2_extended = l15_hmt_write_data_s2;
wire [31:0] l15_hmt_write_mask_s2_extended = l15_hmt_write_mask_s2;
wire [31:0] hmt_l15_dout_s3_extended;
assign hmt_l15_dout_s3 = hmt_l15_dout_s3_extended[14 + 8 + 8-1:0];
// sram_1rw_512x32 hmt(
sram_l15_hmt hmt(
    .MEMCLK(clk),
    .RESET_N(rst_n),
    .CE(l15_dcache_val_s2),
    .A(l15_dcache_index_s2),
    .DIN(l15_hmt_write_data_s2_extended),
    .BW(l15_hmt_write_mask_s2_extended),
    .RDWEN(l15_dcache_rw_s2),
    .DOUT(hmt_l15_dout_s3_extended),
    .BIST_COMMAND(rtap_srams_bist_command),
    .BIST_DIN(rtap_srams_bist_data),
    .BIST_DOUT(hmt_rtap_data),
    .SRAMID(8'd8)
);



// MSHR
wire pipe_mshr_writereq_val_s1;
wire [3-1:0] pipe_mshr_writereq_op_s1;
wire [39:0] pipe_mshr_writereq_address_s1;
wire [127:0] pipe_mshr_writereq_write_buffer_data_s1;
wire [15:0] pipe_mshr_writereq_write_buffer_byte_mask_s1;
wire [((((((((((0 + 1) + 1) + 1) + 1) + 3) + 1) + 2) + 1) + 1) + 1)-1:0] pipe_mshr_writereq_control_s1;
wire [2-1:0] pipe_mshr_writereq_mshrid_s1;
wire [0:0] pipe_mshr_writereq_threadid_s1;
wire [0:0] pipe_mshr_readreq_threadid_s1;
wire [2-1:0] pipe_mshr_readreq_mshrid_s1;
wire [((((((((((0 + 1) + 1) + 1) + 1) + 3) + 1) + 2) + 1) + 1) + 1)-1:0] mshr_pipe_readres_control_s1;
wire [(14+8+8)-1:0] mshr_pipe_readres_homeid_s1;
wire [(4*2)-1:0] mshr_pipe_vals_s1;
wire [(40*2)-1:0] mshr_pipe_ld_address;
wire [(40*2)-1:0] mshr_pipe_st_address;
wire [(2*2)-1:0] mshr_pipe_st_way_s1;
wire [(2*2)-1:0] mshr_pipe_st_state_s1;
wire pipe_mshr_write_buffer_rd_en_s2;
wire [0:0] pipe_mshr_threadid_s2;
wire [127:0]mshr_pipe_write_buffer_s2;
wire [15:0] mshr_pipe_write_buffer_byte_mask_s2;
wire pipe_mshr_val_s3;
wire [3-1:0] pipe_mshr_op_s3;
wire [2-1:0] pipe_mshr_mshrid_s3;
wire [0:0] pipe_mshr_threadid_s3;
wire [2-1:0] pipe_mshr_write_update_state_s3;
wire [1:0] pipe_mshr_write_update_way_s3;

wire noc1buffer_mshr_homeid_write_val_s4;
wire [2-1:0] noc1buffer_mshr_homeid_write_mshrid_s4;
wire [(14+8+8)-1:0] noc1buffer_mshr_homeid_write_data_s4;
wire [0:0] noc1buffer_mshr_homeid_write_threadid_s4;

l15_mshr mshr(
    .clk(clk),
    .rst_n(rst_n),
    .pipe_mshr_writereq_val_s1(pipe_mshr_writereq_val_s1),
    .pipe_mshr_writereq_op_s1(pipe_mshr_writereq_op_s1),
    .pipe_mshr_writereq_address_s1(pipe_mshr_writereq_address_s1),
    .pipe_mshr_writereq_write_buffer_data_s1(pipe_mshr_writereq_write_buffer_data_s1),
    .pipe_mshr_writereq_write_buffer_byte_mask_s1(pipe_mshr_writereq_write_buffer_byte_mask_s1),
    .pipe_mshr_writereq_control_s1(pipe_mshr_writereq_control_s1),
    .pipe_mshr_writereq_mshrid_s1(pipe_mshr_writereq_mshrid_s1),
    .pipe_mshr_writereq_threadid_s1(pipe_mshr_writereq_threadid_s1),
    .pipe_mshr_readreq_threadid_s1(pipe_mshr_readreq_threadid_s1),
    .pipe_mshr_readreq_mshrid_s1(pipe_mshr_readreq_mshrid_s1),
    .mshr_pipe_readres_control_s1(mshr_pipe_readres_control_s1),
    .mshr_pipe_readres_homeid_s1(mshr_pipe_readres_homeid_s1),
    .mshr_pipe_vals_s1(mshr_pipe_vals_s1),
    .mshr_pipe_ld_address(mshr_pipe_ld_address),
    .mshr_pipe_st_address(mshr_pipe_st_address),
    .mshr_pipe_st_way_s1(mshr_pipe_st_way_s1),
    .mshr_pipe_st_state_s1(mshr_pipe_st_state_s1),
    .pipe_mshr_write_buffer_rd_en_s2(pipe_mshr_write_buffer_rd_en_s2),
    .pipe_mshr_threadid_s2(pipe_mshr_threadid_s2),
    .mshr_pipe_write_buffer_s2(mshr_pipe_write_buffer_s2),
    .mshr_pipe_write_buffer_byte_mask_s2(mshr_pipe_write_buffer_byte_mask_s2),
    .pipe_mshr_val_s3(pipe_mshr_val_s3),
    .pipe_mshr_op_s3(pipe_mshr_op_s3),
    .pipe_mshr_mshrid_s3(pipe_mshr_mshrid_s3),
    .pipe_mshr_threadid_s3(pipe_mshr_threadid_s3),
    .pipe_mshr_write_update_state_s3(pipe_mshr_write_update_state_s3),
    .pipe_mshr_write_update_way_s3(pipe_mshr_write_update_way_s3),
    
    .noc1buffer_mshr_homeid_write_threadid_s4(noc1buffer_mshr_homeid_write_threadid_s4),
    .noc1buffer_mshr_homeid_write_val_s4(noc1buffer_mshr_homeid_write_val_s4),
    .noc1buffer_mshr_homeid_write_mshrid_s4(noc1buffer_mshr_homeid_write_mshrid_s4),
    .noc1buffer_mshr_homeid_write_data_s4(noc1buffer_mshr_homeid_write_data_s4)
);


// MESI array
wire l15_mesi_read_val_s1;
wire [((9-2))-1:0] l15_mesi_read_index_s1;
wire l15_mesi_write_val_s2;
wire [((9-2))-1:0] l15_mesi_write_index_s2;
wire [7:0] l15_mesi_write_mask_s2;
wire [7:0] l15_mesi_write_data_s2;
wire [7:0] mesi_l15_dout_s2;

rf_l15_mesi mesi(
    .clk(clk),
    .rst_n(rst_n),
    .read_valid(l15_mesi_read_val_s1),
    .read_index(l15_mesi_read_index_s1),
    .write_valid(l15_mesi_write_val_s2),
    .write_index(l15_mesi_write_index_s2),
    .write_mask(l15_mesi_write_mask_s2),
    .write_data(l15_mesi_write_data_s2),
    .read_data(mesi_l15_dout_s2)
);

// LRSC Flag array
wire l15_lrsc_flag_read_val_s1;
wire [((9-2))-1:0] l15_lrsc_flag_read_index_s1;
wire l15_lrsc_flag_write_val_s2;
wire [((9-2))-1:0] l15_lrsc_flag_write_index_s2;
wire [3:0] l15_lrsc_flag_write_mask_s2;
wire [3:0] l15_lrsc_flag_write_data_s2;
wire [3:0] lrsc_flag_l15_dout_s2;

rf_l15_lrsc_flag lrsc_flag(
    .clk(clk),
    .rst_n(rst_n),
    .read_valid(l15_lrsc_flag_read_val_s1),
    .read_index(l15_lrsc_flag_read_index_s1),
    .write_valid(l15_lrsc_flag_write_val_s2),
    .write_index(l15_lrsc_flag_write_index_s2),
    .write_mask(l15_lrsc_flag_write_mask_s2),
    .write_data(l15_lrsc_flag_write_data_s2),
    .read_data(lrsc_flag_l15_dout_s2)
);

// // home map table array
// wire l15_hmt_read_val_s1;
// wire [8:0] l15_hmt_read_index_s1;
// wire l15_hmt_write_val_s2;
// wire [8:0] l15_hmt_write_index_s2;
// wire [31:0] l15_hmt_write_mask_s2;
// wire [31:0] l15_hmt_write_data_s2;
// wire [31:0] hmt_l15_dout_s2;

// l15_rf_512x32 hmt(
//    .clk(clk),
//    .rst_n(rst_n),
//    .read_valid(l15_hmt_read_val_s1),
//    .read_index(l15_hmt_read_index_s1),
//    .write_valid(l15_hmt_write_val_s2),
//    .write_index(l15_hmt_write_index_s2),
//    .write_mask(l15_hmt_write_mask_s2),
//    .write_data(l15_hmt_write_data_s2),
//    .read_data(hmt_l15_dout_s2)
//    );



// way map table
wire l15_wmt_read_val_s2;
wire [6:0] l15_wmt_read_index_s2;
wire l15_wmt_write_val_s3;
wire [6:0] l15_wmt_write_index_s3;
wire [2*((2+0)+1)-1:0] l15_wmt_write_mask_s3;
wire [2*((2+0)+1)-1:0] l15_wmt_write_data_s3;
wire [2*((2+0)+1)-1:0] wmt_l15_data_s3;
rf_l15_wmt wmc(
    .clk(clk),
    .rst_n(rst_n),
    .read_valid(l15_wmt_read_val_s2),
    .read_index(l15_wmt_read_index_s2),
    .write_valid(l15_wmt_write_val_s3),
    .write_index(l15_wmt_write_index_s3),
    .write_mask(l15_wmt_write_mask_s3),
    .write_data(l15_wmt_write_data_s3),
    .read_data(wmt_l15_data_s3)
);

// lru array, psuedo
wire l15_lruarray_read_val_s1;
wire [((9-2))-1:0] l15_lruarray_read_index_s1;
wire l15_lruarray_write_val_s3;
wire [((9-2))-1:0] l15_lruarray_write_index_s3;
wire [5:0] l15_lruarray_write_mask_s3;
wire [5:0] l15_lruarray_write_data_s3;
wire [5:0] lruarray_l15_dout_s2;
rf_l15_lruarray lruarray(
    .clk(clk),
    .rst_n(rst_n),
    .read_valid(l15_lruarray_read_val_s1),
    .read_index(l15_lruarray_read_index_s1),
    .write_valid(l15_lruarray_write_val_s3),
    .write_index(l15_lruarray_write_index_s3),
    .write_mask(l15_lruarray_write_mask_s3),
    .write_data(l15_lruarray_write_data_s3),
    .read_data(lruarray_l15_dout_s2)
);

// pipeline
l15_pipeline pipeline(
    .clk(clk),
    .rst_n(rst_n),
    .dtag_l15_dout_s2(dtag_l15_dout_s2),
    .dcache_l15_dout_s3(dcache_l15_dout_s3),
    .mesi_l15_dout_s2(mesi_l15_dout_s2),
    .lrsc_flag_l15_dout_s2(lrsc_flag_l15_dout_s2),
    .lruarray_l15_dout_s2(lruarray_l15_dout_s2),
    .wmt_l15_data_s3(wmt_l15_data_s3),
    .pcxdecoder_l15_rqtype               (transducer_l15_rqtype),
    .pcxdecoder_l15_amo_op               (transducer_l15_amo_op),
    .pcxdecoder_l15_nc                   (transducer_l15_nc),
    .pcxdecoder_l15_size                 (transducer_l15_size),
    // .pcxdecoder_l15_invalall          (transducer_l15_invalall),
    .pcxdecoder_l15_threadid             (transducer_l15_threadid),
    .pcxdecoder_l15_prefetch             (transducer_l15_prefetch),
    .pcxdecoder_l15_blockstore           (transducer_l15_blockstore),
    .pcxdecoder_l15_blockinitstore       (transducer_l15_blockinitstore),
    .pcxdecoder_l15_l1rplway             (transducer_l15_l1rplway),
    .pcxdecoder_l15_val                  (transducer_l15_val),
    .pcxdecoder_l15_invalidate_cacheline (transducer_l15_invalidate_cacheline),
    .pcxdecoder_l15_address              (transducer_l15_address),
    .pcxdecoder_l15_data                 (transducer_l15_data),
    .pcxdecoder_l15_data_next_entry      (transducer_l15_data_next_entry),
    .pcxdecoder_l15_csm_data             (transducer_l15_csm_data),
    .noc2decoder_l15_val(noc2decoder_l15_val),
    .noc2decoder_l15_mshrid(noc2decoder_l15_mshrid),
    .noc2decoder_l15_l2miss(noc2decoder_l15_l2miss),
    .noc2decoder_l15_icache_type(noc2decoder_l15_icache_type),
    .noc2decoder_l15_f4b(noc2decoder_l15_f4b),
    .noc2decoder_l15_reqtype(noc2decoder_l15_reqtype),
    .noc2decoder_l15_ack_state(noc2decoder_l15_ack_state),
    .noc2decoder_l15_data_0(noc2decoder_l15_data_0),
    .noc2decoder_l15_data_1(noc2decoder_l15_data_1),
    .noc2decoder_l15_data_2(noc2decoder_l15_data_2),
    .noc2decoder_l15_data_3(noc2decoder_l15_data_3),
    .noc2decoder_l15_address(noc2decoder_l15_address),
    .noc2decoder_l15_fwd_subcacheline_vector(noc2decoder_l15_fwd_subcacheline_vector),
    .noc2decoder_l15_src_homeid(noc2decoder_l15_src_homeid),
    .noc2decoder_l15_csm_mshrid(noc2decoder_l15_csm_mshrid),
    .noc2decoder_l15_threadid(noc2decoder_l15_threadid),
    .noc2decoder_l15_hmc_fill(noc2decoder_l15_hmc_fill),
    .cpxencoder_l15_req_ack(transducer_l15_req_ack),
    // .noc1encoder_l15_req_ack(noc1encoder_l15_req_ack),
    .noc1encoder_l15_req_sent(noc1encoder_l15_req_sent),
    .noc1encoder_l15_req_data_sent(noc1encoder_l15_req_data_sent),
    .noc3encoder_l15_req_ack(noc3encoder_l15_req_ack),
    // .chipid(chipid),
    // .coreid_x(coreid_x),
    // .coreid_y(coreid_y),
    
    // OUTPUT
    .l15_dtag_val_s1(l15_dtag_val_s1),
    .l15_dtag_rw_s1(l15_dtag_rw_s1),
    .l15_dtag_index_s1(l15_dtag_index_s1),
    .l15_dtag_write_data_s1(l15_dtag_write_data_s1),
    .l15_dtag_write_mask_s1(l15_dtag_write_mask_s1),
    .l15_dcache_val_s2(l15_dcache_val_s2),
    .l15_dcache_rw_s2(l15_dcache_rw_s2),
    .l15_dcache_index_s2(l15_dcache_index_s2),
    .l15_dcache_write_data_s2(l15_dcache_write_data_s2),
    .l15_dcache_write_mask_s2(l15_dcache_write_mask_s2),
    .l15_mesi_read_val_s1(l15_mesi_read_val_s1),
    .l15_mesi_read_index_s1(l15_mesi_read_index_s1),
    .l15_mesi_write_val_s2(l15_mesi_write_val_s2),
    .l15_mesi_write_index_s2(l15_mesi_write_index_s2),
    .l15_mesi_write_mask_s2(l15_mesi_write_mask_s2),
    .l15_mesi_write_data_s2(l15_mesi_write_data_s2),
    .l15_lrsc_flag_read_val_s1(l15_lrsc_flag_read_val_s1),
    .l15_lrsc_flag_read_index_s1(l15_lrsc_flag_read_index_s1),
    .l15_lrsc_flag_write_val_s2(l15_lrsc_flag_write_val_s2),
    .l15_lrsc_flag_write_index_s2(l15_lrsc_flag_write_index_s2),
    .l15_lrsc_flag_write_mask_s2(l15_lrsc_flag_write_mask_s2),
    .l15_lrsc_flag_write_data_s2(l15_lrsc_flag_write_data_s2),
    .l15_wmt_read_val_s2(l15_wmt_read_val_s2),
    .l15_wmt_read_index_s2(l15_wmt_read_index_s2),
    .l15_wmt_write_val_s3(l15_wmt_write_val_s3),
    .l15_wmt_write_index_s3(l15_wmt_write_index_s3),
    .l15_wmt_write_mask_s3(l15_wmt_write_mask_s3),
    .l15_wmt_write_data_s3(l15_wmt_write_data_s3),
    .l15_lruarray_read_val_s1(l15_lruarray_read_val_s1),
    .l15_lruarray_read_index_s1(l15_lruarray_read_index_s1),
    .l15_lruarray_write_val_s3(l15_lruarray_write_val_s3),
    .l15_lruarray_write_index_s3(l15_lruarray_write_index_s3),
    .l15_lruarray_write_mask_s3(l15_lruarray_write_mask_s3),
    .l15_lruarray_write_data_s3(l15_lruarray_write_data_s3),
    .l15_cpxencoder_val                  (l15_transducer_val),
    .l15_cpxencoder_returntype           (l15_transducer_returntype),
    .l15_cpxencoder_l2miss               (l15_transducer_l2miss),
    .l15_cpxencoder_error                (l15_transducer_error),
    .l15_cpxencoder_noncacheable         (l15_transducer_noncacheable),
    .l15_cpxencoder_atomic               (l15_transducer_atomic),
    .l15_cpxencoder_threadid             (l15_transducer_threadid),
    .l15_cpxencoder_prefetch             (l15_transducer_prefetch),
    .l15_cpxencoder_f4b                  (l15_transducer_f4b),
    .l15_cpxencoder_data_0               (l15_transducer_data_0),
    .l15_cpxencoder_data_1               (l15_transducer_data_1),
    .l15_cpxencoder_data_2               (l15_transducer_data_2),
    .l15_cpxencoder_data_3               (l15_transducer_data_3),
    .l15_cpxencoder_inval_icache_all_way (l15_transducer_inval_icache_all_way),
    .l15_cpxencoder_inval_dcache_all_way (l15_transducer_inval_dcache_all_way),
    .l15_cpxencoder_inval_address_15_4   (l15_transducer_inval_address_15_4),
    .l15_cpxencoder_cross_invalidate     (l15_transducer_cross_invalidate),
    .l15_cpxencoder_cross_invalidate_way (l15_transducer_cross_invalidate_way),
    .l15_cpxencoder_inval_dcache_inval   (l15_transducer_inval_dcache_inval),
    .l15_cpxencoder_inval_icache_inval   (l15_transducer_inval_icache_inval),
    .l15_cpxencoder_inval_way            (l15_transducer_inval_way),
    .l15_cpxencoder_blockinitstore       (l15_transducer_blockinitstore),
    .l15_noc1buffer_req_val(l15_noc1buffer_req_val),
    .l15_noc1buffer_req_type(l15_noc1buffer_req_type),
    .l15_noc1buffer_req_threadid(l15_noc1buffer_req_threadid),
    .l15_noc1buffer_req_mshrid(l15_noc1buffer_req_mshrid),
    .l15_noc1buffer_req_address(l15_noc1buffer_req_address),
    .l15_noc1buffer_req_non_cacheable(l15_noc1buffer_req_non_cacheable),
    .l15_noc1buffer_req_size(l15_noc1buffer_req_size),
    .l15_noc1buffer_req_prefetch(l15_noc1buffer_req_prefetch),
    // .l15_noc1buffer_req_blkstore(l15_noc1buffer_req_blkstore),
    // .l15_noc1buffer_req_blkinitstore(l15_noc1buffer_req_blkinitstore),
    .l15_noc1buffer_req_data_0(l15_noc1buffer_req_data_0),
    .l15_noc1buffer_req_data_1(l15_noc1buffer_req_data_1),
    .l15_noc1buffer_req_csm_data(l15_noc1buffer_req_csm_data),
    .l15_noc3encoder_req_val(l15_noc3encoder_req_val),
    .l15_noc3encoder_req_type(l15_noc3encoder_req_type),
    .l15_noc3encoder_req_data_0(l15_noc3encoder_req_data_0),
    .l15_noc3encoder_req_data_1(l15_noc3encoder_req_data_1),
    .l15_noc3encoder_req_mshrid(l15_noc3encoder_req_mshrid),
    .l15_noc3encoder_req_sequenceid(l15_noc3encoder_req_sequenceid),
    .l15_noc3encoder_req_threadid(l15_noc3encoder_req_threadid),
    .l15_noc3encoder_req_address(l15_noc3encoder_req_address),
    .l15_noc3encoder_req_with_data(l15_noc3encoder_req_with_data),
    .l15_noc3encoder_req_was_inval(l15_noc3encoder_req_was_inval),
    .l15_noc3encoder_req_fwdack_vector(l15_noc3encoder_req_fwdack_vector),
    .l15_noc3encoder_req_homeid(l15_noc3encoder_req_homeid),
    .l15_pcxdecoder_ack(l15_transducer_ack),
    .l15_noc2decoder_ack(l15_noc2decoder_ack),
    .l15_pcxdecoder_header_ack(l15_transducer_header_ack),
    .l15_noc2decoder_header_ack(l15_noc2decoder_header_ack),
    
    // CSM
    .l15_csm_req_address_s2(l15_csm_req_address_s2),
    .l15_csm_req_val_s2(l15_csm_req_val_s2),
    .l15_csm_stall_s3(l15_csm_stall_s3),
    .l15_csm_req_ticket_s2(l15_csm_req_ticket_s2),
    // .l15_csm_clump_tile_count_s2(l15_csm_clump_tile_count_s2),
    .l15_csm_req_type_s2(l15_csm_req_type_s2),
    .l15_csm_req_data_s2(l15_csm_req_data_s2),
    .l15_csm_req_pcx_data_s2(l15_csm_req_pcx_data_s2),
    .csm_l15_res_val_s3(csm_l15_res_val_s3),
    .csm_l15_res_data_s3(csm_l15_res_data_s3),
    
    .l15_noc1buffer_req_csm_ticket(l15_noc1buffer_req_csm_ticket),
    .l15_noc1buffer_req_homeid(l15_noc1buffer_req_homeid),
    .l15_noc1buffer_req_homeid_val(l15_noc1buffer_req_homeid_val),
    
    // .config_csm_en(config_csm_en),
    
    // hmt
     
    .l15_hmt_write_data_s2(l15_hmt_write_data_s2),
    .l15_hmt_write_mask_s2(l15_hmt_write_mask_s2),
    .hmt_l15_dout_s3(hmt_l15_dout_s3),
     
    
    // config registers
    .l15_config_req_val_s2(l15_config_req_val_s2),
    .l15_config_req_rw_s2(l15_config_req_rw_s2),
    .l15_config_write_req_data_s2(l15_config_write_req_data_s2),
    .l15_config_req_address_s2(l15_config_req_address_s2),
    .config_l15_read_res_data_s3(config_l15_read_res_data_s3),
    
    // MSHR
    .pipe_mshr_writereq_val_s1(pipe_mshr_writereq_val_s1),
    .pipe_mshr_writereq_op_s1(pipe_mshr_writereq_op_s1),
    .pipe_mshr_writereq_address_s1(pipe_mshr_writereq_address_s1),
    .pipe_mshr_writereq_write_buffer_data_s1(pipe_mshr_writereq_write_buffer_data_s1),
    .pipe_mshr_writereq_write_buffer_byte_mask_s1(pipe_mshr_writereq_write_buffer_byte_mask_s1),
    .pipe_mshr_writereq_control_s1(pipe_mshr_writereq_control_s1),
    .pipe_mshr_writereq_mshrid_s1(pipe_mshr_writereq_mshrid_s1),
    .pipe_mshr_writereq_threadid_s1(pipe_mshr_writereq_threadid_s1),
    .pipe_mshr_readreq_threadid_s1(pipe_mshr_readreq_threadid_s1),
    .pipe_mshr_readreq_mshrid_s1(pipe_mshr_readreq_mshrid_s1),
    .mshr_pipe_readres_control_s1(mshr_pipe_readres_control_s1),
    .mshr_pipe_readres_homeid_s1(mshr_pipe_readres_homeid_s1),
    .mshr_pipe_vals_s1(mshr_pipe_vals_s1),
    .mshr_pipe_ld_address(mshr_pipe_ld_address),
    .mshr_pipe_st_address(mshr_pipe_st_address),
    .mshr_pipe_st_way_s1(mshr_pipe_st_way_s1),
    .mshr_pipe_st_state_s1(mshr_pipe_st_state_s1),
    .pipe_mshr_write_buffer_rd_en_s2(pipe_mshr_write_buffer_rd_en_s2),
    .pipe_mshr_threadid_s2(pipe_mshr_threadid_s2),
    .mshr_pipe_write_buffer_s2(mshr_pipe_write_buffer_s2),
    .mshr_pipe_write_buffer_byte_mask_s2(mshr_pipe_write_buffer_byte_mask_s2),
    .pipe_mshr_val_s3(pipe_mshr_val_s3),
    .pipe_mshr_op_s3(pipe_mshr_op_s3),
    .pipe_mshr_mshrid_s3(pipe_mshr_mshrid_s3),
    .pipe_mshr_threadid_s3(pipe_mshr_threadid_s3),
    .pipe_mshr_write_update_state_s3(pipe_mshr_write_update_state_s3),
    .pipe_mshr_write_update_way_s3(pipe_mshr_write_update_way_s3)
);

/*
NoC1 buffers data before send out to NoC1, unlike NoC3 which doesn't have to buffer
    The buffer scheme will probably work as follow:
        We will have 4 queues: writeback guard queue, CAS queue, 8B data inst queue, and ld/st queue
        - Combined WBG/ldst queue: 6 entries (1l/1s/1if each thread). No data
        - Dataqueue of 16B
            Supporting CAS/LDSTUB/SWAP and write-through. 1 CAS or 2 LDSTUB/SWAP or 2 write-through.
        All need to have the address + request metadata
    Priorities for the queues:
        1. writeback guard
        2. CAS
        3. data queue
        4. ld/st queue
        Note: TSO will not be violated regardless of how the NoC1 priority is chosen.
                This is due to the fact that only 1 load per thread can be outstanding, and no ordering between different threads enforced
                Actually, WBG might need to be ordered with respect to LD/ST req
*/
noc1buffer noc1buffer(
    .clk(clk),
    .rst_n(rst_n),
    .l15_noc1buffer_req_data_0(l15_noc1buffer_req_data_0),
    .l15_noc1buffer_req_data_1(l15_noc1buffer_req_data_1),
    .l15_noc1buffer_req_val(l15_noc1buffer_req_val),
    .l15_noc1buffer_req_type(l15_noc1buffer_req_type),
    .l15_noc1buffer_req_threadid(l15_noc1buffer_req_threadid),
    .l15_noc1buffer_req_mshrid(l15_noc1buffer_req_mshrid),
    .l15_noc1buffer_req_address(l15_noc1buffer_req_address),
    .l15_noc1buffer_req_non_cacheable(l15_noc1buffer_req_non_cacheable),
    .l15_noc1buffer_req_size(l15_noc1buffer_req_size),
    .l15_noc1buffer_req_prefetch(l15_noc1buffer_req_prefetch),
    // .l15_noc1buffer_req_blkstore(l15_noc1buffer_req_blkstore),
    // .l15_noc1buffer_req_blkinitstore(l15_noc1buffer_req_blkinitstore),
    .l15_noc1buffer_req_csm_data(l15_noc1buffer_req_csm_data),
    
    .l15_noc1buffer_req_csm_ticket(l15_noc1buffer_req_csm_ticket),
    .l15_noc1buffer_req_homeid(l15_noc1buffer_req_homeid),
    .l15_noc1buffer_req_homeid_val(l15_noc1buffer_req_homeid_val),
    .noc1buffer_noc1encoder_req_csm_sdid(noc1buffer_noc1encoder_req_csm_sdid),
    .noc1buffer_noc1encoder_req_csm_lsid(noc1buffer_noc1encoder_req_csm_lsid),
    
    .noc1encoder_noc1buffer_req_ack(noc1encoder_noc1buffer_req_ack),
    
    .noc1buffer_noc1encoder_req_data_0(noc1buffer_noc1encoder_req_data_0),
    .noc1buffer_noc1encoder_req_data_1(noc1buffer_noc1encoder_req_data_1),
    .noc1buffer_noc1encoder_req_val(noc1buffer_noc1encoder_req_val),
    .noc1buffer_noc1encoder_req_type(noc1buffer_noc1encoder_req_type),
    .noc1buffer_noc1encoder_req_mshrid(noc1buffer_noc1encoder_req_mshrid),
    .noc1buffer_noc1encoder_req_threadid(noc1buffer_noc1encoder_req_threadid),
    .noc1buffer_noc1encoder_req_address(noc1buffer_noc1encoder_req_address),
    .noc1buffer_noc1encoder_req_non_cacheable(noc1buffer_noc1encoder_req_non_cacheable),
    .noc1buffer_noc1encoder_req_size(noc1buffer_noc1encoder_req_size),
    .noc1buffer_noc1encoder_req_prefetch(noc1buffer_noc1encoder_req_prefetch),
    // .noc1buffer_noc1encoder_req_blkstore(noc1buffer_noc1encoder_req_blkstore),
    // .noc1buffer_noc1encoder_req_blkinitstore(noc1buffer_noc1encoder_req_blkinitstore),
    
    // stall signal from dmbr prevents the encoder from sending requests to the L2
    // .l15_dmbr_l1missIn(l15_dmbr_l1missIn),
    // .l15_dmbr_l1missTag(l15_dmbr_l1missTag),
    // .dmbr_l15_stall(dmbr_l15_stall),
    
    // CSM
    .l15_csm_read_ticket(l15_csm_read_ticket),
    .l15_csm_clear_ticket(l15_csm_clear_ticket),
    .l15_csm_clear_ticket_val(l15_csm_clear_ticket_val),
    .csm_l15_read_res_data(csm_l15_read_res_data),
    .csm_l15_read_res_val(csm_l15_read_res_val),
    .noc1buffer_noc1encoder_req_homeid(noc1buffer_noc1encoder_req_homeid),
    
    // .noc1buffer_l15_req_ack(noc1encoder_l15_req_ack),
    .noc1buffer_l15_req_sent(noc1encoder_l15_req_sent),
    .noc1buffer_l15_req_data_sent(noc1encoder_l15_req_data_sent),
    
    // homeid
    .noc1buffer_mshr_homeid_write_threadid_s4(noc1buffer_mshr_homeid_write_threadid_s4),
    .noc1buffer_mshr_homeid_write_val_s4(noc1buffer_mshr_homeid_write_val_s4),
    .noc1buffer_mshr_homeid_write_mshrid_s4(noc1buffer_mshr_homeid_write_mshrid_s4),
    .noc1buffer_mshr_homeid_write_data_s4(noc1buffer_mshr_homeid_write_data_s4)
);

noc1encoder noc1encoder(
    .clk(clk),
    .rst_n(rst_n),
    .noc1buffer_noc1encoder_req_data_0(noc1buffer_noc1encoder_req_data_0),
    .noc1buffer_noc1encoder_req_data_1(noc1buffer_noc1encoder_req_data_1),
    .noc1buffer_noc1encoder_req_val(noc1buffer_noc1encoder_req_val),
    .noc1buffer_noc1encoder_req_type(noc1buffer_noc1encoder_req_type),
    .noc1buffer_noc1encoder_req_mshrid(noc1buffer_noc1encoder_req_mshrid),
    .noc1buffer_noc1encoder_req_threadid(noc1buffer_noc1encoder_req_threadid),
    .noc1buffer_noc1encoder_req_address(noc1buffer_noc1encoder_req_address),
    .noc1buffer_noc1encoder_req_non_cacheable(noc1buffer_noc1encoder_req_non_cacheable),
    .noc1buffer_noc1encoder_req_size(noc1buffer_noc1encoder_req_size),
    .noc1buffer_noc1encoder_req_prefetch(noc1buffer_noc1encoder_req_prefetch),
    // .noc1buffer_noc1encoder_req_blkstore(noc1buffer_noc1encoder_req_blkstore),
    // .noc1buffer_noc1encoder_req_blkinitstore(noc1buffer_noc1encoder_req_blkinitstore),
    .noc1buffer_noc1encoder_req_csm_sdid(noc1buffer_noc1encoder_req_csm_sdid),
    .noc1buffer_noc1encoder_req_csm_lsid(noc1buffer_noc1encoder_req_csm_lsid),
    .noc1buffer_noc1encoder_req_homeid(noc1buffer_noc1encoder_req_homeid),
    
    .dmbr_l15_stall(dmbr_l15_stall),
    .chipid(chipid),
    .coreid_x(coreid_x),
    .coreid_y(coreid_y),
    .noc1out_ready(noc1_out_rdy),
    
    .l15_dmbr_l1missIn(l15_dmbr_l1missIn),
    .l15_dmbr_l1missTag(l15_dmbr_l1missTag),
    .noc1encoder_noc1buffer_req_ack(noc1encoder_noc1buffer_req_ack),
    .noc1encoder_noc1out_val(noc1_out_val),
    .noc1encoder_noc1out_data(noc1_out_data),
    
    // csm interface
    .noc1encoder_csm_req_ack(noc1encoder_csm_req_ack),
    .csm_noc1encoder_req_val(csm_noc1encoder_req_val),
    .csm_noc1encoder_req_type(csm_noc1encoder_req_type),
    .csm_noc1encoder_req_mshrid(csm_noc1encoder_req_mshrid),
    .csm_noc1encoder_req_address(csm_noc1encoder_req_address),
    .csm_noc1encoder_req_non_cacheable(csm_noc1encoder_req_non_cacheable),
    .csm_noc1encoder_req_size(csm_noc1encoder_req_size)
);


/*
   1 deep buffer for noc3 to improve performance and reduce timing pressure
*/
noc3buffer noc3buffer(
    .clk(clk),
    .rst_n(rst_n),
    .l15_noc3encoder_req_val(l15_noc3encoder_req_val),
    .l15_noc3encoder_req_type(l15_noc3encoder_req_type),
    .l15_noc3encoder_req_data_0(l15_noc3encoder_req_data_0),
    .l15_noc3encoder_req_data_1(l15_noc3encoder_req_data_1),
    .l15_noc3encoder_req_mshrid(l15_noc3encoder_req_mshrid),
    .l15_noc3encoder_req_sequenceid(l15_noc3encoder_req_sequenceid),
    .l15_noc3encoder_req_threadid(l15_noc3encoder_req_threadid),
    .l15_noc3encoder_req_address(l15_noc3encoder_req_address),
    .l15_noc3encoder_req_with_data(l15_noc3encoder_req_with_data),
    .l15_noc3encoder_req_was_inval(l15_noc3encoder_req_was_inval),
    .l15_noc3encoder_req_fwdack_vector(l15_noc3encoder_req_fwdack_vector),
    .l15_noc3encoder_req_homeid(l15_noc3encoder_req_homeid),
    .noc3buffer_l15_req_ack(noc3encoder_l15_req_ack),
    
    // from buffer to encoder
    .noc3buffer_noc3encoder_req_val(noc3buffer_noc3encoder_req_val),
    .noc3buffer_noc3encoder_req_type(noc3buffer_noc3encoder_req_type),
    .noc3buffer_noc3encoder_req_data_0(noc3buffer_noc3encoder_req_data_0),
    .noc3buffer_noc3encoder_req_data_1(noc3buffer_noc3encoder_req_data_1),
    .noc3buffer_noc3encoder_req_mshrid(noc3buffer_noc3encoder_req_mshrid),
    .noc3buffer_noc3encoder_req_sequenceid(noc3buffer_noc3encoder_req_sequenceid),
    .noc3buffer_noc3encoder_req_threadid(noc3buffer_noc3encoder_req_threadid),
    .noc3buffer_noc3encoder_req_address(noc3buffer_noc3encoder_req_address),
    .noc3buffer_noc3encoder_req_with_data(noc3buffer_noc3encoder_req_with_data),
    .noc3buffer_noc3encoder_req_was_inval(noc3buffer_noc3encoder_req_was_inval),
    .noc3buffer_noc3encoder_req_fwdack_vector(noc3buffer_noc3encoder_req_fwdack_vector),
    .noc3buffer_noc3encoder_req_homeid(noc3buffer_noc3encoder_req_homeid),
    .noc3encoder_noc3buffer_req_ack(noc3encoder_noc3buffer_req_ack)
);

noc3encoder noc3encoder(
    .clk(clk),
    .rst_n(rst_n),
    .l15_noc3encoder_req_val(noc3buffer_noc3encoder_req_val),
    .l15_noc3encoder_req_type(noc3buffer_noc3encoder_req_type),
    .l15_noc3encoder_req_data_0(noc3buffer_noc3encoder_req_data_0),
    .l15_noc3encoder_req_data_1(noc3buffer_noc3encoder_req_data_1),
    .l15_noc3encoder_req_mshrid(noc3buffer_noc3encoder_req_mshrid),
    .l15_noc3encoder_req_sequenceid(noc3buffer_noc3encoder_req_sequenceid),
    .l15_noc3encoder_req_threadid(noc3buffer_noc3encoder_req_threadid),
    .l15_noc3encoder_req_address(noc3buffer_noc3encoder_req_address),
    .l15_noc3encoder_req_with_data(noc3buffer_noc3encoder_req_with_data),
    .l15_noc3encoder_req_was_inval(noc3buffer_noc3encoder_req_was_inval),
    .l15_noc3encoder_req_fwdack_vector(noc3buffer_noc3encoder_req_fwdack_vector),
    .l15_noc3encoder_req_homeid(noc3buffer_noc3encoder_req_homeid),
    .chipid(chipid),
    .coreid_x(coreid_x),
    .coreid_y(coreid_y),
    .noc3out_ready(noc3_out_rdy),
    .noc3encoder_l15_req_ack(noc3encoder_noc3buffer_req_ack),
    .noc3encoder_noc3out_val(noc3_out_val),
    .noc3encoder_noc3out_data(noc3_out_data)
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
//  Filename      : cpxencoder.v
//  Created On    : 2014-03-26 18:58:51
//  Last Modified : 2015-01-22 17:08:52
//  Revision      :
//  Author        : Tri Nguyen
//  Company       : Princeton University
//  Email         : trin@princeton.edu
//
//  Description   :
//
//
//==================================================================================================
//cpxencoder.v

/* OLDNOTE (5/15/14) Note regarding l15_cpxencoder_blockinitstore
    in cpx.cc, the 125 bit of the return inval vector is set when block init store is set
*/
//`default_nettype none
module l15_cpxencoder(
    input wire          clk,
    input wire          rst_n,

    input wire          l15_cpxencoder_val,
    input wire [3:0]    l15_cpxencoder_returntype,
    input wire          l15_cpxencoder_l2miss,
    input wire [1:0]    l15_cpxencoder_error,
    input wire          l15_cpxencoder_noncacheable,
    input wire          l15_cpxencoder_atomic,
    input wire [0:0]    l15_cpxencoder_threadid,
    input wire          l15_cpxencoder_prefetch,
    input wire          l15_cpxencoder_f4b,
    input wire [63:0]  l15_cpxencoder_data_0,
    input wire [63:0]  l15_cpxencoder_data_1,
    input wire [63:0]  l15_cpxencoder_data_2,
    input wire [63:0]  l15_cpxencoder_data_3,
    input wire          l15_cpxencoder_inval_icache_all_way,
    input wire          l15_cpxencoder_inval_dcache_all_way,
    input wire [15:4]   l15_cpxencoder_inval_address_15_4,
    input wire          l15_cpxencoder_cross_invalidate,
    input wire [1:0]    l15_cpxencoder_cross_invalidate_way,
    input wire          l15_cpxencoder_inval_dcache_inval,
    input wire          l15_cpxencoder_inval_icache_inval,
    input wire [1:0]    l15_cpxencoder_inval_way,
    input wire          l15_cpxencoder_blockinitstore,

    output reg uncore_spc_data_ready,
    output reg [145-1:0] uncore_spc_data,

    output reg cpxencoder_l15_req_ack
    );




reg [145-1:0] out;
wire [2:0] cpuid = 3'b000;

reg state;
reg next_state;

// debugging signal
reg [1:0] inval_index_5_4;

always @ (posedge clk)
begin
    if (!rst_n)
    begin
        state <= 1'b0;
    end
    else
    begin
        state <= next_state;
    end
end


always @ *
begin
    uncore_spc_data[145-1:0] = out[145-1:0];
    uncore_spc_data_ready = l15_cpxencoder_val;
    cpxencoder_l15_req_ack = (l15_cpxencoder_val && (next_state == 1'b0));
end

always @ *
begin
    out[144] = l15_cpxencoder_val;
    out[143:140] = l15_cpxencoder_returntype;
    out[139:0] = 1'b0;
    next_state = 1'b0;
    inval_index_5_4 = l15_cpxencoder_inval_address_15_4[5:4];
    if (l15_cpxencoder_val)
    begin
    case(l15_cpxencoder_returntype)
        4'b0000:
        begin
            // load
            out[139] = l15_cpxencoder_l2miss;
            out[138:137] = l15_cpxencoder_error;
            out[136] = l15_cpxencoder_noncacheable;
            out[135:134] = l15_cpxencoder_threadid;
            out[133] = l15_cpxencoder_cross_invalidate;
            out[132:131] = l15_cpxencoder_cross_invalidate_way;
            // bit 130 is 1'b0
            out[129] = l15_cpxencoder_atomic;
            out[128] = l15_cpxencoder_prefetch;
            out[127:0] = {l15_cpxencoder_data_0, l15_cpxencoder_data_1};
        end

        4'b0001:
        begin
            case (state)
                1'b0:
                begin
                    out[139] = l15_cpxencoder_l2miss;
                    out[138:137] = l15_cpxencoder_error;
                    out[136] = l15_cpxencoder_noncacheable;
                    out[135:134] = l15_cpxencoder_threadid;
                    out[133] = l15_cpxencoder_cross_invalidate;
                    out[132:131] = l15_cpxencoder_cross_invalidate_way;
                    out[130] = l15_cpxencoder_f4b;
                    out[129] = 1'b0;
                    out[128] = 1'b0;
                    out[127:0] = {l15_cpxencoder_data_0, l15_cpxencoder_data_1};
                    next_state = 1'b1;
                end
                1'b1:
                begin
                    // no l2 miss
                    out[138:137] = l15_cpxencoder_error;
                    out[136] = l15_cpxencoder_noncacheable;
                    out[135:134] = l15_cpxencoder_threadid;
                    out[133] = l15_cpxencoder_cross_invalidate;
                    out[132:131] = l15_cpxencoder_cross_invalidate_way;
                    out[130] = 1'b0;
                    out[129] = 1'b1;
                    out[128] = 1'b0;
                    out[127:0] = {l15_cpxencoder_data_2, l15_cpxencoder_data_3};
                    next_state = 1'b0;
                end
            endcase
        end

        4'b0011:
        begin
            // eviction
            out[136] = l15_cpxencoder_noncacheable;
            out[128] = 1'b0;
            // Invalidation vector generation
                out[127:126] = 2'b0;
                out[125] = l15_cpxencoder_blockinitstore;
                out[124:123] = {l15_cpxencoder_inval_icache_all_way,
                                l15_cpxencoder_inval_dcache_all_way};
                out[122:121] = l15_cpxencoder_inval_address_15_4[5:4];
                out[120:118] = cpuid[2:0];
                out[117:112] = l15_cpxencoder_inval_address_15_4[11:6];

                // out[`CPX_INV_PA_15_12] = l15_cpxencoder_inval_address_15_4[15:12];
                if (l15_cpxencoder_inval_dcache_inval == 1)
                begin
                    out[5:2] = l15_cpxencoder_inval_way;
                    out[0] = 1'b1;
                end
                // trin: icache inval not used; monitored
        end

        4'b0100:
        begin
            out[136] = l15_cpxencoder_noncacheable;
            out[135:134] = l15_cpxencoder_threadid;
            out[129] = l15_cpxencoder_atomic;
            // Invalidation vector generation
                out[127:126] = 2'b0;
                out[125] = l15_cpxencoder_blockinitstore;
                out[124:123] = {l15_cpxencoder_inval_icache_all_way,
                                l15_cpxencoder_inval_dcache_all_way};
                out[122:121] = l15_cpxencoder_inval_address_15_4[5:4];
                out[120:118] = cpuid[2:0];
                out[117:112] = l15_cpxencoder_inval_address_15_4[11:6];

                // out[`CPX_INV_PA_15_12] = l15_cpxencoder_inval_address_15_4[15:12];
                if (l15_cpxencoder_inval_dcache_inval == 1)
                begin
                    out[5:2] = l15_cpxencoder_inval_way;
                    out[0] = 1'b1;
                end
        end

        4'b0111:
        begin
            // TODO
            // out[136] = l15_cpxencoder_flush;
            // out[127:0] = l15_cpxencoder_data[127:0];
            out[136] = l15_cpxencoder_noncacheable; // is the flush_bit
            out[63:0] = l15_cpxencoder_data_0;

            // bug: threadid must be set even though it's X's in the document
            out[135:134] = l15_cpxencoder_threadid;
        end

        4'b1110:
        begin
            case (state)
                1'b0:
                begin
                    // load return first
                    out[143:140] = 4'b0000;
                    out[139] = l15_cpxencoder_l2miss;
                    out[138:137] = l15_cpxencoder_error;
                    out[136] = l15_cpxencoder_noncacheable;
                    out[135:134] = l15_cpxencoder_threadid;
                    out[133] = l15_cpxencoder_cross_invalidate;
                    out[132:131] = l15_cpxencoder_cross_invalidate_way;
                    // bit 130 is 1'b0
                    out[129] = l15_cpxencoder_atomic;
                    out[128] = l15_cpxencoder_prefetch;
                    out[127:0] = {l15_cpxencoder_data_0, l15_cpxencoder_data_1};
                    next_state = 1'b1;
                end
                1'b1:
                begin
                    // stack
                    out[143:140] = 4'b0100;
                    out[136] = l15_cpxencoder_noncacheable;
                    out[135:134] = l15_cpxencoder_threadid;
                    out[129] = l15_cpxencoder_atomic;
                    // Invalidation vector generation
                        out[127:126] = 2'b0;
                        out[125] = l15_cpxencoder_blockinitstore;
                        out[124:123] = {l15_cpxencoder_inval_icache_all_way,
                                        l15_cpxencoder_inval_dcache_all_way};
                        out[122:121] = l15_cpxencoder_inval_address_15_4[5:4];
                        out[120:118] = cpuid[2:0];
                        out[117:112] = l15_cpxencoder_inval_address_15_4[11:6];
                        // trin: storeacks for CAS do not invalidate anything; monitored
                    next_state = 1'b0;
                end
            endcase
        end

        4'b1100:
        begin
            out[138:137] = l15_cpxencoder_error;
            out[135] = 1'b0;
            out[134] = 1'b0;
        end

        default:
        begin
            out[144] = 1'b0;
        end
    endcase
    end
end



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
module l15_picoencoder(
    input wire          clk,
    input wire          rst_n,
    
    input wire          l15_picoencoder_val,
    input wire [3:0]    l15_picoencoder_returntype,
    
    input wire [63:0]   l15_picoencoder_data_0,
    input wire [63:0]   l15_picoencoder_data_1,
    
    input wire [39:0]   picodecoder_l15_address,  
    
    output reg          pico_mem_ready,
    output wire [31:0]  pico_mem_rdata,
    
    output wire         picoencoder_l15_req_ack,
    output reg          pico_int
);
    
    reg [31:0] rdata_part;
    assign pico_mem_rdata = {rdata_part[7:0], rdata_part[15:8],
                             rdata_part[23:16], rdata_part[31:24]};
    assign picoencoder_l15_req_ack = l15_picoencoder_val;
     
    // keep track of whether we have received the wakeup interrupt
    reg int_recv;
    always @ (posedge clk) begin
        if (!rst_n) begin
            pico_int <= 1'b0;
        end
        else if (int_recv) begin
            pico_int <= 1'b1;
        end
        else if (pico_int) begin
            pico_int <= 1'b0;
        end
    end
       
    always @ * begin
        if (l15_picoencoder_val) begin
            case(l15_picoencoder_returntype)
                4'b0000, 4'b1110: begin
                    // load
                    int_recv = 1'b0;
                    pico_mem_ready = 1'b1;
                    case(picodecoder_l15_address[3:2])
                        2'b00: begin
                            rdata_part = l15_picoencoder_data_0[63:32];
                        end
                        2'b01: begin
                            rdata_part = l15_picoencoder_data_0[31:0];
                        end
                        2'b10: begin
                            rdata_part = l15_picoencoder_data_1[63:32];
                        end
                        2'b11: begin
                            rdata_part = l15_picoencoder_data_1[31:0];
                        end
                        default: begin
                        end
                    endcase 
                end
                4'b0100: begin
                    int_recv = 1'b0;
                    pico_mem_ready = 1'b1;
                    rdata_part = 32'b0;
                end
                4'b0111: begin
                    if (l15_picoencoder_data_0[17:16] == 2'b01) begin
                        int_recv = 1'b1;
                    end
                    else begin
                        int_recv = 1'b0;
                    end
                    pico_mem_ready = 1'b0;
                    rdata_part = 32'b0;
                end
                default: begin
                    int_recv = 1'b0;
                    pico_mem_ready = 1'b0;
                    rdata_part = 32'b0;
                end
            endcase 
        end
        else begin
            int_recv = 1'b0;
            pico_mem_ready = 1'b0;
            rdata_part = 32'b0;
        end
    end
    
endmodule // l15_picoencoder
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

// This module wraps the L1.5 and ties unused signals

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
















































































































































































































































































































































































































































































































































































































































































































































































module l15_wrap (
    input                                   clk,
    input                                   rst_n,

    input [4:0]                             transducer_l15_rqtype,
    input [4-1:0]           transducer_l15_amo_op,
    input                                   transducer_l15_nc,
    input [2:0]                             transducer_l15_size,
    input [0:0]              transducer_l15_threadid,
    input                                   transducer_l15_prefetch,
    input                                   transducer_l15_invalidate_cacheline,
    input                                   transducer_l15_blockstore,
    input                                   transducer_l15_blockinitstore,
    input [1:0]                             transducer_l15_l1rplway,
    input                                   transducer_l15_val,
    input [39:0]                            transducer_l15_address,
    input [63:0]                            transducer_l15_data,
    input [63:0]                            transducer_l15_data_next_entry,
    input [33-1:0]              transducer_l15_csm_data,

    output                                  l15_transducer_ack,
    output                                  l15_transducer_header_ack,

    output                                  l15_transducer_val,
    output [3:0]                            l15_transducer_returntype,
    output                                  l15_transducer_l2miss,
    output [1:0]                            l15_transducer_error,
    output                                  l15_transducer_noncacheable,
    output                                  l15_transducer_atomic,
    output [0:0]             l15_transducer_threadid,
    output                                  l15_transducer_prefetch,
    output                                  l15_transducer_f4b,
    output [63:0]                           l15_transducer_data_0,
    output [63:0]                           l15_transducer_data_1,
    output [63:0]                           l15_transducer_data_2,
    output [63:0]                           l15_transducer_data_3,
    output                                  l15_transducer_inval_icache_all_way,
    output                                  l15_transducer_inval_dcache_all_way,
    output [15:4]                           l15_transducer_inval_address_15_4,
    output                                  l15_transducer_cross_invalidate,
    output [1:0]                            l15_transducer_cross_invalidate_way,
    output                                  l15_transducer_inval_dcache_inval,
    output                                  l15_transducer_inval_icache_inval,
    output [1:0]                            l15_transducer_inval_way,
    output                                  l15_transducer_blockinitstore,

    input                                   transducer_l15_req_ack,

    input                                   noc1_out_rdy,
    input                                   noc2_in_val,
    input [64-1:0]             noc2_in_data,
    input                                   noc3_out_rdy,
    input                                   dmbr_l15_stall,
    input [14-1:0]           chipid,
    input [8-1:0]                coreid_x,
    input [8-1:0]                coreid_y,

    // input from config registers to pipeline
    input [63:0]                            config_l15_read_res_data_s3,
    input                                   config_csm_en,
    input [5:0]                             config_system_tile_count_5_0,
    input [2-1:0]    config_home_alloc_method, 
    input [22-1:0]    config_hmt_base,

    output                                  noc1_out_val,
    output [64-1:0]            noc1_out_data,
    output                                  noc2_in_rdy,
    output                                  noc3_out_val,
    output [64-1:0]            noc3_out_data,
    // output wire pcx_req_squashed,
    output                                  l15_dmbr_l1missIn,
    output [4-1:0]            l15_dmbr_l1missTag,
    output                                  l15_dmbr_l2responseIn,
    output                                  l15_dmbr_l2missIn,
    output [4-1:0]            l15_dmbr_l2missTag,

    // output to config registers to pipeline
    output                                  l15_config_req_val_s2,
    output                                  l15_config_req_rw_s2,
    output [63:0]                           l15_config_write_req_data_s2,
    output [15:8]       l15_config_req_address_s2,

    // sram interface
    output [4-1:0]    srams_rtap_data,
    input  [4-1:0]             rtap_srams_bist_command,
    input  [4-1:0]    rtap_srams_bist_data
);

    wire [31:0]   config_system_tile_count = {26'bx, config_system_tile_count_5_0};
   
    l15 l15 (
        .clk(clk),
        .rst_n(rst_n),

        .transducer_l15_rqtype              (transducer_l15_rqtype),
        .transducer_l15_amo_op              (transducer_l15_amo_op),
        .transducer_l15_nc                  (transducer_l15_nc),
        .transducer_l15_size                (transducer_l15_size),
        // .pcxdecoder_l15_invalall         (transducer_l15_invalall),
        .transducer_l15_threadid            (transducer_l15_threadid),
        .transducer_l15_prefetch            (transducer_l15_prefetch),
        .transducer_l15_blockstore          (transducer_l15_blockstore),
        .transducer_l15_blockinitstore      (transducer_l15_blockinitstore),
        .transducer_l15_l1rplway            (transducer_l15_l1rplway),
        .transducer_l15_val                 (transducer_l15_val),
        .transducer_l15_invalidate_cacheline(transducer_l15_invalidate_cacheline),
        .transducer_l15_address             (transducer_l15_address),
        .transducer_l15_csm_data            (transducer_l15_csm_data),
        .transducer_l15_data                (transducer_l15_data),
        .transducer_l15_data_next_entry     (transducer_l15_data_next_entry),

        .l15_transducer_ack                 (l15_transducer_ack),
        .l15_transducer_header_ack          (l15_transducer_header_ack),
                               
        .l15_transducer_val                 (l15_transducer_val),
        .l15_transducer_returntype          (l15_transducer_returntype),
        .l15_transducer_l2miss              (l15_transducer_l2miss),
        .l15_transducer_error               (l15_transducer_error),
        .l15_transducer_noncacheable        (l15_transducer_noncacheable),
        .l15_transducer_atomic              (l15_transducer_atomic),
        .l15_transducer_threadid            (l15_transducer_threadid),
        .l15_transducer_prefetch            (l15_transducer_prefetch),
        .l15_transducer_f4b                 (l15_transducer_f4b),
        .l15_transducer_data_0              (l15_transducer_data_0),
        .l15_transducer_data_1              (l15_transducer_data_1),
        .l15_transducer_data_2              (l15_transducer_data_2),
        .l15_transducer_data_3              (l15_transducer_data_3),
        .l15_transducer_inval_icache_all_way(l15_transducer_inval_icache_all_way),
        .l15_transducer_inval_dcache_all_way(l15_transducer_inval_dcache_all_way),
        .l15_transducer_inval_address_15_4  (l15_transducer_inval_address_15_4),
        .l15_transducer_cross_invalidate    (l15_transducer_cross_invalidate),
        .l15_transducer_cross_invalidate_way(l15_transducer_cross_invalidate_way),
        .l15_transducer_inval_dcache_inval  (l15_transducer_inval_dcache_inval),
        .l15_transducer_inval_icache_inval  (l15_transducer_inval_icache_inval),
        .l15_transducer_inval_way           (l15_transducer_inval_way),
        .l15_transducer_blockinitstore      (l15_transducer_blockinitstore),

        .transducer_l15_req_ack             (transducer_l15_req_ack),

        .noc1_out_rdy(noc1_out_rdy),
        .noc2_in_val(noc2_in_val),
        .noc2_in_data(noc2_in_data),
        .noc3_out_rdy(noc3_out_rdy),
        .dmbr_l15_stall(dmbr_l15_stall),
        .chipid(chipid),
        .coreid_x(coreid_x),
        .coreid_y(coreid_y),

        .noc1_out_val(noc1_out_val),
        .noc1_out_data(noc1_out_data),
        .noc2_in_rdy(noc2_in_rdy),
        .noc3_out_val(noc3_out_val),
        .noc3_out_data(noc3_out_data),
        // .pcx_req_squashed(pcx_req_squashed),
        .l15_dmbr_l1missIn(l15_dmbr_l1missIn),
        .l15_dmbr_l1missTag(l15_dmbr_l1missTag),
        .l15_dmbr_l2missIn(l15_dmbr_l2missIn),
        .l15_dmbr_l2missTag(l15_dmbr_l2missTag),
        .l15_dmbr_l2responseIn(l15_dmbr_l2responseIn),

        // config registers
        .l15_config_req_val_s2(l15_config_req_val_s2),
        .l15_config_req_rw_s2(l15_config_req_rw_s2),
        .l15_config_write_req_data_s2(l15_config_write_req_data_s2),
        .l15_config_req_address_s2(l15_config_req_address_s2),
        .config_l15_read_res_data_s3(config_l15_read_res_data_s3),

        // config regs
        .config_csm_en(config_csm_en),
        .config_hmt_base(config_hmt_base),
        .config_system_tile_count(config_system_tile_count),
        .config_home_alloc_method(config_home_alloc_method),
        
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
//  Filename      : noc1encoder.v
//  Created On    : 2014-02-05 20:06:27
//  Revision      :
//  Author        : Tri Nguyen
//  Company       : Princeton University
//  Email         : trin@princeton.edu
//
//  Description   :
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









































































































































































































































































































































































































































































































































































































module noc1encoder(
   input wire clk,
   input wire rst_n,

   // interface with noc1buffer
   input wire [63:0] noc1buffer_noc1encoder_req_data_0,
   input wire [63:0] noc1buffer_noc1encoder_req_data_1,
   input wire noc1buffer_noc1encoder_req_val,
   input wire [5-1:0] noc1buffer_noc1encoder_req_type,
   input wire [2-1:0] noc1buffer_noc1encoder_req_mshrid,
   input wire [0:0] noc1buffer_noc1encoder_req_threadid,
   input wire [39:0] noc1buffer_noc1encoder_req_address,
   input wire noc1buffer_noc1encoder_req_non_cacheable,
   input wire [3-1:0] noc1buffer_noc1encoder_req_size,
   input wire noc1buffer_noc1encoder_req_prefetch,
   // input wire noc1buffer_noc1encoder_req_blkstore,
   // input wire noc1buffer_noc1encoder_req_blkinitstore,
   input wire [(14+8+8)-1:0] noc1buffer_noc1encoder_req_homeid,
   input wire [10-1:0] noc1buffer_noc1encoder_req_csm_sdid,
   input wire [6-1:0] noc1buffer_noc1encoder_req_csm_lsid,

   output reg noc1encoder_noc1buffer_req_ack,

   // current chip position
   input wire [14-1:0] chipid,
   input wire [8-1:0] coreid_x,
   input wire [8-1:0] coreid_y,

   // ready at noc1
   input wire noc1out_ready,
   output reg noc1encoder_noc1out_val,
   output reg [63:0] noc1encoder_noc1out_data,

   // dmbr stall interface
   input wire dmbr_l15_stall,
   output reg                       l15_dmbr_l1missIn,
   output reg [4-1:0] l15_dmbr_l1missTag,

   // csm interface
   input wire csm_noc1encoder_req_val,
   input wire [5-1:0] csm_noc1encoder_req_type,
   input wire [3-1:0] csm_noc1encoder_req_mshrid,
   input wire [40-1:0] csm_noc1encoder_req_address,
   input wire csm_noc1encoder_req_non_cacheable,
   input wire  [3-1:0] csm_noc1encoder_req_size,
   output reg noc1encoder_csm_req_ack
);
// The flit sending out this cycle
reg [63:0] flit;
reg [4-1:0] flit_state;
reg [4-1:0] flit_state_next;

reg sending;
reg dmbr_stall;
reg control_raw_data_flit1;

always @ (posedge clk)
begin
   if (!rst_n)
   begin
      flit_state <= 0;
   end
   else
   begin
      flit_state <= flit_state_next;
   end
end


always @ *
begin

   // simple alias for output
   noc1encoder_noc1out_data = flit;

   // sending throttle for dmbr
   dmbr_stall = dmbr_l15_stall && (flit_state == 0); // let's not stall in the middle of a msg
   sending = (noc1buffer_noc1encoder_req_val || csm_noc1encoder_req_val) && !dmbr_stall;
   noc1encoder_noc1out_val = sending;

end

// mux selects between l15 buffer and csm, with priority to csm
// generate request signals
reg [1-1:0] last_req_source;
reg [1-1:0] req_source;
reg [5-1:0] req_type;
reg req_prefetch;
reg req_nc;
// reg req_blkstore;
// reg req_blkinitstore;
reg [63:0] req_data0;
reg [63:0] req_data1;
reg [39:0] req_address;
reg [8-1:0] req_mshrid;
reg [3-1:0] req_size;
reg [8-1:0] req_dest_l2_xpos;
reg [8-1:0] req_dest_l2_ypos;
reg [14-1:0] req_dest_chipid;
reg [6-1:0] req_csm_lsid;
reg [10-1:0] req_csm_sdid;
always @ (posedge clk)
begin
   if (!rst_n)
      last_req_source <= 0;
   else
      last_req_source <= req_source;
end

always @ *
begin
   req_source = 0;
   req_type = 0;
   req_prefetch = 0;
   req_nc = 0;
   // req_blkstore = 0;
   // req_blkinitstore = 0;
   req_data0 = 0;
   req_data1 = 0;
   req_address = 0;
   req_mshrid = 0;
   req_size = 0;
   req_dest_l2_xpos = 0;
   req_dest_l2_ypos = 0;
   req_dest_chipid = 0;
   req_csm_lsid = 0;
   req_csm_sdid = 0;

   if ((last_req_source == 1'b0 && (flit_state != 0)) ||
         csm_noc1encoder_req_val == 1'b0)
   begin
      // condition for selecting L15 as source:
      // 1. was sending L15 message last cycle is is still sending (no interruption)
      // 2. or csm is not sending
      req_source = 1'b0;
   end
   else
   begin
      // else select csm
      req_source = 1'b1;
   end

   if (req_source == 1'b0)
   begin
      req_type = noc1buffer_noc1encoder_req_type;
      req_prefetch = noc1buffer_noc1encoder_req_prefetch;
      req_nc = noc1buffer_noc1encoder_req_non_cacheable;
      // req_blkstore = 1'b0; // All block stores are translated to normal stores
      // req_blkinitstore = 1'b0;
      req_data0 = noc1buffer_noc1encoder_req_data_0;
      req_data1 = noc1buffer_noc1encoder_req_data_1;
      req_address = noc1buffer_noc1encoder_req_address;
      req_mshrid = {noc1buffer_noc1encoder_req_threadid,noc1buffer_noc1encoder_req_mshrid};
      req_size = noc1buffer_noc1encoder_req_size;
      req_dest_l2_xpos = noc1buffer_noc1encoder_req_homeid[8-1:0];
      req_dest_l2_ypos = noc1buffer_noc1encoder_req_homeid[8+8-1:8];
      req_dest_chipid = noc1buffer_noc1encoder_req_homeid[((14+8+8)-1):(8+8)];

      if (req_type != 5'd1)
      begin
        req_csm_lsid = noc1buffer_noc1encoder_req_csm_lsid;
        req_csm_sdid = noc1buffer_noc1encoder_req_csm_sdid;
      end
   end
   else
   begin
      req_type = csm_noc1encoder_req_type;
      req_nc = csm_noc1encoder_req_non_cacheable;
      req_address = csm_noc1encoder_req_address;
      req_size = csm_noc1encoder_req_size;
      req_mshrid = csm_noc1encoder_req_mshrid;

      // send to home l2
      req_dest_l2_xpos = coreid_x;
      req_dest_l2_ypos = coreid_y;
      req_dest_chipid = chipid;

      // csm mshrid has the most significant bit set
      req_mshrid = req_mshrid | {1'b1, {8-1{1'b0}}};
   end


end

// translate req_ -> msg_
reg [40-1:0]              msg_address;
reg [8-1:0]                 msg_dest_l2_xpos;
reg [8-1:0]                 msg_dest_l2_ypos;
reg [8-1:0]                 msg_dest_l2_xpos_new;
reg [8-1:0]                 msg_dest_l2_ypos_new;
wire [8-1:0]                msg_dest_l2_xpos_compat;
wire [8-1:0]                msg_dest_l2_ypos_compat;
reg [14-1:0]            msg_dest_chipid;
reg [4-1:0]             msg_dest_fbits;
reg [8-1:0]                 msg_src_xpos;
reg [8-1:0]                 msg_src_ypos;
reg [14-1:0]            msg_src_chipid;
reg [4-1:0]             msg_src_fbits;
reg [8-1:0]            msg_length;
reg [8-1:0]              msg_type;
reg [8-1:0]            msg_mshrid;
reg [2-1:0]               msg_mesi;
reg [1-1:0]      msg_last_subline;
reg [5:0]                   msg_options_1;
reg [15:0]                  msg_options_2;
reg [29:0]                  msg_options_3;
reg [1-1:0]        msg_cache_type;
reg [4-1:0]    msg_subline_vector;
reg [3-1:0]         msg_data_size;
reg [5:0] t1_interrupt_cpuid;
always @ *
begin
   msg_length = 0;
   msg_type = 0;
   msg_mesi = 0;
   msg_last_subline = 0;
   msg_cache_type = 0;
   msg_subline_vector = 0; // always 0 for requests
   control_raw_data_flit1 = 0;
   t1_interrupt_cpuid = 0;

   msg_address = req_address;
   msg_mshrid = req_mshrid;
   msg_data_size = req_size;

   // source are static
   msg_src_xpos = coreid_x;
   msg_src_ypos = coreid_y;
   msg_src_chipid = chipid;
   msg_src_fbits = 4'd0;
   msg_dest_fbits = 4'd0;

   // default value for a message, will be overwritten by interrupt reqs
   msg_dest_l2_xpos = req_dest_l2_xpos;
   msg_dest_l2_ypos = req_dest_l2_ypos;
   msg_dest_chipid = req_dest_chipid;

   // trin: removing latches
   msg_dest_l2_xpos_new = 0;
   msg_dest_l2_ypos_new = 0;


   case (req_type)
      5'd1:
      begin
         msg_type = 8'd13;
         msg_length = 2; // 2 extra headers
         msg_cache_type = 1'b0;
      end
      5'd2:
      begin
         // this needs to be splitted into
         //  normal ld, ld nc, and prefetch ld
         if (req_prefetch)
            msg_type = 8'd1;
         else if (req_nc)
            msg_type = 8'd14;
         else
            msg_type = 8'd31;
         msg_length = 2; // 2 extra headers
         msg_cache_type = 1'b0;
      end
      5'd3:
      begin
         // can be either normal or nc
         if (req_nc)
            msg_type = 8'd14;
         else
            msg_type = 8'd31;
         msg_length = 2; // 2 extra headers
         msg_cache_type = 1'b1;
      end
      5'd4:
      begin
         // can be nc store, blkstore, or blkinitstore
         if (req_nc)
            msg_type = 8'd15;
         // else if (req_blkstore)
            // msg_type = `MSG_TYPE_BLKINIT_STORE_REQ;
         // else if (req_blkinitstore)
            // msg_type = `MSG_TYPE_BLK_STORE_REQ;
         msg_cache_type = 1'b0;
         msg_length = 3; // 2 extra headers + 1 data
      end
      5'd5, 5'd6:
      begin
         // should specify the sub cacheline? no, the L2 should be able to detect
         //      based on the msg_address
         msg_type = 8'd2;
         msg_cache_type = 1'b0;
         msg_length = 2; // 2 extra headers
      end
      5'd7:
      begin
         msg_type = 8'd5;
         msg_cache_type = 1'b0;
         msg_length = 4; // 2 extra headers + 1 compare data + 1 swap data
      end
      5'd8:
      begin
         msg_type = 8'd9;
         msg_cache_type = 1'b0;
         msg_length = 3; // 2 extra headers + 1 swap data
      end
      5'd9:
      begin
         msg_type = 8'd32;
         msg_length = 1; // just 1 data
         control_raw_data_flit1 = 1'b1;
         t1_interrupt_cpuid = req_data0[14:9];
         msg_dest_l2_xpos_new = req_data0[8+17:18];
         msg_dest_l2_ypos_new = req_data0[8+8+17:8+18];
         msg_dest_l2_xpos = req_data0[63] ? msg_dest_l2_xpos_new : msg_dest_l2_xpos_compat; 
         msg_dest_l2_ypos = req_data0[63] ? msg_dest_l2_ypos_new : msg_dest_l2_ypos_compat; 
         msg_dest_chipid  = req_data0[63] ? req_data0[14+8+8+17:8+8+18] : 14'b0;
      end
      5'd18:
      begin
         msg_type = 8'd60;
         msg_length = 2; // 2 extra headers
         msg_cache_type = 1'b0;
      end
      5'd10:
      begin
         msg_type = 8'd36;
         msg_cache_type = 1'b0;
         msg_length = 3; // 2 extra headers + 1 swap data
      end
      5'd11:
      begin
         msg_type = 8'd37;
         msg_cache_type = 1'b0;
         msg_length = 3; // 2 extra headers + 1 swap data
      end
      5'd12:
      begin
         msg_type = 8'd38;
         msg_cache_type = 1'b0;
         msg_length = 3; // 2 extra headers + 1 swap data
      end
      5'd13:
      begin
         msg_type = 8'd39;
         msg_cache_type = 1'b0;
         msg_length = 3; // 2 extra headers + 1 swap data
      end
      5'd14:
      begin
         msg_type = 8'd40;
         msg_cache_type = 1'b0;
         msg_length = 3; // 2 extra headers + 1 swap data
      end
      5'd15:
      begin
         msg_type = 8'd41;
         msg_cache_type = 1'b0;
         msg_length = 3; // 2 extra headers + 1 swap data
      end
      5'd16:
      begin
         msg_type = 8'd42;
         msg_cache_type = 1'b0;
         msg_length = 3; // 2 extra headers + 1 swap data
      end
      5'd17:
      begin
         msg_type = 8'd43;
         msg_cache_type = 1'b0;
         msg_length = 3; // 2 extra headers + 1 swap data
      end
   endcase
end

flat_id_to_xy cpuid_to_xy (
    .flat_id(t1_interrupt_cpuid),
    .x_coord(msg_dest_l2_xpos_compat),
    .y_coord(msg_dest_l2_ypos_compat)
    );

always @ *
begin
   msg_options_1 = 0;
   msg_options_2 = 0;
   msg_options_3 = 0;

   case (msg_data_size)
      3'b000:
         msg_options_2[10:8] = 3'b001;
      3'b001:
         msg_options_2[10:8] = 3'b010;
      3'b010:
         msg_options_2[10:8] = 3'b011;
      3'b011:
         msg_options_2[10:8] = 3'b100;
      3'b111:
         msg_options_2[10:8] = 3'b101;
   endcase

   msg_options_2[11] = msg_cache_type;
   msg_options_2[15:12] = msg_subline_vector;

   msg_options_3[29:20] = req_csm_sdid;
   msg_options_3[19:14] = req_csm_lsid;
end


// flit filling logic
// translate msg_ -> flit
always @ *
begin
   flit[64-1:0] = 0; // so that the flit is not a latch
   if (flit_state == 4'd0)
   begin
      flit[63:50] = msg_dest_chipid;
      flit[49:42] = msg_dest_l2_xpos;
      flit[41:34] = msg_dest_l2_ypos;
      flit[33:30] = msg_dest_fbits;
      flit[29:22] = msg_length;
      flit[21:14] = msg_type;
      flit[13:6] = msg_mshrid;
      flit[5:0] = msg_options_1;
   end
   else if (flit_state == 4'd1)
   begin
      if (control_raw_data_flit1)
      begin
         flit[64-1:0] = req_data0;
         // also need to suppress cpuid before sending
         flit[15:9] = 0;
      end
      else
      begin
         flit[((16 + 40 - 1)):(16)] = msg_address;
         flit[11] = msg_cache_type;
         flit[15:0] = msg_options_2;
      end
   end
   else if (flit_state == 4'd2)
   begin
      flit[63:50] = msg_src_chipid;
      flit[49:42] = msg_src_xpos;
      flit[41:34] = msg_src_ypos;
      flit[33:30] = msg_src_fbits;
      flit[29:0] = msg_options_3;
   end
   else if (flit_state == 4'd3)
   begin
      flit[64-1:0] = req_data0;
   end
   else if (flit_state == 4'd4)
   begin
      flit[64-1:0] = req_data1;
   end
end

always @ *
begin
   // next flit state logic
   if (sending)
   begin
      if (noc1out_ready)
      begin
         if (flit_state != msg_length)
            flit_state_next = flit_state + 1;
         else
            flit_state_next = 4'd0;
      end
      else
         flit_state_next = flit_state;
   end
   else
      flit_state_next = 4'd0;
end

always @ *
begin
   // ack logic to L1.5
   noc1encoder_noc1buffer_req_ack = 0;
   if (noc1buffer_noc1encoder_req_val && (flit_state == msg_length) && noc1out_ready
   && (req_source == 1'b0))
      noc1encoder_noc1buffer_req_ack = 1'b1;
   else
      noc1encoder_noc1buffer_req_ack = 1'b0;

   // ack logic to CSM
   noc1encoder_csm_req_ack = 0;
   if (csm_noc1encoder_req_val && (flit_state == msg_length) && noc1out_ready 
   && (req_source == 1'b1))
      noc1encoder_csm_req_ack = 1'b1;
   else
      noc1encoder_csm_req_ack = 1'b0;
end

always @ *
begin
   // hook for dmbr
   l15_dmbr_l1missIn = 0;
   l15_dmbr_l1missTag = 0;

   // trin: timing fix
   // removing CSM requests out of the requests to DMBR
   // if (sending && noc1out_ready && (flit_state == msg_length))
   if (noc1encoder_noc1buffer_req_ack)
   begin
      if (req_type == 5'd2 ||
         req_type == 5'd3 ||
         req_type == 5'd5 ||
         req_type == 5'd6 ||
         req_type == 5'd7 ||
         req_type == 5'd8 ||
         req_type == 5'd10 ||
         req_type == 5'd11 ||
         req_type == 5'd12 ||
         req_type == 5'd13 ||
         req_type == 5'd14 ||
         req_type == 5'd15 ||
         req_type == 5'd16 ||
         req_type == 5'd17)
      begin
         l15_dmbr_l1missIn = 1'b1;
         l15_dmbr_l1missTag = msg_mshrid[4-1:0]; // TODO: might be wrong please contact Tri
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
//  Filename      : noc2decoder.v
//  Created On    : 2014-03-03 22:08:49
//  Last Modified : 2015-01-27 16:48:19
//  Revision      :
//
//  Description   :
//
//
//==================================================================================================
//noc2decoder.v

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






module noc2decoder(
    input wire clk,
    input wire rst_n,

    input wire [511:0] noc2_data,
    input wire noc2_data_val,
    input wire l15_noc2decoder_ack,
    input wire l15_noc2decoder_header_ack,

    output reg noc2_data_ack,

    output reg                       l15_dmbr_l2responseIn,
    output reg                       l15_dmbr_l2missIn,
    output reg [4-1:0] l15_dmbr_l2missTag,

    output reg noc2decoder_l15_val,
    output reg [2-1:0] noc2decoder_l15_mshrid,
    output reg [0:0] noc2decoder_l15_threadid,
    output reg noc2decoder_l15_hmc_fill,
    output reg noc2decoder_l15_l2miss,
    output reg noc2decoder_l15_icache_type,
    output reg noc2decoder_l15_f4b,
    output reg [8-1:0] noc2decoder_l15_reqtype,
    output reg [2-1:0] noc2decoder_l15_ack_state,
    output reg [63:0] noc2decoder_l15_data_0,
    output reg [63:0] noc2decoder_l15_data_1,
    output reg [63:0] noc2decoder_l15_data_2,
    output reg [63:0] noc2decoder_l15_data_3,
    output reg [39:0] noc2decoder_l15_address,
    output reg [3:0] noc2decoder_l15_fwd_subcacheline_vector,
    output reg [3-1:0] noc2decoder_l15_csm_mshrid,
    output reg [(14+8+8)-1:0] noc2decoder_l15_src_homeid
    );


reg is_message_new;
reg is_message_new_next;
always @ (posedge clk)
begin
    if (!rst_n)
      is_message_new <= 1'b1;
    else
      is_message_new <= is_message_new_next;
end

reg [8-1:0] noc2_mshrid;
always @ *
begin
    noc2_data_ack = l15_noc2decoder_ack;
    noc2decoder_l15_val = noc2_data_val && is_message_new;

    // these are shared by both requests and replies from L2
    noc2decoder_l15_reqtype = noc2_data[21:14];
    noc2_mshrid = noc2_data[13:6];
    noc2decoder_l15_mshrid = noc2_mshrid[2-1:0];
    noc2decoder_l15_csm_mshrid = noc2_mshrid[3-1:0];
    // the threadid is encoded in the mshrid sent to L2, is the next L15_THREADID_WIDTH bits after the first L15_MSHR_ID_WIDTH bits
    noc2decoder_l15_threadid = noc2_mshrid[2+1 -1 -: 1];
    noc2decoder_l15_hmc_fill = noc2_mshrid[8-1];

    noc2decoder_l15_l2miss = noc2_data[3];
    noc2decoder_l15_icache_type = noc2_data[75];
    noc2decoder_l15_f4b = 0;
    noc2decoder_l15_ack_state = noc2_data[5:4];
    noc2decoder_l15_fwd_subcacheline_vector = noc2_data[79:76];
    noc2decoder_l15_address = noc2_data[119:80];
    noc2decoder_l15_data_0 = noc2_data[2*64 - 1 -: 64];
    noc2decoder_l15_data_1 = noc2_data[3*64 - 1 -: 64];
    noc2decoder_l15_data_2 = noc2_data[4*64 - 1 -: 64];
    noc2decoder_l15_data_3 = noc2_data[5*64 - 1 -: 64];

    noc2decoder_l15_src_homeid = 0;
    noc2decoder_l15_src_homeid[8+8-1:8] = noc2_data[169:162];
    noc2decoder_l15_src_homeid[8-1:0] = noc2_data[177:170];
    noc2decoder_l15_src_homeid[((14+8+8)-1):(8+8)] = noc2_data[191:178];

    // is_message_new is 1 on a new message, even if it's not valid yet
    // when header ack is received, it becomes 0 until the next "new" message
    // otherwise retain the newness value
    is_message_new_next = l15_noc2decoder_ack ? 1'b1 :
                                 l15_noc2decoder_header_ack ? 1'b0 : is_message_new;
end


// fix timing for dmbr by putting flops on the response
reg dmbr_response_val_next;
reg dmbr_l2_miss_next;
reg [4-1:0]dmbr_l2_miss_mshrid_next;
reg dmbr_response_val;
reg dmbr_l2_miss;
reg [4-1:0]dmbr_l2_miss_mshrid;

always @ (posedge clk)
begin
    if (!rst_n)
    begin
        dmbr_response_val = 0;
        dmbr_l2_miss = 0;
        dmbr_l2_miss_mshrid = 0;
    end
    else
    begin
        dmbr_response_val = dmbr_response_val_next;
        dmbr_l2_miss = dmbr_l2_miss_next;
        dmbr_l2_miss_mshrid = dmbr_l2_miss_mshrid_next;
    end
end


always @ *
begin
    // dmbr hook
    dmbr_response_val_next = 0;
    dmbr_l2_miss_next = 0;
    dmbr_l2_miss_mshrid_next = 0;
    if (l15_noc2decoder_ack)
    begin
        if (noc2decoder_l15_reqtype == 8'd29)
        begin
            dmbr_response_val_next = 1'b1;
            dmbr_l2_miss_next = noc2decoder_l15_l2miss;
            dmbr_l2_miss_mshrid_next = {1'b0, noc2decoder_l15_threadid, noc2decoder_l15_mshrid};
        end
    end

    l15_dmbr_l2responseIn = dmbr_response_val;
    l15_dmbr_l2missIn = dmbr_l2_miss;
    l15_dmbr_l2missTag = dmbr_l2_miss_mshrid;
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
//  Filename      : noc3buffer.v
//  Created On    : 2014-02-05 20:06:27
//  Revision      :
//  Author        : Tri Nguyen
//  Company       : Princeton University
//  Email         : trin@princeton.edu
//
//  Description   :
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









































































































































































































































































































































































































































































































































































































module noc3buffer(
    input wire clk,
    input wire rst_n,

    input wire l15_noc3encoder_req_val,
    input wire [3-1:0] l15_noc3encoder_req_type,
    input wire [63:0] l15_noc3encoder_req_data_0,
    input wire [63:0] l15_noc3encoder_req_data_1,
    input wire [2-1:0] l15_noc3encoder_req_mshrid,
    input wire [0:0] l15_noc3encoder_req_threadid,
    input wire [1:0] l15_noc3encoder_req_sequenceid,
    input wire [39:0] l15_noc3encoder_req_address,
    input wire l15_noc3encoder_req_with_data,
    input wire l15_noc3encoder_req_was_inval,
    input wire [3:0] l15_noc3encoder_req_fwdack_vector,
    input wire [(14+8+8)-1:0] l15_noc3encoder_req_homeid,

    output reg noc3buffer_noc3encoder_req_val,
    output reg [3-1:0] noc3buffer_noc3encoder_req_type,
    output reg [63:0] noc3buffer_noc3encoder_req_data_0,
    output reg [63:0] noc3buffer_noc3encoder_req_data_1,
    output reg [2-1:0] noc3buffer_noc3encoder_req_mshrid,
    output reg [1:0] noc3buffer_noc3encoder_req_sequenceid,
    output reg [0:0] noc3buffer_noc3encoder_req_threadid,
    output reg [39:0] noc3buffer_noc3encoder_req_address,
    output reg noc3buffer_noc3encoder_req_with_data,
    output reg noc3buffer_noc3encoder_req_was_inval,
    output reg [3:0] noc3buffer_noc3encoder_req_fwdack_vector,
    output reg [(14+8+8)-1:0] noc3buffer_noc3encoder_req_homeid,


    input wire noc3encoder_noc3buffer_req_ack,
    output reg noc3buffer_l15_req_ack

   );


// reg l15_noc3encoder_req_val_buf;
reg buffer_val;
reg buffer_val_next;
reg new_buffer;
reg [3-1:0] l15_noc3encoder_req_type_buf;
reg [63:0] l15_noc3encoder_req_data_0_buf;
reg [63:0] l15_noc3encoder_req_data_1_buf;
reg [2-1:0] l15_noc3encoder_req_mshrid_buf;
reg [1:0] l15_noc3encoder_req_threadid_buf;
reg [1:0] l15_noc3encoder_req_sequenceid_buf;
reg [39:0] l15_noc3encoder_req_address_buf;
reg l15_noc3encoder_req_with_data_buf;
reg l15_noc3encoder_req_was_inval_buf;
reg [3:0] l15_noc3encoder_req_fwdack_vector_buf;
reg [(14+8+8)-1:0] l15_noc3encoder_req_homeid_buf;

always @ (posedge clk)
begin
    if (!rst_n)
    begin
        buffer_val <= 1'b0;
    end
    else
    begin
        buffer_val <= buffer_val_next;
        if (new_buffer)
        begin
            l15_noc3encoder_req_type_buf <= l15_noc3encoder_req_type;
            l15_noc3encoder_req_data_0_buf <= l15_noc3encoder_req_data_0;
            l15_noc3encoder_req_data_1_buf <= l15_noc3encoder_req_data_1;
            l15_noc3encoder_req_mshrid_buf <= l15_noc3encoder_req_mshrid;
            l15_noc3encoder_req_threadid_buf <= l15_noc3encoder_req_threadid;
            l15_noc3encoder_req_sequenceid_buf <= l15_noc3encoder_req_sequenceid;
            l15_noc3encoder_req_address_buf <= l15_noc3encoder_req_address;
            l15_noc3encoder_req_with_data_buf <= l15_noc3encoder_req_with_data;
            l15_noc3encoder_req_was_inval_buf <= l15_noc3encoder_req_was_inval;
            l15_noc3encoder_req_fwdack_vector_buf <= l15_noc3encoder_req_fwdack_vector;
            l15_noc3encoder_req_homeid_buf <= l15_noc3encoder_req_homeid;
        end
    end
end

always @ *
begin
    noc3buffer_noc3encoder_req_val = buffer_val;
    noc3buffer_noc3encoder_req_type = l15_noc3encoder_req_type_buf;
    noc3buffer_noc3encoder_req_data_0 = l15_noc3encoder_req_data_0_buf;
    noc3buffer_noc3encoder_req_data_1 = l15_noc3encoder_req_data_1_buf;
    noc3buffer_noc3encoder_req_mshrid = l15_noc3encoder_req_mshrid_buf;
    noc3buffer_noc3encoder_req_threadid = l15_noc3encoder_req_threadid_buf;
    noc3buffer_noc3encoder_req_sequenceid = l15_noc3encoder_req_sequenceid_buf;
    noc3buffer_noc3encoder_req_address = l15_noc3encoder_req_address_buf;
    noc3buffer_noc3encoder_req_with_data = l15_noc3encoder_req_with_data_buf;
    noc3buffer_noc3encoder_req_was_inval = l15_noc3encoder_req_was_inval_buf;
    noc3buffer_noc3encoder_req_fwdack_vector = l15_noc3encoder_req_fwdack_vector_buf;
    noc3buffer_noc3encoder_req_homeid = l15_noc3encoder_req_homeid_buf;
end

// val/ack logic

// buffer_l15_ack is 1 only when req is valid, and buffer_val is 0 or noc3encoder_ack is 1
// ie buffer_l15_ack = l15_req && (!buffer_val || noc3encoder_ack);
// case 1: buffer not valid, accepting new req
// case 2: buffer is valid, but accepting new req because noc3 is done with current buffer

// logic for accepting new req: buffer_l15_ack

// buffer_val_next
// = 1 when buffer_l15_ack
// else = 0 when noc3encoder_ack
// else = buffer_val

always @ *
begin
    noc3buffer_l15_req_ack = 1'b0;
    if (l15_noc3encoder_req_val && (!buffer_val || noc3encoder_noc3buffer_req_ack))
        noc3buffer_l15_req_ack = 1'b1;

    new_buffer = noc3buffer_l15_req_ack;

    buffer_val_next = 1'b0;
    if (noc3buffer_l15_req_ack)
        buffer_val_next = 1'b1;
    else if (noc3encoder_noc3buffer_req_ack)
        buffer_val_next = 1'b0;
    else
        buffer_val_next = buffer_val;
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
//  Filename      : noc3encoder.v
//  Created On    : 2014-02-05 20:06:27
//  Revision      :
//  Author        : Tri Nguyen
//  Company       : Princeton University
//  Email         : trin@princeton.edu
//
//  Description   :
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









































































































































































































































































































































































































































































































































































































module noc3encoder(
    input wire clk,
    input wire rst_n,

    input wire l15_noc3encoder_req_val,
    input wire [3-1:0] l15_noc3encoder_req_type,
    input wire [63:0] l15_noc3encoder_req_data_0,
    input wire [63:0] l15_noc3encoder_req_data_1,
    input wire [2-1:0] l15_noc3encoder_req_mshrid,
    input wire [0:0] l15_noc3encoder_req_threadid,
    input wire [1:0] l15_noc3encoder_req_sequenceid,
    input wire [39:0] l15_noc3encoder_req_address,
    input wire l15_noc3encoder_req_with_data,
    // input wire l15_noc3encoder_req_fwdack_hit,
    input wire l15_noc3encoder_req_was_inval,
    input wire [3:0] l15_noc3encoder_req_fwdack_vector,
    input wire [(14+8+8)-1:0] l15_noc3encoder_req_homeid,
    input wire [14-1:0] chipid,
    input wire [8-1:0] coreid_x,
    input wire [8-1:0] coreid_y,

    input wire noc3out_ready,

    output reg noc3encoder_l15_req_ack,

    output reg noc3encoder_noc3out_val,
    output reg [63:0] noc3encoder_noc3out_data
   );


reg [63:0] flit;
reg [4-1:0] flit_state;
reg [4-1:0] flit_state_next;

reg [40-1:0] address;
reg [8-1:0] dest_l2_xpos;
reg [8-1:0] dest_l2_ypos;
reg [14-1:0] dest_chipid;
reg [4-1:0] dest_fbits;
reg [8-1:0] src_l2_xpos;
reg [8-1:0] src_l2_ypos;
reg [14-1:0] src_chipid;
reg [4-1:0] src_fbits;
reg [8-1:0] msg_length;
reg [8-1:0] msg_type;
reg [8-1:0] msg_mshrid;
reg [2-1:0] msg_mesi;
// reg [`MSG_SUBLINE_ID_WIDTH-1:0] msg_subline_id;
reg [1-1:0] msg_last_subline;
reg [5:0] msg_options_1;
reg [15:0] msg_options_2;
reg [29:0] msg_options_3;
reg [5:0] msg_options_4;
reg [1-1:0] msg_cache_type;
reg [4-1:0] msg_subline_vector;
reg [3-1:0] msg_data_size;

reg sending;
reg is_request;
reg is_response;
// reg msg_last_subline;
reg [1:0] last_subcacheline_id;


// 9/24/14: add buffer between dcache and output
reg [63:0] l15_noc3encoder_req_data_0_f;
// reg [63:0] l15_noc3encoder_req_data_1_f;
always @ (posedge clk)
begin
    if (!rst_n)
    begin
        l15_noc3encoder_req_data_0_f <= 0;
        // l15_noc3encoder_req_data_1_f <= 0;
    end
    else
    begin
        if (l15_noc3encoder_req_val && is_request && flit_state_next == 4'd3)
            l15_noc3encoder_req_data_0_f <= l15_noc3encoder_req_data_0;
        else if (l15_noc3encoder_req_val && is_request && flit_state_next == 4'd4)
            l15_noc3encoder_req_data_0_f <= l15_noc3encoder_req_data_1;
        else if (l15_noc3encoder_req_val && is_response && flit_state_next == 4'd1)
            l15_noc3encoder_req_data_0_f <= l15_noc3encoder_req_data_0;
        else if (l15_noc3encoder_req_val && is_response && flit_state_next == 4'd2)
            l15_noc3encoder_req_data_0_f <= l15_noc3encoder_req_data_1;
    end
end

always @ (posedge clk)
begin
    if (!rst_n)
    begin
        flit_state <= 0;
    end
    else
    begin
        flit_state <= flit_state_next;
    end
end

always @ *
begin
    is_request = (l15_noc3encoder_req_type == 3'd1);
    is_response = !is_request;

    last_subcacheline_id =  (l15_noc3encoder_req_fwdack_vector[3] == 1'b1) ? 2'b11 :
                            (l15_noc3encoder_req_fwdack_vector[2] == 1'b1) ? 2'b10 :
                            (l15_noc3encoder_req_fwdack_vector[1] == 1'b1) ? 2'b01 :
                                                                             2'b00 ;
    msg_last_subline = last_subcacheline_id == l15_noc3encoder_req_sequenceid;

    address = l15_noc3encoder_req_address;

    // old non-csm implementation
    // if (`HOME_ID_MASK_X_ENABLE)
    //     dest_l2_xpos = l15_noc3encoder_req_address[`HOME_ID_MASK_X];
    // else
    //     dest_l2_xpos = 0;
    // if (`HOME_ID_MASK_Y_ENABLE)
    //     dest_l2_ypos = l15_noc3encoder_req_address[`HOME_ID_MASK_Y];
    // else
    //     dest_l2_ypos = 0;
    // dest_chipid = 0;

    dest_l2_xpos = l15_noc3encoder_req_homeid[8-1:0];
    dest_l2_ypos = l15_noc3encoder_req_homeid[8+8-1:8];
    dest_chipid = l15_noc3encoder_req_homeid[((14+8+8)-1):(8+8)];
    dest_fbits = 4'd0;

    src_l2_xpos = coreid_x;
    src_l2_ypos = coreid_y;
    src_chipid = chipid;
    src_fbits = 4'd0;

    msg_length = 0;
    msg_type = 0;
    msg_mshrid = {l15_noc3encoder_req_threadid, l15_noc3encoder_req_mshrid};
    msg_mesi = 0;
    // msg_l2_miss = 0;
    // msg_subline_id = 0;
    msg_cache_type = 0;
    msg_subline_vector = 0; // always 0 for requests
    msg_data_size = 0;

    sending = l15_noc3encoder_req_val;
    noc3encoder_noc3out_val = sending;

    case (l15_noc3encoder_req_type)
        3'd1:
        begin
            // specify address (should be specified by default)
            msg_type = 8'd12;
            msg_length = 4; // 2 extra req headers + 2 data (128b)
            // msg_cache_type = `MSG_CACHE_TYPE_DATA;
        end
        3'd3:
        begin
            msg_type = 8'd21;
            if (l15_noc3encoder_req_with_data)
            begin
                msg_length = 2;
                msg_data_size = 3'b110;
            end
            else
                msg_length = 0;
        end
        3'd2:
        begin
            // specify sequence id + if is last
            if (l15_noc3encoder_req_was_inval)
               msg_type = 8'd23;
            else
               msg_type = 8'd22;
           
            if (l15_noc3encoder_req_with_data)
            begin
                msg_length = 2;
                msg_data_size = 3'b110;
            end
            else
                msg_length = 0;
        end
        3'd4:
        begin
            // specify sequence id + if is last
            msg_type = 8'd23;
            msg_length = 0;
        end
    endcase

    msg_options_1 = 0;
    msg_options_2 = 0;
    msg_options_3 = 0;
    msg_options_4 = 0;

    // trin: line coverage: 16B transaction apparently does not happen with the T1 core
    if (msg_data_size == 3'b111)
        msg_options_2[10:8] = 3'b101;
    else
        msg_options_2[10:8] = msg_data_size;
    msg_options_2[11] = msg_cache_type;
    msg_options_2[15:12] = msg_subline_vector;

    msg_options_4[0] = msg_last_subline || (l15_noc3encoder_req_type == 3'd4);
    msg_options_4[2:1] = l15_noc3encoder_req_sequenceid;
    // does not need to specify cache line state
    // no l2miss


    // flit filling logic
    flit[64-1:0] = 0; // so that the flit is not a latch
    if (is_request)
    begin
        if (flit_state == 4'd0)
        begin
            flit[63:50] = dest_chipid;
            flit[49:42] = dest_l2_xpos;
            flit[41:34] = dest_l2_ypos;
            flit[33:30] = dest_fbits;
            flit[29:22] = msg_length;
            flit[21:14] = msg_type;
            flit[13:6] = msg_mshrid;
            flit[5:0] = msg_options_1;
        end
        else if (flit_state == 4'd1)
        begin
            flit[((16 + 40 - 1)):(16)] = address;
            flit[15:0] = msg_options_2;
            // trin: line coverage: 16B transaction apparently does not happen with the T1 core
            if (msg_data_size == 3'b111)
                flit[10:8] = 3'b101;
            else
                flit[10:8] = msg_data_size;
        end
        else if (flit_state == 4'd2)
        begin
            flit[63:50] = src_chipid;
            flit[49:42] = src_l2_xpos;
            flit[41:34] = src_l2_ypos;
            flit[33:30] = src_fbits;
            flit[29:0] = msg_options_3;
        end
        else if (flit_state == 4'd3)
        begin
            flit[64-1:0] = l15_noc3encoder_req_data_0_f;
        end
        else if (flit_state == 4'd4)
        begin
            flit[64-1:0] = l15_noc3encoder_req_data_0_f;
        end
    end
    else if (is_response)
    begin
        if (flit_state == 4'd0)
        begin
            flit[63:50] = dest_chipid;
            flit[49:42] = dest_l2_xpos;
            flit[41:34] = dest_l2_ypos;
            flit[33:30] = dest_fbits;
            flit[29:22] = msg_length;
            flit[21:14] = msg_type;
            flit[13:6] = msg_mshrid;
            flit[5:0] = msg_options_4;
        end
        else if (flit_state == 4'd1)
        begin
            flit[64-1:0] = l15_noc3encoder_req_data_0_f;
        end
        else if (flit_state == 4'd2)
        begin
            flit[64-1:0] = l15_noc3encoder_req_data_0_f;
        end
    end

    noc3encoder_noc3out_data = flit;

    // next flit state logic
    flit_state_next = flit_state;
    if (sending)
    begin
        if (is_request || is_response)
            if (noc3out_ready)
            begin
                if (flit_state != msg_length)
                    flit_state_next = flit_state + 1;
                else
                    flit_state_next = 4'd0;
            end
            else
                flit_state_next = flit_state;
    end
    else
        flit_state_next = 4'd0;

    // ack logic to L2
    if (l15_noc3encoder_req_val && flit_state == msg_length && noc3out_ready)
        noc3encoder_l15_req_ack = 1'b1;
    else
        noc3encoder_l15_req_ack = 1'b0;
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
//  Filename      : pcx_buffer.v
//  Created On    : 2014-03-03 20:20:43
//  Last Modified : 2018-11-14 19:18:52
//  Revision      :
//
//  Description   :
//
//
//==================================================================================================
//pcx_buffer.v

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



//`default_nettype none
module pcx_buffer(
   input wire clk,
   input wire rst_n,

   // Inputs from sparc core
   input wire [4:0] spc_uncore_req,
   input wire spc_uncore_atomic_req,
   input wire [124-1:0] spc_uncore_data,
   input wire [33-1:0] spc_uncore_csm_data,

   // ack from the L1.5 that the entry is processed
   input wire pcxdecoder_pcxbuf_ack,

   // grant to sparc core
   output reg [4:0] uncore_spc_grant,

   //
   output reg [124-1:0] pcxbuf_pcxdecoder_data,
   output reg [124-1:0] pcxbuf_pcxdecoder_data_buf1,
   output reg [33-1:0] pcxbuf_pcxdecoder_csm_data,
   output reg pcxbuf_pcxdecoder_valid
   // output reg pcx_req_squashed
   );

reg [124-1:0]   buffer[0:1];
reg [124-1:0]   buffer_next[0:1];
reg                    buffer_atomic[0:1];
reg                    buffer_atomic_next[0:1];
reg                    buffer_atomic_next_next;
// reg [`PCX_WIDTH-1:0]   buf0, buf1;
// wire [`PCX_WIDTH-1:0] buf0_next, buf1_next;
// wire               buf0_val, buf1_val;
reg [124-1:0]   buffer_csm_data[0:1];
reg [124-1:0]   buffer_csm_data_next[0:1];

reg atomic_req_second_packet_coming;
reg atomic_req_second_packet_coming_next;
reg atomic_ack_second;
reg atomic_ack_second_next;

reg buffer_val [0:1];
reg buffer_val_next [0:1];
reg read_pos;
reg write_pos;
reg read_pos_next;
reg write_pos_next;

reg write_req;
reg write_req_next;
reg read_ack;

// reg invalid_packet;
// reg invalid_packet_next;
// reg invalid_packet_acked;

reg is_buffer_full;
reg is_buffer_full_1back;
// reg is_buffer_full_2back;
reg is_req_squashed;
reg [4:0] uncore_spc_grant_next;

// Reset logic & sequential
always @ (posedge clk)
if (~rst_n)
begin
   // buf0 <= 1'b0;
   // buf1 <= 1'b0;
   buffer[0] <= 1'b0;
   buffer[1] <= 1'b0;
   read_pos <= 1'b0;
   write_pos <= 1'b0;
   write_req <= 1'b0;
   buffer_val[0] <= 1'b0;
   buffer_val[1] <= 1'b0;
   buffer_atomic[0] <= 1'b0;
   buffer_atomic[1] <= 1'b0;
   buffer_atomic_next_next <= 1'b0;
   atomic_req_second_packet_coming <= 1'b0;
   atomic_ack_second <= 1'b0;
   // invalid_packet <= 1'b0;
   // is_buffer_full_2back <= 1'b0;
   is_buffer_full_1back <= 1'b0;
   uncore_spc_grant <= 0;
end
else
begin
   // if (buf0_val && pcxdecoder_pcxbuf_ack)
   // begin
      // buf0 <= buf0_next;
      // buf1 <= buf1_next;
      buffer[0] <= buffer_next[0];
      buffer[1] <= buffer_next[1];
      buffer_csm_data[0] <= buffer_csm_data_next[0];
      buffer_csm_data[1] <= buffer_csm_data_next[1];
      read_pos <= read_pos_next;
      write_pos <= write_pos_next;
      write_req <= write_req_next;
      buffer_val[0] <= buffer_val_next[0];
      buffer_val[1] <= buffer_val_next[1];
      buffer_atomic[0] <= buffer_atomic_next[0];
      buffer_atomic[1] <= buffer_atomic_next[1];
      buffer_atomic_next_next <= spc_uncore_atomic_req && !is_req_squashed;
      atomic_req_second_packet_coming <= atomic_req_second_packet_coming_next;
      atomic_ack_second <= atomic_ack_second_next;
      // invalid_packet <= invalid_packet_next;
      // is_buffer_full_2back <= is_buffer_full_1back;
      is_buffer_full_1back <= is_buffer_full;
      uncore_spc_grant <= uncore_spc_grant_next;
   // end
end

// Combinational

always @ *
begin
   // write_req_next = spc_uncore_req[0] || atomic_req_second_packet_coming;
   // note: the core puts data one cycle after the request
   //       also, only one request for atomic request, but two cycles of data
   read_ack = pcxdecoder_pcxbuf_ack;
   buffer_val_next[0] = buffer_val[0];
   buffer_val_next[1] = buffer_val[1];
   buffer_next[0] = buffer[0];
   buffer_next[1] = buffer[1];
   buffer_atomic_next[0] = buffer_atomic[0];
   buffer_atomic_next[1] = buffer_atomic[1];
   buffer_csm_data_next[0] = buffer_csm_data[0];
   buffer_csm_data_next[1] = buffer_csm_data[1];

   read_pos_next = read_pos;
   write_pos_next = write_pos;
   atomic_ack_second_next = 0;

   if (write_req)
   begin
      // $write("DEBUG: received pcx packet");
      buffer_next[write_pos] = spc_uncore_data;
      buffer_csm_data_next[write_pos] = spc_uncore_csm_data;
      buffer_atomic_next[write_pos] = buffer_atomic_next_next;
      buffer_val_next[write_pos] = 1'b1;
      write_pos_next = write_pos ^ 1'b1;
   end

   if (read_ack)
   begin
      if (buffer_atomic[read_pos] == 1'b1)
      begin
         read_pos_next = read_pos;
         atomic_ack_second_next = 1'b1;
         buffer_val_next[0] = 1'b0;
         buffer_val_next[1] = 1'b0;
      end
      else
      begin
         read_pos_next = read_pos^1'b1;
         buffer_val_next[read_pos] = 1'b0;
      end
   end

   // move this to the end of the process
   // otherwise msim complains about potentially uninitialized "write_pos_next"
   is_buffer_full = (write_pos_next == read_pos) && (buffer_val[read_pos] == 1'b1) && (uncore_spc_grant[0] != 1'b1);
   // is_buffer_full = (write_pos_next == read_pos) && (buffer_val[read_pos] == 1'b1);
   is_req_squashed = is_buffer_full && spc_uncore_req[0];
   // pcx_req_squashed = is_req_squashed;
   atomic_req_second_packet_coming_next = spc_uncore_atomic_req && !is_req_squashed;
   pcxbuf_pcxdecoder_data = buffer[read_pos];
   pcxbuf_pcxdecoder_data_buf1 = buffer[read_pos^1'b1];
   pcxbuf_pcxdecoder_valid = buffer_val[read_pos];
   pcxbuf_pcxdecoder_csm_data = buffer_csm_data[read_pos];
   write_req_next = (spc_uncore_req[0] && !is_req_squashed) || atomic_req_second_packet_coming;

end


always @ *
begin
   // invalid_packet_acked = 0;
   // if (invalid_packet && !pcxdecoder_pcxbuf_ack && !atomic_ack_second)
      // invalid_packet_acked = 1;

   // uncore_spc_grant = {4'b0, pcxdecoder_pcxbuf_ack|atomic_ack_second|invalid_packet};
   // uncore_spc_grant = {4'b0, pcxdecoder_pcxbuf_ack|atomic_ack_second};

   // this signal needs to be early; flopped
   uncore_spc_grant_next = {4'b0, pcxdecoder_pcxbuf_ack|atomic_ack_second};
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
//  Filename      : pcx_decoder.v
//  Created On    : 2014-03-03 22:08:49
//  Last Modified : 2015-01-22 17:20:15
//  Revision      :
//
//  Description   :
//
//
//==================================================================================================
//pcx_decoder.v

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



module pcx_decoder(
   input wire clk,
   input wire rst_n,

   input wire [124-1:0]   pcxbuf_pcxdecoder_data,
   input wire [124-1:0]   pcxbuf_pcxdecoder_data_buf1,
   input wire [33-1:0]   pcxbuf_pcxdecoder_csm_data,
   input wire                    pcxbuf_pcxdecoder_valid,
   input wire                    l15_pcxdecoder_ack,
   input wire                    l15_pcxdecoder_header_ack,

   output reg        pcxdecoder_pcxbuf_ack,
   output reg [4:0]  pcxdecoder_l15_rqtype,
   output reg [4-1:0]  pcxdecoder_l15_amo_op,
   output reg        pcxdecoder_l15_nc,
   output reg [2:0]  pcxdecoder_l15_size,
   output reg [0:0]  pcxdecoder_l15_threadid,
   output reg        pcxdecoder_l15_prefetch,
   output reg        pcxdecoder_l15_invalidate_cacheline,
   output reg        pcxdecoder_l15_blockstore,
   output reg        pcxdecoder_l15_blockinitstore,
   output reg [1:0]  pcxdecoder_l15_l1rplway,
   output reg        pcxdecoder_l15_val,
   output reg [39:0] pcxdecoder_l15_address,
   output reg [63:0] pcxdecoder_l15_data,
   output reg [63:0] pcxdecoder_l15_data_next_entry,
   output reg [33-1:0] pcxdecoder_l15_csm_data
   );


wire [124-1:0] message = pcxbuf_pcxdecoder_data;

reg is_message_new;
reg is_message_new_next;
always @ (posedge clk)
begin
   if (!rst_n)
      is_message_new <= 1'b1;
   else
      is_message_new <= is_message_new_next;
end

always @ *
begin
   pcxdecoder_l15_val = pcxbuf_pcxdecoder_valid && message[123] && is_message_new;
   // pcxdecoder_l15_val = pcxbuf_pcxdecoder_valid;
   pcxdecoder_pcxbuf_ack = l15_pcxdecoder_ack || (pcxbuf_pcxdecoder_valid && !message[123]);
   pcxdecoder_l15_csm_data = pcxbuf_pcxdecoder_csm_data;

   pcxdecoder_l15_rqtype = message[122:118];
   pcxdecoder_l15_amo_op = 4'b0000;
   pcxdecoder_l15_nc = message[117];
   pcxdecoder_l15_threadid = message[113:112];
   pcxdecoder_l15_prefetch = message[110];
   pcxdecoder_l15_blockstore = message[110];
   pcxdecoder_l15_invalidate_cacheline = message[111];
   pcxdecoder_l15_blockinitstore = message[109];
   pcxdecoder_l15_l1rplway = message[108:107];
   pcxdecoder_l15_size = message[106:104];
   pcxdecoder_l15_address = message[103:64]; // 40b
   pcxdecoder_l15_data = message[63:0];
   pcxdecoder_l15_data_next_entry = pcxbuf_pcxdecoder_data_buf1[63:0];

   // is_message_new is 1 on a new message, even if it's not valid yet
   // when header ack is received, it becomes 0 until the next "new" message
   // otherwise retain the newness value
   is_message_new_next = l15_pcxdecoder_ack ? 1'b1 :
                         l15_pcxdecoder_header_ack ? 1'b0 : is_message_new;

   if (message[122:118] == 5'b00010)
   begin
      pcxdecoder_l15_rqtype = 5'b00110;
      pcxdecoder_l15_amo_op = 4'b1100;
   end
   else if (message[122:118] == 5'b00011)
   begin
      pcxdecoder_l15_rqtype = 5'b00110;
      pcxdecoder_l15_amo_op = 4'b1101;
   end
   else if (message[122:118] == 5'b00110)
   begin
      pcxdecoder_l15_rqtype = 5'b00110;
      pcxdecoder_l15_amo_op = 4'b0011;
   end
end
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


module pico_decoder(
    input wire         clk,
    input wire         rst_n,
    
    input wire         pico_mem_valid,
    input wire [31:0]  pico_mem_addr,
    input wire [ 3:0]  pico_mem_wstrb,
    
    input wire [31:0]  pico_mem_wdata,
    input wire [4-1:0] pico_mem_amo_op,
    input wire         l15_picodecoder_ack,
    input wire         l15_picodecoder_header_ack,
    
    // outputs pico uses                    
    output reg  [4:0]  picodecoder_l15_rqtype,
    output      [4-1:0] picodecoder_l15_amo_op,
    output reg  [2:0]  picodecoder_l15_size,
    output wire        picodecoder_l15_val,
    output wire [39:0] picodecoder_l15_address,
    output wire [63:0] picodecoder_l15_data,
    output wire        picodecoder_l15_nc,
    
    
    // outputs pico doesn't use                    
    output wire [0:0]  picodecoder_l15_threadid,
    output wire        picodecoder_l15_prefetch,
    output wire        picodecoder_l15_invalidate_cacheline,
    output wire        picodecoder_l15_blockstore,
    output wire        picodecoder_l15_blockinitstore,
    output wire [1:0]  picodecoder_l15_l1rplway,
    output wire [63:0] picodecoder_l15_data_next_entry,
    output wire [32:0] picodecoder_l15_csm_data
);
    localparam ACK_IDLE = 1'b0;
    localparam ACK_WAIT = 1'b1;

    assign picodecoder_l15_amo_op = pico_mem_amo_op;

    reg current_val;
    reg prev_val;
    // is this a new request from pico?
    wire new_request = current_val & ~prev_val;
    always @ (posedge clk)
    begin
        if (!rst_n) begin
           current_val <= 0;
           prev_val <= 0;
        end
        else begin
           current_val <= pico_mem_valid;
           prev_val <= current_val;
        end
    end 

    // are we waiting for an ack
    reg ack_reg;
    reg ack_next;
    always @ (posedge clk) begin
        if (!rst_n) begin
            ack_reg <= 0;
        end
        else begin
            ack_reg <= ack_next;
        end
    end
    always @ (*) begin
        // be careful with these conditionals.
        if (l15_picodecoder_ack) begin
            ack_next = ACK_IDLE;
        end
        else if (new_request) begin
            ack_next = ACK_WAIT;
        end
        else begin
            ack_next = ack_reg;
        end
    end

    
    // if we haven't got an ack and it's an old request, valid should be high
    // otherwise if we got an ack valid should be high only if we got a new
    // request
	assign picodecoder_l15_val = (ack_reg == ACK_WAIT) ? pico_mem_valid 
                                 : (ack_reg == ACK_IDLE) ? new_request
                                 : pico_mem_valid;

    reg [31:0] pico_wdata_flipped;

    // unused wires tie to zero
	assign picodecoder_l15_threadid = 1'b0;
	assign picodecoder_l15_prefetch = 1'b0;
	assign picodecoder_l15_csm_data = 33'b0;
	assign picodecoder_l15_data_next_entry = 64'b0;

	assign picodecoder_l15_blockstore = 1'b0;
	assign picodecoder_l15_blockinitstore = 1'b0;
	// is this set when something in the l1 gets replaced? pico has no cache
	assign picodecoder_l15_l1rplway = 2'b0;
	// will pico ever need to invalidate cachelines?
	assign picodecoder_l15_invalidate_cacheline = 1'b0;

    // logic to check if a request is new
	assign picodecoder_l15_address = {{8{pico_mem_addr[31]}}, pico_mem_addr};

   	assign picodecoder_l15_nc = pico_mem_addr[31] | (picodecoder_l15_rqtype == 5'b00110);

    assign picodecoder_l15_data = {pico_wdata_flipped, pico_wdata_flipped};
	// set rqtype specific data
	always @ *
	begin
        if (pico_mem_valid) begin
	        // store or atomic operation 
            if (pico_mem_wstrb) begin
	            picodecoder_l15_rqtype = 5'b00001;
                // endian wizardry
                pico_wdata_flipped = {pico_mem_wdata[7:0], pico_mem_wdata[15:8],
                                      pico_mem_wdata[23:16], pico_mem_wdata[31:24]};

                // if it's an atomic operation, modify the request type.
                // That's it
                if (pico_mem_amo_op != 4'b0000) begin
                    picodecoder_l15_rqtype = 5'b00110;
                end

                case(pico_mem_wstrb)
		            4'b1111: begin
		                picodecoder_l15_size = 3'b010;
		            end
		            4'b1100, 4'b0011: begin
		                picodecoder_l15_size = 3'b001;
		            end
		            4'b1000, 4'b0100, 4'b0010, 4'b0001: begin
		                picodecoder_l15_size = 3'b000;
		            end
		            // this should never happen
		            default: begin
		                picodecoder_l15_size = 0;
		            end
	            endcase
	        end
	        // load operation
	        else begin
	            pico_wdata_flipped = 32'b0;
                picodecoder_l15_rqtype = 5'b00000;
	            picodecoder_l15_size = 3'b010;
	        end 
        end
        else begin
            pico_wdata_flipped = 32'b0;
            picodecoder_l15_rqtype = 5'b0;
            picodecoder_l15_size = 3'b0;
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
//  Filename      : simplenocbuffer.v
//  Created On    : 2014-03-03 20:20:43
//  Last Modified : 2014-04-17 19:30:07
//  Revision      :
//
//  Description   :
//
//
//==================================================================================================
//simplenocbuffer.v

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
















































































































































































































































































































































































































































































































































































































































































































































































//`default_nettype none
module simplenocbuffer(
   input wire clk,
   input wire rst_n,
   input wire noc_in_val,
   input wire [64-1:0] noc_in_data,
   input wire msg_ack,
   output reg noc_in_rdy,
   output reg [511:0] msg,
   output reg msg_val
   );

reg [2:0] index;
reg [2:0] index_next;
reg [8-1:0] msg_len;
reg [2-1:0] state;
reg [2-1:0] state_next;
reg [64-1:0] buffer [0:7];
reg [64-1:0] buffer_next [0:7];

// Reset logic & sequential
always @ (posedge clk)
begin
   if (~rst_n)
   begin
      buffer[0] <= 1'b0;
      buffer[1] <= 1'b0;
      buffer[2] <= 1'b0;
      buffer[3] <= 1'b0;
      buffer[4] <= 1'b0;
      buffer[5] <= 1'b0;
      buffer[6] <= 1'b0;
      buffer[7] <= 1'b0;
      index <= 0;
      state <= 0;
   end
   else
   begin
      buffer[0] <= buffer_next[0];
      buffer[1] <= buffer_next[1];
      buffer[2] <= buffer_next[2];
      buffer[3] <= buffer_next[3];
      buffer[4] <= buffer_next[4];
      buffer[5] <= buffer_next[5];
      buffer[6] <= buffer_next[6];
      buffer[7] <= buffer_next[7];
      index <= index_next;
      state <= state_next;
   end
end

// Combinational
always @ *
begin
   msg[(0+1)*64 - 1 -: 64] = buffer[0];
   msg[(1+1)*64 - 1 -: 64] = buffer[1];
   msg[(2+1)*64 - 1 -: 64] = buffer[2];
   msg[(3+1)*64 - 1 -: 64] = buffer[3];
   msg[(4+1)*64 - 1 -: 64] = buffer[4];
   msg[(5+1)*64 - 1 -: 64] = buffer[5];
   msg[(6+1)*64 - 1 -: 64] = buffer[6];
   msg[(7+1)*64 - 1 -: 64] = buffer[7];
end

always @ *
begin
   index_next = index;
   state_next = 0;
   msg_val = 0;
   msg_len = 0;
   buffer_next[0] = buffer[0];
   buffer_next[1] = buffer[1];
   buffer_next[2] = buffer[2];
   buffer_next[3] = buffer[3];
   buffer_next[4] = buffer[4];
   buffer_next[5] = buffer[5];
   buffer_next[6] = buffer[6];
   buffer_next[7] = buffer[7];
   noc_in_rdy = 1'b0;

   if (state == 2'd0)
   begin
      noc_in_rdy = 1'b1;
      msg_len = noc_in_data[29:22];
      if (noc_in_val)
      begin
         buffer_next[0] = noc_in_data;
         if (msg_len == 0)
         begin
            state_next = 2'd2;
         end
         else
         begin
            state_next = 2'd1;
            index_next = index + 1;
         end
      end
   end
   else if (state == 2'd1)
   begin
      noc_in_rdy = 1'b1;
      msg_len = buffer[0][29:22];
      if (noc_in_val)
      begin
         buffer_next[index] = noc_in_data;
         if (index == msg_len)
         begin
            state_next = 2'd2;
         end
         else
         begin
            state_next = 2'd1;
            index_next = index + 1;
         end
      end
      else state_next = state;
   end
   else if (state == 2'd2)
   begin
      noc_in_rdy = 1'b0;
      msg_val = 1'b1;
      if (msg_ack)
      begin
         state_next = 2'd0;
         index_next = 0;
      end
      else
         state_next = 2'd2;
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
//  Filename      : l15_csm.v
//  Created On    : 2014-01-31 18:24:47
//  Revision      :
//  Author        : Tri Nguyen
//  Company       : Princeton University
//  Email         : trin@princeton.edu
//
//  Description   :
//
//
//==================================================================================================

//`timescale 1 ns / 10 ps
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











































































































































































































































































































































































































































































































































































































module l15_csm(
   input wire clk,
   input wire rst_n,
   input wire csm_en,

   // static system input / configure registers
   input wire [6-1:0] system_tile_count,
   input wire [2-1:0] home_alloc_method, 
   input wire [22-1:0] l15_hmt_base_reg,

   // interface to noc1buffer
   input wire [3-1:0] l15_csm_read_ticket,
   input wire [3-1:0] l15_csm_clear_ticket,
   input wire l15_csm_clear_ticket_val,
   output wire [(14+8+8)-1:0] csm_l15_read_res_data,
   output wire csm_l15_read_res_val,

   // interface with pipeline
   input wire [40-1:0] l15_csm_req_address_s2,
   input wire l15_csm_req_val_s2,
   input wire l15_csm_stall_s3,
   input wire [3-1:0] l15_csm_req_ticket_s2,
   input wire  l15_csm_req_type_s2,     //0 for load, 1 for store
   input wire [128-1:0] l15_csm_req_data_s2,
   input wire [33-1:0] l15_csm_req_pcx_data_s2, //
   output reg csm_l15_res_val_s3,
   output reg [63:0] csm_l15_res_data_s3,

   // noc1 interface
   input wire noc1encoder_csm_req_ack,
   output wire csm_noc1encoder_req_val,
   output wire [5-1:0] csm_noc1encoder_req_type,
   output wire [3-1:0] csm_noc1encoder_req_mshrid,
   output wire [40-1:0] csm_noc1encoder_req_address,
   output wire csm_noc1encoder_req_non_cacheable,
   output wire  [3-1:0] csm_noc1encoder_req_size

);

// storage for ticketed ghid translations
reg [(14+8+8)-1:0] ghid_ticketed_cache [8-1:0];
reg [16-1:0] ghid_ticketed_cache_addr [8-1:0];
reg [8-1:0] ghid_ticketed_cache_val;
// write to the ticketed ghid trans
reg [3-1:0] write_index_s2;
reg write_val_s2;
reg [3-1:0] read_index_s2;
reg read_val_s2;

reg [6-1:0] num_homes_s2;
wire [6-1:0] lhid_s2;
reg [(14+8+8)-1:0] ghid_s2;
reg ghid_val_s2;

//Stage 2

reg diag_en_s2;
reg flush_en_s2;
reg rd_en_s2;
reg wr_en_s2;
reg [8-1:0] addr_type_s2;
reg [16-1:0] addr_in_s2;
reg [16-1:0] addr_in_s2_next;
reg [6-1:0] home_addr_bits_s2;
reg special_l2_addr_s2;

reg [6-1:0] l15_csm_clump_tile_count_s2;
reg [10-1:0] l15_csm_req_clump_id_s2;
reg [14-1:0] l15_csm_chipid_s2;
reg [8-1:0] l15_csm_x_s2;
reg [8-1:0] l15_csm_y_s2;
reg l15_csm_clump_sel_s2;


always @ *
begin
    l15_csm_clump_sel_s2 = l15_csm_req_pcx_data_s2[32];
end


always @ *
begin
    if (l15_csm_clump_sel_s2 == 1'b0)
    begin
        l15_csm_clump_tile_count_s2 = l15_csm_req_pcx_data_s2[21:16];
        l15_csm_req_clump_id_s2 =  l15_csm_req_pcx_data_s2[31:22];
    end
    else
    begin
        l15_csm_clump_tile_count_s2 = 0;
        l15_csm_req_clump_id_s2 =  0;
    end
end

always @ *
begin
    if (l15_csm_clump_sel_s2 == 1'b1)
    begin
        l15_csm_chipid_s2 = l15_csm_req_pcx_data_s2[29:16];
        l15_csm_x_s2 =  l15_csm_req_pcx_data_s2[7:0];
        l15_csm_y_s2 =  l15_csm_req_pcx_data_s2[15:8];
    end
    else
    begin
        l15_csm_chipid_s2 = 0;
        l15_csm_x_s2 =  0;
        l15_csm_y_s2 =  0;
    end
end


always @ *
begin
    addr_type_s2 = l15_csm_req_address_s2[39:32];
end

always @ *
begin
    //special l2 addresses start with 0xA
    special_l2_addr_s2 = (l15_csm_req_address_s2[39:36] == 4'b1010);
end

always @ *
begin
    if (special_l2_addr_s2)
    begin
        home_addr_bits_s2 = l15_csm_req_address_s2[6+23 : 24];
    end
    else
    begin
        case (home_alloc_method)
        2'd0:
        begin
            home_addr_bits_s2 = l15_csm_req_address_s2[6+5 : 6];
        end
        2'd1:
        begin
            home_addr_bits_s2 = l15_csm_req_address_s2[6+13 : 14];
        end
        2'd2:
        begin
            home_addr_bits_s2 = l15_csm_req_address_s2[6+23 : 24];
        end
        2'd3:
        begin
            home_addr_bits_s2 = (l15_csm_req_address_s2[6+5 : 6] ^ l15_csm_req_address_s2[6+13 : 14]);
        end
        endcase
    end
end


always @ *
begin
    diag_en_s2 = l15_csm_req_val_s2 && (addr_type_s2 == 8'hb2);
    flush_en_s2 = l15_csm_req_val_s2 && (addr_type_s2 == 8'hb5);
    rd_en_s2 = l15_csm_req_val_s2 && (l15_csm_req_type_s2 == 1'b0)
            && ~(~diag_en_s2 && ~flush_en_s2 && (l15_csm_clump_sel_s2 == 1'b1));
    wr_en_s2 = l15_csm_req_val_s2 && (l15_csm_req_type_s2 == 1'b1);
end

always @ *
begin
   if(diag_en_s2 || flush_en_s2)
   begin
      addr_in_s2 = l15_csm_req_address_s2[16+3:4];
   end
   else
   begin
      addr_in_s2 = {l15_csm_req_clump_id_s2, lhid_s2};
   end
end

always @ *
begin
   if (csm_en)
   begin
      num_homes_s2 = l15_csm_clump_tile_count_s2;
   end
   else
   begin
      num_homes_s2 = system_tile_count;
   end
end

l15_home_encoder    l15_home_encoder(
   // .clk            (clk),
   // .rst_n          (rst_n),
   .home_in        (home_addr_bits_s2),
   .num_homes      (num_homes_s2),
   .lhid_out       (lhid_s2)
);

always @ *
begin
   // write to the ticketed storage. This would normally be written to when the module
   //  gets back the response from memory
   write_index_s2 = l15_csm_req_ticket_s2;
   write_val_s2 = l15_csm_req_val_s2 && wr_en_s2 && (~diag_en_s2) && (~flush_en_s2);
end

always @ *
begin
   read_index_s2 = l15_csm_req_ticket_s2;
   read_val_s2 = l15_csm_req_val_s2 && (l15_csm_req_type_s2 == 1'b0);
end




always @ *
begin  
    case(ghid_ticketed_cache_addr[write_index_s2][1:0])
    2'd0:
    begin
        ghid_s2 = l15_csm_req_data_s2;
        ghid_val_s2 = l15_csm_req_data_s2 >> ((128/4) - 1);
    end
    2'd1:
    begin
        ghid_s2 = l15_csm_req_data_s2 >> (128/4);
        ghid_val_s2 = l15_csm_req_data_s2 >> ((128/4) * 2 - 1);
    end
    2'd2:
    begin
        ghid_s2 = l15_csm_req_data_s2 >> ((128/4) * 2);
        ghid_val_s2 = l15_csm_req_data_s2 >> ((128/4) * 3 - 1);
    end
    2'd3:
    begin
        ghid_s2 = l15_csm_req_data_s2 >> ((128/4) * 3);
        ghid_val_s2 = l15_csm_req_data_s2 >> ((128/4) * 4 - 1);
    end
    endcase
end


always @ (posedge clk)
begin
   if (!rst_n)
   begin
      ghid_ticketed_cache_val[0] <= 0;
ghid_ticketed_cache[0] <= 0;
ghid_ticketed_cache_val[1] <= 0;
ghid_ticketed_cache[1] <= 0;
ghid_ticketed_cache_val[2] <= 0;
ghid_ticketed_cache[2] <= 0;
ghid_ticketed_cache_val[3] <= 0;
ghid_ticketed_cache[3] <= 0;
ghid_ticketed_cache_val[4] <= 0;
ghid_ticketed_cache[4] <= 0;
ghid_ticketed_cache_val[5] <= 0;
ghid_ticketed_cache[5] <= 0;
ghid_ticketed_cache_val[6] <= 0;
ghid_ticketed_cache[6] <= 0;
ghid_ticketed_cache_val[7] <= 0;
ghid_ticketed_cache[7] <= 0;

   end
   else
   begin
      if (write_val_s2)
      begin
         ghid_ticketed_cache[write_index_s2] <= ghid_s2;
         ghid_ticketed_cache_val[write_index_s2] <= ghid_val_s2;
         if (l15_csm_clear_ticket_val && (l15_csm_clear_ticket != write_index_s2))
         begin
            ghid_ticketed_cache_val[l15_csm_clear_ticket] <= 1'b0;
         end
      end
      else if (read_val_s2)
      begin
         ghid_ticketed_cache[read_index_s2] <= 0;
         ghid_ticketed_cache_val[read_index_s2] <=1'b0;
         if (l15_csm_clear_ticket_val && (l15_csm_clear_ticket != read_index_s2))
         begin
            ghid_ticketed_cache_val[l15_csm_clear_ticket] <= 1'b0;
         end
      end
      else if (l15_csm_clear_ticket_val)
      begin
          ghid_ticketed_cache_val[l15_csm_clear_ticket] <= 1'b0;
      end
   end
end


always @ *
begin
    if (write_val_s2)
    begin
        addr_in_s2_next = ghid_ticketed_cache_addr[write_index_s2];
    end
    else
    begin
        addr_in_s2_next = addr_in_s2;
    end
end

//Stage 2 => Stage 3

reg [40-1:0] l15_csm_req_address_s3;
reg [10-1:0] l15_csm_req_clump_id_s3;
reg l15_csm_req_val_s3;
reg [3-1:0] l15_csm_req_ticket_s3;
reg [16-1:0] addr_in_s3;
reg [128-1:0] data_in_s3;
reg [6-1:0] lhid_s3;
reg diag_en_s3;
reg flush_en_s3;
reg rd_en_s3;
reg wr_en_s3;
reg [14-1:0] l15_csm_chipid_s3;
reg [8-1:0] l15_csm_x_s3;
reg [8-1:0] l15_csm_y_s3;
reg l15_csm_clump_sel_s3;

always @ (posedge clk)
begin
   if (!rst_n)
   begin
      l15_csm_req_address_s3 <= 0;
      l15_csm_req_clump_id_s3 <= 0;
      l15_csm_req_val_s3 <= 0;
      l15_csm_req_ticket_s3 <= 0;
      addr_in_s3 <= 0;
      data_in_s3 <= 0;
      lhid_s3 <= 0;
      diag_en_s3 <= 0;
      flush_en_s3 <= 0;
      rd_en_s3 <= 0;
      wr_en_s3 <= 0;
      l15_csm_chipid_s3 <= 0;
      l15_csm_x_s3 <= 0;
      l15_csm_y_s3 <= 0;
      l15_csm_clump_sel_s3 <= 0;
   end
   else
   begin
      if (!l15_csm_stall_s3)
      begin
         l15_csm_req_address_s3 <= l15_csm_req_address_s2;
         l15_csm_req_clump_id_s3 <= l15_csm_req_clump_id_s2;
         l15_csm_req_val_s3 <= l15_csm_req_val_s2;
         l15_csm_req_ticket_s3 <= l15_csm_req_ticket_s2;
         addr_in_s3 <= addr_in_s2_next;
         data_in_s3 <= l15_csm_req_data_s2;
         lhid_s3 <= lhid_s2;
         diag_en_s3 <= diag_en_s2;
         flush_en_s3 <= flush_en_s2;
         rd_en_s3 <= rd_en_s2;
         wr_en_s3 <= wr_en_s2;
         l15_csm_chipid_s3 <= l15_csm_chipid_s2;
         l15_csm_x_s3 <= l15_csm_x_s2;
         l15_csm_y_s3 <= l15_csm_y_s2;
         l15_csm_clump_sel_s3 <= l15_csm_clump_sel_s2;
      end
   end
end


//Stage 3



reg [2-1:0] addr_op_s3;
reg refill_req_val_s3;
wire hit_s3;
wire [30-1:0] data_out_s3;
wire [4-1:0] valid_out_s3;
wire [14-1:0] tag_out_s3;

wire [8-1:0] lhid_s3_x;
wire [8-1:0] lhid_s3_y;


always @ *
begin
    addr_op_s3 = l15_csm_req_address_s3[31:30];
end


l15_hmc l15_hmc(
   .clk            (clk),
   .rst_n          (rst_n),
   .rd_en          (rd_en_s3),
   .wr_en          (wr_en_s3),
   .rd_diag_en     (diag_en_s3),
   .wr_diag_en     (diag_en_s3),
   .flush_en       (flush_en_s3),
   .addr_op        (addr_op_s3),
   .rd_addr_in     (addr_in_s3),
   .wr_addr_in     (addr_in_s3),
   .data_in        (data_in_s3),
   .hit            (hit_s3),
   .data_out       (data_out_s3),
   .valid_out      (valid_out_s3),
   .tag_out        (tag_out_s3)
);


always @ *
begin
    refill_req_val_s3 = csm_en && rd_en_s3 && (~diag_en_s3) && (~flush_en_s3) && (~hit_s3);
end

always @ *
begin
    if (csm_en)
    begin
        csm_l15_res_val_s3 = rd_en_s3 && ~refill_req_val_s3;
    end
    else
    begin
        csm_l15_res_val_s3 = l15_csm_req_val_s3;
    end
end

always @ *
begin
    if (diag_en_s3)
    begin
        if (addr_op_s3 == 0)
        begin
            csm_l15_res_data_s3 = data_out_s3;
        end
        else if (addr_op_s3 == 1)
        begin
            csm_l15_res_data_s3 = valid_out_s3;
        end
        else if (addr_op_s3 == 2)
        begin
            csm_l15_res_data_s3 = tag_out_s3;
        end
        else
        begin
            csm_l15_res_data_s3 = 0;
        end
    end
    else
    begin
        if (csm_en)
        begin
            if (~diag_en_s3 && ~flush_en_s3 && (l15_csm_clump_sel_s3 == 1'b1))
            begin
                csm_l15_res_data_s3 = 0;
                csm_l15_res_data_s3[((14+8+8)-1):(8+8)] = l15_csm_chipid_s3;
                //TODO, need to make x, y position consistant
                csm_l15_res_data_s3[8-1:0] = l15_csm_y_s3;
                csm_l15_res_data_s3[8+8-1:8] = l15_csm_x_s3;
            end
            else
            begin
                csm_l15_res_data_s3 = data_out_s3;
            end
        end
        else
        begin
            // csm_l15_res_data_s3 = {{`NOC_CHIPID_WIDTH{1'b0}},
            //                       {(`NOC_X_WIDTH-`HOME_ID_Y_POS_WIDTH){1'b0}}, lhid_s3[`HOME_ID_Y_POS],
            //                       {(`NOC_X_WIDTH-`HOME_ID_X_POS_WIDTH){1'b0}}, lhid_s3[`HOME_ID_X_POS]};
            csm_l15_res_data_s3 = 0;
            csm_l15_res_data_s3[((14+8+8)-1):(8+8)] = 1'b0; // non-csm mode only has 1 chip alone
            csm_l15_res_data_s3[8+8-1:8] = lhid_s3_y;
            csm_l15_res_data_s3[8-1:0] = lhid_s3_x;
        end
    end
end

flat_id_to_xy lhid_to_xy (
    .flat_id(lhid_s3[3+3-1:0]),
    .x_coord(lhid_s3_x),
    .y_coord(lhid_s3_y)
    );

/*
always @ *
begin
    if (csm_en)
    begin
      refill_req_val_s3 = rd_en_s3 && (~diag_en_s3) && (~flush_en_s3) && (~hit_s3);
       ;
      // csm_l15_res_data_s3 = data_out_s3;
      // csm_l15_res_valid_s3 = valid_out_s3;
      // csm_l15_res_tag_s3 = tag_out_s3;
      if (diag_en_s3 && rd_en_s3 && (addr_op_s3 == 1))
      begin
        csm_l15_res_data_s3 = valid_out_s3;
      end
      else if (diag_en_s3 && rd_en_s3 && (addr_op_s3 == 2))
      begin
        csm_l15_res_data_s3 = tag_out_s3;
      end
      else
      begin
        csm_l15_res_data_s3 = data_out_s3; // TODO: mux this with valid and tag
      end
    end
    else
    begin
      // non-csm mode
      refill_req_val_s3 = 1'b0;
      csm_l15_res_val_s3 = l15_csm_req_val_s3;
      // csm_l15_res_data_s3 = {{`NOC_CHIPID_WIDTH{1'b0}},
      //                       {(`NOC_X_WIDTH-`HOME_ID_Y_POS_WIDTH){1'b0}}, lhid_s3[`HOME_ID_Y_POS],
      //                       {(`NOC_X_WIDTH-`HOME_ID_X_POS_WIDTH){1'b0}}, lhid_s3[`HOME_ID_X_POS]};
      csm_l15_res_data_s3 = 0;
      csm_l15_res_data_s3[`PACKET_HOME_ID_CHIP_MASK] = 1'b0; // non-csm mode only has 1 chip alone
      csm_l15_res_data_s3[`PACKET_HOME_ID_Y_MASK] = lhid_s3[`HOME_ID_Y_POS];
      csm_l15_res_data_s3[`PACKET_HOME_ID_X_MASK] = lhid_s3[`HOME_ID_X_POS];
    end
end
*/



always @ (posedge clk)
begin
   if (!rst_n)
   begin
      ghid_ticketed_cache_addr[0] <= 0;
ghid_ticketed_cache_addr[1] <= 0;
ghid_ticketed_cache_addr[2] <= 0;
ghid_ticketed_cache_addr[3] <= 0;
ghid_ticketed_cache_addr[4] <= 0;
ghid_ticketed_cache_addr[5] <= 0;
ghid_ticketed_cache_addr[6] <= 0;
ghid_ticketed_cache_addr[7] <= 0;

   end
   else
   begin
      if (refill_req_val_s3)
      begin
         ghid_ticketed_cache_addr[l15_csm_req_ticket_s3] <= addr_in_s3;
      end
   end
end




// read port for ghid
assign csm_l15_read_res_data = ghid_ticketed_cache[l15_csm_read_ticket];
assign csm_l15_read_res_val = ghid_ticketed_cache_val[l15_csm_read_ticket];

//Output buffer

reg [40-1:0] refill_req_addr_buf [8-1:0];
reg [3-1:0] refill_req_ticket_buf [8-1:0];
reg [8-1:0] refill_req_val_buf;

reg [3-1:0] refill_req_buf_rd_ptr;
reg [3-1:0] refill_req_buf_rd_ptr_next;
reg [3-1:0] refill_req_buf_wr_ptr;
reg [3-1:0] refill_req_buf_wr_ptr_next;
reg [3:0] refill_req_buf_counter;
reg [3:0] refill_req_buf_counter_next;

always @ *
begin
    if (!rst_n)
    begin
        refill_req_buf_counter_next = 0;
    end
    else if (refill_req_val_s3 && noc1encoder_csm_req_ack)
    begin
        refill_req_buf_counter_next = refill_req_buf_counter;
    end
    else if (refill_req_val_s3)
    begin
        refill_req_buf_counter_next = refill_req_buf_counter + 1;
    end
    else if (noc1encoder_csm_req_ack)
    begin
        refill_req_buf_counter_next = refill_req_buf_counter - 1;
    end
    else
    begin
        refill_req_buf_counter_next = refill_req_buf_counter;
    end
end

always @ (posedge clk)
begin
    refill_req_buf_counter <= refill_req_buf_counter_next;
end

always @ *
begin
    if (!rst_n)
    begin
        refill_req_buf_rd_ptr_next = 0;
    end
    else if (noc1encoder_csm_req_ack)
    begin
        refill_req_buf_rd_ptr_next = refill_req_buf_rd_ptr + 1;
    end
    else
    begin
        refill_req_buf_rd_ptr_next = refill_req_buf_rd_ptr;
    end
end

always @ (posedge clk)
begin
    refill_req_buf_rd_ptr <= refill_req_buf_rd_ptr_next;
end

always @ *
begin
    if (!rst_n)
    begin
        refill_req_buf_wr_ptr_next = 0;
    end
    else if (refill_req_val_s3)
    begin
        refill_req_buf_wr_ptr_next = refill_req_buf_wr_ptr + 1;
    end
    else
    begin
        refill_req_buf_wr_ptr_next = refill_req_buf_wr_ptr;
    end
end

always @ (posedge clk)
begin
    refill_req_buf_wr_ptr <= refill_req_buf_wr_ptr_next;
end

always @ (posedge clk)
begin
   if (!rst_n)
   begin
      refill_req_addr_buf[0] <= 0;
refill_req_ticket_buf[0] <= 0;
refill_req_val_buf[0] <= 0;
refill_req_addr_buf[1] <= 0;
refill_req_ticket_buf[1] <= 0;
refill_req_val_buf[1] <= 0;
refill_req_addr_buf[2] <= 0;
refill_req_ticket_buf[2] <= 0;
refill_req_val_buf[2] <= 0;
refill_req_addr_buf[3] <= 0;
refill_req_ticket_buf[3] <= 0;
refill_req_val_buf[3] <= 0;
refill_req_addr_buf[4] <= 0;
refill_req_ticket_buf[4] <= 0;
refill_req_val_buf[4] <= 0;
refill_req_addr_buf[5] <= 0;
refill_req_ticket_buf[5] <= 0;
refill_req_val_buf[5] <= 0;
refill_req_addr_buf[6] <= 0;
refill_req_ticket_buf[6] <= 0;
refill_req_val_buf[6] <= 0;
refill_req_addr_buf[7] <= 0;
refill_req_ticket_buf[7] <= 0;
refill_req_val_buf[7] <= 0;

   end
   else
   begin
      if (refill_req_val_s3)
      begin
        refill_req_addr_buf[refill_req_buf_wr_ptr] <= {l15_hmt_base_reg, addr_in_s3[15:2], 4'd0};
        refill_req_ticket_buf[refill_req_buf_wr_ptr] <= l15_csm_req_ticket_s3;
        refill_req_val_buf[refill_req_buf_wr_ptr] <= refill_req_val_s3;
      end
   end
end



assign csm_noc1encoder_req_val = refill_req_val_buf[refill_req_buf_rd_ptr] && (refill_req_buf_counter > 0);
assign csm_noc1encoder_req_type = 5'd2;
assign csm_noc1encoder_req_mshrid = refill_req_ticket_buf[refill_req_buf_rd_ptr];
assign csm_noc1encoder_req_address = refill_req_addr_buf[refill_req_buf_rd_ptr];
assign csm_noc1encoder_req_non_cacheable = 1'b1;
assign csm_noc1encoder_req_size = 3'b111; // use pcx def because noc1enc translate it to packet def

/*
// calculate (or look up ghid from address)
// basic calculation derived from old model in L1.5 noc1 buffer/encoder
always @ *
begin
   // Stage 2 calculation
   if (`HOME_ID_MASK_X_ENABLE)
      dest_x_s2[`L15_CSM_GHID_XPOS_MASK] = l15_csm_req_address_s2[`HOME_ID_MASK_X];
   else
      dest_x_s2 = 0;
   if (`HOME_ID_MASK_Y_ENABLE)
      dest_y_s2[`L15_CSM_GHID_YPOS_MASK] = l15_csm_req_address_s2[`HOME_ID_MASK_Y];
   else
      dest_y_s2 = 0;
   dest_chip_s2 = 0;

   ghid_s2[`L15_CSM_GHID_XPOS_MASK] = dest_x_s2;
   ghid_s2[`L15_CSM_GHID_YPOS_MASK] = dest_y_s2;
   ghid_s2[`L15_CSM_GHID_CHIP_MASK] = dest_chip_s2;
   req_val_s3_next = l15_csm_req_val_s2 ? 1'b1 : 1'b0;

   // Stage 3 (response) calculation
   // res_val_s3 = 1'b1; // no cache miss in this model
   res_val_s3 = 1'b1; // or simulate all cache miss model
   csm_l15_res_data_s3 = ghid_s3;
   csm_l15_res_val_s3 = req_val_s3 && res_val_s3;

   // write to the ticketed storage. This would normally be written to when the module
   //  gets back the response from memory
   write_index = ticketid_s3;
   write_val = req_val_s3 && !l15_csm_stall_s3;
end

always @ (posedge clk)
begin
   if (!rst_n)
   begin
      ghid_ticketed_cache_val[0] <= 0;
ghid_ticketed_cache[0] <= 0;
ghid_ticketed_cache_val[1] <= 0;
ghid_ticketed_cache[1] <= 0;
ghid_ticketed_cache_val[2] <= 0;
ghid_ticketed_cache[2] <= 0;
ghid_ticketed_cache_val[3] <= 0;
ghid_ticketed_cache[3] <= 0;
ghid_ticketed_cache_val[4] <= 0;
ghid_ticketed_cache[4] <= 0;
ghid_ticketed_cache_val[5] <= 0;
ghid_ticketed_cache[5] <= 0;
ghid_ticketed_cache_val[6] <= 0;
ghid_ticketed_cache[6] <= 0;
ghid_ticketed_cache_val[7] <= 0;
ghid_ticketed_cache[7] <= 0;

   end
   else
   begin
      if (!l15_csm_stall_s3)
      begin
         ghid_s3 <= ghid_s2;
         ticketid_s3 <= l15_csm_req_ticket_s2;
         req_val_s3 <= req_val_s3_next;
      end

      if (write_val)
      begin
         ghid_ticketed_cache[write_index] <= ghid_s3;
      end
      // logic for valid bit is a bit more complicated
      // although it should be impossible to both write and clear a slot at once
      
      if (write_val && write_index == 0)
         ghid_ticketed_cache_val[0] <= 1'b1;
      else if (l15_csm_clear_ticket_val && l15_csm_clear_ticket == 0)
         ghid_ticketed_cache_val[0] <= 1'b0;
      

      if (write_val && write_index == 1)
         ghid_ticketed_cache_val[1] <= 1'b1;
      else if (l15_csm_clear_ticket_val && l15_csm_clear_ticket == 1)
         ghid_ticketed_cache_val[1] <= 1'b0;
      

      if (write_val && write_index == 2)
         ghid_ticketed_cache_val[2] <= 1'b1;
      else if (l15_csm_clear_ticket_val && l15_csm_clear_ticket == 2)
         ghid_ticketed_cache_val[2] <= 1'b0;
      

      if (write_val && write_index == 3)
         ghid_ticketed_cache_val[3] <= 1'b1;
      else if (l15_csm_clear_ticket_val && l15_csm_clear_ticket == 3)
         ghid_ticketed_cache_val[3] <= 1'b0;
      

      if (write_val && write_index == 4)
         ghid_ticketed_cache_val[4] <= 1'b1;
      else if (l15_csm_clear_ticket_val && l15_csm_clear_ticket == 4)
         ghid_ticketed_cache_val[4] <= 1'b0;
      

      if (write_val && write_index == 5)
         ghid_ticketed_cache_val[5] <= 1'b1;
      else if (l15_csm_clear_ticket_val && l15_csm_clear_ticket == 5)
         ghid_ticketed_cache_val[5] <= 1'b0;
      

      if (write_val && write_index == 6)
         ghid_ticketed_cache_val[6] <= 1'b1;
      else if (l15_csm_clear_ticket_val && l15_csm_clear_ticket == 6)
         ghid_ticketed_cache_val[6] <= 1'b0;
      

      if (write_val && write_index == 7)
         ghid_ticketed_cache_val[7] <= 1'b1;
      else if (l15_csm_clear_ticket_val && l15_csm_clear_ticket == 7)
         ghid_ticketed_cache_val[7] <= 1'b0;
      

   end
end

// read port for ghid
always @ *
begin
   csm_l15_read_res_data = ghid_ticketed_cache[l15_csm_read_ticket];
   csm_l15_read_res_val = ghid_ticketed_cache_val[l15_csm_read_ticket];
end
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
//  Filename      : l15_hmc.v
//  Created On    : 2014-06-07
//  Revision      :
//  Author        : Yaosheng Fu
//  Company       : Princeton University
//  Email         : yfu@princeton.edu
//
//  Description   : The HMC in the L15 cache
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






































































































































































































































































































































































































































































































































































































module l15_hmc(

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


l15_priority_encoder_4 priority_encoder_cmp_4bits( 

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



l15_priority_encoder_4 priority_encoder_wr_cmp_4bits( 

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


l15_priority_encoder_4 priority_encoder_replace_cmp_4bits( 

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
//  Filename      : l15_home_encoder.v
//  Created On    : 2014-06-23
//  Revision      :
//  Author        : Yaosheng Fu
//  Company       : Princeton University
//  Email         : yfu@princeton.edu
//
//  Description   : Home encoder for the L15 cache
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








































































































































































































































































































































































































































































































































































































module l15_home_encoder(
    // input wire clk,
    // input wire rst_n,
    input wire [6-1:0] home_in,
    //num_homes = 0 means have the maximum number of homes
    input wire [6-1:0] num_homes,

    output reg [6-1:0] lhid_out
);

reg [6-1:0] home_mask;
reg [6-1:0] home_low_mask;
reg [6-1:0] home_mod;
reg isPowerOf2;

always @ *
begin
    if (num_homes[5])
    begin
        home_low_mask = 6'b011111;
        isPowerOf2 = ~(|(num_homes[4:0]));
    end
    else if (num_homes[4])
    begin
        home_low_mask = 6'b001111;
        isPowerOf2 = ~(|(num_homes[3:0]));
    end
    else if (num_homes[3])
    begin
        home_low_mask = 6'b000111;
        isPowerOf2 = ~(|(num_homes[2:0]));
    end
    else if (num_homes[2])
    begin
        home_low_mask = 6'b000011;
        isPowerOf2 = ~(|(num_homes[1:0]));
    end
    else if (num_homes[1])
    begin
        home_low_mask = 6'b000001;
        isPowerOf2 = ~(|(num_homes[0:0]));
    end
    else if (num_homes[0])
    begin
        home_low_mask = 6'b000000;
        isPowerOf2 = 1'b1;
    end
    else
    begin
        home_low_mask = 6'b111111;
        isPowerOf2 = 1'b1;
    end

end

always @ *
begin
    if (isPowerOf2) 
    begin
        home_mask = home_low_mask;
    end
    else 
    begin
        home_mask = {home_low_mask[6-2:0], 1'b1};
    end
end

always @ *
begin
    home_mod = home_in & home_mask;
end

always @ *
begin
    if (home_mod < num_homes)
    begin
        lhid_out = home_mod;
    end
    else
    begin
        lhid_out = home_mod & home_low_mask;
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
//  Filename      : l15_mshr.v
//  Created On    : 2014-02-06 02:44:17
//  Last Modified : 2015-01-22 17:33:10
//  Revision      :
//  Author        : Tri Nguyen
//  Company       : Princeton University
//  Email         : trin@princeton.edu
//
//  Description   : asynchronous read, synchronous write on posedge
//
//
//==================================================================================================

//`timescale 1 ns / 10 ps
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



















































































































































































































































































































































































































































































































































































































































































































































































module l15_mshr(
    input wire clk,
    input wire rst_n,

    //s1
    input wire pipe_mshr_writereq_val_s1,
    input wire [3-1:0] pipe_mshr_writereq_op_s1,
    input wire [39:0] pipe_mshr_writereq_address_s1,
    input wire [127:0] pipe_mshr_writereq_write_buffer_data_s1,
    input wire [15:0] pipe_mshr_writereq_write_buffer_byte_mask_s1,
    input wire [((((((((((0 + 1) + 1) + 1) + 1) + 3) + 1) + 2) + 1) + 1) + 1)-1:0] pipe_mshr_writereq_control_s1,
    input wire [2-1:0] pipe_mshr_writereq_mshrid_s1,
    input wire [0:0] pipe_mshr_writereq_threadid_s1,

    input wire [0:0] pipe_mshr_readreq_threadid_s1,
    input wire [2-1:0] pipe_mshr_readreq_mshrid_s1,
    output reg [((((((((((0 + 1) + 1) + 1) + 1) + 3) + 1) + 2) + 1) + 1) + 1)-1:0]mshr_pipe_readres_control_s1,
    output reg [(14+8+8)-1:0] mshr_pipe_readres_homeid_s1,

    // s1/2/3 (address conflict checking)
    output reg [(4*2)-1:0] mshr_pipe_vals_s1,
    output reg [(40*2)-1:0] mshr_pipe_ld_address,
    output reg [(40*2)-1:0] mshr_pipe_st_address,
    output reg [(2*2)-1:0] mshr_pipe_st_way_s1,
    output reg [(2*2)-1:0] mshr_pipe_st_state_s1,

    //s2
    input wire pipe_mshr_write_buffer_rd_en_s2,
    input wire [0:0] pipe_mshr_threadid_s2,
    output reg [127:0]mshr_pipe_write_buffer_s2,
    output reg [15:0] mshr_pipe_write_buffer_byte_mask_s2,

    //s3
    input wire pipe_mshr_val_s3,
    input wire [3-1:0] pipe_mshr_op_s3,
    input wire [2-1:0] pipe_mshr_mshrid_s3,
    input wire [0:0] pipe_mshr_threadid_s3,
    input wire [2-1:0] pipe_mshr_write_update_state_s3,
    input wire [1:0] pipe_mshr_write_update_way_s3,

    // output reg mshr_pipe_t0_ld_address_val,
    // output reg mshr_pipe_t0_st_address_val,
    // output reg mshr_pipe_t1_ld_address_val,
    // output reg mshr_pipe_t1_st_address_val,



    // homeid related signals
    input wire noc1buffer_mshr_homeid_write_val_s4,
    input wire [2-1:0] noc1buffer_mshr_homeid_write_mshrid_s4,
    input wire [0:0] noc1buffer_mshr_homeid_write_threadid_s4,
    input wire [(14+8+8)-1:0] noc1buffer_mshr_homeid_write_data_s4
    );

// reg [`L15_PADDR_HI:0]                    address_array [0:9];
// reg [`L15_CONTROL_WIDTH-1:0]   control_array [0:9];
// reg [9:0] val_array;

// reg [127:0] st8_data;
// reg [127:0] st9_data;
// reg [15:0] st8_data_byte_mask;
// reg [15:0] st9_data_byte_mask;
// reg [1:0] st8_way;
// reg [1:0] st9_way;
// reg [`L15_MESI_TRANS_STATE_WIDTH-1:0] st8_state;
// reg [`L15_MESI_TRANS_STATE_WIDTH-1:0] st9_state;
// reg [`PACKET_HOME_ID_WIDTH-1:0] st_fill_homeid[0:1];

reg [39:0] ld_address [0:2-1];
reg [((((((((((0 + 1) + 1) + 1) + 1) + 3) + 1) + 2) + 1) + 1) + 1)-1:0] ld_control [0:2-1];
reg [2-1:0] ld_val;
reg [(14+8+8)-1:0] ld_homeid [0:2-1];

// reg [`L15_PADDR_HI:0] ifill_address [0:`L15_NUM_THREADS-1];
reg [((((((((((0 + 1) + 1) + 1) + 1) + 3) + 1) + 2) + 1) + 1) + 1)-1:0] ifill_control [0:2-1];
reg [2-1:0] ifill_val;
// reg [`PACKET_HOME_ID_WIDTH-1:0] ifill_homeid [0:`L15_NUM_THREADS-1];

reg [39:0] st_address [0:2-1];
reg [((((((((((0 + 1) + 1) + 1) + 1) + 3) + 1) + 2) + 1) + 1) + 1)-1:0] st_control [0:2-1];
reg [2-1:0] st_val;
reg [(14+8+8)-1:0] st_homeid [0:2-1];
reg [2-1:0] st_state [0:2-1];
reg [1:0] st_way [0:2-1];
reg [127:0] st_write_buffer [0:2-1];
reg [15:0] st_write_buffer_byte_mask [0:2-1];

/////////////////
// renaming
/////////////////
reg [3-1:0] op_s1;
reg [3-1:0] op_s3;
reg [0:0] threadid_s1;
reg [0:0] threadid_s3;
reg [2-1:0] mshrid_s1;
reg [2-1:0] mshrid_s3;
always @ *
begin
    op_s1 = pipe_mshr_writereq_op_s1;
    threadid_s1 = pipe_mshr_writereq_threadid_s1;
    mshrid_s1 = pipe_mshr_writereq_mshrid_s1;

    threadid_s3 = pipe_mshr_threadid_s3;
    op_s3 = pipe_mshr_op_s3;
    mshrid_s3 = pipe_mshr_mshrid_s3;
end

// Read operation/outputs
reg [4-1:0] tmp_vals [2-1:0];
reg [39:0] tmp_st_address [2-1:0];
reg [39:0] tmp_ld_address [2-1:0];
reg [2-1:0] tmp_st_way [2-1:0];
reg [2-1:0] tmp_st_state [2-1:0];
always @ *
begin
    
tmp_vals[0] = 0;
tmp_vals[0][2'd1] = ifill_val[0];
tmp_vals[0][2'd2] = ld_val[0];
tmp_vals[0][2'd3] = st_val[0];

tmp_st_address[0] = st_address[0];
tmp_ld_address[0] = ld_address[0];
tmp_st_way[0] = st_way[0];
tmp_st_state[0] =st_state[0];


tmp_vals[1] = 0;
tmp_vals[1][2'd1] = ifill_val[1];
tmp_vals[1][2'd2] = ld_val[1];
tmp_vals[1][2'd3] = st_val[1];

tmp_st_address[1] = st_address[1];
tmp_ld_address[1] = ld_address[1];
tmp_st_way[1] = st_way[1];
tmp_st_state[1] =st_state[1];


    mshr_pipe_vals_s1 = {tmp_vals[1], tmp_vals[0]};
    mshr_pipe_ld_address = {tmp_ld_address[1], tmp_ld_address[0]};
    mshr_pipe_st_address = {tmp_st_address[1], tmp_st_address[0]};
    mshr_pipe_st_way_s1 = {tmp_st_way[1], tmp_st_way[0]};
    mshr_pipe_st_state_s1 = {tmp_st_state[1], tmp_st_state[0]};

    // S1 read
    mshr_pipe_readres_homeid_s1[(14+8+8)-1:0] = 0;
    mshr_pipe_readres_control_s1[((((((((((0 + 1) + 1) + 1) + 1) + 3) + 1) + 2) + 1) + 1) + 1)-1:0] = 0;

    case (pipe_mshr_readreq_mshrid_s1)
        2'd1:
        begin
            mshr_pipe_readres_control_s1 = ifill_control[pipe_mshr_readreq_threadid_s1];
        end
        2'd2:
        begin
            mshr_pipe_readres_control_s1 = ld_control[pipe_mshr_readreq_threadid_s1];
            mshr_pipe_readres_homeid_s1 = ld_homeid[pipe_mshr_readreq_threadid_s1];
        end
        2'd3:
        begin
            mshr_pipe_readres_control_s1 = st_control[pipe_mshr_readreq_threadid_s1];
            mshr_pipe_readres_homeid_s1 = st_homeid[pipe_mshr_readreq_threadid_s1];
        end
    endcase

    // write-buffer reading in S2
    mshr_pipe_write_buffer_s2[127:0] = 128'b0;
    mshr_pipe_write_buffer_byte_mask_s2 = 16'b0;
    if (pipe_mshr_write_buffer_rd_en_s2)
    begin
        mshr_pipe_write_buffer_s2 = st_write_buffer[pipe_mshr_threadid_s2];
        mshr_pipe_write_buffer_byte_mask_s2 = st_write_buffer_byte_mask[pipe_mshr_threadid_s2];
    end
end

// Write operation
// generating mask for write
reg [127:0] bit_write_mask_s1;
always @ *
begin
    bit_write_mask_s1[0] = pipe_mshr_writereq_write_buffer_byte_mask_s1[0];
bit_write_mask_s1[1] = pipe_mshr_writereq_write_buffer_byte_mask_s1[0];
bit_write_mask_s1[2] = pipe_mshr_writereq_write_buffer_byte_mask_s1[0];
bit_write_mask_s1[3] = pipe_mshr_writereq_write_buffer_byte_mask_s1[0];
bit_write_mask_s1[4] = pipe_mshr_writereq_write_buffer_byte_mask_s1[0];
bit_write_mask_s1[5] = pipe_mshr_writereq_write_buffer_byte_mask_s1[0];
bit_write_mask_s1[6] = pipe_mshr_writereq_write_buffer_byte_mask_s1[0];
bit_write_mask_s1[7] = pipe_mshr_writereq_write_buffer_byte_mask_s1[0];
bit_write_mask_s1[8] = pipe_mshr_writereq_write_buffer_byte_mask_s1[1];
bit_write_mask_s1[9] = pipe_mshr_writereq_write_buffer_byte_mask_s1[1];
bit_write_mask_s1[10] = pipe_mshr_writereq_write_buffer_byte_mask_s1[1];
bit_write_mask_s1[11] = pipe_mshr_writereq_write_buffer_byte_mask_s1[1];
bit_write_mask_s1[12] = pipe_mshr_writereq_write_buffer_byte_mask_s1[1];
bit_write_mask_s1[13] = pipe_mshr_writereq_write_buffer_byte_mask_s1[1];
bit_write_mask_s1[14] = pipe_mshr_writereq_write_buffer_byte_mask_s1[1];
bit_write_mask_s1[15] = pipe_mshr_writereq_write_buffer_byte_mask_s1[1];
bit_write_mask_s1[16] = pipe_mshr_writereq_write_buffer_byte_mask_s1[2];
bit_write_mask_s1[17] = pipe_mshr_writereq_write_buffer_byte_mask_s1[2];
bit_write_mask_s1[18] = pipe_mshr_writereq_write_buffer_byte_mask_s1[2];
bit_write_mask_s1[19] = pipe_mshr_writereq_write_buffer_byte_mask_s1[2];
bit_write_mask_s1[20] = pipe_mshr_writereq_write_buffer_byte_mask_s1[2];
bit_write_mask_s1[21] = pipe_mshr_writereq_write_buffer_byte_mask_s1[2];
bit_write_mask_s1[22] = pipe_mshr_writereq_write_buffer_byte_mask_s1[2];
bit_write_mask_s1[23] = pipe_mshr_writereq_write_buffer_byte_mask_s1[2];
bit_write_mask_s1[24] = pipe_mshr_writereq_write_buffer_byte_mask_s1[3];
bit_write_mask_s1[25] = pipe_mshr_writereq_write_buffer_byte_mask_s1[3];
bit_write_mask_s1[26] = pipe_mshr_writereq_write_buffer_byte_mask_s1[3];
bit_write_mask_s1[27] = pipe_mshr_writereq_write_buffer_byte_mask_s1[3];
bit_write_mask_s1[28] = pipe_mshr_writereq_write_buffer_byte_mask_s1[3];
bit_write_mask_s1[29] = pipe_mshr_writereq_write_buffer_byte_mask_s1[3];
bit_write_mask_s1[30] = pipe_mshr_writereq_write_buffer_byte_mask_s1[3];
bit_write_mask_s1[31] = pipe_mshr_writereq_write_buffer_byte_mask_s1[3];
bit_write_mask_s1[32] = pipe_mshr_writereq_write_buffer_byte_mask_s1[4];
bit_write_mask_s1[33] = pipe_mshr_writereq_write_buffer_byte_mask_s1[4];
bit_write_mask_s1[34] = pipe_mshr_writereq_write_buffer_byte_mask_s1[4];
bit_write_mask_s1[35] = pipe_mshr_writereq_write_buffer_byte_mask_s1[4];
bit_write_mask_s1[36] = pipe_mshr_writereq_write_buffer_byte_mask_s1[4];
bit_write_mask_s1[37] = pipe_mshr_writereq_write_buffer_byte_mask_s1[4];
bit_write_mask_s1[38] = pipe_mshr_writereq_write_buffer_byte_mask_s1[4];
bit_write_mask_s1[39] = pipe_mshr_writereq_write_buffer_byte_mask_s1[4];
bit_write_mask_s1[40] = pipe_mshr_writereq_write_buffer_byte_mask_s1[5];
bit_write_mask_s1[41] = pipe_mshr_writereq_write_buffer_byte_mask_s1[5];
bit_write_mask_s1[42] = pipe_mshr_writereq_write_buffer_byte_mask_s1[5];
bit_write_mask_s1[43] = pipe_mshr_writereq_write_buffer_byte_mask_s1[5];
bit_write_mask_s1[44] = pipe_mshr_writereq_write_buffer_byte_mask_s1[5];
bit_write_mask_s1[45] = pipe_mshr_writereq_write_buffer_byte_mask_s1[5];
bit_write_mask_s1[46] = pipe_mshr_writereq_write_buffer_byte_mask_s1[5];
bit_write_mask_s1[47] = pipe_mshr_writereq_write_buffer_byte_mask_s1[5];
bit_write_mask_s1[48] = pipe_mshr_writereq_write_buffer_byte_mask_s1[6];
bit_write_mask_s1[49] = pipe_mshr_writereq_write_buffer_byte_mask_s1[6];
bit_write_mask_s1[50] = pipe_mshr_writereq_write_buffer_byte_mask_s1[6];
bit_write_mask_s1[51] = pipe_mshr_writereq_write_buffer_byte_mask_s1[6];
bit_write_mask_s1[52] = pipe_mshr_writereq_write_buffer_byte_mask_s1[6];
bit_write_mask_s1[53] = pipe_mshr_writereq_write_buffer_byte_mask_s1[6];
bit_write_mask_s1[54] = pipe_mshr_writereq_write_buffer_byte_mask_s1[6];
bit_write_mask_s1[55] = pipe_mshr_writereq_write_buffer_byte_mask_s1[6];
bit_write_mask_s1[56] = pipe_mshr_writereq_write_buffer_byte_mask_s1[7];
bit_write_mask_s1[57] = pipe_mshr_writereq_write_buffer_byte_mask_s1[7];
bit_write_mask_s1[58] = pipe_mshr_writereq_write_buffer_byte_mask_s1[7];
bit_write_mask_s1[59] = pipe_mshr_writereq_write_buffer_byte_mask_s1[7];
bit_write_mask_s1[60] = pipe_mshr_writereq_write_buffer_byte_mask_s1[7];
bit_write_mask_s1[61] = pipe_mshr_writereq_write_buffer_byte_mask_s1[7];
bit_write_mask_s1[62] = pipe_mshr_writereq_write_buffer_byte_mask_s1[7];
bit_write_mask_s1[63] = pipe_mshr_writereq_write_buffer_byte_mask_s1[7];
bit_write_mask_s1[64] = pipe_mshr_writereq_write_buffer_byte_mask_s1[8];
bit_write_mask_s1[65] = pipe_mshr_writereq_write_buffer_byte_mask_s1[8];
bit_write_mask_s1[66] = pipe_mshr_writereq_write_buffer_byte_mask_s1[8];
bit_write_mask_s1[67] = pipe_mshr_writereq_write_buffer_byte_mask_s1[8];
bit_write_mask_s1[68] = pipe_mshr_writereq_write_buffer_byte_mask_s1[8];
bit_write_mask_s1[69] = pipe_mshr_writereq_write_buffer_byte_mask_s1[8];
bit_write_mask_s1[70] = pipe_mshr_writereq_write_buffer_byte_mask_s1[8];
bit_write_mask_s1[71] = pipe_mshr_writereq_write_buffer_byte_mask_s1[8];
bit_write_mask_s1[72] = pipe_mshr_writereq_write_buffer_byte_mask_s1[9];
bit_write_mask_s1[73] = pipe_mshr_writereq_write_buffer_byte_mask_s1[9];
bit_write_mask_s1[74] = pipe_mshr_writereq_write_buffer_byte_mask_s1[9];
bit_write_mask_s1[75] = pipe_mshr_writereq_write_buffer_byte_mask_s1[9];
bit_write_mask_s1[76] = pipe_mshr_writereq_write_buffer_byte_mask_s1[9];
bit_write_mask_s1[77] = pipe_mshr_writereq_write_buffer_byte_mask_s1[9];
bit_write_mask_s1[78] = pipe_mshr_writereq_write_buffer_byte_mask_s1[9];
bit_write_mask_s1[79] = pipe_mshr_writereq_write_buffer_byte_mask_s1[9];
bit_write_mask_s1[80] = pipe_mshr_writereq_write_buffer_byte_mask_s1[10];
bit_write_mask_s1[81] = pipe_mshr_writereq_write_buffer_byte_mask_s1[10];
bit_write_mask_s1[82] = pipe_mshr_writereq_write_buffer_byte_mask_s1[10];
bit_write_mask_s1[83] = pipe_mshr_writereq_write_buffer_byte_mask_s1[10];
bit_write_mask_s1[84] = pipe_mshr_writereq_write_buffer_byte_mask_s1[10];
bit_write_mask_s1[85] = pipe_mshr_writereq_write_buffer_byte_mask_s1[10];
bit_write_mask_s1[86] = pipe_mshr_writereq_write_buffer_byte_mask_s1[10];
bit_write_mask_s1[87] = pipe_mshr_writereq_write_buffer_byte_mask_s1[10];
bit_write_mask_s1[88] = pipe_mshr_writereq_write_buffer_byte_mask_s1[11];
bit_write_mask_s1[89] = pipe_mshr_writereq_write_buffer_byte_mask_s1[11];
bit_write_mask_s1[90] = pipe_mshr_writereq_write_buffer_byte_mask_s1[11];
bit_write_mask_s1[91] = pipe_mshr_writereq_write_buffer_byte_mask_s1[11];
bit_write_mask_s1[92] = pipe_mshr_writereq_write_buffer_byte_mask_s1[11];
bit_write_mask_s1[93] = pipe_mshr_writereq_write_buffer_byte_mask_s1[11];
bit_write_mask_s1[94] = pipe_mshr_writereq_write_buffer_byte_mask_s1[11];
bit_write_mask_s1[95] = pipe_mshr_writereq_write_buffer_byte_mask_s1[11];
bit_write_mask_s1[96] = pipe_mshr_writereq_write_buffer_byte_mask_s1[12];
bit_write_mask_s1[97] = pipe_mshr_writereq_write_buffer_byte_mask_s1[12];
bit_write_mask_s1[98] = pipe_mshr_writereq_write_buffer_byte_mask_s1[12];
bit_write_mask_s1[99] = pipe_mshr_writereq_write_buffer_byte_mask_s1[12];
bit_write_mask_s1[100] = pipe_mshr_writereq_write_buffer_byte_mask_s1[12];
bit_write_mask_s1[101] = pipe_mshr_writereq_write_buffer_byte_mask_s1[12];
bit_write_mask_s1[102] = pipe_mshr_writereq_write_buffer_byte_mask_s1[12];
bit_write_mask_s1[103] = pipe_mshr_writereq_write_buffer_byte_mask_s1[12];
bit_write_mask_s1[104] = pipe_mshr_writereq_write_buffer_byte_mask_s1[13];
bit_write_mask_s1[105] = pipe_mshr_writereq_write_buffer_byte_mask_s1[13];
bit_write_mask_s1[106] = pipe_mshr_writereq_write_buffer_byte_mask_s1[13];
bit_write_mask_s1[107] = pipe_mshr_writereq_write_buffer_byte_mask_s1[13];
bit_write_mask_s1[108] = pipe_mshr_writereq_write_buffer_byte_mask_s1[13];
bit_write_mask_s1[109] = pipe_mshr_writereq_write_buffer_byte_mask_s1[13];
bit_write_mask_s1[110] = pipe_mshr_writereq_write_buffer_byte_mask_s1[13];
bit_write_mask_s1[111] = pipe_mshr_writereq_write_buffer_byte_mask_s1[13];
bit_write_mask_s1[112] = pipe_mshr_writereq_write_buffer_byte_mask_s1[14];
bit_write_mask_s1[113] = pipe_mshr_writereq_write_buffer_byte_mask_s1[14];
bit_write_mask_s1[114] = pipe_mshr_writereq_write_buffer_byte_mask_s1[14];
bit_write_mask_s1[115] = pipe_mshr_writereq_write_buffer_byte_mask_s1[14];
bit_write_mask_s1[116] = pipe_mshr_writereq_write_buffer_byte_mask_s1[14];
bit_write_mask_s1[117] = pipe_mshr_writereq_write_buffer_byte_mask_s1[14];
bit_write_mask_s1[118] = pipe_mshr_writereq_write_buffer_byte_mask_s1[14];
bit_write_mask_s1[119] = pipe_mshr_writereq_write_buffer_byte_mask_s1[14];
bit_write_mask_s1[120] = pipe_mshr_writereq_write_buffer_byte_mask_s1[15];
bit_write_mask_s1[121] = pipe_mshr_writereq_write_buffer_byte_mask_s1[15];
bit_write_mask_s1[122] = pipe_mshr_writereq_write_buffer_byte_mask_s1[15];
bit_write_mask_s1[123] = pipe_mshr_writereq_write_buffer_byte_mask_s1[15];
bit_write_mask_s1[124] = pipe_mshr_writereq_write_buffer_byte_mask_s1[15];
bit_write_mask_s1[125] = pipe_mshr_writereq_write_buffer_byte_mask_s1[15];
bit_write_mask_s1[126] = pipe_mshr_writereq_write_buffer_byte_mask_s1[15];
bit_write_mask_s1[127] = pipe_mshr_writereq_write_buffer_byte_mask_s1[15];

end



////////////////////////
// s1 stage writes
////////////////////////
// do write, either allocating a new mshr or updating write-buffer or deallocating an mshr
always @ (posedge clk)
begin
    if (pipe_mshr_writereq_val_s1 && (op_s1 == 3'b001))
    begin
        case (mshrid_s1)
            2'd1:
            begin
                // ifill_address[threadid_s1] <= pipe_mshr_writereq_address_s1;
                ifill_control[threadid_s1] <= pipe_mshr_writereq_control_s1;
            end
            2'd2:
            begin
                ld_address[threadid_s1] <= pipe_mshr_writereq_address_s1;
                ld_control[threadid_s1] <= pipe_mshr_writereq_control_s1;
            end
            2'd3:
            begin
                st_address[threadid_s1] <= pipe_mshr_writereq_address_s1;
                st_control[threadid_s1] <= pipe_mshr_writereq_control_s1;
                st_write_buffer[threadid_s1] <= (pipe_mshr_writereq_write_buffer_data_s1 & bit_write_mask_s1);
                st_write_buffer_byte_mask[threadid_s1] <= pipe_mshr_writereq_write_buffer_byte_mask_s1;
            end
        endcase
    end // address and control allocation

    else if (pipe_mshr_writereq_val_s1 && op_s1 == 3'b100)
    begin
        st_write_buffer[threadid_s1] <= ((st_write_buffer[threadid_s1] & ~bit_write_mask_s1) | (pipe_mshr_writereq_write_buffer_data_s1 & bit_write_mask_s1));
        st_write_buffer_byte_mask[threadid_s1] <= (st_write_buffer_byte_mask[threadid_s1] | pipe_mshr_writereq_write_buffer_byte_mask_s1);
    end // update write-buffer
end

////////////////////////
// s3 state writes
////////////////////////
always @ (posedge clk)
begin
    if (pipe_mshr_val_s3 && op_s3 == 3'b011)
    begin
        st_state[threadid_s3] <= pipe_mshr_write_update_state_s3;
        st_way[threadid_s3] <= pipe_mshr_write_update_way_s3;
    end // update store mshr state
end // mshr write

// special write logic for valid because of potential allocation/deallocation conflicts
// it's impossible because the L1.5 won't allocate until the valid bit is unset
//  but if there's a conflict, allocation wins
reg [2-1:0] ld_val_next;
reg [2-1:0] st_val_next;
reg [2-1:0] ifill_val_next;
always @ (posedge clk)
begin
    if (!rst_n)
    begin
        ld_val <= 0;
        st_val <= 0;
        ifill_val <= 0;
    end
    else
    begin
        ld_val <= ld_val_next;
        st_val <= st_val_next;
        ifill_val <= ifill_val_next;
    end
end

reg [2-1:0] ld_alloc_mask;
reg [2-1:0] st_alloc_mask;
reg [2-1:0] ifill_alloc_mask;
reg [2-1:0] ld_dealloc_mask;
reg [2-1:0] st_dealloc_mask;
reg [2-1:0] ifill_dealloc_mask;
always @ *
begin
    // deallocate_mask_inv[9:0] = 0;
    // if (mshr_write_val_s3 && mshr_write_type_s3 == `L15_MSHR_WRITE_TYPE_DEALLOCATION)
    //     deallocate_mask_inv[9:0] = 10'b0000000001 << mshr_write_mshrid_s3;
    // deallocate_mask[9:0] = ~deallocate_mask_inv[9:0];

    // allocate_mask[9:0] = 0;
    // if (mshr_val_s1 && op_s1 == `L15_MSHR_WRITE_TYPE_ALLOCATION)
    //     allocate_mask[9:0] = 10'b0000000001 << mshrid_s1;

    // val_array_next[9:0] = (val_array[9:0] & deallocate_mask[9:0]) | allocate_mask[9:0];


    // if (mshr_val_s1 && (op_s1 == `L15_MSHR_WRITE_TYPE_ALLOCATION)
    //     && mshr_write_val_s3 && (mshr_write_type_s3 == `L15_MSHR_WRITE_TYPE_DEALLOCATION)
    //     && (mshrid_s1 == mshr_write_mshrid_s3)
    //     && (threadid_s1 == mshr_write_threadid_s3))
    ld_alloc_mask = 0;
    st_alloc_mask = 0;
    ifill_alloc_mask = 0;
    if (pipe_mshr_writereq_val_s1 && (op_s1 == 3'b001))
    begin
        if (mshrid_s1 == 2'd2)
            ld_alloc_mask[threadid_s1] = 1'b1;
        else if (mshrid_s1 == 2'd3)
            st_alloc_mask[threadid_s1] = 1'b1;
        else if (mshrid_s1 == 2'd1)
            ifill_alloc_mask[threadid_s1] = 1'b1;
    end

    ld_dealloc_mask = 0;
    st_dealloc_mask = 0;
    ifill_dealloc_mask = 0;
    if (pipe_mshr_val_s3 && (op_s3 == 3'b010))
    begin
        if (mshrid_s3 == 2'd2)
            ld_dealloc_mask[threadid_s3] = 1'b1;
        else if (mshrid_s3 == 2'd3)
            st_dealloc_mask[threadid_s3] = 1'b1;
        else if (mshrid_s3 == 2'd1)
            ifill_dealloc_mask[threadid_s3] = 1'b1;
    end

    ld_val_next = ld_val;
    st_val_next = st_val;
    ifill_val_next = ifill_val;

    ld_val_next = (ld_val & ~ld_dealloc_mask) | ld_alloc_mask;
    st_val_next = (st_val & ~st_dealloc_mask) | st_alloc_mask;
    ifill_val_next = (ifill_val & ~ifill_dealloc_mask) | ifill_alloc_mask;
end

// write logic for homeid fills
always @ (posedge clk)
begin
    if (!rst_n)
    begin
        st_homeid[0] <= 0;
        st_homeid[1] <= 0;
        ld_homeid[0] <= 0;
        ld_homeid[1] <= 0;
    end
    else
    begin
        if (noc1buffer_mshr_homeid_write_val_s4)
        begin
            if (noc1buffer_mshr_homeid_write_mshrid_s4 == 2'd2)
                ld_homeid[noc1buffer_mshr_homeid_write_threadid_s4] <= noc1buffer_mshr_homeid_write_data_s4;
            else if (noc1buffer_mshr_homeid_write_mshrid_s4 == 2'd3)
                st_homeid[noc1buffer_mshr_homeid_write_threadid_s4] <= noc1buffer_mshr_homeid_write_data_s4;
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
//  Filename      : l15_ctl.v
//  Created On    : 2014-01-31 18:24:47
//  Last Modified : 2018-01-17 14:00:15
//  Revision      :
//  Author        : Tri Nguyen
//  Company       : Princeton University
//  Email         : trin@princeton.edu
//
//  Description   :
//
//
//==================================================================================================

//`timescale 1 ns / 10 ps
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
















































































































































































































































































































































































































































































































































































































































































































































































// devices.xml





module l15_pipeline(
    input wire clk,
    input wire rst_n,

    // pcx
    input wire [5-1:0] pcxdecoder_l15_rqtype,
    input wire [4-1:0] pcxdecoder_l15_amo_op,
    input wire pcxdecoder_l15_nc,
    input wire [3-1:0] pcxdecoder_l15_size,
    // input wire pcxdecoder_l15_invalall,         // unused input from core
    input wire [0:0] pcxdecoder_l15_threadid,
    input wire pcxdecoder_l15_prefetch,
    input wire pcxdecoder_l15_blockstore,
    input wire pcxdecoder_l15_blockinitstore,
    input wire [2-1:0] pcxdecoder_l15_l1rplway,
    input wire pcxdecoder_l15_val,
    input wire [39:0] pcxdecoder_l15_address,
    input wire [33-1:0] pcxdecoder_l15_csm_data,
    input wire [63:0] pcxdecoder_l15_data,
    input wire [63:0] pcxdecoder_l15_data_next_entry,
    input wire pcxdecoder_l15_invalidate_cacheline,
    // noc2
    input wire noc2decoder_l15_val,
    input wire [2-1:0] noc2decoder_l15_mshrid,
    input wire [0:0] noc2decoder_l15_threadid,
    input wire noc2decoder_l15_hmc_fill,
    input wire noc2decoder_l15_l2miss,
    input wire noc2decoder_l15_icache_type,
    input wire noc2decoder_l15_f4b, // tcov: not used by L2, is passed directly from L2 to the core
    input wire [8-1:0] noc2decoder_l15_reqtype, // tcov: message type length is dictated by packet format
    input wire [2-1:0] noc2decoder_l15_ack_state,
    input wire [3:0] noc2decoder_l15_fwd_subcacheline_vector,
    input wire [63:0] noc2decoder_l15_data_0,
    input wire [63:0] noc2decoder_l15_data_1,
    input wire [63:0] noc2decoder_l15_data_2,
    input wire [63:0] noc2decoder_l15_data_3,
    input wire [39:0] noc2decoder_l15_address,
    input wire [(14+8+8)-1:0] noc2decoder_l15_src_homeid,
    input wire [3-1:0] noc2decoder_l15_csm_mshrid,
    // ack from output
    input wire cpxencoder_l15_req_ack,
    // input wire noc1encoder_l15_req_ack,
    input wire noc1encoder_l15_req_sent,
    input wire [2-1:0] noc1encoder_l15_req_data_sent,
    input wire noc3encoder_l15_req_ack,

    // input from config registers to pipeline
    input wire [63:0] config_l15_read_res_data_s3,

    // MEMORY BLOCK OUTPUTS
    // data tag
    output reg l15_dtag_val_s1,
    output reg l15_dtag_rw_s1,
    output reg [((9-2))-1:0] l15_dtag_index_s1,
    output reg [33*4-1:0] l15_dtag_write_data_s1,
    output reg [33*4-1:0] l15_dtag_write_mask_s1,
    input wire [33*4-1:0] dtag_l15_dout_s2,
    // dcache
    output reg l15_dcache_val_s2,
    output reg l15_dcache_rw_s2,
    output reg [(((9-2))+2)-1:0] l15_dcache_index_s2,
    output reg [127:0] l15_dcache_write_data_s2,
    output reg [127:0] l15_dcache_write_mask_s2,
    input wire [127:0] dcache_l15_dout_s3,

    // mesi
    output reg l15_mesi_read_val_s1,
    output reg [((9-2))-1:0] l15_mesi_read_index_s1,
    input wire [7:0] mesi_l15_dout_s2,
    output reg l15_mesi_write_val_s2,
    output reg [((9-2))-1:0] l15_mesi_write_index_s2,
    output reg [7:0] l15_mesi_write_mask_s2,
    output reg [7:0] l15_mesi_write_data_s2,

    // lrsc_flag
    output reg l15_lrsc_flag_read_val_s1,
    output reg [((9-2))-1:0] l15_lrsc_flag_read_index_s1,
    input wire [3:0] lrsc_flag_l15_dout_s2,
    output reg l15_lrsc_flag_write_val_s2,
    output reg [((9-2))-1:0] l15_lrsc_flag_write_index_s2,
    output reg [3:0] l15_lrsc_flag_write_mask_s2,
    output reg [3:0] l15_lrsc_flag_write_data_s2,

    // lruarray
    output reg l15_lruarray_read_val_s1,
    output reg [((9-2))-1:0] l15_lruarray_read_index_s1,
    input wire [(2 + 4)-1:0] lruarray_l15_dout_s2,
    output reg l15_lruarray_write_val_s3,
    output reg [((9-2))-1:0] l15_lruarray_write_index_s3,
    output reg [(2 + 4)-1:0] l15_lruarray_write_mask_s3,    // tcov: writemask is designed to always be 1's
    output reg [(2 + 4)-1:0] l15_lruarray_write_data_s3,

    // hmt
    
    input wire [14 + 8 + 8-1:0] hmt_l15_dout_s3,
    output reg [14 + 8 + 8-1:0] l15_hmt_write_data_s2,
    output reg [14 + 8 + 8-1:0] l15_hmt_write_mask_s2,
    

    // wmt
    output reg l15_wmt_read_val_s2,
    output reg [6:0] l15_wmt_read_index_s2,
    input wire [2*((2+0)+1)-1:0] wmt_l15_data_s3,
    output reg l15_wmt_write_val_s3,
    output reg [6:0] l15_wmt_write_index_s3,
    output reg [2*((2+0)+1)-1:0] l15_wmt_write_mask_s3,
    output reg [2*((2+0)+1)-1:0] l15_wmt_write_data_s3,

    // MSHR
    //s1 (allocating)
    output reg pipe_mshr_writereq_val_s1,
    output reg [3-1:0] pipe_mshr_writereq_op_s1,   // tcov: one bit not used for encoding
    output reg [39:0] pipe_mshr_writereq_address_s1,
    output reg [127:0] pipe_mshr_writereq_write_buffer_data_s1,
    output reg [15:0] pipe_mshr_writereq_write_buffer_byte_mask_s1,
    output reg [((((((((((0 + 1) + 1) + 1) + 1) + 3) + 1) + 2) + 1) + 1) + 1)-1:0] pipe_mshr_writereq_control_s1,
    output reg [2-1:0] pipe_mshr_writereq_mshrid_s1,
    output reg [0:0] pipe_mshr_writereq_threadid_s1,
    // s1 (reading mshr)
    output reg [0:0] pipe_mshr_readreq_threadid_s1,
    output reg [2-1:0] pipe_mshr_readreq_mshrid_s1,
    input wire [((((((((((0 + 1) + 1) + 1) + 1) + 3) + 1) + 2) + 1) + 1) + 1)-1:0] mshr_pipe_readres_control_s1,
    input wire [(14+8+8)-1:0] mshr_pipe_readres_homeid_s1,
    // s1/2/3 (address conflict checking)
        // tcov: 4 mshr per thread but 1 is unused
    input wire [(4*2)-1:0] mshr_pipe_vals_s1,
    input wire [(40*2)-1:0] mshr_pipe_ld_address,
    input wire [(40*2)-1:0] mshr_pipe_st_address,
    input wire [(2*2)-1:0] mshr_pipe_st_way_s1,
    input wire [(2*2)-1:0] mshr_pipe_st_state_s1,
    //s2 (loading store buffer)
    output reg pipe_mshr_write_buffer_rd_en_s2,
    output reg [0:0] pipe_mshr_threadid_s2,
    input wire [127:0] mshr_pipe_write_buffer_s2,
    input wire [15:0] mshr_pipe_write_buffer_byte_mask_s2,
    //s3 (deallocation or updating write states)
    output reg pipe_mshr_val_s3,
    output reg [3-1:0] pipe_mshr_op_s3,
    output reg [2-1:0] pipe_mshr_mshrid_s3,
    output reg [0:0] pipe_mshr_threadid_s3,
    output reg [2-1:0] pipe_mshr_write_update_state_s3,
    output reg [1:0] pipe_mshr_write_update_way_s3,

    // PCX,CPX,NOC
    // cpx
    output reg l15_cpxencoder_val,
    output reg [3:0] l15_cpxencoder_returntype,
    output reg l15_cpxencoder_l2miss,
    output reg [1:0] l15_cpxencoder_error,  // tcov: to core but not utilized
    output reg l15_cpxencoder_noncacheable,
    output reg l15_cpxencoder_atomic,
    output reg [0:0] l15_cpxencoder_threadid,
    output reg l15_cpxencoder_prefetch,
    output reg l15_cpxencoder_f4b,
    output reg [63:0] l15_cpxencoder_data_0,
    output reg [63:0] l15_cpxencoder_data_1,
    output reg [63:0] l15_cpxencoder_data_2,
    output reg [63:0] l15_cpxencoder_data_3,
    output reg l15_cpxencoder_inval_icache_all_way,
    output reg l15_cpxencoder_inval_dcache_all_way,         // tcov: to core but not utilized, inval individually
    output reg [15:4] l15_cpxencoder_inval_address_15_4,
    output reg l15_cpxencoder_cross_invalidate,             // tcov: to core but not utilized
    output reg [1:0] l15_cpxencoder_cross_invalidate_way,   // tcov: to core but not utilized
    output reg l15_cpxencoder_inval_dcache_inval,
    output reg l15_cpxencoder_inval_icache_inval,           // tcov: to core but not utilized, instead inval all way
    output reg l15_cpxencoder_blockinitstore,
    output reg [1:0] l15_cpxencoder_inval_way,
    // noc1
    output reg l15_noc1buffer_req_val,
    output reg [5-1:0] l15_noc1buffer_req_type,
    output reg [0:0] l15_noc1buffer_req_threadid,
    output reg [2-1:0] l15_noc1buffer_req_mshrid,
    output reg [39:0] l15_noc1buffer_req_address,
    output reg l15_noc1buffer_req_non_cacheable,
    output reg [2:0] l15_noc1buffer_req_size,
    output reg l15_noc1buffer_req_prefetch,
    output reg [33-1:0] l15_noc1buffer_req_csm_data,
    output reg [63:0] l15_noc1buffer_req_data_0,
    output reg [63:0] l15_noc1buffer_req_data_1,
    // output reg l15_noc1buffer_req_blkstore,
    // output reg l15_noc1buffer_req_blkinitstore,
    // noc3
    output reg l15_noc3encoder_req_val,
    output reg [3-1:0] l15_noc3encoder_req_type,
    output reg [63:0] l15_noc3encoder_req_data_0,
    output reg [63:0] l15_noc3encoder_req_data_1,
    output reg [2-1:0] l15_noc3encoder_req_mshrid,
    output reg [1:0] l15_noc3encoder_req_sequenceid,
    output reg [0:0] l15_noc3encoder_req_threadid,
    output reg [39:0] l15_noc3encoder_req_address,
    output reg l15_noc3encoder_req_with_data,
    output reg l15_noc3encoder_req_was_inval,
    output reg [3:0] l15_noc3encoder_req_fwdack_vector,
    output reg [(14+8+8)-1:0] l15_noc3encoder_req_homeid,
    // ack to inputs
    output reg l15_pcxdecoder_ack,
    output reg l15_noc2decoder_ack,
    output reg l15_pcxdecoder_header_ack,
    output reg l15_noc2decoder_header_ack,

    // CSM
    output reg [40-1:0] l15_csm_req_address_s2,
    output reg l15_csm_req_val_s2,
    output reg l15_csm_stall_s3,
    output reg [3-1:0] l15_csm_req_ticket_s2,
    // output reg [`HOME_ID_WIDTH-1:0] l15_csm_clump_tile_count_s2,
    output reg  l15_csm_req_type_s2,     //0 for load, 1 for store
    output reg [127:0] l15_csm_req_data_s2, // remember to duplicate this in l15
    output reg [33-1:0] l15_csm_req_pcx_data_s2, // remember to duplicate this in l15
    input wire csm_l15_res_val_s3,
    input wire [63:0] csm_l15_res_data_s3,

    // homeid info to noc1buffer
    output reg [3-1:0] l15_noc1buffer_req_csm_ticket,
    output reg [(14+8+8)-1:0] l15_noc1buffer_req_homeid,
    output reg l15_noc1buffer_req_homeid_val,

    // output to config registers to pipeline
    output reg l15_config_req_val_s2,
    output reg l15_config_req_rw_s2,
    output reg [63:0] l15_config_write_req_data_s2,
    output reg [15:8] l15_config_req_address_s2
    );

// GLOBAL VARIABLES
reg stall_s1;
reg stall_s2;
reg stall_s3;
reg val_s1; // val_s1 is basically predecode_val_s1, so anything that depends on this...
reg val_s2;
reg val_s3;

// ack signals
reg pcx_ack_s1;
reg pcx_ack_s2;
reg pcx_ack_s3;
reg noc2_ack_s1;
reg noc2_ack_s2;
reg noc2_ack_s3;

// fetch state signals
// used in different stages in S1
reg [3-1:0] fetch_state_s1;
reg [3-1:0] fetch_state_next_s1;

// addresses from s2&s3, used to calculate stall_s1
reg [((9-2))-1:0] cache_index_s2;
reg [((6 + 4) - 4 + 1)-1:0] cache_index_l1d_s2;
reg [((9-2))-1:0] cache_index_s3;
reg [((6 + 4) - 4 + 1)-1:0] cache_index_l1d_s3;

// bought early, borrowed to calculate dtag way...
reg [2-1:0] lru_way_s2;

// PCX/NOC2 acks aggregator
always @ *
begin
    l15_pcxdecoder_ack = pcx_ack_s1 || pcx_ack_s2 || pcx_ack_s3;
    l15_noc2decoder_ack = noc2_ack_s1 || noc2_ack_s2 || noc2_ack_s3;
end

//////////////////////////
// STAGE 1
//////////////////////////
// Stage 1 is relatively long, consisting of the (concurrent and sequential) sequences
// * predecode: checks pcx&noc1&noc3 inputs and translates them to internal requests. should depends on nothing
// * creditman: check noc1buffer and stalls the pcx input if no space. depends on predecode
// * fetchstate: keeps track of message fissioning (eg.: an invalidation needs to be splitted to 4 invals to 4 indices)
// * decoder: decodes internal requests -> control bits

//////////////////////////
// PREDECODE
//////////////////////////
// depends on nothing
reg [(40 - 4 - ((9-2)))-1:0] predecode_dtag_write_data_s1;
reg [((9-2))-1:0] predecode_cache_index_s1;
// reg [`L15_PADDR_MASK] predecode_mshr_address_s1;
reg [((((((((((0 + 1) + 1) + 1) + 1) + 3) + 1) + 2) + 1) + 1) + 1)-1:0] predecode_mshr_read_control_s1;
reg [(14+8+8)-1:0] predecode_mshr_read_homeid_s1;
reg [6-1:0] predecode_reqtype_s1;
reg [39:0] predecode_address_s1;
reg [39:0] predecode_address_plus0_s1;
reg [39:0] predecode_address_plus1_s1;
reg [39:0] predecode_address_plus2_s1;
reg [39:0] predecode_address_plus3_s1;
reg [2:0] predecode_size_s1;
reg [0:0] predecode_threadid_s1;
reg [2-1:0] predecode_l1_replacement_way_s1;
reg predecode_non_cacheable_s1;
reg predecode_is_last_inval_s1;
// reg predecode_icache_do_inval_s1;
reg predecode_blockstore_bit_s1;
reg predecode_blockstore_init_s1;
reg predecode_prefetch_bit_s1;
// reg predecode_invalidate_index_s1;
reg predecode_l2_miss_s1;
reg predecode_f4b_s1;
reg predecode_dcache_load_s1;
reg predecode_atomic_s1;
reg predecode_dcache_noc2_store_im_s1;
reg predecode_dcache_noc2_store_sm_s1;
reg predecode_icache_bit_s1;
reg predecode_noc2_inval_s1;
reg predecode_val_s1;
reg [2-1:0] predecode_source_s1;
reg predecode_interrupt_broadcast_s1;
reg [3:0] predecode_fwd_subcacheline_vector_s1;

always @ *
begin
    predecode_source_s1 = 0;

    case (fetch_state_s1)
        3'd0:
            predecode_source_s1 = (noc2decoder_l15_val) ? 2'd2 :
                                    (pcxdecoder_l15_val) ? 2'd1 :
                                                            2'd0;
        3'd1:
            predecode_source_s1 = 2'd1;
        3'd5:
            predecode_source_s1 = 2'd2;
        3'd2,
        3'd3,
        3'd4:
            predecode_source_s1 = 2'd2;
        3'd6:
            predecode_source_s1 = 2'd2;
    endcase
end

// retrieve mshr info from mshr module
// and expanding signals
reg [4-1:0] mshr_val_array [1:0];



reg [2-1:0] mshr_st_state_array [1:0];

reg [39:0] mshr_st_address_array [1:0];
reg [39:0] mshr_ld_address_array [1:0];
reg [2-1:0] mshr_st_way_array [1:0];
always @ *
begin
    pipe_mshr_readreq_mshrid_s1 = noc2decoder_l15_mshrid;
    pipe_mshr_readreq_threadid_s1 = noc2decoder_l15_threadid;

    predecode_mshr_read_control_s1 = mshr_pipe_readres_control_s1;
    // predecode_mshr_read_address_s1 = mshr_pipe_address_s1;
    predecode_mshr_read_homeid_s1 = mshr_pipe_readres_homeid_s1;

    // mshr_val_array
    mshr_val_array[0] = mshr_pipe_vals_s1[4*1 - 1 -: 4];
    mshr_st_state_array[0] = mshr_pipe_st_state_s1[2*1 - 1 -: 2];
    mshr_st_address_array[0] = mshr_pipe_st_address[40*1 - 1 -: 40];
    mshr_ld_address_array[0] = mshr_pipe_ld_address[40*1 - 1 -: 40];
    mshr_st_way_array[0] = mshr_pipe_st_way_s1[2*1 - 1 -: 2];

    mshr_val_array[1] = mshr_pipe_vals_s1[4*2 - 1 -: 4];
    mshr_st_state_array[1] = mshr_pipe_st_state_s1[2*2 - 1 -: 2];
    mshr_st_address_array[1] = mshr_pipe_st_address[40*2 - 1 -: 40];
    mshr_ld_address_array[1] = mshr_pipe_ld_address[40*2 - 1 -: 40];
    mshr_st_way_array[1] = mshr_pipe_st_way_s1[2*2 - 1 -: 2];
end

// match pcx address to special accesses
reg [8-1:0] predecode_special_access_s1;
reg predecode_is_pcx_config_asi_s1;
reg predecode_is_pcx_diag_data_access_s1;
reg predecode_is_pcx_diag_line_flush_s1;
reg predecode_is_hmc_diag_access_s1;
reg predecode_is_hmc_flush_s1;
always @ *
begin
    predecode_special_access_s1 = pcxdecoder_l15_address[39:32];
    predecode_is_pcx_config_asi_s1 = predecode_special_access_s1 == 8'hba;
    predecode_is_pcx_diag_data_access_s1 = predecode_special_access_s1 == 8'hb0;
    predecode_is_pcx_diag_line_flush_s1 = predecode_special_access_s1 == 8'hb3;
    predecode_is_hmc_diag_access_s1 = predecode_special_access_s1 == 8'hb2;
    predecode_is_hmc_flush_s1 = predecode_special_access_s1 == 8'hb5;
end

// decode requests to predecode signals
reg predecode_tagcheck_matched_t0ld_s1;
reg predecode_tagcheck_matched_t0st_s1;
reg predecode_tagcheck_matched_t1ld_s1;
reg predecode_tagcheck_matched_t1st_s1;
reg predecode_int_vec_dis_s1;
reg predecode_tagcheck_matched_s1;
reg [19:4] predecode_partial_tag_s1;
reg predecode_hit_stbuf_s1;
reg [0:0] predecode_hit_stbuf_threadid_s1;

wire [39:0] constant_int_vec_dis_address = 40'h9800000800;

always @ *
begin
    predecode_reqtype_s1 = 0;
    predecode_address_s1 = 0;
    predecode_address_plus0_s1 = 0;
    predecode_address_plus1_s1 = 0;
    predecode_address_plus2_s1 = 0;
    predecode_address_plus3_s1 = 0;
    predecode_is_last_inval_s1 = 0;
    // predecode_icache_do_inval_s1 = 0;
    predecode_size_s1 = 0;
    predecode_threadid_s1 = 0;
    predecode_l1_replacement_way_s1 = 0;
    predecode_non_cacheable_s1 = 0;
    predecode_blockstore_bit_s1 = 0;
    predecode_blockstore_init_s1 = 0;
    predecode_prefetch_bit_s1 = 0;
    // predecode_invalidate_index_s1 = 0;
    predecode_l2_miss_s1 = 0;
    predecode_f4b_s1 = 0;
    predecode_icache_bit_s1 = 0;
    predecode_dcache_load_s1 = 0;
    predecode_atomic_s1 = 0;
    predecode_dcache_noc2_store_im_s1 = 0;
    predecode_dcache_noc2_store_sm_s1 = 0;
    predecode_noc2_inval_s1 = 0;
    predecode_fwd_subcacheline_vector_s1 = 0;
    predecode_interrupt_broadcast_s1 = 0;
    case (predecode_source_s1)
        2'd2:
        begin
            // predecode_address_s1 = predecode_mshr_address_s1;
            predecode_size_s1 = predecode_mshr_read_control_s1[(((((0 + 1) + 1) + 1) + 1) + 3) -: 3];
            // predecode_threadid_s1 = predecode_mshr_read_control_s1[`L15_CONTROL_THREADID -: `L15_THREADID_WIDTH];
            predecode_threadid_s1[0:0] = noc2decoder_l15_threadid[0:0];
            predecode_l1_replacement_way_s1 = predecode_mshr_read_control_s1[(((((((0 + 1) + 1) + 1) + 1) + 3) + 1) + 2) -: 2];
            predecode_non_cacheable_s1 = predecode_mshr_read_control_s1[((((((((0 + 1) + 1) + 1) + 1) + 3) + 1) + 2) + 1) -: 1];
            // 4.16.14: disable blockstores
            predecode_blockstore_bit_s1 = predecode_mshr_read_control_s1[(((((((((0 + 1) + 1) + 1) + 1) + 3) + 1) + 2) + 1) + 1) -: 1];
            predecode_blockstore_init_s1 = predecode_mshr_read_control_s1[(0 + 1) -: 1];
            predecode_prefetch_bit_s1 = predecode_mshr_read_control_s1[0 -: 1];
            // predecode_invalidate_index_s1 = predecode_mshr_read_control_s1[`L15_CONTROL_INVALIDATE_INDEX_1B -: 1];
            predecode_l2_miss_s1 = noc2decoder_l15_l2miss;
            predecode_f4b_s1 = noc2decoder_l15_f4b;
            predecode_atomic_s1 = predecode_mshr_read_control_s1[(((0 + 1) + 1) + 1)];
            predecode_dcache_load_s1 = predecode_mshr_read_control_s1[((0 + 1) + 1)];
            predecode_fwd_subcacheline_vector_s1 = noc2decoder_l15_fwd_subcacheline_vector;

            predecode_dcache_noc2_store_im_s1 = mshr_st_state_array[predecode_threadid_s1] == 2'b10;
            predecode_dcache_noc2_store_sm_s1 = mshr_st_state_array[predecode_threadid_s1] == 2'b01;


            predecode_address_plus0_s1 = {noc2decoder_l15_address[39:6], 2'b00, noc2decoder_l15_address[3:0]};
            predecode_address_plus1_s1 = {noc2decoder_l15_address[39:6], 2'b01, noc2decoder_l15_address[3:0]};
            predecode_address_plus2_s1 = {noc2decoder_l15_address[39:6], 2'b10, noc2decoder_l15_address[3:0]};
            predecode_address_plus3_s1 = {noc2decoder_l15_address[39:6], 2'b11, noc2decoder_l15_address[3:0]};

            if (noc2decoder_l15_icache_type)
                predecode_is_last_inval_s1 = ((fetch_state_s1 == 3'd6) && (predecode_fwd_subcacheline_vector_s1[3:2] == 2'b11)) ||
                                             ((fetch_state_s1 == 3'd0) && (predecode_fwd_subcacheline_vector_s1[3:0] == 4'b0011));
                // predecode_is_last_inval_s1 = ((fetch_state_s1 == `L15_FETCH_STATE_ICACHE_INVAL_2));
            else
                predecode_is_last_inval_s1 = ((fetch_state_s1 == 3'd4) && (predecode_fwd_subcacheline_vector_s1[3])) ||
                                             ((fetch_state_s1 == 3'd3) && (predecode_fwd_subcacheline_vector_s1[3:2] == 2'b01)) ||
                                             ((fetch_state_s1 == 3'd2) && (predecode_fwd_subcacheline_vector_s1[3:1] == 3'b001)) ||
                                             ((fetch_state_s1 == 3'd0) && (predecode_fwd_subcacheline_vector_s1[3:0] == 4'b0001));

            // predecode_icache_do_inval_s1 = ((fetch_state_s1 == `L15_FETCH_STATE_ICACHE_INVAL_2) && predecode_fwd_subcacheline_vector_s1[3:2] == 2'b11) ||
            //                                 ((fetch_state_s1 == `L15_FETCH_STATE_NORMAL) && predecode_fwd_subcacheline_vector_s1[`L15_UNPARAM_1_0] == 2'b11);
            // predecode_icache_do_inval_s1 = 1'b1;

            case(noc2decoder_l15_reqtype)
                8'd17:
                begin
                    predecode_icache_bit_s1 = noc2decoder_l15_icache_type;
                    if (predecode_icache_bit_s1)
                    begin
                        // if (predecode_icache_do_inval_s1)
                            predecode_reqtype_s1 = 6'd18;
                        // else
                        //     predecode_reqtype_s1 = `L15_REQTYPE_IGNORE;
                        predecode_address_s1 = (fetch_state_s1 == 3'd6) ? predecode_address_plus2_s1 :
                                                                                                predecode_address_plus0_s1;
                    end
                    else
                    begin
                        predecode_reqtype_s1 = 6'd6;
                        predecode_address_s1 = (fetch_state_s1 == 3'd2) ? predecode_address_plus1_s1 :
                                                (fetch_state_s1 == 3'd3) ? predecode_address_plus2_s1 :
                                                (fetch_state_s1 == 3'd4) ? predecode_address_plus3_s1 :
                                                                                                predecode_address_plus0_s1;
                    end
                end
                8'd18:
                begin
                    predecode_icache_bit_s1 = noc2decoder_l15_icache_type;
                    if (predecode_icache_bit_s1)
                    begin
                        // if (predecode_icache_do_inval_s1)
                            predecode_reqtype_s1 = 6'd18;
                        // else
                        //     predecode_reqtype_s1 = `L15_REQTYPE_IGNORE;
                        predecode_address_s1 = (fetch_state_s1 == 3'd6) ? predecode_address_plus2_s1 :
                                                                                                predecode_address_plus0_s1;
                    end
                    else
                    begin
                        predecode_reqtype_s1 = 6'd6;
                        predecode_noc2_inval_s1 = 1'b1;
                        predecode_address_s1 = (fetch_state_s1 == 3'd2) ? predecode_address_plus1_s1 :
                                                (fetch_state_s1 == 3'd3) ? predecode_address_plus2_s1 :
                                                (fetch_state_s1 == 3'd4) ? predecode_address_plus3_s1 :
                                                                                                predecode_address_plus0_s1;
                    end
                end
                8'd16:
                begin
                    predecode_icache_bit_s1 = noc2decoder_l15_icache_type;
                    if (predecode_icache_bit_s1)
                    begin
                        // if (predecode_icache_do_inval_s1)
                            predecode_reqtype_s1 = 6'd18;
                        // else
                        //     predecode_reqtype_s1 = `L15_REQTYPE_IGNORE;
                        predecode_address_s1 = (fetch_state_s1 == 3'd6) ? predecode_address_plus2_s1 :
                                                                                                predecode_address_plus0_s1;
                    end
                    else
                    begin
                        predecode_reqtype_s1 = 6'd7;
                        predecode_address_s1 = (fetch_state_s1 == 3'd2) ? predecode_address_plus1_s1 :
                                                (fetch_state_s1 == 3'd3) ? predecode_address_plus2_s1 :
                                                (fetch_state_s1 == 3'd4) ? predecode_address_plus3_s1 :
                                                                                                predecode_address_plus0_s1;
                    end
                end
                8'd29:
                begin
                    predecode_icache_bit_s1 = predecode_mshr_read_control_s1[((((0 + 1) + 1) + 1) + 1)];

                    if (noc2decoder_l15_mshrid == 2'd2)
                        predecode_address_s1 = mshr_ld_address_array[noc2decoder_l15_threadid];
                    else if (noc2decoder_l15_mshrid == 2'd3)
                        predecode_address_s1 = mshr_st_address_array[noc2decoder_l15_threadid];

                    if (noc2decoder_l15_hmc_fill)
                    begin
                        predecode_reqtype_s1 = 6'd28;
                        predecode_address_s1 = 0;   // more bug fix, no mshr entry is associated with this
                    end
                    else
                    if (predecode_non_cacheable_s1)
                    begin
                        if (predecode_icache_bit_s1)
                            predecode_reqtype_s1 = 6'd9;
                        else if (predecode_dcache_load_s1)
                            predecode_reqtype_s1 = 6'd8;
                        else if (predecode_atomic_s1)
                            predecode_reqtype_s1 = 6'd14;
                        else
                            predecode_reqtype_s1 = 6'd1; // error case
                    end
                    else
                    begin
                        if (predecode_icache_bit_s1)
                            predecode_reqtype_s1 = 6'd9;
                        else if (predecode_dcache_load_s1)
                            predecode_reqtype_s1 = 6'd10;
                        else if (predecode_dcache_noc2_store_im_s1)
                            predecode_reqtype_s1 = 6'd11;
                        else if (predecode_dcache_noc2_store_sm_s1)
                            predecode_reqtype_s1 = 6'd12;
                        else if (predecode_atomic_s1)
                            predecode_reqtype_s1 = 6'd46;
                            // Only Load Reserve is both Cacheable and AMO 
                            // (just store NC=0 to mshr[in MSHR S1 logic], nc bit is still 1 in LR PCX req)
                        else
                            predecode_reqtype_s1 = 6'd1; // error case
                    end
                end
                8'd28:
                begin
                    predecode_icache_bit_s1 = predecode_mshr_read_control_s1[((((0 + 1) + 1) + 1) + 1)];
                    predecode_address_s1 = mshr_st_address_array[noc2decoder_l15_threadid];

                    // one way to distinguish prefetch ack from write-through ack is the mshrid
                    if (noc2decoder_l15_mshrid == 2'd3)
                        predecode_reqtype_s1 = 6'd13;
                    else
                    begin
                        predecode_reqtype_s1 = 6'd32;
                        predecode_address_s1 = 0; // bug fix, address cannot be X's
                    end

                end
                8'd33:
                begin
                    predecode_reqtype_s1 = 6'd20;
                end
            endcase
        end

        2'd1:
        begin
            predecode_address_s1 = pcxdecoder_l15_address;
            predecode_size_s1 = pcxdecoder_l15_size;
            predecode_threadid_s1[0:0] = pcxdecoder_l15_threadid[0:0];
            predecode_l1_replacement_way_s1 = pcxdecoder_l15_l1rplway;
            predecode_non_cacheable_s1 = pcxdecoder_l15_nc;
            predecode_blockstore_bit_s1 = pcxdecoder_l15_blockstore;
            predecode_blockstore_init_s1 = pcxdecoder_l15_blockinitstore;
            predecode_prefetch_bit_s1 = pcxdecoder_l15_prefetch;
            // predecode_invalidate_index_s1 = pcxdecoder_l15_invalall;

            case(pcxdecoder_l15_rqtype)
                5'b00000:
                begin
                    if (predecode_is_pcx_config_asi_s1)
                        predecode_reqtype_s1 = 6'd24;
                    else if (predecode_is_pcx_diag_data_access_s1)
                        predecode_reqtype_s1 = 6'd25;
                    else if (predecode_is_hmc_diag_access_s1)
                        predecode_reqtype_s1 = 6'd29;
                    else if (predecode_prefetch_bit_s1)
                        predecode_reqtype_s1 = 6'd16;
                    else if (predecode_non_cacheable_s1)
                        predecode_reqtype_s1 = 6'd15;
                    else if (pcxdecoder_l15_invalidate_cacheline)
                        predecode_reqtype_s1 = 6'd34;
                    else
                        predecode_reqtype_s1 = 6'd21;
                    predecode_dcache_load_s1 = 1;
                end
                5'b10000:
                begin
                    if (pcxdecoder_l15_invalidate_cacheline)
                        predecode_reqtype_s1 = 6'd33;
                    else
                        predecode_reqtype_s1 = 6'd2;
                    predecode_icache_bit_s1 = 1;
                end
                5'b00001:
                    if (predecode_is_pcx_config_asi_s1)
                        predecode_reqtype_s1 = 6'd23;
                    else if (predecode_is_pcx_diag_data_access_s1)
                        predecode_reqtype_s1 = 6'd26;
                    else if (predecode_is_pcx_diag_line_flush_s1)
                        predecode_reqtype_s1 = 6'd27;
                    else if (predecode_is_hmc_diag_access_s1)
                        predecode_reqtype_s1 = 6'd30;
                    else if (predecode_is_hmc_flush_s1)
                        predecode_reqtype_s1 = 6'd31;
                    else if (predecode_non_cacheable_s1)
                    begin
                        predecode_reqtype_s1 = 6'd17;
                        // bug 108: clear blocksotre/prefetch bit to distinguish returned prefetch load
                        // predecode_blockstore_bit_s1 = 1'b0;
                        // predecode_blockstore_init_s1 = 1'b0;
                        predecode_prefetch_bit_s1 = 1'b0;
                    end
                    else
                        predecode_reqtype_s1 = 6'd3;
                //`PCX_REQTYPE_CAS1:
                //begin
                //    predecode_reqtype_s1 = `L15_REQTYPE_CAS;
                //    predecode_atomic_s1 = 1;
                //end
                //`PCX_REQTYPE_CAS2:
                //    predecode_reqtype_s1 = `L15_REQTYPE_IGNORE;
                //`PCX_REQTYPE_SWP_LOADSTUB:
                //begin
                //    predecode_reqtype_s1 = `L15_REQTYPE_SWP_LOADSTUB;
                //    predecode_atomic_s1 = 1;
                //end
                5'b00110:
                begin
                    case (pcxdecoder_l15_amo_op)
                    4'b0000:
                    begin
                        predecode_reqtype_s1 = 6'd1;
                    end
                    4'b0001:
                    begin
                        predecode_reqtype_s1 = 6'd35;
                        predecode_atomic_s1 = 1;
                    end
                    4'b0010:
                    begin
                        predecode_reqtype_s1 = 6'd36;
                        predecode_atomic_s1 = 1;
                    end
                    4'b0011:
                    begin
                        predecode_reqtype_s1 = 6'd5;
                        predecode_atomic_s1 = 1;
                    end
                    4'b0100:
                    begin
                        predecode_reqtype_s1 = 6'd38;
                        predecode_atomic_s1 = 1;
                    end
                    4'b0101:
                    begin
                        predecode_reqtype_s1 = 6'd39;
                        predecode_atomic_s1 = 1;
                    end
                    4'b0110:
                    begin
                        predecode_reqtype_s1 = 6'd40;
                        predecode_atomic_s1 = 1;
                    end
                    4'b0111:
                    begin
                        predecode_reqtype_s1 = 6'd41;
                        predecode_atomic_s1 = 1;
                    end
                    4'b1000:
                    begin
                        predecode_reqtype_s1 = 6'd42;
                        predecode_atomic_s1 = 1;
                    end
                    4'b1001:
                    begin
                        predecode_reqtype_s1 = 6'd43;
                        predecode_atomic_s1 = 1;

                    end
                    4'b1010:
                    begin
                        predecode_reqtype_s1 = 6'd44;
                        predecode_atomic_s1 = 1;

                    end
                    4'b1011:
                    begin
                        predecode_reqtype_s1 = 6'd45;
                        predecode_atomic_s1 = 1;

                    end
                    4'b1100:
                    begin
                        predecode_reqtype_s1 = 6'd4;
                        predecode_atomic_s1 = 1;
                    end
                    4'b1101:
                    begin
                        predecode_reqtype_s1 = 6'd1;
                    end
                    endcase
                end
                5'b01001:
                begin
                    predecode_reqtype_s1 = 6'd19;
                    predecode_interrupt_broadcast_s1 = predecode_non_cacheable_s1;
                end
                5'b01010:
                    predecode_reqtype_s1 = 6'd1;
                5'b01011:
                    predecode_reqtype_s1 = 6'd1;
                5'b00100:
                    predecode_reqtype_s1 = 6'd1;
                5'b00101:
                    predecode_reqtype_s1 = 6'd1;
                5'b01101:
                    predecode_reqtype_s1 = 6'd1;
                5'b01110:
                    predecode_reqtype_s1 = 6'd1;
            endcase
        end
    endcase

    predecode_val_s1 = (predecode_source_s1 != 2'd0);
    val_s1 = predecode_val_s1;

    predecode_cache_index_s1[((9-2))-1:0]
         = predecode_address_s1[(((9-2))+4-1):4]; // index should be 7b (128 indices); // trinn
    predecode_dtag_write_data_s1[(40 - 4 - ((9-2)))-1:0] = predecode_address_s1[(39):((((9-2))+4-1) + 1)];

    // GENERATE INTERRUPT
    // `define L15_INT_VEC_DIS 40'h98_0000_0800
    // predecode_int_vec_dis_s1 = (pcxdecoder_l15_address == `L15_INT_VEC_DIS); // 40b compare
    predecode_int_vec_dis_s1 = (pcxdecoder_l15_address[39:32] == constant_int_vec_dis_address[39:32] 
                              && pcxdecoder_l15_address[11:8] == constant_int_vec_dis_address[11:8]); 


    // TAG CHECKING
    predecode_partial_tag_s1[19:4] = pcxdecoder_l15_address[19:4]; // compare partial tag to save energy & timing
    predecode_tagcheck_matched_t0ld_s1 = mshr_val_array[0][2'd2] 
                                        && (predecode_partial_tag_s1[19:4] == mshr_ld_address_array[0][19:4]);
    predecode_tagcheck_matched_t1ld_s1 = mshr_val_array[1][2'd2] 
                                        && (predecode_partial_tag_s1[19:4] == mshr_ld_address_array[1][19:4]);
    predecode_tagcheck_matched_t0st_s1 = mshr_val_array[0][2'd3] 
                                        && (pcxdecoder_l15_address[39:4] == mshr_st_address_array[0][39:4]);
    predecode_tagcheck_matched_t1st_s1 = mshr_val_array[1][2'd3] 
                                        && (pcxdecoder_l15_address[39:4] == mshr_st_address_array[1][39:4]);

    predecode_tagcheck_matched_s1 = predecode_tagcheck_matched_t0ld_s1 || predecode_tagcheck_matched_t1ld_s1
                                    || predecode_tagcheck_matched_t0st_s1 || predecode_tagcheck_matched_t1st_s1;


    // misc
    predecode_hit_stbuf_s1 = predecode_tagcheck_matched_t0st_s1 || predecode_tagcheck_matched_t1st_s1;
    predecode_hit_stbuf_threadid_s1 = predecode_tagcheck_matched_t1st_s1 ? 1'b1 : 1'b0;
    // note: only work with 2 threads for now; need to change the algo of mshr if need to increase the num of threads
end

//////////////////////////
// NOC1 CREDIT MANAGEMENT
//////////////////////////
// this module is needed to ensure that interaction with Noc1 does not
// deadlock NoC.
// creditman depends on predecode (and stall signal)
// its dependencies are: none

reg [3:0] creditman_noc1_avail;
reg [3:0] creditman_noc1_data_avail;
reg [3:0] creditman_noc1_avail_next;
reg [3:0] creditman_noc1_data_avail_next;
reg [3:0] creditman_noc1_reserve;
reg [3:0] creditman_noc1_reserve_next;

always @ (posedge clk)
begin
    if (!rst_n)
    begin
        creditman_noc1_avail <= 8;
        creditman_noc1_data_avail <= 2;
        creditman_noc1_reserve <= 0;
    end
    else
    begin
        creditman_noc1_avail <= creditman_noc1_avail_next;
        creditman_noc1_data_avail <= creditman_noc1_data_avail_next;
        creditman_noc1_reserve <= creditman_noc1_reserve_next;
    end
end

reg creditman_noc1_data_add1;
reg creditman_noc1_data_add2;
reg creditman_noc1_data_minus1;
reg creditman_noc1_data_minus2;
reg creditman_noc1_add2;
reg creditman_noc1_add1;
reg creditman_noc1_minus1;
reg creditman_noc1_minus2;
reg creditman_noc1_reserve_add1;
reg creditman_noc1_reserve_minus1;

always @ *
begin
    creditman_noc1_avail_next = creditman_noc1_add2 ? creditman_noc1_avail + 2 :
                             creditman_noc1_add1 ? creditman_noc1_avail + 1 :
                             creditman_noc1_minus1 ? creditman_noc1_avail - 1 :
                             creditman_noc1_minus2 ? creditman_noc1_avail - 2 :
                                                creditman_noc1_avail;
    creditman_noc1_data_avail_next = creditman_noc1_data_add1 ? creditman_noc1_data_avail + 1 :
                                    creditman_noc1_data_add2 ? creditman_noc1_data_avail + 2 :
                                    creditman_noc1_data_minus1 ? creditman_noc1_data_avail - 1 :
                                    creditman_noc1_data_minus2 ? creditman_noc1_data_avail - 2 :
                                                            creditman_noc1_data_avail;

    creditman_noc1_reserve_next = creditman_noc1_reserve_add1 ? creditman_noc1_reserve + 1 :
                                  creditman_noc1_reserve_minus1 ? creditman_noc1_reserve - 1 :
                                                                creditman_noc1_reserve;
end

reg creditman_noc1_mispredicted_s3;
reg creditman_noc1_reserve_s3;
reg creditman_noc1_req;

reg creditman_noc1_upX;
reg creditman_noc1_up1;
reg creditman_noc1_up2;
reg creditman_noc1_down1;
reg creditman_noc1_down2;
reg creditman_noc1_data_up1;
reg creditman_noc1_data_up2;
reg creditman_noc1_data_down1;
reg creditman_noc1_data_down2;

reg decoder_creditman_req_8B_s1;
reg decoder_creditman_req_16B_s1;
reg [1:0] decoder_creditman_noc1_needed;
reg decoder_creditman_noc1_unreserve_s1;

always @ *
begin
    creditman_noc1_req = val_s1 && !stall_s1 && (decoder_creditman_noc1_needed != 2'd0);

    // misprediction is necessary because we want to allocate buffer at the beginning but not until
    //  tag access will the pipeline know whether a noc1 transaction is needed or not
    creditman_noc1_upX = noc1encoder_l15_req_sent || creditman_noc1_mispredicted_s3;
    creditman_noc1_up1 = noc1encoder_l15_req_sent ^ creditman_noc1_mispredicted_s3;
    creditman_noc1_up2 = noc1encoder_l15_req_sent && creditman_noc1_mispredicted_s3;
    creditman_noc1_down1 = creditman_noc1_req && decoder_creditman_noc1_needed == 2'd1;
    creditman_noc1_down2 = creditman_noc1_req && decoder_creditman_noc1_needed == 2'd2;

    creditman_noc1_add2 = creditman_noc1_up2 && ~creditman_noc1_req;
    creditman_noc1_add1 = (creditman_noc1_up2 && creditman_noc1_down1) || (creditman_noc1_up1 && ~creditman_noc1_req);
    creditman_noc1_minus1 = (creditman_noc1_down2 && creditman_noc1_up1) || (creditman_noc1_down1 && !creditman_noc1_upX);
    creditman_noc1_minus2 = creditman_noc1_down2 && !creditman_noc1_upX;

    creditman_noc1_data_up1 = noc1encoder_l15_req_sent && (noc1encoder_l15_req_data_sent == 2'd1);
    // COV: 0 and 1 is impossible here
    creditman_noc1_data_up2 = noc1encoder_l15_req_sent && (noc1encoder_l15_req_data_sent == 2'd2);

    creditman_noc1_data_down1 = creditman_noc1_req && decoder_creditman_req_8B_s1;
    creditman_noc1_data_down2 = creditman_noc1_req && decoder_creditman_req_16B_s1;

    creditman_noc1_data_add2 = creditman_noc1_data_up2 && !creditman_noc1_data_down1 && !creditman_noc1_data_down2;
    creditman_noc1_data_add1 = creditman_noc1_data_up1 &&  !creditman_noc1_data_down1 && !creditman_noc1_data_down2 ||
                            creditman_noc1_data_up2 && creditman_noc1_data_down1;
    creditman_noc1_data_minus2 = creditman_noc1_data_down2 && !creditman_noc1_data_up1 && !creditman_noc1_data_up2;
    creditman_noc1_data_minus1 = creditman_noc1_data_down1 &&  !creditman_noc1_data_up1 && !creditman_noc1_data_up2 ||
                            creditman_noc1_data_down2 && creditman_noc1_data_up1;


    creditman_noc1_reserve_add1 = (creditman_noc1_reserve_s3 && !stall_s3 && val_s3) 
                                    && !(val_s1 && !stall_s1 && decoder_creditman_noc1_unreserve_s1);
    creditman_noc1_reserve_minus1 = !(creditman_noc1_reserve_s3 && !stall_s3 && val_s3) 
                                    && (val_s1 && !stall_s1 && decoder_creditman_noc1_unreserve_s1);
end

////////////////////////////////
// Fetch_state stage (s1)
////////////////////////////////
// fetch state comes after predecoding because fetch_state_next needs information from predecoding
//  (as shown in below temp variables)
// Other stages in S1 can use fetch_state because it's a flop
// depends on predecode

// temp variables
reg fetch_is_pcx_atomic_instruction_s1;
// reg fetch_is_pcx_blockstore_instruction_s1;
reg fetch_is_pcx_storenc_instruction_s1;
reg fetch_is_pcx_loadnc_instruction_s1;
reg fetch_is_noc2_data_invalidation_s1;
reg fetch_is_noc2_instruction_invalidation_s1;
reg fetch_is_noc2_ackdt_s1;
reg fetch_is_pcx_flush_s1;

always @ (posedge clk)
begin
    if (!rst_n)
    begin
        fetch_state_s1 <= 3'd0;
    end
    else
    begin
        fetch_state_s1 <= fetch_state_next_s1;
    end
end

always @ *
begin
    fetch_is_pcx_atomic_instruction_s1 =
        (predecode_reqtype_s1 == 6'd4 ||
         predecode_reqtype_s1 == 6'd5 ||
         predecode_reqtype_s1 == 6'd35 ||
         predecode_reqtype_s1 == 6'd36 ||
         predecode_reqtype_s1 == 6'd38 ||
         predecode_reqtype_s1 == 6'd39 ||
         predecode_reqtype_s1 == 6'd40 ||
         predecode_reqtype_s1 == 6'd41 ||
         predecode_reqtype_s1 == 6'd42 ||
         predecode_reqtype_s1 == 6'd43 ||
         predecode_reqtype_s1 == 6'd44 ||
         predecode_reqtype_s1 == 6'd45);
    // fetch_is_pcx_blockstore_instruction_s1 = 0;
    fetch_is_pcx_storenc_instruction_s1 = (predecode_reqtype_s1 == 6'd17) && !predecode_int_vec_dis_s1;
    fetch_is_pcx_loadnc_instruction_s1 = (predecode_reqtype_s1 == 6'd15);
    fetch_is_noc2_data_invalidation_s1 = (predecode_reqtype_s1 == 6'd6 ||
                                    predecode_reqtype_s1 == 6'd7);
    fetch_is_noc2_instruction_invalidation_s1 = (predecode_reqtype_s1 == 6'd18);
    fetch_is_noc2_ackdt_s1 = (predecode_reqtype_s1 == 6'd10 || predecode_reqtype_s1 == 6'd11 || predecode_reqtype_s1 == 6'd46);
    fetch_state_next_s1 = 3'd0;
    fetch_is_pcx_flush_s1 = predecode_reqtype_s1 == 6'd27;

    case (fetch_state_s1)
        3'd0:
        begin
            fetch_state_next_s1 = 3'd0;
            if (!stall_s1)
            begin
                // AMO_SC can be finished within one cycle.
                if ((fetch_is_pcx_atomic_instruction_s1 && (predecode_reqtype_s1 != 6'd36))
                    || fetch_is_pcx_storenc_instruction_s1 || fetch_is_pcx_loadnc_instruction_s1 || fetch_is_pcx_flush_s1)
                    fetch_state_next_s1 = 3'd1;
                else if (fetch_is_noc2_data_invalidation_s1)
                    fetch_state_next_s1 = 3'd2;
                else if (fetch_is_noc2_instruction_invalidation_s1)
                    fetch_state_next_s1 = 3'd6;
                else if (fetch_is_noc2_ackdt_s1)
                    fetch_state_next_s1 = 3'd5;
            end
        end
        3'd1:
        begin
            fetch_state_next_s1 = 3'd1;
            if (!stall_s1)
                fetch_state_next_s1 = 3'd0;
        end
        3'd5:
        begin
            fetch_state_next_s1 = 3'd5;
            if (!stall_s1)
                fetch_state_next_s1 = 3'd0;
        end
        3'd2:
        begin
            fetch_state_next_s1 = 3'd2;
            if (!stall_s1)
                fetch_state_next_s1 = 3'd3;
        end
        3'd3:
        begin
            fetch_state_next_s1 = 3'd3;
            if (!stall_s1)
                fetch_state_next_s1 = 3'd4;
        end
        3'd4:
        begin
            fetch_state_next_s1 = 3'd4;
            if (!stall_s1)
                fetch_state_next_s1 = 3'd0;
        end
        3'd6:
        begin
            fetch_state_next_s1 = 3'd6;
            if (!stall_s1)
                fetch_state_next_s1 = 3'd0;
        end
    endcase
end

////////////////////////////////
// decoder stage (s1)
////////////////////////////////
// generate control bits from instruction
reg [2-1:0] decoder_pcx_ack_stage_s1;
reg [2-1:0] decoder_noc2_ack_stage_s1;
reg decoder_stall_on_mshr_allocation_s1;
reg [2-1:0] decoder_mshr_allocation_type_s1;
reg decoder_stall_on_matched_bypassed_index_s1;
// reg decoder_stall_on_ld_mshr_s1;
// reg decoder_stall_on_st_mshr_s1;
reg [2-1:0]decoder_s1_mshr_operation_s1;
reg [2-1:0]decoder_dtag_operation_s1;
reg [1-1:0]decoder_s2_mshr_operation_s1;
reg [1-1:0]decoder_mesi_read_op_s1;
reg [4-1:0]decoder_dcache_operation_s1;
reg [3-1:0]decoder_s3_mshr_operation_s1;
reg [3-1:0]decoder_mesi_write_op_s1;
reg [1-1:0]decoder_wmt_read_op_s1;
reg [3-1:0]decoder_wmt_write_op_s1;
reg [3-1:0]decoder_wmt_compare_op_s1;
reg [3-1:0]decoder_lruarray_write_op_s1;
reg [5-1:0]decoder_cpx_operation_s1;

reg [1-1:0]decoder_hmt_op_s1;

reg [5-1:0]decoder_noc1_operation_s1;
reg [4-1:0]decoder_noc3_operation_s1;
reg [4-1:0]decoder_csm_op_s1;
reg [2-1:0]decoder_config_op_s1;
reg decoder_no_free_mshr_s1;
reg decoder_stall_on_matched_mshr_s1;
reg [2-1:0]decoder_mshrid_s1;
reg decoder_lrsc_flag_read_op_s1;
reg [3-1:0] decoder_lrsc_flag_write_op_s1;

always @ *
begin
    decoder_pcx_ack_stage_s1 = 1'b0;
    decoder_noc2_ack_stage_s1 = 1'b0;
    decoder_stall_on_mshr_allocation_s1 = 1'b0;
    decoder_stall_on_matched_bypassed_index_s1 = 1'b0;
    // decoder_stall_on_ld_mshr_s1 = 1'b0;
    // decoder_stall_on_st_mshr_s1 = 1'b0;
    decoder_s1_mshr_operation_s1 = 1'b0;
    decoder_dtag_operation_s1 = 1'b0;
    decoder_s2_mshr_operation_s1 = 1'b0;
    decoder_mesi_read_op_s1 = 1'b0;
    decoder_dcache_operation_s1 = 1'b0;
    decoder_s3_mshr_operation_s1 = 1'b0;
    decoder_mesi_write_op_s1 = 1'b0;
    decoder_wmt_read_op_s1 = 1'b0;
    decoder_wmt_write_op_s1 = 1'b0;
    decoder_wmt_compare_op_s1 = 1'b0;
    decoder_lruarray_write_op_s1 = 1'b0;
    decoder_cpx_operation_s1 = 1'b0;
    decoder_noc1_operation_s1 = 1'b0;
    decoder_noc3_operation_s1 = 1'b0;
    decoder_csm_op_s1 = 1'b0;
    decoder_config_op_s1 = 1'b0;
    decoder_creditman_noc1_needed = 2'b0;
    decoder_creditman_noc1_unreserve_s1 = 1'b0;
    decoder_creditman_req_8B_s1 = 1'b0;
    decoder_creditman_req_16B_s1 = 1'b0;
    decoder_stall_on_matched_mshr_s1 = 1'b0;
    decoder_mshr_allocation_type_s1 = 0; 
    decoder_lrsc_flag_read_op_s1 = 1'b0;
    decoder_lrsc_flag_write_op_s1 = 2'b0;
    decoder_no_free_mshr_s1 = 0;
    
    decoder_hmt_op_s1 = 0;
    
    case (predecode_reqtype_s1)
        6'd15:
        begin
            if (fetch_state_s1 == 3'd0)
            begin // write-back for nc loads
                decoder_stall_on_matched_bypassed_index_s1 = 1;
                decoder_stall_on_matched_mshr_s1 = 1;
                decoder_stall_on_mshr_allocation_s1 = 1'b1;
                decoder_mshr_allocation_type_s1 = 2'd2;

                decoder_dtag_operation_s1 = 2'd1;
                decoder_mesi_read_op_s1 = 1'd1;
                decoder_mesi_write_op_s1 = 3'd3;
                decoder_lrsc_flag_write_op_s1 = 3'd2;
                decoder_wmt_read_op_s1 = 1'd1;
                decoder_wmt_write_op_s1 = 3'd1;
                decoder_wmt_compare_op_s1 = 3'd2;

                decoder_dcache_operation_s1 = 4'd1;
                // decoder_hmt_op_s1 = `L15_HMT_READ_TAGCHECK_WAY_IF_M;
                decoder_lruarray_write_op_s1 = 3'd4;

                decoder_cpx_operation_s1 = 5'd1;
                decoder_noc1_operation_s1 = 5'd9;
                decoder_creditman_noc1_needed = 2'd2; // this one is for the subsequent load too
                decoder_noc3_operation_s1 = 4'd5;
                decoder_csm_op_s1 = 4'd10;
            end
            else
            begin // nc load
                decoder_pcx_ack_stage_s1 = 2'd1;
                decoder_s1_mshr_operation_s1 = 2'd1;
                decoder_mshr_allocation_type_s1 = 2'd2;
                decoder_csm_op_s1 = 4'd1;
                decoder_noc1_operation_s1 = 5'd1;
            end
        end

        6'd16:
        begin
            // if (tagcheck_tag_match_s1)
            // begin // prefetch ld hits mshr
            //    decoder_pcx_ack_stage_s1 = `L15_ACK_STAGE_S1;
            //    decoder_stall_on_matched_bypassed_index_s1 = 1;
            //    decoder_cpx_operation_s1 = `L15_CPX_GEN_LD_RESPONSE_BOGUS_DATA;
            // end
            // else
            // begin // prefetch ld
            //    decoder_pcx_ack_stage_s1 = `L15_ACK_STAGE_S1;
            //    decoder_stall_on_matched_bypassed_index_s1 = 1;
            //    decoder_stall_on_ld_mshr_s1 = 1;
            //    decoder_stall_on_st_mshr_s1 = 1;
            //    decoder_stall_on_mshr_allocation_s1 = `L15_MSHR_ALLOCATE_TYPE_LD;
            //    decoder_s1_mshr_operation_s1 = `L15_S1_MSHR_OP_ALLOCATE_LD;
            //    decoder_dtag_operation_s1 = `L15_DTAG_OP_READ;
            //    decoder_mesi_read_op_s1 = `L15_S2_MESI_READ;
            //    decoder_csm_op_s1 = `L15_CSM_OP_READ_GHID;
            //    decoder_s3_mshr_operation_s1 = `L15_S3_MSHR_OP_DEALLOCATION_IF_TAGCHECK_MES;
            //    decoder_cpx_operation_s1 = `L15_CPX_GEN_LD_RESPONSE_BOGUS_DATA_IF_TAGCHECK_MES;
            //    decoder_noc1_operation_s1 = `L15_NOC1_GEN_DATA_LD_REQUEST_IF_TAGCHECK_MISS;
            //    decoder_creditman_noc1_needed = 2'd1;
            // end
            decoder_pcx_ack_stage_s1 = 2'd1;
            // decoder_csm_op_s1 = `L15_CSM_OP_READ_GHID;
            decoder_cpx_operation_s1 = 5'd2;
            // decoder_noc1_operation_s1 = `L15_NOC1_GEN_DATA_LD_REQUEST;
            // decoder_creditman_noc1_needed = 2'd1;
        end

        6'd21:
        begin // a normal ld
            decoder_pcx_ack_stage_s1 = 2'd1;
            decoder_stall_on_matched_bypassed_index_s1 = 1;
            decoder_stall_on_matched_mshr_s1 = 1;
            decoder_stall_on_mshr_allocation_s1 = 1'b1;
            decoder_mshr_allocation_type_s1 = 2'd2;
            decoder_s1_mshr_operation_s1 = 2'd1;
            decoder_dtag_operation_s1 = 2'd1;
            decoder_mesi_read_op_s1 = 1'd1;
            decoder_dcache_operation_s1 = 4'd2;
            decoder_s3_mshr_operation_s1 = 3'd2;
            decoder_cpx_operation_s1 = 5'd4;
            decoder_wmt_read_op_s1 = 1'd1;
            decoder_wmt_write_op_s1 = 3'd4;
            decoder_wmt_compare_op_s1 = 3'd2;
            decoder_csm_op_s1 = 4'd6;
            decoder_lruarray_write_op_s1 = 3'd1;
            decoder_noc1_operation_s1 = 5'd2;
            decoder_creditman_noc1_needed = 2'd1;
        end

        6'd25:
        begin // a diagnostic read
            decoder_pcx_ack_stage_s1 = 2'd1;
            decoder_dcache_operation_s1 = 4'd8;
            decoder_cpx_operation_s1 = 5'd16;
        end

        6'd2:
        begin
            decoder_pcx_ack_stage_s1 = 2'd1;

            decoder_stall_on_mshr_allocation_s1 = 1'b1;
            decoder_mshr_allocation_type_s1 = 2'd1;
            decoder_s1_mshr_operation_s1 = 2'd1;
            decoder_csm_op_s1 = 4'd1;
            decoder_noc1_operation_s1 = 5'd3;
            decoder_creditman_noc1_needed = 2'd1;
        end

        6'd17:
        begin
            if (predecode_int_vec_dis_s1)
            begin
                // is actually an ASI store to generate cross-cpu interrupt
                //`L15_REQTYPE_INT_VEC_DIS:
                decoder_pcx_ack_stage_s1 = 2'd3;
                decoder_cpx_operation_s1 = 5'd6;
                decoder_noc1_operation_s1 = 5'd10;
                decoder_creditman_req_8B_s1 = 1'b1;
                decoder_creditman_noc1_needed = 2'd1;
            end
            else
            begin
                if (fetch_state_s1 == 3'd0)
                begin // write-back for st nc or blockstore
                    decoder_stall_on_matched_bypassed_index_s1 = 1;
                    decoder_stall_on_matched_mshr_s1 = 1;
                    decoder_stall_on_mshr_allocation_s1 = 1'b1;
                    decoder_mshr_allocation_type_s1 = 2'd3;

                    decoder_dtag_operation_s1 = 2'd1;

                    decoder_mesi_read_op_s1 = 1'd1;
                    decoder_dcache_operation_s1 = 4'd1;

                    decoder_mesi_write_op_s1 = 3'd3;
                    decoder_lrsc_flag_write_op_s1 = 3'd2;
                    // decoder_wmt_operation_s1 = `L15_WMT_READ_TAGCHECK_WAY_IF_MES_AND_DEMAP_ENTRY;
                    decoder_wmt_read_op_s1 = 1'd1;
                    decoder_wmt_write_op_s1 = 3'd1;
                    decoder_wmt_compare_op_s1 = 3'd2;
                    decoder_lruarray_write_op_s1 = 3'd4;

                    decoder_cpx_operation_s1 = 5'd1;
                    decoder_noc1_operation_s1 = 5'd9;
                    // decoder_creditman_noc1_needed = 2'd1;
                    decoder_creditman_noc1_needed = 2'd2; // this one is for the subsequent write too
                    decoder_noc3_operation_s1 = 4'd5;
                    // moved for atomicity
                    decoder_creditman_req_8B_s1 = 1'b1;
                    decoder_csm_op_s1 = 4'd10;
                end
                else
                begin // st nc or blockstore
                    decoder_pcx_ack_stage_s1 = 2'd3;
                    decoder_s1_mshr_operation_s1 = 2'd1;
                    decoder_mshr_allocation_type_s1 = 2'd3;
                    decoder_csm_op_s1 = 4'd1;
                    decoder_noc1_operation_s1 = 5'd5;

                    decoder_s3_mshr_operation_s1 = 3'd6;
                    
                    // decoder_creditman_req_8B_s1 = 1'b1;
                    // decoder_creditman_noc1_needed = 2'd1;
                end
            end
        end

        6'd3:
        begin
            if (predecode_hit_stbuf_s1)
            begin // st hits st MSHR in IM or SM state
                decoder_pcx_ack_stage_s1 = 2'd1;
                decoder_stall_on_matched_bypassed_index_s1 = 1'b1;
                decoder_stall_on_mshr_allocation_s1 = 1'b1;
                decoder_mshr_allocation_type_s1 = 2'd3;
                decoder_s1_mshr_operation_s1 = 2'd2;
                decoder_mesi_read_op_s1 = 1'd1;
                decoder_wmt_read_op_s1 = 1'd1;
                decoder_wmt_compare_op_s1 = 3'd4;
                decoder_cpx_operation_s1 = 5'd11;
            end
            else
            begin // regular st
                decoder_pcx_ack_stage_s1 = 2'd1;
                decoder_stall_on_matched_bypassed_index_s1 = 1'b1;
                decoder_stall_on_matched_mshr_s1 = 1'b1;
                decoder_stall_on_mshr_allocation_s1 = 1'b1;
                decoder_s1_mshr_operation_s1 = 2'd1;
                decoder_mshr_allocation_type_s1 = 2'd3;
                decoder_dtag_operation_s1 = 2'd1;

                decoder_mesi_read_op_s1 = 1'd1;
                decoder_s2_mshr_operation_s1 = 1'd1;
                decoder_dcache_operation_s1 = 4'd4;

                decoder_wmt_read_op_s1 = 1'd1;
                decoder_wmt_compare_op_s1 = 3'd2;

                decoder_lruarray_write_op_s1 = 3'd1;
                decoder_csm_op_s1 = 4'd2;
                decoder_s3_mshr_operation_s1 = 3'd3;
                decoder_mesi_write_op_s1 = 3'd2;
                decoder_cpx_operation_s1 = 5'd7;
                decoder_noc1_operation_s1 = 5'd4;
                decoder_creditman_noc1_needed = 2'd1;
            end
        end

        6'd26:
        begin
            // diag store, is not cache coherent
            decoder_pcx_ack_stage_s1 = 2'd2; // need data until S2
            decoder_dcache_operation_s1 = 4'd9;
            decoder_cpx_operation_s1 = 5'd6;
        end

        6'd27:
        begin
            // need to be done in two stages: ack the store, then the actual flush
            if (fetch_state_s1 == 3'd0)
            begin // ack store
                decoder_stall_on_matched_bypassed_index_s1 = 1;
                decoder_cpx_operation_s1 = 5'd6;
            end
            else
            begin
                decoder_pcx_ack_stage_s1 = 2'd1;

                decoder_dtag_operation_s1 = 2'd1;

                decoder_mesi_read_op_s1 = 1'd1;
                decoder_dcache_operation_s1 = 4'd10;

                decoder_mesi_write_op_s1 = 3'd6;
                decoder_lrsc_flag_write_op_s1 = 3'd4;
                decoder_wmt_read_op_s1 = 1'd1;
                decoder_wmt_write_op_s1 = 3'd5;
                decoder_wmt_compare_op_s1 = 3'd3;
                decoder_lruarray_write_op_s1 = 3'd5;

                decoder_cpx_operation_s1 = 5'd17;
                decoder_noc1_operation_s1 = 5'd12;
                decoder_creditman_noc1_needed = 2'd1;
                decoder_noc3_operation_s1 = 4'd8;
                decoder_csm_op_s1 = 4'd9;
            end
        end

        6'd36: 
        begin
            begin 
            // Store Conditional
                decoder_pcx_ack_stage_s1 = 2'd1;
                decoder_stall_on_matched_bypassed_index_s1 = 1'b1;
                decoder_stall_on_matched_mshr_s1 = 1'b1;
                decoder_stall_on_mshr_allocation_s1 = 1'b1;
                decoder_s1_mshr_operation_s1 = 2'd1;
                decoder_mshr_allocation_type_s1 = 2'd3;
                decoder_dtag_operation_s1 = 2'd1;

                decoder_mesi_read_op_s1 = 1'd1;
                decoder_lrsc_flag_read_op_s1 = 1'd1;

                decoder_s2_mshr_operation_s1 = 1'd1;
                decoder_dcache_operation_s1 = 4'd11;

                decoder_wmt_read_op_s1 = 1'd1;
                decoder_wmt_compare_op_s1 = 3'd2;

                decoder_lruarray_write_op_s1 = 3'd6;
                //decoder_csm_op_s1 = `L15_CSM_OP_READ_GHID_IF_TAGCHECK_SI;
                decoder_s3_mshr_operation_s1 = 3'd1;  // deallocate anyway
                //decoder_mesi_write_op_s1 = `L15_S3_MESI_WRITE_TAGCHECK_WAY_M_IF_LRSC_SET;
                decoder_lrsc_flag_write_op_s1 = 3'd2;
                decoder_cpx_operation_s1 = 5'd20;
                //decoder_noc1_operation_s1 = `L15_NOC1_GEN_DATA_ST_UPGRADE_IF_TAGCHECK_S_ELSE_ST_FILL_IF_TAGCHECK_I;
                //decoder_creditman_noc1_needed = 2'd1;
            end
        end

        6'd4,
        6'd5,
        6'd35,
        6'd38,
        6'd39,
        6'd40,
        6'd41,
        6'd42,
        6'd43,
        6'd44,
        6'd45:
        begin
            if (fetch_state_s1 == 3'd0)
            begin // writeback for CAS/Atomic
                decoder_stall_on_matched_bypassed_index_s1 = 1;
                decoder_stall_on_matched_mshr_s1 = 1;
                decoder_stall_on_mshr_allocation_s1 = 1'b1;
                decoder_mshr_allocation_type_s1 = 2'd2;

                decoder_dtag_operation_s1 = 2'd1;

                decoder_mesi_read_op_s1 = 1'd1;
                decoder_dcache_operation_s1 = 4'd1;

                decoder_mesi_write_op_s1 = 3'd3;
                decoder_lrsc_flag_write_op_s1 = 3'd2;
                // decoder_wmt_operation_s1 = `L15_WMT_READ_TAGCHECK_WAY_IF_MES_AND_DEMAP_ENTRY;
                decoder_wmt_read_op_s1 = 1'd1;
                decoder_wmt_write_op_s1 = 3'd1;
                decoder_wmt_compare_op_s1 = 3'd2;
                decoder_lruarray_write_op_s1 = 3'd4;

                decoder_cpx_operation_s1 = 5'd1;
                decoder_noc1_operation_s1 = 5'd9;
                decoder_creditman_noc1_needed = 2'd2;
                decoder_noc3_operation_s1 = 4'd5;
                decoder_csm_op_s1 = 4'd10;

                // move data request to the first operation to keep atomicity
                if (predecode_reqtype_s1 == 6'd4)
                begin
                    decoder_creditman_req_16B_s1 = 1'b1;
                end
                else if (predecode_reqtype_s1 != 6'd35)
                begin
                    // LR won't sent NOC1 msg with data; other AMO reqs will
                    decoder_creditman_req_8B_s1 = 1'b1;
                end
            end
            else
            begin
                // second packet of CAS
                decoder_pcx_ack_stage_s1 = 2'd3;
                decoder_s1_mshr_operation_s1 = 2'd1;
                decoder_mshr_allocation_type_s1 = 2'd2;

                decoder_csm_op_s1 = 4'd1;
                //if (predecode_reqtype_s1 == `L15_REQTYPE_CAS)
                //begin
                //    decoder_noc1_operation_s1 = `L15_NOC1_GEN_DATA_CAS_REQUEST_FROM_PCX;
                //    // decoder_creditman_req_16B_s1 = 1'b1;
                //end
                //else
                //begin
                //    decoder_noc1_operation_s1 = `L15_NOC1_GEN_DATA_SWAP_REQUEST_FROM_PCX;
                //    // decoder_creditman_req_8B_s1 = 1'b1;
                //end
                case (predecode_reqtype_s1)
                    6'd35:
                    begin
                        decoder_noc1_operation_s1 = 5'd21;
                        decoder_s3_mshr_operation_s1 = 3'd6;
                        // For LR, mshr_update_state can not be IM or SM, 
                        // otherwise the ACKDT_LR will be recognised as a ACKDT_ST_IM/SM. 
                        // So we explicitly set the mshr_update_state to WAIT_ACK
                    end
                    6'd4:
                        decoder_noc1_operation_s1 = 5'd6;
                    6'd5:
                        decoder_noc1_operation_s1 = 5'd7;
                    6'd38:
                        decoder_noc1_operation_s1 = 5'd13;
                    6'd39:
                        decoder_noc1_operation_s1 = 5'd14;
                    6'd40:
                        decoder_noc1_operation_s1 = 5'd15;
                    6'd41:
                        decoder_noc1_operation_s1 = 5'd16;
                    6'd42:
                        decoder_noc1_operation_s1 = 5'd17;
                    6'd43:
                        decoder_noc1_operation_s1 = 5'd18;
                    6'd44:
                        decoder_noc1_operation_s1 = 5'd19;
                    6'd45:
                        decoder_noc1_operation_s1 = 5'd20;
                endcase
                // decoder_creditman_noc1_needed = 2'd1;
            end
        end

        6'd18:
        begin
            // decoder_pcx_ack_stage_s1 = `L15_ACK_STAGE_S1;
            decoder_cpx_operation_s1 = 5'd14;
            if (predecode_is_last_inval_s1)
            begin
                decoder_noc2_ack_stage_s1 = 2'd1;
                decoder_noc3_operation_s1 = 4'd7;
            end
        end

        6'd33:
        begin
            decoder_pcx_ack_stage_s1 = 2'd1;
            decoder_cpx_operation_s1 = 5'd14;
        end

        6'd34:
        begin
            decoder_pcx_ack_stage_s1 = 2'd1;
            decoder_stall_on_matched_bypassed_index_s1 = 1;
            decoder_dtag_operation_s1 = 2'd1;
            decoder_wmt_read_op_s1 = 1'd1;
            decoder_wmt_write_op_s1 = 3'd1;
            // decoder_wmt_compare_op_s1 = `L15_WMT_COMPARE_LRU;



            decoder_wmt_compare_op_s1 = 3'd2; // bug 3/28/16
            

            decoder_cpx_operation_s1 = 5'd19;
        end // todo

        6'd6:
        begin
            if (fetch_state_s1 == 3'd0)
            begin // we only stall on the first inval
                decoder_noc2_ack_stage_s1 = 2'd0;
                decoder_stall_on_matched_bypassed_index_s1 = 1;
            end
            else if (fetch_state_s1 == 3'd4)
            begin // we only ack on the last inval
                decoder_noc2_ack_stage_s1 = 2'd1;
                decoder_stall_on_matched_bypassed_index_s1 = 1;
            end
            else
            begin // we do not stall/ack on the 2nd, 3rd
                decoder_noc2_ack_stage_s1 = 2'd0;
                decoder_stall_on_matched_bypassed_index_s1 = 1;
            end

            decoder_dtag_operation_s1 = 2'd1;

            decoder_mesi_read_op_s1 = 1'd1;
            decoder_dcache_operation_s1 = 4'd1;

            decoder_s3_mshr_operation_s1 = 3'd4;
            decoder_mesi_write_op_s1 = 3'd3;
            decoder_lrsc_flag_write_op_s1 = 3'd2;
            // decoder_wmt_operation_s1 = `L15_WMT_READ_TAGCHECK_WAY_IF_MES_AND_DEMAP_ENTRY;
            decoder_wmt_read_op_s1 = 1'd1;
            decoder_wmt_write_op_s1 = 3'd1;
            decoder_wmt_compare_op_s1 = 3'd2;

            decoder_lruarray_write_op_s1 = 3'd4;

            decoder_cpx_operation_s1 = 5'd1;
            if (predecode_is_last_inval_s1)
                decoder_noc3_operation_s1 = 4'd1;
            else
                decoder_noc3_operation_s1 = 4'd2;

            decoder_csm_op_s1 = 4'd10;
        end

        6'd7:
        begin
            if (fetch_state_s1 == 3'd0)
            begin // we only stall on the first writeback
                decoder_noc2_ack_stage_s1 = 2'd0;
                decoder_stall_on_matched_bypassed_index_s1 = 1;
            end
            else if (fetch_state_s1 == 3'd4)
            begin // we only ack on the last writeback
                decoder_noc2_ack_stage_s1 = 2'd1;
                decoder_stall_on_matched_bypassed_index_s1 = 1;
            end
            else
            begin // we do not stall/ack on the 2nd, 3rd
                decoder_noc2_ack_stage_s1 = 2'd0;
                decoder_stall_on_matched_bypassed_index_s1 = 1;
            end

            decoder_dtag_operation_s1 = 2'd1;

            decoder_mesi_read_op_s1 = 1'd1;
            decoder_dcache_operation_s1 = 4'd1;

            decoder_mesi_write_op_s1 = 3'd1;
            decoder_lrsc_flag_write_op_s1 = 3'd2;

            if (predecode_is_last_inval_s1)
                decoder_noc3_operation_s1 = 4'd3;
            else
                decoder_noc3_operation_s1 = 4'd4;
        end

        6'd10:
        begin
            if (fetch_state_s1 == 3'd0)
            begin // eviction for the fill
                decoder_stall_on_matched_bypassed_index_s1 = 1;
                decoder_dtag_operation_s1 = 2'd1;
                decoder_mesi_read_op_s1 = 1'd1;
                decoder_dcache_operation_s1 = 4'd3;
                decoder_s3_mshr_operation_s1 = 3'd5;
                // decoder_wmt_operation_s1 = `L15_WMT_READ_LRU_WAY_IF_MES_AND_DEMAP_ENTRY;
                decoder_wmt_read_op_s1 = 1'd1;
                decoder_wmt_write_op_s1 = 3'd2;
                decoder_wmt_compare_op_s1 = 3'd1;
                decoder_lruarray_write_op_s1 = 3'd2;
                decoder_lrsc_flag_write_op_s1 = 3'd3;
                decoder_cpx_operation_s1 = 5'd8;
                decoder_noc1_operation_s1 = 5'd8;
                decoder_creditman_noc1_needed = 2'd1;
                decoder_noc3_operation_s1 = 4'd6;
                decoder_creditman_noc1_unreserve_s1 = 1'b1;
                decoder_csm_op_s1 = 4'd8;
            end
            else // the fill
            begin
                decoder_noc2_ack_stage_s1 = 2'd3;
                decoder_dtag_operation_s1 = 2'd2;
                decoder_dcache_operation_s1 = 4'd5;
                decoder_s3_mshr_operation_s1 = 3'd1;
                decoder_mesi_write_op_s1 = 3'd4;
                // decoder_wmt_operation_s1 = `L15_WMT_WRITE_LRU_WAY_L1_REPL_AND_DEMAP_ENTRY;
                decoder_wmt_read_op_s1 = 1'd1;
                decoder_wmt_write_op_s1 = 3'd3;



                // decoder_wmt_compare_op_s1 = `L15_WMT_COMPARE_TAGCHECK; // bug fix 3/28/16                
                decoder_wmt_compare_op_s1 = 3'd1;
                
                decoder_lruarray_write_op_s1 = 3'd3;
                decoder_cpx_operation_s1 = 5'd5;
                
                decoder_hmt_op_s1 = 1'd1;
                
            end
        end

        6'd8:
        begin
            decoder_noc2_ack_stage_s1 = 2'd3;
            decoder_stall_on_matched_bypassed_index_s1 = 1;
            decoder_s3_mshr_operation_s1 = 3'd1;
            decoder_cpx_operation_s1 = 5'd5;
            // decoder_csm_op_s1 = `L15_CSM_OP_EVICT_IF_M; // trin todo: not sure if should be disabled
        end

        6'd9:
        begin
            decoder_noc2_ack_stage_s1 = 2'd3;
            decoder_s3_mshr_operation_s1 = 3'd1;
            decoder_cpx_operation_s1 = 5'd9;
        end

        6'd11:
        begin
            if (fetch_state_s1 == 3'd0)
            begin // eviction for the fill
                decoder_stall_on_matched_bypassed_index_s1 = 1;
                decoder_dtag_operation_s1 = 2'd1;
                decoder_mesi_read_op_s1 = 1'd1;
                decoder_dcache_operation_s1 = 4'd3;
                decoder_s3_mshr_operation_s1 = 3'd5;
                // decoder_wmt_operation_s1 = `L15_WMT_READ_LRU_WAY_IF_MES_AND_DEMAP_ENTRY;
                decoder_wmt_read_op_s1 = 1'd1;
                decoder_wmt_write_op_s1 = 3'd2;
                decoder_wmt_compare_op_s1 = 3'd1;
                decoder_lruarray_write_op_s1 = 3'd2;
                decoder_lrsc_flag_write_op_s1 = 3'd3;
                decoder_cpx_operation_s1 = 5'd8;
                decoder_noc1_operation_s1 = 5'd8;
                decoder_creditman_noc1_needed = 2'd1;
                decoder_noc3_operation_s1 = 4'd6;
                decoder_creditman_noc1_unreserve_s1 = 1'b1;
                decoder_csm_op_s1 = 4'd8;
            end
            else // the fill
            begin
                decoder_noc2_ack_stage_s1 = 2'd3;
                decoder_dtag_operation_s1 = 2'd2;
                decoder_s2_mshr_operation_s1 = 1'd1;
                decoder_dcache_operation_s1 = 4'd6;
                decoder_s3_mshr_operation_s1 = 3'd1;
                decoder_mesi_write_op_s1 = 3'd4;
                decoder_lruarray_write_op_s1 = 3'd3;
                decoder_cpx_operation_s1 = 5'd6;
                
                decoder_hmt_op_s1 = 1'd1;
                
            end
        end

        6'd12:
        begin
            decoder_noc2_ack_stage_s1 = 2'd1;
            decoder_stall_on_matched_bypassed_index_s1 = 1;
            decoder_s2_mshr_operation_s1 = 1'd1;
            decoder_dcache_operation_s1 = 4'd7;
            decoder_s3_mshr_operation_s1 = 3'd1;
            decoder_mesi_write_op_s1 = 3'd5;
            decoder_wmt_read_op_s1 = 1'd1;
            decoder_wmt_compare_op_s1 = 3'd4; // wmt todo // WTF
            decoder_cpx_operation_s1 = 5'd11;
            decoder_creditman_noc1_unreserve_s1 = 1'b1;
        end

        6'd13:
        begin
            decoder_noc2_ack_stage_s1 = 2'd1;
            decoder_stall_on_matched_bypassed_index_s1 = 1;
            decoder_s3_mshr_operation_s1 = 3'd1;
            decoder_cpx_operation_s1 = 5'd6;
        end

        6'd32:
        begin
           decoder_noc2_ack_stage_s1 = 2'd1;
           // decoder_stall_on_matched_bypassed_index_s1 = 1;
           // decoder_s3_mshr_operation_s1 = `L15_S3_MSHR_OP_DEALLOCATION;
           // decoder_cpx_operation_s1 = `L15_CPX_GEN_LD_RESPONSE_BOGUS_DATA;
        end

        6'd14:
        begin
            decoder_noc2_ack_stage_s1 = 2'd3;
            decoder_stall_on_matched_bypassed_index_s1 = 1;
            decoder_s3_mshr_operation_s1 = 3'd1;
            decoder_cpx_operation_s1 = 5'd10;
        end

        6'd46:
        begin
            if (fetch_state_s1 == 3'd0)
            begin // eviction for the fill
                decoder_stall_on_matched_bypassed_index_s1 = 1;
                decoder_dtag_operation_s1 = 2'd1;
                decoder_mesi_read_op_s1 = 1'd1;
                decoder_dcache_operation_s1 = 4'd3;
                decoder_s3_mshr_operation_s1 = 3'd5;
                // decoder_wmt_operation_s1 = `L15_WMT_READ_LRU_WAY_IF_MES_AND_DEMAP_ENTRY;
                decoder_wmt_read_op_s1 = 1'd1;
                decoder_wmt_write_op_s1 = 3'd2;
                decoder_wmt_compare_op_s1 = 3'd1;
                decoder_lruarray_write_op_s1 = 3'd2;
                decoder_cpx_operation_s1 = 5'd8;
                decoder_noc1_operation_s1 = 5'd8;
                decoder_creditman_noc1_needed = 2'd1;
                decoder_noc3_operation_s1 = 4'd6;
                decoder_creditman_noc1_unreserve_s1 = 1'b1;
                decoder_csm_op_s1 = 4'd8;
            end
            else // the fill
            begin
                decoder_noc2_ack_stage_s1 = 2'd3;
                decoder_dtag_operation_s1 = 2'd2;
                decoder_dcache_operation_s1 = 4'd5;
                decoder_s3_mshr_operation_s1 = 3'd1;
                decoder_mesi_write_op_s1 = 3'd4; // do this for unity, it can be set to M directly.
                decoder_lrsc_flag_write_op_s1 = 3'd1;
                // decoder_wmt_operation_s1 = `L15_WMT_WRITE_LRU_WAY_L1_REPL_AND_DEMAP_ENTRY;
                decoder_wmt_read_op_s1 = 1'd1;
                // L1 won't save the line after an LR, so L15 should not update the wmt
                //decoder_wmt_write_op_s1 = `L15_WMT_UPDATE_LRU_WAY_AND_DEDUP_ENTRY;



                // decoder_wmt_compare_op_s1 = `L15_WMT_COMPARE_TAGCHECK; // bug fix 3/28/16
                decoder_wmt_compare_op_s1 = 3'd1;

                decoder_lruarray_write_op_s1 = 3'd3;
                decoder_cpx_operation_s1 = 5'd10;
                
                decoder_hmt_op_s1 = 1'd1;
                
            end
        end

        6'd20:
        begin
            decoder_noc2_ack_stage_s1 = 2'd3;
            decoder_cpx_operation_s1 = 5'd13;
        end

        6'd19:
        begin
            if (predecode_interrupt_broadcast_s1)
            begin
                decoder_pcx_ack_stage_s1 = 2'd1;
                decoder_cpx_operation_s1 = 5'd12;
            end
            else
            begin
                decoder_pcx_ack_stage_s1 = 2'd3;
                decoder_noc1_operation_s1 = 5'd10;
                decoder_creditman_noc1_needed = 2'd1;
                decoder_creditman_req_8B_s1 = 1'b1;
            end
        end

        6'd24:
        begin
            decoder_pcx_ack_stage_s1 = 2'd1;
            decoder_cpx_operation_s1 = 5'd15;
            decoder_config_op_s1 = 2'd1;
        end

        6'd23:
        begin
            decoder_pcx_ack_stage_s1 = 2'd2;
            decoder_cpx_operation_s1 = 5'd6;
            decoder_config_op_s1 = 2'd2;
        end

        6'd28:
        begin
            decoder_noc2_ack_stage_s1 = 2'd2;
            decoder_csm_op_s1 = 4'd3;
            // decoder_cpx_operation_s1 = `L15_CPX_GEN_ST_ACK;
        end

        6'd29:
        begin
            decoder_pcx_ack_stage_s1 = 2'd1;
            decoder_csm_op_s1 = 4'd5;
            decoder_cpx_operation_s1 = 5'd18;
        end

        6'd30:
        begin
            decoder_pcx_ack_stage_s1 = 2'd2;
            decoder_csm_op_s1 = 4'd4;
            decoder_cpx_operation_s1 = 5'd6;
        end

        6'd31:
        begin
            decoder_pcx_ack_stage_s1 = 2'd1;
            decoder_csm_op_s1 = 4'd7;
            decoder_cpx_operation_s1 = 5'd6;
        end

        6'd1:
        begin // skip this request
            decoder_pcx_ack_stage_s1 = 2'd1;
        end
    endcase

    // some stuff we can calculate right after the decoder stage
    decoder_no_free_mshr_s1 = mshr_val_array[predecode_threadid_s1][decoder_mshr_allocation_type_s1];
    decoder_mshrid_s1 =
        (predecode_source_s1 == 2'd2) ? noc2decoder_l15_mshrid : decoder_mshr_allocation_type_s1;
end

///////////////////////////////////
// STALL LOGIC FOR S1
///////////////////////////////////
// depends on creditman, predecoder, and decoder
// output dependencies: should be none (except for flops)

reg stall_tag_match_stall_s1;
reg stall_index_bypass_match_s1;
reg stall_index_conflict_stall_s1;
reg stall_mshr_allocation_busy_s1;

reg stall_noc1_data_buffer_unavail_s1;
reg stall_noc1_command_buffer_1_unavail_s1;
reg stall_noc1_command_buffer_2_unavail_s1;
reg stall_noc1_command_buffer_unavail_s1;
reg stall_pcx_noc1_buffer_s1;

reg [4:0] stall_tmp_operand1;
reg [4:0] stall_tmp_operand2;
reg [4:0] stall_tmp_result;


always @ *
begin
    // CALCULATE NOC1 DATA BUFFER AVAILABILITY
    stall_noc1_data_buffer_unavail_s1 = decoder_creditman_req_8B_s1  ? creditman_noc1_data_avail == 4'd0 :
                                                    decoder_creditman_req_16B_s1 ? creditman_noc1_data_avail < 4'd2 :
                                                                                            1'b0;
    // CALCULATE NOC1 COMMAND BUFFER AVAILABILITY
    // note: the complicated sequence of calculation below is to determine
    //  creditman_noc1_avail <= creditman_noc1_reserve
    //  a simple expression (creditman_noc1_avail <= creditman_noc1_reserve) might not have the most efficient implementation
    stall_tmp_operand1 = {1'b1, creditman_noc1_avail[3:0]};
    stall_tmp_operand2 = {1'b0, creditman_noc1_reserve[3:0]};
    stall_tmp_result = stall_tmp_operand1 - stall_tmp_operand2;

    stall_noc1_command_buffer_1_unavail_s1 = (stall_tmp_result[3:0] == 4'b0) || stall_tmp_result[4] == 1'b0;
    stall_noc1_command_buffer_2_unavail_s1 = stall_noc1_command_buffer_1_unavail_s1 || (stall_tmp_result[3:0] == 4'b1);

    stall_noc1_command_buffer_unavail_s1 = (decoder_creditman_noc1_needed == 2'd1 && stall_noc1_command_buffer_1_unavail_s1)
                                        || (decoder_creditman_noc1_needed == 2'd2 && stall_noc1_command_buffer_2_unavail_s1);
    // only stall if it's an PCX request (noc2 does not stall)
    stall_pcx_noc1_buffer_s1 = (stall_noc1_command_buffer_unavail_s1 || stall_noc1_data_buffer_unavail_s1)
                                            && (predecode_source_s1 == 2'd1)
                                            && (fetch_state_s1 == 3'd0);

    // stall on tag match mshr
    stall_tag_match_stall_s1 = predecode_tagcheck_matched_s1 && decoder_stall_on_matched_mshr_s1;

    // stall on index conflict
    stall_index_bypass_match_s1 = (val_s2 && (predecode_cache_index_s1 == cache_index_s2))
                                     || (val_s3 && (predecode_cache_index_s1 == cache_index_s3));
    stall_index_conflict_stall_s1 = decoder_stall_on_matched_bypassed_index_s1 && stall_index_bypass_match_s1;
    stall_mshr_allocation_busy_s1 = decoder_no_free_mshr_s1 && decoder_stall_on_mshr_allocation_s1;

    // aggregating all the stalls
    stall_s1 = val_s1 && (stall_tag_match_stall_s1 || stall_index_conflict_stall_s1 || stall_s2 || stall_mshr_allocation_busy_s1
                    || stall_pcx_noc1_buffer_s1);
end

////////////////////////////
// DTAG logics
////////////////////////////
// is SRAM
reg dtag_val_s1;
reg dtag_rw_s1;
reg [((9-2))-1:0] dtag_index_s1;
reg [2-1:0] dtag_write_way_s1;
reg [33-1:0] dtag_write_tag_s1;
reg [3:0] dtag_write_way_mask;

always @ *
begin
    dtag_val_s1 = 0;
    dtag_rw_s1 = 0;
    dtag_index_s1 = 0;
    dtag_write_way_s1 = 0;
    dtag_write_tag_s1[33-1:0]  = 0;
    dtag_write_way_mask = 0;
    case (decoder_dtag_operation_s1)
        2'd1:
        begin
            dtag_val_s1 = val_s1;
            dtag_rw_s1 = 1'b1;
            dtag_index_s1 = predecode_cache_index_s1;
        end
        2'd2:
        begin
            dtag_val_s1 = val_s1;
            dtag_rw_s1 = 1'b0;
            dtag_index_s1 = predecode_cache_index_s1;
            dtag_write_way_s1 = lru_way_s2; // previous instruction (eviction) is guaranteed to be in s2
            // pad the address to the raw tag space required
            dtag_write_tag_s1[33-1:0]  = {{33-(40 - 4 - ((9-2))){1'b0}},predecode_dtag_write_data_s1[(40 - 4 - ((9-2)))-1:0]} ;
        end
    endcase

    dtag_write_way_mask = (dtag_write_way_s1 == 2'b00) ? 4'b0_0_0_1 :
                            (dtag_write_way_s1 == 2'b01) ? 4'b0_0_1_0 :
                            (dtag_write_way_s1 == 2'b10) ? 4'b0_1_0_0 :
                                                        4'b1_0_0_0 ;

    // INPUT/OUTPUT TO DTAG
    l15_dtag_val_s1 = dtag_val_s1 && !stall_s1;
    l15_dtag_rw_s1 = dtag_rw_s1;
    l15_dtag_index_s1[((9-2))-1:0] = dtag_index_s1;
    l15_dtag_write_data_s1[33*4-1:0] = {4{dtag_write_tag_s1[33-1:0]}} ;
    l15_dtag_write_mask_s1[33*4-1:0] =
                                                                    {{33{dtag_write_way_mask[3]}},
                                                                     {33{dtag_write_way_mask[2]}},
                                                                     {33{dtag_write_way_mask[1]}},
                                                                     {33{dtag_write_way_mask[0]}}};
end

////////////////////////////
// MESI read control logics
////////////////////////////
// is SRAM
reg mesi_read_val_s1;
reg [((9-2))-1:0] mesi_read_index_s1;
always @ *
begin
    mesi_read_val_s1 = 0;
    mesi_read_index_s1 = 0;
    case (decoder_mesi_read_op_s1)
        1'd1:
        begin
            mesi_read_val_s1 = 1'b1;
            mesi_read_index_s1 = predecode_cache_index_s1;
        end
    endcase

    l15_mesi_read_val_s1 = mesi_read_val_s1 && val_s1 && !stall_s1;
    l15_mesi_read_index_s1[((9-2))-1:0] = mesi_read_index_s1;
end

////////////////////////////
// LRSC FLAG read control logics
////////////////////////////
reg lrsc_flag_read_val_s1;
reg [((9-2))-1:0] lrsc_flag_read_index_s1;
always @ *
begin
    lrsc_flag_read_val_s1 = 0;
    lrsc_flag_read_index_s1 = 0;
    case (decoder_lrsc_flag_read_op_s1)
        1'd1:
        begin
            lrsc_flag_read_val_s1 = 1'b1;
            lrsc_flag_read_index_s1 = predecode_cache_index_s1;
        end
    endcase

    l15_lrsc_flag_read_val_s1 = lrsc_flag_read_val_s1 && val_s1 && !stall_s1;
    l15_lrsc_flag_read_index_s1[((9-2))-1:0] = lrsc_flag_read_index_s1;
end

////////////////////////////
// MSHR S1 logic
////////////////////////////
// depends on predecode and decode

reg [((((((((((0 + 1) + 1) + 1) + 1) + 3) + 1) + 2) + 1) + 1) + 1)-1:0] mshr_control_bits_write_s1;
// reg mshr_next_available_mshrid_valid_s1;
always @ *
begin
    // mux betweeen new mshrid and noc2 returned mshrid

    // write
    mshr_control_bits_write_s1 = 0;
    mshr_control_bits_write_s1 [(((((0 + 1) + 1) + 1) + 1) + 3) -: 3] = predecode_size_s1;
    mshr_control_bits_write_s1 [((((((0 + 1) + 1) + 1) + 1) + 3) + 1) -: 1] = predecode_threadid_s1;
    mshr_control_bits_write_s1 [(((((((0 + 1) + 1) + 1) + 1) + 3) + 1) + 2) -: 2] = predecode_l1_replacement_way_s1;
    // for Load Reserve, we evict and send INVs like a NC req. But when we get the data ack, we reagrd it as a Cacheable req
    mshr_control_bits_write_s1 [((((((((0 + 1) + 1) + 1) + 1) + 3) + 1) + 2) + 1) -: 1] = (predecode_reqtype_s1 == 6'd35) ? 1'b0 : predecode_non_cacheable_s1;
    mshr_control_bits_write_s1 [(((((((((0 + 1) + 1) + 1) + 1) + 3) + 1) + 2) + 1) + 1) -: 1] = predecode_blockstore_bit_s1;
    mshr_control_bits_write_s1 [(0 + 1) -: 1] = predecode_blockstore_init_s1;
    mshr_control_bits_write_s1 [0 -: 1] = predecode_prefetch_bit_s1;
    // mshr_control_bits_write_s1 [`L15_CONTROL_INVALIDATE_INDEX_1B -: 1] = predecode_invalidate_index_s1;
    mshr_control_bits_write_s1 [((((0 + 1) + 1) + 1) + 1) -: 1] = predecode_icache_bit_s1;
    mshr_control_bits_write_s1 [((0 + 1) + 1) -: 1] = predecode_dcache_load_s1;
    mshr_control_bits_write_s1 [(((0 + 1) + 1) + 1) -: 1] = predecode_atomic_s1;
end

////////////////////////////
// MSHR allocation logic
////////////////////////////
// is FLOPs
reg s1_mshr_write_val_s1;
reg [3-1:0] s1_mshr_write_type_s1;
reg [39:0] s1_mshr_write_address_s1;
reg [((((((((((0 + 1) + 1) + 1) + 1) + 3) + 1) + 2) + 1) + 1) + 1)-1:0] s1_mshr_write_control_s1;
reg [2-1:0] s1_mshr_write_mshrid_s1;
reg [0:0] s1_mshr_write_threadid_s1;
reg [15:0] unshifted_write_mask_s1;
reg [15:0] write_mask_s1;
reg [15:0] write_mask_1B_s1;
reg [15:0] write_mask_2B_s1;
reg [15:0] write_mask_4B_s1;
reg [15:0] write_mask_8B_s1;
reg [15:0] write_mask_16B_s1;
// reg odd_extended_word_s1;
always @ *
begin
    s1_mshr_write_val_s1 = 0;
    s1_mshr_write_type_s1 = 0;
    s1_mshr_write_address_s1 = 0;
    s1_mshr_write_control_s1 = 0;
    s1_mshr_write_mshrid_s1 = 0;
    // s1_mshr_write_data = 0;
    // s1_mshr_write_byte_mask = 0;
    write_mask_s1 = 0;
    s1_mshr_write_threadid_s1 = 0;

    case (decoder_s1_mshr_operation_s1)
        2'd1:
        begin
            s1_mshr_write_val_s1 = 1'b1; // make request, then see if mshr is busy
            s1_mshr_write_type_s1 = 3'b001;
            s1_mshr_write_address_s1 = pcxdecoder_l15_address;
            s1_mshr_write_control_s1 = mshr_control_bits_write_s1;
            s1_mshr_write_mshrid_s1 = decoder_mshr_allocation_type_s1;
            s1_mshr_write_threadid_s1[0:0] = predecode_threadid_s1[0:0];
        end
        2'd2:
        begin
            s1_mshr_write_val_s1 = 1'b1;
            s1_mshr_write_type_s1 = 3'b100;
            // s1_mshr_write_mshrid_s1 = predecode_hit_stbuf_mshrid_s1;
            s1_mshr_write_threadid_s1 = predecode_hit_stbuf_threadid_s1;
        end
    endcase

    unshifted_write_mask_s1 =   (predecode_size_s1 == 3'b000) ? 16'b1000_0000_0000_0000 :
                                (predecode_size_s1 == 3'b001) ? 16'b1100_0000_0000_0000 :
                                (predecode_size_s1 == 3'b010) ? 16'b1111_0000_0000_0000 :
                                                                    16'b1111_1111_0000_0000 ;

    write_mask_1B_s1 = unshifted_write_mask_s1 >> (pcxdecoder_l15_address & 4'b1111);
    write_mask_2B_s1 = unshifted_write_mask_s1 >> (pcxdecoder_l15_address & 4'b1110);
    write_mask_4B_s1 = unshifted_write_mask_s1 >> (pcxdecoder_l15_address & 4'b1100);
    write_mask_8B_s1 = unshifted_write_mask_s1 >> (pcxdecoder_l15_address & 4'b1000);


    write_mask_16B_s1 = {16{1'b1}};

    case(predecode_size_s1)
        3'b000:
        begin
            write_mask_s1 = write_mask_1B_s1;
        end
        3'b001:
        begin
            write_mask_s1 = write_mask_2B_s1;
        end
        3'b010:
        begin
            write_mask_s1 = write_mask_4B_s1;
        end
        3'b011:
        begin
            write_mask_s1 = write_mask_8B_s1;
        end
        3'b111:
        begin
            write_mask_s1 = write_mask_16B_s1;
        end
    endcase


    // s1 write
    pipe_mshr_writereq_val_s1 = s1_mshr_write_val_s1 && !stall_s1 && val_s1;
    pipe_mshr_writereq_op_s1[3-1:0] = s1_mshr_write_type_s1[3-1:0];
    pipe_mshr_writereq_address_s1[39:0] = s1_mshr_write_address_s1[39:0];
    pipe_mshr_writereq_control_s1[((((((((((0 + 1) + 1) + 1) + 1) + 3) + 1) + 2) + 1) + 1) + 1)-1:0] = s1_mshr_write_control_s1[((((((((((0 + 1) + 1) + 1) + 1) + 3) + 1) + 2) + 1) + 1) + 1)-1:0];
    pipe_mshr_writereq_write_buffer_data_s1[127:0] = {pcxdecoder_l15_data, pcxdecoder_l15_data};
    pipe_mshr_writereq_write_buffer_byte_mask_s1[15:0] = write_mask_s1[15:0];
    pipe_mshr_writereq_mshrid_s1[2-1:0] = s1_mshr_write_mshrid_s1[2-1:0];
    pipe_mshr_writereq_threadid_s1 = s1_mshr_write_threadid_s1;
end

//////////////////////
// LRU read logic
//////////////////////
always @ *
begin
    l15_lruarray_read_val_s1 = val_s1 && !stall_s1;
    l15_lruarray_read_index_s1 = predecode_cache_index_s1;
end


/*********************************/
/**** PCX & NOC2 S1 ACK LOGIC ****/
/*********************************/
reg acklogic_pcx_s1;
reg acklogic_noc2_s1;
always @ *
begin
    acklogic_pcx_s1 = 0;
    acklogic_noc2_s1 = 0;

    if (decoder_pcx_ack_stage_s1 == 2'd1)
        acklogic_pcx_s1 = 1;
    if (decoder_noc2_ack_stage_s1 == 2'd1)
        acklogic_noc2_s1 = 1;

    // ack logics have to be guarded by stalls
    pcx_ack_s1 = val_s1 && !stall_s1 && acklogic_pcx_s1;
    noc2_ack_s1 = val_s1 && !stall_s1 && acklogic_noc2_s1;

    // ack header of pcx and noc2 at s1
    // note: the message data does not have to be acked at s1
    l15_pcxdecoder_header_ack = (predecode_source_s1 == 2'd1) && !stall_s1 && val_s1
                                                && (fetch_state_s1 == 3'd0);

    // COV: case 1 1 0 1 is impossible
    l15_noc2decoder_header_ack = (predecode_source_s1 == 2'd2) && !stall_s1 && val_s1
                                                && (fetch_state_s1 == 3'd0);
end


/***********
 * STAGE 2 *
 ***********/

// propagated variables (flops) from S2






reg val_s2_next;

reg [1-1:0] threadid_s2;
reg [1-1:0] threadid_s2_next;
reg [2-1:0] mshrid_s2;
reg [2-1:0] mshrid_s2_next;
reg [40-1:0] address_s2;
reg [40-1:0] address_s2_next;
reg [1-1:0] non_cacheable_s2;
reg [1-1:0] non_cacheable_s2_next;
reg [3-1:0] size_s2;
reg [3-1:0] size_s2_next;
reg [1-1:0] prefetch_s2;
reg [1-1:0] prefetch_s2_next;
reg [2-1:0] l1_replacement_way_s2;
reg [2-1:0] l1_replacement_way_s2_next;
reg [1-1:0] l2_miss_s2;
reg [1-1:0] l2_miss_s2_next;
reg [1-1:0] f4b_s2;
reg [1-1:0] f4b_s2_next;
reg [1-1:0] predecode_noc2_inval_s2;
reg [1-1:0] predecode_noc2_inval_s2_next;
reg [4-1:0] predecode_fwd_subcacheline_vector_s2;
reg [4-1:0] predecode_fwd_subcacheline_vector_s2_next;
reg [3-1:0] lrsc_flag_write_op_s2;
reg [3-1:0] lrsc_flag_write_op_s2_next;
reg [1-1:0] blockstore_s2;
reg [1-1:0] blockstore_s2_next;
reg [1-1:0] blockstoreinit_s2;
reg [1-1:0] blockstoreinit_s2_next;
reg [6-1:0] predecode_reqtype_s2;
reg [6-1:0] predecode_reqtype_s2_next;
reg [2-1:0] decoder_dtag_operation_s2;
reg [2-1:0] decoder_dtag_operation_s2_next;
reg [3-1:0] wmt_write_op_s2;
reg [3-1:0] wmt_write_op_s2_next;
reg [3-1:0] wmt_compare_op_s2;
reg [3-1:0] wmt_compare_op_s2_next;
reg [3-1:0] lruarray_write_op_s2;
reg [3-1:0] lruarray_write_op_s2_next;
reg [4-1:0] csm_op_s2;
reg [4-1:0] csm_op_s2_next;
reg [2-1:0] config_op_s2;
reg [2-1:0] config_op_s2_next;
reg [1-1:0] wmt_read_op_s2;
reg [1-1:0] wmt_read_op_s2_next;
reg [(14+8+8)-1:0] noc2_src_homeid_s2;
reg [(14+8+8)-1:0] noc2_src_homeid_s2_next;
reg [(14+8+8)-1:0] hmt_fill_homeid_s2;
reg [(14+8+8)-1:0] hmt_fill_homeid_s2_next;
reg [3-1:0] s3_mshr_operation_s2;
reg [3-1:0] s3_mshr_operation_s2_next;
reg [5-1:0] cpx_operation_s2;
reg [5-1:0] cpx_operation_s2_next;
reg [5-1:0] noc1_operation_s2;
reg [5-1:0] noc1_operation_s2_next;
reg [4-1:0] noc3_operations_s2;
reg [4-1:0] noc3_operations_s2_next;
reg [1-1:0] mesi_read_op_s2;
reg [1-1:0] mesi_read_op_s2_next;
reg [3-1:0] mesi_write_op_s2;
reg [3-1:0] mesi_write_op_s2_next;
reg [4-1:0] dcache_operation_s2;
reg [4-1:0] dcache_operation_s2_next;
reg [1-1:0] s2_mshr_operation_s2;
reg [1-1:0] s2_mshr_operation_s2_next;
reg [2-1:0] pcx_ack_stage_s2;
reg [2-1:0] pcx_ack_stage_s2_next;
reg [2-1:0] noc2_ack_stage_s2;
reg [2-1:0] noc2_ack_stage_s2_next;
reg [2-1:0] noc2_ack_state_s2;
reg [2-1:0] noc2_ack_state_s2_next;
reg [33-1:0] csm_pcx_data_s2;
reg [33-1:0] csm_pcx_data_s2_next;
reg [33-1:0] hmt_op_s2;
reg [33-1:0] hmt_op_s2_next;


always @ (posedge clk)
begin
    if (!rst_n)
    begin
        val_s2 <= 1'b0;
        threadid_s2 <= 0;
mshrid_s2 <= 0;
address_s2 <= 0;
non_cacheable_s2 <= 0;
size_s2 <= 0;
prefetch_s2 <= 0;
l1_replacement_way_s2 <= 0;
l2_miss_s2 <= 0;
f4b_s2 <= 0;
predecode_noc2_inval_s2 <= 0;
predecode_fwd_subcacheline_vector_s2 <= 0;
lrsc_flag_write_op_s2 <= 0;
blockstore_s2 <= 0;
blockstoreinit_s2 <= 0;
predecode_reqtype_s2 <= 0;
decoder_dtag_operation_s2 <= 0;
wmt_write_op_s2 <= 0;
wmt_compare_op_s2 <= 0;
lruarray_write_op_s2 <= 0;
csm_op_s2 <= 0;
config_op_s2 <= 0;
wmt_read_op_s2 <= 0;
noc2_src_homeid_s2 <= 0;
hmt_fill_homeid_s2 <= 0;
s3_mshr_operation_s2 <= 0;
cpx_operation_s2 <= 0;
noc1_operation_s2 <= 0;
noc3_operations_s2 <= 0;
mesi_read_op_s2 <= 0;
mesi_write_op_s2 <= 0;
dcache_operation_s2 <= 0;
s2_mshr_operation_s2 <= 0;
pcx_ack_stage_s2 <= 0;
noc2_ack_stage_s2 <= 0;
noc2_ack_state_s2 <= 0;
csm_pcx_data_s2 <= 0;
hmt_op_s2 <= 0;

    end
    else
    begin
        val_s2 <= val_s2_next;
        threadid_s2 <= threadid_s2_next;
mshrid_s2 <= mshrid_s2_next;
address_s2 <= address_s2_next;
non_cacheable_s2 <= non_cacheable_s2_next;
size_s2 <= size_s2_next;
prefetch_s2 <= prefetch_s2_next;
l1_replacement_way_s2 <= l1_replacement_way_s2_next;
l2_miss_s2 <= l2_miss_s2_next;
f4b_s2 <= f4b_s2_next;
predecode_noc2_inval_s2 <= predecode_noc2_inval_s2_next;
predecode_fwd_subcacheline_vector_s2 <= predecode_fwd_subcacheline_vector_s2_next;
lrsc_flag_write_op_s2 <= lrsc_flag_write_op_s2_next;
blockstore_s2 <= blockstore_s2_next;
blockstoreinit_s2 <= blockstoreinit_s2_next;
predecode_reqtype_s2 <= predecode_reqtype_s2_next;
decoder_dtag_operation_s2 <= decoder_dtag_operation_s2_next;
wmt_write_op_s2 <= wmt_write_op_s2_next;
wmt_compare_op_s2 <= wmt_compare_op_s2_next;
lruarray_write_op_s2 <= lruarray_write_op_s2_next;
csm_op_s2 <= csm_op_s2_next;
config_op_s2 <= config_op_s2_next;
wmt_read_op_s2 <= wmt_read_op_s2_next;
noc2_src_homeid_s2 <= noc2_src_homeid_s2_next;
hmt_fill_homeid_s2 <= hmt_fill_homeid_s2_next;
s3_mshr_operation_s2 <= s3_mshr_operation_s2_next;
cpx_operation_s2 <= cpx_operation_s2_next;
noc1_operation_s2 <= noc1_operation_s2_next;
noc3_operations_s2 <= noc3_operations_s2_next;
mesi_read_op_s2 <= mesi_read_op_s2_next;
mesi_write_op_s2 <= mesi_write_op_s2_next;
dcache_operation_s2 <= dcache_operation_s2_next;
s2_mshr_operation_s2 <= s2_mshr_operation_s2_next;
pcx_ack_stage_s2 <= pcx_ack_stage_s2_next;
noc2_ack_stage_s2 <= noc2_ack_stage_s2_next;
noc2_ack_state_s2 <= noc2_ack_state_s2_next;
csm_pcx_data_s2 <= csm_pcx_data_s2_next;
hmt_op_s2 <= hmt_op_s2_next;

    end
end

reg [1:0] way_mshr_st_s2;
reg [(40 - 4 - ((9-2)))-1:0] address_cache_tag_s2;
always @ *
begin
    cache_index_s2 = address_s2[(((9-2))+4-1):4];
    cache_index_l1d_s2 = address_s2[(6 + 4):4];
    address_cache_tag_s2 = address_s2[(39):((((9-2))+4-1) + 1)];

    way_mshr_st_s2 = mshr_st_way_array[threadid_s2];

    // next signals
    if (stall_s2)
    begin
        val_s2_next = val_s2;
        
        threadid_s2_next = threadid_s2;
mshrid_s2_next = mshrid_s2;
address_s2_next = address_s2;
non_cacheable_s2_next = non_cacheable_s2;
size_s2_next = size_s2;
prefetch_s2_next = prefetch_s2;
l1_replacement_way_s2_next = l1_replacement_way_s2;
l2_miss_s2_next = l2_miss_s2;
f4b_s2_next = f4b_s2;
predecode_noc2_inval_s2_next = predecode_noc2_inval_s2;
predecode_fwd_subcacheline_vector_s2_next = predecode_fwd_subcacheline_vector_s2;
lrsc_flag_write_op_s2_next = lrsc_flag_write_op_s2;
blockstore_s2_next = blockstore_s2;
blockstoreinit_s2_next = blockstoreinit_s2;
predecode_reqtype_s2_next = predecode_reqtype_s2;
decoder_dtag_operation_s2_next = decoder_dtag_operation_s2;
wmt_write_op_s2_next = wmt_write_op_s2;
wmt_compare_op_s2_next = wmt_compare_op_s2;
lruarray_write_op_s2_next = lruarray_write_op_s2;
csm_op_s2_next = csm_op_s2;
config_op_s2_next = config_op_s2;
wmt_read_op_s2_next = wmt_read_op_s2;
noc2_src_homeid_s2_next = noc2_src_homeid_s2;
hmt_fill_homeid_s2_next = hmt_fill_homeid_s2;
s3_mshr_operation_s2_next = s3_mshr_operation_s2;
cpx_operation_s2_next = cpx_operation_s2;
noc1_operation_s2_next = noc1_operation_s2;
noc3_operations_s2_next = noc3_operations_s2;
mesi_read_op_s2_next = mesi_read_op_s2;
mesi_write_op_s2_next = mesi_write_op_s2;
dcache_operation_s2_next = dcache_operation_s2;
s2_mshr_operation_s2_next = s2_mshr_operation_s2;
pcx_ack_stage_s2_next = pcx_ack_stage_s2;
noc2_ack_stage_s2_next = noc2_ack_stage_s2;
noc2_ack_state_s2_next = noc2_ack_state_s2;
csm_pcx_data_s2_next = csm_pcx_data_s2;
hmt_op_s2_next = hmt_op_s2;

        





































    end
    else
    begin
        val_s2_next = val_s1 && !stall_s1;
        threadid_s2_next = predecode_threadid_s1;
        mshrid_s2_next = decoder_mshrid_s1;
        address_s2_next = predecode_address_s1;
        non_cacheable_s2_next = predecode_non_cacheable_s1;
        size_s2_next = predecode_size_s1;
        prefetch_s2_next = predecode_prefetch_bit_s1;
        l1_replacement_way_s2_next = predecode_l1_replacement_way_s1;
        l2_miss_s2_next = predecode_l2_miss_s1;
        f4b_s2_next = predecode_f4b_s1;
        // atomic_s2_next = predecode_atomic_s1;
        wmt_write_op_s2_next = decoder_wmt_write_op_s1;
        wmt_compare_op_s2_next = decoder_wmt_compare_op_s1;
        lruarray_write_op_s2_next = decoder_lruarray_write_op_s1;
        csm_op_s2_next = decoder_csm_op_s1;
        config_op_s2_next = decoder_config_op_s1;
        s3_mshr_operation_s2_next = decoder_s3_mshr_operation_s1;
        cpx_operation_s2_next = decoder_cpx_operation_s1;
        noc1_operation_s2_next = decoder_noc1_operation_s1;
        noc3_operations_s2_next = decoder_noc3_operation_s1;
        mesi_read_op_s2_next = decoder_mesi_read_op_s1;
        mesi_write_op_s2_next = decoder_mesi_write_op_s1;
        dcache_operation_s2_next = decoder_dcache_operation_s1;
        s2_mshr_operation_s2_next = decoder_s2_mshr_operation_s1;
        pcx_ack_stage_s2_next = decoder_pcx_ack_stage_s1;
        noc2_ack_stage_s2_next = decoder_noc2_ack_stage_s1;
        noc2_ack_state_s2_next = noc2decoder_l15_ack_state;
        predecode_reqtype_s2_next = predecode_reqtype_s1;
        predecode_fwd_subcacheline_vector_s2_next = predecode_fwd_subcacheline_vector_s1;
        predecode_noc2_inval_s2_next = predecode_noc2_inval_s1;
        blockstore_s2_next = predecode_blockstore_bit_s1;
        blockstoreinit_s2_next = predecode_blockstore_init_s1;
        noc2_src_homeid_s2_next = noc2decoder_l15_src_homeid;
        lrsc_flag_write_op_s2_next = decoder_lrsc_flag_write_op_s1;

        
        hmt_fill_homeid_s2_next = predecode_mshr_read_homeid_s1;
        
        csm_pcx_data_s2_next = pcxdecoder_l15_csm_data;
        decoder_dtag_operation_s2_next = decoder_dtag_operation_s1;
        wmt_read_op_s2_next = decoder_wmt_read_op_s1;
        
        hmt_op_s2_next = decoder_hmt_op_s1;
        
    end
end


always @ *
begin
    // Stalling logics
    // The only reason that S2 can stall is that S3 is stalled
    stall_s2 = val_s2 && stall_s3;
end


always @ *
begin
    // PCX/Noc2 ack logics
    // COV: doesn't seem like anything acks in stage 2
    pcx_ack_s2 = val_s2 && !stall_s2 && (pcx_ack_stage_s2 == 2'd2);
    noc2_ack_s2 = val_s2 && !stall_s2 && (noc2_ack_stage_s2 == 2'd2);
end


// tag check logics
reg [33-1:0] dtag_tag_way0_s2;
reg [33-1:0] dtag_tag_way1_s2;
reg [33-1:0] dtag_tag_way2_s2;
reg [33-1:0] dtag_tag_way3_s2;
reg [2-1:0] mesi_state_way0_s2;
reg [2-1:0] mesi_state_way1_s2;
reg [2-1:0] mesi_state_way2_s2;
reg [2-1:0] mesi_state_way3_s2;
reg [7:0] mesi_read_data_s2;
reg tagcheck_way0_equals;
reg tagcheck_way1_equals;
reg tagcheck_way2_equals;
reg tagcheck_way3_equals;

reg [1:0] tagcheck_state_s2;
reg tagcheck_state_me_s2;
reg tagcheck_state_mes_s2;
reg tagcheck_state_s_s2;
reg tagcheck_state_m_s2;
reg tagcheck_state_e_s2;
reg [1:0] tagcheck_way_s2;
// reg [`L15_UNPARAM_3_0] tagcheck_way_mask_s2;
reg tagcheck_val_s2;
reg tagcheck_lrsc_flag_s2;

reg [1:0] lru_state_s2;
reg lru_state_m_s2;
reg lru_state_mes_s2;
reg [(40 - 4 - ((9-2)))-1:0] lru_way_tag_s2;
reg [39:0] lru_way_address_s2;

reg [1:0] flush_state_s2;
reg flush_state_m_s2;
reg flush_state_me_s2;
reg flush_state_mes_s2;
reg [1:0] flush_way_s2;
// reg [`L15_UNPARAM_3_0] flush_way_mask_s2;
reg [(40 - 4 - ((9-2)))-1:0] flush_way_tag_s2;
reg [39:0] flush_way_address_s2;
always @ *
begin
    // note: dtag has 33b per tag entry way, but we are only using 29b
    dtag_tag_way0_s2 = dtag_l15_dout_s2[0*33 +: 33];
    dtag_tag_way1_s2 = dtag_l15_dout_s2[1*33 +: 33];
    dtag_tag_way2_s2 = dtag_l15_dout_s2[2*33 +: 33];
    dtag_tag_way3_s2 = dtag_l15_dout_s2[3*33 +: 33];
    mesi_state_way0_s2 = mesi_l15_dout_s2[1:0];
    mesi_state_way1_s2 = mesi_l15_dout_s2[3:2];
    mesi_state_way2_s2 = mesi_l15_dout_s2[5:4];
    mesi_state_way3_s2 = mesi_l15_dout_s2[7:6];
    mesi_read_data_s2 = mesi_l15_dout_s2;

    // for lru way check
    lru_state_s2 =   (lru_way_s2 == 0) ? mesi_state_way0_s2 :
                        (lru_way_s2 == 1) ? mesi_state_way1_s2 :
                        (lru_way_s2 == 2) ? mesi_state_way2_s2 :
                                                mesi_state_way3_s2;
    lru_way_tag_s2[(40 - 4 - ((9-2)))-1:0] =
                        (lru_way_s2 == 0) ? dtag_tag_way0_s2[(40 - 4 - ((9-2)))-1:0] :
                        (lru_way_s2 == 1) ? dtag_tag_way1_s2[(40 - 4 - ((9-2)))-1:0] :
                        (lru_way_s2 == 2) ? dtag_tag_way2_s2[(40 - 4 - ((9-2)))-1:0] :
                                                dtag_tag_way3_s2[(40 - 4 - ((9-2)))-1:0];
    lru_way_address_s2 = {lru_way_tag_s2, address_s2[(((9-2))+4-1):4], 4'b0};


    // DTAG COMPARISON
    // only compare L15_CACHE_TAG_WIDTH (29b), not full raw tag
    tagcheck_way0_equals = (address_cache_tag_s2[(40 - 4 - ((9-2)))-1:0] == dtag_tag_way0_s2[(40 - 4 - ((9-2)))-1:0]);
    tagcheck_way1_equals = (address_cache_tag_s2[(40 - 4 - ((9-2)))-1:0] == dtag_tag_way1_s2[(40 - 4 - ((9-2)))-1:0]);
    tagcheck_way2_equals = (address_cache_tag_s2[(40 - 4 - ((9-2)))-1:0] == dtag_tag_way2_s2[(40 - 4 - ((9-2)))-1:0]);
    tagcheck_way3_equals = (address_cache_tag_s2[(40 - 4 - ((9-2)))-1:0] == dtag_tag_way3_s2[(40 - 4 - ((9-2)))-1:0]);

    {tagcheck_val_s2, tagcheck_way_s2} = tagcheck_way0_equals && (mesi_state_way0_s2 != 2'd0) ? {1'b1, 2'd0} :
                                                    tagcheck_way1_equals && (mesi_state_way1_s2 != 2'd0) ?  {1'b1, 2'd1} :
                                                    tagcheck_way2_equals && (mesi_state_way2_s2 != 2'd0) ?  {1'b1, 2'd2} :
                                                    tagcheck_way3_equals && (mesi_state_way3_s2 != 2'd0) ?  {1'b1, 2'd3} : 3'b0;

    // tagcheck_way_mask_s2[`L15_UNPARAM_3_0] = tagcheck_way_s2 == 2'd0 ? 4'b0001 :
    //                                                               2'd1 ? 4'b0010 :
    //                                                               2'd2 ? 4'b0100 :
    //                                                                         4'b1000 ;

    tagcheck_lrsc_flag_s2 = (tagcheck_val_s2 == 1'b0) ? 1'b0 :
                                (tagcheck_way_s2 == 2'd0) ? lrsc_flag_l15_dout_s2[0] :
                                (tagcheck_way_s2 == 2'd1) ? lrsc_flag_l15_dout_s2[1] :
                                (tagcheck_way_s2 == 2'd2) ? lrsc_flag_l15_dout_s2[2] :
                                                            lrsc_flag_l15_dout_s2[3] ;

    tagcheck_state_s2 = (tagcheck_val_s2 == 1'b0) ? 2'd0 :
                                (tagcheck_way_s2 == 2'd0) ? mesi_state_way0_s2 :
                                (tagcheck_way_s2 == 2'd1) ? mesi_state_way1_s2 :
                                (tagcheck_way_s2 == 2'd2) ? mesi_state_way2_s2 :
                                                                     mesi_state_way3_s2 ;

    flush_way_s2 = address_s2[25:24];
    flush_state_s2 =  (flush_way_s2 == 2'd0) ? mesi_state_way0_s2 :
                            (flush_way_s2 == 2'd1) ? mesi_state_way1_s2 :
                            (flush_way_s2 == 2'd2) ? mesi_state_way2_s2 :
                                                             mesi_state_way3_s2 ;

    flush_way_tag_s2[(40 - 4 - ((9-2)))-1:0] =
                        (flush_way_s2 == 0) ? dtag_tag_way0_s2[(40 - 4 - ((9-2)))-1:0] :
                        (flush_way_s2 == 1) ? dtag_tag_way1_s2[(40 - 4 - ((9-2)))-1:0] :
                        (flush_way_s2 == 2) ? dtag_tag_way2_s2[(40 - 4 - ((9-2)))-1:0] :
                                                        dtag_tag_way3_s2[(40 - 4 - ((9-2)))-1:0];

    flush_way_address_s2 = {flush_way_tag_s2, address_s2[(((9-2))+4-1):4], 4'b0};

    // expanding some signals
    tagcheck_state_me_s2 = tagcheck_state_s2 == 2'd3 || tagcheck_state_s2 == 2'd2;
    tagcheck_state_mes_s2 = tagcheck_state_s2 == 2'd3 || tagcheck_state_s2 == 2'd2
                                                        || tagcheck_state_s2 == 2'd1;
    tagcheck_state_s_s2 = tagcheck_state_s2 == 2'd1;
    tagcheck_state_m_s2 = tagcheck_state_s2 == 2'd3;
    tagcheck_state_e_s2 = tagcheck_state_s2 == 2'd2;

    lru_state_m_s2 = lru_state_s2 == 2'd3;
    lru_state_mes_s2 = lru_state_s2 == 2'd3 || lru_state_s2 == 2'd2
                                                        || lru_state_s2 == 2'd1;

    flush_state_m_s2 = flush_state_s2 == 2'd3;
    flush_state_me_s2 = flush_state_s2 == 2'd3 || flush_state_s2 == 2'd2;
    flush_state_mes_s2 = flush_state_s2 == 2'd3 || flush_state_s2 == 2'd2
                                                        || flush_state_s2 == 2'd1;
end


//////////////////////
// LRU logic
//////////////////////
reg [4-1:0] lru_used_bits_s2;
reg [2-1:0] lru_round_robin_turn_s2;
// reg [`L15_WAY_MASK] lru_way_s2; // moved earlier for
always @ *
begin
    lru_used_bits_s2[4-1:0] = lruarray_l15_dout_s2[4-1:0];
    lru_round_robin_turn_s2[2-1:0] = lruarray_l15_dout_s2[4+2-1 -: 2];
    lru_way_s2 = 0;
    if (&lru_used_bits_s2 == 1'b1)
    begin
        // if all were used
        lru_way_s2[2-1:0] = lru_round_robin_turn_s2;
    end
    else
    begin
        case (lru_round_robin_turn_s2)
            2'd0:
            begin
                lru_way_s2[2-1:0] = (lru_used_bits_s2[2'd0] == 1'b0) ? 2'd0 :
                                            (lru_used_bits_s2[2'd1] == 1'b0) ? 2'd1 :
                                            (lru_used_bits_s2[2'd2] == 1'b0) ? 2'd2 :
                                                                      2'd3 ;
            end
            2'd1:
            begin
                lru_way_s2[2-1:0] = (lru_used_bits_s2[2'd1] == 1'b0) ? 2'd1 :
                                            (lru_used_bits_s2[2'd2] == 1'b0) ? 2'd2 :
                                            (lru_used_bits_s2[2'd3] == 1'b0) ? 2'd3 :
                                                                      2'd0 ;
            end
            2'd2:
            begin
                lru_way_s2[2-1:0] = (lru_used_bits_s2[2'd2] == 1'b0) ? 2'd2 :
                                            (lru_used_bits_s2[2'd3] == 1'b0) ? 2'd3 :
                                            (lru_used_bits_s2[2'd0] == 1'b0) ? 2'd0 :
                                                                      2'd1 ;
            end
            2'd3:
            begin
                lru_way_s2[2-1:0] = (lru_used_bits_s2[2'd3] == 1'b0) ? 2'd3 :
                                            (lru_used_bits_s2[2'd0] == 1'b0) ? 2'd0 :
                                            (lru_used_bits_s2[2'd1] == 1'b0) ? 2'd1 :
                                                                      2'd2 ;
            end
        endcase
    end
end

//////////////////////////////
// S2 MSHR write-buffer read
//////////////////////////////
reg s2_mshr_val_s2;
reg [2-1:0] s2_mshr_mshrid_s2;
always @ *
begin
    s2_mshr_val_s2 = 0;
    s2_mshr_mshrid_s2 = 0;
    case (s2_mshr_operation_s2)
        1'd1:
        begin
            s2_mshr_val_s2 = 1;
            s2_mshr_mshrid_s2 = mshrid_s2;
        end
    endcase

    // 7/16/14 timing fix: Tri: don't need stall signal for read
    // pipe_mshr_write_buffer_rd_en_s2 = s2_mshr_val_s2 && !stall_s2;
    pipe_mshr_write_buffer_rd_en_s2 = s2_mshr_val_s2;
    pipe_mshr_threadid_s2 = threadid_s2;
end

//////////////////////////////
// dcache control logics
//////////////////////////////
reg dcache_val_s2;
reg dcache_rw_s2;
reg [((9-2))-1:0] dcache_index_s2;
reg [1:0] dcache_way_s2;
reg [127:0] dcache_mshr_write_mask_s2;
reg [127:0] dcache_write_merge_mshr_noc2_s2;
reg [3-1:0] dcache_source_s2;
reg [127:0] dcache_write_mask_s2;
reg [127:0] dcache_write_data_s2;
// diag accesses
reg [((9-2))-1:0] dcache_diag_index_s2;
reg [1:0] dcache_diag_way_s2;
reg [0:0] dcache_diag_offset_s2;
reg [1:0] lru_way_s3_bypassed;


always @ *
begin
    dcache_val_s2 = 0;
    dcache_rw_s2 = 0;
    dcache_index_s2 = 0;
    dcache_way_s2 = 0;
    dcache_mshr_write_mask_s2 = 0;
    dcache_write_merge_mshr_noc2_s2 = 0;
    dcache_source_s2 = 0;
    dcache_write_mask_s2 = 0;
    dcache_write_data_s2 = 0;

    dcache_diag_way_s2 = address_s2[25:24];
    dcache_diag_index_s2 = address_s2[(((9-2))+4-1):4];
    dcache_diag_offset_s2 = address_s2[3];

    case (dcache_operation_s2)
        4'd1:
        begin
            dcache_val_s2 = tagcheck_state_m_s2;
            dcache_rw_s2 = 1'b1;
            dcache_index_s2 = cache_index_s2;
            dcache_way_s2 = tagcheck_way_s2;
        end
        4'd2:
        begin
            dcache_val_s2 = tagcheck_state_mes_s2;
            dcache_rw_s2 = 1'b1;
            dcache_index_s2 = cache_index_s2;
            dcache_way_s2 = tagcheck_way_s2;
        end
        4'd3:
        begin
            dcache_val_s2 = lru_state_m_s2;
            dcache_rw_s2 = 1'b1;
            dcache_index_s2 = cache_index_s2;
            dcache_way_s2 = lru_way_s2;
        end
        4'd10:
        begin
            dcache_val_s2 = flush_state_m_s2;
            dcache_rw_s2 = 1'b1;
            dcache_index_s2 = cache_index_s2;
            dcache_way_s2 = flush_way_s2;
        end
        4'd4:
        begin
            dcache_val_s2 = tagcheck_state_me_s2;
            dcache_rw_s2 = 1'b0;
            dcache_index_s2 = cache_index_s2;
            dcache_way_s2 = tagcheck_way_s2;
            dcache_source_s2 = 3'd1;
        end
        4'd11:
        begin
            dcache_val_s2 = (tagcheck_state_m_s2 & tagcheck_lrsc_flag_s2); // Check state_m is just redundant, but need to be conservative
            dcache_rw_s2 = 1'b0;
            dcache_index_s2 = cache_index_s2;
            dcache_way_s2 = tagcheck_way_s2;
            dcache_source_s2 = 3'd1;
        end
        4'd5:
        begin
            dcache_val_s2 = 1'b1;
            dcache_rw_s2 = 1'b0;
            dcache_index_s2 = cache_index_s2;
            dcache_way_s2 = lru_way_s3_bypassed; // need to be from S3, shouldn't change behavior but...
            dcache_source_s2 = 3'd2;
        end
        4'd6:
        begin
            dcache_val_s2 = 1'b1;
            dcache_rw_s2 = 1'b0;
            dcache_index_s2 = cache_index_s2;
            dcache_way_s2 = lru_way_s3_bypassed; // need to be from S3, shouldn't change behavior but...
            dcache_source_s2 = 3'd3;
        end
        4'd7:
        begin
            // writing to dcache the mshr way stored in st_mshr
            dcache_val_s2 = 1'b1;
            dcache_rw_s2 = 1'b0;
            dcache_index_s2 = cache_index_s2;
            dcache_way_s2 = way_mshr_st_s2;
            dcache_source_s2 = 3'd1;
        end
        4'd8:
        begin
            dcache_val_s2 = 1'b1;
            dcache_rw_s2 = 1'b1;
            dcache_index_s2 = dcache_diag_index_s2;
            dcache_way_s2 = dcache_diag_way_s2;
        end
        4'd9:
        begin
            dcache_val_s2 = 1'b1;
            dcache_rw_s2 = 1'b0;
            dcache_index_s2 = dcache_diag_index_s2;
            dcache_way_s2 = dcache_diag_way_s2;
            dcache_source_s2 = 3'd4;
        end
    endcase

    dcache_mshr_write_mask_s2 = {
        {8{mshr_pipe_write_buffer_byte_mask_s2[15]}},
        {8{mshr_pipe_write_buffer_byte_mask_s2[14]}},
        {8{mshr_pipe_write_buffer_byte_mask_s2[13]}},
        {8{mshr_pipe_write_buffer_byte_mask_s2[12]}},
        {8{mshr_pipe_write_buffer_byte_mask_s2[11]}},
        {8{mshr_pipe_write_buffer_byte_mask_s2[10]}},
        {8{mshr_pipe_write_buffer_byte_mask_s2[9]}},
        {8{mshr_pipe_write_buffer_byte_mask_s2[8]}},
        {8{mshr_pipe_write_buffer_byte_mask_s2[7]}},
        {8{mshr_pipe_write_buffer_byte_mask_s2[6]}},
        {8{mshr_pipe_write_buffer_byte_mask_s2[5]}},
        {8{mshr_pipe_write_buffer_byte_mask_s2[4]}},
        {8{mshr_pipe_write_buffer_byte_mask_s2[3]}},
        {8{mshr_pipe_write_buffer_byte_mask_s2[2]}},
        {8{mshr_pipe_write_buffer_byte_mask_s2[1]}},
        {8{mshr_pipe_write_buffer_byte_mask_s2[0]}}
    };


    dcache_write_merge_mshr_noc2_s2[127:0] =
                    {(~dcache_mshr_write_mask_s2[127:64] & noc2decoder_l15_data_0[63:0]),
                    (~dcache_mshr_write_mask_s2[63:0] & noc2decoder_l15_data_1[63:0])}
                        | (dcache_mshr_write_mask_s2[127:0] & mshr_pipe_write_buffer_s2[127:0]);

    case (dcache_source_s2)
        3'd1:
        begin
            dcache_write_mask_s2[127:0] = dcache_mshr_write_mask_s2[127:0];
            dcache_write_data_s2[127:0] = mshr_pipe_write_buffer_s2[127:0];
        end
        3'd2:
        begin
            dcache_write_mask_s2[127:0] = {128{1'b1}};
            dcache_write_data_s2[127:0] = {noc2decoder_l15_data_0[63:0], noc2decoder_l15_data_1[63:0]};
        end
        3'd3:
        begin
            dcache_write_mask_s2[127:0] = {128{1'b1}};
            dcache_write_data_s2[127:0] = dcache_write_merge_mshr_noc2_s2[127:0];
        end
        3'd4:
        begin
            dcache_write_mask_s2[127:0] = dcache_diag_offset_s2 == 1'b0 ? {{64{1'b1}},64'b0} : {64'b0,{64{1'b1}}};
            dcache_write_data_s2[127:0] = {pcxdecoder_l15_data[63:0],pcxdecoder_l15_data[63:0]};
        end
    endcase

    l15_dcache_val_s2 = dcache_val_s2 && val_s2 && !stall_s2;
    l15_dcache_rw_s2 = dcache_rw_s2;
    l15_dcache_index_s2 = {dcache_index_s2, dcache_way_s2};
    l15_dcache_write_mask_s2[127:0] = dcache_write_mask_s2;
    l15_dcache_write_data_s2[127:0] = dcache_write_data_s2;


    // extra data for CSM homeid table
    // encode packet format to internal format (smaller)

    // if source is MSHR it means a write hi
    // hmt_fill_data_s2 = (dcache_source_s2 == `L15_DCACHE_SOURCE_MSHR) ? 
    
    l15_hmt_write_data_s2[14 + 8 + 8-1:0] = 0;
    l15_hmt_write_data_s2[14 + 8 + 8 - 1 -:  14] = hmt_fill_homeid_s2[((14+8+8)-1):(8+8)];
    l15_hmt_write_data_s2[8 + 8 - 1 -: 8] = hmt_fill_homeid_s2[8-1:0];
    l15_hmt_write_data_s2[8 - 1 -: 8] = hmt_fill_homeid_s2[8+8-1:8];
    // l15_hmt_write_data_s2[`L15_CSM_GHID_CHIP_MASK] = noc2_src_homeid_s2[`PACKET_HOME_ID_CHIP_MASK];
    // l15_hmt_write_data_s2[`L15_CSM_GHID_XPOS_MASK] = noc2_src_homeid_s2[`PACKET_HOME_ID_X_MASK];
    // l15_hmt_write_data_s2[`L15_CSM_GHID_YPOS_MASK] = noc2_src_homeid_s2[`PACKET_HOME_ID_Y_MASK];
    l15_hmt_write_mask_s2 = 0;
    if (hmt_op_s2 == 1'd1)
      l15_hmt_write_mask_s2[14 + 8 + 8-1:0] = {14 + 8 + 8{1'b1}};
    
end

////////////////////////
// Home Map Table
////////////////////////
// reg hmt_val_s2;
// reg hmt_rw_s2;
// reg [(`L15_CACHE_INDEX_WIDTH+`L15_PADDR_WIDTH)-1:0] hmt_index_s2;
// reg [31:0] hmt_write_data_s2;
// reg [31:0] hmt_write_mask_s2;

////////////////////////
// MESI write control
////////////////////////
reg mesi_write_val_s2;
reg [((9-2))-1:0] mesi_write_index_s2;
reg [1:0] mesi_write_way_s2;
reg [1:0] mesi_write_state_s2;
always @ *
begin
    mesi_write_val_s2 = 0;
    mesi_write_index_s2 = 0;
    mesi_write_way_s2 = 0;
    mesi_write_state_s2 = 0;
    case (mesi_write_op_s2)
        3'd3:
        begin
            mesi_write_val_s2 = tagcheck_state_mes_s2;
            mesi_write_index_s2 = cache_index_s2;
            mesi_write_way_s2 = tagcheck_way_s2;
            mesi_write_state_s2 = 2'd0;
        end
        3'd6:
        begin
            mesi_write_val_s2 = flush_state_mes_s2;
            mesi_write_index_s2 = cache_index_s2;
            mesi_write_way_s2 = flush_way_s2;
            mesi_write_state_s2 = 2'd0;
        end
        3'd1:
        begin
            mesi_write_val_s2 = tagcheck_state_me_s2;
            mesi_write_index_s2 = cache_index_s2;
            mesi_write_way_s2 = tagcheck_way_s2;
            mesi_write_state_s2 = 2'd1;
        end
        3'd2:
        begin
            mesi_write_val_s2 = tagcheck_state_e_s2;
            mesi_write_index_s2 = cache_index_s2;
            mesi_write_way_s2 = tagcheck_way_s2;
            mesi_write_state_s2 = 2'd3;
        end
        3'd4:
        begin
            mesi_write_val_s2 = 1'b1;
            mesi_write_index_s2 = cache_index_s2;
            mesi_write_way_s2 = lru_way_s2;
            mesi_write_state_s2 = noc2_ack_state_s2;
        end
        3'd5:
        begin
            mesi_write_val_s2 = 1'b1;
            mesi_write_index_s2 = cache_index_s2;
            mesi_write_way_s2 = way_mshr_st_s2;
            mesi_write_state_s2 = noc2_ack_state_s2;
        end
    endcase
    // bugfix (stall signal needs to be here)
    // trin todo: why stall_s2 is needed
    l15_mesi_write_val_s2 = mesi_write_val_s2 && val_s2 && !stall_s2;
    l15_mesi_write_index_s2[((9-2))-1:0] = mesi_write_index_s2;
    l15_mesi_write_mask_s2[7:0] =  (mesi_write_way_s2 == 0) ? 8'b00_00_00_11 :
                            (mesi_write_way_s2 == 1) ? 8'b00_00_11_00 :
                            (mesi_write_way_s2 == 2) ? 8'b00_11_00_00 :
                                                    8'b11_00_00_00 ;
    l15_mesi_write_data_s2[7:0] = {4{mesi_write_state_s2}};
end


////////////////////////
// LRSC flag write control
////////////////////////
reg lrsc_flag_write_val_s2;
reg [((9-2))-1:0] lrsc_flag_write_index_s2;
reg [1:0] lrsc_flag_write_way_s2;
reg lrsc_flag_write_state_s2;
always @ *
begin
    lrsc_flag_write_val_s2 = 0;
    lrsc_flag_write_index_s2 = 0;
    lrsc_flag_write_way_s2 = 0;
    lrsc_flag_write_state_s2 = 0;
    case (lrsc_flag_write_op_s2)
        3'd1:
        begin
            lrsc_flag_write_val_s2 = 1'b1;
            lrsc_flag_write_index_s2 = cache_index_s2;
            lrsc_flag_write_way_s2 = lru_way_s2;
            lrsc_flag_write_state_s2 = 1'b1;
        end
        3'd2:  
        begin
            lrsc_flag_write_val_s2 = tagcheck_state_m_s2;  // can be always 1
            lrsc_flag_write_index_s2 = cache_index_s2;
            lrsc_flag_write_way_s2 = tagcheck_way_s2;
            lrsc_flag_write_state_s2 = 1'b0;
        end
        3'd3:  
        begin
            lrsc_flag_write_val_s2 = 1'b1;
            lrsc_flag_write_index_s2 = cache_index_s2;
            lrsc_flag_write_way_s2 = lru_way_s2;
            lrsc_flag_write_state_s2 = 1'b0;
        end
        3'd4:  
        begin
            lrsc_flag_write_val_s2 = flush_state_m_s2;
            lrsc_flag_write_index_s2 = cache_index_s2;
            lrsc_flag_write_way_s2 = flush_way_s2;
            lrsc_flag_write_state_s2 = 1'b0;
        end
    endcase
    // bugfix (stall signal needs to be here)
    // trin todo: why stall_s2 is needed
    l15_lrsc_flag_write_val_s2 = lrsc_flag_write_val_s2 && val_s2 && !stall_s2;
    l15_lrsc_flag_write_index_s2[((9-2))-1:0] = lrsc_flag_write_index_s2;
    l15_lrsc_flag_write_mask_s2[3:0] =  (lrsc_flag_write_way_s2 == 0) ? 4'b0001 :
                            (lrsc_flag_write_way_s2 == 1) ? 4'b0010 :
                            (lrsc_flag_write_way_s2 == 2) ? 4'b0100 :
                                                    4'b1000 ;
    l15_lrsc_flag_write_data_s2[3:0] = {4{lrsc_flag_write_state_s2}};
end

////////////////////////////
// output to CSM
////////////////////////////

// reg csm_read_ghid_val_s2;
reg [3-1:0] csm_ticket_s2;
// reg [`L15_CSM_NUM_TICKETS_LOG2-1:0] csm_ticket_s2_next;

// always @ (posedge clk)
// begin
//     if (!rst_n)
//     begin
//         csm_ticket_s2 <= 0;
//     end
//     else
//     begin
//         if (csm_read_ghid_val_s2 && !stall_s2 && val_s2)
//             csm_ticket_s2 <= csm_ticket_s2_next;
//     end
// end

reg [127:0] csm_fill_data;
reg csm_req_val_s2;
reg csm_req_type_s2;
reg csm_req_lru_address_s2;
reg [39:0] csm_address_s2;
always @ *
begin
    // csm_ticket_s2_next = csm_ticket_s2 + 1;
    // csm_read_ghid_val_s2 = 0;
    csm_fill_data = 0;
    csm_req_val_s2 = 0;
    csm_req_type_s2 = 0;
    csm_req_lru_address_s2 = 0;
    csm_address_s2 = 0;
    // l15_csm_req_ticket_s2 = 0;
    csm_ticket_s2 = {threadid_s2, mshrid_s2};
    l15_csm_req_ticket_s2 = csm_ticket_s2;
    case (csm_op_s2)
        4'd1:
        begin
            csm_req_val_s2 = 1'b1;
            csm_req_type_s2 = 1'b0;
            // csm_read_ghid_val_s2 = 1'b1;
            csm_address_s2 = address_s2;
            // l15_csm_req_ticket_s2 = csm_ticket_s2;
        end
        4'd2:
        begin
            csm_req_val_s2 = (tagcheck_state_me_s2 == 1'b0);
            csm_req_type_s2 = 1'b0;
            // csm_read_ghid_val_s2 = 1'b1;
            csm_address_s2 = address_s2;
            // l15_csm_req_ticket_s2 = csm_ticket_s2;
        end
        4'd6:
        begin
            csm_req_val_s2 = (tagcheck_state_mes_s2 == 1'b0);
            csm_req_type_s2 = 1'b0;
            // csm_read_ghid_val_s2 = 1'b1;
            csm_address_s2 = address_s2;
            // l15_csm_req_ticket_s2 = csm_ticket_s2;
        end
        4'd3:
        begin
            csm_req_val_s2 = 1'b1;
            csm_req_type_s2 = 1'b1;
            csm_fill_data = {noc2decoder_l15_data_1[63:0], noc2decoder_l15_data_0[63:0]};
            //the req ticket is embedded in the mshrid of the refill msg
            l15_csm_req_ticket_s2 = noc2decoder_l15_csm_mshrid[3-1:0];
        end
        4'd4:
        begin
            csm_req_val_s2 = 1'b1;
            csm_req_type_s2 = 1'b1;
            csm_fill_data = {pcxdecoder_l15_data[63:0],pcxdecoder_l15_data[63:0]};
            // l15_csm_req_ticket_s2 = csm_ticket_s2;
            csm_address_s2 = address_s2;
        end
        4'd5:
        begin
            csm_req_val_s2 = 1'b1;
            csm_req_type_s2 = 1'b0;
            // l15_csm_req_ticket_s2 = csm_ticket_s2;
            csm_address_s2 = address_s2;
        end
        4'd7:
        begin
            csm_req_val_s2 = 1'b1;
            // csm_req_type_s2 = 1'b0; // type is not needed?
            // l15_csm_req_ticket_s2 = csm_ticket_s2;
            csm_address_s2 = address_s2;
        end
        
        
















    endcase

    l15_csm_req_address_s2 = csm_address_s2;
    l15_csm_req_val_s2 = csm_req_val_s2 && !stall_s2 && val_s2;
    l15_csm_req_type_s2 = csm_req_type_s2;
    // l15_csm_stall_s2 = stall_s2;
    // l15_csm_clump_tile_count_s2 = 1'b0; // adsfasdf
    l15_csm_req_data_s2 = csm_fill_data[127:0];
    l15_csm_req_pcx_data_s2 = csm_pcx_data_s2;
end


///////////////////////
// WMT read op
///////////////////////
reg wmt_read_val_s2;
reg [6:0] wmt_read_index_s2;
always @ *
begin
    wmt_read_val_s2 = 0;
    wmt_read_index_s2 = 0;

    case(wmt_read_op_s2)
        1'd1:
        begin
            wmt_read_val_s2 = 1'b1;
            wmt_read_index_s2 = cache_index_l1d_s2[6:0];
            // assuming l1.5 is bigger or equal to l1d
        end
    endcase

    l15_wmt_read_val_s2 = wmt_read_val_s2 && val_s2 && !stall_s2;
    l15_wmt_read_index_s2 = wmt_read_index_s2;
end

////////////////////////////
// config operation
////////////////////////////

reg config_req_val_s2;
reg config_req_rw_s2;
reg [63:0] config_write_req_data_s2;
reg [39:0] config_req_address_s2;

always @ *
begin
    config_req_val_s2 = 0;
    config_req_rw_s2 = 0;
    config_write_req_data_s2 = 0;
    config_req_address_s2 = 0;
    case (config_op_s2)
        2'd1:
        begin
            config_req_val_s2 = 1'b1;
            config_req_rw_s2 = 1'b0;
            config_req_address_s2 = address_s2;
        end
        2'd2:
        begin
            config_req_val_s2 = 1'b1;
            config_req_rw_s2 = 1'b1;
            config_req_address_s2 = address_s2;
            config_write_req_data_s2[63:0] = pcxdecoder_l15_data[63:0];
        end
    endcase

    l15_config_req_val_s2 = config_req_val_s2 && val_s2 && !stall_s2;
    l15_config_req_rw_s2 = config_req_rw_s2;
    l15_config_write_req_data_s2 = config_write_req_data_s2;
    l15_config_req_address_s2 = config_req_address_s2[15:8];
end

/************************************************/
/*                 STAGE 3                      */
/************************************************/

// propagated variables (flops) from S2

reg [2-1:0] tagcheck_way_s3;
reg [2-1:0] tagcheck_way_s3_next;
reg [2-1:0] tagcheck_state_s3;
reg [2-1:0] tagcheck_state_s3_next;
reg [1-1:0] tagcheck_lrsc_flag_s3;
reg [1-1:0] tagcheck_lrsc_flag_s3_next;
reg [2-1:0] flush_way_s3;
reg [2-1:0] flush_way_s3_next;
reg [2-1:0] flush_state_s3;
reg [2-1:0] flush_state_s3_next;
reg [2-1:0] lru_way_s3;
reg [2-1:0] lru_way_s3_next;
reg [2-1:0] lru_state_s3;
reg [2-1:0] lru_state_s3_next;
reg [2-1:0] mshrid_s3;
reg [2-1:0] mshrid_s3_next;
reg [40-1:0] address_s3;
reg [40-1:0] address_s3_next;
reg [1-1:0] threadid_s3;
reg [1-1:0] threadid_s3_next;
reg [1-1:0] non_cacheable_s3;
reg [1-1:0] non_cacheable_s3_next;
reg [3-1:0] size_s3;
reg [3-1:0] size_s3_next;
reg [1-1:0] prefetch_s3;
reg [1-1:0] prefetch_s3_next;
reg [2-1:0] l1_replacement_way_s3;
reg [2-1:0] l1_replacement_way_s3_next;
reg [1-1:0] l2_miss_s3;
reg [1-1:0] l2_miss_s3_next;
reg [1-1:0] f4b_s3;
reg [1-1:0] f4b_s3_next;
reg [1-1:0] blockstore_s3;
reg [1-1:0] blockstore_s3_next;
reg [1-1:0] blockstoreinit_s3;
reg [1-1:0] blockstoreinit_s3_next;
reg [(14+8+8)-1:0] noc2_src_homeid_s3;
reg [(14+8+8)-1:0] noc2_src_homeid_s3_next;
reg [3-1:0] lruarray_write_op_s3;
reg [3-1:0] lruarray_write_op_s3_next;
reg [1-1:0] predecode_noc2_inval_s3;
reg [1-1:0] predecode_noc2_inval_s3_next;
reg [4-1:0] predecode_fwd_subcacheline_vector_s3;
reg [4-1:0] predecode_fwd_subcacheline_vector_s3_next;
reg [6-1:0] predecode_reqtype_s3;
reg [6-1:0] predecode_reqtype_s3_next;
reg [3-1:0] wmt_write_op_s3;
reg [3-1:0] wmt_write_op_s3_next;
reg [3-1:0] wmt_compare_op_s3;
reg [3-1:0] wmt_compare_op_s3_next;
reg [3-1:0] csm_ticket_s3;
reg [3-1:0] csm_ticket_s3_next;
reg [3-1:0] s3_mshr_operation_s3;
reg [3-1:0] s3_mshr_operation_s3_next;
reg [5-1:0] cpx_operation_s3;
reg [5-1:0] cpx_operation_s3_next;
reg [5-1:0] noc1_operation_s3;
reg [5-1:0] noc1_operation_s3_next;
reg [4-1:0] noc3_operations_s3;
reg [4-1:0] noc3_operations_s3_next;
reg [2-1:0] pcx_ack_stage_s3;
reg [2-1:0] pcx_ack_stage_s3_next;
reg [2-1:0] noc2_ack_stage_s3;
reg [2-1:0] noc2_ack_stage_s3_next;
reg [2-1:0] noc2_ack_state_s3;
reg [2-1:0] noc2_ack_state_s3_next;
reg [(40 - 4 - ((9-2)))-1:0] lru_way_tag_s3;
reg [(40 - 4 - ((9-2)))-1:0] lru_way_tag_s3_next;
reg [(40 - 4 - ((9-2)))-1:0] flush_way_tag_s3;
reg [(40 - 4 - ((9-2)))-1:0] flush_way_tag_s3_next;
reg [33-1:0] csm_pcx_data_s3;
reg [33-1:0] csm_pcx_data_s3_next;

reg val_s3_next;
reg [(2 + 4)-1:0] lru_data_s3;

// local variables
reg cpxencoder_req_staled_s3;
reg cpxencoder_req_staled_s3_next;
reg noc1encoder_req_staled_s3;
reg noc1encoder_req_staled_s3_next;
reg noc3encoder_req_staled_s3;
reg noc3encoder_req_staled_s3_next;
reg stall_for_cpx_s3;
// reg stall_for_noc1_s3;
reg stall_for_noc3_s3;

reg tagcheck_state_me_s3;
reg tagcheck_state_mes_s3;
reg tagcheck_state_s_s3;
reg tagcheck_state_e_s3;
reg tagcheck_state_m_s3;
reg lru_state_m_s3;
reg lru_state_mes_s3;
reg flush_state_m_s3;
reg flush_state_mes_s3;

reg [39:0] lru_way_address_s3;
reg [39:0] flush_way_address_s3;

always @ (posedge clk)
begin
    if (!rst_n)
    begin
        cpxencoder_req_staled_s3 <= 0;
        noc1encoder_req_staled_s3 <= 0;
        noc3encoder_req_staled_s3 <= 0;
        val_s3 <= 0;
        lru_data_s3 <= 0;
        tagcheck_way_s3 <= 0;
tagcheck_state_s3 <= 0;
tagcheck_lrsc_flag_s3 <= 0;
flush_way_s3 <= 0;
flush_state_s3 <= 0;
lru_way_s3 <= 0;
lru_state_s3 <= 0;
mshrid_s3 <= 0;
address_s3 <= 0;
threadid_s3 <= 0;
non_cacheable_s3 <= 0;
size_s3 <= 0;
prefetch_s3 <= 0;
l1_replacement_way_s3 <= 0;
l2_miss_s3 <= 0;
f4b_s3 <= 0;
blockstore_s3 <= 0;
blockstoreinit_s3 <= 0;
noc2_src_homeid_s3 <= 0;
lruarray_write_op_s3 <= 0;
predecode_noc2_inval_s3 <= 0;
predecode_fwd_subcacheline_vector_s3 <= 0;
predecode_reqtype_s3 <= 0;
wmt_write_op_s3 <= 0;
wmt_compare_op_s3 <= 0;
csm_ticket_s3 <= 0;
s3_mshr_operation_s3 <= 0;
cpx_operation_s3 <= 0;
noc1_operation_s3 <= 0;
noc3_operations_s3 <= 0;
pcx_ack_stage_s3 <= 0;
noc2_ack_stage_s3 <= 0;
noc2_ack_state_s3 <= 0;
lru_way_tag_s3 <= 0;
flush_way_tag_s3 <= 0;
csm_pcx_data_s3 <= 0;

    end
    else
    begin
        cpxencoder_req_staled_s3 <= cpxencoder_req_staled_s3_next;
        noc1encoder_req_staled_s3 <= noc1encoder_req_staled_s3_next;
        noc3encoder_req_staled_s3 <= noc3encoder_req_staled_s3_next;
        val_s3 <= val_s3_next;
        lru_data_s3 <= lruarray_l15_dout_s2;
        tagcheck_way_s3 <= tagcheck_way_s3_next;
tagcheck_state_s3 <= tagcheck_state_s3_next;
tagcheck_lrsc_flag_s3 <= tagcheck_lrsc_flag_s3_next;
flush_way_s3 <= flush_way_s3_next;
flush_state_s3 <= flush_state_s3_next;
lru_way_s3 <= lru_way_s3_next;
lru_state_s3 <= lru_state_s3_next;
mshrid_s3 <= mshrid_s3_next;
address_s3 <= address_s3_next;
threadid_s3 <= threadid_s3_next;
non_cacheable_s3 <= non_cacheable_s3_next;
size_s3 <= size_s3_next;
prefetch_s3 <= prefetch_s3_next;
l1_replacement_way_s3 <= l1_replacement_way_s3_next;
l2_miss_s3 <= l2_miss_s3_next;
f4b_s3 <= f4b_s3_next;
blockstore_s3 <= blockstore_s3_next;
blockstoreinit_s3 <= blockstoreinit_s3_next;
noc2_src_homeid_s3 <= noc2_src_homeid_s3_next;
lruarray_write_op_s3 <= lruarray_write_op_s3_next;
predecode_noc2_inval_s3 <= predecode_noc2_inval_s3_next;
predecode_fwd_subcacheline_vector_s3 <= predecode_fwd_subcacheline_vector_s3_next;
predecode_reqtype_s3 <= predecode_reqtype_s3_next;
wmt_write_op_s3 <= wmt_write_op_s3_next;
wmt_compare_op_s3 <= wmt_compare_op_s3_next;
csm_ticket_s3 <= csm_ticket_s3_next;
s3_mshr_operation_s3 <= s3_mshr_operation_s3_next;
cpx_operation_s3 <= cpx_operation_s3_next;
noc1_operation_s3 <= noc1_operation_s3_next;
noc3_operations_s3 <= noc3_operations_s3_next;
pcx_ack_stage_s3 <= pcx_ack_stage_s3_next;
noc2_ack_stage_s3 <= noc2_ack_stage_s3_next;
noc2_ack_state_s3 <= noc2_ack_state_s3_next;
lru_way_tag_s3 <= lru_way_tag_s3_next;
flush_way_tag_s3 <= flush_way_tag_s3_next;
csm_pcx_data_s3 <= csm_pcx_data_s3_next;

    end
end



// reg [`L15_UNPARAM_2_0] lru_way_wmt_data_s3;
// reg [`L15_UNPARAM_1_0] lru_way_to_l1_s3;
// reg lru_l1waymap_val_s3;

// reg [`L15_UNPARAM_2_0] flush_way_wmt_data_s3;
// reg [`L15_UNPARAM_1_0] flush_way_to_l1_s3;
// reg flush_l1waymap_val_s3;

// reg [`L15_UNPARAM_2_0] tagcheck_way_wmt_data_s3;
// reg [`L15_UNPARAM_1_0] tagcheck_way_to_l1_s3;
// reg tagcheck_l1waymap_val_s3;

// reg [`L15_UNPARAM_2_0] dedup_wmt_way_wmt_data_s3;
// reg [`L15_UNPARAM_1_0] dedup_wmt_way_to_l1_s3;
// reg dedup_wmt_l1waymap_val_s3;

// reg [`L15_UNPARAM_2_0] stbuf_way_wmt_data_s3;
// reg [`L15_UNPARAM_1_0] stbuf_way_to_l1_s3;
// reg stbuf_l1waymap_val_s3;


reg [1:0] stbuf_compare_address_match_s3;
reg [1:0] stbuf_compare_match_s3;
reg [1:0] stbuf_compare_lru_match_s3;
reg [0:0] stbuf_compare_threadid_s3;
reg [0:0] stbuf_compare_lru_threadid_s3;
reg stbuf_compare_match_val_s3;
reg stbuf_compare_lru_match_val_s3;
reg [1:0] stbuf_way_s3; // wmt todo: move calculation to s2

// STORE BUFFER STUFF
always @ *
begin



    stbuf_compare_address_match_s3[0] = mshr_st_address_array[0][39:4] == address_s3[39:4];

    stbuf_compare_match_s3[0] = mshr_val_array[0][2'd3] 
                                && (mshr_st_state_array[0] == 2'b01) 
                                && (stbuf_compare_address_match_s3[0] == 1'b1);
    stbuf_compare_lru_match_s3[0] = stbuf_compare_match_s3[0] && (mshr_st_way_array[0] == lru_way_s3);




    stbuf_compare_address_match_s3[1] = mshr_st_address_array[1][39:4] == address_s3[39:4];

    stbuf_compare_match_s3[1] = mshr_val_array[1][2'd3] 
                                && (mshr_st_state_array[1] == 2'b01) 
                                && (stbuf_compare_address_match_s3[1] == 1'b1);
    stbuf_compare_lru_match_s3[1] = stbuf_compare_match_s3[1] && (mshr_st_way_array[1] == lru_way_s3);

    stbuf_compare_threadid_s3 = stbuf_compare_match_s3[1] ? 1'b1 : 1'b0;
    stbuf_compare_lru_threadid_s3 = stbuf_compare_lru_match_s3[1] ? 1'b1 : 1'b0;
    stbuf_compare_match_val_s3 = stbuf_compare_match_s3[0] || stbuf_compare_match_s3[1];
    stbuf_compare_lru_match_val_s3 = stbuf_compare_lru_match_s3[0] || stbuf_compare_lru_match_s3[1];

    stbuf_way_s3 = mshr_st_way_array[stbuf_compare_threadid_s3];
    // stbuf_way_wmt_data_s3 = wmt_data_s3[stbuf_way_s3];
    // stbuf_way_to_l1_s3 = stbuf_way_wmt_data_s3[`L15_UNPARAM_1_0];
    // stbuf_l1waymap_val_s3 = stbuf_way_wmt_data_s3[2];
end

reg [3:0] tagcheck_way_mask_s3;
always @ *
begin
    // expanding some signals
    tagcheck_way_mask_s3[3:0] = tagcheck_way_s3 == 2'd0 ? 4'b0001 :
                                                  2'd1 ? 4'b0010 :
                                                  2'd2 ? 4'b0100 :
                                                        4'b1000 ;

    tagcheck_state_me_s3 = tagcheck_state_s3 == 2'd3 || tagcheck_state_s3 == 2'd2;
    tagcheck_state_mes_s3 = tagcheck_state_s3 == 2'd3 || tagcheck_state_s3 == 2'd2
                                                        || tagcheck_state_s3 == 2'd1;
    tagcheck_state_s_s3 = tagcheck_state_s3 == 2'd1;
    tagcheck_state_m_s3 = tagcheck_state_s3 == 2'd3;
    tagcheck_state_e_s3 = tagcheck_state_s3 == 2'd2;

    lru_state_m_s3 = lru_state_s3 == 2'd3;
    lru_state_mes_s3 = lru_state_s3 == 2'd3 || lru_state_s3 == 2'd2
                                                        || lru_state_s3 == 2'd1;

    flush_state_m_s3 = flush_state_s3 == 2'd3;
    flush_state_mes_s3 = flush_state_s3 == 2'd3 || flush_state_s3 == 2'd2
                                                        || flush_state_s3 == 2'd1;

    cache_index_s3 = address_s3[(((9-2))+4-1):4];
    cache_index_l1d_s3 = address_s3[(6 + 4):4];
    lru_way_address_s3 = {lru_way_tag_s3, cache_index_s3, 4'b0};
    flush_way_address_s3 = {flush_way_tag_s3, cache_index_s3, 4'b0};

end


always @* begin
    // next signals
    if (stall_s3)
    begin
        val_s3_next = val_s3;
        tagcheck_way_s3_next = tagcheck_way_s3;
tagcheck_state_s3_next = tagcheck_state_s3;
tagcheck_lrsc_flag_s3_next = tagcheck_lrsc_flag_s3;
flush_way_s3_next = flush_way_s3;
flush_state_s3_next = flush_state_s3;
lru_way_s3_next = lru_way_s3;
lru_state_s3_next = lru_state_s3;
mshrid_s3_next = mshrid_s3;
address_s3_next = address_s3;
threadid_s3_next = threadid_s3;
non_cacheable_s3_next = non_cacheable_s3;
size_s3_next = size_s3;
prefetch_s3_next = prefetch_s3;
l1_replacement_way_s3_next = l1_replacement_way_s3;
l2_miss_s3_next = l2_miss_s3;
f4b_s3_next = f4b_s3;
blockstore_s3_next = blockstore_s3;
blockstoreinit_s3_next = blockstoreinit_s3;
noc2_src_homeid_s3_next = noc2_src_homeid_s3;
lruarray_write_op_s3_next = lruarray_write_op_s3;
predecode_noc2_inval_s3_next = predecode_noc2_inval_s3;
predecode_fwd_subcacheline_vector_s3_next = predecode_fwd_subcacheline_vector_s3;
predecode_reqtype_s3_next = predecode_reqtype_s3;
wmt_write_op_s3_next = wmt_write_op_s3;
wmt_compare_op_s3_next = wmt_compare_op_s3;
csm_ticket_s3_next = csm_ticket_s3;
s3_mshr_operation_s3_next = s3_mshr_operation_s3;
cpx_operation_s3_next = cpx_operation_s3;
noc1_operation_s3_next = noc1_operation_s3;
noc3_operations_s3_next = noc3_operations_s3;
pcx_ack_stage_s3_next = pcx_ack_stage_s3;
noc2_ack_stage_s3_next = noc2_ack_stage_s3;
noc2_ack_state_s3_next = noc2_ack_state_s3;
lru_way_tag_s3_next = lru_way_tag_s3;
flush_way_tag_s3_next = flush_way_tag_s3;
csm_pcx_data_s3_next = csm_pcx_data_s3;

    end
    else
    begin
        val_s3_next = val_s2 && !stall_s2;
        tagcheck_way_s3_next = tagcheck_way_s2;
tagcheck_state_s3_next = tagcheck_state_s2;
tagcheck_lrsc_flag_s3_next = tagcheck_lrsc_flag_s2;
flush_way_s3_next = flush_way_s2;
flush_state_s3_next = flush_state_s2;
lru_way_s3_next = lru_way_s2;
lru_state_s3_next = lru_state_s2;
mshrid_s3_next = mshrid_s2;
address_s3_next = address_s2;
threadid_s3_next = threadid_s2;
non_cacheable_s3_next = non_cacheable_s2;
size_s3_next = size_s2;
prefetch_s3_next = prefetch_s2;
l1_replacement_way_s3_next = l1_replacement_way_s2;
l2_miss_s3_next = l2_miss_s2;
f4b_s3_next = f4b_s2;
blockstore_s3_next = blockstore_s2;
blockstoreinit_s3_next = blockstoreinit_s2;
noc2_src_homeid_s3_next = noc2_src_homeid_s2;
lruarray_write_op_s3_next = lruarray_write_op_s2;
predecode_noc2_inval_s3_next = predecode_noc2_inval_s2;
predecode_fwd_subcacheline_vector_s3_next = predecode_fwd_subcacheline_vector_s2;
predecode_reqtype_s3_next = predecode_reqtype_s2;
wmt_write_op_s3_next = wmt_write_op_s2;
wmt_compare_op_s3_next = wmt_compare_op_s2;
csm_ticket_s3_next = csm_ticket_s2;
s3_mshr_operation_s3_next = s3_mshr_operation_s2;
cpx_operation_s3_next = cpx_operation_s2;
noc1_operation_s3_next = noc1_operation_s2;
noc3_operations_s3_next = noc3_operations_s2;
pcx_ack_stage_s3_next = pcx_ack_stage_s2;
noc2_ack_stage_s3_next = noc2_ack_stage_s2;
noc2_ack_state_s3_next = noc2_ack_state_s2;
lru_way_tag_s3_next = lru_way_tag_s2;
flush_way_tag_s3_next = flush_way_tag_s2;
csm_pcx_data_s3_next = csm_pcx_data_s2;

    end
end

always @* begin
    // stale logics
    if (!cpxencoder_req_staled_s3)
        cpxencoder_req_staled_s3_next = (!stall_for_cpx_s3 && stall_for_noc3_s3) ? 1'b1 : 1'b0;
    else
        cpxencoder_req_staled_s3_next = stall_s3 ? 1'b1 : 1'b0;
end

always @* begin
    if (!noc1encoder_req_staled_s3)
        noc1encoder_req_staled_s3_next = (stall_for_cpx_s3 || stall_for_noc3_s3) ? 1'b1 : 1'b0;
    else
        noc1encoder_req_staled_s3_next = stall_s3 ? 1'b1 : 1'b0;
end

always @* begin
    if (!noc3encoder_req_staled_s3)
        noc3encoder_req_staled_s3_next = (!stall_for_noc3_s3 && stall_for_cpx_s3) ? 1'b1 : 1'b0;
    else
        noc3encoder_req_staled_s3_next = stall_s3 ? 1'b1 : 1'b0;
end

always @* begin
    //
    // Stalling logics
    // 1. s3 can stall from CPX not completing the request
    // 2. '' from NoC1 not completing the request
    // 3. '' from NoC3 not completing the request
    stall_for_cpx_s3 = !cpxencoder_req_staled_s3 && l15_cpxencoder_val && !cpxencoder_l15_req_ack;
    // stall_for_noc1_s3 = 1'b0;
    stall_for_noc3_s3 = !noc3encoder_req_staled_s3 && l15_noc3encoder_req_val && !noc3encoder_l15_req_ack;
    stall_s3 = val_s3 && (stall_for_cpx_s3 || stall_for_noc3_s3);

    // PCX/Noc2 ack logics
    pcx_ack_s3 = val_s3 && !stall_s3 && (pcx_ack_stage_s3 == 2'd3);
    noc2_ack_s3 = val_s3 && !stall_s3 && (noc2_ack_stage_s3 == 2'd3);

    // CSM logics
    l15_csm_stall_s3 = stall_s3;

    // misc
    lru_way_s3_bypassed = lru_way_s3;
end



// L1D tag table check logic
// FLOPPED INPUTS: 
//      wmt_l15_data_s3
//      l1_tagcheck_op_s3
//      lru_way_s3
//      tagcheck_way_s3
//      flush_way_s3
// CALCULATED INPUTS IN S3
//      stbuf_way_s3
// LOCAL VARIABLES
reg [(2+0)-1:0] wmt_compare_data_s3;
reg [2-1:0] wmt_compare_way_s3;
// OUTPUTS
reg [((2+0)+1)-1:0] wmt_data_s3 [0:2-1];
reg [2-1:0] wmt_compare_mask_s3;
reg wmt_compare_match_s3;
reg [1-1:0] wmt_compare_match_way_s3;

always @ *
begin
    // note: [`L15_UNPARAM_2_0] = {valid, way[2]}
    // wmt_data_s3[0] = wmt_l15_data_s3[`L15_WMT_ENTRY_0_MASK];
    // wmt_data_s3[1] = wmt_l15_data_s3[`L15_WMT_ENTRY_1_MASK];
    // wmt_data_s3[2] = wmt_l15_data_s3[`L15_WMT_ENTRY_2_MASK];
    // wmt_data_s3[3] = wmt_l15_data_s3[`L15_WMT_ENTRY_3_MASK];
    
  wmt_data_s3[0] = wmt_l15_data_s3[(0+1)*((2+0)+1)-1 -: ((2+0)+1)];


  wmt_data_s3[1] = wmt_l15_data_s3[(1+1)*((2+0)+1)-1 -: ((2+0)+1)];



    wmt_compare_data_s3 = 0;
    wmt_compare_way_s3 = 0;

    case (wmt_compare_op_s3)
        3'd1:
        begin
            wmt_compare_way_s3 = lru_way_s3;
        end
        3'd2:
        begin
            wmt_compare_way_s3 = tagcheck_way_s3;
        end
        3'd3:
        begin
            wmt_compare_way_s3 = flush_way_s3;
        end
        3'd4:
        begin
            wmt_compare_way_s3 = stbuf_way_s3;
        end
    endcase


    wmt_compare_data_s3 = wmt_compare_way_s3;




    // invalidating entries in way table due to invals, evictions, non cacheable, etc...
    // first, find the mask
    // wmt_compare_mask_s3 = 0;
    // wmt_compare_mask_s3[0] = wmt_data_s3[0][`L15_WMT_VALID_MASK] && (wmt_compare_data_s3[`L15_WMT_DATA_MASK] == wmt_data_s3[0][`L15_WMT_DATA_MASK]);
    // wmt_compare_mask_s3[1] = wmt_data_s3[1][`L15_WMT_VALID_MASK] && (wmt_compare_data_s3[`L15_WMT_DATA_MASK] == wmt_data_s3[1][`L15_WMT_DATA_MASK]);
    // wmt_compare_mask_s3[2] = wmt_data_s3[2][`L15_WMT_VALID_MASK] && (wmt_compare_data_s3[`L15_WMT_DATA_MASK] == wmt_data_s3[2][`L15_WMT_DATA_MASK]);
    // wmt_compare_mask_s3[3] = wmt_data_s3[3][`L15_WMT_VALID_MASK] && (wmt_compare_data_s3[`L15_WMT_DATA_MASK] == wmt_data_s3[3][`L15_WMT_DATA_MASK]);

    
  wmt_compare_mask_s3[0] = wmt_data_s3[0][(2+0)] && (wmt_compare_data_s3[(2+0)-1:0] == wmt_data_s3[0][(2+0)-1:0]);


  wmt_compare_mask_s3[1] = wmt_data_s3[1][(2+0)] && (wmt_compare_data_s3[(2+0)-1:0] == wmt_data_s3[1][(2+0)-1:0]);



    // results used for cpx invalidations
    wmt_compare_match_s3 = |wmt_compare_mask_s3;
    // wmt_compare_match_way_s3 = wmt_compare_mask_s3[0] ? 2'd0 :
                               // wmt_compare_mask_s3[1] ? 2'd1 :
                               // wmt_compare_mask_s3[2] ? 2'd2 :
                                                        // 2'd3 ;
    wmt_compare_match_way_s3 = 0;
if (wmt_compare_mask_s3[0])
   wmt_compare_match_way_s3 = 0;
else if (wmt_compare_mask_s3[1])
   wmt_compare_match_way_s3 = 1;

end

//////////////////////
// LRU write logic
//////////////////////
// note: lru write is moved to s3 from s2 for timing
// correctness should be the same
reg [(2 + 4)-1:0] lruarray_write_data_s3;
reg lruarray_write_val_s3;
reg [3:0] lruarray_lru_mask_s3;
reg [3:0] lruarray_tagcheck_mask_s3;
reg [3:0] lruarray_flush_mask_s3;
reg [((9-2))-1:0] lruarray_write_index_s3;
always @ *
begin
    lruarray_write_data_s3 = 0;
    lruarray_write_val_s3 = 0;
    lruarray_lru_mask_s3 = lru_way_s3[1:0] == 2'd0 ? 4'b0001 :
                                  lru_way_s3[1:0] == 2'd1 ? 4'b0010 :
                                  lru_way_s3[1:0] == 2'd2 ? 4'b0100 :
                                                                     4'b1000 ;
    lruarray_flush_mask_s3 =   flush_way_s3[1:0] == 2'd0 ? 4'b0001 :
                                        flush_way_s3[1:0] == 2'd1 ? 4'b0010 :
                                        flush_way_s3[1:0] == 2'd2 ? 4'b0100 :
                                                                            4'b1000 ;
    lruarray_tagcheck_mask_s3 = tagcheck_way_mask_s3;

    case (lruarray_write_op_s3)
        3'd1:
        begin
            lruarray_write_val_s3 = tagcheck_state_mes_s3;
            if ((lru_data_s3[3:0] | lruarray_tagcheck_mask_s3) == 4'b1111)
                lruarray_write_data_s3[3:0] = 4'b0000;
            else
                lruarray_write_data_s3[3:0] = lru_data_s3[3:0] | lruarray_tagcheck_mask_s3;
            lruarray_write_data_s3[5:4] = lru_data_s3[5:4]; // retain old round robin
        end
        3'd6:
        begin
            lruarray_write_val_s3 = tagcheck_lrsc_flag_s3;
            if ((lru_data_s3[3:0] | lruarray_tagcheck_mask_s3) == 4'b1111)
                lruarray_write_data_s3[3:0] = 4'b0000;
            else
                lruarray_write_data_s3[3:0] = lru_data_s3[3:0] | lruarray_tagcheck_mask_s3;
            lruarray_write_data_s3[5:4] = lru_data_s3[5:4]; // retain old round robin
        end
        3'd2:
        begin
            // lruarray_write_val_s3 = 1'b1; // might be an error to change here
            // lruarray_write_data_s3[`L15_UNPARAM_3_0] = lru_data_s3[`L15_UNPARAM_3_0] & ~lruarray_lru_mask_s3;
            //    // retain old access bits and remove the evicted bit
            // lruarray_write_data_s3[5:4] = lru_data_s3[5:4] + 2'b1;
        end
        3'd3:
        begin
            // lruarray_write_val_s3 = 1'b0; // no change for now, only turn on access bit on the second access
            // lruarray_write_data_s3[`L15_UNPARAM_3_0] = lru_data_s3[`L15_UNPARAM_3_0] & ~lruarray_lru_mask_s3;
            //    // retain old access bits and remove the evicted bit
            // lruarray_write_data_s3[5:4] = lru_data_s3[5:4] + 2'b1;
            lruarray_write_val_s3 = 1'b1; // no change for now, only turn on access bit on the second access
            lruarray_write_data_s3[3:0] = lru_data_s3[3:0] | lruarray_lru_mask_s3;
            lruarray_write_data_s3[5:4] = lru_data_s3[5:4] + 2'b1;
        end
        3'd4:
        begin
            lruarray_write_val_s3 = tagcheck_state_mes_s3;
            lruarray_write_data_s3[3:0] = lru_data_s3[3:0] & ~lruarray_tagcheck_mask_s3;
                // retain old access bits and remove the evicted bit
            lruarray_write_data_s3[5:4] = lru_data_s3[5:4];
        end
        3'd5:
        begin
            lruarray_write_val_s3 = flush_state_mes_s3;
            lruarray_write_data_s3[3:0] = lru_data_s3[3:0] & ~lruarray_flush_mask_s3;
                // retain old access bits and remove the evicted bit
            lruarray_write_data_s3[5:4] = lru_data_s3[5:4];
        end
    endcase
    lruarray_write_index_s3 = cache_index_s3;

    l15_lruarray_write_val_s3 = lruarray_write_val_s3 && val_s3;
    l15_lruarray_write_data_s3 = lruarray_write_data_s3;
    l15_lruarray_write_mask_s3 = 6'b111111;
    // l15_lruarray_write_mask_s3 = 6'b0;
    l15_lruarray_write_index_s3 = lruarray_write_index_s3;
end

///////////////////////
// WMT write op
///////////////////////

// FLOPPED INPUTS
// CALCULATED INPUTS IN S3
// LOCAL VARIABLES
// OUTPUTS

reg wmt_write_val_s3;
reg [6:0] wmt_write_index_s3;
reg [2*((2+0)+1)-1:0] wmt_write_data_s3;
reg wmt_write_inval_val_s3;
reg wmt_write_update_val_s3;
reg wmt_write_dedup_l1way_val_s3;

reg [2-1:0] wmt_write_update_way_s3;
// reg [`L1D_WAY_MASK] wmt_write_inval_way_s3;
// reg [`L15_UNPARAM_2_0] wmt_write_dedup_l1way_s3;

reg [2*((2+0)+1)-1:0] wmt_write_inval_mask_s3;
reg [2*((2+0)+1)-1:0] wmt_write_update_mask_s3;
reg [2*((2+0)+1)-1:0] wmt_write_dedup_mask_s3;
reg [2*((2+0)+1)-1:0] wmt_write_mask_s3;

// LOCAL VARIABLES
reg [(2+0)-1:0] wmt_write_update_data_s3;
reg [0-1:0] wmt_alias_bits;

// OUTPUTS


// reg [`L15_WMT_ENTRY_MASK] wmt_data_s3 [0:`L1D_WAY_COUNT-1];
// reg [`L1D_WAY_COUNT-1:0] wmt_compare_mask_s3;
// reg wmt_compare_match_s3;
// reg [`L1D_WAY_MASK] wmt_compare_match_way_s3;
// // reg [?:0] dummy;

always @ *
begin
    wmt_write_val_s3 = 0;
    wmt_write_index_s3 = 0;

    wmt_write_inval_val_s3 = 0;
    wmt_write_update_val_s3 = 0;
    wmt_write_dedup_l1way_val_s3 = 0;

    wmt_write_update_way_s3 = 0;
    wmt_write_update_data_s3 = 0;
    // wmt_write_inval_way_s3 = 0;
    // wmt_write_dedup_l1way_s3 = 0;

    wmt_write_inval_mask_s3 = 0;
    wmt_write_update_mask_s3 = 0;
    wmt_write_dedup_mask_s3 = 0;

    case(wmt_write_op_s3)
        3'd1:
        begin
            wmt_write_val_s3 = tagcheck_state_mes_s3;
            wmt_write_index_s3 = cache_index_l1d_s3[6:0];
            wmt_write_inval_val_s3 = 1'b1;
            // wmt_write_inval_way_s3 = tagcheck_way_s3;
        end
        3'd2:
        begin
            wmt_write_val_s3 = lru_state_mes_s3;
            wmt_write_index_s3 = cache_index_l1d_s3[6:0];
            wmt_write_inval_val_s3 = 1'b1;
            // wmt_write_inval_way_s3 = lru_way_s3;
        end
        3'd5:
        begin
            wmt_write_val_s3 = val_s3 && flush_state_mes_s3;
            wmt_write_index_s3 = cache_index_l1d_s3[6:0];
            wmt_write_inval_val_s3 = 1'b1;
            // wmt_write_inval_way_s3 = flush_way_s3;
        end
        3'd3: 
        begin
            wmt_write_val_s3 = 1'b1;
            wmt_write_index_s3 = cache_index_l1d_s3[6:0];
            wmt_write_update_val_s3 = 1'b1;
            wmt_write_update_way_s3 = lru_way_s3;
            wmt_write_inval_val_s3 = 1'b1;
        end
        3'd4:
        begin
            wmt_write_val_s3 = tagcheck_state_mes_s3;
            wmt_write_index_s3 = cache_index_l1d_s3[6:0];
            wmt_write_update_val_s3 = 1'b1;
            wmt_write_update_way_s3 = tagcheck_way_s3;
            wmt_write_inval_val_s3 = 1'b1;
        end
    endcase

    // some processings

    // updating way table for filling to l1d:
    // just override l1_replacement_way_s3 with wmt_write_update_way_s3
    // wmt_write_update_mask_s3[`L15_WMT_MASK] =  (wmt_write_update_val_s3 == 1'b0) ? 12'b000_000_000_000 :
    //                                                  (l1_replacement_way_s3 == 2'd0) ? 12'b000_000_000_111 :
    //                                                  (l1_replacement_way_s3 == 2'd1) ? 12'b000_000_111_000 :
    //                                                  (l1_replacement_way_s3 == 2'd2) ? 12'b000_111_000_000 :
    //                                                                                    12'b111_000_000_000;
    wmt_write_update_mask_s3[2*((2+0)+1)-1:0] = 0;
    if (wmt_write_update_val_s3 == 1'b1)
    begin
        // if (l1_replacement_way_s3 == 2'd0)
        //     wmt_write_update_mask_s3[`L15_WMT_ENTRY_0_MASK] = {`L15_WMT_ENTRY_WIDTH{1'b1}};
        // if (l1_replacement_way_s3 == 2'd1)
        //     wmt_write_update_mask_s3[`L15_WMT_ENTRY_1_MASK] = {`L15_WMT_ENTRY_WIDTH{1'b1}};
        // if (l1_replacement_way_s3 == 2'd2)
        //     wmt_write_update_mask_s3[`L15_WMT_ENTRY_2_MASK] = {`L15_WMT_ENTRY_WIDTH{1'b1}};
        // if (l1_replacement_way_s3 == 2'd3)
        //     wmt_write_update_mask_s3[`L15_WMT_ENTRY_3_MASK] = {`L15_WMT_ENTRY_WIDTH{1'b1}};
        
  if (l1_replacement_way_s3 == 0)
      wmt_write_update_mask_s3[(0+1)*((2+0)+1)-1 -: ((2+0)+1)] = {((2+0)+1){1'b1}};


  if (l1_replacement_way_s3 == 1)
      wmt_write_update_mask_s3[(1+1)*((2+0)+1)-1 -: ((2+0)+1)] = {((2+0)+1){1'b1}};


    end

    // wmt_write_inval_mask_s3[`L15_WMT_MASK] = (wmt_write_inval_val_s3 == 1'b0) ? 12'b000_000_000_000 :
                                                        // (wmt_write_inval_way_s3 == 2'd0) ? 12'b000_000_000_111 :
                                                        // (wmt_write_inval_way_s3 == 2'd1) ? 12'b000_000_111_000 :
                                                        // (wmt_write_inval_way_s3 == 2'd2) ? 12'b000_111_000_000 :
    //                                                                                             12'b111_000_000_000;


    // wmt_write_inval_mask_s3[`L15_WMT_MASK] = (wmt_write_inval_val_s3 == 1'b0) ? 12'b000_000_000_000 :
    //                                                     {
    //                                                      {3{wmt_compare_mask_s3[3]}},
    //                                                      {3{wmt_compare_mask_s3[2]}},
    //                                                      {3{wmt_compare_mask_s3[1]}},
    //                                                      {3{wmt_compare_mask_s3[0]}}
    //                                                      };
    wmt_write_inval_mask_s3[2*((2+0)+1)-1:0] = 0;
    if (wmt_write_inval_val_s3 == 1'b1)
    begin
        // wmt_write_inval_mask_s3[`L15_WMT_ENTRY_0_MASK] = {`L15_WMT_ENTRY_WIDTH{wmt_compare_mask_s3[0]}};
        // wmt_write_inval_mask_s3[`L15_WMT_ENTRY_1_MASK] = {`L15_WMT_ENTRY_WIDTH{wmt_compare_mask_s3[1]}};
        // wmt_write_inval_mask_s3[`L15_WMT_ENTRY_2_MASK] = {`L15_WMT_ENTRY_WIDTH{wmt_compare_mask_s3[2]}};
        // wmt_write_inval_mask_s3[`L15_WMT_ENTRY_3_MASK] = {`L15_WMT_ENTRY_WIDTH{wmt_compare_mask_s3[3]}};
        
  wmt_write_inval_mask_s3[(0+1)*((2+0)+1)-1 -: ((2+0)+1)] = {((2+0)+1){wmt_compare_mask_s3[0]}};


  wmt_write_inval_mask_s3[(1+1)*((2+0)+1)-1 -: ((2+0)+1)] = {((2+0)+1){wmt_compare_mask_s3[1]}};


    end


    wmt_write_update_data_s3 = wmt_write_update_way_s3;





    // wmt_write_mask_s3[`L15_WMT_MASK] = wmt_write_inval_mask_s3 | wmt_write_update_mask_s3 | wmt_write_dedup_mask_s3;
    wmt_write_mask_s3[2*((2+0)+1)-1:0] = wmt_write_inval_mask_s3 | wmt_write_update_mask_s3;
    wmt_write_data_s3[2*((2+0)+1)-1:0] = {2{{1'b1,wmt_write_update_data_s3}}} & wmt_write_update_mask_s3;

    // MODULE OUTPUT
    // trin: timing fix: removing guard for wmt writes; should still be correct
    // l15_wmt_write_val_s3 = wmt_write_val_s3 && !stall_s3 && val_s3;
    l15_wmt_write_val_s3 = wmt_write_val_s3 && val_s3;
    l15_wmt_write_index_s3 = wmt_write_index_s3;
    l15_wmt_write_mask_s3 = wmt_write_mask_s3;
    l15_wmt_write_data_s3 = wmt_write_data_s3;
end





/////////////////////////////////////////////////
// S3 MSHR state update & deallocation control
/////////////////////////////////////////////////

reg lru_eviction_matched_st1_s3;
reg lru_eviction_matched_st2_s3;
reg tagcheck_matched_st1_s3;
reg tagcheck_matched_st2_s3;
reg s3_mshr_val_s3;
reg [3-1:0] s3_mshr_write_type_s3;
reg [2-1:0] s3_mshr_update_state_s3;
reg [2-1:0] s3_mshr_write_mshrid_s3;
reg [1:0] s3_mshr_update_way_s3;
reg [0:0] s3_mshr_write_threadid_s3;

always @ *
begin
    s3_mshr_val_s3 = 0;
    s3_mshr_write_type_s3 = 0;
    s3_mshr_write_mshrid_s3 = 0;
    s3_mshr_update_state_s3 = 0;
    s3_mshr_update_way_s3 = 0;
    s3_mshr_write_threadid_s3[0:0] = threadid_s3[0:0];

    case (s3_mshr_operation_s3)
        3'd1:
        begin
            s3_mshr_val_s3 = val_s3;
            s3_mshr_write_type_s3 = 3'b010;
            s3_mshr_write_mshrid_s3 = mshrid_s3;
        end
        3'd2:
        begin
            s3_mshr_val_s3 = val_s3 && tagcheck_state_mes_s3;
            s3_mshr_write_type_s3 = 3'b010;
            s3_mshr_write_mshrid_s3 = mshrid_s3;
        end
        3'd3:
        begin
            s3_mshr_val_s3 = val_s3;
            s3_mshr_write_type_s3 = tagcheck_state_me_s3 ?  3'b010 :
                                                                            3'b011;
            s3_mshr_write_mshrid_s3 = mshrid_s3;
            s3_mshr_update_state_s3 = tagcheck_state_mes_s3 ?
                                                    2'b01 : 2'b10;
            s3_mshr_update_way_s3 = tagcheck_way_s3;
        end
        3'd4:
        begin
            s3_mshr_val_s3 = val_s3 && stbuf_compare_match_val_s3;
            s3_mshr_write_type_s3 = 3'b011;
            // s3_mshr_write_mshrid_s3 = tagcheck_matched_st1_s3 ? 4'd8 : 4'd9; // doesn't matter to update states
            s3_mshr_update_state_s3 = 2'b10;
            // s3_mshr_update_way_s3 = tagcheck_way_s3;
            s3_mshr_write_threadid_s3[0:0] = stbuf_compare_threadid_s3[0:0];
        end
        3'd5:
        begin
            s3_mshr_val_s3 = val_s3 && stbuf_compare_lru_match_val_s3;
            s3_mshr_write_type_s3 = 3'b011;
            // s3_mshr_write_mshrid_s3 = lru_eviction_matched_st1_s3 ? 4'd8 : 4'd9; // doesn't matter to update states
            s3_mshr_update_state_s3 = 2'b10;
            s3_mshr_write_threadid_s3[0:0] = stbuf_compare_lru_threadid_s3[0:0];
            // s3_mshr_update_way_s3 = lru_way_s3;
        end
        3'd6:
        begin
            s3_mshr_val_s3 = val_s3;
            s3_mshr_write_type_s3 = 3'b011;
            // s3_mshr_write_mshrid_s3 = lru_eviction_matched_st1_s3 ? 4'd8 : 4'd9; // doesn't matter to update states
            s3_mshr_update_state_s3 = 2'b11;
            // s3_mshr_write_threadid_s3[`L15_THREADID_MASK] = stbuf_compare_lru_threadid_s3[`L15_T
            // s3_mshr_update_way_s3 = lru_way_s3;
        end
    endcase

    // s3
    pipe_mshr_val_s3 = s3_mshr_val_s3;
    pipe_mshr_op_s3 = s3_mshr_write_type_s3;
    pipe_mshr_mshrid_s3 = s3_mshr_write_mshrid_s3;
    pipe_mshr_threadid_s3[0:0] = s3_mshr_write_threadid_s3[0:0];
    pipe_mshr_write_update_state_s3 = s3_mshr_update_state_s3;
    pipe_mshr_write_update_way_s3 = s3_mshr_update_way_s3;
end

// CPX request logic
reg cpx_req_val_s3;
reg [4-1:0] cpx_type_s3;
reg cpx_invalidate_l1_s3;
// reg cpx_dcache_inval_all_s3;
reg [1:0] cpx_inval_way_s3;
reg [3-1:0] cpx_data_source_s3;
reg cpx_atomic_bit_s3;
reg cpx_icache_inval_s3;
always @ *
begin
    cpx_req_val_s3 = 0;
    cpx_type_s3 = 0;
    cpx_invalidate_l1_s3 = 0;
    // cpx_dcache_inval_all_s3 = 0;
    cpx_inval_way_s3 = 0;
    cpx_data_source_s3 = 0;
    cpx_atomic_bit_s3 = 0;
    cpx_icache_inval_s3 = 0;
    case(cpx_operation_s3)
        5'd14:
        begin
            cpx_req_val_s3 = 1'b1;
            cpx_type_s3 = 4'b0011;
            cpx_icache_inval_s3 = 1;
        end
        5'd19:
        begin
            cpx_req_val_s3 = 1'b1;
            cpx_type_s3 = 4'b0011;
            cpx_invalidate_l1_s3 = 1;
            cpx_inval_way_s3 = wmt_compare_match_way_s3;
        end
        5'd1:
        begin
            cpx_req_val_s3 = tagcheck_state_mes_s3 && wmt_compare_match_s3;
            cpx_type_s3 = 4'b0011;
            cpx_invalidate_l1_s3 = 1;
            cpx_inval_way_s3 = wmt_compare_match_way_s3;
        end
        5'd8:
        begin
            cpx_req_val_s3 = lru_state_mes_s3 && wmt_compare_match_s3;
            cpx_type_s3 = 4'b0011;
            cpx_invalidate_l1_s3 = 1;
            cpx_inval_way_s3 = wmt_compare_match_way_s3;
        end
        5'd17:
        begin
            cpx_req_val_s3 = flush_state_mes_s3 && wmt_compare_match_s3;
            cpx_type_s3 = 4'b0011;
            cpx_invalidate_l1_s3 = 1;
            cpx_inval_way_s3 = wmt_compare_match_way_s3;
        end
        5'd2:
        begin
            cpx_req_val_s3 = 1'b1;
            cpx_type_s3 = 4'b0000;
            cpx_data_source_s3 = 3'd0;
        end
        5'd3:
        begin
            cpx_req_val_s3 = tagcheck_state_mes_s3;
            cpx_type_s3 = 4'b0000;
            cpx_data_source_s3 = 3'd0;
        end
        5'd4:
        begin
            cpx_req_val_s3 = tagcheck_state_mes_s3;
            cpx_type_s3 = 4'b0000;
            cpx_data_source_s3 = 3'd1;
        end
        5'd5:
        begin
            cpx_req_val_s3 = 1'b1;
            cpx_type_s3 = 4'b0000;
            cpx_data_source_s3 = 3'd2;
        end
        5'd6:
        begin
            cpx_req_val_s3 = 1'b1;
            cpx_type_s3 = 4'b0100;
        end
        5'd20:
        begin
            cpx_req_val_s3 = 1'b1;
            cpx_type_s3 = 4'b1110;
            cpx_data_source_s3 = 3'd5;
            cpx_atomic_bit_s3 = 1;
        end
        5'd11:
        begin
            // gen st ack if store hits storebuffer or store req is returned
            //  so that the L1 can update its cache with the write data

            // first, the stbuffer's address must match current address
            // then, stbuffer's state must be in SM (otherwise L1.5 does not have a copy and so l1 does not)
            // third, stbuffer's L1-mapped way must be valid

            cpx_req_val_s3 = 1'b1;
            cpx_invalidate_l1_s3 = wmt_compare_match_s3 && stbuf_compare_match_val_s3;
            cpx_type_s3 = 4'b0100;
            cpx_inval_way_s3 = wmt_compare_match_way_s3;
        end
        5'd7:
        begin
            cpx_req_val_s3 = tagcheck_state_me_s3;
            cpx_type_s3 = 4'b0100;
            cpx_invalidate_l1_s3 = wmt_compare_match_s3;
            cpx_inval_way_s3 = wmt_compare_match_way_s3;
        end
        5'd9:
        begin
            cpx_req_val_s3 = 1'b1;
            cpx_type_s3 = 4'b0001;
            cpx_data_source_s3 = 3'd2;
        end
        5'd10:
        begin
            cpx_req_val_s3 = 1'b1;
            cpx_type_s3 = 4'b1110;
            cpx_data_source_s3 = 3'd2;
            cpx_atomic_bit_s3 = 1;
        end
        5'd12:
        begin
            cpx_req_val_s3 = 1'b1;
            cpx_type_s3 = 4'b0111;
            // cpx_flush_bit_s3 = 1'b1;
        end
        5'd13:
        begin
            cpx_req_val_s3 = 1'b1;
            cpx_type_s3 = 4'b0111;
            cpx_data_source_s3 = 3'd2;
            // cpx_flush_bit_s3 = 1'b1;
        end
        5'd15:
        begin
            cpx_req_val_s3 = 1'b1;
            cpx_type_s3 = 4'b0000;
            cpx_data_source_s3 = 3'd3;
        end
        5'd16:
        begin
            // this is for diag read to dcache
            cpx_req_val_s3 = 1'b1;
            cpx_type_s3 = 4'b0000;
            cpx_data_source_s3 = 3'd1;
        end
        5'd18:
        begin
            // return diagnostic read from csm module back to the core
            cpx_req_val_s3 = 1'b1;
            cpx_type_s3 = 4'b0000;
            cpx_data_source_s3 = 3'd4;
        end
    endcase

    l15_cpxencoder_returntype[3:0] = cpx_type_s3; //default
    l15_cpxencoder_val = val_s3 && cpx_req_val_s3 && !cpxencoder_req_staled_s3;

    l15_cpxencoder_l2miss = 0;
    l15_cpxencoder_error[1:0] = 0;
    l15_cpxencoder_noncacheable = 0;
    l15_cpxencoder_threadid = 0;
    l15_cpxencoder_prefetch = 0;
    l15_cpxencoder_f4b = 0;
    l15_cpxencoder_atomic = 0;
    l15_cpxencoder_inval_icache_all_way = 0;
    l15_cpxencoder_inval_dcache_all_way = 0;
    l15_cpxencoder_inval_address_15_4[15:4] = 0;
    l15_cpxencoder_cross_invalidate = 0;
    l15_cpxencoder_cross_invalidate_way[1:0] = 0;
    l15_cpxencoder_inval_dcache_inval = 0;
    l15_cpxencoder_inval_icache_inval = 0;
    l15_cpxencoder_inval_way[1:0] = 0;
    l15_cpxencoder_blockinitstore = 0;
    if (cpx_operation_s3 != 5'd13)
    begin
        l15_cpxencoder_l2miss = l2_miss_s3;
        l15_cpxencoder_error[1:0] = 2'b00; // no error feed back from L2 or L1 for now
        l15_cpxencoder_noncacheable = non_cacheable_s3;
        l15_cpxencoder_threadid = threadid_s3;
        l15_cpxencoder_prefetch = prefetch_s3;
        l15_cpxencoder_f4b = f4b_s3; // I think this is the instruction fill operation, should get from L2
        l15_cpxencoder_atomic = cpx_atomic_bit_s3;
        l15_cpxencoder_inval_icache_all_way = cpx_icache_inval_s3;
        // l15_cpxencoder_inval_dcache_all_way = cpx_dcache_inval_all_s3;
        l15_cpxencoder_inval_address_15_4[15:4] = address_s3[15:4];
        l15_cpxencoder_cross_invalidate = 0; // Don't think we will be cross invalidating
        l15_cpxencoder_cross_invalidate_way[1:0] = 2'b0;
        l15_cpxencoder_inval_dcache_inval = cpx_invalidate_l1_s3; // default
        // l15_cpxencoder_inval_icache_inval = cpx_icache_inval_s3; // default
        l15_cpxencoder_inval_icache_inval = 0; // default
        l15_cpxencoder_inval_way[1:0] = cpx_inval_way_s3[1:0];
        l15_cpxencoder_blockinitstore = blockstore_s3 || blockstoreinit_s3;
    end

    l15_cpxencoder_data_0[63:0] = 0;
    l15_cpxencoder_data_1[63:0] = 0;
    l15_cpxencoder_data_2[63:0] = 0;
    l15_cpxencoder_data_3[63:0] = 0;
    case (cpx_data_source_s3)
        3'd5:
        begin
            l15_cpxencoder_data_0[63:0] = ((size_s3 == 3'b010) ?
                {7'b0,(~tagcheck_lrsc_flag_s3),24'b0, 7'b0,(~tagcheck_lrsc_flag_s3),24'b0} :
                {7'b0,(~tagcheck_lrsc_flag_s3),56'b0});
            l15_cpxencoder_data_1[63:0] = ((size_s3 == 3'b010) ?
                {7'b0,(~tagcheck_lrsc_flag_s3),24'b0, 7'b0,(~tagcheck_lrsc_flag_s3),24'b0} :
                {7'b0,(~tagcheck_lrsc_flag_s3),56'b0}); // write 0 on success
        end
        3'd1:
        begin
            l15_cpxencoder_data_0[63:0] = dcache_l15_dout_s3[127:64];
            l15_cpxencoder_data_1[63:0] = dcache_l15_dout_s3[63:0];
        end
        3'd2:
        begin
            l15_cpxencoder_data_0[63:0] = noc2decoder_l15_data_0[63:0];
            l15_cpxencoder_data_1[63:0] = noc2decoder_l15_data_1[63:0];
            l15_cpxencoder_data_2[63:0] = noc2decoder_l15_data_2[63:0];
            l15_cpxencoder_data_3[63:0] = noc2decoder_l15_data_3[63:0];
        end
        3'd3:
        begin
            l15_cpxencoder_data_0[63:0] = config_l15_read_res_data_s3[63:0];
        end
        3'd4:
        begin
            l15_cpxencoder_data_0[63:0] = csm_l15_res_data_s3[63:0];
            l15_cpxencoder_data_1[63:0] = csm_l15_res_data_s3[63:0];
        end
    endcase
end

// helper block for expanding HMT entry to packet homeid for consumption in noc1 & noc3 encoder

reg [(14+8+8)-1:0] expanded_hmt_homeid_s3;
always @ *
begin
    expanded_hmt_homeid_s3 = 0;
    // translater from internal smaller format to standardized packet format
    expanded_hmt_homeid_s3[((14+8+8)-1):(8+8)] = hmt_l15_dout_s3[14 + 8 + 8 - 1 -:  14];
    expanded_hmt_homeid_s3[8+8-1:8] = hmt_l15_dout_s3[8 - 1 -: 8];
    expanded_hmt_homeid_s3[8-1:0] = hmt_l15_dout_s3[8 + 8 - 1 -: 8];
end

// NoC1 request logic
reg noc1_req_val_s3;
reg [5-1:0] noc1_type_s3;
reg [2-1:0] noc1_data_source_s3;
reg noc1_homeid_not_required_s3;
reg [2-1:0] noc1_homeid_source_s3;
always @ *
begin
    noc1_req_val_s3 = 0;
    noc1_type_s3 = 0;
    noc1_data_source_s3 = 0;
    noc1_homeid_not_required_s3 = 0;
    creditman_noc1_mispredicted_s3 = 0;
    creditman_noc1_reserve_s3 = 0;
    l15_noc1buffer_req_address = 0;
    noc1_homeid_source_s3 = 0;
    case (noc1_operation_s3)
        5'd8:
        begin
            noc1_req_val_s3 = val_s3 && lru_state_m_s3;
            noc1_type_s3 = 5'd1;
            l15_noc1buffer_req_address = lru_way_address_s3;
            creditman_noc1_mispredicted_s3 = noc1_req_val_s3 ? 0 : val_s3;
            noc1_homeid_source_s3 = 2'd3;
        end
        5'd12:
        begin
            noc1_req_val_s3 = val_s3 && flush_state_m_s3;
            noc1_type_s3 = 5'd1;
            l15_noc1buffer_req_address = flush_way_address_s3;
            creditman_noc1_mispredicted_s3 = noc1_req_val_s3 ? 0 : val_s3;
            noc1_homeid_source_s3 = 2'd3;
        end
        5'd9:
        begin
            noc1_req_val_s3 = val_s3 && tagcheck_state_m_s3;
            noc1_type_s3 = 5'd1;
            l15_noc1buffer_req_address = address_s3;
            creditman_noc1_mispredicted_s3 = noc1_req_val_s3 ? 0 : val_s3;
            noc1_homeid_source_s3 = 2'd3;
        end
        5'd1:
        begin
            noc1_req_val_s3 = val_s3;
            noc1_type_s3 = 5'd2;
            l15_noc1buffer_req_address = address_s3;
            noc1_homeid_source_s3 = 2'd1;
            // creditman_noc1_reserve_s3 = val_s3; // These are actually non-cacheable/prefetch loads
        end
        5'd2:
        begin
            noc1_req_val_s3 = val_s3 && !tagcheck_state_mes_s3;
            noc1_type_s3 = 5'd2;
            l15_noc1buffer_req_address = address_s3;
            noc1_homeid_source_s3 = 2'd1;
            creditman_noc1_mispredicted_s3 = noc1_req_val_s3 ? 1'b0 : val_s3;
            creditman_noc1_reserve_s3 = noc1_req_val_s3 ? 1'b1 : 1'b0;
        end
        5'd3:
        begin
            noc1_req_val_s3 = val_s3;
            noc1_type_s3 = 5'd3;
            l15_noc1buffer_req_address = address_s3;
            noc1_homeid_source_s3 = 2'd1;
        end
        5'd5:
        begin
            noc1_req_val_s3 = val_s3;
            noc1_type_s3 = 5'd4;
            noc1_data_source_s3 = 2'd1;
            l15_noc1buffer_req_address = address_s3;
            noc1_homeid_source_s3 = 2'd1;
        end
        5'd4:
        begin
            noc1_req_val_s3 = val_s3 && ((tagcheck_state_s3 == 2'd1) || (tagcheck_state_s3 == 2'd0));
            noc1_type_s3 = (tagcheck_state_s3 == 2'd1) ? 5'd5 :
                                                                        5'd6;
            noc1_data_source_s3 = 2'd1;
            l15_noc1buffer_req_address = address_s3;
            noc1_homeid_source_s3 = 2'd1;
            creditman_noc1_mispredicted_s3 = noc1_req_val_s3 ? 1'b0 : val_s3;
            creditman_noc1_reserve_s3 = noc1_req_val_s3 ? 1'b1 : 1'b0;
        end
        5'd21:
        begin
            noc1_req_val_s3 = val_s3;
            noc1_type_s3 = 5'd18;
            //noc1_data_source_s3 = `L15_NOC1_SOURCE_PCX_128B;
            l15_noc1buffer_req_address = address_s3;
            noc1_homeid_source_s3 = 2'd1;
            creditman_noc1_reserve_s3 = val_s3;
        end
        5'd6:
        begin
            noc1_req_val_s3 = val_s3;
            noc1_type_s3 = 5'd7;
            noc1_data_source_s3 = 2'd2;
            l15_noc1buffer_req_address = address_s3;
            noc1_homeid_source_s3 = 2'd1;
        end
        5'd7:
        begin
            noc1_req_val_s3 = val_s3;
            noc1_type_s3 = 5'd8;
            noc1_data_source_s3 = 2'd1;
            l15_noc1buffer_req_address = address_s3;
            noc1_homeid_source_s3 = 2'd1;
        end
        5'd13:
        begin
            noc1_req_val_s3 = val_s3;
            noc1_type_s3 = 5'd10;
            noc1_data_source_s3 = 2'd1;
            l15_noc1buffer_req_address = address_s3;
            noc1_homeid_source_s3 = 2'd1;
        end
        5'd14:
        begin
            noc1_req_val_s3 = val_s3;
            noc1_type_s3 = 5'd11;
            noc1_data_source_s3 = 2'd1;
            l15_noc1buffer_req_address = address_s3;
            noc1_homeid_source_s3 = 2'd1;
        end
        5'd15:
        begin
            noc1_req_val_s3 = val_s3;
            noc1_type_s3 = 5'd12;
            noc1_data_source_s3 = 2'd1;
            l15_noc1buffer_req_address = address_s3;
            noc1_homeid_source_s3 = 2'd1;
        end
        5'd16:
        begin
            noc1_req_val_s3 = val_s3;
            noc1_type_s3 = 5'd13;
            noc1_data_source_s3 = 2'd1;
            l15_noc1buffer_req_address = address_s3;
            noc1_homeid_source_s3 = 2'd1;
        end
        5'd17:
        begin
            noc1_req_val_s3 = val_s3;
            noc1_type_s3 = 5'd14;
            noc1_data_source_s3 = 2'd1;
            l15_noc1buffer_req_address = address_s3;
            noc1_homeid_source_s3 = 2'd1;
        end
        5'd18:
        begin
            noc1_req_val_s3 = val_s3;
            noc1_type_s3 = 5'd15;
            noc1_data_source_s3 = 2'd1;
            l15_noc1buffer_req_address = address_s3;
            noc1_homeid_source_s3 = 2'd1;
        end
        5'd19:
        begin
            noc1_req_val_s3 = val_s3;
            noc1_type_s3 = 5'd16;
            noc1_data_source_s3 = 2'd1;
            l15_noc1buffer_req_address = address_s3;
            noc1_homeid_source_s3 = 2'd1;
        end
        5'd20:
        begin
            noc1_req_val_s3 = val_s3;
            noc1_type_s3 = 5'd17;
            noc1_data_source_s3 = 2'd1;
            l15_noc1buffer_req_address = address_s3;
            noc1_homeid_source_s3 = 2'd1;
        end
        5'd10:
        begin
            noc1_req_val_s3 = val_s3;
            noc1_type_s3 = 5'd9;
            noc1_data_source_s3 = 2'd1;
            noc1_homeid_not_required_s3 = 1'b1;
        end
    endcase

    l15_noc1buffer_req_val = noc1_req_val_s3 && !noc1encoder_req_staled_s3;
    l15_noc1buffer_req_type = noc1_type_s3;
    l15_noc1buffer_req_threadid = threadid_s3;
    l15_noc1buffer_req_mshrid = mshrid_s3;
    l15_noc1buffer_req_non_cacheable = non_cacheable_s3;
    l15_noc1buffer_req_size = size_s3;
    l15_noc1buffer_req_prefetch = prefetch_s3;
    // l15_noc1buffer_req_blkstore = 0; // not using block stores
    // l15_noc1buffer_req_blkinitstore = 0; // not using block stores

    l15_noc1buffer_req_data_0[63:0] = 0;
    l15_noc1buffer_req_data_1[63:0] = 0;
    case (noc1_data_source_s3)
        2'd1:
        begin
            l15_noc1buffer_req_data_0[63:0] = pcxdecoder_l15_data[63:0];
            // l15_noc1buffer_req_data_1[`L15_UNPARAM_63_0] = 64'b0;
        end
        2'd2:
        begin
            l15_noc1buffer_req_data_0[63:0] = pcxdecoder_l15_data[63:0];
            l15_noc1buffer_req_data_1[63:0] = pcxdecoder_l15_data_next_entry[63:0];
        end
    endcase

    // output homeid info to noc1
    l15_noc1buffer_req_csm_ticket = csm_ticket_s3;

    l15_noc1buffer_req_homeid = 0;
    l15_noc1buffer_req_homeid_val = 0;







    case (noc1_homeid_source_s3)
        2'd1:
        begin
            l15_noc1buffer_req_homeid = csm_l15_res_data_s3;
            l15_noc1buffer_req_homeid_val = csm_l15_res_val_s3;
        end

        2'd3:
        begin
            l15_noc1buffer_req_homeid = expanded_hmt_homeid_s3;
            l15_noc1buffer_req_homeid_val = 1'b1;
        end

    endcase
    l15_noc1buffer_req_homeid_val = l15_noc1buffer_req_homeid_val || noc1_homeid_not_required_s3;

    l15_noc1buffer_req_csm_data = csm_pcx_data_s3; // extra information for sdid, lsid, hdid, lhid...
end


// NoC3 request logic
reg noc3_req_val_s3;
reg [3-1:0] noc3_type_s3;
reg noc3_with_data_s3;
reg [39:0] noc3_address_s3;
reg [2-1:0] noc3_homeid_source_s3;
always @ *
begin
    noc3_req_val_s3 = 0;
    noc3_type_s3 = 0;
    noc3_with_data_s3 = 0;
    noc3_address_s3 = 0;
    noc3_homeid_source_s3 = 0;

    case (noc3_operations_s3)
        4'd5:
        begin
            noc3_req_val_s3 = val_s3 && tagcheck_state_m_s3;
            noc3_type_s3 = 3'd1;
            noc3_with_data_s3 = 1;
            noc3_address_s3 = address_s3;
            noc3_homeid_source_s3 = 2'd3;
        end
        4'd6:
        begin
            noc3_req_val_s3 = val_s3 && lru_state_m_s3;
            noc3_type_s3 = 3'd1;
            noc3_with_data_s3 = 1;
            noc3_address_s3 = lru_way_address_s3;
            noc3_homeid_source_s3 = 2'd3;
        end
        4'd8:
        begin
            noc3_req_val_s3 = val_s3 && flush_state_m_s3;
            noc3_type_s3 = 3'd1;
            noc3_with_data_s3 = 1;
            noc3_address_s3 = flush_way_address_s3;
            noc3_homeid_source_s3 = 2'd3;
        end
        4'd1:
        begin
            noc3_req_val_s3 = val_s3;
            noc3_type_s3 = 3'd2;
            noc3_with_data_s3 = tagcheck_state_m_s3;
            noc3_address_s3 = address_s3;
            noc3_homeid_source_s3 = 2'd2;
        end
        4'd2:
        begin
            noc3_req_val_s3 = val_s3 && (tagcheck_state_m_s3);
            noc3_type_s3 = 3'd2;
            noc3_with_data_s3 = 1;
            noc3_address_s3 = address_s3;
            noc3_homeid_source_s3 = 2'd2;
        end
        4'd3:
        begin
            noc3_req_val_s3 = val_s3;
            noc3_type_s3 = 3'd3;
            noc3_with_data_s3 = tagcheck_state_m_s3;
            noc3_address_s3 = address_s3;
            noc3_homeid_source_s3 = 2'd2;
        end
        4'd4:
        begin
            noc3_req_val_s3 = val_s3 && (tagcheck_state_m_s3);
            noc3_type_s3 = 3'd3;
            noc3_with_data_s3 = 1;
            noc3_address_s3 = address_s3;
            noc3_homeid_source_s3 = 2'd2;
        end
        4'd7:
        begin
            noc3_req_val_s3 = val_s3;
            noc3_type_s3 = 3'd4;
            noc3_address_s3 = address_s3;
            noc3_homeid_source_s3 = 2'd2;
        end
    endcase
    l15_noc3encoder_req_val = noc3_req_val_s3 && !noc3encoder_req_staled_s3;
    l15_noc3encoder_req_type = noc3_type_s3;
    l15_noc3encoder_req_data_0 = dcache_l15_dout_s3[127:64];
    l15_noc3encoder_req_data_1 = dcache_l15_dout_s3[63:0];
    l15_noc3encoder_req_mshrid = mshrid_s3;
    l15_noc3encoder_req_sequenceid = cache_index_s3[1:0];
    l15_noc3encoder_req_threadid = threadid_s3[0:0];
    l15_noc3encoder_req_address[39:0] = noc3_address_s3[39:0];
    l15_noc3encoder_req_with_data = noc3_with_data_s3;
    l15_noc3encoder_req_was_inval = predecode_noc2_inval_s3;
    l15_noc3encoder_req_fwdack_vector = predecode_fwd_subcacheline_vector_s3;

    l15_noc3encoder_req_homeid = 0;
    // `ifdef NO_RTL_CSM
    //     noc1_homeid_source_s3 = `L15_HOMEID_SRC_CSM_MODULE;
    // `endif
    case (noc3_homeid_source_s3)
        2'd2:
            l15_noc3encoder_req_homeid = noc2_src_homeid_s3[(14+8+8)-1:0];
        2'd3:
            





                // otherwise use the homeid table
                l15_noc3encoder_req_homeid = expanded_hmt_homeid_s3;
            
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

//==================================================================================================
//  Filename      : l15_priority_encoder.v
//  Created On    : 2014-07-09
//  Revision      :
//  Author        : Yaosheng Fu
//  Company       : Princeton University
//  Email         : yfu@princeton.edu
//
//  Description   : The priority encoder for pipeline1 in the L15 cache
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









































































































































































































































































































































































































































































































































































































module l15_priority_encoder_1(
    input wire [1:0] data_in,
    output wire [0:0] data_out,
    output wire [1:0] data_out_mask,
    output wire nonzero_out
);

assign data_out = data_in[0] ? 1'b0 : 1'b1;
assign data_out_mask = data_in[0] ? 2'b10 : 2'b01;
assign nonzero_out = | (data_in[1:0]);
endmodule

module l15_priority_encoder_2(
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

l15_priority_encoder_1 encoder_high_1 (.data_in(data_in[3:2]), .data_out(data_high), .data_out_mask(data_high_mask), .nonzero_out(nonzero_high));

l15_priority_encoder_1 encoder_low_1(.data_in(data_in[1:0]), .data_out(data_low), .data_out_mask(data_low_mask), .nonzero_out(nonzero_low));

assign data_out = nonzero_low ? {1'b0, data_low} : {1'b1, data_high};

assign data_out_mask = nonzero_low ? {{2{1'b1}}, data_low_mask} : {data_high_mask,{2{1'b1}}};

assign nonzero_out = nonzero_low | nonzero_high;
endmodule

module l15_priority_encoder_3(
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

l15_priority_encoder_2 encoder_high_2 (.data_in(data_in[7:4]), .data_out(data_high), .data_out_mask(data_high_mask), .nonzero_out(nonzero_high));

l15_priority_encoder_2 encoder_low_2(.data_in(data_in[3:0]), .data_out(data_low), .data_out_mask(data_low_mask), .nonzero_out(nonzero_low));

assign data_out = nonzero_low ? {1'b0, data_low} : {1'b1, data_high};

assign data_out_mask = nonzero_low ? {{4{1'b1}}, data_low_mask} : {data_high_mask,{4{1'b1}}};

assign nonzero_out = nonzero_low | nonzero_high;
endmodule

module l15_priority_encoder_4(
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

l15_priority_encoder_3 encoder_high_3 (.data_in(data_in[15:8]), .data_out(data_high), .data_out_mask(data_high_mask), .nonzero_out(nonzero_high));

l15_priority_encoder_3 encoder_low_3(.data_in(data_in[7:0]), .data_out(data_low), .data_out_mask(data_low_mask), .nonzero_out(nonzero_low));

assign data_out = nonzero_low ? {1'b0, data_low} : {1'b1, data_high};

assign data_out_mask = nonzero_low ? {{8{1'b1}}, data_low_mask} : {data_high_mask,{8{1'b1}}};

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
//  Filename      : noc1buffer.v
//  Created On    : 2014-02-05 20:06:27
//  Last Modified : 2018-11-14 19:21:02
//  Revision      :
//  Author        : Tri Nguyen
//  Company       : Princeton University
//  Email         : trin@princeton.edu
//
//  Description   :
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





















































































































































































































































































































































































































































































































































































































































































































































































module noc1buffer(
   input wire clk,
   input wire rst_n,
   input wire [63:0] l15_noc1buffer_req_data_0,
   input wire [63:0] l15_noc1buffer_req_data_1,
   input wire l15_noc1buffer_req_val,
   input wire [5-1:0] l15_noc1buffer_req_type,
   input wire [2-1:0] l15_noc1buffer_req_mshrid,
   input wire [0:0] l15_noc1buffer_req_threadid,
   input wire [39:0] l15_noc1buffer_req_address,
   input wire l15_noc1buffer_req_non_cacheable,
   input wire [3-1:0] l15_noc1buffer_req_size,
   input wire l15_noc1buffer_req_prefetch,
   // input wire l15_noc1buffer_req_blkstore,
   // input wire l15_noc1buffer_req_blkinitstore,
   input wire [3-1:0] l15_noc1buffer_req_csm_ticket,
   input wire [(14+8+8)-1:0] l15_noc1buffer_req_homeid,
   input wire l15_noc1buffer_req_homeid_val,
   input wire [33-1:0] l15_noc1buffer_req_csm_data,

   input wire noc1encoder_noc1buffer_req_ack,

   // csm interface
   input wire [(14+8+8)-1:0] csm_l15_read_res_data,
   input wire csm_l15_read_res_val,


   output reg [63:0] noc1buffer_noc1encoder_req_data_0,
   output reg [63:0] noc1buffer_noc1encoder_req_data_1,
   output reg noc1buffer_noc1encoder_req_val,
   output reg [5-1:0] noc1buffer_noc1encoder_req_type,
   output reg [2-1:0] noc1buffer_noc1encoder_req_mshrid,
   output reg [0:0] noc1buffer_noc1encoder_req_threadid,
   output reg [39:0] noc1buffer_noc1encoder_req_address,
   output reg noc1buffer_noc1encoder_req_non_cacheable,
   output reg [3-1:0] noc1buffer_noc1encoder_req_size,
   output reg noc1buffer_noc1encoder_req_prefetch,
   // output reg noc1buffer_noc1encoder_req_blkstore,
   // output reg noc1buffer_noc1encoder_req_blkinitstore,
   output reg [(14+8+8)-1:0] noc1buffer_noc1encoder_req_homeid,
   output reg [10-1:0] noc1buffer_noc1encoder_req_csm_sdid,
   output reg [6-1:0] noc1buffer_noc1encoder_req_csm_lsid,

   // csm interface
   output reg [3-1:0] l15_csm_read_ticket,
   output reg [3-1:0] l15_csm_clear_ticket,
   output reg l15_csm_clear_ticket_val,

   // output to mshrid when we have the csm
   output reg noc1buffer_mshr_homeid_write_val_s4,
   output reg [2-1:0] noc1buffer_mshr_homeid_write_mshrid_s4,
   output reg [0:0] noc1buffer_mshr_homeid_write_threadid_s4,
   output reg [(14+8+8)-1:0] noc1buffer_mshr_homeid_write_data_s4,

   // output reg noc1buffer_l15_req_ack,
   output reg noc1buffer_l15_req_sent,
   output reg [2-1:0] noc1buffer_l15_req_data_sent

);

reg [0 + 5 - 1 + 1 + 2 - 1 + 1 + 1 - 1 + 1 + 40 - 1 + 1 + 1 - 1 + 1 + 3 - 1 + 1 + 1 - 1 + 1 + 1 - 1 + 1 + 3 - 1 + 1 + (14+8+8) - 1 + 1 + 1 - 1 + 1 + 10 - 1 + 1 + 6 - 1 + 1-1:0] command_buffer [0:8-1];
reg [63:0] data_buffer [0:2-1];
reg [0 + 5 - 1 + 1 + 2 - 1 + 1 + 1 - 1 + 1 + 40 - 1 + 1 + 1 - 1 + 1 + 3 - 1 + 1 + 1 - 1 + 1 + 1 - 1 + 1 + 3 - 1 + 1 + (14+8+8) - 1 + 1 + 1 - 1 + 1 + 10 - 1 + 1 + 6 - 1 + 1-1:0] command_buffer_next [0:8-1];
reg [63:0] data_buffer_next [0:2-1];

reg command_buffer_val [0:8-1];
reg command_buffer_val_next [0:8-1];

reg [3-1:0] command_wrindex;
reg [3-1:0] command_wrindex_next;
reg [3-1:0] command_rdindex;
reg [3-1:0] command_rdindex_next;
reg [3-1:0] command_rdindex_plus1;
reg [1-1:0] data_wrindex;
reg [1-1:0] data_wrindex_next;
reg [1-1:0] data_wrindex_plus_1;
reg [1-1:0] data_wrindex_plus_2;
reg [1-1:0] data_rdindex;
reg [1-1:0] data_rdindex_plus1;

always @ (posedge clk)
begin
   if (!rst_n)
   begin
      command_buffer[0] <= 0;
command_buffer_val[0] <= 0;
command_buffer[1] <= 0;
command_buffer_val[1] <= 0;
command_buffer[2] <= 0;
command_buffer_val[2] <= 0;
command_buffer[3] <= 0;
command_buffer_val[3] <= 0;
command_buffer[4] <= 0;
command_buffer_val[4] <= 0;
command_buffer[5] <= 0;
command_buffer_val[5] <= 0;
command_buffer[6] <= 0;
command_buffer_val[6] <= 0;
command_buffer[7] <= 0;
command_buffer_val[7] <= 0;
data_buffer[0] <= 0;
data_buffer[1] <= 0;

      data_wrindex <= 0;
      command_wrindex <= 0;
      command_rdindex <= 0;
   end
   else
   begin
      // for (i = 0; i < `NOC1_BUFFER_NUM_SLOTS; i = i + 1)
      // begin
      //     command_buffer[i] <= command_buffer_next[i];
      //     command_buffer_val[i] <= command_buffer_val_next[i];
      // end
      // for (i = 0; i < `NOC1_BUFFER_NUM_DATA_SLOTS; i = i + 1)
      // begin
      //     data_buffer[i] <= data_buffer_next[i];
      // end
      command_buffer[0] <= command_buffer_next[0];
command_buffer_val[0] <= command_buffer_val_next[0];
command_buffer[1] <= command_buffer_next[1];
command_buffer_val[1] <= command_buffer_val_next[1];
command_buffer[2] <= command_buffer_next[2];
command_buffer_val[2] <= command_buffer_val_next[2];
command_buffer[3] <= command_buffer_next[3];
command_buffer_val[3] <= command_buffer_val_next[3];
command_buffer[4] <= command_buffer_next[4];
command_buffer_val[4] <= command_buffer_val_next[4];
command_buffer[5] <= command_buffer_next[5];
command_buffer_val[5] <= command_buffer_val_next[5];
command_buffer[6] <= command_buffer_next[6];
command_buffer_val[6] <= command_buffer_val_next[6];
command_buffer[7] <= command_buffer_next[7];
command_buffer_val[7] <= command_buffer_val_next[7];
data_buffer[0] <= data_buffer_next[0];
data_buffer[1] <= data_buffer_next[1];

      data_wrindex <= data_wrindex_next;
      command_wrindex <= command_wrindex_next;
      command_rdindex <= command_rdindex_next;
   end
end

// Mostly related to writes
always @ *
begin
   command_buffer_next[0] = command_buffer[0];
command_buffer_next[1] = command_buffer[1];
command_buffer_next[2] = command_buffer[2];
command_buffer_next[3] = command_buffer[3];
command_buffer_next[4] = command_buffer[4];
command_buffer_next[5] = command_buffer[5];
command_buffer_next[6] = command_buffer[6];
command_buffer_next[7] = command_buffer[7];
data_buffer_next[0] = data_buffer[0];
data_buffer_next[1] = data_buffer[1];


   command_wrindex_next = command_wrindex;
   data_wrindex_next = data_wrindex;
   data_wrindex_plus_1 = data_wrindex + 1;
   data_wrindex_plus_2 = data_wrindex + 2;

   if (l15_noc1buffer_req_val)
   begin
      command_buffer_next[command_wrindex][0 + 5 - 1:0] = l15_noc1buffer_req_type;
      command_buffer_next[command_wrindex][0 + 5 - 1 + 1 + 2 - 1:0 + 5 - 1 + 1] = l15_noc1buffer_req_mshrid;
      command_buffer_next[command_wrindex][0 + 5 - 1 + 1 + 2 - 1 + 1 + 1 - 1:0 + 5 - 1 + 1 + 2 - 1 + 1] = l15_noc1buffer_req_threadid;
      command_buffer_next[command_wrindex][0 + 5 - 1 + 1 + 2 - 1 + 1 + 1 - 1 + 1 + 40 - 1:0 + 5 - 1 + 1 + 2 - 1 + 1 + 1 - 1 + 1] = l15_noc1buffer_req_address;
      command_buffer_next[command_wrindex][0 + 5 - 1 + 1 + 2 - 1 + 1 + 1 - 1 + 1 + 40 - 1 + 1 + 1 - 1:0 + 5 - 1 + 1 + 2 - 1 + 1 + 1 - 1 + 1 + 40 - 1 + 1] = l15_noc1buffer_req_non_cacheable;
      command_buffer_next[command_wrindex][0 + 5 - 1 + 1 + 2 - 1 + 1 + 1 - 1 + 1 + 40 - 1 + 1 + 1 - 1 + 1 + 3 - 1:0 + 5 - 1 + 1 + 2 - 1 + 1 + 1 - 1 + 1 + 40 - 1 + 1 + 1 - 1 + 1] = l15_noc1buffer_req_size;
      command_buffer_next[command_wrindex][0 + 5 - 1 + 1 + 2 - 1 + 1 + 1 - 1 + 1 + 40 - 1 + 1 + 1 - 1 + 1 + 3 - 1 + 1 + 1 - 1:0 + 5 - 1 + 1 + 2 - 1 + 1 + 1 - 1 + 1 + 40 - 1 + 1 + 1 - 1 + 1 + 3 - 1 + 1] = l15_noc1buffer_req_prefetch;
      // command_buffer_next[command_wrindex][`L15_NOC1BUFFER_BLKSTORE] = l15_noc1buffer_req_blkstore;
      // command_buffer_next[command_wrindex][`L15_NOC1BUFFER_BLKINITSTORE] = l15_noc1buffer_req_blkinitstore;
      command_buffer_next[command_wrindex][0 + 5 - 1 + 1 + 2 - 1 + 1 + 1 - 1 + 1 + 40 - 1 + 1 + 1 - 1 + 1 + 3 - 1 + 1 + 1 - 1 + 1 + 1 - 1:0 + 5 - 1 + 1 + 2 - 1 + 1 + 1 - 1 + 1 + 40 - 1 + 1 + 1 - 1 + 1 + 3 - 1 + 1 + 1 - 1 + 1] = data_wrindex;
      command_buffer_next[command_wrindex][0 + 5 - 1 + 1 + 2 - 1 + 1 + 1 - 1 + 1 + 40 - 1 + 1 + 1 - 1 + 1 + 3 - 1 + 1 + 1 - 1 + 1 + 1 - 1 + 1 + 3 - 1:0 + 5 - 1 + 1 + 2 - 1 + 1 + 1 - 1 + 1 + 40 - 1 + 1 + 1 - 1 + 1 + 3 - 1 + 1 + 1 - 1 + 1 + 1 - 1 + 1] = l15_noc1buffer_req_csm_ticket;
      command_buffer_next[command_wrindex][0 + 5 - 1 + 1 + 2 - 1 + 1 + 1 - 1 + 1 + 40 - 1 + 1 + 1 - 1 + 1 + 3 - 1 + 1 + 1 - 1 + 1 + 1 - 1 + 1 + 3 - 1 + 1 + (14+8+8) - 1:0 + 5 - 1 + 1 + 2 - 1 + 1 + 1 - 1 + 1 + 40 - 1 + 1 + 1 - 1 + 1 + 3 - 1 + 1 + 1 - 1 + 1 + 1 - 1 + 1 + 3 - 1 + 1] = l15_noc1buffer_req_homeid;
      command_buffer_next[command_wrindex][0 + 5 - 1 + 1 + 2 - 1 + 1 + 1 - 1 + 1 + 40 - 1 + 1 + 1 - 1 + 1 + 3 - 1 + 1 + 1 - 1 + 1 + 1 - 1 + 1 + 3 - 1 + 1 + (14+8+8) - 1 + 1 + 1 - 1:0 + 5 - 1 + 1 + 2 - 1 + 1 + 1 - 1 + 1 + 40 - 1 + 1 + 1 - 1 + 1 + 3 - 1 + 1 + 1 - 1 + 1 + 1 - 1 + 1 + 3 - 1 + 1 + (14+8+8) - 1 + 1] = l15_noc1buffer_req_homeid_val;
      command_buffer_next[command_wrindex][0 + 5 - 1 + 1 + 2 - 1 + 1 + 1 - 1 + 1 + 40 - 1 + 1 + 1 - 1 + 1 + 3 - 1 + 1 + 1 - 1 + 1 + 1 - 1 + 1 + 3 - 1 + 1 + (14+8+8) - 1 + 1 + 1 - 1 + 1 + 10 - 1:0 + 5 - 1 + 1 + 2 - 1 + 1 + 1 - 1 + 1 + 40 - 1 + 1 + 1 - 1 + 1 + 3 - 1 + 1 + 1 - 1 + 1 + 1 - 1 + 1 + 3 - 1 + 1 + (14+8+8) - 1 + 1 + 1 - 1 + 1] = l15_noc1buffer_req_csm_data[15:6];
      command_buffer_next[command_wrindex][0 + 5 - 1 + 1 + 2 - 1 + 1 + 1 - 1 + 1 + 40 - 1 + 1 + 1 - 1 + 1 + 3 - 1 + 1 + 1 - 1 + 1 + 1 - 1 + 1 + 3 - 1 + 1 + (14+8+8) - 1 + 1 + 1 - 1 + 1 + 10 - 1 + 1 + 6 - 1:0 + 5 - 1 + 1 + 2 - 1 + 1 + 1 - 1 + 1 + 40 - 1 + 1 + 1 - 1 + 1 + 3 - 1 + 1 + 1 - 1 + 1 + 1 - 1 + 1 + 3 - 1 + 1 + (14+8+8) - 1 + 1 + 1 - 1 + 1 + 10 - 1 + 1] = l15_noc1buffer_req_csm_data[5:0];

      command_wrindex_next = command_wrindex + 1;

      if (l15_noc1buffer_req_type == 5'd7)
      begin
         data_buffer_next[data_wrindex] = l15_noc1buffer_req_data_0;
         data_buffer_next[data_wrindex_plus_1] = l15_noc1buffer_req_data_1;
         data_wrindex_next = data_wrindex_plus_2;
      end
      else if (l15_noc1buffer_req_type == 5'd8 ||
               l15_noc1buffer_req_type == 5'd10 ||
               l15_noc1buffer_req_type == 5'd11 ||
               l15_noc1buffer_req_type == 5'd12 ||
               l15_noc1buffer_req_type == 5'd13 ||
               l15_noc1buffer_req_type == 5'd14 ||
               l15_noc1buffer_req_type == 5'd15 ||
               l15_noc1buffer_req_type == 5'd16 ||
               l15_noc1buffer_req_type == 5'd17 ||
               l15_noc1buffer_req_type == 5'd9 ||
               l15_noc1buffer_req_type == 5'd4)
      begin
         data_buffer_next[data_wrindex] = l15_noc1buffer_req_data_0;
         data_wrindex_next = data_wrindex_plus_1;
      end
   end
end

// issue port to noc1encoder
reg [(14+8+8)-1:0] homeid;
reg homeid_val;
always @ *
begin
   // noc1buffer_l15_req_ack = noc1encoder_noc1buffer_req_ack;    // deprecated as noc1 is non-blocking
                                                // from pipeline's perspective

   noc1buffer_noc1encoder_req_type = command_buffer[command_rdindex][0 + 5 - 1:0];
   noc1buffer_noc1encoder_req_mshrid = command_buffer[command_rdindex][0 + 5 - 1 + 1 + 2 - 1:0 + 5 - 1 + 1];
   noc1buffer_noc1encoder_req_threadid = command_buffer[command_rdindex][0 + 5 - 1 + 1 + 2 - 1 + 1 + 1 - 1:0 + 5 - 1 + 1 + 2 - 1 + 1];
   noc1buffer_noc1encoder_req_address = command_buffer[command_rdindex][0 + 5 - 1 + 1 + 2 - 1 + 1 + 1 - 1 + 1 + 40 - 1:0 + 5 - 1 + 1 + 2 - 1 + 1 + 1 - 1 + 1];
   noc1buffer_noc1encoder_req_non_cacheable = command_buffer[command_rdindex][0 + 5 - 1 + 1 + 2 - 1 + 1 + 1 - 1 + 1 + 40 - 1 + 1 + 1 - 1:0 + 5 - 1 + 1 + 2 - 1 + 1 + 1 - 1 + 1 + 40 - 1 + 1];
   noc1buffer_noc1encoder_req_size = command_buffer[command_rdindex][0 + 5 - 1 + 1 + 2 - 1 + 1 + 1 - 1 + 1 + 40 - 1 + 1 + 1 - 1 + 1 + 3 - 1:0 + 5 - 1 + 1 + 2 - 1 + 1 + 1 - 1 + 1 + 40 - 1 + 1 + 1 - 1 + 1];
   noc1buffer_noc1encoder_req_prefetch = command_buffer[command_rdindex][0 + 5 - 1 + 1 + 2 - 1 + 1 + 1 - 1 + 1 + 40 - 1 + 1 + 1 - 1 + 1 + 3 - 1 + 1 + 1 - 1:0 + 5 - 1 + 1 + 2 - 1 + 1 + 1 - 1 + 1 + 40 - 1 + 1 + 1 - 1 + 1 + 3 - 1 + 1];
   // noc1buffer_noc1encoder_req_blkstore = command_buffer[command_rdindex][`L15_NOC1BUFFER_BLKSTORE];
   // noc1buffer_noc1encoder_req_blkinitstore = command_buffer[command_rdindex][`L15_NOC1BUFFER_BLKINITSTORE];
   noc1buffer_noc1encoder_req_csm_sdid = command_buffer[command_rdindex][0 + 5 - 1 + 1 + 2 - 1 + 1 + 1 - 1 + 1 + 40 - 1 + 1 + 1 - 1 + 1 + 3 - 1 + 1 + 1 - 1 + 1 + 1 - 1 + 1 + 3 - 1 + 1 + (14+8+8) - 1 + 1 + 1 - 1 + 1 + 10 - 1:0 + 5 - 1 + 1 + 2 - 1 + 1 + 1 - 1 + 1 + 40 - 1 + 1 + 1 - 1 + 1 + 3 - 1 + 1 + 1 - 1 + 1 + 1 - 1 + 1 + 3 - 1 + 1 + (14+8+8) - 1 + 1 + 1 - 1 + 1];
   noc1buffer_noc1encoder_req_csm_lsid = command_buffer[command_rdindex][0 + 5 - 1 + 1 + 2 - 1 + 1 + 1 - 1 + 1 + 40 - 1 + 1 + 1 - 1 + 1 + 3 - 1 + 1 + 1 - 1 + 1 + 1 - 1 + 1 + 3 - 1 + 1 + (14+8+8) - 1 + 1 + 1 - 1 + 1 + 10 - 1 + 1 + 6 - 1:0 + 5 - 1 + 1 + 2 - 1 + 1 + 1 - 1 + 1 + 40 - 1 + 1 + 1 - 1 + 1 + 3 - 1 + 1 + 1 - 1 + 1 + 1 - 1 + 1 + 3 - 1 + 1 + (14+8+8) - 1 + 1 + 1 - 1 + 1 + 10 - 1 + 1];

   data_rdindex = command_buffer[command_rdindex][0 + 5 - 1 + 1 + 2 - 1 + 1 + 1 - 1 + 1 + 40 - 1 + 1 + 1 - 1 + 1 + 3 - 1 + 1 + 1 - 1 + 1 + 1 - 1:0 + 5 - 1 + 1 + 2 - 1 + 1 + 1 - 1 + 1 + 40 - 1 + 1 + 1 - 1 + 1 + 3 - 1 + 1 + 1 - 1 + 1];
   data_rdindex_plus1 = data_rdindex + 1;
   noc1buffer_noc1encoder_req_data_0 = data_buffer[data_rdindex];
   noc1buffer_noc1encoder_req_data_1 = data_buffer[data_rdindex_plus1];

   noc1buffer_noc1encoder_req_homeid = homeid;

   noc1buffer_noc1encoder_req_val = command_buffer_val[command_rdindex] && homeid_val;
end


// Tri: for now just issue FIFO. we can try OoO issuing later when CSM is verified and stable.
reg [(14+8+8)-1:0] cached_homeid;
reg cached_homeid_val;
reg [(14+8+8)-1:0] fetch_homeid;
reg fetch_homeid_val;
   // Note: cached value is obtained at s3. meant to speed issuing if ghid is cached in the csm module
   //  the normal value is from reading the csm module (in case the translation wasn't cached)
   //  However for this "blocking single issue" the normal value is just as fast as cached
// CSM and homeid
always @ *
begin
   cached_homeid_val = command_buffer[command_rdindex][0 + 5 - 1 + 1 + 2 - 1 + 1 + 1 - 1 + 1 + 40 - 1 + 1 + 1 - 1 + 1 + 3 - 1 + 1 + 1 - 1 + 1 + 1 - 1 + 1 + 3 - 1 + 1 + (14+8+8) - 1 + 1 + 1 - 1:0 + 5 - 1 + 1 + 2 - 1 + 1 + 1 - 1 + 1 + 40 - 1 + 1 + 1 - 1 + 1 + 3 - 1 + 1 + 1 - 1 + 1 + 1 - 1 + 1 + 3 - 1 + 1 + (14+8+8) - 1 + 1];
   cached_homeid = command_buffer[command_rdindex][0 + 5 - 1 + 1 + 2 - 1 + 1 + 1 - 1 + 1 + 40 - 1 + 1 + 1 - 1 + 1 + 3 - 1 + 1 + 1 - 1 + 1 + 1 - 1 + 1 + 3 - 1 + 1 + (14+8+8) - 1:0 + 5 - 1 + 1 + 2 - 1 + 1 + 1 - 1 + 1 + 40 - 1 + 1 + 1 - 1 + 1 + 3 - 1 + 1 + 1 - 1 + 1 + 1 - 1 + 1 + 3 - 1 + 1];
   fetch_homeid_val = csm_l15_read_res_val;
   fetch_homeid = csm_l15_read_res_data;

   homeid_val = cached_homeid_val | fetch_homeid_val;
   homeid = cached_homeid_val ? cached_homeid : fetch_homeid;

   // output to CSM module
   l15_csm_read_ticket = command_buffer[command_rdindex][0 + 5 - 1 + 1 + 2 - 1 + 1 + 1 - 1 + 1 + 40 - 1 + 1 + 1 - 1 + 1 + 3 - 1 + 1 + 1 - 1 + 1 + 1 - 1 + 1 + 3 - 1:0 + 5 - 1 + 1 + 2 - 1 + 1 + 1 - 1 + 1 + 40 - 1 + 1 + 1 - 1 + 1 + 3 - 1 + 1 + 1 - 1 + 1 + 1 - 1 + 1]; // read req
   l15_csm_clear_ticket = l15_csm_read_ticket;
   l15_csm_clear_ticket_val = noc1encoder_noc1buffer_req_ack; // clear when sent

   // output to MSHR module
   noc1buffer_mshr_homeid_write_val_s4 = (noc1buffer_noc1encoder_req_mshrid == 2'd2 || noc1buffer_noc1encoder_req_mshrid == 2'd3) &&
                                    noc1encoder_noc1buffer_req_ack;
   noc1buffer_mshr_homeid_write_mshrid_s4 = noc1buffer_noc1encoder_req_mshrid;
   noc1buffer_mshr_homeid_write_threadid_s4 = noc1buffer_noc1encoder_req_threadid;
   noc1buffer_mshr_homeid_write_data_s4 = homeid;
end



// handling valid array (and conflicts)
always @ *
begin
   
   if (l15_noc1buffer_req_val && (command_wrindex == 0))
      command_buffer_val_next[0] = 1'b1;
   else if (noc1encoder_noc1buffer_req_ack && (command_rdindex == 0))
      command_buffer_val_next[0] = 1'b0;
   else
      command_buffer_val_next[0] = command_buffer_val[0];
   

   if (l15_noc1buffer_req_val && (command_wrindex == 1))
      command_buffer_val_next[1] = 1'b1;
   else if (noc1encoder_noc1buffer_req_ack && (command_rdindex == 1))
      command_buffer_val_next[1] = 1'b0;
   else
      command_buffer_val_next[1] = command_buffer_val[1];
   

   if (l15_noc1buffer_req_val && (command_wrindex == 2))
      command_buffer_val_next[2] = 1'b1;
   else if (noc1encoder_noc1buffer_req_ack && (command_rdindex == 2))
      command_buffer_val_next[2] = 1'b0;
   else
      command_buffer_val_next[2] = command_buffer_val[2];
   

   if (l15_noc1buffer_req_val && (command_wrindex == 3))
      command_buffer_val_next[3] = 1'b1;
   else if (noc1encoder_noc1buffer_req_ack && (command_rdindex == 3))
      command_buffer_val_next[3] = 1'b0;
   else
      command_buffer_val_next[3] = command_buffer_val[3];
   

   if (l15_noc1buffer_req_val && (command_wrindex == 4))
      command_buffer_val_next[4] = 1'b1;
   else if (noc1encoder_noc1buffer_req_ack && (command_rdindex == 4))
      command_buffer_val_next[4] = 1'b0;
   else
      command_buffer_val_next[4] = command_buffer_val[4];
   

   if (l15_noc1buffer_req_val && (command_wrindex == 5))
      command_buffer_val_next[5] = 1'b1;
   else if (noc1encoder_noc1buffer_req_ack && (command_rdindex == 5))
      command_buffer_val_next[5] = 1'b0;
   else
      command_buffer_val_next[5] = command_buffer_val[5];
   

   if (l15_noc1buffer_req_val && (command_wrindex == 6))
      command_buffer_val_next[6] = 1'b1;
   else if (noc1encoder_noc1buffer_req_ack && (command_rdindex == 6))
      command_buffer_val_next[6] = 1'b0;
   else
      command_buffer_val_next[6] = command_buffer_val[6];
   

   if (l15_noc1buffer_req_val && (command_wrindex == 7))
      command_buffer_val_next[7] = 1'b1;
   else if (noc1encoder_noc1buffer_req_ack && (command_rdindex == 7))
      command_buffer_val_next[7] = 1'b0;
   else
      command_buffer_val_next[7] = command_buffer_val[7];
   

end

// data credit logic
always @ *
begin
   noc1buffer_l15_req_data_sent = 0;
   noc1buffer_l15_req_sent = noc1encoder_noc1buffer_req_ack;
   command_rdindex_plus1 = command_rdindex + 1;
   command_rdindex_next = command_rdindex;
   if (noc1encoder_noc1buffer_req_ack == 1'b1)
   begin
      command_rdindex_next = command_rdindex_plus1;
      case (noc1buffer_noc1encoder_req_type)
         5'd4,
         5'd8,
         5'd10,
         5'd11,
         5'd12,
         5'd13,
         5'd14,
         5'd15,
         5'd16,
         5'd17,
         5'd9:
         begin
            noc1buffer_l15_req_data_sent = 2'd1;
         end
         5'd7:
         begin
            noc1buffer_l15_req_data_sent = 2'd2;
         end
      endcase
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
//  Filename      : rf_l15_lrsc_flag.v
//  Created On    : 2018-11-08 18:14:58
//  Last Modified : 
//  Revision      :
//  Author        : Fei Gao
//  Company       : Princeton University
//  Email         : feig@princeton.edu
//
//  Description   :
//
//
//==================================================================================================
//rf_l15_lrsc_flag.v

//`timescale 1 ns / 10 ps
//`default_nettype none


// devices.xml

module rf_l15_lrsc_flag(
   input wire clk,
   input wire rst_n,

   input wire read_valid,
   input wire [((9-2))-1:0] read_index,

   input wire write_valid,
   input wire [((9-2))-1:0] write_index,
   input wire [3:0] write_mask,
   input wire [3:0] write_data,

   output wire [3:0] read_data
   );



// reg read_valid_f;
reg [((9-2))-1:0] read_index_f;
reg [((9-2))-1:0] write_index_f;
reg [3:0] write_data_f;
reg [3:0] write_mask_f;
reg write_valid_f;

reg [3:0] regfile [0:(512/4)-1];

always @ (posedge clk)
begin
   if (!rst_n)
   begin
      read_index_f <= 0;
   end
   else
   if (read_valid)
      read_index_f <= read_index;
   else
      read_index_f <= read_index_f;
end

// read port
assign read_data = regfile[read_index_f];

// Write port
always @ (posedge clk)
begin
   write_valid_f <= write_valid;
   if (write_valid)
   begin
      write_data_f <= write_data;
      write_index_f <= write_index;
      write_mask_f <= write_mask;
   end
end

always @ (posedge clk)
begin
   if (!rst_n)
   begin
      regfile[0] <= 4'b0;
regfile[1] <= 4'b0;
regfile[2] <= 4'b0;
regfile[3] <= 4'b0;
regfile[4] <= 4'b0;
regfile[5] <= 4'b0;
regfile[6] <= 4'b0;
regfile[7] <= 4'b0;
regfile[8] <= 4'b0;
regfile[9] <= 4'b0;
regfile[10] <= 4'b0;
regfile[11] <= 4'b0;
regfile[12] <= 4'b0;
regfile[13] <= 4'b0;
regfile[14] <= 4'b0;
regfile[15] <= 4'b0;
regfile[16] <= 4'b0;
regfile[17] <= 4'b0;
regfile[18] <= 4'b0;
regfile[19] <= 4'b0;
regfile[20] <= 4'b0;
regfile[21] <= 4'b0;
regfile[22] <= 4'b0;
regfile[23] <= 4'b0;
regfile[24] <= 4'b0;
regfile[25] <= 4'b0;
regfile[26] <= 4'b0;
regfile[27] <= 4'b0;
regfile[28] <= 4'b0;
regfile[29] <= 4'b0;
regfile[30] <= 4'b0;
regfile[31] <= 4'b0;
regfile[32] <= 4'b0;
regfile[33] <= 4'b0;
regfile[34] <= 4'b0;
regfile[35] <= 4'b0;
regfile[36] <= 4'b0;
regfile[37] <= 4'b0;
regfile[38] <= 4'b0;
regfile[39] <= 4'b0;
regfile[40] <= 4'b0;
regfile[41] <= 4'b0;
regfile[42] <= 4'b0;
regfile[43] <= 4'b0;
regfile[44] <= 4'b0;
regfile[45] <= 4'b0;
regfile[46] <= 4'b0;
regfile[47] <= 4'b0;
regfile[48] <= 4'b0;
regfile[49] <= 4'b0;
regfile[50] <= 4'b0;
regfile[51] <= 4'b0;
regfile[52] <= 4'b0;
regfile[53] <= 4'b0;
regfile[54] <= 4'b0;
regfile[55] <= 4'b0;
regfile[56] <= 4'b0;
regfile[57] <= 4'b0;
regfile[58] <= 4'b0;
regfile[59] <= 4'b0;
regfile[60] <= 4'b0;
regfile[61] <= 4'b0;
regfile[62] <= 4'b0;
regfile[63] <= 4'b0;
regfile[64] <= 4'b0;
regfile[65] <= 4'b0;
regfile[66] <= 4'b0;
regfile[67] <= 4'b0;
regfile[68] <= 4'b0;
regfile[69] <= 4'b0;
regfile[70] <= 4'b0;
regfile[71] <= 4'b0;
regfile[72] <= 4'b0;
regfile[73] <= 4'b0;
regfile[74] <= 4'b0;
regfile[75] <= 4'b0;
regfile[76] <= 4'b0;
regfile[77] <= 4'b0;
regfile[78] <= 4'b0;
regfile[79] <= 4'b0;
regfile[80] <= 4'b0;
regfile[81] <= 4'b0;
regfile[82] <= 4'b0;
regfile[83] <= 4'b0;
regfile[84] <= 4'b0;
regfile[85] <= 4'b0;
regfile[86] <= 4'b0;
regfile[87] <= 4'b0;
regfile[88] <= 4'b0;
regfile[89] <= 4'b0;
regfile[90] <= 4'b0;
regfile[91] <= 4'b0;
regfile[92] <= 4'b0;
regfile[93] <= 4'b0;
regfile[94] <= 4'b0;
regfile[95] <= 4'b0;
regfile[96] <= 4'b0;
regfile[97] <= 4'b0;
regfile[98] <= 4'b0;
regfile[99] <= 4'b0;
regfile[100] <= 4'b0;
regfile[101] <= 4'b0;
regfile[102] <= 4'b0;
regfile[103] <= 4'b0;
regfile[104] <= 4'b0;
regfile[105] <= 4'b0;
regfile[106] <= 4'b0;
regfile[107] <= 4'b0;
regfile[108] <= 4'b0;
regfile[109] <= 4'b0;
regfile[110] <= 4'b0;
regfile[111] <= 4'b0;
regfile[112] <= 4'b0;
regfile[113] <= 4'b0;
regfile[114] <= 4'b0;
regfile[115] <= 4'b0;
regfile[116] <= 4'b0;
regfile[117] <= 4'b0;
regfile[118] <= 4'b0;
regfile[119] <= 4'b0;
regfile[120] <= 4'b0;
regfile[121] <= 4'b0;
regfile[122] <= 4'b0;
regfile[123] <= 4'b0;
regfile[124] <= 4'b0;
regfile[125] <= 4'b0;
regfile[126] <= 4'b0;
regfile[127] <= 4'b0;

      // regfile <= 1024'b0;
   end
   else
   if (write_valid_f)
   begin
      // regfile[write_index] <= (write_data & write_mask) | (regfile[write_index] & ~write_mask);
      regfile[write_index_f] <= (write_data_f & write_mask_f) | (regfile[write_index_f] & ~write_mask_f);
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
//  Filename      : rf_l15_lruarray.v
//  Created On    : 2014-02-04 18:14:58
//  Last Modified : 2014-02-13 18:30:34
//  Revision      :
//  Author        : Tri Nguyen
//  Company       : Princeton University
//  Email         : trin@princeton.edu
//
//  Description   :
//
//
//==================================================================================================
//rf_l15_lruarray.v


// devices.xml

//`timescale 1 ns / 10 ps
//`default_nettype none
module rf_l15_lruarray(
   input wire clk,
   input wire rst_n,

   input wire read_valid,
   input wire [((9-2))-1:0] read_index,

   input wire write_valid,
   input wire [((9-2))-1:0] write_index,
   input wire [5:0] write_mask,
   input wire [5:0] write_data,

   output wire [5:0] read_data
   );



// reg read_valid_f;
reg [((9-2))-1:0] read_index_f;

reg [5:0] regfile [0:(512/4)-1];

always @ (posedge clk)
begin
   if (!rst_n)
   begin
      read_index_f <= 0;
   end
   else
   if (read_valid)
      read_index_f <= read_index;
   else
      read_index_f <= read_index_f;
end

// read port
assign read_data = regfile[read_index_f];

// Write port
always @ (posedge clk)
begin
   if (!rst_n)
   begin
      regfile[0] <= 6'b0;
regfile[1] <= 6'b0;
regfile[2] <= 6'b0;
regfile[3] <= 6'b0;
regfile[4] <= 6'b0;
regfile[5] <= 6'b0;
regfile[6] <= 6'b0;
regfile[7] <= 6'b0;
regfile[8] <= 6'b0;
regfile[9] <= 6'b0;
regfile[10] <= 6'b0;
regfile[11] <= 6'b0;
regfile[12] <= 6'b0;
regfile[13] <= 6'b0;
regfile[14] <= 6'b0;
regfile[15] <= 6'b0;
regfile[16] <= 6'b0;
regfile[17] <= 6'b0;
regfile[18] <= 6'b0;
regfile[19] <= 6'b0;
regfile[20] <= 6'b0;
regfile[21] <= 6'b0;
regfile[22] <= 6'b0;
regfile[23] <= 6'b0;
regfile[24] <= 6'b0;
regfile[25] <= 6'b0;
regfile[26] <= 6'b0;
regfile[27] <= 6'b0;
regfile[28] <= 6'b0;
regfile[29] <= 6'b0;
regfile[30] <= 6'b0;
regfile[31] <= 6'b0;
regfile[32] <= 6'b0;
regfile[33] <= 6'b0;
regfile[34] <= 6'b0;
regfile[35] <= 6'b0;
regfile[36] <= 6'b0;
regfile[37] <= 6'b0;
regfile[38] <= 6'b0;
regfile[39] <= 6'b0;
regfile[40] <= 6'b0;
regfile[41] <= 6'b0;
regfile[42] <= 6'b0;
regfile[43] <= 6'b0;
regfile[44] <= 6'b0;
regfile[45] <= 6'b0;
regfile[46] <= 6'b0;
regfile[47] <= 6'b0;
regfile[48] <= 6'b0;
regfile[49] <= 6'b0;
regfile[50] <= 6'b0;
regfile[51] <= 6'b0;
regfile[52] <= 6'b0;
regfile[53] <= 6'b0;
regfile[54] <= 6'b0;
regfile[55] <= 6'b0;
regfile[56] <= 6'b0;
regfile[57] <= 6'b0;
regfile[58] <= 6'b0;
regfile[59] <= 6'b0;
regfile[60] <= 6'b0;
regfile[61] <= 6'b0;
regfile[62] <= 6'b0;
regfile[63] <= 6'b0;
regfile[64] <= 6'b0;
regfile[65] <= 6'b0;
regfile[66] <= 6'b0;
regfile[67] <= 6'b0;
regfile[68] <= 6'b0;
regfile[69] <= 6'b0;
regfile[70] <= 6'b0;
regfile[71] <= 6'b0;
regfile[72] <= 6'b0;
regfile[73] <= 6'b0;
regfile[74] <= 6'b0;
regfile[75] <= 6'b0;
regfile[76] <= 6'b0;
regfile[77] <= 6'b0;
regfile[78] <= 6'b0;
regfile[79] <= 6'b0;
regfile[80] <= 6'b0;
regfile[81] <= 6'b0;
regfile[82] <= 6'b0;
regfile[83] <= 6'b0;
regfile[84] <= 6'b0;
regfile[85] <= 6'b0;
regfile[86] <= 6'b0;
regfile[87] <= 6'b0;
regfile[88] <= 6'b0;
regfile[89] <= 6'b0;
regfile[90] <= 6'b0;
regfile[91] <= 6'b0;
regfile[92] <= 6'b0;
regfile[93] <= 6'b0;
regfile[94] <= 6'b0;
regfile[95] <= 6'b0;
regfile[96] <= 6'b0;
regfile[97] <= 6'b0;
regfile[98] <= 6'b0;
regfile[99] <= 6'b0;
regfile[100] <= 6'b0;
regfile[101] <= 6'b0;
regfile[102] <= 6'b0;
regfile[103] <= 6'b0;
regfile[104] <= 6'b0;
regfile[105] <= 6'b0;
regfile[106] <= 6'b0;
regfile[107] <= 6'b0;
regfile[108] <= 6'b0;
regfile[109] <= 6'b0;
regfile[110] <= 6'b0;
regfile[111] <= 6'b0;
regfile[112] <= 6'b0;
regfile[113] <= 6'b0;
regfile[114] <= 6'b0;
regfile[115] <= 6'b0;
regfile[116] <= 6'b0;
regfile[117] <= 6'b0;
regfile[118] <= 6'b0;
regfile[119] <= 6'b0;
regfile[120] <= 6'b0;
regfile[121] <= 6'b0;
regfile[122] <= 6'b0;
regfile[123] <= 6'b0;
regfile[124] <= 6'b0;
regfile[125] <= 6'b0;
regfile[126] <= 6'b0;
regfile[127] <= 6'b0;

      // regfile <= 1024'b0;
   end
   else
   if (write_valid)
   begin
      regfile[write_index] <= (write_data & write_mask) | (regfile[write_index] & ~write_mask);
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
//  Filename      : rf_l15_mesi.v
//  Created On    : 2014-02-04 18:14:58
//  Last Modified : 2014-12-17 16:09:50
//  Revision      :
//  Author        : Tri Nguyen
//  Company       : Princeton University
//  Email         : trin@princeton.edu
//
//  Description   :
//
//
//==================================================================================================
//rf_l15_mesi.v

//`timescale 1 ns / 10 ps
//`default_nettype none

// 12/17 timing fix: move write to s3

// devices.xml

module rf_l15_mesi(
   input wire clk,
   input wire rst_n,

   input wire read_valid,
   input wire [((9-2))-1:0] read_index,

   input wire write_valid,
   input wire [((9-2))-1:0] write_index,
   input wire [7:0] write_mask,
   input wire [7:0] write_data,

   output wire [7:0] read_data
   );



// reg read_valid_f;
reg [((9-2))-1:0] read_index_f;
reg [((9-2))-1:0] write_index_f;
reg [7:0] write_data_f;
reg [7:0] write_mask_f;
reg write_valid_f;

reg [7:0] regfile [0:(512/4)-1];

always @ (posedge clk)
begin
   if (!rst_n)
   begin
      read_index_f <= 0;
   end
   else
   if (read_valid)
      read_index_f <= read_index;
   else
      read_index_f <= read_index_f;
end

// read port
assign read_data = regfile[read_index_f];

// Write port
always @ (posedge clk)
begin
   write_valid_f <= write_valid;
   if (write_valid)
   begin
      write_data_f <= write_data;
      write_index_f <= write_index;
      write_mask_f <= write_mask;
   end
end

always @ (posedge clk)
begin
   if (!rst_n)
   begin
      regfile[0] <= 8'b0;
regfile[1] <= 8'b0;
regfile[2] <= 8'b0;
regfile[3] <= 8'b0;
regfile[4] <= 8'b0;
regfile[5] <= 8'b0;
regfile[6] <= 8'b0;
regfile[7] <= 8'b0;
regfile[8] <= 8'b0;
regfile[9] <= 8'b0;
regfile[10] <= 8'b0;
regfile[11] <= 8'b0;
regfile[12] <= 8'b0;
regfile[13] <= 8'b0;
regfile[14] <= 8'b0;
regfile[15] <= 8'b0;
regfile[16] <= 8'b0;
regfile[17] <= 8'b0;
regfile[18] <= 8'b0;
regfile[19] <= 8'b0;
regfile[20] <= 8'b0;
regfile[21] <= 8'b0;
regfile[22] <= 8'b0;
regfile[23] <= 8'b0;
regfile[24] <= 8'b0;
regfile[25] <= 8'b0;
regfile[26] <= 8'b0;
regfile[27] <= 8'b0;
regfile[28] <= 8'b0;
regfile[29] <= 8'b0;
regfile[30] <= 8'b0;
regfile[31] <= 8'b0;
regfile[32] <= 8'b0;
regfile[33] <= 8'b0;
regfile[34] <= 8'b0;
regfile[35] <= 8'b0;
regfile[36] <= 8'b0;
regfile[37] <= 8'b0;
regfile[38] <= 8'b0;
regfile[39] <= 8'b0;
regfile[40] <= 8'b0;
regfile[41] <= 8'b0;
regfile[42] <= 8'b0;
regfile[43] <= 8'b0;
regfile[44] <= 8'b0;
regfile[45] <= 8'b0;
regfile[46] <= 8'b0;
regfile[47] <= 8'b0;
regfile[48] <= 8'b0;
regfile[49] <= 8'b0;
regfile[50] <= 8'b0;
regfile[51] <= 8'b0;
regfile[52] <= 8'b0;
regfile[53] <= 8'b0;
regfile[54] <= 8'b0;
regfile[55] <= 8'b0;
regfile[56] <= 8'b0;
regfile[57] <= 8'b0;
regfile[58] <= 8'b0;
regfile[59] <= 8'b0;
regfile[60] <= 8'b0;
regfile[61] <= 8'b0;
regfile[62] <= 8'b0;
regfile[63] <= 8'b0;
regfile[64] <= 8'b0;
regfile[65] <= 8'b0;
regfile[66] <= 8'b0;
regfile[67] <= 8'b0;
regfile[68] <= 8'b0;
regfile[69] <= 8'b0;
regfile[70] <= 8'b0;
regfile[71] <= 8'b0;
regfile[72] <= 8'b0;
regfile[73] <= 8'b0;
regfile[74] <= 8'b0;
regfile[75] <= 8'b0;
regfile[76] <= 8'b0;
regfile[77] <= 8'b0;
regfile[78] <= 8'b0;
regfile[79] <= 8'b0;
regfile[80] <= 8'b0;
regfile[81] <= 8'b0;
regfile[82] <= 8'b0;
regfile[83] <= 8'b0;
regfile[84] <= 8'b0;
regfile[85] <= 8'b0;
regfile[86] <= 8'b0;
regfile[87] <= 8'b0;
regfile[88] <= 8'b0;
regfile[89] <= 8'b0;
regfile[90] <= 8'b0;
regfile[91] <= 8'b0;
regfile[92] <= 8'b0;
regfile[93] <= 8'b0;
regfile[94] <= 8'b0;
regfile[95] <= 8'b0;
regfile[96] <= 8'b0;
regfile[97] <= 8'b0;
regfile[98] <= 8'b0;
regfile[99] <= 8'b0;
regfile[100] <= 8'b0;
regfile[101] <= 8'b0;
regfile[102] <= 8'b0;
regfile[103] <= 8'b0;
regfile[104] <= 8'b0;
regfile[105] <= 8'b0;
regfile[106] <= 8'b0;
regfile[107] <= 8'b0;
regfile[108] <= 8'b0;
regfile[109] <= 8'b0;
regfile[110] <= 8'b0;
regfile[111] <= 8'b0;
regfile[112] <= 8'b0;
regfile[113] <= 8'b0;
regfile[114] <= 8'b0;
regfile[115] <= 8'b0;
regfile[116] <= 8'b0;
regfile[117] <= 8'b0;
regfile[118] <= 8'b0;
regfile[119] <= 8'b0;
regfile[120] <= 8'b0;
regfile[121] <= 8'b0;
regfile[122] <= 8'b0;
regfile[123] <= 8'b0;
regfile[124] <= 8'b0;
regfile[125] <= 8'b0;
regfile[126] <= 8'b0;
regfile[127] <= 8'b0;

      // regfile <= 1024'b0;
   end
   else
   if (write_valid_f)
   begin
      // regfile[write_index] <= (write_data & write_mask) | (regfile[write_index] & ~write_mask);
      regfile[write_index_f] <= (write_data_f & write_mask_f) | (regfile[write_index_f] & ~write_mask_f);
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
//  Filename      : rf_l15_wmt.v
//  Created On    : 2014-02-04 18:14:58
//  Last Modified : 2014-12-18 17:10:02
//  Revision      :
//  Author        : Tri Nguyen
//  Company       : Princeton University
//  Email         : trin@princeton.edu
//
//  Description   :
//
//
//==================================================================================================
//rf_l15_wmt.v

// trin timing fix 12/16: move read s3 to s2
// timing 12/17: move write to s2 to s3

// devices.xml

module rf_l15_wmt(
   input wire clk,
   input wire rst_n,

   input wire read_valid,
   input wire [6:0] read_index,

   input wire write_valid,
   input wire [6:0] write_index,
   input wire [2*((2+0)+1)-1:0] write_mask,
   input wire [2*((2+0)+1)-1:0] write_data,

   output wire [2*((2+0)+1)-1:0] read_data
   );


// reg [`L15_WMT_MASK] data_out_f;

// reg [`L15_WMT_MASK] regfile [0:127];

// always @ (posedge clk)
// begin
//    if (read_valid)
//       data_out_f <= regfile[read_index];
// end


// assign read_data = data_out_f;


reg [2*((2+0)+1)-1:0] data_out_f;
reg [6:0] write_index_f;
reg [2*((2+0)+1)-1:0] write_data_f;
reg [2*((2+0)+1)-1:0] write_mask_f;
reg write_valid_f;

reg [2*((2+0)+1)-1:0] regfile [0:(256/2)-1];

always @ (posedge clk)
begin
   if (read_valid)
      data_out_f <= regfile[read_index];
end


assign read_data = data_out_f;

// Write port

always @ (posedge clk)
begin
   write_valid_f <= write_valid;
   if (write_valid)
   begin
      write_data_f <= write_data;
      write_index_f <= write_index;
      write_mask_f <= write_mask;
   end
end

always @ (posedge clk)
begin
   if (!rst_n)
   begin
      regfile[0][(0+1)*((2+0)+1)-1] <= 1'b0;
regfile[0][(1+1)*((2+0)+1)-1] <= 1'b0;
regfile[1][(0+1)*((2+0)+1)-1] <= 1'b0;
regfile[1][(1+1)*((2+0)+1)-1] <= 1'b0;
regfile[2][(0+1)*((2+0)+1)-1] <= 1'b0;
regfile[2][(1+1)*((2+0)+1)-1] <= 1'b0;
regfile[3][(0+1)*((2+0)+1)-1] <= 1'b0;
regfile[3][(1+1)*((2+0)+1)-1] <= 1'b0;
regfile[4][(0+1)*((2+0)+1)-1] <= 1'b0;
regfile[4][(1+1)*((2+0)+1)-1] <= 1'b0;
regfile[5][(0+1)*((2+0)+1)-1] <= 1'b0;
regfile[5][(1+1)*((2+0)+1)-1] <= 1'b0;
regfile[6][(0+1)*((2+0)+1)-1] <= 1'b0;
regfile[6][(1+1)*((2+0)+1)-1] <= 1'b0;
regfile[7][(0+1)*((2+0)+1)-1] <= 1'b0;
regfile[7][(1+1)*((2+0)+1)-1] <= 1'b0;
regfile[8][(0+1)*((2+0)+1)-1] <= 1'b0;
regfile[8][(1+1)*((2+0)+1)-1] <= 1'b0;
regfile[9][(0+1)*((2+0)+1)-1] <= 1'b0;
regfile[9][(1+1)*((2+0)+1)-1] <= 1'b0;
regfile[10][(0+1)*((2+0)+1)-1] <= 1'b0;
regfile[10][(1+1)*((2+0)+1)-1] <= 1'b0;
regfile[11][(0+1)*((2+0)+1)-1] <= 1'b0;
regfile[11][(1+1)*((2+0)+1)-1] <= 1'b0;
regfile[12][(0+1)*((2+0)+1)-1] <= 1'b0;
regfile[12][(1+1)*((2+0)+1)-1] <= 1'b0;
regfile[13][(0+1)*((2+0)+1)-1] <= 1'b0;
regfile[13][(1+1)*((2+0)+1)-1] <= 1'b0;
regfile[14][(0+1)*((2+0)+1)-1] <= 1'b0;
regfile[14][(1+1)*((2+0)+1)-1] <= 1'b0;
regfile[15][(0+1)*((2+0)+1)-1] <= 1'b0;
regfile[15][(1+1)*((2+0)+1)-1] <= 1'b0;
regfile[16][(0+1)*((2+0)+1)-1] <= 1'b0;
regfile[16][(1+1)*((2+0)+1)-1] <= 1'b0;
regfile[17][(0+1)*((2+0)+1)-1] <= 1'b0;
regfile[17][(1+1)*((2+0)+1)-1] <= 1'b0;
regfile[18][(0+1)*((2+0)+1)-1] <= 1'b0;
regfile[18][(1+1)*((2+0)+1)-1] <= 1'b0;
regfile[19][(0+1)*((2+0)+1)-1] <= 1'b0;
regfile[19][(1+1)*((2+0)+1)-1] <= 1'b0;
regfile[20][(0+1)*((2+0)+1)-1] <= 1'b0;
regfile[20][(1+1)*((2+0)+1)-1] <= 1'b0;
regfile[21][(0+1)*((2+0)+1)-1] <= 1'b0;
regfile[21][(1+1)*((2+0)+1)-1] <= 1'b0;
regfile[22][(0+1)*((2+0)+1)-1] <= 1'b0;
regfile[22][(1+1)*((2+0)+1)-1] <= 1'b0;
regfile[23][(0+1)*((2+0)+1)-1] <= 1'b0;
regfile[23][(1+1)*((2+0)+1)-1] <= 1'b0;
regfile[24][(0+1)*((2+0)+1)-1] <= 1'b0;
regfile[24][(1+1)*((2+0)+1)-1] <= 1'b0;
regfile[25][(0+1)*((2+0)+1)-1] <= 1'b0;
regfile[25][(1+1)*((2+0)+1)-1] <= 1'b0;
regfile[26][(0+1)*((2+0)+1)-1] <= 1'b0;
regfile[26][(1+1)*((2+0)+1)-1] <= 1'b0;
regfile[27][(0+1)*((2+0)+1)-1] <= 1'b0;
regfile[27][(1+1)*((2+0)+1)-1] <= 1'b0;
regfile[28][(0+1)*((2+0)+1)-1] <= 1'b0;
regfile[28][(1+1)*((2+0)+1)-1] <= 1'b0;
regfile[29][(0+1)*((2+0)+1)-1] <= 1'b0;
regfile[29][(1+1)*((2+0)+1)-1] <= 1'b0;
regfile[30][(0+1)*((2+0)+1)-1] <= 1'b0;
regfile[30][(1+1)*((2+0)+1)-1] <= 1'b0;
regfile[31][(0+1)*((2+0)+1)-1] <= 1'b0;
regfile[31][(1+1)*((2+0)+1)-1] <= 1'b0;
regfile[32][(0+1)*((2+0)+1)-1] <= 1'b0;
regfile[32][(1+1)*((2+0)+1)-1] <= 1'b0;
regfile[33][(0+1)*((2+0)+1)-1] <= 1'b0;
regfile[33][(1+1)*((2+0)+1)-1] <= 1'b0;
regfile[34][(0+1)*((2+0)+1)-1] <= 1'b0;
regfile[34][(1+1)*((2+0)+1)-1] <= 1'b0;
regfile[35][(0+1)*((2+0)+1)-1] <= 1'b0;
regfile[35][(1+1)*((2+0)+1)-1] <= 1'b0;
regfile[36][(0+1)*((2+0)+1)-1] <= 1'b0;
regfile[36][(1+1)*((2+0)+1)-1] <= 1'b0;
regfile[37][(0+1)*((2+0)+1)-1] <= 1'b0;
regfile[37][(1+1)*((2+0)+1)-1] <= 1'b0;
regfile[38][(0+1)*((2+0)+1)-1] <= 1'b0;
regfile[38][(1+1)*((2+0)+1)-1] <= 1'b0;
regfile[39][(0+1)*((2+0)+1)-1] <= 1'b0;
regfile[39][(1+1)*((2+0)+1)-1] <= 1'b0;
regfile[40][(0+1)*((2+0)+1)-1] <= 1'b0;
regfile[40][(1+1)*((2+0)+1)-1] <= 1'b0;
regfile[41][(0+1)*((2+0)+1)-1] <= 1'b0;
regfile[41][(1+1)*((2+0)+1)-1] <= 1'b0;
regfile[42][(0+1)*((2+0)+1)-1] <= 1'b0;
regfile[42][(1+1)*((2+0)+1)-1] <= 1'b0;
regfile[43][(0+1)*((2+0)+1)-1] <= 1'b0;
regfile[43][(1+1)*((2+0)+1)-1] <= 1'b0;
regfile[44][(0+1)*((2+0)+1)-1] <= 1'b0;
regfile[44][(1+1)*((2+0)+1)-1] <= 1'b0;
regfile[45][(0+1)*((2+0)+1)-1] <= 1'b0;
regfile[45][(1+1)*((2+0)+1)-1] <= 1'b0;
regfile[46][(0+1)*((2+0)+1)-1] <= 1'b0;
regfile[46][(1+1)*((2+0)+1)-1] <= 1'b0;
regfile[47][(0+1)*((2+0)+1)-1] <= 1'b0;
regfile[47][(1+1)*((2+0)+1)-1] <= 1'b0;
regfile[48][(0+1)*((2+0)+1)-1] <= 1'b0;
regfile[48][(1+1)*((2+0)+1)-1] <= 1'b0;
regfile[49][(0+1)*((2+0)+1)-1] <= 1'b0;
regfile[49][(1+1)*((2+0)+1)-1] <= 1'b0;
regfile[50][(0+1)*((2+0)+1)-1] <= 1'b0;
regfile[50][(1+1)*((2+0)+1)-1] <= 1'b0;
regfile[51][(0+1)*((2+0)+1)-1] <= 1'b0;
regfile[51][(1+1)*((2+0)+1)-1] <= 1'b0;
regfile[52][(0+1)*((2+0)+1)-1] <= 1'b0;
regfile[52][(1+1)*((2+0)+1)-1] <= 1'b0;
regfile[53][(0+1)*((2+0)+1)-1] <= 1'b0;
regfile[53][(1+1)*((2+0)+1)-1] <= 1'b0;
regfile[54][(0+1)*((2+0)+1)-1] <= 1'b0;
regfile[54][(1+1)*((2+0)+1)-1] <= 1'b0;
regfile[55][(0+1)*((2+0)+1)-1] <= 1'b0;
regfile[55][(1+1)*((2+0)+1)-1] <= 1'b0;
regfile[56][(0+1)*((2+0)+1)-1] <= 1'b0;
regfile[56][(1+1)*((2+0)+1)-1] <= 1'b0;
regfile[57][(0+1)*((2+0)+1)-1] <= 1'b0;
regfile[57][(1+1)*((2+0)+1)-1] <= 1'b0;
regfile[58][(0+1)*((2+0)+1)-1] <= 1'b0;
regfile[58][(1+1)*((2+0)+1)-1] <= 1'b0;
regfile[59][(0+1)*((2+0)+1)-1] <= 1'b0;
regfile[59][(1+1)*((2+0)+1)-1] <= 1'b0;
regfile[60][(0+1)*((2+0)+1)-1] <= 1'b0;
regfile[60][(1+1)*((2+0)+1)-1] <= 1'b0;
regfile[61][(0+1)*((2+0)+1)-1] <= 1'b0;
regfile[61][(1+1)*((2+0)+1)-1] <= 1'b0;
regfile[62][(0+1)*((2+0)+1)-1] <= 1'b0;
regfile[62][(1+1)*((2+0)+1)-1] <= 1'b0;
regfile[63][(0+1)*((2+0)+1)-1] <= 1'b0;
regfile[63][(1+1)*((2+0)+1)-1] <= 1'b0;
regfile[64][(0+1)*((2+0)+1)-1] <= 1'b0;
regfile[64][(1+1)*((2+0)+1)-1] <= 1'b0;
regfile[65][(0+1)*((2+0)+1)-1] <= 1'b0;
regfile[65][(1+1)*((2+0)+1)-1] <= 1'b0;
regfile[66][(0+1)*((2+0)+1)-1] <= 1'b0;
regfile[66][(1+1)*((2+0)+1)-1] <= 1'b0;
regfile[67][(0+1)*((2+0)+1)-1] <= 1'b0;
regfile[67][(1+1)*((2+0)+1)-1] <= 1'b0;
regfile[68][(0+1)*((2+0)+1)-1] <= 1'b0;
regfile[68][(1+1)*((2+0)+1)-1] <= 1'b0;
regfile[69][(0+1)*((2+0)+1)-1] <= 1'b0;
regfile[69][(1+1)*((2+0)+1)-1] <= 1'b0;
regfile[70][(0+1)*((2+0)+1)-1] <= 1'b0;
regfile[70][(1+1)*((2+0)+1)-1] <= 1'b0;
regfile[71][(0+1)*((2+0)+1)-1] <= 1'b0;
regfile[71][(1+1)*((2+0)+1)-1] <= 1'b0;
regfile[72][(0+1)*((2+0)+1)-1] <= 1'b0;
regfile[72][(1+1)*((2+0)+1)-1] <= 1'b0;
regfile[73][(0+1)*((2+0)+1)-1] <= 1'b0;
regfile[73][(1+1)*((2+0)+1)-1] <= 1'b0;
regfile[74][(0+1)*((2+0)+1)-1] <= 1'b0;
regfile[74][(1+1)*((2+0)+1)-1] <= 1'b0;
regfile[75][(0+1)*((2+0)+1)-1] <= 1'b0;
regfile[75][(1+1)*((2+0)+1)-1] <= 1'b0;
regfile[76][(0+1)*((2+0)+1)-1] <= 1'b0;
regfile[76][(1+1)*((2+0)+1)-1] <= 1'b0;
regfile[77][(0+1)*((2+0)+1)-1] <= 1'b0;
regfile[77][(1+1)*((2+0)+1)-1] <= 1'b0;
regfile[78][(0+1)*((2+0)+1)-1] <= 1'b0;
regfile[78][(1+1)*((2+0)+1)-1] <= 1'b0;
regfile[79][(0+1)*((2+0)+1)-1] <= 1'b0;
regfile[79][(1+1)*((2+0)+1)-1] <= 1'b0;
regfile[80][(0+1)*((2+0)+1)-1] <= 1'b0;
regfile[80][(1+1)*((2+0)+1)-1] <= 1'b0;
regfile[81][(0+1)*((2+0)+1)-1] <= 1'b0;
regfile[81][(1+1)*((2+0)+1)-1] <= 1'b0;
regfile[82][(0+1)*((2+0)+1)-1] <= 1'b0;
regfile[82][(1+1)*((2+0)+1)-1] <= 1'b0;
regfile[83][(0+1)*((2+0)+1)-1] <= 1'b0;
regfile[83][(1+1)*((2+0)+1)-1] <= 1'b0;
regfile[84][(0+1)*((2+0)+1)-1] <= 1'b0;
regfile[84][(1+1)*((2+0)+1)-1] <= 1'b0;
regfile[85][(0+1)*((2+0)+1)-1] <= 1'b0;
regfile[85][(1+1)*((2+0)+1)-1] <= 1'b0;
regfile[86][(0+1)*((2+0)+1)-1] <= 1'b0;
regfile[86][(1+1)*((2+0)+1)-1] <= 1'b0;
regfile[87][(0+1)*((2+0)+1)-1] <= 1'b0;
regfile[87][(1+1)*((2+0)+1)-1] <= 1'b0;
regfile[88][(0+1)*((2+0)+1)-1] <= 1'b0;
regfile[88][(1+1)*((2+0)+1)-1] <= 1'b0;
regfile[89][(0+1)*((2+0)+1)-1] <= 1'b0;
regfile[89][(1+1)*((2+0)+1)-1] <= 1'b0;
regfile[90][(0+1)*((2+0)+1)-1] <= 1'b0;
regfile[90][(1+1)*((2+0)+1)-1] <= 1'b0;
regfile[91][(0+1)*((2+0)+1)-1] <= 1'b0;
regfile[91][(1+1)*((2+0)+1)-1] <= 1'b0;
regfile[92][(0+1)*((2+0)+1)-1] <= 1'b0;
regfile[92][(1+1)*((2+0)+1)-1] <= 1'b0;
regfile[93][(0+1)*((2+0)+1)-1] <= 1'b0;
regfile[93][(1+1)*((2+0)+1)-1] <= 1'b0;
regfile[94][(0+1)*((2+0)+1)-1] <= 1'b0;
regfile[94][(1+1)*((2+0)+1)-1] <= 1'b0;
regfile[95][(0+1)*((2+0)+1)-1] <= 1'b0;
regfile[95][(1+1)*((2+0)+1)-1] <= 1'b0;
regfile[96][(0+1)*((2+0)+1)-1] <= 1'b0;
regfile[96][(1+1)*((2+0)+1)-1] <= 1'b0;
regfile[97][(0+1)*((2+0)+1)-1] <= 1'b0;
regfile[97][(1+1)*((2+0)+1)-1] <= 1'b0;
regfile[98][(0+1)*((2+0)+1)-1] <= 1'b0;
regfile[98][(1+1)*((2+0)+1)-1] <= 1'b0;
regfile[99][(0+1)*((2+0)+1)-1] <= 1'b0;
regfile[99][(1+1)*((2+0)+1)-1] <= 1'b0;
regfile[100][(0+1)*((2+0)+1)-1] <= 1'b0;
regfile[100][(1+1)*((2+0)+1)-1] <= 1'b0;
regfile[101][(0+1)*((2+0)+1)-1] <= 1'b0;
regfile[101][(1+1)*((2+0)+1)-1] <= 1'b0;
regfile[102][(0+1)*((2+0)+1)-1] <= 1'b0;
regfile[102][(1+1)*((2+0)+1)-1] <= 1'b0;
regfile[103][(0+1)*((2+0)+1)-1] <= 1'b0;
regfile[103][(1+1)*((2+0)+1)-1] <= 1'b0;
regfile[104][(0+1)*((2+0)+1)-1] <= 1'b0;
regfile[104][(1+1)*((2+0)+1)-1] <= 1'b0;
regfile[105][(0+1)*((2+0)+1)-1] <= 1'b0;
regfile[105][(1+1)*((2+0)+1)-1] <= 1'b0;
regfile[106][(0+1)*((2+0)+1)-1] <= 1'b0;
regfile[106][(1+1)*((2+0)+1)-1] <= 1'b0;
regfile[107][(0+1)*((2+0)+1)-1] <= 1'b0;
regfile[107][(1+1)*((2+0)+1)-1] <= 1'b0;
regfile[108][(0+1)*((2+0)+1)-1] <= 1'b0;
regfile[108][(1+1)*((2+0)+1)-1] <= 1'b0;
regfile[109][(0+1)*((2+0)+1)-1] <= 1'b0;
regfile[109][(1+1)*((2+0)+1)-1] <= 1'b0;
regfile[110][(0+1)*((2+0)+1)-1] <= 1'b0;
regfile[110][(1+1)*((2+0)+1)-1] <= 1'b0;
regfile[111][(0+1)*((2+0)+1)-1] <= 1'b0;
regfile[111][(1+1)*((2+0)+1)-1] <= 1'b0;
regfile[112][(0+1)*((2+0)+1)-1] <= 1'b0;
regfile[112][(1+1)*((2+0)+1)-1] <= 1'b0;
regfile[113][(0+1)*((2+0)+1)-1] <= 1'b0;
regfile[113][(1+1)*((2+0)+1)-1] <= 1'b0;
regfile[114][(0+1)*((2+0)+1)-1] <= 1'b0;
regfile[114][(1+1)*((2+0)+1)-1] <= 1'b0;
regfile[115][(0+1)*((2+0)+1)-1] <= 1'b0;
regfile[115][(1+1)*((2+0)+1)-1] <= 1'b0;
regfile[116][(0+1)*((2+0)+1)-1] <= 1'b0;
regfile[116][(1+1)*((2+0)+1)-1] <= 1'b0;
regfile[117][(0+1)*((2+0)+1)-1] <= 1'b0;
regfile[117][(1+1)*((2+0)+1)-1] <= 1'b0;
regfile[118][(0+1)*((2+0)+1)-1] <= 1'b0;
regfile[118][(1+1)*((2+0)+1)-1] <= 1'b0;
regfile[119][(0+1)*((2+0)+1)-1] <= 1'b0;
regfile[119][(1+1)*((2+0)+1)-1] <= 1'b0;
regfile[120][(0+1)*((2+0)+1)-1] <= 1'b0;
regfile[120][(1+1)*((2+0)+1)-1] <= 1'b0;
regfile[121][(0+1)*((2+0)+1)-1] <= 1'b0;
regfile[121][(1+1)*((2+0)+1)-1] <= 1'b0;
regfile[122][(0+1)*((2+0)+1)-1] <= 1'b0;
regfile[122][(1+1)*((2+0)+1)-1] <= 1'b0;
regfile[123][(0+1)*((2+0)+1)-1] <= 1'b0;
regfile[123][(1+1)*((2+0)+1)-1] <= 1'b0;
regfile[124][(0+1)*((2+0)+1)-1] <= 1'b0;
regfile[124][(1+1)*((2+0)+1)-1] <= 1'b0;
regfile[125][(0+1)*((2+0)+1)-1] <= 1'b0;
regfile[125][(1+1)*((2+0)+1)-1] <= 1'b0;
regfile[126][(0+1)*((2+0)+1)-1] <= 1'b0;
regfile[126][(1+1)*((2+0)+1)-1] <= 1'b0;
regfile[127][(0+1)*((2+0)+1)-1] <= 1'b0;
regfile[127][(1+1)*((2+0)+1)-1] <= 1'b0;

   end
   else
   if (write_valid_f)
   begin
      // regfile[write_index] <= (write_data & write_mask) | (regfile[write_index] & ~write_mask);
      regfile[write_index_f] <= (write_data_f & write_mask_f) | (regfile[write_index_f] & ~write_mask_f);
   end
end
endmodule
