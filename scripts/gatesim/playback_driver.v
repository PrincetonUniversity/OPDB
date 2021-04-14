// Modified by Princeton University on June 9th, 2015
// ========== Copyright Header Begin ==========================================
//
// OpenSPARC T1 Processor File: playback_driver.v
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
//`timescale 1ns/1ps
//`define ARM_UD_MODEL
`include "define.tmp.h"

module playback_driver();

parameter inputwidth=361;
parameter outputwidth=331;
parameter clockwidth=1;


reg [256*8-1:0] stimfile;
reg [256*8-1:0] iofile;

integer result;
reg     tmp;

initial begin
    stimfile = "not_provided";
    result = $test$plusargs("stim_file");
    if(result) begin
        tmp = $value$plusargs("stim_file=%s", stimfile);
    end
end


reg	[inputwidth-1:0]	input_vector;
reg	[inputwidth-1:0]	input_vector_a;
reg [outputwidth-1:0]   output_vector_ref;
wire                    clock_vector;
reg                     core_ref_clk;

//
// NOTE: Assign clock vector to correct clock signal
//
wire clk = clock_vector;

initial begin
    core_ref_clk = 1'b0;
    forever #2000 core_ref_clk = ~core_ref_clk;
end



assign clock_vector = core_ref_clk;

integer fid, code, not_used_ret;
integer	mismatch;
integer cycle;

initial begin
  fid = $fopen(stimfile, "r");
end


always @(posedge clock_vector) begin
  #300;
  input_vector = input_vector_a;
  code = $fscanf(fid, "%b\n", input_vector_a);
  if(code == 0 || code == -1) begin
    if(mismatch == 0)
      $display("Playback PASSED!");
   else
      $display("Playback FAILED with %1d mismatches!", mismatch);
    $finish;
  end
  @(negedge clock_vector);
  #100;
  not_used_ret = $fscanf(fid, "%b\n", output_vector_ref);
end




task displayMismatch;
input [8:0] port;
input exp;
input got;
begin
  if(port < 1) begin
    $display("thanksIn_P: Expect:%b Got:%b at cycle %d", exp , got, cycle);
  end else
  if(port < 2) begin
    $display("yummyOut_P: Expect:%b Got:%b at cycle %d", exp , got, cycle);
  end else
  if(port < 3) begin
    $display("yummyOut_W: Expect:%b Got:%b at cycle %d", exp , got, cycle);
  end else
  if(port < 4) begin
    $display("yummyOut_S: Expect:%b Got:%b at cycle %d", exp , got, cycle);
  end else
  if(port < 5) begin
    $display("yummyOut_E: Expect:%b Got:%b at cycle %d", exp , got, cycle);
  end else
  if(port < 6) begin
    $display("yummyOut_N: Expect:%b Got:%b at cycle %d", exp , got, cycle);
  end else
  if(port < 7) begin
    $display("validOut_P: Expect:%b Got:%b at cycle %d", exp , got, cycle);
  end else
  if(port < 8) begin
    $display("validOut_W: Expect:%b Got:%b at cycle %d", exp , got, cycle);
  end else
  if(port < 9) begin
    $display("validOut_S: Expect:%b Got:%b at cycle %d", exp , got, cycle);
  end else
  if(port < 10) begin
    $display("validOut_E: Expect:%b Got:%b at cycle %d", exp , got, cycle);
  end else
  if(port < 11) begin
    $display("validOut_N: Expect:%b Got:%b at cycle %d", exp , got, cycle);
  end else
  if(port < 75) begin
    $display("dataOut_P[64]: Expect:%b Got:%b at cycle %d", exp , got, cycle);
  end else
  if(port < 139) begin
    $display("dataOut_W[64]: Expect:%b Got:%b at cycle %d", exp , got, cycle);
  end else
  if(port < 203) begin
    $display("dataOut_S[64]: Expect:%b Got:%b at cycle %d", exp , got, cycle);
  end else
  if(port < 267) begin
    $display("dataOut_E[64]: Expect:%b Got:%b at cycle %d", exp , got, cycle);
  end else
  begin
    $display("dataOut_N[64]: Expect:%b Got:%b at cycle %d", exp , got, cycle);
  end
end
endtask




wire	[outputwidth-1:0]	output_vector;
reg	[outputwidth-1:0]	output_vector_mask;
wire	[outputwidth-1:0]	output_vector_masked;
wire	[outputwidth-1:0]	output_vector_ref_masked;


wire  reset_in;
wire [`DATA_WIDTH-1:0] dataIn_N;
wire [`DATA_WIDTH-1:0] dataIn_E;
wire [`DATA_WIDTH-1:0] dataIn_S;
wire [`DATA_WIDTH-1:0] dataIn_W;
wire [`DATA_WIDTH-1:0] dataIn_P;
wire  validIn_N;
wire  validIn_E;
wire  validIn_S;
wire  validIn_W;
wire  validIn_P;
wire  yummyIn_N;
wire  yummyIn_E;
wire  yummyIn_S;
wire  yummyIn_W;
wire  yummyIn_P;
wire [`XY_WIDTH-1:0] myLocX;
wire [`XY_WIDTH-1:0] myLocY;
wire [`CHIP_ID_WIDTH-1:0] myChipID;
wire [`DATA_WIDTH-1:0] dataOut_N;
wire [`DATA_WIDTH-1:0] dataOut_E;
wire [`DATA_WIDTH-1:0] dataOut_S;
wire [`DATA_WIDTH-1:0] dataOut_W;
wire [`DATA_WIDTH-1:0] dataOut_P;
wire  validOut_N;
wire  validOut_E;
wire  validOut_S;
wire  validOut_W;
wire  validOut_P;
wire  yummyOut_N;
wire  yummyOut_E;
wire  yummyOut_S;
wire  yummyOut_W;
wire  yummyOut_P;
wire  thanksIn_P;



// my_dynamic_node instance
dynamic_node_top_wrap my_dynamic_node (
    .clk(clk),
    .reset_in(reset_in),
    .dataIn_N(dataIn_N),
    .dataIn_E(dataIn_E),
    .dataIn_S(dataIn_S),
    .dataIn_W(dataIn_W),
    .dataIn_P(dataIn_P),
    .validIn_N(validIn_N),
    .validIn_E(validIn_E),
    .validIn_S(validIn_S),
    .validIn_W(validIn_W),
    .validIn_P(validIn_P),
    .yummyIn_N(yummyIn_N),
    .yummyIn_E(yummyIn_E),
    .yummyIn_S(yummyIn_S),
    .yummyIn_W(yummyIn_W),
    .yummyIn_P(yummyIn_P),
    .myLocX(myLocX),
    .myLocY(myLocY),
    .myChipID(myChipID),
    .dataOut_N(dataOut_N),
    .dataOut_E(dataOut_E),
    .dataOut_S(dataOut_S),
    .dataOut_W(dataOut_W),
    .dataOut_P(dataOut_P),
    .validOut_N(validOut_N),
    .validOut_E(validOut_E),
    .validOut_S(validOut_S),
    .validOut_W(validOut_W),
    .validOut_P(validOut_P),
    .yummyOut_N(yummyOut_N),
    .yummyOut_E(yummyOut_E),
    .yummyOut_S(yummyOut_S),
    .yummyOut_W(yummyOut_W),
    .yummyOut_P(yummyOut_P),
    .thanksIn_P(thanksIn_P)
);



task generate_mask;
integer i;
begin
for(i=0;i<outputwidth;i=i+1)
  output_vector_mask[i] = (output_vector_ref[i] === 1'b0) | (output_vector_ref[i] === 1'b1);
end
endtask


assign {
    reset_in,
    dataIn_N,
    dataIn_E,
    dataIn_S,
    dataIn_W,
    dataIn_P,
    validIn_N,
    validIn_E,
    validIn_S,
    validIn_W,
    validIn_P,
    yummyIn_N,
    yummyIn_E,
    yummyIn_S,
    yummyIn_W,
    yummyIn_P,
    myLocX,
    myLocY,
    myChipID

} = input_vector;

assign output_vector = {
    dataOut_N,
    dataOut_E,
    dataOut_S,
    dataOut_W,
    dataOut_P,
    validOut_N,
    validOut_E,
    validOut_S,
    validOut_W,
    validOut_P,
    yummyOut_N,
    yummyOut_E,
    yummyOut_S,
    yummyOut_W,
    yummyOut_P,
    thanksIn_P

};





assign output_vector_ref_masked = output_vector_ref & output_vector_mask;
assign output_vector_masked = output_vector & output_vector_mask;

always @(output_vector_ref)
  generate_mask;

integer i;
initial generate_mask;

initial mismatch = 0;
initial cycle = 0;

always @(negedge clock_vector) begin
  cycle = cycle + 1;
  if(cycle > 2)
  begin
  if(output_vector_ref_masked !== output_vector_masked) begin
    mismatch = mismatch + 1;
    for(i=0;i<outputwidth;i=i+1)
      if(output_vector_ref_masked[i] !== output_vector_masked[i])
	 displayMismatch(i, output_vector_ref_masked[i], output_vector_masked[i]);
    $display("Number of cycles mismatched %d\n",mismatch);
  end
  end
end

endmodule

module cmp_top();

playback_driver iop();

//`ifdef DUMP_ON

initial
begin
  //if($test$plusargs("dump"))
  //begin
  //  $fsdbDumpvars(0, cmp_top.iop);
  //end
  $dumpfile("gate_sim.vcd");
  $dumpvars;
  #4500000000000;
  $finish;
end
//`endif

endmodule


