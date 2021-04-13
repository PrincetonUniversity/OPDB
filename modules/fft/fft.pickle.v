////////////////////////////////////////////////////////////////////////////////
//
// Filename:	../rtl/bimpy.v
//
// Project:	A General Purpose Pipelined FFT Implementation
//
// Purpose:	A simple 2-bit multiply based upon the fact that LUT's allow
//		6-bits of input.  In other words, I could build a 3-bit
//	multiply from 6 LUTs (5 actually, since the first could have two
//	outputs).  This would allow multiplication of three bit digits, save
//	only for the fact that you would need two bits of carry.  The bimpy
//	approach throttles back a bit and does a 2x2 bit multiply in a LUT,
//	guaranteeing that it will never carry more than one bit.  While this
//	multiply is hardware independent (and can still run under Verilator
//	therefore), it is really motivated by trying to optimize for a
//	specific piece of hardware (Xilinx-7 series ...) that has at least
//	4-input LUT's with carry chains.
//
//
//
// Creator:	Dan Gisselquist, Ph.D.
//		Gisselquist Technology, LLC
//
////////////////////////////////////////////////////////////////////////////////
//
// Copyright (C) 2015-2020, Gisselquist Technology, LLC
//
// This file is part of the general purpose pipelined FFT project.
//
// The pipelined FFT project is free software (firmware): you can redistribute
// it and/or modify it under the terms of the GNU Lesser General Public License
// as published by the Free Software Foundation, either version 3 of the
// License, or (at your option) any later version.
//
// The pipelined FFT project is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTIBILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU Lesser
// General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  (It's in the $(ROOT)/doc directory.  Run make
// with no target there if the PDF file isn't present.)  If not, see
// <http://www.gnu.org/licenses/> for a copy.
//
// License:	LGPL, v3, as defined and found on www.gnu.org,
//		http://www.gnu.org/licenses/lgpl.html
//
//
////////////////////////////////////////////////////////////////////////////////
//
//
`default_nettype	none
//
module	bimpy(i_clk, i_ce, i_a, i_b, o_r);
	parameter	BW=18; // Number of bits in i_b
	localparam	LUTB=2; // Number of bits in i_a for our LUT multiply
	input	wire			i_clk, i_ce;
	input	wire	[(LUTB-1):0]	i_a;
	input	wire	[(BW-1):0]	i_b;
	output	reg	[(BW+LUTB-1):0]	o_r;

	wire	[(BW+LUTB-2):0]	w_r;
	wire	[(BW+LUTB-3):1]	c;

	assign	w_r =  { ((i_a[1])?i_b:{(BW){1'b0}}), 1'b0 }
				^ { 1'b0, ((i_a[0])?i_b:{(BW){1'b0}}) };
	assign	c = { ((i_a[1])?i_b[(BW-2):0]:{(BW-1){1'b0}}) }
			& ((i_a[0])?i_b[(BW-1):1]:{(BW-1){1'b0}});

	initial o_r = 0;
	always @(posedge i_clk)
	if (i_ce)
		o_r <= w_r + { c, 2'b0 };
























endmodule
////////////////////////////////////////////////////////////////////////////////
//
// Filename:	bitreverse.v
//
// Project:	A General Purpose Pipelined FFT Implementation
//
// Purpose:	This module bitreverses a pipelined FFT input.  It differes
//		from the dblreverse module in that this is just a simple and
//	straightforward bitreverse, rather than one written to handle two
//	words at once.
//
//
// Creator:	Dan Gisselquist, Ph.D.
//		Gisselquist Technology, LLC
//
////////////////////////////////////////////////////////////////////////////////
//
// Copyright (C) 2015-2020, Gisselquist Technology, LLC
//
// This file is part of the general purpose pipelined FFT project.
//
// The pipelined FFT project is free software (firmware): you can redistribute
// it and/or modify it under the terms of the GNU Lesser General Public License
// as published by the Free Software Foundation, either version 3 of the
// License, or (at your option) any later version.
//
// The pipelined FFT project is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTIBILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU Lesser
// General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  (It's in the $(ROOT)/doc directory.  Run make
// with no target there if the PDF file isn't present.)  If not, see
// <http://www.gnu.org/licenses/> for a copy.
//
// License:	LGPL, v3, as defined and found on www.gnu.org,
//		http://www.gnu.org/licenses/lgpl.html
//
//
////////////////////////////////////////////////////////////////////////////////
//
//
`default_nettype	none
//
module	bitreverse(i_clk, i_reset, i_ce, i_in, o_out, o_sync);
	parameter			LGSIZE=5, WIDTH=24;
	input	wire			i_clk, i_reset, i_ce;
	input	wire	[(2*WIDTH-1):0]	i_in;
	output	reg	[(2*WIDTH-1):0]	o_out;
	output	reg			o_sync;
	reg	[(LGSIZE):0]	wraddr;
	wire	[(LGSIZE):0]	rdaddr;

	reg	[(2*WIDTH-1):0]	brmem	[0:((1<<(LGSIZE+1))-1)];

	genvar	k;
	generate for(k=0; k<LGSIZE; k=k+1)
	begin : DBL
		assign rdaddr[k] = wraddr[LGSIZE-1-k];
	end endgenerate
	assign	rdaddr[LGSIZE] = !wraddr[LGSIZE];

	reg	in_reset;

	initial	in_reset = 1'b1;
	always @(posedge i_clk)
		if (i_reset)
			in_reset <= 1'b1;
		else if ((i_ce)&&(&wraddr[(LGSIZE-1):0]))
			in_reset <= 1'b0;

	initial	wraddr = 0;
	always @(posedge i_clk)
		if (i_reset)
			wraddr <= 0;
		else if (i_ce)
		begin
			brmem[wraddr] <= i_in;
			wraddr <= wraddr + 1;
		end

	always @(posedge i_clk)
		if (i_ce) // If (i_reset) we just output junk ... not a problem
			o_out <= brmem[rdaddr]; // w/o a sync pulse

	initial	o_sync = 1'b0;
	always @(posedge i_clk)
		if (i_reset)
			o_sync <= 1'b0;
		else if ((i_ce)&&(!in_reset))
			o_sync <= (wraddr[(LGSIZE-1):0] == 0);


















































































	// FORMAL
endmodule
////////////////////////////////////////////////////////////////////////////////
//
// Filename:	butterfly.v
//
// Project:	A General Purpose Pipelined FFT Implementation
//
// Purpose:	This routine caculates a butterfly for a decimation
//		in frequency version of an FFT.  Specifically, given
//	complex Left and Right values together with a coefficient, the output
//	of this routine is given by:
//
//		L' = L + R
//		R' = (L - R)*C
//
//	The rest of the junk below handles timing (mostly), to make certain
//	that L' and R' reach the output at the same clock.  Further, just to
//	make certain that is the case, an 'aux' input exists.  This aux value
//	will come out of this routine synchronized to the values it came in
//	with.  (i.e., both L', R', and aux all have the same delay.)  Hence,
//	a caller of this routine may set aux on the first input with valid
//	data, and then wait to see aux set on the output to know when to find
//	the first output with valid data.
//
//	All bits are preserved until the very last clock, where any more bits
//	than OWIDTH will be quietly discarded.
//
//	This design features no overflow checking.
//
// Notes:
//	CORDIC:
//		Much as we might like, we can't use a cordic here.
//		The goal is to accomplish an FFT, as defined, and a
//		CORDIC places a scale factor onto the data.  Removing
//		the scale factor would cost two multiplies, which
//		is precisely what we are trying to avoid.
//
//
//	3-MULTIPLIES:
//		It should also be possible to do this with three multiplies
//		and an extra two addition cycles.
//
//		We want
//			R+I = (a + jb) * (c + jd)
//			R+I = (ac-bd) + j(ad+bc)
//		We multiply
//			P1 = ac
//			P2 = bd
//			P3 = (a+b)(c+d)
//		Then
//			R+I=(P1-P2)+j(P3-P2-P1)
//
//		WIDTHS:
//		On multiplying an X width number by an
//		Y width number, X>Y, the result should be (X+Y)
//		bits, right?
//		-2^(X-1) <= a <= 2^(X-1) - 1
//		-2^(Y-1) <= b <= 2^(Y-1) - 1
//		(2^(Y-1)-1)*(-2^(X-1)) <= ab <= 2^(X-1)2^(Y-1)
//		-2^(X+Y-2)+2^(X-1) <= ab <= 2^(X+Y-2) <= 2^(X+Y-1) - 1
//		-2^(X+Y-1) <= ab <= 2^(X+Y-1)-1
//		YUP!  But just barely.  Do this and you'll really want
//		to drop a bit, although you will risk overflow in so
//		doing.
//
//	20150602 -- The sync logic lines have been completely redone.  The
//		synchronization lines no longer go through the FIFO with the
//		left hand sum, but are kept out of memory.  This allows the
//		butterfly to use more optimal memory resources, while also
//		guaranteeing that the sync lines can be properly reset upon
//		any reset signal.
//
//
// Creator:	Dan Gisselquist, Ph.D.
//		Gisselquist Technology, LLC
//
////////////////////////////////////////////////////////////////////////////////
//
// Copyright (C) 2015-2020, Gisselquist Technology, LLC
//
// This file is part of the general purpose pipelined FFT project.
//
// The pipelined FFT project is free software (firmware): you can redistribute
// it and/or modify it under the terms of the GNU Lesser General Public License
// as published by the Free Software Foundation, either version 3 of the
// License, or (at your option) any later version.
//
// The pipelined FFT project is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTIBILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU Lesser
// General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  (It's in the $(ROOT)/doc directory.  Run make
// with no target there if the PDF file isn't present.)  If not, see
// <http://www.gnu.org/licenses/> for a copy.
//
// License:	LGPL, v3, as defined and found on www.gnu.org,
//		http://www.gnu.org/licenses/lgpl.html
//
//
////////////////////////////////////////////////////////////////////////////////
//
//
`default_nettype	none
//
module	butterfly(i_clk, i_reset, i_ce, i_coef, i_left, i_right, i_aux,
		o_left, o_right, o_aux);
	// Public changeable parameters ...
	parameter IWIDTH=16,CWIDTH=20,OWIDTH=17;
	parameter	SHIFT=0;
	// The number of clocks per each i_ce.  The actual number can be
	// more, but the algorithm depends upon at least this many for
	// extra internal processing.
	parameter	CKPCE=1;
	//
	// Local/derived parameters that are calculated from the above
	// params.  Apart from algorithmic changes below, these should not
	// be adjusted
	//
	// The first step is to calculate how many clocks it takes our
	// multiply to come back with an answer within.  The time in the
	// multiply depends upon the input value with the fewest number of
	// bits--to keep the pipeline depth short.  So, let's find the
	// fewest number of bits here.
	localparam MXMPYBITS = 
		((IWIDTH+2)>(CWIDTH+1)) ? (CWIDTH+1) : (IWIDTH + 2);
	//
	// Given this "fewest" number of bits, we can calculate the
	// number of clocks the multiply itself will take.
	localparam	MPYDELAY=((MXMPYBITS+1)/2)+2;
	//
	// In an environment when CKPCE > 1, the multiply delay isn't
	// necessarily the delay felt by this algorithm--measured in
	// i_ce's.  In particular, if the multiply can operate with more
	// operations per clock, it can appear to finish "faster".
	// Since most of the logic in this core operates on the slower
	// clock, we'll need to map that speed into the number of slower
	// clock ticks that it takes.
	localparam	LCLDELAY = (CKPCE == 1) ? MPYDELAY
		: (CKPCE == 2) ? (MPYDELAY/2+2)
		: (MPYDELAY/3 + 2);
	localparam	LGDELAY = (MPYDELAY>64) ? 7
			: (MPYDELAY > 32) ? 6
			: (MPYDELAY > 16) ? 5
			: (MPYDELAY >  8) ? 4
			: (MPYDELAY >  4) ? 3
			: 2;
	localparam	AUXLEN=(LCLDELAY+3);
	localparam	MPYREMAINDER = MPYDELAY - CKPCE*(MPYDELAY/CKPCE);


	input	wire	i_clk, i_reset, i_ce;
	input	wire	[(2*CWIDTH-1):0] i_coef;
	input	wire	[(2*IWIDTH-1):0] i_left, i_right;
	input	wire	i_aux;
	output	wire	[(2*OWIDTH-1):0] o_left, o_right;
	output	reg	o_aux;

































	reg	[(2*IWIDTH-1):0]	r_left, r_right;
	reg	[(2*CWIDTH-1):0]	r_coef, r_coef_2;
	wire	signed	[(IWIDTH-1):0]	r_left_r, r_left_i, r_right_r, r_right_i;
	assign	r_left_r  = r_left[ (2*IWIDTH-1):(IWIDTH)];
	assign	r_left_i  = r_left[ (IWIDTH-1):0];
	assign	r_right_r = r_right[(2*IWIDTH-1):(IWIDTH)];
	assign	r_right_i = r_right[(IWIDTH-1):0];

	reg	signed	[(IWIDTH):0]	r_sum_r, r_sum_i, r_dif_r, r_dif_i;

	reg	[(LGDELAY-1):0]	fifo_addr;
	wire	[(LGDELAY-1):0]	fifo_read_addr;
	assign	fifo_read_addr = fifo_addr - LCLDELAY[(LGDELAY-1):0];
	reg	[(2*IWIDTH+1):0]	fifo_left [ 0:((1<<LGDELAY)-1)];

	// Set up the input to the multiply
	always @(posedge i_clk)
	if (i_ce)
	begin
		// One clock just latches the inputs
		r_left <= i_left;	// No change in # of bits
		r_right <= i_right;
		r_coef  <= i_coef;
		// Next clock adds/subtracts
		r_sum_r <= r_left_r + r_right_r; // Now IWIDTH+1 bits
		r_sum_i <= r_left_i + r_right_i;
		r_dif_r <= r_left_r - r_right_r;
		r_dif_i <= r_left_i - r_right_i;
		// Other inputs are simply delayed on second clock
		r_coef_2<= r_coef;
	end

	// Don't forget to record the even side, since it doesn't need
	// to be multiplied, but yet we still need the results in sync
	// with the answer when it is ready.
	initial fifo_addr = 0;
	always @(posedge i_clk)
	if (i_reset)
		fifo_addr <= 0;
	else if (i_ce)
		// Need to delay the sum side--nothing else happens
		// to it, but it needs to stay synchronized with the
		// right side.
		fifo_addr <= fifo_addr + 1;

	always @(posedge i_clk)
	if (i_ce)
		fifo_left[fifo_addr] <= { r_sum_r, r_sum_i };

	wire	signed	[(CWIDTH-1):0]	ir_coef_r, ir_coef_i;
	assign	ir_coef_r = r_coef_2[(2*CWIDTH-1):CWIDTH];
	assign	ir_coef_i = r_coef_2[(CWIDTH-1):0];
	wire	signed	[((IWIDTH+2)+(CWIDTH+1)-1):0]	p_one, p_two, p_three;


	// Multiply output is always a width of the sum of the widths of
	// the two inputs.  ALWAYS.  This is independent of the number of
	// bits in p_one, p_two, or p_three.  These values needed to
	// accumulate a bit (or two) each.  However, this approach to a
	// three multiply complex multiply cannot increase the total
	// number of bits in our final output.  We'll take care of
	// dropping back down to the proper width, OWIDTH, in our routine
	// below.


	// We accomplish here "Karatsuba" multiplication.  That is,
	// by doing three multiplies we accomplish the work of four.
	// Let's prove to ourselves that this works ... We wish to
	// multiply: (a+jb) * (c+jd), where a+jb is given by
	//	a + jb = r_dif_r + j r_dif_i, and
	//	c + jd = ir_coef_r + j ir_coef_i.
	// We do this by calculating the intermediate products P1, P2,
	// and P3 as
	//	P1 = ac
	//	P2 = bd
	//	P3 = (a + b) * (c + d)
	// and then complete our final answer with
	//	ac - bd = P1 - P2 (this checks)
	//	ad + bc = P3 - P2 - P1
	//	        = (ac + bc + ad + bd) - bd - ac
	//	        = bc + ad (this checks)


	// This should really be based upon an IF, such as in
	// if (IWIDTH < CWIDTH) then ...
	// However, this is the only (other) way I know to do it.
	generate if (CKPCE <= 1)
	begin

		wire	[(CWIDTH):0]	p3c_in;
		wire	[(IWIDTH+1):0]	p3d_in;
		assign	p3c_in = ir_coef_i + ir_coef_r;
		assign	p3d_in = r_dif_r + r_dif_i;

		// We need to pad these first two multiplies by an extra
		// bit just to keep them aligned with the third,
		// simpler, multiply.
		longbimpy #(CWIDTH+1,IWIDTH+2) p1(i_clk, i_ce,
				{ir_coef_r[CWIDTH-1],ir_coef_r},
				{r_dif_r[IWIDTH],r_dif_r}, p_one



			);
		longbimpy #(CWIDTH+1,IWIDTH+2) p2(i_clk, i_ce,
				{ir_coef_i[CWIDTH-1],ir_coef_i},
				{r_dif_i[IWIDTH],r_dif_i}, p_two



			);
		longbimpy #(CWIDTH+1,IWIDTH+2) p3(i_clk, i_ce,
				p3c_in, p3d_in, p_three



			);

	end else if (CKPCE == 2)
	begin : CKPCE_TWO
		// Coefficient multiply inputs
		reg		[2*(CWIDTH)-1:0]	mpy_pipe_c;
		// Data multiply inputs
		reg		[2*(IWIDTH+1)-1:0]	mpy_pipe_d;
		wire	signed	[(CWIDTH-1):0]	mpy_pipe_vc;
		wire	signed	[(IWIDTH):0]	mpy_pipe_vd;
		//
		reg	signed	[(CWIDTH+1)-1:0]	mpy_cof_sum;
		reg	signed	[(IWIDTH+2)-1:0]	mpy_dif_sum;

		assign	mpy_pipe_vc =  mpy_pipe_c[2*(CWIDTH)-1:CWIDTH];
		assign	mpy_pipe_vd =  mpy_pipe_d[2*(IWIDTH+1)-1:IWIDTH+1];

		reg			mpy_pipe_v;
		reg			ce_phase;

		reg	signed	[(CWIDTH+IWIDTH+3)-1:0]	mpy_pipe_out;
		reg	signed [IWIDTH+CWIDTH+3-1:0]	longmpy;














		initial	ce_phase = 1'b0;
		always @(posedge i_clk)
		if (i_reset)
			ce_phase <= 1'b0;
		else if (i_ce)
			ce_phase <= 1'b1;
		else
			ce_phase <= 1'b0;

		always @(*)
			mpy_pipe_v = (i_ce)||(ce_phase);

		always @(posedge i_clk)
		if (ce_phase)
		begin
			mpy_pipe_c[2*CWIDTH-1:0] <=
					{ ir_coef_r, ir_coef_i };
			mpy_pipe_d[2*(IWIDTH+1)-1:0] <=
					{ r_dif_r, r_dif_i };

			mpy_cof_sum  <= ir_coef_i + ir_coef_r;
			mpy_dif_sum <= r_dif_r + r_dif_i;

		end else if (i_ce)
		begin
			mpy_pipe_c[2*(CWIDTH)-1:0] <= {
				mpy_pipe_c[(CWIDTH)-1:0], {(CWIDTH){1'b0}} };
			mpy_pipe_d[2*(IWIDTH+1)-1:0] <= {
				mpy_pipe_d[(IWIDTH+1)-1:0], {(IWIDTH+1){1'b0}} };
		end

		longbimpy #(CWIDTH+1,IWIDTH+2) mpy0(i_clk, mpy_pipe_v,
				mpy_cof_sum, mpy_dif_sum, longmpy



			);

		longbimpy #(CWIDTH+1,IWIDTH+2) mpy1(i_clk, mpy_pipe_v,
				{ mpy_pipe_vc[CWIDTH-1], mpy_pipe_vc },
				{ mpy_pipe_vd[IWIDTH  ], mpy_pipe_vd },
				mpy_pipe_out



			);

		reg	signed	[((IWIDTH+2)+(CWIDTH+1)-1):0]
					rp_one, rp_two, rp_three,
					rp2_one, rp2_two, rp2_three;

		always @(posedge i_clk)
		if (((i_ce)&&(!MPYDELAY[0]))
			||((ce_phase)&&(MPYDELAY[0])))
		begin
			rp_one <= mpy_pipe_out;




		end

		always @(posedge i_clk)
		if (((i_ce)&&(MPYDELAY[0]))
			||((ce_phase)&&(!MPYDELAY[0])))
		begin
			rp_two <= mpy_pipe_out;




		end

		always @(posedge i_clk)
		if (i_ce)
		begin
			rp_three <= longmpy;




		end


		// Our outputs *MUST* be set on a clock where i_ce is
		// true for the following logic to work.  Make that
		// happen here.
		always @(posedge i_clk)
		if (i_ce)
		begin
			rp2_one<= rp_one;
			rp2_two <= rp_two;
			rp2_three<= rp_three;










		end

		assign	p_one	= rp2_one;
		assign	p_two	= (!MPYDELAY[0])? rp2_two  : rp_two;
		assign	p_three	= ( MPYDELAY[0])? rp_three : rp2_three;

		// verilator lint_off UNUSED
		wire	[2*(IWIDTH+CWIDTH+3)-1:0]	unused;
		assign	unused = { rp2_two, rp2_three };
		// verilator lint_on  UNUSED












	end else if (CKPCE <= 3)
	begin : CKPCE_THREE
		// Coefficient multiply inputs
		reg		[3*(CWIDTH+1)-1:0]	mpy_pipe_c;
		// Data multiply inputs
		reg		[3*(IWIDTH+2)-1:0]	mpy_pipe_d;
		wire	signed	[(CWIDTH):0]	mpy_pipe_vc;
		wire	signed	[(IWIDTH+1):0]	mpy_pipe_vd;

		assign	mpy_pipe_vc =  mpy_pipe_c[3*(CWIDTH+1)-1:2*(CWIDTH+1)];
		assign	mpy_pipe_vd =  mpy_pipe_d[3*(IWIDTH+2)-1:2*(IWIDTH+2)];

		reg			mpy_pipe_v;
		reg		[2:0]	ce_phase;

		wire	signed	[  (CWIDTH+IWIDTH+3)-1:0]	mpy_pipe_out;













		initial	ce_phase = 3'b011;
		always @(posedge i_clk)
		if (i_reset)
			ce_phase <= 3'b011;
		else if (i_ce)
			ce_phase <= 3'b000;
		else if (ce_phase != 3'b011)
			ce_phase <= ce_phase + 1'b1;

		always @(*)
			mpy_pipe_v = (i_ce)||(ce_phase < 3'b010);

		always @(posedge i_clk)
		if (ce_phase == 3'b000)
		begin
			// Second clock
			mpy_pipe_c[3*(CWIDTH+1)-1:(CWIDTH+1)] <= {
				ir_coef_r[CWIDTH-1], ir_coef_r,
				ir_coef_i[CWIDTH-1], ir_coef_i };
			mpy_pipe_c[CWIDTH:0] <= ir_coef_i + ir_coef_r;
			mpy_pipe_d[3*(IWIDTH+2)-1:(IWIDTH+2)] <= {
				r_dif_r[IWIDTH], r_dif_r,
				r_dif_i[IWIDTH], r_dif_i };
			mpy_pipe_d[(IWIDTH+2)-1:0] <= r_dif_r + r_dif_i;

		end else if (mpy_pipe_v)
		begin
			mpy_pipe_c[3*(CWIDTH+1)-1:0] <= {
				mpy_pipe_c[2*(CWIDTH+1)-1:0], {(CWIDTH+1){1'b0}} };
			mpy_pipe_d[3*(IWIDTH+2)-1:0] <= {
				mpy_pipe_d[2*(IWIDTH+2)-1:0], {(IWIDTH+2){1'b0}} };
		end

		longbimpy #(CWIDTH+1,IWIDTH+2) mpy(i_clk, mpy_pipe_v,
				mpy_pipe_vc, mpy_pipe_vd, mpy_pipe_out



			);

		reg	signed	[((IWIDTH+2)+(CWIDTH+1)-1):0]
				rp_one,  rp_two,  rp_three,
				rp2_one, rp2_two, rp2_three,
				rp3_one;

		always @(posedge i_clk)
		if (MPYREMAINDER == 0)
		begin

			if (i_ce)
			begin
				rp_two   <= mpy_pipe_out;




			end else if (ce_phase == 3'b000)
			begin
				rp_three <= mpy_pipe_out;




			end else if (ce_phase == 3'b001)
			begin
				rp_one   <= mpy_pipe_out;




			end
		end else if (MPYREMAINDER == 1)
		begin

			if (i_ce)
			begin
				rp_one   <= mpy_pipe_out;




			end else if (ce_phase == 3'b000)
			begin
				rp_two   <= mpy_pipe_out;




			end else if (ce_phase == 3'b001)
			begin
				rp_three <= mpy_pipe_out;




			end
		end else // if (MPYREMAINDER == 2)
		begin

			if (i_ce)
			begin
				rp_three <= mpy_pipe_out;




			end else if (ce_phase == 3'b000)
			begin
				rp_one   <= mpy_pipe_out;




			end else if (ce_phase == 3'b001)
			begin
				rp_two   <= mpy_pipe_out;




			end
		end

		always @(posedge i_clk)
		if (i_ce)
		begin
			rp2_one   <= rp_one;
			rp2_two   <= rp_two;
			rp2_three <= (MPYREMAINDER == 2) ? mpy_pipe_out : rp_three;
			rp3_one   <= (MPYREMAINDER == 0) ? rp2_one : rp_one;












		end

		assign	p_one   = rp3_one;
		assign	p_two   = rp2_two;
		assign	p_three = rp2_three;












	end endgenerate
	// These values are held in memory and delayed during the
	// multiply.  Here, we recover them.  During the multiply,
	// values were multiplied by 2^(CWIDTH-2)*exp{-j*2*pi*...},
	// therefore, the left_x values need to be right shifted by
	// CWIDTH-2 as well.  The additional bits come from a sign
	// extension.
	wire	signed	[(IWIDTH+CWIDTH):0]	fifo_i, fifo_r;
	reg		[(2*IWIDTH+1):0]	fifo_read;
	assign	fifo_r = { {2{fifo_read[2*(IWIDTH+1)-1]}},
		fifo_read[(2*(IWIDTH+1)-1):(IWIDTH+1)], {(CWIDTH-2){1'b0}} };
	assign	fifo_i = { {2{fifo_read[(IWIDTH+1)-1]}},
		fifo_read[((IWIDTH+1)-1):0], {(CWIDTH-2){1'b0}} };


	reg	signed	[(CWIDTH+IWIDTH+3-1):0]	mpy_r, mpy_i;

	// Let's do some rounding and remove unnecessary bits.
	// We have (IWIDTH+CWIDTH+3) bits here, we need to drop down to
	// OWIDTH, and SHIFT by SHIFT bits in the process.  The trick is
	// that we don't need (IWIDTH+CWIDTH+3) bits.  We've accumulated
	// them, but the actual values will never fill all these bits.
	// In particular, we only need:
	//	 IWIDTH bits for the input
	//	     +1 bit for the add/subtract
	//	+CWIDTH bits for the coefficient multiply
	//	     +1 bit for the add/subtract in the complex multiply
	//	 ------
	//	 (IWIDTH+CWIDTH+2) bits at full precision.
	//
	// However, the coefficient multiply multiplied by a maximum value
	// of 2^(CWIDTH-2).  Thus, we only have
	//	   IWIDTH bits for the input
	//	       +1 bit for the add/subtract
	//	+CWIDTH-2 bits for the coefficient multiply
	//	       +1 (optional) bit for the add/subtract in the cpx mpy.
	//	 -------- ... multiply.  (This last bit may be shifted out.)
	//	 (IWIDTH+CWIDTH) valid output bits.
	// Now, if the user wants to keep any extras of these (via OWIDTH),
	// or if he wishes to arbitrarily shift some of these off (via
	// SHIFT) we accomplish that here.

	wire	signed	[(OWIDTH-1):0]	rnd_left_r, rnd_left_i, rnd_right_r, rnd_right_i;

	wire	signed	[(CWIDTH+IWIDTH+3-1):0]	left_sr, left_si;
	assign	left_sr = { {(2){fifo_r[(IWIDTH+CWIDTH)]}}, fifo_r };
	assign	left_si = { {(2){fifo_i[(IWIDTH+CWIDTH)]}}, fifo_i };

	convround #(CWIDTH+IWIDTH+3,OWIDTH,SHIFT+4) do_rnd_left_r(i_clk, i_ce,
				left_sr, rnd_left_r);

	convround #(CWIDTH+IWIDTH+3,OWIDTH,SHIFT+4) do_rnd_left_i(i_clk, i_ce,
				left_si, rnd_left_i);

	convround #(CWIDTH+IWIDTH+3,OWIDTH,SHIFT+4) do_rnd_right_r(i_clk, i_ce,
				mpy_r, rnd_right_r);

	convround #(CWIDTH+IWIDTH+3,OWIDTH,SHIFT+4) do_rnd_right_i(i_clk, i_ce,
				mpy_i, rnd_right_i);

	always @(posedge i_clk)
	if (i_ce)
	begin
		// First clock, recover all values
		fifo_read <= fifo_left[fifo_read_addr];
		// These values are IWIDTH+CWIDTH+3 bits wide
		// although they only need to be (IWIDTH+1)
		// + (CWIDTH) bits wide.  (We've got two
		// extra bits we need to get rid of.)
		mpy_r <= p_one - p_two;
		mpy_i <= p_three - p_one - p_two;
	end

	reg	[(AUXLEN-1):0]	aux_pipeline;
	initial	aux_pipeline = 0;
	always @(posedge i_clk)
	if (i_reset)
		aux_pipeline <= 0;
	else if (i_ce)
		aux_pipeline <= { aux_pipeline[(AUXLEN-2):0], i_aux };

	initial o_aux = 1'b0;
	always @(posedge i_clk)
	if (i_reset)
		o_aux <= 1'b0;
	else if (i_ce)
	begin
		// Second clock, latch for final clock
		o_aux <= aux_pipeline[AUXLEN-1];
	end

	// As a final step, we pack our outputs into two packed two's
	// complement numbers per output word, so that each output word
	// has (2*OWIDTH) bits in it, with the top half being the real
	// portion and the bottom half being the imaginary portion.
	assign	o_left = { rnd_left_r, rnd_left_i };
	assign	o_right= { rnd_right_r,rnd_right_i};




























































































































































































































































































 // FORMAL
endmodule
////////////////////////////////////////////////////////////////////////////////
//
// Filename: 	convround.v
//
// Project:	A General Purpose Pipelined FFT Implementation
//
// Purpose:	A convergent rounding routine, also known as banker's
//		rounding, Dutch rounding, Gaussian rounding, unbiased
//	rounding, or ... more, at least according to Wikipedia.
//
//	This form of rounding works by rounding, when the direction is in
//	question, towards the nearest even value.
//
//
// Creator:	Dan Gisselquist, Ph.D.
//		Gisselquist Technology, LLC
//
////////////////////////////////////////////////////////////////////////////////
//
// Copyright (C) 2015-2020, Gisselquist Technology, LLC
//
// This file is part of the general purpose pipelined FFT project.
//
// The pipelined FFT project is free software (firmware): you can redistribute
// it and/or modify it under the terms of the GNU Lesser General Public License
// as published by the Free Software Foundation, either version 3 of the
// License, or (at your option) any later version.
//
// The pipelined FFT project is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTIBILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU Lesser
// General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  (It's in the $(ROOT)/doc directory.  Run make
// with no target there if the PDF file isn't present.)  If not, see
// <http://www.gnu.org/licenses/> for a copy.
//
// License:	LGPL, v3, as defined and found on www.gnu.org,
//		http://www.gnu.org/licenses/lgpl.html
//
//
////////////////////////////////////////////////////////////////////////////////
//
//
`default_nettype	none
//
module	convround(i_clk, i_ce, i_val, o_val);
	parameter	IWID=16, OWID=8, SHIFT=0;
	input	wire				i_clk, i_ce;
	input	wire	signed	[(IWID-1):0]	i_val;
	output	reg	signed	[(OWID-1):0]	o_val;

	// Let's deal with three cases to be as general as we can be here
	//
	//	1. The desired output would lose no bits at all
	//	2. One bit would be dropped, so the rounding is simply
	//		adjusting the value to be the nearest even number in
	//		cases of being halfway between two.  If identically
	//		equal to a number, we just leave it as is.
	//	3. Two or more bits would be dropped.  In this case, we round
	//		normally unless we are rounding a value of exactly
	//		halfway between the two.  In the halfway case we round
	//		to the nearest even number.
	generate
	if (IWID == OWID) // In this case, the shift is irrelevant and
	begin // cannot be applied.  No truncation or rounding takes
	// effect here.

		always @(posedge i_clk)
		if (i_ce)	o_val <= i_val[(IWID-1):0];

	end else if (IWID-SHIFT < OWID)
	begin // No truncation or rounding, output drops no bits
	// Instead, we need to stuff the bits in the output

		always @(posedge i_clk)
		if (i_ce)	o_val <= { {(OWID-IWID+SHIFT){i_val[IWID-SHIFT-1]}}, i_val[(IWID-SHIFT-1):0] };

	end else if (IWID-SHIFT == OWID)
	begin // No truncation or rounding, output drops no bits

		always @(posedge i_clk)
		if (i_ce)	o_val <= i_val[(IWID-SHIFT-1):0];

	end else if (IWID-SHIFT-1 == OWID)
	begin // Output drops one bit, can only add one or ... not.
		wire	[(OWID-1):0]	truncated_value, rounded_up;
		wire			last_valid_bit, first_lost_bit;
		assign	truncated_value=i_val[(IWID-1-SHIFT):(IWID-SHIFT-OWID)];
		assign	rounded_up=truncated_value + {{(OWID-1){1'b0}}, 1'b1 };
		assign	last_valid_bit = truncated_value[0];
		assign	first_lost_bit = i_val[0];

		always @(posedge i_clk)
		if (i_ce)
		begin
			if (!first_lost_bit) // Round down / truncate
				o_val <= truncated_value;
			else if (last_valid_bit)// Round up to nearest
				o_val <= rounded_up; // even value
			else // else round down to the nearest
				o_val <= truncated_value; // even value
		end

	end else // If there's more than one bit we are dropping
	begin
		wire	[(OWID-1):0]	truncated_value, rounded_up;
		wire			last_valid_bit, first_lost_bit;

		assign	truncated_value=i_val[(IWID-1-SHIFT):(IWID-SHIFT-OWID)];
		assign	rounded_up=truncated_value + {{(OWID-1){1'b0}}, 1'b1 };
		assign	last_valid_bit = truncated_value[0];
		assign	first_lost_bit = i_val[(IWID-SHIFT-OWID-1)];

		wire	[(IWID-SHIFT-OWID-2):0]	other_lost_bits;
		assign	other_lost_bits = i_val[(IWID-SHIFT-OWID-2):0];

		always @(posedge i_clk)
			if (i_ce)
			begin
				if (!first_lost_bit) // Round down / truncate
					o_val <= truncated_value;
				else if (|other_lost_bits) // Round up to
					o_val <= rounded_up; // closest value
				else if (last_valid_bit) // Round up to
					o_val <= rounded_up; // nearest even
				else	// else round down to nearest even
					o_val <= truncated_value;
			end
	end
	endgenerate

endmodule
////////////////////////////////////////////////////////////////////////////////
//
// Filename:	fftmain.v
//
// Project:	A General Purpose Pipelined FFT Implementation
//
// Purpose:	This is the main module in the General Purpose FPGA FFT
//		implementation.  As such, all other modules are subordinate
//	to this one.  This module accomplish a fixed size Complex FFT on
//	256 data points.
//	The FFT is fully pipelined, and accepts as inputs one complex two's
//	complement sample per clock.
//
// Parameters:
//	i_clk	The clock.  All operations are synchronous with this clock.
//	i_reset	Synchronous reset, active high.  Setting this line will
//			force the reset of all of the internals to this routine.
//			Further, following a reset, the o_sync line will go
//			high the same time the first output sample is valid.
//	i_ce	A clock enable line.  If this line is set, this module
//			will accept one complex input value, and produce
//			one (possibly empty) complex output value.
//	i_sample	The complex input sample.  This value is split
//			into two two's complement numbers, 16 bits each, with
//			the real portion in the high order bits, and the
//			imaginary portion taking the bottom 16 bits.
//	o_result	The output result, of the same format as i_sample,
//			only having 21 bits for each of the real and imaginary
//			components, leading to 42 bits total.
//	o_sync	A one bit output indicating the first sample of the FFT frame.
//			It also indicates the first valid sample out of the FFT
//			on the first frame.
//
// Arguments:	This file was computer generated using the following command
//		line:
//
//		% ./fftgen -v -d ../rtl -f 256 -1 -k 1 -p 0 -n 16 -a ../bench/cpp/fftsize.h
//
//	This core will use hardware accelerated multiplies (DSPs)
//	for 0 of the 8 stages
//
// Creator:	Dan Gisselquist, Ph.D.
//		Gisselquist Technology, LLC
//
////////////////////////////////////////////////////////////////////////////////
//
// Copyright (C) 2015-2020, Gisselquist Technology, LLC
//
// This file is part of the general purpose pipelined FFT project.
//
// The pipelined FFT project is free software (firmware): you can redistribute
// it and/or modify it under the terms of the GNU Lesser General Public License
// as published by the Free Software Foundation, either version 3 of the
// License, or (at your option) any later version.
//
// The pipelined FFT project is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTIBILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU Lesser
// General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  (It's in the $(ROOT)/doc directory.  Run make
// with no target there if the PDF file isn't present.)  If not, see
// <http://www.gnu.org/licenses/> for a copy.
//
// License:	LGPL, v3, as defined and found on www.gnu.org,
//		http://www.gnu.org/licenses/lgpl.html
//
//
////////////////////////////////////////////////////////////////////////////////
//
//
`default_nettype	none
//
//
//
module fftmain(i_clk, i_reset, i_ce,
		i_sample, o_result, o_sync);
	// The bit-width of the input, IWIDTH, output, OWIDTH, and the log
	// of the FFT size.  These are localparams, rather than parameters,
	// because once the core has been generated, they can no longer be
	// changed.  (These values can be adjusted by running the core
	// generator again.)  The reason is simply that these values have
	// been hardwired into the core at several places.
	localparam	IWIDTH=16, OWIDTH=21, LGWIDTH=8;
	//
	input	wire				i_clk, i_reset, i_ce;
	//
	input	wire	[(2*IWIDTH-1):0]	i_sample;
	output	reg	[(2*OWIDTH-1):0]	o_result;
	output	reg				o_sync;


	// Outputs of the FFT, ready for bit reversal.
	wire				br_sync;
	wire	[(2*OWIDTH-1):0]	br_result;


	wire		w_s256;
	wire	[33:0]	w_d256;
	fftstage	#(IWIDTH,IWIDTH+4,17,7,0,
			0, 1, "cmem_256.hex")
		stage_256(i_clk, i_reset, i_ce,
			(!i_reset), i_sample, w_d256, w_s256);


	wire		w_s128;
	wire	[35:0]	w_d128;
	fftstage	#(17,21,18,6,0,
			0, 1, "cmem_128.hex")
		stage_128(i_clk, i_reset, i_ce,
			w_s256, w_d256, w_d128, w_s128);

	wire		w_s64;
	wire	[35:0]	w_d64;
	fftstage	#(18,22,18,5,0,
			0, 1, "cmem_64.hex")
		stage_64(i_clk, i_reset, i_ce,
			w_s128, w_d128, w_d64, w_s64);

	wire		w_s32;
	wire	[37:0]	w_d32;
	fftstage	#(18,22,19,4,0,
			0, 1, "cmem_32.hex")
		stage_32(i_clk, i_reset, i_ce,
			w_s64, w_d64, w_d32, w_s32);

	wire		w_s16;
	wire	[37:0]	w_d16;
	fftstage	#(19,23,19,3,0,
			0, 1, "cmem_16.hex")
		stage_16(i_clk, i_reset, i_ce,
			w_s32, w_d32, w_d16, w_s16);

	wire		w_s8;
	wire	[39:0]	w_d8;
	fftstage	#(19,23,20,2,0,
			0, 1, "cmem_8.hex")
		stage_8(i_clk, i_reset, i_ce,
			w_s16, w_d16, w_d8, w_s8);

	wire		w_s4;
	wire	[39:0]	w_d4;
	qtrstage	#(20,20,8,0,0)	stage_4(i_clk, i_reset, i_ce,
						w_s8, w_d8, w_d4, w_s4);
	wire		w_s2;
	wire	[41:0]	w_d2;
	laststage	#(20,21,1)	stage_2(i_clk, i_reset, i_ce,
					w_s4, w_d4, w_d2, w_s2);


	wire	br_start;
	reg	r_br_started;
	initial	r_br_started = 1'b0;
	always @(posedge i_clk)
		if (i_reset)
			r_br_started <= 1'b0;
		else if (i_ce)
			r_br_started <= r_br_started || w_s2;
	assign	br_start = r_br_started || w_s2;

	// Now for the bit-reversal stage.
	bitreverse	#(8,21)
		revstage(i_clk, i_reset,
			(i_ce & br_start), w_d2,
			br_result, br_sync);


	// Last clock: Register our outputs, we're done.
	initial	o_sync  = 1'b0;
	always @(posedge i_clk)
	if (i_reset)
		o_sync  <= 1'b0;
	else if (i_ce)
		o_sync  <= br_sync;

	always @(posedge i_clk)
	if (i_ce)
		o_result  <= br_result;


endmodule
////////////////////////////////////////////////////////////////////////////////
//
// Filename:	fftstage.v
//
// Project:	A General Purpose Pipelined FFT Implementation
//
// Purpose:	This file is (almost) a Verilog source file.  It is meant to
//		be used by a FFT core compiler to generate FFTs which may be
//	used as part of an FFT core.  Specifically, this file encapsulates
//	the options of an FFT-stage.  For any 2^N length FFT, there shall be
//	(N-1) of these stages.
//
//
// Operation:
// 	Given a stream of values, operate upon them as though they were
// 	value pairs, x[n] and x[n+N/2].  The stream begins when n=0, and ends
// 	when n=N/2-1 (i.e. there's a full set of N values).  When the value
// 	x[0] enters, the synchronization input, i_sync, must be true as well.
//
// 	For this stream, produce outputs
// 	y[n    ] = x[n] + x[n+N/2], and
// 	y[n+N/2] = (x[n] - x[n+N/2]) * c[n],
// 			where c[n] is a complex coefficient found in the
// 			external memory file COEFFILE.
// 	When y[0] is output, a synchronization bit o_sync will be true as
// 	well, otherwise it will be zero.
//
// 	Most of the work to do this is done within the butterfly, whether the
// 	hardware accelerated butterfly (uses a DSP) or not.
//
// Creator:	Dan Gisselquist, Ph.D.
//		Gisselquist Technology, LLC
//
////////////////////////////////////////////////////////////////////////////////
//
// Copyright (C) 2015-2020, Gisselquist Technology, LLC
//
// This file is part of the general purpose pipelined FFT project.
//
// The pipelined FFT project is free software (firmware): you can redistribute
// it and/or modify it under the terms of the GNU Lesser General Public License
// as published by the Free Software Foundation, either version 3 of the
// License, or (at your option) any later version.
//
// The pipelined FFT project is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTIBILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU Lesser
// General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  (It's in the $(ROOT)/doc directory.  Run make
// with no target there if the PDF file isn't present.)  If not, see
// <http://www.gnu.org/licenses/> for a copy.
//
// License:	LGPL, v3, as defined and found on www.gnu.org,
//		http://www.gnu.org/licenses/lgpl.html
//
//
////////////////////////////////////////////////////////////////////////////////
//
//
`default_nettype	none
//
module	fftstage(i_clk, i_reset, i_ce, i_sync, i_data, o_data, o_sync);
	parameter	IWIDTH=16,CWIDTH=20,OWIDTH=17;
	// Parameters specific to the core that should be changed when this
	// core is built ... Note that the minimum LGSPAN (the base two log
	// of the span, or the base two log of the current FFT size) is 3.
	// Smaller spans (i.e. the span of 2) must use the dbl laststage module.
	parameter	LGSPAN=7, BFLYSHIFT=0; // LGWIDTH=8
	parameter	[0:0]	OPT_HWMPY = 1;
	// Clocks per CE.  If your incoming data rate is less than 50% of your
	// clock speed, you can set CKPCE to 2'b10, make sure there's at least
	// one clock between cycles when i_ce is high, and then use two
	// multiplies instead of three.  Setting CKPCE to 2'b11, and insisting
	// on at least two clocks with i_ce low between cycles with i_ce high,
	// then the hardware optimized butterfly code will used one multiply
	// instead of two.
	parameter		CKPCE = 1;
	// The COEFFILE parameter contains the name of the file containing the
	// FFT twiddle factors
	parameter	COEFFILE="cmem_256.hex";




	localparam [0:0] ZERO_ON_IDLE = 1'b0;
 // VERILATOR

	input	wire				i_clk, i_reset, i_ce, i_sync;
	input	wire	[(2*IWIDTH-1):0]	i_data;
	output	reg	[(2*OWIDTH-1):0]	o_data;
	output	reg				o_sync;

	// I am using the prefixes
	// 	ib_*	to reference the inputs to the butterfly, and
	// 	ob_*	to reference the outputs from the butterfly
	reg	wait_for_sync;
	reg	[(2*IWIDTH-1):0]	ib_a, ib_b;
	reg	[(2*CWIDTH-1):0]	ib_c;
	reg	ib_sync;

	reg	b_started;
	wire	ob_sync;
	wire	[(2*OWIDTH-1):0]	ob_a, ob_b;

	// cmem is defined as an array of real and complex values,
	// where the top CWIDTH bits are the real value and the bottom
	// CWIDTH bits are the imaginary value.
	//
	// cmem[i] = { (2^(CWIDTH-2)) * cos(2*pi*i/(2^LGWIDTH)),
	//		(2^(CWIDTH-2)) * sin(2*pi*i/(2^LGWIDTH)) };
	//
	reg	[(2*CWIDTH-1):0]	cmem [0:((1<<LGSPAN)-1)];



	initial	$readmemh(COEFFILE,cmem);



	reg	[(LGSPAN):0]		iaddr;
	reg	[(2*IWIDTH-1):0]	imem	[0:((1<<LGSPAN)-1)];

	reg	[LGSPAN:0]		oaddr;
	reg	[(2*OWIDTH-1):0]	omem	[0:((1<<LGSPAN)-1)];

	reg				idle;
	reg	[(LGSPAN-1):0]		nxt_oaddr;
	reg	[(2*OWIDTH-1):0]	pre_ovalue;

	initial wait_for_sync = 1'b1;
	initial iaddr = 0;
	always @(posedge i_clk)
	if (i_reset)
	begin
		wait_for_sync <= 1'b1;
		iaddr <= 0;
	end else if ((i_ce)&&((!wait_for_sync)||(i_sync)))
	begin
		//
		// First step: Record what we're not ready to use yet
		//
		iaddr <= iaddr + { {(LGSPAN){1'b0}}, 1'b1 };
		wait_for_sync <= 1'b0;
	end
	always @(posedge i_clk) // Need to make certain here that we don't read
	if ((i_ce)&&(!iaddr[LGSPAN])) // and write the same address on
		imem[iaddr[(LGSPAN-1):0]] <= i_data; // the same clk

	//
	// Now, we have all the inputs, so let's feed the butterfly
	//
	// ib_sync is the synchronization bit to the butterfly.  It will
	// be tracked within the butterfly, and used to create the o_sync
	// value when the results from this output are produced
	initial ib_sync = 1'b0;
	always @(posedge i_clk)
	if (i_reset)
		ib_sync <= 1'b0;
	else if (i_ce)
	begin
		// Set the sync to true on the very first
		// valid input in, and hence on the very
		// first valid data out per FFT.
		ib_sync <= (iaddr==(1<<(LGSPAN)));
	end

	// Read the values from our input memory, and use them to feed
	// first of two butterfly inputs
	always	@(posedge i_clk)
	if (i_ce)
	begin
		// One input from memory, ...
		ib_a <= imem[iaddr[(LGSPAN-1):0]];
		// One input clocked in from the top
		ib_b <= i_data;
		// and the coefficient or twiddle factor
		ib_c <= cmem[iaddr[(LGSPAN-1):0]];
	end

	// The idle register is designed to keep track of when an input
	// to the butterfly is important and going to be used.  It's used
	// in a flag following, so that when useful values are placed
	// into the butterfly they'll be non-zero (idle=0), otherwise when
	// the inputs to the butterfly are irrelevant and will be ignored,
	// then (idle=1) those inputs will be set to zero.  This
	// functionality is not designed to be used in operation, but only
	// within a Verilator simulation context when chasing a bug.
	// In this limited environment, the non-zero answers will stand
	// in a trace making it easier to highlight a bug.
	generate if (ZERO_ON_IDLE)
	begin
		initial	idle = 1;
		always @(posedge i_clk)
		if (i_reset)
			idle <= 1'b1;
		else if (i_ce)
			idle <= (!iaddr[LGSPAN])&&(!wait_for_sync);

	end else begin

		always @(*) idle = 0;

	end endgenerate

// For the formal proof, we'll assume the outputs of hwbfly and/or
// butterfly, rather than actually calculating them.  This will simplify
// the proof and (if done properly) will be equivalent.  Be careful of
// defining FORMAL if you want the full logic!

	//
	generate if (OPT_HWMPY)
	begin : HWBFLY

		hwbfly #(.IWIDTH(IWIDTH),.CWIDTH(CWIDTH),.OWIDTH(OWIDTH),
				.CKPCE(CKPCE), .SHIFT(BFLYSHIFT))
			bfly(i_clk, i_reset, i_ce,
				(idle && !i_ce) ? 0:ib_c,
				(idle && !i_ce) ? 0:ib_a,
				(idle && !i_ce) ? 0:ib_b,
				(ib_sync && i_ce),
				ob_a, ob_b, ob_sync);

	end else begin : FWBFLY

		butterfly #(.IWIDTH(IWIDTH),.CWIDTH(CWIDTH),.OWIDTH(OWIDTH),
				.CKPCE(CKPCE),.SHIFT(BFLYSHIFT))
			bfly(i_clk, i_reset, i_ce,
				(idle && !i_ce)?0:ib_c,
				(idle && !i_ce)?0:ib_a,
				(idle && !i_ce)?0:ib_b,
				(ib_sync && i_ce),
				ob_a, ob_b, ob_sync);

	end endgenerate


	//
	// Next step: recover the outputs from the butterfly
	//
	// The first output can go immediately to the output of this routine
	// The second output must wait until this time in the idle cycle
	// oaddr is the output memory address, keeping track of where we are
	// in this output cycle.
	initial oaddr     = 0;
	initial o_sync    = 0;
	initial b_started = 0;
	always @(posedge i_clk)
	if (i_reset)
	begin
		oaddr     <= 0;
		o_sync    <= 0;
		// b_started will be true once we've seen the first ob_sync
		b_started <= 0;
	end else if (i_ce)
	begin
		o_sync <= (!oaddr[LGSPAN])?ob_sync : 1'b0;
		if (ob_sync||b_started)
			oaddr <= oaddr + 1'b1;
		if ((ob_sync)&&(!oaddr[LGSPAN]))
			// If b_started is true, then a butterfly output
			// is available
			b_started <= 1'b1;
	end

	always @(posedge i_clk)
	if (i_ce)
		nxt_oaddr[0] <= oaddr[0];
	generate if (LGSPAN>1)
	begin

		always @(posedge i_clk)
		if (i_ce)
			nxt_oaddr[LGSPAN-1:1] <= oaddr[LGSPAN-1:1] + 1'b1;

	end endgenerate

	// Only write to the memory on the first half of the outputs
	// We'll use the memory value on the second half of the outputs
	always @(posedge i_clk)
	if ((i_ce)&&(!oaddr[LGSPAN]))
		omem[oaddr[(LGSPAN-1):0]] <= ob_b;

	always @(posedge i_clk)
	if (i_ce)
		pre_ovalue <= omem[nxt_oaddr[(LGSPAN-1):0]];

	always @(posedge i_clk)
	if (i_ce)
		o_data <= (!oaddr[LGSPAN]) ? ob_a : pre_ovalue;


























































































































































endmodule
////////////////////////////////////////////////////////////////////////////////
//
// Filename:	hwbfly.v
//
// Project:	A General Purpose Pipelined FFT Implementation
//
// Purpose:	This routine is identical to the butterfly.v routine found
//		in 'butterfly.v', save only that it uses the verilog
//	operator '*' in hopes that the synthesizer would be able to optimize
//	it with hardware resources.
//
//	It is understood that a hardware multiply can complete its operation in
//	a single clock.
//
// Operation:
//
//	Given two inputs, A (i_left) and B (i_right), and a complex
//	coefficient C (i_coeff), return two outputs, O1 and O2, where:
//
//		O1 = A + B, and
//		O2 = (A - B)*C
//
//	This operation is commonly known as a Decimation in Frequency (DIF)
//	Radix-2 Butterfly.
//	O1 and O2 are rounded before being returned in (o_left) and o_right
//	to OWIDTH bits.  If SHIFT is one, an extra bit is dropped from these
//	values during the rounding process.
//
//	Further, since these outputs will take some number of clocks to
//	calculate, we'll pipe a value (i_aux) through the system and return
//	it with the results (o_aux), so you can synchronize to the outgoing
//	output stream.
//
//
// Creator:	Dan Gisselquist, Ph.D.
//		Gisselquist Technology, LLC
//
////////////////////////////////////////////////////////////////////////////////
//
// Copyright (C) 2015-2020, Gisselquist Technology, LLC
//
// This file is part of the general purpose pipelined FFT project.
//
// The pipelined FFT project is free software (firmware): you can redistribute
// it and/or modify it under the terms of the GNU Lesser General Public License
// as published by the Free Software Foundation, either version 3 of the
// License, or (at your option) any later version.
//
// The pipelined FFT project is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTIBILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU Lesser
// General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  (It's in the $(ROOT)/doc directory.  Run make
// with no target there if the PDF file isn't present.)  If not, see
// <http://www.gnu.org/licenses/> for a copy.
//
// License:	LGPL, v3, as defined and found on www.gnu.org,
//		http://www.gnu.org/licenses/lgpl.html
//
//
////////////////////////////////////////////////////////////////////////////////
//
//
`default_nettype	none
//
module	hwbfly(i_clk, i_reset, i_ce, i_coef, i_left, i_right, i_aux,
		o_left, o_right, o_aux);
	// Public changeable parameters ...
	//	- IWIDTH, number of bits in each component of the input
	//	- CWIDTH, number of bits in each component of the twiddle factor
	//	- OWIDTH, number of bits in each component of the output
	parameter IWIDTH=16,CWIDTH=IWIDTH+4,OWIDTH=IWIDTH+1;
	// Drop an additional bit on the output?
	parameter		SHIFT=0;
	// The number of clocks per clock enable, 1, 2, or 3.
	parameter	[1:0]	CKPCE=1;
	//
	input	wire	i_clk, i_reset, i_ce;
	input	wire	[(2*CWIDTH-1):0]	i_coef;
	input	wire	[(2*IWIDTH-1):0]	i_left, i_right;
	input	wire	i_aux;
	output	wire	[(2*OWIDTH-1):0]	o_left, o_right;
	output	reg	o_aux;


	reg	[(2*IWIDTH-1):0]	r_left, r_right;
	reg				r_aux, r_aux_2;
	reg	[(2*CWIDTH-1):0]	r_coef;
	wire	signed	[(IWIDTH-1):0]	r_left_r, r_left_i, r_right_r, r_right_i;
	assign	r_left_r  = r_left[ (2*IWIDTH-1):(IWIDTH)];
	assign	r_left_i  = r_left[ (IWIDTH-1):0];
	assign	r_right_r = r_right[(2*IWIDTH-1):(IWIDTH)];
	assign	r_right_i = r_right[(IWIDTH-1):0];
	reg	signed	[(CWIDTH-1):0]	ir_coef_r, ir_coef_i;

	reg	signed	[(IWIDTH):0]	r_sum_r, r_sum_i, r_dif_r, r_dif_i;

	reg	[(2*IWIDTH+2):0]	leftv, leftvv;

	// Set up the input to the multiply
	initial r_aux   = 1'b0;
	initial r_aux_2 = 1'b0;
	always @(posedge i_clk)
		if (i_reset)
		begin
			r_aux <= 1'b0;
			r_aux_2 <= 1'b0;
		end else if (i_ce)
		begin
			// One clock just latches the inputs
			r_aux <= i_aux;
			// Next clock adds/subtracts
			// Other inputs are simply delayed on second clock
			r_aux_2 <= r_aux;
		end
	always @(posedge i_clk)
		if (i_ce)
		begin
			// One clock just latches the inputs
			r_left <= i_left;	// No change in # of bits
			r_right <= i_right;
			r_coef  <= i_coef;
			// Next clock adds/subtracts
			r_sum_r <= r_left_r + r_right_r; // Now IWIDTH+1 bits
			r_sum_i <= r_left_i + r_right_i;
			r_dif_r <= r_left_r - r_right_r;
			r_dif_i <= r_left_i - r_right_i;
			// Other inputs are simply delayed on second clock
			ir_coef_r <= r_coef[(2*CWIDTH-1):CWIDTH];
			ir_coef_i <= r_coef[(CWIDTH-1):0];
		end


	// See comments in the butterfly.v source file for a discussion of
	// these operations and the appropriate bit widths.

	wire	signed	[((IWIDTH+1)+(CWIDTH)-1):0]	p_one, p_two;
	wire	signed	[((IWIDTH+2)+(CWIDTH+1)-1):0]	p_three;

	initial leftv    = 0;
	initial leftvv   = 0;
	always @(posedge i_clk)
		if (i_reset)
		begin
			leftv <= 0;
			leftvv <= 0;
		end else if (i_ce)
		begin
			// Second clock, pipeline = 1
			leftv <= { r_aux_2, r_sum_r, r_sum_i };

			// Third clock, pipeline = 3
			//   As desired, each of these lines infers a DSP48
			leftvv <= leftv;
		end

	generate if (CKPCE <= 1)
	begin : CKPCE_ONE
		// Coefficient multiply inputs
		reg	signed	[(CWIDTH-1):0]	p1c_in, p2c_in;
		// Data multiply inputs
		reg	signed	[(IWIDTH):0]	p1d_in, p2d_in;
		// Product 3, coefficient input
		reg	signed	[(CWIDTH):0]	p3c_in;
		// Product 3, data input
		reg	signed	[(IWIDTH+1):0]	p3d_in;

		reg	signed	[((IWIDTH+1)+(CWIDTH)-1):0]	rp_one, rp_two;
		reg	signed	[((IWIDTH+2)+(CWIDTH+1)-1):0]	rp_three;

		always @(posedge i_clk)
		if (i_ce)
		begin
			// Second clock, pipeline = 1
			p1c_in <= ir_coef_r;
			p2c_in <= ir_coef_i;
			p1d_in <= r_dif_r;
			p2d_in <= r_dif_i;
			p3c_in <= ir_coef_i + ir_coef_r;
			p3d_in <= r_dif_r + r_dif_i;
		end


		always @(posedge i_clk)
		if (i_ce)
		begin
			// Third clock, pipeline = 3
			//   As desired, each of these lines infers a DSP48
			rp_one   <= p1c_in * p1d_in;
			rp_two   <= p2c_in * p2d_in;
			rp_three <= p3c_in * p3d_in;
		end


















 // FORMAL

		assign	p_one   = rp_one;
		assign	p_two   = rp_two;
		assign	p_three = rp_three;

	end else if (CKPCE <= 2)
	begin : CKPCE_TWO
		// Coefficient multiply inputs
		reg		[2*(CWIDTH)-1:0]	mpy_pipe_c;
		// Data multiply inputs
		reg		[2*(IWIDTH+1)-1:0]	mpy_pipe_d;
		wire	signed	[(CWIDTH-1):0]	mpy_pipe_vc;
		wire	signed	[(IWIDTH):0]	mpy_pipe_vd;
		//
		reg	signed	[(CWIDTH+1)-1:0]	mpy_cof_sum;
		reg	signed	[(IWIDTH+2)-1:0]	mpy_dif_sum;

		assign	mpy_pipe_vc =  mpy_pipe_c[2*(CWIDTH)-1:CWIDTH];
		assign	mpy_pipe_vd =  mpy_pipe_d[2*(IWIDTH+1)-1:IWIDTH+1];

		reg			mpy_pipe_v;
		reg			ce_phase;

		reg	signed	[(CWIDTH+IWIDTH+1)-1:0]	mpy_pipe_out;
		reg	signed [IWIDTH+CWIDTH+3-1:0]	longmpy;


		initial	ce_phase = 1'b1;
		always @(posedge i_clk)
		if (i_reset)
			ce_phase <= 1'b1;
		else if (i_ce)
			ce_phase <= 1'b0;
		else
			ce_phase <= 1'b1;

		always @(*)
			mpy_pipe_v = (i_ce)||(!ce_phase);

		always @(posedge i_clk)
		if (!ce_phase)
		begin
			// Pre-clock
			mpy_pipe_c[2*CWIDTH-1:0] <=
					{ ir_coef_r, ir_coef_i };
			mpy_pipe_d[2*(IWIDTH+1)-1:0] <=
					{ r_dif_r, r_dif_i };

			mpy_cof_sum  <= ir_coef_i + ir_coef_r;
			mpy_dif_sum <= r_dif_r + r_dif_i;

		end else if (i_ce)
		begin
			// First clock
			mpy_pipe_c[2*(CWIDTH)-1:0] <= {
				mpy_pipe_c[(CWIDTH)-1:0], {(CWIDTH){1'b0}} };
			mpy_pipe_d[2*(IWIDTH+1)-1:0] <= {
				mpy_pipe_d[(IWIDTH+1)-1:0], {(IWIDTH+1){1'b0}} };
		end


		always @(posedge i_clk)
		if (i_ce) // First clock
			longmpy <= mpy_cof_sum * mpy_dif_sum;

		always @(posedge i_clk)
		if (mpy_pipe_v)
			mpy_pipe_out <= mpy_pipe_vc * mpy_pipe_vd;




















		reg	signed	[((IWIDTH+1)+(CWIDTH)-1):0]	rp_one,
							rp2_one, rp_two;
		reg	signed	[((IWIDTH+2)+(CWIDTH+1)-1):0]	rp_three;

		always @(posedge i_clk)
		if (!ce_phase) // 1.5 clock
			rp_one <= mpy_pipe_out;
		always @(posedge i_clk)
		if (i_ce) // two clocks
			rp_two <= mpy_pipe_out;
		always @(posedge i_clk)
		if (i_ce) // Second clock
			rp_three<= longmpy;
		always @(posedge i_clk)
		if (i_ce)
			rp2_one<= rp_one;

		assign	p_one  = rp2_one;
		assign	p_two  = rp_two;
		assign	p_three= rp_three;

	end else if (CKPCE <= 2'b11)
	begin : CKPCE_THREE
		// Coefficient multiply inputs
		reg		[3*(CWIDTH+1)-1:0]	mpy_pipe_c;
		// Data multiply inputs
		reg		[3*(IWIDTH+2)-1:0]	mpy_pipe_d;
		wire	signed	[(CWIDTH):0]	mpy_pipe_vc;
		wire	signed	[(IWIDTH+1):0]	mpy_pipe_vd;

		assign	mpy_pipe_vc =  mpy_pipe_c[3*(CWIDTH+1)-1:2*(CWIDTH+1)];
		assign	mpy_pipe_vd =  mpy_pipe_d[3*(IWIDTH+2)-1:2*(IWIDTH+2)];

		reg			mpy_pipe_v;
		reg		[2:0]	ce_phase;

		reg	signed	[  (CWIDTH+IWIDTH+3)-1:0]	mpy_pipe_out;

		initial	ce_phase = 3'b011;
		always @(posedge i_clk)
		if (i_reset)
			ce_phase <= 3'b011;
		else if (i_ce)
			ce_phase <= 3'b000;
		else if (ce_phase != 3'b011)
			ce_phase <= ce_phase + 1'b1;

		always @(*)
			mpy_pipe_v = (i_ce)||(ce_phase < 3'b010);

		always @(posedge i_clk)
			if (ce_phase == 3'b000)
			begin
				// Second clock
				mpy_pipe_c[3*(CWIDTH+1)-1:(CWIDTH+1)] <= {
					ir_coef_r[CWIDTH-1], ir_coef_r,
					ir_coef_i[CWIDTH-1], ir_coef_i };
				mpy_pipe_c[CWIDTH:0] <= ir_coef_i + ir_coef_r;
				mpy_pipe_d[3*(IWIDTH+2)-1:(IWIDTH+2)] <= {
					r_dif_r[IWIDTH], r_dif_r,
					r_dif_i[IWIDTH], r_dif_i };
				mpy_pipe_d[(IWIDTH+2)-1:0] <= r_dif_r + r_dif_i;

			end else if (mpy_pipe_v)
			begin
				mpy_pipe_c[3*(CWIDTH+1)-1:0] <= {
					mpy_pipe_c[2*(CWIDTH+1)-1:0], {(CWIDTH+1){1'b0}} };
				mpy_pipe_d[3*(IWIDTH+2)-1:0] <= {
					mpy_pipe_d[2*(IWIDTH+2)-1:0], {(IWIDTH+2){1'b0}} };
			end


		always @(posedge i_clk)
			if (mpy_pipe_v)
				mpy_pipe_out <= mpy_pipe_vc * mpy_pipe_vd;









	// FORMAL

		reg	signed	[((IWIDTH+1)+(CWIDTH)-1):0]	rp_one, rp_two,
						rp2_one, rp2_two;
		reg	signed	[((IWIDTH+2)+(CWIDTH+1)-1):0]	rp_three, rp2_three;

		always @(posedge i_clk)
		if(i_ce)
			rp_one <= mpy_pipe_out[(CWIDTH+IWIDTH):0];
		always @(posedge i_clk)
		if(ce_phase == 3'b000)
			rp_two <= mpy_pipe_out[(CWIDTH+IWIDTH):0];
		always @(posedge i_clk)
		if(ce_phase == 3'b001)
			rp_three <= mpy_pipe_out;
		always @(posedge i_clk)
		if (i_ce)
		begin
			rp2_one<= rp_one;
			rp2_two<= rp_two;
			rp2_three<= rp_three;
		end
		assign	p_one	= rp2_one;
		assign	p_two	= rp2_two;
		assign	p_three	= rp2_three;

	end endgenerate
	wire	signed	[((IWIDTH+2)+(CWIDTH+1)-1):0]	w_one, w_two;
	assign	w_one = { {(2){p_one[((IWIDTH+1)+(CWIDTH)-1)]}}, p_one };
	assign	w_two = { {(2){p_two[((IWIDTH+1)+(CWIDTH)-1)]}}, p_two };

	// These values are held in memory and delayed during the
	// multiply.  Here, we recover them.  During the multiply,
	// values were multiplied by 2^(CWIDTH-2)*exp{-j*2*pi*...},
	// therefore, the left_x values need to be right shifted by
	// CWIDTH-2 as well.  The additional bits come from a sign
	// extension.
	wire	aux_s;
	wire	signed	[(IWIDTH+CWIDTH):0]	left_si, left_sr;
	reg		[(2*IWIDTH+2):0]	left_saved;
	assign	left_sr = { {2{left_saved[2*(IWIDTH+1)-1]}}, left_saved[(2*(IWIDTH+1)-1):(IWIDTH+1)], {(CWIDTH-2){1'b0}} };
	assign	left_si = { {2{left_saved[(IWIDTH+1)-1]}}, left_saved[((IWIDTH+1)-1):0], {(CWIDTH-2){1'b0}} };
	assign	aux_s = left_saved[2*IWIDTH+2];

	(* use_dsp48="no" *)
	reg	signed	[(CWIDTH+IWIDTH+3-1):0]	mpy_r, mpy_i;

	initial left_saved = 0;
	initial o_aux      = 1'b0;
	always @(posedge i_clk)
		if (i_reset)
		begin
			left_saved <= 0;
			o_aux <= 1'b0;
		end else if (i_ce)
		begin
			// First clock, recover all values
			left_saved <= leftvv;

			// Second clock, round and latch for final clock
			o_aux <= aux_s;
		end
	always @(posedge i_clk)
		if (i_ce)
		begin
			// These values are IWIDTH+CWIDTH+3 bits wide
			// although they only need to be (IWIDTH+1)
			// + (CWIDTH) bits wide.  (We've got two
			// extra bits we need to get rid of.)

			// These two lines also infer DSP48's.
			// To keep from using extra DSP48 resources,
			// they are prevented from using DSP48's
			// by the (* use_dsp48 ... *) comment above.
			mpy_r <= w_one - w_two;
			mpy_i <= p_three - w_one - w_two;
		end

	// Round the results
	wire	signed	[(OWIDTH-1):0]	rnd_left_r, rnd_left_i, rnd_right_r, rnd_right_i;

	convround #(CWIDTH+IWIDTH+1,OWIDTH,SHIFT+2) do_rnd_left_r(i_clk, i_ce,
				left_sr, rnd_left_r);

	convround #(CWIDTH+IWIDTH+1,OWIDTH,SHIFT+2) do_rnd_left_i(i_clk, i_ce,
				left_si, rnd_left_i);

	convround #(CWIDTH+IWIDTH+3,OWIDTH,SHIFT+4) do_rnd_right_r(i_clk, i_ce,
				mpy_r, rnd_right_r);

	convround #(CWIDTH+IWIDTH+3,OWIDTH,SHIFT+4) do_rnd_right_i(i_clk, i_ce,
				mpy_i, rnd_right_i);

	// As a final step, we pack our outputs into two packed two's
	// complement numbers per output word, so that each output word
	// has (2*OWIDTH) bits in it, with the top half being the real
	// portion and the bottom half being the imaginary portion.
	assign	o_left = { rnd_left_r, rnd_left_i };
	assign	o_right= { rnd_right_r,rnd_right_i};






























































































































































































































 // FORMAL
endmodule
////////////////////////////////////////////////////////////////////////////////
//
// Filename:	ifftmain.v
//
// Project:	A General Purpose Pipelined FFT Implementation
//
// Purpose:	This is the main module in the General Purpose FPGA FFT
//		implementation.  As such, all other modules are subordinate
//	to this one.  This module accomplish a fixed size Complex FFT on
//	256 data points.
//	The FFT is fully pipelined, and accepts as inputs one complex two's
//	complement sample per clock.
//
// Parameters:
//	i_clk	The clock.  All operations are synchronous with this clock.
//	i_reset	Synchronous reset, active high.  Setting this line will
//			force the reset of all of the internals to this routine.
//			Further, following a reset, the o_sync line will go
//			high the same time the first output sample is valid.
//	i_ce	A clock enable line.  If this line is set, this module
//			will accept one complex input value, and produce
//			one (possibly empty) complex output value.
//	i_sample	The complex input sample.  This value is split
//			into two two's complement numbers, 16 bits each, with
//			the real portion in the high order bits, and the
//			imaginary portion taking the bottom 16 bits.
//	o_result	The output result, of the same format as i_sample,
//			only having 21 bits for each of the real and imaginary
//			components, leading to 42 bits total.
//	o_sync	A one bit output indicating the first sample of the FFT frame.
//			It also indicates the first valid sample out of the FFT
//			on the first frame.
//
// Arguments:	This file was computer generated using the following command
//		line:
//
//		% ./fftgen -i -d ../rtl -f 256 -1 -k 1 -p 0 -n 16 -a ../bench/cpp/ifftsize.h
//
//	This core will use hardware accelerated multiplies (DSPs)
//	for 0 of the 8 stages
//
// Creator:	Dan Gisselquist, Ph.D.
//		Gisselquist Technology, LLC
//
////////////////////////////////////////////////////////////////////////////////
//
// Copyright (C) 2015-2020, Gisselquist Technology, LLC
//
// This file is part of the general purpose pipelined FFT project.
//
// The pipelined FFT project is free software (firmware): you can redistribute
// it and/or modify it under the terms of the GNU Lesser General Public License
// as published by the Free Software Foundation, either version 3 of the
// License, or (at your option) any later version.
//
// The pipelined FFT project is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTIBILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU Lesser
// General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  (It's in the $(ROOT)/doc directory.  Run make
// with no target there if the PDF file isn't present.)  If not, see
// <http://www.gnu.org/licenses/> for a copy.
//
// License:	LGPL, v3, as defined and found on www.gnu.org,
//		http://www.gnu.org/licenses/lgpl.html
//
//
////////////////////////////////////////////////////////////////////////////////
//
//
`default_nettype	none
//
//
//
module ifftmain(i_clk, i_reset, i_ce,
		i_sample, o_result, o_sync);
	// The bit-width of the input, IWIDTH, output, OWIDTH, and the log
	// of the FFT size.  These are localparams, rather than parameters,
	// because once the core has been generated, they can no longer be
	// changed.  (These values can be adjusted by running the core
	// generator again.)  The reason is simply that these values have
	// been hardwired into the core at several places.
	localparam	IWIDTH=16, OWIDTH=21, LGWIDTH=8;
	//
	input	wire				i_clk, i_reset, i_ce;
	//
	input	wire	[(2*IWIDTH-1):0]	i_sample;
	output	reg	[(2*OWIDTH-1):0]	o_result;
	output	reg				o_sync;


	// Outputs of the FFT, ready for bit reversal.
	wire				br_sync;
	wire	[(2*OWIDTH-1):0]	br_result;


	wire		w_s256;
	wire	[33:0]	w_d256;
	fftstage	#(IWIDTH,IWIDTH+4,17,7,0,
			0, 1, "icmem_256.hex")
		stage_256(i_clk, i_reset, i_ce,
			(!i_reset), i_sample, w_d256, w_s256);


	wire		w_s128;
	wire	[35:0]	w_d128;
	fftstage	#(17,21,18,6,0,
			0, 1, "icmem_128.hex")
		stage_128(i_clk, i_reset, i_ce,
			w_s256, w_d256, w_d128, w_s128);

	wire		w_s64;
	wire	[35:0]	w_d64;
	fftstage	#(18,22,18,5,0,
			0, 1, "icmem_64.hex")
		stage_64(i_clk, i_reset, i_ce,
			w_s128, w_d128, w_d64, w_s64);

	wire		w_s32;
	wire	[37:0]	w_d32;
	fftstage	#(18,22,19,4,0,
			0, 1, "icmem_32.hex")
		stage_32(i_clk, i_reset, i_ce,
			w_s64, w_d64, w_d32, w_s32);

	wire		w_s16;
	wire	[37:0]	w_d16;
	fftstage	#(19,23,19,3,0,
			0, 1, "icmem_16.hex")
		stage_16(i_clk, i_reset, i_ce,
			w_s32, w_d32, w_d16, w_s16);

	wire		w_s8;
	wire	[39:0]	w_d8;
	fftstage	#(19,23,20,2,0,
			0, 1, "icmem_8.hex")
		stage_8(i_clk, i_reset, i_ce,
			w_s16, w_d16, w_d8, w_s8);

	wire		w_s4;
	wire	[39:0]	w_d4;
	qtrstage	#(20,20,8,1,0)	stage_4(i_clk, i_reset, i_ce,
						w_s8, w_d8, w_d4, w_s4);
	wire		w_s2;
	wire	[41:0]	w_d2;
	laststage	#(20,21,1)	stage_2(i_clk, i_reset, i_ce,
					w_s4, w_d4, w_d2, w_s2);


	wire	br_start;
	reg	r_br_started;
	initial	r_br_started = 1'b0;
	always @(posedge i_clk)
		if (i_reset)
			r_br_started <= 1'b0;
		else if (i_ce)
			r_br_started <= r_br_started || w_s2;
	assign	br_start = r_br_started || w_s2;

	// Now for the bit-reversal stage.
	bitreverse	#(8,21)
		revstage(i_clk, i_reset,
			(i_ce & br_start), w_d2,
			br_result, br_sync);


	// Last clock: Register our outputs, we're done.
	initial	o_sync  = 1'b0;
	always @(posedge i_clk)
	if (i_reset)
		o_sync  <= 1'b0;
	else if (i_ce)
		o_sync  <= br_sync;

	always @(posedge i_clk)
	if (i_ce)
		o_result  <= br_result;


endmodule
////////////////////////////////////////////////////////////////////////////////
//
// Filename:	laststage.v
//
// Project:	A General Purpose Pipelined FFT Implementation
//
// Purpose:	This is part of an FPGA implementation that will process
//		the final stage of a decimate-in-frequency FFT, running
//	through the data at one sample per clock.
//
//
// Creator:	Dan Gisselquist, Ph.D.
//		Gisselquist Technology, LLC
//
////////////////////////////////////////////////////////////////////////////////
//
// Copyright (C) 2015-2020, Gisselquist Technology, LLC
//
// This file is part of the general purpose pipelined FFT project.
//
// The pipelined FFT project is free software (firmware): you can redistribute
// it and/or modify it under the terms of the GNU Lesser General Public License
// as published by the Free Software Foundation, either version 3 of the
// License, or (at your option) any later version.
//
// The pipelined FFT project is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTIBILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU Lesser
// General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  (It's in the $(ROOT)/doc directory.  Run make
// with no target there if the PDF file isn't present.)  If not, see
// <http://www.gnu.org/licenses/> for a copy.
//
// License:	LGPL, v3, as defined and found on www.gnu.org,
//		http://www.gnu.org/licenses/lgpl.html
//
//
////////////////////////////////////////////////////////////////////////////////
//
//
`default_nettype	none
//
module	laststage(i_clk, i_reset, i_ce, i_sync, i_val, o_val, o_sync);
	parameter	IWIDTH=16,OWIDTH=IWIDTH+1, SHIFT=0;
	input	wire				i_clk, i_reset, i_ce, i_sync;
	input	wire	[(2*IWIDTH-1):0]	i_val;
	output	wire	[(2*OWIDTH-1):0]	o_val;
	output	reg				o_sync;

	reg	signed	[(IWIDTH-1):0]	m_r, m_i;
	wire	signed	[(IWIDTH-1):0]	i_r, i_i;

	assign	i_r = i_val[(2*IWIDTH-1):(IWIDTH)]; 
	assign	i_i = i_val[(IWIDTH-1):0]; 

	// Don't forget that we accumulate a bit by adding two values
	// together. Therefore our intermediate value must have one more
	// bit than the two originals.
	reg	signed	[(IWIDTH):0]	rnd_r, rnd_i, sto_r, sto_i;
	reg				wait_for_sync, stage;
	reg		[1:0]		sync_pipe;

	initial	wait_for_sync = 1'b1;
	initial	stage         = 1'b0;
	always @(posedge i_clk)
	if (i_reset)
	begin
		wait_for_sync <= 1'b1;
		stage         <= 1'b0;
	end else if ((i_ce)&&((!wait_for_sync)||(i_sync))&&(!stage))
	begin
		wait_for_sync <= 1'b0;
		//
		stage <= 1'b1;
		//
	end else if (i_ce)
		stage <= 1'b0;

	initial	sync_pipe = 0;
	always @(posedge i_clk)
	if (i_reset)
		sync_pipe <= 0;
	else if (i_ce)
		sync_pipe <= { sync_pipe[0], i_sync };

	initial	o_sync = 1'b0;
	always @(posedge i_clk)
	if (i_reset)
		o_sync <= 1'b0;
	else if (i_ce)
		o_sync <= sync_pipe[1];

	always @(posedge i_clk)
	if (i_ce)
	begin
		if (!stage)
		begin
			// Clock 1
			m_r <= i_r;
			m_i <= i_i;
			// Clock 3
			rnd_r <= sto_r;
			rnd_i <= sto_i;
			//
		end else begin
			// Clock 2
			rnd_r <= m_r + i_r;
			rnd_i <= m_i + i_i;
			//
			sto_r <= m_r - i_r;
			sto_i <= m_i - i_i;
			//
		end
	end

	// Now that we have our results, let's round them and report them
	wire	signed	[(OWIDTH-1):0]	o_r, o_i;

	convround #(IWIDTH+1,OWIDTH,SHIFT) do_rnd_r(i_clk, i_ce, rnd_r, o_r);
	convround #(IWIDTH+1,OWIDTH,SHIFT) do_rnd_i(i_clk, i_ce, rnd_i, o_i);

	assign	o_val  = { o_r, o_i };

















































































 // FORMAL
endmodule
////////////////////////////////////////////////////////////////////////////////
//
// Filename: 	../rtl/longbimpy.v
//
// Project:	A General Purpose Pipelined FFT Implementation
//
// Purpose:	A portable shift and add multiply, built with the knowledge
//	of the existence of a six bit LUT and carry chain.  That knowledge
//	allows us to multiply two bits from one value at a time against all
//	of the bits of the other value.  This sub multiply is called the
//	bimpy.
//
//	For minimal processing delay, make the first parameter the one with
//	the least bits, so that AWIDTH <= BWIDTH.
//
//
//
// Creator:	Dan Gisselquist, Ph.D.
//		Gisselquist Technology, LLC
//
////////////////////////////////////////////////////////////////////////////////
//
// Copyright (C) 2015-2020, Gisselquist Technology, LLC
//
// This file is part of the general purpose pipelined FFT project.
//
// The pipelined FFT project is free software (firmware): you can redistribute
// it and/or modify it under the terms of the GNU Lesser General Public License
// as published by the Free Software Foundation, either version 3 of the
// License, or (at your option) any later version.
//
// The pipelined FFT project is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTIBILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU Lesser
// General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  (It's in the $(ROOT)/doc directory.  Run make
// with no target there if the PDF file isn't present.)  If not, see
// <http://www.gnu.org/licenses/> for a copy.
//
// License:	LGPL, v3, as defined and found on www.gnu.org,
//		http://www.gnu.org/licenses/lgpl.html
//
//
////////////////////////////////////////////////////////////////////////////////
//
//
`default_nettype	none
//
module	longbimpy(i_clk, i_ce, i_a_unsorted, i_b_unsorted, o_r



		);
	parameter	IAW=8,	// The width of i_a, min width is 5
			IBW=12,	// The width of i_b, can be anything
			// The following three parameters should not be changed
			// by any implementation, but are based upon hardware
			// and the above values:
			OW=IAW+IBW;	// The output width
	localparam	AW = (IAW<IBW) ? IAW : IBW,
			BW = (IAW<IBW) ? IBW : IAW,
			IW=(AW+1)&(-2),	// Internal width of A
			LUTB=2,	// How many bits we can multiply by at once
			TLEN=(AW+(LUTB-1))/LUTB; // Nmbr of rows in our tableau
	input	wire			i_clk, i_ce;
	input	wire	[(IAW-1):0]	i_a_unsorted;
	input	wire	[(IBW-1):0]	i_b_unsorted;
	output	reg	[(AW+BW-1):0]	o_r;






	//
	// Swap parameter order, so that AW <= BW -- for performance
	// reasons
	wire	[AW-1:0]	i_a;
	wire	[BW-1:0]	i_b;
	generate if (IAW <= IBW)
	begin : NO_PARAM_CHANGE_I
		assign i_a = i_a_unsorted;
		assign i_b = i_b_unsorted;
	end else begin : SWAP_PARAMETERS_I
		assign i_a = i_b_unsorted;
		assign i_b = i_a_unsorted;
	end endgenerate

	reg	[(IW-1):0]	u_a;
	reg	[(BW-1):0]	u_b;
	reg			sgn;

	reg	[(IW-1-2*(LUTB)):0]	r_a[0:(TLEN-3)];
	reg	[(BW-1):0]		r_b[0:(TLEN-3)];
	reg	[(TLEN-1):0]		r_s;
	reg	[(IW+BW-1):0]		acc[0:(TLEN-2)];
	genvar k;

	// First step:
	// Switch to unsigned arithmetic for our multiply, keeping track
	// of the along the way.  We'll then add the sign again later at
	// the end.
	//
	// If we were forced to stay within two's complement arithmetic,
	// taking the absolute value here would require an additional bit.
	// However, because our results are now unsigned, we can stay
	// within the number of bits given (for now).
	initial u_a = 0;
	generate if (IW > AW)
	begin : ABS_AND_ADD_BIT_TO_A
		always @(posedge i_clk)
			if (i_ce)
				u_a <= { 1'b0, (i_a[AW-1])?(-i_a):(i_a) };
	end else begin : ABS_A
		always @(posedge i_clk)
			if (i_ce)
				u_a <= (i_a[AW-1])?(-i_a):(i_a);
	end endgenerate

	initial sgn = 0;
	initial u_b = 0;
	always @(posedge i_clk)
	if (i_ce)
	begin : ABS_B
		u_b <= (i_b[BW-1])?(-i_b):(i_b);
		sgn <= i_a[AW-1] ^ i_b[BW-1];
	end

	wire	[(BW+LUTB-1):0]	pr_a, pr_b;

	//
	// Second step: First two 2xN products.
	//
	// Since we have no tableau of additions (yet), we can do both
	// of the first two rows at the same time and add them together.
	// For the next round, we'll then have a previous sum to accumulate
	// with new and subsequent product, and so only do one product at
	// a time can follow this--but the first clock can do two at a time.
	bimpy	#(BW) lmpy_0(i_clk,i_ce,u_a[(  LUTB-1):   0], u_b, pr_a);
	bimpy	#(BW) lmpy_1(i_clk,i_ce,u_a[(2*LUTB-1):LUTB], u_b, pr_b);

	initial r_s    = 0;
	initial r_a[0] = 0;
	initial r_b[0] = 0;
	always @(posedge i_clk)
		if (i_ce) r_a[0] <= u_a[(IW-1):(2*LUTB)];
	always @(posedge i_clk)
		if (i_ce) r_b[0] <= u_b;
	always @(posedge i_clk)
		if (i_ce) r_s <= { r_s[(TLEN-2):0], sgn };

	initial acc[0] = 0;
	always @(posedge i_clk) // One clk after p[0],p[1] become valid
	if (i_ce) acc[0] <= { {(IW-LUTB){1'b0}}, pr_a}
		  +{ {(IW-(2*LUTB)){1'b0}}, pr_b, {(LUTB){1'b0}} };

	generate // Keep track of intermediate values, before multiplying them
	if (TLEN > 3) for(k=0; k<TLEN-3; k=k+1)
	begin : GENCOPIES

		initial r_a[k+1] = 0;
		initial r_b[k+1] = 0;
		always @(posedge i_clk)
		if (i_ce)
		begin
			r_a[k+1] <= { {(LUTB){1'b0}},
				r_a[k][(IW-1-(2*LUTB)):LUTB] };
			r_b[k+1] <= r_b[k];
		end
	end endgenerate

	generate // The actual multiply and accumulate stage
	if (TLEN > 2) for(k=0; k<TLEN-2; k=k+1)
	begin : GENSTAGES
		wire	[(BW+LUTB-1):0] genp;

		// First, the multiply: 2-bits times BW bits
		bimpy #(BW) genmpy(i_clk,i_ce,r_a[k][(LUTB-1):0],r_b[k], genp);

		// Then the accumulate step -- on the next clock
		initial acc[k+1] = 0;
		always @(posedge i_clk)
		if (i_ce)
			acc[k+1] <= acc[k] + {{(IW-LUTB*(k+3)){1'b0}},
				genp, {(LUTB*(k+2)){1'b0}} };
	end endgenerate

	wire	[(IW+BW-1):0]	w_r;
	assign	w_r = (r_s[TLEN-1]) ? (-acc[TLEN-2]) : acc[TLEN-2];

	initial o_r = 0;
	always @(posedge i_clk)
	if (i_ce)
		o_r <= w_r[(AW+BW-1):0];

	generate if (IW > AW)
	begin : VUNUSED
		// verilator lint_off UNUSED
		wire	[(IW-AW)-1:0]	unused;
		assign	unused = w_r[(IW+BW-1):(AW+BW)];
		// verilator lint_on UNUSED
	end endgenerate



































































































































































































































	// FORMAL
endmodule
////////////////////////////////////////////////////////////////////////////////
//
// Filename:	qtrstage.v
//
// Project:	A General Purpose Pipelined FFT Implementation
//
// Purpose:	This file encapsulates the 4 point stage of a decimation in
//		frequency FFT.  This particular implementation is optimized
//	so that all of the multiplies are accomplished by additions and
//	multiplexers only.
//
// Operation:
// 	The operation of this stage is identical to the regular stages of
// 	the FFT (see them for details), with one additional and critical
// 	difference: this stage doesn't require any hardware multiplication.
// 	The multiplies within it may all be accomplished using additions and
// 	subtractions.
//
// 	Let's see how this is done.  Given x[n] and x[n+2], cause thats the
// 	stage we are working on, with i_sync true for x[0] being input,
// 	produce the output:
//
// 	y[n  ] = x[n] + x[n+2]
// 	y[n+2] = (x[n] - x[n+2]) * e^{-j2pi n/2}	(forward transform)
// 	       = (x[n] - x[n+2]) * -j^n
//
// 	y[n].r = x[n].r + x[n+2].r	(This is the easy part)
// 	y[n].i = x[n].i + x[n+2].i
//
// 	y[2].r = x[0].r - x[2].r
// 	y[2].i = x[0].i - x[2].i
//
// 	y[3].r =   (x[1].i - x[3].i)		(forward transform)
// 	y[3].i = - (x[1].r - x[3].r)
//
// 	y[3].r = - (x[1].i - x[3].i)		(inverse transform)
// 	y[3].i =   (x[1].r - x[3].r)		(INVERSE = 1)
//
// Creator:	Dan Gisselquist, Ph.D.
//		Gisselquist Technology, LLC
//
////////////////////////////////////////////////////////////////////////////////
//
// Copyright (C) 2015-2020, Gisselquist Technology, LLC
//
// This file is part of the general purpose pipelined FFT project.
//
// The pipelined FFT project is free software (firmware): you can redistribute
// it and/or modify it under the terms of the GNU Lesser General Public License
// as published by the Free Software Foundation, either version 3 of the
// License, or (at your option) any later version.
//
// The pipelined FFT project is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTIBILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU Lesser
// General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  (It's in the $(ROOT)/doc directory.  Run make
// with no target there if the PDF file isn't present.)  If not, see
// <http://www.gnu.org/licenses/> for a copy.
//
// License:	LGPL, v3, as defined and found on www.gnu.org,
//		http://www.gnu.org/licenses/lgpl.html
//
//
////////////////////////////////////////////////////////////////////////////////
//
//
`default_nettype	none
//
module	qtrstage(i_clk, i_reset, i_ce, i_sync, i_data, o_data, o_sync);
	parameter	IWIDTH=16, OWIDTH=IWIDTH+1;
	parameter	LGWIDTH=8, INVERSE=0,SHIFT=0;
	input	wire				i_clk, i_reset, i_ce, i_sync;
	input	wire	[(2*IWIDTH-1):0]	i_data;
	output	reg	[(2*OWIDTH-1):0]	o_data;
	output	reg				o_sync;
	
	reg		wait_for_sync;
	reg	[2:0]	pipeline;

	reg	signed [(IWIDTH):0]	sum_r, sum_i, diff_r, diff_i;

	reg	[(2*OWIDTH-1):0]	ob_a;
	wire	[(2*OWIDTH-1):0]	ob_b;
	reg	[(OWIDTH-1):0]		ob_b_r, ob_b_i;
	assign	ob_b = { ob_b_r, ob_b_i };

	reg	[(LGWIDTH-1):0]		iaddr;
	reg	[(2*IWIDTH-1):0]	imem	[0:1];

	wire	signed	[(IWIDTH-1):0]	imem_r, imem_i;
	assign	imem_r = imem[1][(2*IWIDTH-1):(IWIDTH)];
	assign	imem_i = imem[1][(IWIDTH-1):0];

	wire	signed	[(IWIDTH-1):0]	i_data_r, i_data_i;
	assign	i_data_r = i_data[(2*IWIDTH-1):(IWIDTH)];
	assign	i_data_i = i_data[(IWIDTH-1):0];

	reg	[(2*OWIDTH-1):0]	omem [0:1];

	//
	// Round our output values down to OWIDTH bits
	//
	wire	signed	[(OWIDTH-1):0]	rnd_sum_r, rnd_sum_i,
			rnd_diff_r, rnd_diff_i, n_rnd_diff_r, n_rnd_diff_i;
	convround #(IWIDTH+1,OWIDTH,SHIFT)	do_rnd_sum_r(i_clk, i_ce,
				sum_r, rnd_sum_r);

	convround #(IWIDTH+1,OWIDTH,SHIFT)	do_rnd_sum_i(i_clk, i_ce,
				sum_i, rnd_sum_i);

	convround #(IWIDTH+1,OWIDTH,SHIFT)	do_rnd_diff_r(i_clk, i_ce,
				diff_r, rnd_diff_r);

	convround #(IWIDTH+1,OWIDTH,SHIFT)	do_rnd_diff_i(i_clk, i_ce,
				diff_i, rnd_diff_i);

	assign n_rnd_diff_r = - rnd_diff_r;
	assign n_rnd_diff_i = - rnd_diff_i;
	initial wait_for_sync = 1'b1;
	initial iaddr = 0;
	always @(posedge i_clk)
	if (i_reset)
	begin
		wait_for_sync <= 1'b1;
		iaddr <= 0;
	end else if ((i_ce)&&((!wait_for_sync)||(i_sync)))
	begin
		iaddr <= iaddr + 1'b1;
		wait_for_sync <= 1'b0;
	end

	always @(posedge i_clk)
	if (i_ce)
	begin
		imem[0] <= i_data;
		imem[1] <= imem[0];
	end


	// Note that we don't check on wait_for_sync or i_sync here.
	// Why not?  Because iaddr will always be zero until after the
	// first i_ce, so we are safe.
	initial pipeline = 3'h0;
	always	@(posedge i_clk)
	if (i_reset)
		pipeline <= 3'h0;
	else if (i_ce) // is our pipeline process full?  Which stages?
		pipeline <= { pipeline[1:0], iaddr[1] };

	// This is the pipeline[-1] stage, pipeline[0] will be set next.
	always	@(posedge i_clk)
	if ((i_ce)&&(iaddr[1]))
	begin
		sum_r  <= imem_r + i_data_r;
		sum_i  <= imem_i + i_data_i;
		diff_r <= imem_r - i_data_r;
		diff_i <= imem_i - i_data_i;
	end

	// pipeline[1] takes sum_x and diff_x and produces rnd_x

	// Now for pipeline[2].  We can actually do this at all i_ce
	// clock times, since nothing will listen unless pipeline[3]
	// on the next clock.  Thus, we simplify this logic and do
	// it independent of pipeline[2].
	always	@(posedge i_clk)
	if (i_ce)
	begin
		ob_a <= { rnd_sum_r, rnd_sum_i };
		// on Even, W = e^{-j2pi 1/4 0} = 1
		if (!iaddr[0])
		begin
			ob_b_r <= rnd_diff_r;
			ob_b_i <= rnd_diff_i;
		end else if (INVERSE==0) begin
			// on Odd, W = e^{-j2pi 1/4} = -j
			ob_b_r <=   rnd_diff_i;
			ob_b_i <= n_rnd_diff_r;
		end else begin
			// on Odd, W = e^{j2pi 1/4} = j
			ob_b_r <= n_rnd_diff_i;
			ob_b_i <=   rnd_diff_r;
		end
	end

	always	@(posedge i_clk)
	if (i_ce)
	begin // In sequence, clock = 3
		omem[0] <= ob_b;
		omem[1] <= omem[0];
		if (pipeline[2])
			o_data <= ob_a;
		else
			o_data <= omem[1];
	end

	initial	o_sync = 1'b0;
	always	@(posedge i_clk)
	if (i_reset)
		o_sync <= 1'b0;
	else if (i_ce)
		o_sync <= (iaddr[2:0] == 3'b101);


























































































































































endmodule
////////////////////////////////////////////////////////////////////////////////
//
// Filename:	windowfn.v
//
// Project:	A General Purpose Pipelined FFT Implementation
//
// Purpose:	Apply a window function to incoming real data points, so as
//		to create an outgoing stream of data samples that can be used
//	in an FFT construct using 50% overlap.  The overlap, coupled with the
//	FFT's requirements, can make for somewhat of a problem.  Hence, there
//	are two 'ce' signals coming into the core.  A primary ce signal when
//	new data is ready, and an alternate that must take place between
//	primary signals.  This allows the second/alternate CE signal to be
//	appropriately spaced between the primary CE signals so that the
//	outgoing signals to the FFT will still meet separation
//	requirements--whatever they would be for the application.
//
//	For this module, the window size is the FFT length.
//
// Ports:
//	i_clk, i_reset	Should be self explanatory.  The reset is assumed to
//			be synchronous.
//
//	i_tap_wr, i_tap	For use when OPT_FIXED_TAPS is zero, i_tap_wr signals
//			that a "tap" or "coefficient" of the filter should be
//		written.  When i_tap_wr is high, i_tap is taken to be
//		a coefficient to the core.  There's an internal address
//		counter, so no address need be given.  However, the counter is
//		reset on an i_reset signal.
//
//	i_ce, i_alt_ce	As discussed above, these signals need to alternate
//		back and forth.  Following a reset, the first signal coming in
//		should be an i_ce signal.
//
//	i_sample	The incoming sample data, valid any time i_ce is true,
//			and accepted into the core on that clock tick.
//
//	o_ce		True when the core has a valid output
//	o_sample	The output calculated by the core, ready to pass to
//			the FFT
//	o_frame		True on the first sample of any frame.  Following a
//			reset, o_ce will remain false until o_frame is also
//		true with it.  From then on out, o_frame will be true once
//		every FFT length.
//
//	For a timing/signaling diagram, please feel free to run the formal
//	tools in cover mode for this module, 'sby -f windowfn.sby cover',
//	and then check out the generated trace.
//
//
// Creator:	Dan Gisselquist, Ph.D.
//		Gisselquist Technology, LLC
//
////////////////////////////////////////////////////////////////////////////////
//
// Copyright (C) 2018-2020, Gisselquist Technology, LLC
//
// This file is part of the general purpose pipelined FFT project.
//
// The pipelined FFT project is free software (firmware): you can redistribute
// it and/or modify it under the terms of the GNU Lesser General Public License
// as published by the Free Software Foundation, either version 3 of the
// License, or (at your option) any later version.
//
// The pipelined FFT project is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTIBILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU Lesser
// General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  (It's in the $(ROOT)/doc directory.  Run make
// with no target there if the PDF file isn't present.)  If not, see
// <http://www.gnu.org/licenses/> for a copy.
//
// License:	LGPL, v3, as defined and found on www.gnu.org,
//		http://www.gnu.org/licenses/lgpl.html
//
//
////////////////////////////////////////////////////////////////////////////////
//
//
`default_nettype	none
//
module	windowfn(i_clk, i_reset, i_tap_wr, i_tap,
		i_ce, i_sample, i_alt_ce,
		o_frame, o_ce, o_sample);
	parameter		IW=16, OW=16, TW=16, LGNFFT = 4;
	parameter	[0:0]	OPT_FIXED_TAPS = 1'b0;
	parameter		INITIAL_COEFFS = "";
	//
	// OPT_TLAST_FRAME
	//
	// This core can be a challenge to use with an AXI stream interface,
	// simply because the o_frame output normally indicates the *first*
	// value of any frame, not the last.  Set OPT_TLAST_FRAME high to adjust
	// this behavior and make o_frame a reference to the last data sample
	// in any FFT frame.
	parameter [0:0]	OPT_TLAST_FRAME = 1'b0;
	//
	localparam	AW=IW+TW;
	//
	input	wire			i_clk, i_reset;
	//
	input	wire			i_tap_wr;
	input	wire	[(TW-1):0]	i_tap;
	//
	input	wire			i_ce;
	input	wire	[(IW-1):0]	i_sample;
	input	wire			i_alt_ce;
	//
	output	reg			o_frame, o_ce;
	output	reg	[(OW-1):0]	o_sample;


	reg	[(TW-1):0]	cmem	[0:(1<<LGNFFT)-1];
	reg	[(TW-1):0]	dmem	[0:(1<<LGNFFT)-1];

	//
	// LOAD THE TAPS
	//
	wire	[LGNFFT-1:0]	tapwidx;
	generate if (OPT_FIXED_TAPS)
	begin : SET_FIXED_TAPS

		initial $readmemh(INITIAL_COEFFS, cmem);

		assign	tapwidx = 0;
		// Make Verilators -Wall happy
		// Verilator lint_off UNUSED
		wire	[TW:0]	ignored_inputs;
		assign	ignored_inputs = { i_tap_wr, i_tap };
		// Verilator lint_on  UNUSED

	end else begin : DYNAMICALLY_SET_TAPS

		// Coef memory write index
		reg	[(LGNFFT-1):0]	r_tapwidx;

		initial	r_tapwidx = 0;
		always @(posedge i_clk)
			if(i_reset)
				r_tapwidx <= 0;
			else if (i_tap_wr)
				r_tapwidx <= r_tapwidx + 1'b1;

		if (INITIAL_COEFFS != 0)
			initial $readmemh(INITIAL_COEFFS, cmem);
		always @(posedge i_clk)
			if (i_tap_wr)
				cmem[r_tapwidx] <= i_tap;

		assign	tapwidx = r_tapwidx;
	end endgenerate


	reg		[LGNFFT-1:0]	dwidx, didx; // Data indices: write&read
	reg		[LGNFFT-1:0]	tidx;	// Coefficient index
	reg				top_of_block, first_block;
	reg		[1:0]		frame_count;
	reg				p_ce, d_ce;
	reg	signed	[IW-1:0]	data;
	reg	signed	[TW-1:0]	tap;

	////////////////////////////////////////////////////////////////////////
	//
	// Clock #0: Incoming data
	//	Write data to memory, so we can come back to it later to get
	//	the other half of our 50% overlap.
	//

	//
	//
	// Record the incoming data into a local memory
	//
	//

	// Notice how this data writing section is *independent* of the reset,
	// depending only upon new sample data.

	initial	dwidx = 0;
	always @(posedge i_clk)
	if (i_reset)
		dwidx <= 0;
	else if (i_ce)
		dwidx <= dwidx + 1'b1;

	always @(posedge i_clk)
	if (i_ce)
		dmem[dwidx] <= i_sample;

	//
	// first_block is our attempt to make certain that the entire first
	// FFT's worth of data is repressed.  This is the data that wouldn't
	// be valid, due to the data in memory not being valid.  In the case
	// of a first_block, we do everything like we otherwise would--only
	// we don't output either o_ce or o_frame--hence none of the following
	// processing should operate on our outputs until we have a full and
	// valid frame of data.
	//
	initial	first_block = 1'b1;
	always @(posedge i_clk)
	if (i_reset)
		first_block <= 1'b1;
	else if ((i_alt_ce)&&(&tidx)&&(dwidx==0))
		first_block <= 1'b0;

	////////////////////////////////////////////////////////////////////////
	//
	// Clock #0: Sample arrives
	//
	//	Indicator signals:	i_ce, or i_alt_ce
	//
	//	Valid signals:
	//		This is the state machine stage.  Signals here are
	//		valid from clock to clock
	//
	//		first_block
	//


	//
	//
	// Keep track of the top of the block.  The top of the block is the
	// first incoming data sample on an FFT or half FFT boundary.  This
	// is where data processing starts from.
	//
	initial	top_of_block = 1'b0;
	always @(posedge i_clk)
	if (i_reset)
		top_of_block <= 1'b0;
	else if (i_alt_ce)
		top_of_block <= (&tidx)&&((!first_block)||(dwidx==0));
	else if (i_ce)
		top_of_block <= 1'b0;

	//
	// Data and coefficient memory indices.
	//
	// The data index goes from 0:(1<<LGNFFT)-1 for the first FFT, and then
	// (1<<(LGNFFT-1)):(1<<LGNFFT)-1,0:(1<<(LGNFFT-1))-1 for the overlapped
	// FFT.  During these same two runs, the tap (coefficient) index goes
	// from 0:(1<<LGNFFT)-1 each time, so the bottom (LGNFFT-1) bits can
	// be shared between the two pointers.
	//
	// Note that each of these pointers is set the clock before the data
	// actually arrives (as it should be)
	//
	initial	didx = 0;
	always @(posedge i_clk)
	if (i_reset)
		didx <= 0;
	else if ((i_alt_ce)&&(dwidx[LGNFFT-2:0]==0))
	begin
		// Restart on the first point of the next FFT
		didx[LGNFFT-2:0] <= 0;
		// Maintain the top bit, so as to keep
		// the overlap working properly
		didx[LGNFFT-1] <= dwidx[LGNFFT-1];
	end else if ((i_ce)||(i_alt_ce))
		// Process the next point in this FFT
		didx <= didx + 1'b1;

	always @(*)
		tidx[LGNFFT-2:0] = didx[LGNFFT-2:0];

	initial	tidx[LGNFFT-1] = 0;
	always @(posedge i_clk)
	if (i_reset)
		tidx[LGNFFT-1] <= 0;
	else if ((i_alt_ce)&&(dwidx[LGNFFT-2:0]==0))
	begin
		// Restart the counter for the first point
		// of the next FFT.
		tidx[LGNFFT-1] <= 0;
	end else if ((i_ce)||(i_alt_ce))
	begin
		// Process the next point in the window function
		//
		// Here we implement the carry only
		if (&tidx[LGNFFT-2:0])
			tidx[LGNFFT-1] <= !tidx[LGNFFT-1];
	end

	//
	// frame_count is based uff of the first index at the top of the
	// block.  It's used to make certain that the o_frame output is
	// properly aligned with the first valid clock of the output.
	//
	initial	frame_count = 0;
	always @(posedge i_clk)
	if (i_reset)
		frame_count <= 0;
	else if ((i_ce)&&(top_of_block)&&(!first_block))
		frame_count <= 3;
	else if (frame_count != 0)
		frame_count <= frame_count - 1'b1;

	//
	// Following any initial i_ce, ...
	// 	d_ce: The data (and coefficient), read from memory,will be valid
	// 	p_ce: The produc of data and coefficient is valid
	//
	initial	{ p_ce, d_ce } = 2'h0;
	always @(posedge i_clk)
	if (i_reset)
		{ p_ce, d_ce } <= 2'h0;
	else
		{ p_ce, d_ce } <= { d_ce, (!first_block)&&((i_ce)||(i_alt_ce))};

	////////////////////////////////////////////////////////////////////////
	//
	// Clock #1
	//
	//	Indicator signals: (i_ce || i_alt_ce)
	//	Valid signals:	didx, tidx, top_of_block
	//

	// Read the data sample point, and the filter coefficient, from block
	// RAM.  Because this is block RAM, we have to be careful not to
	// do anything else here.
	initial	data = 0;
	initial	tap = 0;
	always @(posedge i_clk)
	begin
		data <= dmem[didx];
		tap  <= cmem[tidx];
	end

	////////////////////////////////////////////////////////////////////////
	//
	// Clock #2
	//
	//	Indicator signals: d_ce
	//	Valid signals:	data, tap
	//

	//
	// Multiply the two values together
	 reg	signed	[IW+TW-1:0]	product;




	always @(posedge i_clk)
		product <= data * tap;


	////////////////////////////////////////////////////////////////////////
	//
	// Clock #3
	//
	//	Indicator signals:	p_ce
	//	Valid signals:		product, (frame_count)
	//

	//
	// Output CE.  This value will be true exactly two clocks after any
	//	(i_ce || i_alt_ce) input, indicating that we have a new valid
	//	output sample.
	//
	initial	o_ce = 0;
	always @(posedge i_clk)
	if (i_reset)
		o_ce <= 0;
	else
		o_ce <= p_ce;

	//
	// o_frame is the indicator that this is the first clock cycle of a
	// new FFT frame.  It's *like* TLAST in its function, but acts on the
	// first cycle, rather than the last.  If you want to use this with
	// an AXI stream, you can set OPT_TLAST_FRAME above, and then o_frame
	// will be set on the last o_ce clock cycle of any outgoing frame.
	//
	initial	o_frame = 1'b0;
	always @(posedge i_clk)
	if (i_reset)
		o_frame <= 1'b0;
	else if ((OPT_TLAST_FRAME && frame_count == 1)
		||(!OPT_TLAST_FRAME && frame_count == 2))
		o_frame <= 1'b1;
	else
		o_frame <= 1'b0;


	generate if (OW == AW)
	begin : BIT_ADJUSTMENT_NONE

		initial	o_sample = 0;
		always @(posedge i_clk)
		if (i_reset)
			o_sample <= 0;
		else if (p_ce)
			o_sample <= product;

	end else if (OW < AW)
	begin : BIT_ADJUSTMENT_ROUNDING
		wire	[AW-1:0]	rounded;

		assign	rounded = product + { {(OW){1'b0}}, product[AW-OW],
				{(AW-OW-1){!product[AW-OW]}} };

		initial	o_sample = 0;
		always @(posedge i_clk)
		if (i_reset)
			o_sample <= 0;
		else if (p_ce)
			o_sample <= rounded[(AW-1):(AW-OW)];

		// Make Verilator happy
		// verilator lint_off UNUSED
		wire	[AW-OW-1:0]	unused_rounding_bits;
		assign	unused_rounding_bits = rounded[AW-OW-1:0];
		// verilator lint_on  UNUSED

	end else // if (OW > AW)
	begin : BIT_ADJUSTMENT_EXTENDING

		always @(posedge i_clk)
		if (i_reset)
			o_sample <= 0;
		else if (p_ce)
			o_sample <= { product, {(OW-AW){1'b0}} };

	end endgenerate

	////////////////////////////////////////////////////////////////////////
	//
	// Clock #4: All done
	//
	//	Indicator signals:	o_ce
	//	Valid signals:		o_sample
	//


	// Make Verilator happy
	// verilator lint_off UNUSED
	wire	[LGNFFT-1:0]	unused;
	assign	unused = tapwidx;
	// verilator lint_on  UNUSED





































































































































































































































































































































































endmodule
