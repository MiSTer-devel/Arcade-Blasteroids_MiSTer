`timescale 1ns/1ps
//============================================================================
//  Atari LETA at 8A — the Whirly-Gig counters, FF9801 / FF9805.
//  SP-316 sheet 4; the operator manual's parts list calls the part
//  137434-1001.
//
//  Copyright (C) 2026 RetroShrimp.  GPL-3.0-or-later; see LICENSE and
//  NOTICE.md.  The behaviour is that of JROK's LETA_REP.vhd ("LETA on a
//  CPLD", jrok.com/project/leta), a functional replacement built by
//  bench-testing a real 137304-2002 and run on Rampart, Off the Wall and
//  Blasteroids boards; none of its code is copied.  The same model ships in
//  the Arcade-AtariSystem2 MiSTer core (rtl/sound/sys2_leta.sv, commit
//  f35c810b).
//
//  ---- wiring, off sheet 4 ------------------------------------------------
//    24 VCC   12 GND
//    13 CK    = 8H                    (the H counter's bit 3)
//    23 /CE   = /LETA                 (10B LS139 half 2, Y2 -> FF9800-FF9BFF)
//     9 TEST  = GND
//    14 RESOL = PR4 = 1               (R50 1 K to +5 V, sheet 1)
//    10 A1    = GND                   -> only channels 0 and 1 are reachable
//    11 A0    = BA2                   -> FF9801 = ch 0, FF9805 = ch 1
//    15-22 D0-D7 = BD0-BD7            -> the low byte lane, odd addresses
//     8 CLK0 / 7 DIR0 = P1-CLK / P1-DIR   (LET-5 / LET-6, via R6/R7 1 K)
//     6 CLK1 / 5 DIR1 = P2-CLK / P2-DIR   (LET-7 / LET-8, via R8/R9 1 K)
//     1,2,3,4 DIR3, CLK3, DIR2, CLK2 = all tied to GND — channels 2 and 3
//                                      are unused on this board.
//  Each input line has 0.01 uF to ground and a 3.3 K pull-down, so an
//  unconnected Whirly-Gig reads 0 0.
//
//  ---- what the chip does, per channel, on every CK rising edge -----------
//   * CLK and DIR are each shifted into a 3-sample history.  A line is
//     SETTLED when all three samples agree; a 1- or 2-sample glitch is
//     ignored, so every input state must last at least 3 CK periods.
//   * When both lines are settled, the settled (CLK, DIR) pair is compared
//     with the last settled pair:
//       DIR changed, CLK not   -> always counts: up if new DIR == CLK, else down
//       CLK changed, DIR not   -> counts only at RESOL = 1:
//                                 up if new CLK != DIR, else down
//       both changed           -> no count
//     So RESOL = 0 counts DIR edges only (2 counts per quadrature cycle) and
//     RESOL = 1 counts both lines (4 per cycle).  There is no 1x mode.  Each
//     line's "last settled" level follows it whenever it settles.
//   * Up, in (CLK, DIR), is therefore 00 -> 10 -> 11 -> 01 -> 00: CLK leads.
//   * TEST = 1 clears the counters and the filters, and the data pins show
//     the raw inputs, inverted: {CLK3,DIR3,CLK2,DIR2,CLK1,DIR1,CLK0,DIR0}
//     for any address.
//  This board straps RESOL = 1 and TEST = 0, so it runs at 4x.  Both are
//  parameters only so the other modes can be exercised.
//
//  ---- the contract the game ROM depends on -------------------------------
//  The VBLANK handler reads both ports once per frame and does
//      move.b (a1),d0 ; sub.b 2(a0),d0 ; ext.w d0
//  i.e. an 8-bit subtraction of the previous sample that relies on
//  wrap-around, then a quadratic response `delta * |delta|`.  So the counter
//  is a free-running 8-bit up/down counter: it wraps, a read does not clear
//  it, and it does not saturate.  The self test's encoder-rate page polls the
//  same port in a tight loop counting iterations between changes, so the
//  count also advances on its own between reads.
//
//  ---- sampling rate ------------------------------------------------------
//  CK is 8H, the H counter's bit 3: 7.15909 MHz / 16 = 447.4 kHz of rising
//  edges, one per 128 clk_sys.  With the 3-sample filter, each (CLK, DIR)
//  state must be held for at least 3 x 2.24 us = 6.7 us to be seen.  The
//  caller wires the 8H level and this module finds its rising edges.
//
//  ---- reset --------------------------------------------------------------
//  The part has no reset pin, so on a real machine the counters power up in
//  an unknown state and are never cleared; the game differences the value,
//  so the initial count is unobservable.  `init_reset` exists to make
//  simulation deterministic.  It primes the filters and the last settled
//  pair from the current inputs, so an idle channel never fabricates a count
//  when it is released.
//============================================================================

module blstroid_leta #(
	parameter bit RESOL = 1'b1,      // pin 14: PR4 on this board
	parameter bit TEST  = 1'b0       // pin 9:  GND on this board
)(
	input  logic       clk,          // clk_sys
	input  logic       init_reset,   // FPGA init only — the part has no reset pin

	input  logic       h8,           // 8H, the CK pin (level; edges found here)

	// Channel inputs as the LET connector delivers them, already synchronised.
	// [0] = player 1 (LET-5/6), [1] = player 2 (LET-7/8).
	input  logic [1:0] enc_a,        // CLK0 / CLK1
	input  logic [1:0] enc_b,        // DIR0 / DIR1

	input  logic       a2,           // BA2 -> A0: 0 = FF9801 (P1), 1 = FF9805 (P2)
	output logic [7:0] dout,         // BD7:0, combinational (the read strobe is /CE)

	// exported for observation only
	output logic [7:0] count0,
	output logic [7:0] count1
);

	logic [7:0] count [0:1];
	logic [2:0] ch    [0:1];         // CLK sample history, newest in bit 0
	logic [2:0] dh    [0:1];         // DIR sample history
	logic [1:0] cl, dl;              // last settled CLK / DIR
	logic       h8_d;

	// The settled levels and the count decision, from the registered
	// histories, exactly as the chip compares them on the next CK edge.
	logic [1:0] c_set, d_set;        // all three samples agree
	logic [1:0] c_new, d_new;        // the settled level
	logic [1:0] dir_edge, clk_edge;  // a counting transition
	logic [1:0] up;                  // its direction

	genvar i;
	generate
		for (i = 0; i < 2; i = i + 1) begin : g_ch
			assign c_set[i]    = (ch[i] == 3'b000) | (ch[i] == 3'b111);
			assign d_set[i]    = (dh[i] == 3'b000) | (dh[i] == 3'b111);
			assign c_new[i]    = ch[i][0];
			assign d_new[i]    = dh[i][0];
			assign dir_edge[i] = c_set[i] & d_set[i] & (c_new[i] == cl[i]) & (d_new[i] != dl[i]);
			assign clk_edge[i] = c_set[i] & d_set[i] & RESOL & (c_new[i] != cl[i]) & (d_new[i] == dl[i]);
			assign up[i]       = dir_edge[i] ? (d_new[i] == c_new[i])
			                                 : (c_new[i] != d_new[i]);
		end
	endgenerate

	always_ff @(posedge clk) begin
		if (init_reset) begin
			h8_d <= h8;
			for (int n = 0; n < 2; n++) begin
				count[n] <= 8'd0;
				ch[n]    <= {3{enc_a[n]}};
				dh[n]    <= {3{enc_b[n]}};
			end
			cl <= enc_a;
			dl <= enc_b;
		end else begin
			h8_d <= h8;
			if (h8 & ~h8_d) begin              // rising edge of 8H = the CK pin
				for (int n = 0; n < 2; n++) begin
					if (TEST) begin
						count[n] <= 8'd0;
						ch[n]    <= 3'b000;
						dh[n]    <= 3'b000;
						cl[n]    <= 1'b0;
						dl[n]    <= 1'b0;
					end else begin
						ch[n] <= {ch[n][1:0], enc_a[n]};
						dh[n] <= {dh[n][1:0], enc_b[n]};
						if (c_set[n]) cl[n] <= c_new[n];
						if (d_set[n]) dl[n] <= d_new[n];
						if (dir_edge[n] | clk_edge[n])
							count[n] <= up[n] ? count[n] + 8'd1 : count[n] - 8'd1;
					end
				end
			end
		end
	end

	assign dout   = TEST ? ~{4'b0000, enc_a[1], enc_b[1], enc_a[0], enc_b[0]}
	                     : count[a2];
	assign count0 = count[0];
	assign count1 = count[1];

endmodule
