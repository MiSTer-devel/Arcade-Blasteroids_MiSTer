`timescale 1ns/1ps
//============================================================================
//  Atari LETA at 8A — the Whirly-Gig counters, FF9801 / FF9805.
//  SP-316 sheet 4.
//
//  Copyright (C) 2026 RetroShrimp.  GPL-3.0-or-later; see LICENSE and
//  NOTICE.md.  Checked against the Arcade-Atari-System2 MiSTer core's
//  rtl/sound/sys2_leta.sv (GPL-2.0-or-later, RetroShrimp) — the same custom
//  part, wired differently on this board.
//
//  ---- wiring, off sheet 4 ------------------------------------------------
//    24 VCC   12 GND
//    13 CK    = 8H                    (the H counter's bit 3)
//    23 /CE   = /LETA                 (10B LS139 half 2, Y2 -> FF9800-FF9BFF)
//     9 TEST  = GND
//    14 RESOL = PR4 (= 1)
//    10 A1    = GND                   -> only channels 0 and 1 are reachable
//    11 A0    = BA2                   -> FF9801 = ch 0, FF9805 = ch 1
//    15-22 D0-D7 = BD0-BD7            -> the low byte lane, odd addresses
//     8 CLK0 / 7 DIR0 = P1-CLK / P1-DIR   (LET-5 / LET-6, via R6/R7 1 K)
//     6 CLK1 / 5 DIR1 = P2-CLK / P2-DIR   (LET-7 / LET-8, via R8/R9 1 K)
//     1,2,3,4 DIR3, CLK3, DIR2, CLK2 = all tied to GND — channels 2 and 3
//                                      are unused on this board.
//  Each input line has 0.01 uF to ground and a 3.3 K pull-down, so an
//  unconnected Whirly-Gig reads 0, not 1.
//
//  ---- the contract the game ROM depends on -------------------------------
//  The VBLANK handler reads both ports once per frame and does
//      move.b (a1),d0 ; sub.b 2(a0),d0 ; ext.w d0
//  i.e. an 8-bit subtraction of the previous sample that relies on
//  wrap-around, then a quadratic response `delta * |delta|`.  So the counter
//  is a free-running 8-bit up/down counter: it must wrap, it must not be
//  cleared by a read, and it must not saturate.  A counter that did any of
//  those breaks the steering.  The self test's encoder-rate page polls the
//  same port in a tight loop counting iterations between changes, so the count
//  must also advance on its own between reads.
//
//  ---- how the CLK/DIR pair is decoded ------------------------------------
//  No LETA data sheet is public and MAME has no LETA device at all (it reads
//  these addresses as plain IPT_DIAL ports, sensitivity 60).  The decode here
//  follows the Atari System 2 core's, which is calibrated against a game whose
//  service mode counts encoder pulses (720's 72-tooth disc = 144 counts/rev,
//  i.e. 2x decoding):
//   * the pin pair is treated as quadrature, CLK = phase A, DIR = phase B;
//   * with RESOL = 1, the way sheet 4 straps it, the chip counts on both
//     transitions of A — the 2x decode;
//   * direction is the usual quadrature test, "A and B agree at the sampling
//     edge" -> count up.
//  blstroid_whirly.sv drives the pins with a matching quadrature pattern that
//  a plain clock+direction reading would decode identically, so only the sign
//  could differ; the OSD "Whirly direction" switch covers that for the player.
//
//  ---- sampling rate ------------------------------------------------------
//  CK is 8H, i.e. the H counter's bit 3, which toggles every 8 H counts:
//  7.15909 MHz / 16 = 447.4 kHz of rising edges.  That is the chip's whole
//  input filter — an encoder spun faster than that loses counts on real
//  hardware rather than reading wrong.  The caller wires the 8H level and this
//  module finds its edges, so nothing has to synthesise a second clock enable.
//
//  ---- reset --------------------------------------------------------------
//  This board gives the part no reset input (pin 14 is RESOL here, not the
//  System 2 board's TBRES), so on a real machine the counters power up in an
//  unknown state and are never cleared.  `init_reset` exists only to make
//  simulation deterministic; the game differences the value, so the initial
//  count is unobservable.
//============================================================================

module blstroid_leta
(
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
	logic [1:0] a_d;
	logic       h8_d;

	assign dout   = count[a2];
	assign count0 = count[0];
	assign count1 = count[1];

	// 2x decode: any transition of A counts; up when A and B agree.
	always_ff @(posedge clk) begin
		if (init_reset) begin
			count[0] <= 8'd0;
			count[1] <= 8'd0;
			// Prime the A history from the current inputs, not from zero,
			// or an idle channel fabricates one count on the first sample.
			a_d      <= enc_a;
			h8_d     <= h8;
		end else begin
			h8_d <= h8;
			if (h8 & ~h8_d) begin              // rising edge of 8H = the CK pin
				a_d <= enc_a;
				for (int i = 0; i < 2; i++)
					if (enc_a[i] != a_d[i])
						count[i] <= (enc_a[i] == enc_b[i]) ? count[i] + 8'd1
						                                   : count[i] - 8'd1;
			end
		end
	end

endmodule
