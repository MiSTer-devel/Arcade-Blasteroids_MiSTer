`timescale 1ns/1ps
//============================================================================
//  Blasteroids video DACs — SP-316 sheet 11: three identical packaged R/2R
//  ladders (RN3 red, RN2 green, RN1 blue) driven from the HC273 output
//  registers, plus the LS260 + 15A U06 code-0 pedestal sink.
//
//  Copyright (C) 2026 RetroShrimp.  GPL-3.0-or-later; see LICENSE and
//  NOTICE.md.  Module shape and the DAC_MAME_COMPAT idea come from the Xybots
//  MiSTer core's rtl/video/xybots_dac.sv (GPL-3.0, same author); the transfer
//  itself is Blasteroids' own, since Xybots' DAC is a binary-weighted ladder
//  multiplied by an intensity ladder and this one is a plain 5-bit R/2R.
//
//  ---- the circuit --------------------------------------------------------
//  Per channel: a packaged R/2R network (pin 2 = D0 tied to GND, pins 3..7 =
//  the five colour bits), a 1.5 K pull-up to +5 V, a 2.4 K load to ground, a
//  2N3904 emitter follower into 68 Ohm, and a 10 Ohm series output.  There are
//  no per-bit discrete resistors: the weighting is entirely inside the network,
//  so bit n contributes exactly 2^n / 32 of full scale and the transfer is
//  linear in the 5-bit code.  The network's R value is not printed anywhere in
//  the package, so the absolute volts are unknown; they only set the gain.
//
//  ---- the one non-linearity, and why it is not emulated -------------------
//  A 74LS260 5-input NOR per channel detects "all five bits zero" and drives a
//  15A U06 open-collector inverter that sinks the ladder node through 1 K,
//  cancelling the pull-up's pedestal so that code 0 is a true black.  Codes
//  1..31 are exactly linear; code 0 sits below the linear extrapolation.  Both
//  transfers below map code 0 to 0 and code 31 to full scale, so the pedestal
//  is invisible in any digital comparison and is left unmodelled.
//
//  ---- the two transfers ---------------------------------------------------
//    DAC_MAME_COMPAT = 1  (default)  MAME's `pal5bit(x) = (x << 3) | (x >> 2)`
//                                    — bit replication, what `xRGB_555` does.
//    DAC_MAME_COMPAT = 0             round(255 * x / 31), the exact linear
//                                    ladder value.
//  They agree on 28 of the 32 codes and differ by exactly 1/255 on codes
//  3, 7, 24 and 28, which no monitor will show.  The default is the MAME curve
//  so that a frame captured from this core can be compared bit for bit with one
//  captured from MAME; the ladder's own answer is 0.
//
//  `blank_n` is redundant with the HC273 /CLR that blstroid_cram already
//  models (blanking makes the code 0 and the pedestal makes 0 black), and is
//  kept only so a caller that drives this module directly cannot get it wrong.
//============================================================================

module blstroid_dac #(
	parameter bit DAC_MAME_COMPAT = 1'b1
)(
	input  logic [4:0] r5,           // colour word D14:10
	input  logic [4:0] g5,           // colour word D9:5
	input  logic [4:0] b5,           // colour word D4:0
	input  logic       blank_n,      // /BLANK: low forces the code to 0

	output logic [7:0] red,
	output logic [7:0] green,
	output logic [7:0] blue
);

	// The widest intermediate is 255 * 31 + 15 = 7920, so the division is exact
	// in 32-bit integer arithmetic and the 8-bit cast cannot truncate.
	function automatic logic [7:0] xfer(input logic [4:0] c);
		if (DAC_MAME_COMPAT) xfer = {c, c[4:2]};      // (c << 3) | (c >> 2)
		else                 xfer = 8'((255 * int'(c) + 15) / 31);
	endfunction

	// Continuous assignments rather than an always_comb: Icarus Verilog emits a
	// "sorry" diagnostic for a function's internal part-selects when the call
	// sits inside an always_* process.
	assign red   = blank_n ? xfer(r5) : 8'h00;
	assign green = blank_n ? xfer(g5) : 8'h00;
	assign blue  = blank_n ? xfer(b5) : 8'h00;

endmodule
