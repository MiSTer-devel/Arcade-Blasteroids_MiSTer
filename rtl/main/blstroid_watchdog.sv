`timescale 1ns/1ps
//============================================================================
//  Blasteroids watchdog — SP-316 sheet 2 (14D LS00, 1B LS90, Q4 2N3904).
//
//  Copyright (C) 2026 RetroShrimp.  GPL-3.0-or-later; see LICENSE and
//  NOTICE.md.  Derived from the Xybots MiSTer core's
//  rtl/main/xybots_watchdog.sv (GPL-3.0, same author); the two boards use the
//  identical LS90 arrangement.
//
//    14D LS00  pin 1 = /WDOG, pin 2 = WDOGDIS -> pin 3.
//              WDOGDIS is pulled to +5 V by R58 1 K; a two-pad jumper (pads
//              "1" and "2") shorts it to ground, which nails 14D/3 HIGH and so
//              holds the counter cleared forever = the watchdog is disabled.
//    1B  LS90  CKA (14) = /VBLANK  — one count per frame, FALLING edge, i.e.
//                                    at the START of vertical blank
//              CKB (1)  = QA (12)  — the ordinary BCD cascade
//              R0(1),R0(2) (2,3) = 14D/3   — a /WDOG write clears to 0
//              R9(1),R9(2) (6,7) = POR     — power-on parks the count at 9
//    Q4        base <- R46 from QD (11), collector = /RESET (R47 pull-up)
//              -> /RESET = ~QD
//
//  QD is high for BCD 8 and 9, so /RESET asserts after 8 /VBLANK edges
//  (~133 ms at 59.9 Hz) without a /WDOG write and stays asserted for two
//  frames before the counter rolls 9 -> 0 and releases it.  At power-up the
//  R9 pair holds the count at 9, so /RESET is asserted out of the box; when
//  POR releases, the next /VBLANK edge rolls 9 -> 0 and the board starts.
//  On a real LS90 the R9 pair overrides the R0 pair and both are level master
//  resets that override the clock; that ordering is reproduced below.
//
//  This is load bearing.  Every unused vector in the ROM points at a single
//  `stop #$2700` stub at 0x000100 which does not feed the dog, so a stray
//  exception is meant to reboot the board a few frames later.  Get the
//  watchdog wrong and that turns into a reboot loop that looks like a
//  completely different bug.
//
//  MAME instantiates `WATCHDOG_TIMER(config, "watchdog")` with default
//  timing: neither the 8-frame period, nor the 2-frame pulse, nor the
//  park-at-9 power-up is modelled there.
//
//  What /RESET reaches is deliberately narrow: the 68000's RESET+HALT pins
//  (tied together), the 14/15B LS74 EEPROM unlock flip-flop's /CLR and the
//  14B LS08 that gates the EEPROM /OE.  It does not reach the SOS-2, the
//  LETA, the SCOM or the audio connector — the audio board is reset by a
//  write to FF8C00 instead.
//============================================================================

module blstroid_watchdog #(
	// The WDOGDIS jumper fitted (shorted to ground) -> the watchdog never
	// fires.  A shipped board has it OPEN, so the default is 0.
	parameter bit WDOGDIS = 1'b0
)(
	input  logic clk,
	input  logic por,          // power-on reset, active high (sheet 2, 8C LS14)
	input  logic vblank_start, // 1-clk pulse at the falling edge of /VBLANK
	input  logic wdog_clr,     // 1-clk pulse: any write to FF8000 (/WDOG)

	output logic reset_n,      // the /RESET net, active low
	output logic [3:0] count   // 1B QD..QA, exported for observation only
);

	// 14D LS00 (1 = /WDOG, 2 = WDOGDIS -> 3) drives the R0 pair.  With the
	// jumper fitted WDOGDIS = 0 so the output is stuck high and R0 is
	// permanently asserted.
	wire r0 = WDOGDIS ? 1'b1 : wdog_clr;

	always_ff @(posedge clk) begin
		if (por)               count <= 4'd9;   // R9 pair, highest priority
		else if (r0)           count <= 4'd0;   // R0 pair
		else if (vblank_start) count <= (count == 4'd9) ? 4'd0 : count + 4'd1;
	end

	// On the real LS90 both master-reset pairs are asynchronous LEVELS that
	// override the clock, so QD follows them immediately rather than at the
	// next edge.  The counter itself is kept synchronous (an FPGA should not
	// take an asynchronous reset from a decoded write strobe), and the level
	// behaviour is reproduced at the OUTPUT, which is the only place it is
	// observable: a /WDOG write releases /RESET in the same cycle, and with the
	// WDOGDIS jumper fitted (R0 permanently asserted) /RESET is never asserted
	// at all once POR has released — a one-clock glitch there would be a model
	// artefact, not board behaviour.
	// Only QD leaves the chip, so only QD is levelled.
	wire qd_eff = por ? 1'b1 : (r0 ? 1'b0 : count[3]);

	assign reset_n = ~qd_eff;                   // Q4, /RESET = ~QD

endmodule
