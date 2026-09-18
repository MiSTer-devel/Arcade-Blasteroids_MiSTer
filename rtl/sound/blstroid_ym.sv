`timescale 1ns/1ps
//============================================================================
//  Blasteroids (Atari Games, 1987) - YM2151 wrapper: the jt51 core plus the
//  fast-6502 to slow-YM write bridge.
//
//  Copyright (C) 2026 RetroShrimp.  GPL-3.0-or-later; see LICENSE and NOTICE.md.
//  Derived from the Xybots MiSTer core's xybots_ym.sv (GPL-3.0, same author,
//  itself from Toobin'); SP-316 sheet 16 wires 2F-YAM the same way.
//
//  SP-316 sheet 16, 2F-YAM: D7..D0 = SD7..SD0, PhiM(24) = 3579K, A0(4) = SA0,
//  /IC(3) = /YAMRES (the /WRIO latch bit 0), /WR(5) = /SWR, /RD(6) = /SRD,
//  /CS(7) = /YAM, /IRQ(2) onto the shared 6502 /IRQ pull-up R52.  SO/SH1/SH2
//  and the clock output drive the 2D YM3012 DAC, whose CH1/CH2 feed the
//  sheet-16 mixer.  CT1/CT2 gate the POKEY/speech sum into the left/right
//  analogue paths; on Blasteroids that sum is silent (both sockets empty), so
//  CT1/CT2 have no audible effect and are left unconnected at the top level.
//
//  The 6502 writes the YM at bus speed, but jt51 captures
//  `write = !cs_n & !wr_n` on cen_p1 (its register clock).  Each 6502 YM write
//  (a0 + data) is therefore latched into a pending register, with cs_n/wr_n
//  held asserted until a cen_p1 pulse consumes it.  Reads return jt51's status
//  (dout) directly.  Audio is jt51's full-resolution xleft/xright output.
//
//  Why zero latency matters: the YM2151 sets BUSY when it accepts the write and
//  the sound ROM spins on it (BIT $2001 / BPL).  A one-cen_p1 (559 ns) delay
//  shifts that window by one 6502 cycle and occasionally costs the poll loop an
//  extra pass.  This JSA driver family seeds its random effect pitch from an
//  LFSR that its idle loop steps at about 28 kHz, so accumulated microseconds
//  turn into audible semitone shifts there.  The real 2F latches on /WR + /CS
//  during the bus cycle, so zero latency is what the part does.
//
//  ---- clock enables --------------------------------------------------------
//  clk_sys is 57.272724 MHz = 4 x the 14.318181 MHz video crystal, and the JSA
//  crystal is exactly that crystal / 4, so blstroid_core derives
//      ce_ym    = clk_sys/16 = 3.579545  MHz   -> cen
//      ce_6502  = clk_sys/32 = 1.7897725 MHz   -> cen_p1
//  from one free-running counter.  ce_6502 is a strict subset of ce_ym (the
//  even ce_ym pulses), which is precisely the "cen_p1 is every other cen,
//  aligned with it" relationship jt51 documents.  No fractional enable
//  generator is needed anywhere on this board.
//============================================================================

module blstroid_ym
(
	input  logic        clk,
	input  logic        reset,
	input  logic        cen,        // YM2151 clock enable (3.579545 MHz)
	input  logic        cen_p1,     // half-rate enable, phase-aligned with cen
	input  logic        ym_reset_n, // /YAMRES from the /WRIO latch (active low)

	// bus side (from blstroid_jsa_bus)
	input  logic        ym_cs,      // access this cycle
	input  logic        ym_a0,      // register select
	input  logic        ym_we,      // write strobe (one bus cycle)
	input  logic  [7:0] ym_dout,    // 6502 -> YM data
	output logic  [7:0] ym_din,     // YM status -> 6502

	// audio + peripheral
	output logic        sample,
	output logic signed [15:0] aud_left,
	output logic signed [15:0] aud_right,
	output logic signed [15:0] aud_left_lo,
	output logic signed [15:0] aud_right_lo,
	output logic        ct1,
	output logic        ct2,
	output logic        ym_irq      // active high; wired-OR onto the shared
	                                // 6502 /IRQ net (SP-316 sheets 16/17)
);

	// ---- write bridge ----
	logic       wr_pend, wr_a0;
	logic [7:0] wr_d;
	wire        wr_cap = ym_cs & ym_we;         // one bus-cycle write strobe
	always_ff @(posedge clk) begin
		if (reset) wr_pend <= 1'b0;
		else if (wr_cap & ~cen_p1) begin        // strobe missed cen_p1: hold it
			wr_pend <= 1'b1; wr_a0 <= ym_a0; wr_d <= ym_dout;
		end else if (cen_p1) begin
			wr_pend <= 1'b0;                    // jt51 consumed it on cen_p1
		end
	end

	wire        wr_go   = wr_cap | wr_pend;
	wire        jt51_a0 = wr_cap ? ym_a0   : wr_a0;
	wire  [7:0] jt51_d  = wr_cap ? ym_dout : wr_d;

	wire irq_n;
	assign ym_irq = ~irq_n;

	jt51 u_jt51 (
		.rst    (reset | ~ym_reset_n),
		.clk    (clk),
		.cen    (cen),
		.cen_p1 (cen_p1),
		.cs_n   (~wr_go),
		.wr_n   (~wr_go),
		.a0     (jt51_a0),
		.din    (jt51_d),
		.dout   (ym_din),
		.ct1    (ct1),
		.ct2    (ct2),
		.irq_n  (irq_n),
		.sample (sample),
		.left   (aud_left_lo),
		.right  (aud_right_lo),
		.xleft  (aud_left),
		.xright (aud_right)
	);

endmodule
