`timescale 1ns/1ps
//============================================================================
//  Blasteroids — SP-316 sheet 10: the priority RAM 17E (27LS00A, 256 x 1), its
//  two inverting 74F158 address multiplexers (15E, 16E) and the /CS3 gate that
//  makes a transparent motion-object pixel mean "playfield".
//
//  Copyright (C) 2026 RetroShrimp.  GPL-3.0-or-later; see LICENSE and
//  NOTICE.md.  Module shape from the Xybots MiSTer core's
//  rtl/video/xybots_prio.sv (GPL-3.0, same author); Blasteroids' network is a
//  one-bit lookup rather than Xybots' 74S85 comparator, so nothing else is
//  shared.
//
//  ---- the address -------------------------------------------------------
//  15E/16E F158 are inverting 2:1 muxes, but they invert the CPU write address
//  and the video lookup address identically, so the inversion cancels and
//  software sees sheet 12's
//
//      priority RAM address = H PPP MMMM
//                             |  |   +--- MO palette   = LBCOL3:0
//                             |  +------- PF palette   = PFPAL2:0
//                             +---------- PF pen bit 3 = PFPIX3
//
//  which is byte-for-byte MAME's
//  `priaddr = ((pf & 8) << 4) | (pf & 0x70) | ((mo & 0xf0) >> 4)`.
//
//  ---- the decision, and the polarities ----------------------------------
//  17E's D0 (pin 6) is an inverting open-collector output pulled up by R57, and
//  /CS3 (pin 5) = 14B LS08 (/PRIRAM, "MO pixel is transparent").  So:
//
//    LBPIX == 0  -> 17E deselected -> R57 pulls D0 to 1 -> playfield wins
//    LBPIX != 0  -> D0 = ~stored              -> stored = 1 -> MO wins
//
//  i.e. a stored 1 means "MO on top", which is what sheet 12 says and what
//  MAME's `if (mo[x] != 0xffff)` guard does.  The result is registered by
//  14C F74 on the /14M edge; that register lives in `blstroid_cram` with the
//  17D/16D F374 pair it steers, so this module is combinational plus the store.
//
//  ---- the CPU port ------------------------------------------------------
//  512 byte addresses FF8800-FF89FE, one bit each, write only (D1 pin 13 = BD0,
//  /WE = /PRIRAM | 7M).  `cpu_a` is BA8..BA1, i.e. (address - 0xFF8800) >> 1.
//  The sheet has no CPU read path to 17E: the 68000's `clr.w` over this region
//  is a read-modify-write whose read only has to terminate, and MAME maps the
//  region write-only as well.  Nothing is exported here for it.
//
//  ---- one copy of the store ---------------------------------------------
//  `blstroid_main.sv` also carries a 256 x 1 priority RAM, next to the 68000
//  write strobe, exported as `prio_vaddr` / `prio_vdata` (registered, one
//  clk_sys).  Set `INTERNAL_RAM = 0` to use that copy — `prio_addr` then drives
//  `prio_vaddr` and `prio_bit_in` takes `prio_vdata` — or leave it at 1 and let
//  this module own the store.  Never enable both: two copies of a RAM the
//  68000 writes will diverge.
//============================================================================

module blstroid_prio #(
	parameter bit INTERNAL_RAM = 1
)(
	input  logic       clk,

	// ---- 68000 write port (FF8800-FF89FE, D0); INTERNAL_RAM only ----
	input  logic       cpu_we,
	input  logic [7:0] cpu_a,        // BA8..BA1
	input  logic       cpu_d,        // BD0

	// ---- playfield layer (blstroid_pf) ----
	/* verilator lint_off UNUSEDSIGNAL */   // sheet 12's address uses PFPIX3
	input  logic [3:0] pf_pix,              // only; the whole pen is the port
	/* verilator lint_on UNUSEDSIGNAL */
	input  logic [2:0] pf_pal,

	// ---- motion objects (blstroid_mo, TRUE sense) ----
	input  logic [3:0] lb_pix,
	input  logic [3:0] lb_col,

	// ---- external store port; INTERNAL_RAM = 0 only ----
	output logic [7:0] prio_addr,    // -> blstroid_main prio_vaddr
	input  logic       prio_bit_in,  // <- blstroid_main prio_vdata

	// ---- to the colour-address multiplexer (blstroid_cram) ----
	output logic       pf_win,       // 14C F74 D: 1 = the playfield wins
	output logic       mo_opaque     // 16J F21 (second section), = /CS3 asserted
);

	logic pri [0:255];
	logic pri_int;

`ifndef ALTERA_RESERVED_QIS
	initial begin
		for (int i = 0; i < 256; i++) pri[i] = 1'b0;
	end
`endif

	always_ff @(posedge clk) begin
		if (cpu_we) pri[cpu_a] <= cpu_d;
	end

	assign prio_addr = {pf_pix[3], pf_pal, lb_col};

	// The internal array is read combinationally (17E is an asynchronous
	// 27LS00A); the external port is registered by blstroid_main, which costs
	// one clk_sys and none of the four this pixel lasts.
	always_comb pri_int = pri[prio_addr];

	wire pri_bit = INTERNAL_RAM ? pri_int : prio_bit_in;

	// /CS3 deselects 17E whenever the motion-object pixel is transparent, and
	// R57 then forces "playfield".
	assign mo_opaque = (lb_pix != 4'd0);
	assign pf_win    = mo_opaque ? ~pri_bit : 1'b1;

endmodule
