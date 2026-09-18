`timescale 1ns/1ps
//============================================================================
//  Blasteroids 68000 address decode — SP-316 sheet 3, gate for gate.
//
//  Copyright (C) 2026 RetroShrimp.  GPL-3.0-or-later; see LICENSE and
//  NOTICE.md.  Structure from the Xybots MiSTer core's
//  rtl/main/xybots_addr_decode.sv (GPL-3.0, same author); every equation below
//  is read off SP-316 sheet 3 rather than taken from MAME's main_map.  The two
//  descriptions agree over the whole 24-bit space: the mirrors MAME documents
//  are simply the address bits the drawn decoders never look at.
//
//  ---- the gates ----------------------------------------------------------
//    8C  LS14   AS  = ~/AS                        (active-high copy, pin 2)
//    8C  LS14   /R  = ~R//W                       (low during READS)
//    8C  LS14   /W  = ~/R = R//W                  (low during WRITES)
//    12D F32    /WH = /UDS | R//W ;  /WL = /LDS | R//W
//    12B LS32   /ROM      = A23 | /AS
//    13D LS10   /SLAPSTIK = ~( /A23 & BA16 & BA15 )   <- no /AS, no R//W
//    11J LS02 + 11B F00
//               /VIDRAM   = ~( A23 & AS & BA14 )
//    12B LS32 x2  10B LS139 half 1 G = BA14 | /A23 | /AS
//    10B LS139 half 1 : {B,A} = {BA13,BA12}
//              1Y0 -> 10C LS138 /G2A   1Y1 -> 10B LS139 half 2 G
//              1Y2 = /CRAM             1Y3 = /EEROM
//    10B LS139 half 2 : {B,A} = {BA11,BA10}, G = 1Y1
//              2Y0 unused (FF9000)  2Y1 /AUDRD  2Y2 /LETA  2Y3 /INPUTS
//              No R//W qualifier: these strobe on writes too; the parts simply
//              have nothing but read drivers.
//    10C LS138 : {C,B,A} = {BA11,BA10,BA9}, G1 = AS, /G2A = 1Y0, /G2B = /W
//              Y0 /WDOG    Y1 /IRQACK  Y2 /VBLACK  Y3 /UNLOCK
//              Y4 /PRIRAM  Y5 /AUDWR   Y6 /AUDRES  Y7 /WAITHBL
//              /G2B = /W and nothing else: no /UDS //LDS term, so a
//              high-byte-only write strobes them exactly like a word write.
//    1D  LS20   /VPA = ~( AS & FC0 & FC1 & FC2 )  -> every IRQ autovectored
//
//  ---- byte lanes ---------------------------------------------------------
//  The X2804A (/CE = /EEROM, /WE = /WL), the LETA (D0-D7 = BD0-BD7) and the
//  SCOM (D0-D7 = BD0-BD7) all sit on the low byte lane, i.e. at odd byte
//  addresses in the 68000's big-endian world, and the game reads them that
//  way.  The 12L F244 status buffer and the 10A LS257 button mux likewise
//  drive BD7:0 only, and nothing at all drives BD15:8 for any of them.
//
//  This module therefore also exports `lane_lo` / `lane_hi` (the /LDS and /UDS
//  copies) so the read mux can be honest about which lanes a device drives; it
//  does not gate the strobes with them, because the drawn gates do not.
//
//  ---- what the mirrors are ------------------------------------------------
//  A22-A15 reach no decoder at all, so the whole FF8000-FFFFFF register/RAM
//  map repeats every 32 KB over 800000-FFFFFF (MAME: `global_mask(0x83ffff)`
//  plus per-entry `.mirror(0x038000)`).  Inside FF8000-FF8FFF only A11-A9 are
//  decoded, so each write strobe answers over a whole 512-byte page (MAME:
//  `.mirror(0x0381fe)`) — except /PRIRAM, whose A8-A1 are the priority RAM's
//  own address.  Inside FF9000-FF9FFF only A11-A10 are decoded, so /AUDRD,
//  /LETA and /INPUTS each answer over a 1 KB page; A2 selects the LETA channel
//  and A1 selects the button player, which is why the ROM's
//  `movep.w $402(a1)` can read FF9C05 (A2 = 1, A1 = 1) and get the player-2
//  byte — that "mirror" is not a special case here, it is just what an
//  undecoded A2 does.  On the A23 = 0 half only A17 (the ROM pair /CE) is
//  decoded, so the 256 KB image repeats 32x over 000000-7FFFFF.
//
//  ---- the SLAPSTIC socket -------------------------------------------------
//  Decoded and exported because sheet 3 draws it; it reaches nothing.  The 2A
//  socket is empty ("The SLAPSTIC chip is not installed", sheet 3) and the 4A
//  LS157 that would substitute its bank bits is strapped to its B inputs by
//  R77, so 038000-03FFFF is plain program ROM.  Do not implement one.
//============================================================================

module blstroid_addr_decode
(
	// 68000 A23..A1.  There is no A0 anywhere on this board; the byte lane is
	// /UDS //LDS and nothing else.
	input  logic [23:1] a,

	input  logic        as,       // active-high copy of /AS (8C LS14 1->2)
	input  logic        uds_n,
	input  logic        lds_n,
	input  logic        rw,       // 1 = read, 0 = write (68000 R//W pin)
	input  logic  [2:0] fc,

	// ---- derived control (8C LS14 / 12D F32) ----
	output logic        r_n,      // /R  : low during reads
	output logic        w_n,      // /W  : low during writes
	output logic        wh_n,     // /WH : /UDS | R//W
	output logic        wl_n,     // /WL : /LDS | R//W
	output logic        lane_hi,  // ~/UDS, for the read mux
	output logic        lane_lo,  // ~/LDS, for the read mux

	// ---- top-level chip selects ----
	output logic        rom_n,
	output logic        slapstic_n,  // 2A socket is empty; reaches nothing
	output logic        vidram_n,
	output logic        cram_n,
	output logic        eerom_n,

	// ---- 10B LS139 half 2, the read decodes ----
	output logic        io9000_n,    // 2Y0, FF9000-FF93FF: no net on the sheet
	output logic        audrd_n,
	output logic        leta_n,
	output logic        inputs_n,

	// ---- 10C LS138, the eight write strobes ----
	output logic        wdog_n,
	output logic        irqack_n,
	output logic        vblack_n,
	output logic        unlock_n,
	output logic        priram_n,
	output logic        audwr_n,
	output logic        audres_n,
	output logic        waithbl_n,

	// ---- autovector ----
	output logic        vpa_n
);

	// ---- 8C LS14 / 12D F32 ------------------------------------------------
	assign r_n  = ~rw;            // 8C LS14 3->4
	assign w_n  = rw;             // 8C LS14 11->10  (= ~r_n)
	assign wh_n = uds_n | rw;     // 12D F32 1,2->3
	assign wl_n = lds_n | rw;     // 12D F32 12,13->11
	assign lane_hi = ~uds_n;
	assign lane_lo = ~lds_n;

	// ---- 12B LS32 / 13D LS10 / 11J LS02 + 11B F00 -------------------------
	assign rom_n      = a[23] | ~as;
	assign slapstic_n = ~(~a[23] & a[16] & a[15]);
	assign vidram_n   = ~(a[23] & as & a[14]);

	// ---- 10B LS139 half 1 : 1A = BA12, 1B = BA13 --------------------------
	// G = BA14 | /A23 | /AS  ->  enabled for A23 = 1, A14 = 0, AS.
	wire       io_en = a[23] & as & ~a[14];
	wire [1:0] sel1  = {a[13], a[12]};
	wire       ff8_n = ~(io_en & (sel1 == 2'd0));   // 1Y0 -> LS138 /G2A
	wire       ff9_n = ~(io_en & (sel1 == 2'd1));   // 1Y1 -> half 2 G
	assign     cram_n  = ~(io_en & (sel1 == 2'd2)); // 1Y2
	assign     eerom_n = ~(io_en & (sel1 == 2'd3)); // 1Y3

	// ---- 10B LS139 half 2 : 2A = BA10, 2B = BA11, G = 1Y1 -----------------
	// No R//W term anywhere in this half — see the header.
	wire [1:0] sel2 = {a[11], a[10]};
	assign io9000_n = ~(~ff9_n & (sel2 == 2'd0));
	assign audrd_n  = ~(~ff9_n & (sel2 == 2'd1));
	assign leta_n   = ~(~ff9_n & (sel2 == 2'd2));
	assign inputs_n = ~(~ff9_n & (sel2 == 2'd3));

	// ---- 10C LS138 : G1 = AS, /G2A = 1Y0, /G2B = /W -----------------------
	wire       wr_en = ~ff8_n & ~w_n;               // AS is already in ff8_n
	wire [2:0] sel3  = {a[11], a[10], a[9]};
	assign wdog_n    = ~(wr_en & (sel3 == 3'd0));
	assign irqack_n  = ~(wr_en & (sel3 == 3'd1));
	assign vblack_n  = ~(wr_en & (sel3 == 3'd2));
	assign unlock_n  = ~(wr_en & (sel3 == 3'd3));
	assign priram_n  = ~(wr_en & (sel3 == 3'd4));
	assign audwr_n   = ~(wr_en & (sel3 == 3'd5));
	assign audres_n  = ~(wr_en & (sel3 == 3'd6));
	assign waithbl_n = ~(wr_en & (sel3 == 3'd7));

	// ---- 1D LS20, /VPA ----------------------------------------------------
	assign vpa_n = ~(fc[0] & fc[1] & fc[2] & as);

	// A22..A18, A17 (the ROM-pair /CE, consumed inside blstroid_prog_rom) and
	// A8..A1 (consumed by the RAMs, the priority RAM, the LETA's A2 and the
	// button mux's A1) reach no decoder on sheet 3.  That omission is the whole
	// source of the mirrors described in the header, so it is deliberate; sink
	// the bits so the tools do not warn.
	/* verilator lint_off UNUSEDSIGNAL */
	wire unused_a = &{1'b0, a[22:17], a[8:1]};
	/* verilator lint_on UNUSEDSIGNAL */

endmodule
