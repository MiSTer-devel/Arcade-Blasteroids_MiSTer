`timescale 1ns/1ps
//============================================================================
//  Blasteroids (Atari Games, 1987) - JSA I sound-CPU (6502) address decoder.
//
//  Copyright (C) 2026 RetroShrimp.  GPL-3.0-or-later; see LICENSE and NOTICE.md.
//  Derived from the Xybots MiSTer core's xybots_jsa_decode.sv (GPL-3.0, same
//  author, itself from Toobin').  Both games carry the same Atari "Stand-Alone
//  Audio PCB" drawing 043713-xx - Xybots rev A (SP-313 sheets 11-14),
//  Blasteroids rev B (SP-316 sheets 14-17) - and the drawn decode nets are
//  identical.
//
//  This module is the 2L PAL plus the two chips it enables: the 4M LS138 and
//  the POKEY socket's chip select, wired as SP-316 sheet 15 draws them.
//  blstroid_jsa_pal2l.sv is the literal fuse map of Atari 136056-2101, so the
//  decode is taken from the part rather than inferred.  Three consequences
//  worth stating, because they differ from the obvious guess:
//
//   1. /YAM is $2000-$27FF, a 2 KB window with SA10:SA0 don't-care, not just
//      the two register addresses $2000/$2001.
//   2. The decoder is NOT direction-agnostic.  The 4M LS138's active-high G1
//      is wired to /SRD, and the PAL deliberately inverts /SRD inside the
//      $2800-$2FFF block so the enable is true for (READ & SA9=0) or
//      (WRITE & SA9=1).  Y1..Y3 (/RDP, /RDIO, /IRQACK) therefore fire only on
//      reads and Y4..Y7 (/VOICE, /WRP, /WRIO, /MIX) only on writes.  /IRQACK
//      is read-only on hardware; MAME acknowledges on both, and the shipping
//      ROM only ever reads it (LDA $280E at $41BB and $41C9).
//   3. /ROM is read-only: the 2K's /CE carries SR//W.
//
//  ---- what the board decodes (SP-316 sheet 15) ----------------------------
//
//    * 2L PAL 136056-2101 -> /ROM, /YAM, /REST, /RAM, A13B, A12B, /SWR, /SRD.
//    * 4M 74LS138, G1(6)=/SRD, /G2A(4)=/REST, /G2B(5)=SA10,
//      C(3)=SA9, B(2)=SA2, A(1)=SA1:
//         Y0 0x2800 n/c    Y1 /RDP     Y2 /RDIO    Y3 /IRQACK   (reads)
//         Y4 /VOICE        Y5 /WRP     Y6 /WRIO    Y7 /MIX      (writes)
//      Only SA9, SA2 and SA1 are decoded, so SA8..SA3 and SA0 are don't-cares
//      (MAME's .mirror(0x01f9)).  The Blasteroids sound ROM really does use the
//      mirror: /RDP as $280A, /RDIO as $280C and /IRQACK as $280E.
//    * POKEY 3K: CS0(30)=/REST, CS1(31)=SA10 - the socket answers to the whole
//      of 0x2C00-0x2FFF with SA3:SA0 as the register.  3K is not populated on
//      Blasteroids, but the select is a property of the PAL and of SA10, not of
//      the chip, so it is still produced here and blstroid_jsa_bus answers 0xFF
//      where the empty socket floats.
//    * YM2151 2F: /CS(7)=/YAM from 2L, A0(4)=SA0.
//
//    0x0000-0x1FFF  RAM (8 KB, 2H)            /CS1=/RAM, /WE=/SWR, /OE=/SRD
//    0x2000-0x27FF  YM2151 (2F, reg = SA0)    /CS=/YAM        (2 KB mirror)
//    0x2800         n/c                       (mirror 0x01F9) read
//    0x2802         /RDP   sound command      (mirror 0x01F9) READ only
//    0x2804         /RDIO  input port         (mirror 0x01F9) READ only
//    0x2806         /IRQACK                   (mirror 0x01F9) READ only
//    0x2A00         /VOICE TMS5220 latch (3F) (mirror 0x01F9) WRITE only
//                          socket empty, but the LS374 is stuffed and feeds
//                          the /MIX latch
//    0x2A02         /WRP   sound response     (mirror 0x01F9) WRITE only
//    0x2A04         /WRIO  output latch (5H)  (mirror 0x01F9) WRITE only
//    0x2A06         /MIX   mixer latch  (5F)  (mirror 0x01F9) WRITE only
//    0x2C00-0x2FFF  POKEY (3K)                (mirror 0x03F0) socket empty
//    0x3000-0x3FFF  banked ROM (A13B/A12B carry BA13/BA12; all four banks are
//                   used by this ROM)
//    0x4000-0xFFFF  fixed ROM
//
//  MAME mirror semantics: a register at base B with mirror M matches when
//  (addr & ~M) == B, so the "care" mask is ~M.
//
//  ---- O2 ------------------------------------------------------------------
//  `phi2` is the PAL's O2 pin.  A 6502 bus cycle here is one ce_6502 clock
//  enable, so that enable IS O2: high for exactly the clock on which the
//  transfer happens and low otherwise, which is what every O2-qualified strobe
//  on the board needs.  Note which outputs do NOT carry it - /RAM, /YAM, /ROM,
//  A13B and A12B are pure address decodes in the PAL, so sel_ram / sel_ym /
//  sel_bank / sel_rom stay valid for the whole cycle and can drive the
//  synchronous memories, whose reads are registered a clock ahead of `ce`.
//============================================================================

module blstroid_jsa_decode
(
	input  logic [15:0] addr,        // SA15..SA0
	input  logic        rnw,         // SR//W  (1 = read)   — 2L pin 1
	input  logic        phi2,        // O2                  — 2L pin 2
	input  logic  [1:0] bank,        // BA13:BA12 from the 5H /WRIO latch

	// ---- address-only device selects (no Φ2, no direction) ----
	output logic        sel_ram,     // /RAM  asserted: 0x0000-0x1FFF
	output logic        sel_ym,      // /YAM  asserted: 0x2000-0x27FF
	output logic        sel_bank,    // 0x3000-0x3FFF (the A13B/A12B window)
	output logic        sel_rom,     // 0x4000-0xFFFF
	output logic        sel_pokey,   // /REST & SA10 — POKEY socket, empty

	// ---- 4M LS138 strobes (Φ2- AND direction-qualified by the PAL) ----
	output logic        sel_rdp,     // Y1, read
	output logic        sel_rdio,    // Y2, read
	output logic        sel_irqack,  // Y3, read
	output logic        sel_voice,   // Y4, write — 3F LS374, feeds /MIX
	output logic        sel_wrp,     // Y5, write
	output logic        sel_wrio,    // Y6, write
	output logic        sel_mix,     // Y7, write

	// ---- raw 2L outputs, active-high "asserted" form ----
	output logic        rom_ce,      // /ROM: READ & 0x3000-0xFFFF
	output logic        srd,         // /SRD asserted
	output logic        swr,         // /SWR asserted
	output logic        a13b,        // 2K pin 26
	output logic        a12b,        // 2K pin 2

	output logic        ym_reg,      // YM2151 register select (SA0)
	output logic  [3:0] pokey_reg    // POKEY register (SA3..SA0)
);

	// ---- 2L-PAL 136056-2101 -------------------------------------------------
	logic rom_n, yam_n, rest_n, ram_n, swr_n, srd_n;
	blstroid_jsa_pal2l u_pal2l (
		.i1 (rnw),        .i2 (phi2),      .i3 (bank[0]),   .i4 (bank[1]),
		.i5 (addr[9]),    .i6 (addr[11]),  .i7 (addr[12]),  .i8 (addr[13]),
		.i9 (addr[14]),   .i11(addr[15]),
		.o12(rom_n),      .o13(yam_n),     .o14(rest_n),    .o15(ram_n),
		.o16(a13b),       .o17(a12b),      .o18(swr_n),     .o19(srd_n) );

	assign rom_ce  = ~rom_n;
	assign sel_ram = ~ram_n;                                  // 0x0000-0x1FFF
	assign sel_ym  = ~yam_n;                                  // 0x2000-0x27FF
	assign srd     = ~srd_n;
	assign swr     = ~swr_n;
	wire   rest    = ~rest_n;                                 // O2 & 0x2800-0x2FFF

	// The 2K's own address bus splits the read window the same way the PAL's
	// A13B/A12B do; these two are the plain address decodes the synchronous ROM
	// path needs (/ROM itself is read-only and is exported separately).
	assign sel_bank = (addr[15:12] == 4'h3);                  // 0x3000-0x3FFF
	assign sel_rom  = (addr[15:14] != 2'b00);                 // 0x4000-0xFFFF

	// ---- 4M 74LS138 ---------------------------------------------------------
	// G1(6) = /SRD -> the enable is /SRD NOT asserted; /G2A(4) = /REST and
	// /G2B(5) = SA10 -> both must be low, i.e. /REST asserted and SA10 = 0.
	// C(3)/B(2)/A(1) = SA9/SA2/SA1.
	wire       en138 = rest & ~srd & ~addr[10];
	wire [2:0] s138  = {addr[9], addr[2], addr[1]};
	wire [7:0] y138  = en138 ? (8'h01 << s138) : 8'h00;

	// y138[0] is 0x2800 / Y0 — unlabelled on sheet 15 and connected to nothing.
	assign sel_rdp    = y138[1];
	assign sel_rdio   = y138[2];
	assign sel_irqack = y138[3];
	assign sel_voice  = y138[4];
	assign sel_wrp    = y138[5];
	assign sel_wrio   = y138[6];
	assign sel_mix    = y138[7];

	// ---- POKEY socket 3K: CS0 = /REST, CS1 = SA10 ---------------------------
	assign sel_pokey  = rest & addr[10];                      // 0x2C00-0x2FFF

	assign ym_reg    = addr[0];
	assign pokey_reg = addr[3:0];

	// SA8..SA4 are decoded by nothing on this board - they are the middle of the
	// LS138's 0x01F9 mirror, and the 2K/2H address bus takes them straight to
	// the memories, which is why $280A/$280C/$280E alias $2802/$2804/$2806.  Y0
	// (0x2800) is likewise unlabelled on sheet 15 and connected to nothing.
	wire _unused_dec = &{1'b0, y138[0], addr[8:4]};

endmodule
