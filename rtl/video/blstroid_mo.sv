`timescale 1ns/1ps
//============================================================================
//  Blasteroids — the motion-object engine: SP-316 sheets 7, 8 and 9 tied
//  together.
//
//  Copyright (C) 2026 RetroShrimp.  GPL-3.0-or-later; see LICENSE and
//  NOTICE.md.  Structurally derived from the Xybots MiSTer core's
//  rtl/video/xybots_mo*.sv (GPL-3.0, same author) and from the Toobin' core's
//  linked-list walker; the arithmetic and the timing are Blasteroids'.
//
//  This module carries three of the four blocks that are one pipeline on the
//  board — the fourth, the LINK WALK (13E LS74 + 7D LS273A + 7E LS374, sheet
//  5), lives in `blstroid_vram.sv` with the address multiplexers it drives:
//
//      VMATCH 8K            the vertical match / size compare and the stamp
//                           ROM address (sheet 7, Atari 137552-001)
//      the ROM read         sixteen 27512s (sheets 7 + 8) as ONE burst-8-byte
//                           read through the blstroid_gfx_mem MO client
//      SOS-1 10K            the 16-pixel shifter (sheet 8, Atari 137550-001)
//
//  `blstroid_mo_linebuf` (sheet 9) is instantiated at the bottom.
//
//  ---------------------------------------------------------------------------
//  The 8-count group — the whole engine in one table
//  ---------------------------------------------------------------------------
//  One `1H` count = TWO 14.318181 MHz pixel clocks, so `ce_7m` is the second
//  `ce_14m` of each count and h[8:0] advances on it.
//  456 counts per line / 8 counts per link = 57 links per scanline, and
//  /LINKRES restarts the walk at link 0 on every line, so the engine follows
//  links 0..56 of the chain and no more.
//
//   h%8 | VRAM slot     | what this module does
//   ----+---------------+-------------------------------------------------
//    0  | MOVPOS  (w0)  | (ce_7m: VMATCH INCLK falls) latch Y[8:0], vsize[3:0]
//    1  | MOPIC   (w1)  | the vertical compare resolves; ISSUE the ROM burst
//       |               | (first ce_14m of the count, as soon as `vd` is
//       |               | valid); (ce_7m: INCLK rises, /4HD2H clocks 7F)
//       |               | latch hflip/vflip/code
//    2  | CPU           |
//    3  | playfield     | (ce_7m: 4H rises = VMATCH OUTCLK on the board)
//    4  | MOLINK  (w2)  | blstroid_vram latches VD11:3 into 13E + 7D
//    5  | MOHPOS  (w3)  | /MOL loads the fill buffer's write pointer with
//       |               | VD15:6; (ce_7m: 4HD2H) SOS-1 LDCLK takes
//       |               | MOPAL3:0 = VD3:0 and the draw registers take the row
//    6  |               | the 16 pixels start here
//    7  |               |
//   ----+---------------+-------------------------------------------------
//   and pixels 4..15 continue over h%8 = 0..5 of the NEXT group.
//
//  So link n is FETCHED over counts 8n..8n+7 and DRAWN over counts 8n+6..8n+13.
//  Link 56 (the 57th) is fetched over 448..455 and drawn over 454..461, i.e.
//  counts 454, 455 of this line and 0..5 of the next — finishing EXACTLY at the
//  5->6 boundary where `1VD6H` (4E LS174C, clocked by 4HD2H) flips the buffers
//  over.  57 links fit the line to the count.
//
//  ---------------------------------------------------------------------------
//  The ROM row, and the one place this is not gate-for-gate
//  ---------------------------------------------------------------------------
//  The board reads the sprite ROMs CONTINUOUSLY: VMATCH holds RA14:2 for eight
//  counts while 4D LS86 walks {MOROM1, MOROM0} = {/4H, 2H} XOR HFLD through the
//  four words of the stamp row, four pixels per word.  Here the whole row is
//  ONE blstroid_gfx_mem burst (8 bytes: 4 from the M half and 4 from the N
//  half), so:
//
//    * the request is issued on the FIRST ce_14m of h%8 == 1, the moment
//      `vd` carries MOPIC, and only when the vertical match succeeded;
//    * it must be answered by the ce_7m of h%8 == 5, which loads the draw
//      registers — four counts and four clk_sys = 36 clk_sys.  `fetch_late` is
//      sticky and reports a miss;
//    * the four address XORs collapse into a plain 16-pixel reversal, because
//      HFLD reverses the word order AND (inside SOS-1) the pixel order inside
//      each word.
//
//  PIXEL ORDER, recorded here because it is the one thing the sheets cannot
//  show — it is inside SOS-1.  `mo_data` is {N3,N2,N1,N0, M3,M2,M1,M0}; the
//  word the shifter sees is MP15:0 = {N_k, M_k} and it unpacks MP15:12, MP11:8,
//  MP7:4, MP3:0 left to right, i.e.
//
//      pixel 4k+0 = N_k[7:4]   pixel 4k+1 = N_k[3:0]
//      pixel 4k+2 = M_k[7:4]   pixel 4k+3 = M_k[3:0]
//
//  so the N (high, 0x80000) half supplies the LEFT pair of every four.  That is
//  MAME's `molayout` x-offset list {RGN_FRAC(1,2)+0, RGN_FRAC(1,2)+4, 0, 4,
//  ...} read back out.
//
//  ---- what comes out, and what comes in ------------------------------------
//  `lb_pix` is the TRUE pen (17J F398's /Q pins) and `lb_col` the true 4-bit
//  motion-object palette (14E F174).  Pen 0 is transparent and a location no
//  object touched reads pen 0 — the pull-ups R63-R70 put 0xF on the pixel bus,
//  the F398 inverts it, and the read-and-clear cycle writes that same 0xF back.
//  `mo_hpos` is the line-buffer address those two belong to, so the compositor
//  can check the phase instead of assuming it.
//
//  The only input from the video RAM is `vd`: `blstroid_vram` owns the sheet-5
//  address bank and the link registers, and during count h `vd` carries the
//  word addressed in count h.  This module needs nothing else from it.
//============================================================================

module blstroid_mo #(
	// The horizontal calibration constant.  A displayed pixel at line-buffer
	// address A appears A pixel clocks after the SOS-2's /MOR reset point, and
	// where inside the line /MOR falls is hidden inside the custom.
	// `MO_H_OFFSET` is the value the display read pointer is PRESET to on /MOR,
	// so it absorbs that unknown.  With the blstroid_sos2_sync defaults
	// (MOR_H = 5, HB_END = 16) the read pointer runs
	//
	//     A(h, pix) = MO_H_OFFSET + 2*(h - MOR_H - 1) + pix      (mod 1024)
	//
	// and the first VISIBLE pixel is (h, pix) = (HB_END, 0), so making the
	// line-buffer address equal the screen column — which is MAME's `xpos`
	// exactly, atarimo.cpp folding xpos >= 640 by -1024 where the 10-bit counter
	// wraps — needs
	//
	//     MO_H_OFFSET = (1024 - 2*(HB_END - MOR_H - 1)) mod 1024 = 1004.
	//
	// MOR_H itself is pinned at 5 (not the 6 the sibling Atari cores use) by the
	// sheet-9 display-side read-and-clear: at 6 the fill counter free-runs two
	// 14M periods past the 1VD6H buffer flip and the clear blanks
	// X(link 56)+16/+17 on every scanline, which shows up as a black column.
	// The pairs (5, 1004) and (6, 1006) draw the identical picture, so this
	// constant must always move with MOR_H.
	parameter int MO_H_OFFSET = 1004,
	// 1 = walk the full 14-bit code when a tall object crosses a 1024-stamp
	// boundary (MAME's `code++`).  0 = the schematic's 10-bit wrap, because
	// RA14:5 is the only part of the address VMATCH touches.
	parameter bit MO_CODE_CARRY = 1'b0,
	// The VMATCH custom's own line counter (VCLK = /256H, reset by /VRES) counts
	// the line being PREPARED, not the one being displayed: the buffer filled
	// during the LAST line of the frame is displayed on line 0, so the compare
	// operand must wrap at V_TOTAL and not at 512.  /VRES's phase is inside the
	// SOS-2 and cannot be probed, but this is the only reading under which line
	// 0 of the picture is correct, and it is what MAME's `ypos` arithmetic
	// assumes.
	parameter int V_TOTAL = 262
)(
	input  logic        clk,          // clk_sys, 57.272724 MHz
	input  logic        reset,        // synchronous, active high
	input  logic        ce_14m,       // the LAST clk_sys cycle of each pixel
	input  logic        ce_7m,        // the LAST clk_sys cycle of each 1H count
	input  logic        pix,          // 0 = first pixel of the count, 1 = second

	// ---- SOS-2 3F (sheet 4), via blstroid_sos2_sync ----
	input  logic  [8:0] h,            // 1H..256H, 0..455
	input  logic  [8:0] vline,        // the VMATCH line counter (VCLK = /256H)
	input  logic        mol_n,        // SOS-2 pin 27 /MOL  — per-object load
	input  logic        mor_n,        // SOS-2 pin 26 /MOR  — per-line read reset
	input  logic        v1,           // 1V, raw (11L LS32 ping-pong)
	input  logic        v1d6h,        // 4E LS174C Q10 — 1V retimed on 4HD2H

	// ---- video-side VRAM read data (blstroid_vram owns the address bank) ----
	input  logic [15:0] vd,           // VD15:0, the continuously-driven RAM bus

	// ---- sprite ROM client (core/rtl/mem/blstroid_gfx_mem.sv) ----
	output logic        mo_req,
	output logic [18:2] mo_addr,      // WITHIN-HALF byte address of the stamp row
	input  logic        mo_ack,
	input  logic [63:0] mo_data,      // {N3,N2,N1,N0, M3,M2,M1,M0}

	// ---- to the sheet-10 priority network ----
	output logic  [3:0] lb_pix,       // TRUE pen; 0 = transparent
	output logic  [3:0] lb_col,       // TRUE 4-bit motion-object palette
	output logic  [9:0] mo_hpos,      // the line-buffer address those belong to

	// ---- status (debug; leave unconnected in synthesis) ----
	output logic        fetch_late,   // sticky: a ROM row missed its deadline
	output logic  [5:0] links_matched // VMATCH hits on the line just finished
);

	wire [2:0] hp = h[2:0];

	// =====================================================================
	// 1. The object word latches
	// =====================================================================
	// MOVPOS is on VD during h%8 == 0 and MOPIC during h%8 == 1.  On the board
	// ONE pin does both: VMATCH's INCLK (12E F10) is low only over count 1, so
	// its falling edge samples MOVPOS and its rising edge MOPIC.
	logic [8:0]  movpos_y;    // MOVPOS D15:7 — 9-bit V position
	logic [3:0]  movsiz;      // MOVPOS D3:0  — height - 1, in 8-line stamps

	always_ff @(posedge clk) begin
		if (reset) begin
			movpos_y <= '0;
			movsiz   <= '0;
		end else if (ce_7m && (hp == 3'd0)) begin
			movpos_y <= vd[15:7];
			movsiz   <= vd[3:0];
		end
	end

	// HFLD — VMATCH pin 31, the registered MOPIC D15 that drives the 4D LS86
	// address XORs and SOS-1 pin 7.  It is captured with MOPIC (count 1) and
	// must be HELD until the draw registers take it at count 5: the sixteen
	// pixels of THIS object run to count 5 of the NEXT group, so writing the
	// next object's flip straight into the shifter would mirror the second half
	// of every object — which shows up as a palindromic 16-pixel run.
	logic mopic_hflip;
	always_ff @(posedge clk) begin
		if (reset)                          mopic_hflip <= 1'b0;
		else if (ce_7m && (hp == 3'd1))     mopic_hflip <= vd[15];
	end

	// MOPIC, read COMBINATIONALLY off `vd` during count 1 — the ROM request is
	// issued inside that count, four clk_sys after `vd` settles, which is the
	// earliest the address can possibly be known and buys the SDRAM four extra
	// clk_sys of deadline over waiting for the ce_7m.
	wire [13:0] pic_code  = vd[13:0];
	wire        pic_vflip = vd[14];

	// =====================================================================
	// 2. VMATCH 8K (Atari 137552-001) — vertical match, size, ROM address
	// =====================================================================
	// The custom keeps its own copy of the line counter (VCLK = /256H, the same
	// edge the SOS-2's own V counter uses, /VRES the same frame reset) and
	// compares it with MOVPOS.  A column computed while the beam is on line V is
	// written into the OFF-SCREEN buffer and read out on line V+1 (the 1VD6H
	// ping-pong), so the operand is the DISPLAYED line D = V + 1 and the custom
	// carries that +1 — exactly the +1 the Atari System 1 equivalent carries
	// through its adder's C0.
	//
	//   U    = (Y + D) mod 512
	//   sum  = U[8:3] + (height-1) + 1                 (7 bits, 0..79)
	//   match      = sum >= 64          = sum[6]
	//   stamp row  = sum - 64           = sum[3:0]
	//   line in stamp = U[2:0]
	//
	// which is atarimo.cpp's `ypos = -(Y + 8*height)` covering 8*height lines,
	// re-derived: the object covers D iff U >= 512 - 8*height.
	wire [8:0] disp_line = (vline == 9'(V_TOTAL - 1)) ? 9'd0 : (vline + 9'd1);
	wire [8:0] u         = movpos_y + disp_line;
	/* verilator lint_off UNUSEDSIGNAL */   // vsum[5:4] cannot be set: the sum
	wire [6:0] vsum      = {1'b0, u[8:3]} + {3'b0, movsiz} + 7'd1;  // is <= 79
	/* verilator lint_on UNUSEDSIGNAL */

	wire       vm_match  = vsum[6];
	wire [3:0] stamp_row = vsum[3:0];
	wire [2:0] line_row  = u[2:0];

	// VFLIPEN (pin 30) is tied to PR2 = 1, so MOPIC D14 is honoured.  MAME draws
	// the stamp rows bottom-up and flips inside the stamp; the shipping ROM
	// never sets the bit, so this path is unexercised by the game and is here
	// only because the sheet wires the pin.
	wire [3:0] row_eff  = pic_vflip ? (movsiz - stamp_row) : stamp_row;
	wire [2:0] line_eff = pic_vflip ? ~line_row            : line_row;

	// RA14:5 = code[9:0] + row, TEN BITS WIDE — MOROM18:15 = code[13:10] comes
	// straight through 7F/6F and VMATCH never sees it, so the arithmetic wraps
	// inside the 1024-stamp block.  MO_CODE_CARRY = 1 restores MAME's full
	// 14-bit add for comparison.
	wire [9:0]  code_lo  = pic_code[9:0] + {6'b0, row_eff};
	wire [13:0] code_eff = MO_CODE_CARRY ? (pic_code + {10'b0, row_eff})
	                                     : {pic_code[13:10], code_lo};

	// {MOROM18:15, MOROM14:5, MOROM4:2} = 17 bits = the byte address 18:2 of the
	// stamp row inside ONE 512 KB half.  32 bytes per stamp per half, 4 per row,
	// so the slices concatenate with no adder.
	wire [16:0] rom_row_addr = {code_eff, line_eff};

	// =====================================================================
	// 3. The sprite-ROM client
	// =====================================================================
	logic [63:0] rom_fetched;
	logic        rom_rdy;
	logic        rom_pending;

	wire issue = ce_14m && (hp == 3'd1) && !pix;   // first ce_14m of the count
	wire late  = ce_7m  && (hp == 3'd5);           // the draw registers load here

	always_ff @(posedge clk) begin
		if (reset) begin
			mo_req      <= 1'b0;
			mo_addr     <= '0;
			rom_fetched <= '0;
			rom_rdy     <= 1'b0;
			rom_pending <= 1'b0;
			fetch_late  <= 1'b0;
		end else begin
			if (mo_ack) begin
				rom_fetched <= mo_data;
				mo_req      <= 1'b0;
				rom_rdy     <= 1'b1;
			end
			if (issue) begin
				rom_rdy     <= 1'b0;
				rom_pending <= vm_match;
				// A request still outstanding here means the previous row blew
				// its deadline by a whole group; never overlap two.
				if (mo_req && !mo_ack) fetch_late <= 1'b1;
				else if (vm_match) begin
					mo_addr <= rom_row_addr;
					mo_req  <= 1'b1;
				end
			end
			if (late && rom_pending && !rom_rdy && !mo_ack) fetch_late <= 1'b1;
		end
	end

	// =====================================================================
	// 4. SOS-1 10K — the draw-side registers (LDCLK = 4HD2H, h%8 5->6)
	// =====================================================================
	logic [63:0] draw_row;
	logic        draw_hflip;
	logic        draw_match;
	logic [3:0]  draw_col;      // MOPAL3:0 = MOHPOS D3:0, captured off VD

	always_ff @(posedge clk) begin
		if (reset) begin
			draw_row   <= '0;
			draw_hflip <= 1'b0;
			draw_match <= 1'b0;
			draw_col   <= '0;
		end else if (late) begin
			draw_row   <= mo_ack ? mo_data : rom_fetched;
			draw_match <= rom_pending && (rom_rdy || mo_ack);
			draw_col   <= vd[3:0];
			draw_hflip <= mopic_hflip;
		end
	end

	// ---- the pixel index inside the 16-pixel stamp row --------------------
	// h%8 = 6 is draw count 0 and each count is two pixels, so the index is
	// {(h%8 + 2) mod 8, pix}: 0..15 over counts 6, 7, 0, 1, 2, 3, 4, 5.
	wire [2:0] draw_cnt = hp + 3'd2;
	wire [3:0] pix_idx  = {draw_cnt, pix};

	// h-flip is a plain reversal of the sixteen pixels: {MOROM1, MOROM0}
	// reverses the four words and SOS-1 reverses the pixels inside each word.
	wire [3:0] pix_sel  = draw_hflip ? ~pix_idx : pix_idx;

	// pixel 4k+0 = N_k[7:4], 4k+1 = N_k[3:0], 4k+2 = M_k[7:4], 4k+3 = M_k[3:0]
	// with mo_data = {N3,N2,N1,N0, M3,M2,M1,M0}: M_k = [8k +: 8], N_k = [32+8k +: 8].
	wire [5:0] m_sh   = {1'b0, pix_sel[3:2], 3'b000};        //      8k
	wire [5:0] n_sh   = {1'b1, pix_sel[3:2], 3'b000};        // 32 + 8k
	wire [7:0] m_byte = draw_row[m_sh +: 8];
	wire [7:0] n_byte = draw_row[n_sh +: 8];

	// Written as one continuous assignment rather than an always_comb case:
	// Icarus cannot infer the sensitivity of a variable part-select inside an
	// always_* process and refuses to elaborate it.
	wire [3:0] pen = pix_sel[1] ? (pix_sel[0] ? m_byte[3:0] : m_byte[7:4])
	                            : (pix_sel[0] ? n_byte[3:0] : n_byte[7:4]);

	// MOPIX3:0 is the COMPLEMENT of the pen: 16J F21
	// ANDs the four and the result inhibits the write, so the inhibited value is
	// 0xF, and the pull-ups make an undriven bus read 0xF too.  With no vertical
	// match the shifter is never loaded and the bus stays at 0xF.
	wire [3:0] mopix = draw_match ? ~pen : 4'hF;
	wire [3:0] mopal = draw_col;

	// =====================================================================
	// 5. Census: VMATCH hits per line (the ROM rows the SDRAM must serve)
	// =====================================================================
	logic [5:0] links_cnt;
	wire        link_hit = (hp == 3'd1) && vm_match;
	always_ff @(posedge clk) begin
		if (reset) begin
			links_cnt     <= '0;
			links_matched <= '0;
		end else if (ce_7m) begin
			if (h == 9'd455) begin
				links_matched <= links_cnt + (link_hit ? 6'd1 : 6'd0);
				links_cnt     <= '0;
			end else if (link_hit) begin
				links_cnt <= links_cnt + 6'd1;
			end
		end
	end

	// =====================================================================
	// 6. The two line buffers (sheet 9)
	// =====================================================================
	// /MOL and /MOR are one 14M period wide, not one `1H` count.  The F163
	// chains on sheet 9 are clocked at 14M with a synchronous /LD, so a strobe
	// two 14M periods wide would load the counter twice — and the second load
	// would land on the 16th pixel of the object being drawn, writing it at the
	// next object's X.  The widths cannot be read off the sheet (they are inside
	// the SOS-2), but one 14M period is the only width at which the sheet-9
	// circuit draws sixteen contiguous pixels.
	// `blstroid_sos2_sync` decodes the strobes from H alone and so asserts them
	// for a whole count; the second half of the count is selected here.  /MOR is
	// qualified the same way for symmetry — for the READ counter the two widths
	// are indistinguishable, because both loads write the same value.
	wire mol_pulse_n = mol_n | ~pix;
	wire mor_pulse_n = mor_n | ~pix;

	blstroid_mo_linebuf #(.MO_H_OFFSET(MO_H_OFFSET)) u_lb (
		.clk      (clk),
		.reset    (reset),
		.ce_14m   (ce_14m),
		.mol_n    (mol_pulse_n),
		.mor_n    (mor_pulse_n),
		.v1       (v1),
		.v1d6h    (v1d6h),
		.a_in     (vd[15:6]),
		.mopix    (mopix),
		.mopal    (mopal),
		.lb_pix   (lb_pix),
		.lb_col   (lb_col),
		.mo_hpos  (mo_hpos)
	);

endmodule
