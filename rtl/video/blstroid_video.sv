`timescale 1ns/1ps
//============================================================================
//  Blasteroids — the whole video board (SP-316 sheets 4 to 11) as one module:
//  the SOS-2 sync generator, the video/work RAM and its 8-count slot schedule,
//  the playfield pipeline, the motion-object engine, the priority RAM, the
//  colour RAM and the DACs.  This is the rest of the core's single point of
//  contact with the video side.
//
//  Copyright (C) 2026 RetroShrimp.  GPL-3.0-or-later; see LICENSE and
//  NOTICE.md.  Port style and instantiation order ported from the Xybots MiSTer
//  core's rtl/video/xybots_video.sv (GPL-3.0, same author).
//
//  ---- where the RAMs live -----------------------------------------------
//  `blstroid_main.sv` contains the 16 KB video/work RAM, the 512 x 16 colour
//  RAM and the 256 x 1 priority RAM, because the 68000 write strobes are
//  decoded there.  Either
//
//    (a) set `INTERNAL_VRAM/CRAM/PRIO = 0` and wire this module's
//        `vram_a`/`vram_dout`, `cram_vaddr`/`cram_vdata`,
//        `prio_vaddr`/`prio_vdata` to `blstroid_main`'s video read ports,
//        leaving the CPU ports here unconnected — what `blstroid_core` does; or
//    (b) leave them at 1 and drive the CPU ports here instead, which makes this
//        module self-contained.
//
//  Never both: two copies of a RAM the 68000 writes will diverge.  An external
//  array must present a one-clk_sys registered read, which is what
//  `blstroid_main`'s video ports do.
//
//  ---- the picture lags the raster by two pixel clocks --------------------
//  `blstroid_cram` registers the PF/MO decision and the colour address once and
//  the colour word once, so the RGB for layer pixel P leaves during pixel P+2
//  (`VID_LATENCY`).  The `vid_*` sync/blank/DE outputs are delayed by the same
//  two pixels and are the ones a scaler must use.  The `h3`/`v3`/`hblank_cpu`/
//  `vblank`/`pfm2` outputs are raw and are the ones `blstroid_main` must use:
//  the 68000's FF9C01 status bits, the halt-until-HBLANK flip-flop and the IRQ1
//  gate all sample the board's own signals, not the delayed picture.
//
//  ---- the two HBLANKs ---------------------------------------------------
//  This module exports two active-high horizontal blanking signals.  Wiring the
//  wrong one to `blstroid_main` silently kills the row-programmable interrupt:
//  no error and no visible artefact, just a game that never takes an IRQ1 and
//  therefore never switches motion-object bands.
//
//    `hblank`      the DISPLAYED window, `HB_END` = 16 … `HB_START` = 336.
//                  `HB_END` is two whole stamp groups instead of the board's
//                  ~6, to buy the SDRAM playfield prefetch a full group.  The
//                  picture is identical: 16 is a whole number of groups and 640
//                  is 40 whole stamps.  This is what the compositor and the
//                  scaler use.
//
//    `hblank_cpu`  the BOARD's window, `HB_END_CPU` = 6 … `HB_START_CPU` = 326.
//                  This is what the 68000 sees at FF9C01 D4, what releases the
//                  halt-until-HBLANK flip-flop, and what clocks the sheet-2
//                  13C LS74 that makes IRQ1.
//
//  Why they cannot be one signal.  Sheet 2 clocks the IRQ1 latch on the rising
//  edge of HBLANK and samples `PFM2`, and `PFM2` is the 4K LS174B's latched
//  VD15 of the last playfield word fetched — a value that changes once every
//  8 H counts all the way across the line, because the sheet-5 slot schedule is
//  a pure function of H[2:0] and the LS174B's 4H clock is ungated.  Sheet 12
//  puts the row-interrupt flag in column 40 only.  Column 40 is fetched in the
//  group h = 320…327 and latched at the end of count 323, so
//
//      PFM2 holds the column-40 flag for counts 324 … 331, and nowhere else.
//
//  HBLANK must therefore rise inside that one stamp group, which pins
//  `HB_START_CPU` to 325 … 332 — an independent derivation of a constant the
//  video sheets leave inside the SOS-2 custom, and the reason the board's value
//  is ~326.  The displayed `HB_START` of 336 is four counts past the end of
//  that window: it samples column 41, whose D15 is ordinary map data, and the
//  interrupt never fires.  Wiring `hblank` to `blstroid_main` gives a cold boot
//  with no level-1 interrupts and no FF8E00 halts at all; `hblank_cpu` gives
//  both.
//
//  The cost of the prefetch deviation is therefore just this: the DISPLAYED
//  blanking window moves by ~10 counts (1.4 µs of a 63.7 µs line) and nothing
//  else, provided the CPU side keeps the board's own edge.  Set `HB_END` = 6 /
//  `HB_START` = 326 and give `blstroid_pf` a one-group prefetch and the two
//  signals become identical again.
//
//  ---- the two calibration constants on the video side --------------------
//  `MO_H_OFFSET` (see `blstroid_mo.sv`) is tied to `blstroid_sos2_sync`'s
//  `MOR_H` and to `HB_END`; changing one alone moves every motion object
//  sideways.  The shipped pair is (MOR_H = 5, MO_H_OFFSET = 1004).  The picture
//  is identical for (6, 1006), but that pair lets the sheet-9 display-side
//  read-and-clear eat two line-buffer locations per scanline and shows as thin
//  vertical black columns.  Move them together or not at all.
//
//  `MOL_H3` must be 5.  /MOL loads the line-buffer write counter from VD15:6
//  while MOHPOS is on the video bus, and sheet 5 puts MOHPOS at H[2:0] = 5.
//  Any other value loads the counter from the wrong RAM word and, again, moves
//  every motion object sideways.  This module passes 5 explicitly even though
//  `blstroid_sos2_sync` already defaults to it, so that an edit to either shows
//  up as a disagreement rather than as a silent change.
//============================================================================

module blstroid_video #(
	// ---- sync generator (see blstroid_sos2_sync for the derivations) ----
	parameter int H_TOTAL         = 456,
	parameter int HB_END          = 16,
	parameter int HB_START        = 336,
	// The BOARD's HBLANK window, which is not the displayed one — see "the two
	// HBLANKs" in the header.  `HB_START_CPU` must put the rising edge inside
	// the column-40 stamp group, i.e. it must be in 325 … 332.
	parameter int HB_END_CPU      = 6,
	parameter int HB_START_CPU    = 326,
	parameter int V_TOTAL         = 262,
	parameter int VB_START        = 240,
	parameter int MOR_H           = 5,      // pinned by the sheet-9 edge clear
	// ---- motion objects ----
	parameter int MO_H_OFFSET     = 1004,   // moves with MOR_H; see the header
	parameter bit MO_CODE_CARRY   = 1'b0,   // 0 = the stamp code wraps at 10 bits
	// ---- compositor ----
	parameter bit DAC_MAME_COMPAT = 1'b1,   // see blstroid_dac.sv
	parameter int PF_COLS_VIS     = 40,
	// ---- where the RAMs live ----
	parameter bit INTERNAL_VRAM   = 1'b1,
	parameter bit INTERNAL_CRAM   = 1'b1,
	parameter bit INTERNAL_PRIO   = 1'b1
)(
	input  logic        clk,          // clk_sys, 57.272724 MHz
	input  logic        reset,
	input  logic        ce_14m,       // clk_sys/4 — one PIXEL
	input  logic        ce_7m,        // clk_sys/8 — one `1H` COUNT

	// =====================================================================
	// 68000 ports (used only where the matching INTERNAL_* is 1)
	// =====================================================================
	// video / work RAM, FFC000-FFFFFF
	input  logic [12:0] cpu_a,        // BA13..BA1
	input  logic [15:0] cpu_din,
	input  logic        cpu_sel,      // ~/VIDRAM & AS, held for the bus cycle
	input  logic        cpu_we_hi,
	input  logic        cpu_we_lo,
	output logic [15:0] cpu_dout,
	output logic        cpu_slot,
	output logic        cpu_ack,
	// colour RAM, FFA000-FFA3FE
	input  logic  [8:0] cram_cpu_a,   // BA9..BA1
	input  logic [15:0] cram_cpu_din,
	input  logic        cram_cpu_we_hi,
	input  logic        cram_cpu_we_lo,
	output logic [15:0] cram_cpu_dout,
	// priority RAM, FF8800-FF89FE (write only, D0)
	input  logic        prio_cpu_we,
	input  logic  [7:0] prio_cpu_a,   // BA8..BA1
	input  logic        prio_cpu_d,

	// =====================================================================
	// external RAM ports (used only where the matching INTERNAL_* is 0)
	// =====================================================================
	output logic [12:0] vram_a,
	output logic [15:0] vram_din,
	output logic        vram_we_hi,
	output logic        vram_we_lo,
	input  logic [15:0] vram_dout,
	output logic  [8:0] cram_vaddr,
	input  logic [15:0] cram_vdata,
	output logic  [7:0] prio_vaddr,
	input  logic        prio_vdata,

	// =====================================================================
	// graphics-SDRAM read clients; see blstroid_gfx_mem.sv for the handshake
	// =====================================================================
	output logic        pf_req,
	output logic [17:2] pf_addr,
	input  logic        pf_ack,
	input  logic [31:0] pf_data,
	output logic        mo_req,
	output logic [18:2] mo_addr,
	input  logic        mo_ack,
	input  logic [63:0] mo_data,

	// =====================================================================
	// RAW board timing — what blstroid_main must sample
	// =====================================================================
	output logic  [2:0] h3,           // {4H, 2H, 1H}
	output logic  [2:0] v3,           // {4V, 2V, 1V}
	output logic        h8,           // 8H — the LETA CK pin
	output logic  [8:0] h,
	output logic  [8:0] vline,
	output logic        hblank,       // ACTIVE HIGH — the DISPLAYED window
	output logic        hblank_cpu,   // ACTIVE HIGH — the BOARD's window; this
	                                  // is the one blstroid_main must take
	output logic        vblank,       // ACTIVE HIGH -> FF9C01 D5
	output logic        pfm2,         // 4K LS174B Q2 — the IRQ1 row condition
	output logic        hblank_rise_o,// clocks 13C LS74 (IRQ1) + the halt FF
	output logic        hblank_fall_o,
	output logic  [9:0] hpos,         // 0..639 while `de_raw` — the LAYER grid
	output logic  [8:0] vpos,         // 0..239 while `de_raw`
	output logic        de_raw,       // layer-time display enable (NOT delayed)

	// =====================================================================
	// the picture — DELAYED by VID_LATENCY, this is what a scaler uses
	// =====================================================================
	output logic  [7:0] red,
	output logic  [7:0] green,
	output logic  [7:0] blue,
	output logic        vid_hs_n,
	output logic        vid_vs_n,
	output logic        vid_cs_n,
	output logic        vid_hb,       // active high
	output logic        vid_vb,       // active high
	output logic        vid_de,
	output logic        ce_pix,       // = ce_14m, for the scaler

	// =====================================================================
	// status and observation only — nothing in the picture path reads these
	// =====================================================================
	output logic        mo_fetch_late,
	output logic        pf_fetch_late,
	output logic  [5:0] mo_links_matched,
	output logic  [3:0] lb_pix,
	output logic  [3:0] lb_col,
	output logic  [9:0] mo_hpos,
	output logic  [3:0] pf_pix,
	output logic  [2:0] pf_pal,
	output logic        pf_win
);

	// The compositor's own depth, in pixel clocks: blstroid_cram stage 1
	// (14C F74 + 17D/16D F374) plus stage 2 (the two HC273s).
	localparam int VID_LATENCY = 2;

	// =====================================================================
	// 1. SOS-2 at 3F — sheet 4
	// =====================================================================
	logic       m7, pix;
	logic [7:0] v;
	logic       linkres_n, mol_n, mor_n;
	logic       selcpu, ramfree;
	logic       hblank_i, vblank_i, blank_n, hsync_n, vsync_n, csync_n;
	logic       hblank_rise, hblank_fall;

	/* verilator lint_off PINCONNECTEMPTY */
	blstroid_sos2_sync #(
		.H_TOTAL (H_TOTAL),  .HB_END  (HB_END),  .HB_START(HB_START),
		.V_TOTAL (V_TOTAL),  .VB_START(VB_START),
		.MOR_H   (MOR_H),
		// /MOL must land on the count that carries MOHPOS (sheet 5, H[2:0] = 5)
		// — see the header.  Passed explicitly even though the module's own
		// default is already 5.
		.MOL_H3  (5)
	) u_sync (
		.clk(clk), .reset(reset), .ce_14m(ce_14m), .ce_7m(ce_7m),
		.m14(), .m14p_n(), .m14_n(), .m7(m7), .m7_n(), .pix(pix),
		.h(h), .v(v), .v256(), .vline(vline),
		.h1_n(), .h2_n(), .h4_n(), .h256(), .h256_n(),
		.h_4hd1h(), .h_4hd2h(), .h_4hd2h_n(), .v1_n(),
		.hblank(hblank_i), .vblank(vblank_i), .vblank_n(), .blank_n(blank_n),
		.hsync_n(hsync_n), .vsync_n(vsync_n), .csync_n(csync_n),
		.vres_n(), .linkres_n(linkres_n), .mol_n(mol_n), .mor_n(mor_n),
		.selcpu(selcpu), .ramfree(ramfree), .ramcpu_n(), .vram_sel(),
		.inclk(), .n7m1h(),
		.hpos(hpos), .vpos(vpos), .hde(), .vde(), .de(de_raw),
		.line_start(), .frame_start(), .vblank_start(),
		.hblank_rise(hblank_rise), .hblank_fall(hblank_fall)
	);
	/* verilator lint_on PINCONNECTEMPTY */

	assign h3     = h[2:0];
	assign v3     = v[2:0];
	assign h8     = h[3];
	assign hblank = hblank_i;

	// ---- the BOARD's HBLANK — see "the two HBLANKs" in the header ---------
	// A plain window on the same `h`, so it costs two comparators and cannot
	// drift from the raster.  Nothing in the picture path uses it.
	assign hblank_cpu = (h >= 9'(HB_START_CPU)) | (h < 9'(HB_END_CPU));
	assign vblank = vblank_i;
	assign hblank_rise_o = hblank_rise;
	assign hblank_fall_o = hblank_fall;
	assign ce_pix = ce_14m;

	// =====================================================================
	// 2. Video / work RAM — sheet 5
	// =====================================================================
	logic [15:0] vd;
	logic        pmlnk8, molnk8;
	logic  [2:0] pfm;

	/* verilator lint_off PINCONNECTEMPTY */
	blstroid_vram #(.INTERNAL_RAM(INTERNAL_VRAM)) u_vram (
		.clk(clk), .reset(reset), .ce_14m(ce_14m), .ce_7m(ce_7m),
		.h(h), .v(v), .m7(m7), .selcpu(selcpu), .ramfree(ramfree),
		.linkres_n(linkres_n), .molnk8(molnk8),
		.cpu_a(cpu_a), .cpu_din(cpu_din), .cpu_sel(cpu_sel),
		.cpu_we_hi(cpu_we_hi), .cpu_we_lo(cpu_we_lo),
		.cpu_dout(cpu_dout), .cpu_slot(cpu_slot), .cpu_ack(cpu_ack),
		.vd(vd), .vram_a(vram_a),
		.pmlnk8(pmlnk8), .mo_link(), .mo_word(), .mo_word_idx(),
		.mo_strobe(), .pf_strobe(), .pfm(pfm), .pfm_strobe(),
		.ram_din(vram_din), .ram_we_hi(vram_we_hi), .ram_we_lo(vram_we_lo),
		.ram_dout(vram_dout)
	);
	/* verilator lint_on PINCONNECTEMPTY */

	// ---- 4K LS174B (sheet 6): PFM2:0 = the map word's D15:13, on 4H -------
	// The latch itself lives in blstroid_vram, so the IRQ1 path does not depend
	// on whether the playfield module is drawing; both consumers read it here.
	assign pfm2 = pfm[2];

	// =====================================================================
	// 3. Playfield — sheet 6
	// =====================================================================
	logic v1d6h;

	blstroid_pf #(.PF_COLS_VIS(PF_COLS_VIS)) u_pf (
		.clk(clk), .reset(reset), .ce_7m(ce_7m),
		.h(h), .v(v), .vd(vd), .pfm(pfm), .pmlnk8(pmlnk8),
		.pf_req(pf_req), .pf_addr(pf_addr), .pf_ack(pf_ack), .pf_data(pf_data),
		.pf_pix(pf_pix), .pf_pal(pf_pal),
		.molnk8(molnk8), .v1d6h(v1d6h), .late(pf_fetch_late)
	);

	// =====================================================================
	// 4. Motion objects — sheets 7, 8, 9
	// =====================================================================
	blstroid_mo #(
		.MO_H_OFFSET  (MO_H_OFFSET),
		.MO_CODE_CARRY(MO_CODE_CARRY),
		.V_TOTAL      (V_TOTAL)
	) u_mo (
		.clk(clk), .reset(reset), .ce_14m(ce_14m), .ce_7m(ce_7m), .pix(pix),
		.h(h), .vline(vline),
		.mol_n(mol_n), .mor_n(mor_n), .v1(v[0]), .v1d6h(v1d6h),
		.vd(vd),
		.mo_req(mo_req), .mo_addr(mo_addr), .mo_ack(mo_ack), .mo_data(mo_data),
		.lb_pix(lb_pix), .lb_col(lb_col), .mo_hpos(mo_hpos),
		.fetch_late(mo_fetch_late), .links_matched(mo_links_matched)
	);

	// =====================================================================
	// 5. Priority RAM — sheet 10
	// =====================================================================
	/* verilator lint_off PINCONNECTEMPTY */
	blstroid_prio #(.INTERNAL_RAM(INTERNAL_PRIO)) u_prio (
		.clk(clk),
		.cpu_we(prio_cpu_we), .cpu_a(prio_cpu_a), .cpu_d(prio_cpu_d),
		.pf_pix(pf_pix), .pf_pal(pf_pal),
		.lb_pix(lb_pix), .lb_col(lb_col),
		.prio_addr(prio_vaddr), .prio_bit_in(prio_vdata),
		.pf_win(pf_win), .mo_opaque()
	);
	/* verilator lint_on PINCONNECTEMPTY */

	// =====================================================================
	// 6. Colour RAM and the output registers — sheet 10
	// =====================================================================
	logic [4:0] r5, g5, b5;

	/* verilator lint_off PINCONNECTEMPTY */
	blstroid_cram #(.INTERNAL_RAM(INTERNAL_CRAM)) u_cram (
		.clk(clk), .reset(reset), .ce_14m(ce_14m),
		.cpu_a(cram_cpu_a), .cpu_din(cram_cpu_din),
		.cpu_we_hi(cram_cpu_we_hi), .cpu_we_lo(cram_cpu_we_lo),
		.cpu_dout(cram_cpu_dout),
		.pf_pix(pf_pix), .pf_pal(pf_pal),
		.lb_pix(lb_pix), .lb_col(lb_col), .pf_win(pf_win),
		.blank_n(blank_n),
		.cram_vaddr(cram_vaddr), .cram_vdata(cram_vdata),
		.cra(), .mo_wins(),
		.r5(r5), .g5(g5), .b5(b5)
	);
	/* verilator lint_on PINCONNECTEMPTY */

	// =====================================================================
	// 7. The DACs — sheet 11
	// =====================================================================
	// `blank_n` is tied high here: blanking is already done by clearing the
	// HC273s inside blstroid_cram, which is how the board does it.
	blstroid_dac #(.DAC_MAME_COMPAT(DAC_MAME_COMPAT)) u_dac (
		.r5(r5), .g5(g5), .b5(b5), .blank_n(1'b1),
		.red(red), .green(green), .blue(blue)
	);

	// =====================================================================
	// 8. Sync / blanking delayed to match the picture
	// =====================================================================
	logic [VID_LATENCY-1:0] hs_d, vs_d, cs_d, hb_d, vb_d, de_d;

	always_ff @(posedge clk) begin
		if (reset) begin
			hs_d <= '1; vs_d <= '1; cs_d <= '1;
			hb_d <= '1; vb_d <= '1; de_d <= '0;
		end else if (ce_14m) begin
			hs_d <= {hs_d[VID_LATENCY-2:0], hsync_n};
			vs_d <= {vs_d[VID_LATENCY-2:0], vsync_n};
			cs_d <= {cs_d[VID_LATENCY-2:0], csync_n};
			hb_d <= {hb_d[VID_LATENCY-2:0], hblank_i};
			vb_d <= {vb_d[VID_LATENCY-2:0], vblank_i};
			de_d <= {de_d[VID_LATENCY-2:0], ~(hblank_i | vblank_i)};
		end
	end

	assign vid_hs_n = hs_d[VID_LATENCY-1];
	assign vid_vs_n = vs_d[VID_LATENCY-1];
	assign vid_cs_n = cs_d[VID_LATENCY-1];
	assign vid_hb   = hb_d[VID_LATENCY-1];
	assign vid_vb   = vb_d[VID_LATENCY-1];
	assign vid_de   = de_d[VID_LATENCY-1];

endmodule
