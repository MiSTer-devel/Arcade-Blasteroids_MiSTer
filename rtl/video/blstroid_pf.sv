`timescale 1ns/1ps
//============================================================================
//  Blasteroids — SP-316 sheet 6: the playfield picture latches (4K LS174B,
//  4L LS374C, 6F F174A), the four PF graphics ROMs (1L, 1M, 3L, 3M), the
//  2J/1J LS194A shifters and the 4E LS174C re-timing latch.
//
//  Copyright (C) 2026 RetroShrimp.  GPL-3.0-or-later; see LICENSE and
//  NOTICE.md.  Structurally derived from the Xybots MiSTer core's
//  rtl/video/xybots_pf.sv (GPL-3.0, same author); every number below is
//  re-derived for Blasteroids from sheet 6.
//
//  ---------------------------------------------------------------------------
//  What the playfield is on this board
//  ---------------------------------------------------------------------------
//  A 64 x 32 word map at FFC000, of which 40 x 30 stamps are visible.  Each
//  stamp is 16 x 8 screen pixels built from 8 x 8 ROM pixels doubled
//  horizontally, and there is no scrolling and no h-flip or v-flip bit — the
//  map word is `PPP CCCC CCCC CCCC` (3-bit palette + 13-bit code) and the map
//  address is a pure function of the H and V counters, formed in
//  `blstroid_vram`.  So the whole layer is
//
//      pen(x,y) = tiles[ map[y>>3][x>>4].code * 32 + (y & 7) * 4 + ((x>>2) & 3) ]
//                       nibble ((x >> 1) & 1) ? low : high
//      pal(x,y) = map[y>>3][x>>4].palette
//
//  ---------------------------------------------------------------------------
//  The fetch sequence, in `1H` counts
//  ---------------------------------------------------------------------------
//  `H[2:0]` = {4H, 2H, 1H}.  Sheet 5's slot schedule gives the playfield
//  exactly one video-RAM slot per 8 counts, at `H[2:0]` = 3, and sheet 6 latches
//  the word on the `4H` rising edge at the 3->4 boundary.  For map column
//  g = `h[8:3]`:
//
//    end of 8g+3   F EDGE.  `vd` holds column g's map word.  4K/4L/6F latch it
//                  on the board; here the 13-bit code goes straight out as ONE
//                  `blstroid_gfx_mem` PF request for the whole 4-byte stamp
//                  row, and the previously completed row moves down the
//                  prefetch pipe (`row_new` -> `row_mid`).
//    end of 8g+11  the NEXT F edge — the deadline.  Column g's row must have
//                  arrived by here: 8 counts = 64 clk_sys, comfortably inside
//                  what the graphics memory arbiter guarantees.  `late` pulses
//                  if it has not.
//    end of 8g+15  S EDGE.  `row_mid` -> `row_cur`, `pal_mid` -> `pal_cur`.
//    8g+16..8g+23  group g+2 displays column g: ROM pixel `H[2:0]`, two screen
//                  pixels each.
//
//  So the row reaches the screen 12 counts = 96 clk_sys after its map word was
//  read, against the board's ~2 counts of asynchronous 27512 access.  That is
//  the one deliberate difference from the PCB and it is invisible in the
//  picture, because the sync generator's `HB_END = 16` is exactly the same two
//  stamp groups: column 0 still lands on screen x = 0.  See the note in
//  `blstroid_sos2_sync.sv`.
//
//  ---------------------------------------------------------------------------
//  ROM address
//  ---------------------------------------------------------------------------
//      A15..A5 = PFPIC10..PFPIC0     A4..A2 = 4V, 2V, 1V     A1, A0 = /4H, 2H
//      chip    = {PFPIC12, PFPIC11}  -> MAME `tiles` offset * 0x10000
//  so the linear byte address is `code * 32 + line * 4 + byte_in_row` and the
//  `blstroid_gfx_mem` request address (bits 17:2 of it) is simply
//  `{code[12:0], line[2:0]}` — no multiply, no adder.
//  `{A1, A0}` = `{/4H, 2H}` runs 0, 0, 1, 1, 2, 2, 3, 3 over `H[2:0]` = 4..3,
//  i.e. byte 0 first, which is the `b0` of `pf_data`.
//
//  ---------------------------------------------------------------------------
//  The 2J / 1J LS194A shifters and the horizontal doubling
//  ---------------------------------------------------------------------------
//  `S0` is tied to `PR3` = 1 and `S1` = `1H`, so `1H` alone picks the mode:
//  `1H = 1` -> parallel load, `1H = 0` -> shift right.  `CK` = `/7M`, whose
//  rising edge is the MIDDLE of a count.  Immediately after a load
//  `PFPIX3:0` = `{JD7, JD6, JD5, JD4}` (the high nibble is the LEFT pixel);
//  after one shift it is `{JD3, JD2, JD1, JD0}`.  Each pen therefore lasts one
//  whole `1H` count = two 14.318 MHz pixel clocks — that IS the horizontal
//  doubling, there is no separate doubling circuit.
//
//  This model runs the same load/shift alternation but clocks it on the `ce_7m`
//  boundary instead of the board's mid-count `/7M` edge, so the pen occupies
//  the whole count it belongs to rather than straddling two counts by half.
//  The board recovers the same alignment downstream (sheet 10's 14C F74 on
//  `/14M` and the two F374s on `14M`); doing it here keeps `pf_pix` exactly on
//  the `hpos` grid the compositor uses:
//
//      during count h, ROM pixel index rp = h[2:0]; byte = rp >> 1;
//      nibble = rp even ? high : low;
//      screen x = 16 * (h[8:3] - 2) + 2 * h[2:0] + pix = hpos.
//
//  ---------------------------------------------------------------------------
//  What else this sheet produces
//  ---------------------------------------------------------------------------
//  `4E LS174C` (CK = `4HD2H`, rising at the 5->6 boundary) re-times three
//  unrelated signals two counts after the map latch:
//      D3 = PFM2 -> Q2 = PFPAL2      D13 = PMLNK8 -> Q12 = MOLNK8
//      D4 = PFM1 -> Q5 = PFPAL1      D11 = 1V     -> Q10 = 1VD6H
//      D6 = PFM0 -> Q7 = PFPAL0
//  `MOLNK8` goes back to sheet 5 as `VRAM10` in the four motion-object slots
//  (`blstroid_vram.molnk8`) and `1VD6H` is the sheet-9 line-buffer ping-pong
//  select, so this module exports both even though neither is a pixel.
//  `PFM2:0` themselves are the 4K LS174B latch, modelled in `blstroid_vram` so
//  that the IRQ1 path does not have to depend on the playfield module.
//============================================================================

module blstroid_pf #(
	// Visible map columns.  Column c is displayed in stamp group c + 2 (the
	// two-group prefetch above) and the sync generator's first visible group is
	// HB_END/8, so this module and `blstroid_sos2_sync` agree only while
	// HB_END == 16.  Groups 40..56 are fetched by the board but never seen;
	// skipping them keeps the SDRAM load at the 41 rows per line budgeted for.
	parameter int PF_COLS_VIS = 40
)(
	input  logic        clk,
	input  logic        reset,
	input  logic        ce_7m,       // one `1H` count

	// ---- from blstroid_sos2_sync ----------------------------------------
	input  logic  [8:0] h,           // 1H..256H
	input  logic  [7:0] v,           // 1V..128V

	// ---- from blstroid_vram ---------------------------------------------
	input  logic [15:0] vd,          // VD15:0, valid at every count's ce_7m
	input  logic  [2:0] pfm,         // 4K LS174B Q2/Q5/Q7 = PFM2, PFM1, PFM0
	input  logic        pmlnk8,      // 13E LS74 Q5

	// ---- blstroid_gfx_mem playfield client ------------------------------
	output logic        pf_req,
	output logic [17:2] pf_addr,     // {code[12:0], line[2:0]}
	input  logic        pf_ack,
	input  logic [31:0] pf_data,     // {b3, b2, b1, b0}

	// ---- the layer ------------------------------------------------------
	output logic  [3:0] pf_pix,      // PFPIX3:0 — the 4-bit pen
	output logic  [2:0] pf_pal,      // PFPAL2:0 — the 3-bit palette

	// ---- 4E LS174C, the two non-playfield signals it re-times ------------
	output logic        molnk8,      // Q12 -> blstroid_vram VRAM10 (MO slots)
	output logic        v1d6h,       // Q10 -> sheet 9 line-buffer ping-pong

	// ---- a stamp row missed the 64 clk_sys deadline (bench / debug) ------
	output logic        late
);

	// The two pipeline edges (see the header).  `ce_7m` marks the LAST
	// clk_sys cycle of the count `h` names, so "the 3->4 boundary" is
	// `ce_7m & (h[2:0] == 3)`.
	wire f_edge = ce_7m & (h[2:0] == 3'd3);   // 4H rising  — map latch + fetch
	wire s_edge = ce_7m & (h[2:0] == 3'd7);   // the display swap
	wire p_edge = ce_7m & (h[2:0] == 3'd5);   // 4HD2H rising — 4E LS174C

	localparam logic [5:0] NCOL = PF_COLS_VIS[5:0];
	wire col_used = (h[8:3] < NCOL);

	// =====================================================================
	// 1. 4E LS174C — PFPAL2:0, MOLNK8, 1VD6H (CK = 4HD2H)
	// =====================================================================
	// 4HD2H rises at the 5->6 boundary, two counts after 4H, so it captures
	// PFM2:0 of the SAME map word 4K latched at the 3->4 boundary, and PMLNK8
	// of the same MOLINK 13E latched at the 4->5 boundary.
	logic [2:0] pfpal;
	always_ff @(posedge clk) begin
		if (reset) begin
			pfpal  <= 3'd0;
			molnk8 <= 1'b0;
			v1d6h  <= 1'b0;
		end else if (p_edge) begin
			pfpal  <= pfm;
			molnk8 <= pmlnk8;
			v1d6h  <= v[0];
		end
	end

	// =====================================================================
	// 2. The stamp-row prefetch pipe
	// =====================================================================
	// One outstanding request at a time, as the graphics memory client contract
	// asks: raise `pf_req` with the address, hold both, drop on `pf_ack`.
	logic [31:0] row_new, row_mid, row_cur;
	logic  [2:0] pal_mid, pal_cur;
	logic        got;                  // the outstanding fetch has landed

	// An ack can arrive in the very cycle of the F edge; take the data from the
	// bus in that case so the row is never a whole stamp stale.
	wire        ack_now = pf_req & pf_ack;
	wire [31:0] row_in  = ack_now ? pf_data : row_new;
	wire        got_now = got | ack_now;

	always_ff @(posedge clk) begin
		if (reset) begin
			pf_req  <= 1'b0;
			pf_addr <= '0;
			row_new <= '0;
			row_mid <= '0;
			row_cur <= '0;
			pal_mid <= 3'd0;
			pal_cur <= 3'd0;
			got     <= 1'b1;          // nothing outstanding out of reset
			late    <= 1'b0;
		end else begin
			late <= 1'b0;

			if (ack_now) begin
				row_new <= pf_data;
				pf_req  <= 1'b0;
				got     <= 1'b1;
			end

			if (f_edge) begin
				// the deadline: the row requested at the PREVIOUS F edge is due
				if (!got_now) late <= 1'b1;

				row_mid <= row_in;
				pal_mid <= pfpal;      // 4E's output, already this column's

				// 4K / 4L / 6F latch the new map word; the fetch starts here
				pf_addr <= {vd[12:0], v[2:0]};
				pf_req  <= col_used;
				got     <= ~col_used;
				if (!col_used) row_new <= '0;
			end

			if (s_edge) begin
				row_cur <= row_mid;
				pal_cur <= pal_mid;
			end
		end
	end

	// =====================================================================
	// 3. 2J / 1J LS194A — parallel load, then one shift, per ROM byte
	// =====================================================================
	// On the board `S1` = `1H` loads inside the ODD count and shifts inside the
	// EVEN one, on the mid-count `/7M` edge, so a board pen straddles two counts
	// by half.  Here the same alternation is clocked on the `ce_7m` boundary and
	// `rp_n` is the ROM pixel index of the count that is about to START: the
	// decision is made one count ahead, so the pen occupies exactly the count it
	// belongs to and `pf_pix` sits on the `hpos` grid (see the header).  At the
	// S edge the source is `row_mid`, which becomes `row_cur` on that same edge.
	wire [31:0] row_use = s_edge ? row_mid : row_cur;
	wire  [2:0] rp_n    = h[2:0] + 3'd1;     // wraps 7 -> 0 at the S edge

	// JD7:0, the byte bus the four PF ROMs share; `{A1, A0}` = `{/4H, 2H}` is
	// `rp_n[2:1]`, i.e. byte 0 over ROM pixels 0-1, byte 1 over 2-3 and so on.
	// A continuous assignment rather than always_comb so that no tool has to
	// infer a sensitivity list over the part-selects (Icarus widens it).
	wire [7:0] jd = (rp_n[2:1] == 2'd0) ? row_use[7:0]
	              : (rp_n[2:1] == 2'd1) ? row_use[15:8]
	              : (rp_n[2:1] == 2'd2) ? row_use[23:16]
	                                    : row_use[31:24];

	logic [7:0] sr;                          // {2J, 1J} as one 8-bit register
	always_ff @(posedge clk) begin
		if (reset)       sr <= 8'd0;
		else if (ce_7m) begin
			if (!rp_n[0]) sr <= jd;              // S1 S0 = 11: parallel load
			else          sr <= {sr[3:0], 4'h0}; // S1 S0 = 01: shift right
		end
	end

	assign pf_pix = sr[7:4];
	assign pf_pal = pal_cur;

	// v[7:3] is the map row, formed in blstroid_vram's address multiplexer.
	/* verilator lint_off UNUSEDSIGNAL */
	wire unused_pf = |v[7:3] | |vd[15:13];
	/* verilator lint_on UNUSEDSIGNAL */

endmodule
