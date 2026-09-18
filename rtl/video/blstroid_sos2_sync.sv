`timescale 1ns/1ps
//============================================================================
//  Blasteroids — SP-316 sheet 4: the SOS-2 sync generator (3F, Atari
//  137550-001) and every discrete gate on that sheet that shapes its outputs
//  into the nets the other video sheets consume.
//
//  Copyright (C) 2026 RetroShrimp.  GPL-3.0-or-later; see LICENSE and
//  NOTICE.md.  Structurally ported from the Xybots MiSTer core's
//  rtl/video/xybots_syngen.sv (GPL-3.0, same author), which models the same
//  Atari sync-generator family; every number below is re-derived for
//  Blasteroids.
//
//  ---- what the sheet gives, and what it does not -------------------------
//  3F is a 40-pin Atari custom and SP-316 draws only its pinout, so the
//  counters are derived rather than read:
//
//    * `VCLOCK` (pin 39) = `/256H` (4D LS86 used as an inverter), so the
//      vertical counter is clocked by the H counter's MSB and the H counter
//      must wrap from a value >= 257 and <= 512.  It cannot be clocked at 14M
//      (912 does not fit in 9 bits), so it runs at 7M, and the only value in
//      257..512 that yields a standard 15.7 kHz line is 456:
//      7 159 090 / 456 = 15 700.6 Hz, 63.709 us.  That is MAME's 456*2, arrived
//      at independently.
//    * One `1H` count is therefore two 14.318181 MHz pixel clocks, and the
//      whole board counts horizontally in those 2-pixel units.  Only the motion
//      objects run at the full pixel rate.
//    * The visible window is 40 stamps x 16 px = 640 = 320 counts, and 30 stamp
//      rows x 8 lines = 240 — MAME's 640 x 240 exactly.
//
//  Everything else about the raster is inside the custom and the package gives
//  no decode at all: the vertical total, the reload behaviour of `/VRES`, and
//  the H/V positions of HBLANK, VBLANK, /HSYNC, /VSYNC, /LINKRES, /MOL, /MOR.
//  Pin 37 (`256V`) is a bare stub, so the board cannot even tell us whether the
//  V counter passes 255.  Each parameter below says how confident its value is.
//
//  ---- the one place this deliberately differs from the board -------------
//  `HB_END` is 16 rather than the ~6 the board almost certainly uses (Xybots'
//  equivalent falls at h = 9 of the same 456-count line).  On the PCB the
//  playfield stamp row comes out of an asynchronous 27512 about two counts
//  after the map word; here it comes from SDRAM, so `blstroid_pf` prefetches
//  two stamp groups and the row is due 8 counts = 64 clk_sys after the request,
//  twice the worst-case latency the graphics-memory arbiter can show.  The
//  picture and the H counter keep their real relationship; only the constant
//  that says which count the visible window starts on moves, and that constant
//  is inside the SOS-2 and unknown.  Two consequences:
//
//    * the visible content is identical.  Stamp column 0 is still screen x = 0
//      and column 39 still ends at x = 639, because HB_END is a whole number of
//      8-count stamp groups and 640 = 40 whole stamps;
//    * HBLANK — which the CPU reads at FF9C01 D4 and which drives IRQ1 and the
//      halt-until-HBLANK flip-flop — moves by about 10 counts = 1.4 us out of
//      the 63.7 us line.  Nothing in the ROM measures HBLANK to better than a
//      whole line; the level-1 handler simply stalls on it.  Note that
//      `blstroid_video` therefore exports a second, board-aligned HBLANK for
//      the CPU side; read its header before rewiring either one.
//
//  Set HB_END/HB_START back to 6/326 and give `blstroid_pf` a one-group
//  prefetch to get the board's own alignment.
//
//  ---- clocking and the phase convention ---------------------------------
//  One clk_sys domain (57.272727 MHz) with the core's two enables.  The pixel
//  clock is 14.318181 MHz — twice Xybots' — so:
//
//      ce_14m  = clk_sys/4   one PIXEL
//      ce_7m   = clk_sys/8   one `1H` COUNT = two pixels; a strict subset of
//                            ce_14m, so `ce_7m` implies `ce_14m`.
//
//  `ce_7m` marks the LAST clk_sys cycle of the count `h` currently names, and
//  `ce_14m` the last clk_sys cycle of the pixel `pix` currently names.  So in a
//  cycle where ce_7m is high, `h` still holds the count that is ending, and the
//  edge that ends that cycle makes it h+1.  Every marker exported here is
//  qualified that way (`line_start = ce_7m & (h == 0)`), and `blstroid_pf` and
//  `blstroid_vram` latch on the same convention.
//
//  `pix` is 0 for the first 14M period of a count and 1 for the second; `m7`
//  (the SOS-2's own 7M output, pin 13) is high over the first, which is the
//  half in which the video-RAM write strobes fire.  Which half 7M is high in is
//  not readable from the package, and nothing outside this module's `m7` and
//  `n7m1h` outputs depends on it.
//============================================================================

module blstroid_sos2_sync #(
	// ---- horizontal, derived from the VCLOCK = /256H wiring ---------------
	parameter int H_TOTAL  = 456,   // counts of 7M per line = 912 pixel clocks
	parameter int HB_END   = 16,    // first VISIBLE count   (see the note above)
	parameter int HB_START = 336,   // first BLANKED count; 336-16 = 320 = 640 px
	// ---- /HSYNC.  Inside the custom; taken from the same family (Xybots'
	//      SYNGEN puts it at these counts of the same 456-count line) -------
	parameter int HS_START = 377,   // /HSYNC low over 377..408 = 32 counts
	parameter int HS_END   = 409,   //   = 4.47 us, inside the 136-count blank
	// ---- vertical -------------------------------------------------------
	// 262 is MAME's figure, whose own comment says it comes from published
	// specs rather than measurement.  263 is what the Atari System 1 / Xybots
	// sync-generator family produces from its dumped sync PROM (240 visible +
	// 22 blanked + one extra line made by /VRES loading 0xFF), and the SOS-2
	// exposes the same /VRES net.  The SOS-2 counts V internally and the
	// schematics do not settle it, so this stays a build-time parameter:
	// 59.9227 Hz at 262, 59.6949 Hz at 263.  Rebuild with 263 to try it.
	parameter int V_TOTAL  = 262,
	parameter int VB_START = 240,   // derived: 30 stamp rows x 8 lines visible
	// ---- /VSYNC.  Inside the custom; from the family (Xybots' sync PROM) --
	parameter int VS_START = 243,   // /VSYNC low over lines 243..246
	parameter int VS_END   = 247,
	// ---- motion-object strobes -------------------------------------------
	// /MOR: one count per LINE.  Its position is inside 3F, and MO_H_OFFSET in
	// blstroid_mo absorbs any constant shift, but the value is not free.  At
	// h = 6 (the Xybots /MOHR position) two 14M periods sit between the 1VD6H
	// buffer flip at the h[2:0] 5->6 boundary and the preset, and during them
	// the display side's unconditional read-and-clear blanks line-buffer
	// addresses X(link 56)+16 and +17 on every scanline — 2-pixel vertical
	// black columns, visible on a real monitor.  h = 5 presets on the same edge
	// 1VD6H is registered on, still writes link 56's pixel 15, and loses
	// nothing.  h < 5 would corrupt link 56's own pixels 14/15.
	parameter int MOR_H    = 5,
	// /MOL: one count per 8-count GROUP, and derived rather than borrowed: the
	// sheet-9 line-buffer address counters load VD15:6 = MOHPOS's 10-bit X, and
	// MOHPOS is on VD only during H[2:0] = 5, so /MOL has to be low in that
	// count.  (Xybots' /MOHLD is at h = 7 because its slot schedule puts its own
	// X word there — do not carry that over.)
	parameter int MOL_H3   = 5
)(
	input  logic       clk,        // clk_sys, 57.272727 MHz
	input  logic       reset,      // synchronous, active high
	input  logic       ce_14m,     // clk_sys/4 — one PIXEL
	input  logic       ce_7m,      // clk_sys/8 — one `1H` COUNT

	// ---- sheet-4 clock tree (LEVELS, never use one as a clock) -----------
	output logic       m14,        // 14M    2E/F F04 11->10
	output logic       m14p_n,     // /14MP  2E/F F04  9-> 8 (the oscillator buffer)
	output logic       m14_n,      // /14M   SOS-2 pin 11
	output logic       m7,         // 7M     SOS-2 pin 13
	output logic       m7_n,       // /7M    SOS-2 pin 12
	output logic       pix,        // 0 = first 14M period of the count, 1 = second

	// ---- counters --------------------------------------------------------
	output logic [8:0] h,          // 1H..256H, SOS-2 pins 16,17,18,8,7,6,5,4,3
	output logic [7:0] v,          // 1V..128V, SOS-2 pins 28,30..36
	output logic       v256,       // SOS-2 pin 37 — a BARE STUB on the PCB
	output logic [8:0] vline,      // {v256, v}: the true 0..V_TOTAL-1 line index

	// ---- horizontal taps and the two delayed copies ----------------------
	output logic       h1_n,       // /1H  2E/F-family inverter
	output logic       h2_n,       // /2H  2E/F F04 5->6
	output logic       h4_n,       // /4H  2E/F F04 3->4
	output logic       h256,       // 256H
	output logic       h256_n,     // /256H = 4D LS86 (4,5->6) -> VCLOCK, VMATCH
	output logic       h_4hd1h,    // 4HD1H  SOS-2 pin  2  (4H delayed one count)
	output logic       h_4hd2h,    // 4HD2H  SOS-2 pin 19  (4H delayed two counts)
	output logic       h_4hd2h_n,  // /4HD2H = 4M LS04 (3->4), clocks 7F LS174

	// ---- vertical taps ---------------------------------------------------
	output logic       v1_n,       // /1V  2E/F F04 1->2

	// ---- blanking and sync ----------------------------------------------
	output logic       hblank,     // pin 21, ACTIVE HIGH -> FF9C01 D4
	output logic       vblank,     // pin 22, ACTIVE HIGH -> FF9C01 D5
	output logic       vblank_n,   // 4D LS86 (1,2->3)
	output logic       blank_n,    // /BLANK = 11J LS02 (8,9->10) = ~(HBLANK|VBLANK)
	output logic       hsync_n,    // pin 23
	output logic       vsync_n,    // pin 24
	output logic       csync_n,    // VID-9, 14D LS00 (12,13->11) then 15A U06 (1->2)

	// ---- frame / line resets and the MO strobes (all active low) ---------
	output logic       vres_n,     // pin 38 -> VMATCH pin 28
	output logic       linkres_n,  // pin 25 -> 13E LS74 /CLR, 7D LS273A /CLR
	output logic       mol_n,      // pin 27 -> 11L LS32 -> /MOHLD0//MOHLD1
	output logic       mor_n,      // pin 26 -> 11L LS32 -> /MOHR0//MOHR1

	// ---- the video-RAM slot schedule (sheet 5) --------------------------
	output logic       selcpu,     // 14D LS00 (9=/4H, 10=1H -> 8) = 4H | /1H
	output logic       ramfree,    // 11J LS02 + 14B LS08          = 2H & SELCPU
	output logic       ramcpu_n,   // 4M LS04 (13->12) = ~RAMFREE
	output logic [1:0] vram_sel,   // the LS253 code {2H, SELCPU}: 3 CPU, 2 PF,
	                               //   1/0 MO.  See blstroid_vram.
	output logic       inclk,      // 12E F10 (1,2,13->12) = ~(1H & /2H & /4H)
	                               //   -> VMATCH pin 25; LOW only at H[2:0] = 1
	output logic       n7m1h,      // 12F F00 (9,10->8) = ~(7M & 1H) -> SOS-1

	// ---- display coordinates (this core's contract, not a PCB net) -------
	output logic [9:0] hpos,       // 0..639 while `hde`
	output logic [8:0] vpos,       // 0..239 while `vde`
	output logic       hde,        // horizontal display enable = ~hblank
	output logic       vde,        // vertical   display enable = ~vblank
	output logic       de,         // hde & vde

	// ---- markers, one clk_sys cycle wide, aligned to ce_7m ---------------
	output logic       line_start, // h == 0
	output logic       frame_start,// h == 0 and vline == 0
	output logic       vblank_start,// h == 0 and vline == VB_START (VBLANK rising)

	// ---- the two HBLANK edges, one clk_sys cycle wide, aligned to ce_7m --
	// `hblank_rise` is the edge sheet 2 uses twice: it clocks 13C LS74 (the
	// row-programmable IRQ1) and 11D LS74 upper (halt-until-HBLANK).  It is the
	// last clk_sys cycle of count HB_START-1, i.e. the cycle whose clock edge
	// makes HBLANK high.
	output logic       hblank_rise,
	output logic       hblank_fall
);

	// Sized copies of the raster constants, so every compare below is a plain
	// 9-bit unsigned one and no tool has to guess about parameter widths.
	localparam logic [8:0] HTOT  = H_TOTAL [8:0];
	localparam logic [8:0] HBE   = HB_END  [8:0];
	localparam logic [8:0] HBS   = HB_START[8:0];
	localparam logic [8:0] HSS   = HS_START[8:0];
	localparam logic [8:0] HSE   = HS_END  [8:0];
	localparam logic [8:0] VTOT  = V_TOTAL [8:0];
	localparam logic [8:0] VBS   = VB_START[8:0];
	localparam logic [8:0] VSS   = VS_START[8:0];
	localparam logic [8:0] VSE   = VS_END  [8:0];
	localparam logic [8:0] MORH  = MOR_H   [8:0];
	localparam logic [2:0] MOLH3 = MOL_H3  [2:0];
	localparam int         LASTGI = (H_TOTAL/8) - 1;
	localparam logic [5:0] LASTG  = LASTGI[5:0];

	// =====================================================================
	// 1. Pixel phase inside the count
	// =====================================================================
	// ce_7m is a strict subset of ce_14m (blstroid_core: both are decodes of one
	// free-running /32 counter), so the two assignments below cannot fight and
	// `pix` re-locks to the enables within one count after any disturbance.
	always_ff @(posedge clk) begin
		if (reset)       pix <= 1'b0;
		else if (ce_7m)  pix <= 1'b0;   // ce_7m ends the count: next is pixel 0
		else if (ce_14m) pix <= 1'b1;   // the mid-count 14M edge
	end

	// A clk_sys/4 sub-phase locked to ce_14m recovers the two halves of a pixel,
	// which is what the sheet-9 line buffers need ("write while 14M is low, read
	// while it is high").  ce_14m marks the last clk_sys
	// cycle of a pixel, so ph runs 0,1,2,3 with ce_14m at ph == 3, and 14M is
	// high over the first half.  These are LEVELS: never use one as a clock.
	logic [1:0] ph;
	always_ff @(posedge clk) begin
		if (reset)       ph <= 2'd0;
		else if (ce_14m) ph <= 2'd0;
		else             ph <= ph + 2'd1;
	end

	assign m14    = ~ph[1];      // 14M    2E/F F04 11->10
	assign m14_n  = ~m14;        // /14M   SOS-2 pin 11
	assign m14p_n = ~m14;        // /14MP  2E/F F04 9->8 (the oscillator buffer)

	assign m7   = ~pix;   // 7M high over the first pixel of the count
	assign m7_n =  pix;

	// =====================================================================
	// 2. The counters
	// =====================================================================
	// There is no reset on the PCB (the SOS-2 has no reset pin at all and
	// /VRES is an OUTPUT), so the counters free-run from power-on.  The
	// synchronous reset below exists only so that simulation starts on a known
	// state; the core holds it low once the ROM download has finished.
	logic [8:0] vcnt;

	wire last_h = (h == HTOT - 9'd1);
	wire last_v = (vcnt == VTOT - 9'd1);

	always_ff @(posedge clk) begin
		if (reset) begin
			h    <= 9'd0;
			vcnt <= '0;
		end else if (ce_7m) begin
			if (last_h) begin
				h <= 9'd0;
				// The V counter is clocked by /256H (SOS-2 pin 39), i.e. by the
				// FALLING edge of 256H.  With a 0..455 counter 256H rises once
				// at h = 256 and falls once at the wrap, so "advance V at the
				// line wrap" is that edge.
				vcnt <= last_v ? '0 : (vcnt + 1'b1);
			end else begin
				h <= h + 9'd1;
			end
		end
	end

	// 1V..128V are the low eight bits of the line index and 256V is the ninth —
	// which on this board is pin 37, a bare stub, so lines 256..V_TOTAL-1 alias
	// onto v = 0..(V_TOTAL-257) as far as every consumer of 1V..128V is
	// concerned.  That is unobservable: every one of those lines is inside
	// VBLANK, the playfield row address only matters while `vde`, and the IRQ1
	// gate is qualified by /VBLANK (sheet 2).
	assign v     = vcnt[7:0];
	assign v256  = vcnt[8];
	assign vline = vcnt;

	// =====================================================================
	// 3. Delayed horizontal taps — 4HD1H (pin 2) and 4HD2H (pin 19)
	// =====================================================================
	// 4H rises at the 3->4 boundary, so 4HD1H rises at 4->5 and 4HD2H at 5->6.
	// Those are exactly the edges that move the motion-object link along
	// (13E/7D at 4HD1H, 7E/4E at 4HD2H).
	always_ff @(posedge clk) begin
		if (reset) begin
			h_4hd1h <= 1'b0;
			h_4hd2h <= 1'b0;
		end else if (ce_7m) begin
			h_4hd1h <= h[2];
			h_4hd2h <= h_4hd1h;
		end
	end
	assign h_4hd2h_n = ~h_4hd2h;

	// =====================================================================
	// 4. Blanking, sync and the frame/line resets
	// =====================================================================
	// All of these are combinational decodes of the counters.  On the board
	// they come out of the custom already registered; taking them
	// combinationally here keeps every edge exactly on its count boundary.
	wire hb = (h < HBE) | (h >= HBS);
	wire vb = (vcnt >= VBS);

	assign hblank   = hb;
	assign vblank   = vb;
	assign vblank_n = ~vb;
	assign blank_n  = ~(hb | vb);                       // 11J LS02

	assign hsync_n  = ~((h >= HSS) & (h < HSE));
	assign vsync_n  = ~((vcnt >= VSS) & (vcnt < VSE));
	// 14D LS00 NANDs /HSYNC and /VSYNC (= HSYNC | VSYNC) and 15A U06 inverts
	// it, so VID-9 composite sync is active low.
	assign csync_n  = hsync_n & vsync_n;

	// /VRES — pin 38, the frame reset the VMATCH custom (8K pin 28) shares with
	// the SOS-2's own vertical counter.  Its position is inside 3F; low over the
	// last line of the frame is the only placement consistent with "VMATCH keeps
	// its own copy of the line counter, in step with 1V..128V".
	assign vres_n = ~last_v;

	// /LINKRES — pin 25.  It must clear 13E/7D after the last 4HD1H of the line
	// and before the first motion-object slot of the next line, so it belongs
	// somewhere in the final 8-count group; the exact count is inside 3F.  The
	// whole last group is the safe choice and costs nothing — the link that
	// group would have latched is discarded on the board too.
	assign linkres_n = ~(h[8:3] == LASTG);

	// /MOL, /MOR — pins 27 and 26.  Xybots' equivalents are /MOHR at h = 6 once
	// per line and /MOHLD once per group at h == 7.  Here /MOL is derived (h%8
	// == 5, the count where MOHPOS is on VD) and /MOR is pinned at h = 5 by the
	// sheet-9 edge clear rather than by the family analogy — see MOR_H above.
	// Together they set the constant horizontal offset of the motion-object
	// layer, which blstroid_mo's MO_H_OFFSET tracks; both are parameters so that
	// calibration is a one-line change here.
	assign mor_n = ~(h == MORH);
	assign mol_n = ~(h[2:0] == MOLH3);

	// =====================================================================
	// 5. Sheet-4 glue: the taps every other sheet consumes
	// =====================================================================
	assign h1_n   = ~h[0];
	assign h2_n   = ~h[1];
	assign h4_n   = ~h[2];
	assign h256   =  h[8];
	assign h256_n = ~h[8];                       // 4D LS86 (4,5->6) -> VCLOCK
	assign v1_n   = ~v[0];

	// The video-RAM slot schedule.  SELCPU = ~(/4H & 1H) = 4H | /1H (14D LS00,
	// sheet 5); RAMFREE = ~((1H & /4H) | /2H) = 2H & SELCPU (11J LS02 + 14B
	// LS08, sheet 2):
	//
	//    H[2:0]      0    1    2    3    4    5    6    7
	//    owner       MO   MO   CPU  PF   MO   MO   CPU  CPU
	//    SELCPU      1    0    1    0    1    1    1    1
	//    RAMFREE     0    0    1    0    0    0    1    1
	//    {2H,SELCPU} 01   00   11   10   01   01   11   11
	assign selcpu   = h[2] | ~h[0];
	assign ramfree  = h[1] & selcpu;
	assign ramcpu_n = ~ramfree;
	assign vram_sel = {h[1], selcpu};

	// 12E F10 -> VMATCH INCLK: low ONLY at H[2:0] = 1, so its falling edge sits
	// where VD holds MOVPOS and its rising edge where VD holds MOPIC — one pin
	// captures both words.
	assign inclk = ~(h[0] & ~h[1] & ~h[2]);

	// 12F F00 -> the SOS-1 shift control.
	assign n7m1h = ~(m7 & h[0]);

	// =====================================================================
	// 6. Display coordinates and markers
	// =====================================================================
	assign hde  = ~hb;
	assign vde  = ~vb;
	assign de   = hde & vde;
	assign hpos = {(h - HBE), pix};              // 2*(h-HB_END) + pix
	assign vpos = vcnt;

	assign line_start   = ce_7m & (h == 9'd0);
	assign frame_start  = line_start & (vcnt == '0);
	assign vblank_start = line_start & (vcnt == VBS);

	// HBLANK is combinational in `h`, so its 0->1 edge is the clk_sys edge that
	// ends count HBS-1 and its 1->0 edge the one that ends count HBE-1.
	assign hblank_rise  = ce_7m & (h == (HBS - 9'd1));
	assign hblank_fall  = ce_7m & (h == (HBE - 9'd1));

endmodule
