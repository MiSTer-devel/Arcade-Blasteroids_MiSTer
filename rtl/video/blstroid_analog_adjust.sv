// Blasteroids analog-output alignment stage: the OSD decode, the read-rate
// generator and the blanking glue around rmonic79's `crt_adjust`
// (rtl/video/crt_adjust.sv, vendored from MiSTer-CRT-Adjust).  Written from
// that project's examples/core_side_snippet.v (GPL-3.0, Umberto Parisi),
// re-derived for a 4-clock pixel and for feeding arcade_video rather than the
// VGA_* pins directly.
//
// Copyright (C) 2026 RetroShrimp.  GPL-3.0-or-later; see LICENSE and NOTICE.md.
//
// The OSD carries three controls plus an On/Off: CRT H-Size, CRT H-Position and
// CRT V-Shift.  H-Position has two possible mechanisms inside `crt_adjust`,
// picked at compile time by `HPOS_MODE`:
//
//   HPOS_CONTENTSHIFT (1)  shifts the CONTENT in the line buffer, HSync native
//   HPOS_SYNCSHIFT    (0)  shifts the HSYNC, content native
//
// Only one of the two may be built, and the whole engine restarts on one line
// reference, `hs_ref_out`.  The read-rate accumulator below must be reset on
// that edge rather than on the raw HSync, or the picture trembles.
//
// ---------------- HPOS_MODE for Blasteroids: CONTENTSHIFT -------------------
// Upstream's rule is "CONTENTSHIFT for wide/centred games, SYNCSHIFT for narrow
// side-anchored ones, because CONTENTSHIFT runs the picture out of the buffer
// window on the short-margin side and shows a black block there".  Our numbers
// (blstroid_sos2_sync, in PIXELS -- its parameters are counts of 7M, i.e. half
// a pixel each):
//
//     line total        912 px
//     active            px 32 .. 671          (640 px)
//     HSync high        px 754 .. 817
//
// The line buffer's write pointer resets on the HSync rise, so in BUFFER
// coordinates the active region is 190 .. 830: a 190-pixel lead-in and an
// 82-pixel run-out.  The short margin is therefore the RIGHT one, 82 px, and
// the H-Position range this core offers is +-64 -- so the worst case is
// 830 + 64 = 894 read ticks, still inside the ~912 a line delivers at H-Size 0.
// CONTENTSHIFT is safe here and it is upstream's recommended default.
//
// It is also the more USEFUL of the two here.  Enlarging (H-Size > 0) slows the
// read, so the 190-pixel lead-in is stretched too and the right of the picture
// runs off the end of the line; a NEGATIVE H-Position pulls the content earlier
// and buys that room back.  SYNCSHIFT cannot do that -- it moves the sync and
// the content together, so it re-centres the picture on the tube but never
// changes what fits in a line.
//
// If a real CRT ever shows a black block at the right edge, flip HPOS_MODE to 0
// here and nothing else changes.
//
// ---------------- the read rate, and why 64/CLK_PER_PIX ---------------------
// Upstream steps the read period in quarter clocks because its reference core's
// pixel is 16 clk_sys cycles: 64 quarters = one pixel, so one H-Size click is
// 1/64 = 1.56%.  Ours is 4 (ce_14m = clk_sys/4, the 14.318181 MHz pixel clock),
// and a fixed quarter-clock step would make one click 6.25% instead.  The
// accumulator below counts 1/CLK_PER_PIX-th of a clock instead, which keeps the
// 1.56% step at any pixel period:
//
//     period = 64 + hsize      units, where 64 units == CLK_PER_PIX clocks
//     acc   += 64/CLK_PER_PIX  per clock
//
// so hsize spans 48..79 units = 3.00..4.94 clocks per pixel at CLK_PER_PIX = 4.
// ACC_STEP must stay an exact integer -- 64/CLK_PER_PIX with CLK_PER_PIX a
// power of two -- or the OSD step silently drifts away from the menu text.
//
// ---------------- where this sits, and the DE it hands on -------------------
// Between blstroid_core and arcade_video, so the mixer, the scandoubler, gamma
// and video_freak all see the adjusted raster and the adjusted pixel enable.
// Nothing here touches the core's own timing: the pixel clock and the native
// sync rates are untouched, so there is no jitter and nothing the game can see.
//
// Upstream's core-side glue also builds a `de_osd` window whose rising edge is
// pinned to the NATIVE active region, so the MiSTer OSD (which centres itself on
// the rise of VGA_DE) does not slide when H-Position moves the picture.  This
// core deliberately does not use it -- see the DE section below for why it does
// not survive this raster.
//
// All three OSD fields are plain two's complement, so the config string just
// lists 0,+1..+max then -min..-1 and the decode is a $signed() -- no bespoke
// mapping to get out of step with the menu text.

module blstroid_analog_adjust #(
	parameter int H_TOTAL     = 912,   // ce_pix ticks per line (blstroid_sos2_sync: 912)
	parameter int V_TOTAL     = 262,   // lines per frame       (blstroid_sos2_sync: 262)
	parameter int CLK_PER_PIX = 4,     // clk_sys cycles per ce_pix (ce_14m = clk_sys/4)
	parameter int HPOS_MODE   = 1,     // 1 = CONTENTSHIFT, 0 = SYNCSHIFT (see above)
	parameter int AW          = 11     // line-buffer address bits; 912 px needs > 512/bank
) (
	input  logic       clk,
	input  logic       ce_pix,

	// OSD
	input  logic       osd_on,         // CRT Adjust Off/On; Off is a bit-exact bypass
	input  logic [4:0] osd_hsize,      // CRT H-Size        +15 / -16   two's complement
	input  logic [6:0] osd_hpos,       // CRT H-Position    +63 / -64   two's complement
	input  logic [5:0] osd_vshift,     // CRT V-Shift       +31 / -32   two's complement

	input  logic [7:0] r_in, g_in, b_in,
	input  logic       hs_in, vs_in, hb_in, vb_in,

	output logic [7:0] r_out, g_out, b_out,
	output logic       hs_out, vs_out, hb_out, vb_out,
	output logic       ce_out
);

	localparam [7:0] ACC_STEP = 8'(64 / CLK_PER_PIX);

	// ---------------- OSD decode ----------------
	// Everything is sampled at ce_pix so a mid-frame menu change lands on a
	// pixel boundary rather than anywhere inside one.
	logic              active;
	logic signed [4:0] hsize_s;
	logic signed [8:0] hpos_s;
	logic signed [5:0] vshift_s;

	always_ff @(posedge clk) if (ce_pix) begin
		active   <= osd_on;
		hsize_s  <= $signed(osd_hsize);
		hpos_s   <= 9'($signed(osd_hpos));    // sign-extend 7 -> 9 bits
		vshift_s <= $signed(osd_vshift);
	end

	// ---------------- read-rate generator ----------------
	// The wiring rule: reset the accumulator on the rise of the module's
	// `hs_ref_out`, never on the raw HSync, so the write side, the module's read
	// counter and this read rate all restart on the same edge.
	wire  hs_ref;
	logic hs_ref_d;
	always_ff @(posedge clk) hs_ref_d <= hs_ref;
	wire  hs_ref_rise = hs_ref & ~hs_ref_d;

	wire  [7:0] rd_period = 8'd64 + {{3{hsize_s[4]}}, hsize_s};   // 48..79
	logic [7:0] rd_acc;
	wire        rd_tick = (rd_acc + ACC_STEP) >= rd_period;

	always_ff @(posedge clk) begin
		if      (hs_ref_rise) rd_acc <= 8'd0;
		else if (rd_tick)     rd_acc <= rd_acc + ACC_STEP - rd_period;
		else                  rd_acc <= rd_acc + ACC_STEP;
	end

	// When On the line buffer is always live -- even at H-Size 0, where rd_tick
	// runs at the same average rate as ce_pix but re-phased onto hs_ref.
	wire rd_ce = active ? rd_tick : ce_pix;

	// ---------------- the engine ----------------
	wire [7:0] str_r, str_g, str_b;
	wire       str_hs, str_vs, str_hb, str_vb;

	crt_adjust #(
		.VTOTAL    (V_TOTAL),
		.HTOTAL    (H_TOTAL),
		.HPOS_MODE (HPOS_MODE),
		.AW        (AW)
	) u_crt_adjust (
		.clk        (clk),
		.pxl_cen    (ce_pix),
		.pxl2_cen   (rd_ce),
		.active     (active),
		.hsize      (hsize_s),      // kept for upstream parity; the rate above is H-Size
		.hoffset    (hpos_s),
		.voffset    (vshift_s),
		.r_in       (r_in), .g_in (g_in), .b_in (b_in),
		.hs_in      (hs_in),        // NATIVE sync in -- the module derives hs_ref itself
		.vs_in      (vs_in),
		.hb_in      (hb_in | vb_in),
		.vb_in      (vb_in),        // TRUE vertical blank, not the combined blank
		.r_out      (str_r), .g_out (str_g), .b_out (str_b),
		.hs_out     (str_hs), .vs_out (str_vs),
		.hb_out     (str_hb), .vb_out (str_vb),
		.hs_ref_out (hs_ref)
	);

	// ---------------- the vertical blank handed to arcade_video ------------
	// The read side emits the PREVIOUS line, so crt_adjust gates `pass_q` with a
	// vblank that is one line late (`vb_active` inside it).  `str_vb` is not
	// that signal -- it is the native, write-rate blank -- so handing `str_vb`
	// to arcade_video would blank the last visible line while the read side is
	// still emitting it.  `vb_read` below is a bit-exact replica of the module's
	// own two-flop chain, sampled on the same `hs_ref` edge, so DE goes low over
	// exactly the lines `pass_q` is already suppressing and no line is lost.
	logic hs_ref_q;
	always_ff @(posedge clk) if (ce_pix) hs_ref_q <= hs_ref;
	wire  ref_line_tick = ce_pix & hs_ref & ~hs_ref_q;

	logic vb_line, vb_read;
	always_ff @(posedge clk) if (ref_line_tick) begin
		vb_line <= vb_in;
		vb_read <= vb_line;
	end

	// ---------------- what is NOT here: upstream's `de_osd` ----------------
	// Upstream's core-side snippet hands the framework a DE window whose rising
	// edge is anchored to the native active region and whose falling edge
	// follows the stretched one, so the MiSTer OSD -- which centres itself on
	// the rise of VGA_DE -- stays put while H-Position moves the picture.  It is
	// a real improvement and it does not work on this raster.
	//
	// It assumes the stretched active never STARTS before the native one.  Our
	// active region sits 190 pixels into the line buffer (see the HPOS_MODE
	// discussion above), and pulling the content left is exactly what a negative
	// H-Position is for here -- it is the direction that buys back room when
	// H-Size enlarges.  The moment `hoffset` goes negative the content opens
	// before the window does, and everything to the left of the native active
	// start is DE-blanked: the picture loses its left edge, and if the content
	// finishes before the native rise the window opens on nothing and stays open
	// across HSync.  That is measured, not assumed: with the anchored window in
	// place, a simulated frame at H-Position = -6 put the content 61 read ticks
	// past the window's opening -- i.e. into the NEXT line's window, the shifted
	// one having been blanked away entirely.
	//
	// So DE here simply follows the picture.  The cost is the one upstream set
	// out to remove: the OSD slides with H-Position.  Anyone re-attempting the
	// anchored window has to make its rising edge the EARLIER of the native and
	// the stretched active, and suppress the native rise on a line whose
	// stretched active has already ended -- do not just copy the snippet back in.

	// ---------------- outputs ----------------
	// Off: `crt_adjust` is in its own pure bypass, so everything here is the
	// native stream delayed by the module's two input registers.
	assign r_out  = str_r;
	assign g_out  = str_g;
	assign b_out  = str_b;
	assign hs_out = str_hs;
	assign vs_out = str_vs;
	assign hb_out = str_hb;
	assign vb_out = active ? vb_read : str_vb;
	assign ce_out = rd_ce;

endmodule
