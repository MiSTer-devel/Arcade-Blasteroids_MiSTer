`timescale 1ns/1ps
//============================================================================
//  Blasteroids bring-up test pattern — a self-contained raster generator plus
//  colour bars, status strips and a bouncing box.
//
//  Copyright (C) 2026 RetroShrimp.  GPL-3.0-or-later; see LICENSE and
//  NOTICE.md.  The pattern is adapted from the Xybots MiSTer core's
//  rtl/video/xybots_testpat.sv (GPL-3.0, same author); the raster generator
//  here is new, because this module deliberately does not depend on the game's
//  own sync generator.
//
//  Nothing in this file models Blasteroids hardware, and nothing in the game
//  uses it.  `blstroid_core` instantiates it only under
//  `BLSTROID_BRINGUP_DEBUG`, which no release build defines: it is there so
//  that a board coming up black can be diagnosed without the game logic.  It
//  shows, in one picture, that
//    * the PLL locks and clk_sys / ce_14m really toggle (the box moves),
//    * a 912 x 262 / 640 x 240 raster locks a 15 kHz monitor and MiSTer's
//      scaler (the white frame sits exactly on the visible edge),
//    * the RGB path through arcade_video is wired right (bars in SMPTE order,
//      the grey ramp monotonic left to right),
//    * the memories are alive — `probe` is a live read-back from the program
//      ROM, the sound ROM and the graphics fetch, XORed per frame and
//      displayed, so a stuck SDRAM phase or an unwritten BRAM shows as a
//      frozen or wrong strip,
//    * the controls reach the core — `dials` and `btns` are displayed as bit
//      strips, so the Whirly-Gig mapping (spinner / paddle / mouse), the panel
//      buttons, the coin inputs and the Service switch can be checked with no
//      CPU running.
//
//  Three 16-bit strips, each 16 lines tall, 16 cells of 40 px, MSB left:
//    y 100..115  GREEN   btns   — panel buttons, coins, Service
//    y 128..143  YELLOW  dials  — the Whirly-Gig CLK/DIR pairs
//    y 156..171  WHITE   probe  — the per-frame memory read-back
//  The caller chooses what goes in each word; see blstroid_core.sv.
//
//  ---- the raster ----------------------------------------------------------
//  912 x 262 total, 640 x 240 visible, pixel clock 14.318181 MHz
//  => 15.700 kHz line, 59.923 Hz frame — the same totals the game raster uses.
//  The blanking split below (front porch, sync width, back porch, and the VSYNC
//  line) is this module's own and is not taken from the SP-316 sheets: it is
//  Xybots' measured SP-313 split (456 total: 32 front, 32 sync, 56 back) scaled
//  x2 to this line and rounded to a 4.75 us HSYNC.  For the board's real
//  positions see blstroid_sos2_sync.sv.
//
//    h    0..639   visible          v    0..239   visible
//       640..711   front porch         240..242   front porch
//       712..779   HSYNC (68 px)       243..246   VSYNC (4 lines)
//       780..911   back porch          247..261   back porch
//
//  ---- colour depth and pixel aspect ---------------------------------------
//  8 bits per channel out, matching the game path, at full intensity only.
//  The native raster is 640 x 240 on a 4:3 monitor, i.e. 2:1 wide pixels, so
//  the bouncing box is 64 x 32 raster pixels in order to appear square.  If it
//  looks like a tall rectangle, the aspect handling in Arcade-Blasteroids.sv is
//  wrong — which is the kind of thing this pattern exists to catch.
//============================================================================

module blstroid_testpat (
	input  logic        clk,
	input  logic        reset,
	input  logic        ce,          // ce_14m, one pulse per pixel

	// live read-back from the core's memories; displayed as a bit strip so that
	// they cannot be synthesised away (see the header)
	input  logic [15:0] probe,
	// control words, displayed as bit strips (see the header)
	input  logic [15:0] dials,
	input  logic [15:0] btns,

	// raster taps, for whatever the caller wants to align to the pattern
	output logic  [9:0] h,           // 0..911
	output logic  [8:0] v,           // 0..261
	output logic        frame_start, // ce & first pixel of line 0

	output logic        hsync,       // active high
	output logic        vsync,       // active high
	output logic        hblank,      // active high
	output logic        vblank,      // active high

	output logic  [7:0] r,
	output logic  [7:0] g,
	output logic  [7:0] b
);

	// ---- raster constants (this module's own; see the header) ------------
	localparam logic  [9:0] H_TOTAL  = 10'd912;
	localparam logic  [9:0] H_ACTIVE = 10'd640;
	localparam logic  [9:0] HS_BEG   = 10'd712;
	localparam logic  [9:0] HS_END   = 10'd780;   // exclusive
	localparam logic  [8:0] V_TOTAL  =  9'd262;
	localparam logic  [8:0] V_ACTIVE =  9'd240;
	localparam logic  [8:0] VS_BEG   =  9'd243;
	localparam logic  [8:0] VS_END   =  9'd247;   // exclusive

	// ---- counters --------------------------------------------------------
	wire line_end  = (h == H_TOTAL - 10'd1);
	wire frame_end = line_end && (v == V_TOTAL - 9'd1);

	always_ff @(posedge clk) begin
		if (reset) begin
			h <= 10'd0;
			v <=  9'd0;
		end else if (ce) begin
			if (line_end) begin
				h <= 10'd0;
				v <= frame_end ? 9'd0 : (v + 9'd1);
			end else begin
				h <= h + 10'd1;
			end
		end
	end

	assign frame_start = ce && (h == 10'd0) && (v == 9'd0);

	always_ff @(posedge clk) begin
		if (reset) begin
			hblank <= 1'b1;
			vblank <= 1'b1;
			hsync  <= 1'b0;
			vsync  <= 1'b0;
		end else if (ce) begin
			hblank <= (h >= H_ACTIVE - 10'd1) && !line_end;
			hsync  <= (h >= HS_BEG   - 10'd1) && (h < HS_END - 10'd1);
			// V flags change at the start of the line, so they are evaluated
			// against the line the counter is ABOUT to become.
			vblank <= line_end ? ((frame_end ? 9'd0 : v + 9'd1) >= V_ACTIVE)
			                   : (v >= V_ACTIVE);
			vsync  <= line_end ? (((frame_end ? 9'd0 : v + 9'd1) >= VS_BEG) &&
			                      ((frame_end ? 9'd0 : v + 9'd1) <  VS_END))
			                   : ((v >= VS_BEG) && (v < VS_END));
		end
	end

	// ---- pattern geometry ------------------------------------------------
	localparam logic [9:0] BOX_W    = 10'd64;   // 2:1 pixels -> square on screen
	localparam logic [8:0] BOX_H    =  9'd32;
	localparam logic [9:0] BOX_XMAX = H_ACTIVE - BOX_W;    // 576
	localparam logic [8:0] BOX_YMAX = V_ACTIVE - BOX_H;    // 208

	// 640 = 8 bars x 80 px = 16 ramp steps x 40 px = 16 probe cells x 40 px.
	localparam logic [6:0] BAR_W  = 7'd80;
	localparam logic [5:0] STEP_W = 6'd40;

	localparam logic [8:0] RAMP_Y   = 9'd180;   // grey ramp below this line
	localparam logic [8:0] BTNS_Y0  = 9'd100;   // buttons / coins / service strip
	localparam logic [8:0] DIALS_Y0 = 9'd128;   // whirly-gig strip
	localparam logic [8:0] PROBE_Y0 = 9'd156;   // memory read-back strip
	localparam logic [8:0] STRIP_H  = 9'd16;

	// ---- bouncing box, one step per frame (~59.92 Hz) --------------------
	logic [9:0] box_x;
	logic [8:0] box_y;
	logic       box_dx, box_dy;   // 1 = moving in the increasing direction

	always_ff @(posedge clk) begin
		if (reset) begin
			box_x  <= 10'd80;
			box_y  <=  9'd40;
			box_dx <= 1'b1;
			box_dy <= 1'b1;
		end else if (frame_start) begin
			if (box_dx) begin
				if (box_x >= BOX_XMAX - 10'd4) begin box_x <= BOX_XMAX;  box_dx <= 1'b0; end
				else                                 box_x <= box_x + 10'd4;
			end else begin
				if (box_x <= 10'd4)            begin box_x <= 10'd0;     box_dx <= 1'b1; end
				else                                 box_x <= box_x - 10'd4;
			end
			if (box_dy) begin
				if (box_y >= BOX_YMAX - 9'd1)  begin box_y <= BOX_YMAX;  box_dy <= 1'b0; end
				else                                 box_y <= box_y + 9'd1;
			end else begin
				if (box_y == 9'd0)             begin                     box_dy <= 1'b1; end
				else                                 box_y <= box_y - 9'd1;
			end
		end
	end

	// ---- per-line bar / ramp / probe-cell indices ------------------------
	// Counters instead of x/80 and x/40 so no divider is inferred.  They are
	// cleared throughout HBLANK, so at h == k they read exactly k.
	logic [6:0] bar_cnt;
	logic [2:0] bar_idx;
	logic [5:0] step_cnt;
	logic [3:0] ramp_lvl;

	always_ff @(posedge clk) begin
		if (reset) begin
			bar_cnt  <= 7'd0;
			bar_idx  <= 3'd0;
			step_cnt <= 6'd0;
			ramp_lvl <= 4'd0;
		end else if (ce) begin
			if (hblank) begin
				bar_cnt  <= 7'd0;
				bar_idx  <= 3'd0;
				step_cnt <= 6'd0;
				ramp_lvl <= 4'd0;
			end else begin
				if (bar_cnt == BAR_W - 7'd1) begin
					bar_cnt <= 7'd0;
					bar_idx <= bar_idx + 3'd1;
				end else begin
					bar_cnt <= bar_cnt + 7'd1;
				end
				if (step_cnt == STEP_W - 6'd1) begin
					step_cnt <= 6'd0;
					ramp_lvl <= ramp_lvl + 4'd1;
				end else begin
					step_cnt <= step_cnt + 6'd1;
				end
			end
		end
	end

	// ---- pattern ---------------------------------------------------------
	wire       active = ~hblank & ~vblank;
	wire [9:0] x      = h;
	wire [8:0] y      = v;

	// 2-pixel white frame exactly on the 640 x 240 visible edge.
	wire on_border = active &&
	                 ((x < 10'd4) || (x >= H_ACTIVE - 10'd4) ||
	                  (y <  9'd2) || (y >= V_ACTIVE - 9'd2));

	wire in_box = active &&
	              (x >= box_x) && (x < box_x + BOX_W) &&
	              (y >= box_y) && (y < box_y + BOX_H);

	wire in_ramp  = active && (y >= RAMP_Y);
	wire in_btns  = active && (y >= BTNS_Y0)  && (y < BTNS_Y0  + STRIP_H);
	wire in_dials = active && (y >= DIALS_Y0) && (y < DIALS_Y0 + STRIP_H);
	wire in_probe = active && (y >= PROBE_Y0) && (y < PROBE_Y0 + STRIP_H);

	// One 40 px cell per bit, MSB on the left.  `ramp_lvl` is the 0..15 cell
	// index (it counts one step per 40 px, which is what the ramp needs too).
	wire [3:0] cell_idx  = 4'd15 - ramp_lvl;
	wire       btns_bit  = btns [cell_idx];
	wire       dials_bit = dials[cell_idx];
	wire       probe_bit = probe[cell_idx];

	logic [7:0] bar_r, bar_g, bar_b;
	always_comb begin
		case (bar_idx)                                        // SMPTE order
			3'd0:    {bar_r, bar_g, bar_b} = {8'hFF, 8'hFF, 8'hFF};  // white
			3'd1:    {bar_r, bar_g, bar_b} = {8'hFF, 8'hFF, 8'h00};  // yellow
			3'd2:    {bar_r, bar_g, bar_b} = {8'h00, 8'hFF, 8'hFF};  // cyan
			3'd3:    {bar_r, bar_g, bar_b} = {8'h00, 8'hFF, 8'h00};  // green
			3'd4:    {bar_r, bar_g, bar_b} = {8'hFF, 8'h00, 8'hFF};  // magenta
			3'd5:    {bar_r, bar_g, bar_b} = {8'hFF, 8'h00, 8'h00};  // red
			3'd6:    {bar_r, bar_g, bar_b} = {8'h00, 8'h00, 8'hFF};  // blue
			default: {bar_r, bar_g, bar_b} = {8'h00, 8'h00, 8'h00};  // black
		endcase
	end

	wire [7:0] ramp8 = {ramp_lvl, ramp_lvl};

	logic [7:0] nr, ng, nb;
	always_comb begin
		if      (!active)   {nr, ng, nb} = {8'h00, 8'h00, 8'h00};
		else if (on_border) {nr, ng, nb} = {8'hFF, 8'hFF, 8'hFF};
		else if (in_box)    {nr, ng, nb} = {8'hFF, 8'h80, 8'h00};   // orange box
		else if (in_btns)   {nr, ng, nb} = btns_bit  ? {8'h00, 8'hFF, 8'h00}
		                                             : {8'h00, 8'h20, 8'h00};
		else if (in_dials)  {nr, ng, nb} = dials_bit ? {8'hFF, 8'hFF, 8'h00}
		                                             : {8'h20, 8'h20, 8'h00};
		else if (in_probe)  {nr, ng, nb} = probe_bit ? {8'hFF, 8'hFF, 8'hFF}
		                                             : {8'h00, 8'h00, 8'h40};
		else if (in_ramp)   {nr, ng, nb} = {ramp8, ramp8, ramp8};
		else                {nr, ng, nb} = {bar_r, bar_g, bar_b};
	end

	always_ff @(posedge clk) begin
		if (reset) begin
			r <= 8'h00;
			g <= 8'h00;
			b <= 8'h00;
		end else if (ce) begin
			r <= nr;
			g <= ng;
			b <= nb;
		end
	end

endmodule
