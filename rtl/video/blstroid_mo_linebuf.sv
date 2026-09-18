`timescale 1ns/1ps
//============================================================================
//  Blasteroids — SP-316 sheet 9: the two motion-object line buffers (14K and
//  14F, 2K x 8 35 ns), their F163 address counter chains (14L/15L/13L and
//  14J/15J/13J), the transparency write gate and the read-and-clear cycle.
//
//  Copyright (C) 2026 RetroShrimp.  GPL-3.0-or-later; see LICENSE and
//  NOTICE.md.  Structurally derived from the Xybots MiSTer core's
//  rtl/video/xybots_mo_linebuf.sv (GPL-3.0, same author), which models the same
//  function as an Atari "LB" custom; Blasteroids builds it out of two discrete
//  SRAMs, so this version is the discrete circuit as drawn.
//
//  ---------------------------------------------------------------------------
//  The mechanism, in one table (shown on a 1VD6H = 0 line)
//  ---------------------------------------------------------------------------
//                     | buffer 0 (14K)            | buffer 1 (14F)
//    counter          | cleared by /MOHR0, then   | reloaded with each object's
//                     | free-runs at 14M = the    | X by /MOHLD1 = the WRITE
//                     | DISPLAY address           | address
//    pixel bus D3:0   | undriven -> R65/66/69/70  | MOPIX3:0 from SOS-1
//                     | pull-ups -> 0xF           |
//    /OE              | enabled while 14M is high | disabled
//    /WE              | asserted every 14M low,   | asserted only when
//                     | UNCONDITIONALLY -> writes | MOPIX != 0xF -> only
//                     | 0xF back = transparent    | opaque pixels are stored
//    net effect       | read, then clear          | fill for the next line
//
//  and everything swaps on 1VD6H = 1.  There is no separate clear pass: each
//  location is blanked in the same pixel period in which it is displayed.
//
//  The three data buses are the crux.  The COLOUR nibble D7:4 is shared by both
//  RAMs and is driven by 12L F244 (from MOPAL3:0) only while 14M is LOW — the
//  write half — so during the read half the DISPLAY RAM drives it and 14E F174
//  latches it on the 14M falling edge as LBCOL3:0.  Each buffer has its OWN
//  pixel bus D3:0 with its own 1 K pull-ups, and 17J F398 selects between them
//  on 1VD6H, presenting the true pen on its /Q pins as LBPIX3:0.
//
//  So an untouched location reads pen 0, and pen 0 is transparent — MAME's
//  "transparent pen index = 0" exactly.  The clear writes colour 0 alongside
//  the 0xF pixel here, which the board does not (12L is driving whatever the
//  current object's MOPAL is during the write half); it is unobservable,
//  because CRA7:4 only takes LBCOL when the motion object WINS, which requires
//  LBPIX != 0, and 17E is deselected by /CS3 whenever LBPIX == 0.  Writing 0
//  makes "pen 0 <=> colour 0" an invariant.
//
//  ---------------------------------------------------------------------------
//  The counters — one per buffer, doing both jobs
//  ---------------------------------------------------------------------------
//  Each buffer has exactly ONE three-stage F163 chain, and the ping-pong is
//  done by crossing its /LD and /CLR (11L LS32):
//
//      /MOHR0  = /MOR | 1V       /MOHR1  = /1V | /MOR
//      /MOHLD1 = 1V   | /MOL     /MOHLD0 = /1V | /MOL
//
//  so on a 1V = 0 line buffer 0's counter sees only /CLR (it is the display
//  address) and buffer 1's only /LD (it is the write address), and vice versa.
//  Between strobes both free-run at 14M, which is why one object's sixteen
//  pixels land at X, X+1 ... X+15: /MOL loads X while MOHPOS is on the video
//  bus (h%8 == 5) and the counter walks the sixteen pixels that follow.
//
//  The counters are TEN bits and the RAMs' A10 is grounded, so a column
//  starting near 1023 wraps into the left edge — which is exactly
//  atarimo.cpp's `xpos &= 0x3ff` followed by its `xpos >= width ? xpos - 1024`
//  fold.
//
//  The /OE and pixel-bus selection uses 1VD6H, which is 1V retimed by 4E LS174C
//  on 4HD2H — it flips at the h%8 5->6 boundary, exactly where the 57th link's
//  sixteen pixels finish.  The COUNTER strobes use the raw 1V, one group
//  earlier, which is what lets the 57th object of the old line and the 1st of
//  the new one both land in the right buffer.
//
//  MO_H_OFFSET: the display read counter is PRESET to it rather than cleared,
//  so whatever count inside the line the SOS-2's /MOR really falls on is
//  absorbed by this one number.  It must move with blstroid_sos2_sync's MOR_H,
//  which is 5 (1004 = 1024 - 2*(16-5-1)); at MOR_H = 6 the display-side
//  unconditional clear below eats two locations per scanline and leaves a
//  black column on screen.
//============================================================================

module blstroid_mo_linebuf #(
	parameter int MO_H_OFFSET = 1004
)(
	input  logic        clk,
	input  logic        reset,
	input  logic        ce_14m,     // one pixel clock

	// ---- SOS-2 strobes (sheet 4 pins 26/27) and the retimed 1V ----
	input  logic        mol_n,      // /MOL — once per 8 counts, at h%8 == 5
	input  logic        mor_n,      // /MOR — once per line
	input  logic        v1,         // 1V, raw
	input  logic        v1d6h,      // 4E LS174C Q10

	// ---- write side ----
	input  logic  [9:0] a_in,       // VD15:6 = MOHPOS's X, straight off the bus
	input  logic  [3:0] mopix,      // SOS-1 PIX3:0 — the COMPLEMENT of the pen
	input  logic  [3:0] mopal,      // SOS-1 L3:0   — the object's palette

	// ---- read side ----
	output logic  [3:0] lb_pix,     // 17J F398 /Q — the TRUE pen, 0 = transparent
	output logic  [3:0] lb_col,     // 14E F174    — the true palette
	output logic  [9:0] mo_hpos     // the address lb_pix / lb_col belong to
);

	localparam logic [7:0] LB_EMPTY = 8'h0F;      // colour 0, MOPIX = 0xF = pen 0
	localparam logic [9:0] H_OFF    = MO_H_OFFSET[9:0];

	// =====================================================================
	// 1. 11L LS32 — the crossed ping-pong strobes, and 16J F21
	// =====================================================================
	wire mohr0_n  = mor_n |  v1;
	wire mohr1_n  = mor_n | ~v1;
	wire mohld0_n = mol_n | ~v1;
	wire mohld1_n = mol_n |  v1;

	wire opaque   = ~(&mopix);                    // 16J F21: 0xF is transparent

	// 1VD6H picks the buffer that is READ; the other one is being filled.
	wire rd0 = ~v1d6h;
	wire rd1 =  v1d6h;

	// =====================================================================
	// 2. The two F163 counter chains
	// =====================================================================
	logic [9:0] c0, c1;

	wire [9:0] c0_nxt = !mohr0_n  ? H_OFF :
	                    !mohld0_n ? a_in  : (c0 + 10'd1);
	wire [9:0] c1_nxt = !mohr1_n  ? H_OFF :
	                    !mohld1_n ? a_in  : (c1 + 10'd1);

	always_ff @(posedge clk) begin
		if (reset) begin
			c0 <= '0;
			c1 <= '0;
		end else if (ce_14m) begin
			c0 <= c0_nxt;
			c1 <= c1_nxt;
		end
	end

	// =====================================================================
	// 3. The two 2K x 8 SRAMs, used as 1K x 8
	// =====================================================================
	// One write port and one read port each, so Quartus infers a simple
	// dual-port M10K.  The read port is addressed with the counter's NEXT state
	// so that the registered output reproduces the SRAM's ASYNCHRONOUS read, and
	// the clear then writes the location being displayed on the very pixel it is
	// displayed — which is what /OE-high-then-/WE-low does on the board.  On the
	// buffer being DISPLAYED the two addresses differ by exactly one, so there
	// is no read-during-write; on the buffer being FILLED the read output is
	// discarded by the 17J F398 anyway.
	logic [7:0] lb0 [0:1023];
	logic [7:0] lb1 [0:1023];

	wire [7:0] di = {mopal, mopix};

	logic [7:0] q0, q1;

	// The write address is the buffer's own counter in BOTH roles — the clear
	// writes the location just displayed, the fill writes the object's pixel.
	wire       we0 = rd0 ? 1'b1     : opaque;
	wire [7:0] dw0 = rd0 ? LB_EMPTY : di;

	wire       we1 = rd1 ? 1'b1     : opaque;
	wire [7:0] dw1 = rd1 ? LB_EMPTY : di;

`ifndef ALTERA_RESERVED_QIS
	initial begin
		for (int i = 0; i < 1024; i++) begin
			lb0[i] = LB_EMPTY;
			lb1[i] = LB_EMPTY;
		end
	end
`endif

	always_ff @(posedge clk) begin
		if (ce_14m) begin
			if (we0) lb0[c0] <= dw0;
			q0 <= lb0[c0_nxt];
		end
	end

	always_ff @(posedge clk) begin
		if (ce_14m) begin
			if (we1) lb1[c1] <= dw1;
			q1 <= lb1[c1_nxt];
		end
	end

	// =====================================================================
	// 4. 17J F398 (pixel, /Q) and 14E F174 (colour)
	// =====================================================================
	wire [7:0] dout = rd1 ? q1 : q0;

	assign lb_pix  = ~dout[3:0];
	assign lb_col  =  dout[7:4];
	assign mo_hpos = rd1 ? c1 : c0;

endmodule
