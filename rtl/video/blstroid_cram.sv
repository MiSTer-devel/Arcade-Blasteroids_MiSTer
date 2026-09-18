`timescale 1ns/1ps
//============================================================================
//  Blasteroids — SP-316 sheet 10: the PF/MO decision register (14C F74), the
//  two colour-address drivers (17D F374 for the motion object, 16D F374A for
//  the playfield), the CRA8 multiplexer, the 512 x 16 colour RAM (15/16B +
//  15/16C) with its 68000 port, and the two HC273 output registers whose /CLR
//  is /BLANK.
//
//  Copyright (C) 2026 RetroShrimp.  GPL-3.0-or-later; see LICENSE and
//  NOTICE.md.  Module shape from the Xybots MiSTer core's
//  rtl/video/xybots_cram.sv (GPL-3.0, same author); Xybots' palette is
//  IRGB 4:4:4:4 at 1 K words and Blasteroids' is xRGB 5:5:5 at 512, so nothing
//  else is shared.
//
//  ---- the colour-RAM address --------------------------------------------
//      video, MO wins : CRA8 = 0, CRA7:0 = { LBCOL3:0, LBPIX3:0 }  0x000..0x0FF
//      video, PF wins : CRA8 = 1, CRA7 = 0, CRA6:4 = PFPAL2:0,
//                                 CRA3:0 = PFPIX3:0               0x100..0x17F
//      CPU            : { BA9, BA8..BA1 }                         0x000..0x1FF
//
//  which is sheet 12's `XX0P PPPC CCC0` (MO) and `XX10 PPPC CCC0` (PF).  Only
//  0x100-0x17F is reachable as a playfield colour — 8 palettes x 16 pens;
//  0x180-0x1FF is CPU-addressable but never displayed.  MAME's
//  `GFXDECODE_SCALE("tiles", 0, ..., 256, 16, 2, 1)` uses the same base 256
//  with 8 of the 16 groups.
//
//  ---- the pipeline is two pixel clocks long -----------------------------
//    pixel P    LBPIX/LBCOL (17J F398, 14E F174) and PFPIX/PFPAL are valid;
//               17E is looked up combinationally (blstroid_prio)
//    P -> P+1   14C F74 registers the decision on the /14M edge and 17D/16D
//               register CRA7:0 on the same edge   ................. stage 1
//    P+1        the colour RAM is asynchronous on the board (45 ns) and a
//               registered BRAM here, so CRD15:0 settles inside the pixel
//    P+1 -> P+2 16/17C and 16/17B HC273 latch the colour word ...... stage 2
//
//  so the DAC code for layer pixel P appears during pixel P+2.  Every consumer
//  must delay its own timing by the same two pixels; `blstroid_video.sv` does
//  that for HSYNC/VSYNC/blanking/DE with one shift register and names the depth
//  `VID_LATENCY` so nothing has to rediscover it.
//
//  ---- blanking clears the latches, it does not gate the DAC -------------
//  `/CLR` (pin 1) of both HC273s is `/BLANK` = ~(HBLANK | VBLANK), so during
//  blanking the DAC code is literally 0 and the sheet-11 code-0 pedestal pulls
//  all three outputs to the blanking level.  There is no analogue blanking
//  switch.  `blank_n` is delayed by one pixel here so that it clears the latch
//  in step with the data that entered the pipeline with it; taking the board's
//  raw wire instead would shift the picture two pixels inside its own blanking
//  window.
//
//  ---- the CPU port, and where the array lives ---------------------------
//  512 words at FFA000-FFA3FE, read/write, `cpu_a` = BA9..BA1.  On the board
//  the 68000 side is time-shared with the video side by 14C F74 (lower half);
//  in the FPGA it is a true dual-port BRAM, so a CPU cycle never disturbs a
//  video read and the board's "HC273 clock suppressed during a CPU colour-RAM
//  cycle" gate has nothing left to protect against.  MAME behaves the same way.
//
//  `blstroid_main.sv` also carries a 512 x 16 colour RAM, next to the 68000
//  write strobes, exported as `cram_vaddr` / `cram_vdata` (registered, one
//  clk_sys).  Set `INTERNAL_RAM = 0` to use that copy; leave it at 1 and this
//  module owns the array.  Never enable both.
//============================================================================

module blstroid_cram #(
	parameter bit INTERNAL_RAM = 1
)(
	input  logic        clk,
	input  logic        reset,
	input  logic        ce_14m,

	// ---- 68000 port (FFA000-FFA3FE); INTERNAL_RAM only ----
	// With INTERNAL_RAM = 0 these four are deliberately unused: blstroid_main
	// owns the array and its 68000 port, and `cpu_dout` reads back as 0 here.
	/* verilator lint_off UNUSEDSIGNAL */
	input  logic  [8:0] cpu_a,        // BA9..BA1
	input  logic [15:0] cpu_din,
	input  logic        cpu_we_hi,
	input  logic        cpu_we_lo,
	/* verilator lint_on UNUSEDSIGNAL */
	output logic [15:0] cpu_dout,

	// ---- layer pixel P ----
	input  logic  [3:0] pf_pix,       // PFPIX3:0
	input  logic  [2:0] pf_pal,       // PFPAL2:0
	input  logic  [3:0] lb_pix,       // LBPIX3:0, TRUE pen
	input  logic  [3:0] lb_col,       // LBCOL3:0
	input  logic        pf_win,       // blstroid_prio, 17E D0 through R57
	input  logic        blank_n,      // /BLANK for pixel P

	// ---- external colour-RAM video port; INTERNAL_RAM = 0 only ----
	output logic  [8:0] cram_vaddr,   // -> blstroid_main cram_vaddr
	/* verilator lint_off UNUSEDSIGNAL */   // unused when INTERNAL_RAM = 1
	input  logic [15:0] cram_vdata,   // <- blstroid_main cram_vdata
	/* verilator lint_on UNUSEDSIGNAL */

	// ---- to the sheet-11 DACs, valid at pixel P+2 ----
	output logic  [8:0] cra,          // the registered CRA8:0 (stage 1)
	output logic        mo_wins,      // 14C F74 /Q — informational
	output logic  [4:0] r5,           // 16/17C  R4..R0
	output logic  [4:0] g5,           // 16/17C + 16/17B  G4..G0
	output logic  [4:0] b5            // 16/17B  B4..B0
);

	// =====================================================================
	// 1. Stage 1 — 14C F74 (the decision) + 17D / 16D F374 (the address)
	// =====================================================================
	// 17D drives CRA7:0 when 14C Q = 0 (the motion object wins) and 16D when
	// Q = 1; CRA8 is the two-NAND multiplexer 14/15C F00, whose video term IS
	// that same Q.  One register stage, one enable.
	wire [8:0] cra_mo = {1'b0, lb_col, lb_pix};             // 0x000 .. 0x0FF
	wire [8:0] cra_pf = {2'b10, pf_pal, pf_pix};            // 0x100 .. 0x17F

	logic blank_n_d;

	always_ff @(posedge clk) begin
		if (reset) begin
			cra       <= 9'h100;
			mo_wins   <= 1'b0;
			blank_n_d <= 1'b0;
		end else if (ce_14m) begin
			cra       <= pf_win ? cra_pf : cra_mo;
			mo_wins   <= ~pf_win;
			blank_n_d <= blank_n;
		end
	end

	// =====================================================================
	// 2. The colour RAM
	// =====================================================================
	assign cram_vaddr = cra;

	/* verilator lint_off UNUSEDSIGNAL */   // D15 is the "x" of xRGB 5:5:5
	logic [15:0] mem_dout;
	/* verilator lint_on UNUSEDSIGNAL */

	generate
		if (INTERNAL_RAM) begin : g_cram
			// 512 x 16 with independent byte lanes, one clk_sys registered read
			// on each port — the same shape as blstroid_dpram_be, inlined so
			// that this module stands on its own.
			logic [7:0] cmem_hi [0:511];
			logic [7:0] cmem_lo [0:511];
			always_ff @(posedge clk) begin
				if (cpu_we_hi) cmem_hi[cpu_a] <= cpu_din[15:8];
				if (cpu_we_lo) cmem_lo[cpu_a] <= cpu_din[7:0];
				cpu_dout <= {cmem_hi[cpu_a], cmem_lo[cpu_a]};
				mem_dout <= {cmem_hi[cra],   cmem_lo[cra]};
			end
		end else begin : g_extcram
			assign mem_dout = cram_vdata;
			always_ff @(posedge clk) cpu_dout <= 16'd0;
		end
	endgenerate

	// =====================================================================
	// 3. Stage 2 — 16/17C and 16/17B HC273, /CLR = /BLANK
	// =====================================================================
	// The colour word is xRGB 5:5:5 (sheet 12; MAME's xRGB_555):
	//   D15 x | D14:10 R4:R0 | D9:5 G4:G0 | D4:0 B4:B0
	always_ff @(posedge clk) begin
		if (reset) begin
			r5 <= '0; g5 <= '0; b5 <= '0;
		end else if (ce_14m) begin
			if (!blank_n_d) begin                      // /CLR wins over the clock
				r5 <= '0; g5 <= '0; b5 <= '0;
			end else begin
				r5 <= mem_dout[14:10];
				g5 <= mem_dout[9:5];
				b5 <= mem_dout[4:0];
			end
		end
	end

endmodule
