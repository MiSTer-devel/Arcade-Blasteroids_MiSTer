`timescale 1ns/1ps
//============================================================================
//  Generic 16-bit dual-port block RAM with independent byte write enables.
//
//  Copyright (C) 2026 RetroShrimp.  GPL-3.0-or-later; see LICENSE and
//  NOTICE.md.  Derived from the Xybots MiSTer core's
//  rtl/main/xybots_dpram_be.sv (GPL-3.0, same author), unchanged apart from
//  the module name.
//
//  Used twice on the main board:
//    * the 16 KB video + work RAM at FFC000-FFFFFF (AW = 13, 8 K words):
//      playfield RAM FFC000-FFCFFF, motion-object RAM FFD000-FFDFFF, free work
//      RAM FFE000-FFFFFF.  Byte writes on either lane must be honoured
//      independently (SP-316 sheet 12);
//    * the colour RAM at FFA000-FFA3FE, 512 x 16 (AW = 9): 0x000-0x0FF are the
//      motion-object palettes, 0x100-0x1FF the playfield palettes, xRGB 5:5:5.
//
//  Port A is the 68000 side (read + byte writes), port B is the video side
//  (read only).  Both are registered-output, which is what an M10K infers and
//  what makes the read data settle a full clk_sys before /DTACK is asserted.
//  Two byte-wide arrays rather than one 16-bit array with bit-select writes:
//  that is the byte-enable inference pattern Quartus recognises.
//============================================================================

module blstroid_dpram_be #(
	parameter int AW = 13
)(
	input  logic          clk,

	// port A — 68000
	input  logic [AW-1:0] a_addr,
	input  logic [15:0]   a_din,
	input  logic          a_we_hi,   // D15:8
	input  logic          a_we_lo,   // D7:0
	output logic [15:0]   a_dout,

	// port B — video (read only)
	input  logic [AW-1:0] b_addr,
	output logic [15:0]   b_dout
);

	localparam int WORDS = 1 << AW;

	logic [7:0] mem_hi [0:WORDS-1];
	logic [7:0] mem_lo [0:WORDS-1];

`ifndef ALTERA_RESERVED_QIS
	// Simulation-only zero fill so a simulator never reads X out of an
	// unwritten location.  Quartus zeroes M10K contents at configuration.
	initial begin
		for (int i = 0; i < WORDS; i++) begin
			mem_hi[i] = '0;
			mem_lo[i] = '0;
		end
	end
`endif

	always_ff @(posedge clk) begin
		if (a_we_hi) mem_hi[a_addr] <= a_din[15:8];
		if (a_we_lo) mem_lo[a_addr] <= a_din[7:0];
		a_dout <= {mem_hi[a_addr], mem_lo[a_addr]};
		b_dout <= {mem_hi[b_addr], mem_lo[b_addr]};
	end

endmodule
