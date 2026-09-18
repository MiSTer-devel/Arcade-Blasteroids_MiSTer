`timescale 1ns/1ps
//============================================================================
//  Generic byte-wide dual-port block RAM (one write port, one read port).
//
//  Copyright (C) 2026 RetroShrimp.  GPL-3.0-or-later; see LICENSE and
//  NOTICE.md.  Follows the inference style of the Xybots MiSTer core's
//  rtl/mem/xybots_dpram_be.sv (GPL-3.0, same author) with a single byte lane.
//
//  Used for the JSA I sound board's 6502 program ROM, 64 KB (AW = 16):
//  written by the ROM loader, read by the sound CPU.  The parameter makes it
//  reusable for any single-lane byte array of this shape.
//
//  Registered read, which is what an M10K infers: data is valid one clk after
//  the address.  Read-during-write to the same address returns the old data;
//  either port is only ever driven by one master at a time, so it never
//  matters here.
//============================================================================

module blstroid_ram8 #(
	parameter int AW = 16
)(
	input  logic          clk,

	input  logic          wr,
	input  logic [AW-1:0] wr_addr,
	input  logic  [7:0]   wr_data,

	input  logic [AW-1:0] rd_addr,
	output logic  [7:0]   rd_data
);

	localparam int WORDS = 1 << AW;

	logic [7:0] mem [0:WORDS-1];

`ifndef ALTERA_RESERVED_QIS
	// Simulation-only zero fill, so a read of an unwritten location is 0 rather
	// than X.  Quartus zeroes M10K contents at configuration anyway, and the
	// loop exceeds the synthesis unroll limit, hence the guard.
	initial begin
		for (int i = 0; i < WORDS; i++) mem[i] = '0;
	end
`endif

	always_ff @(posedge clk) begin
		if (wr) mem[wr_addr] <= wr_data;
		rd_data <= mem[rd_addr];
	end

endmodule
