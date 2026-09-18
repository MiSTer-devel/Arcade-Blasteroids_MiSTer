`timescale 1ns/1ps
//============================================================================
//  Blasteroids 68000 program ROM — 256 KB (128K x 16) in on-chip block RAM.
//
//  Copyright (C) 2026 RetroShrimp.  GPL-3.0-or-later; see LICENSE and
//  NOTICE.md.  Adapted from the Xybots MiSTer core's
//  rtl/mem/xybots_prog_rom.sv (GPL-3.0, same author; itself ported from the
//  Toobin' core's toobin_program_rom.sv), resized to 256 KB and with Xybots'
//  SLAPSTIC bank substitution and 27256 A16 fold removed — neither exists on
//  this board.
//
//  ---- the four sockets ----------------------------------------------------
//  SP-316 sheet 12 labels the region "256K BYTES PROGRAM ROM": four 27512s in
//  two even/odd pairs,
//    6C / 6B   even / odd, 68000 0x000000-0x01FFFF   (2 x 64 KB)
//    4C / 4B   even / odd, 68000 0x020000-0x03FFFF   (2 x 64 KB)
//  which the MRA concatenates into one flat 0x40000-byte image.  So this
//  module is a single linear 128K-word array with no socket decode at all.
//  Both pairs decode all 16 address lines, so there is no A16 fold.
//
//  Sheet 12 reserves 0x038000-0x03FFFF as "32K BYTES SLAPSTIC PROGRAM ROM
//  SPACE", but no slapstic is fitted on a Blasteroids board and MAME's driver
//  has none either, so this is a plain ROM.
//  (A18-A22 are undecoded on the board, so the whole A23 = 0 half mirrors
//  every 0x40000; that mirroring belongs to the bus decoder, not to the ROM.)
//
//  ---- read port -----------------------------------------------------------
//  rd_a is the 68000 WORD address A18..A1 (rd_a[0] = A1), i.e. the byte
//  address >> 1 with A19+ already stripped by the bus decoder.  Registered
//  read: data is valid one clk after the address, which must be held stable
//  for that clock — the bus adapter holds it for the whole 68000 cycle.
//
//  ---- byte order, the one thing that must not be guessed -------------------
//  Download byte 2N is the first byte of the MRA's interleave (map="01") and
//  is the 68000 word's HIGH lane D15:8; byte 2N+1 (map="10") is D7:0.
//  Reversing this byte-swaps every opcode; the CPU then fetches a garbage
//  reset vector and runs nonsense from its first instruction, with no error
//  message anywhere to say so.
//
//  ---- resource ------------------------------------------------------------
//  128K x 8 per lane = 128 M10K per lane, so 256 blocks live in this one
//  module — a large but affordable slice of a DE10-Nano's 553.  Two byte-wide
//  arrays, rather than one 16-bit array with bit-select writes, because that
//  is the textbook byte-enable inference pattern and leaves no doubt about
//  Quartus picking M10K over LABs.  No megafunction instantiation and no init
//  file: the loader fills the array at download time.
//============================================================================

module blstroid_prog_rom
(
	input  logic        clk,

	// loader write port (byte stream from blstroid_rom_loader)
	input  logic        wr,
	input  logic [17:0] wr_addr,    // byte offset 0..0x3FFFF
	input  logic  [7:0] wr_data,

	// 68000 read port (16-bit big-endian word)
	input  logic [16:0] rd_a,       // 68000 A18..A1
	output logic [15:0] rd_data
);

	localparam int WORDS = 131072;   // 0x40000 bytes / 2

	logic [7:0] mem_hi [0:WORDS-1];   // even download byte -> D15:8
	logic [7:0] mem_lo [0:WORDS-1];   // odd  download byte -> D7:0

	// Power-up to 0.  Quartus zeroes M10K contents on configuration; the loop is
	// simulation-only (it keeps an unwritten hole from reading X) and is skipped
	// during synthesis, where >5000 iterations exceed the unroll limit.
`ifndef ALTERA_RESERVED_QIS
	initial begin
		for (int i = 0; i < WORDS; i++) begin
			mem_hi[i] = '0;
			mem_lo[i] = '0;
		end
	end
`endif

	logic [7:0] q_hi, q_lo;

	always_ff @(posedge clk) begin
		if (wr && !wr_addr[0]) mem_hi[wr_addr[17:1]] <= wr_data;
		if (wr &&  wr_addr[0]) mem_lo[wr_addr[17:1]] <= wr_data;
		q_hi <= mem_hi[rd_a];
		q_lo <= mem_lo[rd_a];
	end

	assign rd_data = {q_hi, q_lo};

endmodule
