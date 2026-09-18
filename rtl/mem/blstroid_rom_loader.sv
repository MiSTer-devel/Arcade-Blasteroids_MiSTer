`timescale 1ns/1ps
//============================================================================
//  Blasteroids ROM download router — MiSTer index-0 ioctl stream -> region
//  writes.
//
//  Copyright (C) 2026 RetroShrimp.  GPL-3.0-or-later; see LICENSE and
//  NOTICE.md.  Derived from the Xybots MiSTer core's
//  rtl/mem/xybots_rom_loader.sv (GPL-3.0, same author; itself from the Toobin'
//  core); the stream map and the region widths are Blasteroids-specific.
//
//  ---- the index-0 stream map ----------------------------------------------
//  The MRA concatenates MAME's ROM regions in this order, and the offsets
//  below must match it byte for byte:
//
//    stream base   size       MAME region  destination
//    0x000000      0x040000   maincpu      68000 program ROM   -> BRAM (word)
//    0x040000      0x010000   jsa:cpu      6502 sound ROM      -> BRAM (byte)
//    0x050000      0x040000   tiles        playfield stamps    -> SDRAM
//    0x090000      0x100000   sprites      motion-object stamps-> SDRAM
//    0x190000      0x000200   —            X2804 EEPROM image  -> BRAM (byte)
//    total         0x190200                (1 638 912 bytes)
//
//  There are no socket holes and no ROM_RELOAD anywhere in this stream, so
//  every region is flat and the decode below is a pure comparison chain.
//
//  ---- the sprite region is TWO 512 KB halves ------------------------------
//  `sprites` is 1 MB: bytes 0x00000-0x7FFFF are the board's M row MO stamp
//  ROMs (sheet 8) and 0x80000-0xFFFFF the N row (sheet 7).  Both banks are
//  addressed with the SAME within-half address and together give one 16-pixel
//  MO stamp row.  This module keeps the region FLAT — `sprites_addr[19]` simply
//  IS the half select — and `blstroid_sdram_loader` performs the interleave
//  that puts the M and N bytes of one stamp row into four consecutive SDRAM
//  words.  The half order is load-bearing: reverse it and every motion object
//  is mirrored in 2-pixel pairs.
//
//  ---- byte order (the one thing that must not be guessed) -----------------
//  maincpu is emitted as BIG-ENDIAN INTERLEAVED WORDS — the even-offset MAME
//  ROM (6C / 4C) is `map="01"`, the FIRST byte of each output word, i.e.
//  D15:8.  So download byte 2N is the 68000 word's high lane and byte 2N+1 is
//  the low lane; `blstroid_prog_rom` splits them exactly that way.  Getting
//  this backwards byte-swaps every opcode and the 68000 fetches a garbage
//  reset vector with no error anywhere to say so.
//
//  ---- back-pressure -------------------------------------------------------
//  The BRAM regions accept a byte per clock and never stall.  The two SDRAM
//  regions go through `blstroid_sdram_loader`, which packs byte pairs into
//  words and raises `wr_busy` while a word is outstanding; `blstroid_core`
//  wires
//    assign ioctl_wait = sdl_wr_busy;
//  This module deliberately does not consume `ioctl_wait` itself: hps_io holds
//  `ioctl_addr`/`ioctl_dout` and simply does not issue the next `ioctl_wr`, so
//  the decode is unaffected.
//
//  Outputs are REGISTERED, one clk_sys after the ioctl_wr.
//
//  `rom_loaded` latches at the END of the INDEX-0 download only, with a
//  defined power-up value, so an NVRAM (index 2) restore on a core that has
//  never seen a ROM download cannot announce "ROMs are loaded".
//============================================================================

module blstroid_rom_loader
(
	input  logic        clk,
	input  logic        reset,          // ~pll_locked ONLY (never the game reset)

	// ---- HPS ioctl download channel ----
	input  logic        ioctl_download,
	input  logic        ioctl_wr,
	input  logic [26:0] ioctl_addr,     // byte offset into the index-0 stream
	input  logic  [7:0] ioctl_dout,
	input  logic [15:0] ioctl_index,

	// ---- 68000 program ROM (BRAM), byte port ----
	output logic        prog_rom_wr,
	output logic [17:0] prog_rom_addr,  // byte offset 0..0x3FFFF
	output logic  [7:0] prog_rom_data,

	// ---- JSA I 6502 sound ROM (BRAM), byte port ----
	output logic        snd_rom_wr,
	output logic [15:0] snd_rom_addr,   // byte offset 0..0xFFFF
	output logic  [7:0] snd_rom_data,

	// ---- playfield stamps -> SDRAM (via blstroid_sdram_loader) ----
	output logic        tiles_wr,
	output logic [17:0] tiles_addr,     // byte offset 0..0x3FFFF

	// ---- motion-object stamps -> SDRAM (via blstroid_sdram_loader) ----
	output logic        sprites_wr,
	// byte offset 0..0xFFFFF.  [19] = the M/N half select: 0 = M row (stream
	// 0x090000, sockets 5M..16M), 1 = N row (stream 0x110000, 5N..16N).
	output logic [19:0] sprites_addr,

	// shared byte for both SDRAM regions (they never strobe in the same cycle)
	output logic  [7:0] gfx_data,

	// ---- X2804 EEPROM initial image (BRAM/NVRAM), byte port ----
	output logic        eeprom_wr,
	output logic  [8:0] eeprom_addr,    // byte offset 0..0x1FF
	output logic  [7:0] eeprom_data,

	// ---- high once the index-0 download has ended ----
	output logic        rom_loaded
);

	// Region base offsets in the concatenated download stream; they must agree
	// with the MRA's region order (see the table in the header).
	localparam [26:0] BASE_PROG   = 27'h000000;
	localparam [26:0] BASE_SND    = 27'h040000;
	localparam [26:0] BASE_TILES  = 27'h050000;
	localparam [26:0] BASE_SPR    = 27'h090000;
	localparam [26:0] BASE_EEPROM = 27'h190000;
	localparam [26:0] END_STREAM  = 27'h190200;

	wire is_idx0 = (ioctl_index == 16'd0);
	wire wr      = ioctl_download & ioctl_wr & is_idx0;

	wire in_prog   = wr && (ioctl_addr <  BASE_SND);       // BASE_PROG == 0
	wire in_snd    = wr && (ioctl_addr >= BASE_SND)    && (ioctl_addr < BASE_TILES);
	wire in_tiles  = wr && (ioctl_addr >= BASE_TILES)  && (ioctl_addr < BASE_SPR);
	wire in_spr    = wr && (ioctl_addr >= BASE_SPR)    && (ioctl_addr < BASE_EEPROM);
	wire in_eeprom = wr && (ioctl_addr >= BASE_EEPROM) && (ioctl_addr < END_STREAM);

	logic [7:0] data_r;

	always_ff @(posedge clk) begin
		data_r        <= ioctl_dout;
		prog_rom_wr   <= in_prog   & ~reset;
		snd_rom_wr    <= in_snd    & ~reset;
		tiles_wr      <= in_tiles  & ~reset;
		sprites_wr    <= in_spr    & ~reset;
		eeprom_wr     <= in_eeprom & ~reset;
		prog_rom_addr <= 18'(ioctl_addr - BASE_PROG);
		snd_rom_addr  <= 16'(ioctl_addr - BASE_SND);
		tiles_addr    <= 18'(ioctl_addr - BASE_TILES);
		sprites_addr  <= 20'(ioctl_addr - BASE_SPR);
		eeprom_addr   <=  9'(ioctl_addr - BASE_EEPROM);
	end

	// one registered byte, fanned out to every destination port
	assign prog_rom_data = data_r;
	assign snd_rom_data  = data_r;
	assign gfx_data      = data_r;
	assign eeprom_data   = data_r;

	// rom_loaded latches on the falling edge of the INDEX-0 download.  `reset`
	// (~pll_locked) gives it a defined value from power-up, so simulation and
	// hardware agree and no consumer ever samples an X.
	wire  dl0 = ioctl_download & is_idx0;
	logic dl0_d;
	always_ff @(posedge clk) begin
		if (reset) begin
			dl0_d      <= 1'b0;
			rom_loaded <= 1'b0;
		end else begin
			dl0_d <= dl0;
			if (dl0)        rom_loaded <= 1'b0;   // download in progress
			else if (dl0_d) rom_loaded <= 1'b1;   // first cycle after it ends
		end
	end

endmodule
