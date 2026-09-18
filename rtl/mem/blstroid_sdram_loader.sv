`timescale 1ns/1ps
//============================================================================
//  Blasteroids SDRAM loader-writer — packs the download byte stream into
//  16-bit words and writes the two graphics regions through the SINGLE
//  `blstroid_gfx_mem` write port.
//
//  Copyright (C) 2026 RetroShrimp.  GPL-3.0-or-later; see LICENSE and
//  NOTICE.md.  Derived from the Xybots MiSTer core's
//  rtl/mem/xybots_sdram_loader.sv (GPL-3.0, same author; itself from the
//  Toobin' core's toobin_sdram_loader.sv).
//
//  ---- the sprite half interleave, the one non-linear part of the map -------
//  The playfield region maps linearly (byte -> byte/2), because one 8x8 4bpp
//  stamp row is 4 consecutive bytes in one flat region:
//
//    tiles   (MAME gfx_8x8x4_packed_msb): 32 bytes per 8x8 stamp,
//            byte = code[12:0]*32 + row[2:0]*4 + (A1,A0);
//            one PF stamp row = 4 CONSECUTIVE bytes = 2 words = ONE burst-2.
//
//  The motion object is not.  A Blasteroids MO stamp is 16 x 8 and its pixels
//  come alternately from the two 512 KB halves of the `sprites` region — the
//  board's M row and N row banks of MO stamp ROMs (sheets 8 and 7), addressed
//  with the SAME within-half address (MAME `molayout`):
//
//    sprites (MAME molayout): 32 bytes per 16x8 stamp PER HALF,
//            within-half byte = code[13:0]*32 + row[2:0]*4 + (A1,A0);
//            one MO stamp row = 4 bytes from M AND 4 bytes from N, both at
//            the SAME within-half offset -> 8 bytes, 4 words.
//
//  If the halves were stored where the download stream puts them (M at +0,
//  N at +0x80000) an MO row would need TWO separate 4-byte reads 512 KB apart,
//  i.e. two ACTIVEs, two precharges and two arbitration rounds per stamp row.
//  So the loader interleaves the halves at load time, moving the half-select
//  bit down to sit just above the within-row byte:
//
//    sprites word address = SPR_BASE + { a[18:2], half, a[1] }
//                           where half = sprites_addr[19]  (0 = M, 1 = N)
//                             and a    = sprites_addr[18:0] (within-half byte)
//
//  which lays each stamp row out as four consecutive, 4-word-aligned words:
//
//    {a[18:2], 0, 0} = M bytes 0,1     {a[18:2], 1, 0} = N bytes 0,1
//    {a[18:2], 0, 1} = M bytes 2,3     {a[18:2], 1, 1} = N bytes 2,3
//
//  so `blstroid_gfx_mem` fetches a whole 16-pixel MO stamp row with one
//  burst-4 read whose 64 bits are {N3,N2,N1,N0,M3,M2,M1,M0}.  Being 4-word
//  aligned it can never cross an SDRAM row (512 words) either.
//
//  This is a permutation of the address bits only — each 16-bit word still
//  holds the same consecutive even/odd byte pair it would have held in a flat
//  map — so the even/odd pairing logic below is untouched.
//
//  The M/N half order is load-bearing: reverse it and every motion object is
//  mirrored in 2-pixel pairs, a corruption subtle enough to survive a casual
//  look at the screen.
//
//  ---- SDRAM word map (word addresses; 32 MB module, 1.25 MB used) ----------
//    TILE_BASE 0x000000 .. 0x01FFFF   tiles   0x40000 bytes (256 KB)
//    SPR_BASE  0x020000 .. 0x09FFFF   sprites 0x100000 bytes (1 MB, interleaved)
//
//  ---- byte lanes ----------------------------------------------------------
//  EVEN download byte -> D7:0, ODD download byte -> D15:8.  A stamp row is then
//  {word1, word0} = {b3, b2, b1, b0} with b0 the (A1,A0)=00 ROM byte — the
//  leftmost pixel pair before h-flip.  This is the opposite convention from the
//  program ROM, which is big-endian because the MRA interleaves it into 68000
//  words; the graphics ROMs are byte-addressed devices with no word order of
//  their own, so the natural little-endian packing is used and both sides of
//  the datapath agree on it.
//
//  Even and odd bytes of a word are always consecutive in the download stream,
//  so the even byte is latched and the word is emitted on the odd byte.
//  `wr_busy` is combinational and is asserted while a word is outstanding (and
//  in the cycle a word completes); drive `ioctl_wait` with it so the byte
//  stream can never outrun the SDRAM.
//
//  ---- the write-path rule -------------------------------------------------
//  This is a SINGLE write path through the shared `blstroid_gfx_mem` controller
//  port — NOT a separate write client behind the read arbiter.  A write client
//  placed behind the arbiter ends up with no path to the pins and Quartus
//  prunes it away silently; after any arbiter change, check the fitter's
//  map report for "Lost fanout" on the request/we nets.
//============================================================================

module blstroid_sdram_loader #(
	parameter int AW = 24,
	parameter logic [AW-1:0] TILE_BASE = 24'h000000,
	parameter logic [AW-1:0] SPR_BASE  = 24'h020000
)(
	input  logic          clk,
	input  logic          reset,

	// ---- byte strobes from blstroid_rom_loader (the SDRAM regions only) ----
	input  logic          tiles_wr,
	input  logic [17:0]   tiles_addr,     // byte 0..0x3FFFF
	input  logic          sprites_wr,
	input  logic [19:0]   sprites_addr,   // byte 0..0xFFFFF ([19] = M/N half)
	input  logic  [7:0]   ld_data,
	output logic          wr_busy,        // -> ioctl_wait (back-pressure)

	// ---- gfx_mem write port ----
	output logic          dl_wr,
	output logic [AW-1:0] dl_waddr,
	output logic [15:0]   dl_wdata,
	input  logic          dl_ack
);

	// ---- is this byte the even (first) or the word-completing (odd) one? ----
	wire tile_ev = tiles_wr   && !tiles_addr[0];
	wire tile_od = tiles_wr   &&  tiles_addr[0];
	wire spr_ev  = sprites_wr && !sprites_addr[0];
	wire spr_od  = sprites_wr &&  sprites_addr[0];

	wire any_ev = tile_ev | spr_ev;
	wire any_od = tile_od | spr_od;     // word-completing

	logic [7:0] lo_buf;                 // the latched even byte -> D7:0

	// The half-interleave permutation lives here and nowhere else (see header).
	wire [18:0] spr_word = {sprites_addr[18:2], sprites_addr[19], sprites_addr[1]};

	wire [AW-1:0] word_addr = tile_od
		? (TILE_BASE + {{(AW-17){1'b0}}, tiles_addr[17:1]})
		: (SPR_BASE  + {{(AW-19){1'b0}}, spr_word});
	wire [15:0]   word_data = {ld_data, lo_buf};   // odd -> high, even -> low

	typedef enum logic [0:0] { L_IDLE, L_WR } lstate_t;
	lstate_t st;

	// combinational back-pressure: busy while writing, or the cycle a word completes
	assign wr_busy = (st == L_WR) || (st == L_IDLE && any_od);

	always_ff @(posedge clk) begin
		if (reset) begin
			st <= L_IDLE; dl_wr <= 1'b0;
		end else begin
			// The even byte is latched whatever the state.  Latching only in
			// L_IDLE would drop an even byte that arrived while a word was
			// outstanding, and the NEXT odd byte would then pair with a stale low
			// byte — a silently CORRUPTED word rather than a lost one.  `wr_busy`
			// makes that unreachable as long as it is wired to `ioctl_wait`;
			// latching unconditionally makes it unreachable either way.
			if (any_ev) lo_buf <= ld_data;
			case (st)
				L_IDLE: begin
					dl_wr <= 1'b0;
					if (any_od) begin
						dl_waddr <= word_addr;
						dl_wdata <= word_data;
						dl_wr    <= 1'b1;
						st       <= L_WR;
					end
				end
				L_WR: begin
					if (dl_ack) begin dl_wr <= 1'b0; st <= L_IDLE; end
				end
				default: st <= L_IDLE;
			endcase
		end
	end

	// Simulation-only overrun check, in its own process so no synthesisable
	// always_ff contains a system task.  A word-completing (odd) byte arriving
	// while the previous word is still outstanding cannot be recovered — it
	// means `ioctl_wait` is not wired to `wr_busy`.
`ifndef ALTERA_RESERVED_QIS
	always @(posedge clk)
		if (!reset && st == L_WR && any_od)
			$display("ERROR: blstroid_sdram_loader overrun - a word-completing byte arrived while a write was outstanding; ioctl_wait is not wired to wr_busy");
`endif

endmodule
