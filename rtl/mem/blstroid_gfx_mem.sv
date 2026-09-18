`timescale 1ns/1ps
//============================================================================
//  Blasteroids graphics memory subsystem — arbitrates the two SDRAM read
//  clients plus the loader write port onto the single-port `blstroid_sdram`
//  controller.
//
//  Copyright (C) 2026 RetroShrimp.  GPL-3.0-or-later; see LICENSE and
//  NOTICE.md.  Derived from the Xybots MiSTer core's rtl/mem/xybots_gfx_mem.sv
//  (GPL-3.0, same author; itself from the Toobin' core's toobin_gfx_mem.sv).
//
//  This header is the interface contract: both video clients can be driven
//  from this text alone, without reading the body.
//
//  ---- clients -------------------------------------------------------------
//    PF stamp row  one 8x8 4bpp playfield stamp row = 4 bytes -> ONE burst-2
//                  read, assembled into 32 bits.  (The stamp is 16 x 8 on
//                  screen; its 8 ROM pixels are doubled horizontally, so the
//                  ROM row really is 4 bytes.)  Region `tiles`, 256 KB.
//    MO stamp row  one 16x8 4bpp motion-object stamp row = 4 bytes from the
//                  M-row bank AND 4 from the N-row bank at the same within-half
//                  offset -> ONE burst-4 read, assembled into 64 bits, because
//                  `blstroid_sdram_loader` interleaved the two halves at load
//                  time.  Region `sprites`, 1 MB.
//    Loader        one 16-bit word write (address already in SDRAM word space,
//                  produced by blstroid_sdram_loader).
//
//  ---- read-client contract ------------------------------------------------
//
//    pf_addr[17:2]  BYTE address bits 17:2 of the wanted `tiles` byte; the
//                   fetch returns the four bytes {pf_addr,2'b00}..{pf_addr,2'b11}.
//    pf_req         raise WITH the address and HOLD both.
//    pf_ack         one-cycle pulse; `pf_data` is valid in that cycle and is
//                   HELD until this client's next transaction completes.
//    pf_data[31:0]  {b3,b2,b1,b0}; b0 is the (A1,A0)=00 ROM byte, i.e. the
//                   leftmost pixel pair before h-flip.
//    then           drop `pf_req`.  The arbiter waits for the drop before
//                   serving anything else, so one request = one transaction.
//
//    mo_addr[18:2]  BYTE address bits 18:2 of the wanted stamp row WITHIN ONE
//                   512 KB HALF (both halves share this address — that is what
//                   the M/N banks are).  Do NOT add 0x80000 for the N row.
//    mo_req/mo_ack  identical protocol to PF.
//    mo_data[63:0]  { N3,N2,N1,N0, M3,M2,M1,M0 } — the low 32 bits are the four
//                   M-row (first half, stream 0x090000) bytes and the high 32
//                   the four N-row (second half, stream 0x110000) bytes, both
//                   at {mo_addr,2'b00}..{mo_addr,2'b11} within their half.
//                   b0 of each half is again the (A1,A0)=00 byte.
//
//  The h-flip XOR on A1/A0 lives in the pixel path, which is why the whole row
//  is fetched unconditionally.  Which half feeds which pixel of a pair is
//  decided there too, not here; this module only guarantees that M and N
//  arrive together and correctly labelled.
//
//  ---- arbitration: round-robin between the readers, not fixed priority ----
//  The loader write port beats both readers — it only runs while
//  `ioctl_download` is high, when neither reader exists yet.  The two READERS
//  ALTERNATE: when only one is asking it is served immediately; when both ask,
//  whichever was NOT served last goes first.
//
//  This is deliberately not Toobin's fixed `write > MO > PF`.  Both layers here
//  have a hard per-stamp deadline — the playfield must have its row before the
//  stamp is shifted out, the motion object before its line buffer is displayed
//  — so neither can be the absolute loser; with fixed priority a run of
//  back-to-back MO fetches starves the playfield well past its deadline.
//  Alternating bounds each client at its own transaction plus one of the
//  other's and changes total occupancy not at all.
//
//  ---- the per-scanline budget ---------------------------------------------
//  One scanline is 912 pixels x 4 clk_sys = 3648 clk_sys (the pixel clock is
//  14.318181 MHz = clk_sys/4).  Per line the SDRAM must serve:
//
//    PF  one burst-2 per visible 16-pixel stamp: 640/16 = 40, +1 prefetch = 41
//        (hardware ceiling 57 — the board fetches 456/8 = 57 map words a line)
//    MO  one burst-4 per object covering the line; hardware ceiling is again
//        57, because the sheet-5 slot schedule gives the motion-object engine
//        one link per 8 `1H` counts and a line is 456 counts
//
//  Controller occupancy is 10 clk for a burst-2, 12 for a burst-4 and 5 for an
//  AUTO_REFRESH (~9 per line), so even the hardware ceiling
//
//    57 x 10 + 57 x 12 + 9 x 5 = 1299 clk = 36 % of 3648
//
//  fits comfortably: one 57.272727 MHz controller carries both layers in well
//  under half a scanline and no second memory port or PLL tap is needed.
//
//  ---- controller contract -------------------------------------------------
//  When `sd_ready` is high, pulse `sd_req` for one cycle with addr/we/blen
//  (/wdata); read data returns on `sd_rdata` with one `sd_valid` pulse per
//  word, consecutive for a burst; `sd_ready` returns high when the access
//  completes.  A request coinciding with a due refresh is latched inside the
//  controller, so the single-cycle pulse is safe.
//============================================================================

module blstroid_gfx_mem #(
	parameter int AW = 24,
	parameter logic [AW-1:0] TILE_BASE = 24'h000000,   // 0x20000 words (256 KB)
	parameter logic [AW-1:0] SPR_BASE  = 24'h020000    // 0x80000 words (1 MB)
)(
	input  logic          clk,
	input  logic          reset,

	// ---- playfield stamp-row client ----
	input  logic          pf_req,
	input  logic [17:2]   pf_addr,       // tiles byte address, bits 17:2
	output logic          pf_ack,
	output logic [31:0]   pf_data,       // {b3,b2,b1,b0}

	// ---- motion-object stamp-row client ----
	input  logic          mo_req,
	input  logic [18:2]   mo_addr,       // WITHIN-HALF sprites byte address 18:2
	output logic          mo_ack,
	output logic [63:0]   mo_data,       // {N3,N2,N1,N0, M3,M2,M1,M0}

	// ---- loader write port (16-bit word, address already in SDRAM word space) ----
	input  logic          dl_wr,
	input  logic [AW-1:0] dl_waddr,
	input  logic [15:0]   dl_wdata,
	output logic          dl_ack,

	// ---- SDRAM controller port ----
	output logic          sd_req,
	output logic [AW-1:0] sd_addr,
	output logic          sd_we,
	output logic [1:0]    sd_blen,
	output logic [15:0]   sd_wdata,
	input  logic          sd_ready,
	input  logic          sd_valid,
	input  logic [15:0]   sd_rdata
);

	typedef enum logic [1:0] { G_IDLE, G_REQ, G_WAIT, G_HOLD } gstate_t;
	gstate_t st;

	typedef enum logic [1:0] { OP_WR, OP_MO, OP_PF } op_t;
	op_t op;

	logic [1:0]  wcnt;      // burst word counter (0..3)
	logic [47:0] wbuf;      // words already collected, low word first
	logic        last_mo;   // round-robin: 1 = MO was the last reader served

	// when both readers ask, the one that did NOT go last goes now
	wire take_mo = mo_req && (!pf_req || !last_mo);
	wire take_pf = pf_req && (!mo_req ||  last_mo);

	// PF: byte address {pf_addr,2'b00} -> word address {pf_addr,1'b0}, burst-2.
	// MO: the half-interleaved map puts the four words of one stamp row at
	//     {mo_addr, 2'b00} .. {mo_addr, 2'b11} (blstroid_sdram_loader's header),
	//     so the burst-4 starts at {mo_addr, 2'b00}.
	wire [AW-1:0] mo_wa = SPR_BASE  + {{(AW-19){1'b0}}, mo_addr, 2'b00};
	wire [AW-1:0] pf_wa = TILE_BASE + {{(AW-17){1'b0}}, pf_addr, 1'b0};

	// last word of the burst: 3 for MO (burst-4), 1 for PF (burst-2)
	wire [1:0] wlast = (op == OP_MO) ? 2'd3 : 2'd1;

	always_ff @(posedge clk) begin
		if (reset) begin
			st <= G_IDLE; sd_req <= 1'b0; sd_we <= 1'b0; sd_blen <= 2'd0;
			mo_ack <= 1'b0; pf_ack <= 1'b0; dl_ack <= 1'b0; last_mo <= 1'b0;
		end else begin
			mo_ack <= 1'b0; pf_ack <= 1'b0; dl_ack <= 1'b0;
			sd_req <= 1'b0;
			case (st)
				// -------- pick a client (write first, then alternate readers) ----
				G_IDLE: begin
					wcnt <= 2'd0;
					if (dl_wr) begin
						op <= OP_WR; sd_addr <= dl_waddr; sd_we <= 1'b1; sd_blen <= 2'd0;
						sd_wdata <= dl_wdata;
						if (sd_ready) begin sd_req <= 1'b1; st <= G_REQ; end
					end else if (take_mo) begin
						op <= OP_MO; sd_addr <= mo_wa; sd_we <= 1'b0; sd_blen <= 2'd3;
						if (sd_ready) begin sd_req <= 1'b1; st <= G_REQ; last_mo <= 1'b1; end
					end else if (take_pf) begin
						op <= OP_PF; sd_addr <= pf_wa; sd_we <= 1'b0; sd_blen <= 2'd1;
						if (sd_ready) begin sd_req <= 1'b1; st <= G_REQ; last_mo <= 1'b0; end
					end
				end
				// -------- req asserted for exactly this cycle --------
				G_REQ: begin
					sd_req <= 1'b0; sd_we <= 1'b0;
					st <= G_WAIT;
				end
				// -------- await completion (burst words on consecutive sd_valid) --------
				G_WAIT: begin
					if (op == OP_WR) begin
						if (sd_ready) begin dl_ack <= 1'b1; st <= G_HOLD; end
					end else if (sd_valid) begin
						if (wcnt != wlast) begin
							wbuf <= {sd_rdata, wbuf[47:16]};   // shift in, low word first
							wcnt <= wcnt + 2'd1;
						end else begin
							if (op == OP_MO) begin
								mo_data <= {sd_rdata, wbuf[47:0]};
								mo_ack  <= 1'b1;
							end else begin
								pf_data <= {sd_rdata, wbuf[47:32]};
								pf_ack  <= 1'b1;
							end
							st <= G_HOLD;
						end
					end
				end
				// -------- one transaction per request: wait for req to drop --------
				G_HOLD: begin
					case (op)
						OP_WR:   if (!dl_wr)  st <= G_IDLE;
						OP_MO:   if (!mo_req) st <= G_IDLE;
						default: if (!pf_req) st <= G_IDLE;   // OP_PF
					endcase
				end
				default: st <= G_IDLE;
			endcase
		end
	end

endmodule
