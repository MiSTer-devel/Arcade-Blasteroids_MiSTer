`timescale 1ns/1ps
//============================================================================
//  Blasteroids (Atari Games, 1987) — game logic top, below the MiSTer `emu`
//  glue in Arcade-Blasteroids.sv.
//
//  Copyright (C) 2026 RetroShrimp.  GPL-3.0-or-later; see LICENSE and
//  NOTICE.md.  Port style, clock-enable generator and reset structure ported
//  from the Xybots MiSTer core's rtl/xybots_core.sv (GPL-3.0, same author).
//
//  Three boards and one memory path hang off this file:
//
//    blstroid_rom_loader / _prog_rom / _ram8 / _sdram_loader / _gfx_mem /
//    _sdram        the ioctl download, the 256 KB program ROM and the 64 KB
//                  sound ROM in BRAM, and the graphics SDRAM
//    blstroid_main   SP-316 sheets 2-5 and 12: fx68k, the address decode,
//                  /DTACK, the IRQ latches, the watchdog, the X2804 EEPROM and
//                  its MiSTer NVRAM transport, the inputs, the LETA, and the
//                  three RAMs the 68000 writes — the 16 KB video/work RAM, the
//                  512 x 16 colour RAM and the 256 x 1 priority RAM
//    blstroid_video  sheets 4-11: the SOS-2 sync generator, the sheet-5 slot
//                  arbiter, the playfield pipeline, the motion-object engine,
//                  the priority decision, the colour RAM read side and the R/2R
//                  DACs
//    blstroid_jsa    sheets 14-17: the whole JSA I audio PCB (T65 + jt51 + the
//                  SCOM mailbox + the mixer + the low-pass)
//
//  ---- where the three shared RAMs live ------------------------------------
//  `blstroid_main` owns all three arrays, because the 68000 write strobes are
//  decoded there, and `blstroid_video` is instantiated with
//  `INTERNAL_VRAM = INTERNAL_CRAM = INTERNAL_PRIO = 0` so that it reads them
//  through the registered video-side ports:
//
//      blstroid_video.vram_a     -> blstroid_main.vram_vaddr   (13 bits)
//      blstroid_video.vram_dout  <- blstroid_main.vram_vdata   (16 bits)
//      blstroid_video.cram_vaddr -> blstroid_main.cram_vaddr   ( 9 bits)
//      blstroid_video.cram_vdata <- blstroid_main.cram_vdata   (16 bits)
//      blstroid_video.prio_vaddr -> blstroid_main.prio_vaddr   ( 8 bits)
//      blstroid_video.prio_vdata <- blstroid_main.prio_vdata   ( 1 bit )
//
//  The latencies match by construction: `blstroid_dpram_be` port B is a
//  one-clk_sys registered read, which is what the video modules require of an
//  external array, and one clk_sys is none of the four a pixel lasts.  The
//  video block's own CPU ports are therefore tied off, and /DTACK stays where
//  the sheets put it, in the sheet-2 11D LS74 inside `blstroid_dtack`, one H
//  count ahead of the slot.  `blstroid_vram` still drives `vram_a` and still
//  latches VD15 into the 4K LS174B on 4H, so `PFM2` — the row-interrupt
//  condition — still comes from the sheet-6 latch regardless of who owns the
//  array.
//
//  ---- what blstroid_main takes from the video side ------------------------
//  The raw board timing, never the delayed picture: `h3`, `v3`, `h8`,
//  `hblank_cpu`, `vblank`, `pfm2`.  The 68000's FF9C01 status bits, the
//  halt-until-HBLANK flip-flop and the IRQ1 gate all sample the board's own
//  signals.  The scaler gets `vid_*`, delayed by the compositor's own two-pixel
//  depth.
//
//  It is `hblank_cpu`, not `hblank`.  The two differ by ten H counts, and the
//  difference decides whether the row-programmable interrupt exists at all —
//  see the declaration of `raw_hblank` below and blstroid_video.sv's header.
//
//  ---- clocking ------------------------------------------------------------
//  One clock domain, clk_sys = 57.272727 MHz = 4 x the 14.318181 MHz master
//  crystal, with integer clock enables and no fractional enables anywhere.
//  This works because the JSA I sound crystal (3.579545 MHz) is exactly the
//  video crystal / 4, so every rate on both boards divides clk_sys:
//
//     ce_14m  = clk_sys / 4   = 14.318181 MHz   pixel clock
//     ce_7m   = clk_sys / 8   =  7.159091 MHz   68000 and the 1H count
//     ce_ym   = clk_sys / 16  =  3.579545 MHz   YM2151 (JSA I)
//     ce_6502 = clk_sys / 32  =  1.789772 MHz   JSA I 6502
//
//  The pixel enable is ce_14m, so the video pipeline has four clk_sys cycles
//  per pixel — half what a 7 MHz-pixel Atari board of the same era gives — and
//  the per-scanline graphics fetch budget is 912 pixel clocks = 3 648 clk_sys.
//
//  All four enables are decodes of one free-running 5-bit counter, so each
//  slower enable is a strict subset of every faster one (a ce_6502 pulse is
//  always also a ce_ym, ce_7m and ce_14m pulse).  That property is what lets
//  the SDC treat the whole core as a single clk_sys domain with multicycle
//  exceptions per CE-gated block, and it is why no clock-domain crossing exists
//  inside the game logic.  The two crystals are physically independent on the
//  real PCB set; exact-frequency integer enables are the best available model,
//  and the SCOM mailbox is the only place the difference could be observed.
//
//  ---- reset structure -----------------------------------------------------
//    init_reset   ~pll_locked only.  Reaches the SDRAM controller, the graphics
//                 arbiter, the SDRAM word packer and the ROM loader, and
//                 nothing else.  A game reset there would drop SDRAM_CKE in the
//                 middle of a download and silently lose every write already
//                 made.
//    sys_reset    reset | ~rom_loaded.  The "board is not ready" reset: video,
//                 the JSA board, and the main bus's own init_reset/por.
//                 Holding the game off until the ROMs are in BRAM and SDRAM is
//                 what stops the 68000 executing FF FF.
//    cpu_reset    sys_reset | watchdog_reset — formed inside blstroid_main,
//                 because the sheet-2 /RESET net reaches only the 68000's
//                 RESET/HALT pins and the EEPROM unlock latch.  The watchdog
//                 does not reset the video side or the audio board; the audio
//                 board's own path is FF8C00 -> SCOM /RESREQ, which is
//                 `snd_reset`.
//============================================================================

module blstroid_core #(
	// The SOS-2's vertical total: 262 lines, MAME's figure.  The SOS-2 counts V
	// internally and the schematics do not settle 262 against the 263 the same
	// Atari sync-generator family produces, so this is a build-time parameter;
	// rebuild with 263 to try it.  It is not an OSD option because a runtime
	// value would put a comparator against a register into the sync chain and
	// into blstroid_mo's `disp_line`, with only four clk_sys per pixel to spare.
	parameter int V_TOTAL = 262,
	// 1 = MAME's xRGB_555 transfer, 0 = the sheet-11 R/2R ladder value.  The two
	// differ on 4 of the 32 codes and neither is distinguishable on a monitor;
	// see blstroid_dac.sv.
	parameter bit DAC_MAME_COMPAT = 1'b1
)(
	input  logic        clk_sys,        // 57.272727 MHz
	input  logic        reset,          // active-high system reset

	// SDRAM/loader init reset — MUST be ~pll_locked ONLY, never the game reset.
	input  logic        init_reset,

	// ---- HPS ioctl ROM download (index 0) ----
	input  logic        ioctl_download,
	input  logic        ioctl_wr,
	input  logic [26:0] ioctl_addr,
	input  logic  [7:0] ioctl_dout,
	input  logic [15:0] ioctl_index,
	output logic        ioctl_wait,

	// ---- NVRAM (X2804 EEPROM, 512 bytes) save-back to HPS (index 2) ----
	// The channel is handled by blstroid_nvram_io inside blstroid_main, which
	// takes both the index-0 MRA tail at 0x190000 and the index-2 restore/save
	// stream.  blstroid_rom_loader also extracts that tail; only one of the two
	// paths may be wired, and this core wires the main board's, so the loader's
	// `eeprom_*` outputs are deliberately unused.
	input  logic        ioctl_upload,
	output logic        ioctl_upload_req,
	output logic  [7:0] ioctl_upload_index,
	output logic  [7:0] ioctl_din,

	// ---- controls ----
	// Whirly-Gig: the LET connector's CLK/DIR pin pair per player, already
	// synchronised.  `blstroid_whirly` in Arcade-Blasteroids.sv synthesises them
	// from the MiSTer spinner / paddle / mouse / d-pad; the COUNTER is the LETA
	// custom at 8A, inside blstroid_main where sheet 4 puts it.
	//   [0] = player 1 (LET-5/6)   [1] = player 2 (LET-7/8)
	input  logic  [1:0] whirly_a,       // CLK0 / CLK1
	input  logic  [1:0] whirly_b,       // DIR0 / DIR1
	// Panel buttons, ACTIVE HIGH ("pressed").  0xFF9C01 / 0xFF9C03 D3:0 read
	// ACTIVE LOW on the board, and that inversion is in blstroid_inputs where
	// the sheet is transcribed.
	//   [0] Fire   [1] Thrust   [2] Transform   [3] Spare
	// D3 is sheet 12's "spare" and it is not a Start button.  The operator
	// manual lists exactly four inputs per player in the Switch Test (Spare,
	// Xform, Thrust, Fire), the control-panel figure and the CTRA connector
	// carry three buttons per player, and MAME defines no IPT_START1/2 either.
	// Blasteroids has a continuous buy-in: a player joins by coining up.  D3 is
	// wired anyway because the self test displays it.
	input  logic  [3:0] btn1, btn2,
	// Coins are read on the JSA audio board; the self-test switch is SW1 on the
	// same board and fans out to /RDIO D7 there and over JSCOM-1 to the main
	// board's 0xFF9C01 D7 — one switch, two readers.
	input  logic        coin1,
	input  logic        coin2,
	input  logic        self_test,

	// ---- video out (single clk_sys domain, qualified by ce_pix) ----
	// 8 bits per channel: colour RAM is xRGB 5:5:5 (sheet 12) and the DAC
	// transfer widens it to 8, so the core's output bus is 8-bit and
	// arcade_video is instantiated with DW = 24.
	output logic        ce_pix,
	output logic  [7:0] vga_r, vga_g, vga_b,
	output logic        hsync, vsync, hblank, vblank,

	// ---- audio ----
	output logic signed [15:0] aud_l, aud_r,

	// ---- SDRAM chip ----
	inout  wire  [15:0] SDRAM_DQ,
	output logic [12:0] SDRAM_A,
	output logic  [1:0] SDRAM_BA,
	output logic        SDRAM_DQML,
	output logic        SDRAM_DQMH,
	output logic        SDRAM_CKE,
	output logic        SDRAM_nCS,
	output logic        SDRAM_nRAS,
	output logic        SDRAM_nCAS,
	output logic        SDRAM_nWE
);

	// ===================== clock enables =====================
	// One free-running /32 counter; every enable is a decode of it, so the
	// slower enables are subsets of the faster ones (see the header).  The
	// counter is deliberately not reset — the PCB's sync and JSA dividers
	// free-run too, and holding it in reset would make the phase relationship
	// depend on when reset released.  There is no initialiser either: Quartus
	// powers the register up at 0, which is the phase the SDC's CE multicycle
	// exceptions assume.
	logic [4:0] ce_cnt;
	always_ff @(posedge clk_sys) ce_cnt <= ce_cnt + 5'd1;

	wire ce_14m  = (ce_cnt[1:0] == 2'd0);   // 14.318181 MHz  pixel
	wire ce_7m   = (ce_cnt[2:0] == 3'd0);   //  7.159091 MHz  68000 / 1H
	wire ce_ym   = (ce_cnt[3:0] == 4'd0);   //  3.579545 MHz  YM2151
	wire ce_6502 = (ce_cnt      == 5'd0);   //  1.789772 MHz  JSA 6502

	// ===================== resets =====================
	// `rom_loaded` latches at the end of the index-0 download, so the 68000
	// cannot execute FF FF out of an unwritten program ROM.  The MiSTer NVRAM
	// restore is a separate index-2 transfer that arrives just afterwards and
	// writes the X2804 array, so the 68000 is held for its duration too and the
	// restored image is in place before the boot reads a single EEPROM block;
	// otherwise the transfer lands under a running CPU.  The board's /RESET does
	// not erase the cell array, on the PCB or here, so nothing is lost by
	// holding it.
	logic rom_loaded;
	wire  nvram_download = ioctl_download & (ioctl_index == 16'd2);
	wire  sys_reset      = reset | ~rom_loaded | nvram_download;

	// =====================================================================
	//  ROM download; see blstroid_rom_loader.sv for the MRA layout it expects
	// =====================================================================
	logic        prog_rom_wr;
	logic [17:0] prog_rom_addr;
	logic  [7:0] prog_rom_data;

	logic        snd_rom_wr;
	logic [15:0] snd_rom_addr_w;
	logic  [7:0] snd_rom_data_w;

	logic        tiles_wr,   sprites_wr;
	logic [17:0] tiles_addr;
	logic [19:0] sprites_addr;
	logic  [7:0] gfx_data;

	logic        eeprom_wr;
	logic  [8:0] eeprom_addr;
	logic  [7:0] eeprom_data;

	blstroid_rom_loader u_loader (
		.clk            (clk_sys),
		.reset          (init_reset),     // ~pll_locked ONLY

		.ioctl_download (ioctl_download),
		.ioctl_wr       (ioctl_wr),
		.ioctl_addr     (ioctl_addr),
		.ioctl_dout     (ioctl_dout),
		.ioctl_index    (ioctl_index),

		.prog_rom_wr    (prog_rom_wr),
		.prog_rom_addr  (prog_rom_addr),
		.prog_rom_data  (prog_rom_data),

		.snd_rom_wr     (snd_rom_wr),
		.snd_rom_addr   (snd_rom_addr_w),
		.snd_rom_data   (snd_rom_data_w),

		.tiles_wr       (tiles_wr),
		.tiles_addr     (tiles_addr),
		.sprites_wr     (sprites_wr),
		.sprites_addr   (sprites_addr),
		.gfx_data       (gfx_data),

		.eeprom_wr      (eeprom_wr),
		.eeprom_addr    (eeprom_addr),
		.eeprom_data    (eeprom_data),

		.rom_loaded     (rom_loaded)
	);

	// =====================================================================
	//  The graphics memory path: byte pairs -> SDRAM words -> the single
	//  write port of the read arbiter -> the controller.
	//
	//  Three rules, none of them optional:
	//    1. the controller's `reset` is `init_reset` (~pll_locked) and nothing
	//       else — a game reset here would drop SDRAM_CKE mid-download and
	//       silently lose every loader write;
	//    2. one direct write path.  `blstroid_sdram_loader` -> gfx_mem's
	//       dedicated `dl_*` port, never behind the read arbiter, or Quartus
	//       prunes it ("Lost fanout");
	//    3. `ioctl_wait` must be the loader's `wr_busy`, or a word-completing
	//       byte can arrive while a write is outstanding and be lost.
	// =====================================================================
	logic        sdl_wr_busy;
	logic        dl_wr, dl_ack;
	logic [23:0] dl_waddr;
	logic [15:0] dl_wdata;

	blstroid_sdram_loader u_sdram_loader (
		.clk        (clk_sys),
		.reset      (init_reset),         // ~pll_locked ONLY
		.tiles_wr   (tiles_wr),   .tiles_addr   (tiles_addr),
		.sprites_wr (sprites_wr), .sprites_addr (sprites_addr),
		.ld_data    (gfx_data),
		.wr_busy    (sdl_wr_busy),
		.dl_wr      (dl_wr), .dl_waddr (dl_waddr), .dl_wdata (dl_wdata),
		.dl_ack     (dl_ack)
	);

	// Rule 3.  hps_io holds ioctl_addr/ioctl_dout and simply does not issue the
	// next ioctl_wr while this is high.
	assign ioctl_wait = sdl_wr_busy;

	// ---- the two read ports blstroid_video consumes ----------------------
	// Full contract in blstroid_gfx_mem.sv's header.
	//   PF: raise pf_req with pf_addr (tiles byte address 17:2) and hold both;
	//       pf_ack pulses once, pf_data = {b3,b2,b1,b0} of that 8x8 stamp row.
	//   MO: raise mo_req with mo_addr (within-half sprites byte address 18:2);
	//       mo_ack pulses once, mo_data = {N3..N0, M3..M0} — both halves of one
	//       16x8 stamp row, from one burst-4.  Do not add 0x80000 for the N row.
	// Then drop the request.  Worst-case latency is 32 clk_sys against the
	// 64 clk_sys a 16-pixel stamp allows.
	logic        pf_req, pf_ack;
	logic [17:2] pf_addr;
	logic [31:0] pf_data;
	logic        mo_req, mo_ack;
	logic [18:2] mo_addr;
	logic [63:0] mo_data;

	logic        sd_req, sd_we, sd_ready, sd_valid;
	logic [23:0] sd_addr;
	logic  [1:0] sd_blen;
	logic [15:0] sd_wdata, sd_rdata;

	blstroid_gfx_mem u_gfx_mem (
		.clk      (clk_sys),
		.reset    (init_reset),           // ~pll_locked ONLY (rule 1)
		.pf_req   (pf_req), .pf_addr (pf_addr), .pf_ack (pf_ack), .pf_data (pf_data),
		.mo_req   (mo_req), .mo_addr (mo_addr), .mo_ack (mo_ack), .mo_data (mo_data),
		.dl_wr    (dl_wr), .dl_waddr (dl_waddr), .dl_wdata (dl_wdata), .dl_ack (dl_ack),
		.sd_req   (sd_req), .sd_addr (sd_addr), .sd_we (sd_we), .sd_blen (sd_blen),
		.sd_wdata (sd_wdata), .sd_ready (sd_ready), .sd_valid (sd_valid),
		.sd_rdata (sd_rdata)
	);

	blstroid_sdram #(.CLK_KHZ (57273), .ROW_BITS (13), .COL_BITS (9)) u_sdram (
		.clk   (clk_sys),
		.reset (init_reset),              // ~pll_locked ONLY (rule 1)
		.addr  (sd_addr), .wdata (sd_wdata), .we (sd_we), .blen (sd_blen),
		.req   (sd_req), .rdata (sd_rdata), .valid (sd_valid), .ready (sd_ready),
		.SDRAM_DQ (SDRAM_DQ), .SDRAM_A (SDRAM_A), .SDRAM_BA (SDRAM_BA),
		.SDRAM_DQML (SDRAM_DQML), .SDRAM_DQMH (SDRAM_DQMH), .SDRAM_CKE (SDRAM_CKE),
		.SDRAM_nCS (SDRAM_nCS), .SDRAM_nRAS (SDRAM_nRAS),
		.SDRAM_nCAS (SDRAM_nCAS), .SDRAM_nWE (SDRAM_nWE)
	);

	// =====================================================================
	//  The two program ROMs in BRAM (256 + 64 M10K)
	// =====================================================================
	// 68000 program ROM, 128K x 16, read port shaped exactly as
	// blstroid_main's `prog_a` / `prog_data` (A17..A1, one clk_sys latency).
	logic [16:0] prog_a;
	logic [15:0] prog_data;

	blstroid_prog_rom u_prog_rom (
		.clk     (clk_sys),
		.wr      (prog_rom_wr), .wr_addr (prog_rom_addr), .wr_data (prog_rom_data),
		.rd_a    (prog_a),      .rd_data (prog_data)
	);

	// JSA I 6502 program ROM, 64 KB.  It is NOT inside blstroid_jsa: the board
	// reads it through `snd_rom_addr` (combinational) / `snd_rom_data` (one
	// clk_sys later), which is exactly what blstroid_ram8 provides.  The JSA's
	// 8 KB work RAM (2H) IS inside blstroid_jsa, in blstroid_jsa_mem.
	logic [15:0] snd_a;
	logic  [7:0] snd_q;

	blstroid_ram8 #(.AW(16)) u_snd_rom (
		.clk     (clk_sys),
		.wr      (snd_rom_wr), .wr_addr (snd_rom_addr_w), .wr_data (snd_rom_data_w),
		.rd_addr (snd_a),      .rd_data (snd_q)
	);

	// =====================================================================
	//  Video board — SP-316 sheets 4 to 11
	// =====================================================================
	// INTERNAL_VRAM/CRAM/PRIO = 0: the arrays live in blstroid_main (see the
	// header).  The CPU ports of this module are therefore tied off and its
	// `cpu_dout` / `cpu_slot` / `cpu_ack` / `cram_cpu_dout` are unused.
	logic [12:0] vram_vaddr;
	logic [15:0] vram_vdata;
	logic  [8:0] cram_vaddr;
	logic [15:0] cram_vdata;
	logic  [7:0] prio_vaddr;
	logic        prio_vdata;

	logic  [2:0] vid_h3, vid_v3;
	logic        vid_h8;
	logic  [8:0] vid_h, vid_vline;
	// Two horizontal blanks, and the difference is load bearing.  `raw_hblank`
	// is the displayed window (HB_END = 16 .. HB_START = 336, two whole stamp
	// groups of playfield prefetch instead of the board's ~6); `hblank_cpu` is
	// the board's (6 .. 326).  The row-interrupt latch samples PFM2, which holds
	// the column-40 flag for exactly the eight counts 324..331, so HBLANK must
	// rise inside that group or IRQ1 never fires: wiring `raw_hblank` to the
	// main board instead gives a cold boot with no level-1 interrupts and no
	// FF8E00 halts at all.  See blstroid_video.sv's header.  `raw_hblank` is
	// left exported and unused here so the asymmetry is visible rather than
	// implicit.
	logic        raw_hblank, hblank_cpu, raw_vblank, raw_pfm2;
	logic        hblank_rise, hblank_fall;
	logic  [9:0] hpos;
	logic  [8:0] vpos;
	logic        de_raw;

	logic  [7:0] vid_r, vid_g, vid_b;
	logic        vid_hs_n, vid_vs_n, vid_cs_n, vid_hb, vid_vb, vid_de;

	logic        mo_fetch_late, pf_fetch_late;
	logic  [5:0] mo_links_matched;
	logic  [3:0] lb_pix, lb_col, pf_pix;
	logic  [2:0] pf_pal;
	logic        pf_win;
	logic  [9:0] mo_hpos;

	// Unused external write side of blstroid_vram (INTERNAL_RAM = 0): the CPU
	// writes go through blstroid_main's own port A, so these are constants.
	logic [15:0] vid_vram_din;
	logic        vid_vram_we_hi, vid_vram_we_lo;
	logic [15:0] vid_cpu_dout, vid_cram_cpu_dout;
	logic        vid_cpu_slot, vid_cpu_ack;

	blstroid_video #(
		.V_TOTAL        (V_TOTAL),
		.DAC_MAME_COMPAT(DAC_MAME_COMPAT),
		.INTERNAL_VRAM  (1'b0),
		.INTERNAL_CRAM  (1'b0),
		.INTERNAL_PRIO  (1'b0)
	) u_video (
		.clk(clk_sys), .reset(sys_reset), .ce_14m(ce_14m), .ce_7m(ce_7m),

		// 68000 ports — unused at INTERNAL_* = 0
		.cpu_a(13'd0), .cpu_din(16'd0), .cpu_sel(1'b0),
		.cpu_we_hi(1'b0), .cpu_we_lo(1'b0),
		.cpu_dout(vid_cpu_dout), .cpu_slot(vid_cpu_slot), .cpu_ack(vid_cpu_ack),
		.cram_cpu_a(9'd0), .cram_cpu_din(16'd0),
		.cram_cpu_we_hi(1'b0), .cram_cpu_we_lo(1'b0),
		.cram_cpu_dout(vid_cram_cpu_dout),
		.prio_cpu_we(1'b0), .prio_cpu_a(8'd0), .prio_cpu_d(1'b0),

		// the ONE copy of each RAM, owned by blstroid_main
		.vram_a(vram_vaddr), .vram_din(vid_vram_din),
		.vram_we_hi(vid_vram_we_hi), .vram_we_lo(vid_vram_we_lo),
		.vram_dout(vram_vdata),
		.cram_vaddr(cram_vaddr), .cram_vdata(cram_vdata),
		.prio_vaddr(prio_vaddr), .prio_vdata(prio_vdata),

		// graphics SDRAM clients
		.pf_req(pf_req), .pf_addr(pf_addr), .pf_ack(pf_ack), .pf_data(pf_data),
		.mo_req(mo_req), .mo_addr(mo_addr), .mo_ack(mo_ack), .mo_data(mo_data),

		// RAW board timing -> blstroid_main
		.h3(vid_h3), .v3(vid_v3), .h8(vid_h8), .h(vid_h), .vline(vid_vline),
		.hblank(raw_hblank), .hblank_cpu(hblank_cpu),
		.vblank(raw_vblank), .pfm2(raw_pfm2),
		.hblank_rise_o(hblank_rise), .hblank_fall_o(hblank_fall),
		.hpos(hpos), .vpos(vpos), .de_raw(de_raw),

		// the picture, delayed by the compositor's own two pixels
		.red(vid_r), .green(vid_g), .blue(vid_b),
		.vid_hs_n(vid_hs_n), .vid_vs_n(vid_vs_n), .vid_cs_n(vid_cs_n),
		.vid_hb(vid_hb), .vid_vb(vid_vb), .vid_de(vid_de), .ce_pix(ce_pix),

		// status, observation only
		.mo_fetch_late(mo_fetch_late), .pf_fetch_late(pf_fetch_late),
		.mo_links_matched(mo_links_matched),
		.lb_pix(lb_pix), .lb_col(lb_col), .mo_hpos(mo_hpos),
		.pf_pix(pf_pix), .pf_pal(pf_pal), .pf_win(pf_win)
	);

	// =====================================================================
	//  Main board — SP-316 sheets 2, 3, 4, 5 and 12
	// =====================================================================
	logic        snd_cmd_wr, snd_resp_rd, snd_reset;
	logic  [7:0] snd_cmd_data, snd_resp_data;
	logic        snd_irq, snd_m2s_ready;

	logic [23:1] cpu_a;
	logic [15:0] cpu_dout, cpu_din;
	logic        cpu_as, cpu_uds_n, cpu_lds_n, cpu_rw, cpu_dtack;
	logic  [2:0] cpu_fc, ipl;
	logic        watchdog_reset, reset_net, irq1_pending, irq2_pending;

	// One-clk_sys pipeline stage on /VBLANK into blstroid_main (its consumers,
	// the sheet-2 watchdog and the IRQ2 latch, are all edge-detected).
	logic vblank_main;
	always_ff @(posedge clk_sys) vblank_main <= raw_vblank;

	blstroid_main #(.CLK_HZ(57_272_724)) u_main (
		.clk(clk_sys), .ce_7m(ce_7m),
		// sys_reset, NOT init_reset: the 68000 must be held while the ROMs
		// stream in, and the watchdog must be parked at POR until then.
		.init_reset(sys_reset), .por(sys_reset),

		// the raw board timing, never the delayed picture
		.h3(vid_h3), .v3(vid_v3), .h8(vid_h8),
		// hblank_cpu, not raw_hblank -- see the declaration above.
		.hblank(hblank_cpu), .vblank(vblank_main), .pfm2(raw_pfm2),

		.prog_a(prog_a), .prog_data(prog_data),

		// The X2804 factory image and the save/restore channel both arrive
		// through blstroid_nvram_io inside this module (index-0 tail at
		// 0x190000 and index 2).  blstroid_rom_loader's `eeprom_*` outputs are
		// the alternative hook — wire one, never both.
		.eeprom_img_wr(1'b0), .eeprom_img_addr(9'd0), .eeprom_img_data(8'd0),

		// the ONE copy of each RAM the video board reads
		.vram_vaddr(vram_vaddr), .vram_vdata(vram_vdata),
		.cram_vaddr(cram_vaddr), .cram_vdata(cram_vdata),
		.prio_vaddr(prio_vaddr), .prio_vdata(prio_vdata),

		.p1_buttons(btn1), .p2_buttons(btn2), .self_test(self_test),
		.whirly_a(whirly_a), .whirly_b(whirly_b),

		.snd_cmd_wr(snd_cmd_wr), .snd_cmd_data(snd_cmd_data),
		.snd_resp_rd(snd_resp_rd), .snd_reset(snd_reset),
		.snd_resp_data(snd_resp_data), .snd_irq(snd_irq),
		.snd_m2s_ready(snd_m2s_ready),

		.ioctl_download(ioctl_download), .ioctl_wr(ioctl_wr),
		.ioctl_addr(ioctl_addr), .ioctl_dout(ioctl_dout),
		.ioctl_index(ioctl_index), .ioctl_upload(ioctl_upload),
		.ioctl_upload_req(ioctl_upload_req),
		.ioctl_upload_index(ioctl_upload_index), .ioctl_din(ioctl_din),

		.cpu_a(cpu_a), .cpu_dout(cpu_dout), .cpu_din(cpu_din),
		.cpu_as(cpu_as), .cpu_uds_n(cpu_uds_n), .cpu_lds_n(cpu_lds_n),
		.cpu_rw(cpu_rw), .cpu_dtack(cpu_dtack), .cpu_fc(cpu_fc), .ipl(ipl),
		.watchdog_reset(watchdog_reset), .reset_net(reset_net),
		.irq1_pending(irq1_pending), .irq2_pending(irq2_pending)
	);

	// =====================================================================
	//  JSA I audio PCB — SP-316 sheets 14 to 17
	// =====================================================================
	// The watchdog does not reach this board; its only reset paths are power-up
	// and FF8C00 -> SCOM /RESREQ (`snd_reset`).  Coins are not swapped on
	// Blasteroids, so there is no SWAP_COINS parameter: COIN1 is JCOIN-4, the
	// right-hand chute.
	logic coin_ctr1, coin_ctr2;
	logic snd_busy_n;

	blstroid_jsa u_jsa (
		.clk(clk_sys), .reset(sys_reset),
		.ce_6502(ce_6502), .ce_ym(ce_ym),

		.snd_rom_addr(snd_a), .snd_rom_data(snd_q),

		.main_cmd_wr(snd_cmd_wr), .main_cmd_data(snd_cmd_data),
		.main_resp_rd(snd_resp_rd), .main_sndrst_wr(snd_reset),
		.main_resp_data(snd_resp_data),
		.main_irq4(snd_irq), .main_busy_n(snd_busy_n),
		.main_to_sound_ready(snd_m2s_ready),

		.coin1(coin1), .coin2(coin2), .coin3(1'b0), .coin4(1'b0),
		.self_test(self_test),
		.coin_ctr1(coin_ctr1), .coin_ctr2(coin_ctr2),

		// The board's own self-clearing periodic divider (sheet 17), not MAME's
		// free-running model of it; the sheet-16 low-pass is real hardware and
		// stays in circuit.
		.mame_irq_model(1'b0), .lpf_bypass(1'b0),

		.aud_left(aud_l), .aud_right(aud_r)
	);

	// =====================================================================
	//  Video output
	// =====================================================================
	// arcade_video wants active-high sync and blanking; the board's own signals
	// are active low.  These are the delayed (`vid_*`) ones — the compositor is
	// two pixel clocks deep and the scaler must see the picture's own timing,
	// not the raster's.
	assign hsync  = ~vid_hs_n;
	assign vsync  = ~vid_vs_n;
	assign hblank = vid_hb;
	assign vblank = vid_vb;

`ifdef BLSTROID_BRINGUP_DEBUG
	// ---- bring-up test pattern, not built for release ---------------------
	// Define BLSTROID_BRINGUP_DEBUG to replace the game's picture with the
	// colour-bar/box pattern.  It exercises the PLL, the raster, the memory
	// path and the whole MiSTer video chain independently of the game logic,
	// which is worth having when a board comes up black.  The pattern runs off
	// `reset`, not `sys_reset`, so it appears even on a board whose ROM
	// download never completes — which is exactly the board that needs
	// diagnosing.
	//
	// Its `probe` word is fed from the live graphics fetch and the two program
	// ROMs, so a stuck SDRAM phase or an unwritten BRAM shows as a frozen or
	// wrong probe strip rather than as a plausible picture.
	wire [9:0] tp_h;
	wire [8:0] tp_v;
	wire       tp_frame_start;
	wire [7:0] tp_r, tp_g, tp_b;
	wire       tp_hs, tp_vs, tp_hb, tp_vb;

	wire [15:0] probe_mix = prog_data ^ {snd_q, 8'h00}
	                      ^ pf_data[15:0]  ^ pf_data[31:16]
	                      ^ mo_data[15:0]  ^ mo_data[31:16]
	                      ^ mo_data[47:32] ^ mo_data[63:48];
	logic [15:0] probe_run, probe_lat;
	always_ff @(posedge clk_sys) begin
		if (reset) begin
			probe_run <= 16'd0;
			probe_lat <= 16'd0;
		end else if (tp_frame_start) begin
			probe_lat <= probe_run ^ probe_mix;
			probe_run <= 16'd0;
		end else begin
			probe_run <= probe_run ^ probe_mix;
		end
	end

	blstroid_testpat u_testpat (
		.clk        (clk_sys),
		.reset      (reset),            // NOT sys_reset — see above
		.ce         (ce_14m),           // 14.318181 MHz pixel clock

		.probe      (probe_lat),
		.dials      ({6'd0, whirly_a, 6'd0, whirly_b}),
		.btns       ({btn1, btn2, coin1, coin2, self_test, 5'b00000}),

		.h          (tp_h),
		.v          (tp_v),
		.frame_start(tp_frame_start),

		.hsync      (tp_hs),
		.vsync      (tp_vs),
		.hblank     (tp_hb),
		.vblank     (tp_vb),

		.r          (tp_r),
		.g          (tp_g),
		.b          (tp_b)
	);

	assign vga_r = tp_r;
	assign vga_g = tp_g;
	assign vga_b = tp_b;

	// The game's own picture is still generated (and still fetches tiles and
	// sprites out of the SDRAM, and the 68000 still runs) — it just does not
	// reach the pin.  That is what makes this a bring-up mode rather than a
	// different design.
	/* verilator lint_off UNUSEDSIGNAL */
	wire _unused_tp = &{1'b0, vid_r, vid_g, vid_b, tp_h, tp_v,
	                    tp_hs, tp_vs, tp_hb, tp_vb, 1'b0};
	/* verilator lint_on UNUSEDSIGNAL */
`else
	assign vga_r = vid_r;
	assign vga_g = vid_g;
	assign vga_b = vid_b;
`endif

	// =====================================================================
	//  Observation-only nets
	// =====================================================================
	// Everything below is either a board net with no consumer in a MiSTer core
	// (the coin-counter solenoids, the composite sync the VID connector
	// carries), a port that exists only because a module can be configured the
	// other way round (the video block's unused CPU ports at INTERNAL_* = 0,
	// the loader's unused EEPROM tap), or a status probe.  Collecting them in
	// one reader keeps the lint meaningful instead of blanket-disabled, and
	// gives Quartus exactly one unused-net warning to report.
	/* verilator lint_off UNUSEDSIGNAL */
	wire _unused_core = &{1'b0,
		coin_ctr1, coin_ctr2, snd_busy_n,                           // solenoids
		eeprom_wr, eeprom_addr, eeprom_data,                        // see header
		vid_cpu_dout, vid_cpu_slot, vid_cpu_ack, vid_cram_cpu_dout,
		vid_vram_din, vid_vram_we_hi, vid_vram_we_lo,
		vid_cs_n, vid_de,                                           // composite
		raw_hblank,                    // the displayed window; see above
		vid_h, vid_vline, hpos, vpos, de_raw,
		hblank_rise, hblank_fall,
		mo_fetch_late, pf_fetch_late, mo_links_matched,
		lb_pix, lb_col, mo_hpos, pf_pix, pf_pal, pf_win,
		cpu_a, cpu_dout, cpu_din, cpu_as, cpu_uds_n, cpu_lds_n, cpu_rw,
		cpu_dtack, cpu_fc, ipl, watchdog_reset, reset_net,
		irq1_pending, irq2_pending,
		1'b0};
	/* verilator lint_on UNUSEDSIGNAL */

endmodule
