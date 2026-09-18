`timescale 1ns/1ps
//============================================================================
//  Blasteroids (Atari Games, 1987) - JSA I sound-board bus.  Composes
//  blstroid_jsa_decode + _mem + _io + blstroid_sound_comm onto the 6502's bus
//  and exposes the YM2151 strobes for the external jt51 core: the whole JSA I
//  board minus the 6502, the YM2151 and the analogue tail.
//
//  Copyright (C) 2026 RetroShrimp.  GPL-3.0-or-later; see LICENSE and NOTICE.md.
//  Derived from the Xybots MiSTer core's xybots_jsa_bus.sv (GPL-3.0, same
//  author).
//
//  Bus timing: `ce` marks a valid 6502 bus cycle and plays the 2L PAL's O2
//  input.  Reads mux combinationally while the RAM/ROM read is registered (the
//  address is presented one ce early, as for a synchronous memory).
//
//  ---- direction comes from the 2L PAL -------------------------------------
//  blstroid_jsa_pal2l.sv is the fuse map of Atari 136056-2101, so /SWR and
//  /SRD are real nets here.  Two consequences:
//
//    * the RAM, the YM2151 and the ROM are gated with the strobes those chips
//      actually have on their /WE and /OE pins (2H /WE = /SWR, /OE = /SRD;
//      2K /CE = /ROM, /OE = /SRD; 2F /CS = /YAM).  /SWR is asserted for RAM
//      and YM writes only - a write anywhere in $2800-$2FFF or $3000-$FFFF
//      produces no write strobe at all.
//    * the 4M LS138's register strobes arrive from blstroid_jsa_decode already
//      O2- and direction-qualified (Y1..Y3 read, Y4..Y7 write), so
//      sel_rdp/sel_rdio/sel_irqack are true only during a read and
//      sel_voice/sel_wrp/sel_wrio/sel_mix only during a write.  Nothing here
//      re-ANDs them with `ce`.  /IRQACK is therefore read-only on hardware;
//      MAME also acknowledges on a write, but the shipping ROM only reads it.
//
//  ---- what an empty socket reads ------------------------------------------
//  0x2C00-0x2FFF is still decoded with the POKEY removed: the select is
//  CS0 = /REST (a PAL output) and CS1 = SA10, neither of which involves the
//  chip.  Nothing then drives SD7..SD0, and there are no pull-ups on the sound
//  data bus anywhere on SP-316 sheets 15-17 (the only pull-ups are R18/R28/R52
//  on the 6502's /NMI, RDY and /IRQ, R17 /SNDRES, R11 SELF-TEST, R74/R75/R77/
//  R78 on the coin lines, R36 POKEY SOD and R37 TMS5220), so a real board
//  returns floating open bus.  0xFF is used here: it matches MAME's explicit
//  pokey_r -> 0xff and it is where a floating TTL bus settles.  The Blasteroids
//  sound ROM never accesses that range.
//  0x2806 (/IRQACK) is the same open-bus situation and returns 0xFF here;
//  MAME's sound_irq_ack_r returns 0x00.  The ROM discards the byte in both of
//  its /IRQACK reads ($41BB and $41C9), so the difference is unobservable.
//  /VOICE (0x2A00) writes are accepted and kept: the 3F LS374 is stuffed, it
//  just drives an empty TMS5220 socket and the /MIX latch.
//============================================================================

module blstroid_jsa_bus #(
	parameter int CK4_DIV  = 1024,
	parameter int MAME_DIV = 14336
) (
	input  logic        clk,
	input  logic        reset,
	input  logic        ce,             // 6502 bus-cycle enable (1.7897725 MHz)
	input  logic        ce_jsa,         // JSA master enable    (3.579545  MHz)
	input  logic        mame_irq_model, // 1 = MAME's free-running periodic IRQ

	// ---- 6502 ----
	input  logic [15:0] cpu_addr,
	input  logic  [7:0] cpu_dout,       // 6502 write data
	input  logic        cpu_rnw,        // 1 = read
	output logic  [7:0] cpu_din,        // read data to the 6502
	output logic        cpu_irq,        // periodic timer OR YM2151 (wired-OR)
	input  logic        ym_irq,         // YM2151 timer IRQ (active high)
	output logic        cpu_nmi,        // command-pending NMI (SCOM FULL)
	output logic        cpu_reset,      // /SNDRES from the main CPU's /AUDRES

	// ---- external 64 KB sound ROM (core/rtl/mem) ----
	output logic [15:0] rom_addr,
	input  logic  [7:0] rom_data,

	// ---- main-CPU mailbox side ----
	input  logic        main_cmd_wr,
	input  logic  [7:0] main_cmd_data,
	input  logic        main_resp_rd,
	input  logic        main_sndrst_wr,
	output logic  [7:0] main_resp_data,
	output logic        main_irq,             // -> 68000 IRQ4
	output logic        main_to_sound_ready,  // FF9C01 D6 is its COMPLEMENT

	// ---- board inputs / outputs ----
	input  logic        coin1,          // JCOIN-4, cabinet RIGHT chute
	input  logic        coin2,          // JCOIN-3, cabinet LEFT chute
	input  logic        coin3,          // JCOIN-2 (unused by the harness)
	input  logic        coin4,          // JCOIN-1 (unused by the harness)
	input  logic        self_test,      // SW1 on this board, also to JSCOM-1
	output logic        ym_reset_n,
	output logic        coin_ctr1,
	output logic        coin_ctr2,
	output logic  [1:0] cpu_bank,

	// ---- YM2151 external core ----
	output logic        ym_cs,          // access strobe (this cycle)
	output logic        ym_a0,          // register select (SA0)
	output logic        ym_we,          // write strobe (ce-gated)
	output logic        ym_rd,          // read strobe  (ce-gated)
	output logic  [7:0] ym_dout,        // 6502 -> YM data
	input  logic  [7:0] ym_din,         // YM -> 6502 data

	// ---- /MIX write (to the audio mixer, 5F LS273B) ----
	output logic        mix_wr,
	output logic  [7:0] mix_data
);

	// ---- decode (2L PAL + the 4M LS138 + the POKEY chip select) ----
	// `ce` is the PAL's O2 and `cpu_rnw` its SR//W, so every Φ2- and
	// direction-qualified strobe below comes straight out of the part.
	wire sel_ram, sel_ym, sel_rdp, sel_rdio, sel_irqack, sel_voice,
	     sel_wrp, sel_wrio, sel_mix, sel_pokey, sel_bank, sel_rom, ym_reg;
	wire rom_ce, srd, swr, a13b, a12b;
	wire [3:0] pokey_reg;
	blstroid_jsa_decode u_dec (
		.addr(cpu_addr), .rnw(cpu_rnw), .phi2(ce), .bank(cpu_bank),
		.sel_ram(sel_ram), .sel_ym(sel_ym), .sel_bank(sel_bank), .sel_rom(sel_rom),
		.sel_pokey(sel_pokey),
		.sel_rdp(sel_rdp), .sel_rdio(sel_rdio), .sel_irqack(sel_irqack),
		.sel_voice(sel_voice), .sel_wrp(sel_wrp), .sel_wrio(sel_wrio), .sel_mix(sel_mix),
		.rom_ce(rom_ce), .srd(srd), .swr(swr), .a13b(a13b), .a12b(a12b),
		.ym_reg(ym_reg), .pokey_reg(pokey_reg) );

	// The POKEY register index is decoded on the board (SA3:SA0 straight into
	// the empty 3K socket) and answers to nothing here.
	wire unused = &{1'b0, pokey_reg};

	// ---- 3F 74LS374 (/VOICE) -> 5F 74LS273B (/MIX) ---------------------------
	// On SP-316 sheet 16 the 5F mixer latch takes its D inputs from VO7..VO0
	// (3F's Q outputs), not from SD7..SD0.  A mixer update is therefore a
	// two-write sequence - STA $2A00 loads 3F, STA $2A06 transfers it into 5F -
	// which is what the ROM does at all three of its /MIX sites ($42BC, $57EB,
	// $58E4).  MAME reaches the same result by taking the byte on the bus during
	// the $2A06 write, which for this ROM is the same accumulator value.
	// 3F has no /POR clear, so the real VO bus powers up undefined; 5F IS
	// cleared by /POR, so the mixer powers up fully muted either way.  `vo` is
	// reset to 0 here only for simulation determinism.
	logic [7:0] vo;
	always_ff @(posedge clk) begin
		if (reset)             vo <= 8'h00;
		else if (sel_voice)    vo <= cpu_dout;   // Y4: already write+Φ2 qualified
	end
	assign mix_wr   = sel_mix;                   // /MIX (0x2A06) clocks 5F
	assign mix_data = vo;                        // ... from 3F, not from the bus

	// ---- memory ----
	// The 2H work RAM's /WE is /SWR and its /CS1 is /RAM; the 2K's A13/A12 pins
	// are the PAL's A13B/A12B, which carry BA13/BA12 inside 0x3000-0x3FFF and
	// SA13/SA12 everywhere else - i.e. the 4 KB paging window, taken off the
	// part rather than re-derived from the address.
	wire [7:0] mem_dout;
	blstroid_jsa_mem u_mem (
		.clk(clk),
		.addr(cpu_addr), .din(cpu_dout), .we(swr & sel_ram),
		.sel_ram(sel_ram), .sel_bank(sel_bank), .sel_rom(sel_rom), .bank({a13b, a12b}),
		.rom_addr(rom_addr), .rom_data(rom_data),
		.dout(mem_dout) );

	// ---- SCOM mailbox ----
	wire [7:0] snd_cmd_data;
	wire       snd_nmi, snd_reset, snd2main_ready;
	blstroid_sound_comm u_comm (
		.clk(clk), .reset(reset),
		.main_cmd_wr(main_cmd_wr), .main_cmd_data(main_cmd_data), .main_resp_rd(main_resp_rd),
		.main_sndrst_wr(main_sndrst_wr), .main_resp_data(main_resp_data), .main_irq(main_irq),
		.main_to_sound_ready(main_to_sound_ready), .sound_to_main_ready(snd2main_ready),
		.snd_cmd_rd(sel_rdp), .snd_resp_wr(sel_wrp), .snd_resp_data(cpu_dout),
		.snd_cmd_data(snd_cmd_data), .snd_nmi(snd_nmi), .snd_reset(snd_reset) );

	// ---- I/O + banking + periodic IRQ ----
	wire [7:0] rdio_data;
	wire       sound_irq;
	blstroid_jsa_io #(.CK4_DIV(CK4_DIV), .MAME_DIV(MAME_DIV)) u_io (
		.clk(clk), .reset(reset), .ce_jsa(ce_jsa), .mame_irq_model(mame_irq_model),
		.wrio_wr(sel_wrio), .wrio_data(cpu_dout),
		.irqack(sel_irqack),         // /IRQACK: LS138 Y3, a READ of 0x2806 only
		.self_test(self_test), .nmi_line_n(~snd_nmi), .resp_full(snd2main_ready),
		.coin1(coin1), .coin2(coin2), .coin3(coin3), .coin4(coin4),
		.cpu_bank(cpu_bank), .coin_ctr1(coin_ctr1), .coin_ctr2(coin_ctr2),
		.ym_reset_n(ym_reset_n), .rdio_data(rdio_data), .sound_irq(sound_irq) );

	// ---- YM2151 strobes (2F: /CS = /YAM, A0 = SA0, /WR = /SWR, /RD = /SRD) ----
	assign ym_cs   = sel_ym;
	assign ym_a0   = ym_reg;
	assign ym_we   = sel_ym & swr;
	assign ym_rd   = sel_ym & srd;
	assign ym_dout = cpu_dout;

	// ---- 6502 read mux ----
	// Every source below is qualified by the strobe that actually enables its
	// output driver: the 2H's /OE and the 2K's /OE are both /SRD, the 2K's /CE
	// is /ROM (which the PAL already makes read-only), the YM2151's /RD is
	// /SRD, and the LS138's read half is enabled only when G1 = /SRD is high.
	always_comb begin
		if      (srd & sel_ram) cpu_din = mem_dout;      // 2H
		else if (rom_ce & srd)  cpu_din = mem_dout;      // 2K, 0x3000-0xFFFF
		else if (srd & sel_ym)  cpu_din = ym_din;        // 2F
		else if (sel_rdp)       cpu_din = snd_cmd_data;
		else if (sel_rdio)      cpu_din = rdio_data;
		// 0x2C00-0x2FFF is decoded but the POKEY socket (3K) is empty: nothing
		// drives SD7..SD0 and there are no pull-ups on the sound data bus, so
		// the real board floats.  Split out from the catch-all so the intent is
		// visible.
		else if (sel_pokey)     cpu_din = 8'hFF;
		else                    cpu_din = 8'hFF;   // /IRQACK, /VOICE, gaps
	end

	assign cpu_irq   = sound_irq | ym_irq;   // Q8 open collector + the YM's /IRQ
	                                         // sharing R52's pull-up
	assign cpu_nmi   = snd_nmi;
	assign cpu_reset = snd_reset;

endmodule
