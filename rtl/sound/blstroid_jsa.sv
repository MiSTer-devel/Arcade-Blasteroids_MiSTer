`timescale 1ns/1ps
//============================================================================
//  Blasteroids (Atari Games, 1987) - the complete JSA I audio PCB
//  (Atari drawing 043713-xx rev B, schematics SP-316 sheets 14-17).
//
//  Copyright (C) 2026 RetroShrimp.  GPL-3.0-or-later; see LICENSE and NOTICE.md.
//  Derived from the Xybots MiSTer core's xybots_jsa.sv (GPL-3.0, same author,
//  itself from Toobin').
//
//  6502 (T65) + blstroid_jsa_bus (decode / memory / I/O / SCOM mailbox) +
//  YM2151 (jt51) + the sheet-16 /MIX mixer + the sheet-16 output low-pass.
//  One module, instantiated by blstroid_core.
//
//  ---- what is particular about Blasteroids' JSA I -------------------------
//  This is the same generic Stand-Alone Audio PCB that Xybots, Toobin',
//  Vindicators and Escape carry; which sockets are stuffed is a per-game build
//  decision, and the drawing carries no "not loaded" marks.  On Blasteroids:
//
//   * No POKEY (3K) and no TMS5220C (3D).  Everything around them is still on
//     the board - the /VOICE latch (3F LS374), the POKEY chip select, the
//     speech and POKEY volume bits of /MIX, the PS summing network and the
//     CT1/CT2 stereo gates - so they are decoded here and drive nothing.
//     Reads of the empty POKEY socket return 0xFF; see blstroid_jsa_bus.
//   * The coin harness has no crossover: COIN1 lands on /RDIO D0.  On the
//     cabinet, though, COIN1 is the RIGHT chute - an input-mapping caveat, not
//     a wiring one.
//   * The periodic-IRQ divider is self-clearing.  Sheet 17 feeds the IRQ
//     flip-flop's Q back into the last LS393's CLR, so the divider is
//     re-anchored on every /IRQACK and the interrupt rate depends on how
//     quickly the handler acknowledges (about 257.7 Hz for this ROM).  MAME
//     instead free-runs at 249.6893 Hz; mame_irq_model selects that model.
//   * The /MIX latch is loaded from the /VOICE latch, not from the data bus.
//     Modelled in blstroid_jsa_bus.
//   * All four sound-ROM banks are used.
//   * Self test is SW1 on THIS board, and it fans out both to /RDIO D7 here
//     and, over JSCOM-1 -> AUD-1, to the main board's FF9C01 D7.  One logical
//     input, active high = "self test requested".
//
//  Clock enables (only two, and both exact integer divisions): clk_sys is
//  4 x the 14.318181 MHz video crystal and the JSA crystal is that crystal / 4,
//  so blstroid_core supplies
//     ce_6502  = clk_sys/32 = 1.7897725 MHz  (6502, jt51 cen_p1, the LPF)
//     ce_ym    = clk_sys/16 = 3.579545  MHz  (jt51 cen, the periodic divider)
//  with ce_6502 a strict subset of ce_ym - the cen/cen_p1 relationship jt51
//  needs.  No fractional clock-enable generator exists on this board.
//
//  The 64 KB sound program ROM (2K, 136057-1135.2k) is NOT inside this module.
//  It is one BRAM in rtl/mem, shared with the download loader and read here
//  through snd_rom_addr / snd_rom_data: the address is combinational and the
//  data is expected back one clk later.  The 8 KB work RAM (2H) IS inside, in
//  blstroid_jsa_mem.
//
//  T65 is instantiated as T65_wrap (a VHDL wrapper, for Quartus).
//============================================================================

module blstroid_jsa #(
	parameter int CK4_DIV  = 1024,   // ce_ym per CK4 edge (3579545/4/16/16)
	parameter int MAME_DIV = 14336   // ce_ym per free-running IRQ (= 14 x CK4)
) (
	input  logic        clk,
	input  logic        reset,
	// clock enables
	input  logic        ce_6502,
	input  logic        ce_ym,
	// sound ROM read port -> the 64 KB BRAM in core/rtl/mem (loader-filled)
	output logic [15:0] snd_rom_addr,          // combinational
	input  logic  [7:0] snd_rom_data,          // registered, 1 clk later
	// main-CPU mailbox (SCOM 9A master <-> 1M slave)
	input  logic        main_cmd_wr,           // 68000 write to FF8A01, D7:0
	input  logic  [7:0] main_cmd_data,
	input  logic        main_resp_rd,          // 68000 read of FF9401 (acks IRQ4)
	input  logic        main_sndrst_wr,        // 68000 write to FF8C00
	output logic  [7:0] main_resp_data,
	output logic        main_irq4,             // -> 68000 /IPL2 => IRQ4 (level)
	output logic        main_busy_n,           // -> FF9C01 D6 (0 = busy)
	output logic        main_to_sound_ready,   // the raw flag (= ~main_busy_n)
	// board inputs
	input  logic        coin1,                 // JCOIN-4 -> /RDIO D0 (RIGHT chute)
	input  logic        coin2,                 // JCOIN-3 -> /RDIO D1 (LEFT chute)
	input  logic        coin3,                 // JCOIN-2 -> /RDIO D2
	input  logic        coin4,                 // JCOIN-1 -> /RDIO D3
	input  logic        self_test,             // SW1 / JSCOM-1, active high
	// board outputs
	output logic        coin_ctr1,
	output logic        coin_ctr2,
	// options
	input  logic        mame_irq_model,        // 1 = free-running 249.6893 Hz
	input  logic        lpf_bypass,            // 1 = skip the sheet-16 low-pass
	// audio (stereo: CH2 -> LAUD -> SPKR1 -> JSPK-5, CH1 -> RAUD -> SPKR2)
	output logic signed [15:0] aud_left,
	output logic signed [15:0] aud_right
);

	// ---- 6502 <-> bus ----
	wire [15:0] cpu_a; wire [7:0] cpu_do, cpu_di; wire cpu_rwn;
	wire cpu_irq, cpu_nmi, cpu_reset;
	wire [7:0] dbgA_nc, dbgX_nc, dbgY_nc, dbgS_nc, dbgP_nc; wire cpu_sync_nc;
	T65_wrap u_cpu (
		// Rdy = 1: no wait states.  The board's wait-state PAL (2M-FPAL) is not
		// populated on Blasteroids, so the 6502's RDY pin sits on its pull-up.
		.Mode(2'b00), .Res_n(~(reset | cpu_reset)), .Clk(clk), .Enable(ce_6502), .Rdy(1'b1),
		.IRQ_n(~cpu_irq), .NMI_n(~cpu_nmi), .DI(cpu_di),
		.A(cpu_a), .DO(cpu_do), .R_W_n(cpu_rwn), .Sync(cpu_sync_nc),
		.dbg_A(dbgA_nc), .dbg_X(dbgX_nc), .dbg_Y(dbgY_nc), .dbg_S(dbgS_nc), .dbg_P(dbgP_nc) );

	// ---- bus (decode + memory + I/O + SCOM mailbox) ----
	wire ym_reset_n; wire [1:0] cpu_bank_nc;
	wire ym_cs, ym_a0, ym_we, ym_rd; wire [7:0] ym_dout, ym_din;
	wire mix_wr; wire [7:0] mix_data;
	wire ym_irq_w; wire m2s_ready;
	blstroid_jsa_bus #(.CK4_DIV(CK4_DIV), .MAME_DIV(MAME_DIV)) u_bus (
		// ce_jsa is the periodic-IRQ divider enable: the LS393 cascade counts
		// master-clock ticks, so it must be ce_ym and not the half-rate ce_6502.
		.clk(clk), .reset(reset), .ce(ce_6502), .ce_jsa(ce_ym),
		.mame_irq_model(mame_irq_model),
		.cpu_addr(cpu_a), .cpu_dout(cpu_do), .cpu_rnw(cpu_rwn), .cpu_din(cpu_di),
		.cpu_irq(cpu_irq), .ym_irq(ym_irq_w), .cpu_nmi(cpu_nmi), .cpu_reset(cpu_reset),
		.rom_addr(snd_rom_addr), .rom_data(snd_rom_data),
		.main_cmd_wr(main_cmd_wr), .main_cmd_data(main_cmd_data), .main_resp_rd(main_resp_rd),
		.main_sndrst_wr(main_sndrst_wr), .main_resp_data(main_resp_data), .main_irq(main_irq4),
		.main_to_sound_ready(m2s_ready),
		.coin1(coin1), .coin2(coin2), .coin3(coin3), .coin4(coin4), .self_test(self_test),
		.ym_reset_n(ym_reset_n), .coin_ctr1(coin_ctr1), .coin_ctr2(coin_ctr2),
		.cpu_bank(cpu_bank_nc),
		.ym_cs(ym_cs), .ym_a0(ym_a0), .ym_we(ym_we), .ym_rd(ym_rd),
		.ym_dout(ym_dout), .ym_din(ym_din),
		.mix_wr(mix_wr), .mix_data(mix_data) );

	// FF9C01 D6 (/AUDIO BUSY) is the complement of the command-pending flag:
	// SCOM 9A pin 5 BUSY, where 0 means busy.
	assign main_to_sound_ready = m2s_ready;
	assign main_busy_n         = ~m2s_ready;

	// ---- YM2151 (jt51) ----
	wire signed [15:0] ym_l, ym_r, ym_l_lo_nc, ym_r_lo_nc;
	wire ym_sample_nc, ym_ct1_nc, ym_ct2_nc;
	blstroid_ym u_ym (
		.clk(clk), .reset(reset), .cen(ce_ym), .cen_p1(ce_6502), .ym_reset_n(ym_reset_n),
		.ym_cs(ym_cs), .ym_a0(ym_a0), .ym_we(ym_we), .ym_dout(ym_dout), .ym_din(ym_din),
		.sample(ym_sample_nc), .aud_left(ym_l), .aud_right(ym_r),
		.aud_left_lo(ym_l_lo_nc), .aud_right_lo(ym_r_lo_nc),
		.ct1(ym_ct1_nc), .ct2(ym_ct2_nc), .ym_irq(ym_irq_w) );

	// CT1/CT2 gate the PS (POKEY + speech) sum into the left/right analogue
	// paths on SP-316 sheet 16 (R65 12K through a 4066B gated by CT1 into the
	// LEFT channel, R66 12K gated by CT2 into the RIGHT).  Both of those sockets
	// are empty on Blasteroids, so PS is silent and the gates have nothing to
	// switch - they are read here and discarded rather than left dangling.
	// Likewise ym_rd (the YM2151 is read through the same combinational path as
	// any other source), the ROM bank output, the T65 debug ports and jt51's
	// low-resolution audio.
	wire _unused_jsa = &{1'b0, ym_rd, cpu_bank_nc, cpu_sync_nc,
		dbgA_nc, dbgX_nc, dbgY_nc, dbgS_nc, dbgP_nc, ym_sample_nc,
		ym_ct1_nc, ym_ct2_nc, ym_l_lo_nc, ym_r_lo_nc};

	// ---- /MIX audio mixer ----
	wire signed [15:0] mix_l, mix_r; wire lpf_engage;
	blstroid_jsa_mix u_mix (
		.clk(clk), .reset(reset), .mix_wr(mix_wr), .mix_data(mix_data),
		.ym_left(ym_l), .ym_right(ym_r),
		.out_left(mix_l), .out_right(mix_r), .lpf_engage(lpf_engage) );

	// ---- output low-pass (SP-316 sheet 16) ----
	// LM324 4A Sallen-Key on each channel, with Q5/Q6 switching C54/C56 in.
	//
	// Stereo routing: 2D YM3012 pin 9 (CH2, from the YM2151's SH2) -> the
	// R62/R64/R63 ladder -> R44 -> this filter -> LAUD -> SPKR1 -> JSPK-5 ->
	// LS1; pin 10 (CH1, SH1) -> R69/R67/R68 -> R47 -> RAUD -> SPKR2 -> JSPK-2
	// -> LS2.  The board is genuinely stereo.  SP-316 does not say which of
	// jt51's xleft / xright corresponds to the part's SH1 vs SH2, and jt51 has
	// no SH pins to disambiguate, so xleft -> LAUD/SPKR1 is shipped, matching
	// MAME's stream 0 -> left and the LAUD/RAUD net names.
	blstroid_jsa_lpf u_lpf_l (
		.clk(clk), .reset(reset), .ce(ce_6502), .engage(lpf_engage), .bypass(lpf_bypass),
		.in(mix_l), .out(aud_left) );
	blstroid_jsa_lpf u_lpf_r (
		.clk(clk), .reset(reset), .ce(ce_6502), .engage(lpf_engage), .bypass(lpf_bypass),
		.in(mix_r), .out(aud_right) );

endmodule
