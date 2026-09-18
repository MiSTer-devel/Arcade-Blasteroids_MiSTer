`timescale 1ns/1ps
//============================================================================
//  Blasteroids (Atari Games, 1987) - JSA I board I/O: the 5H /WRIO output
//  register, the 5J /RDIO input port, ROM banking, and the sheet-17 periodic
//  sound interrupt.
//
//  Copyright (C) 2026 RetroShrimp.  GPL-3.0-or-later; see LICENSE and NOTICE.md.
//  Derived from the Xybots MiSTer core's xybots_jsa_io.sv (GPL-3.0, same
//  author).  This is the module that differs most from that copy, in two ways:
//    1. the periodic-IRQ divider is not free-running.  SP-316 sheet 17 feeds
//       the IRQ flip-flop's Q back into the last LS393's CLR, so the divider is
//       re-anchored on every /IRQACK.  It is implemented gate for gate below,
//       with a mame_irq_model input that selects MAME's free-running
//       249.6893 Hz timer instead;
//    2. there is no coin swap.  The Blasteroids harness is drawn end to end on
//       SP-316 sheets 13 and 19 with no crossover: COIN1 -> /RDIO D0, COIN2 ->
//       /RDIO D1.  (On the cabinet, COIN1 is the RIGHT chute - a naming caveat
//       for the MiSTer input map, not a wiring one.)
//
//  ---- /WRIO output register (SP-316 sheet 15, 5H 74LS273A) ----------------
//  CK(11) = /WRIO, /CLR(1) = /POR (R103 150K + C84 0.1 uF), so every bit powers
//  up at 0 - including /YAMRES, i.e. the YM2151 is held in reset until the
//  6502's first /WRIO write.  The Blasteroids sound ROM's very first bus cycle
//  is STA $2A04 with 0x01 at pc = $4007, which releases it.
//
//    SD7 -> BA13     ROM bank bit 1   (all four banks are used)
//    SD6 -> BA12     ROM bank bit 0
//    SD5 -> CCTR2    coin counter 2 (R53 -> Q10 2N5306 -> CC2-)
//    SD4 -> CCTR1    coin counter 1 (R54 -> Q9  2N5306 -> CC1-)
//    SD3 -> SQUEAK   TMS5220 clock-divider select (4D LS169)  socket empty
//    SD2 -> /SPHRES  TMS5220 RS                                socket empty
//    SD1 -> /SPHWR   TMS5220 WS                                socket empty
//    SD0 -> /YAMRES  YM2151 /IC (active low: 1 = run)
//  MAME's wrio_w agrees bit for bit.  The ROM holds /SPHRES asserted forever,
//  so the speech strobes are inert here by construction.
//
//  ---- /RDIO input port (SP-316 sheet 17, 5J 74LS240A) ---------------------
//  The LS240A is an INVERTING buffer, enabled by /RDIO on /1G(1) and /2G(19),
//  so a logically asserted (low) input reads back as 1:
//
//    SD7 = ~SELF-TEST   SW1 on THIS board (R11 1K pull-up), also to JSCOM-1 ->
//                       AUD-1 -> the main board's $FF9C01 D7
//    SD6 = ~(/NMI)      the slave SCOM's FULL output
//    SD5 = ~SFULL       the slave SCOM's BUSY output
//    SD4 = ~(/SPHRDY)   TMS5220 RDY                (socket empty -> reads 0)
//    SD3 = ~COIN4       JCOIN-1, R76 1K            (a real polled input)
//    SD2 = ~COIN3       JCOIN-2, R81 1K
//    SD1 = ~COIN2       JCOIN-3, R79 1K            LEFT chute
//    SD0 = ~COIN1       JCOIN-4, R80 1K            RIGHT chute
//  Each coin line is pulled up (R74/R75/R77/R78 1K) and filtered (C62-C65
//  0.1 uF); a closed switch pulls it to JCOIN-6 = GND, so after the inverter a
//  closed switch reads as 1.
//
//  Three points where the schematic and MAME disagree.  The schematic is
//  followed in all three, and none of them is observable for this game:
//   * D7 reads 0 in MAME, whose rdio_r applies the self-test inversion twice,
//     making the JSA's own sound self test unreachable there.  This RTL drives
//     D7 from the real SW1 line.
//   * D6: the LS240A inverts the physical /NMI, so the 6502 reads 1 while a
//     command is pending; MAME declares the lane active low.  The Blasteroids
//     ROM never tests bit 6.
//   * D3 (COIN4) is a real input that MAME marks unused.  The coin scan at
//     $5AD8/$5AFC is an LSR walk over D0..D3 with a four-entry index, so it is
//     polled; coin3/coin4 are therefore ports, defaulting inactive.
//  D5 is the one flag the ROM actually tests ($4152, $56D3: LDA #$20 / BIT
//  $280C / BNE skips the "send a response" path while the bit is set), which
//  confirms the un-inverted reading of resp_full = "response latch full".
//
//  ---- the periodic interrupt (SP-316 sheet 17) ----------------------------
//
//    894.886 kHz (= 3579545/4)
//      -> 6J LS393 half 1 : QD = /16  ->  55 930.39 Hz
//      -> 6J LS393 half 2 : QD = /16  ->   3 495.649 Hz    [CK4]
//      -> 6K LS393 half 2 : A(13) = CK4,  C(12) = CLR  <-- feedback
//            QB(10) -> 5K LS74 #1 CK(11)
//            QC(9)  -> 5K LS74 #1 D (12)
//            QD(8)  -> 5K LS74 #2 D (2)
//      5K LS74 #1 : PRE(10)=PR2, CLR(13)=PR2  (both permanently inactive)
//                   Q(9) -> 5K LS74 #2 CK(3)
//      5K LS74 #2 : PRE(4)=PR2, CLR(1)=/IRQACK
//                   Q(5) -> R51 10K -> Q8 2N3904 (open collector) -> /IRQ
//                   Q(5) -> CLR(12) of the last LS393            <-- feedback
//
//  Every earlier LS393 CLR is tied to the GND bus; only the last stage's comes
//  from the IRQ flip-flop.  Walking the ripple counter from cleared, QB rises
//  at N = 2, 6, 10, 14; LS74 #1 samples QC there (0,1,0,1) so its Q rises at
//  N = 6 and 14; LS74 #2 samples QD on those rises (0 at 6, 1 at 14).  So the
//  flag sets on the 14th count - the same /4/16/16/14 ratio MAME uses - and its
//  Q then holds the counter cleared until the 6502 reads $280E.
//
//    T = 1/3495.6486 Hz = 286.0704 us
//    free-running:  14*T = 4004.985 us = 249.6893 Hz  (MAME's model)
//    this board:    t_ack + (T - phase) + 13*T,  phase uniform in [0,T)
//                   mean = t_ack + 13.5*T = t_ack + 3861.95 us
//  The handler reaches LDA $280E at $41C9 about 20-25 6502 cycles (~13 us)
//  after entry, so the real mean period is roughly 3880 us, about 257.7 Hz -
//  some 3.2 % faster than the free-running model.  mame_irq_model switches in a
//  plain free-running MAME_DIV counter for comparison against that reference.
//
//  /IRQACK is level-dominant on LS74 #2's asynchronous CLR: an edge arriving
//  while /IRQACK is asserted cannot set Q.  Both models keep that ordering.
//
//  The YM2151's own /IRQ (2F pin 2) shares R52's pull-up and is NOT cleared by
//  /IRQACK; the wire-OR happens in blstroid_jsa_bus.
//
//  ---- parameters ----------------------------------------------------------
//  CK4_DIV  ce_ym ticks per CK4 edge.  3.579545 MHz / 4 / 16 / 16 = 1024.
//  MAME_DIV ce_ym ticks per free-running interrupt = 14 * CK4_DIV = 14336.
//  These are the hardware ratios; only a simulation may override them.
//============================================================================

module blstroid_jsa_io #(
	parameter int CK4_DIV  = 1024,
	parameter int MAME_DIV = 14336
) (
	input  logic        clk,
	input  logic        reset,
	input  logic        ce_jsa,        // JSA master-clock enable (3.579545 MHz)

	// interrupt-model select: 0 = the board (self-clearing divider, sheet 17),
	// 1 = MAME's free-running 249.6893 Hz timer.
	input  logic        mame_irq_model,

	// /WRIO write (0x2A04)
	input  logic        wrio_wr,
	input  logic  [7:0] wrio_data,
	// /IRQACK strobe (0x2806, read or write) clears the periodic IRQ
	input  logic        irqack,

	// input-port sources (all logical "asserted" = 1)
	input  logic        self_test,     // SW1 on this board / JSCOM-1
	input  logic        nmi_line_n,    // physical /NMI, before the LS240 inverts it
	input  logic        resp_full,     // sound->main response pending (SFULL)
	input  logic        coin1,         // JCOIN-4 -> SD0 (cabinet RIGHT chute)
	input  logic        coin2,         // JCOIN-3 -> SD1 (cabinet LEFT chute)
	input  logic        coin3,         // JCOIN-2 -> SD2, unused by the harness
	input  logic        coin4,         // JCOIN-1 -> SD3, unused by the harness

	// outputs
	output logic  [1:0] cpu_bank,      // BA13:BA12 -> the 0x3000-0x3FFF window
	output logic        coin_ctr1,
	output logic        coin_ctr2,
	output logic        ym_reset_n,    // /YAMRES (active low: 1 = run)
	output logic  [7:0] rdio_data,     // assembled /RDIO byte
	output logic        sound_irq      // level, into the 6502 /IRQ wired-OR
);

	// bits [3:1] drive the (unpopulated) TMS5220 strobes /SPHRES and /SPHWR and
	// the 4D LS169 SQUEAK divider.  They are latched by the real 5H and reach an
	// empty socket; nothing here can observe them.
	wire unused_tms = &{1'b0, wrio_data[3:1]};

	// ---- /WRIO output latch (5H LS273A, cleared by /POR) ----
	always_ff @(posedge clk) begin
		if (reset) begin
			cpu_bank <= 2'd0; coin_ctr1 <= 1'b0; coin_ctr2 <= 1'b0; ym_reset_n <= 1'b0;
		end else if (wrio_wr) begin
			cpu_bank   <= wrio_data[7:6];
			coin_ctr2  <= wrio_data[5];
			coin_ctr1  <= wrio_data[4];
			ym_reset_n <= wrio_data[0];
		end
	end

	// ---- /RDIO input assembly (5J LS240A, inverting) ----
	// No harness crossover on Blasteroids: COIN1 -> SD0, COIN2 -> SD1.
	// SD4: /SPHRDY floats at the empty TMS5220 socket, so the inverting LS240A
	// drives 0.  MAME reaches the same value via `result &= ~0x10`.
	assign rdio_data = { self_test, ~nmi_line_n, resp_full, 1'b0,
	                     coin4, coin3, coin2, coin1 };

	// ---- CK4: the two free-running 6J LS393 halves ----
	// Their CLR pins are on the GND bus, so this prescaler free-runs in BOTH
	// interrupt models and is never re-anchored by anything.
	localparam int PW = (CK4_DIV > 1) ? $clog2(CK4_DIV) : 1;
	logic [PW-1:0] ck4_div;
	logic          ce_ck4;
	always_ff @(posedge clk) begin
		if (reset) begin
			ck4_div <= '0; ce_ck4 <= 1'b0;
		end else begin
			ce_ck4 <= 1'b0;
			if (ce_jsa) begin
				if (ck4_div == PW'(CK4_DIV-1)) begin ck4_div <= '0; ce_ck4 <= 1'b1; end
				else                                 ck4_div <= ck4_div + 1'b1;
			end
		end
	end

	// ---- model A: the board (6K LS393 half + the 5K LS74 pair) ----
	// div_n  = the last LS393 half's QD..QA, held cleared while ff2 is set
	// ff1    = 5K LS74 #1 (D = QC, CK = QB, no clear)
	// ff2    = 5K LS74 #2 (D = QD, CK = ff1, CLR = /IRQACK) -> Q8 -> /IRQ
	logic [3:0] div_n;
	logic       ff1, ff2;

	always_ff @(posedge clk) begin
		logic [3:0] div_nxt;
		logic       qb_rise, ff1_nxt, ff2_nxt;
		if (reset) begin
			div_n <= 4'd0; ff1 <= 1'b0; ff2 <= 1'b0;
		end else begin
			div_nxt = div_n;
			qb_rise = 1'b0;
			ff1_nxt = ff1;
			ff2_nxt = ff2;
			if (ce_ck4) begin
				// ff2's Q on the LS393 CLR: while it is set the counter is held
				// at 0 and QB never moves, so ff1 holds too.
				div_nxt = ff2 ? 4'd0 : (div_n + 4'd1);
				qb_rise = ~div_n[1] & div_nxt[1];
				if (qb_rise) begin
					ff1_nxt = div_nxt[2];                       // D = QC
					if (~ff1 && ff1_nxt) ff2_nxt = div_nxt[3];  // ff1 rising: D = QD
				end
			end
			// /IRQACK is the asynchronous CLR and wins a coincidence.
			if (irqack) ff2_nxt = 1'b0;
			// ... and the CLR feedback is asynchronous too: the count is wiped
			// in the same instant Q goes high.
			div_n <= ff2_nxt ? 4'd0 : div_nxt;
			ff1   <= ff1_nxt;
			ff2   <= ff2_nxt;
		end
	end

	// ---- model B: MAME's free-running periodic timer ----
	// `atari_jsa_base_device` arms a plain 3579545/4/16/16/14 = 249.6893 Hz
	// timer that ignores /IRQACK entirely (the ack only clears the CPU line).
	localparam int MW = $clog2(MAME_DIV);
	logic [MW-1:0] mame_cnt;
	logic          mame_irq;
	always_ff @(posedge clk) begin
		if (reset) begin
			mame_cnt <= '0; mame_irq <= 1'b0;
		end else begin
			if (ce_jsa) begin
				if (mame_cnt == MW'(MAME_DIV-1)) begin mame_cnt <= '0; mame_irq <= 1'b1; end
				else                                   mame_cnt <= mame_cnt + 1'b1;
			end
			if (irqack) mame_irq <= 1'b0;   // async /CLR, dominant
		end
	end

	assign sound_irq = mame_irq_model ? mame_irq : ff2;

endmodule
