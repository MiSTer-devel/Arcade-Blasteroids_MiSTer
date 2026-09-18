`timescale 1ns/1ps
//============================================================================
//  Blasteroids interrupt latches and priority encoder — SP-316 sheet 2
//  (11C LS11, 1D LS20, 13C LS74, 13D LS10, 11B F00).
//
//  Copyright (C) 2026 RetroShrimp.  GPL-3.0-or-later; see LICENSE and
//  NOTICE.md.  Structure from the Xybots MiSTer core's rtl/main/xybots_irq.sv
//  (GPL-3.0, same author); the equations are read off SP-316 sheets 2 and 6.
//
//  ---- IRQ1, the row-programmable playfield interrupt --------------------
//  Real hardware, fully wired, and the one thing MAME stubs out:
//
//    11C LS11 (3,4,5 -> 6)     = 1V & 2V & 4V
//    1D  LS20 (1,2,4,5 -> 6)   = ~( 11C/6 & PR4 & PFM2 & /VBLANK )
//    13C LS74 upper: D (12) = 1D/6      CK (11) = HBLANK
//                    /PRE (10) = /IRQACK   /CLR (13) = PR5 (inactive)
//                    /Q (8) -> 13D LS10 pin 10
//
//  and PFM2 is SP-316 sheet 6, 4K LS174B: D (3) = VD15, Q (2) = PFM2, CK = 4H
//  — bit 15 of the last playfield word the video side fetched, which for the
//  non-displayed column-40 word of a row is sheet 12's ROW PROGRAMMABLE
//  INTERRUPT flag.  It arrives here as an input from the video block.
//
//  Behaviour: on every rising edge of HBLANK the latch samples
//  ~(V[2:0] == 7 & PFM2 & ~VBLANK).  On the last line of an 8-line stamp row,
//  outside vertical blanking, with that row's flag set, D = 0 is clocked in ->
//  /Q = 1 -> IRQ1.  The next HBLANK edge that does not meet the condition
//  clocks D = 1 back in, so the request is one scan line wide and clears
//  itself.  That matters: the rev-4 ROM never writes FF8200 (/IRQACK), so a
//  latch that needed the acknowledge would take one IRQ1 and then spin in the
//  handler forever.  /IRQACK is still implemented, because sheet 3 decodes it.
//
//  PR4 in the 1D LS20 term is a 1 K pull-up strap (R50) with no jumper pads
//  drawn, so it is permanently 1 and the feature cannot be disabled.
//
//  ---- IRQ2, VBLANK ------------------------------------------------------
//    13C LS74 lower: D (2) = GND    CK (3) = VBLANK
//                    /PRE (4) = /VBLACK   /CLR (1) = PR5 (inactive)
//                    /Q (6) -> 11B F00 pin 13
//  The rising edge of VBLANK clocks 0 into Q; a write to FF8400 presets it.
//
//  ---- IRQ4, the audio board ---------------------------------------------
//  /IPL2 is the SCOM's RES output (9A pin 16) directly: it goes low when the
//  audio board has posted a response byte and is cleared by the FF9401 read
//  itself.  There is NO acknowledge register.  `snd_irq` here is that level,
//  active high, from the sound mailbox.
//
//  ---- the encoder -------------------------------------------------------
//    13D LS10 (9,10,11 -> 8):  /IPL0 = ~( /IPL2 & /Q1 & /IPL1 )
//    11B F00  (12,13 -> 11):   /IPL1 = ~( /IPL2 & /Q2 )
//  Both latches are preset by their acknowledge, so Q = 1 (/Q = 0) is "idle".
//  The gates give the priority for free: /IPL2 low forces both NANDs high, so
//  level 4 wins; a pending VBLANK forces /IPL0 high, so level 2 beats level 1.
//  There is no level 3, 5, 6 or 7 on this board.
//
//    condition                /IPL2 /IPL1 /IPL0   level
//    idle                       1     1     1       0
//    VBLANK latched             1     0     1       2
//    row latched                1     1     0       1
//    audio                      0     1     1       4
//
//  ---- reset ---------------------------------------------------------------
//  Neither 13C half has a usable /CLR (both are strapped to PR5), so a
//  watchdog timeout or a 68000 RESET instruction leaves a pending interrupt
//  pending.  Drive `reset` from power-up only, never from the /RESET net — the
//  CPU comes out of reset with the SR mask at 7 in any case.
//============================================================================

module blstroid_irq
(
	input  logic       clk,
	input  logic       reset,       // power-up only — see the header

	// ---- video side ----
	input  logic       pfm2,        // sheet 6, 4K LS174B pin 2 (latched VD15)
	input  logic [2:0] v,           // {4V, 2V, 1V}
	input  logic       hblank,      // active high
	input  logic       vblank,      // active high

	// ---- CPU side ----
	input  logic       irqack_wr,   // 1-clk pulse: any write to FF8200
	input  logic       vblack_wr,   // 1-clk pulse: any write to FF8400
	input  logic       snd_irq,     // SCOM /RES asserted (active high here)

	output logic [2:0] ipl,         // {IPL2,IPL1,IPL0}, active low, 111 = none
	output logic       irq1_pending,// 13C upper /Q  (1 = level 1 requested)
	output logic       irq2_pending // 13C lower /Q  (1 = level 2 requested)
);

	// ---- 11C LS11 + 1D LS20 ----------------------------------------------
	wire d_row = ~((v == 3'd7) & pfm2 & ~vblank);   // PR4 = 1

	// ---- 13C LS74 upper (row) --------------------------------------------
	logic q_row, hblank_d;
	always_ff @(posedge clk) begin
		if (reset) begin
			q_row    <= 1'b1;
			hblank_d <= 1'b0;
		end else begin
			hblank_d <= hblank;
			// /PRE is a level on the part and dominates the clock.
			if (irqack_wr)                q_row <= 1'b1;
			else if (hblank & ~hblank_d)  q_row <= d_row;
		end
	end

	// ---- 13C LS74 lower (VBLANK) -----------------------------------------
	logic q_vbl, vblank_d;
	always_ff @(posedge clk) begin
		if (reset) begin
			q_vbl    <= 1'b1;
			vblank_d <= 1'b0;
		end else begin
			vblank_d <= vblank;
			if (vblack_wr)               q_vbl <= 1'b1;
			else if (vblank & ~vblank_d) q_vbl <= 1'b0;   // D = GND
		end
	end

	assign irq1_pending = ~q_row;
	assign irq2_pending = ~q_vbl;

	// ---- 13D LS10 / 11B F00 ----------------------------------------------
	wire ipl2_n = ~snd_irq;
	wire ipl1_n = ~(ipl2_n & irq2_pending);
	wire ipl0_n = ~(ipl2_n & irq1_pending & ipl1_n);
	assign ipl  = {ipl2_n, ipl1_n, ipl0_n};

endmodule
