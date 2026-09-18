`timescale 1ns/1ps
//============================================================================
//  Blasteroids /DTACK generator — SP-316 sheet 2
//  (12C LS163A + 11C LS11 + 11D LS74 + 14/15B LS74 upper + 16A LS260).
//
//  Copyright (C) 2026 RetroShrimp.  GPL-3.0-or-later; see LICENSE and
//  NOTICE.md.  Derived from the Xybots MiSTer core's rtl/main/xybots_dtack.sv
//  (GPL-3.0, same author), re-done gate for gate off SP-316 sheet 2.
//
//  ---- the NOR that makes /DTACK -----------------------------------------
//    16A LS260 (5-input NOR, pins 10/11 grounded), output pin 6:
//        /DTACK = ~( RCO(12C) | /Q(11D lower) | /Q(11D upper) )
//    so DTACK (active high here) = RCO | vidram_ack | waithbl_ack.
//
//  ---- 12C LS163A, the wait-state counter --------------------------------
//        CK  (2) = /7M                     (the 68000 clock)
//        CLR (1) = 11C LS11 pin 12 = /WAITHBL & /VIDRAM & AS   (sync clear)
//        LD  (9) = QD (11)                 (self-load feedback)
//        A(3)=PR5=1  B(4)=/EEROM  C(5)=PR5=1  D(6)=PR5=1
//        ENT (10) = /VPA                   -> RCO forced low during an IACK
//        ENP (7)  = /DTACK                 -> counting stops once DTACK is out
//        RCO (15) = (count == 15) & ENT
//
//  Preload = {D,C,B,A} = {1,1,/EEROM,1} = 15 normally, 13 for the EEPROM.
//  A normal cycle therefore loads 15 on the first /7M edge after AS and RCO is
//  immediate — one /7M edge of latency, a 4-clock 68000 bus cycle with no wait
//  states.  An /EEROM cycle loads 13 and has to count 14, 15, i.e. two extra
//  /7M periods (~280 ns) for the X2804A-45's access time.  During an
//  interrupt-acknowledge cycle /VPA = 0 forces RCO low, so the acknowledge can
//  only ever terminate through the 68000's own /VPA (autovector) path — get
//  this wrong and the core hangs on the first VBLANK, because the ROM's vector
//  table stops at vector 63 and there is no vector hardware on the board.
//
//  ---- 11D LS74 lower, the video-RAM acknowledge -------------------------
//        D (2) = /VIDRAM      CK (3) = 14/15B LS74 upper Q (pin 5)
//        /PRE (4) = AS        /CLR (1) = PR5 (inactive)     /Q (6) -> 16A
//  and the clock chain, all sheet 2:
//        g12B = 12B LS32 (9,10->8)     = /1H | 2H
//        g13D = 13D LS10 (1,2,13->12)  = ~( /1H & 2H & 4H )
//        g14D = 14D LS00 (4,5->6)      = ~( g12B & g13D )   -> h = 1, 5, 6
//        g14B = 14B LS08 (1,2->3)      = 7M & g14D
//        14/15B upper: D = g14B, CK = 14M, /PRE = PR5, /CLR = /VPA
//  so the acknowledge flip-flop is clocked once per H-counter group at
//  h = {4H,2H,1H} = 1, 5 and 6 — each exactly one count ahead of the three CPU
//  video-RAM slots at h = 2, 6, 7 (see blstroid_main).  While /AS is inactive
//  the flip-flop is preset (Q = 1, /Q = 0, no acknowledge); during a /VIDRAM
//  cycle the next slot clock samples D = 0 and /DTACK asserts.  Worst case is
//  h = 1 -> 5, four H counts.  /CLR = /VPA stops the clock during an
//  interrupt acknowledge.
//
//  This module takes `h3` = {4H,2H,1H} straight from the video counter and
//  makes the once-per-count clock itself: `ce_7m` is the H counter's own rate,
//  so a strobe at a fixed phase inside the count is exactly the 14M edge the
//  14/15B sees during the 7M high phase.  Nothing observable depends on which
//  clk_sys inside the count it is — /DTACK resolution on this board is one
//  /7M period.
//
//  ---- 11D LS74 upper, halt until HBLANK ---------------------------------
//        D (12) = /WAITHBL    CK (11) = HBLANK (rising)
//        /PRE (10) = AS       /CLR (13) = PR1 (inactive)    /Q (8) -> 16A
//  A write to FF8E00 asserts /WAITHBL, which also holds the LS163A cleared
//  (11C LS11), so no acknowledge is produced until the rising edge of HBLANK —
//  the CPU stalls inside the bus cycle for up to a whole scan line (~63.7 us).
//  The game's only caller is its level-1 handler.  MAME spins the CPU instead,
//  which is observationally equivalent but is not what the board does.
//
//  ---- what is not in the equation ---------------------------------------
//  No strobe, no chip select, nothing that says "this address exists".  Reads
//  of the write-only strobes (the read-modify-writes to FF8C00 and FF8000, and
//  the priority-RAM reads) and the undecoded FF9000 page all terminate exactly
//  like a ROM cycle.  There is no bus-error source on this board either (68000
//  pin 22 = PR4), so the only way to hang the CPU is a /WAITHBL write with the
//  raster stopped.
//============================================================================

module blstroid_dtack
(
	input  logic       clk,        // clk_sys
	input  logic       ce_7m,      // /7M enable (clk_sys / 8), the H-counter rate
	input  logic       reset,      // active high; parks the counter at 0

	input  logic       as,         // active-high copy of /AS
	input  logic       vidram_n,   // /VIDRAM
	input  logic       waithbl_n,  // /WAITHBL  (the FF8E00 strobe, level)
	input  logic       eerom_n,    // /EEROM
	input  logic       vpa_n,      // /VPA -> 12C ENT and 14/15B /CLR
	input  logic [2:0] h3,         // {4H, 2H, 1H}
	input  logic       hblank,     // active high

	output logic       dtack,      // active high (= ~/DTACK) for the CPU wrapper
	output logic       rco,        // 12C RCO, exported for observation only
	output logic       vidram_ack, // 11D lower /Q
	output logic       waithbl_ack,// 11D upper /Q
	output logic [3:0] wcnt        // 12C QD..QA, exported for observation only
);

	// ---- the once-per-group video slot clock (g14D = h in {1,5,6}) -------
	// A fixed phase inside the H count; see the header.
	logic [2:0] ph;
	always_ff @(posedge clk) begin
		if (reset)      ph <= 3'd0;
		else if (ce_7m) ph <= 3'd0;
		else            ph <= ph + 3'd1;
	end
	wire g14d      = (h3 == 3'd1) | (h3 == 3'd5) | (h3 == 3'd6);
	wire vslot_clk = (ph == 3'd2) & g14d & vpa_n;   // /CLR = /VPA kills the clock

	// ---- 12C LS163A ------------------------------------------------------
	wire clr_n = waithbl_n & vidram_n & as;   // 11C LS11
	wire ld_n  = wcnt[3];                     // /LD = QD
	wire ent   = vpa_n;
	wire enp   = ~dtack;                      // ENP = /DTACK

	assign rco = (&wcnt) & ent;

	always_ff @(posedge clk) begin
		if (reset) begin
			wcnt <= 4'd0;
		end else if (ce_7m) begin
			if (!clr_n)         wcnt <= 4'd0;                    // synchronous clear
			else if (!ld_n)     wcnt <= {2'b11, eerom_n, 1'b1};  // {D,C,B,A}
			else if (ent & enp) wcnt <= wcnt + 4'd1;
		end
	end

	// ---- 11D LS74 lower — video RAM --------------------------------------
	// /PRE = AS is asynchronous on the real part, so a cycle that ends between
	// slot clocks drops the acknowledge immediately.
	logic q_vid;
	always_ff @(posedge clk) begin
		if (reset)          q_vid <= 1'b1;
		else if (!as)       q_vid <= 1'b1;        // /PRE = AS
		else if (vslot_clk) q_vid <= vidram_n;
	end
	assign vidram_ack = ~q_vid;

	// ---- 11D LS74 upper — halt until HBLANK ------------------------------
	logic q_hbh, hblank_d;
	always_ff @(posedge clk) begin
		if (reset) begin
			q_hbh    <= 1'b1;
			hblank_d <= 1'b0;
		end else begin
			hblank_d <= hblank;
			if (!as)                        q_hbh <= 1'b1;   // /PRE = AS
			else if (hblank & ~hblank_d)    q_hbh <= waithbl_n;
		end
	end
	assign waithbl_ack = ~q_hbh;

	// ---- 16A LS260 -------------------------------------------------------
	assign dtack = rco | vidram_ack | waithbl_ack;

endmodule
