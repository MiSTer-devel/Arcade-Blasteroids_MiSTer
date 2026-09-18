`timescale 1ns/1ps
//============================================================================
//  Blasteroids — SP-316 sheet 5: the 16 KB video / work RAM (5L, 6/7L), its
//  address multiplexers, the eight-count slot arbiter, the CPU data path and
//  the motion-object link registers.
//
//  Copyright (C) 2026 RetroShrimp.  GPL-3.0-or-later; see LICENSE and
//  NOTICE.md.  Structurally derived from the Xybots MiSTer core's video-RAM
//  arbiter (GPL-3.0, same author), with every gate re-derived from sheet 5.
//
//  ---------------------------------------------------------------------------
//  The crux of sheet 5: /OE is tied low
//  ---------------------------------------------------------------------------
//  Both 6264s have /OE (pin 22) hard-wired to ground, so the RAM drives VD15:0
//  continuously — on every count, in every slot.  There is no video read strobe
//  anywhere on the board: each consumer simply latches VD on its own clock
//  edge.  This module reproduces that: `vd` is the registered read of whatever
//  address the multiplexers select, valid from the second clk_sys cycle of each
//  `1H` count and therefore stable at that count's `ce_7m` — the edge on which
//  sheet 6's 4K/4L/6F latch the playfield map word and sheet 5's 13E/7D latch
//  the motion-object link.
//
//  ---------------------------------------------------------------------------
//  The eight-count slot schedule
//  ---------------------------------------------------------------------------
//  Ownership is a pure function of `H[2:0]` = {4H, 2H, 1H}, so the pattern
//  repeats every 8 counts = 16 pixel clocks = one playfield stamp:
//
//    H[2:0]        0      1      2     3      4       5       6     7
//    owner         MO     MO     CPU   PF     MO      MO      CPU   CPU
//    word          MOVPOS MOPIC  --    map    MOLINK  MOHPOS  --    --
//    SELCPU        1      0      1     0      1       1       1     1
//    RAMFREE       0      0      1     0      0       0       1     1
//    {2H,SELCPU}   01     00     11    10     01      01      11    11
//
//  Four motion-object slots, three CPU slots and one playfield slot per group.
//  Blasteroids has no alphanumerics layer, so the slot a System-1-style board
//  would spend on it goes to the 68000 instead.
//
//  Resulting WORD addresses ({VRAM12..VRAM0} = CPU A13..A1):
//    PF   {2'b00, v[7:3], h[8:3]}                 = row * 64 + column
//    MO   {2'b01, MOLNK8, link[7:0], h[2], h[0]}  = 0x800 + link*4 + word
//    CPU  {BA13..BA1}                             = the whole 16 KB
//  FFD000 is 0x1000 BYTES above FFC000 and therefore word 0x800 — which is
//  exactly the `{2'b01, ...}` the 5F LS253 drives.  Do not confuse the two.
//  `VRAM12` = 11C LS11 = `BA13 & RAMFREE`, so the video side can only ever
//  address FFC000-FFDFFF; the CPU reaches the work RAM at FFE000-FFFFFF too.
//
//  ---------------------------------------------------------------------------
//  The CPU handshake
//  ---------------------------------------------------------------------------
//  There is no request/grant on the board — the CPU's address and data sit on
//  the bus for the whole 68000 cycle and the slot logic simply uses them when
//  the slot comes round.  The FPGA port is the same shape:
//
//    cpu_sel     LEVEL, held for the whole video-RAM bus cycle (= ~/VIDRAM & AS).
//    cpu_a[12:0] BA13..BA1, cpu_din[15:0] = BD15:0, held with `cpu_sel`.
//    cpu_we_hi   LEVEL = ~/WH  -> writes vd[15:8] (6/7L, the high byte lane)
//    cpu_we_lo   LEVEL = ~/WL  -> writes vd[7:0]  (5L,  the low  byte lane)
//                A write reaches the RAM only when RAMFREE & cpu_sel & the lane
//                & 7M is high (sheet 2: /VWRH = /VMEMH | /7M), i.e. ONE pulse in
//                the first half of each CPU slot.  Byte lanes are independent.
//    cpu_dout    the 10D / 8D LS373 hold latch: it follows `vd` while RAMFREE
//                and holds the last CPU slot's word afterwards.  It is valid
//                from the middle of the CPU slot on and stays valid, which is
//                why /DTACK may be — and on the board IS — asserted one count
//                EARLIER than the slot (blstroid_dtack clocks at H[2:0] = 1, 5,
//                6, each exactly one count ahead of the slots at 2, 6, 7).
//    cpu_slot    LEVEL, = RAMFREE.  "the CPU owns the RAM this count".
//    cpu_ack     ONE clk_sys pulse at the END of every CPU slot in which
//                `cpu_sel` was high: the write has happened / `cpu_dout` holds
//                this cycle's word.  Purely informational — /DTACK comes from
//                `blstroid_dtack`, not from here.
//
//  Worst-case wait: the acknowledge points are counts 1, 5 and 6 of every 8,
//  one per CPU slot, so a cycle that starts just after the h = 6 point waits
//  until h = 1 of the next group — four counts = eight pixel clocks = 0.56 us.
//  Best case is zero.
//
//  ---------------------------------------------------------------------------
//  Where the RAM itself lives
//  ---------------------------------------------------------------------------
//  `INTERNAL_RAM = 1` (default) instantiates the 8 K x 16 byte-writable array
//  here.  `INTERNAL_RAM = 0` exports the same port (`vram_a`, `ram_din`,
//  `ram_we_hi/lo`, `ram_dout`) so the core can instead share the one copy that
//  `blstroid_main` already instantiates for the CPU side: wire `vram_a` ->
//  `vram_vaddr` and `ram_dout` <- `vram_vdata`.  An external RAM must have a
//  one-clk_sys registered read, like `blstroid_dpram_be`.
//
//  ---------------------------------------------------------------------------
//  What is deliberately not here
//  ---------------------------------------------------------------------------
//  The 4K LS174B / 4L LS374C / 6F F174A map-word latches and the 4E LS174C
//  re-timing latch (PFPAL2:0, MOLNK8, 1VD6H) are on sheet 6 and live in
//  `blstroid_pf.sv`, with the rest of the playfield pipeline they belong to.
//  This module therefore EXPORTS `pmlnk8` (13E LS74 Q, sheet 5) and IMPORTS
//  `molnk8` (4E LS174C Q12, sheet 6) — exactly the two wires that cross between
//  the sheets.
//============================================================================

module blstroid_vram #(
	parameter bit INTERNAL_RAM = 1
)(
	input  logic        clk,
	input  logic        reset,
	input  logic        ce_14m,
	input  logic        ce_7m,

	// ---- from blstroid_sos2_sync ----------------------------------------
	input  logic  [8:0] h,          // 1H..256H
	input  logic  [7:0] v,          // 1V..128V
	input  logic        m7,         // 7M — gates the write strobes
	input  logic        selcpu,     // 14D LS00
	input  logic        ramfree,    // 11J LS02 + 14B LS08
	input  logic        linkres_n,  // SOS-2 pin 25 -> 13E //CLR, 7D //CLR

	// ---- from blstroid_pf (sheet 6, 4E LS174C Q12) ----------------------
	input  logic        molnk8,     // VRAM10 in the four motion-object slots

	// ---- 68000 port (see "THE CPU HANDSHAKE" above) ---------------------
	input  logic [12:0] cpu_a,      // BA13..BA1
	input  logic [15:0] cpu_din,    // BD15:0
	input  logic        cpu_sel,    // ~/VIDRAM & AS, held for the bus cycle
	input  logic        cpu_we_hi,  // ~/WH
	input  logic        cpu_we_lo,  // ~/WL
	output logic [15:0] cpu_dout,   // 10D / 8D LS373 hold latch
	output logic        cpu_slot,   // = RAMFREE
	output logic        cpu_ack,    // one clk_sys pulse at the end of a CPU slot

	// ---- the continuously-driven video data bus -------------------------
	output logic [15:0] vd,         // VD15:0 — valid from clk 2 of every count
	output logic [12:0] vram_a,     // VRAM12:0, the multiplexed word address

	// ---- 4K LS174B, the playfield map word's palette field ---------------
	// Q2, Q5, Q7 = PFM2, PFM1, PFM0 = the latched VD15, VD14, VD13, captured on
	// the `4H` rising edge once per stamp column.  `pfm[2]` (PFM2) is BOTH the
	// playfield palette MSB and the row-programmable-interrupt condition that
	// sheet 2's 11C LS11 / 1D LS20 / 13C LS74 chain turns into IRQ1, which is
	// why it leaves this module rather than `blstroid_pf`: the interrupt path
	// must not depend on the playfield pipeline.
	output logic  [2:0] pfm,        // {PFM2, PFM1, PFM0}
	output logic        pfm_strobe, // one clk_sys pulse ON that 4H edge

	// ---- motion-object link walk (13E LS74 / 7D LS273A / 7E LS374) ------
	output logic        pmlnk8,     // 13E LS74 Q5  -> sheet 6 4E LS174C D13
	output logic  [8:0] mo_link,    // {MOLNK8, VRAM9:2} = the link being READ
	output logic        mo_word,    // 1 while this count is a motion-object slot
	output logic  [1:0] mo_word_idx,// {4H, 1H} = 0 MOVPOS 1 MOPIC 2 MOLINK 3 MOHPOS
	output logic        mo_strobe,  // one clk_sys pulse at the END of an MO slot
	                                //   (vd holds word `mo_word_idx`)
	output logic        pf_strobe,  // one clk_sys pulse at the END of the PF slot

	// ---- external RAM port, used only when INTERNAL_RAM = 0 -------------
	output logic [15:0] ram_din,
	output logic        ram_we_hi,
	output logic        ram_we_lo,
	input  logic [15:0] ram_dout
);

	// =====================================================================
	// 1. The motion-object link registers — 13E LS74, 7D LS273A, 7E LS374
	// =====================================================================
	// MOLINK is on VD during H[2:0] = 4.  `4HD1H` rises at the 4->5 boundary
	// and captures VD11:3 into 13E + 7D; `4HD2H` rises at the 5->6 boundary and
	// moves it into 7E, where it becomes the video-RAM address for the NEXT
	// group's four motion-object slots.  In this core's phase convention those
	// two edges are `ce_7m & (h[2:0] == 4)` and `ce_7m & (h[2:0] == 5)`.
	//
	// `/LINKRES` clears 13E and 7D once per line, so the walk starts from link 0
	// at the top of every scanline.  There is no SLIP RAM and no per-band start
	// pointer on this board.
	logic [7:0] link_a;    // 7D LS273A — the link just latched off VD
	logic [7:0] link_b;    // 7E LS374  — the link the address mux is using

	always_ff @(posedge clk) begin
		if (reset) begin
			pmlnk8 <= 1'b0;
			link_a <= 8'd0;
			link_b <= 8'd0;
		end else begin
			if (!linkres_n) begin                       // asynchronous /CLR
				pmlnk8 <= 1'b0;
				link_a <= 8'd0;
			end else if (ce_7m && (h[2:0] == 3'd4)) begin   // 4HD1H rising
				pmlnk8 <= vd[11];
				link_a <= vd[10:3];
			end
			if (ce_7m && (h[2:0] == 3'd5)) link_b <= link_a; // 4HD2H rising
		end
	end

	// 4K LS174B — CK 9 = `4H`, /C 1 = PR2.  `4H` rises at the 3->4 boundary,
	// which in this core's phase convention is `ce_7m & (h[2:0] == 3)`; `vd`
	// then holds the playfield map word read in the PF slot at H[2:0] = 3.
	assign pf_strobe   = ce_7m & (h[2:0] == 3'd3);
	assign pfm_strobe  = pf_strobe;
	always_ff @(posedge clk) begin
		if (reset)          pfm <= 3'd0;
		else if (pf_strobe) pfm <= vd[15:13];
	end

	assign mo_link     = {molnk8, link_b};
	assign mo_word_idx = {h[2], h[0]};
	assign mo_word     = ~h[1];                    // H[2:0] = 0, 1, 4, 5
	assign mo_strobe   = ce_7m & mo_word;

	// =====================================================================
	// 2. Address multiplexers — 5F LS253, 4F LS253A, 6E/5E LS257, 11C LS11
	// =====================================================================
	// The LS253 select is {SB, SA} = {2H, SELCPU}; the LS257 pair is enabled
	// only while 2H = 1 (the CPU and PF slots) and selects the counters when
	// SELCPU = 0, so at H[2:0] = 3 the playfield wins and at 2, 6, 7 the CPU
	// does.  While 2H = 0 the 7E LS374 drives VRAM9:2 with the latched link.
	// Written as continuous assignments rather than always_comb so that no tool
	// has to infer a sensitivity list over the part-selects (Icarus widens it).
	wire is_cpu = h[1] &  selcpu;                     // H[2:0] = 2, 6, 7
	wire is_pf  = h[1] & ~selcpu;                     // H[2:0] = 3
	//    is_mo  = ~h[1]                              // H[2:0] = 0, 1, 4, 5

	wire [1:0] a11_10 = is_cpu ? cpu_a[11:10]        // C3 — CPU
	                  : is_pf  ? {1'b0, v[7]}        // C2 — playfield
	                           : {1'b1, molnk8};     // C1/C0 — motion objects
	wire [1:0] a1_0   = is_cpu ? cpu_a[1:0]
	                  : is_pf  ? {h[4], h[3]}
	                           : {h[2], h[0]};
	wire [7:0] a9_2   = ~h[1]  ? link_b              // 7E LS374, the MO slots
	                  : selcpu ? cpu_a[9:2]          // 6E/5E B inputs, CPU
	                           : {v[6:3], h[8:5]};   // 6E/5E A inputs, playfield

	// 11C LS11: VRAM12 = BA13 & 2H & SELCPU = BA13 & RAMFREE.
	assign vram_a = {cpu_a[12] & ramfree, a11_10, a9_2, a1_0};

	// =====================================================================
	// 3. Write strobes — sheet 2, 11E LS32 chain
	// =====================================================================
	//   /VMEMH = /WH | (/RAMCPU | /VIDRAM)     /VWRH = /VMEMH | /7M
	// so a CPU write lands only in a CPU slot, in the selected region, on the
	// selected byte lane, while 7M is high.  `ce_14m & m7` is that half-count
	// window reduced to exactly one clk_sys pulse.
	wire wr_win = ce_14m & m7 & ramfree & cpu_sel;

	assign ram_din   = cpu_din;
	assign ram_we_hi = wr_win & cpu_we_hi;
	assign ram_we_lo = wr_win & cpu_we_lo;

	// =====================================================================
	// 4. The RAM
	// =====================================================================
	generate
		if (INTERNAL_RAM) begin : g_ram
			// Split copies of the two lanes: keeping the part-selects OUTSIDE
			// the RAM's always_ff blocks keeps Icarus from widening their
			// sensitivity lists.  They live INSIDE this branch because at
			// INTERNAL_RAM = 0 the array is blstroid_main's and nothing here
			// writes it, so declaring them outside would leave two unused
			// signals behind.
			wire [7:0] din_hi = ram_din[15:8];
			wire [7:0] din_lo = ram_din[7:0];

			// 8 K x 16 with independent byte lanes = 5L (low) + 6/7L (high).
			// One clk_sys of read latency models the async SRAM well enough:
			// the address only moves at a count boundary, so `vd` is valid from
			// the second clk_sys cycle of the count and is stable at its ce_7m.
			logic [7:0] mem_hi [0:8191];
			logic [7:0] mem_lo [0:8191];
			logic [7:0] q_hi, q_lo;
			always_ff @(posedge clk) begin
				if (ram_we_hi) mem_hi[vram_a] <= din_hi;
				q_hi <= mem_hi[vram_a];
			end
			always_ff @(posedge clk) begin
				if (ram_we_lo) mem_lo[vram_a] <= din_lo;
				q_lo <= mem_lo[vram_a];
			end
			assign vd = {q_hi, q_lo};
		end else begin : g_extram
			assign vd = ram_dout;
		end
	endgenerate

	// =====================================================================
	// 5. CPU data path — 10D / 8D LS373 (G = RAMFREE)
	// =====================================================================
	// The LS373s are TRANSPARENT through the CPU slot and hold afterwards, and
	// their /OC is only enabled during a video-RAM read, so the 68000 sees the
	// held word for the rest of its bus cycle.  Sampling at `ce_14m & ramfree`
	// reproduces that: it is inside the slot and after `vd` has settled.
	always_ff @(posedge clk) begin
		if (reset)                    cpu_dout <= 16'd0;
		else if (ce_14m && ramfree)   cpu_dout <= vd;
	end

	assign cpu_slot = ramfree;
	assign cpu_ack  = ce_7m & ramfree & cpu_sel;

	// `v[2:0]` is the line inside the stamp — sheet 6's PF ROM address, formed in
	// blstroid_pf, not part of the map address.  `ram_dout` is the INTERNAL_RAM
	// = 0 port and is unused in the default configuration.
	/* verilator lint_off UNUSEDSIGNAL */
	wire unused_vram = |v[2:0] | |ram_dout;
	/* verilator lint_on UNUSEDSIGNAL */

endmodule
