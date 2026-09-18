`timescale 1ns/1ps
//============================================================================
//  Xicor X2804A (512 x 8) EEPROM at 2A/B, plus the 14/15B LS74 unlock latch —
//  SP-316 sheet 3.
//
//  Copyright (C) 2026 RetroShrimp.  GPL-3.0-or-later; see LICENSE and
//  NOTICE.md.  Derived from the Xybots MiSTer core's
//  rtl/main/xybots_eeprom_2804.sv (GPL-3.0, same author; itself from Toobin');
//  the two boards wire the part identically apart from the strobe addresses.
//
//  ---- wiring -------------------------------------------------------------
//    A0..A10 (8,7,6,5,4,3,2,1,23,22,19) = BA1..BA11
//    D0..D7                             = BD0-BD7 only, the low byte lane
//    /CE (18) = /EEROM        (10B LS139 Y3, FFB000-FFBFFF)
//    /WE (21) = /WL           (12D F32, the board-global low-byte write strobe)
//    /OE (20) = 14B LS08 pin 8 = Q(14/15B lower) & /RESET
//
//  The socket is wired for a 2816 (2 K x 8) but the fitted X2804A is 512 x 8
//  (A0..A8 only), so the device repeats four times over FFB001-FFBFFF — which
//  is what sheet 12's "a = ADDRESS BITS ON 2816 ONLY" note means and what
//  MAME's `.mirror(0x038c00)` produces.  Addressing here is 9 bits; the caller
//  drops BA10/BA11.
//
//  Low byte lane means odd byte addresses: the game walks the part from
//  FFB001 with a stride of 2, and every EEPROM bus cycle uses D7:0.
//
//  ---- the unlock latch, 14/15B LS74 lower half --------------------------
//    D (12) = GND    CK (11) = /WL    /PRE (10) = /UNLOCK   /CLR (13) = /RESET
//    Q (9) -> 14B LS08 pin 9;  14B pin 10 = /RESET;  14B pin 8 -> /OE
//
//  The X2804A family inhibits a write while /OE is LOW, so:
//   * after reset Q = 0 -> /OE = 0 -> the part drives the bus on reads and
//     writes are inhibited.  This is "locked".
//   * a write to FF8600 presets Q = 1 -> /OE = 1 -> outputs off, and a /WE
//     pulse now takes effect.  This is "unlocked".
//   * the first following low-byte write clocks D = 0 on the /WL rising edge,
//     so the part re-locks after exactly one write — MAME's
//     `lock_after_write(true)`.
//   * /RESET (power-on, watchdog, or a 68000 RESET instruction — they are all
//     one net) clears the latch and forces /OE low.
//
//  Two orderings that matter, both reproduced here:
//   * the unlock write is itself a word write to FF8600, so /WL rises at the
//     end of that very cycle.  /UNLOCK is a level on the part's /PRE and beats
//     it, so the unlock survives its own write.
//   * `write_accepted` samples `unlocked` before the same strobe re-locks the
//     part, so the byte the game unlocked for does get stored.
//  The game brackets the unlock+write pair with `ori.w #$700,sr` for exactly
//  this reason: an interrupt in between would consume the unlock.
//
//  ---- /OE and reads ------------------------------------------------------
//  While the part is unlocked (Q = 1) /OE is high and the outputs are off: a
//  read in that window returns the floating board bus, not the cell.  The game
//  never reads there, but the behaviour is modelled rather than assumed away.
//
//  ---- programming time ---------------------------------------------------
//  The X2804A data sheet gives a self-timed byte write of typically 5 ms and
//  at most 10 ms with the I/O pins high-Z throughout; MAME models no write
//  time at all.  The default here is the 10 ms bound, and `WRITE_CYCLES` is a
//  parameter so simulation need not spend half a million clocks per byte.  The
//  game's background writer compares each cell against a RAM shadow every
//  VBLANK and writes at most one differing byte per pass, so a busy read
//  returning 0xFF simply makes it try again on the next frame.
//============================================================================

module blstroid_eeprom_2804 #(
	parameter int CLK_HZ       = 57_272_724,
	parameter int WRITE_CYCLES = CLK_HZ / 100      // 10 ms
)(
	input  logic       clk,
	input  logic       init_reset,   // FPGA / PLL init: clears the busy state
	input  logic       reset,        // the board /RESET net, active high

	input  logic       unlock,       // 1-clk pulse: any write to FF8600
	input  logic       low_write,    // 1-clk pulse: rising /WL, from anywhere
	input  logic       cpu_we,       // 1-clk pulse: /EEROM write with /WL low
	input  logic [8:0] cpu_addr,     // BA9..BA1
	input  logic [7:0] cpu_wdata,
	output logic [7:0] cpu_rdata,
	output logic       busy,
	output logic       oe_n,         // 14B LS08 output -> 2A/B pin 20
	output logic       unlocked,
	output logic       write_accepted,

	// MiSTer NVRAM restore / save-back (blstroid_nvram_io)
	input  logic       load_we,
	input  logic [8:0] load_addr,
	input  logic [7:0] load_data,
	input  logic [8:0] dump_addr,
	output logic [7:0] dump_data
);

	localparam int BUSY_W = (WRITE_CYCLES <= 1) ? 1 : $clog2(WRITE_CYCLES + 1);
	logic [BUSY_W-1:0] busy_ctr;
	logic [7:0] mem [0:511];

`ifndef ALTERA_RESERVED_QIS
	// Simulation only: an unprogrammed X2804A reads 0xFF, which is also what
	// the MRA ships as the factory image, so a first cold boot walks the
	// game's own default-writing pass.
	initial for (int i = 0; i < 512; i++) mem[i] = 8'hFF;
`endif

	// ---- power-up erase: a factory X2804A reads 0xFF, an FPGA RAM reads 0x00
	//
	// The `initial` fill above is simulation only.  On the FPGA the array is an
	// inferred RAM whose cells power up at 0x00, and an all-zero part is not a
	// state this game can leave behind: the boot check reads an all-zero
	// 15-byte block as eight *correctable* Hamming errors rather than as an
	// unprogrammed one, believes it, walks every block instead of giving up
	// after two, and ends with an error count over its own threshold and an
	// "EEPROM ERROR" banner on screen.  An erased (0xFF) block is
	// uncorrectable, which is what the check is looking for.
	//
	// So the erased state is established here, by the part itself, and does not
	// depend on the loader, the MRA or the synthesis tool's RAM initialisation.
	//
	// Every loader byte reaches this module while `init_reset` is high (at
	// blstroid_main that input covers the whole index-0 ROM stream and the
	// whole of an index-2 NVRAM restore), so that window is exactly one "image
	// load".  The decision taken when it releases is:
	//
	//   an image was loaded and every byte of it was 0x00  -> erase
	//        (a save file the HPS has never actually written; "blank file"
	//         therefore means "blank part")
	//   no image was loaded and nothing has ever been written -> erase
	//        (the MRA's 0x200-byte erased tail did not arrive)
	//   anything else -> leave the array alone
	//
	// `programmed` is the "something real has been written here" flag.  It is
	// deliberately active high and resetless, so that the power-up value of
	// such a register — 0 — is already the correct one.  It must not be cleared
	// by `init_reset`, which also pulses on every OSD reset: clearing it there
	// would erase a restored image or a freshly programmed option.
	//
	// One erase pass is 512 clk_sys = 9 us, and the watchdog holds the 68000
	// off until the first /VBLANK, so the pass always finishes before the CPU
	// runs.  A loader byte in the same cycle wins and aborts the pass.
	logic       programmed;
	logic       ld_seen, ld_nonzero;
	logic       por_run;
	logic [8:0] por_addr;
	wire        ld_data_nz  = load_we & (|load_data);
	wire        nx_seen     = ld_seen    | load_we;
	wire        nx_nonzero  = ld_nonzero | ld_data_nz;
	wire        por_we      = por_run & ~load_we;

	always_ff @(posedge clk) begin
		if (ld_data_nz || write_accepted) programmed <= 1'b1;

		if (init_reset) begin
			ld_seen    <= nx_seen;
			ld_nonzero <= nx_nonzero;
			// armed, not running; re-evaluated on every held cycle
			por_run    <= nx_seen ? ~nx_nonzero : ~programmed;
			por_addr   <= 9'd0;
		end else begin
			ld_seen    <= 1'b0;
			ld_nonzero <= 1'b0;
			if (load_we || write_accepted) por_run <= 1'b0;
			else if (por_run) begin
				por_addr <= por_addr + 1'b1;
				if (&por_addr) por_run <= 1'b0;
			end
		end
	end

	assign busy           = |busy_ctr;
	assign write_accepted = cpu_we & unlocked & ~busy;
	assign dump_data      = mem[dump_addr];
	assign oe_n           = unlocked & ~reset;   // 14B LS08 (Q, /RESET)

	// 14/15B LS74 lower.  /CLR = /RESET is asynchronous on the part; /UNLOCK
	// (/PRE) beats the /WL edge of its own cycle — see the header.
	always_ff @(posedge clk) begin
		if (init_reset || reset) unlocked <= 1'b0;
		else if (unlock)         unlocked <= 1'b1;
		else if (low_write)      unlocked <= 1'b0;
	end

	// The cell array has no reset pin, so programming continues across a board
	// reset.  An out-of-band loader write is the power-up restore and wins.
	//
	// The loader write sits outside the `init_reset` guard, and must stay
	// there: `init_reset` is asserted for the whole of the download, so a
	// loader write inside the guard would silently discard every byte of the
	// factory image and leave the array at the FPGA RAM's 0x00 power-up value.
	// Simulation does not catch that, because the sim-only `initial` fill has
	// already put 0xFF in every cell.
	always_ff @(posedge clk) begin
		if (load_we) begin
			mem[load_addr] <= load_data;
			busy_ctr <= '0;
		end else if (init_reset) begin
			busy_ctr <= '0;
		end else begin
			if (busy) busy_ctr <= busy_ctr - 1'b1;
			if (por_we) begin
				mem[por_addr] <= 8'hFF;      // factory-erased cell
			end else if (write_accepted) begin
				mem[cpu_addr] <= cpu_wdata;
				busy_ctr <= BUSY_W'(WRITE_CYCLES);
			end
		end

		// During programming the I/O pins float; so do they while /OE is high
		// (the unlocked window).  There are no pull-ups on BD0-BD7, so the
		// modelled float value is the 0xFF an unmapped MAME read gives.
		if (busy || oe_n) cpu_rdata <= 8'hFF;
		else              cpu_rdata <= mem[cpu_addr];
	end

endmodule
