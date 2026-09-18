`timescale 1ns/1ps
//============================================================================
//  Blasteroids (Atari Games, 1987) - JSA I sound-CPU memory: the 8 KB work RAM
//  (2H) plus the A13B/A12B paging of the 64 KB program ROM (2K).
//
//  Copyright (C) 2026 RetroShrimp.  GPL-3.0-or-later; see LICENSE and NOTICE.md.
//  Derived from the Xybots MiSTer core's xybots_jsa_mem.sv (GPL-3.0, same
//  author, itself from Toobin').
//
//  The 64 KB ROM array is not here.  This core keeps one sound-ROM BRAM in
//  rtl/mem, shared with the download loader; this module drives its address and
//  consumes its registered read data through rom_addr / rom_data.  Only the
//  work RAM is local.
//
//  SP-316 sheet 15:
//    * "2H" 8K x 8 150 ns: A12..A0 = SA12..SA0, /CS1(20)=/RAM, CS2(26)=VCC,
//      /WE(27)=/SWR, /OE(22)=/SRD.                        -> 0x0000-0x1FFF
//    * "2K" 27512 300 ns (136057-1135.2k): A15(1)=SA15, A14(27)=SA14,
//      A13(26)=A13B, A12(2)=A12B, A11..A0 = SA11..SA0,
//      /OE(22)=/SRD, /CE(20)=/ROM.
//      A13B/A12B are 2L PAL outputs that substitute the /WRIO latch's
//      BA13/BA12 in the 0x3000-0x3FFF window and pass SA13/SA12 everywhere
//      else - MAME's bankr("cpubank") (4 x 0x1000 entries over the ROM's
//      bottom 16 KB) plus rom() at 0x4000-0xFFFF.
//
//  ROM offset:
//    sel_bank -> {2'b00, bank, addr[11:0]}   (image 0x0000-0x3FFF)
//    sel_rom  -> addr                        (image 0x4000-0xFFFF)
//
//  All four banks are live.  The Blasteroids sound ROM's boot checksum at
//  $4090-$40D5 walks them explicitly (AND #$3F = bank 0, then ORA #$40,
//  ORA #$80, ORA #$C0), and the runtime selector at $57F4 is
//  LDA $3F / AND #$3F / ORA $63D3,X / STA $3F / STA $2A04.
//
//  ---- read timing ---------------------------------------------------------
//  Registered (BRAM) read: `dout` is valid one clk after addr/selects, on both
//  paths.  `rom_addr` is combinational from addr/bank and `rom_data` is
//  expected back one clk later, so the external ROM and the local RAM land in
//  the same cycle and the select that was valid at address time picks between
//  them.
//
//  The work RAM has its own dedicated registered-output read.  Do not fold it
//  into a shared output register with anything else: an M10K's output register
//  cannot be shared, and Quartus then reports "asynchronous read logic" and
//  falls back to flip-flops, which costs enough logic to overflow the device.
//============================================================================

module blstroid_jsa_mem
(
	input  logic        clk,

	// ---- 6502 access ----
	input  logic [15:0] addr,
	input  logic  [7:0] din,
	input  logic        we,          // 6502 write (RAM only; 2K has no /SWR path)
	input  logic        sel_ram,
	input  logic        sel_bank,
	input  logic        sel_rom,
	input  logic  [1:0] bank,        // BA13:BA12 from the 5H /WRIO latch

	// ---- external 64 KB sound ROM (core/rtl/mem, filled by the loader) ----
	output logic [15:0] rom_addr,    // combinational; data expected 1 clk later
	input  logic  [7:0] rom_data,

	output logic  [7:0] dout
);

	(* ramstyle = "no_rw_check, M10K" *) logic [7:0] ram [0:8191];   // 2H, 8 KB
`ifndef ALTERA_RESERVED_QIS
	// Simulation-only zero fill, so a simulator never reads X out of an
	// unwritten location.  Quartus zeroes M10K contents at configuration anyway.
	initial begin for (int i = 0; i < 8192; i++) ram[i] = 8'h00; end
`endif

	assign rom_addr = sel_bank ? {2'b00, bank, addr[11:0]} : addr;

	logic [7:0] ram_q;
	logic       sel_ram_q, sel_romish_q;
	always_ff @(posedge clk) begin
		if (we && sel_ram) ram[addr[12:0]] <= din;
		ram_q        <= ram[addr[12:0]];
		sel_ram_q    <= sel_ram;
		sel_romish_q <= sel_bank | sel_rom;
	end

	// Combinational mux over the two registered read paths == a 1-clk latency,
	// selected by the select that was valid at address time.
	always_comb begin
		if      (sel_ram_q)    dout = ram_q;
		else if (sel_romish_q) dout = rom_data;
		else                   dout = 8'hFF;
	end

endmodule
