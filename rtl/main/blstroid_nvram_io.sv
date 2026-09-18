`timescale 1ns/1ps
//============================================================================
//  MiSTer transport / save-request controller for the Blasteroids X2804A.
//
//  Copyright (C) 2026 RetroShrimp.  GPL-3.0-or-later; see LICENSE and
//  NOTICE.md.  Derived from the Xybots MiSTer core's
//  rtl/main/xybots_nvram_io.sv (GPL-3.0, same author; itself from Toobin'),
//  re-based on this core's download stream.
//
//  Two channels feed the same 512-byte array:
//   * ioctl index 0 is the MRA ROM stream.  Blasteroids.mra appends the EEPROM
//     image as its last 0x200 bytes at 0x190000, shipped erased (0xFF) exactly
//     like an Atari factory part, so a first cold boot walks the game's own
//     332-byte default-writing pass.  That is the factory image.
//   * ioctl index 2 is the `<nvram index="2" size="512"/>` restore/save
//     channel; it lands on top of the factory image at boot and is what the
//     HPS reads back when the game programs a byte.
//
//  Blasteroids has no DIP switches at all: every operator option lives in that
//  EEPROM and is set from the self test, so this path carries the whole of the
//  machine's persistent configuration.
//
//  The cell / unlock / busy behaviour stays in blstroid_eeprom_2804.
//
//  The dirty flag deliberately survives a board or watchdog reset, because the
//  physical EEPROM does; dropping it there could lose a freshly programmed
//  option or high score.  Only FPGA initialisation or an out-of-band
//  factory/NVRAM load establishes a clean image.  If a write and an upload
//  start coincide, the write wins and is requested again.
//============================================================================

module blstroid_nvram_io #(
	// ~1.17 s at 57.272724 MHz: long enough that a burst of option writes from
	// one self-test page produces a single upload.
	parameter int unsigned SETTLE_CYCLES = 67_108_863,
	parameter logic [26:0] EEP_BASE = 27'h190000
)(
	input  logic        clk,
	input  logic        init_reset,

	input  logic        write_accepted,

	input  logic        ioctl_download,
	input  logic        ioctl_wr,
	input  logic [26:0] ioctl_addr,
	input  logic  [7:0] ioctl_dout,
	input  logic [15:0] ioctl_index,
	input  logic        ioctl_upload,

	output logic        load_we,
	output logic  [8:0] load_addr,
	output logic  [7:0] load_data,
	output logic  [8:0] dump_addr,
	input  logic  [7:0] dump_data,

	output logic        ioctl_upload_req,
	output logic  [7:0] ioctl_upload_index,
	output logic  [7:0] ioctl_din
);

	wire index0_eeprom = (ioctl_index == 16'd0) &&
	                     (ioctl_addr >= EEP_BASE) &&
	                     (ioctl_addr < EEP_BASE + 27'h200);
	wire index2_nvram  = (ioctl_index == 16'd2);

	assign load_we   = ioctl_download & ioctl_wr & (index0_eeprom | index2_nvram);
	assign load_addr = index2_nvram ? ioctl_addr[8:0] : 9'(ioctl_addr - EEP_BASE);
	assign load_data = ioctl_dout;
	assign dump_addr = ioctl_addr[8:0];

	localparam int SETTLE_W = (SETTLE_CYCLES <= 1) ? 1 : $clog2(SETTLE_CYCLES + 1);
	logic [SETTLE_W-1:0] settle;
	logic dirty;
	wire  settled      = (settle == SETTLE_W'(SETTLE_CYCLES));
	wire  upload_start = ioctl_upload & index2_nvram;

	always_ff @(posedge clk) begin
		if (init_reset) begin
			dirty  <= 1'b0;
			settle <= '0;
		end else if (write_accepted) begin
			dirty  <= 1'b1;      // a new byte cannot be cleaned by a concurrent upload
			settle <= '0;
		end else if (load_we || upload_start) begin
			dirty  <= 1'b0;
			settle <= '0;
		end else if (dirty && !settled) begin
			settle <= settle + 1'b1;
		end
	end

	assign ioctl_upload_req   = dirty & settled;
	assign ioctl_upload_index = 8'd2;
	assign ioctl_din          = dump_data;

endmodule
