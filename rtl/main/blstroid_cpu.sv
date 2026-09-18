`timescale 1ns/1ps
//============================================================================
//  Blasteroids main CPU — fx68k (cycle-exact 68000) wired as SP-316 sheet 2
//  draws the 2C U68000.  This is the only 68000 in the core.
//
//  Copyright (C) 2026 RetroShrimp.  GPL-3.0-or-later; see LICENSE and
//  NOTICE.md.  The CPU itself is rtl/lib/fx68k/ — Jorge Cwik's fx68k,
//  GPL-3.0-or-later, vendored verbatim.  Wrapper from the Xybots MiSTer core's
//  rtl/main/xybots_cpu.sv (GPL-3.0, same author); the clocking and phase
//  generation are unchanged, because both boards clock a 68000 at crystal/2
//  the same way.
//
//  fx68k models the real part's microcode, so its instruction timing is the
//  datasheet's and a read-modify-write emits its bus cycles in the real order.
//  The game depends on both: its power-on delay loop is timed by instruction
//  count, and its `clr.w $8c00.w` reads the write-only /AUDRES strobe before
//  writing it.
//
//  ---- clock enables ------------------------------------------------------
//  fx68k is fully synchronous on `clk` (= clk_sys, 57.272724 MHz) and takes
//  TWO enables per CPU clock, one per phase of the nominal 7.15909 MHz /7M:
//      enPhi1 -> the CPU clock's HIGH phase -> the `ce_7m` tick (clk_sys/8)
//      enPhi2 -> the LOW phase, 4 clk_sys later (the ce_14m tick between)
//  The second phase is regenerated here from `ce_7m` by a counter that is
//  re-zeroed on every ce_7m tick, so it is locked to it by construction and
//  the rest of sheet 2 (the 12C LS163A wait counter, the 11D LS74 pair, the
//  /VPA path) needs no second enable.
//
//  ---- pins as sheet 2 wires them ----------------------------------------
//    15 CLK        = /7M                     -> enPhi1/enPhi2
//    17 HALT, 18 RESET = the /RESET net       -> `reset` (see deviation 1)
//    10 DTACK      = /DTACK  (16A LS260; `dtack` here is active high)
//    21 VPA        = /VPA    (1D LS20)
//    22 BERR, 13 BR, 12 BGACK = PR4, never asserted — there is no bus-error
//                    timeout on this board
//    11 BG, 19 VMA, 20 E = unconnected — no second master, no 6800 peripheral
//    29-52 A1-A23  = a[23:1].  There is no A0 anywhere on the board; the byte
//                    lane is /UDS //LDS and nothing else.
//    54-64,1-5 D0-D15 = the data bus.  The board bus BD is only connected to
//                    the CPU when A23 = 1 (the 7C/8B LS245 /G = /A23), so a
//                    program-ROM read comes straight off the ROMs and
//                    everything else over BD; `blstroid_main` muxes the read
//                    sources, which is the same thing with no tristates.
//
//  ---- three deviations from a literal transcription ----------------------
//  1. HALTn is tied inactive (1), not to the /RESET net.  On the board pins 17
//     and 18 are one net, so the watchdog stops and resets the CPU with a
//     single wire.  In fx68k `extReset` performs the whole 68000 reset and the
//     HALTn input is documented by its author as "used for single step only.
//     Force high if not used" — internally it only gates `busAvail` in the bus
//     arbiter, so driving it low alongside extReset would add a bus-grant
//     interaction the board does not have.  `oHALTEDn` (fx68k's
//     double-bus-fault output) has no load on sheet 2 and is left open.
//  2. `pwrUp` is driven from the same `reset`, which is what fx68k's own top
//     level does; on this board the /RESET net is a full CPU reset either way,
//     so a watchdog timeout re-inits the sequencer the way power-up does.
//  3. BERRn is tied inactive: sheet 2 has no bus-error source at all.
//
//  ---- /VPA and autovectoring --------------------------------------------
//  Sheet 2 makes /VPA = ~(AS & FC0 & FC1 & FC2) (1D LS20) and wires it both to
//  the 68000's VPA pin and to the 12C LS163A's ENT, so an FC = 7 cycle
//  deliberately gets no /DTACK and can only end through the CPU's own
//  E-synchronised VPA path.  Every interrupt on this board is autovectored and
//  there is no vector hardware anywhere; the ROM's vector table stops at
//  vector 63 and puts code at 0x100, 0x200 and 0x37E, so a core that let an
//  interrupt acknowledge terminate with /DTACK would vector into the middle of
//  a routine.
//
//  ---- the RESET instruction ---------------------------------------------
//  `oRESETn` is asserted while a `RESET` instruction runs, and the game
//  executes one on the way into the self test.  That net reaches only the
//  14/15B LS74 EEPROM unlock flip-flop's /CLR and the 14B LS08 that gates the
//  EEPROM /OE, so all it does is force-lock the EEPROM.
//============================================================================

module blstroid_cpu
(
	input  logic        clk,        // clk_sys, 57.272724 MHz
	input  logic        ce_7m,      // /7M enable, clk_sys / 8
	input  logic        reset,      // active high: the /RESET net + power-up
	input  logic  [2:0] ipl,        // active low, "111" = none (blstroid_irq)

	output logic [23:1] a,
	output logic [15:0] wdata,
	input  logic [15:0] rdata,
	output logic        as,         // active-high copy of /AS (8C LS14)
	output logic        uds_n,
	output logic        lds_n,
	output logic        rw,         // 1 = read, 0 = write
	input  logic        dtack,      // active high (= ~/DTACK)
	input  logic        vpa_n,      // 1D LS20, active low; 2C pin 21
	output logic  [2:0] fc,
	output logic        reset_inst  // 1 while a RESET instruction drives /RESET
);

	// ---------------------------------------------------------------- phases
	logic [2:0] ph;
	always_ff @(posedge clk) begin
		if (ce_7m) ph <= 3'd0;
		else       ph <= ph + 3'd1;
	end

	wire en_phi1 = ce_7m;          // CPU clock high phase (= the ce_7m tick)
	wire en_phi2 = (ph == 3'd3);   // 4 clk_sys later

	// ------------------------------------------------------------------- IPL
	// fx68k samples IPL under enPhi2 into its own two-deep synchroniser, so
	// ordinary posedge staging here is correct and the extra latency is
	// invisible at the ce_7m sampling rate.
	logic [2:0] ipl_r, ipl_rr;
	always_ff @(posedge clk) begin
		if (reset) begin
			ipl_r  <= 3'b111;
			ipl_rr <= 3'b111;
		end else begin
			ipl_r  <= ipl;
			ipl_rr <= ipl_r;
		end
	end

	// ----------------------------------------------------------------- fx68k
	wire        fx_as_n, fx_reset_n;
	wire [23:1] fx_eab;

	/* verilator lint_off UNUSEDSIGNAL */
	wire        fx_e, fx_vma_n;    // 6800 bus, unconnected on this board
	wire        fx_bg_n;           // no second bus master
	wire        fx_halted_n;       // double bus fault; no load on sheet 2
	/* verilator lint_on UNUSEDSIGNAL */

	assign a          = fx_eab;
	assign as         = ~fx_as_n;
	assign reset_inst = ~fx_reset_n;

	fx68k u_fx68k (
		.clk      (clk),
		.HALTn    (1'b1),          // see deviation 1 in the header
		.extReset (reset),
		.pwrUp    (reset),         // see deviation 2 in the header
		.enPhi1   (en_phi1),
		.enPhi2   (en_phi2),

		.eRWn     (rw),            // 1 = read, 0 = write
		.ASn      (fx_as_n),
		.LDSn     (lds_n),
		.UDSn     (uds_n),
		.E        (fx_e),
		.VMAn     (fx_vma_n),

		.FC0      (fc[0]),
		.FC1      (fc[1]),
		.FC2      (fc[2]),
		.BGn      (fx_bg_n),
		.oRESETn  (fx_reset_n),
		.oHALTEDn (fx_halted_n),

		.DTACKn   (~dtack),        // blstroid_dtack is active high here
		.VPAn     (vpa_n),         // 1D LS20 -> 2C pin 21
		.BERRn    (1'b1),          // 2C pin 22 = PR4, inactive
		.BRn      (1'b1),
		.BGACKn   (1'b1),
		.IPL0n    (ipl_rr[0]),
		.IPL1n    (ipl_rr[1]),
		.IPL2n    (ipl_rr[2]),

		.iEdb     (rdata),
		.oEdb     (wdata),
		.eab      (fx_eab)
	);

endmodule
