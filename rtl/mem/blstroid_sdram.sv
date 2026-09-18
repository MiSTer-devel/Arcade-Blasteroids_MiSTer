`timescale 1ns/1ps
//============================================================================
//  Blasteroids SDR-SDRAM controller for the MiSTer I/O-board memory module.
//
//  Copyright (C) 2026 RetroShrimp.  GPL-3.0-or-later; see LICENSE and
//  NOTICE.md.  Derived from the Xybots MiSTer core's rtl/mem/xybots_sdram.sv
//  (GPL-3.0, same author), which is itself from the Toobin' core's
//  toobin_sdram.sv, a SystemVerilog re-implementation of the
//  Arcade-Atari-system1 rtl/lib/mem/sdram.vhd command/mode encoding.
//
//  Target part: MT48LC16M16A2-class, 4 banks x 13-bit row x 9-bit col x16,
//  CL2, READ/WRITE with auto-precharge, AUTO_REFRESH interleaved while idle.
//
//  ---- host contract -------------------------------------------------------
//  When `ready` is high, pulse `req` for one cycle with `addr`/`we`(/`wdata`)
//  and `blen`.  `ready` drops for the access and returns high when it is done.
//  Read data returns on `rdata` with one `valid` pulse per word; a burst
//  returns its words on consecutive `valid` pulses, in address order.  The
//  controller latches a request that coincides with a due refresh, so a single-
//  cycle `req` pulse issued after observing `ready` is always safe.
//
//  `addr` is a 16-BIT-WORD address, {BA[1:0], ROW[12:0], COL[8:0]}.
//  `blen` = words-1 (0/1/3).  A burst group must be consecutive and must not
//  cross a row (512 words).  Both graphics clients are aligned by construction
//  — a playfield fetch is a 2-word group on a 2-word boundary and a motion-
//  object fetch a 4-word group on a 4-word boundary — so a burst can never
//  straddle a row.  One ACTIVE opens the row, then blen+1 READ commands issue
//  on consecutive cycles (CAS-pipelined) with auto-precharge (A10) asserted
//  only on the LAST read.  blen=0 is cycle-identical to a plain single-word
//  read.  Writes are always single-word.
//
//  ---- reset must be `~pll_locked`, never the game reset --------------------
//  `reset` drives SDRAM_CKE.  If a game or download reset can reach it, a reset
//  pulse during the ROM download drops CKE, restarts the 200 us init mid-
//  download and silently loses every loader write already made — a black screen
//  with no other symptom.  `blstroid_core` carries a separate `init_reset` port
//  precisely so this cannot be forgotten, and `Arcade-Blasteroids.sv` drives it
//  from `~pll_locked` and nothing else.
//
//  ---- SDRAM_CLK -----------------------------------------------------------
//  Not an output of this module.  `Arcade-Blasteroids.sv` drives the SDRAM_CLK
//  pin from the PLL's second, phase-shifted output (`outclk_1`, +13095 ps =
//  270 degrees at this clock rate), so the phase is a synthesis-time constant
//  that can be retuned without touching this controller.  Read data is captured
//  OPEN-LOOP at bcyc >= CL+1, so which edge the data lands on is a property of
//  the board rather than of the protocol; the shipped phase was chosen by
//  sweeping it on real hardware, which is the only thing that can choose it —
//  static timing analysis is unreliable here, and in simulation the chip model
//  clocks on the same edge as the controller.
//
//  ---- timing at clk_sys = 57.272727 MHz (period 17.4603 ns) ----------------
//  clk_sys is four times the 14.318181 MHz pixel clock, so one clock domain
//  serves the video pipeline and the memory alike.  CLK_KHZ = 57273 (kHz, so
//  the products stay inside a 32-bit int).  Delay minima use ceil(ns * f); the
//  refresh INTERVAL uses floor so the command is issued at 7.8125 us or
//  sooner, never one clock later through rounding.
//
//    tINIT  200 us  -> 11455 clk (200.00 us)   power-up wait
//    tRFC    66 ns  ->     4 clk ( 69.84 ns)   AUTO_REFRESH cycle time
//    tRC     60 ns  ->     4 clk ( 69.84 ns)   ACTIVE -> ACTIVE, same bank
//    tRCD    18 ns  ->     2 clk ( 34.92 ns)   ACTIVE -> READ/WRITE
//    tRP     18 ns  ->     2 clk ( 34.92 ns)   PRECHARGE recovery
//    tMRD    12 ns  ->     1 clk (2 clk gap)   LOAD MODE -> next command
//    tREFI 7.8125us ->   446 clk (  7.787 us)  AUTO_REFRESH interval
//    CL             =     2 clk                read latency (mode register)
//
//  tRC has no counter of its own because no state path can violate it: the
//  shortest possible ACTIVE-to-ACTIVE gap is a single write (6 clk = 104.8 ns),
//  a burst-2 read is 10 clk and a burst-4 read 12 clk, all well over the 4 clk
//  minimum.  The as-built gaps are more conservative than the minima above
//  because the registered command pins add a cycle to each wait: ACTIVE->READ
//  is 3 clk (52.4 ns) rather than 2, and the last READ's auto-precharge gets
//  6 clk before the next ACTIVE rather than the 3 it needs.
//
//  The 18 ns tRCD/tRP are conservative for both the -7E Micron parts (15 ns)
//  and the AS4C -6 family (18 ns).  tRFC is 66 ns even on a -7E whose ordinary
//  ACTIVE-to-ACTIVE tRC is 60 ns, so AUTO_REFRESH recovery uses 66.
//
//  Transaction lengths, in clk_sys cycles, from leaving S_IDLE:
//    burst-4 read : ACT 1 + tRCD 2 + BURST 7 + tRP 2 = 12 ; words at +8..+11
//    burst-2 read : ACT 1 + tRCD 2 + BURST 5 + tRP 2 = 10 ; words at +8, +9
//    single write : ACT 1 + tRCD 2 + WR    1 + tRP 2 =  6
//    refresh      : REF 1 + tRFC 4                   =  5
//
//  ---- two things that look like bugs and are not ---------------------------
//    * `rfsh_req` is both set by the interval counter and cleared by S_IDLE in
//      the same always_ff.  When they collide the clear wins — but S_IDLE is
//      simultaneously entering S_REF, so the refresh still happens and the
//      counter has just restarted.  Nothing is lost.
//    * `ready` is computed from the OLD `rfsh_req`, so a host can pulse `req`
//      in the very cycle a refresh becomes due.  The `pending` latch in S_IDLE
//      exists for exactly that race, and the request is served after the
//      refresh.
//
//  Command, address and write-data pins are REGISTERED off the combinational
//  state decode: at this clock rate the state->pin path fails address setup
//  (tIS) against the phase-shifted SDRAM_CLK, and registering makes each a
//  clean reg->pin path Quartus packs into the I/O output registers.  The
//  uniform one-cycle output delay is absorbed by capturing read data one cycle
//  later (bcyc >= CL+1), so every relative timing is preserved.
//
//  The address and bank output registers HOLD on the cycles that carry no
//  address instead of being driven to zero: the chip ignores A[12:0]/BA[1:0] on
//  NOP and AUTO REFRESH, so holding and clearing are indistinguishable to the
//  SDRAM and holding halves the switching on thirteen memory-bus pins.
//============================================================================

module blstroid_sdram #(
	parameter int CLK_KHZ  = 57273,   // controller clock in kHz
	parameter int ROW_BITS = 13,
	parameter int COL_BITS = 9
) (
	input  logic        clk,
	input  logic        reset,        // MUST be ~pll_locked only (see header)

	// ---- host word port ----
	input  logic [ROW_BITS+COL_BITS+1:0] addr,
	input  logic [15:0] wdata,
	input  logic        we,
	input  logic [1:0]  blen,         // read burst: words-1 (0/1/3)
	input  logic        req,
	output logic [15:0] rdata,
	output logic        valid,
	output logic        ready,

	// ---- SDRAM chip pins (SDRAM_CLK is driven in Arcade-Blasteroids.sv) ----
	inout  wire  [15:0] SDRAM_DQ,
	output logic [12:0] SDRAM_A,
	output logic [1:0]  SDRAM_BA,
	output logic        SDRAM_DQML,
	output logic        SDRAM_DQMH,
	output logic        SDRAM_CKE,
	output logic        SDRAM_nCS,
	output logic        SDRAM_nRAS,
	output logic        SDRAM_nCAS,
	output logic        SDRAM_nWE
);

// ---- timing, in clk cycles (see the header table) ---------------------------
// CLK_KHZ is cycles per millisecond, so ns -> cycles is  ns * CLK_KHZ / 1e6
// and us -> cycles is  us * CLK_KHZ / 1e3.  Written that way to keep every
// intermediate product inside a 32-bit int (200000 * 57273 would not fit).
localparam int tINIT = (   200*CLK_KHZ +     999) /    1000;   // 200 us, ceil
localparam int tRFC  = (    66*CLK_KHZ +  999999) / 1000000;   //  66 ns, ceil
localparam int tRCD  = (    18*CLK_KHZ +  999999) / 1000000;   //  18 ns, ceil
localparam int tRP   = (    18*CLK_KHZ +  999999) / 1000000;   //  18 ns, ceil
localparam int tMRD  = (    12*CLK_KHZ +  999999) / 1000000;   //  12 ns, ceil
localparam int tREFI_RAW = (7800*CLK_KHZ) / 1000000;           // 7.8 us, floor
localparam int tREFI = (tREFI_RAW > 0) ? tREFI_RAW : 1;
localparam logic [15:0] tREFI_LAST = 16'(tREFI - 1);
localparam int CL = 2;

localparam logic [3:0] CMD_LMR=4'b0000, CMD_REFRESH=4'b0001, CMD_PRECHARGE=4'b0010,
                       CMD_ACTIVE=4'b0011, CMD_WRITE=4'b0100, CMD_READ=4'b0101,
                       CMD_NOP=4'b0111;
// mode register: burst length 1, sequential, CL=2, standard operation, single write
localparam logic [12:0] MODE_REG = {3'b000, 1'b1, 2'b00, 3'b010, 1'b0, 3'b000};

localparam int AW     = ROW_BITS + COL_BITS + 2;
localparam int BA_HI  = AW-1,  BA_LO  = AW-2;
localparam int ROW_HI = COL_BITS + ROW_BITS - 1, ROW_LO = COL_BITS;

typedef enum logic [3:0] {
	S_INIT, S_PRE, S_TRP_I, S_REFI, S_TRC_I, S_MRD, S_TMRD,
	S_IDLE, S_ACT, S_TRCD, S_BURST, S_WR, S_RECOV, S_REF, S_TRC
} state_t;
state_t state;

logic [15:0] dly;
logic [3:0]  ref_init;
logic [15:0] rfsh_ctr;
logic        rfsh_req;

logic           we_l;
logic [15:0]    wdata_l;
logic [AW-1:0]  addr_l;
logic [1:0]     blen_l;
logic [2:0]     bcyc;             // burst cycle: READs at 0..blen_l, data at CL+1..
logic           pending;          // a captured request awaiting service

// ---- combinational command / address / DQ-OE from the single-cycle states ----
logic [3:0]  cmd;
logic [12:0] a_comb;
logic [1:0]  ba_comb;
logic        dq_oe;

// Column address on A; A10 = auto-precharge.  COL_BITS<=9 so the column sits in
// A[8:0] and A10 is free.  Built as one concatenation (iverilog mishandles
// partial bit-selects inside always_comb).  A10 asserts only on the LAST read of
// a burst so the row stays open for the intermediate reads; writes enter S_WR
// with bcyc==0 and blen_l==0 and so always auto-precharge.
wire [COL_BITS-1:0] col_cur  = addr_l[COL_BITS-1:0] + {{(COL_BITS-3){1'b0}}, bcyc};
wire                ap_last  = (bcyc == {1'b0, blen_l});
wire [12:0] col_a = {2'b00, ap_last, {(10-COL_BITS){1'b0}}, col_cur};
localparam logic [12:0] PRE_ALL = 13'h0400; // A10=1 = precharge all banks

// Pre-extracted as continuous-assign wires; iverilog mishandles constant
// part-selects inside always_* blocks (drives "all bits").
wire [1:0]           ba_w  = addr_l[BA_HI:BA_LO];
wire [ROW_BITS-1:0]  row_w = addr_l[ROW_HI:ROW_LO];
wire [12:0]          row_a = {{(13-ROW_BITS){1'b0}}, row_w};

always_comb begin
	cmd = CMD_NOP; a_comb = '0; ba_comb = '0; dq_oe = 1'b0;
	case (state)
		S_PRE:  begin cmd = CMD_PRECHARGE; a_comb = PRE_ALL; end
		S_REFI: cmd = CMD_REFRESH;
		S_MRD:  begin cmd = CMD_LMR; a_comb = MODE_REG; end
		S_ACT:  begin cmd = CMD_ACTIVE; ba_comb = ba_w; a_comb = row_a; end
		S_BURST: if (bcyc <= {1'b0, blen_l}) begin cmd = CMD_READ; ba_comb = ba_w; a_comb = col_a; end
		S_WR:   begin cmd = CMD_WRITE; ba_comb = ba_w; a_comb = col_a; dq_oe = 1'b1; end
		S_REF:  cmd = CMD_REFRESH;
		default: ;
	endcase
end

logic [3:0]  cmd_r;  logic [12:0] a_r;  logic [1:0] ba_r;
logic        dqoe_r; logic [15:0] wdata_r;

// `cmd_r` keeps its reset: a clear ALONE is packable into an I/O register.
wire cmd_has_addr = (cmd == CMD_PRECHARGE) || (cmd == CMD_LMR)
                 || (cmd == CMD_ACTIVE)    || (cmd == CMD_READ)
                 || (cmd == CMD_WRITE);

always_ff @(posedge clk) begin
	if (reset) begin cmd_r <= CMD_NOP; dqoe_r <= 1'b0; end
	else       begin cmd_r <= cmd;     dqoe_r <= dq_oe; end
	if (cmd_has_addr) begin a_r <= a_comb; ba_r <= ba_comb; end
	wdata_r <= wdata_l;
end

assign SDRAM_CKE = ~reset;
assign {SDRAM_nCS, SDRAM_nRAS, SDRAM_nCAS, SDRAM_nWE} = cmd_r;
assign SDRAM_A   = a_r;
assign SDRAM_BA  = ba_r;
assign SDRAM_DQ  = dqoe_r ? wdata_r : 16'hzzzz;
assign {SDRAM_DQMH, SDRAM_DQML} = 2'b00;
assign ready     = (state == S_IDLE) && !rfsh_req && !pending;

// helper: wait state that decrements dly, jumps to NEXT when it hits 0.
`define BL_SDRAM_WAIT(NEXT) begin if (dly != 0) dly <= dly - 1'b1; else state <= NEXT; end

always_ff @(posedge clk) begin
	valid <= 1'b0;

	if (state != S_INIT && state != S_PRE && state != S_TRP_I &&
	    state != S_REFI && state != S_TRC_I && state != S_MRD && state != S_TMRD) begin
		if (rfsh_ctr >= tREFI_LAST) begin rfsh_ctr <= '0; rfsh_req <= 1'b1; end
		else rfsh_ctr <= rfsh_ctr + 1'b1;
	end

	if (reset) begin
		state <= S_INIT; dly <= tINIT[15:0]; ref_init <= '0;
		rfsh_ctr <= '0; rfsh_req <= 1'b0; pending <= 1'b0;
	end else begin
		case (state)
		// ---- power-up init ----
		S_INIT:  begin if (dly != 0) dly <= dly - 1'b1; else state <= S_PRE; end
		S_PRE:   begin state <= S_TRP_I; dly <= tRP[15:0]-1'b1; end
		S_TRP_I: begin if (dly != 0) dly <= dly-1'b1; else begin state <= S_REFI; ref_init <= '0; end end
		S_REFI:  begin state <= S_TRC_I; dly <= tRFC[15:0]-1'b1; end
		S_TRC_I: begin if (dly != 0) dly <= dly-1'b1;
		               else if (ref_init == 4'd7) state <= S_MRD;
		               else begin ref_init <= ref_init + 1'b1; state <= S_REFI; end end
		S_MRD:   begin state <= S_TMRD; dly <= tMRD[15:0]-1'b1; end
		S_TMRD:  `BL_SDRAM_WAIT(S_IDLE)

		// ---- normal operation ----
		S_IDLE: begin
			// Capture an incoming request so a coincident refresh cannot drop it.
			if (req && !pending) begin
				we_l <= we; wdata_l <= wdata; addr_l <= addr; pending <= 1'b1;
				blen_l <= we ? 2'd0 : blen;          // writes are always single-word
			end
			if (rfsh_req) begin state <= S_REF; rfsh_req <= 1'b0; end
			else if (pending || req) begin pending <= 1'b0; state <= S_ACT; end
		end
		S_ACT:   begin state <= S_TRCD; dly <= tRCD[15:0]-1'b1; end
		S_TRCD:  begin if (dly != 0) dly <= dly-1'b1; else begin state <= we_l ? S_WR : S_BURST; bcyc <= 3'd0; end end
		// READ commands issue at bcyc 0..blen_l (the registered outputs shift them
		// one clk); each word's DQ is captured CL+1 cycles after its READ, i.e. at
		// bcyc CL+1 .. CL+1+blen_l, on consecutive `valid` pulses.
		S_BURST: begin
			bcyc <= bcyc + 3'd1;
			if (bcyc >= 3'(CL+1)) begin rdata <= SDRAM_DQ; valid <= 1'b1; end
			if (bcyc == (3'(CL+1) + {1'b0, blen_l})) begin state <= S_RECOV; dly <= tRP[15:0]-1'b1; end
		end
		S_WR:    begin state <= S_RECOV; dly <= tRP[15:0]-1'b1; end
		S_RECOV: `BL_SDRAM_WAIT(S_IDLE)
		S_REF:   begin state <= S_TRC; dly <= tRFC[15:0]-1'b1; end
		S_TRC:   `BL_SDRAM_WAIT(S_IDLE)
		default: state <= S_IDLE;
		endcase
	end
end

`undef BL_SDRAM_WAIT

endmodule
