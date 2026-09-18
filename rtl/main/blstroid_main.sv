`timescale 1ns/1ps
//============================================================================
//  Blasteroids main board — SP-316 sheets 2, 3, 4, 5 and 12 as one block.
//
//  Copyright (C) 2026 RetroShrimp.  GPL-3.0-or-later; see LICENSE and
//  NOTICE.md.  Structure from the Xybots MiSTer core's
//  rtl/main/xybots_main_bus.sv (GPL-3.0, same author); the board-specific
//  parts are read off SP-316.
//
//  Composes: blstroid_cpu (fx68k) + blstroid_addr_decode + blstroid_dtack +
//  blstroid_irq + blstroid_watchdog + blstroid_eeprom_2804 +
//  blstroid_nvram_io + blstroid_inputs + blstroid_leta + the 16 KB video/work
//  RAM (5L, 6/7L), the 512 x 16 colour RAM and the 256 x 1 priority RAM.
//
//  ---- what is a port and why --------------------------------------------
//  * The program ROM is a read port, shaped like blstroid_prog_rom's
//    (`rd_a` = A17..A1, one clk_sys of latency).  There is no slapstic and no
//    bank substitution on this board (the 2A socket is empty and the 4A LS157
//    is strapped through by R77), so the raw address is all the ROM needs.
//  * The video RAM, the colour RAM and the priority RAM live inside this
//    module, because their CPU-side arbitration is sheet 2/5 logic.  Their
//    video sides leave here as read-only ports for the video block.
//  * `h3`, `v3`, `h8`, `hblank`, `vblank` and `pfm2` come from the video block.
//    `pfm2` is sheet 6's 4K LS174B pin 2 — bit 15 of the last playfield word
//    fetched, latched at 4H — and it is what arms the row interrupt; see
//    blstroid_irq.  The video block owns it; this module only gates it.
//
//  ---- the video-RAM slot schedule ---------------------------------------
//  The 6E/5E LS257 address mux is output-enabled by /2H and A/B-selected by
//  ~(/4H & 1H), which puts the eight H counts of a group at
//
//      h = {4H,2H,1H}   0    1    2    3    4    5    6    7
//      owner            MO   MO   CPU  PF   MO   MO   CPU  CPU
//      RAMFREE          0    0    1    0    0    0    1    1
//      /DTACK clock     -    X    -    -    -    X    X    -
//
//  and that is independently confirmed by the sheet-2 RAMFREE term
//  (11J LS02: RAMFREE = 2H & (/1H | 4H) = 1 at h = 2, 6, 7).  A CPU write only
//  reaches the RAM inside one of those three slots (the /VMEMH //VMEML strobes
//  are gated by /RAMCPU), and a CPU read is captured by the 10D/8D LS373
//  latches, which are transparent while RAMFREE.  The CPU is made to wait for
//  its slot by the 11D LS74 in `blstroid_dtack`, not by anything here.
//
//  Blasteroids has no alphanumerics layer, which is why the slot Xybots spends
//  on its alpha plane is a third CPU slot here.
//
//  ---- the /RESET net -----------------------------------------------------
//  The watchdog's /RESET drives the 68000's RESET+HALT pins (one net) and,
//  leaving sheet 2 for sheet 3 only, the EEPROM unlock flip-flop's /CLR and
//  the 14B LS08 that gates the EEPROM /OE.  A `RESET` instruction pulls the
//  same net low through the CPU's open-drain pin, so it force-locks the EEPROM
//  and does nothing else — in particular it does not reset the SOS-2, the
//  LETA, the SCOM or the audio board (that is FF8C00 -> SCOM /RESREQ).
//
//  ---- reads of write-only strobes ---------------------------------------
//  Nothing drives the board bus for them, so they read 0xFFFF, which is also
//  MAME's `unmap_value_high()`.  The game does this on purpose in three
//  places: the read-modify-writes to FF8C00 (audio reset) and FF8000
//  (watchdog), and the priority-RAM fill.  /DTACK does not depend on any
//  strobe, so all of them terminate exactly like a ROM cycle.
//============================================================================

module blstroid_main #(
	parameter int  CLK_HZ          = 57_272_724,
	parameter int  EE_WRITE_CYCLES = CLK_HZ / 100,   // X2804A 10 ms bound
	parameter bit  WDOGDIS         = 1'b0            // watchdog disable jumper
)(
	input  logic        clk,          // clk_sys 57.272724 MHz
	input  logic        ce_7m,        // /7M enable, clk_sys / 8
	input  logic        init_reset,   // FPGA init / ROM not loaded yet
	input  logic        por,          // sheet-2 power-on reset, active high

	// ---- video timing and the row-interrupt input, from the video block ----
	input  logic  [2:0] h3,           // {4H, 2H, 1H}
	input  logic  [2:0] v3,           // {4V, 2V, 1V}
	input  logic        h8,           // 8H — the LETA CK pin
	input  logic        hblank,       // active high
	input  logic        vblank,       // active high
	input  logic        pfm2,         // sheet 6, 4K LS174B pin 2 (latched VD15)

	// ---- program ROM read port (blstroid_prog_rom) ----
	output logic [16:0] prog_a,       // 68000 A17..A1
	input  logic [15:0] prog_data,    // valid 1 clk_sys after the address

	// ---- EEPROM factory image, from the index-0 download tail ----
	input  logic        eeprom_img_wr,
	input  logic  [8:0] eeprom_img_addr,
	input  logic  [7:0] eeprom_img_data,

	// ---- video-side read ports ----
	input  logic [12:0] vram_vaddr,   // 8 K words at FFC000 (PF, MO, work)
	output logic [15:0] vram_vdata,
	input  logic  [8:0] cram_vaddr,   // 512 words at FFA000
	output logic [15:0] cram_vdata,
	input  logic  [7:0] prio_vaddr,   // H PPP MMMM
	output logic        prio_vdata,   // 1 = motion object on top

	// ---- control panel, active-HIGH "pressed" ----
	//   [0] FIRE  [1] THRUST  [2] TRANSFORM  [3] SPARE (unwired in a cabinet)
	input  logic  [3:0] p1_buttons,
	input  logic  [3:0] p2_buttons,
	input  logic        self_test,    // 1 = the self-test switch is ON
	input  logic  [1:0] whirly_a,     // LET CLK0 / CLK1
	input  logic  [1:0] whirly_b,     // LET DIR0 / DIR1

	// ---- JSA I sound mailbox (SCOM 9A) ----
	output logic        snd_cmd_wr,   // FF8A01 write strobe
	output logic  [7:0] snd_cmd_data,
	output logic        snd_resp_rd,  // FF9401 access strobe (clears IRQ4)
	output logic        snd_reset,    // FF8C00 write strobe -> /RESREQ
	input  logic  [7:0] snd_resp_data,
	input  logic        snd_irq,      // SCOM /RES asserted -> level 4
	input  logic        snd_m2s_ready,// 1 = a command is still pending

	// ---- MiSTer NVRAM (index 2) ----
	input  logic        ioctl_download,
	input  logic        ioctl_wr,
	input  logic [26:0] ioctl_addr,
	input  logic  [7:0] ioctl_dout,
	input  logic [15:0] ioctl_index,
	input  logic        ioctl_upload,
	output logic        ioctl_upload_req,
	output logic  [7:0] ioctl_upload_index,
	output logic  [7:0] ioctl_din,

	// ---- observation / debug ----
	output logic [23:1] cpu_a,
	output logic [15:0] cpu_dout,
	output logic [15:0] cpu_din,
	output logic        cpu_as,
	output logic        cpu_uds_n,
	output logic        cpu_lds_n,
	output logic        cpu_rw,
	output logic        cpu_dtack,
	output logic  [2:0] cpu_fc,
	output logic  [2:0] ipl,
	output logic        watchdog_reset,
	output logic        reset_net,    // the sheet-2 /RESET net, active high here
	output logic        irq1_pending,
	output logic        irq2_pending
);

	// =====================================================================
	//  CPU (2C)
	// =====================================================================
	logic        cpu_reset, reset_inst;
	logic [15:0] cpu_rdata_r;
	logic        vpa_n;               // 1D LS20: the CPU pin AND the 12C ENT

	blstroid_cpu u_cpu (
		.clk(clk), .ce_7m(ce_7m), .reset(cpu_reset), .ipl(ipl),
		.a(cpu_a), .wdata(cpu_dout), .rdata(cpu_rdata_r),
		.as(cpu_as), .uds_n(cpu_uds_n), .lds_n(cpu_lds_n), .rw(cpu_rw),
		.dtack(cpu_dtack), .vpa_n(vpa_n), .fc(cpu_fc),
		.reset_inst(reset_inst));

	assign cpu_din = cpu_rdata_r;

	// =====================================================================
	//  Decode (sheet 3)
	// =====================================================================
	logic r_n, w_n, wh_n, wl_n, lane_hi, lane_lo;
	logic rom_n, slapstic_n, vidram_n, cram_n, eerom_n;
	logic io9000_n, audrd_n, leta_n, inputs_n;
	logic wdog_n, irqack_n, vblack_n, unlock_n, priram_n, audwr_n, audres_n,
	      waithbl_n;

	blstroid_addr_decode u_dec (
		.a(cpu_a), .as(cpu_as), .uds_n(cpu_uds_n), .lds_n(cpu_lds_n),
		.rw(cpu_rw), .fc(cpu_fc),
		.r_n(r_n), .w_n(w_n), .wh_n(wh_n), .wl_n(wl_n),
		.lane_hi(lane_hi), .lane_lo(lane_lo),
		.rom_n(rom_n), .slapstic_n(slapstic_n), .vidram_n(vidram_n),
		.cram_n(cram_n), .eerom_n(eerom_n),
		.io9000_n(io9000_n), .audrd_n(audrd_n), .leta_n(leta_n),
		.inputs_n(inputs_n),
		.wdog_n(wdog_n), .irqack_n(irqack_n), .vblack_n(vblack_n),
		.unlock_n(unlock_n), .priram_n(priram_n), .audwr_n(audwr_n),
		.audres_n(audres_n), .waithbl_n(waithbl_n),
		.vpa_n(vpa_n));

	// /SLAPSTIK is decoded because sheet 3 draws it and reaches nothing (the
	// 2A socket is empty); FF9000 is an unused LS139 output.  Both are read
	// here so the tools see them used and so a future revision has a hook.
	/* verilator lint_off UNUSEDSIGNAL */
	wire unused_dec = &{1'b0, slapstic_n, io9000_n, r_n, w_n, lane_hi, lane_lo};
	/* verilator lint_on UNUSEDSIGNAL */

	assign prog_a = cpu_a[17:1];

	// =====================================================================
	//  Bus-cycle edges
	// =====================================================================
	// A 68000 asserts /AS in S2 and the data strobes in S4 for a WRITE, so the
	// write one-shot fires on the data strobe, not on /AS.
	logic as_d, ws_d;
	wire  ws = cpu_as & ~cpu_rw & (~cpu_uds_n | ~cpu_lds_n);
	always_ff @(posedge clk) begin
		if (init_reset) begin as_d <= 1'b0; ws_d <= 1'b0; end
		else            begin as_d <= cpu_as; ws_d <= ws;  end
	end
	wire cyc_start = cpu_as & ~as_d;   // falling edge of /AS
	wire wr_stb    = ws & ~ws_d;       // one write strobe per bus cycle

	// =====================================================================
	//  Strobes
	// =====================================================================
	// The 10C LS138 is qualified by /W and nothing else — no /UDS, no /LDS —
	// so a byte write on either lane strobes these exactly like a word write.
	// The game writes all of them as words with don't-care data.
	wire wdog_wr   = wr_stb & ~wdog_n;
	wire irqack_wr = wr_stb & ~irqack_n;
	wire vblack_wr = wr_stb & ~vblack_n;
	wire unlock_wr = wr_stb & ~unlock_n;
	wire priram_wr = wr_stb & ~priram_n;
	wire audwr_wr  = wr_stb & ~audwr_n;
	wire audres_wr = wr_stb & ~audres_n;

	// The 10B LS139 read decoder has no R//W term at all, so /AUDRD reaches the
	// SCOM's /RD pin on a write to FF9400 too.  The game never does that; the
	// strobe is faithful anyway.
	wire audrd_stb = cyc_start & ~audrd_n;

	assign snd_cmd_wr   = audwr_wr;
	assign snd_cmd_data = cpu_dout[7:0];
	assign snd_resp_rd  = audrd_stb;
	assign snd_reset    = audres_wr;

	// =====================================================================
	//  Video / work RAM (5L, 6/7L) — 8 K words, true byte lanes, CPU slots
	// =====================================================================
	// 11E LS32 gives 6/7L its own /WE from /WH and 5L its own /WE from /WL, so
	// byte writes on either lane are honoured — and they must be: the game
	// writes playfield RAM a byte at a time, so a word-only port corrupts it.
	wire ramfree = (h3 == 3'd2) | (h3 == 3'd6) | (h3 == 3'd7);

	wire         vram_wr_req = wr_stb & ~vidram_n;
	logic        vram_wr_pend, vram_wpend_hi, vram_wpend_lo;
	logic [12:0] vram_wa;
	logic [15:0] vram_wd;

	always_ff @(posedge clk) begin
		if (init_reset) vram_wr_pend <= 1'b0;
		else if (vram_wr_req) begin
			vram_wa       <= cpu_a[13:1];
			vram_wd       <= cpu_dout;
			vram_wpend_hi <= ~wh_n;
			vram_wpend_lo <= ~wl_n;
			vram_wr_pend  <= ~ramfree;
		end else if (ramfree) begin
			vram_wr_pend  <= 1'b0;
		end
	end

	wire vram_we_now = (vram_wr_req | vram_wr_pend) & ramfree;
	wire vram_we_hi  = vram_we_now & (vram_wr_req ? ~wh_n : vram_wpend_hi);
	wire vram_we_lo  = vram_we_now & (vram_wr_req ? ~wl_n : vram_wpend_lo);

	logic [15:0] vram_dout;
	blstroid_dpram_be #(.AW(13)) u_vram (
		.clk(clk),
		.a_addr(vram_wr_pend ? vram_wa : cpu_a[13:1]),
		.a_din (vram_wr_pend ? vram_wd : cpu_dout),
		.a_we_hi(vram_we_hi), .a_we_lo(vram_we_lo), .a_dout(vram_dout),
		.b_addr(vram_vaddr), .b_dout(vram_vdata));

	// The 10D/8D LS373 read latches are transparent while RAMFREE and output
	// enabled only during a video-RAM read, so what the CPU sees is the word
	// captured in a CPU slot.  `ramfree_d` aligns the capture with the block
	// RAM's registered output.
	logic        ramfree_d;
	logic [15:0] vram_rdata;
	always_ff @(posedge clk) begin
		ramfree_d <= ramfree;
		if (ramfree_d) vram_rdata <= vram_dout;
	end

	// =====================================================================
	//  Colour RAM — 512 x 16 at FFA000-FFA3FE
	// =====================================================================
	// Sheet 12: XX0P PPPC CCC0 = 256 motion-object entries (16 palettes x 16)
	// at FFA000-FFA1FE, XX10 PPPC CCC0 = 128 playfield entries (8 x 16) at
	// FFA200-FFA2FE, and FFA300-FFA3FE is addressable RAM nothing uses.  The
	// colour RAM is arbitrated by the 14C F74 pair on sheet 10 and does not
	// gate /DTACK: it takes the normal one-/7M-edge acknowledgement.  The game
	// only ever touches it with word accesses, so the per-lane behaviour is
	// unobservable here; both lanes are wired independently anyway.
	wire cram_we_hi = wr_stb & ~cram_n & ~wh_n;
	wire cram_we_lo = wr_stb & ~cram_n & ~wl_n;
	logic [15:0] cram_rdata;
	blstroid_dpram_be #(.AW(9)) u_cram (
		.clk(clk),
		.a_addr(cpu_a[9:1]), .a_din(cpu_dout),
		.a_we_hi(cram_we_hi), .a_we_lo(cram_we_lo), .a_dout(cram_rdata),
		.b_addr(cram_vaddr), .b_dout(cram_vdata));

	// =====================================================================
	//  Priority RAM — 256 x 1, write only (sheet 10, 17E 27LS00A)
	// =====================================================================
	// Sheet 12: FF8800-FF89FE, address `H PPP MMMM` = BA8..BA1 (H = playfield
	// pen bit 3, P = playfield palette, M = motion-object palette), data D0,
	// 1 = motion object on top.  The LS138 strobe has no lane qualifier and the
	// game writes it as a word.  This is the only strobe whose A8..A1 are not
	// don't-care.  A CPU read returns the undriven bus (0xFFFF); MAME keeps a
	// readable shadow instead, so the two disagree there.
	logic prio_mem [0:255];
`ifndef ALTERA_RESERVED_QIS
	initial for (int i = 0; i < 256; i++) prio_mem[i] = 1'b0;
`endif
	always_ff @(posedge clk) begin
		if (priram_wr) prio_mem[cpu_a[8:1]] <= cpu_dout[0];
		prio_vdata <= prio_mem[prio_vaddr];
	end

	// =====================================================================
	//  Watchdog (1B LS90) and the /RESET net
	// =====================================================================
	logic vblank_d;
	always_ff @(posedge clk) vblank_d <= vblank;
	wire vblank_start = vblank & ~vblank_d;      // falling edge of /VBLANK

	logic wd_reset_n;
	/* verilator lint_off UNUSEDSIGNAL */
	wire [3:0] wd_count;
	/* verilator lint_on UNUSEDSIGNAL */
	blstroid_watchdog #(.WDOGDIS(WDOGDIS)) u_wdog (
		.clk(clk), .por(por | init_reset),
		.vblank_start(vblank_start), .wdog_clr(wdog_wr),
		.reset_n(wd_reset_n), .count(wd_count));

	assign watchdog_reset = ~wd_reset_n;
	assign cpu_reset      = watchdog_reset | init_reset;
	// 2C pins 17 and 18 are the same net, and the CPU's own RESET instruction
	// pulls it low without resetting the CPU.
	assign reset_net      = watchdog_reset | init_reset | reset_inst;

	// =====================================================================
	//  Interrupts (sheet 2)
	// =====================================================================
	// `reset` here is power-up only: neither 13C half has a usable /CLR, so a
	// watchdog timeout must not drop a pending interrupt.
	blstroid_irq u_irq (
		.clk(clk), .reset(init_reset),
		.pfm2(pfm2), .v(v3), .hblank(hblank), .vblank(vblank),
		.irqack_wr(irqack_wr), .vblack_wr(vblack_wr), .snd_irq(snd_irq),
		.ipl(ipl), .irq1_pending(irq1_pending), .irq2_pending(irq2_pending));

	// =====================================================================
	//  EEPROM (2A/B) + unlock latch + MiSTer NVRAM
	// =====================================================================
	wire eeprom_we = wr_stb & ~eerom_n & ~wl_n;   // /CE = /EEROM, /WE = /WL
	wire low_write = wr_stb & ~wl_n;              // the board-global /WL edge

	logic [7:0] ee_rdata, ee_load_data, ee_dump_data;
	logic [8:0] ee_load_addr, ee_dump_addr;
	logic       ee_load_we, ee_write_accepted;
	/* verilator lint_off UNUSEDSIGNAL */
	wire        ee_busy, ee_oe_n, ee_unlocked;
	/* verilator lint_on UNUSEDSIGNAL */

	// Two restore paths reach the same 512 cells and carry the same bytes: the
	// ROM loader can strip the MRA's 0x200-byte EEPROM tail out of the index-0
	// download, and blstroid_nvram_io takes the `<nvram index="2">` channel
	// (and, on its own, the same index-0 tail).  Wire either or both.
	wire       ee_wr_any  = ee_load_we | eeprom_img_wr;
	wire [8:0] ee_wr_addr = eeprom_img_wr ? eeprom_img_addr : ee_load_addr;
	wire [7:0] ee_wr_data = eeprom_img_wr ? eeprom_img_data : ee_load_data;

	blstroid_eeprom_2804 #(.CLK_HZ(CLK_HZ), .WRITE_CYCLES(EE_WRITE_CYCLES))
	u_eeprom (
		.clk(clk), .init_reset(init_reset), .reset(reset_net),
		.unlock(unlock_wr), .low_write(low_write), .cpu_we(eeprom_we),
		.cpu_addr(cpu_a[9:1]), .cpu_wdata(cpu_dout[7:0]), .cpu_rdata(ee_rdata),
		.busy(ee_busy), .oe_n(ee_oe_n), .unlocked(ee_unlocked),
		.write_accepted(ee_write_accepted),
		.load_we(ee_wr_any), .load_addr(ee_wr_addr), .load_data(ee_wr_data),
		.dump_addr(ee_dump_addr), .dump_data(ee_dump_data));

	blstroid_nvram_io u_nvram (
		.clk(clk), .init_reset(init_reset), .write_accepted(ee_write_accepted),
		.ioctl_download(ioctl_download), .ioctl_wr(ioctl_wr),
		.ioctl_addr(ioctl_addr), .ioctl_dout(ioctl_dout),
		.ioctl_index(ioctl_index), .ioctl_upload(ioctl_upload),
		.load_we(ee_load_we), .load_addr(ee_load_addr), .load_data(ee_load_data),
		.dump_addr(ee_dump_addr), .dump_data(ee_dump_data),
		.ioctl_upload_req(ioctl_upload_req),
		.ioctl_upload_index(ioctl_upload_index), .ioctl_din(ioctl_din));

	// =====================================================================
	//  Inputs (12L F244 + 10A LS257) and the Whirly-Gigs (LETA 8A)
	// =====================================================================
	logic [15:0] inputs_port;
	blstroid_inputs u_inputs (
		.p1_fire(p1_buttons[0]), .p1_thrust(p1_buttons[1]),
		.p1_transform(p1_buttons[2]), .p1_spare(p1_buttons[3]),
		.p2_fire(p2_buttons[0]), .p2_thrust(p2_buttons[1]),
		.p2_transform(p2_buttons[2]), .p2_spare(p2_buttons[3]),
		.self_test(self_test), .snd_m2s_ready(snd_m2s_ready),
		.vblank(vblank), .hblank(hblank),
		.a1(cpu_a[1]), .port(inputs_port));

	logic [7:0] leta_dout;
	/* verilator lint_off UNUSEDSIGNAL */
	wire [7:0] leta_c0, leta_c1;
	/* verilator lint_on UNUSEDSIGNAL */
	blstroid_leta u_leta (
		.clk(clk), .init_reset(init_reset), .h8(h8),
		.enc_a(whirly_a), .enc_b(whirly_b),
		.a2(cpu_a[2]), .dout(leta_dout),
		.count0(leta_c0), .count1(leta_c1));

	// =====================================================================
	//  Read data
	// =====================================================================
	// Only the ROMs (A23 = 0) and the devices behind the 7C/8B LS245 buffers
	// (A23 = 1) ever drive the CPU bus.  The EEPROM, the SCOM, the LETA, the
	// status buffer and the button mux are all on BD7:0 and nothing drives
	// BD15:8 for them, so the high lane reads as the undriven bus.
	logic [15:0] rdata_comb;
	always_comb begin
		if      (!rom_n)    rdata_comb = prog_data;
		else if (!vidram_n) rdata_comb = vram_rdata;
		else if (!cram_n)   rdata_comb = cram_rdata;
		else if (!eerom_n)  rdata_comb = {8'hFF, ee_rdata};
		else if (!audrd_n)  rdata_comb = {8'hFF, snd_resp_data};
		else if (!leta_n)   rdata_comb = {8'hFF, leta_dout};
		else if (!inputs_n) rdata_comb = inputs_port;
		else                rdata_comb = 16'hFFFF;
	end

	// Register the read path so the value the CPU latches has settled for a
	// full clk_sys instead of arriving through a live decode + mux in the same
	// cycle /DTACK does.  Both block RAMs and the program ROM are already
	// registered-output, so the CPU sees data 2 clk_sys after the address —
	// well inside the 8 clk_sys (one /7M period) minimum before /DTACK.
	always_ff @(posedge clk) cpu_rdata_r <= rdata_comb;

	// =====================================================================
	//  DTACK (12C LS163A + 11D LS74 pair + 16A LS260)
	// =====================================================================
	/* verilator lint_off UNUSEDSIGNAL */
	wire       dt_rco, dt_vack, dt_hack;
	wire [3:0] dt_wcnt;
	/* verilator lint_on UNUSEDSIGNAL */
	blstroid_dtack u_dtack (
		.clk(clk), .ce_7m(ce_7m), .reset(init_reset),
		.as(cpu_as), .vidram_n(vidram_n), .waithbl_n(waithbl_n),
		.eerom_n(eerom_n), .vpa_n(vpa_n), .h3(h3), .hblank(hblank),
		.dtack(cpu_dtack), .rco(dt_rco), .vidram_ack(dt_vack),
		.waithbl_ack(dt_hack), .wcnt(dt_wcnt));

endmodule
