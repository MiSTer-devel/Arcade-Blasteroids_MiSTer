`timescale 1ns/1ps
//============================================================================
//  Blasteroids (Atari Games, 1987) - inter-CPU sound mailbox: the Atari SCOM
//  pair, 9A on the game PCB and 1M on the audio PCB.
//
//  Copyright (C) 2026 RetroShrimp.  GPL-3.0-or-later; see LICENSE and NOTICE.md.
//  Derived from the Xybots MiSTer core's xybots_sound_comm.sv (GPL-3.0, same
//  author), which reproduces MAME's atari_sound_comm device.  The register
//  semantics are unchanged; what differs on Blasteroids is the main-side
//  wiring - the sound interrupt is IRQ4, not Xybots' IRQ2, and the main-side
//  strobes live at different addresses.
//
//  ---- what the hardware is (SP-316 sheets 3 and 17) -----------------------
//  The main/sound link is not a pair of parallel latches.  It is two Atari SCOM
//  customs exchanging a byte serially over one wire plus a clock and one flag
//  each way:
//
//    game PCB 9A (MASTER, S/M- = GND)       audio PCB 1M (SLAVE, S/M- = PR1)
//      /WR   = /AUDWR  (FF8A01 write)         /WR   = /WRP  (0x2A02 write)
//      /RD   = /AUDRD  (FF9401 read)          /RD   = /RDP  (0x2802 read)
//      /RESREQ = /AUDRES (FF8C00 write)       /RESREQ = PR1 (never asserted)
//      /EFULL  = /AUDFLAG (from the slave)    /EFULL  = FIN  (from the master)
//      CK      = AUD CLK = /4H                CK      = /CLK (the master's)
//      /RES  (16) -> /IPL2 => IRQ4            RES  (16) = /SNDRES -> 6502 /RES
//      BUSY  (5)  -> /AUDBUSY -> FF9C01 D6    BUSY (5)  = SFULL -> /RDIO D5
//      FULL  (4)  -> AUD-4                    FULL (4)  = /NMI -> 6502 NMI
//      SD    (19) = AUDAT ---------------- /DATA (bidirectional serial data)
//
//  Two things a register-level model has to be honest about:
//
//   1. The sound-CPU reset travels over the link.  There is no reset wire in
//      the AUD ribbon: a /RESREQ pulse on the master makes the slave's RES
//      output drive /SNDRES.  Modelled as snd_reset pulsing on main_sndrst_wr
//      (the FF8C00 write), which is what MAME does too.
//   2. A transfer takes a serial frame, not zero time.  AUD CLK is /4H; the
//      68000 runs at 7.15909 MHz, so 4H is about 894.9 kHz and a byte costs at
//      least 8 bit times, roughly 8.94 us.  That latency is deliberately not
//      modelled: no code path in this game waits on the link - the 68000 always
//      finds /AUDIO BUSY ready before it writes a command.  If a latency model
//      is ever needed, the place for it is here, as a countdown that delays the
//      *set* of m2s_ready / s2m_ready.
//
//  ---- register-level semantics --------------------------------------------
//    main_command_w  (68000 writes FF8A01): latch main->sound data, set
//                     main_to_sound_ready, assert the 6502 NMI.  FF9C01 D6
//                     (/AUDIO BUSY) reads 0 while that flag is set.
//    sound_command_r (6502 reads 0x2802 /RDP): clear main_to_sound_ready + NMI.
//    sound_response_w(6502 writes 0x2A02 /WRP): latch sound->main data, set
//                     sound_to_main_ready, assert IRQ4.
//    main_response_r (68000 reads FF9401): clear sound_to_main_ready + IRQ4.
//                     There is no acknowledge register for IRQ4 - the read IS
//                     the acknowledge, and the level-4 handler at 0xE48 makes
//                     no other hardware access.
//    sound reset     (68000 writes FF8C00): reset the 6502 and clear the
//                     response FLAG.  The response DATA latch survives, which
//                     is why a read straight after /AUDRES returns 0xFF.
//
//  The two CPUs run in one FPGA fabric on clock enables, so MAME's zero-delay
//  synchronisation timers collapse to synchronous set/clear here.  On a
//  same-cycle set+clear of a ready flag the set (the new datum) wins, so a
//  command or a response is never silently dropped.
//============================================================================

module blstroid_sound_comm
(
	input  logic       clk,
	input  logic       reset,           // global/power-on reset

	// ---- main-CPU (68000) side ----
	input  logic       main_cmd_wr,     // pulse: 68000 wrote FF8A01 (/AUDWR)
	input  logic [7:0] main_cmd_data,   // D7:0 of that write
	input  logic       main_resp_rd,    // pulse: 68000 read  FF9401 (/AUDRD)
	input  logic       main_sndrst_wr,  // pulse: 68000 wrote FF8C00 (/AUDRES)
	output logic [7:0] main_resp_data,  // data returned to the 68000 read
	output logic       main_irq,        // -> 68000 IRQ4 (level, via /IPL2)
	output logic       main_to_sound_ready,
	output logic       sound_to_main_ready,

	// ---- sound-CPU (6502) side ----
	input  logic       snd_cmd_rd,      // pulse: 6502 read  0x2802 (/RDP)
	input  logic       snd_resp_wr,     // pulse: 6502 wrote 0x2A02 (/WRP)
	input  logic [7:0] snd_resp_data,
	output logic [7:0] snd_cmd_data,    // data returned to the 6502 command read
	output logic       snd_nmi,         // -> 6502 NMI (level while a command is pending)
	output logic       snd_reset        // -> 6502 reset (1-cycle pulse)
);

	logic [7:0] m2s_data, s2m_data;
	logic       m2s_ready, s2m_ready;

	assign main_to_sound_ready = m2s_ready;
	assign sound_to_main_ready = s2m_ready;
	assign snd_cmd_data        = m2s_data;
	assign main_resp_data      = s2m_data;
	assign snd_nmi             = m2s_ready;   // SCOM FULL -> 6502 /NMI
	assign main_irq            = s2m_ready;   // SCOM /RES -> /IPL2 -> IRQ4

	always_ff @(posedge clk) begin
		if (reset) begin
			m2s_data  <= 8'h00;
			s2m_data  <= 8'h00;
			m2s_ready <= 1'b0;
			s2m_ready <= 1'b0;
			snd_reset <= 1'b0;
		end else begin
			snd_reset <= 1'b0;

			// main -> sound command latch
			if (main_cmd_wr) begin
				m2s_data  <= main_cmd_data;
				m2s_ready <= 1'b1;          // set wins over a coincident 6502 read
			end else if (snd_cmd_rd) begin
				m2s_ready <= 1'b0;
			end

			// sound -> main response latch
			if (snd_resp_wr) begin
				s2m_data  <= snd_resp_data;
				s2m_ready <= 1'b1;          // set wins over a coincident 68000 read
			end else if (main_resp_rd || main_sndrst_wr) begin
				s2m_ready <= 1'b0;
			end

			// /AUDRES -> master /RESREQ -> slave RES -> /SNDRES
			if (main_sndrst_wr)
				snd_reset <= 1'b1;
		end
	end

endmodule
