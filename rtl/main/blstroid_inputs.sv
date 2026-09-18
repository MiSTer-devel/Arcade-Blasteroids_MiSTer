`timescale 1ns/1ps
//============================================================================
//  Blasteroids status and button ports, FF9C01 / FF9C03 — SP-316 sheet 4
//  (12L F244, 10A LS257).
//
//  Copyright (C) 2026 RetroShrimp.  GPL-3.0-or-later; see LICENSE and
//  NOTICE.md.
//
//  Two devices share the /INPUTS strobe and both drive the LOW byte only:
//  nothing on the board drives BD15:8 for this port, and the game reads it
//  only as a byte at the odd address.
//
//  ---- 12L F244, the status nibble — not selected by A1 ------------------
//    BD7  A17 -> Y3   SELFTEST    active LOW  (R1 1 K pull-up on sheet 3; the
//                                 audio board's SW1 pulls it down)
//    BD6  A15 -> Y5   /AUDBUSY    0 = busy    (SCOM 9A BUSY pin 5)
//    BD5  A13 -> Y7   VBLANK      active HIGH
//    BD4  A11 -> Y9   HBLANK      active HIGH
//  Both /1G and /2G are /INPUTS, so these four bits read the same at FF9C01
//  and FF9C03.  Sheet 12 prints "XXXX XX01" on the HBLANK row and "XXXX XXX1"
//  on the three above it; the hardware makes all four A1-independent, so that
//  is a drafting slip — D7:D4 read identically at both addresses.
//
//  ---- 10A LS257, the button nibble — selected by BA1 --------------------
//    BD0 FIRE    A 14 / B 13 -> Y 12
//    BD1 THRUST  A  5 / B  6 -> Y  7
//    BD2 TRANS   A 11 / B 10 -> Y  9
//    BD3 SPARE   A  2 / B  3 -> Y  4
//    SEL (1) = BA1, /G (15) = /INPUTS.  An LS257 selects its A inputs when
//    SELECT is low, so FF9C01 (A1 = 0) is player 1 and FF9C03 (A1 = 1) is
//    player 2, matching sheet 12.
//  Every button line has a 470 R pull-up, a 0.1 uF cap and a 1 K series
//  resistor into the mux, so buttons are active low.  SPARE1/SPARE2 land on
//  CTRA-9/CTRA-10, which sheet 13's cabinet harness leaves unconnected — they
//  read 1 on a real machine.  MAME maps them as BUTTON4 and the self test
//  displays them, so they are exposed here.
//
//  A2 is not decoded for this port, so the ROM's `movep.w $402(a1)` read of
//  FF9C05 (A2 = 1, A1 = 1) simply lands on the player-2 byte; nothing special
//  is needed for it — see blstroid_addr_decode.
//
//  D4, the HBLANK bit, is never read by the program: the game synchronises
//  with the FF8E00 halt register instead.  The bit still has to be present,
//  but nothing observable depends on its value.
//
//  Pure combinational packing: no clock, no state.
//============================================================================

module blstroid_inputs
(
	// ---- active-high "pressed" inputs; inverted into the port here --------
	input  logic p1_fire, p1_thrust, p1_transform, p1_spare,
	input  logic p2_fire, p2_thrust, p2_transform, p2_spare,

	// ---- 12L F244 status -------------------------------------------------
	input  logic self_test,      // 1 = the self-test switch is ON -> D7 = 0
	input  logic snd_m2s_ready,  // 1 = a command is still pending -> D6 = 0
	input  logic vblank,         // active high
	input  logic hblank,         // active high

	// ---- 10A LS257 select ------------------------------------------------
	input  logic a1,             // BA1: 0 = player 1 (FF9C01), 1 = player 2

	output logic [15:0] port     // D15:8 undriven, modelled as 1s
);

	wire p_fire   = a1 ? p2_fire      : p1_fire;
	wire p_thrust = a1 ? p2_thrust    : p1_thrust;
	wire p_trans  = a1 ? p2_transform : p1_transform;
	wire p_spare  = a1 ? p2_spare     : p1_spare;

	assign port = { 8'hFF,            // D15:8 not driven by anything
	                ~self_test,       // D7 SELFTEST,  active low
	                ~snd_m2s_ready,   // D6 /AUDBUSY,  0 = busy
	                 vblank,          // D5 VBLANK,    active high
	                 hblank,          // D4 HBLANK,    active high
	                ~p_spare,         // D3 SPARE,     active low
	                ~p_trans,         // D2 TRANSFORM
	                ~p_thrust,        // D1 THRUST
	                ~p_fire };        // D0 FIRE

endmodule
