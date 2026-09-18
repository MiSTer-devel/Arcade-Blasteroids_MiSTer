`timescale 1ns/1ps
//============================================================================
//  Blasteroids (Atari Games, 1987) - JSA I audio PCB, the 2L PAL.
//  Atari 136056-2101, PAL16L8A, 25 ns: the literal fuse map.
//
//  Copyright (C) 2026 RetroShrimp.  GPL-3.0-or-later; see LICENSE and NOTICE.md.
//
//  ---- provenance ----------------------------------------------------------
//  The part number comes from the TM-316 operator manual's Stand-Alone Audio
//  PCB parts list: 2L is the only PAL the Blasteroids BOM populates, and the
//  second PAL site on sheet 15 (2M-FPAL, which would drive the 6502's RDY) is
//  not in the list at all, so RDY really does just sit on its pull-up and
//  there are no wait states.  The equations below were read out of the PLD
//  Archive dump of 136056-2101 and are transcribed verbatim in the CUPL form
//  the archive prints, with i<n> = the input on pin n and o<n> = the output on
//  pin n.
//
//      !o12 = i1 & i9 # i1 & i11 # i1 & i7 & i8;
//      !o13 = !i6 & !i7 & i8 & !i9 & !i11;
//      !o14 = i2 & i6 & !i7 & i8 & !i9 & !i11;
//      !o15 = !i8 & !i9 & !i11;
//       o16 = i4 & i8 # !i7 & i8 # i8 & i9 # i8 & i11;
//       o17 = i3 & i7 # i7 & !i8 # i7 & i9 # i7 & i11;
//      !o18 = !i1 & i2 & !i8 & !i9 & !i11
//           # !i1 & i2 & !i6 & !i7 & !i9 & !i11;
//      !o19 = i1 & i2 & i5 # i1 & i2 & !i6 # i1 & i2 & i7 # i1 & i2 & !i8
//           # i1 & i2 & i9 # i1 & i2 & i11
//           # !i2 & i6 & !i7 & i8 & !i9 & !i11
//           # !i1 & !i5 & i6 & !i7 & i8 & !i9 & !i11;
//
//  Pins 12, 13, 14, 15, 18 and 19 are active low (the !o form); 16 and 17 are
//  active high.  All eight are combinatorial - a 16L8 has no registers - and
//  none of the six I/O pins is fed back, so this module is pure combinational
//  logic with no output enables to model.
//
//  ---- pin translation (SP-316 sheet 15) -----------------------------------
//    in   1 SR//W (6502 R/W, HIGH = read)   out 12 /ROM   (2K  /CE)
//         2 O2    (6502 phase 2)                13 /YAM   (2F  /CS)
//         3 BA12  (5H /WRIO latch D6)           14 /REST  (LS138 /G2A, POKEY)
//         4 BA13  (5H /WRIO latch D7)           15 /RAM   (2H  /CS1)
//         5 SA9                                 16  A13B  (2K  A13)
//         6 SA11                                17  A12B  (2K  A12)
//         7 SA12                                18 /SWR   (2H  /WE)
//         8 SA13                                19 /SRD   (2K//2H /OE, LS138 G1)
//         9 SA14                             (10 GND, 20 VCC)
//        11 SA15
//
//  ---- what the eight outputs mean in net terms ----------------------------
//   /ROM  = READ & ($3000-$FFFF)                       - read-only chip enable
//   /YAM  = $2000-$27FF                                - 2 KB, SA10:SA0 mirror
//   /REST = O2 & $2800-$2FFF
//   /RAM  = $0000-$1FFF                                - not O2- or R/W-gated
//    A13B = SA13, except BA13 inside $3000-$3FFF
//    A12B = SA12, except BA12 inside $3000-$3FFF
//   /SWR  = WRITE & O2 & ($0000-$1FFF | $2000-$27FF)   - RAM and YM only
//   /SRD  = READ  & O2                                   outside $2800-$2FFF;
//           inside $2800-$2FFF it inverts:    !O2
//                                           | READ  &  SA9
//                                           | WRITE & !SA9
//
//  That last term is the point of the part.  The 4M LS138's active-high G1 is
//  wired to /SRD, so the decoder is enabled when /SRD is high, and inside the
//  $2800 block /SRD is high exactly when (READ & SA9=0) or (WRITE & SA9=1) -
//  i.e. Y0..Y3 (/RDP, /RDIO, /IRQACK) fire only on reads and Y4..Y7 (/VOICE,
//  /WRP, /WRIO, /MIX) fire only on writes.  The direction gating of the
//  register strobes therefore comes out of this PAL, not out of the logic
//  around it.
//============================================================================

module blstroid_jsa_pal2l
(
	// inputs, by PAL pin number
	input  logic i1,    // SR//W (1 = read)
	input  logic i2,    // O2
	input  logic i3,    // BA12
	input  logic i4,    // BA13
	input  logic i5,    // SA9
	input  logic i6,    // SA11
	input  logic i7,    // SA12
	input  logic i8,    // SA13
	input  logic i9,    // SA14
	input  logic i11,   // SA15
	// outputs, by PAL pin number
	output logic o12,   // /ROM   (active low)
	output logic o13,   // /YAM   (active low)
	output logic o14,   // /REST  (active low)
	output logic o15,   // /RAM   (active low)
	output logic o16,   //  A13B  (active high)
	output logic o17,   //  A12B  (active high)
	output logic o18,   // /SWR   (active low)
	output logic o19    // /SRD   (active low)
);

	// The eight product-term sums, transcribed one for one from the dump.
	// `#` (OR) is `|`, `&` stays `&`, `!` is `~`; the six active-low pins are
	// written as the complement of the archive's `!o<n>` equation so the code
	// and the dump can be diffed line by line.

	assign o12 = ~( (i1 & i9)
	              | (i1 & i11)
	              | (i1 & i7 & i8) );

	assign o13 = ~( ~i6 & ~i7 & i8 & ~i9 & ~i11 );

	assign o14 = ~( i2 & i6 & ~i7 & i8 & ~i9 & ~i11 );

	assign o15 = ~( ~i8 & ~i9 & ~i11 );

	assign o16 =  ( (i4 & i8)
	              | (~i7 & i8)
	              | (i8 & i9)
	              | (i8 & i11) );

	assign o17 =  ( (i3 & i7)
	              | (i7 & ~i8)
	              | (i7 & i9)
	              | (i7 & i11) );

	assign o18 = ~( (~i1 & i2 & ~i8 & ~i9 & ~i11)
	              | (~i1 & i2 & ~i6 & ~i7 & ~i9 & ~i11) );

	assign o19 = ~( (i1 & i2 & i5)
	              | (i1 & i2 & ~i6)
	              | (i1 & i2 & i7)
	              | (i1 & i2 & ~i8)
	              | (i1 & i2 & i9)
	              | (i1 & i2 & i11)
	              | (~i2 & i6 & ~i7 & i8 & ~i9 & ~i11)
	              | (~i1 & ~i5 & i6 & ~i7 & i8 & ~i9 & ~i11) );

endmodule
