`timescale 1ns/1ps
//============================================================================
//  Blasteroids (Atari Games, 1987) - JSA I audio mixer: the 5F /MIX latch and
//  the sheet-16 resistor ladder.
//
//  Copyright (C) 2026 RetroShrimp.  GPL-3.0-or-later; see LICENSE and NOTICE.md.
//  Derived from the Xybots MiSTer core's xybots_jsa_mix.sv (GPL-3.0, same
//  author, itself from Toobin').  Every resistor and capacitor value on SP-316
//  sheet 16 is identical, so the arithmetic is unchanged.  The one Blasteroids
//  difference is upstream of this module: the 5F LS273B's D inputs come from
//  the 3F /VOICE latch, not from SD7:0, so blstroid_jsa_bus drives mix_data
//  from its `vo` register rather than from the 6502's write data.
//
//  ---- the register (SP-316 sheet 16, 5F 74LS273B) -------------------------
//  /MIX clocks it, /CLR = /POR, so every bit powers up at 0 (all sources muted,
//  filter disengaged) until the 6502 does its STA $2A00 / STA $2A06 pair:
//
//    D7 -> SM2   speech volume bit 1   \  TMS5220 socket empty on Blasteroids:
//    D6 -> SM1   speech volume bit 0   /  R72 75K / R73 150K switch nothing.
//    D5 -> PM2   POKEY  volume bit 1   \  POKEY socket empty on Blasteroids:
//    D4 -> PM1   POKEY  volume bit 0   /  R70 75K / R71 150K switch nothing.
//    D3 -> YM2   YM2151 volume bit 2      R62 7.5K left / R69 7.5K right
//    D2 -> YM1   YM2151 volume bit 1      R64 15K  / R67 15K
//    D1 -> YM0   YM2151 volume bit 0      R63 30K  / R68 30K
//    D0 -> LPF   output low-pass enable   Q5/Q6 2N3904, R56/R59 150K
//
//  ---- the analogue mixer (SP-316 sheet 16) --------------------------------
//  4066B analogue switches at 5C (left) and 5D (right) shunt binary-weighted
//  resistors into a virtual-earth summing amp (LM324 4B, R43/R48 12K feedback,
//  referenced to 2.5 V).  The conductances are 4 : 2 : 1, so the YM2151 volume
//  really is a linear (D3:D1)/7 attenuator - the same as MAME's
//  m_ym2151_volume = ((data >> 1) & 7) / 7.0.
//  The remaining switched input is PS through 12K, R65 gated by the YM2151's
//  CT1 into the LEFT channel and R66 gated by CT2 into the RIGHT (MAME applies
//  CT1 to both).  PS is the POKEY + speech sum, and both of those sockets are
//  empty on Blasteroids, so PS is silent and the whole CT1/CT2 path, plus /MIX
//  bits 7:4, are no-ops: decoded, latched and then ignored, which is what the
//  real board does.
//
//  Division by 7 uses a reciprocal multiply (x*9363>>16 ~= /7).  The combined
//  volume x reciprocal coefficient is selected before the sample multiply -
//  algebraically identical to (sample * volume) * reciprocal, but it avoids a
//  two-DSP combinational chain.  The sum saturates; with nothing else to add,
//  saturation can only trigger at the signed rails, but a wrap there would be
//  an audible click so it is kept.
//============================================================================

module blstroid_jsa_mix
(
	input  logic        clk,
	input  logic        reset,
	input  logic        mix_wr,
	input  logic  [7:0] mix_data,      // the 3F /VOICE latch outputs (VO7:VO0)
	input  logic signed [15:0] ym_left,
	input  logic signed [15:0] ym_right,
	output logic signed [15:0] out_left,
	output logic signed [15:0] out_right,
	output logic               lpf_engage    // -> blstroid_jsa_lpf
);

	logic [2:0] ym_vol; logic lpf;
	always_ff @(posedge clk) begin
		if (reset) begin ym_vol <= 3'd0; lpf <= 1'b0; end
		else if (mix_wr) begin ym_vol <= mix_data[3:1]; lpf <= mix_data[0]; end
	end
	// D7:D6 speech volume, D5:D4 POKEY volume — both sockets empty.
	wire unused = &{1'b0, mix_data[7:4]};

	// Sheet 16 drives the Q5/Q6 filter-switch base from R58 1K (from LPF) and
	// R57 1K (from YM0) with the word "OR" printed between them, mirrored as
	// R60/R57 on the right.  The drawing does not say whether that is a
	// wired-OR or an alternative-population marking, and it does not matter for
	// this ROM: the mixer byte is computed as $3E & $FC (pc $58E0) or $3E | $03
	// ($57E8), so D0 and D1 are always equal and lpf | ym_vol[0] == lpf ==
	// ym_vol[0] for every value the hardware ever sees.  The OR form is correct
	// for both possible populations, so it stays.  (MAME does not implement the
	// filter at all.)
	assign lpf_engage = lpf | ym_vol[0];

	// Precomputed volume*reciprocal coefficients.  Max = 7*9363 = 65541.
	logic signed [17:0] ym_gain;
	always_comb begin
		case (ym_vol)
			3'd0: ym_gain = 18'sd0;
			3'd1: ym_gain = 18'sd9363;
			3'd2: ym_gain = 18'sd18726;
			3'd3: ym_gain = 18'sd28089;
			3'd4: ym_gain = 18'sd37452;
			3'd5: ym_gain = 18'sd46815;
			3'd6: ym_gain = 18'sd56178;
			default: ym_gain = 18'sd65541;
		endcase
	end

	wire signed [33:0] ym_l_m = $signed(ym_left)  * $signed(ym_gain);
	wire signed [33:0] ym_r_m = $signed(ym_right) * $signed(ym_gain);
	wire signed [18:0] sum_l  = 19'(ym_l_m >>> 16);
	wire signed [18:0] sum_r  = 19'(ym_r_m >>> 16);

	function automatic signed [15:0] sat16(input signed [18:0] v);
		if      (v >  19'sd32767)  sat16 =  16'sd32767;
		else if (v < -19'sd32768)  sat16 =  16'sh8000;   // -32768
		else                       sat16 = v[15:0];
	endfunction

	always_ff @(posedge clk) begin
		out_left  <= sat16(sum_l);
		out_right <= sat16(sum_r);
	end

endmodule
