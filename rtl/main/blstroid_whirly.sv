`timescale 1ns/1ps
//============================================================================
//  Blasteroids Whirly-Gig encoder — a MiSTer spinner, paddle, mouse or d-pad
//  turned into the CLK+DIR pin pair the LETA at 8A actually sees.
//
//  Copyright (C) 2026 RetroShrimp.  GPL-3.0-or-later; see LICENSE and
//  NOTICE.md.  The 8-bit counter the game reads is not here; it lives in
//  blstroid_leta.sv, where the schematic puts it.  This module is only the
//  cabinet side of the LET connector.
//
//  ---- what this stands in for -------------------------------------------
//  SP-316 sheet 13: LET-5/6 = P1-CLK / P1-DIR, LET-7/8 = P2-CLK / P2-DIR, each
//  with a 3.3 K pull-down, a 0.01 uF cap and a 1 K series resistor into the
//  LETA (sheet 4).  On a real cabinet those two lines come from the optical
//  Whirly-Gig knob; here they are synthesised from whatever the player has
//  plugged into the MiSTer.
//
//  The pin pattern is the 4-state Gray code the LETA's 2x decoder expects (see
//  blstroid_leta.sv): A = phase[1], B = phase[0] ^ phase[1], two phase steps
//  per counter count, so
//
//      counting up    (0,0) -> (0,1) -> (1,1) -> (1,0) -> (0,0)
//      counting down  the same sequence backwards
//
//  Every A transition is a count and "A == B at the sample" is the direction,
//  which is also how a plain clock+direction chip would read it.  Only the
//  sign of the result could therefore differ, and `INVERT` plus the OSD
//  "Whirly direction" switch cover that.
//
//  ---- pacing: even spacing and a speed limit -----------------------------
//  A MiSTer spinner does not turn like an optical knob.  It reports in bursts,
//  one signed delta per update, and a high-resolution unit can report far more
//  counts per game frame than Atari's slotted wheel could physically make.
//  Both reach the game, which differences the 8-bit LETA counter once per
//  frame with `sub.b` / `ext.w`: a frame delta of 128 or more wraps and reads
//  as a turn the other way, which the player sees as the ship spinning
//  backwards.  So the backlog is drained by a rate-proportional accumulator:
//
//      acc += min(|backlog|, RATE_CAP)   every clk_sys
//      one phase step each time acc reaches SMOOTH_K
//
//      counts/s = min(|backlog|, RATE_CAP) * f_clk / (2 * SMOOTH_K)
//
//    * Even spacing: below the cap the rate is proportional to the backlog, so
//      a burst decays with tau = 2*SMOOTH_K / f_clk = 8.0 ms and leaves as
//      evenly spaced edges over about half a frame instead of one clump.
//    * Speed limit: at |backlog| >= RATE_CAP the rate pins at 6000 counts/s =
//      100.1 counts per 59.92 Hz frame, so at most 101 land in one frame
//      sample — 27 short of the wrap at 128, at any sensitivity and any spin.
//      The margin covers the frame sample falling between two edges and the
//      d-pad being summed in on top.
//    * The fastest phase-step interval is SMOOTH_K / RATE_CAP = 4773 clk_sys,
//      37 times the LETA's 8H sampling period, so no edge is ever missed.
//    * The accumulator parks one short of SMOOTH_K while the backlog is empty,
//      so the first edge of a movement leaves on the next clock and the
//      smoothing adds no dead time to the start of a turn.
//
//  The backlog is clamped to +/-127 counts, so a wild flick cannot keep the
//  knob turning for seconds afterwards.  RATE_CAP is the largest rate the
//  game's 8-bit difference can represent, with margin, not a measured property
//  of the cabinet; it and the sensitivity table below were tuned against
//  reports from players using real MiSTer spinners.
//
//  ---- the four sources ---------------------------------------------------
//  All four are summed, so several can be plugged in at once and any of them
//  works with no OSD selector:
//    spinner  hps_io `spinner_N`: [7:0] signed delta, [8] toggles per update.
//    paddle   hps_io `paddle_N`: absolute 0..255, differenced here so a paddle
//             at a stop stops contributing instead of spinning forever.
//    mouse    ps2_mouse X ([24] toggle, signed 9-bit dx).
//    d-pad    Left/Right stepped at a fixed rate, the same idea as MAME's
//             PORT_KEYDELTA, so a gamepad or keyboard can play at all.
//
//  ---- how fast the game wants it ----------------------------------------
//  The VBLANK handler applies `delta * |delta|` to the per-frame difference
//  and sets an acceleration flag at |delta| >= 4 counts per frame.  The useful
//  range is therefore a few counts per frame at a nudge and tens at a spin,
//  and 3 vs 4 counts/frame is a qualitative change in feel: the sensitivity
//  table has to be picked against that curve, not against a pixel rate.  MAME
//  uses PORT_SENSITIVITY(60) PORT_KEYDELTA(10).
//
//  ---- the two runtime controls -------------------------------------------
//  `sens` and `invert` are inputs, not parameters, so a player can try either
//  without a rebuild.  `sens[3:0]` is the OSD index and scales the summed
//  delta in eighth counts (table below), 1/4x .. 4x, with index 0 = 1x the
//  exact identity; the residue accumulator in that scaler matters, because a
//  naive multiply-then-shift would round a one-count spinner nudge to zero at
//  every setting below 1x and the knob would simply be dead.  `invert` is
//  XORed with the `INVERT` parameter, so a build can pin the sign while the
//  OSD switch stays the user-facing control.
//============================================================================

module blstroid_whirly #(
	// Pacing (see "pacing" above).  tau = 2*SMOOTH_K/f_clk; top speed =
	// RATE_CAP*f_clk/(2*SMOOTH_K) counts/s.  Keep SMOOTH_K/RATE_CAP well above
	// the LETA's 128-clk_sys sampling period.
	parameter int unsigned SMOOTH_K  = 229091,
	parameter int unsigned RATE_CAP  = 48,
	// clk_sys cycles between d-pad steps, and counts per step.
	parameter int unsigned DPAD_DIV  = 65536,
	parameter int unsigned DPAD_STEP = 2,
	// Build-time sign of the knob; the OSD Invert switch XORs with it.
	parameter bit          INVERT    = 1'b0
)(
	input  logic       clk,
	input  logic       reset,

	input  logic [8:0] spinner,     // hps_io spinner_N: [8] toggle, [7:0] signed
	input  logic [7:0] paddle,      // hps_io paddle_N: absolute 0..255
	input  logic       mouse_stb,   // ps2_mouse[24] toggle (tie 0 if unused)
	input  logic [8:0] mouse_dx,    // signed 9-bit
	input  logic       pad_left,
	input  logic       pad_right,

	// ---- runtime controls (OSD) ----
	input  logic [3:0] sens,        // OSD index: 0 = 1x, see the table below
	input  logic       invert,      // XORed with the INVERT parameter

	output logic       enc_a,       // -> LETA CLKn
	output logic       enc_b        // -> LETA DIRn
);

	// ---- spinner: contribute on every toggle change ----------------------
	logic sp_t;
	always_ff @(posedge clk) sp_t <= spinner[8];
	wire signed [8:0] sp_d = (spinner[8] != sp_t) ? $signed({spinner[7], spinner[7:0]})
	                                              : 9'sd0;

	// ---- paddle: contribute the change in absolute position --------------
	logic [7:0] pd_prev;
	always_ff @(posedge clk) pd_prev <= paddle;
	wire signed [8:0] pd_d = $signed({1'b0, paddle}) - $signed({1'b0, pd_prev});

	// ---- mouse X ---------------------------------------------------------
	logic ms_t;
	always_ff @(posedge clk) ms_t <= mouse_stb;
	wire signed [8:0] ms_d = (mouse_stb != ms_t) ? $signed(mouse_dx) : 9'sd0;

	// ---- d-pad -----------------------------------------------------------
	localparam int                DIVW      = $clog2(DPAD_DIV);
	localparam logic signed [8:0] PAD_DELTA = 9'(DPAD_STEP);
	logic [DIVW-1:0] pdiv;
	logic            ptick;
	always_ff @(posedge clk) begin
		if (reset) begin
			pdiv  <= '0;
			ptick <= 1'b0;
		end else begin
			ptick <= (pdiv == DIVW'(DPAD_DIV - 1));
			pdiv  <= (pdiv == DIVW'(DPAD_DIV - 1)) ? '0 : (pdiv + 1'b1);
		end
	end
	wire signed [8:0] pad_d = !ptick     ?  9'sd0 :
	                           pad_right ?  PAD_DELTA :
	                           pad_left  ? -PAD_DELTA : 9'sd0;

	// ---- the backlog, in LETA counts -------------------------------------
	wire signed [10:0] delta_in = 11'(sp_d) + 11'(pd_d) + 11'(ms_d) + 11'(pad_d);

	// ---- sensitivity: eighth-count scaling with a carried residue ---------
	// `mul8` is the multiplier in eighths, so 8 = 1x is the identity.  The
	// residue `frac` keeps the fraction the shift throws away, which is what
	// stops a slow spinner from being rounded to a standstill below 1x.  At
	// sens = 0 (mul8 = 8) `frac` starts at 0 and stays there, so the default
	// path is arithmetically identical to having no scaler at all.
	//
	// The index order matches the OSD list in Arcade-Blasteroids.sv's CONF_STR:
	// 1x first so a cleared status word is the default, then up, then down.
	logic [5:0] mul8;
	always_comb begin
		case (sens)
			4'd0:  mul8 = 6'd8;    // 1x   (default)
			4'd1:  mul8 = 6'd9;    // 1.125x
			4'd2:  mul8 = 6'd10;   // 1.25x
			4'd3:  mul8 = 6'd11;   // 1.375x
			4'd4:  mul8 = 6'd12;   // 1.5x
			4'd5:  mul8 = 6'd14;   // 1.75x
			4'd6:  mul8 = 6'd16;   // 2x
			4'd7:  mul8 = 6'd20;   // 2.5x
			4'd8:  mul8 = 6'd24;   // 3x
			4'd9:  mul8 = 6'd32;   // 4x
			4'd10: mul8 = 6'd2;    // 0.25x
			4'd11: mul8 = 6'd3;    // 0.375x
			4'd12: mul8 = 6'd4;    // 0.5x
			4'd13: mul8 = 6'd5;    // 0.625x
			4'd14: mul8 = 6'd6;    // 0.75x
			default: mul8 = 6'd7;  // 0.875x
		endcase
	end

	wire signed [6:0]  mul8_s   = $signed({1'b0, mul8});
	logic signed [15:0] frac;
	// |delta_in| <= 1020 and mul8 <= 32, so |scaled_q| <= 32 647: fits 16 bits.
	wire  signed [15:0] scaled_q = (16'(delta_in) * 16'(mul8_s)) + frac;
	// Arithmetic shift = floor, so the residue is always in 0..7 and is carried
	// forward; nothing is lost and nothing accumulates without bound.
	wire  signed [15:0] whole    = scaled_q >>> 3;

	always_ff @(posedge clk) begin
		if (reset) frac <= '0;
		else       frac <= scaled_q - (whole <<< 3);
	end

	// Clamp into the range the unscaled sum could already reach (four sources
	// of at most +/-255), so widening the scaler cannot overflow `err_next`.
	wire signed [10:0] delta_s = (whole >  16'sd1023) ?  11'sd1023 :
	                             (whole < -16'sd1023) ? -11'sd1023 : 11'(whole);

	wire sign_flip = INVERT ^ invert;

	// ---- one pipeline stage, and it is not optional ----------------------
	// The multiply -> shift -> clamp -> negate -> accumulate -> clamp chain is
	// long enough to become the whole design's critical path if it is left
	// combinational, so the scaled delta is registered here and the
	// accumulator consumes it on the next clk_sys.  Nothing is lost or
	// double-counted: exactly one value is produced per cycle and exactly one
	// is consumed.
	logic signed [10:0] delta;
	always_ff @(posedge clk) begin
		if (reset) delta <= '0;
		else       delta <= sign_flip ? -delta_s : delta_s;
	end

	logic signed [10:0]        err;     // backlog, clamped to +/-127 counts
	logic                      sub;     // half-step within a count
	logic                      pos0;    // LSB of the emitted count position
	logic                      dir_up;  // held while stopped, so the pins rest

	wire moving = (err != 11'sd0);

	// ---- the pacer (see "pacing" in the header) ---------------------------
	localparam int         ACCW  = $clog2(SMOOTH_K + RATE_CAP + 1);
	localparam [ACCW-1:0]  K     = ACCW'(SMOOTH_K);
	localparam [ACCW-1:0]  CAP   = ACCW'(RATE_CAP);
	logic      [ACCW-1:0]  acc;
	wire       [6:0]       mag   = (err < 0) ? 7'(-err) : 7'(err);   // |err| <= 127
	wire       [ACCW-1:0]  inc   = (ACCW'(mag) > CAP) ? CAP : ACCW'(mag);
	wire       [ACCW:0]    acc_sum = {1'b0, acc} + {1'b0, inc};
	wire step_tick = moving && (acc_sum >= {1'b0, K});

	// One count is retired only on the SECOND half-step of a pair.
	wire retire    = step_tick & sub;
	wire signed [10:0] err_step = retire ? ((err > 0) ? -11'sd1 : 11'sd1) : 11'sd0;
	wire signed [11:0] err_next = 12'(err) + 12'(delta) + 12'(err_step);

	always_ff @(posedge clk) begin
		if (reset) begin
			acc    <= K - 1'b1;
			err    <= '0;
			sub    <= 1'b0;
			pos0   <= 1'b0;
			dir_up <= 1'b1;
		end else begin
			// Parked one short of K while idle: the first edge of a movement
			// leaves on the next clock.
			if      (!moving)   acc <= K - 1'b1;
			else if (step_tick) acc <= ACCW'(acc_sum - {1'b0, K});
			else                acc <= ACCW'(acc_sum);

			if      (err_next >  12'sd127) err <= 11'sd127;
			else if (err_next < -12'sd127) err <= -11'sd127;
			else                           err <= 11'(err_next);

			if (step_tick) begin
				dir_up <= (err > 0);
				sub    <= ~sub;
				if (sub) pos0 <= ~pos0;            // a whole count emitted
			end
		end
	end

	// phase = {pos0, sub};  A = phase[1], B = phase[0] ^ phase[1].
	// Counting down runs the same sequence backwards, which the decoder reads
	// as "A and B disagree at the sampling edge".
	wire [1:0] phase = dir_up ? {pos0, sub} : {pos0, ~sub};
	assign enc_a = phase[1];
	assign enc_b = phase[0] ^ phase[1];

	/* verilator lint_off UNUSEDSIGNAL */
	wire _unused = &{1'b0, sp_d[8], pd_d[8], ms_d[8], pad_d[8], 1'b0};
	/* verilator lint_on UNUSEDSIGNAL */

endmodule
