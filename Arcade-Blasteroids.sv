//============================================================================
//
//  Blasteroids (Atari Games, 1987) MiSTer FPGA core - top-level glue (emu).
//
//  Copyright (C) 2026 RetroShrimp.  GPL-3.0-or-later; see LICENSE and NOTICE.md.
//  Derived from the Xybots MiSTer core's Arcade-Xybots.sv (GPL-3.0, same
//  author).
//
//  blstroid_core is the whole machine: the 68000 main board, the video board,
//  the JSA I audio PCB and the graphics SDRAM path, all in one clk_sys domain
//  on integer clock enables.  This file is the MiSTer side of it - PLL clocks,
//  hps_io (OSD, ROM download, NVRAM and controls), the control mapping, the
//  analog-alignment stage, arcade_video and the SDRAM chip pins.
//
//  Blasteroids runs on a horizontal monitor (MAME ROT0), so there is no
//  rotation: the raster goes straight through arcade_video.  The Orientation
//  option only offers a 180-degree flip, for a monitor mounted the other way
//  up, which screen_rotate implements through the DDR framebuffer with
//  no_rotate held high.
//
//  ---- raster, and the parameters that follow from it ----------------------
//  The pixel clock is 14.318181 MHz and the raster is 640 x 240 visible in
//  912 x 262 total, from a 57.272727 MHz clk_sys.  Four parameters below are
//  derived from that and none of them is optional:
//
//    ce_pix        = clk_sys / 4                (rtl/blstroid_core.sv)
//    CLK_PER_PIX   = 4                          (blstroid_analog_adjust)
//    H_TOTAL / V_TOTAL = 912 / 262              (blstroid_analog_adjust)
//    AW            = 11                         (crt_adjust line buffer)
//    arcade_video WIDTH = 640
//
//    * a wrong CLK_PER_PIX makes one CRT H-Size click the wrong percentage and
//      the OSD list stops matching the effect;
//    * a wrong H_TOTAL makes the HSync shift register in crt_adjust wrap
//      mid-line;
//    * AW must give at least one line per ping-pong bank: AW = 10 is 512
//      samples, and a 912-pixel line would wrap the write pointer mid-line;
//    * an arcade_video WIDTH below the real active line under-sizes the
//      scandoubler line buffer and loses the right-hand part of the picture.
//
//  The aspect ratio is 4:3.  640 x 240 is not a 4:3 pixel grid, but the native
//  pixels are 2:1 wide (the playfield stamps are 8 x 8 ROM pixels doubled
//  horizontally), so 640 x 240 with 2:1 pixels displays as 4:3.  Do not
//  "fix" it to 8:3.
//
//  The Whirly-Gig counter is the Atari LETA custom at 8A, inside
//  blstroid_main.  What this file instantiates is blstroid_whirly, the cabinet
//  side of the LET connector, which turns a MiSTer spinner, paddle, mouse or
//  d-pad into the CLK+DIR pin pair the LETA sees.
//
//============================================================================

`timescale 1ns/1ps

module emu
(
	`include "sys/emu_ports.vh"
);

///////// Default values for ports not used in this core /////////

assign ADC_BUS  = 'Z;
assign USER_OUT = '1;
assign {UART_RTS, UART_TXD, UART_DTR} = 0;
assign {SD_SCK, SD_MOSI, SD_CS} = 'Z;

assign VGA_F1        = 0;
assign VGA_SCALER    = 0;
assign VGA_DISABLE   = 0;
assign HDMI_FREEZE   = 0;
assign HDMI_BLACKOUT = 0;
assign HDMI_BOB_DEINT= 0;
assign FB_FORCE_BLANK= 0;

assign AUDIO_MIX = 0;

assign LED_DISK  = 0;
assign LED_POWER = 0;
assign BUTTONS   = 0;

//////////////////////////////////////////////////////////////////

// Horizontal cabinet: the native raster is 640 x 240 with 2:1 wide pixels, i.e.
// 4:3 on the monitor, and the Orientation option never rotates (only flips
// 180), so "Original" is 4:3 in every state.  VIDEO_ARX/ARY are actually driven
// by video_freak further down; these are the ratios it starts from.
wire  [1:0] ar  = status[122:121];
wire [11:0] arx = (ar == 2'd0) ? 12'd4 : 12'(ar - 1'd1);
wire [11:0] ary = (ar == 2'd0) ? 12'd3 : 12'd0;

`include "build_id.v"
localparam CONF_STR = {
	"Blasteroids;;",
	"-;",
	"O[122:121],Aspect ratio,Original,Full Screen,[ARC1],[ARC2];",
	"O[4],Orientation,Original,Flip;",
	"O[22:20],Scale,Normal,V-Integer,HV-Integer,Narrower HV-Integer;",
	"-;",
	// CRT Adjust: rmonic79's MiSTer-CRT-Adjust, core-side variant.  Three
	// amounts plus an On/Off.  H-Position is built as a content shift (the
	// engine can also build it as a sync shift; which one is compiled in is set
	// by crt_adjust's HPOS_MODE, and the reasoning for this raster is in
	// rtl/video/blstroid_analog_adjust.sv).
	//
	// Every amount is plain two's complement, so the list is 0,+1..+max then
	// -min..-1 and the decode is a $signed().  None of these touch the core's
	// timing; the stage sits between blstroid_core and arcade_video.  Off is a
	// bit-exact bypass, so HDMI is untouched unless the user turns it On.
	//
	// H1 hides the three amounts while the stage is inactive, which includes the
	// case where the framework has forced the scandoubler and the stage gates
	// itself off (see `crt_enable`).
	"P1,CRT Adjust;",
	"P1-;",
	"P1O[23],CRT Adjust,Off,On;",
	"H1P1O[28:24],CRT H-Size,0,+1,+2,+3,+4,+5,+6,+7,+8,+9,+10,+11,+12,+13,+14,+15,-16,-15,-14,-13,-12,-11,-10,-9,-8,-7,-6,-5,-4,-3,-2,-1;",
	"H1P1O[35:29],CRT H-Position,0,+1,+2,+3,+4,+5,+6,+7,+8,+9,+10,+11,+12,+13,+14,+15,+16,+17,+18,+19,+20,+21,+22,+23,+24,+25,+26,+27,+28,+29,+30,+31,+32,+33,+34,+35,+36,+37,+38,+39,+40,+41,+42,+43,+44,+45,+46,+47,+48,+49,+50,+51,+52,+53,+54,+55,+56,+57,+58,+59,+60,+61,+62,+63,-64,-63,-62,-61,-60,-59,-58,-57,-56,-55,-54,-53,-52,-51,-50,-49,-48,-47,-46,-45,-44,-43,-42,-41,-40,-39,-38,-37,-36,-35,-34,-33,-32,-31,-30,-29,-28,-27,-26,-25,-24,-23,-22,-21,-20,-19,-18,-17,-16,-15,-14,-13,-12,-11,-10,-9,-8,-7,-6,-5,-4,-3,-2,-1;",
	"H1P1O[41:36],CRT V-Shift,0,+1,+2,+3,+4,+5,+6,+7,+8,+9,+10,+11,+12,+13,+14,+15,+16,+17,+18,+19,+20,+21,+22,+23,+24,+25,+26,+27,+28,+29,+30,+31,-32,-31,-30,-29,-28,-27,-26,-25,-24,-23,-22,-21,-20,-19,-18,-17,-16,-15,-14,-13,-12,-11,-10,-9,-8,-7,-6,-5,-4,-3,-2,-1;",
	"-;",
	// No DIP page: Blasteroids has no dip switches.  MAME's
	// INPUT_PORTS_START(blstroid) has only the two dial ports and the two
	// button/status words, and every operator setting lives in the X2804
	// EEPROM at 0xFFB000, changed from the game's own self test.  A "DIP;" line
	// here would ask MiSTer Main to splice in a menu the MRA can never populate.
	//
	// The Whirly-Gig knob has two runtime controls:
	//   * SENSITIVITY.  The game differences the LETA counter once per VBLANK
	//     and applies delta*|delta|, with an acceleration flag at |delta| >= 4
	//     counts per frame, so 3 versus 4 counts per frame is a qualitative
	//     change in feel.  Entry 0 is the 1x default and the steps are eighths.
	//   * DIRECTION.  The LETA's CLK/DIR sense is not documented, and
	//     blstroid_whirly and blstroid_leta are self-consistent under either
	//     reading, so this switch selects between them.
	"O[15:12],Whirly sensitivity,1x,1.125x,1.25x,1.375x,1.5x,1.75x,2x,2.5x,3x,4x,0.25x,0.375x,0.5x,0.625x,0.75x,0.875x;",
	"O[11],Whirly direction,Normal,Inverted;",
	"-;",
	// SW1 on the JSA audio PCB, behind the upper coin door (TM-316 page 1-2).
	// One switch, two readers: /RDIO D7 on the audio board AND, over JSCOM-1,
	// the main board's 0xFF9C01 D7.  It is a toggle, not a momentary button.
	"O[6],Service,Off,On;",
	"-;",
	"T[0],Reset;",
	// Control panel (per player): a Whirly-Gig knob plus Fire, Thrust and
	// Transform; coins are on the JSA audio board.  The knob is an optical
	// encoder, so it is the spinner/paddle/mouse axis rather than a stick
	// direction -- but D-pad Left/Right also step it
	// (rtl/main/blstroid_whirly.sv) so a plain gamepad can play.
	//
	// There is no Start button.  The cabinet has a continuous buy-in feature: a
	// player joins by coining up.  TM-316's Switch Test lists exactly four
	// inputs per player (Spare, Xform, Thrust, Fire), the control-panel figure
	// and the CTRA connector carry three buttons per player, and MAME's
	// blstroid.cpp defines no IPT_START1/2.  Button 4 is sheet 12's D3 "spare",
	// which is unwired in a cabinet but IS displayed by the self test, so it is
	// mapped rather than tied off - that is what lets a user confirm the whole
	// input path from the Switch Test.
	// This list matches mra/Blasteroids.mra's
	// <buttons names="Fire,Thrust,Transform,Spare,Coin" default="A,B,X,Y,Select"/>
	// exactly; the MRA's names override these in the OSD mapping screen.
	"J1,Fire,Thrust,Transform,Spare,Coin;",
	"jn,A,B,X,Y,Select;",
	"V,v",`BUILD_DATE
};

wire        forced_scandoubler;
wire  [1:0] buttons;
wire [127:0] status;
wire [10:0] ps2_key;
wire        direct_video;
wire [21:0] gamma_bus;

wire        ioctl_download;
wire        ioctl_wr;
wire [26:0] ioctl_addr;
wire  [7:0] ioctl_dout;
wire [15:0] ioctl_index;
wire        ioctl_wait;
// NVRAM: the X2804 EEPROM image is 512 bytes and travels on ioctl index 2 in
// both directions (the MRA declares <nvram index="2" size="512"/>).  The
// transport itself lives in blstroid_core; these are the hps_io wires it needs.
wire        ioctl_upload, ioctl_upload_req;
wire  [7:0] ioctl_upload_index, ioctl_din;

wire [31:0] joystick_0, joystick_1;
wire  [8:0] spinner_0, spinner_1;
wire  [7:0] paddle_0, paddle_1;
wire [24:0] ps2_mouse;

// CRT Adjust master enable.  Declared here because status_menumask below reads
// it; what it means is documented at the stage itself (VIDEO section).  The
// forced-scandoubler term is upstream's own integration rule: the read-rate
// base assumes the native 15 kHz pixel-CE ratio, and once the framework has
// doubled the line rate there is no analog CRT geometry left to fit anyway.
wire crt_enable = status[23] & ~forced_scandoubler;

hps_io #(.CONF_STR(CONF_STR)) hps_io
(
	.clk_sys(clk_sys),
	.HPS_BUS(HPS_BUS),
	.EXT_BUS(),
	.gamma_bus(gamma_bus),

	.forced_scandoubler(forced_scandoubler),
	.direct_video(direct_video),

	.buttons(buttons),
	.status(status),
	// H1 = "hide while set": hide the three CRT Adjust amounts whenever the
	// stage is not actually doing anything.  `crt_enable` is the same signal
	// the stage itself runs on, so the menu never offers a control that is
	// being ignored.
	.status_menumask({14'd0, ~crt_enable, 1'b0}),

	.ioctl_download(ioctl_download),
	.ioctl_wr(ioctl_wr),
	.ioctl_addr(ioctl_addr),
	.ioctl_dout(ioctl_dout),
	.ioctl_index(ioctl_index),
	.ioctl_wait(ioctl_wait),

	.ioctl_upload(ioctl_upload),
	.ioctl_upload_req(ioctl_upload_req),
	.ioctl_upload_index(ioctl_upload_index),
	.ioctl_din(ioctl_din),

	.joystick_0(joystick_0),
	.joystick_1(joystick_1),

	// Whirly-Gig sources.  All of them are summed per player in blstroid_whirly,
	// so no OSD selector is needed and any connected device works.
	.spinner_0(spinner_0),
	.spinner_1(spinner_1),
	.paddle_0(paddle_0),
	.paddle_1(paddle_1),
	.ps2_mouse(ps2_mouse),

	.ps2_key(ps2_key)
);

///////////////////////   CLOCKS   ///////////////////////////////

wire clk_sys;               // 57.272727 MHz = 4 x 14.318181 MHz
wire clk_sdram;             // same rate, phase-shifted for the SDRAM_CLK pin
wire pll_locked;

pll pll
(
	.refclk(CLK_50M),
	.rst(1'b0),
	.outclk_0(clk_sys),
	.outclk_1(clk_sdram),
	.locked(pll_locked)
);

wire reset = RESET | status[0] | buttons[1] | ~pll_locked;

///////////////////////   CONTROLS   /////////////////////////////

// MiSTer joystick bits: [0]=Right [1]=Left [2]=Down [3]=Up, then the CONF_STR
// J1 list "Fire,Thrust,Transform,Spare,Coin" gives [4]=Fire [5]=Thrust
// [6]=Transform [7]=Spare [8]=Coin - the same order the MRA's
// <buttons names="Fire,Thrust,Transform,Spare,Coin"> uses, which overrides
// these labels in the OSD mapping screen.
//
// The core's `btn1`/`btn2` are the 0xFF9C01 / 0xFF9C03 D3:0 nibble in schematic
// bit order but active high ("pressed"); the board's active-low sense belongs
// in blstroid_inputs, where the sheet is transcribed:
//   D0 Fire   D1 Thrust   D2 Transform   D3 "spare"
// D3 is not a Start button - Blasteroids has none (see the CONF_STR comment).
// It is wired because the self test's Switch Test displays it.
wire [3:0] btn1 = {joystick_0[7], joystick_0[6], joystick_0[5], joystick_0[4]};
wire [3:0] btn2 = {joystick_1[7], joystick_1[6], joystick_1[5], joystick_1[4]};

// Coins are read on the JSA audio board; the self-test switch is SW1 on that
// same board and fans out to /RDIO D7 there and, over JSCOM-1, to the main
// board's 0xFF9C01 D7.  On the cabinet, COIN1 is the RIGHT chute.
wire coin1     = joystick_0[8];
wire coin2     = joystick_1[8];
wire self_test = status[6];

// Whirly-Gig knobs: the cabinet side of the LET connector.  The counter itself
// is the Atari LETA custom at 8A, inside blstroid_main where sheet 4 puts it;
// this module only synthesises the CLK/DIR pin pair that reaches it.  Player 1
// also gets the mouse X axis as a fallback; player 2 does not, since there is
// only one mouse.
//
// The OSD sensitivity list is ordered so that entry 0 is the 1x default, and
// blstroid_whirly decodes that same index directly (its `mul8` table), so the
// status field goes straight through.
wire [3:0] whirly_sens = status[15:12];
wire       whirly_inv  = status[11];

wire [1:0] whirly_a, whirly_b;

blstroid_whirly u_whirly1
(
	.clk(clk_sys), .reset(reset),
	.spinner(spinner_0), .paddle(paddle_0),
	.mouse_stb(ps2_mouse[24]), .mouse_dx({ps2_mouse[4], ps2_mouse[15:8]}),
	.pad_left(joystick_0[1]), .pad_right(joystick_0[0]),
	.sens(whirly_sens), .invert(whirly_inv),
	.enc_a(whirly_a[0]), .enc_b(whirly_b[0])
);

blstroid_whirly u_whirly2
(
	.clk(clk_sys), .reset(reset),
	.spinner(spinner_1), .paddle(paddle_1),
	.mouse_stb(1'b0), .mouse_dx(9'd0),
	.pad_left(joystick_1[1]), .pad_right(joystick_1[0]),
	.sens(whirly_sens), .invert(whirly_inv),
	.enc_a(whirly_a[1]), .enc_b(whirly_b[1])
);

///////////////////////   CORE   /////////////////////////////////

wire        ce_pix;
wire  [7:0] core_r, core_g, core_b;
wire        core_hs, core_vs, core_hb, core_vb;
wire signed [15:0] aud_l, aud_r;

blstroid_core u_core
(
	.clk_sys(clk_sys), .reset(reset), .init_reset(~pll_locked),

	.ioctl_download(ioctl_download), .ioctl_wr(ioctl_wr), .ioctl_addr(ioctl_addr),
	.ioctl_dout(ioctl_dout), .ioctl_index(ioctl_index), .ioctl_wait(ioctl_wait),
	.ioctl_upload(ioctl_upload), .ioctl_upload_req(ioctl_upload_req),
	.ioctl_upload_index(ioctl_upload_index), .ioctl_din(ioctl_din),

	.whirly_a(whirly_a), .whirly_b(whirly_b),
	.btn1(btn1), .btn2(btn2),
	.coin1(coin1), .coin2(coin2), .self_test(self_test),

	.ce_pix(ce_pix), .vga_r(core_r), .vga_g(core_g), .vga_b(core_b),
	.hsync(core_hs), .vsync(core_vs), .hblank(core_hb), .vblank(core_vb),

	.aud_l(aud_l), .aud_r(aud_r),

	.SDRAM_DQ(SDRAM_DQ), .SDRAM_A(SDRAM_A), .SDRAM_BA(SDRAM_BA), .SDRAM_DQML(SDRAM_DQML),
	.SDRAM_DQMH(SDRAM_DQMH), .SDRAM_CKE(SDRAM_CKE), .SDRAM_nCS(SDRAM_nCS),
	.SDRAM_nRAS(SDRAM_nRAS), .SDRAM_nCAS(SDRAM_nCAS), .SDRAM_nWE(SDRAM_nWE)
);

// SDRAM clock pin from the phase-shifted PLL output, not from ~clk_sys.  The
// shift lives in rtl/pll/pll_0002.v and sets when the controller captures read
// data; it is a board property, so it was found by sweeping it on real
// hardware rather than from static timing analysis.  225 to 315 degrees all
// work and the shipped value, +13095 ps, is 270 degrees, the centre of that
// window.  The graphics ROMs live in this SDRAM, so a wrong phase would show
// as corrupt tiles and sprites over a correct picture layout, not as a black
// screen.
assign SDRAM_CLK = clk_sdram;

///////////////////////   AUDIO   ////////////////////////////////

assign AUDIO_S = 1'b1;              // signed samples
assign AUDIO_L = aud_l;
assign AUDIO_R = aud_r;

///////////////////////   VIDEO   ////////////////////////////////

// CRT Adjust (H-Size / H-Position / V-Shift): rmonic79's crt_adjust engine with
// this core's OSD decode, read-rate generator and DE glue around it.  Placed
// ahead of arcade_video so the mixer, scandoubler and video_freak all see the
// adjusted raster.  With the stage On the output pixel rate is no longer
// ce_pix, which is why arcade_video is clocked from adj_ce rather than ce_pix.
// Off is a bit-exact bypass.
//
// H_TOTAL, V_TOTAL and CLK_PER_PIX describe the Blasteroids raster and are all
// load-bearing; HPOS_MODE = 1 selects the content-shift form of H-Position, for
// the reasons set out in rtl/video/blstroid_analog_adjust.sv.
//
// `crt_enable` is declared up with the hps_io wires because status_menumask
// reads it too: the three amounts disappear from the menu whenever the stage is
// not running, rather than sitting there being ignored.
wire [7:0] adj_r, adj_g, adj_b;
wire       adj_hs, adj_vs, adj_hb, adj_vb, adj_ce;

blstroid_analog_adjust #(
	.H_TOTAL(912), .V_TOTAL(262), .CLK_PER_PIX(4), .HPOS_MODE(1), .AW(11)
) u_analog_adjust
(
	.clk        (clk_sys),
	.ce_pix     (ce_pix),
	.osd_on     (crt_enable),
	.osd_hsize  (status[28:24]),
	.osd_hpos   (status[35:29]),
	.osd_vshift (status[41:36]),
	.r_in(core_r), .g_in(core_g), .b_in(core_b),
	.hs_in(core_hs), .vs_in(core_vs), .hb_in(core_hb), .vb_in(core_vb),
	.r_out(adj_r), .g_out(adj_g), .b_out(adj_b),
	.hs_out(adj_hs), .vs_out(adj_vs), .hb_out(adj_hb), .vb_out(adj_vb),
	.ce_out(adj_ce)
);

wire [7:0] R = adj_r;
wire [7:0] G = adj_g;
wire [7:0] B = adj_b;
wire       HSync  = adj_hs;
wire       VSync  = adj_vs;
wire       HBlank = adj_hb;
wire       VBlank = adj_vb;

// The cabinet raster is presented natively; MiSTer's forced_scandoubler stays
// framework-controlled for displays that need it.
wire [2:0] fx = 3'b000;

// Orientation (status[4]).  Blasteroids is ROT0, so the core never rotates:
// screen_rotate is held with no_rotate = 1, where it derives
// do_flip = no_rotate && flip and fb_en = ~no_rotate | flip
// (sys/arcade_video.v), i.e. "Original" is a pure passthrough with the DDR
// framebuffer disabled, and "Flip" is a 180-degree flip through the
// framebuffer.  direct_video bypasses the scaler entirely, so it forces the
// passthrough state (the standard MiSTer arcade idiom).
wire       rotate_ccw = 1'b0;
wire       no_rotate  = 1'b1;
wire       flip       = status[4] & ~direct_video;
wire       video_rotated;

wire vga_de_raw;

// WIDTH = 640 = the visible width of the Blasteroids raster (912 x 262 total).
// arcade_video sizes the scandoubler's line buffer from it, and a WIDTH smaller
// than the real active line silently truncates the picture.
// DW = 24: the colour RAM is xRGB 5:5:5 (SP-316 sheet 12) and the core expands
// it through the R/2R DAC transfer, so it hands over 8 bits per channel.
arcade_video #(.WIDTH(640), .DW(24)) arcade_video
(
	.clk_video (clk_sys),
	.ce_pix    (adj_ce),
	.RGB_in    ({R, G, B}),
	.HBlank    (HBlank),
	.VBlank    (VBlank),
	.HSync     (HSync),
	.VSync     (VSync),

	.CLK_VIDEO (CLK_VIDEO),
	.CE_PIXEL  (CE_PIXEL),
	.VGA_R     (VGA_R),
	.VGA_G     (VGA_G),
	.VGA_B     (VGA_B),
	.VGA_HS    (VGA_HS),
	.VGA_VS    (VGA_VS),
	.VGA_DE    (vga_de_raw),
	.VGA_SL    (VGA_SL),

	.fx                 (fx),
	.forced_scandoubler (forced_scandoubler),
	.gamma_bus          (gamma_bus)
);

// Scale (status[22:20]) maps onto video_freak's SCALE encoding, documented at
// sys/video_freak.sv as 0 normal, 1 V-integer, 2 HV-Integer-, 3 HV-Integer+,
// 4 HV-Integer.  "Narrower HV-Integer" is therefore 2 and plain "HV-Integer"
// is 4 -- they are not adjacent, so menu order and encoding deliberately differ.
wire [2:0] scale_sel = (status[22:20] == 3'd0) ? 3'd0 :   // Normal
                       (status[22:20] == 3'd1) ? 3'd1 :   // V-Integer
                       (status[22:20] == 3'd2) ? 3'd4 :   // HV-Integer
                                                 3'd2;    // Narrower HV-Integer

video_freak video_freak
(
	.CLK_VIDEO  (CLK_VIDEO),
	.CE_PIXEL   (CE_PIXEL),
	.VGA_VS     (VGA_VS),
	.HDMI_WIDTH (HDMI_WIDTH),
	.HDMI_HEIGHT(HDMI_HEIGHT),
	.VGA_DE     (VGA_DE),
	.VIDEO_ARX  (VIDEO_ARX),
	.VIDEO_ARY  (VIDEO_ARY),
	.VGA_DE_IN  (vga_de_raw),
	.ARX        (arx),
	.ARY        (ary),
	.CROP_SIZE  (12'd0),
	.CROP_OFF   (5'd0),
	.SCALE      (scale_sel)
);

screen_rotate screen_rotate (.*);

///////////////////////   STATUS LED   ///////////////////////////

assign LED_USER = ioctl_download;

endmodule
