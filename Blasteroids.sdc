derive_pll_clocks
derive_clock_uncertainty

# ============================================================================
#  Blasteroids core timing constraints.
#
#  Copyright (C) 2026 RetroShrimp.  GPL-3.0-or-later; see LICENSE and NOTICE.md.
#  Derived from core/Xybots.sdc in the Xybots MiSTer core (GPL-3.0, same
#  author).
#
#  There is exactly ONE fabric clock in the game half of this design:
#
#     clk_sys = 57.272727 MHz  (emu|pll outclk_0 = PLL 50 MHz x 63 / 5 / 11)
#             = 4 x 14.318181 MHz, the Blasteroids master crystal.
#             Period 17.461 ns.
#
#  Everything below it advances on synchronous, integer clock ENABLES generated
#  in rtl/blstroid_core.sv from one free-running /32 counter:
#
#     ce_14m  = clk_sys / 4   = 14.318181 MHz  pixel clock
#     ce_7m   = clk_sys / 8   =  7.159091 MHz  68000
#     ce_ym   = clk_sys / 16  =  3.579545 MHz  YM2151 (JSA I)
#     ce_6502 = clk_sys / 32  =  1.789772 MHz  JSA I 6502
#
#  Each slower enable is a strict subset of every faster one (they are decodes
#  of the same counter), so no fractional or asynchronous enable exists anywhere
#  and there is no clock-domain crossing inside the game logic.  The only
#  genuine crossing between the two PCBs is the SCOM mailbox, which is modelled
#  with an explicit handshake rather than with a second clock.
#
#  outclk_1 is the same 57.272727 MHz, phase-shifted, driving the SDRAM_CLK pin.
# ============================================================================

# ============================================================================
#  SDRAM external I/O timing (MT48LC16M16 class, CL2, 57.272727 MHz controller).
#  This section is derived from Xybots.sdc in the Xybots MiSTer core (GPL-3.0,
#  same author).
#
#  SDRAM_CLK is PLL outclk_1 (the second output counter, general[1]) wired
#  straight to the pin in Arcade-Blasteroids.sv - a clean phase-controlled
#  clock, so these delays can actually close.  The numbers are datasheet
#  values.  If Quartus reports that the -source PLL node cannot be found, open
#  Timing Analyzer -> Report Clocks and substitute the actual
#  outclk_1 ...general[1]...divclk path; if the constraint does not apply, the
#  SDRAM I/O simply falls back to unconstrained rather than failing the build.
#
#  Do not tune the read-capture phase from the STA report.  The Setup Summary
#  is keyed by the CAPTURE clock, so the SDRAM_CLK row is the command/write
#  launch and the DQ return is buried in the clk_sys row with ordinary core
#  logic; neither isolates the capture, and the spread across refits is the
#  same size as the effect.  The shipped phase in rtl/pll/pll_0002.v was found
#  by sweeping it on real hardware: 225 to 315 degrees all work and 270 is the
#  centre of that window.
#
#  Seven "Pin SDRAM_* is stuck at GND/VCC" fitter warnings are expected and
#  correct: nCS (never deselected), DQML/DQMH (every access is a full 16-bit
#  word), BA[1:0] (the whole map is in bank 0) and A[12]/A[11] (1280 rows, so
#  only A[10:0] is ever non-zero).  If one of the seven starts toggling, a
#  behaviour has changed.
# ============================================================================
create_generated_clock -name SDRAM_CLK \
  -source [get_pins -compatibility_mode {*|pll|pll_inst|altera_pll_i|general[1].gpll~PLL_OUTPUT_COUNTER|divclk}] \
  [get_ports {SDRAM_CLK}]

# Read capture: data access time tAC = 6.0 ns (max), output hold tOH = 2.5 ns (min).
set_input_delay  -clock SDRAM_CLK -max 6.0 [get_ports {SDRAM_DQ[*]}]
set_input_delay  -clock SDRAM_CLK -min 2.5 [get_ports {SDRAM_DQ[*]}]

# Command/address/data launch: input setup tIS = 1.5 ns (max), hold tIH = 0.8 ns (min).
set_output_delay -clock SDRAM_CLK -max  1.5 [get_ports {SDRAM_A[*] SDRAM_BA[*] SDRAM_DQ[*] SDRAM_DQML SDRAM_DQMH SDRAM_nCS SDRAM_nRAS SDRAM_nCAS SDRAM_nWE SDRAM_CKE}]
set_output_delay -clock SDRAM_CLK -min -0.8 [get_ports {SDRAM_A[*] SDRAM_BA[*] SDRAM_DQ[*] SDRAM_DQML SDRAM_DQMH SDRAM_nCS SDRAM_nRAS SDRAM_nCAS SDRAM_nWE SDRAM_CKE}]

# The controller launches and captures on clk_sys (outclk_0); SDRAM_CLK is
# phase-shifted from it, so allow the read the proper (next) capture edge
# instead of the half-cycle one.  Endpoints must be CLOCK collections
# (get_clocks), not pins.  get_clocks does Tcl string matching where [..] is a
# character class, so the literal "general[0]" is matched with "?" wildcards;
# the *|pll|pll_inst| prefix disambiguates it from the framework's audio PLL.
set_multicycle_path -setup -end 2 \
  -from [get_clocks {SDRAM_CLK}] \
  -to   [get_clocks {*|pll|pll_inst|altera_pll_i|general?0?.gpll~PLL_OUTPUT_COUNTER|divclk}]
set_multicycle_path -hold -end 1 \
  -from [get_clocks {SDRAM_CLK}] \
  -to   [get_clocks {*|pll|pll_inst|altera_pll_i|general?0?.gpll~PLL_OUTPUT_COUNTER|divclk}]

# ============================================================================
#  The two exceptions above are the only ones in this file, and they exist for
#  the SDRAM_CLK-to-clk_sys phase relationship, not for the game logic.  No
#  module gets a multicycle exception of its own - not fx68k, not T65, not
#  jt51, not the SDRAM controller, not the graphics arbiter, not the
#  motion-object engine, not the video pipeline, not the bus decode.  The
#  design closes at the full clk_sys rate as written, and the video pipeline
#  runs at ce_14m = clk_sys/4, i.e. only four fabric clocks per pixel, so it is
#  nearer to full rate than in the sibling cores this one is derived from.
#
#  If an exception is ever added, add it with its justification and with a
#  guard that proves it still matches something: a get_registers filter that
#  matches nothing is a silent no-op, and a constraint that quietly stops
#  applying is worse than one that was never there.
# ============================================================================
