# Blasteroids for MiSTer FPGA

A hardware recreation of the 1987 Atari Games arcade title **Blasteroids**,
implemented as an FPGA core for the
[MiSTer](https://github.com/MiSTer-devel/Main_MiSTer/wiki) platform.

This is not a software port. The core reconstructs the original PCB set — the
68000 game board plus an Atari JSA I sound board — from Atari's SP-316
schematic package, with MAME used as a functional oracle where the schematics
are silent. Where a schematic and MAME disagree, the schematic wins and the
disagreement is recorded — see
[Where this core differs from MAME](#where-this-core-differs-from-mame).

<img src="https://img.shields.io/badge/Quartus-17.0.2-blue" alt="Quartus 17.0.2"> <img src="https://img.shields.io/badge/license-GPL--3.0--or--later-blue" alt="GPL-3.0-or-later">

---

## Supported ROM sets

Five MAME sets run on identical RTL — they differ only in the main CPU ROMs.

| MRA | MAME set | Notes |
| --- | --- | --- |
| `Blasteroids.mra` | `blstroid` | rev 4, the parent set |
| `_alternatives/_Blasteroids/Blasteroids (rev 3).mra` | `blstroid3` | rev 3 |
| `_alternatives/_Blasteroids/Blasteroids (rev 2).mra` | `blstroid2` | rev 2 |
| `_alternatives/_Blasteroids/Blasteroids (German, rev 2).mra` | `blstroidg` | German text |
| `_alternatives/_Blasteroids/Blasteroids (with heads).mra` | `blstroidh` | the "with heads" variant |

Each alternate MRA declares `zip="<clone>.zip|blstroid.zip"`, because a MAME
clone zip holds only the ROMs that differ; keep the parent `blstroid.zip`
alongside any clone you use.

---

## Requirements

| Item | Requirement |
| --- | --- |
| Board | DE10-Nano (Cyclone V `5CSEBA6U23I7`) |
| SDRAM | **Required** — a standard 32 MB module or larger |
| Display | Horizontal (ROT0) cabinet; no rotation is needed or offered |
| ROMs | Supplied by you from a MAME archive; none are included here |

**The SDRAM module is not optional.** All 1.25 MB of graphics data — 256 KB of
playfield stamps and 1 MB of motion-object stamps — is streamed from SDRAM
every scanline and there is no block-RAM fallback. The 256 KB program ROM and
the 64 KB sound ROM live in FPGA block RAM.

---

## Installing

1. Copy `releases/Blasteroids_YYYYMMDD.rbf` to `/media/fat/_Arcade/cores/`.
2. Copy `releases/Blasteroids.mra` to `/media/fat/_Arcade/`, and any wanted
   alternates from `releases/_alternatives/_Blasteroids/` to
   `/media/fat/_Arcade/_alternatives/_Blasteroids/`.
3. Place the matching MAME ROM archive (`blstroid.zip`, plus the clone zip for
   an alternate) in `/media/fat/games/mame/`.

**No ROM data is included in this repository, and none ever will be.** The MRA
files reference ROM parts by name and CRC only; MiSTer assembles the image the
core loads from an archive you already own.

---

## Controls

Blasteroids is a two-player game. Each player has an optical **Whirly-Gig**
knob and three buttons. The control panel is colour-coded: player 1 is the
left, red side and player 2 the right, green side.

| Button | Default pad | Function |
| --- | --- | --- |
| Fire | A | Fire |
| Thrust | B | Thrust |
| Transform | X | Change ship form (the self test calls it *Xform*) |
| Spare | Y | Unwired in a cabinet, but shown by the self test's Switch Test |
| Coin | Select | Coin (both chutes credit the same pool) |

**There is no Start button.** The cabinet has a *continuous buy-in* feature:
a player joins at any time by inserting a coin. The operator manual lists
exactly four switches per player, and the MAME driver defines no Start input.

**The four D-pad directions are mostly unused.** MiSTer's controller-mapping
screen asks for Up/Down/Left/Right before it reaches the named buttons; that
prompt comes from the framework. Left and Right do step the knob at a fixed
rate so a plain gamepad or keyboard can play; Up and Down are ignored.

### The Whirly-Gig knob

The knob is an optical encoder read by an Atari LETA custom, so the core
takes it from an analog source. All of these are summed and any of them works
with no selector:

| Source | How it is read |
| --- | --- |
| Spinner | `spinner_0` / `spinner_1` — the natural fit |
| Paddle | `paddle_0` / `paddle_1`, differenced, so a paddle at its stop stops turning |
| Mouse | PS/2 mouse X axis — player 1 only |
| D-pad Left / Right | Stepped at a fixed rate |

Two OSD options tune it:

* **Whirly sensitivity** — `1x` (default), then 1.125x to 4x, and 0.25x to
  0.875x in eighths. The game squares the per-frame knob delta and applies an
  acceleration flag at four counts per frame, so small steps here change the
  feel noticeably. Pick it by playing. Spinner input is spread into evenly
  spaced counts and capped at about 100 counts per frame, so no setting can
  make the ship spin the wrong way (the game's 8-bit difference wraps at 128).
* **Whirly direction** — `Normal` (default) or `Inverted`.

### Service and self test

* **Service** (OSD, default Off) is SW1 on the JSA audio PCB, behind the
  upper coin door. It is a toggle, not a momentary button.
* The self test's **Switch Test** page is the acceptance check for every
  input: press each button and its name highlights; spin each knob and the
  counter under "red knob" / "green knob" moves. Leave the page with red
  Thrust and red Fire together.

---

## Options

Blasteroids has **no DIP switches**. Every operator setting — coinage, lives,
difficulty, attract sound — lives in the on-board X2804 EEPROM and is edited
from the game's own self test (Coin Options and Game Options), reached with
the OSD **Service** toggle. Settings and bookkeeping persist through MiSTer's
*Save Settings* (the MRA declares a 512-byte NVRAM).

| Option | Values |
| --- | --- |
| Aspect ratio | Original, Full Screen, and the two MRA-supplied ratios |
| Orientation | Original / Flip |
| Scale | Normal, V-Integer, HV-Integer, Narrower HV-Integer |
| CRT Adjust (page) | Off / On, then CRT H-Size −16…+15, CRT H-Position −64…+63, CRT V-Shift −32…+31 |
| Whirly sensitivity / direction | See [The Whirly-Gig knob](#the-whirly-gig-knob) |
| Service | Off / On |

**CRT Adjust** is for lining the picture up on an analog CRT. It is
rmonic79's [MiSTer-CRT-Adjust](https://github.com/rmonic79/MiSTer-CRT-Adjust)
engine, core-side variant: the picture moves and resizes through a line buffer
while the sync stays native, so the monitor cannot lose lock however far you
push it, and no pixel is blended or duplicated. **Off is a bit-exact bypass**;
leave it Off for an untouched HDMI picture, because when it is On the whole
video path follows the adjustment. The three amounts are hidden while the
stage is inactive, which includes the case where MiSTer has forced the
scandoubler.

**Audio** is stereo, as on the board: the YM3012's two channels drive the two
speaker outputs through separate low-pass filters. There is no audio option.

---

## Hardware baseline

| Block | Implementation |
| --- | --- |
| Main CPU | Motorola 68000 @ 7.15909 MHz (14.318181 MHz crystal / 2), fx68k cycle-exact |
| Program ROM | 256 KB, four 27512s, even/odd interleave |
| Protection | None — SP-316 sheet 3 notes *"The SLAPSTIC chip is not installed"* |
| Sound board | Atari JSA I: 6502 @ 1.7898 MHz + YM2151. POKEY and TMS5220 are not populated on this game |
| Raster | 912 × 262 total, **640 × 240 visible**, ~59.92 Hz, 14.318181 MHz pixel clock, horizontal |
| Playfield | 40 × 30 stamps of 16 × 8 pixels, 4 bpp, no scrolling |
| Motion objects | Linked-list engine driven by the SOS-2 and SOS-1 customs, full 640-wide resolution, restarted every scanline |
| Priority | 256 × 1 priority RAM (playfield pen vs motion-object pen) |
| Palette | 512 × 16-bit colour RAM, xRGB 5:5:5, through three R/2R ladder DACs |
| Controls | LETA optical-encoder counter per player, three buttons, coins on the audio board |
| NVRAM | Xicor X2804 parallel EEPROM, unlock-then-write, busy while programming |
| Watchdog | LS90 decade counter clocked by /VBLANK; resets the 68000 after 8 frames without a write |

The whole core runs in one 57.272727 MHz `clk_sys` domain with clock enables.
That is four fabric clocks per 14.318181 MHz pixel, eight per 68000 clock,
and the sound board's 3.579545 MHz crystal is exactly the video crystal
divided by four, so every rate on both boards is an integer division of one
clock and no clock-domain crossing exists inside the game logic.

Resource use on the DE10-Nano's Cyclone V: about 14,700 / 41,910 ALMs (35 %)
and 471 / 553 M10K blocks (85 %). Timing closes with zero negative slack on
every clock at all four corners.

---

## Accuracy and verification

This core was developed with AI assistance. Because that raises a fair
question about whether the result was actually checked, here is the
verification record.

**Simulation suite** — some forty module testbenches (Icarus Verilog, GHDL and
Verilator) plus Verilator lint, covering the address decode, DTACK and bus
timing, the interrupt latches and watchdog, the EEPROM, the SDRAM controller
and arbiter, every video stage, and the full JSA I sound path.

**Full-core co-simulation** — the shipping RTL, with fx68k, T65, jt51, both
boards and the SDRAM controller against a chip model, boots cold from the real
MRA download image and is compared bus-event by bus-event against an
instrumented MAME 0.288. Over 44,000 bus events and 43,000 executed program
counters match with zero divergences. All three interrupt levels are taken,
including the row-programmable playfield interrupt that MAME does not
implement, and the sound mailbox closes between the two real CPUs.

**Pixel-exact compositor gate** — MAME is instrumented to capture complete
video state at chosen frames; that state is replayed through the real core
RTL and the rendered frame compared to MAME's own output pixel for pixel.
**15 / 15 frames** from attract, self test and gameplay are bit-identical over
all 153,600 pixels.

**On real hardware** (DE10-Nano) — tested by a number of users with no open
issues: boot, gameplay, sound, the self test, coins and EEPROM persistence.
The SDRAM read-capture phase was swept on hardware (every phase from 225° to
315° passes; 270° is shipped, in the middle of that range), and the Whirly-Gig
speed cap and sensitivity steps were tuned from reports by spinner owners.

---

## Where this core differs from MAME

MAME 0.288 was this project's functional oracle and every gate above measures
the core *against* it, so this is a narrow claim. It applies where the SP-316
schematics document something MAME's model does not.

| Behaviour | MAME | This core |
| --- | --- | --- |
| Row-programmable interrupt (IRQ1) | Stubbed out; MAME takes zero level-1 interrupts | Real and fully wired: sheet 6 latches VD15 as PFM2 and sheet 2's LS74 is clocked by HBLANK, so the interrupt is one scanline wide and self-clearing. The game arms 2–10 rows per frame and uses it to rewrite the first motion-object link per 8-line band |
| Motion objects per scanline | Walks the whole linked list | One link per 8 H counts with the walk restarted every line, so at most 57 links are followed |
| Video DAC | Linear `xRGB_555` | The three R/2R ladders on sheet 11, including their code-0 pedestal |
| JSA I periodic interrupt | A free-running 249.7 Hz timer | The LS393 divider is held cleared by the interrupt flip-flop until the 6502 acknowledges, so the period depends on acknowledge latency (about 258 Hz in practice) |
| JSA I `/RDIO` status byte | D7 (self-test switch) is stuck at 0, so the sound board's own self test is unreachable; D6 is declared active-low; D3 is unused | D7 follows the Service switch; D6 reads 1 while a command is pending, as the sheet's LS240 inverter makes it; D3 is a fourth coin input |
| Watchdog | Generic timer with default timing | Sheet 2's LS90: reset after 8 frames without a write, held for two frames, parked at 9 on power-up |

The first two are differences in *mechanism*. On every captured frame they are
observationally identical to MAME, because the per-band chains the game
installs are short and no captured scanline exceeds 57 live links. The core
implements the hardware-derived behaviour because that is what the board does.

---

## Open questions

Everything below is settled as far as the documents can settle it. What is
left needs a real Blasteroids PCB or cabinet part. If you have one and are
willing to help, open an issue.

1. **262 or 263 lines per frame.** The SOS-2 custom counts V internally and
   this board has no sync PROM, so the schematic package cannot say. The core
   uses 262 (MAME's published figure) as a compile-time parameter; rebuild
   with `V_TOTAL = 263` to try the other. A frame-rate measurement on a real
   board settles it.
2. **Whirly-Gig resolution.** The knob's slots per revolution are not
   documented. The core's speed cap is chosen from the game's own arithmetic
   (the 8-bit per-frame difference must not wrap), not from a measured knob.
3. **Which YM2151 channel is left.** SP-316 does not label them, so the core
   follows MAME.
4. **LETA count direction.** There is no LETA datasheet. The shipped default
   plays correctly for everyone who has reported; the *Whirly direction*
   option exists in case a setup disagrees.

---

## Building

Quartus Prime **17.0.2** (Lite or Standard) — the version MiSTer standardises
on; a newer Quartus rewrites `sys/`.

```
quartus_sh --flow compile Blasteroids
```

The revision is named `Blasteroids`, so the bitstream lands in
`output_files/Blasteroids.rbf` with no rename step, and the MRA's
`<rbf>Blasteroids</rbf>` resolves against it. `clean.bat` removes the build
products.

Source files are listed in `files.qip`; add and remove entries there by hand
rather than through the Quartus GUI, which rewrites `Blasteroids.qsf`. Never
edit `sys/` — it is the untouched MiSTer framework.

---

## Attribution

This core's own RTL is **GPL-3.0-or-later**, which is also the licence of the
combined work in `LICENSE`. Third-party components keep their own notices,
which must be preserved; `NOTICE.md` lists each with its path and licence.

| Component | Author | License |
| --- | --- | --- |
| MiSTer framework (`sys/`) | Alexey Melnikov (sorgelig) and contributors | GPL-2.0-or-later / GPL-3.0-or-later per file |
| fx68k (68000) | Jorge Cwik (ijor) | GPL-3.0-or-later |
| T65 (6502) | Daniel Wallner, with fixes by Mike Johnson, Wolfgang Scherr and others (OpenCores / FPGAArcade) | BSD-style |
| jt51 (YM2151) | Jose Tejada (jotego) | GPL-3.0-or-later |
| `crt_adjust` | Umberto Parisi (rmonic79) | GPL-3.0-or-later |
| PLL wrapper (`rtl/pll/`) | Intel FPGA IP, as generated | Intel FPGA IP licence |

The SDRAM controller, ROM loader and video-pipeline structure come from this
author's [Arcade-Toobin_MiSTer](https://github.com/MiSTer-devel/Arcade-Toobin_MiSTer)
and [Arcade-Xybots_MiSTer](https://github.com/MiSTer-devel/Arcade-Xybots_MiSTer)
cores.

Thanks to the MAME team, whose `blstroid.cpp`, `atarijsa` and `atarimo` code
served as the functional oracle throughout, and to whoever preserved the Atari
SP-316 schematic package and the TM-316 operator manual.

Blasteroids and its ROMs, artwork and manuals are © Atari Games. This project
contains no copyrighted ROM data.

Core by **RetroShrimp**.
