# NOTICE — third-party components and attributions

This repository's own RTL is **GPL-3.0-or-later**, and so is the combined work
(see `LICENSE`). The tree is not uniformly GPL-3.0: this file names every
third-party component in it, its authors and its licence, with the paths as
they exist in this repository. File headers are preserved in place and every
third-party file is present unedited unless stated otherwise below.

## Compiled into the bitstream

| Component | Author(s) | Licence | Location |
|---|---|---|---|
| MiSTer framework (`sys_top`, `hps_io`, `arcade_video`, `video_freak`, `ascal`, `osd`, `shadowmask`, `vga_out`, PLL configuration, …), taken from Template_MiSTer and **never edited** | Till Harbaum, Alexey Melnikov (sorgelig), MiSTer-devel contributors | GPL-2.0-or-later / GPL-3.0-or-later per file | `sys/` |
| **fx68k** — cycle-exact 68000 (microcode level), upstream commit `0602ee46`, whole repository. The main CPU. The microcode images `microrom.mem` and `nanorom.mem` are loaded by `$readmemb` with relative paths and are part of the component | Jorge Cwik (ijor), <https://github.com/ijor/fx68k> | GPL-3.0-or-later | `rtl/lib/fx68k/` |
| **T65** 6502 soft CPU — the JSA I sound CPU. `T65_wrap.vhd` is a thin flat-port wrapper written for the author's Toobin' core and carries the same terms | Daniel Wallner, Mike Johnson, Wolfgang Scherr, Morten Leikvoll (OpenCores / FPGAArcade) | BSD-style; the "redistributions in synthesized form" clause is satisfied by this notice | `rtl/lib/T65/` |
| **jt51** YM2151 (OPM) FM synthesis core. `hdl/filter/` and `hdl/deprecated/` are carried so the tree matches upstream and are not compiled | Jose Tejada (jotego), <https://github.com/jotego/jt51> | GPL-3.0-or-later | `rtl/lib/jt51/hdl/` |
| **crt_adjust** — "CRT Adjust", the analog-VGA line-buffer engine behind the OSD's CRT H-Size / H-Position / V-Shift, core-side variant, upstream commit `c682de9`. Vendored with four small adaptations recorded in the file's own header (`AW` made a parameter, the 2048-entry `initial` loop dropped, a sign-extension fix for negative H-Position, and a `VB_LATE` parameter, built as 0, because this raster's HSync follows the active line) | Umberto Parisi (rmonic79), <https://github.com/rmonic79/MiSTer-CRT-Adjust> | GPL-3.0-or-later | `rtl/video/crt_adjust.sv` |
| Altera/Intel PLL megafunction instance (`altera_pll`), hand-parameterised wrapper: 50 MHz × 63 / 5 / 11 = 57.272727 MHz | Intel Corporation (generated IP) | Intel FPGA IP licence, as generated | `rtl/pll.v`, `rtl/pll.qip`, `rtl/pll/` |

`rtl/lib/README.md` is the provenance record for the three vendored CPU and
sound trees: upstream URLs, the fx68k commit and its per-file SHA-256 sums.

**POKEY and TMS5220 are deliberately absent.** Blasteroids' JSA I audio PCB
has neither part populated, so no POKEY or TMS core is vendored.

## Derived work — the author's earlier MiSTer cores

The Blasteroids RTL, the framework glue and the project files were written by
the same author as the Xybots and Toobin' MiSTer cores (both GPL-3.0), and most
modules are structurally ported from the Xybots core — which in turn took its
sound, memory and loader modules from Toobin', whose SDRAM controller is a
SystemVerilog re-implementation of the Arcade-Atari-system1 `sdram.vhd`
command encoding. Every such file names its ancestor in its own header; in
each case the board-specific logic (address decode, DTACK, interrupts, the
SOS-2 raster, the motion-object arithmetic, the DAC transfer, every sound
filter value) is re-derived from the SP-316 schematics.

| Group | Ancestor |
|---|---|
| `Blasteroids.qsf`, `.qpf`, `.srf`, `.sdc`, `files.qip`, `clean.bat`, `.gitignore`, `Arcade-Blasteroids.sv`, `rtl/blstroid_core.sv` | the Xybots core's project files, framework glue and core top |
| `rtl/main/blstroid_{addr_decode,cpu,dtack,irq,main,watchdog,eeprom_2804,nvram_io,dpram_be}.sv`, `rtl/mem/blstroid_ram8.sv` | the Xybots core's `rtl/main/xybots_*.sv` (EEPROM and NVRAM transport originally from Toobin') |
| `rtl/mem/blstroid_{sdram,gfx_mem,sdram_loader,rom_loader,prog_rom}.sv` | the Xybots core's `rtl/mem/xybots_*.sv`, from Toobin' |
| `rtl/sound/blstroid_*.sv` | the Xybots core's `rtl/sound/xybots_*.sv`, from Toobin' (the same JSA I board) |
| `rtl/video/blstroid_{sos2_sync,pf,vram,mo,mo_linebuf,prio,cram,dac,video,testpat}.sv` | the Xybots core's `rtl/video/xybots_*.sv` (`xybots_syngen`, `xybots_mo*`, …), plus Toobin' `toobin_mo*.sv` for the motion-object engine — module shape only |
| `rtl/video/blstroid_analog_adjust.sv` | rmonic79's `examples/core_side_snippet.v` from MiSTer-CRT-Adjust (OSD decode, read-rate generator), re-derived for a 4-clock pixel |

`rtl/main/blstroid_leta.sv`, `blstroid_whirly.sv` and `blstroid_inputs.sv` are
written for this core: Blasteroids' Whirly-Gig is an optical encoder read by an
Atari LETA custom, which no prior core of the author's has. `blstroid_leta.sv`'s
header names the Arcade-Atari-System2 core's `sys2_leta.sv` (GPL-2.0-or-later,
RetroShrimp) as the tested model it was checked against; no code was copied
from it.

## Reference material (read for behaviour, never compiled or redistributed)

MAME 0.288 driver sources (`blstroid.cpp`, `atarijsa.cpp`, `atarimo.cpp`;
BSD-3-Clause, Aaron Giles and the MAME contributors) were used as a functional
oracle. The Atari SP-316 schematic package, the Blasteroids operator manual
TM-316 and all ROM images are inputs that are never committed or redistributed
by this project.

## Trademarks and content

This project distributes no ROM data and no copyrighted artwork. *Blasteroids*
is a trademark of its rights holders. Use only with software you are legally
entitled to.
