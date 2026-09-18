# Vendored third-party RTL — `rtl/lib/`

Everything in this directory is **third-party code copied verbatim**, headers
intact. Nothing here is edited: any Blasteroids-specific behaviour lives in
wrapper modules under `rtl/main`, `rtl/sound` or `rtl/video`, not in these
files. `NOTICE.md` at the repository root is the authoritative list of
third-party components and their licences; this file records where each copy
came from.

| Directory | Component | Upstream | Licence | Used as |
| --- | --- | --- | --- | --- |
| `fx68k/` | fx68k — Jorge Cwik's cycle-exact 68000, microcode level | <https://github.com/ijor/fx68k>, commit `0602ee4627b10f301298f2673d826cdd6baa9327` ("Optional output E signal clock enable", 2021-02-16) | **GPL-3.0-or-later** (`fx68k/LICENSE` is the GPL-3.0 text; every source file carries the "version 3 or later" header) | the main CPU (`rtl/main/blstroid_cpu.sv`) |
| `T65/` | T65 6502 soft core | Daniel Wallner / MikeJ / WoS (FPGAArcade), OpenCores; copied from the author's Toobin' MiSTer core | **BSD 3-clause** (see the header in `T65.vhd`) | the JSA I sound CPU (`rtl/sound/blstroid_jsa.sv`) |
| `jt51/` | JT51 Yamaha YM2151 (OPM) | Jose Tejada Gomez (jotego), <https://github.com/jotego/jt51>; copied from the author's Toobin' MiSTer core | **GPL-3.0-or-later** | the YM2151 (`rtl/sound/blstroid_ym.sv`) |

## fx68k

`fx68k/` is byte-for-byte upstream; all eight files of the repository are
present (`LICENSE`, `README.md`, `fx68k.sv`, `fx68k.txt`, `fx68kAlu.sv`,
`uaddrPla.sv`, `microrom.mem`, `nanorom.mem`). SHA-256, as fetched:

```
a718bb869a8a969dbdda89facd76f8621f655f8a84e7afa9551b2785eee5656a  fx68k.sv
3f22006cbdb8cb661e5f54cc779f79bb697b587c86d586103478703fca38bcce  fx68kAlu.sv
07ceb3b1fbbd74f255c2a909151fe5efd38a575ba4f6aa47c6208050d2e27326  uaddrPla.sv
9d13082be0cf4b04bff887e6da23de84f9f2ee4321f3a2576b4f9540615dece6  microrom.mem
b3998009fb10605e8468cddf22fdd799a9792ba86f449de2e30615cfcefbeece  nanorom.mem
3972dc9744f6499f0f9b2dbf76696f2ae7ad8af9b23dde66d6af86c9dfb36986  LICENSE
c96263ece59f55f759c046ca8027b67a6805e9cb31995f0a1bc175a826e15f96  README.md
2a7d451e8f7ae8683e4ec4f40a9b3adf91fb5c8b5d569c0f4cdb2b5de090811d  fx68k.txt
```

fx68k loads its microcode with **relative** `$readmemb` paths
(`"microrom.mem"` and `"nanorom.mem"` in `fx68k.sv`). The files are not
edited, so the Quartus build needs
`set_global_assignment -name SEARCH_PATH rtl/lib/fx68k`, which `files.qip`
provides. A build that silently fails to find them synthesises an **empty**
microcode ROM and boots to a black screen with no error that looks like the
cause. Check the generated `db/Blasteroids.ram0_uRom_*.hdl.mif` and
`db/Blasteroids.ram0_nanoRom_*.hdl.mif` after a build if in doubt: an unloaded
ROM has zero non-zero words.

## T65 and jt51

* `T65/T65_wrap.vhd` is **not** upstream T65. It is a thin flat-port wrapper
  written for the author's Toobin' core so a VHDL-to-Verilog netlist can be
  generated for simulation (the T65 `DEBUG` port is a VHDL record and cannot
  be a top-level netlist port). It carries the same BSD terms as the core it
  wraps.
* `jt51/hdl/deprecated/` and `jt51/hdl/filter/` are carried so the tree
  matches upstream; only `jt51/hdl/jt51.qip` is registered for synthesis, and
  it lists neither directory.
* **POKEY and TMS5220 are deliberately absent.** Blasteroids' JSA I board has
  neither part populated, so no POKEY or speech core is vendored.

## Lint and warnings

The vendored cores are not held to this repository's warning-clean bar. In the
Quartus build, jt51's `has no driver or initial value` warnings on the
`lfo_lut` / `sinetable` / `explut` write ports are upstream and expected.
