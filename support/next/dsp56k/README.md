# DSP56001 core for the NeXT-Color soft DSP

This is the Motorola DSP56001 interpreter from **Hatari**
(https://github.com/hatari/hatari, `src/falcon/`, master 5081dd8 of
2026-11-07), which the ARAnyM developer team wrote and the Hatari team
maintains.  On top of it are the NeXT changes that **Previous** (r1851,
`src/dsp/`) carries.  `support/next/next_dsp.cpp` runs it against the
NeXT-Color core's host-port link.

## Licence

The core is GNU GPL version 2 **or (at your option) any later version**; see
the headers of `dsp_core.c`, `dsp_cpu.c` and `dsp_disasm.c`, and `COPYING`
(Hatari's `gpl.txt`).  Main_MiSTer is GPL v3, which a GPL-2.0-or-later work
may be combined with.  The files below keep their copyright lines.

## Files

| file | from |
|---|---|
| `dsp_core.c/h`, `dsp_cpu.c/h`, `dsp_disasm.c/h` | Hatari master + the Previous r1851 diff + link hooks |
| `next_dsp_shim.h` | new: stand-ins for Hatari's `log.h`, `debugui.h`, `profile.h` and the rest; the link hook prototypes |
| `main.h`, `log.h`, `debugui.h`, `profile.h`, `ioMem.h`, `statusbar.h`, `dsp.h` | one-line includes of `next_dsp_shim.h` |

## Changes from Hatari

1. **Previous r1851** (`diff -u hatari/src/falcon previous/src/dsp`, applied
   as is):
   - **The NeXT DSP memory map.**  External RAM is a pointer (`ramext`),
     sized by `dsp_core_config_ramext` (SCR2 DSP_MEM_EN): 8K words, or 32K
     with the expansion.  X and Y are separate only when A15 is high.
     Reads return 0 while the RAM is off.
   - **Start modes.**  `dsp_core_start(mode, bootstrap)`: mode 1 runs the
     32-word bootstrap P ROM (`p_rom`), which loads 512 words or stops at
     HF0, then jumps to `$0`.  Mode 2 starts at `$E000`.  An OMR write
     takes effect after `mode_wait`.
   - **Port C TXD (PCD bit 1) as the host's TXD interrupt**
     (`DSP_HandleTXD`).
   - The ICR write records the host's DMA mode and direction (`dma_*`).
   - Two SSI changes, which do nothing here, because the SSI is not
     connected.
2. **Link hooks** for `next_dsp.cpp`, declared in `next_dsp_shim.h`:
   - `dsp_core_hostport_dspread`: `dsp_link_hrx_read()` when the DSP reads
     HRX while HRDF is set, so one host word has been consumed.
   - `dsp_reset` (the RESET instruction): `dsp_link_hi_reset()`.
   - `dsp_postexecute_interrupts`: `dsp_link_hc_taken()` when a host
     command interrupt is taken.
   - `DSP_HandleTXD` maps to `dsp_link_txd()`.
3. Tracing, the debugger and the profiler are compiled out by the shim
   macros.

To redo the import:

```bash
cp hatari/src/falcon/dsp_{core,cpu,disasm}.[ch] .
```

```bash
for f in dsp_core.c dsp_core.h dsp_cpu.c dsp_disasm.c; do diff -u hatari/src/falcon/$f previous/src/dsp/$f | patch $f; done
```

Then put the four link hooks back (search for "NeXT-Color soft DSP"), and
point the `#include "main.h"` lines in `dsp_core.c` and `dsp_cpu.c` at
`next_dsp_shim.h`.

## Not done

- **The DO-loop end fix.**  The lost `next-color` Main branch had a fix,
  6a3ae03 "Previous's DO-loop end", whose contents are not known.  This
  import has not been checked against it.
- **SSI and SCI** (the DSP port connector) go nowhere.
