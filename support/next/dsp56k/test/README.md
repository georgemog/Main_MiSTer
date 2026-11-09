# DSP core checks (not built into MiSTer)

- `fuzz.c` + `hooks.c`: differential fuzz.  Random P/X/Y memory, registers
  and address modes, 400 instructions per round; prints a running hash of
  the DSP state every 1/10 of the rounds.  Build it against this tree and
  against an unmodified import (`git show <rev>:support/next/dsp56k/...`)
  and compare the output, e.g.

      cc -O2 -w -I.. -o fuzz_new fuzz.c hooks.c ../dsp_core.c ../dsp_cpu.c ../dsp_disasm.c
      ./fuzz_new 1 20000 > new.txt   # same for the reference build, then cmp

  It catches a wrong carry in add56 and a wrong modulo buffer mask.
- `dspbench.cpp`: speed on the MiSTer (thread CPU time), the bootstrap idle
  loop and a 16-tap FIR (MAC with XY moves, modulo, DO), with or without
  the link code.  Build notes are in its header.
