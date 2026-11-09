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
- `lockstep.c` + `lockstep.sh`: the reference core (a git revision) and this
  tree in one process, compared after every instruction (registers, pc,
  cycles, interrupt/loop/AGU state, stack; memories every 64).  Needs GNU
  ld/objcopy (the build box):

      ./lockstep.sh export HEAD      # on a git checkout: ls_ref/ = HEAD's core
      ./lockstep.sh                  # build ./lockstep (LSFLAGS=-D... for both)
      ./lockstep fuzz 1 20000        # random programs, interrupts open 1 in 4
      ./lockstep replay DUMP 10000000 40   # a next_dsp profile dump from pc

  An injected sign-extension bug in a parallel move shows at step 4.
- `dspbench` extras: `PERF=1` prints A9 hardware counters (cycles, ARM
  instructions, branch/cache/TLB misses per DSP instruction; the A9 PMU
  needs kernel counting on, so it is not excluded); `PCSAMPLE=file` writes a
  SIGPROF PC histogram for addr2line.
