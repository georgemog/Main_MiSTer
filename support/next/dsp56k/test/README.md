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

  Dumps that exercise the JIT (mister2, /media/usb0/next_dsp/bench/):
  cov.bin, scale.bin and cov_t30.bin from pc 40 (Music Kit orchestra
  loops), pach_t40.bin from pc 40 (Pachelbel's orchestra) and from its own
  pc, cov_t15.bin from its own pc (a host polling loop), mulaw.bin (host
  polling, interpreted).  A dump is taken by writing a label to
  /media/usb0/next_dsp/dump while a score plays (no profiling needed); one
  taken in a score's first ~15 s is usually still in the bootstrap ROM (pc
  0017) and runs no JIT code: check `JIT=1 dspbench replay` reports blocks.
  pach_hang.bin is the DSP at Pachelbel's hang (in the bootstrap ROM).
- `dspbench` extras: `PERF=1` prints A9 hardware counters (cycles, ARM
  instructions, branch/cache/TLB misses per DSP instruction; the A9 PMU
  needs kernel counting on, so it is not excluded); `PCSAMPLE=file` writes a
  SIGPROF PC histogram for addr2line.
- JIT (`../dsp_jit.inc`, ARM only): `LOCKSTEP_JIT=1 ./lockstep ...` runs the
  new core through `dsp56k_jit_step` (blocks) against the reference
  interpreter, comparing state and cycles at each block end.  It must run on
  ARM: `CROSS=arm-none-linux-gnueabihf- ./lockstep.sh` on the build box,
  then run the binary on the MiSTer.  `NEXT_DSP_JIT_MAX=1` makes one-instruction
  blocks (to find which instruction differs); `NEXT_DSP_JIT_ALU=0` calls Hatari for
  every ALU op (to tell native ALU bugs from the rest); `NEXT_DSP_JIT=0` turns the JIT off.
  `NEXT_DSP_JIT_DBG=1` records each block instruction's cycles and the running
  total, which lockstep prints beside the reference's on a cycle mismatch;
  `NEXT_DSP_JIT_RC=0` keeps A/B in dsp_core instead of ARM registers;
  `NEXT_DSP_JIT_DBG=2` also ends the block after every ALU op whose flags
  were deferred, so lockstep checks the rebuilt SR each time.
  The fuzz seed is ORed with 1, so use odd seeds (2 and 3 are the same run);
  ROUNDS counts programs of about 420 instructions each.
  `dspbench`: `JIT=1` runs the kernels through the JIT.
