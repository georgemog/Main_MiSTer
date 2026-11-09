#!/bin/sh
# lockstep.sh [REV]: build ./lockstep, the core at git REV (default HEAD) as
# ref_*, this tree as new_*.  Needs GNU ld and objcopy (Linux); CROSS=arm-none-linux-gnueabihf- builds it for the MiSTer.
set -e
REV=${REV:-${1:-HEAD}}
T=$(dirname "$0"); cd "$T"
if [ "${1:-}" = export ]; then
	REV=${2:-HEAD}; mkdir -p ls_ref
	for f in dsp_core.c dsp_cpu.c dsp_disasm.c; do git show "$REV:./../$f" > ls_ref/$f; done
	echo "ls_ref: $REV"; exit 0
fi
rm -rf ls_build && mkdir -p ls_build/ref ls_build/new
for f in dsp_core.c dsp_cpu.c dsp_disasm.c; do
	# ls_ref/ (written by: lockstep.sh export) when there is no git here
	if [ -f ls_ref/$f ]; then cp ls_ref/$f ls_build/ref/$f; else git show "$REV:./../$f" > ls_build/ref/$f; fi
	cp ../$f ls_build/new/$f
done
for v in ref new; do
	for f in dsp_core dsp_cpu dsp_disasm; do ${CROSS}gcc -O2 -w -I.. $LSFLAGS -c ls_build/$v/$f.c -o ls_build/$v/$f.o; done
	${CROSS}ld -r -o ls_build/$v.o ls_build/$v/dsp_core.o ls_build/$v/dsp_cpu.o ls_build/$v/dsp_disasm.o
	${CROSS}nm --defined-only -g ls_build/$v.o | awk -v p=${v}_ '{print $3" "p$3}' > ls_build/$v.map
	${CROSS}objcopy --redefine-syms=ls_build/$v.map ls_build/$v.o
done
${CROSS}gcc -O2 -w -I.. -o lockstep lockstep.c hooks.c ls_build/ref.o ls_build/new.o
echo "lockstep: ref = $REV, new = working tree"
