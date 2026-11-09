#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include "dsp_core.h"
#include "dsp_cpu.h"
static void hi(int x){(void)x;}
static uint64_t s; static uint32_t rnd(void){ s^=s<<13; s^=s>>7; s^=s<<17; return (uint32_t)s; }
static uint32_t ram[8192];
static uint64_t h=1469598103934665603ULL; static void mix(const void*p,size_t n){const uint8_t*b=p; for(size_t i=0;i<n;i++){h^=b[i];h*=1099511628211ULL;}}
int main(int c,char**v){ s=strtoull(v[1],0,0)|1; long rounds=atol(v[2]);
 dsp_core_init(hi);
 for(long r=0;r<rounds;r++){
  dsp_core_reset(); dsp_core.mode=0; dsp_core_config_ramext(ram,8192);
  for(int i=0;i<512;i++){ dsp_core.ramint[0][i]=rnd()&0xffffff; dsp_core.ramint[1][i]=rnd()&0xffffff;
    /* P: mostly ALU/parallel-move instructions, some others */
    uint32_t op=rnd()&0xffffff; if(rnd()%4) op|=0x100000; dsp_core.ramint[2][i]=op; }
  for(int i=0;i<8192;i++) ram[i]=rnd()&0xffffff;
  for(int i=0;i<8;i++){ dsp_core.registers[DSP_REG_R0+i]=rnd()&0x1ff; dsp_core.registers[DSP_REG_N0+i]=rnd()%5; uint32_t m=rnd()%4; dsp_core.registers[DSP_REG_M0+i]= m==0?0xffff: m==1? (rnd()%40)+1 : m==2? 0 : (rnd()&0x7fff); }
  dsp_core.registers[DSP_REG_X0]=rnd()&0xffffff; dsp_core.registers[DSP_REG_X1]=rnd()&0xffffff; dsp_core.registers[DSP_REG_Y0]=rnd()&0xffffff; dsp_core.registers[DSP_REG_Y1]=rnd()&0xffffff;
  dsp_core.registers[DSP_REG_A0]=rnd()&0xffffff; dsp_core.registers[DSP_REG_A1]=rnd()&0xffffff; dsp_core.registers[DSP_REG_A2]=rnd()&0xff;
  dsp_core.registers[DSP_REG_B0]=rnd()&0xffffff; dsp_core.registers[DSP_REG_B1]=rnd()&0xffffff; dsp_core.registers[DSP_REG_B2]=rnd()&0xff;
  dsp_core.registers[DSP_REG_SR]= (rnd()&0x0c00) | 0x0300;  /* random scaling mode, interrupts masked */
  dsp_core.pc=rnd()&0x1ff;
  for(int k=0;k<400;k++){ dsp56k_execute_instruction(); if(dsp_core.pc>=0x200) dsp_core.pc&=0x1ff; }
  mix(dsp_core.registers,sizeof dsp_core.registers); mix(dsp_core.ramint,sizeof dsp_core.ramint); mix(ram,sizeof ram); mix(&dsp_core.pc,2); mix(dsp_core.stack,sizeof dsp_core.stack);
  if((r+1)%(rounds/10)==0) printf("round %ld hash %016llx\n",r+1,(unsigned long long)h);
 }
}
