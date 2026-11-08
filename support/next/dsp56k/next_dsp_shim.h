/*
	Stand-ins for the Hatari / Previous headers the DSP56001 core includes
	(main.h, log.h, debugui.h, profile.h, ioMem.h, statusbar.h, dsp.h), for
	Main_MiSTer's NeXT soft DSP (support/next/next_dsp.cpp).  Tracing and the
	debugger are compiled out; the DSP's links to the rest of the machine go
	to the next_dsp hooks below.

	This file is part of Main_MiSTer and is distributed under the GNU General
	Public License, version 2 or later (like the DSP core it serves).
*/
#ifndef NEXT_DSP_SHIM_H
#define NEXT_DSP_SHIM_H

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#ifdef __cplusplus
extern "C" {
#endif

/* log.h: no tracing */
#define LOG_TRACE(level, ...)        do { } while (0)
#define LOG_TRACE_LEVEL(level)       (0)
#define LOG_TRACE_DIRECT_INIT()      do { } while (0)
#define LOG_TRACE_PRINT(...)         do { } while (0)
#define TraceFile                    stderr
enum { LOG_FATAL, LOG_ERROR, LOG_WARN, LOG_INFO, LOG_TODO, LOG_DEBUG };
void next_dsp_log(int level, const char *fmt, ...);
#define Log_Printf next_dsp_log

/* debugui.h / profile.h: no debugger, no profiler */
#define REASON_DSP_EXCEPTION 0
#define DebugUI(reason)              do { } while (0)
#define ExceptionDebugMask           0
#define EXCEPT_DSP                   0
#define Profile_DspAddressData(pc, pct, cnt, cyc, diff) (false)

/* statusbar.h */
#define Statusbar_SetDspLed(on)      do { } while (0)

/* dsp.h: the SSI goes nowhere on the NeXT-Color (the DSP port connector) */
#define DSP_SsiTransmit_SC1()        do { } while (0)
#define DSP_SsiTransmit_SC2(frame)   do { } while (0)
#define DSP_InitSSI(value)           do { (void)(value); } while (0)

/* the DSP -> the link (next_dsp.cpp) */
void dsp_link_hrx_read(void);    /* the DSP read HRX while HRDF was set */
void dsp_link_txd(int low);      /* port C TXD driven low (1) / high (0) */
void dsp_link_hi_reset(void);    /* the RESET instruction reset the host port */
void dsp_link_hc_taken(void);    /* a host command interrupt was taken */
#define DSP_HandleTXD(set)           dsp_link_txd(set)

#ifdef __cplusplus
}
#endif

#endif
