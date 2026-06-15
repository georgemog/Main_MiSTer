
#ifndef __MINIMIG_A2065_H__
#define __MINIMIG_A2065_H__

// A2065 ethernet interface selection (host-side daemon control)
#define A2065_OFF  0
#define A2065_ETH0 1
#define A2065_ETH1 2

// Returns the persisted iface mode (default A2065_ETH1 if no state file).
int  a2065_get_iface(void);

// Sets + persists the iface mode and (re)starts/stops the daemon.
void a2065_set_iface(int mode);

// Applies the current mode: (re)starts the daemon, or stops it if OFF.
// Idempotent (kills any running daemon first). Called on minimig boot/reset.
void a2065_start(void);

// Stops the daemon. Called when leaving the minimig core.
void a2065_stop(void);

// Human-readable label for the OSD ("OFF" / "eth0" / "eth1").
const char *a2065_iface_msg(int mode);

#endif
