#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "minimig_a2065.h"

// Persisted selection (host-side concept, kept out of the binary mm_configTYPE
// blob whose loader does an exact sizeof() check).
#define STATE_FILE "/media/fat/config/a2065_iface.txt"

// Daemon binary. Deploy to a stable path on the MiSTer SD.
#define DAEMON     "/media/fat/linux/a2065d_doorbell"

static int cur_mode = -1;

const char *a2065_iface_msg(int mode)
{
	return (mode == A2065_ETH0) ? "eth0" : (mode == A2065_ETH1) ? "eth1" : "OFF";
}

int a2065_get_iface(void)
{
	if (cur_mode >= 0) return cur_mode;

	cur_mode = A2065_ETH1; // default

	FILE *f = fopen(STATE_FILE, "r");
	if (f)
	{
		char b[16] = {};
		if (fgets(b, sizeof(b), f))
		{
			if      (!strncmp(b, "off",  3)) cur_mode = A2065_OFF;
			else if (!strncmp(b, "eth0", 4)) cur_mode = A2065_ETH0;
			else if (!strncmp(b, "eth1", 4)) cur_mode = A2065_ETH1;
		}
		fclose(f);
	}
	return cur_mode;
}

static void a2065_persist(int mode)
{
	FILE *f = fopen(STATE_FILE, "w");
	if (f)
	{
		fprintf(f, "%s\n", a2065_iface_msg(mode));
		fclose(f);
	}
}

void a2065_stop(void)
{
	system("killall -q a2065d_doorbell 2>/dev/null");
}

void a2065_start(void)
{
	int mode = a2065_get_iface();

	a2065_stop();
	if (mode == A2065_OFF) return;

	// The stock minimig_netd daemon conflicts with the A2065 daemon over the
	// shared DDR3 mailbox region — disable it before starting ours.
	system("killall -q minimig_netd 2>/dev/null");

	char cmd[256];
	snprintf(cmd, sizeof(cmd),
		"%s --iface %s >/tmp/a2065d.log 2>&1 &",
		DAEMON, a2065_iface_msg(mode));
	system(cmd);
	printf("A2065: started daemon on %s\n", a2065_iface_msg(mode));
}

void a2065_set_iface(int mode)
{
	if (mode < A2065_OFF || mode > A2065_ETH1) mode = A2065_OFF;
	cur_mode = mode;
	a2065_persist(mode);
	a2065_start();
}
