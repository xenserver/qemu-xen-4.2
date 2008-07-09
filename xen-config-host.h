extern char domain_name[64];
extern int domid;

#include <errno.h>

#ifdef bool
# define XEN_CONFIG_HOST_BOOL_WAS_DEFINED 1
#else
# define bool xen_bool
#endif

#include "xenctrl.h"
#include "xs.h"
#include "blktaplib.h"

#ifndef XEN_CONFIG_HOST_BOOL_WAS_DEFINED
# undef bool
#endif

#define BIOS_SIZE ((256 + 64) * 1024)

#undef CONFIG_GDBSTUB

void main_loop_prepare(void);

extern int xc_handle;
extern int xen_pause_requested;
extern int vcpus;

#define DEFAULT_NETWORK_SCRIPT "/etc/xen/qemu-ifup"
#define DEFAULT_NETWORK_DOWN_SCRIPT "/etc/xen/qemu-ifdown"

void xenstore_parse_domain_config(int domid);
void xenstore_read_vncpasswd(int domid, char *pwbuf, size_t pwbuflen);
struct CharDriverState;
void xenstore_store_serial_port_info(int i, struct CharDriverState *chr,
				     const char *devname);
