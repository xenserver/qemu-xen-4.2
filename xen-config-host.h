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

#undef CONFIG_GDBSTUB

extern int xc_handle;
extern int xen_pause_requested;
