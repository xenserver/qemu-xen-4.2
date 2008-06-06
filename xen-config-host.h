extern char domain_name[64];
extern int domid;

#include <errno.h>

#define bool xen_bool
#include "xenctrl.h"
#include "xs.h"
#include "blktaplib.h"
#undef bool

#undef CONFIG_GDBSTUB

extern int xc_handle;
