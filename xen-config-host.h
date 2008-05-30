extern char domain_name[64];
extern int domid;

#include <errno.h>

#define bool xen_bool
#include "xenctrl.h"
#include "xs.h"
#undef bool

extern int xc_handle;
