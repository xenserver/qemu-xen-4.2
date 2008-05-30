extern char domain_name[64];
extern int domid;

#define bool xen_bool
#include "xenctrl.h"
#include "xs.h"
#undef bool

extern int xc_handle;
