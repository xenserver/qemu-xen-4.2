/*
 * We #include this from vl.c.
 *
 * This is a bit yucky, but it means that the line numbers and other
 * textual differences in vl.c remain small.
 */
/* There is no need for multiple-inclusion protection since
 * there is only one place where this file is included. */

#include "qemu-xen.h"

/* forward declarations of things in vl.c */

static int qemu_savevm_state(QEMUFile *f);
static int qemu_loadvm_state(QEMUFile *f);

/* We use simpler state save/load functions for Xen */

void do_savevm(const char *name)
{
    QEMUFile *f;
    int saved_vm_running, ret;

    f = qemu_fopen(name, "wb");
    
    /* ??? Should this occur after vm_stop?  */
    qemu_aio_flush();

    saved_vm_running = vm_running;
    vm_stop(0);

    if (!f) {
        fprintf(logfile, "Failed to open savevm file '%s'\n", name);
        goto the_end;
    }
    
    ret = qemu_savevm_state(f);
    qemu_fclose(f);

    if (ret < 0)
        fprintf(logfile, "Error %d while writing VM to savevm file '%s'\n",
                ret, name);

 the_end:
    if (saved_vm_running)
        vm_start();

    return;
}
void do_loadvm(const char *name)
{
    QEMUFile *f;
    int saved_vm_running, ret;

    /* Flush all IO requests so they don't interfere with the new state.  */
    qemu_aio_flush();

    saved_vm_running = vm_running;
    vm_stop(0);

    /* restore the VM state */
    f = qemu_fopen(name, "rb");
    if (!f) {
        fprintf(logfile, "Could not open VM state file\n");
        goto the_end;
    }

    ret = qemu_loadvm_state(f);
    qemu_fclose(f);
    if (ret < 0) {
        fprintf(logfile, "Error %d while loading savevm file '%s'\n",
                ret, name);
        goto the_end; 
    }

#if 0 
    /* del tmp file */
    if (unlink(name) == -1)
        fprintf(stderr, "delete tmp qemu state file failed.\n");
#endif


 the_end:
    if (saved_vm_running)
        vm_start();
}

struct qemu_alarm_timer;
static int unix_start_timer(struct qemu_alarm_timer *t) { return 0; }
static void unix_stop_timer(struct qemu_alarm_timer *t) { }
