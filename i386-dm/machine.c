void cpu_save(QEMUFile *f, void *opaque) { }
int cpu_load(QEMUFile *f, void *opaque, int version_id) { return 0; }

static void register_machines(void)
{
    qemu_register_machine(&xenfv_machine);
    qemu_register_machine(&xenpv_machine);
}
