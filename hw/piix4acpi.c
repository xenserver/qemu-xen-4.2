 /*
 * PIIX4 ACPI controller emulation
 *
 * Winston liwen Wang, winston.l.wang@intel.com
 * Copyright (c) 2006 , Intel Corporation.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include "hw.h"
#include "pc.h"
#include "pci.h"
#include "sysemu.h"
#include "qemu-xen.h"
#include "battery_mgmt.h"

#include <xen/hvm/ioreq.h>
#include <xen/hvm/params.h>

/* PM1a_CNT bits, as defined in the ACPI specification. */
#define SCI_EN            (1 <<  0)
#define GBL_RLS           (1 <<  2)
#define SLP_TYP_Sx        (7 << 10)
#define SLP_EN            (1 << 13)

/* Sleep state type codes as defined by the \_Sx objects in the DSDT. */
/* These must be kept in sync with the DSDT (hvmloader/acpi/dsdt.asl) */
#define SLP_TYP_S4        (6 << 10)
#define SLP_TYP_S3        (5 << 10)
#define SLP_TYP_S5        (7 << 10)

#define ACPI_DBG_IO_ADDR  0xb044
#define ACPI_PHP_IO_ADDR  0x10c0

#define PHP_EVT_ADD     0x0
#define PHP_EVT_REMOVE  0x3

/* The bit in GPE0_STS/EN to notify the pci hotplug event */
#define ACPI_PHP_GPE_BIT 3

typedef struct AcpiDeviceState AcpiDeviceState;
AcpiDeviceState *acpi_device_table;

typedef struct PCIAcpiState {
    PCIDevice dev;
    uint16_t pm1_control; /* pm1a_ECNT_BLK */
} PCIAcpiState;

typedef struct GPEState {
    /* GPE0 block */
    uint8_t gpe0_sts[ACPI_GPE0_BLK_LEN / 2];
    uint8_t gpe0_en[ACPI_GPE0_BLK_LEN / 2];

    /* SCI IRQ level */
    uint8_t sci_asserted;

} GPEState;

static GPEState gpe_state;

typedef struct PHPSlots {
    uint8_t status[NR_PCI_DEV]; /* Apaptor stats */
    uint8_t plug_evt;           /* PHP_EVT_ADD or PHP_EVT_REMOVE
                                 * PSTA in ASL */
    uint8_t plug_slot;          /* Slot number
                                 * PSTB in ASL */
} PHPSlots;

static PHPSlots php_slots;
int s3_shutdown_flag;
static qemu_irq sci_irq;

static void piix4acpi_save(QEMUFile *f, void *opaque)
{
    PCIAcpiState *s = opaque;
    pci_device_save(&s->dev, f);
    qemu_put_be16s(f, &s->pm1_control);
}

static int piix4acpi_load(QEMUFile *f, void *opaque, int version_id)
{
    PCIAcpiState *s = opaque;
    int ret;
    if (version_id > 1) 
        return -EINVAL;
    ret = pci_device_load(&s->dev, f);
    if (ret < 0)
        return ret;
    qemu_get_be16s(f, &s->pm1_control);
    return 0;
}

static void acpiPm1Control_writeb(void *opaque, uint32_t addr, uint32_t val)
{
    PCIAcpiState *s = opaque;
    s->pm1_control = (s->pm1_control & 0xff00) | (val & 0xff);
}

static uint32_t acpiPm1Control_readb(void *opaque, uint32_t addr)
{
    PCIAcpiState *s = opaque;
    /* Mask out the write-only bits */
    return (uint8_t)(s->pm1_control & ~(GBL_RLS|SLP_EN));
}

static void acpi_shutdown(uint32_t val)
{
    if (!(val & SLP_EN))
        return;

    switch (val & SLP_TYP_Sx) {
    case SLP_TYP_S3:
        s3_shutdown_flag = 1;
        qemu_system_reset();
        s3_shutdown_flag = 0;
        cmos_set_s3_resume();
        xc_set_hvm_param(xc_handle, domid, HVM_PARAM_ACPI_S_STATE, 3);
        break;
    case SLP_TYP_S4:
    case SLP_TYP_S5:
        qemu_system_shutdown_request();
        break;
    default:
        break;
    }
}

static void acpiPm1ControlP1_writeb(void *opaque, uint32_t addr, uint32_t val)
{
    PCIAcpiState *s = opaque;

    val <<= 8;
    s->pm1_control = ((s->pm1_control & 0xff) | val) & ~SLP_EN;

    acpi_shutdown(val);
}

static uint32_t acpiPm1ControlP1_readb(void *opaque, uint32_t addr)
{
    PCIAcpiState *s = opaque;
    /* Mask out the write-only bits */
    return (uint8_t)((s->pm1_control & ~(GBL_RLS|SLP_EN)) >> 8);
}

static void acpiPm1Control_writew(void *opaque, uint32_t addr, uint32_t val)
{
    PCIAcpiState *s = opaque;

    s->pm1_control = val & ~SLP_EN;

    acpi_shutdown(val);
}

static uint32_t acpiPm1Control_readw(void *opaque, uint32_t addr)
{
    PCIAcpiState *s = opaque;
    /* Mask out the write-only bits */
    return (s->pm1_control & ~(GBL_RLS|SLP_EN));
}

static void acpi_map(PCIDevice *pci_dev, int region_num,
                     uint32_t addr, uint32_t size, int type)
{
    PCIAcpiState *d = (PCIAcpiState *)pci_dev;

    /* Byte access */
    register_ioport_write(addr + 4, 1, 1, acpiPm1Control_writeb, d);
    register_ioport_read(addr + 4, 1, 1, acpiPm1Control_readb, d);
    register_ioport_write(addr + 4 + 1, 1, 1, acpiPm1ControlP1_writeb, d);
    register_ioport_read(addr + 4 +1, 1, 1, acpiPm1ControlP1_readb, d);

    /* Word access */
    register_ioport_write(addr + 4, 2, 2, acpiPm1Control_writew, d);
    register_ioport_read(addr + 4, 2, 2, acpiPm1Control_readw, d);

    battery_mgmt_init(pci_dev);
}

#ifdef CONFIG_PASSTHROUGH

static inline int test_bit(uint8_t *map, int bit)
{
    return ( map[bit / 8] & (1 << (bit % 8)) );
}

static inline void set_bit(uint8_t *map, int bit)
{
    map[bit / 8] |= (1 << (bit % 8));
}

static inline void clear_bit(uint8_t *map, int bit)
{
    map[bit / 8] &= ~(1 << (bit % 8));
}

extern FILE *logfile;
static void acpi_dbg_writel(void *opaque, uint32_t addr, uint32_t val)
{
#if defined(DEBUG)
    printf("ACPI: DBG: 0x%08x\n", val);
#endif
    fprintf(logfile, "ACPI:debug: write addr=0x%x, val=0x%x.\n", addr, val);
}

/*
 * simple PCI hotplug controller IO
 * ACPI_PHP_IO_ADDR + :
 * 0 - the hotplug event
 * 1 - the slot that has a hotplug event
 * 2 - 1st php slot ctr/sts reg
 * 3 - 2nd php slot ctr/sts reg
 * ...
 */
static uint32_t acpi_php_readb(void *opaque, uint32_t addr)
{
    PHPSlots *hotplug_slots = opaque;
    int num;
    uint32_t val; 

    switch (addr)
    {
    case ACPI_PHP_IO_ADDR:
        val = hotplug_slots->plug_evt;
        break;
    case ACPI_PHP_IO_ADDR + 1:
        val = hotplug_slots->plug_slot;
        break;
    default:
        num = addr - ACPI_PHP_IO_ADDR - 2;
        val = hotplug_slots->status[num];
    }

    fprintf(logfile, "ACPI PCI hotplug: read addr=0x%x, val=0x%x.\n",
            addr, val);

    return val;
}

static void acpi_php_writeb(void *opaque, uint32_t addr, uint32_t val)
{
    PHPSlots *hotplug_slots = opaque;
    int slot;

    fprintf(logfile, "ACPI PCI hotplug: write addr=0x%x, val=0x%x.\n",
            addr, val);

    switch (addr)
    {
    case ACPI_PHP_IO_ADDR:
    case ACPI_PHP_IO_ADDR + 1:
        break;
    default:
        slot = addr - ACPI_PHP_IO_ADDR - 2;
        if ( val == 0x1 ) { /* Eject command */
            /* make _STA of the slot 0 */
            hotplug_slots->status[slot] = 0;

            /* clear the hotplug event */
            hotplug_slots->plug_evt = 0;
            hotplug_slots->plug_slot = 0;

            /* power off the slot */
            power_off_php_slot(slot);

            /* signal the CP ACPI hot remove done. */
            xenstore_record_dm_state("pci-removed");
        }
    }
}

static void pcislots_save(QEMUFile* f, void* opaque)
{
    PHPSlots *hotplug_slots = opaque;
    int i;
    for ( i = 0; i < NR_PCI_DEV; i++ ) {
        qemu_put_8s( f, &hotplug_slots->status[i]);
    }
    qemu_put_8s(f, &hotplug_slots->plug_evt);
    qemu_put_8s(f, &hotplug_slots->plug_slot);
}

static int pcislots_load(QEMUFile* f, void* opaque, int version_id)
{
    PHPSlots *hotplug_slots = opaque;
    int i;
    if (version_id != 1)
        return -EINVAL;
    for ( i = 0; i < NR_PCI_DEV; i++ ) {
        qemu_get_8s( f, &hotplug_slots->status[i]);
    }
    qemu_get_8s(f, &hotplug_slots->plug_evt);
    qemu_get_8s(f, &hotplug_slots->plug_slot);
    return 0;
}

static void php_slots_init(void)
{
    int i;
    memset(&php_slots, 0, sizeof(PHPSlots));

    /* update the pci slot status */
    for ( i = 0; i < NR_PCI_DEV; i++ ) {
        if ( test_pci_slot(i) )
            php_slots.status[i] = 0xf;
    }

    /* ACPI PCI hotplug controller */
    register_ioport_read(ACPI_PHP_IO_ADDR, NR_PCI_DEV + 2, 1,
                         acpi_php_readb, &php_slots);
    register_ioport_write(ACPI_PHP_IO_ADDR, NR_PCI_DEV + 2, 1,
                          acpi_php_writeb, &php_slots);
    register_savevm("pcislots", 0, 1, pcislots_save, pcislots_load,
                    &php_slots);
}

/* GPEx_STS occupy 1st half of the block, while GPEx_EN 2nd half */
static uint32_t gpe_sts_read(void *opaque, uint32_t addr)
{
    GPEState *s = opaque;

    return s->gpe0_sts[addr - ACPI_GPE0_BLK_ADDRESS];
}

/* write 1 to clear specific GPE bits */
static void gpe_sts_write(void *opaque, uint32_t addr, uint32_t val)
{
    GPEState *s = opaque;
    int hotplugged = 0;

    fprintf(logfile, "gpe_sts_write: addr=0x%x, val=0x%x.\n", addr, val);

    hotplugged = test_bit(&s->gpe0_sts[0], ACPI_PHP_GPE_BIT);
    s->gpe0_sts[addr - ACPI_GPE0_BLK_ADDRESS] &= ~val;
    if ( s->sci_asserted &&
         hotplugged &&
         !test_bit(&s->gpe0_sts[0], ACPI_PHP_GPE_BIT)) {
        fprintf(logfile, "Clear the GPE0_STS bit for ACPI hotplug & deassert the IRQ.\n");
        qemu_irq_lower(sci_irq);
    }

}

static uint32_t gpe_en_read(void *opaque, uint32_t addr)
{
    GPEState *s = opaque;

    return s->gpe0_en[addr - (ACPI_GPE0_BLK_ADDRESS + ACPI_GPE0_BLK_LEN / 2)];
}

/* write 0 to clear en bit */
static void gpe_en_write(void *opaque, uint32_t addr, uint32_t val)
{
    GPEState *s = opaque;
    int reg_count;

    fprintf(logfile, "gpe_en_write: addr=0x%x, val=0x%x.\n", addr, val);
    reg_count = addr - (ACPI_GPE0_BLK_ADDRESS + ACPI_GPE0_BLK_LEN / 2);
    s->gpe0_en[reg_count] = val;
    /* If disable GPE bit right after generating SCI on it, 
     * need deassert the intr to avoid redundant intrs
     */
    if ( s->sci_asserted &&
         reg_count == (ACPI_PHP_GPE_BIT / 8) &&
         !(val & (1 << (ACPI_PHP_GPE_BIT % 8))) ) {
        fprintf(logfile, "deassert due to disable GPE bit.\n");
        s->sci_asserted = 0;
        qemu_irq_lower(sci_irq);
    }

}

static void gpe_save(QEMUFile* f, void* opaque)
{
    GPEState *s = (GPEState*)opaque;
    int i;

    for ( i = 0; i < ACPI_GPE0_BLK_LEN / 2; i++ ) {
        qemu_put_8s(f, &s->gpe0_sts[i]);
        qemu_put_8s(f, &s->gpe0_en[i]);
    }

    qemu_put_8s(f, &s->sci_asserted);
    if ( s->sci_asserted ) {
        fprintf(logfile, "gpe_save with sci asserted!\n");
    }
}

static int gpe_load(QEMUFile* f, void* opaque, int version_id)
{
    GPEState *s = (GPEState*)opaque;
    int i;
    if (version_id != 1)
        return -EINVAL;

    for ( i = 0; i < ACPI_GPE0_BLK_LEN / 2; i++ ) {
        qemu_get_8s(f, &s->gpe0_sts[i]);
        qemu_get_8s(f, &s->gpe0_en[i]);
    }

    qemu_get_8s(f, &s->sci_asserted);
    return 0;
}

static void gpe_acpi_init(void)
{
    GPEState *s = &gpe_state;
    memset(s, 0, sizeof(GPEState));

    register_ioport_read(ACPI_GPE0_BLK_ADDRESS,
                         ACPI_GPE0_BLK_LEN / 2,
                         1,
                         gpe_sts_read,
                         s);
    register_ioport_read(ACPI_GPE0_BLK_ADDRESS + ACPI_GPE0_BLK_LEN / 2,
                         ACPI_GPE0_BLK_LEN / 2,
                         1,
                         gpe_en_read,
                         s);

    register_ioport_write(ACPI_GPE0_BLK_ADDRESS,
                          ACPI_GPE0_BLK_LEN / 2,
                          1,
                          gpe_sts_write,
                          s);
    register_ioport_write(ACPI_GPE0_BLK_ADDRESS + ACPI_GPE0_BLK_LEN / 2,
                          ACPI_GPE0_BLK_LEN / 2,
                          1,
                          gpe_en_write,
                          s);

    register_savevm("gpe", 0, 1, gpe_save, gpe_load, s);
}

static void acpi_sci_intr(GPEState *s)
{
    if ( !test_bit(&s->gpe0_sts[0], ACPI_PHP_GPE_BIT) &&
         test_bit(&s->gpe0_en[0], ACPI_PHP_GPE_BIT) ) {

        set_bit(&s->gpe0_sts[0], ACPI_PHP_GPE_BIT);
        s->sci_asserted = 1;
        qemu_irq_raise(sci_irq);
        fprintf(logfile, "generate a sci for PHP.\n");
    }
}

void acpi_php_del(int slot)
{
    GPEState *s = &gpe_state;

    if ( test_pci_slot(slot) < 0 ) {
        fprintf(logfile, "hot remove: pci slot %d "
                "is not used by a hotplug device.\n", slot);

        return;
    }

    /* update the php controller status */
    php_slots.plug_evt = PHP_EVT_REMOVE;
    php_slots.plug_slot = slot;

    /* generate a SCI interrupt */
    acpi_sci_intr(s);
}

void acpi_php_add(int slot)
{
    GPEState *s = &gpe_state;
    char ret_str[30];

    if ( slot < 0 ) {
        fprintf(logfile, "hot add pci slot %d exceed.\n", slot);

        if ( slot == -1 )
            sprintf(ret_str, "no free hotplug slots");
        else if ( slot == -2 )
            sprintf(ret_str, "wrong bdf or vslot");

        if ( strlen(ret_str) > 0 )
            xenstore_record_dm("parameter", ret_str);

        return;
    }

    /* update the php controller status */
    php_slots.plug_evt = PHP_EVT_ADD;
    php_slots.plug_slot = slot;

    /* update the slot status as present */
    php_slots.status[slot] = 0xf;

    /* power on the slot */
    power_on_php_slot(slot);

    /* tell Control panel which slot for the new pass-throgh dev */
    sprintf(ret_str, "0x%02x", slot);
    xenstore_record_dm("parameter", ret_str);

    /* signal the CP ACPI hot insert done */
    xenstore_record_dm_state("pci-inserted");

    /* generate a SCI interrupt */
    acpi_sci_intr(s);
}

#endif /* CONFIG_PASSTHROUGH */

/* PIIX4 acpi pci configuration space, func 2 */
i2c_bus *piix4_pm_init(PCIBus *bus, int devfn, uint32_t smb_io_base,
                       qemu_irq sci_irq_spec)
{
    PCIAcpiState *d;
    uint8_t *pci_conf;

    sci_irq = sci_irq_spec;

    /* we ignore smb_io_base as we don't give HVM guests an emulated smbus */

    /* register a function 2 of PIIX4 */
    d = (PCIAcpiState *)pci_register_device(
        bus, "PIIX4 ACPI", sizeof(PCIAcpiState),
        devfn, NULL, NULL);

    pci_conf = d->dev.config;
    pci_conf[0x00] = 0x86;  /* Intel */
    pci_conf[0x01] = 0x80;
    pci_conf[0x02] = 0x13;
    pci_conf[0x03] = 0x71;
    pci_conf[0x08] = 0x01;  /* B0 stepping */
    pci_conf[0x09] = 0x00;  /* base class */
    pci_conf[0x0a] = 0x80;  /* Sub class */
    pci_conf[0x0b] = 0x06;
    pci_conf[0x0e] = 0x00;
    pci_conf[0x3d] = 0x01;  /* Hardwired to PIRQA is used */


    /* PMBA POWER MANAGEMENT BASE ADDRESS, hardcoded to 0x1f40 
     * to make shutdown work for IPF, due to IPF Guest Firmware 
     * will enumerate pci devices. 
     *
     * TODO:  if Guest Firmware or Guest OS will change this PMBA,
     * More logic will be added.
     */
    pci_conf[0x40] = 0x41; /* Special device-specific BAR at 0x40 */
    pci_conf[0x41] = 0x1f;
    pci_conf[0x42] = 0x00;
    pci_conf[0x43] = 0x00;
    d->pm1_control = SCI_EN;

    acpi_map((PCIDevice *)d, 0, 0x1f40, 0x10, PCI_ADDRESS_SPACE_IO);

#ifdef CONFIG_PASSTHROUGH
    gpe_acpi_init();
    php_slots_init();
    register_ioport_write(ACPI_DBG_IO_ADDR, 4, 4, acpi_dbg_writel, d);
#endif

    register_savevm("piix4acpi", 0, 1, piix4acpi_save, piix4acpi_load, d);

    return NULL;
}

void qemu_system_hot_add_init() { }
void qemu_system_device_hot_add(int bus, int slot, int state) {
    fputs("qemu-upstream PCI hotplug not supported in qemu-dm\n",stderr);
    exit(-1);
}

void i440fx_init_memory_mappings(PCIDevice *d) {
    /* our implementation doesn't need this */
}
