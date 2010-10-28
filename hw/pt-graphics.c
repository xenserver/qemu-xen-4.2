/*
 * graphics passthrough
 */

#include "pass-through.h"
#include "pci/header.h"
#include "pci/pci.h"

#include <unistd.h>
#include <sys/ioctl.h>
#include <assert.h>

extern int gfx_passthru;
extern int igd_passthru;

static int pch_map_irq(PCIDevice *pci_dev, int irq_num)
{
    PT_LOG("pch_map_irq called\n");
    return irq_num;
}

void intel_pch_init(PCIBus *bus)
{
    uint16_t vid, did;
    uint8_t  rid;

    if ( !gfx_passthru )
        return;

    vid = pt_pci_host_read(0, 0x1f, 0, 0, 2);
    did = pt_pci_host_read(0, 0x1f, 0, 2, 2);
    rid = pt_pci_host_read(0, 0x1f, 0, 8, 1);

    if ( vid == 0x8086 ) 
        pci_bridge_init(bus, PCI_DEVFN(0x1f, 0), vid, did, rid,
                        pch_map_irq, "intel_bridge_1f");
}

void igd_pci_write(PCIDevice *pci_dev, uint32_t config_addr, uint32_t val, int len)
{
    assert(pci_dev->devfn == 0x00);
    if ( !igd_passthru ) {
        pci_default_write_config(pci_dev, config_addr, val, len);
        return;
    }

    switch (config_addr)
    {
        case 0x58:        // PAVPC Offset
            pt_pci_host_write(0, 0, 0, config_addr, val, len);
            PT_LOG("pci_config_write: %x:%x.%x: addr=%x len=%x val=%x\n",
                   pci_bus_num(pci_dev->bus), PCI_SLOT(pci_dev->devfn),
                   PCI_FUNC(pci_dev->devfn), config_addr, len, val);
            break;
        default:
            pci_default_write_config(pci_dev, config_addr, val, len);
    }
}

uint32_t igd_pci_read(PCIDevice *pci_dev, uint32_t config_addr, int len)
{
    uint32_t val;

    assert(pci_dev->devfn == 0x00);
    if ( !igd_passthru ) {
        return pci_default_read_config(pci_dev, config_addr, len);
    }

    switch (config_addr)
    {
        case 0x00:        /* vendor id */
        case 0x02:        /* device id */
        case 0x52:        /* processor graphics control register */
        case 0xa0:        /* top of memory */
        case 0xb0:        /* ILK: BSM: should read from dev 2 offset 0x5c */
        case 0x58:        /* SNB: PAVPC Offset */
        case 0xa4:        /* SNB: graphics base of stolen memory */
        case 0xa8:        /* SNB: base of GTT stolen memory */
            val = pt_pci_host_read(0, PCI_SLOT(pci_dev->devfn),
                                   0, config_addr, len);
            PT_LOG("pci_config_read: %x:%x.%x: addr=%x len=%x val=%x\n",
                   pci_bus_num(pci_dev->bus), PCI_SLOT(pci_dev->devfn),
                   PCI_FUNC(pci_dev->devfn), config_addr, len, val);
            break;
        default:
            val = pci_default_read_config(pci_dev, config_addr, len);
    }
    return val;
}

/*
 * register VGA resources for the domain with assigned gfx
 */
int register_vga_regions(struct pt_dev *real_device)
{
    u32 vendor_id, igd_opregion;
    int ret = 0;

    if ( !gfx_passthru || real_device->pci_dev->device_class != 0x0300 )
        return ret;

    ret |= xc_domain_ioport_mapping(xc_handle, domid, 0x3B0,
            0x3B0, 0xC, DPCI_ADD_MAPPING);

    ret |= xc_domain_ioport_mapping(xc_handle, domid, 0x3C0,
            0x3C0, 0x20, DPCI_ADD_MAPPING);

    ret |= xc_domain_memory_mapping(xc_handle, domid,
            0xa0000 >> XC_PAGE_SHIFT,
            0xa0000 >> XC_PAGE_SHIFT,
            0x20,
            DPCI_ADD_MAPPING);

    /* 1:1 map ASL Storage register value */
    vendor_id = pt_pci_host_read(0, 2, 0, 0, 2);
    igd_opregion = pt_pci_host_read(0, 2, 0, 0xfc, 4);
    if ( (vendor_id == 0x8086) && igd_opregion )
    {
        ret |= xc_domain_memory_mapping(xc_handle, domid,
                igd_opregion >> XC_PAGE_SHIFT,
                igd_opregion >> XC_PAGE_SHIFT,
                2,
                DPCI_ADD_MAPPING);
        PT_LOG("register_vga: igd_opregion = %x\n", igd_opregion);
    }

    if ( ret != 0 )
        PT_LOG("VGA region mapping failed\n");

    return ret;
}

/*
 * unregister VGA resources for the domain with assigned gfx
 */
int unregister_vga_regions(struct pt_dev *real_device)
{
    u32 vendor_id, igd_opregion;
    int ret = 0;

    if ( !gfx_passthru || real_device->pci_dev->device_class != 0x0300 )
        return ret;

    ret |= xc_domain_ioport_mapping(xc_handle, domid, 0x3B0,
            0x3B0, 0xC, DPCI_REMOVE_MAPPING);

    ret |= xc_domain_ioport_mapping(xc_handle, domid, 0x3C0,
            0x3C0, 0x20, DPCI_REMOVE_MAPPING);

    ret |= xc_domain_memory_mapping(xc_handle, domid,
            0xa0000 >> XC_PAGE_SHIFT,
            0xa0000 >> XC_PAGE_SHIFT,
            20,
            DPCI_REMOVE_MAPPING);

    vendor_id = pt_pci_host_read(0, 2, 0, 0, 2);
    igd_opregion = pt_pci_host_read(0, 2, 0, 0xfc, 4);
    if ( (vendor_id == 0x8086) && igd_opregion )
    {
        ret |= xc_domain_memory_mapping(xc_handle, domid,
                igd_opregion >> XC_PAGE_SHIFT,
                igd_opregion >> XC_PAGE_SHIFT,
                2,
                DPCI_REMOVE_MAPPING);
    }

    if ( ret != 0 )
        PT_LOG("VGA region unmapping failed\n");

    return ret;
}

static int get_vgabios(unsigned char *buf)
{
    int fd;
    uint32_t bios_size = 0;
    uint32_t start = 0xC0000;
    uint16_t magic = 0;

    if ( (fd = open("/dev/mem", O_RDONLY)) < 0 )
    {
        PT_LOG("Error: Can't open /dev/mem: %s\n", strerror(errno));
        return 0;
    }

    /*
     * Check if it a real bios extension.
     * The magic number is 0xAA55.
     */
    if ( start != lseek(fd, start, SEEK_SET) )
        goto out;
    if ( read(fd, &magic, 2) != 2 )
        goto out;
    if ( magic != 0xAA55 )
        goto out;

    /* Find the size of the rom extension */
    if ( start != lseek(fd, start, SEEK_SET) )
        goto out;
    if ( lseek(fd, 2, SEEK_CUR) != (start + 2) )
        goto out;
    if ( read(fd, &bios_size, 1) != 1 )
        goto out;

    /* This size is in 512 bytes */
    bios_size *= 512;

    /*
     * Set the file to the begining of the rombios,
     * to start the copy.
     */
    if ( start != lseek(fd, start, SEEK_SET) )
        goto out;

    if ( bios_size != read(fd, buf, bios_size))
        bios_size = 0;

out:
    close(fd);
    return bios_size;
}

int setup_vga_pt(struct pt_dev *real_device)
{
    unsigned char *bios = NULL;
    int bios_size = 0;
    char *c = NULL;
    char checksum = 0;
    int rc = 0;

    if ( !gfx_passthru || real_device->pci_dev->device_class != 0x0300 )
        return rc;

    /* Allocated 64K for the vga bios */
    if ( !(bios = malloc(64 * 1024)) )
        return -1;

    bios_size = get_vgabios(bios);
    if ( bios_size == 0 || bios_size > 64 * 1024)
    {
        PT_LOG("vga bios size (0x%x) is invalid!\n", bios_size);
        rc = -1;
        goto out;
    }

    /* Adjust the bios checksum */
    for ( c = (char*)bios; c < ((char*)bios + bios_size); c++ )
        checksum += *c;
    if ( checksum )
    {
        bios[bios_size - 1] -= checksum;
        PT_LOG("vga bios checksum is adjusted!\n");
    }

    cpu_physical_memory_rw(0xc0000, bios, bios_size, 1);
out:
    free(bios);
    return rc;
}
