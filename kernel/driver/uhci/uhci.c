/*
 * Universal Host Controller Interface driver
 *
 * This file is part of PanicOS.
 *
 * PanicOS is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * PanicOS is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with PanicOS.  If not, see <https://www.gnu.org/licenses/>.
 */

#include <common/x86.h>
#include <defs.h>
#include <driver/pci/pci.h>
#include <driver/usb/usb.h>
#include <memlayout.h>

#include "uhci-regs.h"

struct UHCIDevice {
    ioport_t iobase;
    uint32_t *frame_list;
};

static void uhci_intr(struct PCIDevice *pcidev) {
    // Acknowledge interrupts if needed
    uint16_t status = inw(((struct UHCIDevice*)pcidev->private)->iobase + UHCI_USBSTS);
    outw(((struct UHCIDevice*)pcidev->private)->iobase + UHCI_USBSTS, status);
}

static void uhci_port_reset(void *private, unsigned int port) {
    struct UHCIDevice *uhci = private;
    uint16_t reg = uhci->iobase + UHCI_PORTSC + port * 2;
    // Assert port reset
    outw(reg, inw(reg) | UHCI_PORTSC_PORT_RESET);
    mdelay(50);
    // Deassert reset
    outw(reg, inw(reg) & ~UHCI_PORTSC_PORT_RESET);
    // Clear change bits
    outw(reg, UHCI_PORTSC_CSC | UHCI_PORTSC_PEC | UHCI_PORTSC_PSC);
}

static enum USBPortStatus uhci_get_port_status(void *private, unsigned int port) {
    struct UHCIDevice *dev = private;
    uint16_t portstat = inw(dev->iobase + UHCI_PORTSC + port * 2);
    if (portstat & UHCI_PORTSC_CONNECT_STATUS) {
        return (portstat & UHCI_PORTSC_LOW_SPEED)
               ? USB_PORT_STATUS_CONNECT_LOW_SPEED
               : USB_PORT_STATUS_CONNECT_FULL_SPEED;
    }
    return USB_PORT_STATUS_NOT_CONNECT;
}

static void uhci_run(struct UHCIDevice *dev, int frnum) {
    // Load frame number
    outw(dev->iobase + UHCI_FRNUM, frnum);
    // Set Run/Stop bit
    outw(dev->iobase + UHCI_USBCMD,
         inw(dev->iobase + UHCI_USBCMD) | UHCI_USBCMD_RS);
}

static void uhci_stop(struct UHCIDevice *dev) {
    // Clear Run/Stop bit
    outw(dev->iobase + UHCI_USBCMD,
         inw(dev->iobase + UHCI_USBCMD) & ~UHCI_USBCMD_RS);
    // Wait until halted
    while (!(inw(dev->iobase + UHCI_USBSTS) & UHCI_USBSTS_HCH));
}

static enum USBTransferStatus uhci_transfer_packet(void *private,
                                                   unsigned int addr,
                                                   unsigned int endpoint,
                                                   const struct USBPacket *packets,
                                                   unsigned int num) {
    struct UHCIDevice *dev = private;
    // Reset frame list entries
    for (int i = 0; i < 1024; i++) {
        dev->frame_list[i] = 1;  // Terminate
    }

    // Allocate transfer descriptors (contiguous)
    struct UHCITransferDesc {
        uint32_t lnk;
        uint32_t sta;
        uint32_t maxlen;
        uint32_t bufptr;
    } PACKED;

    struct UHCITransferDesc *td = kalloc_aligned(sizeof(struct UHCITransferDesc) * num, PGSIZE);
    uint32_t td_phys = V2P(td);

    // Build TD chain
    for (unsigned int i = 0; i < num; i++) {
        // Link pointer
        if (i < num - 1) {
            td[i].lnk = (td_phys + sizeof(*td) * (i + 1)) & ~0x0F;
        } else {
            td[i].lnk = 1;  // terminate
        }
        // Status: active
        td[i].sta = UHCI_STATUS_ACTIVE;
        // Low-speed flag
        if (inw(dev->iobase + UHCI_PORTSC) & UHCI_PORTSC_LOW_SPEED) {
            td[i].sta |= UHCI_STATUS_LOWSPEED;
        }
        // Token: PID, addr, endpoint, maxlen, toggle
        uint32_t token = ((packets[i].maxlen - 1) << 21)
                       | (endpoint << 15)
                       | (addr << 8);
        switch (packets[i].type) {
        case USB_PACKET_IN:    token |= UHCI_TOKEN_PID_IN;    break;
        case USB_PACKET_OUT:   token |= UHCI_TOKEN_PID_OUT;   break;
        case USB_PACKET_SETUP: token |= UHCI_TOKEN_PID_SETUP; break;
        }
        if (packets[i].toggle) token |= UHCI_TOKEN_TOGGLE;
        td[i].maxlen = token;
        td[i].bufptr = V2P(packets[i].buffer);

        // Enqueue into frame list (same frame for simplicity)
        dev->frame_list[0] = td_phys + sizeof(*td) * i;
    }

    // Start transfer
    uhci_run(dev, 0);
    // Wait for completion (interrupt-driven would be better)
    while (inw(dev->iobase + UHCI_USBSTS) & UHCI_USBSTS_HCH);
    uhci_stop(dev);

    kfree(td);
    return USB_STATUS_OK;
}

const static struct USBHostControllerDriver uhci_usbhc_driver = {
    .reset_port      = uhci_port_reset,
    .get_port_status = uhci_get_port_status,
    .transfer_packet = uhci_transfer_packet,
};

static void uhci_controller_init(struct PCIDevice *pcidev) {
    pci_enable_bus_mastering(&pcidev->addr);
    pci_register_intr_handler(pcidev, uhci_intr);

    struct UHCIDevice *dev = kalloc();
    pcidev->private = dev;

    dev->iobase = pci_read_bar(&pcidev->addr, 4);
    cprintf("[uhci] IOBASE=%x\n", dev->iobase);

    // Reset controller
    outw(dev->iobase + UHCI_USBCMD, UHCI_USBCMD_HCRESET);
    udelay(10);
    outw(dev->iobase + UHCI_USBCMD, 0);
    udelay(10);

    // Disable interrupts
    outw(dev->iobase + UHCI_USBINTR, 0);

    // Allocate and init frame list (4 KB aligned)
    dev->frame_list = kalloc_aligned(4096, 4096);
    for (int i = 0; i < 1024; i++) {
        dev->frame_list[i] = 1;
    }
    outdw(dev->iobase + UHCI_FRBASEADD, V2P(dev->frame_list));

    // Set SOF timing
    outb(dev->iobase + UHCI_SOFMOD, 64);

    // Probe root ports
    for (int i = 0; i < 2; i++) {
        uint16_t ps = inw(dev->iobase + UHCI_PORTSC + i * 2);
        if (ps & UHCI_PORTSC_CONNECT_STATUS) {
            cprintf("[uhci] port %d %s-speed device\n",
                    i, (ps & UHCI_PORTSC_LOW_SPEED) ? "low" : "full");
        }
    }

    usb_register_host_controller(dev, "uhci", 2, &uhci_usbhc_driver);
}

struct PCIDriver uhci_pci_driver = {
    .name        = "uhci",
    .class_type  = 0x0c0300,
    .init        = uhci_controller_init,
};

void uhci_init(void) {
    pci_register_driver(&uhci_pci_driver);
}
