/*
 * Copyright (C) 2016 Imagination Technologies
 * Author: Paul Burton <paul.burton@imgtec.com>
 *
 * pcibios_align_resource taken from arch/arm/kernel/bios32.c.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#include <linux/pci.h>

/*
 * We need to avoid collisions with `mirrored' VGA ports
 * and other strange ISA hardware, so we always want the
 * addresses to be allocated in the 0x000-0x0ff region
 * modulo 0x400.
 *
 * Why? Because some silly external IO cards only decode
 * the low 10 bits of the IO address. The 0x00-0xff region
 * is reserved for motherboard devices that decode all 16
 * bits, so it's ok to allocate at, say, 0x2800-0x28ff,
 * but we want to try to avoid allocating at 0x2900-0x2bff
 * which might have be mirrored at 0x0100-0x03ff..
 */
resource_size_t pcibios_align_resource(void *data, const struct resource *res,
				resource_size_t size, resource_size_t align)
{
	struct pci_dev *dev = data;
	resource_size_t start = res->start;
	struct pci_host_bridge *host_bridge;

	if (res->flags & IORESOURCE_IO && start & 0x300)
		start = (start + 0x3ff) & ~0x3ff;

	start = (start + align - 1) & ~(align - 1);

	host_bridge = pci_find_host_bridge(dev->bus);

	if (host_bridge->align_resource)
		return host_bridge->align_resource(dev, res,
				start, size, align);

	return start;
}

void pcibios_fixup_bus(struct pci_bus *bus)
{
	pci_read_bridge_bases(bus);
}

int pci_remap_iospace(const struct resource *res, phys_addr_t phys_addr)
{
	pr_debug("pci_remap_iospace: %pa -> %pa\n", &res->start, &phys_addr);
	phys_addr -= res->start;
	if (phys_addr & ~PAGE_MASK)
		return -EINVAL;

	set_io_port_base((unsigned long)ioremap(phys_addr, resource_size(res))
			 + res->start);

	return 0;
}
