/*
 * vhost-iommu device
 *
 * Copyright (c) 2018 Semihalf
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2 or later, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#ifndef QEMU_VHOST_IOMMU_H
#define QEMU_VHOST_IOMMU_H

#include "standard-headers/linux/virtio_iommu.h"
#include "hw/virtio/vhost.h"
#include "hw/virtio/virtio.h"
#include "hw/pci/pci.h"

#define TYPE_VHOST_IOMMU "vhost-iommu-device"
#define VHOST_IOMMU(obj) \
        OBJECT_CHECK(VhostIOMMU, (obj), TYPE_VHOST_IOMMU)

#define TYPE_VHOST_IOMMU_MEMORY_REGION "vhost-iommu-memory-region"

typedef enum IommuPgtf IommuPgtf;

enum IommuPgtf {
	IOMMU_PGTF_NONE,
	IOMMU_PGTF_ARM_LPAE,

	IOMMU_LAST,
};

typedef struct VhostIOMMU {
    VirtIODevice parent_obj;
    VirtQueue *req_vq;
    VirtQueue *event_vq;
    struct virtio_iommu_config config;
    uint32_t host_features;
    GHashTable *as_by_busptr;
    PCIBus *primary_bus;
    QemuMutex mutex;
    struct vhost_dev dev;
    struct vhost_virtqueue vqs;
    bool msi_bypass;
    IommuPgtf pgtf;
} VhostIOMMU;

#endif
