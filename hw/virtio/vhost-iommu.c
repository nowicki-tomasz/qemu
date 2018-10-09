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

#include "qemu/osdep.h"
#include "qemu/iov.h"
#include "qemu-common.h"
#include "hw/virtio/virtio.h"
#include "sysemu/kvm.h"
#include "qapi/error.h"
#include "qemu/error-report.h"
#include "hw/i386/pc.h"
#include "hw/arm/virt.h"
#include "trace.h"

#include "standard-headers/linux/virtio_ids.h"
#include <linux/vhost.h>
#include <linux/virtio_iommu.h>

#include "hw/virtio/vhost.h"
#include "hw/virtio/virtio-bus.h"
#include "hw/virtio/virtio-access.h"
#include "hw/virtio/virtio-iommu.h"
#include "hw/virtio/vhost-iommu.h"
#include "hw/pci/pci_bus.h"
#include "hw/pci/pci.h"

enum {
    VHOST_IOMMU_VQ_RQ = 0,
    VHOST_IOMMU_VQ_EVT_VQ = 1,
    VHOST_IOMMU_VQ_MAX = 2,
};

#define VIOMMU_DEFAULT_QUEUE_SIZE 256
#define VIOMMU_PROBE_SIZE 512

static AddressSpace *vhost_iommu_find_add_as(PCIBus *bus, void *opaque,
                                              int devfn)
{
    VhostIOMMU *s = opaque;
    IOMMUPciBus *sbus = g_hash_table_lookup(s->as_by_busptr, &bus);
    IOMMUDevice *sdev;

    if (!sbus) {
        sbus = g_malloc0(sizeof(IOMMUPciBus) +
                         sizeof(IOMMUDevice *) * IOMMU_PCI_DEVFN_MAX);
        sbus->bus = bus;
        g_hash_table_insert(s->as_by_busptr, bus, sbus);
    }

    sdev = sbus->pbdev[devfn];
    if (!sdev) {
        char *name = g_strdup_printf("%s-%d-%d",
                                     TYPE_VHOST_IOMMU_MEMORY_REGION,
                                     pci_bus_num(bus), devfn);
        sdev = sbus->pbdev[devfn] = g_malloc0(sizeof(IOMMUDevice));

        sdev->viommu = s;
        sdev->bus = bus;
        sdev->devfn = devfn;
        sdev->type = VHOST_IOMMU;

        memory_region_init_iommu(&sdev->iommu_mr, sizeof(sdev->iommu_mr),
                                 TYPE_VHOST_IOMMU_MEMORY_REGION,
                                 OBJECT(s), name,
                                 UINT64_MAX);
        address_space_init(&sdev->as,
                           MEMORY_REGION(&sdev->iommu_mr), TYPE_VHOST_IOMMU);
    }

    return &sdev->as;

}

static IOMMUTLBEntry vhost_iommu_translate(IOMMUMemoryRegion *mr, hwaddr addr,
                                           IOMMUAccessFlags flag,
                                           int iommu_idx)
{
    IOMMUDevice *sdev = container_of(mr, IOMMUDevice, iommu_mr);
    VhostIOMMU *s = sdev->viommu;
    struct vhost_dev *vhost = &s->dev;
    uint32_t devid = PCI_BUILD_BDF(pci_bus_num(sdev->bus), sdev->devfn);
    uint64_t xlat;
    int ret;

    IOMMUTLBEntry entry = {
        .target_as = &address_space_memory,
        .iova = addr,
        .translated_addr = addr,
        .addr_mask = (1 << ctz32(s->config.page_size_mask)) - 1,
        .perm = IOMMU_RW,
    };

    qemu_mutex_lock(&s->mutex);
    ret = vhost_backend_iommu_xlate(vhost, addr, flag, &xlat, devid);
    if (ret < 0) {
        error_report("vhost_backend_iommu_xlate failed for addr 0x%lx\n",
                     (long)addr);
        goto unlock;
    }

    entry.translated_addr = xlat;
unlock:
    qemu_mutex_unlock(&s->mutex);
    return entry;
}

static void vhost_iommu_get_config(VirtIODevice *vdev, uint8_t *config_data)
{
    VhostIOMMU *s = VHOST_IOMMU(vdev);

    memcpy(config_data, &s->config, sizeof(struct virtio_iommu_config));
}

static void vhost_iommu_set_config(VirtIODevice *vdev,
                                      const uint8_t *config_data)
{
}

static uint64_t vhost_iommu_get_features(VirtIODevice *vdev, uint64_t f,
                                            Error **errp)
{
    VhostIOMMU *s = VHOST_IOMMU(vdev);
    f |= s->host_features;
    virtio_add_feature(&f, VIRTIO_RING_F_EVENT_IDX);
    virtio_add_feature(&f, VIRTIO_RING_F_INDIRECT_DESC);
    virtio_add_feature(&f, VIRTIO_IOMMU_F_INPUT_RANGE);
    virtio_add_feature(&f, VIRTIO_IOMMU_F_MAP_UNMAP);
    virtio_add_feature(&f, VIRTIO_IOMMU_F_PROBE);
    virtio_add_feature(&f, VIRTIO_IOMMU_F_ATTACH_TABLE);
    return f;
}

static void vhost_iommu_set_features(VirtIODevice *vdev, uint64_t val)
{
}

static void vhost_iommu_device_reset(VirtIODevice *vdev)
{
}

static int vhost_iommu_start(VirtIODevice *vdev)
{
    VhostIOMMU *s = VHOST_IOMMU(vdev);
    BusState *qbus = BUS(qdev_get_parent_bus(DEVICE(vdev)));
    VirtioBusClass *k = VIRTIO_BUS_GET_CLASS(qbus);
    int ret, i;

    if (!k->set_guest_notifiers) {
        error_report("binding does not support guest notifiers");
        return -ENOSYS;
    }

    ret = vhost_dev_enable_notifiers(&s->dev, vdev);
    if (ret < 0) {
        return ret;
    }

    ret = k->set_guest_notifiers(qbus->parent, s->dev.nvqs, true);
    if (ret < 0) {
        error_report("Error binding guest notifier");
        goto err_host_notifiers;
    }

    s->dev.acked_features = vdev->guest_features;
    ret = vhost_dev_start(&s->dev, vdev);
    if (ret < 0) {
        error_report("Error start vhost dev");
        goto err_guest_notifiers;
    }

    /* guest_notifier_mask/pending not used yet, so just unmask
     * everything here.  virtio-pci will do the right thing by
     * enabling/disabling irqfd.
     */
    for (i = 0; i < s->dev.nvqs; i++) {
        vhost_virtqueue_mask(&s->dev, vdev, i, false);
    }

    return ret;

err_guest_notifiers:
    k->set_guest_notifiers(qbus->parent, s->dev.nvqs, false);
err_host_notifiers:
    vhost_dev_disable_notifiers(&s->dev, vdev);
    return ret;
}

static int vhost_iommu_sync_config(VhostIOMMU *s)
{
    const VhostOps *vhost_ops = s->dev.vhost_ops;
    struct vhost_iommu_config config;
    int ret;

    memset(&config, 0, sizeof(config));
    memcpy(&config.config, &s->config, sizeof(s->config));
    config.pgtf = s->pgtf;
    ret = vhost_ops->vhost_iommu_set_config(&s->dev, &config);
    if (ret < 0) {
        return -errno;
    }
    return 0;
}

static void vhost_iommu_stop(VirtIODevice *vdev)
{
    VhostIOMMU *s = VHOST_IOMMU(vdev);
    BusState *qbus = BUS(qdev_get_parent_bus(DEVICE(vdev)));
    VirtioBusClass *k = VIRTIO_BUS_GET_CLASS(qbus);
    int ret = 0;

    vhost_dev_stop(&s->dev, vdev);

    if (k->set_guest_notifiers) {
        ret = k->set_guest_notifiers(qbus->parent, s->dev.nvqs, false);
        if (ret < 0) {
                error_report("vhost guest notifier cleanup failed: %d", ret);
        }
    }
    assert(ret >= 0);

    vhost_dev_disable_notifiers(&s->dev, vdev);
}

static void vhost_iommu_set_status(VirtIODevice *vdev, uint8_t val)
{
    VhostIOMMU *s = VHOST_IOMMU(vdev);
    bool start = (val & VIRTIO_CONFIG_S_DRIVER_OK);

    if (s->dev.started == start) {
        return;
    }

    if (start) {
        int ret;

        ret = vhost_iommu_sync_config(s);
        if (ret < 0) {
            error_report("unable to set config vhost-iommu: %s", strerror(-ret));
                        exit(1);
        }

        ret = vhost_iommu_start(vdev);
        if (ret < 0) {
            error_report("unable to start vhost-iommu: %s", strerror(-ret));
            exit(1);
        }
    } else {
        vhost_iommu_stop(vdev);
    }
}

static int vhost_iommu_set_id(VhostIOMMU *s)
{
    const VhostOps *vhost_ops = s->dev.vhost_ops;
    uint32_t vhostfd;
    int ret;

    vhostfd = (uint32_t)(uintptr_t)s->dev.opaque;
    ret = vhost_ops->vhost_iommu_set_id(&s->dev, &vhostfd);
    if (ret < 0) {
        return -errno;
    }
    return 0;
}

static void vhost_dummy_handle_comman(VirtIODevice *vdev, VirtQueue *vq)
{
}

static void vhost_iommu_device_realize(DeviceState *dev, Error **errp)
{
    VirtIODevice *vdev = VIRTIO_DEVICE(dev);
    VhostIOMMU *s = VHOST_IOMMU(dev);
    int ret, vhostfd = -1;

    vhostfd = open("/dev/vhost-iommu", O_RDWR);
    if (vhostfd < 0) {
        error_setg(errp, "vhost-iommu: open vhost char device failed: %s",
                   strerror(errno));
        return;
    }

    virtio_init(vdev, "vhost-iommu", VIRTIO_ID_IOMMU,
                sizeof(struct virtio_iommu_config));

    s->req_vq = virtio_add_queue(vdev, VIOMMU_DEFAULT_QUEUE_SIZE,
                                 vhost_dummy_handle_comman);
    s->event_vq = virtio_add_queue(vdev, VIOMMU_DEFAULT_QUEUE_SIZE, NULL);

    s->config.page_size_mask = TARGET_PAGE_MASK;
    s->config.input_range.end = -1UL;
    s->config.probe_size = VIOMMU_PROBE_SIZE;

    qemu_mutex_init(&s->mutex);
    s->as_by_busptr = g_hash_table_new(NULL, NULL);
    if (s->primary_bus) {
        pci_setup_iommu(s->primary_bus, vhost_iommu_find_add_as, s);
    } else {
        error_setg(errp, "VHOST-IOMMU is not attached to any PCI bus!");
    }

    s->dev.nvqs = VHOST_IOMMU_VQ_MAX;
    s->dev.vqs = g_new(struct vhost_virtqueue, s->dev.nvqs);
    s->dev.vq_index = 0;
    s->dev.backend_features = 0;

    ret = vhost_dev_init(&s->dev, (void *)(uintptr_t)vhostfd,
                         VHOST_BACKEND_TYPE_KERNEL, 0);
    if (ret < 0) {
        error_setg(errp, "vhost-iommu: vhost initialization failed: %s",
                   strerror(-ret));
        goto out;
    }

    ret = vhost_iommu_set_id(s);
    if (ret < 0) {
        error_setg(errp, "vhost-iommu: unable to set ID vhost-iommu: %s",
                   strerror(-ret));
        goto out;
    }

    return;

out:
    g_free(s->dev.vqs);
    close(vhostfd);
}

static void vhost_iommu_device_unrealize(DeviceState *dev, Error **errp)
{
    VirtIODevice *vdev = VIRTIO_DEVICE(dev);
    VhostIOMMU *s = VHOST_IOMMU(dev);

    /* This will stop vhost backend if appropriate. */
    vhost_iommu_set_status(vdev, 0);

    vhost_dev_cleanup(&s->dev);
    g_free(s->dev.vqs);
    virtio_cleanup(vdev);
}

static bool vhost_iommu_get_msi_bypass(Object *obj, Error **errp)
{
    VhostIOMMU *s = VHOST_IOMMU(obj);

    return s->msi_bypass;
}

static void vhost_iommu_set_msi_bypass(Object *obj, bool value, Error **errp)
{
    VhostIOMMU *s = VHOST_IOMMU(obj);

    s->msi_bypass = value;
}

static const QEnumLookup IommuPgtfMap = {
    .array = (const char *const[]) {
        [IOMMU_PGTF_NONE] = "none",
        [IOMMU_PGTF_ARM_LPAE] = "arm-lpae",
    },
    .size = IOMMU_LAST
};

static int vhost_iommu_get_pt(Object *obj, Error **errp)
{
    VhostIOMMU *s = VHOST_IOMMU(obj);

    return s->pgtf;
}


static void vhost_iommu_set_pt(Object *obj, int value, Error **errp)
{
    VhostIOMMU *s = VHOST_IOMMU(obj);

    s->pgtf = value;
}

static void vhost_iommu_instance_init(Object *obj)
{
    VhostIOMMU *s = VHOST_IOMMU(obj);

    object_property_add_bool(obj, "msi_bypass", vhost_iommu_get_msi_bypass,
                             vhost_iommu_set_msi_bypass, NULL);
    object_property_set_description(obj, "msi_bypass",
                                    "Indicates whether msis are bypassed by "
                                    "the IOMMU. Default is YES",
                                    NULL);

    s->msi_bypass = true;

    object_property_add_enum(obj, "pt", "IommuPgtf", &IommuPgtfMap,
                             vhost_iommu_get_pt, vhost_iommu_set_pt, NULL);
    object_property_set_description(obj, "pt",
                                    "Indicates page descriptor format."
                                    "Default is map tree", NULL);
    s->pgtf = IOMMU_PGTF_NONE;
}

static int vhost_iommu_post_load_device(void *opaque, int version_id)
{
    return 0;
}

static const VMStateDescription vmstate_vhost_iommu_device = {
    .name = "vhost-iommu-device",
    .version_id = 1,
    .minimum_version_id = 1,
    .post_load = vhost_iommu_post_load_device,
    .fields = (VMStateField[]) {
        VMSTATE_END_OF_LIST()
    },
};

static const VMStateDescription vmstate_vhost_iommu = {
    .name = "vhost-iommu",
    .minimum_version_id = 1,
    .version_id = 1,
    .fields = (VMStateField[]) {
        VMSTATE_VIRTIO_DEVICE,
        VMSTATE_END_OF_LIST()
    },
};

static Property vhost_iommu_properties[] = {
    DEFINE_PROP_LINK("primary-bus", VhostIOMMU, primary_bus, "PCI", PCIBus *),
    DEFINE_PROP_END_OF_LIST(),
};

static void vhost_iommu_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    VirtioDeviceClass *vdc = VIRTIO_DEVICE_CLASS(klass);

    dc->props = vhost_iommu_properties;
    dc->vmsd = &vmstate_vhost_iommu;

    set_bit(DEVICE_CATEGORY_MISC, dc->categories);
    vdc->realize = vhost_iommu_device_realize;
    vdc->unrealize = vhost_iommu_device_unrealize;
    vdc->reset = vhost_iommu_device_reset;
    vdc->get_config = vhost_iommu_get_config;
    vdc->set_config = vhost_iommu_set_config;
    vdc->get_features = vhost_iommu_get_features;
    vdc->set_features = vhost_iommu_set_features;
    vdc->set_status = vhost_iommu_set_status;
    vdc->vmsd = &vmstate_vhost_iommu_device;
}

static void vhost_iommu_memory_region_class_init(ObjectClass *klass,
                                                  void *data)
{
    IOMMUMemoryRegionClass *imrc = IOMMU_MEMORY_REGION_CLASS(klass);

    imrc->translate = vhost_iommu_translate;
}

static const TypeInfo vhost_iommu_info = {
    .name = TYPE_VHOST_IOMMU,
    .parent = TYPE_VIRTIO_DEVICE,
    .instance_size = sizeof(VhostIOMMU),
    .instance_init = vhost_iommu_instance_init,
    .class_init = vhost_iommu_class_init,
};

static const TypeInfo vhost_iommu_memory_region_info = {
    .parent = TYPE_IOMMU_MEMORY_REGION,
    .name = TYPE_VHOST_IOMMU_MEMORY_REGION,
    .class_init = vhost_iommu_memory_region_class_init,
};


static void vhost_register_types(void)
{
    type_register_static(&vhost_iommu_info);
    type_register_static(&vhost_iommu_memory_region_info);
}

type_init(vhost_register_types)
