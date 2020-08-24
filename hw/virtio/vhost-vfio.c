/*
 * Virtio VFIO device
 *
 * Copyright (c) 2020 Semihalf
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
#include "qapi/error.h"
#include "qemu/error-report.h"
#include "hw/qdev-properties.h"
#include "hw/vfio/vfio-platform.h"
#include "hw/virtio/vhost.h"
#include "hw/virtio/virtio-bus.h"

#include <linux/vhost.h>

#define TYPE_VHOST_VFIO_PLATFORM "vhost-vfio-platform"
#define VHOST_VFIO_PLAT(obj) \
        OBJECT_CHECK(VhostVFIOPlat, (obj), TYPE_VHOST_VFIO_PLATFORM)

#define VHOST_VFIO_PLAT_DEFAULT_QUEUE_SIZE 256

enum {
    VHOST_VFIO_PLAT_VQ_REQ = 0,
    VHOST_VFIO_PLAT_VQ_EVT = 1,
    VHOST_VFIO_PLAT_VQ_MAX = 2,
};

typedef struct VhostVFIOPlat {
    VirtIODevice parent_obj;
    VirtQueue *req_vq;
    VirtQueue *event_vq;
    uint32_t host_features;
    struct vhost_virtqueue vqs[VHOST_VFIO_PLAT_VQ_MAX];
    struct vhost_dev dev;
    MemoryRegion iomem;
    DeviceState *vfio_dev;
    uint32_t device_id;
    char *name;
    uint8_t index;
} VhostVFIOPlat;


static uint64_t vhost_vfio_plat_get_features(VirtIODevice *vdev,
                                             uint64_t requested_features,
                                             Error **errp)
{
    return requested_features;
}

static int vhost_vfio_set_running(VhostVFIOPlat *vfio, int start)
{
    const VhostOps *vhost_ops = vfio->dev.vhost_ops;
    int ret;

    if (!vhost_ops->vhost_vfio_set_running) {
        return -ENOSYS;
    }

    ret = vhost_ops->vhost_vfio_set_running(&vfio->dev, start);
    if (ret < 0) {
        return -errno;
    }
    return 0;
}

static int vhost_vfio_plat_start(VirtIODevice *vdev)
{
    VhostVFIOPlat *s = VHOST_VFIO_PLAT(vdev);
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

    ret = vhost_vfio_set_running(s, 1);
    if (ret < 0) {
        error_report("Error starting vhost vfio: %d", -ret);
        goto err_dev_start;
    }

    /* guest_notifier_mask/pending not used yet, so just unmask
     * everything here.  virtio-pci will do the right thing by
     * enabling/disabling irqfd.
     */
    for (i = 0; i < s->dev.nvqs; i++) {
        vhost_virtqueue_mask(&s->dev, vdev, i, false);
    }

    return ret;

err_dev_start:
    vhost_dev_stop(&s->dev, vdev);
err_guest_notifiers:
    k->set_guest_notifiers(qbus->parent, s->dev.nvqs, false);
err_host_notifiers:
    vhost_dev_disable_notifiers(&s->dev, vdev);
    return ret;
}

static void vhost_vfio_plat_stop(VirtIODevice *vdev)
{
    VhostVFIOPlat *s = VHOST_VFIO_PLAT(vdev);
    BusState *qbus = BUS(qdev_get_parent_bus(DEVICE(vdev)));
    VirtioBusClass *k = VIRTIO_BUS_GET_CLASS(qbus);
    int ret = 0;

    if (!k->set_guest_notifiers) {
        return;
    }

    ret = vhost_vfio_set_running(s, 0);
    if (ret < 0) {
        error_report("vhost vfio set running failed: %d", ret);
        return;
    }

    vhost_dev_stop(&s->dev, vdev);

    ret = k->set_guest_notifiers(qbus->parent, s->dev.nvqs, false);
    if (ret < 0) {
            error_report("vhost guest notifier cleanup failed: %d", ret);
            return;
    }

    vhost_dev_disable_notifiers(&s->dev, vdev);
}

static void vhost_vfio_plat_set_status(VirtIODevice *vdev, uint8_t val)
{
    VhostVFIOPlat *s = VHOST_VFIO_PLAT(vdev);
    bool start = (val & VIRTIO_CONFIG_S_DRIVER_OK);

    if (s->dev.started == start) {
        return;
    }

    if (start) {
        int ret;

        ret = vhost_vfio_plat_start(vdev);
        if (ret < 0) {
            error_report("unable to start vhost-iommu: %s", strerror(-ret));
            exit(1);
        }
    } else {
        vhost_vfio_plat_stop(vdev);
    }
}

static void vhost_dummy_handle_comman(VirtIODevice *vdev, VirtQueue *vq)
{
    /* Do nothing */
}

static void vhost_vfio_guest_notifier_mask(VirtIODevice *vdev, int idx,
                                            bool mask)
{
	VhostVFIOPlat *s = VHOST_VFIO_PLAT(vdev);

    vhost_virtqueue_mask(&s->dev, vdev, idx, mask);
}

static bool vhost_vfio_guest_notifier_pending(VirtIODevice *vdev, int idx)
{
	VhostVFIOPlat *s = VHOST_VFIO_PLAT(vdev);

    return vhost_virtqueue_pending(&s->dev, idx);
}

static int vhost_dev_set_vfio_fd(VhostVFIOPlat *vfio,
                                 struct vhost_vfio_dev_info *info)
{
    const VhostOps *vhost_ops = vfio->dev.vhost_ops;
    int ret;

    if (!vhost_ops->vhost_vfio_set_fd) {
        return -ENOSYS;
    }

    ret = vhost_ops->vhost_vfio_set_fd(&vfio->dev, info);
    if (ret < 0) {
        return -errno;
    }
    return 0;
}

static void vhost_vfio_plat_device_realize(DeviceState *dev, Error **errp)
{
    VirtIODevice *vdev = VIRTIO_DEVICE(dev);
    VhostVFIOPlat *s = VHOST_VFIO_PLAT(dev);
    struct vhost_vfio_dev_info info;
    VFIOPlatformDevice *vfio_dev;
    int ret, vhostfd = -1;

    if (!s->device_id) {
        error_setg(errp, "vhost-pipe: dev type undefined: %s",
                   strerror(-EINVAL));
        return;
    }

    vhostfd = open("/dev/vhost-pipe", O_RDWR);
    if (vhostfd < 0) {
        error_setg(errp, "vhost-pipe: open vhost char device failed: %s",
                   strerror(errno));
        return;
    }

    virtio_init(vdev, "vhost-pipe", s->device_id, 0);
    s->req_vq = virtio_add_queue(vdev, VHOST_VFIO_PLAT_DEFAULT_QUEUE_SIZE,
                                 vhost_dummy_handle_comman);
    s->event_vq = virtio_add_queue(vdev, VHOST_VFIO_PLAT_DEFAULT_QUEUE_SIZE, NULL);

    s->dev.nvqs = VHOST_VFIO_PLAT_VQ_MAX;
    s->dev.vqs = s->vqs;
    s->dev.vq_index = 0;
    s->dev.backend_features = 0;
    ret = vhost_dev_init(&s->dev, (void *)(uintptr_t)vhostfd,
                         VHOST_BACKEND_TYPE_KERNEL, 0);
    if (ret < 0) {
        error_setg(errp, "vhost-pipe: vhost initialization failed: %s",
                   strerror(-ret));
        goto err_virtio;
    }

    /* Combine vhost dev with VFIO device */
    vfio_dev = VFIO_PLATFORM_DEVICE(s->vfio_dev);
    info.vfio_consumer_fd = vfio_dev->vbasedev.fd;
    info.vhost_dev_index = s->index;
    info.vhost_dev_type = s->device_id;
    ret = vhost_dev_set_vfio_fd(s, &info);
    if (ret < 0) {
        error_setg_errno(errp, -ret, "vhost-pipe: unable to set VFIO fd");
        goto err_vhost_dev;
    }

    return;

err_vhost_dev:
    vhost_dev_cleanup(&s->dev);
    /* vhost_dev_cleanup() closes the vhostfd passed to vhost_dev_init() */
    vhostfd = -1;
err_virtio:
    virtio_delete_queue(s->req_vq);
    virtio_delete_queue(s->event_vq);
    virtio_cleanup(vdev);
    if (vhostfd >= 0) {
        close(vhostfd);
    }
    return;
}

static void vhost_vfio_plat_device_unrealize(DeviceState *dev, Error **errp)
{
    VirtIODevice *vdev = VIRTIO_DEVICE(dev);
    VhostVFIOPlat *s = VHOST_VFIO_PLAT(dev);

    /* This will stop vhost backend if appropriate. */
    vhost_vfio_plat_set_status(vdev, 0);

    vhost_dev_cleanup(&s->dev);
    virtio_delete_queue(s->req_vq);
    virtio_delete_queue(s->event_vq);
    virtio_cleanup(vdev);
}

static Property vhost_iommu_properties[] = {
    DEFINE_PROP_LINK("dev-client", VhostVFIOPlat, vfio_dev, "vfio-platform",
                     DeviceState *),
    DEFINE_PROP_STRING("name", VhostVFIOPlat, name),
    DEFINE_PROP_UINT8("index", VhostVFIOPlat, index, 0),
    DEFINE_PROP_UINT32("device_id", VhostVFIOPlat, device_id, 0),
    DEFINE_PROP_END_OF_LIST(),
};

static void vhost_iommu_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    VirtioDeviceClass *vdc = VIRTIO_DEVICE_CLASS(klass);

    device_class_set_props(dc, vhost_iommu_properties);
    set_bit(DEVICE_CATEGORY_MISC, dc->categories);
    vdc->realize = vhost_vfio_plat_device_realize;
    vdc->unrealize = vhost_vfio_plat_device_unrealize;
    vdc->get_features = vhost_vfio_plat_get_features;
    vdc->set_status = vhost_vfio_plat_set_status;
    vdc->guest_notifier_mask = vhost_vfio_guest_notifier_mask;
    vdc->guest_notifier_pending = vhost_vfio_guest_notifier_pending;
}

static const TypeInfo vhost_vfio_plat_info = {
    .name = TYPE_VHOST_VFIO_PLATFORM,
    .parent = TYPE_VIRTIO_DEVICE,
    .instance_size = sizeof(VhostVFIOPlat),
    .class_init = vhost_iommu_class_init,
};

static void vhost_vfio_plat_register_types(void)
{
    type_register_static(&vhost_vfio_plat_info);
}

type_init(vhost_vfio_plat_register_types)
