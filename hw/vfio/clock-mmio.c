/*
 * Copyright Semihalf Limited, 2020
 *
 * Authors:
 *  Tomasz Nowicki <tn@semihalf.com>
 *
 * This work is licensed under the terms of the GNU GPL, version 2.  See
 * the COPYING file in the top-level directory.
 */

#include "qemu/osdep.h"
#include <linux/vfio.h>
#include <sys/ioctl.h>

#include "hw/qdev-properties.h"
#include "hw/vfio/vfio-platform.h"
#include "migration/vmstate.h"
#include "qemu/error-report.h"
#include "qemu/log.h"

#define TYPE_CLOCK_MMIO "clock-mmio"

typedef struct ClockMMIODevice {
    SysBusDevice sbdev;
    MemoryRegion iomem;
    DeviceState *dev;
    char *name;
    uint8_t index;
} ClockMMIODevice;

typedef struct ClockMMIODeviceClass {
    /*< private >*/
    SysBusDeviceClass parent_class;
    /*< public >*/
} ClockMMIODeviceClass;

#define CLOCK_MMIO_DEVICE(obj) \
     OBJECT_CHECK(ClockMMIODevice, (obj), TYPE_CLOCK_MMIO)
#define CLOCK_MMIO_CLASS(klass) \
     OBJECT_CLASS_CHECK(ClockMMIODevice, (klass), TYPE_CLOCK_MMIO)
#define CLOCK_MMIO_GET_CLASS(obj) \
     OBJECT_GET_CLASS(ClockMMIODevice, (obj), TYPE_CLOCK_MMIO)

#define CLK_MMIO_RATE		    0x0
#define CLK_MMIO_FALGS		    0x8 // READONLY
#define CLK_MMIO_PREPARE	    0x10
#define CLK_MMIO_ENABLE         0x14

static MemTxResult clock_mmio_readll(ClockMMIODevice *clk_dev, hwaddr offset,
                                     uint64_t *data, MemTxAttrs attrs)
{
    VFIOPlatformDevice *vdev = VFIO_PLATFORM_DEVICE(clk_dev->dev);
    VFIODevice *vbasedev = &vdev->vbasedev;
    struct vfio_clk *clk;
    int argsz = sizeof(*clk);
    uint64_t *payload;
    int ret;

    switch (offset) {
    case CLK_MMIO_RATE:
        argsz += sizeof(*payload);
        clk = g_malloc0(argsz);

        clk->argsz = argsz;
        clk->flags = VFIO_CLK_GET_RATE;
        clk->index = clk_dev->index;
        payload = (uint64_t *)&clk->data; // rate
        break;
    case CLK_MMIO_FALGS:
        argsz += sizeof(*payload);
        clk = g_malloc0(argsz);

        clk->argsz = argsz;
        clk->flags = VFIO_CLK_GET_FLAGS;
        clk->index = clk_dev->index;
        payload = (uint64_t *)&clk->data; // flags
        break;
    default:
        *data = 0;
        qemu_log_mask(LOG_UNIMP,
                      "%s unhandled 64-bit access at 0x%"PRIx64"\n",
                      __func__, offset);
        return MEMTX_DECODE_ERROR;
    }

    ret = ioctl(vbasedev->fd, VFIO_DEVICE_CLK, clk);
    if (ret) {
        error_report("vfio_clk: Failed to set up device %s clock",
                     clk_dev->name);
        return MEMTX_ERROR;
    }

    *data = *payload;
    g_free(clk);
    return MEMTX_OK;
}

static MemTxResult clock_mmio_read(void *opaque, hwaddr offset, uint64_t *data,
                                  unsigned size, MemTxAttrs attrs)
{
    ClockMMIODevice *clk_dev = opaque;
    MemTxResult r;

    switch (size) {
    case 8:
        r = clock_mmio_readll(clk_dev, offset, data, attrs);
        break;
    default:
        r = MEMTX_ERROR;
        break;
    }

    return r;
}

static MemTxResult clock_mmio_writell(ClockMMIODevice *clk_dev, hwaddr offset,
                                      uint64_t data, MemTxAttrs attrs)
{
    VFIOPlatformDevice *vdev = VFIO_PLATFORM_DEVICE(clk_dev->dev);
    VFIODevice *vbasedev = &vdev->vbasedev;
    struct vfio_clk *clk;
    int argsz = sizeof(*clk);
    uint64_t *payload;
    int ret;

    switch (offset) {
    case CLK_MMIO_RATE:
        argsz += sizeof(*payload);
        clk = g_malloc0(argsz);

        clk->argsz = argsz;
        clk->flags = VFIO_CLK_SET_RATE;
        clk->index = clk_dev->index;
        payload = (uint64_t *)&clk->data;
        *payload = data;
        break;
    default:
        qemu_log_mask(LOG_UNIMP,
                      "%s unhandled 64-bit access at 0x%"PRIx64"\n",
                      __func__, offset);
        return MEMTX_DECODE_ERROR;
    }

    ret = ioctl(vbasedev->fd, VFIO_DEVICE_CLK, clk);
    if (ret) {
        error_report("vfio_clk: Failed to set up device %s clock",
                     clk_dev->name);
        return MEMTX_ERROR;
    }

    g_free(clk);
    return MEMTX_OK;
}

static MemTxResult clock_mmio_writel(ClockMMIODevice *clk_dev, hwaddr offset,
                                     uint32_t data, MemTxAttrs attrs)
{
    VFIOPlatformDevice *vdev = VFIO_PLATFORM_DEVICE(clk_dev->dev);
    VFIODevice *vbasedev = &vdev->vbasedev;
    struct vfio_clk *clk;
    int argsz = sizeof(*clk);
    uint32_t *payload;
    int ret;

    switch (offset) {
    case CLK_MMIO_PREPARE:
        argsz += sizeof(*payload);
        clk = g_malloc0(argsz);

        clk->argsz = argsz;
        clk->flags = data ? VFIO_CLK_PREPARE : VFIO_CLK_UNPREPARE;
        clk->index = clk_dev->index;
        payload = (uint32_t *)&clk->data;
        *payload = data;
        break;
    case CLK_MMIO_ENABLE:
        argsz += sizeof(*payload);
        clk = g_malloc0(argsz);

        clk->argsz = argsz;
        clk->flags = data ? VFIO_CLK_ENABLE : VFIO_CLK_DISABLE;
        clk->index = clk_dev->index;
        payload = (uint32_t *)&clk->data;
        *payload = data;
        break;
    default:
        qemu_log_mask(LOG_UNIMP,
                      "%s unhandled 32-bit access at 0x%"PRIx64"\n",
                      __func__, offset);
        return MEMTX_DECODE_ERROR;
    }

    ret = ioctl(vbasedev->fd, VFIO_DEVICE_CLK, clk);
    if (ret) {
        error_report("vfio_clk: Failed to set up device %s clock",
                     clk_dev->name);
        return MEMTX_ERROR;
    }

    g_free(clk);
    return MEMTX_OK;
}

static MemTxResult clock_mmio_write(void *opaque, hwaddr offset, uint64_t data,
                                   unsigned size, MemTxAttrs attrs)
{
    ClockMMIODevice *clk_dev = opaque;
    MemTxResult r;

    switch (size) {
    case 8:
        r = clock_mmio_writell(clk_dev, offset, data, attrs);
        break;
    case 4:
        r = clock_mmio_writel(clk_dev, offset, data, attrs);
        break;
    default:
        r = MEMTX_ERROR;
        break;
    }

    return r;
}

static const MemoryRegionOps clock_mmio_ops = {
    .read_with_attrs = clock_mmio_read,
    .write_with_attrs = clock_mmio_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = {
        .min_access_size = 4,
        .max_access_size = 8,
    },
    .impl = {
        .min_access_size = 4,
        .max_access_size = 8,
    },
};

static void clock_mmio_init(Object *obj)
{
	ClockMMIODevice *clk_dev = CLOCK_MMIO_DEVICE(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&clk_dev->iomem, obj, &clock_mmio_ops, clk_dev,
                          "clk_mmio", 0x100);
    sysbus_init_mmio(sbd, &clk_dev->iomem);
}

static const VMStateDescription clock_mmio_vmstate = {
    .name = TYPE_CLOCK_MMIO,
    .unmigratable = 1,
};

static Property clock_mmio_dev_properties[] = {
    DEFINE_PROP_LINK("dev-client", ClockMMIODevice, dev, "vfio-platform",
                     DeviceState *),
    DEFINE_PROP_STRING("name", ClockMMIODevice, name),
    DEFINE_PROP_UINT8("index", ClockMMIODevice, index, 0),
    DEFINE_PROP_END_OF_LIST(),
};

static void clock_mmio_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    device_class_set_props(dc, clock_mmio_dev_properties);
    dc->vmsd = &clock_mmio_vmstate;
    dc->desc = "MMIO clock device";
    set_bit(DEVICE_CATEGORY_MISC, dc->categories);
    /* Supported by TYPE_VIRT_MACHINE */
    dc->user_creatable = true;
}

static const TypeInfo clock_mmio_dev_info = {
    .name = TYPE_CLOCK_MMIO,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(ClockMMIODevice),
    .instance_init = clock_mmio_init,
    .class_init = clock_mmio_class_init,
    .class_size = sizeof(ClockMMIODeviceClass),
};

static void register_clock_mmio_dev_type(void)
{
    type_register_static(&clock_mmio_dev_info);
}

type_init(register_clock_mmio_dev_type)
