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

#include "hw/vfio/vfio-platform.h"
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
#define CLK_MMIO_FALGS		    0x4 // READONLY
#define CLK_MMIO_PREPARE	    0x8
#define CLK_MMIO_ENABLE         0xc

static MemTxResult clock_mmio_read(void *opaque, hwaddr offset, uint64_t *data,
                                  unsigned size, MemTxAttrs attrs)
{
    ClockMMIODevice *clk_dev = opaque;
    VFIOPlatformDevice *vdev = VFIO_PLATFORM_DEVICE(clk_dev->dev);
    VFIODevice *vbasedev = &vdev->vbasedev;
    struct vfio_clk *clk;
    uint32_t *payload;
    int argsz = sizeof(*clk);
    int ret;

    error_report("--------------------> %s clk %s offset 0x%lx size 0x%lx",
                 __func__, clk_dev->name, (long)offset, (long)size);

    switch (offset) {
    case CLK_MMIO_RATE:
        argsz += sizeof(*payload);
        clk = g_malloc0(argsz);

        clk->argsz = argsz;
        clk->flags = VFIO_CLK_GET_RATE;
        clk->index = clk_dev->index;
        payload = (uint32_t *)&clk->data; // rate

        error_report("--------------------> %s clk get rate %s",
                     __func__, clk_dev->name);

        break;
    case CLK_MMIO_FALGS:
    	argsz += sizeof(*payload);
        clk = g_malloc0(argsz);

        clk->argsz = argsz;
        clk->flags = VFIO_CLK_GET_FLAGS;
        clk->index = clk_dev->index;
        payload = (uint32_t *)&clk->data; // flags

        error_report("--------------------> %s clk get flags %s",
                             __func__, clk_dev->name);

        break;
    default:
        *data = 0;
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

    error_report("--------------------> %s clk %s payload %ld",
                 __func__, clk_dev->name, (long)(*payload));

    *data = *payload;
    g_free(clk);
    return MEMTX_OK;
}

static MemTxResult clock_mmio_write(void *opaque, hwaddr offset, uint64_t data,
                                   unsigned size, MemTxAttrs attrs)
{
    ClockMMIODevice *clk_dev = opaque;
    VFIOPlatformDevice *vdev = VFIO_PLATFORM_DEVICE(clk_dev->dev);
    VFIODevice *vbasedev = &vdev->vbasedev;
    struct vfio_clk *clk;
    uint32_t *payload;
    int argsz = sizeof(*clk);
    int ret;

    error_report("--------------------> %s clk %s offset 0x%lx size 0x%lx data 0x%lx",
                 __func__, clk_dev->name, (long)offset, (long)size, (long)data);

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
    case CLK_MMIO_RATE:
    	argsz += sizeof(*payload);
        clk = g_malloc0(argsz);

        clk->argsz = argsz;
        clk->flags = VFIO_CLK_SET_RATE;
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

static const MemoryRegionOps clock_mmio_ops = {
    .read_with_attrs = clock_mmio_read,
    .write_with_attrs = clock_mmio_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = {
        .min_access_size = 4,
        .max_access_size = 4,
    },
    .impl = {
        .min_access_size = 4,
        .max_access_size = 4,
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

    dc->props = clock_mmio_dev_properties;
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
