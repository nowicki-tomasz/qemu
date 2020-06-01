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

static void clock_mmio_init(Object *obj)
{
}

static const VMStateDescription clock_mmio_vmstate = {
    .name = TYPE_CLOCK_MMIO,
    .unmigratable = 1,
};

static Property clock_mmio_dev_properties[] = {
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
