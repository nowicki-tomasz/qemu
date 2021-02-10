/*
 * vfio based device assignment support - platform devices
 *
 * Copyright Linaro Limited, 2014
 *
 * Authors:
 *  Kim Phillips <kim.phillips@linaro.org>
 *  Eric Auger <eric.auger@linaro.org>
 *
 * This work is licensed under the terms of the GNU GPL, version 2.  See
 * the COPYING file in the top-level directory.
 *
 * Based on vfio based PCI device assignment support:
 *  Copyright Red Hat, Inc. 2012
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include <sys/ioctl.h>
#include <linux/vfio.h>

#include "hw/iommu/iommu.h"
#include "hw/vfio/vfio-platform.h"
#include "migration/vmstate.h"
#include "qemu/error-report.h"
#include "qemu/main-loop.h"
#include "qemu/module.h"
#include "qemu/range.h"
#include "exec/memory.h"
#include "exec/address-spaces.h"
#include "qemu/queue.h"
#include "hw/sysbus.h"
#include "trace.h"
#include "hw/irq.h"
#include "hw/platform-bus.h"
#include "hw/qdev-properties.h"
#include "sysemu/kvm.h"

/*
 * Functions used whatever the injection method
 */

static inline bool vfio_irq_is_automasked(VFIOINTp *intp)
{
    return intp->flags & VFIO_IRQ_INFO_AUTOMASKED;
}

/**
 * vfio_init_intp - allocate, initialize the IRQ struct pointer
 * and add it into the list of IRQs
 * @vbasedev: the VFIO device handle
 * @info: irq info struct retrieved from VFIO driver
 * @errp: error object
 */
static VFIOINTp *vfio_init_intp(VFIODevice *vbasedev,
                                struct vfio_irq_info info, Error **errp)
{
    int ret;
    VFIOPlatformDevice *vdev =
        container_of(vbasedev, VFIOPlatformDevice, vbasedev);
    SysBusDevice *sbdev = SYS_BUS_DEVICE(vdev);
    VFIOINTp *intp;

    intp = g_malloc0(sizeof(*intp));
    intp->vdev = vdev;
    intp->pin = info.index;
    intp->flags = info.flags;
    intp->state = VFIO_IRQ_INACTIVE;
    intp->kvm_accel = false;

    sysbus_init_irq(sbdev, &intp->qemuirq);

    /* Get an eventfd for trigger */
    intp->interrupt = g_malloc0(sizeof(EventNotifier));
    ret = event_notifier_init(intp->interrupt, 0);
    if (ret) {
        g_free(intp->interrupt);
        g_free(intp);
        error_setg_errno(errp, -ret,
                         "failed to initialize trigger eventfd notifier");
        return NULL;
    }
    if (vfio_irq_is_automasked(intp)) {
        /* Get an eventfd for resample/unmask */
        intp->unmask = g_malloc0(sizeof(EventNotifier));
        ret = event_notifier_init(intp->unmask, 0);
        if (ret) {
            g_free(intp->interrupt);
            g_free(intp->unmask);
            g_free(intp);
            error_setg_errno(errp, -ret,
                             "failed to initialize resample eventfd notifier");
            return NULL;
        }
    }

    QLIST_INSERT_HEAD(&vdev->intp_list, intp, next);
    return intp;
}

/**
 * vfio_set_trigger_eventfd - set VFIO eventfd handling
 *
 * @intp: IRQ struct handle
 * @handler: handler to be called on eventfd signaling
 *
 * Setup VFIO signaling and attach an optional user-side handler
 * to the eventfd
 */
static int vfio_set_trigger_eventfd(VFIOINTp *intp,
                                    eventfd_user_side_handler_t handler)
{
    VFIODevice *vbasedev = &intp->vdev->vbasedev;
    int32_t fd = event_notifier_get_fd(intp->interrupt);
    Error *err = NULL;
    int ret;

    qemu_set_fd_handler(fd, (IOHandler *)handler, NULL, intp);

    ret = vfio_set_irq_signaling(vbasedev, intp->pin, 0,
                                 VFIO_IRQ_SET_ACTION_TRIGGER, fd, &err);
    if (ret) {
        error_reportf_err(err, VFIO_MSG_PREFIX, vbasedev->name);
        qemu_set_fd_handler(fd, NULL, NULL, NULL);
    }

    return ret;
}

/*
 * Functions only used when eventfds are handled on user-side
 * ie. without irqfd
 */

/**
 * vfio_mmap_set_enabled - enable/disable the fast path mode
 * @vdev: the VFIO platform device
 * @enabled: the target mmap state
 *
 * enabled = true ~ fast path = MMIO region is mmaped (no KVM TRAP);
 * enabled = false ~ slow path = MMIO region is trapped and region callbacks
 * are called; slow path enables to trap the device IRQ status register reset
*/

static void vfio_mmap_set_enabled(VFIOPlatformDevice *vdev, bool enabled)
{
    int i;

    for (i = 0; i < vdev->vbasedev.num_regions; i++) {
        vfio_region_mmaps_set_enabled(vdev->regions[i], enabled);
    }
}

/**
 * vfio_intp_mmap_enable - timer function, restores the fast path
 * if there is no more active IRQ
 * @opaque: actually points to the VFIO platform device
 *
 * Called on mmap timer timout, this function checks whether the
 * IRQ is still active and if not, restores the fast path.
 * by construction a single eventfd is handled at a time.
 * if the IRQ is still active, the timer is re-programmed.
 */
static void vfio_intp_mmap_enable(void *opaque)
{
    VFIOINTp *tmp;
    VFIOPlatformDevice *vdev = (VFIOPlatformDevice *)opaque;

    qemu_mutex_lock(&vdev->intp_mutex);
    QLIST_FOREACH(tmp, &vdev->intp_list, next) {
        if (tmp->state == VFIO_IRQ_ACTIVE) {
            trace_vfio_platform_intp_mmap_enable(tmp->pin);
            /* re-program the timer to check active status later */
            timer_mod(vdev->mmap_timer,
                      qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL) +
                          vdev->mmap_timeout);
            qemu_mutex_unlock(&vdev->intp_mutex);
            return;
        }
    }
    vfio_mmap_set_enabled(vdev, true);
    qemu_mutex_unlock(&vdev->intp_mutex);
}

/**
 * vfio_intp_inject_pending_lockheld - Injects a pending IRQ
 * @opaque: opaque pointer, in practice the VFIOINTp handle
 *
 * The function is called on a previous IRQ completion, from
 * vfio_platform_eoi, while the intp_mutex is locked.
 * Also in such situation, the slow path already is set and
 * the mmap timer was already programmed.
 */
static void vfio_intp_inject_pending_lockheld(VFIOINTp *intp)
{
    trace_vfio_platform_intp_inject_pending_lockheld(intp->pin,
                              event_notifier_get_fd(intp->interrupt));

    intp->state = VFIO_IRQ_ACTIVE;

    /* trigger the virtual IRQ */
    qemu_set_irq(intp->qemuirq, 1);
}

/**
 * vfio_intp_interrupt - The user-side eventfd handler
 * @opaque: opaque pointer which in practice is the VFIOINTp handle
 *
 * the function is entered in event handler context:
 * the vIRQ is injected into the guest if there is no other active
 * or pending IRQ.
 */
static void vfio_intp_interrupt(VFIOINTp *intp)
{
    int ret;
//    VFIOINTp *tmp;
    VFIOPlatformDevice *vdev = intp->vdev;
//    bool delay_handling = false;

    qemu_mutex_lock(&vdev->intp_mutex);
//    if (intp->state == VFIO_IRQ_INACTIVE) {
//        QLIST_FOREACH(tmp, &vdev->intp_list, next) {
//            if (tmp->state == VFIO_IRQ_ACTIVE ||
//                tmp->state == VFIO_IRQ_PENDING) {
//
//                error_report("%s fd=%d irq %d state %d", __func__,
//                                         event_notifier_get_fd(intp->interrupt),
//                                         (int)tmp->pin, (int)tmp->state);
//
//                delay_handling = true;
//                break;
//            }
//        }
//    }
//    if (delay_handling) {

//        error_report("%s fd=%d delay_handling", __func__,
//                         event_notifier_get_fd(intp->interrupt));
        /*
         * the new IRQ gets a pending status and is pushed in
         * the pending queue
         */
//        intp->state = VFIO_IRQ_PENDING;
//        trace_vfio_intp_interrupt_set_pending(intp->pin);
//        QSIMPLEQ_INSERT_TAIL(&vdev->pending_intp_queue,
//                             intp, pqnext);
//        ret = event_notifier_test_and_clear(intp->interrupt);
//        qemu_mutex_unlock(&vdev->intp_mutex);
//        return;
//    }

    trace_vfio_platform_intp_interrupt(intp->pin,
                              event_notifier_get_fd(intp->interrupt));

    ret = event_notifier_test_and_clear(intp->interrupt);
    if (!ret) {
        error_report("Error when clearing fd=%d (ret = %d)",
                     event_notifier_get_fd(intp->interrupt), ret);
    }

    intp->state = VFIO_IRQ_ACTIVE;

    /* sets slow path */
    vfio_mmap_set_enabled(vdev, false);

    /* trigger the virtual IRQ */
    qemu_set_irq(intp->qemuirq, 1);

    /*
     * Schedule the mmap timer which will restore fastpath when no IRQ
     * is active anymore
     */
    if (vdev->mmap_timeout) {
        timer_mod(vdev->mmap_timer,
                  qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL) +
                      vdev->mmap_timeout);
    }
    qemu_mutex_unlock(&vdev->intp_mutex);
}

/**
 * vfio_platform_eoi - IRQ completion routine
 * @vbasedev: the VFIO device handle
 *
 * De-asserts the active virtual IRQ and unmasks the physical IRQ
 * (effective for level sensitive IRQ auto-masked by the  VFIO driver).
 * Then it handles next pending IRQ if any.
 * eoi function is called on the first access to any MMIO region
 * after an IRQ was triggered, trapped since slow path was set.
 * It is assumed this access corresponds to the IRQ status
 * register reset. With such a mechanism, a single IRQ can be
 * handled at a time since there is no way to know which IRQ
 * was completed by the guest (we would need additional details
 * about the IRQ status register mask).
 */
static void vfio_platform_eoi(VFIODevice *vbasedev)
{
    VFIOINTp *intp;
    VFIOPlatformDevice *vdev =
        container_of(vbasedev, VFIOPlatformDevice, vbasedev);

    qemu_mutex_lock(&vdev->intp_mutex);
    QLIST_FOREACH(intp, &vdev->intp_list, next) {
        if (intp->state == VFIO_IRQ_ACTIVE) {
            trace_vfio_platform_eoi(intp->pin,
                                event_notifier_get_fd(intp->interrupt));
            intp->state = VFIO_IRQ_INACTIVE;

            /* deassert the virtual IRQ */
            qemu_set_irq(intp->qemuirq, 0);

            if (vfio_irq_is_automasked(intp)) {
                /* unmasks the physical level-sensitive IRQ */
                vfio_unmask_single_irqindex(vbasedev, intp->pin);
            }

            /* a single IRQ can be active at a time */
            break;
        }
    }
    /* in case there are pending IRQs, handle the first one */
    if (!QSIMPLEQ_EMPTY(&vdev->pending_intp_queue)) {
        intp = QSIMPLEQ_FIRST(&vdev->pending_intp_queue);
        vfio_intp_inject_pending_lockheld(intp);
        QSIMPLEQ_REMOVE_HEAD(&vdev->pending_intp_queue, pqnext);
    }
    qemu_mutex_unlock(&vdev->intp_mutex);
}

/**
 * vfio_start_eventfd_injection - starts the virtual IRQ injection using
 * user-side handled eventfds
 * @sbdev: the sysbus device handle
 * @irq: the qemu irq handle
 */

static void vfio_start_eventfd_injection(SysBusDevice *sbdev, qemu_irq irq)
{
    VFIOPlatformDevice *vdev = VFIO_PLATFORM_DEVICE(sbdev);
    VFIOINTp *intp;

    QLIST_FOREACH(intp, &vdev->intp_list, next) {
        if (intp->qemuirq == irq) {
            break;
        }
    }
    assert(intp);

    if (vfio_set_trigger_eventfd(intp, vfio_intp_interrupt)) {
        abort();
    }
}

/*
 * Functions used for irqfd
 */

/**
 * vfio_set_resample_eventfd - sets the resamplefd for an IRQ
 * @intp: the IRQ struct handle
 * programs the VFIO driver to unmask this IRQ when the
 * intp->unmask eventfd is triggered
 */
static int vfio_set_resample_eventfd(VFIOINTp *intp)
{
    int32_t fd = event_notifier_get_fd(intp->unmask);
    VFIODevice *vbasedev = &intp->vdev->vbasedev;
    Error *err = NULL;
    int ret;

    qemu_set_fd_handler(fd, NULL, NULL, NULL);
    ret = vfio_set_irq_signaling(vbasedev, intp->pin, 0,
                                 VFIO_IRQ_SET_ACTION_UNMASK, fd, &err);
    if (ret) {
        error_reportf_err(err, VFIO_MSG_PREFIX, vbasedev->name);
    }
    return ret;
}

/**
 * vfio_start_irqfd_injection - starts the virtual IRQ injection using
 * irqfd
 *
 * @sbdev: the sysbus device handle
 * @irq: the qemu irq handle
 *
 * In case the irqfd setup fails, we fallback to userspace handled eventfd
 */
static void vfio_start_irqfd_injection(SysBusDevice *sbdev, qemu_irq irq)
{
    VFIOPlatformDevice *vdev = VFIO_PLATFORM_DEVICE(sbdev);
    VFIOINTp *intp;

    if (!kvm_irqfds_enabled() || !kvm_resamplefds_enabled() ||
        !vdev->irqfd_allowed) {
        goto fail_irqfd;
    }

    QLIST_FOREACH(intp, &vdev->intp_list, next) {
        if (intp->qemuirq == irq) {
            break;
        }
    }
    assert(intp);

    error_report("-------------> %s 0", __func__);

    if (kvm_irqchip_add_irqfd_notifier(kvm_state, intp->interrupt,
                                   intp->unmask, irq) < 0) {
        error_report("-------------> %s 0 1", __func__);
        goto fail_irqfd;
    }

    error_report("-------------> %s 1", __func__);

    if (vfio_set_trigger_eventfd(intp, NULL) < 0) {
        error_report("-------------> %s 1 1", __func__);
        goto fail_vfio;
    }

    error_report("-------------> %s 2", __func__);

    if (vfio_irq_is_automasked(intp)) {
        if (vfio_set_resample_eventfd(intp) < 0) {
            error_report("-------------> %s 2 1", __func__);
            goto fail_vfio;
        }
        trace_vfio_platform_start_level_irqfd_injection(intp->pin,
                                    event_notifier_get_fd(intp->interrupt),
                                    event_notifier_get_fd(intp->unmask));
    } else {
        trace_vfio_platform_start_edge_irqfd_injection(intp->pin,
                                    event_notifier_get_fd(intp->interrupt));
    }

    error_report("-------------> %s 3", __func__);

    intp->kvm_accel = true;

    return;
fail_vfio:
    kvm_irqchip_remove_irqfd_notifier(kvm_state, intp->interrupt, irq);
    abort();
fail_irqfd:
    vfio_start_eventfd_injection(sbdev, irq);
    return;
}

/* VFIO skeleton */

static void vfio_platform_compute_needs_reset(VFIODevice *vbasedev)
{
    vbasedev->needs_reset = true;
}

/* not implemented yet */
static int vfio_platform_hot_reset_multi(VFIODevice *vbasedev)
{
    return -1;
}

static char *vfio_platform_get_regulator(VFIODevice *vbasedev, int index,
                                       Error **errp)
{
    struct vfio_regulator_info reg_info = {
            .argsz = sizeof(reg_info),
            .index = index,
            .len = 32, /* Arbitrary name length */
    };
    int ret;

    reg_info.usr_data = (uint64_t)g_new0(uint8_t, reg_info.len);
    ret = ioctl(vbasedev->fd, VFIO_DEVICE_GET_REGULATOR_INFO, &reg_info);
    if (ret) {
        if (errno != ENOSPC)
            goto err;

        reg_info.usr_data = (uint64_t)g_renew(uint8_t, reg_info.usr_data,
                                              reg_info.len);
        ret = ioctl(vbasedev->fd, VFIO_DEVICE_GET_REGULATOR_INFO, &reg_info);
        if (ret)
            goto err;
    }

    return (char *)reg_info.usr_data;
err:
    error_setg_errno(errp, errno, "error getting regulator info");
    return NULL;
}

static int vfio_platform_get_gpio(VFIODevice *vbasedev, int index, Error **errp)
{
    struct vfio_gpio_info gpio_info = {
            .argsz = sizeof(gpio_info),
            .index = index,
            .len = 32, /* Arbitrary name length */
    };
    struct vfio_gpio_func_info gpio_func_info = {
            .argsz = sizeof(gpio_func_info),
            .index = index,
    };
    int ret, i;

    error_report("%s start getting gpio info for idx %d", __func__, index);

    gpio_info.usr_data = (uint64_t)g_new0(uint8_t, gpio_info.len);
    ret = ioctl(vbasedev->fd, VFIO_DEVICE_GET_GPIO_INFO, &gpio_info);
    if (ret) {
        if (errno != ENOSPC)
            goto err;

        gpio_info.usr_data = (uint64_t)g_renew(uint8_t, gpio_info.usr_data,
                                              gpio_info.len);
        ret = ioctl(vbasedev->fd, VFIO_DEVICE_GET_GPIO_INFO, &gpio_info);
        if (ret)
            goto err;
    }

    vbasedev->gpio[index].gpio_func_names = (char *)gpio_info.usr_data;
    vbasedev->gpio[index].pin_num = gpio_info.pin_num;

    error_report("%s function name %s index %d pins %ld",
            __func__, (char *)gpio_info.usr_data, index, (long)gpio_info.pin_num);

    vbasedev->gpio[index].flags = (uint64_t *)g_new0(uint64_t, gpio_info.pin_num);
    for (i = 0; i < gpio_info.pin_num; i++) {
        gpio_func_info.pin_idx = i;
        ret = ioctl(vbasedev->fd, VFIO_DEVICE_GET_GPIO_FUNC_INFO, &gpio_func_info);
        if (ret)
            goto err;

        error_report("%s function %s index %d pin %d flags 0x%lx",
                __func__, (char *)gpio_info.usr_data, index, i, (long)gpio_func_info.flags);
        vbasedev->gpio[index].flags[i] = gpio_func_info.flags;
    }

    return 0;
err:
    error_setg_errno(errp, errno, "error getting GPIO info");
    return ret;
}

static void vfio_init_fault_regions(VFIOPlatformDevice *vdev, Error **errp)
{
    struct vfio_region_info *fault_region_info = NULL;
    struct vfio_region_info_cap_fault *cap_fault;
    VFIODevice *vbasedev = &vdev->vbasedev;
    struct vfio_info_cap_header *hdr;
    char *fault_region_name;
    int ret;

    ret = vfio_get_dev_region_info(&vdev->vbasedev,
                                   VFIO_REGION_TYPE_NESTED,
                                   VFIO_REGION_SUBTYPE_NESTED_DMA_FAULT,
                                   &fault_region_info);
    if (ret) {
        goto out;
    }

    error_report("%s found ext region index = %d",
            __func__, fault_region_info->index);

    hdr = vfio_get_region_info_cap(fault_region_info,
                                   VFIO_REGION_INFO_CAP_DMA_FAULT);
    if (!hdr) {
        error_setg(errp, "failed to retrieve DMA FAULT capability");
        goto out;
    }
    cap_fault = container_of(hdr, struct vfio_region_info_cap_fault,
                             header);
    if (cap_fault->version != 1) {
        error_setg(errp, "Unsupported DMA FAULT API version %d",
                   cap_fault->version);
        goto out;
    }

    error_report("%s ext region index = %d has required caps",
            __func__, fault_region_info->index);

    fault_region_name = g_strdup_printf("%s DMA FAULT %d",
                                        vbasedev->name,
                                        fault_region_info->index);

    ret = vfio_region_setup(OBJECT(vdev), vbasedev,
                            &vdev->dma_fault_region,
                            fault_region_info->index,
                            fault_region_name);
    g_free(fault_region_name);
    if (ret) {
        error_setg_errno(errp, -ret,
                         "failed to set up the DMA FAULT region %d",
                         fault_region_info->index);
        goto out;
    }

    error_report("%s ext region index = %d setup properly",
            __func__, fault_region_info->index);

    ret = vfio_region_mmap(&vdev->dma_fault_region);
    if (ret) {
        error_setg_errno(errp, -ret, "Failed to mmap the DMA FAULT queue");
    }

    error_report("%s ext region index = %d mapped properly, ret = %d",
                __func__, fault_region_info->index, ret);

out:
    g_free(fault_region_info);
}

/**
 * vfio_populate_device - Allocate and populate MMIO region
 * and IRQ structs according to driver returned information
 * @vbasedev: the VFIO device handle
 * @errp: error object
 *
 */
static int vfio_populate_device(VFIODevice *vbasedev, Error **errp)
{
    VFIOINTp *intp, *tmp;
    int i, ret = -1;
    VFIOPlatformDevice *vdev =
        container_of(vbasedev, VFIOPlatformDevice, vbasedev);
    Error *err = NULL;

    if (!(vbasedev->flags & VFIO_DEVICE_FLAGS_PLATFORM)) {
        error_setg(errp, "this isn't a platform device");
        return ret;
    }

    vdev->regions = g_new0(VFIORegion *, vbasedev->num_regions);

    for (i = 0; i < vbasedev->num_regions; i++) {
        char *name = g_strdup_printf("VFIO %s region %d\n", vbasedev->name, i);

        vdev->regions[i] = g_new0(VFIORegion, 1);
        ret = vfio_region_setup(OBJECT(vdev), vbasedev,
                                vdev->regions[i], i, name);
        g_free(name);
        if (ret) {
            error_setg_errno(errp, -ret, "failed to get region %d info", i);
            goto reg_error;
        }
    }

    vdev->mmap_timer = timer_new_ms(QEMU_CLOCK_VIRTUAL,
                                    vfio_intp_mmap_enable, vdev);

    QSIMPLEQ_INIT(&vdev->pending_intp_queue);

    vfio_init_fault_regions(vdev, &err);
    if (err) {
        error_propagate(errp, err);
        goto reg_error;
    }

    for (i = 0; i < vbasedev->num_irqs; i++) {
        struct vfio_irq_info irq = { .argsz = sizeof(irq) };

        irq.index = i;
        ret = ioctl(vbasedev->fd, VFIO_DEVICE_GET_IRQ_INFO, &irq);
        if (ret) {
            error_setg_errno(errp, -ret, "failed to get device irq info");
            goto irq_err;
        } else {
            trace_vfio_platform_populate_interrupts(irq.index,
                                                    irq.count,
                                                    irq.flags);
            intp = vfio_init_intp(vbasedev, irq, errp);
            if (!intp) {
                ret = -1;
                goto irq_err;
            }
        }
    }

    vbasedev->regulator_names = g_new(char *, vbasedev->num_regulators);
    for (i = 0; i < vbasedev->num_regulators; i++) {
        vbasedev->regulator_names[i] = vfio_platform_get_regulator(vbasedev, i,
                                                                   errp);
        if (!vbasedev->regulator_names[i])
            goto regulator_err;
    }

    vbasedev->gpio = g_new(VFIODeviceGpio, vbasedev->num_gpio_func);
    for (i = 0; i < vbasedev->num_gpio_func; i++) {
        ret = vfio_platform_get_gpio(vbasedev, i, errp);
        if (ret)
            goto regulator_err;
    }

    return 0;
regulator_err:
    for (i = 0; i < vbasedev->num_regulators; i++)
        g_free(vbasedev->regulator_names[i]);
    g_free(vbasedev->regulator_names);
irq_err:
    timer_del(vdev->mmap_timer);
    QLIST_FOREACH_SAFE(intp, &vdev->intp_list, next, tmp) {
        QLIST_REMOVE(intp, next);
        g_free(intp);
    }
reg_error:
    vfio_region_finalize(&vdev->dma_fault_region);
    for (i = 0; i < vbasedev->num_regions; i++) {
        if (vdev->regions[i]) {
            vfio_region_finalize(vdev->regions[i]);
        }
        g_free(vdev->regions[i]);
    }
    g_free(vdev->regions);
    return ret;
}

/* specialized functions for VFIO Platform devices */
static VFIODeviceOps vfio_platform_ops = {
    .vfio_compute_needs_reset = vfio_platform_compute_needs_reset,
    .vfio_hot_reset_multi = vfio_platform_hot_reset_multi,
    .vfio_eoi = vfio_platform_eoi,
};

/**
 * vfio_base_device_init - perform preliminary VFIO setup
 * @vbasedev: the VFIO device handle
 * @errp: error object
 *
 * Implement the VFIO command sequence that allows to discover
 * assigned device resources: group extraction, device
 * fd retrieval, resource query.
 * Precondition: the device name must be initialized
 */
static int vfio_base_device_init(SysBusDevice *sbdev, Error **errp)
{
    VFIOPlatformDevice *vdev = VFIO_PLATFORM_DEVICE(sbdev);
    VFIODevice *vbasedev = &vdev->vbasedev;
    VFIOGroup *group;
    VFIODevice *vbasedev_iter;
    char *tmp, group_path[PATH_MAX], *group_name;
    ssize_t len;
    struct stat st;
    int groupid;
    int ret;

    error_report("%s -------------> 00", __func__);

    /* @sysfsdev takes precedence over @host */
    if (vbasedev->sysfsdev) {
        g_free(vbasedev->name);
        vbasedev->name = g_path_get_basename(vbasedev->sysfsdev);
    } else {
        if (!vbasedev->name || strchr(vbasedev->name, '/')) {
            error_setg(errp, "wrong host device name");
            return -EINVAL;
        }

        vbasedev->sysfsdev = g_strdup_printf("/sys/bus/platform/devices/%s",
                                             vbasedev->name);
    }

    if (stat(vbasedev->sysfsdev, &st) < 0) {
        error_setg_errno(errp, errno,
                         "failed to get the sysfs host device file status");
        return -errno;
    }

    tmp = g_strdup_printf("%s/iommu_group", vbasedev->sysfsdev);
    len = readlink(tmp, group_path, sizeof(group_path));
    g_free(tmp);

    if (len < 0 || len >= sizeof(group_path)) {
        ret = len < 0 ? -errno : -ENAMETOOLONG;
        error_setg_errno(errp, -ret, "no iommu_group found");
        return ret;
    }

    group_path[len] = 0;

    group_name = basename(group_path);
    if (sscanf(group_name, "%d", &groupid) != 1) {
        error_setg_errno(errp, errno, "failed to read %s", group_path);
        return -errno;
    }

    trace_vfio_platform_base_device_init(vbasedev->name, groupid);

    error_report("%s -------------> 0", __func__);

    group = vfio_get_group(groupid, platform_bus_device_iommu_address_space(sbdev), errp);
    if (!group) {
        return -ENOENT;
    }

    error_report("%s -------------> 1", __func__);

    QLIST_FOREACH(vbasedev_iter, &group->device_list, next) {
        if (strcmp(vbasedev_iter->name, vbasedev->name) == 0) {
            error_setg(errp, "device is already attached");
            vfio_put_group(group);
            return -EBUSY;
        }
    }

    error_report("%s -------------> 2", __func__);

    ret = vfio_get_device(group, vbasedev->name, vbasedev, errp);
    if (ret) {
        vfio_put_group(group);
        return ret;
    }

    error_report("%s -------------> 3 nb_ext_irqs %d", __func__, vdev->vbasedev.num_ext_irqs);
    if (vdev->vbasedev.num_ext_irqs > 0) {
        vdev->ext_irqs = g_new0(VFIOPlatformExtIRQ, vdev->vbasedev.num_ext_irqs);
    }

    error_report("%s -------------> 4", __func__);

    ret = vfio_populate_device(vbasedev, errp);
    if (ret) {
        vfio_put_group(group);
    }

    return ret;
}

static int vfio_iommu_set_pasid_table(SysBusDevice *sbdev, IOMMUConfig *config)
{
    VFIOPlatformDevice *vdev = VFIO_PLATFORM_DEVICE(sbdev);
    VFIOContainer *container = vdev->vbasedev.group->container;
    struct vfio_iommu_type1_set_pasid_table info;

    info.argsz = sizeof(info);
    info.flags = VFIO_PASID_TABLE_FLAG_SET;
    memcpy(&info.config, &config->pasid_cfg, sizeof(config->pasid_cfg));

    return ioctl(container->fd, VFIO_IOMMU_SET_PASID_TABLE, &info);
}

static VFIOPlatformPASIDOps vfio_platform_pasid_ops = {
    .set_pasid_table = vfio_iommu_set_pasid_table,
};

static void vfio_platform_setup_pasid_ops(SysBusDevice *sbdev,
                                          VFIOPlatformPASIDOps *ops)
{
    VFIOPlatformDevice *vdev = VFIO_PLATFORM_DEVICE(sbdev);

    assert(ops && !vdev->pasid_ops);
    vdev->pasid_ops = ops;
}

bool vfio_platform_device_is_pasid_ops_set(SysBusDevice *sbdev)
{
    VFIOPlatformDevice *vdev = VFIO_PLATFORM_DEVICE(sbdev);

    return !!(vdev && vdev->pasid_ops);
}

int vfio_platform_device_set_pasid_table(SysBusDevice *sbdev,
                                         IOMMUConfig *config)
{
    VFIOPlatformDevice *vdev = VFIO_PLATFORM_DEVICE(sbdev);

    if (vdev && vdev->pasid_ops && vdev->pasid_ops->set_pasid_table) {
        return vdev->pasid_ops->set_pasid_table(sbdev, config);
    }
    return -ENOENT;
}

static void vfio_platform_dma_fault_notifier_handler(void *opaque)
{
    VFIOPlatformExtIRQ *ext_irq = opaque;
    VFIOPlatformDevice *vdev = ext_irq->vdev;
    AddressSpace *as = platform_bus_device_iommu_address_space(SYS_BUS_DEVICE(vdev));
    IOMMUMemoryRegion *iommu_mr = IOMMU_MEMORY_REGION(as->root);
    struct vfio_region_dma_fault header;
    struct iommu_fault *queue;
    char *queue_buffer = NULL;
    ssize_t bytes;

    if (!event_notifier_test_and_clear(&ext_irq->notifier)) {
        return;
    }

    bytes = pread(vdev->vbasedev.fd, &header, sizeof(header),
                  vdev->dma_fault_region.fd_offset);
    if (bytes != sizeof(header)) {
        error_report("%s unable to read the fault region header (0x%lx)",
                     __func__, bytes);
        return;
    }

    /* Normally the fault queue is mmapped */
    queue = (struct iommu_fault *)vdev->dma_fault_region.mmaps[0].mmap;
    if (!queue) {
        size_t queue_size = header.nb_entries * header.entry_size;

        error_report("%s: fault queue not mmapped: slower fault handling",
                     vdev->vbasedev.name);

        queue_buffer = g_malloc(queue_size);
        bytes =  pread(vdev->vbasedev.fd, queue_buffer, queue_size,
                       vdev->dma_fault_region.fd_offset + header.offset);
        if (bytes != queue_size) {
            error_report("%s unable to read the fault queue (0x%lx)",
                         __func__, bytes);
            return;
        }

        queue = (struct iommu_fault *)queue_buffer;
    }

    while (vdev->fault_tail_index != header.head) {
        memory_region_inject_faults(iommu_mr, 1,
                                    &queue[vdev->fault_tail_index]);
        vdev->fault_tail_index =
            (vdev->fault_tail_index + 1) % header.nb_entries;
    }
    bytes = pwrite(vdev->vbasedev.fd, &vdev->fault_tail_index, 4,
                   vdev->dma_fault_region.fd_offset);
    if (bytes != 4) {
        error_report("%s unable to write the fault region tail index (0x%lx)",
                     __func__, bytes);
    }
    g_free(queue_buffer);
}

static int vfio_platform_register_ext_irq_handler(VFIOPlatformDevice *vdev,
                                         uint32_t type, uint32_t subtype,
                                         IOHandler *handler)
{
    int32_t fd, ext_irq_index, index;
    struct vfio_irq_info *irq_info;
    Error *err = NULL;
    EventNotifier *n;
    int ret;

    error_report("%s: 0", __func__);

    ret = vfio_get_dev_irq_info(&vdev->vbasedev, type, subtype, &irq_info);
    if (ret) {
        return ret;
    }

    error_report("%s: 1 irq_info->index = %d, vdev->vbasedev.num_irqs = %d",
            __func__, (int)irq_info->index, (int)vdev->vbasedev.num_irqs);

    index = irq_info->index;
    ext_irq_index = irq_info->index - vdev->vbasedev.num_irqs;
    g_free(irq_info);

    error_report("%s: 2 ext_irq_index = %d", __func__, (int)ext_irq_index);

    assert(ext_irq_index >= 0 && ext_irq_index < vdev->vbasedev.num_ext_irqs);

    vdev->ext_irqs[ext_irq_index].vdev = vdev;
    vdev->ext_irqs[ext_irq_index].index = index;
    n = &vdev->ext_irqs[ext_irq_index].notifier;

    ret = event_notifier_init(n, 0);
    if (ret) {
        error_report("vfio: Unable to init event notifier for ext irq %d(%d)",
                     ext_irq_index, ret);
        return ret;
    }

    error_report("%s: 3", __func__);

    fd = event_notifier_get_fd(n);

    error_report("%s: 4", __func__);

    qemu_set_fd_handler(fd, handler, NULL,
                        &vdev->ext_irqs[ext_irq_index]);

    error_report("%s: 5", __func__);

    ret = vfio_set_irq_signaling(&vdev->vbasedev, index, 0,
                                 VFIO_IRQ_SET_ACTION_TRIGGER, fd, &err);
    if (ret) {
        error_report("%s: 6", __func__);
        error_reportf_err(err, VFIO_MSG_PREFIX, vdev->vbasedev.name);
        qemu_set_fd_handler(fd, NULL, NULL, vdev);
        event_notifier_cleanup(n);
    }
    return ret;
}

static void vfio_platform_unregister_ext_irq_notifiers(VFIOPlatformDevice *vdev)
{
    VFIODevice *vbasedev = &vdev->vbasedev;
    Error *err = NULL;
    int i;

    for (i = 0; i < vbasedev->num_ext_irqs; i++) {
        if (vfio_set_irq_signaling(vbasedev, i + vbasedev->num_irqs , 0,
                                   VFIO_IRQ_SET_ACTION_TRIGGER, -1, &err)) {
            error_reportf_err(err, VFIO_MSG_PREFIX, vdev->vbasedev.name);
        }
        qemu_set_fd_handler(event_notifier_get_fd(&vdev->ext_irqs[i].notifier),
                            NULL, NULL, vdev);
        event_notifier_cleanup(&vdev->ext_irqs[i].notifier);
    }
    g_free(vdev->ext_irqs);
}

/**
 * vfio_platform_realize  - the device realize function
 * @dev: device state pointer
 * @errp: error
 *
 * initialize the device, its memory regions and IRQ structures
 * IRQ are started separately
 */
static void vfio_platform_realize(DeviceState *dev, Error **errp)
{
    VFIOPlatformDevice *vdev = VFIO_PLATFORM_DEVICE(dev);
    SysBusDevice *sbdev = SYS_BUS_DEVICE(dev);
    VFIODevice *vbasedev = &vdev->vbasedev;
    int i, ret;

    error_report("%s -------------> 0", __func__);

    vbasedev->type = VFIO_DEVICE_TYPE_PLATFORM;
    vbasedev->dev = dev;
    vbasedev->ops = &vfio_platform_ops;

    qemu_mutex_init(&vdev->intp_mutex);

    trace_vfio_platform_realize(vbasedev->sysfsdev ?
                                vbasedev->sysfsdev : vbasedev->name,
                                vdev->compat);

    ret = vfio_base_device_init(sbdev, errp);
    if (ret) {
        goto out;
    }

    if (!vdev->compat) {
        GError *gerr = NULL;
        gchar *contents;
        gsize length;
        char *path;

        path = g_strdup_printf("%s/of_node/compatible", vbasedev->sysfsdev);
        if (!g_file_get_contents(path, &contents, &length, &gerr)) {
            error_setg(errp, "%s", gerr->message);
            g_error_free(gerr);
            g_free(path);
            return;
        }
        g_free(path);
        vdev->compat = contents;
        for (vdev->num_compat = 0; length; vdev->num_compat++) {
            size_t skip = strlen(contents) + 1;
            contents += skip;
            length -= skip;
        }
    }

    for (i = 0; i < vbasedev->num_regions; i++) {
        if (vfio_region_mmap(vdev->regions[i])) {
            warn_report("%s mmap unsupported, performance may be slow",
                        memory_region_name(vdev->regions[i]->mem));
        }
        sysbus_init_mmio(sbdev, vdev->regions[i]->mem);
    }
out:
    if (!ret) {

        vfio_platform_register_ext_irq_handler(vdev, VFIO_IRQ_TYPE_NESTED,
                                      VFIO_IRQ_SUBTYPE_DMA_FAULT,
                                      vfio_platform_dma_fault_notifier_handler);
        vfio_platform_setup_pasid_ops(sbdev, &vfio_platform_pasid_ops);
        return;
    }

    if (vdev->vbasedev.name) {
        error_prepend(errp, VFIO_MSG_PREFIX, vdev->vbasedev.name);
    } else {
        error_prepend(errp, "vfio error: ");
    }
}

static void vfio_platform_unrealize(DeviceState *dev, Error **errp)
{
    VFIOPlatformDevice *vdev = VFIO_PLATFORM_DEVICE(dev);

    vfio_platform_unregister_ext_irq_notifiers(vdev);
}

static const VMStateDescription vfio_platform_vmstate = {
    .name = "vfio-platform",
    .unmigratable = 1,
};

static Property vfio_platform_dev_properties[] = {
    DEFINE_PROP_STRING("host", VFIOPlatformDevice, vbasedev.name),
    DEFINE_PROP_STRING("sysfsdev", VFIOPlatformDevice, vbasedev.sysfsdev),
    DEFINE_PROP_BOOL("x-no-mmap", VFIOPlatformDevice, vbasedev.no_mmap, false),
    DEFINE_PROP_UINT32("mmap-timeout-ms", VFIOPlatformDevice,
                       mmap_timeout, 1100),
    DEFINE_PROP_BOOL("x-irqfd", VFIOPlatformDevice, irqfd_allowed, true),
    DEFINE_PROP_LINK("parent", VFIOPlatformDevice, parent, "vfio-platform",
                     struct VFIOPlatformDevice *),
    DEFINE_PROP_UINT32("request-id", VFIOPlatformDevice, requestid, 0),
    DEFINE_PROP_END_OF_LIST(),
};

static void vfio_platform_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    SysBusDeviceClass *sbc = SYS_BUS_DEVICE_CLASS(klass);

    dc->realize = vfio_platform_realize;
    dc->unrealize = vfio_platform_unrealize;
    device_class_set_props(dc, vfio_platform_dev_properties);
    dc->vmsd = &vfio_platform_vmstate;
    dc->desc = "VFIO-based platform device assignment";
    sbc->connect_irq_notifier = vfio_start_irqfd_injection;
    set_bit(DEVICE_CATEGORY_MISC, dc->categories);
    /* Supported by TYPE_VIRT_MACHINE */
    dc->user_creatable = true;
}

static const TypeInfo vfio_platform_dev_info = {
    .name = TYPE_VFIO_PLATFORM,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(VFIOPlatformDevice),
    .class_init = vfio_platform_class_init,
    .class_size = sizeof(VFIOPlatformDeviceClass),
};

static void register_vfio_platform_dev_type(void)
{
    type_register_static(&vfio_platform_dev_info);
}

type_init(register_vfio_platform_dev_type)
