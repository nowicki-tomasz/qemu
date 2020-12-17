/*
 * ARM Platform Bus device tree generation helpers
 *
 * Copyright (c) 2014 Linaro Limited
 *
 * Authors:
 *  Alex Graf <agraf@suse.de>
 *  Eric Auger <eric.auger@linaro.org>
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
#include <libfdt.h>
#ifdef CONFIG_LINUX
#include <linux/vfio.h>
#endif
#ifdef CONFIG_FNMATCH
#include <fnmatch.h>
#endif
#include "hw/arm/sysbus-fdt.h"
#include "qemu/error-report.h"
#include "sysemu/device_tree.h"
#include "sysemu/tpm.h"
#include "hw/platform-bus.h"
#include "hw/vfio/vfio-platform.h"
#include "hw/vfio/vfio-calxeda-xgmac.h"
#include "hw/vfio/vfio-amd-xgbe.h"
#include "hw/virtio/virtio-mmio.h"
#include "hw/display/ramfb.h"
#include "hw/arm/fdt.h"

#include "standard-headers/linux/virtio_ids.h"

enum GenericPropertyAction {
    PROP_IGNORE,
    PROP_WARN,
    PROP_COPY,
    PROP_REJECT,
};

typedef struct ProppertList{
    const char *name;
    enum GenericPropertyAction action;
} ProppertList;

/* struct that allows to match a device and create its FDT node */
typedef struct BindingEntry {
    const char *typename;
    const char *compat;
    int  (*add_fn)(SysBusDevice *sbdev, void *opaque, ProppertList *properties);
    bool (*match_fn)(SysBusDevice *sbdev, const struct BindingEntry *combo);
    ProppertList *properties;
} BindingEntry;

/*
 * internal struct that contains the information to create dynamic
 * sysbus device node
 */
typedef struct PlatformBusFDTData {
    void *fdt; /* device tree handle */
    int irq_start; /* index of the first IRQ usable by platform bus devices */
    const char *pbus_node_name; /* name of the platform bus node */
    PlatformBusDevice *pbus;
    VFIOPlatformDevice *parent;
    const BindingEntry *bindings;
    unsigned int binding_size;
} PlatformBusFDTData;

/* helpers */

typedef struct HostProperty {
    const char *name;
    bool optional;
} HostProperty;

static char gpu_node_name[100];
static gchar *platform_bus_node;

#ifdef CONFIG_LINUX

static QLIST_HEAD(, VFIOPlatformDevice) dev_list =
    QLIST_HEAD_INITIALIZER(dev_list);

static uint32_t gmu_phandle;
static uint32_t ec_pwm_handle;
static uint32_t panel_backlight_handle;
static uint32_t disp_clk_handle;
static uint32_t panel_gpio_handle;
static uint32_t panel_dsi_bridge_phandle, panel_dsi_bridge_rempte_endpoint;
static uint32_t mdss_dsi_phandle, mdss_dsi_rempte_endpoint;
static uint32_t mdss_dp_phandle, mdss_dp_rempte_endpoint;
static uint32_t mdss_dsi_bridge_phandle, mdss_dsi_bridge_rempte_endpoint;

/**
 * copy_properties_from_host
 *
 * copies properties listed in an array from host device tree to
 * guest device tree. If a non optional property is not found, the
 * function asserts. An optional property is ignored if not found
 * in the host device tree.
 * @props: array of HostProperty to copy
 * @nb_props: number of properties in the array
 * @host_dt: host device tree blob
 * @guest_dt: guest device tree blob
 * @node_path: host dt node path where the property is supposed to be
              found
 * @nodename: guest node name the properties should be added to
 */
static void copy_properties_from_host(HostProperty *props, int nb_props,
                                      void *host_fdt, void *guest_fdt,
                                      char *node_path, char *nodename)
{
    int i, prop_len;
    const void *r;
    Error *err = NULL;

    for (i = 0; i < nb_props; i++) {
        r = qemu_fdt_getprop(host_fdt, node_path,
                             props[i].name,
                             &prop_len,
                             &err);
        if (r) {
            qemu_fdt_setprop(guest_fdt, nodename,
                             props[i].name, r, prop_len);
        } else {
            if (props[i].optional && prop_len == -FDT_ERR_NOTFOUND) {
                /* optional property does not exist */
                error_free(err);
            } else {
                error_report_err(err);
            }
            if (!props[i].optional) {
                /* mandatory property not found: bail out */
                exit(1);
            }
            err = NULL;
        }
    }
}

/* clock properties whose values are copied/pasted from host */
static HostProperty clock_copied_properties[] = {
    {"compatible", false},
    {"#clock-cells", false},
    {"clock-frequency", true},
    {"clock-output-names", true},
};

/**
 * fdt_build_clock_node
 *
 * Build a guest clock node, used as a dependency from a passthrough'ed
 * device. Most information are retrieved from the host clock node.
 * Also check the host clock is a fixed one.
 *
 * @host_fdt: host device tree blob from which info are retrieved
 * @guest_fdt: guest device tree blob where the clock node is added
 * @host_phandle: phandle of the clock in host device tree
 * @guest_phandle: phandle to assign to the guest node
 */
static void fdt_build_clock_node(void *host_fdt, void *guest_fdt,
                                uint32_t host_phandle,
                                uint32_t guest_phandle)
{
    char *node_path = NULL;
    char *nodename;
    const void *r;
    int ret, node_offset, prop_len, path_len = 16;

    node_offset = fdt_node_offset_by_phandle(host_fdt, host_phandle);
    if (node_offset <= 0) {
        error_report("not able to locate clock handle %d in host device tree",
                     host_phandle);
        exit(1);
    }
    node_path = g_malloc(path_len);
    while ((ret = fdt_get_path(host_fdt, node_offset, node_path, path_len))
            == -FDT_ERR_NOSPACE) {
        path_len += 16;
        node_path = g_realloc(node_path, path_len);
    }
    if (ret < 0) {
        error_report("not able to retrieve node path for clock handle %d",
                     host_phandle);
        exit(1);
    }

    r = qemu_fdt_getprop(host_fdt, node_path, "compatible", &prop_len,
                         &error_fatal);
    if (strcmp(r, "fixed-clock")) {
        error_report("clock handle %d is not a fixed clock", host_phandle);
        exit(1);
    }

    nodename = strrchr(node_path, '/');
    qemu_fdt_add_subnode(guest_fdt, nodename);

    copy_properties_from_host(clock_copied_properties,
                              ARRAY_SIZE(clock_copied_properties),
                              host_fdt, guest_fdt,
                              node_path, nodename);

    qemu_fdt_setprop_cell(guest_fdt, nodename, "phandle", guest_phandle);

    g_free(node_path);
}

/**
 * sysfs_to_dt_name: convert the name found in sysfs into the node name
 * for instance e0900000.xgmac is converted into xgmac@e0900000
 * @sysfs_name: directory name in sysfs
 *
 * returns the device tree name upon success or NULL in case the sysfs name
 * does not match the expected format
 */
static char *sysfs_to_dt_name(const char *sysfs_name)
{
    gchar **substrings =  g_strsplit(sysfs_name, ".", 2);
    char *dt_name = NULL;

    if (!substrings || !substrings[0] || !substrings[1]) {
        goto out;
    }
    dt_name = g_strdup_printf("%s@%s", substrings[1], substrings[0]);
out:
    g_strfreev(substrings);
    return dt_name;
}

/* Device Specific Code */

/**
 * add_calxeda_midway_xgmac_fdt_node
 *
 * Generates a simple node with following properties:
 * compatible string, regs, interrupts, dma-coherent
 */
static int add_calxeda_midway_xgmac_fdt_node(SysBusDevice *sbdev, void *opaque,
                                             ProppertList *properties)
{
    PlatformBusFDTData *data = opaque;
    PlatformBusDevice *pbus = data->pbus;
    void *fdt = data->fdt;
    const char *parent_node = data->pbus_node_name;
    int compat_str_len, i;
    char *nodename;
    uint32_t *irq_attr, *reg_attr;
    uint64_t mmio_base, irq_number;
    VFIOPlatformDevice *vdev = VFIO_PLATFORM_DEVICE(sbdev);
    VFIODevice *vbasedev = &vdev->vbasedev;

    mmio_base = platform_bus_get_mmio_addr(pbus, sbdev, 0);
    nodename = g_strdup_printf("%s/%s@%" PRIx64, parent_node,
                               vbasedev->name, mmio_base);
    qemu_fdt_add_subnode(fdt, nodename);

    compat_str_len = strlen(vdev->compat) + 1;
    qemu_fdt_setprop(fdt, nodename, "compatible",
                          vdev->compat, compat_str_len);

    qemu_fdt_setprop(fdt, nodename, "dma-coherent", "", 0);

    reg_attr = g_new(uint32_t, vbasedev->num_regions * 2);
    for (i = 0; i < vbasedev->num_regions; i++) {
        mmio_base = platform_bus_get_mmio_addr(pbus, sbdev, i);
        reg_attr[2 * i] = cpu_to_be32(mmio_base);
        reg_attr[2 * i + 1] = cpu_to_be32(
                                memory_region_size(vdev->regions[i]->mem));
    }
    qemu_fdt_setprop(fdt, nodename, "reg", reg_attr,
                     vbasedev->num_regions * 2 * sizeof(uint32_t));

    irq_attr = g_new(uint32_t, vbasedev->num_irqs * 3);
    for (i = 0; i < vbasedev->num_irqs; i++) {
        irq_number = platform_bus_get_irqn(pbus, sbdev , i)
                         + data->irq_start;
        irq_attr[3 * i] = cpu_to_be32(GIC_FDT_IRQ_TYPE_SPI);
        irq_attr[3 * i + 1] = cpu_to_be32(irq_number);
        irq_attr[3 * i + 2] = cpu_to_be32(GIC_FDT_IRQ_FLAGS_LEVEL_HI);
    }
    qemu_fdt_setprop(fdt, nodename, "interrupts",
                     irq_attr, vbasedev->num_irqs * 3 * sizeof(uint32_t));
    g_free(irq_attr);
    g_free(reg_attr);
    g_free(nodename);
    return 0;
}

/* AMD xgbe properties whose values are copied/pasted from host */
static HostProperty amd_xgbe_copied_properties[] = {
    {"compatible", false},
    {"dma-coherent", true},
    {"amd,per-channel-interrupt", true},
    {"phy-mode", false},
    {"mac-address", true},
    {"amd,speed-set", false},
    {"amd,serdes-blwc", true},
    {"amd,serdes-cdr-rate", true},
    {"amd,serdes-pq-skew", true},
    {"amd,serdes-tx-amp", true},
    {"amd,serdes-dfe-tap-config", true},
    {"amd,serdes-dfe-tap-enable", true},
    {"clock-names", false},
};

/**
 * add_amd_xgbe_fdt_node
 *
 * Generates the combined xgbe/phy node following kernel >=4.2
 * binding documentation:
 * Documentation/devicetree/bindings/net/amd-xgbe.txt:
 * Also 2 clock nodes are created (dma and ptp)
 *
 * Asserts in case of error
 */
static int add_amd_xgbe_fdt_node(SysBusDevice *sbdev, void *opaque,
                                 ProppertList *properties)
{
    PlatformBusFDTData *data = opaque;
    PlatformBusDevice *pbus = data->pbus;
    VFIOPlatformDevice *vdev = VFIO_PLATFORM_DEVICE(sbdev);
    VFIODevice *vbasedev = &vdev->vbasedev;
    VFIOINTp *intp;
    const char *parent_node = data->pbus_node_name;
    char **node_path, *nodename, *dt_name;
    void *guest_fdt = data->fdt, *host_fdt;
    const void *r;
    int i, prop_len;
    uint32_t *irq_attr, *reg_attr, *host_clock_phandles;
    uint64_t mmio_base, irq_number;
    uint32_t guest_clock_phandles[2];

    host_fdt = load_device_tree_from_sysfs();

    dt_name = sysfs_to_dt_name(vbasedev->name);
    if (!dt_name) {
        error_report("%s incorrect sysfs device name %s",
                     __func__, vbasedev->name);
        exit(1);
    }
    node_path = qemu_fdt_node_path(host_fdt, dt_name, vdev->compat,
                                   &error_fatal);
    if (!node_path || !node_path[0]) {
        error_report("%s unable to retrieve node path for %s/%s",
                     __func__, dt_name, vdev->compat);
        exit(1);
    }

    if (node_path[1]) {
        error_report("%s more than one node matching %s/%s!",
                     __func__, dt_name, vdev->compat);
        exit(1);
    }

    g_free(dt_name);

    if (vbasedev->num_regions != 5) {
        error_report("%s Does the host dt node combine XGBE/PHY?", __func__);
        exit(1);
    }

    /* generate nodes for DMA_CLK and PTP_CLK */
    r = qemu_fdt_getprop(host_fdt, node_path[0], "clocks",
                         &prop_len, &error_fatal);
    if (prop_len != 8) {
        error_report("%s clocks property should contain 2 handles", __func__);
        exit(1);
    }
    host_clock_phandles = (uint32_t *)r;
    guest_clock_phandles[0] = qemu_fdt_alloc_phandle(guest_fdt);
    guest_clock_phandles[1] = qemu_fdt_alloc_phandle(guest_fdt);

    /**
     * clock handles fetched from host dt are in be32 layout whereas
     * rest of the code uses cpu layout. Also guest clock handles are
     * in cpu layout.
     */
    fdt_build_clock_node(host_fdt, guest_fdt,
                         be32_to_cpu(host_clock_phandles[0]),
                         guest_clock_phandles[0]);

    fdt_build_clock_node(host_fdt, guest_fdt,
                         be32_to_cpu(host_clock_phandles[1]),
                         guest_clock_phandles[1]);

    /* combined XGBE/PHY node */
    mmio_base = platform_bus_get_mmio_addr(pbus, sbdev, 0);
    nodename = g_strdup_printf("%s/%s@%" PRIx64, parent_node,
                               vbasedev->name, mmio_base);
    qemu_fdt_add_subnode(guest_fdt, nodename);

    copy_properties_from_host(amd_xgbe_copied_properties,
                       ARRAY_SIZE(amd_xgbe_copied_properties),
                       host_fdt, guest_fdt,
                       node_path[0], nodename);

    qemu_fdt_setprop_cells(guest_fdt, nodename, "clocks",
                           guest_clock_phandles[0],
                           guest_clock_phandles[1]);

    reg_attr = g_new(uint32_t, vbasedev->num_regions * 2);
    for (i = 0; i < vbasedev->num_regions; i++) {
        mmio_base = platform_bus_get_mmio_addr(pbus, sbdev, i);
        reg_attr[2 * i] = cpu_to_be32(mmio_base);
        reg_attr[2 * i + 1] = cpu_to_be32(
                                memory_region_size(vdev->regions[i]->mem));
    }
    qemu_fdt_setprop(guest_fdt, nodename, "reg", reg_attr,
                     vbasedev->num_regions * 2 * sizeof(uint32_t));

    irq_attr = g_new(uint32_t, vbasedev->num_irqs * 3);
    for (i = 0; i < vbasedev->num_irqs; i++) {
        irq_number = platform_bus_get_irqn(pbus, sbdev , i)
                         + data->irq_start;
        irq_attr[3 * i] = cpu_to_be32(GIC_FDT_IRQ_TYPE_SPI);
        irq_attr[3 * i + 1] = cpu_to_be32(irq_number);
        /*
          * General device interrupt and PCS auto-negotiation interrupts are
          * level-sensitive while the 4 per-channel interrupts are edge
          * sensitive
          */
        QLIST_FOREACH(intp, &vdev->intp_list, next) {
            if (intp->pin == i) {
                break;
            }
        }
        if (intp->flags & VFIO_IRQ_INFO_AUTOMASKED) {
            irq_attr[3 * i + 2] = cpu_to_be32(GIC_FDT_IRQ_FLAGS_LEVEL_HI);
        } else {
            irq_attr[3 * i + 2] = cpu_to_be32(GIC_FDT_IRQ_FLAGS_EDGE_LO_HI);
        }
    }
    qemu_fdt_setprop(guest_fdt, nodename, "interrupts",
                     irq_attr, vbasedev->num_irqs * 3 * sizeof(uint32_t));

    g_free(host_fdt);
    g_strfreev(node_path);
    g_free(irq_attr);
    g_free(reg_attr);
    g_free(nodename);
    return 0;
}

static ProppertList mvrl_armada_ahci_properties[] = {
    { "name",            PROP_IGNORE }, /* handled automatically */
    { "phandle",         PROP_IGNORE }, /* not needed for the generic case */
    { "linux,phandle",   PROP_IGNORE }, /* not needed for the generic case */

    /* The following are copied and remapped by dedicated code */
    { "reg",             PROP_IGNORE },
    { "interrupts",      PROP_IGNORE },

    /* The following are handled by the host */
    { "power-domains",   PROP_IGNORE }, /* power management (+ opt. clocks) */
    { "iommus",          PROP_IGNORE }, /* isolation */
    { "resets",          PROP_IGNORE }, /* isolation */
    { "pinctrl-*",       PROP_IGNORE }, /* pin control */

    /* Ignoring the following may or may not work, hence the warning */
    { "gpio-ranges",     PROP_WARN },   /* no support for pinctrl yet */
    { "dmas",            PROP_WARN },   /* no support for external DMACs yet */

    /* The following are irrelevant, as corresponding specifiers are ignored */
    { "reset-names",     PROP_IGNORE },
    { "dma-names",       PROP_IGNORE },
    { "clocks",          PROP_IGNORE },
    { "phys",            PROP_IGNORE },
    { "phy-names",       PROP_IGNORE },
    { "usb-phy",         PROP_IGNORE },

    /* Whitelist of properties not taking phandles, and thus safe to copy */
    { "clock-names",     PROP_COPY },
    { "compatible",      PROP_COPY },
    { "status",          PROP_COPY },
    { "reg-names",       PROP_COPY },
    { "interrupt-names", PROP_COPY },
    { "#*-cells",        PROP_COPY },
    { "comreset_u",      PROP_COPY },
    { "comwake",         PROP_COPY },
    { "dma-coherent",    PROP_COPY },

    { NULL,              PROP_IGNORE }, /* last element */
};

static ProppertList mvrl_armada_xhci_properties[] = {
    { "name",            PROP_IGNORE }, /* handled automatically */
    { "phandle",         PROP_IGNORE }, /* not needed for the generic case */
    { "linux,phandle",   PROP_IGNORE }, /* not needed for the generic case */

    /* The following are copied and remapped by dedicated code */
    { "reg",             PROP_IGNORE },
    { "interrupts",      PROP_IGNORE },

    /* The following are handled by the host */
    { "power-domains",   PROP_IGNORE }, /* power management (+ opt. clocks) */
    { "iommus",          PROP_IGNORE }, /* isolation */
    { "resets",          PROP_IGNORE }, /* isolation */
    { "pinctrl-*",       PROP_IGNORE }, /* pin control */

    /* Ignoring the following may or may not work, hence the warning */
    { "gpio-ranges",     PROP_WARN },   /* no support for pinctrl yet */
    { "dmas",            PROP_WARN },   /* no support for external DMACs yet */

    /* The following are irrelevant, as corresponding specifiers are ignored */
    { "reset-names",     PROP_IGNORE },
    { "dma-names",       PROP_IGNORE },
    { "clocks",          PROP_IGNORE },
    { "phys",            PROP_IGNORE },
    { "phy-names",       PROP_IGNORE },
    { "usb-phy",         PROP_IGNORE },

    /* Whitelist of properties not taking phandles, and thus safe to copy */
    { "clock-names",     PROP_COPY },
    { "compatible",      PROP_COPY },
    { "status",          PROP_COPY },
    { "reg-names",       PROP_COPY },
    { "interrupt-names", PROP_COPY },
    { "#*-cells",        PROP_COPY },
    { "dma-coherent",    PROP_COPY },

    { NULL,              PROP_IGNORE }, /* last element */
};

static ProppertList mvrl_armada_sdhci_properties[] = {
    { "name",            PROP_IGNORE }, /* handled automatically */
    { "phandle",         PROP_IGNORE }, /* not needed for the generic case */
    { "linux,phandle",   PROP_IGNORE }, /* not needed for the generic case */

    /* The following are copied and remapped by dedicated code */
    { "reg",             PROP_IGNORE },
    { "interrupts",      PROP_IGNORE },
    { "clocks",          PROP_IGNORE },
    { "*-supply",        PROP_IGNORE },

    /* The following are handled by the host */
    { "power-domains",   PROP_IGNORE }, /* power management (+ opt. clocks) */
    { "iommus",          PROP_IGNORE }, /* isolation */
    { "resets",          PROP_IGNORE }, /* isolation */
    { "pinctrl-*",       PROP_IGNORE }, /* pin control */

    /* Ignoring the following may or may not work, hence the warning */
    { "gpio-ranges",     PROP_WARN },   /* no support for pinctrl yet */
    { "dmas",            PROP_WARN },   /* no support for external DMACs yet */

    /* The following are irrelevant, as corresponding specifiers are ignored */
    { "reset-names",     PROP_IGNORE },
    { "dma-names",       PROP_IGNORE },
    { "phys",            PROP_IGNORE },
    { "phy-names",       PROP_IGNORE },
    { "usb-phy",         PROP_IGNORE },

    /* Whitelist of properties not taking phandles, and thus safe to copy */
    { "clock-names",     PROP_COPY },
    { "compatible",      PROP_COPY },
    { "status",          PROP_COPY },
    { "reg-names",       PROP_COPY },
    { "interrupt-names", PROP_COPY },
    { "#*-cells",        PROP_COPY },
    { "dma-coherent",    PROP_COPY },

    { "no-1-8-v",        PROP_COPY },
    { "broken-cd",       PROP_COPY },
    { "bus-width",       PROP_COPY },

    { NULL,              PROP_IGNORE },  /* last element */
};

#ifndef CONFIG_FNMATCH
/* Fall back to exact string matching instead of allowing wildcards */
static inline int fnmatch(const char *pattern, const char *string, int flags)
{
        return strcmp(pattern, string);
}
#endif

static enum GenericPropertyAction get_property_action(const char *name,
                                                      ProppertList *properties)
{
    unsigned int i;

    if (!fnmatch("dont-probe", name, 0)) {
        return PROP_IGNORE;
    }

    for (i = 0; properties[i].name; i++) {
        if (!fnmatch(properties[i].name, name, 0)) {
            return properties[i].action;
        }
    }

    /*
     * Unfortunately DT properties do not carry type information,
     * so we have to assume everything else contains a phandle,
     * and must be rejected
     */
    return PROP_REJECT;
}

/**
 * copy_node_prop
 *
 * Copy the listed properties from the host DT node
 */
static void copy_node_prop(void *host_fdt, void *guest_fdt, char *host_path,
                              char *guest_path, ProppertList *properties)
{
    int node, prop, len;
    const void *data;
    const char *name;
    enum GenericPropertyAction action;

    node = fdt_path_offset(host_fdt, host_path);
    if (node < 0) {
        error_report("Cannot find node %s: %s", host_path, fdt_strerror(node));
        exit(1);
    }

    /* MICAH this is the child node not supported thing */
    /* Subnodes are not yet supported, ignore */
    if (fdt_first_subnode(host_fdt, node) >= 0) {
        warn_report("%s has unsupported subnodes", host_path);
    }

    /* Copy properties */
    fdt_for_each_property_offset(prop, host_fdt, node) {
        data = fdt_getprop_by_offset(host_fdt, prop, &name, &len);
        if (!data) {
            error_report("Cannot get property of %s: %s", host_path,
                         fdt_strerror(len));
            exit(1);
        }

        if (!len) {
            /* Zero-sized properties are safe to copy */
            action = PROP_COPY;
        } else {
            action = get_property_action(name, properties);
        }

        switch (action) {
        case PROP_WARN:
            warn_report("%s: Ignoring %s property", host_path, name);
        case PROP_IGNORE:
            error_report("%s name %s ignored", __func__, name);

            break;

        case PROP_COPY:
            qemu_fdt_setprop(guest_fdt, guest_path, name, data, len);

            error_report("%s name %s copied", __func__, name);

            break;

        case PROP_REJECT:
            error_report("%s has unsupported %s property", host_path, name);
            goto unsupported;
        }
    }

//    qemu_fdt_setprop(guest_fdt, guest_path, "no-1-8-v", NULL, 0);
//    error_report("%s name %s copied", __func__, "no-1-8-v");

    return;

unsupported:
    exit(1);
}

static void copy_host_node_prop(VFIOPlatformDevice *vdev, void *fdt_guest,
                                char *node_name, ProppertList *properties)
{
    VFIODevice *vbasedev = &vdev->vbasedev;
    char *tmp, sysfs_node_path[PATH_MAX], *root_node_path;
    char path_delimit[] = "devicetree/base";
    char **fdt_path_array, *fdt_path_final = NULL;
    void *fdt_host;
    ssize_t len;
    int i = 0;

    /*
     * It is perfectly legal to have two nodes with the same name and
     * compatible, therefore for lookup we need to deal with full DTS path.
     */
    tmp = g_strdup_printf("%s/of_node", vbasedev->sysfsdev);
    len = readlink(tmp, sysfs_node_path, sizeof(sysfs_node_path));
    g_free(tmp);

    if (len < 0 || len >= sizeof(sysfs_node_path)) {
        error_report("no of_node found for %s", vdev->compat);
        exit(1);
    }

    sysfs_node_path[len] = 0;
    root_node_path = strstr(sysfs_node_path, path_delimit);
    /* Skip delimiter to have root based DTS path */
    root_node_path += strlen(path_delimit);

    fdt_host = load_device_tree_from_sysfs();
    fdt_path_array = qemu_fdt_node_path(fdt_host, NULL, vdev->compat,
                                   &error_fatal);

    while (fdt_path_array[i]) {
        if (strncmp(fdt_path_array[i], root_node_path,
                    strlen(root_node_path)) == 0) {
            fdt_path_final = fdt_path_array[i];
            break;
        }
        i++;
    }

    if (!fdt_path_final) {
        error_report("%s %s node not found!", __func__, root_node_path);
        exit(1);
    }

    /* Copy whitelist properties from host */
    copy_node_prop(fdt_host, fdt_guest, fdt_path_final, node_name, properties);

    g_strfreev(fdt_path_array);
    g_free(fdt_host);
}

static void fdt_virtio_realize(PlatformBusFDTData *data,
                                   VFIOPlatformDevice *vdev, int index,
                                   uint64_t *mmio_base, uint64_t *mmio_size,
                                   int *irq_number,
                                   uint64_t virtio_id)
{
    PlatformBusDevice *pbus = data->pbus;
    DeviceState *dev, *proxy_dev;
    VirtIOMMIOProxy *proxy;
    static int instance;
    char *clk_dev_name;
    MemoryRegion *mr;

    /* Allocate resources dynamically under platform bus. */
    proxy_dev = qdev_create(NULL, "virtio-mmio");
    qdev_init_nofail(proxy_dev);
    proxy = VIRTIO_MMIO(proxy_dev);
    platform_bus_link_device(pbus, SYS_BUS_DEVICE(proxy_dev));

    dev = DEVICE(object_new("vhost-vfio-platform"));
    qdev_set_parent_bus(dev, BUS(&proxy->bus));

    clk_dev_name = g_strdup_printf("%s-%" PRIx32, "clk-virtio-mmio", instance++);
    object_property_set_str(OBJECT(dev), clk_dev_name , "name", &error_abort);
    object_property_set_uint(OBJECT(dev), virtio_id , "device_id", &error_abort);
    object_property_set_link(OBJECT(dev), OBJECT(vdev), "dev-client", &error_abort);
    object_property_set_uint(OBJECT(dev), index , "index", &error_abort);
    qdev_init_nofail(dev);

    mr = sysbus_mmio_get_region(SYS_BUS_DEVICE(proxy), 0);
    *mmio_base = object_property_get_uint(OBJECT(mr), "addr", NULL);
    *mmio_size = memory_region_size(mr);
    *irq_number = platform_bus_get_irqn(pbus, SYS_BUS_DEVICE(proxy_dev), 0) +
                                                              data->irq_start;
    // MICAH should print number here to see whats going on
}

static void fdt_build_virtio_clk_mmio_node(PlatformBusFDTData *data,
                                          VFIOPlatformDevice *vdev,
                                          int index,
                                          void *guest_fdt,
                                          uint32_t guest_clk_phandle)
{
    uint64_t mmio_base, mmio_size;
    uint32_t reg_attr[2];
    char *clk_node_name;
    int irq;

    fdt_virtio_realize(data, vdev, index, &mmio_base, &mmio_size, &irq,
                           VIRTIO_ID_CLK);

    clk_node_name = g_strdup_printf("%s/%s@%" PRIx64, platform_bus_node,
                                    "virtio_mmio", mmio_base);
    qemu_fdt_add_subnode(guest_fdt, clk_node_name);
    qemu_fdt_setprop_string(guest_fdt, clk_node_name, "compatible",
                            "virtio,mmio");
    qemu_fdt_setprop_cells(guest_fdt, clk_node_name, "#clock-cells", 0);
    qemu_fdt_setprop_string(guest_fdt, clk_node_name, "clock-output-names",
                            "mmio_clock_output");
    // MICAH so we do a line like this when we want a node to have a phandle i guess
    qemu_fdt_setprop_cell(guest_fdt, clk_node_name, "phandle",
                          guest_clk_phandle);

    reg_attr[0] = cpu_to_be32(mmio_base);
    reg_attr[1] = cpu_to_be32(mmio_size);
    qemu_fdt_setprop(guest_fdt, clk_node_name, "reg", reg_attr,
                     2 * sizeof(uint32_t));
    qemu_fdt_setprop_cells(guest_fdt, clk_node_name, "interrupts",
                           GIC_FDT_IRQ_TYPE_SPI, irq,
                           GIC_FDT_IRQ_FLAGS_EDGE_LO_HI);
    // MICAH or could just look at the DT node that gets created
}

static void fdt_build_virtio_phy_mmio_node(PlatformBusFDTData *data,
                                           VFIOPlatformDevice *vdev,
                                           int index,
                                           void *guest_fdt,
                                           uint32_t guest_clk_phandle)
{
    uint64_t mmio_base, mmio_size;
    uint32_t reg_attr[2];
    char *clk_node_name;
    int irq;

    fdt_virtio_realize(data, vdev, index, &mmio_base, &mmio_size, &irq,
                           VIRTIO_ID_PHY);

    clk_node_name = g_strdup_printf("%s/%s@%" PRIx64, platform_bus_node,
                                    "virtio_mmio", mmio_base);
    qemu_fdt_add_subnode(guest_fdt, clk_node_name);
    qemu_fdt_setprop_string(guest_fdt, clk_node_name, "compatible",
                            "virtio,mmio");
    qemu_fdt_setprop_cells(guest_fdt, clk_node_name, "#phy-cells", 0);
    qemu_fdt_setprop_cell(guest_fdt, clk_node_name, "phandle",
                          guest_clk_phandle);

    reg_attr[0] = cpu_to_be32(mmio_base);
    reg_attr[1] = cpu_to_be32(mmio_size);
    qemu_fdt_setprop(guest_fdt, clk_node_name, "reg", reg_attr,
                     2 * sizeof(uint32_t));
    qemu_fdt_setprop_cells(guest_fdt, clk_node_name, "interrupts",
                           GIC_FDT_IRQ_TYPE_SPI, irq,
                           GIC_FDT_IRQ_FLAGS_EDGE_LO_HI);
}

static void fdt_build_virtio_pinctrl_mmio_node(PlatformBusFDTData *data,
                                           VFIOPlatformDevice *vdev,
                                           int index,
                                           void *guest_fdt,
                                           uint32_t guest_clk_phandle)
{
    uint64_t mmio_base, mmio_size;
    uint32_t reg_attr[2];
    char *clk_node_name;
    int irq;

    fdt_virtio_realize(data, vdev, index, &mmio_base, &mmio_size, &irq,
                       VIRTIO_ID_PINCTRL);

    clk_node_name = g_strdup_printf("%s/%s@%" PRIx64, platform_bus_node,
                                    "virtio_mmio", mmio_base);
    qemu_fdt_add_subnode(guest_fdt, clk_node_name);
    qemu_fdt_setprop_string(guest_fdt, clk_node_name, "compatible",
                            "virtio,mmio");

    reg_attr[0] = cpu_to_be32(mmio_base);
    reg_attr[1] = cpu_to_be32(mmio_size);
    qemu_fdt_setprop(guest_fdt, clk_node_name, "reg", reg_attr,
                     2 * sizeof(uint32_t));
    qemu_fdt_setprop_cells(guest_fdt, clk_node_name, "interrupts",
                           GIC_FDT_IRQ_TYPE_SPI, irq,
                           GIC_FDT_IRQ_FLAGS_EDGE_LO_HI);

    /* State subnode */
    clk_node_name = g_strdup_printf("%s/state_0", clk_node_name);
    qemu_fdt_add_subnode(guest_fdt, clk_node_name);
    qemu_fdt_setprop_string(guest_fdt, clk_node_name, "pins", "gpio0");
    qemu_fdt_setprop(guest_fdt, clk_node_name, "bias-disable", "", 0);
    qemu_fdt_setprop_cell(guest_fdt, clk_node_name, "phandle",
                          guest_clk_phandle);
}

static void fdt_build_virtio_inter_mmio_node(PlatformBusFDTData *data,
                                            VFIOPlatformDevice *vdev,
                                            int index,
                                            void *guest_fdt,
                                            uint32_t guest_clk_phandle)
{
    uint64_t mmio_base, mmio_size;
    uint32_t reg_attr[2];
    char *clk_node_name;
    int irq;

    fdt_virtio_realize(data, vdev, index, &mmio_base, &mmio_size, &irq,
                       VIRTIO_ID_INTERCONNECT);

    clk_node_name = g_strdup_printf("%s/%s@%" PRIx64, platform_bus_node,
                                    "virtio_mmio", mmio_base);
    qemu_fdt_add_subnode(guest_fdt, clk_node_name);
    qemu_fdt_setprop_string(guest_fdt, clk_node_name, "compatible",
                            "virtio,mmio");
    qemu_fdt_setprop_cells(guest_fdt, clk_node_name, "#interconnect-cells", 1);
    qemu_fdt_setprop_cell(guest_fdt, clk_node_name, "phandle",
                          guest_clk_phandle);

    reg_attr[0] = cpu_to_be32(mmio_base);
    reg_attr[1] = cpu_to_be32(mmio_size);
    qemu_fdt_setprop(guest_fdt, clk_node_name, "reg", reg_attr,
                     2 * sizeof(uint32_t));
    qemu_fdt_setprop_cells(guest_fdt, clk_node_name, "interrupts",
                           GIC_FDT_IRQ_TYPE_SPI, irq,
                           GIC_FDT_IRQ_FLAGS_EDGE_LO_HI);
}

static ProppertList mvrl_armada_sdhci_regulator_properties[] = {
    { "compatible",              PROP_IGNORE },
    { "phandle",                 PROP_IGNORE },

    { "name",                    PROP_COPY },
    { "regulator-always-on",     PROP_COPY },
    { "regulator-max-microvolt", PROP_COPY },
    { "regulator-min-microvolt", PROP_COPY },
    { "regulator-name",          PROP_COPY },
    { "status",                  PROP_COPY },

    { NULL,                      PROP_IGNORE },  /* last element */
};

static void copy_host_regulator_node_prop(char *sysfsdev,
                                          void *fdt_guest,
                                          char *node_name,
                                          ProppertList *allowed_props)
{
    char *tmp, sysfs_node_path[PATH_MAX], *root_node_path;
    char path_delimit[] = "devicetree/base";
    void *fdt_host;
    ssize_t len;

    error_report("%s sysfsdev %s", __func__, sysfsdev);

    /*
     * It is perfectly legal to have two nodes with the same name and
     * compatible, therefore for lookup we need to deal with full DTS path.
     */
    tmp = g_strdup_printf("%s/of_node", sysfsdev);
    len = readlink(tmp, sysfs_node_path, sizeof(sysfs_node_path));
    g_free(tmp);

    if (len < 0 || len >= sizeof(sysfs_node_path)) {
        error_report("no of_node found for %s", sysfsdev);
        exit(1);
    }

    error_report("%s sysfs_node_path %s",
                         __func__, sysfs_node_path);

    sysfs_node_path[len] = 0;
    root_node_path = strstr(sysfs_node_path, path_delimit);
    /* Skip delimiter to have root based DTS path */
    root_node_path += strlen(path_delimit);

    error_report("%s root_node_path %s",
                         __func__, root_node_path);

    fdt_host = load_device_tree_from_sysfs();

    error_report("\n\n\n\n %s -------------------------", __func__);

    /* Copy whitelist properties from host */
    copy_node_prop(fdt_host, fdt_guest, root_node_path, node_name,
                   allowed_props);

    error_report("\n\n\n\n %s -------------------------", __func__);

    g_free(fdt_host);
}

static void fdt_build_virtio_regulator_mmio_node(PlatformBusFDTData *data,
                                                 VFIOPlatformDevice *vdev,
                                                 int index,
                                                 void *guest_fdt,
                                                 uint32_t guest_phandle,
                                                 char *regulator_name,
                                                 ProppertList *allowed_props)
{
    VFIODevice *vbasedev = &vdev->vbasedev;
    uint64_t mmio_base, mmio_size;
    uint32_t reg_attr[2];
    char *regulator_node_name;
    char *regulator_node_path;
    int irq;

    fdt_virtio_realize(data, vdev, index, &mmio_base, &mmio_size, &irq,
                       VIRTIO_ID_REGULATOR);

    regulator_node_name = g_strdup_printf("%s/%s@%" PRIx64, platform_bus_node,
                                    "virtio_mmio", mmio_base);
    qemu_fdt_add_subnode(guest_fdt, regulator_node_name);
    qemu_fdt_setprop_string(guest_fdt, regulator_node_name, "compatible",
                            "virtio,mmio");
    qemu_fdt_setprop_cell(guest_fdt, regulator_node_name, "phandle",
                          guest_phandle);

    reg_attr[0] = cpu_to_be32(mmio_base);
    reg_attr[1] = cpu_to_be32(mmio_size);
    qemu_fdt_setprop(guest_fdt, regulator_node_name, "reg", reg_attr,
                     2 * sizeof(uint32_t));
    qemu_fdt_setprop_cells(guest_fdt, regulator_node_name, "interrupts",
                           GIC_FDT_IRQ_TYPE_SPI, irq,
                           GIC_FDT_IRQ_FLAGS_EDGE_LO_HI);

    regulator_node_path = g_strdup_printf("%s/%s-%s", vbasedev->sysfsdev,
                                          vbasedev->name, regulator_name);

    copy_host_regulator_node_prop(regulator_node_path, guest_fdt,
                                  regulator_node_name, allowed_props);
}

static void fdt_build_virtio_gpio_mmio_node(PlatformBusFDTData *data,
                                            VFIOPlatformDevice *vdev,
                                            int index,
                                            void *guest_fdt,
                                            uint32_t phandle)
{
    uint64_t mmio_base, mmio_size;
    uint32_t reg_attr[2];
    char *node_name;
    int irq;

    fdt_virtio_realize(data, vdev, index, &mmio_base, &mmio_size, &irq,
                       VIRTIO_ID_PINCTRL);

    node_name = g_strdup_printf("%s/%s@%" PRIx64, platform_bus_node,
                                    "virtio_mmio", mmio_base);
    qemu_fdt_add_subnode(guest_fdt, node_name);
    qemu_fdt_setprop_string(guest_fdt, node_name, "compatible",
                            "virtio,mmio");
    qemu_fdt_setprop(guest_fdt, node_name, "gpio-controller", "", 0);
    qemu_fdt_setprop_cells(guest_fdt, node_name, "#gpio-cells", 2);
    qemu_fdt_setprop_cell(guest_fdt, node_name, "phandle", phandle);

    reg_attr[0] = cpu_to_be32(mmio_base);
    reg_attr[1] = cpu_to_be32(mmio_size);
    qemu_fdt_setprop(guest_fdt, node_name, "reg", reg_attr,
                     2 * sizeof(uint32_t));
    qemu_fdt_setprop_cells(guest_fdt, node_name, "interrupts",
                           GIC_FDT_IRQ_TYPE_SPI, irq,
                           GIC_FDT_IRQ_FLAGS_EDGE_LO_HI);
}

static int add_mvrl_armada_fdt_node(SysBusDevice *sbdev, void *opaque,
                                   ProppertList *properties)
{
    PlatformBusFDTData *data = opaque;
    const char *parent_node = data->pbus_node_name;
    PlatformBusDevice *pbus = data->pbus;
    void *fdt_guest = data->fdt;
    VFIOPlatformDevice *vdev = VFIO_PLATFORM_DEVICE(sbdev);
    VFIODevice *vbasedev = &vdev->vbasedev;
    uint32_t *reg_attr, *irq_attr, *guest_clk_phandle;
    char *node_name;
    uint64_t mmio_base;
    int i, irq_number;
    VFIOINTp *intp;

    mmio_base = platform_bus_get_mmio_addr(pbus, sbdev, 0);
    node_name = g_strdup_printf("%s/%s@%" PRIx64, parent_node,
                                vbasedev->name, mmio_base);

    qemu_fdt_add_subnode(fdt_guest, node_name);
    copy_host_node_prop(vdev, fdt_guest, node_name, properties);

    /* Copy reg (remapped) */
    reg_attr = g_new(uint32_t, vbasedev->num_regions * 2);
    for (i = 0; i < vbasedev->num_regions; i++) {
        mmio_base = platform_bus_get_mmio_addr(pbus, sbdev, i);
        reg_attr[2 * i] = cpu_to_be32(mmio_base);
        reg_attr[2 * i + 1] = cpu_to_be32(
                                memory_region_size(vdev->regions[i]->mem));
    }
    qemu_fdt_setprop(fdt_guest, node_name, "reg", reg_attr,
                     vbasedev->num_regions * 2 * sizeof(uint32_t));

    /* Copy interrupts (remapped) */
    if (vbasedev->num_irqs) {
        irq_attr = g_new(uint32_t, vbasedev->num_irqs * 3);
        for (i = 0; i < vbasedev->num_irqs; i++) {
            irq_number = platform_bus_get_irqn(pbus, sbdev, i) +
                         data->irq_start;
            irq_attr[3 * i] = cpu_to_be32(GIC_FDT_IRQ_TYPE_SPI);
            irq_attr[3 * i + 1] = cpu_to_be32(irq_number);
            QLIST_FOREACH(intp, &vdev->intp_list, next) {
                if (intp->pin == i) {
                    break;
                }
            }
            if (intp->flags & VFIO_IRQ_INFO_AUTOMASKED) {
                irq_attr[3 * i + 2] = cpu_to_be32(GIC_FDT_IRQ_FLAGS_LEVEL_HI);
            } else {
                irq_attr[3 * i + 2] = cpu_to_be32(GIC_FDT_IRQ_FLAGS_EDGE_LO_HI);
            }
        }
        qemu_fdt_setprop(fdt_guest, node_name, "interrupts",
                         irq_attr, vbasedev->num_irqs * 3 * sizeof(uint32_t));
        g_free(irq_attr);
    }

    guest_clk_phandle = g_new(uint32_t, vbasedev->num_clks);
    for (i = 0; i < vbasedev->num_clks;  i++) {
        guest_clk_phandle[i] = qemu_fdt_alloc_phandle(fdt_guest);
        fdt_build_virtio_clk_mmio_node(data, vdev, i, fdt_guest,
                                       guest_clk_phandle[i]);
        guest_clk_phandle[i] = cpu_to_be32(guest_clk_phandle[i]);
    }
    if (vbasedev->num_clks > 0) {
        qemu_fdt_setprop(fdt_guest, node_name, "clocks", guest_clk_phandle,
                         vbasedev->num_clks * sizeof(uint32_t));
    }
    g_free(guest_clk_phandle);

    guest_clk_phandle = g_new(uint32_t, vbasedev->num_regulators);
    for (i = 0; i < vbasedev->num_regulators;  i++) {
        char *supply_name;

        guest_clk_phandle[i] = qemu_fdt_alloc_phandle(fdt_guest);
        fdt_build_virtio_regulator_mmio_node(data, vdev, i, fdt_guest,
                                             guest_clk_phandle[i],
                                             vbasedev->regulator_names[i],
                                             mvrl_armada_sdhci_regulator_properties);
        supply_name = g_strdup_printf("%s-supply",
                                      vbasedev->regulator_names[i]);

        error_report("%s copying properties of regulator name %s",
                     __func__, supply_name);

        qemu_fdt_setprop_cell(fdt_guest, node_name, supply_name,
                              guest_clk_phandle[i]);
    }

    g_free(guest_clk_phandle);
    g_free(reg_attr);
    g_free(node_name);
    return 0;
}

static ProppertList qcom_trogdor_wifi_properties[] = {
    { "name",            PROP_IGNORE }, /* handled automatically */
    { "phandle",         PROP_IGNORE }, /* not needed for the generic case */
    { "linux,phandle",   PROP_IGNORE }, /* not needed for the generic case */

    /* The following are copied and remapped by dedicated code */
    { "reg",             PROP_IGNORE },
    { "interrupts",      PROP_IGNORE },
    { "clocks",          PROP_IGNORE },
    { "*-supply",        PROP_IGNORE },

    /* The following are handled by the host */
    { "power-domains",   PROP_IGNORE }, /* power management (+ opt. clocks) */
    { "iommus",          PROP_IGNORE }, /* isolation */
    { "resets",          PROP_IGNORE }, /* isolation */
    { "pinctrl-*",       PROP_IGNORE }, /* pin control */

    /* Ignoring the following may or may not work, hence the warning */
    { "gpio-ranges",     PROP_WARN },   /* no support for pinctrl yet */
    { "dmas",            PROP_WARN },   /* no support for external DMACs yet */

    /* The following are irrelevant, as corresponding specifiers are ignored */
    { "reset-names",     PROP_IGNORE },
    { "dma-names",       PROP_IGNORE },

    /* Qualcomm Wifi specific */
    { "memory-region",   PROP_IGNORE },

    /* Whitelist of properties not taking phandles, and thus safe to copy */
    { "clock-names",     PROP_COPY },
    { "compatible",      PROP_COPY },
    { "status",          PROP_COPY },
    { "reg-names",       PROP_COPY },
    { "interrupt-names", PROP_COPY },
    { "#*-cells",        PROP_COPY },
    { "dma-coherent",    PROP_COPY },

    /* Qualcomm Wifi specific */
    { "local-mac-address",      PROP_COPY },
    { "qcom,msa-fixed-perm",    PROP_COPY },

    { NULL,              PROP_IGNORE },  /* last element */
};

static ProppertList qcom_trogdor_sdhci_properties[] = {
    { "name",            PROP_IGNORE }, /* handled automatically */
    { "phandle",         PROP_IGNORE }, /* not needed for the generic case */
    { "linux,phandle",   PROP_IGNORE }, /* not needed for the generic case */

    /* The following are copied and remapped by dedicated code */
    { "reg",             PROP_IGNORE },
    { "interrupts",      PROP_IGNORE },
    { "clocks",          PROP_IGNORE },
    { "*-supply",        PROP_IGNORE },

    /* The following are handled by the host */
    { "power-domains",   PROP_IGNORE }, /* power management (+ opt. clocks) */
    { "iommus",          PROP_IGNORE }, /* isolation */
    { "resets",          PROP_IGNORE }, /* isolation */
    { "pinctrl-names",   PROP_COPY },
    { "pinctrl-*",       PROP_IGNORE }, /* pin control */

    /* Ignoring the following may or may not work, hence the warning */
    { "gpio-ranges",     PROP_WARN },   /* no support for pinctrl yet */
    { "dmas",            PROP_WARN },   /* no support for external DMACs yet */

    /* The following are irrelevant, as corresponding specifiers are ignored */
    { "reset-names",     PROP_IGNORE },
    { "dma-names",       PROP_IGNORE },

    /* Qualcomm SDHCI specific */
    { "operating-points-v2",   PROP_IGNORE },

    /* Whitelist of properties not taking phandles, and thus safe to copy */
    { "clock-names",     PROP_COPY },
    { "compatible",      PROP_COPY },
    { "status",          PROP_COPY },
    { "reg-names",       PROP_COPY },
    { "interrupt-names", PROP_COPY },
    { "#*-cells",        PROP_COPY },
    { "dma-coherent",    PROP_COPY },

    /* Qualcomm Wifi specific */
    { "bus-width",       PROP_COPY },
    { "non-removable",   PROP_COPY },
    {"supports-cqe",     PROP_COPY },
    {"mmc-ddr-1_8v",     PROP_COPY },
    {"mmc-hs200-1_8v",   PROP_COPY },
    {"mmc-hs400-1_8v",   PROP_COPY },
    {"mmc-hs400-enhanced-strobe", PROP_COPY },

    { NULL,              PROP_IGNORE },  /* last element */
};

static ProppertList qcom_trogdor_parent_xhci_properties[] = {
    { "name",            PROP_IGNORE }, /* handled automatically */
    { "phandle",         PROP_IGNORE }, /* not needed for the generic case */
    { "linux,phandle",   PROP_IGNORE }, /* not needed for the generic case */

    /* The following are copied and remapped by dedicated code */
    { "reg",             PROP_IGNORE },
    { "interrupts",      PROP_IGNORE },
    { "interrupts-extended",      PROP_IGNORE },
    { "clocks",          PROP_IGNORE },
    { "*-supply",        PROP_IGNORE },

    /* The following are handled by the host */
    { "power-domains",   PROP_IGNORE }, /* power management (+ opt. clocks) */
    { "iommus",          PROP_IGNORE }, /* isolation */
    { "resets",          PROP_IGNORE }, /* isolation */
    { "pinctrl-*",       PROP_IGNORE }, /* pin control */

    /* Ignoring the following may or may not work, hence the warning */
    { "gpio-ranges",     PROP_WARN },   /* no support for pinctrl yet */
    { "dmas",            PROP_WARN },   /* no support for external DMACs yet */

    /* The following are irrelevant, as corresponding specifiers are ignored */
    { "reset-names",     PROP_IGNORE },
    { "dma-names",       PROP_IGNORE },
    { "phys",            PROP_IGNORE },
    { "phy-names",       PROP_IGNORE },
    { "interconnects",   PROP_IGNORE },
    { "assigned-clocks", PROP_IGNORE },

    /* Whitelist of properties not taking phandles, and thus safe to copy */
    { "clock-names",     PROP_COPY },
    { "compatible",      PROP_COPY },
    { "status",          PROP_COPY },
    { "reg-names",       PROP_COPY },
    { "interrupt-names", PROP_COPY },
    { "#*-cells",        PROP_COPY },
    { "dma-coherent",    PROP_COPY },
    { "interconnect-names", PROP_COPY },

    /* Qualcomm XHCI specific */
    { "ranges",                 PROP_COPY },
    { "dma-ranges",             PROP_COPY },
    { "assigned-clock-rates",   PROP_COPY },

    { NULL,              PROP_IGNORE },  /* last element */
};

static ProppertList qcom_trogdor_panel_properties[] = {
    { "name",            PROP_IGNORE }, /* handled automatically */
    { "phandle",         PROP_IGNORE }, /* not needed for the generic case */
    { "linux,phandle",   PROP_IGNORE }, /* not needed for the generic case */

    { "*-supply",        PROP_IGNORE },
    { "*-gpios",         PROP_IGNORE },
    { "backlight",       PROP_IGNORE },

    { "compatible",      PROP_COPY },

    { NULL,              PROP_IGNORE },  /* last element */
};

static ProppertList qcom_trogdor_backlight_properties[] = {
    { "name",            PROP_IGNORE }, /* handled automatically */
    { "phandle",         PROP_IGNORE }, /* not needed for the generic case */
    { "linux,phandle",   PROP_IGNORE }, /* not needed for the generic case */

    { "pinctrl-names",   PROP_COPY },
    { "pinctrl-*",       PROP_IGNORE }, /* pin control */
    { "*-supply",        PROP_IGNORE },
    { "*-gpios",         PROP_IGNORE },
    { "pwms",            PROP_IGNORE },

    { "compatible",                 PROP_COPY },
    { "brightness-levels",          PROP_COPY },
    { "num-interpolated-steps",     PROP_COPY },
    { "default-brightness-level",   PROP_COPY },

    { NULL,              PROP_IGNORE },  /* last element */
};

static ProppertList qcom_trogdor_audio_codec_properties[] = {
    { "name",            PROP_IGNORE }, /* handled automatically */
    { "phandle",         PROP_IGNORE }, /* not needed for the generic case */
    { "linux,phandle",   PROP_IGNORE }, /* not needed for the generic case */

    { "pinctrl-names",   PROP_COPY },
    { "pinctrl-*",       PROP_IGNORE }, /* pin control */
    { "*-gpios",         PROP_IGNORE },

    { "compatible",      PROP_COPY },
    { "#sound-dai-cells", PROP_COPY },

    { NULL,              PROP_IGNORE },  /* last element */
};

static ProppertList qcom_trogdor_xhci_properties[] = {
    { "name",            PROP_IGNORE }, /* handled automatically */
    { "phandle",         PROP_IGNORE }, /* not needed for the generic case */
    { "linux,phandle",   PROP_IGNORE }, /* not needed for the generic case */

    /* The following are copied and remapped by dedicated code */
    { "reg",             PROP_IGNORE },
    { "interrupts",      PROP_IGNORE },
    { "clocks",          PROP_IGNORE },
    { "*-supply",        PROP_IGNORE },

    /* The following are handled by the host */
    { "power-domains",   PROP_IGNORE }, /* power management (+ opt. clocks) */
    { "iommus",          PROP_IGNORE }, /* isolation */
    { "resets",          PROP_IGNORE }, /* isolation */
    { "pinctrl-*",       PROP_IGNORE }, /* pin control */

    /* Ignoring the following may or may not work, hence the warning */
    { "gpio-ranges",     PROP_WARN },   /* no support for pinctrl yet */
    { "dmas",            PROP_WARN },   /* no support for external DMACs yet */

    /* The following are irrelevant, as corresponding specifiers are ignored */
    { "reset-names",     PROP_IGNORE },
    { "dma-names",       PROP_IGNORE },
    { "phys",            PROP_IGNORE },

    /* Whitelist of properties not taking phandles, and thus safe to copy */
    { "compatible",      PROP_COPY },
    { "status",          PROP_COPY },
    { "reg-names",       PROP_COPY },
    { "interrupt-names", PROP_COPY },
    { "#*-cells",        PROP_COPY },
    { "dma-coherent",    PROP_COPY },
    { "phy-names",       PROP_COPY },

    /* Qualcomm Wifi specific */
    { "snps,dis_u2_susphy_quirk", PROP_COPY },
    { "snps,dis_enblslpm_quirk",  PROP_COPY },
    { "maximum-speed",            PROP_COPY },
    { "dr_mode",                  PROP_COPY },

    { NULL,              PROP_IGNORE },  /* last element */
};

static ProppertList qcom_trogdor_gpu_properties[] = {
    { "name",            PROP_IGNORE }, /* handled automatically */
    { "phandle",         PROP_IGNORE }, /* not needed for the generic case */
    { "linux,phandle",   PROP_IGNORE }, /* not needed for the generic case */

    { "reg",             PROP_IGNORE },
    { "interrupts",      PROP_IGNORE },
    { "iommus",          PROP_IGNORE }, /* isolation */
    { "operating-points-v2", PROP_IGNORE },
    { "qcom,gmu",        PROP_IGNORE },
    { "interconnects",   PROP_IGNORE },
    { "#stream-id-cells", PROP_IGNORE },

    { "compatible",      PROP_COPY },
    { "reg-names",       PROP_COPY },
    { "interconnect-names", PROP_COPY },

    { NULL,              PROP_IGNORE },  /* last element */
};

static ProppertList qcom_trogdor_gmu_properties[] = {
    { "name",            PROP_IGNORE }, /* handled automatically */
    { "phandle",         PROP_IGNORE }, /* not needed for the generic case */
    { "linux,phandle",   PROP_IGNORE }, /* not needed for the generic case */

    /* The following are copied and remapped by dedicated code */
    { "reg",             PROP_IGNORE },
    { "interrupts",      PROP_IGNORE },
    { "clocks",          PROP_IGNORE },
    { "power-domain*",   PROP_IGNORE }, /* power management (+ opt. clocks) */
    { "iommus",          PROP_IGNORE }, /* isolation */
    { "operating-points-v2",   PROP_IGNORE },


    { "compatible",      PROP_COPY },
    { "reg-names",       PROP_COPY },
    { "interrupt-names", PROP_COPY },
    { "clock-names",     PROP_COPY },

    { NULL,              PROP_IGNORE },  /* last element */
};

static ProppertList qcom_trogdor_geni_properties[] = {
    { "name",            PROP_IGNORE }, /* handled automatically */
    { "phandle",         PROP_IGNORE }, /* not needed for the generic case */
    { "linux,phandle",   PROP_IGNORE }, /* not needed for the generic case */

    /* The following are copied and remapped by dedicated code */
    { "reg",             PROP_IGNORE },
    { "clocks",          PROP_IGNORE },
    { "interconnects",   PROP_IGNORE },
    { "#*-cells",        PROP_IGNORE },

    /* The following are handled by the host */
    { "iommus",          PROP_IGNORE }, /* isolation */

    /* Whitelist of properties not taking phandles, and thus safe to copy */
    { "compatible",      PROP_COPY },
    { "status",          PROP_COPY },
    { "ranges",          PROP_COPY },
    { "clock-names",     PROP_COPY },
    { "interconnect-names", PROP_COPY },

    { NULL,              PROP_IGNORE },  /* last element */
};

static ProppertList qcom_trogdor_geni_uart_prop[] = {
    { "name",            PROP_IGNORE }, /* handled automatically */
    { "phandle",         PROP_IGNORE }, /* not needed for the generic case */
    { "linux,phandle",   PROP_IGNORE }, /* not needed for the generic case */

    /* The following are copied and remapped by dedicated code */
    { "reg",             PROP_IGNORE },
    { "interrupts-extended",      PROP_IGNORE },
    { "clocks",          PROP_IGNORE },

    /* The following are handled by the host */
    { "power-domains",   PROP_IGNORE }, /* power management (+ opt. clocks) */
    { "pinctrl-names",   PROP_COPY },
    { "pinctrl-*",       PROP_IGNORE }, /* pin control */
    { "interconnects",   PROP_IGNORE },

    { "operating-points-v2",   PROP_IGNORE },

    /* Whitelist of properties not taking phandles, and thus safe to copy */
    { "clock-names",     PROP_COPY },
    { "interconnect-names", PROP_COPY },
    { "compatible",      PROP_COPY },
    { "status",          PROP_COPY },

    { NULL,              PROP_IGNORE },  /* last element */
};

static ProppertList qcom_trogdor_geni_i2c_prop[] = {
    { "name",            PROP_IGNORE }, /* handled automatically */
    { "phandle",         PROP_IGNORE }, /* not needed for the generic case */
    { "linux,phandle",   PROP_IGNORE }, /* not needed for the generic case */

    /* The following are copied and remapped by dedicated code */
    { "reg",             PROP_IGNORE },
    { "interrupts",      PROP_IGNORE },
    { "clocks",          PROP_IGNORE },

    /* The following are handled by the host */
    { "pinctrl-names",   PROP_COPY },
    { "pinctrl-*",       PROP_IGNORE }, /* pin control */
    { "interconnects",   PROP_IGNORE },

    /* Whitelist of properties not taking phandles, and thus safe to copy */
    { "clock-names",     PROP_COPY },
    { "interconnect-names", PROP_COPY },
    { "compatible",      PROP_COPY },
    { "status",          PROP_COPY },
    { "#*-cells",        PROP_COPY },
    { "clock-frequency", PROP_COPY },

    { NULL,              PROP_IGNORE },  /* last element */
};

static ProppertList qcom_trogdor_geni_spi_prop[] = {
    { "name",            PROP_IGNORE }, /* handled automatically */
    { "phandle",         PROP_IGNORE }, /* not needed for the generic case */
    { "linux,phandle",   PROP_IGNORE }, /* not needed for the generic case */

    /* The following are copied and remapped by dedicated code */
    { "reg",             PROP_IGNORE },
    { "interrupts",      PROP_IGNORE },
    { "clocks",          PROP_IGNORE },
    { "pinctrl-names",   PROP_COPY },
    { "pinctrl-*",       PROP_IGNORE }, /* pin control */
    { "power-domains",   PROP_IGNORE },
    { "operating-points-v2",   PROP_IGNORE },
    { "interconnects",   PROP_IGNORE },

    { "compatible",      PROP_COPY },
    { "clock-names",     PROP_COPY },
    { "#*-cells",        PROP_COPY },
    { "status",          PROP_COPY },
    { "interconnect-names", PROP_COPY },

    { NULL,              PROP_IGNORE },  /* last element */
};

static ProppertList qcom_trogdor_geni_bluetooth_prop[] = {
    { "name",             PROP_IGNORE }, /* handled automatically */
    { "phandle",          PROP_IGNORE }, /* not needed for the generic case */
    { "linux,phandle",    PROP_IGNORE }, /* not needed for the generic case */

    { "*-supply",         PROP_IGNORE },

    { "compatible",       PROP_COPY },
    { "status",           PROP_COPY },
    { "local-bd-address", PROP_COPY },
    { "max-speed",        PROP_COPY },

    { NULL,               PROP_IGNORE },  /* last element */
};

static ProppertList qcom_trogdor_geni_touchscreen_prop[] = {
    { "name",             PROP_IGNORE }, /* handled automatically */
    { "phandle",          PROP_IGNORE }, /* not needed for the generic case */
    { "linux,phandle",    PROP_IGNORE }, /* not needed for the generic case */

    { "interrupt-parent", PROP_IGNORE },
    { "interrupts",       PROP_IGNORE },
    { "*-supply",         PROP_IGNORE },
    { "pinctrl-names",    PROP_COPY },
    { "pinctrl-*",        PROP_IGNORE }, /* pin control */

    { "compatible",       PROP_COPY },
    { "status",           PROP_COPY },
    { "hid-descr-addr",   PROP_COPY },
    { "post-power-on-delay-ms",        PROP_COPY },
    { "reg",              PROP_COPY },

    { NULL,               PROP_IGNORE },  /* last element */
};

static ProppertList qcom_trogdor_geni_dsi_bridge_prop[] = {
    { "name",             PROP_IGNORE }, /* handled automatically */
    { "phandle",          PROP_IGNORE }, /* not needed for the generic case */
    { "linux,phandle",    PROP_IGNORE }, /* not needed for the generic case */

    { "interrupts",       PROP_IGNORE },
    { "pinctrl-names",    PROP_COPY },
    { "pinctrl-*",        PROP_IGNORE }, /* pin control */
    { "interrupt-parent", PROP_IGNORE },
    { "*-gpios",          PROP_IGNORE },
    { "*-supply",         PROP_IGNORE },
    { "clocks",           PROP_IGNORE },

    { "compatible",       PROP_COPY },
    { "reg",              PROP_COPY },
    { "gpio-controller",  PROP_COPY },
    { "#gpio-cells",      PROP_COPY },
    { "clock-names",      PROP_COPY },

    { NULL,               PROP_IGNORE },  /* last element */
};

static ProppertList qcom_trogdor_geni_ec_prop[] = {
    { "name",             PROP_IGNORE }, /* handled automatically */
    { "phandle",          PROP_IGNORE }, /* not needed for the generic case */
    { "linux,phandle",    PROP_IGNORE }, /* not needed for the generic case */

    { "interrupt-parent", PROP_IGNORE },
    { "interrupts",       PROP_IGNORE },
    { "pinctrl-names",    PROP_COPY },
    { "pinctrl-*",        PROP_IGNORE }, /* pin control */

    { "compatible",       PROP_COPY },
    { "reg",              PROP_COPY },
    { "spi-max-frequency",PROP_COPY },

    { NULL,               PROP_IGNORE },  /* last element */
};

static ProppertList qcom_trogdor_mdss_properties[] = {
    { "name",             PROP_IGNORE }, /* handled automatically */
    { "phandle",          PROP_IGNORE }, /* not needed for the generic case */
    { "linux,phandle",    PROP_IGNORE }, /* not needed for the generic case */

    { "reg",              PROP_IGNORE },
    { "power-domains",    PROP_IGNORE },
    { "clocks",           PROP_IGNORE },
    { "assigned-clocks",  PROP_IGNORE },
    { "interrupts",       PROP_IGNORE },
    { "interconnects",    PROP_IGNORE },
    { "iommus",           PROP_IGNORE },
    { "#address-cells",   PROP_IGNORE },
    { "#size-cells",      PROP_IGNORE },
    { "dont-probe",       PROP_IGNORE },

    { "compatible",       PROP_COPY },
    { "reg-names",        PROP_COPY },
    { "clock-names",      PROP_COPY },
    { "assigned-clock-rates", PROP_COPY },
    { "interrupt-controller", PROP_COPY },
    { "#interrupt-cells",     PROP_COPY },
    { "interconnect-names",   PROP_COPY },
    { "ranges",           PROP_COPY },
    { "status",           PROP_COPY },

    { NULL,               PROP_IGNORE },  /* last element */
};

static ProppertList qcom_trogdor_mdp_prop[] = {
    { "name",             PROP_IGNORE }, /* handled automatically */
    { "phandle",          PROP_IGNORE }, /* not needed for the generic case */
    { "linux,phandle",    PROP_IGNORE }, /* not needed for the generic case */

    { "reg",              PROP_IGNORE },
    { "clocks",           PROP_IGNORE },
    { "assigned-clocks",  PROP_IGNORE },
    { "operating-points-v2", PROP_IGNORE },
    { "power-domains",    PROP_IGNORE },
    { "interrupt-parent", PROP_IGNORE },
    { "dont-probe",       PROP_IGNORE },

    { "compatible",       PROP_COPY },
    { "reg-names",        PROP_COPY },
    { "clock-names",      PROP_COPY },
    { "assigned-clock-rates", PROP_COPY },
    { "interrupts",       PROP_COPY },
    { "status",           PROP_COPY },

    { NULL,               PROP_IGNORE },  /* last element */
};

static ProppertList qcom_trogdor_dsi_prop[] = {
    { "name",             PROP_IGNORE }, /* handled automatically */
    { "phandle",          PROP_IGNORE }, /* not needed for the generic case */
    { "linux,phandle",    PROP_IGNORE }, /* not needed for the generic case */

    { "reg",              PROP_IGNORE },
    { "interrupt-parent", PROP_IGNORE },
    { "clocks",           PROP_IGNORE },
    { "operating-points-v2", PROP_IGNORE },
    { "power-domains",    PROP_IGNORE },
    { "phys",              PROP_IGNORE },
    { "*-supply",         PROP_IGNORE },
    { "dont-probe",       PROP_IGNORE },

    { "compatible",       PROP_COPY },
    { "reg-names",        PROP_COPY },
    { "interrupts",       PROP_COPY },
    { "clock-names",      PROP_COPY },
    { "phy-names",        PROP_COPY },
    { "#address-cells",   PROP_COPY },
    { "#size-cells",      PROP_COPY },
    { "status",           PROP_COPY },

    { NULL,               PROP_IGNORE },  /* last element */
};

static ProppertList qcom_trogdor_dsi_phy_prop[] = {
    { "name",             PROP_IGNORE }, /* handled automatically */
    { "phandle",          PROP_IGNORE }, /* not needed for the generic case */
    { "linux,phandle",    PROP_IGNORE }, /* not needed for the generic case */

    { "reg",              PROP_IGNORE },
    { "clocks",           PROP_IGNORE },
    { "*-supply",         PROP_IGNORE },
    { "dont-probe",       PROP_IGNORE },

    { "compatible",       PROP_COPY },
    { "reg-names",        PROP_COPY },
    { "#phy-cells",       PROP_COPY },
    { "clock-names",      PROP_COPY },
    { "status",           PROP_COPY },
    { "#clock-cells",     PROP_COPY },

    { NULL,               PROP_IGNORE },  /* last element */
};

static ProppertList qcom_trogdor_dp_prop[] = {
    { "name",             PROP_IGNORE }, /* handled automatically */
    { "phandle",          PROP_IGNORE }, /* not needed for the generic case */
    { "linux,phandle",    PROP_IGNORE }, /* not needed for the generic case */

    { "reg",              PROP_IGNORE },
    { "interrupt-parent", PROP_IGNORE },
    { "clocks",           PROP_IGNORE },
    { "#clock-cells",     PROP_IGNORE },
    { "assigned-clocks",  PROP_IGNORE },
    { "assigned-clock-parents", PROP_IGNORE },
    { "phy",              PROP_IGNORE },
    { "operating-points-v2", PROP_IGNORE },
    { "power-domains",    PROP_IGNORE },
    { "pinctrl-names",    PROP_COPY },
    { "pinctrl-*",        PROP_IGNORE }, /* pin control */
    { "*-supply",         PROP_IGNORE },
    { "dont-probe",       PROP_IGNORE },

    { "compatible",       PROP_COPY },
    { "status",           PROP_COPY },
    { "interrupts",       PROP_COPY },
    { "phy-names",        PROP_COPY },
    { "data-lanes",       PROP_COPY },

    { NULL,               PROP_IGNORE },  /* last element */
};

static ProppertList qcom_trogdor_disp_clk_prop[] = {
    { "name",             PROP_IGNORE }, /* handled automatically */
    { "phandle",          PROP_IGNORE }, /* not needed for the generic case */
    { "linux,phandle",    PROP_IGNORE }, /* not needed for the generic case */

    { "reg",              PROP_IGNORE },
    { "clocks",           PROP_IGNORE },
    { "clock-names",      PROP_IGNORE },

    { "compatible",       PROP_COPY },
    { "#clock-cells",     PROP_COPY },
    { "#reset-cells",     PROP_COPY },
    { "#power-domain-cells", PROP_COPY },

    { NULL,               PROP_IGNORE },  /* last element */
};

static ProppertList qcom_trogdor_regulator_properties[] = {
    { "compatible",              PROP_IGNORE },
    { "phandle",                 PROP_IGNORE },
    { "name",                    PROP_IGNORE },

    { "regulator-initial-mode",  PROP_COPY },
    { "regulator-max-microvolt", PROP_COPY },
    { "regulator-min-microvolt", PROP_COPY },

    { NULL,                      PROP_IGNORE },  /* last element */
};

static ProppertList qcom_trogdor_regulator_touchscreen_properties[] = {
    { "compatible",              PROP_IGNORE },
    { "phandle",                 PROP_IGNORE },
    { "name",                    PROP_IGNORE },

    { "regulator-name",          PROP_COPY },
    { "regulator-max-microvolt", PROP_COPY },
    { "regulator-min-microvolt", PROP_COPY },

    { "enable-active-high", PROP_COPY },
    { "gpio",               PROP_IGNORE },
    { "pinctrl-*",          PROP_IGNORE },
    { "*-supply",           PROP_IGNORE },

    { NULL,                      PROP_IGNORE },  /* last element */
};

static ProppertList qcom_trogdor_regulator_backlight_properties[] = {
    { "compatible",             PROP_IGNORE },
    { "phandle",                PROP_IGNORE },
    { "name",                    PROP_IGNORE },

    { "regulator-name",         PROP_COPY },
    { "regulator-always-on",    PROP_COPY },
    { "regulator-boot-on",      PROP_COPY },

    { NULL,                     PROP_IGNORE },  /* last element */
};

static ProppertList qcom_trogdor_regulator_panel_properties[] = {
    { "compatible",             PROP_IGNORE },
    { "phandle",                PROP_IGNORE },
    { "name",                    PROP_IGNORE },

    { "gpio",                   PROP_IGNORE },
    { "enable-active-high",     PROP_IGNORE },
    { "pinctrl-*",              PROP_IGNORE },
    { "*-supply",               PROP_IGNORE },

    { "regulator-name",         PROP_COPY },
    { "regulator-max-microvolt", PROP_COPY },
    { "regulator-min-microvolt", PROP_COPY },

    { NULL,                     PROP_IGNORE },  /* last element */
};

/*
 * This is Linux-specific flags. Default controllers' and Linux' mapping match.
 */
enum of_gpio_flags {
    OF_GPIO_ACTIVE_LOW = 0x1,
    OF_GPIO_SINGLE_ENDED = 0x2,
    OF_GPIO_OPEN_DRAIN = 0x4,
    OF_GPIO_TRANSITORY = 0x8,
    OF_GPIO_PULL_UP = 0x10,
    OF_GPIO_PULL_DOWN = 0x20,
};

static uint64_t fdt_xlate_flags(uint64_t flags)
{
    uint64_t xlate_flags = 0;

    if (flags & VFIO_GPIO_ACTIVE_LOW)
        xlate_flags |= OF_GPIO_ACTIVE_LOW;

    if (flags & VFIO_GPIO_OPEN_SOURCE)
        xlate_flags |= OF_GPIO_SINGLE_ENDED;
    else if (flags & VFIO_GPIO_OPEN_DRAIN)
        xlate_flags |= OF_GPIO_SINGLE_ENDED | OF_GPIO_OPEN_DRAIN;

    if (flags & VFIO_GPIO_TRANSITORY)
        xlate_flags |= OF_GPIO_TRANSITORY;

    if (flags & VFIO_GPIO_PULL_UP)
        xlate_flags |= OF_GPIO_PULL_UP;

    if (flags & VFIO_GPIO_PULL_DOWN)
        xlate_flags |= OF_GPIO_PULL_DOWN;

    return xlate_flags;
}

static bool vfio_platform_match_raw(SysBusDevice *sbdev, const char *pattern)
{
    VFIOPlatformDevice *vdev = VFIO_PLATFORM_DEVICE(sbdev);
    const char *compat;
    unsigned int n;

    for (n = vdev->num_compat, compat = vdev->compat; n > 0;
         n--, compat += strlen(compat) + 1) {

        error_report("%s looking for %s trying %s", __func__, compat, pattern);
        if (!strcmp(pattern, compat)) {
            return true;
        }
    }

    return false;
}

// MICAH hook into this for qcom geni se
static int add_qcom_trogdor_fdt_node(SysBusDevice *sbdev, void *opaque,
                                   ProppertList *properties)
{
    PlatformBusFDTData *data = opaque;
    const char *parent_node = data->pbus_node_name;
    PlatformBusDevice *pbus = data->pbus;
    void *fdt = data->fdt;
    VFIOPlatformDevice *vdev = VFIO_PLATFORM_DEVICE(sbdev);
    VFIODevice *vbasedev = &vdev->vbasedev;
    char *node_name;
    uint64_t mmio_base;
    int i;

    error_report("%s 0", __func__);

    if (vbasedev->num_regions > 0) {
        mmio_base = platform_bus_get_mmio_addr(pbus, sbdev, 0);
        node_name = g_strdup_printf("%s/%s@%" PRIx64, parent_node,
                                    vbasedev->name, mmio_base);
    } else
        node_name = g_strdup_printf("%s/%s", parent_node, vbasedev->name);

    error_report("%s 1", __func__);

    qemu_fdt_add_subnode(fdt, node_name);
    copy_host_node_prop(vdev, fdt, node_name, properties);

    if (vfio_platform_match_raw(sbdev, "pwm-backlight")) {
        uint32_t pwms_attr[2];

        if (ec_pwm_handle == 0)
            ec_pwm_handle = qemu_fdt_alloc_phandle(fdt);

        pwms_attr[0] = cpu_to_be32(ec_pwm_handle);
        pwms_attr[1] = cpu_to_be32(1);

        qemu_fdt_setprop(fdt, node_name, "pwms", pwms_attr,
                         2 * sizeof(uint32_t));

        if (panel_backlight_handle == 0)
            panel_backlight_handle = qemu_fdt_alloc_phandle(fdt);

        qemu_fdt_setprop_cell(fdt, node_name, "phandle", panel_backlight_handle);
    }

    if (vfio_platform_match_raw(sbdev, "boe,nv133fhm-n62")) {
        uint32_t gpio_attr[3];
        char *port_name;

        /* Backlight reference */
        if (panel_backlight_handle == 0)
            panel_backlight_handle = qemu_fdt_alloc_phandle(fdt);

        qemu_fdt_setprop_cell(fdt, node_name, "backlight", panel_backlight_handle);

        /*
         * GPIO reference as-is (GPIO controller is assigned entirely),
         * except phandle.
         */
        if (panel_gpio_handle == 0)
            panel_gpio_handle = qemu_fdt_alloc_phandle(fdt);

        gpio_attr[0] = cpu_to_be32(panel_gpio_handle);
        gpio_attr[1] = cpu_to_be32(2);
        gpio_attr[2] = cpu_to_be32(0);

        qemu_fdt_setprop(fdt, node_name, "hpd-gpios", gpio_attr,
                         3 * sizeof(uint32_t));

        /* Port tree */
        port_name = g_strdup_printf("%s/%s", node_name, "ports");
        qemu_fdt_add_subnode(fdt, port_name);
        port_name = g_strdup_printf("%s/%s", port_name, "port");
        qemu_fdt_add_subnode(fdt, port_name);
        port_name = g_strdup_printf("%s/%s", port_name, "endpoint");
        qemu_fdt_add_subnode(fdt, port_name);

        if (panel_dsi_bridge_rempte_endpoint == 0)
            panel_dsi_bridge_rempte_endpoint = qemu_fdt_alloc_phandle(fdt);
        if (panel_dsi_bridge_phandle == 0)
            panel_dsi_bridge_phandle = qemu_fdt_alloc_phandle(fdt);

        qemu_fdt_setprop_cell(fdt, port_name, "remote-endpoint",
                              panel_dsi_bridge_phandle);
        qemu_fdt_setprop_cell(fdt, port_name, "phandle",
                              panel_dsi_bridge_rempte_endpoint);
    }

    if (vfio_platform_match_raw(sbdev, "qcom,sc7180-dispcc")) {

        if (disp_clk_handle == 0)
            disp_clk_handle = qemu_fdt_alloc_phandle(fdt);

        qemu_fdt_setprop_cell(fdt, node_name, "phandle", disp_clk_handle);
    }

    if (vfio_platform_match_raw(sbdev, "qcom,adreno-gmu")) {

        if (gmu_phandle == 0)
            gmu_phandle = qemu_fdt_alloc_phandle(fdt);

        qemu_fdt_setprop_cell(fdt, node_name, "phandle", gmu_phandle);
    }

    if (vfio_platform_match_raw(sbdev, "qcom,adreno")) {

        if (gmu_phandle == 0)
            gmu_phandle = qemu_fdt_alloc_phandle(fdt);

        qemu_fdt_setprop_cell(fdt, node_name, "qcom,gmu", gmu_phandle);
    }

    error_report("%s 2", __func__);
    /* Copy reg (remapped) */
    if (vbasedev->num_regions > 0) {
        uint32_t *reg_attr = g_new(uint32_t, vbasedev->num_regions * 2);
        for (i = 0; i < vbasedev->num_regions; i++) {
            mmio_base = platform_bus_get_mmio_addr(pbus, sbdev, i);
            reg_attr[2 * i] = cpu_to_be32(mmio_base);
            reg_attr[2 * i + 1] = cpu_to_be32(
                                    memory_region_size(vdev->regions[i]->mem));
        }
        qemu_fdt_setprop(fdt, node_name, "reg", reg_attr,
                         vbasedev->num_regions * 2 * sizeof(uint32_t));
        g_free(reg_attr);
    }

    error_report("%s 3", __func__);

    /* Copy interrupts (remapped) */
    if (vbasedev->num_irqs) {
        uint32_t *irq_attr = g_new(uint32_t, vbasedev->num_irqs * 3);
        for (i = 0; i < vbasedev->num_irqs; i++) {
            VFIOINTp *intp;
            int irq_number = platform_bus_get_irqn(pbus, sbdev, i) +
                                                             data->irq_start;

            irq_attr[3 * i] = cpu_to_be32(GIC_FDT_IRQ_TYPE_SPI);
            irq_attr[3 * i + 1] = cpu_to_be32(irq_number);
            QLIST_FOREACH(intp, &vdev->intp_list, next) {
                if (intp->pin == i) {
                    break;
                }
            }
            if (intp->flags & VFIO_IRQ_INFO_AUTOMASKED) {
                irq_attr[3 * i + 2] = cpu_to_be32(GIC_FDT_IRQ_FLAGS_LEVEL_HI);
            } else {
                irq_attr[3 * i + 2] = cpu_to_be32(GIC_FDT_IRQ_FLAGS_EDGE_LO_HI);
            }
        }
        qemu_fdt_setprop(fdt, node_name, "interrupts",
                         irq_attr, vbasedev->num_irqs * 3 * sizeof(uint32_t));
        g_free(irq_attr);
    }

    error_report("%s 4", __func__);

    if (vbasedev->num_clks > 0) {

        error_report("%s creating %d clk nodes",
                     __func__, vbasedev->num_clks);

        uint32_t *clk = g_new(uint32_t, vbasedev->num_clks);
        for (i = 0; i < vbasedev->num_clks;  i++) {
            clk[i] = qemu_fdt_alloc_phandle(fdt);
            fdt_build_virtio_clk_mmio_node(data, vdev, i, fdt, clk[i]);
            clk[i] = cpu_to_be32(clk[i]);
        }
        qemu_fdt_setprop(fdt, node_name, "clocks", clk,
                         vbasedev->num_clks * sizeof(uint32_t));
        g_free(clk);
    }

    error_report("%s 5", __func__);
    if (vbasedev->num_regulators) {
        error_report("%s creating %d regulator nodes",
                     __func__, vbasedev->num_regulators);
        for (i = 0; i < vbasedev->num_regulators;  i++) {
            char *supply_name;
            uint32_t reg;

            reg = qemu_fdt_alloc_phandle(fdt);

            if (vfio_platform_match_raw(sbdev, "pwm-backlight")) {
                fdt_build_virtio_regulator_mmio_node(data, vdev, i, fdt, reg,
                                                     vbasedev->regulator_names[i],
                                                     qcom_trogdor_regulator_backlight_properties);
            } else if (vfio_platform_match_raw(sbdev, "boe,nv133fhm-n62")) {
                fdt_build_virtio_regulator_mmio_node(data, vdev, i, fdt, reg,
                                                     vbasedev->regulator_names[i],
                                                     qcom_trogdor_regulator_panel_properties);
            } else {
                fdt_build_virtio_regulator_mmio_node(data, vdev, i, fdt, reg,
                                                     vbasedev->regulator_names[i],
                                                     qcom_trogdor_regulator_properties);
            }

            supply_name = g_strdup_printf("%s-supply",
                                          vbasedev->regulator_names[i]);

            error_report("%s creating regulator name %s", __func__, supply_name);
            qemu_fdt_setprop_cell(fdt, node_name, supply_name, reg);
        }
    }

    error_report("%s 6", __func__);
    if (vbasedev->num_gpio_func > 0) {
        error_report("%s creating %d gpio nodes",
                     __func__, vbasedev->num_gpio_func);
        for (i = 0; i < vbasedev->num_gpio_func;  i++) {
            uint32_t *gpio_attr;
            unsigned int pin;
            char *func_name;
            uint32_t gpio;

            gpio = qemu_fdt_alloc_phandle(fdt);
            fdt_build_virtio_gpio_mmio_node(data, vdev, i, fdt, gpio);
            func_name = g_strdup_printf("%s-gpios",
                                          vbasedev->gpio[i].gpio_func_names);

            error_report("%s creating GPIO func name %s", __func__, func_name);
            gpio_attr = g_new(uint32_t, vbasedev->gpio[i].pin_num * 3);
            for (pin = 0; pin < vbasedev->gpio[i].pin_num; pin++) {
                uint64_t flags = fdt_xlate_flags(vbasedev->gpio[i].flags[pin]);

                error_report("%s creating GPIO func name %s pin %d flags 0x%lx xflags 0x%lx",
                        __func__, func_name, pin, (long)vbasedev->gpio[i].flags[pin],
                        (long)flags);

                gpio_attr[(pin * 3) + 0] = cpu_to_be32(gpio);
                gpio_attr[(pin * 3) + 1] = cpu_to_be32(pin);
                gpio_attr[(pin * 3) + 2] = cpu_to_be32(flags);
            }
            qemu_fdt_setprop(fdt, node_name, func_name, gpio_attr,
                             vbasedev->gpio[i].pin_num * 3 * sizeof(uint32_t));
            g_free(gpio_attr);
        }
    }
    error_report("%s 7", __func__);

    if (vbasedev->num_interconnects) {
        uint32_t *inter_attr = g_new(uint32_t, vbasedev->num_interconnects * 4);
        error_report("%s creating %d interconnect nodes",
                     __func__, vbasedev->num_interconnects);

        for (i = 0; i < vbasedev->num_interconnects;  i++) {
            uint32_t inter;

            inter = qemu_fdt_alloc_phandle(fdt);
            fdt_build_virtio_inter_mmio_node(data, vdev, i, fdt, inter);
            /* src node points to dst node within the same provider */
            inter_attr[(i * 4) + 0] = cpu_to_be32(inter);
            inter_attr[(i * 4) + 1] = cpu_to_be32(0); // node id
            inter_attr[(i * 4) + 2] = cpu_to_be32(inter);
            inter_attr[(i * 4) + 3] = cpu_to_be32(1); // node id
        }
        qemu_fdt_setprop(fdt, node_name, "interconnects", inter_attr,
                         vbasedev->num_interconnects * sizeof(uint32_t) * 4);
        g_free(inter_attr);
    }

    error_report("%s 8", __func__);
    if (vbasedev->num_phys > 0) {
        uint32_t *phy = g_new(uint32_t, vbasedev->num_phys);

        error_report("%s creating %d phy nodes", __func__, vbasedev->num_phys);

        for (i = 0; i < vbasedev->num_phys;  i++) {
            phy[i] = qemu_fdt_alloc_phandle(fdt);
            fdt_build_virtio_phy_mmio_node(data, vdev, i, fdt, phy[i]);
            phy[i] = cpu_to_be32(phy[i]);
        }
        qemu_fdt_setprop(fdt, node_name, "phys", phy,
                         vbasedev->num_phys * sizeof(uint32_t));
        g_free(phy);
    }

    error_report("%s 9", __func__);
    if (vbasedev->num_pctrl_states) {
        error_report("%s creating %d pinctrl nodes",
                     __func__, vbasedev->num_pctrl_states);
        for (i = 0; i < vbasedev->num_pctrl_states;  i++) {
            char *pinctrl_name = g_strdup_printf("pinctrl-%u", i);
            uint32_t pctrl = qemu_fdt_alloc_phandle(fdt);

            fdt_build_virtio_pinctrl_mmio_node(data, vdev, i, fdt, pctrl);
            qemu_fdt_setprop_cell(fdt, node_name, pinctrl_name, pctrl);
        }
    }

    g_free(node_name);
    return 0;
}

static int add_qcom_trogdor_dwc3_child_node(SysBusDevice *sbdev, void *opaque,
                                   ProppertList *properties)
{
    PlatformBusFDTData *data = opaque;
    const char *parent_node = data->pbus_node_name;
    PlatformBusDevice *pbus = data->pbus;
    void *fdt_guest = data->fdt;
    VFIOPlatformDevice *vdev = VFIO_PLATFORM_DEVICE(sbdev);
    VFIODevice *vbasedev = &vdev->vbasedev;
    uint32_t *reg_attr, *irq_attr, *guest_clk_phandle;
    uint32_t phandle;
    char *node_name;
    uint64_t mmio_base;
    int i, irq_number;
    VFIOINTp *intp;
    PlatformBusFDTData dwc3_child_data = {
        .fdt = fdt_guest,
        .irq_start = data->irq_start,
        .pbus = pbus,
    };

    mmio_base = platform_bus_get_mmio_addr(pbus, sbdev, 0);
    node_name = g_strdup_printf("%s/dwc3", parent_node);

    qemu_fdt_add_subnode(fdt_guest, node_name);
    copy_host_node_prop(vdev, fdt_guest, node_name, properties);

    /* Copy reg (remapped) */
    reg_attr = g_new(uint32_t, vbasedev->num_regions * 2);
    for (i = 0; i < vbasedev->num_regions; i++) {
        mmio_base = platform_bus_get_mmio_addr(pbus, sbdev, i);
        reg_attr[2 * i] = cpu_to_be32(mmio_base);
        reg_attr[2 * i + 1] = cpu_to_be32(
                                memory_region_size(vdev->regions[i]->mem));
    }
    qemu_fdt_setprop(fdt_guest, node_name, "reg", reg_attr,
                     vbasedev->num_regions * 2 * sizeof(uint32_t));

    /* Copy interrupts (remapped) */
    if (vbasedev->num_irqs) {
        irq_attr = g_new(uint32_t, vbasedev->num_irqs * 3);
        for (i = 0; i < vbasedev->num_irqs; i++) {
            irq_number = platform_bus_get_irqn(pbus, sbdev, i) +
                         data->irq_start;
            irq_attr[3 * i] = cpu_to_be32(GIC_FDT_IRQ_TYPE_SPI);
            irq_attr[3 * i + 1] = cpu_to_be32(irq_number);
            QLIST_FOREACH(intp, &vdev->intp_list, next) {
                if (intp->pin == i) {
                    break;
                }
            }
            if (intp->flags & VFIO_IRQ_INFO_AUTOMASKED) {
                irq_attr[3 * i + 2] = cpu_to_be32(GIC_FDT_IRQ_FLAGS_LEVEL_HI);
            } else {
                irq_attr[3 * i + 2] = cpu_to_be32(GIC_FDT_IRQ_FLAGS_EDGE_LO_HI);
            }
        }
        qemu_fdt_setprop(fdt_guest, node_name, "interrupts",
                         irq_attr, vbasedev->num_irqs * 3 * sizeof(uint32_t));
        g_free(irq_attr);
    }

    guest_clk_phandle = g_new(uint32_t, vbasedev->num_clks);
    for (i = 0; i < vbasedev->num_clks;  i++) {
        guest_clk_phandle[i] = qemu_fdt_alloc_phandle(fdt_guest);
        fdt_build_virtio_clk_mmio_node(data, vdev, i, fdt_guest,
                                       guest_clk_phandle[i]);
        guest_clk_phandle[i] = cpu_to_be32(guest_clk_phandle[i]);
    }
    if (vbasedev->num_clks > 0) {
        qemu_fdt_setprop(fdt_guest, node_name, "clocks", guest_clk_phandle,
                         vbasedev->num_clks * sizeof(uint32_t));
    }
    g_free(guest_clk_phandle);

    guest_clk_phandle = g_new(uint32_t, vbasedev->num_regulators);
    for (i = 0; i < vbasedev->num_regulators;  i++) {
        char *supply_name;

        guest_clk_phandle[i] = qemu_fdt_alloc_phandle(fdt_guest);
        fdt_build_virtio_regulator_mmio_node(data, vdev, i, fdt_guest,
                                             guest_clk_phandle[i],
                                             vbasedev->regulator_names[i],
                                             qcom_trogdor_regulator_properties);
        supply_name = g_strdup_printf("%s-supply",
                                      vbasedev->regulator_names[i]);

        error_report("%s copying properties of regulator name %s",
                     __func__, supply_name);

        qemu_fdt_setprop_cell(fdt_guest, node_name, supply_name,
                              guest_clk_phandle[i]);
    }

    g_free(guest_clk_phandle);

    guest_clk_phandle = g_new(uint32_t, vbasedev->num_interconnects * 4);

    error_report("%s creating %d interconnect nodes",
                 __func__, vbasedev->num_interconnects);

    for (i = 0; i < vbasedev->num_interconnects;  i++) {
        phandle = qemu_fdt_alloc_phandle(fdt_guest);
        fdt_build_virtio_inter_mmio_node(data, vdev, i, fdt_guest, phandle);
        /* src node points to dst node within the same provider */
        guest_clk_phandle[(i * 4) + 0] = cpu_to_be32(phandle);
        guest_clk_phandle[(i * 4) + 1] = cpu_to_be32(0); // node id
        guest_clk_phandle[(i * 4) + 2] = cpu_to_be32(phandle);
        guest_clk_phandle[(i * 4) + 3] = cpu_to_be32(1); // node id
    }
    if (vbasedev->num_interconnects > 0) {
        qemu_fdt_setprop(fdt_guest, node_name, "interconnects", guest_clk_phandle,
                         vbasedev->num_interconnects * sizeof(uint32_t) * 4);
    }

    g_free(guest_clk_phandle);

    guest_clk_phandle = g_new(uint32_t, vbasedev->num_phys);

    error_report("%s creating %d phys nodes",
                 __func__, vbasedev->num_phys);

    for (i = 0; i < vbasedev->num_phys;  i++) {
        guest_clk_phandle[i] = qemu_fdt_alloc_phandle(fdt_guest);
        fdt_build_virtio_phy_mmio_node(data, vdev, i, fdt_guest,
                                       guest_clk_phandle[i]);
        guest_clk_phandle[i] = cpu_to_be32(guest_clk_phandle[i]);
    }
    if (vbasedev->num_phys > 0) {
        qemu_fdt_setprop(fdt_guest, node_name, "phys", guest_clk_phandle,
                         vbasedev->num_phys * sizeof(uint32_t));
    }
    g_free(guest_clk_phandle);

    guest_clk_phandle = g_new(uint32_t, vbasedev->num_pctrl_states);
    for (i = 0; i < vbasedev->num_pctrl_states;  i++) {
        char *pinctrl_name = g_strdup_printf("pinctrl-%u", i);

        guest_clk_phandle[i] = qemu_fdt_alloc_phandle(fdt_guest);
        fdt_build_virtio_pinctrl_mmio_node(data, vdev, i, fdt_guest,
                                           guest_clk_phandle[i]);

        error_report("%s copying properties of regulator name %s",
                     __func__, pinctrl_name);

        qemu_fdt_setprop_cell(fdt_guest, node_name, pinctrl_name,
                              guest_clk_phandle[i]);
    }

    g_free(guest_clk_phandle);
    g_free(reg_attr);
    g_free(node_name);
    return 0;
}

static int add_qcom_trogdor_bluetooth_node(SysBusDevice *sbdev, void *opaque,
                                   ProppertList *properties)
{
    PlatformBusFDTData *data = opaque;
    const char *parent_node = data->pbus_node_name;
    PlatformBusDevice *pbus = data->pbus;
    void *fdt_guest = data->fdt;
    VFIOPlatformDevice *vdev = VFIO_PLATFORM_DEVICE(sbdev);
    VFIODevice *vbasedev = &vdev->vbasedev;
    uint32_t *guest_clk_phandle;
    char *node_name;
    int i;

    node_name = g_strdup_printf("%s/bluetooth", parent_node);
    qemu_fdt_add_subnode(fdt_guest, node_name);
    copy_host_node_prop(vdev, fdt_guest, node_name, properties);

    guest_clk_phandle = g_new(uint32_t, vbasedev->num_regulators);
    for (i = 0; i < vbasedev->num_regulators;  i++) {
        char *supply_name;

        guest_clk_phandle[i] = qemu_fdt_alloc_phandle(fdt_guest);
        fdt_build_virtio_regulator_mmio_node(data, vdev, i, fdt_guest,
                                             guest_clk_phandle[i],
                                             vbasedev->regulator_names[i],
                                             qcom_trogdor_regulator_properties);
        supply_name = g_strdup_printf("%s-supply",
                                      vbasedev->regulator_names[i]);

        error_report("%s copying properties of regulator name %s",
                     __func__, supply_name);

        qemu_fdt_setprop_cell(fdt_guest, node_name, supply_name,
                              guest_clk_phandle[i]);
    }

    g_free(guest_clk_phandle);
    g_free(node_name);
    return 0;
}

static int add_qcom_trogdor_touchscreen_node(SysBusDevice *sbdev, void *opaque,
                                   ProppertList *properties)
{
    PlatformBusFDTData *data = opaque;
    const char *parent_node = data->pbus_node_name;
    PlatformBusDevice *pbus = data->pbus;
    void *fdt_guest = data->fdt;
    VFIOPlatformDevice *vdev = VFIO_PLATFORM_DEVICE(sbdev);
    VFIODevice *vbasedev = &vdev->vbasedev;
    uint32_t *irq_attr, *guest_phandle;
    char *node_name;
    VFIOINTp *intp;
    int i, irq_number;

    node_name = g_strdup_printf("%s/touchscreen@10", parent_node);
    qemu_fdt_add_subnode(fdt_guest, node_name);
    copy_host_node_prop(vdev, fdt_guest, node_name, properties);

    /* Copy interrupts (remapped) */
    if (vbasedev->num_irqs) {
        irq_attr = g_new(uint32_t, vbasedev->num_irqs * 3);
        for (i = 0; i < vbasedev->num_irqs; i++) {
            irq_number = platform_bus_get_irqn(pbus, sbdev, i) +
                         data->irq_start;
            irq_attr[3 * i] = cpu_to_be32(GIC_FDT_IRQ_TYPE_SPI);
            irq_attr[3 * i + 1] = cpu_to_be32(irq_number);
            QLIST_FOREACH(intp, &vdev->intp_list, next) {
                if (intp->pin == i) {
                    break;
                }
            }
            if (intp->flags & VFIO_IRQ_INFO_AUTOMASKED) {
                irq_attr[3 * i + 2] = cpu_to_be32(GIC_FDT_IRQ_FLAGS_LEVEL_HI);
            } else {
                irq_attr[3 * i + 2] = cpu_to_be32(GIC_FDT_IRQ_FLAGS_EDGE_LO_HI);
            }
        }
        qemu_fdt_setprop(fdt_guest, node_name, "interrupts",
                         irq_attr, vbasedev->num_irqs * 3 * sizeof(uint32_t));
        g_free(irq_attr);
    }

    guest_phandle = g_new(uint32_t, vbasedev->num_regulators);
    for (i = 0; i < vbasedev->num_regulators;  i++) {
        char *supply_name;

        guest_phandle[i] = qemu_fdt_alloc_phandle(fdt_guest);
        fdt_build_virtio_regulator_mmio_node(data, vdev, i, fdt_guest,
                                             guest_phandle[i],
                                             vbasedev->regulator_names[i],
                                             qcom_trogdor_regulator_touchscreen_properties);
        supply_name = g_strdup_printf("%s-supply",
                                      vbasedev->regulator_names[i]);

        error_report("%s copying properties of regulator name %s",
                     __func__, supply_name);

        qemu_fdt_setprop_cell(fdt_guest, node_name, supply_name,
                              guest_phandle[i]);
    }

    g_free(guest_phandle);
    guest_phandle = g_new(uint32_t, vbasedev->num_pctrl_states);
    for (i = 0; i < vbasedev->num_pctrl_states;  i++) {
        char *pinctrl_name = g_strdup_printf("pinctrl-%u", i);

        guest_phandle[i] = qemu_fdt_alloc_phandle(fdt_guest);
        fdt_build_virtio_pinctrl_mmio_node(data, vdev, i, fdt_guest,
                                           guest_phandle[i]);

        error_report("%s copying properties of regulator name %s",
                     __func__, pinctrl_name);

        qemu_fdt_setprop_cell(fdt_guest, node_name, pinctrl_name,
                              guest_phandle[i]);
    }

    g_free(guest_phandle);
    g_free(node_name);
    return 0;
}

static int add_qcom_trogdor_dsi_bridge_node(SysBusDevice *sbdev, void *opaque,
                                            ProppertList *properties)
{
    PlatformBusFDTData *data = opaque;
    const char *parent_node = data->pbus_node_name;
    PlatformBusDevice *pbus = data->pbus;
    void *fdt = data->fdt;
    VFIOPlatformDevice *vdev = VFIO_PLATFORM_DEVICE(sbdev);
    VFIODevice *vbasedev = &vdev->vbasedev;
    char *node_name, *ports_name, *port_name, *endpoint_name;
    int i;

    node_name = g_strdup_printf("%s/bridge@2d", parent_node);
    qemu_fdt_add_subnode(fdt, node_name);
    copy_host_node_prop(vdev, fdt, node_name, properties);

    /* Reference for panel GPIO */
    if (panel_gpio_handle == 0)
        panel_gpio_handle = qemu_fdt_alloc_phandle(fdt);
    qemu_fdt_setprop_cell(fdt, node_name, "phandle", panel_gpio_handle);

    /* Port tree */
    ports_name = g_strdup_printf("%s/%s", node_name, "ports");
    qemu_fdt_add_subnode(fdt, ports_name);
    qemu_fdt_setprop_cell(fdt, ports_name, "#address-cells", 1);
    qemu_fdt_setprop_cell(fdt, ports_name, "#size-cells", 0);

    /* First port, MDSS reference */
    port_name = g_strdup_printf("%s/%s", ports_name, "port@0");
    qemu_fdt_add_subnode(fdt, port_name);
    qemu_fdt_setprop_cell(fdt, port_name, "reg", 0);

    endpoint_name = g_strdup_printf("%s/%s", port_name, "endpoint");
    qemu_fdt_add_subnode(fdt, endpoint_name);

    if (mdss_dsi_bridge_rempte_endpoint == 0)
        mdss_dsi_bridge_rempte_endpoint = qemu_fdt_alloc_phandle(fdt);
    if (mdss_dsi_bridge_phandle == 0)
        mdss_dsi_bridge_phandle = qemu_fdt_alloc_phandle(fdt);

    qemu_fdt_setprop_cell(fdt, endpoint_name, "remote-endpoint",
                          mdss_dsi_bridge_rempte_endpoint);
    qemu_fdt_setprop_cell(fdt, endpoint_name, "phandle",
                          mdss_dsi_bridge_phandle);

    /* Second port, panel reference */
    port_name = g_strdup_printf("%s/%s", ports_name, "port@1");
    qemu_fdt_add_subnode(fdt, port_name);
    qemu_fdt_setprop_cell(fdt, port_name, "reg", 1);

    endpoint_name = g_strdup_printf("%s/%s", port_name, "endpoint");
    qemu_fdt_add_subnode(fdt, endpoint_name);
    qemu_fdt_setprop_sized_cells(fdt, endpoint_name, "data-lanes", 1, 0, 1, 1);

    if (panel_dsi_bridge_rempte_endpoint == 0)
        panel_dsi_bridge_rempte_endpoint = qemu_fdt_alloc_phandle(fdt);
    if (panel_dsi_bridge_phandle == 0)
        panel_dsi_bridge_phandle = qemu_fdt_alloc_phandle(fdt);

    qemu_fdt_setprop_cell(fdt, endpoint_name, "remote-endpoint",
                          panel_dsi_bridge_rempte_endpoint);
    qemu_fdt_setprop_cell(fdt, endpoint_name, "phandle",
                          panel_dsi_bridge_phandle);
    /* Port tree end */

    /* Copy interrupts (remapped) */
    if (vbasedev->num_irqs) {
        uint32_t *irq_attr = g_new(uint32_t, vbasedev->num_irqs * 3);
        for (i = 0; i < vbasedev->num_irqs; i++) {
            VFIOINTp *intp;
            int irq_number = platform_bus_get_irqn(pbus, sbdev, i) +
                                                             data->irq_start;

            irq_attr[3 * i] = cpu_to_be32(GIC_FDT_IRQ_TYPE_SPI);
            irq_attr[3 * i + 1] = cpu_to_be32(irq_number);
            QLIST_FOREACH(intp, &vdev->intp_list, next) {
                if (intp->pin == i) {
                    break;
                }
            }
            if (intp->flags & VFIO_IRQ_INFO_AUTOMASKED) {
                irq_attr[3 * i + 2] = cpu_to_be32(GIC_FDT_IRQ_FLAGS_LEVEL_HI);
            } else {
                irq_attr[3 * i + 2] = cpu_to_be32(GIC_FDT_IRQ_FLAGS_EDGE_LO_HI);
            }
        }
        qemu_fdt_setprop(fdt, node_name, "interrupts",
                         irq_attr, vbasedev->num_irqs * 3 * sizeof(uint32_t));
        g_free(irq_attr);
    }

    if (vbasedev->num_clks > 0) {

        error_report("%s creating %d clk nodes",
                     __func__, vbasedev->num_clks);

        uint32_t *clk = g_new(uint32_t, vbasedev->num_clks);
        for (i = 0; i < vbasedev->num_clks;  i++) {
            clk[i] = qemu_fdt_alloc_phandle(fdt);
            fdt_build_virtio_clk_mmio_node(data, vdev, i, fdt, clk[i]);
            clk[i] = cpu_to_be32(clk[i]);
        }
        qemu_fdt_setprop(fdt, node_name, "clocks", clk,
                         vbasedev->num_clks * sizeof(uint32_t));
        g_free(clk);
    }

    if (vbasedev->num_regulators) {

        error_report("%s creating %d regulator nodes",
                     __func__, vbasedev->num_regulators);

        for (i = 0; i < vbasedev->num_regulators;  i++) {
            char *supply_name;
            uint32_t reg;

            reg = qemu_fdt_alloc_phandle(fdt);
            fdt_build_virtio_regulator_mmio_node(data, vdev, i, fdt, reg,
                                                 vbasedev->regulator_names[i],
                                                 qcom_trogdor_regulator_properties);
            supply_name = g_strdup_printf("%s-supply",
                                          vbasedev->regulator_names[i]);

            error_report("%s creating regulator name %s", __func__, supply_name);
            qemu_fdt_setprop_cell(fdt, node_name, supply_name, reg);
        }
    }

    if (vbasedev->num_pctrl_states) {

        error_report("%s creating %d pinctrl nodes",
                     __func__, vbasedev->num_pctrl_states);

        for (i = 0; i < vbasedev->num_pctrl_states;  i++) {
            char *pinctrl_name = g_strdup_printf("pinctrl-%u", i);
            uint32_t pctrl = qemu_fdt_alloc_phandle(fdt);

            fdt_build_virtio_pinctrl_mmio_node(data, vdev, i, fdt, pctrl);
            qemu_fdt_setprop_cell(fdt, node_name, pinctrl_name, pctrl);
        }
    }

    if (vbasedev->num_gpio_func > 0) {

        error_report("%s creating %d gpio nodes",
                     __func__, vbasedev->num_gpio_func);

        for (i = 0; i < vbasedev->num_gpio_func;  i++) {
            uint32_t *gpio_attr;
            unsigned int pin;
            char *func_name;
            uint32_t gpio;

            gpio = qemu_fdt_alloc_phandle(fdt);
            fdt_build_virtio_gpio_mmio_node(data, vdev, i, fdt, gpio);
            func_name = g_strdup_printf("%s-gpios",
                                          vbasedev->gpio[i].gpio_func_names);

            error_report("%s creating GPIO func name %s", __func__, func_name);
            gpio_attr = g_new(uint32_t, vbasedev->gpio[i].pin_num * 3);
            for (pin = 0; pin < vbasedev->gpio[i].pin_num; pin++) {
                uint64_t flags = fdt_xlate_flags(vbasedev->gpio[i].flags[pin]);

                error_report("%s creating GPIO func name %s pin %d flags 0x%lx xflags 0x%lx",
                        __func__, func_name, pin, (long)vbasedev->gpio[i].flags[pin],
                        (long)flags);

                gpio_attr[(pin * 3) + 0] = cpu_to_be32(gpio);
                gpio_attr[(pin * 3) + 1] = cpu_to_be32(pin);
                gpio_attr[(pin * 3) + 2] = cpu_to_be32(flags);
            }
            qemu_fdt_setprop(fdt, node_name, func_name, gpio_attr,
                             vbasedev->gpio[i].pin_num * 3 * sizeof(uint32_t));
            g_free(gpio_attr);
        }
    }

    g_free(node_name);
    return 0;
}

static int add_qcom_trogdor_ec_node(SysBusDevice *sbdev, void *opaque,
                                    ProppertList *properties)
{
    PlatformBusFDTData *data = opaque;
    const char *parent_node = data->pbus_node_name;
    PlatformBusDevice *pbus = data->pbus;
    void *fdt_guest = data->fdt;
    VFIOPlatformDevice *vdev = VFIO_PLATFORM_DEVICE(sbdev);
    VFIODevice *vbasedev = &vdev->vbasedev;
    uint32_t *irq_attr, *guest_phandle;
    char *node_name, *ec_pwm;
    VFIOINTp *intp;
    int i, irq_number;

    node_name = g_strdup_printf("%s/ec@0", parent_node);
    qemu_fdt_add_subnode(fdt_guest, node_name);
    copy_host_node_prop(vdev, fdt_guest, node_name, properties);

    /* EC-PWM child */
    ec_pwm = g_strdup_printf("%s/ec-pwm", node_name);
    qemu_fdt_add_subnode(fdt_guest, ec_pwm);
    qemu_fdt_setprop_string(fdt_guest, ec_pwm, "compatible",
                            "google,cros-ec-pwm");
    qemu_fdt_setprop_cell(fdt_guest, ec_pwm, "#pwm-cells", 1);
    if (ec_pwm_handle == 0)
        ec_pwm_handle = qemu_fdt_alloc_phandle(fdt_guest);
    qemu_fdt_setprop_cell(fdt_guest, ec_pwm, "phandle", ec_pwm_handle);
    /* EC-PWM child end */

    /* Copy interrupts (remapped) */
    if (vbasedev->num_irqs) {
        irq_attr = g_new(uint32_t, vbasedev->num_irqs * 3);
        for (i = 0; i < vbasedev->num_irqs; i++) {
            irq_number = platform_bus_get_irqn(pbus, sbdev, i) +
                         data->irq_start;
            irq_attr[3 * i] = cpu_to_be32(GIC_FDT_IRQ_TYPE_SPI);
            irq_attr[3 * i + 1] = cpu_to_be32(irq_number);
            QLIST_FOREACH(intp, &vdev->intp_list, next) {
                if (intp->pin == i) {
                    break;
                }
            }
            if (intp->flags & VFIO_IRQ_INFO_AUTOMASKED) {
                irq_attr[3 * i + 2] = cpu_to_be32(GIC_FDT_IRQ_FLAGS_LEVEL_HI);
            } else {
                irq_attr[3 * i + 2] = cpu_to_be32(GIC_FDT_IRQ_FLAGS_EDGE_LO_HI);
            }
        }
        qemu_fdt_setprop(fdt_guest, node_name, "interrupts",
                         irq_attr, vbasedev->num_irqs * 3 * sizeof(uint32_t));
        g_free(irq_attr);
    }

    guest_phandle = g_new(uint32_t, vbasedev->num_pctrl_states);
    for (i = 0; i < vbasedev->num_pctrl_states;  i++) {
        char *pinctrl_name = g_strdup_printf("pinctrl-%u", i);

        guest_phandle[i] = qemu_fdt_alloc_phandle(fdt_guest);
        fdt_build_virtio_pinctrl_mmio_node(data, vdev, i, fdt_guest,
                                           guest_phandle[i]);

        error_report("%s copying properties of regulator name %s",
                     __func__, pinctrl_name);

        qemu_fdt_setprop_cell(fdt_guest, node_name, pinctrl_name,
                              guest_phandle[i]);
    }

    g_free(guest_phandle);
    g_free(node_name);
    return 0;
}

/* DT compatible matching */
static bool vfio_platform_match(SysBusDevice *sbdev,
                                const BindingEntry *entry)
{
    VFIOPlatformDevice *vdev = VFIO_PLATFORM_DEVICE(sbdev);
    const char *compat;
    unsigned int n;

    for (n = vdev->num_compat, compat = vdev->compat; n > 0;
         n--, compat += strlen(compat) + 1) {

        error_report("%s looking for %s trying %s", __func__, compat, entry->compat);
        if (!strcmp(entry->compat, compat)) {
            return true;
        }
    }

    return false;
}

#define VFIO_PLATFORM_BINDING(compat, add_fn, prop) \
    {TYPE_VFIO_PLATFORM, (compat), (add_fn), vfio_platform_match, (prop)}

/* Device type based matching */
static bool type_match(SysBusDevice *sbdev, const BindingEntry *entry)
{
    return !strcmp(object_get_typename(OBJECT(sbdev)), entry->typename);
}

#define TYPE_BINDING(type, add_fn) {(type), NULL, (add_fn), NULL, NULL}

static int no_fdt_node(SysBusDevice *sbdev, void *opaque,
                       ProppertList *properties)
{
    return 0;
}

static const BindingEntry geniqup_grand_child_bindings[] = {
#ifdef CONFIG_LINUX
    VFIO_PLATFORM_BINDING("qcom,geni-se-qup", no_fdt_node, NULL),

    VFIO_PLATFORM_BINDING("qcom,geni-uart", no_fdt_node, NULL),
    VFIO_PLATFORM_BINDING("qcom,wcn3991-bt",
            add_qcom_trogdor_bluetooth_node, qcom_trogdor_geni_bluetooth_prop),

    VFIO_PLATFORM_BINDING("qcom,geni-i2c", no_fdt_node, NULL),
    VFIO_PLATFORM_BINDING("hid-over-i2c",
            add_qcom_trogdor_touchscreen_node, qcom_trogdor_geni_touchscreen_prop),
    VFIO_PLATFORM_BINDING("ti,sn65dsi86",
            add_qcom_trogdor_dsi_bridge_node, qcom_trogdor_geni_dsi_bridge_prop),

    VFIO_PLATFORM_BINDING("qcom,geni-spi", no_fdt_node, NULL),
    VFIO_PLATFORM_BINDING("google,cros-ec-spi",
            add_qcom_trogdor_ec_node, qcom_trogdor_geni_ec_prop),
#endif
};

static void add_fdt_child_geniqup_grand_child_node(SysBusDevice *sbdev, void *opaque)
{
    int i, ret;

    for (i = 0; i < ARRAY_SIZE(geniqup_grand_child_bindings); i++) {
        const BindingEntry *iter = &geniqup_grand_child_bindings[i];

        if (type_match(sbdev, iter)) {
            if (!iter->match_fn || iter->match_fn(sbdev, iter)) {
                ret = iter->add_fn(sbdev, opaque, iter->properties);
                assert(!ret);
                return;
            }
        }
    }
}

static void add_fdt_node(SysBusDevice *sbdev, void *opaque);

static int add_qcom_trogdor_geni_uart_node(SysBusDevice *sbdev, void *opaque,
                                   ProppertList *properties)
{
    PlatformBusFDTData *data = opaque;
    const char *parent_node = data->pbus_node_name;
    PlatformBusDevice *pbus = data->pbus;
    void *fdt_guest = data->fdt;
    VFIOPlatformDevice *vdev = VFIO_PLATFORM_DEVICE(sbdev);
    VFIODevice *vbasedev = &vdev->vbasedev;
    uint32_t *reg_attr, *irq_attr, *guest_clk_phandle;
    uint32_t phandle;
    char *node_name, *alias_node;
    uint64_t mmio_base;
    int i, irq_number;
    VFIOINTp *intp;
    PlatformBusFDTData child_data = {
        .fdt = fdt_guest,
        .irq_start = data->irq_start,
        .pbus = pbus,
        .parent = vdev,
        .bindings = geniqup_grand_child_bindings,
        .binding_size = ARRAY_SIZE(geniqup_grand_child_bindings),
    };

    mmio_base = platform_bus_get_mmio_addr(pbus, sbdev, 0);
    node_name = g_strdup_printf("%s/%s@%" PRIx64, parent_node,
                                vbasedev->name, mmio_base);

    qemu_fdt_add_subnode(fdt_guest, node_name);
    copy_host_node_prop(vdev, fdt_guest, node_name, properties);

    alias_node = g_strdup_printf("/aliases");
    qemu_fdt_add_subnode(fdt_guest, alias_node);
    qemu_fdt_setprop_string(fdt_guest, alias_node, "hsuart0", node_name);

    /* Copy reg (remapped) */
    reg_attr = g_new(uint32_t, vbasedev->num_regions * 2);
    for (i = 0; i < vbasedev->num_regions; i++) {
        mmio_base = platform_bus_get_mmio_addr(pbus, sbdev, i);
        reg_attr[2 * i] = cpu_to_be32(mmio_base);
        reg_attr[2 * i + 1] = cpu_to_be32(
                                memory_region_size(vdev->regions[i]->mem));
    }
    qemu_fdt_setprop(fdt_guest, node_name, "reg", reg_attr,
                     vbasedev->num_regions * 2 * sizeof(uint32_t));

    /* Copy interrupts (remapped) */
    if (vbasedev->num_irqs) {
        irq_attr = g_new(uint32_t, vbasedev->num_irqs * 3);
        for (i = 0; i < vbasedev->num_irqs; i++) {
            irq_number = platform_bus_get_irqn(pbus, sbdev, i) +
                         data->irq_start;
            irq_attr[3 * i] = cpu_to_be32(GIC_FDT_IRQ_TYPE_SPI);
            irq_attr[3 * i + 1] = cpu_to_be32(irq_number);
            QLIST_FOREACH(intp, &vdev->intp_list, next) {
                if (intp->pin == i) {
                    break;
                }
            }
            if (intp->flags & VFIO_IRQ_INFO_AUTOMASKED) {
                irq_attr[3 * i + 2] = cpu_to_be32(GIC_FDT_IRQ_FLAGS_LEVEL_HI);
            } else {
                irq_attr[3 * i + 2] = cpu_to_be32(GIC_FDT_IRQ_FLAGS_EDGE_LO_HI);
            }
        }
        qemu_fdt_setprop(fdt_guest, node_name, "interrupts",
                         irq_attr, vbasedev->num_irqs * 3 * sizeof(uint32_t));
        g_free(irq_attr);
    }

    guest_clk_phandle = g_new(uint32_t, vbasedev->num_clks);
    for (i = 0; i < vbasedev->num_clks;  i++) {
        guest_clk_phandle[i] = qemu_fdt_alloc_phandle(fdt_guest);
        fdt_build_virtio_clk_mmio_node(data, vdev, i, fdt_guest,
                                       guest_clk_phandle[i]);
        guest_clk_phandle[i] = cpu_to_be32(guest_clk_phandle[i]);
    }
    if (vbasedev->num_clks > 0) {
        qemu_fdt_setprop(fdt_guest, node_name, "clocks", guest_clk_phandle,
                         vbasedev->num_clks * sizeof(uint32_t));
    }
    g_free(guest_clk_phandle);

    guest_clk_phandle = g_new(uint32_t, vbasedev->num_regulators);
    for (i = 0; i < vbasedev->num_regulators;  i++) {
        char *supply_name;

        guest_clk_phandle[i] = qemu_fdt_alloc_phandle(fdt_guest);
        fdt_build_virtio_regulator_mmio_node(data, vdev, i, fdt_guest,
                                             guest_clk_phandle[i],
                                             vbasedev->regulator_names[i],
                                             qcom_trogdor_regulator_properties);
        supply_name = g_strdup_printf("%s-supply",
                                      vbasedev->regulator_names[i]);

        error_report("%s copying properties of regulator name %s",
                     __func__, supply_name);

        qemu_fdt_setprop_cell(fdt_guest, node_name, supply_name,
                              guest_clk_phandle[i]);
    }

    g_free(guest_clk_phandle);

    guest_clk_phandle = g_new(uint32_t, vbasedev->num_interconnects * 4);

    error_report("%s creating %d interconnect nodes",
                 __func__, vbasedev->num_interconnects);

    for (i = 0; i < vbasedev->num_interconnects;  i++) {
        phandle = qemu_fdt_alloc_phandle(fdt_guest);
        fdt_build_virtio_inter_mmio_node(data, vdev, i, fdt_guest, phandle);
        /* src node points to dst node within the same provider */
        guest_clk_phandle[(i * 4) + 0] = cpu_to_be32(phandle);
        guest_clk_phandle[(i * 4) + 1] = cpu_to_be32(0); // node id
        guest_clk_phandle[(i * 4) + 2] = cpu_to_be32(phandle);
        guest_clk_phandle[(i * 4) + 3] = cpu_to_be32(1); // node id
    }
    if (vbasedev->num_interconnects > 0) {
        qemu_fdt_setprop(fdt_guest, node_name, "interconnects", guest_clk_phandle,
                         vbasedev->num_interconnects * sizeof(uint32_t) * 4);
    }

    g_free(guest_clk_phandle);

    guest_clk_phandle = g_new(uint32_t, vbasedev->num_phys);
    for (i = 0; i < vbasedev->num_phys;  i++) {
        guest_clk_phandle[i] = qemu_fdt_alloc_phandle(fdt_guest);
        fdt_build_virtio_phy_mmio_node(data, vdev, i, fdt_guest,
                                       guest_clk_phandle[i]);
        guest_clk_phandle[i] = cpu_to_be32(guest_clk_phandle[i]);
    }
    if (vbasedev->num_phys > 0) {
        qemu_fdt_setprop(fdt_guest, node_name, "phys", guest_clk_phandle,
                         vbasedev->num_phys * sizeof(uint32_t));
    }
    g_free(guest_clk_phandle);

    guest_clk_phandle = g_new(uint32_t, vbasedev->num_pctrl_states);
    for (i = 0; i < vbasedev->num_pctrl_states;  i++) {
        char *pinctrl_name = g_strdup_printf("pinctrl-%u", i);

        guest_clk_phandle[i] = qemu_fdt_alloc_phandle(fdt_guest);
        fdt_build_virtio_pinctrl_mmio_node(data, vdev, i, fdt_guest,
                                           guest_clk_phandle[i]);

        error_report("%s copying properties of regulator name %s",
                     __func__, pinctrl_name);

        qemu_fdt_setprop_cell(fdt_guest, node_name, pinctrl_name,
                              guest_clk_phandle[i]);
    }

    g_free(guest_clk_phandle);
    g_free(reg_attr);

    error_report("\nCreating child node - start");

    child_data.pbus_node_name = node_name;
    foreach_dynamic_sysbus_device(add_fdt_node, &child_data);

    error_report("Creating child node - end\n");

    g_free(node_name);
    return 0;
}

static int add_qcom_trogdor_geni_i2c_node(SysBusDevice *sbdev, void *opaque,
                                   ProppertList *properties)
{
    PlatformBusFDTData *data = opaque;
    const char *parent_node = data->pbus_node_name;
    PlatformBusDevice *pbus = data->pbus;
    void *fdt_guest = data->fdt;
    VFIOPlatformDevice *vdev = VFIO_PLATFORM_DEVICE(sbdev);
    VFIODevice *vbasedev = &vdev->vbasedev;
    uint32_t *reg_attr, *irq_attr, *guest_clk_phandle;
    uint32_t phandle;
    char *node_name;
    uint64_t mmio_base;
    int i, irq_number;
    VFIOINTp *intp;
    PlatformBusFDTData child_data = {
        .fdt = fdt_guest,
        .irq_start = data->irq_start,
        .pbus = pbus,
        .parent = vdev,
        .bindings = geniqup_grand_child_bindings,
        .binding_size = ARRAY_SIZE(geniqup_grand_child_bindings),
    };

    mmio_base = platform_bus_get_mmio_addr(pbus, sbdev, 0);
    node_name = g_strdup_printf("%s/%s@%" PRIx64, parent_node,
                                vbasedev->name, mmio_base);

    qemu_fdt_add_subnode(fdt_guest, node_name);
    copy_host_node_prop(vdev, fdt_guest, node_name, properties);

    /* Copy reg (remapped) */
    reg_attr = g_new(uint32_t, vbasedev->num_regions * 2);
    for (i = 0; i < vbasedev->num_regions; i++) {
        mmio_base = platform_bus_get_mmio_addr(pbus, sbdev, i);
        reg_attr[2 * i] = cpu_to_be32(mmio_base);
        reg_attr[2 * i + 1] = cpu_to_be32(
                                memory_region_size(vdev->regions[i]->mem));
    }
    qemu_fdt_setprop(fdt_guest, node_name, "reg", reg_attr,
                     vbasedev->num_regions * 2 * sizeof(uint32_t));

    /* Copy interrupts (remapped) */
    if (vbasedev->num_irqs) {
        irq_attr = g_new(uint32_t, vbasedev->num_irqs * 3);
        for (i = 0; i < vbasedev->num_irqs; i++) {
            irq_number = platform_bus_get_irqn(pbus, sbdev, i) +
                         data->irq_start;
            irq_attr[3 * i] = cpu_to_be32(GIC_FDT_IRQ_TYPE_SPI);
            irq_attr[3 * i + 1] = cpu_to_be32(irq_number);
            QLIST_FOREACH(intp, &vdev->intp_list, next) {
                if (intp->pin == i) {
                    break;
                }
            }
            if (intp->flags & VFIO_IRQ_INFO_AUTOMASKED) {
                irq_attr[3 * i + 2] = cpu_to_be32(GIC_FDT_IRQ_FLAGS_LEVEL_HI);
            } else {
                irq_attr[3 * i + 2] = cpu_to_be32(GIC_FDT_IRQ_FLAGS_EDGE_LO_HI);
            }
        }
        qemu_fdt_setprop(fdt_guest, node_name, "interrupts",
                         irq_attr, vbasedev->num_irqs * 3 * sizeof(uint32_t));
        g_free(irq_attr);
    }

    guest_clk_phandle = g_new(uint32_t, vbasedev->num_clks);
    for (i = 0; i < vbasedev->num_clks;  i++) {
        guest_clk_phandle[i] = qemu_fdt_alloc_phandle(fdt_guest);
        fdt_build_virtio_clk_mmio_node(data, vdev, i, fdt_guest,
                                       guest_clk_phandle[i]);
        guest_clk_phandle[i] = cpu_to_be32(guest_clk_phandle[i]);
    }
    if (vbasedev->num_clks > 0) {
        qemu_fdt_setprop(fdt_guest, node_name, "clocks", guest_clk_phandle,
                         vbasedev->num_clks * sizeof(uint32_t));
    }
    g_free(guest_clk_phandle);

    guest_clk_phandle = g_new(uint32_t, vbasedev->num_regulators);
    for (i = 0; i < vbasedev->num_regulators;  i++) {
        char *supply_name;

        guest_clk_phandle[i] = qemu_fdt_alloc_phandle(fdt_guest);
        fdt_build_virtio_regulator_mmio_node(data, vdev, i, fdt_guest,
                                             guest_clk_phandle[i],
                                             vbasedev->regulator_names[i],
                                             qcom_trogdor_regulator_properties);
        supply_name = g_strdup_printf("%s-supply",
                                      vbasedev->regulator_names[i]);

        error_report("%s copying properties of regulator name %s",
                     __func__, supply_name);

        qemu_fdt_setprop_cell(fdt_guest, node_name, supply_name,
                              guest_clk_phandle[i]);
    }

    g_free(guest_clk_phandle);

    guest_clk_phandle = g_new(uint32_t, vbasedev->num_interconnects * 4);

    error_report("%s creating %d interconnect nodes",
                 __func__, vbasedev->num_interconnects);

    for (i = 0; i < vbasedev->num_interconnects;  i++) {
        phandle = qemu_fdt_alloc_phandle(fdt_guest);
        fdt_build_virtio_inter_mmio_node(data, vdev, i, fdt_guest, phandle);
        /* src node points to dst node within the same provider */
        guest_clk_phandle[(i * 4) + 0] = cpu_to_be32(phandle);
        guest_clk_phandle[(i * 4) + 1] = cpu_to_be32(0); // node id
        guest_clk_phandle[(i * 4) + 2] = cpu_to_be32(phandle);
        guest_clk_phandle[(i * 4) + 3] = cpu_to_be32(1); // node id
    }
    if (vbasedev->num_interconnects > 0) {
        qemu_fdt_setprop(fdt_guest, node_name, "interconnects", guest_clk_phandle,
                         vbasedev->num_interconnects * sizeof(uint32_t) * 4);
    }

    g_free(guest_clk_phandle);

    guest_clk_phandle = g_new(uint32_t, vbasedev->num_phys);
    for (i = 0; i < vbasedev->num_phys;  i++) {
        guest_clk_phandle[i] = qemu_fdt_alloc_phandle(fdt_guest);
        fdt_build_virtio_phy_mmio_node(data, vdev, i, fdt_guest,
                                       guest_clk_phandle[i]);
        guest_clk_phandle[i] = cpu_to_be32(guest_clk_phandle[i]);
    }
    if (vbasedev->num_phys > 0) {
        qemu_fdt_setprop(fdt_guest, node_name, "phys", guest_clk_phandle,
                         vbasedev->num_phys * sizeof(uint32_t));
    }
    g_free(guest_clk_phandle);

    guest_clk_phandle = g_new(uint32_t, vbasedev->num_pctrl_states);
    for (i = 0; i < vbasedev->num_pctrl_states;  i++) {
        char *pinctrl_name = g_strdup_printf("pinctrl-%u", i);

        guest_clk_phandle[i] = qemu_fdt_alloc_phandle(fdt_guest);
        fdt_build_virtio_pinctrl_mmio_node(data, vdev, i, fdt_guest,
                                           guest_clk_phandle[i]);

        error_report("%s copying properties of regulator name %s",
                     __func__, pinctrl_name);

        qemu_fdt_setprop_cell(fdt_guest, node_name, pinctrl_name,
                              guest_clk_phandle[i]);
    }

    g_free(guest_clk_phandle);
    g_free(reg_attr);

    error_report("\nCreating child node - start");

    child_data.pbus_node_name = node_name;
    foreach_dynamic_sysbus_device(add_fdt_node, &child_data);

    error_report("Creating child node - end\n");

    g_free(node_name);
    return 0;
}

static const BindingEntry child_bindings[] = {
#ifdef CONFIG_LINUX
    VFIO_PLATFORM_BINDING("snps,dwc3",
            add_qcom_trogdor_dwc3_child_node, qcom_trogdor_xhci_properties),
    /* GENIQUP */
    VFIO_PLATFORM_BINDING("qcom,geni-se-qup", no_fdt_node, NULL),

    VFIO_PLATFORM_BINDING("qcom,geni-uart",
            add_qcom_trogdor_geni_uart_node, qcom_trogdor_geni_uart_prop),
    VFIO_PLATFORM_BINDING("qcom,wcn3991-bt", no_fdt_node, NULL),

    VFIO_PLATFORM_BINDING("qcom,geni-i2c",
            add_qcom_trogdor_geni_i2c_node, qcom_trogdor_geni_i2c_prop),
    VFIO_PLATFORM_BINDING("hid-over-i2c", no_fdt_node, NULL),
    VFIO_PLATFORM_BINDING("ti,sn65dsi86", no_fdt_node, NULL),

    VFIO_PLATFORM_BINDING("qcom,geni-spi",
            add_qcom_trogdor_geni_i2c_node, qcom_trogdor_geni_spi_prop),
    VFIO_PLATFORM_BINDING("google,cros-ec-spi", no_fdt_node, NULL),
#endif
};

static void add_fdt_child_node(SysBusDevice *sbdev, void *opaque)
{
    int i, ret;

    for (i = 0; i < ARRAY_SIZE(child_bindings); i++) {
        const BindingEntry *iter = &child_bindings[i];

        if (type_match(sbdev, iter)) {
            if (!iter->match_fn || iter->match_fn(sbdev, iter)) {
                ret = iter->add_fn(sbdev, opaque, iter->properties);
                assert(!ret);
                return;
            }
        }
    }
}

static uint32_t mdss_irq_parent, dsi_phy;

static int add_qcom_trogdor_mdss_child_node(SysBusDevice *sbdev, void *opaque,
                                     ProppertList *properties)
{
    PlatformBusFDTData *data = opaque;
    const char *parent_node = data->pbus_node_name;
    PlatformBusDevice *pbus = data->pbus;
    void *fdt = data->fdt;
    VFIOPlatformDevice *vdev = VFIO_PLATFORM_DEVICE(sbdev);
    VFIODevice *vbasedev = &vdev->vbasedev;
    char *node_name, *ports_name, *port_name, *endpoint_name;
    uint64_t mmio_base;
    int i;

    error_report("%s 0", __func__);

    if (vbasedev->num_regions > 0) {
        mmio_base = platform_bus_get_mmio_addr(pbus, sbdev, 0);
        node_name = g_strdup_printf("%s/%s@%" PRIx64, parent_node,
                                    vbasedev->name, mmio_base);
    } else
        node_name = g_strdup_printf("%s/%s", parent_node, vbasedev->name);

    error_report("%s 1", __func__);

    qemu_fdt_add_subnode(fdt, node_name);
    copy_host_node_prop(vdev, fdt, node_name, properties);

    if (vfio_platform_match_raw(sbdev, "qcom,sc7180-dpu")) {

        if (mdss_irq_parent == 0)
            mdss_irq_parent = qemu_fdt_alloc_phandle(fdt);

        qemu_fdt_setprop_cell(fdt, node_name, "interrupt-parent", mdss_irq_parent);

        /* Port tree */
        ports_name = g_strdup_printf("%s/%s", node_name, "ports");
        qemu_fdt_add_subnode(fdt, ports_name);
        qemu_fdt_setprop_cell(fdt, ports_name, "#address-cells", 1);
        qemu_fdt_setprop_cell(fdt, ports_name, "#size-cells", 0);

        /* First port, DSI reference */
        port_name = g_strdup_printf("%s/%s", ports_name, "port@0");
        qemu_fdt_add_subnode(fdt, port_name);
        qemu_fdt_setprop_cell(fdt, port_name, "reg", 0);

        endpoint_name = g_strdup_printf("%s/%s", port_name, "endpoint");
        qemu_fdt_add_subnode(fdt, endpoint_name);

        if (mdss_dsi_rempte_endpoint == 0)
            mdss_dsi_rempte_endpoint = qemu_fdt_alloc_phandle(fdt);
        if (mdss_dsi_phandle == 0)
            mdss_dsi_phandle = qemu_fdt_alloc_phandle(fdt);

        qemu_fdt_setprop_cell(fdt, endpoint_name, "remote-endpoint",
                              mdss_dsi_rempte_endpoint);
        qemu_fdt_setprop_cell(fdt, endpoint_name, "phandle",
                              mdss_dsi_phandle);

        /* Second port, DP reference */
        port_name = g_strdup_printf("%s/%s", ports_name, "port@2");
        qemu_fdt_add_subnode(fdt, port_name);
        qemu_fdt_setprop_cell(fdt, port_name, "reg", 2);

        endpoint_name = g_strdup_printf("%s/%s", port_name, "endpoint");
        qemu_fdt_add_subnode(fdt, endpoint_name);

        if (mdss_dp_rempte_endpoint == 0)
            mdss_dp_rempte_endpoint = qemu_fdt_alloc_phandle(fdt);
        if (mdss_dp_phandle == 0)
            mdss_dp_phandle = qemu_fdt_alloc_phandle(fdt);

        qemu_fdt_setprop_cell(fdt, endpoint_name, "remote-endpoint",
                              mdss_dp_rempte_endpoint);
        qemu_fdt_setprop_cell(fdt, endpoint_name, "phandle",
                              mdss_dp_phandle);
        /* Port tree end */
    } else if (vfio_platform_match_raw(sbdev, "qcom,mdss-dsi-ctrl")) {

        if (mdss_irq_parent == 0)
            mdss_irq_parent = qemu_fdt_alloc_phandle(fdt);

        qemu_fdt_setprop_cell(fdt, node_name, "interrupt-parent", mdss_irq_parent);

        if (dsi_phy == 0)
            dsi_phy = qemu_fdt_alloc_phandle(fdt);

        qemu_fdt_setprop_cell(fdt, node_name, "phys", dsi_phy);

        /* Port tree */
        ports_name = g_strdup_printf("%s/%s", node_name, "ports");
        qemu_fdt_add_subnode(fdt, ports_name);
        qemu_fdt_setprop_cell(fdt, ports_name, "#address-cells", 1);
        qemu_fdt_setprop_cell(fdt, ports_name, "#size-cells", 0);

        /* First port, MDSS reference */
        port_name = g_strdup_printf("%s/%s", ports_name, "port@0");
        qemu_fdt_add_subnode(fdt, port_name);
        qemu_fdt_setprop_cell(fdt, port_name, "reg", 0);

        endpoint_name = g_strdup_printf("%s/%s", port_name, "endpoint");
        qemu_fdt_add_subnode(fdt, endpoint_name);

        if (mdss_dsi_rempte_endpoint == 0)
            mdss_dsi_rempte_endpoint = qemu_fdt_alloc_phandle(fdt);
        if (mdss_dsi_phandle == 0)
            mdss_dsi_phandle = qemu_fdt_alloc_phandle(fdt);

        qemu_fdt_setprop_cell(fdt, endpoint_name, "remote-endpoint",
                              mdss_dsi_phandle);
        qemu_fdt_setprop_cell(fdt, endpoint_name, "phandle",
                              mdss_dsi_rempte_endpoint);

        /* Second port, DSI bridge reference */
        port_name = g_strdup_printf("%s/%s", ports_name, "port@1");
        qemu_fdt_add_subnode(fdt, port_name);
        qemu_fdt_setprop_cell(fdt, port_name, "reg", 1);

        endpoint_name = g_strdup_printf("%s/%s", port_name, "endpoint");
        qemu_fdt_add_subnode(fdt, endpoint_name);

        if (mdss_dsi_bridge_rempte_endpoint == 0)
            mdss_dsi_bridge_rempte_endpoint = qemu_fdt_alloc_phandle(fdt);
        if (mdss_dsi_bridge_phandle == 0)
            mdss_dsi_bridge_phandle = qemu_fdt_alloc_phandle(fdt);

        qemu_fdt_setprop_cell(fdt, endpoint_name, "remote-endpoint",
                              mdss_dsi_bridge_phandle);
        qemu_fdt_setprop_cell(fdt, endpoint_name, "phandle",
                              mdss_dsi_bridge_rempte_endpoint);
        qemu_fdt_setprop_sized_cells(fdt, endpoint_name, "data-lanes", 1, 0, 1, 1, 1, 2, 1, 3);
        /* Port tree end */
    } else if (vfio_platform_match_raw(sbdev, "qcom,dsi-phy-10nm")) {

        if (dsi_phy == 0)
            dsi_phy = qemu_fdt_alloc_phandle(fdt);

        qemu_fdt_setprop_cell(fdt, node_name, "phandle", dsi_phy);
    } else if (vfio_platform_match_raw(sbdev, "qcom,sc7180-dp")) {

        if (mdss_irq_parent == 0)
            mdss_irq_parent = qemu_fdt_alloc_phandle(fdt);

        qemu_fdt_setprop_cell(fdt, node_name, "interrupt-parent", mdss_irq_parent);

        /* Port tree */
        ports_name = g_strdup_printf("%s/%s", node_name, "ports");
        qemu_fdt_add_subnode(fdt, ports_name);
        qemu_fdt_setprop_cell(fdt, ports_name, "#address-cells", 1);
        qemu_fdt_setprop_cell(fdt, ports_name, "#size-cells", 0);

        /* First port, MDSS reference */
        port_name = g_strdup_printf("%s/%s", ports_name, "port@0");
        qemu_fdt_add_subnode(fdt, port_name);
        qemu_fdt_setprop_cell(fdt, port_name, "reg", 0);

        endpoint_name = g_strdup_printf("%s/%s", port_name, "endpoint");
        qemu_fdt_add_subnode(fdt, endpoint_name);

        if (mdss_dp_rempte_endpoint == 0)
            mdss_dp_rempte_endpoint = qemu_fdt_alloc_phandle(fdt);
        if (mdss_dp_phandle == 0)
            mdss_dp_phandle = qemu_fdt_alloc_phandle(fdt);

        qemu_fdt_setprop_cell(fdt, endpoint_name, "remote-endpoint",
                              mdss_dp_phandle);
        qemu_fdt_setprop_cell(fdt, endpoint_name, "phandle",
                              mdss_dp_rempte_endpoint);

        /* Second port, DSI bridge reference */
        port_name = g_strdup_printf("%s/%s", ports_name, "port@1");
        qemu_fdt_add_subnode(fdt, port_name);
        qemu_fdt_setprop_cell(fdt, port_name, "reg", 1);

        endpoint_name = g_strdup_printf("%s/%s", port_name, "endpoint");
        qemu_fdt_add_subnode(fdt, endpoint_name);

        /* Port tree end */
    }

    error_report("%s 2", __func__);
    /* Copy reg (remapped) */
    if (vbasedev->num_regions > 0) {
        uint32_t *reg_attr = g_new(uint32_t, vbasedev->num_regions * 2);
        for (i = 0; i < vbasedev->num_regions; i++) {
            mmio_base = platform_bus_get_mmio_addr(pbus, sbdev, i);
            reg_attr[2 * i] = cpu_to_be32(mmio_base);
            reg_attr[2 * i + 1] = cpu_to_be32(
                                    memory_region_size(vdev->regions[i]->mem));
        }
        qemu_fdt_setprop(fdt, node_name, "reg", reg_attr,
                         vbasedev->num_regions * 2 * sizeof(uint32_t));
        g_free(reg_attr);
    }

    error_report("%s 3", __func__);

    /* Copy interrupts (remapped) */
    if (vbasedev->num_irqs) {
        uint32_t *irq_attr = g_new(uint32_t, vbasedev->num_irqs * 3);
        for (i = 0; i < vbasedev->num_irqs; i++) {
            VFIOINTp *intp;
            int irq_number = platform_bus_get_irqn(pbus, sbdev, i) +
                                                             data->irq_start;

            irq_attr[3 * i] = cpu_to_be32(GIC_FDT_IRQ_TYPE_SPI);
            irq_attr[3 * i + 1] = cpu_to_be32(irq_number);
            QLIST_FOREACH(intp, &vdev->intp_list, next) {
                if (intp->pin == i) {
                    break;
                }
            }
            if (intp->flags & VFIO_IRQ_INFO_AUTOMASKED) {
                irq_attr[3 * i + 2] = cpu_to_be32(GIC_FDT_IRQ_FLAGS_LEVEL_HI);
            } else {
                irq_attr[3 * i + 2] = cpu_to_be32(GIC_FDT_IRQ_FLAGS_EDGE_LO_HI);
            }
        }
        qemu_fdt_setprop(fdt, node_name, "interrupts",
                         irq_attr, vbasedev->num_irqs * 3 * sizeof(uint32_t));
        g_free(irq_attr);
    }

    error_report("%s 4", __func__);

    if (vbasedev->num_clks > 0) {

        error_report("%s creating %d clk nodes",
                     __func__, vbasedev->num_clks);

        uint32_t *clk = g_new(uint32_t, vbasedev->num_clks);
        for (i = 0; i < vbasedev->num_clks;  i++) {
            clk[i] = qemu_fdt_alloc_phandle(fdt);
            fdt_build_virtio_clk_mmio_node(data, vdev, i, fdt, clk[i]);
            clk[i] = cpu_to_be32(clk[i]);
        }

        if (vfio_platform_match_raw(sbdev, "qcom,sc7180-dpu")) {
            uint32_t assigned_clk[8];
            uint32_t clk_attr[11];

            /* Clocks */
            if (disp_clk_handle == 0)
                disp_clk_handle = qemu_fdt_alloc_phandle(fdt);

            clk_attr[0] = clk[0];
            clk_attr[1] = cpu_to_be32(disp_clk_handle);
            clk_attr[2] = cpu_to_be32(0x02);
            clk_attr[3] = cpu_to_be32(disp_clk_handle);
            clk_attr[4] = cpu_to_be32(0x1a);
            clk_attr[5] = cpu_to_be32(disp_clk_handle);
            clk_attr[6] = cpu_to_be32(0x16);
            clk_attr[7] = cpu_to_be32(disp_clk_handle);
            clk_attr[8] = cpu_to_be32(0x14);
            clk_attr[9] = cpu_to_be32(disp_clk_handle);
            clk_attr[10] = cpu_to_be32(0x1e);
            qemu_fdt_setprop(fdt, node_name, "clocks", clk_attr,
                             11 * sizeof(uint32_t));

            assigned_clk[0] = cpu_to_be32(disp_clk_handle);
            assigned_clk[1] = cpu_to_be32(0x14);
            assigned_clk[2] = cpu_to_be32(disp_clk_handle);
            assigned_clk[3] = cpu_to_be32(0x1e);
            assigned_clk[4] = cpu_to_be32(disp_clk_handle);
            assigned_clk[5] = cpu_to_be32(0x1a);
            assigned_clk[6] = cpu_to_be32(disp_clk_handle);
            assigned_clk[7] = cpu_to_be32(0x02);
            qemu_fdt_setprop(fdt, node_name, "assigned-clocks",
                             assigned_clk, 8 * sizeof(uint32_t));
        } else if (vfio_platform_match_raw(sbdev, "qcom,mdss-dsi-ctrl")) {
            uint32_t clk_attr[11];

            /* Clocks */
            if (disp_clk_handle == 0)
                disp_clk_handle = qemu_fdt_alloc_phandle(fdt);

            clk_attr[0] = cpu_to_be32(disp_clk_handle);
            clk_attr[1] = cpu_to_be32(0x04);
            clk_attr[2] = cpu_to_be32(disp_clk_handle);
            clk_attr[3] = cpu_to_be32(0x07);
            clk_attr[4] = cpu_to_be32(disp_clk_handle);
            clk_attr[5] = cpu_to_be32(0x18);
            clk_attr[6] = cpu_to_be32(disp_clk_handle);
            clk_attr[7] = cpu_to_be32(0x12);
            clk_attr[8] = cpu_to_be32(disp_clk_handle);
            clk_attr[9] = cpu_to_be32(0x02);
            clk_attr[10] = clk[0];
            qemu_fdt_setprop(fdt, node_name, "clocks", clk_attr,
                             11 * sizeof(uint32_t));
        } else if (vfio_platform_match_raw(sbdev, "qcom,dsi-phy-10nm")) {
            uint32_t clk_attr[3];

            /* Clocks */
            if (disp_clk_handle == 0)
                disp_clk_handle = qemu_fdt_alloc_phandle(fdt);

            clk_attr[0] = cpu_to_be32(disp_clk_handle);
            clk_attr[1] = cpu_to_be32(0x02);
            clk_attr[2] = clk[0];
            qemu_fdt_setprop(fdt, node_name, "clocks", clk_attr,
                             3 * sizeof(uint32_t));
        } else if (vfio_platform_match_raw(sbdev, "qcom,sc7180-dp")) {
            uint32_t clk_attr[10];

            /* Clocks */
            if (disp_clk_handle == 0)
                disp_clk_handle = qemu_fdt_alloc_phandle(fdt);

            clk_attr[0] = cpu_to_be32(disp_clk_handle);
            clk_attr[1] = cpu_to_be32(0x02);
            clk_attr[2] = cpu_to_be32(disp_clk_handle);
            clk_attr[3] = cpu_to_be32(0x08);
            clk_attr[4] = cpu_to_be32(disp_clk_handle);
            clk_attr[5] = cpu_to_be32(0x0c);
            clk_attr[6] = cpu_to_be32(disp_clk_handle);
            clk_attr[7] = cpu_to_be32(0x0f);
            clk_attr[8] = cpu_to_be32(disp_clk_handle);
            clk_attr[9] = cpu_to_be32(0x10);
            qemu_fdt_setprop(fdt, node_name, "clocks", clk_attr,
                             10 * sizeof(uint32_t));
        }

        g_free(clk);
    }

    error_report("%s 5", __func__);
    if (vbasedev->num_regulators) {

        error_report("%s creating %d regulator nodes",
                     __func__, vbasedev->num_regulators);

        for (i = 0; i < vbasedev->num_regulators;  i++) {
            char *supply_name;
            uint32_t reg;

            reg = qemu_fdt_alloc_phandle(fdt);

            fdt_build_virtio_regulator_mmio_node(data, vdev, i, fdt, reg,
                                                 vbasedev->regulator_names[i],
                                                 qcom_trogdor_regulator_properties);

            supply_name = g_strdup_printf("%s-supply",
                                          vbasedev->regulator_names[i]);

            error_report("%s creating regulator name %s", __func__, supply_name);
            qemu_fdt_setprop_cell(fdt, node_name, supply_name, reg);
        }
    }

    error_report("%s 6", __func__);
    if (vbasedev->num_gpio_func > 0) {
        error_report("%s creating %d gpio nodes",
                     __func__, vbasedev->num_gpio_func);
        for (i = 0; i < vbasedev->num_gpio_func;  i++) {
            uint32_t *gpio_attr;
            unsigned int pin;
            char *func_name;
            uint32_t gpio;

            gpio = qemu_fdt_alloc_phandle(fdt);
            fdt_build_virtio_gpio_mmio_node(data, vdev, i, fdt, gpio);
            func_name = g_strdup_printf("%s-gpios",
                                          vbasedev->gpio[i].gpio_func_names);

            error_report("%s creating GPIO func name %s", __func__, func_name);
            gpio_attr = g_new(uint32_t, vbasedev->gpio[i].pin_num * 3);
            for (pin = 0; pin < vbasedev->gpio[i].pin_num; pin++) {
                uint64_t flags = fdt_xlate_flags(vbasedev->gpio[i].flags[pin]);

                error_report("%s creating GPIO func name %s pin %d flags 0x%lx xflags 0x%lx",
                        __func__, func_name, pin, (long)vbasedev->gpio[i].flags[pin],
                        (long)flags);

                gpio_attr[(pin * 3) + 0] = cpu_to_be32(gpio);
                gpio_attr[(pin * 3) + 1] = cpu_to_be32(pin);
                gpio_attr[(pin * 3) + 2] = cpu_to_be32(flags);
            }
            qemu_fdt_setprop(fdt, node_name, func_name, gpio_attr,
                             vbasedev->gpio[i].pin_num * 3 * sizeof(uint32_t));
            g_free(gpio_attr);
        }
    }
    error_report("%s 7", __func__);

    if (vbasedev->num_interconnects) {
        uint32_t *inter_attr = g_new(uint32_t, vbasedev->num_interconnects * 4);
        error_report("%s creating %d interconnect nodes",
                     __func__, vbasedev->num_interconnects);

        for (i = 0; i < vbasedev->num_interconnects;  i++) {
            uint32_t inter;

            inter = qemu_fdt_alloc_phandle(fdt);
            fdt_build_virtio_inter_mmio_node(data, vdev, i, fdt, inter);
            /* src node points to dst node within the same provider */
            inter_attr[(i * 4) + 0] = cpu_to_be32(inter);
            inter_attr[(i * 4) + 1] = cpu_to_be32(0); // node id
            inter_attr[(i * 4) + 2] = cpu_to_be32(inter);
            inter_attr[(i * 4) + 3] = cpu_to_be32(1); // node id
        }
        qemu_fdt_setprop(fdt, node_name, "interconnects", inter_attr,
                         vbasedev->num_interconnects * sizeof(uint32_t) * 4);
        g_free(inter_attr);
    }

    error_report("%s 8", __func__);
    if (vbasedev->num_phys > 0) {
        uint32_t *phy = g_new(uint32_t, vbasedev->num_phys);

        error_report("%s creating %d phy nodes", __func__, vbasedev->num_phys);

        for (i = 0; i < vbasedev->num_phys;  i++) {
            phy[i] = qemu_fdt_alloc_phandle(fdt);
            fdt_build_virtio_phy_mmio_node(data, vdev, i, fdt, phy[i]);
            phy[i] = cpu_to_be32(phy[i]);
        }
        qemu_fdt_setprop(fdt, node_name, "phys", phy,
                         vbasedev->num_phys * sizeof(uint32_t));
        g_free(phy);
    }

    error_report("%s 9", __func__);
    if (vbasedev->num_pctrl_states) {
        error_report("%s creating %d pinctrl nodes",
                     __func__, vbasedev->num_pctrl_states);
        for (i = 0; i < vbasedev->num_pctrl_states;  i++) {
            char *pinctrl_name = g_strdup_printf("pinctrl-%u", i);
            uint32_t pctrl = qemu_fdt_alloc_phandle(fdt);

            fdt_build_virtio_pinctrl_mmio_node(data, vdev, i, fdt, pctrl);
            qemu_fdt_setprop_cell(fdt, node_name, pinctrl_name, pctrl);
        }
    }

    g_free(node_name);
    return 0;
}

static const BindingEntry mdss_childen_bindings[] = {
#ifdef CONFIG_LINUX
    VFIO_PLATFORM_BINDING("qcom,sc7180-dpu",
            add_qcom_trogdor_mdss_child_node, qcom_trogdor_mdp_prop),
    VFIO_PLATFORM_BINDING("qcom,mdss-dsi-ctrl",
            add_qcom_trogdor_mdss_child_node, qcom_trogdor_dsi_prop),
    VFIO_PLATFORM_BINDING("qcom,dsi-phy-10nm",
            add_qcom_trogdor_mdss_child_node, qcom_trogdor_dsi_phy_prop),
    VFIO_PLATFORM_BINDING("qcom,sc7180-dp",
            add_qcom_trogdor_mdss_child_node, qcom_trogdor_dp_prop),
#endif
};

static int add_qcom_trogdor_fdt_mdss_node(SysBusDevice *sbdev, void *opaque,
                                   ProppertList *properties)
{
    PlatformBusFDTData *data = opaque;
    const char *parent_node = data->pbus_node_name;
    PlatformBusDevice *pbus = data->pbus;
    void *fdt = data->fdt;
    VFIOPlatformDevice *vdev = VFIO_PLATFORM_DEVICE(sbdev);
    VFIODevice *vbasedev = &vdev->vbasedev;
    char *node_name;
    uint64_t mmio_base;
    uint32_t pwr_attr[2];
    int i;
    PlatformBusFDTData child_data = {
        .fdt = fdt,
        .irq_start = data->irq_start,
        .pbus = pbus,
        .parent = vdev,
        .bindings = mdss_childen_bindings,
        .binding_size = ARRAY_SIZE(mdss_childen_bindings),
    };

    error_report("%s 0", __func__);

    if (vbasedev->num_regions > 0) {
        mmio_base = platform_bus_get_mmio_addr(pbus, sbdev, 0);
        node_name = g_strdup_printf("%s/%s@%" PRIx64, parent_node,
                                    vbasedev->name, mmio_base);
    } else
        node_name = g_strdup_printf("%s/%s", parent_node, vbasedev->name);

    error_report("%s 1", __func__);

    qemu_fdt_add_subnode(fdt, node_name);
    copy_host_node_prop(vdev, fdt, node_name, properties);

    qemu_fdt_setprop_cells(fdt, node_name, "#size-cells", 1);
    qemu_fdt_setprop_cells(fdt, node_name, "#address-cells", 1);

    if (mdss_irq_parent == 0)
        mdss_irq_parent = qemu_fdt_alloc_phandle(fdt);

    qemu_fdt_setprop_cell(fdt, node_name, "phandle", mdss_irq_parent);

    if (disp_clk_handle == 0)
        disp_clk_handle = qemu_fdt_alloc_phandle(fdt);

    pwr_attr[0] = cpu_to_be32(disp_clk_handle);
    pwr_attr[1] = cpu_to_be32(0x00);
    qemu_fdt_setprop(fdt, node_name, "power-domains", pwr_attr,
                     2 * sizeof(uint32_t));

    error_report("%s 2", __func__);
    /* Copy reg (remapped) */
    if (vbasedev->num_regions > 0) {
        uint32_t *reg_attr = g_new(uint32_t, vbasedev->num_regions * 2);
        for (i = 0; i < vbasedev->num_regions; i++) {
            mmio_base = platform_bus_get_mmio_addr(pbus, sbdev, i);
            reg_attr[2 * i] = cpu_to_be32(mmio_base);
            reg_attr[2 * i + 1] = cpu_to_be32(
                                    memory_region_size(vdev->regions[i]->mem));
        }
        qemu_fdt_setprop(fdt, node_name, "reg", reg_attr,
                         vbasedev->num_regions * 2 * sizeof(uint32_t));
        g_free(reg_attr);
    }

    error_report("%s 3", __func__);

    /* Copy interrupts (remapped) */
    if (vbasedev->num_irqs) {
        uint32_t *irq_attr = g_new(uint32_t, vbasedev->num_irqs * 3);
        for (i = 0; i < vbasedev->num_irqs; i++) {
            VFIOINTp *intp;
            int irq_number = platform_bus_get_irqn(pbus, sbdev, i) +
                                                             data->irq_start;

            irq_attr[3 * i] = cpu_to_be32(GIC_FDT_IRQ_TYPE_SPI);
            irq_attr[3 * i + 1] = cpu_to_be32(irq_number);
            QLIST_FOREACH(intp, &vdev->intp_list, next) {
                if (intp->pin == i) {
                    break;
                }
            }
            if (intp->flags & VFIO_IRQ_INFO_AUTOMASKED) {
                irq_attr[3 * i + 2] = cpu_to_be32(GIC_FDT_IRQ_FLAGS_LEVEL_HI);
            } else {
                irq_attr[3 * i + 2] = cpu_to_be32(GIC_FDT_IRQ_FLAGS_EDGE_LO_HI);
            }
        }
        qemu_fdt_setprop(fdt, node_name, "interrupts",
                         irq_attr, vbasedev->num_irqs * 3 * sizeof(uint32_t));
        g_free(irq_attr);
    }

    error_report("%s 4", __func__);

    if (vbasedev->num_clks > 0) {
        uint32_t *clk = g_new(uint32_t, vbasedev->num_clks);
        uint32_t clk_attr[5];

        error_report("%s creating %d clk nodes",
                     __func__, vbasedev->num_clks);

        for (i = 0; i < vbasedev->num_clks;  i++) {
            clk[i] = qemu_fdt_alloc_phandle(fdt);
            fdt_build_virtio_clk_mmio_node(data, vdev, i, fdt, clk[i]);
            clk[i] = cpu_to_be32(clk[i]);
        }

        /* Local clock construction */
        if (disp_clk_handle == 0)
            disp_clk_handle = qemu_fdt_alloc_phandle(fdt);

        clk_attr[0] = clk[0];
        clk_attr[1] = cpu_to_be32(disp_clk_handle);
        clk_attr[2] = cpu_to_be32(0x02);
        clk_attr[3] = cpu_to_be32(disp_clk_handle);
        clk_attr[4] = cpu_to_be32(0x14);
        qemu_fdt_setprop(fdt, node_name, "clocks", clk_attr,
                         5 * sizeof(uint32_t));

        clk_attr[0] = cpu_to_be32(disp_clk_handle);
        clk_attr[1] = cpu_to_be32(0x14);
        qemu_fdt_setprop(fdt, node_name, "assigned-clocks", clk_attr,
                         2 * sizeof(uint32_t));
        g_free(clk);
    }

    error_report("%s 5", __func__);
    if (vbasedev->num_regulators) {

        error_report("%s creating %d regulator nodes",
                     __func__, vbasedev->num_regulators);

        for (i = 0; i < vbasedev->num_regulators;  i++) {
            char *supply_name;
            uint32_t reg;

            reg = qemu_fdt_alloc_phandle(fdt);

            fdt_build_virtio_regulator_mmio_node(data, vdev, i, fdt, reg,
                                                 vbasedev->regulator_names[i],
                                                 qcom_trogdor_regulator_properties);

            supply_name = g_strdup_printf("%s-supply",
                                          vbasedev->regulator_names[i]);

            error_report("%s creating regulator name %s", __func__, supply_name);
            qemu_fdt_setprop_cell(fdt, node_name, supply_name, reg);
        }
    }

    error_report("%s 6", __func__);
    if (vbasedev->num_gpio_func > 0) {
        error_report("%s creating %d gpio nodes",
                     __func__, vbasedev->num_gpio_func);
        for (i = 0; i < vbasedev->num_gpio_func;  i++) {
            uint32_t *gpio_attr;
            unsigned int pin;
            char *func_name;
            uint32_t gpio;

            gpio = qemu_fdt_alloc_phandle(fdt);
            fdt_build_virtio_gpio_mmio_node(data, vdev, i, fdt, gpio);
            func_name = g_strdup_printf("%s-gpios",
                                          vbasedev->gpio[i].gpio_func_names);

            error_report("%s creating GPIO func name %s", __func__, func_name);
            gpio_attr = g_new(uint32_t, vbasedev->gpio[i].pin_num * 3);
            for (pin = 0; pin < vbasedev->gpio[i].pin_num; pin++) {
                uint64_t flags = fdt_xlate_flags(vbasedev->gpio[i].flags[pin]);

                error_report("%s creating GPIO func name %s pin %d flags 0x%lx xflags 0x%lx",
                        __func__, func_name, pin, (long)vbasedev->gpio[i].flags[pin],
                        (long)flags);

                gpio_attr[(pin * 3) + 0] = cpu_to_be32(gpio);
                gpio_attr[(pin * 3) + 1] = cpu_to_be32(pin);
                gpio_attr[(pin * 3) + 2] = cpu_to_be32(flags);
            }
            qemu_fdt_setprop(fdt, node_name, func_name, gpio_attr,
                             vbasedev->gpio[i].pin_num * 3 * sizeof(uint32_t));
            g_free(gpio_attr);
        }
    }
    error_report("%s 7", __func__);

    if (vbasedev->num_interconnects) {
        uint32_t *inter_attr = g_new(uint32_t, vbasedev->num_interconnects * 4);
        error_report("%s creating %d interconnect nodes",
                     __func__, vbasedev->num_interconnects);

        for (i = 0; i < vbasedev->num_interconnects;  i++) {
            uint32_t inter;

            inter = qemu_fdt_alloc_phandle(fdt);
            fdt_build_virtio_inter_mmio_node(data, vdev, i, fdt, inter);
            /* src node points to dst node within the same provider */
            inter_attr[(i * 4) + 0] = cpu_to_be32(inter);
            inter_attr[(i * 4) + 1] = cpu_to_be32(0); // node id
            inter_attr[(i * 4) + 2] = cpu_to_be32(inter);
            inter_attr[(i * 4) + 3] = cpu_to_be32(1); // node id
        }
        qemu_fdt_setprop(fdt, node_name, "interconnects", inter_attr,
                         vbasedev->num_interconnects * sizeof(uint32_t) * 4);
        g_free(inter_attr);
    }

    error_report("%s 8", __func__);
    if (vbasedev->num_phys > 0) {
        uint32_t *phy = g_new(uint32_t, vbasedev->num_phys);

        error_report("%s creating %d phy nodes", __func__, vbasedev->num_phys);

        for (i = 0; i < vbasedev->num_phys;  i++) {
            phy[i] = qemu_fdt_alloc_phandle(fdt);
            fdt_build_virtio_phy_mmio_node(data, vdev, i, fdt, phy[i]);
            phy[i] = cpu_to_be32(phy[i]);
        }
        qemu_fdt_setprop(fdt, node_name, "phys", phy,
                         vbasedev->num_phys * sizeof(uint32_t));
        g_free(phy);
    }

    error_report("%s 9", __func__);
    if (vbasedev->num_pctrl_states) {
        error_report("%s creating %d pinctrl nodes",
                     __func__, vbasedev->num_pctrl_states);
        for (i = 0; i < vbasedev->num_pctrl_states;  i++) {
            char *pinctrl_name = g_strdup_printf("pinctrl-%u", i);
            uint32_t pctrl = qemu_fdt_alloc_phandle(fdt);

            fdt_build_virtio_pinctrl_mmio_node(data, vdev, i, fdt, pctrl);
            qemu_fdt_setprop_cell(fdt, node_name, pinctrl_name, pctrl);
        }
    }

    error_report("\nCreating child node - start");

    child_data.pbus_node_name = node_name;
    foreach_dynamic_sysbus_device(add_fdt_node, &child_data);

    error_report("Creating child node - end\n");

    g_free(node_name);
    return 0;
}

static int add_qcom_trogdor_wrapper_geniqup_node(SysBusDevice *sbdev, void *opaque,
                                   ProppertList *properties)
{
    PlatformBusFDTData *data = opaque;
    const char *parent_node = data->pbus_node_name;
    PlatformBusDevice *pbus = data->pbus;
    void *fdt_guest = data->fdt;
    VFIOPlatformDevice *vdev = VFIO_PLATFORM_DEVICE(sbdev);
    VFIODevice *vbasedev = &vdev->vbasedev;
    uint32_t *reg_attr, *irq_attr, *guest_clk_phandle;
    uint32_t phandle;
    char *node_name;
    uint64_t mmio_base;
    int i, irq_number;
    VFIOINTp *intp;
    PlatformBusFDTData child_data = {
        .fdt = fdt_guest,
        .irq_start = data->irq_start,
        .pbus = pbus,
        .parent = vdev,
        .bindings = child_bindings,
        .binding_size = ARRAY_SIZE(child_bindings),
    };

    mmio_base = platform_bus_get_mmio_addr(pbus, sbdev, 0);
    node_name = g_strdup_printf("%s/%s@%" PRIx64, parent_node,
                                vbasedev->name, mmio_base);

    qemu_fdt_add_subnode(fdt_guest, node_name);
    copy_host_node_prop(vdev, fdt_guest, node_name, properties);

    qemu_fdt_setprop_cells(fdt_guest, node_name, "#size-cells", 1);
    qemu_fdt_setprop_cells(fdt_guest, node_name, "#address-cells", 1);

    /* Copy reg (remapped) */
    reg_attr = g_new(uint32_t, vbasedev->num_regions * 2);
    for (i = 0; i < vbasedev->num_regions; i++) {
        mmio_base = platform_bus_get_mmio_addr(pbus, sbdev, i);
        reg_attr[2 * i] = cpu_to_be32(mmio_base);
        reg_attr[2 * i + 1] = cpu_to_be32(
                                memory_region_size(vdev->regions[i]->mem));
    }
    qemu_fdt_setprop(fdt_guest, node_name, "reg", reg_attr,
                     vbasedev->num_regions * 2 * sizeof(uint32_t));

    /* Copy interrupts (remapped) */
    if (vbasedev->num_irqs) {
        irq_attr = g_new(uint32_t, vbasedev->num_irqs * 3);
        for (i = 0; i < vbasedev->num_irqs; i++) {
            irq_number = platform_bus_get_irqn(pbus, sbdev, i) +
                         data->irq_start;
            irq_attr[3 * i] = cpu_to_be32(GIC_FDT_IRQ_TYPE_SPI);
            irq_attr[3 * i + 1] = cpu_to_be32(irq_number);
            QLIST_FOREACH(intp, &vdev->intp_list, next) {
                if (intp->pin == i) {
                    break;
                }
            }
            if (intp->flags & VFIO_IRQ_INFO_AUTOMASKED) {
                irq_attr[3 * i + 2] = cpu_to_be32(GIC_FDT_IRQ_FLAGS_LEVEL_HI);
            } else {
                irq_attr[3 * i + 2] = cpu_to_be32(GIC_FDT_IRQ_FLAGS_EDGE_LO_HI);
            }
        }
        qemu_fdt_setprop(fdt_guest, node_name, "interrupts",
                         irq_attr, vbasedev->num_irqs * 3 * sizeof(uint32_t));
        g_free(irq_attr);
    }

    guest_clk_phandle = g_new(uint32_t, vbasedev->num_clks);
    for (i = 0; i < vbasedev->num_clks;  i++) {
        guest_clk_phandle[i] = qemu_fdt_alloc_phandle(fdt_guest);
        fdt_build_virtio_clk_mmio_node(data, vdev, i, fdt_guest,
                                       guest_clk_phandle[i]);
        guest_clk_phandle[i] = cpu_to_be32(guest_clk_phandle[i]);
    }
    if (vbasedev->num_clks > 0) {
        qemu_fdt_setprop(fdt_guest, node_name, "clocks", guest_clk_phandle,
                         vbasedev->num_clks * sizeof(uint32_t));
    }
    g_free(guest_clk_phandle);

    guest_clk_phandle = g_new(uint32_t, vbasedev->num_regulators);
    for (i = 0; i < vbasedev->num_regulators;  i++) {
        char *supply_name;

        guest_clk_phandle[i] = qemu_fdt_alloc_phandle(fdt_guest);
        fdt_build_virtio_regulator_mmio_node(data, vdev, i, fdt_guest,
                                             guest_clk_phandle[i],
                                             vbasedev->regulator_names[i],
                                             qcom_trogdor_regulator_properties);
        supply_name = g_strdup_printf("%s-supply",
                                      vbasedev->regulator_names[i]);

        error_report("%s copying properties of regulator name %s",
                     __func__, supply_name);

        qemu_fdt_setprop_cell(fdt_guest, node_name, supply_name,
                              guest_clk_phandle[i]);
    }

    g_free(guest_clk_phandle);

    guest_clk_phandle = g_new(uint32_t, vbasedev->num_interconnects * 4);

    error_report("%s creating %d interconnect nodes",
                 __func__, vbasedev->num_interconnects);

    for (i = 0; i < vbasedev->num_interconnects;  i++) {
        phandle = qemu_fdt_alloc_phandle(fdt_guest);
        fdt_build_virtio_inter_mmio_node(data, vdev, i, fdt_guest, phandle);
        /* src node points to dst node within the same provider */
        guest_clk_phandle[(i * 4) + 0] = cpu_to_be32(phandle);
        guest_clk_phandle[(i * 4) + 1] = cpu_to_be32(0); // node id
        guest_clk_phandle[(i * 4) + 2] = cpu_to_be32(phandle);
        guest_clk_phandle[(i * 4) + 3] = cpu_to_be32(1); // node id
    }
    if (vbasedev->num_interconnects > 0) {
        qemu_fdt_setprop(fdt_guest, node_name, "interconnects", guest_clk_phandle,
                         vbasedev->num_interconnects * sizeof(uint32_t) * 4);
    }

    g_free(guest_clk_phandle);

    guest_clk_phandle = g_new(uint32_t, vbasedev->num_phys);
    for (i = 0; i < vbasedev->num_phys;  i++) {
        guest_clk_phandle[i] = qemu_fdt_alloc_phandle(fdt_guest);
        fdt_build_virtio_phy_mmio_node(data, vdev, i, fdt_guest,
                                       guest_clk_phandle[i]);
        guest_clk_phandle[i] = cpu_to_be32(guest_clk_phandle[i]);
    }
    if (vbasedev->num_phys > 0) {
        qemu_fdt_setprop(fdt_guest, node_name, "phys", guest_clk_phandle,
                         vbasedev->num_phys * sizeof(uint32_t));
    }
    g_free(guest_clk_phandle);

    guest_clk_phandle = g_new(uint32_t, vbasedev->num_pctrl_states);
    for (i = 0; i < vbasedev->num_pctrl_states;  i++) {
        char *pinctrl_name = g_strdup_printf("pinctrl-%u", i);

        guest_clk_phandle[i] = qemu_fdt_alloc_phandle(fdt_guest);
        fdt_build_virtio_pinctrl_mmio_node(data, vdev, i, fdt_guest,
                                           guest_clk_phandle[i]);

        error_report("%s copying properties of regulator name %s",
                     __func__, pinctrl_name);

        qemu_fdt_setprop_cell(fdt_guest, node_name, pinctrl_name,
                              guest_clk_phandle[i]);
    }

    g_free(guest_clk_phandle);
    g_free(reg_attr);

    error_report("\nCreating child node - start");

    child_data.pbus_node_name = node_name;
    foreach_dynamic_sysbus_device(add_fdt_node, &child_data);

    error_report("Creating child node - end\n");

    g_free(node_name);
    return 0;
}

static int add_qcom_trogdor_wrapper_usb_node(SysBusDevice *sbdev, void *opaque,
                                   ProppertList *properties)
{
    PlatformBusFDTData *data = opaque;
    const char *parent_node = data->pbus_node_name;
    PlatformBusDevice *pbus = data->pbus;
    void *fdt_guest = data->fdt;
    VFIOPlatformDevice *vdev = VFIO_PLATFORM_DEVICE(sbdev);
    VFIODevice *vbasedev = &vdev->vbasedev;
    uint32_t *reg_attr, *irq_attr, *guest_clk_phandle;
    uint32_t guest_clk_assigned_phandle[2];
    uint32_t phandle;
    char *node_name;
    uint64_t mmio_base;
    int i, irq_number;
    VFIOINTp *intp;
    PlatformBusFDTData child_data = {
        .fdt = fdt_guest,
        .irq_start = data->irq_start,
        .pbus = pbus,
        .parent = vdev,
        .bindings = child_bindings,
        .binding_size = ARRAY_SIZE(child_bindings),
    };

    mmio_base = platform_bus_get_mmio_addr(pbus, sbdev, 0);
    node_name = g_strdup_printf("%s/%s@%" PRIx64, parent_node,
                                vbasedev->name, mmio_base);

    qemu_fdt_add_subnode(fdt_guest, node_name);
    copy_host_node_prop(vdev, fdt_guest, node_name, properties);

    qemu_fdt_setprop_cells(fdt_guest, node_name, "#size-cells", 1);
    qemu_fdt_setprop_cells(fdt_guest, node_name, "#address-cells", 1);

    /* Copy reg (remapped) */
    reg_attr = g_new(uint32_t, vbasedev->num_regions * 2);
    for (i = 0; i < vbasedev->num_regions; i++) {
        mmio_base = platform_bus_get_mmio_addr(pbus, sbdev, i);
        reg_attr[2 * i] = cpu_to_be32(mmio_base);
        reg_attr[2 * i + 1] = cpu_to_be32(
                                memory_region_size(vdev->regions[i]->mem));
    }
    qemu_fdt_setprop(fdt_guest, node_name, "reg", reg_attr,
                     vbasedev->num_regions * 2 * sizeof(uint32_t));

    /* Copy interrupts (remapped) */
    if (vbasedev->num_irqs) {
        irq_attr = g_new(uint32_t, vbasedev->num_irqs * 3);
        for (i = 0; i < vbasedev->num_irqs; i++) {
            irq_number = platform_bus_get_irqn(pbus, sbdev, i) +
                         data->irq_start;
            irq_attr[3 * i] = cpu_to_be32(GIC_FDT_IRQ_TYPE_SPI);
            irq_attr[3 * i + 1] = cpu_to_be32(irq_number);
            QLIST_FOREACH(intp, &vdev->intp_list, next) {
                if (intp->pin == i) {
                    break;
                }
            }
            if (intp->flags & VFIO_IRQ_INFO_AUTOMASKED) {
                irq_attr[3 * i + 2] = cpu_to_be32(GIC_FDT_IRQ_FLAGS_LEVEL_HI);
            } else {
                irq_attr[3 * i + 2] = cpu_to_be32(GIC_FDT_IRQ_FLAGS_EDGE_LO_HI);
            }
        }
        qemu_fdt_setprop(fdt_guest, node_name, "interrupts",
                         irq_attr, vbasedev->num_irqs * 3 * sizeof(uint32_t));
        g_free(irq_attr);
    }

    guest_clk_phandle = g_new(uint32_t, vbasedev->num_clks);
    for (i = 0; i < vbasedev->num_clks;  i++) {
        guest_clk_phandle[i] = qemu_fdt_alloc_phandle(fdt_guest);
        fdt_build_virtio_clk_mmio_node(data, vdev, i, fdt_guest,
                                       guest_clk_phandle[i]);
        guest_clk_phandle[i] = cpu_to_be32(guest_clk_phandle[i]);
    }
    if (vbasedev->num_clks > 0) {
        qemu_fdt_setprop(fdt_guest, node_name, "clocks", guest_clk_phandle,
                         vbasedev->num_clks * sizeof(uint32_t));
        guest_clk_assigned_phandle[0] = guest_clk_phandle[3];
        guest_clk_assigned_phandle[1] = guest_clk_phandle[1];
        qemu_fdt_setprop(fdt_guest, node_name, "assigned-clocks",
                         guest_clk_assigned_phandle, 2 * sizeof(uint32_t));
    }
    g_free(guest_clk_phandle);

    guest_clk_phandle = g_new(uint32_t, vbasedev->num_regulators);
    for (i = 0; i < vbasedev->num_regulators;  i++) {
        char *supply_name;

        guest_clk_phandle[i] = qemu_fdt_alloc_phandle(fdt_guest);
        fdt_build_virtio_regulator_mmio_node(data, vdev, i, fdt_guest,
                                             guest_clk_phandle[i],
                                             vbasedev->regulator_names[i],
                                             qcom_trogdor_regulator_properties);
        supply_name = g_strdup_printf("%s-supply",
                                      vbasedev->regulator_names[i]);

        error_report("%s copying properties of regulator name %s",
                     __func__, supply_name);

        qemu_fdt_setprop_cell(fdt_guest, node_name, supply_name,
                              guest_clk_phandle[i]);
    }

    g_free(guest_clk_phandle);

    guest_clk_phandle = g_new(uint32_t, vbasedev->num_interconnects * 4);

    error_report("%s creating %d interconnect nodes",
                 __func__, vbasedev->num_interconnects);

    for (i = 0; i < vbasedev->num_interconnects;  i++) {
        phandle = qemu_fdt_alloc_phandle(fdt_guest);
        fdt_build_virtio_inter_mmio_node(data, vdev, i, fdt_guest, phandle);
        /* src node points to dst node within the same provider */
        guest_clk_phandle[(i * 4) + 0] = cpu_to_be32(phandle);
        guest_clk_phandle[(i * 4) + 1] = cpu_to_be32(0); // node id
        guest_clk_phandle[(i * 4) + 2] = cpu_to_be32(phandle);
        guest_clk_phandle[(i * 4) + 3] = cpu_to_be32(1); // node id
    }
    if (vbasedev->num_interconnects > 0) {
        qemu_fdt_setprop(fdt_guest, node_name, "interconnects", guest_clk_phandle,
                         vbasedev->num_interconnects * sizeof(uint32_t) * 4);
    }

    g_free(guest_clk_phandle);

    guest_clk_phandle = g_new(uint32_t, vbasedev->num_phys);
    for (i = 0; i < vbasedev->num_phys;  i++) {
        guest_clk_phandle[i] = qemu_fdt_alloc_phandle(fdt_guest);
        fdt_build_virtio_phy_mmio_node(data, vdev, i, fdt_guest,
                                       guest_clk_phandle[i]);
        guest_clk_phandle[i] = cpu_to_be32(guest_clk_phandle[i]);
    }
    if (vbasedev->num_phys > 0) {
        qemu_fdt_setprop(fdt_guest, node_name, "phys", guest_clk_phandle,
                         vbasedev->num_phys * sizeof(uint32_t));
    }
    g_free(guest_clk_phandle);

    guest_clk_phandle = g_new(uint32_t, vbasedev->num_pctrl_states);
    for (i = 0; i < vbasedev->num_pctrl_states;  i++) {
        char *pinctrl_name = g_strdup_printf("pinctrl-%u", i);

        guest_clk_phandle[i] = qemu_fdt_alloc_phandle(fdt_guest);
        fdt_build_virtio_pinctrl_mmio_node(data, vdev, i, fdt_guest,
                                           guest_clk_phandle[i]);

        error_report("%s copying properties of regulator name %s",
                     __func__, pinctrl_name);

        qemu_fdt_setprop_cell(fdt_guest, node_name, pinctrl_name,
                              guest_clk_phandle[i]);
    }

    g_free(guest_clk_phandle);
    g_free(reg_attr);

    error_report("\nCreating child node - start");

    child_data.pbus_node_name = node_name;
    foreach_dynamic_sysbus_device(add_fdt_node, &child_data);

    error_report("Creating child node - end\n");

    g_free(node_name);
    return 0;
}

#endif /* CONFIG_LINUX */

/*
 * add_tpm_tis_fdt_node: Create a DT node for TPM TIS
 *
 * See kernel documentation:
 * Documentation/devicetree/bindings/security/tpm/tpm_tis_mmio.txt
 * Optional interrupt for command completion is not exposed
 */
static int add_tpm_tis_fdt_node(SysBusDevice *sbdev, void *opaque,
                                ProppertList *properties)
{
    PlatformBusFDTData *data = opaque;
    PlatformBusDevice *pbus = data->pbus;
    void *fdt = data->fdt;
    const char *parent_node = data->pbus_node_name;
    char *nodename;
    uint32_t reg_attr[2];
    uint64_t mmio_base;

    mmio_base = platform_bus_get_mmio_addr(pbus, sbdev, 0);
    nodename = g_strdup_printf("%s/tpm_tis@%" PRIx64, parent_node, mmio_base);
    qemu_fdt_add_subnode(fdt, nodename);

    qemu_fdt_setprop_string(fdt, nodename, "compatible", "tcg,tpm-tis-mmio");

    reg_attr[0] = cpu_to_be32(mmio_base);
    reg_attr[1] = cpu_to_be32(0x5000);
    qemu_fdt_setprop(fdt, nodename, "reg", reg_attr, 2 * sizeof(uint32_t));

    g_free(nodename);
    return 0;
}

/* list of supported dynamic sysbus bindings */
static const BindingEntry bindings[] = {
#ifdef CONFIG_LINUX
    TYPE_BINDING(TYPE_VFIO_CALXEDA_XGMAC, add_calxeda_midway_xgmac_fdt_node),
    TYPE_BINDING(TYPE_VFIO_AMD_XGBE, add_amd_xgbe_fdt_node),
    VFIO_PLATFORM_BINDING("amd,xgbe-seattle-v1a", add_amd_xgbe_fdt_node, NULL),
    VFIO_PLATFORM_BINDING("marvell,armada-8k-ahci",
            add_mvrl_armada_fdt_node, mvrl_armada_ahci_properties),
    VFIO_PLATFORM_BINDING("generic-xhci",
            add_mvrl_armada_fdt_node, mvrl_armada_xhci_properties),
    VFIO_PLATFORM_BINDING("marvell,armada-cp110-sdhci",
            add_mvrl_armada_fdt_node, mvrl_armada_sdhci_properties),
    VFIO_PLATFORM_BINDING("qcom,wcn3990-wifi",
            add_qcom_trogdor_fdt_node, qcom_trogdor_wifi_properties),
    VFIO_PLATFORM_BINDING("qcom,sdhci-msm-v5",
            add_qcom_trogdor_fdt_node, qcom_trogdor_sdhci_properties),
    VFIO_PLATFORM_BINDING("qcom,dwc3",
            add_qcom_trogdor_wrapper_usb_node, qcom_trogdor_parent_xhci_properties),
    VFIO_PLATFORM_BINDING("maxim,max98357a",
            add_qcom_trogdor_fdt_node, qcom_trogdor_audio_codec_properties),
    VFIO_PLATFORM_BINDING("pwm-backlight",
            add_qcom_trogdor_fdt_node, qcom_trogdor_backlight_properties),
    VFIO_PLATFORM_BINDING("boe,nv133fhm-n62",
            add_qcom_trogdor_fdt_node, qcom_trogdor_panel_properties),
    VFIO_PLATFORM_BINDING("snps,dwc3", no_fdt_node, NULL),
//    VFIO_PLATFORM_BINDING("snps,dwc3",
//            add_qcom_trogdor_dwc3_child_node, qcom_trogdor_xhci_properties),
    VFIO_PLATFORM_BINDING("qcom,adreno",
            add_qcom_trogdor_fdt_node, qcom_trogdor_gpu_properties),
    VFIO_PLATFORM_BINDING("qcom,adreno-gmu",
            add_qcom_trogdor_fdt_node, qcom_trogdor_gmu_properties),

    VFIO_PLATFORM_BINDING("qcom,sc7180-mdss",
            add_qcom_trogdor_fdt_mdss_node, qcom_trogdor_mdss_properties),
    VFIO_PLATFORM_BINDING("qcom,sc7180-dispcc",
            add_qcom_trogdor_fdt_node, qcom_trogdor_disp_clk_prop),

    /* GENIQUP tree */
    VFIO_PLATFORM_BINDING("qcom,geni-se-qup",
            add_qcom_trogdor_wrapper_geniqup_node, qcom_trogdor_geni_properties),
    VFIO_PLATFORM_BINDING("qcom,geni-uart", no_fdt_node, NULL),
    VFIO_PLATFORM_BINDING("qcom,wcn3991-bt", no_fdt_node, NULL),
    VFIO_PLATFORM_BINDING("qcom,geni-i2c", no_fdt_node, NULL),
    VFIO_PLATFORM_BINDING("hid-over-i2c", no_fdt_node, NULL),
    VFIO_PLATFORM_BINDING("ti,sn65dsi86", no_fdt_node, NULL),
    VFIO_PLATFORM_BINDING("qcom,geni-spi", no_fdt_node, NULL),
    VFIO_PLATFORM_BINDING("google,cros-ec-spi", no_fdt_node, NULL),
#endif
    TYPE_BINDING(TYPE_TPM_TIS_SYSBUS, add_tpm_tis_fdt_node),
    TYPE_BINDING(TYPE_RAMFB_DEVICE, no_fdt_node),
    TYPE_BINDING("", NULL), /* last element */
};

/* Generic Code */

/**
 * add_fdt_node - add the device tree node of a dynamic sysbus device
 *
 * @sbdev: handle to the sysbus device
 * @opaque: handle to the PlatformBusFDTData
 *
 * Checks the sysbus type belongs to the list of device types that
 * are dynamically instantiable and if so call the node creation
 * function.
 */
static void add_fdt_node(SysBusDevice *sbdev, void *opaque)
{
    VFIOPlatformDevice *vdev = VFIO_PLATFORM_DEVICE(sbdev);
    PlatformBusFDTData *data = opaque;
    int i, ret;

    for (i = 0; i < data->binding_size; i++) {
        const BindingEntry *iter = &data->bindings[i];
        VFIOPlatformDevice *dev_iter;

        error_report("Device %s dev parent %p bus parent %p",
                      qdev_fw_name(DEVICE(sbdev)), vdev->parent, data->parent);

        QLIST_FOREACH(dev_iter, &dev_list, list) {
            if (dev_iter == vdev) {
                error_report("Skipped %s, already probed dev %p",
                             qdev_fw_name(DEVICE(sbdev)), vdev);
                return;
            }

        }

        if (type_match(sbdev, iter)) {
            if ((!iter->match_fn || iter->match_fn(sbdev, iter)) &&
                vdev->parent == data->parent) {
                QLIST_INSERT_HEAD(&dev_list, vdev, list);
                ret = iter->add_fn(sbdev, opaque, iter->properties);
                assert(!ret);
                return;
            }
        }
    }
}

static void fdt_node_check(SysBusDevice *sbdev, void *opaque)
{
    VFIOPlatformDevice *vdev = VFIO_PLATFORM_DEVICE(sbdev);
    VFIOPlatformDevice *dev_iter;

    QLIST_FOREACH(dev_iter, &dev_list, list) {
        if (dev_iter == vdev) {
            return;
        }
    }
    error_report("Device %s can not be dynamically instantiated",
                     qdev_fw_name(DEVICE(sbdev)));
    exit(1);
}

void platform_bus_add_all_fdt_nodes(void *fdt, const char *intc, hwaddr addr,
                                    hwaddr bus_size, int irq_start)
{
    const char platcomp[] = "qemu,platform\0simple-bus";
    PlatformBusDevice *pbus;
    DeviceState *dev;
    gchar *node;

    assert(fdt);

    node = g_strdup_printf("/platform@%"PRIx64, addr);

    /* Create a /platform node that we can put all devices into */
    qemu_fdt_add_subnode(fdt, node);
    qemu_fdt_setprop(fdt, node, "compatible", platcomp, sizeof(platcomp));

    /* Our platform bus region is less than 32bits, so 1 cell is enough for
     * address and size
     */
    qemu_fdt_setprop_cells(fdt, node, "#size-cells", 1);
    qemu_fdt_setprop_cells(fdt, node, "#address-cells", 1);
    qemu_fdt_setprop_cells(fdt, node, "ranges", 0, addr >> 32, addr, bus_size);

    qemu_fdt_setprop_phandle(fdt, node, "interrupt-parent", intc);

    dev = qdev_find_recursive(sysbus_get_default(), TYPE_PLATFORM_BUS_DEVICE);
    pbus = PLATFORM_BUS_DEVICE(dev);

    platform_bus_node = node;

    PlatformBusFDTData data = {
        .fdt = fdt,
        .irq_start = irq_start,
        .pbus_node_name = node,
        .pbus = pbus,
        .parent = NULL,
        .bindings = bindings,
        .binding_size = ARRAY_SIZE(bindings),
    };

    /* Loop through all dynamic sysbus devices and create their node */
    foreach_dynamic_sysbus_device(add_fdt_node, &data);

    /* Check that all devices created their DT nodes */
    foreach_dynamic_sysbus_device(fdt_node_check, &data);
    g_free(node);
}
