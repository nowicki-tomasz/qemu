/*
 * Copyright (c) 2021 Semihalf sp. z o.o.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, see <http://www.gnu.org/licenses/>.
 */

#ifndef HW_ARM_SMMUV2_H
#define HW_ARM_SMMUV2_H

#include "hw/arm/smmu-common.h"
#include "hw/register.h"
#include "hw/registerfields.h"

#define TYPE_SMMUV2_IOMMU_MEMORY_REGION "smmuv2-iommu-memory-region"

/* This should be configurable per instance.  */
#define PAGESIZE 4096

#define MAX_CB 128

#define R_MAX (2 * MAX_CB * PAGESIZE)

typedef struct SMMUv2State {
    SMMUState     smmu_state;

    QemuMutex mutex;

    struct {
        qemu_irq global;
        qemu_irq context[MAX_CB];
    } irq;

    struct {
        uint32_t pamax;
        uint16_t num_smr;
        uint16_t num_cb;
        uint16_t num_pages;
    } cfg;

    RegisterAccessInfo *rai_smr;
    RegisterAccessInfo *rai_cb;
    uint32_t regs[R_MAX];
    RegisterInfo regs_info[R_MAX];
    uint32_t cb_to_s2cr[R_MAX];
} SMMUv2State;

typedef struct {
    /*< private >*/
    SMMUBaseClass smmu_base_class;
    /*< public >*/

    DeviceRealize parent_realize;
    DeviceReset   parent_reset;
} SMMUv2Class;

#define TYPE_ARM_SMMUV2   "arm-smmuv2"
#define ARM_SMMUV2(obj) OBJECT_CHECK(SMMUv2State, (obj), TYPE_ARM_SMMUV2)
#define ARM_SMMUV2_CLASS(klass)                              \
    OBJECT_CLASS_CHECK(SMMUv2Class, (klass), TYPE_ARM_SMMUV2)
#define ARM_SMMUV2_GET_CLASS(obj) \
     OBJECT_GET_CLASS(SMMUv2Class, (obj), TYPE_ARM_SMMUV2)

#endif
