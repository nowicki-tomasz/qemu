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

#ifdef __linux__
#include "linux/iommu.h"
#endif

#include "qemu/osdep.h"
#include "hw/irq.h"
#include "hw/sysbus.h"
#include "migration/vmstate.h"
#include "hw/qdev-core.h"
#include "hw/qdev-properties.h"
#include "hw/pci/pci.h"
#include "hw/vfio/vfio-platform.h"
#include "exec/address-spaces.h"
#include "cpu.h"
#include "trace.h"
#include "qemu/log.h"
#include "qemu/error-report.h"
#include "qapi/error.h"

#include "hw/arm/smmuv2.h"
#include "smmuv2-internal.h"

#ifndef SMMUV2_ERR_DEBUG
#define SMMUV2_ERR_DEBUG 0
#endif

#define DEBUG_DEV_SMMU 0

#define D(...) do {             \
    if (DEBUG_DEV_SMMU) {       \
        qemu_log(__VA_ARGS__);  \
    }                           \
} while (0);

/* Compute the base offset (index into s->regs) for a given CB.  */
static unsigned int smmu_cb_offset(SMMUv2State *s, unsigned int cb)
{
    return ((s->cfg.num_pages + cb) * PAGESIZE) / 4;
}

static int smmu_stream_id_match(SMMUv2State *s, uint32_t sid, SMMUTransCfg *cfg,
                                SMMUv2Status *status)
{
    unsigned int nr_smr = ARRAY_FIELD_EX32(s->regs, SMMU_SIDR0, NUMSMRG);
    bool single_match = false;
    uint32_t s2cr, type;
    unsigned int i;

    for (i = 0; i < nr_smr; i++) {
        uint32_t v = s->regs[R_SMMU_SMR0 + i];
        bool valid = FIELD_EX32(v, SMMU_SMR0, VALID);
        uint16_t mask = FIELD_EX32(v, SMMU_SMR0, MASK);
        uint16_t id = FIELD_EX32(v, SMMU_SMR0, ID);

        /* FIXME: Check for multiple matches */
        if (valid && (~mask & id) == (~mask & sid)) {
            single_match = true;
            break;
        }
    }

    if (!single_match) {
        if (ARRAY_FIELD_EX32(s->regs, SMMU_SCR0, USFCFG)) {
            status->status = SMMU_TRANS_ERROR;
            status->type = SMMU_FAULT_GLOBAL_UDEF_SID;
            return -EINVAL;
        } else {
            status->status = SMMU_TRANS_BYPASS;
            cfg->bypassed = true;
            return 0;
        }
    }

    s2cr = s->regs[R_SMMU_S2CR0 + i];
    type = FIELD_EX32(s2cr, SMMU_S2CR0, TYPE);
    switch (type) {
    case S2CR_TYPE_TRANS:
        status->cb = FIELD_EX32(s2cr, SMMU_S2CR0, CBNDX_VMID);
        break;
    case S2CR_TYPE_BYPASS:
        status->status = SMMU_TRANS_BYPASS;
        cfg->bypassed = true;
        return 0;
    case S2CR_TYPE_FAULT:
        status->status = SMMU_TRANS_ERROR;
        cfg->aborted = true;
        return 0;
    default:
        g_assert_not_reached();
    }

//    error_report("SMMU StreamID 0x%x -> CB%d", sid, status->cb);
    return 0;
}

static int smmu_decode_cbar(SMMUv2State *s, SMMUTransCfg *cfg,
                            SMMUv2Status *status)
{
    unsigned int cb = status->cb;
    uint32_t cbar, type, cba2r;

    cbar = s->regs[R_SMMU_CBAR0 + cb];
    type = FIELD_EX32(cbar, SMMU_CBAR0, TYPE);
    switch (type) {
    case CBAR_TYPE_S2_EN:
        error_report("SMMUv2 does not support S2 yet");
        status->status = SMMU_TRANS_ERROR;
        status->type = SMMU_FAULT_GLOBAL_INVAL_CTX;
        return -EINVAL;
    case CBAR_TYPE_S1_EN_S2_FAULT:
        error_report("SMMUv2 does not support S1 enabled and S2 aborted yet");
        status->status = SMMU_TRANS_ERROR;
        status->type = SMMU_FAULT_GLOBAL_INVAL_CTX;
        return -EINVAL;
    case CBAR_TYPE_NESTED_EN:
        error_report("SMMUv2 does not support nested stages yet");
        status->status = SMMU_TRANS_ERROR;
        status->type = SMMU_FAULT_GLOBAL_INVAL_CTX;
        return -EINVAL;
    }

    /* We don't support 32bit page-tables yet */
    cba2r = s->regs[R_SMMU_CBA2R0 + cb];
    if (!FIELD_EX32(cba2r, SMMU_CBA2R0, VA64)) {
        status->status = SMMU_TRANS_ERROR;
        status->type = SMMU_FAULT_GLOBAL_UNIMPL_CTX_BANK;
        error_report("SMMU does not support 32bit page-tables yet");
        return -EINVAL;
    }

    return 0;
}

static int decode_ctx(SMMUv2State *s, SMMUTransCfg *cfg, SMMUv2Status *status)
{
    unsigned int cb_offset = smmu_cb_offset(s, status->cb);
    uint32_t sctlr, tcr, tcr2, a1;
    uint64_t ttbr;
    int i;

    sctlr = s->regs[R_SMMU_CB0_SCTLR + cb_offset];
    if (FIELD_EX32(sctlr, SMMU_CB0_SCTLR, M) == 0) {
        status->type = SMMU_FAULT_GLOBAL_UNIMPL_CTX_BANK;
        status->status = SMMU_TRANS_ERROR;
        error_report("SMMU disabled for context %d sctlr=%x",
                      status->cb, sctlr);
        return -EINVAL;
    }

    /* We don't support stall model yet */
    if (FIELD_EX32(sctlr, SMMU_CB0_SCTLR, CFCFG)) {
        status->type = SMMU_FAULT_GLOBAL_UNIMPL_CTX_BANK;
        status->status = SMMU_TRANS_ERROR;
        error_report("SMMU does not support fault stall model yet");
        return -EINVAL;
    }

    tcr2 = s->regs[R_SMMU_CB0_TCR2 + cb_offset];
    if (FIELD_EX32(tcr2, SMMU_CB0_TCR2, HD)) {
        status->type = SMMU_FAULT_GLOBAL_UNIMPL_CTX_BANK;
        status->status = SMMU_TRANS_ERROR;
        error_report("SMMU does not support HD yet");
        return -EINVAL;
    }

    if (FIELD_EX32(tcr2, SMMU_CB0_TCR2, HA)) {
        status->type = SMMU_FAULT_GLOBAL_UNIMPL_CTX_BANK;
        status->status = SMMU_TRANS_ERROR;
        error_report("SMMU does not support HA yet");
        return -EINVAL;
    }

    /* we support only those at the moment */
    cfg->aa64 = true;
    cfg->stage = 1;

    cfg->oas = oas2bits(FIELD_EX32(tcr2, SMMU_CB0_TCR2, PASIZE));
    cfg->oas = MIN(s->cfg.pamax, cfg->oas);
    cfg->tbi = FIELD_EX32(tcr2, SMMU_CB0_TCR2, TBI1) << 1 |
               FIELD_EX32(tcr2, SMMU_CB0_TCR2, TBI0);


    tcr = s->regs[R_SMMU_CB0_TCR_LPAE + cb_offset];
    a1 = FIELD_EX32(tcr, SMMU_CB0_TCR_LPAE, A1);
    if (a1) {
        ttbr = s->regs[R_SMMU_CB0_TTBR1_HIGH + cb_offset];
        cfg->asid =  FIELD_EX32(ttbr, SMMU_CB0_TTBR1_HIGH, ASID);
    } else {
        ttbr = s->regs[R_SMMU_CB0_TTBR0_HIGH + cb_offset];
        cfg->asid =  FIELD_EX32(ttbr, SMMU_CB0_TTBR0_HIGH, ASID);
    }

//    trace_smmuv3_decode_cd(cfg->oas);

#define TCR_TSZ(tcr, sel)   extract32(tcr, (16 * (sel)) + 0, 6)
#define TCR_EPD(tcr, sel)   extract32(tcr, (16 * (sel)) + 7, 1)
#define TCR_TG(tcr, sel)    extract32(tcr, (16 * (sel)) + 14, 2)
    /* decode data dependent on TT */
    for (i = 0; i <= 1; i++) {
        int tg, tsz;
        SMMUTransTableInfo *tt = &cfg->tt[i];

        cfg->tt[i].disabled = TCR_EPD(tcr, i);
        if (cfg->tt[i].disabled) {
            continue;
        }

        tsz = TCR_TSZ(tcr, i);
        tg = TCR_TG(tcr, i);
        tt->granule_sz = tg2granule(tg, i);
        if ((tt->granule_sz != 12 && tt->granule_sz != 16) ||
                FIELD_EX32(sctlr, SMMU_CB0_SCTLR, E)) {

            error_report("%s %d granule size", __func__, i);

            goto bad_cd;
        }

        tt->tsz = tsz;

        if (i == 1) {
            tt->ttb = s->regs[R_SMMU_CB0_TTBR1_HIGH + cb_offset];
            tt->ttb <<= 32;
            tt->ttb |= s->regs[R_SMMU_CB0_TTBR1_LOW + cb_offset];
        } else {
            tt->ttb = s->regs[R_SMMU_CB0_TTBR0_HIGH + cb_offset];
            tt->ttb <<= 32;
            tt->ttb |= s->regs[R_SMMU_CB0_TTBR0_LOW + cb_offset];
        }

        if (tt->ttb & ~(MAKE_64BIT_MASK(0, cfg->oas))) {
            error_report("%s %d invalid ttbr addr", __func__, i);
            goto bad_cd;
        }
//        trace_smmuv3_decode_cd_tt(i, tt->tsz, tt->ttb, tt->granule_sz);
    }

//    status->record_trans_faults = CD_R(cd);

    return 0;

bad_cd:
    status->type = SMMU_FAULT_GLOBAL_UNIMPL_CTX_BANK;
    status->status = SMMU_TRANS_ERROR;
    return -EINVAL;
}

/**
 * smmuv2_decode_config - Prepare the translation configuration
 * for the @mr iommu region
 * @mr: iommu memory region the translation config must be prepared for
 * @cfg: output translation configuration which is populated through
 *       the different configuration decoding steps
 * @status: must be zero'ed by the caller
 *
 * return < 0 in case of config decoding error (@status is filled
 * accordingly). Return 0 otherwise.
 */
static int smmuv2_decode_config(SMMUDevice *sdev, SMMUTransCfg *cfg,
                                SMMUv2Status *status)
{
    uint32_t sid = smmu_get_sid(sdev);
    SMMUv2State *s = sdev->smmu;
    int ret;

    ret = smmu_stream_id_match(s, sid, cfg, status);
    if (ret) {

        error_report("%s sid=0x%"PRIx32" no sid match",
                     __func__, sid);

        return -EINVAL;
    }

    if (cfg->aborted || cfg->bypassed) {

        error_report("%s sid=0x%"PRIx32" aborted or bypassed",
                     __func__, sid);

        return 0;
    }

    ret = smmu_decode_cbar(s, cfg, status);
    if (ret) {

        error_report("%s sid=0x%"PRIx32" cbar decode error",
                     __func__, sid);

        return ret;
    }

    ret = decode_ctx(s, cfg, status);
    if (ret) {
            error_report("%s sid=0x%"PRIx32" ctx decode error ret=%d",
                         __func__, sid, ret);
    }

    return ret;
}

/**
 * smmuv3_get_config - Look up for a cached copy of configuration data for
 * @sdev and on cache miss performs a configuration structure decoding from
 * guest RAM.
 *
 * @sdev: SMMUDevice handle
 * @event: output event info
 *
 * The configuration cache contains data resulting from both STE and CD
 * decoding under the form of an SMMUTransCfg struct. The hash table is indexed
 * by the SMMUDevice handle.
 */
static SMMUTransCfg *smmuv2_get_config(SMMUDevice *sdev, SMMUv2Status *status)
{
    SMMUv2State *s = sdev->smmu;
    SMMUState *bc = &s->smmu_state;
    SMMUTransCfg *cfg;

    cfg = g_hash_table_lookup(bc->configs, sdev);
    if (cfg) {
        sdev->cfg_cache_hits++;

//        trace_smmuv3_config_cache_hit(smmu_get_sid(sdev),
//                            sdev->cfg_cache_hits, sdev->cfg_cache_misses,
//                            100 * sdev->cfg_cache_hits /
//                            (sdev->cfg_cache_hits + sdev->cfg_cache_misses));
    } else {
        sdev->cfg_cache_misses++;

//        trace_smmuv3_config_cache_miss(smmu_get_sid(sdev),
//                            sdev->cfg_cache_hits, sdev->cfg_cache_misses,
//                            100 * sdev->cfg_cache_hits /
//                            (sdev->cfg_cache_hits + sdev->cfg_cache_misses));
        cfg = g_new0(SMMUTransCfg, 1);

        if (smmuv2_decode_config(sdev, cfg, status)) {
            g_free(cfg);
            cfg = NULL;
        }
//        if (!smmuv2_decode_config(&sdev->iommu, cfg, status)) {
//            g_hash_table_insert(bc->configs, sdev, cfg);
//        } else {
//            g_free(cfg);
//            cfg = NULL;
//        }
    }
    return cfg;
}

static void smmu_update_ctx_irq(SMMUv2State *s, unsigned int cb)
{
    unsigned int cb_offset = smmu_cb_offset(s, cb);
    uint32_t sctlr;
    uint32_t fsr;
    bool ie, tf;
    bool pending;

    fsr = s->regs[R_SMMU_CB0_FSR + cb_offset];
    sctlr = s->regs[R_SMMU_CB0_SCTLR + cb_offset];

    tf = FIELD_EX32(fsr, SMMU_CB0_FSR, TF);
    ie = FIELD_EX32(sctlr, SMMU_CB0_SCTLR, CFIE);
    pending = tf && ie;
    qemu_set_irq(s->irq.context[cb], pending);
}

static void smmu_fault(SMMUv2State *s, unsigned int cb, hwaddr addr)
{
    unsigned int cb_offset = smmu_cb_offset(s, cb);

    s->regs[R_SMMU_CB0_FSR + cb_offset] |= 1 << 1;

    /* FIXME: Stage 2 support */
//    s->regs[R_SMMU_CB0_IPAFAR_LOW + cb_offset] = addr;
//    s->regs[R_SMMU_CB0_IPAFAR_HIGH + cb_offset] = addr >> 32;
//    if (req->stage == 2) {
        s->regs[R_SMMU_CB0_FAR_LOW + cb_offset] = addr;
        s->regs[R_SMMU_CB0_FAR_HIGH + cb_offset] = addr >> 32;
//    }
    smmu_update_ctx_irq(s, cb);
}

static IOMMUTLBEntry smmuv2_translate(IOMMUMemoryRegion *mr, hwaddr addr,
                                      IOMMUAccessFlags flag, int iommu_idx)
{
    SMMUDevice *sdev = container_of(mr, SMMUDevice, iommu);
    SMMUv2State *s = sdev->smmu;
    uint32_t sid = smmu_get_sid(sdev);
    SMMUv2Status status = {.type = SMMU_FAULT_CTX_NONE,
                           .sid = sid};
    SMMUPTWEventInfo ptw_info = {};
//    SMMUState *bs = ARM_SMMU(s);
    uint64_t page_mask, aligned_addr;
    IOMMUTLBEntry *cached_entry = NULL;
    SMMUTransTableInfo *tt;
    SMMUTransCfg *cfg = NULL;
    IOMMUTLBEntry entry = {
        .target_as = &address_space_memory,
        .iova = addr,
        .translated_addr = addr,
        .addr_mask = ~(hwaddr)0,
        .perm = IOMMU_NONE,
    };
//    SMMUIOTLBKey key, *new_key;

    qemu_mutex_lock(&s->mutex);

    if (!smmuv2_enabled(s)) {
        status.status = SMMU_TRANS_DISABLE;
        status.addr = addr;

        error_report("%s iova=0x%"PRIx64" smmu disabled",
                     mr->parent_obj.name, addr);

        goto epilogue_no_cfg_free;
    }

    cfg = smmuv2_get_config(sdev, &status);
    if (!cfg) {
        error_report("%s iova=0x%"PRIx64" no cfg obtained",
                     mr->parent_obj.name, addr);

        status.addr = addr;
        goto epilogue_no_cfg_free;
    }

    if (cfg->aborted) {
        status.type = SMMU_FAULT_CTX_TRANS;
        status.addr = addr;
        status.flag = flag & 0x1;
        status.status = SMMU_TRANS_ERROR;

        error_report("%s iova=0x%"PRIx64" smmu aborted",
                     mr->parent_obj.name, addr);

        goto epilogue;
    }

    if (cfg->bypassed) {
        status.status = SMMU_TRANS_BYPASS;

        error_report("%s iova=0x%"PRIx64" smmu bypassed",
                     mr->parent_obj.name, addr);

        goto epilogue;
    }

    tt = select_tt(cfg, addr);
    if (!tt) {
        status.type = SMMU_FAULT_CTX_TRANS;
        status.addr = addr;
        status.flag = flag & 0x1;
        status.status = SMMU_TRANS_ERROR;

        error_report("%s iova=0x%"PRIx64" tt selection failed",
                     mr->parent_obj.name, addr);

        goto epilogue;
    }

    page_mask = (1ULL << (tt->granule_sz)) - 1;
    aligned_addr = addr & ~page_mask;

//    key.asid = cfg->asid;
//    key.iova = aligned_addr;
//
//    cached_entry = g_hash_table_lookup(bs->iotlb, &key);
//    if (cached_entry) {
//        cfg->iotlb_hits++;
//        trace_smmu_iotlb_cache_hit(cfg->asid, aligned_addr,
//                                   cfg->iotlb_hits, cfg->iotlb_misses,
//                                   100 * cfg->iotlb_hits /
//                                   (cfg->iotlb_hits + cfg->iotlb_misses));
//        if ((flag & IOMMU_WO) && !(cached_entry->perm & IOMMU_WO)) {
//            status.status = SMMU_TRANS_ERROR;
//            status.type = SMMU_FAULT_CTX_PERM;
//            status.addr = addr;
//            status.flag = flag & 0x1;
//        } else {
//            status.status = SMMU_TRANS_SUCCESS;
//        }
//        goto epilogue;
//    }
//
//    cfg->iotlb_misses++;
//    trace_smmu_iotlb_cache_miss(cfg->asid, addr & ~page_mask,
//                                cfg->iotlb_hits, cfg->iotlb_misses,
//                                100 * cfg->iotlb_hits /
//                                (cfg->iotlb_hits + cfg->iotlb_misses));
//
//    if (g_hash_table_size(bs->iotlb) >= SMMU_IOTLB_MAX_SIZE) {
//        smmu_iotlb_inv_all(bs);
//    }

    cached_entry = g_new0(IOMMUTLBEntry, 1);

    if (smmu_ptw(cfg, aligned_addr, flag, cached_entry, &ptw_info)) {
        g_free(cached_entry);

        error_report("%s iova=0x%"PRIx64" ptw error",
                     mr->parent_obj.name, addr);

        switch (ptw_info.type) {
        case SMMU_PTW_ERR_WALK_EABT:
            status.type = SMMU_FAULT_CTX_EXT;
            status.addr = addr;
            status.flag = flag & 0x1;
            break;
        case SMMU_PTW_ERR_TRANSLATION:
            status.type = SMMU_FAULT_CTX_TRANS;
            status.addr = addr;
            status.flag = flag & 0x1;
            break;
        case SMMU_PTW_ERR_ADDR_SIZE:
            status.type = SMMU_FAULT_CTX_TRANS;
            status.addr = addr;
            status.flag = flag & 0x1;
            break;
        case SMMU_PTW_ERR_ACCESS:
            status.type = SMMU_FAULT_CTX_PERM;
            status.addr = addr;
            status.flag = flag & 0x1;
            break;
        case SMMU_PTW_ERR_PERMISSION:
            status.type = SMMU_FAULT_CTX_PERM;
            status.addr = addr;
            status.flag = flag & 0x1;
            break;
        default:
            g_assert_not_reached();
        }
        status.status = SMMU_TRANS_ERROR;
    } else {
//        new_key = g_new0(SMMUIOTLBKey, 1);
//        new_key->asid = cfg->asid;
//        new_key->iova = aligned_addr;
//        g_hash_table_insert(bs->iotlb, new_key, cached_entry);

//        error_report("%s iova=0x%"PRIx64" ptw success",
//                     mr->parent_obj.name, addr);

        status.status = SMMU_TRANS_SUCCESS;
    }

epilogue:
    g_free(cfg);
epilogue_no_cfg_free:
    qemu_mutex_unlock(&s->mutex);
    switch (status.status) {
    case SMMU_TRANS_SUCCESS:
        entry.perm = flag;
        entry.translated_addr = cached_entry->translated_addr +
                                    (addr & page_mask);
        entry.addr_mask = cached_entry->addr_mask;

//        error_report("%s translation success for iova=0x%"PRIx64" -> pa=0x%"PRIx64"",
//                     mr->parent_obj.name, addr, entry.translated_addr);

        g_free(cached_entry);
//        trace_smmuv3_translate_success(mr->parent_obj.name, sid, addr,
//                                       entry.translated_addr, entry.perm);
        break;
    case SMMU_TRANS_DISABLE:
        entry.perm = flag;
        entry.addr_mask = ~TARGET_PAGE_MASK;
//        trace_smmuv3_translate_disable(mr->parent_obj.name, sid, addr,
//                                      entry.perm);
        break;
    case SMMU_TRANS_BYPASS:
        entry.perm = flag;
        entry.addr_mask = ~TARGET_PAGE_MASK;
//        trace_smmuv3_translate_bypass(mr->parent_obj.name, sid, addr,
//                                      entry.perm);
        break;
    case SMMU_TRANS_ABORT:
        /* no error is recorded on abort */
//        trace_smmuv3_translate_abort(mr->parent_obj.name, sid, addr,
//                                     entry.perm);
        break;
    case SMMU_TRANS_ERROR:
        error_report("%s translation failed for iova=0x%"PRIx64"(%s)",
                     mr->parent_obj.name, addr, smmu_event_string(status.type));
//        smmuv3_record_event(s, &event);
        smmu_fault(s, status.cb, status.addr);
        break;
    }

    return entry;
}

/**
 * smmuv2_notify_iova - call the notifier @n for a given
 * @asid and @iova tuple.
 *
 * @mr: IOMMU mr region handle
 * @n: notifier to be called
 * @asid: address space ID or negative value if we don't care
 * @iova: iova
 */
//static void smmuv2_notify_iova(IOMMUMemoryRegion *mr,
//                               IOMMUNotifier *n,
//                               int asid, dma_addr_t iova, bool leaf)
//{
//    SMMUDevice *sdev = container_of(mr, SMMUDevice, iommu);
//    SMMUv2Status status = {};
//    SMMUTransTableInfo *tt;
//    SMMUTransCfg *cfg;
//    IOMMUTLBEntry entry;
//
//    cfg = smmuv2_get_config(sdev, &status);
//    if (!cfg) {
//        return;
//    }
//
//    if (asid >= 0 && cfg->asid != asid) {
//        return;
//    }
//
//    tt = select_tt(cfg, iova);
//    if (!tt) {
//        return;
//    }
//
//    entry.target_as = &address_space_memory;
//    entry.iova = iova;
//    entry.addr_mask = (1 << tt->granule_sz) - 1;
//    entry.perm = IOMMU_NONE;
//    entry.arch_id = asid;
//    entry.leaf = leaf;
//
//    memory_region_notify_one(n, &entry);
//}
//
///* invalidate an asid/iova tuple in all mr's */
//static void smmuv2_inv_notifiers_iova(SMMUState *s, int asid,
//                                      dma_addr_t iova, bool leaf)
//{
//    SMMUDevice *sdev;
//
//    QLIST_FOREACH(sdev, &s->devices_with_notifiers, next) {
//        IOMMUMemoryRegion *mr = &sdev->iommu;
//        IOMMUNotifier *n;
//
////        trace_smmuv3_inv_notifiers_iova(mr->parent_obj.name, asid, iova);
//
//        IOMMU_NOTIFIER_FOREACH(n, mr) {
//            smmuv2_notify_iova(mr, n, asid, iova, leaf);
//        }
//    }
//}

static void smmu_gat(SMMUv2State *s, hwaddr addr, uint32_t cb, bool wr)
{
    SMMUv2Status status = {.status = SMMU_TRANS_SUCCESS,
                           .type = SMMU_FAULT_CTX_NONE,
                           .cb = cb,
                          };
    IOMMUAccessFlags flag = wr ? IOMMU_WO : IOMMU_RO;
    SMMUPTWEventInfo ptw_info = {};
    uint64_t page_mask, aligned_addr;
    IOMMUTLBEntry *cached_entry = NULL;
    SMMUTransTableInfo *tt;
    SMMUTransCfg *cfg;
    int ret;

    qemu_mutex_lock(&s->mutex);

    assert(!FIELD_EX32(s->regs[R_SMMU_GATSR], SMMU_GATSR, ACTIVE));

    /* .ACTIVE == 1, let know that ATS is ongoing */
    s->regs[R_SMMU_GATSR] |= 0x1;

    cfg = g_new0(SMMUTransCfg, 1);
    ret = smmu_decode_cbar(s, cfg, &status);
    if (ret) {
        status.type = SMMU_FAULT_GLOBAL_UNIMPL_CTX_BANK;
        status.status = SMMU_TRANS_ERROR;
        goto epilogue;
    }

    ret = decode_ctx(s, cfg, &status);
    if (ret) {
        status.type = SMMU_FAULT_GLOBAL_UNIMPL_CTX_BANK;
        status.status = SMMU_TRANS_ERROR;
        goto epilogue;
    }

    tt = select_tt(cfg, addr);
    if (!tt) {
        status.type = SMMU_FAULT_CTX_TRANS;
        status.status = SMMU_TRANS_ERROR;
        goto epilogue;
    }

    page_mask = (1ULL << (tt->granule_sz)) - 1;
    aligned_addr = addr & ~page_mask;

    cached_entry = g_new0(IOMMUTLBEntry, 1);
    if (smmu_ptw(cfg, aligned_addr, flag, cached_entry, &ptw_info)) {
        g_free(cached_entry);
        switch (ptw_info.type) {
        case SMMU_PTW_ERR_WALK_EABT:
            status.type = SMMU_FAULT_CTX_EXT;
            status.addr = addr;
            status.flag = flag & 0x1;
            break;
        case SMMU_PTW_ERR_TRANSLATION:
            status.type = SMMU_FAULT_CTX_TRANS;
            status.addr = addr;
            status.flag = flag & 0x1;
            break;
        case SMMU_PTW_ERR_ADDR_SIZE:
            status.type = SMMU_FAULT_CTX_TRANS;
            status.addr = addr;
            status.flag = flag & 0x1;
            break;
        case SMMU_PTW_ERR_ACCESS:
            status.type = SMMU_FAULT_CTX_PERM;
            status.addr = addr;
            status.flag = flag & 0x1;
            break;
        case SMMU_PTW_ERR_PERMISSION:
            status.type = SMMU_FAULT_CTX_PERM;
            status.addr = addr;
            status.flag = flag & 0x1;
            break;
        default:
            g_assert_not_reached();
        }
        status.status = SMMU_TRANS_ERROR;
    }

epilogue:
    qemu_mutex_unlock(&s->mutex);
    switch (status.status) {
    case SMMU_TRANS_SUCCESS:
        /* TODO: fill in attr */
        s->regs[R_SMMU_GPAR] = cached_entry->translated_addr;
        s->regs[R_SMMU_GPAR_H] = cached_entry->translated_addr >> 32;
        g_free(cached_entry);
        break;
    case SMMU_TRANS_ERROR:
        s->regs[R_SMMU_GPAR] = 1;
        smmu_fault(s, status.cb, status.addr);
        break;
    default:
        break;
    }

    /* Release */
    s->regs[R_SMMU_GATSR] &= ~0x1;
}

static void smmu_gats1pr(RegisterInfo *reg, uint64_t val)
{
    SMMUv2State *s = ARM_SMMUV2(reg->opaque);
    uint32_t bottom = s->regs[(reg->access->addr / 4) - 1];
    uint32_t cb = FIELD_EX32(bottom, SMMU_GATS1PR, NDX);
    hwaddr addr;

    val <<= 32;
    val |= bottom;

    addr = FIELD_EX64(val, SMMU_GATS1PR, ADDR) << R_SMMU_GATS1PR_ADDR_SHIFT;
    smmu_gat(s, addr, cb, false);
}

static void smmuv2_inv_notifiers_all(RegisterInfo *reg, uint64_t val)
{
    SMMUv2State *s = ARM_SMMUV2(reg->opaque);
    SMMUState *bs = ARM_SMMU(s);

    smmu_inv_notifiers_all(&s->smmu_state);
    smmu_iotlb_inv_all(bs);
}

static void smmu_gats1pw(RegisterInfo *reg, uint64_t val)
{
    SMMUv2State *s = ARM_SMMUV2(reg->opaque);
    uint32_t bottom = s->regs[(reg->access->addr / 4) - 1];
    uint32_t cb = FIELD_EX32(bottom, SMMU_GATS1PW, NDX);
    hwaddr addr;

    val <<= 32;
    val |= bottom;

    addr = FIELD_EX64(val, SMMU_GATS1PW, ADDR) << R_SMMU_GATS1PW_ADDR_SHIFT;
    smmu_gat(s, addr, cb, true);
}

static void smmu_gats12pr(RegisterInfo *reg, uint64_t val)
{
    g_assert_not_reached();
}

static void smmu_gats12pw(RegisterInfo *reg, uint64_t val)
{
    g_assert_not_reached();
}

static void smmu_nscr0_pw(RegisterInfo *reg, uint64_t val)
{
    SMMUv2State *s = ARM_SMMUV2(reg->opaque);

    /* FIXME: Take care of secure vs non-secure accesses.  */
    s->regs[R_SMMU_SCR0] = val;
    s->regs[R_SMMU_NSCR0] = val;
}

static void smmu_fsr_pw(RegisterInfo *reg, uint64_t val)
{
    SMMUv2State *s = ARM_SMMUV2(reg->opaque);
    unsigned int i;

    for (i = 0; i < 16; i++) {
        smmu_update_ctx_irq(s, i);
    }
}

static void smmuv2_flush_config(SMMUDevice *sdev)
{
    SMMUv2State *s = sdev->smmu;
    SMMUState *bc = &s->smmu_state;

//    trace_smmuv3_config_cache_inv(smmu_get_sid(sdev));
    g_hash_table_remove(bc->configs, sdev);
}

static void smmuv2_notify_config_change(RegisterInfo *reg, uint64_t val)
{
#ifdef __linux__
    SMMUv2State *s = ARM_SMMUV2(reg->opaque);
    SMMUState *bs = &s->smmu_state;
    SMMUv2Status status = { .type = SMMU_FAULT_CTX_NONE };
    IOMMUConfig iommu_config = { };
    uint32_t smr, smr_idx, mask, sid, tcr, a1;
    IOMMUMemoryRegion *mr;
    unsigned int cb_offset;
    SMMUTransCfg *cfg;
    SMMUDevice *sdev;
    uint64_t ttbr;
    bool valid;
    int ret;

    /* Fist find our SID in corresponding SMR register... */
    smr_idx = (reg->access->addr - A_SMMU_S2CR0) / 4;
    smr = s->regs[R_SMMU_SMR0 + smr_idx];
    valid = FIELD_EX32(smr, SMMU_SMR0, VALID);
    sid = FIELD_EX32(smr, SMMU_SMR0, ID);
    mask = FIELD_EX32(smr, SMMU_SMR0, MASK);
    if (!valid) {
        error_report("%s SMR[%d] is not valid", __func__, (int)smr_idx);
        return;
    }

    error_report("%s SMR[%d] is valid -> SID 0x%x mask 0x%x",
            __func__, (int)smr_idx, sid, mask);


    /* ... then lookup IOMMU memory region */
    mr = smmu_find_smmu_platform_dev(bs, sid);
    if (!mr) {
        error_report("%s IOMMU memory region not found", __func__);
        return;
    }

    error_report("%s IOMMU memory region found", __func__);

    sdev = container_of(mr, SMMUDevice, iommu);
    assert(sdev->smmu == bs);

    /* flush QEMU config cache */
    smmuv2_flush_config(sdev);

    if (sdev->dev_type == SMMU_DEV_PCI) {
        error_report("%s SMMU_DEV_PCI dev", __func__);
        ret = !pci_device_is_pasid_ops_set(sdev->bus, sdev->devfn);
    } else if (sdev->dev_type == SMMU_DEV_PLATFORM) {
        error_report("%s SMMU_DEV_PLATFORM dev", __func__);
        ret = !vfio_platform_device_is_pasid_ops_set(sdev->sbdev);
    }

    if (ret)
        return;

    status.sid = sid;
    cfg = smmuv2_get_config(sdev, &status);
    if (!cfg) {
        error_report("%s cfg not found", __func__);
        return;
    }

    iommu_config.pasid_cfg.version = PASID_TABLE_CFG_VERSION_1;
    iommu_config.pasid_cfg.format = IOMMU_PASID_FORMAT_SMMUV2;
    iommu_config.pasid_cfg.smmuv2.version = PASID_TABLE_SMMUV2_CFG_VERSION_1;

    s = sdev->smmu;
    cb_offset = smmu_cb_offset(s, status.cb);

    /* TTBRs */
    ttbr = s->regs[R_SMMU_CB0_TTBR0_HIGH + cb_offset];
    ttbr <<= 32;
    ttbr |= s->regs[R_SMMU_CB0_TTBR0_LOW + cb_offset];
    iommu_config.pasid_cfg.smmuv2.ttbr[0] = ttbr;

    ttbr = s->regs[R_SMMU_CB0_TTBR1_HIGH + cb_offset];
    ttbr <<= 32;
    ttbr |= s->regs[R_SMMU_CB0_TTBR1_LOW + cb_offset];
    iommu_config.pasid_cfg.smmuv2.ttbr[1] = ttbr;

    /* ASID */
    tcr = s->regs[R_SMMU_CB0_TCR_LPAE + cb_offset];
    a1 = FIELD_EX32(tcr, SMMU_CB0_TCR_LPAE, A1);
    if (a1) {
        ttbr = s->regs[R_SMMU_CB0_TTBR1_HIGH + cb_offset];
        iommu_config.pasid_cfg.smmuv2.asid = FIELD_EX32(ttbr, SMMU_CB0_TTBR1_HIGH, ASID);
    } else {
        ttbr = s->regs[R_SMMU_CB0_TTBR0_HIGH + cb_offset];
        iommu_config.pasid_cfg.smmuv2.asid = FIELD_EX32(ttbr, SMMU_CB0_TTBR0_HIGH, ASID);
    }

    /* TCR and MAIR */
    iommu_config.pasid_cfg.smmuv2.tcr[0] = s->regs[R_SMMU_CB0_TCR_LPAE + cb_offset];
    iommu_config.pasid_cfg.smmuv2.tcr[1] = s->regs[R_SMMU_CB0_TCR2 + cb_offset];

    iommu_config.pasid_cfg.smmuv2.mair[0] = s->regs[R_SMMU_CB0_PRRR_MAIR0 + cb_offset];
    iommu_config.pasid_cfg.smmuv2.mair[1] = s->regs[R_SMMU_CB0_NMRR_MAIR1 + cb_offset];

    /* We don't support S1 32bit page-tables yet */
    if (!FIELD_EX32(s->regs[R_SMMU_CBA2R0 + status.cb], SMMU_CBA2R0, VA64)) {
        error_report("SMMU does not support S1 32bit page-tables yet");
        g_assert_not_reached();
    }
    iommu_config.pasid_cfg.smmuv2.s1fmt = ASID_TABLE_SMMUV2_CTX_FMT_AARCH64;

    if (cfg->disabled || cfg->bypassed) {
        iommu_config.pasid_cfg.config = IOMMU_PASID_CONFIG_BYPASS;
    } else if (cfg->aborted) {
        iommu_config.pasid_cfg.config = IOMMU_PASID_CONFIG_ABORT;
    } else {
        iommu_config.pasid_cfg.config = IOMMU_PASID_CONFIG_TRANSLATE;
    }

//    trace_smmuv3_notify_config_change(mr->parent_obj.name,
//                                      iommu_config.pasid_cfg.config,
//                                      iommu_config.pasid_cfg.base_ptr);

    if (sdev->dev_type == SMMU_DEV_PCI)
        ret = pci_device_set_pasid_table(sdev->bus, sdev->devfn, &iommu_config);
    else if (sdev->dev_type == SMMU_DEV_PLATFORM)
        ret = vfio_platform_device_set_pasid_table(sdev->sbdev, &iommu_config);

    if (ret)
        error_report("Failed to pass PASID table to host for iommu mr %s (%m)",
                     mr->parent_obj.name);
    else
        error_report("Passed PASID table to host for iommu mr %s (%m)",
                     mr->parent_obj.name);
#endif
}

//static void smmuv2_s2cr(RegisterInfo *reg, uint64_t val)
//{
//    SMMUv2State *s = ARM_SMMUV2(reg->opaque);
//    uint32_t s2cr = s->regs[(reg->access->addr / 4)];
//    uint32_t type, cb, smr_idx;
//
//    type = FIELD_EX32(s2cr, SMMU_S2CR0, TYPE);
//    if (type == S2CR_TYPE_TRANS) {
//        cb = FIELD_EX32(s2cr, SMMU_S2CR0, CBNDX_VMID);
//        smr_idx = (reg->access->addr - A_SMMU_S2CR0) / 4;
//        s->cb_to_s2cr[cb] =
//    }
//
//
//}
//
//static void smmuv2_sctlr(RegisterInfo *reg, uint64_t val)
//{
//
//}

static const RegisterAccessInfo smmu500_regs_info[] = {
    /* Manually added.  */
    {   .name = "SMMU_GATS1PR",  .addr = A_SMMU_GATS1PR,
    },
    {   .name = "SMMU_GATS1PR_H",  .addr = A_SMMU_GATS1PR_H,
        .post_write = smmu_gats1pr,
    },
    {   .name = "SMMU_GATS1PW",  .addr = A_SMMU_GATS1PW,
    },
    {   .name = "SMMU_GATS1PW_H",  .addr = A_SMMU_GATS1PW_H,
        .post_write = smmu_gats1pw,
    },

    {   .name = "SMMU_GATS12PR",  .addr = A_SMMU_GATS12PR,
    },
    {   .name = "SMMU_GATS12PR_H",  .addr = A_SMMU_GATS12PR_H,
//        .post_write = smmu_gats12pr,
    },

    {   .name = "SMMU_GATS12PW",  .addr = A_SMMU_GATS12PW,
    },
    {   .name = "SMMU_GATS12PW_H",  .addr = A_SMMU_GATS12PW_H,
//        .post_write = smmu_gats12pw,
    },

    {   .name = "SMMU_GPAR",  .addr = A_SMMU_GPAR, },
    {   .name = "SMMU_GPAR_H",  .addr = A_SMMU_GPAR_H, },
    {   .name = "SMMU_GATSR",  .addr = A_SMMU_GATSR, },

    {   .name = "SMMU_SCR0",  .addr = A_SMMU_SCR0,
        .reset = 0x200001,
        .ro = 0x200330,
    },{ .name = "SMMU_SCR1",  .addr = A_SMMU_SCR1,
        .reset = 0x2013010,
        .ro = 0x10ff0000,
    },{ .name = "SMMU_SACR",  .addr = A_SMMU_SACR,
        .reset = 0x4000004,
    },{ .name = "SMMU_SIDR0",  .addr = A_SMMU_SIDR0,
        .reset = 0xfc013e30,
        .ro = 0xffff7eff,
    },{ .name = "SMMU_SIDR1",  .addr = A_SMMU_SIDR1,
        .reset = 0x30000f10,
        .ro = 0xf0ff9fff,
    },{ .name = "SMMU_SIDR2",  .addr = A_SMMU_SIDR2,
        .reset = 0x5555,
        .ro = 0x7fff,
    },{ .name = "SMMU_SIDR7",  .addr = A_SMMU_SIDR7,
        .reset = 0x21,
        .ro = 0xff,
    },{ .name = "SMMU_SGFAR_LOW",  .addr = A_SMMU_SGFAR_LOW,
    },{ .name = "SMMU_SGFAR_HIGH",  .addr = A_SMMU_SGFAR_HIGH,
    },{ .name = "SMMU_SGFSR",  .addr = A_SMMU_SGFSR,
    },{ .name = "SMMU_SGFSRRESTORE",  .addr = A_SMMU_SGFSRRESTORE,
    },{ .name = "SMMU_SGFSYNR0",  .addr = A_SMMU_SGFSYNR0,
        .ro = 0x40,
    },{ .name = "SMMU_SGFSYNR1",  .addr = A_SMMU_SGFSYNR1,
    },{ .name = "SMMU_STLBIALL",  .addr = A_SMMU_STLBIALL,
        .post_write = smmuv2_inv_notifiers_all,
    },{ .name = "SMMU_TLBIVMID",  .addr = A_SMMU_TLBIVMID,
        .post_write = smmuv2_inv_notifiers_all,
    },{ .name = "SMMU_TLBIALLNSNH",  .addr = A_SMMU_TLBIALLNSNH,
        .post_write = smmuv2_inv_notifiers_all,
    },{ .name = "SMMU_STLBGSYNC",  .addr = A_SMMU_STLBGSYNC,
    },{ .name = "SMMU_STLBGSTATUS",  .addr = A_SMMU_STLBGSTATUS,
        .ro = 0x1,
    },{ .name = "SMMU_DBGRPTRTBU",  .addr = A_SMMU_DBGRPTRTBU,
    },{ .name = "SMMU_DBGRDATATBU",  .addr = A_SMMU_DBGRDATATBU,
        .ro = 0xffffffff,
    },{ .name = "SMMU_DBGRPTRTCU",  .addr = A_SMMU_DBGRPTRTCU,
    },{ .name = "SMMU_DBGRDATATCU",  .addr = A_SMMU_DBGRDATATCU,
        .ro = 0xffffffff,
    },{ .name = "SMMU_STLBIVALM_LOW",  .addr = A_SMMU_STLBIVALM_LOW,
    },{ .name = "SMMU_STLBIVALM_HIGH",  .addr = A_SMMU_STLBIVALM_HIGH,
        .post_write = smmuv2_inv_notifiers_all,
    },{ .name = "SMMU_STLBIVAM_LOW",  .addr = A_SMMU_STLBIVAM_LOW,
    },{ .name = "SMMU_STLBIVAM_HIGH",  .addr = A_SMMU_STLBIVAM_HIGH,
        .post_write = smmuv2_inv_notifiers_all,
    },{ .name = "SMMU_STLBIALLM",  .addr = A_SMMU_STLBIALLM,
        .post_write = smmuv2_inv_notifiers_all,
    },{ .name = "SMMU_NSCR0",  .addr = A_SMMU_NSCR0,
        .reset = 0x200001,
        .ro = 0x200330,
        .post_write = smmu_nscr0_pw,
    },{ .name = "SMMU_NSACR",  .addr = A_SMMU_NSACR,
        .reset = 0x400001c,
    },{ .name = "SMMU_NSGFAR_LOW",  .addr = A_SMMU_NSGFAR_LOW,
    },{ .name = "SMMU_NSGFAR_HIGH",  .addr = A_SMMU_NSGFAR_HIGH,
    },{ .name = "SMMU_NSGFSR",  .addr = A_SMMU_NSGFSR,
    },{ .name = "SMMU_NSGFSRRESTORE",  .addr = A_SMMU_NSGFSRRESTORE,
    },{ .name = "SMMU_NSGFSYNR0",  .addr = A_SMMU_NSGFSYNR0,
        .ro = 0x40,
    },{ .name = "SMMU_NSGFSYNDR1",  .addr = A_SMMU_NSGFSYNDR1,
        .ro = 0x7fff0000,
    },{ .name = "SMMU_NSTLBGSYNC",  .addr = A_SMMU_NSTLBGSYNC,
    },{ .name = "SMMU_NSTLBGSTATUS",  .addr = A_SMMU_NSTLBGSTATUS,
        .ro = 0x1,
    },{ .name = "SMMU_PIDR4",  .addr = A_SMMU_PIDR4,
        .reset = 0x4,
        .ro = 0xff,
    },{ .name = "SMMU_PIDR5",  .addr = A_SMMU_PIDR5,
        .ro = 0xffffffff,
    },{ .name = "SMMU_PIDR6",  .addr = A_SMMU_PIDR6,
        .ro = 0xffffffff,
    },{ .name = "SMMU_PIDR7",  .addr = A_SMMU_PIDR7,
        .ro = 0xffffffff,
    },{ .name = "SMMU_PIDR0",  .addr = A_SMMU_PIDR0,
        .reset = 0x81,
        .ro = 0xff,
    },{ .name = "SMMU_PIDR1",  .addr = A_SMMU_PIDR1,
        .reset = 0xb4,
        .ro = 0xff,
    },{ .name = "SMMU_PIDR2",  .addr = A_SMMU_PIDR2,
        .reset = 0x1b,
        .ro = 0xff,
    },{ .name = "SMMU_PIDR3",  .addr = A_SMMU_PIDR3,
        .ro = 0xff,
    },{ .name = "SMMU_CIDR0",  .addr = A_SMMU_CIDR0,
        .reset = 0xd,
        .ro = 0xff,
    },{ .name = "SMMU_CIDR1",  .addr = A_SMMU_CIDR1,
        .reset = 0xf0,
        .ro = 0xff,
    },{ .name = "SMMU_CIDR2",  .addr = A_SMMU_CIDR2,
        .reset = 0x5,
        .ro = 0xff,
    },{ .name = "SMMU_CIDR3",  .addr = A_SMMU_CIDR3,
        .reset = 0xb1,
        .ro = 0xff,
    },{ .name = "SMMU_ITCTRL",  .addr = A_SMMU_ITCTRL,
    },{ .name = "SMMU_ITIP",  .addr = A_SMMU_ITIP,
        .ro = 0x1,
    },{ .name = "SMMU_ITOP_GLBL",  .addr = A_SMMU_ITOP_GLBL,
        .ro = 0x202,
    },{ .name = "SMMU_ITOP_PERF_INDEX",  .addr = A_SMMU_ITOP_PERF_INDEX,
    },{ .name = "SMMU_ITOP_CXT0TO31_RAM0",  .addr = A_SMMU_ITOP_CXT0TO31_RAM0,
    },{ .name = "SMMU_TBUQOS0",  .addr = A_SMMU_TBUQOS0,
    },{ .name = "SMMU_PER",  .addr = A_SMMU_PER,
        .ro = 0xffff,
    },{ .name = "SMMU_TBU_PWR_STATUS",  .addr = A_SMMU_TBU_PWR_STATUS,
        .ro = 0xffffffff,
    },{ .name = "PMEVCNTR0",  .addr = A_PMEVCNTR0,
    },{ .name = "PMEVCNTR1",  .addr = A_PMEVCNTR1,
    },{ .name = "PMEVCNTR2",  .addr = A_PMEVCNTR2,
    },{ .name = "PMEVCNTR3",  .addr = A_PMEVCNTR3,
    },{ .name = "PMEVCNTR4",  .addr = A_PMEVCNTR4,
    },{ .name = "PMEVCNTR5",  .addr = A_PMEVCNTR5,
    },{ .name = "PMEVCNTR6",  .addr = A_PMEVCNTR6,
    },{ .name = "PMEVCNTR7",  .addr = A_PMEVCNTR7,
    },{ .name = "PMEVCNTR8",  .addr = A_PMEVCNTR8,
    },{ .name = "PMEVCNTR9",  .addr = A_PMEVCNTR9,
    },{ .name = "PMEVCNTR10",  .addr = A_PMEVCNTR10,
    },{ .name = "PMEVCNTR11",  .addr = A_PMEVCNTR11,
    },{ .name = "PMEVCNTR12",  .addr = A_PMEVCNTR12,
    },{ .name = "PMEVCNTR13",  .addr = A_PMEVCNTR13,
    },{ .name = "PMEVCNTR14",  .addr = A_PMEVCNTR14,
    },{ .name = "PMEVCNTR15",  .addr = A_PMEVCNTR15,
    },{ .name = "PMEVCNTR16",  .addr = A_PMEVCNTR16,
    },{ .name = "PMEVCNTR17",  .addr = A_PMEVCNTR17,
    },{ .name = "PMEVCNTR18",  .addr = A_PMEVCNTR18,
    },{ .name = "PMEVCNTR19",  .addr = A_PMEVCNTR19,
    },{ .name = "PMEVCNTR20",  .addr = A_PMEVCNTR20,
    },{ .name = "PMEVCNTR21",  .addr = A_PMEVCNTR21,
    },{ .name = "PMEVCNTR22",  .addr = A_PMEVCNTR22,
    },{ .name = "PMEVCNTR23",  .addr = A_PMEVCNTR23,
    },{ .name = "PMEVTYPER0",  .addr = A_PMEVTYPER0,
    },{ .name = "PMEVTYPER1",  .addr = A_PMEVTYPER1,
    },{ .name = "PMEVTYPER2",  .addr = A_PMEVTYPER2,
    },{ .name = "PMEVTYPER3",  .addr = A_PMEVTYPER3,
    },{ .name = "PMEVTYPER4",  .addr = A_PMEVTYPER4,
    },{ .name = "PMEVTYPER5",  .addr = A_PMEVTYPER5,
    },{ .name = "PMEVTYPER6",  .addr = A_PMEVTYPER6,
    },{ .name = "PMEVTYPER7",  .addr = A_PMEVTYPER7,
    },{ .name = "PMEVTYPER8",  .addr = A_PMEVTYPER8,
    },{ .name = "PMEVTYPER9",  .addr = A_PMEVTYPER9,
    },{ .name = "PMEVTYPER10",  .addr = A_PMEVTYPER10,
    },{ .name = "PMEVTYPER11",  .addr = A_PMEVTYPER11,
    },{ .name = "PMEVTYPER12",  .addr = A_PMEVTYPER12,
    },{ .name = "PMEVTYPER13",  .addr = A_PMEVTYPER13,
    },{ .name = "PMEVTYPER14",  .addr = A_PMEVTYPER14,
    },{ .name = "PMEVTYPER15",  .addr = A_PMEVTYPER15,
    },{ .name = "PMEVTYPER16",  .addr = A_PMEVTYPER16,
    },{ .name = "PMEVTYPER17",  .addr = A_PMEVTYPER17,
    },{ .name = "PMEVTYPER18",  .addr = A_PMEVTYPER18,
    },{ .name = "PMEVTYPER19",  .addr = A_PMEVTYPER19,
    },{ .name = "PMEVTYPER20",  .addr = A_PMEVTYPER20,
    },{ .name = "PMEVTYPER21",  .addr = A_PMEVTYPER21,
    },{ .name = "PMEVTYPER22",  .addr = A_PMEVTYPER22,
    },{ .name = "PMEVTYPER23",  .addr = A_PMEVTYPER23,
    },{ .name = "PMCGCR0",  .addr = A_PMCGCR0,
        .reset = 0x4000000,
        .ro = 0xf7f0000,
    },{ .name = "PMCGCR1",  .addr = A_PMCGCR1,
        .reset = 0x4010000,
        .ro = 0xf7f0000,
    },{ .name = "PMCGCR2",  .addr = A_PMCGCR2,
        .reset = 0x4020000,
        .ro = 0xf7f0000,
    },{ .name = "PMCGCR3",  .addr = A_PMCGCR3,
        .reset = 0x4030000,
        .ro = 0xf7f0000,
    },{ .name = "PMCGCR4",  .addr = A_PMCGCR4,
        .reset = 0x4040000,
        .ro = 0xf7f0000,
    },{ .name = "PMCGCR5",  .addr = A_PMCGCR5,
        .reset = 0x4050000,
        .ro = 0xf7f0000,
    },{ .name = "PMCGSMR0",  .addr = A_PMCGSMR0,
    },{ .name = "PMCGSMR1",  .addr = A_PMCGSMR1,
    },{ .name = "PMCGSMR2",  .addr = A_PMCGSMR2,
    },{ .name = "PMCGSMR3",  .addr = A_PMCGSMR3,
    },{ .name = "PMCGSMR4",  .addr = A_PMCGSMR4,
    },{ .name = "PMCGSMR5",  .addr = A_PMCGSMR5,
    },{ .name = "PMCNTENSET",  .addr = A_PMCNTENSET,
    },{ .name = "PMCNTENCLR",  .addr = A_PMCNTENCLR,
    },{ .name = "PMINTENSET",  .addr = A_PMINTENSET,
    },{ .name = "PMINTENCLR",  .addr = A_PMINTENCLR,
    },{ .name = "PMOVSCLR",  .addr = A_PMOVSCLR,
    },{ .name = "PMOVSSET",  .addr = A_PMOVSSET,
    },{ .name = "PMCFGR",  .addr = A_PMCFGR,
        .reset = 0x5011f17,
        .ro = 0xff09ffff,
    },{ .name = "PMCR",  .addr = A_PMCR,
        .ro = 0xff000002,
    },{ .name = "PMCEID0",  .addr = A_PMCEID0,
        .reset = 0x30303,
        .ro = 0x38383,
    },{ .name = "PMAUTHSTATUS",  .addr = A_PMAUTHSTATUS,
        .reset = 0x80,
        .ro = 0xff,
    },{ .name = "PMDEVTYPE",  .addr = A_PMDEVTYPE,
        .reset = 0x56,
        .ro = 0xff,
    }
};

static const RegisterAccessInfo smmu_cb_regs_info[] = {
    { .name = "AR",  .addr = A_SMMU_CBAR0,
      .reset = 0x20000,
      .ro = 0xff000000,
    },{ .name = "FRSYNRA",  .addr = A_SMMU_CBFRSYNRA0,
      .ro = 0x7fff0000,
    },{ .name = "A2R",  .addr = A_SMMU_CBA2R0,
    }
};

static const RegisterAccessInfo smmu_cb_page_regs_info[] = {
    { .name = "SCTLR",  .addr = A_SMMU_CB0_SCTLR,
        .reset = 0x100,
        .ro = 0x1000,
    },{ .name = "ACTLR",  .addr = A_SMMU_CB0_ACTLR,
        .reset = 0x3,
    },{ .name = "RESUME",  .addr = A_SMMU_CB0_RESUME,
    },{ .name = "TCR2",  .addr = A_SMMU_CB0_TCR2,
        .reset = 0x60,
        .ro = 0x60,
    },{ .name = "TTBR0_LOW",  .addr = A_SMMU_CB0_TTBR0_LOW,
        .ro = 0x4,
    },{ .name = "TTBR0_HIGH",  .addr = A_SMMU_CB0_TTBR0_HIGH,
    },{ .name = "TTBR1_LOW",  .addr = A_SMMU_CB0_TTBR1_LOW,
    },{ .name = "TTBR1_HIGH",  .addr = A_SMMU_CB0_TTBR1_HIGH,
    },{ .name = "TCR_LPAE",  .addr = A_SMMU_CB0_TCR_LPAE,
    },{ .name = "CONTEXTIDR",  .addr = A_SMMU_CB0_CONTEXTIDR,
    },{ .name = "PRRR_MAIR0",  .addr = A_SMMU_CB0_PRRR_MAIR0,
    },{ .name = "NMRR_MAIR1",  .addr = A_SMMU_CB0_NMRR_MAIR1,
    },{ .name = "FSR",  .addr = A_SMMU_CB0_FSR,
        .w1c = 0xffffffff,
        .post_write = smmu_fsr_pw,
    },{ .name = "FSRRESTORE",  .addr = A_SMMU_CB0_FSRRESTORE,
    },{ .name = "FAR_LOW",  .addr = A_SMMU_CB0_FAR_LOW,
    },{ .name = "FAR_HIGH",  .addr = A_SMMU_CB0_FAR_HIGH,
    },{ .name = "FSYNR0",  .addr = A_SMMU_CB0_FSYNR0,
        .ro = 0x200,
    },{ .name = "IPAFAR_LOW",  .addr = A_SMMU_CB0_IPAFAR_LOW,
        .ro = 0xfff,
    },{ .name = "IPAFAR_HIGH",  .addr = A_SMMU_CB0_IPAFAR_HIGH,
    },{ .name = "TLBIVA_LOW",  .addr = A_SMMU_CB0_TLBIVA_LOW,
    },{ .name = "TLBIVA_HIGH",  .addr = A_SMMU_CB0_TLBIVA_HIGH,
        .post_write = smmuv2_inv_notifiers_all,
    },{ .name = "TLBIVAA_LOW",  .addr = A_SMMU_CB0_TLBIVAA_LOW,
    },{ .name = "TLBIVAA_HIGH",  .addr = A_SMMU_CB0_TLBIVAA_HIGH,
        .post_write = smmuv2_inv_notifiers_all,
    },{ .name = "TLBIASID",  .addr = A_SMMU_CB0_TLBIASID,
        .post_write = smmuv2_inv_notifiers_all,
    },{ .name = "TLBIALL",  .addr = A_SMMU_CB0_TLBIALL,
        .post_write = smmuv2_inv_notifiers_all,
    },{ .name = "TLBIVAL_LOW",  .addr = A_SMMU_CB0_TLBIVAL_LOW,
    },{ .name = "TLBIVAL_HIGH",  .addr = A_SMMU_CB0_TLBIVAL_HIGH,
        .post_write = smmuv2_inv_notifiers_all,
    },{ .name = "TLBIVAAL_LOW",  .addr = A_SMMU_CB0_TLBIVAAL_LOW,
    },{ .name = "TLBIVAAL_HIGH",  .addr = A_SMMU_CB0_TLBIVAAL_HIGH,
        .post_write = smmuv2_inv_notifiers_all,
    },{ .name = "TLBIIPAS2_LOW",  .addr = A_SMMU_CB0_TLBIIPAS2_LOW,
    },{ .name = "TLBIIPAS2_HIGH",  .addr = A_SMMU_CB0_TLBIIPAS2_HIGH,
        .post_write = smmuv2_inv_notifiers_all,
    },{ .name = "TLBIIPAS2L_LOW",  .addr = A_SMMU_CB0_TLBIIPAS2L_LOW,
    },{ .name = "TLBIIPAS2L_HIGH",  .addr = A_SMMU_CB0_TLBIIPAS2L_HIGH,
        .post_write = smmuv2_inv_notifiers_all,
    },{ .name = "TLBSYNC",  .addr = A_SMMU_CB0_TLBSYNC,
    },{ .name = "TLBSTATUS",  .addr = A_SMMU_CB0_TLBSTATUS,
        .ro = 0x1,
    },{ .name = "PMEVCNTR0",  .addr = A_SMMU_CB0_PMEVCNTR0,
    },{ .name = "PMEVCNTR1",  .addr = A_SMMU_CB0_PMEVCNTR1,
    },{ .name = "PMEVCNTR2",  .addr = A_SMMU_CB0_PMEVCNTR2,
    },{ .name = "PMEVCNTR3",  .addr = A_SMMU_CB0_PMEVCNTR3,
    },{ .name = "PMEVTYPER0",  .addr = A_SMMU_CB0_PMEVTYPER0,
    },{ .name = "PMEVTYPER1",  .addr = A_SMMU_CB0_PMEVTYPER1,
    },{ .name = "PMEVTYPER2",  .addr = A_SMMU_CB0_PMEVTYPER2,
    },{ .name = "PMEVTYPER3",  .addr = A_SMMU_CB0_PMEVTYPER3,
    },{ .name = "PMCFGR",  .addr = A_SMMU_CB0_PMCFGR,
        .reset = 0x11f03,
        .ro = 0xff09ffff,
    },{ .name = "PMCR",  .addr = A_SMMU_CB0_PMCR,
        .ro = 0xff000002,
    },{ .name = "PMCEID",  .addr = A_SMMU_CB0_PMCEID,
        .reset = 0x30303,
        .ro = 0x38383,
    },{ .name = "PMCNTENSE",  .addr = A_SMMU_CB0_PMCNTENSE,
    },{ .name = "PMCNTENCLR",  .addr = A_SMMU_CB0_PMCNTENCLR,
    },{ .name = "PMCNTENSET",  .addr = A_SMMU_CB0_PMCNTENSET,
    },{ .name = "PMINTENCLR",  .addr = A_SMMU_CB0_PMINTENCLR,
    },{ .name = "PMOVSCLR",  .addr = A_SMMU_CB0_PMOVSCLR,
    },{ .name = "PMOVSSET",  .addr = A_SMMU_CB0_PMOVSSET,
    },{ .name = "PMAUTHSTATUS",  .addr = A_SMMU_CB0_PMAUTHSTATUS,
        .reset = 0x80,
        .ro = 0xff,
    }
};

#define NUM_REGS_PER_CB (ARRAY_SIZE(smmu_cb_page_regs_info) + \
                         ARRAY_SIZE(smmu_cb_regs_info))

static void smmu_reset(DeviceState *dev)
{
    SMMUv2State *s = ARM_SMMUV2(dev);
    SMMUv2Class *c = ARM_SMMUV2_GET_CLASS(s);
    unsigned int num_pages_log2 = 31 - clz32(s->cfg.num_cb);
    unsigned int i;

    c->parent_reset(dev);

    for (i = 0; i < ARRAY_SIZE(s->regs_info); ++i) {
        register_reset(&s->regs_info[i]);
    }

    ARRAY_FIELD_DP32(s->regs, SMMU_SIDR0, NUMSMRG, s->cfg.num_smr);
    ARRAY_FIELD_DP32(s->regs, SMMU_SIDR1, NUMCB, s->cfg.num_cb);
    ARRAY_FIELD_DP32(s->regs, SMMU_SIDR1, NUMPAGENDXB, num_pages_log2 - 1);
}

static const MemoryRegionOps smmu_mem_ops = {
    .read = register_read_memory,
    .write = register_write_memory,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = {
        .min_access_size = 4,
        .max_access_size = 8,
    },
};

static int smmu_populate_regarray(SMMUv2State *s,
                                  RegisterInfoArray *r_array, int pos,
                                  const RegisterAccessInfo *rae,
                                  int num_rae)
{
    int i;

    for (i = 0; i < num_rae; i++) {
        int index = rae[i].addr / 4;
        RegisterInfo *r = &s->regs_info[index];

        *r = (RegisterInfo) {
            .data = &s->regs[index],
            .data_size = sizeof(uint32_t),
            .access = &rae[i],
            .opaque = OBJECT(s),
        };
        register_init(r);

        r_array->r[i + pos] = r;
    }

    return i + pos;
}

static void smmu_create_rai_smr(SMMUv2State *s)
{
    int i;

    s->rai_smr = g_new0(RegisterAccessInfo, s->cfg.num_smr * 2);
    for (i = 0; i < s->cfg.num_smr; i++) {
        s->rai_smr[i * 2].name = g_strdup_printf("SMMU_SMR%d", i);
        s->rai_smr[i * 2].addr = A_SMMU_SMR0 + i * 4;
        s->rai_smr[i * 2 + 1].name = g_strdup_printf("SMMU_S2CR%d", i);
        s->rai_smr[i * 2 + 1].addr = A_SMMU_S2CR0 + i * 4;
        s->rai_smr[i * 2 + 1].post_write = smmuv2_notify_config_change;
    }
}

static void smmu_create_rai_cb(SMMUv2State *s)
{
    int cb;

    s->rai_cb = g_new0(RegisterAccessInfo, s->cfg.num_cb * NUM_REGS_PER_CB);

    for (cb = 0; cb < s->cfg.num_cb; cb++) {
        int pos = cb * NUM_REGS_PER_CB;
        int i;

        memcpy(s->rai_cb + pos, smmu_cb_regs_info, sizeof smmu_cb_regs_info);
        for (i = 0; i < ARRAY_SIZE(smmu_cb_regs_info); i++) {
            const char *name = smmu_cb_regs_info[i].name;
            uint64_t addr = smmu_cb_regs_info[i].addr;

            s->rai_cb[i + pos].name = g_strdup_printf("SMMU_CB%s%d", name, cb);
            s->rai_cb[i + pos].addr = addr + cb * 4;
        }
        pos += 3;

        /* Initialize with CB template.  */
        memcpy(s->rai_cb + pos, smmu_cb_page_regs_info,
               sizeof smmu_cb_page_regs_info);

        /* Generate context specific names.  */
        for (i = 0; i < ARRAY_SIZE(smmu_cb_page_regs_info); i++) {
            const char *name = smmu_cb_page_regs_info[i].name;
            uint64_t addr = smmu_cb_page_regs_info[i].addr;
            unsigned int cb_base = smmu_cb_offset(s, cb) * 4;

            s->rai_cb[i + pos].name = g_strdup_printf("SMMU_CB%d_%s",
                                                      cb, name);
            s->rai_cb[i + pos].addr = cb_base + addr;
        }
    }
}

static void smmu_create_regarray(SMMUv2State *s)
{
    SMMUState *sys = ARM_SMMU(s);
    const char *device_prefix = object_get_typename(OBJECT(s));
    uint64_t memory_size = R_MAX * 4;
    RegisterInfoArray *r_array;
    int num_regs;
    int pos = 0;

    if (s->cfg.num_smr < 1 || s->cfg.num_smr > 127) {
        error_report("num-smr %d must be between 1 - 127", s->cfg.num_smr);
        exit(EXIT_FAILURE);
    }

    if (s->cfg.num_cb < 4 || s->cfg.num_cb > MAX_CB
        || s->cfg.num_cb > s->cfg.num_pages) {
        error_report("num-cb %d must be between 4 - 128 and <= num-pages %d",
                      s->cfg.num_cb, s->cfg.num_pages);
        exit(EXIT_FAILURE);
    }

    if (s->cfg.num_pages < 4 || !is_power_of_2(s->cfg.num_pages)) {
        error_report("num-pages %d must be > 4 and a power of 2",
                     s->cfg.num_pages);
        exit(EXIT_FAILURE);
    }

    smmu_create_rai_smr(s);
    smmu_create_rai_cb(s);

    num_regs = ARRAY_SIZE(smmu500_regs_info);
    num_regs += s->cfg.num_smr * 2;
    num_regs += s->cfg.num_cb * NUM_REGS_PER_CB;

    r_array = g_new0(RegisterInfoArray, 1);
    r_array->r = g_new0(RegisterInfo *, num_regs);
    r_array->num_elements = num_regs;
    r_array->debug = SMMUV2_ERR_DEBUG;
    r_array->prefix = device_prefix;

    pos = smmu_populate_regarray(s, r_array, pos,
                                 smmu500_regs_info,
                                 ARRAY_SIZE(smmu500_regs_info));

    pos = smmu_populate_regarray(s, r_array, pos,
                                 s->rai_smr, s->cfg.num_smr * 2);

    pos = smmu_populate_regarray(s, r_array, pos,
                                 s->rai_cb, s->cfg.num_cb * NUM_REGS_PER_CB);

    memory_region_init_io(&sys->iomem, OBJECT(s), &smmu_mem_ops, r_array,
                          device_prefix, memory_size);
}

static void smmu_init_irq(SMMUv2State *s, SysBusDevice *dev)
{
    int i;

    sysbus_init_irq(dev, &s->irq.global);
    for (i = 0; i < s->cfg.num_cb; i++) {
        sysbus_init_irq(dev, &s->irq.context[i]);
    }
}

static void smmu_realize(DeviceState *d, Error **errp)
{
    SMMUState *sys = ARM_SMMU(d);
    SMMUv2State *s = ARM_SMMUV2(sys);
    SMMUv2Class *c = ARM_SMMUV2_GET_CLASS(s);
    SysBusDevice *dev = SYS_BUS_DEVICE(d);
    Error *local_err = NULL;
    int i;

    c->parent_realize(d, &local_err);
    if (local_err) {
        error_propagate(errp, local_err);
        return;
    }

    qemu_mutex_init(&s->mutex);

    for (i = 0; i < s->cfg.num_cb; i++)
        s->cb_to_s2cr[i] = ~0;

    smmu_create_regarray(s);

    sys->mrtypename = TYPE_SMMUV2_IOMMU_MEMORY_REGION;

    sysbus_init_mmio(dev, &sys->iomem);

    smmu_init_irq(s, dev);
}

static Property smmu_properties[] = {
    DEFINE_PROP_UINT32("pamax", SMMUv2State, cfg.pamax, 48),
    DEFINE_PROP_UINT16("num-smr", SMMUv2State, cfg.num_smr, 48),
    DEFINE_PROP_UINT16("num-cb", SMMUv2State, cfg.num_cb, 16),
    DEFINE_PROP_UINT16("num-pages", SMMUv2State, cfg.num_pages, 16),
    DEFINE_PROP_END_OF_LIST(),
};

static const VMStateDescription vmstate_smmuv2 = {
    .name = "smmuv2",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
        VMSTATE_END_OF_LIST(),
    },
};

static void smmuv2_instance_init(Object *obj)
{
    /* Nothing much to do here as of now */
}

static void smmuv2_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    SMMUv2Class *c = ARM_SMMUV2_CLASS(klass);

    dc->vmsd = &vmstate_smmuv2;
    device_class_set_props(dc, smmu_properties);
    device_class_set_parent_reset(dc, smmu_reset, &c->parent_reset);
    c->parent_realize = dc->realize;
    dc->realize = smmu_realize;
}

static int smmuv2_notify_flag_changed(IOMMUMemoryRegion *iommu,
                                      IOMMUNotifierFlag old,
                                      IOMMUNotifierFlag new,
                                      Error **errp)
{
    SMMUDevice *sdev = container_of(iommu, SMMUDevice, iommu);
    SMMUv2State *s2 = sdev->smmu;
    SMMUState *s = &(s2->smmu_state);

    if (old == IOMMU_NOTIFIER_NONE) {
//        trace_smmuv3_notify_flag_add(iommu->parent_obj.name);
        QLIST_INSERT_HEAD(&s->devices_with_notifiers, sdev, next);
    } else if (new == IOMMU_NOTIFIER_NONE) {
//        trace_smmuv3_notify_flag_del(iommu->parent_obj.name);
        QLIST_REMOVE(sdev, next);
    }
    return 0;
}

static int smmuv2_get_attr(IOMMUMemoryRegion *iommu,
                           enum IOMMUMemoryRegionAttr attr,
                           void *data)
{
    if (attr == IOMMU_ATTR_VFIO_NESTED) {
        *(bool *) data = true;
        return 0;
    } else if (attr == IOMMU_ATTR_MSI_TRANSLATE) {
        *(bool *) data = true;
        return 0;
    }
    return -EINVAL;
}

struct iommu_fault;

static inline int
smmuv2_inject_faults(IOMMUMemoryRegion *iommu_mr, int count,
                     struct iommu_fault *buf)
{
#ifdef __linux__
    SMMUDevice *sdev = container_of(iommu_mr, SMMUDevice, iommu);
    SMMUv2State *s = sdev->smmu;
    uint32_t sid = smmu_get_sid(sdev);
    int i;

    for (i = 0; i < count; i++) {
        struct iommu_fault_unrecoverable *record;
        SMMUv2Status status = {};

        if (buf[i].type != IOMMU_FAULT_DMA_UNRECOV) {
            continue;
        }

        status.sid = sid;
        record = &buf[i].event;

        switch (record->reason) {
        case IOMMU_FAULT_REASON_PASID_INVALID:
            status.type = SMMU_FAULT_GLOBAL_INVAL_CTX;
            break;
        case IOMMU_FAULT_REASON_PASID_FETCH:
            status.type = SMMU_FAULT_GLOBAL_INVAL_CTX;
            break;
        case IOMMU_FAULT_REASON_BAD_PASID_ENTRY:
            status.type = SMMU_FAULT_GLOBAL_INVAL_CTX;
            break;
        case IOMMU_FAULT_REASON_WALK_EABT:
            status.type = SMMU_FAULT_CTX_TRANS;
            status.addr = record->addr;
            break;
        case IOMMU_FAULT_REASON_PTE_FETCH:
            status.type = SMMU_FAULT_CTX_TRANS;
            status.addr = record->addr;
            break;
        case IOMMU_FAULT_REASON_OOR_ADDRESS:
            status.type = SMMU_FAULT_CTX_TRANS;
            status.addr = record->addr;
            break;
        case IOMMU_FAULT_REASON_ACCESS:
            status.type = SMMU_FAULT_CTX_PERM;
            status.addr = record->addr;
            break;
        case IOMMU_FAULT_REASON_PERMISSION:
            status.type = SMMU_FAULT_CTX_PERM;
            status.addr = record->addr;
            break;
        default:
            warn_report("%s Unexpected fault reason received from host: %d",
                        __func__, record->reason);
            continue;
        }

        smmu_fault(s, status.cb, status.addr);
    }
    return 0;
#else
    return -1;
#endif
}

static void smmuv2_iommu_memory_region_class_init(ObjectClass *klass,
                                                  void *data)
{
    IOMMUMemoryRegionClass *imrc = IOMMU_MEMORY_REGION_CLASS(klass);

    imrc->translate = smmuv2_translate;
    imrc->notify_flag_changed = smmuv2_notify_flag_changed;
    imrc->get_attr = smmuv2_get_attr;
    imrc->inject_faults = smmuv2_inject_faults;
}

static const TypeInfo smmuv2_type_info = {
    .name          = TYPE_ARM_SMMUV2,
    .parent        = TYPE_ARM_SMMU,
    .instance_size = sizeof(SMMUv2State),
    .instance_init = smmuv2_instance_init,
    .class_size    = sizeof(SMMUv2Class),
    .class_init    = smmuv2_class_init,
};

static const TypeInfo smmuv2_iommu_memory_region_info = {
    .parent = TYPE_IOMMU_MEMORY_REGION,
    .name = TYPE_SMMUV2_IOMMU_MEMORY_REGION,
    .class_init = smmuv2_iommu_memory_region_class_init,
};

static void smmuv2_register_types(void)
{
    type_register(&smmuv2_type_info);
    type_register(&smmuv2_iommu_memory_region_info);
}

type_init(smmuv2_register_types)

