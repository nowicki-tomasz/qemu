/* SPDX-License-Identifier: BSD-3-Clause */
/*
 * Virtio-iommu definition v0.7
 *
 * Copyright (C) 2018 Arm Ltd.
 */
#ifndef _LINUX_VIRTIO_IOMMU_H
#define _LINUX_VIRTIO_IOMMU_H

#include "standard-headers/linux/types.h"

/* Feature bits */
#define VIRTIO_IOMMU_F_INPUT_RANGE		0
#define VIRTIO_IOMMU_F_DOMAIN_BITS		1
#define VIRTIO_IOMMU_F_MAP_UNMAP		2
#define VIRTIO_IOMMU_F_BYPASS			3
#define VIRTIO_IOMMU_F_PROBE			4
#define VIRTIO_IOMMU_F_ATTACH_TABLE		5

struct virtio_iommu_config {
	/* Supported page sizes */
	uint64_t					page_size_mask;
	/* Supported IOVA range */
	struct virtio_iommu_range {
		uint64_t				start;
		uint64_t				end;
	} input_range;
	/* Max domain ID size */
	uint8_t					domain_bits;
	uint8_t					padding[3];
	/* Probe buffer size */
	uint32_t					probe_size;
} QEMU_PACKED;

/* Request types */
#define VIRTIO_IOMMU_T_ATTACH			0x01
#define VIRTIO_IOMMU_T_DETACH			0x02
#define VIRTIO_IOMMU_T_MAP			0x03
#define VIRTIO_IOMMU_T_UNMAP			0x04
#define VIRTIO_IOMMU_T_PROBE			0x05
#define VIRTIO_IOMMU_T_ATTACH_TABLE		0x06
#define VIRTIO_IOMMU_T_INVALIDATE		0x07

/* Status types */
#define VIRTIO_IOMMU_S_OK			0x00
#define VIRTIO_IOMMU_S_IOERR			0x01
#define VIRTIO_IOMMU_S_UNSUPP			0x02
#define VIRTIO_IOMMU_S_DEVERR			0x03
#define VIRTIO_IOMMU_S_INVAL			0x04
#define VIRTIO_IOMMU_S_RANGE			0x05
#define VIRTIO_IOMMU_S_NOENT			0x06
#define VIRTIO_IOMMU_S_FAULT			0x07

struct virtio_iommu_req_head {
	uint8_t					type;
	uint8_t					reserved[3];
} QEMU_PACKED;

struct virtio_iommu_req_tail {
	uint8_t					status;
	uint8_t					reserved[3];
} QEMU_PACKED;

/* Arm LPAE Page Table Descriptor */
struct virtio_iommu_pgt_arm {
	/*
	 * Note that only VA64 format of SMMUv2 is supported at the moment.
	 * PAN, ENDI etc. are not present. SMMUv3 users are expected to use
	 * virtio_iommu_pst_arm tables.
	 */
#define VIRTIO_IOMMU_PGTF_ARM_SEP_SHIFT		47
#define VIRTIO_IOMMU_PGTF_ARM_SEP_MASK		0x3ULL
#define VIRTIO_IOMMU_PGTF_ARM_HD		(1ULL << 43)
#define VIRTIO_IOMMU_PGTF_ARM_HA		(1ULL << 42)
#define VIRTIO_IOMMU_PGTF_ARM_HAD1		(1ULL << 41)
#define VIRTIO_IOMMU_PGTF_ARM_HAD0		(1ULL << 40)
#define VIRTIO_IOMMU_PGTF_ARM_TBI1		(1ULL << 38)
#define VIRTIO_IOMMU_PGTF_ARM_TBI0		(1ULL << 37)
	/* ASID size */
#define VIRTIO_IOMMU_PGTF_ARM_AS		(1ULL << 36)
#define VIRTIO_IOMMU_PGTF_ARM_PASIZE_SHIFT	32
#define VIRTIO_IOMMU_PGTF_ARM_PASIZE_MASK	0x7ULL
#define VIRTIO_IOMMU_PGTF_ARM_TG1_SHIFT		30
#define VIRTIO_IOMMU_PGTF_ARM_TG1_MASK		0x3
#define VIRTIO_IOMMU_PGTF_ARM_SH1_SHIFT		28
#define VIRTIO_IOMMU_PGTF_ARM_SH1_MASK		0x3
#define VIRTIO_IOMMU_PGTF_ARM_ORGN1_SHIFT	26
#define VIRTIO_IOMMU_PGTF_ARM_ORGN1_MASK	0x3
#define VIRTIO_IOMMU_PGTF_ARM_IRGN1_SHIFT	24
#define VIRTIO_IOMMU_PGTF_ARM_IRGN1_MASK	0x3
#define VIRTIO_IOMMU_PGTF_ARM_EPD1		(1 << 23)
#define VIRTIO_IOMMU_PGTF_ARM_A1		(1 << 22)
#define VIRTIO_IOMMU_PGTF_ARM_T1SZ_SHIFT	16
#define VIRTIO_IOMMU_PGTF_ARM_T1SZ_MASK		0x3f
#define VIRTIO_IOMMU_PGTF_ARM_TG0_SHIFT		14
#define VIRTIO_IOMMU_PGTF_ARM_TG0_MASK		0x3
#define VIRTIO_IOMMU_PGTF_ARM_SH0_SHIFT		12
#define VIRTIO_IOMMU_PGTF_ARM_SH0_MASK		0x3
#define VIRTIO_IOMMU_PGTF_ARM_ORGN0_SHIFT	10
#define VIRTIO_IOMMU_PGTF_ARM_ORGN0_MASK	0x3
#define VIRTIO_IOMMU_PGTF_ARM_IRGN0_SHIFT	8
#define VIRTIO_IOMMU_PGTF_ARM_IRGN0_MASK	0x3
#define VIRTIO_IOMMU_PGTF_ARM_EPD0		(1 << 7)
#define VIRTIO_IOMMU_PGTF_ARM_T0SZ_SHIFT	0
#define VIRTIO_IOMMU_PGTF_ARM_T0SZ_MASK		0x3f
	uint64_t					tcr;
	/* If EPD1 is 0. TTBR0 is the base pointer in virtio_iommu_pgt */
	uint64_t					ttbr1;
	/* MAIR0[31:0], MAIR1[63:32] */
	uint64_t					mair;
	uint16_t					asid;
	uint8_t					reserved[6];
} QEMU_PACKED;

struct virtio_iommu_pgt {
	uint64_t					base;

	uint8_t					format;
	uint8_t					reserved[3];
	union {
		struct virtio_iommu_pgt_arm	arm;
	};
} QEMU_PACKED;

/* Arm SMMUv3 PASID Table Descriptor */
struct virtio_iommu_pst_arm {
#define VIRTIO_IOMMU_PSTF_ARM_SV3_LINEAR	0x0
#define VIRTIO_IOMMU_PSTF_ARM_SV3_4KL2		0x1
#define VIRTIO_IOMMU_PSTF_ARM_SV3_64KL2		0x2
	uint8_t					s1fmt;
#define VIRTIO_IOMMU_PSTF_ARM_SV3_DSS_TERM	0x0
#define VIRTIO_IOMMU_PSTF_ARM_SV3_DSS_BYPASS	0x1
#define VIRTIO_IOMMU_PSTF_ARM_SV3_DSS_0		0x2
	uint8_t					s1dss;
} QEMU_PACKED;

struct virtio_iommu_pst {
	uint64_t					base;
	uint32_t					size;

	uint8_t					format;
	uint8_t					reserved[3];
	union {
		struct virtio_iommu_pst_arm	arm;
	};
} QEMU_PACKED;

#define VIRTIO_IOMMU_ATTACH_F_PST	(1 << 0)
#define VIRTIO_IOMMU_ATTACH_F_PGT	(1 << 1)

struct virtio_iommu_req_attach {
	struct virtio_iommu_req_head		head;

	uint32_t					domain;
	uint32_t					endpoint;
	uint32_t					reserved;

	struct virtio_iommu_req_tail		tail;
} QEMU_PACKED;

struct virtio_iommu_req_attach_table {
	struct virtio_iommu_req_head		head;

	uint32_t					domain;
	uint32_t					endpoint;
	uint32_t					flags;

	union {
		struct virtio_iommu_pst		pst;
		struct virtio_iommu_pgt		pgt;
		uint8_t				padding[64];
	};

	struct virtio_iommu_req_tail		tail;
} QEMU_PACKED;

struct virtio_iommu_req_detach {
	struct virtio_iommu_req_head		head;

	uint32_t					endpoint;
	uint32_t					reserved;

	struct virtio_iommu_req_tail		tail;
} QEMU_PACKED;

#define VIRTIO_IOMMU_MAP_F_READ			(1 << 0)
#define VIRTIO_IOMMU_MAP_F_WRITE		(1 << 1)
#define VIRTIO_IOMMU_MAP_F_EXEC			(1 << 2)
#define VIRTIO_IOMMU_MAP_F_MMIO			(1 << 3)

#define VIRTIO_IOMMU_MAP_F_MASK			(VIRTIO_IOMMU_MAP_F_READ |	\
						 VIRTIO_IOMMU_MAP_F_WRITE |	\
						 VIRTIO_IOMMU_MAP_F_EXEC |	\
						 VIRTIO_IOMMU_MAP_F_MMIO)

struct virtio_iommu_req_map {
	struct virtio_iommu_req_head		head;

	uint32_t					domain;
	uint64_t					virt_start;
	uint64_t					virt_end;
	uint64_t					phys_start;
	uint32_t					flags;

	struct virtio_iommu_req_tail		tail;
} QEMU_PACKED;

struct virtio_iommu_req_unmap {
	struct virtio_iommu_req_head		head;

	uint32_t					domain;
	uint64_t					virt_start;
	uint64_t					virt_end;
	uint32_t					reserved;

	struct virtio_iommu_req_tail		tail;
} QEMU_PACKED;

#define VIRTIO_IOMMU_RESV_MEM_T_RESERVED	0
#define VIRTIO_IOMMU_RESV_MEM_T_MSI		1

struct virtio_iommu_probe_resv_mem {
	uint8_t					subtype;
	uint8_t					reserved[3];
	uint64_t					start;
	uint64_t					end;
} QEMU_PACKED;

struct virtio_iommu_probe_page_size_mask {
	__u32					reserved;
	uint64_t					mask;
} QEMU_PACKED;

struct virtio_iommu_probe_input_range {
	__u32					reserved;
	uint64_t					start;
	uint64_t					end;
} QEMU_PACKED;

struct virtio_iommu_probe_output_size {
	uint8_t					reserved[3];
	uint8_t					bits;
} QEMU_PACKED;

struct virtio_iommu_probe_pasid_size {
	uint8_t					reserved[3];
	uint8_t					bits;
} QEMU_PACKED;

/* Arm LPAE Page Table Format */
struct virtio_iommu_probe_pgtf_arm {
#define VIRTIO_IOMMU_PGTF_ARM_LPAE		1
	uint8_t					format;
	uint8_t					reserved[3];

	/* Bitmap reserved for future use */
	uint64_t					quirks;
} QEMU_PACKED;

/* Arm SMMUv3 PASID Table Format */
struct virtio_iommu_probe_pstf_arm {
#define VIRTIO_IOMMU_PSTF_ARM_SV3		1
	uint8_t					format;
	uint8_t					reserved[3];

	/* Info needed for populating the table */
#define VIRTIO_IOMMU_PSTF_ARM_SV3_F_STALL	(1ULL << 0)
#define VIRTIO_IOMMU_PSTF_ARM_SV3_F_STALL_FORCE	(1ULL << 1)
#define VIRTIO_IOMMU_PSTF_ARM_SV3_F_HW_DIRTY	(1ULL << 2)
#define VIRTIO_IOMMU_PSTF_ARM_SV3_F_HW_ACCESS	(1ULL << 3)
#define VIRTIO_IOMMU_PSTF_ARM_SV3_F_ASID16	(1ULL << 4)
#define VIRTIO_IOMMU_PSTF_ARM_SV3_F_BTM		(1ULL << 5)
	uint64_t					flags;
} QEMU_PACKED;

#define VIRTIO_IOMMU_PROBE_T_NONE		0
#define VIRTIO_IOMMU_PROBE_T_RESV_MEM		1
#define VIRTIO_IOMMU_PROBE_T_PAGE_SIZE_MASK	2
#define VIRTIO_IOMMU_PROBE_T_INPUT_RANGE	3
#define VIRTIO_IOMMU_PROBE_T_OUTPUT_SIZE	4
#define VIRTIO_IOMMU_PROBE_T_PASID_SIZE		5
#define VIRTIO_IOMMU_PROBE_T_PAGE_TABLE_FMT	6
#define VIRTIO_IOMMU_PROBE_T_PASID_TABLE_FMT	7

#define VIRTIO_IOMMU_PROBE_T_MASK		0xfff

struct virtio_iommu_probe_property {
	uint16_t					type;
	uint16_t					length;
	uint8_t					value[];
} QEMU_PACKED;

struct virtio_iommu_req_probe {
	struct virtio_iommu_req_head		head;
	uint32_t					endpoint;
	uint8_t					reserved[64];

	uint8_t					properties[];

	/* Tail follows the variable-length properties array (no padding) */
} QEMU_PACKED;

#define VIRTIO_IOMMU_INVAL_S_DOMAIN		(1 << 0)
/* 'pasid', 'tag' are valid */
#define VIRTIO_IOMMU_INVAL_S_PASID		(1 << 1)
/* 'pasid', 'tag', 'virt_start', 'nr_pages' and 'granule' are valid */
#define VIRTIO_IOMMU_INVAL_S_VA			(1 << 2)

#define VIRTIO_IOMMU_INVAL_F_CONFIG		(1 << 0)
#define VIRTIO_IOMMU_INVAL_F_LEAF		(1 << 1)

struct virtio_iommu_req_invalidate {
	struct virtio_iommu_req_head		head;
	uint32_t					scope;
	uint32_t					flags;

	uint32_t					domain;
	uint32_t					pasid;
	uint64_t					virt_start;
	uint64_t					nr_pages;
	/* Page size, in nr of bits, typically 12 for 4k, 30 for 2MB, etc.) */
	uint8_t					granule;
	uint8_t					reserved[3];

	/* Arch-specific field */
	uint64_t					tag;
	/* TODO: padding */

	struct virtio_iommu_req_tail		tail;
} QEMU_PACKED;

union virtio_iommu_req {
	struct virtio_iommu_req_head		head;

	struct virtio_iommu_req_attach		attach;
	struct virtio_iommu_req_attach_table	attach_table;
	struct virtio_iommu_req_detach		detach;
	struct virtio_iommu_req_map		map;
	struct virtio_iommu_req_unmap		unmap;
	struct virtio_iommu_req_probe		probe;
	struct virtio_iommu_req_invalidate	inval;
};

/* Fault types */
#define VIRTIO_IOMMU_FAULT_R_UNKNOWN		0
#define VIRTIO_IOMMU_FAULT_R_DOMAIN		1
#define VIRTIO_IOMMU_FAULT_R_MAPPING		2

#define VIRTIO_IOMMU_FAULT_F_READ		(1 << 0)
#define VIRTIO_IOMMU_FAULT_F_WRITE		(1 << 1)
#define VIRTIO_IOMMU_FAULT_F_EXEC		(1 << 2)
#define VIRTIO_IOMMU_FAULT_F_ADDRESS		(1 << 8)

struct virtio_iommu_fault {
	uint8_t					reason;
	uint8_t					padding[3];
	uint32_t					flags;
	uint32_t					endpoint;
	uint64_t					address;
} QEMU_PACKED;

#endif
