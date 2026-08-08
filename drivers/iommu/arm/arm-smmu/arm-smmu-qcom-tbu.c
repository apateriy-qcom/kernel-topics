// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#include <linux/interconnect.h>
#include <linux/iopoll.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/spinlock.h>
#include <linux/qcom-iommu-util.h>

#include "arm-smmu.h"
#include "arm-smmu-qcom.h"
#include "../../qcom-iommu-debug.h"

#define TBU_DBG_TIMEOUT_US		100
#define DEBUG_AXUSER_REG		0x30
#define DEBUG_AXUSER_CDMID		GENMASK_ULL(43, 36)
#define DEBUG_AXUSER_CDMID_VAL		0xff
#define DEBUG_PAR_REG			0x28
#define DEBUG_PAR_FAULT_VAL		BIT(0)
#define DEBUG_PAR_PA			GENMASK_ULL(47, 12)
#define DEBUG_SID_HALT_REG		0x0
#define DEBUG_SID_HALT_VAL		BIT(16)
#define DEBUG_SID_HALT_SID		GENMASK(9, 0)
#define DEBUG_SR_HALT_ACK_REG		0x20
#define DEBUG_SR_HALT_ACK_VAL		BIT(1)
#define DEBUG_SR_ECATS_RUNNING_VAL	BIT(0)
#define DEBUG_TXN_AXCACHE		GENMASK(5, 2)
#define DEBUG_TXN_AXPROT		GENMASK(8, 6)
#define DEBUG_TXN_AXPROT_PRIV		0x1
#define DEBUG_TXN_AXPROT_NSEC		0x2
#define DEBUG_TXN_TRIGG_REG		0x18
#define DEBUG_TXN_TRIGGER		BIT(0)
#define DEBUG_VA_ADDR_REG		0x8

/* QTB constants */
#define QTB_DBG_TIMEOUT_US		100

#define QTB_SWID_LOW			0x0

#define QTB_OVR_DBG_FENCEREQ		0x410
#define QTB_OVR_DBG_FENCEREQ_HALT	BIT(0)

#define QTB_OVR_DBG_FENCEACK		0x418
#define QTB_OVR_DBG_FENCEACK_ACK	BIT(0)

#define QTB_OVR_ECATS_INFLD0			0x430
#define QTB_OVR_ECATS_INFLD0_PCIE_NO_SNOOP	BIT(21)
#define QTB_OVR_ECATS_INFLD0_SEC_SID		BIT(20)
#define QTB_OVR_ECATS_INFLD0_QAD		GENMASK(19, 16)
#define QTB_OVR_ECATS_INFLD0_SID		GENMASK(9, 0)

#define QTB_OVR_ECATS_INFLD1		0x438
#define QTB_OVR_ECATS_INFLD1_PNU	BIT(13)
#define QTB_OVR_ECATS_INFLD1_IND	BIT(12)
#define QTB_OVR_ECATS_INFLD1_DIRTY	BIT(11)
#define QTB_OVR_ECATS_INFLD1_TR_TYPE	GENMASK(10, 8)
#define QTB_OVR_ECATS_INFLD1_TR_TYPE_SHARED 4
#define QTB_OVR_ECATS_INFLD1_ALLOC	GENMASK(7, 4)
#define QTB_OVR_ECATS_INFLD1_NON_SEC	BIT(3)
#define QTB_OVR_ECATS_INFLD1_OPC	GENMASK(2, 0)
#define QTB_OVR_ECATS_INFLD1_OPC_WRI	1

#define QTB_OVR_ECATS_INFLD2	0x440

#define QTB_OVR_ECATS_TRIGGER		0x448
#define QTB_OVR_ECATS_TRIGGER_START	BIT(0)

#define QTB_OVR_ECATS_STATUS		0x450
#define QTB_OVR_ECATS_STATUS_DONE	BIT(0)

#define QTB_OVR_ECATS_OUTFLD0			0x458
#define QTB_OVR_ECATS_OUTFLD0_PA		GENMASK_ULL(63, 12)
#define QTB_OVR_ECATS_OUTFLD0_FAULT_TYPE	GENMASK(5, 4)
#define QTB_OVR_ECATS_OUTFLD0_FAULT		BIT(0)

#define QTB_NS_DBG_PORT_N_OT_SNAPSHOT(port_num)	(0xc10 + (0x10 * port_num))

struct qsmmuv500_tbu {
	struct device *dev;
	struct arm_smmu_device *smmu;
	u32 sid_range[2];
	struct list_head list;
	struct clk *clk;
	struct icc_path	*path;
	void __iomem *base;
	spinlock_t halt_lock; /* multiple halt or resume can't execute concurrently */
	int halt_count;
	const struct qsmmuv500_tbu_impl	*impl;
};

struct qtb500_device {
	struct qsmmuv500_tbu tbu;
	bool no_halt;
	u32 num_ports;
	void __iomem			*debugchain_base;
	void __iomem			*transactiontracker_base;
};

#define to_qtb500(tbu)		container_of(tbu, struct qtb500_device, tbu)

struct qsmmuv500_tbu_impl {
	int (*halt_req)(struct qsmmuv500_tbu *tbu);
	int (*halt_poll)(struct qsmmuv500_tbu *tbu);
	void (*resume)(struct qsmmuv500_tbu *tbu);
	phys_addr_t (*trigger_atos)(struct qsmmuv500_tbu *tbu, dma_addr_t iova, u32 sid,
				    unsigned long trans_flags);
	void (*write_sync)(struct qsmmuv500_tbu *tbu);
	void (*log_outstanding_transactions)(struct qsmmuv500_tbu *tbu);
};

static DEFINE_SPINLOCK(atos_lock);

static struct qcom_smmu *to_qcom_smmu(struct arm_smmu_device *smmu)
{
	return container_of(smmu, struct qcom_smmu, smmu);
}

static struct arm_smmu_domain *to_smmu_domain(struct iommu_domain *dom)
{
	return container_of(dom, struct arm_smmu_domain, domain);
}

static struct qsmmuv500_tbu *qsmmuv500_find_tbu(struct qcom_smmu *qsmmu, u32 sid)
{
	struct qsmmuv500_tbu *tbu = NULL;
	u32 start, end;

	mutex_lock(&qsmmu->tbu_list_lock);

	list_for_each_entry(tbu, &qsmmu->tbu_list, list) {
		start = tbu->sid_range[0];
		end = start + tbu->sid_range[1];

		if (start <= sid && sid < end)
			break;
	}

	mutex_unlock(&qsmmu->tbu_list_lock);

	return tbu;
}

static int qsmmuv500_tbu_halt(struct qsmmuv500_tbu *tbu, struct arm_smmu_domain *smmu_domain)
{
	struct arm_smmu_device *smmu = smmu_domain->smmu;
	int ret = 0, idx = smmu_domain->cfg.cbndx;
	unsigned long flags;
	u32 val, fsr, status;

	spin_lock_irqsave(&tbu->halt_lock, flags);
	if (tbu->halt_count) {
		tbu->halt_count++;
		goto out;
	}

	val = readl_relaxed(tbu->base + DEBUG_SID_HALT_REG);
	val |= DEBUG_SID_HALT_VAL;
	writel_relaxed(val, tbu->base + DEBUG_SID_HALT_REG);

	fsr = arm_smmu_cb_read(smmu, idx, ARM_SMMU_CB_FSR);
	if ((fsr & ARM_SMMU_CB_FSR_FAULT) && (fsr & ARM_SMMU_CB_FSR_SS)) {
		u32 sctlr_orig, sctlr;

		/*
		 * We are in a fault. Our request to halt the bus will not
		 * complete until transactions in front of us (such as the fault
		 * itself) have completed. Disable iommu faults and terminate
		 * any existing transactions.
		 */
		sctlr_orig = arm_smmu_cb_read(smmu, idx, ARM_SMMU_CB_SCTLR);
		sctlr = sctlr_orig & ~(ARM_SMMU_SCTLR_CFCFG | ARM_SMMU_SCTLR_CFIE);
		arm_smmu_cb_write(smmu, idx, ARM_SMMU_CB_SCTLR, sctlr);
		arm_smmu_cb_write(smmu, idx, ARM_SMMU_CB_FSR, fsr);
		arm_smmu_cb_write(smmu, idx, ARM_SMMU_CB_RESUME, ARM_SMMU_RESUME_TERMINATE);
		arm_smmu_cb_write(smmu, idx, ARM_SMMU_CB_SCTLR, sctlr_orig);
	}

	if (readl_poll_timeout_atomic(tbu->base + DEBUG_SR_HALT_ACK_REG, status,
				      (status & DEBUG_SR_HALT_ACK_VAL),
				      0, TBU_DBG_TIMEOUT_US)) {
		dev_err(tbu->dev, "Timeout while trying to halt TBU!\n");
		ret = -ETIMEDOUT;

		val = readl_relaxed(tbu->base + DEBUG_SID_HALT_REG);
		val &= ~DEBUG_SID_HALT_VAL;
		writel_relaxed(val, tbu->base + DEBUG_SID_HALT_REG);

		goto out;
	}

	tbu->halt_count = 1;

out:
	spin_unlock_irqrestore(&tbu->halt_lock, flags);
	return ret;
}

static void qsmmuv500_tbu_resume(struct qsmmuv500_tbu *tbu)
{
	unsigned long flags;
	u32 val;

	spin_lock_irqsave(&tbu->halt_lock, flags);
	if (!tbu->halt_count) {
		WARN(1, "%s: halt_count is 0", dev_name(tbu->dev));
		goto out;
	}

	if (tbu->halt_count > 1) {
		tbu->halt_count--;
		goto out;
	}

	val = readl_relaxed(tbu->base + DEBUG_SID_HALT_REG);
	val &= ~DEBUG_SID_HALT_VAL;
	writel_relaxed(val, tbu->base + DEBUG_SID_HALT_REG);

	tbu->halt_count = 0;
out:
	spin_unlock_irqrestore(&tbu->halt_lock, flags);
}

static phys_addr_t qsmmuv500_tbu_trigger_atos(struct arm_smmu_domain *smmu_domain,
					      struct qsmmuv500_tbu *tbu, dma_addr_t iova, u32 sid)
{
	bool atos_timedout = false;
	phys_addr_t phys = 0;
	ktime_t timeout;
	u64 val;

	/* Set address and stream-id */
	val = readq_relaxed(tbu->base + DEBUG_SID_HALT_REG);
	val &= ~DEBUG_SID_HALT_SID;
	val |= FIELD_PREP(DEBUG_SID_HALT_SID, sid);
	writeq_relaxed(val, tbu->base + DEBUG_SID_HALT_REG);
	writeq_relaxed(iova, tbu->base + DEBUG_VA_ADDR_REG);
	val = FIELD_PREP(DEBUG_AXUSER_CDMID, DEBUG_AXUSER_CDMID_VAL);
	writeq_relaxed(val, tbu->base + DEBUG_AXUSER_REG);

	/* Write-back read and write-allocate */
	val = FIELD_PREP(DEBUG_TXN_AXCACHE, 0xf);

	/* Non-secure access */
	val |= FIELD_PREP(DEBUG_TXN_AXPROT, DEBUG_TXN_AXPROT_NSEC);

	/* Priviledged access */
	val |= FIELD_PREP(DEBUG_TXN_AXPROT, DEBUG_TXN_AXPROT_PRIV);

	val |= DEBUG_TXN_TRIGGER;
	writeq_relaxed(val, tbu->base + DEBUG_TXN_TRIGG_REG);

	timeout = ktime_add_us(ktime_get(), TBU_DBG_TIMEOUT_US);
	for (;;) {
		val = readl_relaxed(tbu->base + DEBUG_SR_HALT_ACK_REG);
		if (!(val & DEBUG_SR_ECATS_RUNNING_VAL))
			break;
		val = readl_relaxed(tbu->base + DEBUG_PAR_REG);
		if (val & DEBUG_PAR_FAULT_VAL)
			break;
		if (ktime_compare(ktime_get(), timeout) > 0) {
			atos_timedout = true;
			break;
		}
	}

	val = readq_relaxed(tbu->base + DEBUG_PAR_REG);
	if (val & DEBUG_PAR_FAULT_VAL)
		dev_err(tbu->dev, "ATOS generated a fault interrupt! PAR = %llx, SID=0x%x\n",
			val, sid);
	else if (atos_timedout)
		dev_err_ratelimited(tbu->dev, "ATOS translation timed out!\n");
	else
		phys = FIELD_GET(DEBUG_PAR_PA, val);

	/* Reset hardware */
	writeq_relaxed(0, tbu->base + DEBUG_TXN_TRIGG_REG);
	writeq_relaxed(0, tbu->base + DEBUG_VA_ADDR_REG);
	val = readl_relaxed(tbu->base + DEBUG_SID_HALT_REG);
	val &= ~DEBUG_SID_HALT_SID;
	writel_relaxed(val, tbu->base + DEBUG_SID_HALT_REG);

	return phys;
}

static phys_addr_t qsmmuv500_iova_to_phys(struct arm_smmu_domain *smmu_domain,
					  dma_addr_t iova, u32 sid)
{
	struct arm_smmu_device *smmu = smmu_domain->smmu;
	struct qcom_smmu *qsmmu = to_qcom_smmu(smmu);
	int idx = smmu_domain->cfg.cbndx;
	struct qsmmuv500_tbu *tbu;
	u32 sctlr_orig, sctlr;
	phys_addr_t phys = 0;
	unsigned long flags;
	int attempt = 0;
	int ret;
	u64 fsr;

	tbu = qsmmuv500_find_tbu(qsmmu, sid);
	if (!tbu)
		return 0;

	ret = icc_set_bw(tbu->path, 0, UINT_MAX);
	if (ret)
		return ret;

	ret = clk_prepare_enable(tbu->clk);
	if (ret)
		goto disable_icc;

	ret = qsmmuv500_tbu_halt(tbu, smmu_domain);
	if (ret)
		goto disable_clk;

	/*
	 * ATOS/ECATS can trigger the fault interrupt, so disable it temporarily
	 * and check for an interrupt manually.
	 */
	sctlr_orig = arm_smmu_cb_read(smmu, idx, ARM_SMMU_CB_SCTLR);
	sctlr = sctlr_orig & ~(ARM_SMMU_SCTLR_CFCFG | ARM_SMMU_SCTLR_CFIE);
	arm_smmu_cb_write(smmu, idx, ARM_SMMU_CB_SCTLR, sctlr);

	fsr = arm_smmu_cb_read(smmu, idx, ARM_SMMU_CB_FSR);
	if (fsr & ARM_SMMU_CB_FSR_FAULT) {
		/* Clear pending interrupts */
		arm_smmu_cb_write(smmu, idx, ARM_SMMU_CB_FSR, fsr);

		/*
		 * TBU halt takes care of resuming any stalled transcation.
		 * Kept it here for completeness sake.
		 */
		if (fsr & ARM_SMMU_CB_FSR_SS)
			arm_smmu_cb_write(smmu, idx, ARM_SMMU_CB_RESUME,
					  ARM_SMMU_RESUME_TERMINATE);
	}

	/* Only one concurrent atos operation */
	spin_lock_irqsave(&atos_lock, flags);

	/*
	 * If the translation fails, attempt the lookup more time."
	 */
	do {
		phys = qsmmuv500_tbu_trigger_atos(smmu_domain, tbu, iova, sid);

		fsr = arm_smmu_cb_read(smmu, idx, ARM_SMMU_CB_FSR);
		if (fsr & ARM_SMMU_CB_FSR_FAULT) {
			/* Clear pending interrupts */
			arm_smmu_cb_write(smmu, idx, ARM_SMMU_CB_FSR, fsr);

			if (fsr & ARM_SMMU_CB_FSR_SS)
				arm_smmu_cb_write(smmu, idx, ARM_SMMU_CB_RESUME,
						  ARM_SMMU_RESUME_TERMINATE);
		}
	} while (!phys && attempt++ < 2);

	arm_smmu_cb_write(smmu, idx, ARM_SMMU_CB_SCTLR, sctlr_orig);
	spin_unlock_irqrestore(&atos_lock, flags);
	qsmmuv500_tbu_resume(tbu);

	/* Read to complete prior write transcations */
	readl_relaxed(tbu->base + DEBUG_SR_HALT_ACK_REG);

disable_clk:
	clk_disable_unprepare(tbu->clk);
disable_icc:
	icc_set_bw(tbu->path, 0, 0);

	return phys;
}

static phys_addr_t qcom_smmu_iova_to_phys_hard(struct iommu_domain *domain, dma_addr_t iova)
{
	struct arm_smmu_domain *smmu_domain = to_smmu_domain(domain);
	struct arm_smmu_device *smmu = smmu_domain->smmu;
	int idx = smmu_domain->cfg.cbndx;
	u32 frsynra;
	u16 sid;

	frsynra = arm_smmu_gr1_read(smmu, ARM_SMMU_GR1_CBFRSYNRA(idx));
	sid = FIELD_GET(ARM_SMMU_CBFRSYNRA_SID, frsynra);

	return qsmmuv500_iova_to_phys(smmu_domain, iova, sid);
}

static phys_addr_t qcom_smmu_verify_fault(struct iommu_domain *domain, dma_addr_t iova, u32 fsr)
{
	struct arm_smmu_domain *smmu_domain = to_smmu_domain(domain);
	struct io_pgtable *iop = io_pgtable_ops_to_pgtable(smmu_domain->pgtbl_ops);
	struct arm_smmu_device *smmu = smmu_domain->smmu;
	phys_addr_t phys_post_tlbiall;
	phys_addr_t phys;

	phys = qcom_smmu_iova_to_phys_hard(domain, iova);
	io_pgtable_tlb_flush_all(iop);
	phys_post_tlbiall = qcom_smmu_iova_to_phys_hard(domain, iova);

	if (phys != phys_post_tlbiall) {
		dev_err(smmu->dev,
			"ATOS results differed across TLBIALL... (before: %pa after: %pa)\n",
			&phys, &phys_post_tlbiall);
	}

	return (phys == 0 ? phys_post_tlbiall : phys);
}


phys_addr_t qsmmuv500_iova_to_phys_ecats(struct arm_smmu_domain *smmu_domain, dma_addr_t iova,
					   u32 sid, unsigned long trans_flags)
{
	struct arm_smmu_device *smmu = smmu_domain->smmu;
	struct arm_smmu_cfg *cfg = &smmu_domain->cfg;
	struct qsmmuv500_tbu *tbu;
	phys_addr_t phys = 0;
	int idx = cfg->cbndx;
	int needs_redo = 0;
	u32 sctlr_orig, sctlr, fsr;

	/* Log function entry and arguments */
	pr_err("%s: Entry - smmu=%p idx=%d iova=%pad sid=0x%x flags=0x%lx\n", 
		__func__, smmu, idx, &iova, sid, trans_flags);

	tbu = qsmmuv500_find_tbu(to_qcom_smmu(smmu), sid);
	if (!tbu) {
		pr_err("%s: Failed to find TBU for sid 0x%x\n", __func__, sid);
		return 0;
	}
	pr_err("%s: TBU found: %p\n", __func__, tbu);

/*	if (qsmmuv500_tbu_halt(tbu, smmu_domain))
		goto out_power_off;
*/
	/*
	 * ECATS can trigger the fault interrupt, so disable it temporarily
	 * and check for an interrupt manually.
	 */
	sctlr_orig = arm_smmu_cb_read(smmu, idx, ARM_SMMU_CB_SCTLR);
	sctlr = sctlr_orig & ~(ARM_SMMU_SCTLR_CFCFG | ARM_SMMU_SCTLR_CFIE);
	arm_smmu_cb_write(smmu, idx, ARM_SMMU_CB_SCTLR, sctlr);

	pr_err("%s: SCTLR modified. Orig: 0x%x, New: 0x%x (Disabled CFIE/CFCFG)\n", 
		__func__, sctlr_orig, sctlr);

	fsr = arm_smmu_cb_read(smmu, idx, ARM_SMMU_CB_FSR);
	if (fsr & ARM_SMMU_CB_FSR_FAULT) {
		pr_err("%s: Pre-existing fault detected! FSR=0x%x\n", __func__, fsr);

		/* Clear pending interrupts */
		arm_smmu_cb_write(smmu, idx, ARM_SMMU_CB_FSR, fsr);
		/*
		 * Barrier required to ensure that the FSR is cleared
		 * before resuming SMMU operation.
		 */
		wmb();

		/*
		 * TBU halt takes care of resuming any stalled transcation.
		 * Kept it here for completeness sake.
		 */
		if (fsr & ARM_SMMU_CB_FSR_SS) {
			pr_err("%s: Terminating stalled transaction (FSR_SS set)\n", __func__);
			arm_smmu_cb_write(smmu, idx, ARM_SMMU_CB_RESUME,
					  ARM_SMMU_RESUME_TERMINATE);
		}
	}

	/*
	 * After a failed translation, the next successful translation will
	 * incorrectly be reported as a failure.
	 */
	do {
		pr_err("%s: Triggering ATOS (attempt %d)...\n", __func__, needs_redo);
		phys = tbu->impl->trigger_atos(tbu, iova, sid, trans_flags);
		pr_err("%s: ATOS result: phys=%pa\n", __func__, &phys);

		fsr = arm_smmu_cb_read(smmu, idx, ARM_SMMU_CB_FSR);
		if (fsr & ARM_SMMU_CB_FSR_FAULT) {
			pr_err("%s: Fault during ATOS! FSR=0x%x\n", __func__, fsr);

			/* Clear pending interrupts */
			arm_smmu_cb_write(smmu, idx, ARM_SMMU_CB_FSR, fsr);
			/*
			 * Barrier required to ensure that the FSR is cleared
			 * before resuming SMMU operation.
			 */
			wmb();

			if (fsr & ARM_SMMU_CB_FSR_SS) {
				pr_err("%s: Terminating stalled transaction in loop\n", __func__);
				arm_smmu_cb_write(smmu, idx, ARM_SMMU_CB_RESUME,
						  ARM_SMMU_RESUME_TERMINATE);
			}
		}
	} while (!phys && needs_redo++ < 2);

	if (!phys)
		pr_err("%s: ATOS failed after retries.\n", __func__);
	else
		pr_err("%s: ATOS successful. Phys: %pa\n", __func__, &phys);

	arm_smmu_cb_write(smmu, idx, ARM_SMMU_CB_SCTLR, sctlr_orig);
	pr_err("%s: Restored SCTLR to 0x%x\n", __func__, sctlr_orig);

//	qsmmuv500_tbu_resume(tbu);

out_power_off:
	/* Read to complete prior write transcations */
	tbu->impl->write_sync(tbu);

	/* Wait for read to complete before off */
	rmb();
	
	pr_err("%s: Exit returning %pa\n", __func__, &phys);
	return phys;
}

phys_addr_t qsmmuv500_iova_to_phys_hard(
					struct iommu_domain *domain,
					struct qcom_iommu_atos_txn *txn)
{
	return qsmmuv500_iova_to_phys_ecats(to_smmu_domain(domain), txn->addr, txn->id,
				txn->flags);
}
EXPORT_SYMBOL_GPL(qsmmuv500_iova_to_phys_hard);

static int qtb500_tbu_halt_req(struct qsmmuv500_tbu *tbu)
{
	void __iomem *qtb_base = tbu->base;
	struct qtb500_device *qtb = to_qtb500(tbu);
	u64 val;

	if (qtb->no_halt)
		return 0;

	val = readq_relaxed(qtb_base + QTB_OVR_DBG_FENCEREQ);
	val |= QTB_OVR_DBG_FENCEREQ_HALT;
	writeq_relaxed(val, qtb_base  + QTB_OVR_DBG_FENCEREQ);

	return 0;
}

static int qtb500_tbu_halt_poll(struct qsmmuv500_tbu *tbu)
{
	void __iomem *qtb_base = tbu->base;
	struct qtb500_device *qtb = to_qtb500(tbu);
	u64 val, status;

	if (qtb->no_halt)
		return 0;

	if (readq_poll_timeout_atomic(qtb_base + QTB_OVR_DBG_FENCEACK, status,
				      (status &  QTB_OVR_DBG_FENCEACK_ACK), 0,
				      QTB_DBG_TIMEOUT_US)) {
		dev_err(tbu->dev, "Couldn't halt QTB\n");

		val = readq_relaxed(qtb_base + QTB_OVR_DBG_FENCEREQ);
		val &= ~QTB_OVR_DBG_FENCEREQ_HALT;
		writeq_relaxed(val, qtb_base + QTB_OVR_DBG_FENCEREQ);

		return -ETIMEDOUT;
	}

	return 0;
}

static void qtb500_tbu_resume(struct qsmmuv500_tbu *tbu)
{
	void __iomem *qtb_base = tbu->base;
	struct qtb500_device *qtb = to_qtb500(tbu);
	u64 val;

	if (qtb->no_halt)
		return;

	val = readq_relaxed(qtb_base + QTB_OVR_DBG_FENCEREQ);
	val &= ~QTB_OVR_DBG_FENCEREQ_HALT;
	writeq_relaxed(val, qtb_base  + QTB_OVR_DBG_FENCEREQ);
}

static phys_addr_t qtb500_trigger_atos(struct qsmmuv500_tbu *tbu, dma_addr_t iova,
				       u32 sid, unsigned long trans_flags)
{
	void __iomem *qtb_base = tbu->base;
	u64 infld0, infld1, infld2, val, status_val;
	phys_addr_t phys = 0;
	ktime_t timeout;
	bool ecats_timedout = false;
	int loop_cnt = 0;

	/* Log Entry */
	pr_err("%s: Entry - tbu=%p base=%p iova=%pad sid=0x%x flags=0x%lx\n",
		__func__, tbu, qtb_base, &iova, sid, trans_flags);

	/*
	 * Recommended to set:
	 *
	 * QTB_OVR_ECATS_INFLD0.QAD == 0 (AP Access Domain)
	 * QTB_OVR_EACTS_INFLD0.PCIE_NO_SNOOP == 0 (IO-Coherency enabled)
	 */
	infld0 = FIELD_PREP(QTB_OVR_ECATS_INFLD0_SID, sid);
	if (trans_flags & IOMMU_TRANS_SEC)
		infld0 |= QTB_OVR_ECATS_INFLD0_SEC_SID;

	infld1 = 0;
	if (trans_flags & IOMMU_TRANS_PRIV)
		infld1 |= QTB_OVR_ECATS_INFLD1_PNU;
	if (trans_flags & IOMMU_TRANS_INST)
		infld1 |= QTB_OVR_ECATS_INFLD1_IND;
	/*
	 * Recommended to set:
	 *
	 * QTB_OVR_ECATS_INFLD1.DIRTY == 0,
	 * QTB_OVR_ECATS_INFLD1.TR_TYPE == 4 (Cacheable and Shareable memory)
	 * QTB_OVR_ECATS_INFLD1.ALLOC == 0 (No allocation in TLB/caches)
	 */
	infld1 |= FIELD_PREP(QTB_OVR_ECATS_INFLD1_TR_TYPE, QTB_OVR_ECATS_INFLD1_TR_TYPE_SHARED);
	if (!(trans_flags & IOMMU_TRANS_SEC))
		infld1 |= QTB_OVR_ECATS_INFLD1_NON_SEC;
	if (trans_flags & IOMMU_TRANS_WRITE)
		infld1 |= FIELD_PREP(QTB_OVR_ECATS_INFLD1_OPC, QTB_OVR_ECATS_INFLD1_OPC_WRI);

	infld2 = iova;

	/* Log Register Prep */
	pr_err("%s: Writing ECATS Regs:\n"
	       "  INFLD0 (SID/Attr): 0x%llx\n"
	       "  INFLD1 (Prot/Ops): 0x%llx\n"
	       "  INFLD2 (IOVA)    : 0x%llx\n",
	       __func__, infld0, infld1, infld2);

	writeq_relaxed(infld0, qtb_base + QTB_OVR_ECATS_INFLD0);
	writeq_relaxed(infld1, qtb_base + QTB_OVR_ECATS_INFLD1);
	writeq_relaxed(infld2, qtb_base + QTB_OVR_ECATS_INFLD2);
	
	pr_err("%s: Triggering START...\n", __func__);
	writeq_relaxed(QTB_OVR_ECATS_TRIGGER_START, qtb_base + QTB_OVR_ECATS_TRIGGER);

	timeout = ktime_add_us(ktime_get(), QTB_DBG_TIMEOUT_US);
	for (;;) {
		loop_cnt++;
		status_val = readq_relaxed(qtb_base + QTB_OVR_ECATS_STATUS);
		if (status_val & QTB_OVR_ECATS_STATUS_DONE)
			break;
		val = readq_relaxed(qtb_base + QTB_OVR_ECATS_OUTFLD0);
		if (val & QTB_OVR_ECATS_OUTFLD0_FAULT)
			break;
		if (ktime_compare(ktime_get(), timeout) > 0) {
			ecats_timedout = true;
			break;
		}
	}

	/* Read final state */
	val = readq_relaxed(qtb_base + QTB_OVR_ECATS_OUTFLD0);

	/* Log HW Response */
	pr_err("%s: Poll finished after %d loops. TimedOut=%d\n"
	       "  STATUS : 0x%llx (DONE=%d)\n"
	       "  OUTFLD0: 0x%llx (FAULT=%d)\n",
	       __func__, loop_cnt, ecats_timedout,
	       status_val, !!(status_val & QTB_OVR_ECATS_STATUS_DONE),
	       val, !!(val & QTB_OVR_ECATS_OUTFLD0_FAULT));

	if (val & QTB_OVR_ECATS_OUTFLD0_FAULT)
		dev_err(tbu->dev, "ECATS generated a fault interrupt! OUTFLD0 = 0x%llx SID = 0x%x\n",
			val, sid);
	else if (ecats_timedout)
		dev_err_ratelimited(tbu->dev, "ECATS translation timed out!\n");
	else {
		phys = FIELD_GET(QTB_OVR_ECATS_OUTFLD0_PA, val);
		pr_err("%s: Success! PA extracted: %pa\n", __func__, &phys);
	}

	/* Reset hardware for next transaction. */
	writeq_relaxed(0, qtb_base + QTB_OVR_ECATS_TRIGGER);

	return phys;
}

static void qtb500_tbu_write_sync(struct qsmmuv500_tbu *tbu)
{
	readl_relaxed(tbu->base + QTB_SWID_LOW);
}

static const struct qsmmuv500_tbu_impl qtb500_impl = {
	.halt_req = qtb500_tbu_halt_req,
	.halt_poll = qtb500_tbu_halt_poll,
	.resume = qtb500_tbu_resume,
	.trigger_atos = qtb500_trigger_atos,
	.write_sync = qtb500_tbu_write_sync,
};

static struct qsmmuv500_tbu *qtb500_impl_init(struct qsmmuv500_tbu *tbu)
{
	struct resource *ttres;
	struct qtb500_device *qtb;
	struct device *dev = tbu->dev;
	struct platform_device *pdev = to_platform_device(dev);
	int ret;

	qtb = devm_krealloc(dev, tbu, sizeof(*qtb), GFP_KERNEL);
	if (!qtb)
		return ERR_PTR(-ENOMEM);

	qtb->tbu.impl = &qtb500_impl;

	ret = of_property_read_u32(dev->of_node, "qcom,num-qtb-ports", &qtb->num_ports);
	if (ret)
		return ERR_PTR(ret);

	qtb->no_halt = of_property_read_bool(dev->of_node, "qcom,no-qtb-atos-halt");

	ttres = platform_get_resource_byname(pdev, IORESOURCE_MEM, "transactiontracker-base");
	if (ttres) {
		qtb->transactiontracker_base = devm_ioremap_resource(dev, ttres);
		if (IS_ERR(qtb->transactiontracker_base))
			dev_info(dev, "devm_ioremap failure for transaction tracker\n");
	} else {
		qtb->transactiontracker_base = NULL;
		dev_info(dev, "Unable to get the transactiontracker-base\n");
	}

	return &qtb->tbu;
}

static struct qsmmuv500_tbu *qsmmuv500_tbu_impl_init(struct qsmmuv500_tbu *tbu)
{
	if (of_device_is_compatible(tbu->dev->of_node, "qcom,qtb500"))
		return qtb500_impl_init(tbu);
	return tbu;
}

static int qsmmuv500_tbu_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct device *dev = &pdev->dev;
	struct arm_smmu_device *smmu;
	struct qsmmuv500_tbu *tbu;
	struct resource *res;
	struct qcom_smmu *qsmmu;
	int ret;

	pr_err("%s: Probing TBU for node: %pOF\n", __func__, np);

	smmu = dev_get_drvdata(dev->parent);
	if (!smmu) {
		pr_err("%s: Failed to get SMMU driver data from parent. Deferring probe.\n", __func__);
		return -EPROBE_DEFER;
	}

	qsmmu = to_qcom_smmu(smmu);

	tbu = devm_kzalloc(dev, sizeof(*tbu), GFP_KERNEL);
	if (!tbu) {
		pr_err("%s: Failed to allocate memory for TBU structure\n", __func__);
		return -ENOMEM;
	}

	tbu->dev = dev;

	tbu = qsmmuv500_tbu_impl_init(tbu);
	if (IS_ERR(tbu)) {
		pr_err("%s: qsmmuv500_tbu_impl_init failed with error %ld\n", __func__, PTR_ERR(tbu));
		return PTR_ERR(tbu);
	}

	INIT_LIST_HEAD(&tbu->list);
	spin_lock_init(&tbu->halt_lock);

	/* 
	 * CRITICAL: Logging the attempt and result of mapping the address 
	 * to debug the ECATS register issue.
	 */
	pr_err("%s: Attempting to map TBU base address for %pOF\n", __func__, np);
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res)
		return -EINVAL;

	tbu->base = devm_ioremap(dev, res->start, resource_size(res)); //devm_of_iomap(dev, np, 0, NULL);
	if (IS_ERR(tbu->base)) {
		pr_err("%s: FAILED to map TBU base address! Error: %ld\n", __func__, PTR_ERR(tbu->base));
		return PTR_ERR(tbu->base);
	}
	pr_err("%s: TBU base mapped successfully at virtual address: %p\n", __func__, tbu->base);

	ret = of_property_read_u32_array(np, "qcom,stream-id-range", tbu->sid_range, 2);
	if (ret) {
		pr_err("%s: Failed to read 'qcom,stream-id-range' (ret: %d)\n", __func__, ret);
		dev_err(dev, "The DT property 'qcom,stream-id-range' is mandatory\n");
		return ret;
	}
	pr_err("%s: Stream ID Range: start=%u, num=%u\n", __func__, tbu->sid_range[0], tbu->sid_range[1]);

	tbu->clk = devm_clk_get_optional(dev, NULL);
	if (IS_ERR(tbu->clk)) {
		pr_err("%s: Failed to get optional clock. Error: %ld\n", __func__, PTR_ERR(tbu->clk));
		return PTR_ERR(tbu->clk);
	}

	tbu->path = devm_of_icc_get(dev, NULL);
	if (IS_ERR(tbu->path)) {
		pr_err("%s: Failed to get interconnect path. Error: %ld\n", __func__, PTR_ERR(tbu->path));
		return PTR_ERR(tbu->path);
	}

	mutex_lock(&qsmmu->tbu_list_lock);
	list_add_tail(&tbu->list, &qsmmu->tbu_list);
	mutex_unlock(&qsmmu->tbu_list_lock);

	dev_set_drvdata(dev, tbu);

	pr_err("%s: TBU Probe completed successfully for %pOF\n", __func__, np);

	return 0;
}

static const struct of_device_id qsmmuv500_tbu_of_match[] = {
	{ .compatible = "qcom,qsmmuv500-tbu" },
	{ }
};

static struct platform_driver qsmmuv500_tbu_driver = {
	.driver = {
		.name           = "qsmmuv500-tbu",
		.of_match_table = qsmmuv500_tbu_of_match,
	},
	.probe = qsmmuv500_tbu_probe,
};
builtin_platform_driver(qsmmuv500_tbu_driver);
