/*
 * Copyright (c) 2016, NVIDIA CORPORATION. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *  * Neither the name of NVIDIA CORPORATION nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ``AS IS'' AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
 * OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef INCLUDE_T194_HSP_TEGRA_TOP_HW_H
#define INCLUDE_T194_HSP_TEGRA_TOP_HW_H

/* FIXME: There should be a header to define this: */
/* define doorbell master ID */
#define TEGRA_HSP_DB_MASTER_CCPLEX	1
#define TEGRA_HSP_DB_MASTER_DPMU	2
#define TEGRA_HSP_DB_MASTER_BPMP	3
#define TEGRA_HSP_DB_MASTER_SPE		4
#define TEGRA_HSP_DB_MASTER_SCE		5
#define TEGRA_HSP_DB_MASTER_DMA		6
#define TEGRA_HSP_DB_MASTER_TSECA	7
#define TEGRA_HSP_DB_MASTER_TSECB	8
#define TEGRA_HSP_DB_MASTER_JTAGM	9
#define TEGRA_HSP_DB_MASTER_CSITE	10
#define TEGRA_HSP_DB_MASTER_APE		11
#define TEGRA_HSP_DB_MASTER_PEATRANS	12
#define TEGRA_HSP_DB_MASTER_NVDEC	13
#define TEGRA_HSP_DB_MASTER_RCE		14
#define TEGRA_HSP_DB_MASTER_MAXNUM	15
#define TEGRA_HSP_DB_MASTER_NON_SECURE	16

/* FIXME: There should be a header to define this: */
/* define doorbell number and usage*/
#define TEGRA_HSP_DB_DPMU		0
#define TEGRA_HSP_DB_CCPLEX		1
#define TEGRA_HSP_DB_CCPLEX_TZ		2
#define TEGRA_HSP_DB_BPMP		3
#define TEGRA_HSP_DB_SPE		4
#define TEGRA_HSP_DB_SCE		5
#define TEGRA_HSP_DB_APE		6
#define TEGRA_HSP_DB_RCE		7
#define TEGRA_HSP_DB_MAXNUM		8

#endif /* INCLUDE_T194_HSP_TEGRA_TOP_HW_H */
