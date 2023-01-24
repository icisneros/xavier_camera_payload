/*
 * Copyright (c) 2017 NVIDIA Corporation.  All rights reserved.
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

#if !defined(NV_MC_DEFS_H)
#define NV_MC_DEFS_H
#define NV_MC_EMEM_HP_BITS 1
#define NV_MC_EMEM_TIMEOUT_BITS 7
#define NV_MC_TYPE_SR 0
#define NV_MC_TYPE_SW 1
#define NV_MC_TYPE_BR 2
#define NV_MC_TYPE_BW 3
#define NV_MC_TYPE_CW 5
#define NV_MC_HAS_CSR 1
#define NV_MC_HAS_CSW 1
#define NV_MC_MAX_AW     40
#define NV_MC_MAX_MW     512
#define NV_MC_MAX_BW     64
#define NV_MC_MIN_WDLOG2 6
#define NV_MC_INBAND_SWID_WIDTH 0
#define NV_AONDMAPC2MC_RETPIPECHAIN           1
#define NV_AONDMAPC2MC_RETPIPESRC           aonpg2
#define NV_AONDMAPC2MC_RETPIPEROOT          mchbb2
#define NV_AONDMAPC2MC_NBCLIENTS              2
#define NV_AONDMAPC2MC_NBRCLIENTS             1
#define NV_AONDMAPC2MC_RCPIDWIDTH             1
#define NV_AONDMAPC2MC_NBWCLIENTS             1
#define NV_AONDMAPC2MC_WRCPIDWIDTH            1
#define NV_AONDMAPC2MC_AW                    40
#define NV_AONDMAPC2MC_WDLOG2                 5
#define NV_AONDMAPC2MC_MW                   512
#define NV_AONDMAPC2MC_BW                    64
#define NV_AONDMAPC2MC_TW                     8
#define NV_AONDMAPC2MC_COALESCABLE            0
#define NV_AONDMAPC2MC_MULTITHREADED          0
#define NV_AONDMAPC2MC_RVPR                   1
#define NV_AONDMA2MC_PID                      0
#define NV_AONDMA2MC_NBCLIENTS                2
#define NV_AONDMA2MC_NBRCLIENTS               1
#define NV_AONDMA2MC_NBRCLIENTS               1
#define NV_AONDMA2MC_NBWCLIENTS               1
#define NV_AONDMA2MC_NBDIRCLIENTS             0
#define NV_AONDMA2MC_AW                      40
#define NV_AONDMA2MC_WDLOG2                   5
#define NV_AONDMA2MC_CWDLOG2                  5
#define NV_AONDMAR2MC_SR_CWDLOG2              5
#define NV_AONDMAW2MC_SW_CWDLOG2              5
#define NV_AONDMA2MC_CW                     256
#define NV_AONDMA2MC_BW                      64
#define NV_AONDMA2MC_CBW                     32
#define NV_AONDMA2MC_TW                       8
#define NV_AONDMA2MC_NBTILECLIENTS            0
#define NV_AONDMAR2MC_SR_PID                  0
#define NV_AONDMAW2MC_SW_PID                  1
#define NV_AONDMAR2MC_SR_SID                  0
#define NV_AONDMAW2MC_SW_SID                  1
#define NV_AONDMAW2MC_SWBW                   32
#define NV_AONDMAR2MC_RFIFO_MAXCREDITS        8
#define NV_AONDMAW2MC_RFIFO_MAXCREDITS        7
#define NV_AONDMAR2MC_COALESCABLE             1
#define NV_AONDMAW2MC_COALESCABLE             1
#define NV_AONDMAR2MC_MULTITHREADED           0
#define NV_AONDMAW2MC_MULTITHREADED           0
#define NV_AONDMAR2MC_TILE                    0
#define NV_AONDMAR2MC_ADRXY                   0
#define NV_AONDMAW2MC_TILE                    0
#define NV_AONDMAW2MC_ADRXY                   0
#define NV_AONDMAR2MC_SR_LA_SCALING_2       0
#define NV_AONDMAW2MC_SW_LA_SCALING_2       0
#define NV_AONDMAR2MC_SWNAME                aon
#define NV_AONDMAW2MC_SWNAME                aon
#define NV_AONDMAR2MC_SR_LA_SCALING_2       0
#define NV_AONDMAW2MC_SW_LA_SCALING_2       0
#define NV_AONPC2MC_RETPIPECHAIN              1
#define NV_AONPC2MC_RETPIPESRC              aonpg2
#define NV_AONPC2MC_RETPIPEROOT             mchbb2
#define NV_AONPC2MC_NBCLIENTS                 2
#define NV_AONPC2MC_NBRCLIENTS                1
#define NV_AONPC2MC_RCPIDWIDTH                1
#define NV_AONPC2MC_NBWCLIENTS                1
#define NV_AONPC2MC_WRCPIDWIDTH               1
#define NV_AONPC2MC_AW                       40
#define NV_AONPC2MC_WDLOG2                    5
#define NV_AONPC2MC_MW                      512
#define NV_AONPC2MC_BW                       64
#define NV_AONPC2MC_TW                        8
#define NV_AONPC2MC_COALESCABLE               0
#define NV_AONPC2MC_MULTITHREADED             0
#define NV_AONPC2MC_RVPR                      1
#define NV_AON2MC_PID                         0
#define NV_AON2MC_NBCLIENTS                   2
#define NV_AON2MC_NBRCLIENTS                  1
#define NV_AON2MC_NBRCLIENTS                  1
#define NV_AON2MC_NBWCLIENTS                  1
#define NV_AON2MC_NBDIRCLIENTS                0
#define NV_AON2MC_AW                         40
#define NV_AON2MC_WDLOG2                      5
#define NV_AON2MC_CWDLOG2                     5
#define NV_AONR2MC_SR_CWDLOG2                 5
#define NV_AONW2MC_SW_CWDLOG2                 5
#define NV_AON2MC_CW                        256
#define NV_AON2MC_BW                         64
#define NV_AON2MC_CBW                        32
#define NV_AON2MC_TW                          8
#define NV_AON2MC_NBTILECLIENTS               0
#define NV_AONR2MC_SR_PID                     0
#define NV_AONW2MC_SW_PID                     1
#define NV_AONR2MC_SR_SID                     0
#define NV_AONW2MC_SW_SID                     1
#define NV_AONW2MC_SWBW                      32
#define NV_AONR2MC_RFIFO_MAXCREDITS           8
#define NV_AONW2MC_RFIFO_MAXCREDITS           7
#define NV_AONR2MC_COALESCABLE                1
#define NV_AONW2MC_COALESCABLE                1
#define NV_AONR2MC_MULTITHREADED              0
#define NV_AONW2MC_MULTITHREADED              0
#define NV_AONR2MC_TILE                       0
#define NV_AONR2MC_ADRXY                      0
#define NV_AONW2MC_TILE                       0
#define NV_AONW2MC_ADRXY                      0
#define NV_AONR2MC_SR_LA_SCALING_2          0
#define NV_AONW2MC_SW_LA_SCALING_2          0
#define NV_AONR2MC_SWNAME                   aon
#define NV_AONW2MC_SWNAME                   aon
#define NV_AONR2MC_SR_LA_SCALING_2          0
#define NV_AONW2MC_SW_LA_SCALING_2          0
#define NV_APB2MC_RETPIPECHAIN                1
#define NV_APB2MC_RETPIPESRC                sec12
#define NV_APB2MC_RETPIPEROOT               mchbb2
#define NV_APB2MC_NBCLIENTS                   6
#define NV_APB2MC_NBRCLIENTS                  3
#define NV_APB2MC_RCPIDWIDTH                  3
#define NV_APB2MC_NBWCLIENTS                  3
#define NV_APB2MC_WRCPIDWIDTH                 3
#define NV_APB2MC_AW                         40
#define NV_APB2MC_WDLOG2                      5
#define NV_APB2MC_MW                        512
#define NV_APB2MC_BW                         64
#define NV_APB2MC_TW                          8
#define NV_APB2MC_COALESCABLE                 0
#define NV_APB2MC_MULTITHREADED               0
#define NV_APB2MC_RVPR                        3
#define NV_SE2MC_PID                          0
#define NV_SE2MC_NBCLIENTS                    2
#define NV_SE2MC_NBRCLIENTS                   1
#define NV_SE2MC_NBRCLIENTS                   1
#define NV_SE2MC_NBWCLIENTS                   1
#define NV_SE2MC_NBDIRCLIENTS                 0
#define NV_SE2MC_AW                          40
#define NV_SE2MC_WDLOG2                       5
#define NV_SE2MC_CWDLOG2                      5
#define NV_SESRD2MC_SR_CWDLOG2                5
#define NV_SESWR2MC_SW_CWDLOG2                5
#define NV_SE2MC_CW                         256
#define NV_SE2MC_BW                          64
#define NV_SE2MC_CBW                         32
#define NV_SE2MC_TW                           8
#define NV_SE2MC_NBTILECLIENTS                0
#define NV_SESRD2MC_SR_PID                    0
#define NV_SESWR2MC_SW_PID                    3
#define NV_SESRD2MC_SR_SID                    0
#define NV_SESWR2MC_SW_SID                    1
#define NV_SESRD2MC_RFIFO_MAXCREDITS          8
#define NV_SESWR2MC_RFIFO_MAXCREDITS          7
#define NV_SESRD2MC_COALESCABLE               1
#define NV_SESWR2MC_COALESCABLE               1
#define NV_SESRD2MC_MULTITHREADED             0
#define NV_SESWR2MC_MULTITHREADED             0
#define NV_SESRD2MC_TILE                      0
#define NV_SESRD2MC_ADRXY                     0
#define NV_SESWR2MC_TILE                      0
#define NV_SESWR2MC_ADRXY                     0
#define NV_SESRD2MC_SR_LA_SCALING_2         0
#define NV_SESWR2MC_SW_LA_SCALING_2         0
#define NV_SESRD2MC_SWNAME                  se
#define NV_SESWR2MC_SWNAME                  se
#define NV_SESRD2MC_SR_LA_SCALING_2         0
#define NV_SESWR2MC_SW_LA_SCALING_2         0
#define NV_TSEC2MC_PID                        1
#define NV_TSEC2MC_NBCLIENTS                  2
#define NV_TSEC2MC_NBRCLIENTS                 1
#define NV_TSEC2MC_NBRCLIENTS                 1
#define NV_TSEC2MC_NBWCLIENTS                 1
#define NV_TSEC2MC_NBDIRCLIENTS               0
#define NV_TSEC2MC_AW                        40
#define NV_TSEC2MC_WDLOG2                     5
#define NV_TSEC2MC_CWDLOG2                    5
#define NV_TSECSRD2MC_SR_CWDLOG2              5
#define NV_TSECSWR2MC_SW_CWDLOG2              5
#define NV_TSEC2MC_CW                       256
#define NV_TSEC2MC_BW                        64
#define NV_TSEC2MC_CBW                       32
#define NV_TSEC2MC_TW                         8
#define NV_TSEC2MC_NBTILECLIENTS              0
#define NV_TSECSRD2MC_SR_PID                  1
#define NV_TSECSWR2MC_SW_PID                  4
#define NV_TSECSRD2MC_SR_SID                  0
#define NV_TSECSWR2MC_SW_SID                  1
#define NV_TSECSRD2MC_RFIFO_MAXCREDITS        8
#define NV_TSECSWR2MC_RFIFO_MAXCREDITS        7
#define NV_TSECSRD2MC_COALESCABLE             1
#define NV_TSECSWR2MC_COALESCABLE             1
#define NV_TSECSRD2MC_MULTITHREADED           0
#define NV_TSECSWR2MC_MULTITHREADED           0
#define NV_TSECSRD2MC_TILE                    0
#define NV_TSECSRD2MC_ADRXY                   0
#define NV_TSECSWR2MC_TILE                    0
#define NV_TSECSWR2MC_ADRXY                   0
#define NV_TSECSRD2MC_SR_LA_SCALING_2       0
#define NV_TSECSWR2MC_SW_LA_SCALING_2       0
#define NV_TSECSRD2MC_SWNAME                tsec
#define NV_TSECSWR2MC_SWNAME                tsec
#define NV_TSECSRD2MC_SR_LA_SCALING_2       0
#define NV_TSECSWR2MC_SW_LA_SCALING_2       0
#define NV_TSECB2MC_PID                       2
#define NV_TSECB2MC_NBCLIENTS                 2
#define NV_TSECB2MC_NBRCLIENTS                1
#define NV_TSECB2MC_NBRCLIENTS                1
#define NV_TSECB2MC_NBWCLIENTS                1
#define NV_TSECB2MC_NBDIRCLIENTS              0
#define NV_TSECB2MC_AW                       40
#define NV_TSECB2MC_WDLOG2                    5
#define NV_TSECB2MC_CWDLOG2                   5
#define NV_TSECSRDB2MC_SR_CWDLOG2             5
#define NV_TSECSWRB2MC_SW_CWDLOG2             5
#define NV_TSECB2MC_CW                      256
#define NV_TSECB2MC_BW                       64
#define NV_TSECB2MC_CBW                      32
#define NV_TSECB2MC_TW                        8
#define NV_TSECB2MC_NBTILECLIENTS             0
#define NV_TSECSRDB2MC_SR_PID                 2
#define NV_TSECSWRB2MC_SW_PID                 5
#define NV_TSECSRDB2MC_SR_SID                 0
#define NV_TSECSWRB2MC_SW_SID                 1
#define NV_TSECSRDB2MC_RFIFO_MAXCREDITS       8
#define NV_TSECSWRB2MC_RFIFO_MAXCREDITS       7
#define NV_TSECSRDB2MC_COALESCABLE            1
#define NV_TSECSWRB2MC_COALESCABLE            1
#define NV_TSECSRDB2MC_MULTITHREADED          0
#define NV_TSECSWRB2MC_MULTITHREADED          0
#define NV_TSECSRDB2MC_TILE                   0
#define NV_TSECSRDB2MC_ADRXY                  0
#define NV_TSECSWRB2MC_TILE                   0
#define NV_TSECSWRB2MC_ADRXY                  0
#define NV_TSECSRDB2MC_SR_LA_SCALING_2      0
#define NV_TSECSWRB2MC_SW_LA_SCALING_2      0
#define NV_TSECSRDB2MC_SWNAME               tsecb
#define NV_TSECSWRB2MC_SWNAME               tsecb
#define NV_TSECSRDB2MC_SR_LA_SCALING_2      0
#define NV_TSECSWRB2MC_SW_LA_SCALING_2      0
#define NV_APEDMAPC2MC_RETPIPECHAIN           1
#define NV_APEDMAPC2MC_RETPIPESRC           aud2
#define NV_APEDMAPC2MC_RETPIPEROOT          mchbb2
#define NV_APEDMAPC2MC_NBCLIENTS              2
#define NV_APEDMAPC2MC_NBRCLIENTS             1
#define NV_APEDMAPC2MC_RCPIDWIDTH             1
#define NV_APEDMAPC2MC_NBWCLIENTS             1
#define NV_APEDMAPC2MC_WRCPIDWIDTH            1
#define NV_APEDMAPC2MC_AW                    40
#define NV_APEDMAPC2MC_WDLOG2                 5
#define NV_APEDMAPC2MC_MW                   512
#define NV_APEDMAPC2MC_BW                    64
#define NV_APEDMAPC2MC_TW                     8
#define NV_APEDMAPC2MC_COALESCABLE            0
#define NV_APEDMAPC2MC_MULTITHREADED          0
#define NV_APEDMAPC2MC_RVPR                   1
#define NV_APEDMA2MC_PID                      0
#define NV_APEDMA2MC_NBCLIENTS                2
#define NV_APEDMA2MC_NBRCLIENTS               1
#define NV_APEDMA2MC_NBRCLIENTS               1
#define NV_APEDMA2MC_NBWCLIENTS               1
#define NV_APEDMA2MC_NBDIRCLIENTS             0
#define NV_APEDMA2MC_AW                      40
#define NV_APEDMA2MC_WDLOG2                   5
#define NV_APEDMA2MC_CWDLOG2                  5
#define NV_APEDMAR2MC_SR_CWDLOG2              5
#define NV_APEDMAW2MC_SW_CWDLOG2              5
#define NV_APEDMA2MC_CW                     256
#define NV_APEDMA2MC_BW                      64
#define NV_APEDMA2MC_CBW                     32
#define NV_APEDMA2MC_TW                       8
#define NV_APEDMA2MC_NBTILECLIENTS            0
#define NV_APEDMAR2MC_SR_PID                  0
#define NV_APEDMAW2MC_SW_PID                  1
#define NV_APEDMAR2MC_SR_SID                  0
#define NV_APEDMAW2MC_SW_SID                  1
#define NV_APEDMAW2MC_SWBW                   32
#define NV_APEDMAR2MC_RFIFO_MAXCREDITS        8
#define NV_APEDMAW2MC_RFIFO_MAXCREDITS        7
#define NV_APEDMAR2MC_COALESCABLE             1
#define NV_APEDMAW2MC_COALESCABLE             1
#define NV_APEDMAR2MC_MULTITHREADED           0
#define NV_APEDMAW2MC_MULTITHREADED           0
#define NV_APEDMAR2MC_TILE                    0
#define NV_APEDMAR2MC_ADRXY                   0
#define NV_APEDMAW2MC_TILE                    0
#define NV_APEDMAW2MC_ADRXY                   0
#define NV_APEDMAR2MC_SR_LA_SCALING_2       0
#define NV_APEDMAW2MC_SW_LA_SCALING_2       0
#define NV_APEDMAR2MC_SWNAME                ape
#define NV_APEDMAW2MC_SWNAME                ape
#define NV_APEDMAR2MC_SR_LA_SCALING_2       0
#define NV_APEDMAW2MC_SW_LA_SCALING_2       0
#define NV_AUD2MC_RETPIPECHAIN                1
#define NV_AUD2MC_RETPIPESRC                aud2
#define NV_AUD2MC_RETPIPEROOT               mchbb2
#define NV_AUD2MC_NBCLIENTS                   2
#define NV_AUD2MC_NBRCLIENTS                  1
#define NV_AUD2MC_RCPIDWIDTH                  1
#define NV_AUD2MC_NBWCLIENTS                  1
#define NV_AUD2MC_WRCPIDWIDTH                 1
#define NV_AUD2MC_AW                         40
#define NV_AUD2MC_WDLOG2                      5
#define NV_AUD2MC_MW                        512
#define NV_AUD2MC_BW                         64
#define NV_AUD2MC_TW                          8
#define NV_AUD2MC_COALESCABLE                 0
#define NV_AUD2MC_MULTITHREADED               0
#define NV_AUD2MC_RVPR                        1
#define NV_APE2MC_PID                         0
#define NV_APE2MC_NBCLIENTS                   2
#define NV_APE2MC_NBRCLIENTS                  1
#define NV_APE2MC_NBRCLIENTS                  1
#define NV_APE2MC_NBWCLIENTS                  1
#define NV_APE2MC_NBDIRCLIENTS                0
#define NV_APE2MC_AW                         40
#define NV_APE2MC_WDLOG2                      5
#define NV_APE2MC_CWDLOG2                     5
#define NV_APER2MC_SR_CWDLOG2                 5
#define NV_APEW2MC_SW_CWDLOG2                 5
#define NV_APE2MC_CW                        256
#define NV_APE2MC_BW                         64
#define NV_APE2MC_CBW                        32
#define NV_APE2MC_TW                          8
#define NV_APE2MC_NBTILECLIENTS               0
#define NV_APER2MC_SR_PID                     0
#define NV_APEW2MC_SW_PID                     1
#define NV_APER2MC_SR_SID                     0
#define NV_APEW2MC_SW_SID                     1
#define NV_APER2MC_RFIFO_MAXCREDITS           8
#define NV_APEW2MC_RFIFO_MAXCREDITS           7
#define NV_APER2MC_COALESCABLE                1
#define NV_APEW2MC_COALESCABLE                1
#define NV_APER2MC_MULTITHREADED              0
#define NV_APEW2MC_MULTITHREADED              0
#define NV_APER2MC_TILE                       0
#define NV_APER2MC_ADRXY                      0
#define NV_APEW2MC_TILE                       0
#define NV_APEW2MC_ADRXY                      0
#define NV_APER2MC_SR_LA_SCALING_2          0
#define NV_APEW2MC_SW_LA_SCALING_2          0
#define NV_APER2MC_SWNAME                   ape
#define NV_APEW2MC_SWNAME                   ape
#define NV_APER2MC_SR_LA_SCALING_2          0
#define NV_APEW2MC_SW_LA_SCALING_2          0
#define NV_BPMPDMAPC2MC_RETPIPECHAIN          1
#define NV_BPMPDMAPC2MC_RETPIPESRC          bpmp2
#define NV_BPMPDMAPC2MC_RETPIPEROOT         mchbb2
#define NV_BPMPDMAPC2MC_NBCLIENTS             2
#define NV_BPMPDMAPC2MC_NBRCLIENTS            1
#define NV_BPMPDMAPC2MC_RCPIDWIDTH            1
#define NV_BPMPDMAPC2MC_NBWCLIENTS            1
#define NV_BPMPDMAPC2MC_WRCPIDWIDTH           1
#define NV_BPMPDMAPC2MC_AW                   40
#define NV_BPMPDMAPC2MC_WDLOG2                5
#define NV_BPMPDMAPC2MC_MW                  512
#define NV_BPMPDMAPC2MC_BW                   64
#define NV_BPMPDMAPC2MC_TW                    8
#define NV_BPMPDMAPC2MC_COALESCABLE           0
#define NV_BPMPDMAPC2MC_MULTITHREADED         0
#define NV_BPMPDMAPC2MC_RVPR                  1
#define NV_BPMPDMA2MC_PID                     0
#define NV_BPMPDMA2MC_NBCLIENTS               2
#define NV_BPMPDMA2MC_NBRCLIENTS              1
#define NV_BPMPDMA2MC_NBRCLIENTS              1
#define NV_BPMPDMA2MC_NBWCLIENTS              1
#define NV_BPMPDMA2MC_NBDIRCLIENTS            0
#define NV_BPMPDMA2MC_AW                     40
#define NV_BPMPDMA2MC_WDLOG2                  5
#define NV_BPMPDMA2MC_CWDLOG2                 5
#define NV_BPMPDMAR2MC_SR_CWDLOG2             5
#define NV_BPMPDMAW2MC_SW_CWDLOG2             5
#define NV_BPMPDMA2MC_CW                    256
#define NV_BPMPDMA2MC_BW                     64
#define NV_BPMPDMA2MC_CBW                    32
#define NV_BPMPDMA2MC_TW                      8
#define NV_BPMPDMA2MC_NBTILECLIENTS           0
#define NV_BPMPDMAR2MC_SR_PID                 0
#define NV_BPMPDMAW2MC_SW_PID                 1
#define NV_BPMPDMAR2MC_SR_SID                 0
#define NV_BPMPDMAW2MC_SW_SID                 1
#define NV_BPMPDMAW2MC_SWBW                  32
#define NV_BPMPDMAR2MC_RFIFO_MAXCREDITS       8
#define NV_BPMPDMAW2MC_RFIFO_MAXCREDITS       7
#define NV_BPMPDMAR2MC_COALESCABLE            1
#define NV_BPMPDMAW2MC_COALESCABLE            1
#define NV_BPMPDMAR2MC_MULTITHREADED          0
#define NV_BPMPDMAW2MC_MULTITHREADED          0
#define NV_BPMPDMAR2MC_TILE                   0
#define NV_BPMPDMAR2MC_ADRXY                  0
#define NV_BPMPDMAW2MC_TILE                   0
#define NV_BPMPDMAW2MC_ADRXY                  0
#define NV_BPMPDMAR2MC_SR_LA_SCALING_2      0
#define NV_BPMPDMAW2MC_SW_LA_SCALING_2      0
#define NV_BPMPDMAR2MC_SWNAME               bpmp
#define NV_BPMPDMAW2MC_SWNAME               bpmp
#define NV_BPMPDMAR2MC_SR_LA_SCALING_2      0
#define NV_BPMPDMAW2MC_SW_LA_SCALING_2      0
#define NV_BPMPPC2MC_RETPIPECHAIN             1
#define NV_BPMPPC2MC_RETPIPESRC             bpmp2
#define NV_BPMPPC2MC_RETPIPEROOT            mchbb2
#define NV_BPMPPC2MC_NBCLIENTS                2
#define NV_BPMPPC2MC_NBRCLIENTS               1
#define NV_BPMPPC2MC_RCPIDWIDTH               1
#define NV_BPMPPC2MC_NBWCLIENTS               1
#define NV_BPMPPC2MC_WRCPIDWIDTH              1
#define NV_BPMPPC2MC_AW                      40
#define NV_BPMPPC2MC_WDLOG2                   5
#define NV_BPMPPC2MC_MW                     512
#define NV_BPMPPC2MC_BW                      64
#define NV_BPMPPC2MC_TW                       8
#define NV_BPMPPC2MC_COALESCABLE              0
#define NV_BPMPPC2MC_MULTITHREADED            0
#define NV_BPMPPC2MC_RVPR                     1
#define NV_BPMP2MC_PID                        0
#define NV_BPMP2MC_NBCLIENTS                  2
#define NV_BPMP2MC_NBRCLIENTS                 1
#define NV_BPMP2MC_NBRCLIENTS                 1
#define NV_BPMP2MC_NBWCLIENTS                 1
#define NV_BPMP2MC_NBDIRCLIENTS               0
#define NV_BPMP2MC_AW                        40
#define NV_BPMP2MC_WDLOG2                     5
#define NV_BPMP2MC_CWDLOG2                    5
#define NV_BPMPR2MC_SR_CWDLOG2                5
#define NV_BPMPW2MC_SW_CWDLOG2                5
#define NV_BPMP2MC_CW                       256
#define NV_BPMP2MC_BW                        64
#define NV_BPMP2MC_CBW                       32
#define NV_BPMP2MC_TW                         8
#define NV_BPMP2MC_NBTILECLIENTS              0
#define NV_BPMPR2MC_SR_PID                    0
#define NV_BPMPW2MC_SW_PID                    1
#define NV_BPMPR2MC_SR_SID                    0
#define NV_BPMPW2MC_SW_SID                    1
#define NV_BPMPW2MC_SWBW                     32
#define NV_BPMPR2MC_RFIFO_MAXCREDITS          8
#define NV_BPMPW2MC_RFIFO_MAXCREDITS          7
#define NV_BPMPR2MC_COALESCABLE               1
#define NV_BPMPW2MC_COALESCABLE               1
#define NV_BPMPR2MC_MULTITHREADED             0
#define NV_BPMPW2MC_MULTITHREADED             0
#define NV_BPMPR2MC_TILE                      0
#define NV_BPMPR2MC_ADRXY                     0
#define NV_BPMPW2MC_TILE                      0
#define NV_BPMPW2MC_ADRXY                     0
#define NV_BPMPR2MC_SR_LA_SCALING_2         0
#define NV_BPMPW2MC_SW_LA_SCALING_2         0
#define NV_BPMPR2MC_SWNAME                  bpmp
#define NV_BPMPW2MC_SWNAME                  bpmp
#define NV_BPMPR2MC_SR_LA_SCALING_2         0
#define NV_BPMPW2MC_SW_LA_SCALING_2         0
#define NV_DFD2MC_RETPIPECHAIN                1
#define NV_DFD2MC_RETPIPESRC                dfd2
#define NV_DFD2MC_RETPIPEROOT               mchbb2
#define NV_DFD2MC_NBCLIENTS                   2
#define NV_DFD2MC_NBRCLIENTS                  1
#define NV_DFD2MC_RCPIDWIDTH                  1
#define NV_DFD2MC_NBWCLIENTS                  1
#define NV_DFD2MC_WRCPIDWIDTH                 1
#define NV_DFD2MC_AW                         40
#define NV_DFD2MC_WDLOG2                      5
#define NV_DFD2MC_MW                        512
#define NV_DFD2MC_BW                         64
#define NV_DFD2MC_TW                          8
#define NV_DFD2MC_COALESCABLE                 0
#define NV_DFD2MC_MULTITHREADED               0
#define NV_DFD2MC_RVPR                        1
#define NV_ETR2MC_PID                         0
#define NV_ETR2MC_NBCLIENTS                   2
#define NV_ETR2MC_NBRCLIENTS                  1
#define NV_ETR2MC_NBRCLIENTS                  1
#define NV_ETR2MC_NBWCLIENTS                  1
#define NV_ETR2MC_NBDIRCLIENTS                0
#define NV_ETR2MC_AW                         40
#define NV_ETR2MC_WDLOG2                      5
#define NV_ETR2MC_CWDLOG2                     5
#define NV_ETRR2MC_SR_CWDLOG2                 5
#define NV_ETRW2MC_SW_CWDLOG2                 5
#define NV_ETR2MC_CW                        256
#define NV_ETR2MC_BW                         64
#define NV_ETR2MC_CBW                        32
#define NV_ETR2MC_TW                          8
#define NV_ETR2MC_NBTILECLIENTS               0
#define NV_ETRR2MC_SR_PID                     0
#define NV_ETRW2MC_SW_PID                     1
#define NV_ETRR2MC_SR_SID                     0
#define NV_ETRW2MC_SW_SID                     1
#define NV_ETRR2MC_RFIFO_MAXCREDITS           8
#define NV_ETRW2MC_RFIFO_MAXCREDITS           7
#define NV_ETRR2MC_COALESCABLE                1
#define NV_ETRW2MC_COALESCABLE                1
#define NV_ETRR2MC_MULTITHREADED              0
#define NV_ETRW2MC_MULTITHREADED              0
#define NV_ETRR2MC_TILE                       0
#define NV_ETRR2MC_ADRXY                      0
#define NV_ETRW2MC_TILE                       0
#define NV_ETRW2MC_ADRXY                      0
#define NV_ETRR2MC_SR_LA_SCALING_2          0
#define NV_ETRW2MC_SW_LA_SCALING_2          0
#define NV_ETRR2MC_SWNAME                   etr
#define NV_ETRW2MC_SWNAME                   etr
#define NV_ETRR2MC_SR_LA_SCALING_2          0
#define NV_ETRW2MC_SW_LA_SCALING_2          0
#define NV_DIS2MC_RETPIPECHAIN                1
#define NV_DIS2MC_RETPIPESRC                disiha0
#define NV_DIS2MC_RETPIPEROOT               mcha00
#define NV_DIS2MC_NBCLIENTS                   1
#define NV_DIS2MC_NBRCLIENTS                  1
#define NV_DIS2MC_RCPIDWIDTH                  0
#define NV_DIS2MC_NBWCLIENTS                  0
#define NV_DIS2MC_WRCPIDWIDTH                 0
#define NV_DIS2MC_AW                         40
#define NV_DIS2MC_WDLOG2                      5
#define NV_DIS2MC_MW                        512
#define NV_DIS2MC_TW                          8
#define NV_DIS2MC_COALESCABLE                 0
#define NV_DIS2MC_MULTITHREADED               0
#define NV_DIS2MC_RVPR                        1
#define NV_NVDISPLAY2MC_PID                   0
#define NV_NVDISPLAY2MC_NBCLIENTS             1
#define NV_NVDISPLAY2MC_NBRCLIENTS            1
#define NV_NVDISPLAY2MC_NBRCLIENTS            1
#define NV_NVDISPLAY2MC_NBWCLIENTS            0
#define NV_NVDISPLAY2MC_NBDIRCLIENTS          0
#define NV_NVDISPLAY2MC_AW                   40
#define NV_NVDISPLAY2MC_WDLOG2                5
#define NV_NVDISPLAY2MC_CWDLOG2               6
#define NV_NVDISPLAYR2MC_SR_CWDLOG2           6
#define NV_NVDISPLAY2MC_CW                  512
#define NV_NVDISPLAY2MC_TW                    8
#define NV_NVDISPLAY2MC_NBTILECLIENTS         0
#define NV_NVDISPLAYR2MC_SR_PID               0
#define NV_NVDISPLAYR2MC_SR_SID               0
#define NV_NVDISPLAYR2MC_RFIFO_MAXCREDITS     8
#define NV_NVDISPLAYR2MC_COALESCABLE          1
#define NV_NVDISPLAYR2MC_MULTITHREADED        0
#define NV_NVDISPLAYR2MC_TILE                 0
#define NV_NVDISPLAYR2MC_ADRXY                0
#define NV_NVDISPLAYR2MC_SR_LA_SCALING_2    0
#define NV_NVDISPLAYR2MC_SWNAME             nvdisplay
#define NV_NVDISPLAYR2MC_SR_LA_SCALING_2    0
#define NV_DIS22MC_RETPIPECHAIN               1
#define NV_DIS22MC_RETPIPESRC               disiha1
#define NV_DIS22MC_RETPIPEROOT              mcha01
#define NV_DIS22MC_NBCLIENTS                  1
#define NV_DIS22MC_NBRCLIENTS                 1
#define NV_DIS22MC_RCPIDWIDTH                 0
#define NV_DIS22MC_NBWCLIENTS                 0
#define NV_DIS22MC_WRCPIDWIDTH                0
#define NV_DIS22MC_AW                        40
#define NV_DIS22MC_WDLOG2                     5
#define NV_DIS22MC_MW                       512
#define NV_DIS22MC_TW                         8
#define NV_DIS22MC_COALESCABLE                0
#define NV_DIS22MC_MULTITHREADED              0
#define NV_DIS22MC_RVPR                       1
#define NV_NVDISPLAY12MC_PID                  0
#define NV_NVDISPLAY12MC_NBCLIENTS            1
#define NV_NVDISPLAY12MC_NBRCLIENTS           1
#define NV_NVDISPLAY12MC_NBRCLIENTS           1
#define NV_NVDISPLAY12MC_NBWCLIENTS           0
#define NV_NVDISPLAY12MC_NBDIRCLIENTS         0
#define NV_NVDISPLAY12MC_AW                  40
#define NV_NVDISPLAY12MC_WDLOG2               5
#define NV_NVDISPLAY12MC_CWDLOG2              6
#define NV_NVDISPLAYR12MC_SR_CWDLOG2          6
#define NV_NVDISPLAY12MC_CW                 512
#define NV_NVDISPLAY12MC_TW                   8
#define NV_NVDISPLAY12MC_NBTILECLIENTS        0
#define NV_NVDISPLAYR12MC_SR_PID              0
#define NV_NVDISPLAYR12MC_SR_SID              0
#define NV_NVDISPLAYR12MC_RFIFO_MAXCREDITS    8
#define NV_NVDISPLAYR12MC_COALESCABLE         1
#define NV_NVDISPLAYR12MC_MULTITHREADED       0
#define NV_NVDISPLAYR12MC_TILE                0
#define NV_NVDISPLAYR12MC_ADRXY               0
#define NV_NVDISPLAYR12MC_SR_LA_SCALING_2   0
#define NV_NVDISPLAYR12MC_SWNAME            nvdisplay
#define NV_NVDISPLAYR12MC_SR_LA_SCALING_2   0
#define NV_EQOSPC2MC_RETPIPECHAIN             1
#define NV_EQOSPC2MC_RETPIPESRC             ufs2
#define NV_EQOSPC2MC_RETPIPEROOT            mchbb2
#define NV_EQOSPC2MC_NBCLIENTS                2
#define NV_EQOSPC2MC_NBRCLIENTS               1
#define NV_EQOSPC2MC_RCPIDWIDTH               1
#define NV_EQOSPC2MC_NBWCLIENTS               1
#define NV_EQOSPC2MC_WRCPIDWIDTH              1
#define NV_EQOSPC2MC_AW                      40
#define NV_EQOSPC2MC_WDLOG2                   5
#define NV_EQOSPC2MC_MW                     512
#define NV_EQOSPC2MC_BW                      64
#define NV_EQOSPC2MC_TW                       8
#define NV_EQOSPC2MC_COALESCABLE              0
#define NV_EQOSPC2MC_MULTITHREADED            0
#define NV_EQOSPC2MC_RVPR                     1
#define NV_EQOS2MC_PID                        0
#define NV_EQOS2MC_NBCLIENTS                  2
#define NV_EQOS2MC_NBRCLIENTS                 1
#define NV_EQOS2MC_NBRCLIENTS                 1
#define NV_EQOS2MC_NBWCLIENTS                 1
#define NV_EQOS2MC_NBDIRCLIENTS               0
#define NV_EQOS2MC_AW                        40
#define NV_EQOS2MC_WDLOG2                     5
#define NV_EQOS2MC_CWDLOG2                    5
#define NV_EQOSR2MC_SR_CWDLOG2                5
#define NV_EQOSW2MC_SW_CWDLOG2                5
#define NV_EQOS2MC_CW                       256
#define NV_EQOS2MC_BW                        64
#define NV_EQOS2MC_CBW                       32
#define NV_EQOS2MC_TW                         8
#define NV_EQOS2MC_NBTILECLIENTS              0
#define NV_EQOSR2MC_SR_PID                    0
#define NV_EQOSW2MC_SW_PID                    1
#define NV_EQOSR2MC_SR_SID                    0
#define NV_EQOSW2MC_SW_SID                    1
#define NV_EQOSW2MC_SWBW                     32
#define NV_EQOSR2MC_RFIFO_MAXCREDITS          8
#define NV_EQOSW2MC_RFIFO_MAXCREDITS          7
#define NV_EQOSR2MC_COALESCABLE               1
#define NV_EQOSW2MC_COALESCABLE               1
#define NV_EQOSR2MC_MULTITHREADED             0
#define NV_EQOSW2MC_MULTITHREADED             0
#define NV_EQOSR2MC_TILE                      0
#define NV_EQOSR2MC_ADRXY                     0
#define NV_EQOSW2MC_TILE                      0
#define NV_EQOSW2MC_ADRXY                     0
#define NV_EQOSR2MC_SR_LA_SCALING_2         0
#define NV_EQOSW2MC_SW_LA_SCALING_2         0
#define NV_EQOSR2MC_SWNAME                  eqos
#define NV_EQOSW2MC_SWNAME                  eqos
#define NV_EQOSR2MC_SR_LA_SCALING_2         0
#define NV_EQOSW2MC_SW_LA_SCALING_2         0
#define NV_FTOP2MC_NBCLIENTS                  1
#define NV_FTOP2MC_NBRCLIENTS                 0
#define NV_FTOP2MC_RCPIDWIDTH                 0
#define NV_FTOP2MC_NBWCLIENTS                 1
#define NV_FTOP2MC_WRCPIDWIDTH                0
#define NV_FTOP2MC_AW                        40
#define NV_FTOP2MC_WDLOG2                     5
#define NV_FTOP2MC_MW                       512
#define NV_FTOP2MC_BW                        64
#define NV_FTOP2MC_TW                         8
#define NV_FTOP2MC_COALESCABLE                0
#define NV_FTOP2MC_MULTITHREADED              0
#define NV_FTOP2MC_RVPR                       1
#define NV_MPCORE2MC_PID                      0
#define NV_MPCORE2MC_NBCLIENTS                1
#define NV_MPCORE2MC_NBRCLIENTS               0
#define NV_MPCORE2MC_NBRCLIENTS               0
#define NV_MPCORE2MC_NBWCLIENTS               1
#define NV_MPCORE2MC_NBDIRCLIENTS             1
#define NV_MPCORE2MC_AW                      40
#define NV_MPCORE2MC_WDLOG2                   5
#define NV_MPCORE2MC_CWDLOG2                  5
#define NV_MLL_MPCORER2MC_CWDLOG2             5
#define NV_MPCORER2MC_SR_CWDLOG2              5
#define NV_MPCOREW2MC_SW_CWDLOG2              5
#define NV_MPCORE2MC_CW                     256
#define NV_MLL_MPCORER2MC_CW                256
#define NV_MLL_MPCORER2MC_AW                 40
#define NV_MPCORE2MC_BW                      64
#define NV_MPCORE2MC_CBW                     32
#define NV_MPCORE2MC_TW                       8
#define NV_MPCORE2MC_NBTILECLIENTS            0
#define NV_MPCOREW2MC_SW_PID                  0
#define NV_MPCORER2MC_SR_PID                  0
#define NV_MPCOREW2MC_SW_SID                  0
#define NV_MPCORER2MC_SR_SID                  0
#define NV_MPCOREW2MC_RFIFO_MAXCREDITS        7
#define NV_MPCORER2MC_RFIFO_MAXCREDITS        8
#define NV_MPCOREW2MC_COALESCABLE             0
#define NV_MPCORER2MC_COALESCABLE             1
#define NV_MPCOREW2MC_MULTITHREADED           0
#define NV_MPCORER2MC_MULTITHREADED           0
#define NV_MPCOREW2MC_TILE                    0
#define NV_MPCOREW2MC_ADRXY                   0
#define NV_MPCORER2MC_TILE                    0
#define NV_MPCORER2MC_ADRXY                   0
#define NV_MPCOREW2MC_SW_LA_SCALING_2       0
#define NV_MPCORER2MC_SR_LA_SCALING_2       0
#define NV_MPCOREW2MC_SWNAME                mpcore
#define NV_MPCORER2MC_SWNAME                mpcore
#define NV_MPCOREW2MC_SW_LA_SCALING_2       0
#define NV_MPCORER2MC_SR_LA_SCALING_2       0
#define NV_GK2MC_RETPIPECHAIN                 1
#define NV_GK2MC_RETPIPESRC                 mchb0
#define NV_GK2MC_RETPIPEROOT                mcha00
#define NV_GK2MC_NBCLIENTS                    2
#define NV_GK2MC_NBRCLIENTS                   1
#define NV_GK2MC_RCPIDWIDTH                   1
#define NV_GK2MC_NBWCLIENTS                   1
#define NV_GK2MC_WRCPIDWIDTH                  1
#define NV_GK2MC_AW                          40
#define NV_GK2MC_WDLOG2                       5
#define NV_GK2MC_MW                         512
#define NV_GK2MC_BW                          64
#define NV_GK2MC_TW                           8
#define NV_GK2MC_COALESCABLE                  0
#define NV_GK2MC_MULTITHREADED                0
#define NV_GK2MC_RVPR                         1
#define NV_GPU2MC_PID                         0
#define NV_GPU2MC_NBCLIENTS                   2
#define NV_GPU2MC_NBRCLIENTS                  1
#define NV_GPU2MC_NBRCLIENTS                  1
#define NV_GPU2MC_NBWCLIENTS                  1
#define NV_GPU2MC_NBDIRCLIENTS                0
#define NV_GPU2MC_AW                         40
#define NV_GPU2MC_WDLOG2                      5
#define NV_GPU2MC_CWDLOG2                     6
#define NV_GPUSRD2MC_SR_CWDLOG2               6
#define NV_GPUSWR2MC_SW_CWDLOG2               6
#define NV_GPU2MC_CW                        512
#define NV_GPU2MC_BW                         64
#define NV_GPU2MC_CBW                        64
#define NV_GPU2MC_TW                          8
#define NV_GPU2MC_NBTILECLIENTS               0
#define NV_GPUSRD2MC_SR_PID                   0
#define NV_GPUSWR2MC_SW_PID                   1
#define NV_GPUSRD2MC_SR_SID                   0
#define NV_GPUSWR2MC_SW_SID                   1
#define NV_GPUSWR2MC_SWBW                    64
#define NV_GPUSRD2MC_RFIFO_MAXCREDITS         8
#define NV_GPUSWR2MC_RFIFO_MAXCREDITS         7
#define NV_GPUSRD2MC_COALESCABLE              1
#define NV_GPUSWR2MC_COALESCABLE              1
#define NV_GPUSRD2MC_MULTITHREADED            0
#define NV_GPUSWR2MC_MULTITHREADED            0
#define NV_GPUSRD2MC_TILE                     0
#define NV_GPUSRD2MC_ADRXY                    0
#define NV_GPUSWR2MC_TILE                     0
#define NV_GPUSWR2MC_ADRXY                    0
#define NV_GPUSRD2MC_SR_LA_SCALING_2        0
#define NV_GPUSWR2MC_SW_LA_SCALING_2        0
#define NV_GPUSRD2MC_SWNAME                 gpu
#define NV_GPUSWR2MC_SWNAME                 gpu
#define NV_GPUSRD2MC_SR_LA_SCALING_2        0
#define NV_GPUSWR2MC_SW_LA_SCALING_2        0
#define NV_GK22MC_RETPIPECHAIN                1
#define NV_GK22MC_RETPIPESRC                mchb1
#define NV_GK22MC_RETPIPEROOT               mcha01
#define NV_GK22MC_NBCLIENTS                   2
#define NV_GK22MC_NBRCLIENTS                  1
#define NV_GK22MC_RCPIDWIDTH                  1
#define NV_GK22MC_NBWCLIENTS                  1
#define NV_GK22MC_WRCPIDWIDTH                 1
#define NV_GK22MC_AW                         40
#define NV_GK22MC_WDLOG2                      5
#define NV_GK22MC_MW                        512
#define NV_GK22MC_BW                         64
#define NV_GK22MC_TW                          8
#define NV_GK22MC_COALESCABLE                 0
#define NV_GK22MC_MULTITHREADED               0
#define NV_GK22MC_RVPR                        1
#define NV_GPU22MC_PID                        0
#define NV_GPU22MC_NBCLIENTS                  2
#define NV_GPU22MC_NBRCLIENTS                 1
#define NV_GPU22MC_NBRCLIENTS                 1
#define NV_GPU22MC_NBWCLIENTS                 1
#define NV_GPU22MC_NBDIRCLIENTS               0
#define NV_GPU22MC_AW                        40
#define NV_GPU22MC_WDLOG2                     5
#define NV_GPU22MC_CWDLOG2                    6
#define NV_GPUSRD22MC_SR_CWDLOG2              6
#define NV_GPUSWR22MC_SW_CWDLOG2              6
#define NV_GPU22MC_CW                       512
#define NV_GPU22MC_BW                        64
#define NV_GPU22MC_CBW                       64
#define NV_GPU22MC_TW                         8
#define NV_GPU22MC_NBTILECLIENTS              0
#define NV_GPUSRD22MC_SR_PID                  0
#define NV_GPUSWR22MC_SW_PID                  1
#define NV_GPUSRD22MC_SR_SID                  0
#define NV_GPUSWR22MC_SW_SID                  1
#define NV_GPUSWR22MC_SWBW                   64
#define NV_GPUSRD22MC_RFIFO_MAXCREDITS        8
#define NV_GPUSWR22MC_RFIFO_MAXCREDITS        7
#define NV_GPUSRD22MC_COALESCABLE             1
#define NV_GPUSWR22MC_COALESCABLE             1
#define NV_GPUSRD22MC_MULTITHREADED           0
#define NV_GPUSWR22MC_MULTITHREADED           0
#define NV_GPUSRD22MC_TILE                    0
#define NV_GPUSRD22MC_ADRXY                   0
#define NV_GPUSWR22MC_TILE                    0
#define NV_GPUSWR22MC_ADRXY                   0
#define NV_GPUSRD22MC_SR_LA_SCALING_2       0
#define NV_GPUSWR22MC_SW_LA_SCALING_2       0
#define NV_GPUSRD22MC_SWNAME                gpu
#define NV_GPUSWR22MC_SWNAME                gpu
#define NV_GPUSRD22MC_SR_LA_SCALING_2       0
#define NV_GPUSWR22MC_SW_LA_SCALING_2       0
#define NV_HDAPC2MC_RETPIPECHAIN              1
#define NV_HDAPC2MC_RETPIPESRC              sor2
#define NV_HDAPC2MC_RETPIPEROOT             mchbb2
#define NV_HDAPC2MC_NBCLIENTS                 2
#define NV_HDAPC2MC_NBRCLIENTS                1
#define NV_HDAPC2MC_RCPIDWIDTH                1
#define NV_HDAPC2MC_NBWCLIENTS                1
#define NV_HDAPC2MC_WRCPIDWIDTH               1
#define NV_HDAPC2MC_AW                       40
#define NV_HDAPC2MC_WDLOG2                    5
#define NV_HDAPC2MC_MW                      512
#define NV_HDAPC2MC_BW                       64
#define NV_HDAPC2MC_TW                        8
#define NV_HDAPC2MC_COALESCABLE               0
#define NV_HDAPC2MC_MULTITHREADED             0
#define NV_HDAPC2MC_RVPR                      1
#define NV_HDA2MC_PID                         0
#define NV_HDA2MC_NBCLIENTS                   2
#define NV_HDA2MC_NBRCLIENTS                  1
#define NV_HDA2MC_NBRCLIENTS                  1
#define NV_HDA2MC_NBWCLIENTS                  1
#define NV_HDA2MC_NBDIRCLIENTS                0
#define NV_HDA2MC_AW                         40
#define NV_HDA2MC_WDLOG2                      5
#define NV_HDA2MC_CWDLOG2                     5
#define NV_HDAR2MC_SR_CWDLOG2                 5
#define NV_HDAW2MC_SW_CWDLOG2                 5
#define NV_HDA2MC_CW                        256
#define NV_HDA2MC_BW                         64
#define NV_HDA2MC_CBW                        32
#define NV_HDA2MC_TW                          8
#define NV_HDA2MC_NBTILECLIENTS               0
#define NV_HDAR2MC_SR_PID                     0
#define NV_HDAW2MC_SW_PID                     1
#define NV_HDAR2MC_SR_SID                     0
#define NV_HDAW2MC_SW_SID                     1
#define NV_HDAR2MC_RFIFO_MAXCREDITS           8
#define NV_HDAW2MC_RFIFO_MAXCREDITS           7
#define NV_HDAR2MC_COALESCABLE                1
#define NV_HDAW2MC_COALESCABLE                1
#define NV_HDAR2MC_MULTITHREADED              0
#define NV_HDAW2MC_MULTITHREADED              0
#define NV_HDAR2MC_TILE                       0
#define NV_HDAR2MC_ADRXY                      0
#define NV_HDAW2MC_TILE                       0
#define NV_HDAW2MC_ADRXY                      0
#define NV_HDAR2MC_SR_LA_SCALING_2          0
#define NV_HDAW2MC_SW_LA_SCALING_2          0
#define NV_HDAR2MC_SWNAME                   hda
#define NV_HDAW2MC_SWNAME                   hda
#define NV_HDAR2MC_SR_LA_SCALING_2          0
#define NV_HDAW2MC_SW_LA_SCALING_2          0
#define NV_HOST2MC_RETPIPECHAIN               1
#define NV_HOST2MC_RETPIPESRC               hostlpa2
#define NV_HOST2MC_RETPIPEROOT              mchbb2
#define NV_HOST2MC_NBCLIENTS                  1
#define NV_HOST2MC_NBRCLIENTS                 1
#define NV_HOST2MC_RCPIDWIDTH                 0
#define NV_HOST2MC_NBWCLIENTS                 0
#define NV_HOST2MC_WRCPIDWIDTH                0
#define NV_HOST2MC_AW                        40
#define NV_HOST2MC_WDLOG2                     5
#define NV_HOST2MC_MW                       512
#define NV_HOST2MC_TW                         8
#define NV_HOST2MC_COALESCABLE                0
#define NV_HOST2MC_MULTITHREADED              0
#define NV_HOST2MC_RVPR                       1
#define NV_HOST1X2MC_PID                      0
#define NV_HOST1X2MC_NBCLIENTS                1
#define NV_HOST1X2MC_NBRCLIENTS               1
#define NV_HOST1X2MC_NBRCLIENTS               1
#define NV_HOST1X2MC_NBWCLIENTS               0
#define NV_HOST1X2MC_NBDIRCLIENTS             0
#define NV_HOST1X2MC_AW                      40
#define NV_HOST1X2MC_WDLOG2                   5
#define NV_HOST1X2MC_CWDLOG2                  5
#define NV_HOST1XDMAR2MC_SR_CWDLOG2           5
#define NV_HOST1X2MC_CW                     256
#define NV_HOST1X2MC_TW                       8
#define NV_HOST1X2MC_NBTILECLIENTS            0
#define NV_HOST1XDMAR2MC_SR_PID               0
#define NV_HOST1XDMAR2MC_SR_SID               0
#define NV_HOST1XDMAR2MC_RFIFO_MAXCREDITS     8
#define NV_HOST1XDMAR2MC_COALESCABLE          1
#define NV_HOST1XDMAR2MC_MULTITHREADED        0
#define NV_HOST1XDMAR2MC_TILE                 0
#define NV_HOST1XDMAR2MC_ADRXY                0
#define NV_HOST1XDMAR2MC_SR_LA_SCALING_2    0
#define NV_HOST1XDMAR2MC_SWNAME             hc
#define NV_HOST1XDMAR2MC_SR_LA_SCALING_2    0
#define NV_ISP2MC_RETPIPECHAIN                1
#define NV_ISP2MC_RETPIPESRC                ispac2
#define NV_ISP2MC_RETPIPEROOT               mchbb2
#define NV_ISP2MC_NBCLIENTS                   3
#define NV_ISP2MC_NBRCLIENTS                  1
#define NV_ISP2MC_RCPIDWIDTH                  2
#define NV_ISP2MC_NBWCLIENTS                  2
#define NV_ISP2MC_WRCPIDWIDTH                 2
#define NV_ISP2MC_AW                         40
#define NV_ISP2MC_WDLOG2                      5
#define NV_ISP2MC_MW                        512
#define NV_ISP2MC_BW                         64
#define NV_ISP2MC_TW                          8
#define NV_ISP2MC_COALESCABLE                 0
#define NV_ISP2MC_MULTITHREADED               0
#define NV_ISP2MC_RVPR                        1
#define NV_ISP22MC_PID                        0
#define NV_ISP22MC_NBCLIENTS                  3
#define NV_ISP22MC_NBRCLIENTS                 1
#define NV_ISP22MC_NBRCLIENTS                 1
#define NV_ISP22MC_NBWCLIENTS                 2
#define NV_ISP22MC_NBDIRCLIENTS               0
#define NV_ISP22MC_AW                        40
#define NV_ISP22MC_WDLOG2                     5
#define NV_ISP22MC_CWDLOG2                    5
#define NV_ISPRA2MC_SR_CWDLOG2                5
#define NV_ISPWA2MC_SW_CWDLOG2                5
#define NV_ISPWB2MC_SW_CWDLOG2                5
#define NV_ISP22MC_CW                       256
#define NV_ISP22MC_BW                        64
#define NV_ISP22MC_CBW                       32
#define NV_ISP22MC_TW                         8
#define NV_ISP22MC_NBTILECLIENTS              0
#define NV_ISPRA2MC_SR_PID                    0
#define NV_ISPWA2MC_SW_PID                    1
#define NV_ISPWB2MC_SW_PID                    2
#define NV_ISPRA2MC_SR_SID                    0
#define NV_ISPWA2MC_SW_SID                    1
#define NV_ISPWB2MC_SW_SID                    2
#define NV_ISPRA2MC_RFIFO_MAXCREDITS          8
#define NV_ISPWA2MC_RFIFO_MAXCREDITS          7
#define NV_ISPWB2MC_RFIFO_MAXCREDITS          7
#define NV_ISPRA2MC_COALESCABLE               1
#define NV_ISPWA2MC_COALESCABLE               1
#define NV_ISPWB2MC_COALESCABLE               1
#define NV_ISPRA2MC_MULTITHREADED             0
#define NV_ISPWA2MC_MULTITHREADED             0
#define NV_ISPWB2MC_MULTITHREADED             0
#define NV_ISPRA2MC_TILE                      0
#define NV_ISPRA2MC_ADRXY                     0
#define NV_ISPWA2MC_TILE                      0
#define NV_ISPWA2MC_ADRXY                     0
#define NV_ISPWB2MC_TILE                      0
#define NV_ISPWB2MC_ADRXY                     0
#define NV_ISPRA2MC_SR_LA_SCALING_2         0
#define NV_ISPWA2MC_SW_LA_SCALING_2         0
#define NV_ISPWB2MC_SW_LA_SCALING_2         0
#define NV_ISPRA2MC_SWNAME                  isp2
#define NV_ISPWA2MC_SWNAME                  isp2
#define NV_ISPWB2MC_SWNAME                  isp2
#define NV_ISPRA2MC_SR_LA_SCALING_2         0
#define NV_ISPWA2MC_SW_LA_SCALING_2         0
#define NV_ISPWB2MC_SW_LA_SCALING_2         0
#define NV_JPG2MC_RETPIPECHAIN                1
#define NV_JPG2MC_RETPIPESRC                nvjpg2
#define NV_JPG2MC_RETPIPEROOT               mchbb2
#define NV_JPG2MC_NBCLIENTS                   2
#define NV_JPG2MC_NBRCLIENTS                  1
#define NV_JPG2MC_RCPIDWIDTH                  1
#define NV_JPG2MC_NBWCLIENTS                  1
#define NV_JPG2MC_WRCPIDWIDTH                 1
#define NV_JPG2MC_AW                         40
#define NV_JPG2MC_WDLOG2                      5
#define NV_JPG2MC_MW                        512
#define NV_JPG2MC_BW                         64
#define NV_JPG2MC_TW                          8
#define NV_JPG2MC_COALESCABLE                 0
#define NV_JPG2MC_MULTITHREADED               0
#define NV_JPG2MC_RVPR                        1
#define NV_NVJPG2MC_PID                       0
#define NV_NVJPG2MC_NBCLIENTS                 2
#define NV_NVJPG2MC_NBRCLIENTS                1
#define NV_NVJPG2MC_NBRCLIENTS                1
#define NV_NVJPG2MC_NBWCLIENTS                1
#define NV_NVJPG2MC_NBDIRCLIENTS              0
#define NV_NVJPG2MC_AW                       40
#define NV_NVJPG2MC_WDLOG2                    5
#define NV_NVJPG2MC_CWDLOG2                   5
#define NV_NVJPGSRD2MC_SR_CWDLOG2             5
#define NV_NVJPGSWR2MC_SW_CWDLOG2             5
#define NV_NVJPG2MC_CW                      256
#define NV_NVJPG2MC_BW                       64
#define NV_NVJPG2MC_CBW                      32
#define NV_NVJPG2MC_TW                        8
#define NV_NVJPG2MC_NBTILECLIENTS             0
#define NV_NVJPGSRD2MC_SR_PID                 0
#define NV_NVJPGSWR2MC_SW_PID                 1
#define NV_NVJPGSRD2MC_SR_SID                 0
#define NV_NVJPGSWR2MC_SW_SID                 1
#define NV_NVJPGSRD2MC_RFIFO_MAXCREDITS       8
#define NV_NVJPGSWR2MC_RFIFO_MAXCREDITS       7
#define NV_NVJPGSRD2MC_COALESCABLE            1
#define NV_NVJPGSWR2MC_COALESCABLE            1
#define NV_NVJPGSRD2MC_MULTITHREADED          0
#define NV_NVJPGSWR2MC_MULTITHREADED          0
#define NV_NVJPGSRD2MC_TILE                   0
#define NV_NVJPGSRD2MC_ADRXY                  0
#define NV_NVJPGSWR2MC_TILE                   0
#define NV_NVJPGSWR2MC_ADRXY                  0
#define NV_NVJPGSRD2MC_SR_LA_SCALING_2      0
#define NV_NVJPGSWR2MC_SW_LA_SCALING_2      0
#define NV_NVJPGSRD2MC_SWNAME               nvjpg
#define NV_NVJPGSWR2MC_SWNAME               nvjpg
#define NV_NVJPGSRD2MC_SR_LA_SCALING_2      0
#define NV_NVJPGSWR2MC_SW_LA_SCALING_2      0
#define NV_MSE2MC_RETPIPECHAIN                1
#define NV_MSE2MC_RETPIPESRC                nvencb2
#define NV_MSE2MC_RETPIPEROOT               mchbb2
#define NV_MSE2MC_NBCLIENTS                   1
#define NV_MSE2MC_NBRCLIENTS                  1
#define NV_MSE2MC_RCPIDWIDTH                  0
#define NV_MSE2MC_NBWCLIENTS                  0
#define NV_MSE2MC_WRCPIDWIDTH                 0
#define NV_MSE2MC_AW                         40
#define NV_MSE2MC_WDLOG2                      5
#define NV_MSE2MC_MW                        512
#define NV_MSE2MC_TW                          8
#define NV_MSE2MC_COALESCABLE                 0
#define NV_MSE2MC_MULTITHREADED               0
#define NV_MSE2MC_RVPR                        1
#define NV_NVENC2MC_PID                       0
#define NV_NVENC2MC_NBCLIENTS                 1
#define NV_NVENC2MC_NBRCLIENTS                1
#define NV_NVENC2MC_NBRCLIENTS                1
#define NV_NVENC2MC_NBWCLIENTS                0
#define NV_NVENC2MC_NBDIRCLIENTS              0
#define NV_NVENC2MC_AW                       40
#define NV_NVENC2MC_WDLOG2                    5
#define NV_NVENC2MC_CWDLOG2                   5
#define NV_NVENCSRD2MC_SR_CWDLOG2             5
#define NV_NVENC2MC_CW                      256
#define NV_NVENC2MC_TW                        8
#define NV_NVENC2MC_NBTILECLIENTS             0
#define NV_NVENCSRD2MC_SR_PID                 0
#define NV_NVENCSRD2MC_SR_SID                 0
#define NV_NVENCSRD2MC_RFIFO_MAXCREDITS       8
#define NV_NVENCSRD2MC_COALESCABLE            1
#define NV_NVENCSRD2MC_MULTITHREADED          0
#define NV_NVENCSRD2MC_TILE                   0
#define NV_NVENCSRD2MC_ADRXY                  0
#define NV_NVENCSRD2MC_SR_LA_SCALING_2      0
#define NV_NVENCSRD2MC_SWNAME               nvenc
#define NV_NVENCSRD2MC_SR_LA_SCALING_2      0
#define NV_MSE22MC_RETPIPECHAIN               1
#define NV_MSE22MC_RETPIPESRC               nvencb2
#define NV_MSE22MC_RETPIPEROOT              mchbb2
#define NV_MSE22MC_NBCLIENTS                  1
#define NV_MSE22MC_NBRCLIENTS                 0
#define NV_MSE22MC_RCPIDWIDTH                 0
#define NV_MSE22MC_NBWCLIENTS                 1
#define NV_MSE22MC_WRCPIDWIDTH                0
#define NV_MSE22MC_AW                        40
#define NV_MSE22MC_WDLOG2                     5
#define NV_MSE22MC_MW                       512
#define NV_MSE22MC_BW                        64
#define NV_MSE22MC_TW                         8
#define NV_MSE22MC_COALESCABLE                0
#define NV_MSE22MC_MULTITHREADED              0
#define NV_NVENC22MC_PID                      0
#define NV_NVENC22MC_NBCLIENTS                1
#define NV_NVENC22MC_NBRCLIENTS               0
#define NV_NVENC22MC_NBRCLIENTS               0
#define NV_NVENC22MC_NBWCLIENTS               1
#define NV_NVENC22MC_NBDIRCLIENTS             0
#define NV_NVENC22MC_AW                      40
#define NV_NVENC22MC_WDLOG2                   5
#define NV_NVENC22MC_CWDLOG2                  5
#define NV_NVENCSWR2MC_SW_CWDLOG2             5
#define NV_NVENC22MC_CW                     256
#define NV_NVENC22MC_BW                      64
#define NV_NVENC22MC_CBW                     32
#define NV_NVENC22MC_TW                       8
#define NV_NVENC22MC_NBTILECLIENTS            0
#define NV_NVENCSWR2MC_SW_PID                 0
#define NV_NVENCSWR2MC_SW_SID                 0
#define NV_NVENCSWR2MC_RFIFO_MAXCREDITS       7
#define NV_NVENCSWR2MC_COALESCABLE            1
#define NV_NVENCSWR2MC_MULTITHREADED          0
#define NV_NVENCSWR2MC_TILE                   0
#define NV_NVENCSWR2MC_ADRXY                  0
#define NV_NVENCSWR2MC_SW_LA_SCALING_2      0
#define NV_NVENCSWR2MC_SWNAME               nvenc
#define NV_NVENCSWR2MC_SW_LA_SCALING_2      0
#define NV_NIC2MC_RETPIPECHAIN                1
#define NV_NIC2MC_RETPIPESRC                nic12
#define NV_NIC2MC_RETPIPEROOT               mchbb2
#define NV_NIC2MC_NBCLIENTS                   2
#define NV_NIC2MC_NBRCLIENTS                  1
#define NV_NIC2MC_RCPIDWIDTH                  1
#define NV_NIC2MC_NBWCLIENTS                  1
#define NV_NIC2MC_WRCPIDWIDTH                 1
#define NV_NIC2MC_AW                         40
#define NV_NIC2MC_WDLOG2                      5
#define NV_NIC2MC_MW                        512
#define NV_NIC2MC_BW                         64
#define NV_NIC2MC_TW                          8
#define NV_NIC2MC_COALESCABLE                 0
#define NV_NIC2MC_MULTITHREADED               0
#define NV_NIC2MC_RVPR                        1
#define NV_AXIS2MC_PID                        0
#define NV_AXIS2MC_NBCLIENTS                  2
#define NV_AXIS2MC_NBRCLIENTS                 1
#define NV_AXIS2MC_NBRCLIENTS                 1
#define NV_AXIS2MC_NBWCLIENTS                 1
#define NV_AXIS2MC_NBDIRCLIENTS               0
#define NV_AXIS2MC_AW                        40
#define NV_AXIS2MC_WDLOG2                     5
#define NV_AXIS2MC_CWDLOG2                    5
#define NV_AXISR2MC_SR_CWDLOG2                5
#define NV_AXISW2MC_SW_CWDLOG2                5
#define NV_AXIS2MC_CW                       256
#define NV_AXIS2MC_BW                        64
#define NV_AXIS2MC_CBW                       32
#define NV_AXIS2MC_TW                         8
#define NV_AXIS2MC_NBTILECLIENTS              0
#define NV_AXISR2MC_SR_PID                    0
#define NV_AXISW2MC_SW_PID                    1
#define NV_AXISR2MC_SR_SID                    0
#define NV_AXISW2MC_SW_SID                    1
#define NV_AXISW2MC_SWBW                     32
#define NV_AXISR2MC_RFIFO_MAXCREDITS          8
#define NV_AXISW2MC_RFIFO_MAXCREDITS          7
#define NV_AXISR2MC_COALESCABLE               1
#define NV_AXISW2MC_COALESCABLE               1
#define NV_AXISR2MC_MULTITHREADED             0
#define NV_AXISW2MC_MULTITHREADED             0
#define NV_AXISR2MC_TILE                      0
#define NV_AXISR2MC_ADRXY                     0
#define NV_AXISW2MC_TILE                      0
#define NV_AXISW2MC_ADRXY                     0
#define NV_AXISR2MC_SR_LA_SCALING_2         0
#define NV_AXISW2MC_SW_LA_SCALING_2         0
#define NV_AXISR2MC_SWNAME                  axis
#define NV_AXISW2MC_SWNAME                  axis
#define NV_AXISR2MC_SR_LA_SCALING_2         0
#define NV_AXISW2MC_SW_LA_SCALING_2         0
#define NV_NVD2MC_RETPIPECHAIN                1
#define NV_NVD2MC_RETPIPESRC                nvdeca0
#define NV_NVD2MC_RETPIPEROOT               mcha00
#define NV_NVD2MC_NBCLIENTS                   1
#define NV_NVD2MC_NBRCLIENTS                  1
#define NV_NVD2MC_RCPIDWIDTH                  0
#define NV_NVD2MC_NBWCLIENTS                  0
#define NV_NVD2MC_WRCPIDWIDTH                 0
#define NV_NVD2MC_AW                         40
#define NV_NVD2MC_WDLOG2                      5
#define NV_NVD2MC_MW                        512
#define NV_NVD2MC_TW                          8
#define NV_NVD2MC_COALESCABLE                 0
#define NV_NVD2MC_MULTITHREADED               0
#define NV_NVD2MC_RVPR                        1
#define NV_NVDEC2MC_PID                       0
#define NV_NVDEC2MC_NBCLIENTS                 1
#define NV_NVDEC2MC_NBRCLIENTS                1
#define NV_NVDEC2MC_NBRCLIENTS                1
#define NV_NVDEC2MC_NBWCLIENTS                0
#define NV_NVDEC2MC_NBDIRCLIENTS              0
#define NV_NVDEC2MC_AW                       40
#define NV_NVDEC2MC_WDLOG2                    5
#define NV_NVDEC2MC_CWDLOG2                   5
#define NV_NVDECSRD2MC_SR_CWDLOG2             5
#define NV_NVDEC2MC_CW                      256
#define NV_NVDEC2MC_TW                        8
#define NV_NVDEC2MC_NBTILECLIENTS             0
#define NV_NVDECSRD2MC_SR_PID                 0
#define NV_NVDECSRD2MC_SR_SID                 0
#define NV_NVDECSRD2MC_RFIFO_MAXCREDITS       8
#define NV_NVDECSRD2MC_COALESCABLE            1
#define NV_NVDECSRD2MC_MULTITHREADED          0
#define NV_NVDECSRD2MC_TILE                   0
#define NV_NVDECSRD2MC_ADRXY                  0
#define NV_NVDECSRD2MC_SR_LA_SCALING_2      0
#define NV_NVDECSRD2MC_SWNAME               nvdec
#define NV_NVDECSRD2MC_SR_LA_SCALING_2      0
#define NV_NVD22MC_RETPIPECHAIN               1
#define NV_NVD22MC_RETPIPESRC               nvdeca1
#define NV_NVD22MC_RETPIPEROOT              mcha01
#define NV_NVD22MC_NBCLIENTS                  1
#define NV_NVD22MC_NBRCLIENTS                 1
#define NV_NVD22MC_RCPIDWIDTH                 0
#define NV_NVD22MC_NBWCLIENTS                 0
#define NV_NVD22MC_WRCPIDWIDTH                0
#define NV_NVD22MC_AW                        40
#define NV_NVD22MC_WDLOG2                     5
#define NV_NVD22MC_MW                       512
#define NV_NVD22MC_TW                         8
#define NV_NVD22MC_COALESCABLE                0
#define NV_NVD22MC_MULTITHREADED              0
#define NV_NVD22MC_RVPR                       1
#define NV_NVDEC12MC_PID                      0
#define NV_NVDEC12MC_NBCLIENTS                1
#define NV_NVDEC12MC_NBRCLIENTS               1
#define NV_NVDEC12MC_NBRCLIENTS               1
#define NV_NVDEC12MC_NBWCLIENTS               0
#define NV_NVDEC12MC_NBDIRCLIENTS             0
#define NV_NVDEC12MC_AW                      40
#define NV_NVDEC12MC_WDLOG2                   5
#define NV_NVDEC12MC_CWDLOG2                  5
#define NV_NVDECSRD12MC_SR_CWDLOG2            5
#define NV_NVDEC12MC_CW                     256
#define NV_NVDEC12MC_TW                       8
#define NV_NVDEC12MC_NBTILECLIENTS            0
#define NV_NVDECSRD12MC_SR_PID                0
#define NV_NVDECSRD12MC_SR_SID                0
#define NV_NVDECSRD12MC_RFIFO_MAXCREDITS      8
#define NV_NVDECSRD12MC_COALESCABLE           1
#define NV_NVDECSRD12MC_MULTITHREADED         0
#define NV_NVDECSRD12MC_TILE                  0
#define NV_NVDECSRD12MC_ADRXY                 0
#define NV_NVDECSRD12MC_SR_LA_SCALING_2     0
#define NV_NVDECSRD12MC_SWNAME              nvdec
#define NV_NVDECSRD12MC_SR_LA_SCALING_2     0
#define NV_NVD32MC_RETPIPECHAIN               1
#define NV_NVD32MC_RETPIPESRC               nvdeca2
#define NV_NVD32MC_RETPIPEROOT              mchbb2
#define NV_NVD32MC_NBCLIENTS                  1
#define NV_NVD32MC_NBRCLIENTS                 0
#define NV_NVD32MC_RCPIDWIDTH                 0
#define NV_NVD32MC_NBWCLIENTS                 1
#define NV_NVD32MC_WRCPIDWIDTH                0
#define NV_NVD32MC_AW                        40
#define NV_NVD32MC_WDLOG2                     5
#define NV_NVD32MC_MW                       512
#define NV_NVD32MC_BW                        64
#define NV_NVD32MC_TW                         8
#define NV_NVD32MC_COALESCABLE                0
#define NV_NVD32MC_MULTITHREADED              0
#define NV_NVDEC32MC_PID                      0
#define NV_NVDEC32MC_NBCLIENTS                1
#define NV_NVDEC32MC_NBRCLIENTS               0
#define NV_NVDEC32MC_NBRCLIENTS               0
#define NV_NVDEC32MC_NBWCLIENTS               1
#define NV_NVDEC32MC_NBDIRCLIENTS             0
#define NV_NVDEC32MC_AW                      40
#define NV_NVDEC32MC_WDLOG2                   5
#define NV_NVDEC32MC_CWDLOG2                  5
#define NV_NVDECSWR2MC_SW_CWDLOG2             5
#define NV_NVDEC32MC_CW                     256
#define NV_NVDEC32MC_BW                      64
#define NV_NVDEC32MC_CBW                     32
#define NV_NVDEC32MC_TW                       8
#define NV_NVDEC32MC_NBTILECLIENTS            0
#define NV_NVDECSWR2MC_SW_PID                 0
#define NV_NVDECSWR2MC_SW_SID                 0
#define NV_NVDECSWR2MC_RFIFO_MAXCREDITS       7
#define NV_NVDECSWR2MC_COALESCABLE            1
#define NV_NVDECSWR2MC_MULTITHREADED          0
#define NV_NVDECSWR2MC_TILE                   0
#define NV_NVDECSWR2MC_ADRXY                  0
#define NV_NVDECSWR2MC_SW_LA_SCALING_2      0
#define NV_NVDECSWR2MC_SWNAME               nvdec
#define NV_NVDECSWR2MC_SW_LA_SCALING_2      0
#define NV_PCX2MC_RETPIPECHAIN                1
#define NV_PCX2MC_RETPIPESRC                pcx2
#define NV_PCX2MC_RETPIPEROOT               mchbb2
#define NV_PCX2MC_NBCLIENTS                   2
#define NV_PCX2MC_NBRCLIENTS                  1
#define NV_PCX2MC_RCPIDWIDTH                  1
#define NV_PCX2MC_NBWCLIENTS                  1
#define NV_PCX2MC_WRCPIDWIDTH                 1
#define NV_PCX2MC_AW                         40
#define NV_PCX2MC_WDLOG2                      5
#define NV_PCX2MC_MW                        512
#define NV_PCX2MC_BW                         64
#define NV_PCX2MC_TW                          8
#define NV_PCX2MC_COALESCABLE                 0
#define NV_PCX2MC_MULTITHREADED               0
#define NV_PCX2MC_RVPR                        1
#define NV_AFI2MC_PID                         0
#define NV_AFI2MC_NBCLIENTS                   2
#define NV_AFI2MC_NBRCLIENTS                  1
#define NV_AFI2MC_NBRCLIENTS                  1
#define NV_AFI2MC_NBWCLIENTS                  1
#define NV_AFI2MC_NBDIRCLIENTS                0
#define NV_AFI2MC_AW                         40
#define NV_AFI2MC_WDLOG2                      5
#define NV_AFI2MC_CWDLOG2                     5
#define NV_AFIR2MC_SR_CWDLOG2                 5
#define NV_AFIW2MC_SW_CWDLOG2                 5
#define NV_AFI2MC_CW                        256
#define NV_AFI2MC_BW                         64
#define NV_AFI2MC_CBW                        32
#define NV_AFI2MC_TW                          8
#define NV_AFI2MC_NBTILECLIENTS               0
#define NV_AFIR2MC_SR_PID                     0
#define NV_AFIW2MC_SW_PID                     1
#define NV_AFIR2MC_SR_SID                     0
#define NV_AFIW2MC_SW_SID                     1
#define NV_AFIR2MC_RFIFO_MAXCREDITS           8
#define NV_AFIW2MC_RFIFO_MAXCREDITS           7
#define NV_AFIR2MC_COALESCABLE                1
#define NV_AFIW2MC_COALESCABLE                1
#define NV_AFIR2MC_MULTITHREADED              0
#define NV_AFIW2MC_MULTITHREADED              0
#define NV_AFIR2MC_TILE                       0
#define NV_AFIR2MC_ADRXY                      0
#define NV_AFIW2MC_TILE                       0
#define NV_AFIW2MC_ADRXY                      0
#define NV_AFIR2MC_SR_LA_SCALING_2          0
#define NV_AFIW2MC_SW_LA_SCALING_2          0
#define NV_AFIR2MC_SWNAME                   afi
#define NV_AFIW2MC_SWNAME                   afi
#define NV_AFIR2MC_SR_LA_SCALING_2          0
#define NV_AFIW2MC_SW_LA_SCALING_2          0
#define NV_SAX2MC_RETPIPECHAIN                1
#define NV_SAX2MC_RETPIPESRC                sax2
#define NV_SAX2MC_RETPIPEROOT               mchbb2
#define NV_SAX2MC_NBCLIENTS                   2
#define NV_SAX2MC_NBRCLIENTS                  1
#define NV_SAX2MC_RCPIDWIDTH                  1
#define NV_SAX2MC_NBWCLIENTS                  1
#define NV_SAX2MC_WRCPIDWIDTH                 1
#define NV_SAX2MC_AW                         40
#define NV_SAX2MC_WDLOG2                      5
#define NV_SAX2MC_MW                        512
#define NV_SAX2MC_BW                         64
#define NV_SAX2MC_TW                          8
#define NV_SAX2MC_COALESCABLE                 0
#define NV_SAX2MC_MULTITHREADED               0
#define NV_SAX2MC_RVPR                        1
#define NV_SATA2MC_PID                        0
#define NV_SATA2MC_NBCLIENTS                  2
#define NV_SATA2MC_NBRCLIENTS                 1
#define NV_SATA2MC_NBRCLIENTS                 1
#define NV_SATA2MC_NBWCLIENTS                 1
#define NV_SATA2MC_NBDIRCLIENTS               0
#define NV_SATA2MC_AW                        40
#define NV_SATA2MC_WDLOG2                     5
#define NV_SATA2MC_CWDLOG2                    5
#define NV_SATAR2MC_SR_CWDLOG2                5
#define NV_SATAW2MC_SW_CWDLOG2                5
#define NV_SATA2MC_CW                       256
#define NV_SATA2MC_BW                        64
#define NV_SATA2MC_CBW                       32
#define NV_SATA2MC_TW                         8
#define NV_SATA2MC_NBTILECLIENTS              0
#define NV_SATAR2MC_SR_PID                    0
#define NV_SATAW2MC_SW_PID                    1
#define NV_SATAR2MC_SR_SID                    0
#define NV_SATAW2MC_SW_SID                    1
#define NV_SATAR2MC_RFIFO_MAXCREDITS          8
#define NV_SATAW2MC_RFIFO_MAXCREDITS          7
#define NV_SATAR2MC_COALESCABLE               1
#define NV_SATAW2MC_COALESCABLE               1
#define NV_SATAR2MC_MULTITHREADED             0
#define NV_SATAW2MC_MULTITHREADED             0
#define NV_SATAR2MC_TILE                      0
#define NV_SATAR2MC_ADRXY                     0
#define NV_SATAW2MC_TILE                      0
#define NV_SATAW2MC_ADRXY                     0
#define NV_SATAR2MC_SR_LA_SCALING_2         0
#define NV_SATAW2MC_SW_LA_SCALING_2         0
#define NV_SATAR2MC_SWNAME                  sata
#define NV_SATAW2MC_SWNAME                  sata
#define NV_SATAR2MC_SR_LA_SCALING_2         0
#define NV_SATAW2MC_SW_LA_SCALING_2         0
#define NV_SCEDMAPC2MC_RETPIPECHAIN           1
#define NV_SCEDMAPC2MC_RETPIPESRC           sce2
#define NV_SCEDMAPC2MC_RETPIPEROOT          mchbb2
#define NV_SCEDMAPC2MC_NBCLIENTS              2
#define NV_SCEDMAPC2MC_NBRCLIENTS             1
#define NV_SCEDMAPC2MC_RCPIDWIDTH             1
#define NV_SCEDMAPC2MC_NBWCLIENTS             1
#define NV_SCEDMAPC2MC_WRCPIDWIDTH            1
#define NV_SCEDMAPC2MC_AW                    40
#define NV_SCEDMAPC2MC_WDLOG2                 5
#define NV_SCEDMAPC2MC_MW                   512
#define NV_SCEDMAPC2MC_BW                    64
#define NV_SCEDMAPC2MC_TW                     8
#define NV_SCEDMAPC2MC_COALESCABLE            0
#define NV_SCEDMAPC2MC_MULTITHREADED          0
#define NV_SCEDMAPC2MC_RVPR                   1
#define NV_SCEDMA2MC_PID                      0
#define NV_SCEDMA2MC_NBCLIENTS                2
#define NV_SCEDMA2MC_NBRCLIENTS               1
#define NV_SCEDMA2MC_NBRCLIENTS               1
#define NV_SCEDMA2MC_NBWCLIENTS               1
#define NV_SCEDMA2MC_NBDIRCLIENTS             0
#define NV_SCEDMA2MC_AW                      40
#define NV_SCEDMA2MC_WDLOG2                   5
#define NV_SCEDMA2MC_CWDLOG2                  5
#define NV_SCEDMAR2MC_SR_CWDLOG2              5
#define NV_SCEDMAW2MC_SW_CWDLOG2              5
#define NV_SCEDMA2MC_CW                     256
#define NV_SCEDMA2MC_BW                      64
#define NV_SCEDMA2MC_CBW                     32
#define NV_SCEDMA2MC_TW                       8
#define NV_SCEDMA2MC_NBTILECLIENTS            0
#define NV_SCEDMAR2MC_SR_PID                  0
#define NV_SCEDMAW2MC_SW_PID                  1
#define NV_SCEDMAR2MC_SR_SID                  0
#define NV_SCEDMAW2MC_SW_SID                  1
#define NV_SCEDMAW2MC_SWBW                   32
#define NV_SCEDMAR2MC_RFIFO_MAXCREDITS        8
#define NV_SCEDMAW2MC_RFIFO_MAXCREDITS        7
#define NV_SCEDMAR2MC_COALESCABLE             1
#define NV_SCEDMAW2MC_COALESCABLE             1
#define NV_SCEDMAR2MC_MULTITHREADED           0
#define NV_SCEDMAW2MC_MULTITHREADED           0
#define NV_SCEDMAR2MC_TILE                    0
#define NV_SCEDMAR2MC_ADRXY                   0
#define NV_SCEDMAW2MC_TILE                    0
#define NV_SCEDMAW2MC_ADRXY                   0
#define NV_SCEDMAR2MC_SR_LA_SCALING_2       0
#define NV_SCEDMAW2MC_SW_LA_SCALING_2       0
#define NV_SCEDMAR2MC_SWNAME                sce
#define NV_SCEDMAW2MC_SWNAME                sce
#define NV_SCEDMAR2MC_SR_LA_SCALING_2       0
#define NV_SCEDMAW2MC_SW_LA_SCALING_2       0
#define NV_SCEPC2MC_RETPIPECHAIN              1
#define NV_SCEPC2MC_RETPIPESRC              sce2
#define NV_SCEPC2MC_RETPIPEROOT             mchbb2
#define NV_SCEPC2MC_NBCLIENTS                 2
#define NV_SCEPC2MC_NBRCLIENTS                1
#define NV_SCEPC2MC_RCPIDWIDTH                1
#define NV_SCEPC2MC_NBWCLIENTS                1
#define NV_SCEPC2MC_WRCPIDWIDTH               1
#define NV_SCEPC2MC_AW                       40
#define NV_SCEPC2MC_WDLOG2                    5
#define NV_SCEPC2MC_MW                      512
#define NV_SCEPC2MC_BW                       64
#define NV_SCEPC2MC_TW                        8
#define NV_SCEPC2MC_COALESCABLE               0
#define NV_SCEPC2MC_MULTITHREADED             0
#define NV_SCEPC2MC_RVPR                      1
#define NV_SCE2MC_PID                         0
#define NV_SCE2MC_NBCLIENTS                   2
#define NV_SCE2MC_NBRCLIENTS                  1
#define NV_SCE2MC_NBRCLIENTS                  1
#define NV_SCE2MC_NBWCLIENTS                  1
#define NV_SCE2MC_NBDIRCLIENTS                0
#define NV_SCE2MC_AW                         40
#define NV_SCE2MC_WDLOG2                      5
#define NV_SCE2MC_CWDLOG2                     5
#define NV_SCER2MC_SR_CWDLOG2                 5
#define NV_SCEW2MC_SW_CWDLOG2                 5
#define NV_SCE2MC_CW                        256
#define NV_SCE2MC_BW                         64
#define NV_SCE2MC_CBW                        32
#define NV_SCE2MC_TW                          8
#define NV_SCE2MC_NBTILECLIENTS               0
#define NV_SCER2MC_SR_PID                     0
#define NV_SCEW2MC_SW_PID                     1
#define NV_SCER2MC_SR_SID                     0
#define NV_SCEW2MC_SW_SID                     1
#define NV_SCEW2MC_SWBW                      32
#define NV_SCER2MC_RFIFO_MAXCREDITS           8
#define NV_SCEW2MC_RFIFO_MAXCREDITS           7
#define NV_SCER2MC_COALESCABLE                1
#define NV_SCEW2MC_COALESCABLE                1
#define NV_SCER2MC_MULTITHREADED              0
#define NV_SCEW2MC_MULTITHREADED              0
#define NV_SCER2MC_TILE                       0
#define NV_SCER2MC_ADRXY                      0
#define NV_SCEW2MC_TILE                       0
#define NV_SCEW2MC_ADRXY                      0
#define NV_SCER2MC_SR_LA_SCALING_2          0
#define NV_SCEW2MC_SW_LA_SCALING_2          0
#define NV_SCER2MC_SWNAME                   sce
#define NV_SCEW2MC_SWNAME                   sce
#define NV_SCER2MC_SR_LA_SCALING_2          0
#define NV_SCEW2MC_SW_LA_SCALING_2          0
#define NV_SD2MC_RETPIPECHAIN                 1
#define NV_SD2MC_RETPIPESRC                 grttb22
#define NV_SD2MC_RETPIPEROOT                mchbb2
#define NV_SD2MC_NBCLIENTS                    2
#define NV_SD2MC_NBRCLIENTS                   1
#define NV_SD2MC_RCPIDWIDTH                   1
#define NV_SD2MC_NBWCLIENTS                   1
#define NV_SD2MC_WRCPIDWIDTH                  1
#define NV_SD2MC_AW                          40
#define NV_SD2MC_WDLOG2                       5
#define NV_SD2MC_MW                         512
#define NV_SD2MC_BW                          64
#define NV_SD2MC_TW                           8
#define NV_SD2MC_COALESCABLE                  0
#define NV_SD2MC_MULTITHREADED                0
#define NV_SD2MC_RVPR                         1
#define NV_SDMMC2MC_PID                       0
#define NV_SDMMC2MC_NBCLIENTS                 2
#define NV_SDMMC2MC_NBRCLIENTS                1
#define NV_SDMMC2MC_NBRCLIENTS                1
#define NV_SDMMC2MC_NBWCLIENTS                1
#define NV_SDMMC2MC_NBDIRCLIENTS              0
#define NV_SDMMC2MC_AW                       40
#define NV_SDMMC2MC_WDLOG2                    5
#define NV_SDMMC2MC_CWDLOG2                   5
#define NV_SDMMCR2MC_SR_CWDLOG2               5
#define NV_SDMMCW2MC_SW_CWDLOG2               5
#define NV_SDMMC2MC_CW                      256
#define NV_SDMMC2MC_BW                       64
#define NV_SDMMC2MC_CBW                      32
#define NV_SDMMC2MC_TW                        8
#define NV_SDMMC2MC_NBTILECLIENTS             0
#define NV_SDMMCR2MC_SR_PID                   0
#define NV_SDMMCW2MC_SW_PID                   1
#define NV_SDMMCR2MC_SR_SID                   0
#define NV_SDMMCW2MC_SW_SID                   1
#define NV_SDMMCR2MC_RFIFO_MAXCREDITS         8
#define NV_SDMMCW2MC_RFIFO_MAXCREDITS         7
#define NV_SDMMCR2MC_COALESCABLE              1
#define NV_SDMMCW2MC_COALESCABLE              1
#define NV_SDMMCR2MC_MULTITHREADED            0
#define NV_SDMMCW2MC_MULTITHREADED            0
#define NV_SDMMCR2MC_TILE                     0
#define NV_SDMMCR2MC_ADRXY                    0
#define NV_SDMMCW2MC_TILE                     0
#define NV_SDMMCW2MC_ADRXY                    0
#define NV_SDMMCR2MC_SR_LA_SCALING_2        0
#define NV_SDMMCW2MC_SW_LA_SCALING_2        0
#define NV_SDMMCR2MC_SWNAME                 sdmmc3a
#define NV_SDMMCW2MC_SWNAME                 sdmmc3a
#define NV_SDMMCR2MC_SR_LA_SCALING_2        0
#define NV_SDMMCW2MC_SW_LA_SCALING_2        0
#define NV_SDM2MC_RETPIPECHAIN                1
#define NV_SDM2MC_RETPIPESRC                grtta12
#define NV_SDM2MC_RETPIPEROOT               mchbb2
#define NV_SDM2MC_NBCLIENTS                   4
#define NV_SDM2MC_NBRCLIENTS                  2
#define NV_SDM2MC_RCPIDWIDTH                  2
#define NV_SDM2MC_NBWCLIENTS                  2
#define NV_SDM2MC_WRCPIDWIDTH                 2
#define NV_SDM2MC_AW                         40
#define NV_SDM2MC_WDLOG2                      5
#define NV_SDM2MC_MW                        512
#define NV_SDM2MC_BW                         64
#define NV_SDM2MC_TW                          8
#define NV_SDM2MC_COALESCABLE                 0
#define NV_SDM2MC_MULTITHREADED               0
#define NV_SDM2MC_RVPR                        2
#define NV_SDMMCA2MC_PID                      0
#define NV_SDMMCA2MC_NBCLIENTS                2
#define NV_SDMMCA2MC_NBRCLIENTS               1
#define NV_SDMMCA2MC_NBRCLIENTS               1
#define NV_SDMMCA2MC_NBWCLIENTS               1
#define NV_SDMMCA2MC_NBDIRCLIENTS             0
#define NV_SDMMCA2MC_AW                      40
#define NV_SDMMCA2MC_WDLOG2                   5
#define NV_SDMMCA2MC_CWDLOG2                  5
#define NV_SDMMCRA2MC_SR_CWDLOG2              5
#define NV_SDMMCWA2MC_SW_CWDLOG2              5
#define NV_SDMMCA2MC_CW                     256
#define NV_SDMMCA2MC_BW                      64
#define NV_SDMMCA2MC_CBW                     32
#define NV_SDMMCA2MC_TW                       8
#define NV_SDMMCA2MC_NBTILECLIENTS            0
#define NV_SDMMCRA2MC_SR_PID                  0
#define NV_SDMMCWA2MC_SW_PID                  2
#define NV_SDMMCRA2MC_SR_SID                  0
#define NV_SDMMCWA2MC_SW_SID                  1
#define NV_SDMMCRA2MC_RFIFO_MAXCREDITS        8
#define NV_SDMMCWA2MC_RFIFO_MAXCREDITS        7
#define NV_SDMMCRA2MC_COALESCABLE             1
#define NV_SDMMCWA2MC_COALESCABLE             1
#define NV_SDMMCRA2MC_MULTITHREADED           0
#define NV_SDMMCWA2MC_MULTITHREADED           0
#define NV_SDMMCRA2MC_TILE                    0
#define NV_SDMMCRA2MC_ADRXY                   0
#define NV_SDMMCWA2MC_TILE                    0
#define NV_SDMMCWA2MC_ADRXY                   0
#define NV_SDMMCRA2MC_SR_LA_SCALING_2       0
#define NV_SDMMCWA2MC_SW_LA_SCALING_2       0
#define NV_SDMMCRA2MC_SWNAME                sdmmc1a
#define NV_SDMMCWA2MC_SWNAME                sdmmc1a
#define NV_SDMMCRA2MC_SR_LA_SCALING_2       0
#define NV_SDMMCWA2MC_SW_LA_SCALING_2       0
#define NV_SDMMCAB2MC_PID                     1
#define NV_SDMMCAB2MC_NBCLIENTS               2
#define NV_SDMMCAB2MC_NBRCLIENTS              1
#define NV_SDMMCAB2MC_NBRCLIENTS              1
#define NV_SDMMCAB2MC_NBWCLIENTS              1
#define NV_SDMMCAB2MC_NBDIRCLIENTS            0
#define NV_SDMMCAB2MC_AW                     40
#define NV_SDMMCAB2MC_WDLOG2                  5
#define NV_SDMMCAB2MC_CWDLOG2                 5
#define NV_SDMMCRAB2MC_SR_CWDLOG2             5
#define NV_SDMMCWAB2MC_SW_CWDLOG2             5
#define NV_SDMMCAB2MC_CW                    256
#define NV_SDMMCAB2MC_BW                     64
#define NV_SDMMCAB2MC_CBW                    32
#define NV_SDMMCAB2MC_TW                      8
#define NV_SDMMCAB2MC_NBTILECLIENTS           0
#define NV_SDMMCRAB2MC_SR_PID                 1
#define NV_SDMMCWAB2MC_SW_PID                 3
#define NV_SDMMCRAB2MC_SR_SID                 0
#define NV_SDMMCWAB2MC_SW_SID                 1
#define NV_SDMMCRAB2MC_RFIFO_MAXCREDITS       8
#define NV_SDMMCWAB2MC_RFIFO_MAXCREDITS       7
#define NV_SDMMCRAB2MC_COALESCABLE            1
#define NV_SDMMCWAB2MC_COALESCABLE            1
#define NV_SDMMCRAB2MC_MULTITHREADED          0
#define NV_SDMMCWAB2MC_MULTITHREADED          0
#define NV_SDMMCRAB2MC_TILE                   0
#define NV_SDMMCRAB2MC_ADRXY                  0
#define NV_SDMMCWAB2MC_TILE                   0
#define NV_SDMMCWAB2MC_ADRXY                  0
#define NV_SDMMCRAB2MC_SR_LA_SCALING_2      0
#define NV_SDMMCWAB2MC_SW_LA_SCALING_2      0
#define NV_SDMMCRAB2MC_SWNAME               sdmmc4a
#define NV_SDMMCWAB2MC_SWNAME               sdmmc4a
#define NV_SDMMCRAB2MC_SR_LA_SCALING_2      0
#define NV_SDMMCWAB2MC_SW_LA_SCALING_2      0
#define NV_SDM12MC_RETPIPECHAIN               1
#define NV_SDM12MC_RETPIPESRC               grtbb12
#define NV_SDM12MC_RETPIPEROOT              mchbb2
#define NV_SDM12MC_NBCLIENTS                  2
#define NV_SDM12MC_NBRCLIENTS                 1
#define NV_SDM12MC_RCPIDWIDTH                 1
#define NV_SDM12MC_NBWCLIENTS                 1
#define NV_SDM12MC_WRCPIDWIDTH                1
#define NV_SDM12MC_AW                        40
#define NV_SDM12MC_WDLOG2                     5
#define NV_SDM12MC_MW                       512
#define NV_SDM12MC_BW                        64
#define NV_SDM12MC_TW                         8
#define NV_SDM12MC_COALESCABLE                0
#define NV_SDM12MC_MULTITHREADED              0
#define NV_SDM12MC_RVPR                       1
#define NV_SDMMCAA2MC_PID                     0
#define NV_SDMMCAA2MC_NBCLIENTS               2
#define NV_SDMMCAA2MC_NBRCLIENTS              1
#define NV_SDMMCAA2MC_NBRCLIENTS              1
#define NV_SDMMCAA2MC_NBWCLIENTS              1
#define NV_SDMMCAA2MC_NBDIRCLIENTS            0
#define NV_SDMMCAA2MC_AW                     40
#define NV_SDMMCAA2MC_WDLOG2                  5
#define NV_SDMMCAA2MC_CWDLOG2                 5
#define NV_SDMMCRAA2MC_SR_CWDLOG2             5
#define NV_SDMMCWAA2MC_SW_CWDLOG2             5
#define NV_SDMMCAA2MC_CW                    256
#define NV_SDMMCAA2MC_BW                     64
#define NV_SDMMCAA2MC_CBW                    32
#define NV_SDMMCAA2MC_TW                      8
#define NV_SDMMCAA2MC_NBTILECLIENTS           0
#define NV_SDMMCRAA2MC_SR_PID                 0
#define NV_SDMMCWAA2MC_SW_PID                 1
#define NV_SDMMCRAA2MC_SR_SID                 0
#define NV_SDMMCWAA2MC_SW_SID                 1
#define NV_SDMMCRAA2MC_RFIFO_MAXCREDITS       8
#define NV_SDMMCWAA2MC_RFIFO_MAXCREDITS       7
#define NV_SDMMCRAA2MC_COALESCABLE            1
#define NV_SDMMCWAA2MC_COALESCABLE            1
#define NV_SDMMCRAA2MC_MULTITHREADED          0
#define NV_SDMMCWAA2MC_MULTITHREADED          0
#define NV_SDMMCRAA2MC_TILE                   0
#define NV_SDMMCRAA2MC_ADRXY                  0
#define NV_SDMMCWAA2MC_TILE                   0
#define NV_SDMMCWAA2MC_ADRXY                  0
#define NV_SDMMCRAA2MC_SR_LA_SCALING_2      0
#define NV_SDMMCWAA2MC_SW_LA_SCALING_2      0
#define NV_SDMMCRAA2MC_SWNAME               sdmmc2a
#define NV_SDMMCWAA2MC_SWNAME               sdmmc2a
#define NV_SDMMCRAA2MC_SR_LA_SCALING_2      0
#define NV_SDMMCWAA2MC_SW_LA_SCALING_2      0
#define NV_UFSHCPC2MC_RETPIPECHAIN            1
#define NV_UFSHCPC2MC_RETPIPESRC            ufs2
#define NV_UFSHCPC2MC_RETPIPEROOT           mchbb2
#define NV_UFSHCPC2MC_NBCLIENTS               2
#define NV_UFSHCPC2MC_NBRCLIENTS              1
#define NV_UFSHCPC2MC_RCPIDWIDTH              1
#define NV_UFSHCPC2MC_NBWCLIENTS              1
#define NV_UFSHCPC2MC_WRCPIDWIDTH             1
#define NV_UFSHCPC2MC_AW                     40
#define NV_UFSHCPC2MC_WDLOG2                  5
#define NV_UFSHCPC2MC_MW                    512
#define NV_UFSHCPC2MC_BW                     64
#define NV_UFSHCPC2MC_TW                      8
#define NV_UFSHCPC2MC_COALESCABLE             0
#define NV_UFSHCPC2MC_MULTITHREADED           0
#define NV_UFSHCPC2MC_RVPR                    1
#define NV_UFSHC2MC_PID                       0
#define NV_UFSHC2MC_NBCLIENTS                 2
#define NV_UFSHC2MC_NBRCLIENTS                1
#define NV_UFSHC2MC_NBRCLIENTS                1
#define NV_UFSHC2MC_NBWCLIENTS                1
#define NV_UFSHC2MC_NBDIRCLIENTS              0
#define NV_UFSHC2MC_AW                       40
#define NV_UFSHC2MC_WDLOG2                    5
#define NV_UFSHC2MC_CWDLOG2                   5
#define NV_UFSHCR2MC_SR_CWDLOG2               5
#define NV_UFSHCW2MC_SW_CWDLOG2               5
#define NV_UFSHC2MC_CW                      256
#define NV_UFSHC2MC_BW                       64
#define NV_UFSHC2MC_CBW                      32
#define NV_UFSHC2MC_TW                        8
#define NV_UFSHC2MC_NBTILECLIENTS             0
#define NV_UFSHCR2MC_SR_PID                   0
#define NV_UFSHCW2MC_SW_PID                   1
#define NV_UFSHCR2MC_SR_SID                   0
#define NV_UFSHCW2MC_SW_SID                   1
#define NV_UFSHCW2MC_SWBW                    32
#define NV_UFSHCR2MC_RFIFO_MAXCREDITS         8
#define NV_UFSHCW2MC_RFIFO_MAXCREDITS         7
#define NV_UFSHCR2MC_COALESCABLE              1
#define NV_UFSHCW2MC_COALESCABLE              1
#define NV_UFSHCR2MC_MULTITHREADED            0
#define NV_UFSHCW2MC_MULTITHREADED            0
#define NV_UFSHCR2MC_TILE                     0
#define NV_UFSHCR2MC_ADRXY                    0
#define NV_UFSHCW2MC_TILE                     0
#define NV_UFSHCW2MC_ADRXY                    0
#define NV_UFSHCR2MC_SR_LA_SCALING_2        0
#define NV_UFSHCW2MC_SW_LA_SCALING_2        0
#define NV_UFSHCR2MC_SWNAME                 ufshc
#define NV_UFSHCW2MC_SWNAME                 ufshc
#define NV_UFSHCR2MC_SR_LA_SCALING_2        0
#define NV_UFSHCW2MC_SW_LA_SCALING_2        0
#define NV_USBD2MC_RETPIPECHAIN               1
#define NV_USBD2MC_RETPIPESRC               xusbb2
#define NV_USBD2MC_RETPIPEROOT              mchbb2
#define NV_USBD2MC_NBCLIENTS                  2
#define NV_USBD2MC_NBRCLIENTS                 1
#define NV_USBD2MC_RCPIDWIDTH                 1
#define NV_USBD2MC_NBWCLIENTS                 1
#define NV_USBD2MC_WRCPIDWIDTH                1
#define NV_USBD2MC_AW                        40
#define NV_USBD2MC_WDLOG2                     5
#define NV_USBD2MC_MW                       512
#define NV_USBD2MC_BW                        64
#define NV_USBD2MC_TW                         8
#define NV_USBD2MC_COALESCABLE                0
#define NV_USBD2MC_MULTITHREADED              0
#define NV_USBD2MC_RVPR                       1
#define NV_XUSB_DEV2MC_PID                    0
#define NV_XUSB_DEV2MC_NBCLIENTS              2
#define NV_XUSB_DEV2MC_NBRCLIENTS             1
#define NV_XUSB_DEV2MC_NBRCLIENTS             1
#define NV_XUSB_DEV2MC_NBWCLIENTS             1
#define NV_XUSB_DEV2MC_NBDIRCLIENTS           0
#define NV_XUSB_DEV2MC_AW                    40
#define NV_XUSB_DEV2MC_WDLOG2                 5
#define NV_XUSB_DEV2MC_CWDLOG2                5
#define NV_XUSB_DEVR2MC_SR_CWDLOG2            5
#define NV_XUSB_DEVW2MC_SW_CWDLOG2            5
#define NV_XUSB_DEV2MC_CW                   256
#define NV_XUSB_DEV2MC_BW                    64
#define NV_XUSB_DEV2MC_CBW                   32
#define NV_XUSB_DEV2MC_TW                     8
#define NV_XUSB_DEV2MC_NBTILECLIENTS          0
#define NV_XUSB_DEVR2MC_SR_PID                0
#define NV_XUSB_DEVW2MC_SW_PID                1
#define NV_XUSB_DEVR2MC_SR_SID                0
#define NV_XUSB_DEVW2MC_SW_SID                1
#define NV_XUSB_DEVR2MC_RFIFO_MAXCREDITS      8
#define NV_XUSB_DEVW2MC_RFIFO_MAXCREDITS      7
#define NV_XUSB_DEVR2MC_COALESCABLE           1
#define NV_XUSB_DEVW2MC_COALESCABLE           1
#define NV_XUSB_DEVR2MC_MULTITHREADED         0
#define NV_XUSB_DEVW2MC_MULTITHREADED         0
#define NV_XUSB_DEVR2MC_TILE                  0
#define NV_XUSB_DEVR2MC_ADRXY                 0
#define NV_XUSB_DEVW2MC_TILE                  0
#define NV_XUSB_DEVW2MC_ADRXY                 0
#define NV_XUSB_DEVR2MC_SR_LA_SCALING_2     0
#define NV_XUSB_DEVW2MC_SW_LA_SCALING_2     0
#define NV_XUSB_DEVR2MC_SWNAME              xusb_dev
#define NV_XUSB_DEVW2MC_SWNAME              xusb_dev
#define NV_XUSB_DEVR2MC_SR_LA_SCALING_2     0
#define NV_XUSB_DEVW2MC_SW_LA_SCALING_2     0
#define NV_USBX2MC_RETPIPECHAIN               1
#define NV_USBX2MC_RETPIPESRC               xusbc02
#define NV_USBX2MC_RETPIPEROOT              mchbb2
#define NV_USBX2MC_NBCLIENTS                  2
#define NV_USBX2MC_NBRCLIENTS                 1
#define NV_USBX2MC_RCPIDWIDTH                 1
#define NV_USBX2MC_NBWCLIENTS                 1
#define NV_USBX2MC_WRCPIDWIDTH                1
#define NV_USBX2MC_AW                        40
#define NV_USBX2MC_WDLOG2                     5
#define NV_USBX2MC_MW                       512
#define NV_USBX2MC_BW                        64
#define NV_USBX2MC_TW                         8
#define NV_USBX2MC_COALESCABLE                0
#define NV_USBX2MC_MULTITHREADED              0
#define NV_USBX2MC_RVPR                       1
#define NV_XUSB_HOST2MC_PID                   0
#define NV_XUSB_HOST2MC_NBCLIENTS             2
#define NV_XUSB_HOST2MC_NBRCLIENTS            1
#define NV_XUSB_HOST2MC_NBRCLIENTS            1
#define NV_XUSB_HOST2MC_NBWCLIENTS            1
#define NV_XUSB_HOST2MC_NBDIRCLIENTS          0
#define NV_XUSB_HOST2MC_AW                   40
#define NV_XUSB_HOST2MC_WDLOG2                5
#define NV_XUSB_HOST2MC_CWDLOG2               5
#define NV_XUSB_HOSTR2MC_SR_CWDLOG2           5
#define NV_XUSB_HOSTW2MC_SW_CWDLOG2           5
#define NV_XUSB_HOST2MC_CW                  256
#define NV_XUSB_HOST2MC_BW                   64
#define NV_XUSB_HOST2MC_CBW                  32
#define NV_XUSB_HOST2MC_TW                    8
#define NV_XUSB_HOST2MC_NBTILECLIENTS         0
#define NV_XUSB_HOSTR2MC_SR_PID               0
#define NV_XUSB_HOSTW2MC_SW_PID               1
#define NV_XUSB_HOSTR2MC_SR_SID               0
#define NV_XUSB_HOSTW2MC_SW_SID               1
#define NV_XUSB_HOSTR2MC_RFIFO_MAXCREDITS     8
#define NV_XUSB_HOSTW2MC_RFIFO_MAXCREDITS     7
#define NV_XUSB_HOSTR2MC_COALESCABLE          1
#define NV_XUSB_HOSTW2MC_COALESCABLE          1
#define NV_XUSB_HOSTR2MC_MULTITHREADED        0
#define NV_XUSB_HOSTW2MC_MULTITHREADED        0
#define NV_XUSB_HOSTR2MC_TILE                 0
#define NV_XUSB_HOSTR2MC_ADRXY                0
#define NV_XUSB_HOSTW2MC_TILE                 0
#define NV_XUSB_HOSTW2MC_ADRXY                0
#define NV_XUSB_HOSTR2MC_SR_LA_SCALING_2    0
#define NV_XUSB_HOSTW2MC_SW_LA_SCALING_2    0
#define NV_XUSB_HOSTR2MC_SWNAME             xusb_host
#define NV_XUSB_HOSTW2MC_SWNAME             xusb_host
#define NV_XUSB_HOSTR2MC_SR_LA_SCALING_2    0
#define NV_XUSB_HOSTW2MC_SW_LA_SCALING_2    0
#define NV_VE2MC_RETPIPECHAIN                 1
#define NV_VE2MC_RETPIPESRC                 ve3
#define NV_VE2MC_RETPIPEROOT                mchbb3
#define NV_VE2MC_NBCLIENTS                    1
#define NV_VE2MC_NBRCLIENTS                   0
#define NV_VE2MC_RCPIDWIDTH                   0
#define NV_VE2MC_NBWCLIENTS                   1
#define NV_VE2MC_WRCPIDWIDTH                  0
#define NV_VE2MC_AW                          40
#define NV_VE2MC_WDLOG2                       5
#define NV_VE2MC_MW                         512
#define NV_VE2MC_BW                          64
#define NV_VE2MC_TW                           8
#define NV_VE2MC_COALESCABLE                  0
#define NV_VE2MC_MULTITHREADED                0
#define NV_VI22MC_PID                         0
#define NV_VI22MC_NBCLIENTS                   1
#define NV_VI22MC_NBRCLIENTS                  0
#define NV_VI22MC_NBRCLIENTS                  0
#define NV_VI22MC_NBWCLIENTS                  1
#define NV_VI22MC_NBDIRCLIENTS                0
#define NV_VI22MC_AW                         40
#define NV_VI22MC_WDLOG2                      5
#define NV_VI22MC_CWDLOG2                     5
#define NV_VIW2MC_SW_CWDLOG2                  5
#define NV_VI22MC_CW                        256
#define NV_VI22MC_BW                         64
#define NV_VI22MC_CBW                        32
#define NV_VI22MC_TW                          8
#define NV_VI22MC_NBTILECLIENTS               0
#define NV_VIW2MC_SW_PID                      0
#define NV_VIW2MC_SW_SID                      0
#define NV_VIW2MC_RFIFO_MAXCREDITS            7
#define NV_VIW2MC_COALESCABLE                 1
#define NV_VIW2MC_MULTITHREADED               0
#define NV_VIW2MC_TILE                        0
#define NV_VIW2MC_ADRXY                       0
#define NV_VIW2MC_SW_LA_SCALING_2           0
#define NV_VIW2MC_SWNAME                    vi
#define NV_VIW2MC_SW_LA_SCALING_2           0
#define NV_VICPC2MC_RETPIPECHAIN              1
#define NV_VICPC2MC_RETPIPESRC              vica0
#define NV_VICPC2MC_RETPIPEROOT             mcha00
#define NV_VICPC2MC_NBCLIENTS                 1
#define NV_VICPC2MC_NBRCLIENTS                1
#define NV_VICPC2MC_RCPIDWIDTH                0
#define NV_VICPC2MC_NBWCLIENTS                0
#define NV_VICPC2MC_WRCPIDWIDTH               0
#define NV_VICPC2MC_AW                       40
#define NV_VICPC2MC_WDLOG2                    5
#define NV_VICPC2MC_MW                      512
#define NV_VICPC2MC_TW                        8
#define NV_VICPC2MC_COALESCABLE               0
#define NV_VICPC2MC_MULTITHREADED             0
#define NV_VICPC2MC_RVPR                      1
#define NV_VIC2MC_PID                         0
#define NV_VIC2MC_NBCLIENTS                   1
#define NV_VIC2MC_NBRCLIENTS                  1
#define NV_VIC2MC_NBRCLIENTS                  1
#define NV_VIC2MC_NBWCLIENTS                  0
#define NV_VIC2MC_NBDIRCLIENTS                0
#define NV_VIC2MC_AW                         40
#define NV_VIC2MC_WDLOG2                      5
#define NV_VIC2MC_CWDLOG2                     5
#define NV_VICSRD2MC_SR_CWDLOG2               5
#define NV_VIC2MC_CW                        256
#define NV_VIC2MC_TW                          8
#define NV_VIC2MC_NBTILECLIENTS               0
#define NV_VICSRD2MC_SR_PID                   0
#define NV_VICSRD2MC_SR_SID                   0
#define NV_VICSRD2MC_RFIFO_MAXCREDITS         8
#define NV_VICSRD2MC_COALESCABLE              1
#define NV_VICSRD2MC_MULTITHREADED            0
#define NV_VICSRD2MC_TILE                     0
#define NV_VICSRD2MC_ADRXY                    0
#define NV_VICSRD2MC_SR_LA_SCALING_2        0
#define NV_VICSRD2MC_SWNAME                 vic
#define NV_VICSRD2MC_SR_LA_SCALING_2        0
#define NV_VICPC22MC_RETPIPECHAIN             1
#define NV_VICPC22MC_RETPIPESRC             vica1
#define NV_VICPC22MC_RETPIPEROOT            mcha01
#define NV_VICPC22MC_NBCLIENTS                1
#define NV_VICPC22MC_NBRCLIENTS               1
#define NV_VICPC22MC_RCPIDWIDTH               0
#define NV_VICPC22MC_NBWCLIENTS               0
#define NV_VICPC22MC_WRCPIDWIDTH              0
#define NV_VICPC22MC_AW                      40
#define NV_VICPC22MC_WDLOG2                   5
#define NV_VICPC22MC_MW                     512
#define NV_VICPC22MC_TW                       8
#define NV_VICPC22MC_COALESCABLE              0
#define NV_VICPC22MC_MULTITHREADED            0
#define NV_VICPC22MC_RVPR                     1
#define NV_VIC12MC_PID                        0
#define NV_VIC12MC_NBCLIENTS                  1
#define NV_VIC12MC_NBRCLIENTS                 1
#define NV_VIC12MC_NBRCLIENTS                 1
#define NV_VIC12MC_NBWCLIENTS                 0
#define NV_VIC12MC_NBDIRCLIENTS               0
#define NV_VIC12MC_AW                        40
#define NV_VIC12MC_WDLOG2                     5
#define NV_VIC12MC_CWDLOG2                    5
#define NV_VICSRD12MC_SR_CWDLOG2              5
#define NV_VIC12MC_CW                       256
#define NV_VIC12MC_TW                         8
#define NV_VIC12MC_NBTILECLIENTS              0
#define NV_VICSRD12MC_SR_PID                  0
#define NV_VICSRD12MC_SR_SID                  0
#define NV_VICSRD12MC_RFIFO_MAXCREDITS        8
#define NV_VICSRD12MC_COALESCABLE             1
#define NV_VICSRD12MC_MULTITHREADED           0
#define NV_VICSRD12MC_TILE                    0
#define NV_VICSRD12MC_ADRXY                   0
#define NV_VICSRD12MC_SR_LA_SCALING_2       0
#define NV_VICSRD12MC_SWNAME                vic
#define NV_VICSRD12MC_SR_LA_SCALING_2       0
#define NV_VICPC32MC_RETPIPECHAIN             1
#define NV_VICPC32MC_RETPIPESRC             vica3
#define NV_VICPC32MC_RETPIPEROOT            mchbb3
#define NV_VICPC32MC_NBCLIENTS                1
#define NV_VICPC32MC_NBRCLIENTS               0
#define NV_VICPC32MC_RCPIDWIDTH               0
#define NV_VICPC32MC_NBWCLIENTS               1
#define NV_VICPC32MC_WRCPIDWIDTH              0
#define NV_VICPC32MC_AW                      40
#define NV_VICPC32MC_WDLOG2                   5
#define NV_VICPC32MC_MW                     512
#define NV_VICPC32MC_BW                      64
#define NV_VICPC32MC_TW                       8
#define NV_VICPC32MC_COALESCABLE              0
#define NV_VICPC32MC_MULTITHREADED            0
#define NV_VIC32MC_PID                        0
#define NV_VIC32MC_NBCLIENTS                  1
#define NV_VIC32MC_NBRCLIENTS                 0
#define NV_VIC32MC_NBRCLIENTS                 0
#define NV_VIC32MC_NBWCLIENTS                 1
#define NV_VIC32MC_NBDIRCLIENTS               0
#define NV_VIC32MC_AW                        40
#define NV_VIC32MC_WDLOG2                     5
#define NV_VIC32MC_CWDLOG2                    5
#define NV_VICSWR2MC_SW_CWDLOG2               5
#define NV_VIC32MC_CW                       256
#define NV_VIC32MC_BW                        64
#define NV_VIC32MC_CBW                       32
#define NV_VIC32MC_TW                         8
#define NV_VIC32MC_NBTILECLIENTS              0
#define NV_VICSWR2MC_SW_PID                   0
#define NV_VICSWR2MC_SW_SID                   0
#define NV_VICSWR2MC_RFIFO_MAXCREDITS         7
#define NV_VICSWR2MC_COALESCABLE              1
#define NV_VICSWR2MC_MULTITHREADED            0
#define NV_VICSWR2MC_TILE                     0
#define NV_VICSWR2MC_ADRXY                    0
#define NV_VICSWR2MC_SW_LA_SCALING_2        0
#define NV_VICSWR2MC_SWNAME                 vic
#define NV_VICSWR2MC_SW_LA_SCALING_2        0
#define NV_PTCR2MC_SR_CWDLOG2                 0
#define  NV_MC_LA_SCALING_2_CLIST
#define  NV_MC_DVFS_SCLIST                   nvdisplay isp2 vi2
#define NV_MCHA002MC_RETPIPECHAIN_RETIME      0
#define NV_MCHA002MC_RETPIPECHAIN_READ        1
#define NV_MCHA002MC_RETPIPECHAIN_RCPIDWIDTH   1
#define NV_MCHA002MC_RETPIPECHAIN_RVPR        1
#define NV_MCHA002MC_RETPIPECHAIN_WRITE       1
#define NV_MCHA002MC_RETPIPECHAIN_WRCPIDWIDTH   1
#define NV_MCHA002MC_RETPIPECHAIN_LEFT_LISTPC
#define NV_MCHA002MC_RETPIPECHAIN_RIGHT_LISTPC dis nvd vicpc
#define NV_MCHA002MC_RETPIPECHAIN_BOTTOM_LISTPC gk
#define NV_MCHA002MC_RETPIPECHAIN_LOCAL_LISTPC
#define NV_MCHA002MC_RETPIPECHAIN_ALL_LISTPC dis nvd vicpc gk
#define NV_MCHA102MC_RETPIPECHAIN_RETIME      1
#define NV_MCHA102MC_RETPIPECHAIN_READ        1
#define NV_MCHA102MC_RETPIPECHAIN_RCPIDWIDTH   1
#define NV_MCHA102MC_RETPIPECHAIN_RVPR        1
#define NV_MCHA102MC_RETPIPECHAIN_WRITE       1
#define NV_MCHA102MC_RETPIPECHAIN_WRCPIDWIDTH   1
#define NV_MCHA102MC_RETPIPECHAIN_LEFT_LISTPC
#define NV_MCHA102MC_RETPIPECHAIN_RIGHT_LISTPC gk
#define NV_MCHA102MC_RETPIPECHAIN_BOTTOM_LISTPC
#define NV_MCHA102MC_RETPIPECHAIN_LOCAL_LISTPC
#define NV_MCHA102MC_RETPIPECHAIN_ALL_LISTPC gk
#define NV_MCHB02MC_RETPIPECHAIN_RETIME       0
#define NV_MCHB02MC_RETPIPECHAIN_READ         1
#define NV_MCHB02MC_RETPIPECHAIN_RCPIDWIDTH   1
#define NV_MCHB02MC_RETPIPECHAIN_RVPR         1
#define NV_MCHB02MC_RETPIPECHAIN_WRITE        1
#define NV_MCHB02MC_RETPIPECHAIN_WRCPIDWIDTH   1
#define NV_MCHB02MC_RETPIPECHAIN_LEFT_LISTPC
#define NV_MCHB02MC_RETPIPECHAIN_RIGHT_LISTPC
#define NV_MCHB02MC_RETPIPECHAIN_BOTTOM_LISTPC
#define NV_MCHB02MC_RETPIPECHAIN_LOCAL_LISTPC gk
#define NV_MCHB02MC_RETPIPECHAIN_ALL_LISTPC gk
#define NV_MCHBB02MC_RETPIPECHAIN_RETIME      1
#define NV_MCHBB02MC_RETPIPECHAIN_READ        1
#define NV_MCHBB02MC_RETPIPECHAIN_RCPIDWIDTH   0
#define NV_MCHBB02MC_RETPIPECHAIN_RVPR        1
#define NV_MCHBB02MC_RETPIPECHAIN_WRITE       0
#define NV_MCHBB02MC_RETPIPECHAIN_WRCPIDWIDTH   0
#define NV_MCHBB02MC_RETPIPECHAIN_LEFT_LISTPC
#define NV_MCHBB02MC_RETPIPECHAIN_RIGHT_LISTPC dis nvd vicpc
#define NV_MCHBB02MC_RETPIPECHAIN_BOTTOM_LISTPC
#define NV_MCHBB02MC_RETPIPECHAIN_LOCAL_LISTPC
#define NV_MCHBB02MC_RETPIPECHAIN_ALL_LISTPC dis nvd vicpc
#define NV_SEC002MC_RETPIPECHAIN_RETIME       1
#define NV_SEC002MC_RETPIPECHAIN_READ         1
#define NV_SEC002MC_RETPIPECHAIN_RCPIDWIDTH   0
#define NV_SEC002MC_RETPIPECHAIN_RVPR         1
#define NV_SEC002MC_RETPIPECHAIN_WRITE        0
#define NV_SEC002MC_RETPIPECHAIN_WRCPIDWIDTH   0
#define NV_SEC002MC_RETPIPECHAIN_LEFT_LISTPC
#define NV_SEC002MC_RETPIPECHAIN_RIGHT_LISTPC dis nvd
#define NV_SEC002MC_RETPIPECHAIN_BOTTOM_LISTPC vicpc
#define NV_SEC002MC_RETPIPECHAIN_LOCAL_LISTPC
#define NV_SEC002MC_RETPIPECHAIN_ALL_LISTPC dis nvd vicpc
#define NV_VICA02MC_RETPIPECHAIN_RETIME       1
#define NV_VICA02MC_RETPIPECHAIN_READ         1
#define NV_VICA02MC_RETPIPECHAIN_RCPIDWIDTH   0
#define NV_VICA02MC_RETPIPECHAIN_RVPR         1
#define NV_VICA02MC_RETPIPECHAIN_WRITE        0
#define NV_VICA02MC_RETPIPECHAIN_WRCPIDWIDTH   0
#define NV_VICA02MC_RETPIPECHAIN_LEFT_LISTPC
#define NV_VICA02MC_RETPIPECHAIN_RIGHT_LISTPC
#define NV_VICA02MC_RETPIPECHAIN_BOTTOM_LISTPC
#define NV_VICA02MC_RETPIPECHAIN_LOCAL_LISTPC vicpc
#define NV_VICA02MC_RETPIPECHAIN_ALL_LISTPC vicpc
#define NV_SEC102MC_RETPIPECHAIN_RETIME       1
#define NV_SEC102MC_RETPIPECHAIN_READ         1
#define NV_SEC102MC_RETPIPECHAIN_RCPIDWIDTH   0
#define NV_SEC102MC_RETPIPECHAIN_RVPR         1
#define NV_SEC102MC_RETPIPECHAIN_WRITE        0
#define NV_SEC102MC_RETPIPECHAIN_WRCPIDWIDTH   0
#define NV_SEC102MC_RETPIPECHAIN_LEFT_LISTPC
#define NV_SEC102MC_RETPIPECHAIN_RIGHT_LISTPC dis
#define NV_SEC102MC_RETPIPECHAIN_BOTTOM_LISTPC nvd
#define NV_SEC102MC_RETPIPECHAIN_LOCAL_LISTPC
#define NV_SEC102MC_RETPIPECHAIN_ALL_LISTPC dis nvd
#define NV_NVDECA02MC_RETPIPECHAIN_RETIME     1
#define NV_NVDECA02MC_RETPIPECHAIN_READ       1
#define NV_NVDECA02MC_RETPIPECHAIN_RCPIDWIDTH   0
#define NV_NVDECA02MC_RETPIPECHAIN_RVPR       1
#define NV_NVDECA02MC_RETPIPECHAIN_WRITE      0
#define NV_NVDECA02MC_RETPIPECHAIN_WRCPIDWIDTH   0
#define NV_NVDECA02MC_RETPIPECHAIN_LEFT_LISTPC
#define NV_NVDECA02MC_RETPIPECHAIN_RIGHT_LISTPC
#define NV_NVDECA02MC_RETPIPECHAIN_BOTTOM_LISTPC
#define NV_NVDECA02MC_RETPIPECHAIN_LOCAL_LISTPC nvd
#define NV_NVDECA02MC_RETPIPECHAIN_ALL_LISTPC nvd
#define NV_NIC02MC_RETPIPECHAIN_RETIME        1
#define NV_NIC02MC_RETPIPECHAIN_READ          1
#define NV_NIC02MC_RETPIPECHAIN_RCPIDWIDTH    0
#define NV_NIC02MC_RETPIPECHAIN_RVPR          1
#define NV_NIC02MC_RETPIPECHAIN_WRITE         0
#define NV_NIC02MC_RETPIPECHAIN_WRCPIDWIDTH   0
#define NV_NIC02MC_RETPIPECHAIN_LEFT_LISTPC
#define NV_NIC02MC_RETPIPECHAIN_RIGHT_LISTPC dis
#define NV_NIC02MC_RETPIPECHAIN_BOTTOM_LISTPC
#define NV_NIC02MC_RETPIPECHAIN_LOCAL_LISTPC
#define NV_NIC02MC_RETPIPECHAIN_ALL_LISTPC  dis
#define NV_DISIHA02MC_RETPIPECHAIN_RETIME     1
#define NV_DISIHA02MC_RETPIPECHAIN_READ       1
#define NV_DISIHA02MC_RETPIPECHAIN_RCPIDWIDTH   0
#define NV_DISIHA02MC_RETPIPECHAIN_RVPR       1
#define NV_DISIHA02MC_RETPIPECHAIN_WRITE      0
#define NV_DISIHA02MC_RETPIPECHAIN_WRCPIDWIDTH   0
#define NV_DISIHA02MC_RETPIPECHAIN_LEFT_LISTPC
#define NV_DISIHA02MC_RETPIPECHAIN_RIGHT_LISTPC
#define NV_DISIHA02MC_RETPIPECHAIN_BOTTOM_LISTPC
#define NV_DISIHA02MC_RETPIPECHAIN_LOCAL_LISTPC dis
#define NV_DISIHA02MC_RETPIPECHAIN_ALL_LISTPC dis
#define NV_MCHA012MC_RETPIPECHAIN_RETIME      0
#define NV_MCHA012MC_RETPIPECHAIN_READ        1
#define NV_MCHA012MC_RETPIPECHAIN_RCPIDWIDTH   1
#define NV_MCHA012MC_RETPIPECHAIN_RVPR        1
#define NV_MCHA012MC_RETPIPECHAIN_WRITE       1
#define NV_MCHA012MC_RETPIPECHAIN_WRCPIDWIDTH   1
#define NV_MCHA012MC_RETPIPECHAIN_LEFT_LISTPC
#define NV_MCHA012MC_RETPIPECHAIN_RIGHT_LISTPC dis2 nvd2 vicpc2
#define NV_MCHA012MC_RETPIPECHAIN_BOTTOM_LISTPC gk2
#define NV_MCHA012MC_RETPIPECHAIN_LOCAL_LISTPC
#define NV_MCHA012MC_RETPIPECHAIN_ALL_LISTPC dis2 nvd2 vicpc2 gk2
#define NV_MCHA112MC_RETPIPECHAIN_RETIME      1
#define NV_MCHA112MC_RETPIPECHAIN_READ        1
#define NV_MCHA112MC_RETPIPECHAIN_RCPIDWIDTH   1
#define NV_MCHA112MC_RETPIPECHAIN_RVPR        1
#define NV_MCHA112MC_RETPIPECHAIN_WRITE       1
#define NV_MCHA112MC_RETPIPECHAIN_WRCPIDWIDTH   1
#define NV_MCHA112MC_RETPIPECHAIN_LEFT_LISTPC
#define NV_MCHA112MC_RETPIPECHAIN_RIGHT_LISTPC gk2
#define NV_MCHA112MC_RETPIPECHAIN_BOTTOM_LISTPC
#define NV_MCHA112MC_RETPIPECHAIN_LOCAL_LISTPC
#define NV_MCHA112MC_RETPIPECHAIN_ALL_LISTPC gk2
#define NV_MCHB12MC_RETPIPECHAIN_RETIME       0
#define NV_MCHB12MC_RETPIPECHAIN_READ         1
#define NV_MCHB12MC_RETPIPECHAIN_RCPIDWIDTH   1
#define NV_MCHB12MC_RETPIPECHAIN_RVPR         1
#define NV_MCHB12MC_RETPIPECHAIN_WRITE        1
#define NV_MCHB12MC_RETPIPECHAIN_WRCPIDWIDTH   1
#define NV_MCHB12MC_RETPIPECHAIN_LEFT_LISTPC
#define NV_MCHB12MC_RETPIPECHAIN_RIGHT_LISTPC
#define NV_MCHB12MC_RETPIPECHAIN_BOTTOM_LISTPC
#define NV_MCHB12MC_RETPIPECHAIN_LOCAL_LISTPC gk2
#define NV_MCHB12MC_RETPIPECHAIN_ALL_LISTPC gk2
#define NV_MCHBB12MC_RETPIPECHAIN_RETIME      1
#define NV_MCHBB12MC_RETPIPECHAIN_READ        1
#define NV_MCHBB12MC_RETPIPECHAIN_RCPIDWIDTH   0
#define NV_MCHBB12MC_RETPIPECHAIN_RVPR        1
#define NV_MCHBB12MC_RETPIPECHAIN_WRITE       0
#define NV_MCHBB12MC_RETPIPECHAIN_WRCPIDWIDTH   0
#define NV_MCHBB12MC_RETPIPECHAIN_LEFT_LISTPC
#define NV_MCHBB12MC_RETPIPECHAIN_RIGHT_LISTPC dis2 nvd2 vicpc2
#define NV_MCHBB12MC_RETPIPECHAIN_BOTTOM_LISTPC
#define NV_MCHBB12MC_RETPIPECHAIN_LOCAL_LISTPC
#define NV_MCHBB12MC_RETPIPECHAIN_ALL_LISTPC dis2 nvd2 vicpc2
#define NV_SEC012MC_RETPIPECHAIN_RETIME       1
#define NV_SEC012MC_RETPIPECHAIN_READ         1
#define NV_SEC012MC_RETPIPECHAIN_RCPIDWIDTH   0
#define NV_SEC012MC_RETPIPECHAIN_RVPR         1
#define NV_SEC012MC_RETPIPECHAIN_WRITE        0
#define NV_SEC012MC_RETPIPECHAIN_WRCPIDWIDTH   0
#define NV_SEC012MC_RETPIPECHAIN_LEFT_LISTPC
#define NV_SEC012MC_RETPIPECHAIN_RIGHT_LISTPC dis2 nvd2
#define NV_SEC012MC_RETPIPECHAIN_BOTTOM_LISTPC vicpc2
#define NV_SEC012MC_RETPIPECHAIN_LOCAL_LISTPC
#define NV_SEC012MC_RETPIPECHAIN_ALL_LISTPC dis2 nvd2 vicpc2
#define NV_VICA12MC_RETPIPECHAIN_RETIME       1
#define NV_VICA12MC_RETPIPECHAIN_READ         1
#define NV_VICA12MC_RETPIPECHAIN_RCPIDWIDTH   0
#define NV_VICA12MC_RETPIPECHAIN_RVPR         1
#define NV_VICA12MC_RETPIPECHAIN_WRITE        0
#define NV_VICA12MC_RETPIPECHAIN_WRCPIDWIDTH   0
#define NV_VICA12MC_RETPIPECHAIN_LEFT_LISTPC
#define NV_VICA12MC_RETPIPECHAIN_RIGHT_LISTPC
#define NV_VICA12MC_RETPIPECHAIN_BOTTOM_LISTPC
#define NV_VICA12MC_RETPIPECHAIN_LOCAL_LISTPC vicpc2
#define NV_VICA12MC_RETPIPECHAIN_ALL_LISTPC vicpc2
#define NV_SEC112MC_RETPIPECHAIN_RETIME       1
#define NV_SEC112MC_RETPIPECHAIN_READ         1
#define NV_SEC112MC_RETPIPECHAIN_RCPIDWIDTH   0
#define NV_SEC112MC_RETPIPECHAIN_RVPR         1
#define NV_SEC112MC_RETPIPECHAIN_WRITE        0
#define NV_SEC112MC_RETPIPECHAIN_WRCPIDWIDTH   0
#define NV_SEC112MC_RETPIPECHAIN_LEFT_LISTPC
#define NV_SEC112MC_RETPIPECHAIN_RIGHT_LISTPC dis2
#define NV_SEC112MC_RETPIPECHAIN_BOTTOM_LISTPC nvd2
#define NV_SEC112MC_RETPIPECHAIN_LOCAL_LISTPC
#define NV_SEC112MC_RETPIPECHAIN_ALL_LISTPC dis2 nvd2
#define NV_NVDECA12MC_RETPIPECHAIN_RETIME     1
#define NV_NVDECA12MC_RETPIPECHAIN_READ       1
#define NV_NVDECA12MC_RETPIPECHAIN_RCPIDWIDTH   0
#define NV_NVDECA12MC_RETPIPECHAIN_RVPR       1
#define NV_NVDECA12MC_RETPIPECHAIN_WRITE      0
#define NV_NVDECA12MC_RETPIPECHAIN_WRCPIDWIDTH   0
#define NV_NVDECA12MC_RETPIPECHAIN_LEFT_LISTPC
#define NV_NVDECA12MC_RETPIPECHAIN_RIGHT_LISTPC
#define NV_NVDECA12MC_RETPIPECHAIN_BOTTOM_LISTPC
#define NV_NVDECA12MC_RETPIPECHAIN_LOCAL_LISTPC nvd2
#define NV_NVDECA12MC_RETPIPECHAIN_ALL_LISTPC nvd2
#define NV_NIC12MC_RETPIPECHAIN_RETIME        1
#define NV_NIC12MC_RETPIPECHAIN_READ          1
#define NV_NIC12MC_RETPIPECHAIN_RCPIDWIDTH    0
#define NV_NIC12MC_RETPIPECHAIN_RVPR          1
#define NV_NIC12MC_RETPIPECHAIN_WRITE         0
#define NV_NIC12MC_RETPIPECHAIN_WRCPIDWIDTH   0
#define NV_NIC12MC_RETPIPECHAIN_LEFT_LISTPC
#define NV_NIC12MC_RETPIPECHAIN_RIGHT_LISTPC dis2
#define NV_NIC12MC_RETPIPECHAIN_BOTTOM_LISTPC
#define NV_NIC12MC_RETPIPECHAIN_LOCAL_LISTPC
#define NV_NIC12MC_RETPIPECHAIN_ALL_LISTPC  dis2
#define NV_DISIHA12MC_RETPIPECHAIN_RETIME     1
#define NV_DISIHA12MC_RETPIPECHAIN_READ       1
#define NV_DISIHA12MC_RETPIPECHAIN_RCPIDWIDTH   0
#define NV_DISIHA12MC_RETPIPECHAIN_RVPR       1
#define NV_DISIHA12MC_RETPIPECHAIN_WRITE      0
#define NV_DISIHA12MC_RETPIPECHAIN_WRCPIDWIDTH   0
#define NV_DISIHA12MC_RETPIPECHAIN_LEFT_LISTPC
#define NV_DISIHA12MC_RETPIPECHAIN_RIGHT_LISTPC
#define NV_DISIHA12MC_RETPIPECHAIN_BOTTOM_LISTPC
#define NV_DISIHA12MC_RETPIPECHAIN_LOCAL_LISTPC dis2
#define NV_DISIHA12MC_RETPIPECHAIN_ALL_LISTPC dis2
#define NV_MCHBB22MC_RETPIPECHAIN_RETIME      1
#define NV_MCHBB22MC_RETPIPECHAIN_READ        1
#define NV_MCHBB22MC_RETPIPECHAIN_RCPIDWIDTH   3
#define NV_MCHBB22MC_RETPIPECHAIN_RVPR        1
#define NV_MCHBB22MC_RETPIPECHAIN_WRITE       1
#define NV_MCHBB22MC_RETPIPECHAIN_WRCPIDWIDTH   3
#define NV_MCHBB22MC_RETPIPECHAIN_LEFT_LISTPC
#define NV_MCHBB22MC_RETPIPECHAIN_RIGHT_LISTPC aondmapc aonpc apb apedmapc aud bpmpdmapc bpmppc dfd eqospc hdapc host isp jpg mse mse2 nic nvd3 pcx sax scedmapc scepc sd sdm sdm1 ufshcpc usbd usbx
#define NV_MCHBB22MC_RETPIPECHAIN_BOTTOM_LISTPC
#define NV_MCHBB22MC_RETPIPECHAIN_LOCAL_LISTPC
#define NV_MCHBB22MC_RETPIPECHAIN_ALL_LISTPC aondmapc aonpc apb apedmapc aud bpmpdmapc bpmppc dfd eqospc hdapc host isp jpg mse mse2 nic nvd3 pcx sax scedmapc scepc sd sdm sdm1 ufshcpc usbd usbx
#define NV_SEC022MC_RETPIPECHAIN_RETIME       1
#define NV_SEC022MC_RETPIPECHAIN_READ         1
#define NV_SEC022MC_RETPIPECHAIN_RCPIDWIDTH   3
#define NV_SEC022MC_RETPIPECHAIN_RVPR         1
#define NV_SEC022MC_RETPIPECHAIN_WRITE        1
#define NV_SEC022MC_RETPIPECHAIN_WRCPIDWIDTH   3
#define NV_SEC022MC_RETPIPECHAIN_LEFT_LISTPC
#define NV_SEC022MC_RETPIPECHAIN_RIGHT_LISTPC aondmapc aonpc apb apedmapc aud bpmpdmapc bpmppc eqospc hdapc host isp jpg mse mse2 nic nvd3 pcx sax scedmapc scepc sd sdm sdm1 ufshcpc usbd usbx
#define NV_SEC022MC_RETPIPECHAIN_BOTTOM_LISTPC dfd
#define NV_SEC022MC_RETPIPECHAIN_LOCAL_LISTPC
#define NV_SEC022MC_RETPIPECHAIN_ALL_LISTPC aondmapc aonpc apb apedmapc aud bpmpdmapc bpmppc eqospc hdapc host isp jpg mse mse2 nic nvd3 pcx sax scedmapc scepc sd sdm sdm1 ufshcpc usbd usbx dfd
#define NV_DFD22MC_RETPIPECHAIN_RETIME        0
#define NV_DFD22MC_RETPIPECHAIN_READ          1
#define NV_DFD22MC_RETPIPECHAIN_RCPIDWIDTH    1
#define NV_DFD22MC_RETPIPECHAIN_RVPR          1
#define NV_DFD22MC_RETPIPECHAIN_WRITE         1
#define NV_DFD22MC_RETPIPECHAIN_WRCPIDWIDTH   1
#define NV_DFD22MC_RETPIPECHAIN_LEFT_LISTPC
#define NV_DFD22MC_RETPIPECHAIN_RIGHT_LISTPC
#define NV_DFD22MC_RETPIPECHAIN_BOTTOM_LISTPC
#define NV_DFD22MC_RETPIPECHAIN_LOCAL_LISTPC dfd
#define NV_DFD22MC_RETPIPECHAIN_ALL_LISTPC  dfd
#define NV_SEC122MC_RETPIPECHAIN_RETIME       1
#define NV_SEC122MC_RETPIPECHAIN_READ         1
#define NV_SEC122MC_RETPIPECHAIN_RCPIDWIDTH   3
#define NV_SEC122MC_RETPIPECHAIN_RVPR         1
#define NV_SEC122MC_RETPIPECHAIN_WRITE        1
#define NV_SEC122MC_RETPIPECHAIN_WRCPIDWIDTH   3
#define NV_SEC122MC_RETPIPECHAIN_LEFT_LISTPC
#define NV_SEC122MC_RETPIPECHAIN_RIGHT_LISTPC aondmapc aonpc apedmapc aud bpmpdmapc bpmppc eqospc hdapc host isp jpg mse mse2 nic pcx sax scedmapc scepc sd sdm sdm1 ufshcpc usbd usbx
#define NV_SEC122MC_RETPIPECHAIN_BOTTOM_LISTPC nvd3
#define NV_SEC122MC_RETPIPECHAIN_LOCAL_LISTPC apb
#define NV_SEC122MC_RETPIPECHAIN_ALL_LISTPC aondmapc aonpc apedmapc aud bpmpdmapc bpmppc eqospc hdapc host isp jpg mse mse2 nic pcx sax scedmapc scepc sd sdm sdm1 ufshcpc usbd usbx nvd3 apb
#define NV_NVDECA22MC_RETPIPECHAIN_RETIME     1
#define NV_NVDECA22MC_RETPIPECHAIN_READ       0
#define NV_NVDECA22MC_RETPIPECHAIN_RCPIDWIDTH   0
#define NV_NVDECA22MC_RETPIPECHAIN_RVPR       0
#define NV_NVDECA22MC_RETPIPECHAIN_WRITE      1
#define NV_NVDECA22MC_RETPIPECHAIN_WRCPIDWIDTH   0
#define NV_NVDECA22MC_RETPIPECHAIN_LEFT_LISTPC
#define NV_NVDECA22MC_RETPIPECHAIN_RIGHT_LISTPC
#define NV_NVDECA22MC_RETPIPECHAIN_BOTTOM_LISTPC
#define NV_NVDECA22MC_RETPIPECHAIN_LOCAL_LISTPC nvd3
#define NV_NVDECA22MC_RETPIPECHAIN_ALL_LISTPC nvd3
#define NV_NIC022MC_RETPIPECHAIN_RETIME       0
#define NV_NIC022MC_RETPIPECHAIN_READ         1
#define NV_NIC022MC_RETPIPECHAIN_RCPIDWIDTH   2
#define NV_NIC022MC_RETPIPECHAIN_RVPR         1
#define NV_NIC022MC_RETPIPECHAIN_WRITE        1
#define NV_NIC022MC_RETPIPECHAIN_WRCPIDWIDTH   2
#define NV_NIC022MC_RETPIPECHAIN_LEFT_LISTPC aondmapc aonpc hdapc sd sdm
#define NV_NIC022MC_RETPIPECHAIN_RIGHT_LISTPC apedmapc aud bpmpdmapc bpmppc eqospc host isp jpg mse mse2 nic pcx sax scedmapc scepc sdm1 ufshcpc usbd usbx
#define NV_NIC022MC_RETPIPECHAIN_BOTTOM_LISTPC
#define NV_NIC022MC_RETPIPECHAIN_LOCAL_LISTPC
#define NV_NIC022MC_RETPIPECHAIN_ALL_LISTPC aondmapc aonpc hdapc sd sdm apedmapc aud bpmpdmapc bpmppc eqospc host isp jpg mse mse2 nic pcx sax scedmapc scepc sdm1 ufshcpc usbd usbx
#define NV_NIC222MC_RETPIPECHAIN_RETIME       1
#define NV_NIC222MC_RETPIPECHAIN_READ         1
#define NV_NIC222MC_RETPIPECHAIN_RCPIDWIDTH   2
#define NV_NIC222MC_RETPIPECHAIN_RVPR         1
#define NV_NIC222MC_RETPIPECHAIN_WRITE        1
#define NV_NIC222MC_RETPIPECHAIN_WRCPIDWIDTH   2
#define NV_NIC222MC_RETPIPECHAIN_LEFT_LISTPC
#define NV_NIC222MC_RETPIPECHAIN_RIGHT_LISTPC aondmapc aonpc hdapc sd sdm
#define NV_NIC222MC_RETPIPECHAIN_BOTTOM_LISTPC
#define NV_NIC222MC_RETPIPECHAIN_LOCAL_LISTPC
#define NV_NIC222MC_RETPIPECHAIN_ALL_LISTPC aondmapc aonpc hdapc sd sdm
#define NV_GRTCBB022MC_RETPIPECHAIN_RETIME    1
#define NV_GRTCBB022MC_RETPIPECHAIN_READ      1
#define NV_GRTCBB022MC_RETPIPECHAIN_RCPIDWIDTH   2
#define NV_GRTCBB022MC_RETPIPECHAIN_RVPR      1
#define NV_GRTCBB022MC_RETPIPECHAIN_WRITE     1
#define NV_GRTCBB022MC_RETPIPECHAIN_WRCPIDWIDTH   2
#define NV_GRTCBB022MC_RETPIPECHAIN_LEFT_LISTPC
#define NV_GRTCBB022MC_RETPIPECHAIN_RIGHT_LISTPC aondmapc aonpc hdapc sd sdm
#define NV_GRTCBB022MC_RETPIPECHAIN_BOTTOM_LISTPC
#define NV_GRTCBB022MC_RETPIPECHAIN_LOCAL_LISTPC
#define NV_GRTCBB022MC_RETPIPECHAIN_ALL_LISTPC aondmapc aonpc hdapc sd sdm
#define NV_GRTCBB122MC_RETPIPECHAIN_RETIME    1
#define NV_GRTCBB122MC_RETPIPECHAIN_READ      1
#define NV_GRTCBB122MC_RETPIPECHAIN_RCPIDWIDTH   2
#define NV_GRTCBB122MC_RETPIPECHAIN_RVPR      1
#define NV_GRTCBB122MC_RETPIPECHAIN_WRITE     1
#define NV_GRTCBB122MC_RETPIPECHAIN_WRCPIDWIDTH   2
#define NV_GRTCBB122MC_RETPIPECHAIN_LEFT_LISTPC
#define NV_GRTCBB122MC_RETPIPECHAIN_RIGHT_LISTPC aondmapc aonpc hdapc sd sdm
#define NV_GRTCBB122MC_RETPIPECHAIN_BOTTOM_LISTPC
#define NV_GRTCBB122MC_RETPIPECHAIN_LOCAL_LISTPC
#define NV_GRTCBB122MC_RETPIPECHAIN_ALL_LISTPC aondmapc aonpc hdapc sd sdm
#define NV_GRTCBB222MC_RETPIPECHAIN_RETIME    1
#define NV_GRTCBB222MC_RETPIPECHAIN_READ      1
#define NV_GRTCBB222MC_RETPIPECHAIN_RCPIDWIDTH   2
#define NV_GRTCBB222MC_RETPIPECHAIN_RVPR      1
#define NV_GRTCBB222MC_RETPIPECHAIN_WRITE     1
#define NV_GRTCBB222MC_RETPIPECHAIN_WRCPIDWIDTH   2
#define NV_GRTCBB222MC_RETPIPECHAIN_LEFT_LISTPC
#define NV_GRTCBB222MC_RETPIPECHAIN_RIGHT_LISTPC hdapc
#define NV_GRTCBB222MC_RETPIPECHAIN_BOTTOM_LISTPC aondmapc aonpc sd sdm
#define NV_GRTCBB222MC_RETPIPECHAIN_LOCAL_LISTPC
#define NV_GRTCBB222MC_RETPIPECHAIN_ALL_LISTPC hdapc aondmapc aonpc sd sdm
#define NV_VE22MC_RETPIPECHAIN_RETIME         1
#define NV_VE22MC_RETPIPECHAIN_READ           1
#define NV_VE22MC_RETPIPECHAIN_RCPIDWIDTH     1
#define NV_VE22MC_RETPIPECHAIN_RVPR           1
#define NV_VE22MC_RETPIPECHAIN_WRITE          1
#define NV_VE22MC_RETPIPECHAIN_WRCPIDWIDTH    1
#define NV_VE22MC_RETPIPECHAIN_LEFT_LISTPC
#define NV_VE22MC_RETPIPECHAIN_RIGHT_LISTPC hdapc
#define NV_VE22MC_RETPIPECHAIN_BOTTOM_LISTPC
#define NV_VE22MC_RETPIPECHAIN_LOCAL_LISTPC
#define NV_VE22MC_RETPIPECHAIN_ALL_LISTPC   hdapc
#define NV_SOR22MC_RETPIPECHAIN_RETIME        1
#define NV_SOR22MC_RETPIPECHAIN_READ          1
#define NV_SOR22MC_RETPIPECHAIN_RCPIDWIDTH    1
#define NV_SOR22MC_RETPIPECHAIN_RVPR          1
#define NV_SOR22MC_RETPIPECHAIN_WRITE         1
#define NV_SOR22MC_RETPIPECHAIN_WRCPIDWIDTH   1
#define NV_SOR22MC_RETPIPECHAIN_LEFT_LISTPC
#define NV_SOR22MC_RETPIPECHAIN_RIGHT_LISTPC
#define NV_SOR22MC_RETPIPECHAIN_BOTTOM_LISTPC
#define NV_SOR22MC_RETPIPECHAIN_LOCAL_LISTPC hdapc
#define NV_SOR22MC_RETPIPECHAIN_ALL_LISTPC  hdapc
#define NV_GRTTB022MC_RETPIPECHAIN_RETIME     1
#define NV_GRTTB022MC_RETPIPECHAIN_READ       1
#define NV_GRTTB022MC_RETPIPECHAIN_RCPIDWIDTH   2
#define NV_GRTTB022MC_RETPIPECHAIN_RVPR       1
#define NV_GRTTB022MC_RETPIPECHAIN_WRITE      1
#define NV_GRTTB022MC_RETPIPECHAIN_WRCPIDWIDTH   2
#define NV_GRTTB022MC_RETPIPECHAIN_LEFT_LISTPC
#define NV_GRTTB022MC_RETPIPECHAIN_RIGHT_LISTPC aondmapc aonpc sd sdm
#define NV_GRTTB022MC_RETPIPECHAIN_BOTTOM_LISTPC
#define NV_GRTTB022MC_RETPIPECHAIN_LOCAL_LISTPC
#define NV_GRTTB022MC_RETPIPECHAIN_ALL_LISTPC aondmapc aonpc sd sdm
#define NV_GRTTB122MC_RETPIPECHAIN_RETIME     1
#define NV_GRTTB122MC_RETPIPECHAIN_READ       1
#define NV_GRTTB122MC_RETPIPECHAIN_RCPIDWIDTH   2
#define NV_GRTTB122MC_RETPIPECHAIN_RVPR       1
#define NV_GRTTB122MC_RETPIPECHAIN_WRITE      1
#define NV_GRTTB122MC_RETPIPECHAIN_WRCPIDWIDTH   2
#define NV_GRTTB122MC_RETPIPECHAIN_LEFT_LISTPC
#define NV_GRTTB122MC_RETPIPECHAIN_RIGHT_LISTPC sdm
#define NV_GRTTB122MC_RETPIPECHAIN_BOTTOM_LISTPC aondmapc aonpc sd
#define NV_GRTTB122MC_RETPIPECHAIN_LOCAL_LISTPC
#define NV_GRTTB122MC_RETPIPECHAIN_ALL_LISTPC sdm aondmapc aonpc sd
#define NV_GRTTB222MC_RETPIPECHAIN_RETIME     0
#define NV_GRTTB222MC_RETPIPECHAIN_READ       1
#define NV_GRTTB222MC_RETPIPECHAIN_RCPIDWIDTH   1
#define NV_GRTTB222MC_RETPIPECHAIN_RVPR       1
#define NV_GRTTB222MC_RETPIPECHAIN_WRITE      1
#define NV_GRTTB222MC_RETPIPECHAIN_WRCPIDWIDTH   1
#define NV_GRTTB222MC_RETPIPECHAIN_LEFT_LISTPC
#define NV_GRTTB222MC_RETPIPECHAIN_RIGHT_LISTPC aondmapc aonpc
#define NV_GRTTB222MC_RETPIPECHAIN_BOTTOM_LISTPC
#define NV_GRTTB222MC_RETPIPECHAIN_LOCAL_LISTPC sd
#define NV_GRTTB222MC_RETPIPECHAIN_ALL_LISTPC aondmapc aonpc sd
#define NV_AONPG22MC_RETPIPECHAIN_RETIME      1
#define NV_AONPG22MC_RETPIPECHAIN_READ        1
#define NV_AONPG22MC_RETPIPECHAIN_RCPIDWIDTH   1
#define NV_AONPG22MC_RETPIPECHAIN_RVPR        1
#define NV_AONPG22MC_RETPIPECHAIN_WRITE       1
#define NV_AONPG22MC_RETPIPECHAIN_WRCPIDWIDTH   1
#define NV_AONPG22MC_RETPIPECHAIN_LEFT_LISTPC
#define NV_AONPG22MC_RETPIPECHAIN_RIGHT_LISTPC
#define NV_AONPG22MC_RETPIPECHAIN_BOTTOM_LISTPC
#define NV_AONPG22MC_RETPIPECHAIN_LOCAL_LISTPC aonpc aondmapc
#define NV_AONPG22MC_RETPIPECHAIN_ALL_LISTPC aonpc aondmapc
#define NV_GRTTB322MC_RETPIPECHAIN_RETIME     1
#define NV_GRTTB322MC_RETPIPECHAIN_READ       1
#define NV_GRTTB322MC_RETPIPECHAIN_RCPIDWIDTH   2
#define NV_GRTTB322MC_RETPIPECHAIN_RVPR       1
#define NV_GRTTB322MC_RETPIPECHAIN_WRITE      1
#define NV_GRTTB322MC_RETPIPECHAIN_WRCPIDWIDTH   2
#define NV_GRTTB322MC_RETPIPECHAIN_LEFT_LISTPC
#define NV_GRTTB322MC_RETPIPECHAIN_RIGHT_LISTPC sdm
#define NV_GRTTB322MC_RETPIPECHAIN_BOTTOM_LISTPC
#define NV_GRTTB322MC_RETPIPECHAIN_LOCAL_LISTPC
#define NV_GRTTB322MC_RETPIPECHAIN_ALL_LISTPC sdm
#define NV_GRTTB422MC_RETPIPECHAIN_RETIME     1
#define NV_GRTTB422MC_RETPIPECHAIN_READ       1
#define NV_GRTTB422MC_RETPIPECHAIN_RCPIDWIDTH   2
#define NV_GRTTB422MC_RETPIPECHAIN_RVPR       1
#define NV_GRTTB422MC_RETPIPECHAIN_WRITE      1
#define NV_GRTTB422MC_RETPIPECHAIN_WRCPIDWIDTH   2
#define NV_GRTTB422MC_RETPIPECHAIN_LEFT_LISTPC
#define NV_GRTTB422MC_RETPIPECHAIN_RIGHT_LISTPC sdm
#define NV_GRTTB422MC_RETPIPECHAIN_BOTTOM_LISTPC
#define NV_GRTTB422MC_RETPIPECHAIN_LOCAL_LISTPC
#define NV_GRTTB422MC_RETPIPECHAIN_ALL_LISTPC sdm
#define NV_GRTTB522MC_RETPIPECHAIN_RETIME     1
#define NV_GRTTB522MC_RETPIPECHAIN_READ       1
#define NV_GRTTB522MC_RETPIPECHAIN_RCPIDWIDTH   2
#define NV_GRTTB522MC_RETPIPECHAIN_RVPR       1
#define NV_GRTTB522MC_RETPIPECHAIN_WRITE      1
#define NV_GRTTB522MC_RETPIPECHAIN_WRCPIDWIDTH   2
#define NV_GRTTB522MC_RETPIPECHAIN_LEFT_LISTPC
#define NV_GRTTB522MC_RETPIPECHAIN_RIGHT_LISTPC sdm
#define NV_GRTTB522MC_RETPIPECHAIN_BOTTOM_LISTPC
#define NV_GRTTB522MC_RETPIPECHAIN_LOCAL_LISTPC
#define NV_GRTTB522MC_RETPIPECHAIN_ALL_LISTPC sdm
#define NV_GRTTA022MC_RETPIPECHAIN_RETIME     1
#define NV_GRTTA022MC_RETPIPECHAIN_READ       1
#define NV_GRTTA022MC_RETPIPECHAIN_RCPIDWIDTH   2
#define NV_GRTTA022MC_RETPIPECHAIN_RVPR       1
#define NV_GRTTA022MC_RETPIPECHAIN_WRITE      1
#define NV_GRTTA022MC_RETPIPECHAIN_WRCPIDWIDTH   2
#define NV_GRTTA022MC_RETPIPECHAIN_LEFT_LISTPC
#define NV_GRTTA022MC_RETPIPECHAIN_RIGHT_LISTPC sdm
#define NV_GRTTA022MC_RETPIPECHAIN_BOTTOM_LISTPC
#define NV_GRTTA022MC_RETPIPECHAIN_LOCAL_LISTPC
#define NV_GRTTA022MC_RETPIPECHAIN_ALL_LISTPC sdm
#define NV_GRTTA122MC_RETPIPECHAIN_RETIME     1
#define NV_GRTTA122MC_RETPIPECHAIN_READ       1
#define NV_GRTTA122MC_RETPIPECHAIN_RCPIDWIDTH   2
#define NV_GRTTA122MC_RETPIPECHAIN_RVPR       1
#define NV_GRTTA122MC_RETPIPECHAIN_WRITE      1
#define NV_GRTTA122MC_RETPIPECHAIN_WRCPIDWIDTH   2
#define NV_GRTTA122MC_RETPIPECHAIN_LEFT_LISTPC
#define NV_GRTTA122MC_RETPIPECHAIN_RIGHT_LISTPC
#define NV_GRTTA122MC_RETPIPECHAIN_BOTTOM_LISTPC
#define NV_GRTTA122MC_RETPIPECHAIN_LOCAL_LISTPC sdm
#define NV_GRTTA122MC_RETPIPECHAIN_ALL_LISTPC sdm
#define NV_NIC122MC_RETPIPECHAIN_RETIME       1
#define NV_NIC122MC_RETPIPECHAIN_READ         1
#define NV_NIC122MC_RETPIPECHAIN_RCPIDWIDTH   2
#define NV_NIC122MC_RETPIPECHAIN_RVPR         1
#define NV_NIC122MC_RETPIPECHAIN_WRITE        1
#define NV_NIC122MC_RETPIPECHAIN_WRCPIDWIDTH   2
#define NV_NIC122MC_RETPIPECHAIN_LEFT_LISTPC
#define NV_NIC122MC_RETPIPECHAIN_RIGHT_LISTPC apedmapc aud bpmpdmapc bpmppc eqospc host isp jpg mse mse2 pcx sax scedmapc scepc sdm1 ufshcpc usbd usbx
#define NV_NIC122MC_RETPIPECHAIN_BOTTOM_LISTPC
#define NV_NIC122MC_RETPIPECHAIN_LOCAL_LISTPC nic
#define NV_NIC122MC_RETPIPECHAIN_ALL_LISTPC apedmapc aud bpmpdmapc bpmppc eqospc host isp jpg mse mse2 pcx sax scedmapc scepc sdm1 ufshcpc usbd usbx nic
#define NV_HOSTLPA22MC_RETPIPECHAIN_RETIME    1
#define NV_HOSTLPA22MC_RETPIPECHAIN_READ      1
#define NV_HOSTLPA22MC_RETPIPECHAIN_RCPIDWIDTH   2
#define NV_HOSTLPA22MC_RETPIPECHAIN_RVPR      1
#define NV_HOSTLPA22MC_RETPIPECHAIN_WRITE     1
#define NV_HOSTLPA22MC_RETPIPECHAIN_WRCPIDWIDTH   2
#define NV_HOSTLPA22MC_RETPIPECHAIN_LEFT_LISTPC
#define NV_HOSTLPA22MC_RETPIPECHAIN_RIGHT_LISTPC bpmpdmapc bpmppc scedmapc scepc
#define NV_HOSTLPA22MC_RETPIPECHAIN_BOTTOM_LISTPC apedmapc aud eqospc isp jpg mse mse2 pcx sax sdm1 ufshcpc usbd usbx
#define NV_HOSTLPA22MC_RETPIPECHAIN_LOCAL_LISTPC host
#define NV_HOSTLPA22MC_RETPIPECHAIN_ALL_LISTPC bpmpdmapc bpmppc scedmapc scepc apedmapc aud eqospc isp jpg mse mse2 pcx sax sdm1 ufshcpc usbd usbx host
#define NV_BPMP22MC_RETPIPECHAIN_RETIME       1
#define NV_BPMP22MC_RETPIPECHAIN_READ         1
#define NV_BPMP22MC_RETPIPECHAIN_RCPIDWIDTH   1
#define NV_BPMP22MC_RETPIPECHAIN_RVPR         1
#define NV_BPMP22MC_RETPIPECHAIN_WRITE        1
#define NV_BPMP22MC_RETPIPECHAIN_WRCPIDWIDTH   1
#define NV_BPMP22MC_RETPIPECHAIN_LEFT_LISTPC scedmapc scepc
#define NV_BPMP22MC_RETPIPECHAIN_RIGHT_LISTPC
#define NV_BPMP22MC_RETPIPECHAIN_BOTTOM_LISTPC
#define NV_BPMP22MC_RETPIPECHAIN_LOCAL_LISTPC bpmppc bpmpdmapc
#define NV_BPMP22MC_RETPIPECHAIN_ALL_LISTPC scedmapc scepc bpmppc bpmpdmapc
#define NV_SCE22MC_RETPIPECHAIN_RETIME        1
#define NV_SCE22MC_RETPIPECHAIN_READ          1
#define NV_SCE22MC_RETPIPECHAIN_RCPIDWIDTH    1
#define NV_SCE22MC_RETPIPECHAIN_RVPR          1
#define NV_SCE22MC_RETPIPECHAIN_WRITE         1
#define NV_SCE22MC_RETPIPECHAIN_WRCPIDWIDTH   1
#define NV_SCE22MC_RETPIPECHAIN_LEFT_LISTPC
#define NV_SCE22MC_RETPIPECHAIN_RIGHT_LISTPC
#define NV_SCE22MC_RETPIPECHAIN_BOTTOM_LISTPC
#define NV_SCE22MC_RETPIPECHAIN_LOCAL_LISTPC scepc scedmapc
#define NV_SCE22MC_RETPIPECHAIN_ALL_LISTPC  scepc scedmapc
#define NV_GRTCBA022MC_RETPIPECHAIN_RETIME    1
#define NV_GRTCBA022MC_RETPIPECHAIN_READ      1
#define NV_GRTCBA022MC_RETPIPECHAIN_RCPIDWIDTH   2
#define NV_GRTCBA022MC_RETPIPECHAIN_RVPR      1
#define NV_GRTCBA022MC_RETPIPECHAIN_WRITE     1
#define NV_GRTCBA022MC_RETPIPECHAIN_WRCPIDWIDTH   2
#define NV_GRTCBA022MC_RETPIPECHAIN_LEFT_LISTPC
#define NV_GRTCBA022MC_RETPIPECHAIN_RIGHT_LISTPC apedmapc aud eqospc isp jpg mse mse2 pcx sax sdm1 ufshcpc usbd usbx
#define NV_GRTCBA022MC_RETPIPECHAIN_BOTTOM_LISTPC
#define NV_GRTCBA022MC_RETPIPECHAIN_LOCAL_LISTPC
#define NV_GRTCBA022MC_RETPIPECHAIN_ALL_LISTPC apedmapc aud eqospc isp jpg mse mse2 pcx sax sdm1 ufshcpc usbd usbx
#define NV_GRTCBA122MC_RETPIPECHAIN_RETIME    1
#define NV_GRTCBA122MC_RETPIPECHAIN_READ      1
#define NV_GRTCBA122MC_RETPIPECHAIN_RCPIDWIDTH   2
#define NV_GRTCBA122MC_RETPIPECHAIN_RVPR      1
#define NV_GRTCBA122MC_RETPIPECHAIN_WRITE     1
#define NV_GRTCBA122MC_RETPIPECHAIN_WRCPIDWIDTH   2
#define NV_GRTCBA122MC_RETPIPECHAIN_LEFT_LISTPC
#define NV_GRTCBA122MC_RETPIPECHAIN_RIGHT_LISTPC apedmapc aud eqospc isp mse mse2 pcx sax sdm1 ufshcpc usbd usbx
#define NV_GRTCBA122MC_RETPIPECHAIN_BOTTOM_LISTPC jpg
#define NV_GRTCBA122MC_RETPIPECHAIN_LOCAL_LISTPC
#define NV_GRTCBA122MC_RETPIPECHAIN_ALL_LISTPC apedmapc aud eqospc isp mse mse2 pcx sax sdm1 ufshcpc usbd usbx jpg
#define NV_NVJPG22MC_RETPIPECHAIN_RETIME      1
#define NV_NVJPG22MC_RETPIPECHAIN_READ        1
#define NV_NVJPG22MC_RETPIPECHAIN_RCPIDWIDTH   1
#define NV_NVJPG22MC_RETPIPECHAIN_RVPR        1
#define NV_NVJPG22MC_RETPIPECHAIN_WRITE       1
#define NV_NVJPG22MC_RETPIPECHAIN_WRCPIDWIDTH   1
#define NV_NVJPG22MC_RETPIPECHAIN_LEFT_LISTPC
#define NV_NVJPG22MC_RETPIPECHAIN_RIGHT_LISTPC
#define NV_NVJPG22MC_RETPIPECHAIN_BOTTOM_LISTPC
#define NV_NVJPG22MC_RETPIPECHAIN_LOCAL_LISTPC jpg
#define NV_NVJPG22MC_RETPIPECHAIN_ALL_LISTPC jpg
#define NV_GRTCBA222MC_RETPIPECHAIN_RETIME    0
#define NV_GRTCBA222MC_RETPIPECHAIN_READ      1
#define NV_GRTCBA222MC_RETPIPECHAIN_RCPIDWIDTH   2
#define NV_GRTCBA222MC_RETPIPECHAIN_RVPR      1
#define NV_GRTCBA222MC_RETPIPECHAIN_WRITE     1
#define NV_GRTCBA222MC_RETPIPECHAIN_WRCPIDWIDTH   2
#define NV_GRTCBA222MC_RETPIPECHAIN_LEFT_LISTPC isp
#define NV_GRTCBA222MC_RETPIPECHAIN_RIGHT_LISTPC apedmapc aud eqospc pcx sax sdm1 ufshcpc usbd usbx
#define NV_GRTCBA222MC_RETPIPECHAIN_BOTTOM_LISTPC mse mse2
#define NV_GRTCBA222MC_RETPIPECHAIN_LOCAL_LISTPC
#define NV_GRTCBA222MC_RETPIPECHAIN_ALL_LISTPC isp apedmapc aud eqospc pcx sax sdm1 ufshcpc usbd usbx mse mse2
#define NV_NVENCB22MC_RETPIPECHAIN_RETIME     1
#define NV_NVENCB22MC_RETPIPECHAIN_READ       1
#define NV_NVENCB22MC_RETPIPECHAIN_RCPIDWIDTH   0
#define NV_NVENCB22MC_RETPIPECHAIN_RVPR       1
#define NV_NVENCB22MC_RETPIPECHAIN_WRITE      1
#define NV_NVENCB22MC_RETPIPECHAIN_WRCPIDWIDTH   0
#define NV_NVENCB22MC_RETPIPECHAIN_LEFT_LISTPC
#define NV_NVENCB22MC_RETPIPECHAIN_RIGHT_LISTPC
#define NV_NVENCB22MC_RETPIPECHAIN_BOTTOM_LISTPC
#define NV_NVENCB22MC_RETPIPECHAIN_LOCAL_LISTPC mse mse2
#define NV_NVENCB22MC_RETPIPECHAIN_ALL_LISTPC mse mse2
#define NV_ISPAC22MC_RETPIPECHAIN_RETIME      1
#define NV_ISPAC22MC_RETPIPECHAIN_READ        1
#define NV_ISPAC22MC_RETPIPECHAIN_RCPIDWIDTH   2
#define NV_ISPAC22MC_RETPIPECHAIN_RVPR        1
#define NV_ISPAC22MC_RETPIPECHAIN_WRITE       1
#define NV_ISPAC22MC_RETPIPECHAIN_WRCPIDWIDTH   2
#define NV_ISPAC22MC_RETPIPECHAIN_LEFT_LISTPC
#define NV_ISPAC22MC_RETPIPECHAIN_RIGHT_LISTPC
#define NV_ISPAC22MC_RETPIPECHAIN_BOTTOM_LISTPC
#define NV_ISPAC22MC_RETPIPECHAIN_LOCAL_LISTPC isp
#define NV_ISPAC22MC_RETPIPECHAIN_ALL_LISTPC isp
#define NV_GRTCBA322MC_RETPIPECHAIN_RETIME    1
#define NV_GRTCBA322MC_RETPIPECHAIN_READ      1
#define NV_GRTCBA322MC_RETPIPECHAIN_RCPIDWIDTH   1
#define NV_GRTCBA322MC_RETPIPECHAIN_RVPR      1
#define NV_GRTCBA322MC_RETPIPECHAIN_WRITE     1
#define NV_GRTCBA322MC_RETPIPECHAIN_WRCPIDWIDTH   1
#define NV_GRTCBA322MC_RETPIPECHAIN_LEFT_LISTPC
#define NV_GRTCBA322MC_RETPIPECHAIN_RIGHT_LISTPC apedmapc aud eqospc pcx sax sdm1 ufshcpc usbd usbx
#define NV_GRTCBA322MC_RETPIPECHAIN_BOTTOM_LISTPC
#define NV_GRTCBA322MC_RETPIPECHAIN_LOCAL_LISTPC
#define NV_GRTCBA322MC_RETPIPECHAIN_ALL_LISTPC apedmapc aud eqospc pcx sax sdm1 ufshcpc usbd usbx
#define NV_GRTCBA422MC_RETPIPECHAIN_RETIME    0
#define NV_GRTCBA422MC_RETPIPECHAIN_READ      1
#define NV_GRTCBA422MC_RETPIPECHAIN_RCPIDWIDTH   1
#define NV_GRTCBA422MC_RETPIPECHAIN_RVPR      1
#define NV_GRTCBA422MC_RETPIPECHAIN_WRITE     1
#define NV_GRTCBA422MC_RETPIPECHAIN_WRCPIDWIDTH   1
#define NV_GRTCBA422MC_RETPIPECHAIN_LEFT_LISTPC
#define NV_GRTCBA422MC_RETPIPECHAIN_RIGHT_LISTPC apedmapc aud sdm1 usbd usbx
#define NV_GRTCBA422MC_RETPIPECHAIN_BOTTOM_LISTPC eqospc pcx sax ufshcpc
#define NV_GRTCBA422MC_RETPIPECHAIN_LOCAL_LISTPC
#define NV_GRTCBA422MC_RETPIPECHAIN_ALL_LISTPC apedmapc aud sdm1 usbd usbx eqospc pcx sax ufshcpc
#define NV_PCX22MC_RETPIPECHAIN_RETIME        1
#define NV_PCX22MC_RETPIPECHAIN_READ          1
#define NV_PCX22MC_RETPIPECHAIN_RCPIDWIDTH    1
#define NV_PCX22MC_RETPIPECHAIN_RVPR          1
#define NV_PCX22MC_RETPIPECHAIN_WRITE         1
#define NV_PCX22MC_RETPIPECHAIN_WRCPIDWIDTH   1
#define NV_PCX22MC_RETPIPECHAIN_LEFT_LISTPC
#define NV_PCX22MC_RETPIPECHAIN_RIGHT_LISTPC eqospc ufshcpc
#define NV_PCX22MC_RETPIPECHAIN_BOTTOM_LISTPC sax
#define NV_PCX22MC_RETPIPECHAIN_LOCAL_LISTPC pcx
#define NV_PCX22MC_RETPIPECHAIN_ALL_LISTPC  eqospc ufshcpc sax pcx
#define NV_SAX22MC_RETPIPECHAIN_RETIME        1
#define NV_SAX22MC_RETPIPECHAIN_READ          1
#define NV_SAX22MC_RETPIPECHAIN_RCPIDWIDTH    1
#define NV_SAX22MC_RETPIPECHAIN_RVPR          1
#define NV_SAX22MC_RETPIPECHAIN_WRITE         1
#define NV_SAX22MC_RETPIPECHAIN_WRCPIDWIDTH   1
#define NV_SAX22MC_RETPIPECHAIN_LEFT_LISTPC
#define NV_SAX22MC_RETPIPECHAIN_RIGHT_LISTPC
#define NV_SAX22MC_RETPIPECHAIN_BOTTOM_LISTPC
#define NV_SAX22MC_RETPIPECHAIN_LOCAL_LISTPC sax
#define NV_SAX22MC_RETPIPECHAIN_ALL_LISTPC  sax
#define NV_UFS22MC_RETPIPECHAIN_RETIME        1
#define NV_UFS22MC_RETPIPECHAIN_READ          1
#define NV_UFS22MC_RETPIPECHAIN_RCPIDWIDTH    1
#define NV_UFS22MC_RETPIPECHAIN_RVPR          1
#define NV_UFS22MC_RETPIPECHAIN_WRITE         1
#define NV_UFS22MC_RETPIPECHAIN_WRCPIDWIDTH   1
#define NV_UFS22MC_RETPIPECHAIN_LEFT_LISTPC
#define NV_UFS22MC_RETPIPECHAIN_RIGHT_LISTPC
#define NV_UFS22MC_RETPIPECHAIN_BOTTOM_LISTPC
#define NV_UFS22MC_RETPIPECHAIN_LOCAL_LISTPC eqospc ufshcpc
#define NV_UFS22MC_RETPIPECHAIN_ALL_LISTPC  eqospc ufshcpc
#define NV_GRTCBA522MC_RETPIPECHAIN_RETIME    1
#define NV_GRTCBA522MC_RETPIPECHAIN_READ      1
#define NV_GRTCBA522MC_RETPIPECHAIN_RCPIDWIDTH   1
#define NV_GRTCBA522MC_RETPIPECHAIN_RVPR      1
#define NV_GRTCBA522MC_RETPIPECHAIN_WRITE     1
#define NV_GRTCBA522MC_RETPIPECHAIN_WRCPIDWIDTH   1
#define NV_GRTCBA522MC_RETPIPECHAIN_LEFT_LISTPC
#define NV_GRTCBA522MC_RETPIPECHAIN_RIGHT_LISTPC sdm1 usbd usbx
#define NV_GRTCBA522MC_RETPIPECHAIN_BOTTOM_LISTPC apedmapc aud
#define NV_GRTCBA522MC_RETPIPECHAIN_LOCAL_LISTPC
#define NV_GRTCBA522MC_RETPIPECHAIN_ALL_LISTPC sdm1 usbd usbx apedmapc aud
#define NV_APEC22MC_RETPIPECHAIN_RETIME       1
#define NV_APEC22MC_RETPIPECHAIN_READ         1
#define NV_APEC22MC_RETPIPECHAIN_RCPIDWIDTH   1
#define NV_APEC22MC_RETPIPECHAIN_RVPR         1
#define NV_APEC22MC_RETPIPECHAIN_WRITE        1
#define NV_APEC22MC_RETPIPECHAIN_WRCPIDWIDTH   1
#define NV_APEC22MC_RETPIPECHAIN_LEFT_LISTPC
#define NV_APEC22MC_RETPIPECHAIN_RIGHT_LISTPC apedmapc aud
#define NV_APEC22MC_RETPIPECHAIN_BOTTOM_LISTPC
#define NV_APEC22MC_RETPIPECHAIN_LOCAL_LISTPC
#define NV_APEC22MC_RETPIPECHAIN_ALL_LISTPC apedmapc aud
#define NV_AUD22MC_RETPIPECHAIN_RETIME        1
#define NV_AUD22MC_RETPIPECHAIN_READ          1
#define NV_AUD22MC_RETPIPECHAIN_RCPIDWIDTH    1
#define NV_AUD22MC_RETPIPECHAIN_RVPR          1
#define NV_AUD22MC_RETPIPECHAIN_WRITE         1
#define NV_AUD22MC_RETPIPECHAIN_WRCPIDWIDTH   1
#define NV_AUD22MC_RETPIPECHAIN_LEFT_LISTPC
#define NV_AUD22MC_RETPIPECHAIN_RIGHT_LISTPC
#define NV_AUD22MC_RETPIPECHAIN_BOTTOM_LISTPC
#define NV_AUD22MC_RETPIPECHAIN_LOCAL_LISTPC aud apedmapc
#define NV_AUD22MC_RETPIPECHAIN_ALL_LISTPC  aud apedmapc
#define NV_GRTCBA622MC_RETPIPECHAIN_RETIME    0
#define NV_GRTCBA622MC_RETPIPECHAIN_READ      1
#define NV_GRTCBA622MC_RETPIPECHAIN_RCPIDWIDTH   1
#define NV_GRTCBA622MC_RETPIPECHAIN_RVPR      1
#define NV_GRTCBA622MC_RETPIPECHAIN_WRITE     1
#define NV_GRTCBA622MC_RETPIPECHAIN_WRCPIDWIDTH   1
#define NV_GRTCBA622MC_RETPIPECHAIN_LEFT_LISTPC
#define NV_GRTCBA622MC_RETPIPECHAIN_RIGHT_LISTPC sdm1
#define NV_GRTCBA622MC_RETPIPECHAIN_BOTTOM_LISTPC usbd usbx
#define NV_GRTCBA622MC_RETPIPECHAIN_LOCAL_LISTPC
#define NV_GRTCBA622MC_RETPIPECHAIN_ALL_LISTPC sdm1 usbd usbx
#define NV_XUSBA22MC_RETPIPECHAIN_RETIME      1
#define NV_XUSBA22MC_RETPIPECHAIN_READ        1
#define NV_XUSBA22MC_RETPIPECHAIN_RCPIDWIDTH   1
#define NV_XUSBA22MC_RETPIPECHAIN_RVPR        1
#define NV_XUSBA22MC_RETPIPECHAIN_WRITE       1
#define NV_XUSBA22MC_RETPIPECHAIN_WRCPIDWIDTH   1
#define NV_XUSBA22MC_RETPIPECHAIN_LEFT_LISTPC
#define NV_XUSBA22MC_RETPIPECHAIN_RIGHT_LISTPC usbd usbx
#define NV_XUSBA22MC_RETPIPECHAIN_BOTTOM_LISTPC
#define NV_XUSBA22MC_RETPIPECHAIN_LOCAL_LISTPC
#define NV_XUSBA22MC_RETPIPECHAIN_ALL_LISTPC usbd usbx
#define NV_XUSBC022MC_RETPIPECHAIN_RETIME     1
#define NV_XUSBC022MC_RETPIPECHAIN_READ       1
#define NV_XUSBC022MC_RETPIPECHAIN_RCPIDWIDTH   1
#define NV_XUSBC022MC_RETPIPECHAIN_RVPR       1
#define NV_XUSBC022MC_RETPIPECHAIN_WRITE      1
#define NV_XUSBC022MC_RETPIPECHAIN_WRCPIDWIDTH   1
#define NV_XUSBC022MC_RETPIPECHAIN_LEFT_LISTPC
#define NV_XUSBC022MC_RETPIPECHAIN_RIGHT_LISTPC usbd
#define NV_XUSBC022MC_RETPIPECHAIN_BOTTOM_LISTPC
#define NV_XUSBC022MC_RETPIPECHAIN_LOCAL_LISTPC usbx
#define NV_XUSBC022MC_RETPIPECHAIN_ALL_LISTPC usbd usbx
#define NV_XUSBC122MC_RETPIPECHAIN_RETIME     1
#define NV_XUSBC122MC_RETPIPECHAIN_READ       1
#define NV_XUSBC122MC_RETPIPECHAIN_RCPIDWIDTH   1
#define NV_XUSBC122MC_RETPIPECHAIN_RVPR       1
#define NV_XUSBC122MC_RETPIPECHAIN_WRITE      1
#define NV_XUSBC122MC_RETPIPECHAIN_WRCPIDWIDTH   1
#define NV_XUSBC122MC_RETPIPECHAIN_LEFT_LISTPC
#define NV_XUSBC122MC_RETPIPECHAIN_RIGHT_LISTPC usbd
#define NV_XUSBC122MC_RETPIPECHAIN_BOTTOM_LISTPC
#define NV_XUSBC122MC_RETPIPECHAIN_LOCAL_LISTPC
#define NV_XUSBC122MC_RETPIPECHAIN_ALL_LISTPC usbd
#define NV_XUSBB22MC_RETPIPECHAIN_RETIME      0
#define NV_XUSBB22MC_RETPIPECHAIN_READ        1
#define NV_XUSBB22MC_RETPIPECHAIN_RCPIDWIDTH   1
#define NV_XUSBB22MC_RETPIPECHAIN_RVPR        1
#define NV_XUSBB22MC_RETPIPECHAIN_WRITE       1
#define NV_XUSBB22MC_RETPIPECHAIN_WRCPIDWIDTH   1
#define NV_XUSBB22MC_RETPIPECHAIN_LEFT_LISTPC
#define NV_XUSBB22MC_RETPIPECHAIN_RIGHT_LISTPC
#define NV_XUSBB22MC_RETPIPECHAIN_BOTTOM_LISTPC
#define NV_XUSBB22MC_RETPIPECHAIN_LOCAL_LISTPC usbd
#define NV_XUSBB22MC_RETPIPECHAIN_ALL_LISTPC usbd
#define NV_GRTBB022MC_RETPIPECHAIN_RETIME     1
#define NV_GRTBB022MC_RETPIPECHAIN_READ       1
#define NV_GRTBB022MC_RETPIPECHAIN_RCPIDWIDTH   1
#define NV_GRTBB022MC_RETPIPECHAIN_RVPR       1
#define NV_GRTBB022MC_RETPIPECHAIN_WRITE      1
#define NV_GRTBB022MC_RETPIPECHAIN_WRCPIDWIDTH   1
#define NV_GRTBB022MC_RETPIPECHAIN_LEFT_LISTPC
#define NV_GRTBB022MC_RETPIPECHAIN_RIGHT_LISTPC sdm1
#define NV_GRTBB022MC_RETPIPECHAIN_BOTTOM_LISTPC
#define NV_GRTBB022MC_RETPIPECHAIN_LOCAL_LISTPC
#define NV_GRTBB022MC_RETPIPECHAIN_ALL_LISTPC sdm1
#define NV_GRTBB122MC_RETPIPECHAIN_RETIME     1
#define NV_GRTBB122MC_RETPIPECHAIN_READ       1
#define NV_GRTBB122MC_RETPIPECHAIN_RCPIDWIDTH   1
#define NV_GRTBB122MC_RETPIPECHAIN_RVPR       1
#define NV_GRTBB122MC_RETPIPECHAIN_WRITE      1
#define NV_GRTBB122MC_RETPIPECHAIN_WRCPIDWIDTH   1
#define NV_GRTBB122MC_RETPIPECHAIN_LEFT_LISTPC
#define NV_GRTBB122MC_RETPIPECHAIN_RIGHT_LISTPC
#define NV_GRTBB122MC_RETPIPECHAIN_BOTTOM_LISTPC
#define NV_GRTBB122MC_RETPIPECHAIN_LOCAL_LISTPC sdm1
#define NV_GRTBB122MC_RETPIPECHAIN_ALL_LISTPC sdm1
#define NV_MCHBB32MC_RETPIPECHAIN_RETIME      1
#define NV_MCHBB32MC_RETPIPECHAIN_READ        0
#define NV_MCHBB32MC_RETPIPECHAIN_RCPIDWIDTH   0
#define NV_MCHBB32MC_RETPIPECHAIN_RVPR        0
#define NV_MCHBB32MC_RETPIPECHAIN_WRITE       1
#define NV_MCHBB32MC_RETPIPECHAIN_WRCPIDWIDTH   0
#define NV_MCHBB32MC_RETPIPECHAIN_LEFT_LISTPC
#define NV_MCHBB32MC_RETPIPECHAIN_RIGHT_LISTPC ve vicpc3
#define NV_MCHBB32MC_RETPIPECHAIN_BOTTOM_LISTPC
#define NV_MCHBB32MC_RETPIPECHAIN_LOCAL_LISTPC
#define NV_MCHBB32MC_RETPIPECHAIN_ALL_LISTPC ve vicpc3
#define NV_SEC032MC_RETPIPECHAIN_RETIME       1
#define NV_SEC032MC_RETPIPECHAIN_READ         0
#define NV_SEC032MC_RETPIPECHAIN_RCPIDWIDTH   0
#define NV_SEC032MC_RETPIPECHAIN_RVPR         0
#define NV_SEC032MC_RETPIPECHAIN_WRITE        1
#define NV_SEC032MC_RETPIPECHAIN_WRCPIDWIDTH   0
#define NV_SEC032MC_RETPIPECHAIN_LEFT_LISTPC
#define NV_SEC032MC_RETPIPECHAIN_RIGHT_LISTPC ve
#define NV_SEC032MC_RETPIPECHAIN_BOTTOM_LISTPC vicpc3
#define NV_SEC032MC_RETPIPECHAIN_LOCAL_LISTPC
#define NV_SEC032MC_RETPIPECHAIN_ALL_LISTPC ve vicpc3
#define NV_VICA32MC_RETPIPECHAIN_RETIME       1
#define NV_VICA32MC_RETPIPECHAIN_READ         0
#define NV_VICA32MC_RETPIPECHAIN_RCPIDWIDTH   0
#define NV_VICA32MC_RETPIPECHAIN_RVPR         0
#define NV_VICA32MC_RETPIPECHAIN_WRITE        1
#define NV_VICA32MC_RETPIPECHAIN_WRCPIDWIDTH   0
#define NV_VICA32MC_RETPIPECHAIN_LEFT_LISTPC
#define NV_VICA32MC_RETPIPECHAIN_RIGHT_LISTPC
#define NV_VICA32MC_RETPIPECHAIN_BOTTOM_LISTPC
#define NV_VICA32MC_RETPIPECHAIN_LOCAL_LISTPC vicpc3
#define NV_VICA32MC_RETPIPECHAIN_ALL_LISTPC vicpc3
#define NV_SEC132MC_RETPIPECHAIN_RETIME       1
#define NV_SEC132MC_RETPIPECHAIN_READ         0
#define NV_SEC132MC_RETPIPECHAIN_RCPIDWIDTH   0
#define NV_SEC132MC_RETPIPECHAIN_RVPR         0
#define NV_SEC132MC_RETPIPECHAIN_WRITE        1
#define NV_SEC132MC_RETPIPECHAIN_WRCPIDWIDTH   0
#define NV_SEC132MC_RETPIPECHAIN_LEFT_LISTPC
#define NV_SEC132MC_RETPIPECHAIN_RIGHT_LISTPC ve
#define NV_SEC132MC_RETPIPECHAIN_BOTTOM_LISTPC
#define NV_SEC132MC_RETPIPECHAIN_LOCAL_LISTPC
#define NV_SEC132MC_RETPIPECHAIN_ALL_LISTPC ve
#define NV_NIC32MC_RETPIPECHAIN_RETIME        1
#define NV_NIC32MC_RETPIPECHAIN_READ          0
#define NV_NIC32MC_RETPIPECHAIN_RCPIDWIDTH    0
#define NV_NIC32MC_RETPIPECHAIN_RVPR          0
#define NV_NIC32MC_RETPIPECHAIN_WRITE         1
#define NV_NIC32MC_RETPIPECHAIN_WRCPIDWIDTH   0
#define NV_NIC32MC_RETPIPECHAIN_LEFT_LISTPC
#define NV_NIC32MC_RETPIPECHAIN_RIGHT_LISTPC ve
#define NV_NIC32MC_RETPIPECHAIN_BOTTOM_LISTPC
#define NV_NIC32MC_RETPIPECHAIN_LOCAL_LISTPC
#define NV_NIC32MC_RETPIPECHAIN_ALL_LISTPC  ve
#define NV_GRTCBB032MC_RETPIPECHAIN_RETIME    1
#define NV_GRTCBB032MC_RETPIPECHAIN_READ      0
#define NV_GRTCBB032MC_RETPIPECHAIN_RCPIDWIDTH   0
#define NV_GRTCBB032MC_RETPIPECHAIN_RVPR      0
#define NV_GRTCBB032MC_RETPIPECHAIN_WRITE     1
#define NV_GRTCBB032MC_RETPIPECHAIN_WRCPIDWIDTH   0
#define NV_GRTCBB032MC_RETPIPECHAIN_LEFT_LISTPC
#define NV_GRTCBB032MC_RETPIPECHAIN_RIGHT_LISTPC ve
#define NV_GRTCBB032MC_RETPIPECHAIN_BOTTOM_LISTPC
#define NV_GRTCBB032MC_RETPIPECHAIN_LOCAL_LISTPC
#define NV_GRTCBB032MC_RETPIPECHAIN_ALL_LISTPC ve
#define NV_GRTCBB132MC_RETPIPECHAIN_RETIME    1
#define NV_GRTCBB132MC_RETPIPECHAIN_READ      0
#define NV_GRTCBB132MC_RETPIPECHAIN_RCPIDWIDTH   0
#define NV_GRTCBB132MC_RETPIPECHAIN_RVPR      0
#define NV_GRTCBB132MC_RETPIPECHAIN_WRITE     1
#define NV_GRTCBB132MC_RETPIPECHAIN_WRCPIDWIDTH   0
#define NV_GRTCBB132MC_RETPIPECHAIN_LEFT_LISTPC
#define NV_GRTCBB132MC_RETPIPECHAIN_RIGHT_LISTPC ve
#define NV_GRTCBB132MC_RETPIPECHAIN_BOTTOM_LISTPC
#define NV_GRTCBB132MC_RETPIPECHAIN_LOCAL_LISTPC
#define NV_GRTCBB132MC_RETPIPECHAIN_ALL_LISTPC ve
#define NV_GRTCBB232MC_RETPIPECHAIN_RETIME    1
#define NV_GRTCBB232MC_RETPIPECHAIN_READ      0
#define NV_GRTCBB232MC_RETPIPECHAIN_RCPIDWIDTH   0
#define NV_GRTCBB232MC_RETPIPECHAIN_RVPR      0
#define NV_GRTCBB232MC_RETPIPECHAIN_WRITE     1
#define NV_GRTCBB232MC_RETPIPECHAIN_WRCPIDWIDTH   0
#define NV_GRTCBB232MC_RETPIPECHAIN_LEFT_LISTPC
#define NV_GRTCBB232MC_RETPIPECHAIN_RIGHT_LISTPC ve
#define NV_GRTCBB232MC_RETPIPECHAIN_BOTTOM_LISTPC
#define NV_GRTCBB232MC_RETPIPECHAIN_LOCAL_LISTPC
#define NV_GRTCBB232MC_RETPIPECHAIN_ALL_LISTPC ve
#define NV_VE32MC_RETPIPECHAIN_RETIME         1
#define NV_VE32MC_RETPIPECHAIN_READ           0
#define NV_VE32MC_RETPIPECHAIN_RCPIDWIDTH     0
#define NV_VE32MC_RETPIPECHAIN_RVPR           0
#define NV_VE32MC_RETPIPECHAIN_WRITE          1
#define NV_VE32MC_RETPIPECHAIN_WRCPIDWIDTH    0
#define NV_VE32MC_RETPIPECHAIN_LEFT_LISTPC
#define NV_VE32MC_RETPIPECHAIN_RIGHT_LISTPC
#define NV_VE32MC_RETPIPECHAIN_BOTTOM_LISTPC
#define NV_VE32MC_RETPIPECHAIN_LOCAL_LISTPC ve
#define NV_VE32MC_RETPIPECHAIN_ALL_LISTPC   ve
#define NV_MC_RETPIPECHAIN_READ               1
#define NV_MC_RETPIPECHAIN_WRITE              1
#define NV_MC_RETPIPECHAIN_RCPID              3
#define NV_MC_RETPIPECHAIN_WRCPID             3
#define NV_MC_RETPIPECHAIN_RVPR               1
#define NV_MC_RSPHUB_RDRSP_NUM_CHAINS         3
#define NV_MC_RSPHUB_WRRSP_NUM_CHAINS         4
#define NV_MC_RSPHUB_RDRSP_CHAIN0_DEST      mcha00
#define NV_MC_RSPHUB_RDRSP_CHAIN0_LISTPC    dis nvd vicpc gk
#define NV_MC_RSPHUB_RDRSP_CHAIN0_LISTCGID  146 120 108 88
#define NV_MC_RSPHUB_RDRSP_CHAIN0_READ        1
#define NV_MC_RSPHUB_RDRSP_CHAIN0_RCPIDWIDTH   1
#define NV_MC_RSPHUB_RDRSP_CHAIN0_RVPR        1
#define NV_MC_RSPHUB_RDRSP_CHAIN1_DEST      mcha01
#define NV_MC_RSPHUB_RDRSP_CHAIN1_LISTPC    dis2 nvd2 vicpc2 gk2
#define NV_MC_RSPHUB_RDRSP_CHAIN1_LISTCGID  161 163 162 136
#define NV_MC_RSPHUB_RDRSP_CHAIN1_READ        1
#define NV_MC_RSPHUB_RDRSP_CHAIN1_RCPIDWIDTH   1
#define NV_MC_RSPHUB_RDRSP_CHAIN1_RVPR        1
#define NV_MC_RSPHUB_RDRSP_CHAIN2_DEST      mchbb2
#define NV_MC_RSPHUB_RDRSP_CHAIN2_LISTPC    aondmapc aonpc apb apedmapc aud bpmpdmapc bpmppc dfd eqospc hdapc host isp jpg mse nic pcx sax scedmapc scepc sd sdm sdm1 ufshcpc usbd usbx
#define NV_MC_RSPHUB_RDRSP_CHAIN2_LISTCGID  153 151 128 84 134 159 122 149 147 132 142 21 22 68 126 28 140 14 31 157 155 98 96 99 97 144 76 74
#define NV_MC_RSPHUB_RDRSP_CHAIN2_READ        1
#define NV_MC_RSPHUB_RDRSP_CHAIN2_RCPIDWIDTH   3
#define NV_MC_RSPHUB_RDRSP_CHAIN2_RVPR        1
#define NV_MC_RSPHUB_WRRSP_CHAIN0_DEST      mcha00
#define NV_MC_RSPHUB_WRRSP_CHAIN0_LISTPC    gk
#define NV_MC_RSPHUB_WRRSP_CHAIN0_LISTCGID  89
#define NV_MC_RSPHUB_WRRSP_CHAIN0_WRITE       1
#define NV_MC_RSPHUB_WRRSP_CHAIN0_WRCPIDWIDTH   1
#define NV_MC_RSPHUB_WRRSP_CHAIN1_DEST      mcha01
#define NV_MC_RSPHUB_WRRSP_CHAIN1_LISTPC    gk2
#define NV_MC_RSPHUB_WRRSP_CHAIN1_LISTCGID  137
#define NV_MC_RSPHUB_WRRSP_CHAIN1_WRITE       1
#define NV_MC_RSPHUB_WRRSP_CHAIN1_WRCPIDWIDTH   1
#define NV_MC_RSPHUB_WRRSP_CHAIN2_DEST      mchbb2
#define NV_MC_RSPHUB_WRRSP_CHAIN2_LISTPC    aondmapc aonpc apb apedmapc aud bpmpdmapc bpmppc dfd eqospc hdapc isp jpg mse2 nic nvd3 pcx sax scedmapc scepc sd sdm sdm1 ufshcpc usbd usbx
#define NV_MC_RSPHUB_WRRSP_CHAIN2_LISTCGID  154 152 129 85 135 160 123 150 148 133 143 53 70 71 127 43 141 121 49 61 158 156 102 100 103 101 145 77 75
#define NV_MC_RSPHUB_WRRSP_CHAIN2_WRITE       1
#define NV_MC_RSPHUB_WRRSP_CHAIN2_WRCPIDWIDTH   3
#define NV_MC_RSPHUB_WRRSP_CHAIN3_DEST      mchbb3
#define NV_MC_RSPHUB_WRRSP_CHAIN3_LISTPC    ve vicpc3
#define NV_MC_RSPHUB_WRRSP_CHAIN3_LISTCGID  114 109
#define NV_MC_RSPHUB_WRRSP_CHAIN3_WRITE       1
#define NV_MC_RSPHUB_WRRSP_CHAIN3_WRCPIDWIDTH   0
#define  NV_MC_LA_SCALING_2_CLIST
//     Direct-client is: mll_mpcorer
#define NV_MLL_MPCORER2MC_NBCLIENTS           1
#define NV_MLL_MPCORER2MC_NBRCLIENTS          1
#define NV_MLL_MPCORER2MC_NBWCLIENTS          0
#define NV_MLL_MPCORER2MC_WDLOG2              5
#define NV_MC_NBGCIDS                       164
#define NV_MC_NBGCIDS_REGS                    5
#define NV_MC_MAXRCLIENTID                  163
#define NV_MC_NBRGCIDS                      164
#define NV_MC_NBRGCIDSLOG2                    8
#define NV_MC_NBCLIENTS                      72
#define NV_MC_NBRCLIENTS                     38
#define NV_MC_NBWCLIENTS                     34
#define NV_MC_NBSCLIENTS                     41
#define NV_MC_NBPCLIENTS                     38
#define NV_MC_NBHPCLIENTS                     0
#define NV_MC_NBRLLCLIENTS                    0
#define NV_MC_NBDIRCLIENTS                    1
#define NV_MC_NBTILECLIENTS                   0
#define NV_MC_NBGCIDSLOG2                     8
#define NV_MC_NBCLIENTSLOG2                   7
#define NV_MC_NBRCLIENTSLOG2                  6
#define NV_MC_NBWCLIENTSLOG2                  6
#define NV_MC_NBSCLIENTSLOG2                  6
#define NV_MC_NBPCLIENTSLOG2                  6
#define NV_MC_NBHPCLIENTSLOG2                 0
#define NV_MC_NBRLLCLIENTSLOG2                0
#define NV_MC_NBDIRCLIENTSLOG2                1
#define NV_MC_HAS_LOW_LATENCY                 1
#define NV_EMC_HAS_LOW_LATENCY                1
#define NV_MC_IMEM_REQ_ID_WIDTH              10
#define NV_MC_IMEM_RDI_ID_WIDTH              10
#define NV_MC_EMEM_REQ_ID_WIDTH               8
#define NV_MC_EMEM_RDI_ID_WIDTH               8
#define NV_AONDMAPC2MC_MAXCREDITS            37
#define NV_AONDMAPC2MC_MAXCREDITS_WR         11
#define NV_AONPC2MC_MAXCREDITS               11
#define NV_AONPC2MC_MAXCREDITS_WR             4
#define NV_APB2MC_MAXCREDITS                 18
#define NV_APB2MC_MAXCREDITS_WR              18
#define NV_APEDMAPC2MC_MAXCREDITS            40
#define NV_APEDMAPC2MC_MAXCREDITS_WR         12
#define NV_AUD2MC_MAXCREDITS                 19
#define NV_AUD2MC_MAXCREDITS_WR              12
#define NV_BPMPDMAPC2MC_MAXCREDITS           31
#define NV_BPMPDMAPC2MC_MAXCREDITS_WR         9
#define NV_BPMPPC2MC_MAXCREDITS              11
#define NV_BPMPPC2MC_MAXCREDITS_WR            4
#define NV_DFD2MC_MAXCREDITS                 18
#define NV_DFD2MC_MAXCREDITS_WR              18
#define NV_DIS2MC_MAXCREDITS                 21
#define NV_DIS2MC_MAXCREDITS_WR               0
#define NV_DIS22MC_MAXCREDITS                21
#define NV_DIS22MC_MAXCREDITS_WR              0
#define NV_EQOSPC2MC_MAXCREDITS              32
#define NV_EQOSPC2MC_MAXCREDITS_WR           12
#define NV_FTOP2MC_MAXCREDITS                16
#define NV_FTOP2MC_MAXCREDITS_WR             16
#define NV_MPCORER2MC_MAXCREDITS             16
#define NV_MPCORER2MC_MAXCREDITS_WR           0
#define NV_GK2MC_MAXCREDITS                  11
#define NV_GK2MC_MAXCREDITS_WR               11
#define NV_GK22MC_MAXCREDITS                 11
#define NV_GK22MC_MAXCREDITS_WR              11
#define NV_HDAPC2MC_MAXCREDITS                8
#define NV_HDAPC2MC_MAXCREDITS_WR             4
#define NV_HOST2MC_MAXCREDITS                26
#define NV_HOST2MC_MAXCREDITS_WR              0
#define NV_ISP2MC_MAXCREDITS                 29
#define NV_ISP2MC_MAXCREDITS_WR              29
#define NV_JPG2MC_MAXCREDITS                 36
#define NV_JPG2MC_MAXCREDITS_WR              36
#define NV_MSE2MC_MAXCREDITS                 28
#define NV_MSE2MC_MAXCREDITS_WR               0
#define NV_MSE22MC_MAXCREDITS                32
#define NV_MSE22MC_MAXCREDITS_WR             32
#define NV_NIC2MC_MAXCREDITS                 21
#define NV_NIC2MC_MAXCREDITS_WR              21
#define NV_NVD2MC_MAXCREDITS                 19
#define NV_NVD2MC_MAXCREDITS_WR               0
#define NV_NVD22MC_MAXCREDITS                19
#define NV_NVD22MC_MAXCREDITS_WR              0
#define NV_NVD32MC_MAXCREDITS                24
#define NV_NVD32MC_MAXCREDITS_WR             24
#define NV_PCX2MC_MAXCREDITS                 41
#define NV_PCX2MC_MAXCREDITS_WR              41
#define NV_SAX2MC_MAXCREDITS                 32
#define NV_SAX2MC_MAXCREDITS_WR              12
#define NV_SCEDMAPC2MC_MAXCREDITS            31
#define NV_SCEDMAPC2MC_MAXCREDITS_WR          9
#define NV_SCEPC2MC_MAXCREDITS               11
#define NV_SCEPC2MC_MAXCREDITS_WR             4
#define NV_SD2MC_MAXCREDITS                  37
#define NV_SD2MC_MAXCREDITS_WR               37
#define NV_SDM2MC_MAXCREDITS                 50
#define NV_SDM2MC_MAXCREDITS_WR              50
#define NV_SDM12MC_MAXCREDITS                46
#define NV_SDM12MC_MAXCREDITS_WR             46
#define NV_UFSHCPC2MC_MAXCREDITS             43
#define NV_UFSHCPC2MC_MAXCREDITS_WR          12
#define NV_USBD2MC_MAXCREDITS                48
#define NV_USBD2MC_MAXCREDITS_WR             13
#define NV_USBX2MC_MAXCREDITS                48
#define NV_USBX2MC_MAXCREDITS_WR             13
#define NV_VE2MC_MAXCREDITS                  27
#define NV_VE2MC_MAXCREDITS_WR               27
#define NV_VICPC2MC_MAXCREDITS               17
#define NV_VICPC2MC_MAXCREDITS_WR             0
#define NV_VICPC22MC_MAXCREDITS              17
#define NV_VICPC22MC_MAXCREDITS_WR            0
#define NV_VICPC32MC_MAXCREDITS              23
#define NV_VICPC32MC_MAXCREDITS_WR           23
#define NV_AONDMAPC2MC_ID                     0
#define NV_AONPC2MC_ID                        1
#define NV_APB2MC_ID                          2
#define NV_APEDMAPC2MC_ID                     3
#define NV_AUD2MC_ID                          4
#define NV_BPMPDMAPC2MC_ID                    5
#define NV_BPMPPC2MC_ID                       6
#define NV_DFD2MC_ID                          7
#define NV_DIS2MC_ID                          8
#define NV_DIS22MC_ID                         9
#define NV_EQOSPC2MC_ID                      10
#define NV_FTOP2MC_ID                        11
#define NV_GK2MC_ID                          12
#define NV_GK22MC_ID                         13
#define NV_HDAPC2MC_ID                       14
#define NV_HOST2MC_ID                        15
#define NV_ISP2MC_ID                         16
#define NV_JPG2MC_ID                         17
#define NV_MSE2MC_ID                         18
#define NV_MSE22MC_ID                        19
#define NV_NIC2MC_ID                         20
#define NV_NVD2MC_ID                         21
#define NV_NVD22MC_ID                        22
#define NV_NVD32MC_ID                        23
#define NV_PCX2MC_ID                         24
#define NV_SAX2MC_ID                         25
#define NV_SCEDMAPC2MC_ID                    26
#define NV_SCEPC2MC_ID                       27
#define NV_SD2MC_ID                          28
#define NV_SDM2MC_ID                         29
#define NV_SDM12MC_ID                        30
#define NV_UFSHCPC2MC_ID                     31
#define NV_USBD2MC_ID                        32
#define NV_USBX2MC_ID                        33
#define NV_VE2MC_ID                          34
#define NV_VICPC2MC_ID                       35
#define NV_VICPC22MC_ID                      36
#define NV_VICPC32MC_ID                      37
#define NV_AFI2MC_ID                          0
#define NV_AON2MC_ID                          1
#define NV_AONDMA2MC_ID                       2
#define NV_APE2MC_ID                          3
#define NV_APEDMA2MC_ID                       4
#define NV_AXIS2MC_ID                         5
#define NV_BPMP2MC_ID                         6
#define NV_BPMPDMA2MC_ID                      7
#define NV_EQOS2MC_ID                         8
#define NV_ETR2MC_ID                          9
#define NV_GPU2MC_ID                         10
#define NV_GPU22MC_ID                        11
#define NV_HDA2MC_ID                         12
#define NV_HOST1X2MC_ID                      13
#define NV_ISP22MC_ID                        14
#define NV_MPCORE2MC_ID                      15
#define NV_NVDEC2MC_ID                       16
#define NV_NVDEC12MC_ID                      17
#define NV_NVDEC32MC_ID                      18
#define NV_NVDISPLAY2MC_ID                   19
#define NV_NVDISPLAY12MC_ID                  20
#define NV_NVENC2MC_ID                       21
#define NV_NVENC22MC_ID                      22
#define NV_NVJPG2MC_ID                       23
#define NV_SATA2MC_ID                        24
#define NV_SCE2MC_ID                         25
#define NV_SCEDMA2MC_ID                      26
#define NV_SDMMC2MC_ID                       27
#define NV_SDMMCA2MC_ID                      28
#define NV_SDMMCAA2MC_ID                     29
#define NV_SDMMCAB2MC_ID                     30
#define NV_SE2MC_ID                          31
#define NV_TSEC2MC_ID                        32
#define NV_TSECB2MC_ID                       33
#define NV_UFSHC2MC_ID                       34
#define NV_VI22MC_ID                         35
#define NV_VIC2MC_ID                         36
#define NV_VIC12MC_ID                        37
#define NV_VIC32MC_ID                        38
#define NV_XUSB_DEV2MC_ID                    39
#define NV_XUSB_HOST2MC_ID                   40
#define NV_MLL_MPCORER2MC_ID                 41
#define NV_PTCR2MC_SR_ID                      0
#define NV_AFIR2MC_SR_ID                     14
#define NV_HDAR2MC_SR_ID                     21
#define NV_HOST1XDMAR2MC_SR_ID               22
#define NV_NVENCSRD2MC_SR_ID                 28
#define NV_SATAR2MC_SR_ID                    31
#define NV_MPCORER2MC_SR_ID                  39
#define NV_NVENCSWR2MC_SW_ID                 43
#define NV_AFIW2MC_SW_ID                     49
#define NV_HDAW2MC_SW_ID                     53
#define NV_MPCOREW2MC_SW_ID                  57
#define NV_SATAW2MC_SW_ID                    61
#define NV_ISPRA2MC_SR_ID                    68
#define NV_ISPWA2MC_SW_ID                    70
#define NV_ISPWB2MC_SW_ID                    71
#define NV_XUSB_HOSTR2MC_SR_ID               74
#define NV_XUSB_HOSTW2MC_SW_ID               75
#define NV_XUSB_DEVR2MC_SR_ID                76
#define NV_XUSB_DEVW2MC_SW_ID                77
#define NV_TSECSRD2MC_SR_ID                  84
#define NV_TSECSWR2MC_SW_ID                  85
#define NV_GPUSRD2MC_SR_ID                   88
#define NV_GPUSWR2MC_SW_ID                   89
#define NV_SDMMCRA2MC_SR_ID                  96
#define NV_SDMMCRAA2MC_SR_ID                 97
#define NV_SDMMCR2MC_SR_ID                   98
#define NV_SDMMCRAB2MC_SR_ID                 99
#define NV_SDMMCWA2MC_SW_ID                 100
#define NV_SDMMCWAA2MC_SW_ID                101
#define NV_SDMMCW2MC_SW_ID                  102
#define NV_SDMMCWAB2MC_SW_ID                103
#define NV_VICSRD2MC_SR_ID                  108
#define NV_VICSWR2MC_SW_ID                  109
#define NV_VIW2MC_SW_ID                     114
#define NV_NVDECSRD2MC_SR_ID                120
#define NV_NVDECSWR2MC_SW_ID                121
#define NV_APER2MC_SR_ID                    122
#define NV_APEW2MC_SW_ID                    123
#define NV_NVJPGSRD2MC_SR_ID                126
#define NV_NVJPGSWR2MC_SW_ID                127
#define NV_SESRD2MC_SR_ID                   128
#define NV_SESWR2MC_SW_ID                   129
#define NV_ETRR2MC_SR_ID                    132
#define NV_ETRW2MC_SW_ID                    133
#define NV_TSECSRDB2MC_SR_ID                134
#define NV_TSECSWRB2MC_SW_ID                135
#define NV_GPUSRD22MC_SR_ID                 136
#define NV_GPUSWR22MC_SW_ID                 137
#define NV_AXISR2MC_SR_ID                   140
#define NV_AXISW2MC_SW_ID                   141
#define NV_EQOSR2MC_SR_ID                   142
#define NV_EQOSW2MC_SW_ID                   143
#define NV_UFSHCR2MC_SR_ID                  144
#define NV_UFSHCW2MC_SW_ID                  145
#define NV_NVDISPLAYR2MC_SR_ID              146
#define NV_BPMPR2MC_SR_ID                   147
#define NV_BPMPW2MC_SW_ID                   148
#define NV_BPMPDMAR2MC_SR_ID                149
#define NV_BPMPDMAW2MC_SW_ID                150
#define NV_AONR2MC_SR_ID                    151
#define NV_AONW2MC_SW_ID                    152
#define NV_AONDMAR2MC_SR_ID                 153
#define NV_AONDMAW2MC_SW_ID                 154
#define NV_SCER2MC_SR_ID                    155
#define NV_SCEW2MC_SW_ID                    156
#define NV_SCEDMAR2MC_SR_ID                 157
#define NV_SCEDMAW2MC_SW_ID                 158
#define NV_APEDMAR2MC_SR_ID                 159
#define NV_APEDMAW2MC_SW_ID                 160
#define NV_NVDISPLAYR12MC_SR_ID             161
#define NV_VICSRD12MC_SR_ID                 162
#define NV_NVDECSRD12MC_SR_ID               163
#define NV_AFIR2MC_SR_SWID                    0
#define NV_AFIW2MC_SW_SWID                    0
#define NV_HOST1XDMAR2MC_SR_SWID              6
#define NV_HDAR2MC_SR_SWID                    7
#define NV_HDAW2MC_SW_SWID                    7
#define NV_ISPRA2MC_SR_SWID                   8
#define NV_ISPWA2MC_SW_SWID                   8
#define NV_ISPWB2MC_SW_SWID                   8
#define NV_MPCORER2MC_SR_SWID                 9
#define NV_MPCOREW2MC_SW_SWID                 9
#define NV_NVENCSRD2MC_SR_SWID               11
#define NV_NVENCSWR2MC_SW_SWID               11
#define NV_PTCR2MC_SR_SWID                    0
#define NV_SATAR2MC_SR_SWID                  15
#define NV_SATAW2MC_SW_SWID                  15
#define NV_VIW2MC_SW_SWID                    17
#define NV_VICSRD2MC_SR_SWID                 18
#define NV_VICSWR2MC_SW_SWID                 18
#define NV_VICSRD12MC_SR_SWID                18
#define NV_XUSB_HOSTR2MC_SR_SWID             19
#define NV_XUSB_HOSTW2MC_SW_SWID             19
#define NV_XUSB_DEVR2MC_SR_SWID              20
#define NV_XUSB_DEVW2MC_SW_SWID              20
#define NV_TSECSRD2MC_SR_SWID                22
#define NV_TSECSWR2MC_SW_SWID                22
#define NV_SDMMCRA2MC_SR_SWID                29
#define NV_SDMMCWA2MC_SW_SWID                29
#define NV_SDMMCRAA2MC_SR_SWID               30
#define NV_SDMMCWAA2MC_SW_SWID               30
#define NV_SDMMCR2MC_SR_SWID                 31
#define NV_SDMMCW2MC_SW_SWID                 31
#define NV_SDMMCRAB2MC_SR_SWID               32
#define NV_SDMMCWAB2MC_SW_SWID               32
#define NV_GPUSRD2MC_SR_SWID                 34
#define NV_GPUSWR2MC_SW_SWID                 34
#define NV_GPUSRD22MC_SR_SWID                34
#define NV_GPUSWR22MC_SW_SWID                34
#define NV_NVDECSRD2MC_SR_SWID               37
#define NV_NVDECSWR2MC_SW_SWID               37
#define NV_NVDECSRD12MC_SR_SWID              37
#define NV_APER2MC_SR_SWID                   38
#define NV_APEW2MC_SW_SWID                   38
#define NV_APEDMAR2MC_SR_SWID                38
#define NV_APEDMAW2MC_SW_SWID                38
#define NV_SESRD2MC_SR_SWID                  39
#define NV_SESWR2MC_SW_SWID                  39
#define NV_NVJPGSRD2MC_SR_SWID               40
#define NV_NVJPGSWR2MC_SW_SWID               40
#define NV_ETRR2MC_SR_SWID                   44
#define NV_ETRW2MC_SW_SWID                   44
#define NV_TSECSRDB2MC_SR_SWID               45
#define NV_TSECSWRB2MC_SW_SWID               45
#define NV_AXISR2MC_SR_SWID                  50
#define NV_AXISW2MC_SW_SWID                  50
#define NV_EQOSR2MC_SR_SWID                  51
#define NV_EQOSW2MC_SW_SWID                  51
#define NV_UFSHCR2MC_SR_SWID                 52
#define NV_UFSHCW2MC_SW_SWID                 52
#define NV_NVDISPLAYR2MC_SR_SWID             53
#define NV_NVDISPLAYR12MC_SR_SWID            53
#define NV_BPMPR2MC_SR_SWID                  54
#define NV_BPMPW2MC_SW_SWID                  54
#define NV_BPMPDMAR2MC_SR_SWID               54
#define NV_BPMPDMAW2MC_SW_SWID               54
#define NV_AONR2MC_SR_SWID                   55
#define NV_AONW2MC_SW_SWID                   55
#define NV_AONDMAR2MC_SR_SWID                55
#define NV_AONDMAW2MC_SW_SWID                55
#define NV_SCER2MC_SR_SWID                   56
#define NV_SCEW2MC_SW_SWID                   56
#define NV_SCEDMAR2MC_SR_SWID                56
#define NV_SCEDMAW2MC_SW_SWID                56
#define NV_MC_SMMU_NUM_SWIDS                 56
#define NV_MC_SMMU_SWID_BITS                  6
#define NV_MC_LAYOUTPARTITIONS aonpg aud bpmp dfd disiha ftop gr grtbb grtta grttb host ispac nic nvdeca nvencb nvjpg pcx sax sce sec sor ufs ve vica xusbb xusbc
#define NV_MC_HAS_LAYOUTPARTITION_AONPG 1
#define NV_MC_HAS_LAYOUTPARTITION_AUD 1
#define NV_MC_HAS_LAYOUTPARTITION_BPMP 1
#define NV_MC_HAS_LAYOUTPARTITION_DFD 1
#define NV_MC_HAS_LAYOUTPARTITION_DISIHA 1
#define NV_MC_HAS_LAYOUTPARTITION_FTOP 1
#define NV_MC_HAS_LAYOUTPARTITION_GR 1
#define NV_MC_HAS_LAYOUTPARTITION_GRTBB 1
#define NV_MC_HAS_LAYOUTPARTITION_GRTTA 1
#define NV_MC_HAS_LAYOUTPARTITION_GRTTB 1
#define NV_MC_HAS_LAYOUTPARTITION_HOST 1
#define NV_MC_HAS_LAYOUTPARTITION_ISPAC 1
#define NV_MC_HAS_LAYOUTPARTITION_NIC 1
#define NV_MC_HAS_LAYOUTPARTITION_NVDECA 1
#define NV_MC_HAS_LAYOUTPARTITION_NVENCB 1
#define NV_MC_HAS_LAYOUTPARTITION_NVJPG 1
#define NV_MC_HAS_LAYOUTPARTITION_PCX 1
#define NV_MC_HAS_LAYOUTPARTITION_SAX 1
#define NV_MC_HAS_LAYOUTPARTITION_SCE 1
#define NV_MC_HAS_LAYOUTPARTITION_SEC 1
#define NV_MC_HAS_LAYOUTPARTITION_SOR 1
#define NV_MC_HAS_LAYOUTPARTITION_UFS 1
#define NV_MC_HAS_LAYOUTPARTITION_VE 1
#define NV_MC_HAS_LAYOUTPARTITION_VICA 1
#define NV_MC_HAS_LAYOUTPARTITION_XUSBB 1
#define NV_MC_HAS_LAYOUTPARTITION_XUSBC 1
#define LIST_MC_LAYOUTPARTITIONS(_op_) \
  _op_(AONPG, aonpg) \
  _op_(AUD, aud) \
  _op_(BPMP, bpmp) \
  _op_(DFD, dfd) \
  _op_(DISIHA, disiha) \
  _op_(FTOP, ftop) \
  _op_(GR, gr) \
  _op_(GRTBB, grtbb) \
  _op_(GRTTA, grtta) \
  _op_(GRTTB, grttb) \
  _op_(HOST, host) \
  _op_(ISPAC, ispac) \
  _op_(NIC, nic) \
  _op_(NVDECA, nvdeca) \
  _op_(NVENCB, nvencb) \
  _op_(NVJPG, nvjpg) \
  _op_(PCX, pcx) \
  _op_(SAX, sax) \
  _op_(SCE, sce) \
  _op_(SEC, sec) \
  _op_(SOR, sor) \
  _op_(UFS, ufs) \
  _op_(VE, ve) \
  _op_(VICA, vica) \
  _op_(XUSBB, xusbb) \
  _op_(XUSBC, xusbc)
#define LIST_MC_PSYNC_CLIENTS(_op_)

#define LIST_MC_SC_CLIENTS(_op_) \
 _op_(14_49, AFIR) \
 _op_(151_152, AONR) \
 _op_(153_154, AONDMAR) \
 _op_(122_123, APER) \
 _op_(159_160, APEDMAR) \
 _op_(140_141, AXISR) \
 _op_(147_148, BPMPR) \
 _op_(149_150, BPMPDMAR) \
 _op_(142_143, EQOSR) \
 _op_(132_133, ETRR) \
 _op_(88_89, GPUSRD) \
 _op_(136_137, GPUSRD2) \
 _op_(21_53, HDAR) \
 _op_(22, HOST1XDMAR) \
 _op_(68_70_71, ISPRA) \
 _op_(57_39, MPCOREW) \
 _op_(120, NVDECSRD) \
 _op_(121, NVDECSWR) \
 _op_(146, NVDISPLAYR) \
 _op_(28, NVENCSRD) \
 _op_(43, NVENCSWR) \
 _op_(126_127, NVJPGSRD) \
 _op_(31_61, SATAR) \
 _op_(155_156, SCER) \
 _op_(157_158, SCEDMAR) \
 _op_(98_102, SDMMCR) \
 _op_(96_100, SDMMCRA) \
 _op_(97_101, SDMMCRAA) \
 _op_(99_103, SDMMCRAB) \
 _op_(128_129, SESRD) \
 _op_(84_85, TSECSRD) \
 _op_(134_135, TSECSRDB) \
 _op_(144_145, UFSHCR) \
 _op_(114, VIW) \
 _op_(108, VICSRD) \
 _op_(109, VICSWR) \
 _op_(76_77, XUSB_DEVR) \
 _op_(74_75, XUSB_HOSTR)

#define LIST_MC_PC_CLIENTS(_op_) \
 _op_(128_84_134_129_85_135, apb) \
 _op_(96_99_100_103, sdm)

#define NV_MC_HAS_PARTITIONCLIENT_aondmapc 1
#define NV_MC_HAS_PARTITIONCLIENT_aonpc 1
#define NV_MC_HAS_PARTITIONCLIENT_apb 1
#define NV_MC_HAS_PARTITIONCLIENT_apedmapc 1
#define NV_MC_HAS_PARTITIONCLIENT_aud 1
#define NV_MC_HAS_PARTITIONCLIENT_bpmpdmapc 1
#define NV_MC_HAS_PARTITIONCLIENT_bpmppc 1
#define NV_MC_HAS_PARTITIONCLIENT_dfd 1
#define NV_MC_HAS_PARTITIONCLIENT_dis 1
#define NV_MC_HAS_PARTITIONCLIENT_dis2 1
#define NV_MC_HAS_PARTITIONCLIENT_eqospc 1
#define NV_MC_HAS_PARTITIONCLIENT_ftop 1
#define NV_MC_HAS_PARTITIONCLIENT_gk 1
#define NV_MC_HAS_PARTITIONCLIENT_gk2 1
#define NV_MC_HAS_PARTITIONCLIENT_hdapc 1
#define NV_MC_HAS_PARTITIONCLIENT_host 1
#define NV_MC_HAS_PARTITIONCLIENT_isp 1
#define NV_MC_HAS_PARTITIONCLIENT_jpg 1
#define NV_MC_HAS_PARTITIONCLIENT_mse 1
#define NV_MC_HAS_PARTITIONCLIENT_mse2 1
#define NV_MC_HAS_PARTITIONCLIENT_nic 1
#define NV_MC_HAS_PARTITIONCLIENT_nvd 1
#define NV_MC_HAS_PARTITIONCLIENT_nvd2 1
#define NV_MC_HAS_PARTITIONCLIENT_nvd3 1
#define NV_MC_HAS_PARTITIONCLIENT_pcx 1
#define NV_MC_HAS_PARTITIONCLIENT_sax 1
#define NV_MC_HAS_PARTITIONCLIENT_scedmapc 1
#define NV_MC_HAS_PARTITIONCLIENT_scepc 1
#define NV_MC_HAS_PARTITIONCLIENT_sd 1
#define NV_MC_HAS_PARTITIONCLIENT_sdm 1
#define NV_MC_HAS_PARTITIONCLIENT_sdm1 1
#define NV_MC_HAS_PARTITIONCLIENT_ufshcpc 1
#define NV_MC_HAS_PARTITIONCLIENT_usbd 1
#define NV_MC_HAS_PARTITIONCLIENT_usbx 1
#define NV_MC_HAS_PARTITIONCLIENT_ve 1
#define NV_MC_HAS_PARTITIONCLIENT_vicpc 1
#define NV_MC_HAS_PARTITIONCLIENT_vicpc2 1
#define NV_MC_HAS_PARTITIONCLIENT_vicpc3 1
#define LIST_MC_PARTITIONCLIENTS(_op_)\
  _op_(AONDMAPC, aondmapc)\
  _op_(AONPC, aonpc)\
  _op_(APB, apb)\
  _op_(APEDMAPC, apedmapc)\
  _op_(AUD, aud)\
  _op_(BPMPDMAPC, bpmpdmapc)\
  _op_(BPMPPC, bpmppc)\
  _op_(DFD, dfd)\
  _op_(DIS, dis)\
  _op_(DIS2, dis2)\
  _op_(EQOSPC, eqospc)\
  _op_(FTOP, ftop)\
  _op_(GK, gk)\
  _op_(GK2, gk2)\
  _op_(HDAPC, hdapc)\
  _op_(HOST, host)\
  _op_(ISP, isp)\
  _op_(JPG, jpg)\
  _op_(MSE, mse)\
  _op_(MSE2, mse2)\
  _op_(NIC, nic)\
  _op_(NVD, nvd)\
  _op_(NVD2, nvd2)\
  _op_(NVD3, nvd3)\
  _op_(PCX, pcx)\
  _op_(SAX, sax)\
  _op_(SCEDMAPC, scedmapc)\
  _op_(SCEPC, scepc)\
  _op_(SD, sd)\
  _op_(SDM, sdm)\
  _op_(SDM1, sdm1)\
  _op_(UFSHCPC, ufshcpc)\
  _op_(USBD, usbd)\
  _op_(USBX, usbx)\
  _op_(VE, ve)\
  _op_(VICPC, vicpc)\
  _op_(VICPC2, vicpc2)\
  _op_(VICPC3, vicpc3)
#define LIST_INSIDEMC_PARTITIONCLIENTS(_op_)
#define LIST_MC_SUPERCLIENTS(_op_)
#define LIST_MC2_SUPERCLIENTS(_op_) \
  _op_(AFI, afi) \
  _op_(AON, aon) \
  _op_(AONDMA, aondma) \
  _op_(APE, ape) \
  _op_(APEDMA, apedma) \
  _op_(AXIS, axis) \
  _op_(BPMP, bpmp) \
  _op_(BPMPDMA, bpmpdma) \
  _op_(EQOS, eqos) \
  _op_(ETR, etr) \
  _op_(GPU, gpu) \
  _op_(GPU2, gpu2) \
  _op_(HDA, hda) \
  _op_(HOST1X, host1x) \
  _op_(ISP2, isp2) \
  _op_(NVDEC, nvdec) \
  _op_(NVDEC3, nvdec3) \
  _op_(NVDISPLAY, nvdisplay) \
  _op_(NVENC, nvenc) \
  _op_(NVENC2, nvenc2) \
  _op_(NVJPG, nvjpg) \
  _op_(SATA, sata) \
  _op_(SCE, sce) \
  _op_(SCEDMA, scedma) \
  _op_(SDMMC, sdmmc) \
  _op_(SDMMCA, sdmmca) \
  _op_(SDMMCAA, sdmmcaa) \
  _op_(SDMMCAB, sdmmcab) \
  _op_(SE, se) \
  _op_(TSEC, tsec) \
  _op_(TSECB, tsecb) \
  _op_(UFSHC, ufshc) \
  _op_(VI2, vi2) \
  _op_(VIC, vic) \
  _op_(VIC3, vic3) \
  _op_(XUSB_DEV, xusb_dev) \
  _op_(XUSB_HOST, xusb_host) \
  _op_(MPCORE, mpcore)
#define LIST_MCCMOD_SUPERCLIENTS(_op_)
#define LIST_MC2CMOD_SUPERCLIENTS(_op_) \
  _op_(AFI, afi) \
  _op_(AON, aon) \
  _op_(AONDMA, aondma) \
  _op_(APE, ape) \
  _op_(APEDMA, apedma) \
  _op_(AXIS, axis) \
  _op_(BPMP, bpmp) \
  _op_(BPMPDMA, bpmpdma) \
  _op_(EQOS, eqos) \
  _op_(ETR, etr) \
  _op_(GPU, gpu) \
  _op_(GPU2, gpu2) \
  _op_(HDA, hda) \
  _op_(HOST1X, host1x) \
  _op_(ISP2, isp2) \
  _op_(MPCORE, mpcore) \
  _op_(NVDEC, nvdec) \
  _op_(NVDEC1, nvdec1) \
  _op_(NVDEC3, nvdec3) \
  _op_(NVDISPLAY, nvdisplay) \
  _op_(NVDISPLAY1, nvdisplay1) \
  _op_(NVENC, nvenc) \
  _op_(NVENC2, nvenc2) \
  _op_(NVJPG, nvjpg) \
  _op_(SATA, sata) \
  _op_(SCE, sce) \
  _op_(SCEDMA, scedma) \
  _op_(SDMMC, sdmmc) \
  _op_(SDMMCA, sdmmca) \
  _op_(SDMMCAA, sdmmcaa) \
  _op_(SDMMCAB, sdmmcab) \
  _op_(SE, se) \
  _op_(TSEC, tsec) \
  _op_(TSECB, tsecb) \
  _op_(UFSHC, ufshc) \
  _op_(VI2, vi2) \
  _op_(VIC, vic) \
  _op_(VIC1, vic1) \
  _op_(VIC3, vic3) \
  _op_(XUSB_DEV, xusb_dev) \
  _op_(XUSB_HOST, xusb_host)\
 _op_(MLL_MPCORER, mll_mpcorer)

#define LIST_INSIDEMC_SUPERCLIENTS(_op_)

#define LIST_INSIDEMC2_SUPERCLIENTS(_op_)
#define LIST_SID_OVERRIDE_CLIENTS(_op_) \
 _op_(NV_PTCR2MC_SR_ID, PTCR)  \
 _op_(NV_AFIR2MC_SR_ID, AFIR)  \
 _op_(NV_HDAR2MC_SR_ID, HDAR)  \
 _op_(NV_HOST1XDMAR2MC_SR_ID, HOST1XDMAR)  \
 _op_(NV_NVENCSRD2MC_SR_ID, NVENCSRD)  \
 _op_(NV_SATAR2MC_SR_ID, SATAR)  \
 _op_(NV_MPCORER2MC_SR_ID, MPCORER)  \
 _op_(NV_NVENCSWR2MC_SW_ID, NVENCSWR)  \
 _op_(NV_AFIW2MC_SW_ID, AFIW)  \
 _op_(NV_HDAW2MC_SW_ID, HDAW)  \
 _op_(NV_MPCOREW2MC_SW_ID, MPCOREW)  \
 _op_(NV_SATAW2MC_SW_ID, SATAW)  \
 _op_(NV_ISPRA2MC_SR_ID, ISPRA)  \
 _op_(NV_ISPWA2MC_SW_ID, ISPWA)  \
 _op_(NV_ISPWB2MC_SW_ID, ISPWB)  \
 _op_(NV_XUSB_HOSTR2MC_SR_ID, XUSB_HOSTR)  \
 _op_(NV_XUSB_HOSTW2MC_SW_ID, XUSB_HOSTW)  \
 _op_(NV_XUSB_DEVR2MC_SR_ID, XUSB_DEVR)  \
 _op_(NV_XUSB_DEVW2MC_SW_ID, XUSB_DEVW)  \
 _op_(NV_TSECSRD2MC_SR_ID, TSECSRD)  \
 _op_(NV_TSECSWR2MC_SW_ID, TSECSWR)  \
 _op_(NV_GPUSRD2MC_SR_ID, GPUSRD)  \
 _op_(NV_GPUSWR2MC_SW_ID, GPUSWR)  \
 _op_(NV_SDMMCRA2MC_SR_ID, SDMMCRA)  \
 _op_(NV_SDMMCRAA2MC_SR_ID, SDMMCRAA)  \
 _op_(NV_SDMMCR2MC_SR_ID, SDMMCR)  \
 _op_(NV_SDMMCRAB2MC_SR_ID, SDMMCRAB)  \
 _op_(NV_SDMMCWA2MC_SW_ID, SDMMCWA)  \
 _op_(NV_SDMMCWAA2MC_SW_ID, SDMMCWAA)  \
 _op_(NV_SDMMCW2MC_SW_ID, SDMMCW)  \
 _op_(NV_SDMMCWAB2MC_SW_ID, SDMMCWAB)  \
 _op_(NV_VICSRD2MC_SR_ID, VICSRD)  \
 _op_(NV_VICSWR2MC_SW_ID, VICSWR)  \
 _op_(NV_VIW2MC_SW_ID, VIW)  \
 _op_(NV_NVDECSRD2MC_SR_ID, NVDECSRD)  \
 _op_(NV_NVDECSWR2MC_SW_ID, NVDECSWR)  \
 _op_(NV_APER2MC_SR_ID, APER)  \
 _op_(NV_APEW2MC_SW_ID, APEW)  \
 _op_(NV_NVJPGSRD2MC_SR_ID, NVJPGSRD)  \
 _op_(NV_NVJPGSWR2MC_SW_ID, NVJPGSWR)  \
 _op_(NV_SESRD2MC_SR_ID, SESRD)  \
 _op_(NV_SESWR2MC_SW_ID, SESWR)  \
 _op_(NV_ETRR2MC_SR_ID, ETRR)  \
 _op_(NV_ETRW2MC_SW_ID, ETRW)  \
 _op_(NV_TSECSRDB2MC_SR_ID, TSECSRDB)  \
 _op_(NV_TSECSWRB2MC_SW_ID, TSECSWRB)  \
 _op_(NV_GPUSRD22MC_SR_ID, GPUSRD2)  \
 _op_(NV_GPUSWR22MC_SW_ID, GPUSWR2)  \
 _op_(NV_AXISR2MC_SR_ID, AXISR)  \
 _op_(NV_AXISW2MC_SW_ID, AXISW)  \
 _op_(NV_EQOSR2MC_SR_ID, EQOSR)  \
 _op_(NV_EQOSW2MC_SW_ID, EQOSW)  \
 _op_(NV_UFSHCR2MC_SR_ID, UFSHCR)  \
 _op_(NV_UFSHCW2MC_SW_ID, UFSHCW)  \
 _op_(NV_NVDISPLAYR2MC_SR_ID, NVDISPLAYR)  \
 _op_(NV_BPMPR2MC_SR_ID, BPMPR)  \
 _op_(NV_BPMPW2MC_SW_ID, BPMPW)  \
 _op_(NV_BPMPDMAR2MC_SR_ID, BPMPDMAR)  \
 _op_(NV_BPMPDMAW2MC_SW_ID, BPMPDMAW)  \
 _op_(NV_AONR2MC_SR_ID, AONR)  \
 _op_(NV_AONW2MC_SW_ID, AONW)  \
 _op_(NV_AONDMAR2MC_SR_ID, AONDMAR)  \
 _op_(NV_AONDMAW2MC_SW_ID, AONDMAW)  \
 _op_(NV_SCER2MC_SR_ID, SCER)  \
 _op_(NV_SCEW2MC_SW_ID, SCEW)  \
 _op_(NV_SCEDMAR2MC_SR_ID, SCEDMAR)  \
 _op_(NV_SCEDMAW2MC_SW_ID, SCEDMAW)  \
 _op_(NV_APEDMAR2MC_SR_ID, APEDMAR)  \
 _op_(NV_APEDMAW2MC_SW_ID, APEDMAW)  \
 _op_(NV_NVDISPLAYR12MC_SR_ID, NVDISPLAYR1)  \
 _op_(NV_VICSRD12MC_SR_ID, VICSRD1)  \
 _op_(NV_NVDECSRD12MC_SR_ID, NVDECSRD1)

#define LIST_MC_CLIENTS(_op_)
#define LIST_MC2_CLIENTS(_op_) \
  _op_(smmu, 0, SMMU, PTCR, SR)  \
  _op_(afi, 0, AFI, AFIR, SR)  \
  _op_(aondma, 0, AONDMA, AONDMAR, SR)  \
  _op_(aon, 0, AON, AONR, SR)  \
  _op_(apedma, 0, APEDMA, APEDMAR, SR)  \
  _op_(ape, 0, APE, APER, SR)  \
  _op_(axis, 0, AXIS, AXISR, SR)  \
  _op_(bpmpdma, 0, BPMPDMA, BPMPDMAR, SR)  \
  _op_(bpmp, 0, BPMP, BPMPR, SR)  \
  _op_(eqos, 0, EQOS, EQOSR, SR)  \
  _op_(etr, 0, ETR, ETRR, SR)  \
  _op_(gpu, 0, GPU, GPUSRD, SR)  \
  _op_(gpu2, 0, GPU2, GPUSRD2, SR)  \
  _op_(hda, 0, HDA, HDAR, SR)  \
  _op_(host1x, 0, HOST1X, HOST1XDMAR, SR)  \
  _op_(isp2, 0, ISP2, ISPRA, SR)  \
  _op_(nvdec, 0, NVDEC, NVDECSRD, SR)  \
  _op_(nvdec1, 0, NVDEC1, NVDECSRD1, SR)  \
  _op_(nvdisplay, 0, NVDISPLAY, NVDISPLAYR, SR)  \
  _op_(nvdisplay1, 0, NVDISPLAY1, NVDISPLAYR1, SR)  \
  _op_(nvenc, 0, NVENC, NVENCSRD, SR)  \
  _op_(nvjpg, 0, NVJPG, NVJPGSRD, SR)  \
  _op_(sata, 0, SATA, SATAR, SR)  \
  _op_(scedma, 0, SCEDMA, SCEDMAR, SR)  \
  _op_(sce, 0, SCE, SCER, SR)  \
  _op_(sdmmc, 0, SDMMC, SDMMCR, SR)  \
  _op_(sdmmca, 0, SDMMCA, SDMMCRA, SR)  \
  _op_(sdmmcaa, 0, SDMMCAA, SDMMCRAA, SR)  \
  _op_(sdmmcab, 0, SDMMCAB, SDMMCRAB, SR)  \
  _op_(se, 0, SE, SESRD, SR)  \
  _op_(tsec, 0, TSEC, TSECSRD, SR)  \
  _op_(tsecb, 0, TSECB, TSECSRDB, SR)  \
  _op_(ufshc, 0, UFSHC, UFSHCR, SR)  \
  _op_(vic, 0, VIC, VICSRD, SR)  \
  _op_(vic1, 0, VIC1, VICSRD1, SR)  \
  _op_(xusb_dev, 0, XUSB_DEV, XUSB_DEVR, SR)  \
  _op_(xusb_host, 0, XUSB_HOST, XUSB_HOSTR, SR)  \
  _op_(mll_mpcorer, 0, MLL_MPCORER, MPCORER, SR)  \
  _op_(afi, 0, AFI, AFIW, SW)  \
  _op_(aondma, 0, AONDMA, AONDMAW, SW)  \
  _op_(aon, 0, AON, AONW, SW)  \
  _op_(apedma, 0, APEDMA, APEDMAW, SW)  \
  _op_(ape, 0, APE, APEW, SW)  \
  _op_(axis, 0, AXIS, AXISW, SW)  \
  _op_(bpmpdma, 0, BPMPDMA, BPMPDMAW, SW)  \
  _op_(bpmp, 0, BPMP, BPMPW, SW)  \
  _op_(eqos, 0, EQOS, EQOSW, SW)  \
  _op_(etr, 0, ETR, ETRW, SW)  \
  _op_(gpu, 0, GPU, GPUSWR, SW)  \
  _op_(gpu2, 0, GPU2, GPUSWR2, SW)  \
  _op_(hda, 0, HDA, HDAW, SW)  \
  _op_(isp2, 0, ISP2, ISPWA, SW)  \
  _op_(isp2, 0, ISP2, ISPWB, SW)  \
  _op_(mpcore, 0, MPCORE, MPCOREW, SW)  \
  _op_(nvdec3, 0, NVDEC3, NVDECSWR, SW)  \
  _op_(nvenc2, 0, NVENC2, NVENCSWR, SW)  \
  _op_(nvjpg, 0, NVJPG, NVJPGSWR, SW)  \
  _op_(sata, 0, SATA, SATAW, SW)  \
  _op_(scedma, 0, SCEDMA, SCEDMAW, SW)  \
  _op_(sce, 0, SCE, SCEW, SW)  \
  _op_(sdmmc, 0, SDMMC, SDMMCW, SW)  \
  _op_(sdmmca, 0, SDMMCA, SDMMCWA, SW)  \
  _op_(sdmmcaa, 0, SDMMCAA, SDMMCWAA, SW)  \
  _op_(sdmmcab, 0, SDMMCAB, SDMMCWAB, SW)  \
  _op_(se, 0, SE, SESWR, SW)  \
  _op_(tsec, 0, TSEC, TSECSWR, SW)  \
  _op_(tsecb, 0, TSECB, TSECSWRB, SW)  \
  _op_(ufshc, 0, UFSHC, UFSHCW, SW)  \
  _op_(vic3, 0, VIC3, VICSWR, SW)  \
  _op_(vi2, 0, VI2, VIW, SW)  \
  _op_(xusb_dev, 0, XUSB_DEV, XUSB_DEVW, SW)  \
  _op_(xusb_host, 0, XUSB_HOST, XUSB_HOSTW, SW)
#define LIST_AXI4_FABRIC_CLIENTS(_op_) \
  _op_(afi, AFI, 0, afir2mc_axi, AFIR, NV_AFIR2MC_SR_ID, SR, 148)  \
  _op_(aondma, AONDMA, 0, aondmar2mc_axi, AONDMAR, NV_AONDMAR2MC_SR_ID, SR, 44)  \
  _op_(aon, AON, 0, aonr2mc_axi, AONR, NV_AONR2MC_SR_ID, SR, 14)  \
  _op_(apedma, APEDMA, 0, apedmar2mc_axi, APEDMAR, NV_APEDMAR2MC_SR_ID, SR, 36)  \
  _op_(ape, APE, 0, aper2mc_axi, APER, NV_APER2MC_SR_ID, SR, 14)  \
  _op_(axis, AXIS, 0, axisr2mc_axi, AXISR, NV_AXISR2MC_SR_ID, SR, 44)  \
  _op_(bpmpdma, BPMPDMA, 0, bpmpdmar2mc_axi, BPMPDMAR, NV_BPMPDMAR2MC_SR_ID, SR, 44)  \
  _op_(bpmp, BPMP, 0, bpmpr2mc_axi, BPMPR, NV_BPMPR2MC_SR_ID, SR, 14)  \
  _op_(eqos, EQOS, 0, eqosr2mc_axi, EQOSR, NV_EQOSR2MC_SR_ID, SR, 28)  \
  _op_(etr, ETR, 0, etrr2mc_axi, ETRR, NV_ETRR2MC_SR_ID, SR, 8)  \
  _op_(gpu, GPU, 0, gpusrd2mc_axi, GPUSRD, NV_GPUSRD2MC_SR_ID, SR, 268)  \
  _op_(gpu2, GPU2, 0, gpusrd22mc_axi, GPUSRD2, NV_GPUSRD22MC_SR_ID, SR, 268)  \
  _op_(hda, HDA, 0, hdar2mc_axi, HDAR, NV_HDAR2MC_SR_ID, SR, 8)  \
  _op_(host1x, HOST1X, 0, host1xdmar2mc_axi, HOST1XDMAR, NV_HOST1XDMAR2MC_SR_ID, SR, 44)  \
  _op_(isp2, ISP2, 0, ispra2mc_axi, ISPRA, NV_ISPRA2MC_SR_ID, SR, 140)  \
  _op_(nvdec, NVDEC, 0, nvdecsrd2mc_axi, NVDECSRD, NV_NVDECSRD2MC_SR_ID, SR, 175)  \
  _op_(nvdisplay, NVDISPLAY, 0, nvdisplayr2mc_axi, NVDISPLAYR, NV_NVDISPLAYR2MC_SR_ID, SR, 274)  \
  _op_(nvenc, NVENC, 0, nvencsrd2mc_axi, NVENCSRD, NV_NVENCSRD2MC_SR_ID, SR, 268)  \
  _op_(nvjpg, NVJPG, 0, nvjpgsrd2mc_axi, NVJPGSRD, NV_NVJPGSRD2MC_SR_ID, SR, 172)  \
  _op_(sata, SATA, 0, satar2mc_axi, SATAR, NV_SATAR2MC_SR_ID, SR, 28)  \
  _op_(scedma, SCEDMA, 0, scedmar2mc_axi, SCEDMAR, NV_SCEDMAR2MC_SR_ID, SR, 44)  \
  _op_(sce, SCE, 0, scer2mc_axi, SCER, NV_SCER2MC_SR_ID, SR, 14)  \
  _op_(sdmmc, SDMMC, 0, sdmmcr2mc_axi, SDMMCR, NV_SDMMCR2MC_SR_ID, SR, 36)  \
  _op_(sdmmca, SDMMCA, 0, sdmmcra2mc_axi, SDMMCRA, NV_SDMMCRA2MC_SR_ID, SR, 36)  \
  _op_(sdmmcaa, SDMMCAA, 0, sdmmcraa2mc_axi, SDMMCRAA, NV_SDMMCRAA2MC_SR_ID, SR, 60)  \
  _op_(sdmmcab, SDMMCAB, 0, sdmmcrab2mc_axi, SDMMCRAB, NV_SDMMCRAB2MC_SR_ID, SR, 108)  \
  _op_(se, SE, 0, sesrd2mc_axi, SESRD, NV_SESRD2MC_SR_ID, SR, 148)  \
  _op_(tsec, TSEC, 0, tsecsrd2mc_axi, TSECSRD, NV_TSECSRD2MC_SR_ID, SR, 32)  \
  _op_(tsecb, TSECB, 0, tsecsrdb2mc_axi, TSECSRDB, NV_TSECSRDB2MC_SR_ID, SR, 32)  \
  _op_(ufshc, UFSHC, 0, ufshcr2mc_axi, UFSHCR, NV_UFSHCR2MC_SR_ID, SR, 84)  \
  _op_(vic, VIC, 0, vicsrd2mc_axi, VICSRD, NV_VICSRD2MC_SR_ID, SR, 257)  \
  _op_(xusb_dev, XUSB_DEV, 0, xusb_devr2mc_axi, XUSB_DEVR, NV_XUSB_DEVR2MC_SR_ID, SR, 52)  \
  _op_(xusb_host, XUSB_HOST, 0, xusb_hostr2mc_axi, XUSB_HOSTR, NV_XUSB_HOSTR2MC_SR_ID, SR, 84)  \
  _op_(afi, AFI, 0, afiw2mc_axi, AFIW, NV_AFIW2MC_SW_ID, SW, 72)  \
  _op_(aondma, AONDMA, 0, aondmaw2mc_axi, AONDMAW, NV_AONDMAW2MC_SW_ID, SW, 24)  \
  _op_(aon, AON, 0, aonw2mc_axi, AONW, NV_AONW2MC_SW_ID, SW, 8)  \
  _op_(apedma, APEDMA, 0, apedmaw2mc_axi, APEDMAW, NV_APEDMAW2MC_SW_ID, SW, 24)  \
  _op_(ape, APE, 0, apew2mc_axi, APEW, NV_APEW2MC_SW_ID, SW, 20)  \
  _op_(axis, AXIS, 0, axisw2mc_axi, AXISW, NV_AXISW2MC_SW_ID, SW, 40)  \
  _op_(bpmpdma, BPMPDMA, 0, bpmpdmaw2mc_axi, BPMPDMAW, NV_BPMPDMAW2MC_SW_ID, SW, 24)  \
  _op_(bpmp, BPMP, 0, bpmpw2mc_axi, BPMPW, NV_BPMPW2MC_SW_ID, SW, 8)  \
  _op_(eqos, EQOS, 0, eqosw2mc_axi, EQOSW, NV_EQOSW2MC_SW_ID, SW, 24)  \
  _op_(etr, ETR, 0, etrw2mc_axi, ETRW, NV_ETRW2MC_SW_ID, SW, 72)  \
  _op_(gpu, GPU, 0, gpuswr2mc_axi, GPUSWR, NV_GPUSWR2MC_SW_ID, SW, 140)  \
  _op_(gpu2, GPU2, 0, gpuswr22mc_axi, GPUSWR2, NV_GPUSWR22MC_SW_ID, SW, 140)  \
  _op_(hda, HDA, 0, hdaw2mc_axi, HDAW, NV_HDAW2MC_SW_ID, SW, 8)  \
  _op_(isp2, ISP2, 0, ispwa2mc_axi, ISPWA, NV_ISPWA2MC_SW_ID, SW, 72)  \
  _op_(isp2, ISP2, 0, ispwb2mc_axi, ISPWB, NV_ISPWB2MC_SW_ID, SW, 28)  \
  _op_(nvdec3, NVDEC3, 0, nvdecswr2mc_axi, NVDECSWR, NV_NVDECSWR2MC_SW_ID, SW, 72)  \
  _op_(nvenc2, NVENC2, 0, nvencswr2mc_axi, NVENCSWR, NV_NVENCSWR2MC_SW_ID, SW, 40)  \
  _op_(nvjpg, NVJPG, 0, nvjpgswr2mc_axi, NVJPGSWR, NV_NVJPGSWR2MC_SW_ID, SW, 40)  \
  _op_(sata, SATA, 0, sataw2mc_axi, SATAW, NV_SATAW2MC_SW_ID, SW, 24)  \
  _op_(scedma, SCEDMA, 0, scedmaw2mc_axi, SCEDMAW, NV_SCEDMAW2MC_SW_ID, SW, 24)  \
  _op_(sce, SCE, 0, scew2mc_axi, SCEW, NV_SCEW2MC_SW_ID, SW, 8)  \
  _op_(sdmmc, SDMMC, 0, sdmmcw2mc_axi, SDMMCW, NV_SDMMCW2MC_SW_ID, SW, 24)  \
  _op_(sdmmca, SDMMCA, 0, sdmmcwa2mc_axi, SDMMCWA, NV_SDMMCWA2MC_SW_ID, SW, 24)  \
  _op_(sdmmcaa, SDMMCAA, 0, sdmmcwaa2mc_axi, SDMMCWAA, NV_SDMMCWAA2MC_SW_ID, SW, 40)  \
  _op_(sdmmcab, SDMMCAB, 0, sdmmcwab2mc_axi, SDMMCWAB, NV_SDMMCWAB2MC_SW_ID, SW, 72)  \
  _op_(se, SE, 0, seswr2mc_axi, SESWR, NV_SESWR2MC_SW_ID, SW, 72)  \
  _op_(tsec, TSEC, 0, tsecswr2mc_axi, TSECSWR, NV_TSECSWR2MC_SW_ID, SW, 32)  \
  _op_(tsecb, TSECB, 0, tsecswrb2mc_axi, TSECSWRB, NV_TSECSWRB2MC_SW_ID, SW, 32)  \
  _op_(ufshc, UFSHC, 0, ufshcw2mc_axi, UFSHCW, NV_UFSHCW2MC_SW_ID, SW, 40)  \
  _op_(vic3, VIC3, 0, vicswr2mc_axi, VICSWR, NV_VICSWR2MC_SW_ID, SW, 140)  \
  _op_(vi2, VI2, 0, viw2mc_axi, VIW, NV_VIW2MC_SW_ID, SW, 136)  \
  _op_(xusb_dev, XUSB_DEV, 0, xusb_devw2mc_axi, XUSB_DEVW, NV_XUSB_DEVW2MC_SW_ID, SW, 48)  \
  _op_(xusb_host, XUSB_HOST, 0, xusb_hostw2mc_axi, XUSB_HOSTW, NV_XUSB_HOSTW2MC_SW_ID, SW, 80)
#define LIST_CLIENTS_SR(_op_) \
  _op_(smmu, 0, SMMU, PTCR, SR)  \
  _op_(afi, 0, AFI, AFIR, SR)  \
  _op_(aondma, 0, AONDMA, AONDMAR, SR)  \
  _op_(aon, 0, AON, AONR, SR)  \
  _op_(apedma, 0, APEDMA, APEDMAR, SR)  \
  _op_(ape, 0, APE, APER, SR)  \
  _op_(axis, 0, AXIS, AXISR, SR)  \
  _op_(bpmpdma, 0, BPMPDMA, BPMPDMAR, SR)  \
  _op_(bpmp, 0, BPMP, BPMPR, SR)  \
  _op_(eqos, 0, EQOS, EQOSR, SR)  \
  _op_(etr, 0, ETR, ETRR, SR)  \
  _op_(gpu, 0, GPU, GPUSRD, SR)  \
  _op_(gpu2, 0, GPU2, GPUSRD2, SR)  \
  _op_(hda, 0, HDA, HDAR, SR)  \
  _op_(host1x, 0, HOST1X, HOST1XDMAR, SR)  \
  _op_(isp2, 0, ISP2, ISPRA, SR)  \
  _op_(nvdec, 0, NVDEC, NVDECSRD, SR)  \
  _op_(nvdec1, 0, NVDEC1, NVDECSRD1, SR)  \
  _op_(nvdisplay, 0, NVDISPLAY, NVDISPLAYR, SR)  \
  _op_(nvdisplay1, 0, NVDISPLAY1, NVDISPLAYR1, SR)  \
  _op_(nvenc, 0, NVENC, NVENCSRD, SR)  \
  _op_(nvjpg, 0, NVJPG, NVJPGSRD, SR)  \
  _op_(sata, 0, SATA, SATAR, SR)  \
  _op_(scedma, 0, SCEDMA, SCEDMAR, SR)  \
  _op_(sce, 0, SCE, SCER, SR)  \
  _op_(sdmmc, 0, SDMMC, SDMMCR, SR)  \
  _op_(sdmmca, 0, SDMMCA, SDMMCRA, SR)  \
  _op_(sdmmcaa, 0, SDMMCAA, SDMMCRAA, SR)  \
  _op_(sdmmcab, 0, SDMMCAB, SDMMCRAB, SR)  \
  _op_(se, 0, SE, SESRD, SR)  \
  _op_(tsec, 0, TSEC, TSECSRD, SR)  \
  _op_(tsecb, 0, TSECB, TSECSRDB, SR)  \
  _op_(ufshc, 0, UFSHC, UFSHCR, SR)  \
  _op_(vic, 0, VIC, VICSRD, SR)  \
  _op_(vic1, 0, VIC1, VICSRD1, SR)  \
  _op_(xusb_dev, 0, XUSB_DEV, XUSB_DEVR, SR)  \
  _op_(xusb_host, 0, XUSB_HOST, XUSB_HOSTR, SR)  \
  _op_(mll_mpcorer, 0, MLL_MPCORER, MPCORER, SR)
#define LIST_CLIENTS_SW(_op_) \
  _op_(afi, 0, AFI, AFIW, SW)  \
  _op_(aondma, 0, AONDMA, AONDMAW, SW)  \
  _op_(aon, 0, AON, AONW, SW)  \
  _op_(apedma, 0, APEDMA, APEDMAW, SW)  \
  _op_(ape, 0, APE, APEW, SW)  \
  _op_(axis, 0, AXIS, AXISW, SW)  \
  _op_(bpmpdma, 0, BPMPDMA, BPMPDMAW, SW)  \
  _op_(bpmp, 0, BPMP, BPMPW, SW)  \
  _op_(eqos, 0, EQOS, EQOSW, SW)  \
  _op_(etr, 0, ETR, ETRW, SW)  \
  _op_(gpu, 0, GPU, GPUSWR, SW)  \
  _op_(gpu2, 0, GPU2, GPUSWR2, SW)  \
  _op_(hda, 0, HDA, HDAW, SW)  \
  _op_(isp2, 0, ISP2, ISPWA, SW)  \
  _op_(isp2, 0, ISP2, ISPWB, SW)  \
  _op_(mpcore, 0, MPCORE, MPCOREW, SW)  \
  _op_(nvdec3, 0, NVDEC3, NVDECSWR, SW)  \
  _op_(nvenc2, 0, NVENC2, NVENCSWR, SW)  \
  _op_(nvjpg, 0, NVJPG, NVJPGSWR, SW)  \
  _op_(sata, 0, SATA, SATAW, SW)  \
  _op_(scedma, 0, SCEDMA, SCEDMAW, SW)  \
  _op_(sce, 0, SCE, SCEW, SW)  \
  _op_(sdmmc, 0, SDMMC, SDMMCW, SW)  \
  _op_(sdmmca, 0, SDMMCA, SDMMCWA, SW)  \
  _op_(sdmmcaa, 0, SDMMCAA, SDMMCWAA, SW)  \
  _op_(sdmmcab, 0, SDMMCAB, SDMMCWAB, SW)  \
  _op_(se, 0, SE, SESWR, SW)  \
  _op_(tsec, 0, TSEC, TSECSWR, SW)  \
  _op_(tsecb, 0, TSECB, TSECSWRB, SW)  \
  _op_(ufshc, 0, UFSHC, UFSHCW, SW)  \
  _op_(vic3, 0, VIC3, VICSWR, SW)  \
  _op_(vi2, 0, VI2, VIW, SW)  \
  _op_(xusb_dev, 0, XUSB_DEV, XUSB_DEVW, SW)  \
  _op_(xusb_host, 0, XUSB_HOST, XUSB_HOSTW, SW)
#define LIST_CLIENTS_BR(_op_)
#define LIST_CLIENTS_BW(_op_)
#define LIST_CLIENTS_CW(_op_)

#define LIST_INSIDEMC_CLIENTS(_op_)
#define LIST_INSIDEMC_CLIENTS_SR(_op_)
#define LIST_INSIDEMC_CLIENTS_SW(_op_)
#define LIST_INSIDEMC_CLIENTS_BR(_op_)
#define LIST_INSIDEMC_CLIENTS_BW(_op_)
#define LIST_INSIDEMC_CLIENTS_CW(_op_)
#define NV_MC_SWNAME_LIST AFI, HC, HDA, ISP2, MPCORE, NVENC, PTC, SATA, VI, VIC, XUSB_HOST, XUSB_DEV, TSEC, SDMMC1A, SDMMC2A, SDMMC3A, SDMMC4A, GPU, NVDEC, APE, SE, NVJPG, ETR, TSECB, AXIS, EQOS, UFSHC, NVDISPLAY, BPMP, AON, SCE
#define LIST_MC_SWNAME(_op_) \
  _op_(AFI,afi) \
  _op_(HC,hc) \
  _op_(HDA,hda) \
  _op_(ISP2,isp2) \
  _op_(MPCORE,mpcore) \
  _op_(NVENC,nvenc) \
  _op_(PTC,ptc) \
  _op_(SATA,sata) \
  _op_(VI,vi) \
  _op_(VIC,vic) \
  _op_(XUSB_HOST,xusb_host) \
  _op_(XUSB_DEV,xusb_dev) \
  _op_(TSEC,tsec) \
  _op_(SDMMC1A,sdmmc1a) \
  _op_(SDMMC2A,sdmmc2a) \
  _op_(SDMMC3A,sdmmc3a) \
  _op_(SDMMC4A,sdmmc4a) \
  _op_(GPU,gpu) \
  _op_(NVDEC,nvdec) \
  _op_(APE,ape) \
  _op_(SE,se) \
  _op_(NVJPG,nvjpg) \
  _op_(ETR,etr) \
  _op_(TSECB,tsecb) \
  _op_(AXIS,axis) \
  _op_(EQOS,eqos) \
  _op_(UFSHC,ufshc) \
  _op_(NVDISPLAY,nvdisplay) \
  _op_(BPMP,bpmp) \
  _op_(AON,aon) \
  _op_(SCE,sce)

#define LIST_MC_TRANSLATABLE_SWNAME(_op_) \
  _op_(AFI,afi,0) \
  _op_(HC,hc,6) \
  _op_(HDA,hda,7) \
  _op_(ISP2,isp2,8) \
  _op_(NVENC,nvenc,11) \
  _op_(SATA,sata,15) \
  _op_(VI,vi,17) \
  _op_(VIC,vic,18) \
  _op_(XUSB_HOST,xusb_host,19) \
  _op_(XUSB_DEV,xusb_dev,20) \
  _op_(TSEC,tsec,22) \
  _op_(SDMMC1A,sdmmc1a,29) \
  _op_(SDMMC2A,sdmmc2a,30) \
  _op_(SDMMC3A,sdmmc3a,31) \
  _op_(SDMMC4A,sdmmc4a,32) \
  _op_(GPU,gpu,34) \
  _op_(NVDEC,nvdec,37) \
  _op_(APE,ape,38) \
  _op_(SE,se,39) \
  _op_(NVJPG,nvjpg,40) \
  _op_(ETR,etr,44) \
  _op_(TSECB,tsecb,45) \
  _op_(AXIS,axis,50) \
  _op_(EQOS,eqos,51) \
  _op_(UFSHC,ufshc,52) \
  _op_(NVDISPLAY,nvdisplay,53) \
  _op_(BPMP,bpmp,54) \
  _op_(AON,aon,55) \
  _op_(SCE,sce,56)

#define LIST_MC_TRANSLATABLE_SWNAME_AW(_op_) \
  _op_(AFI,afi,0,40) \
  _op_(HC,hc,6,40) \
  _op_(HDA,hda,7,40) \
  _op_(ISP2,isp2,8,40) \
  _op_(NVENC,nvenc,11,40) \
  _op_(SATA,sata,15,40) \
  _op_(VI,vi,17,40) \
  _op_(VIC,vic,18,40) \
  _op_(XUSB_HOST,xusb_host,19,40) \
  _op_(XUSB_DEV,xusb_dev,20,40) \
  _op_(TSEC,tsec,22,40) \
  _op_(SDMMC1A,sdmmc1a,29,40) \
  _op_(SDMMC2A,sdmmc2a,30,40) \
  _op_(SDMMC3A,sdmmc3a,31,40) \
  _op_(SDMMC4A,sdmmc4a,32,40) \
  _op_(GPU,gpu,34,40) \
  _op_(NVDEC,nvdec,37,40) \
  _op_(APE,ape,38,40) \
  _op_(SE,se,39,40) \
  _op_(NVJPG,nvjpg,40,40) \
  _op_(ETR,etr,44,40) \
  _op_(TSECB,tsecb,45,40) \
  _op_(AXIS,axis,50,40) \
  _op_(EQOS,eqos,51,40) \
  _op_(UFSHC,ufshc,52,40) \
  _op_(NVDISPLAY,nvdisplay,53,40) \
  _op_(BPMP,bpmp,54,40) \
  _op_(AON,aon,55,40) \
  _op_(SCE,sce,56,40)

#define LIST_MC_CLIENTSWNAMES(_op_) \
  _op_(PTCR, PTC, 0) \
  _op_(AFIR, AFI, 1) \
  _op_(AONDMAR, AON, 1) \
  _op_(AONR, AON, 1) \
  _op_(APEDMAR, APE, 1) \
  _op_(APER, APE, 1) \
  _op_(AXISR, AXIS, 1) \
  _op_(BPMPDMAR, BPMP, 1) \
  _op_(BPMPR, BPMP, 1) \
  _op_(EQOSR, EQOS, 1) \
  _op_(ETRR, ETR, 1) \
  _op_(GPUSRD, GPU, 1) \
  _op_(GPUSRD2, GPU, 1) \
  _op_(HDAR, HDA, 1) \
  _op_(HOST1XDMAR, HC, 1) \
  _op_(ISPRA, ISP2, 1) \
  _op_(NVDECSRD, NVDEC, 1) \
  _op_(NVDECSRD1, NVDEC, 1) \
  _op_(NVDISPLAYR, NVDISPLAY, 1) \
  _op_(NVDISPLAYR1, NVDISPLAY, 1) \
  _op_(NVENCSRD, NVENC, 1) \
  _op_(NVJPGSRD, NVJPG, 1) \
  _op_(SATAR, SATA, 1) \
  _op_(SCEDMAR, SCE, 1) \
  _op_(SCER, SCE, 1) \
  _op_(SDMMCR, SDMMC3A, 1) \
  _op_(SDMMCRA, SDMMC1A, 1) \
  _op_(SDMMCRAA, SDMMC2A, 1) \
  _op_(SDMMCRAB, SDMMC4A, 1) \
  _op_(SESRD, SE, 1) \
  _op_(TSECSRD, TSEC, 1) \
  _op_(TSECSRDB, TSECB, 1) \
  _op_(UFSHCR, UFSHC, 1) \
  _op_(VICSRD, VIC, 1) \
  _op_(VICSRD1, VIC, 1) \
  _op_(XUSB_DEVR, XUSB_DEV, 1) \
  _op_(XUSB_HOSTR, XUSB_HOST, 1) \
  _op_(MPCORER, MPCORE, 0) \
  _op_(AFIW, AFI, 0) \
  _op_(AONDMAW, AON, 0) \
  _op_(AONW, AON, 0) \
  _op_(APEDMAW, APE, 0) \
  _op_(APEW, APE, 0) \
  _op_(AXISW, AXIS, 0) \
  _op_(BPMPDMAW, BPMP, 0) \
  _op_(BPMPW, BPMP, 0) \
  _op_(EQOSW, EQOS, 0) \
  _op_(ETRW, ETR, 0) \
  _op_(GPUSWR, GPU, 0) \
  _op_(GPUSWR2, GPU, 0) \
  _op_(HDAW, HDA, 0) \
  _op_(ISPWA, ISP2, 0) \
  _op_(ISPWB, ISP2, 0) \
  _op_(MPCOREW, MPCORE, 0) \
  _op_(NVDECSWR, NVDEC, 0) \
  _op_(NVENCSWR, NVENC, 0) \
  _op_(NVJPGSWR, NVJPG, 0) \
  _op_(SATAW, SATA, 0) \
  _op_(SCEDMAW, SCE, 0) \
  _op_(SCEW, SCE, 0) \
  _op_(SDMMCW, SDMMC3A, 0) \
  _op_(SDMMCWA, SDMMC1A, 0) \
  _op_(SDMMCWAA, SDMMC2A, 0) \
  _op_(SDMMCWAB, SDMMC4A, 0) \
  _op_(SESWR, SE, 0) \
  _op_(TSECSWR, TSEC, 0) \
  _op_(TSECSWRB, TSECB, 0) \
  _op_(UFSHCW, UFSHC, 0) \
  _op_(VICSWR, VIC, 0) \
  _op_(VIW, VI, 0) \
  _op_(XUSB_DEVW, XUSB_DEV, 0) \
  _op_(XUSB_HOSTW, XUSB_HOST, 0)

#define LIST_MC_SWNAME_RCOAL(_op_) \
  _op_(AFI) \
  _op_(HC) \
  _op_(HDA) \
  _op_(ISP2) \
  _op_(NVENC) \
  _op_(SATA) \
  _op_(VIC) \
  _op_(XUSB_HOST) \
  _op_(XUSB_DEV) \
  _op_(TSEC) \
  _op_(SDMMC1A) \
  _op_(SDMMC2A) \
  _op_(SDMMC3A) \
  _op_(SDMMC4A) \
  _op_(GPU) \
  _op_(NVDEC) \
  _op_(APE) \
  _op_(SE) \
  _op_(NVJPG) \
  _op_(ETR) \
  _op_(TSECB) \
  _op_(AXIS) \
  _op_(EQOS) \
  _op_(UFSHC) \
  _op_(NVDISPLAY) \
  _op_(BPMP) \
  _op_(AON) \
  _op_(SCE)

#define NV_MC_GCID_LIST PTCR, NOT_A_CLIENT, NOT_A_CLIENT, NOT_A_CLIENT, NOT_A_CLIENT, NOT_A_CLIENT, NOT_A_CLIENT, NOT_A_CLIENT, NOT_A_CLIENT, NOT_A_CLIENT, NOT_A_CLIENT, NOT_A_CLIENT, NOT_A_CLIENT, NOT_A_CLIENT, AFIR, NOT_A_CLIENT, NOT_A_CLIENT, NOT_A_CLIENT, NOT_A_CLIENT, NOT_A_CLIENT, NOT_A_CLIENT, HDAR, HOST1XDMAR, NOT_A_CLIENT, NOT_A_CLIENT, NOT_A_CLIENT, NOT_A_CLIENT, NOT_A_CLIENT, NVENCSRD, NOT_A_CLIENT, NOT_A_CLIENT, SATAR, NOT_A_CLIENT, NOT_A_CLIENT, NOT_A_CLIENT, NOT_A_CLIENT, NOT_A_CLIENT, NOT_A_CLIENT, NOT_A_CLIENT, MPCORER, NOT_A_CLIENT, NOT_A_CLIENT, NOT_A_CLIENT, NVENCSWR, NOT_A_CLIENT, NOT_A_CLIENT, NOT_A_CLIENT, NOT_A_CLIENT, NOT_A_CLIENT, AFIW, NOT_A_CLIENT, NOT_A_CLIENT, NOT_A_CLIENT, HDAW, NOT_A_CLIENT, NOT_A_CLIENT, NOT_A_CLIENT, MPCOREW, NOT_A_CLIENT, NOT_A_CLIENT, NOT_A_CLIENT, SATAW, NOT_A_CLIENT, NOT_A_CLIENT, NOT_A_CLIENT, NOT_A_CLIENT, NOT_A_CLIENT, NOT_A_CLIENT, ISPRA, NOT_A_CLIENT, ISPWA, ISPWB, NOT_A_CLIENT, NOT_A_CLIENT, XUSB_HOSTR, XUSB_HOSTW, XUSB_DEVR, XUSB_DEVW, NOT_A_CLIENT, NOT_A_CLIENT, NOT_A_CLIENT, NOT_A_CLIENT, NOT_A_CLIENT, NOT_A_CLIENT, TSECSRD, TSECSWR, NOT_A_CLIENT, NOT_A_CLIENT, GPUSRD, GPUSWR, NOT_A_CLIENT, NOT_A_CLIENT, NOT_A_CLIENT, NOT_A_CLIENT, NOT_A_CLIENT, NOT_A_CLIENT, SDMMCRA, SDMMCRAA, SDMMCR, SDMMCRAB, SDMMCWA, SDMMCWAA, SDMMCW, SDMMCWAB, NOT_A_CLIENT, NOT_A_CLIENT, NOT_A_CLIENT, NOT_A_CLIENT, VICSRD, VICSWR, NOT_A_CLIENT, NOT_A_CLIENT, NOT_A_CLIENT, NOT_A_CLIENT, VIW, NOT_A_CLIENT, NOT_A_CLIENT, NOT_A_CLIENT, NOT_A_CLIENT, NOT_A_CLIENT, NVDECSRD, NVDECSWR, APER, APEW, NOT_A_CLIENT, NOT_A_CLIENT, NVJPGSRD, NVJPGSWR, SESRD, SESWR, NOT_A_CLIENT, NOT_A_CLIENT, ETRR, ETRW, TSECSRDB, TSECSWRB, GPUSRD2, GPUSWR2, NOT_A_CLIENT, NOT_A_CLIENT, AXISR, AXISW, EQOSR, EQOSW, UFSHCR, UFSHCW, NVDISPLAYR, BPMPR, BPMPW, BPMPDMAR, BPMPDMAW, AONR, AONW, AONDMAR, AONDMAW, SCER, SCEW, SCEDMAR, SCEDMAW, APEDMAR, APEDMAW, NVDISPLAYR1, VICSRD1, NVDECSRD1
#define LIST_MC_GCID(_op_) \
  _op_(PTCR) \
  _op_(NOT_A_CLIENT_0) \
  _op_(NOT_A_CLIENT_1) \
  _op_(NOT_A_CLIENT_2) \
  _op_(NOT_A_CLIENT_3) \
  _op_(NOT_A_CLIENT_4) \
  _op_(NOT_A_CLIENT_5) \
  _op_(NOT_A_CLIENT_6) \
  _op_(NOT_A_CLIENT_7) \
  _op_(NOT_A_CLIENT_8) \
  _op_(NOT_A_CLIENT_9) \
  _op_(NOT_A_CLIENT_10) \
  _op_(NOT_A_CLIENT_11) \
  _op_(NOT_A_CLIENT_12) \
  _op_(AFIR) \
  _op_(NOT_A_CLIENT_13) \
  _op_(NOT_A_CLIENT_14) \
  _op_(NOT_A_CLIENT_15) \
  _op_(NOT_A_CLIENT_16) \
  _op_(NOT_A_CLIENT_17) \
  _op_(NOT_A_CLIENT_18) \
  _op_(HDAR) \
  _op_(HOST1XDMAR) \
  _op_(NOT_A_CLIENT_19) \
  _op_(NOT_A_CLIENT_20) \
  _op_(NOT_A_CLIENT_21) \
  _op_(NOT_A_CLIENT_22) \
  _op_(NOT_A_CLIENT_23) \
  _op_(NVENCSRD) \
  _op_(NOT_A_CLIENT_24) \
  _op_(NOT_A_CLIENT_25) \
  _op_(SATAR) \
  _op_(NOT_A_CLIENT_26) \
  _op_(NOT_A_CLIENT_27) \
  _op_(NOT_A_CLIENT_28) \
  _op_(NOT_A_CLIENT_29) \
  _op_(NOT_A_CLIENT_30) \
  _op_(NOT_A_CLIENT_31) \
  _op_(NOT_A_CLIENT_32) \
  _op_(MPCORER) \
  _op_(NOT_A_CLIENT_33) \
  _op_(NOT_A_CLIENT_34) \
  _op_(NOT_A_CLIENT_35) \
  _op_(NVENCSWR) \
  _op_(NOT_A_CLIENT_36) \
  _op_(NOT_A_CLIENT_37) \
  _op_(NOT_A_CLIENT_38) \
  _op_(NOT_A_CLIENT_39) \
  _op_(NOT_A_CLIENT_40) \
  _op_(AFIW) \
  _op_(NOT_A_CLIENT_41) \
  _op_(NOT_A_CLIENT_42) \
  _op_(NOT_A_CLIENT_43) \
  _op_(HDAW) \
  _op_(NOT_A_CLIENT_44) \
  _op_(NOT_A_CLIENT_45) \
  _op_(NOT_A_CLIENT_46) \
  _op_(MPCOREW) \
  _op_(NOT_A_CLIENT_47) \
  _op_(NOT_A_CLIENT_48) \
  _op_(NOT_A_CLIENT_49) \
  _op_(SATAW) \
  _op_(NOT_A_CLIENT_50) \
  _op_(NOT_A_CLIENT_51) \
  _op_(NOT_A_CLIENT_52) \
  _op_(NOT_A_CLIENT_53) \
  _op_(NOT_A_CLIENT_54) \
  _op_(NOT_A_CLIENT_55) \
  _op_(ISPRA) \
  _op_(NOT_A_CLIENT_56) \
  _op_(ISPWA) \
  _op_(ISPWB) \
  _op_(NOT_A_CLIENT_57) \
  _op_(NOT_A_CLIENT_58) \
  _op_(XUSB_HOSTR) \
  _op_(XUSB_HOSTW) \
  _op_(XUSB_DEVR) \
  _op_(XUSB_DEVW) \
  _op_(NOT_A_CLIENT_59) \
  _op_(NOT_A_CLIENT_60) \
  _op_(NOT_A_CLIENT_61) \
  _op_(NOT_A_CLIENT_62) \
  _op_(NOT_A_CLIENT_63) \
  _op_(NOT_A_CLIENT_64) \
  _op_(TSECSRD) \
  _op_(TSECSWR) \
  _op_(NOT_A_CLIENT_65) \
  _op_(NOT_A_CLIENT_66) \
  _op_(GPUSRD) \
  _op_(GPUSWR) \
  _op_(NOT_A_CLIENT_67) \
  _op_(NOT_A_CLIENT_68) \
  _op_(NOT_A_CLIENT_69) \
  _op_(NOT_A_CLIENT_70) \
  _op_(NOT_A_CLIENT_71) \
  _op_(NOT_A_CLIENT_72) \
  _op_(SDMMCRA) \
  _op_(SDMMCRAA) \
  _op_(SDMMCR) \
  _op_(SDMMCRAB) \
  _op_(SDMMCWA) \
  _op_(SDMMCWAA) \
  _op_(SDMMCW) \
  _op_(SDMMCWAB) \
  _op_(NOT_A_CLIENT_73) \
  _op_(NOT_A_CLIENT_74) \
  _op_(NOT_A_CLIENT_75) \
  _op_(NOT_A_CLIENT_76) \
  _op_(VICSRD) \
  _op_(VICSWR) \
  _op_(NOT_A_CLIENT_77) \
  _op_(NOT_A_CLIENT_78) \
  _op_(NOT_A_CLIENT_79) \
  _op_(NOT_A_CLIENT_80) \
  _op_(VIW) \
  _op_(NOT_A_CLIENT_81) \
  _op_(NOT_A_CLIENT_82) \
  _op_(NOT_A_CLIENT_83) \
  _op_(NOT_A_CLIENT_84) \
  _op_(NOT_A_CLIENT_85) \
  _op_(NVDECSRD) \
  _op_(NVDECSWR) \
  _op_(APER) \
  _op_(APEW) \
  _op_(NOT_A_CLIENT_86) \
  _op_(NOT_A_CLIENT_87) \
  _op_(NVJPGSRD) \
  _op_(NVJPGSWR) \
  _op_(SESRD) \
  _op_(SESWR) \
  _op_(NOT_A_CLIENT_88) \
  _op_(NOT_A_CLIENT_89) \
  _op_(ETRR) \
  _op_(ETRW) \
  _op_(TSECSRDB) \
  _op_(TSECSWRB) \
  _op_(GPUSRD2) \
  _op_(GPUSWR2) \
  _op_(NOT_A_CLIENT_90) \
  _op_(NOT_A_CLIENT_91) \
  _op_(AXISR) \
  _op_(AXISW) \
  _op_(EQOSR) \
  _op_(EQOSW) \
  _op_(UFSHCR) \
  _op_(UFSHCW) \
  _op_(NVDISPLAYR) \
  _op_(BPMPR) \
  _op_(BPMPW) \
  _op_(BPMPDMAR) \
  _op_(BPMPDMAW) \
  _op_(AONR) \
  _op_(AONW) \
  _op_(AONDMAR) \
  _op_(AONDMAW) \
  _op_(SCER) \
  _op_(SCEW) \
  _op_(SCEDMAR) \
  _op_(SCEDMAW) \
  _op_(APEDMAR) \
  _op_(APEDMAW) \
  _op_(NVDISPLAYR1) \
  _op_(VICSRD1) \
  _op_(NVDECSRD1)

#define LIST_CLK_ALLMSCCLOCKS(_op_) \
  _op_(msc_afi2mc_cclk_nocg, C_NOCG, afi, 1, 1) \
  _op_(msc_aon2mc_cclk_nocg, C_NOCG, aon, 1, 1) \
  _op_(msc_aondma2mc_cclk_nocg, C_NOCG, aondma, 1, 1) \
  _op_(msc_ape2mc_cclk_nocg, C_NOCG, ape, 1, 1) \
  _op_(msc_apedma2mc_cclk_nocg, C_NOCG, apedma, 1, 1) \
  _op_(msc_axis2mc_cclk_nocg, C_NOCG, axis, 1, 1) \
  _op_(msc_bpmp2mc_cclk_nocg, C_NOCG, bpmp, 1, 1) \
  _op_(msc_bpmpdma2mc_cclk_nocg, C_NOCG, bpmpdma, 1, 1) \
  _op_(msc_eqos2mc_cclk_nocg, C_NOCG, eqos, 1, 1) \
  _op_(msc_etr2mc_cclk_nocg, C_NOCG, etr, 1, 1) \
  _op_(msc_gpu2mc_cclk_nocg, C_NOCG, gpu, 1, 1) \
  _op_(msc_gpu22mc_cclk_nocg, C_NOCG, gpu2, 1, 1) \
  _op_(msc_hda2mc_cclk_nocg, C_NOCG, hda, 1, 1) \
  _op_(msc_host1x2mc_cclk_nocg, C_NOCG, host1x, 1, 1) \
  _op_(msc_isp22mc_cclk_nocg, C_NOCG, isp2, 1, 1) \
  _op_(msc_mpcore2mc_cclk_nocg, C_NOCG, mpcore, 1, 1) \
  _op_(msc_nvdec2mc_cclk_nocg, C_NOCG, nvdec, 1, 1) \
  _op_(msc_nvdec12mc_cclk_nocg, C_NOCG, nvdec1, 1, 1) \
  _op_(msc_nvdec32mc_cclk_nocg, C_NOCG, nvdec3, 1, 1) \
  _op_(msc_nvdisplay2mc_cclk_nocg, C_NOCG, nvdisplay, 1, 1) \
  _op_(msc_nvdisplay12mc_cclk_nocg, C_NOCG, nvdisplay1, 1, 1) \
  _op_(msc_nvenc2mc_cclk_nocg, C_NOCG, nvenc, 1, 1) \
  _op_(msc_nvenc22mc_cclk_nocg, C_NOCG, nvenc2, 1, 1) \
  _op_(msc_nvjpg2mc_cclk_nocg, C_NOCG, nvjpg, 1, 1) \
  _op_(msc_sata2mc_cclk_nocg, C_NOCG, sata, 1, 1) \
  _op_(msc_sce2mc_cclk_nocg, C_NOCG, sce, 1, 1) \
  _op_(msc_scedma2mc_cclk_nocg, C_NOCG, scedma, 1, 1) \
  _op_(msc_sdmmc2mc_cclk_nocg, C_NOCG, sdmmc, 1, 1) \
  _op_(msc_sdmmca2mc_cclk_nocg, C_NOCG, sdmmca, 1, 1) \
  _op_(msc_sdmmcaa2mc_cclk_nocg, C_NOCG, sdmmcaa, 1, 1) \
  _op_(msc_sdmmcab2mc_cclk_nocg, C_NOCG, sdmmcab, 1, 1) \
  _op_(msc_se2mc_cclk_nocg, C_NOCG, se, 1, 1) \
  _op_(msc_tsec2mc_cclk_nocg, C_NOCG, tsec, 1, 1) \
  _op_(msc_tsecb2mc_cclk_nocg, C_NOCG, tsecb, 1, 1) \
  _op_(msc_ufshc2mc_cclk_nocg, C_NOCG, ufshc, 1, 1) \
  _op_(msc_vi22mc_cclk_nocg, C_NOCG, vi2, 1, 1) \
  _op_(msc_vic2mc_cclk_nocg, C_NOCG, vic, 1, 1) \
  _op_(msc_vic12mc_cclk_nocg, C_NOCG, vic1, 1, 1) \
  _op_(msc_vic32mc_cclk_nocg, C_NOCG, vic3, 1, 1) \
  _op_(msc_xusb_dev2mc_cclk_nocg, C_NOCG, xusb_dev, 1, 1) \
  _op_(msc_xusb_host2mc_cclk_nocg, C_NOCG, xusb_host, 1, 1)

#define LIST_CLK_AFI_MSCCLOCKS(_op_) \
  _op_(msc_afir2mc_rclk) \
  _op_(msc_afir2mc_wclk) \
  _op_(csr_afir2mc_clk) \
  _op_(msc_afiw2mc_rclk) \
  _op_(msc_afiw2mc_wclk) \
  _op_(csw_afiw2mc_clk) \
  _op_(msc_afi2mc_cclk_nocg)

#define LIST_CLK_AON_MSCCLOCKS(_op_) \
  _op_(msc_aonr2mc_rclk) \
  _op_(msc_aonr2mc_wclk) \
  _op_(csr_aonr2mc_clk) \
  _op_(msc_aonw2mc_rclk) \
  _op_(msc_aonw2mc_wclk) \
  _op_(csw_aonw2mc_clk) \
  _op_(msc_aon2mc_cclk_nocg)

#define LIST_CLK_AONDMA_MSCCLOCKS(_op_) \
  _op_(msc_aondmar2mc_rclk) \
  _op_(msc_aondmar2mc_wclk) \
  _op_(csr_aondmar2mc_clk) \
  _op_(msc_aondmaw2mc_rclk) \
  _op_(msc_aondmaw2mc_wclk) \
  _op_(csw_aondmaw2mc_clk) \
  _op_(msc_aondma2mc_cclk_nocg)

#define LIST_CLK_APE_MSCCLOCKS(_op_) \
  _op_(msc_aper2mc_rclk) \
  _op_(msc_aper2mc_wclk) \
  _op_(csr_aper2mc_clk) \
  _op_(msc_apew2mc_rclk) \
  _op_(msc_apew2mc_wclk) \
  _op_(csw_apew2mc_clk) \
  _op_(msc_ape2mc_cclk_nocg)

#define LIST_CLK_APEDMA_MSCCLOCKS(_op_) \
  _op_(msc_apedmar2mc_rclk) \
  _op_(msc_apedmar2mc_wclk) \
  _op_(csr_apedmar2mc_clk) \
  _op_(msc_apedmaw2mc_rclk) \
  _op_(msc_apedmaw2mc_wclk) \
  _op_(csw_apedmaw2mc_clk) \
  _op_(msc_apedma2mc_cclk_nocg)

#define LIST_CLK_AXIS_MSCCLOCKS(_op_) \
  _op_(msc_axisr2mc_rclk) \
  _op_(msc_axisr2mc_wclk) \
  _op_(csr_axisr2mc_clk) \
  _op_(msc_axisw2mc_rclk) \
  _op_(msc_axisw2mc_wclk) \
  _op_(csw_axisw2mc_clk) \
  _op_(msc_axis2mc_cclk_nocg)

#define LIST_CLK_BPMP_MSCCLOCKS(_op_) \
  _op_(msc_bpmpr2mc_rclk) \
  _op_(msc_bpmpr2mc_wclk) \
  _op_(csr_bpmpr2mc_clk) \
  _op_(msc_bpmpw2mc_rclk) \
  _op_(msc_bpmpw2mc_wclk) \
  _op_(csw_bpmpw2mc_clk) \
  _op_(msc_bpmp2mc_cclk_nocg)

#define LIST_CLK_BPMPDMA_MSCCLOCKS(_op_) \
  _op_(msc_bpmpdmar2mc_rclk) \
  _op_(msc_bpmpdmar2mc_wclk) \
  _op_(csr_bpmpdmar2mc_clk) \
  _op_(msc_bpmpdmaw2mc_rclk) \
  _op_(msc_bpmpdmaw2mc_wclk) \
  _op_(csw_bpmpdmaw2mc_clk) \
  _op_(msc_bpmpdma2mc_cclk_nocg)

#define LIST_CLK_EQOS_MSCCLOCKS(_op_) \
  _op_(msc_eqosr2mc_rclk) \
  _op_(msc_eqosr2mc_wclk) \
  _op_(csr_eqosr2mc_clk) \
  _op_(msc_eqosw2mc_rclk) \
  _op_(msc_eqosw2mc_wclk) \
  _op_(csw_eqosw2mc_clk) \
  _op_(msc_eqos2mc_cclk_nocg)

#define LIST_CLK_ETR_MSCCLOCKS(_op_) \
  _op_(msc_etrr2mc_rclk) \
  _op_(msc_etrr2mc_wclk) \
  _op_(csr_etrr2mc_clk) \
  _op_(msc_etrw2mc_rclk) \
  _op_(msc_etrw2mc_wclk) \
  _op_(csw_etrw2mc_clk) \
  _op_(msc_etr2mc_cclk_nocg)

#define LIST_CLK_GPU_MSCCLOCKS(_op_) \
  _op_(msc_gpusrd2mc_rclk) \
  _op_(msc_gpusrd2mc_wclk) \
  _op_(csr_gpusrd2mc_clk) \
  _op_(msc_gpuswr2mc_rclk) \
  _op_(msc_gpuswr2mc_wclk) \
  _op_(csw_gpuswr2mc_clk) \
  _op_(msc_gpu2mc_cclk_nocg)

#define LIST_CLK_GPU2_MSCCLOCKS(_op_) \
  _op_(msc_gpusrd22mc_rclk) \
  _op_(msc_gpusrd22mc_wclk) \
  _op_(csr_gpusrd22mc_clk) \
  _op_(msc_gpuswr22mc_rclk) \
  _op_(msc_gpuswr22mc_wclk) \
  _op_(csw_gpuswr22mc_clk) \
  _op_(msc_gpu22mc_cclk_nocg)

#define LIST_CLK_HDA_MSCCLOCKS(_op_) \
  _op_(msc_hdar2mc_rclk) \
  _op_(msc_hdar2mc_wclk) \
  _op_(csr_hdar2mc_clk) \
  _op_(msc_hdaw2mc_rclk) \
  _op_(msc_hdaw2mc_wclk) \
  _op_(csw_hdaw2mc_clk) \
  _op_(msc_hda2mc_cclk_nocg)

#define LIST_CLK_HOST1X_MSCCLOCKS(_op_) \
  _op_(msc_host1xdmar2mc_rclk) \
  _op_(msc_host1xdmar2mc_wclk) \
  _op_(csr_host1xdmar2mc_clk) \
  _op_(msc_host1x2mc_cclk_nocg)

#define LIST_CLK_ISP2_MSCCLOCKS(_op_) \
  _op_(msc_ispra2mc_rclk) \
  _op_(msc_ispra2mc_wclk) \
  _op_(csr_ispra2mc_clk) \
  _op_(msc_ispwa2mc_rclk) \
  _op_(msc_ispwa2mc_wclk) \
  _op_(csw_ispwa2mc_clk) \
  _op_(msc_ispwb2mc_rclk) \
  _op_(msc_ispwb2mc_wclk) \
  _op_(csw_ispwb2mc_clk) \
  _op_(msc_isp22mc_cclk_nocg)

#define LIST_CLK_MPCORE_MSCCLOCKS(_op_) \
  _op_(msc_mpcorew2mc_rclk) \
  _op_(msc_mpcorew2mc_wclk) \
  _op_(csw_mpcorew2mc_clk) \
  _op_(msc_mll_mpcorer2mc_rclk) \
  _op_(msc_mll_mpcorer2mc_wclk) \
  _op_(csr_mpcorer2mc_clk) \
  _op_(msc_mpcore2mc_cclk_nocg) \
  _op_(mpcore2mc_clk)

#define LIST_CLK_NVDEC_MSCCLOCKS(_op_) \
  _op_(msc_nvdecsrd2mc_rclk) \
  _op_(msc_nvdecsrd2mc_wclk) \
  _op_(csr_nvdecsrd2mc_clk) \
  _op_(msc_nvdec2mc_cclk_nocg)

#define LIST_CLK_NVDEC1_MSCCLOCKS(_op_) \
  _op_(msc_nvdecsrd12mc_rclk) \
  _op_(msc_nvdecsrd12mc_wclk) \
  _op_(csr_nvdecsrd12mc_clk) \
  _op_(msc_nvdec12mc_cclk_nocg)

#define LIST_CLK_NVDEC3_MSCCLOCKS(_op_) \
  _op_(msc_nvdecswr2mc_rclk) \
  _op_(msc_nvdecswr2mc_wclk) \
  _op_(csw_nvdecswr2mc_clk) \
  _op_(msc_nvdec32mc_cclk_nocg)

#define LIST_CLK_NVDISPLAY_MSCCLOCKS(_op_) \
  _op_(msc_nvdisplayr2mc_rclk) \
  _op_(msc_nvdisplayr2mc_wclk) \
  _op_(csr_nvdisplayr2mc_clk) \
  _op_(msc_nvdisplay2mc_cclk_nocg)

#define LIST_CLK_NVDISPLAY1_MSCCLOCKS(_op_) \
  _op_(msc_nvdisplayr12mc_rclk) \
  _op_(msc_nvdisplayr12mc_wclk) \
  _op_(csr_nvdisplayr12mc_clk) \
  _op_(msc_nvdisplay12mc_cclk_nocg)

#define LIST_CLK_NVENC_MSCCLOCKS(_op_) \
  _op_(msc_nvencsrd2mc_rclk) \
  _op_(msc_nvencsrd2mc_wclk) \
  _op_(csr_nvencsrd2mc_clk) \
  _op_(msc_nvenc2mc_cclk_nocg)

#define LIST_CLK_NVENC2_MSCCLOCKS(_op_) \
  _op_(msc_nvencswr2mc_rclk) \
  _op_(msc_nvencswr2mc_wclk) \
  _op_(csw_nvencswr2mc_clk) \
  _op_(msc_nvenc22mc_cclk_nocg)

#define LIST_CLK_NVJPG_MSCCLOCKS(_op_) \
  _op_(msc_nvjpgsrd2mc_rclk) \
  _op_(msc_nvjpgsrd2mc_wclk) \
  _op_(csr_nvjpgsrd2mc_clk) \
  _op_(msc_nvjpgswr2mc_rclk) \
  _op_(msc_nvjpgswr2mc_wclk) \
  _op_(csw_nvjpgswr2mc_clk) \
  _op_(msc_nvjpg2mc_cclk_nocg)

#define LIST_CLK_SATA_MSCCLOCKS(_op_) \
  _op_(msc_satar2mc_rclk) \
  _op_(msc_satar2mc_wclk) \
  _op_(csr_satar2mc_clk) \
  _op_(msc_sataw2mc_rclk) \
  _op_(msc_sataw2mc_wclk) \
  _op_(csw_sataw2mc_clk) \
  _op_(msc_sata2mc_cclk_nocg)

#define LIST_CLK_SCE_MSCCLOCKS(_op_) \
  _op_(msc_scer2mc_rclk) \
  _op_(msc_scer2mc_wclk) \
  _op_(csr_scer2mc_clk) \
  _op_(msc_scew2mc_rclk) \
  _op_(msc_scew2mc_wclk) \
  _op_(csw_scew2mc_clk) \
  _op_(msc_sce2mc_cclk_nocg)

#define LIST_CLK_SCEDMA_MSCCLOCKS(_op_) \
  _op_(msc_scedmar2mc_rclk) \
  _op_(msc_scedmar2mc_wclk) \
  _op_(csr_scedmar2mc_clk) \
  _op_(msc_scedmaw2mc_rclk) \
  _op_(msc_scedmaw2mc_wclk) \
  _op_(csw_scedmaw2mc_clk) \
  _op_(msc_scedma2mc_cclk_nocg)

#define LIST_CLK_SDMMC_MSCCLOCKS(_op_) \
  _op_(msc_sdmmcr2mc_rclk) \
  _op_(msc_sdmmcr2mc_wclk) \
  _op_(csr_sdmmcr2mc_clk) \
  _op_(msc_sdmmcw2mc_rclk) \
  _op_(msc_sdmmcw2mc_wclk) \
  _op_(csw_sdmmcw2mc_clk) \
  _op_(msc_sdmmc2mc_cclk_nocg)

#define LIST_CLK_SDMMCA_MSCCLOCKS(_op_) \
  _op_(msc_sdmmcra2mc_rclk) \
  _op_(msc_sdmmcra2mc_wclk) \
  _op_(csr_sdmmcra2mc_clk) \
  _op_(msc_sdmmcwa2mc_rclk) \
  _op_(msc_sdmmcwa2mc_wclk) \
  _op_(csw_sdmmcwa2mc_clk) \
  _op_(msc_sdmmca2mc_cclk_nocg)

#define LIST_CLK_SDMMCAA_MSCCLOCKS(_op_) \
  _op_(msc_sdmmcraa2mc_rclk) \
  _op_(msc_sdmmcraa2mc_wclk) \
  _op_(csr_sdmmcraa2mc_clk) \
  _op_(msc_sdmmcwaa2mc_rclk) \
  _op_(msc_sdmmcwaa2mc_wclk) \
  _op_(csw_sdmmcwaa2mc_clk) \
  _op_(msc_sdmmcaa2mc_cclk_nocg)

#define LIST_CLK_SDMMCAB_MSCCLOCKS(_op_) \
  _op_(msc_sdmmcrab2mc_rclk) \
  _op_(msc_sdmmcrab2mc_wclk) \
  _op_(csr_sdmmcrab2mc_clk) \
  _op_(msc_sdmmcwab2mc_rclk) \
  _op_(msc_sdmmcwab2mc_wclk) \
  _op_(csw_sdmmcwab2mc_clk) \
  _op_(msc_sdmmcab2mc_cclk_nocg)

#define LIST_CLK_SE_MSCCLOCKS(_op_) \
  _op_(msc_sesrd2mc_rclk) \
  _op_(msc_sesrd2mc_wclk) \
  _op_(csr_sesrd2mc_clk) \
  _op_(msc_seswr2mc_rclk) \
  _op_(msc_seswr2mc_wclk) \
  _op_(csw_seswr2mc_clk) \
  _op_(msc_se2mc_cclk_nocg)

#define LIST_CLK_TSEC_MSCCLOCKS(_op_) \
  _op_(msc_tsecsrd2mc_rclk) \
  _op_(msc_tsecsrd2mc_wclk) \
  _op_(csr_tsecsrd2mc_clk) \
  _op_(msc_tsecswr2mc_rclk) \
  _op_(msc_tsecswr2mc_wclk) \
  _op_(csw_tsecswr2mc_clk) \
  _op_(msc_tsec2mc_cclk_nocg)

#define LIST_CLK_TSECB_MSCCLOCKS(_op_) \
  _op_(msc_tsecsrdb2mc_rclk) \
  _op_(msc_tsecsrdb2mc_wclk) \
  _op_(csr_tsecsrdb2mc_clk) \
  _op_(msc_tsecswrb2mc_rclk) \
  _op_(msc_tsecswrb2mc_wclk) \
  _op_(csw_tsecswrb2mc_clk) \
  _op_(msc_tsecb2mc_cclk_nocg)

#define LIST_CLK_UFSHC_MSCCLOCKS(_op_) \
  _op_(msc_ufshcr2mc_rclk) \
  _op_(msc_ufshcr2mc_wclk) \
  _op_(csr_ufshcr2mc_clk) \
  _op_(msc_ufshcw2mc_rclk) \
  _op_(msc_ufshcw2mc_wclk) \
  _op_(csw_ufshcw2mc_clk) \
  _op_(msc_ufshc2mc_cclk_nocg)

#define LIST_CLK_VI2_MSCCLOCKS(_op_) \
  _op_(msc_viw2mc_rclk) \
  _op_(msc_viw2mc_wclk) \
  _op_(csw_viw2mc_clk) \
  _op_(msc_vi22mc_cclk_nocg)

#define LIST_CLK_VIC_MSCCLOCKS(_op_) \
  _op_(msc_vicsrd2mc_rclk) \
  _op_(msc_vicsrd2mc_wclk) \
  _op_(csr_vicsrd2mc_clk) \
  _op_(msc_vic2mc_cclk_nocg)

#define LIST_CLK_VIC1_MSCCLOCKS(_op_) \
  _op_(msc_vicsrd12mc_rclk) \
  _op_(msc_vicsrd12mc_wclk) \
  _op_(csr_vicsrd12mc_clk) \
  _op_(msc_vic12mc_cclk_nocg)

#define LIST_CLK_VIC3_MSCCLOCKS(_op_) \
  _op_(msc_vicswr2mc_rclk) \
  _op_(msc_vicswr2mc_wclk) \
  _op_(csw_vicswr2mc_clk) \
  _op_(msc_vic32mc_cclk_nocg)

#define LIST_CLK_XUSB_DEV_MSCCLOCKS(_op_) \
  _op_(msc_xusb_devr2mc_rclk) \
  _op_(msc_xusb_devr2mc_wclk) \
  _op_(csr_xusb_devr2mc_clk) \
  _op_(msc_xusb_devw2mc_rclk) \
  _op_(msc_xusb_devw2mc_wclk) \
  _op_(csw_xusb_devw2mc_clk) \
  _op_(msc_xusb_dev2mc_cclk_nocg)

#define LIST_CLK_XUSB_HOST_MSCCLOCKS(_op_) \
  _op_(msc_xusb_hostr2mc_rclk) \
  _op_(msc_xusb_hostr2mc_wclk) \
  _op_(csr_xusb_hostr2mc_clk) \
  _op_(msc_xusb_hostw2mc_rclk) \
  _op_(msc_xusb_hostw2mc_wclk) \
  _op_(csw_xusb_hostw2mc_clk) \
  _op_(msc_xusb_host2mc_cclk_nocg)

#define LIST_ALL_LBW_PIPE_CLOCKS(_op_, master_ce) \
  _op_(mc_all_clk, master_ce)

#define LIST_CLK_MCLPCLOCKS(_op_, master_ce) \
  _op_(mc_aonpg_rclk, master_ce) \
  _op_(mc_aonpg_wclk, master_ce) \
  _op_(mc_aonpg_mcclk, master_ce) \
  _op_(mc_aud_rclk, master_ce) \
  _op_(mc_aud_wclk, master_ce) \
  _op_(mc_aud_mcclk, master_ce) \
  _op_(mc_bpmp_rclk, master_ce) \
  _op_(mc_bpmp_wclk, master_ce) \
  _op_(mc_bpmp_mcclk, master_ce) \
  _op_(mc_dfd_rclk, master_ce) \
  _op_(mc_dfd_wclk, master_ce) \
  _op_(mc_dfd_mcclk, master_ce) \
  _op_(mc_disiha_rclk, master_ce) \
  _op_(mc_disiha_wclk, master_ce) \
  _op_(mc_disiha_mcclk, master_ce) \
  _op_(mc_ftop_rclk, master_ce) \
  _op_(mc_ftop_wclk, master_ce) \
  _op_(mc_ftop_mcclk, master_ce) \
  _op_(mc_gr_rclk, master_ce) \
  _op_(mc_gr_wclk, master_ce) \
  _op_(mc_gr_mcclk, master_ce) \
  _op_(mc_grtbb_rclk, master_ce) \
  _op_(mc_grtbb_wclk, master_ce) \
  _op_(mc_grtbb_mcclk, master_ce) \
  _op_(mc_grtta_rclk, master_ce) \
  _op_(mc_grtta_wclk, master_ce) \
  _op_(mc_grtta_mcclk, master_ce) \
  _op_(mc_grttb_rclk, master_ce) \
  _op_(mc_grttb_wclk, master_ce) \
  _op_(mc_grttb_mcclk, master_ce) \
  _op_(mc_host_rclk, master_ce) \
  _op_(mc_host_wclk, master_ce) \
  _op_(mc_host_mcclk, master_ce) \
  _op_(mc_ispac_rclk, master_ce) \
  _op_(mc_ispac_wclk, master_ce) \
  _op_(mc_ispac_mcclk, master_ce) \
  _op_(mc_nic_rclk, master_ce) \
  _op_(mc_nic_wclk, master_ce) \
  _op_(mc_nic_mcclk, master_ce) \
  _op_(mc_nvdeca_rclk, master_ce) \
  _op_(mc_nvdeca_wclk, master_ce) \
  _op_(mc_nvdeca_mcclk, master_ce) \
  _op_(mc_nvencb_rclk, master_ce) \
  _op_(mc_nvencb_wclk, master_ce) \
  _op_(mc_nvencb_mcclk, master_ce) \
  _op_(mc_nvjpg_rclk, master_ce) \
  _op_(mc_nvjpg_wclk, master_ce) \
  _op_(mc_nvjpg_mcclk, master_ce) \
  _op_(mc_pcx_rclk, master_ce) \
  _op_(mc_pcx_wclk, master_ce) \
  _op_(mc_pcx_mcclk, master_ce) \
  _op_(mc_sax_rclk, master_ce) \
  _op_(mc_sax_wclk, master_ce) \
  _op_(mc_sax_mcclk, master_ce) \
  _op_(mc_sce_rclk, master_ce) \
  _op_(mc_sce_wclk, master_ce) \
  _op_(mc_sce_mcclk, master_ce) \
  _op_(mc_sec_rclk, master_ce) \
  _op_(mc_sec_wclk, master_ce) \
  _op_(mc_sec_mcclk, master_ce) \
  _op_(mc_sor_rclk, master_ce) \
  _op_(mc_sor_wclk, master_ce) \
  _op_(mc_sor_mcclk, master_ce) \
  _op_(mc_ufs_rclk, master_ce) \
  _op_(mc_ufs_wclk, master_ce) \
  _op_(mc_ufs_mcclk, master_ce) \
  _op_(mc_ve_rclk, master_ce) \
  _op_(mc_ve_wclk, master_ce) \
  _op_(mc_ve_mcclk, master_ce) \
  _op_(mc_vica_rclk, master_ce) \
  _op_(mc_vica_wclk, master_ce) \
  _op_(mc_vica_mcclk, master_ce) \
  _op_(mc_xusbb_rclk, master_ce) \
  _op_(mc_xusbb_wclk, master_ce) \
  _op_(mc_xusbb_mcclk, master_ce) \
  _op_(mc_xusbc_rclk, master_ce) \
  _op_(mc_xusbc_wclk, master_ce) \
  _op_(mc_xusbc_mcclk, master_ce) \
  _op_(mc_all_clk, master_ce)

#define LIST_CLK_AONPG2MC_W_CLKEN(_op_) \
  _op_(csr_aondmar2mc_r_clken) \
  _op_(csr_aondmar2mc_w_clken) \
  _op_(csw_aondmaw2mc_r_clken) \
  _op_(csw_aondmaw2mc_w_clken) \
  _op_(csr_aonr2mc_r_clken) \
  _op_(csr_aonr2mc_w_clken) \
  _op_(csw_aonw2mc_r_clken) \
  _op_(csw_aonw2mc_w_clken)
#define NV_MC_AONPG2MC_W_CLKEN  msc_aondmar2mc_wclk_cex_clkenx || msc_aondmaw2mc_wclk_cex_clkenx || msc_aonr2mc_wclk_cex_clkenx || msc_aonw2mc_wclk_cex_clkenx

#define LIST_CLK_AONPG2MC_CLKEN(_op_) \
  _op_(csr_aondmar2mc_c_clken) \
  _op_(csw_aondmaw2mc_c_clken) \
  _op_(csr_aonr2mc_c_clken) \
  _op_(csw_aonw2mc_c_clken)
#define LIST_CLK_AUD2MC_W_CLKEN(_op_) \
  _op_(csr_apedmar2mc_r_clken) \
  _op_(csr_apedmar2mc_w_clken) \
  _op_(csw_apedmaw2mc_r_clken) \
  _op_(csw_apedmaw2mc_w_clken) \
  _op_(csr_aper2mc_r_clken) \
  _op_(csr_aper2mc_w_clken) \
  _op_(csw_apew2mc_r_clken) \
  _op_(csw_apew2mc_w_clken)
#define NV_MC_AUD2MC_W_CLKEN  msc_apedmar2mc_wclk_cex_clkenx || msc_apedmaw2mc_wclk_cex_clkenx || msc_aper2mc_wclk_cex_clkenx || msc_apew2mc_wclk_cex_clkenx

#define LIST_CLK_AUD2MC_CLKEN(_op_) \
  _op_(csr_apedmar2mc_c_clken) \
  _op_(csw_apedmaw2mc_c_clken) \
  _op_(csr_aper2mc_c_clken) \
  _op_(csw_apew2mc_c_clken)
#define LIST_CLK_BPMP2MC_W_CLKEN(_op_) \
  _op_(csr_bpmpdmar2mc_r_clken) \
  _op_(csr_bpmpdmar2mc_w_clken) \
  _op_(csw_bpmpdmaw2mc_r_clken) \
  _op_(csw_bpmpdmaw2mc_w_clken) \
  _op_(csr_bpmpr2mc_r_clken) \
  _op_(csr_bpmpr2mc_w_clken) \
  _op_(csw_bpmpw2mc_r_clken) \
  _op_(csw_bpmpw2mc_w_clken)
#define NV_MC_BPMP2MC_W_CLKEN  msc_bpmpdmar2mc_wclk_cex_clkenx || msc_bpmpdmaw2mc_wclk_cex_clkenx || msc_bpmpr2mc_wclk_cex_clkenx || msc_bpmpw2mc_wclk_cex_clkenx

#define LIST_CLK_BPMP2MC_CLKEN(_op_) \
  _op_(csr_bpmpdmar2mc_c_clken) \
  _op_(csw_bpmpdmaw2mc_c_clken) \
  _op_(csr_bpmpr2mc_c_clken) \
  _op_(csw_bpmpw2mc_c_clken)
#define LIST_CLK_DFD2MC_W_CLKEN(_op_) \
  _op_(csr_etrr2mc_r_clken) \
  _op_(csr_etrr2mc_w_clken) \
  _op_(csw_etrw2mc_r_clken) \
  _op_(csw_etrw2mc_w_clken)
#define NV_MC_DFD2MC_W_CLKEN  msc_etrr2mc_wclk_cex_clkenx || msc_etrw2mc_wclk_cex_clkenx

#define LIST_CLK_DFD2MC_CLKEN(_op_) \
  _op_(csr_etrr2mc_c_clken) \
  _op_(csw_etrw2mc_c_clken)
#define LIST_CLK_DISIHA2MC_W_CLKEN(_op_) \
  _op_(csr_nvdisplayr2mc_r_clken) \
  _op_(csr_nvdisplayr2mc_w_clken) \
  _op_(csr_nvdisplayr12mc_r_clken) \
  _op_(csr_nvdisplayr12mc_w_clken)
#define NV_MC_DISIHA2MC_W_CLKEN  msc_nvdisplayr2mc_wclk_cex_clkenx || msc_nvdisplayr12mc_wclk_cex_clkenx

#define LIST_CLK_DISIHA2MC_CLKEN(_op_) \
  _op_(csr_nvdisplayr2mc_c_clken) \
  _op_(csr_nvdisplayr12mc_c_clken)
#define LIST_CLK_FTOP2MC_W_CLKEN(_op_) \
  _op_(csw_mpcorew2mc_r_clken) \
  _op_(csw_mpcorew2mc_w_clken) \
  _op_(csr_mpcorer2mc_r_clken) \
  _op_(csr_mpcorer2mc_w_clken)
#define NV_MC_FTOP2MC_W_CLKEN  msc_mpcorew2mc_wclk_cex_clkenx || msc_mll_mpcorer2mc_wclk_cex_clkenx

#define LIST_CLK_FTOP2MC_CLKEN(_op_) \
  _op_(csw_mpcorew2mc_c_clken) \
  _op_(csr_mpcorer2mc_c_clken)
#define LIST_CLK_GR2MC_W_CLKEN(_op_) \
  _op_(csr_gpusrd2mc_r_clken) \
  _op_(csr_gpusrd2mc_w_clken) \
  _op_(csw_gpuswr2mc_r_clken) \
  _op_(csw_gpuswr2mc_w_clken) \
  _op_(csr_gpusrd22mc_r_clken) \
  _op_(csr_gpusrd22mc_w_clken) \
  _op_(csw_gpuswr22mc_r_clken) \
  _op_(csw_gpuswr22mc_w_clken)
#define NV_MC_GR2MC_W_CLKEN  msc_gpusrd2mc_wclk_cex_clkenx || msc_gpuswr2mc_wclk_cex_clkenx || msc_gpusrd22mc_wclk_cex_clkenx || msc_gpuswr22mc_wclk_cex_clkenx

#define LIST_CLK_GR2MC_CLKEN(_op_) \
  _op_(csr_gpusrd2mc_c_clken) \
  _op_(csw_gpuswr2mc_c_clken) \
  _op_(csr_gpusrd22mc_c_clken) \
  _op_(csw_gpuswr22mc_c_clken)
#define LIST_CLK_GRTBB2MC_W_CLKEN(_op_) \
  _op_(csr_sdmmcraa2mc_r_clken) \
  _op_(csr_sdmmcraa2mc_w_clken) \
  _op_(csw_sdmmcwaa2mc_r_clken) \
  _op_(csw_sdmmcwaa2mc_w_clken)
#define NV_MC_GRTBB2MC_W_CLKEN  msc_sdmmcraa2mc_wclk_cex_clkenx || msc_sdmmcwaa2mc_wclk_cex_clkenx

#define LIST_CLK_GRTBB2MC_CLKEN(_op_) \
  _op_(csr_sdmmcraa2mc_c_clken) \
  _op_(csw_sdmmcwaa2mc_c_clken)
#define LIST_CLK_GRTTA2MC_W_CLKEN(_op_) \
  _op_(csr_sdmmcra2mc_r_clken) \
  _op_(csr_sdmmcra2mc_w_clken) \
  _op_(csr_sdmmcrab2mc_r_clken) \
  _op_(csr_sdmmcrab2mc_w_clken) \
  _op_(csw_sdmmcwa2mc_r_clken) \
  _op_(csw_sdmmcwa2mc_w_clken) \
  _op_(csw_sdmmcwab2mc_r_clken) \
  _op_(csw_sdmmcwab2mc_w_clken)
#define NV_MC_GRTTA2MC_W_CLKEN  msc_sdmmcra2mc_wclk_cex_clkenx || msc_sdmmcrab2mc_wclk_cex_clkenx || msc_sdmmcwa2mc_wclk_cex_clkenx || msc_sdmmcwab2mc_wclk_cex_clkenx

#define LIST_CLK_GRTTA2MC_CLKEN(_op_) \
  _op_(csr_sdmmcra2mc_c_clken) \
  _op_(csr_sdmmcrab2mc_c_clken) \
  _op_(csw_sdmmcwa2mc_c_clken) \
  _op_(csw_sdmmcwab2mc_c_clken)
#define LIST_CLK_GRTTB2MC_W_CLKEN(_op_) \
  _op_(csr_sdmmcr2mc_r_clken) \
  _op_(csr_sdmmcr2mc_w_clken) \
  _op_(csw_sdmmcw2mc_r_clken) \
  _op_(csw_sdmmcw2mc_w_clken)
#define NV_MC_GRTTB2MC_W_CLKEN  msc_sdmmcr2mc_wclk_cex_clkenx || msc_sdmmcw2mc_wclk_cex_clkenx

#define LIST_CLK_GRTTB2MC_CLKEN(_op_) \
  _op_(csr_sdmmcr2mc_c_clken) \
  _op_(csw_sdmmcw2mc_c_clken)
#define LIST_CLK_HOST2MC_W_CLKEN(_op_) \
  _op_(csr_host1xdmar2mc_r_clken) \
  _op_(csr_host1xdmar2mc_w_clken)
#define NV_MC_HOST2MC_W_CLKEN  msc_host1xdmar2mc_wclk_cex_clkenx

#define LIST_CLK_HOST2MC_CLKEN(_op_) \
  _op_(csr_host1xdmar2mc_c_clken)
#define LIST_CLK_ISPAC2MC_W_CLKEN(_op_) \
  _op_(csr_ispra2mc_r_clken) \
  _op_(csr_ispra2mc_w_clken) \
  _op_(csw_ispwa2mc_r_clken) \
  _op_(csw_ispwa2mc_w_clken) \
  _op_(csw_ispwb2mc_r_clken) \
  _op_(csw_ispwb2mc_w_clken)
#define NV_MC_ISPAC2MC_W_CLKEN  msc_ispra2mc_wclk_cex_clkenx || msc_ispwa2mc_wclk_cex_clkenx || msc_ispwb2mc_wclk_cex_clkenx

#define LIST_CLK_ISPAC2MC_CLKEN(_op_) \
  _op_(csr_ispra2mc_c_clken) \
  _op_(csw_ispwa2mc_c_clken) \
  _op_(csw_ispwb2mc_c_clken)
#define LIST_CLK_NIC2MC_W_CLKEN(_op_) \
  _op_(csr_axisr2mc_r_clken) \
  _op_(csr_axisr2mc_w_clken) \
  _op_(csw_axisw2mc_r_clken) \
  _op_(csw_axisw2mc_w_clken)
#define NV_MC_NIC2MC_W_CLKEN  msc_axisr2mc_wclk_cex_clkenx || msc_axisw2mc_wclk_cex_clkenx

#define LIST_CLK_NIC2MC_CLKEN(_op_) \
  _op_(csr_axisr2mc_c_clken) \
  _op_(csw_axisw2mc_c_clken)
#define LIST_CLK_NVDECA2MC_W_CLKEN(_op_) \
  _op_(csr_nvdecsrd2mc_r_clken) \
  _op_(csr_nvdecsrd2mc_w_clken) \
  _op_(csr_nvdecsrd12mc_r_clken) \
  _op_(csr_nvdecsrd12mc_w_clken) \
  _op_(csw_nvdecswr2mc_r_clken) \
  _op_(csw_nvdecswr2mc_w_clken)
#define NV_MC_NVDECA2MC_W_CLKEN  msc_nvdecsrd2mc_wclk_cex_clkenx || msc_nvdecsrd12mc_wclk_cex_clkenx || msc_nvdecswr2mc_wclk_cex_clkenx

#define LIST_CLK_NVDECA2MC_CLKEN(_op_) \
  _op_(csr_nvdecsrd2mc_c_clken) \
  _op_(csr_nvdecsrd12mc_c_clken) \
  _op_(csw_nvdecswr2mc_c_clken)
#define LIST_CLK_NVENCB2MC_W_CLKEN(_op_) \
  _op_(csr_nvencsrd2mc_r_clken) \
  _op_(csr_nvencsrd2mc_w_clken) \
  _op_(csw_nvencswr2mc_r_clken) \
  _op_(csw_nvencswr2mc_w_clken)
#define NV_MC_NVENCB2MC_W_CLKEN  msc_nvencsrd2mc_wclk_cex_clkenx || msc_nvencswr2mc_wclk_cex_clkenx

#define LIST_CLK_NVENCB2MC_CLKEN(_op_) \
  _op_(csr_nvencsrd2mc_c_clken) \
  _op_(csw_nvencswr2mc_c_clken)
#define LIST_CLK_NVJPG2MC_W_CLKEN(_op_) \
  _op_(csr_nvjpgsrd2mc_r_clken) \
  _op_(csr_nvjpgsrd2mc_w_clken) \
  _op_(csw_nvjpgswr2mc_r_clken) \
  _op_(csw_nvjpgswr2mc_w_clken)
#define NV_MC_NVJPG2MC_W_CLKEN  msc_nvjpgsrd2mc_wclk_cex_clkenx || msc_nvjpgswr2mc_wclk_cex_clkenx

#define LIST_CLK_NVJPG2MC_CLKEN(_op_) \
  _op_(csr_nvjpgsrd2mc_c_clken) \
  _op_(csw_nvjpgswr2mc_c_clken)
#define LIST_CLK_PCX2MC_W_CLKEN(_op_) \
  _op_(csr_afir2mc_r_clken) \
  _op_(csr_afir2mc_w_clken) \
  _op_(csw_afiw2mc_r_clken) \
  _op_(csw_afiw2mc_w_clken)
#define NV_MC_PCX2MC_W_CLKEN  msc_afir2mc_wclk_cex_clkenx || msc_afiw2mc_wclk_cex_clkenx

#define LIST_CLK_PCX2MC_CLKEN(_op_) \
  _op_(csr_afir2mc_c_clken) \
  _op_(csw_afiw2mc_c_clken)
#define LIST_CLK_SAX2MC_W_CLKEN(_op_) \
  _op_(csr_satar2mc_r_clken) \
  _op_(csr_satar2mc_w_clken) \
  _op_(csw_sataw2mc_r_clken) \
  _op_(csw_sataw2mc_w_clken)
#define NV_MC_SAX2MC_W_CLKEN  msc_satar2mc_wclk_cex_clkenx || msc_sataw2mc_wclk_cex_clkenx

#define LIST_CLK_SAX2MC_CLKEN(_op_) \
  _op_(csr_satar2mc_c_clken) \
  _op_(csw_sataw2mc_c_clken)
#define LIST_CLK_SCE2MC_W_CLKEN(_op_) \
  _op_(csr_scedmar2mc_r_clken) \
  _op_(csr_scedmar2mc_w_clken) \
  _op_(csw_scedmaw2mc_r_clken) \
  _op_(csw_scedmaw2mc_w_clken) \
  _op_(csr_scer2mc_r_clken) \
  _op_(csr_scer2mc_w_clken) \
  _op_(csw_scew2mc_r_clken) \
  _op_(csw_scew2mc_w_clken)
#define NV_MC_SCE2MC_W_CLKEN  msc_scedmar2mc_wclk_cex_clkenx || msc_scedmaw2mc_wclk_cex_clkenx || msc_scer2mc_wclk_cex_clkenx || msc_scew2mc_wclk_cex_clkenx

#define LIST_CLK_SCE2MC_CLKEN(_op_) \
  _op_(csr_scedmar2mc_c_clken) \
  _op_(csw_scedmaw2mc_c_clken) \
  _op_(csr_scer2mc_c_clken) \
  _op_(csw_scew2mc_c_clken)
#define LIST_CLK_SEC2MC_W_CLKEN(_op_) \
  _op_(csr_sesrd2mc_r_clken) \
  _op_(csr_sesrd2mc_w_clken) \
  _op_(csr_tsecsrd2mc_r_clken) \
  _op_(csr_tsecsrd2mc_w_clken) \
  _op_(csr_tsecsrdb2mc_r_clken) \
  _op_(csr_tsecsrdb2mc_w_clken) \
  _op_(csw_seswr2mc_r_clken) \
  _op_(csw_seswr2mc_w_clken) \
  _op_(csw_tsecswr2mc_r_clken) \
  _op_(csw_tsecswr2mc_w_clken) \
  _op_(csw_tsecswrb2mc_r_clken) \
  _op_(csw_tsecswrb2mc_w_clken)
#define NV_MC_SEC2MC_W_CLKEN  msc_sesrd2mc_wclk_cex_clkenx || msc_tsecsrd2mc_wclk_cex_clkenx || msc_tsecsrdb2mc_wclk_cex_clkenx || msc_seswr2mc_wclk_cex_clkenx || msc_tsecswr2mc_wclk_cex_clkenx || msc_tsecswrb2mc_wclk_cex_clkenx

#define LIST_CLK_SEC2MC_CLKEN(_op_) \
  _op_(csr_sesrd2mc_c_clken) \
  _op_(csr_tsecsrd2mc_c_clken) \
  _op_(csr_tsecsrdb2mc_c_clken) \
  _op_(csw_seswr2mc_c_clken) \
  _op_(csw_tsecswr2mc_c_clken) \
  _op_(csw_tsecswrb2mc_c_clken)
#define LIST_CLK_SOR2MC_W_CLKEN(_op_) \
  _op_(csr_hdar2mc_r_clken) \
  _op_(csr_hdar2mc_w_clken) \
  _op_(csw_hdaw2mc_r_clken) \
  _op_(csw_hdaw2mc_w_clken)
#define NV_MC_SOR2MC_W_CLKEN  msc_hdar2mc_wclk_cex_clkenx || msc_hdaw2mc_wclk_cex_clkenx

#define LIST_CLK_SOR2MC_CLKEN(_op_) \
  _op_(csr_hdar2mc_c_clken) \
  _op_(csw_hdaw2mc_c_clken)
#define LIST_CLK_UFS2MC_W_CLKEN(_op_) \
  _op_(csr_eqosr2mc_r_clken) \
  _op_(csr_eqosr2mc_w_clken) \
  _op_(csw_eqosw2mc_r_clken) \
  _op_(csw_eqosw2mc_w_clken) \
  _op_(csr_ufshcr2mc_r_clken) \
  _op_(csr_ufshcr2mc_w_clken) \
  _op_(csw_ufshcw2mc_r_clken) \
  _op_(csw_ufshcw2mc_w_clken)
#define NV_MC_UFS2MC_W_CLKEN  msc_eqosr2mc_wclk_cex_clkenx || msc_eqosw2mc_wclk_cex_clkenx || msc_ufshcr2mc_wclk_cex_clkenx || msc_ufshcw2mc_wclk_cex_clkenx

#define LIST_CLK_UFS2MC_CLKEN(_op_) \
  _op_(csr_eqosr2mc_c_clken) \
  _op_(csw_eqosw2mc_c_clken) \
  _op_(csr_ufshcr2mc_c_clken) \
  _op_(csw_ufshcw2mc_c_clken)
#define LIST_CLK_VE2MC_W_CLKEN(_op_) \
  _op_(csw_viw2mc_r_clken) \
  _op_(csw_viw2mc_w_clken)
#define NV_MC_VE2MC_W_CLKEN  msc_viw2mc_wclk_cex_clkenx

#define LIST_CLK_VE2MC_CLKEN(_op_) \
  _op_(csw_viw2mc_c_clken)
#define LIST_CLK_VICA2MC_W_CLKEN(_op_) \
  _op_(csr_vicsrd2mc_r_clken) \
  _op_(csr_vicsrd2mc_w_clken) \
  _op_(csr_vicsrd12mc_r_clken) \
  _op_(csr_vicsrd12mc_w_clken) \
  _op_(csw_vicswr2mc_r_clken) \
  _op_(csw_vicswr2mc_w_clken)
#define NV_MC_VICA2MC_W_CLKEN  msc_vicsrd2mc_wclk_cex_clkenx || msc_vicsrd12mc_wclk_cex_clkenx || msc_vicswr2mc_wclk_cex_clkenx

#define LIST_CLK_VICA2MC_CLKEN(_op_) \
  _op_(csr_vicsrd2mc_c_clken) \
  _op_(csr_vicsrd12mc_c_clken) \
  _op_(csw_vicswr2mc_c_clken)
#define LIST_CLK_XUSBB2MC_W_CLKEN(_op_) \
  _op_(csr_xusb_devr2mc_r_clken) \
  _op_(csr_xusb_devr2mc_w_clken) \
  _op_(csw_xusb_devw2mc_r_clken) \
  _op_(csw_xusb_devw2mc_w_clken)
#define NV_MC_XUSBB2MC_W_CLKEN  msc_xusb_devr2mc_wclk_cex_clkenx || msc_xusb_devw2mc_wclk_cex_clkenx

#define LIST_CLK_XUSBB2MC_CLKEN(_op_) \
  _op_(csr_xusb_devr2mc_c_clken) \
  _op_(csw_xusb_devw2mc_c_clken)
#define LIST_CLK_XUSBC2MC_W_CLKEN(_op_) \
  _op_(csr_xusb_hostr2mc_r_clken) \
  _op_(csr_xusb_hostr2mc_w_clken) \
  _op_(csw_xusb_hostw2mc_r_clken) \
  _op_(csw_xusb_hostw2mc_w_clken)
#define NV_MC_XUSBC2MC_W_CLKEN  msc_xusb_hostr2mc_wclk_cex_clkenx || msc_xusb_hostw2mc_wclk_cex_clkenx

#define LIST_CLK_XUSBC2MC_CLKEN(_op_) \
  _op_(csr_xusb_hostr2mc_c_clken) \
  _op_(csw_xusb_hostw2mc_c_clken)
#define NV_MC_SC2MC_CLKEN_INIT(SC)  NV_MC_##SC##2MC_CLKEN_INIT
#define NV_MC_AFI2MC_CLKEN_INIT \
  if (csr_afir2mc_c_clken_PORT->GetBus()) { UINT clken = 1; csr_afir2mc_c_clken_PORT->Write(&clken); } \
  if (csr_afir2mc_cclk_wakeup_PORT->GetBus()) { UINT clken = 1; csr_afir2mc_cclk_wakeup_PORT->Write(&clken); } \
  if (csr_afir2mc_r_clken_PORT->GetBus()) { UINT clken = 1; csr_afir2mc_r_clken_PORT->Write(&clken); } \
  if (csr_afir2mc_w_clken_PORT->GetBus()) { UINT clken = 1; csr_afir2mc_w_clken_PORT->Write(&clken); } \
  if (csw_afiw2mc_c_clken_PORT->GetBus()) { UINT clken = 1; csw_afiw2mc_c_clken_PORT->Write(&clken); } \
  if (csw_afiw2mc_cclk_wakeup_PORT->GetBus()) { UINT clken = 1; csw_afiw2mc_cclk_wakeup_PORT->Write(&clken); } \
  if (csw_afiw2mc_r_clken_PORT->GetBus()) { UINT clken = 1; csw_afiw2mc_r_clken_PORT->Write(&clken); } \
  if (csw_afiw2mc_w_clken_PORT->GetBus()) { UINT clken = 1; csw_afiw2mc_w_clken_PORT->Write(&clken); }
#define NV_MC_AON2MC_CLKEN_INIT \
  if (csr_aonr2mc_c_clken_PORT->GetBus()) { UINT clken = 1; csr_aonr2mc_c_clken_PORT->Write(&clken); } \
  if (csr_aonr2mc_cclk_wakeup_PORT->GetBus()) { UINT clken = 1; csr_aonr2mc_cclk_wakeup_PORT->Write(&clken); } \
  if (csr_aonr2mc_r_clken_PORT->GetBus()) { UINT clken = 1; csr_aonr2mc_r_clken_PORT->Write(&clken); } \
  if (csr_aonr2mc_w_clken_PORT->GetBus()) { UINT clken = 1; csr_aonr2mc_w_clken_PORT->Write(&clken); } \
  if (csw_aonw2mc_c_clken_PORT->GetBus()) { UINT clken = 1; csw_aonw2mc_c_clken_PORT->Write(&clken); } \
  if (csw_aonw2mc_cclk_wakeup_PORT->GetBus()) { UINT clken = 1; csw_aonw2mc_cclk_wakeup_PORT->Write(&clken); } \
  if (csw_aonw2mc_r_clken_PORT->GetBus()) { UINT clken = 1; csw_aonw2mc_r_clken_PORT->Write(&clken); } \
  if (csw_aonw2mc_w_clken_PORT->GetBus()) { UINT clken = 1; csw_aonw2mc_w_clken_PORT->Write(&clken); }
#define NV_MC_AONDMA2MC_CLKEN_INIT \
  if (csr_aondmar2mc_c_clken_PORT->GetBus()) { UINT clken = 1; csr_aondmar2mc_c_clken_PORT->Write(&clken); } \
  if (csr_aondmar2mc_cclk_wakeup_PORT->GetBus()) { UINT clken = 1; csr_aondmar2mc_cclk_wakeup_PORT->Write(&clken); } \
  if (csr_aondmar2mc_r_clken_PORT->GetBus()) { UINT clken = 1; csr_aondmar2mc_r_clken_PORT->Write(&clken); } \
  if (csr_aondmar2mc_w_clken_PORT->GetBus()) { UINT clken = 1; csr_aondmar2mc_w_clken_PORT->Write(&clken); } \
  if (csw_aondmaw2mc_c_clken_PORT->GetBus()) { UINT clken = 1; csw_aondmaw2mc_c_clken_PORT->Write(&clken); } \
  if (csw_aondmaw2mc_cclk_wakeup_PORT->GetBus()) { UINT clken = 1; csw_aondmaw2mc_cclk_wakeup_PORT->Write(&clken); } \
  if (csw_aondmaw2mc_r_clken_PORT->GetBus()) { UINT clken = 1; csw_aondmaw2mc_r_clken_PORT->Write(&clken); } \
  if (csw_aondmaw2mc_w_clken_PORT->GetBus()) { UINT clken = 1; csw_aondmaw2mc_w_clken_PORT->Write(&clken); }
#define NV_MC_APE2MC_CLKEN_INIT \
  if (csr_aper2mc_c_clken_PORT->GetBus()) { UINT clken = 1; csr_aper2mc_c_clken_PORT->Write(&clken); } \
  if (csr_aper2mc_cclk_wakeup_PORT->GetBus()) { UINT clken = 1; csr_aper2mc_cclk_wakeup_PORT->Write(&clken); } \
  if (csr_aper2mc_r_clken_PORT->GetBus()) { UINT clken = 1; csr_aper2mc_r_clken_PORT->Write(&clken); } \
  if (csr_aper2mc_w_clken_PORT->GetBus()) { UINT clken = 1; csr_aper2mc_w_clken_PORT->Write(&clken); } \
  if (csw_apew2mc_c_clken_PORT->GetBus()) { UINT clken = 1; csw_apew2mc_c_clken_PORT->Write(&clken); } \
  if (csw_apew2mc_cclk_wakeup_PORT->GetBus()) { UINT clken = 1; csw_apew2mc_cclk_wakeup_PORT->Write(&clken); } \
  if (csw_apew2mc_r_clken_PORT->GetBus()) { UINT clken = 1; csw_apew2mc_r_clken_PORT->Write(&clken); } \
  if (csw_apew2mc_w_clken_PORT->GetBus()) { UINT clken = 1; csw_apew2mc_w_clken_PORT->Write(&clken); }
#define NV_MC_APEDMA2MC_CLKEN_INIT \
  if (csr_apedmar2mc_c_clken_PORT->GetBus()) { UINT clken = 1; csr_apedmar2mc_c_clken_PORT->Write(&clken); } \
  if (csr_apedmar2mc_cclk_wakeup_PORT->GetBus()) { UINT clken = 1; csr_apedmar2mc_cclk_wakeup_PORT->Write(&clken); } \
  if (csr_apedmar2mc_r_clken_PORT->GetBus()) { UINT clken = 1; csr_apedmar2mc_r_clken_PORT->Write(&clken); } \
  if (csr_apedmar2mc_w_clken_PORT->GetBus()) { UINT clken = 1; csr_apedmar2mc_w_clken_PORT->Write(&clken); } \
  if (csw_apedmaw2mc_c_clken_PORT->GetBus()) { UINT clken = 1; csw_apedmaw2mc_c_clken_PORT->Write(&clken); } \
  if (csw_apedmaw2mc_cclk_wakeup_PORT->GetBus()) { UINT clken = 1; csw_apedmaw2mc_cclk_wakeup_PORT->Write(&clken); } \
  if (csw_apedmaw2mc_r_clken_PORT->GetBus()) { UINT clken = 1; csw_apedmaw2mc_r_clken_PORT->Write(&clken); } \
  if (csw_apedmaw2mc_w_clken_PORT->GetBus()) { UINT clken = 1; csw_apedmaw2mc_w_clken_PORT->Write(&clken); }
#define NV_MC_AXIS2MC_CLKEN_INIT \
  if (csr_axisr2mc_c_clken_PORT->GetBus()) { UINT clken = 1; csr_axisr2mc_c_clken_PORT->Write(&clken); } \
  if (csr_axisr2mc_cclk_wakeup_PORT->GetBus()) { UINT clken = 1; csr_axisr2mc_cclk_wakeup_PORT->Write(&clken); } \
  if (csr_axisr2mc_r_clken_PORT->GetBus()) { UINT clken = 1; csr_axisr2mc_r_clken_PORT->Write(&clken); } \
  if (csr_axisr2mc_w_clken_PORT->GetBus()) { UINT clken = 1; csr_axisr2mc_w_clken_PORT->Write(&clken); } \
  if (csw_axisw2mc_c_clken_PORT->GetBus()) { UINT clken = 1; csw_axisw2mc_c_clken_PORT->Write(&clken); } \
  if (csw_axisw2mc_cclk_wakeup_PORT->GetBus()) { UINT clken = 1; csw_axisw2mc_cclk_wakeup_PORT->Write(&clken); } \
  if (csw_axisw2mc_r_clken_PORT->GetBus()) { UINT clken = 1; csw_axisw2mc_r_clken_PORT->Write(&clken); } \
  if (csw_axisw2mc_w_clken_PORT->GetBus()) { UINT clken = 1; csw_axisw2mc_w_clken_PORT->Write(&clken); }
#define NV_MC_BPMP2MC_CLKEN_INIT \
  if (csr_bpmpr2mc_c_clken_PORT->GetBus()) { UINT clken = 1; csr_bpmpr2mc_c_clken_PORT->Write(&clken); } \
  if (csr_bpmpr2mc_cclk_wakeup_PORT->GetBus()) { UINT clken = 1; csr_bpmpr2mc_cclk_wakeup_PORT->Write(&clken); } \
  if (csr_bpmpr2mc_r_clken_PORT->GetBus()) { UINT clken = 1; csr_bpmpr2mc_r_clken_PORT->Write(&clken); } \
  if (csr_bpmpr2mc_w_clken_PORT->GetBus()) { UINT clken = 1; csr_bpmpr2mc_w_clken_PORT->Write(&clken); } \
  if (csw_bpmpw2mc_c_clken_PORT->GetBus()) { UINT clken = 1; csw_bpmpw2mc_c_clken_PORT->Write(&clken); } \
  if (csw_bpmpw2mc_cclk_wakeup_PORT->GetBus()) { UINT clken = 1; csw_bpmpw2mc_cclk_wakeup_PORT->Write(&clken); } \
  if (csw_bpmpw2mc_r_clken_PORT->GetBus()) { UINT clken = 1; csw_bpmpw2mc_r_clken_PORT->Write(&clken); } \
  if (csw_bpmpw2mc_w_clken_PORT->GetBus()) { UINT clken = 1; csw_bpmpw2mc_w_clken_PORT->Write(&clken); }
#define NV_MC_BPMPDMA2MC_CLKEN_INIT \
  if (csr_bpmpdmar2mc_c_clken_PORT->GetBus()) { UINT clken = 1; csr_bpmpdmar2mc_c_clken_PORT->Write(&clken); } \
  if (csr_bpmpdmar2mc_cclk_wakeup_PORT->GetBus()) { UINT clken = 1; csr_bpmpdmar2mc_cclk_wakeup_PORT->Write(&clken); } \
  if (csr_bpmpdmar2mc_r_clken_PORT->GetBus()) { UINT clken = 1; csr_bpmpdmar2mc_r_clken_PORT->Write(&clken); } \
  if (csr_bpmpdmar2mc_w_clken_PORT->GetBus()) { UINT clken = 1; csr_bpmpdmar2mc_w_clken_PORT->Write(&clken); } \
  if (csw_bpmpdmaw2mc_c_clken_PORT->GetBus()) { UINT clken = 1; csw_bpmpdmaw2mc_c_clken_PORT->Write(&clken); } \
  if (csw_bpmpdmaw2mc_cclk_wakeup_PORT->GetBus()) { UINT clken = 1; csw_bpmpdmaw2mc_cclk_wakeup_PORT->Write(&clken); } \
  if (csw_bpmpdmaw2mc_r_clken_PORT->GetBus()) { UINT clken = 1; csw_bpmpdmaw2mc_r_clken_PORT->Write(&clken); } \
  if (csw_bpmpdmaw2mc_w_clken_PORT->GetBus()) { UINT clken = 1; csw_bpmpdmaw2mc_w_clken_PORT->Write(&clken); }
#define NV_MC_EQOS2MC_CLKEN_INIT \
  if (csr_eqosr2mc_c_clken_PORT->GetBus()) { UINT clken = 1; csr_eqosr2mc_c_clken_PORT->Write(&clken); } \
  if (csr_eqosr2mc_cclk_wakeup_PORT->GetBus()) { UINT clken = 1; csr_eqosr2mc_cclk_wakeup_PORT->Write(&clken); } \
  if (csr_eqosr2mc_r_clken_PORT->GetBus()) { UINT clken = 1; csr_eqosr2mc_r_clken_PORT->Write(&clken); } \
  if (csr_eqosr2mc_w_clken_PORT->GetBus()) { UINT clken = 1; csr_eqosr2mc_w_clken_PORT->Write(&clken); } \
  if (csw_eqosw2mc_c_clken_PORT->GetBus()) { UINT clken = 1; csw_eqosw2mc_c_clken_PORT->Write(&clken); } \
  if (csw_eqosw2mc_cclk_wakeup_PORT->GetBus()) { UINT clken = 1; csw_eqosw2mc_cclk_wakeup_PORT->Write(&clken); } \
  if (csw_eqosw2mc_r_clken_PORT->GetBus()) { UINT clken = 1; csw_eqosw2mc_r_clken_PORT->Write(&clken); } \
  if (csw_eqosw2mc_w_clken_PORT->GetBus()) { UINT clken = 1; csw_eqosw2mc_w_clken_PORT->Write(&clken); }
#define NV_MC_ETR2MC_CLKEN_INIT \
  if (csr_etrr2mc_c_clken_PORT->GetBus()) { UINT clken = 1; csr_etrr2mc_c_clken_PORT->Write(&clken); } \
  if (csr_etrr2mc_cclk_wakeup_PORT->GetBus()) { UINT clken = 1; csr_etrr2mc_cclk_wakeup_PORT->Write(&clken); } \
  if (csr_etrr2mc_r_clken_PORT->GetBus()) { UINT clken = 1; csr_etrr2mc_r_clken_PORT->Write(&clken); } \
  if (csr_etrr2mc_w_clken_PORT->GetBus()) { UINT clken = 1; csr_etrr2mc_w_clken_PORT->Write(&clken); } \
  if (csw_etrw2mc_c_clken_PORT->GetBus()) { UINT clken = 1; csw_etrw2mc_c_clken_PORT->Write(&clken); } \
  if (csw_etrw2mc_cclk_wakeup_PORT->GetBus()) { UINT clken = 1; csw_etrw2mc_cclk_wakeup_PORT->Write(&clken); } \
  if (csw_etrw2mc_r_clken_PORT->GetBus()) { UINT clken = 1; csw_etrw2mc_r_clken_PORT->Write(&clken); } \
  if (csw_etrw2mc_w_clken_PORT->GetBus()) { UINT clken = 1; csw_etrw2mc_w_clken_PORT->Write(&clken); }
#define NV_MC_GPU2MC_CLKEN_INIT \
  if (csr_gpusrd2mc_c_clken_PORT->GetBus()) { UINT clken = 1; csr_gpusrd2mc_c_clken_PORT->Write(&clken); } \
  if (csr_gpusrd2mc_cclk_wakeup_PORT->GetBus()) { UINT clken = 1; csr_gpusrd2mc_cclk_wakeup_PORT->Write(&clken); } \
  if (csr_gpusrd2mc_r_clken_PORT->GetBus()) { UINT clken = 1; csr_gpusrd2mc_r_clken_PORT->Write(&clken); } \
  if (csr_gpusrd2mc_w_clken_PORT->GetBus()) { UINT clken = 1; csr_gpusrd2mc_w_clken_PORT->Write(&clken); } \
  if (csw_gpuswr2mc_c_clken_PORT->GetBus()) { UINT clken = 1; csw_gpuswr2mc_c_clken_PORT->Write(&clken); } \
  if (csw_gpuswr2mc_cclk_wakeup_PORT->GetBus()) { UINT clken = 1; csw_gpuswr2mc_cclk_wakeup_PORT->Write(&clken); } \
  if (csw_gpuswr2mc_r_clken_PORT->GetBus()) { UINT clken = 1; csw_gpuswr2mc_r_clken_PORT->Write(&clken); } \
  if (csw_gpuswr2mc_w_clken_PORT->GetBus()) { UINT clken = 1; csw_gpuswr2mc_w_clken_PORT->Write(&clken); }
#define NV_MC_GPU22MC_CLKEN_INIT \
  if (csr_gpusrd22mc_c_clken_PORT->GetBus()) { UINT clken = 1; csr_gpusrd22mc_c_clken_PORT->Write(&clken); } \
  if (csr_gpusrd22mc_cclk_wakeup_PORT->GetBus()) { UINT clken = 1; csr_gpusrd22mc_cclk_wakeup_PORT->Write(&clken); } \
  if (csr_gpusrd22mc_r_clken_PORT->GetBus()) { UINT clken = 1; csr_gpusrd22mc_r_clken_PORT->Write(&clken); } \
  if (csr_gpusrd22mc_w_clken_PORT->GetBus()) { UINT clken = 1; csr_gpusrd22mc_w_clken_PORT->Write(&clken); } \
  if (csw_gpuswr22mc_c_clken_PORT->GetBus()) { UINT clken = 1; csw_gpuswr22mc_c_clken_PORT->Write(&clken); } \
  if (csw_gpuswr22mc_cclk_wakeup_PORT->GetBus()) { UINT clken = 1; csw_gpuswr22mc_cclk_wakeup_PORT->Write(&clken); } \
  if (csw_gpuswr22mc_r_clken_PORT->GetBus()) { UINT clken = 1; csw_gpuswr22mc_r_clken_PORT->Write(&clken); } \
  if (csw_gpuswr22mc_w_clken_PORT->GetBus()) { UINT clken = 1; csw_gpuswr22mc_w_clken_PORT->Write(&clken); }
#define NV_MC_HDA2MC_CLKEN_INIT \
  if (csr_hdar2mc_c_clken_PORT->GetBus()) { UINT clken = 1; csr_hdar2mc_c_clken_PORT->Write(&clken); } \
  if (csr_hdar2mc_cclk_wakeup_PORT->GetBus()) { UINT clken = 1; csr_hdar2mc_cclk_wakeup_PORT->Write(&clken); } \
  if (csr_hdar2mc_r_clken_PORT->GetBus()) { UINT clken = 1; csr_hdar2mc_r_clken_PORT->Write(&clken); } \
  if (csr_hdar2mc_w_clken_PORT->GetBus()) { UINT clken = 1; csr_hdar2mc_w_clken_PORT->Write(&clken); } \
  if (csw_hdaw2mc_c_clken_PORT->GetBus()) { UINT clken = 1; csw_hdaw2mc_c_clken_PORT->Write(&clken); } \
  if (csw_hdaw2mc_cclk_wakeup_PORT->GetBus()) { UINT clken = 1; csw_hdaw2mc_cclk_wakeup_PORT->Write(&clken); } \
  if (csw_hdaw2mc_r_clken_PORT->GetBus()) { UINT clken = 1; csw_hdaw2mc_r_clken_PORT->Write(&clken); } \
  if (csw_hdaw2mc_w_clken_PORT->GetBus()) { UINT clken = 1; csw_hdaw2mc_w_clken_PORT->Write(&clken); }
#define NV_MC_HOST1X2MC_CLKEN_INIT \
  if (csr_host1xdmar2mc_c_clken_PORT->GetBus()) { UINT clken = 1; csr_host1xdmar2mc_c_clken_PORT->Write(&clken); } \
  if (csr_host1xdmar2mc_cclk_wakeup_PORT->GetBus()) { UINT clken = 1; csr_host1xdmar2mc_cclk_wakeup_PORT->Write(&clken); } \
  if (csr_host1xdmar2mc_r_clken_PORT->GetBus()) { UINT clken = 1; csr_host1xdmar2mc_r_clken_PORT->Write(&clken); } \
  if (csr_host1xdmar2mc_w_clken_PORT->GetBus()) { UINT clken = 1; csr_host1xdmar2mc_w_clken_PORT->Write(&clken); }
#define NV_MC_ISP22MC_CLKEN_INIT \
  if (csr_ispra2mc_c_clken_PORT->GetBus()) { UINT clken = 1; csr_ispra2mc_c_clken_PORT->Write(&clken); } \
  if (csr_ispra2mc_cclk_wakeup_PORT->GetBus()) { UINT clken = 1; csr_ispra2mc_cclk_wakeup_PORT->Write(&clken); } \
  if (csr_ispra2mc_r_clken_PORT->GetBus()) { UINT clken = 1; csr_ispra2mc_r_clken_PORT->Write(&clken); } \
  if (csr_ispra2mc_w_clken_PORT->GetBus()) { UINT clken = 1; csr_ispra2mc_w_clken_PORT->Write(&clken); } \
  if (csw_ispwa2mc_c_clken_PORT->GetBus()) { UINT clken = 1; csw_ispwa2mc_c_clken_PORT->Write(&clken); } \
  if (csw_ispwa2mc_cclk_wakeup_PORT->GetBus()) { UINT clken = 1; csw_ispwa2mc_cclk_wakeup_PORT->Write(&clken); } \
  if (csw_ispwa2mc_r_clken_PORT->GetBus()) { UINT clken = 1; csw_ispwa2mc_r_clken_PORT->Write(&clken); } \
  if (csw_ispwa2mc_w_clken_PORT->GetBus()) { UINT clken = 1; csw_ispwa2mc_w_clken_PORT->Write(&clken); } \
  if (csw_ispwb2mc_c_clken_PORT->GetBus()) { UINT clken = 1; csw_ispwb2mc_c_clken_PORT->Write(&clken); } \
  if (csw_ispwb2mc_cclk_wakeup_PORT->GetBus()) { UINT clken = 1; csw_ispwb2mc_cclk_wakeup_PORT->Write(&clken); } \
  if (csw_ispwb2mc_r_clken_PORT->GetBus()) { UINT clken = 1; csw_ispwb2mc_r_clken_PORT->Write(&clken); } \
  if (csw_ispwb2mc_w_clken_PORT->GetBus()) { UINT clken = 1; csw_ispwb2mc_w_clken_PORT->Write(&clken); }
#define NV_MC_MPCORE2MC_CLKEN_INIT \
  if (csw_mpcorew2mc_c_clken_PORT->GetBus()) { UINT clken = 1; csw_mpcorew2mc_c_clken_PORT->Write(&clken); } \
  if (csw_mpcorew2mc_cclk_wakeup_PORT->GetBus()) { UINT clken = 1; csw_mpcorew2mc_cclk_wakeup_PORT->Write(&clken); } \
  if (csw_mpcorew2mc_r_clken_PORT->GetBus()) { UINT clken = 1; csw_mpcorew2mc_r_clken_PORT->Write(&clken); } \
  if (csw_mpcorew2mc_w_clken_PORT->GetBus()) { UINT clken = 1; csw_mpcorew2mc_w_clken_PORT->Write(&clken); } \
  if (csr_mpcorer2mc_c_clken_PORT->GetBus()) { UINT clken = 1; csr_mpcorer2mc_c_clken_PORT->Write(&clken); } \
  if (csr_mpcorer2mc_cclk_wakeup_PORT->GetBus()) { UINT clken = 1; csr_mpcorer2mc_cclk_wakeup_PORT->Write(&clken); } \
  if (csr_mpcorer2mc_r_clken_PORT->GetBus()) { UINT clken = 1; csr_mpcorer2mc_r_clken_PORT->Write(&clken); } \
  if (csr_mpcorer2mc_w_clken_PORT->GetBus()) { UINT clken = 1; csr_mpcorer2mc_w_clken_PORT->Write(&clken); }
#define NV_MC_NVDEC2MC_CLKEN_INIT \
  if (csr_nvdecsrd2mc_c_clken_PORT->GetBus()) { UINT clken = 1; csr_nvdecsrd2mc_c_clken_PORT->Write(&clken); } \
  if (csr_nvdecsrd2mc_cclk_wakeup_PORT->GetBus()) { UINT clken = 1; csr_nvdecsrd2mc_cclk_wakeup_PORT->Write(&clken); } \
  if (csr_nvdecsrd2mc_r_clken_PORT->GetBus()) { UINT clken = 1; csr_nvdecsrd2mc_r_clken_PORT->Write(&clken); } \
  if (csr_nvdecsrd2mc_w_clken_PORT->GetBus()) { UINT clken = 1; csr_nvdecsrd2mc_w_clken_PORT->Write(&clken); }
#define NV_MC_NVDEC12MC_CLKEN_INIT \
  if (csr_nvdecsrd12mc_c_clken_PORT->GetBus()) { UINT clken = 1; csr_nvdecsrd12mc_c_clken_PORT->Write(&clken); } \
  if (csr_nvdecsrd12mc_cclk_wakeup_PORT->GetBus()) { UINT clken = 1; csr_nvdecsrd12mc_cclk_wakeup_PORT->Write(&clken); } \
  if (csr_nvdecsrd12mc_r_clken_PORT->GetBus()) { UINT clken = 1; csr_nvdecsrd12mc_r_clken_PORT->Write(&clken); } \
  if (csr_nvdecsrd12mc_w_clken_PORT->GetBus()) { UINT clken = 1; csr_nvdecsrd12mc_w_clken_PORT->Write(&clken); }
#define NV_MC_NVDEC32MC_CLKEN_INIT \
  if (csw_nvdecswr2mc_c_clken_PORT->GetBus()) { UINT clken = 1; csw_nvdecswr2mc_c_clken_PORT->Write(&clken); } \
  if (csw_nvdecswr2mc_cclk_wakeup_PORT->GetBus()) { UINT clken = 1; csw_nvdecswr2mc_cclk_wakeup_PORT->Write(&clken); } \
  if (csw_nvdecswr2mc_r_clken_PORT->GetBus()) { UINT clken = 1; csw_nvdecswr2mc_r_clken_PORT->Write(&clken); } \
  if (csw_nvdecswr2mc_w_clken_PORT->GetBus()) { UINT clken = 1; csw_nvdecswr2mc_w_clken_PORT->Write(&clken); }
#define NV_MC_NVDISPLAY2MC_CLKEN_INIT \
  if (csr_nvdisplayr2mc_c_clken_PORT->GetBus()) { UINT clken = 1; csr_nvdisplayr2mc_c_clken_PORT->Write(&clken); } \
  if (csr_nvdisplayr2mc_cclk_wakeup_PORT->GetBus()) { UINT clken = 1; csr_nvdisplayr2mc_cclk_wakeup_PORT->Write(&clken); } \
  if (csr_nvdisplayr2mc_r_clken_PORT->GetBus()) { UINT clken = 1; csr_nvdisplayr2mc_r_clken_PORT->Write(&clken); } \
  if (csr_nvdisplayr2mc_w_clken_PORT->GetBus()) { UINT clken = 1; csr_nvdisplayr2mc_w_clken_PORT->Write(&clken); }
#define NV_MC_NVDISPLAY12MC_CLKEN_INIT \
  if (csr_nvdisplayr12mc_c_clken_PORT->GetBus()) { UINT clken = 1; csr_nvdisplayr12mc_c_clken_PORT->Write(&clken); } \
  if (csr_nvdisplayr12mc_cclk_wakeup_PORT->GetBus()) { UINT clken = 1; csr_nvdisplayr12mc_cclk_wakeup_PORT->Write(&clken); } \
  if (csr_nvdisplayr12mc_r_clken_PORT->GetBus()) { UINT clken = 1; csr_nvdisplayr12mc_r_clken_PORT->Write(&clken); } \
  if (csr_nvdisplayr12mc_w_clken_PORT->GetBus()) { UINT clken = 1; csr_nvdisplayr12mc_w_clken_PORT->Write(&clken); }
#define NV_MC_NVENC2MC_CLKEN_INIT \
  if (csr_nvencsrd2mc_c_clken_PORT->GetBus()) { UINT clken = 1; csr_nvencsrd2mc_c_clken_PORT->Write(&clken); } \
  if (csr_nvencsrd2mc_cclk_wakeup_PORT->GetBus()) { UINT clken = 1; csr_nvencsrd2mc_cclk_wakeup_PORT->Write(&clken); } \
  if (csr_nvencsrd2mc_r_clken_PORT->GetBus()) { UINT clken = 1; csr_nvencsrd2mc_r_clken_PORT->Write(&clken); } \
  if (csr_nvencsrd2mc_w_clken_PORT->GetBus()) { UINT clken = 1; csr_nvencsrd2mc_w_clken_PORT->Write(&clken); }
#define NV_MC_NVENC22MC_CLKEN_INIT \
  if (csw_nvencswr2mc_c_clken_PORT->GetBus()) { UINT clken = 1; csw_nvencswr2mc_c_clken_PORT->Write(&clken); } \
  if (csw_nvencswr2mc_cclk_wakeup_PORT->GetBus()) { UINT clken = 1; csw_nvencswr2mc_cclk_wakeup_PORT->Write(&clken); } \
  if (csw_nvencswr2mc_r_clken_PORT->GetBus()) { UINT clken = 1; csw_nvencswr2mc_r_clken_PORT->Write(&clken); } \
  if (csw_nvencswr2mc_w_clken_PORT->GetBus()) { UINT clken = 1; csw_nvencswr2mc_w_clken_PORT->Write(&clken); }
#define NV_MC_NVJPG2MC_CLKEN_INIT \
  if (csr_nvjpgsrd2mc_c_clken_PORT->GetBus()) { UINT clken = 1; csr_nvjpgsrd2mc_c_clken_PORT->Write(&clken); } \
  if (csr_nvjpgsrd2mc_cclk_wakeup_PORT->GetBus()) { UINT clken = 1; csr_nvjpgsrd2mc_cclk_wakeup_PORT->Write(&clken); } \
  if (csr_nvjpgsrd2mc_r_clken_PORT->GetBus()) { UINT clken = 1; csr_nvjpgsrd2mc_r_clken_PORT->Write(&clken); } \
  if (csr_nvjpgsrd2mc_w_clken_PORT->GetBus()) { UINT clken = 1; csr_nvjpgsrd2mc_w_clken_PORT->Write(&clken); } \
  if (csw_nvjpgswr2mc_c_clken_PORT->GetBus()) { UINT clken = 1; csw_nvjpgswr2mc_c_clken_PORT->Write(&clken); } \
  if (csw_nvjpgswr2mc_cclk_wakeup_PORT->GetBus()) { UINT clken = 1; csw_nvjpgswr2mc_cclk_wakeup_PORT->Write(&clken); } \
  if (csw_nvjpgswr2mc_r_clken_PORT->GetBus()) { UINT clken = 1; csw_nvjpgswr2mc_r_clken_PORT->Write(&clken); } \
  if (csw_nvjpgswr2mc_w_clken_PORT->GetBus()) { UINT clken = 1; csw_nvjpgswr2mc_w_clken_PORT->Write(&clken); }
#define NV_MC_SATA2MC_CLKEN_INIT \
  if (csr_satar2mc_c_clken_PORT->GetBus()) { UINT clken = 1; csr_satar2mc_c_clken_PORT->Write(&clken); } \
  if (csr_satar2mc_cclk_wakeup_PORT->GetBus()) { UINT clken = 1; csr_satar2mc_cclk_wakeup_PORT->Write(&clken); } \
  if (csr_satar2mc_r_clken_PORT->GetBus()) { UINT clken = 1; csr_satar2mc_r_clken_PORT->Write(&clken); } \
  if (csr_satar2mc_w_clken_PORT->GetBus()) { UINT clken = 1; csr_satar2mc_w_clken_PORT->Write(&clken); } \
  if (csw_sataw2mc_c_clken_PORT->GetBus()) { UINT clken = 1; csw_sataw2mc_c_clken_PORT->Write(&clken); } \
  if (csw_sataw2mc_cclk_wakeup_PORT->GetBus()) { UINT clken = 1; csw_sataw2mc_cclk_wakeup_PORT->Write(&clken); } \
  if (csw_sataw2mc_r_clken_PORT->GetBus()) { UINT clken = 1; csw_sataw2mc_r_clken_PORT->Write(&clken); } \
  if (csw_sataw2mc_w_clken_PORT->GetBus()) { UINT clken = 1; csw_sataw2mc_w_clken_PORT->Write(&clken); }
#define NV_MC_SCE2MC_CLKEN_INIT \
  if (csr_scer2mc_c_clken_PORT->GetBus()) { UINT clken = 1; csr_scer2mc_c_clken_PORT->Write(&clken); } \
  if (csr_scer2mc_cclk_wakeup_PORT->GetBus()) { UINT clken = 1; csr_scer2mc_cclk_wakeup_PORT->Write(&clken); } \
  if (csr_scer2mc_r_clken_PORT->GetBus()) { UINT clken = 1; csr_scer2mc_r_clken_PORT->Write(&clken); } \
  if (csr_scer2mc_w_clken_PORT->GetBus()) { UINT clken = 1; csr_scer2mc_w_clken_PORT->Write(&clken); } \
  if (csw_scew2mc_c_clken_PORT->GetBus()) { UINT clken = 1; csw_scew2mc_c_clken_PORT->Write(&clken); } \
  if (csw_scew2mc_cclk_wakeup_PORT->GetBus()) { UINT clken = 1; csw_scew2mc_cclk_wakeup_PORT->Write(&clken); } \
  if (csw_scew2mc_r_clken_PORT->GetBus()) { UINT clken = 1; csw_scew2mc_r_clken_PORT->Write(&clken); } \
  if (csw_scew2mc_w_clken_PORT->GetBus()) { UINT clken = 1; csw_scew2mc_w_clken_PORT->Write(&clken); }
#define NV_MC_SCEDMA2MC_CLKEN_INIT \
  if (csr_scedmar2mc_c_clken_PORT->GetBus()) { UINT clken = 1; csr_scedmar2mc_c_clken_PORT->Write(&clken); } \
  if (csr_scedmar2mc_cclk_wakeup_PORT->GetBus()) { UINT clken = 1; csr_scedmar2mc_cclk_wakeup_PORT->Write(&clken); } \
  if (csr_scedmar2mc_r_clken_PORT->GetBus()) { UINT clken = 1; csr_scedmar2mc_r_clken_PORT->Write(&clken); } \
  if (csr_scedmar2mc_w_clken_PORT->GetBus()) { UINT clken = 1; csr_scedmar2mc_w_clken_PORT->Write(&clken); } \
  if (csw_scedmaw2mc_c_clken_PORT->GetBus()) { UINT clken = 1; csw_scedmaw2mc_c_clken_PORT->Write(&clken); } \
  if (csw_scedmaw2mc_cclk_wakeup_PORT->GetBus()) { UINT clken = 1; csw_scedmaw2mc_cclk_wakeup_PORT->Write(&clken); } \
  if (csw_scedmaw2mc_r_clken_PORT->GetBus()) { UINT clken = 1; csw_scedmaw2mc_r_clken_PORT->Write(&clken); } \
  if (csw_scedmaw2mc_w_clken_PORT->GetBus()) { UINT clken = 1; csw_scedmaw2mc_w_clken_PORT->Write(&clken); }
#define NV_MC_SDMMC2MC_CLKEN_INIT \
  if (csr_sdmmcr2mc_c_clken_PORT->GetBus()) { UINT clken = 1; csr_sdmmcr2mc_c_clken_PORT->Write(&clken); } \
  if (csr_sdmmcr2mc_cclk_wakeup_PORT->GetBus()) { UINT clken = 1; csr_sdmmcr2mc_cclk_wakeup_PORT->Write(&clken); } \
  if (csr_sdmmcr2mc_r_clken_PORT->GetBus()) { UINT clken = 1; csr_sdmmcr2mc_r_clken_PORT->Write(&clken); } \
  if (csr_sdmmcr2mc_w_clken_PORT->GetBus()) { UINT clken = 1; csr_sdmmcr2mc_w_clken_PORT->Write(&clken); } \
  if (csw_sdmmcw2mc_c_clken_PORT->GetBus()) { UINT clken = 1; csw_sdmmcw2mc_c_clken_PORT->Write(&clken); } \
  if (csw_sdmmcw2mc_cclk_wakeup_PORT->GetBus()) { UINT clken = 1; csw_sdmmcw2mc_cclk_wakeup_PORT->Write(&clken); } \
  if (csw_sdmmcw2mc_r_clken_PORT->GetBus()) { UINT clken = 1; csw_sdmmcw2mc_r_clken_PORT->Write(&clken); } \
  if (csw_sdmmcw2mc_w_clken_PORT->GetBus()) { UINT clken = 1; csw_sdmmcw2mc_w_clken_PORT->Write(&clken); }
#define NV_MC_SDMMCA2MC_CLKEN_INIT \
  if (csr_sdmmcra2mc_c_clken_PORT->GetBus()) { UINT clken = 1; csr_sdmmcra2mc_c_clken_PORT->Write(&clken); } \
  if (csr_sdmmcra2mc_cclk_wakeup_PORT->GetBus()) { UINT clken = 1; csr_sdmmcra2mc_cclk_wakeup_PORT->Write(&clken); } \
  if (csr_sdmmcra2mc_r_clken_PORT->GetBus()) { UINT clken = 1; csr_sdmmcra2mc_r_clken_PORT->Write(&clken); } \
  if (csr_sdmmcra2mc_w_clken_PORT->GetBus()) { UINT clken = 1; csr_sdmmcra2mc_w_clken_PORT->Write(&clken); } \
  if (csw_sdmmcwa2mc_c_clken_PORT->GetBus()) { UINT clken = 1; csw_sdmmcwa2mc_c_clken_PORT->Write(&clken); } \
  if (csw_sdmmcwa2mc_cclk_wakeup_PORT->GetBus()) { UINT clken = 1; csw_sdmmcwa2mc_cclk_wakeup_PORT->Write(&clken); } \
  if (csw_sdmmcwa2mc_r_clken_PORT->GetBus()) { UINT clken = 1; csw_sdmmcwa2mc_r_clken_PORT->Write(&clken); } \
  if (csw_sdmmcwa2mc_w_clken_PORT->GetBus()) { UINT clken = 1; csw_sdmmcwa2mc_w_clken_PORT->Write(&clken); }
#define NV_MC_SDMMCAA2MC_CLKEN_INIT \
  if (csr_sdmmcraa2mc_c_clken_PORT->GetBus()) { UINT clken = 1; csr_sdmmcraa2mc_c_clken_PORT->Write(&clken); } \
  if (csr_sdmmcraa2mc_cclk_wakeup_PORT->GetBus()) { UINT clken = 1; csr_sdmmcraa2mc_cclk_wakeup_PORT->Write(&clken); } \
  if (csr_sdmmcraa2mc_r_clken_PORT->GetBus()) { UINT clken = 1; csr_sdmmcraa2mc_r_clken_PORT->Write(&clken); } \
  if (csr_sdmmcraa2mc_w_clken_PORT->GetBus()) { UINT clken = 1; csr_sdmmcraa2mc_w_clken_PORT->Write(&clken); } \
  if (csw_sdmmcwaa2mc_c_clken_PORT->GetBus()) { UINT clken = 1; csw_sdmmcwaa2mc_c_clken_PORT->Write(&clken); } \
  if (csw_sdmmcwaa2mc_cclk_wakeup_PORT->GetBus()) { UINT clken = 1; csw_sdmmcwaa2mc_cclk_wakeup_PORT->Write(&clken); } \
  if (csw_sdmmcwaa2mc_r_clken_PORT->GetBus()) { UINT clken = 1; csw_sdmmcwaa2mc_r_clken_PORT->Write(&clken); } \
  if (csw_sdmmcwaa2mc_w_clken_PORT->GetBus()) { UINT clken = 1; csw_sdmmcwaa2mc_w_clken_PORT->Write(&clken); }
#define NV_MC_SDMMCAB2MC_CLKEN_INIT \
  if (csr_sdmmcrab2mc_c_clken_PORT->GetBus()) { UINT clken = 1; csr_sdmmcrab2mc_c_clken_PORT->Write(&clken); } \
  if (csr_sdmmcrab2mc_cclk_wakeup_PORT->GetBus()) { UINT clken = 1; csr_sdmmcrab2mc_cclk_wakeup_PORT->Write(&clken); } \
  if (csr_sdmmcrab2mc_r_clken_PORT->GetBus()) { UINT clken = 1; csr_sdmmcrab2mc_r_clken_PORT->Write(&clken); } \
  if (csr_sdmmcrab2mc_w_clken_PORT->GetBus()) { UINT clken = 1; csr_sdmmcrab2mc_w_clken_PORT->Write(&clken); } \
  if (csw_sdmmcwab2mc_c_clken_PORT->GetBus()) { UINT clken = 1; csw_sdmmcwab2mc_c_clken_PORT->Write(&clken); } \
  if (csw_sdmmcwab2mc_cclk_wakeup_PORT->GetBus()) { UINT clken = 1; csw_sdmmcwab2mc_cclk_wakeup_PORT->Write(&clken); } \
  if (csw_sdmmcwab2mc_r_clken_PORT->GetBus()) { UINT clken = 1; csw_sdmmcwab2mc_r_clken_PORT->Write(&clken); } \
  if (csw_sdmmcwab2mc_w_clken_PORT->GetBus()) { UINT clken = 1; csw_sdmmcwab2mc_w_clken_PORT->Write(&clken); }
#define NV_MC_SE2MC_CLKEN_INIT \
  if (csr_sesrd2mc_c_clken_PORT->GetBus()) { UINT clken = 1; csr_sesrd2mc_c_clken_PORT->Write(&clken); } \
  if (csr_sesrd2mc_cclk_wakeup_PORT->GetBus()) { UINT clken = 1; csr_sesrd2mc_cclk_wakeup_PORT->Write(&clken); } \
  if (csr_sesrd2mc_r_clken_PORT->GetBus()) { UINT clken = 1; csr_sesrd2mc_r_clken_PORT->Write(&clken); } \
  if (csr_sesrd2mc_w_clken_PORT->GetBus()) { UINT clken = 1; csr_sesrd2mc_w_clken_PORT->Write(&clken); } \
  if (csw_seswr2mc_c_clken_PORT->GetBus()) { UINT clken = 1; csw_seswr2mc_c_clken_PORT->Write(&clken); } \
  if (csw_seswr2mc_cclk_wakeup_PORT->GetBus()) { UINT clken = 1; csw_seswr2mc_cclk_wakeup_PORT->Write(&clken); } \
  if (csw_seswr2mc_r_clken_PORT->GetBus()) { UINT clken = 1; csw_seswr2mc_r_clken_PORT->Write(&clken); } \
  if (csw_seswr2mc_w_clken_PORT->GetBus()) { UINT clken = 1; csw_seswr2mc_w_clken_PORT->Write(&clken); }
#define NV_MC_TSEC2MC_CLKEN_INIT \
  if (csr_tsecsrd2mc_c_clken_PORT->GetBus()) { UINT clken = 1; csr_tsecsrd2mc_c_clken_PORT->Write(&clken); } \
  if (csr_tsecsrd2mc_cclk_wakeup_PORT->GetBus()) { UINT clken = 1; csr_tsecsrd2mc_cclk_wakeup_PORT->Write(&clken); } \
  if (csr_tsecsrd2mc_r_clken_PORT->GetBus()) { UINT clken = 1; csr_tsecsrd2mc_r_clken_PORT->Write(&clken); } \
  if (csr_tsecsrd2mc_w_clken_PORT->GetBus()) { UINT clken = 1; csr_tsecsrd2mc_w_clken_PORT->Write(&clken); } \
  if (csw_tsecswr2mc_c_clken_PORT->GetBus()) { UINT clken = 1; csw_tsecswr2mc_c_clken_PORT->Write(&clken); } \
  if (csw_tsecswr2mc_cclk_wakeup_PORT->GetBus()) { UINT clken = 1; csw_tsecswr2mc_cclk_wakeup_PORT->Write(&clken); } \
  if (csw_tsecswr2mc_r_clken_PORT->GetBus()) { UINT clken = 1; csw_tsecswr2mc_r_clken_PORT->Write(&clken); } \
  if (csw_tsecswr2mc_w_clken_PORT->GetBus()) { UINT clken = 1; csw_tsecswr2mc_w_clken_PORT->Write(&clken); }
#define NV_MC_TSECB2MC_CLKEN_INIT \
  if (csr_tsecsrdb2mc_c_clken_PORT->GetBus()) { UINT clken = 1; csr_tsecsrdb2mc_c_clken_PORT->Write(&clken); } \
  if (csr_tsecsrdb2mc_cclk_wakeup_PORT->GetBus()) { UINT clken = 1; csr_tsecsrdb2mc_cclk_wakeup_PORT->Write(&clken); } \
  if (csr_tsecsrdb2mc_r_clken_PORT->GetBus()) { UINT clken = 1; csr_tsecsrdb2mc_r_clken_PORT->Write(&clken); } \
  if (csr_tsecsrdb2mc_w_clken_PORT->GetBus()) { UINT clken = 1; csr_tsecsrdb2mc_w_clken_PORT->Write(&clken); } \
  if (csw_tsecswrb2mc_c_clken_PORT->GetBus()) { UINT clken = 1; csw_tsecswrb2mc_c_clken_PORT->Write(&clken); } \
  if (csw_tsecswrb2mc_cclk_wakeup_PORT->GetBus()) { UINT clken = 1; csw_tsecswrb2mc_cclk_wakeup_PORT->Write(&clken); } \
  if (csw_tsecswrb2mc_r_clken_PORT->GetBus()) { UINT clken = 1; csw_tsecswrb2mc_r_clken_PORT->Write(&clken); } \
  if (csw_tsecswrb2mc_w_clken_PORT->GetBus()) { UINT clken = 1; csw_tsecswrb2mc_w_clken_PORT->Write(&clken); }
#define NV_MC_UFSHC2MC_CLKEN_INIT \
  if (csr_ufshcr2mc_c_clken_PORT->GetBus()) { UINT clken = 1; csr_ufshcr2mc_c_clken_PORT->Write(&clken); } \
  if (csr_ufshcr2mc_cclk_wakeup_PORT->GetBus()) { UINT clken = 1; csr_ufshcr2mc_cclk_wakeup_PORT->Write(&clken); } \
  if (csr_ufshcr2mc_r_clken_PORT->GetBus()) { UINT clken = 1; csr_ufshcr2mc_r_clken_PORT->Write(&clken); } \
  if (csr_ufshcr2mc_w_clken_PORT->GetBus()) { UINT clken = 1; csr_ufshcr2mc_w_clken_PORT->Write(&clken); } \
  if (csw_ufshcw2mc_c_clken_PORT->GetBus()) { UINT clken = 1; csw_ufshcw2mc_c_clken_PORT->Write(&clken); } \
  if (csw_ufshcw2mc_cclk_wakeup_PORT->GetBus()) { UINT clken = 1; csw_ufshcw2mc_cclk_wakeup_PORT->Write(&clken); } \
  if (csw_ufshcw2mc_r_clken_PORT->GetBus()) { UINT clken = 1; csw_ufshcw2mc_r_clken_PORT->Write(&clken); } \
  if (csw_ufshcw2mc_w_clken_PORT->GetBus()) { UINT clken = 1; csw_ufshcw2mc_w_clken_PORT->Write(&clken); }
#define NV_MC_VI22MC_CLKEN_INIT \
  if (csw_viw2mc_c_clken_PORT->GetBus()) { UINT clken = 1; csw_viw2mc_c_clken_PORT->Write(&clken); } \
  if (csw_viw2mc_cclk_wakeup_PORT->GetBus()) { UINT clken = 1; csw_viw2mc_cclk_wakeup_PORT->Write(&clken); } \
  if (csw_viw2mc_r_clken_PORT->GetBus()) { UINT clken = 1; csw_viw2mc_r_clken_PORT->Write(&clken); } \
  if (csw_viw2mc_w_clken_PORT->GetBus()) { UINT clken = 1; csw_viw2mc_w_clken_PORT->Write(&clken); }
#define NV_MC_VIC2MC_CLKEN_INIT \
  if (csr_vicsrd2mc_c_clken_PORT->GetBus()) { UINT clken = 1; csr_vicsrd2mc_c_clken_PORT->Write(&clken); } \
  if (csr_vicsrd2mc_cclk_wakeup_PORT->GetBus()) { UINT clken = 1; csr_vicsrd2mc_cclk_wakeup_PORT->Write(&clken); } \
  if (csr_vicsrd2mc_r_clken_PORT->GetBus()) { UINT clken = 1; csr_vicsrd2mc_r_clken_PORT->Write(&clken); } \
  if (csr_vicsrd2mc_w_clken_PORT->GetBus()) { UINT clken = 1; csr_vicsrd2mc_w_clken_PORT->Write(&clken); }
#define NV_MC_VIC12MC_CLKEN_INIT \
  if (csr_vicsrd12mc_c_clken_PORT->GetBus()) { UINT clken = 1; csr_vicsrd12mc_c_clken_PORT->Write(&clken); } \
  if (csr_vicsrd12mc_cclk_wakeup_PORT->GetBus()) { UINT clken = 1; csr_vicsrd12mc_cclk_wakeup_PORT->Write(&clken); } \
  if (csr_vicsrd12mc_r_clken_PORT->GetBus()) { UINT clken = 1; csr_vicsrd12mc_r_clken_PORT->Write(&clken); } \
  if (csr_vicsrd12mc_w_clken_PORT->GetBus()) { UINT clken = 1; csr_vicsrd12mc_w_clken_PORT->Write(&clken); }
#define NV_MC_VIC32MC_CLKEN_INIT \
  if (csw_vicswr2mc_c_clken_PORT->GetBus()) { UINT clken = 1; csw_vicswr2mc_c_clken_PORT->Write(&clken); } \
  if (csw_vicswr2mc_cclk_wakeup_PORT->GetBus()) { UINT clken = 1; csw_vicswr2mc_cclk_wakeup_PORT->Write(&clken); } \
  if (csw_vicswr2mc_r_clken_PORT->GetBus()) { UINT clken = 1; csw_vicswr2mc_r_clken_PORT->Write(&clken); } \
  if (csw_vicswr2mc_w_clken_PORT->GetBus()) { UINT clken = 1; csw_vicswr2mc_w_clken_PORT->Write(&clken); }
#define NV_MC_XUSB_DEV2MC_CLKEN_INIT \
  if (csr_xusb_devr2mc_c_clken_PORT->GetBus()) { UINT clken = 1; csr_xusb_devr2mc_c_clken_PORT->Write(&clken); } \
  if (csr_xusb_devr2mc_cclk_wakeup_PORT->GetBus()) { UINT clken = 1; csr_xusb_devr2mc_cclk_wakeup_PORT->Write(&clken); } \
  if (csr_xusb_devr2mc_r_clken_PORT->GetBus()) { UINT clken = 1; csr_xusb_devr2mc_r_clken_PORT->Write(&clken); } \
  if (csr_xusb_devr2mc_w_clken_PORT->GetBus()) { UINT clken = 1; csr_xusb_devr2mc_w_clken_PORT->Write(&clken); } \
  if (csw_xusb_devw2mc_c_clken_PORT->GetBus()) { UINT clken = 1; csw_xusb_devw2mc_c_clken_PORT->Write(&clken); } \
  if (csw_xusb_devw2mc_cclk_wakeup_PORT->GetBus()) { UINT clken = 1; csw_xusb_devw2mc_cclk_wakeup_PORT->Write(&clken); } \
  if (csw_xusb_devw2mc_r_clken_PORT->GetBus()) { UINT clken = 1; csw_xusb_devw2mc_r_clken_PORT->Write(&clken); } \
  if (csw_xusb_devw2mc_w_clken_PORT->GetBus()) { UINT clken = 1; csw_xusb_devw2mc_w_clken_PORT->Write(&clken); }
#define NV_MC_XUSB_HOST2MC_CLKEN_INIT \
  if (csr_xusb_hostr2mc_c_clken_PORT->GetBus()) { UINT clken = 1; csr_xusb_hostr2mc_c_clken_PORT->Write(&clken); } \
  if (csr_xusb_hostr2mc_cclk_wakeup_PORT->GetBus()) { UINT clken = 1; csr_xusb_hostr2mc_cclk_wakeup_PORT->Write(&clken); } \
  if (csr_xusb_hostr2mc_r_clken_PORT->GetBus()) { UINT clken = 1; csr_xusb_hostr2mc_r_clken_PORT->Write(&clken); } \
  if (csr_xusb_hostr2mc_w_clken_PORT->GetBus()) { UINT clken = 1; csr_xusb_hostr2mc_w_clken_PORT->Write(&clken); } \
  if (csw_xusb_hostw2mc_c_clken_PORT->GetBus()) { UINT clken = 1; csw_xusb_hostw2mc_c_clken_PORT->Write(&clken); } \
  if (csw_xusb_hostw2mc_cclk_wakeup_PORT->GetBus()) { UINT clken = 1; csw_xusb_hostw2mc_cclk_wakeup_PORT->Write(&clken); } \
  if (csw_xusb_hostw2mc_r_clken_PORT->GetBus()) { UINT clken = 1; csw_xusb_hostw2mc_r_clken_PORT->Write(&clken); } \
  if (csw_xusb_hostw2mc_w_clken_PORT->GetBus()) { UINT clken = 1; csw_xusb_hostw2mc_w_clken_PORT->Write(&clken); }

#define LIST_MOD_MSC_BUSES(_op_) \
  _op_(afi,0,msc_afi2mc) \
  _op_(afi,0,csr_afir2mc_c_clken) \
  _op_(afi,0,csr_afir2mc_cclk_wakeup) \
  _op_(afi,0,csw_afiw2mc_c_clken) \
  _op_(afi,0,csw_afiw2mc_cclk_wakeup) \
  _op_(aon,0,msc_aon2mc) \
  _op_(aon,0,csr_aonr2mc_c_clken) \
  _op_(aon,0,csr_aonr2mc_cclk_wakeup) \
  _op_(aon,0,csw_aonw2mc_c_clken) \
  _op_(aon,0,csw_aonw2mc_cclk_wakeup) \
  _op_(aondma,0,msc_aondma2mc) \
  _op_(aondma,0,csr_aondmar2mc_c_clken) \
  _op_(aondma,0,csr_aondmar2mc_cclk_wakeup) \
  _op_(aondma,0,csw_aondmaw2mc_c_clken) \
  _op_(aondma,0,csw_aondmaw2mc_cclk_wakeup) \
  _op_(ape,0,msc_ape2mc) \
  _op_(ape,0,csr_aper2mc_c_clken) \
  _op_(ape,0,csr_aper2mc_cclk_wakeup) \
  _op_(ape,0,csw_apew2mc_c_clken) \
  _op_(ape,0,csw_apew2mc_cclk_wakeup) \
  _op_(apedma,0,msc_apedma2mc) \
  _op_(apedma,0,csr_apedmar2mc_c_clken) \
  _op_(apedma,0,csr_apedmar2mc_cclk_wakeup) \
  _op_(apedma,0,csw_apedmaw2mc_c_clken) \
  _op_(apedma,0,csw_apedmaw2mc_cclk_wakeup) \
  _op_(axis,0,msc_axis2mc) \
  _op_(axis,0,csr_axisr2mc_c_clken) \
  _op_(axis,0,csr_axisr2mc_cclk_wakeup) \
  _op_(axis,0,csw_axisw2mc_c_clken) \
  _op_(axis,0,csw_axisw2mc_cclk_wakeup) \
  _op_(bpmp,0,msc_bpmp2mc) \
  _op_(bpmp,0,csr_bpmpr2mc_c_clken) \
  _op_(bpmp,0,csr_bpmpr2mc_cclk_wakeup) \
  _op_(bpmp,0,csw_bpmpw2mc_c_clken) \
  _op_(bpmp,0,csw_bpmpw2mc_cclk_wakeup) \
  _op_(bpmpdma,0,msc_bpmpdma2mc) \
  _op_(bpmpdma,0,csr_bpmpdmar2mc_c_clken) \
  _op_(bpmpdma,0,csr_bpmpdmar2mc_cclk_wakeup) \
  _op_(bpmpdma,0,csw_bpmpdmaw2mc_c_clken) \
  _op_(bpmpdma,0,csw_bpmpdmaw2mc_cclk_wakeup) \
  _op_(eqos,0,msc_eqos2mc) \
  _op_(eqos,0,csr_eqosr2mc_c_clken) \
  _op_(eqos,0,csr_eqosr2mc_cclk_wakeup) \
  _op_(eqos,0,csw_eqosw2mc_c_clken) \
  _op_(eqos,0,csw_eqosw2mc_cclk_wakeup) \
  _op_(etr,0,msc_etr2mc) \
  _op_(etr,0,csr_etrr2mc_c_clken) \
  _op_(etr,0,csr_etrr2mc_cclk_wakeup) \
  _op_(etr,0,csw_etrw2mc_c_clken) \
  _op_(etr,0,csw_etrw2mc_cclk_wakeup) \
  _op_(gpu,0,msc_gpu2mc) \
  _op_(gpu,0,csr_gpusrd2mc_c_clken) \
  _op_(gpu,0,csr_gpusrd2mc_cclk_wakeup) \
  _op_(gpu,0,csw_gpuswr2mc_c_clken) \
  _op_(gpu,0,csw_gpuswr2mc_cclk_wakeup) \
  _op_(gpu,0,msc_gpu22mc) \
  _op_(gpu,0,csr_gpusrd22mc_c_clken) \
  _op_(gpu,0,csr_gpusrd22mc_cclk_wakeup) \
  _op_(gpu,0,csw_gpuswr22mc_c_clken) \
  _op_(gpu,0,csw_gpuswr22mc_cclk_wakeup) \
  _op_(hda,0,msc_hda2mc) \
  _op_(hda,0,csr_hdar2mc_c_clken) \
  _op_(hda,0,csr_hdar2mc_cclk_wakeup) \
  _op_(hda,0,csw_hdaw2mc_c_clken) \
  _op_(hda,0,csw_hdaw2mc_cclk_wakeup) \
  _op_(host1x,0,msc_host1x2mc) \
  _op_(host1x,0,csr_host1xdmar2mc_c_clken) \
  _op_(host1x,0,csr_host1xdmar2mc_cclk_wakeup) \
  _op_(isp2,0,msc_isp22mc) \
  _op_(isp2,0,csr_ispra2mc_c_clken) \
  _op_(isp2,0,csr_ispra2mc_cclk_wakeup) \
  _op_(isp2,0,csw_ispwa2mc_c_clken) \
  _op_(isp2,0,csw_ispwa2mc_cclk_wakeup) \
  _op_(isp2,0,csw_ispwb2mc_c_clken) \
  _op_(isp2,0,csw_ispwb2mc_cclk_wakeup) \
  _op_(axicif,0,msc_mpcore2mc) \
  _op_(axicif,0,csw_mpcorew2mc_c_clken) \
  _op_(axicif,0,csw_mpcorew2mc_cclk_wakeup) \
  _op_(nvdec,0,msc_nvdec2mc) \
  _op_(nvdec,0,csr_nvdecsrd2mc_c_clken) \
  _op_(nvdec,0,csr_nvdecsrd2mc_cclk_wakeup) \
  _op_(nvdec,0,msc_nvdec12mc) \
  _op_(nvdec,0,csr_nvdecsrd12mc_c_clken) \
  _op_(nvdec,0,csr_nvdecsrd12mc_cclk_wakeup) \
  _op_(nvdec,0,msc_nvdec32mc) \
  _op_(nvdec,0,csw_nvdecswr2mc_c_clken) \
  _op_(nvdec,0,csw_nvdecswr2mc_cclk_wakeup) \
  _op_(nvdisplay,0,msc_nvdisplay2mc) \
  _op_(nvdisplay,0,csr_nvdisplayr2mc_c_clken) \
  _op_(nvdisplay,0,csr_nvdisplayr2mc_cclk_wakeup) \
  _op_(nvdisplay,0,msc_nvdisplay12mc) \
  _op_(nvdisplay,0,csr_nvdisplayr12mc_c_clken) \
  _op_(nvdisplay,0,csr_nvdisplayr12mc_cclk_wakeup) \
  _op_(nvenc,0,msc_nvenc2mc) \
  _op_(nvenc,0,csr_nvencsrd2mc_c_clken) \
  _op_(nvenc,0,csr_nvencsrd2mc_cclk_wakeup) \
  _op_(nvenc,0,msc_nvenc22mc) \
  _op_(nvenc,0,csw_nvencswr2mc_c_clken) \
  _op_(nvenc,0,csw_nvencswr2mc_cclk_wakeup) \
  _op_(nvjpg,0,msc_nvjpg2mc) \
  _op_(nvjpg,0,csr_nvjpgsrd2mc_c_clken) \
  _op_(nvjpg,0,csr_nvjpgsrd2mc_cclk_wakeup) \
  _op_(nvjpg,0,csw_nvjpgswr2mc_c_clken) \
  _op_(nvjpg,0,csw_nvjpgswr2mc_cclk_wakeup) \
  _op_(sata,0,msc_sata2mc) \
  _op_(sata,0,csr_satar2mc_c_clken) \
  _op_(sata,0,csr_satar2mc_cclk_wakeup) \
  _op_(sata,0,csw_sataw2mc_c_clken) \
  _op_(sata,0,csw_sataw2mc_cclk_wakeup) \
  _op_(sce,0,msc_sce2mc) \
  _op_(sce,0,csr_scer2mc_c_clken) \
  _op_(sce,0,csr_scer2mc_cclk_wakeup) \
  _op_(sce,0,csw_scew2mc_c_clken) \
  _op_(sce,0,csw_scew2mc_cclk_wakeup) \
  _op_(scedma,0,msc_scedma2mc) \
  _op_(scedma,0,csr_scedmar2mc_c_clken) \
  _op_(scedma,0,csr_scedmar2mc_cclk_wakeup) \
  _op_(scedma,0,csw_scedmaw2mc_c_clken) \
  _op_(scedma,0,csw_scedmaw2mc_cclk_wakeup) \
  _op_(sdmmc3,0,msc_sdmmc2mc) \
  _op_(sdmmc3,0,csr_sdmmcr2mc_c_clken) \
  _op_(sdmmc3,0,csr_sdmmcr2mc_cclk_wakeup) \
  _op_(sdmmc3,0,csw_sdmmcw2mc_c_clken) \
  _op_(sdmmc3,0,csw_sdmmcw2mc_cclk_wakeup) \
  _op_(sdmmc1,0,msc_sdmmca2mc) \
  _op_(sdmmc1,0,csr_sdmmcra2mc_c_clken) \
  _op_(sdmmc1,0,csr_sdmmcra2mc_cclk_wakeup) \
  _op_(sdmmc1,0,csw_sdmmcwa2mc_c_clken) \
  _op_(sdmmc1,0,csw_sdmmcwa2mc_cclk_wakeup) \
  _op_(sdmmc2,0,msc_sdmmcaa2mc) \
  _op_(sdmmc2,0,csr_sdmmcraa2mc_c_clken) \
  _op_(sdmmc2,0,csr_sdmmcraa2mc_cclk_wakeup) \
  _op_(sdmmc2,0,csw_sdmmcwaa2mc_c_clken) \
  _op_(sdmmc2,0,csw_sdmmcwaa2mc_cclk_wakeup) \
  _op_(sdmmc4,0,msc_sdmmcab2mc) \
  _op_(sdmmc4,0,csr_sdmmcrab2mc_c_clken) \
  _op_(sdmmc4,0,csr_sdmmcrab2mc_cclk_wakeup) \
  _op_(sdmmc4,0,csw_sdmmcwab2mc_c_clken) \
  _op_(sdmmc4,0,csw_sdmmcwab2mc_cclk_wakeup) \
  _op_(se,0,msc_se2mc) \
  _op_(se,0,csr_sesrd2mc_c_clken) \
  _op_(se,0,csr_sesrd2mc_cclk_wakeup) \
  _op_(se,0,csw_seswr2mc_c_clken) \
  _op_(se,0,csw_seswr2mc_cclk_wakeup) \
  _op_(tsec,0,msc_tsec2mc) \
  _op_(tsec,0,csr_tsecsrd2mc_c_clken) \
  _op_(tsec,0,csr_tsecsrd2mc_cclk_wakeup) \
  _op_(tsec,0,csw_tsecswr2mc_c_clken) \
  _op_(tsec,0,csw_tsecswr2mc_cclk_wakeup) \
  _op_(tsecb,0,msc_tsecb2mc) \
  _op_(tsecb,0,csr_tsecsrdb2mc_c_clken) \
  _op_(tsecb,0,csr_tsecsrdb2mc_cclk_wakeup) \
  _op_(tsecb,0,csw_tsecswrb2mc_c_clken) \
  _op_(tsecb,0,csw_tsecswrb2mc_cclk_wakeup) \
  _op_(ufshc,0,msc_ufshc2mc) \
  _op_(ufshc,0,csr_ufshcr2mc_c_clken) \
  _op_(ufshc,0,csr_ufshcr2mc_cclk_wakeup) \
  _op_(ufshc,0,csw_ufshcw2mc_c_clken) \
  _op_(ufshc,0,csw_ufshcw2mc_cclk_wakeup) \
  _op_(vi,0,msc_vi22mc) \
  _op_(vi,0,csw_viw2mc_c_clken) \
  _op_(vi,0,csw_viw2mc_cclk_wakeup) \
  _op_(vic,0,msc_vic2mc) \
  _op_(vic,0,csr_vicsrd2mc_c_clken) \
  _op_(vic,0,csr_vicsrd2mc_cclk_wakeup) \
  _op_(vic,0,msc_vic12mc) \
  _op_(vic,0,csr_vicsrd12mc_c_clken) \
  _op_(vic,0,csr_vicsrd12mc_cclk_wakeup) \
  _op_(vic,0,msc_vic32mc) \
  _op_(vic,0,csw_vicswr2mc_c_clken) \
  _op_(vic,0,csw_vicswr2mc_cclk_wakeup) \
  _op_(xusb_dev,0,msc_xusb_dev2mc) \
  _op_(xusb_dev,0,csr_xusb_devr2mc_c_clken) \
  _op_(xusb_dev,0,csr_xusb_devr2mc_cclk_wakeup) \
  _op_(xusb_dev,0,csw_xusb_devw2mc_c_clken) \
  _op_(xusb_dev,0,csw_xusb_devw2mc_cclk_wakeup) \
  _op_(xusb_host,0,msc_xusb_host2mc) \
  _op_(xusb_host,0,csr_xusb_hostr2mc_c_clken) \
  _op_(xusb_host,0,csr_xusb_hostr2mc_cclk_wakeup) \
  _op_(xusb_host,0,csw_xusb_hostw2mc_c_clken) \
  _op_(xusb_host,0,csw_xusb_hostw2mc_cclk_wakeup) \
  _op_(axicif,0,msc_mll_mpcorer2mc) \
  _op_(axicif,0,msc_req_mll_mpcorer2mc0) \
  _op_(axicif,0,msc_req_mll_mpcorer2mc1) \
  _op_(axicif,0,msc_ret_mll_mpcorer2mc0) \
  _op_(axicif,0,msc_ret_mll_mpcorer2mc1) \
  _op_(axicif,0,csr_mpcorer2mc_c_clken) \
  _op_(axicif,0,csr_mpcorer2mc_cclk_wakeup)

#define NV_PTCR2MC_BRHW  NV_MC_DISABLED
#define NV_PTCR2MC_BRVW  NV_MC_DISABLED
#define NV_PTCR2MC_BRSW  NV_MC_DISABLED
#define NV_PTCR2MC_BRYUV  NV_MC_DISABLED
#define NV_PTCR2MC_BRYUVAVG  NV_MC_DISABLED
#define NV_PTCR2MC_BR_ALLOWSR  NV_MC_DISABLED
#define NV_PTCR2MC_BR_OFFYUV  NV_MC_DISABLED
#define NV_PTCR2MC_BWHW  NV_MC_DISABLED
#define NV_PTCR2MC_BWVW  NV_MC_DISABLED
#define NV_PTCR2MC_BWSW  NV_MC_DISABLED
#define NV_PTCR2MC_BWXY  NV_MC_DISABLED
#define NV_PTCR2MC_BWPK  NV_MC_DISABLED
#define NV_PTCR2MC_BW_ALLOWSW  NV_MC_DISABLED
#define NV_PTCR2MC_CWHW  NV_MC_DISABLED
#define NV_PTCR2MC_CWSW  NV_MC_DISABLED
#define NV_PTCR2MC_SR_HP  NV_MC_DISABLED
#define NV_PTCR2MC_SW_HP  NV_MC_DISABLED
#define NV_PTCR2MC_BR_HP  NV_MC_DISABLED
#define NV_PTCR2MC_BW_HP  NV_MC_DISABLED
#define NV_PTCR2MC_CW_HP  NV_MC_DISABLED
#define NV_AFIR2MC_BRHW  NV_MC_DISABLED
#define NV_AFIR2MC_BRVW  NV_MC_DISABLED
#define NV_AFIR2MC_BRSW  NV_MC_DISABLED
#define NV_AFIR2MC_BRYUV  NV_MC_DISABLED
#define NV_AFIR2MC_BRYUVAVG  NV_MC_DISABLED
#define NV_AFIR2MC_BR_ALLOWSR  NV_MC_DISABLED
#define NV_AFIR2MC_BR_OFFYUV  NV_MC_DISABLED
#define NV_AFIR2MC_BWHW  NV_MC_DISABLED
#define NV_AFIR2MC_BWVW  NV_MC_DISABLED
#define NV_AFIR2MC_BWSW  NV_MC_DISABLED
#define NV_AFIR2MC_BWXY  NV_MC_DISABLED
#define NV_AFIR2MC_BWPK  NV_MC_DISABLED
#define NV_AFIR2MC_BW_ALLOWSW  NV_MC_DISABLED
#define NV_AFIR2MC_CWHW  NV_MC_DISABLED
#define NV_AFIR2MC_CWSW  NV_MC_DISABLED
#define NV_AFIR2MC_SR_HP  NV_MC_DISABLED
#define NV_AFIR2MC_SW_HP  NV_MC_DISABLED
#define NV_AFIR2MC_BR_HP  NV_MC_DISABLED
#define NV_AFIR2MC_BW_HP  NV_MC_DISABLED
#define NV_AFIR2MC_CW_HP  NV_MC_DISABLED
#define NV_HDAR2MC_BRHW  NV_MC_DISABLED
#define NV_HDAR2MC_BRVW  NV_MC_DISABLED
#define NV_HDAR2MC_BRSW  NV_MC_DISABLED
#define NV_HDAR2MC_BRYUV  NV_MC_DISABLED
#define NV_HDAR2MC_BRYUVAVG  NV_MC_DISABLED
#define NV_HDAR2MC_BR_ALLOWSR  NV_MC_DISABLED
#define NV_HDAR2MC_BR_OFFYUV  NV_MC_DISABLED
#define NV_HDAR2MC_BWHW  NV_MC_DISABLED
#define NV_HDAR2MC_BWVW  NV_MC_DISABLED
#define NV_HDAR2MC_BWSW  NV_MC_DISABLED
#define NV_HDAR2MC_BWXY  NV_MC_DISABLED
#define NV_HDAR2MC_BWPK  NV_MC_DISABLED
#define NV_HDAR2MC_BW_ALLOWSW  NV_MC_DISABLED
#define NV_HDAR2MC_CWHW  NV_MC_DISABLED
#define NV_HDAR2MC_CWSW  NV_MC_DISABLED
#define NV_HDAR2MC_SR_HP  NV_MC_DISABLED
#define NV_HDAR2MC_SW_HP  NV_MC_DISABLED
#define NV_HDAR2MC_BR_HP  NV_MC_DISABLED
#define NV_HDAR2MC_BW_HP  NV_MC_DISABLED
#define NV_HDAR2MC_CW_HP  NV_MC_DISABLED
#define NV_HOST1XDMAR2MC_BRHW  NV_MC_DISABLED
#define NV_HOST1XDMAR2MC_BRVW  NV_MC_DISABLED
#define NV_HOST1XDMAR2MC_BRSW  NV_MC_DISABLED
#define NV_HOST1XDMAR2MC_BRYUV  NV_MC_DISABLED
#define NV_HOST1XDMAR2MC_BRYUVAVG  NV_MC_DISABLED
#define NV_HOST1XDMAR2MC_BR_ALLOWSR  NV_MC_DISABLED
#define NV_HOST1XDMAR2MC_BR_OFFYUV  NV_MC_DISABLED
#define NV_HOST1XDMAR2MC_BWHW  NV_MC_DISABLED
#define NV_HOST1XDMAR2MC_BWVW  NV_MC_DISABLED
#define NV_HOST1XDMAR2MC_BWSW  NV_MC_DISABLED
#define NV_HOST1XDMAR2MC_BWXY  NV_MC_DISABLED
#define NV_HOST1XDMAR2MC_BWPK  NV_MC_DISABLED
#define NV_HOST1XDMAR2MC_BW_ALLOWSW  NV_MC_DISABLED
#define NV_HOST1XDMAR2MC_CWHW  NV_MC_DISABLED
#define NV_HOST1XDMAR2MC_CWSW  NV_MC_DISABLED
#define NV_HOST1XDMAR2MC_SR_HP  NV_MC_DISABLED
#define NV_HOST1XDMAR2MC_SW_HP  NV_MC_DISABLED
#define NV_HOST1XDMAR2MC_BR_HP  NV_MC_DISABLED
#define NV_HOST1XDMAR2MC_BW_HP  NV_MC_DISABLED
#define NV_HOST1XDMAR2MC_CW_HP  NV_MC_DISABLED
#define NV_NVENCSRD2MC_BRHW  NV_MC_DISABLED
#define NV_NVENCSRD2MC_BRVW  NV_MC_DISABLED
#define NV_NVENCSRD2MC_BRSW  NV_MC_DISABLED
#define NV_NVENCSRD2MC_BRYUV  NV_MC_DISABLED
#define NV_NVENCSRD2MC_BRYUVAVG  NV_MC_DISABLED
#define NV_NVENCSRD2MC_BR_ALLOWSR  NV_MC_DISABLED
#define NV_NVENCSRD2MC_BR_OFFYUV  NV_MC_DISABLED
#define NV_NVENCSRD2MC_BWHW  NV_MC_DISABLED
#define NV_NVENCSRD2MC_BWVW  NV_MC_DISABLED
#define NV_NVENCSRD2MC_BWSW  NV_MC_DISABLED
#define NV_NVENCSRD2MC_BWXY  NV_MC_DISABLED
#define NV_NVENCSRD2MC_BWPK  NV_MC_DISABLED
#define NV_NVENCSRD2MC_BW_ALLOWSW  NV_MC_DISABLED
#define NV_NVENCSRD2MC_CWHW  NV_MC_DISABLED
#define NV_NVENCSRD2MC_CWSW  NV_MC_DISABLED
#define NV_NVENCSRD2MC_SR_HP  NV_MC_DISABLED
#define NV_NVENCSRD2MC_SW_HP  NV_MC_DISABLED
#define NV_NVENCSRD2MC_BR_HP  NV_MC_DISABLED
#define NV_NVENCSRD2MC_BW_HP  NV_MC_DISABLED
#define NV_NVENCSRD2MC_CW_HP  NV_MC_DISABLED
#define NV_SATAR2MC_BRHW  NV_MC_DISABLED
#define NV_SATAR2MC_BRVW  NV_MC_DISABLED
#define NV_SATAR2MC_BRSW  NV_MC_DISABLED
#define NV_SATAR2MC_BRYUV  NV_MC_DISABLED
#define NV_SATAR2MC_BRYUVAVG  NV_MC_DISABLED
#define NV_SATAR2MC_BR_ALLOWSR  NV_MC_DISABLED
#define NV_SATAR2MC_BR_OFFYUV  NV_MC_DISABLED
#define NV_SATAR2MC_BWHW  NV_MC_DISABLED
#define NV_SATAR2MC_BWVW  NV_MC_DISABLED
#define NV_SATAR2MC_BWSW  NV_MC_DISABLED
#define NV_SATAR2MC_BWXY  NV_MC_DISABLED
#define NV_SATAR2MC_BWPK  NV_MC_DISABLED
#define NV_SATAR2MC_BW_ALLOWSW  NV_MC_DISABLED
#define NV_SATAR2MC_CWHW  NV_MC_DISABLED
#define NV_SATAR2MC_CWSW  NV_MC_DISABLED
#define NV_SATAR2MC_SR_HP  NV_MC_DISABLED
#define NV_SATAR2MC_SW_HP  NV_MC_DISABLED
#define NV_SATAR2MC_BR_HP  NV_MC_DISABLED
#define NV_SATAR2MC_BW_HP  NV_MC_DISABLED
#define NV_SATAR2MC_CW_HP  NV_MC_DISABLED
#define NV_MPCORER2MC_BRHW  NV_MC_DISABLED
#define NV_MPCORER2MC_BRVW  NV_MC_DISABLED
#define NV_MPCORER2MC_BRSW  NV_MC_DISABLED
#define NV_MPCORER2MC_BRYUV  NV_MC_DISABLED
#define NV_MPCORER2MC_BRYUVAVG  NV_MC_DISABLED
#define NV_MPCORER2MC_BR_ALLOWSR  NV_MC_DISABLED
#define NV_MPCORER2MC_BR_OFFYUV  NV_MC_DISABLED
#define NV_MPCORER2MC_BWHW  NV_MC_DISABLED
#define NV_MPCORER2MC_BWVW  NV_MC_DISABLED
#define NV_MPCORER2MC_BWSW  NV_MC_DISABLED
#define NV_MPCORER2MC_BWXY  NV_MC_DISABLED
#define NV_MPCORER2MC_BWPK  NV_MC_DISABLED
#define NV_MPCORER2MC_BW_ALLOWSW  NV_MC_DISABLED
#define NV_MPCORER2MC_CWHW  NV_MC_DISABLED
#define NV_MPCORER2MC_CWSW  NV_MC_DISABLED
#define NV_MPCORER2MC_SR_HP  NV_MC_DISABLED
#define NV_MPCORER2MC_SW_HP  NV_MC_DISABLED
#define NV_MPCORER2MC_BR_HP  NV_MC_DISABLED
#define NV_MPCORER2MC_BW_HP  NV_MC_DISABLED
#define NV_MPCORER2MC_CW_HP  NV_MC_DISABLED
#define NV_NVENCSWR2MC_BRHW  NV_MC_DISABLED
#define NV_NVENCSWR2MC_BRVW  NV_MC_DISABLED
#define NV_NVENCSWR2MC_BRSW  NV_MC_DISABLED
#define NV_NVENCSWR2MC_BRYUV  NV_MC_DISABLED
#define NV_NVENCSWR2MC_BRYUVAVG  NV_MC_DISABLED
#define NV_NVENCSWR2MC_BR_ALLOWSR  NV_MC_DISABLED
#define NV_NVENCSWR2MC_BR_OFFYUV  NV_MC_DISABLED
#define NV_NVENCSWR2MC_BWHW  NV_MC_DISABLED
#define NV_NVENCSWR2MC_BWVW  NV_MC_DISABLED
#define NV_NVENCSWR2MC_BWSW  NV_MC_DISABLED
#define NV_NVENCSWR2MC_BWXY  NV_MC_DISABLED
#define NV_NVENCSWR2MC_BWPK  NV_MC_DISABLED
#define NV_NVENCSWR2MC_BW_ALLOWSW  NV_MC_DISABLED
#define NV_NVENCSWR2MC_CWHW  NV_MC_DISABLED
#define NV_NVENCSWR2MC_CWSW  NV_MC_DISABLED
#define NV_NVENCSWR2MC_SR_HP  NV_MC_DISABLED
#define NV_NVENCSWR2MC_SW_HP  NV_MC_DISABLED
#define NV_NVENCSWR2MC_BR_HP  NV_MC_DISABLED
#define NV_NVENCSWR2MC_BW_HP  NV_MC_DISABLED
#define NV_NVENCSWR2MC_CW_HP  NV_MC_DISABLED
#define NV_AFIW2MC_BRHW  NV_MC_DISABLED
#define NV_AFIW2MC_BRVW  NV_MC_DISABLED
#define NV_AFIW2MC_BRSW  NV_MC_DISABLED
#define NV_AFIW2MC_BRYUV  NV_MC_DISABLED
#define NV_AFIW2MC_BRYUVAVG  NV_MC_DISABLED
#define NV_AFIW2MC_BR_ALLOWSR  NV_MC_DISABLED
#define NV_AFIW2MC_BR_OFFYUV  NV_MC_DISABLED
#define NV_AFIW2MC_BWHW  NV_MC_DISABLED
#define NV_AFIW2MC_BWVW  NV_MC_DISABLED
#define NV_AFIW2MC_BWSW  NV_MC_DISABLED
#define NV_AFIW2MC_BWXY  NV_MC_DISABLED
#define NV_AFIW2MC_BWPK  NV_MC_DISABLED
#define NV_AFIW2MC_BW_ALLOWSW  NV_MC_DISABLED
#define NV_AFIW2MC_CWHW  NV_MC_DISABLED
#define NV_AFIW2MC_CWSW  NV_MC_DISABLED
#define NV_AFIW2MC_SR_HP  NV_MC_DISABLED
#define NV_AFIW2MC_SW_HP  NV_MC_DISABLED
#define NV_AFIW2MC_BR_HP  NV_MC_DISABLED
#define NV_AFIW2MC_BW_HP  NV_MC_DISABLED
#define NV_AFIW2MC_CW_HP  NV_MC_DISABLED
#define NV_HDAW2MC_BRHW  NV_MC_DISABLED
#define NV_HDAW2MC_BRVW  NV_MC_DISABLED
#define NV_HDAW2MC_BRSW  NV_MC_DISABLED
#define NV_HDAW2MC_BRYUV  NV_MC_DISABLED
#define NV_HDAW2MC_BRYUVAVG  NV_MC_DISABLED
#define NV_HDAW2MC_BR_ALLOWSR  NV_MC_DISABLED
#define NV_HDAW2MC_BR_OFFYUV  NV_MC_DISABLED
#define NV_HDAW2MC_BWHW  NV_MC_DISABLED
#define NV_HDAW2MC_BWVW  NV_MC_DISABLED
#define NV_HDAW2MC_BWSW  NV_MC_DISABLED
#define NV_HDAW2MC_BWXY  NV_MC_DISABLED
#define NV_HDAW2MC_BWPK  NV_MC_DISABLED
#define NV_HDAW2MC_BW_ALLOWSW  NV_MC_DISABLED
#define NV_HDAW2MC_CWHW  NV_MC_DISABLED
#define NV_HDAW2MC_CWSW  NV_MC_DISABLED
#define NV_HDAW2MC_SR_HP  NV_MC_DISABLED
#define NV_HDAW2MC_SW_HP  NV_MC_DISABLED
#define NV_HDAW2MC_BR_HP  NV_MC_DISABLED
#define NV_HDAW2MC_BW_HP  NV_MC_DISABLED
#define NV_HDAW2MC_CW_HP  NV_MC_DISABLED
#define NV_MPCOREW2MC_BRHW  NV_MC_DISABLED
#define NV_MPCOREW2MC_BRVW  NV_MC_DISABLED
#define NV_MPCOREW2MC_BRSW  NV_MC_DISABLED
#define NV_MPCOREW2MC_BRYUV  NV_MC_DISABLED
#define NV_MPCOREW2MC_BRYUVAVG  NV_MC_DISABLED
#define NV_MPCOREW2MC_BR_ALLOWSR  NV_MC_DISABLED
#define NV_MPCOREW2MC_BR_OFFYUV  NV_MC_DISABLED
#define NV_MPCOREW2MC_BWHW  NV_MC_DISABLED
#define NV_MPCOREW2MC_BWVW  NV_MC_DISABLED
#define NV_MPCOREW2MC_BWSW  NV_MC_DISABLED
#define NV_MPCOREW2MC_BWXY  NV_MC_DISABLED
#define NV_MPCOREW2MC_BWPK  NV_MC_DISABLED
#define NV_MPCOREW2MC_BW_ALLOWSW  NV_MC_DISABLED
#define NV_MPCOREW2MC_CWHW  NV_MC_DISABLED
#define NV_MPCOREW2MC_CWSW  NV_MC_DISABLED
#define NV_MPCOREW2MC_SR_HP  NV_MC_DISABLED
#define NV_MPCOREW2MC_SW_HP  NV_MC_DISABLED
#define NV_MPCOREW2MC_BR_HP  NV_MC_DISABLED
#define NV_MPCOREW2MC_BW_HP  NV_MC_DISABLED
#define NV_MPCOREW2MC_CW_HP  NV_MC_DISABLED
#define NV_SATAW2MC_BRHW  NV_MC_DISABLED
#define NV_SATAW2MC_BRVW  NV_MC_DISABLED
#define NV_SATAW2MC_BRSW  NV_MC_DISABLED
#define NV_SATAW2MC_BRYUV  NV_MC_DISABLED
#define NV_SATAW2MC_BRYUVAVG  NV_MC_DISABLED
#define NV_SATAW2MC_BR_ALLOWSR  NV_MC_DISABLED
#define NV_SATAW2MC_BR_OFFYUV  NV_MC_DISABLED
#define NV_SATAW2MC_BWHW  NV_MC_DISABLED
#define NV_SATAW2MC_BWVW  NV_MC_DISABLED
#define NV_SATAW2MC_BWSW  NV_MC_DISABLED
#define NV_SATAW2MC_BWXY  NV_MC_DISABLED
#define NV_SATAW2MC_BWPK  NV_MC_DISABLED
#define NV_SATAW2MC_BW_ALLOWSW  NV_MC_DISABLED
#define NV_SATAW2MC_CWHW  NV_MC_DISABLED
#define NV_SATAW2MC_CWSW  NV_MC_DISABLED
#define NV_SATAW2MC_SR_HP  NV_MC_DISABLED
#define NV_SATAW2MC_SW_HP  NV_MC_DISABLED
#define NV_SATAW2MC_BR_HP  NV_MC_DISABLED
#define NV_SATAW2MC_BW_HP  NV_MC_DISABLED
#define NV_SATAW2MC_CW_HP  NV_MC_DISABLED
#define NV_ISPRA2MC_BRHW  NV_MC_DISABLED
#define NV_ISPRA2MC_BRVW  NV_MC_DISABLED
#define NV_ISPRA2MC_BRSW  NV_MC_DISABLED
#define NV_ISPRA2MC_BRYUV  NV_MC_DISABLED
#define NV_ISPRA2MC_BRYUVAVG  NV_MC_DISABLED
#define NV_ISPRA2MC_BR_ALLOWSR  NV_MC_DISABLED
#define NV_ISPRA2MC_BR_OFFYUV  NV_MC_DISABLED
#define NV_ISPRA2MC_BWHW  NV_MC_DISABLED
#define NV_ISPRA2MC_BWVW  NV_MC_DISABLED
#define NV_ISPRA2MC_BWSW  NV_MC_DISABLED
#define NV_ISPRA2MC_BWXY  NV_MC_DISABLED
#define NV_ISPRA2MC_BWPK  NV_MC_DISABLED
#define NV_ISPRA2MC_BW_ALLOWSW  NV_MC_DISABLED
#define NV_ISPRA2MC_CWHW  NV_MC_DISABLED
#define NV_ISPRA2MC_CWSW  NV_MC_DISABLED
#define NV_ISPRA2MC_SR_HP  NV_MC_DISABLED
#define NV_ISPRA2MC_SW_HP  NV_MC_DISABLED
#define NV_ISPRA2MC_BR_HP  NV_MC_DISABLED
#define NV_ISPRA2MC_BW_HP  NV_MC_DISABLED
#define NV_ISPRA2MC_CW_HP  NV_MC_DISABLED
#define NV_ISPWA2MC_BRHW  NV_MC_DISABLED
#define NV_ISPWA2MC_BRVW  NV_MC_DISABLED
#define NV_ISPWA2MC_BRSW  NV_MC_DISABLED
#define NV_ISPWA2MC_BRYUV  NV_MC_DISABLED
#define NV_ISPWA2MC_BRYUVAVG  NV_MC_DISABLED
#define NV_ISPWA2MC_BR_ALLOWSR  NV_MC_DISABLED
#define NV_ISPWA2MC_BR_OFFYUV  NV_MC_DISABLED
#define NV_ISPWA2MC_BWHW  NV_MC_DISABLED
#define NV_ISPWA2MC_BWVW  NV_MC_DISABLED
#define NV_ISPWA2MC_BWSW  NV_MC_DISABLED
#define NV_ISPWA2MC_BWXY  NV_MC_DISABLED
#define NV_ISPWA2MC_BWPK  NV_MC_DISABLED
#define NV_ISPWA2MC_BW_ALLOWSW  NV_MC_DISABLED
#define NV_ISPWA2MC_CWHW  NV_MC_DISABLED
#define NV_ISPWA2MC_CWSW  NV_MC_DISABLED
#define NV_ISPWA2MC_SR_HP  NV_MC_DISABLED
#define NV_ISPWA2MC_SW_HP  NV_MC_DISABLED
#define NV_ISPWA2MC_BR_HP  NV_MC_DISABLED
#define NV_ISPWA2MC_BW_HP  NV_MC_DISABLED
#define NV_ISPWA2MC_CW_HP  NV_MC_DISABLED
#define NV_ISPWB2MC_BRHW  NV_MC_DISABLED
#define NV_ISPWB2MC_BRVW  NV_MC_DISABLED
#define NV_ISPWB2MC_BRSW  NV_MC_DISABLED
#define NV_ISPWB2MC_BRYUV  NV_MC_DISABLED
#define NV_ISPWB2MC_BRYUVAVG  NV_MC_DISABLED
#define NV_ISPWB2MC_BR_ALLOWSR  NV_MC_DISABLED
#define NV_ISPWB2MC_BR_OFFYUV  NV_MC_DISABLED
#define NV_ISPWB2MC_BWHW  NV_MC_DISABLED
#define NV_ISPWB2MC_BWVW  NV_MC_DISABLED
#define NV_ISPWB2MC_BWSW  NV_MC_DISABLED
#define NV_ISPWB2MC_BWXY  NV_MC_DISABLED
#define NV_ISPWB2MC_BWPK  NV_MC_DISABLED
#define NV_ISPWB2MC_BW_ALLOWSW  NV_MC_DISABLED
#define NV_ISPWB2MC_CWHW  NV_MC_DISABLED
#define NV_ISPWB2MC_CWSW  NV_MC_DISABLED
#define NV_ISPWB2MC_SR_HP  NV_MC_DISABLED
#define NV_ISPWB2MC_SW_HP  NV_MC_DISABLED
#define NV_ISPWB2MC_BR_HP  NV_MC_DISABLED
#define NV_ISPWB2MC_BW_HP  NV_MC_DISABLED
#define NV_ISPWB2MC_CW_HP  NV_MC_DISABLED
#define NV_XUSB_HOSTR2MC_BRHW  NV_MC_DISABLED
#define NV_XUSB_HOSTR2MC_BRVW  NV_MC_DISABLED
#define NV_XUSB_HOSTR2MC_BRSW  NV_MC_DISABLED
#define NV_XUSB_HOSTR2MC_BRYUV  NV_MC_DISABLED
#define NV_XUSB_HOSTR2MC_BRYUVAVG  NV_MC_DISABLED
#define NV_XUSB_HOSTR2MC_BR_ALLOWSR  NV_MC_DISABLED
#define NV_XUSB_HOSTR2MC_BR_OFFYUV  NV_MC_DISABLED
#define NV_XUSB_HOSTR2MC_BWHW  NV_MC_DISABLED
#define NV_XUSB_HOSTR2MC_BWVW  NV_MC_DISABLED
#define NV_XUSB_HOSTR2MC_BWSW  NV_MC_DISABLED
#define NV_XUSB_HOSTR2MC_BWXY  NV_MC_DISABLED
#define NV_XUSB_HOSTR2MC_BWPK  NV_MC_DISABLED
#define NV_XUSB_HOSTR2MC_BW_ALLOWSW  NV_MC_DISABLED
#define NV_XUSB_HOSTR2MC_CWHW  NV_MC_DISABLED
#define NV_XUSB_HOSTR2MC_CWSW  NV_MC_DISABLED
#define NV_XUSB_HOSTR2MC_SR_HP  NV_MC_DISABLED
#define NV_XUSB_HOSTR2MC_SW_HP  NV_MC_DISABLED
#define NV_XUSB_HOSTR2MC_BR_HP  NV_MC_DISABLED
#define NV_XUSB_HOSTR2MC_BW_HP  NV_MC_DISABLED
#define NV_XUSB_HOSTR2MC_CW_HP  NV_MC_DISABLED
#define NV_XUSB_HOSTW2MC_BRHW  NV_MC_DISABLED
#define NV_XUSB_HOSTW2MC_BRVW  NV_MC_DISABLED
#define NV_XUSB_HOSTW2MC_BRSW  NV_MC_DISABLED
#define NV_XUSB_HOSTW2MC_BRYUV  NV_MC_DISABLED
#define NV_XUSB_HOSTW2MC_BRYUVAVG  NV_MC_DISABLED
#define NV_XUSB_HOSTW2MC_BR_ALLOWSR  NV_MC_DISABLED
#define NV_XUSB_HOSTW2MC_BR_OFFYUV  NV_MC_DISABLED
#define NV_XUSB_HOSTW2MC_BWHW  NV_MC_DISABLED
#define NV_XUSB_HOSTW2MC_BWVW  NV_MC_DISABLED
#define NV_XUSB_HOSTW2MC_BWSW  NV_MC_DISABLED
#define NV_XUSB_HOSTW2MC_BWXY  NV_MC_DISABLED
#define NV_XUSB_HOSTW2MC_BWPK  NV_MC_DISABLED
#define NV_XUSB_HOSTW2MC_BW_ALLOWSW  NV_MC_DISABLED
#define NV_XUSB_HOSTW2MC_CWHW  NV_MC_DISABLED
#define NV_XUSB_HOSTW2MC_CWSW  NV_MC_DISABLED
#define NV_XUSB_HOSTW2MC_SR_HP  NV_MC_DISABLED
#define NV_XUSB_HOSTW2MC_SW_HP  NV_MC_DISABLED
#define NV_XUSB_HOSTW2MC_BR_HP  NV_MC_DISABLED
#define NV_XUSB_HOSTW2MC_BW_HP  NV_MC_DISABLED
#define NV_XUSB_HOSTW2MC_CW_HP  NV_MC_DISABLED
#define NV_XUSB_DEVR2MC_BRHW  NV_MC_DISABLED
#define NV_XUSB_DEVR2MC_BRVW  NV_MC_DISABLED
#define NV_XUSB_DEVR2MC_BRSW  NV_MC_DISABLED
#define NV_XUSB_DEVR2MC_BRYUV  NV_MC_DISABLED
#define NV_XUSB_DEVR2MC_BRYUVAVG  NV_MC_DISABLED
#define NV_XUSB_DEVR2MC_BR_ALLOWSR  NV_MC_DISABLED
#define NV_XUSB_DEVR2MC_BR_OFFYUV  NV_MC_DISABLED
#define NV_XUSB_DEVR2MC_BWHW  NV_MC_DISABLED
#define NV_XUSB_DEVR2MC_BWVW  NV_MC_DISABLED
#define NV_XUSB_DEVR2MC_BWSW  NV_MC_DISABLED
#define NV_XUSB_DEVR2MC_BWXY  NV_MC_DISABLED
#define NV_XUSB_DEVR2MC_BWPK  NV_MC_DISABLED
#define NV_XUSB_DEVR2MC_BW_ALLOWSW  NV_MC_DISABLED
#define NV_XUSB_DEVR2MC_CWHW  NV_MC_DISABLED
#define NV_XUSB_DEVR2MC_CWSW  NV_MC_DISABLED
#define NV_XUSB_DEVR2MC_SR_HP  NV_MC_DISABLED
#define NV_XUSB_DEVR2MC_SW_HP  NV_MC_DISABLED
#define NV_XUSB_DEVR2MC_BR_HP  NV_MC_DISABLED
#define NV_XUSB_DEVR2MC_BW_HP  NV_MC_DISABLED
#define NV_XUSB_DEVR2MC_CW_HP  NV_MC_DISABLED
#define NV_XUSB_DEVW2MC_BRHW  NV_MC_DISABLED
#define NV_XUSB_DEVW2MC_BRVW  NV_MC_DISABLED
#define NV_XUSB_DEVW2MC_BRSW  NV_MC_DISABLED
#define NV_XUSB_DEVW2MC_BRYUV  NV_MC_DISABLED
#define NV_XUSB_DEVW2MC_BRYUVAVG  NV_MC_DISABLED
#define NV_XUSB_DEVW2MC_BR_ALLOWSR  NV_MC_DISABLED
#define NV_XUSB_DEVW2MC_BR_OFFYUV  NV_MC_DISABLED
#define NV_XUSB_DEVW2MC_BWHW  NV_MC_DISABLED
#define NV_XUSB_DEVW2MC_BWVW  NV_MC_DISABLED
#define NV_XUSB_DEVW2MC_BWSW  NV_MC_DISABLED
#define NV_XUSB_DEVW2MC_BWXY  NV_MC_DISABLED
#define NV_XUSB_DEVW2MC_BWPK  NV_MC_DISABLED
#define NV_XUSB_DEVW2MC_BW_ALLOWSW  NV_MC_DISABLED
#define NV_XUSB_DEVW2MC_CWHW  NV_MC_DISABLED
#define NV_XUSB_DEVW2MC_CWSW  NV_MC_DISABLED
#define NV_XUSB_DEVW2MC_SR_HP  NV_MC_DISABLED
#define NV_XUSB_DEVW2MC_SW_HP  NV_MC_DISABLED
#define NV_XUSB_DEVW2MC_BR_HP  NV_MC_DISABLED
#define NV_XUSB_DEVW2MC_BW_HP  NV_MC_DISABLED
#define NV_XUSB_DEVW2MC_CW_HP  NV_MC_DISABLED
#define NV_TSECSRD2MC_BRHW  NV_MC_DISABLED
#define NV_TSECSRD2MC_BRVW  NV_MC_DISABLED
#define NV_TSECSRD2MC_BRSW  NV_MC_DISABLED
#define NV_TSECSRD2MC_BRYUV  NV_MC_DISABLED
#define NV_TSECSRD2MC_BRYUVAVG  NV_MC_DISABLED
#define NV_TSECSRD2MC_BR_ALLOWSR  NV_MC_DISABLED
#define NV_TSECSRD2MC_BR_OFFYUV  NV_MC_DISABLED
#define NV_TSECSRD2MC_BWHW  NV_MC_DISABLED
#define NV_TSECSRD2MC_BWVW  NV_MC_DISABLED
#define NV_TSECSRD2MC_BWSW  NV_MC_DISABLED
#define NV_TSECSRD2MC_BWXY  NV_MC_DISABLED
#define NV_TSECSRD2MC_BWPK  NV_MC_DISABLED
#define NV_TSECSRD2MC_BW_ALLOWSW  NV_MC_DISABLED
#define NV_TSECSRD2MC_CWHW  NV_MC_DISABLED
#define NV_TSECSRD2MC_CWSW  NV_MC_DISABLED
#define NV_TSECSRD2MC_SR_HP  NV_MC_DISABLED
#define NV_TSECSRD2MC_SW_HP  NV_MC_DISABLED
#define NV_TSECSRD2MC_BR_HP  NV_MC_DISABLED
#define NV_TSECSRD2MC_BW_HP  NV_MC_DISABLED
#define NV_TSECSRD2MC_CW_HP  NV_MC_DISABLED
#define NV_TSECSWR2MC_BRHW  NV_MC_DISABLED
#define NV_TSECSWR2MC_BRVW  NV_MC_DISABLED
#define NV_TSECSWR2MC_BRSW  NV_MC_DISABLED
#define NV_TSECSWR2MC_BRYUV  NV_MC_DISABLED
#define NV_TSECSWR2MC_BRYUVAVG  NV_MC_DISABLED
#define NV_TSECSWR2MC_BR_ALLOWSR  NV_MC_DISABLED
#define NV_TSECSWR2MC_BR_OFFYUV  NV_MC_DISABLED
#define NV_TSECSWR2MC_BWHW  NV_MC_DISABLED
#define NV_TSECSWR2MC_BWVW  NV_MC_DISABLED
#define NV_TSECSWR2MC_BWSW  NV_MC_DISABLED
#define NV_TSECSWR2MC_BWXY  NV_MC_DISABLED
#define NV_TSECSWR2MC_BWPK  NV_MC_DISABLED
#define NV_TSECSWR2MC_BW_ALLOWSW  NV_MC_DISABLED
#define NV_TSECSWR2MC_CWHW  NV_MC_DISABLED
#define NV_TSECSWR2MC_CWSW  NV_MC_DISABLED
#define NV_TSECSWR2MC_SR_HP  NV_MC_DISABLED
#define NV_TSECSWR2MC_SW_HP  NV_MC_DISABLED
#define NV_TSECSWR2MC_BR_HP  NV_MC_DISABLED
#define NV_TSECSWR2MC_BW_HP  NV_MC_DISABLED
#define NV_TSECSWR2MC_CW_HP  NV_MC_DISABLED
#define NV_GPUSRD2MC_BRHW  NV_MC_DISABLED
#define NV_GPUSRD2MC_BRVW  NV_MC_DISABLED
#define NV_GPUSRD2MC_BRSW  NV_MC_DISABLED
#define NV_GPUSRD2MC_BRYUV  NV_MC_DISABLED
#define NV_GPUSRD2MC_BRYUVAVG  NV_MC_DISABLED
#define NV_GPUSRD2MC_BR_ALLOWSR  NV_MC_DISABLED
#define NV_GPUSRD2MC_BR_OFFYUV  NV_MC_DISABLED
#define NV_GPUSRD2MC_BWHW  NV_MC_DISABLED
#define NV_GPUSRD2MC_BWVW  NV_MC_DISABLED
#define NV_GPUSRD2MC_BWSW  NV_MC_DISABLED
#define NV_GPUSRD2MC_BWXY  NV_MC_DISABLED
#define NV_GPUSRD2MC_BWPK  NV_MC_DISABLED
#define NV_GPUSRD2MC_BW_ALLOWSW  NV_MC_DISABLED
#define NV_GPUSRD2MC_CWHW  NV_MC_DISABLED
#define NV_GPUSRD2MC_CWSW  NV_MC_DISABLED
#define NV_GPUSRD2MC_SR_HP  NV_MC_DISABLED
#define NV_GPUSRD2MC_SW_HP  NV_MC_DISABLED
#define NV_GPUSRD2MC_BR_HP  NV_MC_DISABLED
#define NV_GPUSRD2MC_BW_HP  NV_MC_DISABLED
#define NV_GPUSRD2MC_CW_HP  NV_MC_DISABLED
#define NV_GPUSWR2MC_BRHW  NV_MC_DISABLED
#define NV_GPUSWR2MC_BRVW  NV_MC_DISABLED
#define NV_GPUSWR2MC_BRSW  NV_MC_DISABLED
#define NV_GPUSWR2MC_BRYUV  NV_MC_DISABLED
#define NV_GPUSWR2MC_BRYUVAVG  NV_MC_DISABLED
#define NV_GPUSWR2MC_BR_ALLOWSR  NV_MC_DISABLED
#define NV_GPUSWR2MC_BR_OFFYUV  NV_MC_DISABLED
#define NV_GPUSWR2MC_BWHW  NV_MC_DISABLED
#define NV_GPUSWR2MC_BWVW  NV_MC_DISABLED
#define NV_GPUSWR2MC_BWSW  NV_MC_DISABLED
#define NV_GPUSWR2MC_BWXY  NV_MC_DISABLED
#define NV_GPUSWR2MC_BWPK  NV_MC_DISABLED
#define NV_GPUSWR2MC_BW_ALLOWSW  NV_MC_DISABLED
#define NV_GPUSWR2MC_CWHW  NV_MC_DISABLED
#define NV_GPUSWR2MC_CWSW  NV_MC_DISABLED
#define NV_GPUSWR2MC_SR_HP  NV_MC_DISABLED
#define NV_GPUSWR2MC_SW_HP  NV_MC_DISABLED
#define NV_GPUSWR2MC_BR_HP  NV_MC_DISABLED
#define NV_GPUSWR2MC_BW_HP  NV_MC_DISABLED
#define NV_GPUSWR2MC_CW_HP  NV_MC_DISABLED
#define NV_SDMMCRA2MC_BRHW  NV_MC_DISABLED
#define NV_SDMMCRA2MC_BRVW  NV_MC_DISABLED
#define NV_SDMMCRA2MC_BRSW  NV_MC_DISABLED
#define NV_SDMMCRA2MC_BRYUV  NV_MC_DISABLED
#define NV_SDMMCRA2MC_BRYUVAVG  NV_MC_DISABLED
#define NV_SDMMCRA2MC_BR_ALLOWSR  NV_MC_DISABLED
#define NV_SDMMCRA2MC_BR_OFFYUV  NV_MC_DISABLED
#define NV_SDMMCRA2MC_BWHW  NV_MC_DISABLED
#define NV_SDMMCRA2MC_BWVW  NV_MC_DISABLED
#define NV_SDMMCRA2MC_BWSW  NV_MC_DISABLED
#define NV_SDMMCRA2MC_BWXY  NV_MC_DISABLED
#define NV_SDMMCRA2MC_BWPK  NV_MC_DISABLED
#define NV_SDMMCRA2MC_BW_ALLOWSW  NV_MC_DISABLED
#define NV_SDMMCRA2MC_CWHW  NV_MC_DISABLED
#define NV_SDMMCRA2MC_CWSW  NV_MC_DISABLED
#define NV_SDMMCRA2MC_SR_HP  NV_MC_DISABLED
#define NV_SDMMCRA2MC_SW_HP  NV_MC_DISABLED
#define NV_SDMMCRA2MC_BR_HP  NV_MC_DISABLED
#define NV_SDMMCRA2MC_BW_HP  NV_MC_DISABLED
#define NV_SDMMCRA2MC_CW_HP  NV_MC_DISABLED
#define NV_SDMMCRAA2MC_BRHW  NV_MC_DISABLED
#define NV_SDMMCRAA2MC_BRVW  NV_MC_DISABLED
#define NV_SDMMCRAA2MC_BRSW  NV_MC_DISABLED
#define NV_SDMMCRAA2MC_BRYUV  NV_MC_DISABLED
#define NV_SDMMCRAA2MC_BRYUVAVG  NV_MC_DISABLED
#define NV_SDMMCRAA2MC_BR_ALLOWSR  NV_MC_DISABLED
#define NV_SDMMCRAA2MC_BR_OFFYUV  NV_MC_DISABLED
#define NV_SDMMCRAA2MC_BWHW  NV_MC_DISABLED
#define NV_SDMMCRAA2MC_BWVW  NV_MC_DISABLED
#define NV_SDMMCRAA2MC_BWSW  NV_MC_DISABLED
#define NV_SDMMCRAA2MC_BWXY  NV_MC_DISABLED
#define NV_SDMMCRAA2MC_BWPK  NV_MC_DISABLED
#define NV_SDMMCRAA2MC_BW_ALLOWSW  NV_MC_DISABLED
#define NV_SDMMCRAA2MC_CWHW  NV_MC_DISABLED
#define NV_SDMMCRAA2MC_CWSW  NV_MC_DISABLED
#define NV_SDMMCRAA2MC_SR_HP  NV_MC_DISABLED
#define NV_SDMMCRAA2MC_SW_HP  NV_MC_DISABLED
#define NV_SDMMCRAA2MC_BR_HP  NV_MC_DISABLED
#define NV_SDMMCRAA2MC_BW_HP  NV_MC_DISABLED
#define NV_SDMMCRAA2MC_CW_HP  NV_MC_DISABLED
#define NV_SDMMCR2MC_BRHW  NV_MC_DISABLED
#define NV_SDMMCR2MC_BRVW  NV_MC_DISABLED
#define NV_SDMMCR2MC_BRSW  NV_MC_DISABLED
#define NV_SDMMCR2MC_BRYUV  NV_MC_DISABLED
#define NV_SDMMCR2MC_BRYUVAVG  NV_MC_DISABLED
#define NV_SDMMCR2MC_BR_ALLOWSR  NV_MC_DISABLED
#define NV_SDMMCR2MC_BR_OFFYUV  NV_MC_DISABLED
#define NV_SDMMCR2MC_BWHW  NV_MC_DISABLED
#define NV_SDMMCR2MC_BWVW  NV_MC_DISABLED
#define NV_SDMMCR2MC_BWSW  NV_MC_DISABLED
#define NV_SDMMCR2MC_BWXY  NV_MC_DISABLED
#define NV_SDMMCR2MC_BWPK  NV_MC_DISABLED
#define NV_SDMMCR2MC_BW_ALLOWSW  NV_MC_DISABLED
#define NV_SDMMCR2MC_CWHW  NV_MC_DISABLED
#define NV_SDMMCR2MC_CWSW  NV_MC_DISABLED
#define NV_SDMMCR2MC_SR_HP  NV_MC_DISABLED
#define NV_SDMMCR2MC_SW_HP  NV_MC_DISABLED
#define NV_SDMMCR2MC_BR_HP  NV_MC_DISABLED
#define NV_SDMMCR2MC_BW_HP  NV_MC_DISABLED
#define NV_SDMMCR2MC_CW_HP  NV_MC_DISABLED
#define NV_SDMMCRAB2MC_BRHW  NV_MC_DISABLED
#define NV_SDMMCRAB2MC_BRVW  NV_MC_DISABLED
#define NV_SDMMCRAB2MC_BRSW  NV_MC_DISABLED
#define NV_SDMMCRAB2MC_BRYUV  NV_MC_DISABLED
#define NV_SDMMCRAB2MC_BRYUVAVG  NV_MC_DISABLED
#define NV_SDMMCRAB2MC_BR_ALLOWSR  NV_MC_DISABLED
#define NV_SDMMCRAB2MC_BR_OFFYUV  NV_MC_DISABLED
#define NV_SDMMCRAB2MC_BWHW  NV_MC_DISABLED
#define NV_SDMMCRAB2MC_BWVW  NV_MC_DISABLED
#define NV_SDMMCRAB2MC_BWSW  NV_MC_DISABLED
#define NV_SDMMCRAB2MC_BWXY  NV_MC_DISABLED
#define NV_SDMMCRAB2MC_BWPK  NV_MC_DISABLED
#define NV_SDMMCRAB2MC_BW_ALLOWSW  NV_MC_DISABLED
#define NV_SDMMCRAB2MC_CWHW  NV_MC_DISABLED
#define NV_SDMMCRAB2MC_CWSW  NV_MC_DISABLED
#define NV_SDMMCRAB2MC_SR_HP  NV_MC_DISABLED
#define NV_SDMMCRAB2MC_SW_HP  NV_MC_DISABLED
#define NV_SDMMCRAB2MC_BR_HP  NV_MC_DISABLED
#define NV_SDMMCRAB2MC_BW_HP  NV_MC_DISABLED
#define NV_SDMMCRAB2MC_CW_HP  NV_MC_DISABLED
#define NV_SDMMCWA2MC_BRHW  NV_MC_DISABLED
#define NV_SDMMCWA2MC_BRVW  NV_MC_DISABLED
#define NV_SDMMCWA2MC_BRSW  NV_MC_DISABLED
#define NV_SDMMCWA2MC_BRYUV  NV_MC_DISABLED
#define NV_SDMMCWA2MC_BRYUVAVG  NV_MC_DISABLED
#define NV_SDMMCWA2MC_BR_ALLOWSR  NV_MC_DISABLED
#define NV_SDMMCWA2MC_BR_OFFYUV  NV_MC_DISABLED
#define NV_SDMMCWA2MC_BWHW  NV_MC_DISABLED
#define NV_SDMMCWA2MC_BWVW  NV_MC_DISABLED
#define NV_SDMMCWA2MC_BWSW  NV_MC_DISABLED
#define NV_SDMMCWA2MC_BWXY  NV_MC_DISABLED
#define NV_SDMMCWA2MC_BWPK  NV_MC_DISABLED
#define NV_SDMMCWA2MC_BW_ALLOWSW  NV_MC_DISABLED
#define NV_SDMMCWA2MC_CWHW  NV_MC_DISABLED
#define NV_SDMMCWA2MC_CWSW  NV_MC_DISABLED
#define NV_SDMMCWA2MC_SR_HP  NV_MC_DISABLED
#define NV_SDMMCWA2MC_SW_HP  NV_MC_DISABLED
#define NV_SDMMCWA2MC_BR_HP  NV_MC_DISABLED
#define NV_SDMMCWA2MC_BW_HP  NV_MC_DISABLED
#define NV_SDMMCWA2MC_CW_HP  NV_MC_DISABLED
#define NV_SDMMCWAA2MC_BRHW  NV_MC_DISABLED
#define NV_SDMMCWAA2MC_BRVW  NV_MC_DISABLED
#define NV_SDMMCWAA2MC_BRSW  NV_MC_DISABLED
#define NV_SDMMCWAA2MC_BRYUV  NV_MC_DISABLED
#define NV_SDMMCWAA2MC_BRYUVAVG  NV_MC_DISABLED
#define NV_SDMMCWAA2MC_BR_ALLOWSR  NV_MC_DISABLED
#define NV_SDMMCWAA2MC_BR_OFFYUV  NV_MC_DISABLED
#define NV_SDMMCWAA2MC_BWHW  NV_MC_DISABLED
#define NV_SDMMCWAA2MC_BWVW  NV_MC_DISABLED
#define NV_SDMMCWAA2MC_BWSW  NV_MC_DISABLED
#define NV_SDMMCWAA2MC_BWXY  NV_MC_DISABLED
#define NV_SDMMCWAA2MC_BWPK  NV_MC_DISABLED
#define NV_SDMMCWAA2MC_BW_ALLOWSW  NV_MC_DISABLED
#define NV_SDMMCWAA2MC_CWHW  NV_MC_DISABLED
#define NV_SDMMCWAA2MC_CWSW  NV_MC_DISABLED
#define NV_SDMMCWAA2MC_SR_HP  NV_MC_DISABLED
#define NV_SDMMCWAA2MC_SW_HP  NV_MC_DISABLED
#define NV_SDMMCWAA2MC_BR_HP  NV_MC_DISABLED
#define NV_SDMMCWAA2MC_BW_HP  NV_MC_DISABLED
#define NV_SDMMCWAA2MC_CW_HP  NV_MC_DISABLED
#define NV_SDMMCW2MC_BRHW  NV_MC_DISABLED
#define NV_SDMMCW2MC_BRVW  NV_MC_DISABLED
#define NV_SDMMCW2MC_BRSW  NV_MC_DISABLED
#define NV_SDMMCW2MC_BRYUV  NV_MC_DISABLED
#define NV_SDMMCW2MC_BRYUVAVG  NV_MC_DISABLED
#define NV_SDMMCW2MC_BR_ALLOWSR  NV_MC_DISABLED
#define NV_SDMMCW2MC_BR_OFFYUV  NV_MC_DISABLED
#define NV_SDMMCW2MC_BWHW  NV_MC_DISABLED
#define NV_SDMMCW2MC_BWVW  NV_MC_DISABLED
#define NV_SDMMCW2MC_BWSW  NV_MC_DISABLED
#define NV_SDMMCW2MC_BWXY  NV_MC_DISABLED
#define NV_SDMMCW2MC_BWPK  NV_MC_DISABLED
#define NV_SDMMCW2MC_BW_ALLOWSW  NV_MC_DISABLED
#define NV_SDMMCW2MC_CWHW  NV_MC_DISABLED
#define NV_SDMMCW2MC_CWSW  NV_MC_DISABLED
#define NV_SDMMCW2MC_SR_HP  NV_MC_DISABLED
#define NV_SDMMCW2MC_SW_HP  NV_MC_DISABLED
#define NV_SDMMCW2MC_BR_HP  NV_MC_DISABLED
#define NV_SDMMCW2MC_BW_HP  NV_MC_DISABLED
#define NV_SDMMCW2MC_CW_HP  NV_MC_DISABLED
#define NV_SDMMCWAB2MC_BRHW  NV_MC_DISABLED
#define NV_SDMMCWAB2MC_BRVW  NV_MC_DISABLED
#define NV_SDMMCWAB2MC_BRSW  NV_MC_DISABLED
#define NV_SDMMCWAB2MC_BRYUV  NV_MC_DISABLED
#define NV_SDMMCWAB2MC_BRYUVAVG  NV_MC_DISABLED
#define NV_SDMMCWAB2MC_BR_ALLOWSR  NV_MC_DISABLED
#define NV_SDMMCWAB2MC_BR_OFFYUV  NV_MC_DISABLED
#define NV_SDMMCWAB2MC_BWHW  NV_MC_DISABLED
#define NV_SDMMCWAB2MC_BWVW  NV_MC_DISABLED
#define NV_SDMMCWAB2MC_BWSW  NV_MC_DISABLED
#define NV_SDMMCWAB2MC_BWXY  NV_MC_DISABLED
#define NV_SDMMCWAB2MC_BWPK  NV_MC_DISABLED
#define NV_SDMMCWAB2MC_BW_ALLOWSW  NV_MC_DISABLED
#define NV_SDMMCWAB2MC_CWHW  NV_MC_DISABLED
#define NV_SDMMCWAB2MC_CWSW  NV_MC_DISABLED
#define NV_SDMMCWAB2MC_SR_HP  NV_MC_DISABLED
#define NV_SDMMCWAB2MC_SW_HP  NV_MC_DISABLED
#define NV_SDMMCWAB2MC_BR_HP  NV_MC_DISABLED
#define NV_SDMMCWAB2MC_BW_HP  NV_MC_DISABLED
#define NV_SDMMCWAB2MC_CW_HP  NV_MC_DISABLED
#define NV_VICSRD2MC_BRHW  NV_MC_DISABLED
#define NV_VICSRD2MC_BRVW  NV_MC_DISABLED
#define NV_VICSRD2MC_BRSW  NV_MC_DISABLED
#define NV_VICSRD2MC_BRYUV  NV_MC_DISABLED
#define NV_VICSRD2MC_BRYUVAVG  NV_MC_DISABLED
#define NV_VICSRD2MC_BR_ALLOWSR  NV_MC_DISABLED
#define NV_VICSRD2MC_BR_OFFYUV  NV_MC_DISABLED
#define NV_VICSRD2MC_BWHW  NV_MC_DISABLED
#define NV_VICSRD2MC_BWVW  NV_MC_DISABLED
#define NV_VICSRD2MC_BWSW  NV_MC_DISABLED
#define NV_VICSRD2MC_BWXY  NV_MC_DISABLED
#define NV_VICSRD2MC_BWPK  NV_MC_DISABLED
#define NV_VICSRD2MC_BW_ALLOWSW  NV_MC_DISABLED
#define NV_VICSRD2MC_CWHW  NV_MC_DISABLED
#define NV_VICSRD2MC_CWSW  NV_MC_DISABLED
#define NV_VICSRD2MC_SR_HP  NV_MC_DISABLED
#define NV_VICSRD2MC_SW_HP  NV_MC_DISABLED
#define NV_VICSRD2MC_BR_HP  NV_MC_DISABLED
#define NV_VICSRD2MC_BW_HP  NV_MC_DISABLED
#define NV_VICSRD2MC_CW_HP  NV_MC_DISABLED
#define NV_VICSWR2MC_BRHW  NV_MC_DISABLED
#define NV_VICSWR2MC_BRVW  NV_MC_DISABLED
#define NV_VICSWR2MC_BRSW  NV_MC_DISABLED
#define NV_VICSWR2MC_BRYUV  NV_MC_DISABLED
#define NV_VICSWR2MC_BRYUVAVG  NV_MC_DISABLED
#define NV_VICSWR2MC_BR_ALLOWSR  NV_MC_DISABLED
#define NV_VICSWR2MC_BR_OFFYUV  NV_MC_DISABLED
#define NV_VICSWR2MC_BWHW  NV_MC_DISABLED
#define NV_VICSWR2MC_BWVW  NV_MC_DISABLED
#define NV_VICSWR2MC_BWSW  NV_MC_DISABLED
#define NV_VICSWR2MC_BWXY  NV_MC_DISABLED
#define NV_VICSWR2MC_BWPK  NV_MC_DISABLED
#define NV_VICSWR2MC_BW_ALLOWSW  NV_MC_DISABLED
#define NV_VICSWR2MC_CWHW  NV_MC_DISABLED
#define NV_VICSWR2MC_CWSW  NV_MC_DISABLED
#define NV_VICSWR2MC_SR_HP  NV_MC_DISABLED
#define NV_VICSWR2MC_SW_HP  NV_MC_DISABLED
#define NV_VICSWR2MC_BR_HP  NV_MC_DISABLED
#define NV_VICSWR2MC_BW_HP  NV_MC_DISABLED
#define NV_VICSWR2MC_CW_HP  NV_MC_DISABLED
#define NV_VIW2MC_BRHW  NV_MC_DISABLED
#define NV_VIW2MC_BRVW  NV_MC_DISABLED
#define NV_VIW2MC_BRSW  NV_MC_DISABLED
#define NV_VIW2MC_BRYUV  NV_MC_DISABLED
#define NV_VIW2MC_BRYUVAVG  NV_MC_DISABLED
#define NV_VIW2MC_BR_ALLOWSR  NV_MC_DISABLED
#define NV_VIW2MC_BR_OFFYUV  NV_MC_DISABLED
#define NV_VIW2MC_BWHW  NV_MC_DISABLED
#define NV_VIW2MC_BWVW  NV_MC_DISABLED
#define NV_VIW2MC_BWSW  NV_MC_DISABLED
#define NV_VIW2MC_BWXY  NV_MC_DISABLED
#define NV_VIW2MC_BWPK  NV_MC_DISABLED
#define NV_VIW2MC_BW_ALLOWSW  NV_MC_DISABLED
#define NV_VIW2MC_CWHW  NV_MC_DISABLED
#define NV_VIW2MC_CWSW  NV_MC_DISABLED
#define NV_VIW2MC_SR_HP  NV_MC_DISABLED
#define NV_VIW2MC_SW_HP  NV_MC_DISABLED
#define NV_VIW2MC_BR_HP  NV_MC_DISABLED
#define NV_VIW2MC_BW_HP  NV_MC_DISABLED
#define NV_VIW2MC_CW_HP  NV_MC_DISABLED
#define NV_NVDECSRD2MC_BRHW  NV_MC_DISABLED
#define NV_NVDECSRD2MC_BRVW  NV_MC_DISABLED
#define NV_NVDECSRD2MC_BRSW  NV_MC_DISABLED
#define NV_NVDECSRD2MC_BRYUV  NV_MC_DISABLED
#define NV_NVDECSRD2MC_BRYUVAVG  NV_MC_DISABLED
#define NV_NVDECSRD2MC_BR_ALLOWSR  NV_MC_DISABLED
#define NV_NVDECSRD2MC_BR_OFFYUV  NV_MC_DISABLED
#define NV_NVDECSRD2MC_BWHW  NV_MC_DISABLED
#define NV_NVDECSRD2MC_BWVW  NV_MC_DISABLED
#define NV_NVDECSRD2MC_BWSW  NV_MC_DISABLED
#define NV_NVDECSRD2MC_BWXY  NV_MC_DISABLED
#define NV_NVDECSRD2MC_BWPK  NV_MC_DISABLED
#define NV_NVDECSRD2MC_BW_ALLOWSW  NV_MC_DISABLED
#define NV_NVDECSRD2MC_CWHW  NV_MC_DISABLED
#define NV_NVDECSRD2MC_CWSW  NV_MC_DISABLED
#define NV_NVDECSRD2MC_SR_HP  NV_MC_DISABLED
#define NV_NVDECSRD2MC_SW_HP  NV_MC_DISABLED
#define NV_NVDECSRD2MC_BR_HP  NV_MC_DISABLED
#define NV_NVDECSRD2MC_BW_HP  NV_MC_DISABLED
#define NV_NVDECSRD2MC_CW_HP  NV_MC_DISABLED
#define NV_NVDECSWR2MC_BRHW  NV_MC_DISABLED
#define NV_NVDECSWR2MC_BRVW  NV_MC_DISABLED
#define NV_NVDECSWR2MC_BRSW  NV_MC_DISABLED
#define NV_NVDECSWR2MC_BRYUV  NV_MC_DISABLED
#define NV_NVDECSWR2MC_BRYUVAVG  NV_MC_DISABLED
#define NV_NVDECSWR2MC_BR_ALLOWSR  NV_MC_DISABLED
#define NV_NVDECSWR2MC_BR_OFFYUV  NV_MC_DISABLED
#define NV_NVDECSWR2MC_BWHW  NV_MC_DISABLED
#define NV_NVDECSWR2MC_BWVW  NV_MC_DISABLED
#define NV_NVDECSWR2MC_BWSW  NV_MC_DISABLED
#define NV_NVDECSWR2MC_BWXY  NV_MC_DISABLED
#define NV_NVDECSWR2MC_BWPK  NV_MC_DISABLED
#define NV_NVDECSWR2MC_BW_ALLOWSW  NV_MC_DISABLED
#define NV_NVDECSWR2MC_CWHW  NV_MC_DISABLED
#define NV_NVDECSWR2MC_CWSW  NV_MC_DISABLED
#define NV_NVDECSWR2MC_SR_HP  NV_MC_DISABLED
#define NV_NVDECSWR2MC_SW_HP  NV_MC_DISABLED
#define NV_NVDECSWR2MC_BR_HP  NV_MC_DISABLED
#define NV_NVDECSWR2MC_BW_HP  NV_MC_DISABLED
#define NV_NVDECSWR2MC_CW_HP  NV_MC_DISABLED
#define NV_APER2MC_BRHW  NV_MC_DISABLED
#define NV_APER2MC_BRVW  NV_MC_DISABLED
#define NV_APER2MC_BRSW  NV_MC_DISABLED
#define NV_APER2MC_BRYUV  NV_MC_DISABLED
#define NV_APER2MC_BRYUVAVG  NV_MC_DISABLED
#define NV_APER2MC_BR_ALLOWSR  NV_MC_DISABLED
#define NV_APER2MC_BR_OFFYUV  NV_MC_DISABLED
#define NV_APER2MC_BWHW  NV_MC_DISABLED
#define NV_APER2MC_BWVW  NV_MC_DISABLED
#define NV_APER2MC_BWSW  NV_MC_DISABLED
#define NV_APER2MC_BWXY  NV_MC_DISABLED
#define NV_APER2MC_BWPK  NV_MC_DISABLED
#define NV_APER2MC_BW_ALLOWSW  NV_MC_DISABLED
#define NV_APER2MC_CWHW  NV_MC_DISABLED
#define NV_APER2MC_CWSW  NV_MC_DISABLED
#define NV_APER2MC_SR_HP  NV_MC_DISABLED
#define NV_APER2MC_SW_HP  NV_MC_DISABLED
#define NV_APER2MC_BR_HP  NV_MC_DISABLED
#define NV_APER2MC_BW_HP  NV_MC_DISABLED
#define NV_APER2MC_CW_HP  NV_MC_DISABLED
#define NV_APEW2MC_BRHW  NV_MC_DISABLED
#define NV_APEW2MC_BRVW  NV_MC_DISABLED
#define NV_APEW2MC_BRSW  NV_MC_DISABLED
#define NV_APEW2MC_BRYUV  NV_MC_DISABLED
#define NV_APEW2MC_BRYUVAVG  NV_MC_DISABLED
#define NV_APEW2MC_BR_ALLOWSR  NV_MC_DISABLED
#define NV_APEW2MC_BR_OFFYUV  NV_MC_DISABLED
#define NV_APEW2MC_BWHW  NV_MC_DISABLED
#define NV_APEW2MC_BWVW  NV_MC_DISABLED
#define NV_APEW2MC_BWSW  NV_MC_DISABLED
#define NV_APEW2MC_BWXY  NV_MC_DISABLED
#define NV_APEW2MC_BWPK  NV_MC_DISABLED
#define NV_APEW2MC_BW_ALLOWSW  NV_MC_DISABLED
#define NV_APEW2MC_CWHW  NV_MC_DISABLED
#define NV_APEW2MC_CWSW  NV_MC_DISABLED
#define NV_APEW2MC_SR_HP  NV_MC_DISABLED
#define NV_APEW2MC_SW_HP  NV_MC_DISABLED
#define NV_APEW2MC_BR_HP  NV_MC_DISABLED
#define NV_APEW2MC_BW_HP  NV_MC_DISABLED
#define NV_APEW2MC_CW_HP  NV_MC_DISABLED
#define NV_NVJPGSRD2MC_BRHW  NV_MC_DISABLED
#define NV_NVJPGSRD2MC_BRVW  NV_MC_DISABLED
#define NV_NVJPGSRD2MC_BRSW  NV_MC_DISABLED
#define NV_NVJPGSRD2MC_BRYUV  NV_MC_DISABLED
#define NV_NVJPGSRD2MC_BRYUVAVG  NV_MC_DISABLED
#define NV_NVJPGSRD2MC_BR_ALLOWSR  NV_MC_DISABLED
#define NV_NVJPGSRD2MC_BR_OFFYUV  NV_MC_DISABLED
#define NV_NVJPGSRD2MC_BWHW  NV_MC_DISABLED
#define NV_NVJPGSRD2MC_BWVW  NV_MC_DISABLED
#define NV_NVJPGSRD2MC_BWSW  NV_MC_DISABLED
#define NV_NVJPGSRD2MC_BWXY  NV_MC_DISABLED
#define NV_NVJPGSRD2MC_BWPK  NV_MC_DISABLED
#define NV_NVJPGSRD2MC_BW_ALLOWSW  NV_MC_DISABLED
#define NV_NVJPGSRD2MC_CWHW  NV_MC_DISABLED
#define NV_NVJPGSRD2MC_CWSW  NV_MC_DISABLED
#define NV_NVJPGSRD2MC_SR_HP  NV_MC_DISABLED
#define NV_NVJPGSRD2MC_SW_HP  NV_MC_DISABLED
#define NV_NVJPGSRD2MC_BR_HP  NV_MC_DISABLED
#define NV_NVJPGSRD2MC_BW_HP  NV_MC_DISABLED
#define NV_NVJPGSRD2MC_CW_HP  NV_MC_DISABLED
#define NV_NVJPGSWR2MC_BRHW  NV_MC_DISABLED
#define NV_NVJPGSWR2MC_BRVW  NV_MC_DISABLED
#define NV_NVJPGSWR2MC_BRSW  NV_MC_DISABLED
#define NV_NVJPGSWR2MC_BRYUV  NV_MC_DISABLED
#define NV_NVJPGSWR2MC_BRYUVAVG  NV_MC_DISABLED
#define NV_NVJPGSWR2MC_BR_ALLOWSR  NV_MC_DISABLED
#define NV_NVJPGSWR2MC_BR_OFFYUV  NV_MC_DISABLED
#define NV_NVJPGSWR2MC_BWHW  NV_MC_DISABLED
#define NV_NVJPGSWR2MC_BWVW  NV_MC_DISABLED
#define NV_NVJPGSWR2MC_BWSW  NV_MC_DISABLED
#define NV_NVJPGSWR2MC_BWXY  NV_MC_DISABLED
#define NV_NVJPGSWR2MC_BWPK  NV_MC_DISABLED
#define NV_NVJPGSWR2MC_BW_ALLOWSW  NV_MC_DISABLED
#define NV_NVJPGSWR2MC_CWHW  NV_MC_DISABLED
#define NV_NVJPGSWR2MC_CWSW  NV_MC_DISABLED
#define NV_NVJPGSWR2MC_SR_HP  NV_MC_DISABLED
#define NV_NVJPGSWR2MC_SW_HP  NV_MC_DISABLED
#define NV_NVJPGSWR2MC_BR_HP  NV_MC_DISABLED
#define NV_NVJPGSWR2MC_BW_HP  NV_MC_DISABLED
#define NV_NVJPGSWR2MC_CW_HP  NV_MC_DISABLED
#define NV_SESRD2MC_BRHW  NV_MC_DISABLED
#define NV_SESRD2MC_BRVW  NV_MC_DISABLED
#define NV_SESRD2MC_BRSW  NV_MC_DISABLED
#define NV_SESRD2MC_BRYUV  NV_MC_DISABLED
#define NV_SESRD2MC_BRYUVAVG  NV_MC_DISABLED
#define NV_SESRD2MC_BR_ALLOWSR  NV_MC_DISABLED
#define NV_SESRD2MC_BR_OFFYUV  NV_MC_DISABLED
#define NV_SESRD2MC_BWHW  NV_MC_DISABLED
#define NV_SESRD2MC_BWVW  NV_MC_DISABLED
#define NV_SESRD2MC_BWSW  NV_MC_DISABLED
#define NV_SESRD2MC_BWXY  NV_MC_DISABLED
#define NV_SESRD2MC_BWPK  NV_MC_DISABLED
#define NV_SESRD2MC_BW_ALLOWSW  NV_MC_DISABLED
#define NV_SESRD2MC_CWHW  NV_MC_DISABLED
#define NV_SESRD2MC_CWSW  NV_MC_DISABLED
#define NV_SESRD2MC_SR_HP  NV_MC_DISABLED
#define NV_SESRD2MC_SW_HP  NV_MC_DISABLED
#define NV_SESRD2MC_BR_HP  NV_MC_DISABLED
#define NV_SESRD2MC_BW_HP  NV_MC_DISABLED
#define NV_SESRD2MC_CW_HP  NV_MC_DISABLED
#define NV_SESWR2MC_BRHW  NV_MC_DISABLED
#define NV_SESWR2MC_BRVW  NV_MC_DISABLED
#define NV_SESWR2MC_BRSW  NV_MC_DISABLED
#define NV_SESWR2MC_BRYUV  NV_MC_DISABLED
#define NV_SESWR2MC_BRYUVAVG  NV_MC_DISABLED
#define NV_SESWR2MC_BR_ALLOWSR  NV_MC_DISABLED
#define NV_SESWR2MC_BR_OFFYUV  NV_MC_DISABLED
#define NV_SESWR2MC_BWHW  NV_MC_DISABLED
#define NV_SESWR2MC_BWVW  NV_MC_DISABLED
#define NV_SESWR2MC_BWSW  NV_MC_DISABLED
#define NV_SESWR2MC_BWXY  NV_MC_DISABLED
#define NV_SESWR2MC_BWPK  NV_MC_DISABLED
#define NV_SESWR2MC_BW_ALLOWSW  NV_MC_DISABLED
#define NV_SESWR2MC_CWHW  NV_MC_DISABLED
#define NV_SESWR2MC_CWSW  NV_MC_DISABLED
#define NV_SESWR2MC_SR_HP  NV_MC_DISABLED
#define NV_SESWR2MC_SW_HP  NV_MC_DISABLED
#define NV_SESWR2MC_BR_HP  NV_MC_DISABLED
#define NV_SESWR2MC_BW_HP  NV_MC_DISABLED
#define NV_SESWR2MC_CW_HP  NV_MC_DISABLED
#define NV_ETRR2MC_BRHW  NV_MC_DISABLED
#define NV_ETRR2MC_BRVW  NV_MC_DISABLED
#define NV_ETRR2MC_BRSW  NV_MC_DISABLED
#define NV_ETRR2MC_BRYUV  NV_MC_DISABLED
#define NV_ETRR2MC_BRYUVAVG  NV_MC_DISABLED
#define NV_ETRR2MC_BR_ALLOWSR  NV_MC_DISABLED
#define NV_ETRR2MC_BR_OFFYUV  NV_MC_DISABLED
#define NV_ETRR2MC_BWHW  NV_MC_DISABLED
#define NV_ETRR2MC_BWVW  NV_MC_DISABLED
#define NV_ETRR2MC_BWSW  NV_MC_DISABLED
#define NV_ETRR2MC_BWXY  NV_MC_DISABLED
#define NV_ETRR2MC_BWPK  NV_MC_DISABLED
#define NV_ETRR2MC_BW_ALLOWSW  NV_MC_DISABLED
#define NV_ETRR2MC_CWHW  NV_MC_DISABLED
#define NV_ETRR2MC_CWSW  NV_MC_DISABLED
#define NV_ETRR2MC_SR_HP  NV_MC_DISABLED
#define NV_ETRR2MC_SW_HP  NV_MC_DISABLED
#define NV_ETRR2MC_BR_HP  NV_MC_DISABLED
#define NV_ETRR2MC_BW_HP  NV_MC_DISABLED
#define NV_ETRR2MC_CW_HP  NV_MC_DISABLED
#define NV_ETRW2MC_BRHW  NV_MC_DISABLED
#define NV_ETRW2MC_BRVW  NV_MC_DISABLED
#define NV_ETRW2MC_BRSW  NV_MC_DISABLED
#define NV_ETRW2MC_BRYUV  NV_MC_DISABLED
#define NV_ETRW2MC_BRYUVAVG  NV_MC_DISABLED
#define NV_ETRW2MC_BR_ALLOWSR  NV_MC_DISABLED
#define NV_ETRW2MC_BR_OFFYUV  NV_MC_DISABLED
#define NV_ETRW2MC_BWHW  NV_MC_DISABLED
#define NV_ETRW2MC_BWVW  NV_MC_DISABLED
#define NV_ETRW2MC_BWSW  NV_MC_DISABLED
#define NV_ETRW2MC_BWXY  NV_MC_DISABLED
#define NV_ETRW2MC_BWPK  NV_MC_DISABLED
#define NV_ETRW2MC_BW_ALLOWSW  NV_MC_DISABLED
#define NV_ETRW2MC_CWHW  NV_MC_DISABLED
#define NV_ETRW2MC_CWSW  NV_MC_DISABLED
#define NV_ETRW2MC_SR_HP  NV_MC_DISABLED
#define NV_ETRW2MC_SW_HP  NV_MC_DISABLED
#define NV_ETRW2MC_BR_HP  NV_MC_DISABLED
#define NV_ETRW2MC_BW_HP  NV_MC_DISABLED
#define NV_ETRW2MC_CW_HP  NV_MC_DISABLED
#define NV_TSECSRDB2MC_BRHW  NV_MC_DISABLED
#define NV_TSECSRDB2MC_BRVW  NV_MC_DISABLED
#define NV_TSECSRDB2MC_BRSW  NV_MC_DISABLED
#define NV_TSECSRDB2MC_BRYUV  NV_MC_DISABLED
#define NV_TSECSRDB2MC_BRYUVAVG  NV_MC_DISABLED
#define NV_TSECSRDB2MC_BR_ALLOWSR  NV_MC_DISABLED
#define NV_TSECSRDB2MC_BR_OFFYUV  NV_MC_DISABLED
#define NV_TSECSRDB2MC_BWHW  NV_MC_DISABLED
#define NV_TSECSRDB2MC_BWVW  NV_MC_DISABLED
#define NV_TSECSRDB2MC_BWSW  NV_MC_DISABLED
#define NV_TSECSRDB2MC_BWXY  NV_MC_DISABLED
#define NV_TSECSRDB2MC_BWPK  NV_MC_DISABLED
#define NV_TSECSRDB2MC_BW_ALLOWSW  NV_MC_DISABLED
#define NV_TSECSRDB2MC_CWHW  NV_MC_DISABLED
#define NV_TSECSRDB2MC_CWSW  NV_MC_DISABLED
#define NV_TSECSRDB2MC_SR_HP  NV_MC_DISABLED
#define NV_TSECSRDB2MC_SW_HP  NV_MC_DISABLED
#define NV_TSECSRDB2MC_BR_HP  NV_MC_DISABLED
#define NV_TSECSRDB2MC_BW_HP  NV_MC_DISABLED
#define NV_TSECSRDB2MC_CW_HP  NV_MC_DISABLED
#define NV_TSECSWRB2MC_BRHW  NV_MC_DISABLED
#define NV_TSECSWRB2MC_BRVW  NV_MC_DISABLED
#define NV_TSECSWRB2MC_BRSW  NV_MC_DISABLED
#define NV_TSECSWRB2MC_BRYUV  NV_MC_DISABLED
#define NV_TSECSWRB2MC_BRYUVAVG  NV_MC_DISABLED
#define NV_TSECSWRB2MC_BR_ALLOWSR  NV_MC_DISABLED
#define NV_TSECSWRB2MC_BR_OFFYUV  NV_MC_DISABLED
#define NV_TSECSWRB2MC_BWHW  NV_MC_DISABLED
#define NV_TSECSWRB2MC_BWVW  NV_MC_DISABLED
#define NV_TSECSWRB2MC_BWSW  NV_MC_DISABLED
#define NV_TSECSWRB2MC_BWXY  NV_MC_DISABLED
#define NV_TSECSWRB2MC_BWPK  NV_MC_DISABLED
#define NV_TSECSWRB2MC_BW_ALLOWSW  NV_MC_DISABLED
#define NV_TSECSWRB2MC_CWHW  NV_MC_DISABLED
#define NV_TSECSWRB2MC_CWSW  NV_MC_DISABLED
#define NV_TSECSWRB2MC_SR_HP  NV_MC_DISABLED
#define NV_TSECSWRB2MC_SW_HP  NV_MC_DISABLED
#define NV_TSECSWRB2MC_BR_HP  NV_MC_DISABLED
#define NV_TSECSWRB2MC_BW_HP  NV_MC_DISABLED
#define NV_TSECSWRB2MC_CW_HP  NV_MC_DISABLED
#define NV_GPUSRD22MC_BRHW  NV_MC_DISABLED
#define NV_GPUSRD22MC_BRVW  NV_MC_DISABLED
#define NV_GPUSRD22MC_BRSW  NV_MC_DISABLED
#define NV_GPUSRD22MC_BRYUV  NV_MC_DISABLED
#define NV_GPUSRD22MC_BRYUVAVG  NV_MC_DISABLED
#define NV_GPUSRD22MC_BR_ALLOWSR  NV_MC_DISABLED
#define NV_GPUSRD22MC_BR_OFFYUV  NV_MC_DISABLED
#define NV_GPUSRD22MC_BWHW  NV_MC_DISABLED
#define NV_GPUSRD22MC_BWVW  NV_MC_DISABLED
#define NV_GPUSRD22MC_BWSW  NV_MC_DISABLED
#define NV_GPUSRD22MC_BWXY  NV_MC_DISABLED
#define NV_GPUSRD22MC_BWPK  NV_MC_DISABLED
#define NV_GPUSRD22MC_BW_ALLOWSW  NV_MC_DISABLED
#define NV_GPUSRD22MC_CWHW  NV_MC_DISABLED
#define NV_GPUSRD22MC_CWSW  NV_MC_DISABLED
#define NV_GPUSRD22MC_SR_HP  NV_MC_DISABLED
#define NV_GPUSRD22MC_SW_HP  NV_MC_DISABLED
#define NV_GPUSRD22MC_BR_HP  NV_MC_DISABLED
#define NV_GPUSRD22MC_BW_HP  NV_MC_DISABLED
#define NV_GPUSRD22MC_CW_HP  NV_MC_DISABLED
#define NV_GPUSWR22MC_BRHW  NV_MC_DISABLED
#define NV_GPUSWR22MC_BRVW  NV_MC_DISABLED
#define NV_GPUSWR22MC_BRSW  NV_MC_DISABLED
#define NV_GPUSWR22MC_BRYUV  NV_MC_DISABLED
#define NV_GPUSWR22MC_BRYUVAVG  NV_MC_DISABLED
#define NV_GPUSWR22MC_BR_ALLOWSR  NV_MC_DISABLED
#define NV_GPUSWR22MC_BR_OFFYUV  NV_MC_DISABLED
#define NV_GPUSWR22MC_BWHW  NV_MC_DISABLED
#define NV_GPUSWR22MC_BWVW  NV_MC_DISABLED
#define NV_GPUSWR22MC_BWSW  NV_MC_DISABLED
#define NV_GPUSWR22MC_BWXY  NV_MC_DISABLED
#define NV_GPUSWR22MC_BWPK  NV_MC_DISABLED
#define NV_GPUSWR22MC_BW_ALLOWSW  NV_MC_DISABLED
#define NV_GPUSWR22MC_CWHW  NV_MC_DISABLED
#define NV_GPUSWR22MC_CWSW  NV_MC_DISABLED
#define NV_GPUSWR22MC_SR_HP  NV_MC_DISABLED
#define NV_GPUSWR22MC_SW_HP  NV_MC_DISABLED
#define NV_GPUSWR22MC_BR_HP  NV_MC_DISABLED
#define NV_GPUSWR22MC_BW_HP  NV_MC_DISABLED
#define NV_GPUSWR22MC_CW_HP  NV_MC_DISABLED
#define NV_AXISR2MC_BRHW  NV_MC_DISABLED
#define NV_AXISR2MC_BRVW  NV_MC_DISABLED
#define NV_AXISR2MC_BRSW  NV_MC_DISABLED
#define NV_AXISR2MC_BRYUV  NV_MC_DISABLED
#define NV_AXISR2MC_BRYUVAVG  NV_MC_DISABLED
#define NV_AXISR2MC_BR_ALLOWSR  NV_MC_DISABLED
#define NV_AXISR2MC_BR_OFFYUV  NV_MC_DISABLED
#define NV_AXISR2MC_BWHW  NV_MC_DISABLED
#define NV_AXISR2MC_BWVW  NV_MC_DISABLED
#define NV_AXISR2MC_BWSW  NV_MC_DISABLED
#define NV_AXISR2MC_BWXY  NV_MC_DISABLED
#define NV_AXISR2MC_BWPK  NV_MC_DISABLED
#define NV_AXISR2MC_BW_ALLOWSW  NV_MC_DISABLED
#define NV_AXISR2MC_CWHW  NV_MC_DISABLED
#define NV_AXISR2MC_CWSW  NV_MC_DISABLED
#define NV_AXISR2MC_SR_HP  NV_MC_DISABLED
#define NV_AXISR2MC_SW_HP  NV_MC_DISABLED
#define NV_AXISR2MC_BR_HP  NV_MC_DISABLED
#define NV_AXISR2MC_BW_HP  NV_MC_DISABLED
#define NV_AXISR2MC_CW_HP  NV_MC_DISABLED
#define NV_AXISW2MC_BRHW  NV_MC_DISABLED
#define NV_AXISW2MC_BRVW  NV_MC_DISABLED
#define NV_AXISW2MC_BRSW  NV_MC_DISABLED
#define NV_AXISW2MC_BRYUV  NV_MC_DISABLED
#define NV_AXISW2MC_BRYUVAVG  NV_MC_DISABLED
#define NV_AXISW2MC_BR_ALLOWSR  NV_MC_DISABLED
#define NV_AXISW2MC_BR_OFFYUV  NV_MC_DISABLED
#define NV_AXISW2MC_BWHW  NV_MC_DISABLED
#define NV_AXISW2MC_BWVW  NV_MC_DISABLED
#define NV_AXISW2MC_BWSW  NV_MC_DISABLED
#define NV_AXISW2MC_BWXY  NV_MC_DISABLED
#define NV_AXISW2MC_BWPK  NV_MC_DISABLED
#define NV_AXISW2MC_BW_ALLOWSW  NV_MC_DISABLED
#define NV_AXISW2MC_CWHW  NV_MC_DISABLED
#define NV_AXISW2MC_CWSW  NV_MC_DISABLED
#define NV_AXISW2MC_SR_HP  NV_MC_DISABLED
#define NV_AXISW2MC_SW_HP  NV_MC_DISABLED
#define NV_AXISW2MC_BR_HP  NV_MC_DISABLED
#define NV_AXISW2MC_BW_HP  NV_MC_DISABLED
#define NV_AXISW2MC_CW_HP  NV_MC_DISABLED
#define NV_EQOSR2MC_BRHW  NV_MC_DISABLED
#define NV_EQOSR2MC_BRVW  NV_MC_DISABLED
#define NV_EQOSR2MC_BRSW  NV_MC_DISABLED
#define NV_EQOSR2MC_BRYUV  NV_MC_DISABLED
#define NV_EQOSR2MC_BRYUVAVG  NV_MC_DISABLED
#define NV_EQOSR2MC_BR_ALLOWSR  NV_MC_DISABLED
#define NV_EQOSR2MC_BR_OFFYUV  NV_MC_DISABLED
#define NV_EQOSR2MC_BWHW  NV_MC_DISABLED
#define NV_EQOSR2MC_BWVW  NV_MC_DISABLED
#define NV_EQOSR2MC_BWSW  NV_MC_DISABLED
#define NV_EQOSR2MC_BWXY  NV_MC_DISABLED
#define NV_EQOSR2MC_BWPK  NV_MC_DISABLED
#define NV_EQOSR2MC_BW_ALLOWSW  NV_MC_DISABLED
#define NV_EQOSR2MC_CWHW  NV_MC_DISABLED
#define NV_EQOSR2MC_CWSW  NV_MC_DISABLED
#define NV_EQOSR2MC_SR_HP  NV_MC_DISABLED
#define NV_EQOSR2MC_SW_HP  NV_MC_DISABLED
#define NV_EQOSR2MC_BR_HP  NV_MC_DISABLED
#define NV_EQOSR2MC_BW_HP  NV_MC_DISABLED
#define NV_EQOSR2MC_CW_HP  NV_MC_DISABLED
#define NV_EQOSW2MC_BRHW  NV_MC_DISABLED
#define NV_EQOSW2MC_BRVW  NV_MC_DISABLED
#define NV_EQOSW2MC_BRSW  NV_MC_DISABLED
#define NV_EQOSW2MC_BRYUV  NV_MC_DISABLED
#define NV_EQOSW2MC_BRYUVAVG  NV_MC_DISABLED
#define NV_EQOSW2MC_BR_ALLOWSR  NV_MC_DISABLED
#define NV_EQOSW2MC_BR_OFFYUV  NV_MC_DISABLED
#define NV_EQOSW2MC_BWHW  NV_MC_DISABLED
#define NV_EQOSW2MC_BWVW  NV_MC_DISABLED
#define NV_EQOSW2MC_BWSW  NV_MC_DISABLED
#define NV_EQOSW2MC_BWXY  NV_MC_DISABLED
#define NV_EQOSW2MC_BWPK  NV_MC_DISABLED
#define NV_EQOSW2MC_BW_ALLOWSW  NV_MC_DISABLED
#define NV_EQOSW2MC_CWHW  NV_MC_DISABLED
#define NV_EQOSW2MC_CWSW  NV_MC_DISABLED
#define NV_EQOSW2MC_SR_HP  NV_MC_DISABLED
#define NV_EQOSW2MC_SW_HP  NV_MC_DISABLED
#define NV_EQOSW2MC_BR_HP  NV_MC_DISABLED
#define NV_EQOSW2MC_BW_HP  NV_MC_DISABLED
#define NV_EQOSW2MC_CW_HP  NV_MC_DISABLED
#define NV_UFSHCR2MC_BRHW  NV_MC_DISABLED
#define NV_UFSHCR2MC_BRVW  NV_MC_DISABLED
#define NV_UFSHCR2MC_BRSW  NV_MC_DISABLED
#define NV_UFSHCR2MC_BRYUV  NV_MC_DISABLED
#define NV_UFSHCR2MC_BRYUVAVG  NV_MC_DISABLED
#define NV_UFSHCR2MC_BR_ALLOWSR  NV_MC_DISABLED
#define NV_UFSHCR2MC_BR_OFFYUV  NV_MC_DISABLED
#define NV_UFSHCR2MC_BWHW  NV_MC_DISABLED
#define NV_UFSHCR2MC_BWVW  NV_MC_DISABLED
#define NV_UFSHCR2MC_BWSW  NV_MC_DISABLED
#define NV_UFSHCR2MC_BWXY  NV_MC_DISABLED
#define NV_UFSHCR2MC_BWPK  NV_MC_DISABLED
#define NV_UFSHCR2MC_BW_ALLOWSW  NV_MC_DISABLED
#define NV_UFSHCR2MC_CWHW  NV_MC_DISABLED
#define NV_UFSHCR2MC_CWSW  NV_MC_DISABLED
#define NV_UFSHCR2MC_SR_HP  NV_MC_DISABLED
#define NV_UFSHCR2MC_SW_HP  NV_MC_DISABLED
#define NV_UFSHCR2MC_BR_HP  NV_MC_DISABLED
#define NV_UFSHCR2MC_BW_HP  NV_MC_DISABLED
#define NV_UFSHCR2MC_CW_HP  NV_MC_DISABLED
#define NV_UFSHCW2MC_BRHW  NV_MC_DISABLED
#define NV_UFSHCW2MC_BRVW  NV_MC_DISABLED
#define NV_UFSHCW2MC_BRSW  NV_MC_DISABLED
#define NV_UFSHCW2MC_BRYUV  NV_MC_DISABLED
#define NV_UFSHCW2MC_BRYUVAVG  NV_MC_DISABLED
#define NV_UFSHCW2MC_BR_ALLOWSR  NV_MC_DISABLED
#define NV_UFSHCW2MC_BR_OFFYUV  NV_MC_DISABLED
#define NV_UFSHCW2MC_BWHW  NV_MC_DISABLED
#define NV_UFSHCW2MC_BWVW  NV_MC_DISABLED
#define NV_UFSHCW2MC_BWSW  NV_MC_DISABLED
#define NV_UFSHCW2MC_BWXY  NV_MC_DISABLED
#define NV_UFSHCW2MC_BWPK  NV_MC_DISABLED
#define NV_UFSHCW2MC_BW_ALLOWSW  NV_MC_DISABLED
#define NV_UFSHCW2MC_CWHW  NV_MC_DISABLED
#define NV_UFSHCW2MC_CWSW  NV_MC_DISABLED
#define NV_UFSHCW2MC_SR_HP  NV_MC_DISABLED
#define NV_UFSHCW2MC_SW_HP  NV_MC_DISABLED
#define NV_UFSHCW2MC_BR_HP  NV_MC_DISABLED
#define NV_UFSHCW2MC_BW_HP  NV_MC_DISABLED
#define NV_UFSHCW2MC_CW_HP  NV_MC_DISABLED
#define NV_NVDISPLAYR2MC_BRHW  NV_MC_DISABLED
#define NV_NVDISPLAYR2MC_BRVW  NV_MC_DISABLED
#define NV_NVDISPLAYR2MC_BRSW  NV_MC_DISABLED
#define NV_NVDISPLAYR2MC_BRYUV  NV_MC_DISABLED
#define NV_NVDISPLAYR2MC_BRYUVAVG  NV_MC_DISABLED
#define NV_NVDISPLAYR2MC_BR_ALLOWSR  NV_MC_DISABLED
#define NV_NVDISPLAYR2MC_BR_OFFYUV  NV_MC_DISABLED
#define NV_NVDISPLAYR2MC_BWHW  NV_MC_DISABLED
#define NV_NVDISPLAYR2MC_BWVW  NV_MC_DISABLED
#define NV_NVDISPLAYR2MC_BWSW  NV_MC_DISABLED
#define NV_NVDISPLAYR2MC_BWXY  NV_MC_DISABLED
#define NV_NVDISPLAYR2MC_BWPK  NV_MC_DISABLED
#define NV_NVDISPLAYR2MC_BW_ALLOWSW  NV_MC_DISABLED
#define NV_NVDISPLAYR2MC_CWHW  NV_MC_DISABLED
#define NV_NVDISPLAYR2MC_CWSW  NV_MC_DISABLED
#define NV_NVDISPLAYR2MC_SR_HP  NV_MC_DISABLED
#define NV_NVDISPLAYR2MC_SW_HP  NV_MC_DISABLED
#define NV_NVDISPLAYR2MC_BR_HP  NV_MC_DISABLED
#define NV_NVDISPLAYR2MC_BW_HP  NV_MC_DISABLED
#define NV_NVDISPLAYR2MC_CW_HP  NV_MC_DISABLED
#define NV_BPMPR2MC_BRHW  NV_MC_DISABLED
#define NV_BPMPR2MC_BRVW  NV_MC_DISABLED
#define NV_BPMPR2MC_BRSW  NV_MC_DISABLED
#define NV_BPMPR2MC_BRYUV  NV_MC_DISABLED
#define NV_BPMPR2MC_BRYUVAVG  NV_MC_DISABLED
#define NV_BPMPR2MC_BR_ALLOWSR  NV_MC_DISABLED
#define NV_BPMPR2MC_BR_OFFYUV  NV_MC_DISABLED
#define NV_BPMPR2MC_BWHW  NV_MC_DISABLED
#define NV_BPMPR2MC_BWVW  NV_MC_DISABLED
#define NV_BPMPR2MC_BWSW  NV_MC_DISABLED
#define NV_BPMPR2MC_BWXY  NV_MC_DISABLED
#define NV_BPMPR2MC_BWPK  NV_MC_DISABLED
#define NV_BPMPR2MC_BW_ALLOWSW  NV_MC_DISABLED
#define NV_BPMPR2MC_CWHW  NV_MC_DISABLED
#define NV_BPMPR2MC_CWSW  NV_MC_DISABLED
#define NV_BPMPR2MC_SR_HP  NV_MC_DISABLED
#define NV_BPMPR2MC_SW_HP  NV_MC_DISABLED
#define NV_BPMPR2MC_BR_HP  NV_MC_DISABLED
#define NV_BPMPR2MC_BW_HP  NV_MC_DISABLED
#define NV_BPMPR2MC_CW_HP  NV_MC_DISABLED
#define NV_BPMPW2MC_BRHW  NV_MC_DISABLED
#define NV_BPMPW2MC_BRVW  NV_MC_DISABLED
#define NV_BPMPW2MC_BRSW  NV_MC_DISABLED
#define NV_BPMPW2MC_BRYUV  NV_MC_DISABLED
#define NV_BPMPW2MC_BRYUVAVG  NV_MC_DISABLED
#define NV_BPMPW2MC_BR_ALLOWSR  NV_MC_DISABLED
#define NV_BPMPW2MC_BR_OFFYUV  NV_MC_DISABLED
#define NV_BPMPW2MC_BWHW  NV_MC_DISABLED
#define NV_BPMPW2MC_BWVW  NV_MC_DISABLED
#define NV_BPMPW2MC_BWSW  NV_MC_DISABLED
#define NV_BPMPW2MC_BWXY  NV_MC_DISABLED
#define NV_BPMPW2MC_BWPK  NV_MC_DISABLED
#define NV_BPMPW2MC_BW_ALLOWSW  NV_MC_DISABLED
#define NV_BPMPW2MC_CWHW  NV_MC_DISABLED
#define NV_BPMPW2MC_CWSW  NV_MC_DISABLED
#define NV_BPMPW2MC_SR_HP  NV_MC_DISABLED
#define NV_BPMPW2MC_SW_HP  NV_MC_DISABLED
#define NV_BPMPW2MC_BR_HP  NV_MC_DISABLED
#define NV_BPMPW2MC_BW_HP  NV_MC_DISABLED
#define NV_BPMPW2MC_CW_HP  NV_MC_DISABLED
#define NV_BPMPDMAR2MC_BRHW  NV_MC_DISABLED
#define NV_BPMPDMAR2MC_BRVW  NV_MC_DISABLED
#define NV_BPMPDMAR2MC_BRSW  NV_MC_DISABLED
#define NV_BPMPDMAR2MC_BRYUV  NV_MC_DISABLED
#define NV_BPMPDMAR2MC_BRYUVAVG  NV_MC_DISABLED
#define NV_BPMPDMAR2MC_BR_ALLOWSR  NV_MC_DISABLED
#define NV_BPMPDMAR2MC_BR_OFFYUV  NV_MC_DISABLED
#define NV_BPMPDMAR2MC_BWHW  NV_MC_DISABLED
#define NV_BPMPDMAR2MC_BWVW  NV_MC_DISABLED
#define NV_BPMPDMAR2MC_BWSW  NV_MC_DISABLED
#define NV_BPMPDMAR2MC_BWXY  NV_MC_DISABLED
#define NV_BPMPDMAR2MC_BWPK  NV_MC_DISABLED
#define NV_BPMPDMAR2MC_BW_ALLOWSW  NV_MC_DISABLED
#define NV_BPMPDMAR2MC_CWHW  NV_MC_DISABLED
#define NV_BPMPDMAR2MC_CWSW  NV_MC_DISABLED
#define NV_BPMPDMAR2MC_SR_HP  NV_MC_DISABLED
#define NV_BPMPDMAR2MC_SW_HP  NV_MC_DISABLED
#define NV_BPMPDMAR2MC_BR_HP  NV_MC_DISABLED
#define NV_BPMPDMAR2MC_BW_HP  NV_MC_DISABLED
#define NV_BPMPDMAR2MC_CW_HP  NV_MC_DISABLED
#define NV_BPMPDMAW2MC_BRHW  NV_MC_DISABLED
#define NV_BPMPDMAW2MC_BRVW  NV_MC_DISABLED
#define NV_BPMPDMAW2MC_BRSW  NV_MC_DISABLED
#define NV_BPMPDMAW2MC_BRYUV  NV_MC_DISABLED
#define NV_BPMPDMAW2MC_BRYUVAVG  NV_MC_DISABLED
#define NV_BPMPDMAW2MC_BR_ALLOWSR  NV_MC_DISABLED
#define NV_BPMPDMAW2MC_BR_OFFYUV  NV_MC_DISABLED
#define NV_BPMPDMAW2MC_BWHW  NV_MC_DISABLED
#define NV_BPMPDMAW2MC_BWVW  NV_MC_DISABLED
#define NV_BPMPDMAW2MC_BWSW  NV_MC_DISABLED
#define NV_BPMPDMAW2MC_BWXY  NV_MC_DISABLED
#define NV_BPMPDMAW2MC_BWPK  NV_MC_DISABLED
#define NV_BPMPDMAW2MC_BW_ALLOWSW  NV_MC_DISABLED
#define NV_BPMPDMAW2MC_CWHW  NV_MC_DISABLED
#define NV_BPMPDMAW2MC_CWSW  NV_MC_DISABLED
#define NV_BPMPDMAW2MC_SR_HP  NV_MC_DISABLED
#define NV_BPMPDMAW2MC_SW_HP  NV_MC_DISABLED
#define NV_BPMPDMAW2MC_BR_HP  NV_MC_DISABLED
#define NV_BPMPDMAW2MC_BW_HP  NV_MC_DISABLED
#define NV_BPMPDMAW2MC_CW_HP  NV_MC_DISABLED
#define NV_AONR2MC_BRHW  NV_MC_DISABLED
#define NV_AONR2MC_BRVW  NV_MC_DISABLED
#define NV_AONR2MC_BRSW  NV_MC_DISABLED
#define NV_AONR2MC_BRYUV  NV_MC_DISABLED
#define NV_AONR2MC_BRYUVAVG  NV_MC_DISABLED
#define NV_AONR2MC_BR_ALLOWSR  NV_MC_DISABLED
#define NV_AONR2MC_BR_OFFYUV  NV_MC_DISABLED
#define NV_AONR2MC_BWHW  NV_MC_DISABLED
#define NV_AONR2MC_BWVW  NV_MC_DISABLED
#define NV_AONR2MC_BWSW  NV_MC_DISABLED
#define NV_AONR2MC_BWXY  NV_MC_DISABLED
#define NV_AONR2MC_BWPK  NV_MC_DISABLED
#define NV_AONR2MC_BW_ALLOWSW  NV_MC_DISABLED
#define NV_AONR2MC_CWHW  NV_MC_DISABLED
#define NV_AONR2MC_CWSW  NV_MC_DISABLED
#define NV_AONR2MC_SR_HP  NV_MC_DISABLED
#define NV_AONR2MC_SW_HP  NV_MC_DISABLED
#define NV_AONR2MC_BR_HP  NV_MC_DISABLED
#define NV_AONR2MC_BW_HP  NV_MC_DISABLED
#define NV_AONR2MC_CW_HP  NV_MC_DISABLED
#define NV_AONW2MC_BRHW  NV_MC_DISABLED
#define NV_AONW2MC_BRVW  NV_MC_DISABLED
#define NV_AONW2MC_BRSW  NV_MC_DISABLED
#define NV_AONW2MC_BRYUV  NV_MC_DISABLED
#define NV_AONW2MC_BRYUVAVG  NV_MC_DISABLED
#define NV_AONW2MC_BR_ALLOWSR  NV_MC_DISABLED
#define NV_AONW2MC_BR_OFFYUV  NV_MC_DISABLED
#define NV_AONW2MC_BWHW  NV_MC_DISABLED
#define NV_AONW2MC_BWVW  NV_MC_DISABLED
#define NV_AONW2MC_BWSW  NV_MC_DISABLED
#define NV_AONW2MC_BWXY  NV_MC_DISABLED
#define NV_AONW2MC_BWPK  NV_MC_DISABLED
#define NV_AONW2MC_BW_ALLOWSW  NV_MC_DISABLED
#define NV_AONW2MC_CWHW  NV_MC_DISABLED
#define NV_AONW2MC_CWSW  NV_MC_DISABLED
#define NV_AONW2MC_SR_HP  NV_MC_DISABLED
#define NV_AONW2MC_SW_HP  NV_MC_DISABLED
#define NV_AONW2MC_BR_HP  NV_MC_DISABLED
#define NV_AONW2MC_BW_HP  NV_MC_DISABLED
#define NV_AONW2MC_CW_HP  NV_MC_DISABLED
#define NV_AONDMAR2MC_BRHW  NV_MC_DISABLED
#define NV_AONDMAR2MC_BRVW  NV_MC_DISABLED
#define NV_AONDMAR2MC_BRSW  NV_MC_DISABLED
#define NV_AONDMAR2MC_BRYUV  NV_MC_DISABLED
#define NV_AONDMAR2MC_BRYUVAVG  NV_MC_DISABLED
#define NV_AONDMAR2MC_BR_ALLOWSR  NV_MC_DISABLED
#define NV_AONDMAR2MC_BR_OFFYUV  NV_MC_DISABLED
#define NV_AONDMAR2MC_BWHW  NV_MC_DISABLED
#define NV_AONDMAR2MC_BWVW  NV_MC_DISABLED
#define NV_AONDMAR2MC_BWSW  NV_MC_DISABLED
#define NV_AONDMAR2MC_BWXY  NV_MC_DISABLED
#define NV_AONDMAR2MC_BWPK  NV_MC_DISABLED
#define NV_AONDMAR2MC_BW_ALLOWSW  NV_MC_DISABLED
#define NV_AONDMAR2MC_CWHW  NV_MC_DISABLED
#define NV_AONDMAR2MC_CWSW  NV_MC_DISABLED
#define NV_AONDMAR2MC_SR_HP  NV_MC_DISABLED
#define NV_AONDMAR2MC_SW_HP  NV_MC_DISABLED
#define NV_AONDMAR2MC_BR_HP  NV_MC_DISABLED
#define NV_AONDMAR2MC_BW_HP  NV_MC_DISABLED
#define NV_AONDMAR2MC_CW_HP  NV_MC_DISABLED
#define NV_AONDMAW2MC_BRHW  NV_MC_DISABLED
#define NV_AONDMAW2MC_BRVW  NV_MC_DISABLED
#define NV_AONDMAW2MC_BRSW  NV_MC_DISABLED
#define NV_AONDMAW2MC_BRYUV  NV_MC_DISABLED
#define NV_AONDMAW2MC_BRYUVAVG  NV_MC_DISABLED
#define NV_AONDMAW2MC_BR_ALLOWSR  NV_MC_DISABLED
#define NV_AONDMAW2MC_BR_OFFYUV  NV_MC_DISABLED
#define NV_AONDMAW2MC_BWHW  NV_MC_DISABLED
#define NV_AONDMAW2MC_BWVW  NV_MC_DISABLED
#define NV_AONDMAW2MC_BWSW  NV_MC_DISABLED
#define NV_AONDMAW2MC_BWXY  NV_MC_DISABLED
#define NV_AONDMAW2MC_BWPK  NV_MC_DISABLED
#define NV_AONDMAW2MC_BW_ALLOWSW  NV_MC_DISABLED
#define NV_AONDMAW2MC_CWHW  NV_MC_DISABLED
#define NV_AONDMAW2MC_CWSW  NV_MC_DISABLED
#define NV_AONDMAW2MC_SR_HP  NV_MC_DISABLED
#define NV_AONDMAW2MC_SW_HP  NV_MC_DISABLED
#define NV_AONDMAW2MC_BR_HP  NV_MC_DISABLED
#define NV_AONDMAW2MC_BW_HP  NV_MC_DISABLED
#define NV_AONDMAW2MC_CW_HP  NV_MC_DISABLED
#define NV_SCER2MC_BRHW  NV_MC_DISABLED
#define NV_SCER2MC_BRVW  NV_MC_DISABLED
#define NV_SCER2MC_BRSW  NV_MC_DISABLED
#define NV_SCER2MC_BRYUV  NV_MC_DISABLED
#define NV_SCER2MC_BRYUVAVG  NV_MC_DISABLED
#define NV_SCER2MC_BR_ALLOWSR  NV_MC_DISABLED
#define NV_SCER2MC_BR_OFFYUV  NV_MC_DISABLED
#define NV_SCER2MC_BWHW  NV_MC_DISABLED
#define NV_SCER2MC_BWVW  NV_MC_DISABLED
#define NV_SCER2MC_BWSW  NV_MC_DISABLED
#define NV_SCER2MC_BWXY  NV_MC_DISABLED
#define NV_SCER2MC_BWPK  NV_MC_DISABLED
#define NV_SCER2MC_BW_ALLOWSW  NV_MC_DISABLED
#define NV_SCER2MC_CWHW  NV_MC_DISABLED
#define NV_SCER2MC_CWSW  NV_MC_DISABLED
#define NV_SCER2MC_SR_HP  NV_MC_DISABLED
#define NV_SCER2MC_SW_HP  NV_MC_DISABLED
#define NV_SCER2MC_BR_HP  NV_MC_DISABLED
#define NV_SCER2MC_BW_HP  NV_MC_DISABLED
#define NV_SCER2MC_CW_HP  NV_MC_DISABLED
#define NV_SCEW2MC_BRHW  NV_MC_DISABLED
#define NV_SCEW2MC_BRVW  NV_MC_DISABLED
#define NV_SCEW2MC_BRSW  NV_MC_DISABLED
#define NV_SCEW2MC_BRYUV  NV_MC_DISABLED
#define NV_SCEW2MC_BRYUVAVG  NV_MC_DISABLED
#define NV_SCEW2MC_BR_ALLOWSR  NV_MC_DISABLED
#define NV_SCEW2MC_BR_OFFYUV  NV_MC_DISABLED
#define NV_SCEW2MC_BWHW  NV_MC_DISABLED
#define NV_SCEW2MC_BWVW  NV_MC_DISABLED
#define NV_SCEW2MC_BWSW  NV_MC_DISABLED
#define NV_SCEW2MC_BWXY  NV_MC_DISABLED
#define NV_SCEW2MC_BWPK  NV_MC_DISABLED
#define NV_SCEW2MC_BW_ALLOWSW  NV_MC_DISABLED
#define NV_SCEW2MC_CWHW  NV_MC_DISABLED
#define NV_SCEW2MC_CWSW  NV_MC_DISABLED
#define NV_SCEW2MC_SR_HP  NV_MC_DISABLED
#define NV_SCEW2MC_SW_HP  NV_MC_DISABLED
#define NV_SCEW2MC_BR_HP  NV_MC_DISABLED
#define NV_SCEW2MC_BW_HP  NV_MC_DISABLED
#define NV_SCEW2MC_CW_HP  NV_MC_DISABLED
#define NV_SCEDMAR2MC_BRHW  NV_MC_DISABLED
#define NV_SCEDMAR2MC_BRVW  NV_MC_DISABLED
#define NV_SCEDMAR2MC_BRSW  NV_MC_DISABLED
#define NV_SCEDMAR2MC_BRYUV  NV_MC_DISABLED
#define NV_SCEDMAR2MC_BRYUVAVG  NV_MC_DISABLED
#define NV_SCEDMAR2MC_BR_ALLOWSR  NV_MC_DISABLED
#define NV_SCEDMAR2MC_BR_OFFYUV  NV_MC_DISABLED
#define NV_SCEDMAR2MC_BWHW  NV_MC_DISABLED
#define NV_SCEDMAR2MC_BWVW  NV_MC_DISABLED
#define NV_SCEDMAR2MC_BWSW  NV_MC_DISABLED
#define NV_SCEDMAR2MC_BWXY  NV_MC_DISABLED
#define NV_SCEDMAR2MC_BWPK  NV_MC_DISABLED
#define NV_SCEDMAR2MC_BW_ALLOWSW  NV_MC_DISABLED
#define NV_SCEDMAR2MC_CWHW  NV_MC_DISABLED
#define NV_SCEDMAR2MC_CWSW  NV_MC_DISABLED
#define NV_SCEDMAR2MC_SR_HP  NV_MC_DISABLED
#define NV_SCEDMAR2MC_SW_HP  NV_MC_DISABLED
#define NV_SCEDMAR2MC_BR_HP  NV_MC_DISABLED
#define NV_SCEDMAR2MC_BW_HP  NV_MC_DISABLED
#define NV_SCEDMAR2MC_CW_HP  NV_MC_DISABLED
#define NV_SCEDMAW2MC_BRHW  NV_MC_DISABLED
#define NV_SCEDMAW2MC_BRVW  NV_MC_DISABLED
#define NV_SCEDMAW2MC_BRSW  NV_MC_DISABLED
#define NV_SCEDMAW2MC_BRYUV  NV_MC_DISABLED
#define NV_SCEDMAW2MC_BRYUVAVG  NV_MC_DISABLED
#define NV_SCEDMAW2MC_BR_ALLOWSR  NV_MC_DISABLED
#define NV_SCEDMAW2MC_BR_OFFYUV  NV_MC_DISABLED
#define NV_SCEDMAW2MC_BWHW  NV_MC_DISABLED
#define NV_SCEDMAW2MC_BWVW  NV_MC_DISABLED
#define NV_SCEDMAW2MC_BWSW  NV_MC_DISABLED
#define NV_SCEDMAW2MC_BWXY  NV_MC_DISABLED
#define NV_SCEDMAW2MC_BWPK  NV_MC_DISABLED
#define NV_SCEDMAW2MC_BW_ALLOWSW  NV_MC_DISABLED
#define NV_SCEDMAW2MC_CWHW  NV_MC_DISABLED
#define NV_SCEDMAW2MC_CWSW  NV_MC_DISABLED
#define NV_SCEDMAW2MC_SR_HP  NV_MC_DISABLED
#define NV_SCEDMAW2MC_SW_HP  NV_MC_DISABLED
#define NV_SCEDMAW2MC_BR_HP  NV_MC_DISABLED
#define NV_SCEDMAW2MC_BW_HP  NV_MC_DISABLED
#define NV_SCEDMAW2MC_CW_HP  NV_MC_DISABLED
#define NV_APEDMAR2MC_BRHW  NV_MC_DISABLED
#define NV_APEDMAR2MC_BRVW  NV_MC_DISABLED
#define NV_APEDMAR2MC_BRSW  NV_MC_DISABLED
#define NV_APEDMAR2MC_BRYUV  NV_MC_DISABLED
#define NV_APEDMAR2MC_BRYUVAVG  NV_MC_DISABLED
#define NV_APEDMAR2MC_BR_ALLOWSR  NV_MC_DISABLED
#define NV_APEDMAR2MC_BR_OFFYUV  NV_MC_DISABLED
#define NV_APEDMAR2MC_BWHW  NV_MC_DISABLED
#define NV_APEDMAR2MC_BWVW  NV_MC_DISABLED
#define NV_APEDMAR2MC_BWSW  NV_MC_DISABLED
#define NV_APEDMAR2MC_BWXY  NV_MC_DISABLED
#define NV_APEDMAR2MC_BWPK  NV_MC_DISABLED
#define NV_APEDMAR2MC_BW_ALLOWSW  NV_MC_DISABLED
#define NV_APEDMAR2MC_CWHW  NV_MC_DISABLED
#define NV_APEDMAR2MC_CWSW  NV_MC_DISABLED
#define NV_APEDMAR2MC_SR_HP  NV_MC_DISABLED
#define NV_APEDMAR2MC_SW_HP  NV_MC_DISABLED
#define NV_APEDMAR2MC_BR_HP  NV_MC_DISABLED
#define NV_APEDMAR2MC_BW_HP  NV_MC_DISABLED
#define NV_APEDMAR2MC_CW_HP  NV_MC_DISABLED
#define NV_APEDMAW2MC_BRHW  NV_MC_DISABLED
#define NV_APEDMAW2MC_BRVW  NV_MC_DISABLED
#define NV_APEDMAW2MC_BRSW  NV_MC_DISABLED
#define NV_APEDMAW2MC_BRYUV  NV_MC_DISABLED
#define NV_APEDMAW2MC_BRYUVAVG  NV_MC_DISABLED
#define NV_APEDMAW2MC_BR_ALLOWSR  NV_MC_DISABLED
#define NV_APEDMAW2MC_BR_OFFYUV  NV_MC_DISABLED
#define NV_APEDMAW2MC_BWHW  NV_MC_DISABLED
#define NV_APEDMAW2MC_BWVW  NV_MC_DISABLED
#define NV_APEDMAW2MC_BWSW  NV_MC_DISABLED
#define NV_APEDMAW2MC_BWXY  NV_MC_DISABLED
#define NV_APEDMAW2MC_BWPK  NV_MC_DISABLED
#define NV_APEDMAW2MC_BW_ALLOWSW  NV_MC_DISABLED
#define NV_APEDMAW2MC_CWHW  NV_MC_DISABLED
#define NV_APEDMAW2MC_CWSW  NV_MC_DISABLED
#define NV_APEDMAW2MC_SR_HP  NV_MC_DISABLED
#define NV_APEDMAW2MC_SW_HP  NV_MC_DISABLED
#define NV_APEDMAW2MC_BR_HP  NV_MC_DISABLED
#define NV_APEDMAW2MC_BW_HP  NV_MC_DISABLED
#define NV_APEDMAW2MC_CW_HP  NV_MC_DISABLED
#define NV_NVDISPLAYR12MC_BRHW  NV_MC_DISABLED
#define NV_NVDISPLAYR12MC_BRVW  NV_MC_DISABLED
#define NV_NVDISPLAYR12MC_BRSW  NV_MC_DISABLED
#define NV_NVDISPLAYR12MC_BRYUV  NV_MC_DISABLED
#define NV_NVDISPLAYR12MC_BRYUVAVG  NV_MC_DISABLED
#define NV_NVDISPLAYR12MC_BR_ALLOWSR  NV_MC_DISABLED
#define NV_NVDISPLAYR12MC_BR_OFFYUV  NV_MC_DISABLED
#define NV_NVDISPLAYR12MC_BWHW  NV_MC_DISABLED
#define NV_NVDISPLAYR12MC_BWVW  NV_MC_DISABLED
#define NV_NVDISPLAYR12MC_BWSW  NV_MC_DISABLED
#define NV_NVDISPLAYR12MC_BWXY  NV_MC_DISABLED
#define NV_NVDISPLAYR12MC_BWPK  NV_MC_DISABLED
#define NV_NVDISPLAYR12MC_BW_ALLOWSW  NV_MC_DISABLED
#define NV_NVDISPLAYR12MC_CWHW  NV_MC_DISABLED
#define NV_NVDISPLAYR12MC_CWSW  NV_MC_DISABLED
#define NV_NVDISPLAYR12MC_SR_HP  NV_MC_DISABLED
#define NV_NVDISPLAYR12MC_SW_HP  NV_MC_DISABLED
#define NV_NVDISPLAYR12MC_BR_HP  NV_MC_DISABLED
#define NV_NVDISPLAYR12MC_BW_HP  NV_MC_DISABLED
#define NV_NVDISPLAYR12MC_CW_HP  NV_MC_DISABLED
#define NV_VICSRD12MC_BRHW  NV_MC_DISABLED
#define NV_VICSRD12MC_BRVW  NV_MC_DISABLED
#define NV_VICSRD12MC_BRSW  NV_MC_DISABLED
#define NV_VICSRD12MC_BRYUV  NV_MC_DISABLED
#define NV_VICSRD12MC_BRYUVAVG  NV_MC_DISABLED
#define NV_VICSRD12MC_BR_ALLOWSR  NV_MC_DISABLED
#define NV_VICSRD12MC_BR_OFFYUV  NV_MC_DISABLED
#define NV_VICSRD12MC_BWHW  NV_MC_DISABLED
#define NV_VICSRD12MC_BWVW  NV_MC_DISABLED
#define NV_VICSRD12MC_BWSW  NV_MC_DISABLED
#define NV_VICSRD12MC_BWXY  NV_MC_DISABLED
#define NV_VICSRD12MC_BWPK  NV_MC_DISABLED
#define NV_VICSRD12MC_BW_ALLOWSW  NV_MC_DISABLED
#define NV_VICSRD12MC_CWHW  NV_MC_DISABLED
#define NV_VICSRD12MC_CWSW  NV_MC_DISABLED
#define NV_VICSRD12MC_SR_HP  NV_MC_DISABLED
#define NV_VICSRD12MC_SW_HP  NV_MC_DISABLED
#define NV_VICSRD12MC_BR_HP  NV_MC_DISABLED
#define NV_VICSRD12MC_BW_HP  NV_MC_DISABLED
#define NV_VICSRD12MC_CW_HP  NV_MC_DISABLED
#define NV_NVDECSRD12MC_BRHW  NV_MC_DISABLED
#define NV_NVDECSRD12MC_BRVW  NV_MC_DISABLED
#define NV_NVDECSRD12MC_BRSW  NV_MC_DISABLED
#define NV_NVDECSRD12MC_BRYUV  NV_MC_DISABLED
#define NV_NVDECSRD12MC_BRYUVAVG  NV_MC_DISABLED
#define NV_NVDECSRD12MC_BR_ALLOWSR  NV_MC_DISABLED
#define NV_NVDECSRD12MC_BR_OFFYUV  NV_MC_DISABLED
#define NV_NVDECSRD12MC_BWHW  NV_MC_DISABLED
#define NV_NVDECSRD12MC_BWVW  NV_MC_DISABLED
#define NV_NVDECSRD12MC_BWSW  NV_MC_DISABLED
#define NV_NVDECSRD12MC_BWXY  NV_MC_DISABLED
#define NV_NVDECSRD12MC_BWPK  NV_MC_DISABLED
#define NV_NVDECSRD12MC_BW_ALLOWSW  NV_MC_DISABLED
#define NV_NVDECSRD12MC_CWHW  NV_MC_DISABLED
#define NV_NVDECSRD12MC_CWSW  NV_MC_DISABLED
#define NV_NVDECSRD12MC_SR_HP  NV_MC_DISABLED
#define NV_NVDECSRD12MC_SW_HP  NV_MC_DISABLED
#define NV_NVDECSRD12MC_BR_HP  NV_MC_DISABLED
#define NV_NVDECSRD12MC_BW_HP  NV_MC_DISABLED
#define NV_NVDECSRD12MC_CW_HP  NV_MC_DISABLED

#define LIST_PTSA_UNITS(_op_) \
  _op_(DIS) \
  _op_(VE) \
  _op_(ISP) \
  _op_(AUD) \
  _op_(APEDMAPC) \
  _op_(AONPC) \
  _op_(BPMPPC) \
  _op_(BPMPDMAPC) \
  _op_(SCEPC) \
  _op_(HDAPC) \
  _op_(EQOSPC) \
  _op_(RING2) \
  _op_(MLL_MPCORER) \
  _op_(SMMU_SMMU) \
  _op_(FTOP) \
  _op_(RING1) \
  _op_(DFD) \
  _op_(MSE2) \
  _op_(VICPC) \
  _op_(NVD) \
  _op_(AONDMAPC) \
  _op_(SCEDMAPC) \
  _op_(MSE) \
  _op_(HOST) \
  _op_(NVD3) \
  _op_(USBX) \
  _op_(USBD) \
  _op_(GK) \
  _op_(GK2) \
  _op_(APB) \
  _op_(SAX) \
  _op_(NIC) \
  _op_(JPG) \
  _op_(PCX) \
  _op_(SD) \
  _op_(VICPC3) \
  _op_(SDM) \
  _op_(SDM1) \
  _op_(UFSHCPC)

#define LIST_LA_SCALING_2_UNITS(_op_)

#define NV_HOST1XDMAR2MC_SR_REORDER_DEPTH    32
#define NV_NVENCSRD2MC_SR_REORDER_DEPTH     256
#define NV_MPCORER2MC_SR_REORDER_DEPTH       16
#define NV_TSECSRD2MC_SR_REORDER_DEPTH       20
#define NV_VICSRD2MC_SR_REORDER_DEPTH       225
#define NV_NVDECSRD2MC_SR_REORDER_DEPTH     143
#define NV_APER2MC_SR_REORDER_DEPTH           7
#define NV_NVJPGSRD2MC_SR_REORDER_DEPTH     160
#define NV_SESRD2MC_SR_REORDER_DEPTH        136
#define NV_TSECSRDB2MC_SR_REORDER_DEPTH      20
#define NV_VICSRD12MC_SR_REORDER_DEPTH      225
#define NV_NVDECSRD12MC_SR_REORDER_DEPTH    143
#define NV_AFIR2MC_SR_TRANSLATE 	NV_MC_ENABLED
#define NV_HDAR2MC_SR_TRANSLATE 	NV_MC_ENABLED
#define NV_HOST1XDMAR2MC_SR_TRANSLATE 	NV_MC_ENABLED
#define NV_NVENCSRD2MC_SR_TRANSLATE 	NV_MC_ENABLED
#define NV_SATAR2MC_SR_TRANSLATE 	NV_MC_ENABLED
#define NV_NVENCSWR2MC_SW_TRANSLATE 	NV_MC_ENABLED
#define NV_AFIW2MC_SW_TRANSLATE 	NV_MC_ENABLED
#define NV_HDAW2MC_SW_TRANSLATE 	NV_MC_ENABLED
#define NV_SATAW2MC_SW_TRANSLATE 	NV_MC_ENABLED
#define NV_ISPRA2MC_SR_TRANSLATE 	NV_MC_ENABLED
#define NV_ISPWA2MC_SW_TRANSLATE 	NV_MC_ENABLED
#define NV_ISPWB2MC_SW_TRANSLATE 	NV_MC_ENABLED
#define NV_XUSB_HOSTR2MC_SR_TRANSLATE 	NV_MC_ENABLED
#define NV_XUSB_HOSTW2MC_SW_TRANSLATE 	NV_MC_ENABLED
#define NV_XUSB_DEVR2MC_SR_TRANSLATE 	NV_MC_ENABLED
#define NV_XUSB_DEVW2MC_SW_TRANSLATE 	NV_MC_ENABLED
#define NV_TSECSRD2MC_SR_TRANSLATE 	NV_MC_ENABLED
#define NV_TSECSWR2MC_SW_TRANSLATE 	NV_MC_ENABLED
#define NV_GPUSRD2MC_SR_TRANSLATE 	NV_MC_ENABLED
#define NV_GPUSWR2MC_SW_TRANSLATE 	NV_MC_ENABLED
#define NV_SDMMCRA2MC_SR_TRANSLATE 	NV_MC_ENABLED
#define NV_SDMMCRAA2MC_SR_TRANSLATE 	NV_MC_ENABLED
#define NV_SDMMCR2MC_SR_TRANSLATE 	NV_MC_ENABLED
#define NV_SDMMCRAB2MC_SR_TRANSLATE 	NV_MC_ENABLED
#define NV_SDMMCWA2MC_SW_TRANSLATE 	NV_MC_ENABLED
#define NV_SDMMCWAA2MC_SW_TRANSLATE 	NV_MC_ENABLED
#define NV_SDMMCW2MC_SW_TRANSLATE 	NV_MC_ENABLED
#define NV_SDMMCWAB2MC_SW_TRANSLATE 	NV_MC_ENABLED
#define NV_VICSRD2MC_SR_TRANSLATE 	NV_MC_ENABLED
#define NV_VICSWR2MC_SW_TRANSLATE 	NV_MC_ENABLED
#define NV_VIW2MC_SW_TRANSLATE 	NV_MC_ENABLED
#define NV_NVDECSRD2MC_SR_TRANSLATE 	NV_MC_ENABLED
#define NV_NVDECSWR2MC_SW_TRANSLATE 	NV_MC_ENABLED
#define NV_APER2MC_SR_TRANSLATE 	NV_MC_ENABLED
#define NV_APEW2MC_SW_TRANSLATE 	NV_MC_ENABLED
#define NV_NVJPGSRD2MC_SR_TRANSLATE 	NV_MC_ENABLED
#define NV_NVJPGSWR2MC_SW_TRANSLATE 	NV_MC_ENABLED
#define NV_SESRD2MC_SR_TRANSLATE 	NV_MC_ENABLED
#define NV_SESWR2MC_SW_TRANSLATE 	NV_MC_ENABLED
#define NV_ETRR2MC_SR_TRANSLATE 	NV_MC_ENABLED
#define NV_ETRW2MC_SW_TRANSLATE 	NV_MC_ENABLED
#define NV_TSECSRDB2MC_SR_TRANSLATE 	NV_MC_ENABLED
#define NV_TSECSWRB2MC_SW_TRANSLATE 	NV_MC_ENABLED
#define NV_GPUSRD22MC_SR_TRANSLATE 	NV_MC_ENABLED
#define NV_GPUSWR22MC_SW_TRANSLATE 	NV_MC_ENABLED
#define NV_AXISR2MC_SR_TRANSLATE 	NV_MC_ENABLED
#define NV_AXISW2MC_SW_TRANSLATE 	NV_MC_ENABLED
#define NV_EQOSR2MC_SR_TRANSLATE 	NV_MC_ENABLED
#define NV_EQOSW2MC_SW_TRANSLATE 	NV_MC_ENABLED
#define NV_UFSHCR2MC_SR_TRANSLATE 	NV_MC_ENABLED
#define NV_UFSHCW2MC_SW_TRANSLATE 	NV_MC_ENABLED
#define NV_NVDISPLAYR2MC_SR_TRANSLATE 	NV_MC_ENABLED
#define NV_BPMPR2MC_SR_TRANSLATE 	NV_MC_ENABLED
#define NV_BPMPW2MC_SW_TRANSLATE 	NV_MC_ENABLED
#define NV_BPMPDMAR2MC_SR_TRANSLATE 	NV_MC_ENABLED
#define NV_BPMPDMAW2MC_SW_TRANSLATE 	NV_MC_ENABLED
#define NV_AONR2MC_SR_TRANSLATE 	NV_MC_ENABLED
#define NV_AONW2MC_SW_TRANSLATE 	NV_MC_ENABLED
#define NV_AONDMAR2MC_SR_TRANSLATE 	NV_MC_ENABLED
#define NV_AONDMAW2MC_SW_TRANSLATE 	NV_MC_ENABLED
#define NV_SCER2MC_SR_TRANSLATE 	NV_MC_ENABLED
#define NV_SCEW2MC_SW_TRANSLATE 	NV_MC_ENABLED
#define NV_SCEDMAR2MC_SR_TRANSLATE 	NV_MC_ENABLED
#define NV_SCEDMAW2MC_SW_TRANSLATE 	NV_MC_ENABLED
#define NV_APEDMAR2MC_SR_TRANSLATE 	NV_MC_ENABLED
#define NV_APEDMAW2MC_SW_TRANSLATE 	NV_MC_ENABLED
#define NV_NVDISPLAYR12MC_SR_TRANSLATE 	NV_MC_ENABLED
#define NV_VICSRD12MC_SR_TRANSLATE 	NV_MC_ENABLED
#define NV_NVDECSRD12MC_SR_TRANSLATE 	NV_MC_ENABLED
#define NV_ISPRA2MC_MCCIF_OBS 	NV_MC_ENABLED
#define NV_ISPWA2MC_MCCIF_OBS 	NV_MC_ENABLED
#define NV_ISPWB2MC_MCCIF_OBS 	NV_MC_ENABLED
#define NV_VICSRD2MC_MCCIF_OBS 	NV_MC_ENABLED
#define NV_APER2MC_MCCIF_OBS 	NV_MC_ENABLED
#define NV_APEW2MC_MCCIF_OBS 	NV_MC_ENABLED
#define LIST_MC_CLIENT_RST_REG(_op_) \
   _op_(AFIR,	CLK_RST_CONTROLLER_RST_DEVICES_PCIE_0,	CLK_RST_CONTROLLER_RST_DEVICES_PCIE_0_SWR_AFI_RST) \
   _op_(HDAR,	CLK_RST_CONTROLLER_RST_DEVICES_HDA_0,	CLK_RST_CONTROLLER_RST_DEVICES_HDA_0_SWR_HDA_RST) \
   _op_(HOST1XDMAR,	CLK_RST_CONTROLLER_RST_DEVICES_HOST1X_0,	CLK_RST_CONTROLLER_RST_DEVICES_HOST1X_0_SWR_HOST1X_RST) \
   _op_(NVENCSRD,	CLK_RST_CONTROLLER_RST_DEVICES_NVENC_0,	CLK_RST_CONTROLLER_RST_DEVICES_NVENC_0_SWR_NVENC_RST) \
   _op_(SATAR,	CLK_RST_CONTROLLER_RST_DEVICES_SATA_0,	CLK_RST_CONTROLLER_RST_DEVICES_SATA_0_SWR_SATA_RST) \
   _op_(MPCORER,	CLK_RST_CONTROLLER_RST_DEVICES_EMC_0,	CLK_RST_CONTROLLER_RST_DEVICES_EMC_0_SWR_MEM_RST) \
   _op_(NVENCSWR,	CLK_RST_CONTROLLER_RST_DEVICES_NVENC_0,	CLK_RST_CONTROLLER_RST_DEVICES_NVENC_0_SWR_NVENC_RST) \
   _op_(AFIW,	CLK_RST_CONTROLLER_RST_DEVICES_PCIE_0,	CLK_RST_CONTROLLER_RST_DEVICES_PCIE_0_SWR_AFI_RST) \
   _op_(HDAW,	CLK_RST_CONTROLLER_RST_DEVICES_HDA_0,	CLK_RST_CONTROLLER_RST_DEVICES_HDA_0_SWR_HDA_RST) \
   _op_(MPCOREW,	CLK_RST_CONTROLLER_RST_DEVICES_EMC_0,	CLK_RST_CONTROLLER_RST_DEVICES_EMC_0_SWR_MEM_RST) \
   _op_(SATAW,	CLK_RST_CONTROLLER_RST_DEVICES_SATA_0,	CLK_RST_CONTROLLER_RST_DEVICES_SATA_0_SWR_SATA_RST) \
   _op_(ISPRA,	CLK_RST_CONTROLLER_RST_DEVICES_ISP_0,	CLK_RST_CONTROLLER_RST_DEVICES_ISP_0_SWR_ISP_RST) \
   _op_(ISPWA,	CLK_RST_CONTROLLER_RST_DEVICES_ISP_0,	CLK_RST_CONTROLLER_RST_DEVICES_ISP_0_SWR_ISP_RST) \
   _op_(ISPWB,	CLK_RST_CONTROLLER_RST_DEVICES_ISP_0,	CLK_RST_CONTROLLER_RST_DEVICES_ISP_0_SWR_ISP_RST) \
   _op_(XUSB_HOSTR,	CLK_RST_CONTROLLER_RST_DEVICES_XUSB_0,	CLK_RST_CONTROLLER_RST_DEVICES_XUSB_0_SWR_XUSB_HOST_RST) \
   _op_(XUSB_HOSTW,	CLK_RST_CONTROLLER_RST_DEVICES_XUSB_0,	CLK_RST_CONTROLLER_RST_DEVICES_XUSB_0_SWR_XUSB_HOST_RST) \
   _op_(XUSB_DEVR,	CLK_RST_CONTROLLER_RST_DEVICES_XUSB_0,	CLK_RST_CONTROLLER_RST_DEVICES_XUSB_0_SWR_XUSB_DEV_RST) \
   _op_(XUSB_DEVW,	CLK_RST_CONTROLLER_RST_DEVICES_XUSB_0,	CLK_RST_CONTROLLER_RST_DEVICES_XUSB_0_SWR_XUSB_DEV_RST) \
   _op_(TSECSRD,	CLK_RST_CONTROLLER_RST_DEVICES_TSEC_0,	CLK_RST_CONTROLLER_RST_DEVICES_TSEC_0_SWR_TSEC_RST) \
   _op_(TSECSWR,	CLK_RST_CONTROLLER_RST_DEVICES_TSEC_0,	CLK_RST_CONTROLLER_RST_DEVICES_TSEC_0_SWR_TSEC_RST) \
   _op_(GPUSRD,	CLK_RST_CONTROLLER_RST_DEVICES_GPU_0,	CLK_RST_CONTROLLER_RST_DEVICES_GPU_0_SWR_GPU_RST) \
   _op_(GPUSWR,	CLK_RST_CONTROLLER_RST_DEVICES_GPU_0,	CLK_RST_CONTROLLER_RST_DEVICES_GPU_0_SWR_GPU_RST) \
   _op_(SDMMCRA,	CLK_RST_CONTROLLER_RST_DEVICES_SDMMC1_0,	CLK_RST_CONTROLLER_RST_DEVICES_SDMMC1_0_SWR_SDMMC1_RST) \
   _op_(SDMMCRAA,	CLK_RST_CONTROLLER_RST_DEVICES_SDMMC2_0,	CLK_RST_CONTROLLER_RST_DEVICES_SDMMC2_0_SWR_SDMMC2_RST) \
   _op_(SDMMCR,	CLK_RST_CONTROLLER_RST_DEVICES_SDMMC3_0,	CLK_RST_CONTROLLER_RST_DEVICES_SDMMC3_0_SWR_SDMMC3_RST) \
   _op_(SDMMCRAB,	CLK_RST_CONTROLLER_RST_DEVICES_SDMMC4_0,	CLK_RST_CONTROLLER_RST_DEVICES_SDMMC4_0_SWR_SDMMC4_RST) \
   _op_(SDMMCWA,	CLK_RST_CONTROLLER_RST_DEVICES_SDMMC1_0,	CLK_RST_CONTROLLER_RST_DEVICES_SDMMC1_0_SWR_SDMMC1_RST) \
   _op_(SDMMCWAA,	CLK_RST_CONTROLLER_RST_DEVICES_SDMMC2_0,	CLK_RST_CONTROLLER_RST_DEVICES_SDMMC2_0_SWR_SDMMC2_RST) \
   _op_(SDMMCW,	CLK_RST_CONTROLLER_RST_DEVICES_SDMMC3_0,	CLK_RST_CONTROLLER_RST_DEVICES_SDMMC3_0_SWR_SDMMC3_RST) \
   _op_(SDMMCWAB,	CLK_RST_CONTROLLER_RST_DEVICES_SDMMC4_0,	CLK_RST_CONTROLLER_RST_DEVICES_SDMMC4_0_SWR_SDMMC4_RST) \
   _op_(VICSRD,	CLK_RST_CONTROLLER_RST_DEVICES_VIC_0,	CLK_RST_CONTROLLER_RST_DEVICES_VIC_0_SWR_VIC_RST) \
   _op_(VICSWR,	CLK_RST_CONTROLLER_RST_DEVICES_VIC_0,	CLK_RST_CONTROLLER_RST_DEVICES_VIC_0_SWR_VIC_RST) \
   _op_(VIW,	CLK_RST_CONTROLLER_RST_DEVICES_VI_I2C_0,	CLK_RST_CONTROLLER_RST_DEVICES_VI_I2C_0_SWR_VI_I2C_RST) \
   _op_(NVDECSRD,	CLK_RST_CONTROLLER_RST_DEVICES_NVDEC_0,	CLK_RST_CONTROLLER_RST_DEVICES_NVDEC_0_SWR_NVDEC_RST) \
   _op_(NVDECSWR,	CLK_RST_CONTROLLER_RST_DEVICES_NVDEC_0,	CLK_RST_CONTROLLER_RST_DEVICES_NVDEC_0_SWR_NVDEC_RST) \
   _op_(APER,	CLK_RST_CONTROLLER_RST_DEVICES_APE_0,	CLK_RST_CONTROLLER_RST_DEVICES_APE_0_SWR_APE_RST) \
   _op_(APEW,	CLK_RST_CONTROLLER_RST_DEVICES_APE_0,	CLK_RST_CONTROLLER_RST_DEVICES_APE_0_SWR_APE_RST) \
   _op_(NVJPGSRD,	CLK_RST_CONTROLLER_RST_DEVICES_NVJPG_0,	CLK_RST_CONTROLLER_RST_DEVICES_NVJPG_0_SWR_NVJPG_RST) \
   _op_(NVJPGSWR,	CLK_RST_CONTROLLER_RST_DEVICES_NVJPG_0,	CLK_RST_CONTROLLER_RST_DEVICES_NVJPG_0_SWR_NVJPG_RST) \
   _op_(SESRD,	CLK_RST_CONTROLLER_RST_DEVICES_SE_0,	CLK_RST_CONTROLLER_RST_DEVICES_SE_0_SWR_SE_RST) \
   _op_(SESWR,	CLK_RST_CONTROLLER_RST_DEVICES_SE_0,	CLK_RST_CONTROLLER_RST_DEVICES_SE_0_SWR_SE_RST) \
   _op_(ETRR,	CLK_RST_CONTROLLER_RST_DEVICES_CSITE_0,	CLK_RST_CONTROLLER_RST_DEVICES_CSITE_0_SWR_CSITE_RST) \
   _op_(ETRW,	CLK_RST_CONTROLLER_RST_DEVICES_CSITE_0,	CLK_RST_CONTROLLER_RST_DEVICES_CSITE_0_SWR_CSITE_RST) \
   _op_(TSECSRDB,	CLK_RST_CONTROLLER_RST_DEVICES_TSECB_0,	CLK_RST_CONTROLLER_RST_DEVICES_TSECB_0_SWR_TSECB_RST) \
   _op_(TSECSWRB,	CLK_RST_CONTROLLER_RST_DEVICES_TSECB_0,	CLK_RST_CONTROLLER_RST_DEVICES_TSECB_0_SWR_TSECB_RST) \
   _op_(GPUSRD2,	CLK_RST_CONTROLLER_RST_DEVICES_GPU_0,	CLK_RST_CONTROLLER_RST_DEVICES_GPU_0_SWR_GPU_RST) \
   _op_(GPUSWR2,	CLK_RST_CONTROLLER_RST_DEVICES_GPU_0,	CLK_RST_CONTROLLER_RST_DEVICES_GPU_0_SWR_GPU_RST) \
   _op_(AXISR,	CLK_RST_CONTROLLER_RST_DEVICES_BPMP_APB_0,	CLK_RST_CONTROLLER_RST_DEVICES_BPMP_APB_0_SWR_BPMP_APB_RST) \
   _op_(AXISW,	CLK_RST_CONTROLLER_RST_DEVICES_BPMP_APB_0,	CLK_RST_CONTROLLER_RST_DEVICES_BPMP_APB_0_SWR_BPMP_APB_RST) \
   _op_(EQOSR,	CLK_RST_CONTROLLER_RST_DEVICES_EQOS_0,	CLK_RST_CONTROLLER_RST_DEVICES_EQOS_0_SWR_EQOS_RST) \
   _op_(EQOSW,	CLK_RST_CONTROLLER_RST_DEVICES_EQOS_0,	CLK_RST_CONTROLLER_RST_DEVICES_EQOS_0_SWR_EQOS_RST) \
   _op_(UFSHCR,	CLK_RST_CONTROLLER_RST_DEVICES_UFS_0,	CLK_RST_CONTROLLER_RST_DEVICES_UFS_0_SWR_UFSHC_AXI_M_RST) \
   _op_(UFSHCW,	CLK_RST_CONTROLLER_RST_DEVICES_UFS_0,	CLK_RST_CONTROLLER_RST_DEVICES_UFS_0_SWR_UFSHC_AXI_M_RST) \
   _op_(NVDISPLAYR,	CLK_RST_CONTROLLER_RST_DEVICES_NVDISPLAY0_0,	CLK_RST_CONTROLLER_RST_DEVICES_NVDISPLAY0_0_SWR_NVDISPLAY0_HEAD0_RST) \
   _op_(BPMPR,	CLK_RST_CONTROLLER_RST_DEVICES_BPMP_APB_0,	CLK_RST_CONTROLLER_RST_DEVICES_BPMP_APB_0_SWR_BPMP_APB_RST) \
   _op_(BPMPW,	CLK_RST_CONTROLLER_RST_DEVICES_BPMP_APB_0,	CLK_RST_CONTROLLER_RST_DEVICES_BPMP_APB_0_SWR_BPMP_APB_RST) \
   _op_(BPMPDMAR,	CLK_RST_CONTROLLER_RST_DEVICES_BPMP_CPU_NIC_0,	CLK_RST_CONTROLLER_RST_DEVICES_BPMP_CPU_NIC_0_SWR_BPMP_NIC_RST) \
   _op_(BPMPDMAW,	CLK_RST_CONTROLLER_RST_DEVICES_BPMP_CPU_NIC_0,	CLK_RST_CONTROLLER_RST_DEVICES_BPMP_CPU_NIC_0_SWR_BPMP_NIC_RST) \
   _op_(AONR,	CLK_RST_CONTROLLER_RST_DEVICES_AON_CPU_NIC_0,	CLK_RST_CONTROLLER_RST_DEVICES_AON_CPU_NIC_0_SWR_AON_NIC_RST) \
   _op_(AONW,	CLK_RST_CONTROLLER_RST_DEVICES_AON_CPU_NIC_0,	CLK_RST_CONTROLLER_RST_DEVICES_AON_CPU_NIC_0_SWR_AON_NIC_RST) \
   _op_(AONDMAR,	CLK_RST_CONTROLLER_RST_DEVICES_AON_CPU_NIC_0,	CLK_RST_CONTROLLER_RST_DEVICES_AON_CPU_NIC_0_SWR_AON_NIC_RST) \
   _op_(AONDMAW,	CLK_RST_CONTROLLER_RST_DEVICES_AON_CPU_NIC_0,	CLK_RST_CONTROLLER_RST_DEVICES_AON_CPU_NIC_0_SWR_AON_NIC_RST) \
   _op_(SCER,	CLK_RST_CONTROLLER_RST_DEVICES_SCE_APB_0,	CLK_RST_CONTROLLER_RST_DEVICES_SCE_APB_0_SWR_SCE_APB_RST) \
   _op_(SCEW,	CLK_RST_CONTROLLER_RST_DEVICES_SCE_APB_0,	CLK_RST_CONTROLLER_RST_DEVICES_SCE_APB_0_SWR_SCE_APB_RST) \
   _op_(SCEDMAR,	CLK_RST_CONTROLLER_RST_DEVICES_SCE_CPU_NIC_0,	CLK_RST_CONTROLLER_RST_DEVICES_SCE_CPU_NIC_0_SWR_SCE_NIC_RST) \
   _op_(SCEDMAW,	CLK_RST_CONTROLLER_RST_DEVICES_SCE_CPU_NIC_0,	CLK_RST_CONTROLLER_RST_DEVICES_SCE_CPU_NIC_0_SWR_SCE_NIC_RST) \
   _op_(APEDMAR,	CLK_RST_CONTROLLER_RST_DEVICES_APE_0,	CLK_RST_CONTROLLER_RST_DEVICES_APE_0_SWR_APE_RST) \
   _op_(APEDMAW,	CLK_RST_CONTROLLER_RST_DEVICES_APE_0,	CLK_RST_CONTROLLER_RST_DEVICES_APE_0_SWR_APE_RST) \
   _op_(NVDISPLAYR1,	CLK_RST_CONTROLLER_RST_DEVICES_NVDISPLAY0_0,	CLK_RST_CONTROLLER_RST_DEVICES_NVDISPLAY0_0_SWR_NVDISPLAY0_HEAD0_RST) \
   _op_(VICSRD1,	CLK_RST_CONTROLLER_RST_DEVICES_VIC_0,	CLK_RST_CONTROLLER_RST_DEVICES_VIC_0_SWR_VIC_RST) \
   _op_(NVDECSRD1,	CLK_RST_CONTROLLER_RST_DEVICES_NVDEC_0,	CLK_RST_CONTROLLER_RST_DEVICES_NVDEC_0_SWR_NVDEC_RST)
#define NV_MC_HAS_RING2 1
#define CIF_RING2_OBS_REG \
 reg CIF_RING2_OBS_REG incr1 \
 21:0 r RING2_RDY \
 22 r PC_DFD_ENABLED_REQ \
 23 r PC_MSE2_ENABLED_REQ \
 24 r PC_VICPC_ENABLED_REQ \
 25 r PC_NVD_ENABLED_REQ \
 26 r PC_AONDMAPC_ENABLED_REQ \
 27 r PC_SCEDMAPC_ENABLED_REQ \
 28 r PC_MSE_ENABLED_REQ \
 29 r PC_HOST_ENABLED_REQ \
 30 r PC_NVD3_ENABLED_REQ \
 31 r PC_USBX_ENABLED_REQ \
;

#define CIF_RING2_OBS_REG1 \
 reg CIF_RING2_OBS_REG1 incr1 \
 0 r PC_USBD_ENABLED_REQ \
 1 r PC_GK_ENABLED_REQ \
 2 r PC_APB_ENABLED_REQ \
 3 r PC_SAX_ENABLED_REQ \
 4 r PC_NIC_ENABLED_REQ \
 5 r PC_JPG_ENABLED_REQ \
 6 r PC_PCX_ENABLED_REQ \
 7 r PC_SD_ENABLED_REQ \
 8 r PC_VICPC3_ENABLED_REQ \
 9 r PC_SDM_ENABLED_REQ \
 10 r PC_SDM1_ENABLED_REQ \
 11 r PC_UFSHCPC_ENABLED_REQ \
;

#define NV_MC_HAS_RING1 1
#define CIF_RING1_OBS_REG \
 reg CIF_RING1_OBS_REG incr1 \
 11:0 r RING1_RDY \
 12 r RING2_REQ \
 13 r PC_DIS_ENABLED_REQ \
 14 r PC_VE_ENABLED_REQ \
 15 r PC_ISP_ENABLED_REQ \
 16 r PC_AUD_ENABLED_REQ \
 17 r PC_APEDMAPC_ENABLED_REQ \
 18 r PC_AONPC_ENABLED_REQ \
 19 r PC_BPMPPC_ENABLED_REQ \
 20 r PC_BPMPDMAPC_ENABLED_REQ \
 21 r PC_SCEPC_ENABLED_REQ \
 22 r PC_HDAPC_ENABLED_REQ \
 23 r PC_EQOSPC_ENABLED_REQ \
;

#define NV_MC_HAS_RING0 1
#define CIF_RING0_OBS_REG \
 reg CIF_RING0_OBS_REG incr1 \
 3:0 r RING0_RDY \
 4 r RING1_REQ \
 5 r PC_MLL_MPCORER_ENABLED_REQ \
 6 r PC_PTC_ENABLED_REQ \
 7 r PC_FTOP_ENABLED_REQ \
;

#define NV_MC_HAS_RING2 1
#define CIF_RING2_OBS_REG \
 reg CIF_RING2_OBS_REG incr1 \
 21:0 r RING2_RDY \
 22 r PC_DFD_ENABLED_REQ \
 23 r PC_MSE2_ENABLED_REQ \
 24 r PC_VICPC_ENABLED_REQ \
 25 r PC_NVD_ENABLED_REQ \
 26 r PC_AONDMAPC_ENABLED_REQ \
 27 r PC_SCEDMAPC_ENABLED_REQ \
 28 r PC_MSE_ENABLED_REQ \
 29 r PC_HOST_ENABLED_REQ \
 30 r PC_NVD3_ENABLED_REQ \
 31 r PC_USBX_ENABLED_REQ \
;

#define CIF_RING2_OBS_REG1 \
 reg CIF_RING2_OBS_REG1 incr1 \
 0 r PC_USBD_ENABLED_REQ \
 1 r PC_GK_ENABLED_REQ \
 2 r PC_APB_ENABLED_REQ \
 3 r PC_SAX_ENABLED_REQ \
 4 r PC_NIC_ENABLED_REQ \
 5 r PC_JPG_ENABLED_REQ \
 6 r PC_PCX_ENABLED_REQ \
 7 r PC_SD_ENABLED_REQ \
 8 r PC_VICPC3_ENABLED_REQ \
 9 r PC_SDM_ENABLED_REQ \
 10 r PC_SDM1_ENABLED_REQ \
 11 r PC_UFSHCPC_ENABLED_REQ \
;

#define NV_MC_HAS_RING1 1
#define CIF_RING1_OBS_REG \
 reg CIF_RING1_OBS_REG incr1 \
 11:0 r RING1_RDY \
 12 r RING2_REQ \
 13 r PC_DIS_ENABLED_REQ \
 14 r PC_VE_ENABLED_REQ \
 15 r PC_ISP_ENABLED_REQ \
 16 r PC_AUD_ENABLED_REQ \
 17 r PC_APEDMAPC_ENABLED_REQ \
 18 r PC_AONPC_ENABLED_REQ \
 19 r PC_BPMPPC_ENABLED_REQ \
 20 r PC_BPMPDMAPC_ENABLED_REQ \
 21 r PC_SCEPC_ENABLED_REQ \
 22 r PC_HDAPC_ENABLED_REQ \
 23 r PC_EQOSPC_ENABLED_REQ \
;

#define NV_MC_RING_LIST 2 1 0 2 1
#define PER_PC_RCLKEN_OBS_REG \
 reserve [2] incr1; \
 reg APB_RCLKEN_OBS_REG incr1 \
   0 r APB_R_CLKEN \
;\
\
 reserve [1] incr1; \
 reg DIS_RCLKEN_OBS_REG incr1 \
   0 r DIS_R_CLKEN \
;\
\
 reserve [2] incr1; \
 reg PCX_RCLKEN_OBS_REG incr1 \
   0 r PCX_R_CLKEN \
;\
 \
 reg FTOP_RCLKEN_OBS_REG incr1 \
   0 r FTOP_R_CLKEN \
;\
 \
 reg SAX_RCLKEN_OBS_REG incr1 \
   0 r SAX_R_CLKEN \
;\
\
 reserve [5] incr1; \
 reg VE_RCLKEN_OBS_REG incr1 \
   0 r VE_R_CLKEN \
;\
\
 reserve [1] incr1; \
 reg USBX_RCLKEN_OBS_REG incr1 \
   0 r USBX_R_CLKEN \
;\
\
 reserve [1] incr1; \
 reg MSE_RCLKEN_OBS_REG incr1 \
   0 r MSE_R_CLKEN \
;\
\
 reserve [2] incr1; \
 reg GK_RCLKEN_OBS_REG incr1 \
   0 r GK_R_CLKEN \
;\
 \
 reg SD_RCLKEN_OBS_REG incr1 \
   0 r SD_R_CLKEN \
;\
 \
 reg ISP_RCLKEN_OBS_REG incr1 \
   0 r ISP_R_CLKEN \
;\
\
 reserve [1] incr1; \
 reg AUD_RCLKEN_OBS_REG incr1 \
   0 r AUD_R_CLKEN \
;\
 \
 reg HOST_RCLKEN_OBS_REG incr1 \
   0 r HOST_R_CLKEN \
;\
 \
 reg USBD_RCLKEN_OBS_REG incr1 \
   0 r USBD_R_CLKEN \
;\
 \
 reg VICPC_RCLKEN_OBS_REG incr1 \
   0 r VICPC_R_CLKEN \
;\
\
 reserve [2] incr1; \
 reg NVD_RCLKEN_OBS_REG incr1 \
   0 r NVD_R_CLKEN \
;\
 \
 reg JPG_RCLKEN_OBS_REG incr1 \
   0 r JPG_R_CLKEN \
;\
 \
 reg GK2_RCLKEN_OBS_REG incr1 \
   0 r GK2_R_CLKEN \
;\
 \
 reg SDM_RCLKEN_OBS_REG incr1 \
   0 r SDM_R_CLKEN \
;\
 \
 reg HDAPC_RCLKEN_OBS_REG incr1 \
   0 r HDAPC_R_CLKEN \
;\
 \
 reg DFD_RCLKEN_OBS_REG incr1 \
   0 r DFD_R_CLKEN \
;\
 \
 reg SDM1_RCLKEN_OBS_REG incr1 \
   0 r SDM1_R_CLKEN \
;\
 \
 reg NIC_RCLKEN_OBS_REG incr1 \
   0 r NIC_R_CLKEN \
;\
 \
 reg UFSHCPC_RCLKEN_OBS_REG incr1 \
   0 r UFSHCPC_R_CLKEN \
;\
 \
 reg EQOSPC_RCLKEN_OBS_REG incr1 \
   0 r EQOSPC_R_CLKEN \
;\
 \
 reg BPMPPC_RCLKEN_OBS_REG incr1 \
   0 r BPMPPC_R_CLKEN \
;\
 \
 reg BPMPDMAPC_RCLKEN_OBS_REG incr1 \
   0 r BPMPDMAPC_R_CLKEN \
;\
 \
 reg AONPC_RCLKEN_OBS_REG incr1 \
   0 r AONPC_R_CLKEN \
;\
 \
 reg AONDMAPC_RCLKEN_OBS_REG incr1 \
   0 r AONDMAPC_R_CLKEN \
;\
 \
 reg SCEPC_RCLKEN_OBS_REG incr1 \
   0 r SCEPC_R_CLKEN \
;\
 \
 reg SCEDMAPC_RCLKEN_OBS_REG incr1 \
   0 r SCEDMAPC_R_CLKEN \
;\
 \
 reg APEDMAPC_RCLKEN_OBS_REG incr1 \
   0 r APEDMAPC_R_CLKEN \
;\
 \
 reg VICPC2_RCLKEN_OBS_REG incr1 \
   0 r VICPC2_R_CLKEN \
;\
 \
 reg VICPC3_RCLKEN_OBS_REG incr1 \
   0 r VICPC3_R_CLKEN \
;\
 \
 reg DIS2_RCLKEN_OBS_REG incr1 \
   0 r DIS2_R_CLKEN \
;\
 \
 reg NVD2_RCLKEN_OBS_REG incr1 \
   0 r NVD2_R_CLKEN \
;\
 \
 reg NVD3_RCLKEN_OBS_REG incr1 \
   0 r NVD3_R_CLKEN \
;\
 \
 reg MSE2_RCLKEN_OBS_REG incr1 \
   0 r MSE2_R_CLKEN \
;\


#define PTSA_OBS_RING1_REGS \
 reg RING1_DIS_PTSA_REG incr1\
  7:0  r R1_DIS_FRAC \
  13:8 r R1_DIS_INT \
  14   r R1_DIS_GRANT \
  15   r R1_DIS_HP \
; \
 reg RING1_VE_PTSA_REG incr1\
  7:0  r R1_VE_FRAC \
  13:8 r R1_VE_INT \
  14   r R1_VE_GRANT \
  15   r R1_VE_HP \
; \
 reg RING1_ISP_PTSA_REG incr1\
  7:0  r R1_ISP_FRAC \
  13:8 r R1_ISP_INT \
  14   r R1_ISP_GRANT \
  15   r R1_ISP_HP \
; \
 reg RING1_AUD_PTSA_REG incr1\
  7:0  r R1_AUD_FRAC \
  13:8 r R1_AUD_INT \
  14   r R1_AUD_GRANT \
  15   r R1_AUD_HP \
; \
 reg RING1_APEDMAPC_PTSA_REG incr1\
  7:0  r R1_APEDMAPC_FRAC \
  13:8 r R1_APEDMAPC_INT \
  14   r R1_APEDMAPC_GRANT \
  15   r R1_APEDMAPC_HP \
; \
 reg RING1_AONPC_PTSA_REG incr1\
  7:0  r R1_AONPC_FRAC \
  13:8 r R1_AONPC_INT \
  14   r R1_AONPC_GRANT \
  15   r R1_AONPC_HP \
; \
 reg RING1_BPMPPC_PTSA_REG incr1\
  7:0  r R1_BPMPPC_FRAC \
  13:8 r R1_BPMPPC_INT \
  14   r R1_BPMPPC_GRANT \
  15   r R1_BPMPPC_HP \
; \
 reg RING1_BPMPDMAPC_PTSA_REG incr1\
  7:0  r R1_BPMPDMAPC_FRAC \
  13:8 r R1_BPMPDMAPC_INT \
  14   r R1_BPMPDMAPC_GRANT \
  15   r R1_BPMPDMAPC_HP \
; \
 reg RING1_SCEPC_PTSA_REG incr1\
  7:0  r R1_SCEPC_FRAC \
  13:8 r R1_SCEPC_INT \
  14   r R1_SCEPC_GRANT \
  15   r R1_SCEPC_HP \
; \
 reg RING1_HDAPC_PTSA_REG incr1\
  7:0  r R1_HDAPC_FRAC \
  13:8 r R1_HDAPC_INT \
  14   r R1_HDAPC_GRANT \
  15   r R1_HDAPC_HP \
; \
 reg RING1_EQOSPC_PTSA_REG incr1\
  7:0  r R1_EQOSPC_FRAC \
  13:8 r R1_EQOSPC_INT \
  14   r R1_EQOSPC_GRANT \
  15   r R1_EQOSPC_HP \
; \
 reg RING1_RING2_PTSA_REG incr1\
  7:0  r  R1_RING2_FRAC \
  13:8 r  R1_RING2_INT \
  14   r  R1_RING2_GRANT \
  15   r  R1_RING2_HP \
;

#define PTSA_OBS_RING0_REGS \
 reg RING0_MLL_MPCORER_PTSA_REG incr1\
  7:0  r R0_MLL_MPCORER_FRAC \
  13:8 r R0_MLL_MPCORER_INT \
  14   r R0_MLL_MPCORER_GRANT \
  15   r R0_MLL_MPCORER_HP \
; \
 reg RING0_PTC_PTSA_REG incr1\
  7:0  r R0_PTC_FRAC \
  13:8 r R0_PTC_INT \
  14   r R0_PTC_GRANT \
  15   r R0_PTC_HP \
; \
 reg RING0_FTOP_PTSA_REG incr1\
  7:0  r R0_FTOP_FRAC \
  13:8 r R0_FTOP_INT \
  14   r R0_FTOP_GRANT \
  15   r R0_FTOP_HP \
; \
 reg RING0_RING1_PTSA_REG incr1\
  7:0  r  R0_RING1_FRAC \
  13:8 r  R0_RING1_INT \
  14   r  R0_RING1_GRANT \
  15   r  R0_RING1_HP \
;

#define PTSA_OBS_RING2_REGS \
 reg RING2_DFD_PTSA_REG incr1\
  7:0  r R2_DFD_FRAC \
  13:8 r R2_DFD_INT \
  14   r R2_DFD_GRANT \
  15   r R2_DFD_HP \
; \
 reg RING2_MSE2_PTSA_REG incr1\
  7:0  r R2_MSE2_FRAC \
  13:8 r R2_MSE2_INT \
  14   r R2_MSE2_GRANT \
  15   r R2_MSE2_HP \
; \
 reg RING2_VICPC_PTSA_REG incr1\
  7:0  r R2_VICPC_FRAC \
  13:8 r R2_VICPC_INT \
  14   r R2_VICPC_GRANT \
  15   r R2_VICPC_HP \
; \
 reg RING2_NVD_PTSA_REG incr1\
  7:0  r R2_NVD_FRAC \
  13:8 r R2_NVD_INT \
  14   r R2_NVD_GRANT \
  15   r R2_NVD_HP \
; \
 reg RING2_AONDMAPC_PTSA_REG incr1\
  7:0  r R2_AONDMAPC_FRAC \
  13:8 r R2_AONDMAPC_INT \
  14   r R2_AONDMAPC_GRANT \
  15   r R2_AONDMAPC_HP \
; \
 reg RING2_SCEDMAPC_PTSA_REG incr1\
  7:0  r R2_SCEDMAPC_FRAC \
  13:8 r R2_SCEDMAPC_INT \
  14   r R2_SCEDMAPC_GRANT \
  15   r R2_SCEDMAPC_HP \
; \
 reg RING2_MSE_PTSA_REG incr1\
  7:0  r R2_MSE_FRAC \
  13:8 r R2_MSE_INT \
  14   r R2_MSE_GRANT \
  15   r R2_MSE_HP \
; \
 reg RING2_HOST_PTSA_REG incr1\
  7:0  r R2_HOST_FRAC \
  13:8 r R2_HOST_INT \
  14   r R2_HOST_GRANT \
  15   r R2_HOST_HP \
; \
 reg RING2_NVD3_PTSA_REG incr1\
  7:0  r R2_NVD3_FRAC \
  13:8 r R2_NVD3_INT \
  14   r R2_NVD3_GRANT \
  15   r R2_NVD3_HP \
; \
 reg RING2_USBX_PTSA_REG incr1\
  7:0  r R2_USBX_FRAC \
  13:8 r R2_USBX_INT \
  14   r R2_USBX_GRANT \
  15   r R2_USBX_HP \
; \
 reg RING2_USBD_PTSA_REG incr1\
  7:0  r R2_USBD_FRAC \
  13:8 r R2_USBD_INT \
  14   r R2_USBD_GRANT \
  15   r R2_USBD_HP \
; \
 reg RING2_GK_PTSA_REG incr1\
  7:0  r R2_GK_FRAC \
  13:8 r R2_GK_INT \
  14   r R2_GK_GRANT \
  15   r R2_GK_HP \
; \
 reg RING2_APB_PTSA_REG incr1\
  7:0  r R2_APB_FRAC \
  13:8 r R2_APB_INT \
  14   r R2_APB_GRANT \
  15   r R2_APB_HP \
; \
 reg RING2_SAX_PTSA_REG incr1\
  7:0  r R2_SAX_FRAC \
  13:8 r R2_SAX_INT \
  14   r R2_SAX_GRANT \
  15   r R2_SAX_HP \
; \
 reg RING2_NIC_PTSA_REG incr1\
  7:0  r R2_NIC_FRAC \
  13:8 r R2_NIC_INT \
  14   r R2_NIC_GRANT \
  15   r R2_NIC_HP \
; \
 reg RING2_JPG_PTSA_REG incr1\
  7:0  r R2_JPG_FRAC \
  13:8 r R2_JPG_INT \
  14   r R2_JPG_GRANT \
  15   r R2_JPG_HP \
; \
 reg RING2_PCX_PTSA_REG incr1\
  7:0  r R2_PCX_FRAC \
  13:8 r R2_PCX_INT \
  14   r R2_PCX_GRANT \
  15   r R2_PCX_HP \
; \
 reg RING2_SD_PTSA_REG incr1\
  7:0  r R2_SD_FRAC \
  13:8 r R2_SD_INT \
  14   r R2_SD_GRANT \
  15   r R2_SD_HP \
; \
 reg RING2_VICPC3_PTSA_REG incr1\
  7:0  r R2_VICPC3_FRAC \
  13:8 r R2_VICPC3_INT \
  14   r R2_VICPC3_GRANT \
  15   r R2_VICPC3_HP \
; \
 reg RING2_SDM_PTSA_REG incr1\
  7:0  r R2_SDM_FRAC \
  13:8 r R2_SDM_INT \
  14   r R2_SDM_GRANT \
  15   r R2_SDM_HP \
; \
 reg RING2_SDM1_PTSA_REG incr1\
  7:0  r R2_SDM1_FRAC \
  13:8 r R2_SDM1_INT \
  14   r R2_SDM1_GRANT \
  15   r R2_SDM1_HP \
; \
 reg RING2_UFSHCPC_PTSA_REG incr1\
  7:0  r R2_UFSHCPC_FRAC \
  13:8 r R2_UFSHCPC_INT \
  14   r R2_UFSHCPC_GRANT \
  15   r R2_UFSHCPC_HP \
;

#define ET_LIST_TSA_REGS(_) \
    _(TSA_CONFIG_STATIC0_AFI) \
    _(TSA_CONFIG_STATIC0_AON) \
    _(TSA_CONFIG_STATIC0_AONDMA) \
    _(TSA_CONFIG_STATIC0_APE) \
    _(TSA_CONFIG_STATIC0_APEDMA) \
    _(TSA_CONFIG_STATIC0_AXIS) \
    _(TSA_CONFIG_STATIC0_BPMP) \
    _(TSA_CONFIG_STATIC0_BPMPDMA) \
    _(TSA_CONFIG_STATIC0_EQOS) \
    _(TSA_CONFIG_STATIC0_ETR) \
    _(TSA_CONFIG_STATIC0_GPU) \
    _(TSA_CONFIG_STATIC0_GPU2) \
    _(TSA_CONFIG_STATIC0_HDA) \
    _(TSA_CONFIG_STATIC0_HOST1X) \
    _(TSA_CONFIG_STATIC0_ISP2) \
    _(TSA_CONFIG_STATIC0_NVDEC) \
    _(TSA_CONFIG_STATIC0_NVDEC3) \
    _(TSA_CONFIG_STATIC0_NVDISPLAY) \
    _(TSA_CONFIG_STATIC0_NVENC) \
    _(TSA_CONFIG_STATIC0_NVENC2) \
    _(TSA_CONFIG_STATIC0_NVJPG) \
    _(TSA_CONFIG_STATIC0_SATA) \
    _(TSA_CONFIG_STATIC0_SCE) \
    _(TSA_CONFIG_STATIC0_SCEDMA) \
    _(TSA_CONFIG_STATIC0_SDMMC) \
    _(TSA_CONFIG_STATIC0_SDMMCA) \
    _(TSA_CONFIG_STATIC0_SDMMCAA) \
    _(TSA_CONFIG_STATIC0_SDMMCAB) \
    _(TSA_CONFIG_STATIC0_SE) \
    _(TSA_CONFIG_STATIC0_TSEC) \
    _(TSA_CONFIG_STATIC0_TSECB) \
    _(TSA_CONFIG_STATIC0_UFSHC) \
    _(TSA_CONFIG_STATIC0_VI2) \
    _(TSA_CONFIG_STATIC0_VIC) \
    _(TSA_CONFIG_STATIC0_VIC3) \
    _(TSA_CONFIG_STATIC0_XUSB_DEV) \
    _(TSA_CONFIG_STATIC0_XUSB_HOST) \
    _(TSA_CONFIG_STATIC0_CSR_AFIR) \
    _(TSA_CONFIG_STATIC0_CSR_HDAR) \
    _(TSA_CONFIG_STATIC0_CSR_HOST1XDMAR) \
    _(TSA_CONFIG_STATIC0_CSR_NVENCSRD) \
    _(TSA_CONFIG_STATIC0_CSR_SATAR) \
    _(TSA_CONFIG_STATIC0_CSW_NVENCSWR) \
    _(TSA_CONFIG_STATIC0_CSW_AFIW) \
    _(TSA_CONFIG_STATIC0_CSW_HDAW) \
    _(TSA_CONFIG_STATIC0_CSW_SATAW) \
    _(TSA_CONFIG_STATIC0_CSR_ISPRA) \
    _(TSA_CONFIG_STATIC0_CSW_ISPWA) \
    _(TSA_CONFIG_STATIC0_CSW_ISPWB) \
    _(TSA_CONFIG_STATIC0_CSR_XUSB_HOSTR) \
    _(TSA_CONFIG_STATIC0_CSW_XUSB_HOSTW) \
    _(TSA_CONFIG_STATIC0_CSR_XUSB_DEVR) \
    _(TSA_CONFIG_STATIC0_CSW_XUSB_DEVW) \
    _(TSA_CONFIG_STATIC0_CSR_TSECSRD) \
    _(TSA_CONFIG_STATIC0_CSW_TSECSWR) \
    _(TSA_CONFIG_STATIC0_CSR_GPUSRD) \
    _(TSA_CONFIG_STATIC0_CSW_GPUSWR) \
    _(TSA_CONFIG_STATIC0_CSR_SDMMCRA) \
    _(TSA_CONFIG_STATIC0_CSR_SDMMCRAA) \
    _(TSA_CONFIG_STATIC0_CSR_SDMMCR) \
    _(TSA_CONFIG_STATIC0_CSR_SDMMCRAB) \
    _(TSA_CONFIG_STATIC0_CSW_SDMMCWA) \
    _(TSA_CONFIG_STATIC0_CSW_SDMMCWAA) \
    _(TSA_CONFIG_STATIC0_CSW_SDMMCW) \
    _(TSA_CONFIG_STATIC0_CSW_SDMMCWAB) \
    _(TSA_CONFIG_STATIC0_CSR_VICSRD) \
    _(TSA_CONFIG_STATIC0_CSW_VICSWR) \
    _(TSA_CONFIG_STATIC0_CSW_VIW) \
    _(TSA_CONFIG_STATIC0_CSR_NVDECSRD) \
    _(TSA_CONFIG_STATIC0_CSW_NVDECSWR) \
    _(TSA_CONFIG_STATIC0_CSR_APER) \
    _(TSA_CONFIG_STATIC0_CSW_APEW) \
    _(TSA_CONFIG_STATIC0_CSR_NVJPGSRD) \
    _(TSA_CONFIG_STATIC0_CSW_NVJPGSWR) \
    _(TSA_CONFIG_STATIC0_CSR_SESRD) \
    _(TSA_CONFIG_STATIC0_CSW_SESWR) \
    _(TSA_CONFIG_STATIC0_CSR_ETRR) \
    _(TSA_CONFIG_STATIC0_CSW_ETRW) \
    _(TSA_CONFIG_STATIC0_CSR_TSECSRDB) \
    _(TSA_CONFIG_STATIC0_CSW_TSECSWRB) \
    _(TSA_CONFIG_STATIC0_CSR_GPUSRD2) \
    _(TSA_CONFIG_STATIC0_CSW_GPUSWR2) \
    _(TSA_CONFIG_STATIC0_CSR_AXISR) \
    _(TSA_CONFIG_STATIC0_CSW_AXISW) \
    _(TSA_CONFIG_STATIC0_CSR_EQOSR) \
    _(TSA_CONFIG_STATIC0_CSW_EQOSW) \
    _(TSA_CONFIG_STATIC0_CSR_UFSHCR) \
    _(TSA_CONFIG_STATIC0_CSW_UFSHCW) \
    _(TSA_CONFIG_STATIC0_CSR_NVDISPLAYR) \
    _(TSA_CONFIG_STATIC0_CSR_BPMPR) \
    _(TSA_CONFIG_STATIC0_CSW_BPMPW) \
    _(TSA_CONFIG_STATIC0_CSR_BPMPDMAR) \
    _(TSA_CONFIG_STATIC0_CSW_BPMPDMAW) \
    _(TSA_CONFIG_STATIC0_CSR_AONR) \
    _(TSA_CONFIG_STATIC0_CSW_AONW) \
    _(TSA_CONFIG_STATIC0_CSR_AONDMAR) \
    _(TSA_CONFIG_STATIC0_CSW_AONDMAW) \
    _(TSA_CONFIG_STATIC0_CSR_SCER) \
    _(TSA_CONFIG_STATIC0_CSW_SCEW) \
    _(TSA_CONFIG_STATIC0_CSR_SCEDMAR) \
    _(TSA_CONFIG_STATIC0_CSW_SCEDMAW) \
    _(TSA_CONFIG_STATIC0_CSR_APEDMAR) \
    _(TSA_CONFIG_STATIC0_CSW_APEDMAW) \

#define LIST_BCT_VARS_TSA(_) \
    _(TsaConfigStatic0Afi, "Specifies the value for TSA_CONFIG_STATIC0_AFI") \
    _(TsaConfigStatic0Aon, "Specifies the value for TSA_CONFIG_STATIC0_AON") \
    _(TsaConfigStatic0Aondma, "Specifies the value for TSA_CONFIG_STATIC0_AONDMA") \
    _(TsaConfigStatic0Ape, "Specifies the value for TSA_CONFIG_STATIC0_APE") \
    _(TsaConfigStatic0Apedma, "Specifies the value for TSA_CONFIG_STATIC0_APEDMA") \
    _(TsaConfigStatic0Axis, "Specifies the value for TSA_CONFIG_STATIC0_AXIS") \
    _(TsaConfigStatic0Bpmp, "Specifies the value for TSA_CONFIG_STATIC0_BPMP") \
    _(TsaConfigStatic0Bpmpdma, "Specifies the value for TSA_CONFIG_STATIC0_BPMPDMA") \
    _(TsaConfigStatic0Eqos, "Specifies the value for TSA_CONFIG_STATIC0_EQOS") \
    _(TsaConfigStatic0Etr, "Specifies the value for TSA_CONFIG_STATIC0_ETR") \
    _(TsaConfigStatic0Gpu, "Specifies the value for TSA_CONFIG_STATIC0_GPU") \
    _(TsaConfigStatic0Gpu2, "Specifies the value for TSA_CONFIG_STATIC0_GPU2") \
    _(TsaConfigStatic0Hda, "Specifies the value for TSA_CONFIG_STATIC0_HDA") \
    _(TsaConfigStatic0Host1x, "Specifies the value for TSA_CONFIG_STATIC0_HOST1X") \
    _(TsaConfigStatic0Isp2, "Specifies the value for TSA_CONFIG_STATIC0_ISP2") \
    _(TsaConfigStatic0Nvdec, "Specifies the value for TSA_CONFIG_STATIC0_NVDEC") \
    _(TsaConfigStatic0Nvdec3, "Specifies the value for TSA_CONFIG_STATIC0_NVDEC3") \
    _(TsaConfigStatic0Nvdisplay, "Specifies the value for TSA_CONFIG_STATIC0_NVDISPLAY") \
    _(TsaConfigStatic0Nvenc, "Specifies the value for TSA_CONFIG_STATIC0_NVENC") \
    _(TsaConfigStatic0Nvenc2, "Specifies the value for TSA_CONFIG_STATIC0_NVENC2") \
    _(TsaConfigStatic0Nvjpg, "Specifies the value for TSA_CONFIG_STATIC0_NVJPG") \
    _(TsaConfigStatic0Sata, "Specifies the value for TSA_CONFIG_STATIC0_SATA") \
    _(TsaConfigStatic0Sce, "Specifies the value for TSA_CONFIG_STATIC0_SCE") \
    _(TsaConfigStatic0Scedma, "Specifies the value for TSA_CONFIG_STATIC0_SCEDMA") \
    _(TsaConfigStatic0Sdmmc, "Specifies the value for TSA_CONFIG_STATIC0_SDMMC") \
    _(TsaConfigStatic0Sdmmca, "Specifies the value for TSA_CONFIG_STATIC0_SDMMCA") \
    _(TsaConfigStatic0Sdmmcaa, "Specifies the value for TSA_CONFIG_STATIC0_SDMMCAA") \
    _(TsaConfigStatic0Sdmmcab, "Specifies the value for TSA_CONFIG_STATIC0_SDMMCAB") \
    _(TsaConfigStatic0Se, "Specifies the value for TSA_CONFIG_STATIC0_SE") \
    _(TsaConfigStatic0Tsec, "Specifies the value for TSA_CONFIG_STATIC0_TSEC") \
    _(TsaConfigStatic0Tsecb, "Specifies the value for TSA_CONFIG_STATIC0_TSECB") \
    _(TsaConfigStatic0Ufshc, "Specifies the value for TSA_CONFIG_STATIC0_UFSHC") \
    _(TsaConfigStatic0Vi2, "Specifies the value for TSA_CONFIG_STATIC0_VI2") \
    _(TsaConfigStatic0Vic, "Specifies the value for TSA_CONFIG_STATIC0_VIC") \
    _(TsaConfigStatic0Vic3, "Specifies the value for TSA_CONFIG_STATIC0_VIC3") \
    _(TsaConfigStatic0Xusb_dev, "Specifies the value for TSA_CONFIG_STATIC0_XUSB_DEV") \
    _(TsaConfigStatic0Xusb_host, "Specifies the value for TSA_CONFIG_STATIC0_XUSB_HOST") \
    _(TsaConfigStatic0CsrAfir, "Specifies the value for TSA_CONFIG_STATIC0_CSR_AFIR") \
    _(TsaConfigStatic0CsrHdar, "Specifies the value for TSA_CONFIG_STATIC0_CSR_HDAR") \
    _(TsaConfigStatic0CsrHost1xdmar, "Specifies the value for TSA_CONFIG_STATIC0_CSR_HOST1XDMAR") \
    _(TsaConfigStatic0CsrNvencsrd, "Specifies the value for TSA_CONFIG_STATIC0_CSR_NVENCSRD") \
    _(TsaConfigStatic0CsrSatar, "Specifies the value for TSA_CONFIG_STATIC0_CSR_SATAR") \
    _(TsaConfigStatic0CswNvencswr, "Specifies the value for TSA_CONFIG_STATIC0_CSW_NVENCSWR") \
    _(TsaConfigStatic0CswAfiw, "Specifies the value for TSA_CONFIG_STATIC0_CSW_AFIW") \
    _(TsaConfigStatic0CswHdaw, "Specifies the value for TSA_CONFIG_STATIC0_CSW_HDAW") \
    _(TsaConfigStatic0CswSataw, "Specifies the value for TSA_CONFIG_STATIC0_CSW_SATAW") \
    _(TsaConfigStatic0CsrIspra, "Specifies the value for TSA_CONFIG_STATIC0_CSR_ISPRA") \
    _(TsaConfigStatic0CswIspwa, "Specifies the value for TSA_CONFIG_STATIC0_CSW_ISPWA") \
    _(TsaConfigStatic0CswIspwb, "Specifies the value for TSA_CONFIG_STATIC0_CSW_ISPWB") \
    _(TsaConfigStatic0CsrXusb_hostr, "Specifies the value for TSA_CONFIG_STATIC0_CSR_XUSB_HOSTR") \
    _(TsaConfigStatic0CswXusb_hostw, "Specifies the value for TSA_CONFIG_STATIC0_CSW_XUSB_HOSTW") \
    _(TsaConfigStatic0CsrXusb_devr, "Specifies the value for TSA_CONFIG_STATIC0_CSR_XUSB_DEVR") \
    _(TsaConfigStatic0CswXusb_devw, "Specifies the value for TSA_CONFIG_STATIC0_CSW_XUSB_DEVW") \
    _(TsaConfigStatic0CsrTsecsrd, "Specifies the value for TSA_CONFIG_STATIC0_CSR_TSECSRD") \
    _(TsaConfigStatic0CswTsecswr, "Specifies the value for TSA_CONFIG_STATIC0_CSW_TSECSWR") \
    _(TsaConfigStatic0CsrGpusrd, "Specifies the value for TSA_CONFIG_STATIC0_CSR_GPUSRD") \
    _(TsaConfigStatic0CswGpuswr, "Specifies the value for TSA_CONFIG_STATIC0_CSW_GPUSWR") \
    _(TsaConfigStatic0CsrSdmmcra, "Specifies the value for TSA_CONFIG_STATIC0_CSR_SDMMCRA") \
    _(TsaConfigStatic0CsrSdmmcraa, "Specifies the value for TSA_CONFIG_STATIC0_CSR_SDMMCRAA") \
    _(TsaConfigStatic0CsrSdmmcr, "Specifies the value for TSA_CONFIG_STATIC0_CSR_SDMMCR") \
    _(TsaConfigStatic0CsrSdmmcrab, "Specifies the value for TSA_CONFIG_STATIC0_CSR_SDMMCRAB") \
    _(TsaConfigStatic0CswSdmmcwa, "Specifies the value for TSA_CONFIG_STATIC0_CSW_SDMMCWA") \
    _(TsaConfigStatic0CswSdmmcwaa, "Specifies the value for TSA_CONFIG_STATIC0_CSW_SDMMCWAA") \
    _(TsaConfigStatic0CswSdmmcw, "Specifies the value for TSA_CONFIG_STATIC0_CSW_SDMMCW") \
    _(TsaConfigStatic0CswSdmmcwab, "Specifies the value for TSA_CONFIG_STATIC0_CSW_SDMMCWAB") \
    _(TsaConfigStatic0CsrVicsrd, "Specifies the value for TSA_CONFIG_STATIC0_CSR_VICSRD") \
    _(TsaConfigStatic0CswVicswr, "Specifies the value for TSA_CONFIG_STATIC0_CSW_VICSWR") \
    _(TsaConfigStatic0CswViw, "Specifies the value for TSA_CONFIG_STATIC0_CSW_VIW") \
    _(TsaConfigStatic0CsrNvdecsrd, "Specifies the value for TSA_CONFIG_STATIC0_CSR_NVDECSRD") \
    _(TsaConfigStatic0CswNvdecswr, "Specifies the value for TSA_CONFIG_STATIC0_CSW_NVDECSWR") \
    _(TsaConfigStatic0CsrAper, "Specifies the value for TSA_CONFIG_STATIC0_CSR_APER") \
    _(TsaConfigStatic0CswApew, "Specifies the value for TSA_CONFIG_STATIC0_CSW_APEW") \
    _(TsaConfigStatic0CsrNvjpgsrd, "Specifies the value for TSA_CONFIG_STATIC0_CSR_NVJPGSRD") \
    _(TsaConfigStatic0CswNvjpgswr, "Specifies the value for TSA_CONFIG_STATIC0_CSW_NVJPGSWR") \
    _(TsaConfigStatic0CsrSesrd, "Specifies the value for TSA_CONFIG_STATIC0_CSR_SESRD") \
    _(TsaConfigStatic0CswSeswr, "Specifies the value for TSA_CONFIG_STATIC0_CSW_SESWR") \
    _(TsaConfigStatic0CsrEtrr, "Specifies the value for TSA_CONFIG_STATIC0_CSR_ETRR") \
    _(TsaConfigStatic0CswEtrw, "Specifies the value for TSA_CONFIG_STATIC0_CSW_ETRW") \
    _(TsaConfigStatic0CsrTsecsrdb, "Specifies the value for TSA_CONFIG_STATIC0_CSR_TSECSRDB") \
    _(TsaConfigStatic0CswTsecswrb, "Specifies the value for TSA_CONFIG_STATIC0_CSW_TSECSWRB") \
    _(TsaConfigStatic0CsrGpusrd2, "Specifies the value for TSA_CONFIG_STATIC0_CSR_GPUSRD2") \
    _(TsaConfigStatic0CswGpuswr2, "Specifies the value for TSA_CONFIG_STATIC0_CSW_GPUSWR2") \
    _(TsaConfigStatic0CsrAxisr, "Specifies the value for TSA_CONFIG_STATIC0_CSR_AXISR") \
    _(TsaConfigStatic0CswAxisw, "Specifies the value for TSA_CONFIG_STATIC0_CSW_AXISW") \
    _(TsaConfigStatic0CsrEqosr, "Specifies the value for TSA_CONFIG_STATIC0_CSR_EQOSR") \
    _(TsaConfigStatic0CswEqosw, "Specifies the value for TSA_CONFIG_STATIC0_CSW_EQOSW") \
    _(TsaConfigStatic0CsrUfshcr, "Specifies the value for TSA_CONFIG_STATIC0_CSR_UFSHCR") \
    _(TsaConfigStatic0CswUfshcw, "Specifies the value for TSA_CONFIG_STATIC0_CSW_UFSHCW") \
    _(TsaConfigStatic0CsrNvdisplayr, "Specifies the value for TSA_CONFIG_STATIC0_CSR_NVDISPLAYR") \
    _(TsaConfigStatic0CsrBpmpr, "Specifies the value for TSA_CONFIG_STATIC0_CSR_BPMPR") \
    _(TsaConfigStatic0CswBpmpw, "Specifies the value for TSA_CONFIG_STATIC0_CSW_BPMPW") \
    _(TsaConfigStatic0CsrBpmpdmar, "Specifies the value for TSA_CONFIG_STATIC0_CSR_BPMPDMAR") \
    _(TsaConfigStatic0CswBpmpdmaw, "Specifies the value for TSA_CONFIG_STATIC0_CSW_BPMPDMAW") \
    _(TsaConfigStatic0CsrAonr, "Specifies the value for TSA_CONFIG_STATIC0_CSR_AONR") \
    _(TsaConfigStatic0CswAonw, "Specifies the value for TSA_CONFIG_STATIC0_CSW_AONW") \
    _(TsaConfigStatic0CsrAondmar, "Specifies the value for TSA_CONFIG_STATIC0_CSR_AONDMAR") \
    _(TsaConfigStatic0CswAondmaw, "Specifies the value for TSA_CONFIG_STATIC0_CSW_AONDMAW") \
    _(TsaConfigStatic0CsrScer, "Specifies the value for TSA_CONFIG_STATIC0_CSR_SCER") \
    _(TsaConfigStatic0CswScew, "Specifies the value for TSA_CONFIG_STATIC0_CSW_SCEW") \
    _(TsaConfigStatic0CsrScedmar, "Specifies the value for TSA_CONFIG_STATIC0_CSR_SCEDMAR") \
    _(TsaConfigStatic0CswScedmaw, "Specifies the value for TSA_CONFIG_STATIC0_CSW_SCEDMAW") \
    _(TsaConfigStatic0CsrApedmar, "Specifies the value for TSA_CONFIG_STATIC0_CSR_APEDMAR") \
    _(TsaConfigStatic0CswApedmaw, "Specifies the value for TSA_CONFIG_STATIC0_CSW_APEDMAW")

#define LIST_REG_TYP_TSA \
    reg.typ[TSA_CONFIG_STATIC0_AFI] = 0x80000000;\
    reg.typ[TSA_CONFIG_STATIC0_AON] = 0x80000000;\
    reg.typ[TSA_CONFIG_STATIC0_AONDMA] = 0x80000000;\
    reg.typ[TSA_CONFIG_STATIC0_APE] = 0x80000000;\
    reg.typ[TSA_CONFIG_STATIC0_APEDMA] = 0x80000000;\
    reg.typ[TSA_CONFIG_STATIC0_AXIS] = 0x80000000;\
    reg.typ[TSA_CONFIG_STATIC0_BPMP] = 0x80000000;\
    reg.typ[TSA_CONFIG_STATIC0_BPMPDMA] = 0x80000000;\
    reg.typ[TSA_CONFIG_STATIC0_EQOS] = 0x80000000;\
    reg.typ[TSA_CONFIG_STATIC0_ETR] = 0x80000000;\
    reg.typ[TSA_CONFIG_STATIC0_GPU] = 0x80000000;\
    reg.typ[TSA_CONFIG_STATIC0_GPU2] = 0x80000000;\
    reg.typ[TSA_CONFIG_STATIC0_HDA] = 0x80000000;\
    reg.typ[TSA_CONFIG_STATIC0_HOST1X] = 0x80000000;\
    reg.typ[TSA_CONFIG_STATIC0_ISP2] = 0x80000000;\
    reg.typ[TSA_CONFIG_STATIC0_NVDEC] = 0x80000000;\
    reg.typ[TSA_CONFIG_STATIC0_NVDEC3] = 0x80000000;\
    reg.typ[TSA_CONFIG_STATIC0_NVDISPLAY] = 0x80000000;\
    reg.typ[TSA_CONFIG_STATIC0_NVENC] = 0x80000000;\
    reg.typ[TSA_CONFIG_STATIC0_NVENC2] = 0x80000000;\
    reg.typ[TSA_CONFIG_STATIC0_NVJPG] = 0x80000000;\
    reg.typ[TSA_CONFIG_STATIC0_SATA] = 0x80000000;\
    reg.typ[TSA_CONFIG_STATIC0_SCE] = 0x80000000;\
    reg.typ[TSA_CONFIG_STATIC0_SCEDMA] = 0x80000000;\
    reg.typ[TSA_CONFIG_STATIC0_SDMMC] = 0x80000000;\
    reg.typ[TSA_CONFIG_STATIC0_SDMMCA] = 0x80000000;\
    reg.typ[TSA_CONFIG_STATIC0_SDMMCAA] = 0x80000000;\
    reg.typ[TSA_CONFIG_STATIC0_SDMMCAB] = 0x80000000;\
    reg.typ[TSA_CONFIG_STATIC0_SE] = 0x80000000;\
    reg.typ[TSA_CONFIG_STATIC0_TSEC] = 0x80000000;\
    reg.typ[TSA_CONFIG_STATIC0_TSECB] = 0x80000000;\
    reg.typ[TSA_CONFIG_STATIC0_UFSHC] = 0x80000000;\
    reg.typ[TSA_CONFIG_STATIC0_VI2] = 0x80000000;\
    reg.typ[TSA_CONFIG_STATIC0_VIC] = 0x80000000;\
    reg.typ[TSA_CONFIG_STATIC0_VIC3] = 0x80000000;\
    reg.typ[TSA_CONFIG_STATIC0_XUSB_DEV] = 0x80000000;\
    reg.typ[TSA_CONFIG_STATIC0_XUSB_HOST] = 0x80000000;\
    reg.typ[TSA_CONFIG_STATIC0_CSR_AFIR] = 0x00000100;\
    reg.typ[TSA_CONFIG_STATIC0_CSR_HDAR] = 0x00000100;\
    reg.typ[TSA_CONFIG_STATIC0_CSR_HOST1XDMAR] = 0x00000100;\
    reg.typ[TSA_CONFIG_STATIC0_CSR_NVENCSRD] = 0x00000100;\
    reg.typ[TSA_CONFIG_STATIC0_CSR_SATAR] = 0x00000100;\
    reg.typ[TSA_CONFIG_STATIC0_CSW_NVENCSWR] = 0x00000100;\
    reg.typ[TSA_CONFIG_STATIC0_CSW_AFIW] = 0x00000100;\
    reg.typ[TSA_CONFIG_STATIC0_CSW_HDAW] = 0x00000100;\
    reg.typ[TSA_CONFIG_STATIC0_CSW_SATAW] = 0x00000100;\
    reg.typ[TSA_CONFIG_STATIC0_CSR_ISPRA] = 0x00000100;\
    reg.typ[TSA_CONFIG_STATIC0_CSW_ISPWA] = 0x00000100;\
    reg.typ[TSA_CONFIG_STATIC0_CSW_ISPWB] = 0x00000100;\
    reg.typ[TSA_CONFIG_STATIC0_CSR_XUSB_HOSTR] = 0x00000100;\
    reg.typ[TSA_CONFIG_STATIC0_CSW_XUSB_HOSTW] = 0x00000100;\
    reg.typ[TSA_CONFIG_STATIC0_CSR_XUSB_DEVR] = 0x00000100;\
    reg.typ[TSA_CONFIG_STATIC0_CSW_XUSB_DEVW] = 0x00000100;\
    reg.typ[TSA_CONFIG_STATIC0_CSR_TSECSRD] = 0x00000100;\
    reg.typ[TSA_CONFIG_STATIC0_CSW_TSECSWR] = 0x00000100;\
    reg.typ[TSA_CONFIG_STATIC0_CSR_GPUSRD] = 0x00000100;\
    reg.typ[TSA_CONFIG_STATIC0_CSW_GPUSWR] = 0x00000100;\
    reg.typ[TSA_CONFIG_STATIC0_CSR_SDMMCRA] = 0x00000100;\
    reg.typ[TSA_CONFIG_STATIC0_CSR_SDMMCRAA] = 0x00000100;\
    reg.typ[TSA_CONFIG_STATIC0_CSR_SDMMCR] = 0x00000100;\
    reg.typ[TSA_CONFIG_STATIC0_CSR_SDMMCRAB] = 0x00000100;\
    reg.typ[TSA_CONFIG_STATIC0_CSW_SDMMCWA] = 0x00000100;\
    reg.typ[TSA_CONFIG_STATIC0_CSW_SDMMCWAA] = 0x00000100;\
    reg.typ[TSA_CONFIG_STATIC0_CSW_SDMMCW] = 0x00000100;\
    reg.typ[TSA_CONFIG_STATIC0_CSW_SDMMCWAB] = 0x00000100;\
    reg.typ[TSA_CONFIG_STATIC0_CSR_VICSRD] = 0x00000100;\
    reg.typ[TSA_CONFIG_STATIC0_CSW_VICSWR] = 0x00000100;\
    reg.typ[TSA_CONFIG_STATIC0_CSW_VIW] = 0x00000100;\
    reg.typ[TSA_CONFIG_STATIC0_CSR_NVDECSRD] = 0x00000100;\
    reg.typ[TSA_CONFIG_STATIC0_CSW_NVDECSWR] = 0x00000100;\
    reg.typ[TSA_CONFIG_STATIC0_CSR_APER] = 0x00000100;\
    reg.typ[TSA_CONFIG_STATIC0_CSW_APEW] = 0x00000100;\
    reg.typ[TSA_CONFIG_STATIC0_CSR_NVJPGSRD] = 0x00000100;\
    reg.typ[TSA_CONFIG_STATIC0_CSW_NVJPGSWR] = 0x00000100;\
    reg.typ[TSA_CONFIG_STATIC0_CSR_SESRD] = 0x00000100;\
    reg.typ[TSA_CONFIG_STATIC0_CSW_SESWR] = 0x00000100;\
    reg.typ[TSA_CONFIG_STATIC0_CSR_ETRR] = 0x00000100;\
    reg.typ[TSA_CONFIG_STATIC0_CSW_ETRW] = 0x00000100;\
    reg.typ[TSA_CONFIG_STATIC0_CSR_TSECSRDB] = 0x00000100;\
    reg.typ[TSA_CONFIG_STATIC0_CSW_TSECSWRB] = 0x00000100;\
    reg.typ[TSA_CONFIG_STATIC0_CSR_GPUSRD2] = 0x00000100;\
    reg.typ[TSA_CONFIG_STATIC0_CSW_GPUSWR2] = 0x00000100;\
    reg.typ[TSA_CONFIG_STATIC0_CSR_AXISR] = 0x00000100;\
    reg.typ[TSA_CONFIG_STATIC0_CSW_AXISW] = 0x00000100;\
    reg.typ[TSA_CONFIG_STATIC0_CSR_EQOSR] = 0x00000100;\
    reg.typ[TSA_CONFIG_STATIC0_CSW_EQOSW] = 0x00000100;\
    reg.typ[TSA_CONFIG_STATIC0_CSR_UFSHCR] = 0x00000100;\
    reg.typ[TSA_CONFIG_STATIC0_CSW_UFSHCW] = 0x00000100;\
    reg.typ[TSA_CONFIG_STATIC0_CSR_NVDISPLAYR] = 0x00000100;\
    reg.typ[TSA_CONFIG_STATIC0_CSR_BPMPR] = 0x00000100;\
    reg.typ[TSA_CONFIG_STATIC0_CSW_BPMPW] = 0x00000100;\
    reg.typ[TSA_CONFIG_STATIC0_CSR_BPMPDMAR] = 0x00000100;\
    reg.typ[TSA_CONFIG_STATIC0_CSW_BPMPDMAW] = 0x00000100;\
    reg.typ[TSA_CONFIG_STATIC0_CSR_AONR] = 0x00000100;\
    reg.typ[TSA_CONFIG_STATIC0_CSW_AONW] = 0x00000100;\
    reg.typ[TSA_CONFIG_STATIC0_CSR_AONDMAR] = 0x00000100;\
    reg.typ[TSA_CONFIG_STATIC0_CSW_AONDMAW] = 0x00000100;\
    reg.typ[TSA_CONFIG_STATIC0_CSR_SCER] = 0x00000100;\
    reg.typ[TSA_CONFIG_STATIC0_CSW_SCEW] = 0x00000100;\
    reg.typ[TSA_CONFIG_STATIC0_CSR_SCEDMAR] = 0x00000100;\
    reg.typ[TSA_CONFIG_STATIC0_CSW_SCEDMAW] = 0x00000100;\
    reg.typ[TSA_CONFIG_STATIC0_CSR_APEDMAR] = 0x00000100;\
    reg.typ[TSA_CONFIG_STATIC0_CSW_APEDMAW] = 0x00000100;

#define LIST_INIT_PUSH_TSA \
    init.push(TSA_CONFIG_STATIC0_AFI);\
    init.push(TSA_CONFIG_STATIC0_AON);\
    init.push(TSA_CONFIG_STATIC0_AONDMA);\
    init.push(TSA_CONFIG_STATIC0_APE);\
    init.push(TSA_CONFIG_STATIC0_APEDMA);\
    init.push(TSA_CONFIG_STATIC0_AXIS);\
    init.push(TSA_CONFIG_STATIC0_BPMP);\
    init.push(TSA_CONFIG_STATIC0_BPMPDMA);\
    init.push(TSA_CONFIG_STATIC0_EQOS);\
    init.push(TSA_CONFIG_STATIC0_ETR);\
    init.push(TSA_CONFIG_STATIC0_GPU);\
    init.push(TSA_CONFIG_STATIC0_GPU2);\
    init.push(TSA_CONFIG_STATIC0_HDA);\
    init.push(TSA_CONFIG_STATIC0_HOST1X);\
    init.push(TSA_CONFIG_STATIC0_ISP2);\
    init.push(TSA_CONFIG_STATIC0_NVDEC);\
    init.push(TSA_CONFIG_STATIC0_NVDEC3);\
    init.push(TSA_CONFIG_STATIC0_NVDISPLAY);\
    init.push(TSA_CONFIG_STATIC0_NVENC);\
    init.push(TSA_CONFIG_STATIC0_NVENC2);\
    init.push(TSA_CONFIG_STATIC0_NVJPG);\
    init.push(TSA_CONFIG_STATIC0_SATA);\
    init.push(TSA_CONFIG_STATIC0_SCE);\
    init.push(TSA_CONFIG_STATIC0_SCEDMA);\
    init.push(TSA_CONFIG_STATIC0_SDMMC);\
    init.push(TSA_CONFIG_STATIC0_SDMMCA);\
    init.push(TSA_CONFIG_STATIC0_SDMMCAA);\
    init.push(TSA_CONFIG_STATIC0_SDMMCAB);\
    init.push(TSA_CONFIG_STATIC0_SE);\
    init.push(TSA_CONFIG_STATIC0_TSEC);\
    init.push(TSA_CONFIG_STATIC0_TSECB);\
    init.push(TSA_CONFIG_STATIC0_UFSHC);\
    init.push(TSA_CONFIG_STATIC0_VI2);\
    init.push(TSA_CONFIG_STATIC0_VIC);\
    init.push(TSA_CONFIG_STATIC0_VIC3);\
    init.push(TSA_CONFIG_STATIC0_XUSB_DEV);\
    init.push(TSA_CONFIG_STATIC0_XUSB_HOST);\
    init.push(TSA_CONFIG_STATIC0_CSR_AFIR);\
    init.push(TSA_CONFIG_STATIC0_CSR_HDAR);\
    init.push(TSA_CONFIG_STATIC0_CSR_HOST1XDMAR);\
    init.push(TSA_CONFIG_STATIC0_CSR_NVENCSRD);\
    init.push(TSA_CONFIG_STATIC0_CSR_SATAR);\
    init.push(TSA_CONFIG_STATIC0_CSW_NVENCSWR);\
    init.push(TSA_CONFIG_STATIC0_CSW_AFIW);\
    init.push(TSA_CONFIG_STATIC0_CSW_HDAW);\
    init.push(TSA_CONFIG_STATIC0_CSW_SATAW);\
    init.push(TSA_CONFIG_STATIC0_CSR_ISPRA);\
    init.push(TSA_CONFIG_STATIC0_CSW_ISPWA);\
    init.push(TSA_CONFIG_STATIC0_CSW_ISPWB);\
    init.push(TSA_CONFIG_STATIC0_CSR_XUSB_HOSTR);\
    init.push(TSA_CONFIG_STATIC0_CSW_XUSB_HOSTW);\
    init.push(TSA_CONFIG_STATIC0_CSR_XUSB_DEVR);\
    init.push(TSA_CONFIG_STATIC0_CSW_XUSB_DEVW);\
    init.push(TSA_CONFIG_STATIC0_CSR_TSECSRD);\
    init.push(TSA_CONFIG_STATIC0_CSW_TSECSWR);\
    init.push(TSA_CONFIG_STATIC0_CSR_GPUSRD);\
    init.push(TSA_CONFIG_STATIC0_CSW_GPUSWR);\
    init.push(TSA_CONFIG_STATIC0_CSR_SDMMCRA);\
    init.push(TSA_CONFIG_STATIC0_CSR_SDMMCRAA);\
    init.push(TSA_CONFIG_STATIC0_CSR_SDMMCR);\
    init.push(TSA_CONFIG_STATIC0_CSR_SDMMCRAB);\
    init.push(TSA_CONFIG_STATIC0_CSW_SDMMCWA);\
    init.push(TSA_CONFIG_STATIC0_CSW_SDMMCWAA);\
    init.push(TSA_CONFIG_STATIC0_CSW_SDMMCW);\
    init.push(TSA_CONFIG_STATIC0_CSW_SDMMCWAB);\
    init.push(TSA_CONFIG_STATIC0_CSR_VICSRD);\
    init.push(TSA_CONFIG_STATIC0_CSW_VICSWR);\
    init.push(TSA_CONFIG_STATIC0_CSW_VIW);\
    init.push(TSA_CONFIG_STATIC0_CSR_NVDECSRD);\
    init.push(TSA_CONFIG_STATIC0_CSW_NVDECSWR);\
    init.push(TSA_CONFIG_STATIC0_CSR_APER);\
    init.push(TSA_CONFIG_STATIC0_CSW_APEW);\
    init.push(TSA_CONFIG_STATIC0_CSR_NVJPGSRD);\
    init.push(TSA_CONFIG_STATIC0_CSW_NVJPGSWR);\
    init.push(TSA_CONFIG_STATIC0_CSR_SESRD);\
    init.push(TSA_CONFIG_STATIC0_CSW_SESWR);\
    init.push(TSA_CONFIG_STATIC0_CSR_ETRR);\
    init.push(TSA_CONFIG_STATIC0_CSW_ETRW);\
    init.push(TSA_CONFIG_STATIC0_CSR_TSECSRDB);\
    init.push(TSA_CONFIG_STATIC0_CSW_TSECSWRB);\
    init.push(TSA_CONFIG_STATIC0_CSR_GPUSRD2);\
    init.push(TSA_CONFIG_STATIC0_CSW_GPUSWR2);\
    init.push(TSA_CONFIG_STATIC0_CSR_AXISR);\
    init.push(TSA_CONFIG_STATIC0_CSW_AXISW);\
    init.push(TSA_CONFIG_STATIC0_CSR_EQOSR);\
    init.push(TSA_CONFIG_STATIC0_CSW_EQOSW);\
    init.push(TSA_CONFIG_STATIC0_CSR_UFSHCR);\
    init.push(TSA_CONFIG_STATIC0_CSW_UFSHCW);\
    init.push(TSA_CONFIG_STATIC0_CSR_NVDISPLAYR);\
    init.push(TSA_CONFIG_STATIC0_CSR_BPMPR);\
    init.push(TSA_CONFIG_STATIC0_CSW_BPMPW);\
    init.push(TSA_CONFIG_STATIC0_CSR_BPMPDMAR);\
    init.push(TSA_CONFIG_STATIC0_CSW_BPMPDMAW);\
    init.push(TSA_CONFIG_STATIC0_CSR_AONR);\
    init.push(TSA_CONFIG_STATIC0_CSW_AONW);\
    init.push(TSA_CONFIG_STATIC0_CSR_AONDMAR);\
    init.push(TSA_CONFIG_STATIC0_CSW_AONDMAW);\
    init.push(TSA_CONFIG_STATIC0_CSR_SCER);\
    init.push(TSA_CONFIG_STATIC0_CSW_SCEW);\
    init.push(TSA_CONFIG_STATIC0_CSR_SCEDMAR);\
    init.push(TSA_CONFIG_STATIC0_CSW_SCEDMAW);\
    init.push(TSA_CONFIG_STATIC0_CSR_APEDMAR);\
    init.push(TSA_CONFIG_STATIC0_CSW_APEDMAW);

#define LIST_HW_REGW_TSA \
    HW_REGW(TsaBase, TSA, CONFIG_STATIC0_AFI, 0x80000000);\
    HW_REGW(TsaBase, TSA, CONFIG_STATIC0_AON, 0x80000000);\
    HW_REGW(TsaBase, TSA, CONFIG_STATIC0_AONDMA, 0x80000000);\
    HW_REGW(TsaBase, TSA, CONFIG_STATIC0_APE, 0x80000000);\
    HW_REGW(TsaBase, TSA, CONFIG_STATIC0_APEDMA, 0x80000000);\
    HW_REGW(TsaBase, TSA, CONFIG_STATIC0_AXIS, 0x80000000);\
    HW_REGW(TsaBase, TSA, CONFIG_STATIC0_BPMP, 0x80000000);\
    HW_REGW(TsaBase, TSA, CONFIG_STATIC0_BPMPDMA, 0x80000000);\
    HW_REGW(TsaBase, TSA, CONFIG_STATIC0_EQOS, 0x80000000);\
    HW_REGW(TsaBase, TSA, CONFIG_STATIC0_ETR, 0x80000000);\
    HW_REGW(TsaBase, TSA, CONFIG_STATIC0_GPU, 0x80000000);\
    HW_REGW(TsaBase, TSA, CONFIG_STATIC0_GPU2, 0x80000000);\
    HW_REGW(TsaBase, TSA, CONFIG_STATIC0_HDA, 0x80000000);\
    HW_REGW(TsaBase, TSA, CONFIG_STATIC0_HOST1X, 0x80000000);\
    HW_REGW(TsaBase, TSA, CONFIG_STATIC0_ISP2, 0x80000000);\
    HW_REGW(TsaBase, TSA, CONFIG_STATIC0_NVDEC, 0x80000000);\
    HW_REGW(TsaBase, TSA, CONFIG_STATIC0_NVDEC3, 0x80000000);\
    HW_REGW(TsaBase, TSA, CONFIG_STATIC0_NVDISPLAY, 0x80000000);\
    HW_REGW(TsaBase, TSA, CONFIG_STATIC0_NVENC, 0x80000000);\
    HW_REGW(TsaBase, TSA, CONFIG_STATIC0_NVENC2, 0x80000000);\
    HW_REGW(TsaBase, TSA, CONFIG_STATIC0_NVJPG, 0x80000000);\
    HW_REGW(TsaBase, TSA, CONFIG_STATIC0_SATA, 0x80000000);\
    HW_REGW(TsaBase, TSA, CONFIG_STATIC0_SCE, 0x80000000);\
    HW_REGW(TsaBase, TSA, CONFIG_STATIC0_SCEDMA, 0x80000000);\
    HW_REGW(TsaBase, TSA, CONFIG_STATIC0_SDMMC, 0x80000000);\
    HW_REGW(TsaBase, TSA, CONFIG_STATIC0_SDMMCA, 0x80000000);\
    HW_REGW(TsaBase, TSA, CONFIG_STATIC0_SDMMCAA, 0x80000000);\
    HW_REGW(TsaBase, TSA, CONFIG_STATIC0_SDMMCAB, 0x80000000);\
    HW_REGW(TsaBase, TSA, CONFIG_STATIC0_SE, 0x80000000);\
    HW_REGW(TsaBase, TSA, CONFIG_STATIC0_TSEC, 0x80000000);\
    HW_REGW(TsaBase, TSA, CONFIG_STATIC0_TSECB, 0x80000000);\
    HW_REGW(TsaBase, TSA, CONFIG_STATIC0_UFSHC, 0x80000000);\
    HW_REGW(TsaBase, TSA, CONFIG_STATIC0_VI2, 0x80000000);\
    HW_REGW(TsaBase, TSA, CONFIG_STATIC0_VIC, 0x80000000);\
    HW_REGW(TsaBase, TSA, CONFIG_STATIC0_VIC3, 0x80000000);\
    HW_REGW(TsaBase, TSA, CONFIG_STATIC0_XUSB_DEV, 0x80000000);\
    HW_REGW(TsaBase, TSA, CONFIG_STATIC0_XUSB_HOST, 0x80000000);\
    HW_REGW(TsaBase, TSA, CONFIG_STATIC0_CSR_AFIR, 0x00000100);\
    HW_REGW(TsaBase, TSA, CONFIG_STATIC0_CSR_HDAR, 0x00000100);\
    HW_REGW(TsaBase, TSA, CONFIG_STATIC0_CSR_HOST1XDMAR, 0x00000100);\
    HW_REGW(TsaBase, TSA, CONFIG_STATIC0_CSR_NVENCSRD, 0x00000100);\
    HW_REGW(TsaBase, TSA, CONFIG_STATIC0_CSR_SATAR, 0x00000100);\
    HW_REGW(TsaBase, TSA, CONFIG_STATIC0_CSW_NVENCSWR, 0x00000100);\
    HW_REGW(TsaBase, TSA, CONFIG_STATIC0_CSW_AFIW, 0x00000100);\
    HW_REGW(TsaBase, TSA, CONFIG_STATIC0_CSW_HDAW, 0x00000100);\
    HW_REGW(TsaBase, TSA, CONFIG_STATIC0_CSW_SATAW, 0x00000100);\
    HW_REGW(TsaBase, TSA, CONFIG_STATIC0_CSR_ISPRA, 0x00000100);\
    HW_REGW(TsaBase, TSA, CONFIG_STATIC0_CSW_ISPWA, 0x00000100);\
    HW_REGW(TsaBase, TSA, CONFIG_STATIC0_CSW_ISPWB, 0x00000100);\
    HW_REGW(TsaBase, TSA, CONFIG_STATIC0_CSR_XUSB_HOSTR, 0x00000100);\
    HW_REGW(TsaBase, TSA, CONFIG_STATIC0_CSW_XUSB_HOSTW, 0x00000100);\
    HW_REGW(TsaBase, TSA, CONFIG_STATIC0_CSR_XUSB_DEVR, 0x00000100);\
    HW_REGW(TsaBase, TSA, CONFIG_STATIC0_CSW_XUSB_DEVW, 0x00000100);\
    HW_REGW(TsaBase, TSA, CONFIG_STATIC0_CSR_TSECSRD, 0x00000100);\
    HW_REGW(TsaBase, TSA, CONFIG_STATIC0_CSW_TSECSWR, 0x00000100);\
    HW_REGW(TsaBase, TSA, CONFIG_STATIC0_CSR_GPUSRD, 0x00000100);\
    HW_REGW(TsaBase, TSA, CONFIG_STATIC0_CSW_GPUSWR, 0x00000100);\
    HW_REGW(TsaBase, TSA, CONFIG_STATIC0_CSR_SDMMCRA, 0x00000100);\
    HW_REGW(TsaBase, TSA, CONFIG_STATIC0_CSR_SDMMCRAA, 0x00000100);\
    HW_REGW(TsaBase, TSA, CONFIG_STATIC0_CSR_SDMMCR, 0x00000100);\
    HW_REGW(TsaBase, TSA, CONFIG_STATIC0_CSR_SDMMCRAB, 0x00000100);\
    HW_REGW(TsaBase, TSA, CONFIG_STATIC0_CSW_SDMMCWA, 0x00000100);\
    HW_REGW(TsaBase, TSA, CONFIG_STATIC0_CSW_SDMMCWAA, 0x00000100);\
    HW_REGW(TsaBase, TSA, CONFIG_STATIC0_CSW_SDMMCW, 0x00000100);\
    HW_REGW(TsaBase, TSA, CONFIG_STATIC0_CSW_SDMMCWAB, 0x00000100);\
    HW_REGW(TsaBase, TSA, CONFIG_STATIC0_CSR_VICSRD, 0x00000100);\
    HW_REGW(TsaBase, TSA, CONFIG_STATIC0_CSW_VICSWR, 0x00000100);\
    HW_REGW(TsaBase, TSA, CONFIG_STATIC0_CSW_VIW, 0x00000100);\
    HW_REGW(TsaBase, TSA, CONFIG_STATIC0_CSR_NVDECSRD, 0x00000100);\
    HW_REGW(TsaBase, TSA, CONFIG_STATIC0_CSW_NVDECSWR, 0x00000100);\
    HW_REGW(TsaBase, TSA, CONFIG_STATIC0_CSR_APER, 0x00000100);\
    HW_REGW(TsaBase, TSA, CONFIG_STATIC0_CSW_APEW, 0x00000100);\
    HW_REGW(TsaBase, TSA, CONFIG_STATIC0_CSR_NVJPGSRD, 0x00000100);\
    HW_REGW(TsaBase, TSA, CONFIG_STATIC0_CSW_NVJPGSWR, 0x00000100);\
    HW_REGW(TsaBase, TSA, CONFIG_STATIC0_CSR_SESRD, 0x00000100);\
    HW_REGW(TsaBase, TSA, CONFIG_STATIC0_CSW_SESWR, 0x00000100);\
    HW_REGW(TsaBase, TSA, CONFIG_STATIC0_CSR_ETRR, 0x00000100);\
    HW_REGW(TsaBase, TSA, CONFIG_STATIC0_CSW_ETRW, 0x00000100);\
    HW_REGW(TsaBase, TSA, CONFIG_STATIC0_CSR_TSECSRDB, 0x00000100);\
    HW_REGW(TsaBase, TSA, CONFIG_STATIC0_CSW_TSECSWRB, 0x00000100);\
    HW_REGW(TsaBase, TSA, CONFIG_STATIC0_CSR_GPUSRD2, 0x00000100);\
    HW_REGW(TsaBase, TSA, CONFIG_STATIC0_CSW_GPUSWR2, 0x00000100);\
    HW_REGW(TsaBase, TSA, CONFIG_STATIC0_CSR_AXISR, 0x00000100);\
    HW_REGW(TsaBase, TSA, CONFIG_STATIC0_CSW_AXISW, 0x00000100);\
    HW_REGW(TsaBase, TSA, CONFIG_STATIC0_CSR_EQOSR, 0x00000100);\
    HW_REGW(TsaBase, TSA, CONFIG_STATIC0_CSW_EQOSW, 0x00000100);\
    HW_REGW(TsaBase, TSA, CONFIG_STATIC0_CSR_UFSHCR, 0x00000100);\
    HW_REGW(TsaBase, TSA, CONFIG_STATIC0_CSW_UFSHCW, 0x00000100);\
    HW_REGW(TsaBase, TSA, CONFIG_STATIC0_CSR_NVDISPLAYR, 0x00000100);\
    HW_REGW(TsaBase, TSA, CONFIG_STATIC0_CSR_BPMPR, 0x00000100);\
    HW_REGW(TsaBase, TSA, CONFIG_STATIC0_CSW_BPMPW, 0x00000100);\
    HW_REGW(TsaBase, TSA, CONFIG_STATIC0_CSR_BPMPDMAR, 0x00000100);\
    HW_REGW(TsaBase, TSA, CONFIG_STATIC0_CSW_BPMPDMAW, 0x00000100);\
    HW_REGW(TsaBase, TSA, CONFIG_STATIC0_CSR_AONR, 0x00000100);\
    HW_REGW(TsaBase, TSA, CONFIG_STATIC0_CSW_AONW, 0x00000100);\
    HW_REGW(TsaBase, TSA, CONFIG_STATIC0_CSR_AONDMAR, 0x00000100);\
    HW_REGW(TsaBase, TSA, CONFIG_STATIC0_CSW_AONDMAW, 0x00000100);\
    HW_REGW(TsaBase, TSA, CONFIG_STATIC0_CSR_SCER, 0x00000100);\
    HW_REGW(TsaBase, TSA, CONFIG_STATIC0_CSW_SCEW, 0x00000100);\
    HW_REGW(TsaBase, TSA, CONFIG_STATIC0_CSR_SCEDMAR, 0x00000100);\
    HW_REGW(TsaBase, TSA, CONFIG_STATIC0_CSW_SCEDMAW, 0x00000100);\
    HW_REGW(TsaBase, TSA, CONFIG_STATIC0_CSR_APEDMAR, 0x00000100);\
    HW_REGW(TsaBase, TSA, CONFIG_STATIC0_CSW_APEDMAW, 0x00000100);

#define ET_LIST_MC_SID_REGS(_) \
    _(MC_SID_STREAMID_OVERRIDE_CONFIG_PTCR) \
    _(MC_SID_STREAMID_SECURITY_CONFIG_PTCR) \
    _(MC_SID_STREAMID_OVERRIDE_CONFIG_AFIR) \
    _(MC_SID_STREAMID_SECURITY_CONFIG_AFIR) \
    _(MC_SID_STREAMID_OVERRIDE_CONFIG_HDAR) \
    _(MC_SID_STREAMID_SECURITY_CONFIG_HDAR) \
    _(MC_SID_STREAMID_OVERRIDE_CONFIG_HOST1XDMAR) \
    _(MC_SID_STREAMID_SECURITY_CONFIG_HOST1XDMAR) \
    _(MC_SID_STREAMID_OVERRIDE_CONFIG_NVENCSRD) \
    _(MC_SID_STREAMID_SECURITY_CONFIG_NVENCSRD) \
    _(MC_SID_STREAMID_OVERRIDE_CONFIG_SATAR) \
    _(MC_SID_STREAMID_SECURITY_CONFIG_SATAR) \
    _(MC_SID_STREAMID_OVERRIDE_CONFIG_MPCORER) \
    _(MC_SID_STREAMID_SECURITY_CONFIG_MPCORER) \
    _(MC_SID_STREAMID_OVERRIDE_CONFIG_NVENCSWR) \
    _(MC_SID_STREAMID_SECURITY_CONFIG_NVENCSWR) \
    _(MC_SID_STREAMID_OVERRIDE_CONFIG_AFIW) \
    _(MC_SID_STREAMID_SECURITY_CONFIG_AFIW) \
    _(MC_SID_STREAMID_OVERRIDE_CONFIG_HDAW) \
    _(MC_SID_STREAMID_SECURITY_CONFIG_HDAW) \
    _(MC_SID_STREAMID_OVERRIDE_CONFIG_MPCOREW) \
    _(MC_SID_STREAMID_SECURITY_CONFIG_MPCOREW) \
    _(MC_SID_STREAMID_OVERRIDE_CONFIG_SATAW) \
    _(MC_SID_STREAMID_SECURITY_CONFIG_SATAW) \
    _(MC_SID_STREAMID_OVERRIDE_CONFIG_ISPRA) \
    _(MC_SID_STREAMID_SECURITY_CONFIG_ISPRA) \
    _(MC_SID_STREAMID_OVERRIDE_CONFIG_ISPWA) \
    _(MC_SID_STREAMID_SECURITY_CONFIG_ISPWA) \
    _(MC_SID_STREAMID_OVERRIDE_CONFIG_ISPWB) \
    _(MC_SID_STREAMID_SECURITY_CONFIG_ISPWB) \
    _(MC_SID_STREAMID_OVERRIDE_CONFIG_XUSB_HOSTR) \
    _(MC_SID_STREAMID_SECURITY_CONFIG_XUSB_HOSTR) \
    _(MC_SID_STREAMID_OVERRIDE_CONFIG_XUSB_HOSTW) \
    _(MC_SID_STREAMID_SECURITY_CONFIG_XUSB_HOSTW) \
    _(MC_SID_STREAMID_OVERRIDE_CONFIG_XUSB_DEVR) \
    _(MC_SID_STREAMID_SECURITY_CONFIG_XUSB_DEVR) \
    _(MC_SID_STREAMID_OVERRIDE_CONFIG_XUSB_DEVW) \
    _(MC_SID_STREAMID_SECURITY_CONFIG_XUSB_DEVW) \
    _(MC_SID_STREAMID_OVERRIDE_CONFIG_TSECSRD) \
    _(MC_SID_STREAMID_SECURITY_CONFIG_TSECSRD) \
    _(MC_SID_STREAMID_OVERRIDE_CONFIG_TSECSWR) \
    _(MC_SID_STREAMID_SECURITY_CONFIG_TSECSWR) \
    _(MC_SID_STREAMID_OVERRIDE_CONFIG_GPUSRD) \
    _(MC_SID_STREAMID_SECURITY_CONFIG_GPUSRD) \
    _(MC_SID_STREAMID_OVERRIDE_CONFIG_GPUSWR) \
    _(MC_SID_STREAMID_SECURITY_CONFIG_GPUSWR) \
    _(MC_SID_STREAMID_OVERRIDE_CONFIG_SDMMCRA) \
    _(MC_SID_STREAMID_SECURITY_CONFIG_SDMMCRA) \
    _(MC_SID_STREAMID_OVERRIDE_CONFIG_SDMMCRAA) \
    _(MC_SID_STREAMID_SECURITY_CONFIG_SDMMCRAA) \
    _(MC_SID_STREAMID_OVERRIDE_CONFIG_SDMMCR) \
    _(MC_SID_STREAMID_SECURITY_CONFIG_SDMMCR) \
    _(MC_SID_STREAMID_OVERRIDE_CONFIG_SDMMCRAB) \
    _(MC_SID_STREAMID_SECURITY_CONFIG_SDMMCRAB) \
    _(MC_SID_STREAMID_OVERRIDE_CONFIG_SDMMCWA) \
    _(MC_SID_STREAMID_SECURITY_CONFIG_SDMMCWA) \
    _(MC_SID_STREAMID_OVERRIDE_CONFIG_SDMMCWAA) \
    _(MC_SID_STREAMID_SECURITY_CONFIG_SDMMCWAA) \
    _(MC_SID_STREAMID_OVERRIDE_CONFIG_SDMMCW) \
    _(MC_SID_STREAMID_SECURITY_CONFIG_SDMMCW) \
    _(MC_SID_STREAMID_OVERRIDE_CONFIG_SDMMCWAB) \
    _(MC_SID_STREAMID_SECURITY_CONFIG_SDMMCWAB) \
    _(MC_SID_STREAMID_OVERRIDE_CONFIG_VICSRD) \
    _(MC_SID_STREAMID_SECURITY_CONFIG_VICSRD) \
    _(MC_SID_STREAMID_OVERRIDE_CONFIG_VICSWR) \
    _(MC_SID_STREAMID_SECURITY_CONFIG_VICSWR) \
    _(MC_SID_STREAMID_OVERRIDE_CONFIG_VIW) \
    _(MC_SID_STREAMID_SECURITY_CONFIG_VIW) \
    _(MC_SID_STREAMID_OVERRIDE_CONFIG_NVDECSRD) \
    _(MC_SID_STREAMID_SECURITY_CONFIG_NVDECSRD) \
    _(MC_SID_STREAMID_OVERRIDE_CONFIG_NVDECSWR) \
    _(MC_SID_STREAMID_SECURITY_CONFIG_NVDECSWR) \
    _(MC_SID_STREAMID_OVERRIDE_CONFIG_APER) \
    _(MC_SID_STREAMID_SECURITY_CONFIG_APER) \
    _(MC_SID_STREAMID_OVERRIDE_CONFIG_APEW) \
    _(MC_SID_STREAMID_SECURITY_CONFIG_APEW) \
    _(MC_SID_STREAMID_OVERRIDE_CONFIG_NVJPGSRD) \
    _(MC_SID_STREAMID_SECURITY_CONFIG_NVJPGSRD) \
    _(MC_SID_STREAMID_OVERRIDE_CONFIG_NVJPGSWR) \
    _(MC_SID_STREAMID_SECURITY_CONFIG_NVJPGSWR) \
    _(MC_SID_STREAMID_OVERRIDE_CONFIG_SESRD) \
    _(MC_SID_STREAMID_SECURITY_CONFIG_SESRD) \
    _(MC_SID_STREAMID_OVERRIDE_CONFIG_SESWR) \
    _(MC_SID_STREAMID_SECURITY_CONFIG_SESWR) \
    _(MC_SID_STREAMID_OVERRIDE_CONFIG_ETRR) \
    _(MC_SID_STREAMID_SECURITY_CONFIG_ETRR) \
    _(MC_SID_STREAMID_OVERRIDE_CONFIG_ETRW) \
    _(MC_SID_STREAMID_SECURITY_CONFIG_ETRW) \
    _(MC_SID_STREAMID_OVERRIDE_CONFIG_TSECSRDB) \
    _(MC_SID_STREAMID_SECURITY_CONFIG_TSECSRDB) \
    _(MC_SID_STREAMID_OVERRIDE_CONFIG_TSECSWRB) \
    _(MC_SID_STREAMID_SECURITY_CONFIG_TSECSWRB) \
    _(MC_SID_STREAMID_OVERRIDE_CONFIG_GPUSRD2) \
    _(MC_SID_STREAMID_SECURITY_CONFIG_GPUSRD2) \
    _(MC_SID_STREAMID_OVERRIDE_CONFIG_GPUSWR2) \
    _(MC_SID_STREAMID_SECURITY_CONFIG_GPUSWR2) \
    _(MC_SID_STREAMID_OVERRIDE_CONFIG_AXISR) \
    _(MC_SID_STREAMID_SECURITY_CONFIG_AXISR) \
    _(MC_SID_STREAMID_OVERRIDE_CONFIG_AXISW) \
    _(MC_SID_STREAMID_SECURITY_CONFIG_AXISW) \
    _(MC_SID_STREAMID_OVERRIDE_CONFIG_EQOSR) \
    _(MC_SID_STREAMID_SECURITY_CONFIG_EQOSR) \
    _(MC_SID_STREAMID_OVERRIDE_CONFIG_EQOSW) \
    _(MC_SID_STREAMID_SECURITY_CONFIG_EQOSW) \
    _(MC_SID_STREAMID_OVERRIDE_CONFIG_UFSHCR) \
    _(MC_SID_STREAMID_SECURITY_CONFIG_UFSHCR) \
    _(MC_SID_STREAMID_OVERRIDE_CONFIG_UFSHCW) \
    _(MC_SID_STREAMID_SECURITY_CONFIG_UFSHCW) \
    _(MC_SID_STREAMID_OVERRIDE_CONFIG_NVDISPLAYR) \
    _(MC_SID_STREAMID_SECURITY_CONFIG_NVDISPLAYR) \
    _(MC_SID_STREAMID_OVERRIDE_CONFIG_BPMPR) \
    _(MC_SID_STREAMID_SECURITY_CONFIG_BPMPR) \
    _(MC_SID_STREAMID_OVERRIDE_CONFIG_BPMPW) \
    _(MC_SID_STREAMID_SECURITY_CONFIG_BPMPW) \
    _(MC_SID_STREAMID_OVERRIDE_CONFIG_BPMPDMAR) \
    _(MC_SID_STREAMID_SECURITY_CONFIG_BPMPDMAR) \
    _(MC_SID_STREAMID_OVERRIDE_CONFIG_BPMPDMAW) \
    _(MC_SID_STREAMID_SECURITY_CONFIG_BPMPDMAW) \
    _(MC_SID_STREAMID_OVERRIDE_CONFIG_AONR) \
    _(MC_SID_STREAMID_SECURITY_CONFIG_AONR) \
    _(MC_SID_STREAMID_OVERRIDE_CONFIG_AONW) \
    _(MC_SID_STREAMID_SECURITY_CONFIG_AONW) \
    _(MC_SID_STREAMID_OVERRIDE_CONFIG_AONDMAR) \
    _(MC_SID_STREAMID_SECURITY_CONFIG_AONDMAR) \
    _(MC_SID_STREAMID_OVERRIDE_CONFIG_AONDMAW) \
    _(MC_SID_STREAMID_SECURITY_CONFIG_AONDMAW) \
    _(MC_SID_STREAMID_OVERRIDE_CONFIG_SCER) \
    _(MC_SID_STREAMID_SECURITY_CONFIG_SCER) \
    _(MC_SID_STREAMID_OVERRIDE_CONFIG_SCEW) \
    _(MC_SID_STREAMID_SECURITY_CONFIG_SCEW) \
    _(MC_SID_STREAMID_OVERRIDE_CONFIG_SCEDMAR) \
    _(MC_SID_STREAMID_SECURITY_CONFIG_SCEDMAR) \
    _(MC_SID_STREAMID_OVERRIDE_CONFIG_SCEDMAW) \
    _(MC_SID_STREAMID_SECURITY_CONFIG_SCEDMAW) \
    _(MC_SID_STREAMID_OVERRIDE_CONFIG_APEDMAR) \
    _(MC_SID_STREAMID_SECURITY_CONFIG_APEDMAR) \
    _(MC_SID_STREAMID_OVERRIDE_CONFIG_APEDMAW) \
    _(MC_SID_STREAMID_SECURITY_CONFIG_APEDMAW) \
    _(MC_SID_STREAMID_OVERRIDE_CONFIG_NVDISPLAYR1) \
    _(MC_SID_STREAMID_SECURITY_CONFIG_NVDISPLAYR1) \
    _(MC_SID_STREAMID_OVERRIDE_CONFIG_VICSRD1) \
    _(MC_SID_STREAMID_SECURITY_CONFIG_VICSRD1) \
    _(MC_SID_STREAMID_OVERRIDE_CONFIG_NVDECSRD1) \
    _(MC_SID_STREAMID_SECURITY_CONFIG_NVDECSRD1) \

#define LIST_BCT_VARS_MC_SID(_) \
    _(McSidStreamidOverrideConfigPtcr, "Specifies the value for MC_SID_STREAMID_OVERRIDE_CONFIG_PTCR") \
    _(McSidStreamidSecurityConfigPtcr, "Specifies the value for MC_SID_STREAMID_SECURITY_CONFIG_PTCR") \
    _(McSidStreamidOverrideConfigAfir, "Specifies the value for MC_SID_STREAMID_OVERRIDE_CONFIG_AFIR") \
    _(McSidStreamidSecurityConfigAfir, "Specifies the value for MC_SID_STREAMID_SECURITY_CONFIG_AFIR") \
    _(McSidStreamidOverrideConfigHdar, "Specifies the value for MC_SID_STREAMID_OVERRIDE_CONFIG_HDAR") \
    _(McSidStreamidSecurityConfigHdar, "Specifies the value for MC_SID_STREAMID_SECURITY_CONFIG_HDAR") \
    _(McSidStreamidOverrideConfigHost1xdmar, "Specifies the value for MC_SID_STREAMID_OVERRIDE_CONFIG_HOST1XDMAR") \
    _(McSidStreamidSecurityConfigHost1xdmar, "Specifies the value for MC_SID_STREAMID_SECURITY_CONFIG_HOST1XDMAR") \
    _(McSidStreamidOverrideConfigNvencsrd, "Specifies the value for MC_SID_STREAMID_OVERRIDE_CONFIG_NVENCSRD") \
    _(McSidStreamidSecurityConfigNvencsrd, "Specifies the value for MC_SID_STREAMID_SECURITY_CONFIG_NVENCSRD") \
    _(McSidStreamidOverrideConfigSatar, "Specifies the value for MC_SID_STREAMID_OVERRIDE_CONFIG_SATAR") \
    _(McSidStreamidSecurityConfigSatar, "Specifies the value for MC_SID_STREAMID_SECURITY_CONFIG_SATAR") \
    _(McSidStreamidOverrideConfigMpcorer, "Specifies the value for MC_SID_STREAMID_OVERRIDE_CONFIG_MPCORER") \
    _(McSidStreamidSecurityConfigMpcorer, "Specifies the value for MC_SID_STREAMID_SECURITY_CONFIG_MPCORER") \
    _(McSidStreamidOverrideConfigNvencswr, "Specifies the value for MC_SID_STREAMID_OVERRIDE_CONFIG_NVENCSWR") \
    _(McSidStreamidSecurityConfigNvencswr, "Specifies the value for MC_SID_STREAMID_SECURITY_CONFIG_NVENCSWR") \
    _(McSidStreamidOverrideConfigAfiw, "Specifies the value for MC_SID_STREAMID_OVERRIDE_CONFIG_AFIW") \
    _(McSidStreamidSecurityConfigAfiw, "Specifies the value for MC_SID_STREAMID_SECURITY_CONFIG_AFIW") \
    _(McSidStreamidOverrideConfigHdaw, "Specifies the value for MC_SID_STREAMID_OVERRIDE_CONFIG_HDAW") \
    _(McSidStreamidSecurityConfigHdaw, "Specifies the value for MC_SID_STREAMID_SECURITY_CONFIG_HDAW") \
    _(McSidStreamidOverrideConfigMpcorew, "Specifies the value for MC_SID_STREAMID_OVERRIDE_CONFIG_MPCOREW") \
    _(McSidStreamidSecurityConfigMpcorew, "Specifies the value for MC_SID_STREAMID_SECURITY_CONFIG_MPCOREW") \
    _(McSidStreamidOverrideConfigSataw, "Specifies the value for MC_SID_STREAMID_OVERRIDE_CONFIG_SATAW") \
    _(McSidStreamidSecurityConfigSataw, "Specifies the value for MC_SID_STREAMID_SECURITY_CONFIG_SATAW") \
    _(McSidStreamidOverrideConfigIspra, "Specifies the value for MC_SID_STREAMID_OVERRIDE_CONFIG_ISPRA") \
    _(McSidStreamidSecurityConfigIspra, "Specifies the value for MC_SID_STREAMID_SECURITY_CONFIG_ISPRA") \
    _(McSidStreamidOverrideConfigIspwa, "Specifies the value for MC_SID_STREAMID_OVERRIDE_CONFIG_ISPWA") \
    _(McSidStreamidSecurityConfigIspwa, "Specifies the value for MC_SID_STREAMID_SECURITY_CONFIG_ISPWA") \
    _(McSidStreamidOverrideConfigIspwb, "Specifies the value for MC_SID_STREAMID_OVERRIDE_CONFIG_ISPWB") \
    _(McSidStreamidSecurityConfigIspwb, "Specifies the value for MC_SID_STREAMID_SECURITY_CONFIG_ISPWB") \
    _(McSidStreamidOverrideConfigXusb_hostr, "Specifies the value for MC_SID_STREAMID_OVERRIDE_CONFIG_XUSB_HOSTR") \
    _(McSidStreamidSecurityConfigXusb_hostr, "Specifies the value for MC_SID_STREAMID_SECURITY_CONFIG_XUSB_HOSTR") \
    _(McSidStreamidOverrideConfigXusb_hostw, "Specifies the value for MC_SID_STREAMID_OVERRIDE_CONFIG_XUSB_HOSTW") \
    _(McSidStreamidSecurityConfigXusb_hostw, "Specifies the value for MC_SID_STREAMID_SECURITY_CONFIG_XUSB_HOSTW") \
    _(McSidStreamidOverrideConfigXusb_devr, "Specifies the value for MC_SID_STREAMID_OVERRIDE_CONFIG_XUSB_DEVR") \
    _(McSidStreamidSecurityConfigXusb_devr, "Specifies the value for MC_SID_STREAMID_SECURITY_CONFIG_XUSB_DEVR") \
    _(McSidStreamidOverrideConfigXusb_devw, "Specifies the value for MC_SID_STREAMID_OVERRIDE_CONFIG_XUSB_DEVW") \
    _(McSidStreamidSecurityConfigXusb_devw, "Specifies the value for MC_SID_STREAMID_SECURITY_CONFIG_XUSB_DEVW") \
    _(McSidStreamidOverrideConfigTsecsrd, "Specifies the value for MC_SID_STREAMID_OVERRIDE_CONFIG_TSECSRD") \
    _(McSidStreamidSecurityConfigTsecsrd, "Specifies the value for MC_SID_STREAMID_SECURITY_CONFIG_TSECSRD") \
    _(McSidStreamidOverrideConfigTsecswr, "Specifies the value for MC_SID_STREAMID_OVERRIDE_CONFIG_TSECSWR") \
    _(McSidStreamidSecurityConfigTsecswr, "Specifies the value for MC_SID_STREAMID_SECURITY_CONFIG_TSECSWR") \
    _(McSidStreamidOverrideConfigGpusrd, "Specifies the value for MC_SID_STREAMID_OVERRIDE_CONFIG_GPUSRD") \
    _(McSidStreamidSecurityConfigGpusrd, "Specifies the value for MC_SID_STREAMID_SECURITY_CONFIG_GPUSRD") \
    _(McSidStreamidOverrideConfigGpuswr, "Specifies the value for MC_SID_STREAMID_OVERRIDE_CONFIG_GPUSWR") \
    _(McSidStreamidSecurityConfigGpuswr, "Specifies the value for MC_SID_STREAMID_SECURITY_CONFIG_GPUSWR") \
    _(McSidStreamidOverrideConfigSdmmcra, "Specifies the value for MC_SID_STREAMID_OVERRIDE_CONFIG_SDMMCRA") \
    _(McSidStreamidSecurityConfigSdmmcra, "Specifies the value for MC_SID_STREAMID_SECURITY_CONFIG_SDMMCRA") \
    _(McSidStreamidOverrideConfigSdmmcraa, "Specifies the value for MC_SID_STREAMID_OVERRIDE_CONFIG_SDMMCRAA") \
    _(McSidStreamidSecurityConfigSdmmcraa, "Specifies the value for MC_SID_STREAMID_SECURITY_CONFIG_SDMMCRAA") \
    _(McSidStreamidOverrideConfigSdmmcr, "Specifies the value for MC_SID_STREAMID_OVERRIDE_CONFIG_SDMMCR") \
    _(McSidStreamidSecurityConfigSdmmcr, "Specifies the value for MC_SID_STREAMID_SECURITY_CONFIG_SDMMCR") \
    _(McSidStreamidOverrideConfigSdmmcrab, "Specifies the value for MC_SID_STREAMID_OVERRIDE_CONFIG_SDMMCRAB") \
    _(McSidStreamidSecurityConfigSdmmcrab, "Specifies the value for MC_SID_STREAMID_SECURITY_CONFIG_SDMMCRAB") \
    _(McSidStreamidOverrideConfigSdmmcwa, "Specifies the value for MC_SID_STREAMID_OVERRIDE_CONFIG_SDMMCWA") \
    _(McSidStreamidSecurityConfigSdmmcwa, "Specifies the value for MC_SID_STREAMID_SECURITY_CONFIG_SDMMCWA") \
    _(McSidStreamidOverrideConfigSdmmcwaa, "Specifies the value for MC_SID_STREAMID_OVERRIDE_CONFIG_SDMMCWAA") \
    _(McSidStreamidSecurityConfigSdmmcwaa, "Specifies the value for MC_SID_STREAMID_SECURITY_CONFIG_SDMMCWAA") \
    _(McSidStreamidOverrideConfigSdmmcw, "Specifies the value for MC_SID_STREAMID_OVERRIDE_CONFIG_SDMMCW") \
    _(McSidStreamidSecurityConfigSdmmcw, "Specifies the value for MC_SID_STREAMID_SECURITY_CONFIG_SDMMCW") \
    _(McSidStreamidOverrideConfigSdmmcwab, "Specifies the value for MC_SID_STREAMID_OVERRIDE_CONFIG_SDMMCWAB") \
    _(McSidStreamidSecurityConfigSdmmcwab, "Specifies the value for MC_SID_STREAMID_SECURITY_CONFIG_SDMMCWAB") \
    _(McSidStreamidOverrideConfigVicsrd, "Specifies the value for MC_SID_STREAMID_OVERRIDE_CONFIG_VICSRD") \
    _(McSidStreamidSecurityConfigVicsrd, "Specifies the value for MC_SID_STREAMID_SECURITY_CONFIG_VICSRD") \
    _(McSidStreamidOverrideConfigVicswr, "Specifies the value for MC_SID_STREAMID_OVERRIDE_CONFIG_VICSWR") \
    _(McSidStreamidSecurityConfigVicswr, "Specifies the value for MC_SID_STREAMID_SECURITY_CONFIG_VICSWR") \
    _(McSidStreamidOverrideConfigViw, "Specifies the value for MC_SID_STREAMID_OVERRIDE_CONFIG_VIW") \
    _(McSidStreamidSecurityConfigViw, "Specifies the value for MC_SID_STREAMID_SECURITY_CONFIG_VIW") \
    _(McSidStreamidOverrideConfigNvdecsrd, "Specifies the value for MC_SID_STREAMID_OVERRIDE_CONFIG_NVDECSRD") \
    _(McSidStreamidSecurityConfigNvdecsrd, "Specifies the value for MC_SID_STREAMID_SECURITY_CONFIG_NVDECSRD") \
    _(McSidStreamidOverrideConfigNvdecswr, "Specifies the value for MC_SID_STREAMID_OVERRIDE_CONFIG_NVDECSWR") \
    _(McSidStreamidSecurityConfigNvdecswr, "Specifies the value for MC_SID_STREAMID_SECURITY_CONFIG_NVDECSWR") \
    _(McSidStreamidOverrideConfigAper, "Specifies the value for MC_SID_STREAMID_OVERRIDE_CONFIG_APER") \
    _(McSidStreamidSecurityConfigAper, "Specifies the value for MC_SID_STREAMID_SECURITY_CONFIG_APER") \
    _(McSidStreamidOverrideConfigApew, "Specifies the value for MC_SID_STREAMID_OVERRIDE_CONFIG_APEW") \
    _(McSidStreamidSecurityConfigApew, "Specifies the value for MC_SID_STREAMID_SECURITY_CONFIG_APEW") \
    _(McSidStreamidOverrideConfigNvjpgsrd, "Specifies the value for MC_SID_STREAMID_OVERRIDE_CONFIG_NVJPGSRD") \
    _(McSidStreamidSecurityConfigNvjpgsrd, "Specifies the value for MC_SID_STREAMID_SECURITY_CONFIG_NVJPGSRD") \
    _(McSidStreamidOverrideConfigNvjpgswr, "Specifies the value for MC_SID_STREAMID_OVERRIDE_CONFIG_NVJPGSWR") \
    _(McSidStreamidSecurityConfigNvjpgswr, "Specifies the value for MC_SID_STREAMID_SECURITY_CONFIG_NVJPGSWR") \
    _(McSidStreamidOverrideConfigSesrd, "Specifies the value for MC_SID_STREAMID_OVERRIDE_CONFIG_SESRD") \
    _(McSidStreamidSecurityConfigSesrd, "Specifies the value for MC_SID_STREAMID_SECURITY_CONFIG_SESRD") \
    _(McSidStreamidOverrideConfigSeswr, "Specifies the value for MC_SID_STREAMID_OVERRIDE_CONFIG_SESWR") \
    _(McSidStreamidSecurityConfigSeswr, "Specifies the value for MC_SID_STREAMID_SECURITY_CONFIG_SESWR") \
    _(McSidStreamidOverrideConfigEtrr, "Specifies the value for MC_SID_STREAMID_OVERRIDE_CONFIG_ETRR") \
    _(McSidStreamidSecurityConfigEtrr, "Specifies the value for MC_SID_STREAMID_SECURITY_CONFIG_ETRR") \
    _(McSidStreamidOverrideConfigEtrw, "Specifies the value for MC_SID_STREAMID_OVERRIDE_CONFIG_ETRW") \
    _(McSidStreamidSecurityConfigEtrw, "Specifies the value for MC_SID_STREAMID_SECURITY_CONFIG_ETRW") \
    _(McSidStreamidOverrideConfigTsecsrdb, "Specifies the value for MC_SID_STREAMID_OVERRIDE_CONFIG_TSECSRDB") \
    _(McSidStreamidSecurityConfigTsecsrdb, "Specifies the value for MC_SID_STREAMID_SECURITY_CONFIG_TSECSRDB") \
    _(McSidStreamidOverrideConfigTsecswrb, "Specifies the value for MC_SID_STREAMID_OVERRIDE_CONFIG_TSECSWRB") \
    _(McSidStreamidSecurityConfigTsecswrb, "Specifies the value for MC_SID_STREAMID_SECURITY_CONFIG_TSECSWRB") \
    _(McSidStreamidOverrideConfigGpusrd2, "Specifies the value for MC_SID_STREAMID_OVERRIDE_CONFIG_GPUSRD2") \
    _(McSidStreamidSecurityConfigGpusrd2, "Specifies the value for MC_SID_STREAMID_SECURITY_CONFIG_GPUSRD2") \
    _(McSidStreamidOverrideConfigGpuswr2, "Specifies the value for MC_SID_STREAMID_OVERRIDE_CONFIG_GPUSWR2") \
    _(McSidStreamidSecurityConfigGpuswr2, "Specifies the value for MC_SID_STREAMID_SECURITY_CONFIG_GPUSWR2") \
    _(McSidStreamidOverrideConfigAxisr, "Specifies the value for MC_SID_STREAMID_OVERRIDE_CONFIG_AXISR") \
    _(McSidStreamidSecurityConfigAxisr, "Specifies the value for MC_SID_STREAMID_SECURITY_CONFIG_AXISR") \
    _(McSidStreamidOverrideConfigAxisw, "Specifies the value for MC_SID_STREAMID_OVERRIDE_CONFIG_AXISW") \
    _(McSidStreamidSecurityConfigAxisw, "Specifies the value for MC_SID_STREAMID_SECURITY_CONFIG_AXISW") \
    _(McSidStreamidOverrideConfigEqosr, "Specifies the value for MC_SID_STREAMID_OVERRIDE_CONFIG_EQOSR") \
    _(McSidStreamidSecurityConfigEqosr, "Specifies the value for MC_SID_STREAMID_SECURITY_CONFIG_EQOSR") \
    _(McSidStreamidOverrideConfigEqosw, "Specifies the value for MC_SID_STREAMID_OVERRIDE_CONFIG_EQOSW") \
    _(McSidStreamidSecurityConfigEqosw, "Specifies the value for MC_SID_STREAMID_SECURITY_CONFIG_EQOSW") \
    _(McSidStreamidOverrideConfigUfshcr, "Specifies the value for MC_SID_STREAMID_OVERRIDE_CONFIG_UFSHCR") \
    _(McSidStreamidSecurityConfigUfshcr, "Specifies the value for MC_SID_STREAMID_SECURITY_CONFIG_UFSHCR") \
    _(McSidStreamidOverrideConfigUfshcw, "Specifies the value for MC_SID_STREAMID_OVERRIDE_CONFIG_UFSHCW") \
    _(McSidStreamidSecurityConfigUfshcw, "Specifies the value for MC_SID_STREAMID_SECURITY_CONFIG_UFSHCW") \
    _(McSidStreamidOverrideConfigNvdisplayr, "Specifies the value for MC_SID_STREAMID_OVERRIDE_CONFIG_NVDISPLAYR") \
    _(McSidStreamidSecurityConfigNvdisplayr, "Specifies the value for MC_SID_STREAMID_SECURITY_CONFIG_NVDISPLAYR") \
    _(McSidStreamidOverrideConfigBpmpr, "Specifies the value for MC_SID_STREAMID_OVERRIDE_CONFIG_BPMPR") \
    _(McSidStreamidSecurityConfigBpmpr, "Specifies the value for MC_SID_STREAMID_SECURITY_CONFIG_BPMPR") \
    _(McSidStreamidOverrideConfigBpmpw, "Specifies the value for MC_SID_STREAMID_OVERRIDE_CONFIG_BPMPW") \
    _(McSidStreamidSecurityConfigBpmpw, "Specifies the value for MC_SID_STREAMID_SECURITY_CONFIG_BPMPW") \
    _(McSidStreamidOverrideConfigBpmpdmar, "Specifies the value for MC_SID_STREAMID_OVERRIDE_CONFIG_BPMPDMAR") \
    _(McSidStreamidSecurityConfigBpmpdmar, "Specifies the value for MC_SID_STREAMID_SECURITY_CONFIG_BPMPDMAR") \
    _(McSidStreamidOverrideConfigBpmpdmaw, "Specifies the value for MC_SID_STREAMID_OVERRIDE_CONFIG_BPMPDMAW") \
    _(McSidStreamidSecurityConfigBpmpdmaw, "Specifies the value for MC_SID_STREAMID_SECURITY_CONFIG_BPMPDMAW") \
    _(McSidStreamidOverrideConfigAonr, "Specifies the value for MC_SID_STREAMID_OVERRIDE_CONFIG_AONR") \
    _(McSidStreamidSecurityConfigAonr, "Specifies the value for MC_SID_STREAMID_SECURITY_CONFIG_AONR") \
    _(McSidStreamidOverrideConfigAonw, "Specifies the value for MC_SID_STREAMID_OVERRIDE_CONFIG_AONW") \
    _(McSidStreamidSecurityConfigAonw, "Specifies the value for MC_SID_STREAMID_SECURITY_CONFIG_AONW") \
    _(McSidStreamidOverrideConfigAondmar, "Specifies the value for MC_SID_STREAMID_OVERRIDE_CONFIG_AONDMAR") \
    _(McSidStreamidSecurityConfigAondmar, "Specifies the value for MC_SID_STREAMID_SECURITY_CONFIG_AONDMAR") \
    _(McSidStreamidOverrideConfigAondmaw, "Specifies the value for MC_SID_STREAMID_OVERRIDE_CONFIG_AONDMAW") \
    _(McSidStreamidSecurityConfigAondmaw, "Specifies the value for MC_SID_STREAMID_SECURITY_CONFIG_AONDMAW") \
    _(McSidStreamidOverrideConfigScer, "Specifies the value for MC_SID_STREAMID_OVERRIDE_CONFIG_SCER") \
    _(McSidStreamidSecurityConfigScer, "Specifies the value for MC_SID_STREAMID_SECURITY_CONFIG_SCER") \
    _(McSidStreamidOverrideConfigScew, "Specifies the value for MC_SID_STREAMID_OVERRIDE_CONFIG_SCEW") \
    _(McSidStreamidSecurityConfigScew, "Specifies the value for MC_SID_STREAMID_SECURITY_CONFIG_SCEW") \
    _(McSidStreamidOverrideConfigScedmar, "Specifies the value for MC_SID_STREAMID_OVERRIDE_CONFIG_SCEDMAR") \
    _(McSidStreamidSecurityConfigScedmar, "Specifies the value for MC_SID_STREAMID_SECURITY_CONFIG_SCEDMAR") \
    _(McSidStreamidOverrideConfigScedmaw, "Specifies the value for MC_SID_STREAMID_OVERRIDE_CONFIG_SCEDMAW") \
    _(McSidStreamidSecurityConfigScedmaw, "Specifies the value for MC_SID_STREAMID_SECURITY_CONFIG_SCEDMAW") \
    _(McSidStreamidOverrideConfigApedmar, "Specifies the value for MC_SID_STREAMID_OVERRIDE_CONFIG_APEDMAR") \
    _(McSidStreamidSecurityConfigApedmar, "Specifies the value for MC_SID_STREAMID_SECURITY_CONFIG_APEDMAR") \
    _(McSidStreamidOverrideConfigApedmaw, "Specifies the value for MC_SID_STREAMID_OVERRIDE_CONFIG_APEDMAW") \
    _(McSidStreamidSecurityConfigApedmaw, "Specifies the value for MC_SID_STREAMID_SECURITY_CONFIG_APEDMAW") \
    _(McSidStreamidOverrideConfigNvdisplayr1, "Specifies the value for MC_SID_STREAMID_OVERRIDE_CONFIG_NVDISPLAYR1") \
    _(McSidStreamidSecurityConfigNvdisplayr1, "Specifies the value for MC_SID_STREAMID_SECURITY_CONFIG_NVDISPLAYR1") \
    _(McSidStreamidOverrideConfigVicsrd1, "Specifies the value for MC_SID_STREAMID_OVERRIDE_CONFIG_VICSRD1") \
    _(McSidStreamidSecurityConfigVicsrd1, "Specifies the value for MC_SID_STREAMID_SECURITY_CONFIG_VICSRD1") \
    _(McSidStreamidOverrideConfigNvdecsrd1, "Specifies the value for MC_SID_STREAMID_OVERRIDE_CONFIG_NVDECSRD1") \
    _(McSidStreamidSecurityConfigNvdecsrd1, "Specifies the value for MC_SID_STREAMID_SECURITY_CONFIG_NVDECSRD1") \

#define LIST_REG_TYP_MC_SID \
    reg.typ[MC_SID_STREAMID_OVERRIDE_CONFIG_PTCR] = NV_RESETVAL(MC_SID,STREAMID_OVERRIDE_CONFIG_PTCR);\
    reg.typ[MC_SID_STREAMID_SECURITY_CONFIG_PTCR] = NV_RESETVAL(MC_SID,STREAMID_SECURITY_CONFIG_PTCR);\
    reg.typ[MC_SID_STREAMID_OVERRIDE_CONFIG_AFIR] = NV_RESETVAL(MC_SID,STREAMID_OVERRIDE_CONFIG_AFIR);\
    reg.typ[MC_SID_STREAMID_SECURITY_CONFIG_AFIR] = NV_RESETVAL(MC_SID,STREAMID_SECURITY_CONFIG_AFIR);\
    reg.typ[MC_SID_STREAMID_OVERRIDE_CONFIG_HDAR] = NV_RESETVAL(MC_SID,STREAMID_OVERRIDE_CONFIG_HDAR);\
    reg.typ[MC_SID_STREAMID_SECURITY_CONFIG_HDAR] = NV_RESETVAL(MC_SID,STREAMID_SECURITY_CONFIG_HDAR);\
    reg.typ[MC_SID_STREAMID_OVERRIDE_CONFIG_HOST1XDMAR] = NV_RESETVAL(MC_SID,STREAMID_OVERRIDE_CONFIG_HOST1XDMAR);\
    reg.typ[MC_SID_STREAMID_SECURITY_CONFIG_HOST1XDMAR] = NV_RESETVAL(MC_SID,STREAMID_SECURITY_CONFIG_HOST1XDMAR);\
    reg.typ[MC_SID_STREAMID_OVERRIDE_CONFIG_NVENCSRD] = NV_RESETVAL(MC_SID,STREAMID_OVERRIDE_CONFIG_NVENCSRD);\
    reg.typ[MC_SID_STREAMID_SECURITY_CONFIG_NVENCSRD] = NV_RESETVAL(MC_SID,STREAMID_SECURITY_CONFIG_NVENCSRD);\
    reg.typ[MC_SID_STREAMID_OVERRIDE_CONFIG_SATAR] = NV_RESETVAL(MC_SID,STREAMID_OVERRIDE_CONFIG_SATAR);\
    reg.typ[MC_SID_STREAMID_SECURITY_CONFIG_SATAR] = NV_RESETVAL(MC_SID,STREAMID_SECURITY_CONFIG_SATAR);\
    reg.typ[MC_SID_STREAMID_OVERRIDE_CONFIG_MPCORER] = NV_RESETVAL(MC_SID,STREAMID_OVERRIDE_CONFIG_MPCORER);\
    reg.typ[MC_SID_STREAMID_SECURITY_CONFIG_MPCORER] = NV_RESETVAL(MC_SID,STREAMID_SECURITY_CONFIG_MPCORER);\
    reg.typ[MC_SID_STREAMID_OVERRIDE_CONFIG_NVENCSWR] = NV_RESETVAL(MC_SID,STREAMID_OVERRIDE_CONFIG_NVENCSWR);\
    reg.typ[MC_SID_STREAMID_SECURITY_CONFIG_NVENCSWR] = NV_RESETVAL(MC_SID,STREAMID_SECURITY_CONFIG_NVENCSWR);\
    reg.typ[MC_SID_STREAMID_OVERRIDE_CONFIG_AFIW] = NV_RESETVAL(MC_SID,STREAMID_OVERRIDE_CONFIG_AFIW);\
    reg.typ[MC_SID_STREAMID_SECURITY_CONFIG_AFIW] = NV_RESETVAL(MC_SID,STREAMID_SECURITY_CONFIG_AFIW);\
    reg.typ[MC_SID_STREAMID_OVERRIDE_CONFIG_HDAW] = NV_RESETVAL(MC_SID,STREAMID_OVERRIDE_CONFIG_HDAW);\
    reg.typ[MC_SID_STREAMID_SECURITY_CONFIG_HDAW] = NV_RESETVAL(MC_SID,STREAMID_SECURITY_CONFIG_HDAW);\
    reg.typ[MC_SID_STREAMID_OVERRIDE_CONFIG_MPCOREW] = NV_RESETVAL(MC_SID,STREAMID_OVERRIDE_CONFIG_MPCOREW);\
    reg.typ[MC_SID_STREAMID_SECURITY_CONFIG_MPCOREW] = NV_RESETVAL(MC_SID,STREAMID_SECURITY_CONFIG_MPCOREW);\
    reg.typ[MC_SID_STREAMID_OVERRIDE_CONFIG_SATAW] = NV_RESETVAL(MC_SID,STREAMID_OVERRIDE_CONFIG_SATAW);\
    reg.typ[MC_SID_STREAMID_SECURITY_CONFIG_SATAW] = NV_RESETVAL(MC_SID,STREAMID_SECURITY_CONFIG_SATAW);\
    reg.typ[MC_SID_STREAMID_OVERRIDE_CONFIG_ISPRA] = NV_RESETVAL(MC_SID,STREAMID_OVERRIDE_CONFIG_ISPRA);\
    reg.typ[MC_SID_STREAMID_SECURITY_CONFIG_ISPRA] = NV_RESETVAL(MC_SID,STREAMID_SECURITY_CONFIG_ISPRA);\
    reg.typ[MC_SID_STREAMID_OVERRIDE_CONFIG_ISPWA] = NV_RESETVAL(MC_SID,STREAMID_OVERRIDE_CONFIG_ISPWA);\
    reg.typ[MC_SID_STREAMID_SECURITY_CONFIG_ISPWA] = NV_RESETVAL(MC_SID,STREAMID_SECURITY_CONFIG_ISPWA);\
    reg.typ[MC_SID_STREAMID_OVERRIDE_CONFIG_ISPWB] = NV_RESETVAL(MC_SID,STREAMID_OVERRIDE_CONFIG_ISPWB);\
    reg.typ[MC_SID_STREAMID_SECURITY_CONFIG_ISPWB] = NV_RESETVAL(MC_SID,STREAMID_SECURITY_CONFIG_ISPWB);\
    reg.typ[MC_SID_STREAMID_OVERRIDE_CONFIG_XUSB_HOSTR] = NV_RESETVAL(MC_SID,STREAMID_OVERRIDE_CONFIG_XUSB_HOSTR);\
    reg.typ[MC_SID_STREAMID_SECURITY_CONFIG_XUSB_HOSTR] = NV_RESETVAL(MC_SID,STREAMID_SECURITY_CONFIG_XUSB_HOSTR);\
    reg.typ[MC_SID_STREAMID_OVERRIDE_CONFIG_XUSB_HOSTW] = NV_RESETVAL(MC_SID,STREAMID_OVERRIDE_CONFIG_XUSB_HOSTW);\
    reg.typ[MC_SID_STREAMID_SECURITY_CONFIG_XUSB_HOSTW] = NV_RESETVAL(MC_SID,STREAMID_SECURITY_CONFIG_XUSB_HOSTW);\
    reg.typ[MC_SID_STREAMID_OVERRIDE_CONFIG_XUSB_DEVR] = NV_RESETVAL(MC_SID,STREAMID_OVERRIDE_CONFIG_XUSB_DEVR);\
    reg.typ[MC_SID_STREAMID_SECURITY_CONFIG_XUSB_DEVR] = NV_RESETVAL(MC_SID,STREAMID_SECURITY_CONFIG_XUSB_DEVR);\
    reg.typ[MC_SID_STREAMID_OVERRIDE_CONFIG_XUSB_DEVW] = NV_RESETVAL(MC_SID,STREAMID_OVERRIDE_CONFIG_XUSB_DEVW);\
    reg.typ[MC_SID_STREAMID_SECURITY_CONFIG_XUSB_DEVW] = NV_RESETVAL(MC_SID,STREAMID_SECURITY_CONFIG_XUSB_DEVW);\
    reg.typ[MC_SID_STREAMID_OVERRIDE_CONFIG_TSECSRD] = NV_RESETVAL(MC_SID,STREAMID_OVERRIDE_CONFIG_TSECSRD);\
    reg.typ[MC_SID_STREAMID_SECURITY_CONFIG_TSECSRD] = NV_RESETVAL(MC_SID,STREAMID_SECURITY_CONFIG_TSECSRD);\
    reg.typ[MC_SID_STREAMID_OVERRIDE_CONFIG_TSECSWR] = NV_RESETVAL(MC_SID,STREAMID_OVERRIDE_CONFIG_TSECSWR);\
    reg.typ[MC_SID_STREAMID_SECURITY_CONFIG_TSECSWR] = NV_RESETVAL(MC_SID,STREAMID_SECURITY_CONFIG_TSECSWR);\
    reg.typ[MC_SID_STREAMID_OVERRIDE_CONFIG_GPUSRD] = NV_RESETVAL(MC_SID,STREAMID_OVERRIDE_CONFIG_GPUSRD);\
    reg.typ[MC_SID_STREAMID_SECURITY_CONFIG_GPUSRD] = NV_RESETVAL(MC_SID,STREAMID_SECURITY_CONFIG_GPUSRD);\
    reg.typ[MC_SID_STREAMID_OVERRIDE_CONFIG_GPUSWR] = NV_RESETVAL(MC_SID,STREAMID_OVERRIDE_CONFIG_GPUSWR);\
    reg.typ[MC_SID_STREAMID_SECURITY_CONFIG_GPUSWR] = NV_RESETVAL(MC_SID,STREAMID_SECURITY_CONFIG_GPUSWR);\
    reg.typ[MC_SID_STREAMID_OVERRIDE_CONFIG_SDMMCRA] = NV_RESETVAL(MC_SID,STREAMID_OVERRIDE_CONFIG_SDMMCRA);\
    reg.typ[MC_SID_STREAMID_SECURITY_CONFIG_SDMMCRA] = NV_RESETVAL(MC_SID,STREAMID_SECURITY_CONFIG_SDMMCRA);\
    reg.typ[MC_SID_STREAMID_OVERRIDE_CONFIG_SDMMCRAA] = NV_RESETVAL(MC_SID,STREAMID_OVERRIDE_CONFIG_SDMMCRAA);\
    reg.typ[MC_SID_STREAMID_SECURITY_CONFIG_SDMMCRAA] = NV_RESETVAL(MC_SID,STREAMID_SECURITY_CONFIG_SDMMCRAA);\
    reg.typ[MC_SID_STREAMID_OVERRIDE_CONFIG_SDMMCR] = NV_RESETVAL(MC_SID,STREAMID_OVERRIDE_CONFIG_SDMMCR);\
    reg.typ[MC_SID_STREAMID_SECURITY_CONFIG_SDMMCR] = NV_RESETVAL(MC_SID,STREAMID_SECURITY_CONFIG_SDMMCR);\
    reg.typ[MC_SID_STREAMID_OVERRIDE_CONFIG_SDMMCRAB] = NV_RESETVAL(MC_SID,STREAMID_OVERRIDE_CONFIG_SDMMCRAB);\
    reg.typ[MC_SID_STREAMID_SECURITY_CONFIG_SDMMCRAB] = NV_RESETVAL(MC_SID,STREAMID_SECURITY_CONFIG_SDMMCRAB);\
    reg.typ[MC_SID_STREAMID_OVERRIDE_CONFIG_SDMMCWA] = NV_RESETVAL(MC_SID,STREAMID_OVERRIDE_CONFIG_SDMMCWA);\
    reg.typ[MC_SID_STREAMID_SECURITY_CONFIG_SDMMCWA] = NV_RESETVAL(MC_SID,STREAMID_SECURITY_CONFIG_SDMMCWA);\
    reg.typ[MC_SID_STREAMID_OVERRIDE_CONFIG_SDMMCWAA] = NV_RESETVAL(MC_SID,STREAMID_OVERRIDE_CONFIG_SDMMCWAA);\
    reg.typ[MC_SID_STREAMID_SECURITY_CONFIG_SDMMCWAA] = NV_RESETVAL(MC_SID,STREAMID_SECURITY_CONFIG_SDMMCWAA);\
    reg.typ[MC_SID_STREAMID_OVERRIDE_CONFIG_SDMMCW] = NV_RESETVAL(MC_SID,STREAMID_OVERRIDE_CONFIG_SDMMCW);\
    reg.typ[MC_SID_STREAMID_SECURITY_CONFIG_SDMMCW] = NV_RESETVAL(MC_SID,STREAMID_SECURITY_CONFIG_SDMMCW);\
    reg.typ[MC_SID_STREAMID_OVERRIDE_CONFIG_SDMMCWAB] = NV_RESETVAL(MC_SID,STREAMID_OVERRIDE_CONFIG_SDMMCWAB);\
    reg.typ[MC_SID_STREAMID_SECURITY_CONFIG_SDMMCWAB] = NV_RESETVAL(MC_SID,STREAMID_SECURITY_CONFIG_SDMMCWAB);\
    reg.typ[MC_SID_STREAMID_OVERRIDE_CONFIG_VICSRD] = NV_RESETVAL(MC_SID,STREAMID_OVERRIDE_CONFIG_VICSRD);\
    reg.typ[MC_SID_STREAMID_SECURITY_CONFIG_VICSRD] = NV_RESETVAL(MC_SID,STREAMID_SECURITY_CONFIG_VICSRD);\
    reg.typ[MC_SID_STREAMID_OVERRIDE_CONFIG_VICSWR] = NV_RESETVAL(MC_SID,STREAMID_OVERRIDE_CONFIG_VICSWR);\
    reg.typ[MC_SID_STREAMID_SECURITY_CONFIG_VICSWR] = NV_RESETVAL(MC_SID,STREAMID_SECURITY_CONFIG_VICSWR);\
    reg.typ[MC_SID_STREAMID_OVERRIDE_CONFIG_VIW] = NV_RESETVAL(MC_SID,STREAMID_OVERRIDE_CONFIG_VIW);\
    reg.typ[MC_SID_STREAMID_SECURITY_CONFIG_VIW] = NV_RESETVAL(MC_SID,STREAMID_SECURITY_CONFIG_VIW);\
    reg.typ[MC_SID_STREAMID_OVERRIDE_CONFIG_NVDECSRD] = NV_RESETVAL(MC_SID,STREAMID_OVERRIDE_CONFIG_NVDECSRD);\
    reg.typ[MC_SID_STREAMID_SECURITY_CONFIG_NVDECSRD] = NV_RESETVAL(MC_SID,STREAMID_SECURITY_CONFIG_NVDECSRD);\
    reg.typ[MC_SID_STREAMID_OVERRIDE_CONFIG_NVDECSWR] = NV_RESETVAL(MC_SID,STREAMID_OVERRIDE_CONFIG_NVDECSWR);\
    reg.typ[MC_SID_STREAMID_SECURITY_CONFIG_NVDECSWR] = NV_RESETVAL(MC_SID,STREAMID_SECURITY_CONFIG_NVDECSWR);\
    reg.typ[MC_SID_STREAMID_OVERRIDE_CONFIG_APER] = NV_RESETVAL(MC_SID,STREAMID_OVERRIDE_CONFIG_APER);\
    reg.typ[MC_SID_STREAMID_SECURITY_CONFIG_APER] = NV_RESETVAL(MC_SID,STREAMID_SECURITY_CONFIG_APER);\
    reg.typ[MC_SID_STREAMID_OVERRIDE_CONFIG_APEW] = NV_RESETVAL(MC_SID,STREAMID_OVERRIDE_CONFIG_APEW);\
    reg.typ[MC_SID_STREAMID_SECURITY_CONFIG_APEW] = NV_RESETVAL(MC_SID,STREAMID_SECURITY_CONFIG_APEW);\
    reg.typ[MC_SID_STREAMID_OVERRIDE_CONFIG_NVJPGSRD] = NV_RESETVAL(MC_SID,STREAMID_OVERRIDE_CONFIG_NVJPGSRD);\
    reg.typ[MC_SID_STREAMID_SECURITY_CONFIG_NVJPGSRD] = NV_RESETVAL(MC_SID,STREAMID_SECURITY_CONFIG_NVJPGSRD);\
    reg.typ[MC_SID_STREAMID_OVERRIDE_CONFIG_NVJPGSWR] = NV_RESETVAL(MC_SID,STREAMID_OVERRIDE_CONFIG_NVJPGSWR);\
    reg.typ[MC_SID_STREAMID_SECURITY_CONFIG_NVJPGSWR] = NV_RESETVAL(MC_SID,STREAMID_SECURITY_CONFIG_NVJPGSWR);\
    reg.typ[MC_SID_STREAMID_OVERRIDE_CONFIG_SESRD] = NV_RESETVAL(MC_SID,STREAMID_OVERRIDE_CONFIG_SESRD);\
    reg.typ[MC_SID_STREAMID_SECURITY_CONFIG_SESRD] = NV_RESETVAL(MC_SID,STREAMID_SECURITY_CONFIG_SESRD);\
    reg.typ[MC_SID_STREAMID_OVERRIDE_CONFIG_SESWR] = NV_RESETVAL(MC_SID,STREAMID_OVERRIDE_CONFIG_SESWR);\
    reg.typ[MC_SID_STREAMID_SECURITY_CONFIG_SESWR] = NV_RESETVAL(MC_SID,STREAMID_SECURITY_CONFIG_SESWR);\
    reg.typ[MC_SID_STREAMID_OVERRIDE_CONFIG_ETRR] = NV_RESETVAL(MC_SID,STREAMID_OVERRIDE_CONFIG_ETRR);\
    reg.typ[MC_SID_STREAMID_SECURITY_CONFIG_ETRR] = NV_RESETVAL(MC_SID,STREAMID_SECURITY_CONFIG_ETRR);\
    reg.typ[MC_SID_STREAMID_OVERRIDE_CONFIG_ETRW] = NV_RESETVAL(MC_SID,STREAMID_OVERRIDE_CONFIG_ETRW);\
    reg.typ[MC_SID_STREAMID_SECURITY_CONFIG_ETRW] = NV_RESETVAL(MC_SID,STREAMID_SECURITY_CONFIG_ETRW);\
    reg.typ[MC_SID_STREAMID_OVERRIDE_CONFIG_TSECSRDB] = NV_RESETVAL(MC_SID,STREAMID_OVERRIDE_CONFIG_TSECSRDB);\
    reg.typ[MC_SID_STREAMID_SECURITY_CONFIG_TSECSRDB] = NV_RESETVAL(MC_SID,STREAMID_SECURITY_CONFIG_TSECSRDB);\
    reg.typ[MC_SID_STREAMID_OVERRIDE_CONFIG_TSECSWRB] = NV_RESETVAL(MC_SID,STREAMID_OVERRIDE_CONFIG_TSECSWRB);\
    reg.typ[MC_SID_STREAMID_SECURITY_CONFIG_TSECSWRB] = NV_RESETVAL(MC_SID,STREAMID_SECURITY_CONFIG_TSECSWRB);\
    reg.typ[MC_SID_STREAMID_OVERRIDE_CONFIG_GPUSRD2] = NV_RESETVAL(MC_SID,STREAMID_OVERRIDE_CONFIG_GPUSRD2);\
    reg.typ[MC_SID_STREAMID_SECURITY_CONFIG_GPUSRD2] = NV_RESETVAL(MC_SID,STREAMID_SECURITY_CONFIG_GPUSRD2);\
    reg.typ[MC_SID_STREAMID_OVERRIDE_CONFIG_GPUSWR2] = NV_RESETVAL(MC_SID,STREAMID_OVERRIDE_CONFIG_GPUSWR2);\
    reg.typ[MC_SID_STREAMID_SECURITY_CONFIG_GPUSWR2] = NV_RESETVAL(MC_SID,STREAMID_SECURITY_CONFIG_GPUSWR2);\
    reg.typ[MC_SID_STREAMID_OVERRIDE_CONFIG_AXISR] = NV_RESETVAL(MC_SID,STREAMID_OVERRIDE_CONFIG_AXISR);\
    reg.typ[MC_SID_STREAMID_SECURITY_CONFIG_AXISR] = NV_RESETVAL(MC_SID,STREAMID_SECURITY_CONFIG_AXISR);\
    reg.typ[MC_SID_STREAMID_OVERRIDE_CONFIG_AXISW] = NV_RESETVAL(MC_SID,STREAMID_OVERRIDE_CONFIG_AXISW);\
    reg.typ[MC_SID_STREAMID_SECURITY_CONFIG_AXISW] = NV_RESETVAL(MC_SID,STREAMID_SECURITY_CONFIG_AXISW);\
    reg.typ[MC_SID_STREAMID_OVERRIDE_CONFIG_EQOSR] = NV_RESETVAL(MC_SID,STREAMID_OVERRIDE_CONFIG_EQOSR);\
    reg.typ[MC_SID_STREAMID_SECURITY_CONFIG_EQOSR] = NV_RESETVAL(MC_SID,STREAMID_SECURITY_CONFIG_EQOSR);\
    reg.typ[MC_SID_STREAMID_OVERRIDE_CONFIG_EQOSW] = NV_RESETVAL(MC_SID,STREAMID_OVERRIDE_CONFIG_EQOSW);\
    reg.typ[MC_SID_STREAMID_SECURITY_CONFIG_EQOSW] = NV_RESETVAL(MC_SID,STREAMID_SECURITY_CONFIG_EQOSW);\
    reg.typ[MC_SID_STREAMID_OVERRIDE_CONFIG_UFSHCR] = NV_RESETVAL(MC_SID,STREAMID_OVERRIDE_CONFIG_UFSHCR);\
    reg.typ[MC_SID_STREAMID_SECURITY_CONFIG_UFSHCR] = NV_RESETVAL(MC_SID,STREAMID_SECURITY_CONFIG_UFSHCR);\
    reg.typ[MC_SID_STREAMID_OVERRIDE_CONFIG_UFSHCW] = NV_RESETVAL(MC_SID,STREAMID_OVERRIDE_CONFIG_UFSHCW);\
    reg.typ[MC_SID_STREAMID_SECURITY_CONFIG_UFSHCW] = NV_RESETVAL(MC_SID,STREAMID_SECURITY_CONFIG_UFSHCW);\
    reg.typ[MC_SID_STREAMID_OVERRIDE_CONFIG_NVDISPLAYR] = NV_RESETVAL(MC_SID,STREAMID_OVERRIDE_CONFIG_NVDISPLAYR);\
    reg.typ[MC_SID_STREAMID_SECURITY_CONFIG_NVDISPLAYR] = NV_RESETVAL(MC_SID,STREAMID_SECURITY_CONFIG_NVDISPLAYR);\
    reg.typ[MC_SID_STREAMID_OVERRIDE_CONFIG_BPMPR] = NV_RESETVAL(MC_SID,STREAMID_OVERRIDE_CONFIG_BPMPR);\
    reg.typ[MC_SID_STREAMID_SECURITY_CONFIG_BPMPR] = NV_RESETVAL(MC_SID,STREAMID_SECURITY_CONFIG_BPMPR);\
    reg.typ[MC_SID_STREAMID_OVERRIDE_CONFIG_BPMPW] = NV_RESETVAL(MC_SID,STREAMID_OVERRIDE_CONFIG_BPMPW);\
    reg.typ[MC_SID_STREAMID_SECURITY_CONFIG_BPMPW] = NV_RESETVAL(MC_SID,STREAMID_SECURITY_CONFIG_BPMPW);\
    reg.typ[MC_SID_STREAMID_OVERRIDE_CONFIG_BPMPDMAR] = NV_RESETVAL(MC_SID,STREAMID_OVERRIDE_CONFIG_BPMPDMAR);\
    reg.typ[MC_SID_STREAMID_SECURITY_CONFIG_BPMPDMAR] = NV_RESETVAL(MC_SID,STREAMID_SECURITY_CONFIG_BPMPDMAR);\
    reg.typ[MC_SID_STREAMID_OVERRIDE_CONFIG_BPMPDMAW] = NV_RESETVAL(MC_SID,STREAMID_OVERRIDE_CONFIG_BPMPDMAW);\
    reg.typ[MC_SID_STREAMID_SECURITY_CONFIG_BPMPDMAW] = NV_RESETVAL(MC_SID,STREAMID_SECURITY_CONFIG_BPMPDMAW);\
    reg.typ[MC_SID_STREAMID_OVERRIDE_CONFIG_AONR] = NV_RESETVAL(MC_SID,STREAMID_OVERRIDE_CONFIG_AONR);\
    reg.typ[MC_SID_STREAMID_SECURITY_CONFIG_AONR] = NV_RESETVAL(MC_SID,STREAMID_SECURITY_CONFIG_AONR);\
    reg.typ[MC_SID_STREAMID_OVERRIDE_CONFIG_AONW] = NV_RESETVAL(MC_SID,STREAMID_OVERRIDE_CONFIG_AONW);\
    reg.typ[MC_SID_STREAMID_SECURITY_CONFIG_AONW] = NV_RESETVAL(MC_SID,STREAMID_SECURITY_CONFIG_AONW);\
    reg.typ[MC_SID_STREAMID_OVERRIDE_CONFIG_AONDMAR] = NV_RESETVAL(MC_SID,STREAMID_OVERRIDE_CONFIG_AONDMAR);\
    reg.typ[MC_SID_STREAMID_SECURITY_CONFIG_AONDMAR] = NV_RESETVAL(MC_SID,STREAMID_SECURITY_CONFIG_AONDMAR);\
    reg.typ[MC_SID_STREAMID_OVERRIDE_CONFIG_AONDMAW] = NV_RESETVAL(MC_SID,STREAMID_OVERRIDE_CONFIG_AONDMAW);\
    reg.typ[MC_SID_STREAMID_SECURITY_CONFIG_AONDMAW] = NV_RESETVAL(MC_SID,STREAMID_SECURITY_CONFIG_AONDMAW);\
    reg.typ[MC_SID_STREAMID_OVERRIDE_CONFIG_SCER] = NV_RESETVAL(MC_SID,STREAMID_OVERRIDE_CONFIG_SCER);\
    reg.typ[MC_SID_STREAMID_SECURITY_CONFIG_SCER] = NV_RESETVAL(MC_SID,STREAMID_SECURITY_CONFIG_SCER);\
    reg.typ[MC_SID_STREAMID_OVERRIDE_CONFIG_SCEW] = NV_RESETVAL(MC_SID,STREAMID_OVERRIDE_CONFIG_SCEW);\
    reg.typ[MC_SID_STREAMID_SECURITY_CONFIG_SCEW] = NV_RESETVAL(MC_SID,STREAMID_SECURITY_CONFIG_SCEW);\
    reg.typ[MC_SID_STREAMID_OVERRIDE_CONFIG_SCEDMAR] = NV_RESETVAL(MC_SID,STREAMID_OVERRIDE_CONFIG_SCEDMAR);\
    reg.typ[MC_SID_STREAMID_SECURITY_CONFIG_SCEDMAR] = NV_RESETVAL(MC_SID,STREAMID_SECURITY_CONFIG_SCEDMAR);\
    reg.typ[MC_SID_STREAMID_OVERRIDE_CONFIG_SCEDMAW] = NV_RESETVAL(MC_SID,STREAMID_OVERRIDE_CONFIG_SCEDMAW);\
    reg.typ[MC_SID_STREAMID_SECURITY_CONFIG_SCEDMAW] = NV_RESETVAL(MC_SID,STREAMID_SECURITY_CONFIG_SCEDMAW);\
    reg.typ[MC_SID_STREAMID_OVERRIDE_CONFIG_APEDMAR] = NV_RESETVAL(MC_SID,STREAMID_OVERRIDE_CONFIG_APEDMAR);\
    reg.typ[MC_SID_STREAMID_SECURITY_CONFIG_APEDMAR] = NV_RESETVAL(MC_SID,STREAMID_SECURITY_CONFIG_APEDMAR);\
    reg.typ[MC_SID_STREAMID_OVERRIDE_CONFIG_APEDMAW] = NV_RESETVAL(MC_SID,STREAMID_OVERRIDE_CONFIG_APEDMAW);\
    reg.typ[MC_SID_STREAMID_SECURITY_CONFIG_APEDMAW] = NV_RESETVAL(MC_SID,STREAMID_SECURITY_CONFIG_APEDMAW);\
    reg.typ[MC_SID_STREAMID_OVERRIDE_CONFIG_NVDISPLAYR1] = NV_RESETVAL(MC_SID,STREAMID_OVERRIDE_CONFIG_NVDISPLAYR1);\
    reg.typ[MC_SID_STREAMID_SECURITY_CONFIG_NVDISPLAYR1] = NV_RESETVAL(MC_SID,STREAMID_SECURITY_CONFIG_NVDISPLAYR1);\
    reg.typ[MC_SID_STREAMID_OVERRIDE_CONFIG_VICSRD1] = NV_RESETVAL(MC_SID,STREAMID_OVERRIDE_CONFIG_VICSRD1);\
    reg.typ[MC_SID_STREAMID_SECURITY_CONFIG_VICSRD1] = NV_RESETVAL(MC_SID,STREAMID_SECURITY_CONFIG_VICSRD1);\
    reg.typ[MC_SID_STREAMID_OVERRIDE_CONFIG_NVDECSRD1] = NV_RESETVAL(MC_SID,STREAMID_OVERRIDE_CONFIG_NVDECSRD1);\
    reg.typ[MC_SID_STREAMID_SECURITY_CONFIG_NVDECSRD1] = NV_RESETVAL(MC_SID,STREAMID_SECURITY_CONFIG_NVDECSRD1);

#define LIST_REG_TYP_MC_SID_TESTING \
    reg.typ[MC_SID_STREAMID_OVERRIDE_CONFIG_PTCR] = 0x13;\
    reg.typ[MC_SID_STREAMID_SECURITY_CONFIG_PTCR] = NV_FLD_SET_DRF_NUM(MC_SID, STREAMID_SECURITY_CONFIG_PTCR,PTCR_STREAMID_OVERRIDE, 1 , NV_RESETVAL(MC_SID,STREAMID_SECURITY_CONFIG_PTCR));\
    reg.typ[MC_SID_STREAMID_OVERRIDE_CONFIG_AFIR] = 0x13;\
    reg.typ[MC_SID_STREAMID_SECURITY_CONFIG_AFIR] = NV_FLD_SET_DRF_NUM(MC_SID, STREAMID_SECURITY_CONFIG_AFIR,AFIR_STREAMID_OVERRIDE, 1 , NV_RESETVAL(MC_SID,STREAMID_SECURITY_CONFIG_AFIR));\
    reg.typ[MC_SID_STREAMID_OVERRIDE_CONFIG_HDAR] = 0x13;\
    reg.typ[MC_SID_STREAMID_SECURITY_CONFIG_HDAR] = NV_FLD_SET_DRF_NUM(MC_SID, STREAMID_SECURITY_CONFIG_HDAR,HDAR_STREAMID_OVERRIDE, 1 , NV_RESETVAL(MC_SID,STREAMID_SECURITY_CONFIG_HDAR));\
    reg.typ[MC_SID_STREAMID_OVERRIDE_CONFIG_HOST1XDMAR] = 0x13;\
    reg.typ[MC_SID_STREAMID_SECURITY_CONFIG_HOST1XDMAR] = NV_FLD_SET_DRF_NUM(MC_SID, STREAMID_SECURITY_CONFIG_HOST1XDMAR,HOST1XDMAR_STREAMID_OVERRIDE, 1 , NV_RESETVAL(MC_SID,STREAMID_SECURITY_CONFIG_HOST1XDMAR));\
    reg.typ[MC_SID_STREAMID_OVERRIDE_CONFIG_NVENCSRD] = 0x13;\
    reg.typ[MC_SID_STREAMID_SECURITY_CONFIG_NVENCSRD] = NV_FLD_SET_DRF_NUM(MC_SID, STREAMID_SECURITY_CONFIG_NVENCSRD,NVENCSRD_STREAMID_OVERRIDE, 1 , NV_RESETVAL(MC_SID,STREAMID_SECURITY_CONFIG_NVENCSRD));\
    reg.typ[MC_SID_STREAMID_OVERRIDE_CONFIG_SATAR] = 0x13;\
    reg.typ[MC_SID_STREAMID_SECURITY_CONFIG_SATAR] = NV_FLD_SET_DRF_NUM(MC_SID, STREAMID_SECURITY_CONFIG_SATAR,SATAR_STREAMID_OVERRIDE, 1 , NV_RESETVAL(MC_SID,STREAMID_SECURITY_CONFIG_SATAR));\
    reg.typ[MC_SID_STREAMID_OVERRIDE_CONFIG_MPCORER] = 0x13;\
    reg.typ[MC_SID_STREAMID_SECURITY_CONFIG_MPCORER] = NV_FLD_SET_DRF_NUM(MC_SID, STREAMID_SECURITY_CONFIG_MPCORER,MPCORER_STREAMID_OVERRIDE, 1 , NV_RESETVAL(MC_SID,STREAMID_SECURITY_CONFIG_MPCORER));\
    reg.typ[MC_SID_STREAMID_OVERRIDE_CONFIG_NVENCSWR] = 0x13;\
    reg.typ[MC_SID_STREAMID_SECURITY_CONFIG_NVENCSWR] = NV_FLD_SET_DRF_NUM(MC_SID, STREAMID_SECURITY_CONFIG_NVENCSWR,NVENCSWR_STREAMID_OVERRIDE, 1 , NV_RESETVAL(MC_SID,STREAMID_SECURITY_CONFIG_NVENCSWR));\
    reg.typ[MC_SID_STREAMID_OVERRIDE_CONFIG_AFIW] = 0x13;\
    reg.typ[MC_SID_STREAMID_SECURITY_CONFIG_AFIW] = NV_FLD_SET_DRF_NUM(MC_SID, STREAMID_SECURITY_CONFIG_AFIW,AFIW_STREAMID_OVERRIDE, 1 , NV_RESETVAL(MC_SID,STREAMID_SECURITY_CONFIG_AFIW));\
    reg.typ[MC_SID_STREAMID_OVERRIDE_CONFIG_HDAW] = 0x13;\
    reg.typ[MC_SID_STREAMID_SECURITY_CONFIG_HDAW] = NV_FLD_SET_DRF_NUM(MC_SID, STREAMID_SECURITY_CONFIG_HDAW,HDAW_STREAMID_OVERRIDE, 1 , NV_RESETVAL(MC_SID,STREAMID_SECURITY_CONFIG_HDAW));\
    reg.typ[MC_SID_STREAMID_OVERRIDE_CONFIG_MPCOREW] = 0x13;\
    reg.typ[MC_SID_STREAMID_SECURITY_CONFIG_MPCOREW] = NV_FLD_SET_DRF_NUM(MC_SID, STREAMID_SECURITY_CONFIG_MPCOREW,MPCOREW_STREAMID_OVERRIDE, 1 , NV_RESETVAL(MC_SID,STREAMID_SECURITY_CONFIG_MPCOREW));\
    reg.typ[MC_SID_STREAMID_OVERRIDE_CONFIG_SATAW] = 0x13;\
    reg.typ[MC_SID_STREAMID_SECURITY_CONFIG_SATAW] = NV_FLD_SET_DRF_NUM(MC_SID, STREAMID_SECURITY_CONFIG_SATAW,SATAW_STREAMID_OVERRIDE, 1 , NV_RESETVAL(MC_SID,STREAMID_SECURITY_CONFIG_SATAW));\
    reg.typ[MC_SID_STREAMID_OVERRIDE_CONFIG_ISPRA] = 0x13;\
    reg.typ[MC_SID_STREAMID_SECURITY_CONFIG_ISPRA] = NV_FLD_SET_DRF_NUM(MC_SID, STREAMID_SECURITY_CONFIG_ISPRA,ISPRA_STREAMID_OVERRIDE, 1 , NV_RESETVAL(MC_SID,STREAMID_SECURITY_CONFIG_ISPRA));\
    reg.typ[MC_SID_STREAMID_OVERRIDE_CONFIG_ISPWA] = 0x13;\
    reg.typ[MC_SID_STREAMID_SECURITY_CONFIG_ISPWA] = NV_FLD_SET_DRF_NUM(MC_SID, STREAMID_SECURITY_CONFIG_ISPWA,ISPWA_STREAMID_OVERRIDE, 1 , NV_RESETVAL(MC_SID,STREAMID_SECURITY_CONFIG_ISPWA));\
    reg.typ[MC_SID_STREAMID_OVERRIDE_CONFIG_ISPWB] = 0x13;\
    reg.typ[MC_SID_STREAMID_SECURITY_CONFIG_ISPWB] = NV_FLD_SET_DRF_NUM(MC_SID, STREAMID_SECURITY_CONFIG_ISPWB,ISPWB_STREAMID_OVERRIDE, 1 , NV_RESETVAL(MC_SID,STREAMID_SECURITY_CONFIG_ISPWB));\
    reg.typ[MC_SID_STREAMID_OVERRIDE_CONFIG_XUSB_HOSTR] = 0x13;\
    reg.typ[MC_SID_STREAMID_SECURITY_CONFIG_XUSB_HOSTR] = NV_FLD_SET_DRF_NUM(MC_SID, STREAMID_SECURITY_CONFIG_XUSB_HOSTR,XUSB_HOSTR_STREAMID_OVERRIDE, 1 , NV_RESETVAL(MC_SID,STREAMID_SECURITY_CONFIG_XUSB_HOSTR));\
    reg.typ[MC_SID_STREAMID_OVERRIDE_CONFIG_XUSB_HOSTW] = 0x13;\
    reg.typ[MC_SID_STREAMID_SECURITY_CONFIG_XUSB_HOSTW] = NV_FLD_SET_DRF_NUM(MC_SID, STREAMID_SECURITY_CONFIG_XUSB_HOSTW,XUSB_HOSTW_STREAMID_OVERRIDE, 1 , NV_RESETVAL(MC_SID,STREAMID_SECURITY_CONFIG_XUSB_HOSTW));\
    reg.typ[MC_SID_STREAMID_OVERRIDE_CONFIG_XUSB_DEVR] = 0x13;\
    reg.typ[MC_SID_STREAMID_SECURITY_CONFIG_XUSB_DEVR] = NV_FLD_SET_DRF_NUM(MC_SID, STREAMID_SECURITY_CONFIG_XUSB_DEVR,XUSB_DEVR_STREAMID_OVERRIDE, 1 , NV_RESETVAL(MC_SID,STREAMID_SECURITY_CONFIG_XUSB_DEVR));\
    reg.typ[MC_SID_STREAMID_OVERRIDE_CONFIG_XUSB_DEVW] = 0x13;\
    reg.typ[MC_SID_STREAMID_SECURITY_CONFIG_XUSB_DEVW] = NV_FLD_SET_DRF_NUM(MC_SID, STREAMID_SECURITY_CONFIG_XUSB_DEVW,XUSB_DEVW_STREAMID_OVERRIDE, 1 , NV_RESETVAL(MC_SID,STREAMID_SECURITY_CONFIG_XUSB_DEVW));\
    reg.typ[MC_SID_STREAMID_OVERRIDE_CONFIG_TSECSRD] = 0x13;\
    reg.typ[MC_SID_STREAMID_SECURITY_CONFIG_TSECSRD] = NV_FLD_SET_DRF_NUM(MC_SID, STREAMID_SECURITY_CONFIG_TSECSRD,TSECSRD_STREAMID_OVERRIDE, 1 , NV_RESETVAL(MC_SID,STREAMID_SECURITY_CONFIG_TSECSRD));\
    reg.typ[MC_SID_STREAMID_OVERRIDE_CONFIG_TSECSWR] = 0x13;\
    reg.typ[MC_SID_STREAMID_SECURITY_CONFIG_TSECSWR] = NV_FLD_SET_DRF_NUM(MC_SID, STREAMID_SECURITY_CONFIG_TSECSWR,TSECSWR_STREAMID_OVERRIDE, 1 , NV_RESETVAL(MC_SID,STREAMID_SECURITY_CONFIG_TSECSWR));\
    reg.typ[MC_SID_STREAMID_OVERRIDE_CONFIG_GPUSRD] = 0x13;\
    reg.typ[MC_SID_STREAMID_SECURITY_CONFIG_GPUSRD] = NV_FLD_SET_DRF_NUM(MC_SID, STREAMID_SECURITY_CONFIG_GPUSRD,GPUSRD_STREAMID_OVERRIDE, 1 , NV_RESETVAL(MC_SID,STREAMID_SECURITY_CONFIG_GPUSRD));\
    reg.typ[MC_SID_STREAMID_OVERRIDE_CONFIG_GPUSWR] = 0x13;\
    reg.typ[MC_SID_STREAMID_SECURITY_CONFIG_GPUSWR] = NV_FLD_SET_DRF_NUM(MC_SID, STREAMID_SECURITY_CONFIG_GPUSWR,GPUSWR_STREAMID_OVERRIDE, 1 , NV_RESETVAL(MC_SID,STREAMID_SECURITY_CONFIG_GPUSWR));\
    reg.typ[MC_SID_STREAMID_OVERRIDE_CONFIG_SDMMCRA] = 0x13;\
    reg.typ[MC_SID_STREAMID_SECURITY_CONFIG_SDMMCRA] = NV_FLD_SET_DRF_NUM(MC_SID, STREAMID_SECURITY_CONFIG_SDMMCRA,SDMMCRA_STREAMID_OVERRIDE, 1 , NV_RESETVAL(MC_SID,STREAMID_SECURITY_CONFIG_SDMMCRA));\
    reg.typ[MC_SID_STREAMID_OVERRIDE_CONFIG_SDMMCRAA] = 0x13;\
    reg.typ[MC_SID_STREAMID_SECURITY_CONFIG_SDMMCRAA] = NV_FLD_SET_DRF_NUM(MC_SID, STREAMID_SECURITY_CONFIG_SDMMCRAA,SDMMCRAA_STREAMID_OVERRIDE, 1 , NV_RESETVAL(MC_SID,STREAMID_SECURITY_CONFIG_SDMMCRAA));\
    reg.typ[MC_SID_STREAMID_OVERRIDE_CONFIG_SDMMCR] = 0x13;\
    reg.typ[MC_SID_STREAMID_SECURITY_CONFIG_SDMMCR] = NV_FLD_SET_DRF_NUM(MC_SID, STREAMID_SECURITY_CONFIG_SDMMCR,SDMMCR_STREAMID_OVERRIDE, 1 , NV_RESETVAL(MC_SID,STREAMID_SECURITY_CONFIG_SDMMCR));\
    reg.typ[MC_SID_STREAMID_OVERRIDE_CONFIG_SDMMCRAB] = 0x13;\
    reg.typ[MC_SID_STREAMID_SECURITY_CONFIG_SDMMCRAB] = NV_FLD_SET_DRF_NUM(MC_SID, STREAMID_SECURITY_CONFIG_SDMMCRAB,SDMMCRAB_STREAMID_OVERRIDE, 1 , NV_RESETVAL(MC_SID,STREAMID_SECURITY_CONFIG_SDMMCRAB));\
    reg.typ[MC_SID_STREAMID_OVERRIDE_CONFIG_SDMMCWA] = 0x13;\
    reg.typ[MC_SID_STREAMID_SECURITY_CONFIG_SDMMCWA] = NV_FLD_SET_DRF_NUM(MC_SID, STREAMID_SECURITY_CONFIG_SDMMCWA,SDMMCWA_STREAMID_OVERRIDE, 1 , NV_RESETVAL(MC_SID,STREAMID_SECURITY_CONFIG_SDMMCWA));\
    reg.typ[MC_SID_STREAMID_OVERRIDE_CONFIG_SDMMCWAA] = 0x13;\
    reg.typ[MC_SID_STREAMID_SECURITY_CONFIG_SDMMCWAA] = NV_FLD_SET_DRF_NUM(MC_SID, STREAMID_SECURITY_CONFIG_SDMMCWAA,SDMMCWAA_STREAMID_OVERRIDE, 1 , NV_RESETVAL(MC_SID,STREAMID_SECURITY_CONFIG_SDMMCWAA));\
    reg.typ[MC_SID_STREAMID_OVERRIDE_CONFIG_SDMMCW] = 0x13;\
    reg.typ[MC_SID_STREAMID_SECURITY_CONFIG_SDMMCW] = NV_FLD_SET_DRF_NUM(MC_SID, STREAMID_SECURITY_CONFIG_SDMMCW,SDMMCW_STREAMID_OVERRIDE, 1 , NV_RESETVAL(MC_SID,STREAMID_SECURITY_CONFIG_SDMMCW));\
    reg.typ[MC_SID_STREAMID_OVERRIDE_CONFIG_SDMMCWAB] = 0x13;\
    reg.typ[MC_SID_STREAMID_SECURITY_CONFIG_SDMMCWAB] = NV_FLD_SET_DRF_NUM(MC_SID, STREAMID_SECURITY_CONFIG_SDMMCWAB,SDMMCWAB_STREAMID_OVERRIDE, 1 , NV_RESETVAL(MC_SID,STREAMID_SECURITY_CONFIG_SDMMCWAB));\
    reg.typ[MC_SID_STREAMID_OVERRIDE_CONFIG_VICSRD] = 0x13;\
    reg.typ[MC_SID_STREAMID_SECURITY_CONFIG_VICSRD] = NV_FLD_SET_DRF_NUM(MC_SID, STREAMID_SECURITY_CONFIG_VICSRD,VICSRD_STREAMID_OVERRIDE, 1 , NV_RESETVAL(MC_SID,STREAMID_SECURITY_CONFIG_VICSRD));\
    reg.typ[MC_SID_STREAMID_OVERRIDE_CONFIG_VICSWR] = 0x13;\
    reg.typ[MC_SID_STREAMID_SECURITY_CONFIG_VICSWR] = NV_FLD_SET_DRF_NUM(MC_SID, STREAMID_SECURITY_CONFIG_VICSWR,VICSWR_STREAMID_OVERRIDE, 1 , NV_RESETVAL(MC_SID,STREAMID_SECURITY_CONFIG_VICSWR));\
    reg.typ[MC_SID_STREAMID_OVERRIDE_CONFIG_VIW] = 0x13;\
    reg.typ[MC_SID_STREAMID_SECURITY_CONFIG_VIW] = NV_FLD_SET_DRF_NUM(MC_SID, STREAMID_SECURITY_CONFIG_VIW,VIW_STREAMID_OVERRIDE, 1 , NV_RESETVAL(MC_SID,STREAMID_SECURITY_CONFIG_VIW));\
    reg.typ[MC_SID_STREAMID_OVERRIDE_CONFIG_NVDECSRD] = 0x13;\
    reg.typ[MC_SID_STREAMID_SECURITY_CONFIG_NVDECSRD] = NV_FLD_SET_DRF_NUM(MC_SID, STREAMID_SECURITY_CONFIG_NVDECSRD,NVDECSRD_STREAMID_OVERRIDE, 1 , NV_RESETVAL(MC_SID,STREAMID_SECURITY_CONFIG_NVDECSRD));\
    reg.typ[MC_SID_STREAMID_OVERRIDE_CONFIG_NVDECSWR] = 0x13;\
    reg.typ[MC_SID_STREAMID_SECURITY_CONFIG_NVDECSWR] = NV_FLD_SET_DRF_NUM(MC_SID, STREAMID_SECURITY_CONFIG_NVDECSWR,NVDECSWR_STREAMID_OVERRIDE, 1 , NV_RESETVAL(MC_SID,STREAMID_SECURITY_CONFIG_NVDECSWR));\
    reg.typ[MC_SID_STREAMID_OVERRIDE_CONFIG_APER] = 0x13;\
    reg.typ[MC_SID_STREAMID_SECURITY_CONFIG_APER] = NV_FLD_SET_DRF_NUM(MC_SID, STREAMID_SECURITY_CONFIG_APER,APER_STREAMID_OVERRIDE, 1 , NV_RESETVAL(MC_SID,STREAMID_SECURITY_CONFIG_APER));\
    reg.typ[MC_SID_STREAMID_OVERRIDE_CONFIG_APEW] = 0x13;\
    reg.typ[MC_SID_STREAMID_SECURITY_CONFIG_APEW] = NV_FLD_SET_DRF_NUM(MC_SID, STREAMID_SECURITY_CONFIG_APEW,APEW_STREAMID_OVERRIDE, 1 , NV_RESETVAL(MC_SID,STREAMID_SECURITY_CONFIG_APEW));\
    reg.typ[MC_SID_STREAMID_OVERRIDE_CONFIG_NVJPGSRD] = 0x13;\
    reg.typ[MC_SID_STREAMID_SECURITY_CONFIG_NVJPGSRD] = NV_FLD_SET_DRF_NUM(MC_SID, STREAMID_SECURITY_CONFIG_NVJPGSRD,NVJPGSRD_STREAMID_OVERRIDE, 1 , NV_RESETVAL(MC_SID,STREAMID_SECURITY_CONFIG_NVJPGSRD));\
    reg.typ[MC_SID_STREAMID_OVERRIDE_CONFIG_NVJPGSWR] = 0x13;\
    reg.typ[MC_SID_STREAMID_SECURITY_CONFIG_NVJPGSWR] = NV_FLD_SET_DRF_NUM(MC_SID, STREAMID_SECURITY_CONFIG_NVJPGSWR,NVJPGSWR_STREAMID_OVERRIDE, 1 , NV_RESETVAL(MC_SID,STREAMID_SECURITY_CONFIG_NVJPGSWR));\
    reg.typ[MC_SID_STREAMID_OVERRIDE_CONFIG_SESRD] = 0x13;\
    reg.typ[MC_SID_STREAMID_SECURITY_CONFIG_SESRD] = NV_FLD_SET_DRF_NUM(MC_SID, STREAMID_SECURITY_CONFIG_SESRD,SESRD_STREAMID_OVERRIDE, 1 , NV_RESETVAL(MC_SID,STREAMID_SECURITY_CONFIG_SESRD));\
    reg.typ[MC_SID_STREAMID_OVERRIDE_CONFIG_SESWR] = 0x13;\
    reg.typ[MC_SID_STREAMID_SECURITY_CONFIG_SESWR] = NV_FLD_SET_DRF_NUM(MC_SID, STREAMID_SECURITY_CONFIG_SESWR,SESWR_STREAMID_OVERRIDE, 1 , NV_RESETVAL(MC_SID,STREAMID_SECURITY_CONFIG_SESWR));\
    reg.typ[MC_SID_STREAMID_OVERRIDE_CONFIG_ETRR] = 0x13;\
    reg.typ[MC_SID_STREAMID_SECURITY_CONFIG_ETRR] = NV_FLD_SET_DRF_NUM(MC_SID, STREAMID_SECURITY_CONFIG_ETRR,ETRR_STREAMID_OVERRIDE, 1 , NV_RESETVAL(MC_SID,STREAMID_SECURITY_CONFIG_ETRR));\
    reg.typ[MC_SID_STREAMID_OVERRIDE_CONFIG_ETRW] = 0x13;\
    reg.typ[MC_SID_STREAMID_SECURITY_CONFIG_ETRW] = NV_FLD_SET_DRF_NUM(MC_SID, STREAMID_SECURITY_CONFIG_ETRW,ETRW_STREAMID_OVERRIDE, 1 , NV_RESETVAL(MC_SID,STREAMID_SECURITY_CONFIG_ETRW));\
    reg.typ[MC_SID_STREAMID_OVERRIDE_CONFIG_TSECSRDB] = 0x13;\
    reg.typ[MC_SID_STREAMID_SECURITY_CONFIG_TSECSRDB] = NV_FLD_SET_DRF_NUM(MC_SID, STREAMID_SECURITY_CONFIG_TSECSRDB,TSECSRDB_STREAMID_OVERRIDE, 1 , NV_RESETVAL(MC_SID,STREAMID_SECURITY_CONFIG_TSECSRDB));\
    reg.typ[MC_SID_STREAMID_OVERRIDE_CONFIG_TSECSWRB] = 0x13;\
    reg.typ[MC_SID_STREAMID_SECURITY_CONFIG_TSECSWRB] = NV_FLD_SET_DRF_NUM(MC_SID, STREAMID_SECURITY_CONFIG_TSECSWRB,TSECSWRB_STREAMID_OVERRIDE, 1 , NV_RESETVAL(MC_SID,STREAMID_SECURITY_CONFIG_TSECSWRB));\
    reg.typ[MC_SID_STREAMID_OVERRIDE_CONFIG_GPUSRD2] = 0x13;\
    reg.typ[MC_SID_STREAMID_SECURITY_CONFIG_GPUSRD2] = NV_FLD_SET_DRF_NUM(MC_SID, STREAMID_SECURITY_CONFIG_GPUSRD2,GPUSRD2_STREAMID_OVERRIDE, 1 , NV_RESETVAL(MC_SID,STREAMID_SECURITY_CONFIG_GPUSRD2));\
    reg.typ[MC_SID_STREAMID_OVERRIDE_CONFIG_GPUSWR2] = 0x13;\
    reg.typ[MC_SID_STREAMID_SECURITY_CONFIG_GPUSWR2] = NV_FLD_SET_DRF_NUM(MC_SID, STREAMID_SECURITY_CONFIG_GPUSWR2,GPUSWR2_STREAMID_OVERRIDE, 1 , NV_RESETVAL(MC_SID,STREAMID_SECURITY_CONFIG_GPUSWR2));\
    reg.typ[MC_SID_STREAMID_OVERRIDE_CONFIG_AXISR] = 0x13;\
    reg.typ[MC_SID_STREAMID_SECURITY_CONFIG_AXISR] = NV_FLD_SET_DRF_NUM(MC_SID, STREAMID_SECURITY_CONFIG_AXISR,AXISR_STREAMID_OVERRIDE, 1 , NV_RESETVAL(MC_SID,STREAMID_SECURITY_CONFIG_AXISR));\
    reg.typ[MC_SID_STREAMID_OVERRIDE_CONFIG_AXISW] = 0x13;\
    reg.typ[MC_SID_STREAMID_SECURITY_CONFIG_AXISW] = NV_FLD_SET_DRF_NUM(MC_SID, STREAMID_SECURITY_CONFIG_AXISW,AXISW_STREAMID_OVERRIDE, 1 , NV_RESETVAL(MC_SID,STREAMID_SECURITY_CONFIG_AXISW));\
    reg.typ[MC_SID_STREAMID_OVERRIDE_CONFIG_EQOSR] = 0x13;\
    reg.typ[MC_SID_STREAMID_SECURITY_CONFIG_EQOSR] = NV_FLD_SET_DRF_NUM(MC_SID, STREAMID_SECURITY_CONFIG_EQOSR,EQOSR_STREAMID_OVERRIDE, 1 , NV_RESETVAL(MC_SID,STREAMID_SECURITY_CONFIG_EQOSR));\
    reg.typ[MC_SID_STREAMID_OVERRIDE_CONFIG_EQOSW] = 0x13;\
    reg.typ[MC_SID_STREAMID_SECURITY_CONFIG_EQOSW] = NV_FLD_SET_DRF_NUM(MC_SID, STREAMID_SECURITY_CONFIG_EQOSW,EQOSW_STREAMID_OVERRIDE, 1 , NV_RESETVAL(MC_SID,STREAMID_SECURITY_CONFIG_EQOSW));\
    reg.typ[MC_SID_STREAMID_OVERRIDE_CONFIG_UFSHCR] = 0x13;\
    reg.typ[MC_SID_STREAMID_SECURITY_CONFIG_UFSHCR] = NV_FLD_SET_DRF_NUM(MC_SID, STREAMID_SECURITY_CONFIG_UFSHCR,UFSHCR_STREAMID_OVERRIDE, 1 , NV_RESETVAL(MC_SID,STREAMID_SECURITY_CONFIG_UFSHCR));\
    reg.typ[MC_SID_STREAMID_OVERRIDE_CONFIG_UFSHCW] = 0x13;\
    reg.typ[MC_SID_STREAMID_SECURITY_CONFIG_UFSHCW] = NV_FLD_SET_DRF_NUM(MC_SID, STREAMID_SECURITY_CONFIG_UFSHCW,UFSHCW_STREAMID_OVERRIDE, 1 , NV_RESETVAL(MC_SID,STREAMID_SECURITY_CONFIG_UFSHCW));\
    reg.typ[MC_SID_STREAMID_OVERRIDE_CONFIG_NVDISPLAYR] = 0x13;\
    reg.typ[MC_SID_STREAMID_SECURITY_CONFIG_NVDISPLAYR] = NV_FLD_SET_DRF_NUM(MC_SID, STREAMID_SECURITY_CONFIG_NVDISPLAYR,NVDISPLAYR_STREAMID_OVERRIDE, 1 , NV_RESETVAL(MC_SID,STREAMID_SECURITY_CONFIG_NVDISPLAYR));\
    reg.typ[MC_SID_STREAMID_OVERRIDE_CONFIG_BPMPR] = 0x13;\
    reg.typ[MC_SID_STREAMID_SECURITY_CONFIG_BPMPR] = NV_FLD_SET_DRF_NUM(MC_SID, STREAMID_SECURITY_CONFIG_BPMPR,BPMPR_STREAMID_OVERRIDE, 1 , NV_RESETVAL(MC_SID,STREAMID_SECURITY_CONFIG_BPMPR));\
    reg.typ[MC_SID_STREAMID_OVERRIDE_CONFIG_BPMPW] = 0x13;\
    reg.typ[MC_SID_STREAMID_SECURITY_CONFIG_BPMPW] = NV_FLD_SET_DRF_NUM(MC_SID, STREAMID_SECURITY_CONFIG_BPMPW,BPMPW_STREAMID_OVERRIDE, 1 , NV_RESETVAL(MC_SID,STREAMID_SECURITY_CONFIG_BPMPW));\
    reg.typ[MC_SID_STREAMID_OVERRIDE_CONFIG_BPMPDMAR] = 0x13;\
    reg.typ[MC_SID_STREAMID_SECURITY_CONFIG_BPMPDMAR] = NV_FLD_SET_DRF_NUM(MC_SID, STREAMID_SECURITY_CONFIG_BPMPDMAR,BPMPDMAR_STREAMID_OVERRIDE, 1 , NV_RESETVAL(MC_SID,STREAMID_SECURITY_CONFIG_BPMPDMAR));\
    reg.typ[MC_SID_STREAMID_OVERRIDE_CONFIG_BPMPDMAW] = 0x13;\
    reg.typ[MC_SID_STREAMID_SECURITY_CONFIG_BPMPDMAW] = NV_FLD_SET_DRF_NUM(MC_SID, STREAMID_SECURITY_CONFIG_BPMPDMAW,BPMPDMAW_STREAMID_OVERRIDE, 1 , NV_RESETVAL(MC_SID,STREAMID_SECURITY_CONFIG_BPMPDMAW));\
    reg.typ[MC_SID_STREAMID_OVERRIDE_CONFIG_AONR] = 0x13;\
    reg.typ[MC_SID_STREAMID_SECURITY_CONFIG_AONR] = NV_FLD_SET_DRF_NUM(MC_SID, STREAMID_SECURITY_CONFIG_AONR,AONR_STREAMID_OVERRIDE, 1 , NV_RESETVAL(MC_SID,STREAMID_SECURITY_CONFIG_AONR));\
    reg.typ[MC_SID_STREAMID_OVERRIDE_CONFIG_AONW] = 0x13;\
    reg.typ[MC_SID_STREAMID_SECURITY_CONFIG_AONW] = NV_FLD_SET_DRF_NUM(MC_SID, STREAMID_SECURITY_CONFIG_AONW,AONW_STREAMID_OVERRIDE, 1 , NV_RESETVAL(MC_SID,STREAMID_SECURITY_CONFIG_AONW));\
    reg.typ[MC_SID_STREAMID_OVERRIDE_CONFIG_AONDMAR] = 0x13;\
    reg.typ[MC_SID_STREAMID_SECURITY_CONFIG_AONDMAR] = NV_FLD_SET_DRF_NUM(MC_SID, STREAMID_SECURITY_CONFIG_AONDMAR,AONDMAR_STREAMID_OVERRIDE, 1 , NV_RESETVAL(MC_SID,STREAMID_SECURITY_CONFIG_AONDMAR));\
    reg.typ[MC_SID_STREAMID_OVERRIDE_CONFIG_AONDMAW] = 0x13;\
    reg.typ[MC_SID_STREAMID_SECURITY_CONFIG_AONDMAW] = NV_FLD_SET_DRF_NUM(MC_SID, STREAMID_SECURITY_CONFIG_AONDMAW,AONDMAW_STREAMID_OVERRIDE, 1 , NV_RESETVAL(MC_SID,STREAMID_SECURITY_CONFIG_AONDMAW));\
    reg.typ[MC_SID_STREAMID_OVERRIDE_CONFIG_SCER] = 0x13;\
    reg.typ[MC_SID_STREAMID_SECURITY_CONFIG_SCER] = NV_FLD_SET_DRF_NUM(MC_SID, STREAMID_SECURITY_CONFIG_SCER,SCER_STREAMID_OVERRIDE, 1 , NV_RESETVAL(MC_SID,STREAMID_SECURITY_CONFIG_SCER));\
    reg.typ[MC_SID_STREAMID_OVERRIDE_CONFIG_SCEW] = 0x13;\
    reg.typ[MC_SID_STREAMID_SECURITY_CONFIG_SCEW] = NV_FLD_SET_DRF_NUM(MC_SID, STREAMID_SECURITY_CONFIG_SCEW,SCEW_STREAMID_OVERRIDE, 1 , NV_RESETVAL(MC_SID,STREAMID_SECURITY_CONFIG_SCEW));\
    reg.typ[MC_SID_STREAMID_OVERRIDE_CONFIG_SCEDMAR] = 0x13;\
    reg.typ[MC_SID_STREAMID_SECURITY_CONFIG_SCEDMAR] = NV_FLD_SET_DRF_NUM(MC_SID, STREAMID_SECURITY_CONFIG_SCEDMAR,SCEDMAR_STREAMID_OVERRIDE, 1 , NV_RESETVAL(MC_SID,STREAMID_SECURITY_CONFIG_SCEDMAR));\
    reg.typ[MC_SID_STREAMID_OVERRIDE_CONFIG_SCEDMAW] = 0x13;\
    reg.typ[MC_SID_STREAMID_SECURITY_CONFIG_SCEDMAW] = NV_FLD_SET_DRF_NUM(MC_SID, STREAMID_SECURITY_CONFIG_SCEDMAW,SCEDMAW_STREAMID_OVERRIDE, 1 , NV_RESETVAL(MC_SID,STREAMID_SECURITY_CONFIG_SCEDMAW));\
    reg.typ[MC_SID_STREAMID_OVERRIDE_CONFIG_APEDMAR] = 0x13;\
    reg.typ[MC_SID_STREAMID_SECURITY_CONFIG_APEDMAR] = NV_FLD_SET_DRF_NUM(MC_SID, STREAMID_SECURITY_CONFIG_APEDMAR,APEDMAR_STREAMID_OVERRIDE, 1 , NV_RESETVAL(MC_SID,STREAMID_SECURITY_CONFIG_APEDMAR));\
    reg.typ[MC_SID_STREAMID_OVERRIDE_CONFIG_APEDMAW] = 0x13;\
    reg.typ[MC_SID_STREAMID_SECURITY_CONFIG_APEDMAW] = NV_FLD_SET_DRF_NUM(MC_SID, STREAMID_SECURITY_CONFIG_APEDMAW,APEDMAW_STREAMID_OVERRIDE, 1 , NV_RESETVAL(MC_SID,STREAMID_SECURITY_CONFIG_APEDMAW));\
    reg.typ[MC_SID_STREAMID_OVERRIDE_CONFIG_NVDISPLAYR1] = 0x13;\
    reg.typ[MC_SID_STREAMID_SECURITY_CONFIG_NVDISPLAYR1] = NV_FLD_SET_DRF_NUM(MC_SID, STREAMID_SECURITY_CONFIG_NVDISPLAYR1,NVDISPLAYR1_STREAMID_OVERRIDE, 1 , NV_RESETVAL(MC_SID,STREAMID_SECURITY_CONFIG_NVDISPLAYR1));\
    reg.typ[MC_SID_STREAMID_OVERRIDE_CONFIG_VICSRD1] = 0x13;\
    reg.typ[MC_SID_STREAMID_SECURITY_CONFIG_VICSRD1] = NV_FLD_SET_DRF_NUM(MC_SID, STREAMID_SECURITY_CONFIG_VICSRD1,VICSRD1_STREAMID_OVERRIDE, 1 , NV_RESETVAL(MC_SID,STREAMID_SECURITY_CONFIG_VICSRD1));\
    reg.typ[MC_SID_STREAMID_OVERRIDE_CONFIG_NVDECSRD1] = 0x13;\
    reg.typ[MC_SID_STREAMID_SECURITY_CONFIG_NVDECSRD1] = NV_FLD_SET_DRF_NUM(MC_SID, STREAMID_SECURITY_CONFIG_NVDECSRD1,NVDECSRD1_STREAMID_OVERRIDE, 1 , NV_RESETVAL(MC_SID,STREAMID_SECURITY_CONFIG_NVDECSRD1));

#define LIST_INIT_PUSH_MC_SID \
    init.push(MC_SID_STREAMID_OVERRIDE_CONFIG_PTCR, McSidStreamidOverrideConfigPtcr);\
    init.push(MC_SID_STREAMID_SECURITY_CONFIG_PTCR, McSidStreamidSecurityConfigPtcr);\
    init.push(MC_SID_STREAMID_OVERRIDE_CONFIG_AFIR, McSidStreamidOverrideConfigAfir);\
    init.push(MC_SID_STREAMID_SECURITY_CONFIG_AFIR, McSidStreamidSecurityConfigAfir);\
    init.push(MC_SID_STREAMID_OVERRIDE_CONFIG_HDAR, McSidStreamidOverrideConfigHdar);\
    init.push(MC_SID_STREAMID_SECURITY_CONFIG_HDAR, McSidStreamidSecurityConfigHdar);\
    init.push(MC_SID_STREAMID_OVERRIDE_CONFIG_HOST1XDMAR, McSidStreamidOverrideConfigHost1xdmar);\
    init.push(MC_SID_STREAMID_SECURITY_CONFIG_HOST1XDMAR, McSidStreamidSecurityConfigHost1xdmar);\
    init.push(MC_SID_STREAMID_OVERRIDE_CONFIG_NVENCSRD, McSidStreamidOverrideConfigNvencsrd);\
    init.push(MC_SID_STREAMID_SECURITY_CONFIG_NVENCSRD, McSidStreamidSecurityConfigNvencsrd);\
    init.push(MC_SID_STREAMID_OVERRIDE_CONFIG_SATAR, McSidStreamidOverrideConfigSatar);\
    init.push(MC_SID_STREAMID_SECURITY_CONFIG_SATAR, McSidStreamidSecurityConfigSatar);\
    init.push(MC_SID_STREAMID_OVERRIDE_CONFIG_MPCORER, McSidStreamidOverrideConfigMpcorer);\
    init.push(MC_SID_STREAMID_SECURITY_CONFIG_MPCORER, McSidStreamidSecurityConfigMpcorer);\
    init.push(MC_SID_STREAMID_OVERRIDE_CONFIG_NVENCSWR, McSidStreamidOverrideConfigNvencswr);\
    init.push(MC_SID_STREAMID_SECURITY_CONFIG_NVENCSWR, McSidStreamidSecurityConfigNvencswr);\
    init.push(MC_SID_STREAMID_OVERRIDE_CONFIG_AFIW, McSidStreamidOverrideConfigAfiw);\
    init.push(MC_SID_STREAMID_SECURITY_CONFIG_AFIW, McSidStreamidSecurityConfigAfiw);\
    init.push(MC_SID_STREAMID_OVERRIDE_CONFIG_HDAW, McSidStreamidOverrideConfigHdaw);\
    init.push(MC_SID_STREAMID_SECURITY_CONFIG_HDAW, McSidStreamidSecurityConfigHdaw);\
    init.push(MC_SID_STREAMID_OVERRIDE_CONFIG_MPCOREW, McSidStreamidOverrideConfigMpcorew);\
    init.push(MC_SID_STREAMID_SECURITY_CONFIG_MPCOREW, McSidStreamidSecurityConfigMpcorew);\
    init.push(MC_SID_STREAMID_OVERRIDE_CONFIG_SATAW, McSidStreamidOverrideConfigSataw);\
    init.push(MC_SID_STREAMID_SECURITY_CONFIG_SATAW, McSidStreamidSecurityConfigSataw);\
    init.push(MC_SID_STREAMID_OVERRIDE_CONFIG_ISPRA, McSidStreamidOverrideConfigIspra);\
    init.push(MC_SID_STREAMID_SECURITY_CONFIG_ISPRA, McSidStreamidSecurityConfigIspra);\
    init.push(MC_SID_STREAMID_OVERRIDE_CONFIG_ISPWA, McSidStreamidOverrideConfigIspwa);\
    init.push(MC_SID_STREAMID_SECURITY_CONFIG_ISPWA, McSidStreamidSecurityConfigIspwa);\
    init.push(MC_SID_STREAMID_OVERRIDE_CONFIG_ISPWB, McSidStreamidOverrideConfigIspwb);\
    init.push(MC_SID_STREAMID_SECURITY_CONFIG_ISPWB, McSidStreamidSecurityConfigIspwb);\
    init.push(MC_SID_STREAMID_OVERRIDE_CONFIG_XUSB_HOSTR, McSidStreamidOverrideConfigXusb_hostr);\
    init.push(MC_SID_STREAMID_SECURITY_CONFIG_XUSB_HOSTR, McSidStreamidSecurityConfigXusb_hostr);\
    init.push(MC_SID_STREAMID_OVERRIDE_CONFIG_XUSB_HOSTW, McSidStreamidOverrideConfigXusb_hostw);\
    init.push(MC_SID_STREAMID_SECURITY_CONFIG_XUSB_HOSTW, McSidStreamidSecurityConfigXusb_hostw);\
    init.push(MC_SID_STREAMID_OVERRIDE_CONFIG_XUSB_DEVR, McSidStreamidOverrideConfigXusb_devr);\
    init.push(MC_SID_STREAMID_SECURITY_CONFIG_XUSB_DEVR, McSidStreamidSecurityConfigXusb_devr);\
    init.push(MC_SID_STREAMID_OVERRIDE_CONFIG_XUSB_DEVW, McSidStreamidOverrideConfigXusb_devw);\
    init.push(MC_SID_STREAMID_SECURITY_CONFIG_XUSB_DEVW, McSidStreamidSecurityConfigXusb_devw);\
    init.push(MC_SID_STREAMID_OVERRIDE_CONFIG_TSECSRD, McSidStreamidOverrideConfigTsecsrd);\
    init.push(MC_SID_STREAMID_SECURITY_CONFIG_TSECSRD, McSidStreamidSecurityConfigTsecsrd);\
    init.push(MC_SID_STREAMID_OVERRIDE_CONFIG_TSECSWR, McSidStreamidOverrideConfigTsecswr);\
    init.push(MC_SID_STREAMID_SECURITY_CONFIG_TSECSWR, McSidStreamidSecurityConfigTsecswr);\
    init.push(MC_SID_STREAMID_OVERRIDE_CONFIG_GPUSRD, McSidStreamidOverrideConfigGpusrd);\
    init.push(MC_SID_STREAMID_SECURITY_CONFIG_GPUSRD, McSidStreamidSecurityConfigGpusrd);\
    init.push(MC_SID_STREAMID_OVERRIDE_CONFIG_GPUSWR, McSidStreamidOverrideConfigGpuswr);\
    init.push(MC_SID_STREAMID_SECURITY_CONFIG_GPUSWR, McSidStreamidSecurityConfigGpuswr);\
    init.push(MC_SID_STREAMID_OVERRIDE_CONFIG_SDMMCRA, McSidStreamidOverrideConfigSdmmcra);\
    init.push(MC_SID_STREAMID_SECURITY_CONFIG_SDMMCRA, McSidStreamidSecurityConfigSdmmcra);\
    init.push(MC_SID_STREAMID_OVERRIDE_CONFIG_SDMMCRAA, McSidStreamidOverrideConfigSdmmcraa);\
    init.push(MC_SID_STREAMID_SECURITY_CONFIG_SDMMCRAA, McSidStreamidSecurityConfigSdmmcraa);\
    init.push(MC_SID_STREAMID_OVERRIDE_CONFIG_SDMMCR, McSidStreamidOverrideConfigSdmmcr);\
    init.push(MC_SID_STREAMID_SECURITY_CONFIG_SDMMCR, McSidStreamidSecurityConfigSdmmcr);\
    init.push(MC_SID_STREAMID_OVERRIDE_CONFIG_SDMMCRAB, McSidStreamidOverrideConfigSdmmcrab);\
    init.push(MC_SID_STREAMID_SECURITY_CONFIG_SDMMCRAB, McSidStreamidSecurityConfigSdmmcrab);\
    init.push(MC_SID_STREAMID_OVERRIDE_CONFIG_SDMMCWA, McSidStreamidOverrideConfigSdmmcwa);\
    init.push(MC_SID_STREAMID_SECURITY_CONFIG_SDMMCWA, McSidStreamidSecurityConfigSdmmcwa);\
    init.push(MC_SID_STREAMID_OVERRIDE_CONFIG_SDMMCWAA, McSidStreamidOverrideConfigSdmmcwaa);\
    init.push(MC_SID_STREAMID_SECURITY_CONFIG_SDMMCWAA, McSidStreamidSecurityConfigSdmmcwaa);\
    init.push(MC_SID_STREAMID_OVERRIDE_CONFIG_SDMMCW, McSidStreamidOverrideConfigSdmmcw);\
    init.push(MC_SID_STREAMID_SECURITY_CONFIG_SDMMCW, McSidStreamidSecurityConfigSdmmcw);\
    init.push(MC_SID_STREAMID_OVERRIDE_CONFIG_SDMMCWAB, McSidStreamidOverrideConfigSdmmcwab);\
    init.push(MC_SID_STREAMID_SECURITY_CONFIG_SDMMCWAB, McSidStreamidSecurityConfigSdmmcwab);\
    init.push(MC_SID_STREAMID_OVERRIDE_CONFIG_VICSRD, McSidStreamidOverrideConfigVicsrd);\
    init.push(MC_SID_STREAMID_SECURITY_CONFIG_VICSRD, McSidStreamidSecurityConfigVicsrd);\
    init.push(MC_SID_STREAMID_OVERRIDE_CONFIG_VICSWR, McSidStreamidOverrideConfigVicswr);\
    init.push(MC_SID_STREAMID_SECURITY_CONFIG_VICSWR, McSidStreamidSecurityConfigVicswr);\
    init.push(MC_SID_STREAMID_OVERRIDE_CONFIG_VIW, McSidStreamidOverrideConfigViw);\
    init.push(MC_SID_STREAMID_SECURITY_CONFIG_VIW, McSidStreamidSecurityConfigViw);\
    init.push(MC_SID_STREAMID_OVERRIDE_CONFIG_NVDECSRD, McSidStreamidOverrideConfigNvdecsrd);\
    init.push(MC_SID_STREAMID_SECURITY_CONFIG_NVDECSRD, McSidStreamidSecurityConfigNvdecsrd);\
    init.push(MC_SID_STREAMID_OVERRIDE_CONFIG_NVDECSWR, McSidStreamidOverrideConfigNvdecswr);\
    init.push(MC_SID_STREAMID_SECURITY_CONFIG_NVDECSWR, McSidStreamidSecurityConfigNvdecswr);\
    init.push(MC_SID_STREAMID_OVERRIDE_CONFIG_APER, McSidStreamidOverrideConfigAper);\
    init.push(MC_SID_STREAMID_SECURITY_CONFIG_APER, McSidStreamidSecurityConfigAper);\
    init.push(MC_SID_STREAMID_OVERRIDE_CONFIG_APEW, McSidStreamidOverrideConfigApew);\
    init.push(MC_SID_STREAMID_SECURITY_CONFIG_APEW, McSidStreamidSecurityConfigApew);\
    init.push(MC_SID_STREAMID_OVERRIDE_CONFIG_NVJPGSRD, McSidStreamidOverrideConfigNvjpgsrd);\
    init.push(MC_SID_STREAMID_SECURITY_CONFIG_NVJPGSRD, McSidStreamidSecurityConfigNvjpgsrd);\
    init.push(MC_SID_STREAMID_OVERRIDE_CONFIG_NVJPGSWR, McSidStreamidOverrideConfigNvjpgswr);\
    init.push(MC_SID_STREAMID_SECURITY_CONFIG_NVJPGSWR, McSidStreamidSecurityConfigNvjpgswr);\
    init.push(MC_SID_STREAMID_OVERRIDE_CONFIG_SESRD, McSidStreamidOverrideConfigSesrd);\
    init.push(MC_SID_STREAMID_SECURITY_CONFIG_SESRD, McSidStreamidSecurityConfigSesrd);\
    init.push(MC_SID_STREAMID_OVERRIDE_CONFIG_SESWR, McSidStreamidOverrideConfigSeswr);\
    init.push(MC_SID_STREAMID_SECURITY_CONFIG_SESWR, McSidStreamidSecurityConfigSeswr);\
    init.push(MC_SID_STREAMID_OVERRIDE_CONFIG_ETRR, McSidStreamidOverrideConfigEtrr);\
    init.push(MC_SID_STREAMID_SECURITY_CONFIG_ETRR, McSidStreamidSecurityConfigEtrr);\
    init.push(MC_SID_STREAMID_OVERRIDE_CONFIG_ETRW, McSidStreamidOverrideConfigEtrw);\
    init.push(MC_SID_STREAMID_SECURITY_CONFIG_ETRW, McSidStreamidSecurityConfigEtrw);\
    init.push(MC_SID_STREAMID_OVERRIDE_CONFIG_TSECSRDB, McSidStreamidOverrideConfigTsecsrdb);\
    init.push(MC_SID_STREAMID_SECURITY_CONFIG_TSECSRDB, McSidStreamidSecurityConfigTsecsrdb);\
    init.push(MC_SID_STREAMID_OVERRIDE_CONFIG_TSECSWRB, McSidStreamidOverrideConfigTsecswrb);\
    init.push(MC_SID_STREAMID_SECURITY_CONFIG_TSECSWRB, McSidStreamidSecurityConfigTsecswrb);\
    init.push(MC_SID_STREAMID_OVERRIDE_CONFIG_GPUSRD2, McSidStreamidOverrideConfigGpusrd2);\
    init.push(MC_SID_STREAMID_SECURITY_CONFIG_GPUSRD2, McSidStreamidSecurityConfigGpusrd2);\
    init.push(MC_SID_STREAMID_OVERRIDE_CONFIG_GPUSWR2, McSidStreamidOverrideConfigGpuswr2);\
    init.push(MC_SID_STREAMID_SECURITY_CONFIG_GPUSWR2, McSidStreamidSecurityConfigGpuswr2);\
    init.push(MC_SID_STREAMID_OVERRIDE_CONFIG_AXISR, McSidStreamidOverrideConfigAxisr);\
    init.push(MC_SID_STREAMID_SECURITY_CONFIG_AXISR, McSidStreamidSecurityConfigAxisr);\
    init.push(MC_SID_STREAMID_OVERRIDE_CONFIG_AXISW, McSidStreamidOverrideConfigAxisw);\
    init.push(MC_SID_STREAMID_SECURITY_CONFIG_AXISW, McSidStreamidSecurityConfigAxisw);\
    init.push(MC_SID_STREAMID_OVERRIDE_CONFIG_EQOSR, McSidStreamidOverrideConfigEqosr);\
    init.push(MC_SID_STREAMID_SECURITY_CONFIG_EQOSR, McSidStreamidSecurityConfigEqosr);\
    init.push(MC_SID_STREAMID_OVERRIDE_CONFIG_EQOSW, McSidStreamidOverrideConfigEqosw);\
    init.push(MC_SID_STREAMID_SECURITY_CONFIG_EQOSW, McSidStreamidSecurityConfigEqosw);\
    init.push(MC_SID_STREAMID_OVERRIDE_CONFIG_UFSHCR, McSidStreamidOverrideConfigUfshcr);\
    init.push(MC_SID_STREAMID_SECURITY_CONFIG_UFSHCR, McSidStreamidSecurityConfigUfshcr);\
    init.push(MC_SID_STREAMID_OVERRIDE_CONFIG_UFSHCW, McSidStreamidOverrideConfigUfshcw);\
    init.push(MC_SID_STREAMID_SECURITY_CONFIG_UFSHCW, McSidStreamidSecurityConfigUfshcw);\
    init.push(MC_SID_STREAMID_OVERRIDE_CONFIG_NVDISPLAYR, McSidStreamidOverrideConfigNvdisplayr);\
    init.push(MC_SID_STREAMID_SECURITY_CONFIG_NVDISPLAYR, McSidStreamidSecurityConfigNvdisplayr);\
    init.push(MC_SID_STREAMID_OVERRIDE_CONFIG_BPMPR, McSidStreamidOverrideConfigBpmpr);\
    init.push(MC_SID_STREAMID_SECURITY_CONFIG_BPMPR, McSidStreamidSecurityConfigBpmpr);\
    init.push(MC_SID_STREAMID_OVERRIDE_CONFIG_BPMPW, McSidStreamidOverrideConfigBpmpw);\
    init.push(MC_SID_STREAMID_SECURITY_CONFIG_BPMPW, McSidStreamidSecurityConfigBpmpw);\
    init.push(MC_SID_STREAMID_OVERRIDE_CONFIG_BPMPDMAR, McSidStreamidOverrideConfigBpmpdmar);\
    init.push(MC_SID_STREAMID_SECURITY_CONFIG_BPMPDMAR, McSidStreamidSecurityConfigBpmpdmar);\
    init.push(MC_SID_STREAMID_OVERRIDE_CONFIG_BPMPDMAW, McSidStreamidOverrideConfigBpmpdmaw);\
    init.push(MC_SID_STREAMID_SECURITY_CONFIG_BPMPDMAW, McSidStreamidSecurityConfigBpmpdmaw);\
    init.push(MC_SID_STREAMID_OVERRIDE_CONFIG_AONR, McSidStreamidOverrideConfigAonr);\
    init.push(MC_SID_STREAMID_SECURITY_CONFIG_AONR, McSidStreamidSecurityConfigAonr);\
    init.push(MC_SID_STREAMID_OVERRIDE_CONFIG_AONW, McSidStreamidOverrideConfigAonw);\
    init.push(MC_SID_STREAMID_SECURITY_CONFIG_AONW, McSidStreamidSecurityConfigAonw);\
    init.push(MC_SID_STREAMID_OVERRIDE_CONFIG_AONDMAR, McSidStreamidOverrideConfigAondmar);\
    init.push(MC_SID_STREAMID_SECURITY_CONFIG_AONDMAR, McSidStreamidSecurityConfigAondmar);\
    init.push(MC_SID_STREAMID_OVERRIDE_CONFIG_AONDMAW, McSidStreamidOverrideConfigAondmaw);\
    init.push(MC_SID_STREAMID_SECURITY_CONFIG_AONDMAW, McSidStreamidSecurityConfigAondmaw);\
    init.push(MC_SID_STREAMID_OVERRIDE_CONFIG_SCER, McSidStreamidOverrideConfigScer);\
    init.push(MC_SID_STREAMID_SECURITY_CONFIG_SCER, McSidStreamidSecurityConfigScer);\
    init.push(MC_SID_STREAMID_OVERRIDE_CONFIG_SCEW, McSidStreamidOverrideConfigScew);\
    init.push(MC_SID_STREAMID_SECURITY_CONFIG_SCEW, McSidStreamidSecurityConfigScew);\
    init.push(MC_SID_STREAMID_OVERRIDE_CONFIG_SCEDMAR, McSidStreamidOverrideConfigScedmar);\
    init.push(MC_SID_STREAMID_SECURITY_CONFIG_SCEDMAR, McSidStreamidSecurityConfigScedmar);\
    init.push(MC_SID_STREAMID_OVERRIDE_CONFIG_SCEDMAW, McSidStreamidOverrideConfigScedmaw);\
    init.push(MC_SID_STREAMID_SECURITY_CONFIG_SCEDMAW, McSidStreamidSecurityConfigScedmaw);\
    init.push(MC_SID_STREAMID_OVERRIDE_CONFIG_APEDMAR, McSidStreamidOverrideConfigApedmar);\
    init.push(MC_SID_STREAMID_SECURITY_CONFIG_APEDMAR, McSidStreamidSecurityConfigApedmar);\
    init.push(MC_SID_STREAMID_OVERRIDE_CONFIG_APEDMAW, McSidStreamidOverrideConfigApedmaw);\
    init.push(MC_SID_STREAMID_SECURITY_CONFIG_APEDMAW, McSidStreamidSecurityConfigApedmaw);\
    init.push(MC_SID_STREAMID_OVERRIDE_CONFIG_NVDISPLAYR1, McSidStreamidOverrideConfigNvdisplayr1);\
    init.push(MC_SID_STREAMID_SECURITY_CONFIG_NVDISPLAYR1, McSidStreamidSecurityConfigNvdisplayr1);\
    init.push(MC_SID_STREAMID_OVERRIDE_CONFIG_VICSRD1, McSidStreamidOverrideConfigVicsrd1);\
    init.push(MC_SID_STREAMID_SECURITY_CONFIG_VICSRD1, McSidStreamidSecurityConfigVicsrd1);\
    init.push(MC_SID_STREAMID_OVERRIDE_CONFIG_NVDECSRD1, McSidStreamidOverrideConfigNvdecsrd1);\
    init.push(MC_SID_STREAMID_SECURITY_CONFIG_NVDECSRD1, McSidStreamidSecurityConfigNvdecsrd1);

#define LIST_HW_REGW_MC_SID \
    HW_REGW(McSidBase, MC_SID, STREAMID_OVERRIDE_CONFIG_PTCR, pData->McSidStreamidOverrideConfigPtcr);\
    HW_REGW(McSidBase, MC_SID, STREAMID_SECURITY_CONFIG_PTCR, pData->McSidStreamidSecurityConfigPtcr);\
    HW_REGW(McSidBase, MC_SID, STREAMID_OVERRIDE_CONFIG_AFIR, pData->McSidStreamidOverrideConfigAfir);\
    HW_REGW(McSidBase, MC_SID, STREAMID_SECURITY_CONFIG_AFIR, pData->McSidStreamidSecurityConfigAfir);\
    HW_REGW(McSidBase, MC_SID, STREAMID_OVERRIDE_CONFIG_HDAR, pData->McSidStreamidOverrideConfigHdar);\
    HW_REGW(McSidBase, MC_SID, STREAMID_SECURITY_CONFIG_HDAR, pData->McSidStreamidSecurityConfigHdar);\
    HW_REGW(McSidBase, MC_SID, STREAMID_OVERRIDE_CONFIG_HOST1XDMAR, pData->McSidStreamidOverrideConfigHost1xdmar);\
    HW_REGW(McSidBase, MC_SID, STREAMID_SECURITY_CONFIG_HOST1XDMAR, pData->McSidStreamidSecurityConfigHost1xdmar);\
    HW_REGW(McSidBase, MC_SID, STREAMID_OVERRIDE_CONFIG_NVENCSRD, pData->McSidStreamidOverrideConfigNvencsrd);\
    HW_REGW(McSidBase, MC_SID, STREAMID_SECURITY_CONFIG_NVENCSRD, pData->McSidStreamidSecurityConfigNvencsrd);\
    HW_REGW(McSidBase, MC_SID, STREAMID_OVERRIDE_CONFIG_SATAR, pData->McSidStreamidOverrideConfigSatar);\
    HW_REGW(McSidBase, MC_SID, STREAMID_SECURITY_CONFIG_SATAR, pData->McSidStreamidSecurityConfigSatar);\
    HW_REGW(McSidBase, MC_SID, STREAMID_OVERRIDE_CONFIG_MPCORER, pData->McSidStreamidOverrideConfigMpcorer);\
    HW_REGW(McSidBase, MC_SID, STREAMID_SECURITY_CONFIG_MPCORER, pData->McSidStreamidSecurityConfigMpcorer);\
    HW_REGW(McSidBase, MC_SID, STREAMID_OVERRIDE_CONFIG_NVENCSWR, pData->McSidStreamidOverrideConfigNvencswr);\
    HW_REGW(McSidBase, MC_SID, STREAMID_SECURITY_CONFIG_NVENCSWR, pData->McSidStreamidSecurityConfigNvencswr);\
    HW_REGW(McSidBase, MC_SID, STREAMID_OVERRIDE_CONFIG_AFIW, pData->McSidStreamidOverrideConfigAfiw);\
    HW_REGW(McSidBase, MC_SID, STREAMID_SECURITY_CONFIG_AFIW, pData->McSidStreamidSecurityConfigAfiw);\
    HW_REGW(McSidBase, MC_SID, STREAMID_OVERRIDE_CONFIG_HDAW, pData->McSidStreamidOverrideConfigHdaw);\
    HW_REGW(McSidBase, MC_SID, STREAMID_SECURITY_CONFIG_HDAW, pData->McSidStreamidSecurityConfigHdaw);\
    HW_REGW(McSidBase, MC_SID, STREAMID_OVERRIDE_CONFIG_MPCOREW, pData->McSidStreamidOverrideConfigMpcorew);\
    HW_REGW(McSidBase, MC_SID, STREAMID_SECURITY_CONFIG_MPCOREW, pData->McSidStreamidSecurityConfigMpcorew);\
    HW_REGW(McSidBase, MC_SID, STREAMID_OVERRIDE_CONFIG_SATAW, pData->McSidStreamidOverrideConfigSataw);\
    HW_REGW(McSidBase, MC_SID, STREAMID_SECURITY_CONFIG_SATAW, pData->McSidStreamidSecurityConfigSataw);\
    HW_REGW(McSidBase, MC_SID, STREAMID_OVERRIDE_CONFIG_ISPRA, pData->McSidStreamidOverrideConfigIspra);\
    HW_REGW(McSidBase, MC_SID, STREAMID_SECURITY_CONFIG_ISPRA, pData->McSidStreamidSecurityConfigIspra);\
    HW_REGW(McSidBase, MC_SID, STREAMID_OVERRIDE_CONFIG_ISPWA, pData->McSidStreamidOverrideConfigIspwa);\
    HW_REGW(McSidBase, MC_SID, STREAMID_SECURITY_CONFIG_ISPWA, pData->McSidStreamidSecurityConfigIspwa);\
    HW_REGW(McSidBase, MC_SID, STREAMID_OVERRIDE_CONFIG_ISPWB, pData->McSidStreamidOverrideConfigIspwb);\
    HW_REGW(McSidBase, MC_SID, STREAMID_SECURITY_CONFIG_ISPWB, pData->McSidStreamidSecurityConfigIspwb);\
    HW_REGW(McSidBase, MC_SID, STREAMID_OVERRIDE_CONFIG_XUSB_HOSTR, pData->McSidStreamidOverrideConfigXusb_hostr);\
    HW_REGW(McSidBase, MC_SID, STREAMID_SECURITY_CONFIG_XUSB_HOSTR, pData->McSidStreamidSecurityConfigXusb_hostr);\
    HW_REGW(McSidBase, MC_SID, STREAMID_OVERRIDE_CONFIG_XUSB_HOSTW, pData->McSidStreamidOverrideConfigXusb_hostw);\
    HW_REGW(McSidBase, MC_SID, STREAMID_SECURITY_CONFIG_XUSB_HOSTW, pData->McSidStreamidSecurityConfigXusb_hostw);\
    HW_REGW(McSidBase, MC_SID, STREAMID_OVERRIDE_CONFIG_XUSB_DEVR, pData->McSidStreamidOverrideConfigXusb_devr);\
    HW_REGW(McSidBase, MC_SID, STREAMID_SECURITY_CONFIG_XUSB_DEVR, pData->McSidStreamidSecurityConfigXusb_devr);\
    HW_REGW(McSidBase, MC_SID, STREAMID_OVERRIDE_CONFIG_XUSB_DEVW, pData->McSidStreamidOverrideConfigXusb_devw);\
    HW_REGW(McSidBase, MC_SID, STREAMID_SECURITY_CONFIG_XUSB_DEVW, pData->McSidStreamidSecurityConfigXusb_devw);\
    HW_REGW(McSidBase, MC_SID, STREAMID_OVERRIDE_CONFIG_TSECSRD, pData->McSidStreamidOverrideConfigTsecsrd);\
    HW_REGW(McSidBase, MC_SID, STREAMID_SECURITY_CONFIG_TSECSRD, pData->McSidStreamidSecurityConfigTsecsrd);\
    HW_REGW(McSidBase, MC_SID, STREAMID_OVERRIDE_CONFIG_TSECSWR, pData->McSidStreamidOverrideConfigTsecswr);\
    HW_REGW(McSidBase, MC_SID, STREAMID_SECURITY_CONFIG_TSECSWR, pData->McSidStreamidSecurityConfigTsecswr);\
    HW_REGW(McSidBase, MC_SID, STREAMID_OVERRIDE_CONFIG_GPUSRD, pData->McSidStreamidOverrideConfigGpusrd);\
    HW_REGW(McSidBase, MC_SID, STREAMID_SECURITY_CONFIG_GPUSRD, pData->McSidStreamidSecurityConfigGpusrd);\
    HW_REGW(McSidBase, MC_SID, STREAMID_OVERRIDE_CONFIG_GPUSWR, pData->McSidStreamidOverrideConfigGpuswr);\
    HW_REGW(McSidBase, MC_SID, STREAMID_SECURITY_CONFIG_GPUSWR, pData->McSidStreamidSecurityConfigGpuswr);\
    HW_REGW(McSidBase, MC_SID, STREAMID_OVERRIDE_CONFIG_SDMMCRA, pData->McSidStreamidOverrideConfigSdmmcra);\
    HW_REGW(McSidBase, MC_SID, STREAMID_SECURITY_CONFIG_SDMMCRA, pData->McSidStreamidSecurityConfigSdmmcra);\
    HW_REGW(McSidBase, MC_SID, STREAMID_OVERRIDE_CONFIG_SDMMCRAA, pData->McSidStreamidOverrideConfigSdmmcraa);\
    HW_REGW(McSidBase, MC_SID, STREAMID_SECURITY_CONFIG_SDMMCRAA, pData->McSidStreamidSecurityConfigSdmmcraa);\
    HW_REGW(McSidBase, MC_SID, STREAMID_OVERRIDE_CONFIG_SDMMCR, pData->McSidStreamidOverrideConfigSdmmcr);\
    HW_REGW(McSidBase, MC_SID, STREAMID_SECURITY_CONFIG_SDMMCR, pData->McSidStreamidSecurityConfigSdmmcr);\
    HW_REGW(McSidBase, MC_SID, STREAMID_OVERRIDE_CONFIG_SDMMCRAB, pData->McSidStreamidOverrideConfigSdmmcrab);\
    HW_REGW(McSidBase, MC_SID, STREAMID_SECURITY_CONFIG_SDMMCRAB, pData->McSidStreamidSecurityConfigSdmmcrab);\
    HW_REGW(McSidBase, MC_SID, STREAMID_OVERRIDE_CONFIG_SDMMCWA, pData->McSidStreamidOverrideConfigSdmmcwa);\
    HW_REGW(McSidBase, MC_SID, STREAMID_SECURITY_CONFIG_SDMMCWA, pData->McSidStreamidSecurityConfigSdmmcwa);\
    HW_REGW(McSidBase, MC_SID, STREAMID_OVERRIDE_CONFIG_SDMMCWAA, pData->McSidStreamidOverrideConfigSdmmcwaa);\
    HW_REGW(McSidBase, MC_SID, STREAMID_SECURITY_CONFIG_SDMMCWAA, pData->McSidStreamidSecurityConfigSdmmcwaa);\
    HW_REGW(McSidBase, MC_SID, STREAMID_OVERRIDE_CONFIG_SDMMCW, pData->McSidStreamidOverrideConfigSdmmcw);\
    HW_REGW(McSidBase, MC_SID, STREAMID_SECURITY_CONFIG_SDMMCW, pData->McSidStreamidSecurityConfigSdmmcw);\
    HW_REGW(McSidBase, MC_SID, STREAMID_OVERRIDE_CONFIG_SDMMCWAB, pData->McSidStreamidOverrideConfigSdmmcwab);\
    HW_REGW(McSidBase, MC_SID, STREAMID_SECURITY_CONFIG_SDMMCWAB, pData->McSidStreamidSecurityConfigSdmmcwab);\
    HW_REGW(McSidBase, MC_SID, STREAMID_OVERRIDE_CONFIG_VICSRD, pData->McSidStreamidOverrideConfigVicsrd);\
    HW_REGW(McSidBase, MC_SID, STREAMID_SECURITY_CONFIG_VICSRD, pData->McSidStreamidSecurityConfigVicsrd);\
    HW_REGW(McSidBase, MC_SID, STREAMID_OVERRIDE_CONFIG_VICSWR, pData->McSidStreamidOverrideConfigVicswr);\
    HW_REGW(McSidBase, MC_SID, STREAMID_SECURITY_CONFIG_VICSWR, pData->McSidStreamidSecurityConfigVicswr);\
    HW_REGW(McSidBase, MC_SID, STREAMID_OVERRIDE_CONFIG_VIW, pData->McSidStreamidOverrideConfigViw);\
    HW_REGW(McSidBase, MC_SID, STREAMID_SECURITY_CONFIG_VIW, pData->McSidStreamidSecurityConfigViw);\
    HW_REGW(McSidBase, MC_SID, STREAMID_OVERRIDE_CONFIG_NVDECSRD, pData->McSidStreamidOverrideConfigNvdecsrd);\
    HW_REGW(McSidBase, MC_SID, STREAMID_SECURITY_CONFIG_NVDECSRD, pData->McSidStreamidSecurityConfigNvdecsrd);\
    HW_REGW(McSidBase, MC_SID, STREAMID_OVERRIDE_CONFIG_NVDECSWR, pData->McSidStreamidOverrideConfigNvdecswr);\
    HW_REGW(McSidBase, MC_SID, STREAMID_SECURITY_CONFIG_NVDECSWR, pData->McSidStreamidSecurityConfigNvdecswr);\
    HW_REGW(McSidBase, MC_SID, STREAMID_OVERRIDE_CONFIG_APER, pData->McSidStreamidOverrideConfigAper);\
    HW_REGW(McSidBase, MC_SID, STREAMID_SECURITY_CONFIG_APER, pData->McSidStreamidSecurityConfigAper);\
    HW_REGW(McSidBase, MC_SID, STREAMID_OVERRIDE_CONFIG_APEW, pData->McSidStreamidOverrideConfigApew);\
    HW_REGW(McSidBase, MC_SID, STREAMID_SECURITY_CONFIG_APEW, pData->McSidStreamidSecurityConfigApew);\
    HW_REGW(McSidBase, MC_SID, STREAMID_OVERRIDE_CONFIG_NVJPGSRD, pData->McSidStreamidOverrideConfigNvjpgsrd);\
    HW_REGW(McSidBase, MC_SID, STREAMID_SECURITY_CONFIG_NVJPGSRD, pData->McSidStreamidSecurityConfigNvjpgsrd);\
    HW_REGW(McSidBase, MC_SID, STREAMID_OVERRIDE_CONFIG_NVJPGSWR, pData->McSidStreamidOverrideConfigNvjpgswr);\
    HW_REGW(McSidBase, MC_SID, STREAMID_SECURITY_CONFIG_NVJPGSWR, pData->McSidStreamidSecurityConfigNvjpgswr);\
    HW_REGW(McSidBase, MC_SID, STREAMID_OVERRIDE_CONFIG_SESRD, pData->McSidStreamidOverrideConfigSesrd);\
    HW_REGW(McSidBase, MC_SID, STREAMID_SECURITY_CONFIG_SESRD, pData->McSidStreamidSecurityConfigSesrd);\
    HW_REGW(McSidBase, MC_SID, STREAMID_OVERRIDE_CONFIG_SESWR, pData->McSidStreamidOverrideConfigSeswr);\
    HW_REGW(McSidBase, MC_SID, STREAMID_SECURITY_CONFIG_SESWR, pData->McSidStreamidSecurityConfigSeswr);\
    HW_REGW(McSidBase, MC_SID, STREAMID_OVERRIDE_CONFIG_ETRR, pData->McSidStreamidOverrideConfigEtrr);\
    HW_REGW(McSidBase, MC_SID, STREAMID_SECURITY_CONFIG_ETRR, pData->McSidStreamidSecurityConfigEtrr);\
    HW_REGW(McSidBase, MC_SID, STREAMID_OVERRIDE_CONFIG_ETRW, pData->McSidStreamidOverrideConfigEtrw);\
    HW_REGW(McSidBase, MC_SID, STREAMID_SECURITY_CONFIG_ETRW, pData->McSidStreamidSecurityConfigEtrw);\
    HW_REGW(McSidBase, MC_SID, STREAMID_OVERRIDE_CONFIG_TSECSRDB, pData->McSidStreamidOverrideConfigTsecsrdb);\
    HW_REGW(McSidBase, MC_SID, STREAMID_SECURITY_CONFIG_TSECSRDB, pData->McSidStreamidSecurityConfigTsecsrdb);\
    HW_REGW(McSidBase, MC_SID, STREAMID_OVERRIDE_CONFIG_TSECSWRB, pData->McSidStreamidOverrideConfigTsecswrb);\
    HW_REGW(McSidBase, MC_SID, STREAMID_SECURITY_CONFIG_TSECSWRB, pData->McSidStreamidSecurityConfigTsecswrb);\
    HW_REGW(McSidBase, MC_SID, STREAMID_OVERRIDE_CONFIG_GPUSRD2, pData->McSidStreamidOverrideConfigGpusrd2);\
    HW_REGW(McSidBase, MC_SID, STREAMID_SECURITY_CONFIG_GPUSRD2, pData->McSidStreamidSecurityConfigGpusrd2);\
    HW_REGW(McSidBase, MC_SID, STREAMID_OVERRIDE_CONFIG_GPUSWR2, pData->McSidStreamidOverrideConfigGpuswr2);\
    HW_REGW(McSidBase, MC_SID, STREAMID_SECURITY_CONFIG_GPUSWR2, pData->McSidStreamidSecurityConfigGpuswr2);\
    HW_REGW(McSidBase, MC_SID, STREAMID_OVERRIDE_CONFIG_AXISR, pData->McSidStreamidOverrideConfigAxisr);\
    HW_REGW(McSidBase, MC_SID, STREAMID_SECURITY_CONFIG_AXISR, pData->McSidStreamidSecurityConfigAxisr);\
    HW_REGW(McSidBase, MC_SID, STREAMID_OVERRIDE_CONFIG_AXISW, pData->McSidStreamidOverrideConfigAxisw);\
    HW_REGW(McSidBase, MC_SID, STREAMID_SECURITY_CONFIG_AXISW, pData->McSidStreamidSecurityConfigAxisw);\
    HW_REGW(McSidBase, MC_SID, STREAMID_OVERRIDE_CONFIG_EQOSR, pData->McSidStreamidOverrideConfigEqosr);\
    HW_REGW(McSidBase, MC_SID, STREAMID_SECURITY_CONFIG_EQOSR, pData->McSidStreamidSecurityConfigEqosr);\
    HW_REGW(McSidBase, MC_SID, STREAMID_OVERRIDE_CONFIG_EQOSW, pData->McSidStreamidOverrideConfigEqosw);\
    HW_REGW(McSidBase, MC_SID, STREAMID_SECURITY_CONFIG_EQOSW, pData->McSidStreamidSecurityConfigEqosw);\
    HW_REGW(McSidBase, MC_SID, STREAMID_OVERRIDE_CONFIG_UFSHCR, pData->McSidStreamidOverrideConfigUfshcr);\
    HW_REGW(McSidBase, MC_SID, STREAMID_SECURITY_CONFIG_UFSHCR, pData->McSidStreamidSecurityConfigUfshcr);\
    HW_REGW(McSidBase, MC_SID, STREAMID_OVERRIDE_CONFIG_UFSHCW, pData->McSidStreamidOverrideConfigUfshcw);\
    HW_REGW(McSidBase, MC_SID, STREAMID_SECURITY_CONFIG_UFSHCW, pData->McSidStreamidSecurityConfigUfshcw);\
    HW_REGW(McSidBase, MC_SID, STREAMID_OVERRIDE_CONFIG_NVDISPLAYR, pData->McSidStreamidOverrideConfigNvdisplayr);\
    HW_REGW(McSidBase, MC_SID, STREAMID_SECURITY_CONFIG_NVDISPLAYR, pData->McSidStreamidSecurityConfigNvdisplayr);\
    HW_REGW(McSidBase, MC_SID, STREAMID_OVERRIDE_CONFIG_BPMPR, pData->McSidStreamidOverrideConfigBpmpr);\
    HW_REGW(McSidBase, MC_SID, STREAMID_SECURITY_CONFIG_BPMPR, pData->McSidStreamidSecurityConfigBpmpr);\
    HW_REGW(McSidBase, MC_SID, STREAMID_OVERRIDE_CONFIG_BPMPW, pData->McSidStreamidOverrideConfigBpmpw);\
    HW_REGW(McSidBase, MC_SID, STREAMID_SECURITY_CONFIG_BPMPW, pData->McSidStreamidSecurityConfigBpmpw);\
    HW_REGW(McSidBase, MC_SID, STREAMID_OVERRIDE_CONFIG_BPMPDMAR, pData->McSidStreamidOverrideConfigBpmpdmar);\
    HW_REGW(McSidBase, MC_SID, STREAMID_SECURITY_CONFIG_BPMPDMAR, pData->McSidStreamidSecurityConfigBpmpdmar);\
    HW_REGW(McSidBase, MC_SID, STREAMID_OVERRIDE_CONFIG_BPMPDMAW, pData->McSidStreamidOverrideConfigBpmpdmaw);\
    HW_REGW(McSidBase, MC_SID, STREAMID_SECURITY_CONFIG_BPMPDMAW, pData->McSidStreamidSecurityConfigBpmpdmaw);\
    HW_REGW(McSidBase, MC_SID, STREAMID_OVERRIDE_CONFIG_AONR, pData->McSidStreamidOverrideConfigAonr);\
    HW_REGW(McSidBase, MC_SID, STREAMID_SECURITY_CONFIG_AONR, pData->McSidStreamidSecurityConfigAonr);\
    HW_REGW(McSidBase, MC_SID, STREAMID_OVERRIDE_CONFIG_AONW, pData->McSidStreamidOverrideConfigAonw);\
    HW_REGW(McSidBase, MC_SID, STREAMID_SECURITY_CONFIG_AONW, pData->McSidStreamidSecurityConfigAonw);\
    HW_REGW(McSidBase, MC_SID, STREAMID_OVERRIDE_CONFIG_AONDMAR, pData->McSidStreamidOverrideConfigAondmar);\
    HW_REGW(McSidBase, MC_SID, STREAMID_SECURITY_CONFIG_AONDMAR, pData->McSidStreamidSecurityConfigAondmar);\
    HW_REGW(McSidBase, MC_SID, STREAMID_OVERRIDE_CONFIG_AONDMAW, pData->McSidStreamidOverrideConfigAondmaw);\
    HW_REGW(McSidBase, MC_SID, STREAMID_SECURITY_CONFIG_AONDMAW, pData->McSidStreamidSecurityConfigAondmaw);\
    HW_REGW(McSidBase, MC_SID, STREAMID_OVERRIDE_CONFIG_SCER, pData->McSidStreamidOverrideConfigScer);\
    HW_REGW(McSidBase, MC_SID, STREAMID_SECURITY_CONFIG_SCER, pData->McSidStreamidSecurityConfigScer);\
    HW_REGW(McSidBase, MC_SID, STREAMID_OVERRIDE_CONFIG_SCEW, pData->McSidStreamidOverrideConfigScew);\
    HW_REGW(McSidBase, MC_SID, STREAMID_SECURITY_CONFIG_SCEW, pData->McSidStreamidSecurityConfigScew);\
    HW_REGW(McSidBase, MC_SID, STREAMID_OVERRIDE_CONFIG_SCEDMAR, pData->McSidStreamidOverrideConfigScedmar);\
    HW_REGW(McSidBase, MC_SID, STREAMID_SECURITY_CONFIG_SCEDMAR, pData->McSidStreamidSecurityConfigScedmar);\
    HW_REGW(McSidBase, MC_SID, STREAMID_OVERRIDE_CONFIG_SCEDMAW, pData->McSidStreamidOverrideConfigScedmaw);\
    HW_REGW(McSidBase, MC_SID, STREAMID_SECURITY_CONFIG_SCEDMAW, pData->McSidStreamidSecurityConfigScedmaw);\
    HW_REGW(McSidBase, MC_SID, STREAMID_OVERRIDE_CONFIG_APEDMAR, pData->McSidStreamidOverrideConfigApedmar);\
    HW_REGW(McSidBase, MC_SID, STREAMID_SECURITY_CONFIG_APEDMAR, pData->McSidStreamidSecurityConfigApedmar);\
    HW_REGW(McSidBase, MC_SID, STREAMID_OVERRIDE_CONFIG_APEDMAW, pData->McSidStreamidOverrideConfigApedmaw);\
    HW_REGW(McSidBase, MC_SID, STREAMID_SECURITY_CONFIG_APEDMAW, pData->McSidStreamidSecurityConfigApedmaw);\
    HW_REGW(McSidBase, MC_SID, STREAMID_OVERRIDE_CONFIG_NVDISPLAYR1, pData->McSidStreamidOverrideConfigNvdisplayr1);\
    HW_REGW(McSidBase, MC_SID, STREAMID_SECURITY_CONFIG_NVDISPLAYR1, pData->McSidStreamidSecurityConfigNvdisplayr1);\
    HW_REGW(McSidBase, MC_SID, STREAMID_OVERRIDE_CONFIG_VICSRD1, pData->McSidStreamidOverrideConfigVicsrd1);\
    HW_REGW(McSidBase, MC_SID, STREAMID_SECURITY_CONFIG_VICSRD1, pData->McSidStreamidSecurityConfigVicsrd1);\
    HW_REGW(McSidBase, MC_SID, STREAMID_OVERRIDE_CONFIG_NVDECSRD1, pData->McSidStreamidOverrideConfigNvdecsrd1);\
    HW_REGW(McSidBase, MC_SID, STREAMID_SECURITY_CONFIG_NVDECSRD1, pData->McSidStreamidSecurityConfigNvdecsrd1);

#endif

