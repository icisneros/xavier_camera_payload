// WARNING!!! THIS HEADER INCLUDES SOFTWARE METHODS!!!
// ********** DO NOT USE IN HW TREE.  **********
/*
 * Copyright (c) 1993-2019, NVIDIA CORPORATION.  All rights reserved.
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

#ifndef _clb1b6_h_
#define _clb1b6_h_

#ifdef __cplusplus
extern "C" {
#endif

#define NVB1B6_VIDEO_COMPOSITOR                                             (0x0000B1B6)

typedef volatile struct {
    NvU32 Reserved00[0x40];
    NvU32 Nop;                                                                  // 0x00000100 - 0x00000103
    NvU32 Reserved01[0xF];
    NvU32 PmTrigger;                                                            // 0x00000140 - 0x00000143
    NvU32 Reserved02[0x2F];
    NvU32 SetApplicationID;                                                     // 0x00000200 - 0x00000203
    NvU32 SetWatchdogTimer;                                                     // 0x00000204 - 0x00000207
    NvU32 Reserved03[0xE];
    NvU32 SemaphoreA;                                                           // 0x00000240 - 0x00000243
    NvU32 SemaphoreB;                                                           // 0x00000244 - 0x00000247
    NvU32 SemaphoreC;                                                           // 0x00000248 - 0x0000024B
    NvU32 CtxSaveArea;                                                          // 0x0000024C - 0x0000024F
    NvU32 CtxSwitch;                                                            // 0x00000250 - 0x00000253
    NvU32 Reserved04[0x2B];
    NvU32 Execute;                                                              // 0x00000300 - 0x00000303
    NvU32 SemaphoreD;                                                           // 0x00000304 - 0x00000307
    NvU32 Reserved05[0xFE];
    NvU32 SetPictureIndex;                                                      // 0x00000700 - 0x00000703
    NvU32 SetControlParams;                                                     // 0x00000704 - 0x00000707
    NvU32 SetConfigStructOffset;                                                // 0x00000708 - 0x0000070B
    NvU32 SetFilterStructOffset;                                                // 0x0000070C - 0x0000070F
    NvU32 SetPaletteOffset;                                                     // 0x00000710 - 0x00000713
    NvU32 SetHistOffset;                                                        // 0x00000714 - 0x00000717
    NvU32 SetContextId;                                                         // 0x00000718 - 0x0000071B
    NvU32 SetFceUcodeSize;                                                      // 0x0000071C - 0x0000071F
    NvU32 SetOutputSurfaceLumaOffset;                                           // 0x00000720 - 0x00000723
    NvU32 SetOutputSurfaceChromaU_Offset;                                       // 0x00000724 - 0x00000727
    NvU32 SetOutputSurfaceChromaV_Offset;                                       // 0x00000728 - 0x0000072B
    NvU32 SetFceUcodeOffset;                                                    // 0x0000072C - 0x0000072F
    NvU32 SetCrcStructOffset;                                                   // 0x00000730 - 0x00000733
    NvU32 SetCrcMode;                                                           // 0x00000734 - 0x00000737
    NvU32 SetStatusOffset;                                                      // 0x00000738 - 0x0000073B
    NvU32 Reserved06[0x1];
    NvU32 SetSlotContextId[16];                                                 // 0x00000740 - 0x0000077F
    NvU32 SetHistoryBufferOffset[16];                                           // 0x00000780 - 0x000007BF
    NvU32 SetCompTagBuffer_Offset[16];                                          // 0x000007C0 - 0x000007FF
    NvU32 Reserved07[0x245];
    NvU32 PmTriggerEnd;                                                         // 0x00001114 - 0x00001117
    NvU32 Reserved08[0x3A];
    NvU32 SetSurface0LumaOffset[16];                                            // 0x00001200 - 0x0000123F
    NvU32 Reserved09[0xF];
    NvU32 SetSurface0ChromaU_Offset[16];                                        // 0x00001204 - 0x00001243
    NvU32 Reserved10[0xF];
    NvU32 SetSurface0ChromaV_Offset[16];                                        // 0x00001208 - 0x00001247
    NvU32 Reserved11[0xF];
    NvU32 SetSurface1LumaOffset[16];                                            // 0x0000120C - 0x0000124B
    NvU32 Reserved12[0xF];
    NvU32 SetSurface1ChromaU_Offset[16];                                        // 0x00001210 - 0x0000124F
    NvU32 Reserved13[0xF];
    NvU32 SetSurface1ChromaV_Offset[16];                                        // 0x00001214 - 0x00001253
    NvU32 Reserved14[0xF];
    NvU32 SetSurface2LumaOffset[16];                                            // 0x00001218 - 0x00001257
    NvU32 Reserved15[0xF];
    NvU32 SetSurface2ChromaU_Offset[16];                                        // 0x0000121C - 0x0000125B
    NvU32 Reserved16[0xF];
    NvU32 SetSurface2ChromaV_Offset[16];                                        // 0x00001220 - 0x0000125F
    NvU32 Reserved17[0xF];
    NvU32 SetSurface3LumaOffset[16];                                            // 0x00001224 - 0x00001263
    NvU32 Reserved18[0xF];
    NvU32 SetSurface3ChromaU_Offset[16];                                        // 0x00001228 - 0x00001267
    NvU32 Reserved19[0xF];
    NvU32 SetSurface3ChromaV_Offset[16];                                        // 0x0000122C - 0x0000126B
    NvU32 Reserved20[0xF];
    NvU32 SetSurface4LumaOffset[16];                                            // 0x00001230 - 0x0000126F
    NvU32 Reserved21[0xF];
    NvU32 SetSurface4ChromaU_Offset[16];                                        // 0x00001234 - 0x00001273
    NvU32 Reserved22[0xF];
    NvU32 SetSurface4ChromaV_Offset[16];                                        // 0x00001238 - 0x00001277
    NvU32 Reserved23[0xF];
    NvU32 SetSurface5LumaOffset[16];                                            // 0x0000123C - 0x0000127B
    NvU32 Reserved24[0xF];
    NvU32 SetSurface5ChromaU_Offset[16];                                        // 0x00001240 - 0x0000127F
    NvU32 Reserved25[0xF];
    NvU32 SetSurface5ChromaV_Offset[16];                                        // 0x00001244 - 0x00001283
    NvU32 Reserved26[0xF];
    NvU32 SetSurface6LumaOffset[16];                                            // 0x00001248 - 0x00001287
    NvU32 Reserved27[0xF];
    NvU32 SetSurface6ChromaU_Offset[16];                                        // 0x0000124C - 0x0000128B
    NvU32 Reserved28[0xF];
    NvU32 SetSurface6ChromaV_Offset[16];                                        // 0x00001250 - 0x0000128F
    NvU32 Reserved29[0xF];
    NvU32 SetSurface7LumaOffset[16];                                            // 0x00001254 - 0x00001293
    NvU32 Reserved30[0xF];
    NvU32 SetSurface7ChromaU_Offset[16];                                        // 0x00001258 - 0x00001297
    NvU32 Reserved31[0xF];
    NvU32 SetSurface7ChromaV_Offset[16];                                        // 0x0000125C - 0x0000129B
    NvU32 Reserved32[0x359];
} B1B6VICControlPio;

#define NVB1B6_VIDEO_COMPOSITOR_NOP                                                              (0x00000100)
#define NVB1B6_VIDEO_COMPOSITOR_NOP_PARAMETER                                                    31:0
#define NVB1B6_VIDEO_COMPOSITOR_PM_TRIGGER                                                       (0x00000140)
#define NVB1B6_VIDEO_COMPOSITOR_PM_TRIGGER_V                                                     31:0
#define NVB1B6_VIDEO_COMPOSITOR_SET_APPLICATION_ID                                               (0x00000200)
#define NVB1B6_VIDEO_COMPOSITOR_SET_APPLICATION_ID_ID                                            31:0
#define NVB1B6_VIDEO_COMPOSITOR_SET_APPLICATION_ID_ID_COMPOSITOR                                 (0x00000000)
#define NVB1B6_VIDEO_COMPOSITOR_SET_WATCHDOG_TIMER                                               (0x00000204)
#define NVB1B6_VIDEO_COMPOSITOR_SET_WATCHDOG_TIMER_TIMER                                         31:0
#define NVB1B6_VIDEO_COMPOSITOR_SEMAPHORE_A                                                      (0x00000240)
#define NVB1B6_VIDEO_COMPOSITOR_SEMAPHORE_A_UPPER                                                7:0
#define NVB1B6_VIDEO_COMPOSITOR_SEMAPHORE_B                                                      (0x00000244)
#define NVB1B6_VIDEO_COMPOSITOR_SEMAPHORE_B_LOWER                                                31:0
#define NVB1B6_VIDEO_COMPOSITOR_SEMAPHORE_C                                                      (0x00000248)
#define NVB1B6_VIDEO_COMPOSITOR_SEMAPHORE_C_PAYLOAD                                              31:0
#define NVB1B6_VIDEO_COMPOSITOR_CTX_SAVE_AREA                                                    (0x0000024C)
#define NVB1B6_VIDEO_COMPOSITOR_CTX_SAVE_AREA_OFFSET                                             27:0
#define NVB1B6_VIDEO_COMPOSITOR_CTX_SAVE_AREA_CTX_VALID                                          31:28
#define NVB1B6_VIDEO_COMPOSITOR_CTX_SWITCH                                                       (0x00000250)
#define NVB1B6_VIDEO_COMPOSITOR_CTX_SWITCH_RESTORE                                               0:0
#define NVB1B6_VIDEO_COMPOSITOR_CTX_SWITCH_RESTORE_FALSE                                         (0x00000000)
#define NVB1B6_VIDEO_COMPOSITOR_CTX_SWITCH_RESTORE_TRUE                                          (0x00000001)
#define NVB1B6_VIDEO_COMPOSITOR_CTX_SWITCH_RST_NOTIFY                                            1:1
#define NVB1B6_VIDEO_COMPOSITOR_CTX_SWITCH_RST_NOTIFY_FALSE                                      (0x00000000)
#define NVB1B6_VIDEO_COMPOSITOR_CTX_SWITCH_RST_NOTIFY_TRUE                                       (0x00000001)
#define NVB1B6_VIDEO_COMPOSITOR_CTX_SWITCH_RESERVED                                              7:2
#define NVB1B6_VIDEO_COMPOSITOR_CTX_SWITCH_ASID                                                  23:8
#define NVB1B6_VIDEO_COMPOSITOR_EXECUTE                                                          (0x00000300)
#define NVB1B6_VIDEO_COMPOSITOR_EXECUTE_NOTIFY                                                   0:0
#define NVB1B6_VIDEO_COMPOSITOR_EXECUTE_NOTIFY_DISABLE                                           (0x00000000)
#define NVB1B6_VIDEO_COMPOSITOR_EXECUTE_NOTIFY_ENABLE                                            (0x00000001)
#define NVB1B6_VIDEO_COMPOSITOR_EXECUTE_NOTIFY_ON                                                1:1
#define NVB1B6_VIDEO_COMPOSITOR_EXECUTE_NOTIFY_ON_END                                            (0x00000000)
#define NVB1B6_VIDEO_COMPOSITOR_EXECUTE_NOTIFY_ON_BEGIN                                          (0x00000001)
#define NVB1B6_VIDEO_COMPOSITOR_EXECUTE_AWAKEN                                                   8:8
#define NVB1B6_VIDEO_COMPOSITOR_EXECUTE_AWAKEN_DISABLE                                           (0x00000000)
#define NVB1B6_VIDEO_COMPOSITOR_EXECUTE_AWAKEN_ENABLE                                            (0x00000001)
#define NVB1B6_VIDEO_COMPOSITOR_SEMAPHORE_D                                                      (0x00000304)
#define NVB1B6_VIDEO_COMPOSITOR_SEMAPHORE_D_STRUCTURE_SIZE                                       0:0
#define NVB1B6_VIDEO_COMPOSITOR_SEMAPHORE_D_STRUCTURE_SIZE_ONE                                   (0x00000000)
#define NVB1B6_VIDEO_COMPOSITOR_SEMAPHORE_D_STRUCTURE_SIZE_FOUR                                  (0x00000001)
#define NVB1B6_VIDEO_COMPOSITOR_SEMAPHORE_D_AWAKEN_ENABLE                                        8:8
#define NVB1B6_VIDEO_COMPOSITOR_SEMAPHORE_D_AWAKEN_ENABLE_FALSE                                  (0x00000000)
#define NVB1B6_VIDEO_COMPOSITOR_SEMAPHORE_D_AWAKEN_ENABLE_TRUE                                   (0x00000001)
#define NVB1B6_VIDEO_COMPOSITOR_SEMAPHORE_D_OPERATION                                            17:16
#define NVB1B6_VIDEO_COMPOSITOR_SEMAPHORE_D_OPERATION_RELEASE                                    (0x00000000)
#define NVB1B6_VIDEO_COMPOSITOR_SEMAPHORE_D_OPERATION_RESERVED0                                  (0x00000001)
#define NVB1B6_VIDEO_COMPOSITOR_SEMAPHORE_D_OPERATION_RESERVED1                                  (0x00000002)
#define NVB1B6_VIDEO_COMPOSITOR_SEMAPHORE_D_OPERATION_TRAP                                       (0x00000003)
#define NVB1B6_VIDEO_COMPOSITOR_SEMAPHORE_D_FLUSH_DISABLE                                        21:21
#define NVB1B6_VIDEO_COMPOSITOR_SEMAPHORE_D_FLUSH_DISABLE_FALSE                                  (0x00000000)
#define NVB1B6_VIDEO_COMPOSITOR_SEMAPHORE_D_FLUSH_DISABLE_TRUE                                   (0x00000001)
#define NVB1B6_VIDEO_COMPOSITOR_SET_PICTURE_INDEX                                                (0x00000700)
#define NVB1B6_VIDEO_COMPOSITOR_SET_PICTURE_INDEX_INDEX                                          31:0
#define NVB1B6_VIDEO_COMPOSITOR_SET_CONTROL_PARAMS                                               (0x00000704)
#define NVB1B6_VIDEO_COMPOSITOR_SET_CONTROL_PARAMS_GPTIMER_ON                                    0:0
#define NVB1B6_VIDEO_COMPOSITOR_SET_CONTROL_PARAMS_DEBUG_MODE                                    4:4
#define NVB1B6_VIDEO_COMPOSITOR_SET_CONTROL_PARAMS_FALCON_CONTROL                                8:8
#define NVB1B6_VIDEO_COMPOSITOR_SET_CONTROL_PARAMS_CONFIG_STRUCT_SIZE                            31:16
#define NVB1B6_VIDEO_COMPOSITOR_SET_CONFIG_STRUCT_OFFSET                                         (0x00000708)
#define NVB1B6_VIDEO_COMPOSITOR_SET_CONFIG_STRUCT_OFFSET_OFFSET                                  31:0
#define NVB1B6_VIDEO_COMPOSITOR_SET_FILTER_STRUCT_OFFSET                                         (0x0000070C)
#define NVB1B6_VIDEO_COMPOSITOR_SET_FILTER_STRUCT_OFFSET_OFFSET                                  31:0
#define NVB1B6_VIDEO_COMPOSITOR_SET_PALETTE_OFFSET                                               (0x00000710)
#define NVB1B6_VIDEO_COMPOSITOR_SET_PALETTE_OFFSET_OFFSET                                        31:0
#define NVB1B6_VIDEO_COMPOSITOR_SET_HIST_OFFSET                                                  (0x00000714)
#define NVB1B6_VIDEO_COMPOSITOR_SET_HIST_OFFSET_OFFSET                                           31:0
#define NVB1B6_VIDEO_COMPOSITOR_SET_CONTEXT_ID                                                   (0x00000718)
#define NVB1B6_VIDEO_COMPOSITOR_SET_CONTEXT_ID_FCE_UCODE                                         3:0
#define NVB1B6_VIDEO_COMPOSITOR_SET_CONTEXT_ID_CONFIG                                            7:4
#define NVB1B6_VIDEO_COMPOSITOR_SET_CONTEXT_ID_PALETTE                                           11:8
#define NVB1B6_VIDEO_COMPOSITOR_SET_CONTEXT_ID_OUTPUT                                            15:12
#define NVB1B6_VIDEO_COMPOSITOR_SET_CONTEXT_ID_HIST                                              19:16
#define NVB1B6_VIDEO_COMPOSITOR_SET_FCE_UCODE_SIZE                                               (0x0000071C)
#define NVB1B6_VIDEO_COMPOSITOR_SET_FCE_UCODE_SIZE_FCE_SZ                                        15:0
#define NVB1B6_VIDEO_COMPOSITOR_SET_OUTPUT_SURFACE_LUMA_OFFSET                                   (0x00000720)
#define NVB1B6_VIDEO_COMPOSITOR_SET_OUTPUT_SURFACE_LUMA_OFFSET_OFFSET                            31:0
#define NVB1B6_VIDEO_COMPOSITOR_SET_OUTPUT_SURFACE_CHROMA_U_OFFSET                               (0x00000724)
#define NVB1B6_VIDEO_COMPOSITOR_SET_OUTPUT_SURFACE_CHROMA_U_OFFSET_OFFSET                        31:0
#define NVB1B6_VIDEO_COMPOSITOR_SET_OUTPUT_SURFACE_CHROMA_V_OFFSET                               (0x00000728)
#define NVB1B6_VIDEO_COMPOSITOR_SET_OUTPUT_SURFACE_CHROMA_V_OFFSET_OFFSET                        31:0
#define NVB1B6_VIDEO_COMPOSITOR_SET_FCE_UCODE_OFFSET                                             (0x0000072C)
#define NVB1B6_VIDEO_COMPOSITOR_SET_FCE_UCODE_OFFSET_OFFSET                                      31:0
#define NVB1B6_VIDEO_COMPOSITOR_SET_CRC_STRUCT_OFFSET                                            (0x00000730)
#define NVB1B6_VIDEO_COMPOSITOR_SET_CRC_STRUCT_OFFSET_OFFSET                                     31:0
#define NVB1B6_VIDEO_COMPOSITOR_SET_CRC_MODE                                                     (0x00000734)
#define NVB1B6_VIDEO_COMPOSITOR_SET_CRC_MODE_INTF_PART_ASEL                                      3:0
#define NVB1B6_VIDEO_COMPOSITOR_SET_CRC_MODE_INTF_PART_BSEL                                      7:4
#define NVB1B6_VIDEO_COMPOSITOR_SET_CRC_MODE_INTF_PART_CSEL                                      11:8
#define NVB1B6_VIDEO_COMPOSITOR_SET_CRC_MODE_INTF_PART_DSEL                                      15:12
#define NVB1B6_VIDEO_COMPOSITOR_SET_CRC_MODE_CRC_MODE                                            16:16
#define NVB1B6_VIDEO_COMPOSITOR_SET_STATUS_OFFSET                                                (0x00000738)
#define NVB1B6_VIDEO_COMPOSITOR_SET_STATUS_OFFSET_OFFSET                                         31:0
#define NVB1B6_VIDEO_COMPOSITOR_SET_SLOT_CONTEXT_ID(b)                                           (0x00000740 + (b)*0x00000004)
#define NVB1B6_VIDEO_COMPOSITOR_SET_SLOT_CONTEXT_ID_CTX_ID_SFC0                                  3:0
#define NVB1B6_VIDEO_COMPOSITOR_SET_SLOT_CONTEXT_ID_CTX_ID_SFC1                                  7:4
#define NVB1B6_VIDEO_COMPOSITOR_SET_SLOT_CONTEXT_ID_CTX_ID_SFC2                                  11:8
#define NVB1B6_VIDEO_COMPOSITOR_SET_SLOT_CONTEXT_ID_CTX_ID_SFC3                                  15:12
#define NVB1B6_VIDEO_COMPOSITOR_SET_SLOT_CONTEXT_ID_CTX_ID_SFC4                                  19:16
#define NVB1B6_VIDEO_COMPOSITOR_SET_SLOT_CONTEXT_ID_CTX_ID_SFC5                                  23:20
#define NVB1B6_VIDEO_COMPOSITOR_SET_SLOT_CONTEXT_ID_CTX_ID_SFC6                                  27:24
#define NVB1B6_VIDEO_COMPOSITOR_SET_SLOT_CONTEXT_ID_CTX_ID_SFC7                                  31:28
#define NVB1B6_VIDEO_COMPOSITOR_SET_HISTORY_BUFFER_OFFSET(b)                                     (0x00000780 + (b)*0x00000004)
#define NVB1B6_VIDEO_COMPOSITOR_SET_HISTORY_BUFFER_OFFSET_OFFSET                                 31:0
#define NVB1B6_VIDEO_COMPOSITOR_SET_COMP_TAG_BUFFER_OFFSET(b)                                    (0x000007C0 + (b)*0x00000004)
#define NVB1B6_VIDEO_COMPOSITOR_SET_COMP_TAG_BUFFER_OFFSET_OFFSET                                31:0
#define NVB1B6_VIDEO_COMPOSITOR_PM_TRIGGER_END                                                   (0x00001114)
#define NVB1B6_VIDEO_COMPOSITOR_PM_TRIGGER_END_V                                                 31:0
#define NVB1B6_VIDEO_COMPOSITOR_SET_SURFACE0_LUMA_OFFSET(b)                                      (0x00001200 + (b)*0x00000060)
#define NVB1B6_VIDEO_COMPOSITOR_SET_SURFACE0_LUMA_OFFSET_OFFSET                                  31:0
#define NVB1B6_VIDEO_COMPOSITOR_SET_SURFACE0_CHROMA_U_OFFSET(b)                                  (0x00001204 + (b)*0x00000060)
#define NVB1B6_VIDEO_COMPOSITOR_SET_SURFACE0_CHROMA_U_OFFSET_OFFSET                              31:0
#define NVB1B6_VIDEO_COMPOSITOR_SET_SURFACE0_CHROMA_V_OFFSET(b)                                  (0x00001208 + (b)*0x00000060)
#define NVB1B6_VIDEO_COMPOSITOR_SET_SURFACE0_CHROMA_V_OFFSET_OFFSET                              31:0
#define NVB1B6_VIDEO_COMPOSITOR_SET_SURFACE1_LUMA_OFFSET(b)                                      (0x0000120C + (b)*0x00000060)
#define NVB1B6_VIDEO_COMPOSITOR_SET_SURFACE1_LUMA_OFFSET_OFFSET                                  31:0
#define NVB1B6_VIDEO_COMPOSITOR_SET_SURFACE1_CHROMA_U_OFFSET(b)                                  (0x00001210 + (b)*0x00000060)
#define NVB1B6_VIDEO_COMPOSITOR_SET_SURFACE1_CHROMA_U_OFFSET_OFFSET                              31:0
#define NVB1B6_VIDEO_COMPOSITOR_SET_SURFACE1_CHROMA_V_OFFSET(b)                                  (0x00001214 + (b)*0x00000060)
#define NVB1B6_VIDEO_COMPOSITOR_SET_SURFACE1_CHROMA_V_OFFSET_OFFSET                              31:0
#define NVB1B6_VIDEO_COMPOSITOR_SET_SURFACE2_LUMA_OFFSET(b)                                      (0x00001218 + (b)*0x00000060)
#define NVB1B6_VIDEO_COMPOSITOR_SET_SURFACE2_LUMA_OFFSET_OFFSET                                  31:0
#define NVB1B6_VIDEO_COMPOSITOR_SET_SURFACE2_CHROMA_U_OFFSET(b)                                  (0x0000121C + (b)*0x00000060)
#define NVB1B6_VIDEO_COMPOSITOR_SET_SURFACE2_CHROMA_U_OFFSET_OFFSET                              31:0
#define NVB1B6_VIDEO_COMPOSITOR_SET_SURFACE2_CHROMA_V_OFFSET(b)                                  (0x00001220 + (b)*0x00000060)
#define NVB1B6_VIDEO_COMPOSITOR_SET_SURFACE2_CHROMA_V_OFFSET_OFFSET                              31:0
#define NVB1B6_VIDEO_COMPOSITOR_SET_SURFACE3_LUMA_OFFSET(b)                                      (0x00001224 + (b)*0x00000060)
#define NVB1B6_VIDEO_COMPOSITOR_SET_SURFACE3_LUMA_OFFSET_OFFSET                                  31:0
#define NVB1B6_VIDEO_COMPOSITOR_SET_SURFACE3_CHROMA_U_OFFSET(b)                                  (0x00001228 + (b)*0x00000060)
#define NVB1B6_VIDEO_COMPOSITOR_SET_SURFACE3_CHROMA_U_OFFSET_OFFSET                              31:0
#define NVB1B6_VIDEO_COMPOSITOR_SET_SURFACE3_CHROMA_V_OFFSET(b)                                  (0x0000122C + (b)*0x00000060)
#define NVB1B6_VIDEO_COMPOSITOR_SET_SURFACE3_CHROMA_V_OFFSET_OFFSET                              31:0
#define NVB1B6_VIDEO_COMPOSITOR_SET_SURFACE4_LUMA_OFFSET(b)                                      (0x00001230 + (b)*0x00000060)
#define NVB1B6_VIDEO_COMPOSITOR_SET_SURFACE4_LUMA_OFFSET_OFFSET                                  31:0
#define NVB1B6_VIDEO_COMPOSITOR_SET_SURFACE4_CHROMA_U_OFFSET(b)                                  (0x00001234 + (b)*0x00000060)
#define NVB1B6_VIDEO_COMPOSITOR_SET_SURFACE4_CHROMA_U_OFFSET_OFFSET                              31:0
#define NVB1B6_VIDEO_COMPOSITOR_SET_SURFACE4_CHROMA_V_OFFSET(b)                                  (0x00001238 + (b)*0x00000060)
#define NVB1B6_VIDEO_COMPOSITOR_SET_SURFACE4_CHROMA_V_OFFSET_OFFSET                              31:0
#define NVB1B6_VIDEO_COMPOSITOR_SET_SURFACE5_LUMA_OFFSET(b)                                      (0x0000123C + (b)*0x00000060)
#define NVB1B6_VIDEO_COMPOSITOR_SET_SURFACE5_LUMA_OFFSET_OFFSET                                  31:0
#define NVB1B6_VIDEO_COMPOSITOR_SET_SURFACE5_CHROMA_U_OFFSET(b)                                  (0x00001240 + (b)*0x00000060)
#define NVB1B6_VIDEO_COMPOSITOR_SET_SURFACE5_CHROMA_U_OFFSET_OFFSET                              31:0
#define NVB1B6_VIDEO_COMPOSITOR_SET_SURFACE5_CHROMA_V_OFFSET(b)                                  (0x00001244 + (b)*0x00000060)
#define NVB1B6_VIDEO_COMPOSITOR_SET_SURFACE5_CHROMA_V_OFFSET_OFFSET                              31:0
#define NVB1B6_VIDEO_COMPOSITOR_SET_SURFACE6_LUMA_OFFSET(b)                                      (0x00001248 + (b)*0x00000060)
#define NVB1B6_VIDEO_COMPOSITOR_SET_SURFACE6_LUMA_OFFSET_OFFSET                                  31:0
#define NVB1B6_VIDEO_COMPOSITOR_SET_SURFACE6_CHROMA_U_OFFSET(b)                                  (0x0000124C + (b)*0x00000060)
#define NVB1B6_VIDEO_COMPOSITOR_SET_SURFACE6_CHROMA_U_OFFSET_OFFSET                              31:0
#define NVB1B6_VIDEO_COMPOSITOR_SET_SURFACE6_CHROMA_V_OFFSET(b)                                  (0x00001250 + (b)*0x00000060)
#define NVB1B6_VIDEO_COMPOSITOR_SET_SURFACE6_CHROMA_V_OFFSET_OFFSET                              31:0
#define NVB1B6_VIDEO_COMPOSITOR_SET_SURFACE7_LUMA_OFFSET(b)                                      (0x00001254 + (b)*0x00000060)
#define NVB1B6_VIDEO_COMPOSITOR_SET_SURFACE7_LUMA_OFFSET_OFFSET                                  31:0
#define NVB1B6_VIDEO_COMPOSITOR_SET_SURFACE7_CHROMA_U_OFFSET(b)                                  (0x00001258 + (b)*0x00000060)
#define NVB1B6_VIDEO_COMPOSITOR_SET_SURFACE7_CHROMA_U_OFFSET_OFFSET                              31:0
#define NVB1B6_VIDEO_COMPOSITOR_SET_SURFACE7_CHROMA_V_OFFSET(b)                                  (0x0000125C + (b)*0x00000060)
#define NVB1B6_VIDEO_COMPOSITOR_SET_SURFACE7_CHROMA_V_OFFSET_OFFSET                              31:0

#ifdef __cplusplus
};     /* extern "C" */
#endif
#endif // _clb1b6_h
