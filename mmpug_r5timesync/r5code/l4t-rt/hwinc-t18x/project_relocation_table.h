//
// Copyright (c) 2017 NVIDIA Corporation.
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
//  * Neither the name of NVIDIA CORPORATION nor the names of its
//    contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ``AS IS'' AND ANY
// EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
// PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
// CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
// EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
// PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
// OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//

//////////////////////////////////////////////////////////////////
//
//
//
// START OF NV DEVICE ID MAP
//
//
//
//////////////////////////////////////////////////////////////////
//// Note: '0' indicates an invalid dev id and is used to indicate that an entity does not have a dev id assigned to it.
// Memory Aperture: Internal Memory
#define NV_DEVID_IMEM                                          1

// Memory Aperture: External Memory
#define NV_DEVID_EMEM                                          2

// Memory Aperture: TCRAM
#define NV_DEVID_TCRAM                                         3

// Memory Aperture: IRAM
#define NV_DEVID_IRAM                                          4

// Memory Aperture: NOR FLASH
#define NV_DEVID_NOR                                           5

// Memory Aperture: EXIO
#define NV_DEVID_EXIO                                          6

// Memory Aperture: GART
#define NV_DEVID_GART                                          7

// Device Aperture: Graphics Host (HOST1X)
#define NV_DEVID_HOST1X                                        8

// Device Aperture: ARM PERIPH registers
#define NV_DEVID_ARM_PERIPH                                    9

// Device Aperture: MSELECT
#define NV_DEVID_MSELECT                                      10

// Device Aperture: memory controller
#define NV_DEVID_MC                                           11

// Device Aperture: external memory controller
#define NV_DEVID_EMC                                          12

// Device Aperture: video input
#define NV_DEVID_VI                                           13

// Device Aperture: encoder pre-processor
#define NV_DEVID_EPP                                          14

// Device Aperture: video encoder
#define NV_DEVID_MPE                                          15

// Device Aperture: 3D engine
#define NV_DEVID_GR3D                                         16

// Device Aperture: 2D + SBLT engine
#define NV_DEVID_GR2D                                         17

// Device Aperture: Image Signal Processor
#define NV_DEVID_ISP                                          18

// Device Aperture: DISPLAY
#define NV_DEVID_DISPLAY                                      19

// Device Aperture: UPTAG
#define NV_DEVID_UPTAG                                        20

// Device Aperture - SHR_SEM
#define NV_DEVID_SHR_SEM                                      21

// Device Aperture - ARB_SEM
#define NV_DEVID_ARB_SEM                                      22

// Device Aperture - ARB_PRI
#define NV_DEVID_ARB_PRI                                      23

// Obsoleted for AP15
#define NV_DEVID_PRI_INTR                                     24

// Obsoleted for AP15
#define NV_DEVID_SEC_INTR                                     25

// Device Aperture: Timer Programmable
#define NV_DEVID_TMR                                          26

// Device Aperture: Clock and Reset
#define NV_DEVID_CAR                                          27

// Device Aperture: Flow control
#define NV_DEVID_FLOW                                         28

// Device Aperture: Event
#define NV_DEVID_EVENT                                        29

// Device Aperture: AHB DMA
#define NV_DEVID_AHB_DMA                                      30

// Device Aperture: APB DMA
#define NV_DEVID_APB_DMA                                      31

// Obsolete - use AVP_CACHE
#define NV_DEVID_CC                                           32

// Device Aperture: COP Cache Controller
#define NV_DEVID_AVP_CACHE                                    32

// Device Aperture: SYS_REG
#define NV_DEVID_SYS_REG                                      32

// Device Aperture: System Statistic monitor
#define NV_DEVID_STAT                                         33

// Device Aperture: GPIO
#define NV_DEVID_GPIO                                         34

// Device Aperture: Vector Co-Processor 2
#define NV_DEVID_VCP                                          35

// Device Aperture: Arm Vectors
#define NV_DEVID_VECTOR                                       36

//
#define NV_DEVID_MEM                                          37

// Device Aperture: BSEA (now in AVP cluster)
#define NV_DEVID_BSEA                                         47

// Device Aperture: Misc regs
#define NV_DEVID_MISC                                         49

// Obsolete
#define NV_DEVID_AC97                                         50

// Device Aperture: S/P-DIF
#define NV_DEVID_SPDIF                                        51

// Device Aperture: I2S
#define NV_DEVID_I2S                                          52

// Device Aperture: UART
#define NV_DEVID_UART                                         53

// Device Aperture: VFIR
#define NV_DEVID_VFIR                                         54

// Device Aperture: NAND Flash Controller
#define NV_DEVID_NANDCTRL                                     55

// Obsolete - use NANDCTRL
#define NV_DEVID_NANDFLASH                                    55

// Device Aperture: HSMMC
#define NV_DEVID_HSMMC                                        56

// Device Aperture: XIO
#define NV_DEVID_XIO                                          57

// Device Aperture: PWM
#define NV_DEVID_PWM                                          58

// Device Aperture: MIPI
#define NV_DEVID_MIPI_HS                                      59

// Device Aperture: I2C
#define NV_DEVID_I2C                                          60

// Device Aperture: TWC
#define NV_DEVID_TWC                                          61

// Device Aperture: SLINK
#define NV_DEVID_SLINK                                        62

// Device Aperture: SLINK4B
#define NV_DEVID_SLINK4B                                      63

// Obsolete - use DTV
#define NV_DEVID_SPI                                          64

// Device Aperture: DVC
#define NV_DEVID_DVC                                          65

// Device Aperture: RTC
#define NV_DEVID_RTC                                          66

// Device Aperture: KeyBoard Controller
#define NV_DEVID_KBC                                          67

// Device Aperture: PMIF
#define NV_DEVID_PMIF                                         68

// Device Aperture: FUSE
#define NV_DEVID_FUSE                                         69

// Device Aperture: L2 Cache Controller
#define NV_DEVID_CMC                                          70

// Device Apertuer: NOR FLASH Controller
#define NV_DEVID_NOR_REG                                      71

// Device Aperture: EIDE
#define NV_DEVID_EIDE                                         72

// Device Aperture: USB
#define NV_DEVID_USB                                          73

// Device Aperture: SDIO
#define NV_DEVID_SDIO                                         74

// Device Aperture: TVO
#define NV_DEVID_TVO                                          75

// Device Aperture: DSI
#define NV_DEVID_DSI                                          76

// Device Aperture: Third Interrupt Controller extra registers
#define NV_DEVID_TRI_INTR                                     78

// Device Aperture: Common Interrupt Controller
#define NV_DEVID_ICTLR                                        79

// Non-Aperture Interrupt: DMA TX interrupts
#define NV_DEVID_DMA_TX_INTR                                  80

// Non-Aperture Interrupt: DMA RX interrupts
#define NV_DEVID_DMA_RX_INTR                                  81

// Non-Aperture Interrupt: SW reserved interrupt
#define NV_DEVID_SW_INTR                                      82

// Non-Aperture Interrupt: CPU PMU Interrupt
#define NV_DEVID_CPU_INTR                                     83

// Device Aperture: Timer Free Running MicroSecond
#define NV_DEVID_TMRUS                                        84

// Device Aperture: Interrupt Controller ARB_GNT Registers
#define NV_DEVID_ICTLR_ARBGNT                                 85

// Device Aperture: Interrupt Controller DMA Registers
#define NV_DEVID_ICTLR_DRQ                                    86

// Device Aperture: AHB DMA Channel
#define NV_DEVID_AHB_DMA_CH                                   87

// Device Aperture: APB DMA Channel
#define NV_DEVID_APB_DMA_CH                                   88

// Device Aperture: AHB Arbitration Controller
#define NV_DEVID_AHB_ARBC                                     89

// Obsolete - use AHB_ARBC
#define NV_DEVID_AHB_ARB_CTRL                                 89

// Device Aperture: AHB/APB Debug Bus Registers
#define NV_DEVID_AHPBDEBUG                                    91

// Device Aperture: Secure Boot Register
#define NV_DEVID_SECURE_BOOT                                  92

// Device Aperture: SPROM
#define NV_DEVID_SPROM                                        93

// Non-Aperture Interrupt: External PMU interrupt
#define NV_DEVID_PMU_EXT                                      95

// Device Aperture: AHB EMEM to MC Flush Register
#define NV_DEVID_PPCS                                         96

// Device Aperture: MMU TLB registers for COP/AVP
#define NV_DEVID_MMU_TLB                                      97

// Device Aperture: OVG engine
#define NV_DEVID_VG                                           98

// Device Aperture: CSI
#define NV_DEVID_CSI                                          99

// Device ID for COP
#define NV_DEVID_AVP                                         100

// Device ID for MPCORE
#define NV_DEVID_CPU                                         101

// Device Aperture: ULPI controller
#define NV_DEVID_ULPI                                        102

// Device Aperture: ARM CONFIG registers
#define NV_DEVID_ARM_CONFIG                                  103

// Device Aperture: ARM PL310 (L2 controller)
#define NV_DEVID_ARM_PL310                                   104

// Device Aperture: PCIe
#define NV_DEVID_PCIE                                        105

// Device Aperture: OWR (one wire)
#define NV_DEVID_OWR                                         106

// Device Aperture: AVPUCQ
#define NV_DEVID_AVPUCQ                                      107

// Device Aperture: AVPBSEA (obsolete)
#define NV_DEVID_AVPBSEA                                     108

// Device Aperture: Sync NOR
#define NV_DEVID_SNOR                                        109

// Device Aperture: SDMMC
#define NV_DEVID_SDMMC                                       110

// Device Aperture: KFUSE
#define NV_DEVID_KFUSE                                       111

// Device Aperture: CSITE
#define NV_DEVID_CSITE                                       112

// Non-Aperture Interrupt: ARM Interprocessor Interrupt
#define NV_DEVID_ARM_IPI                                     113

// Device Aperture: ARM Interrupts 0-31
#define NV_DEVID_ARM_ICTLR                                   114

// Obsolete: use mod_IOBIST
#define NV_DEVID_IOBIST                                      115

// Device Aperture: SPEEDO
#define NV_DEVID_SPEEDO                                      116

// Device Aperture: LA
#define NV_DEVID_LA                                          117

// Device Aperture: VS
#define NV_DEVID_VS                                          118

// Device Aperture: VCI
#define NV_DEVID_VCI                                         119

// Device Aperture: APBIF
#define NV_DEVID_APBIF                                       120

// Device Aperture: AUDIO
#define NV_DEVID_AUDIO                                       121

// Device Aperture: DAM
#define NV_DEVID_DAM                                         122

// Device Aperture: TSENSOR
#define NV_DEVID_TSENSOR                                     123

// Device Aperture: SE
#define NV_DEVID_SE                                          124

// Device Aperture: TZRAM
#define NV_DEVID_TZRAM                                       125

// Obsolete from T210 onwards. Device Aperture: AUDIO_CLUSTER
#define NV_DEVID_AUDIO_CLUSTER                               126

// Device Aperture: HDA
#define NV_DEVID_HDA                                         127

// Device Aperture: SATA
#define NV_DEVID_SATA                                        128

// Device Aperture: ATOMICS
#define NV_DEVID_ATOMICS                                     129

// Device Aperture: IPATCH
#define NV_DEVID_IPATCH                                      130

// Device Aperture: Activity Monitor
#define NV_DEVID_ACTMON                                      131

// Device Aperture: Watch Dog Timer
#define NV_DEVID_WDT                                         132

// Device Aperture: DTV
#define NV_DEVID_DTV                                         133

// Device Aperture: Shared Timer
#define NV_DEVID_TMR_SHARED                                  134

// Device Aperture: Consumer Electronics Control
#define NV_DEVID_CEC                                         135

// Device Aperture: MIPIHSI
#define NV_DEVID_MIPIHSI                                     136

// Device Aperture: SATA_IOBIST
#define NV_DEVID_SATA_IOBIST                                 137

// Device Aperture: HDMI_IOBIST
#define NV_DEVID_HDMI_IOBIST                                 138

// Device Aperture: MIPI_IOBIST
#define NV_DEVID_MIPI_IOBIST                                 139

// Device Aperture: LPDDR2_IOBIST
#define NV_DEVID_LPDDR2_IOBIST                               140

// Device Aperture: PCIE_X2_0_IOBIST
#define NV_DEVID_PCIE_X2_0_IOBIST                            141

// Device Aperture: PCIE_X2_1_IOBIST
#define NV_DEVID_PCIE_X2_1_IOBIST                            142

// Device Aperture: PCIE_X4_IOBIST
#define NV_DEVID_PCIE_X4_IOBIST                              143

// Device Aperture: SPEEDO_PMON
#define NV_DEVID_SPEEDO_PMON                                 144

// Device Aperture: NVENC
#define NV_DEVID_NVENC                                       145

// Device Aperture: XUSB_HOST
#define NV_DEVID_XUSB_HOST                                   146

// Device Aperture: XUSB_DEV
#define NV_DEVID_XUSB_DEV                                    147

// Device Aperture: TSEC
#define NV_DEVID_TSEC                                        148

// Device Aperture: DDS
#define NV_DEVID_DDS                                         149

// Device Aperture: DP2
#define NV_DEVID_DP2                                         150

// Device Aperture: APB2JTAG
#define NV_DEVID_APB2JTAG                                    151

// Device Aperture: SOC_THERM
#define NV_DEVID_SOC_THERM                                   152

// Device Aperture: MIPI_CAL
#define NV_DEVID_MIPI_CAL                                    153

// Device Aperture: AMX
#define NV_DEVID_AMX                                         154

// Device Aperture: ADX
#define NV_DEVID_ADX                                         155

// Device Aperture: SYSCTR
#define NV_DEVID_SYSCTR                                      156

// Device Aperture:  Interrupt Controller HIER_GROUP1
#define NV_DEVID_ICTLR_HIER_GROUP1                           157

// Device Aperture:  DVFS
#define NV_DEVID_DVFS                                        158

// Device Aperture:  Configurable EMEM/MMIO aperture
#define NV_DEVID_EMEMIO                                      159

// Device Aperture: XUSB_PADCTL
#define NV_DEVID_XUSB_PADCTL                                 160

// Device Aperture: GPU
#define NV_DEVID_GPU                                         161

// Device Aperture: SOR
#define NV_DEVID_SOR                                         162

// Device Aperture: DPAUX
#define NV_DEVID_DPAUX                                       163

// Device Aperture: COP1
#define NV_DEVID_COP1                                        164

// Device Aperture: Image Signal Processor
#define NV_DEVID_ISPB                                        165

// Device Aperture: VIC
#define NV_DEVID_VIC                                         166

// Non-Aperture Interrupt: COP1 PMU Interrupt
#define NV_DEVID_COP1_INTR                                   167

// Device Aperture: NANDB Flash Controller
#define NV_DEVID_NANDBCTRL                                   168

// Non Aperture for External clock pin
#define NV_DEVID_EXTERNAL_CLOCK                              169

// Device Aperture: MIPIBIF
#define NV_DEVID_MIPIBIF                                     170

// Device Aperture: VGPIO
#define NV_DEVID_VGPIO                                       171

// Device Aperture: DMIC
#define NV_DEVID_DMIC                                        172

// Device Aperture: AFC
#define NV_DEVID_AFC                                         173

// Device Aperture of clock-control registers for the core cluster
#define NV_DEVID_CLUSTER_CLOCK                               174

// Non-Aperture Interrupt: BBC
#define NV_DEVID_BBC0                                        175

// Device Aperture: NVDEC
#define NV_DEVID_NVDEC                                       176

// Device Aperture: CSI3
#define NV_DEVID_CSI3                                        177

// Device Aperture: NVJPG
#define NV_DEVID_NVJPG                                       178

//  Device Aperture: MC0
#define NV_DEVID_MC0                                         180

//  Device Aperture: MC1
#define NV_DEVID_MC1                                         181

//  Device Aperture: EMC0
#define NV_DEVID_EMC0                                        182

//  Device Aperture: EMC1
#define NV_DEVID_EMC1                                        183

//  Device Aperture: BBC_MAILBOX
#define NV_DEVID_BBC_MAILBOX                                 184

//  Device Aperture: QSPI
#define NV_DEVID_QSPI                                        185

//  Device Aperture: EXL
#define NV_DEVID_EXL                                         186

// Device Aperture: DPAUX1
#define NV_DEVID_DPAUX1                                      187

// Device Aperture: video input i2c instance
#define NV_DEVID_VII2C                                       188

//  Device Aperture: APE
#define NV_DEVID_APE                                         189

//  Device Aperture: STM DFD subunit
#define NV_DEVID_STM                                         190

// Device Aperture: SOR1
#define NV_DEVID_SOR1                                        191

// Device Aperture: TSECB
#define NV_DEVID_TSECB                                       192

// Device Aperture: Verif BFMs
#define NV_DEVID_MIOBFM                                      193

// Non-Aperture Interrupt: BBC1
#define NV_DEVID_BBC1                                        194

// Device Aperture: PADCTL
#define NV_DEVID_PADCTL                                      195

// Device Aperture: LIC
#define NV_DEVID_LIC                                         196

//////////////////////////////////////////////////////////////////
//
//
//
// END OF NV DEVICE ID MAP
//
//
//
//////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////
//
//
//
// START OF RELOCATION TABLE (DEPRECATED)
//
//
//
//////////////////////////////////////////////////////////////////
//// Note: Use of Power Groups & BAR values in Relocation Table is strongly deprecated.
//// Hence all Power Groups have been forced to invalid value 0xf.
//// Also all BAR have been forced to constant value 0x0.
// ------------------------------------------------------------
//    hw powergroups
// ------------------------------------------------------------
// Always On
#define NV_POWERGROUP_AO                         0
// Main
#define NV_POWERGROUP_NPG                        1
// CPU related blocks
#define NV_POWERGROUP_CPU                        2
// 3D graphics
#define NV_POWERGROUP_TD                         3
// Video encode engine blocks
#define NV_POWERGROUP_VE                         4
// PCIe
#define NV_POWERGROUP_PCIE                       5
// MPEG encoder
#define NV_POWERGROUP_MPE                        7
// SW define for Power Group maximum
#define NV_POWERGROUP_MAX                        8
// non-mapped power group
#define NV_POWERGROUP_INVALID                    0xffff
// SW table for mapping power group define to register enums (NV_POWERGROUP_INVALID = no mapping)
//  use as 'int table[NV_POWERGROUP_MAX] = { NV_POWERGROUP_ENUM_TABLE }'
#define NV_POWERGROUP_ENUM_TABLE                 NV_POWERGROUP_INVALID, NV_POWERGROUP_INVALID, 0, 1, 2, 3, NV_POWERGROUP_INVALID, 6
#define NV_RELOCATION_TABLE_PTR_OFFSET 64
#define NV_RELOCATION_TABLE_SIZE  858
#define NV_RELOCATION_TABLE_INIT \
     0x0000000001LL, \
     0x0000af10f0LL, 0x00000000LL, 0x00000000LL, \
     0x0000c210f0LL, 0x00000000LL, 0x00000000LL, \
     0x00005310f0LL, 0x00000000LL, 0x00000000LL, \
     0x00005310f0LL, 0x00000000LL, 0x00000000LL, \
     0x00005310f0LL, 0x00000000LL, 0x00000000LL, \
     0x00005310f0LL, 0x00000000LL, 0x00000000LL, \
     0x00005310f0LL, 0x00000000LL, 0x00000000LL, \
     0x00005310f0LL, 0x00000000LL, 0x00000000LL, \
     0x00005310f0LL, 0x00000000LL, 0x00000000LL, \
     0x00005310f0LL, 0x00000000LL, 0x00000000LL, \
     0x00005310f0LL, 0x00000000LL, 0x00000000LL, \
     0x00005310f0LL, 0x00000000LL, 0x00000000LL, \
     0x00007110f0LL, 0x00000000LL, 0x00000000LL, \
     0x00007110f0LL, 0x00000000LL, 0x00000000LL, \
     0x00007110f0LL, 0x00000000LL, 0x00000000LL, \
     0x00007110f0LL, 0x00000000LL, 0x00000000LL, \
     0x00007110f0LL, 0x00000000LL, 0x00000000LL, \
     0x00007110f0LL, 0x00000000LL, 0x00000000LL, \
     0x00007110f0LL, 0x00000000LL, 0x00000000LL, \
     0x00007110f0LL, 0x00000000LL, 0x00000000LL, \
     0x00007110f0LL, 0x00000000LL, 0x00000000LL, \
     0x00007110f0LL, 0x00000000LL, 0x00000000LL, \
     0x00007110f0LL, 0x00000000LL, 0x00000000LL, \
     0x00007110f0LL, 0x00000000LL, 0x00000000LL, \
     0x00007110f0LL, 0x00000000LL, 0x00000000LL, \
     0x00007110f0LL, 0x00000000LL, 0x00000000LL, \
     0x00007110f0LL, 0x00000000LL, 0x00000000LL, \
     0x00007110f0LL, 0x00000000LL, 0x00000000LL, \
     0x00005f10f0LL, 0x00000000LL, 0x00000000LL, \
     0x00005210f0LL, 0x00000000LL, 0x00000000LL, \
     0x00009f00f0LL, 0x00000000LL, 0x01000000LL, \
     0x00003121f0LL, 0x00100000LL, 0x00010000LL, \
     0x00000000f0LL, 0x00100000LL, 0x00000400LL, \
     0x00000000f0LL, 0x00100400LL, 0x00000400LL, \
     0x00000000f0LL, 0x00100800LL, 0x00000400LL, \
     0x00000000f0LL, 0x00100c00LL, 0x00000100LL, \
     0x00000000f0LL, 0x00100d00LL, 0x00000100LL, \
     0x00000000f0LL, 0x00101000LL, 0x00000100LL, \
     0x00000000f0LL, 0x00103000LL, 0x00001000LL, \
     0x0000bd11f0LL, 0x02900000LL, 0x00200000LL, \
     0x00000010f0LL, 0x03010000LL, 0x000e0000LL, \
     0x00008610f0LL, 0x03010000LL, 0x00010000LL, \
     0x00001a10f0LL, 0x03020000LL, 0x00010000LL, \
     0x00001a10f0LL, 0x03030000LL, 0x00010000LL, \
     0x00001a10f0LL, 0x03040000LL, 0x00010000LL, \
     0x00001a10f0LL, 0x03050000LL, 0x00010000LL, \
     0x00001a10f0LL, 0x03060000LL, 0x00010000LL, \
     0x00001a10f0LL, 0x03070000LL, 0x00010000LL, \
     0x00001a10f0LL, 0x03080000LL, 0x00010000LL, \
     0x00001a10f0LL, 0x03090000LL, 0x00010000LL, \
     0x00001a10f0LL, 0x030a0000LL, 0x00010000LL, \
     0x00001a10f0LL, 0x030b0000LL, 0x00010000LL, \
     0x00008410f0LL, 0x030c0000LL, 0x00010000LL, \
     0x00008410f0LL, 0x030d0000LL, 0x00010000LL, \
     0x00008410f0LL, 0x030e0000LL, 0x00010000LL, \
     0x00003513f0LL, 0x03100000LL, 0x00010000LL, \
     0x00003513f0LL, 0x03110000LL, 0x00010000LL, \
     0x00003513f0LL, 0x03130000LL, 0x00010000LL, \
     0x00003513f0LL, 0x03140000LL, 0x00010000LL, \
     0x00003c12f0LL, 0x03160000LL, 0x00010000LL, \
     0x00003c12f0LL, 0x03160000LL, 0x00010000LL, \
     0x00003c12f0LL, 0x03180000LL, 0x00010000LL, \
     0x00003c12f0LL, 0x03190000LL, 0x00010000LL, \
     0x00003c12f0LL, 0x031a0000LL, 0x00010000LL, \
     0x00003c12f0LL, 0x031b0000LL, 0x00010000LL, \
     0x00004012f0LL, 0x03210000LL, 0x00010000LL, \
     0x00000012f0LL, 0x03220000LL, 0x00010000LL, \
     0x00004012f0LL, 0x03230000LL, 0x00010000LL, \
     0x00004012f0LL, 0x03240000LL, 0x00010000LL, \
     0x0000b910f0LL, 0x03270000LL, 0x00010000LL, \
     0x00003a10f0LL, 0x03280000LL, 0x00010000LL, \
     0x00000041f0LL, 0x03400000LL, 0x00020000LL, \
     0x00006e41f0LL, 0x03400000LL, 0x00010000LL, \
     0x00006e41f0LL, 0x03410000LL, 0x00010000LL, \
     0x00000041f0LL, 0x03420000LL, 0x00020000LL, \
     0x00006e41f0LL, 0x03420000LL, 0x00010000LL, \
     0x00006e41f0LL, 0x03430000LL, 0x00010000LL, \
     0x00000041f0LL, 0x03440000LL, 0x00020000LL, \
     0x00006e41f0LL, 0x03440000LL, 0x00010000LL, \
     0x00006e41f0LL, 0x03450000LL, 0x00010000LL, \
     0x00000041f0LL, 0x03460000LL, 0x00020000LL, \
     0x00006e41f0LL, 0x03460000LL, 0x00010000LL, \
     0x00006e41f0LL, 0x03470000LL, 0x00010000LL, \
     0x00008010f0LL, 0x03500000LL, 0x00010000LL, \
     0x00007f10f0LL, 0x03510000LL, 0x00010000LL, \
     0x0000a010f0LL, 0x03520000LL, 0x00010000LL, \
     0x00009210f0LL, 0x03530000LL, 0x00010000LL, \
     0x00009310f0LL, 0x03550000LL, 0x00010000LL, \
     0x00004920f0LL, 0x03560000LL, 0x00010000LL, \
     0x00004921f0LL, 0x03570000LL, 0x00010000LL, \
     0x00004922f0LL, 0x03580000LL, 0x00010000LL, \
     0x0000c110f0LL, 0x03800000LL, 0x00010000LL, \
     0x00004511f0LL, 0x03820000LL, 0x00010000LL, \
     0x00006f10f0LL, 0x03830000LL, 0x00010000LL, \
     0x0000ae10f0LL, 0x03840000LL, 0x00040000LL, \
     0x00000910f0LL, 0x03880000LL, 0x00020000LL, \
     0x00007210f0LL, 0x03881000LL, 0x00001000LL, \
     0x00007210f0LL, 0x03882000LL, 0x00002000LL, \
     0x00007210f0LL, 0x03884000LL, 0x00001000LL, \
     0x00007210f0LL, 0x03885000LL, 0x00001000LL, \
     0x00007210f0LL, 0x03886000LL, 0x00002000LL, \
     0x00002110f0LL, 0x03900000LL, 0x00010000LL, \
     0x00003830f0LL, 0x03910000LL, 0x00010000LL, \
     0x00003910f0LL, 0x03920000LL, 0x00010000LL, \
     0x00007b10f0LL, 0x03950000LL, 0x00010000LL, \
     0x00008711f0LL, 0x03960000LL, 0x00010000LL, \
     0x00009610f0LL, 0x03970000LL, 0x00010000LL, \
     0x00009710f0LL, 0x03980000LL, 0x00010000LL, \
     0x00009910f0LL, 0x03990000LL, 0x00010000LL, \
     0x00009e10f0LL, 0x039a0000LL, 0x00010000LL, \
     0x00008a10f0LL, 0x039d0000LL, 0x00010000LL, \
     0x00008b10f0LL, 0x039e0000LL, 0x00010000LL, \
     0x00008c10f0LL, 0x039f0000LL, 0x00010000LL, \
     0x00008d10f0LL, 0x03a00000LL, 0x00010000LL, \
     0x00008e10f0LL, 0x03a10000LL, 0x00010000LL, \
     0x00008f10f0LL, 0x03a20000LL, 0x00010000LL, \
     0x00008910f0LL, 0x03a30000LL, 0x00010000LL, \
     0x00003d10f0LL, 0x03a40000LL, 0x00010000LL, \
     0x00008510f0LL, 0x03a50000LL, 0x00010000LL, \
     0x00000000f0LL, 0x03a90000LL, 0x00010000LL, \
     0x00007c25f0LL, 0x03ac0000LL, 0x00030000LL, \
     0x00007410f0LL, 0x03b00000LL, 0x00010000LL, \
     0x00007410f0LL, 0x03b10000LL, 0x00010000LL, \
     0x00009010f0LL, 0x03b20000LL, 0x00010000LL, \
     0x00009010f0LL, 0x03b30000LL, 0x00010000LL, \
     0x0000000000LL, 0x05000000LL, 0x01000000LL, \
     0x00000610f0LL, 0x06000000LL, 0x00400000LL, \
     0x00007010f0LL, 0x08000000LL, 0x02000000LL, \
     0x0000be10f0LL, 0x0a000000LL, 0x01000000LL, \
     0x00003c12f0LL, 0x0c240000LL, 0x00010000LL, \
     0x00004012f0LL, 0x0c260000LL, 0x00010000LL, \
     0x00003513f0LL, 0x0c280000LL, 0x00010000LL, \
     0x00004211f0LL, 0x0c2a0000LL, 0x00010000LL, \
     0x00009c10f0LL, 0x0c2b0000LL, 0x00010000LL, \
     0x00009c10f0LL, 0x0c2c0000LL, 0x00010000LL, \
     0x00003a10f0LL, 0x0c340000LL, 0x00010000LL, \
     0x00004412f0LL, 0x0c360000LL, 0x00040000LL, \
     0x00005c10f0LL, 0x0d210000LL, 0x00010000LL, \
     0x00008310f0LL, 0x0d230000LL, 0x00010000LL, \
     0x00009810f0LL, 0x0d280000LL, 0x00010000LL, \
     0x00009f11f0LL, 0x10000000LL, 0x10000000LL, \
     0x00000810f0LL, 0x13e00000LL, 0x00110000LL, \
     0x0000bb10f0LL, 0x15040000LL, 0x00040000LL, \
     0x00006331f0LL, 0x15080000LL, 0x00040000LL, \
     0x0000c020f0LL, 0x15100000LL, 0x00040000LL, \
     0x00001317f0LL, 0x15200000LL, 0x00040000LL, \
     0x00001317f0LL, 0x15240000LL, 0x00040000LL, \
     0x00004c11f0LL, 0x15300000LL, 0x00040000LL, \
     0x0000a620f0LL, 0x15340000LL, 0x00040000LL, \
     0x0000b211f0LL, 0x15380000LL, 0x00040000LL, \
     0x00004c11f0LL, 0x15400000LL, 0x00040000LL, \
     0x0000b030f0LL, 0x15480000LL, 0x00040000LL, \
     0x00009161f0LL, 0x154c0000LL, 0x00040000LL, \
     0x00009420f0LL, 0x15500000LL, 0x00040000LL, \
     0x0000a210f0LL, 0x15540000LL, 0x00040000LL, \
     0x0000bf15f0LL, 0x15580000LL, 0x00040000LL, \
     0x0000a310f0LL, 0x155c0000LL, 0x00040000LL, \
     0x00001240f0LL, 0x15600000LL, 0x00040000LL, \
     0x00001240f0LL, 0x15680000LL, 0x00040000LL, \
     0x0000bc20f0LL, 0x156c0000LL, 0x00040000LL, \
     0x00000d40f0LL, 0x15700000LL, 0x00100000LL, \
     0x0000a120f0LL, 0x17000000LL, 0x09000000LL, \
     0x00006910f0LL, 0x40000000LL, 0x40000000LL, \
     0x00009f11f0LL, 0x40000000LL, 0x20000000LL, \
     0x00000411f0LL, 0x40000000LL, 0x00010000LL, \
     0x00000411f0LL, 0x40010000LL, 0x00010000LL, \
     0x00000411f0LL, 0x40020000LL, 0x00010000LL, \
     0x00000411f0LL, 0x40030000LL, 0x00010000LL, \
     0x00009f00f0LL, 0x41000000LL, 0x01000000LL, \
     0x00002010f0LL, 0x50040000LL, 0x00002000LL, \
     0x00000a10f0LL, 0x50060000LL, 0x00001000LL, \
     0x00009f00f0LL, 0x51000000LL, 0x03000000LL, \
     0x00009f00f0LL, 0x55000000LL, 0x02000000LL, \
     0x00009f00f0LL, 0x57000000LL, 0x09000000LL, \
     0x00009f11f0LL, 0x60000000LL, 0x20000000LL, \
     0x00001410f0LL, 0x60000000LL, 0x00001000LL, \
     0x00001510f0LL, 0x60001000LL, 0x00001000LL, \
     0x00001610f0LL, 0x60002000LL, 0x00001000LL, \
     0x00001710f0LL, 0x60003000LL, 0x00001000LL, \
     0x00000010f0LL, 0x60005000LL, 0x00001000LL, \
     0x00005410f0LL, 0x60005010LL, 0x00000040LL, \
     0x00001a10f0LL, 0x60005090LL, 0x00000008LL, \
     0x00001a10f0LL, 0x60005098LL, 0x00000008LL, \
     0x00001a10f0LL, 0x600050a0LL, 0x00000008LL, \
     0x00001a10f0LL, 0x600050a8LL, 0x00000008LL, \
     0x00008410f0LL, 0x60005160LL, 0x00000020LL, \
     0x00008410f0LL, 0x60005180LL, 0x00000020LL, \
     0x00001c10f0LL, 0x60007000LL, 0x00001000LL, \
     0x00001e11f0LL, 0x60008000LL, 0x00002000LL, \
     0x00005710f0LL, 0x60009000LL, 0x00000020LL, \
     0x00005710f0LL, 0x60009020LL, 0x00000020LL, \
     0x00005710f0LL, 0x60009040LL, 0x00000020LL, \
     0x00005710f0LL, 0x60009060LL, 0x00000020LL, \
     0x00002010f0LL, 0x6000c000LL, 0x00001000LL, \
     0x00005910f0LL, 0x6000c000LL, 0x00000150LL, \
     0x00005b10f0LL, 0x6000c150LL, 0x000000b0LL, \
     0x00002410f0LL, 0x6000f000LL, 0x00001000LL, \
     0x00002221f0LL, 0x60010000LL, 0x00001000LL, \
     0x00002221f0LL, 0x60011000LL, 0x00001000LL, \
     0x00002221f0LL, 0x60012000LL, 0x00001000LL, \
     0x00002221f0LL, 0x60013000LL, 0x00001000LL, \
     0x00002221f0LL, 0x60014000LL, 0x00001000LL, \
     0x00002221f0LL, 0x60015000LL, 0x00001000LL, \
     0x00002221f0LL, 0x60016000LL, 0x00001000LL, \
     0x00002221f0LL, 0x60017000LL, 0x00001000LL, \
     0x00008210f0LL, 0x6001d000LL, 0x00001000LL, \
     0x00001f11f0LL, 0x60020000LL, 0x00004000LL, \
     0x00005810f0LL, 0x60021000LL, 0x00000040LL, \
     0x00005810f0LL, 0x60021040LL, 0x00000040LL, \
     0x00005810f0LL, 0x60021080LL, 0x00000040LL, \
     0x00005810f0LL, 0x600210c0LL, 0x00000040LL, \
     0x00005810f0LL, 0x60021100LL, 0x00000040LL, \
     0x00005810f0LL, 0x60021140LL, 0x00000040LL, \
     0x00005810f0LL, 0x60021180LL, 0x00000040LL, \
     0x00005810f0LL, 0x600211c0LL, 0x00000040LL, \
     0x00005810f0LL, 0x60021200LL, 0x00000040LL, \
     0x00005810f0LL, 0x60021240LL, 0x00000040LL, \
     0x00005810f0LL, 0x60021280LL, 0x00000040LL, \
     0x00005810f0LL, 0x600212c0LL, 0x00000040LL, \
     0x00005810f0LL, 0x60021300LL, 0x00000040LL, \
     0x00005810f0LL, 0x60021340LL, 0x00000040LL, \
     0x00005810f0LL, 0x60021380LL, 0x00000040LL, \
     0x00005810f0LL, 0x600213c0LL, 0x00000040LL, \
     0x00005810f0LL, 0x60021400LL, 0x00000040LL, \
     0x00005810f0LL, 0x60021440LL, 0x00000040LL, \
     0x00005810f0LL, 0x60021480LL, 0x00000040LL, \
     0x00005810f0LL, 0x600214c0LL, 0x00000040LL, \
     0x00005810f0LL, 0x60021500LL, 0x00000040LL, \
     0x00005810f0LL, 0x60021540LL, 0x00000040LL, \
     0x00005810f0LL, 0x60021580LL, 0x00000040LL, \
     0x00005810f0LL, 0x600215c0LL, 0x00000040LL, \
     0x00005810f0LL, 0x60021600LL, 0x00000040LL, \
     0x00005810f0LL, 0x60021640LL, 0x00000040LL, \
     0x00005810f0LL, 0x60021680LL, 0x00000040LL, \
     0x00005810f0LL, 0x600216c0LL, 0x00000040LL, \
     0x00005810f0LL, 0x60021700LL, 0x00000040LL, \
     0x00005810f0LL, 0x60021740LL, 0x00000040LL, \
     0x00005810f0LL, 0x60021780LL, 0x00000040LL, \
     0x00005810f0LL, 0x600217c0LL, 0x00000040LL, \
     0x0000ab10f0LL, 0x60024000LL, 0x00001000LL, \
     0x00004f10f0LL, 0x60030000LL, 0x00001000LL, \
     0x00005510f0LL, 0x60031000LL, 0x00001000LL, \
     0x00004f11f0LL, 0x60032000LL, 0x00001000LL, \
     0x00004f12f0LL, 0x60033000LL, 0x00001000LL, \
     0x00004f13f0LL, 0x60034000LL, 0x00001000LL, \
     0x00004f14f0LL, 0x60035000LL, 0x00001000LL, \
     0x00004f15f0LL, 0x60036000LL, 0x00001000LL, \
     0x00004f16f0LL, 0x60037000LL, 0x00001000LL, \
     0x00009f00f0LL, 0x61000000LL, 0x07000000LL, \
     0x00009f00f0LL, 0x69000000LL, 0x07000000LL, \
     0x00000021f0LL, 0x70000000LL, 0x00004000LL, \
     0x00000010f0LL, 0x7000a000LL, 0x00001000LL, \
     0x00008110f0LL, 0x70016000LL, 0x00002000LL, \
     0x00000020f0LL, 0x70019000LL, 0x00001000LL, \
     0x00000020f0LL, 0x70019000LL, 0x00001000LL, \
     0x00000013f0LL, 0x7001b000LL, 0x00001000LL, \
     0x00000013f0LL, 0x7001b000LL, 0x00001000LL, \
     0x00000010f0LL, 0x7001c000LL, 0x00001000LL, \
     0x00000010f0LL, 0x7001d000LL, 0x00001000LL, \
     0x00000010f0LL, 0x7001e000LL, 0x00001000LL, \
     0x00000010f0LL, 0x7001f000LL, 0x00001000LL, \
     0x0000aa10f0LL, 0x70120000LL, 0x00001000LL, \
     0x00006a11f0LL, 0x70225000LL, 0x00001000LL, \
     0x00000010f0LL, 0x70500000LL, 0x00010000LL, \
     0x00000010f0LL, 0x70510000LL, 0x00010000LL, \
     0x00000010f0LL, 0x70520000LL, 0x00010000LL, \
     0x00000010f0LL, 0x70530000LL, 0x00010000LL, \
     0x00000010f0LL, 0x70540000LL, 0x00010000LL, \
     0x00000010f0LL, 0x70600000LL, 0x00010000LL, \
     0x00000010f0LL, 0x70610000LL, 0x00010000LL, \
     0x00000010f0LL, 0x70620000LL, 0x00010000LL, \
     0x00000010f0LL, 0x70630000LL, 0x00010000LL, \
     0x00000010f0LL, 0x70640000LL, 0x00010000LL, \
     0x00000010f0LL, 0x70650000LL, 0x00010000LL, \
     0x00000010f0LL, 0x70660000LL, 0x00010000LL, \
     0x00000010f0LL, 0x70670000LL, 0x00010000LL, \
     0x00000010f0LL, 0x70680000LL, 0x00010000LL, \
     0x00000010f0LL, 0x70690000LL, 0x00010000LL, \
     0x00009f00f0LL, 0x77000000LL, 0x01000000LL, \
     0x00009f00f0LL, 0x78000000LL, 0x01000000LL, \
     0x00009f00f0LL, 0x79000000LL, 0x03000000LL, \
     0x00006010f0LL, 0x7c000000LL, 0x00010000LL, \
     0x00007d10f0LL, 0x7c010000LL, 0x00010000LL, \
     0x00009f00f0LL, 0x7e000000LL, 0x02000000LL, \
     0x00000200f0LL, 0x80000000LL, 0x380000000LL, \
     0x0000000000LL, \
     0x0000000000LL

//////////////////////////////////////////////////////////////////
//
//
//
// END OF RELOCATION TABLE (DEPRECATED)
//
//
//
//////////////////////////////////////////////////////////////////

