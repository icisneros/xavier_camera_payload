/*
 * Copyright (c) 2014, NVIDIA CORPORATION. All rights reserved.
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

/**
 * NvBootInfoTable (BIT) provides information from the Boot ROM (BR)
 * to Bootloaders (BLs).
 *
 * Initially, the BIT is cleared.
 *
 * As the BR works its way through its boot sequence, it records data in
 * the BIT.  This includes information determined as it boots and a log
 * of the boot process.  The BIT also contains a pointer to an in-memory,
 * plaintext copy of the Boot Configuration Table (BCT).
 *
 * The BIT allows BLs to determine how they were loaded, any errors that
 * occured along the way, and which of the set of BLs was finally loaded.
 *
 * The BIT also serves as a tool for diagnosing boot failures.  The cold boot
 * process is necessarily opaque, and the BIT provides a window into what
 * actually happened.  If the device is completely unable to boot, the BR
 * will enter Recovery Mode (RCM), using which one can load an applet that
 * dumps the BIT and BCT contents for analysis on the host.
 */

#ifndef INCLUDED_NVBOOT_BIT_H
#define INCLUDED_NVBOOT_BIT_H

#include "nvcommon.h"
#include "nvboot_bct.h"
#include "nvboot_config.h"


#if defined(__cplusplus)
extern "C"
{
#endif

/// Specifies the amount of status data needed in the BIT.
/// One bit of status is used for each of the journal blocks,
/// and 1 additional bit is needed for the second BCT in block 0.
#define NVBOOT_BCT_STATUS_BITS  (NVBOOT_MAX_BCT_SEARCH_BLOCKS + 1)
#define NVBOOT_BCT_STATUS_BYTES ((NVBOOT_BCT_STATUS_BITS + 7) >> 3)

/// This volatile write makes sure two consecutive writes to BR
/// tracker are not optimized away.
#define BIT_TRACKER(x) \
    (*(volatile NvBootFlowStatus*)&BootInfoTable.BootROMtracker=x)

/**
 * Defines the type of boot.
 * Note: There is no BIT for warm boot.
 */
typedef enum
{
    /// Specifies a default (unset) value.
    NvBootType_None = 0,

    /// Specifies a cold boot
    NvBootType_Cold,

    /// Specifies a warm boot
    NvBootType_Warm,

    /// Specifies the BR entered RCM
    NvBootType_Recovery,

    /// Specifies UART boot (only available internal to NVIDIA)
    NvBootType_Uart,

    /// Specifies that the BR immediately exited for debugging
    /// purposes.
    /// This can only occur when NOT in ODM production mode,
    /// and when a special BOOT_SELECT value is set.
    NvBootType_ExitRcm,

    NvBootType_Force32 = 0x7fffffff
} NvBootType;

/**
 * Defines the information recorded about each BL.
 *
 * BLs that do not become the primary copy to load have status of None.
 * They may still experience Ecc errors if used to recover from ECC
 * errors of another copy of the BL.
 */
typedef struct NvBootBlStateRec
{
    /// Specifies the outcome of attempting to load this BL.
    NvU32           Status;

    /// Specifies the first block that experienced an ECC error, if any.
    /// 0 otherwise.
    NvU32           FirstEccBlock;

    /// Specifies the first page that experienced an ECC error, if any.
    /// 0 otherwise.
    NvU32           FirstEccPage;

    /// Specifies the first block that experienced a correctable ECC error,
    /// if any. 0 otherwise. Only correctable errors that push the limits of
    /// the ECC algorithm are recorded (i.e., those very likely to become
    /// uncorrectable errors in the near future).
    NvU32           FirstCorrectedEccBlock;

    /// Specifies the first page that experienced a correctable ECC error,
    /// if any. 0 otherwise. Similar to FirstCorrectedEccBlock.
    NvU32           FirstCorrectedEccPage;

    /// Specifies if the BL experienced any ECC errors.
    NvBool          HadEccError;

    /// Specifies if the BL experienced any CRC errors.
    NvBool          HadCrcError;

    /// Specifies if the BL experienced any corrected ECC errors.
    /// As with FirstCorrectedEcc*, only nearly-uncorrectable errors count.
    NvBool          HadCorrectedEccError;

    /// Specifies if the BL provided data for another BL that experienced an
    /// ECC error.
    NvBool          UsedForEccRecovery;
} NvBootBlState;

/**
 * Defines the status from eMMC and eSD
 */
typedef struct NvBootSdmmcStatusRec
{
    ///
    /// Parameters specified by fuses or straps.
    ///

    /// Specifies the clock source.
    NvU8 ModuleClkSource;

    /// Specifies the clk divisor.
    NvU8 ModuleClkDivisor;

    /// Specifies the clk enable.
    NvU8 ModuleClkEnable;

    /// Specifies the module reset status.
    NvU8 ModuleClkRstStatus;

    /// Specifies the clk internal divisor for interface via fuses or straps.
    NvU8 SdmmcIntrlClkDivisor;

    /// Specifies the data transfer mode.
    NvU8 SdmmcDataMode;

    /// Specifies the data width specified by fuses or straps.
    NvU8 FuseDataWidth;

    /// Specifies the Ddr mode specified by fuses or straps.
    NvBool FuseDdrMode;

    /// Specifies the Config option specified by fuses or straps.
    NvU8 FuseConfig;

    /// Specifies the Read multi/Single page option from config option.
    NvBool FuseReadMode;

    ///
    /// Parameters discovered during operation
    ///

    /// Specifies the discovered card type
    NvU8 DiscoveredCardType;

    /// Specifies the discovered voltage range
    NvU32 DiscoveredVoltageRange;

    /// Specifies the data width chosen to conform to power class constraints
    NvU8 DataWidthUnderUse;

    /// Specifies the power class chosen to conform to power class constraints
    NvU8 PowerClassUnderUse;

    /// Specifies the Auto Cal status
    NvBool AutoCalStatus;

    ///
    /// Parameters provided by the device
    ///

    /// Specifies the card identification data.
    NvU32 Cid[4];

    ///
    /// Information for driver validation
    ///

    /// Specifies the number of pages read from the beginning.
    NvU32 NumPagesRead;

    /// Specifies the # of CRC errors
    NvU32 NumCrcErrors;

    /// Specifies whether the boot was attempted from a Boot Partition.
    NvU8 BootFromBootPartition;

    /// Overall sdmmc hw init and media enumeration time
    NvU32 SdmmcInit;
    
    /// Controller, clock/resets & pad initialization.
    NvU32 SdmmcControllerInit;
    
    ///     eMMC device Identification/Transfer state transition time
    NvU32 eMMCDeviceEnumeration;
    
    /// Get Operating condition command 1 time, device power up
    NvU32 eMMCCmd1;
    
    /// Retrieve CID from attached device via Cmd2
    NvU32 eMMC_ALL_SEND_CID;
    
    /// Retrieve Card Specific Data via SEND_CSD(CMD9) Command.
    NvU32 eMMC_CSD;
    
    /// Place attached card into transfer mode from identification mode 
    NvU32 eMMC_transfer_mode;
    
    /// Obtain Extended Card specific data
    NvU32 eMMC_ExtCsd;
    
    /// PowerClass settings based on BCT.
    NvU32 eMMC_PowerClass;
    
    /// Select Bus width and partition setup time.
    NvU32 eMMC_WidthPartitionSetup;
    
    ///Data read time for payload (data throughput)
    NvU32 ReadTime;
    ///Associated Data length in bytes for ReadTime
    NvU32 Payload;
    
} NvBootSdmmcStatus;
/**
 * Defines the status from SPI flash devices
 * Overlay this structure with space reserved for this purpose in BIT
 *  SecondaryDevStatus[NVBOOT_SIZE_DEV_STATUS]
 */
typedef struct NvBootSpiFlashStatusRec
{
    /// Clk enable for module
    NvU32 ClkEnable;

    /// for module reset asserted
    NvU32 ClkRstStatus;

    /// transfer mode pio/dma
    NvU32 Mode;

    /// Interface bus width x1/x4
    NvU32 DataWidth;

    /// Specifies the chosen clock source
    NvU32 ClockSource;

    /// Specifies the chosen clock divider
    NvU32 ClockDivider;

    /// Specifies whether fast read was selected
    NvU32 IsFastRead;

    /// Specifies the number of pages read from the beginning.
    NvU32 NumPagesRead;

    /// Specifies the last block read
    NvU32 LastBlockRead;
    /// Specifies the last page read
    NvU32 LastPageRead;

    /// Specifies the boot status
    NvU32 BootStatus;

    /// Specifies the init status
    NvU32 InitStatus;

    /// Specifies the read status
    NvU32 ReadStatus;

    /// Specifies whether parameters successfully validated
    NvU32 ParamsValidated;

    /// Overall Qspi hw controller init, clocks/resets, pads
    NvU32 QspiInit;

    /// Data read time for payload (data throughput)
    NvU32 ReadTime;

    /// Associated Data length in bytes for ReadTime
    NvU32 Payload;
} NvBootSpiFlashStatus;

/**
 * Defines the status from UFS
 * Overlay this structure with space reserved for this purpose in BIT
 * SecondaryDevStatus[NVBOOT_SIZE_DEV_STATUS]
 */
typedef struct NvBootUfsStatusRec
{
    ///
    /// Parameters specified by fuses or straps.
    ///
    /// Specifies the clock source.
    uint8_t ModuleClkSource;

    /// Specifies the clk divisor.
    uint8_t ModuleClkDivisor;

    /// Specifies the clk enable.
    uint8_t ModuleClkEnable;

    /// Specifies the module reset status.
    uint8_t ModuleClkRstStatus;

    /// Specifies the data transfer mode.
    uint8_t UfsSpeedMode;

    /// Specifies the Config option specified by fuses or straps.
    uint8_t FuseUfsConfig;

    ///
    /// Parameters discovered during operation
    /// Boot Enabled info retrieved from device descriptor
    uint8_t BootEnabled;

    /// Active Logical Unit Number info retrieved from device descriptor
    uint8_t NumLun;

    /// Active Boot Logical Unit Number info retrieved from device descriptor
    uint8_t NumWLU;

    /// Boot Logical unit number
    uint8_t BootLun;

    ///Active phy lanes configured for data transfer
    uint8_t ActiveLanes;

    /// Phy lanes in config 0 (2) and config 1 (1)
    uint8_t NumLanes;

    /// Driver error status for UFS commands for media Init  and Read
    uint8_t UfsDriverStatus;

    /// Query device status updated during Read commands.
    uint8_t DeviceStatus;

    /// Ufs device controller and media reinitialized
    uint8_t DeviceReintialized;

    /// Ufs device controller and media reinitialized
    uint8_t AutoSlowMode;

    ///
    /// Parameters provided by the device
    ///
    /// Covers overall (NvBootUfsInit) hw init and media enumeration time.
    uint32_t UfsDeviceControllerInit;
    
    /// Clock/resets, pad, phy/lane initialization. 
    uint32_t UfsDeviceHwControllerInit;
    
    /// Initialize Uphy Pll's and Lanes based on NumLanes (NvBootUfsLinkUphySetup)
    uint32_t UfsLinkUphyInit;
    
    /// Initialize Mipi phy (NvBootUfsLinkMphySetup)
    uint32_t UfsLinkMphyInit;
    
    /// Profile stamp for device to be ready to receive UPIU's (NvBootUfsChkIfDevRdy2RcvDesc).
    uint32_t UfsDevicecheck;
    
    /// change number of lanes (reinit).
    uint32_t UfsDeviceConfigureLanes;
    
    /// Change speed mode from G1 to G4 (fuse/reinit) 
    uint32_t UfsDeviceConfigurePWMGear;
    
    ///Data read time for payload (data throughput)
    uint32_t ReadTime;
    ///Associated Data length in bytes for ReadTime
    uint32_t Payload;

    ///
    /// Information for driver validation
    ///

} NvBootUfsStatus;



/**
 * Defines the status from SATA
 * Overlay this structure with space reserved for this purpose in BIT
 * SecondaryDevStatus[NVBOOT_SIZE_DEV_STATUS]
 */
typedef struct NvBootSataStatusRec
{

    /// Specifies the sata and sata oob clock sources
    /// SataClockSource[15:0] : Sata clock source
    /// SataClockSource[31:16]: Sata Oob clock source
    NvU32 SataClockSource;

    /// Specifies the sata clock divider
    NvU32 SataClockDivider;

    /// Specifies the sata oob clock divider
    NvU32 SataOobClockDivider;

    /// Specifies the last used mode
    /// SataMode[15:0] : of type NvBootSataMode, could be AHCI or Legacy
    /// SataMode[31:16]: of type NvBootSataTransferMode, could be PIO or DMA
    NvU32 SataMode;

    /// Specifies the init status
    /// InitStatus[5:0] : Initialization status (makes use of the fact that
    ///                   NvBootError has notmore than 48 enumerated values)
    /// InitStatus[6:6] : Sata reinitialized then 1, else 0
    /// InitStatus[7:7] : Plle reinitialized then 1, else 0
    /// InitStatus[8:8] : Plle init failed then 1, else 0
    /// InitStatus[9:9] : Plle SS failed then 1, else 0
    /// InitStatus[10:10] : Pllrefe init failed then 1, else 0
    /// InitStatus[11:11] : Com reset failed (cominit was not received within the
    ///                     expected timelimit)  then 1, else 0
    /// InitStatus[12:12] : SSD detection failed then 1, else 0
    /// InitStatus[13:13] : Pad pll not locked then 1, else 0
    /// InitStatus[14:14] : Params validated
    /// InitStatus[15:15] : Uphy pll cal failed then 1, else 0
    /// InitStatus[16:16] : Uphy pll cal not cleared then 1, else 0
    /// InitStatus[17:17] : Uphy pll rcal failed then 1, else 0
    /// InitStatus[18:18] : Uphy pll rcal not cleared then 1, else 0
    NvU32 InitStatus;

    /// Specifies the number of pages read from the beginning.
    NvU32 NumPagesRead;

    /// Specifies the last page read
    NvU32 LastBlockRead;

    /// Specifies the last page read
    NvU32 LastPageRead;

    /// Specifies the Port Error that occurred.
    /// PortStatus[0:0] : SataStatus_CommandListRunning
    /// PortStatus[1:1] : SataStatus_DrqHigh
    /// PortStatus[2:2] : SataStatus_BsyHigh
    /// PortStatus[3:3] : SataStatus_PortBsy
    NvU32 PortStatus;

    /// Specifies the sata buffers base
    NvU32 AhciSataBuffersBase;

    /// Specifies the data buffers base for ahci dma
    NvU32 AhciDataBuffersBase;

    /// Specifies if Ahci Dma Status
    /// AhciDmaStatus[0:0] : Ahci Dma transfer could not be completed within a
    /// stipulated time then 1, else 0
    /// AhciDmaStatus[1:1] : SDB FIS was not received then 1, else 0
    /// AhciDmaStatus[2:2] : An error occurred during ahci dma transfer then 1,
    /// else 0. If this bit is set, look at other fields of Bit sata status to
    /// ascertain the type of Ahci Dma Error that occurred.
    /// AhciDmaStatus[3:3] : equivalent of AhciDataXmissionError in T30
    /// Specifies if there was an ahci data transmission error
    /// AhciDmaStatus[4:4] : equivalent of AhciCommandError in T30
    /// Specifies if there was a command error in case the last transaction
    /// was of ahci dma type.
    /// AhciDmaStatus[5:5] : equivalent of AhciTfdError in T30
    /// Specifies the Task File Data Error that occurred in case the last transaction
    /// was of ahci dma type
    NvU32 AhciDmaStatus;

} NvBootSataStatus;


/*
 * status structures of all the supported secondary boot device types.
 */
typedef union NvBootSecondaryDeviceStatusRec
{
    /// Specifies the status from eMMC
    NvBootSdmmcStatus       SdmmcStatus;

    /// Specifies the status from SPI flash
    NvBootSpiFlashStatus    SpiStatus;

    /// Specifies the status from Ufs
    NvBootUfsStatus         UfsStatus;

    /// Specifies the status from Sata
    NvBootSataStatus        SataStatus;

} NvBootSecondaryDeviceStatus;


typedef struct NvBootFlowLogRec
{
    // Init timestamp
    NvU32   NvBootFlowLogInit;
    // Exit timestamp
    NvU32   NvBootFlowLogExit;
    // Function Id
    NvU32   NvBootFlowFuncId;   // every function in the dispatcher is provided with id.
    // Fuction Exit Status log
    NvU32   NvBootFlowFuncStatus; // function exit status
} NvBootFlowLog;

 /**
  * Defines the status codes pertaining to BootROM flow.
 *  Each important component state is logged as the BR proceeds
 */
typedef enum
{
    /// Specifies a default (unset) value.
    NvBootFlowStatus_None = 0,

    /// Specifies IromPatch uncorrected error
    NvBootFlowStatus_IPatchUncorrected_error,

    /// Specifies IromPatch Success.
    NvBootFlowStatus_IPatchSuccess,

    NvBootFlowStatus_PiromRegionSetupDone,

    /// Nonsecure task flow
    /// Specifies Oscillator setup.
    NvBootFlowStatus_SetupOscClk,

    /// Specifies AON RAMs active configuration.
    NvBootFlowStatus_AonRams_UnPG,

    /// Specifies Low Battery, under charged.
    NvBootFlowStatus_LowBat_NotCharged,

    /// Specifies Low Battery Success.
    NvBootFlowStatus_LowBat_Charged,

    /// Specifies PLLP clock enable.
    NvBootFlowStatus_PllpEnabled,

    /// Specifies Sysram/roc MSS reset deassertion
    NvBootFlowStatus_MSSInitialized,

    /// Specifies System Fabric initialisation
    NvBootFlowStatus_FabricInitialized,

    /// Specifies a entry to Non Secure Dispatcher routine.
    NvBootFlowStatus_NonSecureDispatcherEntry,

    /// Specifies a exiting from Non Secure Dispatcher routine.
    NvBootFlowStatus_NonSecureDispatcherExit,

    /// Specifies FA mode of operation
    NvBootFlowStatus_FAMode,

    /// Specifies Pre-production UART Mode
    NvBootFlowStatus_PreproductionModeUart,

    /// Specifies NV-production Mode
    NvBootFlowStatus_ProductionMode,

    /// Specifies ODM production Mode
    NvBootFlowStatus_OdmproductionMode,

    /// Specifies Debug RCM Mode
    NvBootFlowStatus_DbgRCMMode,

    /// Specifies Recovery Mode of operation.
    NvBootFlowStatus_RecoveryMode,

    NvBootFlowStatus_PmicSequencingCompleted,
    NvBootFlowStatus_EnableFaultResetGen,
    NvBootFlowStatus_AonRamsOutOfReset,
    NvBootFlowStatus_BpmpWdtEnabled,
    NvBootFlowStatus_GlobalClockOverrideDone,


    /// Secure task flow
    /// Specifies a entry to Secure Dispatcher routine.
    NvBootFlowStatus_SecureDispatcherEntry,

    /// Specifies a Exit from Secure Dispatcher routine.
    NvBootFlowStatus_SecureDispatcherExit,

    /// Specifies a entry to Ramdump routine.
    NvBootFlowStatus_RamDumpInit,

    /// Specifies a entry to Ramdump setup done.
    NvBootFlowStatus_RamDumpExit,

    NvBootFlowStatus_Apb2jtagPatchDone,
    NvBootFlowStatus_FuseKeyDecryptionDone,


    /// Coldboot task flow
    /// Specifies a entry to Cold Boot routine.
    NvBootFlowStatus_ColdBootEntry,

    /// Specifies a exit from Cold Boot routine.
    NvBootFlowStatus_ColdBootExit,

    /// Specifies Setup Boot device Init success.
    NvBootFlowStatus_CBSetupBootDevice,

    /// Specifies BCT Read success.
    NvBootFlowStatus_CBBctDone,

    /// Specifies Mss Init success.
    NvBootFlowStatus_MssRegionUnInitialized,
    NvBootFlowStatus_MssRegionEnableInitialized,

    /// Specifies Mts Init success. if supported.
    NvBootFlowStatus_CBMtsPrebootInit,

    /// Specifies Reinit success, If supported.
    NvBootFlowStatus_CBReinitSuccess,

    /// Specifies Sdram Init success, If supported.
    NvBootFlowStatus_CBSdramInitSuccess,

    /// Specifies Payload load success.
    NvBootFlowStatus_CBPayloadSuccess,

    /// Specifies a entry to Secure Boot routine.
    NvBootFlowStatus_SecureBootEntry,

    /// Specifies a Exit from Secure Boot routine.
    NvBootFlowStatus_SecureBootExit,

    NvBootFlowStatus_MssGenerationDone,
    NvBootFlowStatus_CBSetBctClocksDone,
    NvBootFlowStatus_CBReadBct,
    NvBootFlowStatus_CBAuthenticateBct,
    NvBootFlowStatus_CBDecryptBct,
    NvBootFlowStatus_CBReadMb1,
    NvBootFlowStatus_AuthenticateOEMBCH,
    NvBootFlowStatus_AuthenticateNVBCH,
    NvBootFlowStatus_AuthenticateOEMMB1,
    NvBootFlowStatus_AuthenticateNVMB1,
    NvBootFlowStatus_DecryptOEMMB1,
    NvBootFlowStatus_DecryptNVMB1,
    NvBootFlowStatus_ReadBackupMB1,
    NvBootFlowStatus_ChangeBootChainMB1,

    /// Warm boot flow
    /// Specifies a entry to Wb0 routine.
    NvBootFlowStatus_Sc7Entry,

    /// Specifies a error in sc7 mrom acknowledge waypoint.
    NvBootFlowStatus_Sc7AckWaypoint,

    /// Specifies a error in sc7 Doorbell hw timeout.
    NvBootFlowStatus_Sc7DbellError,

    /// Specifies a exit from Wb0 routine.
    NvBootFlowStatus_Sc7Exit,

    NvBootFlowStatus_Sc7PllsConfigured,
    NvBootFlowStatus_Sc7CpuFabricConfigured,
    NvBootFlowStatus_Sc7SdramInit,
    NvBootFlowStatus_Sc7LoadMts,
    NvBootFlowStatus_Sc7AssertWayPoint,
    NvBootFlowStatus_Sc7RecoveryOemValidated,
    NvBootFlowStatus_Sc7RecoveryNvValidated,
    NvBootFlowStatus_Sc7RecoveryOEMDecrypt,
    NvBootFlowStatus_Sc7FetchResumeFwFromSdram,
    NvBootFlowStatus_SEInit,
    NvBootFlowStatus_SEContextRestore,
    NvBootFlowStatus_SRKRestoreDone,


    /// RCM flow
    /// Specifies a entry to Rcm routine.
    NvBootFlowStatus_RcmEntry,

    /// Specifies exit from RCM routine.
    NvBootFlowStatus_RcmExit,

    NvBootFlowStatus_RcmDetectedViaStraps,
    NvBootFlowStatus_RcmDetectedViaScratch,
    NvBootFlowStatus_RcmSentEcidToHost,
    NvBootFlowStatus_RcmUsbSsInit,
    NvBootFlowStatus_RcmUsbHsInit,
    NvBootFlowStatus_RcmDetectedValidUsbSsConnection,
    NvBootFlowStatus_RcmDetectedValidUsbHsConnection,


    /// Specifies a Reset Status Based events.
    /// Detected SC7 Reset Source with AOWAKE[SPE_WDT]=1
    NvBootFlowStatus_DetectedSc7SpeWdt_1_Reset,

    /// Detected SC7 Reset Source with AOWAKE[SPE_WDT]=0
    NvBootFlowStatus_DetectedSc7SpeWdt_0_Reset,

    NvBootFlowStatus_DetectedWDTReset,
    NvBootFlowStatus_DetectedWDTL2Reset,
    NvBootFlowStatus_DetectedAoTag_SensorReset,
    NvBootFlowStatus_DetectedVfSensorReset,
    NvBootFlowStatus_DetectedSwMainReset,
    NvBootFlowStatus_DetectedHsmReset,
    NvBootFlowStatus_DetectedCsiteDbgReset,
    NvBootFlowStatus_DetectedSc8,
    NvBootFlowStatus_DetectedSysResetN,
    NvBootFlowStatus_DetectedL1aAsync,
    NvBootFlowStatus_DetectedBpmpBoot,
    NvBootFlowStatus_DetectedFuseCrc,
    NvBootFlowStatus_DetectedRtcRail_Violation,
    NvBootFlowStatus_InitI2c5_Success,
    NvBootFlowStatus_InitI2c5_Error,

    
    /// Specifies boot device initialization events.
    NvBootFlowStatus_BootDevQspiInit,
    NvBootFlowStatus_BootDevSdmmcInit,
    NvBootFlowStatus_BootDevSataInit,
    NvBootFlowStatus_BootDevUfsInit,
    NvBootFlowStatus_BootDevUartInit,


    NvBootFlowStatus_Force32 = 0x7fffffff
} NvBootFlowStatus;


/**
 * Bootrom flow time, payload read/auth/decryption time 
 * and length profiling 
 */

typedef struct NvBootProfilingLogRec {
    // Init timestamp
    NvU32   NvBootTimeLogInit;
    // Exit timestamp
    NvU32   NvBootTimeLogExit;
    // Generic Config times
    // Early - time from init of us timer to DeviceInitTime
    // Late - time for secure exit (only covers C subroutines)
    NvU32   EarlyBRConfig;
    NvU32   LateBRConfig;

    NvU32   BootDeviceInitTime;
    NvU32   BootDeviceReinitTime;

    // Bct read/auth/decryption time
    NvU32   BctReadTime;

    NvU32   BctSignedLen;
    NvU32   BctAuthTime;

    NvU32   BctEncryptedLen;
    NvU32   BctDecryptTime;

    // Bct stage 1/stage 2 auth/decryption time
    NvU32   BchStage2Len;
    NvU32   BchStage2AuthTime;

    NvU32   BchStage1Len;
    NvU32   BchStage1AuthTime;

    // Mb1 read/stage 1/stage 2 auth/decryption time
    NvU32   Mb1ReadTime;

    NvU32   Mb1Stage2Len;
    NvU32   Mb1Stage2AuthTime;
    NvU32   Mb1Stage2DecryptTime;

    NvU32   Mb1Stage1Len;
    NvU32   Mb1Stage1AuthTime;
    NvU32   Mb1Stage1DecryptTime;

    // Rcm-related auth/decryption time
    NvU32   RcmMsgReceiveTime;
    
    NvU32   RcmHeaderLen;
    NvU32   RcmHeaderAuthTime;

    NvU32   RcmPayloadLen;
    NvU32   RcmPayloadAuthTime;

    // Sc7-related auth/decryption time
    NvU32   Sc7HeaderStage2Len;
    NvU32   Sc7HeaderStage2AuthTime;

    NvU32   Sc7HeaderStage1Len;
    NvU32   Sc7HeaderStage1AuthTime;

    NvU32   Sc7RfStage2Len;
    NvU32   Sc7RfStage2AuthTime;
    NvU32   Sc7RfStage2DecryptTime;

    NvU32   Sc7RfStage1Len;
    NvU32   Sc7RfStage1AuthTime;
} NvBootProfilingLog;


/**
 * Defines the BIT.
 *
 * Notes:
 * * SecondaryDevice: This is set by cold boot (and soon UART) processing.
 *   Recovery mode does not alter its value.
 * * BctStatus[] is a bit vector representing the cause of BCT read failures.
 *       A 0 bit indicates a validation failure
 *       A 1 bit indicates a device read error
 *       Bit 0 contains the status for the BCT at block 0, slot 0.
 *       Bit 1 contains the status for the BCT at block 0, slot 1.
 *       Bit N contains the status for the BCT at block (N-1), slot 0, which
 *       is a failed attempt to locate the journal block at block N.
 *       (1 <= N < NVBOOT_MAX_BCT_SEARCH_BLOCKS)
 */
typedef union  NvBootInfoTableRec
{
    NvU32 BitForceSize[NVBOOT_BIT_REQUIRED_SIZE/4];
    
    struct
    {

        ///
        /// Version information
        ///

        /// Specifies the version number of the BR code.
        /// !! NOTE !!
        /// This MUST be the first entry in the BIT. BR
        /// uses this field to temporarily store some exit configuration
        /// data. BR will set the correct Boot ROM version number
        /// just before exit.
        ///
        NvU32               BootRomVersion;

        /// Specifies the version number of the BR data structure.
        NvU32               DataVersion;

        /// Specifies the version number of the RCM protocol.
        NvU32               RcmVersion;

        /// Specifies the type of boot.
        NvBootType          BootType;

        /// Specifies the primary boot device.
        NvBootDevType       PrimaryDevice;

        /// Specifies the secondary boot device.
        NvBootDevType       SecondaryDevice;

        /// Specifies the Authentication scheme
        NvU32               AuthenticationScheme;
        
        /// Specifies the Encryption scheme
        NvBool              EncryptionEnabled;

        /// BootROM flow tracker
        NvBootFlowStatus    BootROMtracker;

        /// Boot flow logging
        NvU32               BootFlowLogIndex;
        NvBootFlowLog       BootFlowLog[NVBOOT_FLOW_LOG_DEPTH];

        /// Bootrom profiling
        NvBootProfilingLog  ProfilingLog;

        ///
        /// Hardware status information
        ///

        /// Specifies whether the device was initialized.
        NvBool              DevInitialized;

        /// Specifies whether the Preboot was initialized.
        NvBool              PrebootInitialized;

        /// Specifies whether SDRAM was initialized.
        NvBool              SdramInitialized;

        /// Specifies whether the ForceRecovery AO bit was cleared.
        NvBool              ClearedForceRecovery;

        /// Specifies if in FactoryServiceProvisioningMode for coldboot and RCM for MB1.
        NvBool              FactorySecureProvisioningMode;

        /// Specifies whether which Boot chain is used. 0 = Primary; 1 = Secondary
        uint8_t             BootChainUsed;

        /// IROM patch status
        /// [3:0] - Hamming decode status
        /// [6:4] - Reserved '0'
        /// [7:7] - IROM patch fuse payload present
        uint8_t             IRomPatchStatus;

        ///
        /// BCT information
        ///

        /// Specifies if a Size of BCT was found to be valid.
        NvBool              BctSizeValid;

        /// Specifies the status of attempting to read Size of BCT during the
        /// initial search process.  See the notes above for more details.
        uint8_t             BctSizeStatus[NVBOOT_BCT_STATUS_BYTES];

        /// Specifies the block number in which the Size of BCT was validated.
        NvU32               BctSizeBlock;

        /// Specifies the page number of the start of the Size of BCT was validated.
        NvU32               BctSizePage;

        /// Specifies if a valid BCT was found.
        NvBool              BctValid;

        /// Specifies the status of attempting to read BCTs during the
        /// BCT search process.  See the notes above for more details.
        uint8_t             BctStatus[NVBOOT_BCT_STATUS_BYTES];

        /// Specifies the block number in which the BCT was found.
        NvU32               BctBlock;

        /// Specifies the page number of the start of the BCT that was found.
        NvU32               BctPage;

        /// Specifies the size of the BCT in bytes.  It is 0 until BCT loading
        /// is attempted.
        NvU32               BctSize;  /* 0 until BCT loading is attempted */

        /// Specifies a pointer to the BCT in memory.  It is NULL until BCT
        /// loading is attempted.  The BCT in memory is the last BCT that
        /// the BR tried to load, regardless of whether the operation was
        /// successful.
        NvU32              BctPtr;

        /// Specifies the state of attempting to load each of the BLs.
        NvBootBlState       BlState[NVBOOT_MAX_BOOTLOADERS];

        /// Specifies device-specific status information from the operation
        /// of the secondary boot device. Size define nvboot_config.h
        NvU32               SecondaryDevStatus[NVBOOT_DEV_STATUS_SIZE_BYTES/4];

        /// Specifies the status of usb charger detection
        /// [0:0] = ChargerDetectionEnabled (0 if disabled, 1 if enabled)
        /// [1:1] = IsBatteryLowDetected (1 if low, 0 if high)
        /// [2:2] = IsUsbCableConnected (1 if usb cable is connected, else 0)
        /// [3:3] = RESERVED. Was IsChargingPort (1 if charging port, else 0 if SDP)
        /// [4:4] = RESERVED. Was IsDividerChargerDetected (1 if divider charger detected, else 0)
        /// [5:5] = RESERVED. Was IsACADetected (1 if ACA A/B/C detected, else 0)
        /// [6:6] = RESERVED. Was IsNonCompliantChargerDetected (1 if charger is non-compliant, else 0)
        /// [7:7] = RESERVED. Was IsDcpCdpDetected (1 if DCP/CDP, else 0)
        /// [8:8] = PmicHighCurrentStatusAsserted (1 if driven high, 0 if low)
        /// [9:9] = RESERVED. Was DeviceEnumerationComplete (1 if device enumeration performed)
        /// [10:10] = RESERVED. Was DeviceEnumerationFailed (1 if device enum failed, else 0)
        /// [11:11] = RESERVED. Was ReadPMICUsbChargerControlForHiCurrentFailed (set to 1 if read/write failed, else 0)
        /// [12:12] = RESERVED. Was UsbChargingBITStat_ReadPMICUsbChargerControlForUsbSuspendFailed (set to 1 if
        ///        read/write failed, else 0)
        /// [13:13] = RESERVED. Was UsbChargingBITStat_Timer0Disabled (set to 1 if Timer0 disabled, else 0)
        NvU32               UsbChargingStatus;

        /// NvBootError_Success implies reading the BOOT_SEL
        /// register from the PMIC was successful.
        NvBool              PmuBootSelReadError;

        /// Specify the desire to debug MB1 post NV_PRODUCTION and
        /// before SECURITY_MODE is fused (OEM production).
        NvBool      MB1DebugProduction;

        /// Specify the desire to debug MTS post NV_PRODUCTION and before
        /// SECURITY_MODE is fused (OEM production);
        /// MTSDebugProduction support in MB1 is NOT POR, but MTS Debug Keys
        /// are still implemented in IROM secure region and BR will still load
        /// the keys according to this bit set.
        NvBool      MTSDebugProduction;

        /// Specify the desire to debug IST-FW post NV_PRODUCTION and before
        /// SECURITY_MODE is fused (OEM production);
        NvBool      ISTFWDebugProduction;

        /// Specifies the lowest iRAM address that preserves communicated data.
        /// SafeStartAddr starts out with the address of memory following
        /// the BIT.  When BCT loading starts, it is bumped up to the
        /// memory following the BCT.
        NvU32               SafeStartAddr;

        /// Specifies soft fuses loaded
        NvBool              SoftFusesLoaded;

        /// Specifies a reserved area at the end of the BIT.
        uint8_t             Reserved[NVBOOT_BIT_RESERVED_SIZE];
    };
} NvBootInfoTable;

#if defined(__cplusplus)
}
#endif

#endif /* #ifndef INCLUDED_NVBOOT_BIT_H */
