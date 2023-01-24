/*
 * Copyright (c) 2007-2012, NVIDIA CORPORATION. All rights reserved.
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
 * Recovery Mode (RCM) is the process through which fuses can be burned and
 * applets loaded directly into IRAM.  All RCM communication is over USB.
 * RCM is intended for system bringup and failure diagnosis.
 *
 * RCM is entered under any of the following conditions:
 * 1. The recovery mode strap is tied high.
 * 2. The AO bit is set (see nvboot_pmc_scratch_map.h for details).
 * 3. The Boot ROM (BR) encounters an uncorrectable error during booting.
 *    These conditions include trying to read a BCT from an empty storage
 *    medium and failing to locate a valid bootloader (BL).
 *
 * Upon entering RCM, the primary USB interface is configured and a message
 * is transmitted to the host which provides the chip's Unique ID (UID).
 * As with all responses from the chip, this message is sent in the clear.
 *
 * After receiving the UID from the target, the host can send RCM messages
 * to the target.  The message format is described below, but a few general
 * points should be made here:
 * 1. All messages are signed.  For NvProduction and OdmNonSecure modes, this
 *    is with a key of 0's, which permits communication to be established
 *    with new parts and with those for which security is not an issue.  In the
 *    OdmSecure mode, the messages must be signed and encrypted with the SBK.
 * 2. RCM will send a response to each message.  Query messages will respond
 *    with the appropriate data.  All other messages produce an error code -
 *    0 for success, non-zero indicating a particular failure.
 * 3. Upon encountering any error, RCM locks up the chip to complicate efforts
 *    at tampering with a device in the field.
 *
 * Because knowledge of the SBK grants the keys to the kingdom, it is
 * essential that great care be taken with the selection, storage, and
 * handling of the SBKs.  For the greatest protection, unique SBKs should
 * be chosen for each chip, as this reduces the value of a compromised key
 * to a single device.
 *
 * The initial transmission of the UID provides a means to programmatically
 * identify the chip.  In a trusted environment, the UID can be used to
 * find the corresponding SBK in a database.  In an untrusted environment, the
 * UID can be sent to a trusted server that responds with a set of
 * encrypted & signed messages along with the expected response codes, thereby
 * eliminating any need for the SBK to leave the secure enviornment.
 *
 * Over RCM, the following messages can be sent:
 *     Sync: Checks that the chip is in RCM.  Simply returns success.
 *     QueryBootRomVersion: Returns the 32-bit BR code version number
 *     QueryRcmVersion: Returns the 32-bit RCM protocol version number
 *     QueryBootDataVersion: Returns the 32-bit BR data structure version
 *         number
 *     DownloadExecuteNvBinary: Loads the supplied applet into SysRAM, cleans up, and
 *         transfers execution to the applet.  Note that the USB connection
 *         is not dropped during this transition, so the applet can continue
 *         to communicate with the host without reinitializing the USB
 *         hardware.
 *     SoftFuses: Loads 64 32-bit fuses values into SysRAM at the same offset where BCT
 *         would locate them.
 *
 * All messages consist of an NvBootRcmMsg structure.  The applet code
 * for a DownloadExecuteNvBinary command follows the NvBootRcmMsg structure.
 * Similarly, the SoftFuses data command comes immediately after the 
 * NvBootRcmMsg structure.  
 * Any message less than the minimum message size must be padded to a multiple
 * of 16 bytes and to be at least the minimum size.  Padding bytes shall be
 * filled with the pattern 0x80 followed by all 0x00 bytes.
 *
 * Signing starts with the RandomAesBlock field of the
 * NvBootRcmMsg structure and extend to the end of the message.
 *
 * For a message to be considered valid, the following must be true after
 * computing its hash and decrypting the contents (if appropriate):
 * 1. The hash must match.
 * 2. The LengthSecure must match the LengthInsecure.
 * 3. PayloadLength <=  LengthSecure - sizeof(NvBootRcmMsg)
 * 4. RcmVersion must be set to NVBOOT_VERSION(1,0).  This is the RCM version
 *    number for AP15 and AP16.  By keeping this a constant when moving to
 *    AP20, a host program can communicate with any of NVIDIA's AP's without
 *    knowing the chip family a priori.
 * 5. The header padding must contain the correct pattern.
 * 6. Data between the end of the payload and the end of the message data
 *    must contain the correct padding pattern.
 * 7. Commands must be correct:
 *    a. Sync and Query commands: PayloadLength must be 0.
 *    b. DownloadExecuteNvBinary: The entry point must be within the
 *       address range occupied by the applet in memory.
 *    c. SoftFuses the payload must equal the SecureLength - header size.
 *    d. Any other supplied opcode will cause an error.
 */

#ifndef INCLUDED_NVBOOT_RCM_H
#define INCLUDED_NVBOOT_RCM_H

#include "nvcommon.h"

#include "nvboot_crypto_param.h"
#include "nvboot_config.h"
#include "nvboot_fuse.h"
#include "nvboot_se_aes.h"
#include "nvboot_se_rsa.h"

#if defined(__cplusplus)
extern "C"
{
#endif

/**
 * Defines the minimum message length, in bytes.
 * Although typically set larger to preserve key
 * and hash integrity, it must be long enough to
 * hold:
 * - LengthInsecure
 * - A hash
 * - 1 AES block
 * - An opcode
 * - Sufficient padding to an AES block
 */
#define NVBOOT_RCM_MIN_MSG_LENGTH (sizeof(NvBootRcmMsg)) /* In bytes */

/**
 * Defines the length of padding.
 */
#define NVBOOT_RCM_MSG_PADDING_LENGTH 12

/**
 * NvBootRcmResponse - These values are used either as return codes
 * directly or as indices into a table of return values.
 */
typedef enum
{
    /// Specifies successful execution of a command.
    NvBootRcmResponse_Success = 0,

    /// Specifies incorrect data padding.
    NvBootRcmResponse_BadDataPadding,

    /// Specifies incorrect message padding.
    NvBootRcmResponse_BadMsgPadding,

    /// Specifies incorrect RCM protocol version number was provided.
    NvBootRcmResponse_RcmVersionMismatch,

    /// Specifies that the hash check failed.
    NvBootRcmResponse_HashCheckFailed,

    /// Specifies an attempt to perform fuse operations in an illegal
    /// operating mode.
    NvBootRcmResponse_IllegalModeForFuseOps,

    /// Specifies an invalid entry point was provided.
    NvBootRcmResponse_InvalidEntryPoint,

    /// Specifies invalid fuse data was provided.
    NvBootRcmResponse_InvalidFuseData,

    /// Specifies that the insecure length was invalid.
    NvBootRcmResponse_InvalidInsecureLength,

    /// Specifies that the message contained an invalid opcode
    NvBootRcmResponse_InvalidOpcode,

    /// Specifies that the secure & insecure lengths did not match.
    NvBootRcmResponse_LengthMismatch,

    /// Specifies that the payload was too large.
    NvBootRcmResponse_PayloadTooLarge,

    /// Specifies that there was an error with USB.
    NvBootRcmResponse_UsbError,

    /// Specifies that USB setup failed.
    NvBootRcmResponse_UsbSetupFailed,

    /// Specifies that the fuse verification operation failed
    NvBootRcmResponse_VerifyFailed,

    /// Specifies a transfer overflow.
    NvBootRcmResponse_XferOverflow,

    ///
    /// New in Version 2.0
    ///

    /// Specifies an unsupported opcode was received.  Non-fatal error.
    NvBootRcmResponse_UnsupportedOpcode,

    /// Specifies that the command's payload was too small
    NvBootRcmResponse_PayloadTooSmall,

    /// Specifies that an ECID mismatch occurred while trying
    /// to enable JTAG with the NvBootRcmOpcode_SetDebugFeatures message.
    NvBootRcmResponse_ECIDMismatch,

    /// Specifies that public key has not been validated
    NvBootRcmResponse_PublicKeyNotValidated,

    /// Specifies a mismatch between SecProvisioningKeyNum_Secure
    /// SecProvisioningKeyNum_Insecure occurred.
    NvBootRcmResponse_SecProvisioningRcmKeyMismatch,

    /// Specifies that an invalid secure provisioning key number was specified
    // in the RCM header.
    NvBootRcmResponse_SecProvisioningRcmInvalidKeyInput,

    /// Specifies that signature check failed.
    NvBootRcmResponse_HashOrSignatureCheckFailed,

    /// Specifies that Decryption process failed.
    NvBootRcmResponse_DecryptionError,

    /// Specifes that required command not received
    /// or encountered error during reception.
    NvBootRcmResponse_DownloadPreboot,

    /// Specifes that BCH sanity failed
    NvBootRcmResponse_BchSanityFail,
    
    /// Prod debug flow not allowed
    NvBootRcmResponse_ProdDebugNotAllowed,
    
    /// The following two values must appear last
    NvBootRcmResponse_Num,
    NvBootRcmResponse_Force32 = 0x7ffffff
} NvBootRcmResponse;

/**
 * Defines the RCM command opcodes.
 */
typedef enum
{
    /// Specifies that no opcode was provided.
    NvBootRcmOpcode_None = 0,

    /// Specifes a sync command.
    NvBootRcmOpcode_Sync,

    /// Specifes a command that programs fuses.
    /// Deprecated. Use ProgramFuseArray
    NvBootRcmOpcode_ProgramFuses,

    /// Specifes a command that verifies fuse data.
    /// Deprecated. Use VerifyFuseArray.
    NvBootRcmOpcode_VerifyFuses,

    /// Specifes a command that downloads & executes an applet.
    /// Deprecated. Use DownloadExecuteNvBinary.
    NvBootRcmOpcode_DownloadExecute,

    /// Specifes a command that downloads & executes an Nvidia encrypted applet.
    NvBootRcmOpcode_DownloadExecuteNvBinary,

    /// Specifes a command that queries the BR code version number.
    NvBootRcmOpcode_QueryBootRomVersion,

    /// Specifes a command that queries the RCM protocol version number.
    NvBootRcmOpcode_QueryRcmVersion,

    /// Specifes a command that queries the BR data structure version number.
    NvBootRcmOpcode_QueryBootDataVersion,

    ///
    /// New w/v2.0
    ///

    /// Specifes a command that programs fuses.
    NvBootRcmOpcode_ProgramFuseArray,

    /// Specifes a command that verifies fuse data.
    NvBootRcmOpcode_VerifyFuseArray,

    /// !! DEPRECATED OPCODE !!
    /// Specifies a command that enables JTAG at BR exit.
    /// Keep opcode in place to give subsequent new opcodes a new unique number.
    NvBootRcmOpcode_EnableJtag,

    /// Specifies a command that sets the debug features of the chip at
    /// RCM exit. Used in conjunction with SecureDebugControl in NvBootRcmMsg.
    NvBootRcmOpcode_SetDebugFeatures,

    ///
    /// New w/v3.0
    ///

    /// Load soft fuses.
    NvBootRcmOpcode_LoadSoftFuses,

    /// Specifes a command that download a binary.
    NvBootRcmOpcode_Download,
    NvBootRcmOpcode_Force32 = 0x7fffffff,
} NvBootRcmOpcode;

/**
 * Defines the RCM fuse data
 * Deprecated.  Use fuse arrays instead.
 */
typedef struct NvBootRcmFuseDataRec
{
    /// Specifies the Secure Boot Key (SBK).
    uint8_t              SecureBootKey[NVBOOT_AES_KEY_MAX_BYTES];

    /// Specifies the Device Key (DK).
    uint8_t              DeviceKey[NVBOOT_DEVICE_KEY_BYTES];

    /// Specifies the JTAG Disable fuse falue.
    NvBool               JtagDisableFuse;

    /// Specifies the device selection value.
    NvBootFuseBootDevice BootDeviceSelection;

    /// Specifies the device configuration value (right aligned).
    NvU32                BootDeviceConfig;

    /// Specifies the SwReserved value.
    NvU32                SwReserved;

    /// Specifies the ODM Production fuse value.
    NvBool               OdmProductionFuse;

    /// Specifies the SpareBits value.
    NvU32                SpareBits;

    /// Specifies the fuse programming time in cycles.
    /// This value is ignored by the VerifyFuses command.
    NvU32                TProgramCycles;
} NvBootRcmFuseData;

/**
 * Defines the fuse array description in the RCM message header.
 * The array of fuse and mask values follows the message header.
 */
typedef struct NvBootRcmFuseArrayInfoRec
{
    /// Specifies the number of words of fuse data.
    NvU32                NumFuseWords;

    /// Specifies the fuse programming time in cycles.
    /// This value is ignored by the VerifyFuseArray command.
    NvU32                TProgramCycles;
} NvBootRcmFuseArrayInfo;

/**
 * Defines the data needed by the DownloadExecuteNvBinary command.
 */
typedef struct NvBootRcmDownloadDataRec
{
    /// Specifies the entry point address in the downloaded applet.
    /// Deprecated - EntryPoint is NOT sanitized
    ///              Use BCH EntryPoint instead, as RCM header is not authenticated by NV.
    NvU32 EntryPoint;

} NvBootRcmDownloadData;

/**
 * Defines the header for RCM messages from the host.
 * Messages from the host have the format:
 *     NvBootRcmMsg
 *     Payload
 *     Padding
 */
typedef struct NvBootRcmMsgRec
{
    /// Specifies the insecure length (in bytes).
    NvU32           LengthInsecure;

    /// This header will house public, non-secret cryptographic parameters necessary
    /// for the authentication of RCM messages. These parameters are
    /// collectively known as Public Cryptographic Parameters (PCP) and they will
    /// be stored in the unsigned section of the RCM header.
    /// The BR will check the validity of these parameters by calculating the SHA256
    /// hash of the Pcp and compare against the value burned in fuses.
    NvBootPublicCryptoParameters Pcp;

    /// All cryptographic signatures supported will be stored here. The RCM header can be
    /// simultaneously signed by all cryptographic signature types.
    NvBootCryptoSignatures Signatures;

    /// Padding to maintain AES block size alignment.
    uint8_t         UnsignedPadding[8];

    /// Specifies a block of random data. /* Not validated; helps security */
    DECLARE_ALIGNED(NvBootHash RandomAesBlock, 16);

    /// Specifies the Unique ID / ECID of the chip that this RCM message is
    /// specifically generated for. This field is required if the Opcode is
    /// NvBootRcmOpcode_SetDebugFeatures. It is optional otherwise. This is to prevent
    /// a signed RCM message package from being leaked into the field that would
    /// enable JTAG debug for all devices signed with the same private RSA key.
    NvBootECID      UniqueChipId;

    /// Specifies the command opcode.
    NvBootRcmOpcode Opcode;

    /// Specifies the secure length (in bytes).
    NvU32           LengthSecure;

    /// Specifies the payload length (in bytes).
    NvU32           PayloadLength;

    /// Stores the payload hash.
    NvU8            PayloadDigest[32];

    /// Specifies the RCM protocol version number.
    /// Always set to NVBOOT_VERSION(1,0) for compatibility with other AP's.
    NvU32           RcmVersion;

    /// Specifies the command arguments.
    union
    {
        /// Specifies the arguments for fuse commands (program & verify).
        /// Deprecated - use fuse array commands instead.
        NvBootRcmFuseData     FuseData;

        /// Specifies the arguments for fuse array commands (program & verify).
        NvBootRcmFuseArrayInfo FuseArrayInfo;

        /// Specifies the arguments for applet download & execution.
        NvBootRcmDownloadData DownloadData;
    } Args;

    /// Specifies which debug features to be enabled or disabled.
    /// Maps directly to APBDEV_PMC_DEBUG_AUTHENTICATION_0. These bits
    /// are not tied to a specific chip ECID, and UniqueChipId in the BCT
    /// does not need to match the actual chip ECID for them to take effect.
    /// 0x1 = ON, 0x0 = OFF
    /// PVA1_Secure_Debug - bit 13
    /// PVA0_Secure_Debug - bit 12
    /// RCE_Secure_Debug - bit 11
    /// SCE_Secure_Debug - bit 10
    /// SPE_Secure_Debug - bit 9
    /// BPMP_Secure_Debug - bit 8
    /// Reserved bits [7:6]
    /// 0x1 = ENABLED. 0x0 = DISABLED.
    /// NIDEN - bit 4
    NvU32           SecureDebugControl_Not_ECID_Checked;

    /// Specifies which debug features to be enabled or disabled.
    /// Maps directly to APBDEV_PMC_DEBUG_AUTHENTICATION_0.
    /// The ECID of the chip must match the ECID specified in UniqueChipId
    /// for the bits in this field to take effect.
    /// ECID check is mandatory for bits 0, 1, 2, 3, 5, 31.
    /// 0x1 = ENABLED. 0x0 = DISABLED.
    /// Ramdump Enable - bit 31.
    /// DBGEN - bit 5
    /// SPIDEN - bit 3
    /// SPNIDEN - bit 2
    /// DEVICEEN - bit 1
    /// JTAG_ENABLE - bit 0
    NvU32           SecureDebugControl_ECID_Checked;

    /// Specifies the factory secure provisioning key number to use.
    /// There are 64 such 256-bit AES keys.
    /// Specifying a key number of 1 to 15 is invalid. These are anti-cloning keys
    /// numbers and BR will ignore these values.
    /// BR will ignore this field if the secure_provision_index fuse is burned.
    /// Key number 64 (index [63]) is reserved for NVIDIA debug use.
    /// So, this field will only be used if the chip is in NvProductionMode,
    /// and when secure_provision_index is zero, and when SecProvisioningKeyNum
    /// is not 0 to 15.
    /// This key number must match SecProvisioningKeyNum_Insecure.
    NvU32           SecProvisioningKeyNum_Secure;
    
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

    /// For FSKP, derivation strings are used to generate new authentication
    /// key and encryption key.  The original FSKP key is used to encrypt these
    /// derivation strings to generate the new keys.
    uint8_t         SecProvisionDerivationString1[NVBOOT_BCT_DERIVATION_STRING_SIZE];
    uint8_t         SecProvisionDerivationString2[NVBOOT_BCT_DERIVATION_STRING_SIZE];

    /// Specifies space for padding that pushes the size of the encrypted
    /// and hashed portion of the header to the next multiple of AES block
    /// size.
    uint8_t         Padding[NVBOOT_RCM_MSG_PADDING_LENGTH];
} NvBootRcmMsg;

#if defined(__cplusplus)
}
#endif

#endif /* #ifndef INCLUDED_NVBOOT_RCM_H */
