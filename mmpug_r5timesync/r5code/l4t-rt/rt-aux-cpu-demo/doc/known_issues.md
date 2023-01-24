Known Issues {#rt-knownissues}
============
@tableofcontents

This section describes outstanding issues with the samples.

# Flashing Failure

When the SPE binary size is greater than 128-kilobytes, flashing fails.
The binary size includes the binary and NVIDIA-specific headers. To workaround
this issue follow below steps.

# For Jetson TX2

```
Open file flash_l4t_t186.xml, this file can be found under Linux_for_Tegra directory
Look for SPENAME and SPENAME_b partitions and change their size subsections from 131072 to 262144
```

# SC7 Incompatibility

The SPE binary generated with this SDK is not compatible with
NVIDIA<sup>&reg;</sup> SC7.

# CAN bus error causes flash process to hang

When the CAN transceivers are not connected the CAN transmit part of the driver emits continuous
debug prints. This causes a debug UART port ownership issue and a race condition between SPE and
BLs, breaking the flashing process.

One way to solve such a problem is to rate-limit or disable the debug messages from `tegra-can.c`.
Specifically, rate-limit or disable the messages from `tegra_can_irq_handler()` and
`tegra_add_msg_ctlr_list()`.

`tegra_can_irq_handler()` has many *if* conditions which test various interrupt status bits,
but when the CAN transceivers are not connected the function mainly prints debug messages from the
`if((ir & MIT_IR_PED_MASK) || (ir & MIT_IR_PEA_MASK))` block. Rate-limiting such debug messages with
a simple counter, or commenting them out, generally solves the issue.