CAN Application (app/can-app.c)
================================
@tableofcontents

This section explains how to use the CAN application on
NVIDIA<sup>&reg;</sup> Jetson<sup>&trade;</sup> NVIDIA Jetson TX2, Jetson AGX
Xavier, and Jetson Xavier NX.

There are two CAN controllers, which run in Always On (AON) mode. The
application demonstrates how to use CAN controller driver APIs.

# Compiling the Application #

To control compilation of the CAN application, use the
`ENABLE_CAN_APP`, `ENABLE_CAN_TX` and `ENABLE_CAN_RX` flags in the
`soc/*/target_specific.mk` file. Follow the platform-specific sections below to
use the `ENABLE_CAN_TX and ENABLE_CAN_RX` flags.

@note The end user must compile the device tree and flash board to update the
device tree and SCR settings after making kernel device tree and SCR changes as
described in following sections.

# Prerequisites #

Before you can successfully execute the application, you must obtain two CAN
transceivers. The transceivers must be 3.3V I/O compatible, similar to the ones
at:

https://www.amazon.com/SN65HVD230-CAN-Board-Communication-Development/dp/B00KM6XMXO

# Operation #

By default, the CAN AON controller driver sets the bit rate to 500 Kb/second
and the data bit rate to 2 Mb/second. For more information, see the
`mttcan_controller_init` function in the `tegra-can.c` driver source file.

The application prints the details below upon successful CAN message
transmission, and prints the transmitted element, which includes information
about the transmitted message. The transmit element is also created when there
is a bus state change.

    Transmited message from CAN <controller number>
    Transmission complete event
    Transmit event element information:
    Message ID: 0xa5, Event Type: Tx event, CAN Frame: <frame details>

The application prints the details below upon successful CAN message reception.

    Message received at CAN <controller number>
    Message ID: 0xa5, Message data length: 2
    Message Data:
    0xaa 0x55

# Wiring Details #

@code

------------------    ----------------------        -------------------     ----------------------
|                |    |                    |        |                 |     |                    |
|             TX |--->| TX           CAN H |<------>|CAN H         TX |<----| TX                 |
| CAN0           |    |   CAN Transceiver  |        |  CAN Transceiver|     |     CAN1 Controller|
| Controller  RX |<---| RX           CAN L |<------>|CAN L         RX |---->| RX                 |
|                |    |                    |        |                 |     |                    |
------------------    ----------------------        -------------------     ----------------------

@endcode

@note The NVIDIA recommends that you use termination between CAN devices
according to CAN specifications.


# Jetson TX2 Configuration #

The application uses both AON CAN controllers, so you must set both the flags
`ENABLE_CAN_TX and ENABLE_CAN_RX` to 1. Additionally, define `ENABLE_CAN0_AS_TX`
and `ENABLE_CAN1_AS_RX` macros in `can-app.c`. This connects the CAN0 bus to
CAN1 using the CAN transceivers and the diagram above, where:
- CAN0 transmits pre-defined messages and
- CAN1 receives such messages.

Ensure that the CAN is disabled in the kernel device tree
`tegra186-quill-p3310-1000-c03-00-base.dts` as shown below:
```
mttcan@c310000 {
        status = "disabled";
        ...
};
mttcan@c320000 {
        status = "disabled";
        ...
};
```

CAN 0 and 1 controller TX and RX are brought out to 30 pin header J26 as below:
- Pin 2 - 3.3V
- Pin 10 - GND
- Pin 5 - CAN0_RX ---> Connects RX pin of CAN transceiver
- Pin 7 - CAN0_TX ---> Connects TX pin of CAN transceiver
- Pin 15 - CAN1_RX ---> Connects RX pin of CAN transceiver
- Pin 17 - CAN1_TX ---> Connects TX pin of CAN transceiver

Additional 3.3V and GND connections are available at 40 pin header J21 as below:
- Pin 1, 17 - 3.3V
- Pin 25, 39 - GND

# Jetson AGX Configuration #

The application uses both AON CAN controllers, so you must set both the flags
`ENABLE_CAN_TX and ENABLE_CAN_RX` to 1. Additionally, define `ENABLE_CAN0_AS_TX`
and `ENABLE_CAN1_AS_RX` macros in `can-app.c`. This connects the CAN0 bus to
CAN1 using the CAN transceivers and the diagram above, where:
- CAN0 transmits pre-defined messages and
- CAN1 receives such messages.

Ensure that the CAN is disabled in the kernel device tree `tegra194-p2888-0001-p2822-0000-common.dtsi`
as shown below:
```
mttcan@c310000 {
        status = "disabled";
        ...
};
mttcan@c320000 {
        status = "disabled";
        ...
};
```

Modify the SCR setting in `tegra194-mb1-bct-scr-cbb-mini.cfg` as follows:

- scr.378.4 = 0x1a0034ff; # CLK_RST_CONTROLLER_AON_SCR_CAN_0
- scr.407.4 = 0x1a0034ff; # CLK_RST_CONTROLLER_CAN1_SCR_FMON_0
- scr.408.4 = 0x1a0034ff; # CLK_RST_CONTROLLER_CAN2_SCR_FMON_0
- scr.2602.6 = 0x19003232; # AON_NOC_CAN1_BLF_CONTROL_REGISTER_0
- scr.2603.6 = 0x19003232; # AON_NOC_CAN2_BLF_CONTROL_REGISTER_0

Modify the pinmux setting in `tegra19x-mb1-pinmux-p2888-0000-a04-p2822-0000-b01.cfg`
as follows:


- pinmux.0x0c303000 = 0x00000400; # can1_dout_paa0: can1, tristate-disable, input-disable
- pinmux.0x0c303008 = 0x00000458; # can1_din_paa1: can1, pull-up, tristate-enable, input-enable
- pinmux.0x0c303010 = 0x00000400; # can0_dout_paa2: can0, tristate-disable, input-disable
- pinmux.0x0c303018 = 0x00000458; # can0_din_paa3: can0, pull-up, tristate-enable, input-enable

CAN 0 and 1 controller TX and RX are brought out to 40 pin header J30 as below:
- Pin 1, 17 - 3.3V
- Pin 9, 25, 39 - GND
- Pin 29 - CAN0_RX ---> Connects RX pin of CAN transceiver
- Pin 31 - CAN0_TX ---> Connects TX pin of CAN transceiver
- Pin 37 - CAN1_RX ---> Connects RX pin of CAN transceiver
- Pin 33 - CAN1_TX ---> Connects TX pin of CAN transceiver

# Jetson NX Configuration #

The application uses only AON CAN 0 controller, that is the only controller
exposed to the user. Conceptually it follows the same diagram as mentioned above
, where one end will be configured as CAN TX and other as CAN RX. The following
test setup steps assume that two Jetson NX platforms are used to run the
application.

On Jetson NX acting as CAN transmitter:

- Set `ENABLE_CAN_TX` to 1 and `ENABLE_CAN_RX` to 0.
- Additionally, define only `ENABLE_CAN0_AS_TX` macro in `can-app.c` file.

On Jetson NX acting as CAN receiver:

- Set `ENABLE_CAN_TX` to 0 and `ENABLE_CAN_RX` to 1.
- Additionally, define only the `ENABLE_CAN0_AS_RX` macro in `can-app.c` file.

This connects the CAN0 bus of one Jetson NX to CAN0 of another Jetson NX,
using the above mentioned CAN transceivers and diagram, where:
- CAN0 transmits pre-defined messages and
- CAN0 receives such messages from other Jetson NX.

Ensure that the CAN is disabled in the kernel device tree `tegra194-p3668-common.dtsi`
as shown below:
```
mttcan@c310000 {
        status = "disabled";
        ...
};
```

Modify the SCR setting in `tegra194-mb1-bct-scr-cbb-mini-p3668.cfg` as follows:

- scr.378.4 = 0x1a0034ff; # CLK_RST_CONTROLLER_AON_SCR_CAN_0
- scr.407.4 = 0x1a0034ff; # CLK_RST_CONTROLLER_CAN1_SCR_FMON_0
- scr.2602.6 = 0x19003232; # AON_NOC_CAN1_BLF_CONTROL_REGISTER_0

Modify the pinmux setting in `tegra19x-mb1-pinmux-p3668-a01.cfg` as follows:

- pinmux.0x0c303010 = 0x00000400; # can0_dout_paa2: can0, tristate-disable, input-disable
- pinmux.0x0c303018 = 0x00000458; # can0_din_paa3: can0, pull-up, tristate-enable, input-enable

CAN 0 is brought out to 4 pin header J17 as below:
- Pin 1 - CAN0_TX ---> Connects TX pin of CAN transceiver
- Pin 2 - CAN0_RX ---> Connects RX pin of CAN transceiver
- Pin 3 - GND ---> Connects GND pin of CAN transceiver
- Pin 4 - 3.3V ---> Connects 3.3V pin of CAN transceiver
