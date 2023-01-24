@mainpage Welcome

Welcome to the <i>Jetson Sensor Processing Engine (SPE) Developer Guide</i>.

NVIDIA<sup>&reg;</sup> Jetson<sup>&trade;</sup>
provides a built-in Cortex-R5 micro-controller within an always-on
power domain also known as Sensor Processing Engine (SPE). Example use cases SPE
may enable includes sensor data processing, wake up management, UAV, robotics.

SPE micro-controller has the following configurations:
- ARM V7-R ISA
- 32-kilobyte Instruction cache
- 32-kilobyte Data cache
- 256-kilobyte SRAM attached to TCM interface
- Vectored Interrupt
- 64-bit AXI master interface for DRAM request
- 32-bit AXI master interface for MMIO request
- 200MhZ maximum and operating core clock speed

The number of SPE peripherals and their instances will vary based on the Jetson
platform. Some peripherals are shared with the rest of the system.

The following lists supported peripherals by SPE.
- General purpose timers, Watchdog timer, Timestamp counter
- I2C, SPI, UART, GPIO, PWM
- CAN Controller
- DMIC (for voice wake functionality)

For detailed information on the SPE,
see <i>NVIDIA Technical Reference Manual (TRM)</i> for the Jetson platform.


