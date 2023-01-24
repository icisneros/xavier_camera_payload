Generic Timestamping Engine (GTE) Application (app/gte-app.c)
=============================================================

This section explains how to use the generic time-stamping engine (GTE).

The generic timestamping engine snoops a set of signals, timestamps any change
on them and presents the timestamped events to SW via FIFOs. There are three GTE
instances/slices available to AON/SPE each 32bits wide; details about signals
mapping to slices could be found in gte-tegra-hw.h. The application demonstrates
the GTE driver APIs and works in tandem with gpio-app to monitor the interrupt
on the `GPIO_APP_IN` line.
For guidance on regarding wiring details, see [GPIO Application](md_rt-aux-cpu-demo_doc_gpio.html).

GTE ISR
is called when the FIFO reaches the GTE_FIFO_OCCUPANCY threshold.
On successful execution, GTE ISR following message:

    Event Id, Edge = "rising/falling", Time stamp = xxx

The "ENABLE_GTE_APP" flag in the `soc/*/target_specific.mk` file
controls how this app is compiled.

The GTE sample app enables monitoring of the GPIO interrupt event through GTE
slice 1 and that interrupt events are generated with `GPIO_APP_IN GPIO`. The
sample app also monitors activity on `GPIO_APP_IN GPIO`, through GTE slice 2 and
records the time stamp for the input value change.
