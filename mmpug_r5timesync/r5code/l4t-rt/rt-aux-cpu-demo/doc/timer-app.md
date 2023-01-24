Timer application (app/timer-app.c)
===================================

Application demonstrates the timer driver APIs and uses timer2 in 5 seconds
periodic mode.

The `ENABLE_TIMER_APP` flag in the `soc/*/target_specific.mk file`
controls compilation of this demo app.

When the app successfully executes, it
displays the following message at every `TIMER2_PTV` seconds

    Timer2 irq triggered

Additionally, it calls the timer stop API after the count value reaches `STOP_TIMER`.
