# Sync r5 #

The daemon sends the phase and offset to the r5 chip to allow it sync the timestamps.

# Installation #

- Execute "make"  to build the code
- Execute "sudo make install" to install the code as a service
- To alwasys launch the service on boot execute "systemctl enable syncr5clock"


# Check #

Check that sync is working by hooking the r5 trigger pin to a gpio pin and running "gpio-int" It should show a time close to correct.

# Check shm #

Run the syncr5clock service

If you run "sudo sampleshmreader" you should see an output similar to this:
Freq: 31248503 Offset: 1567906480675228 usec Valid 1
st: 1568259893 706874121
tc: 1568259893 712176799

This tells you that the system time and system time calculated from TSC are close.

