# Script for showing time info on Jetson Xavier

#!/bin/bash

#get all the times
#list=$(busybox devmem 0x03010008 W)
nsecsrtc1=$(date +%s%N)
list=$(busybox devmem 0x03010000 W)
nsecsrtc2=$(date +%s%N)
#average to account for scheduling and other delays
nsecsrtc=$(echo "($nsecsrtc1+$nsecsrtc2)/2" | bc)
convs=$(printf "%d\n" $list)
decn=$(echo "$convs/31.249" |bc)
secs=$(echo "$decn/1000000.0" | bc)
secsrtc=$(echo "$nsecsrtc/1000000000" |bc)
delta=$(echo "(-($decn-$nsecsrtc/1000))"|bc)
deltams=$(echo "($delta/1000)"|bc)
deltas=$(echo "($deltams/1000)"|bc)
echo "Difference between TKE_AON_SHARED_TKETSC0_0 /31.249 (is supposed to be 31.25) and linux clock. TSC wraps approximately  every 2 minutes"
echo "TSC:Microsecs:  $decn\t Secs: $secs \nRTC (s): $secsrtc\t ns: $nsecsrtc\nDifference between the clocks:\nDelta (Microsec): $delta\tMillisecs $deltams\tsecs:  $deltas"
