# Software License Agreement (BSD License)
#
# Copyright (c) 2013, Eric Perko
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the names of the authors nor the names of their
#    affiliated organizations may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

"""Defines the main method for the nmea_serial_driver executable."""

import serial
import sys

import rospy

from libnmea_navsat_driver.driver import RosNMEADriver
import pdb  #ivan



#ivan
def deal_with_quirks(input_string):
    string_list = []

    if '\r' in input_string:
        # For an example sentence such as "$GNGLL,,,,,,V,N*7A\r$GNRMC,,V,,,,,,,,,,N*4D" we split at the '\r'
        # and pass back both portions. Though GNGLL sentences are not parsed in this script.
        nmea_string_multiple = input_string.split('\r')
        for i in range(0, len(nmea_string_multiple)):
            string_list.append(nmea_string_multiple[i])
    
    elif input_string.count('$'):
        # For example a sentence such as "$GNGLL,4026.24473,N,07955.36511,W,201328.00,A,A$GNRMC,201329.00,A,4026.24466,N,07955.36517,W,0.467,,291122,,,A*73"
        # where we did not have a '\r' character to split on.
        # Though it seems like it the first string often gets cut short (no checksum).
        nmea_string_multiple = input_string.split('$')
        # the first element of this list will always be empty string '', so ignore that and start iterating at i = 1
        for i in range(1, len(nmea_string_multiple)):
            new_str = '$' + nmea_string_multiple[i]  # append '$' because this gets stripped out by split()
            string_list.append(new_str)


    return string_list




def main():
    """Create and run the nmea_serial_driver ROS node.

    Creates a ROS NMEA Driver and feeds it NMEA sentence strings from a serial device.

    ROS parameters:
        ~port (str): Path of the serial device to open.
        ~baud (int): Baud rate to configure the serial device.
    """
    rospy.init_node('nmea_serial_driver')

    serial_port = rospy.get_param('~port', '/dev/ttyUSB0')
    serial_baud = rospy.get_param('~baud', 4800)
    frame_id = RosNMEADriver.get_frame_id()

    try:
        GPS = serial.Serial(port=serial_port, baudrate=serial_baud, timeout=2)

        try:
            driver = RosNMEADriver()
            while not rospy.is_shutdown():
                # pdb.set_trace()  #ivan
                data = GPS.readline().strip()
                data_list = deal_with_quirks(data)  #ivan  could have more than one string combined together
                for data in data_list:  #ivan
                    try:
                        driver.add_sentence(data, frame_id)
                    except ValueError as e:
                        rospy.logwarn(
                            "Value error, likely due to missing fields in the NMEA message. "
                            "Error was: %s. Please report this issue at "
                            "github.com/ros-drivers/nmea_navsat_driver, including a bag file with the NMEA "
                            "sentences that caused it." %
                            e)

        except (rospy.ROSInterruptException, serial.serialutil.SerialException):
            GPS.close()  # Close GPS serial port
    except serial.SerialException as ex:
        rospy.logfatal(
            "Could not open serial port: I/O error({0}): {1}".format(ex.errno, ex.strerror))
