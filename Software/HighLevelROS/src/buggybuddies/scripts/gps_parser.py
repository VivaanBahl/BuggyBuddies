#!/usr/bin/env python

import rospy
import serial
from gps_common.msg import GPSFix

def start_subscriber_spin():

    gps_pub = rospy.Publisher("GPS",
    rospy.init_node("GPS_Plotter", anonymous=True)

    gps_port = serial.Serial("/dev/ttyUSB0", 57600);

    while not rospy.is_shutdown():
        line = gps_port.readline();
        tokens = line.split(",");

        if (tokens[0] != "$GPGGA" or int(tokens[6]) is 0):
            continue;

        lat = tokens[2];
        lat = int(lat[0:2]) + float(lat[2:]) / 60.0;

        n = tokens[3];
        if n is not "N": lat = -lat;

        lon = tokens[4];
        lon = int(lon[0:3]) + float(lon[3:]) / 60.0;

        w = tokens[5];
        if n is "W": lon = -lon;

        GPSFix msg;
        msg.latitude = lat;
        msg.longitude = lon;

    pass

if __name__ == "__main__":
    start_subscriber_spin()

