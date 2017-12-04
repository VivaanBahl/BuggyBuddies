#!/usr/bin/env python

import rospy
from gps_common.msg import GPSFix
import serial
import utm
import math;

current_lat = 0.0;
current_lon = 0.0;

def subscriber_callback(data):
    global current_lat, current_lon;

    current_lat = data.latitude;
    current_lon = data.longitude;
    pass


def load_waypoints():
    return [
        (40.442775, -79.942775),
        (40.442759, -79.942785),
        (40.442728, -79.942806),
        (40.442693, -79.942824),
        (40.442663, -79.942840),
        (40.442640, -79.942849),
        (40.442612, -79.942862),
        (40.442588, -79.942872),
        (40.442564, -79.942879),
        (40.442534, -79.942889),
        (40.442505, -79.942897),
        (40.442479, -79.942901),
        (40.442455, -79.942906),
        (40.442426, -79.942908),
        (40.442393, -79.942911),
        (40.442397, -79.942914),
        (40.442369, -79.942915),
        (40.442345, -79.942915),
        (40.442320, -79.942916),
        (40.442283, -79.942919),
    ];

def start_subscriber_spin():

    rospy.init_node("NAND_Conroller", anonymous=True);
    rospy.Subscriber("GPS", GPSFix, subscriber_callback);

    waypoint_list = load_waypoints();
    cmd_port = serial.Serial("/dev/ttyUSB1", 115200);

    rate = rospy.Rate(10);

    waypoint_index = 0;
    while not rospy.is_shutdown():

        # choose waypoint
        for i in xrange(waypoint_index, len(waypoint_list)):
            waypoint = waypoint_list[waypoint_index];
            utm_wp = utm.from_latlon(waypoint[0], waypoint[1]);
            utm_cp = utm.from_latlon(current_lat, current_lon);

            distance = (utm_wp[0] - utm_cp[0]) ** 2 + (utm_wp[1] - utm_cp[1]) ** 2;
            if (distance < 20 and distance > 5):
                waypoint_index = i;
                break;

        # steer towards waypoint
        waypoint = waypoint_list[waypoint_index];
        utm_wp = utm.from_latlon(waypoint[0], waypoint[1]);
        utm_cp = utm.from_latlon(current_lat, current_lon);

        dy = utm_wp[1] - utm_cp[1];
        dx = utm_wp[0] - utm_cp[0];

        angle = math.atan2(dy, dx);
        if (angle > 0):
            angle = 1000;
        else:
            angle = -1000;

        cmd_port.write(str(angle) + "\n");

        rate.sleep()

    pass;



if __name__ == "__main__":
    start_subscriber_spin();