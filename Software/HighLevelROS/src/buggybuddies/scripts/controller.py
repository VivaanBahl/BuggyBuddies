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
    return [];


def getdist_m(waypoint, current_lat, current_lon):



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