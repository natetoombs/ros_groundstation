#!/usr/bin/env python3

import sys
import rospy
from roscopter_msgs.srv import *
# from geofence_msgs.srv import *

def add_waypoint_client(req):
    # rospy.wait_for_service('AddWaypoint', 2.0)
    try:
        add_waypoint = rospy.ServiceProxy('add_waypoint', AddWaypoint)
        resp = add_waypoint(req[0],req[1],req[2],req[3],req[4])
        return resp
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def clear_waypoints_client():
    # rospy.wait_for_service('ClearWaypoints', 2.0)
    try:
        clear_waypoints = rospy.ServiceProxy('clear_waypoints', ClearWaypoints)
        resp = clear_waypoints()
        return resp
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def land_client():
    # rospy.wait_for_service('AddWaypoint', 2.0)
    try:
        land = rospy.ServiceProxy('land', Land)
        resp = land()
        return resp
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)
        
def return_to_base_client():
    # rospy.wait_for_service('AddWaypoint', 2.0)
    try:
        return_to_base = rospy.ServiceProxy('return_to_base', ReturnToBase)
        resp = return_to_base()
        return resp
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

# def add_geofence_point_client(req):
#     # rospy.wait_for_service('AddWaypoint', 2.0)
#     try:
#         add_geofence_point = rospy.ServiceProxy('add_geofence_point', AddGeoPoint)
#         resp = add_geofence_point(req[0],req[1])
#         return resp
#     except rospy.ServiceException as e:
#         print("Service call failed: %s"%e)

# def clear_geofence_client():
#     # rospy.wait_for_service('ClearWaypoints', 2.0)
#     try:
#         clear_geofence = rospy.ServiceProxy('clear_geofence', ClearGeofence)
#         resp = clear_geofence()
#         return resp
#     except rospy.ServiceException as e:
#         print("Service call failed: %s"%e)

