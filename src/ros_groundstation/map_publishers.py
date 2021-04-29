import rospy
from rosplane_msgs.msg import Waypoint # TODO: Roscopter Waypoint -Nathan
from .map_subscribers import InitSub
from service_client import *


class PPPub():
    enabled = False
    PPPub_topic = None
    pub = None
    default_altitude = 150  # arbitrarily chosen
    default_airspeed = 15  # arbitrarily chosen

    @staticmethod
    def updatePPPubTopic(new_topic_name):
        print("Updating point publisher to", new_topic_name)
        PPPub.reset()
        PPPub.PPPub_topic = new_topic_name
        if PPPub.PPPub_topic is not None:
            PPPub.pub = rospy.Publisher(PPPub.PPPub_topic, Waypoint, queue_size=10)

    @staticmethod
    def reset():
        if PPPub.pub is not None:
            PPPub.pub.unregister()

    @staticmethod
    def publishWaypoint(position, heading_rad=None, set_current=False): # FIXME: No more airspeed, no more chi_d. -Nathan
        # if PPPub.pub is None:
        #     return
        req = [0]*5
        req[0] = position[0]
        req[1] = position[1]
        req[2] = position[2]

        if heading_rad is None: # FIXME
            req[3] = 0
            # waypoint_msg.chi_valid = False
        else:
            req[3] = heading_rad
            # waypoint_msg.chi_valid = True
        # waypoint_msg.set_current = set_current
        req[4] = -1
        # PPPub.pub.publish(waypoint_msg)
        add_waypoint_client(req)

    @staticmethod
    def publishWaypointShort(n, e, heading_rad=None, set_current=False):
        PPPub.publishWaypoint((n, e, -PPPub.default_altitude), heading_rad, set_current)

    @staticmethod
    def clearWaypoints():
        # if PPPub.pub is None:
        #     return
        # waypoint_msg = Waypoint()
        # waypoint_msg.clear_wp_list = True
        # PPPub.pub.publish(waypoint_msg)
        clear_waypoints_client()

    @staticmethod
    def publishGeofencePoint(n, e):
        req = [0]*2
        req[0] = n
        req[1] = e

        add_geofence_point_client(req)

    @staticmethod
    def clearGeofence():
        clear_geofence_client()

    @staticmethod
    def setDefaultAirspeed(airspeed): # FIXME
        PPPub.default_airspeed = airspeed

    @staticmethod
    def setDefaultAltitude(altitude):
        PPPub.default_altitude = altitude

    @staticmethod
    def getNED(lat, lon, altitude=None):
        if altitude is None:
            altitude = PPPub.default_altitude
        return InitSub.GB.gps_to_ned(lat, lon, altitude)
