import rospy
from rosplane_msgs.msg import Waypoint
from .map_subscribers import InitSub


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
    def publishWaypoint(position, airspeed, heading_rad=None, set_current=False):
        if PPPub.pub is None:
            return
        waypoint_msg = Waypoint()
        waypoint_msg.w[0] = position[0]
        waypoint_msg.w[1] = position[1]
        waypoint_msg.w[2] = position[2]
        if heading_rad is None:
            waypoint_msg.chi_d = 0
            waypoint_msg.chi_valid = False
        else:
            waypoint_msg.chi_d = heading_rad
            waypoint_msg.chi_valid = True
        waypoint_msg.set_current = set_current
        waypoint_msg.Va_d = airspeed
        PPPub.pub.publish(waypoint_msg)

    @staticmethod
    def publishWaypointShort(n, e, heading_rad=None, set_current=False):
        PPPub.publishWaypoint((n, e, -PPPub.default_altitude), PPPub.default_airspeed, heading_rad, set_current)

    @staticmethod
    def clearWaypoints():
        if PPPub.pub is None:
            return
        waypoint_msg = Waypoint()
        waypoint_msg.clear_wp_list = True
        PPPub.pub.publish(waypoint_msg)

    @staticmethod
    def setDefaultAirspeed(airspeed):
        PPPub.default_airspeed = airspeed

    @staticmethod
    def setDefaultAltitude(altitude):
        PPPub.default_altitude = altitude

    @staticmethod
    def getNED(lat, lon, altitude=None):
        if altitude is None:
            altitude = PPPub.default_altitude
        return InitSub.GB.gps_to_ned(lat, lon, altitude)
