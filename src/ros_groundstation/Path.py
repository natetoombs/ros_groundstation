import rospy
from rosplane_msgs.msg import Current_Path
import map_subscribers
from math import isnan


class Path:
    def __init__(self, path):
        if map_subscribers.InitSub.enabled:
            self.Va_d = path.path.Va_d
            self.path_type = path.path.path_type
            self.is_line = (self.path_type == Current_Path.LINE_PATH)
            if self.is_line:
                self.r = Path.convert_ned_to_lla(path.path.r)
                self.line_end = Path.convert_ned_to_lla(path.line_end)
                self.q = path.path.q
            else:
                self.orbit_center = Path.convert_ned_to_lla(path.path.c)
                self.lambda_ = path.path.lambda_
                self.clockwise = (self.lambda_ == Current_Path.CLOCKWISE)
                self.radius = path.path.rho
                self.orbit_start = path.orbit_start
                self.orbit_end = path.orbit_end

    def __str__(self):
        ret = ("ROS Groundstation Path\n"
               "\tpath_type: {}\n"
               "\tis_line: {}\n"
               "\tVa_d: {}\n"
               ).format(str(self.path_type), str(self.is_line), str(self.Va_d))
        if self.is_line:
            ret += ("\tr: {}\n"
                    "\tline_end: {}\n"
                    "\rq: {}").format(str(self.r), str(self.line_end), str(self.q))
        else:
            ret += ("\tc: {}\n"
                    "\tlambda: {}\n"
                    "\tclockwise: {}\n"
                    "\trho: {}\n"
                    "\torbit_start: {}\n"
                    "\torbit_end: {}").format(str(self.orbit_center), str(self.lambda_), str(self.clockwise),
                                              str(self.radius), str(self.orbit_start), str(self.orbit_end))
        return ret

    @staticmethod
    def convert_ned_to_lla(point):
        assert (len(point) == 3)
        if map_subscribers.InitSub.enabled:
            lat, lon, alt = map_subscribers.InitSub.GB.ned_to_gps(*point)
            assert (not isnan(lat))
            assert (not isnan(lon))
            assert (not isnan(alt))
            return lat, lon, alt
        else:
            return 0, 0, 0
