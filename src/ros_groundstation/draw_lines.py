
from .Geo import Geobase
from .marble_map import point_to_pix

class DrawLines
    def draw_single_line(self, painter, point1, point2): #points in lat lon
        if point2 is None or point1 is None:
            print('none')
            return
        else:
            pixel_point1 = point_to_pix(point1)
            pixel_point2 = point_to_pix(point2)
            painter.drawLine(pixel_point1[0], pixel_point1[1], pixel_point2[0], pixel_point2[1])