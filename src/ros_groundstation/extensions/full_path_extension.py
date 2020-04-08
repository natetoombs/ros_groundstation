from PyQt5.QtGui import QPen, QBrush
from PyQt5.QtCore import Qt
import rospy
from rosplane_msgs.msg import Full_Path
from ros_groundstation.Path import Path
from ros_groundstation.map_subscribers import InitSub

class FullPathExtension:
  def __init__(self, gs_widget):
    self.active = False
    self.Sub = None
    self.current_path = None
    gs_widget.add_map_painter(self.on_map_paint)
    gs_widget.add_text_check_option("Extension", "Full Path Topic", self.topic_update, True, "/full_path")

  def on_map_paint(self, painter, marble_map):
    painter.setPen(QPen(QBrush(Qt.cyan), 1.5, Qt.SolidLine, Qt.RoundCap))
    if self.current_path is not None:
        for path in self.current_path:
            marble_map.draw_single_extended_path(painter, path)

  def topic_update(self, checked, topic):
    if checked:
      if self.active:
        self.sub.unregister()
      self.sub = rospy.Subscriber(topic, Full_Path, self.full_path_callback)
      self.active = True
    else:
      if self.active:
        self.active = False
        self.sub.unregister()
        self.sub = None

  def full_path_callback(self, full_path):
    if InitSub.enabled:
      self.current_path = [Path(path) for path in full_path.paths]

def init_extension(gs_widget):
  ext = FullPathExtension(gs_widget)