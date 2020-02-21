from __future__ import print_function
import argparse
from python_qt_binding import QT_BINDING
from python_qt_binding.QtCore import qDebug, QTimer, Qt
from python_qt_binding.QtWidgets import QWidget, QBoxLayout, QVBoxLayout, QHBoxLayout, QPushButton, QSplitter

# Custom Widgets
from .map_widget import MapWindow
from .plot_widget import PlotWidget
from .data_plot import DataPlot
from .artificial_horizon import ArtificialHorizon

class GroundStationWidget(QWidget):

    def __init__(self):
        super(GroundStationWidget, self).__init__()

        self._principle_layout = QBoxLayout(0) # main layout is horizontal (0)
        self._principle_layout = QSplitter(Qt.Horizontal)
        self._map_layout = QVBoxLayout()
        map_widget = QWidget()
        map_widget.setLayout(self._map_layout)
        self._principle_layout.addWidget(map_widget)
        self._control_layout = QVBoxLayout()
        control_widget = QWidget()
        control_widget.setLayout(self._control_layout)
        self._principle_layout.addWidget(control_widget);

        self.setAcceptDrops(False) # Dragging and Dropping not permitted
        self.setWindowTitle('ROSplane Ground Station')

        #=============================
        self._mw = MapWindow()
        self._map_layout.addWidget(self._mw)
        self._tv = PlotWidget()
        self._data_plot = DataPlot(self._tv)
        self._data_plot.set_autoscale(x=False)
        self._data_plot.set_autoscale(y=DataPlot.SCALE_EXTEND|DataPlot.SCALE_VISIBLE)
        self._data_plot.set_xlim([0, 10.0])
        self._tv.switch_data_plot_widget(self._data_plot)

        self._control_layout.addWidget(self._tv, 1) # ratio of these numbers determines window proportions
        self._ah = ArtificialHorizon()
        self._control_layout.addWidget(self._ah, 1)
        #=============================

        print('fake init')
        total_layout = QBoxLayout(QBoxLayout.TopToBottom)
        self._principle_layout.setHandleWidth(8)
        total_layout.addWidget(self._principle_layout)
        self.setLayout(total_layout)

        # Global timer for marble_map and artificial_horizon
        self.interval = 100     # in milliseconds, period of regular update
        self.timer = QTimer(self)
        self.timer.setInterval(self.interval)
        self.timer.timeout.connect(self._mw._marble_map.update)
        self.timer.timeout.connect(self._ah.update)
        self.timer.start()

    def closeEvent(self, event):
        self.timer.stop()

    def save_settings(self, plugin_settings, instance_settings): # have a file to read and write from
        print('fake save') # < prints to terminal

    def restore_settings(self, plugin_settings, instance_settings):
        print('fake restore')
