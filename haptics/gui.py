import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtGui
from PyQt5.QtCore import QTimer
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QHBoxLayout, QCheckBox, QPushButton

import sys

from . import defs
from .comms_manager import CommsManager
from .motion_planner import MotionPlanner


class GUI:
    @classmethod
    def _exit_handler(cls):
        CommsManager.disconnect()

    @classmethod
    def _update_gui(cls):
        pass

    # @classmethod
    # def update_graphs(cls):
    #     global error_buffer
    #     with error_buffer_lock:
    #         error_buffer = {key: value for (key, value) in error_buffer.items() if key >= list(error_buffer.keys())[-1] - defs.GRAPH_TIME_RANGE}
    #         timestamps = list(error_buffer.keys())

    #         for i, plot in enumerate(plots):
    #             values = [value[i] for (key, value) in error_buffer.items()]
    #             plot.plot(timestamps, values, clear=True)

    @classmethod
    def init_gui_blocking(cls, mp: MotionPlanner):
        app = QApplication([])
        app.aboutToQuit.connect(cls._exit_handler)
        window = QWidget()
        window.setWindowTitle("FK Grapher")
        
        layout = QVBoxLayout()
        # plots = []
        # plots.append(pg.PlotWidget(title="x error (mm)"))
        # plots.append(pg.PlotWidget(title="y error (mm)"))
        # plots.append(pg.PlotWidget(title="z error (mm)"))
        # for plot in plots: layout.addWidget(plot)

        run_home_button = QPushButton("Run Homing")
        run_home_button.clicked.connect(lambda: mp.home())
        skip_home_button = QPushButton("Skip Homing")
        skip_home_button.clicked.connect(lambda: mp.skip_home())

        active_compliance_button = QPushButton("Active Compliance")
        active_compliance_button.clicked.connect(lambda: mp.run_active_compliance())
        motion_demo_button = QPushButton("Motion Demo")
        motion_demo_button.clicked.connect(lambda: mp.run_motion_demo())
        remote_control_button = QPushButton("Remote Control")
        remote_control_button.clicked.connect(lambda: mp.run_remote_control())
        run_idle_button = QPushButton("Idle")
        run_idle_button.clicked.connect(lambda: mp.run_idle())

        layout.addWidget(run_home_button)
        layout.addWidget(skip_home_button)
        # layout.addWidget(active_compliance_button)
        layout.addWidget(motion_demo_button)
        # layout.addWidget(remote_control_button)
        layout.addWidget(run_idle_button)

        window.setLayout(layout)
        window.show()

        timer = QTimer()
        timer.timeout.connect(cls._update_gui)
        timer.start(50)

        if (sys.flags.interactive != 1) or not hasattr(QtCore, 'PYQT_VERSION'):
            QApplication.instance().exec_()