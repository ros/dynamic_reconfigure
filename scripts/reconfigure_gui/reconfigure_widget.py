import sys

import QtGui
from QtGui import QWidget, QPushButton, QComboBox, QScrollArea
from QtCore import QTimer

import dynamic_reconfigure.client
import rosservice
import rospy

from .editors import *
from .groups import *
from .updater import Updater

class ReconfigureWidget(QWidget):
    def __init__(self):
        super(ReconfigureWidget, self).__init__()

        self.selector = ReconfigureSelector(self)

        self.client = None

        hbox = QtGui.QHBoxLayout()
        hbox.addWidget(self.selector)

        self.vbox = QtGui.QVBoxLayout()
        self.vbox.addLayout(hbox)
        self.stretch = self.vbox.addStretch(1)

        self.setLayout(self.vbox)

    def show(self, node):
        self.close()

        reconf = None
        
        try:
            reconf = dynamic_reconfigure.client.Client(node, timeout=5.0)
        except rospy.exceptions.ROSException:
            print("Could not connect to %s"%node) 
            return
        finally:
            self.close()

        self.client = ClientWidget(reconf)

        self.scroll = QScrollArea()
        self.scroll.setWidget(self.client)
        self.scroll.setWidgetResizable(True)

        self.vbox.insertWidget(1, self.scroll, 1)
        self.vbox.removeItem(self.vbox.itemAt(2))
        self.stretch = None

    def close(self):
        if self.client is not None:
            # Clear out the old widget
            self.client.close()
            self.client = None

            self.scroll.deleteLater()

    def clear(self):
        if not self.stretch:
            self.stretch = self.vbox.addStretch(1)

class ReconfigureSelector(QWidget):
    def __init__(self, parent):
        super(ReconfigureSelector, self).__init__()

        self.parent = parent
        self.last_nodes = None

        self.combo = QComboBox(self)
        self.update_combo()
        self.combo.activated[str].connect(self.selected)

        self.hbox = QtGui.QHBoxLayout()
        self.hbox.addWidget(self.combo)
        
        self.setLayout(self.hbox)

        self.timer = QTimer()
        self.timer.timeout.connect(self.update_combo)
        self.timer.start(100)

    def update_combo(self):
        try:
            nodes = dynamic_reconfigure.find_reconfigure_services()
        except rosservice.ROSServiceIOException:
            print("Reconfigure GUI cannot connect to master.")
        else:
            if not self.last_nodes:
                for n in nodes:
                    self.combo.addItem(n)
            elif len(nodes) == 0:
                self.combo.clear()
                self.parent.close()
                self.parent.clear()
            else:
                for i, n in enumerate(self.last_nodes):
                    if not n in nodes:
                        self.combo.removeItem(i)

                for n in nodes:
                    if not n in self.last_nodes:
                        self.combo.addItem(n)

            self.last_nodes = nodes

    def selected(self, node):
        self.parent.show(node)

class ClientWidget(Group):
    def __init__(self, reconf):
        super(ClientWidget, self).__init__(Updater(reconf), reconf.get_group_descriptions())

        self.reconf = reconf

        self.updater.start()
        self.reconf.config_callback = self.config_callback

    def config_callback(self, config):
        if config is not None:
            # TODO: should use config.keys but this method doesnt exist
            names = [name for name, v in config.items()]

            for widget in self.widgets:
                if isinstance(widget, Editor):
                    if widget.name in names:
                        widget.update_value(config[widget.name])
                elif isinstance(widget, Group):
                    cfg = find_cfg(config, widget.name)
                    widget.update_group(cfg)
                
    def close(self):
        self.reconf.close()
        self.updater.stop()

        for w in self.widgets:
            w.close()

        self.deleteLater()

