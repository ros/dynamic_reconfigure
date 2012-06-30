import sys

import QtGui
from QtGui import QWidget, QPushButton, QComboBox, QScrollArea, QTreeWidget, QTreeWidgetItem
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
        self.client = None
        self.stretch = None

        self.hbox = QtGui.QHBoxLayout()
        self.selector = ReconfigureSelector(self)
        self.hbox.addWidget(self.selector.tree)

        self.setLayout(self.hbox)

    def show(self, node):
        self.close()

        reconf = None
        
        try:
            reconf = dynamic_reconfigure.client.Client(str(node), timeout=5.0)
        except rospy.exceptions.ROSException:
            print("Could not connect to %s"%node) 
            return
        finally:
            self.close()

        self.client = ClientWidget(reconf)

        self.scroll = QScrollArea()
        # Buffer min width so no horizontal scroll bar appears
        self.scroll.setMinimumWidth(self.client.minimumWidth()+20)

        self.scroll.setWidget(self.client)
        self.scroll.setWidgetResizable(True)

        self.hbox.insertWidget(1, self.scroll, 1)
        self.hbox.removeItem(self.hbox.itemAt(2))
        self.stretch = None

    def close(self):
        if self.client is not None:
            # Clear out the old widget
            self.client.close()
            self.client = None

            self.scroll.deleteLater()

    def clear(self):
        if not self.stretch:
            self.stretch = self.hbox.addStretch(1)

class ReconfigureSelector(QWidget):
    def __init__(self, parent):
        super(ReconfigureSelector, self).__init__()

        self.parent = parent
        self.last_nodes = None
        self.current_node = ''

        self.tree = QTreeWidget()
        self.update_tree()

        self.tree.itemClicked.connect(self.selected)
        self.tree.setSelectionMode(QtGui.QAbstractItemView.SingleSelection)

        header = QTreeWidgetItem()
        header.setText(0, 'Nodes')
        self.tree.setHeaderItem(header)

        self.timer = QTimer()
        self.timer.timeout.connect(self.update_tree)
        self.timer.start(100)

    def update_tree(self):
        try:
            nodes = dynamic_reconfigure.find_reconfigure_services()
        except rosservice.ROSServiceIOException:
            print("Reconfigure GUI cannot connect to master.")
        else:
            if not nodes == self.last_nodes:
                self.tree.clear()
                for n in nodes:
                    item = QTreeWidgetItem()
                    item.setText(0, n)
                    self.tree.addTopLevelItem(item)
            elif len(nodes) == 0:
                self.tree.clear()
                self.parent.close()
                self.parent.clear()

            self.last_nodes = nodes

    def selected(self, node, col):
        print("Selected %s"%node.text(0))
        if not node.text(0) == self.current_node:
            self.parent.show(node.text(0))
            self.current_node = node.text(0)

class ClientWidget(Group):
    def __init__(self, reconf):
        super(ClientWidget, self).__init__(Updater(reconf), reconf.get_group_descriptions())

        self.setMinimumWidth(550)

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

