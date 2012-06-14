import sys

import QtGui
from QtGui import QWidget, QPushButton, QComboBox
from QtCore import QTimer

import dynamic_reconfigure.client
import rosservice

class ReconfigureWidget(QWidget):
    def __init__(self):
        super(ReconfigureWidget, self).__init__()

        self.selector = ReconfigureSelector(self)

        hbox = QtGui.QHBoxLayout()
        hbox.addWidget(self.selector)
        
        vbox = QtGui.QVBoxLayout()
        vbox.addLayout(hbox)
        vbox.addStretch(1)

        self.setLayout(vbox)

    def show(self, node):
        print(node) 

    def shutdown_plugin(self):
        #TODO: Proper shutdown
        pass

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
