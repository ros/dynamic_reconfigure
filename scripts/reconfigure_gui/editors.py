from QtCore import Qt
import QtGui
from QtGui import QWidget, QLabel, QCheckBox, QLineEdit, QSlider, QHBoxLayout

import sys

import math

editor_types = {
    'bool': 'BooleanEditor',
    'str': 'StringEditor',
    'double': 'DoubleEditor',
}

class Editor(QWidget):
    def __init__(self, updater, config):
        super(Editor, self).__init__()
        
        self.updater = updater
        self.name = config['name']

        self.old_value = None

    def _update(self, value):
        if value != self.old_value:
            self.update_configuration(value)
            self.old_value = value

    def update_value(self, value):
        pass

    def update_configuration(self, value):
        self.updater.update({ self.name : value })

    def display(self, grid, row):
        pass

class BooleanEditor(Editor):
    def __init__(self, updater, config): 
        super(BooleanEditor, self).__init__(updater, config)

        hbox = QHBoxLayout()
        self.cb = QCheckBox('', self)

        hbox.addWidget(self.cb)
        self.setLayout(hbox)

        self.update_value(config['default'])

        self.cb.stateChanged.connect(self._update)

    def update_value(self, value):
        self.cb.setChecked(value)

    def display(self, grid, row):
        grid.addWidget(QLabel(self.name), row, 0, Qt.AlignRight)

        grid.addWidget(self, row, 1)

class StringEditor(Editor):
    def __init__(self, updater, config):
        super(StringEditor, self).__init__(updater, config)

        hbox = QHBoxLayout()
        self.tb = QLineEdit(config['default'])

        hbox.addWidget(self.tb)
        self.setLayout(hbox)

        self.tb.editingFinished.connect(self.edit_finished)

    def update_value(self, value):
        self.tb.setString(value)

    def edit_finished(self):
        self._update(self.tb.text())

    def display(self, grid, row):
        grid.addWidget(QLabel(self.name), row, 0, Qt.AlignRight)

        grid.addWidget(self, row, 1)

class DoubleEditor(Editor):
    def __init__(self, updater, config):
        super(DoubleEditor, self).__init__(updater,config)

        # Handle unbounded doubles nicely
        try:
            self.min = float(config['min'])
            self.func = lambda x: x
            self.ifunc = self.func
        except:
            self.min = -1e10000 
            self.func = lambda x: math.atan(x)
            self.ifunc = lambda x: math.tan(x)

        try:
            self.max = float(config['max'])
            self.func = lambda x: x
            self.ifunc = self.func
        except:
            self.max = 1e10000
            self.func = lambda x: math.atan(x)
            self.ifunc = lambda x: math.tan(x)

        self.scale = (self.func(self.max) - self.func(self.min))/100
        self.offset = self.func(self.min)

        hbox = QHBoxLayout()

        self.slider = QSlider(Qt.Horizontal)
        self.slider.setRange(self.slider_value(self.min), self.slider_value(self.max))
        self.slider.sliderReleased.connect(self.slider_release)
        self.slider.sliderMoved.connect(self.update_text)

        self.tb = QLineEdit()
        self.tb.editingFinished.connect(self.editing_finished)

        hbox.addWidget(self.slider, 1)
        hbox.addWidget(self.tb, 0)
        self.setLayout(hbox)

        print(config)

        self.slider.setSliderPosition(self.slider_value(config['default']))
        self.tb.setText(str(config['default']))

    def get_value(self):
        return self.ifunc(self.slider.value() * self.scale)

    def slider_value(self, value):
        return int(round((self.func(value))/self.scale))

    def slider_release(self):
        self.update_text(self.get_value())
        self._update(self.get_value())

    def update_text(self, value):
        self.tb.setText('%s'% self.get_value())

    def editing_finished(self):
        self.slider.setSliderPosition(self.slider_value(float(self.tb.text())))
        self._update(float(self.tb.text()))

    def display(self, grid, row):
        grid.addWidget(QLabel(self.name), row, 0, Qt.AlignRight)

        grid.addWidget(self, row, 1)
