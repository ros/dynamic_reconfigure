from QtCore import Qt
import QtGui
from QtGui import QWidget, QLabel, QCheckBox, QLineEdit, QSlider, QComboBox, QHBoxLayout

import sys
import math

editor_types = {
    'bool': 'BooleanEditor',
    'str': 'StringEditor',
    'int': 'IntegerEditor',
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

    def close(self):
        pass

class BooleanEditor(Editor):
    def __init__(self, updater, config): 
        super(BooleanEditor, self).__init__(updater, config)

        hbox = QHBoxLayout()
        self.cb = QCheckBox('', self)

        hbox.addWidget(self.cb)
        self.setLayout(hbox)

        self.update_value(config['default'])

        self.cb.clicked.connect(self._update)

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
        self.tb.setText(value)

    def edit_finished(self):
        self._update(self.tb.text())

    def display(self, grid, row):
        grid.addWidget(QLabel(self.name), row, 0, Qt.AlignRight)

        grid.addWidget(self, row, 1)


class IntegerEditor(Editor):
    def __init__(self, updater, config):
        super(IntegerEditor, self).__init__(updater, config)
         
        hbox = QHBoxLayout()

        self.min = int(config['min'])
        self.max = int(config['max'])

        self.min_label = QLabel(str(self.min))
        self.max_label = QLabel(str(self.max))

        self.slider = QSlider(Qt.Horizontal)
        self.slider.setRange(self.min, self.max)
        self.slider.sliderReleased.connect(self.slider_released)
        self.slider.sliderMoved.connect(self.update_text)

        self.tb = QLineEdit()   
        self.tb.setValidator(QtGui.QIntValidator(self.min, self.max))
        self.tb.editingFinished.connect(self.editing_finished)

        hbox.addWidget(self.min_label, 0)
        hbox.addWidget(self.slider, 1)
        hbox.addWidget(self.max_label, 0)
        hbox.addWidget(self.tb, 0)

        self.setLayout(hbox)

        # TODO: This should not always get set to the default it should be the current value
        self.tb.setText(str(config['default']))
        self.slider.setSliderPosition(int(config['default']))

    def slider_released(self):
        self.update_text(self.slider.value())
        self._update(self.slider.value())

    def update_text(self, val):
        self.tb.setText(str(val))

    def editing_finished(self):
        self.slider.setSliderPosition(int(self.tb.text()))
        self._update(int(self.tb.text()))

    def update_value(self, val):
        self.slider.setSliderPosition(int(val))
        self.tb.setText(str(val))

    def display(self, grid, row):
        grid.addWidget(QLabel(self.name), row, 0, Qt.AlignRight)
        grid.addWidget(self, row, 1)

class DoubleEditor(Editor):
    def __init__(self, updater, config):
        super(DoubleEditor, self).__init__(updater,config)

        # Handle unbounded doubles nicely
        if config['min'] != -float('inf'):
            self.min = float(config['min'])
            self.min_label = QLabel(str(self.min))

            self.func = lambda x: x
            self.ifunc = self.func
        else:
            self.min = -1e10000 
            self.min_label = QLabel('-inf')

            self.func = lambda x: math.atan(x)
            self.ifunc = lambda x: math.tan(x)

        if config['max'] != float('inf'):
            self.max = float(config['max'])
            self.max_label = QLabel(str(self.max))

            self.func = lambda x: x
            self.ifunc = self.func
        else:
            self.max = 1e10000
            self.max_label = QLabel('inf')

            self.func = lambda x: math.atan(x)
            self.ifunc = lambda x: math.tan(x)

        self.scale = (self.func(self.max) - self.func(self.min))/100
        self.offset = self.func(self.min)

        hbox = QHBoxLayout()

        self.slider = QSlider(Qt.Horizontal)
        self.slider.setRange(self.slider_value(self.min), self.slider_value(self.max))
        self.slider.sliderReleased.connect(self.slider_released)
        self.slider.sliderMoved.connect(self.update_text)

        self.tb = QLineEdit()
        self.tb.setValidator(QtGui.QDoubleValidator(self.min, self.max, 4))
        self.tb.editingFinished.connect(self.editing_finished)

        hbox.addWidget(self.min_label, 0)
        hbox.addWidget(self.slider, 1)
        hbox.addWidget(self.max_label, 0)
        hbox.addWidget(self.tb, 0)

        self.setLayout(hbox)

        self.tb.setText(str(config['default']))
        self.slider.setSliderPosition(self.slider_value(config['default']))

    def get_value(self):
        return self.ifunc(self.slider.value() * self.scale)

    def slider_value(self, value):
        return int(round((self.func(value))/self.scale))

    def slider_released(self):
        self.update_text(self.get_value())
        self._update(self.get_value())

    def update_text(self, value):
        self.tb.setText(str(self.get_value()))

    def editing_finished(self):
        self.slider.setSliderPosition(self.slider_value(float(self.tb.text())))
        self._update(float(self.tb.text()))

    def update_value(self, val):
        self.slider.setSliderPosition(self.slider_value(float(val)))
        self.tb.setText(str(val))

    def display(self, grid, row):
        grid.addWidget(QLabel(self.name), row, 0, Qt.AlignRight)
        grid.addWidget(self, row, 1)

class EnumEditor(Editor):
    def __init__(self, updater, config):
        super(EnumEditor, self).__init__(updater, config)

        try:
            enum = eval(config['edit_method'])['enum']
        except:
            print("Malformed enum")
            return

        self.names = [item['name'] for item in enum]
        self.values = [item['value'] for item in enum]

        items = ["%s (%s)"%(self.names[i], self.values[i]) for i in range(0, len(self.names))]

        self.combo = QComboBox()
        self.combo.addItems(items)
        self.combo.currentIndexChanged['int'].connect(self.selected)

        hbox = QHBoxLayout()
        hbox.addWidget(self.combo)

        self.setLayout(hbox)

    def selected(self, index):
        self._update(self.values[index])

    def update_value(self, val):
        self.combo.setCurrentIndex(self.values.index(val))

    def display(self, grid, row):
        grid.addWidget(QLabel(self.name), row, 0, Qt.AlignRight)
        grid.addWidget(self, row, 1)
