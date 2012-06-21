from QtCore import Qt
import QtGui
from QtGui import QWidget, QLabel, QTabWidget, QGridLayout, QGroupBox

from .editors import *

group_types = {
    '': 'BoxGroup',
    'collapse': 'CollapseGroup',
    'tab': 'TabGroup',
    'hide': 'HideGroup',
}

def find_cfg(config, name):
    """
    reaaaaallly cryptic function which returns the config object for specified group.
    """
    cfg = None
    for k, v in config.items():
        try:
            if k.lower() == name.lower():
                cfg = v
                return cfg
            else:
                try:
                    cfg = find_cfg(v, name)
                    if not cfg == None:
                        return cfg
                except Exception as exc:
                    raise exc
        except AttributeError:
            pass
        except Exception as exc:
            raise exc
    return cfg

class Group(QWidget):
    def __init__(self, updater, config):
        super(Group, self).__init__()
        self.state = config['state']
        self.name = config['name']

        # Every group can have one tab bar
        self.tab_bar = None
        self.tab_bar_shown = False

        self.grid = QtGui.QGridLayout()

        self.updater = updater 

        self.widgets = []
        self.add_widgets(config)

        # Labels should not stretch
        self.grid.setColumnStretch(1,1)
        self.setLayout(self.grid)

    def add_widgets(self, descr):
        for param in descr['parameters']:
            if param['edit_method']:
                widget = EnumEditor(self.updater, param)
            elif param['type'] in editor_types:
                widget = eval(editor_types[param['type']])(self.updater, param)

            self.widgets.append(widget)
        
        for group in descr['groups']:
            if group['type'] == 'tab':
                widget = TabGroup(self, self.updater, group)
            elif group['type'] in group_types.keys():
                widget = eval(group_types[group['type']])(self.updater, group)

            self.widgets.append(widget)

        for i, ed in enumerate(self.widgets):
            ed.display(self.grid, i)

    def display(self, grid, row):
        #groups span across all columns
        grid.addWidget(self, row, 0, 1, -1)

    def update_group(self, config):
        self.state = config['state']
        
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
        for w in self.widgets:
            w.close()

class BoxGroup(Group):
    def __init__(self, updater, config):
        super(BoxGroup, self).__init__(updater, config)

        self.box = QGroupBox(self.name)
        self.box.setLayout(self.grid)

    def display(self, grid, row):
        grid.addWidget(self.box, row, 0, 1, -1)

class CollapseGroup(BoxGroup):
    def __init__(self, updater, config):
        super(CollapseGroup, self).__init__(updater, config)
        self.box.setCheckable(True)

class HideGroup(BoxGroup):
    def update_group(self, config):
        super(HideGroup, self).update_group(config)
        self.box.setVisible(self.state)

class TabGroup(Group):
    def __init__(self, parent, updater, config):
        super(TabGroup, self).__init__(updater, config)
        self.parent = parent

        if self.parent.tab_bar is None:
            self.parent.tab_bar = QTabWidget()

        parent.tab_bar.addTab(self, self.name)

    def display(self, grid, row):
        if not self.parent.tab_bar_shown:
            grid.addWidget(self.parent.tab_bar, row, 0, 1, -1)
            self.parent.tab_bar_shown = True

    def close(self):
        super(TabGroup, self).close()
        self.parent.tab_bar = None
        self.parent.tab_bar_shown = False

