import roslib;roslib.load_manifest('dynamic_reconfigure')
import rospy

from qt_gui.plugin import Plugin

from .reconfigure_widget import ReconfigureWidget

class ReconfigurePlugin(Plugin):
    def __init__(self, context):
        super(ReconfigurePlugin, self).__init__(context)

        self._widget = ReconfigureWidget()

        self.setObjectName('Reconfigure')

        context.add_widget(self._widget)

    def shutdown_plugin(self):
        self._widget.shutdown_plugin()
