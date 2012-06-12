from QtGui import QWidget, QPushButton

class ReconfigureWidget(QWidget):
    def __init__(self):
        super(ReconfigureWidget, self).__init__()

        pb = QPushButton('Red', self)
        pb.move(10,10)

    def shutdown_plugin(self):
        #TODO: Proper shutdown
        pass
