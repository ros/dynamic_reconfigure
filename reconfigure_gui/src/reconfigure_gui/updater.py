import rospy
import dynamic_reconfigure

import threading
import time

class Updater(threading.Thread):
    def __init__(self, reconf):
        super(Updater, self).__init__()
        self.setDaemon(True)

        self._reconf = reconf
        self._cv = threading.Condition()
        self._pending_config = {}
        self._last_pending = None
        self._stop_flag = False

    def run(self):
        last_commit = None

        while not self._stop_flag:
            if last_commit >= self._last_pending:
                    with self._cv:
                        self._cv.wait()

            if self._stop_flag:  
                return

            last_commit = time.time()
            update = self._pending_config.copy()
            self._pending_config = {}

            try:
                updated = self._reconf.update_configuration(update)
            except rospy.ServiceException as ex:
                print('Could not update configuration')
            except Exception as exc:
                raise exc

    def update(self, config):
        with self._cv:
            for name, value in config.items():
                self._pending_config[name] = value

            self._last_pending = time.time()

            self._cv.notify()

    def stop(self):
        self._stop_flag = True 
        with self._cv:
            self._cv.notify()
