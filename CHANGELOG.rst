^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package dynamic_reconfigure
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.5.39 (2015-04-22)
-------------------
* Better error message, to fix `#32 <https://github.com/ros/dynamic_reconfigure/issues/32>`_
* Make Python callback code consistent with the C++ API
* Commented unused parameters to avoid compile warnings
* Contributors: Esteve Fernandez, Morgan Quigley

1.5.38 (2014-12-23)
-------------------
* Fixes `#35 <https://github.com/ros/dynamic_reconfigure/issues/35>`_ by setting queue_size to 10 for publishers.
* Fixes `#31 <https://github.com/ros/dynamic_reconfigure/issues/31>`_ by removing boilerplate and copyright info from config header.
* Python 3 Support
* Honor BUILD_SHARED_LIBS and do not force building shared libraries.
* Unicode support
* Contributors: Basheer Subei, Esteve Fernandez, Gary Servin, Kei Okada, Scott K Logan

1.5.37 (2014-06-16)
-------------------
* Decode level of ParamDescription
* Added testsuite
* Avoid collisions with parameter names (`#6 <https://github.com/ros/dynamic_reconfigure/issues/6>`_)
* Contributors: Esteve Fernandez, pgorczak
