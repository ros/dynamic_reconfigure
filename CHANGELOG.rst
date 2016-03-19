^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package dynamic_reconfigure
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.5.43 (2016-03-19)
-------------------
* add devel space to Python environment to allow .cfg files to import them `#60 <https://github.com/ros/dynamic_reconfigure/issues/60>`_
* Contributors: Dirk Thomas

1.5.42 (2016-03-15)
-------------------
* fix Python environment to make it work on the first run `#59 <https://github.com/ros/dynamic_reconfigure/issues/59>`_
* Contributors: Dirk Thomas

1.5.41 (2016-03-14)
-------------------
* fix Python environment to make it work on the first run `#58 <https://github.com/ros/dynamic_reconfigure/issues/58>`_ 
* Contributors: Dirk Thomas, Mikael Arguedas

1.5.40 (2016-03-11)
-------------------
* updated maintainer
* Contributors: Mikael Arguedas

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
