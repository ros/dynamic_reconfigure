^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package dynamic_reconfigure
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.6.3 (2020-03-19)
------------------
* Revert `#140 <https://github.com/ros/dynamic_reconfigure/issues/140>`_ (`#152 <https://github.com/ros/dynamic_reconfigure/issues/152>`_)
* Contributors: Michael Carroll

1.6.2 (2020-03-19)
------------------
* Set right order of SYSTEM/BEFORE options in dynamic_reconfigure-macros.cmake. Fix `#150 <https://github.com/ros/dynamic_reconfigure/issues/150>`_ (`#151 <https://github.com/ros/dynamic_reconfigure/issues/151>`_)
* Contributors: Alexander

1.6.1 (2020-03-05)
------------------
* Use PYTHON_EXECUTABLE to generate config headers. (`#146 <https://github.com/ros/dynamic_reconfigure/issues/146>`_)
* Python3 compatibility (`#135 <https://github.com/ros/dynamic_reconfigure/issues/135>`_)
* Use system on gen headers (`#140 <https://github.com/ros/dynamic_reconfigure/issues/140>`_)
* Fix GCC8 error for unnecessary parentheses (`#132 <https://github.com/ros/dynamic_reconfigure/issues/132>`_)
* fix generate_dynamic_reconfigure_options (`#10 <https://github.com/ros/dynamic_reconfigure/issues/10>`_) (`#134 <https://github.com/ros/dynamic_reconfigure/issues/134>`_)
* Make Michael Carroll the maintainer (`#125 <https://github.com/ros/dynamic_reconfigure/issues/125>`_)
* Contributors: Christopher Wecht, Markus Grimm, Michael Carroll, Mikael Arguedas, Nicolas Limpert, Sean Yen [MSFT], Victor Lopez

1.6.0 (2018-10-02)
------------------
* fix check preventing range for string and bool parameters (`#122 <https://github.com/ros/dynamic_reconfigure/issues/122>`_)
* Fix build issue on Windows (`#114 <https://github.com/ros/dynamic_reconfigure/issues/114>`_)
* Contributors: Johnson Shih, Mikael Arguedas

1.5.50 (2018-10-02)
-------------------
* final-keyword (`#113 <https://github.com/ros/dynamic_reconfigure/issues/113>`_)
  * Add final keyword to child class since parent has virtual methods and grand parent doesn't have a virtual destructor. This allows the code to be compiled by clang version 6.0 and above.
* [indentation fixups]
  * Use textwrap dedent for multiline strings
  * Remove extra indentation in wikidoc
  * Use textwrap.dedent to form the error message
* [test fix] call shutdown to prevent test from hanging (`#119 <https://github.com/ros/dynamic_reconfigure/issues/119>`_)
* Modernize Python code (`#102 <https://github.com/ros/dynamic_reconfigure/issues/102>`_)
  * Use new-style classes
  * Use with statement to ensure files are closed
* Python 3 compatibility (`#105 <https://github.com/ros/dynamic_reconfigure/issues/105>`_)
  * some randon python cleanup
  * remove iter* method for their 2/3 compatible equivalent
* Contributors: Eric Wieser, Jason Mercer, Mikael Arguedas

1.5.49 (2017-07-27)
-------------------
* Pep8 (`#93 <https://github.com/ros/dynamic_reconfigure/issues/93>`_)
* Python3 use print function and not statement (`#92 <https://github.com/ros/dynamic_reconfigure/issues/92>`_)
* add shebang line in setup_custom_pythonpath.sh.in template (`#91 <https://github.com/ros/dynamic_reconfigure/issues/91>`_)
* switch to package format 2 (`#89 <https://github.com/ros/dynamic_reconfigure/issues/89>`_)
* remove trailing whitespaces (`#88 <https://github.com/ros/dynamic_reconfigure/issues/88>`_)
* Reraising DynamicReconfigureCallbackException in case of a failing service call (`#87 <https://github.com/ros/dynamic_reconfigure/issues/87>`_)
* Contributors: Arne Hitzmann, Mikael Arguedas

1.5.48 (2017-04-07)
-------------------
* [Bugfix] dont enforce ROS names for constants (`#84 <https://github.com/ros/dynamic_reconfigure/issues/84>`_)
* [Compiler warnings] avoid unused-parameter compiler warnings in specialized ParamDescription<std::string>::clamp() (`#83 <https://github.com/ros/dynamic_reconfigure/issues/83>`_)
* Contributors: Johannes Meyer, Mikael Arguedas

1.5.47 (2017-03-27)
-------------------
* reset received_configuration\_ for every request sent (`#82 <https://github.com/ros/dynamic_reconfigure/issues/82>`_)
* Rename arguments (with a\_ prefix) to avoid Wshadow warnings. (`#80 <https://github.com/ros/dynamic_reconfigure/issues/80>`_)
  handle infinity in python generation, fixes (`#77 <https://github.com/ros/dynamic_reconfigure/issues/77>`_)
* Add a c++ Dynamic Reconfigure Client (`#78 <https://github.com/ros/dynamic_reconfigure/issues/78>`_)
* Enforce valid descriptions in cfg files (`#74 <https://github.com/ros/dynamic_reconfigure/issues/74>`_)
* Fix callback returned by get_description_callback (`#73 <https://github.com/ros/dynamic_reconfigure/issues/73>`_) from ros/description_cb
* Contributors: Jeff Eberl, Mikael Arguedas

1.5.46 (2016-11-15)
-------------------
* Add missing group params to wikidoc (`#68 <https://github.com/ros/dynamic_reconfigure/issues/68>`_)
  The catkin generated wikidoc files were missing parameters defined as groups.
  Both the Dox and UsageDox file were generated correctly, but the wikidoc was
  using the wrong method to traverse all groups.
* Contributors: Mark Horn

1.5.45 (2016-10-24)
-------------------
* Merge pull request `#65 <https://github.com/ros/dynamic_reconfigure/issues/65>`_ from bulwahn/master
  address gcc6 build error
* address gcc6 build error
  With gcc6, compiling fails with `stdlib.h: No such file or directory`,
  as including '-isystem /usr/include' breaks with gcc6, cf.,
  https://gcc.gnu.org/bugzilla/show_bug.cgi?id=70129.
  This commit addresses this issue for this package in the same way
  it was addressed in various other ROS packages. A list of related
  commits and pull requests is at:
  https://github.com/ros/rosdistro/issues/12783
  Signed-off-by: Lukas Bulwahn <lukas.bulwahn@oss.bmw-carit.de>
* Contributors: Lukas Bulwahn, Mikael Arguedas

1.5.44 (2016-06-22)
-------------------
* Add server namespaces (`#56 <https://github.com/ros/dynamic_reconfigure/issues/56>`_)
  * Add optional namespace argument to Python Server
  * Add test for server with multiple namespaces
* Merge pull request `#61 <https://github.com/ros/dynamic_reconfigure/issues/61>`_ from vagvaz/Issue_51_Unable_to_reload_parameters_from_file
  fix issue `#51 <https://github.com/ros/dynamic_reconfigure/issues/51>`_ reloading parameters from dumped file
* Contributors: Evangelos Vazaios, Mikael Arguedas, v-lopez

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
