rosbuild_add_executable(testserver testserver.cpp)
target_link_libraries(testserver pthread)
#rosbuild_add_rostest(test/test_cpp.xml)
