cmake_minimum_required(VERSION 2.8.3)
project(rviz_pyplot)

find_package(catkin_simple REQUIRED)
catkin_simple()

# This is a pure python package
catkin_python_setup()

install(DIRECTORY examples/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/examples)

install(DIRECTORY etc/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/etc)


cs_install()
cs_export()
