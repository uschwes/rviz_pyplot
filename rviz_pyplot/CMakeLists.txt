cmake_minimum_required(VERSION 2.8.3)
project(rviz_pyplot)

find_package(catkin_simple REQUIRED)
catkin_simple()

# This is a pure python package
catkin_python_setup()

cs_install()
cs_export()
