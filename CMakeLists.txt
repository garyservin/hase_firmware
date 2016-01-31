cmake_minimum_required(VERSION 2.8.3)
project(hase_firmware)

find_package(catkin REQUIRED COMPONENTS
  hase_msgs
  rosserial_client
  rosserial_mbed
)

catkin_package()
