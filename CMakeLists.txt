cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(icarus_sim)
find_package(roscpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  gazebo_ros
)

include_directories(${roscpp_INCLUDE_DIRS})
include_directories(${std_msgs_INCLUDE_DIRS})

#set(PROJECT_DEPS
#  roscpp
#)

#find_package(catkin REQUIRED COMPONENTS ${PROJECT_DEPS})
# Find Gazebo
find_package(gazebo REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS} )
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

catkin_package(
  DEPENDS 
    roscpp 
    gazebo_ros 
)
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS ${PROJECT_DEPS}
)

include_directories(include ${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})




add_library(RobotPlugin SHARED src/RobotPlugin.cpp)
add_library(MotorControllerModel src/MotorControllerModel.cpp)
add_library(IMUSensor src/Sensor/IMU/IMUSensor.cpp)
add_library(MotorModel src/MotorModel.cpp)
target_link_libraries(RobotPlugin ${GAZEBO_LIBRARIES} ${roscpp_LIBRARIES} ${catkin_LIBRARIES} MotorControllerModel MotorModel IMUSensor)
add_dependencies(RobotPlugin ${catkin_EXPORTED_TARGETS})



