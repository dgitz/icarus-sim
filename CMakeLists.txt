cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(icarus_sim)
find_package(roscpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  gazebo_ros
  eros
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
add_dependencies(RobotPlugin eros_generate_messages_cpp)
add_library(LinearActuatorModel src/Actuator/LinearActuatorModel/LinearActuatorModel.cpp)
add_dependencies(LinearActuatorModel eros_generate_messages_cpp)
add_library(MotorControllerModel src/Power/MotorControllerModel/MotorControllerModel.cpp)
add_dependencies(MotorControllerModel eros_generate_messages_cpp)
add_library(IMUSensor src/Sensor/IMU/IMUSensor.cpp)
add_dependencies(IMUSensor eros_generate_messages_cpp)
add_library(TruthSensor src/Sensor/Truth/TruthSensor.cpp)
add_dependencies(TruthSensor eros_generate_messages_cpp)
add_library(WheelEncoderSensor src/Sensor/WheelEncoder/WheelEncoderSensor.cpp)
add_dependencies(WheelEncoderSensor eros_generate_messages_cpp)
add_library(MotorModel src/Power/MotorModel/MotorModel.cpp)
add_dependencies(MotorModel eros_generate_messages_cpp)
add_library(BatteryModel src/Power/BatteryModel/BatteryModel.cpp)
add_dependencies(BatteryModel eros_generate_messages_cpp)
target_link_libraries(RobotPlugin ${GAZEBO_LIBRARIES} ${roscpp_LIBRARIES} ${catkin_LIBRARIES} logger eros_math MotorControllerModel MotorModel IMUSensor TruthSensor WheelEncoderSensor BatteryModel LinearActuatorModel)
add_dependencies(RobotPlugin ${catkin_EXPORTED_TARGETS})



