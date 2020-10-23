cmake_minimum_required(VERSION 3.0.0)
project(robodog VERSION 1.0.0)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(Boost_USE_STATIC_LIBS OFF)
set(Boost_USE_MULTITHREADED ON)
set(Boost_USE_STATIC_RUNTIME OFF)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
)

catkin_python_setup()

#add_definitions(-DBOOST_ERROR_CODE_HEADER_ONLY)

find_package(Boost COMPONENTS filesystem thread)

include(CTest)
enable_testing()

add_service_files(
  FILES
  GcodeService.srv
  SayService.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package()

include_directories(~/catkin_ws/src/serial/include "${PROJECT_SOURCE_DIR}/Include")
include_directories(${Boost_INCLUDE_DIRS})
include_directories(${catkin_INCLUDE_DIRS})
link_directories(~/catkin_ws/devel/lib)

#message(STATUS "Include: " ${INCLUDE_DIRECTORIES})

set(DOG_LIB_SOURCES
    IK.cpp
    Leg.cpp
    Joints/JointSingle.cpp Joints/JointDouble.cpp
    Servos/LX16a.cpp Servos/Servo.cpp Servos/ServoLX16a.cpp Servos/ServoController.cpp
    Robots/RobotV2.cpp Robots/RobotV3.cpp
    Trajectory/MovePrimitives.cpp Trajectory/MoveParams.cpp Trajectory/GCodeController.cpp Trajectory/BaseState.cpp
    RemoteControl/Button.cpp RemoteControl/CommandMenu.cpp RemoteControl/RemoteController.cpp
    Utils.cpp MathHelpers.cpp Filters.cpp)

# robodog_lib
add_library(${PROJECT_NAME}_lib STATIC ${DOG_LIB_SOURCES})

# robodog_gcode_service
add_executable(${PROJECT_NAME}_hw
  Services/RobotHW.cpp
  Services/SayService.cpp)
target_link_libraries(${PROJECT_NAME}_hw
    ${PROJECT_NAME}_lib
    libserial.so
    ${Boost_LIBRARIES}
    ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME}_hw ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# robodog_client
add_executable(${PROJECT_NAME}_client Services/RobotClient.cpp)
target_link_libraries(${PROJECT_NAME}_client
    ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME}_client ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)
