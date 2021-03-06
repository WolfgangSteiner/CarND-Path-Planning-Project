project(Path_Planning)

cmake_minimum_required (VERSION 3.5)

add_definitions(-std=c++11 -O3)

set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS, "${CXX_FLAGS}")

set(sources src/main.cpp src/Waypoints.cpp src/Utils.cpp src/VehicleController.cpp
        src/Trajectory.cpp src/VehicleState.cpp src/KeepLaneState.cpp
        src/SensorFusion.cpp src/SensorFusion.h src/OtherCar.cpp
        src/TrajectoryCollection.cpp src/StateMachine.cpp
        src/ChangeLaneState.cpp)


if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")

include_directories(/usr/local/include)
include_directories(/usr/local/opt/openssl/include)
link_directories(/usr/local/lib)
link_directories(/usr/local/opt/openssl/lib)
link_directories(/usr/local/Cellar/libuv/1.11.0/lib)

endif(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")


add_executable(path_planning ${sources})
add_executable(tests
        src/test/test.cpp src/test/trajectory.cpp  src/test/waypoints.cpp
        src/Trajectory.cpp src/Waypoints.cpp src/Utils.cpp)

target_link_libraries(path_planning z ssl uv uWS)
target_link_libraries(tests)
