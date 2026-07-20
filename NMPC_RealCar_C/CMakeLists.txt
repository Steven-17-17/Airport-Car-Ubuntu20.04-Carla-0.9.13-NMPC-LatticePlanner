cmake_minimum_required(VERSION 3.10)

project(nmpc_realcar_cpp LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(BUILD_ROS1 "Build the ROS1 wrapper node" OFF)

if(BUILD_ROS1)
    find_package(catkin REQUIRED COMPONENTS roscpp std_msgs geometry_msgs nav_msgs carla_msgs)

    catkin_package(
        INCLUDE_DIRS include
        LIBRARIES nmpc_realcar_cpp_core
        CATKIN_DEPENDS roscpp std_msgs geometry_msgs nav_msgs carla_msgs)

    include_directories(
        include
        ${catkin_INCLUDE_DIRS})

    add_library(nmpc_realcar_cpp_core STATIC test_trailer_parking.cpp)
    target_compile_definitions(nmpc_realcar_cpp_core PRIVATE NMPC_REALCAR_C_LIBRARY_ONLY)
    target_compile_options(nmpc_realcar_cpp_core PRIVATE -Wall -Wextra -Wpedantic)

    add_executable(ros_trailer_parking_node src/ros_trailer_parking_node.cpp)
    target_link_libraries(ros_trailer_parking_node PRIVATE nmpc_realcar_cpp_core ${catkin_LIBRARIES})
    add_dependencies(ros_trailer_parking_node ${catkin_EXPORTED_TARGETS})

    install(TARGETS ros_trailer_parking_node
            RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
    install(TARGETS nmpc_realcar_cpp_core
            ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
    install(DIRECTORY include/
            DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
    install(PROGRAMS launch_original_scene.py
            DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
else()
    add_executable(test_trailer_parking test_trailer_parking.cpp)
    target_include_directories(test_trailer_parking PRIVATE include)
    target_compile_options(test_trailer_parking PRIVATE -Wall -Wextra -Wpedantic)

    install(TARGETS test_trailer_parking
            RUNTIME DESTINATION bin)
endif()
