cmake_minimum_required(VERSION 2.8.3)

project(elektron_escort)

find_package(orocos_kdl REQUIRED)
find_package(catkin REQUIRED COMPONENTS
					geometry_msgs
					roscpp
					roslib
					tf)

add_definitions("-std=gnu++11")

# Find OpenNI
find_package(PkgConfig)
pkg_check_modules(OpenNI REQUIRED libopenni)

catkin_package()

include_directories(${catkin_INCLUDEDIR}
		    ${OpenNI_INCLUDEDIR}
		    ${orocos_kdl_INCLUDE_DIRS})

link_directories(${catkin_LIBRARY_DIRS})
link_directories(${orocos_kdl_LIBRARY_DIRS})

add_executable(escort_main src/escort_main.cpp
        src/SensorsModule.cpp
        src/TaskModule.cpp
        src/MobilityModule.cpp
        src/DataStorage.cpp)

target_link_libraries(escort_main ${catkin_LIBRARIES}
				     ${OpenNI_LIBRARIES}
				     ${orocos_kdl_LIBRARIES})

install(TARGETS escort_main RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
