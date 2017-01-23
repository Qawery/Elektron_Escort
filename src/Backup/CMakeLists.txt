cmake_minimum_required(VERSION 2.8.3)

project(elektron_escort)

find_package(orocos_kdl REQUIRED)
find_package(catkin REQUIRED COMPONENTS geometry_msgs
					roscpp
					roslib
					tf)
# Find OpenNI
find_package(PkgConfig)
pkg_check_modules(OpenNI REQUIRED libopenni)

catkin_package()

include_directories(${catkin_INCLUDEDIR}
		    ${OpenNI_INCLUDEDIR}
		    ${orocos_kdl_INCLUDE_DIRS})

link_directories(${catkin_LIBRARY_DIRS})
link_directories(${orocos_kdl_LIBRARY_DIRS})

add_executable(person_identification src/person_identification.cpp)
add_executable(skeleton_transforms src/skeleton_transforms.cpp)

add_dependencies(person_identification geometry_msgs_gencpp)
add_dependencies(skeleton_transforms geometry_msgs_gencpp)

target_link_libraries(person_identification ${catkin_LIBRARIES}
				     ${OpenNI_LIBRARIES}
				     ${orocos_kdl_LIBRARIES})
target_link_libraries(skeleton_transforms ${catkin_LIBRARIES}
				     ${OpenNI_LIBRARIES}
				     ${orocos_kdl_LIBRARIES})

install(TARGETS person_identification RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(TARGETS skeleton_transforms RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(FILES person_identification.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
