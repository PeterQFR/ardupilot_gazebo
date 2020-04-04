cmake_minimum_required(VERSION 2.8 FATAL_ERROR)


project(ardupilot_sitl_gazebo)


 add_compile_options(-std=c++11 -DBOOST_VARIANT_USE_RELAXED_GET_BY_DEFAULT)

#######################
## Find Dependencies ##
#######################
find_package(catkin REQUIRED
roscpp
rospy
)
find_package(gazebo REQUIRED)

find_package(Boost REQUIRED)
find_package(SDFormat 6 REQUIRED)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

if("${GAZEBO_VERSION}" VERSION_LESS "7.0")
    message(FATAL_ERROR "You need at least Gazebo 7.0. Your version: ${GAZEBO_VERSION}")
else()
    message("Gazebo version: ${GAZEBO_VERSION}")
endif()

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES orb_slam
#  CATKIN_DEPENDS cv_bridge geometry_msgs image_transport roscpp rospy std_msgs
#  DEPENDS system_lib
)
###########
## Build ##
###########

include_directories(
        ${PROJECT_SOURCE_DIR}
        include
        ${GAZEBO_INCLUDE_DIRS}
        ${catkin_INCLUDE_DIRS}
        )

#link_libraries(
#        ${CMAKE_CURRENT_BINARY_DIR}
#        ${GAZEBO_LIBRARIES}
#        ${Boost_LIBRARIES}
#        )


#link_directories(
#        ${GAZEBO_LIBRARY_DIRS}
#        )

#set (plugins_single_header
#        ArduPilotPlugin
#        # ArduCopterIRLockPlugin
#				ArduCopterPlugin
#        GimbalSmall2dPlugin
#	# WindPlugin
#        )

#add_library(ArduCopterIRLockPlugin SHARED src/ArduCopterIRLockPlugin.cc)
#target_link_libraries(ArduCopterIRLockPlugin ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})


add_library(ArduPilotPlugin SHARED src/ArduPilotPlugin.cc)
target_link_libraries(ArduPilotPlugin ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})

add_library(ArduCopterPlugin SHARED src/ArduCopterPlugin.cc)
target_link_libraries(ArduCopterPlugin ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})

add_executable(test_node
 src/test/test.cpp
 
 )
target_link_libraries(test_node 
 
 ${catkin_LIBRARIES}
 ${GAZEBO_LIBRARIES}
 )

if("${GAZEBO_VERSION}" VERSION_LESS "8.0")
    add_library(GimbalSmall2dPlugin SHARED src/GimbalSmall2dPlugin.cc)
    target_link_libraries(GimbalSmall2dPlugin ${GAZEBO_LIBRARIES})
#    install(TARGETS GimbalSmall2dPlugin DESTINATION ${GAZEBO_PLUGIN_PATH})
endif()

# install(TARGETS ArduCopterIRLockPlugin DESTINATION ${GAZEBO_PLUGIN_PATH})
#install(TARGETS ArduPilotPlugin DESTINATION ${GAZEBO_PLUGIN_PATH})
#install(TARGETS ArduCopterPlugin DESTINATION ${GAZEBO_PLUGIN_PATH})
