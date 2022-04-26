cmake_minimum_required(VERSION 3.0.2)
project(rbiz_autorace_monitor)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    cmake_modules
    rbiz_autorace_msgs
    )
find_package(Qt5Widgets REQUIRED)

set(${PROJECT_NAME}_SRCS
    src/main.cpp
    src/main_window.cpp
    src/qnode.cpp
    )

set(${PROJECT_NAME}_HDRS
    include/${PROJECT_NAME}/main_window.hpp
    include/${PROJECT_NAME}/qnode.hpp
    )

set(${PROJECT_NAME}_UIS
    ui/main_window.ui
    )

set(${PROJECT_NAME}_RESOURCES
    resources/images.qrc
    )

set(${PROJECT_NAME}_INCLUDE_DIRECTORIES
    "${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_INCLUDE_DESTINATION}/${PROJECT_NAME}"
    )

if(NOT EXISTS ${${PROJECT_NAME}_INCLUDE_DIRECTORIES})
    file(MAKE_DIRECTORY ${${PROJECT_NAME}_INCLUDE_DIRECTORIES})
endif()

################################################
## Declare ROS messages, services and actions ##
################################################

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

###################################
## catkin specific configuration ##
###################################
catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS roscpp std_msgs rbiz_autorace_msgs
    )

qt5_wrap_cpp(${PROJECT_NAME}_MOCS     ${${PROJECT_NAME}_HDRS})
qt5_add_resources(${PROJECT_NAME}_RCC ${${PROJECT_NAME}_RESOURCES})

# ensure generated header files are being created in the devel space
set(_cmake_current_binary_dir "${CMAKE_CURRENT_BINARY_DIR}")
set(CMAKE_CURRENT_BINARY_DIR ${rbiz_autorace_monitor_INCLUDE_DIRECTORIES})

qt5_wrap_ui(${PROJECT_NAME}_UIS_H ${${PROJECT_NAME}_UIS})

set(CMAKE_CURRENT_BINARY_DIR "${_cmake_current_binary_dir}")

###########
## Build ##
###########
include_directories(
    include
    ${rbiz_autorace_monitor_INCLUDE_DIRECTORIES}
    ${catkin_INCLUDE_DIRS}
    )

add_executable(rbiz_autorace_monitor ${rbiz_autorace_monitor_SRCS} ${rbiz_autorace_monitor_MOCS} ${rbiz_autorace_monitor_UIS_H} ${rbiz_autorace_monitor_RCC})
add_dependencies(rbiz_autorace_monitor ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(rbiz_autorace_monitor Qt5::Widgets ${catkin_LIBRARIES})

#############
## Install ##
#############
install(TARGETS rbiz_autorace_monitor
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )

install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    )

install(DIRECTORY resources ui launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
    )

#############
## Testing ##
#############
