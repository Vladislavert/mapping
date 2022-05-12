
cmake_minimum_required(VERSION 3.12)
set(CMAKE_CXX_STANDARD 17)
project(mapping)
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -fopenmp -lstdc++fs")
# link_directories("/usr/lib/" "/usr/include" )


# set(COMMON_FLAGS "-Wall -Wextra -Werror")
set(CMAKE_CXX_FLAGS "${COMMON_FLAGS}")

find_package (Eigen3  REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
)


# catkin_package(
#   INCLUDE_DIRS include
#   LIBRARIES pathfinder 
#   CATKIN_DEPENDS roscpp rospy std_msgs

# )

include_directories(
  inc/
  src/
  #PATHFINDER_INCLUDE_PATH
  # ${CMAKE_CURRENT_SOURCE_DIR}/include/pathfinder
  # ${VIS_NAV_LIB_INCLUDE_DIR}
  # ${CONTROL_SYS_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  # ${OpenCV_INCLUDE_DIRS}
  # ${CMAKE_CURRENT_SOURCE_DIR}
)

set(CPP_FILES
	src/mapping.cpp
	src/math.cpp
	src/convert.cpp
	src/filters.cpp
)


add_library(cpp_files ${CPP_FILES})

add_executable(main main.cpp)


target_link_libraries(main ${catkin_LIBRARIES} cpp_files)

