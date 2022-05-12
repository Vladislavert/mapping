
cmake_minimum_required(VERSION 3.12)
set(CMAKE_CXX_STANDARD 17)
project(mapping)

set(COMMON_FLAGS "-Wall -Wextra -Werror")
set(CMAKE_CXX_FLAGS "${COMMON_FLAGS}")

find_package (Eigen3  REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
)

include_directories(
  inc/
  src/
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
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
