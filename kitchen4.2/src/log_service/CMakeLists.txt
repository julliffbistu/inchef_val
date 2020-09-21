cmake_minimum_required(VERSION 2.8.3)
project(log_service)

set(CMAKE_CXX_FLAGS "-std=c++11 -pthread ${CMAKE_CXX_FLAGS}")
set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/../../tools/cmake CACHE INTERNAL "" FORCE)

find_package(catkin REQUIRED)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES rlog
)

include_directories(
    include
    ../../lib/include
)

link_directories(${CATKIN_DEVEL_PREFIX}/../../lib ../../lib)

add_executable(log_service
  src/main.cpp
)

target_link_libraries(${PROJECT_NAME} rlog  ${catkin_LIBRARIES})

install(TARGETS ${PROJECT_NAME}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
