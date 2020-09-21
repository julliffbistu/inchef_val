cmake_minimum_required(VERSION 2.8.3)
project(rlog)

set(CMAKE_CXX_FLAGS "-std=c++11 -pthread ${CMAKE_CXX_FLAGS}")
set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/../tools/cmake CACHE INTERNAL "" FORCE)

find_package(catkin REQUIRED)
find_package(Boost 1.54.0 COMPONENTS system thread coroutine context REQUIRED)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES rlog
)

include_directories(
    include
    include/log4cpp
)

add_library(rlog SHARED
  log/rlog.cpp
  log4cpp/src/Appender.cpp
  log4cpp/src/AppenderSkeleton.cpp
  log4cpp/src/AppendersFactory.cpp
  log4cpp/src/BufferingAppender.cpp
  log4cpp/src/FactoryParams.cpp
  log4cpp/src/LayoutsFactory.cpp
  log4cpp/src/LevelEvaluator.cpp
  log4cpp/src/Localtime.cpp
  log4cpp/src/PassThroughLayout.cpp
  log4cpp/src/TriggeringEventEvaluatorFactory.cpp
  log4cpp/src/LayoutAppender.cpp
  log4cpp/src/FileAppender.cpp
  log4cpp/src/DailyRollingFileAppender.cpp
  log4cpp/src/RollingFileAppender.cpp
  log4cpp/src/FixedContextCategory.cpp
  log4cpp/src/OstreamAppender.cpp
  log4cpp/src/ShmAppender.cpp
  log4cpp/src/StringQueueAppender.cpp
  log4cpp/src/PatternLayout.cpp
  log4cpp/src/Category.cpp
  log4cpp/src/CategoryStream.cpp
  log4cpp/src/HierarchyMaintainer.cpp
  log4cpp/src/Configurator.cpp
  log4cpp/src/PropertyConfigurator.cpp
  log4cpp/src/PropertyConfiguratorImpl.cpp
  log4cpp/src/LoggingEvent.cpp
  log4cpp/src/Priority.cpp
  log4cpp/src/NDC.cpp
  log4cpp/src/Filter.cpp
  log4cpp/src/TimeStamp.cpp
  log4cpp/src/StringUtil.cpp
  log4cpp/src/Properties.cpp
  log4cpp/src/DummyThreads.cpp
  log4cpp/src/MSThreads.cpp
  log4cpp/src/OmniThreads.cpp
  log4cpp/src/PThreads.cpp
  log4cpp/src/PortabilityImpl.cpp
  log4cpp/src/BasicLayout.cpp
)

target_link_libraries(rlog
    ${catkin_LIBRARIES}
)

install(PROGRAMS
  comm/proxy_client.py
  DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}/comm
)

install(PROGRAMS
  adapter/actuator.py
  DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}/adapter
)

install(PROGRAMS
  log/rlog.py
  DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}/log
)

install(TARGETS rlog
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
