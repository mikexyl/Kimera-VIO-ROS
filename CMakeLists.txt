cmake_minimum_required(VERSION 2.8.3)
project(kimera_vio_ros)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(catkin_simple REQUIRED)

# Eigen is header-only but some dependent targets export the imported Eigen target
# as an INTERFACE link. Ensure Eigen3 target is defined in this configuration so
# linking doesn't turn 'Eigen3::Eigen' into a raw '-lEigen3::Eigen'.
find_package(Eigen3 REQUIRED)

find_package(rerun_sdk REQUIRED)
find_package(aria_common REQUIRED)
find_package(aria_viz REQUIRED)
find_package(spdlog REQUIRED)
find_package(yaml-cpp REQUIRED)

### System Dependencies
find_package(Boost REQUIRED COMPONENTS
  serialization system filesystem thread date_time regex timer chrono)
if(NOT TARGET Boost::boost)
  add_library(Boost::boost INTERFACE IMPORTED)
  set_target_properties(Boost::boost PROPERTIES
  INTERFACE_LINK_LIBRARIES "${Boost_LIBRARIES}"
  INTERFACE_INCLUDE_DIRECTORIES "${Boost_INCLUDE_DIRS}")
endif()

catkin_simple()

cs_add_library(${PROJECT_NAME}
  # Include files are just here for IDEs
  include/kimera_vio_ros/KimeraVioRos.h
  include/kimera_vio_ros/RosBagDataProvider.h
  include/kimera_vio_ros/RosDataProviderInterface.h
  include/kimera_vio_ros/RosDisplay.h
  include/kimera_vio_ros/RosOnlineDataProvider.h
  include/kimera_vio_ros/RosPublishers.h
  include/kimera_vio_ros/RosVisualizer.h
  include/kimera_vio_ros/RerunVisualizer.h
  include/kimera_vio_ros/utils/UtilsRos.h
  include/kimera_vio_ros/utils/CsvPublisher.h
  src/KimeraVioRos.cpp
  src/RosDataProviderInterface.cpp
  src/RosBagDataProvider.cpp
  src/RosOnlineDataProvider.cpp
  src/RosDisplay.cpp
  src/RosVisualizer.cpp
  src/RerunVisualizer.cpp
  src/RosLoopClosureVisualizer.cpp
  src/LcdRegistrationServer.cpp
  src/utils/UtilsRos.cpp
  src/utils/CsvPublisher.cpp
)
target_link_libraries(${PROJECT_NAME} kimera_vio spdlog::spdlog yaml-cpp rerun_sdk aria_common aria_viz)

cs_add_executable(${PROJECT_NAME}_node
  src/KimeraVioRosNode.cpp
)
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME} )

cs_add_executable(csv_publisher_node
  src/utils/CsvPublisherNode.cpp
)
target_link_libraries(csv_publisher_node ${PROJECT_NAME})

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(testKimeraVioRos
    test/testKimeraVioRos.cpp
  )
  target_link_libraries(testKimeraVioRos ${PROJECT_NAME})
else()
  message(STATUS "Skipping test target 'testKimeraVioRos' because CATKIN_ENABLE_TESTING is OFF")
endif()

cs_install()

cs_export()
