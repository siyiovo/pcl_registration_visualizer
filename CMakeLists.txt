cmake_minimum_required(VERSION 3.0.2)
project(pcl_registration_visualizer)

add_compile_options(-std=c++17)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
  roscpp
)

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io visualization registration)
find_package(OpenMP REQUIRED)
find_package(teaserpp REQUIRED)

include_directories(
	${catkin_INCLUDE_DIRS} 
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
  ${PYTHON_INCLUDE_DIRS}
  include
)

link_directories(
	include
	${PCL_LIBRARY_DIRS}
)

catkin_package(
  CATKIN_DEPENDS roscpp 
  DEPENDS EIGEN3 PCL
  INCLUDE_DIRS
)

add_executable(icp_pcl_node 
  src/example/icp_registration_pcl.cpp 
  src/pointcloud_matching_visualizer.cpp  
)

add_executable(ransac_icp_pcl_node 
  src/example/ransac_icp_registration_pcl.cpp 
  src/pointcloud_matching_visualizer.cpp  
)

target_link_libraries(icp_pcl_node 
  ${catkin_LIBRARIES}
  ${EIGEN3_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OpenMP_CXX_FLAGS}
)

target_link_libraries(ransac_icp_pcl_node  
  ${catkin_LIBRARIES}
  ${EIGEN3_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OpenMP_CXX_FLAGS}
)
