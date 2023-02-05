cmake_minimum_required(VERSION 2.6 FATAL_ERROR)
project( Hello_Nanoflann )
find_package(PCL 1.3 REQUIRED COMPONENTS common io visualization)
find_package(Eigen3)
find_package ( OpenCV REQUIRED )
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
add_executable(icp_tricp src/icp_tricp.cpp)
target_link_libraries(icp_tricp ${PCL_LIBRARIES})
target_link_libraries ( icp_tricp ${OpenCV_LIBS} )
target_include_directories(icp_tricp PRIVATE ${EIGEN3_INCLUDE_DIR})


#find_package(nanoflann)

#target_link_libraries(HelloWorld nanoflann::nanoflann)

# READ MORE:
#   https://github.com/jlblancoc/nanoflann#11-obtaining-the-code
#   https://github.com/jlblancoc/nanoflann#17-use-in-your-project-via-cmake
