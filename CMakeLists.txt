cmake_minimum_required(VERSION 3.0)
project(OPENCV_TEST VERSION 0.1)

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Sophus REQUIRED)
find_package(gtest REQUIRED)

include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${Sophus_INCLUDE_DIRS})
include_directories(/usr/local/include/)
include_directories(${GTEST_INCLUDE_DIRS})

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)
#include_directories(/usr/local/Cellar/opencv/4.1.2/include/opencv4)
#link_directories(/usr/local/Cellar/opencv/4.1.2/lib)
#link_libraries(opencv_core)

option(USE_MYMATH "USE MY MATH FUNCTION" ON)
add_subdirectory(${PROJECT_SOURCE_DIR}/googletest)


add_executable(main main.cc ImageProcess.hpp lei_group.hpp dense_mapping.hpp dense_mapping_slam.hpp camera_calib.hpp algorithm/delaunay.hpp algorithm/delaunay.cc algorithm/delaunay_test.hpp)
add_executable(test_main test_main.cc algorithm/delaunay_test.hpp algorithm/delaunay.cc algorithm/delaunay_test.hpp voronoi.hpp voronoi.cc voronoi_test.hpp)

target_link_libraries(main ${OpenCV_LIBS} Eigen3::Eigen)
target_include_directories(main PUBLIC ${PROJECT_BINARY_DIR} ${PROJECT_SOURCE_DIR})
target_link_libraries(test_main ${OpenCV_LIBS} Eigen3::Eigen ${GTEST_BOTH_LIBRARIES})