cmake_minimum_required(VERSION 3.0)
project(Algorithm LANGUAGES C CXX VERSION 0.1)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)

if (CMAKE_COMPILER_ISGNUCXX)
find_package(openmp)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fopenmp")
endif()
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Sophus REQUIRED)
find_package(gtest REQUIRED)
find_package(cgal REQUIRED)
#set(CMAKE_BUILD_TYPE RELEASE)

if (OPENMP_FOUND)
    message(status "OPENMP FOUND")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAG} ${OpenMP_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
    else()
    message("OpenMP Not Found")
endif()

include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${Sophus_INCLUDE_DIRS})
include_directories(/usr/local/include/)
include(${CGAL_USE_FILE})
link_directories(/usr/local/Cellar/cgal/4.14_1/lib)

add_executable(main main.cc IO/las.cc IO/las.hpp)
add_executable(test_main
        test_main.cc
        algorithm/delaunay_test.hpp
        algorithm/delaunay.cc
        algorithm/delaunay_test.hpp
        voronoi.hpp
        voronoi.cc
        voronoi_test.hpp
        geometry_algorithm.hpp
        geometry_algorithm_test.hpp
        kdtree_test.hpp
        kdtree_test.hpp
        utils/ThreadPool.cc
        utils/ThreadPool.hpp
        utils/thread_pool_unittest.hpp
        algorithm/unconstrained_minization.cc
        algorithm/unconstrained_minization.hpp
        algorithm/unconstrained_minization_unittest.hpp
        )

target_link_libraries(main ${OpenCV_LIBS} Eigen3::Eigen)
target_include_directories(test_main PUBLIC ${PROJECT_BINARY_DIR} ${PROJECT_SOURCE_DIR})
target_link_libraries(test_main ${OpenCV_LIBS} Eigen3::Eigen ${GTEST_BOTH_LIBRARIES} ${CGAL_LIBS} )
add_test(
        NAME test_main
        COMMAND test_main
)