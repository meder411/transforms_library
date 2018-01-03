cmake_minimum_required(VERSION 2.8)
project(TransformsLibrary)

# C++11 compatibility
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
else()
	message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

find_package(Eigen3 REQUIRED)
find_package(Boost COMPONENTS unit_test_framework)

if(Boost_FOUND)
	add_subdirectory(test)
	enable_testing()
endif()