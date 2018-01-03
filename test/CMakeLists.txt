# FOR UNIT TESTS
#######################
# ENABLE UNIT TESTING
#######################
enable_testing()
add_definitions(-DBOOST_TEST_DYN_LINK) # Necessary to automatically generate main() in all tests

######################
# CREATE EXECUTABLES
######################
add_executable(math_util_test math_util_test.cpp)
add_executable(transforms_test transforms_test.cpp)

#######################
# INCLUDE DIRECTORIES
#######################
target_include_directories(
	math_util_test PUBLIC
	${EIGEN3_INCLUDE_DIRS}
)

target_include_directories(
	transforms_test PUBLIC
	${EIGEN3_INCLUDE_DIRS}
)

#######################
# LINK LIBRARIES
#######################
target_link_libraries(
	math_util_test
	${Boost_LIBRARIES}
)

target_link_libraries(
	transforms_test
	${Boost_LIBRARIES}
)


####################
# CREATE THE TESTS
####################
add_test(NAME math_util_test COMMAND math_util_test)
add_test(NAME transforms_test COMMAND transforms_test)
