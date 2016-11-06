#cmake_minimum_required(VERSION 2.8)
cmake_minimum_required(VERSION 3.1.0 FATAL_ERROR)
project(vo)
find_package(OpenCV 3.0 REQUIRED)

# ===========================================================
# check and enable C++ support if available
# ===========================================================
set ( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11" ) 

# ===========================================================
# set the build type. Options are:
# 	None (CMAKE_C_FLAGS or CMAKE_CXX_FLAGS used)
#   Debug (CMAKE_C_FLAGS_DEBUG or CMAKE_CXX_FLAGS_DEBUG)
#	Release (CMAKE_C_FLAGS_RELEASE or CMAKE_CXX_FLAGS_RELEASE)
# ===========================================================
set(CMAKE_BUILD_TYPE Debug)

# set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR}/bin)

OPTION(BUILD_TESTS "Build tests" ON)

include_directories( include )
include_directories( ${OpenCV_INCLUDE_DIRS} )
	
# ==========================================================
# OpenGV library
# ==========================================================
set ( OpenGV_LINK_LIBS ${PROJECT_SOURCE_DIR}/include/libopengv.a )
#link_directories( ${PROJECT_SOURCE_DIR}/include)

# ==========================================================
# Eigen library
# ==========================================================
set (Eigen_INCLUDE_DIR /usr/local/include/eigen3)
include_directories( ${Eigen_INCLUDE_DIR} )


file(GLOB SOURCES 
	# "src/*.h"
    "src/*.cpp"
)

add_executable(vo ${SOURCES})
target_link_libraries(vo ${OpenCV_LIBS})
target_link_libraries( vo ${OpenGV_LINK_LIBS} )

IF (BUILD_TESTS)
	enable_testing()

	add_executable( test_triangulation test/test_triangulation.cpp src/triangulation.cpp )
	target_link_libraries( test_triangulation ${OpenCV_LIBS} )
	target_link_libraries( test_triangulation ${OpenGV_LINK_LIBS} )
	add_test(NAME test_triangulation
		WORKING_DIRECTORY ${EXECUTALBE_OUTPUT_PATH}
		COMMAND test_triangulation)

	add_executable( test_monocular test/test_monocular.cpp src/monocular_pose.cpp src/tool.cpp src/bucket_features.cpp src/feature_detector.cpp)
	target_link_libraries( test_monocular ${OpenCV_LIBS} )
	target_link_libraries( test_monocular ${OpenGV_LINK_LIBS} )
	add_test(NAME test_monocular
		WORKING_DIRECTORY ${EXECUTALBE_OUTPUT_PATH}
		COMMAND test_monocular)

	add_executable( test_stereo test/test_stereo.cpp src/stereo_pose.cpp src/tool.cpp src/bucket_features.cpp src/triangulation.cpp src/feature_detector.cpp )
	target_link_libraries( test_stereo ${OpenCV_LIBS} )
	target_link_libraries( test_stereo ${OpenGV_LINK_LIBS} )
	add_test(NAME test_stereo
		WORKING_DIRECTORY ${EXECUTALBE_OUTPUT_PATH}
		COMMAND test_stereo)
		
	add_executable( test_flann test/test_flann.cpp ) 
	target_link_libraries( test_flann ${OpenCV_LIBS} )
	add_test(NAME test_flann
		WORKING_DIRECTORY ${EXECUTALBE_OUTPUT_PATH}
		COMMAND test_flann)

	add_executable( test_opticalFlow test/test_opticalFlow.cpp src/tool.cpp src/feature_detector.cpp ) 
	target_link_libraries( test_opticalFlow ${OpenCV_LIBS} )
	add_test(NAME test_opticalFlow
		WORKING_DIRECTORY ${EXECUTALBE_OUTPUT_PATH}
		COMMAND test_opticalFlow)

ENDIF()

#target_link_libraries(vo ${OpenGV_})

