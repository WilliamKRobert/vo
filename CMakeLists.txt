cmake_minimum_required(VERSION 3.1.0 FATAL_ERROR)

project(vo)

set( EIGEN_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/Thirdparty/eigen3 )
include_directories( ${EIGEN_INCLUDE_DIR} )

find_package(OpenCV 3.0 REQUIRED)
include_directories( ${OpenCV_INCLUDE_DIRS} )

find_package(Ceres REQUIRED)
include_directories( ${CERES_INCLUDE_DIRS} )

# check and enable C++ support if available
set ( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11" ) 

# set the build type. Options are:
#   None (CMAKE_C_FLAGS or CMAKE_CXX_FLAGS used)
#   Debug (CMAKE_C_FLAGS_DEBUG or CMAKE_CXX_FLAGS_DEBUG)
#   Release (CMAKE_C_FLAGS_RELEASE or CMAKE_CXX_FLAGS_RELEASE)
set(CMAKE_BUILD_TYPE Debug)

# set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR}/bin)
set(KITTI_PATH evaluation/kitti/devkit/cpp)

OPTION(BUILD_TESTS "Build tests" ON)
OPTION(BUILD_KITTI_EVALUATION "Build KITTI evaluation" ON)


include_directories( include )
include_directories( evaluation )
include_directories( ${KITTI_PATH} )
	
# OpenGV library
set ( OpenGV_LINK_LIBS ${PROJECT_SOURCE_DIR}/Thirdparty/OpenGV/libopengv.a )
#find_package(OpenGV REQUIRED)
#include_directories(${OPENGV_INCLUDE_DIRS})



file(GLOB SOURCES 
    "src/*.cpp"
)

add_executable(helloworld src/helloworld.cpp)
target_link_libraries(helloworld ${CERES_LIBRARIES})

#add_executable(vo ${SOURCES})
#target_link_libraries(vo ${OpenCV_LIBS})
#target_link_libraries( vo ${OpenGV_LINK_LIBS} )

IF (BUILD_TESTS)
	enable_testing()

	add_executable( test_triangulation test/test_triangulation.cpp src/triangulation.cpp src/tool.cpp )
	target_link_libraries( test_triangulation ${OpenCV_LIBS} )
	target_link_libraries( test_triangulation ${OpenGV_LINK_LIBS} )
	add_test(NAME test_triangulation
		WORKING_DIRECTORY ${EXECUTALBE_OUTPUT_PATH}
		COMMAND test_triangulation)

	add_executable( test_monocular test/test_monocular.cpp src/tool.cpp src/feature_detector.cpp src/feature_tracker.cpp src/feature_matcher.cpp src/motion_from_image.cpp )
	target_link_libraries( test_monocular ${OpenCV_LIBS} )
	target_link_libraries( test_monocular ${OpenGV_LINK_LIBS} )
	add_test(NAME test_monocular
		WORKING_DIRECTORY ${EXECUTALBE_OUTPUT_PATH}
		COMMAND test_monocular)

	add_executable( test_stereo_pnp test/test_stereo_pnp.cpp src/motion_from_structure_and_image.cpp src/tool.cpp src/triangulation.cpp src/feature_detector.cpp src/feature_tracker.cpp )
	target_link_libraries( test_stereo_pnp ${OpenCV_LIBS} )
	target_link_libraries( test_stereo_pnp ${OpenGV_LINK_LIBS} )
	add_test(NAME test_stereo
		WORKING_DIRECTORY ${EXECUTALBE_OUTPUT_PATH}
		COMMAND test_stereo_pnp)
		
	add_executable( test_flann test/test_flann.cpp ) 
	target_link_libraries( test_flann ${OpenCV_LIBS} )
	add_test(NAME test_flann
		WORKING_DIRECTORY ${EXECUTALBE_OUTPUT_PATH}
		COMMAND test_flann)

	add_executable( test_tracker test/test_tracker.cpp src/feature_detector.cpp src/feature_tracker.cpp ) 
	target_link_libraries( test_tracker ${OpenCV_LIBS} )
	add_test(NAME test_tracker
		WORKING_DIRECTORY ${EXECUTALBE_OUTPUT_PATH}
		COMMAND test_tracker)

	add_executable( test_matcher test/test_matcher.cpp src/feature_matcher.cpp ) 
	target_link_libraries( test_matcher ${OpenCV_LIBS} )
	add_test(NAME test_matcher
		WORKING_DIRECTORY ${EXECUTALBE_OUTPUT_PATH}
		COMMAND test_matcher)

	add_executable( test_bundle_adjustment test/test_bundle_adjustment.cpp src/motion_from_structure_and_image.cpp src/tool.cpp src/triangulation.cpp src/feature_detector.cpp src/feature_tracker.cpp src/optimizer.cpp src/map_association.cpp ) 
	target_link_libraries( test_bundle_adjustment ${OpenCV_LIBS} ${CERES_LIBRARIES})
	add_test(NAME test_bundle_adjustment
		WORKING_DIRECTORY ${EXECUTALBE_OUTPUT_PATH}
		COMMAND test_bundle_adjustment)



ENDIF()

# ----------------------------------------------------------
# Build KITTI dataset evaluation test
# ----------------------------------------------------------
IF (BUILD_KITTI_EVALUATION)
	add_executable( kitti_evaluation ${KITTI_PATH}/evaluate_odometry.cpp ${KITTI_PATH}/matrix.cpp )

ENDIF()

