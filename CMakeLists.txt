#cmake_minimum_required(VERSION 2.8)
cmake_minimum_required(VERSION 3.1.0 FATAL_ERROR)
project(vo)
find_package(OpenCV 3.0 REQUIRED)

# ===========================================================
# check and enable C++ support if available
# ===========================================================
set ( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11" ) 

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
#target_link_libraries(vo ${OpenGV_})

