#cmake_minimum_required(VERSION 2.8)
cmake_minimum_required(VERSION 3.1.0 FATAL_ERROR)
project(vo)
find_package(OpenCV 3.0 REQUIRED)
#find_package(opengv REQUIRED)

include_directories( ${OpenCV_INCLUDE_DIRS} )
include_directories(include /usr/local/include/opengv)
include_directories(include /usr/local/include/eigen3)


file(GLOB vo
    "src/*.h"
    "src/*.cpp"
)
add_executable(vo ${vo})
target_link_libraries(vo ${OpenCV_LIBS})


#target_link_libraries(vo ${OpenGV_LIBS})
target_compile_features(vo PRIVATE cxx_range_for)

