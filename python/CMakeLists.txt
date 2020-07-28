cmake_minimum_required(VERSION 3.10.2)
project(pymss)

set(CMAKE_CXX_STANDARD 14)

add_compile_options(-fPIC)
add_compile_options(-Wdeprecated-declarations)

set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR})

include(FindOpenMP)
if(OPENMP_FOUND)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()
find_package(Boost 1.65.0 COMPONENTS filesystem python3 numpy3)
find_package(DART REQUIRED COMPONENTS collision-bullet CONFIG)
find_package(PythonLibs REQUIRED)

link_directories(../core/)
include_directories(../core/)

file(GLOB srcs "*.h" "*.cpp")

include_directories(${PYTHON_INCLUDE_DIR})
include_directories(${Boost_INCLUDE_DIRS})
include_directories(${DART_INCLUDE_DIRS})

if(APPLE)
    set(CMAKE_SHARED_LIBRARY_SUFFIX ".so")
endif(APPLE)

add_library(pymss SHARED ${srcs})

if(OpenMP_FOUND)
target_link_libraries(pymss ${DART_LIBRARIES} ${Boost_LIBRARIES} ${PYTHON_LIBRARIES} mss OpenMP::OpenMP_CXX)
else()
target_link_libraries(pymss ${DART_LIBRARIES} ${Boost_LIBRARIES} ${PYTHON_LIBRARIES} mss)
endif()
set_target_properties(pymss PROPERTIES PREFIX "" )
