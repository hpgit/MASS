cmake_minimum_required(VERSION 3.10.2)
project(mss)

SET(CMAKE_BUILD_TYPE Release CACHE STRING
	"Choose the type of build, options are: None Debug Release RelWithDebInfo MinSizeRel.")

find_package(DART REQUIRED COMPONENTS collision-bullet CONFIG)
find_package(TinyXML REQUIRED)

include_directories(${DART_INCLUDE_DIRS})

file(GLOB srcs "*.h" "*.cpp")

add_library(mss ${srcs})
target_link_libraries(mss ${DART_LIBRARIES} ${TinyXML_LIBRARIES})
