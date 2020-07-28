cmake_minimum_required(VERSION 3.10.2)
project(render_fltk)

set(CMAKE_CXX_STANDARD 14)

link_directories(../core/)
include_directories(../core/)

set(OpenGL_GL_PREFERENCE GLVND)

find_package(DART REQUIRED COMPONENTS collision-bullet CONFIG)
find_package(FLTK REQUIRED)
find_package(OpenGL REQUIRED)
find_package(GLUT REQUIRED)

find_package(PythonLibs REQUIRED)
find_package(Boost 1.65.0 COMPONENTS filesystem python3 numpy3)

include_directories(${DART_INCLUDE_DIRS})
include_directories(${OPENGL_INCLUDE_DIR})
include_directories(${GLUT_INCLUDE_DIR})
include_directories(${PYTHON_INCLUDE_DIR})
include_directories(${Boost_INCLUDE_DIRS})
include_directories(${FLTK_INCLUDE_DIRS})

if(APPLE)
    add_definitions(-DGL_SILENCE_DEPRECATION)
endif(APPLE)

file(GLOB srcs "*.h" "*.cpp")
add_executable(render ${srcs})
target_link_libraries(render ${DART_LIBRARIES} ${Boost_LIBRARIES} ${PYTHON_LIBRARIES} OpenGL::GL OpenGL::GLU ${GLUT_LIBRARIES} ${FLTK_LIBRARIES} mss)
