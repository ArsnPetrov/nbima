cmake_minimum_required(VERSION 3.5 FATAL_ERROR)

# Project name
project(nbima)

add_executable(nbima main.cpp)
target_sources(nbima PRIVATE GUI.cxx)

target_link_libraries(nbima fftw3f)
target_link_libraries(nbima rtlsdr)
target_link_libraries(nbima fltk)
target_link_libraries(nbima SDL2)
target_link_libraries(nbima pthread)
if (UNIX)
target_link_libraries(nbima X11)
endif (UNIX)