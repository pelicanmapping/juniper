cmake_minimum_required(VERSION 2.8.12)
project(POINTS_PDAL_PLUGIN)

find_package(PDAL 1.5.0 REQUIRED CONFIG)

add_library(libpdal_plugin_reader_points SHARED PointsReader.hpp PointsReader.cpp)
target_link_libraries(libpdal_plugin_reader_points PRIVATE ${PDAL_LIBRARIES} osgJuniper)
target_include_directories(libpdal_plugin_reader_points PRIVATE
    ${PDAL_INCLUDE_DIRS},
    ${JUNIPER_INCLUDE_DIRS},
    /Users/chambbj/loki/pdal/repo/readers)

INSTALL(TARGETS libpdal_plugin_reader_points RUNTIME DESTINATION bin LIBRARY DESTINATION lib )