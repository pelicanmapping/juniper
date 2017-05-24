PROJECT(OSGJUNIPER_APPLICATIONS)

SET(OPENSCENEGRAPH_APPLICATION_DIR ${PROJECT_SOURCE_DIR})

#OpenThreads, osg, osgDB and osgUtil are included elsewhere.
SET(TARGET_COMMON_LIBRARIES
    osgJuniper
    osgJuniperMap
)

SET(TARGET_DEFAULT_PREFIX "application_")
SET(TARGET_DEFAULT_LABEL_PREFIX "Sample")

ADD_SUBDIRECTORY(osgjuniper_pointcloud)
ADD_SUBDIRECTORY(osgjuniper_pointcloudtools)
ADD_SUBDIRECTORY(osgjuniper_bundlerviewer)
ADD_SUBDIRECTORY(osgjuniper_optimize)
ADD_SUBDIRECTORY(osgjuniper_ibrdemo)
ADD_SUBDIRECTORY(osgjuniper_nodesequence)
ADD_SUBDIRECTORY(osgjuniper_qsplatmake)
ADD_SUBDIRECTORY(osgjuniper_makepointcloud)
ADD_SUBDIRECTORY(osgjuniper_spt)
ADD_SUBDIRECTORY(osgjuniper_tunnelviewer)
ADD_SUBDIRECTORY(osgjuniper_velodyne)
ADD_SUBDIRECTORY(osgjuniper_movingpointcloud)
ADD_SUBDIRECTORY(osgjuniper_lastile)
ADD_SUBDIRECTORY(osgjuniper_lasreproject)
ADD_SUBDIRECTORY(airwolf)
