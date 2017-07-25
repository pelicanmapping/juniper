PROJECT(OSGJUNIPER_APPLICATIONS)

SET(OPENSCENEGRAPH_APPLICATION_DIR ${PROJECT_SOURCE_DIR})

#OpenThreads, osg, osgDB and osgUtil are included elsewhere.
SET(TARGET_COMMON_LIBRARIES
    osgJuniper
)

SET(TARGET_DEFAULT_PREFIX "application_")
SET(TARGET_DEFAULT_LABEL_PREFIX "Sample")

ADD_SUBDIRECTORY(osgjuniper_pointcloudtools)
ADD_SUBDIRECTORY(osgjuniper_lastile)
ADD_SUBDIRECTORY(osgjuniper_lastile2)
ADD_SUBDIRECTORY(osgjuniper_lassplit)
