
PROJECT(OSGJUNIPER_PLUGINS_MASTER)


SET(CMAKE_SHARED_MODULE_PREFIX ${OSGJUNIPER_PLUGIN_PREFIX})

IF(MSVC80)
  IF(NOT OSGJUNIPER_MSVC_GENERATE_PLUGINS_AND_WRAPPERS_MANIFESTS)
    SET(CMAKE_MODULE_LINKER_FLAGS "${CMAKE_MODULE_LINKER_FLAGS} /MANIFEST:NO")
  ENDIF(NOT OSGJUNIPER_MSVC_GENERATE_PLUGINS_AND_WRAPPERS_MANIFESTS)
ENDIF(MSVC80)

SET(TARGET_DEFAULT_PREFIX "osgdb_")
SET(TARGET_DEFAULT_LABEL_PREFIX "Plugin")

#OpenThreads, osg, osgDB and osgUtil are included elsewhere.
SET(TARGET_COMMON_LIBRARIES
    osgJuniper
)

############################################################
#
#  NodeKit/Psudo loader plugins
#
# For each plugin, add the line
#   "ADD_SUBDIRECTORY(pluginname)"
#

ADD_SUBDIRECTORY(smf)
ADD_SUBDIRECTORY(velodyne)
ADD_SUBDIRECTORY(pdal_points_plugin)
ADD_SUBDIRECTORY(pdal)
ADD_SUBDIRECTORY(lastile)