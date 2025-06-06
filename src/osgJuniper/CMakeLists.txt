IF   (DYNAMIC_OSGJUNIPER)
    ADD_DEFINITIONS(-DOSGJUNIPER_LIBRARY)
ELSE (DYNAMIC_OSGJUNIPER)
    ADD_DEFINITIONS(-DOSGJUNIPER_LIBRARY_STATIC)
ENDIF(DYNAMIC_OSGJUNIPER)

OPTION(CURL_IS_STATIC "on if curl is a static lib " ON)
MARK_AS_ADVANCED(CURL_IS_STATIC)

IF(WIN32)
    SET(CMAKE_SHARED_LINKER_FLAGS_DEBUG "${CMAKE_SHARED_LINKER_FLAGS_DEBUG} /NODEFAULTLIB:MSVCRT")
    IF(CURL_IS_STATIC)
        ADD_DEFINITIONS(-DCURL_STATICLIB)
        SET(TARGET_EXTERNAL_LIBRARIES ws2_32 winmm wldap32)
    ENDIF(CURL_IS_STATIC)
ENDIF(WIN32)

SET(LIB_NAME osgJuniper)

SET(HEADERS
    Common
    IBR
    IBRCameraNode
    IBRManipulator
    DirectoryPointTileStore
    FilePointTileStore
    NodeSequence
    Utils
    LASTile
    Octree
    OctreeId
    PCap
    PDALUtils
    Point
    PointReaderWriter
    PointCloud
    PointCloudTools
    PointTileStore
    Splitter
    StreamingNode
    TilesetInfo
    Velodyne
    Version
)

SET(SOURCES
    IBR.cpp
    IBRCameraNode.cpp
    IBRManipulator.cpp
    DirectoryPointTileStore.cpp
    FilePointTileStore.cpp
    NodeSequence.cpp
    Utils.cpp
    LASTile.cpp
    Octree.cpp
    OctreeId.cpp
    PCap.cpp
    PDALUtils.cpp
    Point.cpp
    PointReaderWriter.cpp
    PointCloud.cpp
    PointCloudTools.cpp
    PointTileStore.cpp
    Splitter.cpp
    StreamingNode.cpp
    TilesetInfo.cpp
    Velodyne.cpp
    Version.cpp
)

SET(TARGET_LIBRARIES_VARS
    OSG_LIBRARY OSGUTIL_LIBRARY OSGSIM_LIBRARY OSGTERRAIN_LIBRARY OSGDB_LIBRARY OSGFX_LIBRARY OSGVIEWER_LIBRARY OSGTEXT_LIBRARY OSGGA_LIBRARY OPENTHREADS_LIBRARY OSGEARTH_LIBRARY PDAL_LIBRARIES)

IF (ROCKSDB_FOUND)
    LIST(APPEND HEADERS RocksDBPointTileStore)
    LIST(APPEND SOURCES RocksDBPointTileStore.cpp)
    ADD_DEFINITIONS(-DHAVE_ROCKSDB)
    set (CMAKE_CXX_STANDARD 11)

    INCLUDE_DIRECTORIES( ${ROCKSDB_INCLUDE_DIR} )
    LIST(APPEND TARGET_LIBRARIES_VARS ROCKSDB_LIBRARY)

    IF(WIN32)
        SET(TARGET_EXTERNAL_LIBRARIES
        ws2_32
        winmm
        rpcrt4
        cabinet
        D:/dev/snappy-visual-cpp/x64/Release/snappy64.lib
        D:/dev/lz4-1.7.5/visual/VS2010/bin/x64_Release/liblz4_static.lib)
    ENDIF()

ENDIF()

SET(HEADER_PATH ${OSGJUNIPER_SOURCE_DIR}/include/${LIB_NAME})
SET(LIB_PUBLIC_HEADERS
    ${HEADERS}
)

ADD_LIBRARY(${LIB_NAME} SHARED
    ${LIB_PUBLIC_HEADERS}
    ${SOURCES}
)
INCLUDE_DIRECTORIES(${OSG_INCLUDE_DIR} ${OSGEARTH_INCLUDE_DIR} ${PDAL_INCLUDE_DIRS})

IF (WIN32)
  LINK_EXTERNAL(${LIB_NAME} ${TARGET_EXTERNAL_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT} ${MATH_LIBRARY} )
ELSE(WIN32)
  LINK_EXTERNAL(${LIB_NAME} ${TARGET_EXTERNAL_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT} ${MATH_LIBRARY} )
ENDIF(WIN32)

LINK_WITH_VARIABLES(${LIB_NAME} ${TARGET_LIBRARIES_VARS})
LINK_CORELIB_DEFAULT(${LIB_NAME} ${CMAKE_THREAD_LIBS_INIT} ${MATH_LIBRARY})

INCLUDE(ModuleInstall OPTIONAL)
