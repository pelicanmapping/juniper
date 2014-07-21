INCLUDE_DIRECTORIES(${OSG_INCLUDE_DIRS} )

SET(TARGET_LIBRARIES_VARS OSG_LIBRARY OSGDB_LIBRARY OSGUTIL_LIBRARY OSGVIEWER_LIBRARY OPENTHREADS_LIBRARY)

SET(TARGET_SRC 
    qsplat_colorquant.cpp
    qsplat_make_from_mesh.cpp
    qsplat_make_main.cpp
    qsplat_make_qtree_v11.cpp
    qsplat_normquant.cpp
    qsplat_spherequant.cpp
)

#### end var setup  ###
SETUP_APPLICATION(osgjuniper_qsplatmake)