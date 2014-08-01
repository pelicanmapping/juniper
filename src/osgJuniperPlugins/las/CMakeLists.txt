INCLUDE_DIRECTORIES( ${LIBLAS_INCLUDE_DIR} ${Boost_INCLUDE_DIR} )
SET(TARGET_LIBRARIES_VARS
    LIBLAS_LIBRARY)

link_directories(${Boost_LIBRARY_DIRS}) 

SET(TARGET_H
)

SET(TARGET_SRC 
    ReaderWriter.cpp
)
SETUP_PLUGIN(las)

# to install public driver includes:
#SET(LIB_NAME xyz)
#SET(LIB_PUBLIC_HEADERS OSGOptions)
#INCLUDE(ModuleInstallOsgEarthDriverIncludes OPTIONAL)

