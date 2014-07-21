SET(TARGET_H
)
SET(TARGET_SRC 
    Bundler.cpp
)
INCLUDE_DIRECTORIES(${OSGEARTH_INCLUDE_DIR} )
SETUP_PLUGIN(juniper_bundler)


# to install public driver includes:
#SET(LIB_NAME xyz)
#SET(LIB_PUBLIC_HEADERS OSGOptions)
#INCLUDE(ModuleInstallOsgEarthDriverIncludes OPTIONAL)

