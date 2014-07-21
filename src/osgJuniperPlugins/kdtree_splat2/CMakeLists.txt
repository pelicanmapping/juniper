SET(TARGET_H
    QKdTree
)
SET(TARGET_SRC 
    QKdTree.cpp
    KdTreeSplatRenderer.cpp
)
SETUP_PLUGIN(juniper_kdtree_splat2)


# to install public driver includes:
#SET(LIB_NAME xyz)
#SET(LIB_PUBLIC_HEADERS OSGOptions)
#INCLUDE(ModuleInstallOsgEarthDriverIncludes OPTIONAL)

