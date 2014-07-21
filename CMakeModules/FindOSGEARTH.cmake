# Locate OSGEARTH
# This module defines
# OSGEARTH_LIBRARY
# OSGEARTH_INCLUDE_DIR, where to find the headers
#
# Created by Robert Osfield, adapted by GW

GET_FILENAME_COMPONENT(OSGEARTH_SIDE_BY_SIDE ${PROJECT_SOURCE_DIR} PATH)

FIND_PATH(OSGEARTH_INCLUDE_DIR osgEarth/TileSource
	PATHS
    ${OSGEARTH_DIR}
	${OSGEARTH_DIR}/src
	$ENV{OSGEARTH_DIR}
	$ENV{OSGEARTH_DIR}/src
	${OSGEARTH_SIDE_BY_SIDE}/osgEarth
	${OSGEARTH_SIDE_BY_SIDE}/osgEarth/src
	PATH_SUFFIXES
    /include/
)

FIND_LIBRARY(OSGEARTH_LIBRARY
    NAMES osgEarth
    PATHS
    ${OSGEARTH_DIR}
	${OSGEARTH_DIR}/lib/release
	$ENV{OSGEARTH_DIR}
	$ENV{OSGEARTH_DIR}/lib/release
	${OSGEARTH_SIDE_BY_SIDE}/osgEarth
	${OSGEARTH_SIDE_BY_SIDE}/osgEarth/lib/release
    PATH_SUFFIXES
        /lib/
        /lib64/
        /build/lib/
        /build/lib64/
        /Build/lib/
        /Build/lib64/
)

FIND_LIBRARY(OSGEARTH_LIBRARY_DEBUG
    NAMES osgEarthd
    PATHS
    ${OSGEARTH_DIR}
	${OSGEARTH_DIR}/lib/debug
	$ENV{OSGEARTH_DIR}
	$ENV{OSGEARTH_DIR}/lib/debug
	${OSGEARTH_SIDE_BY_SIDE}/osgEarth
	${OSGEARTH_SIDE_BY_SIDE}/osgEarth/lib/debug
    PATH_SUFFIXES
        /lib/
        /lib64/
        /build/lib/
        /build/lib64/
        /Build/lib/
        /Build/lib64/
)

FIND_LIBRARY(OSGEARTHUTIL_LIBRARY
    NAMES osgEarthUtil
    PATHS
    ${OSGEARTH_DIR}
	${OSGEARTH_DIR}/lib/release
	$ENV{OSGEARTH_DIR}
	$ENV{OSGEARTH_DIR}/lib/release
	${OSGEARTH_SIDE_BY_SIDE}/osgEarth
	${OSGEARTH_SIDE_BY_SIDE}/osgEarth/lib/release
    PATH_SUFFIXES
        /lib/
        /lib64/
        /build/lib/
        /build/lib64/
        /Build/lib/
        /Build/lib64/
)

FIND_LIBRARY(OSGEARTHUTIL_LIBRARY_DEBUG
    NAMES osgEarthUtild
    PATHS
    ${OSGEARTH_DIR}
	${OSGEARTH_DIR}/lib/release
	$ENV{OSGEARTH_DIR}
	$ENV{OSGEARTH_DIR}/lib/release
	${OSGEARTH_SIDE_BY_SIDE}/osgEarth
	${OSGEARTH_SIDE_BY_SIDE}/osgEarth/lib/release
    PATH_SUFFIXES
        /lib/
        /lib64/
        /build/lib/
        /build/lib64/
        /Build/lib/
        /Build/lib64/
)

FIND_LIBRARY(OSGEARTHSYMBOLOGY_LIBRARY
    NAMES osgEarthSymbology
    PATHS
    ${OSGEARTH_DIR}
	${OSGEARTH_DIR}/lib/release
	$ENV{OSGEARTH_DIR}
	$ENV{OSGEARTH_DIR}/lib/release
	${OSGEARTH_SIDE_BY_SIDE}/osgEarth
	${OSGEARTH_SIDE_BY_SIDE}/osgEarth/lib/release
    PATH_SUFFIXES
        /lib/
        /lib64/
        /build/lib/
        /build/lib64/
        /Build/lib/
        /Build/lib64/
)

FIND_LIBRARY(OSGEARTHSYMBOLOGY_LIBRARY_DEBUG
    NAMES osgEarthSymbologyd
    PATHS
    ${OSGEARTH_DIR}
	${OSGEARTH_DIR}/lib/release
	$ENV{OSGEARTH_DIR}
	$ENV{OSGEARTH_DIR}/lib/release
	${OSGEARTH_SIDE_BY_SIDE}/osgEarth
	${OSGEARTH_SIDE_BY_SIDE}/osgEarth/lib/release
    PATH_SUFFIXES
        /lib/
        /lib64/
        /build/lib/
        /build/lib64/
        /Build/lib/
        /Build/lib64/
)

FIND_LIBRARY(OSGEARTHFEATURES_LIBRARY
    NAMES osgEarthFeatures
    PATHS
    ${OSGEARTH_DIR}
	${OSGEARTH_DIR}/lib/release
	$ENV{OSGEARTH_DIR}
	$ENV{OSGEARTH_DIR}/lib/release
	${OSGEARTH_SIDE_BY_SIDE}/osgEarth
	${OSGEARTH_SIDE_BY_SIDE}/osgEarth/lib/release
    PATH_SUFFIXES
        /lib/
        /lib64/
        /build/lib/
        /build/lib64/
        /Build/lib/
        /Build/lib64/
)

FIND_LIBRARY(OSGEARTHFEATURES_LIBRARY_DEBUG
    NAMES osgEarthFeaturesd
    PATHS
    ${OSGEARTH_DIR}
	${OSGEARTH_DIR}/lib/release
	$ENV{OSGEARTH_DIR}
	$ENV{OSGEARTH_DIR}/lib/release
	${OSGEARTH_SIDE_BY_SIDE}/osgEarth
	${OSGEARTH_SIDE_BY_SIDE}/osgEarth/lib/release
    PATH_SUFFIXES
        /lib/
        /lib64/
        /build/lib/
        /build/lib64/
        /Build/lib/
        /Build/lib64/
)


FIND_LIBRARY(OSGEARTHANNOTATION_LIBRARY
    NAMES osgEarthAnnotation
    PATHS
    ${OSGEARTH_DIR}
	${OSGEARTH_DIR}/lib/release
	$ENV{OSGEARTH_DIR}
	$ENV{OSGEARTH_DIR}/lib/release
	${OSGEARTH_SIDE_BY_SIDE}/osgEarth
	${OSGEARTH_SIDE_BY_SIDE}/osgEarth/lib/release
    PATH_SUFFIXES
        /lib/
        /lib64/
        /build/lib/
        /build/lib64/
        /Build/lib/
        /Build/lib64/
)

FIND_LIBRARY(OSGEARTHANNOTATION_LIBRARY_DEBUG
    NAMES osgEarthAnnotationd
    PATHS
    ${OSGEARTH_DIR}
	${OSGEARTH_DIR}/lib/release
	$ENV{OSGEARTH_DIR}
	$ENV{OSGEARTH_DIR}/lib/release
	${OSGEARTH_SIDE_BY_SIDE}/osgEarth
	${OSGEARTH_SIDE_BY_SIDE}/osgEarth/lib/release
    PATH_SUFFIXES
        /lib/
        /lib64/
        /build/lib/
        /build/lib64/
        /Build/lib/
        /Build/lib64/
)
