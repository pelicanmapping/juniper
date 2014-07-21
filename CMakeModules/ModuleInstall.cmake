# INSTALL and SOURCE_GROUP commands for OSG/OT/Producer Modules

# Required Vars:
# ${LIB_NAME}
# ${LIB_PUBLIC_HEADERS}

SET(INSTALL_INCDIR include)
SET(INSTALL_BINDIR bin)
IF(WIN32)
    SET(INSTALL_LIBDIR bin)
    SET(INSTALL_ARCHIVEDIR lib)
ELSE(WIN32)
    SET(INSTALL_LIBDIR lib${LIB_POSTFIX})
    SET(INSTALL_ARCHIVEDIR lib${LIB_POSTFIX})
ENDIF(WIN32)

SET(HEADERS_GROUP "Header Files")

SOURCE_GROUP(
    ${HEADERS_GROUP}
    FILES ${LIB_PUBLIC_HEADERS}
)


INSTALL(
    TARGETS ${LIB_NAME}
    RUNTIME DESTINATION ${INSTALL_BINDIR}
    LIBRARY DESTINATION ${INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${INSTALL_ARCHIVEDIR}
)
# FIXME: Do not run for OS X framework
INSTALL(
    FILES        ${LIB_PUBLIC_HEADERS}
    DESTINATION ${INSTALL_INCDIR}/${LIB_NAME}
)

#Install the library to the OSG_DIR as well
IF(OSGJUNIPER_INSTALL_TO_OSG_DIR AND OSG_DIR)
    INSTALL(
	    TARGETS ${LIB_NAME}
	    RUNTIME DESTINATION ${OSG_DIR}/bin
    )
ENDIF(OSGJUNIPER_INSTALL_TO_OSG_DIR AND OSG_DIR)
