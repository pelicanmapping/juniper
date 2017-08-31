/* -*-c++-*- */
/* osgJuniper - Large Dataset Visualization Toolkit for OpenSceneGraph
* Copyright 2010-2017 Pelican Mapping
* Pelican Mapping CONFIDENTIAL
* Copyright (c) 2010-2017 [Pelican Mapping], All Rights Reserved.
*
* NOTICE:  All information contained herein is, and remains the property of Pelican Mapping. The intellectual and technical concepts contained
* herein are proprietary to Pelican Mapping and may be covered by U.S. and Foreign Patents, patents in process, and are protected by trade secret or copyright law.
* Dissemination of this information or reproduction of this material is strictly forbidden unless prior written permission is obtained
* from Pelican Mapping.  Access to the source code contained herein is hereby forbidden to anyone except current Pelican Mapping employees, managers or contractors who have executed
* Confidentiality and Non-disclosure agreements explicitly covering such access.
*
* The copyright notice above does not evidence any actual or intended publication or disclosure  of  this source code, which includes
* information that is confidential and/or proprietary, and is a trade secret, of Pelican Mapping.   ANY REPRODUCTION, MODIFICATION, DISTRIBUTION, PUBLIC  PERFORMANCE,
* OR PUBLIC DISPLAY OF OR THROUGH USE  OF THIS  SOURCE CODE  WITHOUT  THE EXPRESS WRITTEN CONSENT OF PELICAN MAPPING IS STRICTLY PROHIBITED, AND IN VIOLATION OF APPLICABLE
* LAWS AND INTERNATIONAL TREATIES.  THE RECEIPT OR POSSESSION OF  THIS SOURCE CODE AND/OR RELATED INFORMATION DOES NOT CONVEY OR IMPLY ANY RIGHTS
* TO REPRODUCE, DISCLOSE OR DISTRIBUTE ITS CONTENTS, OR TO MANUFACTURE, USE, OR SELL ANYTHING THAT IT  MAY DESCRIBE, IN WHOLE OR IN PART.
*/
#include <osgJuniper/Version>
#include <string>
#include <stdio.h>

#if (OSGJUNIPER_DEVEL_VERSION > 0)
#    define isDevelopmentVersion " DEVELOPMENT "
#else
#    define isDevelopmentVersion " "
#endif

extern "C" {

const char* osgJuniperGetVersion()
{
    static char osgjuniper_version[256];
    static int osgjuniper_version_init = 1;
    if (osgjuniper_version_init)
    {
        if (OSGJUNIPER_RC_VERSION == 0 )
        {
            sprintf(osgjuniper_version,"%d.%d.%d%s",
                OSGJUNIPER_MAJOR_VERSION,
                OSGJUNIPER_MINOR_VERSION,
                OSGJUNIPER_PATCH_VERSION,
                isDevelopmentVersion);
        }
        else
        {
            sprintf(osgjuniper_version,"%d.%d.%d RC%d%s",
                OSGJUNIPER_MAJOR_VERSION,
                OSGJUNIPER_MINOR_VERSION,
                OSGJUNIPER_PATCH_VERSION,
                OSGJUNIPER_RC_VERSION,
                isDevelopmentVersion );
        }

        osgjuniper_version_init = 0;
    }
    
    return osgjuniper_version;
}

const char* osgJuniperGetSOVersion()
{
    static char osgjuniper_soversion[32];
    static int osgjuniper_soversion_init = 1;
    if (osgjuniper_soversion_init)
    {
        sprintf(osgjuniper_soversion,"%d",OSGJUNIPER_SOVERSION);
        osgjuniper_soversion_init = 0;
    }
    
    return osgjuniper_soversion;
}

const char* osgJuniperGetLibraryName()
{
    return "osgJuniper Library";
}

}
