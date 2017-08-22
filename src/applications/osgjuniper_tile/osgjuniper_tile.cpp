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
#include <osgJuniper/Octree>
#include <osgEarth/StringUtils>
#include <osgEarth/FileUtils>
#include <osg/BoundingBox>
#include <osgDB/FileUtils>
#include <osgDB/FileNameUtils>
#include <osg/ArgumentParser>
#include <iostream>
#include <osgJuniper/Utils>
#include <osgJuniper/LASTile>

using namespace osgJuniper;

int main(int argc, char** argv)
{
    osg::Timer_t startTime = osg::Timer::instance()->tick();
    std::vector< std::string > filenames;

    osg::ArgumentParser arguments(&argc,argv);

    std::string directory;
    arguments.read("--directory", directory);

    if (!directory.empty())
    {
        //Load the filenames from a directory
        std::vector< std::string > contents = osgJuniper::Utils::getFilesFromDirectory(directory, "laz");
        for (unsigned int i = 0; i < contents.size(); i++)
        {
            filenames.push_back(contents[i]);
        }
        contents = osgJuniper::Utils::getFilesFromDirectory(directory, "las");
        for (unsigned int i = 0; i < contents.size(); i++)
        {
            filenames.push_back(contents[i]);
        }
    }


    unsigned int targetNumPoints = 0;
    arguments.read("--target", targetNumPoints);

    unsigned int innerLevel = 8;
    arguments.read("--innerLevel", innerLevel);

    unsigned int maxLevel = 8;
    arguments.read("--maxLevel", maxLevel);

    // Initialize the threads
    unsigned int numThreads = OpenThreads::GetNumberOfProcessors();
    arguments.read("--threads", numThreads);

    std::string srcSRSString;
    arguments.read("--src", srcSRSString);
    OSG_NOTICE << "Read src " << srcSRSString << std::endl;

    std::string destSRSString;
    arguments.read("--dest", destSRSString);

    bool geocentric = arguments.read("--geocentric");

    if (geocentric)
    {
        destSRSString = "epsg:4326";
    }

    if (destSRSString.empty() && !srcSRSString.empty() ||
        !destSRSString.empty() && srcSRSString.empty())
    {
        OSG_NOTICE << "Please provide both source and destination srs if you want to reproject" << std::endl;
        return 1;
    }

    //Read in the filenames to process
    for(int pos=1;pos<arguments.argc();++pos)
    {
        if (!arguments.isOption(pos))
        {
			// Expand the filenames using glob
			std::vector< std::string > files = osgJuniper::Utils::glob(arguments[pos]);
			filenames.insert(filenames.end(), files.begin(), files.end());
        }
    }

    if (filenames.size() == 0)
    {
        OSG_NOTICE << "Please specify a filename" << std::endl;
        return 1;
    }


    osg::ref_ptr< osgEarth::SpatialReference > srcSRS;
    osg::ref_ptr< osgEarth::SpatialReference > destSRS;

    if (!srcSRSString.empty() && !destSRSString.empty())
    {
        srcSRS = osgEarth::SpatialReference::create(srcSRSString);
        if (!srcSRS.valid())
        {
            OSG_NOTICE << srcSRSString << " is not a valid SRS" << std::endl;
            return 1;
        }

        destSRS = osgEarth::SpatialReference::create(destSRSString);
        if (!destSRS.valid())
        {
            OSG_NOTICE << destSRSString << " is not a valid SRS" << std::endl;
            return 1;
        }
    }

    OctreeCellBuilder builder;
    for (unsigned int i = 0; i < filenames.size(); i++)
    {
        builder.getInputFiles().push_back(filenames[i]);
        OSG_NOTICE << "Processing filenames " << filenames[i] << std::endl;
    }
    builder.setInnerLevel(innerLevel);
    builder.setMaxLevel(maxLevel);
    builder.setTargetNumPoints(targetNumPoints);
    builder.setSourceSRS(srcSRS.get());
    builder.setDestSRS(destSRS.get());
    builder.setGeocentric(geocentric);
    builder.buildRoot(numThreads);


    osg::Timer_t endTime = osg::Timer::instance()->tick();



    OSG_NOTICE << "Completed " << builder.getProgress()->getTotal() << " in " << osgEarth::prettyPrintTime(osg::Timer::instance()->delta_s(startTime, endTime)) << std::endl;
}
