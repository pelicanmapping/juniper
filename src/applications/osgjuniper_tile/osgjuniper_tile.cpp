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
#include <osgJuniper/Version>

using namespace osgJuniper;

int main(int argc, char** argv)
{
    osg::ArgumentParser arguments(&argc, argv);
    arguments.getApplicationUsage()->setApplicationName(arguments.getApplicationName());
    arguments.getApplicationUsage()->setDescription(arguments.getApplicationName() + " tiles a point cloud to a format suitable for streaming.");
    arguments.getApplicationUsage()->setCommandLineUsage(arguments.getApplicationName() + " [options] file.laz [file2.laz ...]");
    arguments.getApplicationUsage()->addCommandLineOption("--directory", "Loads a directory of laz or las files");
    arguments.getApplicationUsage()->addCommandLineOption("--src", "The source srs");
    arguments.getApplicationUsage()->addCommandLineOption("--dest", "The destination srs");
    arguments.getApplicationUsage()->addCommandLineOption("--geocentric", "Generates a geocentric output");
    arguments.getApplicationUsage()->addCommandLineOption("--target", "The target number of points in a cell");
    arguments.getApplicationUsage()->addCommandLineOption("--innerLevel level", "The octree level to use for the internal box filter for each downsampled cell", "8");
    arguments.getApplicationUsage()->addCommandLineOption("--maxLevel maxLevel", "The maximum level of subdivision for the tileset", "8");    
    arguments.getApplicationUsage()->addCommandLineOption("--threads numThreads", "The number of threads to use", "All available cores");
    arguments.getApplicationUsage()->addCommandLineOption("--version", "Displays the Juniper version");
    arguments.getApplicationUsage()->addCommandLineOption("-h or --help", "Display command line parameters");


    // if user request help write it out to cout.
    if (arguments.read("-h") || arguments.read("--help"))
    {
        arguments.getApplicationUsage()->write(std::cout);
        return 1;
    }

    if (arguments.read("--version"))
    {
        std::cout << osgJuniperGetLibraryName() << " " << osgJuniperGetVersion() << std::endl;
        return 0;
    }

    osg::Timer_t startTime = osg::Timer::instance()->tick();
    std::vector< std::string > filenames;

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
    try
    {
      builder.buildRoot(numThreads);
    }
    catch (pdal::pdal_error& err)
    {
      OSG_FATAL << "osgjuniper_tile: " << err.what() << std::endl;
      return 1;
    }

    osg::Timer_t endTime = osg::Timer::instance()->tick();

    OSG_NOTICE << "Completed " << builder.getProgress()->getTotal() << " in " << osgEarth::prettyPrintTime(osg::Timer::instance()->delta_s(startTime, endTime)) << std::endl;
}
