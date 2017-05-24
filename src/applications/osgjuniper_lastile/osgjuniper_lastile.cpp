/* -*-c++-*- */
/* osgJuniper - Large Dataset Visualization Toolkit for OpenSceneGraph
* Copyright 2010-2011 Pelican Ventures, Inc.
* http://wush.net/trac/juniper
*
* osgEarth is free software; you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License as published by
* the Free Software Foundation; either version 2 of the License, or
* (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU Lesser General Public License for more details.
*
* You should have received a copy of the GNU Lesser General Public License
* along with this program.  If not, see <http://www.gnu.org/licenses/>
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
        std::vector< std::string > contents = Utils::getFilesFromDirectory(directory, "laz");
        for (unsigned int i = 0; i < contents.size(); i++)
        {
            filenames.push_back(contents[i]);
        }
        contents = Utils::getFilesFromDirectory(directory, "las");
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
            filenames.push_back( arguments[pos]);
            OSG_NOTICE << "filename " << arguments[pos] << std::endl;
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
