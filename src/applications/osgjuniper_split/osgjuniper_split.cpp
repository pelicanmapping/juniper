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
#include <osg/ArgumentParser>
#include <osgJuniper/Utils>
#include <osgEarth/SpatialReference>
#include <osgJuniper/Splitter>

#include <iostream>

using namespace osgJuniper;
using namespace pdal;

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

	int level = -1;
    arguments.read("--level", level);    

	int filterLevel = -1;
	int filterX = -1;
	int filterY = -1;
	int filterZ = -1;
	arguments.read("--filter", filterLevel, filterZ, filterX, filterY);

	std::string driver = "filesystem";
	arguments.read("--driver", driver);

	std::string path;
	arguments.read("--out", path);

	if ((driver == "filesystem" || driver == "directory") && path.empty())
	{
		// Write to the current directory
		path = ".";
	}
	else if (driver == "rocksdb" && path.empty())
	{
		path = "tiles.db";
	}


	unsigned int target = 50000;
	arguments.read("--target", target);

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

	// Create a top level splitter to compute the metadata.
	osg::ref_ptr< Splitter > rootSplitter = new Splitter;
    for (unsigned int i = 0; i < filenames.size(); i++)
    {
		rootSplitter->getInputFiles().push_back(filenames[i]);
        OSG_NOTICE << "Processing filenames " << filenames[i] << std::endl;
    }	
	rootSplitter->setFilterID(OctreeId(filterLevel, filterX, filterY, filterZ));
	rootSplitter->setDestSRS(destSRS.get());
	rootSplitter->setSourceSRS(srcSRS.get());
	rootSplitter->setGeocentric(geocentric);
	rootSplitter->setTargetNumPoints(target);
	rootSplitter->computeMetaData();

	// Get a suggested level if one wasn't specified
	if (level < 0)
	{
		level = rootSplitter->suggestSplitLevel();
	}

	OSG_NOTICE << "Splitting to level " << level << std::endl;

	rootSplitter->setLevel(level);

	// Write out the tileset info.
	TilesetInfo info;
	info.setBounds(rootSplitter->getBounds());
	info.setAdditive(false);
	info.setDriver(driver);
	info.setPath(path);
	TilesetInfo::write(info, "tileset.lastile");

	osg::ref_ptr< PointTileStore > tileStore = PointTileStore::create(info);
	if (!tileStore.valid())
	{
		OSG_NOTICE << "Failed to create tilestore " << driver << std::endl;
		return -1;
	}
	rootSplitter->setTileStore(tileStore.get());
    rootSplitter->split();

    osg::Timer_t endTime = osg::Timer::instance()->tick();

    OSG_NOTICE << "Completed in " << osgEarth::prettyPrintTime(osg::Timer::instance()->delta_s(startTime, endTime)) << std::endl;
}
