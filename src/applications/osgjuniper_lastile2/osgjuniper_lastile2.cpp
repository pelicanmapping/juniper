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
#include <osgJuniper/PDALUtils>
#include <osgJuniper/PointReaderWriter>

#include <pdal/StageFactory.hpp>
#include <pdal/filters/StreamCallbackFilter.hpp>
#include <pdal/filters/MergeFilter.hpp>
#include <pdal/io/BufferReader.hpp>

using namespace osgJuniper;
using namespace pdal;

static pdal::StageFactory _factory;

std::string getFilename(OctreeId id, const std::string& ext)
{
	std::stringstream buf;
	buf << "tile_" << id.level << "_" << id.z << "_" << id.x << "_" << id.y << "." << ext;
	return buf.str();
}

// You could autoscan points at this point and only pick things that you know you actually might use.
class PointDatabase : public osg::Referenced
{
public:
	PointDatabase()
	{
		std::vector< std::string > filenames;

		std::string directory = ".";

		// Load all the point readers		
		std::vector< std::string > contents = osgJuniper::Utils::getFilesFromDirectory(directory, "points");
		for (unsigned int i = 0; i < contents.size(); i++)
		{
			filenames.push_back(contents[i]);
		}
		
		for (unsigned int i= 0; i < filenames.size(); i++)
		{
			_filenames.push_back(filenames[i]);
		}

		std::cout << "Readers " << _filenames.size() << std::endl;
	}

	void build()
	{
		PointTable pointTable;
		pointTable.layout()->registerDim(Dimension::Id::X);
		pointTable.layout()->registerDim(Dimension::Id::Y);
		pointTable.layout()->registerDim(Dimension::Id::Z);
		pointTable.layout()->registerDim(Dimension::Id::Red);
		pointTable.layout()->registerDim(Dimension::Id::Green);
		pointTable.layout()->registerDim(Dimension::Id::Blue);
		PointViewPtr view(new PointView(pointTable));

		int idx = 0;

		for (unsigned int i = 0; i < _filenames.size(); i++)
		{
			PointReader reader(_filenames[i]);

			int maxPoints = 1;
			int numRead = 0;

			while (reader.hasMore() && numRead < maxPoints)
			{
				Point point;
				reader.read(point);
				// The point passed, so include it in the list.
				view->setField(pdal::Dimension::Id::X, idx, point.x);
				view->setField(pdal::Dimension::Id::Y, idx, point.y);
				view->setField(pdal::Dimension::Id::Z, idx, point.z);

				view->setField(pdal::Dimension::Id::Red, idx, point.r);
				view->setField(pdal::Dimension::Id::Green, idx, point.g);
				view->setField(pdal::Dimension::Id::Blue, idx, point.b);
				idx++;
				numRead++;
			}			
		}


		BufferReader bufferReader;
		bufferReader.addView(view);

		OSG_NOTICE << "View size " << view->size() << std::endl;

		Stage *writer = 0;
		{PDAL_SCOPED_LOCK; writer = _factory.createStage("writers.las"); }

		std::string filename = "tile_0_0_0_0.laz";
		osgEarth::makeDirectoryForFile(filename);

		Options options;
		options.add("filename", filename);

		writer->setInput(bufferReader);
		writer->setOptions(options);
		{ PDAL_SCOPED_LOCK; writer->prepare(pointTable); }
		writer->execute(pointTable);

		// Destroy the writer stage, we're done with it.
		{PDAL_SCOPED_LOCK; _factory.destroyStage(writer); }
	}	
	
	std::vector< std::string > _filenames;
};




int main(int argc, char** argv)
{
    osg::Timer_t startTime = osg::Timer::instance()->tick();
    std::vector< std::string > filenames;

    osg::ArgumentParser arguments(&argc,argv);	

    int level = 8;
    arguments.read("--level", level);

	// Read the metadata file produced by the splitter
	double minX, minY, minZ, maxX, maxY, maxZ;
	std::ifstream in("metadata.txt");
	in >> minX >> minY >> minZ >> maxX >> maxY >> maxZ;
	std::cout << "Bounds " << minX << " " << minY << " " << maxZ
		<< maxX << " " << maxY << " " << maxZ << std::endl;
	
    // Initialize the threads
    unsigned int numThreads = OpenThreads::GetNumberOfProcessors();
    arguments.read("--threads", numThreads);


	osg::ref_ptr< OctreeNode > root = new OctreeNode();
	root->setBoundingBox(osg::BoundingBoxd(minX, minY, minZ, maxX, maxY, maxZ));

	/*

	for (int l = level; l >= 0; l--)
	{		
		unsigned int dim = root->getDimensions(l);
		OSG_NOTICE << "Working on level " << l << " with dim " << dim << std::endl;

		for (unsigned int x = 0; x < dim; x++)
		{
			for (unsigned int y = 0; y < dim; y++)
			{
				for (unsigned int z = 0; z < dim; z++)
				{
					osg::ref_ptr< OctreeNode > node = root->createChild(OctreeId(l, x, y, z));
				}
			}
		}
	}
	*/
	osg::ref_ptr< PointDatabase > db = new PointDatabase;
	db->build();

#if 0

	PointTable pointTable;
	pointTable.layout()->registerDim(Dimension::Id::X);
	pointTable.layout()->registerDim(Dimension::Id::Y);
	pointTable.layout()->registerDim(Dimension::Id::Z);
	pointTable.layout()->registerDim(Dimension::Id::Red);
	pointTable.layout()->registerDim(Dimension::Id::Green);
	pointTable.layout()->registerDim(Dimension::Id::Blue);
	PointViewPtr view(new PointView(pointTable));

	int idx = 0;

	// The point passed, so include it in the list.
	view->setField(pdal::Dimension::Id::X, idx, 0.0);
	view->setField(pdal::Dimension::Id::Y, idx, 0.0);
	view->setField(pdal::Dimension::Id::Z, idx, 0.0);

	view->setField(pdal::Dimension::Id::Red, idx, 0);
	view->setField(pdal::Dimension::Id::Green, idx, 0);
	view->setField(pdal::Dimension::Id::Blue, idx, 0);


	BufferReader bufferReader;
	bufferReader.addView(view);

	OSG_NOTICE << "View size " << view->size() << std::endl;

	Stage *writer = 0;
	{PDAL_SCOPED_LOCK; writer = _factory.createStage("writers.las"); }

	std::string filename = "tile_0_0_0_0.laz";
	osgEarth::makeDirectoryForFile(filename);

	Options options;
	options.add("filename", filename);

	writer->setInput(bufferReader);
	writer->setOptions(options);
	{ PDAL_SCOPED_LOCK; writer->prepare(pointTable); }
	writer->execute(pointTable);

	// Destroy the writer stage, we're done with it.
	{PDAL_SCOPED_LOCK; _factory.destroyStage(writer); }
#endif


	osg::Timer_t endTime = osg::Timer::instance()->tick();

    OSG_NOTICE << "Completed in " << osgEarth::prettyPrintTime(osg::Timer::instance()->delta_s(startTime, endTime)) << std::endl;
}
