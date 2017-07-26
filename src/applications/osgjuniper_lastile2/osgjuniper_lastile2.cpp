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

void writePointsToLaz(const PointList& points, const std::string& filename)
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

	for (unsigned int i = 0; i < points.size(); i++)
	{
		// The point passed, so include it in the list.
		view->setField(pdal::Dimension::Id::X, idx, points[i].x);
		view->setField(pdal::Dimension::Id::Y, idx, points[i].y);
		view->setField(pdal::Dimension::Id::Z, idx, points[i].z);

		view->setField(pdal::Dimension::Id::Red, idx, points[i].r);
		view->setField(pdal::Dimension::Id::Green, idx, points[i].g);
		view->setField(pdal::Dimension::Id::Blue, idx, points[i].b);
		idx++;
	}

	BufferReader bufferReader;
	bufferReader.addView(view);

	Stage *writer = 0;
	{PDAL_SCOPED_LOCK; writer = _factory.createStage("writers.las"); }

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


void readPointsFromLAZ(PointList& points, const std::string& filename)
{
	points.clear();
	OSG_NOTICE << "Reading LAZ file " << filename << std::endl;
	if (osgDB::fileExists(filename))
	{
		Stage* stage = 0;
		{
			PDAL_SCOPED_LOCK;
			stage = _factory.createStage("readers.las");
			pdal::Options opt;
			opt.add("filename", filename);
			stage->setOptions(opt);
		}

		if (stage)
		{
			pdal::PointTable table;
			{ PDAL_SCOPED_LOCK;  stage->prepare(table); }

			pdal::PointViewSet point_view_set = stage->execute(table);
			pdal::PointViewPtr point_view = *point_view_set.begin();

			OSG_NOTICE << " Read " << point_view->size() << " from " << filename << std::endl;

			for (unsigned int i = 0; i < point_view->size(); i++)
			{
				PointRef point(point_view->point(i));
				Point p;
				p.x = point.getFieldAs<double>(pdal::Dimension::Id::X);
				p.y = point.getFieldAs<double>(pdal::Dimension::Id::Y);
				p.z = point.getFieldAs<double>(pdal::Dimension::Id::Z);
				p.r = point.getFieldAs<int>(pdal::Dimension::Id::Red);
				p.g = point.getFieldAs<int>(pdal::Dimension::Id::Green);
				p.b = point.getFieldAs<int>(pdal::Dimension::Id::Blue);
				points.push_back(p);
			}
		}
	}
	else
	{
		OSG_NOTICE << filename << " does not exist" << std::endl;
	}
}

class TileIndex
{	
public:
	TileIndex(OctreeNode* root):
		_root(root)
	{
	}

	void add(const OctreeId& id)
	{
		_tiles.insert(id);
	}

	void scan(const std::string& directory)
	{
		_tiles.clear();

		std::vector< std::string > filenames;		

		// Load all the point readers		
		std::vector< std::string > contents = osgJuniper::Utils::getFilesFromDirectory(directory, "laz");
		for (unsigned int i = 0; i < contents.size(); i++)
		{
			std::string filename = contents[i];

			osgEarth::StringTokenizer tok("_.");
			osgEarth::StringVector tized;
			tok.tokenize(filename, tized);

			if (tized.size() == 7)
			{
				/*
				OSG_NOTICE << filename << std::endl;
				for (unsigned int j = 0; j < tized.size(); j++)
				{
					OSG_NOTICE << "tized " << j << "=" << tized[j] << std::endl;
				}
				*/
				int level = osgEarth::as<int>(tized[2], 0);

				// TODO:  Remove
				if (level == 7)
				{
					int z = osgEarth::as<int>(tized[3], 0);
					int x = osgEarth::as<int>(tized[4], 0);
					int y = osgEarth::as<int>(tized[5], 0);
					_tiles.insert(OctreeId(level, x, y, z));
				}
			}
		}
		OSG_NOTICE << "Found " << _tiles.size() << std::endl;
	}

	void buildParents(unsigned int level)
	{
		std::set< OctreeId > ids;
		for (auto itr = _tiles.begin(); itr != _tiles.end(); ++itr)
		{
			if (itr->level == level)
			{
				OctreeId parent = OctreeNode::getParentID(*itr);
				ids.insert(parent);
			}
		}

		OSG_NOTICE << "Building " << ids.size() << " for level " << level << std::endl;

		for (auto itr = ids.begin(); itr != ids.end(); ++itr)
		{
			build(*itr);
		}
	}

	void build(const OctreeId &id)
	{
		OSG_NOTICE << "Building " << id.level << ", " << id.x << ", " << id.y << ", " << id.z << std::endl;
		osg::ref_ptr< OctreeNode > node = _root->createChild(id);
		node->split();

		// Read all the children points		
		PointList points;
		for (unsigned int i = 0; i < node->getChildren().size(); i++)
		{
			std::string filename = getFilename(node->getChildren()[i]->getID(), "laz");
			PointList pts;
			readPointsFromLAZ(pts, filename);
			points.insert(points.end(), pts.begin(), pts.end());
		}

		OSG_NOTICE << "Read " << points.size() << " from children" << std::endl;

		if (!points.empty())
		{
			PointList keepers;

			std::set< OctreeId > innerCells;

			unsigned int innerLevel = 6;
			for (PointList::iterator itr = points.begin(); itr != points.end(); ++itr)
			{
				OctreeId cell = node->getID(osg::Vec3d(itr->x, itr->y, itr->z), innerLevel);
				if (innerCells.find(cell) == innerCells.end())
				{
					keepers.push_back(*itr);
					innerCells.insert(cell);
				}
			}
			/*

			for (PointList::iterator itr = points.begin(); itr != points.end(); ++itr)
			{
				keepers.push_back(*itr);
			}
			*/


			if (!keepers.empty())
			{				
				std::string tileFilename = getFilename(id, "laz");
				OSG_NOTICE << "Writing " << keepers.size() << " to " << tileFilename << std::endl;
				writePointsToLaz(keepers, tileFilename);
				_tiles.insert(id);
			}
		}
	}

	std::set< OctreeId > _tiles;
	osg::ref_ptr< OctreeNode > _root;
};

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
	std::cout << "Bounds " << minX << " " << minY << " " << minZ << " to " 
		<< maxX << " " << maxY << " " << maxZ << std::endl;
	
    // Initialize the threads
    unsigned int numThreads = OpenThreads::GetNumberOfProcessors();
    arguments.read("--threads", numThreads);


	osg::ref_ptr< OctreeNode > root = new OctreeNode();
	root->setBoundingBox(osg::BoundingBoxd(minX, minY, minZ, maxX, maxY, maxZ));	
	/*
	osg::ref_ptr< PointDatabase > db = new PointDatabase;
	db->build();
	*/

	TileIndex index(root);
	index.scan(".");

	for (int i = 7; i > 0; i--)
	{
		OSG_NOTICE << "Building parents for " << i << std::endl;
		index.buildParents(i);
	}

	osg::Timer_t endTime = osg::Timer::instance()->tick();

    OSG_NOTICE << "Completed in " << osgEarth::prettyPrintTime(osg::Timer::instance()->delta_s(startTime, endTime)) << std::endl;
}
