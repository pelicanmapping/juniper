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
#include <osgJuniper/FilePointTileStore>
#include <osgJuniper/RocksDBPointTileStore>
#include <osgJuniper/PointReaderWriter>
#include <pdal/io/BufferReader.hpp>

#include <pdal/StageFactory.hpp>
#include <pdal/filters/StreamCallbackFilter.hpp>
#include <pdal/filters/MergeFilter.hpp>

using namespace osgJuniper;
using namespace pdal;

const unsigned int targetPoints = 50000;

static pdal::StageFactory _factory;

Stage* createStageForFile(const std::string& filename) {
	PDAL_SCOPED_LOCK;

	Stage* reader = 0;

		std::string driver = PDALUtils::inferReaderDriver(filename);
		reader = _factory.createStage(driver);
		if (reader) {
			Options opt;
			opt.add("filename", filename);
			reader->setOptions(opt);
		}
		else {
			OSG_WARN << "No reader for " << filename << std::endl;
		}
	return reader;
}

class OctreeCell
{
public:
	OctreeCell():
		_count(0)
	{		
	}

	unsigned int _count;
};

/**
 * Splits a list of input files to a single octree level.
 */
class Splitter
{
public:
	Splitter();
	~Splitter();

	PointTileStore* getTileStore() const { return _tileStore; }
	void setTileStore(PointTileStore* tileStore) { _tileStore = tileStore; }

	void setFilterID(const OctreeId& id) { _filterID = id; }

	void split();

	std::vector<std::string>& getInputFiles();

	void computeMetaData();

	/**
	 * Suggests a split level based on the metadata of the input files
	 */
	int suggestSplitLevel();

	unsigned int getLevel() const { return _level; }
	void setLevel(unsigned int level) { _level = level; }

	void initReader();
	void closeReader();

	OctreeNode* getOctreeNode() const { return _node.get(); }

	const osg::BoundingBoxd& getBounds() const
	{
		return _bounds;
	}

	unsigned int getTotalNumPoints() const
	{
		return _totalNumPoints;
	}

protected:

	void addPoint(const Point& point);

	void writeNode(OctreeNode* node);

	void flush();


	unsigned int _totalNumPoints;
	unsigned int _activePoints;
	osg::BoundingBoxd _bounds;
	unsigned int _level;
	std::vector<std::string> _inputFiles;

	typedef std::map < OctreeId, std::shared_ptr< OctreeCell> > OctreeToCellMap;	
	OctreeToCellMap _cells;
	std::shared_ptr<OctreeCell> getOrCreateCell(const OctreeId& id);

	typedef std::map < OctreeId, osg::ref_ptr< OctreeNode> > OctreeToNodeMap;
	OctreeToNodeMap _nodes;
	osg::ref_ptr<OctreeNode> getOrCreateNode(const OctreeId& id);

	pdal::Stage* _readerStage;

	osg::ref_ptr< OctreeNode > _node;

	osg::ref_ptr< PointTileStore > _tileStore;

	OctreeId _filterID;
};

Splitter::Splitter():
	_totalNumPoints(0),
	_activePoints(0),
	_readerStage(0)
{
}

Splitter::~Splitter()
{
	closeReader();
}

std::vector<std::string>& Splitter::getInputFiles()
{
	return _inputFiles;
}


std::shared_ptr< OctreeCell > Splitter::getOrCreateCell(const OctreeId& id)
{
	OctreeToCellMap::iterator itr = _cells.find(id);
	if (itr != _cells.end())
	{
		return itr->second;
	}

	std::shared_ptr< OctreeCell > cell = std::make_shared<OctreeCell>();
	_cells[id] = cell;
	return cell;
}

osg::ref_ptr< OctreeNode > Splitter::getOrCreateNode(const OctreeId& id)
{
	OctreeToNodeMap::iterator itr = _nodes.find(id);
	if (itr != _nodes.end())
	{
		return itr->second;
	}

	osg::ref_ptr< OctreeNode > node = _node->createChild(id);
	_nodes[id] = node;
	return node;
}

int Splitter::suggestSplitLevel()
{
	float width = _node->getWidth();
	float depth = _node->getDepth();
	float height = _node->getHeight();

	// Compute the volume of the dataset.
	float volume = width * depth * height;

	// Get the number of points per cubic meter
	float pointsPerCubicMeter = (float)_totalNumPoints / volume;

	// Find the level that most closely corresponds to our suggested level.	
	float outVolume = volume;
	unsigned int suggestedLevel = 0;
	for (unsigned int i = 0; i < 10; i++)
	{
		suggestedLevel = i;
		unsigned int numberOfPoints = (outVolume * pointsPerCubicMeter);
		if (numberOfPoints <= targetPoints)
		{
			break;
		}
		outVolume /= 8.0;
	}
	return suggestedLevel;
}

/**
 * Get the leaf node from an OctreeNode where a point should be inserted.
 */
OctreeNode* getLeafNode(OctreeNode* node, const osg::Vec3d& point)
{
	if (!node->isSplit() && node->getBoundingBox().contains(point)) return node;

	if (node->isSplit())
	{
		for (unsigned int i = 0; i < node->getChildren().size(); i++)
		{
			OctreeNode* result = getLeafNode(node->getChildren()[i].get(), point);
			if (result)
			{
				return result;
			}
		}
	}

	return 0;
}

void Splitter::addPoint(const Point& p)
{
	osg::Vec3d position(p.x, p.y, p.z);
	OctreeId childId = _node->getID(position, _level);

	// Figure out which node the point should be inserted at at the split level
    osg::ref_ptr< OctreeNode > baseNode = getOrCreateNode(childId);
	osg::ref_ptr< OctreeNode > node = getLeafNode(baseNode.get(), position);

	// Get the cell count of the desired node
	std::shared_ptr< OctreeCell > cell = getOrCreateCell(node->getID());;
	if (cell->_count == targetPoints)
	{
		OSG_NOTICE << "Splitting cell " << node->getID().level << " / " << node->getID().z << " / " << node->getID().x << " / " << node->getID().y << " with " << cell->_count << " points" << std::endl;
		// This cell is at it's maximum, so we need to subdivide the cell and start inserting at it's children instead

		// Get all of the points for this cell (including ones in memory in the node itself currently)
		_tileStore->get(node->getID(), node->getPoints());

		// Set the cell count to zero, it no longer contains any points.
		cell->_count = 0;

		// Split the node so points will be added to the leaf instead.
		node->split();

		// Remove the node from the store.
		_tileStore->remove(node->getID());


		// Make a temporary list of points to write back out.
		PointList points;
		// Add the input poinnode, it's going to be filtered down to the children in the next loop.
		points.push_back(p);
		points.insert(points.end(), node->getPoints().begin(), node->getPoints().end());
		
		// Clear points out of the node
		node->getPoints().clear();

		for (PointList::iterator itr = node->getPoints().begin(); itr != node->getPoints().end(); ++itr)
		{			
			addPoint(*itr);
		}				
	}
	else
	{
		cell->_count++;
		node->getPoints().push_back(p);

		_activePoints++;
		if (_activePoints >= 50000000)
		{
			flush();
		}
	}
}

void Splitter::writeNode(OctreeNode* node)
{
	_tileStore->set(node->getID(), node->getPoints(), true);
	node->getPoints().clear();

	for (unsigned int i = 0; i < node->getChildren().size(); i++)
	{
		writeNode(node->getChildren()[i].get());
	}
}

void Splitter::split()
{
	// First compute the metadata.
	computeMetaData();	

	// Initialize the reader
	initReader();

	osg::ref_ptr< OctreeNode > filterNode;
	if (_filterID.valid())
	{
		filterNode = _node->createChild(_filterID);
	}

	int complete = 0;

	// Read all the points
	StreamCallbackFilter callbackFilter;
	callbackFilter.setInput(*_readerStage);
	auto cb = [&](PointRef& point) mutable
	{
		double x = point.getFieldAs<double>(Dimension::Id::X);
		double y = point.getFieldAs<double>(Dimension::Id::Y);
		double z = point.getFieldAs<double>(Dimension::Id::Z);

		if (filterNode.valid() && !filterNode->getBoundingBox().contains(osg::Vec3d(x,y,z)))
		{
			complete++;
			if (complete % 10000 == 0)
			{
				OSG_NOTICE << "Completed " << complete << " of " << _totalNumPoints << std::endl;
			}
			return false;
		}

		Point p;
		p.x = x;
		p.y = y; 
		p.z = z;
		p.r = point.getFieldAs<int>(Dimension::Id::Red);
		p.g = point.getFieldAs<int>(Dimension::Id::Green);
		p.b = point.getFieldAs<int>(Dimension::Id::Blue);

		addPoint(p);

		complete++;
		if (complete % 10000 == 0)
		{
			OSG_NOTICE << "Completed " << complete << " of " << _totalNumPoints << std::endl;
		}

		return true;
	};
	callbackFilter.setCallback(cb);

	FixedPointTable fixed(1000);
	{PDAL_SCOPED_LOCK; callbackFilter.prepare(fixed); }
	callbackFilter.execute(fixed);

	flush();
}

void Splitter::computeMetaData()
{
	_totalNumPoints = 0;
	_bounds.init();

	double minX = DBL_MAX;
	double minY = DBL_MAX;
	double minZ = DBL_MAX;
	double maxX = -DBL_MAX;
	double maxY = -DBL_MAX;
	double maxZ = -DBL_MAX;

	unsigned int total = 0;
	for (unsigned int i = 0; i < _inputFiles.size(); i++)
	{
		// Create a reader stage for the file.
		Stage* reader = createStageForFile(_inputFiles[i]);

		// Do a quick preview on the file to get the input.		 
		{
			PDAL_SCOPED_LOCK;
			QuickInfo info = reader->preview();
			if (info.m_valid)
			{
				_totalNumPoints += info.m_pointCount;

				if (minX > info.m_bounds.minx) minX = info.m_bounds.minx;
				if (minY > info.m_bounds.miny) minY = info.m_bounds.miny;
				if (minZ > info.m_bounds.minz) minZ = info.m_bounds.minz;

				if (maxX < info.m_bounds.maxx) maxX = info.m_bounds.maxx;
				if (maxY < info.m_bounds.maxy) maxY = info.m_bounds.maxy;
				if (maxZ < info.m_bounds.maxz) maxZ = info.m_bounds.maxz;
			}
			else
			{
				std::cout << "Computing metadata from file" << std::endl;
				// We have to read all the points to compute the metadata
				StreamCallbackFilter callbackFilter;
				callbackFilter.setInput(*reader);
				auto cb = [&](PointRef& point) mutable
				{
					_totalNumPoints++;
					double x = point.getFieldAs<double>(pdal::Dimension::Id::X);
					double y = point.getFieldAs<double>(pdal::Dimension::Id::Y);
					double z = point.getFieldAs<double>(pdal::Dimension::Id::Z);

					if (minX > x) minX = x;
					if (minY > y) minY = y;
					if (minZ > z) minZ = z;

					if (maxX < x) maxX = x;
					if (maxY < y) maxY = y;
					if (maxZ < z) maxZ = z;
					return true;

				};
				callbackFilter.setCallback(cb);
				FixedPointTable fixed(100);
				{PDAL_SCOPED_LOCK; callbackFilter.prepare(fixed); }
				callbackFilter.execute(fixed);
			}
		}
	}

	_bounds = osg::BoundingBoxd(minX, minY, minZ, maxX, maxY, maxZ);

	double width = _bounds.xMax() - _bounds.xMin();
	double height = _bounds.zMax() - _bounds.zMin();
	double depth = _bounds.yMax() - _bounds.yMin();
	double max = osg::maximum(osg::maximum(width, height), depth);
	double halfMax = max / 2.0;

	osg::Vec3d center = _bounds.center();

	_bounds = osg::BoundingBox(center - osg::Vec3d(halfMax, halfMax, halfMax), center + osg::Vec3d(halfMax, halfMax, halfMax));

	// Create the root OctreeNode
	_node = new OctreeNode();
	_node->setBoundingBox(_bounds);

	// Write out some metadata
	std::ofstream out("metadata.txt");
	out << std::setprecision(8) << _bounds.xMin() << " " << _bounds.yMin() << " " << _bounds.zMin() << " " << _bounds.xMax() << " " << _bounds.yMax() << " " << _bounds.zMax();
	out.close();

	OSG_NOTICE << "points=" << _totalNumPoints << std::endl
		<< " bounds " << _bounds.xMin() << ", " << _bounds.yMin() << ", " << _bounds.zMin() << " to "
		<< _bounds.xMax() << ", " << _bounds.yMax() << ", " << _bounds.zMax() << std::endl;
}


void Splitter::flush()
{
	OSG_NOTICE << "Writing" << std::endl;
	
	for (OctreeToNodeMap::iterator itr = _nodes.begin(); itr != _nodes.end(); ++itr)
	{
		writeNode(itr->second.get());
	}

	_activePoints = 0;
}

void Splitter::initReader()
{
	// Create a merge filter
	MergeFilter *merged = new MergeFilter();
	for (unsigned int i = 0; i < _inputFiles.size(); i++)
	{
		Stage* stage = createStageForFile(_inputFiles[i]);
		if (stage)
		{
			merged->getInputs().push_back(stage);
		}
	}
	_readerStage = merged;
}

void Splitter::closeReader()
{
	PDAL_SCOPED_LOCK;
	if (_readerStage)
	{
		for (unsigned int i = 0; i < _readerStage->getInputs().size(); i++)
		{
			_factory.destroyStage(_readerStage->getInputs()[i]);
		}
		delete _readerStage;
		_readerStage = 0;
	}
}

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

	// Create a top level splitter to compute the metadata.
	Splitter splitter;
    for (unsigned int i = 0; i < filenames.size(); i++)
    {
        splitter.getInputFiles().push_back(filenames[i]);
        OSG_NOTICE << "Processing filenames " << filenames[i] << std::endl;
    }	
	splitter.computeMetaData();

	// Get a suggested level if one wasn't specified
	if (level < 0)
	{
		level = splitter.suggestSplitLevel();		
	}

	OSG_NOTICE << "Splitting to level " << level << std::endl;

    splitter.setLevel(level);

	osg::ref_ptr< PointTileStore > tileStore = new FilePointTileStore(".");
	//osg::ref_ptr< PointTileStore > tileStore = new RocksDBPointTileStore("tiled");
	splitter.setTileStore(tileStore.get());
	splitter.split();

    osg::Timer_t endTime = osg::Timer::instance()->tick();

    OSG_NOTICE << "Completed in " << osgEarth::prettyPrintTime(osg::Timer::instance()->delta_s(startTime, endTime)) << std::endl;
}
