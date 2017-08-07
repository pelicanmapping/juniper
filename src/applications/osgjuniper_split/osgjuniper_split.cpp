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
#include <pdal/io/BufferReader.hpp>

#include <pdal/StageFactory.hpp>
#include <pdal/filters/StreamCallbackFilter.hpp>
#include <pdal/filters/MergeFilter.hpp>

using namespace osgJuniper;
using namespace pdal;

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

	for (PointList::const_iterator itr = points.begin(); itr != points.end(); ++itr)
	{
		const Point& point = *itr;
		// The point passed, so include it in the list.
		view->setField(pdal::Dimension::Id::X, idx, point.x);
		view->setField(pdal::Dimension::Id::Y, idx, point.y);
		view->setField(pdal::Dimension::Id::Z, idx, point.z);

		view->setField(pdal::Dimension::Id::Red, idx, point.r);
		view->setField(pdal::Dimension::Id::Green, idx, point.g);
		view->setField(pdal::Dimension::Id::Blue, idx, point.b);
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
	if (points.size() > 0)
	{
		OSG_NOTICE << "Read " << points.size() << " from " << filename << std::endl;
	}
}

void appendPointsToLaz(const PointList& points, const std::string& filename)
{
	PointList pts;
	readPointsFromLAZ(pts, filename);
	pts.insert(pts.end(), points.begin(), points.end());
	writePointsToLaz(pts, filename);
}

class OctreeCell
{
public:
	OctreeCell(const std::string &filename)		
	{
		this->filename = filename;
	}

	~OctreeCell()
	{
		if (!points.empty())
		{
			appendPointsToLaz(points, filename);
		}
	}

	PointList points;
	std::string filename;
};

/**
 * Splits a list of input files to a single octree level.
 */
class Splitter
{
public:
	Splitter();
	~Splitter();

	void setFilterID(const OctreeId& id) { _filterID = id; }

	void split();

	std::vector<std::string>& getInputFiles();

	void computeMetaData();

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

	std::string getFilename(OctreeId id, const std::string& ext) const;


	unsigned int _totalNumPoints;
	unsigned int _activePoints;
	osg::BoundingBoxd _bounds;
	unsigned int _level;
	std::vector<std::string> _inputFiles;

	typedef std::map < OctreeId, std::shared_ptr< OctreeCell> > OctreeToCellMap;
	OctreeToCellMap _cells;
	std::shared_ptr<OctreeCell> getOrCreateCell(const OctreeId& id);

	pdal::Stage* _readerStage;

	osg::ref_ptr< OctreeNode > _node;

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

std::string Splitter::getFilename(OctreeId id, const std::string& ext) const
{
	std::stringstream buf;
	buf << "tile_" << id.level << "_" << id.z << "_" << id.x << "_" << id.y << "." << ext;
	return buf.str();
}

std::shared_ptr< OctreeCell > Splitter::getOrCreateCell(const OctreeId& id)
{
	OctreeToCellMap::iterator itr = _cells.find(id);
	if (itr != _cells.end())
	{
		return itr->second;
	}

	// Generate a temporay file for the child.
	std::string filename = getFilename(id, "laz");
	osgEarth::makeDirectoryForFile(filename);

	std::shared_ptr< OctreeCell > cell = std::make_shared<OctreeCell>(filename);
	_cells[id] = cell;
	return cell;
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

		OctreeId childId = _node->getID(osg::Vec3d(x, y, z), _level);

		//std::shared_ptr< PointWriter > writer = getOrCreateWriter(childId);
		//writer->write(point);
		std::shared_ptr< OctreeCell > cell = getOrCreateCell(childId);
		Point p;
		p.x = x;
		p.y = y; 
		p.z = z;
		p.r = point.getFieldAs<int>(Dimension::Id::Red);
		p.g = point.getFieldAs<int>(Dimension::Id::Green);
		p.b = point.getFieldAs<int>(Dimension::Id::Blue);
		cell->points.push_back(p);
		_activePoints++;
		if (_activePoints >= 50000000)
		{
			OSG_NOTICE << "Writing" << std::endl;
			_cells.clear();
			_activePoints = 0;
		}

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


	_cells.clear();
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
				FixedPointTable fixed(1000);
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


void call_from_thread(Splitter& splitter)
{
	splitter.split();
}


int main(int argc, char** argv)
{
	_setmaxstdio(5000);
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

    unsigned int level = 8;
    arguments.read("--level", level);

    // Initialize the threads
    unsigned int numThreads = OpenThreads::GetNumberOfProcessors();
    arguments.read("--threads", numThreads);

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
	splitter.setLevel(level);
	splitter.computeMetaData();


	// Make a splitter per thread
	std::vector< std::thread > threads;
	std::vector< Splitter > splitters;
	osg::ref_ptr< OctreeNode > node = splitter.getOctreeNode();
	for (unsigned int i = 0; i < 8; i++)
	{		
		Splitter s;
		s.getInputFiles().insert(s.getInputFiles().begin(), splitter.getInputFiles().begin(), splitter.getInputFiles().end());
		s.setLevel(level);
		osg::ref_ptr< OctreeNode > childNode = node->createChild(i);
		s.setFilterID(childNode->getID());
		threads.push_back(std::thread(call_from_thread, s));
		splitters.push_back(s);
	}

	// Wait for all the threads to finish.
	for (unsigned int i = 0; i < threads.size(); i++)
	{
		threads[i].join();
	}

    osg::Timer_t endTime = osg::Timer::instance()->tick();

    OSG_NOTICE << "Completed in " << osgEarth::prettyPrintTime(osg::Timer::instance()->delta_s(startTime, endTime)) << std::endl;
}
