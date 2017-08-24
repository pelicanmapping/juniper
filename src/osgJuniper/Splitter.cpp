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

#include <osgJuniper/Splitter>

#include <osgEarth/StringUtils>
#include <osgEarth/FileUtils>
#include <osgEarth/SpatialReference>
#include <osgEarth/GeoData>

#include <osg/BoundingBox>
#include <osg/ArgumentParser>

#include <osgDB/FileUtils>
#include <osgDB/FileNameUtils>

#include <osgJuniper/Octree>
#include <osgJuniper/Utils>
#include <osgJuniper/PDALUtils>
#include <osgJuniper/PointTileStore>
#include <osgJuniper/TilesetInfo>

#include <pdal/io/BufferReader.hpp>
#include <pdal/StageFactory.hpp>
#include <pdal/filters/StreamCallbackFilter.hpp>
#include <pdal/filters/MergeFilter.hpp>

using namespace osgEarth;
using namespace osgJuniper;
using namespace pdal;

Splitter::Splitter() :
	_totalNumPoints(0),
	_activePoints(0),
	_readerStage(0),
	_geocentric(false),
	_targetNumPoints(50000),
	_level(6)
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


std::shared_ptr< Splitter::OctreeCell > Splitter::getOrCreateCell(const OctreeId& id)
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
	// Compute volume of octree root node
	double nodeWidth = _node->getWidth();
	double nodeDepth = _node->getDepth();
	double nodeHeight = _node->getHeight();
	double nodeVolume = nodeWidth * nodeDepth * nodeHeight;

	// Compute volume of source dataset
	double dataWidth = _dataBounds.xMax() - _dataBounds.xMin();
	double dataDepth = _dataBounds.yMax() - _dataBounds.yMin();
	double dataHeight = _dataBounds.zMax() - _dataBounds.zMin();
	double dataVolume = dataWidth * dataDepth * dataHeight;

	// Assuming a uniform point distribution, compute the octree level that will contain
	// the target number of points per cell
	double pointRatio = (double)_totalNumPoints / (double)_targetNumPoints;
	double volumeRatio = nodeVolume / dataVolume;
	int suggestedLevel = std::ceil(std::log(pointRatio*volumeRatio) / std::log(8.0));
	suggestedLevel += 2; // Increase since some regions will have much higher than average point density
	return suggestedLevel;
}

/**
* Get the leaf node from an OctreeNode where a point should be inserted.
*/
OctreeNode* getLeafNode(OctreeNode* node, const osg::Vec3d& point)
{
	// Only check this node if the node contains the point.
	if (node->getBoundingBox().contains(point))
	{
		// If the node isn't split, then use this node.
		if (!node->isSplit()) return node;

		// Check the children of this node.
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
	}
	return 0;
}

void Splitter::reprojectPoint(const osg::Vec3d& input, osg::Vec3d& output)
{
	if (_srcSRS.valid() && _destSRS.valid())
	{
		osgEarth::GeoPoint geoPoint(_srcSRS, input);
		osgEarth::GeoPoint mapPoint;
		geoPoint.transform(_destSRS, mapPoint);
		if (!_geocentric)
		{
			output = mapPoint.vec3d();
		}
		else
		{
			mapPoint.toWorld(output);
		}
	}
	else
	{
		output = input;
	}
}


void Splitter::addPoint(const Point& p)
{
	osg::Vec3d position(p.x, p.y, p.z);
	// Make sure the point fits in the bounding box.
	if (!_node->getBoundingBox().contains(position))
	{
		OSG_NOTICE << "Skipping point " << p.x << ", " << p.y << ", " << p.z << " since it doesn't fit in bounding box" << std::endl;
		return;
	}
	OctreeId childId = _node->getID(position, _level);

	// Figure out which node the point should be inserted at at the split level
	osg::ref_ptr< OctreeNode > baseNode = getOrCreateNode(childId);

	osg::ref_ptr< OctreeNode > node = getLeafNode(baseNode.get(), position);

	// Get the cell count of the desired node
	std::shared_ptr< OctreeCell > cell = getOrCreateCell(node->getID());

	if (cell->_count == _targetNumPoints)
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
		node->clearPoints();

		// Add the points back into the dataset, which will navigate down to the child nodes.
		for (PointList::iterator itr = points.begin(); itr != points.end(); ++itr)
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
	// Only write the node if it's non-empty
	if (!node->getPoints().empty())
	{
		_tileStore->set(node->getID(), node->getPoints(), true);
		node->clearPoints();
	}

	// Write all the children
	for (unsigned int i = 0; i < node->getChildren().size(); i++)
	{
		writeNode(node->getChildren()[i].get());
	}
}

osgEarth::SpatialReference* Splitter::getSourceSRS() const
{
	return _srcSRS.get();
}

void Splitter::setSourceSRS(osgEarth::SpatialReference* srs)
{
	_srcSRS = srs;
}

osgEarth::SpatialReference* Splitter::getDestSRS() const
{
	return _destSRS;
}

void Splitter::setDestSRS(osgEarth::SpatialReference* srs)
{
	_destSRS = srs;
}

bool Splitter::getGeocentric() const
{
	return _geocentric;
}

void Splitter::setGeocentric(bool geocentric)
{
	_geocentric = geocentric;
}

Stage* Splitter::createStageForFile(const std::string& filename)
{
	PDAL_SCOPED_LOCK;

	Stage* reader = 0;

	std::string driver = PDALUtils::inferReaderDriver(filename);
	reader = PDALUtils::getStageFactory()->createStage(driver);
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

void Splitter::split()
{
	// First compute the metadata.
	computeMetaData();

	// Initialize the reader
	initReader();

	int complete = 0;

	// Read all the points
	StreamCallbackFilter callbackFilter;
	callbackFilter.setInput(*_readerStage);
	auto cb = [&](PointRef& point) mutable
	{
		double x = point.getFieldAs<double>(Dimension::Id::X);
		double y = point.getFieldAs<double>(Dimension::Id::Y);
		double z = point.getFieldAs<double>(Dimension::Id::Z);

		if (_filterNode.valid() && !_filterNode->getBoundingBox().contains(osg::Vec3d(x, y, z)))
		{
			complete++;
			if (complete % 10000 == 0)
			{
				OSG_NOTICE << "Completed " << complete << " of " << _totalNumPoints << std::endl;
			}
			return false;
		}

		// Reproject the point if necessary
		osg::Vec3d in(x, y, z);
		osg::Vec3d out;
		reprojectPoint(in, out);

		Point p;
		p.x = out.x();
		p.y = out.y();
		p.z = out.z();
		p.r = point.getFieldAs<int>(Dimension::Id::Red);
		p.g = point.getFieldAs<int>(Dimension::Id::Green);
		p.b = point.getFieldAs<int>(Dimension::Id::Blue);
		if (point.hasDim(Dimension::Id::Classification))
		{
			p.classification = point.getFieldAs<char>(Dimension::Id::Classification);
		}
		if (point.hasDim(Dimension::Id::Intensity))
		{
			p.intensity = point.getFieldAs<int>(Dimension::Id::Intensity);
		}

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

	// Reproject the corners of the bounds if reprojection is enabled
	if (_srcSRS.valid() && _destSRS.get())
	{
		minX = DBL_MAX;
		minY = DBL_MAX;
		minZ = DBL_MAX;
		maxX = -DBL_MAX;
		maxY = -DBL_MAX;
		maxZ = -DBL_MAX;

		for (unsigned int i = 0; i < 8; i++)
		{
			osg::Vec3d cornerIn = _bounds.corner(i);
			osg::Vec3d cornerOut;
			reprojectPoint(cornerIn, cornerOut);

			if (minX > cornerOut.x()) minX = cornerOut.x();
			if (minY > cornerOut.y()) minY = cornerOut.y();
			if (minZ > cornerOut.z()) minZ = cornerOut.z();

			if (maxX < cornerOut.x()) maxX = cornerOut.x();
			if (maxY < cornerOut.y()) maxY = cornerOut.y();
			if (maxZ < cornerOut.z()) maxZ = cornerOut.z();
		}

		_bounds = osg::BoundingBoxd(minX, minY, minZ, maxX, maxY, maxZ);
	}

	// Save data bounds
	_dataBounds = _bounds;

	double width = _bounds.xMax() - _bounds.xMin();
	double height = _bounds.zMax() - _bounds.zMin();
	double depth = _bounds.yMax() - _bounds.yMin();
	double max = osg::maximum(osg::maximum(width, height), depth);
	// Expand the half max slightly so that points on the edges don't get culled out due to precision errors.
	double halfMax = 1.001 * (max / 2.0);

	osg::Vec3d center = _bounds.center();

	_bounds = osg::BoundingBox(center - osg::Vec3d(halfMax, halfMax, halfMax), center + osg::Vec3d(halfMax, halfMax, halfMax));

	// Create the root OctreeNode
	_node = new OctreeNode();
	_node->setBoundingBox(_bounds);

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
	// Initialize the filter node.
	if (_filterID.valid())
	{
		_filterNode = _node->createChild(_filterID);
	}

	unsigned int numAdded = 0;

	// Create a merge filter
	MergeFilter *merged = new MergeFilter();
	for (unsigned int i = 0; i < _inputFiles.size(); i++)
	{
		bool stageValid = true;
		Stage* stage = createStageForFile(_inputFiles[i]);
		if (stage)
		{
			// See if the stage is within the filter bounds.
			if (_filterNode.valid())
			{
				PDAL_SCOPED_LOCK;
				QuickInfo info = stage->preview();
				if (info.m_valid)
				{
					osg::BoundingBoxd fileBounds(info.m_bounds.minx, info.m_bounds.miny, info.m_bounds.minz,
						info.m_bounds.maxx, info.m_bounds.maxy, info.m_bounds.maxz);
					if (!fileBounds.intersects(_filterNode->getBoundingBox()))
					{
						stageValid = false;
						OSG_NOTICE << "Skipping file " << _inputFiles[i] << std::endl;
					}
				}
			}

			if (stageValid)
			{
				numAdded++;
				merged->getInputs().push_back(stage);
			}
			else
			{
				PDALUtils::getStageFactory()->destroyStage(stage);
			}
		}
	}

	OSG_NOTICE << "Processing " << numAdded << " of " << _inputFiles.size() << std::endl;
	_readerStage = merged;
}

void Splitter::closeReader()
{
	PDAL_SCOPED_LOCK;
	if (_readerStage)
	{
		for (unsigned int i = 0; i < _readerStage->getInputs().size(); i++)
		{
			PDALUtils::getStageFactory()->destroyStage(_readerStage->getInputs()[i]);
		}
		delete _readerStage;
		_readerStage = 0;
	}
}


unsigned int Splitter::getTargetNumPoints() const
{
	return _targetNumPoints;
}

void Splitter::setTargetNumPoints(unsigned int targetNumPoints)
{
	_targetNumPoints = targetNumPoints;
}