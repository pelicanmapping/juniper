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
#include <osgJuniper/LASTile>
#include <osgEarth/FileUtils>
#include <osgEarth/Random>
#include <osgEarth/GeoData>
#include <osgDB/FileNameUtils>

#include <pdal/filters/MergeFilter.hpp>
#include <pdal/filters/StreamCallbackFilter.hpp>
#include <pdal/io/BufferReader.hpp>

static OpenThreads::Mutex stageMutex;

#define PDAL_LOCK OpenThreads::ScopedLock< OpenThreads::Mutex > lock(stageMutex)

using namespace osgJuniper;

using namespace pdal;
using namespace std;

static pdal::StageFactory _factory;

Progress::Progress():
_complete(0),
    _total(0)
{
}

unsigned int Progress::getTotal()
{
    OpenThreads::ScopedLock<OpenThreads::Mutex> lock(_mutex);
    return _total;
}

void Progress::setTotal(unsigned int total)
{
    OpenThreads::ScopedLock<OpenThreads::Mutex> lock(_mutex);
    _total = total;
}

unsigned int Progress::getComplete()
{
    OpenThreads::ScopedLock<OpenThreads::Mutex> lock(_mutex);
    return _complete;
}

bool Progress::isComplete()
{
    OpenThreads::ScopedLock<OpenThreads::Mutex> lock(_mutex);
    return _complete == _total;
}

void Progress::incrementComplete(unsigned int complete)
{
    OpenThreads::ScopedLock<OpenThreads::Mutex> lock(_mutex);
    _complete += complete;

    if (_complete % 5000 == 0)
    {
        OSG_NOTICE << "Finished " << _complete << " of " << _total << ". " << getPercentComplete()<< "% complete" << std::endl;
    }        
}

float Progress::getPercentComplete()
{        
    return ((float)_complete / (float)_total) * 100.0;
}

/******************************************************************/

/**************************************************/

class BuildCellOperator : public osg::Operation
{
public:
    BuildCellOperator(OctreeCellBuilder* builder):
      _builder(builder)
      {
      }

      void operator()(osg::Object* object)
      {
		  if (_builder.valid())
		  {
			  _builder->build();
			  _builder->buildChildren();
		  }
		  else
		  {
			  OSG_NOTICE << "No builder?" << std::endl;
		  }
      }

      osg::ref_ptr< OctreeCellBuilder > _builder;
};

OctreeCellBuilder::OctreeCellBuilder():
_innerLevel(6),
    _maxLevel(8),
    _targetNumPoints(0),
    _numPoints(0),
    _deleteInputs(false),
    _fraction(1.0),
    _geocentric(false),
	_totalNumPoints(0),
	_readerStage(0)
{
}

OctreeCellBuilder::~OctreeCellBuilder()
{                 
}

unsigned int OctreeCellBuilder::getNumPoints() const
{
    return _numPoints;
}

OctreeNode* OctreeCellBuilder::getNode() const
{
    return _node;
}

void OctreeCellBuilder::setNode( OctreeNode *node )
{
    _node = node;
}

unsigned int OctreeCellBuilder::getInnerLevel() const
{
    return _innerLevel;
}

void OctreeCellBuilder::setInnerLevel(unsigned int innerLevel)
{
    _innerLevel = innerLevel;
}

unsigned int OctreeCellBuilder::getMaxLevel() const
{
    return _maxLevel;
}

void OctreeCellBuilder::setMaxLevel(unsigned int maxLevel)
{
    _maxLevel = maxLevel;
}

std::vector<std::string>& OctreeCellBuilder::getInputFiles()
{
    return _inputFiles;
}

std::vector<std::string>& OctreeCellBuilder::getOutputFiles()
{
    return _outputFiles;
} 

bool OctreeCellBuilder::getDeleteInputs() const
{
    return _deleteInputs;
}

void OctreeCellBuilder::setDeleteInputs(bool deleteInputs)
{
    _deleteInputs = deleteInputs;
}

unsigned int OctreeCellBuilder::getTargetNumPoints() const
{
    return _targetNumPoints;
}

void OctreeCellBuilder::setTargetNumPoints(unsigned int targetNumPoints)
{
    _targetNumPoints = targetNumPoints;
}

float OctreeCellBuilder::getFraction() const
{
    return _fraction;
}

void OctreeCellBuilder::setFraction(float fraction)
{
    _fraction = osg::clampBetween(fraction, 0.0f, 1.0f);
}

static osgEarth::Random random;

bool OctreeCellBuilder::keep()
{
    if (_numPoints >= _targetNumPoints) return false;    
	return true;
}

osgEarth::SpatialReference* OctreeCellBuilder::getSourceSRS() const
{
    return _srcSRS.get();
}

void OctreeCellBuilder::setSourceSRS( osgEarth::SpatialReference* srs)
{
    _srcSRS = srs;
}

osgEarth::SpatialReference* OctreeCellBuilder::getDestSRS() const
{
    return _destSRS;
}

void OctreeCellBuilder::setDestSRS( osgEarth::SpatialReference* srs)
{
    _destSRS = srs;
}    

bool OctreeCellBuilder::getGeocentric() const
{
    return _geocentric;
}

void OctreeCellBuilder::setGeocentric(bool geocentric)
{
    _geocentric = geocentric;
}

osg::Vec3d OctreeCellBuilder::reprojectPoint(const osg::Vec3d& input)
{
    if (_srcSRS.valid() && _destSRS.valid())
    {
        osgEarth::GeoPoint geoPoint(_srcSRS, input);                  
        osgEarth::GeoPoint mapPoint;
        geoPoint.transform(_destSRS, mapPoint);     
        if (!_geocentric)
        {
            return mapPoint.vec3d();
        }    
        osg::Vec3d world;
        mapPoint.toWorld(world);
        return world;              
    }    
    return input;
}

std::string OctreeCellBuilder::getFilename(OctreeId id, const std::string& ext) const
{
    std::stringstream buf;
    buf << "tile_" << id.level << "_" << id.z << "_" << id.x << "_" << id.y << "." << ext;    
    return buf.str();
}

 osg::OperationQueue* OctreeCellBuilder::getOperationQueue() const
 {
     return _queue;
 }

 void OctreeCellBuilder::setOperationQueue(osg::OperationQueue* queue)
 {
     _queue = queue;
 }

 void OctreeCellBuilder::initThreads(unsigned int numThreads)
 {
    for (unsigned int i = 0; i < numThreads; i++)
    {
        osg::OperationsThread* thread = new osg::OperationsThread();
        thread->setOperationQueue(_queue);
        thread->start();
        _threads.push_back(thread);
    }
 }

 Progress* OctreeCellBuilder::getProgress() const
 {
     return _progress;
 }

 void OctreeCellBuilder::setProgress(Progress* progress)
 {
     _progress = progress;
 }

 void OctreeCellBuilder::computeMetaData()
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
			 PDAL_LOCK;
			 QuickInfo info = reader->preview();
			 if (info.m_valid)
			 {
				 _totalNumPoints += info.m_pointCount;

				 if (minX > info.m_bounds.minx) minX = info.m_bounds.minx;
				 if (minY > info.m_bounds.miny) minY = info.m_bounds.miny;
				 if (minZ > info.m_bounds.minz) minZ = info.m_bounds.miny;

				 if (maxX < info.m_bounds.maxx) maxX = info.m_bounds.maxx;
				 if (maxY < info.m_bounds.maxy) maxY = info.m_bounds.maxy;
				 if (maxZ < info.m_bounds.maxz) maxZ = info.m_bounds.maxy;
			 }
		 }
	 }

	 _bounds = osg::BoundingBoxd(minX, minY, minZ, maxX, maxY, maxZ);

	 double width = _bounds.xMax() - _bounds.xMin();
	 double height = _bounds.zMax() - _bounds.zMin();
	 double depth = _bounds.yMax() - _bounds.yMin();
	 double max = osg::maximum(osg::maximum(width, height), depth);
	 _bounds.xMax() = _bounds.xMin() + max;
	 _bounds.yMax() = _bounds.yMin() + max;
	 _bounds.zMax() = _bounds.zMin() + max;
}

 void OctreeCellBuilder::buildRoot(unsigned int numThreads)
 {   	 
	 computeMetaData();
	 
     // This is the root builder.
     // Initialize an operations queue
     _queue = new osg::OperationQueue;

     // Initialize the progress
     _progress = new Progress();

	 _progress->setTotal(_totalNumPoints);
	 
     initThreads(numThreads);

     build();
     buildChildren();    
     // Wait for completion    
     while (!_progress->isComplete())
     {        
         OpenThreads::Thread::microSleep(5 * 1000 * 1000);
     }
 }
 
void OctreeCellBuilder::build()
{        	
    initReader();

    // If we aren't given a node, assume we are the root
    if (!_node.valid())
    {
		_node = new OctreeNode();
		_node->setBoundingBox(_bounds);
    }

	// Initialize the child arrays.	
    _childWriters.clear();
    _children.clear();
    _outputFiles.clear();

    // Initialize the octree children and set their writers to NULL
    for (unsigned int i = 0; i < 8; i++)
    {		
        _childWriters.push_back(0);
        _children.push_back(_node->createChild(i));
        _outputFiles.push_back("");
    }

	PointTable pointTable;
	pointTable.layout()->registerDim(Dimension::Id::X);
	pointTable.layout()->registerDim(Dimension::Id::Y);
	pointTable.layout()->registerDim(Dimension::Id::Z);
	pointTable.layout()->registerDim(Dimension::Id::Red);
	pointTable.layout()->registerDim(Dimension::Id::Green);
	pointTable.layout()->registerDim(Dimension::Id::Blue);
	PointViewPtr view(new PointView(pointTable));

	int idx = 0;
	
	// Read all the points
	StreamCallbackFilter callbackFilter;
	callbackFilter.setInput(*_readerStage);
	auto cb = [this, &idx, &view](PointRef& point) mutable
	{
		double x = point.getFieldAs<double>(Dimension::Id::X);
		double y = point.getFieldAs<double>(Dimension::Id::Y);
		double z = point.getFieldAs<double>(Dimension::Id::Z);		

		// Reproject the point
		osg::Vec3d location(x, y, z);
		osg::Vec3d world = reprojectPoint(location);

		point.setField(Dimension::Id::X, world.x());
		point.setField(Dimension::Id::Y, world.y());
		point.setField(Dimension::Id::Z, world.z());

		// Figure out what cell this point should go in.
		OctreeId id = _node->getID(location, _innerLevel);

		// See how many points are currently in this cell
		int count = getPointsInCell(id);

		if ((_targetNumPoints != 0 && keep()) || count == 0 || _node->getID().level >= _maxLevel)
		{
			// The point passed, so include it in the list.
			view->setField(pdal::Dimension::Id::X, idx, point.getFieldAs<double>(pdal::Dimension::Id::X));
			view->setField(pdal::Dimension::Id::Y, idx, point.getFieldAs<double>(pdal::Dimension::Id::Y));
			view->setField(pdal::Dimension::Id::Z, idx, point.getFieldAs<double>(pdal::Dimension::Id::Z));
			
			view->setField(pdal::Dimension::Id::Red, idx, point.getFieldAs<int>(pdal::Dimension::Id::Red));
			view->setField(pdal::Dimension::Id::Green, idx, point.getFieldAs<int>(pdal::Dimension::Id::Green));
			view->setField(pdal::Dimension::Id::Blue, idx, point.getFieldAs<int>(pdal::Dimension::Id::Blue));
			_progress->incrementComplete(1);
			incrementPointsInCell(id, 1);
			_numPoints++;
			idx++;
		}
		else
		{

			// The point didn't pass, so write it to one of the output files.                
			PointWriter* writer= getOrCreateWriter(location);
			if (writer)
			{
				writer->write(point);
			}
			else
			{
				_progress->incrementComplete(1);
			}
		}		
		return true;
	};
	callbackFilter.setCallback(cb);

	FixedPointTable fixed(1000);
	{PDAL_LOCK; callbackFilter.prepare(fixed); }
	callbackFilter.execute(fixed);	

	BufferReader bufferReader;
	bufferReader.addView(view);

	Stage *writer = 0;
	{PDAL_LOCK; writer = _factory.createStage("writers.las"); }

	std::string filename = getFilename(_node->getID(), "laz");
	osgEarth::makeDirectoryForFile(filename);

	Options options;
	options.add("filename", filename);

	writer->setInput(bufferReader);
	writer->setOptions(options);
	{ PDAL_LOCK; writer->prepare(pointTable); }
	writer->execute(pointTable);

	// Destroy the writer stage, we're done with it.
	{PDAL_LOCK; _factory.destroyStage(writer); }
	
	closeChildWriters();
	closeReader();

	// Delete any inputs
	deleteInputs();
}

void OctreeCellBuilder::buildChildren()
{
    // Build any of the child nodes that have an output file.
    for (unsigned int i = 0; i < 8; i++)
    {
        if (!_outputFiles[i].empty())
        {
            osg::ref_ptr< OctreeNode > node = _children[i];                
			osg::ref_ptr< OctreeCellBuilder >  builder = new OctreeCellBuilder;
            // Copy settings from parent
            builder->setTargetNumPoints(_targetNumPoints);
            builder->setInnerLevel(_innerLevel);
            builder->setNode(node.get());
            builder->getInputFiles().push_back(_outputFiles[i]);
            builder->setDeleteInputs(true);
            builder->setSourceSRS(_srcSRS.get());
            builder->setDestSRS(_destSRS.get());
            builder->setGeocentric(_geocentric);
            builder->setOperationQueue(_queue);
            builder->setProgress(_progress);
            builder->setMaxLevel(_maxLevel);
            _queue->add(new BuildCellOperator(builder));
        }
    }        
}

Stage* OctreeCellBuilder::createStageForFile(const std::string& filename) {
	PDAL_LOCK;

	std::string driver;

	if (osgDB::getFileExtension(filename) == "points")
	{
		driver = "readers.points";
	}
	else
	{
		driver = _factory.inferReaderDriver(filename);
	}
	Stage* reader = _factory.createStage(driver);
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

void OctreeCellBuilder::initReader()
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

void OctreeCellBuilder::closeReader()
{     
	PDAL_LOCK;
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

PointWriter* OctreeCellBuilder::getOrCreateWriter(const osg::Vec3d& location)
{
    int child = -1;
    for (unsigned int i = 0; i < 8; i++)
    {
        if (_children[i]->getBoundingBox().contains(location))
        {          
            child = i;
            break;                
        } 
    }

    // We couldn't find a writer, something is wrong.
    if (child < 0)
    {
        return NULL;
    }

    if (!_childWriters[child])
    {
        // Generate a temporay file for the child.
        std::string filename = getFilename(_children[child]->getID(), "points");
        filename = osgEarth::getTempName("", filename);            
        osgEarth::makeDirectoryForFile(filename);

		_childWriters[child] = new PointWriter(filename);
        _outputFiles[child] = filename;
    }
	return _childWriters[child];
}

void OctreeCellBuilder::closeChildWriters()
{
	for (unsigned int i = 0; i < 8; i++)
    {            
        if (_childWriters[i])
        {
            _childWriters[i]->close();            
            _childWriters[i] = 0;
        }
    }
}

unsigned int OctreeCellBuilder::getPointsInCell(const OctreeId& id)
{
    CellCount::iterator itr = _cellCount.find(id);
    if (itr != _cellCount.end())
    {
        return itr->second;
    }
    return 0;
}

void OctreeCellBuilder::incrementPointsInCell(const OctreeId& id, unsigned int count)
{
    unsigned int current = getPointsInCell(id);
    _cellCount[id] = current + count;

    // Increment the total number of points in this octree node.
    _numPoints += count;
}

void OctreeCellBuilder::deleteInputs()
{
    if (_deleteInputs)
    {
        for (unsigned int i = 0; i < _inputFiles.size(); i++)
        {
            unlink(_inputFiles[i].c_str());
        }
    }
}