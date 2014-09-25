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
#include "lasreader.hpp"
#include "laswriter.hpp"
#include <osgJuniper/Octree>
#include <osgEarth/Profile>
#include <osgEarth/StringUtils>
#include <osgEarth/FileUtils>
#include <osg/BoundingBox>
#include <osg/OperationThread>
#include <osgDB/FileUtils>
#include <osgDB/FileNameUtils>
#include <osg/ArgumentParser>
#include <iostream>
#include <osgJuniper/Utils>
#include <osgEarth/Random>

using namespace osgEarth;
using namespace osgJuniper;

std::string getFilename(OctreeId id)
{
    std::stringstream buf;
    buf << "tile_" << id.level << "_" << id.z << "_" << id.x << "_" << id.y << ".laz";    
    return buf.str();
}

osg::ref_ptr< osg::OperationQueue > queue = new osg::OperationQueue();

std::vector< osg::ref_ptr< osg::OperationsThread > > threads;


class Progress
{
public:
    Progress():
      _complete(0),
          _total(0)
      {
      }

      unsigned int getTotal()
      {
          OpenThreads::ScopedLock<OpenThreads::Mutex> lock(_mutex);
          return _total;
      }

      void setTotal(unsigned int total)
      {
          OpenThreads::ScopedLock<OpenThreads::Mutex> lock(_mutex);
          _total = total;
      }

      unsigned int getComplete()
      {
          OpenThreads::ScopedLock<OpenThreads::Mutex> lock(_mutex);
          return _complete;
      }

      bool isComplete()
      {
          OpenThreads::ScopedLock<OpenThreads::Mutex> lock(_mutex);
          return _complete == _total;
      }

      void incrementComplete(unsigned int complete)
      {
          OpenThreads::ScopedLock<OpenThreads::Mutex> lock(_mutex);
          _complete += complete;

          if (_complete % 5000 == 0)
          {
              OSG_NOTICE << "Finished " << _complete << " of " << _total << ". " << getPercentComplete()<< "% complete" << std::endl;
          }        
      }

      float getPercentComplete()
      {        
          return ((float)_complete / (float)_total) * 100.0;
      }



private:
    unsigned int _total;
    unsigned int _complete;
    OpenThreads::Mutex _mutex;
};

static Progress s_progress;


/**************************************************/
/**
* Class used to build the output of an octree from a series of input files.
*/
class OctreeCellBuilder
{
public:
    OctreeCellBuilder();

    ~OctreeCellBuilder();

    unsigned int getNumPoints() const;

    OctreeNode* getNode() const;

    void setNode( OctreeNode *node );

    unsigned int getInnerLevel() const;

    void setInnerLevel(unsigned int innerLevel);

    std::vector<std::string>& getInputFiles();

    std::vector<std::string>& getOutputFiles();

    bool getDeleteInputs() const;

    void setDeleteInputs(bool deleteInputs);

    unsigned int getTargetNumPoints() const;

    void setTargetNumPoints(unsigned int targetNumPoints);

    osgEarth::SpatialReference* getSourceSRS() const;
    void setSourceSRS( osgEarth::SpatialReference* srs);

    osgEarth::SpatialReference* getDestSRS() const;
    void setDestSRS( osgEarth::SpatialReference* srs);

    bool getGeocentric() const;
    void setGeocentric(bool geocentric);

    void build();


    void buildChildren();

    void initWriter();

    void initReader();

    void closeReader();

    LASwriter* getOrCreateWriter(const osg::Vec3d& location);

    float getFraction() const;
    void setFraction(float fraction);

    bool keep();


    void closeChildWriters();

    void closeWriter();

    unsigned int getPointsInCell(const OctreeId& id);

    void incrementPointsInCell(const OctreeId& id, unsigned int count=1);

    void deleteInputs();

    osg::Vec3d reprojectPoint(const osg::Vec3d& input);

private:
    LASreader* _reader;

    LASwriter* _writer;
    LASheader* _writeHeader;
    LASquantizer* _writeQuantizer;


    osg::ref_ptr< OctreeNode > _node;
    unsigned int _innerLevel;
    unsigned int _targetNumPoints;
    std::vector<std::string> _inputFiles;
    std::vector<std::string> _outputFiles;

    typedef std::map<OctreeId, unsigned int> CellCount;
    CellCount _cellCount;
    unsigned int _numPoints;

    std::vector<LASwriter*> _childWriters;
    std::vector<osg::ref_ptr<OctreeNode>> _children;

    bool _deleteInputs;

    float _fraction;

    osg::ref_ptr< osgEarth::SpatialReference > _srcSRS;
    osg::ref_ptr< osgEarth::SpatialReference > _destSRS;
    bool _geocentric;

};
/**************************************************/

class BuildCellOperator : public osg::Operation
{
public:
    BuildCellOperator(const OctreeCellBuilder& builder):
      _builder(builder)
      {
      }

      void operator()(osg::Object* object)
      {
          _builder.build();
          _builder.buildChildren();
      }

      OctreeCellBuilder _builder;
};


OctreeCellBuilder::OctreeCellBuilder():
_innerLevel(6),
    _targetNumPoints(0),
    _writer(0),
    _reader(0),
    _writeHeader(0),
    _writeQuantizer(0),
    _numPoints(0),
    _deleteInputs(false),
    _fraction(1.0),
    _geocentric(false)
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

    if (_fraction >= 1.0) return true;
    if (_fraction <= 0.0) return false;

    //double f = random.next();

    float f = (float)rand()/(float)RAND_MAX;
    //OSG_NOTICE << "f=" << f << " fraction=" << _fraction << std::endl;
    return f <= _fraction;    
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


void OctreeCellBuilder::build()
{        
    initReader();

    // If we aren't given a node, assume we are the root
    if (!_node.valid())
    {
        osg::BoundingBoxd bounds(_reader->header.min_x, _reader->header.min_y,_reader->header.min_z,
                                 _reader->header.max_x, _reader->header.max_y, _reader->header.max_z);
        double width = bounds.xMax() - bounds.xMin();
        double height = bounds.zMax() -bounds.zMin();
        double depth = bounds.yMax() - bounds.yMin();
        double max = osg::maximum(osg::maximum(width, height), depth);
        bounds.xMax() = bounds.xMin() + max;
        bounds.yMax() = bounds.yMin() + max;
        bounds.zMax() = bounds.zMin() + max;
        _node = new OctreeNode();
        _node->setBoundingBox(bounds);
        /*
        OSG_NOTICE << "BoundingBox " << bounds.xMin() << ", " << bounds.yMin() << ", " << bounds.zMin() << std::endl
                                     << bounds.xMax() << ", " << bounds.yMax() << ", " << bounds.zMax() << std::endl;
                                     */

    }

    //OSG_NOTICE << "Building cell " << _node->getID().level << ": " << _node->getID().x << ", " << _node->getID().y << ", " << _node->getID().z << "  with " << _reader->header.number_of_point_records << std::endl;

    unsigned int numAdded = 0;

    // Initialize the main writer for this cell.
    initWriter();            

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

    LASpoint* point = new LASpoint;
    //point->init(&_reader->header, _reader->header.point_data_format, _reader->header.point_data_record_length);
    point->init(_writeQuantizer, _reader->header.point_data_format, _reader->header.point_data_record_length);

    unsigned int total = _reader->header.number_of_point_records;            
    unsigned int numRejected = 0;

    // Compute the ratio if we're targetting a certain number of points.
    float fraction = (float)_targetNumPoints/(float)total;
    setFraction(fraction);
    //OSG_NOTICE << "Keeping " << _fraction << " of " << total << " points for an output of " << (int)(fraction * (float)total) << " points " << std::endl;
    //OSG_NOTICE << "NumPoints=" << _numPoints << " total=" << total << " target=" << _targetNumPoints << " fraction=" << _fraction << std::endl;

    // Read all the points
    while (_reader->read_point())
    {
        // Reproject the point
        osg::Vec3d location(_reader->point.get_x(), _reader->point.get_y(), _reader->point.get_z());
        osg::Vec3d world = reprojectPoint(location);

        *point = _reader->point;            

        // Figure out what cell this point should go in.
        OctreeId id = _node->getID(location, _innerLevel);

        // See how many points are currently in this cell
        int count = getPointsInCell(id);

        unsigned int numProcessed = (numAdded + numRejected);
        if (numProcessed % 100000 == 0)
        {
            //OSG_NOTICE << "Processed " << (numAdded + numRejected) << " of " << total << " points. " << (int)(100.0f * (float)numProcessed/(float)total) << "%" << std::endl;
        }        

        if ((_targetNumPoints != 0 && keep()) || count == 0)
        {
            // The point passed, so write it to the output file.
            point->set_x(world.x());
            point->set_y(world.y());
            point->set_z(world.z());
            _writer->write_point(point);
            _writer->update_inventory(point);
            s_progress.incrementComplete(1);
            incrementPointsInCell(id, 1);
            _numPoints++;
            numAdded++;
        }
        else
        {
            // The point didn't pass, so write it to one of the output files.                
            LASwriter* writer = getOrCreateWriter(location);
            if (!writer)
            {             
                point->set_x(world.x());
                point->set_y(world.y());
                point->set_z(world.z());
                _writer->write_point(point);
                _writer->update_inventory(point);                            
                incrementPointsInCell(id, 1);        
                _numPoints++;
                numAdded++;
                s_progress.incrementComplete(1);
            }
            else
            {
                writer->write_point(point);
                writer->update_inventory(point);                            
                numRejected++;
            }                                                
        }
    }        

    delete point;

    closeChildWriters();
    closeReader();

    /*
    OSG_NOTICE << "Points added in pass = " << numAdded << std::endl;
    OSG_NOTICE << "Total number of points " << _numPoints << std::endl;
    OSG_NOTICE << "Remaining points " << numRejected << std::endl;      
    */


    // We keep the main writer open until the bitter end.
    closeWriter();

    // Delete any inputs
    deleteInputs();

    //OSG_NOTICE << _numPoints << " generated for node " << _node->getID().level << ": " << _node->getID().x << ", " << _node->getID().y << ", " << _node->getID().z << std::endl;
}

void OctreeCellBuilder::buildChildren()
{
    // Build any of the child nodes that have an output file.
    for (unsigned int i = 0; i < 8; i++)
    {
        if (!_outputFiles[i].empty())
        {
            osg::ref_ptr< OctreeNode > node = _children[i];                
            OctreeCellBuilder builder;
            // Copy settings from parent
            builder.setTargetNumPoints(_targetNumPoints);
            builder.setInnerLevel(_innerLevel);
            builder.setNode(node.get());         
            builder.getInputFiles().push_back(_outputFiles[i]);
            builder.setDeleteInputs(true);
            builder.setSourceSRS(_srcSRS.get());
            builder.setDestSRS(_destSRS.get());
            builder.setGeocentric(_geocentric);
            queue->add(new BuildCellOperator(builder));
        }
    }        
}

void OctreeCellBuilder::initWriter()
{
    // Only open the writer once.
    if (!_writer)
    {
        _writeHeader = new LASheader();
        *_writeHeader = _reader->header;

        _writeQuantizer = new LASquantizer();
        *_writeQuantizer = *_writeHeader;

        // Generate a new quantizer for the output writer
        if (_srcSRS.valid() && _destSRS.valid())
        {            
            osg::Vec3d midPoint((_reader->header.min_x+_reader->header.max_x)/2.0,
                (_reader->header.min_y+_reader->header.max_y)/2.0,
                (_reader->header.min_z+_reader->header.max_z)/2.0);

            double precision = 0.1;
            if ((_destSRS->isGeodetic() || _destSRS->isGeographic()) && !_geocentric)
            {
                precision = 1e-7;
            }

            osg::Vec3d center = reprojectPoint(midPoint);    
            _writeQuantizer->x_scale_factor = precision;
            _writeQuantizer->y_scale_factor = precision;
            _writeQuantizer->z_scale_factor = precision;
            _writeQuantizer->x_offset = ((I64)((center.x()/_writeQuantizer->x_scale_factor)/10000000))*10000000*_writeQuantizer->x_scale_factor;
            _writeQuantizer->y_offset = ((I64)((center.y()/_writeQuantizer->y_scale_factor)/10000000))*10000000*_writeQuantizer->y_scale_factor;
            _writeQuantizer->z_offset = ((I64)((center.z()/_writeQuantizer->z_scale_factor)/10000000))*10000000*_writeQuantizer->z_scale_factor;

            *_writeHeader = *_writeQuantizer;
        }

        LASwriteOpener opener;
        std::string filename = getFilename(_node->getID());            
        opener.set_file_name(filename.c_str());            
        osgDB::makeDirectoryForFile(filename);
        //_writer = opener.open(&_reader->header);
        _writer = opener.open(_writeHeader);
    }
}

void OctreeCellBuilder::initReader()
{        
    // Open up the reader for the input files
    LASreadOpener lasreadopener;
    for (unsigned int i = 0; i < _inputFiles.size(); i++)
    {
        lasreadopener.add_file_name(_inputFiles[i].c_str());
    }

    lasreadopener.set_merged(TRUE);
    lasreadopener.set_populate_header(TRUE);

    _reader = lasreadopener.open();        
}

void OctreeCellBuilder::closeReader()
{        
    _reader->close();
    delete _reader;   
    _reader = 0;
}

LASwriter* OctreeCellBuilder::getOrCreateWriter(const osg::Vec3d& location)
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
        LASwriteOpener opener;
        // Generate a temporay file for the child.
        std::string filename = getFilename(_children[child]->getID());
        filename = getTempName("", filename);            
        osgEarth::makeDirectoryForFile(filename);
        opener.set_file_name(filename.c_str());            
        _childWriters[child] = opener.open(&_reader->header);
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
            _childWriters[i]->update_header(&_reader->header, TRUE);
            _childWriters[i]->close();
            delete _childWriters[i];   
            _childWriters[i] = 0;
        }
    }
}

void OctreeCellBuilder::closeWriter()
{
    if (_writer)
    {
        //_writer->update_header(&_reader->header, TRUE);
        _writer->update_header(_writeHeader, TRUE);
        _writer->close();
        delete _writer;
        _writer = 0;
        /*
        if (_writeHeader) delete _writeHeader;
        if (_writeQuantizer) delete _writeQuantizer;
        */
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
            //OSG_NOTICE << "Deleting " << _inputFiles[i] << std::endl;
            unlink(_inputFiles[i].c_str());
        }
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

    //Read in the filenames to process
    for(int pos=1;pos<arguments.argc();++pos)
    {
        if (!arguments.isOption(pos))
        {
            filenames.push_back( arguments[pos]);
        }
    }

    if (filenames.size() == 0)
    {
        OSG_NOTICE << "Please specify a filename" << std::endl;
        return 1;
    }



    unsigned int targetNumPoints = 0;
    arguments.read("--target", targetNumPoints);

    unsigned int innerLevel = 8;
    arguments.read("--innerLevel", innerLevel);


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


    // Open up all the files to get the total number of points
    // Open up the reader for the input files
    LASreadOpener lasreadopener;
    for (unsigned int i = 0; i < filenames.size(); i++)
    {
        lasreadopener.add_file_name(filenames[i].c_str());
    }
    lasreadopener.set_merged(TRUE);
    lasreadopener.set_populate_header(TRUE);

    LASreader *reader = lasreadopener.open();   
    s_progress.setTotal(reader->header.number_of_point_records);
    reader->close();
    delete reader;

    OSG_NOTICE << "Processing " << s_progress.getTotal() << " points" << std::endl;



    // Initialize the threads
    unsigned int numThreads = OpenThreads::GetNumberOfProcessors();
    arguments.read("--threads", numThreads);

    for (unsigned int i = 0; i < numThreads; i++)
    {
        osg::OperationsThread* thread = new osg::OperationsThread();
        thread->setOperationQueue(queue);
        thread->start();
        threads.push_back(thread);
    }


    OctreeCellBuilder builder;    
    for (unsigned int i = 0; i < filenames.size(); i++)
    {
        builder.getInputFiles().push_back(filenames[i]);
        OSG_NOTICE << "Processing filenames " << filenames[i] << std::endl;
    }
    builder.setInnerLevel(innerLevel);
    builder.setTargetNumPoints(targetNumPoints);
    builder.setSourceSRS(srcSRS.get());
    builder.setDestSRS(destSRS.get());
    builder.setGeocentric(geocentric);
    builder.build();
    builder.buildChildren();    

    // Wait for completion
    while (!s_progress.isComplete())
    {        
        OpenThreads::Thread::microSleep(5 * 1000 * 1000);
    }

    osg::Timer_t endTime = osg::Timer::instance()->tick();



    OSG_NOTICE << "Completed " << s_progress.getTotal() << " in " << osgEarth::prettyPrintTime(osg::Timer::instance()->delta_s(startTime, endTime)) << std::endl;
}
