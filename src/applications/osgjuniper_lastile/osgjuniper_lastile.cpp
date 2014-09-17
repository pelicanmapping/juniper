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

        if (_complete % 10000 == 0)
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

    void build();
    

    void buildChildren();
    
    void initWriter();

    void initReader();

    void closeReader();

    LASwriter* getOrCreateWriter(const osg::Vec3d& location);


    void closeChildWriters();

    void closeWriter();

    unsigned int getPointsInCell(const OctreeId& id);

    void incrementPointsInCell(const OctreeId& id, unsigned int count=1);

    void deleteInputs();

private:
    LASreader* _reader;

    LASwriter* _writer;

    osg::ref_ptr< OctreeNode > _node;
    unsigned int _innerLevel;
    unsigned int _targetNumPoints;
    std::vector<std::string> _inputFiles;
    std::vector<std::string> _outputFiles;

    typedef std::map<OctreeId, unsigned int> CellCount;
    CellCount _cellCount;
    unsigned int _numPoints;
    unsigned int _limit;
    
    std::vector<LASwriter*> _childWriters;
    std::vector<osg::ref_ptr<OctreeNode>> _children;

    bool _deleteInputs;
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
    _targetNumPoints(100000),
    _writer(0),
    _reader(0),
    _numPoints(0),
    _limit(1),
    _deleteInputs(false)
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

void OctreeCellBuilder::build()
{        
    while (true)
    {
        initReader();

        // If we aren't given a node, assume we are the root
        if (!_node.valid())
        {
            osg::BoundingBoxd bounds(_reader->header.min_x, _reader->header.min_y,_reader->header.min_z,
                _reader->header.max_x, _reader->header.max_y, _reader->header.max_z);
            _node = new OctreeNode();
            _node->setBoundingBox(bounds);
        }

        OSG_NOTICE << "Building cell " << _node->getID().level << ": " << _node->getID().x << ", " << _node->getID().y << ", " << _node->getID().z << "  with " << _reader->header.number_of_point_records << " points limit=" << _limit << std::endl;

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
        point->init(&_reader->header, _reader->header.point_data_format, _reader->header.point_data_record_length);

        unsigned int total = _reader->header.number_of_point_records;            
        unsigned int numRejected = 0;


        if (_numPoints + total < _targetNumPoints)            
        {
            OSG_INFO << "Taking all points.  input=" << total << ", target=" << _targetNumPoints << std::endl;
            // If the total number of incoming points is less than the target just take them all.
            while (_reader->read_point())
            {
                *point = _reader->point;   
                _writer->write_point(point);
                _writer->update_inventory(point);
                s_progress.incrementComplete(1);
                _numPoints += 1;
            }                
        }
        else
        {            

            // Read all the points
            while (_reader->read_point())
            {
                *point = _reader->point;    
                osg::Vec3d location((point->X * _reader->header.x_scale_factor) + _reader->header.x_offset,
                    (point->Y * _reader->header.y_scale_factor) + _reader->header.y_offset,
                    (point->Z * _reader->header.z_scale_factor) + _reader->header.z_offset);

                // Figure out what cell this point should go in.
                OctreeId id = _node->getID(location, _innerLevel);

                // See how many points are currently in this cell
                int count = getPointsInCell(id);

                /*
                bool finished = _numPoints >= _targetNumPoints;
                if (_node->getID().level == 0 && _limit == 1)
                {
                finished = false;
                }
                */

                if (/*!finished && */ count < _limit)
                {
                    // The point passed, so write it to the output file.
                    _writer->write_point(point);
                    _writer->update_inventory(point);
                    s_progress.incrementComplete(1);
                    incrementPointsInCell(id, 1);
                    numAdded++;
                }
                else
                {
                    // The point didn't pass, so write it to one of the output files.                
                    LASwriter* writer = getOrCreateWriter(location);
                    if (!writer)
                    {
                        _writer->write_point(point);
                        _writer->update_inventory(point);                            
                        incrementPointsInCell(id, 1);        
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
        }

        closeChildWriters();
        closeReader();

        OSG_INFO << "Points added in pass = " << numAdded << std::endl;
        OSG_INFO << "Total number of points " << _numPoints << std::endl;
        OSG_INFO << "Remaining points " << numRejected << std::endl;

        if (numRejected == 0 || _numPoints >= _targetNumPoints || _limit > 1)
        {
            //OSG_NOTICE << "Quitting" << std::endl;
            break;
        }
        else
        {
            if (_limit == 1)
            {
                unsigned int numCellsWithData = _cellCount.size();
                // For the second pass, do at least a minimum of 2
                _limit = osg::maximum((int)((float)_targetNumPoints / (float)numCellsWithData), 2);
                //OSG_NOTICE << "NumcellsWithData=" << numCellsWithData << " Target=" << _targetNumPoints << " New Limit=" << _limit << std::endl;
            }
            else
            {
                _limit++;
            }
            //OSG_NOTICE << "Doing another pass with limit of " << _limit << std::endl;

            // Delete any old inputs if needed.
            deleteInputs();

            // These new files for this pass will be temporary and should be deleted.
            setDeleteInputs(true);

            // Replace the input files with the output files.                
            _inputFiles.clear();
            for (unsigned int i = 0; i < _outputFiles.size(); i++)
            {
                if (!_outputFiles[i].empty())
                {
                    _inputFiles.push_back(_outputFiles[i]);
                }
            }
        }
    }

    // We keep the main writer open until the bitter end.
    closeWriter();

    // Delete any inputs
    deleteInputs();

    OSG_NOTICE << _numPoints << " generated for node " << _node->getID().level << ": " << _node->getID().x << ", " << _node->getID().y << ", " << _node->getID().z << std::endl;
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
            /*
            builder.build();
            builder.buildChildren();                
            */
            queue->add(new BuildCellOperator(builder));
        }
    }        
}

void OctreeCellBuilder::initWriter()
{
    // Only open the writer once.
    if (!_writer)
    {
        LASwriteOpener opener;
        std::string filename = getFilename(_node->getID());            
        opener.set_file_name(filename.c_str());            
        osgDB::makeDirectoryForFile(filename);
        _writer = opener.open(&_reader->header);
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
        _writer->update_header(&_reader->header, TRUE);
        _writer->close();
        delete _writer;
        _writer = 0;
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



    unsigned int targetNumPoints = 100000;
    arguments.read("--target", targetNumPoints);

    unsigned int innerLevel = 6;
    arguments.read("--innerLevel", innerLevel);


    
    /*
    osgDB::DirectoryContents contents = osgDB::getDirectoryContents("C:/geodata/aam/PointClouds/HongKong2/HongKong_ALS");
    for (unsigned int i = 0; i < contents.size(); i++)
    {
        std::string filename = contents[i];
        if (osgEarth::endsWith(osgDB::getSimpleFileName(filename), "_ALL.laz"))
        {
            filenames.push_back(filename);
            OSG_NOTICE << "Adding " << filename << std::endl;
        }        

        if (filenames.size() == 3)
        {
            break;
        }
    }
    */
    
    /*
    filenames.push_back("C:/geodata/aam/PointClouds/Yarra/pt000001.laz");
    filenames.push_back("C:/geodata/aam/PointClouds/Yarra/pt000002.laz");
    filenames.push_back("C:/geodata/aam/PointClouds/Yarra/pt000003.laz");
    filenames.push_back("C:/geodata/aam/PointClouds/Yarra/pt000004.laz");    
    filenames.push_back("C:/geodata/aam/PointClouds/Yarra/pt000005.laz");
    filenames.push_back("C:/geodata/aam/PointClouds/Yarra/pt000006.laz");
    filenames.push_back("C:/geodata/aam/PointClouds/Yarra/pt000007.laz");
    filenames.push_back("C:/geodata/aam/PointClouds/Yarra/pt000008.laz");
    filenames.push_back("C:/geodata/aam/PointClouds/Yarra/pt000009.laz");
    filenames.push_back("C:/geodata/aam/PointClouds/Yarra/pt000010.laz");
    filenames.push_back("C:/geodata/aam/PointClouds/Yarra/pt000011.laz");
    filenames.push_back("C:/geodata/aam/PointClouds/Yarra/pt000012.laz");    
    filenames.push_back("C:/geodata/aam/PointClouds/Yarra/pt000013.laz");
    filenames.push_back("C:/geodata/aam/PointClouds/Yarra/pt000014.laz");
    filenames.push_back("C:/geodata/aam/PointClouds/Yarra/pt000015.laz");
    filenames.push_back("C:/geodata/aam/PointClouds/Yarra/pt000016.laz");
    filenames.push_back("C:/geodata/aam/PointClouds/Yarra/pt000017.laz");
    filenames.push_back("C:/geodata/aam/PointClouds/Yarra/pt000018.laz");
    filenames.push_back("C:/geodata/aam/PointClouds/Yarra/pt000019.laz");
    filenames.push_back("C:/geodata/aam/PointClouds/Yarra/pt000020.laz");
    filenames.push_back("C:/geodata/aam/PointClouds/Yarra/pt000021.laz");
    filenames.push_back("C:/geodata/aam/PointClouds/Yarra/pt000022.laz");
    filenames.push_back("C:/geodata/aam/PointClouds/Yarra/pt000023.laz");
    filenames.push_back("C:/geodata/aam/PointClouds/Yarra/pt000024.laz");
    filenames.push_back("C:/geodata/aam/PointClouds/Yarra/pt000025.laz");
    filenames.push_back("C:/geodata/aam/PointClouds/Yarra/pt000026.laz");
    filenames.push_back("C:/geodata/aam/PointClouds/Yarra/pt000027.laz");
    filenames.push_back("C:/geodata/aam/PointClouds/Yarra/pt000028.laz");
    filenames.push_back("C:/geodata/aam/PointClouds/Yarra/pt000029.laz"); 
    */


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
