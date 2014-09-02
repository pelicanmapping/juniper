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
#include <iostream>

using namespace osgEarth;
using namespace osgJuniper;

typedef std::map<OctreeId, LASreader*> ReaderMap;
typedef std::map<OctreeId, LASwriter*> WriterMap;

typedef std::vector<OctreeId> NodeIdList;

typedef std::set<OctreeId> NodeSet;

NodeSet nodes;

WriterMap writers;

std::string getFilename(OctreeId id)
{
    std::stringstream buf;
    buf << "tile_" << id.level << "_" << id.z << "_" << id.x << "_" << id.y << ".laz";    
    return buf.str();
}

LASwriter* getOrCreateWriter(OctreeId id, LASheader* header)
{
    WriterMap::iterator itr = writers.find(id);
    if (itr != writers.end() && itr->second)
    {        
        return itr->second;
    }    

    LASwriteOpener laswriteopener;
    
    std::string filename = getFilename(id);
    laswriteopener.set_file_name(filename.c_str());
    LASwriter* writer = laswriteopener.open(header);
    writers[id] = writer;
    return writer;
}

bool split(OctreeNode* node, unsigned int target)
{
    LASreadOpener opener;
    std::string filename = getFilename(node->getID());
    opener.set_file_name(filename.c_str());

    bool split = false;

    LASreader* reader = opener.open();
    if (reader->header.number_of_point_records > target)
    {
        OSG_NOTICE << "Splitting " << filename << " with " << reader->header.number_of_point_records << std::endl;
        std::vector< osg::ref_ptr< OctreeNode> > children;
        std::vector< LASwriter* > childWriters;
        for (unsigned int i = 0; i < 8; i++)
        {
            // Initial writer is null.  It's opened on demand.
            childWriters.push_back(0);
            OctreeNode* child = node->createChild(i);
            children.push_back(child);            
        }             

        LASpoint* point = new LASpoint;
        point->init(&reader->header, reader->header.point_data_format, reader->header.point_data_record_length);

        // Read all the points and determine which cell it goes in.
        while (reader->read_point())
        {
            *point = reader->point;    
            osg::Vec3d location((point->X * reader->header.x_scale_factor) + reader->header.x_offset,
                (point->Y * reader->header.y_scale_factor) + reader->header.y_offset,
                (point->Z * reader->header.z_scale_factor) + reader->header.z_offset);

            bool wrotePoint = false;
            // Now loop over the cells and see which one it fits in
            for (unsigned int i = 0; i < 8; i++)
            {
                if (children[i]->getBoundingBox().contains(location))
                {                       
                    LASwriter* writer = childWriters[i];
                    if (!writer)
                    {                        
                        LASwriteOpener writeOpener;
                        std::string childFilename = getFilename(children[i]->getID());
                        writeOpener.set_file_name(childFilename.c_str());        
                        writer = writeOpener.open(&reader->header);            
                        childWriters[i] = writer;
                    }
                    writer->write_point(point);                    
                    writer->update_inventory(point);                    
                    wrotePoint = true;
                    break;
                }                
            }
            if (!wrotePoint)
            {
                OSG_NOTICE << "Failed to write point!" << std::endl;
            }            
        }    

        delete point;

        // Close all the writers
        for (unsigned int i = 0; i < 8; i++)
        {
            if (childWriters[i])
            {
                childWriters[i]->update_header(&reader->header, TRUE);
                childWriters[i]->close();            
                delete childWriters[i];            
            }
        }
        split = true;
    }
    else
    {
        OSG_NOTICE << "Don't need to split " << filename << " with " << reader->header.number_of_point_records << std::endl;
        split = false;
    }
    reader->close();
    delete reader;

    if (split)
    {
        // Erase the leaf node from the map
        WriterMap::iterator itr = writers.find(node->getID());
        if (itr != writers.end())
        {
            writers.erase(itr);
            unlink(filename.c_str());
        }
    }
    return split;
}

void merge(OctreeId parent, const NodeIdList& leaves, LASheader* header, unsigned int target)
{
    // Given an OctreeNode, collect all the leaf nodes from the children and take points from them until this node
    // has a target number of points or the children have no more points.
    OSG_NOTICE << "Merging " << parent.level << ": " << parent.x << ", " << parent.y << ", " << parent.z << " with " << leaves.size() << " children" << std::endl;

    // Open the writer for this tile.
    LASwriteOpener writeOpener;
    std::string outFilename = getFilename(parent);
    writeOpener.set_file_name(outFilename.c_str());
    LASwriter* writer = writeOpener.open(header);

    std::vector< LASreader* > readers;
    readers.reserve(leaves.size());
    std::vector< unsigned int > remainingPoints;
    remainingPoints.reserve(leaves.size());

    // Initialize all the readers and the number of points they have in them.
    for (unsigned int i = 0; i < leaves.size(); i++)
    {
        LASreadOpener readOpener;
        std::string filename = getFilename(leaves[i]);
        readOpener.set_file_name(filename.c_str());
        LASreader* reader = readOpener.open();
        readers.push_back(reader);
        remainingPoints.push_back(reader->header.number_of_point_records);
    }

    LASpoint* point = new LASpoint;
    point->init(header, header->point_data_format, header->point_data_record_length);

    unsigned int totalPoints = 0;
    while (totalPoints < target)
    {
        // Go through and grab a point from each of the children that actually has data to write to the parent.
        bool gotPoint = false;
        for (unsigned int i = 0; i < leaves.size(); i++)
        {            
            if (remainingPoints[i] > 0)
            {
                LASreader* reader = readers[i];
                // Select a random point from the file                         
                /*
                if (i % 3 == 0)
                {
                    I64 random = ((float)rand()/(float)RAND_MAX) * (float)reader->header.number_of_point_records;
                    reader->seek(random);
                }
                */
                if (reader->read_point())
                {
                    *point = reader->point;    

                    writer->write_point(point);
                    writer->update_inventory(point);

                    // Decrement the remianing points from this cell.
                    remainingPoints[i]--;
                    
                    gotPoint = true;
                    totalPoints++;
                }
            }
        }        
        if (!gotPoint)
        {
            break;
        }
    }
    OSG_NOTICE << "Total points for cell" << parent.level << ", " << parent.x << ", " << parent.y << ", " << parent.z << " = " << totalPoints << std::endl;
    
    // Close the writer
    writer->update_header(header, TRUE);
    writer->close();
    delete writer;

    //TODO:  I suppose you need to remove the points that were removed as well from disk.

    // Remove any empty children from disk as well as from the node set.
    for (unsigned int i = 0; i < leaves.size(); i++)
    {
        readers[i]->close();
        delete readers[i];

        if (remainingPoints[i] == 0)
        {            
            nodes.erase(leaves[i]);
            std::string filename = getFilename(leaves[i]);
            OSG_NOTICE << "Removing empty leaf " << filename << std::endl;
            unlink(filename.c_str());
        }
    }
}


/**
 * Class used to build the output of an octree from a series of input files.
 */
class OctreeCellBuilder
{
public:
    OctreeCellBuilder():
      _innerLevel(8),
      _targetNumPoints(100000),
      _writer(0),
      _reader(0),
      _numPoints(0),
      _limit(1),
      _deleteInputs(false)
    {
    }

    ~OctreeCellBuilder()
    {                 
    }

    unsigned int getNumPoints() const
    {
        return _numPoints;
    }

    OctreeNode* getNode() const
    {
        return _node;
    }

    void setNode( OctreeNode *node )
    {
        _node = node;
    }

    unsigned int getInnerLevel() const
    {
        return _innerLevel;
    }

    void setInnerLevel(unsigned int innerLevel)
    {
        _innerLevel = innerLevel;
    }

    std::vector<std::string>& getInputFiles()
    {
        return _inputFiles;
    }

    std::vector<std::string>& getOutputFiles()
    {
        return _outputFiles;
    } 

    bool getDeleteInputs() const
    {
        return _deleteInputs;
    }

    void setDeleteInputs(bool deleteInputs)
    {
        _deleteInputs = deleteInputs;
    }

    void build()
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
            unsigned int complete = 0;        
            unsigned int numRejected = 0;


            if (total < _targetNumPoints)
            {
                OSG_NOTICE << "Taking all points.  input=" << total << ", target=" << _targetNumPoints << std::endl;
                // If the total number of incoming points is less than the target just take them all.
                while (_reader->read_point())
                {
                    _writer->write_point(point);
                    _writer->update_inventory(point);
                }
                break;
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
                    OctreeId id = _node->getID(location, _innerLevel);
                    int count = getPointsInCell(id);
                    if (count < _limit && _numPoints < _targetNumPoints)
                    {
                        // The point passed, so write it to the output file.
                        _writer->write_point(point);
                        _writer->update_inventory(point);
                        incrementPointsInCell(id, 1);

                        // Quit if we've already reached our target number.
                        // We can't really do this early exit if we want a nice spatially sorted dataset.  Really need to think
                        // about the relationship between the inner spatial grid and the taget # of points.                    
                        /*
                        if (_numPoints >= _targetNumPoints)
                        {
                        OSG_NOTICE << "Reached target number of " << _numPoints << ", quitting early" << std::endl;
                        break;
                        } 
                        */
                    }
                    else
                    {
                        // The point didn't pass, so write it to one of the output files.                
                        LASwriter* writer = getOrCreateWriter(location);
                        if (!writer)
                        {
                            OSG_NOTICE << "Failed to get writer" << std::endl;
                        }
                        else
                        {
                            writer->write_point(point);
                            writer->update_inventory(point);
                        }
                        numRejected++;
                    }

                    complete++;
                    /*
                    if (complete % 10000 == 0)
                    {
                    double percentComplete = ((double)complete / (double)total) * 100.0;
                    std::cout << "Completed " << complete << " of " << total << " points. " << (int)percentComplete << "% complete" << std::endl;
                    }
                    */
                }        
            }

            closeChildWriters();
            closeReader();

            if (!numRejected || _numPoints >= _targetNumPoints)
            {
                //OSG_NOTICE << "Quitting" << std::endl;
                break;
            }
            else
            {
                _limit++;                
                //OSG_NOTICE << "Total number of points " << _numPoints << std::endl;
                //OSG_NOTICE << "Remaining points " << numRejected << std::endl;
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

                //OSG_NOTICE << "Doing another pass with " << _inputFiles.size() << std::endl;
            }
        }

        // We keep the main writer open until the bitter end.
        closeWriter();

        // Delete any inputs
        deleteInputs();

        OSG_NOTICE << _numPoints << " generated for node " << _node->getID().level << ": " << _node->getID().x << ", " << _node->getID().y << ", " << _node->getID().z << std::endl;
    }

    void buildChildren()
    {
        OSG_NOTICE << "Calling buildChildren with parent " << _node->getID().level << ": " << _node->getID().x << ", " << _node->getID().y << ", " << _node->getID().z << std::endl;
        // Build any of the child nodes that have an output file
        for (unsigned int i = 0; i < 8; i++)
        {
            if (!_outputFiles[i].empty())
            {
                osg::ref_ptr< OctreeNode > node = _children[i];                
                OctreeCellBuilder builder;
                builder.setNode(node.get());         
                builder.getInputFiles().push_back(_outputFiles[i]);
                builder.setDeleteInputs(true);
                OSG_NOTICE << "Calling buildChildren for child " << i << " " << node->getID().level << ": " << node->getID().x << ", " << node->getID().y << ", " << node->getID().z << std::endl;
                builder.build();
                builder.buildChildren();                
            }
        }        
    }

    void initWriter()
    {
        // Only open the writer once.
        if (!_writer)
        {
            LASwriteOpener opener;
            std::string filename = getFilename(_node->getID());            
            opener.set_file_name(filename.c_str());            
            _writer = opener.open(&_reader->header);
        }
    }

    void initReader()
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

    void closeReader()
    {        
        _reader->close();
        delete _reader;   
        _reader = 0;
    }

    LASwriter* getOrCreateWriter(const osg::Vec3d& location)
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
            opener.set_file_name(filename.c_str());
            _childWriters[child] = opener.open(&_reader->header);
            _outputFiles[child] = filename;
        }
        return _childWriters[child];
    }

    void closeChildWriters()
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

    void closeWriter()
    {
        if (_writer)
        {
            _writer->update_header(&_reader->header, TRUE);
            _writer->close();
            delete _writer;
            _writer = 0;
        }
    }

    unsigned int getPointsInCell(const OctreeId& id)
    {
        CellCount::iterator itr = _cellCount.find(id);
        if (itr != _cellCount.end())
        {
            return itr->second;
        }
        return 0;
    }

    void incrementPointsInCell(const OctreeId& id, unsigned int count=1)
    {
        unsigned int current = getPointsInCell(id);
        _cellCount[id] = current + count;

        // Increment the total number of points in this octree node.
        _numPoints += count;
    }

    void deleteInputs()
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


int main(int argc, char** argv)
{    
    osg::Timer_t startTime = osg::Timer::instance()->tick();
    std::vector< std::string > filenames;
    filenames.push_back("C:/geodata/aam/PointClouds/Yarra/pt000001.laz");
    /*
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
    OctreeCellBuilder builder;    
    for (unsigned int i = 0; i < filenames.size(); i++)
    {
        builder.getInputFiles().push_back(filenames[i]);
    }
    builder.build();
    builder.buildChildren();




#if 0
    LASreadOpener lasreadopener;
    for (unsigned int i = 0; i < filenames.size(); i++)
    {
        lasreadopener.add_file_name(filenames[i].c_str());
    }

    lasreadopener.set_merged(TRUE);
    lasreadopener.set_populate_header(TRUE);

    LASreader* reader = lasreadopener.open();
    osg::BoundingBoxd bounds(reader->header.min_x, reader->header.min_y,reader->header.min_z,
                             reader->header.max_x, reader->header.max_y, reader->header.max_z);


    OctreeNode root;
    root.setBoundingBox(bounds);    

    LASpoint* point = new LASpoint;
    point->init(&reader->header, reader->header.point_data_format, reader->header.point_data_record_length);

    

    unsigned int total = reader->header.number_of_point_records;
    unsigned int complete = 0;

    unsigned int leafLevel = 3;

    unsigned int targetNumPoints = 100000;

    while (reader->read_point())
    {
        *point = reader->point;    
        osg::Vec3d location((point->X * reader->header.x_scale_factor) + reader->header.x_offset,
                            (point->Y * reader->header.y_scale_factor) + reader->header.y_offset,
                            (point->Z * reader->header.z_scale_factor) + reader->header.z_offset);
        OctreeId id = root.getID(location, leafLevel);
        LASwriter* writer = getOrCreateWriter(id, &reader->header);
        writer->write_point(point);
        writer->update_inventory(point);
        complete++;
        if (complete % 10000 == 0)
        {
            double percentComplete = ((double)complete / (double)total) * 100.0;
            std::cout << "Completed " << complete << " of " << total << " points. " << (int)percentComplete << "% complete" << std::endl;
        }
    }    
    
    std::cout << "Closing " << writers.size() << " writers" << std::endl;

    // Delete all the writers
    for (WriterMap::iterator itr = writers.begin(); itr != writers.end(); itr++)
    {
        itr->second->update_header(&reader->header, TRUE);
        itr->second->close();
        delete itr->second;
        itr->second = NULL;
    }

    
    // Now we have all the leaf nodes at level 3.  Generate all the nodes up to level 0 and stick them in the parent list.    
    for (WriterMap::iterator itr = writers.begin(); itr != writers.end(); itr++)
    {
        OSG_NOTICE << "Processing " << itr->first.level << ": " << itr->first.x << ", " << itr->first.y << ", " << itr->first.z << std::endl;
        // Insert the leaf
        nodes.insert(itr->first);
        
        OctreeId parentId = itr->first;
        while (parentId.level != 0)
        {            
            osg::ref_ptr< OctreeNode > n = root.createChild(parentId);
            parentId = n->getParentID();
            OSG_NOTICE << "Inserting " << parentId.level << ", " << parentId.x << ", " << parentId.y << ", " << parentId.z << std::endl;
            nodes.insert(parentId);
        }        
    }

    OSG_NOTICE << "You now have " << nodes.size() << " nodes to deal with " << std::endl;

    for (unsigned int level = 0; level < leafLevel; level++)
    {       
        NodeIdList parents;

        // Collect all the leaves and parents
        for (NodeSet::iterator itr = nodes.begin(); itr != nodes.end(); itr++)
        {
            if (itr->level == level)
            {
                parents.push_back(*itr);
            }
        }    

        for (NodeIdList::iterator itr = parents.begin(); itr != parents.end(); itr++)
        {
            NodeIdList leaves;
            OctreeId parentId = *itr;
            // Collect any leaf nodes that belong to this parent.            
            for (NodeSet::iterator leafItr = nodes.begin(); leafItr != nodes.end(); leafItr++)
            {
                if (leafItr->level == leafLevel)
                { 
                    // Determine if this leaf is a child of the given parent
                    bool isChild = false;
                    osg::ref_ptr< OctreeNode > childNode = root.createChild(*leafItr);
                    while (childNode->getID().level >= level)
                    {
                        if (childNode->getID() == parentId)
                        {
                            isChild = true;                            
                            break;
                        }
                        childNode = root.createChild(childNode->getParentID());
                    }
                    if (isChild)
                    {
                        leaves.push_back(*leafItr);
                    }
                }                
            }    

            //Now process the parent
            if (leaves.size() > 0)
            {
                merge(parentId, leaves, &reader->header, targetNumPoints);
            }
            else
            {
                OSG_NOTICE << "Removing empty parent " << parentId.level << ", " << parentId.x << ", " << parentId.y << ", " << parentId.z << std::endl;
                nodes.erase(parentId);
            }
        }



        OSG_NOTICE << "Processing parent tiles with level " << level << std::endl;

    }


    reader->close();
    delete reader;


    /*
    unsigned int target = 100000;

    bool keepSplitting = true;

    while(keepSplitting)
    {
        keepSplitting = false;

        // Collect any current ids.  We copy it to a vector so we can manipulator the writers list in split
        std::vector< OctreeId > ids;
        ids.reserve(writers.size());
        for (WriterMap::iterator itr = writers.begin(); itr != writers.end(); itr++)
        {
            ids.push_back(itr->first);
        }

        // Split any files that might need it b/c they are too large.
        //for (WriterMap::iterator itr = writers.begin(); itr != writers.end(); itr++)
        for (unsigned int i = 0; i < ids.size(); ++i)
        {
            osg::ref_ptr< OctreeNode > node = root.createChild(ids[i]);

            bool keepGoing = split(node.get(), target);
            if (keepGoing)
            {
                keepSplitting = true;
            }
        }
    }
    */

#endif



    osg::Timer_t endTime = osg::Timer::instance()->tick();

    OSG_NOTICE << "Completed in " << osgEarth::prettyPrintTime(osg::Timer::instance()->delta_s(startTime, endTime)) << std::endl;
}
