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
#include <osg/BoundingBox>
#include <iostream>

using namespace osgEarth;
using namespace osgJuniper;

typedef std::map<OctreeId, LASwriter*> WriterMap;

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


int main(int argc, char** argv)
{    
    osg::Timer_t startTime = osg::Timer::instance()->tick();
    std::vector< std::string > filenames;
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

    unsigned int level = 3;

    while (reader->read_point())
    {
        *point = reader->point;    
        osg::Vec3d location((point->X * reader->header.x_scale_factor) + reader->header.x_offset,
                            (point->Y * reader->header.y_scale_factor) + reader->header.y_offset,
                            (point->Z * reader->header.z_scale_factor) + reader->header.z_offset);
        OctreeId id = root.getID(location, level);
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

    reader->close();
    delete reader;

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



    osg::Timer_t endTime = osg::Timer::instance()->tick();

    OSG_NOTICE << "Processed " << complete << " points in " << osgEarth::prettyPrintTime(osg::Timer::instance()->delta_s(startTime, endTime)) << std::endl;
}
