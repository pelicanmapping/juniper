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
#include <osg/PagedLOD>
#include <osgDB/FileNameUtils>
#include <osgDB/Registry>
#include <osgDB/ReaderWriter>
#include <osgDB/FileUtils>
#include <osgDB/ReadFile>
#include <osgJuniper/PointCloud>
#include <osgJuniper/Octree>

using namespace osgJuniper;

class LASTileReaderWriter : public osgDB::ReaderWriter
{
public:
    LASTileReaderWriter()
    {
        supportsExtension( "lastile", className() );
    }

    virtual const char* className()
    {
        return "LAS Tiled Point Reader";
    }

    std::string getFilename(OctreeId id) const
    {
        std::stringstream buf;
        buf << "tile_" << id.level << "_" << id.z << "_" << id.x << "_" << id.y << ".laz";    
        return buf.str();
    }


    virtual ReadResult readNode( const std::string& location, const osgDB::ReaderWriter::Options* options ) const
    {
        if ( !acceptsExtension( osgDB::getLowerCaseFileExtension( location ) ) )
            return ReadResult::FILE_NOT_HANDLED;

        std::string file = osgDB::getNameLessExtension( location );
        osg::Node* node = osgDB::readNodeFile( file );
        if (!node) return ReadResult::FILE_NOT_FOUND;

        osg::PagedLOD* plod = new osg::PagedLOD;
        osg::Vec3d center = node->getBound().center();
        plod->setRadius(node->getBound().radius());
        plod->setCenter(center);
        plod->addChild(node);
        plod->setRange(0,0,FLT_MAX);

        std::string path = osgDB::getFilePath(file);

        // Get the octree tile name.        
        std::string tileID = osgDB::getNameLessExtension(osgDB::getSimpleFileName(file));
        unsigned int level, x, y, z;
        sscanf(tileID.c_str(), "tile_%d_%d_%d_%d", &level, &z, &x, &y);

        double radiusFactor = 5;

        OctreeId id(level, x, y, z);
        osg::ref_ptr< OctreeNode > octree = new OctreeNode();
        octree->setId(id);               
        unsigned int childNum = 1;

        double childRadius = node->getBound().radius() / 2.0;        
        for (unsigned int i = 0; i < 8; ++i)
        {
            osg::ref_ptr< OctreeNode > child = octree->createChild(i);
            std::string childFilename = osgDB::concatPaths(path, getFilename(child->getID()));            

            if (osgDB::fileExists(childFilename))
            {
                childFilename += ".lastile";            
                plod->setFileName(childNum, childFilename);
                plod->setRange(childNum, 0, childRadius * radiusFactor);
                childNum++;
            }            
        }        

        return plod;
    }
};

REGISTER_OSGPLUGIN(lastile, LASTileReaderWriter)

