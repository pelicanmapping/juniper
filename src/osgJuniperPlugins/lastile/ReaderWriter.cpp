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

        osgDB::Registry::instance()->addFileExtensionAlias("laz", "las");
    }

    virtual const char* className()
    {
        return "LAS Tiled Point Reader";
    }

    std::string getFilename(OctreeId id, const std::string& ext) const
    {
        std::stringstream buf;
        buf << "tile_" << id.level << "_" << id.z << "_" << id.x << "_" << id.y << "." << ext;    
        return buf.str();
    }


    virtual ReadResult readNode( const std::string& location, const osgDB::ReaderWriter::Options* options ) const
    {
        if ( !acceptsExtension( osgDB::getLowerCaseFileExtension( location ) ) )
            return ReadResult::FILE_NOT_HANDLED;

        std::string file = osgDB::getNameLessExtension( location );        
        osg::Node* node = osgDB::readNodeFile( file );
        if (!node)
        {
            return ReadResult::FILE_NOT_FOUND;
        }

        std::string inExt = osgDB::getFileExtension(file);

        osg::PagedLOD* plod = new osg::PagedLOD;
        osg::Vec3d center = node->getBound().center();
        plod->setRadius(node->getBound().radius());
        plod->setCenter(center);
        plod->addChild(node);
        plod->setRange(0,0,FLT_MAX);

        double radiusFactor = 2.5;
        std::string ext = "laz";
        
        if (options && !options->getOptionString().empty())
        {
            std::string radiusFactorStr = options->getPluginStringData("radiusFactor");        
            if (!radiusFactorStr.empty())
            {
                std::istringstream iss(radiusFactorStr);
                iss >> radiusFactor;
            }

            std::string outExt = options->getPluginStringData("ext");
            if (!outExt.empty())
            {
                ext = outExt;
            }
        }
               

        std::string path = osgDB::getFilePath(file);

        // Get the octree tile name.        
        std::string tileID = osgDB::getNameLessExtension(osgDB::getSimpleFileName(file));
        unsigned int level, x, y, z;
        sscanf(tileID.c_str(), "tile_%d_%d_%d_%d", &level, &z, &x, &y);        

        OctreeId id(level, x, y, z);
        osg::ref_ptr< OctreeNode > octree = new OctreeNode();
        octree->setId(id);               
        unsigned int childNum = 1;

        
        double childRadius = node->getBound().radius() / 2.0;        
        for (unsigned int i = 0; i < 8; ++i)
        {
            osg::ref_ptr< OctreeNode > child = octree->createChild(i);
            std::string childFilename = osgDB::concatPaths(path, getFilename(child->getID(), inExt));            

            if (osgDB::fileExists(childFilename) || osgDB::containsServerAddress(childFilename))
            {
                //std::string outFilename = osgDB::concatPaths(path, getFilename(child->getID(), ext));
                std::stringstream buf;
                buf << path;
                if (!path.empty())
                {
                    buf << "/";
                }
                buf << getFilename(child->getID(), ext);
                std::string outFilename = buf.str();                
                if (ext == "las" || ext == "laz")
                {
                    outFilename += ".lastile";            
                }                
                plod->setFileName(childNum, outFilename);
                plod->setRange(childNum, 0, childRadius * radiusFactor);
                childNum++;                
            }            
        }        

        // If this is the root node go ahead and add a PointCloud decorator to make it look nice.
        if (id.level == 0 && id.x == 0 && id.y == 0 && id.z == 0)
        {
            PointCloudDecorator *decorator = new PointCloudDecorator();
            decorator->addChild(plod);
            return decorator;
        }
        else
        {
            // Just return the PagedLOD.
            return plod;
        }
    }
};

REGISTER_OSGPLUGIN(lastile, LASTileReaderWriter)

