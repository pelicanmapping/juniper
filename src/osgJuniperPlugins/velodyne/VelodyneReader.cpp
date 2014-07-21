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
#include <osg/Geometry>
#include <osg/Geode>
#include <osg/Point>
#include <osg/LOD>
#include <osg/ShapeDrawable>
#include <osgDB/Registry>
#include <osgDB/FileNameUtils>
#include <osgDB/ReaderWriter>
#include <fstream>

#include <osgJuniper/Utils>
#include <osgJuniper/Velodyne>
#include <osgJuniper/StreamingNode>

using namespace osgJuniper;



class VelodyneReader : public osgDB::ReaderWriter
{
public:
    VelodyneReader()
    {
        supportsExtension( "juniper_velodyne", className() );
    }

    virtual const char* className()
    {
        return "Juniper Velodyne Reader";
    }

    virtual ReadResult readNode( const std::string& location, const osgDB::ReaderWriter::Options* options ) const
    {
        if ( !acceptsExtension( osgDB::getLowerCaseFileExtension( location ) ) )
            return ReadResult::FILE_NOT_HANDLED;

        // strip the pseudo-loader extension
        std::string subLocation = osgDB::getNameLessExtension( location );
        if ( subLocation.empty() )
            return ReadResult::FILE_NOT_HANDLED;


        osg::ref_ptr<VelodyneDataset> ds = new VelodyneDataset(subLocation);
        VelodyneStreamingNodeSource* source = new VelodyneStreamingNodeSource( ds.get() );
        source->startStreaming();
        return new StreamingNode( source );
    }
};

REGISTER_OSGPLUGIN(juniper_velodyne, VelodyneReader)

