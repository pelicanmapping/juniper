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
#include <osgDB/ReadFile>
#include <osgDB/ReaderWriter>
#include <osgUtil/SmoothingVisitor>
#include <fstream>

#include <osg/AnimationPath>
#include <osg/MatrixTransform>


class PathReader : public osgDB::ReaderWriter
{
public:
    PathReader()
    {
        supportsExtension( "juniper_path", className() );
    }

    virtual const char* className()
    {
        return "AnimationPath Reader";
    }    

    virtual ReadResult readNode( const std::string& location, const osgDB::ReaderWriter::Options* options ) const
    {
        if ( !acceptsExtension( osgDB::getLowerCaseFileExtension( location ) ) )
            return ReadResult::FILE_NOT_HANDLED;

        std::string filename = osgDB::getNameLessExtension( location );

        std::string::size_type firstDot = filename.find_last_of('.');
        std::string::size_type secondDot = filename.find_last_of('.', firstDot-1 );
        OSG_NOTICE << "First dot = " << firstDot << " second dot = " << secondDot << std::endl;
        std::string animationPath = std::string( filename.begin() + secondDot+1, filename.end());
        std::string file = std::string( filename.begin(), filename.begin() + secondDot );

        std::string dir = osgDB::getFilePath( file );
        OSG_NOTICE << "Dir=" << dir << std::endl;
        animationPath = dir + "/" + animationPath;

        OSG_NOTICE << "Filename =" << file << std::endl;
        OSG_NOTICE << "Animation path =" << animationPath << std::endl;

       
        osg::Node* node = osgDB::readNodeFile( file );
        if (!node) return ReadResult::FILE_NOT_FOUND;
       
        //Read the path
        osg::AnimationPath* path = new osg::AnimationPath;
        std::ifstream in( animationPath.c_str() );
        path->read( in );

        osg::MatrixTransform* mt = new osg::MatrixTransform;
        mt->setUpdateCallback( new osg::AnimationPathCallback(path, 0.0, 1.0));
        mt->addChild( node );
        return mt;
    }

};

REGISTER_OSGPLUGIN(juniper_path, PathReader)

