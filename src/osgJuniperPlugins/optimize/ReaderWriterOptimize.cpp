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

#include <osgDB/Registry>
#include <osgDB/FileNameUtils>
#include <osgDB/ReadFile>
#include <osgDB/ReaderWriter>
#include <osgJuniper/Utils>

class OptimizeReader : public osgDB::ReaderWriter
{
public:
    OptimizeReader()
    {
        supportsExtension( "optimize", className() );
    }

    virtual const char* className()
    {
        return "Juniper optimization plugin";
    }    

    virtual ReadResult readNode( const std::string& location, const osgDB::ReaderWriter::Options* options ) const
    {
        if ( !acceptsExtension( osgDB::getLowerCaseFileExtension( location ) ) )
            return ReadResult::FILE_NOT_HANDLED;

        std::string file = osgDB::getNameLessExtension( location );
        osg::Node* node = osgDB::readNodeFile( file );
        if (!node) return ReadResult::FILE_NOT_FOUND;

        osgJuniper::Utils::optimizeMesh( node );
        node->setDataVariance( osg::Object::DYNAMIC );
        return node;
    }

};

REGISTER_OSGPLUGIN(optimize, OptimizeReader)

