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


#define MAX_ARRAY_SIZE 10000


class XYZReader : public osgDB::ReaderWriter
{
public:
    XYZReader()
    {
        supportsExtension( "juniper_xyz", className() );
    }

    virtual const char* className()
    {
        return "Juniper XYZ Reader";
    }

    void addNormals( osg::Geometry* geom ) const
    {
        const osg::BoundingBox& bbox = geom->getBound();

        osg::Vec3Array* verts = static_cast<osg::Vec3Array*>( geom->getVertexArray() );

        osg::Vec3Array* normals = new osg::Vec3Array( verts->size() );
        for( unsigned int i=0; i<verts->size(); ++i )
        {
            osg::Vec3 vec = (*verts)[i] - bbox.center();
            vec.normalize();
            (*normals)[i] = vec;
        }
        geom->setNormalArray( normals );
        geom->setNormalBinding( osg::Geometry::BIND_PER_VERTEX );
    }

    virtual ReadResult readNode( const std::string& location, const osgDB::ReaderWriter::Options* options ) const
    {
        if ( !acceptsExtension( osgDB::getLowerCaseFileExtension( location ) ) )
            return ReadResult::FILE_NOT_HANDLED;

        std::ifstream in( location.c_str() );
        if ( !in.is_open() )
            return ReadResult::FILE_NOT_FOUND;

        osg::ref_ptr<osg::Geode> geode = new osg::Geode();

        osg::Vec3Array* verts = 0L;
        osg::Geometry*  geom  = 0L;

        do
        {
            if ( !verts || verts->size() == MAX_ARRAY_SIZE || in.eof() )
            {
                if ( geom )
                {
                    geom->addPrimitiveSet( new osg::DrawArrays( GL_POINTS, 0, verts->size() ) );
                }

                if ( !in.eof() )
                {
                    geom = new osg::Geometry();
                    verts = new osg::Vec3Array();
                    geom->setVertexArray( verts );
                    geode->addDrawable( geom );
                }
            }

            if ( !in.eof() )
            {
                osg::Vec3d p;
                in >> p.x() >> p.y() >> p.z();
                verts->push_back( p );
            }
        }
        while( !in.eof() );

        in.close();

        for( unsigned int i=0; i<geode->getNumDrawables(); ++i )
        {
            addNormals( static_cast<osg::Geometry*>( geode->getDrawable(i) ));
        }

        return geode.release();
    }
};

REGISTER_OSGPLUGIN(juniper_xyz, XYZReader)

