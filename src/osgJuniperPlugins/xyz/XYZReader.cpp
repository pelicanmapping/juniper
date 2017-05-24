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
#include <osg/Geometry>
#include <osg/Geode>
#include <osg/Point>
#include <osg/LOD>
#include <osg/ShapeDrawable>
#include <osg/Version>
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
#if OSG_VERSION_GREATER_THAN(3,3,1)
        const osg::BoundingBox& bbox = geom->getBoundingBox();
#else
        const osg::BoundingBox& bbox = geom->getBound();
#endif

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

