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
#include <osg/io_utils>
#include <fstream>


class SPAWARReader : public osgDB::ReaderWriter
{
public:
    SPAWARReader()
    {
        supportsExtension( "juniper_spawar", className() );
    }

    virtual const char* className()
    {
        return "Juniper SPAWAR Reader";
    }

    virtual ReadResult readNode( const std::string& location, const osgDB::ReaderWriter::Options* options ) const
    {
        if ( !acceptsExtension( osgDB::getLowerCaseFileExtension( location ) ) )
            return ReadResult::FILE_NOT_HANDLED;

        std::ifstream in( location.c_str(), std::ios::in | std::ios::binary  );
        if ( !in.is_open() )
            return ReadResult::FILE_NOT_FOUND;

        osg::ref_ptr<osg::Group> grp = new osg::Group;

        in.seekg(0, std::ios_base::end);
        int nbTotalVertexes = in.tellg() / (12 * 3 + 4);
        in.seekg(0, std::ios_base::beg);
        
        osg::notify(osg::NOTICE) << "number of vertexes " << nbTotalVertexes << std::endl;

        // split per bunch of 5 million of vertexes
        int nbVertexes = 0;
        osg::ref_ptr<osg::Geode> geode = new osg::Geode();
        osg::ref_ptr<osg::Vec3Array> verts = new osg::Vec3Array;
        osg::ref_ptr<osg::Vec3Array> normals = new osg::Vec3Array;
        osg::ref_ptr<osg::Vec4Array> colors = new osg::Vec4Array;
        osg::ref_ptr<osg::Vec4Array> scales = new osg::Vec4Array;
        osg::ref_ptr<osg::Geometry> geom  = new osg::Geometry;

        while (!in.eof()) {
            osg::Vec3 vert;
            osg::Vec3 norm;
            osg::Vec4 col;
            float size = 1.0f;
 
            if (in.read((char*)(&vert[0]), 12) && in.read((char*)(&norm[0]), 12) && in.read((char*)(&col[0]), 12) && in.read((char*)(&size), 4))
            {
                col *= 1.0/255.0;
                col[3] = 1.0;
                verts->push_back(vert);
                normals->push_back(norm);
                colors->push_back(col);
                //osg::notify(osg::NOTICE) << "color " << col << std::endl;
                scales->push_back(osg::Vec4(0,0,0,size));
                nbVertexes++;
            }
        }
        osg::notify(osg::NOTICE) << "Read in " << nbVertexes << " verts" << std::endl;
 
        in.close();
        geom->setVertexArray(verts.get());
        geom->setNormalArray(normals.get());
        geom->setColorArray(colors.get());
        geom->setVertexAttribArray(11, scales.get());
        geom->setVertexAttribBinding(11, osg::Geometry::BIND_PER_VERTEX);
        geom->setVertexAttribNormalize(11, false);        
        geom->getPrimitiveSetList().push_back(new osg::DrawArrays(GL_POINTS, 0, verts->size()));
        geode->addDrawable(geom.get());

        return geode.release();
    }
};

REGISTER_OSGPLUGIN(juniper_spawar, SPAWARReader)

