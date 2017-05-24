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

