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

#include <osgJuniper/Point>

using namespace osgJuniper;

class PTSPointSource : public PointSource
{
public:
    
    class PTSPointCursor : public PointCursor
    {
    public:
        
        PTSPointCursor(const std::string& filename):
          _filename(filename)
        {
            _in.open(filename.c_str(), std::ios::in | std::ios::binary );
            if (!_in.is_open())
            {
                osg::notify(osg::NOTICE) << "Failed when opening " << filename << " for reading" << std::endl;
            }
        }

        virtual bool nextPoint(Point& point)
        {
            if (!_in.eof())
            {
                osg::Vec3 vert;
                if (_in.read((char*)(&vert[0]), sizeof(float) * 3))
                {
                    point.position = vert;
                    point.color = osg::Vec4ub(255,255,255,255);
                    return true;
                }
            }
            return false;         
        }

        std::ifstream _in;
        std::string _filename;
    };


    PTSPointSource(const std::string &filename):
      _filename(filename)
    {
    }

    virtual PointCursor* createPointCursor()
    {
        return new PTSPointCursor( _filename );
    }
protected:
    std::string _filename;
};

class PTSReaderWriter : public osgDB::ReaderWriter
{
public:
    PTSReaderWriter()
    {
        supportsExtension( "pts", className() );
    }

    virtual const char* className()
    {
        return "Simple points reader";
    }

    virtual ReadResult readObject(const std::string& file_name, const Options* options) const
    {
        if ( !acceptsExtension(osgDB::getLowerCaseFileExtension( file_name )))
            return ReadResult::FILE_NOT_HANDLED;
        return new PTSPointSource(file_name);
    }
};

REGISTER_OSGPLUGIN(pts, PTSReaderWriter)

