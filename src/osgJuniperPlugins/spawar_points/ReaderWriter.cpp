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

class SPAWARPointSource : public PointSource
{
public:

    class SPAWARPointCursor : public PointCursor
    {
    public:
        SPAWARPointCursor(const std::string& filename):
          _numRead(0)
        {
            _in.open(filename.c_str(), std::ios::in | std::ios::binary  );
            if (!_in.is_open())
            {
                osg::notify(osg::NOTICE) << "Failed when opening " << filename << " for reading" << std::endl;
            }
        }

        virtual bool hasMore() const
        {
            return _in.is_open() && !_in.eof();
        }
        virtual bool nextPoint(Point& point)
        {
            if (hasMore())
            {
                osg::Vec3 vert;
                osg::Vec3 norm;
                osg::Vec4 col;
                float size; 
                if (_in.read((char*)(vert._v), 12) && _in.read((char*)(norm._v), 12) && _in.read((char*)(col._v), 12) && _in.read((char*)(&size), 4))
                {
                    col *= 1.0f/255.0f;
                    col[3] = 1.0;

                    point._position = vert;
                    point._color = col;
                    point._normal = norm;
                    point._size = size;
                    _numRead++;
                    return true;
                }
            }
            return false;
        }

        std::string _filename;
        std::ifstream _in;
        unsigned int _numRead;
    };

    SPAWARPointSource(const std::string& filename):
    _filename(filename)
    {
    }

    virtual PointCursor* createPointCursor()
    {
        return new SPAWARPointCursor(_filename);
    }


    const std::string& getFilename() const { return _filename;}



protected:
    std::string _filename;
};

class SpawarPointsReaderWriter : public osgDB::ReaderWriter
{
public:
    SpawarPointsReaderWriter()
    {
        supportsExtension( "spawar_points", className() );
    }

    virtual const char* className()
    {
        return "Juniper SPAWAR Point Reader";
    }

    virtual ReadResult readObject(const std::string& file_name, const Options* options) const
    {
        if ( !acceptsExtension(osgDB::getLowerCaseFileExtension( file_name )))
            return ReadResult::FILE_NOT_HANDLED;

        return new SPAWARPointSource(file_name);
    }
};

REGISTER_OSGPLUGIN(spawar_points, SpawarPointsReaderWriter)

