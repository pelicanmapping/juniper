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
#include <sstream>

#include <osgJuniper/Point>

using namespace osgJuniper;

class CSVPointSource : public PointSource
{
public:
    
    class CSVPointCursor : public PointCursor
    {
    public:
        
        CSVPointCursor(const std::string& filename):
          _filename(filename)
        {
            _in.open(filename.c_str(), std::ios::in );
            if (!_in.is_open())
            {
                osg::notify(osg::NOTICE) << "Failed when opening " << filename << " for reading" << std::endl;
            }
        }

        virtual bool nextPoint(Point& point)
        {
            osg::Vec3 position;            
            bool gotPoint = false;
            while (!_in.eof() && !gotPoint)
            {
                unsigned int numRead = 0;
                //Read a line from the file
                std::string line;
                getline(_in, line);

                std::stringstream ss(line);
                std::string field;
                while(getline(ss, field, ','))
                {
                    std::stringstream fs(field);
                    float f = FLT_MAX;
                    fs >> f;
                    //If we didn't read in the value correctly, exit and read the next line
                    if (f == FLT_MAX) break;
                    position[numRead] = f;
                    numRead++;
                    if (numRead == 3) break;
                }
                if (numRead == 3) gotPoint = true;
            }
            if (gotPoint)
            {
                point.color    = osg::Vec4ub(255,255,255,255);
                point.position = position;
                return true;
            }
            return false;
        }

        std::ifstream _in;
        std::string _filename;
    };


    CSVPointSource(const std::string &filename):
      _filename(filename)
    {
    }

    virtual PointCursor* createPointCursor()
    {
        return new CSVPointCursor( _filename );
    }



protected:
    std::string _filename;
};

class CSVPointsReaderWriter : public osgDB::ReaderWriter
{
public:
    CSVPointsReaderWriter()
    {
        supportsExtension( "csv", className() );
    }

    virtual const char* className()
    {
        return "CSV Point Reader";
    }

    virtual ReadResult readObject(const std::string& file_name, const Options* options) const
    {
        if ( !acceptsExtension(osgDB::getLowerCaseFileExtension( file_name )))
            return ReadResult::FILE_NOT_HANDLED;

        return new CSVPointSource(file_name);
    }
};

REGISTER_OSGPLUGIN(csv, CSVPointsReaderWriter)

