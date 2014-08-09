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
#include <osgDB/FileNameUtils>
#include <osgDB/Registry>
#include <osgDB/ReaderWriter>
#include <osgEarth/GeoData>
#include <osg/io_utils>
#include <liblas/liblas.hpp>
#include <fstream>
#include <sstream>

#include <osgJuniper/Point>

using namespace osgJuniper;

class LASPointSource : public PointSource
{
public:
    
    class LASPointCursor : public PointCursor
    {
    public:
        
        LASPointCursor(const std::string& filename):
          _filename(filename),
          _in(_filename, std::ios::in | std::ios::binary),
          _reader(liblas::ReaderFactory().CreateWithStream(_in)),
          _hasMore(true),
          _totalPoints(0),
          _numRead(0)
        {               
            OSG_NOTICE << "LASPointCursor" << _filename << std::endl;
            liblas::Header const& header = _reader.GetHeader();

            //std::cout << "Compressed: " << (header.Compressed() == true) ? "true":"false";
            //std::cout << "Signature: " << header.GetFileSignature() << '\n';
            _totalPoints = header.GetPointRecordsCount();
            
            // Hong kong grid
            _srs = osgEarth::SpatialReference::create("epsg:2326");

            // Yarra
            //_srs = osgEarth::SpatialReference::create("epsg:28355");

            // Pauls files
            //_srs = osgEarth::SpatialReference::create("epsg:26911");
            _destSRS = osgEarth::SpatialReference::create("epsg:4326");
            OSG_NOTICE << "Points count: " << header.GetPointRecordsCount() << '\n';            
          }

        virtual bool hasMore() const
        {            
            return _hasMore;          
        }

        virtual bool nextPoint(Point& point)
        {                
            if (hasMore())
            {
                _numRead++;
                /*
                if (_numRead % 5000 == 0)
                {
                OSG_NOTICE << "Read " << _numRead << " of " << _totalPoints << std::endl;
                }
                */


                _hasMore = _reader.ReadNextPoint();
                liblas::Point const& p = _reader.GetPoint();
                //osg::Vec4f color = osg::Vec4(1.0f, 1.0f, 1.0f, 1.0f);
                float scale = 65280.0f;
                osg::Vec4f color(p.GetColor().GetRed()/scale, p.GetColor().GetGreen()/scale, p.GetColor().GetBlue()/scale, 1.0f);
                //OSG_NOTICE << "Color " << color.r() << ", " << color.g() << ", " << color.b() << std::endl;

                bool classify = true;

                if (classify)
                {            
                    std::string className = p.GetClassification().GetClassName();
                    if (className == "Ground")
                    {
                        color = osg::Vec4(1.0f, 0.0f, 0.0f, 1.0f);
                    }
                    else if (className == "Low Vegetation" ||
                        className == "Medium Vegetation" ||
                        className == "High Vegetation")
                    {
                        color = osg::Vec4(0.0f, 1.0f, 0.0f, 1.0f);
                    }
                    else if (className == "Building")
                    {
                        color = osg::Vec4(0.5, 0.5, 0.5, 1.0f);
                    }
                    else if (className == "Water")
                    {
                        color = osg::Vec4(0.0, 0.0, 1.0f, 0.0f);
                    }
                }

                osg::Vec3d position = osg::Vec3d(p.GetX(), p.GetY(), p.GetZ());
                osgEarth::GeoPoint geoPoint(_srs.get(), position);
                osgEarth::GeoPoint latLong;
                geoPoint.transform(_destSRS, latLong);
                //OSG_NOTICE << "latLong=" << latLong.x() << ", " << latLong.y() << ", " << latLong.z() << std::endl;
                osg::Vec3d world;
                latLong.toWorld(world);
                //OSG_NOTICE << "World " << world.x() << ", " << world.y() << ", " << world.z() << std::endl;
                point._position = world;                       
                point._color = color;
                //OSG_NOTICE << "Read point " << point._position.x() << ", " << point._position.y() << " " << point._position.z() << std::endl;          
                return true;    
            }
            return false;
        }

        std::string _filename;
        std::ifstream _in;      
        liblas::Reader _reader;        
        bool _hasMore;
        unsigned int _totalPoints;
        unsigned int _numRead;
        osg::ref_ptr< osgEarth::SpatialReference > _srs;
        osg::ref_ptr< osgEarth::SpatialReference > _destSRS;
    };


    LASPointSource(const std::string &filename):
      _filename(filename)
    {
        OSG_NOTICE << "Creating new LASPointSource " << _filename << std::endl;
    }

    virtual PointCursor* createPointCursor()
    {
        OSG_NOTICE << "Creating point cursor " << _filename << std::endl;
        return new LASPointCursor( _filename );
    }



protected:
    std::string _filename;
};

class LASPointsReaderWriter : public osgDB::ReaderWriter
{
public:
    LASPointsReaderWriter()
    {
        supportsExtension( "las", className() );
    }

    virtual const char* className()
    {
        return "LAS Point Reader";
    }

    virtual ReadResult readObject(const std::string& file_name, const Options* options) const
    {
        if ( !acceptsExtension(osgDB::getLowerCaseFileExtension( file_name )))
            return ReadResult::FILE_NOT_HANDLED;

        return new LASPointSource(file_name);
    }
};

REGISTER_OSGPLUGIN(las, LASPointsReaderWriter)

