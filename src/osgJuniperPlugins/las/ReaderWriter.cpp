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
#include <osgEarth/URI>
#include <osg/io_utils>

#include "lasreader.hpp"

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
              _totalPoints(0),
              _numRead(0),
              _reader(0)
          {               
              LASreadOpener lasreadopener;        
              lasreadopener.set_file_name(filename.c_str());
              _reader = lasreadopener.open();              

              std::string prjLocation = osgDB::getNameLessExtension(_filename) + std::string(".prj");              

              osgEarth::ReadResult rr = osgEarth::URI(prjLocation).readString();
              if (rr.succeeded())
              {
                  _srs = osgEarth::SpatialReference::create(rr.getString());
              }
              else
              {
                  _srs = osgEarth::SpatialReference::create("epsg:4326");
              }

              // Hong kong grid
              //_srs = osgEarth::SpatialReference::create("epsg:2326");

              // Yarra
              //_srs = osgEarth::SpatialReference::create("epsg:28355");

              // Pauls files
              //_srs = osgEarth::SpatialReference::create("epsg:26911");
              _destSRS = osgEarth::SpatialReference::create("epsg:4326");     
              
              

              _header = &(_reader->header);              
              OSG_NOTICE << "Number of point records " << _header->number_of_point_records << std::endl;
              OSG_NOTICE << "Scale factor " << _header->x_scale_factor << ", " << _header->y_scale_factor << ", " << _header->z_scale_factor << std::endl;                            
              
              /*
              for (unsigned int i = 0; i < _header->number_of_variable_length_records; i++)
              {
                  if (strcmp(_header->vlrs[i].user_id, "LASF_Projection") == 0 )
                  {
                      OSG_NOTICE << "Got LASF_Projection" << std::endl;
                  }
                  OSG_NOTICE << "VLR " << _header->vlrs[i].user_id << std::endl;
              }
              */



          }
          
          LASPointCursor::~LASPointCursor()
          {
              if (_reader)
              {
                  delete _reader;
              }
          }
          
          virtual bool nextPoint(Point& point)
          {                
              
              if (_reader->read_point())
              {   
                  _numRead++;           
                  /*
                  if (_numRead % 10000 == 0)
                  {
                      OSG_NOTICE << "Read " << _numRead << " points" << std::endl;
                  }
                  */

                  osg::Vec4 color = osg::Vec4(0.0, 0.0f, 0.0f, 1.0f);
                  // Color by RGB if we have it.
                  if (_reader->point.have_rgb)                  
                  {                   
                      float colorScale = (float)USHRT_MAX;
                      color.set((float)_reader->point.rgb[0] / colorScale,
                                (float)_reader->point.rgb[1] / colorScale,
                                (float)_reader->point.rgb[2] / colorScale,
                                1.0f);
                  }
                  else
                  {
                      // Otherwise color by classification
                      int classification = (int)_reader->point.classification;                      
                      // Ground
                      if (classification == 2)
                      {
                          color = osg::Vec4(1.0f, 0.0f, 0.0f, 1.0f);
                      }
                      // Veg
                      else if (classification == 3 || classification == 4 || classification == 5)
                      {
                          color = osg::Vec4(0.0f, 1.0f, 0.0f, 1.0f);
                      }
                      // Building
                      else if (classification == 6)
                      {
                          color = osg::Vec4(0.5, 0.5, 0.5, 1.0f);
                      }
                      // Water
                      else if (classification == 9)
                      {
                          color = osg::Vec4(0.0, 0.0, 1.0, 1.0);
                      }

                  }                  
                  
                  osg::Vec3d position = osg::Vec3d((double)_reader->point.X * _header->x_scale_factor, (double)_reader->point.Y * _header->y_scale_factor, (double)_reader->point.Z * _header->z_scale_factor);                  
                  osgEarth::GeoPoint geoPoint(_srs.get(), position);
                  osgEarth::GeoPoint latLong;
                  geoPoint.transform(_destSRS, latLong);                  
                  osg::Vec3d world;
                  latLong.toWorld(world);
                  point._position = position;                       
                  point._color = color;                  
                  return true;                
              }              
              return false;                                                
          }

          std::string _filename;
          std::ifstream _in;      
          LASreader* _reader;
          LASheader* _header;          
          unsigned int _totalPoints;
          unsigned int _numRead;
          osg::ref_ptr< osgEarth::SpatialReference > _srs;
          osg::ref_ptr< osgEarth::SpatialReference > _destSRS;
    };


    LASPointSource(const std::string &filename):
    _filename(filename)
    {        
    }

    virtual PointCursor* createPointCursor()
    {     
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
        supportsExtension( "laz", className() );
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

