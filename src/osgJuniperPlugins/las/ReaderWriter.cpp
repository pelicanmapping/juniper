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
#include <osg/MatrixTransform>

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

              

              // Hong kong grid
              //_srs = osgEarth::SpatialReference::create("epsg:2326");           
              
              _header = &(_reader->header);              
              OSG_NOTICE << "Number of point records " << _header->number_of_point_records << std::endl;
              OSG_NOTICE << "Scale factor " << _header->x_scale_factor << ", " << _header->y_scale_factor << ", " << _header->z_scale_factor << std::endl;                            
                            
              
              // Small chunk from lasinfo to get the epsg code.
              for (unsigned int i = 0; i < _header->number_of_variable_length_records; i++)
              {
                  if (strcmp(_header->vlrs[i].user_id, "LASF_Projection") == 0 && (_header->vlrs[i].data != 0))
                  {
                      if (_header->vlrs[i].record_id == 34735)
                      {
                          for (int j = 0; j < _header->vlr_geo_keys->number_of_keys; j++)
                          {
                              switch(_header->vlr_geo_key_entries[j].key_id)
                              {
                              case 3072: // GTModelTypeGeoKey 
                                  int epsg = _header->vlr_geo_key_entries[j].value_offset;
                                  std::stringstream buf;
                                  buf << "epsg:" << epsg;                                  
                                  _srs = osgEarth::SpatialReference::create(buf.str());                                  
                              }                              
                          }                          
                      }
                  }                  
              }       

              // Try to read the prj if we don't have a srs yet.
              if (!_srs.valid())
              {
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
              }
          }
          
          LASPointCursor::~LASPointCursor()
          {
              if (_reader)
              {
                  OSG_NOTICE << "~LASPointCursor" << std::endl;
                  _reader->close();
                  delete _reader;
              }
          }
          
          virtual bool nextPoint(Point& point)
          {                
              
              if (_reader->read_point())
              {   
                  _numRead++;           

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
                  
                  osg::Vec3d position = osg::Vec3d(_reader->point.get_x(), _reader->point.get_y(), _reader->point.get_z());
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

osg::Node* makeNode(const std::string& filename)
{    
    osg::Vec3d anchor;
    bool first = true;

    osg::Geometry* geometry = new osg::Geometry;

    geometry->setUseVertexBufferObjects( true );
    geometry->setUseDisplayList( false );
    
    osg::Vec3Array* verts = new osg::Vec3Array();
    geometry->setVertexArray( verts );    

    osg::Vec4ubArray* colors =new osg::Vec4ubArray();    
    geometry->setColorArray(colors);
    geometry->setColorBinding(osg::Geometry::BIND_PER_VERTEX);

    osg::Vec4Array* dataArray = new osg::Vec4Array();
    geometry->setVertexAttribArray(osg::Drawable::ATTRIBUTE_6, dataArray);
    geometry->setVertexAttribBinding(osg::Drawable::ATTRIBUTE_6, osg::Geometry::BIND_PER_VERTEX);
    geometry->setVertexAttribNormalize(osg::Drawable::ATTRIBUTE_6, false);    
    
    // Read all the points
    LASreadOpener lasreadopener;        
    lasreadopener.set_file_name(filename.c_str());
    LASreader* reader = lasreadopener.open();              

    OSG_NOTICE << "Reading " << reader->header.number_of_point_records << " point records from " << filename << std::endl;
     
    while(reader->read_point())
    {                 
        osg::Vec3d point = osg::Vec3d(reader->point.get_x(), reader->point.get_y(), reader->point.get_z());        

        osg::Vec4 data;
        data.x() = (int)reader->point.classification;
        data.y() = (int)reader->point.return_number;
        data.z() = (int)(reader->point.intensity);
        dataArray->push_back(data);

        if (first)
        {
            anchor = point;
            first = false;
        }
        osg::Vec3 position = point - anchor;
        verts->push_back(position);

        osg::Vec4ub color = osg::Vec4ub(0, 0, 0, 255);
        // Color by RGB if we have it.
        if (reader->point.have_rgb)                  
        {                          
            color.set(U8_CLAMP(reader->point.rgb[0]/256),
                      U8_CLAMP(reader->point.rgb[1]/256),
                      U8_CLAMP(reader->point.rgb[2]/256),
                      U8_CLAMP(reader->point.rgb[3]/256));
        }
        colors->push_back(color);        
    }

    reader->close();
    delete reader;

    osg::Geode* geode = new osg::Geode;
    geode->addDrawable( geometry );
    geometry->addPrimitiveSet( new osg::DrawArrays(GL_POINTS, 0, verts->size()) );

    osg::MatrixTransform* mt = new osg::MatrixTransform;
    mt->setMatrix(osg::Matrixd::translate(anchor));
    mt->addChild(geode);
    return mt;    
}

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

    virtual ReadResult readNode( const std::string& location, const osgDB::ReaderWriter::Options* options ) const
    {
        if ( !acceptsExtension( osgDB::getLowerCaseFileExtension( location ) ) )
            return ReadResult::FILE_NOT_HANDLED;

        return makeNode(location);       
    }
};

REGISTER_OSGPLUGIN(las, LASPointsReaderWriter)

