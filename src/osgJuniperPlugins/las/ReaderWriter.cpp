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
#include <osgDB/FileNameUtils>
#include <osgDB/FileUtils>
#include <osgDB/Registry>
#include <osgDB/ReaderWriter>
#include <osgEarth/GeoData>
#include <osgEarth/URI>
#include <osg/io_utils>
#include <osg/MatrixTransform>

#include "lasreader.hpp"
#include "lasreader_las.hpp"

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
                  _reader->close();
                  delete _reader;
              }
          }
          
          virtual bool nextPoint(Point& point)
          {                
              
              if (_reader->read_point())
              {   
                  _numRead++;           

                  osg::Vec4ub color = osg::Vec4ub(0,0,0,255);
                  // Color by RGB if we have it.
                  if (_reader->point.have_rgb)                  
                  {                          
                      color.set(U8_CLAMP(_reader->point.rgb[0]/256),
                                U8_CLAMP(_reader->point.rgb[1]/256),
                                U8_CLAMP(_reader->point.rgb[2]/256),
                                U8_CLAMP(_reader->point.rgb[3]/256));
                  }

                  osg::Vec3d position = osg::Vec3d(_reader->point.get_x(), _reader->point.get_y(), _reader->point.get_z());
                  point.position = position;                        
                  point.color = color;    
                  point.classification = _reader->point.classification;
                  point.returnNumber = _reader->point.return_number;
                  point.intensity = _reader->point.intensity;
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

osg::Node* makeNode(LASreader* reader, const osgDB::ReaderWriter::Options* options)
{    
    osg::Vec3d anchor;
    bool first = true;

    unsigned int NUM_POINTS_PER_GEOMETRY = 50000;

    osg::Geode* geode = new osg::Geode;

    osg::Geometry* geometry = 0;
    osg::Vec3Array* verts = 0;
    osg::Vec4ubArray* colors = 0;
    osg::Vec4Array* dataArray = 0;
    
    unsigned int keepEvery = 1;
    if (options && !options->getOptionString().empty())
    {
        std::string keepEveryStr = options->getPluginStringData("keepEvery");        
        if (!keepEveryStr.empty())
        {
            std::istringstream iss(keepEveryStr);
            iss >> keepEvery;
        }
    }


    //OSG_NOTICE << "Reading " << reader->header.number_of_point_records << " point records from " << filename << std::endl;
    unsigned int numPoints = 0;
    unsigned int numRead = 0;
     
    while(reader->read_point())
    {                 
        // Initialize the geometry if needed.
        if (!geometry)
        {
            geometry = new osg::Geometry;
            geometry->setUseVertexBufferObjects( true );
            geometry->setUseDisplayList( false );

            verts = new osg::Vec3Array();
            geometry->setVertexArray( verts );    

            colors =new osg::Vec4ubArray();    
            geometry->setColorArray(colors);
            geometry->setColorBinding(osg::Geometry::BIND_PER_VERTEX);

            dataArray = new osg::Vec4Array();
            geometry->setVertexAttribArray(osg::Drawable::ATTRIBUTE_6, dataArray);
            geometry->setVertexAttribBinding(osg::Drawable::ATTRIBUTE_6, osg::Geometry::BIND_PER_VERTEX);
            geometry->setVertexAttribNormalize(osg::Drawable::ATTRIBUTE_6, false);    

        }
        numPoints++;
        if (numPoints % keepEvery != 0) continue;
        numRead++;

        osg::Vec3d point = osg::Vec3d(reader->point.get_x(), reader->point.get_y(), reader->point.get_z());        

        osg::Vec4 data;
        data.x() = (int)reader->point.classification;
        data.y() = (int)reader->point.return_number;
        data.z() = (int)(reader->point.intensity);

        // TODO:  This assumes the point is in geocentric
        // Get the height above ellipsoid for the point.
        osgEarth::GeoPoint pt;
        pt.fromWorld(osgEarth::SpatialReference::create("wgs84"), point);
        data.w() = pt.alt();
        
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

        if (verts->size() == NUM_POINTS_PER_GEOMETRY)
        {
            geode->addDrawable( geometry );
            geometry->addPrimitiveSet( new osg::DrawArrays(GL_POINTS, 0, verts->size()) );
            geometry = 0;
        }
    }        

    // Add a final geometry if necessary
    if (geometry)
    {
        geode->addDrawable( geometry );
        geometry->addPrimitiveSet( new osg::DrawArrays(GL_POINTS, 0, verts->size()) );
        geometry = 0;
    }

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


        // Handle the server case explicitly, don't let it fall through to the curl plugin.
        if (osgDB::containsServerAddress( location ))
        {
            osg::Timer_t start = osg::Timer::instance()->tick();
            osgEarth::ReadResult r = osgEarth::URI(location).readString(options);
            osg::Timer_t end = osg::Timer::instance()->tick();
            OSG_NOTICE << "download of " << location << " took " << osg::Timer::instance()->delta_m(start, end) << "ms" << std::endl;
            if (r.failed())
            {
                // If we couldn't get the file just return an empty group so the pager keeps paging.
                return new osg::Group;
            }
            else
            {
                std::stringstream in(r.getString());
                return readNode(in, options);
            }
        }

        if (!osgDB::fileExists(location))
        {
            return ReadResult::FILE_NOT_FOUND;
        }

        // Read all the points
        LASreadOpener lasreadopener;        
        lasreadopener.set_file_name(location.c_str());
        LASreader* reader = lasreadopener.open();  
        
        osg::Node* node = makeNode(reader, options);       

        reader->close();
        delete reader;
        return node;
    }

    virtual ReadResult readNode(std::istream& in, const osgDB::Options* options) const
    {     
        osg::Timer_t start = osg::Timer::instance()->tick();
        LASreaderLAS* reader = new LASreaderLAS();
        osg::Node* node = 0;
        if (reader->open(in))
        {
            node = makeNode(reader, options);       
            reader->close();           
        }
        delete reader;
        osg::Timer_t end = osg::Timer::instance()->tick();
        OSG_NOTICE << "readNode took " << osg::Timer::instance()->delta_m(start, end) << "ms" << std::endl;
        if (node) return node;
        return ReadResult::FILE_NOT_HANDLED;       
        
    }

};

REGISTER_OSGPLUGIN(las, LASPointsReaderWriter)

