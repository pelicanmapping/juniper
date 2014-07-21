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
#include <osgDB/ReadFile>
#include <osgDB/Registry>
#include <osgDB/FileNameUtils>
#include <osgDB/ReaderWriter>
#include <osg/io_utils>
#include <fstream>

#include <osgJuniper/Point>

using namespace osgJuniper;

class CollectPointsVisitor : public osg::NodeVisitor
{
public: 
    CollectPointsVisitor(PointList& points):
      osg::NodeVisitor(osg::NodeVisitor::TRAVERSE_ALL_CHILDREN),
          _points(points)
      {
      }

      inline void pushMatrix(osg::Matrix& matrix) { _matrixStack.push_back(matrix); }

      inline void popMatrix() { _matrixStack.pop_back(); }

      void apply(osg::Transform& transform)
      {
          osg::Matrix matrix;
          if (!_matrixStack.empty()) matrix = _matrixStack.back();

          transform.computeLocalToWorldMatrix(matrix,this);

          pushMatrix(matrix);

          traverse(transform);

          popMatrix();
      }

      void apply(osg::Geode& geode)
      {
          for(unsigned int i=0; i<geode.getNumDrawables(); ++i)
          {
              applyDrawable(geode.getDrawable(i));
          }
      }

      void applyDrawable(osg::Drawable* drawable)
      {
          osg::Geometry* geom = drawable->asGeometry();
          if (geom)
          {
              osg::Matrixd matrix;
              if (!_matrixStack.empty()) matrix = _matrixStack.back();              

              osg::Vec3Array* verts = dynamic_cast< osg::Vec3Array* >(geom->getVertexArray());
              osg::Vec4Array* colors = dynamic_cast< osg::Vec4Array* >(geom->getColorArray());

              if (verts)
              {
                  for (unsigned int i = 0; i < verts->size(); i++)
                  {
                      osg::Vec3d vert = (*verts)[i] * matrix;
                      osg::Vec4 color = osg::Vec4(1,0,0,0);
                      //OSG_NOTICE << "Adding point " << vert << std::endl;
                      if (colors)
                      {
                          if (geom->getColorBinding() == osg::Geometry::BIND_OVERALL)
                          {
                              color = (*colors)[0];
                          }
                          else if (geom->getColorBinding() == osg::Geometry::BIND_PER_VERTEX)
                          {
                              color = (*colors)[i];
                          }
                      }
                      _points.push_back(Point(vert, osg::Vec3(1,0,0), color));
                  }
              }        
          }
      }


      typedef std::vector<osg::Matrix> MatrixStack;
      MatrixStack         _matrixStack;
      PointList& _points;
};

class OSGPointsReaderWriter : public osgDB::ReaderWriter
{
public:
    OSGPointsReaderWriter()
    {
        supportsExtension( "osg_points", className() );
    }

    virtual const char* className()
    {
        return "OSG Point Reader";
    }

    virtual ReadResult readObject(const std::string& file_name, const Options* options) const
    {
        if ( !acceptsExtension(osgDB::getLowerCaseFileExtension( file_name )))
            return ReadResult::FILE_NOT_HANDLED;

        PointListSource *points = new PointListSource();

        //Strip the psuedo-loader name.
        std::string filename = osgDB::getNameLessExtension( file_name );
        osg::ref_ptr< osg::Node > node = osgDB::readNodeFile( filename );
        if (node.valid())
        {
            OSG_NOTICE << "Collecting points" << std::endl;
            CollectPointsVisitor v(points->getPoints());
            node->accept( v );
        }
        node = NULL; //Deallocate the node's memory;
        OSG_NOTICE << "Found " << points->getPoints().size() << " points" << std::endl;
        return points;
    }
};

REGISTER_OSGPLUGIN(osg_points, OSGPointsReaderWriter)

