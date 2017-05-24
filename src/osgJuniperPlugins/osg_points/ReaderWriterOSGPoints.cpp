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
                      _points.push_back(Point(vert, osg::Vec3(1,0,0), osg::Vec4ub(color.r() * 255.0, color.g() * 255.0, color.b() * 255.0, color.a() * 255.0)));
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

