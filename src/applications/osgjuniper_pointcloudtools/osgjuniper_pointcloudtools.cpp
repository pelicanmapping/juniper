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
#include <osgDB/ReadFile>
#include <osgUtil/Optimizer>
#include <osgUtil/PolytopeIntersector>
#include <osg/CoordinateSystemNode>
#include <osg/Point>
#include <osg/LineWidth>

#include <osgViewer/Viewer>
#include <osgViewer/ViewerEventHandlers>

#include <osgGA/StateSetManipulator>

#include <osgEarth/MapNode>
#include <osgEarthUtil/LatLongFormatter>
#include <osgEarthUtil/ExampleResources>
#include <osgEarthSymbology/Color>

#include <iostream>

using namespace osgEarth;
using namespace osgEarth::Util;

static osg::Point* s_point;

static LabelControl* s_status;

osg::Node::NodeMask MaskMapNode = 0x01;
osg::Node::NodeMask MaskPointCloud = 0x02;

struct PointSizeHandler : public ControlEventHandler
{
    PointSizeHandler( osg::Point* point ) : _point(point) { }
    void onValueChanged( Control* control, float value )
    {        
        _point->setSize( value );        
        OSG_NOTICE << "Point size " << value << std::endl;
    }
    osg::ref_ptr< osg::Point > _point;
};

void buildControls(osgViewer::Viewer& viewer)
{
    ControlCanvas* canvas = ControlCanvas::getOrCreate( &viewer );
    VBox* container = canvas->addControl(new VBox());
    container->setBackColor(Color(Color::Black,0.5));

    // Point size
    HBox* pointSizeBox = container->addControl(new HBox());
    pointSizeBox->setChildVertAlign( Control::ALIGN_CENTER );
    pointSizeBox->setChildSpacing( 10 );
    pointSizeBox->setHorizFill( true );
    pointSizeBox->addControl( new LabelControl("Point Size:", 16) );

    HSliderControl* pointSlider = pointSizeBox->addControl(new HSliderControl(1.0, 10.0f, 1.0f));
    pointSlider->setBackColor( Color::Gray );
    pointSlider->setHeight( 12 );
    pointSlider->setHorizFill( true, 200 );
    pointSlider->addEventHandler( new PointSizeHandler(s_point));   

    // Add a status label
    s_status = container->addControl(new LabelControl());
}


/**
 * Handler to identify a point's location
 */
class IdentifyPointHandler : public osgGA::GUIEventHandler
{
public:
    IdentifyPointHandler():
      _wgs84(SpatialReference::create("epsg:4326")),
      _selectedColor(255, 0, 0, 255),
      _prevSelectedIndex(0)
    {
    }

    bool handle(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa)
    {
        osgViewer::Viewer* viewer = dynamic_cast<osgViewer::Viewer*>(&aa);
        if (!viewer) return false;

        switch (ea.getEventType())
        {
        case osgGA::GUIEventAdapter::PUSH:
            pick(ea.getX(), ea.getY(), viewer);
            break;            
        }
        return false;
    }

protected:
    void pick(float x, float y, osgViewer::Viewer* viewer)
    {
        // Unset the previous selected point if we have one.
        if (_prevColorArray.valid())
        {
            (*_prevColorArray.get())[_prevSelectedIndex] = _prevSelectedColor;
            _prevColorArray->dirty();
            _prevColorArray = 0;
        }
        
        double w = 5.0;
        double h = 5.0;
        osgUtil::PolytopeIntersector *picker = new osgUtil::PolytopeIntersector(osgUtil::Intersector::WINDOW, x - w, y- h, x + w, y + h);
        osgUtil::IntersectionVisitor iv(picker);
        iv.setTraversalMask(MaskPointCloud);
        viewer->getCamera()->accept(iv);
        if (picker->containsIntersections())
        {
            // Get the point that was clicked.
            osgUtil::PolytopeIntersector::Intersection hit = picker->getFirstIntersection();
            osg::Geometry* geometry = hit.drawable->asGeometry();

            osg::Vec3Array* verts = static_cast<osg::Vec3Array*>(geometry->getVertexArray());
            // Since we are just drawing points the point that was clicked is just the primitive index.
            osg::Vec3 vert = (*verts)[hit.primitiveIndex];            

            // Highlight the selected point
            osg::Vec4ubArray* colors = static_cast<osg::Vec4ubArray*>(geometry->getColorArray());
            
            // Store the current values.
            _prevColorArray = colors;
            _prevSelectedColor = (*colors)[hit.primitiveIndex];
            _prevSelectedIndex = hit.primitiveIndex;            

            (*colors)[hit.primitiveIndex] = _selectedColor;
            colors->dirty();

            osg::Matrixd localToWorld = osg::computeLocalToWorld(hit.nodePath);

            // Get the geocentric world coordinate
            osg::Vec3d world = vert * localToWorld;            

            // Convert the geocentric coordinate to lat long
            osgEarth::GeoPoint point;
            point.fromWorld(_wgs84.get(), world);

            LatLongFormatter formatter;
            formatter.setPrecision(8);
            std::stringstream buf;
            buf << formatter.format(point) << ", " << point.z();
            s_status->setText( buf.str() );            
        }
        else
        {
            s_status->setText("");            
        }
    }

    osg::ref_ptr< SpatialReference > _wgs84;
    osg::ref_ptr< osg::Vec4ubArray > _prevColorArray;
    osg::Vec4ub _prevSelectedColor;
    int _prevSelectedIndex;
    osg::Vec4ub _selectedColor;    
};

/**
 * Point to point measure handler
 */
class P2PMeasureHandler : public osgGA::GUIEventHandler
{
public:
    P2PMeasureHandler(osg::Group* root):
      _wgs84(SpatialReference::create("epsg:4326")),
      _root(root)
    {
    }

    bool handle(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa)
    {
        osgViewer::Viewer* viewer = dynamic_cast<osgViewer::Viewer*>(&aa);
        if (!viewer) return false;

        switch (ea.getEventType())
        {
        case osgGA::GUIEventAdapter::PUSH:
            if (ea.getButton() == osgGA::GUIEventAdapter::LEFT_MOUSE_BUTTON)
            {
                pick(ea.getX(), ea.getY(), viewer);
            }
            else if (ea.getButtonMask() == osgGA::GUIEventAdapter::RIGHT_MOUSE_BUTTON)
            {
                clear();
            }
            break;   
        }
        return false;
    }

    void clear()
    {
        _points.clear();
        updateMeasurement();
    }


protected:
    void pick(float x, float y, osgViewer::Viewer* viewer)
    {        
        double w = 5.0;
        double h = 5.0;
        osgUtil::PolytopeIntersector *picker = new osgUtil::PolytopeIntersector(osgUtil::Intersector::WINDOW, x - w, y- h, x + w, y + h);
        osgUtil::IntersectionVisitor iv(picker);
        iv.setTraversalMask(MaskPointCloud);
        viewer->getCamera()->accept(iv);
        if (picker->containsIntersections())
        {
            // Get the point that was clicked.
            osgUtil::PolytopeIntersector::Intersection hit = picker->getFirstIntersection();
            osg::Geometry* geometry = hit.drawable->asGeometry();

            osg::Vec3Array* verts = static_cast<osg::Vec3Array*>(geometry->getVertexArray());
            // Since we are just drawing points the point that was clicked is just the primitive index.
            osg::Vec3 vert = (*verts)[hit.primitiveIndex];            

            osg::Matrixd localToWorld = osg::computeLocalToWorld(hit.nodePath);

            // Get the geocentric world coordinate
            osg::Vec3d world = vert * localToWorld;            

            _points.push_back( world );
            updateMeasurement();
        }
    }

    void updateMeasurement()
    {
        // Remove the line.
        if (_line.valid())
        {
            _root->removeChild( _line.get() );
            _line = 0;
        }

        if (_points.size() >=2 )
        {
            // Build the line
            osg::MatrixTransform* mt = new osg::MatrixTransform;
            osg::Vec3d anchor = _points[0];
            mt->setMatrix(osg::Matrixd::translate(anchor));

            osg::Geometry* geometry = new osg::Geometry;
            osg::Vec3Array* verts = new osg::Vec3Array;
            verts->reserve(_points.size());
            geometry->setVertexArray( verts );

            osg::Vec4Array* colors = new osg::Vec4Array(1);
            (*colors)[0] = osg::Vec4(1.0f, 0.0, 0.0f, 1.0f);            
            geometry->setColorArray( colors );
            geometry->setColorBinding( osg::Geometry::BIND_OVERALL);

            double dist = 0.0;

            for (unsigned int i = 0; i < _points.size(); i++)
            {
                if (i != 0)
                {
                    dist += (_points[i] - _points[i-1]).length();
                }
                verts->push_back(_points[i] - anchor );
            }

            geometry->addPrimitiveSet(new osg::DrawArrays(GL_LINE_STRIP, 0, verts->size()));

            osg::Geode* geode = new osg::Geode;
            geode->addDrawable(geometry);

            mt->addChild( geode );

            mt->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
            mt->getOrCreateStateSet()->setMode(GL_DEPTH_TEST, osg::StateAttribute::OFF);
            mt->getOrCreateStateSet()->setAttribute(new osg::LineWidth(2.0));
            _line = mt;

            _root->addChild( _line.get() );

            // Display the measurement
            std::stringstream buf;
            buf << "Distance = " << dist << " m";
            s_status->setText(buf.str());

        }
        else
        {
            s_status->setText("");   
        }
    }

    osg::ref_ptr< SpatialReference > _wgs84;
    std::vector< osg::Vec3d > _points;
    osg::ref_ptr< osg::Node > _line;
    osg::ref_ptr< osg::Group > _root;
};


int main(int argc, char** argv)
{    
    osg::ArgumentParser arguments(&argc,argv);
    
    osgViewer::Viewer viewer(arguments);

    viewer.setCameraManipulator( new EarthManipulator());

    osg::Group* root = MapNodeHelper().load(arguments, &viewer);

    osg::ref_ptr< MapNode > mapNode = MapNode::findMapNode(root);
    mapNode->getTerrainEngine()->setNodeMask(MaskMapNode);
    mapNode->getModelLayerGroup()->setNodeMask(MaskPointCloud);

    osg::ref_ptr< osg::Group > pointClouds = new osg::Group;
    pointClouds->setNodeMask(MaskPointCloud);

    for (unsigned int pos = 1; pos < arguments.argc(); pos++)
    {
        if (!arguments.isOption(pos))
        {
            osg::Node* pc = osgDB::readNodeFile(arguments[pos]);
            if (pc)
            {
                pointClouds->addChild(pc);
                OSG_NOTICE << "Loaded point cloud from " << arguments[pos] << std::endl;
            }
        }
    }

    pointClouds->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
    s_point = new osg::Point(1.0);
    root->getOrCreateStateSet()->setAttributeAndModes(s_point);

    root->addChild( pointClouds );
                 
    // any option left unread are converted into errors to write out later.
    arguments.reportRemainingOptionsAsUnrecognized();

    buildControls(viewer);

    bool measure = arguments.read("--measure");    

    if (measure)
    {
        OSG_NOTICE << "measuring" << std::endl;
    }
    else
    {
        OSG_NOTICE << "identifying" << std::endl;
    }
    if (!measure)
    {
        viewer.addEventHandler( new IdentifyPointHandler());
    }
    else
    {
        viewer.addEventHandler(new P2PMeasureHandler(root));
    }
  
    viewer.setSceneData( root );    

    return viewer.run();

}
