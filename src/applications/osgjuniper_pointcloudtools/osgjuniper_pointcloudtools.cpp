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

#include <osgViewer/Viewer>
#include <osgViewer/ViewerEventHandlers>

#include <osgGA/StateSetManipulator>

#include <osgJuniper/Utils>

#include <osgEarth/MapNode>
#include <osgEarthUtil/ExampleResources>
#include <osgEarthSymbology/Color>

#include <iostream>

using namespace osgJuniper;
using namespace osgEarth;
using namespace osgEarth::Util;

static osg::Point* s_point;

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
}


// Handler to identify a point's location
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

            osg::Vec3d world = vert * localToWorld;            

            osgEarth::GeoPoint point;
            point.fromWorld(_wgs84.get(), world);

            OSG_NOTICE << "Lat Long=" << point.y() << ", " << point.x() << ", " << point.z() << std::endl;            
            
        }
        else
        {
            OSG_NOTICE << "No hits :(" << std::endl;
        }
    }



    osg::ref_ptr< SpatialReference > _wgs84;
    osg::ref_ptr< osg::Vec4ubArray > _prevColorArray;
    osg::Vec4ub _prevSelectedColor;
    int _prevSelectedIndex;
    osg::Vec4ub _selectedColor;    



};


int main(int argc, char** argv)
{    
    osg::ArgumentParser arguments(&argc,argv);
    
    osgViewer::Viewer viewer(arguments);

    viewer.setCameraManipulator( new EarthManipulator());

    osg::Group* root = MapNodeHelper().load(arguments, &viewer);

    osg::ref_ptr< MapNode > mapNode = MapNode::findMapNode(root);
    mapNode->setNodeMask(MaskMapNode);

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
    pointClouds->getOrCreateStateSet()->setAttributeAndModes(s_point);

    root->addChild( pointClouds );

             
    // any option left unread are converted into errors to write out later.
    arguments.reportRemainingOptionsAsUnrecognized();

    buildControls(viewer);

    viewer.addEventHandler( new IdentifyPointHandler());
  
    viewer.setSceneData( root );    

    return viewer.run();

}
