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

#include <osgJuniper/PointCloudTools>

#include <iostream>

using namespace osgJuniper;
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
        IdentifyPointHandler* identify = new IdentifyPointHandler();
        identify->setNodeMask(MaskPointCloud);
        viewer.addEventHandler(identify);
    }
    else
    {
        P2PMeasureHandler* measure = new P2PMeasureHandler(root);
        measure->setNodeMask(MaskPointCloud);
        viewer.addEventHandler(measure);
    }

    viewer.getCamera()->setNearFarRatio(0.00002);
  
    viewer.setSceneData( root );    

    return viewer.run();

}
