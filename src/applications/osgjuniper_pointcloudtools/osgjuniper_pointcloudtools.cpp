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


#include <osgViewer/Viewer>
#include <osgViewer/ViewerEventHandlers>

#include <osgGA/StateSetManipulator>

#include <osgEarth/MapNode>
#include <osgEarth/TerrainEngineNode>
#include <osgEarthUtil/LatLongFormatter>
#include <osgEarthUtil/ExampleResources>
#include <osgEarthSymbology/Color>

#include <osgJuniper/PointCloud>
#include <osgJuniper/PointCloudTools>

#include <iostream>

using namespace osgJuniper;
using namespace osgEarth;
using namespace osgEarth::Util;

static PointCloudDecorator* s_pointCloud;

static LabelControl* s_status;

osg::Node::NodeMask MaskMapNode = 0x01;
osg::Node::NodeMask MaskPointCloud = 0x02;

struct PointSizeHandler : public ControlEventHandler
{
    PointSizeHandler( PointCloudDecorator* pointCloud ) : _pointCloud(pointCloud) { }
    void onValueChanged( Control* control, float value )
    {        
        _pointCloud->setPointSize(value);
        OSG_NOTICE << "Point size " << value << std::endl;
    }
    osg::ref_ptr< PointCloudDecorator > _pointCloud;
};

struct MaxIntensityHandler : public ControlEventHandler
{
    MaxIntensityHandler( PointCloudDecorator* pointCloud ) : _pointCloud(pointCloud) { }
    void onValueChanged( Control* control, float value )
    {        
        _pointCloud->setMaxIntensity(value);
        OSG_NOTICE << "Max Intensity " << value << std::endl;
    }

    osg::ref_ptr< PointCloudDecorator > _pointCloud;
};

// http://resources.arcgis.com/en/help/main/10.1/index.html#//015w0000005q000000
std::string classificationToString(unsigned short classification)
{

    switch (classification)
    {
    case 0:
        return "Never classified";
    case 1:
        return "Unassigned";
    case 2:
        return "Ground";
    case 3:
        return "Low Vegetation";
    case 4:
        return "Medium Vegetation";
    case 5:
        return "High Vegetation";
    case 6:
        return "Building";
    case 7:
        return "Noise";
    case 8:
        return "Model Key";
    case 9:
        return "Water";
    case 10:
        return "Reserved for ASPRS Definition";
    case 11:
        return "Reserved for ASPRS Definition";
    case 12:
        return "Overlap";
    default:
        return "Reserved fro ASPRS Definition";
    };
}

/**
 * Callback for when a point is identified.  Updates a label with info about the point.
 */
struct IdentifyCallback : public IdentifyPointHandler::Callback
{
    IdentifyCallback(LabelControl* label):
_label(label)
    {
    }

    virtual void selected(const Point& point)
    {
        // Assume the point is in geocentric
        osgEarth::SpatialReference* wgs84 = osgEarth::SpatialReference::create("epsg:4326");
        osgEarth::GeoPoint geoPoint;
        geoPoint.fromWorld(wgs84, point.position);
        LatLongFormatter formatter;
        formatter.setPrecision(8);
        std::stringstream buf;
        buf << "Location: " << formatter.format(geoPoint) << ", " << geoPoint.z() << std::endl
            << "Classification: " << classificationToString(point.classification) << std::endl
            << "Intensity: " << point.intensity << std::endl
            << "RGBA: " << (int)point.color.r() << ", " << (int)point.color.g() << ", " << (int)point.color.b() << ", " << (int)point.color.a() << std::endl
            << "Return: " << (int)point.returnNumber << std::endl;

        s_status->setText( buf.str() );     
        
    }

    virtual void reset()
    {
        s_status->setText("");
    }

    LabelControl* _label;
};

struct P2PMeasureCallback : public P2PMeasureHandler::Callback
{
    virtual void distanceChanged(double distance)
    {
        std::stringstream buf;
        buf << "Distance: " << distance << "m";
        s_status->setText(buf.str());
    }
};

struct ChangeColorModeHandler : public ControlEventHandler
{
    ChangeColorModeHandler(PointCloudDecorator::ColorMode colorMode):
_colorMode(colorMode)
    {
    }

    void onClick(Control* control, int mouseButtonMask)
    {
        s_pointCloud->setColorMode(_colorMode);
    }

    PointCloudDecorator::ColorMode _colorMode;
};

struct ToggleClassificationHandler : public ControlEventHandler
{
    ToggleClassificationHandler(unsigned char classification):
_classification(classification)
{
}

void onValueChanged(Control* control, bool value)
{
    s_pointCloud->setClassificationVisible(_classification, value);
}

unsigned char _classification;
};

struct AutoPointSizeHandler : public ControlEventHandler
{
    void onValueChanged(Control* control, bool value)
    {
        s_pointCloud->setAutoPointSize(!s_pointCloud->getAutoPointSize());
    }
};


void buildControls(osgViewer::Viewer& viewer, osg::Group* root)
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
    pointSlider->addEventHandler( new PointSizeHandler(s_pointCloud));   

    // Max Intensity
    HBox* maxIntensityBox = container->addControl(new HBox());
    maxIntensityBox->setChildVertAlign( Control::ALIGN_CENTER );
    maxIntensityBox->setChildSpacing( 10 );
    maxIntensityBox->setHorizFill( true );
    maxIntensityBox->addControl( new LabelControl("Max Intensity:", 16) );

    //HSliderControl* intensitySlider = maxIntensityBox->addControl(new HSliderControl(0.0f, USHRT_MAX, s_pointCloud->getMaxIntensity()));
    HSliderControl* intensitySlider = maxIntensityBox->addControl(new HSliderControl(0.0f, 255.0, 255.0));
    intensitySlider->setBackColor( Color::Gray );
    intensitySlider->setHeight( 12 );
    intensitySlider->setHorizFill( true, 200 );
    intensitySlider->addEventHandler( new MaxIntensityHandler(s_pointCloud));   


    // Color mode
    Grid* toolbar = new Grid();    
    toolbar->setAbsorbEvents( true );    

    LabelControl* rgb = new LabelControl("RGB");
    rgb->addEventHandler(new ChangeColorModeHandler(PointCloudDecorator::RGB));
    toolbar->setControl(0, 0, rgb);

    LabelControl* intensity = new LabelControl("Intensity");
    intensity->addEventHandler(new ChangeColorModeHandler(PointCloudDecorator::Intensity));
    toolbar->setControl(1, 0, intensity);

    LabelControl* classifiction = new LabelControl("Classification");
    classifiction->addEventHandler(new ChangeColorModeHandler(PointCloudDecorator::Classification));
    toolbar->setControl(2, 0, classifiction);   

    container->addChild(toolbar);
    
    HBox* box = container->addControl(new HBox());
    CheckBoxControl* vegToggle = box->addControl(new CheckBoxControl(true));
    vegToggle->addEventHandler(new ToggleClassificationHandler(3));
    vegToggle->addEventHandler(new ToggleClassificationHandler(4));
    vegToggle->addEventHandler(new ToggleClassificationHandler(5));
    box->addControl(new LabelControl("Vegetation"));

    box = container->addControl(new HBox());
    CheckBoxControl* buildingToggle = box->addControl(new CheckBoxControl(true));
    buildingToggle->addEventHandler(new ToggleClassificationHandler(6));
    box->addControl(new LabelControl("Buildings"));

    box = container->addControl(new HBox());
    CheckBoxControl* groundToggle = box->addControl(new CheckBoxControl(true));
    groundToggle->addEventHandler(new ToggleClassificationHandler(2));
    box->addControl(new LabelControl("Ground"));

    box = container->addControl(new HBox());
    CheckBoxControl* autoPointSizeToggle = box->addControl(new CheckBoxControl(s_pointCloud->getAutoPointSize()));
    autoPointSizeToggle->addEventHandler(new AutoPointSizeHandler());
    box->addControl(new LabelControl("Auto Point Size"));
        
    // Add a status label
    s_status = container->addControl(new LabelControl());

    //root->addChild(canvas);
}

int main(int argc, char** argv)
{    
    osg::ArgumentParser arguments(&argc,argv);
    
    osgViewer::Viewer viewer(arguments);

    viewer.setCameraManipulator( new EarthManipulator());

    osg::Group* root = new osg::Group;

    osg::Node* loaded = osgEarth::Util::MapNodeHelper().load(arguments, &viewer);//osgDB::readNodeFiles(arguments);
    root->addChild(loaded);

    osg::ref_ptr< MapNode > mapNode = MapNode::findMapNode(loaded);
    mapNode->getTerrainEngine()->setNodeMask(MaskMapNode);
    mapNode->getModelLayerGroup()->setNodeMask(MaskPointCloud);

    s_pointCloud = osgEarth::findTopMostNodeOfType<PointCloudDecorator>(loaded);
    if (!s_pointCloud)
    {
        OSG_NOTICE << "Cannot find point cloud" << std::endl;
        return 1;
    }

    // any option left unread are converted into errors to write out later.
    arguments.reportRemainingOptionsAsUnrecognized();

    buildControls(viewer, root);

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
        identify->addCallback( new IdentifyCallback(s_status));
        identify->setNodeMask(MaskPointCloud);
        viewer.addEventHandler(identify);          
    }
    else
    {
        P2PMeasureHandler* measure = new P2PMeasureHandler(root);
        measure->addCallback(new P2PMeasureCallback());
        measure->setNodeMask(MaskPointCloud);
        viewer.addEventHandler(measure);
    }


    viewer.addEventHandler(new osgViewer::StatsHandler());
    viewer.addEventHandler(new osgViewer::WindowSizeHandler());
    viewer.addEventHandler(new osgViewer::ThreadingHandler());
    viewer.addEventHandler(new osgViewer::LODScaleHandler());
    viewer.addEventHandler(new osgGA::StateSetManipulator(viewer.getCamera()->getOrCreateStateSet()));
    viewer.addEventHandler(new osgViewer::RecordCameraPathHandler());

    viewer.getCamera()->setNearFarRatio(0.00002);
  
    viewer.setSceneData( root );    

    return viewer.run();

}
