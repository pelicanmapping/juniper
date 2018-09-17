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
#include <osgDB/ReadFile>
#include <osgUtil/Optimizer>
#include <osgUtil/PolytopeIntersector>


#include <osgViewer/Viewer>
#include <osgViewer/ViewerEventHandlers>

#include <osgGA/StateSetManipulator>

#include <osgEarth/MapNode>
#include <osgEarth/TerrainEngineNode>
#include <osgEarth/NodeUtils>
#include <osgEarthUtil/LatLongFormatter>
#include <osgEarthUtil/ExampleResources>
#include <osgEarthSymbology/Color>

#include <osgJuniper/PointCloud>
#include <osgJuniper/PointCloudTools>


#include <osgGA/TrackballManipulator>
#include <osgGA/FlightManipulator>
#include <osgGA/DriveManipulator>
#include <osgGA/KeySwitchMatrixManipulator>
#include <osgGA/StateSetManipulator>
#include <osgGA/AnimationPathManipulator>
#include <osgGA/TerrainManipulator>
#include <osgGA/SphericalManipulator>


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

struct MinHeightHandler : public ControlEventHandler
{
    MinHeightHandler( PointCloudDecorator* pointCloud ) : _pointCloud(pointCloud) { }
    void onValueChanged( Control* control, float value )
    {
        _pointCloud->setMinHeight(value);
        OSG_NOTICE << "Min Height " << value << std::endl;
    }

    osg::ref_ptr< PointCloudDecorator > _pointCloud;
};

struct MaxHeightHandler : public ControlEventHandler
{
    MaxHeightHandler( PointCloudDecorator* pointCloud ) : _pointCloud(pointCloud) { }
    void onValueChanged( Control* control, float value )
    {
        _pointCloud->setMaxHeight(value);
        OSG_NOTICE << "Max Height " << value << std::endl;
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
		osg::Vec3d world(point.x, point.y, point.z);
        geoPoint.fromWorld(wgs84, world);
        LatLongFormatter formatter;
        formatter.setPrecision(8);
        std::stringstream buf;
        buf << "Location: " << formatter.format(geoPoint) << ", " << geoPoint.z() << std::endl
            << "Classification: " << classificationToString(point.classification) << std::endl
            << "Intensity: " << point.intensity << std::endl
            << "RGBA: " << point.r << ", " << point.g << ", " << point.b << ", " << point.a << std::endl
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

    // Min Height
    HBox* minHeightBox = container->addControl(new HBox());
    minHeightBox->setChildVertAlign( Control::ALIGN_CENTER );
    minHeightBox->setChildSpacing( 10 );
    minHeightBox->setHorizFill( true );
    minHeightBox->addControl( new LabelControl("Min Height:", 16) );

    HSliderControl* minHeightSlider = minHeightBox->addControl(new HSliderControl(0.0f, 30.0, s_pointCloud->getMinHeight()));
    minHeightSlider->setBackColor( Color::Gray );
    minHeightSlider->setHeight( 12 );
    minHeightSlider->setHorizFill( true, 200 );
    minHeightSlider->addEventHandler( new MinHeightHandler(s_pointCloud));

    // Max Height
    HBox* maxHeightBox = container->addControl(new HBox());
    maxHeightBox->setChildVertAlign( Control::ALIGN_CENTER );
    maxHeightBox->setChildSpacing( 10 );
    maxHeightBox->setHorizFill( true );
    maxHeightBox->addControl( new LabelControl("Max Height:", 16) );

    HSliderControl* maxHeightSlider = maxHeightBox->addControl(new HSliderControl(0.0f, 100.0, s_pointCloud->getMaxHeight()));
    maxHeightSlider->setBackColor( Color::Gray );
    maxHeightSlider->setHeight( 12 );
    maxHeightSlider->setHorizFill( true, 200 );
    maxHeightSlider->addEventHandler( new MaxHeightHandler(s_pointCloud));


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

    LabelControl* height = new LabelControl("Height");
    height->addEventHandler(new ChangeColorModeHandler(PointCloudDecorator::Height));
    toolbar->setControl(3, 0, height);

    LabelControl* ramp = new LabelControl("Ramp");
    ramp->addEventHandler(new ChangeColorModeHandler(PointCloudDecorator::Ramp));
    toolbar->setControl(4, 0, ramp);

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

    root->addChild(canvas);
}

int main(int argc, char** argv)
{
    osg::ArgumentParser arguments(&argc,argv);

    osgViewer::Viewer viewer(arguments);
  
    osg::Group* root = new osg::Group;    

    osg::Node* loaded = osgDB::readNodeFiles(arguments);
    root->addChild(loaded);

    osg::ref_ptr< MapNode > mapNode = MapNode::findMapNode(loaded);
    if (mapNode.valid())
    {
        mapNode->getTerrainEngine()->setNodeMask(MaskMapNode);
        mapNode->getLayerNodeGroup()->setNodeMask(MaskPointCloud);
        viewer.setCameraManipulator( new EarthManipulator());
    }
    else
    {
        osg::ref_ptr<osgGA::KeySwitchMatrixManipulator> keyswitchManipulator = new osgGA::KeySwitchMatrixManipulator;

        keyswitchManipulator->addMatrixManipulator('1', "Trackball", new osgGA::TrackballManipulator());
        keyswitchManipulator->addMatrixManipulator('2', "Flight", new osgGA::FlightManipulator());
        keyswitchManipulator->addMatrixManipulator('3', "Drive", new osgGA::DriveManipulator());
        keyswitchManipulator->addMatrixManipulator('4', "Terrain", new osgGA::TerrainManipulator());
        keyswitchManipulator->addMatrixManipulator('5', "Orbit", new osgGA::OrbitManipulator());
        keyswitchManipulator->addMatrixManipulator('6', "FirstPerson", new osgGA::FirstPersonManipulator());
        keyswitchManipulator->addMatrixManipulator('7', "Spherical", new osgGA::SphericalManipulator());

        std::string pathfile;
        double animationSpeed = 1.0;
        while (arguments.read("--speed", animationSpeed)) {}
        char keyForAnimationPath = '8';
        while (arguments.read("-p", pathfile))
        {
            osgGA::AnimationPathManipulator* apm = new osgGA::AnimationPathManipulator(pathfile);
            if (apm || !apm->valid())
            {
                apm->setTimeScale(animationSpeed);

                unsigned int num = keyswitchManipulator->getNumMatrixManipulators();
                keyswitchManipulator->addMatrixManipulator(keyForAnimationPath, "Path", apm);
                keyswitchManipulator->selectMatrixManipulator(num);
                ++keyForAnimationPath;
            }
        }

        viewer.setCameraManipulator(keyswitchManipulator.get());
    }


    s_pointCloud = osgEarth::findTopMostNodeOfType<PointCloudDecorator>(loaded);
    if (!s_pointCloud)
    {
        OSG_NOTICE << "Cannot find point cloud" << std::endl;
        return 1;
    }    

    std::string colorRampFile;
    if (arguments.read("--colorramp", colorRampFile))
    {
        // Set the color ramp to use.
        osg::Texture2D* colorRamp = new osg::Texture2D(osgDB::readImageFile(colorRampFile));
        colorRamp->setResizeNonPowerOfTwoHint(false);
        colorRamp->setWrap(osg::Texture::WRAP_S, osg::Texture::CLAMP_TO_EDGE);
        colorRamp->setWrap(osg::Texture::WRAP_T, osg::Texture::CLAMP_TO_EDGE);
        s_pointCloud->setColorRamp(colorRamp);
    }

    // any option left unread are converted into errors to write out later.
    arguments.reportRemainingOptionsAsUnrecognized();

    buildControls(viewer, root);

    bool measure = arguments.read("--measure");
    bool identify = arguments.read("--identify");

    if (identify)
    {
        OSG_NOTICE << "identifying" << std::endl;
        IdentifyPointHandler* identify = new IdentifyPointHandler();
        identify->addCallback( new IdentifyCallback(s_status));
        identify->setNodeMask(MaskPointCloud);
        viewer.addEventHandler(identify);
    }

    if (measure)
    {
        OSG_NOTICE << "measuring" << std::endl;
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
