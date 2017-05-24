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
#include <osg/CoordinateSystemNode>
#include <osg/Point>

#include <osg/Switch>
#include <osg/LOD>
#include <osg/Program>
#include <osgText/Text>

#include <osgViewer/Viewer>
#include <osgViewer/ViewerEventHandlers>

#include <osgGA/TrackballManipulator>
#include <osgGA/FlightManipulator>
#include <osgGA/DriveManipulator>
#include <osgGA/KeySwitchMatrixManipulator>
#include <osgGA/StateSetManipulator>
#include <osgGA/AnimationPathManipulator>
#include <osgGA/TerrainManipulator>
#include <osgGA/SphericalManipulator>

#include <iostream>

#include <osgDB/FileNameUtils>
#include <osgDB/FileUtils>

#include <osg/PagedLOD>

#include <osgJuniper/Octree>

#include <cctype>
#include <iomanip>
#include <string>

#include <osgJuniper/PointCloud>
#include <osgEarth/NodeUtils>

using namespace osgJuniper;

struct PointCloudHandler : public osgGA::GUIEventHandler
{
    PointCloudHandler( PointCloudDecorator* pointCloud ) : _pointCloud(pointCloud) { }

    bool handle( const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa )
    {
        if ( ea.getEventType() == ea.KEYDOWN)
        {
            switch (ea.getKey())
            {
            case 'u':
                _pointCloud->setPointSize(_pointCloud->getPointSize()+0.2);
                OSG_NOTICE << "Point size " << _pointCloud->getPointSize() << std::endl;
                break;
            case 'U':
                _pointCloud->setPointSize(_pointCloud->getPointSize()-0.2);
                OSG_NOTICE << "Point size " << _pointCloud->getPointSize() << std::endl;
                break;
            case 'c':
                {
                    int colorMode = _pointCloud->getColorMode();
                    colorMode++;
                    if (colorMode >=3) colorMode = 0;
                    _pointCloud->setColorMode((PointCloudDecorator::ColorMode)colorMode);
                }
                break;
            case 'r':
                _pointCloud->setMaxReturn(_pointCloud->getMaxReturn() + 1);
                OSG_NOTICE << "Max return " << _pointCloud->getMaxReturn() << std::endl;
                break;
            case 'R':
                _pointCloud->setMaxReturn(_pointCloud->getMaxReturn() -1);
                OSG_NOTICE << "Max return " << _pointCloud->getMaxReturn() << std::endl;
                break;
            case 'n':
                _pointCloud->setMinIntensity(_pointCloud->getMinIntensity()-1);
                OSG_NOTICE << "Intensity range " << _pointCloud->getMinIntensity() << " to " << _pointCloud->getMaxIntensity() << std::endl;
                break;
            case 'N':
                _pointCloud->setMinIntensity(_pointCloud->getMinIntensity()+1);
                OSG_NOTICE << "Intensity range " << _pointCloud->getMinIntensity() << " to " << _pointCloud->getMaxIntensity() << std::endl;
                break;
            case 'm':
                _pointCloud->setMaxIntensity(_pointCloud->getMaxIntensity()+1);
                OSG_NOTICE << "Intensity range " << _pointCloud->getMinIntensity() << " to " << _pointCloud->getMaxIntensity() << std::endl;
                break;
            case 'M':
                _pointCloud->setMaxIntensity(_pointCloud->getMaxIntensity()-1);
                OSG_NOTICE << "Intensity range " << _pointCloud->getMinIntensity() << " to " << _pointCloud->getMaxIntensity() << std::endl;
                break;
            case 'v':
                _pointCloud->setClassificationVisible(3, !_pointCloud->getClassificationVisible(3));
                _pointCloud->setClassificationVisible(4, !_pointCloud->getClassificationVisible(4));
                _pointCloud->setClassificationVisible(5, !_pointCloud->getClassificationVisible(5));
            default:
                break;
            }
        }
        return false;
    }

    osg::observer_ptr< PointCloudDecorator > _pointCloud;
};



int main(int argc, char** argv)
{
    osgDB::Registry::instance()->addFileExtensionAlias("laz", "las");

    // use an ArgumentParser object to manage the program arguments.
    osg::ArgumentParser arguments(&argc,argv);

    osgViewer::Viewer viewer(arguments);
    // add the state manipulator
    viewer.addEventHandler( new osgGA::StateSetManipulator(viewer.getCamera()->getOrCreateStateSet()) );

    osg::ref_ptr<osgGA::KeySwitchMatrixManipulator> keyswitchManipulator = new osgGA::KeySwitchMatrixManipulator;
    {


        keyswitchManipulator->addMatrixManipulator( '1', "Trackball", new osgGA::TrackballManipulator() );
        keyswitchManipulator->addMatrixManipulator( '2', "Flight", new osgGA::FlightManipulator() );
        keyswitchManipulator->addMatrixManipulator( '3', "Drive", new osgGA::DriveManipulator() );
        keyswitchManipulator->addMatrixManipulator( '4', "Terrain", new osgGA::TerrainManipulator() );

        std::string pathfile;
        char keyForAnimationPath = '5';
        while (arguments.read("-p",pathfile))
        {
            osgGA::AnimationPathManipulator* apm = new osgGA::AnimationPathManipulator(pathfile);
            if (apm || !apm->valid())
            {
                unsigned int num = keyswitchManipulator->getNumMatrixManipulators();
                keyswitchManipulator->addMatrixManipulator( keyForAnimationPath, "Path", apm );
                keyswitchManipulator->selectMatrixManipulator(num);
                ++keyForAnimationPath;
            }
        }

        viewer.setCameraManipulator( keyswitchManipulator.get() );
    }



    // add the window size toggle handler
    viewer.addEventHandler(new osgViewer::WindowSizeHandler);

    // add the stats handler
    viewer.addEventHandler(new osgViewer::StatsHandler);

    // add the LOD Scale handler
    viewer.addEventHandler(new osgViewer::LODScaleHandler);

    // add the screen capture handler
    viewer.addEventHandler(new osgViewer::ScreenCaptureHandler);

    // add the record camera path handler
    viewer.addEventHandler(new osgViewer::RecordCameraPathHandler);

    // Load all the models
    osg::Node* loaded = osgDB::readNodeFiles(arguments);
    PointCloudDecorator* pointCloud = osgEarth::findTopMostNodeOfType<PointCloudDecorator>(loaded);
    if (!pointCloud)
    {
        PointCloudDecorator* pointCloud = new PointCloudDecorator();
        pointCloud->addChild(loaded);
    }

    viewer.addEventHandler(new PointCloudHandler(pointCloud));

    // any option left unread are converted into errors to write out later.
    arguments.reportRemainingOptionsAsUnrecognized();

    // report any errors if they have occurred when parsing the program arguments.
    if (arguments.errors())
    {
        arguments.writeErrorMessages(std::cout);
        return 1;
    }



    viewer.setSceneData( pointCloud );

    viewer.getCamera()->setNearFarRatio(0.00002);

    return viewer.run();

}
