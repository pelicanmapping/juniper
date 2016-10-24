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

#include <osg/Point>
#include <osgDB/ReadFile>
#include <osgUtil/Optimizer>
#include <osg/CoordinateSystemNode>

#include <osgJuniper/IBRCameraNode>
#include <osgJuniper/IBRManipulator>

#include <osg/Switch>
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

#include <osgEarth/Utils>
#include <osgEarth/NodeUtils>
#include <osgEarth/MapNode>
#include <osgEarthUtil/EarthManipulator>
#include <osgEarthUtil/AutoClipPlaneHandler>

#include <iostream>

#include <osg/MatrixTransform>
#include <osg/Timer>

using namespace osgEarth;
using namespace osgEarth::Util;
using namespace osgJuniper;

struct GoToTheBeachHandler : public osgGA::GUIEventHandler 
{
    GoToTheBeachHandler( EarthManipulator* manip ) : _manip(manip) { }

    bool handle( const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa )
    {
        if ( ea.getEventType() == ea.KEYDOWN && ea.getKey() == 'y' )
        {
            _manip->setViewpoint( Viewpoint( "Ocean Beach",   -117.248881, 32.745299, 0.0, 0.0, -90.0, 500.0), 5.0 );
        }
        return false;
    }

    osg::observer_ptr<EarthManipulator> _manip;
};

struct ModelPositionCallback : public osgGA::GUIEventHandler
{

	ModelPositionCallback(osg::MatrixTransform *transform, osg::CoordinateSystemNode* csn, bool enabled=false):
    _transform(transform),
     _csn(csn),
	 _scale(0.137),
	 _heading(133.75),
	 _pitch(3),
	 _roll(-4.5),
	 _lat(osg::DegreesToRadians(32.7448)),
	 _lon(osg::DegreesToRadians(-117.248)),
	 _height(2.35),
	 _pointSize(1.0f),
	 _enabled(enabled)
	{
		update();
	}

	bool handle( const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa )
    {

		if (!_enabled) return false;

		osgViewer::View* view = dynamic_cast<osgViewer::View*>(&aa);
		if (ea.getEventType() == osgGA::GUIEventAdapter::PUSH && 
			ea.getButton() == osgGA::GUIEventAdapter::MIDDLE_MOUSE_BUTTON &&
			ea.getModKeyMask() & osgGA::GUIEventAdapter::MODKEY_CTRL)
		{
			osgUtil::LineSegmentIntersector::Intersections intersections;
			if (view->computeIntersections(ea.getX(), ea.getY(), intersections))
			{
				osg::Vec3d hit = intersections.begin()->getWorldIntersectPoint();
				double h;
				_csn->getEllipsoidModel()->convertXYZToLatLongHeight(hit.x(), hit.y(), hit.z(), _lat, _lon, h);
				update();
				return true;
			}
		}
		else if (ea.getEventType() == osgGA::GUIEventAdapter::KEYDOWN)
		{
			if (ea.getKey() == 'u')
			{
				_scale += 0.001;
				update();
				return true;
			}
			else if (ea.getKey() == 'j')
			{
				_scale -= 0.001;
				update();
				return true;
			}
			else if (ea.getKey() == 'h')
			{
				_heading += 0.05;
				update();
				return true;
			}
			else if (ea.getKey() == 'H')
			{
				_heading -= 0.05;
				update();
				return true;
			}
			else if (ea.getKey() == 'p')
			{
				_pitch += 0.05;
				update();
				return true;
			}
			else if (ea.getKey() == 'P')
			{
				_pitch -= 0.05;
				update();
				return true;
			}
			else if (ea.getKey() == 'r')
			{
				_roll += 0.05;
				update();
				return true;
			}
			else if (ea.getKey() == 'R')
			{
				_roll -= 0.05;
				update();
				return true;
			}
			else if (ea.getKey() == osgGA::GUIEventAdapter::KEY_Up)
			{
				_height += 0.05;
				update();
				return true;
			}
			else if (ea.getKey() == osgGA::GUIEventAdapter::KEY_Down)
			{
				_height -= 0.05;
				update();
				return true;
			}
			else if (ea.getKey() == 'k')
			{
				_pointSize += 0.1f;
				updatePointSize();
				return true;
			}
			else if (ea.getKey() == 'K')
			{
				_pointSize -= 0.1f;
				if (_pointSize <= 0) _pointSize = 0.1f;
				updatePointSize();
				return true;
			}

		}
        return false;
    }

	void update()
	{
		osg::Matrixd localToWorld;
		_csn->getEllipsoidModel()->computeLocalToWorldTransformFromLatLongHeight(_lat, _lon, _height, localToWorld);
		
	    //Scale the matrix
		localToWorld.preMult( osg::Matrixd::scale( _scale, _scale, _scale ) );

		osg::Matrix rot_mat;
		rot_mat.makeRotate( 
			osg::DegreesToRadians(_pitch), osg::Vec3(1,0,0),
			osg::DegreesToRadians(_heading), osg::Vec3(0,0,1),
			osg::DegreesToRadians(_roll), osg::Vec3(0,1,0) );
		localToWorld.preMult( rot_mat );

		_transform->setMatrix( localToWorld );

		osg::notify(osg::NOTICE) << "Model Pos      = " << osg::RadiansToDegrees(_lat) << ", " << osg::RadiansToDegrees(_lon) << ", " << _height
			                     << "Model Scale    = " << _scale << std::endl
		                         << "Model HPR = "   << _heading << ", " << _pitch << ", " << _roll << std::endl;
	}

	void updatePointSize()
	{
		osg::Point* point = static_cast<osg::Point*>(_transform->getOrCreateStateSet()->getAttribute(osg::StateAttribute::POINT));
		if (!point) point = new osg::Point();
		point->setSize(_pointSize);
		_transform->getOrCreateStateSet()->setAttributeAndModes(point, osg::StateAttribute::ON);
	}

	osg::ref_ptr< osg::MatrixTransform > _transform;
	osg::ref_ptr< osg::CoordinateSystemNode > _csn;

	double _lat;
	double _lon;
	double _height;
	double _heading;
	double _pitch;
	double _roll;
	double _scale;
	float _pointSize;
	bool _enabled;
};

struct IBREventHandler : public osgGA::GUIEventHandler
{
	IBREventHandler(CameraList& cameraList):
_cameraList(cameraList)
	{
	}

	bool handle( const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa )
	{
		if (ea.getEventType() == osgGA::GUIEventAdapter::KEYDOWN)
		{
			if (ea.getKey() == '+')
			{
				float s = _cameraList[0]->getScale();
				s += 0.2;
				osg::notify(osg::NOTICE) << "Scale=" << s << std::endl;

				//Increase the scale of the cameras
				for (unsigned int i = 0; i < _cameraList.size(); ++i)
				{
					_cameraList[i]->setScale(s);
				}
				return true;
			}
			else if (ea.getKey() == '-')
			{
				float s = _cameraList[0]->getScale();
				s -= 0.2;
				osg::notify(osg::NOTICE) << "Scale=" << s << std::endl;

				//Increase the scale of the cameras
				for (unsigned int i = 0; i < _cameraList.size(); ++i)
				{
					_cameraList[i]->setScale(s);
				}
				return true;
			}
			else if (ea.getKey() == 'm')
			{
				//Toggle the frustums
				for (unsigned int i = 0; i < _cameraList.size(); ++i)
				{
					_cameraList[i]->setShowFrustum(!_cameraList[i]->getShowFrustum());
				}
				return true;
			}
		}
		return false;
	}

	CameraList _cameraList;
};


int main(int argc, char** argv)
{
    // use an ArgumentParser object to manage the program arguments.
    osg::ArgumentParser arguments(&argc,argv);

    arguments.getApplicationUsage()->setApplicationName(arguments.getApplicationName());
    arguments.getApplicationUsage()->setDescription(arguments.getApplicationName()+" is the standard OpenSceneGraph example which loads and visualises 3d models.");
    arguments.getApplicationUsage()->setCommandLineUsage(arguments.getApplicationName()+" [options] filename ...");
    arguments.getApplicationUsage()->addCommandLineOption("--image <filename>","Load an image and render it on a quad");
    arguments.getApplicationUsage()->addCommandLineOption("--dem <filename>","Load an image/DEM and render it on a HeightField");
    arguments.getApplicationUsage()->addCommandLineOption("--login <url> <username> <password>","Provide authentication information for http file access.");

    osgViewer::Viewer viewer(arguments);

    unsigned int helpType = 0;
    if ((helpType = arguments.readHelpType()))
    {
        arguments.getApplicationUsage()->write(std::cout, helpType);
        return 1;
    }

    // report any errors if they have occurred when parsing the program arguments.
    if (arguments.errors())
    {
        arguments.writeErrorMessages(std::cout);
        return 1;
    }

    
	EarthManipulator* earthManip = new EarthManipulator();

	earthManip->getSettings()->setMinMaxPitch(-89.9, -1);

	viewer.setCameraManipulator(earthManip);

    // add the state manipulator
    viewer.addEventHandler( new osgGA::StateSetManipulator(viewer.getCamera()->getOrCreateStateSet()) );

    // add the window size toggle handler
    viewer.addEventHandler(new osgViewer::WindowSizeHandler);

    // add the stats handler
    viewer.addEventHandler(new osgViewer::StatsHandler);
	

	osg::Group* root = new osg::Group;

	std::string globeFile = "C:/dev/osgearth/tests/ReadyMapNoElevation.earth";
	while (arguments.read("--globe",globeFile));
	while (arguments.read("-g",globeFile));
	osg::notify(osg::NOTICE) << "globeFile=" << globeFile << std::endl;

	//Load the globe
	osg::Node* globe = osgDB::readNodeFile(globeFile);
	root->addChild( globe );

	osg::CoordinateSystemNode* csn = findTopMostNodeOfType<osg::CoordinateSystemNode>(globe);
	if (csn)
	{
		osg::notify(osg::NOTICE) << "Found CSN" << std::endl;
	}

    osg::ref_ptr< osgEarth::MapNode > mapNode = osgEarth::MapNode::findMapNode( globe );

    osg::ref_ptr<AutoClipPlaneCullCallback> autoClip = new AutoClipPlaneCullCallback(mapNode);
    viewer.getCamera()->addCullCallback( autoClip.get() );    


  std::string pointcloud;
	while (arguments.read("--pc",pointcloud));
	while (arguments.read("-c",pointcloud));

  osg::Group* data = new osg::Group;

  if (!pointcloud.empty()) {
      osg::Node* node = osgDB::readNodeFile(pointcloud);
      data->addChild(node);
  }


	//Load the IBR scene
	std::string ibrFile = "C:/dev/juniper/data/alley/bundle.out.juniper_bundler";
	while (arguments.read("--ibr",ibrFile));
	while (arguments.read("-i",ibrFile));
	osg::notify(osg::NOTICE) << "ibrFile=" << ibrFile << std::endl;

	osg::Node* ibr = osgDB::readNodeFile(ibrFile);
    data->addChild(ibr);

	osg::MatrixTransform *ibrTransform = new osg::MatrixTransform();
	ibrTransform->addChild( data );
	root->addChild( ibrTransform );

	IBRCameraFinder cf;
	ibr->accept(cf);
	for (int i = 0; i < cf._cameraList.size(); ++i) {
      //Set the opacity and scale of all the cameras
      cf._cameraList[i]->setOpacity( 0.0f );
      //cf._cameraList[i]->setScale(42.0);
      cf._cameraList[i]->setShowFrustum(false);
      for (int j = 0; j < cf._cameraList[i]->getDescriptions().size(); ++j)
          osg::notify(osg::NOTICE) << cf._cameraList[i]->getDescriptions()[j] << std::endl;
	}

//	CameraManipulator* cm = new CameraManipulator;
	IBRManipulator* cm = new IBRManipulator();
	// set up the camera manipulators.
	osg::ref_ptr<osgGA::KeySwitchMatrixManipulator> keyswitchManipulator = new osgGA::KeySwitchMatrixManipulator;

	keyswitchManipulator->addMatrixManipulator( '1', "Earth", earthManip );
	keyswitchManipulator->addMatrixManipulator( '2', "Bundler", cm);
	viewer.setCameraManipulator( keyswitchManipulator.get() );
	cm->setCameraList(cf._cameraList);

	root->addChild(cm->getGraph());

	viewer.addEventHandler(new IBREventHandler(cf._cameraList));


	viewer.addEventHandler(new ModelPositionCallback(ibrTransform, csn ));

    // any option left unread are converted into errors to write out later.
    arguments.reportRemainingOptionsAsUnrecognized();

    // report any errors if they have occurred when parsing the program arguments.
    if (arguments.errors())
    {
        arguments.writeErrorMessages(std::cout);
        return 1;
    }

	earthManip->getSettings()->bindMouseDoubleClick(
		EarthManipulator::ACTION_GOTO,
		osgGA::GUIEventAdapter::LEFT_MOUSE_BUTTON );

	viewer.addEventHandler(new GoToTheBeachHandler( earthManip ));


    viewer.setSceneData( root );


    viewer.realize();

	//viewer.getCamera()->setComputeNearFarMode( osg::CullSettings::DO_NOT_COMPUTE_NEAR_FAR );    

    //return viewer.run();
	while (!viewer.done())
	{
		if (keyswitchManipulator->getCurrentMatrixManipulator() == cm)
		{
			if (autoClip.valid())
			{				
				autoClip = NULL;
                viewer.getCamera()->removeCullCallback( autoClip.get() );
			}
			double fovy, ar, znear, zfar;
			viewer.getCamera()->getProjectionMatrixAsPerspective( fovy, ar, znear, zfar );
			znear = 0.25;
			zfar = 40000;
			viewer.getCamera()->setProjectionMatrixAsPerspective( fovy, ar, znear, zfar );
			//osg::notify(osg::NOTICE) << "near=" << znear << ", far=" << zfar << std::endl;
		}
		else
		{
			if (!autoClip.valid())
			{
				autoClip = new AutoClipPlaneCullCallback(mapNode);
                viewer.getCamera()->addCullCallback( autoClip.get() );    
				
			}
			//viewer.getCamera()->setComputeNearFarMode( osg::CullSettings::COMPUTE_NEAR_FAR_USING_BOUNDING_VOLUMES );
		}
		viewer.frame();	
	}
	return 0;

}
