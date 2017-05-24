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

#include <string>
#include <iomanip>

#include <osg/io_utils>

#include <osg/Notify>
#include <osg/MatrixTransform>
#include <osg/AutoTransform>
#include <osg/LineWidth>
#include <osg/LineStipple>
#include <osg/ClusterCullingCallback>

#include <osgJuniper/Utils>
#include <osgJuniper/TrackHistoryNode>


#include <osgGA/StateSetManipulator>
#include <osgGA/GUIEventHandler>

#include <osgViewer/Viewer>
#include <osgViewer/ViewerEventHandlers>

#include <osgEarth/Map>
#include <osgEarth/MapNode>
#include <osgEarth/FindNode>
#include <osgEarth/Registry>

#include <osgEarthUtil/Controls>

#include <osgJuniper/Utils>

#include <osgEarthUtil/EarthManipulator>
#include <osgEarthUtil/ObjectLocator>

#include <osgEarthUtil/AutoClipPlaneHandler>

using namespace osgEarth;
using namespace osgEarth::Util::Controls;
using namespace osgEarth::Util;
using namespace osgJuniper;

struct PositionCallback : public osg::NodeCallback
{
public:
    PositionCallback():
      _position(0,0,0),
      _roll( 0 )
    {
    }

    void operator()(osg::Node* node, osg::NodeVisitor* nv) {
        osg::MatrixTransform* locator = static_cast< osg::MatrixTransform* >( node );
        osg::Vec3d newPosition = _position + osg::Vec3d(0.001, 0, 0);
        newPosition.z() = 5000;
        _position = newPosition;

        osg::ref_ptr< osg::EllipsoidModel > em = new osg::EllipsoidModel;
        osg::Matrixd mat;
        em->computeLocalToWorldTransformFromLatLongHeight( osg::DegreesToRadians( _position.y() ), osg::DegreesToRadians( _position.x() ), _position.z(), mat );
        //_roll += 5;

        //mat.preMultRotate( osg::Matrixd::rotate( osg::DegreesToRadians(_roll), osg::Vec3d(1,0,0) ).getRotate() );
        locator->setMatrix( mat );


        //locator->getLocator()->setPosition(  newPosition );

        //ObjectLocatorNode* locator = static_cast< ObjectLocatorNode* >( node );
        //osg::Vec3d newPosition = locator->getLocator()->getPosition() + osg::Vec3d(0.0001, 0, 0);
        //newPosition.z() = 5000;
        //locator->getLocator()->setPosition(  newPosition );
        traverse( node, nv );
    }

    osg::Vec3d _position;
    double _roll;
};

int main(int argc, char** argv)
{
    //Register our PrimitiveFactory.  By passing in true we tell the Registry to stick this PrimitiveFactory to the FRONT
    osg::ArgumentParser arguments(&argc,argv);

    osgViewer::Viewer viewer(arguments );

    osg::ref_ptr< osg::Node > loadedModel = osgDB::readNodeFiles( arguments );
    if (!loadedModel.valid()) {
        OSG_NOTICE << "Failed to load model" << std::endl;
        return 1;
    }

    MapNode* mapNode = MapNode::findMapNode( loadedModel );
    if (!mapNode) {
        OSG_NOTICE << "Terrain must be an earth file" << std::endl;
        return 1;
    }

   osg::ref_ptr< osg::Group > root = new osg::Group;
   root->addChild( loadedModel.get() );

    viewer.setSceneData( root.get() );

    EarthManipulator* manip = new EarthManipulator();
    viewer.setCameraManipulator( manip );

    std::string modelFilename = "cow.osg.1000,1000,1000.scale";
	while (arguments.read("--model",modelFilename));

    osg::ref_ptr<osg::Node> model = osgDB::readNodeFile( modelFilename );
    if ( !model.valid() )
    {
        OSG_NOTICE << "Failed to load model " << modelFilename << std::endl;
        return 1;
    }



    osg::MatrixTransform* objectLocator = new osg::MatrixTransform;
    //ObjectLocatorNode *objectLocator = new ObjectLocatorNode( mapNode->getMap() );
    objectLocator->addChild( model.get() );
    objectLocator->addUpdateCallback( new PositionCallback() );

    osgJuniper::TrackHistoryNode* history = new osgJuniper::TrackHistoryNode();
    history->setTrackNode( model );
    history->setMaxNumVertices( 1000000 );
    history->setRibbonWidth( 1000 );
    history->setLineWidth( 2 );
    history->setColor( osg::Vec4(1,1,0,0.5) );
    history->setStyle( TrackHistoryNode::STYLE_LINE );
    history->setMinDelta( 5 );
    history->setFade( false );
    root->addChild( objectLocator );
    root->addChild( history );

    manip->setTetherNode( objectLocator );


    // add some stock OSG handlers:
    viewer.addEventHandler(new osgViewer::StatsHandler());
    viewer.addEventHandler(new osgViewer::WindowSizeHandler());
    viewer.addEventHandler(new osgViewer::ThreadingHandler());
    viewer.addEventHandler(new osgGA::StateSetManipulator(viewer.getCamera()->getOrCreateStateSet()));
    viewer.addEventHandler(new osgViewer::HelpHandler(arguments.getApplicationUsage()));

    viewer.addEventHandler( new osgEarth::Util::AutoClipPlaneHandler );



    return viewer.run();
}
