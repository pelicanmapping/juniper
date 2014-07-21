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

#include <osg/Notify>
#include <osg/io_utils>

#include <osgDB/ReadFile>
#include <osgGA/StateSetManipulator>
#include <osgViewer/Viewer>
#include <osgViewer/ViewerEventHandlers>

#include <osgEarth/MapNode>
#include <osgEarthUtil/EarthManipulator>

#include <osgEarthUtil/Controls>
#include <osgEarthUtil/ObjectLocator>

#include <osgEarthUtil/AutoClipPlaneHandler>

#include <osgJuniperMap/Articulation>
#include <osgJuniper/Utils>


using namespace osgJuniper;
using namespace osgJuniper::Map;
using namespace osgSim;
using namespace osgEarth;
using namespace osgEarth::Util;
using namespace osgEarth::Util::Controls;

static Grid* s_componentsBox = NULL;


osg::Node*
createControlPanel( osgViewer::View* view )
{
    ControlCanvas* canvas = ControlCanvas::get( view );

    // the outer container:
    s_componentsBox = new Grid();
    s_componentsBox->setBackColor(0,0,0,0.5);
    s_componentsBox->setMargin( 10 );
    s_componentsBox->setPadding( 10 );
    s_componentsBox->setChildSpacing( 10 );
    s_componentsBox->setChildVertAlign( Control::ALIGN_CENTER );
    s_componentsBox->setAbsorbEvents( true );
    s_componentsBox->setVertAlign( Control::ALIGN_BOTTOM );

    canvas->addControl( s_componentsBox );    
    return canvas;
}

struct RotationHandler : public ControlEventHandler
{
    RotationHandler( ArticulationController* controller, const osg::Vec3d& rotationAxis) : 
      _controller( controller), 
      _rotationAxis(rotationAxis)
      { }

    void onValueChanged( Control* control, float value ) {
        osg::notify(osg::NOTICE) << "Setting rotation to " << value << std::endl;
        _controller->setRotation( _rotationAxis * osg::DegreesToRadians(value) );        
    }
    osg::ref_ptr< ArticulationController > _controller;
    osg::Vec3d _rotationAxis;
};

struct EnabledHandler : public ControlEventHandler
{
    EnabledHandler( osg::Node* node ) :  _node( node ) { }
    void onValueChanged( Control* control, bool value ) {
        _node->setNodeMask( value ? ~0 : 0 );
    }
    osg::Node* _node;
};

struct PositionCallback : public osg::NodeCallback
{
public:
    PositionCallback()
    {
    }    
    
    void operator()(osg::Node* node, osg::NodeVisitor* nv) {
        ObjectLocatorNode* locator = static_cast< ObjectLocatorNode* >( node );
        osg::Vec3d newPosition = locator->getLocator()->getPosition() + osg::Vec3d(0.001, 0, 0);
        locator->getLocator()->setPosition(  newPosition );
        traverse( node, nv );
    }
};


void addComponent( osg::Node* model,  const std::string& name, const osg::Vec3d& rotationOffset, const osg::Vec3d& rotationAxis)
{   
    static unsigned int component = 0;
    osg::Node* node = osgJuniper::Utils::findNamedNode( model, name );
    if (node)
    {        
        LabelControl* nameLabel = new LabelControl( name );      
        nameLabel->setVertAlign( Control::ALIGN_CENTER );
        s_componentsBox->setControl( 0, component, nameLabel );

        //The rotation slider
        HSliderControl* rotation = new HSliderControl(0.0f, 360.0f, 0.0f);
        rotation->setWidth( 125 );
        rotation->setHeight( 12 );
        rotation->setVertAlign( Control::ALIGN_CENTER );

        ArticulationController* controller = new ArticulationController( node );
        controller->setRotationOffset( rotationOffset );
        rotation->addEventHandler( new RotationHandler( controller, rotationAxis ) );
        s_componentsBox->setControl( 1, component, rotation );

        //Add some controls        
        CheckBoxControl* enabled = new CheckBoxControl( true );
        enabled->addEventHandler( new EnabledHandler( node ) );        
        enabled->setVertAlign( Control::ALIGN_CENTER );
        s_componentsBox->setControl( 2, component, enabled );
        component++;       
    }
}


int
usage( const std::string& msg )
{
    if ( !msg.empty() )
    {
        std::cout << msg << std::endl;
    }

    std::cout
        << std::endl
        << "USAGE: osgjuniper_articulation" << std::endl
        << std::endl
        << "    --globe file.earth                  ; The .earth file to load" << std::endl
        << "    --model filename                    ; The model to articulate" << std::endl        
        << "    --tank" << std::endl
        << std::endl;

    return -1;
}


int main(int argc, char** argv)
{
    osg::ArgumentParser arguments(&argc,argv);    	

    if (arguments.argc() <= 1)
    {
        return usage("Please specify a .earth file");
    }

    osg::ref_ptr< osg::Node > terrain = osgDB::readNodeFiles( arguments );
    if (!terrain.valid())
    {
        return usage("Could not load specified file");    
    }

    osgEarth::MapNode* mapNode = osgEarth::MapNode::findMapNode( terrain );
    if (!mapNode)
    {
        return usage("Please specify a .earth file");
    }

    std::string modelFilename;
	while (arguments.read("--model",modelFilename));

    bool tank = false;
    while (arguments.read("--tank")) tank = true;
    if (modelFilename.empty())
    {
        if (tank)
        {
            modelFilename = "C:/dev/Mocu_Stuff/Avatars/2S19/tank_dof.flt";
        }
        else
        {
            modelFilename = "c:/dev/Mocu_Stuff/Avatars/Packbot/packbot.osg";      
        }
    }


    osg::ref_ptr<osg::Node> loadedModel = osgDB::readNodeFile( modelFilename );       
    if ( !loadedModel.valid() )
        return 1;

    ObjectLocatorNode *objectLocator = new ObjectLocatorNode( mapNode->getMap() );
    objectLocator->addChild( loadedModel );
    objectLocator->addUpdateCallback( new PositionCallback() );

   
    // assemble the rest of the scene graph and go
    osgViewer::Viewer viewer(arguments);

    EarthManipulator* manip = new EarthManipulator();
    viewer.setCameraManipulator( manip );
    manip->setTetherNode( objectLocator );
    if (!tank)
    {
        objectLocator->getLocator()->setOrientation( osg::Vec3d(0, 90, 0 ) );
    }
    objectLocator->getLocator()->setPosition( osg::Vec3d(5, 5, 0) );

    osg::Group*  root = new osg::Group;

    root->addChild( terrain );
    root->addChild( objectLocator );
    root->addChild( createControlPanel(&viewer) );

    if (!tank)
    {    
        addComponent(loadedModel.get(), "LFlip", osg::Vec3d(0,0,0), osg::Vec3d(1,0,0));
        addComponent(loadedModel.get(), "RFlip", osg::Vec3d(0,0,0), osg::Vec3d(1,0,0));
        addComponent(loadedModel.get(), "LArm",  osg::Vec3d(177.6,0,0), osg::Vec3d(1,0,0));
        addComponent(loadedModel.get(), "MArm",  osg::Vec3d(-175.0,0,0), osg::Vec3d(1,0,0));
        addComponent(loadedModel.get(), "UArm",  osg::Vec3d(174.8,0,0), osg::Vec3d(1,0,0));
        addComponent(loadedModel.get(), "MCamMnt",  osg::Vec3d(-89.4,0,0), osg::Vec3d(1,0,0));
        addComponent(loadedModel.get(), "MCam", osg::Vec3d(0,0,0), osg::Vec3d(1,0,0));
        addComponent(loadedModel.get(), "GripMnt", osg::Vec3d(0,-289.5,0), osg::Vec3d(0,1,0));
        addComponent(loadedModel.get(), "LGrip", osg::Vec3d(0,0,-16), osg::Vec3d(0,0,1));
        addComponent(loadedModel.get(), "RGrip", osg::Vec3d(0,0,16), osg::Vec3d(0,0,1));
        addComponent(loadedModel.get(), "MastRot", osg::Vec3d(0,0,0), osg::Vec3d(0,0,1));

    }   
    else if (tank)
    {
        //addComponent(loadedModel.get(), "Gun", osg::Vec3d(0,0,0), osg::Vec3d(1,0,0));
        //addComponent(loadedModel.get(), "Turret", osg::Vec3d(0,0,0), osg::Vec3d(0,0,1));

        addComponent(loadedModel.get(), "GunDOF", osg::Vec3d(0,0,0), osg::Vec3d(0,1,0));
        addComponent(loadedModel.get(), "TurretDOF", osg::Vec3d(0,0,0), osg::Vec3d(1,0,0));
    }


    

    //root->addChild( loadedModel );
    viewer.setSceneData(root);
        
    viewer.addEventHandler( new osgGA::StateSetManipulator(viewer.getCamera()->getOrCreateStateSet()) );
    viewer.addEventHandler(new osgViewer::StatsHandler);    
    viewer.getCamera()->addCullCallback( new osgEarth::Util::AutoClipPlaneCullCallback(mapNode) );

    return viewer.run();
}
