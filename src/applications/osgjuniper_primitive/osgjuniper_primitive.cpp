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

#include <string>
#include <iomanip>
#include <sstream>

#include <osg/io_utils>

#include <osg/Notify>
#include <osg/MatrixTransform>
#include <osg/AutoTransform>
#include <osg/LineWidth>
#include <osg/LineStipple>
#include <osg/ClusterCullingCallback>

#include <osgJuniper/Utils>


#include <osgGA/StateSetManipulator>
#include <osgGA/GUIEventHandler>

#include <osgViewer/Viewer>
#include <osgViewer/ViewerEventHandlers>

#include <osgEarth/GeoData>
#include <osgEarth/Map>
#include <osgEarth/MapNode>
#include <osgEarth/FindNode>
#include <osgEarth/GeoMath>
#include <osgEarth/Registry>
#include <osg/CoordinateSystemNode>
#include <osgEarth/TerrainEngineNode>

#include <osgEarthFeatures/Feature>
#include <osgEarthFeatures/TransformFilter>
#include <osgEarthFeatures/BuildGeometryFilter>
#include <osgEarthFeatures/ResampleFilter>

#include <osgEarthUtil/Controls>

#include <osgEarthSymbology/Geometry>

#include <osgJuniper/Utils>

#include <osgEarthUtil/EarthManipulator>
#include <osgEarth/Viewpoint>

#include <osgJuniperMap/Primitive>
#include <osgJuniperMap/TextPrimitive>
#include <osgJuniperMap/BitmapPrimitive>
#include <osgJuniperMap/ModelPrimitive>
#include <osgJuniperMap/PolylinePrimitive>
#include <osgJuniperMap/ImageOverlayPrimitive>
#include <osgJuniperMap/BoxPrimitive>
#include <osgJuniperMap/EllipsoidPrimitive>
#include <osgJuniperMap/RangeRingPrimitive>
#include <osgJuniperMap/MeshPrimitive>
#include <osgJuniperMap/Registry>

using namespace osgEarth;
using namespace osgEarth::Util::Controls;
using namespace osgEarth::Util;
using namespace osgEarth::Symbology;
using namespace osgEarth::Features;
using namespace osgJuniper;
using namespace osgJuniper::Map;


PrimitiveId nextId()
{
    static PrimitiveId id = 0;
    return id++;
}


//Create a PrimitiveFactory to handle creating primitives
class MisspellPrimitiveFactory : public PrimitiveFactory
{
public:
    virtual Primitive* create(const std::string& type, PrimitiveId id)
    {
        //Based on the given type we can decide what type of primitive to create.
        //For example, here is a way that we could handle creating some common mispellings of Primitive types
        if (type == "Bitemap" || type == "Bitmop") return new BitmapPrimitive(id);
        if (type == "Polylin" || type == "Plyline") return new PolylinePrimitive(id);

        //Return NULL, other PrimitiveFactorys will have the chance to create primitives
        return NULL;
    }
};

MeshPrimitive* createMesh1()
{
    MeshPrimitive* t = new MeshPrimitive(nextId());
    std::vector< osg::Vec3 > verts;
    std::vector< osg::Vec4 > colors;

    float alpha = 0.5;
    verts.push_back( osg::Vec3(0, 0, 0 ) );
    colors.push_back( osg::Vec4(0,1,0,alpha) );
    verts.push_back( osg::Vec3(1000, 0, 0 ) );
    colors.push_back( osg::Vec4(0,1,0,alpha) );
    verts.push_back( osg::Vec3(1000, 1000, 0 ) );
    colors.push_back( osg::Vec4(0,1,0,alpha) );

    verts.push_back( osg::Vec3(0, 0, 0 ) );
    colors.push_back( osg::Vec4(1,0,0,alpha) );
    verts.push_back( osg::Vec3(1000, 1000, 0 ) );
    colors.push_back( osg::Vec4(1,0,0,alpha) );
    verts.push_back( osg::Vec3(0, 1000, 0 ) );
    colors.push_back( osg::Vec4(1,0,0,alpha) );

    t->setVerts( verts );
    t->setColors( colors );
    t->setProperty("depthtest", (bool)false);

    return t;
}

MeshPrimitive* createMesh2()
{
    MeshPrimitive* t = new MeshPrimitive(nextId());
    std::vector< osg::Vec3 > verts;

    std::vector< osg::Vec2 > texCoords;    

    osg::Vec4 color(1,1,1,1);
    verts.push_back( osg::Vec3(0, 0, 0 ) );
    texCoords.push_back( osg::Vec2(0, 0) );
    verts.push_back( osg::Vec3(1000, 0, 0 ) );    
    texCoords.push_back( osg::Vec2(1, 0) );
    verts.push_back( osg::Vec3(1000, 1000, 0 ) );    
    texCoords.push_back( osg::Vec2(1, 1) );
    verts.push_back( osg::Vec3(0, 1000, 0 ) );
    texCoords.push_back( osg::Vec2(0, 1) );    
    

    t->setTexCoords( texCoords );
    t->setImage( osgDB::readImageFile( "c:\\temp\\robot.gif" ) );
    
    t->setVerts( verts );
    t->setColor( color );
    t->setMode(GL_QUADS);

    t->setProperty("depthtest", false);

    return t;
}

MeshPrimitive* createMesh3()
{
    MeshPrimitive* t = new MeshPrimitive(nextId());
    std::vector< osg::Vec3 > verts;        
    std::vector< osg::Vec4 > colors;

    verts.push_back( osg::Vec3(0, 0, 0 ) );    
    colors.push_back( osg::Vec4(1,0,0,1) );
    verts.push_back( osg::Vec3(1000, 0, 0 ) );    
    colors.push_back( osg::Vec4(0,1,0,1) );
    verts.push_back( osg::Vec3(1000, 1000, 0 ) );    
    colors.push_back( osg::Vec4(0,0,1,1) );
    verts.push_back( osg::Vec3(0, 1000, 0 ) );
    colors.push_back( osg::Vec4(1,0,1,1) );

    std::vector< int > indices;
    indices.push_back( 0 ); indices.push_back( 1 ); indices.push_back( 2 );
    indices.push_back( 0 ); indices.push_back( 2 ); indices.push_back( 3 );
    
    t->setVerts( verts );
    t->setColors( colors );
    t->setIndices( indices );
    t->setMode(GL_TRIANGLES);
    t->setProperty("depthtest", false);

    return t;
}

MeshPrimitive* createMesh4()
{
    MeshPrimitive* t = new MeshPrimitive(nextId());
    std::vector< osg::Vec3 > verts;        

    unsigned int numVerts = 1000;
    double length = 1000;
    double step = length / (double)numVerts;

    verts.reserve( numVerts );

    double freq = 250;
    double exag = 250;

    for (unsigned int i = 0; i < numVerts; i++)
    {       
        double x = 0;
        double y = step * (double)i;
        double z = (sin((y / freq) * osg::PI * 2) + 1) * exag;
        verts.push_back( osg::Vec3(x,y,z) );
    }
    
    t->setVerts( verts );
    t->setColor( osg::Vec4f(0,0,1,1) );
    t->setMode(GL_LINE_STRIP);

    return t;
}


struct PositionHandler : public osgGA::GUIEventHandler
{

    PositionHandler(Primitive* primitive, osgEarth::MapNode* mapNode)   :
_primitive(primitive),
_mapNode( mapNode) 
	{		
	}

	bool handle( const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa )
    {	
		osgViewer::View* view = dynamic_cast<osgViewer::View*>(&aa);
		/*if (ea.getEventType() == osgGA::GUIEventAdapter::PUSH && 
			ea.getButton() == osgGA::GUIEventAdapter::MIDDLE_MOUSE_BUTTON &&
			ea.getModKeyMask() & osgGA::GUIEventAdapter::MODKEY_CTRL)*/
        if (ea.getEventType() == osgGA::GUIEventAdapter::MOVE)
		{
            osg::NodePath path;
            path.push_back( _mapNode->getTerrainEngine() );
			osgUtil::LineSegmentIntersector::Intersections intersections;
			if (view->computeIntersections(ea.getX(), ea.getY(),path,intersections ))
			{
				osg::Vec3d hit = intersections.begin()->getWorldIntersectPoint();
				double h;
                osg::ref_ptr< osg::EllipsoidModel> em = new osg::EllipsoidModel;
                double lat, lon;
				em->convertXYZToLatLongHeight(hit.x(), hit.y(), hit.z(), lat, lon, h);
                lat = osg::RadiansToDegrees(lat);
                lon = osg::RadiansToDegrees(lon);
                //_primitive->setProperty("lat", osg::RadiansToDegrees(lat));
                //_primitive->setProperty("lon", osg::RadiansToDegrees(lon));
                ModelPrimitive* model = dynamic_cast<ModelPrimitive*>(_primitive.get());
                if (model)
                {
                    model->setLocation(Location(lat, lon, 0));
                }
				return true;
			}
		}
		
        return false;
    }

	

	osg::ref_ptr< Primitive > _primitive;	
    osg::ref_ptr< osgEarth::MapNode > _mapNode;
};





int main(int argc, char** argv)
{    
    //Register our PrimitiveFactory.  By passing in true we tell the Registry to stick this PrimitiveFactory to the FRONT
    //of the list so it gets a chance to respond to createPrimitive requests before other PrimitiveFactorys.
    osgJuniper::Map::Registry::instance()->addPrimitiveFactory( new MisspellPrimitiveFactory(), true);

    //Try to create some primitives using the mispellings
    osg::ref_ptr< Primitive> p;
    p = osgJuniper::Map::Registry::instance()->createPrimitive("Bitemap", 0);
    if (p.valid()) OSG_NOTICE << "createPrimitive succeeded from mispelled word, PrimitiveFactory working!" << std::endl;

    p = osgJuniper::Map::Registry::instance()->createPrimitive("bitmap", 0);
    if (p.valid()) OSG_NOTICE << "createPrimitive succeeded from lowercase word" << std::endl;

    p = osgJuniper::Map::Registry::instance()->createPrimitive("biTMap", 0);
    if (p.valid()) OSG_NOTICE << "createPrimitive succeeded from garbled word" << std::endl;


    
    osg::ArgumentParser arguments(&argc,argv);

    osgViewer::Viewer viewer(arguments );

    osg::ref_ptr< osg::Node > loadedModel = osgDB::readNodeFiles( arguments );

    osg::ref_ptr< osg::Group > root = new osg::Group;

    osgEarth::MapNode* mapNode = osgEarth::MapNode::findMapNode( loadedModel.get() );

    osg::ref_ptr< MapContext > elevationContext = new MapContext( root.get(), mapNode, &viewer );

    root->addChild( loadedModel.get() );

    ControlCanvas* canvas = new ControlCanvas( &viewer );
    root->addChild( canvas );   
    

   
    bool declutter = true;
    TextPrimitive *text = new TextPrimitive(nextId());
    text->setMapContext( elevationContext );
    text->setForegroundColor( osg::Vec4(1,0,0,1) );
    text->setBackgroundColor( osg::Vec4(0,1,0,0.5) );
    text->setLocation(Location(0, 0, 1000 ) );
    text->setAltitudeMode(osgEarth::AltitudeMode::ALTMODE_RELATIVE);
    text->setText("Hello");
    root->addChild( text );

    {
        TextPrimitive *text = new TextPrimitive(nextId());
        text->setMapContext( elevationContext );
        text->setForegroundColor( osg::Vec4(1,1,0,1) );
        text->setBackgroundColor( osg::Vec4(0,1,0,0.5) );
        text->setLocation(Location(40.71, -74, 1000.0 ) );
        text->setAltitudeMode(osgEarth::AltitudeMode::ALTMODE_RELATIVE);
        text->setText("New York City");        
        root->addChild( text );
    }

    {
        TextPrimitive *text = new TextPrimitive(nextId());
        text->setMapContext( elevationContext );
        text->setForegroundColor( osg::Vec4(1,1,0,1) );
        text->setBackgroundColor( osg::Vec4(0,1,0,0.5) );
        text->setLocation(Location(38.85, -77.04, 1000.0 ) );
        text->setAltitudeMode(osgEarth::AltitudeMode::ALTMODE_RELATIVE);
        text->setText("Washington DC");        
        root->addChild( text );
    }

    BoxPrimitive *box = new BoxPrimitive(nextId());
    box->setMapContext( elevationContext );
    box->setFillColor( osg::Vec4f(1,0,0,0.2));
    box->setOutlineColor( osg::Vec4f(1,1,1,1));
    box->setLocation( Location(50,50,1000));
    box->setSize(osg::Vec3d(1000,1000,1000) );
    box->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
    root->addChild( box );

    EllipsoidPrimitive *ellipsoid = new EllipsoidPrimitive(nextId());
    ellipsoid->setMapContext( elevationContext );
    ellipsoid->setFillColor( osg::Vec4f(1,0,0,0.2));
    ellipsoid->setOutlineColor( osg::Vec4f(1,1,1,1));
    ellipsoid->setLocation( Location(50,50,1000));
    ellipsoid->setSize(osg::Vec3d(1000,1000,1000) );
    ellipsoid->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
    root->addChild( ellipsoid );

    RangeRingPrimitive *rangeRing = new RangeRingPrimitive(nextId());
    rangeRing->setMapContext( elevationContext );
    rangeRing->setColor( osg::Vec4f(1,0,0,1));    
    rangeRing->setLocation( Location(50,50,1000));
    rangeRing->setRange( 1000 );
    rangeRing->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
    root->addChild( rangeRing );


    /*
    BitmapPrimitive *bitmap = new BitmapPrimitive(nextId());
    //bitmap->setSize( 64, 64 );
    bitmap->setFilename("c:\\temp\\animated.gif");
    bitmap->setLocation( Location(41, -99, 0) );
    root->addChild( bitmap );    
    //bitmap->setRange(0, 500000);
    bitmap->setMapContext( elevationContext );
    //viewer.addEventHandler( new PositionPrimitiveHandler( bitmap ) );
*/

    ModelPrimitive *model = new ModelPrimitive(nextId());
    root->addChild( model );
    model->setMapContext( elevationContext );
    model->setScale(50, 50, 50);
    model->setLocation(Location(0, 0, 0 ) );
    model->setFilename("cow.osg");
    model->setAltitudeMode( AltitudeMode::ALTMODE_RELATIVE );
    model->setOrientToGround(true);
    //model->setScale(50, 50, 50);
    //viewer.addEventHandler( new PositionPrimitiveHandler( model ) );
    //model->setRange(0, 300000);    

    PolylinePrimitive* polyline = new PolylinePrimitive(nextId());
    polyline->setGeometry( new LineString() );
    polyline->setColor( osg::Vec4f(1,1,0,1));    
    polyline->setMapContext( elevationContext );
    polyline->setColor( osg::Vec4(1,0,0,1));
    root->addChild( polyline );

    //viewer.addEventHandler( new DrawPolylineHandler( polyline ) );


    //Create a few meshes
        
    MeshPrimitive* mesh1 = createMesh1();
    mesh1->setLocation( Location(41, -99, 0) );
    mesh1->setMapContext( elevationContext );
    root->addChild( mesh1 );

    MeshPrimitive* mesh2 = createMesh2();
    mesh2->setLocation( Location(41.01, -99, 0) );
    mesh2->setMapContext( elevationContext );
    root->addChild( mesh2 );

    MeshPrimitive* mesh3 = createMesh3();
    mesh3->setLocation( Location(41.03, -99, 0) );
    mesh3->setMapContext( elevationContext );
    root->addChild( mesh3 );

    MeshPrimitive* mesh4 = createMesh4();
    mesh4->setLocation( Location(41.05, -99, 0) );
    mesh4->setMapContext( elevationContext );
    root->addChild( mesh4 );    



    viewer.setSceneData( root.get() );

    EarthManipulator* manip = new EarthManipulator();
    viewer.setCameraManipulator( manip );

    //manip->setTetherNode( model );
    
    // add some stock OSG handlers:
    viewer.addEventHandler(new osgViewer::StatsHandler());
    viewer.addEventHandler(new osgViewer::WindowSizeHandler());
    viewer.addEventHandler(new osgViewer::ThreadingHandler());
    viewer.addEventHandler(new osgGA::StateSetManipulator(viewer.getCamera()->getOrCreateStateSet()));
    viewer.addEventHandler(new osgViewer::HelpHandler(arguments.getApplicationUsage()));

    //Disable small feature culling
    osg::CullStack::CullingMode cullingMode = viewer.getCamera()->getCullingMode();
    cullingMode &= ~(osg::CullStack::SMALL_FEATURE_CULLING);
    viewer.getCamera()->setCullingMode( cullingMode );

    //viewer.addEventHandler(new PositionHandler(model, mapNode));
    viewer.realize();

    unsigned int frameNum = 0;

    while (!viewer.done())
    {
        viewer.frame();        
        frameNum++;
        std::stringstream buf;
        buf << "Frame " << frameNum;
        text->setText(buf.str());
        /*
        if (frameNum % 100 == 0)
        {
            OSG_NOTICE << "Setting mode" << std::endl;
            trimesh->setMode( trimesh->getMode() == GL_POINTS ? GL_TRIANGLES : GL_POINTS );
        }*/
    }
    return 0;
}
