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

#include <osgText/Text>
#include <osg/Geode>
#include <osg/Group>
#include <osg/MatrixTransform>
#include <osg/io_utils>

#include <osg/Sequence>
#include <osgDB/ReadFile>
#include <osgDB/FileUtils>
#include <osgDB/FileNameUtils>

#include <osgGA/StateSetManipulator>

#include <osgViewer/Viewer>
#include <osgViewer/ViewerEventHandlers>

#include <osgJuniper/Utils>
#include <osgJuniper/StreamingNode>

#include <osgEarth/MapNode>
#include <osgEarth/GeoMath>
#include <osgEarth/GeoData>
#include <osgEarthUtil/EarthManipulator>

#include <iostream>

using namespace osgEarth;
using namespace osgJuniper;

/**
* A StreamingNodeSource that has position information on each frame
*/
class PositionedStreamingNodeSource : public StreamingNodeSource, public OpenThreads::Thread
{
public:
    PositionedStreamingNodeSource(osgEarth::MapNode* mapNode);

    virtual void startImplementation();
    virtual void stopImplementation();

    virtual void run();
    virtual int cancel();

    /**
    * Gets the number of points that this source will stream
    */
    unsigned int getNumPoints() const { return _numPoints; }

    /**
    * Sets the number of points that this source will stream.
    */
    void setNumPoints( unsigned int numPoints ) { _numPoints = numPoints; }        

    typedef std::vector< osg::Vec3d> LocationList;

    /**
    *Get the locations for each frame in this source.  Normally this information could come from an actual
    *GPS sensor on the device
    */
    LocationList& getLocations() { return _locations;}

    /**
     * How long a frame lasts in seconds
     */
    double getFrameTime() const { return _frameTime;}
    void setFrameTime(double frameTime) { _frameTime = frameTime;}

    unsigned int getNumSegments() const {return _numSegments;}
    void setNumSegments( unsigned int numSegments ) { _numSegments = numSegments;}

    unsigned int getNumSlices() const {return _numSlices;}
    void setNumSlices( unsigned int numSlices ) { _numSlices = numSlices;}

    double getRadius() const { return _radius;}
    void setRadius( double radius) { _radius = radius;}

protected:        

    /**
    * Utility method for creating a node with the given points and colors.
    */
    osg::Node* makePoints(osg::Vec3Array* points, osg::Vec4Array* colors, unsigned int numPoints);

    ~PositionedStreamingNodeSource();

    LocationList _locations;
    int _frame;
    bool _done;
    unsigned int _numPoints;
    double _frameTime;

    unsigned int _numSegments;
    unsigned int _numSlices;
    double _radius;

    osg::ref_ptr< osgEarth::MapNode > _mapNode;
};

PositionedStreamingNodeSource::PositionedStreamingNodeSource(osgEarth::MapNode* mapNode):
_numPoints( 10000 ),
_done(false),
_frame( 0 ),
_mapNode( mapNode ),
_frameTime(0.1), //Frame time in seconds,
_numSlices( 40 ),
_numSegments( 100 ),
_radius( 500 )
{
}

PositionedStreamingNodeSource::~PositionedStreamingNodeSource()
{
    cancel();
}

void
PositionedStreamingNodeSource::startImplementation()
{
    _done = false;
    startThread();
}

void
PositionedStreamingNodeSource::stopImplementation()
{
    cancel();
}

void
PositionedStreamingNodeSource::run()
{
    do
    {        
        osg::ref_ptr< osg::Group > root = new osg::Group;        

        //We try to use the same batchsize to promote OSG's VBO reuse which can help quite a bit with graphics memory.  If you use the same
        //batch size for a VBO OSG can reuse OpenGL buffer objects more effeciently.
        unsigned int batchSize = 10000;
        osg::ref_ptr< osg::Vec3Array > verts;
        osg::ref_ptr< osg::Vec4Array > colors;

        //Generate some random points
        unsigned int j = 0;

        for (unsigned int i = 0; i < _numSegments; ++i)
        {
            double randVal = ((double)rand() / (double)RAND_MAX);

            //If the random number is > 0.5 then scale the hit.  Otherwise assume no hit.
            double range = _radius;
            if (randVal > 0.5)
            {
                range = randVal * _radius;
            }            
            
            double angle = (double)i * (osg::PI * 2.0 / (double)_numSlices);
            double x = range * cos( angle );
            double y = range * sin( angle );
            for (unsigned int k = 0; k < _numSlices; k++)
            {
                if (!verts.valid() || !colors.valid())
                {
                    verts = new osg::Vec3Array( batchSize );
                    colors = new osg::Vec4Array( batchSize );
                }                
                double z = (_radius / (double)_numSlices) * (double)k;
                (*verts)[j]  = osg::Vec3d(x, y, z );
                (*colors)[j] = osg::Vec4(1,0,0,1);//Utils::randomColor();
                j++;

                if (j == batchSize)
                {
                    root->addChild( makePoints( verts, colors, j) );
                    j = 0;
                    verts = 0;
                    colors = 0;
                }
            }
        }

        if (verts.valid())
        {
            root->addChild( makePoints( verts, colors, j) );
        }

        //Root now contains all of the points that are scaled and positioned locally.
        //Now we position the node in the world.  The locations for each frame are all in lon, lat, elevation format
        const SpatialReference* geoSRS = _mapNode->getMapSRS()->getGeographicSRS();
        osg::Vec3d location = _locations[_frame];
        GeoPoint pos( geoSRS, location.x(), location.y(), location.z(), osgEarth::ALTMODE_ABSOLUTE);
        _frame++;
        if (_frame >= _locations.size()) _frame = 0;

        osg::Matrixd localToWorld;
        pos.createLocalToWorld( localToWorld );

        osg::ref_ptr< osg::MatrixTransform > posMatrix = new osg::MatrixTransform;
        posMatrix->setMatrix( localToWorld );
        posMatrix->addChild( root.get() );

        frame( posMatrix );


        //Sleep for little bit
        OpenThreads::Thread::microSleep( _frameTime * 1000 * 1000 );        
    } while (!_done && !testCancel());
}

int
PositionedStreamingNodeSource::cancel()
{
    _done = true;
    while (isRunning())
    {               
        OpenThreads::Thread::YieldCurrentThread();
    }
    return 0;
}

osg::Node*
PositionedStreamingNodeSource::makePoints(osg::Vec3Array* points, osg::Vec4Array* colors, unsigned int numPoints)
{
    osg::Geometry *geometry = new osg::Geometry;
    geometry->setVertexArray( points );
    geometry->setUseDisplayList( false );
    geometry->setUseVertexBufferObjects( true );

    geometry->setColorBinding( osg::Geometry::BIND_PER_VERTEX );
    geometry->setColorArray( colors );

    geometry->addPrimitiveSet( new osg::DrawArrays(GL_POINTS, 0, numPoints ) );

    osg::Geode *geode = new osg::Geode;
    geode->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
    geode->addDrawable( geometry );
    return geode;    
}

int main( int argc, char **argv )
{   
    // use an ArgumentParser object to manage the program arguments.
    osg::ArgumentParser arguments(&argc,argv);

    unsigned int history = 0;
    while (arguments.read("--history", history)){};    

    // construct the viewer.
    osgViewer::Viewer viewer(arguments);

    osg::ref_ptr < osg::Group > root = new osg::Group;

    osg::ref_ptr< osg::Node > terrain = osgDB::readNodeFiles( arguments );
    if (!terrain.valid())
    {
        std::cout << "Could not load specified file" << std::endl;
        return 0;
    }

    osgEarth::MapNode* mapNode = osgEarth::MapNode::findMapNode( terrain );
    if (!mapNode)
    {
        std::cout << "Please specify a .earth file" << std::endl;
        return 0;
    }
    
    //Add the map to the scene graph
    root->addChild( mapNode );

    //Create a new PositionedStreamingNodeSource and give it some positions
    osg::ref_ptr< PositionedStreamingNodeSource > source = new PositionedStreamingNodeSource( mapNode );
    
    //Start in the gaslamp
    double startLon = -117.1623;
    double startLat = 32.7155;

    //End around old town
    double endLon = -117.2048;
    double endLat = 32.7595;


    int numFrames = 30;//10.0 / source->getFrameTime(); //Animation lasts 10 seconds
    OSG_NOTICE << numFrames << std::endl;
    double dt = 1.0 / numFrames;
    for (unsigned int i = 0; i < numFrames; i++)
    {
        double latRad, lonRad;
        GeoMath::interpolate(osg::DegreesToRadians( startLat ), osg::DegreesToRadians( startLon ),
                             osg::DegreesToRadians( endLat ), osg::DegreesToRadians( endLon ),
                             (double)i * dt,
                             latRad, lonRad );
        source->getLocations().push_back( osg::Vec3d( osg::RadiansToDegrees( lonRad ), osg::RadiansToDegrees( latRad ), 0 ) );
    }
    
    source->startStreaming();
    StreamingNode* streamingNode = new StreamingNode( source );
    streamingNode->setHistory( history );
    root->addChild( streamingNode );

    // add model to viewer.
    viewer.setSceneData(root);

    osgEarth::Util::EarthManipulator* manip = new osgEarth::Util::EarthManipulator();
    manip->setHomeViewpoint(Viewpoint( "San Diego", osg::Vec3d(    startLon,   startLat, 0.0 ), 0.0, -90, 45000 ));

    viewer.setCameraManipulator( manip );

    // add the state manipulator
    viewer.addEventHandler( new osgGA::StateSetManipulator(viewer.getCamera()->getOrCreateStateSet()) );

    // add the window size toggle handler
    viewer.addEventHandler(new osgViewer::WindowSizeHandler);

    // add the stats handler
    viewer.addEventHandler(new osgViewer::StatsHandler);

    return viewer.run();
}
