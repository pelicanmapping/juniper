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

#include <osgJuniper/Utils>

#include <iostream>

using namespace osgJuniper;



static const char *vertSource = {        
    "void main(void)\n"
    "{\n"    
    "    gl_Position = ftransform();\n"
    "    gl_FrontColor = gl_Color;\n"
    "}\n"
};

static const char *fragSource = {    
    "uniform float fadeTime;\n"
    "uniform float fadeStartTime;\n"
    "uniform float osg_FrameTime;\n"
    "void main(void)\n"
    "{\n"
    "    float a = clamp((osg_FrameTime - fadeStartTime)/fadeTime, 0, 1);\n"
    "    gl_FragColor = vec4(gl_Color.rgb, a);\n"
    "}\n"
};

class FadeNode : public osg::Group
{
public:
    FadeNode():
      _lastCulledFrame(-1),
      _fadeTime(0.5),
      _startTime(0.0f)
    {
        getOrCreateStateSet()->getOrCreateUniform("fadeTime",      osg::Uniform::FLOAT)->set(_fadeTime);
        getOrCreateStateSet()->getOrCreateUniform("fadeStartTime", osg::Uniform::FLOAT)->set(_startTime);
        getOrCreateStateSet()->setMode(GL_BLEND, osg::StateAttribute::ON);

        osg::Program* program = new osg::Program();
        program->addShader(new osg::Shader(osg::Shader::VERTEX,   vertSource));
        program->addShader(new osg::Shader(osg::Shader::FRAGMENT, fragSource));
        getOrCreateStateSet()->setAttributeAndModes(program, osg::StateAttribute::ON);
    }

    void setFadeTime( double fadeTime ) 
    {
        if (_fadeTime != fadeTime) 
        {
            _fadeTime = fadeTime;    
            getOrCreateStateSet()->getOrCreateUniform("fadeTime",      osg::Uniform::FLOAT)->set(_fadeTime);        
        }
    }

    double getFadeTime() const
    {
        return _fadeTime;
    }

    virtual void traverse(osg::NodeVisitor& nv)
    {
        if (nv.getVisitorType() == osg::NodeVisitor::CULL_VISITOR)
        {
            //We are being culled
            if (_lastCulledFrame < 0 || nv.getFrameStamp()->getFrameNumber() - _lastCulledFrame > 1) {                
                _startTime = nv.getFrameStamp()->getReferenceTime();
                getOrCreateStateSet()->getOrCreateUniform("fadeStartTime", osg::Uniform::FLOAT)->set(_startTime);
            }
            _lastCulledFrame = nv.getFrameStamp()->getFrameNumber();
        }

        osg::Group::traverse(nv);
    }
protected:    
    float _fadeTime;
    int _lastCulledFrame;
    float _startTime;
};

struct FindGeode : public osg::NodeVisitor
{
    std::vector< osg::ref_ptr< osg::Geode > > _geodes;

    FindGeode() : osg::NodeVisitor(osg::NodeVisitor::TRAVERSE_ALL_CHILDREN) {}
    void apply(osg::Geode& geode) {
        _geodes.push_back( &geode );        
    }
};

class MyReadFileCallback : public osgDB::Registry::ReadFileCallback
{
public:
    virtual osgDB::ReaderWriter::ReadResult readNode(const std::string& fileName, const osgDB::ReaderWriter::Options* options)
    {
        // note when calling the Registry to do the read you have to call readNodeImplementation NOT readNode, as this will
        // cause on infinite recusive loop.
        osgDB::ReaderWriter::ReadResult result = osgDB::Registry::instance()->readNodeImplementation(fileName,options);
        //Wrap any Geode with a FadeNode
        FindGeode geodeFinder;
        result.getNode()->accept( geodeFinder );
        for (unsigned int i = 0; i < geodeFinder._geodes.size(); i++)
        {            
            osg::Geode* geode = geodeFinder._geodes[i].get();
            FadeNode* fade = new FadeNode();
            fade->addChild( geode  );
            osg::Group* parent = geode->getParent(0);
            parent->replaceChild( geode, fade );            

            for (unsigned int j = 0; j < geode->getNumDrawables(); j++ )
            {
                geode->getDrawable(j)->asGeometry()->setUseVertexBufferObjects(true);
                geode->getDrawable(j)->asGeometry()->setUseDisplayList(false);
            }
        }
        return result;
    }
};

osg::Node* buildPointNode(unsigned int numPoints)
{    
    osg::Vec3Array* verts = new osg::Vec3Array( numPoints );
    osg::Vec4Array* colors = new osg::Vec4Array( numPoints );

    osg::Geometry* geometry = new osg::Geometry;
    geometry->setVertexArray( verts );
    geometry->setColorArray( colors );
    geometry->setColorBinding( osg::Geometry::BIND_PER_VERTEX );
    geometry->setUseVertexBufferObjects( true );

    for (unsigned int i = 0; i < numPoints; i++) 
    {
        (*verts)[i]  = Utils::randomVert();
        (*colors)[i] = Utils::randomColor();
    }

    geometry->addPrimitiveSet(new osg::DrawArrays(GL_POINTS, 0, verts->size()));
    
    osg::Geode* geode = new osg::Geode;
    geode->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
    geode->addDrawable( geometry );

    FadeNode* fade = new FadeNode;
    fade->addChild( geode );
    osg::LOD* lod = new osg::LOD;
    lod->addChild(fade, 0, 5);
    return lod;       
}

int main(int argc, char** argv)
{
    // use an ArgumentParser object to manage the program arguments.
    osg::ArgumentParser arguments(&argc,argv);

    osgDB::Registry::instance()->setReadFileCallback(new MyReadFileCallback());

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

    // set up the camera manipulators.
    {
        osg::ref_ptr<osgGA::KeySwitchMatrixManipulator> keyswitchManipulator = new osgGA::KeySwitchMatrixManipulator;

        keyswitchManipulator->addMatrixManipulator( '1', "Trackball", new osgGA::TrackballManipulator() );
        keyswitchManipulator->addMatrixManipulator( '2', "Flight", new osgGA::FlightManipulator() );
        keyswitchManipulator->addMatrixManipulator( '3', "Drive", new osgGA::DriveManipulator() );
        keyswitchManipulator->addMatrixManipulator( '4', "Terrain", new osgGA::TerrainManipulator() );
        keyswitchManipulator->addMatrixManipulator( '5', "Orbit", new osgGA::OrbitManipulator() );
        keyswitchManipulator->addMatrixManipulator( '6', "FirstPerson", new osgGA::FirstPersonManipulator() );
        keyswitchManipulator->addMatrixManipulator( '7', "Spherical", new osgGA::SphericalManipulator() );

        std::string pathfile;
        double animationSpeed = 1.0;
        while(arguments.read("--speed",animationSpeed) ) {}
        char keyForAnimationPath = '8';
        while (arguments.read("-p",pathfile))
        {
            osgGA::AnimationPathManipulator* apm = new osgGA::AnimationPathManipulator(pathfile);
            if (apm || !apm->valid())
            {
                apm->setTimeScale(animationSpeed);

                unsigned int num = keyswitchManipulator->getNumMatrixManipulators();
                keyswitchManipulator->addMatrixManipulator( keyForAnimationPath, "Path", apm );
                keyswitchManipulator->selectMatrixManipulator(num);
                ++keyForAnimationPath;
            }
        }

        viewer.setCameraManipulator( keyswitchManipulator.get() );
    }

    // add the state manipulator
    viewer.addEventHandler( new osgGA::StateSetManipulator(viewer.getCamera()->getOrCreateStateSet()) );

    // add the thread model handler
    viewer.addEventHandler(new osgViewer::ThreadingHandler);

    // add the window size toggle handler
    viewer.addEventHandler(new osgViewer::WindowSizeHandler);

    // add the stats handler
    viewer.addEventHandler(new osgViewer::StatsHandler);

    // add the help handler
    viewer.addEventHandler(new osgViewer::HelpHandler(arguments.getApplicationUsage()));

    // add the record camera path handler
    viewer.addEventHandler(new osgViewer::RecordCameraPathHandler);

    // add the LOD Scale handler
    viewer.addEventHandler(new osgViewer::LODScaleHandler);

    // add the screen capture handler
    viewer.addEventHandler(new osgViewer::ScreenCaptureHandler);

    // load the data
    osg::Group* root = new osg::Group;
    //root->addChild( buildPointNode( 100000 ) );
    root->addChild( osgDB::readNodeFiles(arguments) );
    root->getOrCreateStateSet()->setAttributeAndModes( new osg::Point() );
    
    // any option left unread are converted into errors to write out later.
    arguments.reportRemainingOptionsAsUnrecognized();

    // report any errors if they have occurred when parsing the program arguments.
    if (arguments.errors())
    {
        arguments.writeErrorMessages(std::cout);
        return 1;
    }
    
    viewer.setSceneData( root );

    viewer.realize();

    return viewer.run();

}
