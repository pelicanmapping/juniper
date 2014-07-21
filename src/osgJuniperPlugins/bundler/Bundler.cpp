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
#include <osg/Geometry>
#include <osg/Geode>
#include <osg/Notify>
#include <osg/MatrixTransform>
#include <osg/Texture2D>
#include <osgDB/Registry>
#include <osgDB/FileUtils>
#include <osgDB/FileNameUtils>
#include <osgDB/ReaderWriter>
#include <osgDB/ReadFile>
#include <osgUtil/SmoothingVisitor>
 
#include <osgJuniper/IBR>
#include <osgJuniper/IBRCameraNode>
#include <fstream>
#include <vector>
#include <sstream>
#include <sys/types.h>
#include <climits>

#include <errno.h>
#include <iostream>

#include <osgEarth/StringUtils>



#ifndef WIN32
#include <string.h>
#include <stdio.h>
#endif

using namespace osgJuniper;

#define LOG osg::notify(osg::NOTICE)
#define EOL std::endl;
#define FLUSH std::flush;

#define OVEC(v) "("<<v[0]<<","<<v[1]<<","<<v[2]<<")"

/**
 * Bundler Output Visualization
 * http://phototour.cs.washington.edu/bundler/bundler-v0.2-manual.html#S6
 */

/********************************************************************/
//utilities


/********************************************************************
 * Bundler-specific model loader
 *
 * See:
 * http://phototour.cs.washington.edu/bundler/bundler-v0.2-manual.html#S6
 */

//Options for the bundler file.
struct BundlerOptions
{
    BundlerOptions():
_loadPoints(true),
_loadCameras(true),
_loadDepthModels(false),
_depthPrefix("depth"),
_maxCameras(UINT_MAX),
_scale(1.0f),
_numThreads(-1) //-1 means use default
{
}    
    std::string _pointCloud;
    bool _loadPoints;
    bool _loadCameras;
    bool _loadDepthModels;
    std::string _depthPrefix;
    unsigned int _maxCameras;
    float _scale;
    unsigned int _numThreads;
};

class BundlerModel : public IBRModel
{
public:
    BundlerModel() { }

    bool loadOptions( const std::string& location, BundlerOptions& opt)
    {
        std::string optionsFilename = location + ".opt";
        if (osgDB::fileExists( optionsFilename) )
        {
            osg::notify(osg::NOTICE) << "Loading options " << optionsFilename << std::endl;
            std::ifstream in(optionsFilename.c_str());
            std::string line;
            while ( std::getline(in, line) )
            {
                if ( line[0]=='#' ) continue;  // Comment
                osgEarth::StringVector keyAndValue;
                osgEarth::StringTokenizer izer( "=" );                
                izer.tokenize( line, keyAndValue );

                if ( keyAndValue.size()<2 ) continue;

                std::string key=keyAndValue[0];
                std::string value=keyAndValue[1];

    
                if (key == "point_cloud")
                    opt._pointCloud = value;
                else if (key == "depth_prefix")
                    opt._depthPrefix = value;
                else if (key == "load_points")
                    opt._loadPoints = value == "true";
                else if (key == "load_cameras")
                    opt._loadCameras = value == "true";
                else if (key == "load_depth_models")
                    opt._loadDepthModels = value == "true";
                else if (key == "max_cameras")
                {
                    std::stringstream ss(value);
                    ss >> opt._maxCameras;
                }
                else if (key == "scale")
                {
                    std::stringstream ss(value);
                    ss >> opt._scale;
                }
                else if (key == "num_threads")
                {
                    std::stringstream ss(value);
                    ss >> opt._numThreads;
                }
            }
            return true;
        }
        return false;
    }

    bool load( const std::string& location )
    {
        //Try to load options from the options file.
        if (loadOptions(location, _options))
        {
            osg::notify(osg::NOTICE) << "Loaded options" << std::endl;
        }

        // first read the images list
        std::string prefix = osgDB::getFilePath(location);
        std::string imageListFile = osgDB::concatPaths( prefix, "list.txt" );
        std::string visualizeDirectory = osgDB::concatPaths(prefix, "visualize");
        
        //Setup the depth directory
        std::string depthDirectory;
        if (!_options._depthPrefix.empty())
        {
            depthDirectory = osgDB::concatPaths(prefix, _options._depthPrefix);
        }

        LOG << "Image list file = " << imageListFile << EOL;

        if (osgDB::fileExists(imageListFile))
        {
            std::ifstream listIn(imageListFile.c_str());
            if ( !listIn.is_open() )
                return errorOut( "Cannot open image list file" );
            if ( !loadImageList(listIn, prefix) ) return errorOut( "Error reading image list file" );
        }
        //If no list.txt exists, try to load the visualize directory
        else if (osgDB::fileExists(visualizeDirectory))
        {
            if (!loadImageDirectory( visualizeDirectory)) return errorOut("Error reading visualize directory");
        }
        else
        {
            return errorOut( "Couldn't open image file or visualize directory");
        }

        if (_options._loadDepthModels && !depthDirectory.empty() && osgDB::fileExists(depthDirectory))
        {
            if (!loadDepthModelDirectory( depthDirectory))
                LOG << "no depth directory found, skip depth model representation" << std::endl;
        }

        // then read the cameras list
        std::ifstream camIn(location.c_str());
        if ( !camIn.is_open() )
            return errorOut( "Cannot open cameras file" );

        if ( !loadCameras(camIn) )
            return errorOut( "Error reading cameras file" );

        camIn.close();
        return true;
    }

    bool loadImageList( std::istream& in, const std::string& prefix )
    {
        char buf[2048];
        while(!in.eof())
        {
            in.getline(buf,2048);
            imageFileNames.push_back( osgDB::concatPaths(
                                          prefix, std::string(buf) ) );
        }
        LOG << "Images = " << imageFileNames.size() << EOL;
        return true;
    }

    static bool endsWith(const std::string& fullString, const std::string& ending)
    {
        if (fullString.length() > ending.length()) {
            return (0 == fullString.compare (fullString.length() - ending.length(), ending.length(), ending));
        } else {
            return false;
        }
    }

    bool loadImageDirectory( const std::string &path)
    {
        std::vector< std::string > files = osgDB::getDirectoryContents(path);
        for (unsigned int i = 0; i < files.size(); ++i)
        {
            if (files[i] != "." && files[i] != "..")
            {	
                //See if the base name ends with "_thumb" and if it doesn't, don't append it.  These could be thumbnails used to speed up loading.  Also don't load _compressed images b/c they are optimized DXT compressed files used to increase performance.
                std::string basename = osgDB::getNameLessExtension( files[i] );
                if (!endsWith(basename, "_thumb") && !endsWith(basename, "_compressed"))
                {
                    imageFileNames.push_back( osgDB::concatPaths( path, files[i]) );
                }
            }
        }
        return true;
    }

    bool loadDepthModelDirectory( const std::string &path)
    {
        std::vector< std::string > files = osgDB::getDirectoryContents(path);
        for (unsigned int i = 0; i < files.size(); ++i)
        {
            if (files[i] != "." && 
                files[i] != ".." && 
                osgDB::getFileExtension(files[i]) != "png" )
            {	
                std::string key = osgDB::getStrippedName(files[i]);
                depthModelFileNames[key] = osgDB::concatPaths( path, files[i]);
                osg::notify(osg::NOTICE) << "Loaded depth model " << key << " = " << depthModelFileNames[key] << std::endl;
            }
        }
        return true;
    }

    bool loadCameras( std::istream& in )
    {
        // read the first line
        char marker[1024];
        in.getline( marker, 1024 );
        if ( std::string(marker) != "# Bundle file v0.3" )
            return errorOut( "Not a valid bundler output file" );

        unsigned int numCameras, numPoints;
        in >> numCameras >> numPoints;

        // read the camera configs:
        LOG << "Cameras = " << numCameras << EOL;
        _cameras.reserve( numCameras );
        unsigned int imageIndex = 0;
        for( int c=0; c<numCameras; ++c )
        {
            IBRCamera cam;
            cam._id = c;
            in >> cam._focalLength >> cam._k1 >> cam._k2;

            double r[9];
            for(int i=0; i<9; ++i) 
                in >> r[i];
            cam._rot.set(
                r[0], r[1], r[2], 0.0,
                r[3], r[4], r[5], 0.0,
                r[6], r[7], r[8], 0.0,
                0.0,  0.0,  0.0,  1.0 );

            for(int i=0; i<3; ++i) 
                in >> cam._trans[i];

            //Only add the camera if we actually have an image for it and it has a proper matrix
            if ( c < imageFileNames.size() && !cam.getWorldMatrix().isNaN() )
            {
                cam._imageUri = imageFileNames[imageIndex];
                // check if we have a depth model for this image
                std::string key = osgDB::getStrippedName(cam._imageUri);
                if (_options._loadDepthModels)
                {
                    if (depthModelFileNames.find(key) != depthModelFileNames.end()) {
                        cam._depthModelUri = depthModelFileNames[key];
                    }
                    else
                    {
                        //Try looking for a filename with the orignal naming scheme which is depth-XXXXXXX-ascii
                        key = "depth-" + key + "-ascii";
                        if (depthModelFileNames.find(key) != depthModelFileNames.end()) {
                            cam._depthModelUri = depthModelFileNames[key];
                        }
                    }
                }

                _cameras.push_back( cam );
                imageIndex++;
            }
        }

        // read in the points:
        LOG << "Points = " << numPoints << EOL;
        _points.reserve( numPoints );
        for( int p=0; p<numPoints; ++p )
        {
            IBRPoint point;
            in >> point._position.x() >> point._position.y() >> point._position.z();
            int r, g, b;
            in >> r >> g >> b;
            point._color.r() = r;
            point._color.g() = g;
            point._color.b() = b;
            point._color.a() = 255;

            unsigned int numViews;
            in >> numViews;

            point._views.reserve( numViews );
            for( int v=0; v<numViews; ++v )
            {
                IBRPointView view;
                in >> view._cameraIndex >> view._key >> view._x >> view._y;
                point._views.push_back( view );
            }

            _points.push_back(point);           
        }

        return true;
    }

    const std::string& errorMsg() { 
        return _errorMsg;
    }

    bool errorOut( const std::string& msg ) {
        _errorMsg = msg;
        return false;
    }

    osg::Node* createNode()
    {
        IBRScene* root = new IBRScene();

        if (_options._numThreads > 0)
        {
            LOG << "Setting num threads to " << _options._numThreads << std::endl;
            root->getTaskService()->setNumThreads( _options._numThreads);
        }

        if (_options._loadCameras)
        {
            LOG << "Creating camera Nodes..." << std::endl;
            // limit the number of cameras to render, just to save time while testing
            int camerasToDraw = osg::minimum(_cameras.size(), _options._maxCameras);
            for( int i=0; i<camerasToDraw; ++i )
            {
                LOG << "Creating camera " << i << std::endl;
                osg::Node* camNode = createCameraNode( _cameras[i] );
                if ( camNode )
                    root->addChild( camNode );
            }
        }

        if (_options._loadPoints)
        {
            if (!_options._pointCloud.empty())
            {
                osg::notify(osg::NOTICE) << "Loading point cloud from " << _options._pointCloud << std::endl;
                osg::Node* pc = osgDB::readNodeFile( _options._pointCloud );
                if (pc)
                {
                    root->addChild( pc );
                }
            }
            else
            {
                LOG << "Creating point Node..." << std::endl;
                root->addChild( createPointListNode( _points ) );
            }
        }

        return root;
    }

private:

    /**
     * Builds a colored point cloud.
     */
    osg::Geometry* createPointListGeom( const IBRPointList& points )
    {
        osg::Geometry* geom = new osg::Geometry();

        osg::Vec3Array* v = new osg::Vec3Array();
        osg::Vec4ubArray* c = new osg::Vec4ubArray();
        v->reserve(points.size());
        c->reserve(points.size());
        for( IBRPointList::const_iterator i = points.begin(); i != points.end(); ++i )
        {
            v->push_back( osg::Vec3d( i->_position ) );
            c->push_back( i->_color );
        }
        geom->setVertexArray(v);
        geom->setColorArray(c);
        geom->setColorBinding( osg::Geometry::BIND_PER_VERTEX );

        geom->addPrimitiveSet( new osg::DrawArrays(GL_POINTS, 0, points.size()) );

        return geom;
    }


    osg::Node* createPointListNode( const IBRPointList& points )
    {
        osg::Geode* geode = new osg::Geode();
        geode->addDrawable( createPointListGeom(points) );
        return geode;
    }   

    /**
    * Creates a node representing a single camera. The camera's ID # is
    * encoded in the node description.
    */
    osg::Node* createCameraNode( const IBRCamera& camera )
    {
        IBRCameraNode* node = new IBRCameraNode();
        node->setCamera( camera );
        node->setScale( _options._scale );
        return node;        
    }

    BundlerOptions _options;
    std::string _errorMsg;
    std::vector<std::string> imageFileNames;
    std::map<std::string,std::string> depthModelFileNames;
};

/********************************************************************/

/**
 * Reads Bundler IBR sets.
 */
class BundlerReader : public osgDB::ReaderWriter
{
public:
    BundlerReader()
    {
        supportsExtension( "juniper_bundler", className() );
    }

    //override
    const char* className()
    {
        return "Bundler Renderer for Juniper";
    }

    /**
     * Makes some unit world coordinate system axes for a sense of scale
     */
    osg::Node* createWorldAxes() const
    {
        osg::Geometry* geom = new osg::Geometry();
        
        osg::Vec3Array* v = new osg::Vec3Array(4);
        (*v)[0].set( 0, 0, 0 );
        (*v)[1].set( 1, 0, 0 );
        (*v)[2].set( 0, 1, 0 );
        (*v)[3].set( 0, 0, 1 );
        geom->setVertexArray(v);

        GLushort i[6] = { 0, 1, 0, 2, 0, 3 };
        geom->addPrimitiveSet( new osg::DrawElementsUShort(GL_LINES, 6, i) );

        osg::Vec4Array* c = new osg::Vec4Array(3);
        (*c)[0].set( 1, 0, 0, 1 );
        (*c)[1].set( 0, 1, 0, 1 );
        (*c)[2].set( 0, 0, 1, 1 );
        geom->setColorArray( c );
        geom->setColorBinding( osg::Geometry::BIND_PER_PRIMITIVE );

        geom->getOrCreateStateSet()->setMode( GL_LIGHTING, 0 );

        osg::Geode* geode = new osg::Geode();
        geode->addDrawable( geom );
        return geode;
    }


    //override
    ReadResult readNode( const std::string& location, const osgDB::ReaderWriter::Options* options ) const
    {
        if ( !acceptsExtension( osgDB::getLowerCaseFileExtension( location ) ) )
            return ReadResult::FILE_NOT_HANDLED;

        // strip the pseudo-loader extension
        std::string subLocation = osgDB::getNameLessExtension( location );
        if ( subLocation.empty() )
            return ReadResult::FILE_NOT_HANDLED;

        osg::Group* group = new osg::Group();
        group->getOrCreateStateSet()->setMode( GL_LIGHTING, 0 );

        group->addChild( createWorldAxes() );

        BundlerModel model;
            
        if ( model.load( subLocation ) )
        {
            osg::Node* node = model.createNode();
            if (node)
                group->addChild( node );
        }
        else
        {
            LOG << "ERROR: " << model.errorMsg() << EOL;
        }

        //We've loaded the whole group, now convert y-up to z-up so things are consistent with OSG
        osg::MatrixTransform* toZUp = new osg::MatrixTransform();
        osg::Quat quat;
        quat.makeRotate(osg::Vec3d(0,1,0), osg::Vec3d(0,0,1));
        toZUp->setMatrix(osg::Matrixd(quat));
        toZUp->addChild( group );
        return ReadResult(toZUp);
    }
};

REGISTER_OSGPLUGIN(juniper_bundler, BundlerReader)


