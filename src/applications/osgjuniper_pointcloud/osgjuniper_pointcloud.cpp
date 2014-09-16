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

#include <iostream>

#include <osgDB/FileNameUtils>
#include <osgDB/FileUtils>

#include <osg/PagedLOD>

#include <osgJuniper/Octree>

#include <cctype>
#include <iomanip>
#include <string>

using namespace osgJuniper;

std::string getFilename(OctreeId id)
{
    std::stringstream buf;
    buf << "tile_" << id.level << "_" << id.z << "_" << id.x << "_" << id.y << ".laz";    
    return buf.str();
}

struct ToggleFullHandler : public osgGA::GUIEventHandler 
{
    ToggleFullHandler(osg::Node* tiled, osg::Node* full):
    _tiled(tiled),
    _full(full),
    _showFull(false)
    {        
        updateMask();
    }

    void updateMask()
    {
        _tiled->setNodeMask(_showFull ? 0 : ~0);
        _full->setNodeMask(_showFull ? ~0 : 0);
    }

    bool handle( const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa )
    {
        if ( ea.getEventType() == ea.KEYDOWN && ea.getKey() == 't' )
        {            
            _showFull = !_showFull;
            OSG_NOTICE << "Show full=" << _showFull << std::endl;
            updateMask();
        }
        return false;
    }

    bool _showFull;
    osg::ref_ptr< osg::Node > _tiled;
    osg::ref_ptr< osg::Node > _full;
};


class PointTileReader : public osgDB::ReaderWriter
{
public:
    PointTileReader()
    {
        supportsExtension( "point_tile", className() );
    }

    virtual const char* className()
    {
        return "Point tile reader";
    }    

    virtual ReadResult readNode( const std::string& location, const osgDB::ReaderWriter::Options* options ) const
    {
        if ( !acceptsExtension( osgDB::getLowerCaseFileExtension( location ) ) )
            return ReadResult::FILE_NOT_HANDLED;

        OSG_NOTICE << "Point tile reader " << location << std::endl;
        std::string file = osgDB::getNameLessExtension( location );
        osg::Node* node = osgDB::readNodeFile( file );
        if (!node) return ReadResult::FILE_NOT_FOUND;

        osg::PagedLOD* plod = new osg::PagedLOD;
        osg::Vec3d center = node->getBound().center();
        plod->setRadius(node->getBound().radius());
        plod->setCenter(center);
        plod->addChild(node);
        plod->setRange(0,0,FLT_MAX);

        std::string path = osgDB::getFilePath(file);

        // Get the octree tile name.        
        std::string tileID = osgDB::getNameLessExtension(osgDB::getSimpleFileName(file));
        unsigned int level, x, y, z;
        sscanf(tileID.c_str(), "tile_%d_%d_%d_%d", &level, &z, &x, &y);

        double radiusFactor = 5;

        OctreeId id(level, x, y, z);
        osg::ref_ptr< OctreeNode > octree = new OctreeNode();
        octree->setId(id);               
        unsigned int childNum = 1;
        
        double childRadius = node->getBound().radius() / 2.0;        
        for (unsigned int i = 0; i < 8; ++i)
        {
            osg::ref_ptr< OctreeNode > child = octree->createChild(i);
            std::string childFilename = osgDB::concatPaths(path, getFilename(child->getID()));            

            if (osgDB::fileExists(childFilename))
            {
                childFilename += ".point_tile";            
                plod->setFileName(childNum, childFilename);
                plod->setRange(childNum, 0, childRadius * radiusFactor);
                childNum++;
            }
            else
            {
                OSG_NOTICE << "Child " << childFilename << " doesn't exist" << std::endl;
            }
        }        

        return plod;
    }

};

REGISTER_OSGPLUGIN(point_tile, PointTileReader)



 bool startsWith( const std::string& ref, const std::string& pattern)
{
    if ( pattern.length() > ref.length() )
        return false;

    for( unsigned i=0; i<pattern.length(); ++i )
    {
        if ( ref[i] != pattern[i] )
            return false;
    }
    return true;
}



int main(int argc, char** argv)
{
    osgDB::Registry::instance()->addFileExtensionAlias("laz", "las");

    // use an ArgumentParser object to manage the program arguments.
    osg::ArgumentParser arguments(&argc,argv);

    osgViewer::Viewer viewer(arguments);
    // add the state manipulator
    viewer.addEventHandler( new osgGA::StateSetManipulator(viewer.getCamera()->getOrCreateStateSet()) );


    // add the window size toggle handler
    viewer.addEventHandler(new osgViewer::WindowSizeHandler);

    // add the stats handler
    viewer.addEventHandler(new osgViewer::StatsHandler);

    // add the LOD Scale handler
    viewer.addEventHandler(new osgViewer::LODScaleHandler);

    // add the screen capture handler
    viewer.addEventHandler(new osgViewer::ScreenCaptureHandler);

    // load the data
    osg::Group* root = new osg::Group;

    std::vector< std::string > filenames;
    //Read in the filenames to process
    for(int pos=1;pos<arguments.argc();++pos)
    {
        if (!arguments.isOption(pos))
        {
            filenames.push_back( arguments[pos]);
        }
    }

    
   
    root->getOrCreateStateSet()->setAssociatedModes(new osg::Point(1.2), osg::StateAttribute::ON);
    root->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF);

    for (unsigned int i = 0; i < filenames.size(); i++)
    {
        std::stringstream buf;
        buf << filenames[i] << ".point_tile";
        root->addChild(osgDB::readNodeFile(buf.str()));
    }

    //osg::Node* tiled = osgDB::readNodeFile("C:/geodata/aam/PointClouds/Yarra/tile_0_0_0_0.laz.point_tile");
    //osg::Node* tiled = osgDB::readNodeFile("C:/geodata/aam/PointClouds/HongKong2/HongKong_ALS/tile_0_0_0_0.laz.point_tile");
    /*
    root->addChild(osgDB::readNodeFile("C:/geodata/aam/PointClouds/Yarra/tile_1_0_0_0.laz.point_tile"));
    root->addChild(osgDB::readNodeFile("C:/geodata/aam/PointClouds/Yarra/tile_1_0_0_1.laz.point_tile"));
    root->addChild(osgDB::readNodeFile("C:/geodata/aam/PointClouds/Yarra/tile_1_0_1_0.laz.point_tile"));
    root->addChild(osgDB::readNodeFile("C:/geodata/aam/PointClouds/Yarra/tile_1_0_1_1.laz.point_tile"));
    root->addChild(osgDB::readNodeFile("C:/geodata/aam/PointClouds/Yarra/tile_1_1_0_0.laz.point_tile"));
    root->addChild(osgDB::readNodeFile("C:/geodata/aam/PointClouds/Yarra/tile_1_1_0_1.laz.point_tile"));
    root->addChild(osgDB::readNodeFile("C:/geodata/aam/PointClouds/Yarra/tile_1_1_1_0.laz.point_tile"));
    root->addChild(osgDB::readNodeFile("C:/geodata/aam/PointClouds/Yarra/tile_1_1_1_1.laz.point_tile"));   
    */
    
    //osg::Node* full = osgDB::readNodeFile("pt000001.laz");        
    //root->addChild(full);

    //viewer.addEventHandler(new ToggleFullHandler(tiled, full));
    /*
    osgDB::DirectoryContents contents = osgDB::getDirectoryContents("C:/geodata/aam/PointClouds/Yarra/");
    for (unsigned int i = 0; i < contents.size(); i++)
    {

        if (startsWith(contents[i], "tile"))
        {
            root->addChild(osgDB::readNodeFile(contents[i]));
        }
    }
    */

    // any option left unread are converted into errors to write out later.
    arguments.reportRemainingOptionsAsUnrecognized();

    // report any errors if they have occurred when parsing the program arguments.
    if (arguments.errors())
    {
        arguments.writeErrorMessages(std::cout);
        return 1;
    }
    
    viewer.setSceneData( root );   

    viewer.getCamera()->setNearFarRatio(0.00002);

    return viewer.run();

}
