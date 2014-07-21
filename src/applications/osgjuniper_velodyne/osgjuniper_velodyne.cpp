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
#include <osgJuniper/Velodyne>
#include <osgJuniper/StreamingNode>

#include <winsock2.h>
//#include <netinet/in.h>

#include <iostream>

using namespace osgJuniper;




int main( int argc, char **argv )
{   
    // use an ArgumentParser object to manage the program arguments.
    osg::ArgumentParser arguments(&argc,argv);

    unsigned int history = 0;
    while (arguments.read("--history", history)){};    
    std::string filename = "C:\\dev\\juniper\\data\\VelodynePCAPData\\DriveHomeSplit1.pcap";
    //Read in the filenames to process
    for(int pos=1;pos<arguments.argc();++pos)
    {
        if (!arguments.isOption(pos))
        {
            filename = arguments[pos];
            break;
        }
    }

    if (!osgDB::fileExists( filename))
    {
        osg::notify(osg::NOTICE) << filename << " does not exists, please provide a filename" << std::endl;
        return 0;
    }
   
    // construct the viewer.
    osgViewer::Viewer viewer(arguments);

    osg::ref_ptr < osg::Group > root = new osg::Group;

    osg::ref_ptr<VelodyneDataset> ds = new VelodyneDataset(filename);
    /*
    //osg::ref_ptr<VelodyneDataset> ds = new VelodyneDataset("C:\\dev\\juniper\\data\\VelodynePCAPData\\boatsSplit.pcap");
    //osg::ref_ptr<VelodyneDataset> ds = new VelodyneDataset("C:\\dev\\juniper\\data\\VelodynePCAPData\\Sample Data Capture and Movies\\Street Capture Conventional Mounting - view with DSR.pcap");

    osg::ref_ptr< VelodyneNode > velodyneNode = new VelodyneNode( ds.get() );
    root->addChild( velodyneNode );
    
    viewer.addEventHandler(new MyEventHandler(velodyneNode.get()));
    */

    /*
    SimpleStreamingNodeSource* source = new SimpleStreamingNodeSource();
    source->addFilename( "cow.osg" );
    source->addFilename( "cessna.osg" );
    source->addFilename( "cessnafire.osg" );
    */

    VelodyneStreamingNodeSource* source = new VelodyneStreamingNodeSource( ds.get() );

    /*
    RandomStreamingNodeSource* source = new RandomStreamingNodeSource();
    source->setNumPoints( 500000 );
    */

    source->startStreaming();
    StreamingNode* streamingNode = new StreamingNode( source );
    streamingNode->setHistory( history );
    root->addChild( streamingNode );
    
    // add model to viewer.
    viewer.setSceneData(root);

    // add the state manipulator
    viewer.addEventHandler( new osgGA::StateSetManipulator(viewer.getCamera()->getOrCreateStateSet()) );

    // add the window size toggle handler
    viewer.addEventHandler(new osgViewer::WindowSizeHandler);

    // add the stats handler
    viewer.addEventHandler(new osgViewer::StatsHandler);

    return viewer.run();
}
