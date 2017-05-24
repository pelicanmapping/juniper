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
