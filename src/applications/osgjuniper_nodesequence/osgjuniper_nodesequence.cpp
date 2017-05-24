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

#include <osgDB/ReadFile>
#include <osgDB/FileUtils>
#include <osgDB/FileNameUtils>

#include <osgGA/StateSetManipulator>

#include <osgViewer/Viewer>
#include <osgViewer/ViewerEventHandlers>

#include <iostream>

#include <osgJuniper/NodeSequence>

using namespace osgJuniper;

NodeSequence* createNodeSequence(osg::ArgumentParser& arguments)
{
    std::string directory;
    while (arguments.read("-d", directory) || arguments.read("--directory", directory));

    osg::notify(osg::NOTICE) << "Directory " << directory << std::endl;

    // assumes any remaining parameters are models
    NodeSequence* seq = new NodeSequence;

    typedef std::vector<std::string> Filenames;
    Filenames filenames;

    if (!directory.empty())
    {
        osgDB::DirectoryContents contents = osgDB::getDirectoryContents( directory );
        for (unsigned int i = 0; i < contents.size(); ++i)
        {
            if (contents[i] != "." && contents[i] != "..") filenames.push_back( osgDB::concatPaths(directory, contents[i] ) );
        }
    }
    else if (arguments.argc() > 1)
    {
        for (int i = 1; i < arguments.argc(); ++i)
        {
            filenames.push_back(arguments[i]);
        }
    }
    else
    {
        filenames.push_back("cow.osg");
        filenames.push_back("dumptruck.osg");
        filenames.push_back("cessna.osg");
        filenames.push_back("glider.osg");
    }

    for(Filenames::iterator itr = filenames.begin();
        itr != filenames.end();
        ++itr)
    {
        osg::notify(osg::NOTICE) << "Adding " << *itr << std::endl;
         seq->addFile( *itr );
    }

    return seq;
}

class NodeSequenceEventHandler : public osgGA::GUIEventHandler
{
public:
    NodeSequenceEventHandler(NodeSequence* seq)
    {
        _seq = seq;
    }

    // handle keydown events
    virtual bool handle(const osgGA::GUIEventAdapter& ea,
                        osgGA::GUIActionAdapter&)
    {
        if (ea.getEventType() == osgGA::GUIEventAdapter::KEYDOWN) {
            switch (ea.getKey()) {
            case 'n':
                {
                    _seq->seek( 1 );
                }
                break;
            case 'N':
                {
                    _seq->seek( 50 );
                }
                break;
            case 'p':
                {
                    _seq->seek( -1 );
                }
                break;
            case 'P':
                {
                    _seq->seek( -50 );
                }
            case 'b':
                {
                    //Decrease the buffer size
                    if (_seq->getBufferSize() > 0) _seq->setBufferSize( _seq->getBufferSize() -1 );
                }
                break;
            case 'B':
                {
                    if (_seq->getBufferSize() <= 200) _seq->setBufferSize(_seq->getBufferSize() + 1);
                }
            default:
                break;
            }
        }

        return false;
    }

private:
    osg::ref_ptr< NodeSequence > _seq;
};


int main( int argc, char **argv )
{
    // use an ArgumentParser object to manage the program arguments.
    osg::ArgumentParser arguments(&argc,argv);

    // construct the viewer.
    osgViewer::Viewer viewer(arguments);
    // root
    osg::Group* rootNode = new osg::Group;

    // add sequence of models from command line
    NodeSequence* seq = createNodeSequence( arguments );

	unsigned int buffer=5;
    while (arguments.read("-b", buffer) || arguments.read("--buffer", buffer));
	osg::notify(osg::NOTICE) << "BufferSize=" << buffer << std::endl;
	seq->setBufferSize( buffer );

    rootNode->addChild(seq);

    // add model to viewer.
    viewer.setSceneData(rootNode);

    // add the state manipulator
    viewer.addEventHandler( new osgGA::StateSetManipulator(viewer.getCamera()->getOrCreateStateSet()) );

    // add the window size toggle handler
    viewer.addEventHandler(new osgViewer::WindowSizeHandler);

    // add the stats handler
    viewer.addEventHandler(new osgViewer::StatsHandler);


    // add event handler to control sequence
    viewer.addEventHandler(new NodeSequenceEventHandler(seq));

    return viewer.run();
}
