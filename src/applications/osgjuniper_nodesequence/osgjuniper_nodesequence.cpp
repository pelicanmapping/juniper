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
