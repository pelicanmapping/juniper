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

#include <osgJuniper/StreamingNode>
#include <osgJuniper/Utils>

#include <osgDB/ReadFile>

using namespace osgJuniper;


StreamingNodeSource::StreamingNodeSource():
_isStarted(false)
{
}

void
StreamingNodeSource::startStreaming()
{
    if (!_isStarted)
    {
        startImplementation();
        _isStarted = true;
    }
}

void
StreamingNodeSource::stopStreaming()
{
    if (_isStarted)
    {
        stopImplementation();
        _isStarted = false;
    }
}

void
StreamingNodeSource::frame(osg::Node* node)
{
    for (StreamingNodeCallbackList::iterator itr = _callbacks.begin(); itr != _callbacks.end(); ++itr)
    {
        itr->get()->onFrame( node );
    }
}

/*******************************************************************************/
SimpleStreamingNodeSource::SimpleStreamingNodeSource():
_filenameIndex(0),
_done(false)
{
}

SimpleStreamingNodeSource::~SimpleStreamingNodeSource()
{
    cancel();
}

void
SimpleStreamingNodeSource::addFilename(const std::string& filename)
{
    _filenames.push_back( filename );
}

void
SimpleStreamingNodeSource::startImplementation()
{
    _done = false;
    _filenameIndex = 0;
    startThread();
}

void
SimpleStreamingNodeSource::stopImplementation()
{
    cancel();
}

void
SimpleStreamingNodeSource::run()
{
    do
    {
        if (_filenames.size() > 0)
        {            
            if (_filenameIndex < _filenames.size())
            {
                osg::ref_ptr< osg::Node > node = osgDB::readNodeFile( _filenames[_filenameIndex] );
                if (node.valid())
                {
                    frame( node.get() );
                }
            }
            _filenameIndex++;
            if (_filenameIndex >= _filenames.size())
            {
                _filenameIndex = 0;
            }

            //Sleep for little bit
            OpenThreads::Thread::microSleep( 1 * 1000 * 1000 );
        }
    } while (!_done && !testCancel());
}

int
SimpleStreamingNodeSource::cancel()
{
    _done = true;
    while (isRunning())
    {
        OpenThreads::Thread::YieldCurrentThread();
    }
    return 0;
}


/*******************************************************************************/
RandomStreamingNodeSource::RandomStreamingNodeSource():
_numPoints( 50000 ),
_done(false)
{
}

RandomStreamingNodeSource::~RandomStreamingNodeSource()
{
    cancel();
}

void
RandomStreamingNodeSource::startImplementation()
{
    _done = false;
    startThread();
}

void
RandomStreamingNodeSource::stopImplementation()
{
    cancel();
}

void
RandomStreamingNodeSource::run()
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
        for (unsigned int i = 0; i < _numPoints; ++i)
        {
            if (!verts.valid() || !colors.valid())
            {
                verts = new osg::Vec3Array( batchSize );
                colors = new osg::Vec4Array( batchSize );
            }
            (*verts)[j]  = Utils::randomVert();
            (*colors)[j] = Utils::randomColor();
            j++;

            if (j == batchSize)
            {
                root->addChild( makePoints( verts, colors, j) );
                j = 0;
                verts = 0;
                colors = 0;
            }
        }

        if (verts.valid())
        {
            root->addChild( makePoints( verts, colors, j) );
        }

        frame( root );


        //Sleep for little bit
        OpenThreads::Thread::microSleep( 0.05 * 1000 * 1000 );        
    } while (!_done && !testCancel());
}

int
RandomStreamingNodeSource::cancel()
{
    _done = true;
    while (isRunning())
    {               
        OpenThreads::Thread::YieldCurrentThread();
    }
    return 0;
}

osg::Node*
RandomStreamingNodeSource::makePoints(osg::Vec3Array* points, osg::Vec4Array* colors, unsigned int numPoints)
{
    osg::Geometry *geometry = new osg::Geometry;
    geometry->setVertexArray( points );
    geometry->setUseDisplayList( false );
    geometry->setUseVertexBufferObjects( true );

    //osg::Vec4Array* colors = new osg::Vec4Array;
    //colors->push_back( osg::Vec4(1,0,0,1));
    geometry->setColorBinding( osg::Geometry::BIND_PER_VERTEX );
    geometry->setColorArray( colors );

    //geometry->addPrimitiveSet( new osg::DrawArrays(GL_POINTS, 0, points->size() ) );
    geometry->addPrimitiveSet( new osg::DrawArrays(GL_POINTS, 0, numPoints ) );
    //osg::notify(osg::NOTICE) << "Added a frame with " << points->size() << " verts" << std::endl;

    osg::Geode *geode = new osg::Geode;
    geode->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
    geode->addDrawable( geometry );
    return geode;    
}

/*******************************************************************************/
void
StreamingNode::Callback::onFrame(osg::Node* node)
{
    /*
    if (_pendingNode.valid())
    {
        osg::notify(osg::NOTICE) << "Couldn't merge fast enough, missing frame" << std::endl;
    }
    */
    //This could be called from another thread so take a reference and do nothing with it, it will be merged during the update traversal
    _pendingNode = node;
}


StreamingNode::StreamingNode( StreamingNodeSource* source ):
_source(source),
_history(0)
{
    setNumChildrenRequiringUpdateTraversal( 1 );

    _callback = new Callback();

    source->getCallbacks().push_back( _callback ); 
}

unsigned int
StreamingNode::getHistory() const
{
    return _history;
}

void
StreamingNode::setHistory(unsigned int history)
{
    _history = history;
}


void
StreamingNode::traverse(osg::NodeVisitor& nv)
{
    if (nv.getVisitorType() == osg::NodeVisitor::UPDATE_VISITOR)
    {
        //Take a reference to the pending node
        osg::ref_ptr< osg::Node > pending = _callback->_pendingNode.get();
        if (pending.valid())
        {
            //Remove children up to the history
            //removeChildren( 0, getNumChildren());
            while (getNumChildren() > _history)
            {
                removeChild(0u);
            }
            addChild( pending.get() );
            _callback->_pendingNode = 0;
        }
    }
    osg::Group::traverse(nv);
}