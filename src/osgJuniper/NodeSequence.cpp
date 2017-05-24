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
#include <osgJuniper/NodeSequence>

#include <osg/Notify>
#include <osg/Timer>
#include <osg/Version>


using namespace osgJuniper;

NodeSequence::NodeSequence():
_activeIndex(-1),
_bufferStartIndex(-1),
_bufferEndIndex(-1),
_bufferSize(50)
{
    setNumChildrenRequiringUpdateTraversal( 1 );
    _groups.reserve(_bufferSize);
    _requests.reserve(_bufferSize);
}       

NodeSequence::NodeSequence(const NodeSequence& rhs, const osg::CopyOp &copycop)
{
    //TODO
    setNumChildrenRequiringUpdateTraversal(getNumChildrenRequiringUpdateTraversal()+1);            
}

void
NodeSequence::traverse(osg::NodeVisitor& nv)
{
    if (_activeIndex >= 0 && _activeIndex < _filenames.size())
    {
        //Compute the buffer index
        int bufferIndex = _activeIndex - _bufferStartIndex;
        //osg::notify(osg::NOTICE) << "Active index= " << _activeIndex << "  BufferIndex=" << bufferIndex << std::endl;
        
        
        bool traversed = false;
        //Traverse the group if it is valid and it has some children
        if (_groups[bufferIndex].valid() && _groups[bufferIndex]->getNumChildren() > 0)
        {
            //osg::notify(osg::NOTICE) << "Traversing group " << _activeIndex << " filename=" << _filenames[_activeIndex] << std::endl;
            _groups[bufferIndex]->traverse(nv);
            traversed = true;
        }
        
        
        if (nv.getDatabaseRequestHandler())
        {
            //Preload
            for (unsigned int i = 0; i < _bufferSize; ++i)
            {
                //Create the group if needed
                if (!_groups[i].valid()) _groups[i] = new osg::Group;

                if (!_groups[i]->getNumChildren() > 0)
                {
                    unsigned int filenameIndex = _bufferStartIndex + i;
                    if (filenameIndex >= 0 && filenameIndex < getNumFilenames())
                    {
                        //osg::notify(osg::NOTICE) << "Requesting " << _filenames[filenameIndex] << std::endl;
                        osg::Timer_t startTime = osg::Timer::instance()->tick();
#if OSG_MIN_VERSION_REQUIRED(2,9,11)
                        osg::NodePath nodePath = osg::NodePath(nv.getNodePath());
                        nodePath.push_back( _groups[i] );
                        nv.getDatabaseRequestHandler()->requestNodeFile(_filenames[filenameIndex], nodePath, 1.0f - (float)i, nv.getFrameStamp(), _requests[i]);
#else
                        nv.getDatabaseRequestHandler()->requestNodeFile(_filenames[filenameIndex], _groups[i], 1.0f - (float)i, nv.getFrameStamp(), _requests[i]);
#endif

                        osg::Timer_t endTime = osg::Timer::instance()->tick();
                        //osg::notify(osg::NOTICE)  << "Requested " << _filenames[filenameIndex] << " in " << osg::Timer::instance()->delta_m(startTime, endTime) << std::endl;
                    }
                }
            }

            //We need to traverse *SOMETHING* while we're waiting on the real node to appear, so go backwards until we find something valid.
            if (!traversed)
            {
                int i = bufferIndex;
                while (i > 0)
                {
                    i--;
                    if (_groups[i].valid() && _groups[i]->getNumChildren() > 0)
                    {
                        osg::notify(osg::DEBUG_INFO) << "Traversing " << i << " while " << _activeIndex << " is loading..." << std::endl;
                        _groups[i]->traverse(nv);
                        break;
                    }
                }
            }
        }
    }
    else
    {
        osg::Group::traverse(nv);
    }
}

void
NodeSequence::addFile(const std::string& filename)
{
    _filenames.push_back( filename );
}

std::string
NodeSequence::getFilename(unsigned int i) const
{
    return _filenames[i];
}


unsigned int
NodeSequence::getNumFilenames() const
{
    return _filenames.size();
}

void
NodeSequence::setActive(int index)
{
    if (_activeIndex != index && index >= 0 && index < _filenames.size())
    {
        _activeIndex = index;
        osg::notify(osg::INFO) << "Set active to " << index << std::endl;

        if (_activeIndex >= _bufferStartIndex && _activeIndex <= _bufferEndIndex)
        {
            //The index is within the current range, so just shift things down a bit
            int numToErase = _activeIndex - _bufferStartIndex;
            _groups.erase(_groups.begin(), _groups.begin() + numToErase);
            _requests.erase(_requests.begin(), _requests.begin() + numToErase);            
        }
        else
        {
            //They've moved to a completely different section of the stream.  Erase everything
            _groups.clear();
            _requests.clear();
        }

        _groups.resize( _bufferSize );
        _requests.resize( _bufferSize );
        _bufferStartIndex = _activeIndex;
        _bufferEndIndex = _bufferStartIndex + _bufferSize-1;

    }
}

int
NodeSequence::getActive() const
{
    return _activeIndex;
}

unsigned int
NodeSequence::getBufferSize() const
{
    return _bufferSize;
}


void
NodeSequence::setBufferSize(unsigned int bufferSize)
{
    if (_bufferSize != bufferSize)
    {
        _bufferSize = bufferSize;
        //Clear out the current buffer completely
        _groups.clear();
        _requests.clear();
        _groups.resize( _bufferSize );
        _requests.resize( _bufferSize );
        _bufferStartIndex = _activeIndex;
        _bufferEndIndex = _bufferStartIndex + _bufferSize;
        osg::notify(osg::INFO) << "Set buffer size to " << _bufferSize << std::endl;
    }
}

void
NodeSequence::seek(int offset)
{
    int next = _activeIndex + offset;
    if (next >= getNumFilenames())
    {
        next = 0;
    }
    else if (next < 0)
    {
        next = getNumFilenames()-1;
    }
    setActive( next );
}