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
#include <osgJuniper/Octree>

using namespace osgJuniper;

/****************************************************************************/

OctreeNode::OctreeNode():
//Assume root node
_id(0,0,0,0),
_split(false)
{    
}

double OctreeNode::getWidth() const
{
    return _boundingBox.xMax() - _boundingBox.xMin();
}

double OctreeNode::getHeight() const
{
    return _boundingBox.zMax() - _boundingBox.zMin();
}

double OctreeNode::getDepth() const
{
    return _boundingBox.yMax() - _boundingBox.yMin();
}

OctreeId
OctreeNode::getID(const osg::Vec3d& point, unsigned int level) const
{
    unsigned int dim = 1;
    for (unsigned int i = 0; i < level; i++)
    {
        dim *= 2;
    }

    double width  = getWidth();
    double height = getHeight();
    double depth  = getDepth();
    
    double rx = (point.x() - _boundingBox.xMin()) / width;
    double ry = (point.y() - _boundingBox.yMin()) / depth;
    double rz = (point.z() - _boundingBox.zMin()) / height;

    int tileX = osg::clampBelow( (unsigned int)(rx * (double)dim), dim-1 );
    int tileY = osg::clampBelow( (unsigned int)(ry * (double)dim), dim-1 );
    int tileZ = osg::clampBelow( (unsigned int)(rz * (double)dim), dim-1 );    

    return OctreeId(level, tileX, tileY, tileZ);
}


unsigned int OctreeNode::getDimensions(unsigned int level) const
{
	unsigned int dim = 1;
	for (unsigned int i = 0; i < level; i++)
	{
		dim *= 2;
	}
	return dim;
}

void OctreeNode::clearPoints()
{
	// Rather than calling clear we swap this vector with an empty one to force memory to actually be deallocated.
    //https://prateekvjoshi.com/2013/10/20/c-vector-memory-release/
	PointList pts;
	_points.swap(pts);
}

OctreeNode* OctreeNode::createChild(const OctreeId& id)
{    
	unsigned int dim = getDimensions(id.level);

    double width  = getWidth() / (double)dim;
    double height = getHeight() / (double)dim;
    double depth  = getDepth() / (double)dim;

    double minX = _boundingBox.xMin() + (double)id.x * width;
    double minY = _boundingBox.yMin() + (double)id.y * depth;
    double minZ = _boundingBox.zMin() + (double)id.z * height;

    OctreeNode* node = new OctreeNode();
    node->setId(id);
    node->setBoundingBox( osg::BoundingBoxd(minX, minY, minZ,
                                            minX + width, minY + depth, minZ + height));
    return node;
}
        
OctreeId OctreeNode::getParentID() const
{    
	return getParentID(_id);
}

OctreeId OctreeNode::getParentID(const OctreeId& id)
{
	unsigned int level = id.level - 1;
	unsigned int x = id.x / 2;
	unsigned int y = id.y / 2;
	unsigned int z = id.z / 2;
	/*
	OSG_NOTICE << "Parent of " << id.level << ", " << " (" << id.x << ", " << id.y << ", " << id.z << ") is " 
		       << level << ", " << " (" << x << ", " << y << ", " << z << ")" << std::endl;
			   */
	return OctreeId(level, x, y, z);
}

OctreeNode*
OctreeNode::createChild(unsigned int childNumber)
{
    //Compute the new dimensions of this child
    double width  = getWidth() / 2.0;
    double height = getHeight() / 2.0;
    double depth  = getDepth() / 2.0;

    double xMin = _boundingBox.xMin();
    double yMin = _boundingBox.yMin();
    double zMin = _boundingBox.zMin();

    OctreeId id(_id.level+1, _id.x * 2, _id.y * 2, _id.z * 2);

    //Child 0 is the lower left, back child.  The min values are initialized to child 0.

    //Child 1 is the lower right, back child
    if (childNumber == 1)
    {
        xMin += width;
        id.x+=1;
    }
    //Child 2 is the upper left, back child
    else if (childNumber == 2)
    {
        zMin += height;
        id.z+=1;
    }
    //Child 3 is the upper right, back child
    else if (childNumber == 3)
    {
        xMin += width;
        zMin += height;
        id.x+=1;
        id.z+=1;
    }
    //Child 4 is the lower left, front child.
    else if (childNumber == 4)
    {
        yMin += depth;
        id.y+=1;
    }
    //Child 5 is the lower right, front child
    else if (childNumber == 5)
    {
        yMin += depth;
        xMin += width;
        id.y+=1;
        id.x+=1;
    }
    //Child 6 is the upper left, front child
    else if (childNumber == 6)
    {
        yMin += depth;
        zMin += height;
        id.y+=1;
        id.z+=1;
    }
    //Child 7 is the upper right, front child
    else if (childNumber == 7)
    {
        yMin += depth;
        xMin += width;
        zMin += height;
        id.y+=1;
        id.x+=1;
        id.z+=1;
    }

    double xMax = xMin + width;
    double yMax = yMin + depth;
    double zMax = zMin + height;

    OctreeNode* child = new OctreeNode();
    child->setBoundingBox(osg::BoundingBoxd(xMin, yMin, zMin, xMax, yMax, zMax));
    child->setId(id);
    return child;
}


void
OctreeNode::setId(const OctreeId& id)
{
    _id = id;
}

PointList&
OctreeNode::getPoints()
{
    return _points;
}

const PointList&
OctreeNode::getPoints() const
{
    return _points;
}

bool
OctreeNode::isSplit() const
{
    return _split;
}

bool
OctreeNode::isLeaf() const
{
    return !_split;
}

void
OctreeNode::split()
{
    if (!isSplit())
    {
        _children.reserve(8);
        for (unsigned int i = 0; i < 8; ++i)
        {
            _children.push_back(createChild(i));
        }
        _split = true;
    }
}

const osg::BoundingBoxd& OctreeNode::getBoundingBox() const
{
    return _boundingBox;
}

osg::BoundingBoxd& OctreeNode::getBoundingBox()
{
    return _boundingBox;
}

void
OctreeNode::setBoundingBox(const osg::BoundingBoxd& boundingBox)
{
    _boundingBox = boundingBox;
}   