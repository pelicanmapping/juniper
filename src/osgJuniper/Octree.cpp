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

#include <osgJuniper/Octree>

using namespace osgJuniper;

/****************************************************************************/
OctreeId::OctreeId(int in_level, int in_x, int in_y, int in_z):
level(in_level),
x(in_x),
y(in_y),
z(in_z)
{
}

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

OctreeNode* OctreeNode::createChild(const OctreeId& id)
{
    unsigned int dim = 1;
    for (unsigned int i = 0; i < id.level; i++)
    {
        dim *= 2;
    }

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
    unsigned int level = _id.level -1;
    unsigned int x = _id.x / 2;
    unsigned int y = _id.y / 2;
    unsigned int z = _id.z / 2;
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

void
OctreeNode::setBoundingBox(const osg::BoundingBoxd& boundingBox)
{
    _boundingBox = boundingBox;
}

void OctreeNode::traverse(OctreeNodeVisitor &v)
{
    if (_split)
    {
        for (OctreeNodeList::iterator itr = _children.begin(); itr != _children.end(); ++itr)
        {
            (*itr)->accept(v);
        }
    }   
}

void
OctreeNode::accept(OctreeNodeVisitor &v)
{
    v.apply(*this);
}


/****************************************************************************/
OctreeNodeVisitor::OctreeNodeVisitor()
{
}


void
OctreeNodeVisitor::apply(OctreeNode& node)
{
}

void
OctreeNodeVisitor::traverse(OctreeNode& node)
{
    node.traverse(*this);
}

/****************************************************************************/
CountPointsVisitor::CountPointsVisitor():
_numPoints(0)
{
}

void CountPointsVisitor::apply(OctreeNode& node)
{
    _numPoints += node.getPoints().size();
    traverse(node);
}

/****************************************************************************/
AddPointVisitor::AddPointVisitor(const Point& point, unsigned int maxLevel):
_point(point),
_maxLevel(maxLevel),
_strategy(ACCEPT),
_pointAdded(false)
{
}
        
void
AddPointVisitor::setStrategy(Strategy strategy)
{
    _strategy = strategy;
}

void
AddPointVisitor::setMaxLevel(unsigned int maxLevel)
{
    _maxLevel = maxLevel;
}

void
AddPointVisitor::apply(OctreeNode& node)
{
    //Only process if the point is in the nodes bounding box
    if (!_pointAdded && node.getBoundingBox().contains(_point.position))
    {
        //If the child has no points, just add the point to it.
        if (node.getPoints().empty() && !node.isSplit() )
        {
            node.getPoints().push_back( _point );
            _pointAdded = true;
        }
        //The child has a point already
        else if (node.getPoints().size() > 0 && !node.isSplit())
        {
            if (node.getID().level < _maxLevel)
            {
                //Split the child
                node.split();

                for (PointList::iterator itr = node.getPoints().begin(); itr != node.getPoints().end(); ++itr)
                {
                    AddPointVisitor v(*itr, _maxLevel);
                    node.accept( v );
                }

                //Remove all the points from this child node
                node.getPoints().clear();

                //Continue traversing down, we still need to add the current point
                traverse( node );
            }  
            else
            {
                //We've reached the maximum recursion level, so depending on the strategy, either accept or reject the point
                if (_strategy == ACCEPT)
                {
                    node.getPoints().push_back( _point );
                    _pointAdded = true;
                }
            }                 
        }
        else
        {
            traverse(node);
        }
    }                
}


/****************************************************************************/
CollectPointsVisitor::CollectPointsVisitor():
_remove(true)
{
}

bool
CollectPointsVisitor::getRemove() const
{
    return _remove;
}

void
CollectPointsVisitor::setRemove(bool remove)
{
    _remove = remove;
}

PointList&
CollectPointsVisitor::getPoints()
{
    return _points;
}

const PointList&
CollectPointsVisitor::getPoints() const
{
    return _points;
}

void CollectPointsVisitor::apply(OctreeNode& node)
{
    for (PointList::iterator itr = node.getPoints().begin(); itr != node.getPoints().end(); ++itr)
    {
        //Set the min range based on the radius of this OctreeNode's bounding box
        (*itr).minRange = node.getBoundingBox().radius();
        _points.push_back( *itr );
    }

    if (_remove)
    {
        //Erase all the points from the octree to free up memory
        node.getPoints().clear();
    }
    traverse(node);
}

/****************************************************************************/
ChooseRepresentativePointVisitor::ChooseRepresentativePointVisitor():
_valid(false)
{
}

void ChooseRepresentativePointVisitor::apply(OctreeNode& node)
{
    if (!_valid)
    {            
        if (node.isLeaf() && node.getPoints().size() > 0) 
        {
            //Choose a representative point for this LOD
            unsigned int count = 1;
            for (unsigned int i = 0; i < count; ++i)
            {
                if (node.getPoints().size() > 0)
                {
                    //Remove a random point
                    //unsigned int index = getRandomValueinRange(child->getPoints().size()-1);
                    _points.push_back( node.getPoints().front());                        
                    //Erase the point from the child
                    node.getPoints().pop_front();
                }
            }
            _valid = true;
        }
        else
        {
            traverse( node );
        }
    }
}


/****************************************************************************/
ApplyRepresentativePointVisitor::ApplyRepresentativePointVisitor()
{
}

void
ApplyRepresentativePointVisitor::apply(OctreeNode& node)
{
    if (!node.isLeaf() && node.getPoints().empty()) 
    {
        //This node is not a leaf and needs a representative point chosen from it's children
        ChooseRepresentativePointVisitor v;
        node.accept(v);
        if (v._valid)
        {
            //Add the point to this node
            for (PointList::iterator itr = v._points.begin(); itr != v._points.end(); ++itr)
            {
                node.getPoints().push_back( *itr );
            }
        }    
    }
    traverse(node);
}
   