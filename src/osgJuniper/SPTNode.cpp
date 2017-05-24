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
#include <osgJuniper/SPTNode>

using namespace osgJuniper;

struct SortPointsByRangeFunctor
{
    bool operator() (const Point& lhs,const Point& rhs) const
    {
        if (lhs.minRange > rhs.minRange) return true;
        else return false;
    }
};

SPTNode::SPTNode():
_geometry(0),
_verts(0),
_colors(0),
_drawArrays(0),
_radiusFactor(200),
_enabled(true),
_maxVerts(UINT_MAX)
{
}

float
SPTNode::getRadiusFactor() const
{
    return _radiusFactor;
}

void
SPTNode::setRadiusFactor(float radiusFactor)
{
    _radiusFactor = radiusFactor;
}

bool
SPTNode::getEnabled() const
{
    return _enabled;
}

void
SPTNode::setEnabled( bool enabled)
{
    _enabled = enabled;
}

void
SPTNode::setMaxVerts( unsigned int maxVerts )
{
    _maxVerts = maxVerts;
}

unsigned int
SPTNode::getMaxVerts() const
{
    return _maxVerts;
}

void SPTNode::setPoints(PointList &points)
{
    //Sort the incoming points by max range
    //std::sort(points.begin(), points.end(), SortPointsByRangeFunctor());
    points.sort(SortPointsByRangeFunctor());

    osg::BoundingSphere bounds;
    for (PointList::iterator itr = points.begin(); itr != points.end(); ++itr)
    {
        bounds.expandBy( (*itr).position );
    }

    setInitialBound( bounds );

    //Remove the existing geometry if there is one
    if (_geometry) removeDrawable( _geometry );

    _geometry = new osg::Geometry;
    _geometry->setUseVertexBufferObjects( true );
    _geometry->setUseDisplayList( false );
    _geometry->setDataVariance(osg::Object::DYNAMIC);

    _verts = new osg::Vec3Array;
    _verts->reserve( points.size() );
    _geometry->setVertexArray( _verts );    

    _colors = new osg::Vec4ubArray();
    _colors->reserve( points.size() );
    _geometry->setColorArray( _colors );
    _geometry->setColorBinding( osg::Geometry::BIND_PER_VERTEX);

    _normals = new osg::Vec3Array();
    _normals->reserve( points.size() );
    _geometry->setNormalArray( _normals );
    _geometry->setNormalBinding( osg::Geometry::BIND_PER_VERTEX );

    osg::ref_ptr<osg::Vec4Array> scales = new osg::Vec4Array;
    _geometry->setVertexAttribArray(11, scales.get());
    _geometry->setVertexAttribBinding(11, osg::Geometry::BIND_PER_VERTEX);
    _geometry->setVertexAttribNormalize(11, false);        

    while (points.size() > 0)
    {         
        _verts->push_back( points.front().position );
        _colors->push_back( points.front().color );
        _normals->push_back( points.front().normal );
        scales->push_back( osg::Vec4(0,0,0, points.front().size ));

        //Inititalize the count to 0 if this is the first time we've encountered this range
        if (_rangeMap.find(points.front().minRange) == _rangeMap.end())
        {
            _rangeMap[points.front().minRange] = 0;
        }

        //Increment the count for this range
        _rangeMap[points.front().minRange] += 1;

        //Remove the point from the list to reduce memory usage
        points.pop_front();
    }

    //The rangemap now contains a map containing the number of times each range occurs.
    //Increment all of the count values to include the previous ranges as well
    //It is sorted in increasing levels, but we want to compute the counts from the max range down so use a reverse iterator
    unsigned int prevCount = 0;
    for (RangeMap::reverse_iterator itr = _rangeMap.rbegin(); itr != _rangeMap.rend(); ++itr)
    {
        //osg::notify(osg::NOTICE) << "Initializing range " << (*itr).first << " which has count " << (*itr).second << std::endl;
        (*itr).second += prevCount;
        prevCount = (*itr).second;
    }
    
    //Now the range list 
    //_drawArrays = new osg::DrawArrays(GL_POINTS, 0, 0);
    _drawArrays = new osg::DrawArrays(GL_POINTS, 0, _verts->size());
    _geometry->addPrimitiveSet( _drawArrays );
    addDrawable( _geometry );
}

void
SPTNode::traverse(osg::NodeVisitor &nv)
{
    if (nv.getVisitorType() == osg::NodeVisitor::CULL_VISITOR)
    {
        float required_range = nv.getDistanceToViewPoint(getBound().center(),true);
        required_range /= _radiusFactor;
        RangeMap::iterator itr = _rangeMap.upper_bound(required_range);
        if (_geometry && _drawArrays && _verts)
        {
            unsigned int count = 0;
            if (!_enabled)
            {
            count = _verts->size();
            }
            else
            {
                if (itr == _rangeMap.end())
                {
                    if (required_range >= (*_rangeMap.rbegin()).first)
                        count = (*_rangeMap.rbegin()).second;
                    else if (required_range <= (*_rangeMap.begin()).first)
                        count = (*_rangeMap.begin()).first;
                }

                if (itr != _rangeMap.end())
                {
                    count = itr->second;
                }
            }
            count = osg::clampBetween(count, 0u, _maxVerts);
            //osg::notify(osg::NOTICE) << "Drawing " << count << " for range " << required_range << std::endl;
            _drawArrays->setCount( count );
        }
    }
    osg::Geode::traverse(nv);
}

