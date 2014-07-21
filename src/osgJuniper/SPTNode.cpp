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

#include <osgJuniper/SPTNode>

using namespace osgJuniper;

struct SortPointsByRangeFunctor
{
    bool operator() (const Point& lhs,const Point& rhs) const
    {
        if (lhs._minRange > rhs._minRange) return true;
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
        bounds.expandBy( (*itr)._position );
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

    _colors = new osg::Vec4Array();
    _colors->reserve( points.size() );
    _geometry->setColorArray( _colors );
    _geometry->setColorBinding( osg::Geometry::BIND_PER_VERTEX);

    _normals = new osg::Vec3Array();
    _normals->reserve( points.size() );
    _geometry->setNormalArray( _normals );
    _geometry->setNormalBinding( osg::Geometry::BIND_PER_VERTEX );

    osg::ref_ptr<osg::Vec4Array> scales = new osg::Vec4Array;
    _geometry->setVertexAttribData(11, osg::Geometry::ArrayData(scales.get(), osg::Geometry::BIND_PER_VERTEX));

    while (points.size() > 0)
    {         
        _verts->push_back( points.front()._position );
        _colors->push_back( points.front()._color );
        _normals->push_back( points.front()._normal );
        scales->push_back( osg::Vec4(0,0,0, points.front()._size ));

        //Inititalize the count to 0 if this is the first time we've encountered this range
        if (_rangeMap.find(points.front()._minRange) == _rangeMap.end())
        {
            _rangeMap[points.front()._minRange] = 0;
        }

        //Increment the count for this range
        _rangeMap[points.front()._minRange] += 1;

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

