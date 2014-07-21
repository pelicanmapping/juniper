/* -*-c++-*- */
/* osgJuniper - Large Dataset Visualization Toolkit for OpenSceneGraph
* Copyright 2010-2011 Pelican Ventures, Inc.
* http://wush.net/trac/juniper
*
* osgJuniper is free software; you can redistribute it and/or modify
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

#include <osgJuniperMap/MapContext>

#include <osgUtil/IntersectionVisitor>
#include <osgUtil/LineSegmentIntersector>

#include <osg/CoordinateSystemNode>
#include <osgEarth/TerrainEngineNode>

#include <osgEarth/FindNode>

using namespace osgEarth;
using namespace osgJuniper::Map;

/***************************************************************************/
MapContext::MapContext(osg::Group* root, osgEarth::MapNode* mapNode, osgViewer::ViewerBase* viewer):
_root(root),
_mapNode( mapNode ),
_viewer( viewer )
{
}

const osg::EllipsoidModel*
MapContext::getEllipsoid() const
{
    return _mapNode->getMapSRS()->getEllipsoid();
}


bool
MapContext::getIntersection(const osg::Vec3d& position, osg::Vec3d& hit, osg::Vec3& normal)
{
    //Return if there is no terrain node assigned
    if (!_mapNode.valid()) return false;    

    return getNodeIntersection( _mapNode->getTerrainEngine(), _mapNode->getMap()->getProfile()->getSRS()->getEllipsoid(), position, hit, normal);    
}

bool
MapContext::getNodeIntersection(osg::Node* node, const osg::EllipsoidModel* ellipsoid, const osg::Vec3d& position, osg::Vec3d& hit, osg::Vec3& normal, double altitude)
{
    if (!node || !ellipsoid) return false;

    //Compute the up vector
    osg::Vec3d up = ellipsoid->computeLocalUpVector(position.x(), position.y(), position.z() );
    up.normalize();

    double segOffset = 100000;

    osg::Vec3d start = position + (up * segOffset);
    osg::Vec3d end = position - (up * segOffset);
    
    osgUtil::LineSegmentIntersector* i = new osgUtil::LineSegmentIntersector( start, end );
    
    osgUtil::IntersectionVisitor iv;
    iv.setIntersector( i );
    node->accept( iv );

    osgUtil::LineSegmentIntersector::Intersections& results = i->getIntersections();
    if ( !results.empty() )
    {   
        const osgUtil::LineSegmentIntersector::Intersection& result = *results.begin();
        hit    = result.getWorldIntersectPoint();
        normal = result.getWorldIntersectNormal();
        hit += (up * altitude);
        return true;
    }
    return false;    
}