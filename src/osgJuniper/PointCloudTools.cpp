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
#include <osgJuniper/PointCloudTools>
#include <osgJuniper/PointCloud>

#include <osg/MatrixTransform>
#include <osg/LineWidth>
#include <osgEarth/GeoData>
#include <osgEarthUtil/LatLongFormatter>

using namespace osgJuniper;

/****************************************************************************/
IdentifyPointHandler::IdentifyPointHandler():
    _selectedColor(255, 0, 0, 255),
    _hoverColor(255,255, 0, 255),
    _mask(0xffffffff),
    _selectionRadius(5.0f)
{
}

const osg::Vec4ub& IdentifyPointHandler::getSelectedColor() const
{
    return _selectedColor;
}

void IdentifyPointHandler::setSelectedColor(const osg::Vec4ub& color)
{
    _selectedColor = color;
}

const osg::Vec4ub& IdentifyPointHandler::getHoverColor() const
{
    return _hoverColor;
}

void IdentifyPointHandler::setHoverColor(const osg::Vec4ub& color)
{
    _hoverColor = color;
}

osg::Node::NodeMask IdentifyPointHandler::getNodeMask() const
{
    return _mask;
}

void IdentifyPointHandler::setNodeMask(osg::Node::NodeMask mask)
{
    _mask = mask;
}

float IdentifyPointHandler::getSelectionRadius() const
{
    return _selectionRadius;
}

void IdentifyPointHandler::setSelectionRadius(float selectionRadius)
{
    _selectionRadius = selectionRadius;
}

void IdentifyPointHandler::addCallback(IdentifyPointHandler::Callback* callback)
{
    _callbacks.push_back(callback);
}

bool IdentifyPointHandler::handle(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa)
{
    osgViewer::View* viewer = dynamic_cast<osgViewer::View*>(&aa);
    if (!viewer) return false;

    switch (ea.getEventType())
    {
    case osgGA::GUIEventAdapter::PUSH:
        select(ea.getX(), ea.getY(), viewer);
        break;   
    case osgGA::GUIEventAdapter::MOVE:
        hover(ea.getX(), ea.getY(), viewer);
        break;
    }
    return false;
}

void IdentifyPointHandler::deselect()
{
    if (!_selectedPoints.empty())
    {
        for (SelectionList::iterator itr = _selectedPoints.begin(); itr != _selectedPoints.end(); itr++)
        {
            itr->reset();
        }
        _selectedPoints.clear();
    }
}

void IdentifyPointHandler::unhover()
{
    if (!_hoveredPoints.empty())
    {
        for (SelectionList::iterator itr = _hoveredPoints.begin(); itr != _hoveredPoints.end(); itr++)
        {
            itr->reset();
        }
        _hoveredPoints.clear();
    }
}

void IdentifyPointHandler::hover(float x, float y, osgViewer::View* viewer)
{
    unhover();

    osgUtil::PolytopeIntersector *picker = new osgUtil::PolytopeIntersector(osgUtil::Intersector::WINDOW, x - _selectionRadius, y- _selectionRadius, x + _selectionRadius, y + _selectionRadius);
    picker->setIntersectionLimit(osgUtil::Intersector::LIMIT_ONE);
    osgUtil::IntersectionVisitor iv(picker);
    iv.setTraversalMask(_mask);
    viewer->getCamera()->accept(iv);
    if (picker->containsIntersections())
    {
        // Get the point that was clicked.
        osgUtil::PolytopeIntersector::Intersection hit = picker->getFirstIntersection();
        
        PointCloudDecorator* decorator = 0;
        // Make sure we are selecting a PointCloudDecorator
        for (unsigned int i = 0; i < hit.nodePath.size(); i++)
        {
            decorator = dynamic_cast<PointCloudDecorator*>(hit.nodePath[i]);
            if (decorator)
            {
                break;
            }
        }        
        // We didn't pick a point cloud.
        if (!decorator) return;
        osg::Geometry* geometry = hit.drawable->asGeometry();

        // Highlight the selected point
        osg::Vec4ubArray* colors = static_cast<osg::Vec4ubArray*>(geometry->getColorArray());

        osg::Vec4ub prevColor = (*colors)[hit.primitiveIndex];
        // Store the current values.
        _hoveredPoints.push_back(SelectionInfo(colors, prevColor, hit.primitiveIndex));

        // Changed the color
        (*colors)[hit.primitiveIndex] = _hoverColor;
        colors->dirty();
    }
}

void IdentifyPointHandler::select(float x, float y, osgViewer::View* viewer)
{
    unhover();
    deselect();

    osgUtil::PolytopeIntersector *picker = new osgUtil::PolytopeIntersector(osgUtil::Intersector::WINDOW, x - _selectionRadius, y- _selectionRadius, x + _selectionRadius, y + _selectionRadius);
    picker->setIntersectionLimit(osgUtil::Intersector::LIMIT_ONE);
    osgUtil::IntersectionVisitor iv(picker);
    iv.setTraversalMask(_mask);
    viewer->getCamera()->accept(iv);
    if (picker->containsIntersections())
    {
        // Get the point that was clicked.
        osgUtil::PolytopeIntersector::Intersection hit = picker->getFirstIntersection();
        
        PointCloudDecorator* decorator = 0;
        // Make sure we are selecting a PointCloudDecorator
        for (unsigned int i = 0; i < hit.nodePath.size(); i++)
        {
            decorator = dynamic_cast<PointCloudDecorator*>(hit.nodePath[i]);
            if (decorator)
            {
                break;
            }
        }        
        // We didn't pick a point cloud.
        if (!decorator) return;       

        osg::Geometry* geometry = hit.drawable->asGeometry();

        osg::Vec3Array* verts = static_cast<osg::Vec3Array*>(geometry->getVertexArray());
        // Since we are just drawing points the point that was clicked is just the primitive index.
        osg::Vec3 vert = (*verts)[hit.primitiveIndex];            

        // Highlight the selected point
        osg::Vec4ubArray* colors = static_cast<osg::Vec4ubArray*>(geometry->getColorArray());

        osg::Vec4ub prevColor = (*colors)[hit.primitiveIndex];
        // Store the current values.
        _selectedPoints.push_back(SelectionInfo(colors, prevColor, hit.primitiveIndex));

        // Changed the color
        (*colors)[hit.primitiveIndex] = _selectedColor;
        colors->dirty();

        osg::Matrixd localToWorld = osg::computeLocalToWorld(hit.nodePath);

        // Get the geocentric world coordinate
        osg::Vec3d world = vert * localToWorld; 

        // Get the data from the vertex attributes.
        osg::Vec3usArray* dataArray = static_cast<osg::Vec3usArray*>(geometry->getVertexAttribArray(osg::Drawable::ATTRIBUTE_6));
        osg::Vec3us data = (*dataArray)[hit.primitiveIndex];

        // Notify any callbacks of the point selection.
        Point point;
		point.x = world.x();
		point.y = world.y();
		point.z = world.z();
		/*
		point.r = prevColor.r() * 255;
		point.g = prevColor.g() * 255;
		point.b = prevColor.b() * 255;
		point.a = prevColor.a() * 255;
        point.classification = data.x();
        point.returnNumber = data.y();
        point.intensity = data.z();
		*/
        for( Callbacks::iterator i = _callbacks.begin(); i != _callbacks.end(); ++i )
            i->get()->selected(point);        
    }
    else
    {
        for( Callbacks::iterator i = _callbacks.begin(); i != _callbacks.end(); ++i )
            i->get()->reset();
    }
}    

/****************************************************************************/
/**
* Point to point measure handler
*/
P2PMeasureHandler::P2PMeasureHandler(osg::Group* root):
    _root(root),
    _mask(0xffffffff),
    _selectionRadius(5.0f)
{
}

osg::Node::NodeMask P2PMeasureHandler::getNodeMask() const
{
    return _mask;
}

void P2PMeasureHandler::setNodeMask(osg::Node::NodeMask mask)
{
    _mask = mask;
}

float P2PMeasureHandler::getSelectionRadius() const
{
    return _selectionRadius;
}

void P2PMeasureHandler::setSelectionRadius(float selectionRadius)
{
    _selectionRadius = selectionRadius;
}

bool P2PMeasureHandler::handle(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa)
{
    osgViewer::View* view = dynamic_cast<osgViewer::View*>(&aa);
    if (!view) return false;

    switch (ea.getEventType())
    {
    case osgGA::GUIEventAdapter::PUSH:
        if (ea.getButton() == osgGA::GUIEventAdapter::LEFT_MOUSE_BUTTON)
        {
            pick(ea.getX(), ea.getY(), view);
        }
        else if (ea.getButtonMask() == osgGA::GUIEventAdapter::RIGHT_MOUSE_BUTTON)
        {
            clear();
        }
        break;   
    }
    return false;
}

void P2PMeasureHandler::clear()
{
    _points.clear();
    updateMeasurement();
}

void P2PMeasureHandler::addCallback(P2PMeasureHandler::Callback* callback)
{
    _callbacks.push_back(callback);
}

void P2PMeasureHandler::pick(float x, float y, osgViewer::View* view)
{        
    osgUtil::PolytopeIntersector *picker = new osgUtil::PolytopeIntersector(osgUtil::Intersector::WINDOW, x - _selectionRadius, y- _selectionRadius, x + _selectionRadius, y + _selectionRadius);
    picker->setIntersectionLimit(osgUtil::Intersector::LIMIT_ONE);
    osgUtil::IntersectionVisitor iv(picker);
    iv.setTraversalMask(_mask);
    view->getCamera()->accept(iv);
    if (picker->containsIntersections())
    {
        // Get the point that was clicked.
        osgUtil::PolytopeIntersector::Intersection hit = picker->getFirstIntersection();
        
        PointCloudDecorator* decorator = 0;
        // Make sure we are selecting a PointCloudDecorator
        for (unsigned int i = 0; i < hit.nodePath.size(); i++)
        {
            decorator = dynamic_cast<PointCloudDecorator*>(hit.nodePath[i]);
            if (decorator)
            {
                break;
            }
        }        
        // We didn't pick a point cloud.
        if (!decorator) return;

        osg::Geometry* geometry = hit.drawable->asGeometry();

        osg::Vec3Array* verts = static_cast<osg::Vec3Array*>(geometry->getVertexArray());
        // Since we are just drawing points the point that was clicked is just the primitive index.
        osg::Vec3 vert = (*verts)[hit.primitiveIndex];            

        osg::Matrixd localToWorld = osg::computeLocalToWorld(hit.nodePath);

        // Get the geocentric world coordinate
        osg::Vec3d world = vert * localToWorld;            

        _points.push_back( world );
        updateMeasurement();
    }
}

void P2PMeasureHandler::updateMeasurement()
{
    // Remove the line.
    if (_line.valid())
    {
        _root->removeChild( _line.get() );
        _line = 0;
    }

    double distance = 0.0;

    if (_points.size() >=2 )
    {
        // Build the line
        osg::MatrixTransform* mt = new osg::MatrixTransform;
        osg::Vec3d anchor = _points[0];
        mt->setMatrix(osg::Matrixd::translate(anchor));

        osg::Geometry* geometry = new osg::Geometry;
        osg::Vec3Array* verts = new osg::Vec3Array;
        verts->reserve(_points.size());
        geometry->setVertexArray( verts );

        osg::Vec4Array* colors = new osg::Vec4Array(1);
        (*colors)[0] = osg::Vec4(1.0f, 0.0, 0.0f, 1.0f);            
        geometry->setColorArray( colors );
        geometry->setColorBinding( osg::Geometry::BIND_OVERALL);

        distance = 0.0;

        for (unsigned int i = 0; i < _points.size(); i++)
        {
            if (i != 0)
            {
                distance += (_points[i] - _points[i-1]).length();
            }
            verts->push_back(_points[i] - anchor );
        }

        geometry->addPrimitiveSet(new osg::DrawArrays(GL_LINE_STRIP, 0, verts->size()));

        osg::Geode* geode = new osg::Geode;
        geode->addDrawable(geometry);

        mt->addChild( geode );

        mt->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
        mt->getOrCreateStateSet()->setMode(GL_DEPTH_TEST, osg::StateAttribute::OFF);
        mt->getOrCreateStateSet()->setAttribute(new osg::LineWidth(2.0));
        _line = mt;

        _root->addChild( _line.get() );
    }

    for( Callbacks::iterator i = _callbacks.begin(); i != _callbacks.end(); ++i )
            i->get()->distanceChanged(distance);
}
/****************************************************************************/