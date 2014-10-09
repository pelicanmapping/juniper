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
    _prevSelectedIndex(0),
    _mask(0xffffffff)
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

osg::Node::NodeMask IdentifyPointHandler::getNodeMask() const
{
    return _mask;
}

void IdentifyPointHandler::setNodeMask(osg::Node::NodeMask mask)
{
    _mask = mask;
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
        pick(ea.getX(), ea.getY(), viewer);
        break;            
    }
    return false;
}

void IdentifyPointHandler::pick(float x, float y, osgViewer::View* viewer)
{
    // Unset the previous selected point if we have one.
    if (_prevColorArray.valid())
    {
        (*_prevColorArray.get())[_prevSelectedIndex] = _prevSelectedColor;
        _prevColorArray->dirty();
        _prevColorArray = 0;
    }

    double w = 5.0;
    double h = 5.0;
    osgUtil::PolytopeIntersector *picker = new osgUtil::PolytopeIntersector(osgUtil::Intersector::WINDOW, x - w, y- h, x + w, y + h);
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

        // Store the current values.
        _prevColorArray = colors;
        _prevSelectedColor = (*colors)[hit.primitiveIndex];
        _prevSelectedIndex = hit.primitiveIndex;            

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
        point.position = world;
        point.color = _prevSelectedColor;
        point.classification = data.x();
        point.returnNumber = data.y();
        point.intensity = data.z();
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
    _mask(0xffffffff)
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
    double w = 5.0;
    double h = 5.0;
    osgUtil::PolytopeIntersector *picker = new osgUtil::PolytopeIntersector(osgUtil::Intersector::WINDOW, x - w, y- h, x + w, y + h);
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