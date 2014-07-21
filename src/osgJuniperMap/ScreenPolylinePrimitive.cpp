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

#include <osgJuniperMap/ScreenPolylinePrimitive>

#include <osg/Geode>

#include <osgJuniperMap/Registry>

#include <sstream>

using namespace osgJuniper::Map;

REGISTER_SIMPLE_PRIMITIVE_FACTORY(ScreenPolylinePrimitive)
REGISTER_SIMPLE_PRIMITIVE_FACTORY_TYPE(ScreenPolylinePrimitive, ScreenPolyline)


/***************************************************************************/
ScreenPolylinePrimitive::ScreenPolylinePrimitive(PrimitiveId id):
Primitive(id),
_alpha(1.0)
{
	this->getOrCreateStateSet()->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);
	getOrCreateStateSet()->setMode(GL_DEPTH_TEST, osg::StateAttribute::OFF);

	_geode = new osg::Geode();
    // create Geometry object to store all the vertices and lines primitive.
    _geometry = new osg::Geometry();
    
    _points = new osg::Vec2Array;

    // set the colors
    _color = new osg::Vec4Array;
    _color->push_back(osg::Vec4(1.0f,1.0f,0.0f,1.0f));
    _geometry->setColorArray(_color);
    _geometry->setColorBinding(osg::Geometry::BIND_OVERALL);
    
	_lineWidth = new osg::LineWidth();


    // add the points geometry to the geode.
    _geode->addDrawable(_geometry);

    addChild( _geode.get() );
}

ScreenPolylinePrimitive::~ScreenPolylinePrimitive()
{
}

void
ScreenPolylinePrimitive::updateTransform()
{
	_geometry->setVertexArray(_points);
	_geometry->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::LINES, 0, _points->size()));
}

float
ScreenPolylinePrimitive::getLineWidth() const
{
	return _lineWidth->getWidth();
}

void
ScreenPolylinePrimitive::setLineWidth( float lineWidth )
{
	if (_lineWidth->getWidth() != lineWidth)
    {
        _lineWidth->setWidth(lineWidth);
		_geode->getOrCreateStateSet()->setAttributeAndModes(_lineWidth, osg::StateAttribute::ON);
    }
}

osg::Vec4&
ScreenPolylinePrimitive::getColor() const
{
	if( _color->size() > 0 )
		return _color->asVector()[0];
}

void
ScreenPolylinePrimitive::setColor(osg::Vec4& color)
{
	if( _color->size() > 0 && _color.get()->asVector()[0] != color )
    {
		_color->pop_back();
		_color->push_back(color);
		_geometry->setColorArray(_color);
    }
}

float
ScreenPolylinePrimitive::getAlpha() const
{
	return _alpha;
}

void
ScreenPolylinePrimitive::setAlpha(float alpha)
{
	if( _color->size() > 0 && _alpha != alpha )
    {
		_alpha = alpha;

		osg::Vec4 colorOldAlpha = getColor();

		_color->pop_back();
		colorOldAlpha[3] = _alpha;
		_color->push_back(colorOldAlpha);
		_geometry->setColorArray(_color);
    }
}

ScreenPolylinePrimitive::LineStyle
ScreenPolylinePrimitive::getLineStyle() const
{
    return _lineStyle;
}

void
ScreenPolylinePrimitive::setLineStyle( ScreenPolylinePrimitive::LineStyle lineStyle )
{
    if (_lineStyle != lineStyle)
		_lineStyle = lineStyle;
}

void
ScreenPolylinePrimitive::setProperty(const std::string& name, const std::string& value)
{
    Primitive::setProperty(name, value);
}

void
ScreenPolylinePrimitive::setProperty(const std::string& name, const osg::Vec3d& value)
{
	if (name == PROP_FGCOLOR)
        setColor(osg::Vec4(value.x(), value.y(), value.z(), _alpha));
	else
		Primitive::setProperty(name, value);
}

void
ScreenPolylinePrimitive::setProperty(const std::string& name, int    value)
{
	if( name == PROP_WIDTH )
		setLineWidth(value);
	//else if (name == PROP_LINESTYLE)
	//	setLineStyle((LineStyle)value);
	else if( name == "transparency" )
		setAlpha(1 - ((float) value / 100));
	else
		Primitive::setProperty(name, value);
}

void
ScreenPolylinePrimitive::setProperty(const std::string& name, double value)
{
    Primitive::setProperty(name, value);
}

void
ScreenPolylinePrimitive::setProperty(const std::string& name, std::vector< osg::Vec2 > points)
{
	for(std::vector< osg::Vec2 >::iterator k = points.begin(); k != points.end(); ++k){
		_points->push_back(*k);
	}
	updateTransform();
}
