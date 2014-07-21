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

#include <osgJuniperMap/RangeRingPrimitive>
#include <osgJuniperMap/Registry>

#include <osg/io_utils>
#include <sstream>
#include <osg/ShapeDrawable>
#include <osg/PolygonMode>
#include <osg/LineWidth>

#include <osgEarth/GeoMath>
#include <osgEarthSymbology/MeshSubdivider>

using namespace osgJuniper::Map;
using namespace osgEarth::Symbology;

REGISTER_SIMPLE_PRIMITIVE_FACTORY(RangeRingPrimitive)
REGISTER_SIMPLE_PRIMITIVE_FACTORY_TYPE(RangeRingPrimitive, RangeRing)

/***************************************************************************/
RangeRingPrimitive::RangeRingPrimitive(PrimitiveId id):
Primitive(id),
_color(1,1,1,1),
_range(10),
_lineWidth(2.0f)
{
    //Make the transform
    _transform = new osg::MatrixTransform;
    addChild( _transform.get() );

    //Add the geode to the scene graph
    _geode = new osg::Geode;
    _transform->addChild( _geode );

    getOrCreateStateSet()->setAttributeAndModes(new osg::LineWidth(_lineWidth)); 

    init();
}

RangeRingPrimitive::~RangeRingPrimitive()
{
}

double
RangeRingPrimitive::getRange() const
{
    return _range;
}

void
RangeRingPrimitive::setRange(double range)
{
    if (_range != range)
    {
        _range = range;
        init();
    }
}

const Location&
RangeRingPrimitive::getLocation() const
{
    return _location;
}

void
RangeRingPrimitive::setLocation(const Location& location)
{
    if (_location != location)
    {
        _location = location;
        init();
    }
}

void
RangeRingPrimitive::setMapContext(MapContext* context)
{    
    Primitive::setMapContext(context);
    init();
}


const osg::Vec4&
RangeRingPrimitive::getColor() const
{
    return _color;
}

void
RangeRingPrimitive::setColor( const osg::Vec4& color)
{
    if (_color != color)
    {
        _color = color;    
        init();
    }
}

void
RangeRingPrimitive::init()
{    
    osg::ref_ptr< const osg::EllipsoidModel > em;
    if (_context.valid())
    {
        em = _context->getEllipsoid();
    }
    else
    {        
        em = getWGS84Ellipsoid();
    }

    //Remove all existing drawables
    _geode->removeDrawables(0, _geode->getNumDrawables());

    int numSegments = 50;

    std::vector< osg::Vec3d > verts;
    verts.reserve( numSegments);

    double delta = (2.0 * osg::PI) / (double)numSegments;
    double center_lat = _location.getLatitudeRad();
    double center_lon = _location.getLongitudeRad();
    for (unsigned int i = 0; i < numSegments; ++i)
    {
        double lat, lon;
        osgEarth::GeoMath::destination(center_lat, center_lon, (double)i * delta, _range, lat, lon);
        //Convert it to geocentric
        double x, y, z;
        em->convertLatLongHeightToXYZ(lat, lon, 0, x, y, z);
        verts.push_back( osg::Vec3d( x, y, z) );
    }        

    osg::Vec3d origin = osg::Vec3d(verts[0]);    
    for (unsigned int i = 0; i < verts.size(); ++i)
    {
        verts[i] -= origin;
    }    

    _transform->setMatrix( osg::Matrixd::translate( origin ) );

    osg::Vec3Array* vert_array = new osg::Vec3Array();
    for (unsigned int i = 0; i < verts.size(); ++i)
    {
        vert_array->push_back( osg::Vec3(verts[i]) );
    }
    
    _drawable = new osg::Geometry;
    _drawable->setVertexArray( vert_array );

    osg::Vec4Array* colors = new osg::Vec4Array(1);
    (*colors)[0] = _color;
    _drawable->setColorArray( colors );
    _drawable->setColorBinding(osg::Geometry::BIND_OVERALL );

    _drawable->addPrimitiveSet( new osg::DrawArrays(GL_LINE_LOOP, 0, vert_array->size()) );
    

    MeshSubdivider ms(osg::Matrixd::inverse( _transform->getMatrix() ), _transform->getMatrix() );        
    ms.run(*_drawable.get(), osg::DegreesToRadians(2.0));            

    _geode->addDrawable( _drawable.get() );
}

float
RangeRingPrimitive::getLineWidth() const
{
    return _lineWidth;
}

void
RangeRingPrimitive::setLineWidth( float lineWidth )
{
    if (_lineWidth != lineWidth)
    {
        _lineWidth = lineWidth;
        osg::LineWidth* lw = dynamic_cast<osg::LineWidth*>(getOrCreateStateSet()->getAttribute(osg::StateAttribute::LINEWIDTH));
        if (lw)
        {
            lw->setWidth(_lineWidth );
        }
    }
}



void
RangeRingPrimitive::setProperty(const std::string& name, double value)
{
    if (name == PROP_LATITUDE)
    {
        setLocation(Location(value, _location.getLongitude(), _location.getAltitude()));
    }
    else if (name == PROP_LONGITUDE)
    {
        setLocation(Location(_location.getLatitude(), value, _location.getAltitude()));
    }
    else if (name == PROP_ALTITUDE)
    {
        setLocation(Location(_location.getLatitude(), _location.getLongitude(), value));
    }
    else if (name == PROP_LINEWIDTH) 
    {
        setLineWidth( value );
    }
    else if (name == "range") {
        setRange( value );
    }    
    else {
        Primitive::setProperty( name, value );
    }
}

void
RangeRingPrimitive::setProperty(const std::string& name, const osg::Vec3d& value)
{
    if (name == "fillcolor")
    {
        setColor(osg::Vec4(value.x(), value.y(), value.z(), 1.0f));
    }
	else if (name == PROP_COORD)
	{
		setLocation(Location(value.y(), value.x(), _location.getAltitude()));
	}
    else {
        Primitive::setProperty(name, value);
    }
}

void
RangeRingPrimitive::setProperty(const std::string& name, int value)
{
    if (name == "fillcolor")
    {
        setColor(getColorFromDecimal( value ) );
    }    
    else
    {
        Primitive::setProperty(name, value);
    }    
}