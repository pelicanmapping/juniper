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

#include <osgJuniperMap/DeclutterTextPrimitive>

#include <osg/Geode>
#include <osg/PolygonOffset>
#include <osg/Depth>

#include <osgEarthAnnotation/Decluttering>
#include <osgEarthAnnotation/AnnotationUtils>
#include <osg/io_utils>

#include <osgJuniperMap/Registry>

#include <sstream>

using namespace osgJuniper::Map;

using namespace osgEarth::Annotation;
using namespace osgEarth::Symbology;

REGISTER_SIMPLE_PRIMITIVE_FACTORY(DeclutterTextPrimitive)
REGISTER_SIMPLE_PRIMITIVE_FACTORY_TYPE(DeclutterTextPrimitive, DeclutterText)


/***************************************************************************/
DeclutterTextPrimitive::DeclutterTextPrimitive(PrimitiveId id):
Primitive(id),
_rotation(0.0f),
_text(""),
_font("fonts/arial.ttf"),
_fgColor(1,1,1,1),
_outlineColor(0,0,0,1),
_size(16.0f),
_altitudeMode(AltitudeMode::ABSOLUTE),
_declutter( false ),
_offset(0,0)
{           
    getOrCreateStateSet()->setMode( GL_LIGHTING, osg::StateAttribute::OFF | osg::StateAttribute::OVERRIDE );
    //osgEarth::Annotation::Decluttering::setEnabled( getOrCreateStateSet(), _declutter );
}

DeclutterTextPrimitive::~DeclutterTextPrimitive()
{
}

osgEarth::Annotation::LabelNode*
DeclutterTextPrimitive::getLabel()
{
    if (!_label.valid())
    {
        init();
    }
    return _label.get();
}


void
DeclutterTextPrimitive::init()
{
    if (_label.valid())
    {
        removeChild( _label.get() );
    }

    if (_context.valid())
    {
        OSG_NOTICE << "Initializing label" << std::endl;
        Style labelStyle;
        labelStyle.getOrCreate<TextSymbol>()->alignment() = TextSymbol::ALIGN_CENTER_CENTER;
        labelStyle.getOrCreate<TextSymbol>()->fill()->color() = _fgColor;
        labelStyle.getOrCreate<TextSymbol>()->halo()->color() = _outlineColor;
        labelStyle.getOrCreate<TextSymbol>()->size() = _size;
        labelStyle.getOrCreate<TextSymbol>()->font() = _font;
        labelStyle.getOrCreate<TextSymbol>()->pixelOffset() = _offset;        
        labelStyle.getOrCreate<AltitudeSymbol>()->clamping() = AltitudeSymbol::CLAMP_TO_TERRAIN;

        /*
        if (_altitudeMode == AltitudeMode::RELATIVE_TO_TERRAIN)
        {
            labelStyle.getOrCreate<AltitudeSymbol>()->clamping() = AltitudeSymbol::CLAMP_TO_TERRAIN;
        }
        */

        _label = new LabelNode(_context->getMapNode(),
            GeoPoint(SpatialReference::create( "epsg:4326" ),
            _location.getLongitude(), _location.getLatitude(), _location.getAltitude()),
            _text, labelStyle);
        _label->setDynamic(true);
        addChild( _label.get() );
    }
}


const Location&
DeclutterTextPrimitive::getLocation() const
{
    return _location;
}

void
DeclutterTextPrimitive::setLocation(const Location& location)
{
    if (_location != location)
    {
        _location = location;
        LabelNode* label = getLabel();
        if (label)
        {
            OE_NOTICE << "Setting location " << _location.asVec3d() << std::endl;
            label->setPosition( _location.asVec3d() );
        }
    }
}

bool
DeclutterTextPrimitive::getDeclutter() const
{
    return _declutter;
}

void
DeclutterTextPrimitive::setDeclutter( bool declutter)
{
    if (_declutter != declutter)
    {
        _declutter = declutter;
        osgEarth::Annotation::Decluttering::setEnabled( getOrCreateStateSet(), _declutter );
    }
}

short
DeclutterTextPrimitive::getOffsetX() const
{
    return _offset.x();
}

short
DeclutterTextPrimitive::getOffsetY() const
{
    return _offset.y();
}

void
DeclutterTextPrimitive::setOffset( short x, short y )
{
    if (_offset.x() != x || _offset.y() != y)
    {
        _offset.set( x, y );
        init();
    }
}

AltitudeModeEnum
DeclutterTextPrimitive::getAltitudeMode() const
{
    return _altitudeMode;
}

void
DeclutterTextPrimitive::setAltitudeMode(AltitudeModeEnum altitudeMode)
{
    if (_altitudeMode != altitudeMode)
    {
        _altitudeMode = altitudeMode;
        init();
    }
}

void
DeclutterTextPrimitive::setMapContext(MapContext* context)
{
    Primitive::setMapContext(context);    
    init();
}

const std::string&
DeclutterTextPrimitive::getText() const
{
    return _text;
}

void
DeclutterTextPrimitive::setText( const std::string& text )
{
    if (_text != text)
    {
        _text = text;
        LabelNode* label = getLabel();
        if (label)
        {
            label->setText( _text );
        }
    }
}

void
DeclutterTextPrimitive::setFont( const std::string& font)
{
    if (_font != font)
    {
        _font = font;
        init();
    }
}

const std::string&
DeclutterTextPrimitive::getFont() const
{
    return _font;
}

void
DeclutterTextPrimitive::setForegroundColor( const osg::Vec4& color )
{
    if (_fgColor != color)
    {
        _fgColor = color;
        init();
    }
}
const osg::Vec4&
DeclutterTextPrimitive::getForegroundColor() const
{
    return _fgColor;
}

const osg::Vec4&
DeclutterTextPrimitive::getOutlineColor() const
{
    return _outlineColor;
}


void
DeclutterTextPrimitive::setOutlineColor( const osg::Vec4& color )
{
    if (_outlineColor != color)
    {
        _outlineColor = color;
        init();
    }
}

void
DeclutterTextPrimitive::setSize(float size)
{
    if (_size != size)
    {
        _size = size;
        init();
    }
}

float
DeclutterTextPrimitive::getSize() const
{
    return _size;
}

float
DeclutterTextPrimitive::getRotation() const
{
    return _rotation;
}

void
DeclutterTextPrimitive::setRotation(float rotation)
{
    if (_rotation != rotation)
    {
        _rotation = rotation;
        //getLabel()->setRotation( _rotation );
    }
}

void
DeclutterTextPrimitive::setProperty(const std::string& name, bool value)
{
    if (name == "declutter")
    {
        setDeclutter( value );
    }
    else
    {
        Primitive::setProperty(name, value);
    }
}

void
DeclutterTextPrimitive::setProperty(const std::string& name, const std::string& value)
{
    if (name == PROP_TEXT)
    {
        setText( value );
    }
    else if (name == PROP_FONT)
    {
        setFont( value );
    }
    else
    {
        Primitive::setProperty(name, value);
    }
}

void
DeclutterTextPrimitive::setProperty(const std::string& name, const osg::Vec3d& value)
{
    if (name == PROP_FGCOLOR)
    {
        setForegroundColor(osg::Vec4(value.x(), value.y(), value.z(), _fgColor.a()));
    }
    else if (name == "offset")
    {
        setOffset((short)value.x(), (short)value.y());
    }
    else
    {
        Primitive::setProperty(name, value);
    }
}

void
DeclutterTextPrimitive::setProperty(const std::string& name, int    value)
{
    if (name == PROP_FGCOLOR)
    {
        setForegroundColor(Primitive::getColorFromDecimal(value));
    }
    else if (name == PROP_OUTLINECOLOR)
    {
        setOutlineColor(Primitive::getColorFromDecimal(value));
    }
    else if (name == PROP_ALTITUDE_MODE)
    {
        setAltitudeMode( (AltitudeModeEnum) value );
    }
    else
    {
        Primitive::setProperty(name, value);
    }
}

void
DeclutterTextPrimitive::setProperty(const std::string& name, double value)
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
    else if (name == PROP_SIZE)
    {
        setSize(value);
    }
    else if  (name == PROP_ROTATION)
    {
        setRotation(value);
    }
    else
    {
        Primitive::setProperty(name, value);
    }
}


void
DeclutterTextPrimitive::setProperty(const std::string& name, const LocationList& locations)
{
	if (name.compare(PROP_POINTLIST) == 0)
    {
		setLocation(locations[0]);
	}
}