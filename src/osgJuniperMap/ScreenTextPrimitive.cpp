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

#include <osgJuniperMap/ScreenTextPrimitive>

#include <osg/Geode>

#include <osgJuniperMap/Registry>

#include <sstream>

using namespace osgJuniper::Map;

REGISTER_SIMPLE_PRIMITIVE_FACTORY(ScreenTextPrimitive)
REGISTER_SIMPLE_PRIMITIVE_FACTORY_TYPE(ScreenTextPrimitive, ScreenText)


/***************************************************************************/
ScreenTextPrimitive::ScreenTextPrimitive(PrimitiveId id):
Primitive(id),
_rotation(0.0f),
_text(""),
_font("fonts/arial.ttf"),
_fgColor(1,1,1,1),
_bgColor(0,0,0,1),
_alpha(1.0f),
_size(50.0f)
{        
    this->getOrCreateStateSet()->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);
    //Create the drawable
    _drawable = new osgText::Text;

    //Load the font.
    osgText::Font* font = osgText::readFontFile(_font);
    _drawable->setFont(font);
    _drawable->setColor( _fgColor );
    _drawable->setBackdropColor( _bgColor );
    _drawable->setBackdropType(osgText::Text::OUTLINE);
    
    _drawable->setCharacterSize(_size);    
    _drawable->setDataVariance(osg::Object::DYNAMIC );    

    _geode = new osg::Geode;    
    _geode->addDrawable( _drawable.get() );    
    getOrCreateStateSet()->setMode(GL_DEPTH_TEST, osg::StateAttribute::OFF);

    _transform = new osg::MatrixTransform;
    _transform->addChild( _geode.get() );

    addChild( _transform.get() );
}

ScreenTextPrimitive::~ScreenTextPrimitive()
{
}

const osg::Vec2&
ScreenTextPrimitive::getPosition() const
{
    return _position;
}

void
ScreenTextPrimitive::setPosition( const osg::Vec2& position )
{
    if (_position != position) {
        _position = position;
        updateTransform();
    }
}


const std::string&
ScreenTextPrimitive::getText() const
{
    return _text;
}

void
ScreenTextPrimitive::setText( const std::string& text )
{
    if (_text != text)
    {
        _text = text;
        _drawable->setText( _text );
    }
}

void
ScreenTextPrimitive::setFont( const std::string& font)
{
    if (_font != font)
    {
        _font = font;
        osgText::Font* osgFont = osgText::readFontFile(_font);
        if (osgFont)
        {
            _drawable->setFont(osgFont);
        }
        else
        {
            osg::notify(osg::WARN) << "Couldn't load font file " << _font << std::endl;
        }
    }
}

const std::string&
ScreenTextPrimitive::getFont() const
{
    return _font;
}

void
ScreenTextPrimitive::setForegroundColor( const osg::Vec4& color )
{
    if (_fgColor != color)
    {
        _fgColor = color;
        _drawable->setColor( _fgColor );
    }
}
const osg::Vec4&
ScreenTextPrimitive::getForegroundColor() const
{
    return _fgColor;
}

void
ScreenTextPrimitive::setBackgroundColor( const osg::Vec4& color )
{
    if (_bgColor != color)
    {
        _bgColor = color;
        _drawable->setBackdropColor( _bgColor );
    }
}

const osg::Vec4&
ScreenTextPrimitive::getBackgroundColor() const
{
    return _bgColor;
}

void
ScreenTextPrimitive::setSize(float size)
{
    if (_size != size)
    {
        _size = size;
        _drawable->setCharacterSize( size );
    }
}

float
ScreenTextPrimitive::getSize() const
{
    return _size;
}

float
ScreenTextPrimitive::getRotation() const
{
    return _rotation;
}

void
ScreenTextPrimitive::setRotation(float rotation)
{
    if (_rotation != rotation)
    {
        _rotation = rotation;
        _drawable->setRotation( osg::Quat(osg::DegreesToRadians( _rotation ), osg::Vec3d(0,0,-1)));
    }
}

float
ScreenTextPrimitive::getAlpha() const
{
    return _alpha;
}

void
ScreenTextPrimitive::setAlpha(float alpha)
{
    if (_alpha != alpha)
    {
        _alpha = alpha;        
        //TODO:  Update rendering
    }
}

void
ScreenTextPrimitive::updateTransform()
{
    _transform->setMatrix(osg::Matrixd::translate(_position.x(), _position.y(), 0));    
}

void
ScreenTextPrimitive::setProperty(const std::string& name, const std::string& value)
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
ScreenTextPrimitive::setProperty(const std::string& name, const osg::Vec3d& value)
{
    if (name == PROP_FGCOLOR)
    {
        setForegroundColor(osg::Vec4(value.x(), value.y(), value.z(), 1.0));
    }
    else if (name == PROP_BGCOLOR)
    {
        setBackgroundColor(osg::Vec4(value.x(), value.y(), value.z(), 1.0));
    }
    else
    {
        Primitive::setProperty(name, value);
    }
}

void
ScreenTextPrimitive::setProperty(const std::string& name, int    value)
{
    if (name == PROP_FGCOLOR)
    {
        setForegroundColor(Primitive::getColorFromDecimal(value));
    }
    else if (name == PROP_BGCOLOR)
    {
        setBackgroundColor(Primitive::getColorFromDecimal(value));
    }
    else
    {
        Primitive::setProperty(name, value);
    }
}

void
ScreenTextPrimitive::setProperty(const std::string& name, double value)
{
    if (name == "x")
    {
        setPosition(osg::Vec2(value, _position.y()));        
    }
    else if (name == "y")
    {
        setPosition(osg::Vec2(_position.x(), value));        
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