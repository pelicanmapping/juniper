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

#include <osgJuniperMap/ScreenBitmapPrimitive>
#include <osgJuniperMap/Registry>

#include <osg/Geometry>
#include <osg/Geode>
#include <osg/ClusterCullingCallback>
#include <osg/Texture2D>
#include <osg/ImageStream>

#include <osgDB/ReadFile>

#include <osg/io_utils>


using namespace osgJuniper::Map;

REGISTER_SIMPLE_PRIMITIVE_FACTORY(ScreenBitmapPrimitive)
REGISTER_SIMPLE_PRIMITIVE_FACTORY_TYPE(ScreenBitmapPrimitive, ScreenBitmap)

/***************************************************************************/
ScreenBitmapPrimitive::ScreenBitmapPrimitive(PrimitiveId id):
Primitive(id),
_width(0),
_height(0),
_rotation(0.0f),
_alpha(0.0f),
_fgColor(1,1,1,1),
_bgColor(1,1,1,1),
_position(0,0)
{
    this->getOrCreateStateSet()->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);
    _transform  = new osg::MatrixTransform;
    addChild( _transform );
}



const osg::Vec2&
ScreenBitmapPrimitive::getPosition() const
{
    return _position;
}

void
ScreenBitmapPrimitive::setPosition( const osg::Vec2& position )
{
    if (_position != position) {
        _position = position;
        updateTransform();
    }
}

osg::Image*
ScreenBitmapPrimitive::getImage() const
{
    return _image.get();
}

void
ScreenBitmapPrimitive::setImage( osg::Image* image )
{
    if (_image != image)
    {
        _image = image;
        init();
    }
}


const std::string&
ScreenBitmapPrimitive::getFilename() const
{
    return _filename;
}

void
ScreenBitmapPrimitive::setFilename(const std::string& filename)
{
    if (_filename != filename)
    {
        _filename = filename;
        //Null out the image so it will get reloaded
        _image = NULL;
        init();
    }
}

void
ScreenBitmapPrimitive::setSize(float w, float h)
{
    if (w != _width || h != _height)
    {
        _width  = w;
        _height = h;
        //init();
        updateTransform();
    }
}

float
ScreenBitmapPrimitive::getWidth() const
{
    return _width;
}


float
ScreenBitmapPrimitive::getHeight() const
{
    return _height;
}

float
ScreenBitmapPrimitive::getRotation() const
{
    return _rotation;
}

void
ScreenBitmapPrimitive::setRotation( float rotation)
{
    if (_rotation != rotation)
    {
        _rotation = rotation;
        updateTransform();
    }
}

void
ScreenBitmapPrimitive::setForegroundColor( const osg::Vec4& color )
{
    if (_fgColor != color)
    {
        _fgColor = color;
		init();
    }
}

const osg::Vec4&
ScreenBitmapPrimitive::getForegroundColor() const
{
    return _fgColor;
}

void
ScreenBitmapPrimitive::setBackgroundColor( const osg::Vec4& color )
{
    if (_bgColor != color)
    {
        _bgColor = color;
        init();
    }
}

const osg::Vec4&
ScreenBitmapPrimitive::getBackgroundColor() const
{
    return _bgColor;
}

static osg::Geode* createIcon(osg::Image* image, osg::Vec4Array* fgColor, osg::Vec4Array* bgColor)
{
    if (image)
    {
        float y = 1.0f;
        float x = 1.0f;

        // set up the texture.
        osg::Texture2D* texture = new osg::Texture2D;
        texture->setImage(image);
        texture->setResizeNonPowerOfTwoHint( false );
        texture->setFilter(osg::Texture::MIN_FILTER,osg::Texture::LINEAR);
        // set up the drawstate.
        osg::StateSet* dstate = new osg::StateSet;
        dstate->setMode(GL_CULL_FACE,osg::StateAttribute::OFF);
        dstate->setMode(GL_LIGHTING,osg::StateAttribute::OFF);
        dstate->setTextureAttributeAndModes(0, texture,osg::StateAttribute::ON);

        // set up the geoset.
        osg::Geometry* geom = new osg::Geometry;
        geom->setStateSet(dstate);

        osg::Vec3Array* coords = new osg::Vec3Array(4);
        (*coords)[0].set(0,1,0.0f);
        (*coords)[1].set(0,0,0.0f);
        (*coords)[2].set(1,0,0.0f);
        (*coords)[3].set(1,1,0.0f);
        geom->setVertexArray(coords);

        bool flip = image->getOrigin()==osg::Image::TOP_LEFT;

        osg::Vec2Array* tcoords = new osg::Vec2Array(4);
        (*tcoords)[0].set(0.0f,flip ? 0.0 : 1.0f);
        (*tcoords)[1].set(0.0f,flip ? 1.0 : 0.0f);
        (*tcoords)[2].set(1.0f,flip ? 1.0 : 0.0f);
        (*tcoords)[3].set(1.0f,flip ? 0.0 : 1.0f);
        geom->setTexCoordArray(0,tcoords);

        geom->setColorArray(fgColor);
        geom->setColorBinding(osg::Geometry::BIND_OVERALL);
		geom->setSecondaryColorArray(bgColor);
		geom->setSecondaryColorBinding(osg::Geometry::BIND_OVERALL);

        geom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::QUADS,0,4));

        // set up the geode.
        osg::Geode* geode = new osg::Geode;
        geode->addDrawable(geom);

        return geode;
    }
    else
    {
        return NULL;
    }
}

void
ScreenBitmapPrimitive::init()
{    
    //Remove all the current children
    _transform->removeChildren(0, _transform->getNumChildren());

    updateTransform();

    getOrCreateStateSet()->setMode(GL_DEPTH_TEST, osg::StateAttribute::OFF);
    getOrCreateStateSet()->setMode( GL_BLEND, osg::StateAttribute::ON );
    if (!_image.valid() && !_filename.empty())
    {
        //Cache images since we will probably be reusing the images
        osg::ref_ptr<osgDB::ReaderWriter::Options> opt = new osgDB::ReaderWriter::Options;
        opt->setObjectCacheHint(osgDB::ReaderWriter::Options::CACHE_IMAGES);
        _image = osgDB::readImageFile( _filename, opt.get());
        osg::ImageStream* is = dynamic_cast<osg::ImageStream*>(_image.get());
        if (is)
        {
            is->play();
        }
    }

    if (_image.valid())
    {
		osg::Vec4Array* fgColor = new osg::Vec4Array(1);
		(*fgColor)[0].set(_fgColor._v[0], _fgColor._v[1], _fgColor._v[2], _fgColor._v[3]);
		osg::Vec4Array* bgColor = new osg::Vec4Array(1);
		(*bgColor)[0].set(_bgColor._v[0], _bgColor._v[1], _bgColor._v[2], _bgColor._v[3]);
		osg::Geode* geode = createIcon(_image.get(), fgColor, bgColor);
        _transform->addChild(geode);
    }
}

void
ScreenBitmapPrimitive::updateTransform()
{       
    double w = _width;
    double h = _height;
    if ( (w <= 0 || h <= 0) && _image.valid())
    {
        w = _image->s();
        h = _image->t();
    }
    _transform->setMatrix( 
                           osg::Matrixd::translate(.5, 0.5, 0) * 
                           osg::Matrixd::rotate(osg::DegreesToRadians(_rotation), osg::Vec3d(0,0,-1)) *
                           osg::Matrixd::translate(-.5, -0.5, 0) * 
                           osg::Matrixd::scale(w, h, 1) *
                           osg::Matrixd::translate(_position.x(), _position.y(), 0));        

    /*
    _transform->setMatrix( osg::Matrixd::rotate(osg::DegreesToRadians(_rotation), osg::Vec3d(0,0,-1)) *
                           osg::Matrixd::scale(_width, _height, 1) *
                           osg::Matrixd::translate(_position.x(), _position.y(), 0));    
                           */
}

void
ScreenBitmapPrimitive::setProperty(const std::string& name, double value)
{
    if (name == "x")
    {
        setPosition(osg::Vec2(value, _position.y()));
    }
    else if (name == "y")
    {
        setPosition(osg::Vec2(_position.x(), value));
    }
    else if (name == PROP_ROTATION)
    {
        setRotation(value);
    }
    else if (name == PROP_SIZE)
    {
        setSize(value, value);
    }
    else if (name == PROP_WIDTH)
    {
        setSize(value, _height);
    }
    else if (name == PROP_HEIGHT)
    {
        setSize(_width, value);
    }
    else
    {
        Primitive::setProperty(name, value);
    }
}

void
ScreenBitmapPrimitive::setProperty(const std::string& name, const osg::Vec3& value)
{
    if (name == PROP_SIZE)
    {
        setSize(value.x(), value.y());
    }
    else if (name == PROP_FGCOLOR)
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
ScreenBitmapPrimitive::setProperty(const std::string& name, const std::string& value)
{
    if (name == PROP_FILEPATH)
    {
        setFilename( value );
    }
    else
    {
        Primitive::setProperty(name, value);
    }
}

void
ScreenBitmapPrimitive::setProperty(const std::string& name, int value)
{
    if (name == PROP_FGCOLOR)
    {
        setForegroundColor(Primitive::getColorFromDecimal(value));
    }
    else if (name == PROP_BGCOLOR)
    {
        setBackgroundColor(Primitive::getColorFromDecimal(value));
    }
    else if (name == PROP_SIZE)
    {
        setSize((float)value, (float)value);
    }
    else if (name == PROP_WIDTH)
    {
        setSize((float)value, _height);
    }
    else if (name == PROP_HEIGHT)
    {
        setSize(_width, (float)value);
    }
    else if (name == PROP_IMAGEOBJECT)
    {
        setImage( osgJuniper::Map::Registry::instance()->getImageForObject( value ) );
    }
    else
    {
        Primitive::setProperty(name, value);
    }
}