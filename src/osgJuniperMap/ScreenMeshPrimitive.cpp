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

#include <osgJuniperMap/ScreenMeshPrimitive>

#include <osgJuniperMap/Registry>
#include <osg/Texture2D>

using namespace osgJuniper::Map;

REGISTER_SIMPLE_PRIMITIVE_FACTORY(ScreenMeshPrimitive)
REGISTER_SIMPLE_PRIMITIVE_FACTORY_TYPE(ScreenMeshPrimitive, ScreenMesh)

/***************************************************************************/
ScreenMeshPrimitive::ScreenMeshPrimitive(PrimitiveId id):
Primitive( id ),
_mode( GL_TRIANGLES ),
_color( 1.0f, 1.0f, 1.0f, 1.0f )
{     
}

ScreenMeshPrimitive::~ScreenMeshPrimitive()
{
}

const std::vector< osg::Vec4 >&
ScreenMeshPrimitive::getColors() const
{
    return _colors;
}

void
ScreenMeshPrimitive::setColors( const std::vector< osg::Vec4 > &colors )
{
    _colors = colors;
    init();
}

const std::vector< osg::Vec3 >&
ScreenMeshPrimitive::getVerts() const
{
    return _verts;
}

void
ScreenMeshPrimitive::setVerts( const std::vector< osg::Vec3 > &verts )
{
    _verts = verts;
    init();
}

GLenum
ScreenMeshPrimitive::getMode() const
{
    return _mode;
}

void
ScreenMeshPrimitive::setMode(GLenum mode)
{
    if (_mode != mode)
    {
        _mode = mode;
        if (_primitiveSet.valid())
        {
            _primitiveSet->setMode( _mode );
            _geometry->dirtyDisplayList();
        }
    }
}

const std::vector< int >&
ScreenMeshPrimitive::getIndices() const
{
    return _indices;
}

void
ScreenMeshPrimitive::setIndices( const std::vector< int > &indices)
{
    _indices = indices;
    init();
}

const osg::Vec4f&
ScreenMeshPrimitive::getColor() const
{
    return _color;
}


void
ScreenMeshPrimitive::setColor(const osg::Vec4f& color)
{
    if (_color != color)
    {
        _color = color;
        init();
    }
}

osg::Image*
ScreenMeshPrimitive::getImage() const
{
    return _image.get();
}

void
ScreenMeshPrimitive::setImage( osg::Image* image )
{
    if (_image != image)
    {
        _image = image;
        init();
    }
}

const std::vector< osg::Vec2 >&
ScreenMeshPrimitive::getTexCoords() const
{
    return _texCoords;
}

void
ScreenMeshPrimitive::setTexCoords( const std::vector< osg::Vec2 > &texCoords)
{
    _texCoords = texCoords;
    init();
}

void
ScreenMeshPrimitive::setTexCoords( const std::vector< osg::Vec3 > &texCoords )
{
    std::vector< osg::Vec2 > coords;
    coords.reserve( texCoords.size() );
    for (unsigned int i = 0; i < texCoords.size(); i++)
    {
        coords.push_back( osg::Vec2f(texCoords[i].x(), texCoords[i].y()));
    }
    setTexCoords( coords );
}

void
ScreenMeshPrimitive::init()
{    
    removeChildren( 0, this->getNumChildren());
    if (_verts.size() > 0)
    {
        _geode = new osg::Geode;

        //Create a new Geometry object
        osg::Vec3Array* verts = new osg::Vec3Array(_verts.begin(), _verts.end());        
        osg::Vec4Array* colors;

        _geometry = new osg::Geometry;
        _geometry->setVertexArray( verts );


        if (_colors.size() == _verts.size() && !_colors.empty())
        {
            //Use the specified color values
            colors = new osg::Vec4Array(_colors.begin(), _colors.end());        
            _geometry->setColorBinding( osg::Geometry::BIND_PER_VERTEX);
        }
        else
        {
            //Use the overall color value
            colors = new osg::Vec4Array(1);
            (*colors)[0] = _color;
            _geometry->setColorBinding( osg::Geometry::BIND_OVERALL);
        }
        _geometry->setColorArray( colors );

        //Texture coords
        if (_texCoords.size() == _verts.size() && !_texCoords.empty() && _image.valid())
        {
            osg::Texture2D* tex = new osg::Texture2D( _image.get() );
            tex->setResizeNonPowerOfTwoHint(false);
            tex->setFilter( osg::Texture::MIN_FILTER, osg::Texture::NEAREST );
            tex->setFilter( osg::Texture::MAG_FILTER, osg::Texture::LINEAR );
            _geometry->getOrCreateStateSet()->setTextureAttributeAndModes( 0, tex, osg::StateAttribute::ON );

            osg::Vec2Array* texCoords = new osg::Vec2Array(_texCoords.begin(), _texCoords.end());
            _geometry->setTexCoordArray( 0, texCoords );

        }

        if (!_indices.empty())
        {
            osg::DrawElementsUInt *drawElements = new osg::DrawElementsUInt(_mode);            
            drawElements->reserve( _indices.size() );
            for (unsigned int i = 0; i < _indices.size(); ++i)
            {
                drawElements->push_back( _indices[i] );
            }
            _primitiveSet = drawElements;
        }
        else
        {
            //No indicies provided, just assume we can draw the primitive type with drawarrays
            _primitiveSet =  new osg::DrawArrays(_mode, 0, _verts.size() );
        }
        _geometry->addPrimitiveSet( _primitiveSet.get() );

        _geode->addDrawable( _geometry );

        _geode->getOrCreateStateSet()->setMode( GL_BLEND, osg::StateAttribute::ON);
        addChild( _geode.get() );
    }
}


void
ScreenMeshPrimitive::setProperty(const std::string& name, int    value)
{
    if (name == "mode")
    {
        setMode( (GLenum) value );
    }
    else if (name == "color")
    {
        setColor(Primitive::getColorFromDecimal(value));
    }
    else if (name == PROP_IMAGEOBJECT)
    {
        setImage( osgJuniper::Map::Registry::instance()->getImageForObject( value ) );
    }
    else
    {
        Primitive::setProperty( name, value );
    }
}

void
ScreenMeshPrimitive::setProperty(const std::string& name, const osg::Vec3d& value)
{
    if (name == "color")
    {
        setColor(osg::Vec4f(value.x(), value.y(), value.z(), 1.0) );
    }
    else
    {
        Primitive::setProperty( name, value );
    }
}

void ScreenMeshPrimitive::setProperty(const std::string& name, const std::vector< osg::Vec3 >& value)
{
    if (name == "colors")
    {
        std::vector< osg::Vec4 > colors;
        colors.reserve( value.size() );
        //Convert the colors to vec4
        for (unsigned int i = 0; i < value.size(); ++i)
        {
            colors.push_back( osg::Vec4(value[i].x(), value[i].y(), value[i].z(), 1.0f) );
        }
        setColors( colors );
    }
    else if (name == "verts")
    {
        setVerts( value );
    }
    else if (name == "texcoords")
    {        
        setTexCoords( value );
    }
    else
    {
        Primitive::setProperty( name, value );
    }
}

void
ScreenMeshPrimitive::setProperty(const std::string& name, const std::vector< int >& value)
{
    if (name == "indices")
    {
        setIndices( value );
    }
    else
    {
        Primitive::setProperty( name, value );
    }
}