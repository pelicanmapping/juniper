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

#include <osgJuniperMap/BitmapPrimitive>
#include <osgJuniperMap/Registry>

#include <osg/Geometry>
#include <osg/Geode>
#include <osg/ClusterCullingCallback>
#include <osg/Texture2D>
#include <osg/ImageStream>

#include <osgDB/ReadFile>

#include <osg/CoordinateSystemNode>
#include <osgEarth/TerrainEngineNode>

#include <osg/io_utils>


using namespace osgJuniper::Map;

REGISTER_SIMPLE_PRIMITIVE_FACTORY(BitmapPrimitive)
REGISTER_SIMPLE_PRIMITIVE_FACTORY_TYPE(BitmapPrimitive, Bitmap)

/***************************************************************************/
BitmapPrimitive::BitmapPrimitive(PrimitiveId id):
Primitive(id),
_width(0),
_height(0),
_rotation(0.0f),
_alpha(0.0f),
_altitudeMode(AltitudeMode::ALTMODE_RELATIVE),
_fgColor(1,1,1,1),
_bgColor(1,1,1,1)
{
    this->getOrCreateStateSet()->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);
}

const Location&
BitmapPrimitive::getLocation() const
{
    return _location;
}

void
BitmapPrimitive::setLocation(const Location& location)
{
    if (_location != location)
    {
        _location = location;
        updateTransform();
    }
}

AltitudeMode
BitmapPrimitive::getAltitudeMode() const
{
    return _altitudeMode;
}

void
BitmapPrimitive::setAltitudeMode(AltitudeMode altitudeMode)
{
    if (_altitudeMode != altitudeMode)
    {
        _altitudeMode = altitudeMode;
        updateTransform();
    }
}

void
BitmapPrimitive::setMapContext(MapContext* context)
{
    Primitive::setMapContext(context);
    updateTransform();
}

void
BitmapPrimitive::setEnableClusterCulling(bool enableClusterCulling)
{
    if (_enableClusterCulling != enableClusterCulling)
    {
        _enableClusterCulling = enableClusterCulling;
        updateTransform();
    }
}

osg::Image*
BitmapPrimitive::getImage() const
{
    return _image.get();
}

void
BitmapPrimitive::setImage( osg::Image* image )
{
    if (_image != image)
    {
        _image = image;
        init();
    }
}


const std::string&
BitmapPrimitive::getFilename() const
{
    return _filename;
}

void
BitmapPrimitive::setFilename(const std::string& filename)
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
BitmapPrimitive::setSize(float w, float h)
{
    if (w != _width || h != _height)
    {
        _width  = w;
        _height = h;
        //init();
        updateRotation();
    }
}

float
BitmapPrimitive::getWidth() const
{
    return _width;
}


float
BitmapPrimitive::getHeight() const
{
    return _height;
}

float
BitmapPrimitive::getRotation() const
{
    return _rotation;
}

void
BitmapPrimitive::setRotation( float rotation)
{
    if (_rotation != rotation)
    {
        _rotation = rotation;
        updateRotation();
    }
}

void
BitmapPrimitive::setForegroundColor( const osg::Vec4& color )
{
    if (_fgColor != color)
    {
        _fgColor = color;
		init();
    }
}

const osg::Vec4&
BitmapPrimitive::getForegroundColor() const
{
    return _fgColor;
}

void
BitmapPrimitive::setBackgroundColor( const osg::Vec4& color )
{
    if (_bgColor != color)
    {
        _bgColor = color;
        init();
    }
}

const osg::Vec4&
BitmapPrimitive::getBackgroundColor() const
{
    return _bgColor;
}

osg::Geode* createIcon(osg::Image* image, osg::Vec4Array* fgColor, osg::Vec4Array* bgColor)
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
        (*coords)[0].set(-0.5,0.5,0.0f);
        (*coords)[1].set(-0.5,-0.5,0.0f);
        (*coords)[2].set(0.5,-0.5,0.0f);
        (*coords)[3].set(0.5,0.5,0.0f);
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
BitmapPrimitive::init()
{
    //Initialize the autotransform
    if (!_autoTransform.valid())
    {
        _autoTransform = new osg::AutoTransform;
        _autoTransform->setAutoRotateMode(osg::AutoTransform::ROTATE_TO_SCREEN);
        _autoTransform->setAutoScaleToScreen(true);         
        addChild(_autoTransform.get());
		
    }

    updateTransform();

    if (!_rotationTransform.valid())
    {
        _rotationTransform = new osg::MatrixTransform;
        _autoTransform->addChild( _rotationTransform.get() );
    }
    

    //Remove all the current children of the autotransform
    _rotationTransform->removeChildren(0, _rotationTransform->getNumChildren());


    //setMinimumScale(0);
    //setMaximumScale(500);
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
        _rotationTransform->addChild(geode);
    }

    updateRotation();
}

void
BitmapPrimitive::terrainChanged(const osgEarth::TileKey& tileKey, osg::Node* terrain)
{    
    if (tileKey.getExtent().contains( _location.getLongitude(), _location.getLatitude() ) )
    {
      updateTransform( terrain );
    }    
}

void
BitmapPrimitive::updateTransform(osg::Node* terrain)
{    
    if (!terrain && _context.valid())
    {
        terrain = _context->getMapNode()->getTerrainEngine();
    }

    if (_autoTransform)
    {
        osg::ref_ptr< const osg::EllipsoidModel > em;
        //clamped to ground
        //Compute the geocentric position
        osg::Vec3d position;
        if (_context.valid())
        {
			em = _context->getEllipsoid();

            //Get the location in geocentric coordinates
            _location.getGeocentric(em.get(), position);
            osg::Vec3 normal;
            if (_altitudeMode == AltitudeMode::ALTMODE_RELATIVE)
            {
                MapContext::getNodeIntersection(terrain, em.get(), position, position, normal, _location.getAltitude());
            }
        }
        else
        {
            em = Primitive::getWGS84Ellipsoid();
            _location.getGeocentric(em.get(), position);
        }

        _autoTransform->setPosition(position);
        //_autoTransform->setInitialBound( osg::BoundingSphere(position, 1000));
        //_autoTransform->dirtyBound();

        if (_enableClusterCulling)
        {
            //NOTE:  Must set the cull callback on the group and not the autotransform as getEyePoint will be in a funky coordinate system if you stick it on the autotransform.
            osg::Vec3d normal = em->computeLocalUpVector(position.x(), position.y(), position.z());
            _ccc = static_cast<osg::ClusterCullingCallback*>(getCullCallback());
            if (!_ccc)
            {
                _ccc = new osg::ClusterCullingCallback();
                setCullCallback( _ccc );
            }
            _ccc->set(position, normal, 0, -1);
        }
        else
        {
            setCullCallback( NULL );
        }
    }
}

void
BitmapPrimitive::updateRotation()
{
    if (_rotationTransform.valid() && _image.valid() )
    {
        float w = _width > 0 ? _width : _image->s();
        float h = _height > 0 ? _height : _image->t();
        _rotationTransform->setMatrix( osg::Matrixd::scale( w, h, 1.0 ) *
                                       osg::Matrixd::rotate(_rotation, osg::Vec3d(0,0,-1)));
    }
}


void
BitmapPrimitive::setProperty(const std::string& name, double value)
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
BitmapPrimitive::setProperty(const std::string& name, const osg::Vec3d& value)
{
	if (name == PROP_COORD)
	{
		setLocation(Location(value.y(), value.x(), _location.getAltitude()));
	}
	else
    {
        Primitive::setProperty(name, value);
    }
}

void
BitmapPrimitive::setProperty(const std::string& name, const osg::Vec3& value)
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
BitmapPrimitive::setProperty(const std::string& name, const std::string& value)
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
BitmapPrimitive::setProperty(const std::string& name, int value)
{
    if (name == PROP_FGCOLOR)
    {
        setForegroundColor(Primitive::getColorFromDecimal(value));
    }
    else if (name == PROP_BGCOLOR)
    {
        setBackgroundColor(Primitive::getColorFromDecimal(value));
    }
    else if (name == PROP_ALTITUDE_MODE)
    {
        setAltitudeMode( (AltitudeMode) value );
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

void
BitmapPrimitive::setProperty(const std::string& name, const LocationList& locations)
{
    if (name.compare(PROP_POINTLIST) == 0)
    {
		// Use the PROP_LATITUDE and PROP_LONGITUDE when available.
		// But if the PRIM_POINTLIST is of size 1, then that is a substitute
		if( locations.size() == 1 )
			setLocation(locations[0]);
	}
}