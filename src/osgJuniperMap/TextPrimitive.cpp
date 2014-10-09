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

#include <osgJuniperMap/TextPrimitive>

#include <osg/ClusterCullingCallback>
#include <osg/Geode>
#include <osg/PolygonOffset>
#include <osg/CoordinateSystemNode>
#include <osgEarth/TerrainEngineNode>

#include <osgJuniperMap/Registry>

#include <sstream>

using namespace osgJuniper::Map;

REGISTER_SIMPLE_PRIMITIVE_FACTORY(TextPrimitive)
REGISTER_SIMPLE_PRIMITIVE_FACTORY_TYPE(TextPrimitive, Text)

struct FixSmallFeatureCullingCallback : public osg::NodeCallback
{
    void operator()(osg::Node* node, osg::NodeVisitor* nv) {
        node->setCullingActive( true );
        traverse( node, nv );
        node->setCullCallback( 0 );
    }
};

// override osg Text to get at some of the internal properties
class LabelText : public osgText::Text
{
public:
    const osg::BoundingBox& getTextBB() const { return _textBB; }
    const osg::Matrix& getATMatrix(int contextID) const { return _autoTransformCache[contextID]._matrix; }
};

/***************************************************************************/
TextPrimitive::TextPrimitive(PrimitiveId id):
Primitive(id),
_rotation(0.0f),
_text(""),
_font("fonts/arial.ttf"),
_fgColor(1,1,1,1),
_bgColor(0,0,0,0),
_outlineColor(0,0,0,1),
_alpha(1.0f),
_size(50.0f),
_screenPosition(0.0, 0.0),
_altitudeMode(AltitudeMode::ALTMODE_RELATIVE)
{        
    this->getOrCreateStateSet()->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);
    //Create the drawable
    _drawable = new LabelText();//new osgText::Text;

    //Load the font.
    osgText::Font* font = osgText::readFontFile(_font);
    _drawable->setFont(font);
    _drawable->setColor( _fgColor );
    _drawable->setBackdropColor( _bgColor );
    _drawable->setBackdropType(osgText::Text::OUTLINE);

    _drawable->setAutoRotateToScreen(true);
    _drawable->setCharacterSize(_size);
    _drawable->setCharacterSizeMode(osgText::TextBase::SCREEN_COORDS);
    _drawable->setDataVariance(osg::Object::DYNAMIC );    
    _drawable->setAlignment(osgText::TextBase::CENTER_CENTER);    

    _geode = new osg::Geode;
    //Turn culling off initially.
    //_geode->setCullingActive( false );
    //Set a callback that will set culling active the first time the geode is culled.  This allows the text to be traversed at least once and a proper 
    //bound to be computed.
    //_geode->setCullCallback( new FixSmallFeatureCullingCallback() );
    _geode->addDrawable( _drawable.get() );    
    getOrCreateStateSet()->setMode(GL_DEPTH_TEST, osg::StateAttribute::OFF);

    _transform = new osg::MatrixTransform;
    _transform->addChild( _geode.get() );

    addChild( _transform.get() );
    initBackground();
}

TextPrimitive::~TextPrimitive()
{
}

void
TextPrimitive::initBackground() 
{
    if (_backgroundTransform.valid() ) _transform->removeChild( _backgroundTransform.get() );

    if (_bgColor.a() > 0)
    {

        _backgroundTransform = new osg::AutoTransform();
        _backgroundTransform->setAutoRotateMode( osg::AutoTransform::ROTATE_TO_SCREEN);
        _backgroundTransform->setAutoScaleToScreen( true );
        _backgroundTransform->setCullingActive( true );

        osg::BoundingBox box = static_cast<LabelText*>(_drawable.get())->getTextBB();        
        float x = 0.5 * (box.xMax()-box.xMin());
        float y = 0.5 * (box.yMax()-box.yMin());
        float z = 0.5 * (box.zMax()-box.zMin());

        //OSG_NOTICE << "bounding box " << x << ", " << y << ", " << z << std::endl;

        box.set( -x, -y, -z, x, y, z );

        osg::Geometry* geometry = new osg::Geometry;
        osg::Vec3Array* verts = new osg::Vec3Array();
        verts->reserve( 4 );    
        verts->push_back( osg::Vec3(box.xMin(), box.yMin(), 0) );
        verts->push_back( osg::Vec3(box.xMax(), box.yMin(), 0));
        verts->push_back( osg::Vec3(box.xMax(), box.yMax(), 0) );
        verts->push_back( osg::Vec3(box.xMin(), box.yMax(), 0));

        geometry->setVertexArray( verts );    

        osg::Vec4Array* colors = new osg::Vec4Array(1);
        (*colors)[0] = _bgColor;
        geometry->setColorArray( colors );
        geometry->setColorBinding( osg::Geometry::BIND_OVERALL );

        osg::Geode* geode = new osg::Geode;
        geode->addDrawable( geometry );
        _backgroundTransform->getOrCreateStateSet()->setMode( GL_LIGHTING, osg::StateAttribute::OFF);
        geometry->addPrimitiveSet( new osg::DrawArrays( GL_QUADS, 0, 4) );
        geometry->getOrCreateStateSet()->setMode(GL_BLEND, osg::StateAttribute::ON);

        _backgroundTransform->addChild( geode );       
        _backgroundTransform->getOrCreateStateSet()->setAttributeAndModes(new osg::PolygonOffset( 1, 1 ), osg::StateAttribute::ON );

        _transform->addChild( _backgroundTransform ); 
    }
}

const osg::Vec2&
TextPrimitive::getScreenPosition() const
{
    return _screenPosition;
}

void
TextPrimitive::setScreenPosition( const osg::Vec2& position )
{
    if (_screenPosition != position) {
        _screenPosition = position;
        updateTransform();
    }
}

const Location&
TextPrimitive::getLocation() const
{
    return _location;
}

void
TextPrimitive::setLocation(const Location& location)
{
    if (_location != location)
    {
        _location = location;
        updateTransform();
    }
}

AltitudeMode
TextPrimitive::getAltitudeMode() const
{
    return _altitudeMode;
}

void
TextPrimitive::setAltitudeMode(AltitudeMode altitudeMode)
{
    if (_altitudeMode != altitudeMode)
    {
        _altitudeMode = altitudeMode;
        updateTransform();
    }
}

void
TextPrimitive::setMapContext(MapContext* context)
{
    Primitive::setMapContext(context);
    updateTransform();
}

void
TextPrimitive::setEnableClusterCulling(bool enableClusterCulling)
{
    if (_enableClusterCulling != enableClusterCulling)
    {
        _enableClusterCulling = enableClusterCulling;
        updateTransform();
    }
}

const std::string&
TextPrimitive::getText() const
{
    return _text;
}

void
TextPrimitive::setText( const std::string& text )
{
    if (_text != text)
    {
        _text = text;
        _drawable->setText( _text );
		if( _primType == TYPE_GEOLOCATED )
			initBackground();
    }
}

void
TextPrimitive::setFont( const std::string& font)
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
TextPrimitive::getFont() const
{
    return _font;
}

void
TextPrimitive::setForegroundColor( const osg::Vec4& color )
{
    if (_fgColor != color)
    {
        _fgColor = color;
        _drawable->setColor( _fgColor );
    }
}
const osg::Vec4&
TextPrimitive::getForegroundColor() const
{
    return _fgColor;
}


const osg::Vec4&
TextPrimitive::getBackgroundColor() const
{
    return _bgColor;
}

void
TextPrimitive::setBackgroundColor( const osg::Vec4& color )
{
    if (_bgColor != color)
    {
        _bgColor = color;
		if( _primType == TYPE_GEOLOCATED )
			initBackground();
		else
			_drawable->setBackdropColor( _bgColor);
    }
}

const osg::Vec4&
TextPrimitive::getOutlineColor() const
{
    return _outlineColor;
}


void
TextPrimitive::setOutlineColor( const osg::Vec4& color )
{
    if (_outlineColor != color)
    {
        _outlineColor = color;
        _drawable->setBackdropColor( _outlineColor );
    }
}

void
TextPrimitive::setPrimitiveType(PrimitiveType type){
	_primType = type;
	if( type == TYPE_SCREEN ){
		//Recreate the drawable
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
		
		removeChild(_transform.get());
		_transform = new osg::MatrixTransform;
		_transform->addChild( _geode.get() );
		addChild( _transform.get() );

		if (_context.valid() && _terrainChangedCallback.valid())
		{
			_context->getMapNode()->getTerrain()->removeTerrainCallback( _terrainChangedCallback.get() );
			_terrainChangedCallback = 0;
			removeCullCallback(_ccc);
		}
	}
}

void
TextPrimitive::setSize(float size)
{
    if (_size != size)
    {
        _size = size;
        _drawable->setCharacterSize( size );
    }
}

float
TextPrimitive::getSize() const
{
    return _size;
}

float
TextPrimitive::getRotation() const
{
    return _rotation;
}

void
TextPrimitive::setRotation(float rotation)
{
    if (_rotation != rotation)
    {
        _rotation = rotation;
        _drawable->setRotation( osg::Quat(osg::DegreesToRadians( _rotation ), osg::Vec3d(0,0,-1)));
    }
}

float
TextPrimitive::getAlpha() const
{
    return _alpha;
}

void
TextPrimitive::setAlpha(float alpha)
{
    if (_alpha != alpha)
    {
        _alpha = alpha;        
        //TODO:  Update rendering
    }
}

void
TextPrimitive::terrainChanged(const osgEarth::TileKey& tileKey, osgEarth::TerrainEngineNode* terrain)
{    
    if (tileKey.getExtent().contains( _location.getLongitude(), _location.getLatitude() ) )
    {
      updateTransform( terrain );
    }    
}

void
TextPrimitive::updateTransform(osg::Node* terrain)
{
	if( _primType == TYPE_GEOLOCATED ){
		if (!terrain && _context.valid())
		{
			terrain = _context->getMapNode()->getTerrainEngine();
		}

		osg::ref_ptr< const osg::EllipsoidModel > em;
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

		//Compute the transformation matrix
		osg::Matrixd matrix = osg::Matrixd::translate( position );
		_transform->setMatrix( matrix );

		if (_enableClusterCulling)
		{
			//NOTE:  Must set the cull callback on the group and not the autotransform as getEyePoint will be in a funky coordinate system if you stick it on the autotransform.
			osg::Vec3d normal = em->computeLocalUpVector(position.x(), position.y(), position.z());
			_ccc = dynamic_cast<osg::ClusterCullingCallback*>(getCullCallback());
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
	else
		_transform->setMatrix(osg::Matrixd::translate(_screenPosition.x(), _screenPosition.y(), 0)); 
}

void
TextPrimitive::setProperty(const std::string& name, const std::string& value)
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
TextPrimitive::setProperty(const std::string& name, const osg::Vec3d& value)
{
    if (name == PROP_FGCOLOR)
    {
        setForegroundColor(osg::Vec4(value.x(), value.y(), value.z(), _fgColor.a()));
    }
    else if (name == PROP_BGCOLOR)
    {
        setBackgroundColor(osg::Vec4(value.x(), value.y(), value.z(), _bgColor.a()));
    }
	else if (name == PROP_COORD)
	{
		setLocation(Location(value.y(), value.x(), _location.getAltitude()));
	}
    else
    {
        Primitive::setProperty(name, value);
    }
}

void
TextPrimitive::setProperty(const std::string& name, int    value)
{
    if (name == PROP_FGCOLOR)
    {
        setForegroundColor(Primitive::getColorFromDecimal(value));
    }
    else if (name == PROP_BGCOLOR)
    {
        setBackgroundColor(Primitive::getColorFromDecimal(value));
    }
    else if (name == PROP_OUTLINECOLOR)
    {
        setOutlineColor(Primitive::getColorFromDecimal(value));
    }
    else if (name == PROP_ALTITUDE_MODE)
    {
        setAltitudeMode( (AltitudeMode) value );
    }
    else if (name == PROP_BGTRANSPARENCY)
    {
        setBackgroundColor(osg::Vec4(_bgColor.r(), _bgColor.g(), _bgColor.b(), osg::clampBetween(value/100.0, 0.0, 1.0)));
    }
    else
    {
        Primitive::setProperty(name, value);
    }
}

void
TextPrimitive::setProperty(const std::string& name, const std::vector< osg::Vec2 > value){
	if( value.size() > 0 )
		setScreenPosition(value[value.size() - 1]);
}

void
TextPrimitive::setProperty(const std::string& name, double value)
{
    if (name == PROP_LATITUDE)
    {
        setLocation(Location(value, _location.getLongitude(), _location.getAltitude()));
    }
    else if (name == PROP_LONGITUDE)
    {
		setLocation(Location(_location.getLatitude(), value, _location.getAltitude()));
    }
	else if (name == "x")
    {
        setScreenPosition(osg::Vec2(value, _screenPosition.y()));        
    }
    else if (name == "y")
    {
        setScreenPosition(osg::Vec2(_screenPosition.x(), value));
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
    else if (name == PROP_BGTRANSPARENCY)
    {
        setBackgroundColor(osg::Vec4(_bgColor.r(), _bgColor.g(), _bgColor.b(), osg::clampBetween(value/100.0, 0.0, 1.0)));
    }
    else
    {
        Primitive::setProperty(name, value);
    }
}


void
TextPrimitive::setProperty(const std::string& name, const LocationList& locations)
{
	if (name.compare(PROP_POINTLIST) == 0 && locations.size() > 0)
    {
		setLocation(locations[locations.size() - 1]);
	}
}