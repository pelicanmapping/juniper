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

#include <osgJuniperMap/EllipsoidPrimitive>
#include <osgJuniperMap/Registry>
#include <osg/io_utils>
#include <sstream>
#include <osg/ShapeDrawable>
#include <osg/PolygonMode>

#include <osg/CoordinateSystemNode>
#include <osgEarth/TerrainEngineNode>

using namespace osgJuniper::Map;

REGISTER_SIMPLE_PRIMITIVE_FACTORY(EllipsoidPrimitive)
REGISTER_SIMPLE_PRIMITIVE_FACTORY_TYPE(EllipsoidPrimitive, Ellipsoid)
REGISTER_SIMPLE_PRIMITIVE_FACTORY_TYPE(EllipsoidPrimitive, Ellipse)


/***************************************************************************/
EllipsoidPrimitive::EllipsoidPrimitive(PrimitiveId id):
Primitive(id),
_altitudeMode(AltitudeMode::ALTMODE_RELATIVE),
_fillColor(1,1,1,1),
_size(10,10,10),
_showFill(true),
_showOutline(true)
{
    //Make the transform
    _transform = new osg::MatrixTransform;
    addChild( _transform.get() );

    //Add the geodes to the scene graph
    _fillGeode = new osg::Geode;
    _outlineGeode = new osg::Geode;
    _transform->addChild( _fillGeode );
    _transform->addChild( _outlineGeode );

    osg::Sphere *sphere = new osg::Sphere(osg::Vec3(0,0,0), 1.0f);

    _fillDrawable = new osg::ShapeDrawable(sphere);
    static_cast<osg::ShapeDrawable*>(_fillDrawable.get())->setColor( _fillColor );

    _fillGeode->addDrawable( _fillDrawable.get() );
    _fillGeode->getOrCreateStateSet()->setMode(GL_NORMALIZE, osg::StateAttribute::ON);

    _outlineDrawable = new osg::ShapeDrawable(sphere);
    static_cast<osg::ShapeDrawable*>(_outlineDrawable.get())->setColor( _outlineColor );
    osg::PolygonMode* polymode = new osg::PolygonMode;
    polymode->setMode(osg::PolygonMode::FRONT_AND_BACK,osg::PolygonMode::LINE);    

    _outlineDrawable->getOrCreateStateSet()->setAttributeAndModes(polymode,osg::StateAttribute::OVERRIDE|osg::StateAttribute::ON);
    _outlineDrawable->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF);

    _outlineGeode->addDrawable( _outlineDrawable.get() );    
    
    updateTransform();
}

EllipsoidPrimitive::~EllipsoidPrimitive()
{
}

const osg::Vec3d&
EllipsoidPrimitive::getSize() const
{
    return _size;
}

void
EllipsoidPrimitive::setSize(const osg::Vec3d& size)
{
    if (_size != size)
    {
        _size = size;
        updateTransform();
    }
}

const Location&
EllipsoidPrimitive::getLocation() const
{
    return _location;
}

void
EllipsoidPrimitive::setLocation(const Location& location)
{
    if (_location != location)
    {
        _location = location;
        updateTransform();
    }
}

AltitudeMode
EllipsoidPrimitive::getAltitudeMode() const
{
    return _altitudeMode;
}

void
EllipsoidPrimitive::setAltitudeMode(AltitudeMode altitudeMode)
{
    if (_altitudeMode != altitudeMode)
    {
        _altitudeMode = altitudeMode;
        updateTransform();
    }
}



void
EllipsoidPrimitive::setMapContext(MapContext* context)
{    
    Primitive::setMapContext(context);
    updateTransform();
}


const osg::Vec4&
EllipsoidPrimitive::getFillColor() const
{
    return _fillColor;
}

void
EllipsoidPrimitive::setFillColor( const osg::Vec4& color)
{
    if (_fillColor != color)
    {
        _fillColor = color;
        if (_fillDrawable.valid())
        {
            static_cast<osg::ShapeDrawable*>(_fillDrawable.get())->setColor( _fillColor );
            _fillDrawable->dirtyDisplayList();            
        }
    }
}

const osg::Vec4&
EllipsoidPrimitive::getOutlineColor() const
{
    return _outlineColor;
}

void
EllipsoidPrimitive::setOutlineColor( const osg::Vec4& color)
{
    if (_outlineColor != color)
    {
        _outlineColor = color;
        if (_outlineDrawable.valid())
        {
            static_cast<osg::ShapeDrawable*>(_outlineDrawable.get())->setColor( _outlineColor );
            _outlineDrawable->dirtyDisplayList();
        }
    }
}

void
EllipsoidPrimitive::setShowOutline( bool showOutline )
{
    if (_showOutline != showOutline)
    {
        _showOutline = showOutline;        
        updateStyle();
    }
}

bool
EllipsoidPrimitive::getShowOutline() const
{
    return _showOutline;
}

void
EllipsoidPrimitive::setShowFill( bool showFill )
{
    if (_showFill != showFill)
    {
        _showFill = showFill;
        updateStyle();
    }
}

bool
EllipsoidPrimitive::getShowFill() const
{
    return _showFill;
}

void
EllipsoidPrimitive::updateStyle() 
{
    if (_outlineGeode.valid())
    {
        _outlineGeode->setNodeMask( _showOutline ? ~0x0 : 0x0 );
    }

    if (_fillGeode.valid())
    {
        _fillGeode->setNodeMask( _showFill ? ~0x0 : 0x0 );
    }
}

void
EllipsoidPrimitive::terrainChanged(const osgEarth::TileKey& tileKey, osg::Node* terrain)
{    
    if (tileKey.getExtent().contains( _location.getLongitude(), _location.getLatitude() ) )
    {
      updateTransform( terrain );
    }    
}

void
EllipsoidPrimitive::updateTransform(osg::Node* terrain)
{
    if (!terrain && _context.valid())
    {
        terrain = _context->getMapNode()->getTerrainEngine();
    }

    if (_transform.valid())
    {
        osg::ref_ptr< const osg::EllipsoidModel > em;

        osg::Matrixd matrix;
        osg::Vec3d position;
        osg::Vec3  normal;

        if (_context.valid())
        {
            em = _context->getEllipsoid();
            _location.getGeocentric(em.get(), position);
            osg::Vec3 normal;
            if (_altitudeMode == AltitudeMode::ALTMODE_RELATIVE)
            {
                //_context->getIntersection(position, position, normal);
                MapContext::getNodeIntersection(terrain, em.get(), position, position, normal, _location.getAltitude());
            }
            em->computeLocalToWorldTransformFromXYZ(position.x(), position.y(), position.z(), matrix);            
        }
        else
        {
            em = Primitive::getWGS84Ellipsoid();
            _location.getGeocentric(em.get(), position);
            em->computeLocalToWorldTransformFromXYZ( position.x(), position.y(), position.z(), matrix );                        
        }        

        //Scale
        matrix.preMult(osg::Matrixd::scale(_size));

        //Rotate
        /*osg::Matrix rot_mat;
        rot_mat.makeRotate( 
            osg::DegreesToRadians(_pitch), osg::Vec3(1,0,0),
            osg::DegreesToRadians(_heading), osg::Vec3(0,0,1),
            osg::DegreesToRadians(_roll), osg::Vec3(0,1,0) );        

        matrix.preMult(rot_mat);
        */

        _transform->setMatrix( matrix );

        //NOTE:  Must set the cull callback on the group and not the autotransform as getEyePoint will be in a funky coordinate system if you stick it on the autotransform.
        /*if (_enableClusterCulling)
        {
            osg::Vec3d localNormal = em->computeLocalUpVector(position.x(), position.y(), position.z());
            _ccc = static_cast<osg::ClusterCullingCallback*>(getCullCallback());
            if (!_ccc)
            {
                _ccc = new osg::ClusterCullingCallback();
                setCullCallback( _ccc );
            }
            _ccc->set(position, localNormal, 0, -1);        
        }
        else
        {
            setCullCallback( NULL );
        }*/
    }
}

void
EllipsoidPrimitive::setProperty(const std::string& name, double value)
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
    else if (name == "width") {
        setSize(osg::Vec3d(value, _size.y(), _size.z()));
    }
    else if (name == "length") {
        setSize(osg::Vec3d(_size.x(), value, _size.z()));
    }
    else if (name == "height") {
        setSize(osg::Vec3d(_size.x(), _size.y(), value));
    }
    else {
        Primitive::setProperty( name, value );
    }
}

void
EllipsoidPrimitive::setProperty(const std::string& name, const osg::Vec3d& value)
{
    if (name == "fgcolor")
    {
        setFillColor(osg::Vec4(value.x(), value.y(), value.z(), 1.0f));
    }
    else if (name == "outlinecolor") {
        setOutlineColor(osg::Vec4(value.x(), value.y(), value.z(), 1.0f));
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
EllipsoidPrimitive::setProperty(const std::string& name, int value)
{
    if (name == "fgcolor")
    {
        setFillColor(getColorFromDecimal( value ) );
    }
	else if (name == "transparency") {
		osg::Vec4 changeAlpha = getFillColor();
		changeAlpha[3] = osg::clampBetween(1- (value/100.0), 0.0, 1.0);
		setFillColor(changeAlpha);
	}
    else if (name == "outlinecolor") {
        setOutlineColor(getColorFromDecimal( value ) );
    }
    else if (name == PROP_ALTITUDE_MODE)
    {
        setAltitudeMode( (AltitudeMode) value );
    }
    else if (name == "majoraxis") {
        setSize(osg::Vec3d(value, _size.y(), _size.z()));
    }
    else if (name == "minoraxis") {
        setSize(osg::Vec3d(_size.x(), value, _size.z()));
    }
    else
    {
        Primitive::setProperty(name, value);
    }    
}

void
EllipsoidPrimitive::setProperty(const std::string& name, bool value)
{
    if (name == "showoutline") {
        setShowOutline( value );
    }
    else {
        Primitive::setProperty( name, value );
    }
}

void
EllipsoidPrimitive::setProperty(const std::string& name, const LocationList& locations)
{
    if (name.compare(PROP_POINTLIST) == 0)
    {
		// Use the PROP_LATITUDE and PROP_LONGITUDE when available.
		// But if the PRIM_POINTLIST is of size 1, then that is a substitute
		if( locations.size() == 1 )
			setLocation(locations[0]);
	}
}
