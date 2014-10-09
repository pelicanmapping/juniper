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

#include <osgJuniperMap/BoxPrimitive>
#include <osgJuniperMap/Registry>
#include <osg/io_utils>
#include <sstream>

#include <osg/CoordinateSystemNode>
#include <osgEarth/TerrainEngineNode>

using namespace osgJuniper::Map;

REGISTER_SIMPLE_PRIMITIVE_FACTORY(BoxPrimitive)
REGISTER_SIMPLE_PRIMITIVE_FACTORY_TYPE(BoxPrimitive, Box)

/***************************************************************************/
BoxPrimitive::BoxPrimitive(PrimitiveId id):
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

    //Make the boxes
    //Make a box centered around the origin
    float dx = 0.5f;
    float dy = 0.5f;
    float dz = 0.5f;
    
    osg::Vec3Array* verts = new osg::Vec3Array(8);
    //Looking at the box from the front with zup and xright the first 4 verts are on the front face going CC 
    //starting at the bottom left
    (*verts)[0] = osg::Vec3(-dx, dy, -dz);
    (*verts)[1] = osg::Vec3(dx , dy, -dz);
    (*verts)[2] = osg::Vec3(dx,  dy, dz);
    (*verts)[3] = osg::Vec3(-dx, dy, dz);
    (*verts)[4] = osg::Vec3(-dx, -dy, -dz);
    (*verts)[5] = osg::Vec3(dx , -dy, -dz);
    (*verts)[6] = osg::Vec3(dx,  -dy, dz);
    (*verts)[7] = osg::Vec3(-dx, -dy, dz);
    
    //Normals
    osg::Vec3Array* normals = new osg::Vec3Array(6);
    (*normals)[0] = osg::Vec3(0,1,0);
    (*normals)[1] = osg::Vec3(1,0,0);
    (*normals)[2] = osg::Vec3(0,-1,0);
    (*normals)[3] = osg::Vec3(-1,0,0);
    (*normals)[4] = osg::Vec3(0,0,1);
    (*normals)[5] = osg::Vec3(0,0,-1);
    
    //Make the fill geometry
    _fillGeometry = new osg::Geometry;
    _fillGeometry->setVertexArray( verts );
    GLuint faces_i[24] = { 0, 1, 2, 3, 
                          1, 5, 6, 2,  
                          5, 4, 7, 6,
                          4, 0, 3, 7,
                          3, 2, 6, 7,
                          0, 4, 5, 1
                          };        
    _fillGeometry->addPrimitiveSet( new osg::DrawElementsUInt( GL_QUADS, 24, faces_i ) );
    //TODO:  Fix
    //_fillGeometry->setNormalBinding( osg::Geometry::BIND_PER_PRIMITIVE );
    _fillGeometry->setNormalArray( normals );

    osg::Vec4Array* fillColors = new osg::Vec4Array(1);
    (*fillColors)[0] = _fillColor;
    _fillGeometry->setColorArray(fillColors);
    _fillGeometry->setColorBinding( osg::Geometry::BIND_OVERALL );    

    _fillGeode->addDrawable( _fillGeometry.get() );

    //Make the outline geometry
    _outlineGeometry = new osg::Geometry;
    _outlineGeometry->setVertexArray( verts );
    GLuint lines[24] = { 0, 1,
                         1, 5,
                         5, 4,
                         4, 0,
                         3, 2,
                         2, 6,
                         6, 7,
                         7, 3,
                         0, 3,
                         1, 2,
                         5, 6,
                         4, 7
                       };        
    _outlineGeometry->addPrimitiveSet( new osg::DrawElementsUInt( GL_LINES, 24, lines ) );
    
    osg::Vec4Array* outlineColors = new osg::Vec4Array(1);
    (*outlineColors)[0] = _outlineColor;
    _outlineGeometry->setColorArray(outlineColors);
    _outlineGeometry->setColorBinding( osg::Geometry::BIND_OVERALL );    

    _outlineGeometry->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF);

    _outlineGeode->addDrawable( _outlineGeometry.get() );
    
    
    updateTransform();
}

BoxPrimitive::~BoxPrimitive()
{
}

const osg::Vec3d&
BoxPrimitive::getSize() const
{
    return _size;
}

void
BoxPrimitive::setSize(const osg::Vec3d& size)
{
    if (_size != size)
    {
        _size = size;
        updateTransform();
    }
}

const Location&
BoxPrimitive::getLocation() const
{
    return _location;
}

void
BoxPrimitive::setLocation(const Location& location)
{
    if (_location != location)
    {
        _location = location;
        updateTransform();
    }
}

AltitudeMode
BoxPrimitive::getAltitudeMode() const
{
    return _altitudeMode;
}

void
BoxPrimitive::setAltitudeMode(AltitudeMode altitudeMode)
{
    if (_altitudeMode != altitudeMode)
    {
        _altitudeMode = altitudeMode;
        updateTransform();
    }
}



void
BoxPrimitive::setMapContext(MapContext* context)
{    
    Primitive::setMapContext(context);
    updateTransform();
}


const osg::Vec4&
BoxPrimitive::getFillColor() const
{
    return _fillColor;
}

void
BoxPrimitive::setFillColor( const osg::Vec4& color)
{
    if (_fillColor != color)
    {
        _fillColor = color;
        if (_fillGeometry.valid())
        {
            osg::Vec4Array* colors = static_cast<osg::Vec4Array*>(_fillGeometry->getColorArray());
            if (colors)
            {
                (*colors)[0] = _fillColor;
                _fillGeometry->dirtyDisplayList();
                //Enable blending if the color has an alpha value to it.
                if (_fillColor.a() < 1.0f) 
                {
                    osg::StateSet* stateset = _fillGeometry->getOrCreateStateSet();
                    stateset->setMode(GL_BLEND, osg::StateAttribute::ON);
                    stateset->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);
                }
            }
        }
    }
}

const osg::Vec4&
BoxPrimitive::getOutlineColor() const
{
    return _outlineColor;
}

void
BoxPrimitive::setOutlineColor( const osg::Vec4& color)
{
    if (_outlineColor != color)
    {
        _outlineColor = color;
        if (_outlineGeometry.valid())
        {
            osg::Vec4Array* colors = static_cast<osg::Vec4Array*>(_outlineGeometry->getColorArray());
            if (colors)
            {
                (*colors)[0] = _outlineColor;
                _outlineGeometry->dirtyDisplayList();
            }
        }
    }
}

void
BoxPrimitive::setShowOutline( bool showOutline )
{
    if (_showOutline != showOutline)
    {
        _showOutline = showOutline;        
        updateStyle();
    }
}

bool
BoxPrimitive::getShowOutline() const
{
    return _showOutline;
}

void
BoxPrimitive::setShowFill( bool showFill )
{
    if (_showFill != showFill)
    {
        _showFill = showFill;
        updateStyle();
    }
}

bool
BoxPrimitive::getShowFill() const
{
    return _showFill;
}

void
BoxPrimitive::updateStyle() 
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
BoxPrimitive::terrainChanged(const osgEarth::TileKey& tileKey, osg::Node* terrain)
{    
    if (tileKey.getExtent().contains( _location.getLongitude(), _location.getLatitude() ) )
    {
      updateTransform( terrain );
    }    
}

void
BoxPrimitive::updateTransform(osg::Node* terrain)
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
            _location.getGeocentric(_context->getEllipsoid(), position);
            osg::Vec3 normal;
            if (_altitudeMode == AltitudeMode::ALTMODE_RELATIVE)
            {
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
BoxPrimitive::setProperty(const std::string& name, double value)
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
BoxPrimitive::setProperty(const std::string& name, const osg::Vec3d& value)
{
    if (name == "fillcolor")
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
BoxPrimitive::setProperty(const std::string& name, bool value)
{
    if (name == "showfill")
    {
        setShowFill( value );
    }
    else if (name == "showoutline")
    {
        setShowOutline( value );
    }
    else
    {
        Primitive::setProperty(name, value);
    }
}

void
BoxPrimitive::setProperty(const std::string& name, int value)
{
    if (name == "fillcolor")
    {
        setFillColor(getColorFromDecimal( value ) );
    }
    else if (name == "outlinecolor") {
        setOutlineColor(getColorFromDecimal( value ) );
    }
    else if (name == PROP_ALTITUDE_MODE)
    {
        setAltitudeMode( (AltitudeMode) value );
    }
    else
    {
        Primitive::setProperty(name, value);
    }
}




