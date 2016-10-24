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

#include <osgJuniperMap/PolylinePrimitive>
#include <osgJuniperMap/Registry>

#include <osg/ClusterCullingCallback>
#include <osg/LineWidth>
#include <osg/LineStipple>

#include <osgEarth/GeoMath>

#include <osgEarthFeatures/Feature>
#include <osgEarthFeatures/TransformFilter>
#include <osgEarthFeatures/BuildGeometryFilter>
#include <osgEarthFeatures/ResampleFilter>

#include <osg/Geometry>
#include <osg/io_utils>

#include <osgDB/WriteFile>


#include <sstream>

using namespace osgJuniper::Map;
using namespace osgEarth::Symbology;
using namespace osgEarth::Features; 
using namespace osgEarth::Annotation;

REGISTER_SIMPLE_PRIMITIVE_FACTORY(PolylinePrimitive)
REGISTER_SIMPLE_PRIMITIVE_FACTORY_TYPE(PolylinePrimitive, Polyline)

/***************************************************************************/
PolylinePrimitive::PolylinePrimitive(PrimitiveId id):
Primitive(id),
_color( 1,1,1,1 ),
_lineStyle( LINESTYLE_SOLID ),
_clamped( true ),
_alpha(1.0)
{
	_lineWidth = new osg::LineWidth();
}

PolylinePrimitive::~PolylinePrimitive()
{
	if( _primType == TYPE_SCREEN ){
		_screenGeometry->removePrimitiveSet(0, _points->size());
		_points->clear();
	}
}

const Geometry*
PolylinePrimitive::getGeometry() const
{
    return _geometry.get();
}

Geometry*
PolylinePrimitive::getGeometry()
{
    return _geometry.get();
}

void
PolylinePrimitive::setGeometry(Geometry* geometry)
{
    if (_geometry != geometry)
    {
        _geometry = geometry;
        init();
    }
}

void
PolylinePrimitive::setPrimitiveType(PrimitiveType type){
	_primType = type;

	//Remove all the children
    removeChildren( 0, getNumChildren());

	if( _primType == TYPE_SCREEN ){
		this->getOrCreateStateSet()->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);
		getOrCreateStateSet()->setMode(GL_DEPTH_TEST, osg::StateAttribute::OFF);

		_geode = new osg::Geode();
		// create Geometry object to store all the vertices and lines primitive.
		_screenGeometry = new osg::Geometry();
	    
		_points = new osg::Vec2Array;

		// set the colors
		_colorArray = new osg::Vec4Array;
		_colorArray->push_back(osg::Vec4(1.0f,1.0f,0.0f,1.0f));
		_screenGeometry->setColorArray(_colorArray);
		_screenGeometry->setColorBinding(osg::Geometry::BIND_OVERALL);
	    
		_lineWidth = new osg::LineWidth();


		// add the points geometry to the geode.
		_geode->addDrawable(_screenGeometry);

		addChild( _geode.get() );

		if (_context.valid() && _terrainChangedCallback.valid())
		{
			_context->getMapNode()->getTerrain()->removeTerrainCallback( _terrainChangedCallback.get() );
			_terrainChangedCallback = 0;
			removeCullCallback(_ccc);
		}
	}
}

void
PolylinePrimitive::setMapContext(MapContext* context)
{
    Primitive::setMapContext(context);
    _mapNode = _context->getMapNode();
    init();
}

void PolylinePrimitive::init()
{
    if (_mapNode.valid() && _geometry.valid() && _primType == TYPE_GEOLOCATED)
    {
        //Remove all the children
        removeChildren( 0, getNumChildren());

        _feature = new Feature( _geometry.get(), osgEarth::SpatialReference::create( "epsg:4326"));

        _featureNode = new FeatureNode( _mapNode.get(), _feature.get());

        _featureNode->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
		_featureNode->getOrCreateStateSet()->setMode(GL_LINE_SMOOTH, osg::StateAttribute::ON);

        styleChanged();
        
        addChild( _featureNode.get() );
    }
}

void
PolylinePrimitive::styleChanged()
{
    if (_featureNode.valid() && _feature.valid() && _primType == TYPE_GEOLOCATED)
    {
        //Build a style
        Style style;
        LineSymbol* ls = style.getOrCreateSymbol< LineSymbol >();
		ls->stroke()->color() = _color;
		ls->stroke()->width() = _lineWidth->getWidth();
        if (_lineStyle != LINESTYLE_SOLID)
        {
            ls->stroke()->stipple() = (unsigned short)_lineStyle;
        }

        if (_clamped)
        {
            AltitudeSymbol* as = style.getOrCreate<AltitudeSymbol>();
            as->clamping() = AltitudeSymbol::CLAMP_TO_TERRAIN;
        }

        _feature->style() = style;
        _featureNode->init();
    }
}


float
PolylinePrimitive::getLineWidth() const
{
	return _lineWidth->getWidth();
}

void
PolylinePrimitive::setLineWidth( float lineWidth )
{
	if (_lineWidth->getWidth() != lineWidth)
    {
		_lineWidth->setWidth(lineWidth);
		if( _primType == TYPE_GEOLOCATED )
			styleChanged();
		else
			_geode->getOrCreateStateSet()->setAttributeAndModes(_lineWidth, osg::StateAttribute::ON);
    }
}

PolylinePrimitive::LineStyle
PolylinePrimitive::getLineStyle() const
{
    return _lineStyle;
}

void
PolylinePrimitive::setLineStyle( PolylinePrimitive::LineStyle lineStyle )
{
    if (_lineStyle != lineStyle && _primType == TYPE_GEOLOCATED)
    {
        _lineStyle = lineStyle;
        styleChanged();        
    }
}

bool
PolylinePrimitive::getClamped() const
{
    return _clamped;
}

void
PolylinePrimitive::setClamped( bool clamped )
{
    if (_clamped != clamped && _primType == TYPE_GEOLOCATED)
    {
        _clamped = clamped;
        init();
    }
}

const osg::Vec4&
PolylinePrimitive::getColor() const
{
    return _color;
}

void
PolylinePrimitive::setColor( osg::Vec4& color)
{
    if (_color != color && _primType == TYPE_GEOLOCATED)
    {
        _color = color;
        styleChanged();
    }
}

void
PolylinePrimitive::setProperty(const std::string& name, int value)
{
    if (name == PROP_LINESTYLE)
    {
        setLineStyle((LineStyle)value);
    }
    else if (name == PROP_LINEWIDTH)
    {
        setLineWidth((float)value);
    }
    else if (name == PROP_FGCOLOR)
    {
        setColor(Primitive::getColorFromDecimal(value));
    }
    else if (name == PROP_ALTITUDE_MODE)
    {
        AltitudeMode am = (AltitudeMode)value;
        setClamped( am == AltitudeMode::ALTMODE_RELATIVE );
    }
    else
    {
        Primitive::setProperty(name, value);
    }
}

void
PolylinePrimitive::setProperty(const std::string& name, double value)
{
    if (name == PROP_LINEWIDTH)
    {
        setLineWidth((float)value);
    }
    else
    {
        Primitive::setProperty(name, value);
    }
}

void
PolylinePrimitive::setProperty(const std::string& name, const osg::Vec3d& value)
{
    if (name == PROP_FGCOLOR)
    {
        setColor(osg::Vec4(value.x(), value.y(), value.z(), _alpha));
    }
    else
    {
        Primitive::setProperty(name, value);
    }
}

void
PolylinePrimitive::setProperty(const std::string& name, const LocationList& locations)
{
    if (name.compare(PROP_POINTLIST) == 0)
    {
        osgEarth::Symbology::LineString *lineString = new osgEarth::Symbology::LineString();
        lineString->reserve( locations.size() );

        for (unsigned int i = 0; i < locations.size(); ++i)
        {
            lineString->push_back( locations[i].asVec3d() );            
        }
       
        this->getOrCreateStateSet()->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);           
        this->setGeometry( lineString );
    }
}

void
PolylinePrimitive::setProperty(const std::string& name, std::vector< osg::Vec2 > points)
{
    if (name.compare(PROP_POINTLIST) == 0)
    {
		if( _primType == TYPE_SCREEN ){
			_screenGeometry->removePrimitiveSet(0, _points->size());
			_points->clear();

			// Copy in the first value only one time
			// EX: {0,
			if( points.size() > 0 )
				_points->push_back(points[0]);
			else
				return;
			for(std::vector< osg::Vec2 >::iterator k = points.begin() + 1; k != points.end(); ++k)
			{
				// Copy in additional points twice
				// EX: 1},{1,
				_points->push_back(*k);
				_points->push_back(*k);
			}
			// But we don't want the last element pushed twice, so pop it off
			// EX: 4}
			_points->pop_back();

			_screenGeometry->setVertexArray(_points);
			_screenGeometry->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::LINES, 0, _points->size()));
		}
    }
}
