#include <osgJuniper/TrackHistoryNode>

#include <osg/LineWidth>
#include <osg/Geometry>
#include <osg/Geode>
#include <osg/BlendFunc>
#include <osg/io_utils>
#include <osg/CullFace>
#include <osg/Point>

#include <iostream>

using namespace osgJuniper;

TrackHistoryNode::TrackHistoryNode():
_color(1.0f, 0.0f, 0.0f, 1.0f),
_bottomColor(0.0f, 1.0f, 0.0f, 1.0f),
_ribbonWidth(4.0f),
_lineWidth(3.0f),
_maxNumVertices(5000),
_minDelta(0.0),
_maxDelta(DBL_MAX),
_fade(false),
_style(TrackHistoryNode::STYLE_PLANE)
{
    setNumChildrenRequiringUpdateTraversal( 1 );

	_transform = new osg::MatrixTransform();	

	//Set up stateset modes
	osg::StateSet* stateset = getOrCreateStateSet();
	stateset->setRenderBinDetails(10, "RenderBin");
	stateset->setAttributeAndModes(new osg::LineWidth(_lineWidth) );
	stateset->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
	stateset->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);
	stateset->setAttributeAndModes(new osg::BlendFunc(osg::BlendFunc::SRC_ALPHA, osg::BlendFunc::ONE_MINUS_SRC_ALPHA));	
	stateset->setAttributeAndModes(new osg::Point(10));
    stateset->setDataVariance( osg::Object::DYNAMIC );

	addChild( _transform.get() );

	if (_style == STYLE_PLANE)
	{
		initPlane();
	}
	else if (_style == STYLE_LINE)
	{
		initLine();
	}
}

osg::Node*
TrackHistoryNode::getTrackNode() const
{
	return _trackNode.get();
}

void TrackHistoryNode::setTrackNode(osg::Node* node)
{
    _trackNode = node;
}

void TrackHistoryNode::initPlane()
{
	//Always reserve a reasonable amount, even if maxNumVerts is negative.
	unsigned int toReserve = _maxNumVertices <= 0 ? 1000 : _maxNumVertices;

	//Create the plane draw arrays, which is shared between both the top and bottom passes
	_planeDrawArrays = new osg::DrawArrays();	

	//Allocate the plane vertices and colors
	_planeVertices = new osg::Vec3Array();	
	_planeVertices->reserve(toReserve * 2 );
	_bottomColors = new osg::Vec4Array();	
	_bottomColors->reserve(toReserve * 2 );
	_topColors = new osg::Vec4Array();	
	_topColors->reserve(toReserve * 2 );


	//Create the top pass
	_planeTopGeode = new osg::Geode;
	_planeTopGeode->getOrCreateStateSet()->setAttributeAndModes(new osg::CullFace(osg::CullFace::BACK), osg::StateAttribute::ON);
	_planeTopGeometry = new osg::Geometry;
    _planeTopGeometry->setDataVariance( osg::Object::DYNAMIC );
	_planeTopGeometry->setVertexArray( _planeVertices );
	_planeTopGeometry->setColorArray( _topColors.get() );
	_planeTopGeometry->setColorBinding( osg::Geometry::BIND_PER_VERTEX );
	_planeTopGeometry->addPrimitiveSet( _planeDrawArrays.get() );
	_planeTopGeometry->setUseDisplayList( false );
	_planeTopGeode->addDrawable( _planeTopGeometry.get() );
	_transform->addChild(_planeTopGeode.get());

	//Create the bottom pass.
	_planeBottomGeode = new osg::Geode;
	_planeBottomGeode->getOrCreateStateSet()->setAttributeAndModes(new osg::CullFace(osg::CullFace::FRONT), osg::StateAttribute::ON);
	_planeBottomGeometry = new osg::Geometry;	
    _planeBottomGeometry->setDataVariance( osg::Object::DYNAMIC );
	_planeBottomGeometry->setVertexArray( _planeVertices );
	_planeBottomGeometry->setColorArray( _bottomColors.get() );
	_planeBottomGeometry->setColorBinding( osg::Geometry::BIND_PER_VERTEX );
	_planeBottomGeometry->addPrimitiveSet( _planeDrawArrays.get() );
	_planeBottomGeometry->setUseDisplayList( false );
	_planeBottomGeode->addDrawable( _planeBottomGeometry.get() );
	_planeTopGeode->setCullingActive(false);
	_planeBottomGeode->setCullingActive(false);
	_transform->addChild(_planeBottomGeode.get());
}

void TrackHistoryNode::initLine()
{
	unsigned int toReserve = _maxNumVertices < 0 ? 1000 : _maxNumVertices;

	//Allocate the line vertices and colors
	_lineVertices = new osg::Vec3Array();	
	_lineVertices->reserve(toReserve);
	_lineColors = new osg::Vec4Array();
	_lineColors->reserve(toReserve);


	//Create the line geometry and geode
	_lineGeode = new osg::Geode;
	//osg::Geometry* lineGeometry = new osg::Geometry;
    _lineGeometry = new osg::Geometry;
    _lineGeometry->setDataVariance( osg::Object::DYNAMIC );
	_lineGeometry->setVertexArray( _lineVertices.get() );	
	_lineGeometry->setColorArray( _lineColors.get() );
	_lineGeometry->setColorBinding(osg::Geometry::BIND_PER_VERTEX);
	_lineDrawArrays = new osg::DrawArrays();
	_lineGeometry->addPrimitiveSet( _lineDrawArrays.get() );
	_lineGeometry->setUseDisplayList( false );
	_lineGeode->addDrawable( _lineGeometry.get() );
	_lineGeode->setCullingActive(false);
	_transform->addChild(_lineGeode.get());
}

void TrackHistoryNode::clearPlane()
{
	//Remove the geode
	if (_transform->containsNode( _planeTopGeode.get() )) _transform->removeChild( _planeTopGeode.get() );
	if (_transform->containsNode( _planeBottomGeode.get() )) _transform->removeChild( _planeBottomGeode.get() );
	_planeVertices = 0;
	_planeDrawArrays = 0;
	_planeTopGeode = 0;
	_planeBottomGeode = 0;
	_planeTopGeometry = 0;
	_planeBottomGeometry = 0;
	_topColors = 0;
	_bottomColors = 0;
    dirtyGeometryBounds();
}

void TrackHistoryNode::clearLine()
{
	if (_transform->containsNode( _lineGeode.get() ) ) _transform->removeChild( _lineGeode.get() );
	_lineVertices = 0;
	_lineDrawArrays = 0;
	_lineGeometry = 0;
	_lineGeode = 0;
	_lineColors = 0;
    dirtyGeometryBounds();
}

void TrackHistoryNode::dirtyGeometryBounds()
{
    if (_planeTopGeometry.valid()) _planeTopGeometry->dirtyBound();
    if (_planeBottomGeometry.valid()) _planeBottomGeometry->dirtyBound();
    if (_lineGeometry.valid()) _lineGeometry->dirtyBound();
}

int TrackHistoryNode::getMaxNumVertices() const
{
	return _maxNumVertices;
}

void TrackHistoryNode::setMaxNumVertices(int maxNumVertices)
{
	if (_maxNumVertices != maxNumVertices)
	{
		_maxNumVertices = maxNumVertices;

		if (_maxNumVertices > 0)
		{
			//Determine if we need to remove any verts from the arrays to respect the new maximum
			int delta = _samples.size() - _maxNumVertices;

			if (delta > 0)
			{
				_samples.erase( _samples.begin(), _samples.begin() + delta);
				if (_lineVertices.valid()) _lineVertices->erase(_lineVertices->begin(), _lineVertices->begin() + delta);
				if (_lineColors.valid())   _lineColors->erase(_lineColors->begin(), _lineColors->begin() + delta);
				if (_planeVertices.valid()) _planeVertices->erase(_planeVertices->begin(), _planeVertices->begin() + delta*2);
				if (_topColors.valid()) _topColors->erase(_topColors->begin(), _topColors->begin() + delta *2);
				if (_bottomColors.valid()) _bottomColors->erase( _bottomColors->begin(), _bottomColors->begin() + delta*2);
                dirtyGeometryBounds();
			}


			//Reserve space in the arrays
			if (_lineVertices.valid()) _lineVertices->reserve( _maxNumVertices );		
			if (_lineColors.valid()) _lineColors->reserve( _maxNumVertices );
			if (_planeVertices.valid()) _planeVertices->reserve( _maxNumVertices * 2 );
			if (_topColors.valid()) _topColors->reserve( _maxNumVertices * 2 );
			if (_bottomColors.valid()) _bottomColors->reserve( _maxNumVertices * 2 );
		}
	}
}

double TrackHistoryNode::getMinDelta() const
{
	return _minDelta;
}

void TrackHistoryNode::setMinDelta( double minDelta )
{
	_minDelta = minDelta;
}

double TrackHistoryNode::getMaxDelta() const
{
	return _maxDelta;
}

void TrackHistoryNode::setMaxDelta( double maxDelta )
{
	_maxDelta = maxDelta;
}

float TrackHistoryNode::getLineWidth() const
{
	return _lineWidth;
}

void TrackHistoryNode::setLineWidth( float lineWidth )
{
	if (_lineWidth != lineWidth)
	{
		_lineWidth = lineWidth;
		osg::ref_ptr<osg::LineWidth> lw = static_cast<osg::LineWidth*>(getOrCreateStateSet()->getAttribute(osg::StateAttribute::LINEWIDTH));
		if (lw.valid())
		{
			lw->setWidth( _lineWidth );
		}
	}
}



float TrackHistoryNode::getRibbonWidth() const
{
	return _ribbonWidth;
}

void TrackHistoryNode::setRibbonWidth( float ribbonWidth)
{
	if (_ribbonWidth != ribbonWidth)
	{
		_ribbonWidth = ribbonWidth;

		//Only update the verts if using the plane style.
		if (_style == STYLE_PLANE)
		{
			//Since the ribbon width has changed, we need to regenreate all the verts
			double widthHalf = _ribbonWidth / 2.0f;

			for (unsigned int i = 0; i < _samples.size(); ++i)
			{
				//Get the up vector
				osg::Matrixd &mat = _samples[i];

                osg::Vec3d position(mat(3,0),mat(3,1),mat(3,2));
				osg::Vec3d side(mat(0,0), mat(0,1), mat(0,2));

				osg::Vec3d v0 = position - (side * widthHalf) - _origin;
				osg::Vec3d v1 = position + (side * widthHalf) - _origin;

				(*_planeVertices)[i*2]   = v0;
				(*_planeVertices)[i*2+1] = v1;
			}

            dirtyGeometryBounds();
		}
	}
}

const osg::Vec4f& TrackHistoryNode::getColor() const
{
	return _color;
}

void TrackHistoryNode::setColor( const osg::Vec4f& color )
{
	if (color != _color)
	{
		_color = color;
		updateColors();
	}
}

const osg::Vec4f& TrackHistoryNode::getBottomColor() const
{
	return _bottomColor;
}

void TrackHistoryNode::setBottomColor( const osg::Vec4f& bottomColor)
{
	if (_bottomColor != bottomColor)
	{
		_bottomColor = bottomColor;
		updateColors();
	}
}

TrackHistoryNode::Style TrackHistoryNode::getStyle() const
{
	return _style;
}

void TrackHistoryNode::setStyle( TrackHistoryNode::Style style )
{
	if (_style != style)
	{
		_style = style;

		if (_style == STYLE_PLANE)
		{
			clearLine();
			initPlane();
		}
		else if (_style == STYLE_LINE)
		{
			clearPlane();
			initLine();
		}
	}
}

bool TrackHistoryNode::getFade() const
{
	return _fade;
}

void TrackHistoryNode::setFade(bool fade)
{
	if (_fade != fade)
	{
		_fade = fade;
		updateColors();
	}
}

void TrackHistoryNode::reset()
{
	_samples.clear();

	if (_lineVertices.valid())  _lineVertices->clear();
	if (_planeVertices.valid()) _planeVertices->clear();

	if (_lineDrawArrays.valid())  _lineDrawArrays->set(osg::PrimitiveSet::LINE_STRIP, 0, _lineVertices->size());
	if (_planeDrawArrays.valid()) _planeDrawArrays->set(osg::PrimitiveSet::TRIANGLE_STRIP, 0, _planeVertices->size());

    dirtyGeometryBounds();
	updateColors();
}

void TrackHistoryNode::updateColors()
{
	if (_style == STYLE_LINE)
	{
		//Update the line colors
		//Make sure there are the same number of colors as their are verts
		if (_lineColors->size() != _lineVertices->size())
		{
			_lineColors->resize(_lineVertices->size());
		}

		unsigned int numVerts = _lineVertices->size();
		for (unsigned int i = 0; i < numVerts; ++i)
		{			
			if (_fade)
			{
				float normal = (float)i / (float)numVerts;
				(*_lineColors)[i] = osg::Vec4f(_color.r(), _color.g(), _color.b(), _color.a() * normal * 0.95f);
			}
			else
			{
				(*_lineColors)[i] = _color;
			}
		}
	}

	if (_style == STYLE_PLANE)
	{
		//Update the polygon colors
		if (_topColors->size()    != _planeVertices->size()) _topColors->resize( _planeVertices->size());
		if (_bottomColors->size() != _planeVertices->size()) _bottomColors->resize( _planeVertices->size());

		unsigned int numVerts = _planeVertices->size() / 2;
		for (unsigned int i = 0; i < numVerts; ++i)
		{
			unsigned int j = i * 2;
			if (_fade)
			{
				float normal = (float)i / (float)numVerts;
				osg::Vec4f topColor(_color.r(), _color.g(), _color.b(), _color.a() * normal * 0.95f);
				(*_topColors)[j] = topColor;
				(*_topColors)[j+1] = topColor;

				osg::Vec4f bottomColor(_bottomColor.r(), _bottomColor.g(), _bottomColor.b(), _bottomColor.a() * normal * 0.95f);
				(*_bottomColors)[j] = bottomColor;
				(*_bottomColors)[j+1] = bottomColor;
			}
			else
			{
				(*_topColors)[j] = _color;
				(*_topColors)[j+1] = _color;
				(*_bottomColors)[j] = _bottomColor;
				(*_bottomColors)[j+1] = _bottomColor;
			}
		}
	}
}

void TrackHistoryNode::traverse(osg::NodeVisitor &nv)
{
    if (nv.getVisitorType() == osg::NodeVisitor::UPDATE_VISITOR)
    {
        if (_trackNode.valid())
        {
            osg::NodePathList nodePaths = _trackNode->getParentalNodePaths();
            if (!nodePaths.empty())
            {
                osg::NodePath path = nodePaths[0];
                osg::Matrixd localToWorld = osg::computeLocalToWorld( path );
				//Reset every 50 samples
				/*if (_samples.size() > 50)
				{
					reset();
				}*/
                addSample(localToWorld);
            }
        }
    }
    osg::Group::traverse(nv);
}

void TrackHistoryNode::addSample(const osg::Matrixd &matrix)
{
    osg::Vec3d position(matrix(3,0),matrix(3,1),matrix(3,2));

	//Only add the new sample if it is within the min distance, or we have no points
	if (_samples.size() > 0)
	{
		double distance = (position - _lastPosition).length();
		if (distance < _minDelta)
		{
			return;
		}

		//If the distance between the new point and the last is more than the max delta, reset the trail
		if (distance > _maxDelta)
		{
			reset();
		}
	}

	if (_maxNumVertices > 0 && _samples.size() +1 > _maxNumVertices)
	{
		//Erase a vert from the ribbon
		_samples.erase(_samples.begin());

		//Erase a vert from the line
		if (_lineVertices.valid()) _lineVertices->erase(_lineVertices->begin());

		//Erase the first two verts from the plane
		if (_planeVertices.valid()) _planeVertices->erase(_planeVertices->begin(), _planeVertices->begin() + 2);
	}

	//Add the new sample
	_samples.push_back( matrix );
	_lastPosition = position;
   

	//If this is the first vert we have, use it as the origin.
	if (_samples.size() == 1)
	{
		_origin = position;
		_transform->setMatrix(osg::Matrixd::translate(_origin ) );
	}

	if (_style == STYLE_LINE)
	{
		osg::Vec3f vert = position - _origin;
		_lineVertices->push_back( vert );
        _lineDrawArrays->set(osg::PrimitiveSet::LINE_STRIP, 0, _lineVertices->size());
	}
	else if (_style == STYLE_PLANE)
	{
		double widthHalf = _ribbonWidth / 2.0f;

		//Get the up vector
		osg::Vec3d side(matrix(0,0), matrix(0,1), matrix(0,2));
        //osg::Vec3d side(0,0,1);    
        //OSG_NOTICE << "Side " << side << std::endl;

		osg::Vec3d v0 = position - (side * widthHalf) - _origin;
		osg::Vec3d v1 = position + (side * widthHalf) - _origin;

		_planeVertices->push_back( v0 );
		_planeVertices->push_back( v1 );
        _planeDrawArrays->set(osg::PrimitiveSet::QUAD_STRIP, 0, _planeVertices->size());
	}

	updateColors();
    dirtyGeometryBounds();
}
