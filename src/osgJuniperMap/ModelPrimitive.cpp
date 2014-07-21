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

#include <osgJuniperMap/ModelPrimitive>
#include <osgJuniperMap/Registry>
#include <osgJuniper/Utils>

#include <osg/CoordinateSystemNode>
#include <osgEarth/TerrainEngineNode>

#include <osg/ClusterCullingCallback>
#include <osgDB/ReadFile>
#include <osg/io_utils>

#include <sstream>

using namespace osgJuniper::Map;

REGISTER_SIMPLE_PRIMITIVE_FACTORY(ModelPrimitive)
REGISTER_SIMPLE_PRIMITIVE_FACTORY_TYPE(ModelPrimitive, Model)

/***************************************************************************/
ModelPrimitive::ModelPrimitive(PrimitiveId id):
Primitive(id),
_scale(1,1,1),
_heading(0),
_pitch(0),
_roll(0),
_orientToGround(true),
_locationSet( false )
{
    getOrCreateStateSet()->setMode( GL_NORMALIZE, osg::StateAttribute::ON );
}

const Location&
ModelPrimitive::getLocation() const
{
    return _location;
}

void
ModelPrimitive::setLocation( const Location& location)
{
    if (_location != location)
    {
        _location = location;
        updateTransform();        
        _locationSet = true;
    }
}

AltitudeMode
ModelPrimitive::getAltitudeMode() const
{
    return _altitudeMode;
}

void
ModelPrimitive::setAltitudeMode(AltitudeMode altitudeMode)
{
    if (_altitudeMode != altitudeMode)
    {
        _altitudeMode = altitudeMode;
        updateTransform();
    }
}

void
ModelPrimitive::setMapContext(MapContext* context)
{
    Primitive::setMapContext(context);
    updateTransform();
    //init();
}


const osg::Vec3d&
ModelPrimitive::getScale() const
{
    return _scale;
}

void
ModelPrimitive::setScale( const osg::Vec3d& scale)
{
    if (_scale != scale)
    {
        _scale = scale;
        updateTransform();
    }
}

void
ModelPrimitive::setScale(double x, double y, double z)
{
    setScale(osg::Vec3d(x,y,z));
}



const std::string&
ModelPrimitive::getFilename() const
{
    return _filename;
}

void
ModelPrimitive::setFilename( const std::string& filename)
{
    if (_filename != filename)
    {
        _filename = filename;
        _model = 0;
        init();
    }
}

void
ModelPrimitive::setHeading( double heading )
{
    if (_heading != heading)
    {        
        _heading = heading;
        updateTransform();
    }
}

void
ModelPrimitive::setPitch( double pitch )
{
    if (_pitch != pitch)
    {
        _pitch = pitch;
        updateTransform();
    }
}

void
ModelPrimitive::setRoll( double roll )
{
    if (_roll != roll)
    {
        _roll = roll;
        updateTransform();
    }
}

bool
ModelPrimitive::getOrientToGround() const
{
    return _orientToGround;
}

void
ModelPrimitive::setOrientToGround(bool orientToGround)
{
    if (_orientToGround != orientToGround)
    {
        _orientToGround = orientToGround;
        updateTransform();
    }
}

void
ModelPrimitive::setRotation(double heading, double pitch, double roll)
{
    if (_heading != heading || _pitch != pitch || _roll != roll)
    {
        _heading = heading;
        _pitch = pitch;
        _roll = roll;
        updateTransform();
    }
}

osg::Node*
ModelPrimitive::getModel() const
{
    return _model.get();
}

void
ModelPrimitive::setEnableClusterCulling(bool enableClusterCulling)
{
    if (_enableClusterCulling != enableClusterCulling)
    {
        _enableClusterCulling = enableClusterCulling;
        updateTransform();
    }
}

void
ModelPrimitive::terrainChanged(const osgEarth::TileKey& tileKey, osg::Node* terrain)
{    
    if (tileKey.getExtent().contains( _location.getLongitude(), _location.getLatitude() ) )
    {
      updateTransform( terrain );
    }    
}

void
ModelPrimitive::init()
{
    if (!_transform.valid())
    {
        _transform = new osg::MatrixTransform;
        addChild(_transform.get());
    }
    updateTransform();

    //Load the model if it's not already loaded
    if (!_model.valid())
    {
        //Remove all the current children
        _transform->removeChildren( 0, _transform->getNumChildren() );

        //Load the new file
        _model = osgDB::readNodeFile( _filename ); 

        if (_model.valid())
        {
            //Optimize the model
            //osgJuniper::Utils::optimizeMesh( _model.get() );
            _transform->addChild( _model.get() );
        }
    } 
}

void
ModelPrimitive::updateTransform(osg::Node* terrain)
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

        if (_locationSet)
        {
            if (_context.valid())
            {
                em = _context->getEllipsoid();
                _location.getGeocentric(_context->getEllipsoid(), position);
                osg::Vec3 normal;
                if (_altitudeMode == AltitudeMode::ALTMODE_RELATIVE)
                {
                    MapContext::getNodeIntersection(terrain, em.get(), position, position, normal, _location.getAltitude());
                }

                em->computeLocalToWorldTransformFromXYZ(position.x(), position.y(), position.z()+ 10, matrix);
                if (_orientToGround && _altitudeMode == AltitudeMode::ALTMODE_RELATIVE)
                {
                    osg::Quat q;
                    q.makeRotate(osg::Vec3d(0,0,1), osg::Vec3d(normal));
                    matrix.setRotate(q);
                }
            }
            else
            {
                em = Primitive::getWGS84Ellipsoid();
                _location.getGeocentric(em.get(), position);
                em->computeLocalToWorldTransformFromXYZ( position.x(), position.y(), position.z(), matrix );                        
            }

            //Scale
            matrix.preMult(osg::Matrixd::scale(_scale));

            //Rotate
            osg::Matrix rot_mat;
            rot_mat.makeRotate( 
                _pitch, osg::Vec3(1,0,0),
                _heading, osg::Vec3(0,0,1),
                _roll, osg::Vec3(0,1,0) );

            matrix.preMult(rot_mat);
            _transform->setMatrix( matrix );
        }

        //NOTE:  Must set the cull callback on the group and not the autotransform as getEyePoint will be in a funky coordinate system if you stick it on the autotransform.
        if (_locationSet && _enableClusterCulling)
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
        }
    }
}


void
ModelPrimitive::setProperty(const std::string& name, const std::string& value)
{
    if (name == PROP_FILEPATH)
    {
        setFilename(value);
    }
    else
    {
        Primitive::setProperty(name, value);
    }
}

void
ModelPrimitive::setProperty(const std::string& name, double value)
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
    else if (name == PROP_SCALE)
    {
        setScale(value, value, value);
    }
    else if (name == PROP_YAW)
    {
        setHeading(value);
    }
    else if (name == PROP_PITCH)
    {
        setPitch(value);
    }
    else if (name == PROP_ROLL)
    {
        setRoll(value);
    }
    else
    {
        Primitive::setProperty(name, value);
    }
}

void
ModelPrimitive::setProperty(const std::string& name, const osg::Vec3d& value)
{
    if (name == PROP_SCALE)
    {
        setScale(value);
    }
    else if (name == PROP_ROTATION)
    {
        setRotation(value.x(), value.y(), value.z());
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
ModelPrimitive::setProperty(const std::string& name, bool value)
{
    if (name == PROP_ORIENT_TO_GROUND)
    {
        setOrientToGround( value );
    }
    else
    {
        Primitive::setProperty(name, value);
    }
}

void
ModelPrimitive::setProperty(const std::string& name, int value)
{
    if (name == PROP_ALTITUDE_MODE)
    {
        setAltitudeMode( (AltitudeMode) value );
    }
    else
    {
        Primitive::setProperty(name, value);
    }
}