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

#include <osgJuniperMap/Articulation>

using namespace osgJuniper::Map;

ArticulationController::ArticulationController(osg::Node* node)
{
    _matrixTransform = dynamic_cast< osg::MatrixTransform* >(node);
    _dof             = dynamic_cast< osgSim::DOFTransform* >(node);
    if (!_matrixTransform.valid() && !_dof.valid())
    {
        OSG_NOTICE << "Error:  ArticulationController only support MatrixTransform or DOFTransform nodes" << std::endl;
    }

    if (_matrixTransform.valid())
    {
        _baseTrans = _matrixTransform->getMatrix().getTrans();
    }
    else if (_dof.valid())
    {
        _rotationOffset = _dof->getPutMatrix().getTrans();
        _rotation = _dof->getCurrentHPR();
        _translation = _dof->getCurrentTranslate();
    }
}

osg::Node*
ArticulationController::getNode() const
{
    if (_matrixTransform.valid()) return _matrixTransform.get();
    return _dof.get();
}

const osg::Vec3d&
ArticulationController::getTranslation() const
{
    return _translation;
}

void
ArticulationController::setTranslation( const osg::Vec3d& translation)
{
    if (_translation != translation)
    {
        _translation = translation;
        update();
    }
}


const osg::Vec3d&
ArticulationController::getRotationOffset() const
{
    return _rotationOffset;
}

void
ArticulationController::setRotationOffset( const osg::Vec3d& rotationOffset )
{
    if (_rotationOffset != rotationOffset)
    {
        _rotationOffset = rotationOffset;
        update();
    }
}

const osg::Vec3d&
ArticulationController::getRotation() const
{
    return _rotation;
}

void
ArticulationController::setRotation(const osg::Vec3d& rotation)
{
    if (_rotation != rotation)
    {
        _rotation = rotation;
        update();
    }
}

void
ArticulationController::update()
{
    if (_matrixTransform.valid())
    {
        osg::Matrix mat = _matrixTransform->getMatrix();

        mat =  
            osg::Matrix::translate( -_rotationOffset ) *
            osg::Matrix::rotate(_rotation[0], osg::Vec3d(1,0,0),
                                _rotation[1], osg::Vec3d(0,1,0),
                                _rotation[2], osg::Vec3d(0,0,1)) *                                
            osg::Matrix::translate( _rotationOffset ) *
            osg::Matrix::translate( _translation) *
            osg::Matrix::translate( _baseTrans);
        _matrixTransform->setMatrix( mat );        
    }
    else if (_dof.valid())
    {
        _dof->setPutMatrix( osg::Matrix::translate( _rotationOffset ) );
        _dof->setCurrentTranslate( _translation );
        _dof->setCurrentHPR( _rotation );
    }        
}