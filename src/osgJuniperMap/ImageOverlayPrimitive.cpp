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

#include <osgJuniperMap/ImageOverlayPrimitive>
#include <osgJuniperMap/Registry>

#include <osg/ImageStream>

#include <osgDB/ReadFile>

#include <osg/io_utils>


using namespace osgEarth;
using namespace osgEarth::Annotation;
using namespace osgJuniper::Map;

REGISTER_SIMPLE_PRIMITIVE_FACTORY(ImageOverlayPrimitive)
REGISTER_SIMPLE_PRIMITIVE_FACTORY_TYPE(ImageOverlayPrimitive, ImageOverlay)

/***************************************************************************/
ImageOverlayPrimitive::ImageOverlayPrimitive(PrimitiveId id):
Primitive(id),
_edit(false),
_alpha(1.0f)
{
}



const std::string&
ImageOverlayPrimitive::getFilename() const
{
    return _filename;
}

void
ImageOverlayPrimitive::setFilename(const std::string& filename)
{
    if (_filename != filename)
    {
        _filename = filename;
        _image = osgDB::readImageFile( filename );
        osg::ImageStream* is = dynamic_cast<osg::ImageStream*>(_image.get());
        if (is)
        {
            is->play();
        }
        if (_image.valid())
        {
            if (_overlay)
            {
                _overlay->setImage( _image.get() );
            }
        }
    }
}

osg::Image*
ImageOverlayPrimitive::getImage() const
{
    return _image;
}

void
ImageOverlayPrimitive::setImage( osg::Image* image )
{
    _image = image;
    if (_overlay)
    {
        _overlay->setImage( _image.get() );
    }
}

bool
ImageOverlayPrimitive::getEdit() const
{
    return _edit;
}

void
ImageOverlayPrimitive::setEdit(bool edit)
{
    if (_edit != edit)
    {
        _edit = edit;
        updateEditorVisibility();        
    }
}

void
ImageOverlayPrimitive::updateEditorVisibility()
{
    if (_editor.valid())
    {
        _editor->setNodeMask( _edit ? ~0x0 : 0x0);
    }
}

void
ImageOverlayPrimitive::setMapContext(MapContext* context)
{
    Primitive::setMapContext(context);

    OSG_NOTICE << "ImageOverlayPrimitive::setMapContext" << std::endl;

    if (_editor.valid())
    {
        _context->getRoot()->removeChild(_editor.get());
        _editor = NULL;
    }

      
    osgEarth::MapNode* mapNode = _context->getMapNode();
    if (mapNode)
    {
        OSG_NOTICE << "creating imageoverlay" << std::endl;
        //Remove all the children and create a new overlay
        removeChildren(0, getNumChildren());

        _overlay = new ImageOverlay( mapNode, _image.get());
        _overlay->setImage( _image.get());
        _overlay->setLowerLeft(_lowerLeft.x(), _lowerLeft.y());
        _overlay->setLowerRight(_lowerRight.x(), _lowerRight.y());
        _overlay->setUpperRight(_upperRight.x(), _upperRight.y());
        _overlay->setUpperLeft(_upperLeft.x(), _upperRight.y());
        _overlay->setAlpha( _alpha );

        addChild(_overlay);


        _editor = new ImageOverlayEditor( _overlay.get());
        updateEditorVisibility();
        _context->getRoot()->addChild( _editor.get() );
    }        
    else
    {
        OSG_NOTICE << "couldn't find mapnode" << std::endl;
    }
}



void
ImageOverlayPrimitive::setProperty(const std::string& name, double value)
{    
    if (name == "alpha")
    {
        if (_alpha != (float)value)
        {
            _alpha = (float)value;
            _overlay->setAlpha(_alpha);
        }        
    }
    else
    {
        Primitive::setProperty(name, value);
    }
}

void
ImageOverlayPrimitive::setProperty(const std::string& name, int value)
{
    if (name == PROP_IMAGEOBJECT)
    {
        setImage( osgJuniper::Map::Registry::instance()->getImageForObject( value ) );
    }
    else
    {
        Primitive::setProperty( name, value );
    }
}

void
ImageOverlayPrimitive::setProperty(const std::string &name, const LocationList &locations)
{
    if (name.compare("pointlist") == 0)
    {
        if (locations.size() != 4)
        {
            osg::notify(osg::NOTICE) << "Error:  pointlist requires 4 locations for ImageOverlay" << std::endl;
            return;
        }

        _lowerLeft  = osg::Vec2d(locations[0].getLongitude(), locations[0].getLatitude());
        _lowerRight = osg::Vec2d(locations[1].getLongitude(), locations[1].getLatitude());
        _upperRight = osg::Vec2d(locations[2].getLongitude(), locations[2].getLatitude());
        _upperLeft  = osg::Vec2d(locations[3].getLongitude(), locations[3].getLatitude());

        if (_overlay.valid())
        {
            _overlay->setLowerLeft(_lowerLeft.x(), _lowerLeft.y());
            _overlay->setLowerRight(_lowerRight.x(), _lowerRight.y());
            _overlay->setUpperRight(_upperRight.x(), _upperRight.y());
            _overlay->setUpperLeft(_upperLeft.x(), _upperRight.y());
        }
    }
    else
    {
        Primitive::setProperty(name, locations);
    }
}

void
ImageOverlayPrimitive::setProperty(const std::string& name, const std::string& value)
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
ImageOverlayPrimitive::setProperty(const std::string& name, bool value)
{
    if (name == "edit")
    {
        setEdit( value );
    }
    else
    {
        Primitive::setProperty(name, value);
    }
}