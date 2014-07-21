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

#include <osgJuniperMap/Registry>
#include <osgDB/ReadFile>

#include <algorithm>
#include <sstream>


using namespace osgJuniper::Map;


Registry::Registry()
{
}

Registry::~Registry()
{
}

Registry*
Registry::instance()
{
    static osg::ref_ptr< Registry > s_registry = new Registry();
    return s_registry.get();
}

void
Registry::addPrimitiveFactory(osgJuniper::Map::PrimitiveFactory *factory, bool first)
{
    if (factory==0L) return;
    OpenThreads::ScopedLock< OpenThreads::Mutex > lock(_primitiveFactoriesMutex);
    if (first)
    {
        _primitiveFactories.insert(_primitiveFactories.begin(), factory );
    }
    else
    {
        _primitiveFactories.push_back( factory );
    }
}

void
Registry::removePrimitiveFactory(PrimitiveFactory *factory)
{
    if (factory==0L) return;
    OpenThreads::ScopedLock< OpenThreads::Mutex > lock(_primitiveFactoriesMutex);    
    PrimitiveFactoryList::iterator itr = std::find(_primitiveFactories.begin(), _primitiveFactories.end(), factory);
    if (itr != _primitiveFactories.end())
    {
        _primitiveFactories.erase( itr );
    }
}

Primitive*
Registry::createPrimitive(const std::string& type, PrimitiveId id)
{
    OpenThreads::ScopedLock< OpenThreads::Mutex > lock(_primitiveFactoriesMutex);   
    for (PrimitiveFactoryList::iterator itr = _primitiveFactories.begin(); itr != _primitiveFactories.end(); ++itr)
    {
        Primitive* prim = itr->get()->create(type, id);
        if (prim) return prim;
    }
    return NULL;
}


osg::Image*
Registry::getImageForObject( __int64 id, bool cache/*=true*/, const std::string& plugin/*=""*/ )
{
    std::string ext = _defaultImagePlugin;
    if (!plugin.empty()) ext = plugin;

    std::stringstream buf;
    buf << id << "." << ext;
    osg::ref_ptr< osgDB::Options >  opt;
    if (cache)
    {
        opt = new osgDB::ReaderWriter::Options;
        opt->setObjectCacheHint(osgDB::ReaderWriter::Options::CACHE_IMAGES);        
    }    
    return osgDB::readImageFile( buf.str(), opt.get() );
}

const std::string&
Registry::getDefaultImagePlugin() const
{
    return _defaultImagePlugin;
}

void
Registry::setDefaultImagePlugin( const std::string &defaultImagePlugin )
{
    _defaultImagePlugin = defaultImagePlugin;
}