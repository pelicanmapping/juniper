#include <osgJuniperMap/Primitive>
#include <osg/Notify>
#include <sstream>

using namespace osgJuniper::Map;

Primitive::PrimitiveTerrainChangedCallback::PrimitiveTerrainChangedCallback( Primitive * primitive ):
_primitive( primitive )
{
}

void
Primitive::PrimitiveTerrainChangedCallback::onTileAdded(const osgEarth::TileKey& tileKey, osg::Node* terrain, osgEarth::TerrainCallbackContext&)
{
    if (_primitive)
    {
        _primitive->terrainChanged( tileKey, terrain );
    }
}  

/***************************************************************************/
Primitive::Primitive(PrimitiveId id):
osg::Group(),
_id(id),
_minRange(0.0f),
_maxRange(FLT_MAX),
_enableClusterCulling(true),
_visible(true),
_mainGroup(NULL),
_lod(NULL),
_primType(TYPE_GEOLOCATED),
_renderOrder(-1)
{    
    //Automatically set the name
    setName(getIDAsString());

    _lod = new osg::LOD;
    _mainGroup = new osg::Group;

    _lod->addChild( _mainGroup, _minRange, _maxRange);
    osg::Group::addChild(_lod);
}

Primitive::~Primitive()
{
    if (_context.valid() && _terrainChangedCallback.valid())
    {
        _context->getMapNode()->getTerrain()->removeTerrainCallback( _terrainChangedCallback.get() );
        _terrainChangedCallback = 0;
    }
}

PrimitiveId
Primitive::getID() const
{
    return _id;
}

std::string
Primitive::getIDAsString() const
{
    std::stringstream ss;
    ss << _id;
    return ss.str();
}

void
Primitive::setPrimitiveType(PrimitiveType type){
	_primType = type;
}

bool
Primitive::getVisible() const
{
    return _visible;
}

void
Primitive::setVisible(bool visible)
{
    _visible = visible;
    setNodeMask( _visible ? ~0 : 0);
}

void
Primitive::setMapContext(MapContext* context)
{
    //Unsubscribe the previous MapNode
    if (_context.valid() && _terrainChangedCallback.valid())
    {
        _context->getMapNode()->getTerrain()->removeTerrainCallback( _terrainChangedCallback.get() );
        _terrainChangedCallback = 0;
    }

    _context = context;

    //Subscribe to the new MapNode
    if (_context.valid() && getPrimitiveType() != Primitive::TYPE_SCREEN)
    {
        _terrainChangedCallback = new Primitive::PrimitiveTerrainChangedCallback( this );
        _context->getMapNode()->getTerrain()->addTerrainCallback( _terrainChangedCallback.get() );        

    }
}

float
Primitive::getMaxRange() const
{
    return _maxRange;
}

void
Primitive::setMaxRange(float maxRange)
{
    setRange(_minRange, maxRange);
}

float
Primitive::getMinRange() const
{
    return _minRange;
}

void
Primitive::setMinRange(float minRange)
{
    setRange(minRange, _maxRange);
}

void
Primitive::setRange(float minRange, float maxRange)
{
    if (_minRange != minRange || _maxRange != maxRange)
    {
        _minRange = minRange;
        _maxRange = maxRange;
        _lod->setRange(0, _minRange, _maxRange);
    }
}

int
Primitive::getRenderOrder() const
{
    return _renderOrder;
}

void
Primitive::setRenderOrder(int renderOrder)
{
    if (_renderOrder != renderOrder)
    {
        _renderOrder = renderOrder;
        /*
        if (_renderOrder < 0)
        {
            //Setting the render order to a negative number resets the state back to inherit
            getOrCreateStateSet()->setRenderBinToInherit();
            getOrCreateStateSet()->removeMode(GL_BLEND);
            getOrCreateStateSet()->removeMode(GL_DEPTH_TEST);
            getOrCreateStateSet()->setRenderingHint( osg::StateSet::DEFAULT_BIN );
        }
        else
        {
            getOrCreateStateSet()->setRenderBinDetails(renderOrder, "RenderBin");
            getOrCreateStateSet()->setMode(GL_DEPTH_TEST, osg::StateAttribute::OFF);
            //getOrCreateStateSet()->setRenderingHint( osg::StateSet::TRANSPARENT_BIN );
            getOrCreateStateSet()->setMode(GL_BLEND,osg::StateAttribute::ON);

        }
        */
        getOrCreateStateSet()->setRenderBinDetails(renderOrder, "RenderBin");
        getOrCreateStateSet()->setMode(GL_BLEND,osg::StateAttribute::ON);
    }
}

bool
Primitive::getEnableClusterCulling() const
{
    return _enableClusterCulling;
}

void
Primitive::setEnableClusterCulling(bool enableClusterCulling)
{
    _enableClusterCulling = enableClusterCulling;
}


osg::Vec4
Primitive::getColorFromDecimal(int decimal)
{
    float r = (float)(decimal & 0xff);
    float g = (float)((decimal >> 8) & 0xff);
    float b = (float)((decimal >> 16) & 0xff);

    return osg::Vec4(r/255.0f,g/255.0f,b/255.0f,1.0f);
}

const osg::EllipsoidModel*
Primitive::getWGS84Ellipsoid()
{
    static osg::ref_ptr< osg::EllipsoidModel > wgs84 = new osg::EllipsoidModel;
    return wgs84.get();
}

void
Primitive::setProperty(const std::string& name, bool   value)
{    
    if (name == PROP_SHOW)
    {
        setVisible( value );
    }
    else if (name == PROP_DEPTH_TEST)
    {
        getOrCreateStateSet()->setMode(GL_DEPTH_TEST, value ? osg::StateAttribute::ON  | osg::StateAttribute::OVERRIDE :
                                                              osg::StateAttribute::OFF | osg::StateAttribute::OVERRIDE );
    }
    else if (name == PROP_LIGHTING)
    {
        getOrCreateStateSet()->setMode(GL_LIGHTING, value ? osg::StateAttribute::ON  | osg::StateAttribute::OVERRIDE :
                                                              osg::StateAttribute::OFF | osg::StateAttribute::OVERRIDE );
    }
    else if (name == PROP_ENABLE_CLUSTER_CULLING)
    {
        setEnableClusterCulling( value );
    }
}

void
Primitive::setProperty(const std::string& name, double value)
{
    if (name == PROP_MAXRANGE)
    {
        setMaxRange(value);
    }
    else if (name == PROP_MINRANGE)
    {
        setMinRange(value);
    }
}

void
Primitive::setProperty(const std::string& name, int value)
{
    if (name == PROP_RENDER_ORDER)
    {
        setRenderOrder( value );
    }
}

bool
Primitive::addChild(osg::Node *child)
{
    return _mainGroup->addChild( child );
}

bool
Primitive::removeChildren(unsigned int pos, unsigned int numChildrenToRemove)
{
    return _mainGroup->removeChildren(pos, numChildrenToRemove);
}


/***************************************************************************/

PrimitiveManager::PrimitiveManager()
{
}

PrimitiveManager::~PrimitiveManager()
{
}

const Primitive*
PrimitiveManager::getPrimitive( PrimitiveId id ) const
{
    OpenThreads::ScopedLock< OpenThreads::Mutex > lock(const_cast<PrimitiveManager*>(this)->_mutex);
    PrimitiveMap::const_iterator itr = _primitives.find( id );
    return itr != _primitives.end() ? itr->second : NULL;
}

Primitive*
PrimitiveManager::getPrimitive( PrimitiveId id )
{
    OpenThreads::ScopedLock< OpenThreads::Mutex > lock(_mutex);
    PrimitiveMap::iterator itr = _primitives.find( id );
    return itr != _primitives.end() ? itr->second : NULL;
}

void
PrimitiveManager::addPrimitive(Primitive* primitive)
{
    OpenThreads::ScopedLock< OpenThreads::Mutex > lock(_mutex);
    _primitives[ primitive->getID() ] = primitive;
}

void
PrimitiveManager::removePrimitive(PrimitiveId id)
{
    OpenThreads::ScopedLock< OpenThreads::Mutex > lock(_mutex);
    PrimitiveMap::iterator itr = _primitives.find( id );
    if (itr != _primitives.end())
    {
        _primitives.erase( itr );
    }
}

void
PrimitiveManager::setMapContext(MapContext *context)
{
    OpenThreads::ScopedLock< OpenThreads::Mutex > lock(_mutex);
    for (PrimitiveMap::iterator itr = _primitives.begin(); itr != _primitives.end(); ++itr)
    {
        itr->second->setMapContext( context );
    }
}