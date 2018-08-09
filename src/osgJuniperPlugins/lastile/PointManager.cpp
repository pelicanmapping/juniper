#include "PointManager"
#include <osgUtil/CullVisitor>

using namespace osgJuniper;


PointChunk::PointChunk() :
    osg::Node()
{
}

osg::BoundingSphere PointChunk::computeBound() const
{
    osg::BoundingSphere bs;
    for (PointList::const_iterator itr = points.begin(); itr != points.end(); ++itr)
    {
        //bs.expandBy(osg::Vec3d(itr->x, itr->y, itr->z) - manager->getAnchor());
        bs.expandBy(osg::Vec3d(itr->x, itr->y, itr->z));
    }
    //OSG_NOTICE << "PointChunk::computeBound count= " << points.size() << " " << bs.center().x() << ", " << bs.center().y() << ", " << bs.center().z() << " radius=" << bs.radius() << std::endl;
    return bs;
}

void PointChunk::traverse(osg::NodeVisitor& nv)
{
    if (nv.getVisitorType() == osg::NodeVisitor::CULL_VISITOR)
    {
        TraversedChunks* chunks = dynamic_cast<TraversedChunks*>(nv.getUserData());
        if (chunks)
        {
            chunks->chunks.push_back(this);
        }        
    }
    osg::Node::traverse(nv);
}

PointManager::PointManager()
{
    setNumChildrenRequiringUpdateTraversal(1);

    osg::Geometry* geom = new osg::Geometry;

    geom->setUseVertexBufferObjects(true);
    geom->setUseDisplayList(false);
    geom->setDataVariance(osg::Object::DYNAMIC);

    osg::MultiDrawArraysIndirect* drawArrays = new osg::MultiDrawArraysIndirect(osg::PrimitiveSet::POINTS);
    _commands = new osg::DefaultIndirectCommandDrawArrays();
    drawArrays->setIndirectCommandArray(_commands);
    _commands->setDataVariance(osg::Object::DYNAMIC);

    GLenum usage = GL_DYNAMIC_DRAW;

    _verts = new osg::Vec3Array();
    _verts->setDataVariance(osg::Object::DYNAMIC);
    {
        osg::ref_ptr<osg::BufferObject> vbo = new osg::VertexBufferObject();
        vbo->setUsage(usage);
        _verts->setBufferObject(vbo.get());
    }
    geom->setVertexArray(_verts);

    _colors = new osg::Vec4Array;
    _colors->setDataVariance(osg::Object::DYNAMIC);
    {
        osg::ref_ptr<osg::BufferObject> vbo = new osg::VertexBufferObject();
        vbo->setUsage(usage);
        _colors->setBufferObject(vbo.get());
    }
    geom->setColorArray(_colors, osg::Array::BIND_PER_VERTEX);

    _data = new osg::Vec4Array;
    _data->setDataVariance(osg::Object::DYNAMIC);
    {
        osg::ref_ptr<osg::BufferObject> vbo = new osg::VertexBufferObject();
        vbo->setUsage(usage);
        _data->setBufferObject(vbo.get());
    }

    geom->setVertexAttribArray(osg::Drawable::ATTRIBUTE_6, _data);
    geom->setVertexAttribBinding(osg::Drawable::ATTRIBUTE_6, osg::Geometry::BIND_PER_VERTEX);
    geom->setVertexAttribNormalize(osg::Drawable::ATTRIBUTE_6, false);

    geom->addPrimitiveSet(drawArrays);

    _transform = new osg::MatrixTransform;
    _transform->addChild(geom);
    addChild(_transform);
}

void PointManager::traverse(osg::NodeVisitor& nv)
{
    if (nv.getVisitorType() == osg::NodeVisitor::UPDATE_VISITOR)
    {
        if (_chunksToAdd.size() > 0)
        {
            for (unsigned int i = 0; i < _chunksToAdd.size(); ++i)
            {
                addPointsInternal(_chunksToAdd[i].get());
            }
        }
        _chunksToAdd.clear();
    }
    else if (nv.getVisitorType() == osg::NodeVisitor::CULL_VISITOR)
    {
        osgUtil::CullVisitor* cv = nv.asCullVisitor();
        osg::ref_ptr< TraversedChunks > chunks = new TraversedChunks;
        cv->setUserData(chunks.get());
        osg::Group::traverse(nv);
        //OSG_NOTICE << "Traversed " << chunks->chunks.size() << " chunks" << std::endl;
        _commands->clear();
        for (unsigned int i = 0; i < chunks->chunks.size(); i++)
        {
            _commands->push_back(chunks->chunks[i].get()->cmd);
        }
        //OSG_NOTICE << "Got " << _commands->size() << " commands" << std::endl;
        _commands->dirty();
    }
    osg::Group::traverse(nv);
}

/*
osg::BoundingSphere PointManager::computeBound() const
{
    OpenThreads::ScopedLock<OpenThreads::Mutex> lk(const_cast<PointManager*>(this)->_mutex);
    osg::BoundingSphere bs;
    for (unsigned int i = 0; i < _verts->size(); i++)
    {
        bs.expandBy((*_verts)[i]);
    }
    return bs;
}
*/

void PointManager::addPointsInternal(PointChunk* chunk)
{
    OpenThreads::ScopedLock<OpenThreads::Mutex> lk(_mutex);
    ///create indirect command
    osg::DrawArraysIndirectCommand cmd;
    cmd.count = chunk->points.size();
    cmd.instanceCount = 1;
    cmd.first = _verts->size();
    for (PointList::const_iterator itr = chunk->points.begin(); itr != chunk->points.end(); ++itr)
    {        
        _verts->push_back(osg::Vec3d(itr->x, itr->y, itr->z) - _anchor);
        _colors->push_back(osg::Vec4(itr->r / 65536.0, itr->g / 65536.0, itr->b / 65536.0, itr->a / 65536.0));
        osg::Vec4 data;
        data.x() = itr->classification;
        data.y() = itr->returnNumber;
        data.z() = itr->intensity;
        _data->push_back(data);
    }
    // Clear the points, we don't need them anymore in the chunk.
    //chunk->points.clear();
    chunk->cmd = cmd;
    chunk->index = _commands->size();
    _chunks.push_back(chunk);
    _commands->push_back(cmd);
    _verts->dirty();
    _colors->dirty();
    _commands->dirty();
    _data->dirty();
    dirtyBound();
}

const osg::Vec3d& PointManager::getAnchor()
{
    return _anchor;
}

void PointManager::setAnchor(const osg::Vec3d& anchor)
{
    _anchor = anchor;
    _transform->setMatrix(osg::Matrixd::translate(_anchor));
}

void PointManager::addPoints(PointChunk* chunk)
{
    OpenThreads::ScopedLock<OpenThreads::Mutex> lk(_mutex);
    _chunksToAdd.push_back(chunk);
}


void PointManager::removePoints(PointChunk* chunk)
{
    OpenThreads::ScopedLock<OpenThreads::Mutex> lk(_mutex);
    _commands->erase(_commands->begin() + chunk->index);
    _chunks.erase(_chunks.begin() + chunk->index);
    _commands->dirty();
}