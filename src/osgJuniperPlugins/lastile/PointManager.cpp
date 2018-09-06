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


PointGeometry::PointGeometry(unsigned int initialSize):
    _initialSize(initialSize)
{
    setUseVertexBufferObjects(true);
    setUseDisplayList(false);
    setDataVariance(osg::Object::DYNAMIC);

    osg::MultiDrawArraysIndirect* drawArrays = new osg::MultiDrawArraysIndirect(osg::PrimitiveSet::POINTS);
    _commands = new osg::DefaultIndirectCommandDrawArrays();
    drawArrays->setIndirectCommandArray(_commands);
    _commands->setDataVariance(osg::Object::DYNAMIC);

    GLenum usage = GL_DYNAMIC_DRAW;

    _verts = new osg::Vec3Array(initialSize);
    _verts->setDataVariance(osg::Object::DYNAMIC);
    {
        osg::ref_ptr<osg::BufferObject> vbo = new osg::VertexBufferObject();
        vbo->setUsage(usage);
        _verts->setBufferObject(vbo.get());
    }
    setVertexArray(_verts);

    _colors = new osg::Vec4Array(initialSize);
    _colors->setDataVariance(osg::Object::DYNAMIC);
    {
        osg::ref_ptr<osg::BufferObject> vbo = new osg::VertexBufferObject();
        vbo->setUsage(usage);
        _colors->setBufferObject(vbo.get());
    }
    setColorArray(_colors, osg::Array::BIND_PER_VERTEX);

    _data = new osg::Vec4Array(initialSize);
    _data->setDataVariance(osg::Object::DYNAMIC);
    {
        osg::ref_ptr<osg::BufferObject> vbo = new osg::VertexBufferObject();
        vbo->setUsage(usage);
        _data->setBufferObject(vbo.get());
    }

    setVertexAttribArray(osg::Drawable::ATTRIBUTE_6, _data);
    setVertexAttribBinding(osg::Drawable::ATTRIBUTE_6, osg::Geometry::BIND_PER_VERTEX);
    setVertexAttribNormalize(osg::Drawable::ATTRIBUTE_6, false);

    addPrimitiveSet(drawArrays);
}

PointManager::PointManager()
{
    setNumChildrenRequiringUpdateTraversal(1);

    _transform = new osg::MatrixTransform;
    addChild(_transform);

    addPointGeometry();    
}

void PointManager::addPointGeometry()
{    
    PointGeometry* geom = new PointGeometry(1000000);
    _geometries.push_back(geom);
    unsigned int blockSize = 5000;
    for (unsigned int i = 0; i < geom->_initialSize; i += blockSize)
    {
        _freeBlocks.push_back(Block(i, blockSize, geom));
    }
    _transform->addChild(geom);
    OSG_NOTICE << "Added new PointGeometry size=" << _geometries.size() << std::endl;
}

void PointManager::traverse(osg::NodeVisitor& nv)
{
    if (nv.getVisitorType() == osg::NodeVisitor::UPDATE_VISITOR)
    {
        
        if (_chunksToAdd.size() > 0)
        {
            unsigned int numPoints = 0;
            for (unsigned int i = 0; i < _chunksToAdd.size(); ++i)
            {
                numPoints += _chunksToAdd[i]->points.size();
                addPointsInternal(_chunksToAdd[i].get());
            }
            OSG_NOTICE << "Adding " << _chunksToAdd.size() << " chunks with " << numPoints << std::endl;
            _chunksToAdd.clear();
        }        
    }
    else if (nv.getVisitorType() == osg::NodeVisitor::CULL_VISITOR)
    {
        osgUtil::CullVisitor* cv = nv.asCullVisitor();
        osg::ref_ptr< TraversedChunks > chunks = new TraversedChunks;
        cv->setUserData(chunks.get());
        osg::Group::traverse(nv);
        //OSG_NOTICE << "Traversed " << chunks->chunks.size() << " chunks" << std::endl;
        for (unsigned int i = 0; i < _geometries.size(); i++)
        {
            _geometries[i]->_commands->clear();
            _geometries[i]->_commands->dirty();
        }

        for (unsigned int i = 0; i < chunks->chunks.size(); i++)
        {
            for (unsigned int j = 0; j < chunks->chunks[i].get()->commands.size(); j++)
            {
                PointGeometry* g = static_cast<PointGeometry*>(chunks->chunks[i].get()->blocks[j].geometry);
                g->_commands->push_back(chunks->chunks[i]->commands[j]);
                g->_commands->dirty();
            }            
        }
    }
    osg::Group::traverse(nv);
}

void PointManager::addPointsInternal(PointChunk* chunk)
{
    OpenThreads::ScopedLock<OpenThreads::Mutex> lk(_mutex);

    unsigned int numExpired = 0;
    for (PointChunkList::iterator itr = _chunks.begin(); itr != _chunks.end();)
    {
        if (itr->get()->referenceCount() == 1)
        {
            // Free the blocks
            for (unsigned int i = 0; i < itr->get()->blocks.size(); i++)
            {
                _freeBlocks.push_back(itr->get()->blocks[i]);
            }
            itr = _chunks.erase(itr);
            numExpired++;
        }
        else
        {
            itr++;
        }
    }

    if (numExpired > 0)
    {
        OSG_NOTICE << "Expired " << numExpired << " chunks" << std::endl;
    }
    //OSG_NOTICE << "Free blocks " << _freeBlocks.size() << std::endl;

    // Get enough free blocks to hold space for this chunk of points
    BlockList blocks;
    unsigned int numAllocated = 0;
    while (numAllocated <= chunk->points.size())
    {
        if (_freeBlocks.empty())
        {
            addPointGeometry();
        }
        Block block = _freeBlocks.back();
        _freeBlocks.pop_back();        
        chunk->blocks.push_back(block);
        numAllocated += block.count;
    }
    //OSG_NOTICE << "Allocated " << numAllocated << " for " << chunk->points.size() << std::endl;

    // Add all the points in the correct locations based on the blocks
    unsigned blockIndex = 0;
    unsigned int count = 0;
    for (PointList::const_iterator itr = chunk->points.begin(); itr != chunk->points.end(); ++itr)
    {
        // Get the block we're adding to
        Block& block = chunk->blocks[blockIndex];
        unsigned int index = block.index + count;
        PointGeometry* geom = static_cast<PointGeometry*>(block.geometry);

        if (index >= geom->_verts->size())
        {
            OSG_NOTICE << "Index " << index << " out of range" << std::endl;
        }

        (*geom->_verts)[index] = osg::Vec3d(itr->x, itr->y, itr->z) - _anchor;
        (*geom->_colors)[index] = osg::Vec4(itr->r / 65536.0, itr->g / 65536.0, itr->b / 65536.0, itr->a / 65536.0);
        osg::Vec4 data;
        data.x() = itr->classification;
        data.y() = itr->returnNumber;
        data.z() = itr->intensity;
        (*geom->_data)[index] = data;
        count++;

        geom->_verts->dirty();
        geom->_colors->dirty();
        geom->_data->dirty();

        if (count >= block.count)
        {
            blockIndex++;
            osg::DrawArraysIndirectCommand cmd;
            cmd.count = count;
            cmd.instanceCount = 1;
            cmd.first = block.index;
            chunk->commands.push_back(cmd);
            geom->_verts->getVertexBufferObject()->dirtyRange(block.index * sizeof(float) * 3, count * sizeof(float) * 3);
            geom->_colors->getVertexBufferObject()->dirtyRange(block.index * sizeof(float) * 4, count * sizeof(float) * 4);
            geom->_data->getVertexBufferObject()->dirtyRange(block.index * sizeof(float) * 4, count * sizeof(float) * 4);
            count = 0;
        }
    }

    if (count > 0)
    {
        Block& block = chunk->blocks[blockIndex];
        PointGeometry* geom = static_cast<PointGeometry*>(block.geometry);
        //OSG_NOTICE << "Adding last command with " << count << " points" << std::endl;
        osg::DrawArraysIndirectCommand cmd;
        cmd.count = count;
        cmd.instanceCount = 1;
        cmd.first = block.index;
        chunk->commands.push_back(cmd);
        geom->_verts->getVertexBufferObject()->dirtyRange(block.index * sizeof(float) * 3, count * sizeof(float) * 3);
        geom->_colors->getVertexBufferObject()->dirtyRange(block.index * sizeof(float) * 4, count * sizeof(float) * 4);
        geom->_data->getVertexBufferObject()->dirtyRange(block.index * sizeof(float) * 4, count * sizeof(float) * 4);
    }
    
    ///create indirect command
    _chunks.push_back(chunk);    
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
    //OSG_NOTICE << "Adding chunk " << chunk->points.size() << " to list" << std::endl;
    _chunksToAdd.push_back(chunk);
}