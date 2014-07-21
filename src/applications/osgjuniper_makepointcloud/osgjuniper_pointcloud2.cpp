/* -*-c++-*- */
/* osgJuniper - Large Dataset Visualization Toolkit for OpenSceneGraph
 * Copyright 2010-2011 Pelican Ventures, Inc.
 * http://wush.net/trac/juniper
 *
 * osgEarth is free software; you can redistribute it and/or modify
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

#include <osg/Shape>
#include <osg/ShapeDrawable>
#include <osg/io_utils>
#include <osgDB/FileUtils>
#include <osgDB/FileNameUtils>

#include <osgText/Text>
#include <osg/Geode>
#include <osg/Group>
#include <osg/MatrixTransform>
#include <osg/Point>

#include <osgDB/ReadFile>
#include <osgDB/WriteFile>
#include <osgDB/FileUtils>
#include <osgDB/FileNameUtils>

#include <osgGA/StateSetManipulator>

#include <osgViewer/Viewer>
#include <osgViewer/ViewerEventHandlers>

#include <iostream>
#include <map>
#include <osgJuniper/Utils>
#include <osgJuniper/Octree>

#include <fstream>
#include <sstream>

using namespace osgJuniper;

class SimpleWriter : public osg::Referenced
{
public:
    SimpleWriter(const std::string &filename):
      _filename(filename)
    {
        _out.open( _filename.c_str(), std::ios::out | std::ios::binary);
    }

    void addPoint(const Point& point)
    {
        _out.write((char*)&point._position._v,sizeof(float) * 3);
    }

    std::ofstream _out;
    std::string _filename;
};




class ChooseRepresentativePointVisitor : public OctreeNodeVisitor
{
public:
    ChooseRepresentativePointVisitor():
      _valid(false)
    {
    }

    virtual void apply(OctreeNode& node)
    {
        if (!_valid)
        {            
            if (node.isLeaf() && node.getPoints().size() > 0) 
            {
                //Choose a representative point for this LOD
                unsigned int count = 1000;
                for (unsigned int i = 0; i < count; ++i)
                {
                    if (node.getPoints().size() > 0)
                    {
                        //Remove a random point
                        //unsigned int index = getRandomValueinRange(child->getPoints().size()-1);
                        _points.push_back( node.getPoints().front());                        
                        //Erase the point from the child
                        node.getPoints().pop_front();
                    }
                }
                _valid = true;
            }
            else
            {
                traverse( node );
            }
        }
    }

    PointList _points;
    bool _valid;
};

class ApplyRepresentativePointVisitor : public OctreeNodeVisitor
{
public:
    ApplyRepresentativePointVisitor()
    {
    }

    virtual void apply(OctreeNode& node)
    {
        if (!node.isLeaf() && node.getPoints().empty()) 
        {
            //This node is not a leaf and needs a representative point chosen from it's children
            ChooseRepresentativePointVisitor v;
            node.accept(v);
            if (v._valid)
            {
                //Add the point to this node
                for (PointList::iterator itr = v._points.begin(); itr != v._points.end(); ++itr)
                {
                    node.getPoints().push_back( *itr );
                }
            }    
        }
        traverse(node);
    }
};


const float RADIUS_FACTOR = 6.0f;

class CollectPointsVisitor : public OctreeNodeVisitor
{
public:
    CollectPointsVisitor():
      _remove(true)
    {
    }

    bool getRemove() const
    {
        return _remove;
    }

    void setRemove(bool remove)
    {
        _remove = remove;
    }

    virtual void apply(OctreeNode& node)
    {
        for (PointList::iterator itr = node.getPoints().begin(); itr != node.getPoints().end(); ++itr)
        {
            (*itr)._minRange = node.getBoundingBox().radius() * RADIUS_FACTOR;
            (*itr)._level    = node.getID().level;
            _points.push_back( *itr );
        }

        if (_remove)
        {
            //Erase all the points from the octree to free up memory
            node.getPoints().clear();
        }
        traverse(node);
    }

    bool _remove;
    PointList _points;
};


/****************************************************************************/

class Progress
{
public:
    Progress():
      _complete(0),
      _total(0)
    {
    }

    unsigned int getTotal()
    {
        OpenThreads::ScopedLock<OpenThreads::Mutex> lock(_mutex);
        return _total;
    }

    void setTotal(unsigned int total)
    {
        OpenThreads::ScopedLock<OpenThreads::Mutex> lock(_mutex);
        _total = total;
    }

    unsigned int getComplete()
    {
        OpenThreads::ScopedLock<OpenThreads::Mutex> lock(_mutex);
        return _complete;
    }

    bool isComplete()
    {
        OpenThreads::ScopedLock<OpenThreads::Mutex> lock(_mutex);
        return _complete == _total;
    }

    void incrementComplete(unsigned int complete)
    {
        OpenThreads::ScopedLock<OpenThreads::Mutex> lock(_mutex);
        _complete += complete;
    }



private:
    unsigned int _total;
    unsigned int _complete;
    OpenThreads::Mutex _mutex;
};

static Progress s_progress;

/****************************************************************************/

class SPTNode : public osg::Geode
{
public:
    SPTNode();
    void setPoints( PointList &points );

    virtual void traverse(osg::NodeVisitor &nv);

protected:
    osg::Geometry *_geometry;
    osg::Vec3Array* _verts;
    osg::Vec4Array* _colors;
    typedef std::map< float, int> RangeMap;
    RangeMap _rangeMap;
    osg::DrawArrays* _drawArrays;
};

struct SortPointsByRangeFunctor
{
    bool operator() (const Point& lhs,const Point& rhs) const
    {
        if (lhs._minRange > rhs._minRange) return true;
        else return false;
    }
};

/****************************************************************************/
SPTNode::SPTNode():
_geometry(0),
_verts(0),
_colors(0),
_drawArrays(0)
{
}

void SPTNode::setPoints(PointList &points)
{
    //Sort the incoming points by max range
    //std::sort(points.begin(), points.end(), SortPointsByRangeFunctor());
    points.sort(SortPointsByRangeFunctor());

    osg::BoundingSphere bounds;
    //for (unsigned int i = 0; i < points.size(); ++i)
    for (PointList::iterator itr = points.begin(); itr != points.end(); ++itr)
    {
        bounds.expandBy( (*itr)._position );
    }

    setInitialBound( bounds );

    //Remove the existing geometry if there is one
    if (_geometry) removeDrawable( _geometry );

    _geometry = new osg::Geometry;
    _geometry->setUseVertexBufferObjects( true );
    _geometry->setUseDisplayList( false );
    _geometry->setDataVariance(osg::Object::DYNAMIC);

    _verts = new osg::Vec3Array;
    _verts->reserve( points.size() );
    _geometry->setVertexArray( _verts );    

    _colors = new osg::Vec4Array();
    _colors->reserve( points.size() );
    _geometry->setColorArray( _colors );
    _geometry->setColorBinding( osg::Geometry::BIND_PER_VERTEX);

    typedef std::pair< int, float > CountRangePair;
    typedef std::map<int, CountRangePair > RangeIndex;
    RangeIndex rangeIndex;

    while (points.size() > 0)
    {     
        //TODO:  Fix
        _verts->push_back( points.front()._position );
        _colors->push_back( points.front()._color );
        if (rangeIndex.find(points.front()._level) == rangeIndex.end())
        {
            rangeIndex[points.front()._level] = CountRangePair(0, points.front()._minRange);
        }
        rangeIndex[points.front()._level].first += 1;
        points.pop_front();
    }

    //rangeIndex now contains a mapping from the level to the number of points at that range (and their range)
    //Increment all of the count values to include the previous levels as well
    unsigned int prevCount = 0;
    for (RangeIndex::iterator itr = rangeIndex.begin(); itr != rangeIndex.end(); ++itr)
    {
        (*itr).second.first += prevCount;
        prevCount = (*itr).second.first;
    }

    //Populate the range map with the new values
    for (RangeIndex::iterator itr = rangeIndex.begin(); itr != rangeIndex.end(); ++itr)
    {
        _rangeMap[(*itr).second.second] = (*itr).second.first;
    }



    //Now the range list 
    //_drawArrays = new osg::DrawArrays(GL_POINTS, 0, 0);
    _drawArrays = new osg::DrawArrays(GL_POINTS, 0, _verts->size());
    _geometry->addPrimitiveSet( _drawArrays );
    addDrawable( _geometry );
}

void
SPTNode::traverse(osg::NodeVisitor &nv)
{
    if (nv.getVisitorType() == osg::NodeVisitor::CULL_VISITOR)
    {
        float required_range = nv.getDistanceToViewPoint(getBound().center(),true);    
        RangeMap::iterator itr = _rangeMap.upper_bound(required_range);
        if (_geometry && _drawArrays && _verts)
        {
            unsigned int count = _verts->size();
            if (itr != _rangeMap.end())
            {
                count = itr->second;
            }
            /*unsigned int maxVerts = 2000000;
            count = osg::clampBetween(count, 0u, maxVerts);*/
            //osg::notify(osg::NOTICE) << "Drawing " << count << " for range " << required_range << std::endl;
            _drawArrays->setCount( count );
        }
    }
    osg::Geode::traverse(nv);
}


/****************************************************************************/
class CollectVertsVisitor : public osg::NodeVisitor
{
public:
    CollectVertsVisitor():
      osg::NodeVisitor( osg::NodeVisitor::TRAVERSE_ALL_CHILDREN )
      {
      }

      void apply(osg::Geode& node)
      {
          for (unsigned int i = 0; i < node.getNumDrawables(); ++i)
          {
              osg::Geometry* geom = node.getDrawable(i)->asGeometry();
              if (geom)
              {
                  osg::Vec3Array* verts  = dynamic_cast<osg::Vec3Array*>(geom->getVertexArray());
                  osg::Vec4Array* colors = dynamic_cast<osg::Vec4Array*>(geom->getColorArray());
                  if (verts && colors)
                  {
                      for (unsigned int j = 0; j < verts->size(); j++)
                      {                   
                          _points.push_back(Point((*verts)[j], osg::Vec3(0,0,1), (*colors)[j]));
                      }
                  }
              }
          }
      }
      PointList _points;
};

class RejectionFile : public osg::Referenced
{
public:
    RejectionFile(OctreeNode* node, const std::string& filename):
      _node(node),
      _filename(filename),
      _needsProcessed(true),
      _numPoints(0)
    {
    }

      void close()
      {
          if (_out.is_open()) _out.close();
      }

      const std::string& getFilename() { return _filename;}

      bool contains(osg::Vec3& point)
      {
          return _node->getBoundingBox().contains( point );
      }

      bool getNeedsProcessed() const { return _numPoints > 0 && _needsProcessed;}
      void setNeedsProcessed(bool needsProcessed) { _needsProcessed = needsProcessed;}

      OctreeNode* getNode() { return _node.get();}

      unsigned int getNumPoints() const { return _numPoints; }

      void addPoint(const Point& point)
      {
          _numPoints++;
          if (!_out.is_open())
          {
              _out.open( _filename.c_str(), std::ios::out | std::ios::binary);
          }

          osg::Vec3 color(point._color.r(), point._color.g(), point._color.b());
          color *= 255.0f;

          //No size yet, just write out 1.0f
          float size = 1.0f;
          _out.write((char*)&point._position._v, sizeof(float) * 3);
          _out.write((char*)&point._normal._v, sizeof(float) * 3);
          _out.write((char*)&color._v, sizeof(float) * 3);
          _out.write((char*)&size, sizeof(float));
      }

      void remove()
      {
          if (_out.is_open())
          {
              _out.close();
          }
          //OSG_NOTICE << "Removing " << _filename << std::endl;
          unsigned int result = ::remove(_filename.c_str());
          if (result != 0)
          {
              //OSG_NOTICE << "Could not remove file " << _filename.c_str() << result << std::endl;
          }
      }

    osg::ref_ptr< OctreeNode > _node;
    std::string _filename;
    unsigned int _numPoints;    
    std::ofstream _out;
    bool _needsProcessed;
};

class MakeSceneOp : public osg::Operation
{
public:
    MakeSceneOp(OctreeNode* node, const std::string& filename, unsigned int innerMaxLevel, unsigned int outerMaxLevel, osg::OperationQueue* queue, bool remove=false);

    void operator()(osg::Object*);

    osg::ref_ptr<OctreeNode> _node;
    osg::ref_ptr< osg::OperationQueue > _queue;
    std::string _filename;
    unsigned int _innerMaxLevel;
    unsigned int _outerMaxLevel;
    bool _remove;
};



class MakeSceneVisitor : public OctreeNodeVisitor
{
public:
    MakeSceneVisitor(const std::string &filename, unsigned int innerMaxLevel, unsigned int outerMaxLevel):
      _filename(filename),
      _innerMaxLevel(innerMaxLevel),
      _outerMaxLevel(outerMaxLevel)
    {
    }

      void setOperationQueue(osg::OperationQueue* queue)
      {
          _operationQueue = queue;
      }

    virtual void apply(OctreeNode& node)
    {
        //OSG_NOTICE << "Building scene from filename " << _filename << std::endl;
        if (!_pointSource.valid())
        {
            _pointSource = PointSource::loadPointSource( _filename );
        }

        osg::ref_ptr< PointCursor > pointCursor = _pointSource->createPointCursor();

        std::vector< osg::ref_ptr< RejectionFile > > rejectionFiles;
        rejectionFiles.reserve(8);
        for (unsigned int i = 0; i < 8; ++i)
        {
            OctreeNode* child = node.createChild( i );
            rejectionFiles.push_back( new RejectionFile(child, createRejectionFile(*child)));
        }

        //Create the inner octree which will be used to collect our points for this node in the outer octree
        OctreeNode innerOctree;
        innerOctree.setBoundingBox(node.getBoundingBox());
        int numPointsRead     = 0;
        int numPointsAdded    = 0;
        int numPointsRejected = 0;
        while (pointCursor->hasMore())
        {
            Point p;
            if (pointCursor->nextPoint(p))
            {
                numPointsRead++;
                AddPointVisitor apv(p, _innerMaxLevel);
                if (node.getID().level <= _outerMaxLevel)
                {
                    apv.setStrategy(AddPointVisitor::REJECT);
                }
                else
                {                    
                    OSG_NOTICE << "Reached max level of " << _outerMaxLevel << ", accepting all points" << std::endl;
                    //We can't break it down any further, just accept the points and move on
                    apv.setStrategy(AddPointVisitor::ACCEPT);
                }
                innerOctree.accept(apv);
                if (!apv.getPointAdded())
                {
                    bool addedPointToRejectionFile = false;
                    numPointsRejected++;
                    //Choose which rejection file to use and write the point to it
                    for (unsigned int i = 0; i < 8; ++i)
                    {
                        if (rejectionFiles[i]->contains(p._position))
                        {
                            rejectionFiles[i]->addPoint( p );
                            addedPointToRejectionFile = true;
                            break;
                        }
                    }
                    if (!addedPointToRejectionFile)
                    {
                        OSG_NOTICE << "Could not add point " << p._position << " to rejection file" << std::endl;
                        for (unsigned int i = 0; i < 8; ++i)
                        {
                            OSG_NOTICE << "Rejection file " << i << " " <<  rejectionFiles[i]->getNode()->getBoundingBox()._min << " to " << rejectionFiles[i]->getNode()->getBoundingBox()._max << std::endl;
                        }
                    }
                }
                else
                {
                    numPointsAdded++;
                }
            }
            //if (numPointsRead % 50000 == 0) OSG_NOTICE << "Read " << numPointsRead << " points..." << std::endl;
        }

        unsigned int rejectionThreshold = 10000;        
        for (unsigned int i = 0; i < 8; ++i)
        {
            //Close the rejection file
            rejectionFiles[i]->close();
            //OSG_NOTICE << "Rejection file " << rejectionFiles[i]->getFilename() << " contains " << rejectionFiles[i]->getNumPoints() << std::endl;
            //If the rejection file contains a small number of points, just add them to this node, recusing further won't help us
            if (rejectionFiles[i]->getNumPoints() > 0 && rejectionFiles[i]->getNumPoints() < rejectionThreshold)
            { 
                unsigned int numReadRejectionFile = 0;
                //OSG_NOTICE << "Adding " << rejectionFiles[i]->getNumPoints() << " from rejection file " << rejectionFiles[i]->getFilename() << " to inner octree b/c it has less points than the rejection threshold" << rejectionThreshold << std::endl;
                osg::ref_ptr< PointSource > rejectionSource = PointSource::loadPointSource( rejectionFiles[i]->getFilename() );
                osg::ref_ptr< PointCursor > rejectionCursor = rejectionSource->createPointCursor();                
                while (rejectionCursor->hasMore())
                {
                    Point p;
                    if (rejectionCursor->nextPoint(p))
                    {
                        numReadRejectionFile++;
                        AddPointVisitor apv(p, _innerMaxLevel);
                        apv.setStrategy(AddPointVisitor::ACCEPT);
                        innerOctree.accept(apv);
                        numPointsAdded++;
                        numPointsRejected--;
                    }
                }
                if (numReadRejectionFile != rejectionFiles[i]->getNumPoints())
                {
                    OSG_NOTICE << "Error:  Number of points in rejection file was expected " << std::endl;
                }
                //Delete the cursor and close the rejection file
                rejectionCursor = NULL;
                rejectionFiles[i]->setNeedsProcessed(false);    
                rejectionFiles[i]->remove();
            }
        }
        //OSG_NOTICE << "Read " << numPointsRead << " points  Added=" << numPointsAdded << " Rejected=" << numPointsRejected << std::endl;
        if (numPointsAdded + numPointsRejected != numPointsRead)
        {
            OSG_NOTICE << "Something wrong..." << std::endl;
        }

        //Write out this node
        ApplyRepresentativePointVisitor arpv;
        innerOctree.accept( arpv );

        CollectPointsVisitor pointCollector;
        innerOctree.accept( pointCollector );

        //Create the node to render the points for OctreeNode
        osg::ref_ptr<SPTNode> points = new SPTNode();
        points->setPoints( pointCollector._points );
        points->getOrCreateStateSet()->setAttributeAndModes(new osg::Point(1.1), osg::StateAttribute::ON);
        points->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF);    
        std::string nodeFilename = createURI(node);

        //Create a PagedLOD for this node
        osg::ref_ptr< osg::PagedLOD > pagedLOD = new osg::PagedLOD;
        double range = node.getBoundingBox().radius() * RADIUS_FACTOR;
        osg::Vec3 center = node.getBoundingBox().center();
        pagedLOD->setRadius(node.getBoundingBox().radius());
        pagedLOD->setCenter(center);
        pagedLOD->addChild(points.get());
        pagedLOD->setRange(0, 0, FLT_MAX);

        unsigned int numRejectionFilesAdded = 0;
        //Add any of the rejection files that still needs to be processed
        for (unsigned int i = 0; i < 8; ++i)
        {
            if (rejectionFiles[i]->getNeedsProcessed())
            {
                unsigned int position = numRejectionFilesAdded+1;
                numRejectionFilesAdded++;
                std::string uri = createURI( *rejectionFiles[i]->getNode());
                pagedLOD->setFileName(position, uri);
                pagedLOD->setRange(position, 0, rejectionFiles[i]->getNode()->getBoundingBox().radius() * RADIUS_FACTOR);
            }
        }
        std::string filename = createURI(node);
        //osg::notify(osg::NOTICE) << "Writing node" << filename << std::endl;
        osgDB::writeNodeFile(*pagedLOD, filename);

        //Increment the progress
        s_progress.incrementComplete(numPointsAdded);

        for (unsigned int i = 0; i < 8; ++i)
        {
            if (rejectionFiles[i]->getNeedsProcessed())
            {
                /*
                OSG_NOTICE << "Processing rejection file " << rejectionFiles[i]->getFilename() << " with " << rejectionFiles[i]->getNumPoints() << std::endl;
                MakeSceneVisitor msv(rejectionFiles[i]->getFilename(), _innerMaxLevel, _outerMaxLevel);
                rejectionFiles[i]->getNode()->accept( msv);
                //Remove this Rejection file
                rejectionFiles[i]->remove();*/
                _operationQueue->add( new MakeSceneOp(rejectionFiles[i]->getNode(), rejectionFiles[i]->getFilename(), _innerMaxLevel, _outerMaxLevel, _operationQueue.get(), true));
            }
        }
    }

    std::string createRejectionFile( OctreeNode& node)
    {
        std::stringstream ss;
        ss << "R_" << node.getID().level << "_" 
                   << node.getID().x << "_"
                   << node.getID().y << "_"
                   << node.getID().z << ".spawar_points";
        return ss.str();
    }

    std::string createURI( OctreeNode& node)
    {
        std::stringstream ss;
        ss << node.getID().level << "_" 
           << node.getID().x << "_"
           << node.getID().y << "_"
           << node.getID().z << ".ive";
        return ss.str();
    }


    osg::ref_ptr< osg::OperationQueue > _operationQueue;
    std::string _filename;

    osg::ref_ptr< PointSource> _pointSource;
    unsigned int _innerMaxLevel;
    unsigned int _outerMaxLevel;
};



MakeSceneOp::MakeSceneOp(OctreeNode* node, const std::string& filename, unsigned int innerMaxLevel, unsigned int outerMaxLevel, osg::OperationQueue* queue, bool remove):
_node(node),
_filename(filename),
_innerMaxLevel(innerMaxLevel),
_outerMaxLevel(outerMaxLevel),
_queue(queue),
_remove(remove)
{
}

void
MakeSceneOp::operator ()(osg::Object *)
{
    {
        MakeSceneVisitor msv(_filename, _innerMaxLevel, _outerMaxLevel);
        msv.setOperationQueue( _queue.get() );
        _node->accept(msv);
    }

    if (_remove)
    {
        unsigned int result = ::remove(_filename.c_str());
        if (result != 0)
        {
            OSG_NOTICE << "Could not remove file " << _filename.c_str() << result << std::endl;
        }
    }
}

osg::Node* makeSceneFromPointSource(PointSource *source)
{
    //Create the initial cursor
    osg::ref_ptr< PointCursor > cursor = source->createPointCursor();

    unsigned int maxVerts = 10000;

    unsigned int numRead = 0;
    //Initialize the bounding box from the points
    osg::BoundingBoxd bb;
    while (cursor->hasMore() && numRead < maxVerts)
    {
        Point p;
        if (cursor->nextPoint(p))
        {
            bb.expandBy( p._position );
            numRead++;
            if (numRead%1000 == 0) 
                OSG_NOTICE << "Expanded by " << numRead+1 << " points " << std::endl;
        }
    }

   
    OctreeNode* root = new OctreeNode();
    root->setBoundingBox( bb );    

    unsigned int maxLevel = 4;

    //Reset the cursor
    numRead = 0;
    cursor = source->createPointCursor();
    while (cursor->hasMore() && numRead < maxVerts)
    {
        Point p;
        if (cursor->nextPoint(p))
        {
            AddPointVisitor apv(p, maxLevel);
            root->accept(apv);
            if (numRead%1000 == 0) 
                OSG_NOTICE << "Added " << numRead+1 << " points " << std::endl;
            numRead++;
        }
    }

    osg::notify(osg::NOTICE) << "Added a total of " << numRead << " points" << std::endl;
    
    ApplyRepresentativePointVisitor arpv;
    root->accept( arpv );

    CollectPointsVisitor pointCollector;
    root->accept( pointCollector );

    SPTNode* points = new SPTNode();
    points->setPoints( pointCollector._points );
    points->getOrCreateStateSet()->setAttributeAndModes(new osg::Point(1.1), osg::StateAttribute::ON);
    points->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF);    

    return points;

}


osg::Node* makeSceneFromPoints(PointList &in_points)
{
    //Initialize the bounding box from the points
    osg::BoundingBoxd bb;
    for (PointList::const_iterator itr = in_points.begin(); itr != in_points.end(); ++itr)
    {
        bb.expandBy( (*itr)._position );
    }

    
    OctreeNode* root = new OctreeNode();
    root->setBoundingBox( bb );    

    unsigned int maxLevel = 8;
    unsigned int i = 0;
    while (in_points.size() > 0)
    {
        AddPointVisitor apv(in_points.front(), maxLevel);
        root->accept(apv);
        if (i%1000 == 0) 
            OSG_NOTICE << "Added " << i+1 << " points " << std::endl;
        i++;
        in_points.pop_front();
    }
    
    ApplyRepresentativePointVisitor arpv;
    root->accept( arpv );

    CollectPointsVisitor pointCollector;
    root->accept( pointCollector );

    SPTNode* points = new SPTNode();
    points->setPoints( pointCollector._points );
    points->getOrCreateStateSet()->setAttributeAndModes(new osg::Point(1.1), osg::StateAttribute::ON);
    points->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF);    

    return points;
}


osg::Node* makeScene()
{
    osg::ref_ptr<OctreeNode> root = new OctreeNode();
    root->setBoundingBox(osg::BoundingBoxd(osg::Vec3(-1,-1,-1), osg::Vec3(1,1,1)));    
    
    unsigned int numVerts = 6000000;
    //Generate a bunch of random points
    unsigned int maxLevel = 8;
    for (unsigned int i = 0; i < numVerts; ++i)
    {
        AddPointVisitor apv(Point(Utils::randomVert(), osg::Vec3(0,0,1), Utils::randomColor()), maxLevel);
        root->accept(apv);
        if (i%1000 == 0) OSG_NOTICE << "Added " << i+1 << " of " << numVerts << " points " << std::endl;
    }

    {
        CountPointsVisitor cpv;
        root->accept(cpv);
        osg::notify(osg::NOTICE) << "Added " << numVerts << "  scene contains " << cpv._numPoints << std::endl;
    }

    ApplyRepresentativePointVisitor arpv;
    root->accept( arpv );

    {
        CountPointsVisitor cpv;
        root->accept(cpv);
        osg::notify(osg::NOTICE) << "With rep points, should be " << numVerts << "  scene contains " << cpv._numPoints << std::endl;
    }

    CollectPointsVisitor pointCollector;
    root->accept( pointCollector );

    SPTNode* points = new SPTNode();
    points->setPoints( pointCollector._points );
    points->getOrCreateStateSet()->setAttributeAndModes(new osg::Point(2.0f), osg::StateAttribute::ON);
    points->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF);    

    return points;
}

void makeSimple(const std::string& input, const std::string& output)
{
    //Open the source file
    osg::ref_ptr< PointSource > inputSource = PointSource::loadPointSource(input);
    osg::ref_ptr< PointCursor > inputCursor = inputSource->createPointCursor();

    osg::ref_ptr< SimpleWriter > writer = new SimpleWriter(output);
    while (inputCursor->hasMore())
    {
        Point p;
        if (inputCursor->nextPoint( p ))
        {
            writer->addPoint( p );
        }
    }
}

class MakeSimpleOp : public osg::Operation
{
public:
    MakeSimpleOp(const std::string& input, const std::string& output):
      _input(input),
      _output(output)
    {
    }

    void operator()(osg::Object*)
    {
        OSG_NOTICE << "Converting " << _input << " to " << _output << std::endl;
        makeSimple(_input, _output);
    }

    std::string _input;
    std::string _output;
};


PointSource* makePointSourceFromDirectory( const std::string& directory)
{
    osg::notify(osg::NOTICE) << "Loading directory " << directory << std::endl;
    osgDB::DirectoryContents contents = osgDB::getDirectoryContents( directory );

    std::vector< std::string > filenames;

    for (unsigned int i = 0; i < contents.size(); ++i)
    {
        if (contents[i] != "." && contents[i] != "..")
        {
            std::string inputFilename = osgDB::concatPaths(directory, contents[i]);
            if (osgDB::getFileExtension(inputFilename) == "pts")
            {
                filenames.push_back( inputFilename );
            }
        }
    }

    CompositePointSource* source = new CompositePointSource();
    for (unsigned int i = 0; i < filenames.size(); ++i)
    {
        PointSource *s = PointSource::loadPointSource( filenames[i] );
        OSG_NOTICE << "Loaded pointsource " << filenames[i] << std::endl;
        source->addPointSource( s );
    }
    return source;
}

void makeSimpleFromDirectory(const std::string& directory)
{
        osg::notify(osg::NOTICE) << "Loading directory " << directory << std::endl;
        osgDB::DirectoryContents contents = osgDB::getDirectoryContents( directory );

        std::vector< std::string > filenames;

        for (unsigned int i = 0; i < contents.size(); ++i)
        {
            if (contents[i] != "." && contents[i] != "..")
            {
                std::string inputFilename = osgDB::concatPaths(directory, contents[i]);
                if (osgDB::getFileExtension(inputFilename) == "txt")
                {
                    filenames.push_back( inputFilename );
                }
            }
        }


        osg::ref_ptr< osg::OperationQueue > queue = new osg::OperationQueue;
        for (unsigned int i = 0; i < filenames.size(); ++i)
        {
            std::string inputFilename  = filenames[i];
            std::string outputFilename = osgDB::getNameLessExtension(inputFilename) + ".pts";
            queue->add(new MakeSimpleOp(inputFilename, outputFilename));
        }

        OSG_NOTICE << "Total tasks=" << queue->getNumOperationsInQueue() << std::endl;

        std::vector< osg::ref_ptr< osg::OperationsThread > > threads;
        unsigned int numThreads = 64;
        for (unsigned int i = 0; i < numThreads; ++i)
        {
            osg::OperationsThread* thread = new osg::OperationsThread();
            thread->setOperationQueue(queue.get());
            OSG_NOTICE << "Starting thread " << i << std::endl;
            thread->start();
            threads.push_back( thread );
        }

        while (!queue->empty())
        {
            OpenThreads::Thread::microSleep(5 * 1000 * 1000);
            OSG_NOTICE << "Number of tasks remaining " << queue->getNumOperationsInQueue() << std::endl;
        }

        //Wait for all the threads to finish the operation they are working on
        for (unsigned int i = 0; i < threads.size(); ++i)
        {
            while (threads[i]->getCurrentOperation())
            {
                OpenThreads::Thread::YieldCurrentThread();
            }
        }


        OSG_NOTICE << "All threads finished!!!" << std::endl;

}




int main(int argc, char** argv)
{
    // use an ArgumentParser object to manage the program arguments.
    osg::ArgumentParser arguments(&argc,argv);


    /*
    osg::ref_ptr< RejectionFile > rj = new RejectionFile(NULL, "test.spawar_points");
    rj->addPoint(Point(osg::Vec3(1,1,1), osg::Vec3(1.5,1.5,1.5), osg::Vec4(1.75,1.75,1.75,1)));
    rj->addPoint(Point(osg::Vec3(2,2,2), osg::Vec3(2.5,2.5,2.5), osg::Vec4(2.75,2.75,2.75,1)));
    rj->addPoint(Point(osg::Vec3(3,3,3), osg::Vec3(3.5,3.5,3.5), osg::Vec4(3.75,3.75,3.75,1)));

    rj = NULL;

    osg::ref_ptr< PointSource > ps = PointSource::loadPointSource( "test.spawar_points");
    osg::ref_ptr< PointCursor > pc = ps->createPointCursor();
    while (pc->hasMore())
    {
        Point p;
        if (pc->nextPoint(p))
        {
            OSG_NOTICE << "Read point pos=" << p._position << " normal=" << p._normal << "  color=" << p._color <<  std::endl;
        }
        else
        {
            OSG_NOTICE << "Couldn't read point" << std::endl;
        }
    }
    return 0;*/

    // construct the viewer.
    osgViewer::Viewer viewer(arguments);

    arguments.getApplicationUsage()->setApplicationName(arguments.getApplicationName());
    arguments.getApplicationUsage()->setDescription(arguments.getApplicationName()+" is the standard osgjuniper application to build point cloud paged lod.");
    arguments.getApplicationUsage()->setCommandLineUsage(arguments.getApplicationName()+" [options] filename ...");
    arguments.getApplicationUsage()->addCommandLineOption("--radiusFactor <number>","factor to set the final radius of a node, default is 6.");
    arguments.getApplicationUsage()->addCommandLineOption("--leafPoints <integer>","try to keep the number of points per leafs, default is 10000.");
    arguments.getApplicationUsage()->addCommandLineOption("--directory <directory> or -d <directory>","Specify a directory of files to load.");
    arguments.getApplicationUsage()->addCommandLineOption("--ext    <extension>","The extension to use, default is osg");
    arguments.getApplicationUsage()->addCommandLineOption("--output <name>","The filename to use for the root, default is root ");    

    unsigned int helpType = 0;
    if ((helpType = arguments.readHelpType()))
    {
        arguments.getApplicationUsage()->write(std::cout, helpType);
        return 1;
    }

    // report any errors if they have occurred when parsing the program arguments.
    if (arguments.errors())
    {
        arguments.writeErrorMessages(std::cout);
        return 1;
    }


    // any option left unread are converted into errors to write out later.
    arguments.reportRemainingOptionsAsUnrecognized();

    // report any errors if they have occurred when parsing the program arguments.
    if (arguments.errors())
    {
        arguments.writeErrorMessages(std::cout);
        return 1;
    }
 
     std::string inputFilename = "C:\\dev\\juniper\\data\\alley\\model.spawar_points";
    //std::string inputFilename = "C:\\dev\\juniper\\data\\alley\\point_cloud_split\\xaa.spawar_points";
    //std::string inputFilename = "F:\\geodata\\paul_levy_2005_PostKatrina_35192\\30089_21_63_raw.txt";
    //std::string inputFilename = "F:\\geodata\\paul_levy_2005_PostKatrina_35192\\30089_15_63_raw.pts";    
    //osg::ref_ptr< PointSource> pointSource = PointSource::loadPointSource(inputFilename);
     osg::ref_ptr< PointSource > pointSource = makePointSourceFromDirectory("F:\\geodata\\paul_levy_2005_PostKatrina_35192");
    /*if (!pointSource.valid())
    {
        OSG_NOTICE << "Couldn't load PointSource " << inputFilename << std::endl;
        return 0;
    }*/
    osg::ref_ptr< PointCursor > cursor = pointSource->createPointCursor();

    osg::Timer_t startTime = osg::Timer::instance()->tick();
    {

        //Come up with the initial bounding box
        osg::BoundingBoxd bb;
        unsigned int numPoints = 0;
        while (cursor->hasMore())
        {
            Point p;
            if (cursor->nextPoint(p))
            {
                bb.expandBy( p._position );
                numPoints++;
            }
        }        

        s_progress.setTotal( numPoints );
        OSG_NOTICE << "Read " << numPoints << " points, bounding box is " << bb._min << " to " << bb._max << std::endl;
        osg::ref_ptr< OctreeNode > sceneRoot = new OctreeNode();
        //bb.expandBy(bb._min - osg::Vec3(1,1,1) * 0.01 * bb.radius());
        //bb.expandBy(bb._max + osg::Vec3(1,1,1) * 0.01 * bb.radius());
        sceneRoot->setBoundingBox( bb );


        osg::ref_ptr< osg::OperationQueue > queue = new osg::OperationQueue;
        MakeSceneVisitor msv(inputFilename, 7, 11);
        msv._pointSource = makePointSourceFromDirectory("F:\\geodata\\paul_levy_2005_PostKatrina_35192");
        msv.setOperationQueue( queue.get() );


        //Start up the threads
        std::vector< osg::ref_ptr< osg::OperationsThread > > threads;
        unsigned int numThreads = 16;
        for (unsigned int i = 0; i < numThreads; ++i)
        {
            osg::OperationsThread* thread = new osg::OperationsThread();
            thread->setOperationQueue(queue.get());
            OSG_NOTICE << "Starting thread " << i << std::endl;
            thread->start();
            threads.push_back( thread );
        }

        sceneRoot->accept( msv );

        /*while (!queue->empty())
        {
            OpenThreads::Thread::microSleep(5 * 1000 * 1000);
            OSG_NOTICE << "Number of tasks remaining " << queue->getNumOperationsInQueue() << std::endl;
        }*/

        while (!s_progress.isComplete())
        {
            float percentComplete = floor(((float)s_progress.getComplete() / (float)s_progress.getTotal()) * 100.0f);
            OSG_NOTICE << "Finished " << s_progress.getComplete() << " of " << s_progress.getTotal() << "  " << percentComplete << "% complete" <<   std::endl;
            OpenThreads::Thread::microSleep(5 * 1000 * 1000);
        }

        bool done = false;
        while (!done)
        {
            OSG_NOTICE << "Waiting for threads to exit..." << std::endl;
            done = true;
            for (unsigned int i = 0; i < threads.size(); ++i)
            {
                if (threads[i]->getCurrentOperation())
                {
                    done = false;
                    break;
                }
            }
        }
        float percentComplete = floor(((float)s_progress.getComplete() / (float)s_progress.getTotal()) * 100.0f);
        OSG_NOTICE << "Finished " << s_progress.getComplete() << " of " << s_progress.getTotal() << "  " << percentComplete << "% complete" <<   std::endl;
    }
    OSG_NOTICE << "All threads done!" << std::endl;

    osg::Timer_t endTime = osg::Timer::instance()->tick();

    double seconds = osg::Timer::instance()->delta_s(startTime, endTime);
    double hours   = floor(seconds / 3600.0);
    seconds -= (hours * 3600);
    double minutes = floor(seconds / 60.0);
    seconds -= (minutes * 60);
    
    OSG_NOTICE << "Completed in " << hours << " hours " << minutes << " min " << seconds << " s" << std::endl;


    return 0;   
}
