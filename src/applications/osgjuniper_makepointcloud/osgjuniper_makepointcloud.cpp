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
#include <osgJuniper/SPTNode>

#include <fstream>
#include <sstream>

using namespace osgJuniper;


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


class RejectionFile : public osg::Referenced
{
public:
    RejectionFile(OctreeNode* node, const std::string& filename):
      _node(node),
      _filename(filename),
      _needsProcessed(true),
      _numPoints(0),
      _isFile(false)
    {
    }

      void close()
      {
          if (_out.is_open()) _out.close();
      }

      const std::string& getFilename() { return _filename;}

      PointSource* createPointSource()
      {
          if (_isFile)
          {
              return PointSource::loadPointSource( _filename );
          }
          else
          {
              return new PointListSource(_points);
          }
      }

      bool contains(osg::Vec3d& point)
      {
          return _node->getBoundingBox().contains( point );
      }

      bool getNeedsProcessed() const { return _numPoints > 0 && _needsProcessed;}
      void setNeedsProcessed(bool needsProcessed) { _needsProcessed = needsProcessed;}

      OctreeNode* getNode() { return _node.get();}

      unsigned int getNumPoints() const { return _numPoints; }

      void addPoint(const Point& point)
      {
          //TODO:  Adjust this based on the amount of memory we have.
          unsigned int maxPoints = 1000000;
          _numPoints++;
          if (_isFile)
          {
              writePoint( point );
          }
          else
          {
              if (_points.size() < maxPoints)
              {
                  _points.push_back( point );
              }
              else
              {
                  osg::notify(osg::NOTICE) << "Rejection file has over " << maxPoints << ", dumping to disk" << std::endl;
                  _isFile = true;
                  //The file is not open, and we're at the max points in memory.  Write all the points to disk and move to a file based system
                  for (PointList::iterator itr = _points.begin(); itr != _points.end(); ++itr)
                  {
                      writePoint( *itr );
                  }
                  _points.clear();
                  //Write the new point to disk
                  writePoint( point );
              }
          }
      }

      void remove()
      {
          if (_out.is_open())
          {
              _out.close();
          }
          //osg::notify(osg::NOTICE) << "Removing " << _filename << std::endl;
          if (osgDB::fileExists(_filename))
          {
              unsigned int result = ::remove(_filename.c_str());
              if (result != 0)
              {
                  //osg::notify(osg::NOTICE) << "Could not remove file " << _filename.c_str() << result << std::endl;
              }
          }
      }

protected:
    void writePoint(const Point& point)
    {
        if (!_out.is_open())
        {
            _out.open( _filename.c_str(), std::ios::out | std::ios::binary);
        }

        osg::Vec3 color(point._color.r(), point._color.g(), point._color.b());
        color *= 255.0f;        
        _out.write((char*)point._position._v, sizeof(double) * 3);
        _out.write((char*)point._normal._v, sizeof(float) * 3);
        _out.write((char*)color._v, sizeof(float) * 3);
        _out.write((char*)&point._size, sizeof(float));
    }

    osg::ref_ptr< OctreeNode > _node;
    std::string _filename;
    unsigned int _numPoints;    
    std::ofstream _out;
    bool _needsProcessed;
    PointList _points;
    bool _isFile;
};




/**********************************************************/
class MakeSceneVisitor : public OctreeNodeVisitor
{
public:
    MakeSceneVisitor(const std::string &filename, unsigned int innerMaxLevel, unsigned int outerMaxLevel);
    MakeSceneVisitor(PointSource* pointSource, unsigned int innerMaxLevel, unsigned int outerMaxLevel);

    const std::string& getPrefix() const { return _prefix; }
    void setPrefix( const std::string &prefix )
    {
        _prefix = prefix;
    }

    const std::string& getExtension() const { return _ext;}

    void setExtension(const std::string& extension) { _ext = extension; }

    void setOperationQueue(osg::OperationQueue* queue)
    {
        _operationQueue = queue;
    }

    float getRadiusFactor() const { return _radiusFactor; }
    void setRadiusFactor( float radiusFactor ) { _radiusFactor = radiusFactor; }

    void setUseVBO( bool useVBO ) { _useVBO = useVBO; }
    bool getUseVBO() const { return _useVBO; }


    osg::Node* makeNode(PointList& points);

    void addRejectionFile(RejectionFile* rejectionFile, OctreeNode& node);

    virtual void apply(OctreeNode& node);

    std::string createRejectionFile( OctreeNode& node);
    std::string createURI( OctreeNode& node);

    osg::ref_ptr< osg::OperationQueue > _operationQueue;
    std::string _filename;

    osg::ref_ptr< PointSource> _pointSource;
    unsigned int _innerMaxLevel;
    unsigned int _outerMaxLevel;

    std::string _ext;
    std::string _prefix;
    float _radiusFactor;
    bool _useVBO;
};

/************************************************************/

class MakeSceneOp : public osg::Operation
{
public:
    MakeSceneOp(OctreeNode* node, MakeSceneVisitor visitor, bool remove=false);

    void operator()(osg::Object*);
    osg::ref_ptr<OctreeNode> _node;
    osg::ref_ptr< osg::OperationQueue > _queue;
    MakeSceneVisitor _visitor;
    bool _remove;
};



/************************************************************/
MakeSceneVisitor::MakeSceneVisitor(const std::string &filename, unsigned int innerMaxLevel, unsigned int outerMaxLevel):
_filename(filename),
_innerMaxLevel(innerMaxLevel),
_outerMaxLevel(outerMaxLevel),
_ext("ive"),
_radiusFactor(20.0f),
_useVBO( true )
{
}

MakeSceneVisitor::MakeSceneVisitor(PointSource* pointSource, unsigned int innerMaxLevel, unsigned int outerMaxLevel):
_pointSource(pointSource),
_innerMaxLevel(innerMaxLevel),
_outerMaxLevel(outerMaxLevel),
_ext("ive"),
_radiusFactor(20.0f),
_useVBO( true )
{
}

void MakeSceneVisitor::addRejectionFile(RejectionFile* rejectionFile, OctreeNode& node) 
{
    //Close the rejection file
    rejectionFile->close();
    //osg::notify(osg::NOTICE) << "Rejection file " << rejectionFiles[i]->getFilename() << " contains " << rejectionFiles[i]->getNumPoints() << std::endl;            
    unsigned int numReadRejectionFile = 0;
    osg::ref_ptr< PointSource > rejectionSource = rejectionFile->createPointSource();
    osg::ref_ptr< PointCursor > rejectionCursor = rejectionSource->createPointCursor();                
    while (rejectionCursor->hasMore())
    {
        Point p;
        if (rejectionCursor->nextPoint(p))
        {            
            AddPointVisitor apv(p, _innerMaxLevel);
            apv.setStrategy(AddPointVisitor::ACCEPT);
            node.accept(apv);            
        }
    }    
    //Delete the cursor and close the rejection file
    rejectionCursor = NULL;
    rejectionFile->setNeedsProcessed(false);    
    rejectionFile->remove();      
}

void
MakeSceneVisitor::apply(OctreeNode& node)
{
    //osg::notify(osg::NOTICE) << "Building scene from filename " << _filename << std::endl;
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
                osg::notify(osg::NOTICE) << "Reached max level of " << _outerMaxLevel << ", accepting all points" << std::endl;
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
                    osg::notify(osg::NOTICE) << "Could not add point " << p._position << " to rejection file" << std::endl;
                    for (unsigned int i = 0; i < 8; ++i)
                    {
                        osg::notify(osg::NOTICE) << "Rejection file " << i << " " <<  rejectionFiles[i]->getNode()->getBoundingBox()._min << " to " << rejectionFiles[i]->getNode()->getBoundingBox()._max << std::endl;
                    }
                }
            }
            else
            {
                numPointsAdded++;
            }
        }
        //if (numPointsRead % 50000 == 0) osg::notify(osg::NOTICE) << "Read " << numPointsRead << " points..." << std::endl;
    }

   
    unsigned int maxSize = 60000;

    if (numPointsAdded + numPointsRejected < maxSize)
    {       
        OSG_NOTICE << "Adding points from rejection file, total size of node is < " << maxSize << std::endl;
        for (unsigned int i = 0; i < 8; ++i)
        {
            addRejectionFile(rejectionFiles[i], innerOctree);
            numPointsAdded += rejectionFiles[i]->getNumPoints();
            numPointsRejected -= rejectionFiles[i]->getNumPoints();
        }       
    }
    else
    {
        unsigned int rejectionThreshold = 2000;
        //Include trivially small rejection files
        for (unsigned int i = 0; i < 8; ++i)
        {
            if (rejectionFiles[i]->getNumPoints() < rejectionThreshold)
            {
                OSG_NOTICE << "Adding in small rejection file with " << rejectionFiles[i]->getNumPoints() << std::endl;
                addRejectionFile(rejectionFiles[i], innerOctree);
                numPointsAdded += rejectionFiles[i]->getNumPoints();
                numPointsRejected -= rejectionFiles[i]->getNumPoints();
            }
        }       

    }

    //osg::notify(osg::NOTICE) << "Read " << numPointsRead << " points  Added=" << numPointsAdded << " Rejected=" << numPointsRejected << std::endl;
    if (numPointsAdded + numPointsRejected != numPointsRead)
    {
        osg::notify(osg::NOTICE) << "Something wrong..." << std::endl;
    }

    //Write out this node
    //ApplyRepresentativePointVisitor arpv;
    //innerOctree.accept( arpv );

    CollectPointsVisitor pointCollector;
    innerOctree.accept( pointCollector );

    //Create the node to render the points for OctreeNode
    //osg::ref_ptr<SPTNode> points = new SPTNode();
    //points->setPoints( pointCollector.getPoints() );
    //points->getOrCreateStateSet()->setAttributeAndModes(new osg::Point(1.1), osg::StateAttribute::ON);
    //points->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF);    
    osg::ref_ptr< osg::Node > points = makeNode( pointCollector.getPoints() );
    std::string nodeFilename = createURI(node);

    //Create a PagedLOD for this node
    osg::ref_ptr< osg::PagedLOD > pagedLOD = new osg::PagedLOD;        
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
            pagedLOD->setRange(position, 0, rejectionFiles[i]->getNode()->getBoundingBox().radius() * _radiusFactor);
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
            //MakeSceneVisitor msv(rejectionFiles[i]->getFilename(), _innerMaxLevel, _outerMaxLevel);
            MakeSceneVisitor msv(rejectionFiles[i]->createPointSource(), _innerMaxLevel, _outerMaxLevel);
            msv.setOperationQueue( _operationQueue.get() );
            msv.setExtension( _ext );
            msv.setPrefix( _prefix );
            msv.setRadiusFactor( _radiusFactor );
            msv.setUseVBO( _useVBO );

            //Make the max number of operations configurable            
            _operationQueue->add( new MakeSceneOp(rejectionFiles[i]->getNode(), msv, false));
            //_operationQueue->add( new MakeSceneOp(rejectionFiles[i]->getNode(), rejectionFiles[i]->getFilename(), _innerMaxLevel, _outerMaxLevel, _ext, _prefix, _radiusFactor, _operationQueue.get(), true));
        }
    }
}

static int s_minSize = INT_MAX;
static int s_maxSize = 0;
static int s_numPoints = 0;
static int s_numNodes = 0;
static OpenThreads::Mutex s_statsMutex;

osg::Node*
MakeSceneVisitor::makeNode(PointList& points)
{
    unsigned int batchSize = 100000;

    osg::Geometry* geometry = NULL;
    osg::Vec3Array* verts   = NULL;
    osg::Vec4ubArray* colors  = NULL;    

    osg::Geode* geode = new osg::Geode;

    //Update the stats    
    {
        OpenThreads::ScopedLock< OpenThreads::Mutex> statsMutex(s_statsMutex);
        s_numPoints += points.size();
        if (s_minSize > points.size()) s_minSize = points.size();
        if (s_maxSize < points.size()) s_maxSize = points.size();
        s_numNodes++;
    }

    osg::Vec3d anchor = points.front()._position;


    unsigned int numPoints = 0;
    while (points.size() > 0)
    {         
        if (!geometry)
        {
            numPoints = 0;
            //Allocate the new geometry
            geometry = new osg::Geometry;

            if (_useVBO)
            {
                geometry->setUseVertexBufferObjects( true );
                geometry->setUseDisplayList( false );
            }

            verts = new osg::Vec3Array(batchSize);
            geometry->setVertexArray( verts );    

            colors = new osg::Vec4ubArray(batchSize);
            geometry->setColorArray( colors );
            geometry->setColorBinding( osg::Geometry::BIND_PER_VERTEX);
        }
        
        osg::Vec3 position = points.front()._position - anchor;
        (*verts)[numPoints] = position;
        osg::Vec4ub color = osg::Vec4ub(points.front()._color.r() * 255,
                                        points.front()._color.g() * 255,
                                        points.front()._color.b() * 255,
                                        points.front()._color.a() * 255);
        (*colors)[numPoints] = color;

        numPoints++;

        //Remove the point from the list to reduce memory usage
        points.pop_front();

        if (numPoints >= batchSize)
        {
            geode->addDrawable( geometry );
            geometry->addPrimitiveSet( new osg::DrawArrays(GL_POINTS, 0, numPoints) );
            geometry = 0;
        }
    }

    if (geometry)
    {
        geode->addDrawable( geometry );
        geometry->addPrimitiveSet( new osg::DrawArrays(GL_POINTS, 0, numPoints) );
        geometry = 0;
    }

    osg::MatrixTransform* mt = new osg::MatrixTransform;
    mt->setMatrix(osg::Matrixd::translate(anchor));
    mt->addChild(geode);
    return mt;    
}

std::string MakeSceneVisitor::createRejectionFile( OctreeNode& node)
{
    std::stringstream ss;
    ss << "R_" << node.getID().level << "_" 
        << node.getID().x << "_"
        << node.getID().y << "_"
        << node.getID().z << ".spawar_points";
    return ss.str();
}

std::string MakeSceneVisitor::createURI( OctreeNode& node)
{
    std::stringstream ss;
    ss << _prefix
        << node.getID().level << "_" 
        << node.getID().x << "_"
        << node.getID().y << "_"
        << node.getID().z << "." << _ext;
    return ss.str();
    }

/************************************************************/

MakeSceneOp::MakeSceneOp(OctreeNode* node, MakeSceneVisitor visitor, bool remove):
_node(node),
_visitor(visitor)
{
}

void
MakeSceneOp::operator ()(osg::Object *)
{
    {
        _node->accept( _visitor);     
    }
}

PointSource* createPointSource( const std::vector< std::string > &filenames )
{
    if (filenames.size() == 1)
    {
        return PointSource::loadPointSource( filenames[0] );
    }
    else
    {
        return new CompositePointSource(filenames);
    } 
}

int main(int argc, char** argv)
{
    // use an ArgumentParser object to manage the program arguments.
    osg::ArgumentParser arguments(&argc,argv);

    arguments.getApplicationUsage()->setApplicationName(arguments.getApplicationName());
    arguments.getApplicationUsage()->setDescription(arguments.getApplicationName()+" is the standard osgjuniper application to build point cloud paged lod.");
    arguments.getApplicationUsage()->setCommandLineUsage(arguments.getApplicationName()+" [options] filename ...");
    arguments.getApplicationUsage()->addCommandLineOption("--innerMaxLevel <number>","inner octree max level, default is 5.");
    arguments.getApplicationUsage()->addCommandLineOption("--outerMaxLevel <number>","outer octree max level, default is 11.");
    arguments.getApplicationUsage()->addCommandLineOption("--radiusFactor <number>","factor to set the final radius of a node, default is 5.");
    arguments.getApplicationUsage()->addCommandLineOption("--directory <directory> or -d <directory>","Specify a directory of files to load.");
    arguments.getApplicationUsage()->addCommandLineOption("--filter <filter>","The extension to use to filter files out if loading files from a directory.  'pts' for example");    
    arguments.getApplicationUsage()->addCommandLineOption("--ext    <extension>","The extension to use, default is ive");
    arguments.getApplicationUsage()->addCommandLineOption("--prefix <prefix>","The prefix to use for the filenames ");    
    arguments.getApplicationUsage()->addCommandLineOption("--threads <number>","The number of threads to use, default is the number of CPUs on the system");        
    arguments.getApplicationUsage()->addCommandLineOption("--disableVBO","Disable the use of vertex buffer objects.  Defaults to false.");        

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

    bool disableVBO = false;
    while (arguments.read("--disableVBO")) { disableVBO = true; }
    osg::notify(osg::NOTICE) << "disableVBO " << disableVBO << std::endl;

    std::string prefix;
    while (arguments.read("--prefix", prefix));
    osg::notify(osg::NOTICE) << "prefix " << prefix << std::endl;

    float radiusFactor = 10.0f;
    while (arguments.read("--radiusFactor", radiusFactor));
    osg::notify(osg::NOTICE) << "radiusFactor " << radiusFactor << std::endl;

    std::string ext = "ive";
    while (arguments.read("--ext", ext));
    osg::notify(osg::NOTICE) << "ext " << ext << std::endl;

    std::string directory;
    while (arguments.read("-d", directory));
    while (arguments.read("--directory", directory));
    osg::notify(osg::NOTICE) << "Directory " << directory << std::endl;

    unsigned int innerMaxLevel = 5;
    while (arguments.read("--innerMaxLevel", innerMaxLevel));
    osg::notify(osg::NOTICE) << "innerMaxLevel " << innerMaxLevel << std::endl;

    unsigned int outerMaxLevel = 11;
    while (arguments.read("--outerMaxLevel", outerMaxLevel));
    osg::notify(osg::NOTICE) << "outerMaxLevel " << outerMaxLevel << std::endl;

    unsigned int numThreads = 2;//OpenThreads::GetNumberOfProcessors();
    while (arguments.read("--threads", numThreads));
    osg::notify(osg::NOTICE) << "numThreads " << numThreads << std::endl;

    std::string filter;
    while (arguments.read("--filter", filter));
    osg::notify(osg::NOTICE) << "Filter " << filter << std::endl;



    std::vector< std::string > filenames;

    if (directory.empty())
    {
        //Read in the filenames to process
        for(int pos=1;pos<arguments.argc();++pos)
        {
            if (!arguments.isOption(pos))
            {
                filenames.push_back( arguments[pos]);
            }
        }
    }
    else
    {
        //Load the filenames from a directory
        filenames = Utils::getFilesFromDirectory(directory, filter);
    }
    // any option left unread are converted into errors to write out later.
    arguments.reportRemainingOptionsAsUnrecognized();

    // report any errors if they have occurred when parsing the program arguments.
    if (arguments.errors())
    {
        arguments.writeErrorMessages(std::cout);
        return 1;
    } 

    if (filenames.empty())
    {
        osg::notify(osg::NOTICE) << "You must specify at least one input filename" << std::endl;
        return 1;
    }

    for (unsigned int i = 0; i < filenames.size(); ++i)
    {
        osg::notify(osg::NOTICE) << "Processing file " << i << " : " << filenames[i] << std::endl;
    }
    
    osg::ref_ptr< PointSource> pointSource = createPointSource( filenames );     
    if (!pointSource.valid())
    {
        OSG_NOTICE << "Unable to create a point source from the given filenames." << std::endl;
        return 1;
    }
    osg::ref_ptr< PointCursor > cursor = pointSource->createPointCursor();   

    osg::Timer_t startTime = osg::Timer::instance()->tick();
    {
        osg::notify(osg::NOTICE) << "Reading all points and building initial bounding box..." << std::endl;
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
        osg::notify(osg::NOTICE) << "Read " << numPoints << " points, bounding box is " << bb._min << " to " << bb._max << std::endl;
        osg::ref_ptr< OctreeNode > sceneRoot = new OctreeNode();
        sceneRoot->setBoundingBox( bb );


        osg::ref_ptr< osg::OperationQueue > queue = new osg::OperationQueue;
        MakeSceneVisitor msv(pointSource, innerMaxLevel, outerMaxLevel);
        msv.setOperationQueue( queue.get() );
        msv.setPrefix(prefix);
        msv.setExtension(ext);
        msv.setUseVBO( !disableVBO );
        msv.setRadiusFactor( radiusFactor );


        //Start up the threads
        osg::notify(osg::NOTICE) << "Starting " << numThreads << " processing threads" << std::endl;
        std::vector< osg::ref_ptr< osg::OperationsThread > > threads;
        for (unsigned int i = 0; i < numThreads; ++i)
        {
            osg::OperationsThread* thread = new osg::OperationsThread();
            thread->setOperationQueue(queue.get());
            thread->start();
            threads.push_back( thread );
        }
        osg::notify(osg::NOTICE) << "Started threads" << std::endl;

        sceneRoot->accept( msv );

        while (!s_progress.isComplete())
        {
            float percentComplete = floor(((float)s_progress.getComplete() / (float)s_progress.getTotal()) * 100.0f);
            osg::notify(osg::NOTICE) << "Finished " << s_progress.getComplete() << " of " << s_progress.getTotal() << "  " << percentComplete << "% complete" <<   std::endl;
            OpenThreads::Thread::microSleep(5 * 1000 * 1000);
        }

        bool done = false;
        while (!done)
        {
            osg::notify(osg::NOTICE) << "Waiting for threads to exit..." << std::endl;
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
        osg::notify(osg::NOTICE) << "Finished " << s_progress.getComplete() << " of " << s_progress.getTotal() << "  " << percentComplete << "% complete" <<   std::endl;
    }
    osg::notify(osg::NOTICE) << "All threads done!" << std::endl;

    osg::Timer_t endTime = osg::Timer::instance()->tick();

    double seconds = osg::Timer::instance()->delta_s(startTime, endTime);
    double hours   = floor(seconds / 3600.0);
    seconds -= (hours * 3600);
    double minutes = floor(seconds / 60.0);
    seconds -= (minutes * 60);
    
    osg::notify(osg::NOTICE) << "Completed in " << hours << " hours " << minutes << " min " << seconds << " s" << std::endl;
    OSG_NOTICE << "Points processed= " << s_numPoints << std::endl;
    OSG_NOTICE << "Min size " << s_minSize << std::endl;               
    OSG_NOTICE << "Max size " << s_maxSize << std::endl;  
    OSG_NOTICE << "Avg size " << (double)s_numPoints / (double)s_numNodes << std::endl;



    return 0;   
}
