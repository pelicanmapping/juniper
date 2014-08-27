/* OpenSceneGraph example, osgsequence.
*
*  Permission is hereby granted, free of charge, to any person obtaining a copy
*  of this software and associated documentation files (the "Software"), to deal
*  in the Software without restriction, including without limitation the rights
*  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
*  copies of the Software, and to permit persons to whom the Software is
*  furnished to do so, subject to the following conditions:
*
*  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
*  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
*  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
*  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
*  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
*  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
*  THE SOFTWARE.
*/

#include <osgText/Text>
#include <osg/Geode>
#include <osg/Group>
#include <osg/MatrixTransform>
#include <osg/Point>

#include <osgDB/ReadFile>
#include <osgDB/FileUtils>
#include <osgDB/FileNameUtils>

#include <osgGA/StateSetManipulator>

#include <osgViewer/Viewer>
#include <osgViewer/ViewerEventHandlers>

#include <iostream>

#include <osgJuniper/Point>
#include <osgJuniper/Octree>
#include <osgJuniper/SPTNode>

using namespace osgJuniper;


SPTNode* makeSceneFromPointSource(PointSource *source, unsigned int maxVerts = UINT_MAX)
{
    //Create the initial cursor
    osg::ref_ptr< PointCursor > cursor = source->createPointCursor();

    unsigned int numRead = 0;
    //Initialize the bounding box from the points
    osg::BoundingBoxd bb;
    Point p;
    while (cursor->nextPoint(p) && numRead < maxVerts)
    {
        bb.expandBy( p._position );
        numRead++;
        if (numRead%1000 == 0) 
            osg::notify(osg::NOTICE) << "Expanded by " << numRead+1 << " points " << std::endl;
    }

   
    OctreeNode* root = new OctreeNode();
    root->setBoundingBox( bb );    

    unsigned int maxLevel = 8;

    //Reset the cursor
    numRead = 0;
    cursor = source->createPointCursor();    
    while (cursor->nextPoint(p) && numRead < maxVerts)
    {        
        AddPointVisitor apv(p, maxLevel);
        root->accept(apv);
        if (numRead%1000 == 0) 
            osg::notify(osg::NOTICE) << "Added " << numRead+1 << " points " << std::endl;
        numRead++;
    }

    osg::notify(osg::NOTICE) << "Added a total of " << numRead << " points" << std::endl;
    
    ApplyRepresentativePointVisitor arpv;
    root->accept( arpv );

    CollectPointsVisitor pointCollector;
    root->accept( pointCollector );

    SPTNode* points = new SPTNode();
    points->setPoints( pointCollector.getPoints() );
    points->getOrCreateStateSet()->setAttributeAndModes(new osg::Point(1.1), osg::StateAttribute::ON);
    points->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF);    

    return points;
}

class SPTEventHandler : public osgGA::GUIEventHandler
{
public:
    SPTEventHandler(SPTNode* spt)
    {
        _spt = spt;
    }

    // handle keydown events
    virtual bool handle(const osgGA::GUIEventAdapter& ea,
                        osgGA::GUIActionAdapter&)
    {
        if (ea.getEventType() == osgGA::GUIEventAdapter::KEYDOWN) {
            switch (ea.getKey()) {
            case 'e':
                {     
                    _spt->setEnabled( !_spt->getEnabled() );
                    osg::notify(osg::NOTICE) << "Toggled enabled = " << _spt->getEnabled() << std::endl;
                }
                break;

            case 'l':
                {  
                    if (_spt->getMaxVerts() == UINT_MAX)
                    {
                        _spt->setMaxVerts( 1000000 );
                        osg::notify(osg::NOTICE) << "Limiting max verts" << std::endl;
                    }
                    else
                    {
                        _spt->setMaxVerts( UINT_MAX );
                        osg::notify(osg::NOTICE) << "Unlimited max verts" << std::endl;
                    }
                }
                break;

            case 'r':
                {  
                    _spt->setRadiusFactor( _spt->getRadiusFactor() * 0.9 );
                    osg::notify(osg::NOTICE) << "Set radius factor " << _spt->getRadiusFactor() << std::endl;
                }
                break;

            case 'R':
                {  
                    _spt->setRadiusFactor( _spt->getRadiusFactor() * 1.1 );
                    osg::notify(osg::NOTICE) << "Set radius factor " << _spt->getRadiusFactor() << std::endl;
                }

            case 'u':
                {  
                    if (_spt->getMaxVerts() != UINT_MAX )
                    {
                        _spt->setMaxVerts( _spt->getMaxVerts() * 0.9);
                        osg::notify(osg::NOTICE) << "Set max verts to " << _spt->getMaxVerts() << std::endl;
                    }
                    
                }

            case 'U':
                {  
                    if (_spt->getMaxVerts() != UINT_MAX )
                    {
                        _spt->setMaxVerts( _spt->getMaxVerts() * 1.1);
                        osg::notify(osg::NOTICE) << "Set max verts to " << _spt->getMaxVerts() << std::endl;
                    }

                }

                break;
            default:
                break;
            }
        }

        return false;
    }

private:
    osg::ref_ptr< SPTNode > _spt;
};




int main( int argc, char **argv )
{
    // use an ArgumentParser object to manage the program arguments.
    osg::ArgumentParser arguments(&argc,argv);
   
    // construct the viewer.
    osgViewer::Viewer viewer(arguments);

    std::string filename;

    //Read in the filenames to process
    for(int pos=1;pos<arguments.argc();++pos)
    {
        if (!arguments.isOption(pos))
        {
            filename = arguments[pos];
            break;
        }
    }

    unsigned int maxVerts = UINT_MAX;
    while (arguments.read("--maxVerts", maxVerts));

    osg::notify(osg::NOTICE) << "Loading file " << filename << std::endl;

    osg::ref_ptr< PointSource > source = PointSource::loadPointSource( filename );
    osg::ref_ptr< SPTNode > loadedModel = makeSceneFromPointSource( source.get(), maxVerts );

    //Disable the SPT optimization for demo purposes
    loadedModel->setEnabled( false );



    // add model to viewer.
    viewer.setSceneData(loadedModel.get());

    viewer.addEventHandler( new SPTEventHandler( loadedModel.get() ));

    // add the state manipulator
    viewer.addEventHandler( new osgGA::StateSetManipulator(viewer.getCamera()->getOrCreateStateSet()) );

    // add the window size toggle handler
    viewer.addEventHandler(new osgViewer::WindowSizeHandler);

    // add the stats handler
    viewer.addEventHandler(new osgViewer::StatsHandler);

    return viewer.run();
}
