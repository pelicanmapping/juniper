/* -*-c++-*- */
/* osgJuniper - Large Dataset Visualization Toolkit for OpenSceneGraph
* Copyright 2010-2017 Pelican Mapping
* Pelican Mapping CONFIDENTIAL
* Copyright (c) 2010-2017 [Pelican Mapping], All Rights Reserved.
*
* NOTICE:  All information contained herein is, and remains the property of Pelican Mapping. The intellectual and technical concepts contained
* herein are proprietary to Pelican Mapping and may be covered by U.S. and Foreign Patents, patents in process, and are protected by trade secret or copyright law.
* Dissemination of this information or reproduction of this material is strictly forbidden unless prior written permission is obtained
* from Pelican Mapping.  Access to the source code contained herein is hereby forbidden to anyone except current Pelican Mapping employees, managers or contractors who have executed
* Confidentiality and Non-disclosure agreements explicitly covering such access.
*
* The copyright notice above does not evidence any actual or intended publication or disclosure  of  this source code, which includes
* information that is confidential and/or proprietary, and is a trade secret, of Pelican Mapping.   ANY REPRODUCTION, MODIFICATION, DISTRIBUTION, PUBLIC  PERFORMANCE,
* OR PUBLIC DISPLAY OF OR THROUGH USE  OF THIS  SOURCE CODE  WITHOUT  THE EXPRESS WRITTEN CONSENT OF PELICAN MAPPING IS STRICTLY PROHIBITED, AND IN VIOLATION OF APPLICABLE
* LAWS AND INTERNATIONAL TREATIES.  THE RECEIPT OR POSSESSION OF  THIS SOURCE CODE AND/OR RELATED INFORMATION DOES NOT CONVEY OR IMPLY ANY RIGHTS
* TO REPRODUCE, DISCLOSE OR DISTRIBUTE ITS CONTENTS, OR TO MANUFACTURE, USE, OR SELL ANYTHING THAT IT  MAY DESCRIBE, IN WHOLE OR IN PART.
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
        bb.expandBy( p.position );
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
