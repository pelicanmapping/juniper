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

#include <osgText/Text>
#include <osg/Geode>
#include <osg/Group>
#include <osg/MatrixTransform>
#include <osg/io_utils>

#include <osg/Sequence>
#include <osgDB/ReadFile>
#include <osgDB/FileUtils>
#include <osgDB/FileNameUtils>

#include <osgGA/StateSetManipulator>

#include <osgViewer/Viewer>
#include <osgViewer/ViewerEventHandlers>


#include <osgGA/AnimationPathManipulator>

#include <iostream>


struct AssignColorVisitor : public osg::NodeVisitor
{    
    AssignColorVisitor(const osg::Vec4f& color) : 
      osg::NodeVisitor(osg::NodeVisitor::TRAVERSE_ALL_CHILDREN),
      _color( color )
      {}

    void apply(osg::Geode& node) {
        for (unsigned int i = 0; i < node.getNumDrawables(); i++)
        {
            osg::Geometry* geometry = node.getDrawable(i)->asGeometry();
            if (geometry)
            {
                osg::Vec4Array* colors = dynamic_cast<osg::Vec4Array*>(geometry->getColorArray());
                if (!colors)
                {
                    colors = new osg::Vec4Array(1);
                    (*colors)[0] = _color;
                    geometry->setColorArray( colors );
                    geometry->setColorBinding( osg::Geometry::BIND_OVERALL );
                }
            }
        }
        traverse(node);
    }

    osg::Vec4f _color;
};

osg::Vec3dArray* loadSpine( const std::string& filename )
{
    std::ifstream in(filename.c_str(), std::ios::in );
    osg::Vec3dArray* result = new osg::Vec3dArray();
    while (!in.eof())
    {
        std::string line;
        getline(in, line);
        double x, y, z;
        if (sscanf(line.c_str(), "%lf %lf %lf", &x, &y, &z) == 3)
        {
            //OSG_NOTICE << "Pushing back " << x << ", " << y << ", " << z << std::endl;
            result->push_back( osg::Vec3d(x,y,z) );
        }
    }            
    return result;
}

osg::AnimationPath* createPath( osg::Vec3dArray* spine, double speed )
{
    double time = 0.0;
    osg::AnimationPath* path = new osg::AnimationPath();
    for (unsigned int i = 0; i < spine->size()-1; i++)
    {
        osg::Vec3d p0 = (*spine)[i];
        osg::Vec3d p1 = (*spine)[i+1];
        osg::Vec3d look = p1 - p0;
        double dist = look.length();
        osg::Quat rot = osg::Matrixd::lookAt(p0, p1, osg::Vec3d(0,0,1)).getRotate();
        rot = rot.inverse();        
        path->insert(time, osg::AnimationPath::ControlPoint(p0, rot));
        time += (dist / speed );       
    }
    return path;
}

osg::Node* makeSpine( osg::Vec3dArray* spine )
{
    osg::Geometry* geometry = new osg::Geometry;
    osg::Vec3Array* verts = new osg::Vec3Array( spine->size() );
    for (unsigned int i = 0; i < spine->size(); i++)
    {
        osg::Vec3d p = (*spine)[i];
        (*verts)[i] = osg::Vec3(p.x(), p.y(), p.z());
    }
    geometry->setVertexArray( verts );

    osg::Vec4Array* colors = new osg::Vec4Array(1);
    (*colors)[0] = osg::Vec4f(0,1,0,1);
    geometry->setColorArray( colors );
    geometry->setColorBinding( osg::Geometry::BIND_OVERALL );
    geometry->addPrimitiveSet( new osg::DrawArrays(GL_LINE_STRIP, 0, verts->size() ) );

    osg::Geode* geode = new osg::Geode;

    geode->addDrawable( geometry );
    geode->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF);



    return geode;
}


int main( int argc, char **argv )
{   
    // use an ArgumentParser object to manage the program arguments.
    osg::ArgumentParser arguments(&argc,argv);

    std::string spineFilename  = "C:/dev/RTC_Data/PointCloudData/FullTunnel_AXIS.txt";
    std::string tunnelFilename = "C:/dev/RTC_Data/PointCloudData/FullTunnel_MESH.osgb";

    osg::Vec3dArray* spine = loadSpine( spineFilename );

    // construct the viewer.
    osgViewer::Viewer viewer(arguments);

    osg::ref_ptr < osg::Group > root = new osg::Group;

    osg::Node* tunnel = osgDB::readNodeFile( tunnelFilename );
    //Fix colors in the tunnel if they dont' already have a color assigned
    AssignColorVisitor v(osg::Vec4(1,0,1,1));
    tunnel->accept( v );

    root->addChild( makeSpine( spine ) );
    root->addChild( tunnel );    

    osg::AnimationPath* path = createPath( spine, 1.0 );
    osgGA::AnimationPathManipulator* apm = new osgGA::AnimationPathManipulator( path );
    viewer.setCameraManipulator( apm );
    
    // add model to viewer.
    viewer.setSceneData(root);

    // add the state manipulator
    viewer.addEventHandler( new osgGA::StateSetManipulator(viewer.getCamera()->getOrCreateStateSet()) );

    // add the window size toggle handler
    viewer.addEventHandler(new osgViewer::WindowSizeHandler);

    // add the stats handler
    viewer.addEventHandler(new osgViewer::StatsHandler);

    return viewer.run();
}
