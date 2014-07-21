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
#include <osgJuniper/KdTree>
#include <osg/Geometry>
#include <osg/Geode>
#include <osg/Point>
#include <osg/LOD>
#include <osg/Image>
#include <osg/ShapeDrawable>
#include <osg/Texture2D>
#include <osg/AutoTransform>
#include <osgDB/Registry>
#include <osgDB/FileNameUtils>
#include <osgDB/ReaderWriter>
#include <osgDB/ReadFile>
#include <osgUtil/Optimizer>
#include <fstream>

using namespace osgJuniper;

#define TARGET_POINTS_PER_LEAF 16
#define MIN_POINTS_PER_LEAF 4
#define RADIUS_MULT 35

struct MyComputeBoundCallback : public osg::Node::ComputeBoundingSphereCallback
{
    MyComputeBoundCallback(const osg::BoundingSphere& bs) : _bs(bs) { }
    osg::BoundingSphere computeBound(const osg::Node&) const { return _bs; }
    osg::BoundingSphere _bs;
};

struct KdTreeGraphBuilder : public osg::NodeVisitor
{
    KdTreeGraphBuilder():osg::NodeVisitor(osg::NodeVisitor::TRAVERSE_ALL_CHILDREN)
    {
        _root = new osg::Group();
        _tessHints = new osg::TessellationHints();
        _tessHints->setDetailRatio( 0.25f );
    }
    
    //override
    void apply( osg::Geode& geode )
    {
        for( unsigned int d=0; d<geode.getNumDrawables(); ++d )
        {
            osg::Drawable* dr = geode.getDrawable(d);
            KdTree* kd = dynamic_cast<KdTree*>( dr->getShape() );
            if ( kd )
            {
                osg::Node* child = createKdNodeGraph( kd->getNodes(), 0 );
                if ( child )
                    _root->addChild( child );
            }
        }
    }

    osg::Drawable* createDrawable( const osg::BoundingBox& bb )
    {
        double span = .33*((bb.xMax()-bb.xMin())+(bb.yMax()-bb.yMin())+(bb.zMax()-bb.zMin()));

        //return new new osg::ShapeDrawable(
        //    new osg::Sphere( kdNode.bb.center(), span ),
        //    _tessHints );

        /*return new osg::ShapeDrawable(
            new osg::Box( bb.center(), span ), _tessHints );*/

        return new osg::ShapeDrawable(
            new osg::Box( bb.center(), bb.xMax()-bb.xMin(),bb.yMax()-bb.yMin(),bb.zMax()-bb.zMin() ), _tessHints );


        //osg::Geometry* g = new osg::Geometry();
        //osg::Vec3Array* v = new osg::Vec3Array(1);
        //(*v)[0] = bb.center();
        //g->setVertexArray( v );
        //g->addPrimitiveSet( new osg::DrawArrays(GL_POINTS,0,1));        
        //g->getOrCreateStateSet()->setAttributeAndModes( new osg::Point(30), 1 );
        //g->setInitialBound( bb );
        //return g;
    }
    
    osg::Node* createKdNodeGraph( const KdTree::KdNodeList& nodes, unsigned int index )
    {
        const KdTree::KdNode& kdNode = nodes[index];

        if ( kdNode.first >= 0 || kdNode.second >= MIN_POINTS_PER_LEAF )
        {
            osg::Geode* geode = new osg::Geode();
            geode->addDrawable( createDrawable( kdNode.bb ) );

            if ( kdNode.first < 0 )
            {
                return geode;
            }
            else
            {
                osg::Node* firstChild = createKdNodeGraph( nodes, kdNode.first );
                osg::Node* secondChild = createKdNodeGraph( nodes, kdNode.second );

                osg::BoundingSphere bs;
                if ( firstChild )
                    bs.expandBy( firstChild->getBound() );
                if ( secondChild )
                    bs.expandBy( secondChild->getBound() );

                osg::LOD* lod = new osg::LOD();
                lod->addChild( geode, bs.radius()*RADIUS_MULT, FLT_MAX );
                if ( firstChild )
                    lod->addChild( firstChild, 0, bs.radius()*RADIUS_MULT );
                if ( secondChild )
                    lod->addChild( secondChild, 0, bs.radius()*RADIUS_MULT );

                return lod;
            }
        }
        else
        {
            return 0L;
        }
    }

    osg::Group* _root;
    osg::TessellationHints* _tessHints;
};


class KdTreeRenderer : public osgDB::ReaderWriter
{
public:
    KdTreeRenderer()
    {
        supportsExtension( "juniper_kdtree", className() );
    }

    //override
    const char* className()
    {
        return "KdTree Renderer for Juniper";
    }

    //override
    ReadResult readNode( const std::string& location, const osgDB::ReaderWriter::Options* options ) const
    {
        if ( !acceptsExtension( osgDB::getLowerCaseFileExtension( location ) ) )
            return ReadResult::FILE_NOT_HANDLED;

        // strip the pseudo-loader extension
        std::string subLocation = osgDB::getNameLessExtension( location );
        if ( subLocation.empty() )
            return ReadResult::FILE_NOT_HANDLED;

        // recursively load the subfile.
        osg::ref_ptr<osg::Node> node = osgDB::readNodeFile( subLocation, options );
        if( !node.valid() )
        {
            // propagate the read failure upwards
            osg::notify(osg::WARN) << "Subfile \"" << subLocation << "\" could not be loaded" << std::endl;
            return ReadResult::FILE_NOT_HANDLED;
        }

        osg::ref_ptr<KdTreeBuilder> kdBuilder = new KdTreeBuilder();
        kdBuilder->_buildOptions._targetNumVertsPerLeaf = TARGET_POINTS_PER_LEAF;
        node->accept( *kdBuilder.get() );

        KdTreeGraphBuilder graphBuilder;
        node->accept( graphBuilder );
        return graphBuilder._root;
    }
};

REGISTER_OSGPLUGIN(juniper_kdtree, KdTreeRenderer)


