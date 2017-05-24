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
#include <osgJuniper/KdTree>
#include <osg/Geode>
#include <osg/io_utils>
#include <osg/Version>

using namespace osgJuniper;

struct BuildKdTree
{
    BuildKdTree(KdTree& kdTree) : _kdTree(kdTree) {}

    typedef std::vector< osg::Vec3 >    CenterList;
    typedef std::vector< unsigned int > Indices;
    typedef std::vector< unsigned int > AxisStack;

    bool build(KdTree::BuildOptions& options, osg::Geometry* geometry);

    void computeDivisions(KdTree::BuildOptions& options);

    int divide(KdTree::BuildOptions& options, osg::BoundingBox& bb, int nodeIndex, unsigned int level);

    KdTree&             _kdTree;
    osg::BoundingBox    _bb;
    AxisStack           _axisStack;
    Indices             _primitiveIndices;

protected:

    BuildKdTree& operator = (const BuildKdTree&) { return *this; }
};

/************************************************************************/

bool BuildKdTree::build(KdTree::BuildOptions& options, osg::Geometry* geometry)
{
    
#ifdef VERBOSE_OUTPUT    
    osg::notify(osg::NOTICE)<<"osg::KDTreeBuilder::createKDTree()"<<std::endl;146
#endif

    osg::Vec3Array* vertices = dynamic_cast<osg::Vec3Array*>(geometry->getVertexArray());
    if (!vertices)
        return false;
    
    if (vertices->size() <= options._targetNumVertsPerLeaf)
        return false;

#if OSG_VERSION_GREATER_THAN(3,3,1)
    _bb = geometry->getBoundingBox();
#else
    _bb = geometry->getBound();
#endif
    _kdTree.setVertices(vertices);
    
    unsigned int estimatedSize = (unsigned int)(2.0*float(vertices->size())/float(options._targetNumVertsPerLeaf));

    _kdTree.getNodes().reserve(estimatedSize*5);
    
    computeDivisions(options);

    options._numVerticesProcessed += vertices->size();

    _primitiveIndices.reserve( vertices->size() );
    //unsigned int estimatedNumTriangles = vertices->size()*2;
    //_primitiveIndices.reserve(estimatedNumTriangles);
    //_centers.reserve(estimatedNumTriangles);

    //_kdTree.getTriangles().reserve(estimatedNumTriangles);

    //osg::TriangleIndexFunctor<TriangleIndicesCollector> collectTriangleIndices;
    //collectTriangleIndices._buildKdTree = this;
    //geometry->accept(collectTriangleIndices);
    for(unsigned int i=0; i<vertices->size(); ++i)
    {
        //_centers.push_back( (*vertices)[i] );
        _primitiveIndices.push_back( i );
    }


    //_primitiveIndices.reserve(vertices->size());

    KdTree::KdNode node(-1, _primitiveIndices.size());
    node.bb = _bb;

    int nodeNum = _kdTree.addNode(node);

    osg::BoundingBox bb = _bb;
    nodeNum = divide(options, bb, nodeNum, 0);
    
    // now reorder the triangle list so that it's in order as per the primitiveIndex list.
    //KdTree::TriangleList triangleList(_kdTree.getTriangles().size());
    //for(unsigned int i=0; i<_primitiveIndices.size(); ++i)
    //{
    //    triangleList[i] = _kdTree.getTriangle(_primitiveIndices[i]);
    //}
    //
    //_kdTree.getTriangles().swap(triangleList);
    
    
#ifdef VERBOSE_OUTPUT    
    osg::notify(osg::NOTICE)<<"Root nodeNum="<<nodeNum<<std::endl;
#endif
    
    
//    osg::notify(osg::NOTICE)<<"_kdNodes.size()="<<k_kdNodes.size()<<"  estimated size = "<<estimatedSize<<std::endl;
//    osg::notify(osg::NOTICE)<<"_kdLeaves.size()="<<_kdLeaves.size()<<"  estimated size = "<<estimatedSize<<std::endl<<std::endl;


    return !_kdTree.getNodes().empty();
}

void BuildKdTree::computeDivisions(KdTree::BuildOptions& options)
{
    osg::Vec3 dimensions(_bb.xMax()-_bb.xMin(),
                         _bb.yMax()-_bb.yMin(),
                         _bb.zMax()-_bb.zMin());

#ifdef VERBOSE_OUTPUT    
    osg::notify(osg::NOTICE)<<"computeDivisions("<<options._maxNumLevels<<") "<<dimensions<< " { "<<std::endl;
#endif

    _axisStack.reserve(options._maxNumLevels);
 
    for(unsigned int level=0; level<options._maxNumLevels; ++level)
    {
        int axis = 0;
        if (dimensions[0]>=dimensions[1])
        {
            if (dimensions[0]>=dimensions[2]) axis = 0;
            else axis = 2;
        }
        else if (dimensions[1]>=dimensions[2]) axis = 1;
        else axis = 2;

        _axisStack.push_back(axis);
        dimensions[axis] /= 2.0f;

#ifdef VERBOSE_OUTPUT    
        osg::notify(osg::NOTICE)<<"  "<<level<<", "<<dimensions<<", "<<axis<<std::endl;
#endif
    }

#ifdef VERBOSE_OUTPUT    
    osg::notify(osg::NOTICE)<<"}"<<std::endl;
#endif
}

int BuildKdTree::divide(KdTree::BuildOptions& options, osg::BoundingBox& bb, int nodeIndex, unsigned int level)
{
    KdTree::KdNode& node = _kdTree.getNode(nodeIndex);

    bool needToDivide = level < _axisStack.size() &&
        (node.first<0 && static_cast<unsigned int>(node.second)>options._targetNumVertsPerLeaf);
                        
    if (!needToDivide)
    {
        if (node.first<0)
        {
            int istart = -node.first-1;
            int iend = istart+node.second-1;
    
            // leaf is done, now compute bound on it.
            node.bb.init();
            for(int i=istart; i<=iend; ++i)
            {
                const osg::Vec3& v = (*_kdTree.getVertices())[_primitiveIndices[i]];
                node.bb.expandBy(v);
                //const KdTree::Triangle& tri = _kdTree.getTriangle(_primitiveIndices[i]);
                //const osg::Vec3& v0 = (*_kdTree.getVertices())[tri.p0];
                //const osg::Vec3& v1 = (*_kdTree.getVertices())[tri.p1];
                //const osg::Vec3& v2 = (*_kdTree.getVertices())[tri.p2];
                //node.bb.expandBy(v0);
                //node.bb.expandBy(v1);
                //node.bb.expandBy(v2);                
            }

            if (node.bb.valid())
            {
                float epsilon = 1e-6f;
                node.bb._min.x() -= epsilon;
                node.bb._min.y() -= epsilon;
                node.bb._min.z() -= epsilon;
                node.bb._max.x() += epsilon;
                node.bb._max.y() += epsilon;
                node.bb._max.z() += epsilon;
            }
            
#ifdef VERBOSE_OUTPUT    
            if (!node.bb.valid())
            {
                osg::notify(osg::NOTICE)<<"After reset "<<node.first<<","<<node.second<<std::endl;
                osg::notify(osg::NOTICE)<<"  bb._min ("<<node.bb._min<<")"<<std::endl;
                osg::notify(osg::NOTICE)<<"  bb._max ("<<node.bb._max<<")"<<std::endl;
            }
            else
            {
                osg::notify(osg::NOTICE)<<"Set bb for nodeIndex = "<<nodeIndex<<std::endl;
            }
#endif
        }

        return nodeIndex;

    }

    int axis = _axisStack[level];

#ifdef VERBOSE_OUTPUT    
    osg::notify(osg::NOTICE)<<"divide("<<nodeIndex<<", "<<level<< "), axis="<<axis<<std::endl;
#endif

    if (node.first<0)
    {    
        // leaf node as first <= 0, so look at dividing it.
        
        int istart = -node.first-1;
        int iend = istart+node.second-1;

        //osg::notify(osg::NOTICE)<<"  divide leaf"<<std::endl;
        
        float original_min = bb._min[axis];
        float original_max = bb._max[axis];

        float mid = (original_min+original_max)*0.5f;

        int originalLeftChildIndex = 0;
        int originalRightChildIndex = 0;
        bool insitueDivision = false;

        {
            const osg::Vec3Array* vertices = _kdTree.getVertices();
            int left = istart;
            int right = iend;
            
            while(left<right)
            {
                //while(left<right && (_centers[_primitiveIndices[left]][axis]<=mid)) { ++left; }
                while(left<right && ( (*vertices)[_primitiveIndices[left]][axis]<=mid)) { ++left; }

                //while(left<right && (_centers[_primitiveIndices[right]][axis]>mid)) { --right; }
                while(left<right && ( (*vertices)[_primitiveIndices[right]][axis]>mid)) { --right; }
                
                //while(left<right && (_centers[_primitiveIndices[right]][axis]>mid)) { --right; }
                //while(left<right && ( (*vertices)[_primitiveIndices[right]][axis]>mid)) { --right; }

                if (left<right)
                {
                    std::swap(_primitiveIndices[left], _primitiveIndices[right]);
                    ++left;
                    --right;
                }
            }
            
            if (left==right)
            {
                if ( (*vertices)[_primitiveIndices[left]][axis]<=mid) ++left;
                //if (_centers[_primitiveIndices[left]][axis]<=mid) ++left;
                else --right;
            }
            
            KdTree::KdNode leftLeaf(-istart-1, (right-istart)+1);
            KdTree::KdNode rightLeaf(-left-1, (iend-left)+1);

#if 0
            osg::notify(osg::NOTICE)<<"In  node.first     ="<<node.first     <<" node.second     ="<<node.second<<std::endl;
            osg::notify(osg::NOTICE)<<"    leftLeaf.first ="<<leftLeaf.first <<" leftLeaf.second ="<<leftLeaf.second<<std::endl;
            osg::notify(osg::NOTICE)<<"    rightLeaf.first="<<rightLeaf.first<<" rightLeaf.second="<<rightLeaf.second<<std::endl;
            osg::notify(osg::NOTICE)<<"    left="<<left<<" right="<<right<<std::endl;

            if (node.second != (leftLeaf.second +rightLeaf.second))
            {
                osg::notify(osg::NOTICE)<<"*** Error in size, leaf.second="<<node.second
                                        <<", leftLeaf.second="<<leftLeaf.second
                                        <<", rightLeaf.second="<<rightLeaf.second<<std::endl;
            }
            else
            {
                osg::notify(osg::NOTICE)<<"Size OK, leaf.second="<<node.second
                                        <<", leftLeaf.second="<<leftLeaf.second
                                        <<", rightLeaf.second="<<rightLeaf.second<<std::endl;
            }
#endif

            if (leftLeaf.second<=0)
            {
                //osg::notify(osg::NOTICE)<<"LeftLeaf empty"<<std::endl;
                originalLeftChildIndex = 0;
                //originalRightChildIndex = addNode(rightLeaf);
                originalRightChildIndex = nodeIndex;
                insitueDivision = true;
            }
            else if (rightLeaf.second<=0)
            {
                //osg::notify(osg::NOTICE)<<"RightLeaf empty"<<std::endl;
                // originalLeftChildIndex = addNode(leftLeaf);
                originalLeftChildIndex = nodeIndex;
                originalRightChildIndex = 0;
                insitueDivision = true;
            }
            else
            {
                originalLeftChildIndex = _kdTree.addNode(leftLeaf);
                originalRightChildIndex = _kdTree.addNode(rightLeaf);
            }
        }

        
        float restore = bb._max[axis];
        bb._max[axis] = mid;

        //osg::notify(osg::NOTICE)<<"  divide leftLeaf "<<kdTree.getNode(nodeNum).first<<std::endl;
        int leftChildIndex = originalLeftChildIndex!=0 ? divide(options, bb, originalLeftChildIndex, level+1) : 0;

        bb._max[axis] = restore;
        
        restore = bb._min[axis];
        bb._min[axis] = mid;

        //osg::notify(osg::NOTICE)<<"  divide rightLeaf "<<kdTree.getNode(nodeNum).second<<std::endl;
        int rightChildIndex = originalRightChildIndex!=0 ? divide(options, bb, originalRightChildIndex, level+1) : 0;
        
        bb._min[axis] = restore;
        

        if (!insitueDivision)
        {
            // take a second reference to node we are working on as the std::vector<> resize could
            // have invalidate the previous node ref.
            KdTree::KdNode& newNodeRef = _kdTree.getNode(nodeIndex);
        
            newNodeRef.first = leftChildIndex;
            newNodeRef.second = rightChildIndex; 

            insitueDivision = true;

            newNodeRef.bb.init();
            if (leftChildIndex!=0) newNodeRef.bb.expandBy(_kdTree.getNode(leftChildIndex).bb);
            if (rightChildIndex!=0) newNodeRef.bb.expandBy(_kdTree.getNode(rightChildIndex).bb);
        }
    }
    else
    {
        osg::notify(osg::NOTICE)<<"NOT expecting to get here"<<std::endl;
    }
    
    return nodeIndex;    
}

KdTree::BuildOptions::BuildOptions():
        _numVerticesProcessed(0),
        _targetNumVertsPerLeaf(32),
        _maxNumLevels(32)
{
}

/************************************************************************/

KdTree::KdTree()
{
}

KdTree::KdTree(const KdTree& rhs, const osg::CopyOp& copyop):
    Shape(rhs, copyop),
    _vertices(rhs._vertices),
    _kdNodes(rhs._kdNodes)
{
}

bool KdTree::build(BuildOptions& options, osg::Geometry* geometry)
{
    BuildKdTree build(*this);
    return build.build(options, geometry);
}

/************************************************************************/

KdTreeBuilder::KdTreeBuilder():
    osg::NodeVisitor(osg::NodeVisitor::TRAVERSE_ALL_CHILDREN)
{            
    _kdTreePrototype = new KdTree;
}

KdTreeBuilder::KdTreeBuilder(const KdTreeBuilder& rhs):
    osg::NodeVisitor(osg::NodeVisitor::TRAVERSE_ALL_CHILDREN),
    _buildOptions(rhs._buildOptions),
    _kdTreePrototype(rhs._kdTreePrototype)
{
    //nop
}

void KdTreeBuilder::apply(osg::Geode& geode)
{
    for(unsigned int i=0; i<geode.getNumDrawables(); ++i)
    {            

        osg::Geometry* geom = geode.getDrawable(i)->asGeometry();
        if (geom)
        {
            KdTree* previous = dynamic_cast<KdTree*>(geom->getShape());
            if (previous) continue;

            osg::ref_ptr<KdTree> kdTree = dynamic_cast<KdTree*>(_kdTreePrototype->cloneType());

            if (kdTree->build(_buildOptions, geom))
            {
                geom->setShape(kdTree.get());
            }
        }   
    }
}
