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
#include "QKdTree"
#include <osg/Math>
#include <osg/Geometry>
#include <osg/Geode>
#include <osg/Point>
#include <osg/LOD>
#include <osg/Image>
#include <osg/ShapeDrawable>
#include <osgJuniper/PointCloud>
#include <osg/Texture2D>
#include <osg/AutoTransform>
#include <osg/AlphaFunc>
#include <osg/Timer>
#include <osgDB/Registry>
#include <osgDB/FileNameUtils>
#include <osgDB/ReaderWriter>
#include <osgDB/ReadFile>
#include <osgUtil/Optimizer>
#include <osg/io_utils>
#include <fstream>

#ifndef WIN32
#include <string.h>
#include <stdio.h>
#endif
using namespace osgJuniper;

#define TARGET_POINTS_PER_LEAF 500
#define MIN_POINTS_PER_LEAF 1

#define MINSIZE_MIN 0.01f
#define MINSIZE_MAX 100.0f
#define MINSIZE_REFINE_MULTIPLIER 0.7f

char basic_vertex_splat_source[] =
"void directionalLight(in int i, \n"
"                      in vec3 normal, \n"
"                      inout vec4 ambient, \n"
"                      inout vec4 diffuse, \n"
"                      inout vec4 specular) \n"
"{ \n"
"   float nDotVP;         // normal . light direction \n"
"   float nDotHV;         // normal . light half vector \n"
"   float pf;             // power factor \n"
" \n"
"   nDotVP = max(0.0, dot(normal, normalize(vec3 (gl_LightSource[i].position)))); \n"
"   nDotHV = max(0.0, dot(normal, vec3 (gl_LightSource[i].halfVector))); \n"
" \n"
"   if (nDotVP == 0.0) \n"
"   { \n"
"       pf = 0.0; \n"
"   } \n"
"   else \n"
"   { \n"
"       pf = pow(nDotHV, gl_FrontMaterial.shininess); \n"
" \n"
"   } \n"
"   ambient  += gl_LightSource[i].ambient; \n"
"   diffuse  += gl_LightSource[i].diffuse * nDotVP; \n"
"   specular += gl_LightSource[i].specular * pf; \n"
"} \n"
"\n"
"vec3 fnormal(void)\n"
"{\n"
"    //Compute the normal \n"
"    vec3 normal = gl_NormalMatrix * gl_Normal;\n"
"    normal = normalize(normal);\n"
"    return normal;\n"
"}\n"
"\n"
"\n"
"attribute vec4 splatAttribute; \n"
"uniform bool lightingEnabled; \n"
"uniform bool light0Enabled; \n"
"uniform bool colorEnabled; \n"
"uniform float pixelsPerRadian; \n"
"uniform vec4 frontPlane; \n"
"uniform float minSize; \n"
"\n"
"void main (void)\n"
"{\n"
"  vec4 color = vec4(1.0, 1.0, 1.0, 1.0);\n"
"  if (colorEnabled)\n"
"     color = gl_Color;\n"
"  if ((gl_NormalMatrix * gl_Normal).z <= 0.0) {\n"
"     gl_Position = vec4(10000, 10000, 0, 1);"
"     return;\n"
"  }\n"
"  if (lightingEnabled)\n"
"  {\n"
"     vec3 normal = fnormal(); \n"
"     vec4 ambient = vec4(0.0); \n"
"     vec4 diffuse = vec4(0.0); \n"
"     vec4 specular = vec4(0.0); \n"
"     if (light0Enabled) directionalLight(0, normal, ambient, diffuse, specular); \n"
"     gl_FrontColor = gl_FrontLightModelProduct.sceneColor +\n" 
"                    ambient  * gl_FrontMaterial.ambient +\n"
"                    diffuse  * (color * gl_FrontMaterial.diffuse) + \n"
"                    specular * gl_FrontMaterial.specular; \n"
"  }\n"
"  float splatSize = 2.0*pixelsPerRadian / dot(frontPlane, vec4(gl_Vertex.xyz, 1.0));\n"
"  gl_PointSize = splatAttribute.w * splatSize;\n"
"  gl_PointSize = 0.1 * splatSize;\n"
"  if (gl_PointSize < 1.0)\n"
"    gl_PointSize = 1.0;\n"
"  gl_Position = ftransform();\n"
"}\n";

char basic_fragment_splat_source[] =
"\n"
"void main (void)\n"
"{\n"
"   gl_FragColor = gl_Color;\n"
"}\n"
"";


char pass1_vertex_splat_source[] =
"\n"
"attribute vec4 splatAttribute; \n"
"uniform float pixelsPerRadian; \n"
"uniform vec4 frontPlane; \n"
"uniform bool lightingEnabled; \n"
"uniform bool light0Enabled; \n"
"uniform bool colorEnabled; \n"
"varying vec3 Normals; \n"
"\n"
"void directionalLight(in int i, \n"
"                      in vec3 normal, \n"
"                      inout vec4 ambient, \n"
"                      inout vec4 diffuse, \n"
"                      inout vec4 specular) \n"
"{ \n"
"   float nDotVP;         // normal . light direction \n"
"   float nDotHV;         // normal . light half vector \n"
"   float pf;             // power factor \n"
" \n"
"   nDotVP = max(0.0, dot(normal, normalize(vec3 (gl_LightSource[i].position)))); \n"
"   nDotHV = max(0.0, dot(normal, vec3 (gl_LightSource[i].halfVector))); \n"
" \n"
"   if (nDotVP == 0.0) \n"
"   { \n"
"       pf = 0.0; \n"
"   } \n"
"   else \n"
"   { \n"
"       pf = pow(nDotHV, gl_FrontMaterial.shininess); \n"
" \n"
"   } \n"
"   ambient  += gl_LightSource[i].ambient; \n"
"   diffuse  += gl_LightSource[i].diffuse * nDotVP; \n"
"   specular += gl_LightSource[i].specular * pf; \n"
"} \n"
"\n"
"vec3 fnormal(void)\n"
"{\n"
"    //Compute the normal \n"
"    vec3 normal = gl_NormalMatrix * gl_Normal;\n"
"    normal = normalize(normal);\n"
"    return normal;\n"
"}\n"
"\n"
"void main (void)\n"
"{\n"
"  vec4 color = vec4(1.0, 1.0, 1.0, 1.0);\n"
"  if (colorEnabled)\n"
"     color = gl_Color;\n"
"  if ((gl_NormalMatrix * gl_Normal).z < 0.0) {\n"
"     gl_Position = vec4(100000, 100000, 0, 1);"
"     return;\n"
"  }\n"
"  Normals = normalize(gl_Normal); \n"
"  if (lightingEnabled)\n"
"  {\n"
"     vec3 normal = normalize(gl_NormalMatrix * gl_Normal); \n"
"     vec4 ambient = vec4(0.0); \n"
"     vec4 diffuse = vec4(0.0); \n"
"     vec4 specular = vec4(0.0); \n"
"     if (light0Enabled) directionalLight(0, normal, ambient, diffuse, specular); \n"
"     gl_FrontColor = gl_FrontLightModelProduct.sceneColor +\n" 
"                    ambient  * gl_FrontMaterial.ambient +\n"
"                    diffuse  * (color * gl_FrontMaterial.diffuse) + \n"
"                    specular * gl_FrontMaterial.specular; \n"
"  } else { \n"
"     gl_FrontColor = color;\n"
"  }\n"
"  vec4 position = gl_ProjectionMatrix * gl_ModelViewMatrix * vec4(gl_Vertex.xyz, 1.0);\n"
"  float splatSize = pixelsPerRadian / dot(frontPlane, vec4(position.xyz, 1.0));\n"
"  gl_PointSize = splatAttribute.w * splatSize;\n"
"  gl_PointSize = 0.4;\n"
"  gl_Position = gl_Vertex;\n"
"}\n";


char pass1_geometry_splat_source[] =
"\n"
"#version 120\n"
"#extension GL_EXT_geometry_shader4 : enable\n"
"uniform mat4 MVP;\n"
"varying in vec3 Normals[];\n"
"\n"
"void main(void)\n"
"{\n"
"      int index = 0;\n"
"      vec3 normal = Normals[0];\n"
"      float value = abs(normal[0]);\n"
"      if (abs(normal[1]) < value) {\n"
"         value = abs(normal[1]);\n"
"         index = 1;\n"
"      }\n"
"      if (abs(normal[2]) < value) {\n"
"         value = abs(normal[2]);\n"
"         index = 2;\n"
"      }\n"
"      vec3 maxVec = vec3(0.0);\n"
"      if (normal[index] < 0) {\n"
"         maxVec[index] = -1.0;\n"
"      } else {\n"
"         maxVec[index] = 1.0;\n"
"      }\n"
""
"      vec3 other = normalize(cross(normal, maxVec));\n"
"      maxVec = normalize(cross(other,normal));\n"
"      float size = gl_PointSizeIn[0] * 0.5;\n"
"      vec4 usize = vec4(maxVec*size, 0.0);\n"
"      vec4 vsize = vec4(other*size, 0.0);\n"
"      //usize = vec4(maxVec, 0.0);\n"
"      //vsize = vec4(other, 0.0);\n"
"      //vec4 usize = vec4(1,0,0,0);\n"
"      //vec4 vsize = vec4(0,1,0,0);\n"
"      gl_Position = MVP*(gl_PositionIn[0] - usize + vsize);\n"
"      gl_TexCoord[0] = vec4(0,1,0,0);\n"
"      gl_FrontColor = gl_FrontColorIn[0];\n"
"      EmitVertex();\n"
"      gl_Position = MVP*(gl_PositionIn[0] + usize + vsize);\n"
"      gl_TexCoord[0] = vec4(1,1,0,0);\n"
"      gl_FrontColor = gl_FrontColorIn[0];\n"
"      EmitVertex();\n"
"      gl_Position = MVP*(gl_PositionIn[0] - usize - vsize);\n"
"      gl_TexCoord[0] = vec4(0,0,0,0);\n"
"      gl_FrontColor = gl_FrontColorIn[0];\n"
"      EmitVertex();\n"
"      gl_Position = MVP*(gl_PositionIn[0] + usize - vsize);\n"
"      gl_TexCoord[0] = vec4(1,0,0,0);\n"
"      gl_FrontColor = gl_FrontColorIn[0];\n"
"      EmitVertex();\n"
"      EndPrimitive();\n"
"\n"
"}\n"
"";


char pass1_fragment_splat_source[] =
"\n"
"uniform sampler2D Texture;\n"
"void main (void)\n"
"{\n"
"   vec4 texColor = texture2D(Texture, gl_TexCoord[0].xy);\n"
"   texColor.x *= texColor.w;\n"
"   texColor.y *= texColor.w;\n"
"   texColor.z *= texColor.w;\n"
"   //gl_FragColor = vec4(gl_Color.xyz,texColor.w) ;\n"
"   gl_FragColor = gl_Color * texColor ;\n"
"}\n"
"";


void GeometryGraph::setupGeometry()
{
    _attributeBinding = 11;

    _vertexes = new osg::Vec3Array(2); // avoid optimizer to bind it off
    _vertexes->push_back(osg::Vec3(0,0,0));
    _vertexes->push_back(osg::Vec3(0,0,0));
    _vertexes->setDataVariance(osg::Object::DYNAMIC);

    _normals = new osg::Vec3Array(2);
    _normals->setDataVariance(osg::Object::DYNAMIC);
    _attributes = new osg::Vec4Array(2);
    _attributes->setDataVariance(osg::Object::DYNAMIC);
    _colors = new osg::Vec4Array(2);
    _colors->setDataVariance(osg::Object::DYNAMIC);
    _drawarray = new osg::DrawArrays(GL_POINTS);
    _drawarray->setDataVariance(osg::Object::DYNAMIC);
    _geometry = new osg::Geometry;
    _geometry->setVertexArray(_vertexes.get());
    _geometry->setVertexAttribArray(_attributeBinding, _attributes.get());
    _geometry->setVertexAttribBinding(_attributeBinding, osg::Geometry::BIND_PER_VERTEX);
    _geometry->setVertexAttribNormalize(_attributeBinding, false);

    _geometry->setNormalArray(_normals.get());
    _geometry->setNormalBinding(osg::Geometry::BIND_PER_VERTEX);

    _geometry->setColorArray(_colors.get());
    _geometry->setColorBinding(osg::Geometry::BIND_PER_VERTEX);

    _geometry->getPrimitiveSetList().push_back(_drawarray.get());

    _material = new osg::Material;
    _material->setDiffuse(osg::Material::FRONT_AND_BACK, osg::Vec4(1,1,1,1));
    _geometry->getOrCreateStateSet()->setAttributeAndModes(_material.get());
    _geometry->setDataVariance(osg::Object::DYNAMIC);
    _geometry->setUseDisplayList(false);
    _geometry->setUseVertexBufferObjects(true);
}

void GeometryGraph::update(const ViewerInfo& vi)
{
    osg::StateSet* stateset = _stateset;
    stateset->getOrCreateUniform("pixelsPerRadian", osg::Uniform::FLOAT)->set(vi._pixelsPerRadian);
    stateset->getOrCreateUniform("frontPlane", osg::Uniform::FLOAT_VEC4)->set(vi._zproj.asVec4());
    stateset->getOrCreateUniform("minSize", osg::Uniform::FLOAT)->set(vi._minsize);
    stateset->getOrCreateUniform("colorEnabled", osg::Uniform::BOOL)->set(_colors->size() != 0);

    stateset->getOrCreateUniform("pixelsPerRadian", osg::Uniform::FLOAT)->dirty();
    stateset->getOrCreateUniform("colorEnabled", osg::Uniform::BOOL)->dirty();
    stateset->getOrCreateUniform("minSize", osg::Uniform::FLOAT)->dirty();
    stateset->getOrCreateUniform("frontPlane", osg::Uniform::FLOAT_VEC4)->dirty();
}


void GeometryGraph::addPoint(float zFromCamera, float cx, float cy, float cz,
                            float r, float splatsize,
                            const float *norm, const float *col)
{
    _vertexes->push_back(osg::Vec3(cx, cy, cz));
    _normals->push_back(osg::Vec3(norm[0], norm[1], norm[2]));
    _attributes->push_back(osg::Vec4(0, 0, 0, r ));
    if (col)
        _colors->push_back(osg::Vec4(col[0], col[1], col[2], 1.0));

}

void GeometryGraphBasic::createSceneGraph()
{
    setupGeometry();
    osg::Geode* geode = new osg::Geode;
    addChild(geode);
    geode->addDrawable(_geometry.get());

    geode->getOrCreateStateSet()->setMode(GL_LIGHTING, true);
    osg::StateSet* stateset = geode->getOrCreateStateSet();
    _stateset = stateset;
    stateset->setName("BasicRendering");
    stateset->setDataVariance(osg::Object::DYNAMIC);

    stateset->setAttributeAndModes(new osg::Point());
    stateset->setMode(GL_VERTEX_PROGRAM_POINT_SIZE, true);

    osg::Program* program = new osg::Program;
    program->addBindAttribLocation("splatAttribute", _attributeBinding);
    osg::Shader* vertShader = new osg::Shader(osg::Shader::VERTEX);
    osg::Shader* fragShader = new osg::Shader(osg::Shader::FRAGMENT);
    
    vertShader->setShaderSource(std::string(basic_vertex_splat_source));
    fragShader->setShaderSource(std::string(basic_fragment_splat_source));

    program->addShader(vertShader);
    program->addShader(fragShader);
    stateset->setAttributeAndModes(program);
    stateset->getOrCreateUniform("lightingEnabled", osg::Uniform::BOOL)->set(true);
    stateset->getOrCreateUniform("light0Enabled", osg::Uniform::BOOL)->set(true);
    stateset->getOrCreateUniform("colorEnabled", osg::Uniform::BOOL)->set(_colors->size() != 0);
    stateset->getOrCreateUniform("frontPlane", osg::Uniform::FLOAT_VEC4)->setDataVariance(osg::Object::DYNAMIC);
    stateset->getOrCreateUniform("pixelsPerRadian", osg::Uniform::FLOAT)->setDataVariance(osg::Object::DYNAMIC);

}


void GeometryGraphBasicOrientedSplat::createSceneGraph()
{
    setupGeometry();
    osg::Geode* geode = new osg::Geode;
    addChild(geode);
    geode->addDrawable(_geometry.get());

    geode->getOrCreateStateSet()->setMode(GL_LIGHTING, true);
    osg::StateSet* stateset = geode->getOrCreateStateSet();
    _stateset = stateset;
    stateset->setName("OrientedSplatRendering");
    stateset->setDataVariance(osg::Object::DYNAMIC);
    stateset->setAttributeAndModes(new osg::Point());
    stateset->setMode(GL_VERTEX_PROGRAM_POINT_SIZE, true);

    osg::Program* program = new osg::Program;
    program->addBindAttribLocation("splatAttribute", _attributeBinding);
    osg::Shader* vertShader = new osg::Shader(osg::Shader::VERTEX);
    osg::Shader* fragShader = new osg::Shader(osg::Shader::FRAGMENT);
    osg::Shader* geomShader = new osg::Shader(osg::Shader::GEOMETRY);
    
    vertShader->setShaderSource(std::string(pass1_vertex_splat_source));
    fragShader->setShaderSource(std::string(pass1_fragment_splat_source));
    geomShader->setShaderSource(std::string(pass1_geometry_splat_source));

    program->addShader(vertShader);
    program->addShader(fragShader);
    program->addShader(geomShader);

    program->setParameter( GL_GEOMETRY_VERTICES_OUT_EXT, 4 );
    program->setParameter( GL_GEOMETRY_INPUT_TYPE_EXT, GL_POINTS );
    program->setParameter( GL_GEOMETRY_OUTPUT_TYPE_EXT, GL_TRIANGLE_STRIP );
//    program->setParameter( GL_GEOMETRY_OUTPUT_TYPE_EXT, GL_POINTS );
    stateset->setAttributeAndModes(new osg::AlphaFunc(osg::AlphaFunc::GREATER, 0.5));
    stateset->setMode(GL_BLEND, false);

    stateset->setAttributeAndModes(program);
    stateset->getOrCreateUniform("lightingEnabled", osg::Uniform::BOOL)->set(true);
    stateset->getOrCreateUniform("light0Enabled", osg::Uniform::BOOL)->set(true);
    stateset->getOrCreateUniform("colorEnabled", osg::Uniform::BOOL)->set(_colors->size() != 0);
    stateset->getOrCreateUniform("frontPlane", osg::Uniform::FLOAT_VEC4)->setDataVariance(osg::Object::DYNAMIC);
    stateset->getOrCreateUniform("pixelsPerRadian", osg::Uniform::FLOAT)->setDataVariance(osg::Object::DYNAMIC);

    osg::Image* image = osgDB::readImageFile("texture.png");
    osg::Texture2D* texture = new osg::Texture2D(image);
    texture->setFilter(osg::Texture::MIN_FILTER, osg::Texture::LINEAR_MIPMAP_LINEAR);
    texture->setFilter(osg::Texture::MAG_FILTER, osg::Texture::LINEAR);
    stateset->setTextureAttributeAndModes(0, texture);
    stateset->getOrCreateUniform("Texture", osg::Uniform::INT)->set(0);;
    stateset->getOrCreateUniform("MVP", osg::Uniform::FLOAT_MAT4);
}

void GeometryGraphBasicOrientedSplat::update(const ViewerInfo& vi)
{
    GeometryGraphBasic::update(vi);
    osg::Vec3 lr, ur;
    ur[0] =  vi._modelview.ptr()[0] + vi._modelview.ptr()[1];  ur[1] =  vi._modelview.ptr()[4] + vi._modelview.ptr()[5];  ur[2] =  vi._modelview.ptr()[8] + vi._modelview.ptr()[9];
    lr[0] =  vi._modelview.ptr()[0] - vi._modelview.ptr()[1];  lr[1] =  vi._modelview.ptr()[4] - vi._modelview.ptr()[5];  lr[2] =  vi._modelview.ptr()[8] - vi._modelview.ptr()[9];

    _stateset->getOrCreateUniform("MVP", osg::Uniform::FLOAT_MAT4)->set(vi._modelview * vi._projection);
}

GeometryGraph* KdTreeSplatModel::createNewGeometry()
{
    GeometryGraph* gg = 0;
    //_rendering = ORIENTED_SPLAT;
    switch(_rendering) {
    case BASIC:
        gg = new GeometryGraphBasic;
        break;
    case ORIENTED_SPLAT:
        gg = new GeometryGraphBasicOrientedSplat;
        break;
    }
    if (gg) {
        gg->createSceneGraph();
        //gg->_geometry->setInitialBound(osg::BoundingBox(_center - osg::Vec3(_radius, _radius, _radius), _center + osg::Vec3(_radius, _radius, _radius)));
    }
    return gg;
}

void KdTreeSplatModel::setGeometry(GeometryGraph* geom)
{
    OpenThreads::ScopedLock<OpenThreads::Mutex> lock(_mutex);
    _geometryGraph = geom;
}

void KdTreeSplatModel::setupBuild(const ViewerInfo& vi, float minsize, bool frustrumCulling, bool backfaceCulling)
{
    OpenThreads::ScopedLock<OpenThreads::Mutex> lock(_mutex);
    _viewInfo = vi;
    _viewInfo._minsize = minsize;
    _frustumCulling = frustrumCulling;
    _backfaceCulling = backfaceCulling;
}

void KdTreeSplatModel::operator()(osg::Object*) 
{
    build(_frustumCulling, _backfaceCulling);
}


void KdTreeSplatModel::collect(int nodeIndex, float r, bool backfaceCulling, bool frustrumCulling)
{
    QKdTree::KdNode& node = _tree->getNode(nodeIndex);
    float size = node.bb.radius();
    if (_overallSplatSize) {
        QKdTree::KdNode& root = _tree->getNode(0);
        float maxRadius = root.bb.radius();
        size = size / maxRadius * MINSIZE_MAX;
    }

    if (size > _viewInfo._minsize) {
            
        // leaf node
        if (node.first < 0) {
            int index = -node.first;
            for (int i = 0; i < node.second; ++i) {
                int id = index + i;
                if (_tree->getNormals())
                    _geometryGraph->_normals->push_back((*_tree->getNormals())[id]);
                if (_tree->getColors())
                    _geometryGraph->_colors->push_back((*_tree->getColors())[id]);

                _geometryGraph->_attributes->push_back((*_tree->getSize())[id]);
                _geometryGraph->_vertexes->push_back((*_tree->getVertices())[id]);
            }
        } else {

            collect(node.first, r, backfaceCulling, frustrumCulling);
            collect(node.second, r, backfaceCulling, frustrumCulling);
        }

    } else {
#if 1
        if (node.index != -1) {
            // size is small enough so we collect it and stop to recurse
            int id = node.index;
            if (_tree->getNormals())
                _geometryGraph->_normals->push_back((*_tree->getNormals())[id]);
            if (_tree->getColors())
                _geometryGraph->_colors->push_back((*_tree->getColors())[id]);

            _geometryGraph->_attributes->push_back((*_tree->getSize())[id]);
            _geometryGraph->_vertexes->push_back((*_tree->getVertices())[id]);
        } else {
        }
#endif
    }
}

bool KdTreeSplatModel::build(bool frustrumCulling, bool backfaceCulling)
{
    OpenThreads::ScopedLock<OpenThreads::Mutex> lock(_mutex);
    
    // Prepare for drawing
    _geometryGraph->reset();
    _geometryGraph->_minsize = _viewInfo._minsize;

    int nodeIndex = 0;
    collect(0, nodeIndex, frustrumCulling, backfaceCulling);
    

    _geometryGraph->dirty();
    osg::notify(osg::NOTICE) << "Model has " << _geometryGraph->_vertexes->size() << " vertexes " << _geometryGraph->getBound().center() << " " << _geometryGraph->getBound().radius() <<  " for size " <<  _geometryGraph->_minsize << std::endl;

    return false;
}




class PointCloudKdtree : public osgJuniper::PointCloud
{
public:

    PointCloudKdtree() : osgJuniper::PointCloud()
    {
        setDataVariance(osg::Object::DYNAMIC);
        _previousTime = 0;
        _desiredRate = 30;
        _askRun = false;
        _isOnScreen = true;
        _minsize = MINSIZE_MAX;
        setCullingActive(false);
        _maxVertexes = 5000000;
        _checkRate = 0.5;
    }

    void setBuilder(KdTreeSplatModel* model) { _builder = model; }

    void traverse(osg::NodeVisitor& nv)
    {
        setNumChildrenRequiringUpdateTraversal(1);
        if (nv.getVisitorType() == osg::NodeVisitor::UPDATE_VISITOR) {
            if (_geometryGraph.valid()) {
                _geometryGraph->update(_viewerInfo);
            }
            select();

        } else if (nv.getVisitorType() == osg::NodeVisitor::CULL_VISITOR) {
            osg::CullStack* cs = dynamic_cast<osg::CullStack*>(&nv);
            _viewerInfo.compute(cs);
            if (_geometryGraph.valid())
                _isOnScreen = !cs->isCulled(_geometryGraph->getBound());
            else
                _isOnScreen = false;
        }
        osg::Group::traverse(nv);
    }

    void select() {
        // generate a mesh the first time
        osg::Timer_t now = osg::Timer::instance()->tick();
        if (_viewerInfo._modelview.isIdentity()) // wait to have done a cull before doing anything
            return;
        if (!_previousTime) {
            _operationThread->start();
            _previousTime = now;
            _minsize = refine();
            _builder->setupBuild(_viewerInfo, _minsize, false, false);
            _builder->setGeometry(_builder->createNewGeometry());
            _operationThread->add(_builder.get());
            _askRun = true;
        } else {
            double dt = osg::Timer::instance()->delta_s(_previousTime, now);
            _numFrames++;
            if (dt > _checkRate) {
                float fps = _numFrames / dt;
                // evaluate a time budget
                _previousTime = now;
                if (_isOnScreen) {
                    // 30 % of fps variation from the target will make the decision
                    if (fps > (_desiredRate + 0.3 * _desiredRate)  && !_askRun && canRefine()) {
                        float minsize = refine();
                        _builder->setupBuild(_viewerInfo, minsize, false, false);
                        _operationThread->add(_builder.get());
                        _askRun = true;
                        //osg::notify(osg::NOTICE) << "fps " << fps << " refine " << minsize << std::endl;
                    } else if (fps < (_desiredRate - 0.3*_desiredRate ) && !_askRun ) {
                        float minsize = define();
                        _builder->setupBuild(_viewerInfo, minsize, false, false);
                        _operationThread->add(_builder.get());
                        _askRun = true;
                        //osg::notify(osg::NOTICE) << "fps " << fps << " define " << minsize << std::endl;
                    } else if (!_askRun && false // !_builder->getSelectByDistFromCamera()
                        ) {
                        //osg::notify(osg::NOTICE) << "fps " << fps << " rebuild " << _minsize << std::endl;
                        _builder->setupBuild(_viewerInfo, _minsize, false, false);
                        _operationThread->add(_builder.get());
                        _askRun = true;
                    }
                }

                _numFrames = 0;
            }

            // operation finished update get the geometry
            if (_askRun && !_operationThread->getCurrentOperation() && _operationThread->getOperationQueue()->empty()) {
                osg::ref_ptr<GeometryGraph> newmesh = _builder->getGeometry();
                // too much discard it
                if (newmesh->_vertexes->size() > _maxVertexes) {
                    _askRun = false;
                    osg::notify(osg::NOTICE) << " too many vertexes (" << newmesh->_vertexes->size() << " ) discard it for size, " << newmesh->_minsize  << std::endl;
                } else {

                    osg::ref_ptr<GeometryGraph> old = _geometryGraph;
                    if (old.valid()) {
                        removeChild(_geometryGraph.get());
                    }
                    _geometryGraph = _builder->getGeometry();
                    addChild(_geometryGraph.get());

                    // change the geometry for the next build
                    if (!old.valid()) {
                        old = _builder->createNewGeometry();
                    }
                    _builder->setGeometry(old.get());
                    _askRun = false;
                    osg::notify(osg::NOTICE) << " changed mesh " << _geometryGraph.get() << std::endl;
                    _minsize = newmesh->_minsize;
                }
            }
        }
    }


// Set up state so that the next time this model is drawn it is at a
// higher resolution
    float refine()
    {
        float minsize = _minsize;
        minsize *= MINSIZE_REFINE_MULTIPLIER;
        minsize = floor(minsize);
        minsize = osg::minimum(osg::maximum(minsize, MINSIZE_MIN), MINSIZE_MAX);
        return minsize;
    }

    float define()
    {
        float minsize = _minsize;
        minsize *= 1.0/MINSIZE_REFINE_MULTIPLIER;
        minsize = ceil(minsize);
        minsize = osg::maximum(osg::minimum(minsize, MINSIZE_MAX), MINSIZE_MIN);
        return minsize;
    }
    bool canRefine()
    {
        if (_geometryGraph.valid()) {
            float size = 1.0 * _geometryGraph->_vertexes->size();
            size = size * 1.0/(MINSIZE_REFINE_MULTIPLIER*MINSIZE_REFINE_MULTIPLIER);
            osg::notify(osg::NOTICE) << "estimate size " << size << " of " << _maxVertexes << std::endl;
            float max = (float)_maxVertexes;
            if (size > max)
                return false;
        }
        if (_minsize <= MINSIZE_MIN)
            return false;
        return true; 
    }

protected:
    osg::ref_ptr<GeometryGraph> _geometryGraph;
    osg::ref_ptr<KdTreeSplatModel> _builder;
    osg::Timer_t _previousTime;
    unsigned int _numFrames;
    unsigned int _desiredRate;
    float _minsize;
    bool _askRun;
    bool _isOnScreen;
    ViewerInfo _viewerInfo;
    int _maxVertexes;
    float _checkRate;
};


struct PointCloudBuilder : public osg::NodeVisitor
{
    std::vector<osg::ref_ptr<osg::Group> > _pointClouds;
    KdTreeSplatModel::GeometryGraphType _option;
    void setOption(KdTreeSplatModel::GeometryGraphType option) { _option = option; }
    void apply(osg::Geode& geode) {
        for(unsigned int i=0; i<geode.getNumDrawables(); ++i) {            

            osg::Geometry* geom = geode.getDrawable(i)->asGeometry();
            if (geom) {
                osg::Shape* shape = geom->getShape();
                QKdTree* tree = dynamic_cast<QKdTree*>(shape);
                if (tree) {
                    PointCloudKdtree* pc = new PointCloudKdtree;
                    KdTreeSplatModel* ksm = new KdTreeSplatModel(tree);
                    ksm->setQuality(_option);
                    pc->setBuilder(ksm);
                    _pointClouds.push_back(pc);
                }
            }   
        }
    }    
};

class KdTreeSplatRenderer : public osgDB::ReaderWriter
{
public:
    KdTreeSplatRenderer()
    {
        supportsExtension( "juniper_kdtree_splat2", className() );
    }

    //override
    const char* className()
    {
        return "KdTree Splat2 Renderer for Juniper";
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

        PointCloudBuilder visitor;
        if (options && options->getOptionString() == "oriented" ) {
            osg::notify(osg::NOTICE) << "Use oriented splat" << std::endl;
            visitor.setOption(KdTreeSplatModel::ORIENTED_SPLAT);
        }

        node->accept(visitor);
        if (visitor._pointClouds.empty())
            return 0;

        osg::ref_ptr<osg::Group> grp = new osg::Group;
        for (int i = 0; i < visitor._pointClouds.size(); ++i) {
            grp->addChild(visitor._pointClouds[i].get());
        }


        return grp.release();
    }

};

REGISTER_OSGPLUGIN(juniper_kdtree_splat2, KdTreeSplatRenderer)
