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
#include <osg/io_utils>
#include <osg/Geometry>
#include <osg/Array>
#include <osg/ColorMask>
#include <osg/Geode>
#include <osg/ClearNode>
#include <osg/Notify>
#include <osg/PolygonOffset>
#include <osg/MatrixTransform>
#include <osg/Texture2D>
#include <osg/BlendFunc>
#include <osg/Depth>
#include <osg/Material>
#include <osg/TextureRectangle>
#include <osg/AlphaFunc>
#include <osgDB/Registry>
#include <osgDB/FileNameUtils>
#include <osgDB/ReaderWriter>
#include <osgDB/ReadFile>
#include <osgUtil/SmoothingVisitor>
#include <osg/CullStack>
#include <fstream>
#include <vector>

#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <string.h>

#include <osg/Point>


#include "qsplat_util.h"
#include "qsplat_normquant.h"
#include "qsplat_spherequant.h"
#include "qsplat_colorquant.h"

#ifdef WIN32
# include <windows.h>
# include <winbase.h>
# include <io.h>
#else
# include <unistd.h>
# include <sys/mman.h>
# define CloseHandle close
#endif

#include "QSplat"

// A few random #defines
#define QSPLAT_MAGIC "QSplat"
#define MINSIZE_MIN 0.5f
#define MINSIZE_MAX 50.0f
#define MINSIZE_REFINE_MULTIPLIER 0.7f


#define QSPLAT_FILE_VERSION 11
#define FAST_CUTOFF_FACTOR 2.3f


using namespace osgJuniper;

char pass1_vertex_splat_source[] =
"\n"
"attribute vec4 splatAttribute; \n"
"\n"
"void main (void)\n"
"{\n"
"  gl_PointSize = splatAttribute.w;\n"
"  vec4 position = gl_ProjectionMatrix * gl_ModelViewMatrix * gl_Vertex;\n"
"  gl_FrontColor = vec4(position.z, position.z, position.z, 1.0);\n"
"  gl_Position = gl_Vertex;\n"
"}\n";

char pass1_fragment_splat_source[] =
"\n"
"uniform sampler2D Texture;\n"
"void main (void)\n"
"{\n"
"   vec4 texColor = texture2D(Texture, gl_TexCoord[0].xy);\n"
"   texColor.x *= texColor.w;\n"
"   texColor.y *= texColor.w;\n"
"   texColor.z *= texColor.w;\n"
    "   gl_FragColor = vec4(gl_Color.xyz,texColor.w) ;\n"
"   //gl_FragColor = gl_Color * texColor ;\n"
"}\n"
"";

char pass1_geometry_splat_source[] =
"\n"
"#version 120\n"
"#extension GL_EXT_geometry_shader4 : enable\n"
"uniform vec4 uvec;\n"
"uniform vec4 vvec;\n"
"uniform mat4 MVP;\n"
"\n"
"void main(void)\n"
"{\n"
"      float size = gl_PointSizeIn[0] * 1.0;\n"
"      vec4 usize = uvec*size;\n"
"      vec4 vsize = vvec*size;\n"
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




//=======================================

char pass2_vertex_splat_source[] =
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
"  float splatSize = pixelsPerRadian / dot(frontPlane, vec4(gl_Vertex.xyz, 1.0));\n"
"  gl_PointSize = splatAttribute.w * 1.0;\n"
"  gl_Position = gl_Vertex;\n"
"}\n";

char pass2_fragment_splat_source[] =
"\n"
"uniform sampler2D Texture;\n"
"void main (void)\n"
"{\n"
"   vec4 texColor = texture2D(Texture, gl_TexCoord[0].xy);\n"
"   //texColor.x *= texColor.w;\n"
"   //texColor.y *= texColor.w;\n"
"   //texColor.z *= texColor.w;\n"
"   gl_FragColor = gl_Color * texColor ;\n"
"}\n"
"";



char pass2_geometry_splat_source[] =
"\n"
"#version 120\n"
"#extension GL_EXT_geometry_shader4 : enable\n"
"uniform vec4 uvec;\n"
"uniform vec4 vvec;\n"
"uniform mat4 MVP;\n"
"\n"
"void main(void)\n"
"{\n"
"      float size = gl_PointSizeIn[0] * 1.0;\n"
"      vec4 usize = uvec*size;\n"
"      vec4 vsize = vvec*size;\n"
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
"  if (gl_PointSize < 2.0)\n"
"    gl_PointSize = 2.0;\n"
"  gl_Position = ftransform();\n"
"}\n";

char basic_fragment_splat_source[] =
"\n"
"void main (void)\n"
"{\n"
"   gl_FragColor = gl_Color;\n"
"}\n"
"";


//=======================================================

char deferred_zpass_vertex_splat_source[] =
"\n"
"attribute vec4 splatAttribute; \n"
"uniform float pixelsPerRadian; \n"
"uniform vec4 frontPlane; \n"
"\n"
"void main (void)\n"
"{\n"
"  if ((gl_NormalMatrix * gl_Normal).z < 0.0) {\n"
"     gl_Position = vec4(10000, 10000, 0, 1);"
"     return;\n"
"  }\n"
"  float splatSize = 2.0*pixelsPerRadian / dot(frontPlane, vec4(gl_Vertex.xyz, 1.0));\n"
"  gl_PointSize = splatAttribute.w * splatSize;\n"
"  if (gl_PointSize < 2.0)\n"
"    gl_PointSize = 2.0;\n"
"  float z = (gl_ProjectionMatrix * gl_ModelViewMatrix * gl_Vertex).z;\n"
"  //gl_FrontColor = vec4(z,z,z,1);\n"
"  gl_Position = ftransform();\n"
"  gl_Position.z += .000001;\n" // offset to write in zbuffer
"}\n";

char deferred_zpass_fragment_splat_source[] =
"\n"
"void main (void)\n"
"{\n"
"   gl_FragColor = gl_Color;\n"
"}\n"
"";

char deferred_attributes_pass_vertex_splat_source[] =
"attribute vec4 splatAttribute; \n"
"uniform bool colorEnabled; \n"
"uniform float pixelsPerRadian; \n"
"uniform vec4 frontPlane; \n"
"varying vec4 normal; \n"
"varying vec4 color; \n"
"\n"
"void fnormal(void)\n"
"{\n"
"    //Compute the normal \n"
"    vec3 normalt = gl_NormalMatrix * gl_Normal;\n"
"    normal = vec4(normalize(normalt),1);\n"
"}\n"
"\n"
"void main (void)\n"
"{\n"
"  if ((gl_NormalMatrix * gl_Normal).z < 0.0) {\n"
"     gl_Position = vec4(10000, 10000, 0, 1);"
"     return;\n"
"  }\n"
"  color = vec4(1.0, 1.0, 1.0, 1.0);\n"
"  if (colorEnabled)\n"
"     color = gl_Color;\n"
"  fnormal();\n"
"  float splatSize = 2.0*pixelsPerRadian / dot(frontPlane, vec4(gl_Vertex.xyz, 1.0));\n"
"  gl_PointSize = splatAttribute.w * splatSize;\n"
"  if (gl_PointSize < 2.0)\n"
"    gl_PointSize = 2.0;\n"
"  gl_Position = ftransform();\n"
"}\n";

char deferred_attributes_pass_fragment_splat_source[] =
"\n"
"uniform bool colorEnabled; \n"
"varying vec4 normal; \n"
"varying vec4 color; \n"
"void main (void)\n"
"{\n"
"   gl_FragData[0] = normal;\n"
"   if (colorEnabled)\n"
"      gl_FragData[1] = color;\n"
"}\n"
"";



char deferred_final_pass_vertex_splat_source[] =
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
"\n"
"void main (void)\n"
"{\n"
"  gl_TexCoord[0] = gl_MultiTexCoord0;\n"
"  gl_Position = ftransform();\n"
"}\n";

char deferred_final_pass_fragment_splat_source[] =
"\n"
"#version 120\n"
"#extension GL_ARB_texture_rectangle : enable\n"
"uniform sampler2DRect textureID0;\n"
"uniform sampler2DRect textureID1;\n"
"uniform bool colorEnabled;\n"
"uniform vec3 lightDirection;\n"
"uniform vec4 lightAmbient;\n"
"uniform vec4 lightDiffuse;\n"
"uniform vec4 lightSpecular;\n"
"uniform vec3 lightHalfVector;\n"
"uniform vec4 materialAmbient;\n"
"uniform vec4 materialDiffuse;\n"
"uniform vec4 materialSpecular;\n"
"uniform float materialShininess;\n"
"\n"
"vec4 filter(sampler2DRect sampler) {\n"
"return texture2DRect(sampler, gl_TexCoord[0].st);\n"
"float step_w = 1.0;\n"
"float step_h = 1.0;\n"
"vec2 offset[9];\n"
"offset[0] = vec2(-step_w, -step_h);\n"
"offset[1] = vec2(0.0, -step_h);\n"
"offset[2] = vec2(step_w, -step_h);\n"
"offset[3] = vec2(-step_w, 0.0);\n"
"offset[4] = vec2(0.0, 0.0);\n"
"offset[5] = vec2(step_w, 0.0);\n"
"offset[6] = vec2(-step_w, step_h);\n"
"offset[7] = vec2(0.0, step_h);\n"
"offset[8] = vec2(step_w, step_h);\n"
"    vec4 sum = vec4(0.0, 0.0, 0.0, 0.0);\n"
"	   for( int i=0; i<9; i++ )\n"
"	   {\n"
"			vec4 tmp = texture2DRect(sampler, gl_TexCoord[0].st + offset[i]);\n"
"			sum += tmp;\n"
"	   }\n"
"  return sum;\n"
"\n" 
"}\n"
"\n"
"vec4 ambient;\n"
"vec4 diffuse;\n"
"vec4 specular;\n"
"\n"
"void directionalLight(in vec3 normal) \n"
"{ \n"
"   float nDotVP;         // normal . light direction \n"
"   float nDotHV;         // normal . light half vector \n"
"   float pf;             // power factor \n"
" \n"
"   //nDotVP = max(0.0, dot(normal, lightDirection)); \n"
"   //nDotHV = max(0.0, dot(normal, lightHalfVector)); \n"
"   vec3 mylightDirection = vec3(0,1,1);\n"
"   mylightDirection = normalize(mylightDirection);\n"
"   nDotVP = max(0.0, dot(normal, mylightDirection)); \n"
"   nDotHV = 0; \n"
" \n"
"   if (nDotVP == 0.0) \n"
"   { \n"
"       pf = 0.0; \n"
"   } \n"
"   else \n"
"   { \n"
"       pf = pow(nDotHV, materialShininess); \n"
" \n"
"   } \n"
"   ambient  += materialAmbient * lightAmbient; \n"
"   diffuse  += materialDiffuse * lightDiffuse * nDotVP; \n"
"   specular += lightSpecular * pf; \n"
"} \n"
"void main (void)\n"
"{\n"
"   vec4 tmp = texture2DRect(textureID0, gl_TexCoord[0].st);\n"
"   if (dot(tmp, tmp) < 0.001) {\n"
"      discard;\n"
"      return;\n"
"   }\n"
"\n"
"   vec3 averageNormal = filter(textureID0).rgb;\n"
"   averageNormal = normalize(averageNormal);\n"
"   ambient = vec4(0,0,0,0);\n"
"   diffuse = vec4(0,0,0,0);\n"
"   specular = vec4(0,0,0,0);\n"
"   directionalLight(averageNormal);\n"
"   vec4 color = vec4(1,1,1,1);\n"
"   if (colorEnabled) {\n"
"			 //color = texture2DRect(textureID1, gl_TexCoord[0].st);\n"
"      color = filter(textureID1).rgba;\n"
"      //if (color.a > 0)\n"
"         color = color * 1.0/color.a;\n"
"      //color = vec4(normalize(color.rgb), 1);\n"
"      color.a = 1.0;\n"
"      //color = vec4(1,0,1,1);\n"
"   }\n"
"   color = ambient + diffuse * color;\n"
"   //color = color;\n"
"   gl_FragColor = color;\n"
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
    _geometry->setVertexAttribData(_attributeBinding, osg::Geometry::ArrayData(_attributes.get(),osg::Geometry::BIND_PER_VERTEX));

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


void GeometryGraphDeferredShading::createSceneGraph()
{
    #define NUM_TEXTURES 2
    _texWidth = 1920;
    _texHeight = 1080;

    bool useFloat = true;
    // textures to render to and to use for texturing of the final quad
    osg::TextureRectangle* textureRect[NUM_TEXTURES];
    
    for (int i=0;i<NUM_TEXTURES;i++) {
        textureRect[i] = new osg::TextureRectangle;
        textureRect[i]->setTextureSize(_texWidth, _texHeight);
        textureRect[i]->setInternalFormat(GL_RGBA);
        textureRect[i]->setFilter(osg::Texture2D::MIN_FILTER,osg::Texture2D::LINEAR);
        textureRect[i]->setFilter(osg::Texture2D::MAG_FILTER,osg::Texture2D::LINEAR);
        textureRect[i]->setWrap(osg::Texture::WRAP_S,osg::Texture::CLAMP_TO_BORDER);
        textureRect[i]->setWrap(osg::Texture::WRAP_T,osg::Texture::CLAMP_TO_BORDER);

        if (useFloat)
        {
            // Default HDR format
            //textureRect[i]->setInternalFormat(GL_RGBA32F_ARB);

            // GL_FLOAT_RGBA32_NV might be supported on pre 8-series GPUs
            //textureRect[i]->setInternalFormat(GL_FLOAT_RGBA32_NV);

            // GL_RGBA16F_ARB can be used with this example, 
            // but modify e-12 and e12 in the shaders accordingly
            textureRect[i]->setInternalFormat(GL_RGBA16F_ARB);
            
            textureRect[i]->setSourceFormat(GL_RGBA);
            textureRect[i]->setSourceType(GL_FLOAT);
        }
    }

    setupGeometry();
    osg::Uniform* colorEnabled = _geometry->getOrCreateStateSet()->getOrCreateUniform("colorEnabled", osg::Uniform::BOOL);
    osg::Uniform* lightingEnabled = _geometry->getOrCreateStateSet()->getOrCreateUniform("lightingEnabled", osg::Uniform::BOOL);
    osg::Uniform* light0Enabled = _geometry->getOrCreateStateSet()->getOrCreateUniform("light0Enabled", osg::Uniform::BOOL);
    osg::Uniform* frontPlane = _geometry->getOrCreateStateSet()->getOrCreateUniform("frontPlane", osg::Uniform::FLOAT_VEC4);
    osg::Uniform* pixelsPerRadian =  _geometry->getOrCreateStateSet()->getOrCreateUniform("pixelsPerRadian", osg::Uniform::FLOAT);
    

    _geometry->getOrCreateStateSet()->setName("DeferredRendering");

    osg::Geode* geode = new osg::Geode;
    osg::Geode* geodeZpass = geode;
    geode->addDrawable(_geometry.get());
    geode->getOrCreateStateSet()->setAttributeAndModes(new osg::ColorMask(false, false, false, false));

    osg::StateSet* stateset = geode->getOrCreateStateSet();
    stateset->setDataVariance(osg::Object::DYNAMIC);
    stateset->setAttributeAndModes(new osg::Point());
    stateset->setMode(GL_VERTEX_PROGRAM_POINT_SIZE, true);
    stateset->setMode(GL_DEPTH_TEST, true);
    stateset->setAttributeAndModes(new osg::AlphaFunc(osg::AlphaFunc::GREATER, 0.9));
    stateset->addUniform(colorEnabled);
    stateset->addUniform(lightingEnabled);
    stateset->addUniform(light0Enabled);
    stateset->addUniform(frontPlane);
    stateset->addUniform(pixelsPerRadian);

    osg::Viewport* vp = new osg::Viewport(0,0, _texWidth, _texHeight);

    osg::Program* program = new osg::Program;
    program->addBindAttribLocation("splatAttribute", _attributeBinding);
    osg::Shader* vertShader = new osg::Shader(osg::Shader::VERTEX);
    osg::Shader* fragShader = new osg::Shader(osg::Shader::FRAGMENT);
    
    vertShader->setShaderSource(std::string(deferred_zpass_vertex_splat_source));
    fragShader->setShaderSource(std::string(deferred_zpass_fragment_splat_source));

    program->addShader(vertShader);
    program->addShader(fragShader);
    stateset->setAttributeAndModes(program);

    {
        osg::Geode* geode = new osg::Geode;
        geode->addDrawable(_geometry.get());
        osg::StateSet* stateset = geode->getOrCreateStateSet();

        stateset->setDataVariance(osg::Object::DYNAMIC);

        stateset->setAttributeAndModes(new osg::Point());
        stateset->setMode(GL_VERTEX_PROGRAM_POINT_SIZE, true);

        osg::Program* program = new osg::Program;
        program->addBindAttribLocation("splatAttribute", _attributeBinding);
        osg::Shader* vertShader = new osg::Shader(osg::Shader::VERTEX);
        osg::Shader* fragShader = new osg::Shader(osg::Shader::FRAGMENT);
        osg::Depth* depth = new osg::Depth;
        depth->setFunction(osg::Depth::LEQUAL);
        depth->setWriteMask(false);
        stateset->setAttributeAndModes(depth);
        stateset->setMode(GL_DEPTH_TEST, true);
        stateset->setAttributeAndModes(new osg::AlphaFunc(osg::AlphaFunc::GREATER, 0.9));

        
        vertShader->setShaderSource(std::string(deferred_attributes_pass_vertex_splat_source));
        fragShader->setShaderSource(std::string(deferred_attributes_pass_fragment_splat_source));

        program->addShader(vertShader);
        program->addShader(fragShader);
        stateset->setAttributeAndModes(program);
        geode->getOrCreateStateSet()->setMode(GL_BLEND, true);
        geode->getOrCreateStateSet()->setAttributeAndModes(new osg::BlendFunc(GL_ONE,GL_ONE));
        stateset->addUniform(colorEnabled);
        stateset->addUniform(lightingEnabled);
        stateset->addUniform(frontPlane);
        stateset->addUniform(pixelsPerRadian);
        stateset->addUniform(light0Enabled);

        osg::Camera* camera = new osg::Camera;
        camera->addChild(geodeZpass);
        camera->addChild(geode);
        // tell the camera to use OpenGL frame buffer objects
        camera->setRenderTargetImplementation(osg::Camera::FRAME_BUFFER_OBJECT);
        camera->setViewport(vp);
        camera->setClearColor(osg::Vec4(0.0f,0.0f,0.0f,0.0f));
        camera->setClearMask(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        for (int i=0; i<NUM_TEXTURES; i++) {
            camera->attach(osg::Camera::BufferComponent(osg::Camera::COLOR_BUFFER0+i), textureRect[i]);
        }
        camera->attach(osg::Camera::BufferComponent(osg::Camera::DEPTH_BUFFER), GL_DEPTH_COMPONENT);
        addChild(camera);
        _stateset = stateset;
    }

    {
        osg::ref_ptr<osg::Geode> geode = new osg::Geode;

        osg::ref_ptr<osg::Vec3Array> quad_coords = new osg::Vec3Array; // vertex coords
        // counter-clockwise
        quad_coords->push_back(osg::Vec3d(0, 0, -1));
        quad_coords->push_back(osg::Vec3d(1, 0, -1));
        quad_coords->push_back(osg::Vec3d(1, 1, -1));
        quad_coords->push_back(osg::Vec3d(0, 1, -1));

        osg::ref_ptr<osg::Vec2Array> quad_tcoords = new osg::Vec2Array; // texture coords
        quad_tcoords->push_back(osg::Vec2(0, 0));
        quad_tcoords->push_back(osg::Vec2(_texWidth, 0));
        quad_tcoords->push_back(osg::Vec2(_texWidth, _texHeight));
        quad_tcoords->push_back(osg::Vec2(0, _texHeight));

        osg::ref_ptr<osg::Geometry> quad_geom = new osg::Geometry;
        osg::ref_ptr<osg::DrawArrays> quad_da = new osg::DrawArrays(osg::PrimitiveSet::QUADS,0,4);
        quad_geom->setVertexArray(quad_coords.get());
        quad_geom->setTexCoordArray(0, quad_tcoords.get());
        quad_geom->addPrimitiveSet(quad_da.get());

        osg::StateSet* stateset = geode->getOrCreateStateSet();
        geode->addDrawable(quad_geom.get());
        stateset->setMode(GL_LIGHTING,osg::StateAttribute::OFF);
        stateset->setDataVariance(osg::Object::DYNAMIC);

        osg::Program* program = new osg::Program;
        osg::Shader* vertShader = new osg::Shader(osg::Shader::VERTEX);
        osg::Shader* fragShader = new osg::Shader(osg::Shader::FRAGMENT);
    
        vertShader->setShaderSource(std::string(deferred_final_pass_vertex_splat_source));
        fragShader->setShaderSource(std::string(deferred_final_pass_fragment_splat_source));

        program->addShader(vertShader);
        program->addShader(fragShader);
        stateset->setAttributeAndModes(program);
        stateset->setMode(GL_DEPTH_TEST, false);
        stateset->setMode(GL_BLEND, false);
        stateset->addUniform(colorEnabled);
        stateset->getOrCreateUniform("textureID0", osg::Uniform::INT_SAMPLER_2D_RECT)->set(0);
        stateset->getOrCreateUniform("textureID1", osg::Uniform::INT_SAMPLER_2D_RECT)->set(1);

        for (int i=0;i<NUM_TEXTURES;i++) {
            stateset->setTextureAttributeAndModes(i, textureRect[i], osg::StateAttribute::ON);
        }

        stateset->addUniform(colorEnabled);
        stateset->getOrCreateUniform("lightDirection", osg::Uniform::FLOAT_VEC3);
        stateset->getOrCreateUniform("lightAmbient", osg::Uniform::FLOAT_VEC4)->set(osg::Vec4(0.2,0.2,0.2, 1.0));
        stateset->getOrCreateUniform("lightDiffuse", osg::Uniform::FLOAT_VEC4)->set(osg::Vec4(0.8,0.8,0.8, 1.0));
        stateset->getOrCreateUniform("lightSpecular", osg::Uniform::FLOAT_VEC4);
        stateset->getOrCreateUniform("lightHalfVector", osg::Uniform::FLOAT_VEC3);
        stateset->getOrCreateUniform("materialAmbient", osg::Uniform::FLOAT_VEC4)->set(_material->getAmbient(osg::Material::FRONT));
        stateset->getOrCreateUniform("materialDiffuse", osg::Uniform::FLOAT_VEC4)->set(_material->getDiffuse(osg::Material::FRONT));
        stateset->getOrCreateUniform("materialSpecular", osg::Uniform::FLOAT_VEC4)->set(_material->getSpecular(osg::Material::FRONT));
        stateset->getOrCreateUniform("materialShininess", osg::Uniform::FLOAT)->set(_material->getShininess(osg::Material::FRONT));


        osg::Camera* camera = new osg::Camera;
        camera->setClearMask(0);
        camera->setProjectionMatrix(osg::Matrix::ortho2D(0,1,0,1));
        camera->setReferenceFrame(osg::Transform::ABSOLUTE_RF);
        camera->setViewMatrix(osg::Matrix::identity());
        camera->setRenderOrder(osg::Camera::POST_RENDER);
        camera->addChild(geode);
        addChild(camera);
    }
}
void GeometryGraphDeferredShading::update(const ViewerInfo& vi)
{
    GeometryGraph::update(vi);

    osg::Matrix modelView = vi._modelview;
    osg::Vec3 light = osg::Vec3(0,1,-1)* vi._modelview;
    light.normalize();
    osg::Vec3 halfVector = osg::Vec3(0,0,0) * vi._modelview;
#if 0
    osg::StateSet* stateset = _stateset;
    stateset->getOrCreateUniform("lightDirection", osg::Uniform::FLOAT_VEC3)->set(light);
    stateset->getOrCreateUniform("lightAmbient", osg::Uniform::FLOAT_VEC4)->set(osg::Vec4(0.2, 0.2, 0.2, 1));
    stateset->getOrCreateUniform("lightDiffuse", osg::Uniform::FLOAT_VEC4)->set(osg::Vec4(0.8, 0.8, 0.8, 1));
    stateset->getOrCreateUniform("lightSpecular", osg::Uniform::FLOAT_VEC4)->set(osg::Vec4(0.2, 0.2, 0.2, 1));
    stateset->getOrCreateUniform("lightHalfVector", osg::Uniform::FLOAT_VEC3)->set(halfVector);
    stateset->getOrCreateUniform("colorEnabled", osg::Uniform::BOOL)->set(_colors->size() != 0);
#endif

}

void GeometryGraphAdvanced::createSceneGraph()
{
    osg::Geode* geode1 = new osg::Geode;
    addChild(geode1);
    geode1->addDrawable(_geometry.get());
    geode1->getOrCreateStateSet()->setAttributeAndModes(new osg::BlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA));

    {
    osg::Program* program = new osg::Program;
    osg::Shader* vertShader = new osg::Shader(osg::Shader::VERTEX);
    osg::Shader* fragShader = new osg::Shader(osg::Shader::FRAGMENT);
    osg::Shader* geomShader = new osg::Shader(osg::Shader::GEOMETRY);
    program->addBindAttribLocation("splatAttribute", _attributeBinding);
    geomShader->setShaderSource(std::string(pass2_geometry_splat_source));
    vertShader->setShaderSource(std::string(pass2_vertex_splat_source));
    fragShader->setShaderSource(std::string(pass2_fragment_splat_source));

    program->addShader(geomShader);
    program->addShader(vertShader);
    program->addShader(fragShader);
    program->setParameter( GL_GEOMETRY_VERTICES_OUT_EXT, 4 );
    program->setParameter( GL_GEOMETRY_INPUT_TYPE_EXT, GL_POINTS );
    program->setParameter( GL_GEOMETRY_OUTPUT_TYPE_EXT, GL_TRIANGLE_STRIP );
    
    geode1->getOrCreateStateSet()->setAttributeAndModes(program);
    geode1->setDataVariance(osg::Object::DYNAMIC);
    }
    osg::StateSet* stateset = geode1->getOrCreateStateSet();
    stateset->setDataVariance(osg::Object::DYNAMIC);
    _stateset = stateset;
    _geometry->getOrCreateStateSet()->setName("AdvancedRendering");

    stateset->setMode(GL_VERTEX_PROGRAM_POINT_SIZE, true);
    stateset->setAttributeAndModes(new osg::AlphaFunc(osg::AlphaFunc::GREATER, 0.3));
    stateset->setMode(GL_LIGHTING, true);

    stateset->getOrCreateUniform("lightingEnabled", osg::Uniform::BOOL)->set(true);
    stateset->getOrCreateUniform("light0Enabled", osg::Uniform::BOOL)->set(true);
    stateset->getOrCreateUniform("colorEnabled", osg::Uniform::BOOL)->set(false);
    stateset->getOrCreateUniform("uvec", osg::Uniform::FLOAT_VEC4);
    stateset->getOrCreateUniform("vvec", osg::Uniform::FLOAT_VEC4);

    osg::Image* image = osgDB::readImageFile("texture.png");
    osg::Texture2D* texture = new osg::Texture2D(image);
    texture->setFilter(osg::Texture::MIN_FILTER, osg::Texture::LINEAR);
    texture->setFilter(osg::Texture::MAG_FILTER, osg::Texture::LINEAR);
    stateset->setTextureAttributeAndModes(0, texture);
    stateset->addUniform( new osg::Uniform("Texture", 0));
    stateset->getOrCreateUniform("MVP", osg::Uniform::FLOAT_MAT4);
}

void GeometryGraphAdvanced::update(const ViewerInfo& vi)
{
    GeometryGraph::update(vi);

    osg::Vec3 lr, ur;
    ur[0] =  vi._modelview.ptr()[0] + vi._modelview.ptr()[1];  ur[1] =  vi._modelview.ptr()[4] + vi._modelview.ptr()[5];  ur[2] =  vi._modelview.ptr()[8] + vi._modelview.ptr()[9];
    lr[0] =  vi._modelview.ptr()[0] - vi._modelview.ptr()[1];  lr[1] =  vi._modelview.ptr()[4] - vi._modelview.ptr()[5];  lr[2] =  vi._modelview.ptr()[8] - vi._modelview.ptr()[9];

    _stateset->getOrCreateUniform("uvec", osg::Uniform::FLOAT_VEC3)->set(ur);
    _stateset->getOrCreateUniform("vvec", osg::Uniform::FLOAT_VEC3)->set(lr);
    _stateset->getOrCreateUniform("MVP", osg::Uniform::FLOAT_MAT4)->set(vi._modelview * vi._projection);

    // this is not the best way to do
    // best should to not touch data but instead use a drawElements and sort only the indices
    _sortBuffer.clear();
    for (unsigned int i = 0 ; i < _vertexes->size(); i++) {
        osg::Vec3 v = (*_vertexes)[i];
        _sortBuffer.insert(SplatVertex(v, &(*_normals)[i][0], &(*_colors)[i][0], (*_attributes)[i][3], vi._zproj.distance(v)));
    }
    
    for (SortContainer::iterator it = _sortBuffer.begin() ; it != _sortBuffer.end(); ++it) {
        const SplatVertex& splat = *it;
        _vertexes->push_back(splat._vertex);
        const float* norm = splat._normal;
        _normals->push_back(osg::Vec3(norm[0], norm[1], norm[2]));
        _attributes->push_back(osg::Vec4(0, 0, 0, splat._radius ));
        if (splat._color)
            _colors->push_back(osg::Vec4(splat._color[0], splat._color[1], splat._color[2], 1.0));
    }

    _drawarray->setCount(_vertexes->size());
    _vertexes->dirty();
    _normals->dirty();
    _colors->dirty();
    _attributes->dirty();
}


static void Error(const std::string& message, const std::string& file)
{
    osg::notify(osg::WARN) << "QSplat error " << message << " with file " << file << std::endl;
}


// Dig out the required information from the header of an individual fragment.
// For V11 files, just returns center of the highest-level sphere.
static inline void parse_header(const unsigned char *here,
                                const unsigned char **drawstart,
                                int *numchildren,
                                float *rootcx, float *rootcy,
                                float *rootcz, float *rootr, bool& color, int* nodesize)
{
    unsigned int options = * (int *)(here+16);
    FIX_LONG(options);
    color = options & 1;
    if (nodesize)
        (*nodesize) = (color ? 6 : 4);

    *rootcx = * (float *)(here+20);
    *rootcx = FIX_FLOAT(*rootcx);
    *rootcy = * (float *)(here+24);
    *rootcy = FIX_FLOAT(*rootcy);
    *rootcz = * (float *)(here+28);  
    *rootcz = FIX_FLOAT(*rootcz);
    *rootr  = * (float *)(here+32);  
    *rootr = FIX_FLOAT(*rootr);
    *numchildren = * (int *)(here+36);  FIX_LONG(*numchildren);
    *drawstart = here+40;
}


// Try to open a file.  Returns a new QSplat_Model, or NULL if can't open it.
QSplatModel* QSplatModel::Open(const char *modelfilename)
{
    static bool initialized = false;
    if (!initialized) {
        Init();
        initialized = true;
    }

    osg::notify(osg::INFO) << "Opening file " << modelfilename << "..." << std::endl;
#ifdef WIN32
    OFSTRUCT fdOFSTRUCT;
    HFILE hfd = OpenFile(modelfilename, &fdOFSTRUCT, OF_READ);
    if (hfd == HFILE_ERROR) {
        Error("Couldn't open ", modelfilename);
        return NULL;
    }
    off_t len = FileLen(hfd);
    if (len <= 0) {
        _lclose(hfd);
        Error("Couldn't open ", modelfilename);
        return NULL;
    } else if (len < 40) {
        _lclose(hfd);
        Error(modelfilename, " is not a QSplat file");
        return NULL;
    }
    _lclose(hfd);

    HANDLE fd = CreateFile(modelfilename,
                           GENERIC_READ, 0, NULL, OPEN_EXISTING,
                           FILE_ATTRIBUTE_NORMAL | FILE_FLAG_RANDOM_ACCESS, NULL);
#else
    int fd = open(modelfilename, O_RDONLY);
    if (fd == -1) {
        perror("open");
        return NULL;
    }
    off_t len = FileLen(fd);
    if (len < 0) {
        close(fd);
        Error("Couldn't open ", modelfilename);
        return NULL;
    } else if (len < 40) {
        close(fd);
        Error(modelfilename, " is not a QSplat file");
        return NULL;
    }
#endif

    unsigned char *mem_start = NULL, *map_start = NULL;
    if (!MapFile(fd, len, &mem_start, &map_start)) {
        CloseHandle(fd);
        return NULL;
    }


    // Build the new object
    QSplatModel *q = new QSplatModel(std::string(modelfilename),
                                     mem_start, map_start,
                                     fd, len);

    if (!q->BuildFragmentList(modelfilename)) {
        delete q;
        return NULL;
    }

    q->initModel();

    return q;
}
    


// Destructor - unmap the file
QSplatModel::~QSplatModel()
{
    _fragments.clear();
#ifdef WIN32
    UnmapViewOfFile(_map_start);
#else
    munmap((char *)_map_start, _len);
#endif
    if (_mem_start)
        delete [] _mem_start;
    CloseHandle(_fd);
}


// Initialize global data structures for QSplat
void QSplatModel::Init()
{
    QSplat_ColorQuant::Init();
    QSplat_NormQuant::Init();
    QSplat_SphereQuant::Init();
}




void QSplatModel::initModel()
{
    //check in header if we have color
    _haveColor = true;
    for (unsigned int i = 0; i < _fragments.size(); ++i) {
        bool color;
        float cx, cy, cz, r;
        int numchildren;
        const unsigned char *drawstart;
        parse_header(_fragments[i], &drawstart, &numchildren,
                     &cx, &cy, &cz, &r, color, &_nodeSize);
        if (!color) {
            _haveColor = false;
            break;
        }
    }
    setQuality(DEFERRED);
}


// Determine length of an open file
off_t QSplatModel::FileLen(HFILE fd)
{
#ifdef WIN32
    BY_HANDLE_FILE_INFORMATION fdInfo;
    if (!GetFileInformationByHandle((HANDLE)fd, &fdInfo))
        return -1;
    return fdInfo.nFileSizeLow;
#else
    struct stat statbuf;
    if (fstat(fd, &statbuf) == -1)
        return -1;
    return statbuf.st_size;
#endif
}


// Memory map a file, returning pointers to the start of the allocated
// memory region and the start of the map
// This returns true if we were able to do the mmap
bool QSplatModel::MapFile(HANDLE fd, off_t len,
                          unsigned char **mem_start,
                          unsigned char **map_start)
{
#ifdef WIN32
    static HANDLE fdMapping;
    char mapName[16];
    int mapNum = 0;
    UINT error;
    do {
        if (fdMapping)
            CloseHandle(fdMapping);
        sprintf(mapName, "QSplat%d", mapNum++);
        fdMapping = CreateFileMapping(fd, NULL, PAGE_READONLY,
                                      0, len, mapName);
    } while ((error = GetLastError()) == ERROR_ALREADY_EXISTS);

    if (!fdMapping || error != NO_ERROR) {
        char fdError[32];
        sprintf(fdError, "Mapping Error: %d", error);
        MessageBox(NULL, fdError, NULL, MB_OK);
        return false;
    }

    *map_start = (unsigned char *) MapViewOfFile(fdMapping, FILE_MAP_READ,
                                                 0, 0, len);
    if (*map_start)
        return true;

    char errorstr[32];
    sprintf(errorstr, "Can't Allocate Memory! (%d)\n", GetLastError());
    MessageBox(NULL, errorstr, NULL, MB_OK | MB_ICONHAND);
    return false;

#else

    // First, we just try the mmap
    *map_start = (unsigned char *)
        mmap(0, len, PROT_READ, MAP_SHARED, fd, 0);

    // If that worked, we're done
    if (*map_start != MAP_FAILED)
        return true;


    // OK, that didn't work.  However, on certain operating systems
    // (cough, cough, IRIX, cough, cough), it is sometimes the case
    // that for some bizarre reason the mmap doesn't work right off
    // even though we officially do have a large-enough virtual address
    // space to map in everything.  To try to work around this, we'll
    // try to allocate a large-enough chunk of memory, and mmap there

    // Figure out how much memory to allocate
#ifdef sgi
    int align = sysconf(_SC_MMAP_FIXED_ALIGNMENT);
#else
    int align = sysconf(_SC_PAGE_SIZE);
#endif

    // To work around yet another bug, we temporarily allocate some memory
    // here, else sometimes we don't get to use the heap (!) after the mmap
    unsigned char *ugly_hack = new unsigned char[16*1024*1024];

    // Allocate memory and align the pointer correctly
    *mem_start = new unsigned char[len + align];
    long tmp = long(*mem_start);
    tmp = align * long(ceil((double) tmp / align));
    *map_start = (unsigned char *)tmp;


    // Do the mmap
    *map_start = (unsigned char *)
        mmap( (char *)(*map_start), len,
              PROT_READ, MAP_SHARED | MAP_FIXED,
              fd, 0 );
    delete [] ugly_hack;
    if (*map_start == MAP_FAILED) {
        // Give up
        perror("mmap");
        delete [] (*mem_start);
        *mem_start = *map_start = 0;
        return false;
    }

    return true;
#endif
}


// A single file can have multiple _fragments - the file just looks like
// the files for the individual _fragments catted together
bool QSplatModel::BuildFragmentList(const char *filename)
{
    float xmin=3.3e33f, xmax=-3.3e33f;
    float ymin=3.3e33f, ymax=-3.3e33f;
    float zmin=3.3e33f, zmax=-3.3e33f;

    _comments = "File "; _comments += filename; _comments += "\n";

    const unsigned char *here = _map_start;
    while (here < _map_start+_len) {

        if (_map_start+_len-here < 40) {
            Error("Couldn't read header of ", filename);
            return false;
        }

        if (strncmp((const char *)here, QSPLAT_MAGIC, 6) != 0) {
            Error(filename, " is not a QSplat file");
            return false;
        }
        char buf[3];
        sprintf(buf, "%02d", QSPLAT_FILE_VERSION);
        if (here[6] != buf[0] || here[7] != buf[1]) {
            Error(filename, " was made for a different version of QSplat");
            return false;
        }

        int fraglen = * (int *)(here+8);
        FIX_LONG(fraglen);
        if (here+fraglen > _map_start+_len) {
            Error(filename, " is truncated");
            return false;
        }

        if ((*(unsigned char *)(here+19)) & 2) {
            _comments.append((const char *)(here+20), fraglen-20);
            here += fraglen;
            continue;
        }

        int points = * (int *)(here+12);
        osg::notify(osg::NOTICE) << " " << points << std::endl;
        FIX_LONG(points);

        _leaf_points += points;

        float x = * (float *)(here+20);
        x = FIX_FLOAT(x);
        float y = * (float *)(here+24); 
        y = FIX_FLOAT(y);

        float z = * (float *)(here+28); 
        z = FIX_FLOAT(z);

        float r = * (float *)(here+32); 
        r = FIX_FLOAT(r);

//        osg::notify(osg::NOTICE) << " " << x << " " << y << " " << z << " " << r << std::endl;

        xmin = min(xmin, x-r);  xmax = max(xmax, x+r);
        ymin = min(ymin, y-r);  ymax = max(ymax, y+r);
        zmin = min(zmin, z-r);  zmax = max(zmax, z+r);

        _fragments.push_back(here);
        here += fraglen;

    }

    _center[0] = 0.5f * (xmin + xmax);
    _center[1] = 0.5f * (ymin + ymax);
    _center[2] = 0.5f * (zmin + zmax);
    _radius = 0.5f*sqrtf(sqr(xmax-xmin) + sqr(ymax-ymin) + sqr(zmax-zmin));

    char buf[255];
    sprintf(buf, "%d leaf points\n", _leaf_points);
    _comments += buf;
#ifndef WIN32
    osg::notify(osg::WARN) << "QSplat " << buf;
#endif
    return true;
}

static const float sqrt5 = 2.236068f;


// We've gotten to the lowest level of the hierarchy, and we're just going to
// draw a bunch of leaf nodes without testing their sizes
void QSplatModel::draw_hierarchy_leaves(const unsigned char *here, int numnodes,
                                        float cx, float cy, float cz, float r,
                                        float approx_splatsize_scale)
{

    for (int i=0; i < numnodes; i++, here += _nodeSize) {
        float mycx, mycy, mycz, myr;
        QSplat_SphereQuant::lookup(here,
                                   cx, cy, cz, r,
                                   mycx, mycy, mycz, myr);
        float splatsize = approx_splatsize_scale ?
            myr * approx_splatsize_scale :
            _viewInfo._minsize;
        
        float z = _viewInfo._zproj.distance(osg::Vec3(mycx, mycy, mycz));
        drawpoint(z, mycx, mycy, mycz,
                  myr, splatsize,
                  QSplat_NormQuant::lookup(here+2),
                  _haveColor?QSplat_ColorQuant::lookup(here+4):NULL);
    }
}



// The fast version of the draw routine.  We switch to this when size gets
// down to a few pixels.
// See draw_hierarchy() for comments...
void QSplatModel::draw_hierarchy_fast(const unsigned char *here, int numnodes,
                                      float cx, float cy, float cz, float r)
{
    int childoffset = UNALIGNED_DEREFERENCE_INT(here);
    FIX_LONG(childoffset);
    const unsigned char *there = here + childoffset;
    here += 4;

    int numchildren = 0;
    int grandchildren = 0;

    for (int i=0; i < numnodes; i++, here += _nodeSize, there += _nodeSize*numchildren + grandchildren) {

        numchildren = here[1] & 3;
        if (numchildren) {
            numchildren++;
            grandchildren = here[1] & 4;
        } else {
            grandchildren = 0;
        }

        float mycx, mycy, mycz, myr;
        QSplat_SphereQuant::lookup(here,
                                   cx, cy, cz, r,
                                   mycx, mycy, mycz, myr);

        float z = _viewInfo._zproj.distance(osg::Vec3( mycx, mycy, mycz));
        float splatsize_scale = 2.0f * _viewInfo._pixelsPerRadian / z;
        float splatsize = myr * splatsize_scale;

        if (_overallSplatSize) {
            splatsize = myr/_radius * 5000.0f;
        }

        if (!numchildren || ((splatsize <= _viewInfo._minsize))) {
            drawpoint(z, mycx, mycy, mycz,
                      myr, splatsize,
                      QSplat_NormQuant::lookup(here+2),
                      _haveColor?QSplat_ColorQuant::lookup(here+4):NULL);
        } else if (!grandchildren) {
            draw_hierarchy_leaves(there, numchildren,
                                  mycx, mycy, mycz, myr,
                                  splatsize_scale);
        } else if (splatsize <= FAST_CUTOFF_FACTOR*_viewInfo._minsize) {
            draw_hierarchy_leaves(there+4, numchildren,
                                  mycx, mycy, mycz, myr,
                                  0.0f);
        } else {
            draw_hierarchy_fast(there, numchildren,
                                mycx, mycy, mycz, myr);
        }
    }
}



// The main drawing routine
void QSplatModel::draw_hierarchy(const unsigned char *here, int numnodes,
                                 float cx, float cy, float cz, float r,
                                 bool backfacecull, bool frustumcull)
{
#if 0
    // Check for events, but not too often
    if (! (_checkCounter++ & 0xff)) {
        osg::Timer_t now = osg::Timer::instance()->tick();
        double s = osg::Timer::instance()->delta_s(_startRendering, now);
        //osg::notify(osg::INFO)  << "draw elapsed " << s << std::endl;
        if ( s > (1.0 * 1.0/_desiredRate) ) {
            osg::notify(osg::INFO) << "draw took too much time bail" << std::endl;
//            _bail = true;
//            return;
        }
    }
#endif

    // Where are the children of these nodes stored?
    int childoffset = UNALIGNED_DEREFERENCE_INT(here);
    FIX_LONG(childoffset);
    const unsigned char *there = here + childoffset;
    here += 4;


    int numchildren = 0;
    int grandchildren = 0;

    // For each node in this group of siblings
    for (int i=0; i < numnodes; i++, here += _nodeSize, there += _nodeSize*numchildren + grandchildren) {
        // Find number of children
        numchildren = here[1] & 3;
        if (numchildren) {
            // Code: 0 really means no children, but since 1 child
            // never happens, 1 really means 2 children and so on

            numchildren++;

            // The "grandchildren" bit is set if this node has any
            // grandchildren, and hence has the extra pointer
            // Note that because this uses bit 2, we automagically
            // get the right increment to the child offset just by
            // extracting this bit.

            grandchildren = here[1] & 4;

        } else {
            grandchildren = 0;
        }


        // Determine our position and radius
        float mycx, mycy, mycz, myr;
        QSplat_SphereQuant::lookup(here,
                                   cx, cy, cz, r,
                                   mycx, mycy, mycz, myr);

        // Determine perpendicular distance to screen plane
        float z = _viewInfo._zproj.distance(osg::Vec3(mycx, mycy, mycz));


        // Frustum culling
        bool frustumcull_children = frustumcull;
        if (frustumcull) {
            if ((z <= -myr) ||
                (mycx* _viewInfo._frustum[0][0] + mycy* _viewInfo._frustum[0][1] +
                 mycz* _viewInfo._frustum[0][2] + _viewInfo._frustum[0][3] <= -myr) ||
                (mycx* _viewInfo._frustum[1][0] + mycy* _viewInfo._frustum[1][1] +
                 mycz*_viewInfo._frustum[1][2] + _viewInfo._frustum[1][3] <= -myr) ||
                (mycx*_viewInfo._frustum[2][0] + mycy*_viewInfo._frustum[2][1] +
                 mycz*_viewInfo._frustum[2][2] + _viewInfo._frustum[2][3] <= -myr) ||
                (mycx*_viewInfo._frustum[3][0] + mycy*_viewInfo._frustum[3][1] +
                 mycz*_viewInfo._frustum[3][2] + _viewInfo._frustum[3][3] <= -myr))
                continue;
            if ((z > myr) &&
                (mycx*_viewInfo._frustum[0][0] + mycy*_viewInfo._frustum[0][1] +
                 mycz*_viewInfo._frustum[0][2] + _viewInfo._frustum[0][3] >= myr) &&
                (mycx*_viewInfo._frustum[1][0] + mycy*_viewInfo._frustum[1][1] +
                 mycz*_viewInfo._frustum[1][2] + _viewInfo._frustum[1][3] >= myr) &&
                (mycx*_viewInfo._frustum[2][0] + mycy*_viewInfo._frustum[2][1] +
                 mycz*_viewInfo._frustum[2][2] + _viewInfo._frustum[2][3] >= myr) &&
                (mycx*_viewInfo._frustum[3][0] + mycy*_viewInfo._frustum[3][1] +
                 mycz*_viewInfo._frustum[3][2] + _viewInfo._frustum[3][3] >= myr))
                frustumcull_children = false;
        }


        // Backface culling
        bool backfacecull_children = backfacecull;
        float camdotnorm = 0.0f; // If we're not doing backface
        // culling, this gets left set to a
        // safe value for the sake of code
        // later on.
        if (backfacecull && ((here[3] & 3) != 3)) {
            const float *norm = QSplat_NormQuant::lookup(here+2);
            float camx = _viewInfo._cameraPosition[0] - mycx;
            float camy = _viewInfo._cameraPosition[1] - mycy;
            float camz = _viewInfo._cameraPosition[2] - mycz;
            camdotnorm = camx * norm[0] +
                camy * norm[1] +
                camz * norm[2];
            if (camdotnorm < -myr) {
                float camdist2 = sqr(camx) + sqr(camy) + sqr(camz);
                float cone = QSplat_NormQuant::lookup_cone(here+2);
                if (sqr(camdotnorm + myr) > camdist2 * sqr(cone)) {
                    continue;
                }
            } else if (camdotnorm > myr) {
                float camdist2 = sqr(camx) + sqr(camy) + sqr(camz);
                float cone = QSplat_NormQuant::lookup_cone(here+2);
                if (sqr(camdotnorm - myr) > camdist2 * sqr(cone)) {
                    backfacecull_children = false;
                }
            }
        }

        // Yes, we actually have to (gasp) do a divide.
        float splatsize_scale = 2.0f * _viewInfo._pixelsPerRadian / z;
        float splatsize = myr * splatsize_scale;
        if (_overallSplatSize) {
            splatsize = myr/_radius * 5000.0;
        }

        // Check whether we recurse...
        if (!numchildren || ( ((frustumcull && z > 0.0f) || !frustumcull) && (splatsize <= _viewInfo._minsize))) {
            // No - draw now
            if ( (!backfacecull && !frustumcull) || (( frustumcull && z > 0.0f) && (backfacecull && camdotnorm >= 0.0f))) {
                drawpoint(z, mycx, mycy, mycz,
                          myr, splatsize,
                          QSplat_NormQuant::lookup(here+2),
                          _haveColor?QSplat_ColorQuant::lookup(here+4):NULL);
            }
        } else if (!grandchildren) {
            // We recurse, but children are all leaf nodes
            draw_hierarchy_leaves(there, numchildren,
                                  mycx, mycy, mycz, myr,
                                  splatsize_scale);
        } else if ((!frustumcull_children && !backfacecull_children) ||
                   ((splatsize <= _viewInfo._minsize*FAST_CUTOFF_FACTOR) && (z > 0.0f))) {
            // We recurse, but switch to fast mode
            if (splatsize <= _viewInfo._minsize*FAST_CUTOFF_FACTOR) {
                // Hack: if we're this small, the next round
                // of recursion is going to be awfully close to
                // minsize, so we use the _leaves function
                // to just draw the children...
                draw_hierarchy_leaves(there+4, numchildren,
                                      mycx, mycy, mycz, myr,
                                      0.0f);
            } else {
                draw_hierarchy_fast(there, numchildren,
                                    mycx, mycy, mycz, myr);
            }
        } else {
            // Basic slow-mode recursion
            draw_hierarchy(there, numchildren,
                           mycx, mycy, mycz, myr,
                           backfacecull_children,
                           frustumcull_children);
            if (_bail)
                return;
        }
    }
}



GeometryGraph* QSplatModel::createNewGeometry()
{
    GeometryGraph* gg = 0;
    switch(_rendering) {
    case BASIC:
        gg = new GeometryGraphBasic;
        break;
    case ADVANCED:
        gg = new GeometryGraphAdvanced;
        break;
    case DEFERRED:
        gg = new GeometryGraphDeferredShading;
        break;
    }
    if (gg) {
        gg->createSceneGraph();
        gg->_geometry->setInitialBound(osg::BoundingBox(_center - osg::Vec3(_radius, _radius, _radius), _center + osg::Vec3(_radius, _radius, _radius)));
    }
    return gg;
}

void QSplatModel::setGeometry(GeometryGraph* geom)
{
    OpenThreads::ScopedLock<OpenThreads::Mutex> lock(_mutex);
    _geometryGraph = geom;
}


void QSplatModel::setupBuild(const ViewerInfo& vi, float minsize, bool frustrumCulling, bool backfaceCulling)
{
    OpenThreads::ScopedLock<OpenThreads::Mutex> lock(_mutex);
    _viewInfo = vi;
    _viewInfo._minsize = minsize;
    _frustumCulling = frustrumCulling;
    _backfaceCulling = backfaceCulling;
}

void QSplatModel::operator()(osg::Object*) 
{
    build(_frustumCulling, _backfaceCulling);
}

bool QSplatModel::build(bool frustrumCulling, bool backfaceCulling)
{
    OpenThreads::ScopedLock<OpenThreads::Mutex> lock(_mutex);
    
    // Prepare for drawing
    _geometryGraph->reset();
    _bail = false;
    _geometryGraph->_minsize = _viewInfo._minsize;
    // Draw each fragment
    for (unsigned int f = 0; f < _fragments.size(); f++) {
        float cx, cy, cz, r;
        int numchildren;
        bool color;
        const unsigned char *drawstart;
        parse_header(_fragments[f], &drawstart, &numchildren,
                     &cx, &cy, &cz, &r, color, &_nodeSize);
        draw_hierarchy(drawstart, numchildren,
                       cx, cy, cz, r,
                       backfaceCulling, frustrumCulling);
        if (_bail)
            break;
    }

    _geometryGraph->dirty();

    osg::notify(osg::NOTICE) << "Model has " << _geometryGraph->_vertexes->size() << " vertexes " << _geometryGraph->getBound().center() << " " << _geometryGraph->getBound().radius() <<  " for size " <<  _geometryGraph->_minsize << std::endl;

    return _bail;
}






class PointCloudQSplat : public osgJuniper::PointCloud
{
public:

    PointCloudQSplat() : osgJuniper::PointCloud() 
    {
        setDataVariance(osg::Object::DYNAMIC);
        _previousTime = 0;
        _desiredRate = 30;
        _askRun = false;
        _isOnScreen = true;
        _minsize = MINSIZE_MAX;
        setCullingActive(false);
        _maxVertexes = 4000000;
        _checkRate = 0.001;
        _isMoving = false;
    }

    void setBuilder(QSplatModel* model) { _builder = model; }

    virtual void setQuality(float quality) {
        _quality = osg::clampBetween(quality, 0.0f, 1.0f);
        osg::notify(osg::NOTICE) << "Quality=" << _quality << std::endl;
        float q = 1.0f - _quality;
        float minsize = q * (50 - MINSIZE_MIN) + MINSIZE_MIN;
        osg::notify(osg::NOTICE) << "setting minSize " << minsize << std::endl;
        _builder->setupBuild(_viewerInfo, minsize, false, false);
        _operationThread->add(_builder.get());
        _askRun = true;
    }

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
            osg::Matrixd prevModelView = _viewerInfo._modelview;
            _viewerInfo.compute(cs);
            _isMoving = (prevModelView != _viewerInfo._modelview);
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
                    if (!_askRun && canRefine() && !_isMoving)
                    {
                        float minsize = refine();
                        _builder->setupBuild(_viewerInfo, minsize, false, false);
                        _operationThread->add(_builder.get());
                        _askRun = true;
                        osg::notify(osg::NOTICE) << "fps " << fps << " refine " << minsize << std::endl;
                    }
                    else if (!_askRun && _isMoving && _minsize !=  2.0f)
                    {
                        float minsize = 2.0f;
                        _builder->setupBuild(_viewerInfo, minsize, false, false);
                        _operationThread->add(_builder.get());
                        _askRun = true;
                        osg::notify(osg::NOTICE) << "moving " << minsize << std::endl;
                    }
                    
                    // 30 % of fps variation from the target will make the decision
                    /*if (fps > (_desiredRate + 0.3 * _desiredRate)  && !_askRun && canRefine()) {
                        float minsize = refine();
                        _builder->setupBuild(_viewerInfo, minsize, false, false);
                        _operationThread->add(_builder.get());
                        _askRun = true;
                        osg::notify(osg::NOTICE) << "fps " << fps << " refine " << minsize << std::endl;
                    } else if (fps < (_desiredRate - 0.3*_desiredRate ) && !_askRun ) {
                        float minsize = define();
                        _builder->setupBuild(_viewerInfo, minsize, false, false);
                        _operationThread->add(_builder.get());
                        _askRun = true;
                        osg::notify(osg::NOTICE) << "fps " << fps << " define " << minsize << std::endl;
                    } else if (!_askRun && !_builder->getSelectByDistFromCamera()) {
                        osg::notify(osg::NOTICE) << "fps " << fps << " rebuild " << _minsize << std::endl;
                        _builder->setupBuild(_viewerInfo, _minsize, false, false);
                        _operationThread->add(_builder.get());
                        _askRun = true;
                    }*/
                }

                _numFrames = 0;
            }

            // operation finished update get the geometry
            if (_askRun && !_operationThread->getCurrentOperation() && _operationThread->getOperationQueue()->empty()) {
                osg::ref_ptr<GeometryGraph> newmesh = _builder->getGeometry();
                // too much discard it
                if (newmesh->_vertexes->size() > _maxVertexes) {
                    _askRun = false;
                    osg::notify(osg::NOTICE) << _builder->getFilename() << " too many vertexes (" << newmesh->_vertexes->size() << " ) discard it for size, " << newmesh->_minsize  << std::endl;
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
                    osg::notify(osg::NOTICE) << _builder->getFilename() << " changed mesh " << _geometryGraph.get() << std::endl;
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
        minsize = min(max(minsize, MINSIZE_MIN), MINSIZE_MAX);
        return minsize;
    }

    float define()
    {
        float minsize = _minsize;
        minsize *= 1.0/MINSIZE_REFINE_MULTIPLIER;
        minsize = ceil(minsize);
        minsize = max(min(minsize, MINSIZE_MAX), MINSIZE_MIN);
        return minsize;
    }
    bool canRefine()
    {
        if (_geometryGraph.valid()) {
            float size = 1.0 * _geometryGraph->_vertexes->size();
            size = size * 1.0/(MINSIZE_REFINE_MULTIPLIER*MINSIZE_REFINE_MULTIPLIER);
            //osg::notify(osg::NOTICE) << "estimate size " << size << std::endl;
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
    osg::ref_ptr<QSplatModel> _builder;
    osg::Timer_t _previousTime;
    unsigned int _numFrames;
    unsigned int _desiredRate;
    float _minsize;
    bool _askRun;
    bool _isOnScreen;
    ViewerInfo _viewerInfo;
    int _maxVertexes;
    float _checkRate;
    bool _isMoving;
};



class QSplatReader : public osgDB::ReaderWriter
{
public:
    QSplatReader()
    {
        supportsExtension( "qs", className() );
    }

    virtual const char* className()
    {
        return "QSplat Reader";
    }

    virtual ReadResult readNode( const std::string& location, const osgDB::ReaderWriter::Options* options ) const
    {
        if ( !acceptsExtension( osgDB::getLowerCaseFileExtension( location ) ) )
            return ReadResult::FILE_NOT_HANDLED;

        QSplatModel* qm = QSplatModel::Open(location.c_str());
        if (!qm)
            return 0;

        PointCloudQSplat* node = new PointCloudQSplat;
        node->setBuilder(qm);

        if (options && options->getOptionString() == "quality1" ) {
            osg::notify(osg::NOTICE) << "Use quality path" << std::endl;
            qm->setQuality(QSplatModel::ADVANCED);
        } else if (options && options->getOptionString() == "quality2" ) {
            osg::notify(osg::NOTICE) << "Use quality path" << std::endl;
            qm->setQuality(QSplatModel::DEFERRED);
        } else {
            qm->setQuality(QSplatModel::BASIC);
        }

        return node;
    }
};


REGISTER_OSGPLUGIN(qs, QSplatReader)


