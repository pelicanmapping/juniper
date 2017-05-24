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
#include <osgDB/Registry>
#include <osgDB/FileNameUtils>
#include <osgDB/ReaderWriter>
#include <osgDB/ReadFile>
#include <osg/Point>
#include <osg/Shader>
#include <osg/Program>
#include <osg/CullStack>
#include <osg/io_utils>


/******************************************************************************/
class PointSplatScene : public osg::Group
{
public:
    PointSplatScene();
    virtual void traverse(osg::NodeVisitor& nv);
};
/******************************************************************************/


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


"float pixelSize(in vec3 v, in float radius, in vec4 pixelSizeVector)\n"
"{\n"
"  return abs(float(radius / (v * pixelSizeVector.xyz + pixelSizeVector.w)));\n"
"}\n"

"\n"
"\n"
"uniform bool lightingEnabled; \n"
"uniform bool light0Enabled; \n"
"uniform bool colorEnabled; \n"
"uniform vec4 pixelSizeVector;\n"
"attribute vec4 splatSize; \n"
"\n"
"void main (void)\n"
"{\n"
"  float radius = splatSize.w;\n"
//"  vec4 mod = gl_ModelViewMatrix * gl_Vertex;\n"
//"  gl_PointSize = abs(pixelSize(mod.xyz, radius, pixelSizeVector));\n"
"  gl_PointSize = abs(pixelSize(gl_Vertex.xyz, radius, pixelSizeVector));\n"
"  if (gl_PointSize < 1.0) gl_PointSize = 1.0;\n"
"  else if (gl_PointSize > 40.0) gl_PointSize = 40.0;\n"

"  vec4 color = vec4(1.0, 1.0, 1.0, 1.0);\n"
"  if (colorEnabled)\n"
"     color = gl_Color;\n"

/*
"  if ((gl_NormalMatrix * gl_Normal).z <= 0.0) {\n"
"     gl_Position = vec4(10000, 10000, 0, 1);"
"     return;\n"
"  }\n"
*/

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
"  gl_Position = ftransform();\n"
"}\n";

char basic_fragment_splat_source[] =
"\n"
"void main (void)\n"
"{\n"
"   gl_FragColor = gl_Color;\n"
"}\n"
"";


PointSplatScene::PointSplatScene()
{
    osg::StateSet* stateset = getOrCreateStateSet();
    stateset->setDataVariance(osg::Object::DYNAMIC);
    stateset->setAttributeAndModes(new osg::Point(1));
    stateset->setMode(GL_VERTEX_PROGRAM_POINT_SIZE, true);

    osg::Program* program = new osg::Program;
    osg::Shader* vertShader = new osg::Shader(osg::Shader::VERTEX);
    osg::Shader* fragShader = new osg::Shader(osg::Shader::FRAGMENT);
    
    vertShader->setShaderSource(std::string(basic_vertex_splat_source));
    fragShader->setShaderSource(std::string(basic_fragment_splat_source));

    program->addShader(vertShader);
    program->addShader(fragShader);
    stateset->setAttributeAndModes(program);
    stateset->getOrCreateUniform("lightingEnabled", osg::Uniform::BOOL)->set(true);
    stateset->getOrCreateUniform("light0Enabled", osg::Uniform::BOOL)->set(true);
    stateset->getOrCreateUniform("colorEnabled", osg::Uniform::BOOL)->set(true);    
    stateset->getOrCreateUniform("pixelSizeVector", osg::Uniform::FLOAT_VEC4)->setDataVariance(osg::Object::DYNAMIC);
    program->addBindAttribLocation("splatSize", 11);
}

void
PointSplatScene::traverse(osg::NodeVisitor &nv)
{
    if (nv.getVisitorType() == osg::NodeVisitor::CULL_VISITOR) {
        osg::CullStack* cs = dynamic_cast<osg::CullStack*>(&nv);
        /*osg::Matrix modelview = *(cs->getModelViewMatrix());
        osg::Matrix projection = *(cs->getProjectionMatrix());
        double width = cs->getViewport()->width();
        double height = cs->getViewport()->height();
        osg::Matrix projectionModelView = modelview * projection;

        // Compute some stuff
        float pixelsPerRadian = 0.5f * width * projection.ptr()[0]; // Assume glFrustum only
        getOrCreateStateSet()->getOrCreateUniform("pixelsPerRadian", osg::Uniform::FLOAT)->set(1.0f);        
        osg::notify(osg::NOTICE) << "pixelsPerRadian = " << pixelsPerRadian << std::endl;
        */
        osg::Vec4 pixelSizeVector = cs->getCurrentCullingSet().getPixelSizeVector();
        //osg::notify(osg::NOTICE) << "pixelSizeVector=" << pixelSizeVector << std::endl;
        getOrCreateStateSet()->getOrCreateUniform("pixelSizeVector", osg::Uniform::FLOAT_VEC4)->set(pixelSizeVector);        
    }


    osg::Group::traverse(nv);
}

class SplatRenderer : public osgDB::ReaderWriter
{
public:
    SplatRenderer()
    {
        supportsExtension( "juniper_splat_shader", className() );
    }

    //override
    const char* className()
    {
        return "Splat Shader Renderer for Juniper";
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

        /*osg::Group* root = new osg::Group;
        root->setStateSet(createStateSet());
        root->addChild(node.get());
        return root;*/
        PointSplatScene* splatScene = new PointSplatScene();
        splatScene->addChild( node.get());
        return splatScene;
    }

};

REGISTER_OSGPLUGIN(juniper_splat_shader, SplatRenderer)
