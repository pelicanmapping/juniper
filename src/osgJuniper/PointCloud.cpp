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

#include <osgJuniper/PointCloud>

#include <osg/Program>
#include <osg/Drawable>

using namespace osgJuniper;

PointCloud::PointCloud()
{
    _operationThread = new osg::OperationThread;
    setDataVariance(osg::Object::DYNAMIC);
}

PointCloud::PointCloud(const PointCloud& rhs, const osg::CopyOp& copyop)
{
    // do nothing
}

static const char *vertSource = { 

    "#version 110\n"
    "attribute vec4 data;\n"
    "uniform float maxReturn;\n"
    "uniform int colorMode;\n"
    "uniform float minIntensity;\n"
    "uniform float maxIntensity;\n"
    "uniform bool classificationFilter[32];\n"
    "uniform float minPointSize;\n"
    "uniform float maxPointSize;\n"
    "uniform float maxPointDistance;\n"
    "uniform float pointSize;\n"
    "uniform bool autoPointSize;\n"
    "uniform float minHeight;\n"
    "uniform float maxHeight;\n"
    "uniform sampler2D colorRamp;\n"

    "vec4 classificationToColor(in int classification)\n"
    "{\n"
    // Ground
    "    if (classification == 2) return vec4(1,0,0,1);\n"    
    // Veg
    "    if (classification == 3 || classification == 4 || classification == 5) return vec4(0,1,0,1);\n"
    // Building
    "    if (classification == 6) return vec4(0.5,0.5,0.5,1.0);\n"
    // Water
    "    if (classification == 9) return vec4(0,0,1,1);\n"
    "    return vec4(0,0,0,1);"
    "}\n"

    "vec4 intensityToColor(in float intensity, in float min, in float max)\n"
    "{\n"
    "    float relIntensity = (intensity-min) / (max - min);\n"
    "    return vec4(relIntensity, relIntensity, relIntensity, 1.0);\n"
    "}\n"

    "void main(void)\n"
    "{\n"    
    "    int classification = int(data.x);\n"
    "    float returnNumber = data.y;\n"
    "    float intensity = data.z;\n"
    "    float height = data.w;\n"
    "    vec4 vertexView = gl_ModelViewMatrix * gl_Vertex;\n"    
    "    gl_Position = gl_ProjectionMatrix * vertexView ;\n"
    "    float distance = -vertexView.z;\n"
    // Hide the vert if it's been filtered out.
    "    if (returnNumber > maxReturn || !classificationFilter[classification])\n"
    "    {\n"
    "        gl_Position = vec4(10000, 10000, 0, 1);\n"
    "    }\n"
    "    else \n"
    "    {\n"    
    "        if (colorMode == 1)\n"
    "        {\n"
    "            gl_FrontColor = intensityToColor(intensity, minIntensity, maxIntensity);\n"    
    "        }\n"
    "        else if (colorMode == 2)\n"
    "        {\n"
    "            gl_FrontColor = classificationToColor(classification);\n"
    "        }\n"
    "        else if (colorMode == 3)\n"
    "        {\n"
    "            if (height < minHeight) \n"
    "            {\n"
    "                gl_FrontColor = vec4(0.0, 1.0, 0.0, 1.0);\n"
    "            }\n"
    "            else\n"
    "            {\n"
    "                gl_FrontColor = vec4(1.0, 0.0, 0.0, 1.0);\n"
    "            }\n"
    "        }\n"
    "        else if (colorMode == 4)\n"
    "        {\n"
    "            float sample = (height - minHeight) / (maxHeight - minHeight);\n"
    "            gl_FrontColor = texture2D(colorRamp, vec2(0.5, sample));\n"
    "        }\n"
    "        else\n"
    "        {\n"
    "            gl_FrontColor = vec4(gl_Color.rgb, 1.0);\n"
    "        }\n"    
    "        if (autoPointSize) {\n"
    "            float factor = 1.0 - smoothstep(0.0, maxPointDistance, distance);\n"
    "            gl_PointSize = pointSize * mix(minPointSize, maxPointSize, factor);\n"
    "        }\n"
    "        else {\n"
    "            gl_PointSize = pointSize;\n"    
    "        }\n"
    "    }\n"
    "}\n"
};

static const char *fragSource = {    
    "void main(void)\n"
    "{\n"    
    "    gl_FragColor = gl_Color;\n"
    "}\n"
};


/********************************************************************/
PointCloudDecorator::PointCloudDecorator():
_pointSize(1.0f),
_maxReturn(5),
_minIntensity(0),
_maxIntensity(255),
_colorMode(RGB),
_minPointSize(1.0),
_maxPointSize(4.0),
_minHeight(2.0),
_maxHeight(300.0),
_maxPointDistance(5000.0),
_autoPointSize(true)
{    
    getOrCreateStateSet()->setMode(GL_POINT_SMOOTH, osg::StateAttribute::ON);

    osg::Program* program =new osg::Program;
    program->addShader(new osg::Shader(osg::Shader::VERTEX, vertSource));
    program->addShader(new osg::Shader(osg::Shader::FRAGMENT, fragSource));
    program->addBindAttribLocation("data", osg::Drawable::ATTRIBUTE_6);
    getOrCreateStateSet()->setAttributeAndModes(program, osg::StateAttribute::ON);

    getOrCreateStateSet()->getOrCreateUniform("maxReturn", osg::Uniform::FLOAT)->set((float)_maxReturn);
    getOrCreateStateSet()->getOrCreateUniform("minIntensity", osg::Uniform::FLOAT)->set((float)(_minIntensity));
    getOrCreateStateSet()->getOrCreateUniform("maxIntensity", osg::Uniform::FLOAT)->set((float)(_maxIntensity));
    getOrCreateStateSet()->getOrCreateUniform("minPointSize", osg::Uniform::FLOAT)->set(_minPointSize);
    getOrCreateStateSet()->getOrCreateUniform("maxPointSize", osg::Uniform::FLOAT)->set(_maxPointSize);
    getOrCreateStateSet()->getOrCreateUniform("maxPointDistance", osg::Uniform::FLOAT)->set(_maxPointDistance);
    getOrCreateStateSet()->getOrCreateUniform("pointSize", osg::Uniform::FLOAT)->set(_pointSize);
    getOrCreateStateSet()->getOrCreateUniform("autoPointSize", osg::Uniform::BOOL)->set(_autoPointSize);
    getOrCreateStateSet()->getOrCreateUniform("minHeight", osg::Uniform::FLOAT)->set(_minHeight);
    getOrCreateStateSet()->getOrCreateUniform("maxHeight", osg::Uniform::FLOAT)->set(_maxHeight);
    getOrCreateStateSet()->getOrCreateUniform("colorMode", osg::Uniform::INT)->set((int)_colorMode);
    getOrCreateStateSet()->getOrCreateUniform("colorRamp", osg::Uniform::SAMPLER_2D)->set(0);

    getOrCreateStateSet()->setMode(GL_BLEND, osg::StateAttribute::ON);

    // To enable setting the point size in the vertex program.
    getOrCreateStateSet()->setMode(GL_VERTEX_PROGRAM_POINT_SIZE, true);

    _classificationFilter = getOrCreateStateSet()->getOrCreateUniform("classificationFilter", osg::Uniform::BOOL, 32);
    for (unsigned int i = 0; i < 32; i++)
    {
        _classificationFilter->setElement(i, true);
    }   
}

float PointCloudDecorator::getPointSize() const
{
    return _pointSize;
}

void PointCloudDecorator::setPointSize(float pointSize)
{
    _pointSize = osg::maximum(pointSize, 0.0f);
    getOrCreateStateSet()->getOrCreateUniform("pointSize", osg::Uniform::FLOAT)->set(_pointSize);
}

float PointCloudDecorator::getMinHeight() const
{
    return _minHeight;
}

void PointCloudDecorator::setMinHeight(float minHeight)
{
    _minHeight = minHeight;
    getOrCreateStateSet()->getOrCreateUniform("minHeight", osg::Uniform::FLOAT)->set(_minHeight);
}

float PointCloudDecorator::getMaxHeight() const
{
    return _maxHeight;
}

void PointCloudDecorator::setMaxHeight(float maxHeight)
{
    _maxHeight = maxHeight;
    getOrCreateStateSet()->getOrCreateUniform("maxHeight", osg::Uniform::FLOAT)->set(_maxHeight);
}


bool PointCloudDecorator::getClassificationVisible(unsigned int classification) const
{
      bool result;
      _classificationFilter->getElement(classification, result);
      return result;
}

void PointCloudDecorator::setClassificationVisible(unsigned int classification, bool enabled)
{
    _classificationFilter->setElement(classification, enabled);
}


unsigned int PointCloudDecorator::getMaxReturn() const
{
    return _maxReturn;
}

void PointCloudDecorator::setMaxReturn(unsigned int maxReturn)
{
    _maxReturn = osg::maximum(1u, maxReturn);
    getOrCreateStateSet()->getOrCreateUniform("maxReturn", osg::Uniform::FLOAT)->set((float)_maxReturn);
}

unsigned short PointCloudDecorator::getMaxIntensity() const
{
    return _maxIntensity;
}

void PointCloudDecorator::setMaxIntensity(unsigned short maxIntensity)
{
    _maxIntensity = maxIntensity;    
    getOrCreateStateSet()->getOrCreateUniform("maxIntensity", osg::Uniform::FLOAT)->set((float)(_maxIntensity));
}

unsigned short PointCloudDecorator::getMinIntensity() const
{
    return _minIntensity;
}

void PointCloudDecorator::setMinIntensity(unsigned short minIntensity)
{
    _minIntensity = minIntensity;    
    getOrCreateStateSet()->getOrCreateUniform("minIntensity", osg::Uniform::FLOAT)->set((float)(_minIntensity));
}

PointCloudDecorator::ColorMode PointCloudDecorator::getColorMode() const
{
    return _colorMode;
}

void PointCloudDecorator::setColorMode(PointCloudDecorator::ColorMode colorMode)
{
    _colorMode = colorMode;
    getOrCreateStateSet()->getOrCreateUniform("colorMode", osg::Uniform::INT)->set((int)_colorMode);
}

float PointCloudDecorator::getMinPointSize() const
{
    return _minPointSize;
}

void PointCloudDecorator::setMinPointSize(float minPointSize)
{
    _minPointSize = minPointSize;
    getOrCreateStateSet()->getOrCreateUniform("minPointSize", osg::Uniform::FLOAT)->set(_minPointSize);
}

float PointCloudDecorator::getMaxPointSize() const
{
    return _maxPointSize;
}

void PointCloudDecorator::setMaxPointSize(float maxPointSize)
{
    _maxPointSize = maxPointSize;
    getOrCreateStateSet()->getOrCreateUniform("maxPointSize", osg::Uniform::FLOAT)->set(_maxPointSize);
}


float PointCloudDecorator::getMaxPointDistance() const
{
    return _maxPointDistance;
}

void PointCloudDecorator::setMaxPointDistance(float distance)
{
    _maxPointDistance = distance;
    getOrCreateStateSet()->getOrCreateUniform("maxPointDistance", osg::Uniform::FLOAT)->set(_maxPointDistance);
}

bool PointCloudDecorator::getAutoPointSize() const
{
    return _autoPointSize;
}

void PointCloudDecorator::setAutoPointSize(bool autoPointSize)
{
    _autoPointSize = autoPointSize;
    getOrCreateStateSet()->getOrCreateUniform("autoPointSize", osg::Uniform::BOOL)->set(_autoPointSize);
}


osg::Texture2D* PointCloudDecorator::getColorRamp()
{
    return dynamic_cast<osg::Texture2D*>(getOrCreateStateSet()->getTextureAttribute(0, osg::StateAttribute::TEXTURE));
}

void PointCloudDecorator::setColorRamp( osg::Texture2D* colorRamp )
{
    getOrCreateStateSet()->setTextureAttributeAndModes(0, colorRamp, osg::StateAttribute::ON);
}
