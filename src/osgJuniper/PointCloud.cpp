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

    "vec4 classificationToColor(in float classification)\n"
    "{\n"
    // Ground
    "    if (classification == 2.0) return vec4(1,0,0,1);\n"    
    // Veg
    "    if (classification == 3.0 || classification == 4.0 || classification == 5.0) return vec4(0,1,0,1);\n"
    // Building
    "    if (classification == 6.0) return vec4(0.5,0.5,0.5,1);\n"
    // Water
    "    if (classification == 9.0) return vec4(0,0,1,1);\n"
    "    return vec4(0,0,0,1);"
    "}\n"

    "vec4 intensityToColor(in float intensity, in float min, in float max)\n"
    "{\n"
    "    float relIntensity = (intensity-min) / (max - min);\n"
    "    return vec4(relIntensity, relIntensity, relIntensity, 1.0);\n"
    "}\n"

    "void main(void)\n"
    "{\n"    
    "    float classification = data.x;\n"
    "    float returnNumber = data.y;\n"
    "    float intensity = data.z;\n"
    "    vec4 position = gl_ModelViewProjectionMatrix * gl_Vertex;\n"
    "    float distToCamera = clamp(500.0 / length(position), 0.0, 1.0);\n"
    "    gl_Position = position;\n"
    //"    gl_PointSize = clamp(distToCamera / 100.0, 1.0, 10.0);\n"
    //"    gl_PointSize = mix(1.0, 10.0, distToCamera);\n"
    // Transparent if we are outside of the max return.
    "    if (returnNumber > maxReturn)\n"
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
    "        else\n"
    "        {\n"
    "            gl_FrontColor = vec4(gl_Color.rgb, 1.0);\n"
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
_point(0),
_maxReturn(5),
_minIntensity(0),
_maxIntensity(255),
_colorMode(ColorMode::RGB)
{
    _point = new osg::Point();
    _point->setMinSize(1.0);
    _point->setMaxSize(10.0);
    _point->setDistanceAttenuation(osg::Vec3(1,10,50));
    _point->setFadeThresholdSize(50.0f);
    getOrCreateStateSet()->setAttributeAndModes(_point, osg::StateAttribute::ON);

    osg::Program* program =new osg::Program;
    program->addShader(new osg::Shader(osg::Shader::VERTEX, vertSource));
    program->addShader(new osg::Shader(osg::Shader::FRAGMENT, fragSource));
    program->addBindAttribLocation("data", osg::Drawable::ATTRIBUTE_6);
    getOrCreateStateSet()->setAttributeAndModes(program, osg::StateAttribute::ON);

    getOrCreateStateSet()->getOrCreateUniform("maxReturn", osg::Uniform::FLOAT)->set((float)_maxReturn);

    getOrCreateStateSet()->getOrCreateUniform("minIntensity", osg::Uniform::FLOAT)->set((float)(_minIntensity));
    getOrCreateStateSet()->getOrCreateUniform("maxIntensity", osg::Uniform::FLOAT)->set((float)(_maxIntensity));

    getOrCreateStateSet()->getOrCreateUniform("colorMode", osg::Uniform::INT)->set((int)_colorMode);

    getOrCreateStateSet()->setMode(GL_BLEND, osg::StateAttribute::ON);

    //getOrCreateStateSet()->setMode(GL_VERTEX_PROGRAM_POINT_SIZE, osg::StateAttribute::ON);
}

float PointCloudDecorator::getPointSize() const
{
    return _pointSize;
}

void PointCloudDecorator::setPointSize(float pointSize)
{
    _pointSize = osg::maximum(pointSize, 0.0f);
    _point->setSize(_pointSize);
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
