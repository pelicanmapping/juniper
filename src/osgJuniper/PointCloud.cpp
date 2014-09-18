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
    "uniform bool colorByClassification;\n"

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

    "void main(void)\n"
    "{\n"    
    "    float classification = data.x;\n"
    "    float returnNumber = data.y;\n"
    "    float intensity = data.z;\n"
    "    gl_Position = gl_ModelViewProjectionMatrix * gl_Vertex;\n"
    // Transparent if we are outside of the max return.
    "    if (returnNumber > maxReturn)\n"
    "    {\n"
    "        gl_FrontColor = vec4(0,0,0,0);\n"
    "    }\n"
    "    else \n"
    "    {\n"
    "        if (colorByClassification) {"
    "            gl_FrontColor = classificationToColor(classification);\n"
    "        }\n"
    "        else {\n"
    "            gl_FrontColor = gl_Color;\n"
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
_colorByClassification(false),
_maxReturn(10)
{
    _point = new osg::Point(_pointSize);
    getOrCreateStateSet()->setAttributeAndModes(_point, osg::StateAttribute::ON);

    osg::Program* program =new osg::Program;
    program->addShader(new osg::Shader(osg::Shader::VERTEX, vertSource));
    program->addShader(new osg::Shader(osg::Shader::FRAGMENT, fragSource));
    program->addBindAttribLocation("data", osg::Drawable::ATTRIBUTE_6);
    getOrCreateStateSet()->setAttributeAndModes(program, osg::StateAttribute::ON);

    getOrCreateStateSet()->getOrCreateUniform("colorByClassification", osg::Uniform::BOOL)->set(_colorByClassification);
    getOrCreateStateSet()->getOrCreateUniform("maxReturn", osg::Uniform::FLOAT)->set((float)_maxReturn);
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

bool PointCloudDecorator::getColorByClassification() const
{
    return _colorByClassification;
}

void PointCloudDecorator::setColorByClassification(bool colorByClassification)
{
    _colorByClassification = colorByClassification;
    getOrCreateStateSet()->getOrCreateUniform("colorByClassification", osg::Uniform::BOOL)->set(_colorByClassification);
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
