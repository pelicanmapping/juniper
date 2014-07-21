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
#include "Utils"
#include <string.h>
#include <osgDB/FileUtils>
#include <osgDB/FileNameUtils>
#include <osgUtil/Optimizer>

using namespace osgJuniper;

osg::Image*
Utils::resizeImage( const osg::Image* input, unsigned int new_s, unsigned int new_t )
{
    osg::Image* output = NULL;

    GLenum pf = input->getPixelFormat();

    if ( input && new_s > 0 && new_t > 0 && 
        (pf == GL_RGBA || pf == GL_RGB || pf == GL_LUMINANCE || pf == GL_LUMINANCE_ALPHA) )
    {
        float s_ratio = (float)input->s()/(float)new_s;
        float t_ratio = (float)input->t()/(float)new_t;

        output = new osg::Image();
        output->allocateImage( new_s, new_t, 1, pf, input->getDataType(), input->getPacking() );
        output->setInternalTextureFormat( input->getInternalTextureFormat() );

        unsigned int pixel_size_bytes = input->getRowSizeInBytes() / input->s();

        for( unsigned int output_row=0; output_row < output->t(); output_row++ )
        {
            // get an appropriate input row
            float output_row_ratio = (float)output_row/(float)output->t();
            unsigned int input_row = (unsigned int)( output_row_ratio * (float)input->t() );
            if ( input_row >= input->t() ) input_row = input->t()-1;
            else if ( input_row < 0 ) input_row = 0;

            for( unsigned int output_col = 0; output_col < output->s(); output_col++ )
            {
                float output_col_ratio = (float)output_col/(float)output->s();
                unsigned int input_col = (unsigned int)( output_col_ratio * (float)input->s() );
                if ( input_col >= input->s() ) input_col = input->s()-1;
                else if ( input_row < 0 ) input_row = 0;
                
                ::memcpy( output->data( output_col, output_row ), input->data( input_col, input_row ), pixel_size_bytes );
            }
        }
    }

    return output;
}

osg::Vec4
Utils::randomColor()
{
    float r = (float)rand() / (float)RAND_MAX;
    float g = (float)rand() / (float)RAND_MAX;
    float b = (float)rand() / (float)RAND_MAX;
    return osg::Vec4(r,g,b,1.0f);
}

osg::Vec3
Utils::randomVert()
{
    float x = (float)rand() / (float)RAND_MAX;
    float y = (float)rand() / (float)RAND_MAX;
    float z = (float)rand() / (float)RAND_MAX;
    return osg::Vec3(x,y,z);
}

std::vector< std::string >
Utils::getFilesFromDirectory(const std::string& directory, const std::string &ext)
{
    osgDB::DirectoryContents contents = osgDB::getDirectoryContents( directory );

    std::vector< std::string > filenames;

    for (unsigned int i = 0; i < contents.size(); ++i)
    {
        if (contents[i] != "." && contents[i] != "..")
        {
            std::string inputFilename = osgDB::concatPaths(directory, contents[i]);
            if (ext.empty() || osgDB::getFileExtension(inputFilename) == ext)
            {
                filenames.push_back( inputFilename );
            }
        }
    }
    return filenames;
}

void
Utils::optimizeMesh( osg::Node* node )
{
    EnableVBOVisitor vbo;
    node->accept( vbo );

    osgUtil::Optimizer opt;
    //opt.optimize( node, osgUtil::Optimizer::DEFAULT_OPTIMIZATIONS | osgUtil::Optimizer::INDEX_MESH | osgUtil::Optimizer::VERTEX_POSTTRANSFORM);    
    opt.optimize( node, osgUtil::Optimizer::INDEX_MESH | osgUtil::Optimizer::VERTEX_POSTTRANSFORM);    
}

std::string
Utils::toLower( const std::string& str)
{
    std::string result = str;
    std::transform( result.begin(), result.end(), result.begin(), ::tolower );
    return result;
}

/***************************************************************************************/

osg::Node*
Utils::findNamedNode(osg::Node* node, const std::string& name)
{
    FindNamedNodeVisitor v(name);
    node->accept( v );
    if (v._foundNodes.size() > 0) return v._foundNodes[0].get();
    return NULL;
}

FindNamedNodeVisitor::FindNamedNodeVisitor(const std::string& name):
osg::NodeVisitor(osg::NodeVisitor::TRAVERSE_ALL_CHILDREN),
_name(name) {}
    
void
FindNamedNodeVisitor::apply(osg::Node& node)
{
    if (node.getName().compare( _name ) == 0)
    {
        _foundNodes.push_back(&node);
    }
    traverse(node);
}


/***************************************************************************************/
EnableVBOVisitor::EnableVBOVisitor():
osg::NodeVisitor(osg::NodeVisitor::TRAVERSE_ALL_CHILDREN)
{
}

void EnableVBOVisitor::apply(osg::Geode& geode)
{
    for (unsigned int i = 0; i < geode.getNumDrawables(); i++)
    {
        osg::Geometry* geom = geode.getDrawable(i)->asGeometry();
        if (geom)
        {
            geom->setUseVertexBufferObjects( true );
            geom->setUseDisplayList( false );
        }
    }
}