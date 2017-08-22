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
#include "Utils"
#include <string.h>
#include <osgDB/FileUtils>
#include <osgDB/FileNameUtils>
#include <osgUtil/Optimizer>

#ifdef WIN32
#include <Windows.h>
#endif

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

std::vector< std::string >
Utils::glob(const std::string& pathname)
{
	std::vector< std::string > results;
#ifdef WIN32
	std::string path = osgDB::getFilePath(pathname);

	WIN32_FIND_DATA data;
	HANDLE handle = FindFirstFile(pathname.c_str(), &data);
	if (handle != INVALID_HANDLE_VALUE)
	{
		do
		{
			results.push_back(osgDB::concatPaths(path, data.cFileName));
		} while (FindNextFile(handle, &data) != 0);
		FindClose(handle);
	}
	return results;

#else
	results.push_back(pathname);
#endif
	return results;
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