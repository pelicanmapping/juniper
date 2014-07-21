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

#include <osgDB/ReadFile>
#include <osgDB/WriteFile>
#include <osgDB/FileUtils>
#include <osgDB/FileNameUtils>
#include <osgDB/PluginQuery>
#include <osgJuniper/Utils>

#include <iostream>

using namespace osgJuniper;


static void usage()
{
    OSG_NOTICE << "usage:" << std::endl
        << "osgjuniper_optimize infile outfile" << std::endl;
}

int main( int argc, char **argv )
{    
    osg::ArgumentParser arguments(&argc,argv);

    bool enableVBO = true;

    std::vector< std::string > fileNames;
    for(int pos=1;pos<arguments.argc();++pos)
    {
        if (!arguments.isOption(pos))
        {
            fileNames.push_back(arguments[pos]);
        }
    }

    if (fileNames.size() != 2)
    {
        usage();
        return 1;
    }

    osg::ref_ptr< osg::Node > root = osgDB::readNodeFile( fileNames[0] );

    if (!root.valid())
    {
        OSG_NOTICE << "Failed to load " << fileNames[0] << std::endl;
        return 1;
    }

    Utils::optimizeMesh( root.get() );

    
    osgDB::writeNodeFile( *root.get(), fileNames[1]);

    return 0;      
}
