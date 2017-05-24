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
#include <osg/Geometry>
#include <osg/Geode>
#include <osg/Point>
#include <osg/LOD>
#include <osg/ShapeDrawable>
#include <osgDB/Registry>
#include <osgDB/FileNameUtils>
#include <osgDB/ReadFile>
#include <osgDB/ReaderWriter>
#include <osgUtil/SmoothingVisitor>
#include <fstream>

#include <osg/AnimationPath>
#include <osg/MatrixTransform>


class PathReader : public osgDB::ReaderWriter
{
public:
    PathReader()
    {
        supportsExtension( "juniper_path", className() );
    }

    virtual const char* className()
    {
        return "AnimationPath Reader";
    }    

    virtual ReadResult readNode( const std::string& location, const osgDB::ReaderWriter::Options* options ) const
    {
        if ( !acceptsExtension( osgDB::getLowerCaseFileExtension( location ) ) )
            return ReadResult::FILE_NOT_HANDLED;

        std::string filename = osgDB::getNameLessExtension( location );

        std::string::size_type firstDot = filename.find_last_of('.');
        std::string::size_type secondDot = filename.find_last_of('.', firstDot-1 );
        OSG_NOTICE << "First dot = " << firstDot << " second dot = " << secondDot << std::endl;
        std::string animationPath = std::string( filename.begin() + secondDot+1, filename.end());
        std::string file = std::string( filename.begin(), filename.begin() + secondDot );

        std::string dir = osgDB::getFilePath( file );
        OSG_NOTICE << "Dir=" << dir << std::endl;
        animationPath = dir + "/" + animationPath;

        OSG_NOTICE << "Filename =" << file << std::endl;
        OSG_NOTICE << "Animation path =" << animationPath << std::endl;

       
        osg::Node* node = osgDB::readNodeFile( file );
        if (!node) return ReadResult::FILE_NOT_FOUND;
       
        //Read the path
        osg::AnimationPath* path = new osg::AnimationPath;
        std::ifstream in( animationPath.c_str() );
        path->read( in );

        osg::MatrixTransform* mt = new osg::MatrixTransform;
        mt->setUpdateCallback( new osg::AnimationPathCallback(path, 0.0, 1.0));
        mt->addChild( node );
        return mt;
    }

};

REGISTER_OSGPLUGIN(juniper_path, PathReader)

