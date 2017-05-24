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
#include <osgDB/ReaderWriter>
#include <osg/io_utils>
#include <fstream>
#include <sstream>

#include <osgJuniper/Point>

using namespace osgJuniper;

class CSVPointSource : public PointSource
{
public:
    
    class CSVPointCursor : public PointCursor
    {
    public:
        
        CSVPointCursor(const std::string& filename):
          _filename(filename)
        {
            _in.open(filename.c_str(), std::ios::in );
            if (!_in.is_open())
            {
                osg::notify(osg::NOTICE) << "Failed when opening " << filename << " for reading" << std::endl;
            }
        }

        virtual bool nextPoint(Point& point)
        {
            osg::Vec3 position;            
            bool gotPoint = false;
            while (!_in.eof() && !gotPoint)
            {
                unsigned int numRead = 0;
                //Read a line from the file
                std::string line;
                getline(_in, line);

                std::stringstream ss(line);
                std::string field;
                while(getline(ss, field, ','))
                {
                    std::stringstream fs(field);
                    float f = FLT_MAX;
                    fs >> f;
                    //If we didn't read in the value correctly, exit and read the next line
                    if (f == FLT_MAX) break;
                    position[numRead] = f;
                    numRead++;
                    if (numRead == 3) break;
                }
                if (numRead == 3) gotPoint = true;
            }
            if (gotPoint)
            {
                point.color    = osg::Vec4ub(255,255,255,255);
                point.position = position;
                return true;
            }
            return false;
        }

        std::ifstream _in;
        std::string _filename;
    };


    CSVPointSource(const std::string &filename):
      _filename(filename)
    {
    }

    virtual PointCursor* createPointCursor()
    {
        return new CSVPointCursor( _filename );
    }



protected:
    std::string _filename;
};

class CSVPointsReaderWriter : public osgDB::ReaderWriter
{
public:
    CSVPointsReaderWriter()
    {
        supportsExtension( "csv", className() );
    }

    virtual const char* className()
    {
        return "CSV Point Reader";
    }

    virtual ReadResult readObject(const std::string& file_name, const Options* options) const
    {
        if ( !acceptsExtension(osgDB::getLowerCaseFileExtension( file_name )))
            return ReadResult::FILE_NOT_HANDLED;

        return new CSVPointSource(file_name);
    }
};

REGISTER_OSGPLUGIN(csv, CSVPointsReaderWriter)

