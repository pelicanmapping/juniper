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
#include "lasreader.hpp"
#include "laswriter.hpp"
#include <osgJuniper/Octree>
#include <osgEarth/Profile>
#include <osgEarth/StringUtils>
#include <osgEarth/FileUtils>
#include <osg/BoundingBox>
#include <osg/OperationThread>
#include <osgDB/FileUtils>
#include <osgDB/FileNameUtils>
#include <osg/ArgumentParser>
#include <iostream>
#include <osgJuniper/Utils>

using namespace osgEarth;
using namespace osgJuniper;

osg::Vec3d reprojectPoint(const osg::Vec3d& input, osgEarth::SpatialReference* srcSRS, osgEarth::SpatialReference* destSRS, bool makeWorld)
{
    osgEarth::GeoPoint geoPoint(srcSRS, input);
    osgEarth::GeoPoint mapPoint;
    geoPoint.transform(destSRS, mapPoint);
    if (!makeWorld)
    {
        return mapPoint.vec3d();
    }
    osg::Vec3d world;
    mapPoint.toWorld(world);
    return world;
}

int main(int argc, char** argv)
{
    osg::Timer_t startTime = osg::Timer::instance()->tick();

    osg::ArgumentParser arguments(&argc,argv);


    std::string srcSRSString;
    if (!arguments.read("--src", srcSRSString))
    {
        OSG_NOTICE << "Please provide a source srs" << std::endl;
        return 1;
    }

    osg::ref_ptr< osgEarth::SpatialReference > srcSRS = osgEarth::SpatialReference::create(srcSRSString);
    if (!srcSRS.valid())
    {
        OSG_NOTICE << srcSRSString << " is not a valid SRS" << std::endl;
    }

    std::string destSRSString;
    arguments.read("--dest", destSRSString);

    bool geocentric = arguments.read("--geocentric");

    if (geocentric)
    {
        destSRSString = "epsg:4326";
    }

    if (destSRSString.empty())
    {
        OSG_NOTICE << "Please provide a destintation srs" << std::endl;
        return 1;
    }

    osg::ref_ptr< osgEarth::SpatialReference > destSRS = osgEarth::SpatialReference::create(destSRSString);
    if (!destSRS.valid())
    {
        OSG_NOTICE << destSRSString << " is not a valid SRS" << std::endl;
    }


    std::string input;
    std::string output;

    //Read in the filenames to process
    for(int pos=1;pos<arguments.argc();++pos)
    {
        if (!arguments.isOption(pos))
        {
            if (input.empty())
            {
                input = arguments[pos];
            }
            else if (output.empty())
            {
                output = arguments[pos];
            }

            if (!input.empty() && !output.empty())
            {
                break;
            }
        }
    }

    if (input.empty())
    {
        OSG_NOTICE << "Please specify a filename" << std::endl;
        return 1;
    }

    if (output.empty())
    {
        OSG_NOTICE << "Please specify a destination" << std::endl;
        return 1;
    }

    LASreadOpener lasreadopener;
    lasreadopener.set_file_name(input.c_str());
    lasreadopener.set_merged(TRUE);
    lasreadopener.set_populate_header(TRUE);

    LASreader *reader = lasreadopener.open();

    // Figure out the offset in geocentric by computing the midpoint from the file.
    LASquantizer *quantizer = new LASquantizer();

    osg::Vec3d midPoint((reader->header.min_x+reader->header.max_x)/2.0,
                        (reader->header.min_y+reader->header.max_y)/2.0,
                        (reader->header.min_z+reader->header.max_z)/2.0);

    double precision = 0.1;
    if ((destSRS->isGeodetic() || destSRS->isGeographic()) && !geocentric)
    {
        precision = 1e-7;
    }

    osg::Vec3d center = reprojectPoint(midPoint, srcSRS.get(), destSRS.get(), geocentric);
    quantizer->x_scale_factor = precision;
    quantizer->y_scale_factor = precision;
    quantizer->z_scale_factor = precision;
    quantizer->x_offset = ((I64)((center.x()/quantizer->x_scale_factor)/10000000))*10000000*quantizer->x_scale_factor;
    quantizer->y_offset = ((I64)((center.y()/quantizer->y_scale_factor)/10000000))*10000000*quantizer->y_scale_factor;
    quantizer->z_offset = ((I64)((center.z()/quantizer->z_scale_factor)/10000000))*10000000*quantizer->z_scale_factor;



    /*
    osg::Vec3d center = reprojectPoint(midPoint, srcSRS.get(), destSRS.get(), geocentric);
    OSG_NOTICE << "Midpoint is " << midPoint.x() << ", " << midPoint.y() << ", " << midPoint.z() << std::endl;
    OSG_NOTICE << "Geocentric is " << center.x() << ", " << center.y() << ", " << center.z() << std::endl;
    OSG_NOTICE << "Precision " << precision << std::endl;
    quantizer->x_scale_factor = precision;
    quantizer->y_scale_factor = precision;
    quantizer->z_scale_factor = precision;
    quantizer->x_offset = 0.0;//((I64)((center.x()/quantizer->x_scale_factor)/10000000))*10000000*quantizer->x_scale_factor;
    quantizer->y_offset = 0.0;//((I64)((center.y()/quantizer->y_scale_factor)/10000000))*10000000*quantizer->y_scale_factor;
    quantizer->z_offset = 0.0;//((I64)((center.z()/quantizer->z_scale_factor)/10000000))*10000000*quantizer->z_scale_factor;
    */


    LASheader* writeHeader = new LASheader;
    *writeHeader = reader->header;
    *writeHeader = *quantizer;

    LASwriteOpener writeOpener;
    writeOpener.set_file_name(output.c_str());
    LASwriter* writer = writeOpener.open(writeHeader);


    LASpoint* point = new LASpoint;
    point->init(quantizer, reader->header.point_data_format, reader->header.point_data_record_length);

    while (reader->read_point())
    {
        // Reproject the point
        osg::Vec3d world = reprojectPoint(osg::Vec3d(reader->point.get_x(), reader->point.get_y(), reader->point.get_z()), srcSRS, destSRS, geocentric);

        // Copy all of the data from the original point.
        *point = reader->point;

        point->set_x(world.x());
        point->set_y(world.y());
        point->set_z(world.z());
        writer->write_point(point);
        writer->update_inventory(point);
    }

    delete point;

    writer->update_header(writeHeader, TRUE);
    writer->close();
    delete writer;

    reader->close();
    delete reader;
}
