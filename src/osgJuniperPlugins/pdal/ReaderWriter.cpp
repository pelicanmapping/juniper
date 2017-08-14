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
#include <osgDB/FileNameUtils>
#include <osgDB/FileUtils>
#include <osgDB/Registry>
#include <osgDB/ReaderWriter>
#include <osg/io_utils>
#include <osg/MatrixTransform>

#include <fstream>
#include <sstream>

#include <pdal/StageFactory.hpp>
#include <pdal/PointTable.hpp>
#include <pdal/PipelineExecutor.hpp>

#include <osgJuniper/PDALUtils>
#include <osgJuniper/PointCloud>

using namespace pdal;
using namespace osgJuniper;

class PDALReaderWriter : public osgDB::ReaderWriter
{
public:
	PDALReaderWriter()
    {
		supportsExtension( "pdal", className());
    }

    virtual const char* className()
    {
        return "PDAL Point Reader";
    }

	virtual ReadResult readNode( const std::string& location, const osgDB::ReaderWriter::Options* options ) const
    {
		std::string filename = location;
		if (osgDB::getFileExtension(location) == "pdal")
		{
			filename = osgDB::getNameLessExtension(location);
		}

		StageFactory factory;
		std::string driver = options ? options->getPluginStringData("pdal.driver") : "";
		if (driver.empty())
		{
			driver = PDALUtils::inferReaderDriver(filename);
		}		

		if (driver.empty() && osgDB::getFileExtension(filename) == "json")
		{
			driver = "pipeline";
		}
		
		if (driver.empty())
		{
			return ReadResult::FILE_NOT_HANDLED;
		}

		Stage* stage = 0;

		PipelineManager pipeline;
		
		if (driver == "pipeline")
		{			
			pipeline.readPipeline(filename);			
			stage = pipeline.getStage();
		}
		else
		{
			PDAL_SCOPED_LOCK;
			stage = factory.createStage(driver);
			pdal::Options opt;
			opt.add("filename", filename);
			stage->setOptions(opt);
		}

		if (stage)
		{			
			pdal::PointTable table;
			{ PDAL_SCOPED_LOCK;  stage->prepare(table); }

			pdal::PointViewSet point_view_set = stage->execute(table);
			pdal::PointViewPtr point_view = *point_view_set.begin();

			PointList points;
			points.reserve(point_view->size());

			for (unsigned int i = 0; i < point_view->size(); i++)
			{
				PointRef pdalPoint(point_view->point(i));
				Point point;

				point.x = pdalPoint.getFieldAs<double>(Dimension::Id::X);
				point.y = pdalPoint.getFieldAs<double>(Dimension::Id::Y);
				point.z = pdalPoint.getFieldAs<double>(Dimension::Id::Z);
				point.r = pdalPoint.getFieldAs<int>(Dimension::Id::Red);
				point.g = pdalPoint.getFieldAs<int>(Dimension::Id::Green);
				point.b = pdalPoint.getFieldAs<int>(Dimension::Id::Blue);
				point.a = pdalPoint.getFieldAs<int>(Dimension::Id::Alpha);
				point.intensity = pdalPoint.getFieldAs<int>(Dimension::Id::Intensity);
				point.returnNumber = pdalPoint.getFieldAs<int>(Dimension::Id::ReturnNumber);
				point.classification = pdalPoint.getFieldAs<int>(Dimension::Id::Classification);
				points.push_back(point);
			}

			return new PointCloud(points);
		}
		return ReadResult::ERROR_IN_READING_FILE;
    }
};

REGISTER_OSGPLUGIN(pdal, PDALReaderWriter)

