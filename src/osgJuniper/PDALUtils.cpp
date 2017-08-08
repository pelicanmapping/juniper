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
#include <osgJuniper/PDALUtils>
#include <osgEarth/FileUtils>
#include <osgDB/FileNameUtils>
#include <osgDB/FileUtils>

#include <pdal/io/BufferReader.hpp>
#include <pdal/filters/StreamCallbackFilter.hpp>

using namespace osgJuniper;
using namespace pdal;

namespace
{
	StageFactory STAGE_FACTORY;
}

static ExtensionToDriverMap s_extensionsToDriverMap =
{
	{ "f32", "readers.f32" },
	{ "points", "readers.points" }
};

void PDALUtils::mapExtensionToDriver(const std::string& extension, const std::string& driver)
{
	s_extensionsToDriverMap[extension] = driver;
}

std::string PDALUtils::inferReaderDriver(const std::string& filename)
{
	std::string ext = osgDB::getFileExtension(filename);
	ExtensionToDriverMap::iterator itr = s_extensionsToDriverMap.find(ext);
	if (itr != s_extensionsToDriverMap.end())
	{
		return itr->second;
	}
	pdal::StageFactory factory;
	return factory.inferReaderDriver(filename);
}



void PDALUtils::writePointsToLaz(const PointList& points, const std::string& filename)
{
	PointTable pointTable;
	pointTable.layout()->registerDim(Dimension::Id::X);
	pointTable.layout()->registerDim(Dimension::Id::Y);
	pointTable.layout()->registerDim(Dimension::Id::Z);
	pointTable.layout()->registerDim(Dimension::Id::Red);
	pointTable.layout()->registerDim(Dimension::Id::Green);
	pointTable.layout()->registerDim(Dimension::Id::Blue);
	PointViewPtr view(new PointView(pointTable));

	int idx = 0;

	for (PointList::const_iterator itr = points.begin(); itr != points.end(); ++itr)
	{
		const Point& point = *itr;
		// The point passed, so include it in the list.
		view->setField(pdal::Dimension::Id::X, idx, point.x);
		view->setField(pdal::Dimension::Id::Y, idx, point.y);
		view->setField(pdal::Dimension::Id::Z, idx, point.z);

		view->setField(pdal::Dimension::Id::Red, idx, point.r);
		view->setField(pdal::Dimension::Id::Green, idx, point.g);
		view->setField(pdal::Dimension::Id::Blue, idx, point.b);
		idx++;
	}

	BufferReader bufferReader;
	bufferReader.addView(view);

	Stage *writer = 0;
	{PDAL_SCOPED_LOCK; writer = STAGE_FACTORY.createStage("writers.las"); }

	osgEarth::makeDirectoryForFile(filename);

	Options options;
	options.add("filename", filename);

	writer->setInput(bufferReader);
	writer->setOptions(options);
	{ PDAL_SCOPED_LOCK; writer->prepare(pointTable); }
	writer->execute(pointTable);

	// Destroy the writer stage, we're done with it.
	{PDAL_SCOPED_LOCK; STAGE_FACTORY.destroyStage(writer); }
}

void PDALUtils::readPointsFromLAZ(PointList& points, const std::string& filename)
{
	points.clear();
	if (osgDB::fileExists(filename))
	{
		Stage* stage = 0;
		{
			PDAL_SCOPED_LOCK;
			stage = STAGE_FACTORY.createStage("readers.las");
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

			for (unsigned int i = 0; i < point_view->size(); i++)
			{
				PointRef point(point_view->point(i));
				Point p;
				p.x = point.getFieldAs<double>(pdal::Dimension::Id::X);
				p.y = point.getFieldAs<double>(pdal::Dimension::Id::Y);
				p.z = point.getFieldAs<double>(pdal::Dimension::Id::Z);
				p.r = point.getFieldAs<int>(pdal::Dimension::Id::Red);
				p.g = point.getFieldAs<int>(pdal::Dimension::Id::Green);
				p.b = point.getFieldAs<int>(pdal::Dimension::Id::Blue);
				points.push_back(p);
			}
		}
	}
}

void PDALUtils::appendPointsToLaz(const PointList& points, const std::string& filename)
{
	PointList pts;
	readPointsFromLAZ(pts, filename);
	pts.insert(pts.end(), points.begin(), points.end());
	writePointsToLaz(pts, filename);
}


OpenThreads::ReentrantMutex& PDALUtils::getPDALMutex()
{
	static OpenThreads::ReentrantMutex _pdal_mutex;
	return _pdal_mutex;
}



