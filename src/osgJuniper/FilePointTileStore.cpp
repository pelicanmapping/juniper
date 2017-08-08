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
#include <osgJuniper/FilePointTileStore>
#include <osgJuniper/PDALUtils>

#include <osgDB/FileNameUtils>
#include <osgDB/FileUtils>
#include <osgEarth/FileUtils>

using namespace osgJuniper;

FilePointTileStore::FilePointTileStore(const std::string& path):
	_path(path)
{
}

const std::string& FilePointTileStore::getPath() const
{
	return _path;
}

std::string FilePointTileStore::getFilename(const OctreeId& id)
{
	std::stringstream buf;
	buf << "tile_" << id.level << "_" << id.z << "_" << id.x << "_" << id.y << ".laz";
	return osgDB::concatPaths(_path, buf.str());
}

bool FilePointTileStore::get(const OctreeId& id, PointList& points)
{	
	std::string filename = getFilename(id);
	if (osgDB::fileExists(filename))
	{
		PDALUtils::readPointsFromLAZ(points, filename);
		return true;
	}
	return false;
}

void FilePointTileStore::set(const OctreeId& id, PointList& points, bool append)
{
	std::string filename = getFilename(id);
	osgEarth::makeDirectoryForFile(filename);
	if (append)
	{
		PDALUtils::appendPointsToLaz(points, filename);
	}
	else
	{
		PDALUtils::writePointsToLaz(points, filename);
	}
}
