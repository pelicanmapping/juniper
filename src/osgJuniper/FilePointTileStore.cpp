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
#include <osgJuniper/Utils>

#include <osgDB/FileNameUtils>
#include <osgDB/FileUtils>
#include <osgEarth/FileUtils>
#include <osgEarth/StringUtils>

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
	//return osgDB::convertFileNameToUnixStyle(osgDB::concatPaths(_path, buf.str()));
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

void FilePointTileStore::remove(const OctreeId& id)
{
	std::string filename = getFilename(id);
	if (osgDB::fileExists(filename))
	{
		unlink( filename.c_str() );
	}
}

bool FilePointTileStore::hasKey(const OctreeId& id)
{
	std::string filename = getFilename(id);
	return osgDB::fileExists(filename);
}

void FilePointTileStore::queryKeys(const KeyQuery& query, std::set< OctreeId >& keys)
{
	keys.clear();

	std::vector< std::string > filenames;

	std::vector< std::string > contents = osgJuniper::Utils::getFilesFromDirectory(_path, "laz");
	for (unsigned int i = 0; i < contents.size(); i++)
	{
		std::string filename = osgDB::getSimpleFileName(contents[i]);

		osgEarth::StringTokenizer tok("_.");
		osgEarth::StringVector tized;
		tok.tokenize(filename, tized);

		if (tized.size() == 6)
		{
			int level = osgEarth::as<int>(tized[1], 0);
			int z = osgEarth::as<int>(tized[2], 0);
			int x = osgEarth::as<int>(tized[3], 0);
			int y = osgEarth::as<int>(tized[4], 0);

			if (
				(query._minLevel < 0 || level >= query._minLevel) &&
				(query._maxLevel < 0 || level <= query._maxLevel) &&
				(query._minX < 0 || x >= query._minX) &&
				(query._maxX < 0 || x <= query._maxX) &&
				(query._minY < 0 || y >= query._minY) &&
				(query._maxY < 0 || y <= query._maxY) &&
				(query._minZ < 0 || z >= query._minZ) &&
				(query._maxZ < 0 || z <= query._maxZ)
				)
			{				
				keys.insert(OctreeId(level, x, y, z));
			}
		}
	}	
}
