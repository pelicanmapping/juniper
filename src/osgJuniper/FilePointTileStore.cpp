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

std::string FilePointTileStore::getKeyPath(const OctreeId& id)
{
	//static long guid = 0;
	std::stringstream buf;
	//buf << "tile_" << id.level << "_" << id.z << "_" << id.x << "_" << id.y << ".laz";
	//buf << id.level << "/" << id.z << "/" << id.x << "/" << id.y << "/" << guid++ << ".laz";
	buf << id.level << "/" << id.z << "/" << id.x << "/" << id.y;
	return osgDB::concatPaths(_path, buf.str());
}

bool FilePointTileStore::get(const OctreeId& id, PointList& points)
{	
	std::string keyPath = getKeyPath(id);

	// Read all the files from the directory
	std::vector< std::string > files = Utils::getFilesFromDirectory(keyPath);
	for (unsigned int i = 0; i < files.size(); i++)
	{
		OSG_NOTICE << "Reading " << files[i] << std::endl;
		PDALUtils::readPointsFromLAZ(points, files[i]);
	}
	/*
	if (osgDB::fileExists(filename))
	{
		PDALUtils::readPointsFromLAZ(points, filename);
		return true;
	}
	*/
	return files.size() > 0;
}

void FilePointTileStore::set(const OctreeId& id, PointList& points, bool append)
{
	if (!append)	
	{
		remove(id);
	}

	std::string keyPath = getKeyPath(id);
	
	std::string filename;
	int index = 0;

	while (true)
	{
		std::stringstream buf;
		buf << keyPath << "/" << index << ".laz";
		filename = buf.str();
		if (!osgDB::fileExists(filename))
		{
			break;
		}
		index++;
	}
	osgEarth::makeDirectoryForFile(filename);
	PDALUtils::writePointsToLaz(points, filename);
	/*
	if (append)
	{
		PDALUtils::appendPointsToLaz(points, filename);
	}
	else
	{
		PDALUtils::writePointsToLaz(points, filename);
	}
	*/
}

void FilePointTileStore::remove(const OctreeId& id)
{
	std::string keyPath = getKeyPath(id);
	// Delete all the files
	std::vector< std::string > files = Utils::getFilesFromDirectory(keyPath);
	for (unsigned int i = 0; i < files.size(); i++)
	{
		unlink(files[i].c_str());
	}


	// Remove the directory
#ifdef WIN32
	::RemoveDirectory(keyPath.c_str());
#else
	rmdir(keyPath.c_str());
#endif

	/*
	std::string filename = getFilename(id);
	if (osgDB::fileExists(filename))
	{
		unlink( filename.c_str() );
	}
	*/
}

void FilePointTileStore::queryKeys(const KeyQuery& query, std::set< OctreeId >& keys)
{
	keys.clear();

	std::vector< std::string > paths;

	osgDB::DirectoryContents levels = osgDB::getDirectoryContents(_path);
	for (unsigned int i = 0; i < levels.size(); i++)
	{		
		std::string level = levels[i];
		std::string levelPath = osgDB::concatPaths(_path, level);
		if (level != "." && level != ".." && osgDB::fileType(levelPath) == osgDB::FileType::DIRECTORY)
		{
			osgDB::DirectoryContents zs = osgDB::getDirectoryContents(levelPath);
			for (unsigned int j = 0; j < zs.size(); j++)
			{
				std::string z = zs[j];
				std::string zPath = osgDB::concatPaths(levelPath, z);

				if (z != "." && z != ".." && osgDB::fileType(zPath) == osgDB::FileType::DIRECTORY)
				{
					osgDB::DirectoryContents xs = osgDB::getDirectoryContents(zPath);
					for (unsigned int k = 0; k < xs.size(); k++)
					{
						std::string x = xs[k];
						std::string xPath = osgDB::concatPaths(zPath, x);

						if (x != "." && x != ".." && osgDB::fileType(xPath) == osgDB::FileType::DIRECTORY)
						{
							osgDB::DirectoryContents ys = osgDB::getDirectoryContents(xPath);
							for (unsigned int m = 0; m < ys.size(); m++)
							{
								std::string y = ys[m];
								std::string yPath = osgDB::concatPaths(xPath, y);

								if (y != "." && y != ".." && osgDB::fileType(yPath) == osgDB::FileType::DIRECTORY)
								{
									// Makes sure have some files
									osgDB::DirectoryContents files = osgDB::getDirectoryContents(yPath);
									if (!files.empty())
									{
										keys.insert(OctreeId(osgEarth::as<int>(level, -1),
											osgEarth::as<int>(x, -1),
											osgEarth::as<int>(y, -1),
											osgEarth::as<int>(z, -1)));
										OSG_NOTICE << "Adding key " << level << "/" << z << "/" << x << "/" << y << std::endl;
									}
								}
							}
						}
					}
				}
			}
		}
	}

#if 0

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
#endif
}
