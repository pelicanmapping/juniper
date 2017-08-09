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
#include <osgJuniper/RocksDBPointTileStore>
#include <osgJuniper/Utils>

#include <osgDB/FileNameUtils>
#include <osgDB/FileUtils>
#include <osgEarth/FileUtils>
#include <osgEarth/StringUtils>

#include "rocksdb/db.h"

using namespace osgJuniper;

RocksDBPointTileStore::RocksDBPointTileStore(const std::string& path):
_nextID(0)
{	
	rocksdb::DB* db;
	rocksdb::Options options;
	//options.PrepareForBulkLoad();
	options.create_if_missing = true;
	options.compression = rocksdb::kSnappyCompression;
	rocksdb::Status status = rocksdb::DB::Open(options, path, &db);
	_db = db;
}

RocksDBPointTileStore::~RocksDBPointTileStore()
{
	rocksdb::DB* db = (rocksdb::DB*)_db;
	delete db;
	_db = 0;
}

long RocksDBPointTileStore::nextID()
{
	OpenThreads::ScopedLock< OpenThreads::Mutex > lock(_mutex);
	return _nextID++;
}

std::string RocksDBPointTileStore::getKey(const OctreeId& id)
{	
	std::stringstream buf;
	buf << id.level << "/" << id.z << "/" << id.x << "/" << id.y;
	return buf.str();
}

bool RocksDBPointTileStore::get(const OctreeId& id, PointList& points)
{	
	return false;
}

void RocksDBPointTileStore::set(const OctreeId& id, PointList& points, bool append)
{
	long guid = nextID();

	std::ostringstream out;
	for (PointList::iterator itr = points.begin(); itr != points.end(); ++itr)
	{
		Point& point = *itr;
		float position[3];
		position[0] = point.x;
		position[1] = point.y;
		position[2] = point.z;

		char color[3];
		color[0] = point.r / 65536;
		color[1] = point.g / 65536;
		color[2] = point.b / 65536;

		out.write((char*)position, sizeof(float) * 3);
		out.write((char*)color, sizeof(char) * 3);
		out.write((char*)&point.classification, sizeof(unsigned char));
		out.write((char*)&point.intensity, sizeof(unsigned short));
	}	

	std::stringstream key;
	key << getKey(id) << "/" << guid;
	((rocksdb::DB*)_db)->Put(rocksdb::WriteOptions(), key.str(), out.str());
}

void RocksDBPointTileStore::queryKeys(const KeyQuery& query, std::set< OctreeId >& keys)
{
	// TODO:
}
