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

#ifdef HAVE_ROCKSDB
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
	//points.clear();	

	rocksdb::DB* db = (rocksdb::DB*)_db;
	std::string key = getKey(id);

	//OSG_NOTICE << "RocksDB reading from " <<  key << std::endl;

	rocksdb::Iterator* it = db->NewIterator(rocksdb::ReadOptions());
	for (it->Seek(key);
		it->Valid() && osgEarth::startsWith(it->key().ToString(), key);
		it->Next()) {
		//OSG_NOTICE << "RocksDB reading key " << it->key().ToString() << ": value size=" << it->value().ToString().size() << std::endl;
		std::istringstream in(it->value().ToString(), std::ios::binary);		
		Point::read(in, points);		
	}
	delete it;

	//OSG_NOTICE << "RocksDB read " << points.size() << " from " << key << std::endl;
	return true;
}

void RocksDBPointTileStore::set(const OctreeId& id, PointList& points, bool append)
{
	long guid = nextID();

	std::ostringstream out(std::ios::binary);
	Point::write(out, points);

	std::stringstream key;
	key << getKey(id) << "/";
	if (append)
	{
		key << guid;
	}
	rocksdb::WriteOptions opt;
	((rocksdb::DB*)_db)->Put(opt, key.str(), out.str());
}

void RocksDBPointTileStore::queryKeys(const KeyQuery& query, std::set< OctreeId >& keys)
{
	rocksdb::DB* db = (rocksdb::DB*)_db;
	rocksdb::Iterator* it = db->NewIterator(rocksdb::ReadOptions());
	for (it->SeekToFirst(); it->Valid(); it->Next()) {
		unsigned int level, x, y, z, guid;
		sscanf(it->key().ToString().c_str(), "%d/%d/%d/%d/%d", &level, &z, &x, &y, &guid);
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
	delete it;
}

void RocksDBPointTileStore::remove(const OctreeId& id)
{
	rocksdb::DB* db = (rocksdb::DB*)_db;
	std::string key = getKey(id);

	std::vector< std::string > keys;

	rocksdb::Iterator* it = db->NewIterator(rocksdb::ReadOptions());
	for (it->Seek(key);
		it->Valid() && osgEarth::startsWith(it->key().ToString(), key);
		it->Next()) {
		keys.push_back(it->key().ToString());		
	}
	delete it;

	for (std::vector< std::string >::iterator itr = keys.begin(); itr != keys.end(); ++itr)
	{
		OSG_NOTICE << "Rocks deleting key " << *itr << std::endl;
		db->Delete(rocksdb::WriteOptions(), *itr);
	}	

	db->SyncWAL();
}

#endif