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
#include <osgJuniper/PointTileStore>

#include <osgJuniper/FilePointTileStore>
#include <osgJuniper/RocksDBPointTileStore>

using namespace osgJuniper;

/****************************************************************************/
PointTileStore::PointTileStore()
{
}

KeyQuery::KeyQuery():
	_minLevel(-1),
	_maxLevel(-1),
	_minX(-1),
	_maxX(-1),
	_minY(-1),
	_maxY(-1),
	_minZ(-1),
	_maxZ(-1)
{
}

KeyQuery::KeyQuery(int minLevel, int maxLevel,
	int minX, int maxX,
	int minY, int maxY,
	int minZ, int maxZ):
	_minLevel(minLevel),
	_maxLevel(maxLevel),
	_minX(minX),
	_maxX(maxX),
	_minY(minY),
	_maxY(maxY),
	_minZ(minZ),
	_maxZ(maxZ)
{
}

PointTileStore* PointTileStore::create(const TilesetInfo& info)
{
	if(info.getDriver() == "filesystem")
	{
		return new FilePointTileStore(info.getPath());
	}
#if HAVE_ROCKSDB
	else if (info.getDriver() == "rocksdb")
	{
		return new RocksDBPointTileStore(info.getPath());
	}
#endif
	return 0;
}
