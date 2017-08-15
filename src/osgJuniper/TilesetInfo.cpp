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

#include <osgJuniper/TilesetInfo>
#include <osgEarth/JsonUtils>
#include <iostream>
#include <fstream>

using namespace osgEarth;
using namespace osgJuniper;

TilesetInfo::TilesetInfo() :
	_additive(false),
	_driver("filesystem"),
	_path(".")
{
}

TilesetInfo::TilesetInfo(const TilesetInfo& rhs) :
	_additive(rhs._additive),
	_driver(rhs._driver),
	_path(rhs._path),
	_bounds(rhs._bounds)
{
}

bool TilesetInfo::getAdditive() const
{
	return _additive;
}

void TilesetInfo::setAdditive(bool additive)
{
	_additive = additive;
}

const osg::BoundingBoxd& TilesetInfo::getBounds() const
{
	return _bounds;
}

void TilesetInfo::setBounds(const osg::BoundingBoxd& bounds)
{
	_bounds = bounds;
}

const std::string& TilesetInfo::getDriver() const
{
	return _driver;
}

void TilesetInfo::setDriver(const std::string& driver)
{
	_driver = driver;
}

const std::string& TilesetInfo::getPath() const
{
	return _path;
}

void TilesetInfo::setPath(const std::string& path)
{
	_path = path;
}

TilesetInfo TilesetInfo::read(const std::string& filename)
{
	TilesetInfo info;

	Json::Reader reader;
	Json::Value root(Json::objectValue);

	std::ifstream in(filename);
	if (reader.parse(in, root))
	{
		info._driver = root["driver"].asString();
		info._path = root["path"].asString();
		info._additive = root["additive"].asBool();
		Json::Value bounds = root["bounds"];
		info._bounds.xMin() = bounds[0u].asDouble();
		info._bounds.yMin() = bounds[1u].asDouble();
		info._bounds.zMin() = bounds[2u].asDouble();
		info._bounds.xMax() = bounds[3u].asDouble();
		info._bounds.yMax() = bounds[4u].asDouble();
		info._bounds.zMax() = bounds[5u].asDouble();
	}
	in.close();
	return info;
}

void TilesetInfo::write(const TilesetInfo& info, const std::string& filename)
{
	Json::Value root;

	Json::Value bounds(Json::arrayValue);
	bounds.append(info._bounds.xMin());
	bounds.append(info._bounds.yMin());
	bounds.append(info._bounds.zMin());
	bounds.append(info._bounds.xMax());
	bounds.append(info._bounds.yMax());
	bounds.append(info._bounds.zMax());

	root["bounds"] = bounds;
	root["additive"] = Json::Value(info._additive);
	root["driver"] = Json::Value(info._driver);
	root["path"] = Json::Value(info._path);

	std::string result = Json::StyledWriter().write(root);	
	std::ofstream out(filename);
	out << result;
	out.close();
}