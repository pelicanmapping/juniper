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
#include <osgJuniper/PointReaderWriter>

#include <pdal/pdal_macros.hpp>

using namespace osgJuniper;

PointReader::PointReader(const std::string& filename) :
	_filename(filename)
{
	_in.open(_filename.c_str(), std::ios::in | std::ios::binary);
	if (!_in.is_open())
	{
		osg::notify(osg::NOTICE) << "Failed when opening " << filename << " for reading" << std::endl;
	}
}

bool PointReader::hasMore() const
{
	return _in.is_open() && !_in.eof();
}

bool PointReader::read(Point& point)
{
	if (hasMore())
	{
		double x, y, z;
		int r, g, b;
		if (_in.read((char*)(&x), sizeof(double)) &&
			_in.read((char*)(&y), sizeof(double)) &&
			_in.read((char*)(&z), sizeof(double)) &&
			_in.read((char*)(&r), sizeof(int)) &&
			_in.read((char*)(&g), sizeof(int)) &&
			_in.read((char*)(&b), sizeof(int)))
		{
			point.x = x;
			point.y = y;
			point.z = z;

			point.r = r;
			point.g = g;
			point.b = b;
			return true;
		}
	}
	return false;
}

void PointReader::reset()
{
	_in.seekg(0, std::ios_base::beg);
}

PointReader::~PointReader()
{
	if (_in.is_open()) _in.close();
}





PointWriter::PointWriter(const std::string& filename) :
	_filename(filename)
{
}

PointWriter::~PointWriter()
{
	close();
}

const std::string& PointWriter::getFilename() const
{
	return _filename;
}

void PointWriter::close()
{
	if (_out.is_open()) _out.close();
}

void PointWriter::write(pdal::PointRef& point)
{
	if (!_out.is_open())
	{
		_out.open(_filename.c_str(), std::ios::out | std::ios::binary);
	}

	double x = point.getFieldAs<double>(pdal::Dimension::Id::X);
	double y = point.getFieldAs<double>(pdal::Dimension::Id::Y);
	double z = point.getFieldAs<double>(pdal::Dimension::Id::Z);

	int r = point.getFieldAs<int>(pdal::Dimension::Id::Red);
	int g = point.getFieldAs<int>(pdal::Dimension::Id::Green);
	int b = point.getFieldAs<int>(pdal::Dimension::Id::Blue);

	// Position
	_out.write((char*)&x, sizeof(double));
	_out.write((char*)&y, sizeof(double));
	_out.write((char*)&z, sizeof(double));

	// Color
	_out.write((char*)&r, sizeof(int));
	_out.write((char*)&g, sizeof(int));
	_out.write((char*)&b, sizeof(int));
}


