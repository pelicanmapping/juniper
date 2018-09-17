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
#include <osgJuniper/Point>

#include <osgDB/ReadFile>

using namespace osgJuniper;

/****************************************************************************/
Point::Point() :
	x(0.0),
	y(0.0),
	z(0.0),
	r(0.0),
	g(0.0),
	b(0.0),
	a(0.0),
    normalX(0.0f),
    normalY(0.0f),
    normalZ(0.0f),
	classification(0),
	intensity(0),
	returnNumber(0)
{
}

void Point::write(std::ostream &out, const std::vector< Point >& points)
{
    // TODO:  Do we need this?
	float minX = FLT_MAX;
	float minY = FLT_MAX;
	float minZ = FLT_MAX;

	float maxX = -FLT_MAX;
	float maxY = -FLT_MAX;
	float maxZ = -FLT_MAX;

	// Compute the min/max of the points so we can quantize it.
	for (PointList::const_iterator itr = points.begin(); itr != points.end(); ++itr)
	{
		if (minX > itr->x) minX = itr->x;
		if (maxX < itr->x) maxX = itr->x;

		if (minY > itr->y) minY = itr->y;
		if (maxY < itr->y) maxY = itr->y;

		if (minZ > itr->z) minZ = itr->z;
		if (maxZ < itr->z) maxZ = itr->z;
	}

	// Write out the min/maxes
	out.write((char*)&minX, sizeof(float));
	out.write((char*)&minY, sizeof(float));
	out.write((char*)&minZ, sizeof(float));

	out.write((char*)&maxX, sizeof(float));
	out.write((char*)&maxY, sizeof(float));
	out.write((char*)&maxZ, sizeof(float));

	float width = maxX - minX;
	float depth = maxY - minY;
	float height = maxZ - minZ;

	for (PointList::const_iterator itr = points.begin(); itr != points.end(); ++itr)
	{
		const Point& point = *itr;

		/*
		float position[3];
		position[0] = point.x;
		position[1] = point.y;
		position[2] = point.z;
		*/

		unsigned short position[3];
		position[0] = (unsigned short)(((point.x - minX) / width) * 65535.0f);
		position[1] = (unsigned short)(((point.y - minY) / depth) * 65535.0f);
		position[2] = (unsigned short)(((point.z - minZ) / height) * 65535.0f);

		unsigned char color[3];
		color[0] = point.r / 256;
		color[1] = point.g / 256;
		color[2] = point.b / 256;

		//out.write((char*)position, sizeof(float) * 3);
		out.write((char*)position, sizeof(unsigned short) * 3);
		out.write((char*)color, sizeof(unsigned char) * 3);
		out.write((char*)&point.classification, sizeof(unsigned char));
		out.write((char*)&point.intensity, sizeof(unsigned short));
	}
}
void Point::read(std::istream &in, std::vector< Point >& points)
{
	int numRead = 0;

	float minX = FLT_MAX;
	float minY = FLT_MAX;
	float minZ = FLT_MAX;

	float maxX = -FLT_MAX;
	float maxY = -FLT_MAX;
	float maxZ = -FLT_MAX;

	// Write out the min/maxes
	in.read((char*)&minX, sizeof(float));
	in.read((char*)&minY, sizeof(float));
	in.read((char*)&minZ, sizeof(float));

	in.read((char*)&maxX, sizeof(float));
	in.read((char*)&maxY, sizeof(float));
	in.read((char*)&maxZ, sizeof(float));

	float width = maxX - minX;
	float depth = maxY - minY;
	float height = maxZ - minZ;	
	while (in.good())
	{
		//float position[3];
		unsigned short position[3];
		unsigned char color[3];
		unsigned char classification;
		unsigned short intensity;

		Point point;
		if (//in.read((char*)position, sizeof(float) * 3) &&
			in.read((char*)position, sizeof(unsigned short) * 3) &&
			in.read((char*)color, sizeof(unsigned char) * 3) &&
			in.read((char*)(&classification), sizeof(unsigned char)) &&
			in.read((char*)(&intensity), sizeof(unsigned short))
			)
		{		
			/*
			point.x = position[0];
			point.y = position[1];
			point.z = position[2];
			*/

			point.x = minX + width * (position[0] / 65535.0f);
			point.y = minY + depth * (position[1] / 65535.0f);
			point.z = minZ + height * (position[2] / 65535.0f);

			point.r = color[0] * 256;
			point.g = color[1] * 256;
			point.b = color[2] * 256;

			point.classification = classification;
			point.intensity = intensity;
			points.push_back(point);
			numRead++;
		}
	}
}

/****************************************************************************/
PointSource*
PointSource::loadPointSource(const std::string &filename, const osgDB::ReaderWriter::Options* options)
{
    return dynamic_cast<PointSource*>(osgDB::readObjectFile(filename, options));
}

/****************************************************************************/
PointListCursor::PointListCursor(osgJuniper::PointList &points):
_points(points),
_numRead(0)
{
    _itr = _points.begin();
}

bool
PointListCursor::nextPoint( Point &point )
{
    if (_itr != _points.end())
    {
        point = *_itr;
        _itr++;
        _numRead++;
        return true;
    }
    return false;
}
/****************************************************************************/

PointListSource::PointListSource()
{
}

PointListSource::PointListSource(osgJuniper::PointList points):
_points(points)
{
}

PointCursor*
PointListSource::createPointCursor()
{
    return new PointListCursor(_points);
}

/****************************************************************************/
CompositePointSource::CompositePointSource()
{
}

CompositePointSource::CompositePointSource(const std::vector< std::string > &filenames)
{
    for (unsigned int i = 0; i < filenames.size(); ++i)
    {     
        this->addPointSource( filenames[i] );
    }
}

void
CompositePointSource::addPointSource(osgJuniper::PointSource *pointSource)
{
    _pointSources.push_back( pointSource );
}

void
CompositePointSource::addPointSource(const std::string &filename)
{
    PointSource* pointSource = PointSource::loadPointSource( filename );
    if (pointSource)
    {
        addPointSource( pointSource );
    }
    else
    {
        OSG_NOTICE << "Failed to load point source from " << filename << std::endl;
    }
}

PointCursor*
CompositePointSource::createPointCursor()
{
    CompositePointSourceCursor* c = new CompositePointSourceCursor(_pointSources);
    return c;
}

CompositePointSourceCursor::CompositePointSourceCursor(PointSourceList& pointSources):
_pointSources(pointSources),
_numRead(0)
{
    _iter = _pointSources.begin();
    _currentCursor = _iter->get()->createPointCursor();
}


bool
CompositePointSourceCursor::nextPoint(Point& point)
{
    bool gotPoint = false;
    while (!gotPoint)
    {
        if (_iter == _pointSources.end()) break;
        //If the current cursor is valid, try to get a point from it
        if (_currentCursor.valid())
        {
            gotPoint = _currentCursor->nextPoint(point);
        }

        //We couldn't get a point from  the cursor, so try to go the next source
        if (!gotPoint)
        {
            _iter++;
            if (_iter == _pointSources.end()) break;
            _currentCursor = _iter->get()->createPointCursor();
        }

        if (gotPoint)
        {
            _numRead++;
        }
    }
    return gotPoint;
}