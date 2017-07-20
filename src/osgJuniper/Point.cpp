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
	classification(0),
	intensity(0),
	returnNumber(0)
{
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





