/* -*-c++-*- */
/* osgJuniper - Large Dataset Visualization Toolkit for OpenSceneGraph
 * Copyright 2010-2011 Pelican Ventures, Inc.
 * http://wush.net/trac/juniper
 *
 * osgEarth is free software; you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>
 */

#include <osgJuniper/Point>

#include <osgDB/ReadFile>

using namespace osgJuniper;

/****************************************************************************/
Point::Point()
{
}

Point::Point(const osg::Vec3d& position, const osg::Vec3& normal, const osg::Vec4 &color):
_position(position),
_normal(normal),
_color(color),
_size(1.0)
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
PointListCursor::hasMore() const
{
    return _itr != _points.end();
}

bool
PointListCursor::nextPoint( Point &point )
{
    if (hasMore())
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
CompositePointSourceCursor::hasMore() const
{
    return _iter != _pointSources.end();
}

bool
CompositePointSourceCursor::nextPoint(Point& point)
{
    bool gotPoint = false;
    while (!gotPoint)
    {
        if (!hasMore()) break;
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





