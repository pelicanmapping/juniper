/* -*-c++-*- */
/* osgJuniper - Large Dataset Visualization Toolkit for OpenSceneGraph
* Copyright 2010-2011 Pelican Ventures, Inc.
* http://wush.net/trac/juniper
*
* osgJuniper is free software; you can redistribute it and/or modify
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

#include <osgJuniperMap/Location>

using namespace osgJuniper::Map;

Location::Location():
_latitude(0),
_longitude(0),
_altitude(0)
{
}

Location::Location(double latitude, double longitude, double altitude):
_latitude(latitude),
_longitude(longitude),
_altitude(altitude)
{
}

Location::Location(const Location& rhs):
_latitude(rhs._latitude),
_longitude(rhs._longitude),
_altitude(rhs._altitude)
{
}

double
Location::getLatitude()  const
{
    return _latitude;
}

double
Location::getLongitude() const
{
    return _longitude;
}

double
Location::getAltitude()  const
{
    return _altitude;
}

osg::Vec3d 
Location::asVec3d() const
{
    return osg::Vec3d(_longitude, _latitude, _altitude );
}