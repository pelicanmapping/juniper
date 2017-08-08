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
#include <osgJuniper/OctreeId>

using namespace osgJuniper;

/****************************************************************************/
OctreeId::OctreeId(int in_level, int in_x, int in_y, int in_z):
level(in_level),
x(in_x),
y(in_y),
z(in_z)
{
}

OctreeId::OctreeId() :
	level(-1),
	x(-1),
	y(-1),
	z(-1)
{
}

bool OctreeId::operator == (const OctreeId& rhs) const
{
	return (level == rhs.level) && (x == rhs.x) && (y == rhs.y) && (z == rhs.z);
}

bool OctreeId::operator != (const OctreeId& rhs) const
{
	return (level != rhs.level) || (x != rhs.x) || (y != rhs.y) || (z != rhs.z);
}

bool OctreeId::operator < (const OctreeId& rhs) const
{
	if (level<rhs.level) return true;
	if (level>rhs.level) return false;
	if (x<rhs.x) return true;
	if (x>rhs.x) return false;
	if (y<rhs.y) return true;
	if (y>rhs.y) return false;
	return z<rhs.z;
}

bool OctreeId::valid() const { return level >= 0; }