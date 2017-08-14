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
#include <osg/PagedLOD>
#include <osg/MatrixTransform>
#include <osgDB/FileNameUtils>
#include <osgDB/Registry>
#include <osgDB/ReaderWriter>
#include <osgDB/FileUtils>
#include <osgDB/ReadFile>
#include <osgJuniper/PointCloud>
#include <osgJuniper/Octree>
#include <osgJuniper/PointTileStore>
#include <osgJuniper/FilePointTileStore>
#include <osgJuniper/RocksDBPointTileStore>
#include <osgEarth/PagedNode>

using namespace osgJuniper;

class LASTileReaderWriter : public osgDB::ReaderWriter
{
public:
	LASTileReaderWriter()
	{
		supportsExtension("lastile", className());

		osgDB::Registry::instance()->addFileExtensionAlias("laz", "pdal");
	}

	virtual const char* className()
	{
		return "LAS Tiled Point Reader";
	}

	class PagedOctreeNode : public osgEarth::PagedNode
	{
	public:
		PagedOctreeNode(PointTileStore* tileStore, OctreeNode* octree, float rangeFactor) :
			_tileStore(tileStore),
			_octree(octree)
		{
			setRangeFactor(rangeFactor);
			build();
			setupPaging();
		}

		virtual osg::Node* loadChildren()
		{
			osg::Group* group = new osg::Group;
			for (unsigned int i = 0; i < 8; ++i)
			{
				osg::ref_ptr< OctreeNode > child = _octree->createChild(i);

				// TODO:  Do an exact key match thing instead of reading all the points.
				PointList pts;
				if (_tileStore->get(child->getID(), pts))
				{
					group->addChild(new PagedOctreeNode(_tileStore.get(), child, getRangeFactor()));
				}
			}
			return group;
		}

		virtual void build()
		{
			PointList points;
			_tileStore->get(_octree->getID(), points);
			if (points.size() > 0)
			{
				osg::Node* node = new PointCloud(points);
				if (node)
				{
					_attachPoint->addChild(node);
				}
			}			
		}


		virtual osg::BoundingSphere getKeyBound() const
		{
			return _octree->getBoundingBox();
		}

		virtual bool hasChildren() const
		{		
			for (unsigned int i = 0; i < 8; ++i)
			{
				osg::ref_ptr< OctreeNode > child = _octree->createChild(i);
				PointList pts;
				if (_tileStore->get(child->getID(), pts))
				{
					if (pts.size() > 0) return true;
				}
			}
			return false;			
		}

		osg::ref_ptr< OctreeNode > _octree;
		osg::ref_ptr< PointTileStore > _tileStore;
	};


	virtual ReadResult readNode(const std::string& location, const osgDB::ReaderWriter::Options* options) const
	{
		if (!acceptsExtension(osgDB::getLowerCaseFileExtension(location)))
			return ReadResult::FILE_NOT_HANDLED;

		// Read the metadata file produced by the splitter
		double minX, minY, minZ, maxX, maxY, maxZ;
		std::ifstream in("metadata.txt");
		in >> minX >> minY >> minZ >> maxX >> maxY >> maxZ;

		osg::ref_ptr< OctreeNode > root = new OctreeNode();
		root->setBoundingBox(osg::BoundingBoxd(minX, minY, minZ, maxX, maxY, maxZ));

		std::string file = osgDB::getNameLessExtension(location);

		double radiusFactor = 5.0;		

		//osg::ref_ptr< PointTileStore > tileStore = new RocksDBPointTileStore("tiled");
		osg::ref_ptr< PointTileStore > tileStore = new FilePointTileStore(".");

		PointCloudDecorator* decorator = new PointCloudDecorator;
		decorator->addChild(new PagedOctreeNode(tileStore.get(), root, radiusFactor));
		return decorator;
	}
};

REGISTER_OSGPLUGIN(lastile, LASTileReaderWriter)

