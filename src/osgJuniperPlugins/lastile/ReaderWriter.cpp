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
#include <osgDB/FileNameUtils>
#include <osgDB/Registry>
#include <osgDB/ReaderWriter>
#include <osgDB/FileUtils>
#include <osgDB/ReadFile>
#include <osgJuniper/PointCloud>
#include <osgJuniper/Octree>
#include <osgEarth/PagedNode>

using namespace osgJuniper;

std::string getFilename(OctreeId id, const std::string& ext)
{
	std::stringstream buf;
	buf << "tile_" << id.level << "_" << id.z << "_" << id.x << "_" << id.y << "." << ext;
	return buf.str();
}


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
		PagedOctreeNode(const std::string& filename, OctreeNode* octree, float rangeFactor) :
			_filename(filename),
			_octree(octree)
		{
			setRangeFactor(rangeFactor);
			build();
			setupPaging();
		}

		virtual osg::Node* loadChildren()
		{
			osg::Group* group = new osg::Group;
			std::string path = osgDB::getFilePath(_filename);
			for (unsigned int i = 0; i < 8; ++i)
			{
				osg::ref_ptr< OctreeNode > child = _octree->createChild(i);
				std::string childFilename = osgDB::concatPaths(path, getFilename(child->getID(), "laz"));
				if (osgDB::fileExists(childFilename))
				{
					group->addChild(new PagedOctreeNode(childFilename, child, getRangeFactor()));
				}
			}
			return group;
		}

		virtual void build()
		{
			std::string filename = getFilename(_octree->getID(), "laz");
			osg::Node* node = osgDB::readNodeFile(filename);
			if (node)
			{
				_attachPoint->addChild(node);
			}
		}


		virtual osg::BoundingSphere getKeyBound() const
		{
			return _octree->getBoundingBox();
		}

		virtual bool hasChildren() const
		{
			std::string path = osgDB::getFilePath(_filename);
			bool hasChildren = false;
			for (unsigned int i = 0; i < 8; ++i)
			{
				osg::ref_ptr< OctreeNode > child = _octree->createChild(i);
				std::string childFilename = osgDB::concatPaths(path, getFilename(child->getID(), "laz"));
				if (osgDB::fileExists(childFilename))
				{
					return true;
				}
			}
			return false;
		}

		osg::ref_ptr< OctreeNode > _octree;
		std::string _filename;
	};


	virtual ReadResult readNode(const std::string& location, const osgDB::ReaderWriter::Options* options) const
	{
		if (!acceptsExtension(osgDB::getLowerCaseFileExtension(location)))
			return ReadResult::FILE_NOT_HANDLED;

		// Read the metadata file produced by the splitter
		double minX, minY, minZ, maxX, maxY, maxZ;
		std::ifstream in("metadata.txt");
		in >> minX >> minY >> minZ >> maxX >> maxY >> maxZ;
		std::cout << "Bounds " << minX << " " << minY << " " << minZ << " to "
			<< maxX << " " << maxY << " " << maxZ << std::endl;

		osg::ref_ptr< OctreeNode > root = new OctreeNode();
		root->setBoundingBox(osg::BoundingBoxd(minX, minY, minZ, maxX, maxY, maxZ));

		std::string file = osgDB::getNameLessExtension(location);

		double radiusFactor = 5.0;

		std::string inExt = osgDB::getFileExtension(file);

		std::string ext = "laz";
		if (options && !options->getOptionString().empty())
		{
			std::string radiusFactorStr = options->getPluginStringData("radiusFactor");
			if (!radiusFactorStr.empty())
			{
				std::istringstream iss(radiusFactorStr);
				iss >> radiusFactor;
			}

			std::string outExt = options->getPluginStringData("ext");
			if (!outExt.empty())
			{
				ext = outExt;
			}
		}


		return new PagedOctreeNode(file, root, radiusFactor);
		/*

		std::string file = osgDB::getNameLessExtension( location );
		std::string path = osgDB::getFilePath(file);

		// Get the octree tile name.
		std::string tileID = osgDB::getNameLessExtension(osgDB::getSimpleFileName(file));
		unsigned int level, x, y, z;
		sscanf(tileID.c_str(), "tile_%d_%d_%d_%d", &level, &z, &x, &y);

		// Create octree cell.
		OctreeId id(level, x, y, z);
		osg::ref_ptr< OctreeNode > octree = root->createChild(id);

		double radiusFactor = 2.5;

		std::string inExt = osgDB::getFileExtension(file);

		std::string ext = "laz";
		if (options && !options->getOptionString().empty())
		{
		std::string radiusFactorStr = options->getPluginStringData("radiusFactor");
		if (!radiusFactorStr.empty())
		{
		std::istringstream iss(radiusFactorStr);
		iss >> radiusFactor;
		}

		std::string outExt = options->getPluginStringData("ext");
		if (!outExt.empty())
		{
		ext = outExt;
		}
		}

		bool hasChildren = false;
		// Load all of the children for the octree
		osg::Group* group = new osg::Group;
		for (unsigned int i = 0; i < 8; ++i)
		{
		osg::ref_ptr< OctreeNode > child = octree->createChild(i);
		std::string childFilename = osgDB::concatPaths(path, getFilename(child->getID(), inExt));

		osg::Node* node = osgDB::readNodeFile(childFilename);
		if (node)
		{
		osg::PagedLOD* plod = new osg::PagedLOD;

		group->addChild(node);

		// Now just quickly see if this node has any children
		if (!hasChildren)
		{
		for (unsigned int j = 0; j < 8; ++j)
		{
		osg::ref_ptr< OctreeNode > child2 = octree->createChild(i);
		std::string childFilename2 = osgDB::concatPaths(path, getFilename(child2->getID(), inExt));
		if (osgDB::fileExists(childFilename2))
		{
		hasChildren = true;
		break;
		}
		}
		}
		}



		// Setup the min and max ranges.
		float minRange = (float)(octree->getBoundingBox().radius() * radiusFactor);

		// Replace mode, the parent is replaced by its children.


		if (hasChildren)
		{
		plod->setRange(0, minRange, FLT_MAX);
		plod->setRange(1, 0, minRange);
		}
		else
		{
		plod->setRange(0, 0, FLT_MAX);
		}




		//plod->setRange(1, 0, minRange);
		/*
		double childRadius = node->getBound().radius() / 2.0;
		for (unsigned int i = 0; i < 8; ++i)
		{
		osg::ref_ptr< OctreeNode > child = octree->createChild(i);
		std::string childFilename = osgDB::concatPaths(path, getFilename(child->getID(), inExt));

		if (osgDB::fileExists(childFilename) || osgDB::containsServerAddress(childFilename))
		{
		//std::string outFilename = osgDB::concatPaths(path, getFilename(child->getID(), ext));
		std::stringstream buf;
		buf << path;
		if (!path.empty())
		{
		buf << "/";
		}
		buf << getFilename(child->getID(), ext);
		std::string outFilename = buf.str();
		if (ext == "las" || ext == "laz")
		{
		outFilename += ".lastile";
		}
		plod->setFileName(childNum, outFilename);
		plod->setRange(childNum, 0, childRadius * radiusFactor);
		childNum++;
		}
		}

		// If this is the root node go ahead and add a PointCloud decorator to make it look nice.
		if (id.level == 0 && id.x == 0 && id.y == 0 && id.z == 0)
		{
		PointCloudDecorator *decorator = new PointCloudDecorator();
		decorator->addChild(plod);
		return decorator;
		}
		else
		{
		// Just return the PagedLOD.
		return plod;
		}
		*/
	}
};

REGISTER_OSGPLUGIN(lastile, LASTileReaderWriter)

