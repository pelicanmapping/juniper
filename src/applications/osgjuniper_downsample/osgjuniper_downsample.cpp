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
#include <osgJuniper/Octree>
#include <osgEarth/StringUtils>
#include <osgEarth/FileUtils>
#include <osg/BoundingBox>
#include <osgDB/FileUtils>
#include <osgDB/FileNameUtils>
#include <osg/ArgumentParser>
#include <iostream>
#include <osgJuniper/Utils>
#include <osgJuniper/PDALUtils>
#include <osgJuniper/PointReaderWriter>
#include <osgJuniper/RocksDBPointTileStore>

#include <osgJuniper/FilePointTileStore>
#include <osgJuniper/TilesetInfo>

using namespace osgJuniper;

class TileIndex;

class BuildCellOperator : public osg::Operation
{
public:
	BuildCellOperator(TileIndex *index, const OctreeId& id, unsigned int innerLevel) :
		_index(index),
		_id(id),
		_innerLevel(innerLevel)
	{
	}

	void operator()(osg::Object* object);

	OctreeId _id;
	TileIndex* _index;
	unsigned int _innerLevel;
};

class TileIndex
{	
public:
	TileIndex(OctreeNode* root, PointTileStore* tileStore):
		_root(root),
		_tileStore(tileStore),
		_targetNumPoints(50000)
	{
	}

	unsigned int getTargetNumPoints() const
	{
		return _targetNumPoints;
	}

	void setTargetNumPoints(unsigned int targetNumPoints)
	{
		_targetNumPoints = targetNumPoints;
	}

	void scan(unsigned int &minLevel, unsigned int &maxLevel)
	{
		_tiles.clear();

		// Get the keys in the tile store
		std::set< OctreeId > ids;
		_tileStore->queryKeys(KeyQuery(), ids);

		minLevel = UINT_MAX;
		maxLevel = 0;

		// Mark them all as being the highest resolution
		for (std::set< OctreeId >::iterator itr = ids.begin(); itr != ids.end(); ++itr)
		{
			_tiles[*itr] = true;
			if (itr->level < minLevel) minLevel = itr->level;
			if (itr->level > maxLevel) maxLevel = itr->level;
		}
		OSG_NOTICE << "Found " << _tiles.size() << std::endl;
	}

	void buildParents(unsigned int level, unsigned int innerLevel, unsigned int numThreads)
	{
		std::set< OctreeId > ids;
		{
			OpenThreads::ScopedLock< OpenThreads::Mutex > lock(_tilesMutex);
			for (auto itr = _tiles.begin(); itr != _tiles.end(); ++itr)
			{
				if (itr->first.level == level)
				{
					OctreeId parent = OctreeNode::getParentID(itr->first);
					ids.insert(parent);
				}
			}
		}

		OSG_NOTICE << "Building " << ids.size() << " for level " << level << std::endl;

		osg::ref_ptr< osg::OperationQueue > queue = new osg::OperationQueue;
		std::vector< osg::ref_ptr< osg::OperationsThread > > threads;
		for (unsigned int i = 0; i < numThreads; i++)
		{
			osg::OperationsThread* thread = new osg::OperationsThread();
			thread->setOperationQueue(queue.get());
			thread->start();
			threads.push_back(thread);
		}

		// Add all the operations
		for (auto itr = ids.begin(); itr != ids.end(); ++itr)
		{
			osg::ref_ptr< BuildCellOperator> buildCell = new BuildCellOperator(this, *itr, innerLevel);
			queue->add(buildCell.get());
		}

		// Wait for all operations to be done.
		while (!queue->empty())
		{
			OpenThreads::Thread::YieldCurrentThread();
		}

		for (unsigned int i = 0; i < numThreads; i++)
		{
			threads[i]->setDone(true);
			threads[i]->join();
		}
		OSG_NOTICE << "Done building level " << level << std::endl;
	}

	void build(const OctreeId &id, unsigned int innerLevel)
	{
		OSG_NOTICE << std::endl << "Building tile: " << id.level << "/" << id.z << "/" << id.x << "/" << id.y << std::endl;

		osg::ref_ptr< OctreeNode > node = _root->createChild(id);
		node->split();

		// Read all the children points		
		// TODO:  This assumes that you can call get on the same point list mulitple times.
		PointList points;
		bool highestLevel = true;
		for (unsigned int i = 0; i < node->getChildren().size(); i++)
		{
			_tileStore->get(node->getChildren()[i]->getID(), points);
			OpenThreads::ScopedLock< OpenThreads::Mutex > lock(_tilesMutex);
			std::map<OctreeId, bool>::iterator itr = _tiles.find(node->getChildren()[i]->getID());
			if (itr != _tiles.end())
			{
				if (itr->second == false)
				{
					highestLevel = false;
				}
			}
		}

		if (!points.empty())
		{			
			// If we are at the highest level and the children don't contain at least a minimum number of points
			// then simplify take all of the points and promote them up instead of sampling.		
			if (points.size() <= _targetNumPoints && highestLevel)
			{
				OSG_NOTICE << "Taking all " << points.size() << " points for " << id.level << "/" << id.z << "/" << id.x << "/" << id.y << std::endl;
				// Write all of the points to this cell
				_tileStore->set(id, points, false);						
				{
					OpenThreads::ScopedLock< OpenThreads::Mutex > lock(_tilesMutex);
					// Insert this cell into the index.
					_tiles[id] = true;

					// Remove all of the child tiles from the tile store and the tiles index.
					for (unsigned int i = 0; i < node->getChildren().size(); i++)
					{
						const OctreeId childId = node->getChildren()[i]->getID();
						std::map<OctreeId,bool>::iterator itr = _tiles.find(childId);
						if (itr != _tiles.end())
						{
							_tiles.erase(itr);
						}
						_tileStore->remove(childId);							
					}
				}
			}
			else
			{
				PointList keepers;

				std::set< OctreeId > innerCells;

				for (PointList::iterator itr = points.begin(); itr != points.end(); ++itr)
				{
					OctreeId cell = node->getID(osg::Vec3d(itr->x, itr->y, itr->z), innerLevel);
					if (innerCells.find(cell) == innerCells.end())
					{
						keepers.push_back(*itr);
						innerCells.insert(cell);
					}
				}

				if (!keepers.empty())
				{
					_tileStore->set(id, keepers, false);
					OpenThreads::ScopedLock< OpenThreads::Mutex > lock(_tilesMutex);
					_tiles[id] = false;
				}
				else
				{
					OSG_NOTICE << "No keepers for key " << std::endl;
				}
			}
		}		
	}

	OpenThreads::Mutex _tilesMutex;
	std::map< OctreeId, bool > _tiles;
	osg::ref_ptr< OctreeNode > _root;
	osg::ref_ptr< PointTileStore > _tileStore;
	unsigned int _targetNumPoints;
};


void BuildCellOperator::operator()(osg::Object* object)
{
	_index->build(_id, _innerLevel);
}

int main(int argc, char** argv)
{
    osg::Timer_t startTime = osg::Timer::instance()->tick();
    std::vector< std::string > filenames;

    osg::ArgumentParser arguments(&argc,argv);	

	int innerLevel = 6;
	arguments.read("--innerLevel", innerLevel);


	// Initialize the threads
	unsigned int numThreads = OpenThreads::GetNumberOfProcessors();
	arguments.read("--threads", numThreads);
	OSG_NOTICE << "Num threads " << numThreads << std::endl;

	unsigned int target = 50000;
	arguments.read("--target", target);

	std::string infoFilename;

	for (int pos = 1; pos<arguments.argc(); ++pos)
	{
		if (!arguments.isOption(pos))
		{
			infoFilename = arguments[pos];
			break;
		}
	}

	if (infoFilename.empty())
	{
		OSG_NOTICE << "Please specify a tileset.lastile file";
		return -1;
	}

	TilesetInfo info = TilesetInfo::read(infoFilename);

	osg::ref_ptr< OctreeNode > root = new OctreeNode();
	root->setBoundingBox(info.getBounds());	

	osg::ref_ptr< PointTileStore > tileStore = PointTileStore::create(info);	
	if (!tileStore.valid())
	{
		OSG_NOTICE << "Failed to create tilestore " << info.getDriver() << std::endl;
		return -1;
	}

	TileIndex index(root, tileStore.get());
	index.setTargetNumPoints(target);
	unsigned int minLevel, maxLevel;
	index.scan(minLevel, maxLevel);

	for (int i = maxLevel; i > 0; i--)
	{
		OSG_NOTICE << "Building parents for " << i << std::endl;
		index.buildParents(i, innerLevel, numThreads);
	}

	osg::Timer_t endTime = osg::Timer::instance()->tick();

    OSG_NOTICE << "Completed in " << osgEarth::prettyPrintTime(osg::Timer::instance()->delta_s(startTime, endTime)) << std::endl;
}
