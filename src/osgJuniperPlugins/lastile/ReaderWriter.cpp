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
#include <osgEarth/PagedNode>
#include <osg/ShapeDrawable>
#include <osgText/Text>
#include <osgEarth/ShaderGenerator>
#include <osgEarth/Registry>

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

    static osg::Node* makeBox(const osg::Vec3& size, const osg::Vec4& color)
    {
        osg::Geometry* geom = new osg::Geometry;
        osg::Vec3Array* verts = new osg::Vec3Array;
        geom->setVertexArray(verts);

        osg::Vec4Array* colors = new osg::Vec4Array;
        colors->push_back(color);
        colors->push_back(color);
        colors->push_back(color);
        colors->push_back(color);
        colors->push_back(color);
        colors->push_back(color);
        colors->push_back(color);
        colors->push_back(color);
        geom->setColorArray(colors, osg::Array::BIND_PER_VERTEX);

        float x = size.x()/2.0f;
        float y = size.y()/2.0f;
        float z = size.z()/2.0f;

        verts->push_back(osg::Vec3(-x, -y, -z)); //0
        verts->push_back(osg::Vec3(x, -y, -z));  //1
        verts->push_back(osg::Vec3(-x, -y, z));  //2
        verts->push_back(osg::Vec3(x, -y, z));   //3
        
        verts->push_back(osg::Vec3(-x, y, -z));  //4
        verts->push_back(osg::Vec3(x, y, -z));   //5
        verts->push_back(osg::Vec3(-x, y, z));   //6
        verts->push_back(osg::Vec3(x, y, z));    //7

        osg::DrawElementsUByte* de = new osg::DrawElementsUByte(GL_LINES);
        de->push_back(0); de->push_back(1);
        de->push_back(1); de->push_back(3);
        de->push_back(0); de->push_back(2);
        de->push_back(2); de->push_back(3);

        de->push_back(4); de->push_back(5);
        de->push_back(6); de->push_back(7);
        de->push_back(4); de->push_back(6);
        de->push_back(5); de->push_back(7);

        de->push_back(0); de->push_back(4);
        de->push_back(1); de->push_back(5);
        de->push_back(2); de->push_back(6);
        de->push_back(3); de->push_back(7);

        geom->addPrimitiveSet(de);

        osgEarth::Registry::shaderGenerator().run(geom);

        
        return geom;

    }

	class PagedOctreeNode : public osgEarth::PagedNode
	{
	public:
		PagedOctreeNode(PointTileStore* tileStore, OctreeNode* octree, float rangeFactor, bool additive) :
			_tileStore(tileStore),
			_octree(octree)
		{
			setRangeFactor(rangeFactor);
		    setAdditive(additive),
			build();
			setupPaging();
		}

		virtual osg::Node* loadChild()
		{
			osg::Group* group = new osg::Group;
			for (unsigned int i = 0; i < 8; ++i)
			{
				osg::ref_ptr< OctreeNode > child = _octree->createChild(i);
				if (_tileStore->hasKey(child->getID()))
				{
					group->addChild(new PagedOctreeNode(_tileStore.get(), child, getRangeFactor(), getAdditive()));
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

                osg::BoundingBoxd bounds = _octree->getBoundingBox();

                osgText::Text* text = new osgText::Text;
                text->setAutoRotateToScreen(true);
                text->setCharacterSize(12);
                std::stringstream buf;
                buf << _octree->getID().level << ": (" << _octree->getID().x << ", " << _octree->getID().y << ", " << _octree->getID().z << ")";
                text->setText(buf.str());
                text->setCharacterSizeMode(osgText::Text::SCREEN_COORDS);
                osg::MatrixTransform* mt = new osg::MatrixTransform;
                mt->setMatrix(osg::Matrixd::translate(bounds.center()));                
                mt->addChild(text);
                
                /*
                osg::Geode* geode = new osg::Geode;
                osg::ShapeDrawable* sd = new osg::ShapeDrawable(new osg::Box(osg::Vec3f(0, 0, 0), bounds.radius(), bounds.radius(), bounds.radius()));
                sd->setColor(osg::Vec4(1.0, 0.0, 0.0, 1.0));
                geode->addDrawable(sd);
                mt->addChild(geode);
                */
                osg::Vec4 color(_octree->getID().level, 0.0, 0.0, 1.0);
                mt->addChild(makeBox(osg::Vec3(bounds.xMax() - bounds.xMin(), bounds.yMax() - bounds.yMin(), bounds.zMax() - bounds.zMin()), color));
                _attachPoint->addChild(mt);
			}
		}


		virtual osg::BoundingSphere getChildBound() const
		{
			osg::BoundingSphere bs;
			bs.expandBy(_octree->getBoundingBox());
			return bs;
		}

		virtual bool hasChild() const
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

		TilesetInfo info = TilesetInfo::read(location);

		osg::ref_ptr< OctreeNode > root = new OctreeNode();
		root->setBoundingBox(info.getBounds());

		std::string file = osgDB::getNameLessExtension(location);

		double radiusFactor = 2.0;		

		osg::ref_ptr< PointTileStore > tileStore = PointTileStore::create(info);
		if (!tileStore.valid())
		{
			OSG_NOTICE << "Failed to read tilestore from " << location << std::endl;
			return ReadResult::ERROR_IN_READING_FILE;
		}

		PointCloudDecorator* decorator = new PointCloudDecorator;
		decorator->addChild(new PagedOctreeNode(tileStore.get(), root, radiusFactor, info.getAdditive()));
		return decorator;
	}
};

REGISTER_OSGPLUGIN(lastile, LASTileReaderWriter)

