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
#include <osgDB/FileNameUtils>
#include <osgDB/FileUtils>
#include <osgDB/Registry>
#include <osgDB/ReaderWriter>
#include <osg/io_utils>
#include <osg/MatrixTransform>

#include <fstream>
#include <sstream>

#include <pdal/StageFactory.hpp>
#include <pdal/PointTable.hpp>
#include <pdal/PipelineExecutor.hpp>

#include <osgJuniper/PDALUtils>

using namespace pdal;
using namespace osgJuniper;

namespace
{
	static OpenThreads::Mutex pdalMutex;
}

#define PDAL_LOCK OpenThreads::ScopedLock< OpenThreads::Mutex > lock(pdalMutex)

osg::Node* makeNode(PointViewPtr view, const osgDB::ReaderWriter::Options* options)
{   
	OSG_NOTICE << "Making node with " << view->size() << " points " << std::endl;
    osg::Vec3d anchor;
    bool first = true;

    unsigned int NUM_POINTS_PER_GEOMETRY = 50000;

    osg::Geode* geode = new osg::Geode;

    osg::Geometry* geometry = 0;
    osg::Vec3Array* verts = 0;
    osg::Vec4ubArray* colors = 0;
    osg::Vec4Array* dataArray = 0;	

    for (unsigned int i = 0; i < view->size(); i++)
    {                 
		PointRef point(view->point(i));

        // Initialize the geometry if needed.
        if (!geometry)
        {
            geometry = new osg::Geometry;
            geometry->setUseVertexBufferObjects( true );
            geometry->setUseDisplayList( false );

            verts = new osg::Vec3Array();
            geometry->setVertexArray( verts );    

            colors =new osg::Vec4ubArray();    
            geometry->setColorArray(colors);
            geometry->setColorBinding(osg::Geometry::BIND_PER_VERTEX);

            dataArray = new osg::Vec4Array();
            geometry->setVertexAttribArray(osg::Drawable::ATTRIBUTE_6, dataArray);
            geometry->setVertexAttribBinding(osg::Drawable::ATTRIBUTE_6, osg::Geometry::BIND_PER_VERTEX);
            geometry->setVertexAttribNormalize(osg::Drawable::ATTRIBUTE_6, false);    

        }

		osg::Vec3d location = osg::Vec3d(point.getFieldAs<double>(pdal::Dimension::Id::X), point.getFieldAs<double>(pdal::Dimension::Id::Y), point.getFieldAs<double>(pdal::Dimension::Id::Z));

        osg::Vec4 data;
		data.x() = point.hasDim(pdal::Dimension::Id::Classification) ? point.getFieldAs<int>(pdal::Dimension::Id::Classification) : 0;
		data.y() = point.hasDim(pdal::Dimension::Id::ReturnNumber) ? point.getFieldAs<int>(pdal::Dimension::Id::ReturnNumber) : 0;
		data.z() = point.hasDim(pdal::Dimension::Id::Intensity) ? point.getFieldAs<int>(pdal::Dimension::Id::Intensity) : 0;

        dataArray->push_back(data);

        if (first)
        {
            anchor = location;
            first = false;
        }
        osg::Vec3 position = location - anchor;
        verts->push_back(position);

        osg::Vec4ub color = osg::Vec4ub(0, 0, 0, 255);
		if (point.hasDim(pdal::Dimension::Id::Red))
		{
			color.r() = point.getFieldAs<int>(pdal::Dimension::Id::Red) / 256;
		}

		if (point.hasDim(pdal::Dimension::Id::Green))
		{
			color.g() = point.getFieldAs<int>(pdal::Dimension::Id::Green) / 256;
		}

		if (point.hasDim(pdal::Dimension::Id::Blue))
		{
			color.b() = point.getFieldAs<int>(pdal::Dimension::Id::Blue) / 256;
		}

		if (point.hasDim(pdal::Dimension::Id::Alpha))
		{
			color.a() = point.getFieldAs<int>(pdal::Dimension::Id::Alpha) / 256;
		}

        colors->push_back(color);        

        if (verts->size() == NUM_POINTS_PER_GEOMETRY)
        {
            geode->addDrawable( geometry );
            geometry->addPrimitiveSet( new osg::DrawArrays(GL_POINTS, 0, verts->size()) );
            geometry = 0;
        }
    }        

    // Add a final geometry if necessary
    if (geometry)
    {
        geode->addDrawable( geometry );
        geometry->addPrimitiveSet( new osg::DrawArrays(GL_POINTS, 0, verts->size()) );
        geometry = 0;
    }

    osg::MatrixTransform* mt = new osg::MatrixTransform;
    mt->setMatrix(osg::Matrixd::translate(anchor));
    mt->addChild(geode);
    return mt;    
}

class PDALReaderWriter : public osgDB::ReaderWriter
{
public:
	PDALReaderWriter()
    {
		supportsExtension( "pdal", className());
		supportsExtension( "las", className());
		supportsExtension( "laz", className());
		supportsExtension( "json", className());
    }

    virtual const char* className()
    {
        return "PDAL Point Reader";
    }

	virtual ReadResult readNode( const std::string& location, const osgDB::ReaderWriter::Options* options ) const
    {
        if ( !acceptsExtension( osgDB::getLowerCaseFileExtension( location ) ) )
            return ReadResult::FILE_NOT_HANDLED;   

		std::string filename = location;
		if (osgDB::getFileExtension(location) == "pdal")
		{
			filename = osgDB::getNameLessExtension(location);
		}

		StageFactory factory;
		std::string driver = options ? options->getPluginStringData("pdal.driver") : "";
		if (driver.empty())
		{
			driver = PDALUtils::inferReaderDriver(filename);
		}		

		if (driver.empty() && osgDB::getFileExtension(filename) == "json")
		{
			driver = "pipeline";
		}
		
		if (driver.empty())
		{
			return ReadResult::FILE_NOT_HANDLED;
		}

		Stage* stage = 0;

		PipelineManager pipeline;
		
		if (driver == "pipeline")
		{			
			pipeline.readPipeline(filename);			
			stage = pipeline.getStage();
		}
		else
		{
			PDAL_LOCK;
			stage = factory.createStage(driver);
			pdal::Options opt;
			opt.add("filename", filename);
			stage->setOptions(opt);
		}

		if (stage)
		{			
			pdal::PointTable table;
			{ PDAL_LOCK;  stage->prepare(table); }

			pdal::PointViewSet point_view_set = stage->execute(table);
			pdal::PointViewPtr point_view = *point_view_set.begin();

			return makeNode(point_view, options);
		}
		return ReadResult::ERROR_IN_READING_FILE;
    }
};

REGISTER_OSGPLUGIN(pdal, PDALReaderWriter)

