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

#include <osgJuniper/IBRCameraNode>
#include <osgJuniper/Utils>

#include <osg/Texture2D>
#include <osg/TexGen>
#include <osg/Geode>

#include <osgDB/ReadFile>
#include <osgDB/WriteFile>
#include <osgDB/FileUtils>
#include <osgDB/FilenameUtils>
#include <osg/Notify>
#include <osg/io_utils>

#include <osgUtil/SmoothingVisitor>
#include <osgEarth/NodeUtils>
#include <osgEarth/ImageUtils>

using namespace osgJuniper;



/*************************************************************************************/
IBRScene::IBRScene()
{
    _taskService = new osgEarth::TaskService();
}

/*************************************************************************************/
LoadImageOperation::LoadImageOperation(const std::string& filename):
_filename(filename)
{
}

std::string getCompressedName(const std::string& filename)
{
    std::string basename = osgDB::getNameLessExtension(filename);
    return basename + "_compressed.dds";
}
osg::Image* compress(osg::Image* image)
{
    /*
    osg::notify(osg::INFO) << "Generating compressed image" << std::endl;
    osg::Timer_t start = osg::Timer::instance()->tick();
    osg::Image* result = osgEarth::ImageUtils::compress( image, "" );
    osg::Timer_t end = osg::Timer::instance()->tick();
    osg::notify(osg::INFO) << "Generated compressed image in " << osg::Timer::instance()->delta_m(start, end) << std::endl;
    */
    return image;
    //return result;
}

void
LoadImageOperation::operator ()(osgEarth::ProgressCallback *progress)
{
    //Try to read the compressed file first
    std::string compressedName = getCompressedName( _filename );
    _image = osgDB::readImageFile( compressedName );
    if (!_image.valid())
    {
        //Try to read the regular image.
        _image = osgDB::readImageFile( _filename );        
    }

    //Try to compress the image on the fly if it's not already compressed
    if (_image.valid() && !osgEarth::ImageUtils::isCompressed( _image.get()))
    {
        _image = compress( _image.get() );
        //osg::notify(osg::NOTICE) << "juniper compressed image to " << _image->s() << "x" << _image->t() << std::endl;
    }
}

/*************************************************************************************/

struct GeometryFinder : public osg::NodeVisitor
{
    std::vector<osg::ref_ptr<osg::Geometry> > _geometries;
    GeometryFinder() : osg::NodeVisitor (osg::NodeVisitor::TRAVERSE_ALL_CHILDREN) {}
    void apply(osg::Geode& node) {
        for (int i = 0; i < (int)node.getNumDrawables(); ++i) {
            osg::Geometry* geom = dynamic_cast<osg::Geometry*>(node.getDrawable(i));
            _geometries.push_back(geom);
        }
    }
};

IBRCameraNode::IBRCameraNode():
_opacity(0.0f),
_showFrustum(true),
_frustumColor(1,1,1,1),
_scale(1.0f),
_material(new osg::Material),
_fullRes(false)
{
    _material->setDiffuse(osg::Material::FRONT_AND_BACK, osg::Vec4(1,1,1,1));
    getOrCreateStateSet()->setAttributeAndModes(_material.get());

    setNumChildrenRequiringUpdateTraversal( 1 );
}

const IBRCamera& IBRCameraNode::getCamera() const { return _camera;}

float IBRCameraNode::getOpacity() const
{
	return _opacity;
}

void IBRCameraNode::setOpacity(float opacity)
{
    if (_opacity != opacity)
    {		
        _opacity = opacity;
        _material->setTransparency(osg::Material::FRONT_AND_BACK, 1.0 - _opacity);
    }
}

bool IBRCameraNode::getShowFrustum() const
{
	return _showFrustum;
}

void IBRCameraNode::setShowFrustum(bool showFrustum)
{
	if (_showFrustum != showFrustum)
	{
		_showFrustum = showFrustum;
		if (_frustumGeode.valid())
		{
			_frustumGeode->setNodeMask( _showFrustum ? ~0x0 : 0x0);
		}
	}
}

const osg::Vec4f& IBRCameraNode::getFrustumColor() const
{
	return _frustumColor;
}

float IBRCameraNode::getScale() const
{
	return _scale;
}

void IBRCameraNode::setScale(float scale)
{
    if (_scale != scale)
    {
        _scale = scale;
        setMatrix( osg::Matrixd::scale(_scale, _scale, _scale) * _camera.getWorldMatrix() );
        if (_transform.valid())
            _transform->setMatrix(osg::Matrix::inverse(osg::Matrixd::scale(_scale, _scale, _scale) * _camera.getWorldMatrix()));
    }
}

void IBRCameraNode::setFrustumColor(const osg::Vec4& frustumColor)
{
	if (_frustumColor != frustumColor)
	{
		_frustumColor = frustumColor;

		if (_frustumGeometry.valid())
		{		
		  osg::Vec4Array* colors = static_cast<osg::Vec4Array*>(_frustumGeometry->getColorArray());
		  (*colors)[0] = _frustumColor;
		  _frustumGeometry->dirtyDisplayList();
		}
	}
}

void IBRCameraNode::setCamera(const IBRCamera& camera)
{
	_camera = camera;
	redraw();
}

/**
* Creates a y-up camera pointing down the -z axis, as per the Bunlder
* camera coordinate system.
*
*/
osg::Node* IBRCameraNode::createCameraDepthGeom( const IBRCamera& camera, int imgWidth, int imgHeight)
{
    osg::ref_ptr<osg::Node> depth = osgDB::readNodeFile(camera._depthModelUri);
    if (!depth.valid()) {
        osg::notify(osg::NOTICE) << "Failed to read " << camera._depthModelUri << std::endl;
        return 0;
    }
    //osg::notify(osg::NOTICE) << "Read depth file " << camera._depthModelUri << std::endl;

    osg::TexGen* tg = new osg::TexGen;
		double hfov2 = osg::RadiansToDegrees(atan2( 0.5*(double)imgWidth, camera._focalLength ));
		double vfov2 = osg::RadiansToDegrees(atan2( 0.5*(double)imgHeight, camera._focalLength ));
    depth->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
    depth->getOrCreateStateSet()->setMode(GL_BLEND, osg::StateAttribute::ON);
    depth->getOrCreateStateSet()->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);

    
    osg::Matrix projection;
    projection.makePerspective(hfov2 * 2.0, hfov2/vfov2, 0.01, 1000.0);
    osg::Matrix matrix = camera.getWorldMatrix();
    tg->setPlanesFromMatrix( osg::Matrix::inverse(matrix) * projection * osg::Matrix::translate(0.5,0.5,0));
    depth->getOrCreateStateSet()->setTextureAttributeAndModes(0, tg);
    return depth.release();
}


/**
* Creates a y-up camera pointing down the -z axis, as per the Bunlder
* camera coordinate system.
*
* If you supply a positive image width and height, it will warp the cone to
* reflect the true FOV. A longer frustum and smaller image plane implies a
* higher zoom level. 
*
* Since trhe camera frustum is notional (it continues on forever), you can
* supply a scale factor to control how large it should be in the scene.
* Alternatively, you could scale the parent geode with a matrix transform.
*/
osg::Geometry* IBRCameraNode::createCameraGeom( const IBRCamera& camera, int imgWidth, int imgHeight, float scale )
{
	double xs = 0.5, ys = 0.5; // square, 90deg fov by default

	if ( imgWidth > 0 && imgHeight > 0 )
	{
		// if possible, adjust the camera cone based on the image resolution and the
		// focal length. This gives us a true FOV cone and a visual clue as to the
		// zoom level.
		double hfov2 = atan2( 0.5*(double)imgWidth, camera._focalLength );
		double vfov2 = atan2( 0.5*(double)imgHeight, camera._focalLength );
		xs = tan(hfov2);
		ys = tan(vfov2);
	}

	osg::Geometry* geom = new osg::Geometry();

	osg::Vec3Array* v = new osg::Vec3Array( 4 );
	(*v)[0].set( -xs*scale, -ys*scale, -scale ); //ll
	(*v)[1].set( -xs*scale,  ys*scale, -scale ); //ul

	(*v)[2].set(  xs*scale,  ys*scale, -scale ); //ur
	(*v)[3].set(  xs*scale, -ys*scale, -scale ); //lr

	geom->setVertexArray( v );

	osg::Vec2Array* t = new osg::Vec2Array( 4 );
	(*t)[0].set( 0, 0 );
	(*t)[1].set( 0, 1 );
	(*t)[2].set( 1, 1 );
	(*t)[3].set( 1, 0 );
	geom->setTexCoordArray( 0, t );

	//GLuint face_i[6] = { 0, 1, 3, 1, 2, 3 };
	GLuint face_i[6] = { 0, 2, 1, 0, 3, 2 };
	geom->addPrimitiveSet( new osg::DrawElementsUInt( GL_TRIANGLES, 6, face_i ) );

	geom->getOrCreateStateSet()->setMode(GL_BLEND, osg::StateAttribute::ON);
	geom->getOrCreateStateSet()->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);

	return geom;
}

osg::Geometry* IBRCameraNode::createFrustumGeometry( const IBRCamera& camera, int imgWidth, int imgHeight, float scale )
{
	double xs = 0.5, ys = 0.5; // square, 90deg fov by default

	if ( imgWidth > 0 && imgHeight > 0 )
	{
		// if possible, adjust the camera cone based on the image resolution and the
		// focal length. This gives us a true FOV cone and a visual clue as to the
		// zoom level.
		double hfov2 = atan2( 0.5*(double)imgWidth, camera._focalLength );
		double vfov2 = atan2( 0.5*(double)imgHeight, camera._focalLength );
		xs = tan(hfov2);
		ys = tan(vfov2);
	}

	osg::Geometry* geom = new osg::Geometry();

	osg::Vec3Array* v = new osg::Vec3Array( 5 );
	(*v)[0].set( 0, 0, 0 );
	(*v)[1].set( -xs*scale, -ys*scale, -scale );
	(*v)[2].set( -xs*scale,  ys*scale, -scale );
	(*v)[3].set(  xs*scale,  ys*scale, -scale );
	(*v)[4].set(  xs*scale, -ys*scale, -scale );

	geom->setVertexArray( v );

	GLuint frust_i[8] = { 0, 1, 0, 2, 0, 3, 0, 4 };
	geom->addPrimitiveSet( new osg::DrawElementsUInt( GL_LINES, 8, frust_i ) );

	osg::Vec4Array* colors = new osg::Vec4Array(1);
	(*colors)[0] = _frustumColor;
	geom->setColorArray( colors );
	geom->setColorBinding( osg::Geometry::BIND_OVERALL );

	geom->getOrCreateStateSet()->setMode(GL_BLEND, osg::StateAttribute::ON);
	geom->getOrCreateStateSet()->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);

	return geom;
}

void IBRCameraNode::redraw()
{
    //Remove all the previous children
    removeChildren(0, getNumChildren());

    //Create the face geode
    _faceGeode = new osg::Geode;
    int iw=0, ih=0;
    // texture the geometry with thumbnail images

    osg::ref_ptr< osg::Texture2D > thumbnailTexture;
    if ( !_camera._imageUri.empty() )
    {
        osg::ref_ptr< osg::Image > image = osgDB::readImageFile( _camera._imageUri );
        if ( image.valid() )
        {
            iw = image->s();
            ih = image->t();
            int w = osg::minimum(iw, 256);
            float hr = (float)iw/(float)ih;
            int h = int(w/hr);
            _thumbnail = Utils::resizeImage( image.get(), w, h );
            _thumbnail = compress( _thumbnail.get() );
        }

        //See if we need to create a compressed version of this image to increase the frame rate.
         std::string compressedName = getCompressedName( _camera._imageUri );
         if (!osgDB::fileExists( compressedName ) )
         {
             if (!osgEarth::ImageUtils::isCompressed( image ))
             {
                 osg::ref_ptr< osg::Image > compressedImage = compress( image );
                 if (compressedImage.valid())
                 {
                     osgDB::writeImageFile(*compressedImage.get(), compressedName);
                     osg::notify(osg::NOTICE) << "Wrote compressed image " << compressedName << std::endl;
                 }
             }
         }

        if (_thumbnail.valid())
        {
            thumbnailTexture = new osg::Texture2D();
            thumbnailTexture->setUnRefImageDataAfterApply( true );
            //Linear filtering to improve performance
            thumbnailTexture->setFilter( osg::Texture::MIN_FILTER, osg::Texture::LINEAR );
            thumbnailTexture->setFilter( osg::Texture::MAG_FILTER, osg::Texture::LINEAR );
            thumbnailTexture->setResizeNonPowerOfTwoHint(false);
            thumbnailTexture->setImage( _thumbnail.get() );            
            _faceGeode->getOrCreateStateSet()->setTextureAttributeAndModes( 0, thumbnailTexture );
            _fullRes = false;
        }
        else
        {
            osg::notify(osg::NOTICE) << "Couldn't read image " << _camera._imageUri << std::endl;
        }
    }

    _faceGeometry = createCameraGeom( _camera, iw, ih);
    _faceGeode->addDrawable( _faceGeometry.get());

    if (!_camera._depthModelUri.empty()) {
        _transform = new osg::MatrixTransform;
        _transform->setMatrix(osg::Matrix::inverse(osg::Matrixd::scale(_scale, _scale, _scale) * _camera.getWorldMatrix()));
        addChild(_transform.get());        
        _depthModel = createCameraDepthGeom( _camera, iw,ih);
        _transform->addChild(_depthModel.get());        
        //Set the depth model to have the thumbnail textures
        if (thumbnailTexture.valid())
        {
        _depthModel->getOrCreateStateSet()->setTextureAttributeAndModes( 0, thumbnailTexture.get() );
        }
    } else {
        addChild(_faceGeode.get());
    }

    //Create the frustum geode
    _frustumGeode = new osg::Geode;
    _frustumGeometry = createFrustumGeometry( _camera, iw, ih);
    _frustumGeode->addDrawable(_frustumGeometry.get());
    _frustumGeode->setNodeMask( _showFrustum ? ~0x0 : 0x0);
    addChild(_frustumGeode.get());

    // transform it to world coords
    setMatrix( osg::Matrixd::scale(_scale, _scale, _scale) * _camera.getWorldMatrix() );
    setName("Camera");
    setCullingActive(false);

    // encode the ID
    std::stringstream buf;
    buf << _camera._id;
    std::string bufStr = buf.str();
    addDescription( bufStr );

    getOrCreateStateSet()->setMode(GL_DEPTH_TEST, false);
}


void
IBRCameraNode::traverse(osg::NodeVisitor &nv)
{
    //Try to find the parent IBRScene if it hasn't been set yet.
    if (!_ibrScene.valid())
    {
        _ibrScene = osgEarth::findFirstParentOfType< IBRScene >(this);
        /*
        if (_ibrScene.valid())
        {
            osg::notify(osg::NOTICE) << "Found parent IBR scene..." << std::endl;
        }
        */
    }

    if (nv.getVisitorType() == osg::NodeVisitor::UPDATE_VISITOR)
    {    
        if (_opacity > 0)
        {
            if (!_image.valid() && _loadImageOperation.valid() && _loadImageOperation->isCompleted())
            {
                _image = _loadImageOperation->_image.get();
                _loadImageOperation = NULL;
                osg::Texture2D* texture = static_cast<osg::Texture2D*>(_faceGeode->getStateSet()->getTextureAttribute(0, osg::StateAttribute::TEXTURE));
                if (texture)
                {
                    //osg::notify(osg::NOTICE) << "Setting high resolution image" << _camera._imageUri  << " " << _image->s() << "x" << _image->t() << " " << osgEarth::ImageUtils::isCompressed(_image.get()) << std::endl;
                    texture->setInternalFormatMode(osg::Texture::USE_IMAGE_DATA_FORMAT);
                    texture->setResizeNonPowerOfTwoHint( false );
                    texture->setImage(_image.get());                    
                    _fullRes = true;
                }
            }
        }
        else if (_opacity <= 0)
        {
            if (_image.valid() && _fullRes)
            {
                _image = 0;
                osg::Texture2D* texture = static_cast<osg::Texture2D*>(_faceGeode->getStateSet()->getTextureAttribute(0, osg::StateAttribute::TEXTURE));
                if (texture)
                {
                    //osg::notify(osg::INFO) << "Setting thumbnail resolution image" << _camera._imageUri  << std::endl;
                    texture->setImage(_thumbnail.get());
                    _fullRes = false;
                }
            }
            else if (!_image.valid() && _loadImageOperation.valid())
            {
                if (!_loadImageOperation->isCompleted())
                {
                    //Cancel the loadImageOperation if we aren't valid and it's not yet complete
                    _loadImageOperation->cancel();
                }
                //The load image operation has completed but the opacity has went down to 0, meaning we're not visible.  We need
                //to delete this operation so the result image doesn't just sit around and take up memory waiting to be merged when it's possible
                //that we wont' view this image for a long time.
                _loadImageOperation = 0;
            }
        }
    }
    else if (nv.getVisitorType() == osg::NodeVisitor::CULL_VISITOR)
    {
        if (_opacity > 0)
        {
            if (!_image.valid() && !_loadImageOperation.valid())
            {
                if (_ibrScene.valid() && _ibrScene->getTaskService())
                {
                    osg::notify(osg::INFO) << "Adding op " << _camera._imageUri << " to load thread" << std::endl;
                    _loadImageOperation = new LoadImageOperation(  _camera._imageUri );
                    //getOperationThread()->add( _loadImageOperation.get() );
                    _ibrScene->getTaskService()->add( _loadImageOperation.get() );
                }
            }
        }
        else
        {
            //Don't traverse if the opacity is <= 0 to increase the framerate
            return;
        }
    }
    osg::MatrixTransform::traverse( nv );
}

