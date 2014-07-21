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

#include <osgJuniper/IBRManipulator>
#include <osgViewer/View>
#include <osgDB/ReadFile>
#include <osg/Math>
#include <osg/Texture2D>
#include <osg/Geometry>

using namespace osgJuniper;

IBRManipulator::IBRManipulator() : osgGA::MatrixManipulator()
{
    _matrix = osg::Matrix::identity();
    _currentPosition = 0;
    _offset = 0.0;
    _matrix = osg::Matrix::identity();
    _scrollDelta = 0.2;
    _minOpacity=0.0f;
    _maxOpacity=1.0f;

    createHUD();
    _zoomState = ZOOM_UNACTIVE;

    _zoomMotionIn = osgAnimation::InCubicMotion(0, 0.3, 1);
    _zoomMotionOut = osgAnimation::InCubicMotion(0, 0.3, 1);

    _automaticMode = false;
    _previousTime = false;
}

void IBRManipulator::setCameraList( const CameraList& cameraList)
{
    _cameraList = cameraList;
    if (cameraList.empty())
        _matrix = osg::Matrix::identity();
    else
        _matrix = computeMatrix(0, 0, 0);
}


/** handle events, return true if handled, false otherwise.*/
bool IBRManipulator::handle(const osgGA::GUIEventAdapter& ea,osgGA::GUIActionAdapter& us)
{
    if (_cameraList.empty())
        return false;

    switch( ea.getEventType() )
    {
    case osgGA::GUIEventAdapter::FRAME:
    {
        if (_zoomState != ZOOM_UNACTIVE) {
            updateZoomMode(us);
        }
        if (_automaticMode) {
            _currentPosition += (ea.getTime() - _previousTime) * 0.007;
            calcCamera();
            us.requestRedraw();
        }

        _previousTime = ea.getTime();

        return false;
    }
    break;
    case osgGA::GUIEventAdapter::KEYDOWN:
        if (_zoomState != ZOOM_UNACTIVE)
            return false;
        if (ea.getKey()=='[')
        {
            if (_cameraList[getCameraIndex()]->getTransform()) {                
                osg::Matrix mtr = _cameraList[getCameraIndex()]->getTransform()->getMatrix();
                mtr = osg::Matrix::rotate(osg::PI_4/10.0, osg::Vec3(0,1,0)) * mtr;
                _cameraList[getCameraIndex()]->getTransform()->setMatrix(mtr);
            }
            us.requestRedraw();
            us.requestContinuousUpdate(false);
            return true;
        }
        else if (ea.getKey()==']')
        {
            if (_cameraList[getCameraIndex()]->getTransform()) {                
                osg::Matrix mtr = _cameraList[getCameraIndex()]->getTransform()->getMatrix();
                mtr = osg::Matrix::rotate(osg::PI_4/10.0, osg::Vec3(0,-1,0)) * mtr;
                _cameraList[getCameraIndex()]->getTransform()->setMatrix(mtr);
            }
            us.requestRedraw();
            us.requestContinuousUpdate(false);
            return true;
        }
        if (ea.getKey()=='\'')
        {
            if (_cameraList[getCameraIndex()]->getTransform()) {
                osg::Matrix mtr = _cameraList[getCameraIndex()]->getTransform()->getMatrix();
                mtr = osg::Matrix::scale(1.1, 1.1, 1.1) * mtr;
                _cameraList[getCameraIndex()]->getTransform()->setMatrix(mtr);
            }
            us.requestRedraw();
            us.requestContinuousUpdate(false);
            return true;
        }
        else if (ea.getKey()=='\\')
        {
            if (_cameraList[getCameraIndex()]->getTransform()) {
                osg::Matrix mtr = _cameraList[getCameraIndex()]->getTransform()->getMatrix();
                mtr = osg::Matrix::scale(0.9, .9, 0.9) * mtr;
                _cameraList[getCameraIndex()]->getTransform()->setMatrix(mtr);
            }
            us.requestRedraw();
            us.requestContinuousUpdate(false);
            return true;
        }
        if (ea.getKey()=='n' || ea.getKey()=='N')
        {
            setNextCamera();
            calcCamera();
            us.requestRedraw();
            us.requestContinuousUpdate(false);
            return true;
        }
        else if (ea.getKey()=='p' || ea.getKey() == 'P')
        {
            setPrevCamera();
            calcCamera();
            us.requestRedraw();
            us.requestContinuousUpdate(false);
            return true;
        }
        else if (ea.getKey() =='u' || ea.getKey() == 'U')
        {
            _offset -= 1.0;
            calcCamera();
            us.requestRedraw();
            us.requestContinuousUpdate(false);
            return true;
        }
        else if (ea.getKey() =='j' || ea.getKey() == 'J')
        {
            _offset += 1.0;
            calcCamera();
            us.requestRedraw();
            us.requestContinuousUpdate(false);
            return true;
        }
        else if (ea.getKey() == 'k' || ea.getKey() == 'K')
        {
            if (_automaticMode)
                _automaticMode = false;
            else
                _automaticMode = true;
            osg::notify(osg::INFO) << "IBRManipulator automatic mode " << _automaticMode << std::endl;
            return true;
        }
        break;
		case osgGA::GUIEventAdapter::SCROLL:
        if (_zoomState != ZOOM_UNACTIVE)
            return false;
			{
				double dy = 0.0;
				if (ea.getScrollingMotion() == osgGA::GUIEventAdapter::SCROLL_UP)
				{
					dy = -_scrollDelta;
				}
				else if (ea.getScrollingMotion() == osgGA::GUIEventAdapter::SCROLL_DOWN)
				{
					dy = _scrollDelta;
				}
				_offset += dy;
				return true;
			}
      break;
    case osgGA::GUIEventAdapter::PUSH:
        flushMouseEventStack();
        addMouseEvent(ea);
        handleFrame();
        us.requestRedraw();
        us.requestContinuousUpdate(false);
        return true;
        break;
    case osgGA::GUIEventAdapter::RELEASE:
        if (ea.getButton() == osgGA::GUIEventAdapter::RIGHT_MOUSE_BUTTON) {
            //osg::notify(osg::NOTICE) << "state before " << _zoomState << std::endl;
            if (_zoomState == ZOOM_IN) {
                _zoomState = ZOOM_OUT;
                _startTimeZoom = osg::Timer::instance()->tick();
            } else if (_zoomState == ZOOM_UNACTIVE) {
                enterZoomMode();
            }
            //osg::notify(osg::NOTICE) << "state after " << _zoomState << std::endl;
        }
        return true;
        break;
    case osgGA::GUIEventAdapter::DRAG:
        if (_zoomState != ZOOM_UNACTIVE)
            return false;
        addMouseEvent(ea);
        handleFrame();
        us.requestRedraw();
        us.requestContinuousUpdate(false);
        return true;
        break;
    default:
        break;
    }
    return false;
}



void IBRManipulator::enterZoomMode()
{
    int index = getCameraIndex();
    osgJuniper::IBRCameraNode* camera = _cameraList[index];

    osg::Image* image = osgDB::readImageFile(camera->getCamera()._imageUri);
    osg::Texture2D* texture = new osg::Texture2D;
    texture->setResizeNonPowerOfTwoHint(false);
    texture->setImage(image);

    float iw = image->s();
    float ih = image->t();

    osg::Geometry* geom = camera->createCameraGeom(camera->getCamera(), iw, ih);
    osg::Vec3Array* a = dynamic_cast<osg::Vec3Array*>(geom->getVertexArray());
    (*a)[0][2] = 0;
    (*a)[1][2] = 0;
    (*a)[2][2] = 0;
    (*a)[3][2] = 0;
    _geometry = geom; //createPicture(camera->getCamera()._imageUri);
    _geode->removeDrawables(0, _geode->getNumDrawables());
    _geode->addDrawable(_geometry.get());
    _geode->getOrCreateStateSet()->setTextureAttributeAndModes(0, texture);
    _geode->getOrCreateStateSet()->setMode(GL_LIGHTING, false);
    _startTimeZoom = osg::Timer::instance()->tick();
    _geode->setNodeMask(~0x0);
    _zoomState = ZOOM_IN;

    _restoreAlpha.clear();
    for (int i = 0; i < (int)_cameraList.size(); ++i) {
        _restoreAlpha.push_back(_cameraList[i]->getOpacity());
    }
}

void IBRManipulator::updateZoomMode(osgGA::GUIActionAdapter& us)
{
    {
        osgViewer::View* view = dynamic_cast<osgViewer::View*>(us.asView());
        int width = view->getCamera()->getViewport()->width();
        int height = view->getCamera()->getViewport()->height();
        double r = width * 1.0 / (double)height;
        _camera->setProjectionMatrixAsOrtho(-0.5*r, 0.5*r, -0.5, 0.5, -1, 1);
    }

    double currentTime = osg::Timer::instance()->delta_s(_startTimeZoom, osg::Timer::instance()->tick());
    float ratio;
    if (_zoomState == ZOOM_IN) {
        ratio = _zoomMotionIn.getValueAt(currentTime);
        for (int i = 0; i < (int)_cameraList.size(); ++i) {
            _cameraList[i]->setOpacity(_restoreAlpha[i] * ( 1.0 - ratio));
        }

    } else if (_zoomState == ZOOM_OUT) {
        ratio = 1.0 - _zoomMotionOut.getValueAt(currentTime);
        for (int i = 0; i < (int)_cameraList.size(); ++i) {
            _cameraList[i]->setOpacity(_restoreAlpha[i] * (1.0 - ratio ));
        }

        if (ratio <= 0.0) {
            leaveZoomMode();
        }
    }

    //osg::notify(osg::NOTICE) << "ratio " << ratio << std::endl;
    _geometry->dirtyDisplayList();
}

void IBRManipulator::leaveZoomMode()
{
    //osg::notify(osg::NOTICE) << "leave zoom" << std::endl;
    _geode->setNodeMask(0x0);
    _zoomState = ZOOM_UNACTIVE;
    calcCamera();
}

void IBRManipulator::createHUD()
{
    osg::Camera* camera = new osg::Camera;
    _camera = camera;
    osg::Geode* geode = new osg::Geode;
    camera->addChild(geode);
    _geode = geode;
    _geode->getOrCreateStateSet()->setMode(GL_DEPTH_TEST, false);
    _geode->getOrCreateStateSet()->setMode(GL_CULL_FACE, false);
    camera->setClearMask(0);
    
    
    camera->setProjectionMatrixAsOrtho(-1, 1, -1, 1, -1, 1);
    camera->setReferenceFrame(osg::Transform::ABSOLUTE_RF);
    camera->setViewMatrix(osg::Matrix::identity());
    camera->setRenderOrder(osg::Camera::POST_RENDER);
    camera->setAllowEventFocus(false);
}

void IBRManipulator::flushMouseEventStack()
{
    _ga_t1 = NULL;
    _ga_t0 = NULL;
}

void IBRManipulator::addMouseEvent(const osgGA::GUIEventAdapter& ea)
{
    _ga_t1 = _ga_t0;
    _ga_t0 = &ea;
}

osg::Matrix IBRManipulator::computeMatrix(int in, int out, double ratio)
{
    /*osg::Vec3 pcur = _cameraList[in].get()->getMatrix().getTrans();
    osg::Vec3 pnext = _cameraList[out].get()->getMatrix().getTrans();

    osg::Quat qcur = _cameraList[in].get()->getMatrix().getRotate();
    osg::Quat qnext = _cameraList[out].get()->getMatrix().getRotate();
	*/

	osg::Matrixd localToWorldCur  = getLocalToWorld(_cameraList[in].get());
	osg::Matrixd localToWorldNext = getLocalToWorld(_cameraList[out].get());


	osg::Vec3d pcur  = localToWorldCur.getTrans();
	osg::Vec3d pnext = localToWorldNext.getTrans();

	removeScale(localToWorldCur);
	removeScale(localToWorldNext);

	osg::Quat qcur = localToWorldCur.getRotate();
	osg::Quat qnext = localToWorldNext.getRotate();

    osg::Vec3d position = pcur + (pnext - pcur) * ratio;
    osg::Quat rotate;
    rotate.slerp(ratio, qcur, qnext);

    osg::Matrixd matrix;
    matrix.setRotate(rotate);
    matrix.setTrans(position);
    return matrix;
}


bool IBRManipulator::handleFrame()
{
    float dx=0.0f;
    float dy=0.0f;

    if (_ga_t1.get()==NULL) return false;
    dx = _ga_t0->getXnormalized()-_ga_t1->getXnormalized();
    dy = _ga_t0->getYnormalized()-_ga_t1->getYnormalized();
    double distance = sqrt(dx*dx + dy*dy);
        
    // return if movement is too fast, indicating an error in event values or change in screen.
    if (distance>0.5)
    {
        return false;
    }


    double throwScale = 0.1;
    double sign = dx > 0 ? 1.0 : -1.0;
    _currentPosition += sign * throwScale * distance;

    calcCamera();

    return true;
}

int IBRManipulator::getCameraIndex()
{
    return static_cast<int>(floor(_currentPosition * _cameraList.size()));
}

void IBRManipulator::setNextCamera()
{
    int value = static_cast<int>(floor(1e-5 + _currentPosition * _cameraList.size()));
    _currentPosition = ((value + 1) % _cameraList.size()) * 1.0 / _cameraList.size();
}

void IBRManipulator::setPrevCamera()
{
    int value = static_cast<int>(ceil(-1e-5 + _currentPosition * _cameraList.size()));
    _currentPosition = ( (value + _cameraList.size() - 1) % _cameraList.size()) * 1.0  / _cameraList.size();
}

void IBRManipulator::setCameraIndex(int value)
{
    if (_cameraList.empty())
        return;

    _currentPosition = value * 1.0 / _cameraList.size();
}

void IBRManipulator::calcCamera()
{
    // make it range < 0
    if (_cameraList.empty())
        return;

    double value = _currentPosition;

    value = fmod(value + 100.0, 1.0);
    _currentPosition = value;
    
    double rangeIn = 0;
    double rangeOut = 0;
    rangeIn = floor(_currentPosition * _cameraList.size());
    rangeOut = fmod(ceil(_currentPosition * _cameraList.size()), _cameraList.size());

    int in = static_cast<int>(rangeIn);
    int out = static_cast<int>(rangeOut);

//    osg::notify(osg::NOTICE) << "in " << in << " out " << out << std::endl;
    double ratio = (_currentPosition * (double)_cameraList.size()) - rangeIn;
    _matrix = computeMatrix(in, out, ratio);

    // should be a better fix ...
    for (int i = 0; i < (int)_cameraList.size(); ++i) {
        _cameraList[i]->setOpacity(_minOpacity);
    }

    _cameraList[in]->setOpacity(_minOpacity + (1.0 - ratio) * (_maxOpacity-_minOpacity));
    _cameraList[out]->setOpacity(_minOpacity + (ratio) * (_maxOpacity-_minOpacity));

}

void
IBRManipulator::setNode(osg::Node* node)
{
    if (_node != node)
    {
        _node = node;
        //Clear the existing camera list and replace it with any cameras found in this node
        _cameraList.clear();
        if (_node.valid())
        {
          IBRCameraFinder finder;
          _node->accept( finder );
          setCameraList( finder._cameraList );
        }
        if (_cameraList.size() > 0)
        {
            setCameraIndex( 0 );
            calcCamera();
        }
    }
}

/** Return node if attached.*/
const osg::Node*
IBRManipulator:: getNode() const
{
    return _node.get();
}

/** Return node if attached.*/
osg::Node*
IBRManipulator::getNode()
{
    return _node.get();
}

