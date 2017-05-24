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

#include <osgJuniper/Velodyne>
#include <osgJuniper/Utils>

using namespace osgJuniper;

CalibrationPoint::CalibrationPoint()
{
}

CalibrationPoint::CalibrationPoint(double rotCorrectionDeg, double vertCorrectionDeg, double distCorrectionCm, double vertOffsetCorrectionCm, double horizOffsetCorrectionCm)
{
    _cosRotationCorrection = cos(osg::DegreesToRadians(rotCorrectionDeg));
    _sinRotationCorrection = sin(osg::DegreesToRadians(rotCorrectionDeg));
    _cosVertCorrection     = cos(osg::DegreesToRadians(vertCorrectionDeg));
    _sinVertCorrection     = sin(osg::DegreesToRadians(vertCorrectionDeg));
    _distCorrectionCm        = distCorrectionCm;
    _vertOffsetCorrectionCm = vertOffsetCorrectionCm;
    _horizOffsetCorrectionCm = horizOffsetCorrectionCm;
}


/**********************************************************************************************/

CalibrationDB::CalibrationDB()
{
    _calibrations.reserve(VLS_MAX_NUM_LASERS);
}

const CalibrationPoint&
CalibrationDB::getCalibration(unsigned int laserNum)
{
    return _calibrations[laserNum];
}

void
CalibrationDB::setCalibrationPoint(unsigned int laserNum, const CalibrationPoint& point)
{
    if (laserNum >= _calibrations.size()) _calibrations.resize(laserNum+1);
    _calibrations[laserNum] = point;
}



CalibrationDB* CalibrationDB::buildDefaultCalibrationDatabase()
{
    CalibrationDB* db = new CalibrationDB();
    for (unsigned int i = 0; i < VLS_MAX_NUM_LASERS; ++i)
    {
        db->setCalibrationPoint(i, CalibrationPoint(RawCorrections[i].m_dRotCorrectionDeg, RawCorrections[i].m_dVertCorrectionDeg, RawCorrections[i].m_dDistCorrectionCm, 
                                                    RawCorrections[i].m_dVertOffsetCorrectionCm, RawCorrections[i].m_dHorizOffsetCorrectionCm));
    }
    return db;
}


/**********************************************************************************************/

osg::Geometry* makeGeometry(osg::Vec3Array* points, osg::Vec4Array* colors, unsigned int numPoints)
{
    osg::Geometry *geometry = new osg::Geometry;
    geometry->setVertexArray( points );
    geometry->setUseDisplayList( false );
    geometry->setUseVertexBufferObjects( true );

    geometry->setColorBinding( osg::Geometry::BIND_PER_VERTEX );
    geometry->setColorArray( colors );

    geometry->addPrimitiveSet( new osg::DrawArrays(GL_POINTS, 0, numPoints ) );
    //osg::notify(osg::NOTICE) << "Added a frame with " << points->size() << " verts" << std::endl;

    return geometry;
}

bool
computeCoords(guint16 laserNum, vls_firing_t* data, CalibrationDB* calibrationDB, osg::Vec3& pos, osg::Vec3& pointLoc)
{
    //Precompute the rotation values to speed things up
    static double rotCosTable[VLS_NUM_ROT_ANGLES];
    static double rotSinTable[VLS_NUM_ROT_ANGLES];
    static bool   tablesInitialized = false;
    if (!tablesInitialized)
    {
        tablesInitialized = true;
        for (unsigned int i = 0; i < VLS_NUM_ROT_ANGLES; i++)
        {
            rotCosTable[i] = cos(osg::DegreesToRadians((double)i / 100.0));
            rotSinTable[i] = sin(osg::DegreesToRadians((double)i / 100.0));
        }
    }

    guint16 idx = laserNum % VLS_LASER_PER_FIRING;

    if (data->points[idx].distance == 0) {
        return false;
    }

    //RAW_CORRECTION_DATA* cal = &RawCorrections[idx];
    const CalibrationPoint& cal = calibrationDB->getCalibration( idx );

    //float distance = (db->getDistLSB() * (float)data->points[idx].distance) + cal->m_dDistCorrectionCm;
    float distance = (0.2f * (float)data->points[idx].distance) + cal._distCorrectionCm;

    float cosVertAngle     = cal._cosVertCorrection;
    float sinVertAngle     = cal._sinVertCorrection;
    float cosRotCorrection = cal._cosRotationCorrection;
    float sinRotCorrection = cal._sinRotationCorrection;

    // cos(a-b) = cos(a)*cos(b) + sin(a)*sin(b)
    // sin(a-b) = sin(a)*cos(b) - cos(a)*sin(b)
    float cosRotAngle = rotCosTable[data->position]*cosRotCorrection + rotSinTable[data->position]*sinRotCorrection;
    float sinRotAngle = rotSinTable[data->position]*cosRotCorrection - rotCosTable[data->position]*sinRotCorrection;

    distance /= VLS_DIM_SCALE;

    // The offset corrections are to be applied in planes orthogonal to the rotatation corrected beam
    //float hOffsetCorr = cal->getHorizOffsetCorrection()/VLS_DIM_SCALE;
    //float vOffsetCorr = cal->getVertOffsetCorrection()/VLS_DIM_SCALE;
    float hOffsetCorr = cal._horizOffsetCorrectionCm/VLS_DIM_SCALE;
    float vOffsetCorr = cal._vertOffsetCorrectionCm/VLS_DIM_SCALE;

    //                           / (distance, shifted by vertical offset)
    //    z ^                  /
    //      |                /
    //      --> y          /          distance                                             
    //     /             /           /            vertOffsetCorrection                      
    //    v x          /           /            +-----------+                              
    //               /           /              |     ^     |                             
    //             /           /                |     |     |                         
    // vertOffset/           /                  |<----o     | horizOffsetCorrection
    //          \          /                    |           |
    //           \ 90deg /                      |           |
    //            \    /         xyDist         +-----------+
    //    90-theta \ /   theta   |              Note: the "o" represents the beam     pointing into the screen
        //         ------------------------               if the beam were aligned with         +y in the world frame
        //           ^  |         ^                       then vertOffset would be         aligned with +z and hoizOffset
        //           |     distance*cos(theta)            would be aligned with -x
        //  vertOffset*cos(90-theta) = vertOffset*sin(theta)
        //
        // Note: theta = vertCorrection angle
        //
        //                   / (x,y)
        //                 /
        //     y         / 
        //    ^        /           xyDist
        //    |      /   |       /
        //    -->x   \   |theta/
        //            \  |   /
        //   horizOff  \ | / 
        //           ----/-------- xyDist*sin(theta)
        //
        // Note: theta = rotCorrection angle

        // Compute the distance in the xy plane (without accounting for rotation)
        float xyDistance = distance * cosVertAngle - vOffsetCorr * sinVertAngle;

    // pos is the position of the scanner, factor in rotation angle and     horizontal offset
    pointLoc.x() = xyDistance * sinRotAngle - hOffsetCorr * cosRotAngle + pos.x()/VLS_DIM_SCALE;
    pointLoc.y() = xyDistance * cosRotAngle + hOffsetCorr * sinRotAngle +  pos.y()/VLS_DIM_SCALE;
    pointLoc.z() = distance * sinVertAngle + vOffsetCorr * cosVertAngle +  pos.z()/VLS_DIM_SCALE;
    return true;
}

VelodyneDataset::VelodyneDataset(const std::string& filename):
_reader(filename),
_filename(filename),
_currFiring(0),
_firingNum(-1),
_lastPosition(-1)
{
    //Build the calibration database
    _calibrationDB = CalibrationDB::buildDefaultCalibrationDatabase();
}

osg::Node*
VelodyneDataset::nextFrame()
{    
    unsigned int batchSize = 10000;

    //Create a geode to hold all of the geometries
    osg::ref_ptr< osg::Geode > geode = new osg::Geode;
    //Turn of lighting
    geode->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF);

    osg::ref_ptr< osg::Vec3Array > verts;
    osg::ref_ptr< osg::Vec4Array > colors;

    unsigned int vertIndex = 0;

    _lastPosition = -1;
    
    bool done = false;
    while (!done)
    {
        //Read a packet if the packet is not valid
        if (!_currFiring)
        {            
            if (!_reader.hasMore() || !_reader.nextPacket(_currPacket))
            {
                osg::notify(osg::NOTICE) << "No more data from packet, stopping reading" << std::endl;
                return 0;
            }

            //We just read a packet, so initialize the values
            _currFiring = (vls_firing_t *) & _currPacket.Bytes[VLS_HDR_LEN];
            _firingNum  = 0;
        }

        //We just read a packet, so now we can process the firings
        for (; _firingNum < VLS_FIRING_PER_PKT; ++_firingNum)
        {    
            //Make the points if we loop over
            if (_lastPosition > _currFiring->position)
            {
                //Add the last batch of points if there is one
                if (verts.valid() && colors.valid())
                {
                    geode->addDrawable( makeGeometry( verts.get(), colors.get(), vertIndex) );
                }
                return geode.release();
            }

            for (unsigned int j = 0; j < VLS_LASER_PER_FIRING; ++j)
            {
                vls_point* point = &_currFiring->points[j];
                osg::Vec3 vert;
                if (computeCoords(j, _currFiring, _calibrationDB, osg::Vec3(0,0,0), vert))
                {
                    if (!verts.valid() || !colors.valid())
                    {
                        verts = new osg::Vec3Array(batchSize);
                        colors = new osg::Vec4Array(batchSize);
                    }

                    (*verts)[vertIndex]  = vert;
                    (*colors)[vertIndex] = DefaultLaserColors[j];
                    vertIndex++;

                    if (vertIndex == batchSize)
                    {
                        geode->addDrawable( makeGeometry( verts.get(), colors.get(), vertIndex ) );
                        vertIndex = 0;
                        verts = 0;
                        colors = 0;
                    }
                }
            }
            _lastPosition = _currFiring->position;
            _currFiring++;
        }
        _currFiring = 0;
    }
    return 0;
}

void
VelodyneDataset::reset()
{
    _reader.reset();
    _currFiring = 0;
    _lastPosition = -1;
    _firingNum = -1;
}





/**********************************************************************************************/
VelodyneStreamingNodeSource::VelodyneStreamingNodeSource(VelodyneDataset* dataset):
_dataset(dataset),
_done(false),
_frameTime(0.02)
{
}

VelodyneStreamingNodeSource::~VelodyneStreamingNodeSource()
{
    cancel();
}

int
VelodyneStreamingNodeSource::cancel()
{
    _done = true;
    while (isRunning())
    {
        OpenThreads::Thread::YieldCurrentThread();
    }
    return 0;
}

void
VelodyneStreamingNodeSource::startImplementation()
{
    _done = false;
    _dataset->reset();
    startThread();
}

void
VelodyneStreamingNodeSource::stopImplementation()
{
    cancel();
}

void
VelodyneStreamingNodeSource::run()
{
    do
    {
        osg::ref_ptr< osg::Node > node = _dataset->nextFrame();
        if (!node.valid())
        {
            _dataset->reset();
        }

        if (node.valid())
        {
            frame(node.get());
        }            

        //Sleep for little bit
        OpenThreads::Thread::microSleep( (unsigned int )(_frameTime * 1000.0f * 1000.0f));
    } while (!_done && !testCancel());
}

