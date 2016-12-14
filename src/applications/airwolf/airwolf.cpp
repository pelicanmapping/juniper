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
#include <osgDB/ReadFile>
#include <osgDB/FileUtils>
#include <osgUtil/Optimizer>
#include <osgUtil/PolytopeIntersector>
#include <osg/Sequence>
#include <osgEarth/ElevationQuery>
#include <osg/Depth>


#include <osgViewer/Viewer>
#include <osgViewer/ViewerEventHandlers>

#include <osgGA/StateSetManipulator>

#include <osgEarth/MapNode>
#include <osgEarth/StringUtils>
#include <osgEarth/TerrainEngineNode>
#include <osgEarthUtil/LatLongFormatter>
#include <osgEarthUtil/ExampleResources>
#include <osgEarthSymbology/Color>
#include <osgEarth/GeoTransform>

#include <osgJuniper/PointCloud>
#include <osgJuniper/PointCloudTools>
#include <osg/ImageStream>

#include <iostream>

using namespace osgJuniper;
using namespace osgEarth;
using namespace osgEarth::Util;


/*******************************************/
struct INSReading
{
    double _utcSeconds;  // UTCsecOfDay_Vec
    osg::Vec3d _velocity; // velX_Vec, velY_Vec, velZ_Vec
    double _velocityMag; // velMag_Vec
    double _velocityXYMag; // velXYMag
    double _waz; // waz_Vec
    double _platAZ; // platAz_Vec
    double _roll; // roll_Vec
    double _pitch; // pitch_Vec
    double _heading; // tHdg_Vec
    double _latRadians; // lat_Vec
    double _lonRadians; // lon_Vec
    double _alt; // alt_Vec
    double _rollRate; // rollRate_Vec
    double _pitchRate; // pitchRate_Vec
    double _yawRate; // yawRate_Vec
    double _rollRateFilt; // rollRateFilt_Vec
    double _pitchRateFilt; // pitchRageFilt_Vec
    double _yawRateFilt; // yawRateFilt_Vec
};

typedef std::vector< INSReading> INSReadings;

class INSReader
{
public:
    static bool read(const std::string& filename, INSReadings& readings)
    {
        readings.clear();

        std::ifstream in(filename, std::ios::in);
        if (!in.is_open())
        {
            OSG_NOTICE << "Failed to load " << filename << std::endl;
            return false;
        }

        //Read the first line of the file, it's the header.
        std::string line;
        getline(in, line);

        osgEarth::StringTokenizer izer( "," );

        int read = 0;

        double prevLat = 0.0;
        double prevLon = 0.0;
        double prevAlt = 0.0;
        double prevTime = 0.0;

        bool readLine = false;
        while (in.good())
        {
            //Read a line from the file            
            getline(in, line);            
            StringVector ized;
            izer.tokenize(line, ized);                                   


            if (ized.size() == 20)
            {           
                INSReading reading;                
                reading._utcSeconds = as<double>(ized[0], 0.0);
                reading._velocity.x() = as<double>(ized[1], 0.0);
                reading._velocity.y() = as<double>(ized[2], 0.0);
                reading._velocity.z() = as<double>(ized[3], 0.0);
                reading._velocityMag = as<double>(ized[4], 0.0);
                reading._velocityXYMag = as<double>(ized[5], 0.0);
                reading._waz = as<double>(ized[6], 0.0);
                reading._platAZ = as<double>(ized[7], 0.0);
                reading._roll = as<double>(ized[8], 0.0);
                reading._pitch = as<double>(ized[9], 0.0);
                reading._heading = as<double>(ized[10], 0.0);
                reading._latRadians = as<double>(ized[11], 0.0);
                reading._lonRadians  = as<double>(ized[12], 0.0);
                reading._alt = as<double>(ized[13], 0.0);
                reading._rollRate = as<double>(ized[14], 0.0);
                reading._pitchRate = as<double>(ized[15], 0.0);
                reading._yawRate = as<double>(ized[16], 0.0);
                reading._rollRateFilt = as<double>(ized[17], 0.0);
                reading._pitchRateFilt = as<double>(ized[18], 0.0);
                reading._yawRateFilt = as<double>(ized[19], 0.0);                

                //if (reading._utcSeconds - prevTime > 3)
                //if (prevLat != reading._latRadians || prevLon != reading._lonRadians || prevAlt != reading._alt)
                {
                    //OE_NOTICE << std::setprecision(20) << reading._latRadians << ", " << reading._lonRadians << ", " << reading._alt << std::endl;
                    readings.push_back( reading );
                    prevTime = reading._utcSeconds;                
                    prevLat = reading._latRadians;
                    prevLon = reading._lonRadians;                
                    prevAlt = reading._alt;

                }                           
                read++;
            }

        }

        return true;
    }
};

osg::Node* makeINSNode(INSReadings& readings)
{   
    osg::Geometry* geometry = new osg::Geometry;
    osg::Vec3Array* verts =new osg::Vec3Array;
    verts->reserve(readings.size());
    geometry->setVertexArray(verts);
    osg::Vec4Array* colors = new osg::Vec4Array();
    colors->push_back( osg::Vec4(1,1,0,1));
    geometry->setColorArray(colors, osg::Array::BIND_OVERALL);
    osgEarth::GeoPoint anchorMap(SpatialReference::create("wgs84"), osg::RadiansToDegrees(readings.front()._lonRadians), osg::RadiansToDegrees(readings.front()._latRadians), readings.front()._alt);
    osg::Vec3d anchor;
    anchorMap.toWorld(anchor);

    for (INSReadings::iterator itr = readings.begin(); itr != readings.end(); ++itr)
    {
        osgEarth::GeoPoint map(SpatialReference::create("wgs84"), osg::RadiansToDegrees(itr->_lonRadians), osg::RadiansToDegrees(itr->_latRadians), itr->_alt);
        osg::Vec3d world;
        map.toWorld(world);
        osg::Vec3d diff = world - anchor;
        verts->push_back(diff);
    }
    geometry->addPrimitiveSet(new osg::DrawArrays(GL_LINE_STRIP, 0, verts->size()));
    osg::MatrixTransform *mt = new osg::MatrixTransform;
    mt->setMatrix(osg::Matrixd::translate(anchor));
    mt->addChild(geometry);
    mt->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
    return mt;
}

typedef std::list< GeoPoint > GeoPointList;

bool loadNeptecLidar(const std::string& filename, GeoPointList& readings)
{
    osg::Timer_t start = osg::Timer::instance()->tick();
    readings.clear();

    std::ifstream in(filename, std::ios::in);
    if (!in.is_open())
    {
        OSG_NOTICE << "Failed to load " << filename << std::endl;
        return false;
    }

    //Read the first line of the file, it's the header.
    std::string line;
    
    osgEarth::StringTokenizer izer( "," );

    int read = 0;

    const osgEarth::SpatialReference* wgs84 = osgEarth::SpatialReference::create("wgs84");

    bool readLine = false;
    while (in.good())
    {
        //Read a line from the file            
        getline(in, line);            
        StringVector ized;
        izer.tokenize(line, ized);                                   

        if (ized.size() == 3)
        {           
            double lat = as<double>(ized[0], 0.0);
            double lon = as<double>(ized[1], 0.0);
            double alt = as<double>(ized[2], 0.0);

            if (alt < -1000 || alt > 100) continue;            

            readings.push_back( GeoPoint(wgs84, lon, lat, alt));

            read++;
        }
    }

    osg::Timer_t end = osg::Timer::instance()->tick();
    OE_NOTICE << "Read " << filename << " in " << osg::Timer::instance()->delta_m(start, end) << std::endl;

    return true;
}

void writeLLA(const std::string& filename, GeoPointList& readings)
{
    std::ofstream fout; 
    fout.open(filename, std::ios::binary | std::ios::out);
    for (GeoPointList::iterator itr = readings.begin(); itr != readings.end(); ++itr)
    {
        fout.write((char *)&itr->x(), sizeof(double));
        fout.write((char *)&itr->y(), sizeof(double));
        fout.write((char *)&itr->z(), sizeof(double));
    }
    fout.close();
}

void readLLA(const std::string& filename, GeoPointList& readings)
{
    osg::Timer_t start = osg::Timer::instance()->tick();
    std::ifstream fout; 
    fout.open(filename, std::ios::binary | std::ios::in);
    const osgEarth::SpatialReference* wgs84 = osgEarth::SpatialReference::create("wgs84");
    double lon, lat, alt;
    while (!fout.eof())
    {
        fout.read((char *)&lon, sizeof(double));
        fout.read((char *)&lat, sizeof(double));
        fout.read((char *)&alt, sizeof(double));
        readings.push_back(GeoPoint(wgs84, lon, lat, alt));
    }
    fout.close();

    osg::Timer_t end = osg::Timer::instance()->tick();
    OE_NOTICE << "Read " << filename << " in " << osg::Timer::instance()->delta_m(start, end) << std::endl;
}

void convertCSVtoLLA(const std::string& directory)
{
    osgDB::DirectoryContents neptecFiles = osgDB::getDirectoryContents(directory);    
    for( osgDB::DirectoryContents::const_iterator f = neptecFiles.begin(); f != neptecFiles.end(); ++f )
    {
        if ( f->compare(".") == 0 || f->compare("..") == 0 )
            continue;

        std::string filepath = directory + "/" +  *f;
        std::string ext = osgDB::getFileExtension(filepath);
        if (ext == "csv" && osgDB::fileType(filepath) == osgDB::REGULAR_FILE)
        {
            OE_NOTICE << "Loading " << filepath << std::endl;
            GeoPointList neptecPoints;
            loadNeptecLidar(filepath, neptecPoints);

            std::string base = osgDB::getNameLessExtension(filepath);
            std::string out = base + ".lla";
            OE_NOTICE << "Writing to " << out << std::endl;
            writeLLA(out, neptecPoints);
        }
    }    
}



osg::Node* makeNepticNode( GeoPointList& readings, const osg::Vec4& color)
{
    osg::Geometry* geometry = new osg::Geometry;
    osg::Vec3Array* verts =new osg::Vec3Array;
    verts->reserve(readings.size());
    geometry->setVertexArray(verts);
    osg::Vec4Array* colors = new osg::Vec4Array();
    colors->push_back( color );
    geometry->setColorArray(colors, osg::Array::BIND_OVERALL);
    geometry->setUseDisplayList(false);
    geometry->setUseVertexBufferObjects(true);

    osgEarth::GeoPoint anchorMap(readings.front());
    osg::Vec3d anchor;
    anchorMap.toWorld(anchor);

    OE_NOTICE << "Making node with " << readings.size() << " points" << std::endl;

    for (GeoPointList::iterator itr = readings.begin(); itr != readings.end(); ++itr)
    {
        osg::Vec3d world;
        itr->toWorld(world);
        verts->push_back(world - anchor);
    }
    geometry->addPrimitiveSet(new osg::DrawArrays(GL_POINTS, 0, verts->size()));
    osg::MatrixTransform *mt = new osg::MatrixTransform;
    mt->setMatrix(osg::Matrixd::translate(anchor));
    mt->addChild(geometry);
    mt->getOrCreateStateSet()->setAttributeAndModes(new osg::Point(2.0));
    mt->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
    return mt;
}

osg::Sequence* loadSession(const std::string& neptecDir, unsigned int maxFiles)
{
    unsigned int numRead = 0;
    osg::Sequence* neptecGroup = new osg::Sequence();
    osgDB::DirectoryContents neptecFiles = osgDB::getDirectoryContents(neptecDir);
    for( osgDB::DirectoryContents::const_iterator f = neptecFiles.begin(); f != neptecFiles.end(); ++f )
    {
        if ( f->compare(".") == 0 || f->compare("..") == 0 )
            continue;

        std::string filepath = neptecDir + "/" +  *f;
        std::string ext = osgDB::getFileExtension(filepath);
        if (ext == "csv" && osgDB::fileType(filepath) == osgDB::REGULAR_FILE)
        {
            OE_NOTICE << "Loading " << filepath << std::endl;
            GeoPointList neptecPoints;
            loadNeptecLidar(filepath, neptecPoints);
            if (!neptecPoints.empty())
            {
                neptecGroup->addChild(makeNepticNode(neptecPoints, osg::Vec4(1,1,1,1)), 0.1);
                numRead++;
                if (numRead == maxFiles)
                {
                    break;
                }
            }
        }
    }
    neptecGroup->setInterval(osg::Sequence::LOOP, 0, -1);
    neptecGroup->setDuration(1.0f, -1);
    neptecGroup->setMode(osg::Sequence::START);    
    return neptecGroup;    
}

osg::Node* loadSessionLLA(const std::string& neptecDir, unsigned int maxFiles, double timePerFrame)
{
    unsigned int numRead = 0;
    osg::Sequence* neptecGroup = new osg::Sequence();

    std::vector< std::string > filenames;
    

    osgDB::DirectoryContents neptecFiles = osgDB::getDirectoryContents(neptecDir);
    for( osgDB::DirectoryContents::const_iterator f = neptecFiles.begin(); f != neptecFiles.end(); ++f )
    {
        if ( f->compare(".") == 0 || f->compare("..") == 0 )
            continue;

        std::string filepath = neptecDir + "/" +  *f;
        std::string ext = osgDB::getFileExtension(filepath);
        if (ext == "lla" && osgDB::fileType(filepath) == osgDB::REGULAR_FILE)
        {
            filenames.push_back( filepath );            
        }
    }    



    OE_NOTICE << "Loading " << filenames.size() << std::endl;
    std::sort(filenames.begin(), filenames.end());

    for (unsigned int i = 0; i < filenames.size(); i++)
    {
        std::string filename = filenames[i];
        OE_NOTICE << "Loading " << numRead << " " << filename << std::endl;
        GeoPointList neptecPoints;
        readLLA(filename, neptecPoints);
        if (neptecPoints.size() > 10)
        {
            OE_NOTICE << "Read " << neptecPoints.size() << std::endl;
            neptecGroup->addChild(makeNepticNode(neptecPoints, osg::Vec4(1,0,1,1)));        
            neptecGroup->setTime(neptecGroup->getNumChildren()-1, timePerFrame);            
            numRead++;
            if (numRead == maxFiles)
            {
                break;
            }
        }
    }

    neptecGroup->setInterval(osg::Sequence::LOOP, 0, -1);
    neptecGroup->setDuration(1.0f, -1);
    neptecGroup->setMode(osg::Sequence::START);
    OE_NOTICE << "Number of frames " << neptecGroup->getNumChildren() << std::endl;
    return neptecGroup;    
}

osg::Node* loadSessionLLAFull(const std::string& neptecDir, unsigned int maxFiles)
{
    unsigned int numRead = 0;

    std::vector< std::string > filenames;
  
    osgDB::DirectoryContents neptecFiles = osgDB::getDirectoryContents(neptecDir);
    for( osgDB::DirectoryContents::const_iterator f = neptecFiles.begin(); f != neptecFiles.end(); ++f )
    {
        if ( f->compare(".") == 0 || f->compare("..") == 0 )
            continue;

        std::string filepath = neptecDir + "/" +  *f;
        std::string ext = osgDB::getFileExtension(filepath);
        if (ext == "lla" && osgDB::fileType(filepath) == osgDB::REGULAR_FILE)
        {
            filenames.push_back( filepath );            
        }
    }    



    
    
    OE_NOTICE << "Loading " << filenames.size() << std::endl;
    std::sort(filenames.begin(), filenames.end());

    GeoPointList neptecPoints;
    for (unsigned int i = 0; i < filenames.size(); i++)
    {
        std::string filename = filenames[i];
        OE_NOTICE << "Loading " << numRead << " " << filename << std::endl;        
        readLLA(filename, neptecPoints);
        numRead++;
        if (numRead == maxFiles)
        {
            break;
        }        
    }

    return makeNepticNode(neptecPoints, osg::Vec4(1,1,1,1));
}


osg::AnimationPath* createPath( INSReadings& readings, osgEarth::MapNode* mapNode, double simulationStart )
{
    osg::AnimationPath* path = new osg::AnimationPath();
    const SpatialReference* wgs84 = SpatialReference::create("wgs84");
    double heading = 0.0;

    ElevationQuery   query(mapNode->getMap());
    for (INSReadings::iterator itr = readings.begin(); itr != readings.end(); ++itr)
    {       
        osgEarth::GeoPoint map(wgs84, osg::RadiansToDegrees(itr->_lonRadians), osg::RadiansToDegrees(itr->_latRadians), itr->_alt);
        double elevation;
        osg::Vec3d world;
        map.toWorld(world);
        osg::Matrixd local2world;
        map.createLocalToWorld( local2world );  
        
        double pitch = itr->_pitch;        
        double heading = itr->_heading;
        double roll = itr->_roll;      
        double platAZ = itr->_platAZ;


        osg::Quat pitchTo90(osg::DegreesToRadians(90.0), osg::Vec3(1,0,0));
        osg::Quat ori =
        osg::Quat(roll, osg::Vec3(0,1,0)) * // roll
        osg::Quat(pitch, osg::Vec3(1,0,0)) * //pitch        
        osg::Quat(heading, osg::Vec3(0,0,-1));                

        osg::Quat rot = pitchTo90 * ori * local2world.getRotate();

        double time = itr->_utcSeconds - simulationStart;
        if (time >= 0.0)
        {
            path->insert(time, osg::AnimationPath::ControlPoint(world, rot));
        }       
    }
    return path;
}

static PointCloudDecorator* s_pointCloud;

static LabelControl* s_status;
static LabelControl* s_frameTime;
static LabelControl* s_videoTime;

osg::Node::NodeMask MaskMapNode = 0x01;
osg::Node::NodeMask MaskPointCloud = 0x02;

struct PointSizeHandler : public ControlEventHandler
{
    PointSizeHandler( PointCloudDecorator* pointCloud ) : _pointCloud(pointCloud) { }
    void onValueChanged( Control* control, float value )
    {        
        _pointCloud->setPointSize(value);
        OSG_NOTICE << "Point size " << value << std::endl;
    }
    osg::ref_ptr< PointCloudDecorator > _pointCloud;
};

struct MaxIntensityHandler : public ControlEventHandler
{
    MaxIntensityHandler( PointCloudDecorator* pointCloud ) : _pointCloud(pointCloud) { }
    void onValueChanged( Control* control, float value )
    {        
        _pointCloud->setMaxIntensity(value);
        OSG_NOTICE << "Max Intensity " << value << std::endl;
    }

    osg::ref_ptr< PointCloudDecorator > _pointCloud;
};

struct MinHeightHandler : public ControlEventHandler
{
    MinHeightHandler( PointCloudDecorator* pointCloud ) : _pointCloud(pointCloud) { }
    void onValueChanged( Control* control, float value )
    {        
        _pointCloud->setMinHeight(value);
        OSG_NOTICE << "Min Height " << value << std::endl;
    }

    osg::ref_ptr< PointCloudDecorator > _pointCloud;
};

//double fov = 66.0;
double fov = 90.0;

struct FOVHandler : public ControlEventHandler
{
    FOVHandler( )
    {
    }

    void onValueChanged( Control* control, float value )
    {        
        fov = value;
        OSG_NOTICE << "fov " << fov << std::endl;
    }
};

struct UniformHandler : public ControlEventHandler
{
    UniformHandler(osg::Uniform* uniform ):
_uniform(uniform)
    {
    }

    void onValueChanged( Control* control, float value )
    {                
        _uniform->set(value);
    }

    osg::ref_ptr< osg::Uniform > _uniform;
};



struct MaxHeightHandler : public ControlEventHandler
{
    MaxHeightHandler( PointCloudDecorator* pointCloud ) : _pointCloud(pointCloud) { }
    void onValueChanged( Control* control, float value )
    {        
        _pointCloud->setMaxHeight(value);
        OSG_NOTICE << "Max Height " << value << std::endl;
    }

    osg::ref_ptr< PointCloudDecorator > _pointCloud;
};

struct HazeDistanceHandler : public ControlEventHandler
{
    HazeDistanceHandler( PointCloudDecorator* pointCloud ) : _pointCloud(pointCloud) { }
    void onValueChanged( Control* control, float value )
    {        
        _pointCloud->setHazeDistance(value);
        OSG_NOTICE << "Haze distance " << value << std::endl;
    }

    osg::ref_ptr< PointCloudDecorator > _pointCloud;
};

// http://resources.arcgis.com/en/help/main/10.1/index.html#//015w0000005q000000
std::string classificationToString(unsigned short classification)
{

    switch (classification)
    {
    case 0:
        return "Never classified";
    case 1:
        return "Unassigned";
    case 2:
        return "Ground";
    case 3:
        return "Low Vegetation";
    case 4:
        return "Medium Vegetation";
    case 5:
        return "High Vegetation";
    case 6:
        return "Building";
    case 7:
        return "Noise";
    case 8:
        return "Model Key";
    case 9:
        return "Water";
    case 10:
        return "Reserved for ASPRS Definition";
    case 11:
        return "Reserved for ASPRS Definition";
    case 12:
        return "Overlap";
    default:
        return "Reserved fro ASPRS Definition";
    };
}


/**
* Callback for when a point is identified.  Updates a label with info about the point.
*/
struct IdentifyCallback : public IdentifyPointHandler::Callback
{
    IdentifyCallback(LabelControl* label):
_label(label)
{
}

virtual void selected(const Point& point)
{
    // Assume the point is in geocentric
    osgEarth::SpatialReference* wgs84 = osgEarth::SpatialReference::create("epsg:4326");
    osgEarth::GeoPoint geoPoint;
    geoPoint.fromWorld(wgs84, point.position);
    LatLongFormatter formatter;
    formatter.setPrecision(8);
    std::stringstream buf;
    buf << "Location: " << formatter.format(geoPoint) << ", " << geoPoint.z() << std::endl
        << "Classification: " << classificationToString(point.classification) << std::endl
        << "Intensity: " << point.intensity << std::endl
        << "RGBA: " << (int)point.color.r() << ", " << (int)point.color.g() << ", " << (int)point.color.b() << ", " << (int)point.color.a() << std::endl
        << "Return: " << (int)point.returnNumber << std::endl;

    s_status->setText( buf.str() );     

}

virtual void reset()
{
    s_status->setText("");
}

LabelControl* _label;
};

struct P2PMeasureCallback : public P2PMeasureHandler::Callback
{
    virtual void distanceChanged(double distance)
    {
        std::stringstream buf;
        buf << "Distance: " << distance << "m";
        s_status->setText(buf.str());
    }
};

struct ChangeColorModeHandler : public ControlEventHandler
{
    ChangeColorModeHandler(PointCloudDecorator::ColorMode colorMode):
_colorMode(colorMode)
{
}

void onClick(Control* control, int mouseButtonMask)
{
    s_pointCloud->setColorMode(_colorMode);
}

PointCloudDecorator::ColorMode _colorMode;
};

struct ToggleClassificationHandler : public ControlEventHandler
{
    ToggleClassificationHandler(unsigned char classification):
_classification(classification)
{
}

void onValueChanged(Control* control, bool value)
{
    s_pointCloud->setClassificationVisible(_classification, value);
}

unsigned char _classification;
};

struct AutoPointSizeHandler : public ControlEventHandler
{
    void onValueChanged(Control* control, bool value)
    {
        s_pointCloud->setAutoPointSize(!s_pointCloud->getAutoPointSize());
    }
};


static const char *vertSource = 
        "varying vec4 texCoord0;\n"   
        "void main()"
        "{\n"
        "    texCoord0 = gl_MultiTexCoord0;\n"
        "    gl_Position = gl_ProjectionMatrix * gl_ModelViewMatrix * gl_Vertex;\n"
        "\n"
        "}\n";

    static const char* fragSource =
        "varying vec4 texCoord0;\n"   
        "uniform sampler2D texture_unit;\n"
        "uniform float opacity;\n"
        "uniform float slider;\n"
        "void main()"
        "{\n"
        "   vec4 color = texture2D(texture_unit, texCoord0.st);\n"
        "   color.a = color.a * opacity;\n"
        "   if (gl_FragCoord.x < slider) discard;\n"
        "   gl_FragColor = color;\n"
        "}\n";


osg::Node* createVideoHUD(osg::Image* image, float alpha = 1.0)
{
    osg::Camera* hudCamera = new osg::Camera;
    hudCamera->setReferenceFrame(osg::Transform::ABSOLUTE_RF);
    hudCamera->setProjectionMatrix(osg::Matrix::ortho2D(0.0, 1.0, 0.0, 1.0));
    hudCamera->setViewMatrix(osg::Matrix::identity());    
    hudCamera->setClearMask(GL_DEPTH_BUFFER_BIT);

    //hudCamera->setRenderOrder(osg::Camera::NESTED_RENDER, -10);    
    hudCamera->setRenderOrder(osg::Camera::POST_RENDER);    
    hudCamera->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
    hudCamera->getOrCreateStateSet()->setMode(GL_DEPTH_TEST, osg::StateAttribute::OFF);
    hudCamera->getOrCreateStateSet()->setMode(GL_BLEND, osg::StateAttribute::ON);
    osg::Depth* depth = new osg::Depth();
    depth->setWriteMask(false);
    hudCamera->getOrCreateStateSet()->setAttributeAndModes(depth);
    hudCamera->setAllowEventFocus(false);

    osg::Geometry* geometry = new osg::Geometry;
    osg::Vec3Array* verts = new osg::Vec3Array;
    geometry->setVertexArray(verts);
    verts->push_back(osg::Vec3(0.0, 0.0, 0.0));
    verts->push_back(osg::Vec3(1.0, 0.0, 0.0));
    verts->push_back(osg::Vec3(1.0, 1.0, 0.0));
    verts->push_back(osg::Vec3(0.0, 1.0, 0.0));

    osg::Vec2Array* texCoords = new osg::Vec2Array;

    bool flip = image->getOrigin()==osg::Image::TOP_LEFT;    
    texCoords->push_back(osg::Vec2(0.0f, flip? 1.0f: 0.0f));
    texCoords->push_back(osg::Vec2(1.0f, flip? 1.0f: 0.0f));
    texCoords->push_back(osg::Vec2(1.0f, flip? 0.0f: 1.0f));
    texCoords->push_back(osg::Vec2(0.0f, flip? 0.0f: 1.0f));
    geometry->setTexCoordArray(0, texCoords);

    osg::Vec4Array* colors = new osg::Vec4Array();
    colors->push_back(osg::Vec4(1, 1, 1, alpha));
    geometry->setColorArray(colors);
    geometry->setColorBinding(osg::Geometry::BIND_OVERALL);

    osg::Texture2D* texture = new osg::Texture2D(image);
    texture->setResizeNonPowerOfTwoHint(false);
    texture->setFilter(osg::Texture2D::MIN_FILTER, osg::Texture2D::LINEAR_MIPMAP_LINEAR);
    texture->setFilter(osg::Texture2D::MAG_FILTER, osg::Texture2D::LINEAR);    
    geometry->getOrCreateStateSet()->setTextureAttributeAndModes(0, texture, osg::StateAttribute::ON);

    geometry->addPrimitiveSet(new osg::DrawArrays(GL_QUADS, 0, verts->size()));

    osg::Program *program = new osg::Program;
    program->addShader(new osg::Shader(osg::Shader::VERTEX, vertSource));
    program->addShader(new osg::Shader(osg::Shader::FRAGMENT, fragSource));
    geometry->getOrCreateStateSet()->setAttributeAndModes(program, osg::StateAttribute::ON);
    geometry->getOrCreateStateSet()->addUniform(new osg::Uniform("texture_unit", 0));
            
    hudCamera->addChild( geometry );

    return hudCamera;
        
}



void buildControls(osgViewer::Viewer& viewer, osg::Group* root, osg::Node* videoNode)
{
    ControlCanvas* canvas = ControlCanvas::getOrCreate( &viewer );
    VBox* container = canvas->addControl(new VBox());
    container->setBackColor(Color(Color::Black,0.5));

    if (s_pointCloud)
    {
        // Point size
        HBox* pointSizeBox = container->addControl(new HBox());
        pointSizeBox->setChildVertAlign( Control::ALIGN_CENTER );
        pointSizeBox->setChildSpacing( 10 );
        pointSizeBox->setHorizFill( true );
        pointSizeBox->addControl( new LabelControl("Point Size:", 16) );

        HSliderControl* pointSlider = pointSizeBox->addControl(new HSliderControl(1.0, 10.0f, 1.0f));
        pointSlider->setBackColor( Color::Gray );
        pointSlider->setHeight( 12 );
        pointSlider->setHorizFill( true, 200 );
        pointSlider->addEventHandler( new PointSizeHandler(s_pointCloud));   

        // Max Intensity
        HBox* maxIntensityBox = container->addControl(new HBox());
        maxIntensityBox->setChildVertAlign( Control::ALIGN_CENTER );
        maxIntensityBox->setChildSpacing( 10 );
        maxIntensityBox->setHorizFill( true );
        maxIntensityBox->addControl( new LabelControl("Max Intensity:", 16) );

        //HSliderControl* intensitySlider = maxIntensityBox->addControl(new HSliderControl(0.0f, USHRT_MAX, s_pointCloud->getMaxIntensity()));
        HSliderControl* intensitySlider = maxIntensityBox->addControl(new HSliderControl(0.0f, 255.0, 255.0));
        intensitySlider->setBackColor( Color::Gray );
        intensitySlider->setHeight( 12 );
        intensitySlider->setHorizFill( true, 200 );
        intensitySlider->addEventHandler( new MaxIntensityHandler(s_pointCloud));   

        // Min Height
        HBox* minHeightBox = container->addControl(new HBox());
        minHeightBox->setChildVertAlign( Control::ALIGN_CENTER );
        minHeightBox->setChildSpacing( 10 );
        minHeightBox->setHorizFill( true );
        minHeightBox->addControl( new LabelControl("Min Height:", 16) );

        HSliderControl* minHeightSlider = minHeightBox->addControl(new HSliderControl(0.0f, 30.0, s_pointCloud->getMinHeight()));
        minHeightSlider->setBackColor( Color::Gray );
        minHeightSlider->setHeight( 12 );
        minHeightSlider->setHorizFill( true, 200 );
        minHeightSlider->addEventHandler( new MinHeightHandler(s_pointCloud));   

        // Max Height
        HBox* maxHeightBox = container->addControl(new HBox());
        maxHeightBox->setChildVertAlign( Control::ALIGN_CENTER );
        maxHeightBox->setChildSpacing( 10 );
        maxHeightBox->setHorizFill( true );
        maxHeightBox->addControl( new LabelControl("Max Height:", 16) );

        HSliderControl* maxHeightSlider = maxHeightBox->addControl(new HSliderControl(0.0f, 100.0, 100.0));
        maxHeightSlider->setBackColor( Color::Gray );
        maxHeightSlider->setHeight( 12 );
        maxHeightSlider->setHorizFill( true, 200 );
        maxHeightSlider->addEventHandler( new MaxHeightHandler(s_pointCloud));   
    }

    // video slider
    HBox* sliderBox = container->addControl(new HBox());
    if (videoNode)
    {
        sliderBox->setChildVertAlign( Control::ALIGN_CENTER );
        sliderBox->setChildSpacing( 10 );
        sliderBox->setHorizFill( true );
        sliderBox->addControl( new LabelControl("Video Slider:", 16) );

        HSliderControl* sliderSlider = sliderBox->addControl(new HSliderControl(0.0, 1280.0, 0.0));
        sliderSlider->setBackColor( Color::Gray );
        sliderSlider->setHeight( 12 );
        sliderSlider->setHorizFill( true, 200 );
        sliderSlider->addEventHandler( new UniformHandler(videoNode->getOrCreateStateSet()->getOrCreateUniform("slider", osg::Uniform::FLOAT)));   


        // video opacity
        HBox* alphaBox = container->addControl(new HBox());
        alphaBox->setChildVertAlign( Control::ALIGN_CENTER );
        alphaBox->setChildSpacing( 10 );
        alphaBox->setHorizFill( true );
        alphaBox->addControl( new LabelControl("Video Opacity:", 16) );

        HSliderControl* alphaSlider = alphaBox->addControl(new HSliderControl(0.0, 1.0, 1.0));
        alphaSlider->setBackColor( Color::Gray );
        alphaSlider->setHeight( 12 );
        alphaSlider->setHorizFill( true, 200 );
        alphaSlider->addEventHandler( new UniformHandler(videoNode->getOrCreateStateSet()->getOrCreateUniform("opacity", osg::Uniform::FLOAT)));       
    }


#if 0
    // fov
    HBox* fovBox = container->addControl(new HBox());
    fovBox->setChildVertAlign( Control::ALIGN_CENTER );
    fovBox->setChildSpacing( 10 );
    fovBox->setHorizFill( true );
    fovBox->addControl( new LabelControl("FOV:", 16) );

    HSliderControl* fovSlider = fovBox->addControl(new HSliderControl(0.0, 180.0, 60.0));
    fovSlider->setBackColor( Color::Gray );
    fovSlider->setHeight( 12 );
    fovSlider->setHorizFill( true, 200 );
    fovSlider->addEventHandler( new FOVHandler());   
#endif

    if (s_pointCloud)
    {
        // Haze distance
        HBox* hazeBox = container->addControl(new HBox());
        hazeBox->setChildVertAlign( Control::ALIGN_CENTER );
        hazeBox->setChildSpacing( 10 );
        hazeBox->setHorizFill( true );
        hazeBox->addControl( new LabelControl("Haze Distance:", 16) );

        HSliderControl* hazeSlider = hazeBox->addControl(new HSliderControl(0.0f, 10000.0, 5000.0f));
        hazeSlider->setBackColor( Color::Gray );
        hazeSlider->setHeight( 12 );
        hazeSlider->setHorizFill( true, 200 );
        hazeSlider->addEventHandler( new HazeDistanceHandler(s_pointCloud));   

        // Color mode
        Grid* toolbar = new Grid();    
        toolbar->setAbsorbEvents( true );    

        LabelControl* rgb = new LabelControl("RGB");
        rgb->addEventHandler(new ChangeColorModeHandler(PointCloudDecorator::RGB));
        toolbar->setControl(0, 0, rgb);

        LabelControl* intensity = new LabelControl("Intensity");
        intensity->addEventHandler(new ChangeColorModeHandler(PointCloudDecorator::Intensity));
        toolbar->setControl(1, 0, intensity);

        LabelControl* classifiction = new LabelControl("Classification");
        classifiction->addEventHandler(new ChangeColorModeHandler(PointCloudDecorator::Classification));
        toolbar->setControl(2, 0, classifiction);   

        LabelControl* height = new LabelControl("Height");
        height->addEventHandler(new ChangeColorModeHandler(PointCloudDecorator::Height));
        toolbar->setControl(3, 0, height);   

        LabelControl* ramp = new LabelControl("Ramp");
        ramp->addEventHandler(new ChangeColorModeHandler(PointCloudDecorator::Ramp));
        toolbar->setControl(4, 0, ramp);   

        container->addChild(toolbar);

        HBox* box = container->addControl(new HBox());
        CheckBoxControl* vegToggle = box->addControl(new CheckBoxControl(true));
        vegToggle->addEventHandler(new ToggleClassificationHandler(3));
        vegToggle->addEventHandler(new ToggleClassificationHandler(4));
        vegToggle->addEventHandler(new ToggleClassificationHandler(5));
        box->addControl(new LabelControl("Vegetation"));

        box = container->addControl(new HBox());
        CheckBoxControl* buildingToggle = box->addControl(new CheckBoxControl(true));
        buildingToggle->addEventHandler(new ToggleClassificationHandler(6));
        box->addControl(new LabelControl("Buildings"));

        box = container->addControl(new HBox());
        CheckBoxControl* groundToggle = box->addControl(new CheckBoxControl(true));
        groundToggle->addEventHandler(new ToggleClassificationHandler(2));
        box->addControl(new LabelControl("Ground"));

        box = container->addControl(new HBox());
        CheckBoxControl* autoPointSizeToggle = box->addControl(new CheckBoxControl(s_pointCloud->getAutoPointSize()));
        autoPointSizeToggle->addEventHandler(new AutoPointSizeHandler());
        box->addControl(new LabelControl("Auto Point Size"));
    }

    // Add a status label
    s_status = container->addControl(new LabelControl());

    if (videoNode)
    {
        s_videoTime = container->addControl(new LabelControl());
    }
    s_frameTime = container->addControl(new LabelControl());
}

int
usage( const std::string& msg )
{
    if ( !msg.empty() )
    {
        std::cout << msg << std::endl;
    }

    std::cout
        << std::endl
        << "USAGE: airwolf" << std::endl
        << std::endl
        << "    --video videoFile                   ; The video to load" << std::endl
        << "    --ins insFile                       ; The INS csv to load" << std::endl        
        << "    --startTime utcTime                 ; The start time for the INS animation, used to sync with the video" << std::endl        
        << "    --lodScale                          ; The lod scale to start with" << std::endl        
        << "    --gradient                          ; The gradient texture to load" << std::endl        
        << "    --neptec                            ; Load a directory of neptec lla files" << std::endl        
        << "    --neptecFrameTime                   ; The time in seconds to display each neptec lidar frame" << std::endl
        << "    map.earth" << std::endl
        << std::endl;

    return -1;
}


int main(int argc, char** argv)
{    
    osg::ArgumentParser arguments(&argc,argv);

    if (arguments.argc() <= 1)
    {
        return usage("Please specify a .earth file");
    }

    osgViewer::Viewer viewer(arguments);    

    // Turn on incremental compile operation 
    viewer.setIncrementalCompileOperation(new osgUtil::IncrementalCompileOperation());

    viewer.setCameraManipulator( new EarthManipulator());

    osg::Group* root = new osg::Group;

    osg::Node* loaded = osgEarth::Util::MapNodeHelper().load(arguments, &viewer);
    root->addChild(loaded);

    osg::ref_ptr< MapNode > mapNode = MapNode::findMapNode(loaded);
    mapNode->getTerrainEngine()->setNodeMask(MaskMapNode);
    mapNode->getModelLayerGroup()->setNodeMask(MaskPointCloud);

    std::string gradient = "data/iron_gradient.png";
    arguments.read("--gradient", gradient);


    s_pointCloud = osgEarth::findTopMostNodeOfType<PointCloudDecorator>(loaded);
    if (s_pointCloud)
    {
        s_pointCloud->getOrCreateStateSet()->setRenderBinDetails(99999, "RenderBin");

        // Load the color ramp gradient.        
        osg::Texture2D* colorRamp = new osg::Texture2D(osgDB::readImageFile(gradient));
        colorRamp->setResizeNonPowerOfTwoHint(false);
        colorRamp->setWrap( osg::Texture::WRAP_S, osg::Texture::CLAMP_TO_EDGE );
        colorRamp->setWrap( osg::Texture::WRAP_T, osg::Texture::CLAMP_TO_EDGE );
        s_pointCloud->setColorRamp(colorRamp);
        s_pointCloud->setColorMode(PointCloudDecorator::Ramp);
    }

    double simulationStart = 63629;
    arguments.read("--startTime", simulationStart);
    OSG_NOTICE << "Simulation start time " << simulationStart << std::endl;

    // Preload the ffmpeg plugin.
    std::string ffmpegLib = osgDB::Registry::instance()->createLibraryNameForExtension( "ffmpeg" );
    if ( !ffmpegLib.empty() )
        osgDB::Registry::instance()->loadLibrary( ffmpegLib );    

    // Load the video file.
    std::string videoFile;
    arguments.read("--video", videoFile);

    osg::ref_ptr< osg::Node > videoNode;
    osg::ref_ptr< osg::ImageStream > video;

    if (!videoFile.empty())
    {
        video = dynamic_cast< osg::ImageStream*>(osgDB::readImageFile(videoFile));
        if (video.valid())
        {
            OSG_NOTICE << "Loaded video length=" << video->getLength() << std::endl;        
            videoNode = createVideoHUD(video, 0.5f);
            root->addChild( videoNode );

            videoNode->getOrCreateStateSet()->getOrCreateUniform("opacity", osg::Uniform::FLOAT)->set(1.0f);
            videoNode->getOrCreateStateSet()->getOrCreateUniform("slider", osg::Uniform::FLOAT)->set(0.0f);
        }
    }
    

    buildControls(viewer, root, videoNode);

    viewer.addEventHandler(new osgViewer::StatsHandler());
    viewer.addEventHandler(new osgViewer::WindowSizeHandler());
    viewer.addEventHandler(new osgViewer::LODScaleHandler());
    viewer.addEventHandler(new osgGA::StateSetManipulator(viewer.getCamera()->getOrCreateStateSet()));

    float lodScale = 1.0;
    // Up the lod scale so things behave a bit nicer.
    viewer.getCamera()->setLODScale(lodScale);
    arguments.read("--lodScale", lodScale);
    OSG_NOTICE << "lodScale=" << lodScale << std::endl;
      
    viewer.getCamera()->setNearFarRatio(0.00002);


    // Read INS camera path.
    std::string insFile;
    arguments.read("--ins", insFile);   
    INSReadings readings;
    if (!insFile.empty())
    {        
        osg::Timer_t startTime = osg::Timer::instance()->tick();
        INSReader::read(insFile, readings);
        osg::Timer_t stopTime = osg::Timer::instance()->tick();
        OE_NOTICE << "Read " << readings.size() << " INS readings in " << osg::Timer::instance()->delta_s(startTime, stopTime) << std::endl;
        root->addChild(makeINSNode(readings));
    }

    double neptecFrameTime = 0.1;
    arguments.read("--neptecFrameTime", neptecFrameTime);

    // Load directories full of the neptec lidar series.
    std::string neptec;
    while (arguments.read("--neptec", neptec))
    {
        OSG_NOTICE << "Loading neptec lidar directory " << neptec << std::endl;
        root->addChild(loadSessionLLA(neptec, UINT_MAX, neptecFrameTime));
    }

    // any option left unread are converted into errors to write out later.
    arguments.reportRemainingOptionsAsUnrecognized();


    // Convert a folder of csv files to lla.
    //convertCSVtoLLA("J:/fdp1");
    //convertCSVtoLLA("J:/fdp3");
    //convertCSVtoLLA("J:/fdp4");
    //return 0;

    // Animation path if we loaded some INS data.
    if (readings.size() > 0)
    {
        osg::AnimationPath* path = createPath( readings, mapNode, simulationStart);
        osgGA::AnimationPathManipulator* apm = new osgGA::AnimationPathManipulator( path );
        viewer.setCameraManipulator( apm );    
    }    

    viewer.setSceneData( root );

    viewer.getCamera()->setProjectionResizePolicy(osg::Camera::FIXED);

    if (video)
    {
        video->play();
    }
    while (!viewer.done())
    {
        double frameTime = simulationStart + viewer.getFrameStamp()->getReferenceTime();
        s_frameTime->setText("Frame time " + osgEarth::prettyPrintTime(frameTime));
        if (video)
        {
            double videoTime = simulationStart + video->getCurrentTime();        
            s_videoTime->setText("Video time " + osgEarth::prettyPrintTime(videoTime));
            if (osg::absolute(video->getCurrentTime() - viewer.getFrameStamp()->getReferenceTime()) > 1.0)
            {
                video->seek(viewer.getFrameStamp()->getReferenceTime());
                OE_NOTICE << "Seeking to " << viewer.getFrameStamp()->getReferenceTime() << std::endl;
            }        
        }
        double fovy, ar, znear, zfar;
        viewer.getCamera()->getProjectionMatrixAsPerspective( fovy, ar, znear, zfar );        
        viewer.getCamera()->setProjectionMatrixAsPerspective( fov, ar, znear, zfar );         

        viewer.frame();
    }
}
