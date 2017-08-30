# Building Juniper from source on Windows

Juniper is built on the shoulders of giants, and has the dependencies to prove it!

Unfortunately, all of those giants are also built on giants, so they have dependencies of their own :anguished:

Fortunately, most of the dependencies can be downloaded from the OSGGeo4W project.  

Note:  This guide assumes you're using Visual Studio 2015 

## OSGEO4W Dependencies
1)  Go to https://trac.osgeo.org/osgeo4w/ and down the 64 bit installer.
2)  Run the installer, choose "Advanced Install" and make sure you select the following packages:
    * gdal
    * gdal-dev
    * pdal
    * freetype-vc-devel
3)  Edit the PDAL CMake file at C:\osgeo4w64\lib\pdal\PDALConfig.cmake and change all instances of c:/projects/PDAL/scripts/osgeo4w/install to "${PACKAGE_PREFIX_DIR}"

## OpenSceneGraph 3.5.6
1)  Build OSG 3.5.6 as usual, but pointing to the dependencies at c:\osgeo4w64
2)  Append "legacy_stdio_definitions.lib" to the end of the CMAKE_CXX_STANDARD_LIBRARIES cmake variable
    - Required since VS2015 broke how stdio works, see https://msdn.microsoft.com/en-us/library/bb531344.aspx
    
## osgEarth
1)  Build osgEarth from the master branch as usual, but with all dependencies pointing to the c:\osgeo4w64

## Juniper
1)  Build Juniper as usual, but point the PDAL_DIR variable to C:\osgeo4w64\lib\pdal

After building and installing osg, osgEarth and juniper you'll neeed to setup your path properly so everything can be loaded at runtime.  You can create a batch file similar to this one to launch a shell that contains all the paths to everything you need:
```
REM Setup dependencies from osgeo4w
call "C:\OSGeo4W64\bin\o4w_env.bat"

REM Set to your path to OSG
set PATH=%PATH%;D:\dev\Installs\OpenSceneGraph\vs2015\x64\bin
REM Set to your path to osgEarth
set PATH=%PATH%;D:\dev\Installs\OSGEARTH\OpenSceneGraph\vs2015\x64\bin
REM Set to your path to Juniper
set PATH=%PATH%;D:\dev\Installs\juniper\vs2015\x64\bin
REM Set to your path to Juniper, which contains a pdal points plugin
set PDAL_DRIVER_PATH=D:\dev\Installs\juniper\vs2015\x64\bin

REM Good defaults for osg's database pager threads.
set OSG_NUM_DATABASE_THREADS=8
set OSG_NUM_HTTP_DATABASE_THREADS=4
set OSG_NEAR_FAR_RATIO=0.00002
cmd
```

After tiling a point cloud dataset you should be able to run this to view it.
```
osgviewer tileset.lastile
```









