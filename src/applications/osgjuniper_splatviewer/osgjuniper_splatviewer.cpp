/* OpenSceneGraph example, osgsequence.
*
*  Permission is hereby granted, free of charge, to any person obtaining a copy
*  of this software and associated documentation files (the "Software"), to deal
*  in the Software without restriction, including without limitation the rights
*  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
*  copies of the Software, and to permit persons to whom the Software is
*  furnished to do so, subject to the following conditions:
*
*  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
*  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
*  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
*  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
*  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
*  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
*  THE SOFTWARE.
*/

#include <osgText/Text>
#include <osg/Geode>
#include <osg/Group>
#include <osg/MatrixTransform>

#include <osgDB/ReadFile>
#include <osgDB/FileUtils>
#include <osgDB/FileNameUtils>

#include <osgGA/StateSetManipulator>

#include <osgViewer/Viewer>
#include <osgViewer/ViewerEventHandlers>

#include <iostream>

#include <osgJuniper/PointCloud>

using namespace osgJuniper;

template<typename T>
class FindTopMostNodeOfTypeVisitor : public osg::NodeVisitor
{
public:
    FindTopMostNodeOfTypeVisitor():
      osg::NodeVisitor(osg::NodeVisitor::TRAVERSE_ALL_CHILDREN),
          _foundNode(0)
      {}

      void apply(osg::Node& node)
      {
          T* result = dynamic_cast<T*>(&node);
          if (result)
          {
              _foundNode = result;
          }
          else
          {
              traverse(node);
          }
      }

      T* _foundNode;
};

template<typename T>
T* findTopMostNodeOfType(osg::Node* node)
{
    if (!node) return 0;

    FindTopMostNodeOfTypeVisitor<T> fnotv;
    fnotv.setTraversalMode(osg::NodeVisitor::TRAVERSE_ALL_CHILDREN);
    node->accept(fnotv);

    return fnotv._foundNode;
}    

class PointCloudEventHandler : public osgGA::GUIEventHandler
{
public:
    PointCloudEventHandler(PointCloud* pc)
    {
        _pc = pc;
    }

    // handle keydown events
    virtual bool handle(const osgGA::GUIEventAdapter& ea,
                        osgGA::GUIActionAdapter&)
    {
        if (ea.getEventType() == osgGA::GUIEventAdapter::KEYDOWN) {
            switch (ea.getKey()) {
            case 'n':
                {     
                    //_pc->setQuality( _pc->getQuality() + 0.01);
                }
                break;
            case 'p':
                {
                    //_pc->setQuality( _pc->getQuality() - 0.01);
                }
                break;
            default:
                break;
            }
        }

        return false;
    }

private:
    osg::ref_ptr< PointCloud > _pc;
};


int main( int argc, char **argv )
{
    // use an ArgumentParser object to manage the program arguments.
    osg::ArgumentParser arguments(&argc,argv);
   
    // construct the viewer.
    osgViewer::Viewer viewer(arguments);

    osg::ref_ptr< osg::Node > loadedModel = osgDB::readNodeFiles( arguments );

    // add model to viewer.
    viewer.setSceneData(loadedModel.get());

    // add the state manipulator
    viewer.addEventHandler( new osgGA::StateSetManipulator(viewer.getCamera()->getOrCreateStateSet()) );

    // add the window size toggle handler
    viewer.addEventHandler(new osgViewer::WindowSizeHandler);

    // add the stats handler
    viewer.addEventHandler(new osgViewer::StatsHandler);

    PointCloud* pc = findTopMostNodeOfType<PointCloud>(loadedModel.get());

    if (pc)
    {
        // add event handler to control sequence
        viewer.addEventHandler(new PointCloudEventHandler(pc));
    }

    return viewer.run();
}
