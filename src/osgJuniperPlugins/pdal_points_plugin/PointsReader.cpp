#include "PointsReader.hpp"
#include <pdal/pdal_macros.hpp>
#include <pdal/util/ProgramArgs.hpp>

using namespace osgJuniper;

namespace pdal
{
  static PluginInfo const s_info = PluginInfo(
    "readers.points",
    "points reader.",
    "" );

  CREATE_SHARED_PLUGIN(1, 0, PDALPointsReader, Reader, s_info)

  std::string PDALPointsReader::getName() const { return s_info.name; }

  void PDALPointsReader::addArgs(ProgramArgs& args)
  {
  }

  void PDALPointsReader::addDimensions(PointLayoutPtr layout)
  {
    layout->registerDim(Dimension::Id::X);
    layout->registerDim(Dimension::Id::Y);
    layout->registerDim(Dimension::Id::Z);    
	layout->registerDim(Dimension::Id::Red);
	layout->registerDim(Dimension::Id::Green);
	layout->registerDim(Dimension::Id::Blue);
  }

  Dimension::IdList PDALPointsReader::getDefaultDimensions()
  {
    Dimension::IdList ids;

    ids.push_back(Dimension::Id::X);
    ids.push_back(Dimension::Id::Y);
    ids.push_back(Dimension::Id::Z);
	ids.push_back(Dimension::Id::Red);
	ids.push_back(Dimension::Id::Green);
	ids.push_back(Dimension::Id::Blue);

    return ids;
  }

  void PDALPointsReader::ready(PointTableRef)
  {
	  _reader = new osgJuniper::PointReader(m_filename);
  }

  point_count_t PDALPointsReader::read(PointViewPtr view, point_count_t count)
  {
    PointLayoutPtr layout = view->layout();
    PointId nextId = view->size();
    PointId idx = m_index;
    point_count_t numRead = 0;

	
	while (_reader->hasMore() && numRead < count)
	{
		osgJuniper::Point point;
		if (_reader->read(point))
		{
			view->setField(Dimension::Id::X, nextId, point.x);
			view->setField(Dimension::Id::Y, nextId, point.y);
			view->setField(Dimension::Id::Z, nextId, point.z);
			view->setField(Dimension::Id::Red, nextId, point.r);
			view->setField(Dimension::Id::Green, nextId, point.g);
			view->setField(Dimension::Id::Blue, nextId, point.b);
			nextId++;
			numRead++;
		}		
	}
	m_index = nextId;
	numRead = nextId;
	return numRead;	
  }

  bool PDALPointsReader::processOne(PointRef& point)
  {
	  osgJuniper::Point pt;
	  if (_reader->read(pt))
	  {		  
		  point.setField(Dimension::Id::X, pt.x);
		  point.setField(Dimension::Id::Y, pt.y);
		  point.setField(Dimension::Id::Z, pt.z);
		  point.setField(Dimension::Id::Red, pt.r);
		  point.setField(Dimension::Id::Green, pt.g);
		  point.setField(Dimension::Id::Blue, pt.b);
		  return true;
	  }
	  return false;
  }

  void PDALPointsReader::done(PointTableRef)
  {
	  if (_reader) delete _reader;
  }

} //namespace pdal
