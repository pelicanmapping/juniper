#pragma once

#include <pdal/PointView.hpp>
#include <pdal/Reader.hpp>
#include <pdal/util/IStream.hpp>

#include <osgJuniper/PointReaderWriter>

#include <iostream>>

namespace pdal
{
  class PDALPointsReader : public Reader
  {
  public:
	  PDALPointsReader() :
		Reader(),
		m_index(0),
		_reader(0)
	{
	};

    static void * create();
    static int32_t destroy(void *);
    std::string getName() const;

    static Dimension::IdList getDefaultDimensions();

  private:
	std::ifstream m_stream;
    point_count_t m_index;

    virtual void addDimensions(PointLayoutPtr layout);
    virtual void addArgs(ProgramArgs& args);
    virtual void ready(PointTableRef table);
    virtual point_count_t read(PointViewPtr view, point_count_t count);
	bool processOne(PointRef& point);
    virtual void done(PointTableRef table);

	// TODO:  Delete
	osgJuniper::PointReader* _reader;
  };
}
