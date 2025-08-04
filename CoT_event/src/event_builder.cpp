// event_builder.cpp
#include "event_builder.hpp"
#include <sstream>
#include "detail_builder.hpp"
#include "point_block.hpp"
#include "time_formatter.hpp"

EventBuilder::EventBuilder(const CoTInfo & info)
: e(info)
{
}

std::string EventBuilder::toXML() const
{
  std::ostringstream oss;
  oss << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n";

  oss << "<event version=\"2.0\""
      << " uid=\"" << e.uid << "\""
      << " type=\"" << e.typeinfo.type << "\""
      << " time=\"" << cot::format_time_utc(e.time) << "\""
      << " start=\"" << cot::format_time_utc(e.start_time) << "\""
      << " stale=\"" << cot::format_time_utc(e.stale_time) << "\""
      << " how=\"" << e.how << "\">\n";

  PointBlock pt{e.lat, e.lon, e.hae, e.ce, e.le};
  oss << pt.toXML();
  oss << DetailBuilder(e).toXML();
  oss << "</event>";
  return oss.str();
}
