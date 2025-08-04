// point_block.hpp
#pragma once
#include <iomanip>
#include <sstream>
#include <string>

struct PointBlock
{
  double lat, lon, hae, ce, le;

  std::string toXML() const
  {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(6) << "  <point"
        << " lat=\"" << lat << "\""
        << " lon=\"" << lon << "\""
        << " hae=\"" << hae << "\""
        << " ce=\"" << ce << "\""
        << " le=\"" << le << "\""
        << " />\n";
    return oss.str();
  }
};
