// time_formatter.cpp
#include "time_formatter.hpp"
#include <ctime>
#include <iomanip>

std::string cot::format_time_utc(const std::chrono::time_point<std::chrono::system_clock> & tp)
{
  std::time_t t = std::chrono::system_clock::to_time_t(tp);
  std::tm * gmt = std::gmtime(&t);
  char buffer[32];
  std::strftime(buffer, sizeof(buffer), "%Y-%m-%dT%H:%M:%SZ", gmt);
  return std::string(buffer);
}
