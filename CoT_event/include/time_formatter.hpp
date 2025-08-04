// time_formatter.hpp
#pragma once
#include <chrono>
#include <string>

namespace cot
{
std::string format_time_utc(
  const std::chrono::time_point<std::chrono::system_clock> & tp
);
}
