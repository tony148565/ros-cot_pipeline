#pragma once
#include <string>

namespace cot
{

class CoTValidator
{
public:
  static bool validate_xml(const std::string & xml);
};

}  // namespace cot
