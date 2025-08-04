#include "cot_validator.hpp"
#include <tinyxml2.h>
#include <iostream>
#include <sstream>
#include "sidc_validator.hpp"  // 為了使用 function_index 與 count_specific_match

namespace cot
{

// ✅ XML 結構驗證（使用 tinyxml2）
bool CoTValidator::validate_xml(
  const std::string & xml
)
{
  tinyxml2::XMLDocument doc;
  tinyxml2::XMLError err = doc.Parse(xml.c_str());

  if (
    err != tinyxml2::XML_SUCCESS)
  {
    std::cerr << "[CoTValidator] XML parse error: " << doc.ErrorStr() << std::endl;
    return false;
  }

  tinyxml2::XMLElement * root = doc.RootElement();
  if (
    !root || std::string(root->Name()) != "event")
  {
    std::cerr << "[CoTValidator] Root element is not <event>\n";
    return false;
  }

  if (
    !root->Attribute("type") || !root->Attribute("uid"))
  {
    std::cerr << "[CoTValidator] Missing required attributes\n";
    return false;
  }

  return true;
}

}  // namespace cot
