#pragma once
#include <string>
#include "cottypeinfo.hpp"

namespace sidc
{
std::string sidc_to_cot_type(const std::string & sidc, bool strict = false);
CoTTypeInfo make_cot_type_info(const std::string & sidc_str);
}  // namespace sidc
