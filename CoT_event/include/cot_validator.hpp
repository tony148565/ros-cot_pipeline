#pragma once
#include <string>

namespace cot {

class CoTValidator {
public:
    bool validate_type(const std::string& type);
    bool validate_xml(const std::string& xml);
};

}
