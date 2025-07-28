#pragma once
#include <string>

namespace cot {

class CoTValidator {
public:
    bool validate_xml(const std::string& xml);
};

}
