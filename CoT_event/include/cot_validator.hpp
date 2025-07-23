#pragma once

#include <string>
#include <unordered_set>
#include "cot_info.hpp"

class CoTValidator {
public:
    CoTValidator();

    bool validate_type(const std::string& type);
    bool validate_event(const CoTInfo& info);
    bool validate_xml(const std::string& xml);

private:
    std::unordered_set<std::string> valid_types_;
};
