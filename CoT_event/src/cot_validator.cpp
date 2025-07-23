#include "cot_validator.hpp"
#include "sidc_validator.hpp"  // 為了使用 function_index 與 count_specific_match
#include <tinyxml2.h>
#include <sstream>
#include <iostream>

namespace cot {

// ✅ 使用新邏輯來驗證 CoT type
bool CoTValidator::validate_type(const std::string& type) {
    // 預期格式為 "a-f-A-MFB---"
    std::istringstream iss(type);
    std::string scheme, aff, dim, func;

    if (!std::getline(iss, scheme, '-') ||
        !std::getline(iss, aff, '-') ||
        !std::getline(iss, dim, '-') ||
        !std::getline(iss, func)) {
        std::cerr << "[CoTValidator] Invalid type format: " << type << std::endl;
        return false;
    }

    auto dim_it = sidc::function_index.find(dim);
    if (dim_it == sidc::function_index.end()) return false;

    for (const auto& [pattern, entry] : dim_it->second) {
        int score = sidc::count_specific_match(pattern, func);
        if (score >= 0 &&
            (entry.affiliation == "*" || entry.affiliation.find(aff) != std::string::npos)) {
            return true;
        }
    }

    return false;
}

// ✅ XML 結構驗證（使用 tinyxml2）
bool CoTValidator::validate_xml(const std::string& xml) {
    tinyxml2::XMLDocument doc;
    tinyxml2::XMLError err = doc.Parse(xml.c_str());

    if (err != tinyxml2::XML_SUCCESS) {
        std::cerr << "[CoTValidator] XML parse error: " << doc.ErrorStr() << std::endl;
        return false;
    }

    tinyxml2::XMLElement* root = doc.RootElement();
    if (!root || std::string(root->Name()) != "event") {
        std::cerr << "[CoTValidator] Root element is not <event>\n";
        return false;
    }

    if (!root->Attribute("type") || !root->Attribute("uid")) {
        std::cerr << "[CoTValidator] Missing required attributes\n";
        return false;
    }

    return true;
}

} // namespace cot
