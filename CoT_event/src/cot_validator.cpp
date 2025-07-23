#include "cot_validator.hpp"
#include <tinyxml2.h>
#include <iostream>
#include <fstream>
#include <nlohmann/json.hpp>

using json = nlohmann::json;

CoTValidator::CoTValidator() {
    std::ifstream infile("CoT_event/resources/valid_types.json");
    if (!infile.is_open()) {
        std::cerr << "[CoTValidator] Failed to open valid_types.json" << std::endl;
        return;
    }

    json j;
    infile >> j;

    if (j.contains("valid_types") && j["valid_types"].is_array()) {
        for (const auto& t : j["valid_types"]) {
            valid_types_.insert(t.get<std::string>());
        }
    } else {
        std::cerr << "[CoTValidator] JSON format error: missing valid_types array" << std::endl;
    }
}

bool CoTValidator::validate_type(const std::string& type) {
    return valid_types_.find(type) != valid_types_.end();
}

bool CoTValidator::validate_event(const CoTInfo& info) {
    return !info.uid.empty() &&
           info.start_time <= info.stale_time &&
           info.time <= info.start_time;
}

bool CoTValidator::validate_xml(const std::string& xml) {
    tinyxml2::XMLDocument doc;
    auto result = doc.Parse(xml.c_str());
    if (result != tinyxml2::XML_SUCCESS) return false;

    auto* root = doc.FirstChildElement("event");
    if (!root) return false;

    // 基本檢查：是否有 uid、type 屬性
    const char* uid = root->Attribute("uid");
    const char* type = root->Attribute("type");
    return uid && type;
}
