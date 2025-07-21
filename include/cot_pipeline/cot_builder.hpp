#pragma once
#include "cot_info.hpp"
#include <string>

class CoTBuilder {
public:
    static std::string toXML(const CoTInfo& event);
private:
    static std::string formatTime(const std::chrono::time_point<std::chrono::system_clock>& time_point);
};
