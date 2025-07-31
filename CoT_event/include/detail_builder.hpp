#pragma once
#include "cot_info.hpp"
#include <string>
#include "time_formatter.hpp"

class DetailBuilder {
public:
    DetailBuilder(const CoTInfo& info);
    std::string toXML() const;

private:
    const CoTInfo& e;
    std::string build_icon_path() const;
};
