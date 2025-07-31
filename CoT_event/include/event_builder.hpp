// event_builder.hpp
#pragma once
#include "cot_info.hpp"
#include <string>

class EventBuilder {
public:
    EventBuilder(const CoTInfo& info);
    std::string toXML() const;

private:
    const CoTInfo& e;
};
