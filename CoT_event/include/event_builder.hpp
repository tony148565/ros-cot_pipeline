// event_builder.hpp
#pragma once
#include <string>
#include "cot_info.hpp"

class EventBuilder
{
public:
  EventBuilder(
    const CoTInfo & info
  );
  std::string toXML() const;

private:
  const CoTInfo & e;
};
