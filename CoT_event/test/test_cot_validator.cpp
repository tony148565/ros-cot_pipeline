#include <gtest/gtest.h>
#include "cot_validator.hpp"

TEST(CoTValidatorTest, ValidCoTEventXML) {
    std::string valid_xml = R"(<?xml version="1.0" encoding="UTF-8"?>
<event version="2.0" uid="UAV001" type="a-f-A-M-F" time="2025-08-08T12:00:00Z" start="2025-08-08T12:00:00Z" stale="2025-08-08T12:05:00Z" how="m-g">
  <point lat="0.000000" lon="0.000000" hae="9999999.000000" ce="9999999.000000" le="9999999.000000" />
  <detail>
    <contact callsign="anonymous" />
    <symbol sidc="SFAPMF--------" />
    <usericon iconsetpath="COT_MAPPING_2525B/a/a-f/a-f-A" />
    <archive />
    <groups role="Team Member" affiliation="friendly" />
  </detail>
</event>
)";
    EXPECT_TRUE(cot::CoTValidator::validate_xml(valid_xml));
}

TEST(CoTValidatorTest, InvalidXML_SyntaxError) {
    std::string invalid_xml = R"(<event version="2.0" uid="UAV001" type="a-f-A-M-F")";
    EXPECT_FALSE(cot::CoTValidator::validate_xml(invalid_xml));
}

TEST(CoTValidatorTest, InvalidXML_NoRootEvent) {
    std::string no_event_xml = R"(<foo version="2.0" uid="UAV001" type="a-f-A-M-F"></foo>)";
    EXPECT_FALSE(cot::CoTValidator::validate_xml(no_event_xml));
}

TEST(CoTValidatorTest, InvalidXML_MissingAttributes) {
    std::string missing_attr_xml = R"(<event version="2.0"></event>)";
    EXPECT_FALSE(cot::CoTValidator::validate_xml(missing_attr_xml));
}
