#include <gtest/gtest.h>
#include "your_sidc_header.hpp"  // 包含 sidc_to_cot_type()

TEST(SIDCParserTest, BasicMapping)
{
    std::string sidc = "SFAPMF--------";
    std::string expected = "a-f-A-M-F";
    EXPECT_EQ(sidc_to_cot_type(sidc), expected);
}

TEST(SIDCParserTest, NoFunctionID)
{
    std::string sidc = "SFAP------*****";
    std::string expected = "a-f-A";
    EXPECT_EQ(sidc_to_cot_type(sidc), expected);
}

TEST(SIDCParserTest, InvalidSIDC)
{
    std::string sidc = "S";
    EXPECT_THROW(sidc_to_cot_type(sidc), std::invalid_argument);
}
