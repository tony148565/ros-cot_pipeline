
#include <gtest/gtest.h>
#include "sidc_validator.hpp" 

TEST(SIDCValidatorTest, CountSpecificMatch_AllMatch) {
    EXPECT_EQ(sidc::count_specific_match("ABCDEF", "ABCDEF"), 6);
}

TEST(SIDCValidatorTest, CountSpecificMatch_Wildcard) {
    EXPECT_EQ(sidc::count_specific_match("A--D-F", "AQQDQF"), 3);
}

TEST(SIDCValidatorTest, CountSpecificMatch_Mismatch) {
    EXPECT_EQ(sidc::count_specific_match("ABCDEF", "AXCDEF"), -1);
}

TEST(SIDCValidatorTest, CountSpecificMatch_LengthError) {
    EXPECT_THROW(sidc::count_specific_match("ABCDEF", "ABC"), std::invalid_argument);
}

TEST(SIDCValidatorTest, ContainsIllegalSidcChars) {
    EXPECT_FALSE(sidc::contains_illegal_sidc_chars("SFAPMF--------"));
    EXPECT_TRUE(sidc::contains_illegal_sidc_chars("SFAPMF--xx----"));
    EXPECT_TRUE(sidc::contains_illegal_sidc_chars("sfapmf--------")); // 小寫也不行
}

TEST(SIDCValidatorTest, ToCotType) {
    EXPECT_EQ(sidc::to_cot_type("MFF---"), "m-f-f");
    EXPECT_EQ(sidc::to_cot_type("X-----"), "x");
}

TEST(SIDCValidatorTest, IsValidSidc_Valid) {
    sidc::load_function_index("resources/valid_types.json");
    EXPECT_TRUE(sidc::is_valid_sidc("SFAPMF--------"));
}

TEST(SIDCValidatorTest, IsValidSidc_InvalidDimension) {
    sidc::load_function_index("resources/valid_types.json");
    EXPECT_FALSE(sidc::is_valid_sidc("SFZPMF--------")); // Z 不存在
}

TEST(SIDCValidatorTest, IsValidSidc_TooShort) {
    EXPECT_FALSE(sidc::is_valid_sidc("SFAP")); // 少於10
}

TEST(SIDCValidatorTest, IsValidSidc_TooLong) {
    EXPECT_FALSE(sidc::is_valid_sidc("SFAPMF----------")); // 超過15
}

TEST(SIDCValidatorTest, LoadFunctionIndex_NoFile) {
    EXPECT_THROW(sidc::load_function_index("resources/xxx.json"), std::runtime_error);
}


