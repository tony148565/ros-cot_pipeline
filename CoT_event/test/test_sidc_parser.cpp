#include <gtest/gtest.h>
#include "sidc_parser.hpp"
#include "sidc_validator.hpp" 

TEST(SIDCParserTest, SIDCTooShort) {
    std::string sidc = "S";
    try {
        sidc::sidc_to_cot_type(sidc, true);
        FAIL() << "Expected std::runtime_error";
    } catch (const std::runtime_error& e) {
        EXPECT_STREQ(e.what(), "SIDC too short");
    } catch (...) {
        FAIL() << "Expected std::runtime_error";
    }
}

TEST(SIDCParserTest, SIDCTooLong) {
    std::string sidc = "SFAPMF--------LONG";
    try {
        sidc::sidc_to_cot_type(sidc, true);
        FAIL() << "Expected std::runtime_error";
    } catch (const std::runtime_error& e) {
        EXPECT_STREQ(e.what(), "SIDC too long");
    } catch (...) {
        FAIL() << "Expected std::runtime_error";
    }
}

TEST(SIDCParserTest, SIDCIllegalChar) {
    std::string sidc = "SFAPMF!@#-----";
    try {
        sidc::sidc_to_cot_type(sidc, true);
        FAIL() << "Expected std::runtime_error";
    } catch (const std::runtime_error& e) {
        EXPECT_STREQ(e.what(), "illegal character");
    } catch (...) {
        FAIL() << "Expected std::runtime_error";
    }
}

TEST(SIDCParserTest, UnknownDimension) {
    // 例：第三碼(Battle Dimension)是未支援的字元
    std::string sidc = "SFZPMF--------";
    try {
        sidc::sidc_to_cot_type(sidc, true);
        FAIL() << "Expected std::runtime_error";
    } catch (const std::runtime_error& e) {
        std::cout << "[DEBUG] Exception caught: " << e.what() << std::endl;
        EXPECT_TRUE(std::string(e.what()).find("Unknown dimension") != std::string::npos);
    } catch (...) {
        FAIL() << "Expected std::runtime_error";
    }
}

TEST(SIDCParserTest, NoMatchingPattern) {
    // 製造一個存在合法 battle dimension，但 function id 查不到的 sidc
    std::string sidc = "SFAPMZ--------";
    sidc::load_function_index("resources/valid_types.json");

    try {
        std::string h = sidc::sidc_to_cot_type(sidc, true);
        std::cout << h << std::endl;
        FAIL() << "Expected std::runtime_error";
    } catch (const std::runtime_error& e) {
        std::cout << "[DEBUG] Exception caught: " << e.what() << std::endl;
        EXPECT_STREQ(e.what(), "No matching pattern found for SIDC");
    } catch(const std::exception& e) {
        std::cout << "Caught exception (rethrow): " << e.what() << std::endl;
    } catch(...) {
        std::cout << "Caught non-std exception!" << std::endl;
    }
}
