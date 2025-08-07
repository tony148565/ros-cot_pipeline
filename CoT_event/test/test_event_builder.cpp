#include <gtest/gtest.h>
#include "event_builder.hpp"
#include "cot_info.hpp"

TEST(EventBuilderTest, MinimalXMLStructure) {
    CoTInfo info;
    info.uid = "TEST01";
    info.typeinfo.type = "a-f-A-M-F";
    info.time = std::chrono::system_clock::from_time_t(1723468800);
    info.start_time = info.time;
    info.stale_time = info.time + std::chrono::minutes(5);
    info.how = "m-g";
    info.lat = 0.0;
    info.lon = 0.0;
    info.hae = 9999999.0;
    info.ce = 9999999.0;
    info.le = 9999999.0;

    EventBuilder builder(info);
    std::string xml = builder.toXML();

    // 基本結構
    EXPECT_TRUE(xml.find("<?xml version=\"1.0\" encoding=\"UTF-8\"?>") == 0);
    EXPECT_NE(xml.find("<event "), std::string::npos);
    EXPECT_NE(xml.find("uid=\"TEST01\""), std::string::npos);
    EXPECT_NE(xml.find("type=\"a-f-A-M-F\""), std::string::npos);
    EXPECT_NE(xml.find("how=\"m-g\""), std::string::npos);
    EXPECT_NE(xml.find("<point "), std::string::npos);
    EXPECT_NE(xml.find("<detail>"), std::string::npos);  // 或你 detail 的實際起始
    EXPECT_NE(xml.find("</event>"), std::string::npos);
}
