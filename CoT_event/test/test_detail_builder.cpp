#include <gtest/gtest.h>
#include "detail_builder.hpp"
#include "cot_info.hpp" // 假設 CoTInfo 定義在這

TEST(DetailBuilderTest, BasicXMLStructure) {
    CoTInfo info;
    info.typeinfo.type = "a-f-A-M-F";
    info.typeinfo.sidc = "SFAPMF--------";
    info.callsign = "Alpha";
    info.start_time = std::chrono::system_clock::from_time_t(1723468800);
    info.role = "Team Member";
    info.group_affiliation = "friendly";
    info.remarks = "test remarks";
    info.source = "testsrc";
    // 不加 parent_uid, parent_callsign

    DetailBuilder builder(info);
    std::string xml = builder.toXML();

    // 確認基本結構
    EXPECT_NE(xml.find("<contact callsign=\"Alpha\""), std::string::npos);
    EXPECT_NE(xml.find("<symbol sidc=\"SFAPMF--------\""), std::string::npos);
    EXPECT_NE(xml.find("<usericon iconsetpath=\"COT_MAPPING_2525B/a/a-f/a-f-A\""), std::string::npos); // 根據 build_icon_path
    EXPECT_NE(xml.find("<groups role=\"Team Member\" affiliation=\"friendly\""), std::string::npos);
    EXPECT_NE(xml.find("<remarks>test remarks</remarks>"), std::string::npos);
    EXPECT_NE(xml.find("<sourceinfo source=\"testsrc\""), std::string::npos);

    // 不加 parent_uid/parent_callsign 不該出現 <link
    EXPECT_EQ(xml.find("<link "), std::string::npos);
}

TEST(DetailBuilderTest, ParentLinkAndOptionalFields) {
    CoTInfo info;
    info.typeinfo.type = "a-f-A-M-F";
    info.typeinfo.sidc = "SFAPMF--------";
    info.callsign = "Alpha";
    info.start_time = std::chrono::system_clock::from_time_t(1723468800);
    info.role = "Team Member";
    info.group_affiliation = "friendly";
    info.parent_uid = "PARENT01";
    info.parent_type = "a-f-A";
    info.parent_callsign = "ParentAlpha";
    info.source = "";
    info.remarks = "";

    DetailBuilder builder(info);
    std::string xml = builder.toXML();

    // 應有 <link ... parent_callsign=
    EXPECT_NE(xml.find("uid=\"PARENT01\""), std::string::npos);
    EXPECT_NE(xml.find("type=\"a-f-A\""), std::string::npos);
    EXPECT_NE(xml.find("parent_callsign=\"ParentAlpha\""), std::string::npos);
    EXPECT_NE(xml.find("relation=\"p-p\""), std::string::npos);
    EXPECT_NE(xml.find("production_time=\"2024-") /*年份即可*/, std::string::npos);

    // 不該有 <sourceinfo
    EXPECT_EQ(xml.find("<sourceinfo"), std::string::npos);
    // 不該有 <remarks>
    EXPECT_EQ(xml.find("<remarks>"), std::string::npos);
}
