#include "cot_builder.hpp"
#include <sstream>
#include <iomanip>
#include <ctime>

std::string CoTBuilder::formatTime(const std::chrono::time_point<std::chrono::system_clock>& tp) {
    std::time_t t = std::chrono::system_clock::to_time_t(tp);
    std::tm* gmt = std::gmtime(&t);
    char buffer[32];
    std::strftime(buffer, sizeof(buffer), "%Y-%m-%dT%H:%M:%SZ", gmt);
    return std::string(buffer);
}

std::string CoTBuilder::build_detail(const CoTInfo& e) {
    std::ostringstream oss;
    oss << "  <detail>\n";

    // ✅ 必要欄位：callsign
    oss << "    <contact callsign=\"" << e.callsign << "\" />\n";

    // ✅ sidc（簡短格式 WinTAK 實測可接受）
    oss << "    <symbol sidc=\"" << e.typeinfo.sidc << "\" />\n";

    // ✅ TAK 可接受的 usericon path（深度不超過 3）
    std::string icon_path = "COT_MAPPING_2525B";
    {
        std::stringstream ss(e.typeinfo.type);
        std::string segment;
        std::string prefix;
        int depth = 0;
        const int max_depth = 3;
        while (std::getline(ss, segment, '-') && depth < max_depth) {
            if (!prefix.empty()) prefix += "-";
            prefix += segment;
            icon_path += "/" + prefix;
            ++depth;
        }
    }
    oss << "    <usericon iconsetpath=\"" << icon_path << "\" />\n";

    // ✅ 合法 link 結構（需關閉且屬性完整）
    if (e.parent_uid.has_value()) {
        oss << "    <link type=\"" << e.parent_type << "\""
            << " uid=\"" << e.parent_uid.value() << "\"";
        if (e.parent_callsign.has_value()) {
            oss << " parent_callsign=\"" << e.parent_callsign.value() << "\"";
        }
        oss << " relation=\"p-p\""
            << " production_time=\"" << formatTime(e.start_time) << "\""
            << " />\n";
    }

    // ✅ archive 結尾，不會被忽略
    oss << "    <archive />\n";

    // ✅ 可選項目：sourceinfo 替代 __contact（避免衝突）
    if (!e.source.empty()) {
        oss << "    <sourceinfo source=\"" << e.source << "\" />\n";
    }

    // ✅ 可選項目：groups 替代 __group（避免衝突）
    oss << "    <groups role=\"" << e.role
        << "\" affiliation=\"" << e.group_affiliation << "\" />\n";

    // ✅ 可選項目：remarks
    if (!e.remarks.empty()) {
        oss << "    <remarks>" << e.remarks << "</remarks>\n";
    }

    oss << "  </detail>\n";
    return oss.str();
}



std::string CoTBuilder::toXML(const CoTInfo& e) {
    std::string xml = "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n";
    std::ostringstream oss;

    // event 標籤
    oss << "<event version=\"2.0\""
        << " uid=\"" << e.uid << "\""
        << " type=\"" << e.typeinfo.type << "\""
        << " time=\"" << formatTime(e.time) << "\""
        << " start=\"" << formatTime(e.start_time) << "\""
        << " stale=\"" << formatTime(e.stale_time) << "\""
        << " how=\"" << e.how << "\">\n";

    // 固定小數點表示法
    oss << std::fixed << std::setprecision(6);
    // point 標籤
    oss << "  <point"
        << " lat=\"" << e.lat << "\""
        << " lon=\"" << e.lon << "\""
        << " hae=\"" << e.hae << "\""
        << " ce=\"" << e.ce << "\""
        << " le=\"" << e.le << "\""
        << " />\n";

    // detail 區塊
    oss << build_detail(e);

    // event 結尾
    oss << "</event>";
    xml += oss.str();
    return xml;
}




