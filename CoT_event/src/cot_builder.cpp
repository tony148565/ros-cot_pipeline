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
    oss << "  <detail>\n";

    // contact 子節點
    oss << "    <contact callsign=\"" << e.callsign << "\" />\n";

    // __group 子節點
    oss << "    <__group role=\"" << e.role << "\""
        << " affiliation=\"" << e.group_affiliation << "\" />\n";

    // source 子節點
    oss << "    <__contact source=\"" << e.source << "\" />\n";

    // sidc function id
    oss << "    <symbol sidc=\"" << e.typeinfo.sidc << "\" />\n"; 

    // remarks 子節點（可選）
    if (!e.remarks.empty()) {
        oss << "    <remarks>" << e.remarks << "</remarks>\n";
    }

    oss << "  </detail>\n";

    // event 結尾
    oss << "</event>";
    xml += oss.str();
    return xml;
}
