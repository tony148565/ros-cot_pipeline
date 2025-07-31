#include "detail_builder.hpp"
#include <sstream>

DetailBuilder::DetailBuilder(const CoTInfo& info) : e(info) {}

std::string DetailBuilder::build_icon_path() const {
    std::stringstream ss(e.typeinfo.type);
    std::string segment;
    std::string prefix;
    std::string icon_path = "COT_MAPPING_2525B";
    int depth = 0;
    while (std::getline(ss, segment, '-') && depth < 3) {
        if (!prefix.empty()) prefix += "-";
        prefix += segment;
        icon_path += "/" + prefix;
        ++depth;
    }
    return icon_path;
}

std::string DetailBuilder::toXML() const {
    std::ostringstream oss;
    oss << "  <detail>\n";
    oss << "    <contact callsign=\"" << e.callsign << "\" />\n";
    oss << "    <symbol sidc=\"" << e.typeinfo.sidc << "\" />\n";
    oss << "    <usericon iconsetpath=\"" << build_icon_path() << "\" />\n";

    if (e.parent_uid.has_value()) {
        oss << "    <link type=\"" << e.parent_type << "\""
            << " uid=\"" << e.parent_uid.value() << "\"";
        if (e.parent_callsign.has_value()) {
            oss << " parent_callsign=\"" << e.parent_callsign.value() << "\"";
        }
        oss << " relation=\"p-p\""
            << " production_time=\"" << cot::format_time_utc(e.start_time) << "\""
            << " />\n";
    }

    oss << "    <archive />\n";

    if (!e.source.empty()) {
        oss << "    <sourceinfo source=\"" << e.source << "\" />\n";
    }

    oss << "    <groups role=\"" << e.role
        << "\" affiliation=\"" << e.group_affiliation << "\" />\n";

    if (!e.remarks.empty()) {
        oss << "    <remarks>" << e.remarks << "</remarks>\n";
    }

    oss << "  </detail>\n";
    return oss.str();
}
