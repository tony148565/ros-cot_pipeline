#include "cot_pipeline/cot_builder.hpp"
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
    std::ostringstream oss;

    oss << "<event version=\"2.0\""
        << " uid=\"" << e.uid << "\""
        << " type=\"" << e.type << "\""
        << " time=\"" << formatTime(e.time) << "\""
        << " start=\"" << formatTime(e.start_time) << "\""
        << " stale=\"" << formatTime(e.stale_time) << "\">\n";

    oss << "  <point"
        << " lat=\"" << e.lat << "\""
        << " lon=\"" << e.lon << "\""
        << " hae=\"" << e.hae << "\""
        << " ce=\"" << e.ce << "\""
        << " le=\"" << e.le << "\""
        << " />\n";

    oss << "</event>";
    return oss.str();
}
