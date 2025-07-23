#include "sidc_parser.hpp"
#include "sidc_validator.hpp"  // 使用共享的 function_index
#include <stdexcept>
#include <string>

// 根據以下網站的觀察
    // 1. "https://freetakteam.github.io/FreeTAKServer-User-Docs/About/architecture/mil_std_2525/"
    // 2. "https://github.com/spatialillusions/mil-std-2525/blob/master/tsv-tables"
    // 得出結論(?)
    // 1. SIDC至少有4碼，codingscheme(會被"A"取代)、affiliation、battledimension與status
    // 2. 5-10碼是funcion id

namespace cot {

std::string sidc_to_cot_type(const std::string& sidc) {
    if (sidc.length() < 10) {
        throw std::runtime_error("SIDC too short");
    }

    char aff = sidc[1];                             // affiliation
    char dim = sidc[2];                             // battle dimension
    std::string func = sidc.substr(4, 6);           // function ID
    const std::string fallback_pattern = "------";  // 最寬鬆匹配

    auto dim_it = function_index.find(std::string(1, dim));
    if (dim_it == function_index.end()) {
        throw std::runtime_error("Unknown dimension");
    }

    int best_score = -1;
    std::string best_pattern;
    bool has_fallback = false;

    for (const auto& [pattern, entry] : dim_it->second) {
        int score = count_specific_match(pattern, func);

        if (score >= 0 &&
            (entry.affiliation == "*" || entry.affiliation.find(aff) != std::string::npos)) {

            if (pattern == fallback_pattern) {
                has_fallback = true;
                continue;
            }

            if (score > best_score) {
                best_score = score;
                best_pattern = pattern;
            }
        }
    }

    if (best_score >= 0) {
        return "a-" + std::string(1, aff) + "-" + std::string(1, dim) + "-" + best_pattern;
    }

    if (has_fallback) {
        return "a-" + std::string(1, aff) + "-" + std::string(1, dim) + "-" + fallback_pattern;
    }

    throw std::runtime_error("No matching pattern found for SIDC");
}

} // namespace cot
