#include "sidc_parser.hpp"
#include "sidc_validator.hpp"  // 使用共享的 function_index
#include <stdexcept>
#include <string>
#include <cctype>

// 根據以下網站的觀察
    // 1. "https://freetakteam.github.io/FreeTAKServer-User-Docs/About/architecture/mil_std_2525/"
    // 2. "https://github.com/spatialillusions/mil-std-2525/blob/master/tsv-tables"
    // 得出結論(?)
    // 1. SIDC至少有4碼，codingscheme(會被"a"取代)、affiliation(大寫應轉小寫)、battledimension與status
    // 2. 5-10碼是funcion id
    // 3. SIDC 轉 CoT type 時應保留affiliation、battledimension與funcion id

namespace sidc {

std::string sidc_to_cot_type(const std::string& sidc) {
    if (sidc.length() < 10) {
        throw std::runtime_error("SIDC too short");
    }

    char aff = std::tolower(sidc[1]);  // 強制轉小寫
    char dim = sidc[2];
    std::string func = sidc.substr(4, 6);  // 5~10碼為 function id

    auto dim_it = function_index.find(std::string(1, dim));
    if (dim_it == function_index.end()) {
        throw std::runtime_error("Unknown dimension: " + std::string(1, dim));
    }

    int best_score = -1;
    std::string best_pattern;

    for (const auto& [pattern, entry] : dim_it->second) {
        int score = count_specific_match(pattern, func);

        if (score > best_score &&
            (entry.affiliation == "*" || entry.affiliation.find(aff) != std::string::npos)) {
            best_score = score;
            best_pattern = pattern;
        }
    }

    // 僅接受「嚴格匹配」的 pattern（score >= 1）
    if (best_score > 0) {
        return "a-" + std::string(1, aff) + "-" + std::string(1, dim) + "-" + best_pattern;
    }

    // 特例處理：允許精確 fallback match "------"
    auto fallback_it = dim_it->second.find("------");
    if (fallback_it != dim_it->second.end()) {
        if (func == "------" &&
            (fallback_it->second.affiliation == "*" || fallback_it->second.affiliation.find(aff) != std::string::npos)) {
            return "a-" + std::string(1, aff) + "-" + std::string(1, dim) + "-" + "------";
        }
    }

    throw std::runtime_error("No matching pattern found for SIDC");
}

} // namespace sidc