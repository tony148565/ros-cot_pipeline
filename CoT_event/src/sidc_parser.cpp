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
    // SIDC編碼:
    // Scheme - Affiliation - Battle Dimension - Status - Function ID(5-10) - Symbol Modifier(11, 12) - Country Code(13, 14) - Order of Battle

namespace sidc {

std::string sidc_to_cot_type(const std::string& sidc) {
    if (sidc.length() < 10) {
        throw std::runtime_error("SIDC too short");
    }
    else if (sidc.length() > 15){
        throw std::runtime_error("SIDC too long");
    }
    else if (contains_illegal_sidc_chars(sidc) || !sidc::is_valid_sidc(sidc)){
        throw std::runtime_error("illegal character");
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

    // 特例處理：允許精確 fallback match "------"
    if (func == "------") {
        auto fallback_it = dim_it->second.find("------");
        if (fallback_it != dim_it->second.end() &&
            (fallback_it->second.affiliation == "*" || fallback_it->second.affiliation.find(aff) != std::string::npos)) {
            return "a-" + std::string(1, aff) + "-" + std::string(1, dim);
        }
    }

    // 主要判斷區塊
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
        // e.g. "MFF---" → "m-f-f"
        //best_pattern = normalize_sidc_function_id(best_pattern);
        return "a-" + std::string(1, aff) + "-" + std::string(1, dim) + "-" + to_cot_type(best_pattern);
    }

    

    throw std::runtime_error("No matching pattern found for SIDC");
}

CoTTypeInfo make_cot_type_info(const std::string& sidc_str) {
    CoTTypeInfo info;
    info.sidc = sidc_str;
    info.type = sidc_to_cot_type(sidc_str);
    return info;
}


} // namespace sidc