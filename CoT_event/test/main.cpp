#include <iostream>
#include <vector>
#include <string>
#include "cot_info.hpp"
#include "cot_builder.hpp"
#include "cot_validator.hpp"
#include "sidc_parser.hpp"      // sidc_to_cot_type()
#include "sidc_validator.hpp"   // load_function_index(), is_valid_sidc()

int main() {
    // ✅ 載入 Function Index（新版 JSON）
    cot::load_function_index("CoT_event/resources/valid_types.json");

    std::vector<std::string> sidc_list = {
        "SFAPMF--------",     // → a-f-A-MF----
        "SFAPMF------*",      // → fallback 測試（可微調）
        "S---X-------",       // → invalid
        "S",                  // → too short
        "SFAP------",        // 應匹配到 "------"
        "SFAPM-----",        // 應匹配到 "M-----"
        "SFAPMFB---",        // 應匹配到 "MFB---"
        "SFAPMFF---",        // 應匹配到 "MFF---"
        "SFAXXX----"         // 無合法匹配

    };

    CoTBuilder builder;
    CoTValidator validator;

    for (const auto& sidc : sidc_list) {
        std::cout << "============================\n";
        std::cout << "[SIDC] " << sidc << "\n";

        try {
            // ✅ 先驗證合法性
            if (!cot::is_valid_sidc(sidc)) {
                std::cerr << "❌ 非法 SIDC，跳過解析\n";
                continue;
            }

            std::string type = cot::sidc_to_cot_type(sidc);
            std::cout << "[Type] " << type << "\n";

            CoTInfo info("UAV001", type);

            // ✅ 驗證 type 格式
            if (!validator.validate_type(type)) {
                std::cerr << "❌ Invalid type: " << type << "\n";
                continue;
            }

            std::string xml = builder.toXML(info);
            std::cout << "[XML]\n" << xml << "\n";

            // ✅ 驗證 XML 格式
            if (!validator.validate_xml(xml)) {
                std::cerr << "❌ XML 格式驗證失敗\n";
                continue;
            }

            std::cout << "✅ 測試通過\n";

        } catch (const std::exception& e) {
            std::cerr << "❌ 錯誤: " << e.what() << "\n";
        }
    }

    return 0;
}
