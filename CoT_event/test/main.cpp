#include <iostream>
#include <vector>
#include <string>

#include "cot_info.hpp"
#include "cot_builder.hpp"
#include "cot_validator.hpp"
#include "sidc_parser.hpp"      // sidc::sidc_to_cot_type()
#include "sidc_validator.hpp"   // sidc::load_function_index(), sidc::is_valid_sidc()

int main() {
    // 載入新版 Function Index（整合 type 驗證與解析）
    sidc::load_function_index("CoT_event/resources/valid_types.json");

    //  測試用 SIDC 清單（可擴充）
    std::vector<std::string> sidc_list = {
        "SFAPMF--------",     // → a-f-A-MF----
        "SFAPMF------*",      // → fallback 測試（若 function id 沒 match）
        "S---X-------",       // → invalid：非法結構
        "S",                  // → invalid：長度不足
        "SFAP------",         // → 應匹配到 "------"
        "SFAPM-----",         // → 應匹配到 "M-----"
        "SFAPMFB---",         // → 應匹配到 "MFB---"
        "SFAPMFF---",         // → 應匹配到 "MFF---"
        "SFAXXX----"          // → 無合法匹配（無 fallback）
    };

    CoTBuilder builder;
    cot::CoTValidator validator;

    for (const auto& sidc_str : sidc_list) {
        std::cout << "============================\n";
        std::cout << "[SIDC] " << sidc_str << "\n";

        try {
            //  驗證 SIDC 合法性（由 function_index 驗證）
            if (!sidc::is_valid_sidc(sidc_str)) {
                std::cerr << "X 非法 SIDC，跳過解析\n";
                continue;
            }

            //  從 SIDC 推導出 CoT type
            std::string type = sidc::sidc_to_cot_type(sidc_str);
            std::cout << "[Type] " << type << "\n";

            //  組裝 CoTInfo 並轉為 XML
            CoTInfo info("UAV001", type);
            if (!validator.validate_type(type)) {
                std::cerr << "X 無效 Type，驗證失敗\n";
                continue;
            }

            std::string xml = builder.toXML(info);
            std::cout << "[XML]\n" << xml << "\n";

            //  驗證 XML 結構
            if (!validator.validate_xml(xml)) {
                std::cerr << "X XML 格式驗證失敗\n";
                continue;
            }

            std::cout << "O 測試通過\n";

        } catch (const std::exception& e) {
            std::cerr << "X 發生例外錯誤: " << e.what() << "\n";
        }
    }

    return 0;
}
