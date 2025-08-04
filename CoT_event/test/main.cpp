#include <iostream>
#include <string>
#include <vector>

#include "cot_info.hpp"
//#include "cot_builder.hpp"
#include "cot_validator.hpp"
#include "cottypeinfo.hpp"
#include "event_builder.hpp"
#include "sidc_parser.hpp"     // sidc::sidc_to_cot_type()
#include "sidc_validator.hpp"  // sidc::load_function_index(), sidc::is_valid_sidc()

int main()
{
  // 載入新版 Function Index（整合 type 驗證與解析）
  sidc::load_function_index("CoT_event/resources/valid_types.json");

  //  測試用 SIDC 清單（可擴充）
  std::vector<std::string> sidc_list = {
    "SFAPMF--------",    // → a-f-A-MF----
    "SFAPMF------*",     // → fallback 測試（若 function id 沒 match）
    "S---X-------",      // → invalid：非法結構
    "S",                 // → invalid：長度不足
    "SFAP------",        // → 應匹配到 "------"
    "SFAPM-----",        // → 應匹配到 "M-----"
    "SFAPMFB---",        // → 應匹配到 "MFB---"
    "SFAPMFF---",        // → 應匹配到 "MFF---"
    "SFAXXX----"         // → 無合法匹配（無 fallback）
  };

  // CoTBuilder builder;
  cot::CoTValidator validator;

  for (const auto & sidc_str : sidc_list) {
    std::cout << "============================\n";
    std::cout << "[SIDC] " << sidc_str << "\n";

    try {
      //  從 SIDC 推導出 CoT type
      CoTTypeInfo typeinfo = sidc::make_cot_type_info(sidc_str);
      std::cout << "[Type] " << typeinfo.type << "\n";

      //  組裝 CoTInfo 並轉為 XML
      CoTInfo info("UAV001", typeinfo);
      info.set_parent_infotmation("command-001", "007");
      std::string xml = EventBuilder(info).toXML();
      std::cout << "[XML]\n" << xml << "\n";

      //  驗證 XML 結構
      if (!validator.validate_xml(xml)) {
        std::cerr << "X XML 格式驗證失敗\n";
        continue;
      }

      std::cout << "O 測試通過\n";

    } catch (const std::exception & e) {
      std::cerr << "X 發生例外錯誤: " << e.what() << "\n";
    }
  }

  return 0;
}
