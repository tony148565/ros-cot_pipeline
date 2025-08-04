#pragma once
#include <string>

struct CoTTypeInfo
{
  std::string type;  // 轉換後的 CoT Type，如 "a-f-G-U-C"
  std::string sidc;  // 原始 15 碼 SIDC，如 "SFGPUCI--------"
};
