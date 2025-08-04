#pragma once
#include <string>
#include <unordered_map>

namespace sidc
{

// 每個 function pattern 對應的資料
struct FunctionEntry
{
  std::string name;
  std::string hierarchy;
  std::string affiliation;
};

// 外部可存取的 function_index（parser、validator 共用）
extern std::unordered_map<std::string,
  std::unordered_map<std::string, FunctionEntry>> function_index;

// API
void load_function_index(
  const std::string & filepath
);
bool is_valid_sidc(
  const std::string & sidc
);
int count_specific_match(
  const std::string & pattern, const std::string & func
);
std::string to_cot_type(
  const std::string & pattern
);
bool contains_illegal_sidc_chars(
  const std::string & sidc
);
}  // namespace sidc
