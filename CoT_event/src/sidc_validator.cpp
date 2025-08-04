#include "sidc_validator.hpp"
#include <fstream>
#include <iostream>
#include <nlohmann/json.hpp>
#include <sstream>

namespace sidc
{

//  正確定義（與 header 一致）
std::unordered_map<std::string, std::unordered_map<std::string, FunctionEntry>> function_index;

//  count_specific_match 可放這或放公共工具函式
int count_specific_match(const std::string & pattern, const std::string & func)
{
  int score = 0;

  if (pattern.size() != 6 || func.size() != 6) {
    throw std::invalid_argument("Pattern or function ID length must be 6");
  }

  for (size_t i = 0; i < 6; ++i) {
    if (pattern[i] == '-') {
      continue;
    }
    if (pattern[i] != func[i]) {
      return -1;
    }
    ++score;
  }
  return score;
}

// 正常操作 function_index
void load_function_index(const std::string & filepath)
{
  std::ifstream file(filepath);
  if (!file.is_open()) {
    throw std::runtime_error("Cannot open function index: " + filepath);
  }

  nlohmann::json json_data;
  file >> json_data;

  function_index.clear();
  for (auto & [dim, functions] : json_data.items()) {
    for (auto & [pattern, entry] : functions.items()) {
      FunctionEntry e;
      e.name = entry["name"].get<std::string>();
      e.hierarchy = entry["hierarchy"].get<std::string>();
      e.affiliation = entry["affiliation"].get<std::string>();
      function_index[dim][pattern] = e;
    }
  }

  std::cout << " Loaded function index. Dimension count: " << function_index.size() << std::endl;
}

bool is_valid_sidc(const std::string & sidc)
{
  if (sidc.length() < 10) {
    return false;
  }

  char aff = sidc[1];
  char dim = sidc[2];
  std::string func = sidc.substr(4, 6);

  auto dim_it = function_index.find(std::string(1, dim));
  if (dim_it == function_index.end()) {
    return false;
  }

  for (const auto & [pattern, entry] : dim_it->second) {
    int score = count_specific_match(pattern, func);
    if (score >= 0 &&
      (entry.affiliation == "*" || entry.affiliation.find(aff) != std::string::npos))
    {
      return true;
    }
  }

  return false;
}

std::string to_cot_type(const std::string & pattern)
{
  std::string cot_type;

  for (char c : pattern) {
    if (c == '-') {
      continue;              // 忽略補位符
    }
    cot_type += std::tolower(c);
    cot_type += '-';
  }

  if (!cot_type.empty() && cot_type.back() == '-') {
    cot_type.pop_back();  // 移除尾端 -
  }

  return cot_type;
}

bool contains_illegal_sidc_chars(const std::string & sidc)
{
  for (char c : sidc) {
    if (!(std::isupper(c) || c == '-')) {
      return true;  // 有非法字元
    }
  }
  return false;
}

}  // namespace sidc
