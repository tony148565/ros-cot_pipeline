#include <gtest/gtest.h>
#include <chrono>
#include "time_formatter.hpp"

TEST(TimeFormatterTest, ValidFormat) {
  using namespace std::chrono;
  std::tm tm{};
  tm.tm_year = 2023 - 1900;
  tm.tm_mon = 7;  // 0-based: 7 = August
  tm.tm_mday = 3;
  tm.tm_hour = 0;
  tm.tm_min = 0;
  tm.tm_sec = 0;
  tm.tm_isdst = 0;

  std::time_t t = timegm(&tm);  // 👈 這行是關鍵，強制用 UTC 解讀
  std::chrono::system_clock::time_point tp = std::chrono::system_clock::from_time_t(t);

  std::string result = cot::format_time_utc(tp);
  EXPECT_EQ(result, "2023-08-03T00:00:00Z");
}

TEST(TimeFormatterTest, CurrentTimeFormat) {
  auto now = std::chrono::system_clock::now();
  std::string formatted = cot::format_time_utc(now);

  // Should end with 'Z' and have 'T' as separator
  EXPECT_EQ(formatted.back(), 'Z');
  EXPECT_NE(formatted.find('T'), std::string::npos);
}
