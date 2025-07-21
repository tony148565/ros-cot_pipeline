#pragma once
#include <iostream>
#include <string>
#include <chrono>

struct CoTInfo {
    std::string uid; // 裝置id
    std::string type; // 事件類別 -> 需經過"驗證"
    std::chrono::time_point<std::chrono::system_clock> time; // 事件建立時間
    std::chrono::time_point<std::chrono::system_clock> start_time; // 事件開始時間
    std::chrono::time_point<std::chrono::system_clock> stale_time; // 事件預期時間
    
    double lat = 0.0;
    double lon = 0.0;
    double hae = 9999999.0;
    double ce = 9999999.0;
    double le = 9999999.0;


    // 建構子，在呼叫時必須先給uid, 經過驗證的type_，其餘時間相關的屬性自動生成
    CoTInfo(const std::string& uid_,
            const std::string& type_,
            std::chrono::time_point<std::chrono::system_clock> now = std::chrono::system_clock::now())
        : uid(uid_), type(type_), time(now), start_time(now),
          stale_time(now + std::chrono::seconds(300)) {}
};