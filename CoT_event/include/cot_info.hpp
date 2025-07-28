#pragma once
#include <iostream>
#include <string>
#include <chrono>

struct CoTInfo {
    // ===== 基本事件欄位 =====
    std::string uid;
    std::string type;
    std::string how;
    std::chrono::time_point<std::chrono::system_clock> time;
    std::chrono::time_point<std::chrono::system_clock> start_time;
    std::chrono::time_point<std::chrono::system_clock> stale_time;

    // ===== 附加欄位 =====
    std::string callsign = "anonymous";
    std::string role = "Team Member";
    std::string remarks = "";
    std::string source = "CoT-ROS";
    std::string group_affiliation = "friendly";

    // ===== 建構子 =====
    CoTInfo(const std::string& uid_,
            const std::string& type_,
            std::string how_ = "m-g",
            std::chrono::time_point<std::chrono::system_clock> now = std::chrono::system_clock::now())
        : uid(uid_), type(type_), how(how_), time(now), start_time(now),
          stale_time(now + std::chrono::seconds(300)) {}

    // ===== 封裝位置資訊 =====
private:
    double lat = 0.0, lon = 0.0, hae = 9999999.0, ce = 9999999.0, le = 9999999.0;

public:
    void set_position(double latitude, double longitude,
                      double hae_ = 9999999.0, double ce_ = 9999999.0, double le_ = 9999999.0) {
        lat = latitude;
        lon = longitude;
        hae = hae_;
        ce = ce_;
        le = le_;
    }

    // 如不希望外部讀取，不加 getter。只給 Builder 讀取就加 friend。
    friend class CoTBuilder;
};

