// Flat log reader. Format: see tools/mcap2log.py.
#pragma once

#include <cstdint>
#include <fstream>
#include <functional>
#include <stdexcept>
#include <string>

#include <opencv2/core.hpp>

namespace vlog {

// Calls on_imu / on_frame for every record, in file order.
inline void read(const std::string& path,
                 const std::function<void(int64_t t_ns, const cv::Vec3d& gyro, const cv::Vec3d& acc)>& on_imu,
                 const std::function<void(int64_t t_ns, const cv::Mat& gray)>& on_frame) {
    std::ifstream in(path, std::ios::binary);
    if (!in) throw std::runtime_error("cannot open " + path);
    char type;
    while (in.read(&type, 1)) {
        int64_t t_ns;
        in.read(reinterpret_cast<char*>(&t_ns), 8);
        if (type == 'I') {
            float v[6];
            in.read(reinterpret_cast<char*>(v), sizeof v);
            on_imu(t_ns, cv::Vec3d(v[0], v[1], v[2]), cv::Vec3d(v[3], v[4], v[5]));
        } else if (type == 'F') {
            uint16_t size[2];
            in.read(reinterpret_cast<char*>(size), sizeof size);
            cv::Mat gray(size[1], size[0], CV_8UC1);
            in.read(reinterpret_cast<char*>(gray.data), std::streamsize(size[0]) * size[1]);
            on_frame(t_ns, gray);
        } else {
            throw std::runtime_error("bad record type in " + path);
        }
        if (!in) throw std::runtime_error("truncated record in " + path);
    }
}

}  // namespace vlog
