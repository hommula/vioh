// Run the estimator over a recording and write one CSV line per frame.
// Usage: replay config.txt recording.vlog out.csv
#include <chrono>
#include <cstdio>
#include <iostream>

#include "hold.hpp"
#include "vlog.hpp"

int main(int argc, char** argv) {
    if (argc != 4) {
        std::cerr << "usage: replay config.txt recording.vlog out.csv\n";
        return 2;
    }
    const hold::Config config(argv[1]);
    hold::Estimator estimator(config);
    std::FILE* csv = std::fopen(argv[3], "w");
    std::fprintf(csv, "t_s,tracking,inliers,keyframes,Ux,Uy,scale,Z0_m,excitation,ms\n");

    int frames = 0, tracked = 0;
    double total_ms = 0, worst_ms = 0;
    int64_t t0 = 0;
    vlog::read(
        argv[2],
        [&](int64_t t, const cv::Vec3d& gyro, const cv::Vec3d& acc) { estimator.imu(t, gyro, acc); },
        [&](int64_t t, const cv::Mat& gray) {
            if (t0 == 0) t0 = t;
            const auto start = std::chrono::steady_clock::now();
            estimator.frame(t, gray);
            const double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
            frames++;
            tracked += estimator.tracking;
            total_ms += ms;
            worst_ms = std::max(worst_ms, ms);
            std::fprintf(csv, "%.3f,%d,%d,%d,%.5f,%.5f,%.4f,%.3f,%.3f,%.2f\n", (t - t0) * 1e-9, estimator.tracking,
                         estimator.inliers, estimator.keyframes, estimator.U[0], estimator.U[1], estimator.scale,
                         estimator.Z0, estimator.excitation, ms);
        });
    std::fclose(csv);
    std::printf("frames %d, registered %d (%.1f %%), keyframes %d\n", frames, tracked, 100.0 * tracked / frames, estimator.keyframes);
    std::printf("per frame on this machine: mean %.2f ms, worst %.2f ms\n", total_ms / frames, worst_ms);
    std::printf("end: U = (%.3f, %.3f), scale %.3f, Z0 %.2f m, excitation %.2f\n", estimator.U[0], estimator.U[1],
                estimator.scale, estimator.Z0, estimator.excitation);
    return 0;
}
