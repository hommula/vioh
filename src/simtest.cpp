// Synthetic check of the estimator geometry: a fisheye camera flies a known path over a flat textured
// floor, with matching gyro and accelerometer data. Passes if the estimate follows the true path.
// This tests the maths and the signs only. It says nothing about the real camera, lens or noise.
//
// Usage: simtest config.txt out.csv      (uses the estimator settings from the config; camera values are simulated)
#include <cstdio>
#include <iostream>

#include "hold.hpp"

using cv::Matx33d;
using cv::Vec3d;

// World: x north, y east, z down, floor at z = 0. Body = IMU: x forward, y right, z down.
// Camera: z along body z (down), image x = body y, image y = -body x (top of the image is forward).
const double kGravity = 9.81;
const cv::Size kImage(960, 540);
const Matx33d kCamFromBody(0, 1, 0, -1, 0, 0, 0, 0, 1);
// Simulated lens: OpenCV fisheye model, centre a little off the middle. About 115 degrees across the image.
const Matx33d kLensK(480, 0, 487, 0, 478, 262, 0, 0, 1);
const cv::Vec4d kLensD(-0.03, 0.01, -0.004, 0.001);
const Vec3d kGyroBias(0.015, -0.010, 0.005);   // rad/s, about the size seen in the 20261121 recording
// Pass limits: half of the bench tolerances in Stage 2 section 05 (0.25 m displacement, 0.15 m height),
// and the scale error that 0.25 m in 1.5 m allows.
const double kMaxPositionError = 0.125, kMaxHeightError = 0.075, kMaxScaleError = 0.15;

// The aircraft is held still for the first 7 s (as at power-up on the bench), then the motion fades in.
static double moving(double t) {
    const double x = std::clamp((t - 7) / 3, 0.0, 1.0);
    return x * x * x * (x * (6 * x - 15) + 10);
}

static Vec3d position(double t) {
    return Vec3d(0.30 * std::sin(2 * CV_PI * t / 3.0), 0.20 * std::sin(2 * CV_PI * t / 2.2 + 1.0),
                 -0.10 * std::sin(2 * CV_PI * t / 5.0)) * moving(t) + Vec3d(0, 0, -1.0);
}

static Matx33d attitude(double t) {   // body -> world
    Matx33d roll, pitch, yaw;
    cv::Rodrigues(Vec3d(0.03 + moving(t) * 0.08 * std::sin(2 * CV_PI * t / 1.3), 0, 0), roll);
    cv::Rodrigues(Vec3d(0, -0.02 + moving(t) * 0.06 * std::sin(2 * CV_PI * t / 1.7 + 0.5), 0), pitch);
    cv::Rodrigues(Vec3d(0, 0, 0.3 + moving(t) * 0.20 * std::sin(2 * CV_PI * t / 7.0)), yaw);
    return yaw * pitch * roll;
}

// Every pixel's viewing ray is followed to the floor, and the floor texture is read there.
static cv::Mat render(const cv::Mat& floor, double px_per_m, const std::vector<cv::Point2f>& pixel_rays, double t) {
    const Matx33d world_from_cam = attitude(t) * kCamFromBody.t();
    const Vec3d p = position(t);
    cv::Mat map_x(kImage, CV_32F), map_y(kImage, CV_32F);
    for (int i = 0; i < kImage.area(); i++) {
        const Vec3d ray = world_from_cam * Vec3d(pixel_rays[i].x, pixel_rays[i].y, 1);
        const double reach = -p[2] / ray[2];   // the floor is at z = 0 and the camera at z < 0
        map_x.at<float>(i) = float((p[0] + reach * ray[0]) * px_per_m + floor.cols / 2.0);
        map_y.at<float>(i) = float((p[1] + reach * ray[1]) * px_per_m + floor.rows / 2.0);
    }
    cv::Mat frame;
    cv::remap(floor, frame, map_x, map_y, cv::INTER_LINEAR);
    return frame;
}

int main(int argc, char** argv) {
    if (argc != 3) {
        std::cerr << "usage: simtest config.txt out.csv\n";
        return 2;
    }
    hold::Config config(argv[1]);
    // The simulated camera: its lens, tilt_to_px = f * J * (xy block of camera-from-body), no delay, global shutter.
    const cv::Matx22d M = kLensK(0, 0) * cv::Matx22d(0, -1, 1, 0) *
                          cv::Matx22d(kCamFromBody(0, 0), kCamFromBody(0, 1), kCamFromBody(1, 0), kCamFromBody(1, 1));
    config.values["tilt_to_px_xx"] = M(0, 0);
    config.values["tilt_to_px_xy"] = M(0, 1);
    config.values["tilt_to_px_yx"] = M(1, 0);
    config.values["tilt_to_px_yy"] = M(1, 1);
    config.values["cam_delay_s"] = 0;
    config.values["rolling_shutter_readout_s"] = 0;
    config.values["lens_fx"] = kLensK(0, 0);
    config.values["lens_fy"] = kLensK(1, 1);
    config.values["lens_cx"] = kLensK(0, 2);
    config.values["lens_cy"] = kLensK(1, 2);
    for (int i = 0; i < 4; i++) config.values["lens_k" + std::to_string(i + 1)] = kLensD[i];
    hold::Estimator estimator(config);

    std::vector<cv::Point2f> pixels, pixel_rays;
    for (int y = 0; y < kImage.height; y++)
        for (int x = 0; x < kImage.width; x++) pixels.push_back(cv::Point2f(float(x), float(y)));
    cv::fisheye::undistortPoints(pixels, pixel_rays, kLensK, kLensD);

    const double px_per_m = 500;
    cv::Mat floor(5000, 5000, CV_8U);
    cv::RNG rng(1);
    rng.fill(floor, cv::RNG::UNIFORM, 0, 256);
    cv::GaussianBlur(floor, floor, cv::Size(0, 0), 4);
    cv::normalize(floor, floor, 0, 255, cv::NORM_MINMAX);

    std::FILE* csv = std::fopen(argv[2], "w");
    std::fprintf(csv, "t_s,tracking,true_x,true_y,est_x,est_y,true_height,est_height,Z0_fit\n");

    const double imu_dt = 0.001, frame_dt = 1.0 / 30, duration = 40, h = 1e-4;
    const double cover_from = 30, cover_to = 33;   // lens covered: black frames
    double next_frame = 0.5, worst_error = 0, worst_height_error = 0;
    int untracked_outside_cover = 0, tracked_inside_cover = 0, frames_after_first = 0;
    bool have_origin = false;
    Vec3d origin_position;
    Matx33d origin_yaw;
    double origin_height = 0;
    for (double t = 0; t < duration; t += imu_dt) {
        const Matx33d R = attitude(t);
        Vec3d gyro;
        cv::Rodrigues(R.t() * attitude(t + h), gyro);
        gyro = gyro * (1 / h) + kGyroBias;
        const Vec3d accel_world = (position(t + h) - 2 * position(t) + position(t - h)) * (1 / (h * h));
        const Vec3d acc = R.t() * (accel_world - Vec3d(0, 0, kGravity));
        estimator.imu(int64_t(t * 1e9), gyro, acc);

        if (t < next_frame) continue;
        next_frame += frame_dt;
        const bool covered = t >= cover_from && t < cover_to;
        cv::Mat frame = covered ? cv::Mat(cv::Mat::zeros(kImage, CV_8U)) : render(floor, px_per_m, pixel_rays, t);
        estimator.frame(int64_t(t * 1e9), frame);
        if (!have_origin) {   // the first frame the estimator uses is the origin keyframe
            if (estimator.keyframes == 0) continue;
            have_origin = true;
            origin_position = position(t);
            origin_height = -position(t)[2];
            const Matx33d R0 = attitude(t);
            cv::Rodrigues(Vec3d(0, 0, std::atan2(R0(1, 0), R0(0, 0))), origin_yaw);
            continue;
        }
        frames_after_first++;
        // Truth in the origin level axes.
        const Vec3d truth = origin_yaw.t() * (position(t) - origin_position);
        const double true_height = -position(t)[2];
        const hold::Vec2d estimate = estimator.U * origin_height;
        const double est_height = origin_height / estimator.scale;
        std::fprintf(csv, "%.3f,%d,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f\n", t, estimator.tracking, truth[0], truth[1],
                     estimate[0], estimate[1], true_height, est_height, estimator.Z0);
        if (covered) {
            tracked_inside_cover += estimator.tracking;
        } else if (t < cover_from) {   // before the cover: the estimate must follow the truth
            untracked_outside_cover += !estimator.tracking;
            worst_error = std::max(worst_error, std::hypot(estimate[0] - truth[0], estimate[1] - truth[1]));
            worst_height_error = std::max(worst_height_error, std::abs(est_height - true_height));
        }
    }
    std::fclose(csv);

    std::printf("before the cover: worst position error %.3f m, worst height error %.3f m, untracked frames %d of %d\n",
                worst_error, worst_height_error, untracked_outside_cover, frames_after_first);
    std::printf("origin height: true %.3f m, accelerometer fit %.3f m (excitation %.2f)\n", origin_height, estimator.Z0, estimator.excitation);
    std::printf("covered lens: frames still reported as tracking %d; tracking again at the end: %s\n", tracked_inside_cover,
                estimator.tracking ? "yes" : "no");
    const bool pass = worst_error < kMaxPositionError && worst_height_error < kMaxHeightError && untracked_outside_cover == 0 &&
                      std::abs(estimator.Z0 - origin_height) < kMaxScaleError * origin_height && tracked_inside_cover == 0 &&
                      estimator.tracking;
    std::printf(pass ? "PASS\n" : "FAIL\n");
    return pass ? 0 : 1;
}
