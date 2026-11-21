// Camera-IMU calibration from a recording in which the aircraft is ROTATED by hand (roll and pitch
// wiggles) while the camera looks at something far away. Far away matters: any sideways movement
// of the camera also shifts the image and biases the result.
//
// Model: shift of the image centre between two frames = M * (body rotation between them), where the
// rotation is taken from the gyro at the frame times plus a delay. The tool sweeps the delay, solves M
// by least squares at each step and keeps the delay with the smallest residual. Doing the same for the
// top, middle and bottom of the image gives three delays; their slope against the row is the
// rolling-shutter readout time.
//
// Usage: calib recording.vlog roi_radius_px sweep.csv
#include <algorithm>
#include <cstdio>
#include <iostream>
#include <vector>

#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/video/tracking.hpp>

#include "vlog.hpp"

using cv::Matx33d;
using cv::Vec2d;
using cv::Vec3d;

const double kSweepStart = -0.080, kSweepStep = 0.001;   // seconds
const int kSweepSteps = 161;
const int64_t kMaxFrameGapNs = 150000000;                // skip pairs across dropped frames

struct Pair {
    int64_t t_a, t_b;       // frame timestamps
    Vec2d shift[4];         // image shift with yaw and zoom removed: whole ROI, top, middle, bottom
    double row[4];          // mean row of the points used, as a fraction of the image height
    bool has[4];
    double image_yaw;       // rotation of the image, rad
};

static std::vector<int64_t> gyro_t;
static std::vector<Matx33d> gyro_R;   // gyro-only attitude, good over the short time between two frames

static const Matx33d& attitudeAt(int64_t t) {
    size_t i = std::lower_bound(gyro_t.begin(), gyro_t.end(), t) - gyro_t.begin();
    return gyro_R[std::min(i, gyro_R.size() - 1)];
}

static Vec3d rotationBetween(int64_t t_a, int64_t t_b) {
    Vec3d r;
    cv::Rodrigues(attitudeAt(t_a).t() * attitudeAt(t_b), r);
    return r;
}

// Least squares M (2x3) for one image region and one delay. Returns the RMS residual in pixels.
static double solve(const std::vector<Pair>& pairs, int region, double delay_s, cv::Matx<double, 2, 3>& M) {
    cv::Mat X, Y;
    const int64_t d = int64_t(delay_s * 1e9);
    for (const Pair& p : pairs) {
        if (!p.has[region]) continue;
        const Vec3d r = rotationBetween(p.t_a + d, p.t_b + d);
        X.push_back(cv::Mat(cv::Matx13d(r[0], r[1], r[2])));
        Y.push_back(cv::Mat(cv::Matx12d(p.shift[region][0], p.shift[region][1])));
    }
    cv::Mat Mt;
    cv::solve(X, Y, Mt, cv::DECOMP_SVD);
    M = cv::Matx<double, 2, 3>(cv::Mat(Mt.t()));
    return std::sqrt(cv::norm(X * Mt - Y, cv::NORM_L2SQR) / X.rows);
}

// Delay with the smallest residual, refined with a parabola through the three lowest steps.
static double bestDelay(const std::vector<Pair>& pairs, int region, std::vector<double>* curve) {
    std::vector<double> rms(kSweepSteps);
    cv::Matx<double, 2, 3> M;
    for (int i = 0; i < kSweepSteps; i++) rms[i] = solve(pairs, region, kSweepStart + i * kSweepStep, M);
    const int best = int(std::min_element(rms.begin(), rms.end()) - rms.begin());
    if (curve) *curve = rms;
    if (best == 0 || best == kSweepSteps - 1) return kSweepStart + best * kSweepStep;   // at the edge: no parabola
    const double a = rms[best - 1], b = rms[best], c = rms[best + 1];
    return kSweepStart + (best + 0.5 * (a - c) / (a - 2 * b + c)) * kSweepStep;
}

int main(int argc, char** argv) {
    if (argc != 4) {
        std::cerr << "usage: calib recording.vlog roi_radius_px sweep.csv\n";
        return 2;
    }
    const double roi_radius = std::stod(argv[2]);

    std::vector<Pair> pairs;
    cv::Mat prev;
    int64_t prev_t = 0, last_gyro_t = 0;
    cv::Size size;
    vlog::read(
        argv[1],
        [&](int64_t t, const Vec3d& gyro, const Vec3d&) {
            Matx33d step;
            cv::Rodrigues(gyro * (gyro_t.empty() ? 0.0 : (t - last_gyro_t) * 1e-9), step);
            gyro_R.push_back(gyro_R.empty() ? Matx33d::eye() : gyro_R.back() * step);
            gyro_t.push_back(t);
            last_gyro_t = t;
        },
        [&](int64_t t, const cv::Mat& gray) {
            size = gray.size();
            if (!prev.empty() && t - prev_t < kMaxFrameGapNs) {
                cv::Mat mask = cv::Mat::zeros(gray.size(), CV_8U);
                cv::circle(mask, cv::Point(gray.cols / 2, gray.rows / 2), int(roi_radius), 255, cv::FILLED);
                std::vector<cv::Point2f> a, b, from, to;
                cv::goodFeaturesToTrack(prev, a, 300, 0.01, 10, mask);
                std::vector<uchar> found, agrees;
                std::vector<float> error;
                if (a.size() >= 30) cv::calcOpticalFlowPyrLK(prev, gray, a, b, found, error, cv::Size(21, 21), 4);
                const cv::Point2f centre(gray.cols * 0.5f, gray.rows * 0.5f);
                for (size_t i = 0; i < b.size(); i++) {
                    if (!found[i]) continue;
                    from.push_back(a[i] - centre);
                    to.push_back(b[i] - centre);
                }
                cv::Mat fit;
                if (from.size() >= 30) fit = cv::estimateAffinePartial2D(from, to, agrees, cv::RANSAC, 2.0);
                if (!fit.empty() && cv::countNonZero(agrees) >= 30) {
                    Pair p{};
                    p.t_a = prev_t;
                    p.t_b = t;
                    p.image_yaw = std::atan2(fit.at<double>(1, 0), fit.at<double>(0, 0));
                    const cv::Matx22d L(fit.at<double>(0, 0), fit.at<double>(0, 1), fit.at<double>(1, 0), fit.at<double>(1, 1));
                    Vec2d sum[4];
                    double row_sum[4] = {0, 0, 0, 0};
                    int n[4] = {0, 0, 0, 0};
                    const double third = roi_radius / 3;
                    for (size_t i = 0; i < from.size(); i++) {
                        if (!agrees[i]) continue;
                        const Vec2d p0(from[i].x, from[i].y), p1(to[i].x, to[i].y);
                        const Vec2d shift = p1 - L * p0;   // what is left after yaw and zoom about the centre
                        const int band = p0[1] < -third ? 1 : p0[1] > third ? 3 : 2;
                        for (int region : {0, band}) {
                            sum[region] += shift;
                            row_sum[region] += (p0[1] + p1[1]) * 0.5 / gray.rows + 0.5;
                            n[region]++;
                        }
                    }
                    for (int r = 0; r < 4; r++) {
                        p.has[r] = n[r] >= 8;
                        if (!p.has[r]) continue;
                        p.shift[r] = sum[r] * (1.0 / n[r]);
                        p.row[r] = row_sum[r] / n[r];
                    }
                    pairs.push_back(p);
                }
            }
            prev = gray.clone();
            prev_t = t;
        });

    std::vector<double> curve;
    const double delay = bestDelay(pairs, 0, &curve);
    cv::Matx<double, 2, 3> M, unrelated;
    const double rms = solve(pairs, 0, delay, M);
    const double rms_unrelated = solve(pairs, 0, 10.0, unrelated);   // gyro from 10 s later: shows the raw shift size

    std::FILE* csv = std::fopen(argv[3], "w");
    std::fprintf(csv, "delay_s,rms_px\n");
    for (size_t i = 0; i < curve.size(); i++) std::fprintf(csv, "%.3f,%.4f\n", kSweepStart + i * kSweepStep, curve[i]);
    std::fclose(csv);

    double yaw_num = 0, yaw_den = 0;
    const int64_t d = int64_t(delay * 1e9);
    for (const Pair& p : pairs) {
        const Vec3d r = rotationBetween(p.t_a + d, p.t_b + d);
        yaw_num += p.image_yaw * r[2];
        yaw_den += r[2] * r[2];
    }

    std::printf("image %dx%d, frame pairs used %zu\n", size.width, size.height, pairs.size());
    std::printf("residual: %.2f px rms at the best delay, %.2f px rms with unrelated gyro\n", rms, rms_unrelated);
    std::printf("M (px per rad of body rotation about x, y, z):\n  %9.1f %9.1f %9.1f\n  %9.1f %9.1f %9.1f\n",
                M(0, 0), M(0, 1), M(0, 2), M(1, 0), M(1, 1), M(1, 2));
    std::printf("image yaw per rad of body z rotation: %.3f (expect +1 or -1)\n", yaw_num / yaw_den);
    std::printf("focal length implied: %.1f px\n", std::sqrt(std::abs(M(0, 0) * M(1, 1) - M(0, 1) * M(1, 0))));
    double band_delay[4], band_row[4];
    for (int r = 1; r < 4; r++) {
        band_delay[r] = bestDelay(pairs, r, nullptr);
        double row = 0;
        int n = 0;
        for (const Pair& p : pairs) {
            if (!p.has[r]) continue;
            row += p.row[r];
            n++;
        }
        band_row[r] = row / n;
        std::printf("band %d: mean row %.3f of height, delay %.2f ms (%d pairs)\n", r, band_row[r], band_delay[r] * 1e3, n);
    }
    // Row time = frame stamp + cam_delay - readout * (1 - row), so a band's delay rises with its row.
    const double readout = (band_delay[3] - band_delay[1]) / (band_row[3] - band_row[1]);
    std::printf("\n# config lines\n");
    std::printf("tilt_to_px_xx %.1f\ntilt_to_px_xy %.1f\ntilt_to_px_yx %.1f\ntilt_to_px_yy %.1f\n", M(0, 0), M(0, 1), M(1, 0), M(1, 1));
    std::printf("cam_delay_s %.4f\nrolling_shutter_readout_s %.4f\n", band_delay[2] + readout * (1 - band_row[2]), readout);
    return 0;
}
