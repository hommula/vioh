// Position estimate for a nadir camera + IMU, by registering each frame to a keyframe.
// Pure computation: no devices, no clocks. The onboard program and the replay tool both call this.
//
// Frames used below
//   body   : the APB IMU axes, as the IMU reports them.
//   world  : the body axes at the first IMU sample. `up_` is the direction of gravity reaction in it.
//   level  : body axes with the tilt removed (same heading). Positions are reported in the level axes
//            of the first keyframe ("origin"), x and y only.
//   raw pixel   : image coordinates as the fisheye camera delivers them.
//   level pixel : where the same viewing ray lands in an ideal (pinhole, no distortion) camera at the
//                 same place that looks straight down, with its centre at (0, 0). On a flat floor a
//                 move of the aircraft is then a pure shift, a climb a pure zoom, a yaw a pure rotation.
#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <deque>
#include <fstream>
#include <map>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/video/tracking.hpp>

namespace hold {

using cv::Matx22d;
using cv::Matx33d;
using cv::Vec2d;
using cv::Vec3d;

// "key value" lines, '#' starts a comment. A missing key is an error: there are no built-in defaults.
struct Config {
    std::map<std::string, double> values;

    explicit Config(const std::string& path) {
        std::ifstream in(path);
        if (!in) throw std::runtime_error("config: cannot open " + path);
        std::string line;
        while (std::getline(in, line)) {
            std::istringstream words(line.substr(0, line.find('#')));
            std::string key;
            double value;
            if (words >> key >> value) values[key] = value;
        }
    }

    double operator()(const std::string& key) const {
        auto it = values.find(key);
        if (it == values.end()) throw std::runtime_error("config: missing key " + key);
        return it->second;
    }
};

inline Matx33d expSO3(const Vec3d& rotation_vector) {
    Matx33d R;
    cv::Rodrigues(rotation_vector, R);
    return R;
}

// One IMU sample after integration.
struct ImuState {
    int64_t t_ns;
    Matx33d R;   // body -> world
    Vec3d V;     // integral of (acceleration without gravity), world axes, m/s. Drifts: use differences only.
    Vec3d P;     // integral of V, m. Drifts faster still.
};

class Estimator {
public:
    // Outputs, valid after frame().
    bool tracking = false;    // this frame was registered to the keyframe
    int inliers = 0;          // points that agree with the fitted motion
    int good_frames = 0;      // consecutive registered frames
    Vec2d U{0, 0};            // position / origin height, origin level axes (no unit)
    Vec2d U_rate{0, 0};       // low-passed d(U)/dt, 1/s
    double scale = 1;         // image scale relative to origin: height now = origin height / scale
    Matx22d L_hat = Matx22d::eye();  // image rotation, origin pixel directions -> current pixel directions
    double Z0 = 0;            // origin height from the accelerometer fit, m. Only meaningful if excitation is high.
    double excitation = 0;    // how much visual velocity change the fit has seen, 1/s^2 summed
    int keyframes = 0;

    explicit Estimator(const Config& c)
        : tilt_to_px_(c("tilt_to_px_xx"), c("tilt_to_px_xy"), c("tilt_to_px_yx"), c("tilt_to_px_yy")),
          cam_delay_s_(c("cam_delay_s")),
          readout_s_(c("rolling_shutter_readout_s")),
          roi_radius_px_(c("roi_radius_px")),
          max_corners_(int(c("max_corners"))),
          min_inliers_(int(c("min_inliers"))),
          min_inlier_ratio_(c("min_inlier_ratio")),
          rekey_inlier_ratio_(c("rekey_inlier_ratio")),
          ransac_px_(c("ransac_px")),
          rekey_shift_px_(c("rekey_shift_px")),
          rekey_log_scale_(c("rekey_log_scale")),
          rekey_rotation_rad_(c("rekey_rotation_rad")),
          gravity_gain_(c("gravity_gain_per_s")),
          attitude_settle_s_(c("attitude_settle_s")),
          still_gyro_rad_s_(c("still_gyro_rad_s")),
          rate_filter_s_(c("rate_filter_s")),
          scale_lag_s_(c("scale_lag_s")),
          scale_memory_s_(c("scale_memory_s")),
          still_gyro_sd_(c("still_gyro_sd_rad_s")) {
        // Lens: OpenCV fisheye model (cv::fisheye), from a checkerboard calibration.
        lens_K_ = Matx33d(c("lens_fx"), 0, c("lens_cx"), 0, c("lens_fy"), c("lens_cy"), 0, 0, 1);
        lens_D_ = cv::Vec4d(c("lens_k1"), c("lens_k2"), c("lens_k3"), c("lens_k4"));
        focal_px_ = c("lens_fx");   // focal length chosen for the level image: same scale as the raw centre
        // Pixel shift at the image centre = tilt_to_px * (tilt x, tilt y). For a camera looking along the
        // body z axis this matrix is f * J * C, with focal length f, J = [0 -1; 1 0] and C the x-y block
        // of the camera-from-body rotation. Only the rotation is taken from it; f comes from the lens.
        const Matx22d J(0, -1, 1, 0);
        cv::Mat w, u, vt;
        cv::SVD::compute(cv::Mat(J.t() * tilt_to_px_), w, u, vt);
        const Matx22d C(cv::Mat(u * vt));
        cam_from_body_ = Matx33d(C(0, 0), C(0, 1), 0, C(1, 0), C(1, 1), 0, 0, 0, cv::determinant(C));
        tilt_to_px_ = focal_px_ * J * C;
        // A move of p metres (level axes) at height Z shifts the image by -(1/Z) * J^-1 * tilt_to_px * p,
        // so this turns "origin pixel now under the camera" into position / height.
        px_to_pos_ = tilt_to_px_.inv() * J;
    }

    // Feed every IMU sample, in time order. gyro rad/s, acc m/s^2 (reads +g along "up" at rest).
    void imu(int64_t t_ns, const Vec3d& gyro, const Vec3d& acc) {
        if (imu_.empty()) {
            up_ = cv::normalize(acc);
            g_ = cv::norm(acc);
            first_imu_t_ns_ = t_ns;
            imu_.push_back({t_ns, Matx33d::eye(), Vec3d(0, 0, 0), Vec3d(0, 0, 0)});
            return;
        }
        const ImuState& last = imu_.back();
        const double dt = (t_ns - last.t_ns) * 1e-9;
        // Complementary filter: turn the attitude so that the predicted "up" meets the measured one.
        // The gain starts high (plain averaging) and falls to gravity_gain_, which is kept low because
        // sideways acceleration also tilts the measured "up".
        const Vec3d error = cv::normalize(acc).cross(last.R.t() * up_);
        const double gain = std::max(gravity_gain_, 1e9 / double(t_ns - first_imu_t_ns_));
        // Gyro bias. At rest the gyro reads nothing but its own bias, so follow that reading (over 2 s).
        // "At rest" means the gyro has been steady for the last half second: MEASURED on the board, a
        // hand-held aircraft shows 0.012 to 0.022 rad/s of spread per axis even when held as still as
        // possible, so a small reading alone is not proof of rest. Otherwise the tilt error shows the bias.
        // MEASURED on the board 21 Nov: an earlier version also fed the high start-up gain into the bias.
        // With real accelerometer noise the bias then got a kick of up to 1 rad/s at start and the tilt
        // swung by +-70 degrees for a minute on a level aircraft.
        const double bias_learn_s = 2.0, steady_window_s = 0.5;
        const double window_weight = dt / (steady_window_s + dt);
        gyro_mean_ += (gyro - gyro_mean_) * window_weight;
        gyro_spread2_ += ((gyro - gyro_mean_).dot(gyro - gyro_mean_) - gyro_spread2_) * window_weight;
        const bool resting = gyro_spread2_ < still_gyro_sd_ * still_gyro_sd_ && cv::norm(gyro_mean_) < still_gyro_rad_s_;
        if (resting) gyro_bias_ += (gyro_mean_ - gyro_bias_) * (dt / bias_learn_s);
        else gyro_bias_ -= error * (gravity_gain_ * gravity_gain_ / 4 * dt);
        const Matx33d R = last.R * expSO3((gyro - gyro_bias_ + error * gain) * dt);
        g_ += gravity_gain_ * dt * (cv::norm(acc) - g_);
        imu_.push_back({t_ns, R, last.V + (R * acc - g_ * up_) * dt, last.P + last.V * dt});
        while (imu_.front().t_ns < t_ns - 2'000'000'000) imu_.pop_front();
    }

    // Feed every camera frame. t_ns is the driver timestamp (end of frame).
    // Needs IMU samples up to that time already fed.
    void frame(int64_t t_ns, const cv::Mat& gray) {
        // The attitude starts from one accelerometer sample; give the gravity filter time to settle
        // before any image is levelled with it.
        if (imu_.empty() || imu_.back().t_ns - first_imu_t_ns_ < int64_t(attitude_settle_s_ * 1e9)) return;
        const int64_t t = t_ns + int64_t(cam_delay_s_ * 1e9);
        if (key_image_.empty()) {
            newKeyframe(gray, t);
            origin_level_ = levelled(at(t).R);
            return;
        }

        // Start the search where the gyro says the points went since the last frame.
        const Vec2d centre_tilt = tiltShift(t);
        const cv::Point2f gyro_move(float(centre_tilt[0] - last_tilt_shift_[0]), float(centre_tilt[1] - last_tilt_shift_[1]));
        for (cv::Point2f& p : cur_raw_) p += gyro_move;
        last_tilt_shift_ = centre_tilt;

        std::vector<uchar> found(key_raw_.size(), 0);
        std::vector<float> error;
        if (!key_raw_.empty())   // a keyframe of a covered lens has no points at all
            cv::calcOpticalFlowPyrLK(key_image_, gray, key_raw_, cur_raw_, found, error, cv::Size(21, 21), 3,
                                     cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, 0.01),
                                     cv::OPTFLOW_USE_INITIAL_FLOW);
        const std::vector<cv::Point2f> cur_level = levelPoints(cur_raw_, t, gray.rows);
        std::vector<cv::Point2f> from, to;
        for (size_t i = 0; i < key_raw_.size(); i++) {
            if (!found[i]) continue;
            from.push_back(key_level_[i]);
            to.push_back(cur_level[i]);
        }
        std::vector<uchar> agrees;
        cv::Mat fit;
        if (from.size() >= size_t(min_inliers_))
            fit = cv::estimateAffinePartial2D(from, to, agrees, cv::RANSAC, ransac_px_);
        inliers = fit.empty() ? 0 : cv::countNonZero(agrees);

        // key -> current, as a 3x3 matrix on level pixel coordinates.
        Matx33d A = Matx33d::eye();
        if (!fit.empty())
            A = Matx33d(fit.at<double>(0, 0), fit.at<double>(0, 1), fit.at<double>(0, 2),
                        fit.at<double>(1, 0), fit.at<double>(1, 1), fit.at<double>(1, 2), 0, 0, 1);
        const double a_shift = std::hypot(A(0, 2), A(1, 2));
        const double a_log_scale = std::abs(std::log(std::hypot(A(0, 0), A(1, 0))));
        const double a_rotation = std::abs(std::atan2(A(1, 0), A(0, 0)));
        // A real motion is caught by the keyframe limits long before it reaches twice their size,
        // and most keyframe points agree with it. Anything else is a wrong fit: treat it as lost.
        // "Most" is measured against what this keyframe gave on its first frame: on the real carpet
        // only 150 to 180 of 200 points agree even when all is well (MEASURED on the board).
        const double usual_inliers = key_inliers_ > 0 ? key_inliers_ : double(key_raw_.size());
        const bool believable = inliers >= min_inliers_ && inliers >= min_inlier_ratio_ * usual_inliers &&
                                a_shift <= 2 * rekey_shift_px_ && a_log_scale <= 2 * rekey_log_scale_ &&
                                a_rotation <= 2 * rekey_rotation_rad_;

        if (!believable) {
            // Lost (covered lens, blur, too little texture). Keep the last position and start again
            // from this frame. The position then no longer includes the motion while lost.
            origin_to_key_ = last_total_;
            tracking = false;
            good_frames = 0;
            U_rate = Vec2d(0, 0);
            track_.clear();
            newKeyframe(gray, t);
            return;
        }
        tracking = true;
        good_frames++;
        if (key_inliers_ == 0) key_inliers_ = inliers;

        const Matx33d total = A * origin_to_key_;   // origin -> current
        last_total_ = total;
        const Matx22d L(total(0, 0), total(0, 1), total(1, 0), total(1, 1));
        const Vec2d shift(total(0, 2), total(1, 2));
        scale = std::sqrt(cv::determinant(L));
        L_hat = L * (1.0 / scale);
        const Vec2d U_new = px_to_pos_ * (-(L.inv() * shift));   // origin pixel under the camera now

        updateRateAndScale(t, U_new);
        U = U_new;

        // Points that failed or disagreed: put them where the fit says they are, for the next frame.
        std::vector<cv::Point2f> predicted_level;
        for (const cv::Point2f& p : key_level_) {
            const Vec3d q = A * Vec3d(p.x, p.y, 1);
            predicted_level.push_back(cv::Point2f(float(q[0]), float(q[1])));
        }
        const std::vector<cv::Point2f> predicted_raw = rawPoints(predicted_level, t);
        size_t k = 0;
        for (size_t i = 0; i < key_raw_.size(); i++) {
            const bool ok = found[i] && agrees[k++];
            if (!ok) cur_raw_[i] = predicted_raw[i];
        }

        const bool far = a_shift > rekey_shift_px_ || a_log_scale > rekey_log_scale_ || a_rotation > rekey_rotation_rad_ ||
                         inliers < rekey_inlier_ratio_ * key_inliers_;   // the view has changed too much
        if (far || rekey_requested_) {
            origin_to_key_ = total;
            newKeyframe(gray, t);
        }
    }

    // Make the next registered frame the keyframe (call at engage, so the hold cannot drift).
    void requestKeyframe() { rekey_requested_ = true; }

    // Turns a vector in origin level axes into the current level axes (they differ by the yaw since origin).
    Matx22d originToCurrent() const { return px_to_pos_ * L_hat * px_to_pos_.inv(); }

    // Integrated vertical acceleration, m/s, positive up. Drifts: use its change over short times only.
    double verticalVelocity() const { return imu_.back().V.dot(up_); }

    bool hasImu() const { return !imu_.empty(); }

    // Tilt away from level right now, radians: rotation about the IMU x axis and about the IMU y axis.
    Vec2d tiltNow() const {
        if (imu_.empty()) return Vec2d(0, 0);
        const Vec3d tilt_vector = tilt(imu_.back().R);
        return Vec2d(tilt_vector[0], tilt_vector[1]);
    }

private:
    // IMU state nearest to time t (samples are 1 ms apart, so no interpolation).
    const ImuState& at(int64_t t) const {
        auto it = std::lower_bound(imu_.begin(), imu_.end(), t,
                                   [](const ImuState& s, int64_t time) { return s.t_ns < time; });
        return it == imu_.end() ? imu_.back() : *it;
    }

    // Rotation vector (body axes, z = 0) that takes the level attitude to attitude R.
    Vec3d tilt(const Matx33d& R) const {
        const Vec3d up_body = R.t() * up_;
        const Vec3d z(0, 0, up_body[2] > 0 ? 1 : -1);
        const Vec3d axis = z.cross(up_body);
        const double sine = cv::norm(axis);
        if (sine < 1e-9) return Vec3d(0, 0, 0);
        return axis * (-std::atan2(sine, z.dot(up_body)) / sine);
    }

    Matx33d levelled(const Matx33d& R) const { return R * expSO3(tilt(R)).t(); }

    // Image shift at the centre caused by the tilt at time t, raw pixels.
    Vec2d tiltShift(int64_t t) const {
        const Vec3d tilt_vector = tilt(at(t).R);
        return tilt_to_px_ * Vec2d(tilt_vector[0], tilt_vector[1]);   // close enough to start a search
    }

    // Raw pixels -> level pixels. The lens model turns each pixel into a viewing ray; the ray is then
    // rotated by the tilt at the time its image row was read out (rolling shutter: the last row is read
    // at t, the first one readout_s_ earlier) and projected into the level camera.
    std::vector<cv::Point2f> levelPoints(const std::vector<cv::Point2f>& raw, int64_t t, int rows) const {
        std::vector<cv::Point2f> level, pinhole;
        if (raw.empty()) return level;
        cv::fisheye::undistortPoints(raw, pinhole, lens_K_, lens_D_);   // x/z and y/z of each ray
        for (size_t i = 0; i < raw.size(); i++) {
            const int64_t row_time = t - int64_t(readout_s_ * 1e9 * (1.0 - raw[i].y / rows));
            const Vec3d ray_camera(pinhole[i].x, pinhole[i].y, 1);
            const Vec3d ray_level = cam_from_body_ * (expSO3(tilt(at(row_time).R)) * (cam_from_body_.t() * ray_camera));
            level.push_back(cv::Point2f(float(focal_px_ * ray_level[0] / ray_level[2]), float(focal_px_ * ray_level[1] / ray_level[2])));
        }
        return level;
    }

    // The way back, level pixels -> raw pixels, with the tilt at time t for every point. Only used to
    // predict where to search, so the row time is ignored.
    std::vector<cv::Point2f> rawPoints(const std::vector<cv::Point2f>& level, int64_t t) const {
        std::vector<cv::Point2f> raw, pinhole;
        if (level.empty()) return raw;
        const Matx33d level_to_camera = cam_from_body_ * expSO3(tilt(at(t).R)).t() * cam_from_body_.t();
        for (const cv::Point2f& p : level) {
            const Vec3d ray = level_to_camera * Vec3d(p.x / focal_px_, p.y / focal_px_, 1);
            pinhole.push_back(cv::Point2f(float(ray[0] / ray[2]), float(ray[1] / ray[2])));
        }
        cv::fisheye::distortPoints(pinhole, raw, lens_K_, lens_D_);
        return raw;
    }

    void newKeyframe(const cv::Mat& gray, int64_t t) {
        // Looking for corners costs time in proportion to the pixels searched. MEASURED on the board:
        // a frame that made a keyframe from the whole 960 x 540 image took 65 ms instead of 13 ms.
        // So search only the square around the circle, at half size, and scale the corners back up.
        const cv::Point centre(gray.cols / 2, gray.rows / 2);
        const int radius = int(roi_radius_px_);
        const cv::Rect box = cv::Rect(centre.x - radius, centre.y - radius, 2 * radius, 2 * radius) &
                             cv::Rect(0, 0, gray.cols, gray.rows);
        cv::Mat small;
        cv::resize(gray(box), small, cv::Size(), 0.5, 0.5, cv::INTER_AREA);
        cv::Mat mask = cv::Mat::zeros(small.size(), CV_8U);
        cv::circle(mask, (centre - box.tl()) / 2, radius / 2, 255, cv::FILLED);
        key_raw_.clear();
        cv::goodFeaturesToTrack(small, key_raw_, max_corners_, 0.01, 6, mask);
        // Pixel i of the half-size image covers pixels 2i and 2i + 1 of the full one.
        for (cv::Point2f& p : key_raw_) p = cv::Point2f(box.x + 2 * p.x + 0.5f, box.y + 2 * p.y + 0.5f);
        key_inliers_ = 0;
        key_level_ = levelPoints(key_raw_, t, gray.rows);
        cur_raw_ = key_raw_;
        last_tilt_shift_ = tiltShift(t);
        key_image_ = gray.clone();
        rekey_requested_ = false;
        keyframes++;
    }

    // Velocity for the controller, and the height that makes the visual track match the accelerometer.
    // Over three frames about scale_lag_s apart: (mean velocity over the newer half) - (mean velocity
    // over the older half), once from the image track U and once from the double-integrated
    // accelerometer P. The second is Z0 times the first. Unknown start velocity cancels in both.
    void updateRateAndScale(int64_t t, const Vec2d& U_new) {
        if (good_frames > 1) {
            const double dt = (t - last_t_) * 1e-9;
            U_rate += ((U_new - U) * (1.0 / dt) - U_rate) * (dt / (rate_filter_s_ + dt));
        }
        last_t_ = t;

        const Vec3d p = origin_level_.t() * at(t).P;
        track_.push_back({t, U_new, Vec2d(p[0], p[1])});
        const int64_t lag = int64_t(scale_lag_s_ * 1e9);
        while (track_.size() > 1 && track_[1].t_ns <= t - 2 * lag) track_.pop_front();
        const TrackSample& old = track_.front();
        if (t - old.t_ns < 2 * lag) return;   // not enough history yet
        auto mid_it = std::lower_bound(track_.begin(), track_.end(), t - lag,
                                       [](const TrackSample& s, int64_t time) { return s.t_ns < time; });
        const TrackSample& mid = *mid_it;
        const TrackSample& now = track_.back();
        if (mid.t_ns == old.t_ns || mid.t_ns == now.t_ns) return;
        const double newer_s = (now.t_ns - mid.t_ns) * 1e-9, older_s = (mid.t_ns - old.t_ns) * 1e-9;
        const Vec2d seen = (now.U - mid.U) * (1 / newer_s) - (mid.U - old.U) * (1 / older_s);
        const Vec2d felt = (now.P - mid.P) * (1 / newer_s) - (mid.P - old.P) * (1 / older_s);
        const double keep = std::exp(-(now.t_ns - track_[track_.size() - 2].t_ns) * 1e-9 / scale_memory_s_);
        fit_num_ = fit_num_ * keep + seen.dot(felt);
        fit_den_ = fit_den_ * keep + seen.dot(seen);
        excitation = fit_den_;
        if (fit_den_ > 0) Z0 = fit_num_ / fit_den_;
    }

    struct TrackSample {
        int64_t t_ns;
        Vec2d U;   // position / origin height
        Vec2d P;   // double-integrated acceleration, origin level axes, m
    };

    // Configuration (see config.example for what each is and how to measure it).
    Matx22d tilt_to_px_, px_to_pos_;
    Matx33d cam_from_body_, lens_K_;
    cv::Vec4d lens_D_;
    double focal_px_ = 0;
    double cam_delay_s_, readout_s_, roi_radius_px_;
    int max_corners_, min_inliers_;
    double min_inlier_ratio_, rekey_inlier_ratio_;
    double ransac_px_, rekey_shift_px_, rekey_log_scale_, rekey_rotation_rad_;
    double gravity_gain_, attitude_settle_s_, still_gyro_rad_s_, rate_filter_s_, scale_lag_s_, scale_memory_s_;
    double still_gyro_sd_ = 0;
    Vec3d gyro_mean_{0, 0, 0};      // gyro reading averaged over the last half second
    double gyro_spread2_ = 0.01;    // its mean squared spread over the same time; starts as "not steady"
    int64_t first_imu_t_ns_ = 0;

    std::deque<ImuState> imu_;
    Vec3d up_{0, 0, 1};
    Vec3d gyro_bias_{0, 0, 0};
    double g_ = 0;

    cv::Mat key_image_;
    std::vector<cv::Point2f> key_raw_, key_level_, cur_raw_;
    Matx33d origin_to_key_ = Matx33d::eye();   // origin -> keyframe, level pixel coordinates
    Matx33d last_total_ = Matx33d::eye();      // origin -> last registered frame
    Matx33d origin_level_ = Matx33d::eye();
    bool rekey_requested_ = false;
    int key_inliers_ = 0;   // points that agreed on the first frame after this keyframe, 0 = not known yet

    std::deque<TrackSample> track_;
    Vec2d last_tilt_shift_{0, 0};
    int64_t last_t_ = 0;
    double fit_num_ = 0, fit_den_ = 0;
};

}  // namespace hold
