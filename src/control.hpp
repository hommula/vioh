// Turns position and height errors into the four stick values. Pure computation.
#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>

#include <opencv2/core.hpp>

#include "hold.hpp"

namespace hold {

struct Sticks {
    uint16_t roll = 1500, pitch = 1500, throttle = 1000, yaw = 1500;
};

// Height from pressure, relative to a reference pressure (standard atmosphere).
inline double pressureToHeight(double pa, double reference_pa) {
    return 44330.0 * (1.0 - std::pow(pa / reference_pa, 0.1903));
}

// Height and climb rate: accelerometer for the fast part, barometer for the slow part.
struct HeightFilter {
    double height = 0, climb = 0;
    bool started = false;

    // vertical_velocity_imu: integrated vertical acceleration (drifts; only its change is used).
    void update(double baro_height, double vertical_velocity_imu, double dt, double time_constant_s) {
        if (!started) {
            height = baro_height;
            climb = 0;
            started = true;
        } else {
            const double error = baro_height - height;
            height += (climb + 2.0 / time_constant_s * error) * dt;
            climb += (vertical_velocity_imu - last_imu_) + error * dt / (time_constant_s * time_constant_s);
        }
        last_imu_ = vertical_velocity_imu;
    }

private:
    double last_imu_ = 0;
};

class Controller {
public:
    explicit Controller(const Config& c)
        : kp_(c("pos_kp")), ki_(c("pos_ki")), kd_(c("pos_kd")), max_tilt_(c("max_tilt_deg") * CV_PI / 180),
          kp_z_(c("height_kp_us_per_m")), ki_z_(c("height_ki_us_per_m_s")), kd_z_(c("height_kd_us_per_m_s")),
          throttle_min_(c("throttle_min_us")), throttle_max_(c("throttle_max_us")),
          angle_limit_deg_(c("bf_angle_limit_deg")), centre_rate_(c("bf_rc_rate_deg_s")),
          max_rate_(c("bf_max_rate_deg_s")), deadband_(c("bf_deadband_us")),
          forward_(c("forward_from_x"), c("forward_from_y")), right_(c("right_from_x"), c("right_from_y")) {}

    void reset() {
        integral_ = Vec2d(0, 0);
        integral_z_ = 0;
    }

    // Stick offset from 1500 that makes Betaflight ANGLE mode ask for `tilt` rad.
    // Inverts angle = angle_limit * rate(x) / rate(1) with the ACTUAL rate curve at expo 0:
    // rate(x) = centre * x + (max - centre) * x * |x|   (Betaflight fc/rc.c, see docs/research.md 2.5).
    double tiltToStick(double tilt) const {
        const double wanted = std::min(std::abs(tilt) * 180 / CV_PI / angle_limit_deg_, 1.0) * max_rate_;
        const double a = max_rate_ - centre_rate_;
        const double x = a == 0 ? wanted / centre_rate_
                                : (-centre_rate_ + std::sqrt(centre_rate_ * centre_rate_ + 4 * a * wanted)) / (2 * a);
        const double us = x == 0 ? 0 : deadband_ + x * (500 - deadband_);
        return tilt < 0 ? -us : us;
    }

    // Height loop only: roll, pitch, yaw stay centred. Used when engaged but the camera estimate is not ready.
    Sticks holdHeight(double height_error, double climb, double hover_throttle, double dt) {
        integral_z_ += ki_z_ * height_error * dt;
        Sticks s;
        s.throttle = uint16_t(std::clamp(hover_throttle + kp_z_ * height_error - kd_z_ * climb + integral_z_,
                                         throttle_min_, throttle_max_));
        return s;
    }

    // error: target minus position, velocity: own velocity; both in current level IMU axes, metres.
    Sticks hold(const Vec2d& error, const Vec2d& velocity, double height_error, double climb, double hover_throttle, double dt) {
        Sticks s = holdHeight(height_error, climb, hover_throttle, dt);
        const double limit = std::tan(max_tilt_);
        integral_ += error * (ki_ * dt);
        integral_[0] = std::clamp(integral_[0], -limit * 9.81, limit * 9.81);   // never more than full tilt
        integral_[1] = std::clamp(integral_[1], -limit * 9.81, limit * 9.81);
        const Vec2d accel = error * kp_ - velocity * kd_ + integral_;          // m/s^2 wanted
        const double tilt_forward = std::atan(std::clamp(forward_.dot(accel) / 9.81, -limit, limit));
        const double tilt_right = std::atan(std::clamp(right_.dot(accel) / 9.81, -limit, limit));
        // Betaflight: pitch stick above 1500 tilts the nose down (flies forward), roll above 1500 tilts right.
        s.pitch = uint16_t(1500 + tiltToStick(tilt_forward));
        s.roll = uint16_t(1500 + tiltToStick(tilt_right));
        return s;
    }

private:
    double kp_, ki_, kd_, max_tilt_;
    double kp_z_, ki_z_, kd_z_, throttle_min_, throttle_max_;
    double angle_limit_deg_, centre_rate_, max_rate_, deadband_;
    Vec2d forward_, right_;   // rows that turn a level-IMU-axes vector into aircraft forward / right
    Vec2d integral_{0, 0};
    double integral_z_ = 0;
};

// Everything the command loop knows at one tick.
struct Inputs {
    int64_t now_ns = 0;
    // Flight controller, from MSP_RC.
    int64_t rc_t_ns = 0;          // when the last reply arrived, 0 = never
    uint16_t rc_throttle = 0, rc_arm = 0, rc_engage = 0;
    // Camera estimate.
    int64_t vision_t_ns = 0;      // when the last frame was processed, 0 = never
    bool tracking = false;
    int good_frames = 0;
    Vec2d U{0, 0}, U_rate{0, 0};
    double scale = 1, Z0 = 0, excitation = 0;
    cv::Matx22d origin_to_current = cv::Matx22d::eye();
    double vertical_velocity = 0;
    // Barometer.
    int64_t baro_t_ns = 0;        // when the last value arrived, 0 = never
    double baro_pa = 0;
};

// Decides what to send to the flight controller at each tick. Pure: no devices, no clocks.
// Rules from Stage 2 (sections 03 and 05):
//   - always produce a command, from the first tick;
//   - engage switch off: roll, pitch, yaw 1500 and the pilot's own throttle (the FC ignores it anyway);
//   - engage switch on, estimate not ready: roll, pitch, yaw 1500 and a throttle that holds height;
//   - engage switch on, estimate ready: hold the position and height seen at engage.
class Pilot {
public:
    // Read by the display and the log.
    bool engaged = false, ready = false, fc_alive = false, baro_alive = false, camera_alive = false;
    bool engage_edge = false;        // the switch went on at this tick
    Vec2d shown{0, 0};               // position since the last zero, origin level axes, m
    double height_shown = 0;         // height since the last zero, m: barometer and accelerometer
    double camera_height_shown = 0;  // the same from the camera: the picture shrinks as the aircraft rises.
                                     // Needs the height at the zero (Z then). MEASURED on the board: indoors
                                     // the barometer wanders by 5 Pa (0.4 m) in a minute while held still.
    double Z = 0;                    // height above ground used for scale, m
    const char* height_source = "bench";
    double pilot_throttle;
    HeightFilter height;

    explicit Pilot(const Config& c)
        : pilot_throttle(c("hover_throttle_fallback_us")),
          controller_(c),
          switch_on_us_(c("switch_on_us")),
          ready_frames_(int(c("ready_frames"))),
          baro_min_height_m_(c("baro_min_height_m")),
          bench_height_m_(c("bench_height_m")),
          scale_min_excitation_(c("scale_min_excitation")),
          height_filter_s_(c("height_filter_s")),
          pilot_throttle_filter_s_(c("pilot_throttle_filter_s")),
          hover_throttle_(c("hover_throttle_fallback_us")),
          zero_Z_(c("bench_height_m")) {}

    // Zero the displayed position and height (bench tests: press Enter over the start mark).
    void zero() { zero_wanted_ = true; }

    Sticks step(const Inputs& in, double dt) {
        // The barometer gets longer: if the driver resets the chip, setting it up again takes 2 s,
        // and the height carries on from the accelerometer meanwhile. That must not drop the hold.
        const int64_t kStaleNs = 500000000, kCameraStaleNs = 200000000, kBaroStaleNs = 3000000000;
        fc_alive = in.rc_t_ns != 0 && in.now_ns - in.rc_t_ns < kStaleNs;
        baro_alive = in.baro_t_ns != 0 && in.now_ns - in.baro_t_ns < kBaroStaleNs;
        camera_alive = in.vision_t_ns != 0 && in.now_ns - in.vision_t_ns < kCameraStaleNs;
        const bool armed = fc_alive && in.rc_arm >= switch_on_us_;
        const bool was_engaged = engaged;
        engaged = fc_alive && in.rc_engage >= switch_on_us_;
        engage_edge = engaged && !was_engaged;

        // Height: barometer change, smoothed with the accelerometer.
        if (baro_alive && reference_pa_ == 0) reference_pa_ = in.baro_pa;
        if (baro_alive && in.baro_t_ns != last_baro_t_ns_) {
            height.update(pressureToHeight(in.baro_pa, reference_pa_), in.vertical_velocity,
                          (in.baro_t_ns - last_baro_t_ns_) * 1e-9, height_filter_s_);
            last_baro_t_ns_ = in.baro_t_ns;
        }
        if (armed && !was_armed_ && baro_alive) {   // the aircraft is on the ground when the pilot arms
            ground_height_ = height.height;
            have_ground_ = true;
        }
        if (!armed) have_ground_ = false;
        was_armed_ = armed;

        // Height above ground, for turning image shift into metres. Best source first.
        const double above_ground = height.height - ground_height_;
        Z = bench_height_m_;                          // Stage 2 bench protocol: "about 1 m above the floor"
        height_source = "bench";
        if (have_ground_ && above_ground >= baro_min_height_m_) {
            Z = above_ground;
            height_source = "baro";
        } else if (in.excitation >= scale_min_excitation_ && in.Z0 > 0) {
            Z = in.Z0 / in.scale;                     // accelerometer fit: needs the aircraft to have been moved
            height_source = "accel";
        }
        const double Z_origin = Z * in.scale;         // U is in units of the origin height

        const bool vision_ready = camera_alive && in.tracking && in.good_frames >= ready_frames_;
        ready = vision_ready && baro_alive && fc_alive;

        // Engage edge: remember where to hold.
        if (engage_edge) {
            have_target_ = false;
            have_target_height_ = false;
            hover_throttle_ = pilot_throttle;
            controller_.reset();
            zero_wanted_ = true;
        }
        if (engaged && !have_target_ && vision_ready) {
            target_U_ = in.U;
            have_target_ = true;
        }
        if (engaged && !have_target_height_ && baro_alive) {
            target_height_ = height.height;
            have_target_height_ = true;
        }
        if (zero_wanted_) {
            zero_U_ = in.U;
            zero_height_ = height.height;
            zero_scale_ = in.scale;
            zero_Z_ = Z;
            zero_wanted_ = false;
        }
        shown = (in.U - zero_U_) * Z_origin;
        height_shown = height.height - zero_height_;
        camera_height_shown = zero_Z_ * (zero_scale_ / in.scale - 1);

        Sticks sticks;
        if (!engaged) {
            // The pilot is flying. Follow the pilot's throttle so that the first command at engage holds height.
            if (fc_alive) pilot_throttle += (in.rc_throttle - pilot_throttle) * (dt / (pilot_throttle_filter_s_ + dt));
            sticks.throttle = uint16_t(pilot_throttle);
            return sticks;
        }
        const double height_error = have_target_height_ ? target_height_ - height.height : 0;
        const double climb = have_target_height_ ? height.climb : 0;
        if (!ready || !have_target_) return controller_.holdHeight(height_error, climb, hover_throttle_, dt);
        const Vec2d error = in.origin_to_current * ((target_U_ - in.U) * Z_origin);
        const Vec2d velocity = in.origin_to_current * (in.U_rate * Z_origin);
        return controller_.hold(error, velocity, height_error, climb, hover_throttle_, dt);
    }

private:
    Controller controller_;
    double switch_on_us_;
    int ready_frames_;
    double baro_min_height_m_, bench_height_m_, scale_min_excitation_, height_filter_s_, pilot_throttle_filter_s_;

    double hover_throttle_;
    double reference_pa_ = 0, ground_height_ = 0, target_height_ = 0, zero_height_ = 0;
    double zero_scale_ = 1, zero_Z_;       // image scale and height above ground at the last zero
    int64_t last_baro_t_ns_ = 0;
    bool have_ground_ = false, was_armed_ = false, have_target_ = false, have_target_height_ = false, zero_wanted_ = false;
    Vec2d target_U_{0, 0}, zero_U_{0, 0};
};

}  // namespace hold
