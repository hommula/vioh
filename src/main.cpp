// vioh: GNSS-free position hold for the Orqa M2-AI, running on the APB.
//
//   vioh config.txt log_dir                         fly: estimate, control, talk to the flight controller
//   vioh config.txt record out.vlog seconds nth     save IMU + every nth camera frame (for `calib` and `replay`)
//
// Inputs: nadir camera, APB IMU, APB barometer, and the RC channels from the flight controller (MSP_RC).
// GNSS: this program never asks the flight controller for GPS, altitude or attitude. MSP_ALTITUDE and the
// MSP_ATTITUDE yaw can contain GPS (docs/research.md 2.8), so they are not used either.
//
// Safety (Stage 2, section 03): the flight controller has no command timeout and reads minimum values
// before the first command. So the 50 Hz command loop starts first and never waits for a sensor.
#include <poll.h>
#include <time.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdio>
#include <iostream>
#include <mutex>
#include <thread>

#include "control.hpp"
#include "hold.hpp"
#include "io.hpp"
#include "msp.hpp"

using hold::Vec2d;

// Event configuration, Stage 2 section 02 and 03.
const char* const kCameraDevice = "/dev/video2";
const int kCameraWidth = 1920, kCameraHeight = 1080;
const char* const kFcDevice = "/dev/ttymxc2";
const int kFcBaud = 115200;
const int kLoopHz = 50;

static int64_t nowNs() {
    timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return int64_t(ts.tv_sec) * 1000000000 + ts.tv_nsec;
}

// Live view for tools/live_view.py: a half-size copy of the frame, as a PGM file in memory (/dev/shm is
// not the eMMC). Written under another name and renamed, so a reader never sees half a file.
// A development aid: if it cannot be written the flight code carries on, and says so once.
const char* const kPreviewPath = "/dev/shm/nadirhold_preview.pgm";
const int kPreviewEveryNthFrame = 12;   // 5 pictures per second at 60 frames per second
// The numbers for tools/live_view.py: one line of text starting with '#', same folder.
// The dashboard asks for a zero of the displayed position by creating the second file.
const char* const kZeroRequestPath = "/dev/shm/nadirhold_zero";
const char* const kStatusPath = "/dev/shm/nadirhold_status.txt";

static void writeStatus(const char* text) {
    const std::string temporary = std::string(kStatusPath) + ".tmp";
    std::FILE* out = std::fopen(temporary.c_str(), "w");
    if (!out) return;   // development aid: the preview already complains once if /dev/shm is not writable
    std::fprintf(out, "# %s\n", text);
    std::fclose(out);
    std::rename(temporary.c_str(), kStatusPath);
}

static void writePreview(const cv::Mat& gray) {
    static bool complained = false;
    cv::Mat small;
    cv::resize(gray, small, cv::Size(), 0.5, 0.5, cv::INTER_AREA);
    const std::string temporary = std::string(kPreviewPath) + ".tmp";
    std::FILE* out = std::fopen(temporary.c_str(), "wb");
    if (!out) {
        if (!complained) std::cerr << "preview: cannot write " << temporary << "\n";
        complained = true;
        return;
    }
    std::fprintf(out, "P5\n%d %d\n255\n", small.cols, small.rows);
    for (int y = 0; y < small.rows; y++) std::fwrite(small.ptr(y), 1, small.cols, out);
    std::fclose(out);
    std::rename(temporary.c_str(), kPreviewPath);
}

// What the sensor threads hand to the command loop.
struct Shared {
    std::mutex mutex;
    // vision thread
    int64_t vision_t_ns = 0;      // when the estimate was last updated
    bool tracking = false;
    int good_frames = 0, inliers = 0;
    Vec2d U{0, 0}, U_rate{0, 0};
    Vec2d tilt_rad{0, 0};         // about the IMU x and y axes, from the IMU alone
    double frames_per_s = 0, imu_per_s = 0;
    cv::Vec3d gyro{0, 0, 0}, acc{0, 0, 0};   // the newest IMU sample, rad/s and m/s^2
    int64_t frames_replaced = 0;             // frames that arrived while the estimator was still busy
    double scale = 1, Z0 = 0, excitation = 0, vertical_velocity = 0, frame_ms = 0;
    cv::Matx22d origin_to_current = cv::Matx22d::eye();
    bool keyframe_wanted = false;
    // barometer thread
    int64_t baro_t_ns = 0;
    double baro_pa = 0;
};

// The newest camera frame, handed from the capture thread to the estimator thread. One slot: if the
// estimator is still busy when the next frame arrives, the older frame is replaced and counted.
struct FrameSlot {
    std::mutex mutex;
    std::condition_variable arrived;
    cv::Mat gray;
    int64_t t_ns = 0;
    bool fresh = false;
    int64_t replaced = 0;
};

// Copying a frame out of the camera driver takes about as long as tracking it (MEASURED on the board:
// both near 13 ms, and one frame comes every 16.7 ms). So the copy has its own thread.
static void captureThread(FrameSlot& slot) {
    for (;;) {
        try {
            io::Camera camera(kCameraDevice, kCameraWidth, kCameraHeight);
            cv::Mat gray;
            for (int64_t frames = 0;;) {
                int64_t t_ns = 0;
                if (!camera.read(gray, t_ns, 100)) continue;
                if (frames++ % kPreviewEveryNthFrame == 0) writePreview(gray);
                std::lock_guard<std::mutex> lock(slot.mutex);
                if (slot.fresh) slot.replaced++;
                cv::swap(slot.gray, gray);   // the three pictures (here, slot, estimator) only change hands
                slot.t_ns = t_ns;
                slot.fresh = true;
                slot.arrived.notify_one();
            }
        } catch (const std::exception& e) {
            std::cerr << "camera: " << e.what() << " (retry in 1 s)\n";
            sleep(1);
        }
    }
}

static void estimatorThread(const hold::Config& config, FrameSlot& slot, Shared& shared) {
    for (;;) {
        try {
            io::Imu imu(int(config("imu_rate_hz")));
            hold::Estimator estimator(config);
            std::vector<io::ImuSample> samples;
            cv::Mat gray;
            int64_t rate_start_ns = nowNs();
            int rate_frames = 0;
            double frames_per_s = 0, imu_per_s = 0;
            int64_t imu_rate_start_ns = nowNs(), frames_replaced = 0;
            size_t imu_rate_samples = 0;
            cv::Vec3d gyro(0, 0, 0), acc(0, 0, 0);
            for (;;) {
                // Wait for a frame, but not for ever: the IMU must be read even when the camera is dead.
                int64_t frame_t = 0;
                bool got_frame = false;
                {
                    std::unique_lock<std::mutex> lock(slot.mutex);
                    got_frame = slot.arrived.wait_for(lock, std::chrono::milliseconds(100), [&slot] { return slot.fresh; });
                    if (got_frame) {
                        cv::swap(gray, slot.gray);
                        frame_t = slot.t_ns;
                        slot.fresh = false;
                    }
                    frames_replaced = slot.replaced;
                }
                samples.clear();
                imu.read(samples);
                for (const io::ImuSample& s : samples)
                    estimator.imu(s.t_ns, cv::Vec3d(s.gyro[0], s.gyro[1], s.gyro[2]), cv::Vec3d(s.acc[0], s.acc[1], s.acc[2]));
                if (!samples.empty()) {
                    const io::ImuSample& s = samples.back();
                    gyro = cv::Vec3d(s.gyro[0], s.gyro[1], s.gyro[2]);
                    acc = cv::Vec3d(s.acc[0], s.acc[1], s.acc[2]);
                }
                imu_rate_samples += samples.size();
                if (nowNs() - imu_rate_start_ns >= 1000000000) {   // IMU samples per second, over 1 s
                    imu_per_s = imu_rate_samples * 1e9 / double(nowNs() - imu_rate_start_ns);
                    imu_rate_start_ns = nowNs();
                    imu_rate_samples = 0;
                }
                if (!estimator.hasImu()) continue;

                std::unique_lock<std::mutex> lock(shared.mutex);
                const bool keyframe_wanted = shared.keyframe_wanted;
                shared.keyframe_wanted = false;
                lock.unlock();
                if (keyframe_wanted) estimator.requestKeyframe();

                const int64_t start = nowNs();
                if (got_frame) estimator.frame(frame_t, gray);
                const double frame_ms = (nowNs() - start) * 1e-6;
                if (got_frame && ++rate_frames == 60) {   // frames per second over the last 60 frames
                    frames_per_s = 60e9 / double(nowNs() - rate_start_ns);
                    rate_start_ns = nowNs();
                    rate_frames = 0;
                }

                lock.lock();
                shared.vertical_velocity = estimator.verticalVelocity();
                shared.tilt_rad = estimator.tiltNow();
                shared.frames_per_s = frames_per_s;
                shared.imu_per_s = imu_per_s;
                shared.gyro = gyro;
                shared.acc = acc;
                shared.frames_replaced = frames_replaced;
                if (got_frame) {
                    shared.vision_t_ns = nowNs();
                    shared.tracking = estimator.tracking;
                    shared.good_frames = estimator.good_frames;
                    shared.inliers = estimator.inliers;
                    shared.U = estimator.U;
                    shared.U_rate = estimator.U_rate;
                    shared.scale = estimator.scale;
                    shared.Z0 = estimator.Z0;
                    shared.excitation = estimator.excitation;
                    shared.origin_to_current = estimator.originToCurrent();
                    shared.frame_ms = frame_ms;
                }
            }
        } catch (const std::exception& e) {
            std::cerr << "estimator: " << e.what() << " (retry in 1 s)\n";
            sleep(1);
        }
    }
}

// The barometer class sets the chip up itself and reads about 16 values per second (see io::Baro).
static void baroThread(const hold::Config& config, Shared& shared) {
    for (;;) {
        try {
            io::Baro baro(int(config("baro_rate_hz")), int(config("baro_oversampling")));
            int reported_resets = 0;
            for (;;) {
                double pa = 0;
                const bool usable = baro.read(pa);
                if (baro.resets != reported_resets) {
                    std::cerr << "baro: the driver reset the chip, set it up again (" << baro.resets << " so far)\n";
                    reported_resets = baro.resets;
                }
                if (!usable) continue;
                std::lock_guard<std::mutex> lock(shared.mutex);
                shared.baro_pa = pa;
                shared.baro_t_ns = nowNs();
            }
        } catch (const std::exception& e) {
            std::cerr << "baro: " << e.what() << " (retry in 1 s)\n";
            sleep(1);
        }
    }
}

static int record(const hold::Config& config, const char* path, double seconds, int every_nth) {
    io::Camera camera(kCameraDevice, kCameraWidth, kCameraHeight);
    io::Imu imu(int(config("imu_rate_hz")));
    std::FILE* out = std::fopen(path, "wb");
    if (!out) throw std::runtime_error(std::string("cannot write ") + path);
    writeStatus("recording: no estimate in this mode");
    std::vector<io::ImuSample> samples;
    cv::Mat gray;
    const int64_t end = nowNs() + int64_t(seconds * 1e9);
    int frames = 0, saved = 0;
    size_t imu_count = 0;
    while (nowNs() < end) {
        int64_t t = 0;
        const bool got_frame = camera.read(gray, t, 100);
        samples.clear();
        imu.read(samples);
        for (const io::ImuSample& s : samples) {
            const float v[6] = {float(s.gyro[0]), float(s.gyro[1]), float(s.gyro[2]), float(s.acc[0]), float(s.acc[1]), float(s.acc[2])};
            std::fputc('I', out);
            std::fwrite(&s.t_ns, 8, 1, out);
            std::fwrite(v, sizeof v, 1, out);
        }
        imu_count += samples.size();
        if (got_frame && frames % kPreviewEveryNthFrame == 0) writePreview(gray);
        if (got_frame && frames++ % every_nth == 0) {
            const uint16_t size[2] = {uint16_t(gray.cols), uint16_t(gray.rows)};
            std::fputc('F', out);
            std::fwrite(&t, 8, 1, out);
            std::fwrite(size, sizeof size, 1, out);
            for (int y = 0; y < gray.rows; y++) std::fwrite(gray.ptr(y), 1, gray.cols, out);
            saved++;
        }
    }
    std::fclose(out);
    std::printf("camera frames %d (%.1f per s), saved %d, imu samples %zu (%.0f per s)\n", frames, frames / seconds, saved,
                imu_count, imu_count / seconds);
    return 0;
}

int main(int argc, char** argv) {
    if (argc != 3 && argc != 6) {
        std::cerr << "usage: vioh config.txt log_dir\n       vioh config.txt record out.vlog seconds every_nth_frame\n";
        return 2;
    }
    const hold::Config config(argv[1]);
    if (argc == 6) return record(config, argv[3], std::stod(argv[4]), std::stoi(argv[5]));

    const int arm_index = int(config("arm_channel")) - 1, engage_index = int(config("engage_channel")) - 1;

    io::Serial fc(kFcDevice, kFcBaud);
    hold::Pilot pilot(config);
    Shared shared;
    FrameSlot slot;
    std::thread(captureThread, std::ref(slot)).detach();
    std::thread(estimatorThread, std::cref(config), std::ref(slot), std::ref(shared)).detach();
    std::thread(baroThread, std::cref(config), std::ref(shared)).detach();

    const std::string log_path = std::string(argv[2]) + "/vioh_" + std::to_string(time(nullptr)) + ".csv";
    std::FILE* log = std::fopen(log_path.c_str(), "w");
    if (!log) std::cerr << "cannot write " << log_path << ", running without a log\n";
    if (log) std::fprintf(log, "t_s,engaged,ready,tracking,inliers,roll,pitch,throttle,yaw,x_m,y_m,height_m,climb,Z_m,"
                               "scale,excitation,baro_pa,pilot_throttle,frame_ms\n");

    msp::Parser parser;
    hold::Inputs in;
    const int64_t program_start_ns = nowNs();
    const double switch_on_us = config("switch_on_us");
    double first_ready_up_s = -1;   // seconds after power-on (the monotonic clock starts at boot) of the first READY
    const double dt = 1.0 / kLoopHz;
    timespec next;
    clock_gettime(CLOCK_MONOTONIC, &next);
    for (int64_t tick = 0;; tick++) {
        in.now_ns = nowNs();

        // RC channels from the flight controller.
        uint8_t buffer[256];
        for (size_t n = fc.read(buffer, sizeof buffer), i = 0; i < n; i++) {
            if (!parser.feed(buffer[i]) || parser.cmd != msp::kRc) continue;
            if (parser.payload.size() < 2 * size_t(std::max(arm_index, engage_index) + 1)) continue;
            in.rc_throttle = msp::u16(parser.payload, 3);   // MSP_RC order: roll, pitch, yaw, throttle
            in.rc_arm = msp::u16(parser.payload, arm_index);
            in.rc_engage = msp::u16(parser.payload, engage_index);
            in.rc_t_ns = in.now_ns;
        }

        // Sensor snapshot.
        int inliers;
        double frame_ms, frames_per_s, imu_per_s;
        Vec2d tilt_rad;
        cv::Vec3d gyro, acc;
        int64_t frames_replaced;
        {
            std::lock_guard<std::mutex> lock(shared.mutex);
            in.vision_t_ns = shared.vision_t_ns;
            in.tracking = shared.tracking;
            in.good_frames = shared.good_frames;
            in.U = shared.U;
            in.U_rate = shared.U_rate;
            in.scale = shared.scale;
            in.Z0 = shared.Z0;
            in.excitation = shared.excitation;
            in.origin_to_current = shared.origin_to_current;
            in.vertical_velocity = shared.vertical_velocity;
            in.baro_t_ns = shared.baro_t_ns;
            in.baro_pa = shared.baro_pa;
            inliers = shared.inliers;
            frame_ms = shared.frame_ms;
            frames_per_s = shared.frames_per_s;
            tilt_rad = shared.tilt_rad;
            imu_per_s = shared.imu_per_s;
            gyro = shared.gyro;
            acc = shared.acc;
            frames_replaced = shared.frames_replaced;
        }

        // Enter on the keyboard: zero the displayed position (bench tests).
        pollfd keyboard = {STDIN_FILENO, POLLIN, 0};
        char discard[64];
        if (poll(&keyboard, 1, 0) == 1 && (keyboard.revents & POLLIN) && read(STDIN_FILENO, discard, sizeof discard) > 0)
            pilot.zero();
        if (access(kZeroRequestPath, F_OK) == 0) {   // the same, asked for by the dashboard
            unlink(kZeroRequestPath);
            pilot.zero();
        }

        const hold::Sticks sticks = pilot.step(in, dt);
        if (pilot.engage_edge) {   // hold against the view at engage, so the hold cannot drift
            std::lock_guard<std::mutex> lock(shared.mutex);
            shared.keyframe_wanted = true;
        }

        const std::vector<uint8_t> command = msp::setRawRc(sticks.roll, sticks.pitch, sticks.throttle, sticks.yaw);
        const std::vector<uint8_t> ask_rc = msp::request(msp::kRc);
        fc.write(command.data(), command.size());
        fc.write(ask_rc.data(), ask_rc.size());

        if (log) {
            std::fprintf(log, "%.3f,%d,%d,%d,%d,%u,%u,%u,%u,%.3f,%.3f,%.3f,%.3f,%.3f,%.4f,%.2f,%.1f,%.0f,%.1f\n",
                         in.now_ns * 1e-9, pilot.engaged, pilot.ready, in.tracking, inliers, sticks.roll, sticks.pitch,
                         sticks.throttle, sticks.yaw, pilot.shown[0], pilot.shown[1], pilot.height_shown,
                         pilot.height.climb, pilot.Z, in.scale, in.excitation, in.baro_pa, pilot.pilot_throttle, frame_ms);
            if (tick % kLoopHz == 0) std::fflush(log);
        }
        if (tick % 10 == 0) {
            std::printf("%s %s | x %+6.2f y %+6.2f dist %5.2f m | height %+5.2f m (Z %5.2f %s) | pts %3d | "
                        "R %4u P %4u T %4u Y %4u | fc %s baro %s cam %s %4.1f ms\n",
                        pilot.ready ? "READY    " : "NOT READY", pilot.engaged ? "ENGAGED" : "pilot  ", pilot.shown[0],
                        pilot.shown[1], cv::norm(pilot.shown), pilot.height_shown, pilot.Z, pilot.height_source, inliers,
                        sticks.roll, sticks.pitch, sticks.throttle, sticks.yaw, pilot.fc_alive ? "ok" : "--",
                        pilot.baro_alive ? "ok" : "--", pilot.camera_alive ? "ok" : "--", frame_ms);
            std::fflush(stdout);

        }

        // For tools/live_view.py, 10 times a second: one line of name=value pairs.
        if (pilot.ready && first_ready_up_s < 0) first_ready_up_s = in.now_ns * 1e-9;
        if (tick % 5 == 0) {
            const double degrees = 180 / CV_PI;
            const double turn_deg = std::atan2(in.origin_to_current(1, 0), in.origin_to_current(0, 0)) * degrees;
            char status[800];
            std::snprintf(status, sizeof status,
                          "up=%.2f run=%.1f first_ready=%.1f ready=%d engaged=%d armed=%d tracking=%d points=%d "
                          "x=%.3f y=%.3f z=%.3f zcam=%.3f Z=%.2f zsrc=%s tiltx=%.2f tilty=%.2f turn=%.2f "
                          "roll=%u pitch=%u thr=%u yaw=%u rcthr=%u rcarm=%u rceng=%u "
                          "fps=%.1f ms=%.1f replaced=%lld imu=%.0f gx=%.3f gy=%.3f gz=%.3f ax=%.2f ay=%.2f az=%.2f "
                          "fc=%d baro=%d cam=%d pa=%.0f",
                          in.now_ns * 1e-9, (in.now_ns - program_start_ns) * 1e-9, first_ready_up_s, pilot.ready,
                          pilot.engaged, pilot.fc_alive && in.rc_arm >= switch_on_us, in.tracking, inliers,
                          pilot.shown[0], pilot.shown[1], pilot.height_shown, pilot.camera_height_shown, pilot.Z,
                          pilot.height_source, tilt_rad[0] * degrees, tilt_rad[1] * degrees, turn_deg,
                          sticks.roll, sticks.pitch, sticks.throttle, sticks.yaw,
                          unsigned(in.rc_throttle), unsigned(in.rc_arm), unsigned(in.rc_engage),
                          frames_per_s, frame_ms, static_cast<long long>(frames_replaced), imu_per_s,
                          gyro[0], gyro[1], gyro[2], acc[0], acc[1], acc[2],
                          pilot.fc_alive, pilot.baro_alive, pilot.camera_alive, in.baro_pa);
            writeStatus(status);
        }

        next.tv_nsec += 1000000000 / kLoopHz;
        if (next.tv_nsec >= 1000000000) {
            next.tv_nsec -= 1000000000;
            next.tv_sec++;
        }
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next, nullptr);
    }
}
