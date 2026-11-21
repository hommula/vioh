// Checks that need no hardware: MSP bytes, stick mapping, controller signs. Exit code 0 = all passed.
#include <cstdio>
#include <cstdlib>

#include "control.hpp"
#include "msp.hpp"

static int failures = 0;
#define CHECK(condition) \
    if (!(condition)) { std::printf("FAILED line %d: %s\n", __LINE__, #condition); failures++; }

int main(int argc, char** argv) {
    if (argc != 2) return 2;
    const hold::Config base(argv[1]);

    // MSP_RC request, the well-known 6 bytes.
    CHECK(msp::request(msp::kRc) == (std::vector<uint8_t>{'$', 'M', '<', 0, 105, 105}));

    // MSP_SET_RAW_RC: 16 payload bytes, order roll, pitch, throttle, yaw, little-endian.
    const std::vector<uint8_t> set = msp::setRawRc(1501, 1502, 1503, 1504);
    CHECK(set.size() == 22 && set[3] == 16 && set[4] == 200);
    CHECK(set[5] == (1501 & 0xFF) && set[6] == (1501 >> 8) && set[9] == (1503 & 0xFF) && set[11] == (1504 & 0xFF));

    // Parser: accepts a reply, rejects a corrupted one.
    std::vector<uint8_t> reply = msp::request(msp::kRc, {0xDC, 0x05, 0xE8, 0x03});
    reply[2] = '>';
    msp::Parser parser;
    bool done = false;
    for (uint8_t b : reply) done = parser.feed(b);
    CHECK(done && parser.cmd == msp::kRc && msp::u16(parser.payload, 0) == 1500 && msp::u16(parser.payload, 1) == 1000);
    reply[6] ^= 1;
    done = false;
    for (uint8_t b : reply) done = parser.feed(b) || done;
    CHECK(!done);

    // Stick mapping: forward Betaflight formula applied to our inverse gives the tilt back.
    hold::Config config = base;
    config.values["forward_from_x"] = 1;
    config.values["right_from_y"] = 1;
    hold::Controller controller(config);
    const double limit = config("bf_angle_limit_deg"), centre = config("bf_rc_rate_deg_s"), top = config("bf_max_rate_deg_s"),
                 deadband = config("bf_deadband_us");
    for (double degrees : {-20.0, -5.0, -0.5, 0.5, 3.0, 10.0, 29.0}) {
        const double us = controller.tiltToStick(degrees * CV_PI / 180);
        const double x = (std::abs(us) - deadband) / (500 - deadband);
        const double back = limit * (centre * x + (top - centre) * x * x) / top * (us < 0 ? -1 : 1);
        CHECK(std::abs(back - degrees) < 1e-6);
    }
    CHECK(controller.tiltToStick(0) == 0);
    CHECK(std::abs(controller.tiltToStick(1.0)) <= 500);   // beyond angle_limit: full stick, no more

    // Controller signs: target ahead -> pitch above 1500; target to the right -> roll above 1500;
    // too low -> more throttle; climbing -> less throttle.
    hold::Sticks s = controller.hold({1, 0}, {0, 0}, 0, 0, 1500, 0.02);
    CHECK(s.pitch > 1500 && s.roll == 1500 && s.yaw == 1500);
    controller.reset();
    s = controller.hold({0, 1}, {0, 0}, 0, 0, 1500, 0.02);
    CHECK(s.roll > 1500 && s.pitch == 1500);
    controller.reset();
    s = controller.hold({0, 0}, {1, 0}, 0, 0, 1500, 0.02);
    CHECK(s.pitch < 1500);                                  // moving forward -> brake
    controller.reset();
    CHECK(controller.holdHeight(1, 0, 1500, 0.02).throttle > 1500);
    controller.reset();
    CHECK(controller.holdHeight(0, 1, 1500, 0.02).throttle < 1500);
    controller.reset();
    s = controller.holdHeight(100, 0, 1500, 0.02);
    CHECK(s.throttle == config("throttle_max_us") && s.roll == 1500 && s.pitch == 1500);

    // Unmeasured axis signs (all zero, as shipped) must leave roll and pitch centred.
    hold::Controller unset(base);
    s = unset.hold({5, -5}, {1, 1}, 0, 0, 1500, 0.02);
    CHECK(s.roll == 1500 && s.pitch == 1500);

    // Height from pressure: 12 Pa lower is about 1 m higher near sea level.
    const double h = hold::pressureToHeight(101313, 101325);
    CHECK(h > 0.95 && h < 1.05);

    // ---- Command loop scenarios (Stage 2 bench steps 5 to 8), 50 Hz ticks -----------------------------
    {
        hold::Pilot pilot(config);   // forward = IMU x, right = IMU y
        hold::Inputs in;
        const double minimum = config("throttle_min_us");
        auto tick = [&]() {
            in.now_ns += 20000000;
            const hold::Sticks out = pilot.step(in, 0.02);
            CHECK(out.yaw == 1500 && out.throttle >= minimum && out.roll >= 1000 && out.roll <= 2000 && out.pitch >= 1000 && out.pitch <= 2000);
            return out;
        };
        auto sensorsOk = [&]() {
            in.rc_t_ns = in.vision_t_ns = in.baro_t_ns = in.now_ns + 20000000;
            in.tracking = true;
            in.good_frames = 1000;
        };

        // Nothing connected yet (first tick after power-up): centred sticks, a hover throttle, not ready.
        hold::Sticks s = tick();
        CHECK(s.roll == 1500 && s.pitch == 1500 && s.throttle == config("hover_throttle_fallback_us") && !pilot.ready);

        // Pilot flying at throttle 1430, switch off: we follow that throttle, sticks centred.
        in.rc_throttle = 1430;
        in.rc_arm = 2000;
        in.rc_engage = 1000;
        in.baro_pa = 101000;
        for (int i = 0; i < 300; i++) {
            sensorsOk();
            s = tick();
        }
        CHECK(pilot.ready && !pilot.engaged && s.roll == 1500 && s.pitch == 1500 && std::abs(s.throttle - 1430) <= 1);

        // Engage, ready, not moved: centred, throttle = what the pilot had.
        in.rc_engage = 2000;
        in.rc_throttle = 885;        // MSP_RC now echoes OUR channels; it must not be taken as the pilot throttle
        sensorsOk();
        s = tick();
        CHECK(pilot.engaged && pilot.engage_edge && s.roll == 1500 && s.pitch == 1500 && std::abs(s.throttle - 1430) <= 1);

        // Bench step 5: aircraft moved forward (U grows along IMU x) -> pitch back. Moved right -> roll left.
        in.U = hold::Vec2d(0.5, 0);
        sensorsOk();
        s = tick();
        CHECK(s.pitch < 1500 && s.roll == 1500);
        in.U = hold::Vec2d(0, 0.5);
        sensorsOk();
        s = tick();
        CHECK(s.roll < 1500);
        in.U = hold::Vec2d(-0.5, -0.5);
        sensorsOk();
        s = tick();
        CHECK(s.pitch > 1500 && s.roll > 1500);

        // Bench step 7: camera covered (tracking lost) -> at once roll, pitch, yaw 1500, throttle still sane.
        sensorsOk();
        in.tracking = false;
        in.good_frames = 0;
        s = tick();
        CHECK(!pilot.ready && s.roll == 1500 && s.pitch == 1500 && s.throttle > 1300 && s.throttle < 1560);
        // Uncovered, but not yet enough good frames: still centred.
        sensorsOk();
        in.good_frames = 1;
        s = tick();
        CHECK(!pilot.ready && s.roll == 1500 && s.pitch == 1500);
        // Tracking again: commands return, towards the same target as before the cover.
        sensorsOk();
        s = tick();
        CHECK(pilot.ready && s.pitch > 1500 && s.roll > 1500);

        // Camera thread stalls (no new frames for 0.3 s) -> not ready, centred.
        for (int i = 0; i < 15; i++) {
            const int64_t stale = in.vision_t_ns;
            sensorsOk();
            in.vision_t_ns = stale;
            s = tick();
        }
        CHECK(!pilot.ready && !pilot.camera_alive && s.roll == 1500 && s.pitch == 1500);

        // Engaged and sinking 1 m (pressure up 12 Pa) -> more throttle than at engage.
        in.baro_pa = 101012;
        for (int i = 0; i < 100; i++) {
            sensorsOk();
            s = tick();
        }
        CHECK(s.throttle > 1440);

        // Switch off -> back to following the pilot, centred.
        in.rc_engage = 1000;
        in.rc_throttle = 1430;
        sensorsOk();
        s = tick();
        CHECK(!pilot.engaged && s.roll == 1500 && s.pitch == 1500);
    }
    {
        // Bench step 8: the program starts while the switch is already on. The first command is safe.
        hold::Pilot pilot(config);
        hold::Inputs in;
        in.now_ns = 1000000000;
        in.rc_t_ns = in.now_ns;
        in.rc_engage = 2000;
        in.rc_throttle = 885;
        const hold::Sticks s = pilot.step(in, 0.02);
        CHECK(pilot.engaged && s.roll == 1500 && s.pitch == 1500 && s.yaw == 1500 && s.throttle == config("hover_throttle_fallback_us"));
    }

    std::printf(failures ? "%d check(s) FAILED\n" : "all checks passed\n", failures);
    return failures ? 1 : 0;
}
