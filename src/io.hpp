// io.hpp - camera, IMU, serial port and barometer of the Orqa DTK-APB (NXP i.MX8M Plus, Linux 6.6.52).
// Header-only. Every failed system call throws std::runtime_error with the path and strerror.
//
// =====================================================================================================
// WHERE EACH HARDWARE FACT COMES FROM
// =====================================================================================================
// Source read: NXP linux-imx, git tag lf-6.6.52-2.2.0 (its Makefile says 6.6.52). Paths below are in
// that tree. "ABI" = Documentation/ABI/testing/sysfs-bus-iio. Nothing here was run on the board.
// The board kernel is "6.6.52-orqa-gd70a653e1e06-dirty": Orqa's fork of that tree. Orqa's own patches
// are not public, so every fact below is also checked at run time where a check is possible.
// Read from the board backup (kernel config embedded in /boot/Image): CONFIG_IMX8_ISI_CAPTURE=y,
// CONFIG_VIDEO_IMX8_ISI=y, CONFIG_INV_ICM42600=y (I2C and SPI), CONFIG_DPS310=y, CONFIG_IIO_KFIFO_BUF=y.
//
// 1. IMU SCAN ELEMENTS (drivers/iio/imu/inv_icm42600/, drivers/iio/industrialio-buffer.c)
//  - Files, gyro device: scan_elements/in_anglvel_{x,y,z}_en, in_temp_en, in_timestamp_en, plus a
//    _type and an _index file for each. Accel device: the same with in_accel_{x,y,z}.
//    (iio_buffer_add_channel_sysfs; "scan_elements" and "buffer" are the legacy directory names, still
//    created by iio_buffer_register_legacy_sysfs_groups; the same files are also in buffer0/.)
//  - x, y, z: signed 16 bit in 16 bit, BIG-endian, _type reads "be:s16/16>>0"
//    (INV_ICM42600_GYRO_CHAN / INV_ICM42600_ACCEL_CHAN: .endianness = IIO_BE; text format in
//    iio_show_fixed_type). The bytes are copied unchanged from the chip FIFO
//    (inv_icm42600_gyro_parse_fifo), and the chip is set to big-endian in inv_icm42600_setup.
//  - timestamp: "le:s64/64>>0" (IIO_CHAN_SOFT_TIMESTAMP, CPU endian, aarch64 is little-endian).
//  - _index: x=0, y=1, z=2, temp=3, timestamp=4 (enum inv_icm42600_gyro_scan / _accel_scan).
//  - The driver itself always scans x+y+z+temp (inv_icm42600_gyro_scan_masks). The IIO core removes
//    what is not enabled (iio_buffer_update_demux), so x, y, z without temp is allowed.
//  - One record with x, y, z, timestamp enabled and temp disabled is 16 bytes:
//    bytes 0-1 x, 2-3 y, 4-5 z, 6-7 padding, 8-15 timestamp. Each element is aligned to its own size
//    (iio_compute_scan_bytes). The constructor reads every _type and _index file and throws on any
//    other value.
//  - A sample with all three axes equal to -32768 is "invalid" and the driver drops it
//    (inv_icm42600_fifo_is_data_valid). This happens while a sensor starts up.
//  - The kernel does not apply the mount matrix (file mount_matrix): axes are the chip's axes.
//
// 2. SCALE (inv_icm42600_gyro_read_scale, inv_icm42600_accel_read_scale, ABI)
//  - in_anglvel_scale: rad/s per LSB. in_accel_scale: m/s^2 per LSB. One file per device.
//  - Driver defaults (inv_icm42600_default_conf): +-2000 deg/s = 0.001065264, +-16 g = 0.004788403.
//    This code does not change the range. It reads both files once in the constructor.
//
// 3. SAMPLING RATE (inv_icm42600_gyro_write_odr, inv_icm42600_accel_write_odr)
//  - File "sampling_frequency" in each device directory (info_mask_shared_by_all), one per device.
//  - Allowed for both: 12.5 25 50 100 200 1000 2000 4000 (inv_icm42600_gyro_odr, inv_icm42600_accel_odr).
//    Anything else (500, 8000, ...) gives EINVAL, and the constructor throws. 12.5 cannot be asked for
//    with an int.
//  - The driver does not force gyro and accel to the same rate. This code sets both to rate_hz, so
//    every FIFO packet carries one gyro and one accel sample.
//
// 4. IMU TIMESTAMPS (drivers/iio/common/inv_sensors/inv_sensors_timestamp.c, inv_icm42600_core.c,
//    drivers/iio/industrialio-core.c)
//  - The hard interrupt handler stores a time for each device with iio_get_time_ns
//    (inv_icm42600_irq_timestamp). That is the time the FIFO watermark interrupt fired.
//  - Sample timestamps are not that raw time. They advance by the measured sample period and are
//    pulled toward the interrupt time by at most 2 % of a period per interrupt
//    (inv_sensors_timestamp_interrupt, inv_align_timestamp_it). The sensor's filter delay is not removed.
//  - Clock: file current_timestamp_clock, one per device. Default "realtime" (clock_id 0). Writing
//    "monotonic" makes iio_get_time_ns use ktime_get_ns, the same clock as the camera. The write gives
//    EBUSY while the buffer is enabled (iio_device_set_clock). Gyro and accel are two IIO devices, so
//    both files are written.
//  - KNOWN DEFECT in 6.6.52, read in inv_icm42600_buffer_fifo_parse: a device's timestamp is also
//    updated by interrupts in which that device has no valid sample. While the second sensor starts
//    (accel: 20 ms, INV_ICM42600_ACCEL_STARTUP_TIME_MS) its timestamps fall behind by up to that time
//    and then recover at 2 % per sample, about 1 s (worked out from the code, not measured). This
//    code enables the gyro first: its FIFO only starts after the gyro's 60 ms start-up wait
//    (iio_enable_buffers calls update_scan_mode before postenable), so the gyro timestamps, the ones in
//    ImuSample::t_ns, should be right from the first sample. The accel pairing can be off by up to
//    20 ms during the first second. Mainline today skips a device that has no sample in the interrupt
//    (inv_sensors_timestamp_interrupt in torvalds/linux master); 6.6.52 does not. See UNVERIFIED (c).
//
// 5. IMU BUFFER CONTROL (drivers/iio/industrialio-buffer.c, drivers/iio/buffer/kfifo_buf.c)
//  - Order used: buffer/enable 0, current_timestamp_clock, sampling_frequency, scan_elements/*_en,
//    buffer/length, buffer/watermark, buffer/enable 1. The clock, the scan elements, the length and
//    the watermark all give EBUSY while the buffer is enabled. The watermark must be <= length
//    (watermark_store), so length is written first. The rate must be written before enable: it is
//    applied when the buffer starts (inv_icm42600_gyro_update_scan_mode).
//  - buffer/length defaults to 2 samples (iio_kfifo_allocate). When the kernel buffer is full, new
//    samples are dropped without any error (iio_store_to_kfifo returns -EBUSY, the driver ignores it).
//  - buffer/watermark is passed to the chip FIFO (hwfifo_set_watermark). 1 = one interrupt per sample.
//  - /dev/iio:deviceN can be opened by one process only; a second open gives EBUSY (iio_chrdev_open).
//  - read() with O_NONBLOCK on an empty buffer gives EAGAIN (iio_buffer_read). It returns whole
//    records only (kfifo with element size = record size, iio_read_kfifo), and EINVAL if the user
//    buffer is smaller than one record.
//  - A non-blocking read() that asks for more records than are waiting first reads the chip FIFO over
//    the bus, inside the read() call (iio_buffer_ready -> inv_icm42600_gyro_hwfifo_flush). So
//    Imu::read() never waits for data, but each call costs bus transfers.
//  - The two devices share one chip FIFO (inv_icm42600_buffer_postenable counts users in st->fifo.on).
//    Enabling the second buffer changes the FIFO packet from 8 to 16 bytes; disabling one buffer leaves
//    the other running. Each interrupt, and each flush from either device, fills both kernel buffers.
//
// 6. CAMERA (drivers/staging/media/imx/imx8-isi-cap.c, imx8-isi-core.c, imx8-isi-fmt.c)
//  - Verified driver: the NXP downstream one, imx8-isi-cap.c. Its name is "mxc-isi-cap_v1"
//    (MXC_ISI_CAPTURE in drivers/staging/media/imx/imx8-isi-core.h), which is what the board reports.
//    The mainline driver (drivers/media/platform/nxp/imx8-isi) is also built into the board kernel but
//    calls itself "mxc-isi". The constructor throws if the driver name is anything else.
//  - Capture type is V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE only (mxc_isi_cap_querycap).
//  - YUYV ("YUYV-16" in mxc_isi_out_formats): one plane, 16 bits per pixel, bytes Y0 U Y1 V.
//  - S_FMT (mxc_isi_cap_fmt_try): an unknown pixel format is silently replaced by RGB565, and the size
//    is rounded, so the constructor compares what comes back. plane_fmt[0].bytesperline is
//    max(what the caller passed, width * 2); this code passes 0 and reads the result back.
//    sizeimage = bytesperline * height.
//  - mmap and poll are supported (io_modes = VB2_MMAP | VB2_DMABUF, .poll = vb2_fop_poll). At least 2
//    buffers must be queued before the driver starts (min_buffers_needed = 2). The chip always holds 2.
//  - Timestamp: ktime_get_ns() = CLOCK_MONOTONIC, taken in the interrupt handler when the chip reports
//    "frame stored" (mxc_isi_irq_handler: CHNL_STS_FRM_STRD -> mxc_isi_cap_frame_write_done). That is
//    the END of the frame, as the organizers say. Flag: V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC.
//    V4L2 stores it as seconds + microseconds, so the resolution is 1 us.
//  - If the program is too slow, frames are written to a throw-away buffer and lost (out_discard).
//  - STREAMON makes the KERNEL set the format of the next subdev to width x height with code
//    MEDIA_BUS_FMT_UYVY8_1X16 = 0x200f (mxc_isi_cap_streamon -> mxc_isi_source_fmt_init), and NXP's
//    CSI receiver passes that call on to the sensor (mipi_csis_set_fmt in imx8-mipi-csi2-sam.c). This
//    file never opens a subdev, but see UNVERIFIED (b).
//
// 7. BAROMETER (drivers/iio/pressure/dps310.c, ABI, Infineon DPS310 data sheet)
//  - Files (dps310_channels): in_pressure_input, in_pressure_sampling_frequency,
//    in_pressure_oversampling_ratio, and the same three with in_temp. No buffer, no timestamp.
//  - in_pressure_input is in kilopascal (ABI; dps310_read_pressure returns Pa / 1000). The driver
//    computes whole pascals (dps310_calculate_pressure returns int), so one step is 1 Pa, about 8 cm.
//  - Each read waits for a NEW conversion: dps310_read_pres_raw polls the PRS_RDY bit, and the chip
//    clears that bit when the pressure is read (data sheet, register MEAS_CFG). Nothing is cached.
//    A polling loop gets in_pressure_sampling_frequency fresh values per second, never more, and one
//    read blocks for up to 1 / in_pressure_sampling_frequency seconds.
//  - Driver default after boot (dps310_startup): 1 Hz, oversampling 1. So readPa() blocks up to 1 s.
//  - Allowed values, both files: 1 2 4 8 16 32 64 128 (dps310_set_pres_samp_freq and
//    dps310_set_pres_precision use ilog2; other values are rounded down without an error). The data
//    sheet requires rate_temp * time_temp + rate_pressure * time_pressure < 1 s, with time = 3.6, 5.2,
//    8.4, 14.8, 27.6, 53.2, 104.4, 206.8 ms for oversampling 1..128. The driver does not check this.
//  - If a conversion does not arrive in 1 / rate seconds the driver resets the chip (dps310_ready ->
//    dps310_reset_reinit), which puts both files back to 1 Hz and 1, and the read fails with ETIMEDOUT.
//  - This code sets neither file. BLANK: choose rate and oversampling on the board.
//
// 8. SERIAL (drivers/tty/n_tty.c, drivers/tty/serial/imx.c)
//  - With VMIN = 0 and VTIME = 0 a read with nothing waiting returns 0, not EAGAIN (n_tty_read).
//  - The driver writes the baud rate it really reached back into termios (imx_uart_set_termios ->
//    tty_termios_encode_baud_rate, 2 % tolerance), so the constructor reads it back and compares.
//
// UNVERIFIED, check on the board
//  (a) That Orqa's kernel matches NXP's tag for the files above. The checklist covers what shows.
//  (b) What Orqa's "dummy_csi2_sensor" driver (drivers/media/i2c/dummy_sensor.c, source not public)
//      does when STREAMON asks it for code 0x200f. It may replace the organizers' 0x2006 setting.
//      for m in /dev/media*; do media-ctl -d $m -p; done > /tmp/before
//      (run the program, stop it, then)  for m in /dev/media*; do media-ctl -d $m -p; done | diff /tmp/before -
//  (c) The size of the accel timestamp lag in 4. In the program, for the first 3 s, print
//      clock_gettime(CLOCK_MONOTONIC) minus t_ns of the newest sample. Expect under about 2 ms.
//  (d) Nothing else holds the devices (the organizers' rth-flight-logger service, apb-rtsp):
//      fuser -v /dev/video2 /dev/ttymxc2 /dev/iio:device1 /dev/iio:device2
//  (e) IMU bus (I2C or SPI). At 1000 Hz with watermark 1 a slow I2C bus may not keep up:
//      readlink -f /sys/bus/iio/devices/iio:device1 ; dmesg | grep -i "FIFO full"
//  (f) Barometer rate and oversampling (blank, see 7).
//
// CHECKLIST, run on the board (about 2 minutes). G=/sys/bus/iio/devices/iio:device1, A=...device2
//  1. uname -r; v4l2-ctl -d /dev/video2 --info | grep -i driver        -> 6.6.52-orqa..., mxc-isi-cap_v1
//  2. fuser -v /dev/video2 /dev/ttymxc2 /dev/iio:device1 /dev/iio:device2   -> no other process
//  3. v4l2-ctl -d /dev/video2 --set-fmt-video=width=1920,height=1080,pixelformat=YUYV --stream-mmap=4
//       --stream-count=120 --verbose 2>&1 | tail -4      -> bytesused 4147200, ts-monotonic, 60 fps
//  4. the two media-ctl commands of UNVERIFIED (b), around step 3      -> no difference
//  5. grep . /sys/bus/iio/devices/iio:device*/name        -> dps310, icm42605-gyro, icm42605-accel
//  6. grep . $G/scan_elements/*_type $G/scan_elements/*_index $A/scan_elements/*_type
//       -> x, y, z be:s16/16>>0 index 0 1 2; in_temp le:s16/16>>0 index 3; in_timestamp le:s64/64>>0 index 4
//  7. grep . $G/sampling_frequency_available $G/in_anglvel_scale $A/in_accel_scale $G/current_timestamp_clock
//       -> 12.500000 ... 4000.000000; 0.001065264; 0.004788403; realtime (monotonic once this code ran)
//  8. with the program running: grep . $G/buffer/* $A/buffer/*; grep icm /proc/interrupts; sleep 1;
//       grep icm /proc/interrupts       -> enable 1, length 1024, watermark 1; count grows by rate_hz
//  9. B=/sys/bus/iio/devices/iio:device0; cat $B/in_pressure_sampling_frequency
//       $B/in_pressure_oversampling_ratio; time cat $B/in_pressure_input   -> kPa near 100; time <= 1/rate
// 10. with the program running: stty -F /dev/ttymxc2   -> speed 115200, cs8, -parenb, -cstopb, -icanon
// =====================================================================================================

#pragma once

#include <cerrno>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <stdexcept>
#include <string>
#include <vector>

#include <dirent.h>
#include <fcntl.h>
#include <linux/videodev2.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <termios.h>
#include <unistd.h>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

namespace io {

struct ImuSample { int64_t t_ns; double gyro[3]; double acc[3]; };  // rad/s, m/s^2, CLOCK_MONOTONIC

namespace detail {

inline const std::string kIioRoot = "/sys/bus/iio/devices";

inline std::runtime_error sysError(const char* action, const std::string& path, int err) {
    return std::runtime_error(std::string(action) + " " + path + ": " + std::strerror(err));
}

// Reads a small text file (a sysfs attribute) and removes the trailing newline.
inline std::string readTextFile(const std::string& path) {
    const int fd = ::open(path.c_str(), O_RDONLY | O_CLOEXEC);
    if (fd < 0) throw sysError("open", path, errno);
    char text[256];
    const ssize_t n = ::read(fd, text, sizeof(text));
    const int read_errno = errno;
    ::close(fd);
    if (n < 0) throw sysError("read", path, read_errno);
    std::string result(text, static_cast<size_t>(n));
    while (!result.empty() && result.back() == '\n') result.pop_back();
    return result;
}

// Writes one value to a sysfs attribute. The kernel takes the whole value in one write.
inline void writeTextFile(const std::string& path, const std::string& value) {
    const int fd = ::open(path.c_str(), O_WRONLY | O_CLOEXEC);
    if (fd < 0) throw sysError("open", path, errno);
    const ssize_t n = ::write(fd, value.data(), value.size());
    const int write_errno = errno;
    ::close(fd);
    if (n < 0) throw sysError(("write \"" + value + "\" to").c_str(), path, write_errno);
    if (static_cast<size_t>(n) != value.size()) throw std::runtime_error("short write to " + path);
}

inline void expectTextFile(const std::string& path, const std::string& expected) {
    const std::string actual = readTextFile(path);
    if (actual != expected)
        throw std::runtime_error(path + " is \"" + actual + "\", this code handles only \"" + expected + "\"");
}

inline double parseNumber(const std::string& text, const std::string& path) {
    errno = 0;
    char* end = nullptr;
    const double value = std::strtod(text.c_str(), &end);
    if (text.empty() || end != text.c_str() + text.size() || errno != 0)
        throw std::runtime_error(path + " is not a number: \"" + text + "\"");
    return value;
}

// Returns the directory name ("iio:deviceN") of the IIO device whose `name` file holds `wanted`.
inline std::string findIioDevice(const std::string& wanted) {
    DIR* dir = ::opendir(kIioRoot.c_str());
    if (dir == nullptr) throw sysError("opendir", kIioRoot, errno);
    std::vector<std::string> ids;
    while (const dirent* entry = ::readdir(dir)) {
        const std::string id = entry->d_name;
        if (id.rfind("iio:device", 0) == 0) ids.push_back(id);
    }
    ::closedir(dir);
    for (const std::string& id : ids)
        if (readTextFile(kIioRoot + "/" + id + "/name") == wanted) return id;
    throw std::runtime_error("no IIO device named \"" + wanted + "\" in " + kIioRoot);
}

inline void ioctlOrThrow(int fd, unsigned long request, void* arg, const char* name, const std::string& path) {
    if (::ioctl(fd, request, arg) < 0) throw sysError(name, path, errno);
}

}  // namespace detail

// Opens the camera and starts streaming (multiplanar, mmap buffers).
struct Camera {
    // Requests NV12 width x height; throws if the driver returns anything else.
    // Always ask for the full sensor size: MEASURED on the board, a smaller size is a crop of the
    // top-left corner, not a smaller copy of the whole picture.
    Camera(const std::string& device, int width, int height) : device_(device), width_(width), height_(height) {
        fd_ = ::open(device.c_str(), O_RDWR | O_NONBLOCK | O_CLOEXEC);
        if (fd_ < 0) throw detail::sysError("open", device, errno);
        try {
            start();
        } catch (...) {
            stop();
            throw;
        }
    }
    Camera(const Camera&) = delete;
    Camera& operator=(const Camera&) = delete;

    // Waits up to timeout_ms for a frame. Returns false on timeout (or if a signal interrupts the wait).
    // On success fills `gray` with the luma plane shrunk by 2 in x and y (CV_8UC1, width/2 x height/2)
    // and t_ns with the buffer timestamp (CLOCK_MONOTONIC, end of frame, 1 us steps).
    // Frames come oldest first; up to 2 can be waiting if the caller was slow.
    bool read(cv::Mat& gray, int64_t& t_ns, int timeout_ms) {
        pollfd waiter;
        waiter.fd = fd_;
        waiter.events = POLLIN;
        waiter.revents = 0;
        const int ready = ::poll(&waiter, 1, timeout_ms);
        if (ready < 0 && errno == EINTR) return false;
        if (ready < 0) throw detail::sysError("poll", device_, errno);
        if (ready == 0) return false;
        if ((waiter.revents & POLLIN) == 0)   // POLLERR: the driver is not streaming
            throw std::runtime_error("poll " + device_ + ": revents " + std::to_string(waiter.revents));

        v4l2_plane plane;
        v4l2_buffer buffer;
        std::memset(&plane, 0, sizeof(plane));
        std::memset(&buffer, 0, sizeof(buffer));
        buffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
        buffer.memory = V4L2_MEMORY_MMAP;
        buffer.m.planes = &plane;
        buffer.length = 1;   // number of planes
        if (::ioctl(fd_, VIDIOC_DQBUF, &buffer) < 0) {
            if (errno == EAGAIN) return false;
            throw detail::sysError("VIDIOC_DQBUF", device_, errno);
        }
        if (buffer.index >= mappings_.size())
            throw std::runtime_error(device_ + ": VIDIOC_DQBUF returned buffer index " + std::to_string(buffer.index));
        if ((buffer.flags & V4L2_BUF_FLAG_ERROR) != 0)
            throw std::runtime_error(device_ + ": the driver marked the frame as bad (V4L2_BUF_FLAG_ERROR)");
        if ((buffer.flags & V4L2_BUF_FLAG_TIMESTAMP_MASK) != V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC ||
            (buffer.flags & V4L2_BUF_FLAG_TSTAMP_SRC_MASK) != V4L2_BUF_FLAG_TSTAMP_SRC_EOF)
            throw std::runtime_error(device_ + ": timestamp is not monotonic end-of-frame, buffer flags " +
                                     std::to_string(buffer.flags));
        if (plane.bytesused < static_cast<size_t>(stride_) * height_ * 3 / 2)
            throw std::runtime_error(device_ + ": short frame, " + std::to_string(plane.bytesused) + " bytes");

        // The buffer is device memory that the CPU does not cache: reading it byte by byte took 55 ms a
        // frame on the board (MEASURED, 14 frames per second). So copy the luma plane out in one block
        // first, then shrink the copy. NV12 keeps all the luma at the start of the buffer, one byte a pixel.
        full_.create(height_, width_, CV_8UC1);
        const uint8_t* frame = static_cast<const uint8_t*>(mappings_[buffer.index].start);
        for (int row = 0; row < height_; ++row)
            std::memcpy(full_.ptr<uint8_t>(row), frame + static_cast<size_t>(row) * stride_, static_cast<size_t>(width_));
        // Each output pixel is the mean of a 2 x 2 block, so fine texture does not turn into false patterns.
        cv::resize(full_, gray, cv::Size(width_ / 2, height_ / 2), 0, 0, cv::INTER_AREA);
        t_ns = static_cast<int64_t>(buffer.timestamp.tv_sec) * 1000000000LL +
               static_cast<int64_t>(buffer.timestamp.tv_usec) * 1000LL;

        queueBuffer(buffer.index);
        return true;
    }

    ~Camera() { stop(); }

private:
    struct Mapping { void* start; size_t length; };
    static constexpr unsigned kBufferCount = 4;   // the driver always holds 2; 2 more can wait for the caller

    void start() {
        v4l2_capability capability;
        std::memset(&capability, 0, sizeof(capability));
        detail::ioctlOrThrow(fd_, VIDIOC_QUERYCAP, &capability, "VIDIOC_QUERYCAP", device_);
        const std::string driver(reinterpret_cast<const char*>(capability.driver));
        if (driver != "mxc-isi-cap_v1")
            throw std::runtime_error(device_ + ": driver is \"" + driver +
                                     "\", this code was checked against \"mxc-isi-cap_v1\" only");

        v4l2_format format;
        std::memset(&format, 0, sizeof(format));   // bytesperline 0: the driver fills in the width
        format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
        format.fmt.pix_mp.width = static_cast<uint32_t>(width_);
        format.fmt.pix_mp.height = static_cast<uint32_t>(height_);
        format.fmt.pix_mp.pixelformat = V4L2_PIX_FMT_NV12;
        format.fmt.pix_mp.field = V4L2_FIELD_NONE;
        format.fmt.pix_mp.num_planes = 1;
        detail::ioctlOrThrow(fd_, VIDIOC_S_FMT, &format, "VIDIOC_S_FMT", device_);
        const v4l2_pix_format_mplane& got = format.fmt.pix_mp;
        stride_ = got.plane_fmt[0].bytesperline;
        if (got.width != static_cast<uint32_t>(width_) || got.height != static_cast<uint32_t>(height_) ||
            got.pixelformat != V4L2_PIX_FMT_NV12 || got.num_planes != 1 ||
            stride_ < static_cast<uint32_t>(width_))
            throw std::runtime_error(device_ + ": asked for NV12 " + std::to_string(width_) + "x" +
                                     std::to_string(height_) + " in 1 plane, driver returned fourcc " +
                                     std::to_string(got.pixelformat) + " " + std::to_string(got.width) + "x" +
                                     std::to_string(got.height) + " in " + std::to_string(got.num_planes) +
                                     " plane(s), bytesperline " + std::to_string(stride_));

        v4l2_requestbuffers request;
        std::memset(&request, 0, sizeof(request));
        request.count = kBufferCount;
        request.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
        request.memory = V4L2_MEMORY_MMAP;
        detail::ioctlOrThrow(fd_, VIDIOC_REQBUFS, &request, "VIDIOC_REQBUFS", device_);
        if (request.count != kBufferCount)
            throw std::runtime_error(device_ + ": asked for " + std::to_string(kBufferCount) +
                                     " buffers, driver gave " + std::to_string(request.count));

        for (unsigned index = 0; index < kBufferCount; ++index) {
            v4l2_plane plane;
            v4l2_buffer buffer;
            std::memset(&plane, 0, sizeof(plane));
            std::memset(&buffer, 0, sizeof(buffer));
            buffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
            buffer.memory = V4L2_MEMORY_MMAP;
            buffer.index = index;
            buffer.m.planes = &plane;
            buffer.length = 1;   // number of planes
            detail::ioctlOrThrow(fd_, VIDIOC_QUERYBUF, &buffer, "VIDIOC_QUERYBUF", device_);
            if (plane.length < static_cast<size_t>(stride_) * height_ * 3 / 2)
                throw std::runtime_error(device_ + ": buffer of " + std::to_string(plane.length) +
                                         " bytes is smaller than one frame");
            void* start = ::mmap(nullptr, plane.length, PROT_READ, MAP_SHARED, fd_, plane.m.mem_offset);
            if (start == MAP_FAILED) throw detail::sysError("mmap", device_, errno);
            mappings_.push_back(Mapping{start, plane.length});
        }
        // The driver needs at least 2 queued buffers before STREAMON does anything; queue all of them.
        for (unsigned index = 0; index < kBufferCount; ++index) queueBuffer(index);

        int type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
        detail::ioctlOrThrow(fd_, VIDIOC_STREAMON, &type, "VIDIOC_STREAMON", device_);
    }

    void queueBuffer(unsigned index) {
        v4l2_plane plane;
        v4l2_buffer buffer;
        std::memset(&plane, 0, sizeof(plane));
        std::memset(&buffer, 0, sizeof(buffer));
        buffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
        buffer.memory = V4L2_MEMORY_MMAP;
        buffer.index = index;
        buffer.m.planes = &plane;
        buffer.length = 1;   // number of planes
        detail::ioctlOrThrow(fd_, VIDIOC_QBUF, &buffer, "VIDIOC_QBUF", device_);
    }

    // Used by the destructor and by a failed constructor, so it cannot throw: results are not checked.
    void stop() noexcept {
        int type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
        ::ioctl(fd_, VIDIOC_STREAMOFF, &type);
        for (const Mapping& mapping : mappings_) ::munmap(mapping.start, mapping.length);
        mappings_.clear();
        ::close(fd_);
    }

    std::string device_;
    int width_;
    int height_;
    int fd_ = -1;
    uint32_t stride_ = 0;   // bytes per row, as reported by the driver
    cv::Mat full_;          // full-size luma, copied out of the driver buffer
    std::vector<Mapping> mappings_;
};

// Configures both IIO buffers (x, y, z + timestamp), sets the sampling rate and the monotonic timestamp clock.
struct Imu {
    // rate_hz: 25, 50, 100, 200, 1000, 2000 or 4000. Anything else makes the kernel refuse, and this throws.
    explicit Imu(int rate_hz) : rate_hz_(rate_hz) {
        gyro_id_ = detail::findIioDevice("icm42605-gyro");
        accel_id_ = detail::findIioDevice("icm42605-accel");
        // Open the data devices first: if another process holds one, this fails with EBUSY before
        // anything has been changed.
        gyro_fd_ = openData(gyro_id_);
        try {
            accel_fd_ = openData(accel_id_);
            gyro_scale_ = configure(gyro_id_, "anglvel");
            accel_scale_ = configure(accel_id_, "accel");
            // Gyro first: the device enabled second has late timestamps for about 1 s (comment block, 4).
            detail::writeTextFile(sysfsDir(gyro_id_) + "/buffer/enable", "1");
            detail::writeTextFile(sysfsDir(accel_id_) + "/buffer/enable", "1");
        } catch (...) {
            stop();
            throw;
        }
    }
    Imu(const Imu&) = delete;
    Imu& operator=(const Imu&) = delete;

    // Non-blocking: appends every sample available now to `out`, oldest first. One output per gyro sample,
    // paired with the accel sample nearest in time (the two devices are fed from the same FIFO).
    // t_ns is the gyro timestamp. "Non-blocking" means it does not wait for data; it does talk to the chip.
    void read(std::vector<ImuSample>& out) {
        // Gyro before accel: the kernel fills both buffers together, so after these two calls every
        // gyro sample already has its accel partner.
        readAvailable(gyro_fd_, gyro_id_, gyro_);
        readAvailable(accel_fd_, accel_id_, accel_);

        const size_t one_second = static_cast<size_t>(rate_hz_);
        if (accel_.empty() && gyro_.size() > one_second)
            throw std::runtime_error("icm42605-accel gave no samples while icm42605-gyro gave 1 s of them");
        if (gyro_.empty() && accel_.size() > one_second)
            throw std::runtime_error("icm42605-gyro gave no samples while icm42605-accel gave 1 s of them");
        if (accel_.empty()) return;   // keep the gyro samples until the first accel sample arrives

        size_t nearest = 0;   // index into accel_; only moves forward because both lists are in time order
        for (const RawSample& gyro : gyro_) {
            while (nearest + 1 < accel_.size() &&
                   std::abs(accel_[nearest + 1].t_ns - gyro.t_ns) <= std::abs(accel_[nearest].t_ns - gyro.t_ns))
                ++nearest;
            ImuSample sample;
            sample.t_ns = gyro.t_ns;
            for (int axis = 0; axis < 3; ++axis) {
                sample.gyro[axis] = gyro.axis[axis] * gyro_scale_;
                sample.acc[axis] = accel_[nearest].axis[axis] * accel_scale_;
            }
            out.push_back(sample);
        }
        gyro_.clear();
        // Keep the accel sample used last (it may also be the nearest to the next gyro sample) and newer ones.
        accel_.erase(accel_.begin(), accel_.begin() + static_cast<std::ptrdiff_t>(nearest));
    }

    ~Imu() { stop(); }   // disables the buffers

private:
    struct RawSample { int64_t t_ns; int16_t axis[3]; };
    static constexpr size_t kRecordBytes = 16;     // x, y, z, 2 bytes padding, timestamp (comment block, 1)
    static constexpr int kBufferLength = 1024;     // kernel buffer, in samples; when full, samples are lost

    static std::string sysfsDir(const std::string& id) { return detail::kIioRoot + "/" + id; }

    static int openData(const std::string& id) {
        const std::string path = "/dev/" + id;
        const int fd = ::open(path.c_str(), O_RDONLY | O_NONBLOCK | O_CLOEXEC);
        if (fd < 0) throw detail::sysError("open", path, errno);
        return fd;
    }

    // Sets up one of the two devices with its buffer still disabled and returns its scale.
    // `channel` is "anglvel" or "accel". The order of the writes matters (comment block, 5).
    double configure(const std::string& id, const std::string& channel) const {
        const std::string dir = sysfsDir(id);
        detail::writeTextFile(dir + "/buffer/enable", "0");

        detail::writeTextFile(dir + "/current_timestamp_clock", "monotonic");
        detail::expectTextFile(dir + "/current_timestamp_clock", "monotonic");

        const std::string rate_path = dir + "/sampling_frequency";
        detail::writeTextFile(rate_path, std::to_string(rate_hz_));
        const std::string rate_text = detail::readTextFile(rate_path);
        if (detail::parseNumber(rate_text, rate_path) != rate_hz_)
            throw std::runtime_error(rate_path + " is " + rate_text + " after writing " + std::to_string(rate_hz_));

        const char* const axis_names[3] = {"x", "y", "z"};
        for (int axis = 0; axis < 3; ++axis) {
            const std::string element = dir + "/scan_elements/in_" + channel + "_" + axis_names[axis];
            detail::expectTextFile(element + "_type", "be:s16/16>>0");
            detail::expectTextFile(element + "_index", std::to_string(axis));
            detail::writeTextFile(element + "_en", "1");
        }
        detail::writeTextFile(dir + "/scan_elements/in_temp_en", "0");
        detail::expectTextFile(dir + "/scan_elements/in_timestamp_type", "le:s64/64>>0");
        detail::writeTextFile(dir + "/scan_elements/in_timestamp_en", "1");

        detail::writeTextFile(dir + "/buffer/length", std::to_string(kBufferLength));
        detail::writeTextFile(dir + "/buffer/watermark", "1");   // one interrupt, and one timestamp, per sample

        const std::string scale_path = dir + "/in_" + channel + "_scale";
        return detail::parseNumber(detail::readTextFile(scale_path), scale_path);
    }

    static RawSample decode(const uint8_t* record) {
        RawSample sample;
        for (int axis = 0; axis < 3; ++axis) {   // big-endian signed 16 bit
            const uint16_t bits = static_cast<uint16_t>((record[2 * axis] << 8) | record[2 * axis + 1]);
            sample.axis[axis] = static_cast<int16_t>(bits);
        }
        uint64_t bits = 0;
        for (int byte = 7; byte >= 0; --byte) bits = (bits << 8) | record[8 + byte];   // little-endian signed 64 bit
        sample.t_ns = static_cast<int64_t>(bits);
        return sample;
    }

    // Appends every record waiting on `fd` to `samples`.
    static void readAvailable(int fd, const std::string& id, std::vector<RawSample>& samples) {
        const std::string path = "/dev/" + id;
        uint8_t bytes[kRecordBytes * 256];
        for (;;) {
            const ssize_t n = ::read(fd, bytes, sizeof(bytes));
            if (n < 0 && errno == EAGAIN) return;   // nothing waiting
            if (n < 0) throw detail::sysError("read", path, errno);
            if (n == 0 || static_cast<size_t>(n) % kRecordBytes != 0)
                throw std::runtime_error("read " + path + ": got " + std::to_string(n) + " bytes, records are " +
                                         std::to_string(kRecordBytes) + " bytes");
            for (size_t offset = 0; offset < static_cast<size_t>(n); offset += kRecordBytes)
                samples.push_back(decode(bytes + offset));
            if (static_cast<size_t>(n) < sizeof(bytes)) return;   // the kernel had no more
        }
    }

    // Used by the destructor and by a failed constructor, so it cannot throw. A failure is printed.
    void stop() noexcept {
        disableAndClose(gyro_fd_, gyro_id_);
        disableAndClose(accel_fd_, accel_id_);
    }

    static void disableAndClose(int& fd, const std::string& id) noexcept {
        if (fd < 0) return;   // never opened: not ours to disable
        try {
            detail::writeTextFile(sysfsDir(id) + "/buffer/enable", "0");
        } catch (const std::exception& error) {
            std::fprintf(stderr, "io::Imu: %s\n", error.what());
        }
        ::close(fd);
        fd = -1;
    }

    int rate_hz_;
    std::string gyro_id_;    // "iio:deviceN"
    std::string accel_id_;
    int gyro_fd_ = -1;
    int accel_fd_ = -1;
    double gyro_scale_ = 0.0;    // rad/s per LSB
    double accel_scale_ = 0.0;   // m/s^2 per LSB
    std::vector<RawSample> gyro_;    // read from the kernel, not yet paired
    std::vector<RawSample> accel_;   // read from the kernel, may still be the nearest to a future gyro sample
};

struct Serial {
    // Raw 8N1, non-blocking; supports 115200 and 230400 only.
    Serial(const std::string& device, int baud) : device_(device) {
        speed_t speed;
        if (baud == 115200) speed = B115200;
        else if (baud == 230400) speed = B230400;
        else throw std::runtime_error(device + ": baud " + std::to_string(baud) + " is not supported (115200, 230400)");

        fd_ = ::open(device.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
        if (fd_ < 0) throw detail::sysError("open", device, errno);
        try {
            configure(speed);
        } catch (...) {
            ::close(fd_);
            throw;
        }
    }
    Serial(const Serial&) = delete;
    Serial& operator=(const Serial&) = delete;

    // Returns 0 if nothing is waiting.
    size_t read(uint8_t* buf, size_t max) {
        const ssize_t n = ::read(fd_, buf, max);
        if (n < 0 && errno == EAGAIN) return 0;
        if (n < 0) throw detail::sysError("read", device_, errno);
        return static_cast<size_t>(n);
    }

    // Writes all bytes or throws. It does not wait: if the kernel's output buffer is full this throws
    // (EAGAIN, "Resource temporarily unavailable"), possibly after part of the bytes were accepted.
    void write(const uint8_t* buf, size_t n) {
        size_t done = 0;
        while (done < n) {
            const ssize_t written = ::write(fd_, buf + done, n - done);
            if (written < 0) throw detail::sysError("write", device_, errno);
            if (written == 0) throw std::runtime_error("write " + device_ + ": the kernel accepted 0 bytes");
            done += static_cast<size_t>(written);
        }
    }

    ~Serial() { ::close(fd_); }

private:
    void configure(speed_t speed) {
        termios wanted;
        std::memset(&wanted, 0, sizeof(wanted));   // all input, output and line processing off = raw
        wanted.c_cflag = CS8 | CREAD | CLOCAL;     // 8 data bits, no parity, 1 stop bit, no flow control
        wanted.c_cc[VMIN] = 0;                     // read() returns at once, also with 0 bytes
        wanted.c_cc[VTIME] = 0;
        if (::cfsetispeed(&wanted, speed) < 0) throw detail::sysError("cfsetispeed", device_, errno);
        if (::cfsetospeed(&wanted, speed) < 0) throw detail::sysError("cfsetospeed", device_, errno);
        if (::tcsetattr(fd_, TCSANOW, &wanted) < 0) throw detail::sysError("tcsetattr", device_, errno);

        // tcsetattr succeeds when ANY setting was applied, so read back the ones that matter.
        termios actual;
        if (::tcgetattr(fd_, &actual) < 0) throw detail::sysError("tcgetattr", device_, errno);
        const bool speed_ok = ::cfgetispeed(&actual) == speed && ::cfgetospeed(&actual) == speed;
        const bool frame_ok = (actual.c_cflag & (CSIZE | PARENB | CSTOPB | CRTSCTS)) == CS8;
        const bool raw_ok = (actual.c_lflag & (ICANON | ECHO | ISIG | IEXTEN)) == 0 &&
                            (actual.c_iflag & (IXON | IXOFF | ICRNL | INLCR | IGNCR | ISTRIP | INPCK | PARMRK)) == 0 &&
                            (actual.c_oflag & OPOST) == 0;
        if (!speed_ok || !frame_ok || !raw_ok)
            throw std::runtime_error(device_ + ": the driver did not accept raw 8N1 at the requested baud rate");

        // Drop bytes that arrived before this point: they are not answers to anything this program asked.
        if (::tcflush(fd_, TCIOFLUSH) < 0) throw detail::sysError("tcflush", device_, errno);
    }

    std::string device_;
    int fd_ = -1;
};

// DPS310 through IIO sysfs (no buffer in the mainline 6.6 driver). Finds the device by name "dps310".
//
// MEASURED on the board (21 Nov), then explained by drivers/iio/pressure/dps310.c (v6.6):
//  - A read waits for the chip's "new value" flag, but at most 1 / rate seconds (dps310_read_pres_raw).
//    If the flag does not come in time the driver resets the chip to its defaults: 1 value per second,
//    oversampling 1 (dps310_ready -> dps310_reset_reinit). Seen on the first read after the rate was
//    changed from 1 to 32: the rate was back at 1 afterwards and every read took 1 s.
//  - A value measured before a change of oversampling is converted with the new setting and comes out
//    wrong: the first value read was 120493 Pa in a room at 101460 Pa.
// So this class sets the chip, waits for the old measurement to finish, drops the first values, reads
// only every second chip value (the flag is then already set, so the driver never waits and never
// resets), and sets the chip again if it finds the driver has reset it anyway.
struct Baro {
    // chip_rate_hz and oversampling must be powers of two that the chip can do together.
    // MEASURED on the board: 32 per second at 8x gives values within 1 Pa of each other (1 Pa is 8 cm),
    // no driver reset in 20 reads, about 10 ms a read. At 1x the spread is 3 Pa.
    Baro(int chip_rate_hz, int oversampling)
        : dir_(detail::kIioRoot + "/" + detail::findIioDevice("dps310")),
          chip_rate_hz_(chip_rate_hz),
          oversampling_(oversampling) {
        // MEASURED on the board: with oversampling 16 every read fails with "Numerical result out of range".
        // The 6.6 driver sets the wrong bit for the "shift" that the chip needs from 16 upwards
        // (DPS310_PRS_SHIFT_EN is BIT(4) in dps310.c; the data sheet has P_SHIFT at bit 2). 1 to 8 work.
        if (oversampling > 8)
            throw std::runtime_error("barometer oversampling " + std::to_string(oversampling) +
                                     " does not work with this kernel driver, use 8 or less");
        const std::string path = dir_ + "/in_pressure_input";
        fd_ = ::open(path.c_str(), O_RDONLY | O_CLOEXEC);
        if (fd_ < 0) throw detail::sysError("open", path, errno);
        configure();
    }
    Baro(const Baro&) = delete;
    Baro& operator=(const Baro&) = delete;

    // Waits two chip periods, then reads the pressure in pascal (steps of 1 Pa).
    // Returns false when the value must not be used: the chip was just set up, the driver has reset
    // it, or the number is not a possible air pressure.
    bool read(double& pa) {
        ::usleep(static_cast<useconds_t>(2000000 / chip_rate_hz_));
        char text[64];
        // Reading from offset 0 makes sysfs ask the driver again (fs/seq_file.c, seq_read_iter).
        const ssize_t n = ::pread(fd_, text, sizeof(text) - 1, 0);
        if (n < 0) throw detail::sysError("read", dir_ + "/in_pressure_input", errno);
        std::string kilopascal(text, static_cast<size_t>(n));
        while (!kilopascal.empty() && kilopascal.back() == '\n') kilopascal.pop_back();
        const double value = detail::parseNumber(kilopascal, dir_ + "/in_pressure_input") * 1000.0;

        if (setting("in_pressure_sampling_frequency") != chip_rate_hz_ ||
            setting("in_pressure_oversampling_ratio") != oversampling_) {
            resets++;
            configure();
            return false;
        }
        if (to_drop_ > 0) {
            to_drop_--;
            return false;
        }
        if (value < kLowestPa || value > kHighestPa) {
            rejected++;
            return false;
        }
        pa = value;
        return true;
    }

    int resets = 0;     // times the driver was found back at its defaults
    int rejected = 0;   // values outside the possible range

    ~Baro() { ::close(fd_); }

private:
    // 30 kPa is 9 km up; the highest pressure ever recorded at sea level is below 109 kPa.
    static constexpr double kLowestPa = 30000, kHighestPa = 110000;

    void configure() {
        detail::writeTextFile(dir_ + "/in_pressure_oversampling_ratio", std::to_string(oversampling_));
        detail::writeTextFile(dir_ + "/in_pressure_sampling_frequency", std::to_string(chip_rate_hz_));
        ::usleep(1500000);   // the measurement that was running at the old rate can take 1 s to finish
        to_drop_ = 3;        // values that may still come from the old setting
    }

    int setting(const std::string& file) const {
        const std::string path = dir_ + "/" + file;
        return static_cast<int>(detail::parseNumber(detail::readTextFile(path), path));
    }

    std::string dir_;
    int chip_rate_hz_, oversampling_;
    int fd_ = -1;
    int to_drop_ = 0;
};

}  // namespace io
