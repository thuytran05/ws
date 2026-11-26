#include <OpenNI.h>
#include <iostream>
#include <chrono>
#include <algorithm>
#include <string>
#include <iomanip>
#include <cstdlib>

using namespace openni;
using Clock = std::chrono::steady_clock;

struct Stats {
    int count = 0;
    int total = 0;
    int skipped = 0;
    int last_index = -1;
    double max_gap = 0.0;
    double overall_max_gap = 0.0;
    Clock::time_point last_frame;
    bool received = false;

    void update(const VideoFrameRef& frame) {
        auto now = Clock::now();

        if (received) {
            double gap =
                std::chrono::duration<double, std::milli>(
                    now - last_frame).count();

            max_gap = std::max(max_gap, gap);
            overall_max_gap =
                std::max(overall_max_gap, gap);

            if (frame.getFrameIndex() > last_index + 1) {
                skipped +=
                    frame.getFrameIndex() - last_index - 1;
            }
        }

        last_index = frame.getFrameIndex();
        last_frame = now;
        received = true;

        count++;
        total++;
    }

    void report(const char* name, double elapsed) {
        std::cout
            << name
            << " FPS: " << count / elapsed
            << " | Max gap: " << max_gap << " ms"
            << " | Skipped index: " << skipped
            << std::endl;

        count = 0;
        max_gap = 0;
    }
};

bool startStream(
    Device& device,
    VideoStream& stream,
    SensorType sensor,
    PixelFormat format)
{
    if (!device.hasSensor(sensor)) {
        std::cerr << "Sensor unavailable\n";
        return false;
    }

    if (stream.create(device, sensor) != STATUS_OK) {
        std::cerr << OpenNI::getExtendedError() << "\n";
        return false;
    }

    VideoMode mode;
    mode.setResolution(640, 480);
    mode.setFps(30);
    mode.setPixelFormat(format);

    if (stream.setVideoMode(mode) != STATUS_OK) {
        std::cerr << "Cannot set video mode: "
                  << OpenNI::getExtendedError() << "\n";
        return false;
    }

    if (stream.start() != STATUS_OK) {
        std::cerr << "Cannot start stream: "
                  << OpenNI::getExtendedError() << "\n";
        return false;
    }

    auto actual = stream.getVideoMode();

    std::cout
        << "Started: "
        << actual.getResolutionX() << "x"
        << actual.getResolutionY()
        << " @ " << actual.getFps()
        << " FPS | Format: "
        << actual.getPixelFormat()
        << std::endl;

    return true;
}

int main(int argc, char** argv) {
    std::string mode = argc > 1 ? argv[1] : "both";
    int duration = argc > 2 ? std::atoi(argv[2]) : 60;

    if (mode != "rgb" && mode != "depth" && mode != "both")
        return 1;

    if (duration <= 0)
        return 1;

    std::cout << std::fixed << std::setprecision(2);

    if (OpenNI::initialize() != STATUS_OK) {
        std::cerr << OpenNI::getExtendedError() << "\n";
        return 1;
    }

    Device device;

    if (device.open(ANY_DEVICE) != STATUS_OK) {
        std::cerr << OpenNI::getExtendedError() << "\n";
        OpenNI::shutdown();
        return 1;
    }

    auto info = device.getDeviceInfo();

    std::cout << "Device: "
              << info.getName() << "\n";

    device.setDepthColorSyncEnabled(false);

    if (device.isImageRegistrationModeSupported(
            IMAGE_REGISTRATION_OFF)) {
        device.setImageRegistrationMode(
            IMAGE_REGISTRATION_OFF);
    }

    VideoStream rgb;
    VideoStream depth;

    VideoStream* streams[2];
    Stats* stats[2];
    const char* names[2];

    Stats rgb_stats;
    Stats depth_stats;

    int n = 0;

    if (mode == "rgb" || mode == "both") {
        if (!startStream(
            device, rgb, SENSOR_COLOR, PIXEL_FORMAT_RGB888))
            return 1;

        streams[n] = &rgb;
        stats[n] = &rgb_stats;
        names[n++] = "RGB";
    }

    if (mode == "depth" || mode == "both") {
        if (!startStream(
            device, depth, SENSOR_DEPTH, PIXEL_FORMAT_DEPTH_1_MM))
            return 1;

        streams[n] = &depth;
        stats[n] = &depth_stats;
        names[n++] = "Depth";
    }

    std::cout << "Warmup 5 seconds\n";

    auto start = Clock::now();
    VideoFrameRef frame;

    while (std::chrono::duration<double>(
            Clock::now() - start).count() < 5.0) {
        int ready = -1;

        if (OpenNI::waitForAnyStream(
                streams, n, &ready, 200) == STATUS_OK) {
            streams[ready]->readFrame(&frame);
        }
    }

    std::cout << "Benchmark started\n";

    start = Clock::now();
    auto last_report = start;

    int errors = 0;

    while (std::chrono::duration<double>(
            Clock::now() - start).count() < duration) {
        int ready = -1;

        auto status = OpenNI::waitForAnyStream(
            streams, n, &ready, 200);

        if (status == STATUS_OK &&
            ready >= 0 && ready < n) {

            if (streams[ready]->readFrame(&frame)
                == STATUS_OK && frame.isValid()) {

                stats[ready]->update(frame);

            } else {
                errors++;
            }
        } else {
            errors++;
        }

        auto now = Clock::now();

        double elapsed = std::chrono::duration<double>(
            now - last_report).count();

        if (elapsed >= 5.0) {
            for (int i = 0; i < n; i++)
                stats[i]->report(names[i], elapsed);

            std::cout << "------------------\n";
            last_report = now;
        }
    }

    double elapsed = std::chrono::duration<double>(
        Clock::now() - start).count();

    std::cout << "\n===== FINAL RESULTS =====\n";

    for (int i = 0; i < n; i++) {
        std::cout
            << names[i]
            << " | Average FPS: "
            << stats[i]->total / elapsed
            << " | Frames: " << stats[i]->total
            << " | Max gap: "
            << stats[i]->overall_max_gap << " ms"
            << " | Skipped index: "
            << stats[i]->skipped
            << "\n";
    }

    std::cout << "Wait/read errors: " << errors << "\n";

    frame.release();

    for (int i = 0; i < n; i++) {
        streams[i]->stop();
        streams[i]->destroy();
    }

    device.close();
    OpenNI::shutdown();

    return 0;
}
