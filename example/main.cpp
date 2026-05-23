// =============================================================================
//  main.cpp  –  OLED SSD1306 UI for MyBipedal robot (Radxa 5B+)
//
//  Screens (accessed via encoder menu):
//    0 – Joint State   (pos, temp per joint)
//    1 – IMU           (gx/gy/gz + tilt dot)
//    2 – Joystick      (stick dot + buttons)
//    3 – AimRT Log     (scrolling text)
//    4 – SBC Status    (CPU cores, temp, RAM)
//
//  GPIO (Linux chardev v2):
//    /dev/gpiochip3 offset 13  –  Encoder CLK (A)
//    /dev/gpiochip3 offset 15  –  Encoder DT  (B)
//    /dev/gpiochip3 offset 16  –  Encoder SW  (push)
//    /dev/gpiochip4 offset 20  –  Back button
// =============================================================================

#include "oled_driver.hpp"
#include "i2c.h"

// ── std ──────────────────────────────────────────────────────────────────────
#include <algorithm>
#include <atomic>
#include <cassert>
#include <chrono>
#include <condition_variable>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <deque>
#include <format>
#include <fstream>
#include <mutex>
#include <optional>
#include <queue>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

// ── POSIX / Linux ─────────────────────────────────────────────────────────────
#include <arpa/inet.h>
#include <fcntl.h>
#include <ifaddrs.h>
#include <linux/gpio.h>
#include <netinet/in.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <unistd.h>

using namespace std::chrono_literals;

// ============================================================================
//  Hardware constants
// ============================================================================
static constexpr const char* kI2CDev    = "/dev/i2c-7";
static constexpr uint8_t     kOledAddr  = 0x3C;

struct PinDef { const char* chip; uint32_t offset; };
static constexpr PinDef kPinEncA   { "/dev/gpiochip3", 13 }; // CLK
static constexpr PinDef kPinEncB   { "/dev/gpiochip3", 15 }; // DT
static constexpr PinDef kPinEncBtn { "/dev/gpiochip3", 16 }; // SW
static constexpr PinDef kPinBack   { "/dev/gpiochip4", 20 }; // Back

static constexpr auto kSleepTimeout = 30s;

// ============================================================================
//  GPIO helper  (Linux GPIO chardev v2 – kernel ≥ 5.10)
// ============================================================================
class GpioLine {
public:
    GpioLine(const char* chip, uint32_t offset, const char* consumer = "oled_ui")
        : chip_(chip), offset_(offset), consumer_(consumer) {}

    ~GpioLine() { close(); }

    GpioLine(const GpioLine&)            = delete;
    GpioLine& operator=(const GpioLine&) = delete;

    /// Open for edge-detection (both edges, pull-up).
    bool openEdge() { return openImpl(true); }

    /// Open as plain input (value reading only, no edge fd).
    bool openValue() { return openImpl(false); }

    void close() {
        if (line_fd_ >= 0) { ::close(line_fd_); line_fd_ = -1; }
        if (chip_fd_ >= 0) { ::close(chip_fd_); chip_fd_ = -1; }
    }

    [[nodiscard]] bool valid()   const { return line_fd_ >= 0; }
    [[nodiscard]] int  fd()      const { return line_fd_; }

    /// Read logical level (requires openValue or openEdge).
    [[nodiscard]] bool getValue() const {
        if (line_fd_ < 0) return false;
        gpio_v2_line_values vals{};
        vals.mask = 1;
        if (::ioctl(line_fd_, GPIO_V2_LINE_GET_VALUES_IOCTL, &vals) < 0) return false;
        return (vals.bits & 1u) != 0u;
    }

    /// Consume one pending edge event; returns edge id or 0 on error.
    [[nodiscard]] uint32_t readEvent() const {
        gpio_v2_line_event ev{};
        if (::read(line_fd_, &ev, sizeof(ev)) != static_cast<ssize_t>(sizeof(ev)))
            return 0;
        return ev.id;
    }

private:
    bool openImpl(bool edges) {
        chip_fd_ = ::open(chip_.c_str(), O_RDONLY | O_CLOEXEC);
        if (chip_fd_ < 0) {
            std::perror(("GpioLine::open chip " + chip_).c_str());
            return false;
        }
        gpio_v2_line_request req{};
        req.num_lines    = 1;
        req.offsets[0]   = offset_;
        req.config.flags = GPIO_V2_LINE_FLAG_INPUT | GPIO_V2_LINE_FLAG_BIAS_PULL_UP;
        if (edges)
            req.config.flags |= GPIO_V2_LINE_FLAG_EDGE_RISING
                             |  GPIO_V2_LINE_FLAG_EDGE_FALLING;
        std::strncpy(req.consumer, consumer_.c_str(), GPIO_MAX_NAME_SIZE - 1);

        if (::ioctl(chip_fd_, GPIO_V2_GET_LINE_IOCTL, &req) < 0) {
            std::perror(("GpioLine::open ioctl offset=" + std::to_string(offset_)).c_str());
            ::close(chip_fd_); chip_fd_ = -1;
            return false;
        }
        line_fd_ = req.fd;
        return true;
    }

    std::string chip_, consumer_;
    uint32_t    offset_;
    int         chip_fd_{-1};
    int         line_fd_{-1};
};

// ============================================================================
//  Event system
// ============================================================================
enum class Event { ENC_CW, ENC_CCW, ENC_PUSH, BTN_BACK };

class EventQueue {
public:
    void push(Event e) {
        {
            std::lock_guard lock(mtx_);
            q_.push(e);
        }
        cv_.notify_one();
    }

    /// Block until an event arrives or timeout elapses.
    std::optional<Event> pop(std::chrono::milliseconds timeout) {
        std::unique_lock lock(mtx_);
        if (!cv_.wait_for(lock, timeout, [&]{ return !q_.empty(); }))
            return std::nullopt;
        Event e = q_.front();
        q_.pop();
        return e;
    }

    void clear() {
        std::lock_guard lock(mtx_);
        std::queue<Event> empty;
        std::swap(q_, empty);
    }

private:
    std::queue<Event>       q_;
    std::mutex              mtx_;
    std::condition_variable cv_;
};

// ============================================================================
//  Shared robot / sensor data  (written by external threads; read by UI)
// ============================================================================
struct JointInfo  { std::string name; float pos{0.f}; float temp{0.f}; };

// ---------------------------------------------------------------------------
//  Snapshot: plain-old data copy of SharedData (no mutex → copyable)
// ---------------------------------------------------------------------------
struct DataSnapshot {
    std::string              robot_ip;
    std::string              robot_name;
    std::vector<JointInfo>   joints;
    float imu_gx{0}, imu_gy{0}, imu_gz{0};
    float imu_ax{0}, imu_ay{0}, imu_az{0};
    float joy_lx{0}, joy_ly{0}, joy_rx{0}, joy_ry{0};
    uint16_t                 joy_buttons{0};
    std::deque<std::string>  log_lines;
    std::vector<float>       cpu_core_pct;
    float                    cpu_temp_c{0};
    uint64_t                 ram_total_kb{0};
    uint64_t                 ram_avail_kb{0};
};

// ---------------------------------------------------------------------------
//  SharedData: mutex-protected live data written by background threads
// ---------------------------------------------------------------------------
struct SharedData {
    mutable std::mutex mtx;

    std::string              robot_ip   = "0.0.0.0";
    std::string              robot_name = "MyBipedal By LongVu";
    std::vector<JointInfo>   joints;
    float imu_gx{0}, imu_gy{0}, imu_gz{0};
    float imu_ax{0}, imu_ay{0}, imu_az{0};
    float joy_lx{0}, joy_ly{0}, joy_rx{0}, joy_ry{0};
    uint16_t                 joy_buttons{0};
    std::deque<std::string>  log_lines;
    std::vector<float>       cpu_core_pct;
    float                    cpu_temp_c{0};
    uint64_t                 ram_total_kb{0};
    uint64_t                 ram_avail_kb{0};

    void pushLog(std::string line) {
        std::lock_guard lock(mtx);
        if (log_lines.size() >= 64) log_lines.pop_front();
        log_lines.push_back(std::move(line));
    }

    /// Thread-safe snapshot for rendering
    [[nodiscard]] DataSnapshot snapshot() const {
        std::lock_guard lock(mtx);
        DataSnapshot s;
        s.robot_ip    = robot_ip;
        s.robot_name  = robot_name;
        s.joints      = joints;
        s.imu_gx = imu_gx; s.imu_gy = imu_gy; s.imu_gz = imu_gz;
        s.imu_ax = imu_ax; s.imu_ay = imu_ay; s.imu_az = imu_az;
        s.joy_lx = joy_lx; s.joy_ly = joy_ly;
        s.joy_rx = joy_rx; s.joy_ry = joy_ry;
        s.joy_buttons = joy_buttons;
        s.log_lines   = log_lines;
        s.cpu_core_pct = cpu_core_pct;
        s.cpu_temp_c   = cpu_temp_c;
        s.ram_total_kb = ram_total_kb;
        s.ram_avail_kb = ram_avail_kb;
        return s;
    }
};

// ============================================================================
//  System utilities
// ============================================================================
static std::string getLocalIP() {
    ifaddrs* ifa = nullptr;
    if (::getifaddrs(&ifa) < 0) return "?.?.?.?";
    std::string result = "0.0.0.0";
    for (auto* p = ifa; p; p = p->ifa_next) {
        if (!p->ifa_addr || p->ifa_addr->sa_family != AF_INET) continue;
        if (std::string(p->ifa_name) == "lo") continue;
        char buf[INET_ADDRSTRLEN];
        auto* sa = reinterpret_cast<sockaddr_in*>(p->ifa_addr);
        ::inet_ntop(AF_INET, &sa->sin_addr, buf, sizeof(buf));
        result = buf;
        break;
    }
    ::freeifaddrs(ifa);
    return result;
}

// Returns per-core CPU usage % (reads /proc/stat twice, 200 ms apart)
static std::vector<float> readCpuUsage() {
    auto readStats = []() -> std::vector<std::array<uint64_t,2>> {
        std::ifstream f("/proc/stat");
        std::vector<std::array<uint64_t,2>> v;
        std::string line;
        while (std::getline(f, line)) {
            if (line.rfind("cpu", 0) != 0 || line[3] == ' ') continue;
            std::istringstream ss(line.substr(5));
            uint64_t user, nice, sys, idle, iowait, irq, softirq;
            ss >> user >> nice >> sys >> idle >> iowait >> irq >> softirq;
            uint64_t total = user+nice+sys+idle+iowait+irq+softirq;
            v.push_back({total, idle+iowait});
        }
        return v;
    };

    auto s1 = readStats();
    std::this_thread::sleep_for(200ms);
    auto s2 = readStats();

    std::vector<float> pct;
    for (size_t i = 0; i < std::min(s1.size(), s2.size()); ++i) {
        uint64_t dt    = s2[i][0] - s1[i][0];
        uint64_t didle = s2[i][1] - s1[i][1];
        pct.push_back(dt == 0 ? 0.f : 100.f * (1.f - static_cast<float>(didle) / dt));
    }
    return pct;
}

static float readCpuTemp() {
    // Try a few thermal zones; Radxa typically exposes zone 0
    for (int z = 0; z < 4; ++z) {
        std::ifstream f(std::format("/sys/class/thermal/thermal_zone{}/temp", z));
        if (!f) continue;
        int millideg; f >> millideg;
        if (millideg > 0) return millideg / 1000.f;
    }
    return 0.f;
}

static void readRamInfo(uint64_t& total_kb, uint64_t& avail_kb) {
    std::ifstream f("/proc/meminfo");
    std::string key; uint64_t val;
    total_kb = avail_kb = 0;
    while (f >> key >> val) {
        if (key == "MemTotal:")     total_kb = val;
        if (key == "MemAvailable:") avail_kb = val;
    }
}

// ============================================================================
//  UI State Machine
// ============================================================================
enum class UIState { SLEEPING, MAIN, MENU, SCREEN };

static constexpr std::array<const char*, 5> kMenuItems = {
    "1 Joint State",
    "2 IMU State",
    "3 Joystick",
    "4 AimRT Log",
    "5 SBC Status",
};

struct UIContext {
    UIState state      = UIState::SLEEPING;
    int     menu_sel   = 0;   // cursor in MENU (0–4)
    int     screen_idx = 0;   // active screen (0–4)
    std::chrono::steady_clock::time_point last_active = std::chrono::steady_clock::now();

    void touch() { last_active = std::chrono::steady_clock::now(); }
    [[nodiscard]] bool isTimedOut() const {
        return std::chrono::steady_clock::now() - last_active >= kSleepTimeout;
    }
};

// ============================================================================
//  Screen renderers  (each takes display + shared-data snapshot)
// ============================================================================

// ── Helpers ──────────────────────────────────────────────────────────────────
static void drawTitle(OledDriver& d, std::string_view title) {
    d.fillRect(0, 0, 128, 10, true);
    int tx = (128 - d.textWidth(title)) / 2;
    d.drawString(tx, 1, title, false);
    d.drawHLine(0, 10, 128, true);
}

// ── Screen: MAIN ─────────────────────────────────────────────────────────────
static void renderMain(OledDriver& d, const DataSnapshot& sd) {
    // Robot name – large centred
    auto name = sd.robot_name;
    // Split at " By " for 2 lines
    const std::string line1 = "MyBipedal";
    const std::string line2 = "By LongVu";
    int x1 = (128 - d.textWidth(line1, 2)) / 2;
    int x2 = (128 - d.textWidth(line2))    / 2;
    d.drawString(x1, 4,  line1, true, 2);
    d.drawString(x2, 24, line2, true, 1);

    // Separator
    d.drawHLine(0, 35, 128, true);

    // IP
    d.drawString(0, 38, "IP:", true);
    d.drawString(18, 38, sd.robot_ip, true);

    // Sleep hint
    d.drawString(0, 54, "  Rotate enc -> Menu  ", true);
}

// ── Screen: MENU ─────────────────────────────────────────────────────────────
static void renderMenu(OledDriver& d, int sel) {
    drawTitle(d, "-- MENU --");
    // Show up to 5 items (fit 5 × 10 px in 54 px below title)
    for (int i = 0; i < 5; ++i) {
        int y = 12 + i * 10;
        if (i == sel) {
            d.fillRect(0, y, 128, 10, true);
            d.drawString(2, y + 1, kMenuItems[i], false);
        } else {
            d.drawString(2, y + 1, kMenuItems[i], true);
        }
    }
}

// ── Screen 0: Joint State ────────────────────────────────────────────────────
static void renderJointState(OledDriver& d, const DataSnapshot& sd) {
    drawTitle(d, "Joint State");
    // 6 joints, 2 per row (pos + temp)
    const auto& J = sd.joints;
    for (int i = 0; i < static_cast<int>(J.size()) && i < 6; ++i) {
        int col = i % 2, row = i / 2;
        int x = col * 64, y = 13 + row * 17;
        d.drawString(x, y,     J[i].name, true);
        d.drawString(x, y + 8, std::format("{:.1f}", J[i].pos),  true);
        d.drawString(x+36, y+8, std::format("{:.0f}C", J[i].temp), true);
    }
}

// ── Screen 1: IMU State ──────────────────────────────────────────────────────
static void renderIMU(OledDriver& d, const DataSnapshot& sd) {
    drawTitle(d, "IMU State");

    // Left half: numeric values
    d.drawString(0, 12, std::format("Gx{:+6.1f}", sd.imu_gx), true);
    d.drawString(0, 22, std::format("Gy{:+6.1f}", sd.imu_gy), true);
    d.drawString(0, 32, std::format("Gz{:+6.1f}", sd.imu_gz), true);
    d.drawString(0, 42, std::format("Ax{:+5.2f}", sd.imu_ax), true);
    d.drawString(0, 52, std::format("Ay{:+5.2f}", sd.imu_ay), true);

    // Right half: tilt dot (circle + dot)
    constexpr int CX = 100, CY = 38, R = 22;
    d.drawCircle(CX, CY, R, true);
    d.drawHLine(CX - R, CY, 2*R+1, true);   // crosshair
    d.drawVLine(CX, CY - R, 2*R+1, true);

    // Dot position from normalised gx/gy (clamp to ±90 deg range)
    float nx = std::clamp(sd.imu_gy / 90.f, -1.f, 1.f);
    float ny = std::clamp(sd.imu_gx / 90.f, -1.f, 1.f);
    int dx = CX + static_cast<int>(nx * (R - 3));
    int dy = CY + static_cast<int>(ny * (R - 3));
    d.fillCircle(dx, dy, 3, true);
}

// ── Screen 2: Joystick State ──────────────────────────────────────────────────
static void renderJoystick(OledDriver& d, const DataSnapshot& sd) {
    drawTitle(d, "Joystick");

    auto drawStick = [&](int cx, int cy, int r, float vx, float vy,
                         std::string_view label) {
        d.drawCircle(cx, cy, r, true);
        d.drawHLine(cx - r, cy, 2*r+1, true);
        d.drawVLine(cx, cy - r, 2*r+1, true);
        d.drawString(cx - d.textWidth(label)/2, cy + r + 2, label, true);
        int px = cx + static_cast<int>(vx * (r - 2));
        int py = cy + static_cast<int>(vy * (r - 2));
        d.fillCircle(px, py, 3, true);
        // Axis labels
        d.drawString(cx - r - 2, cy + r + 10,
                     std::format("{:.1f}", vx), true);
    };

    drawStick(28, 37, 20, sd.joy_lx, sd.joy_ly, "L");
    drawStick(100, 37, 20, sd.joy_rx, sd.joy_ry, "R");

    // Button states – bottom row
    constexpr std::array<const char*, 8> kBtnNames = {
        "A","B","X","Y","LB","RB","St","Bk"
    };
    for (int i = 0; i < 8; ++i) {
        bool pressed = (sd.joy_buttons >> i) & 1u;
        int  bx = i * 16, by = 56;
        if (pressed) {
            d.fillRect(bx, by, 14, 8, true);
            d.drawString(bx+1, by, kBtnNames[i], false);
        } else {
            d.drawString(bx+1, by, kBtnNames[i], true);
        }
    }
}

// ── Screen 3: AimRT Log ──────────────────────────────────────────────────────
static void renderLog(OledDriver& d, const DataSnapshot& sd) {
    drawTitle(d, "AimRT Log");
    constexpr int kMaxRows = 6;
    constexpr int kCharsPerRow = 21; // 128/6 = 21.3

    const auto& lines = sd.log_lines;
    int start = static_cast<int>(lines.size()) - kMaxRows;
    if (start < 0) start = 0;
    for (int i = 0; i < kMaxRows && (start + i) < static_cast<int>(lines.size()); ++i) {
        std::string row = lines[start + i];
        if (row.size() > kCharsPerRow) row.resize(kCharsPerRow);
        d.drawString(0, 12 + i * 9, row, true);
    }
}

// ── Screen 4: SBC Status ─────────────────────────────────────────────────────
static void renderSBC(OledDriver& d, const DataSnapshot& sd) {
    drawTitle(d, "SBC Status");

    // CPU cores in 2 columns (up to 8 cores)
    const int ncores = static_cast<int>(sd.cpu_core_pct.size());
    constexpr int BAR_W = 48, BAR_H = 5;
    for (int i = 0; i < ncores && i < 8; ++i) {
        int col = i / 4, row = i % 4;
        int x = col * 64, y = 13 + row * 8;
        char lbl[4]; std::snprintf(lbl, sizeof(lbl), "C%d", i);
        d.drawString(x, y, lbl, true);
        int pct = static_cast<int>(sd.cpu_core_pct[i]);
        d.drawBar(x + 12, y, BAR_W, BAR_H, pct, true);
    }

    // Temp + RAM on last row
    int y_bot = 13 + 4 * 8; // y=45
    d.drawString(0,  y_bot, std::format("T:{:.0f}C", sd.cpu_temp_c), true);

    uint64_t used_kb  = sd.ram_total_kb > sd.ram_avail_kb
                      ? sd.ram_total_kb - sd.ram_avail_kb : 0;
    float used_gb  = used_kb  / (1024.f * 1024.f);
    float total_gb = sd.ram_total_kb / (1024.f * 1024.f);
    d.drawString(48, y_bot, std::format("RAM{:.1f}/{:.0f}G", used_gb, total_gb), true);

    int ram_pct = sd.ram_total_kb
                ? static_cast<int>(100.f * used_kb / sd.ram_total_kb) : 0;
    d.drawBar(0, y_bot + 9, 128, 5, ram_pct, true);
}

// ============================================================================
//  GPIO thread  –  encoder + buttons
// ============================================================================
static void gpioThread(std::atomic<bool>& running, EventQueue& eq) {
    GpioLine enc_clk(kPinEncA.chip,   kPinEncA.offset);
    GpioLine enc_dt (kPinEncB.chip,   kPinEncB.offset);
    GpioLine enc_sw (kPinEncBtn.chip, kPinEncBtn.offset);
    GpioLine back   (kPinBack.chip,   kPinBack.offset);

    if (!enc_clk.openEdge()) {
        std::fprintf(stderr, "[GPIO] Failed to open encoder CLK\n");
    }
    if (!enc_dt.openValue()) {  // DT: plain input for value reading
        std::fprintf(stderr, "[GPIO] Failed to open encoder DT\n");
    }
    if (!enc_sw.openEdge()) {
        std::fprintf(stderr, "[GPIO] Failed to open encoder SW\n");
    }
    if (!back.openEdge()) {
        std::fprintf(stderr, "[GPIO] Failed to open back button\n");
    }

    // We poll CLK, SW, Back for edge events; DT is read on-demand
    pollfd pfds[3] = {};
    pfds[0].fd     = enc_clk.fd();  pfds[0].events = POLLIN;
    pfds[1].fd     = enc_sw.fd();   pfds[1].events = POLLIN;
    pfds[2].fd     = back.fd();     pfds[2].events = POLLIN;

    // Simple debounce: ignore events <5 ms after the last one
    using Clk = std::chrono::steady_clock;
    auto lastEnc  = Clk::now();
    auto lastSw   = Clk::now();
    auto lastBack = Clk::now();

    while (running.load(std::memory_order_relaxed)) {
        int ret = ::poll(pfds, 3, 100 /*ms timeout*/);
        if (ret <= 0) continue;

        auto now = Clk::now();

        // Encoder CLK edge
        if (pfds[0].revents & POLLIN) {
            uint32_t id = enc_clk.readEvent();
            if (id == GPIO_V2_LINE_EVENT_FALLING_EDGE &&
                now - lastEnc > 40ms) {
                lastEnc = now;
                bool dt = enc_dt.getValue();
                // DT high when CLK falls → CW, else CCW
                eq.push(dt ? Event::ENC_CW : Event::ENC_CCW);
            }
            pfds[0].revents = 0;
        }

        // Encoder SW button
        if (pfds[1].revents & POLLIN) {
            uint32_t id = enc_sw.readEvent();
            if (id == GPIO_V2_LINE_EVENT_FALLING_EDGE &&
                now - lastSw > 50ms) {
                lastSw = now;
                eq.push(Event::ENC_PUSH);
            }
            pfds[1].revents = 0;
        }

        // Back button
        if (pfds[2].revents & POLLIN) {
            uint32_t id = back.readEvent();
            if (id == GPIO_V2_LINE_EVENT_FALLING_EDGE &&
                now - lastBack > 50ms) {
                lastBack = now;
                eq.push(Event::BTN_BACK);
            }
            pfds[2].revents = 0;
        }
    }
}

// ============================================================================
//  SBC stats thread  –  updates SharedData at ~1 Hz
// ============================================================================
static void statsThread(std::atomic<bool>& running, SharedData& sd) {
    while (running.load(std::memory_order_relaxed)) {
        auto cores = readCpuUsage();   // blocks ~200 ms internally
        float temp = readCpuTemp();
        uint64_t total_kb, avail_kb;
        readRamInfo(total_kb, avail_kb);

        {
            std::lock_guard lock(sd.mtx);
            sd.cpu_core_pct = std::move(cores);
            sd.cpu_temp_c   = temp;
            sd.ram_total_kb = total_kb;
            sd.ram_avail_kb = avail_kb;
        }

        // Sleep remainder of the 1 s interval
        std::this_thread::sleep_for(800ms);
    }
}

// ============================================================================
//  Display update  –  called on every UI loop iteration
// ============================================================================
static void renderUI(OledDriver& d, const UIContext& ctx, const DataSnapshot& sd) {
    d.clear();

    if (ctx.state == UIState::SLEEPING) {
        // Nothing to draw – display is asleep
    }
    else if (ctx.state == UIState::MAIN) {
        renderMain(d, sd);
    }
    else if (ctx.state == UIState::MENU) {
        renderMenu(d, ctx.menu_sel);
    }
    else { // UIState::SCREEN
        switch (ctx.screen_idx) {
            case 0: renderJointState(d, sd); break;
            case 1: renderIMU(d, sd);        break;
            case 2: renderJoystick(d, sd);   break;
            case 3: renderLog(d, sd);        break;
            case 4: renderSBC(d, sd);        break;
            default: break;
        }
    }

    d.display();
}

// ============================================================================
//  UI event handler  –  state-machine transition
// ============================================================================
static void handleEvent(Event ev, UIContext& ctx, OledDriver& oled) {
    ctx.touch();

    if (ctx.state == UIState::SLEEPING) {
        // Any event wakes to MAIN screen
        ctx.state = UIState::MAIN;
        oled.setSleep(false);
        return;
    }

    switch (ctx.state) {
    // ── MAIN ────────────────────────────────────────────────────────────
    case UIState::MAIN:
        if (ev == Event::ENC_CW || ev == Event::ENC_CCW || ev == Event::ENC_PUSH)
            ctx.state = UIState::MENU;
        if (ev == Event::BTN_BACK)
            ; // Already at top level
        break;

    // ── MENU ────────────────────────────────────────────────────────────
    case UIState::MENU:
        if (ev == Event::ENC_CW) {
            ctx.menu_sel = (ctx.menu_sel + 1) % 5;
        } else if (ev == Event::ENC_CCW) {
            ctx.menu_sel = (ctx.menu_sel + 4) % 5;
        } else if (ev == Event::ENC_PUSH) {
            ctx.screen_idx = ctx.menu_sel;
            ctx.state = UIState::SCREEN;
        } else if (ev == Event::BTN_BACK) {
            ctx.state = UIState::MAIN;
        }
        break;

    // ── SCREEN ──────────────────────────────────────────────────────────
    case UIState::SCREEN:
        if (ev == Event::ENC_CW) {
            ctx.screen_idx = (ctx.screen_idx + 1) % 5;
            ctx.menu_sel   = ctx.screen_idx;
        } else if (ev == Event::ENC_CCW) {
            ctx.screen_idx = (ctx.screen_idx + 4) % 5;
            ctx.menu_sel   = ctx.screen_idx;
        } else if (ev == Event::BTN_BACK || ev == Event::ENC_PUSH) {
            ctx.state = UIState::MENU;
        }
        break;

    default: break;
    }
}

// ============================================================================
//  main
// ============================================================================
int main() {
    std::printf("=== MyBipedal OLED UI  (SSD1306 @ %s 0x%02X) ===\n",
                kI2CDev, kOledAddr);

    // ── Initialise OLED ──────────────────────────────────────────────────
    auto i2c = std::make_unique<I2CDevice>(kI2CDev, kOledAddr);
    if (!i2c->open()) {
        std::fprintf(stderr, "FATAL: cannot open I2C device %s\n", kI2CDev);
        return 1;
    }
    OledDriver oled(std::move(i2c));
    if (!oled.init()) {
        std::fprintf(stderr, "FATAL: OLED init failed\n");
        return 1;
    }
    oled.clear();
    oled.display();

    // ── Shared data & initial values ─────────────────────────────────────
    SharedData sd;
    sd.robot_ip = getLocalIP();

    // Seed some mock joint / IMU data for demonstration
    {
        std::lock_guard lock(sd.mtx);
        sd.joints = {
            {"LHip", 0.12f, 42.1f}, {"RHip", -0.10f, 41.8f},
            {"LKnee",0.45f, 43.0f}, {"RKnee", 0.43f, 43.2f},
            {"LAnk",-0.08f, 39.5f}, {"RAnk", -0.07f, 39.8f},
        };
        sd.imu_gx = 2.3f; sd.imu_gy = -1.1f; sd.imu_gz = 0.4f;
    } // <--- CLOSE THE LOCK SCOPE HERE

    // Call pushLog outside the lock, so it can safely lock the mutex itself
    sd.pushLog("[INFO] AimRT started");
    sd.pushLog("[INFO] Robot: MyBipedal");
    sd.pushLog("[WARN] Waiting for joint ctrl");

    // ── Background threads ───────────────────────────────────────────────
    std::atomic<bool> running{true};
    EventQueue         eq;

    std::thread gpio_thr(gpioThread, std::ref(running), std::ref(eq));
    std::thread stats_thr(statsThread, std::ref(running), std::ref(sd));

    // ── UI state machine ─────────────────────────────────────────────────
    UIContext ctx;
    ctx.state = UIState::SLEEPING;
    oled.setSleep(true);

    std::printf("[UI] Starting event loop.  Press Ctrl-C to exit.\n");

    while (true) {
        // Wait up to 200 ms for an event (keeps display refreshing for
        // live data screens like log / SBC)
        auto ev_opt = eq.pop(200ms);

        if (ev_opt) {
            bool was_sleeping = (ctx.state == UIState::SLEEPING);
            handleEvent(*ev_opt, ctx, oled);
            if (was_sleeping) eq.clear();
        }

        // ── Auto-sleep check ─────────────────────────────────────────────
        if (ctx.state != UIState::SLEEPING && ctx.isTimedOut()) {
            std::printf("[UI] Inactivity timeout – going to sleep\n");
            ctx.state = UIState::SLEEPING;
            oled.setSleep(true);
            oled.clear();
            oled.display();
            continue;
        }

        // ── Render ───────────────────────────────────────────────────────
        if (ctx.state != UIState::SLEEPING) {
            DataSnapshot snap = sd.snapshot();
            renderUI(oled, ctx, snap);
        }
    }

    // (unreachable in practice – send SIGINT to exit cleanly)
    running.store(false);
    gpio_thr.join();
    stats_thr.join();
    return 0;
}