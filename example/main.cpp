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

#include <gpiod.h> // Thêm thư viện này ở đầu file

using namespace std::chrono_literals;

// ============================================================================
//  Hardware constants
// ============================================================================
static constexpr const char* kI2CDev    = "/dev/i2c-7";
static constexpr uint8_t     kOledAddr  = 0x3C;
static constexpr auto kSleepTimeout = 30s;


struct GpioPin {
    const char* chip;
    unsigned int line;
};

// Định nghĩa lại danh sách chân theo phần cứng của Radxa 5B+
inline constexpr GpioPin kPinClk   { "/dev/gpiochip3", 13 }; // CLK (A)
inline constexpr GpioPin kPinDt    { "/dev/gpiochip3", 15 }; // DT (B)
inline constexpr GpioPin kPinSw    { "/dev/gpiochip3", 16 }; // SW (Push)
inline constexpr GpioPin kPinBack  { "/dev/gpiochip4", 20 }; // Back Button

// Lớp bọc an toàn để đọc trạng thái GPIO bằng libgpiod
class InputPin {
public:
    explicit InputPin(const GpioPin& pin)
    {
        chip_ = gpiod_chip_open(pin.chip);
        if (!chip_) {
            throw std::runtime_error("Failed to open chip");
        }

        line_ = gpiod_chip_get_line(chip_, pin.line);
        if (!line_) {
            throw std::runtime_error("Failed to get line");
        }

        // --- Cấu hình Input kèm điện trở kéo lên (Pull-up) ---
        gpiod_line_request_config config{};
        config.consumer = "encoder_debug";
        config.request_type = GPIOD_LINE_REQUEST_DIRECTION_INPUT;
        config.flags = GPIOD_LINE_REQUEST_FLAG_BIAS_PULL_UP; 

        if (gpiod_line_request(line_, &config, 0) < 0) {
            throw std::runtime_error("Failed to request input with pull-up");
        }
    }

    ~InputPin() {
        if (line_) gpiod_line_release(line_);
        if (chip_) gpiod_chip_close(chip_);
    }

    int Read() const {
        return gpiod_line_get_value(line_);
    }

private:
    gpiod_chip* chip_{nullptr};
    gpiod_line* line_{nullptr};
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
    std::string key, unit;   // ← thêm unit
    uint64_t val;
    total_kb = avail_kb = 0;
    while (f >> key >> val >> unit) {   // ← đọc đủ 3 field
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
    int     scroll_offset = 0;   // ← NEW
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

// Vẽ thanh cuộn nhỏ bên phải, chỉ hiện khi content > view
static void drawScrollbar(OledDriver& d, int scroll, int content_h) {
    constexpr int VIEW_H = 54, TRACK_Y = 10;
    if (content_h <= VIEW_H) return;
    int max_s  = content_h - VIEW_H;
    int bar_h  = std::max(6, VIEW_H * VIEW_H / content_h);
    int bar_y  = TRACK_Y + scroll * (VIEW_H - bar_h) / max_s;
    d.fillRect(126, bar_y, 2, bar_h, true);
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

// ── Screen 0: Joint State ────────────────────────────────────────────────
static void renderJointState(OledDriver& d, const DataSnapshot& sd, int& scroll) {
    drawTitle(d, "Joint State");
    const auto& J = sd.joints;
    int content_h = ((int(J.size()) + 1) / 2) * 17;
    scroll = std::clamp(scroll, 0, std::max(0, content_h - 54));

    for (int i = 0; i < (int)J.size() && i < 12; ++i) {
        int x = (i % 2) * 64;
        int y = 13 + (i / 2) * 17 - scroll;
        if (y + 16 < 10 || y > 63) continue;
        d.drawString(x,    y,   J[i].name, true);
        d.drawString(x,    y+8, std::format("{:.1f}", J[i].pos),   true);
        d.drawString(x+36, y+8, std::format("{:.0f}C", J[i].temp), true);
    }
    drawScrollbar(d, scroll, content_h);
}

// ── Screen 1: IMU ────────────────────────────────────────────────────────
static void renderIMU(OledDriver& d, const DataSnapshot& sd, int& scroll) {
    drawTitle(d, "IMU State");
    // 6 dòng × 10px = 60px, view = 54px → max_scroll = 6
    constexpr int CONTENT_H = 60;
    scroll = std::clamp(scroll, 0, std::max(0, CONTENT_H - 54));

    const std::pair<int, std::string> rows[] = {
        {12, std::format("Gx{:+6.1f}", sd.imu_gx)},
        {22, std::format("Gy{:+6.1f}", sd.imu_gy)},
        {32, std::format("Gz{:+6.1f}", sd.imu_gz)},
        {42, std::format("Ax{:+5.2f}", sd.imu_ax)},
        {52, std::format("Ay{:+5.2f}", sd.imu_ay)},
        {62, std::format("Az{:+5.2f}", sd.imu_az)},  // Az bây giờ hiện được khi scroll
    };
    for (auto& [y, text] : rows) {
        int sy = y - scroll;
        if (sy >= 10 && sy < 64) d.drawString(0, sy, text, true);
    }
    // Tilt dot cố định (không scroll)
    constexpr int CX=100, CY=38, R=22;
    d.drawCircle(CX, CY, R, true);
    d.drawHLine(CX-R, CY, 2*R+1, true);
    d.drawVLine(CX, CY-R, 2*R+1, true);
    float nx = std::clamp(sd.imu_gy / 90.f, -1.f, 1.f);
    float ny = std::clamp(sd.imu_gx / 90.f, -1.f, 1.f);
    d.fillCircle(CX + (int)(nx*(R-3)), CY + (int)(ny*(R-3)), 3, true);
    drawScrollbar(d, scroll, CONTENT_H);
}

// ── Screen 3: Log ────────────────────────────────────────────────────────
static void renderLog(OledDriver& d, const DataSnapshot& sd, int& scroll) {
    drawTitle(d, "AimRT Log");
    constexpr int kRowH = 9, kVisRows = 6, kChars = 21;
    const auto& lines = sd.log_lines;
    int total     = (int)lines.size();
    int max_scroll = std::max(0, total - kVisRows);
    scroll = std::clamp(scroll, 0, max_scroll);

    // scroll=0 → log mới nhất, scroll tăng → xem lùi về trước
    int start = std::max(0, total - kVisRows - scroll);
    for (int i = 0; i < kVisRows && (start+i) < total; ++i) {
        std::string row = lines[start+i];
        if ((int)row.size() > kChars) row.resize(kChars);
        d.drawString(0, 12 + i*kRowH, row, true);
    }
    if (total > kVisRows) {
        int bar_h = std::max(6, kVisRows * 54 / total);
        int bar_y = 10 + (max_scroll > 0 ? scroll * (54 - bar_h) / max_scroll : 0);
        d.fillRect(126, bar_y, 2, bar_h, true);
    }
}

// ── Screen 4: SBC ────────────────────────────────────────────────────────
static void renderSBC(OledDriver& d, const DataSnapshot& sd, int& scroll) {
    drawTitle(d, "SBC Status");
    constexpr int CONTENT_H = 58;
    scroll = std::clamp(scroll, 0, std::max(0, CONTENT_H - 54));

    const int ncores = (int)sd.cpu_core_pct.size();
    for (int i = 0; i < ncores && i < 8; ++i) {
        int x = (i / 4) * 64;
        int y = 13 + (i % 4) * 8 - scroll;
        if (y < 10 || y > 63) continue;
        char lbl[4]; std::snprintf(lbl, sizeof(lbl), "C%d", i);
        d.drawString(x, y, lbl, true);
        d.drawBar(x+12, y, 48, 5, (int)sd.cpu_core_pct[i], true);
    }
    int y_bot = 45 - scroll;
    if (y_bot >= 10 && y_bot < 64) {
        d.drawString(0, y_bot, std::format("T:{:.0f}C", sd.cpu_temp_c), true);
        uint64_t used_kb = sd.ram_total_kb > sd.ram_avail_kb
                         ? sd.ram_total_kb - sd.ram_avail_kb : 0;
        d.drawString(48, y_bot, std::format("RAM{:.1f}/{:.0f}G",
            used_kb / (1024.f*1024.f), sd.ram_total_kb / (1024.f*1024.f)), true);
    }
    if (y_bot + 9 >= 10 && y_bot + 9 < 64) {
        int pct = sd.ram_total_kb
                ? (int)(100.f * (sd.ram_total_kb - sd.ram_avail_kb) / sd.ram_total_kb) : 0;
        d.drawBar(0, y_bot+9, 128, 5, pct, true);
    }
    drawScrollbar(d, scroll, CONTENT_H);
}

// ── Screen 2: Joy-stick ────────────────────────────────────────────────────────
static void renderJoystick(OledDriver& d, const DataSnapshot& sd) {
    drawTitle(d, "Joystick");
    // bit map: A=0, B=1, X=2, Y=3, LB=4, RB=5, St=6, Bk=7
    auto btn = [&](int bit) { return (sd.joy_buttons >> bit) & 1u; };

    // Vẽ nút: tô nền trắng + chữ đen khi nhấn, ngược lại khi thả
    auto drawBtn = [&](int x, int y, const char* label, bool pressed) {
        int w = d.textWidth(label) + 2;
        if (pressed) {
            d.fillRect(x-1, y-1, w, 9, true);
            d.drawString(x, y, label, false);
        } else {
            d.drawString(x, y, label, true);
        }
    };

    // ── Hàng trên: LB | Bk | St | RB ──────────────────────────────────
    drawBtn(1,   12, "LB", btn(4));
    drawBtn(43,  12, "Bk", btn(7));
    drawBtn(69,  12, "St", btn(6));
    drawBtn(114, 12, "RB", btn(5));

    // ── Left Stick (lớn, dominant) ─────────────────────────────────────
    constexpr int LX=22, LY=32, LR=14;
    d.drawCircle(LX, LY, LR, true);
    d.drawHLine(LX-LR, LY, 2*LR+1, true);
    d.drawVLine(LX, LY-LR, 2*LR+1, true);
    d.fillCircle(LX + (int)(sd.joy_lx*(LR-3)),
                 LY + (int)(sd.joy_ly*(LR-3)), 3, true);

    // ── Right Stick (nhỏ hơn) ──────────────────────────────────────────
    constexpr int RX=82, RY=22, RR=8;
    d.drawCircle(RX, RY, RR, true);
    d.drawHLine(RX-RR, RY, 2*RR+1, true);
    d.drawVLine(RX, RY-RR, 2*RR+1, true);
    d.fillCircle(RX + (int)(sd.joy_rx*(RR-2)),
                 RY + (int)(sd.joy_ry*(RR-2)), 2, true);  // dot nhỏ hơn

    // ── ABXY Diamond (bên phải, dưới R-stick) ──────────────────────────
    //      Y(3)
    //   X(2)  B(1)
    //      A(0)
    drawBtn(107, 28, "Y", btn(3));  // top
    drawBtn(96,  37, "X", btn(2));  // left
    drawBtn(116, 37, "B", btn(1));  // right
    drawBtn(107, 46, "A", btn(0));  // bottom

    // ── Giá trị analog ─────────────────────────────────────────────────
    d.drawString(0,  56, std::format("L{:+.1f} {:+.1f}", sd.joy_lx, sd.joy_ly), true);
    d.drawString(66, 56, std::format("R{:+.1f} {:+.1f}", sd.joy_rx, sd.joy_ry), true);
}

// ============================================================================
//  GPIO thread  –  encoder + buttons
// ============================================================================
void gpioThread(std::atomic<bool>& running, EventQueue& eq) {
    std::printf("[GPIO] Thread started using libgpiod high-frequency polling.\n");
    
    try {
        // Khởi tạo các chân IO
        InputPin clk(kPinClk);
        InputPin dt(kPinDt);
        InputPin sw(kPinSw);
        InputPin back(kPinBack);

        // Lưu trạng thái trước đó để bắt sườn tín hiệu (Edge change)
        int lastClk  = clk.Read();
        int lastSw   = sw.Read();
        int lastBack = back.Read();

        while (running) {
            // ──────────────────────────────────────────────────────────
            // 1. Xử lý Vặn Encoder (Bắt sườn xuống của chân CLK)
            // ──────────────────────────────────────────────────────────
            int currentClk = clk.Read();
            if (currentClk != lastClk && currentClk == 0) {
                int dtState = dt.Read();
                
                if (dtState != currentClk) {
                    eq.push(Event::ENC_CCW);  // Vặn theo chiều kim đồng hồ (Lên)
                } else {
                    eq.push(Event::ENC_CW); // Vặn ngược chiều kim đồng hồ (Xuống)
                }
                
                // Trì hoãn cực ngắn để ổn định tiếp điểm cơ học của encoder
                std::this_thread::sleep_for(std::chrono::milliseconds(2));
            }
            lastClk = currentClk;

            // ──────────────────────────────────────────────────────────
            // 2. Xử lý Nút nhấn Encoder (SW - Bắt sườn xuống khi bấm)
            // ──────────────────────────────────────────────────────────
            int swState = sw.Read();
            if (swState != lastSw) {
                // Đợi tín hiệu ổn định (debounce cả nhấn lẫn nhả)
                std::this_thread::sleep_for(std::chrono::milliseconds(50));
                
                int confirmedState = sw.Read(); // Đọc lại để xác nhận
                if (confirmedState == swState) {  // Trạng thái thực sự đã đổi
                    if (confirmedState == 0) {
                        eq.push(Event::ENC_PUSH);
                    }
                    // Nếu == 1 (nhả): không push, chỉ cập nhật lastSw
                    lastSw = confirmedState;
                }
                // Nếu confirmedState != swState: là bounce → bỏ qua, KHÔNG cập nhật lastSw
            }

            // ──────────────────────────────────────────────────────────
            // 3. Xử lý Nút BACK (Bắt sườn xuống khi bấm nút quay lại)
            // ──────────────────────────────────────────────────────────
            int backState = back.Read();
            if (backState != lastBack) {
                if (backState == 0) { // Trạng thái nhấn xuống (LOW)
                    eq.push(Event::BTN_BACK); // Đẩy sự kiện BACK vào hàng đợi
                    std::this_thread::sleep_for(std::chrono::milliseconds(50));
                }
                lastBack = backState;
            }

            // ──────────────────────────────────────────────────────────
            // Nhịp quét chu kỳ 500 micro-giây (Tần số lấy mẫu 2KHz)
            // ──────────────────────────────────────────────────────────
            std::this_thread::sleep_for(std::chrono::microseconds(500));
        }
        
    } catch (const std::exception& e) {
        std::fprintf(stderr, "[GPIO Thread CRITICAL ERROR]: %s\n", e.what());
        running = false; // Hạ cờ hệ thống dừng an toàn nếu lỗi driver phần cứng
    }
    
    std::printf("[GPIO] Thread stopped.\n");
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
// renderUI giờ nhận UIContext& (không phải const) để renderer clamp scroll
static void renderUI(OledDriver& d, UIContext& ctx, const DataSnapshot& sd) {
    d.clear();
    if      (ctx.state == UIState::SLEEPING) { /* nothing */ }
    else if (ctx.state == UIState::MAIN)     renderMain(d, sd);
    else if (ctx.state == UIState::MENU)     renderMenu(d, ctx.menu_sel);
    else {
        switch (ctx.screen_idx) {
            case 0: renderJointState(d, sd, ctx.scroll_offset); break;
            case 1: renderIMU       (d, sd, ctx.scroll_offset); break;
            case 2: renderJoystick  (d, sd);                    break;  // không scroll
            case 3: renderLog       (d, sd, ctx.scroll_offset); break;
            case 4: renderSBC       (d, sd, ctx.scroll_offset); break;
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
            if (ev == Event::ENC_CW || ev == Event::ENC_CCW || ev == Event::ENC_PUSH) ctx.state = UIState::MENU;
            if (ev == Event::BTN_BACK) {} // Already at top level
            break;

        // ── MENU ────────────────────────────────────────────────────────────
        case UIState::MENU:
            if (ev == Event::ENC_CW)       { ctx.menu_sel = (ctx.menu_sel + 1) % 5; }
            else if (ev == Event::ENC_CCW) { ctx.menu_sel = (ctx.menu_sel + 4) % 5; }
            else if (ev == Event::ENC_PUSH) {
                ctx.screen_idx    = ctx.menu_sel;
                ctx.state         = UIState::SCREEN;
                ctx.scroll_offset = 0;  // ← reset khi vào màn mới
            }
            else if (ev == Event::BTN_BACK) { ctx.state = UIState::MAIN; }
            break;

        // ── SCREEN ──────────────────────────────────────────────────────────
        case UIState::SCREEN:
            if (ev == Event::ENC_CW) {
                ctx.scroll_offset += 8;              // renderer sẽ clamp max
            } else if (ev == Event::ENC_CCW) {
                ctx.scroll_offset = std::max(0, ctx.scroll_offset - 8);
            } else if (ev == Event::BTN_BACK || ev == Event::ENC_PUSH) {
                ctx.state         = UIState::MENU;
                ctx.scroll_offset = 0;
            }
            break;
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
            {"LHR", 0.12f, 42.1f}, {"RHR", -0.10f, 41.8f},
            {"LHP", 0.12f, 42.1f}, {"RHP", -0.10f, 41.8f},
            {"LK",0.45f, 43.0f}, {"RK", 0.43f, 43.2f},
            {"LAP",-0.08f, 39.5f}, {"RAP", -0.07f, 39.8f},
        };
        sd.imu_gx = 2.3f; sd.imu_gy = -1.1f; sd.imu_gz = 0.4f;
    } // <--- CLOSE THE LOCK SCOPE HERE

    // Call pushLog outside the lock, so it can safely lock the mutex itself
    sd.pushLog("[INFO] AimRT started");
    sd.pushLog("[INFO] Robot: MyBipedal");
    sd.pushLog("[WARN] Waiting for joint ctrl");
    sd.pushLog("[WARN] Waiting for joint ctrl");
    sd.pushLog("[WARN] Waiting for joint ctrl");
    sd.pushLog("[WARN] Waiting for joint ctrl");
    sd.pushLog("[WARN] Waiting for joint ctrl");
    sd.pushLog("[WARN] Waiting for joint ctrl");
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
        auto ev_opt = eq.pop(200ms); // Đợi tối đa 200ms để tối ưu tài nguyên CPU

        if (ev_opt) {
            // Lưu lại trạng thái trước khi xử lý sự kiện để biết màn hình có đang ngủ không
            bool was_sleeping = (ctx.state == UIState::SLEEPING);

            handleEvent(*ev_opt, ctx, oled);

            // Nếu màn hình vừa được đánh thức từ trạng thái ngủ, 
            // lập tức dọn sạch mọi event dội nhiễu cơ học sinh ra trong tích tắc đó.
            if (was_sleeping) {
                eq.clear(); 
            }
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