#pragma once
#include <array>
#include <cstdint>
#include <initializer_list>
#include <memory>
#include <span>
#include <string_view>

class I2CDevice;

// ---------------------------------------------------------------------------
//  OledDriver – SSD1306 128×64 framebuffer driver
//  Uses horizontal addressing mode; one call to display() pushes the
//  entire 1 024-byte framebuffer over I²C.
// ---------------------------------------------------------------------------
class OledDriver {
public:
    static constexpr int WIDTH  = 128;
    static constexpr int HEIGHT = 64;
    static constexpr int PAGES  = HEIGHT / 8;   // 8

    // Font cell (6 columns × 8 rows) – char width with spacing
    static constexpr int FONT_W = 6;
    static constexpr int FONT_H = 8;

    explicit OledDriver(std::unique_ptr<I2CDevice> i2c);
    ~OledDriver() = default;

    OledDriver(const OledDriver&)            = delete;
    OledDriver& operator=(const OledDriver&) = delete;

    // ── Lifecycle ────────────────────────────────────────────────────────
    [[nodiscard]] bool init();

    // ── Framebuffer ops ──────────────────────────────────────────────────
    void clear(bool on = false);
    void display();                         ///< push framebuffer to OLED

    // ── Pixel / primitive drawing ─────────────────────────────────────────
    void setPixel(int x, int y, bool on = true);
    bool getPixel(int x, int y) const;

    void drawHLine(int x, int y, int w, bool on = true);
    void drawVLine(int x, int y, int h, bool on = true);
    void drawLine(int x0, int y0, int x1, int y1, bool on = true);
    void drawRect(int x, int y, int w, int h, bool on = true);
    void fillRect(int x, int y, int w, int h, bool on = true);
    void drawCircle(int cx, int cy, int r, bool on = true);
    void fillCircle(int cx, int cy, int r, bool on = true);

    // ── Text ─────────────────────────────────────────────────────────────
    /// Draw one printable ASCII character.
    /// scale=1 → 6×8 px, scale=2 → 12×16 px, …
    void drawChar(int x, int y, char c, bool on = true, uint8_t scale = 1);

    /// Draw a string; wrapping is NOT done – caller clips manually.
    void drawString(int x, int y, std::string_view str,
                    bool on = true, uint8_t scale = 1);

    /// Pixel width of a string at the given scale.
    [[nodiscard]] int textWidth(std::string_view str, uint8_t scale = 1) const;

    // ── Display control ───────────────────────────────────────────────────
    void setSleep(bool sleep);
    void setInvert(bool invert);
    void setContrast(uint8_t contrast);

    // ── Helper: draw a horizontal progress bar ───────────────────────────
    /// Draws border + fill in one call.  pct in [0,100].
    void drawBar(int x, int y, int w, int h, int pct, bool on = true);

private:
    bool sendCmd(uint8_t cmd);
    bool sendCmds(std::initializer_list<uint8_t> cmds);
    bool sendData(std::span<const uint8_t> data);

    std::unique_ptr<I2CDevice>          i2c_;
    std::array<uint8_t, WIDTH * PAGES>  buf_{};

    // 6×8 bitmap font, printable ASCII 0x20–0x7F (96 entries)
    static const uint8_t FONT_6x8[96][6];
};