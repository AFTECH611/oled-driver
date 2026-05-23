#pragma once
#include <cstdint>
#include <span>
#include <string>
#include <vector>

// ---------------------------------------------------------------------------
// I2CDevice – thin wrapper around Linux i2c-dev
// ---------------------------------------------------------------------------
class I2CDevice {
public:
    I2CDevice(std::string device, uint8_t addr);
    ~I2CDevice();

    I2CDevice(const I2CDevice&)            = delete;
    I2CDevice& operator=(const I2CDevice&) = delete;
    I2CDevice(I2CDevice&&) noexcept;
    I2CDevice& operator=(I2CDevice&&) noexcept;

    bool open();
    void close();
    [[nodiscard]] bool isOpen() const noexcept { return fd_ >= 0; }

    /// Write raw byte buffer directly to the slave address
    bool write(std::span<const uint8_t> data);

    /// Write control-byte then payload in a single kernel transaction
    bool writeReg(uint8_t control, std::span<const uint8_t> payload);

    /// Write register address, then read back data
    bool readReg(uint8_t reg, std::span<uint8_t> data);

private:
    std::string device_;
    uint8_t     addr_;
    int         fd_{-1};
};