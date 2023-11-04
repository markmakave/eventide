#pragma once

#include <utility>
#include <cmath>

#include <driver/i2c.h>
#include <esp_log.h>

namespace eventide {
    
class mpu {
public:

    enum {
        timeout = 1000 / portTICK_PERIOD_MS,
        address = 0x68
    };

    struct vec3 {
        float x, y, z;

        vec3 operator +(vec3 v) const {
            return { x + v.x, y + v.y, z + v.z };
        }

        vec3& operator +=(vec3 v) {
            *this = *this + v;
            return *this;
        }

        vec3 operator -(vec3 v) const {
            return { x - v.x, y - v.y, z - v.z };
        }

        vec3& operator -=(vec3 v)  {
            *this = *this - v;
            return *this;
        }

        vec3 operator *(float f) const {
            return { x * f, y * f, z * f };
        }

        vec3& operator *= (float f) {
            *this = *this * f;
            return *this;
        }

        vec3 operator /(float f) {
            return { x / f, y / f, z / f };
        }

        vec3& operator /= (float f) {
            *this = *this / f;
            return *this;
        }

    };

public:

    mpu(uint8_t sda, uint8_t scl, uint32_t frequency = 100000) {
        i2c_config_t conf = {
            .mode = I2C_MODE_MASTER,
            .sda_io_num = sda,
            .scl_io_num = scl,
            .sda_pullup_en = GPIO_PULLUP_ENABLE,
            .scl_pullup_en = GPIO_PULLUP_ENABLE,
            .master = frequency,
            .clk_flags = 0,
        };

        i2c_param_config(I2C_NUM_0, &conf);
        i2c_driver_install(I2C_NUM_0, conf.mode, 0, 0, 0);

        wake();

        // set accel amplitude to 8

        write(0x1C, 0x10);
        _accel_amplitude = 8;

        // set gyro amplitude to 500
        write(0x1C, 0x08);
        _gyro_amplitude = 500;

        // calibrate
        calibrate(1000);
    }

    void calibrate(int iterations) {
        for (int i = 0; i < iterations; i++) {
            vec3 accel_raw = accel(),
                 gyro_raw = gyro();

            _accel_bias += accel_raw;
            _gyro_bias += gyro_raw;
        }

        _accel_bias /= iterations;
        _gyro_bias /= iterations;
    }

    vec3 accel() {
        uint8_t data[6];

        read(0x3B, data, sizeof(data));

        return vec3(
            int16_t(data[0] << 8 | data[1]), 
            int16_t(data[2] << 8 | data[3]), 
            int16_t(data[4] << 8 | data[5])
        ) * _accel_amplitude / (1 << 15) - _accel_bias;
    }

    vec3 gyro() {
        uint8_t data[6];

        read(0x43, data, sizeof(data));

        return vec3(
            int16_t(data[0] << 8 | data[1]),
            int16_t(data[2] << 8 | data[3]),
            int16_t(data[4] << 8 | data[5])
        ) * _gyro_amplitude / (1 << 15) - _gyro_bias;
    }

    std::pair<float, float> angle() {
        vec3 accel_raw = accel(),
             gyro_raw = gyro();

        float roll_rate = gyro_raw.x,
              pitch_rate = gyro_raw.y,
              roll_angle = atan2f(accel_raw.y, std::sqrt(accel_raw.x * accel_raw.x + accel_raw.z * accel_raw.z)),
              pitch_angle = atan2f(-accel_raw.x, std::sqrt(accel_raw.y * accel_raw.y + accel_raw.z * accel_raw.z));

        return { roll_angle, pitch_angle };
    }

    void wake() {
        write(0x6B, 0x00);
    }

    void sleep() {
        write(0x6B, 0x01);
    }

    void write(uint8_t reg, const void* data, size_t size) {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (address << 1) | I2C_MASTER_WRITE, true);
        i2c_master_write_byte(cmd, reg, true);
        i2c_master_write(cmd, (const uint8_t*)data, size, true);
        i2c_master_stop(cmd);
        i2c_master_cmd_begin(I2C_NUM_0, cmd, timeout);
        i2c_cmd_link_delete(cmd);
    }

    void write(uint8_t reg, uint8_t data) {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (address << 1) | I2C_MASTER_WRITE, true);
        i2c_master_write_byte(cmd, reg, true);
        i2c_master_write_byte(cmd, data, true);
        i2c_master_stop(cmd);
        i2c_master_cmd_begin(I2C_NUM_0, cmd, timeout);
        i2c_cmd_link_delete(cmd);
    }

    void read(uint8_t reg, void* data, size_t size) {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (address << 1) | I2C_MASTER_WRITE, true);
        i2c_master_write_byte(cmd, reg, true);
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (address << 1) | I2C_MASTER_READ, true);
        i2c_master_read(cmd, (uint8_t*)data, size, I2C_MASTER_LAST_NACK);
        i2c_master_stop(cmd);
        i2c_master_cmd_begin(I2C_NUM_0, cmd, timeout);
        i2c_cmd_link_delete(cmd);
    }

    uint8_t read(uint8_t reg) {
        uint8_t data;

        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (address << 1) | I2C_MASTER_WRITE, true);
        i2c_master_write_byte(cmd, reg, true);
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (address << 1) | I2C_MASTER_READ, true);
        i2c_master_read_byte(cmd, &data, I2C_MASTER_LAST_NACK);
        i2c_master_stop(cmd);
        i2c_master_cmd_begin(I2C_NUM_0, cmd, timeout);
        i2c_cmd_link_delete(cmd);

        return data;
    }

protected:

    vec3 _accel_bias = { 0.0f, 0.0f, 0.0f };
    vec3 _gyro_bias = { 0.0f, 0.0f, 0.0f };

    float _accel_amplitude = 8.0f;
    float _gyro_amplitude = 500.0f;
};

} // namespace eventide
