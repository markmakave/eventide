#pragma once

#include <iostream>
#include <stdexcept>

#include "pin.hpp"
#include "uart.hpp"

namespace eventide
{

class vesc : public uart
{
    using base = uart;

    static constexpr u8 vesc_header = 0x02;
    static constexpr u8 vesc_footer = 0x03;

    enum commands
    {
        COMM_FW_VERSION							= 0,
        COMM_JUMP_TO_BOOTLOADER					= 1,
        COMM_ERASE_NEW_APP						= 2,
        COMM_WRITE_NEW_APP_DATA					= 3,
        COMM_GET_VALUES							= 4,
        COMM_SET_DUTY							= 5,
        COMM_SET_CURRENT						= 6,
        COMM_SET_CURRENT_BRAKE					= 7,
        COMM_SET_RPM							= 8,
        COMM_SET_POS							= 9,
        COMM_SET_HANDBRAKE						= 10,
        COMM_SET_DETECT							= 11,
        COMM_SET_SERVO_POS						= 12,
        COMM_SET_MCCONF							= 13,
        COMM_GET_MCCONF							= 14,
        COMM_GET_MCCONF_DEFAULT					= 15,
        COMM_SET_APPCONF						= 16,
        COMM_GET_APPCONF						= 17,
        COMM_GET_APPCONF_DEFAULT				= 18,
        COMM_SAMPLE_PRINT						= 19,
        COMM_TERMINAL_CMD						= 20,
        COMM_PRINT								= 21,
        COMM_ROTOR_POSITION						= 22,
        COMM_EXPERIMENT_SAMPLE					= 23,
        COMM_DETECT_MOTOR_PARAM					= 24,
        COMM_DETECT_MOTOR_R_L					= 25,
        COMM_DETECT_MOTOR_FLUX_LINKAGE			= 26,
        COMM_DETECT_ENCODER						= 27,
        COMM_DETECT_HALL_FOC					= 28,
        COMM_REBOOT								= 29,
        COMM_ALIVE								= 30,
        COMM_GET_DECODED_PPM					= 31,
        COMM_GET_DECODED_ADC					= 32,
        COMM_GET_DECODED_CHUK					= 33,
        COMM_FORWARD_CAN						= 34,
        COMM_SET_CHUCK_DATA						= 35,
        COMM_CUSTOM_APP_DATA					= 36,
        COMM_NRF_START_PAIRING					= 37,
        COMM_GPD_SET_FSW						= 38,
        COMM_GPD_BUFFER_NOTIFY					= 39,
        COMM_GPD_BUFFER_SIZE_LEFT				= 40,
        COMM_GPD_FILL_BUFFER					= 41,
        COMM_GPD_OUTPUT_SAMPLE					= 42,
        COMM_GPD_SET_MODE						= 43,
        COMM_GPD_FILL_BUFFER_INT8				= 44,
        COMM_GPD_FILL_BUFFER_INT16				= 45,
        COMM_GPD_SET_BUFFER_INT_SCALE			= 46,
        COMM_GET_VALUES_SETUP					= 47,
        COMM_SET_MCCONF_TEMP					= 48,
        COMM_SET_MCCONF_TEMP_SETUP				= 49,
        COMM_GET_VALUES_SELECTIVE				= 50,
        COMM_GET_VALUES_SETUP_SELECTIVE			= 51,
        COMM_EXT_NRF_PRESENT					= 52,
        COMM_EXT_NRF_ESB_SET_CH_ADDR			= 53,
        COMM_EXT_NRF_ESB_SEND_DATA				= 54,
        COMM_EXT_NRF_ESB_RX_DATA				= 55,
        COMM_EXT_NRF_SET_ENABLED				= 56,
        COMM_DETECT_MOTOR_FLUX_LINKAGE_OPENLOOP	= 57,
        COMM_DETECT_APPLY_ALL_FOC				= 58,
        COMM_JUMP_TO_BOOTLOADER_ALL_CAN			= 59,
        COMM_ERASE_NEW_APP_ALL_CAN				= 60,
        COMM_WRITE_NEW_APP_DATA_ALL_CAN			= 61,
        COMM_PING_CAN							= 62,
        COMM_APP_DISABLE_OUTPUT					= 63,
        COMM_TERMINAL_CMD_SYNC					= 64,
        COMM_GET_IMU_DATA						= 65,
        COMM_BM_CONNECT							= 66,
        COMM_BM_ERASE_FLASH_ALL					= 67,
        COMM_BM_WRITE_FLASH						= 68,
        COMM_BM_REBOOT							= 69,
        COMM_BM_DISCONNECT						= 70,
        COMM_BM_MAP_PINS_DEFAULT				= 71,
        COMM_BM_MAP_PINS_NRF5X					= 72,
        COMM_ERASE_BOOTLOADER					= 73,
        COMM_ERASE_BOOTLOADER_ALL_CAN			= 74,
        COMM_PLOT_INIT							= 75,
        COMM_PLOT_DATA							= 76,
        COMM_PLOT_ADD_GRAPH						= 77,
        COMM_PLOT_SET_GRAPH						= 78,
        COMM_GET_DECODED_BALANCE				= 79,
        COMM_BM_MEM_READ						= 80,
        COMM_WRITE_NEW_APP_DATA_LZO				= 81,
        COMM_WRITE_NEW_APP_DATA_ALL_CAN_LZO		= 82,
        COMM_BM_WRITE_FLASH_LZO					= 83,
        COMM_SET_CURRENT_REL					= 84,
        COMM_CAN_FWD_FRAME						= 85,
        COMM_SET_BATTERY_CUT					= 86,
        COMM_SET_BLE_NAME						= 87,
        COMM_SET_BLE_PIN						= 88,
        COMM_SET_CAN_MODE						= 89,
        COMM_GET_IMU_CALIBRATION				= 90,
        COMM_GET_MCCONF_TEMP					= 91,
        COMM_GET_CUSTOM_CONFIG_XML				= 92,
        COMM_GET_CUSTOM_CONFIG					= 93,
        COMM_GET_CUSTOM_CONFIG_DEFAULT			= 94,
        COMM_SET_CUSTOM_CONFIG					= 95,
        COMM_BMS_GET_VALUES						= 96,
        COMM_BMS_SET_CHARGE_ALLOWED				= 97,
        COMM_BMS_SET_BALANCE_OVERRIDE			= 98,
        COMM_BMS_RESET_COUNTERS					= 99,
        COMM_BMS_FORCE_BALANCE					= 100,
        COMM_BMS_ZERO_CURRENT_OFFSET			= 101,
        COMM_JUMP_TO_BOOTLOADER_HW				= 102,
        COMM_ERASE_NEW_APP_HW					= 103,
        COMM_WRITE_NEW_APP_DATA_HW				= 104,
        COMM_ERASE_BOOTLOADER_HW				= 105,
        COMM_JUMP_TO_BOOTLOADER_ALL_CAN_HW		= 106,
        COMM_ERASE_NEW_APP_ALL_CAN_HW			= 107,
        COMM_WRITE_NEW_APP_DATA_ALL_CAN_HW		= 108,
        COMM_ERASE_BOOTLOADER_ALL_CAN_HW		= 109,
        COMM_SET_ODOMETER						= 110,
        COMM_PSW_GET_STATUS						= 111,
        COMM_PSW_SWITCH							= 112,
        COMM_BMS_FWD_CAN_RX						= 113,
        COMM_BMS_HW_DATA						= 114,
        COMM_GET_BATTERY_CUT					= 115,
        COMM_BM_HALT_REQ						= 116,
        COMM_GET_QML_UI_HW						= 117,
        COMM_GET_QML_UI_APP						= 118,
        COMM_CUSTOM_HW_DATA						= 119,
        COMM_QMLUI_ERASE						= 120,
        COMM_QMLUI_WRITE						= 121,
        COMM_IO_BOARD_GET_ALL					= 122,
        COMM_IO_BOARD_SET_PWM					= 123,
        COMM_IO_BOARD_SET_DIGITAL				= 124,
        COMM_BM_MEM_WRITE						= 125,
        COMM_BMS_BLNC_SELFTEST					= 126,
        COMM_GET_EXT_HUM_TMP					= 127,
        COMM_GET_STATS							= 128,
        COMM_RESET_STATS						= 129,
        COMM_LISP_READ_CODE						= 130,
        COMM_LISP_WRITE_CODE					= 131,
        COMM_LISP_ERASE_CODE					= 132,
        COMM_LISP_SET_RUNNING					= 133,
        COMM_LISP_GET_STATS						= 134,
        COMM_LISP_PRINT							= 135,
        COMM_BMS_SET_BATT_TYPE					= 136,
        COMM_BMS_GET_BATT_TYPE					= 137,
        COMM_LISP_REPL_CMD						= 138,
        COMM_LISP_STREAM_CODE					= 139,
        COMM_FILE_LIST							= 140,
        COMM_FILE_READ							= 141,
        COMM_FILE_WRITE							= 142,
        COMM_FILE_MKDIR							= 143,
        COMM_FILE_REMOVE						= 144,
        COMM_LOG_START							= 145,
        COMM_LOG_STOP							= 146,
        COMM_LOG_CONFIG_FIELD					= 147,
        COMM_LOG_DATA_F32						= 148,
        COMM_SET_APPCONF_NO_STORE				= 149,
        COMM_GET_GNSS							= 150,
        COMM_LOG_DATA_F64						= 151,
        COMM_LISP_RMSG							= 152,
        //COMM_PINLOCK1							= 153,
        //COMM_PINLOCK2							= 154,
        //COMM_PINLOCK3							= 155,
        COMM_SHUTDOWN							= 156,
        COMM_FW_INFO							= 157,
	    COMM_CAN_UPDATE_BAUD_ALL				= 158,
    };

    enum fault
    {
        FAULT_CODE_NONE = 0,
        FAULT_CODE_OVER_VOLTAGE,
        FAULT_CODE_UNDER_VOLTAGE,
        FAULT_CODE_DRV,
        FAULT_CODE_ABS_OVER_CURRENT,
        FAULT_CODE_OVER_TEMP_FET,
        FAULT_CODE_OVER_TEMP_MOTOR,
        FAULT_CODE_GATE_DRIVER_OVER_VOLTAGE,
        FAULT_CODE_GATE_DRIVER_UNDER_VOLTAGE,
        FAULT_CODE_MCU_UNDER_VOLTAGE,
        FAULT_CODE_BOOTING_FROM_WATCHDOG_RESET,
        FAULT_CODE_ENCODER_SPI,
        FAULT_CODE_ENCODER_SINCOS_BELOW_MIN_AMPLITUDE,
        FAULT_CODE_ENCODER_SINCOS_ABOVE_MAX_AMPLITUDE,
        FAULT_CODE_FLASH_CORRUPTION,
        FAULT_CODE_HIGH_OFFSET_CURRENT_SENSOR_1,
        FAULT_CODE_HIGH_OFFSET_CURRENT_SENSOR_2,
        FAULT_CODE_HIGH_OFFSET_CURRENT_SENSOR_3,
        FAULT_CODE_UNBALANCED_CURRENTS,
        FAULT_CODE_BRK,
        FAULT_CODE_RESOLVER_LOT,
        FAULT_CODE_RESOLVER_DOS,
        FAULT_CODE_RESOLVER_LOS,
        FAULT_CODE_FLASH_CORRUPTION_APP_CFG,
        FAULT_CODE_FLASH_CORRUPTION_MC_CFG,
        FAULT_CODE_ENCODER_NO_MAGNET,
        FAULT_CODE_ENCODER_MAGNET_TOO_STRONG,
        FAULT_CODE_PHASE_FILTER,
        FAULT_CODE_ENCODER_FAULT,
        FAULT_CODE_LV_OUTPUT_FAULT,
    };

public:

    vesc()
    {}

    vesc(pin rx, pin tx)
    :   base(rx, tx)  
    {}

    ~vesc()
    {
        #pragma pack(push, 1)
        struct payload
        {
            u8 cmd = COMM_SHUTDOWN;
        } payload;
        #pragma pack(pop)

        static_assert(sizeof(payload) == sizeof(payload.cmd));

        _send(payload);
    }

    auto info()
    {
        #pragma pack(push, 1)
        struct payload
        {
            u8 cmd = COMM_GET_VALUES;
        } request;
        #pragma pack(pop)
        
        static_assert(sizeof(request) == sizeof(request.cmd));

        _send(request);

        #pragma pack(push, 1)
        struct info
        {
            i16 temp_fet;
            i16 temp_motor;
            i32 avg_motor_current;
            i32 avg_input_current;
            i32 avg_id;
            i32 avg_iq;
            i16 duty_cycle_now;
            i32 rpm;
            i16 v_in;
            i32 amp_hours;
            i32 amp_hours_charged;
            i32 watt_hours;
            i32 watt_hours_charged;
            i32 tachometer;
            i32 tachometer_abs;
            i8  mc_fault_code;
            i32 pid_pos_now;
            i8  app_controller_id;
            i32 time_ms;
        } info;
        #pragma pack(pop)

        _recv(info);

        if (info.mc_fault_code != FAULT_CODE_NONE)
            ESP_LOGE("vesc", "fault: %d", info.mc_fault_code);

        return info;
    }

    void throttle(float factor)
    {
        if (factor > 0)
            accelerate(factor);
        else
            brake(-factor);
    }

    void accelerate(float factor)
    {
        assert(factor >= 0);

        #pragma pack(push, 1)
        struct payload
        {
            u8 cmd = COMM_SET_DUTY;
        } command;
        #pragma pack(pop)

        _send(command);
    }

    void brake(float factor)
    {
        assert(factor >= 0);
    }

protected:

    template <typename T>
    void _send(const T& payload)
    {
        #pragma pack(push, 1)
        struct packet
        {
            u8 header = vesc_header;
            u8 len;
            T  payload;
            u16 crc;
            u8 footer = vesc_footer;
        } packet;
        #pragma pack(pop)

        packet.len = sizeof(payload);
        packet.payload = payload;
        packet.crc = crc16(payload);

        write(&packet, sizeof(packet));
    }

    template <typename T>
    void _recv(T& payload)
    {
        u8 header[2] = {};
        read(header, sizeof(header));
        if (header[0] != vesc_header)
            throw std::runtime_error("invalid vesc header");

        size_t len = header[1];
        if (len != sizeof(T))
            throw std::runtime_error("data length mismatch");

        read(&payload, sizeof(payload));

        u16 crc;
        read(&crc, sizeof(crc));
        if (crc16(payload) != crc)
            throw std::runtime_error("crc mismatch");

        u8 footer;
        read(&footer, sizeof(footer));
        if (header[0] != vesc_header)
            throw std::runtime_error("invalid vesc footer");
    }

    template <typename T>
    static T _reverse(const T& data)
    {
        T copy = data;
        _reverse(&copy, sizeof(copy));
        return copy;
    }

    static void _reverse(void* data, size_t size)
    {
        auto cast = reinterpret_cast<u8*>(data);
        for (size_t i = 0; i < size / 2; ++i)
            std::swap(cast[i], cast[size - i - 1]);
    }

    template <typename T>
    static u16 crc16(const T& data)
    {
        return crc16(&data, sizeof(data));
    }

    static u16 crc16(const void* data, size_t size)
    {
        static constexpr u16 crc16_table[256] = {
            0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50A5, 0x60C6, 0x70E7,
            0x8108, 0x9129, 0xA14A, 0xB16B, 0xC18C, 0xD1AD, 0xE1CE, 0xF1EF,
            0x1231, 0x0210, 0x3273, 0x2252, 0x52B5, 0x4294, 0x72F7, 0x62D6,
            0x9339, 0x8318, 0xB37B, 0xA35A, 0xD3BD, 0xC39C, 0xF3FF, 0xE3DE,
            0x2462, 0x3443, 0x0420, 0x1401, 0x64E6, 0x74C7, 0x44A4, 0x5485,
            0xA56A, 0xB54B, 0x8528, 0x9509, 0xE5EE, 0xF5CF, 0xC5AC, 0xD58D,
            0x3653, 0x2672, 0x1611, 0x0630, 0x76D7, 0x66F6, 0x5695, 0x46B4,
            0xB75B, 0xA77A, 0x9719, 0x8738, 0xF7DF, 0xE7FE, 0xD79D, 0xC7BC,
            0x48C4, 0x58E5, 0x6886, 0x78A7, 0x0840, 0x1861, 0x2802, 0x3823,
            0xC9CC, 0xD9ED, 0xE98E, 0xF9AF, 0x8948, 0x9969, 0xA90A, 0xB92B,
            0x5AF5, 0x4AD4, 0x7AB7, 0x6A96, 0x1A71, 0x0A50, 0x3A33, 0x2A12,
            0xDBFD, 0xCBDC, 0xFBBF, 0xEB9E, 0x9B79, 0x8B58, 0xBB3B, 0xAB1A,
            0x6CA6, 0x7C87, 0x4CE4, 0x5CC5, 0x2C22, 0x3C03, 0x0C60, 0x1C41,
            0xEDAE, 0xFD8F, 0xCDEC, 0xDDCD, 0xAD2A, 0xBD0B, 0x8D68, 0x9D49,
            0x7E97, 0x6EB6, 0x5ED5, 0x4EF4, 0x3E13, 0x2E32, 0x1E51, 0x0E70,
            0xFF9F, 0xEFBE, 0xDFDD, 0xCFFC, 0xBF1B, 0xAF3A, 0x9F59, 0x8F78,
            0x9188, 0x81A9, 0xB1CA, 0xA1EB, 0xD10C, 0xC12D, 0xF14E, 0xE16F,
            0x1080, 0x00A1, 0x30C2, 0x20E3, 0x5004, 0x4025, 0x7046, 0x6067,
            0x83B9, 0x9398, 0xA3FB, 0xB3DA, 0xC33D, 0xD31C, 0xE37F, 0xF35E,
            0x02B1, 0x1290, 0x22F3, 0x32D2, 0x4235, 0x5214, 0x6277, 0x7256,
            0xB5EA, 0xA5CB, 0x95A8, 0x8589, 0xF56E, 0xE54F, 0xD52C, 0xC50D,
            0x34E2, 0x24C3, 0x14A0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405,
            0xA7DB, 0xB7FA, 0x8799, 0x97B8, 0xE75F, 0xF77E, 0xC71D, 0xD73C,
            0x26D3, 0x36F2, 0x0691, 0x16B0, 0x6657, 0x7676, 0x4615, 0x5634,
            0xD94C, 0xC96D, 0xF90E, 0xE92F, 0x99C8, 0x89E9, 0xB98A, 0xA9AB,
            0x5844, 0x4865, 0x7806, 0x6827, 0x18C0, 0x08E1, 0x3882, 0x28A3,
            0xCB7D, 0xDB5C, 0xEB3F, 0xFB1E, 0x8BF9, 0x9BD8, 0xABBB, 0xBB9A,
            0x4A75, 0x5A54, 0x6A37, 0x7A16, 0x0AF1, 0x1AD0, 0x2AB3, 0x3A92,
            0xFD2E, 0xED0F, 0xDD6C, 0xCD4D, 0xBDAA, 0xAD8B, 0x9DE8, 0x8DC9,
            0x7C26, 0x6C07, 0x5C64, 0x4C45, 0x3CA2, 0x2C83, 0x1CE0, 0x0CC1,
            0xEF1F, 0xFF3E, 0xCF5D, 0xDF7C, 0xAF9B, 0xBFBA, 0x8FD9, 0x9FF8,
            0x6E17, 0x7E36, 0x4E55, 0x5E74, 0x2E93, 0x3EB2, 0x0ED1, 0x1EF0
        };

        u16 crc = 0xFFFF;
        for (size_t i = 0; i < size; i++) {
            crc = (crc << 8) ^ crc16_table[((crc >> 8) ^ reinterpret_cast<const u8*>(data)[i]) & 0xFF];
        }
        return crc;
    }

};
    
} // namespace eventide
