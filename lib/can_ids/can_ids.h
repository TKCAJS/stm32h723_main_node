/**
 * can_ids.h
 * Shared CAN bus identifier and payload definitions
 * Honda CBR1000RR Racing Telemetry & Control System
 *
 * Include this file in ALL nodes. Do NOT modify assigned IDs once a
 * node is in use — only extend. Never use raw CAN_ID() calls in
 * firmware; always use the named CAN_x defines below.
 *
 * ID Structure (29-bit extended):
 *  [28:26] Priority     3 bits  (0=highest, 7=lowest)
 *  [25:20] Node ID      6 bits  (0-63, up to 64 nodes)
 *  [19:12] Message Type 8 bits  (0-255 per node)
 *  [11:0]  Sub-index   12 bits  (reserved / future use)
 *
 * Baud rate: 500 Kbps, 29-bit extended frame format throughout.
 */

#ifndef CAN_IDS_H
#define CAN_IDS_H

#include <stdint.h>

// =============================================================================
// ID CONSTRUCTION MACRO
// =============================================================================

#define CAN_ID(priority, node, msgtype, sub) \
    ( (((uint32_t)(priority) & 0x07) << 26) | \
      (((uint32_t)(node)     & 0x3F) << 20) | \
      (((uint32_t)(msgtype)  & 0xFF) << 12) | \
      (((uint32_t)(sub)      & 0xFFF)      ) )

// Extraction macros (for logging / debug)
#define CAN_ID_PRIORITY(id)  (((id) >> 26) & 0x07)
#define CAN_ID_NODE(id)      (((id) >> 20) & 0x3F)
#define CAN_ID_MSGTYPE(id)   (((id) >> 12) & 0xFF)
#define CAN_ID_SUB(id)       (((id)       ) & 0xFFF)


// =============================================================================
// PRIORITY LEVELS
// =============================================================================

#define CAN_PRIO_CRITICAL   0   // Safety-critical: engine protection, brake faults
#define CAN_PRIO_HIGH       1   // Fast control: shift commands, gear position
#define CAN_PRIO_MEDIUM     2   // Sensor data: temps, pressures
#define CAN_PRIO_LOW        3   // Telemetry: GPS, accelerometer
#define CAN_PRIO_INFO       4   // Status, heartbeat
#define CAN_PRIO_DEBUG      7   // Debug messages, bench only


// =============================================================================
// NODE IDs
// =============================================================================

#define NODE_MAIN           0x01    // ESP32-S3   - main controller / SD logger
#define NODE_DISPLAY        0x02    // ESP32-S3   - 8048S043C 800x480 LVGL display
#define NODE_REAR           0x03    // ESP32-S3   - rear node: gear shift + position
// 0x04 retired — cooling folded into NODE_SENSOR (PWM pump control lives there)
#define NODE_GPS            0x05    // (future)   - GPS
#define NODE_IMU            0x06    // (future)   - accelerometer / gyro
#define NODE_LAP            0x07    // (future)   - lap trigger
#define NODE_OIL            0x08    // (future)   - oil temp + pressure
#define NODE_BRAKE          0x09    // (future)   - brake pressure front/rear
// 0x0A-0x3F reserved for future nodes
#define NODE_SENSOR         0x0A    // STM32H562 - analog sensor hub + PWM pump control


// =============================================================================
// MESSAGE TYPES — common to all nodes
// =============================================================================

#define MSGTYPE_HEARTBEAT           0x00    // Node alive broadcast (1 Hz)


// =============================================================================
// MESSAGE TYPES — NODE_MAIN (0x01)
// =============================================================================

#define MSGTYPE_MAIN_STATUS         0x01    // System status flags
#define MSGTYPE_MAIN_LAP            0x02    // Lap count + lap time
#define MSGTYPE_MAIN_SD_STATUS      0x03    // SD card / logging status
#define MSGTYPE_MAIN_RPM            0x04    // RPM broadcast (~20ms)        data[2:3]=RPM uint16_t LE
#define MSGTYPE_MAIN_SHIFT_STATUS   0x05    // Shift mode broadcast (1Hz)   data[2]=mode (0=Auto,1=Manual)
#define MSGTYPE_MAIN_SHIFT_STACK    0x06    // Stacked downshift target      data[2]=targetGear (0=none, 1-6=target)
// 0x40-0x5F commands TO main node
#define MSGTYPE_MAIN_CFG_REQ        0x40    // Request config broadcast


// =============================================================================
// MESSAGE TYPES — NODE_DISPLAY (0x02)
// =============================================================================

#define MSGTYPE_DISP_STATUS         0x01    // Display node status
// 0x40-0x5F commands TO display node
#define MSGTYPE_DISP_INPUT          0x40    // Button / input event from display


// =============================================================================
// MESSAGE TYPES — NODE_REAR (0x03)
// =============================================================================

// Sensor data (outbound from rear node)
#define MSGTYPE_REAR_GEAR_POS       0x01    // Current gear position (filtered)
#define MSGTYPE_REAR_GEAR_RAW       0x02    // Raw ADC gear position (debug)
#define MSGTYPE_REAR_SHIFT_EVENT    0x03    // Shift completed event + result
#define MSGTYPE_REAR_STATUS         0x04    // Rear node status / health

// Commands TO rear node (0x40-0x5F)
#define MSGTYPE_REAR_CMD_SHIFT_UP   0x40    // Command: execute upshift
#define MSGTYPE_REAR_CMD_SHIFT_DN   0x41    // Command: execute downshift

// Acknowledgements FROM rear node (0x60-0x7F)
#define MSGTYPE_REAR_ACK_SHIFT      0x60    // Shift command received, solenoid firing
#define MSGTYPE_REAR_ACK_COMPLETE   0x61    // Shift confirmed complete (position verified)


// =============================================================================
// MESSAGE TYPES — NODE_SENSOR (0x0A)
// =============================================================================

// Sensor data (outbound from sensor node)
#define MSGTYPE_SENS_OIL_PRESSURE   0x01    // data[2:3] = int16_t  kPa×10
#define MSGTYPE_SENS_ENGINE_TEMP    0x02    // data[2:3] = int16_t  °C×10 (engine NTC)
#define MSGTYPE_SENS_TPS            0x03    // data[2:3] = uint16_t 0–10000 (0.01 % resolution)
#define MSGTYPE_SENS_SPEED          0x04    // data[2:3] = uint16_t km/h×10 (from square-wave input)
#define MSGTYPE_SENS_FUEL_1         0x05    // data[2:3] = uint16_t 0–10000 (0.01 % resolution)
#define MSGTYPE_SENS_FUEL_2         0x06    // data[2:3] = uint16_t 0–10000 (0.01 % resolution)
#define MSGTYPE_SENS_PUMP_STATUS    0x07    // data[2]   = uint8_t  0–100 % PWM duty (outbound status)
#define MSGTYPE_SENS_STATUS         0x08    // node health flags
#define MSGTYPE_SENS_RADIATOR_TEMP  0x09    // data[2:3] = int16_t  °C×10 (Dallas DS18B20)
#define MSGTYPE_SENS_RAD_OUT_TEMP   0x0A    // data[2:3] = int16_t  °C×10 (radiator outlet NTC)

// Commands TO sensor node (0x40-0x5F)
#define MSGTYPE_SENS_CMD_PUMP       0x40    // data[2] = uint8_t 0–100 % PWM duty override
#define MSGTYPE_SENS_CMD_TARGET     0x41    // data[2] = uint8_t target engine temp °C (pump curve center)


// =============================================================================
// MESSAGE TYPES — NODE_GPS (0x05, sent by the STM32H723 GPS node)
// =============================================================================

#define MSGTYPE_GPS_POS             0x01    // data[2] = 0:lat 1:lon, data[3:6] = int32 deg×1e7 LE.
                                            // Only sent while the fix is valid.
#define MSGTYPE_GPS_SPEED           0x02    // data[2:3] = uint16 mph×10 LE (0xFFFF = no fix),
                                            // data[4] = satellites, data[5] = bit0 loc_valid bit1 speed_valid
#define MSGTYPE_GPS_STATUS          0x03


// =============================================================================
// MESSAGE TYPES — NODE_IMU (0x06, sent by the STM32H723 GPS node's ICM20948)
// =============================================================================

#define MSGTYPE_IMU_ACCEL           0x01    // int16 mg LE ×3: data[2:3] fwd(+)/brake(-),
                                            // data[4:5] lateral(right+), data[6:7] vertical
#define MSGTYPE_IMU_GYRO            0x02
#define MSGTYPE_IMU_STATUS          0x03


// =============================================================================
// MESSAGE TYPES — NODE_LAP (0x07, future)
// =============================================================================

#define MSGTYPE_LAP_TRIGGER         0x01
#define MSGTYPE_LAP_TIME            0x02
#define MSGTYPE_LAP_STATUS          0x03


// =============================================================================
// MESSAGE TYPES — NODE_OIL (0x08, future)
// =============================================================================

#define MSGTYPE_OIL_TEMP            0x01
#define MSGTYPE_OIL_PRESSURE        0x02
#define MSGTYPE_OIL_STATUS          0x03


// =============================================================================
// MESSAGE TYPES — NODE_BRAKE (0x09, future)
// =============================================================================

#define MSGTYPE_BRAKE_FRONT         0x01
#define MSGTYPE_BRAKE_REAR          0x02
#define MSGTYPE_BRAKE_STATUS        0x03


// =============================================================================
// PRE-BUILT FULL IDs
// Every MSGTYPE_x above has exactly one CAN_x assembled ID here.
// These are the ONLY values that should appear in node firmware.
// =============================================================================

// --- HEARTBEATS (all nodes) ---
#define CAN_HB_MAIN                 CAN_ID(CAN_PRIO_INFO,     NODE_MAIN,    MSGTYPE_HEARTBEAT,          0)
#define CAN_HB_DISPLAY              CAN_ID(CAN_PRIO_INFO,     NODE_DISPLAY, MSGTYPE_HEARTBEAT,          0)
#define CAN_HB_REAR                 CAN_ID(CAN_PRIO_INFO,     NODE_REAR,    MSGTYPE_HEARTBEAT,          0)
#define CAN_HB_GPS                  CAN_ID(CAN_PRIO_INFO,     NODE_GPS,     MSGTYPE_HEARTBEAT,          0)
#define CAN_HB_IMU                  CAN_ID(CAN_PRIO_INFO,     NODE_IMU,     MSGTYPE_HEARTBEAT,          0)
#define CAN_HB_LAP                  CAN_ID(CAN_PRIO_INFO,     NODE_LAP,     MSGTYPE_HEARTBEAT,          0)
#define CAN_HB_OIL                  CAN_ID(CAN_PRIO_INFO,     NODE_OIL,     MSGTYPE_HEARTBEAT,          0)
#define CAN_HB_BRAKE                CAN_ID(CAN_PRIO_INFO,     NODE_BRAKE,   MSGTYPE_HEARTBEAT,          0)
#define CAN_HB_SENSOR               CAN_ID(CAN_PRIO_INFO,     NODE_SENSOR,  MSGTYPE_HEARTBEAT,          0)

// --- MAIN NODE ---
#define CAN_MAIN_STATUS             CAN_ID(CAN_PRIO_INFO,     NODE_MAIN,    MSGTYPE_MAIN_STATUS,        0)
#define CAN_MAIN_LAP                CAN_ID(CAN_PRIO_HIGH,     NODE_MAIN,    MSGTYPE_MAIN_LAP,           0)
#define CAN_MAIN_SD_STATUS          CAN_ID(CAN_PRIO_INFO,     NODE_MAIN,    MSGTYPE_MAIN_SD_STATUS,     0)
#define CAN_MAIN_RPM                CAN_ID(CAN_PRIO_HIGH,     NODE_MAIN,    MSGTYPE_MAIN_RPM,           0)
#define CAN_MAIN_SHIFT_STATUS       CAN_ID(CAN_PRIO_INFO,     NODE_MAIN,    MSGTYPE_MAIN_SHIFT_STATUS,  0)
#define CAN_MAIN_SHIFT_STACK        CAN_ID(CAN_PRIO_HIGH,     NODE_MAIN,    MSGTYPE_MAIN_SHIFT_STACK,   0)
#define CAN_MAIN_CFG_REQ            CAN_ID(CAN_PRIO_LOW,      NODE_MAIN,    MSGTYPE_MAIN_CFG_REQ,       0)

// --- DISPLAY NODE ---
#define CAN_DISP_STATUS             CAN_ID(CAN_PRIO_INFO,     NODE_DISPLAY, MSGTYPE_DISP_STATUS,        0)
#define CAN_DISP_INPUT              CAN_ID(CAN_PRIO_HIGH,     NODE_DISPLAY, MSGTYPE_DISP_INPUT,         0)

// --- REAR NODE ---
#define CAN_REAR_GEAR_POS           CAN_ID(CAN_PRIO_HIGH,     NODE_REAR,    MSGTYPE_REAR_GEAR_POS,      0)
#define CAN_REAR_GEAR_RAW           CAN_ID(CAN_PRIO_DEBUG,    NODE_REAR,    MSGTYPE_REAR_GEAR_RAW,      0)
#define CAN_REAR_SHIFT_EVENT        CAN_ID(CAN_PRIO_HIGH,     NODE_REAR,    MSGTYPE_REAR_SHIFT_EVENT,   0)
#define CAN_REAR_STATUS             CAN_ID(CAN_PRIO_INFO,     NODE_REAR,    MSGTYPE_REAR_STATUS,        0)
#define CAN_REAR_CMD_SHIFT_UP       CAN_ID(CAN_PRIO_CRITICAL, NODE_REAR,    MSGTYPE_REAR_CMD_SHIFT_UP,  0)
#define CAN_REAR_CMD_SHIFT_DN       CAN_ID(CAN_PRIO_CRITICAL, NODE_REAR,    MSGTYPE_REAR_CMD_SHIFT_DN,  0)
#define CAN_REAR_ACK_SHIFT          CAN_ID(CAN_PRIO_HIGH,     NODE_REAR,    MSGTYPE_REAR_ACK_SHIFT,     0)
#define CAN_REAR_ACK_COMPLETE       CAN_ID(CAN_PRIO_HIGH,     NODE_REAR,    MSGTYPE_REAR_ACK_COMPLETE,  0)

// --- SENSOR NODE ---
#define CAN_SENS_OIL_PRESSURE       CAN_ID(CAN_PRIO_MEDIUM,   NODE_SENSOR,  MSGTYPE_SENS_OIL_PRESSURE,  0)
#define CAN_SENS_ENGINE_TEMP        CAN_ID(CAN_PRIO_MEDIUM,   NODE_SENSOR,  MSGTYPE_SENS_ENGINE_TEMP,   0)
#define CAN_SENS_TPS                CAN_ID(CAN_PRIO_HIGH,     NODE_SENSOR,  MSGTYPE_SENS_TPS,           0)
#define CAN_SENS_SPEED              CAN_ID(CAN_PRIO_HIGH,     NODE_SENSOR,  MSGTYPE_SENS_SPEED,         0)
#define CAN_SENS_FUEL_1             CAN_ID(CAN_PRIO_LOW,      NODE_SENSOR,  MSGTYPE_SENS_FUEL_1,        0)
#define CAN_SENS_FUEL_2             CAN_ID(CAN_PRIO_LOW,      NODE_SENSOR,  MSGTYPE_SENS_FUEL_2,        0)
#define CAN_SENS_PUMP_STATUS        CAN_ID(CAN_PRIO_INFO,     NODE_SENSOR,  MSGTYPE_SENS_PUMP_STATUS,   0)
#define CAN_SENS_STATUS             CAN_ID(CAN_PRIO_INFO,     NODE_SENSOR,  MSGTYPE_SENS_STATUS,        0)
#define CAN_SENS_RADIATOR_TEMP      CAN_ID(CAN_PRIO_MEDIUM,   NODE_SENSOR,  MSGTYPE_SENS_RADIATOR_TEMP, 0)
#define CAN_SENS_RAD_OUT_TEMP       CAN_ID(CAN_PRIO_MEDIUM,   NODE_SENSOR,  MSGTYPE_SENS_RAD_OUT_TEMP,  0)
#define CAN_SENS_CMD_PUMP           CAN_ID(CAN_PRIO_MEDIUM,   NODE_SENSOR,  MSGTYPE_SENS_CMD_PUMP,      0)
#define CAN_SENS_CMD_TARGET_TEMP    CAN_ID(CAN_PRIO_MEDIUM,   NODE_SENSOR,  MSGTYPE_SENS_CMD_TARGET,    0)

// --- GPS NODE (future) ---
#define CAN_GPS_POS                 CAN_ID(CAN_PRIO_LOW,      NODE_GPS,     MSGTYPE_GPS_POS,            0)
#define CAN_GPS_SPEED               CAN_ID(CAN_PRIO_LOW,      NODE_GPS,     MSGTYPE_GPS_SPEED,          0)
#define CAN_GPS_STATUS              CAN_ID(CAN_PRIO_INFO,     NODE_GPS,     MSGTYPE_GPS_STATUS,         0)

// --- IMU NODE (future) ---
#define CAN_IMU_ACCEL               CAN_ID(CAN_PRIO_LOW,      NODE_IMU,     MSGTYPE_IMU_ACCEL,          0)
#define CAN_IMU_GYRO                CAN_ID(CAN_PRIO_LOW,      NODE_IMU,     MSGTYPE_IMU_GYRO,           0)
#define CAN_IMU_STATUS              CAN_ID(CAN_PRIO_INFO,     NODE_IMU,     MSGTYPE_IMU_STATUS,         0)

// --- LAP NODE (future) ---
#define CAN_LAP_TRIGGER             CAN_ID(CAN_PRIO_HIGH,     NODE_LAP,     MSGTYPE_LAP_TRIGGER,        0)
#define CAN_LAP_TIME                CAN_ID(CAN_PRIO_HIGH,     NODE_LAP,     MSGTYPE_LAP_TIME,           0)
#define CAN_LAP_STATUS              CAN_ID(CAN_PRIO_INFO,     NODE_LAP,     MSGTYPE_LAP_STATUS,         0)

// --- OIL NODE (future) ---
#define CAN_OIL_TEMP                CAN_ID(CAN_PRIO_MEDIUM,   NODE_OIL,     MSGTYPE_OIL_TEMP,           0)
#define CAN_OIL_PRESSURE            CAN_ID(CAN_PRIO_MEDIUM,   NODE_OIL,     MSGTYPE_OIL_PRESSURE,       0)
#define CAN_OIL_STATUS              CAN_ID(CAN_PRIO_INFO,     NODE_OIL,     MSGTYPE_OIL_STATUS,         0)

// --- BRAKE NODE (future) ---
#define CAN_BRAKE_FRONT             CAN_ID(CAN_PRIO_HIGH,     NODE_BRAKE,   MSGTYPE_BRAKE_FRONT,        0)
#define CAN_BRAKE_REAR              CAN_ID(CAN_PRIO_HIGH,     NODE_BRAKE,   MSGTYPE_BRAKE_REAR,         0)
#define CAN_BRAKE_STATUS            CAN_ID(CAN_PRIO_INFO,     NODE_BRAKE,   MSGTYPE_BRAKE_STATUS,       0)


// =============================================================================
// PAYLOAD CONVENTIONS
// =============================================================================
//
// All frames: 8 bytes
//
// Byte 0:     Sequence counter (0-255, wraps) — detect dropped packets
// Byte 1:     Node status flags (see NODE_STATUS_x below; 0x00 = all OK)
// Bytes 2-7:  Data payload (message-specific, defined per message type)
//
// Fixed-point encoding:
//   Temperatures : int16_t, °C × 10       (e.g. 856 = 85.6 °C)
//   Pressures    : int16_t, kPa × 10
//   Speeds       : uint16_t, km/h × 10
//   Duty cycle   : uint8_t, 0–100 (%)
//   Gear         : uint8_t, 0=Neutral, 1-6=gear, 0xFE=between, 0xFF=unknown
//
// All multi-byte values: LITTLE ENDIAN


// =============================================================================
// NODE STATUS FLAG BITS (Byte 1 of every frame)
// =============================================================================

#define NODE_STATUS_OK              0x00
#define NODE_STATUS_SENSOR_ERR      (1 << 0)    // Sensor read failure
#define NODE_STATUS_ACTUATOR_ERR    (1 << 1)    // Actuator / output fault
#define NODE_STATUS_CAN_ERR         (1 << 2)    // CAN bus error
#define NODE_STATUS_OVERTEMP        (1 << 3)    // Node overtemperature
#define NODE_STATUS_WATCHDOG        (1 << 4)    // Watchdog reset occurred
#define NODE_STATUS_CONFIG_ERR      (1 << 5)    // Configuration invalid
// bits 6-7 reserved


// =============================================================================
// GEAR POSITION SPECIAL VALUES
// =============================================================================

#define GEAR_NEUTRAL                0x00
#define GEAR_1                      0x01
#define GEAR_2                      0x02
#define GEAR_3                      0x03
#define GEAR_4                      0x04
#define GEAR_5                      0x05
#define GEAR_6                      0x06
#define GEAR_BETWEEN                0xFE    // ADC between detents (mid-shift)
#define GEAR_UNKNOWN                0xFF    // Sensor fault or not yet read


// =============================================================================
// BUS CONFIGURATION
// =============================================================================

#define CAN_BAUD_RATE               500000  // 500 Kbps
#define CAN_HEARTBEAT_INTERVAL_MS   1000    // All nodes: heartbeat every 1s
#define CAN_NODE_TIMEOUT_MS         3000    // Main: flag node dead after 3s silence


#endif // CAN_IDS_H
