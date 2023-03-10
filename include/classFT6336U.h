#pragma once
#include <Arduino.h>

#define I2C_ADDR_FT6336U 0x38

// Registers
#define FT6336U_ADDR_DEVICE_MODE 0x00

#define FT6336U_ADDR_GESTURE_ID 0x01
#define FT6336U_ADDR_TD_STATUS 0x02

#define FT6336U_ADDR_TOUCH1_EVENT 0x03
#define FT6336U_ADDR_TOUCH1_ID 0x05
#define FT6336U_ADDR_TOUCH1_X 0x03
#define FT6336U_ADDR_TOUCH1_Y 0x05
#define FT6336U_ADDR_TOUCH1_WEIGHT 0x07
#define FT6336U_ADDR_TOUCH1_MISC 0x08

#define FT6336U_ADDR_TOUCH2_EVENT 0x09
#define FT6336U_ADDR_TOUCH2_ID 0x0B
#define FT6336U_ADDR_TOUCH2_X 0x09
#define FT6336U_ADDR_TOUCH2_Y 0x0B
#define FT6336U_ADDR_TOUCH2_WEIGHT 0x0D
#define FT6336U_ADDR_TOUCH2_MISC 0x0E

#define FT6336U_ADDR_THRESHOLD 0x80
#define FT6336U_ADDR_FILTER_COE 0x85
#define FT6336U_ADDR_CTRL 0x86

#define FT6336U_ADDR_TIME_ENTER_MONITOR 0x87
#define FT6336U_ADDR_ACTIVE_MODE_RATE 0x88
#define FT6336U_ADDR_MONITOR_MODE_RATE 0x89

#define FT6336U_ADDR_RADIAN_VALUE 0x91
#define FT6336U_ADDR_OFFSET_LEFT_RIGHT 0x92
#define FT6336U_ADDR_OFFSET_UP_DOWN 0x93
#define FT6336U_ADDR_DISTANCE_LEFT_RIGHT 0x94
#define FT6336U_ADDR_DISTANCE_UP_DOWN 0x95
#define FT6336U_ADDR_DISTANCE_ZOOM 0x96

#define FT6336U_ADDR_LIBRARY_VERSION_H 0xA1
#define FT6336U_ADDR_LIBRARY_VERSION_L 0xA2
#define FT6336U_ADDR_CHIP_ID 0xA3
#define FT6336U_ADDR_G_MODE 0xA4

#define FT6336U_ADDR_POWER_MODE 0xA5
#define FT6336U_ADDR_FIRMARE_ID 0xA6
#define FT6336U_ADDR_FOCALTECH_ID 0xA8
#define FT6336U_ADDR_RELEASE_CODE_ID 0xAF
#define FT6336U_ADDR_STATE 0xBC

// Function Specific Type
typedef struct
{
  uint16_t x;
  uint16_t y;
} typePoint;

class classFT6336U
{
private:
  int8_t _sdaPin;
  int8_t _sclPin;
  uint8_t _intPin;

public:
  classFT6336U(int8_t sda, int8_t scl, uint8_t int_n);
  void begin(void);
  bool getTouched(void);
  bool readTouchPoint(typePoint *ts);
};
