#ifndef I2CDefs_h
#define I2CDefs_h

// I2C Slave Interface definition
#define I2C_SLAVE_ADDR                0x69
#define I2C_TIME_UPDATE               0x00
#define I2C_GET_OPTIONS               0x01
#define I2C_SET_OPTION_12_24          0x02
#define I2C_SET_OPTION_BLANK_LEAD     0x03
#define I2C_SET_OPTION_SCROLLBACK     0x04
#define I2C_SET_OPTION_SUPPRESS_ACP   0x05
#define I2C_SET_OPTION_DATE_FORMAT    0x06
#define I2C_SET_OPTION_DAY_BLANKING   0x07
#define I2C_SET_OPTION_BLANK_START    0x08
#define I2C_SET_OPTION_BLANK_END      0x09
#define I2C_SET_OPTION_FADE_STEPS     0x0a
#define I2C_SET_OPTION_SCROLL_STEPS   0x0b
#define I2C_SET_OPTION_BACKLIGHT_MODE 0x0c
#define I2C_SET_OPTION_RED_CHANNEL    0x0d
#define I2C_SET_OPTION_GREEN_CHANNEL  0x0e
#define I2C_SET_OPTION_BLUE_CHANNEL   0x0f
#define I2C_SET_OPTION_CYCLE_SPEED    0x10
#define I2C_SHOW_IP_ADDR              0x11
#define I2C_SET_OPTION_FADE           0x12
#define I2C_SET_OPTION_USE_LDR        0x13
#define I2C_SET_OPTION_BLANK_MODE     0x14
#define I2C_SET_OPTION_SLOTS_MODE     0x15
#define I2C_SET_OPTION_PIR_TIMEOUT    0x16
#define I2C_SHOW_VALUE                0x17
#define I2C_SHOW_VALUE_FORMAT         0x18
#define I2C_SET_OPTION_MIN_DIM_V2     0x19
#define I2C_SET_OPTION_MIN_DIM_V1     0x16

#define I2C_DATA_SIZE_V1               22
#define I2C_PROTOCOL_NUMBER_V1         54
#define I2C_DATA_SIZE_V2               24
#define I2C_PROTOCOL_NUMBER_V2         62

#endif
