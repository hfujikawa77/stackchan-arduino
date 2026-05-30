#ifndef HERMESLLM_SI12T_H_
#define HERMESLLM_SI12T_H_

#include <Arduino.h>
#include <M5Unified.h>

#define SI12T_GND_ADDRESS 0x68

#define SI12T_SENSITIVITY1_ADDR 0x02
#define SI12T_SENSITIVITY2_ADDR 0x03
#define SI12T_SENSITIVITY3_ADDR 0x04
#define SI12T_SENSITIVITY4_ADDR 0x05
#define SI12T_SENSITIVITY5_ADDR 0x06
#define SI12T_CTRL1_ADDR        0x08
#define SI12T_CTRL2_ADDR        0x09
#define SI12T_REF_RST1_ADDR     0x0A
#define SI12T_REF_RST2_ADDR     0x0B
#define SI12T_CH_HOLD1_ADDR     0x0C
#define SI12T_CH_HOLD2_ADDR     0x0D
#define SI12T_CAL_HOLD1_ADDR    0x0E
#define SI12T_CAL_HOLD2_ADDR    0x0F
#define SI12T_OUTPUT1_ADDR      0x10

enum SI12T_Type { SI12T_Type_Low, SI12T_Type_High };

enum SI12T_Sensitivity_Level {
    SI12T_Sensitivity_Level_0,
    SI12T_Sensitivity_Level_1,
    SI12T_Sensitivity_Level_2,
    SI12T_Sensitivity_Level_3,
    SI12T_Sensitivity_Level_4,
    SI12T_Sensitivity_Level_5,
    SI12T_Sensitivity_Level_6,
    SI12T_Sensitivity_Level_7,
};

enum output_e { OUTPUT_NONE, OUTPUT_LOW, OUTPUT_MID, OUTPUT_HIGH };

class Si12T : public m5::I2C_Device {
private:
    bool write_reg(std::uint8_t reg_addr, std::uint8_t value);
    bool read_reg(std::uint8_t reg_addr, std::uint8_t* value);

public:
    Si12T(uint8_t sens_type, uint8_t sens_level, m5::I2C_Class* i2c = &m5::In_I2C);
    void begin();
    void read_touch_result();
    void parse_touch_result();

    uint8_t sens_type;
    uint8_t sens_level;
    uint8_t touch_result;
    uint8_t point_type[3];

private:
    void enable_channel();
    int8_t set_sensitivity(uint8_t sens_type, uint8_t sens_level);
    uint8_t set_sens(uint8_t value);
    void set_ctrl1();
    void set_ctrl2();
};

#endif
