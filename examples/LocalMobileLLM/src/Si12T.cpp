#include "Si12T.h"

static uint32_t i2c_freq = 100000;

Si12T::Si12T(uint8_t sens_type, uint8_t sens_level, m5::I2C_Class* i2c)
    : m5::I2C_Device(SI12T_GND_ADDRESS, i2c_freq, i2c),
      sens_type(sens_type),
      sens_level(sens_level),
      touch_result(0) {
    point_type[0] = OUTPUT_NONE;
    point_type[1] = OUTPUT_NONE;
    point_type[2] = OUTPUT_NONE;
}

void Si12T::begin() {
    enable_channel();
    set_ctrl2();
    set_ctrl1();
    set_sensitivity(sens_type, sens_level);
}

bool Si12T::write_reg(std::uint8_t reg_addr, std::uint8_t value) {
    if (!_i2c) return false;
    return writeRegister8(reg_addr, value);
}

bool Si12T::read_reg(std::uint8_t reg_addr, std::uint8_t* value) {
    if (!_i2c) return false;
    return readRegister(reg_addr, value, 1);
}

uint8_t Si12T::set_sens(uint8_t value) {
    write_reg(SI12T_SENSITIVITY1_ADDR, value);
    write_reg(SI12T_SENSITIVITY2_ADDR, value);
    write_reg(SI12T_SENSITIVITY3_ADDR, value);
    write_reg(SI12T_SENSITIVITY4_ADDR, value);
    write_reg(SI12T_SENSITIVITY5_ADDR, value);
    return 0;
}

int8_t Si12T::set_sensitivity(uint8_t sens_type, uint8_t sens_level) {
    uint8_t value = 0x00;
    if (sens_type == SI12T_Type_High) {
        value = 0x88 + (sens_level * 0x11);
    } else {
        value = sens_level * 0x11;
    }
    set_sens(value);
    return 0;
}

void Si12T::set_ctrl1() {
    write_reg(SI12T_CTRL1_ADDR, 0x22);
}

void Si12T::set_ctrl2() {
    write_reg(SI12T_CTRL2_ADDR, 0x0F);
    write_reg(SI12T_CTRL2_ADDR, 0x07);
}

void Si12T::enable_channel() {
    write_reg(SI12T_REF_RST1_ADDR, 0x00);
    write_reg(SI12T_REF_RST2_ADDR, 0x00);
    write_reg(SI12T_CH_HOLD1_ADDR, 0x00);
    write_reg(SI12T_CH_HOLD2_ADDR, 0x00);
    write_reg(SI12T_CAL_HOLD1_ADDR, 0x00);
    write_reg(SI12T_CAL_HOLD2_ADDR, 0x00);
}

void Si12T::read_touch_result() {
    readRegister(SI12T_OUTPUT1_ADDR, &touch_result, 1);
}

void Si12T::parse_touch_result() {
    int index = 0;
    memset(point_type, 0, sizeof(point_type));
    for (int j = 0; j < 6; j += 2) {
        point_type[index++] = (touch_result >> j) & 0x03;
    }
}
