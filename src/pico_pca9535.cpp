
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "pico/stdlib.h"
#include "hardware/i2c.h"

#include "pca9535.h"

PICO_PCA9535::PICO_PCA9535(i2c_inst_t *i2c, uint8_t address)
{
    /**
     * PICO_PCA9535 object
     * @param i2c - I2C bus instance to use
     * @param address - I2C address for the target device
     */
    invalid_params_if(I2C, address < 0x20 || address > 0x27);
    // if (address<0x20 || address> 0x27) {
    //     throw std::out_of_range("PICO_PCA9535 address out of range: 0x20 to 0x27");
    // }

    i2c_inst = i2c;
    i2c_address = address;
}

void PICO_PCA9535::set_pin_direction(uint8_t pin, uint8_t direction)
{
    /**
     * Set the IO direction for an individual pin
     * @param pins - 1 to 16
     * @param direction - 1 = input, 0 = output
     */
    set_pin(pin, direction, PCA9535::CONFIGPORT0, PCA9535::CONFIGPORT1);
}

uint8_t PICO_PCA9535::get_pin_direction(uint8_t pin)
{
    /**
     * Get the IO direction for an individual pin
     * @param pins - 1 to 16
     */

    return get_pin(pin, PCA9535::CONFIGPORT0, PCA9535::CONFIGPORT1);
}

void PICO_PCA9535::set_port_direction(uint8_t port, uint8_t direction)
{
    /**
     * Set the direction for an IO port
     * @param port - 0 = pins 1 to 8, port 1 = pins 9 to 16
     * @param direction - 0 to 255 (0xFF).  For each bit 1 = input, 0 = output
     */
    set_port(port, direction, PCA9535::CONFIGPORT0, PCA9535::CONFIGPORT1);
}

uint8_t PICO_PCA9535::get_port_direction(uint8_t port)
{
    /**
     * Get the direction for an IO port
     * @param port - 0 = pins 1 to 8, port 1 = pins 9 to 16
     */

    return get_port(port, PCA9535::CONFIGPORT0, PCA9535::CONFIGPORT1);
}

void PICO_PCA9535::set_bus_direction(uint16_t direction)
{
    /**
     * Set the direction for the IO bus
     * @param direction - 0 to 65535 (0xFFFF).  For each bit 1 = input, 0 = output
     */
    write_word_data(PCA9535::CONFIGPORT0, direction);
}

uint16_t PICO_PCA9535::get_bus_direction()
{
    /**
     * Get the direction for the IO bus
     */

    return read_word_data(PCA9535::CONFIGPORT0);
}

void PICO_PCA9535::write_pin(uint8_t pin, uint8_t value)
{
    /**
     * Write to an individual pin 1 - 16
     * @param pin - 1 to 16
     * @param value - 0 = logic low, 1 = logic high
     */
    set_pin(pin, value, PCA9535::OUTPUTPORT0, PCA9535::OUTPUTPORT1);
}

void PICO_PCA9535::write_port(uint8_t port, uint8_t value)
{
    /**
     * Write to all pins on the selected port
     * @param port - 0 = pins 1 to 8, port 1 = pins 9 to 16
     * @param value - 0 to 255 (0xFF)
     */
    set_port(port, value, PCA9535::OUTPUTPORT0, PCA9535::OUTPUTPORT1);
}

void PICO_PCA9535::write_bus(uint16_t value)
{
    /**
     * Write to all pins on the selected bus
     * @param value - 0 to 65535 (0xFFFF). For each bit 1 = logic high, 0 = logic low
     */
    write_word_data(PCA9535::OUTPUTPORT0, value);
}

uint8_t PICO_PCA9535::read_pin(uint8_t pin)
{
    /**
     * Read the value of an individual pin
     * @param pin - 1 to 16
     * @returns - 0 = logic low, 1 = logic high
     */

    return get_pin(pin, PCA9535::INPUTPORT0, PCA9535::INPUTPORT1);
}

uint8_t PICO_PCA9535::read_port(uint8_t port)
{
    /**
     * Read all pins on the selected port
     * @param port - 0 = pins 1 to 8, port 1 = pins 9 to 16
     * @returns - 0 to 255 (0xFF). For each bit 1 = logic high, 0 = logic low
     */
    return get_port(port, PCA9535::INPUTPORT0, PCA9535::INPUTPORT1);
}

uint16_t PICO_PCA9535::read_bus()
{
    /**
     * Read all pins on the selected bus
     * @returns - 0 to 65535 (0xFFFF). For each bit 1 = logic high, 0 = logic low
     */
    return read_word_data(PCA9535::INPUTPORT0);
}

void PICO_PCA9535::set_pin_polarity(uint8_t pin, uint8_t polarity)
{
    /**
     * Set the polarity of the selected pin
     * @param pin - 1 to 16
     * @param polarity - 0 = non-inverted, 1 = inverted
     */
    set_pin(pin, polarity, PCA9535::INVERTPORT0, PCA9535::INVERTPORT1);
}

uint8_t PICO_PCA9535::get_pin_polarity(uint8_t pin)
{
    /**
     * Get the polarity of the selected pin
     * @param pin - 1 to 16
     */
    return get_pin(pin, PCA9535::INVERTPORT0, PCA9535::INVERTPORT1);
}

void PICO_PCA9535::set_port_polarity(uint8_t port, uint8_t polarity)
{
    /**
     * Set the polarity of the pins on a selected port
     * @param port - 0 = pins 1 to 8, port 1 = pins 9 to 16
     * @param polarity - 0 to 255 (0xFF). For each bit 0 = non-inverted, 1 = inverted
     */
    set_port(port, polarity, PCA9535::INVERTPORT0, PCA9535::INVERTPORT1);
}

uint8_t PICO_PCA9535::get_port_polarity(uint8_t port)
{
    /**
     * Get the polarity of the selected pin
     * @param port - 0 = pins 1 to 8, port 1 = pins 9 to 16
     */
    return get_port(port, PCA9535::INVERTPORT0, PCA9535::INVERTPORT1);
}

void PICO_PCA9535::set_bus_polarity(uint16_t polarity)
{
    /**
     * Set the polarity of the pins on a selected bus
     * @param polarity - 0 to 65535 (0xFFFF). For each bit 0 = non-inverted, 1 = inverted
     */
    write_word_data(PCA9535::INVERTPORT0, polarity);
}

uint16_t PICO_PCA9535::get_bus_polarity()
{
    /**
     * Get the polarity of the bus
     */
    return read_word_data(PCA9535::INVERTPORT0);
}

uint8_t PICO_PCA9535::read_byte_data(uint8_t reg)
{
    /**
     * Private method for reading a byte from the I2C port
     */
    buf[0] = reg;

    if (PICO_ERROR_GENERIC == i2c_write_blocking(i2c_inst, i2c_address, buf, 1, true)) { // true to keep master control of bus
        // throw std::runtime_error("Failed to write to i2c device for read");
    }

    if (PICO_ERROR_GENERIC == i2c_read_blocking(i2c_inst, i2c_address, buf, 1, false)) { // Read back data into buf[]
        // throw std::runtime_error("Failed to read from slave");
    }

    return (buf[0]);
}

uint16_t PICO_PCA9535::read_word_data(uint8_t reg)
{
    /**
     * Private method for reading a byte from the I2C port
     */
    buf[0] = reg;

    if (PICO_ERROR_GENERIC == i2c_write_blocking(i2c_inst, i2c_address, buf, 1, true)) { // true to keep master control of bus
        // throw std::runtime_error("Failed to write to i2c device for read");
    }

    if (PICO_ERROR_GENERIC == i2c_read_blocking(i2c_inst, i2c_address, buf, 2, false)) { // Read back data into buf[]
        // throw std::runtime_error("Failed to read from slave");
    }

    uint16_t value = (buf[1] << 8) | buf[0];

    return (value);
}

void PICO_PCA9535::write_byte_data(uint8_t reg, uint8_t value)
{
    /**
     * Private method for writing a byte to the I2C port
     */
    buf[0] = reg;
    buf[1] = value;

    if (PICO_ERROR_GENERIC == i2c_write_blocking(i2c_inst, i2c_address, buf, 2, false)) {
        // throw std::runtime_error("Failed to write to i2c device for write");
    }
}

void PICO_PCA9535::write_word_data(uint8_t reg, uint16_t value)
{
    /**
     * Private method for writing a byte to the I2C port
     */
    buf[0] = reg;
    buf[1] = (uint8_t)(value&(0xff)); // lower 8 bits
    buf[2] = (uint8_t)(value>>8) & 0xff; // upper 8 bits

    if (PICO_ERROR_GENERIC == i2c_write_blocking(i2c_inst, i2c_address, buf, 3, false)) {
        // throw std::runtime_error("Failed to write to i2c device for write");
    }
}

uint8_t PICO_PCA9535::updatebyte(uint8_t byte, uint8_t bit, uint8_t value)
{
    /**
     * Private method for updating a bit within a byte
     */
    if (value == 0) {
        return (byte &= ~(1 << bit));
    }

    return (byte |= 1 << bit);
}

uint8_t PICO_PCA9535::checkbit(uint8_t byte, uint8_t bit)
{
    /**
     * Private method for checking the status of a bit within a byte
     */
    if (byte & (1 << bit)) {
        return (1);
    }

    return (0);
}

void PICO_PCA9535::set_pin(uint8_t pin, uint8_t value, uint8_t a_register, uint8_t b_register)
{
    /**
     * Private method for setting the value of a single bit within the device registers
     */
    invalid_params_if(I2C, pin < 1 || pin > 16);

    uint8_t reg = 0;
    uint8_t p = 0;
    if (pin >= 1 && pin <= 8) {
        reg = a_register;
        p = pin - 1;
    } else if (pin >= 9 && pin <= 16) {
        reg = b_register;
        p = pin - 9;
    }

    if (value > 1) {
        // throw std::out_of_range("value out of range: 0 or 1");
    }

    uint8_t newval = updatebyte(read_byte_data(reg), p, value);
    write_byte_data(reg, newval);
}

uint8_t PICO_PCA9535::get_pin(uint8_t pin, uint8_t a_register, uint8_t b_register)
{
    /**
     * Private method for getting the value of a single bit within the device registers
     */
    invalid_params_if(I2C, pin < 1 || pin > 16);
    uint8_t value = 0;

    if (pin >= 1 && pin <= 8) {
        value = checkbit(read_byte_data(a_register), pin - 1);
    } else if (pin >= 9 && pin <= 16) {
        value = checkbit(read_byte_data(b_register), pin - 9);
    }

    return value;
}

void PICO_PCA9535::set_port(uint8_t port, uint8_t value, uint8_t a_register, uint8_t b_register)
{
    /**
     * Private method for setting the value of a device register
     */
    invalid_params_if(I2C, port < 0 || port > 1);

    if (port == 0) {
        write_byte_data(a_register, value);
    }

    write_byte_data(b_register, value);
}

uint8_t PICO_PCA9535::get_port(uint8_t port, uint8_t a_register, uint8_t b_register)
{
    /**
     * Private method for getting the value of a device register
     */
    invalid_params_if(I2C, port < 0 || port > 1);

    if (port == 0) {
        return read_byte_data(a_register);
    }

    return read_byte_data(b_register);
}

void PICO_PCA9535::set_bus(uint16_t value, uint8_t a_register){
    /**
     * Private method for writing a 16-bit value to two consecutive device registers
     */
    write_word_data(a_register, value);
}
