
#ifndef _PICO_PCA9535_PIO_H
#define _PICO_PCA9535_PIO_H

#include <stdint.h>

#include "pico/stdlib.h"
#include "hardware/i2c.h"

#ifdef __cplusplus
extern "C" {
#endif

class PICO_PCA9535
{
public:
    /**
     * PICO_PCA9535 object
     * @param i2c - I2C bus instance to use
     * @param address - I2C address for the target device
     */
    PICO_PCA9535(i2c_inst_t *i2c, uint8_t address);

    /**
     * Set the IO direction for an individual pin
     * @param pins - 1 to 16
     * @param direction - 1 = input, 0 = output
     */
    void set_pin_direction(uint8_t pin, uint8_t direction);

    /**
     * Get the IO direction for an individual pin
     * @param pins - 1 to 16
     */
    uint8_t get_pin_direction(uint8_t pin);

    /**
     * Set the direction for an IO port
     * @param port - 0 = pins 1 to 8, port 1 = pins 9 to 16
     * @param direction - 0 to 255 (0xFF).  For each bit 1 = input, 0 = output
     */
    void set_port_direction(uint8_t port, uint8_t direction);

    /**
     * Get the direction for an IO port
     * @param port - 0 = pins 1 to 8, port 1 = pins 9 to 16
     */
    uint8_t get_port_direction(uint8_t port);

    /**
     * Set the direction for the IO bus
     * @param direction - 0 to 65535 (0xFFFF).  For each bit 1 = input, 0 = output
     */
    void set_bus_direction(uint16_t direction);

    /**
     * Get the direction for the IO bus
     */
    uint16_t get_bus_direction();

    /**
     * Write to an individual pin 1 - 16
     * @param pin - 1 to 16
     * @param value - 0 = logic low, 1 = logic high
     */
    void write_pin(uint8_t pin, uint8_t value);

    /**
     * Write to all pins on the selected port
     * @param port - 0 = pins 1 to 8, port 1 = pins 9 to 16
     * @param value - 0 to 255 (0xFF)
     */
    void write_port(uint8_t port, uint8_t value);

    /**
     * Write to all pins on the selected bus
     * @param value - 0 to 65535 (0xFFFF). For each bit 1 = logic high, 0 = logic low
     */
    void write_bus(uint16_t value);

    /**
     * Read the value of an individual pin
     * @param pin - 1 to 16
     * @returns - 0 = logic low, 1 = logic high
     */
    uint8_t read_pin(uint8_t pin);

    /**
     * Read all pins on the selected port
     * @param port - 0 = pins 1 to 8, port 1 = pins 9 to 16
     * @returns - 0 to 255 (0xFF). For each bit 1 = logic high, 0 = logic low
     */
    uint8_t read_port(uint8_t port);

    /**
     * Read all pins on the selected bus
     * @returns - 0 to 65535 (0xFFFF). For each bit 1 = logic high, 0 = logic low
     */
    uint16_t read_bus();

    /**
     * Set the polarity of the selected pin
     * @param pin - 1 to 16
     * @param polarity - 0 = non-inverted, 1 = inverted
     */
    void set_pin_polarity(uint8_t pin, uint8_t polarity);

    /**
     * Get the polarity of the selected pin
     * @param pin - 1 to 16
     */
    uint8_t get_pin_polarity(uint8_t pin);

    /**
     * Set the polarity of the pins on a selected port
     * @param port - 0 = pins 1 to 8, port 1 = pins 9 to 16
     * @param polarity - 0 to 255 (0xFF). For each bit 0 = non-inverted, 1 = inverted
     */
    void set_port_polarity(uint8_t port, uint8_t polarity);

    /**
     * Get the polarity of the selected pin
     * @param port - 0 = pins 1 to 8, port 1 = pins 9 to 16
     */
    uint8_t get_port_polarity(uint8_t port);

    /**
     * Set the polarity of the pins on a selected bus
     * @param polarity - 0 to 65535 (0xFFFF). For each bit 0 = non-inverted, 1 = inverted
     */
    void set_bus_polarity(uint16_t polarity);

    /**
     * Get the polarity of the bus
     */
    uint16_t get_bus_polarity();

private:
    // PCA9535 register addresses
    enum PCA9535
    {
        INPUTPORT0  = 0x00, // Command byte Input port 0
        INPUTPORT1  = 0x01, // Command byte Input port 1
        OUTPUTPORT0 = 0x02, // Command byte Output port 0
        OUTPUTPORT1 = 0x03, // Command byte Output port 1
        INVERTPORT0 = 0x04, // Command byte Polarity Inversion port 0
        INVERTPORT1 = 0x05, // Command byte Polarity Inversion port 1
        CONFIGPORT0 = 0x06, // Command byte Configuration port 0
        CONFIGPORT1 = 0x07, // Command byte Configuration port 1
    };

    i2c_inst_t *i2c_inst; // I2C bus
    uint8_t i2c_address; // I2C address
    uint8_t buf[3];

    /**
     * Private method for reading a byte from the I2C port
     */
    uint8_t read_byte_data(uint8_t reg);

    /**
     * Private method for reading a word from the I2C port
     */
    uint16_t read_word_data(uint8_t reg);

    /**
     * Private method for writing a byte to the I2C port
     */
    void write_byte_data(uint8_t reg, uint8_t value);

    /**
     * Private method for writing a word to the I2C port
     */
    void write_word_data(uint8_t reg, uint16_t value);

    /**
     * Private method for updating a bit within a byte
     */
    uint8_t updatebyte(uint8_t byte, uint8_t bit, uint8_t value);

    /**
     * Private method for checking the status of a bit within a byte
     */
    uint8_t checkbit(uint8_t byte, uint8_t bit);

    /**
     * Private method for setting the value of a single bit within the device registers
     */
    void set_pin(uint8_t pin, uint8_t value, uint8_t a_register, uint8_t b_register);

    /**
     * Private method for getting the value of a single bit within the device registers
     */
    uint8_t get_pin(uint8_t pin, uint8_t a_register, uint8_t b_register);

    /**
     * Private method for setting the value of a device register
     */
    void set_port(uint8_t port, uint8_t value, uint8_t a_register, uint8_t b_register);

    /**
     * Private method for getting the value of a device register
     */
    uint8_t get_port(uint8_t port, uint8_t a_register, uint8_t b_register);

    /**
     * Private method for writing a 16-bit value to two consecutive device registers
     */
    void set_bus(uint16_t value, uint8_t a_register);
};

#ifdef __cplusplus
}
#endif

#endif
