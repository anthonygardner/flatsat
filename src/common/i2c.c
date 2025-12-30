#include "i2c.h"
#include "stm32f767xx.h"

static void i2c_enable_clocks(void) {
    // PB8: SCL, PB9: SDA
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
    RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;
}

static void i2c_configure_pins(void) {
    // Configure pins as alternate functions
    GPIOB->MODER &= ~((0b11 << 16) | (0b11 << 18));
    GPIOB->MODER |= (0b10 << 16) | (0b10 << 18);

    // Set lines as open-drain
    GPIOB->OTYPER |= (1 << 8) | (1 << 9);

    // Enable internal pull-up resistors
    GPIOB->PUPDR &= ~((0b11 << 16) | (0b11 << 18));
    GPIOB->PUPDR |= (0b01 << 16) | (0b01 << 18);

    // Select AF4 (I2C1)
    GPIOB->AFR[1] &= ~((0xF << 0) | (0xF << 4));
    GPIOB->AFR[1] |= (4 << 0) | (4 << 4);
}

static void i2c_configure_peripheral(void) {
    // Disable I2C1 before configuring
    I2C1->CR1 &= ~I2C_CR1_PE;

    // Set timing for 100 kHz at 108 MHz APB1 clock
    I2C1->TIMINGR = 0x10707DBC;

    // Enable I2C1
    I2C1->CR1 |= I2C_CR1_PE;
}

uint8_t i2c_read(uint8_t addr, uint8_t* data, uint8_t len) {
    I2C1->CR2 = 0;          // Clear first
    I2C1->CR2 = (addr << 1) // Set slave address
              | (len << 16) // Set byte count
              | (1 << 10)   // Set read direction
              | (1 << 25)   // AUTOEND: stop after NBYTES
              | (1 << 13);  // Trigger START

    for (int i = 0; i < len; i++) {
        while (!(I2C1->ISR & (1 << 2))); // Wait for RX
        data[i] = I2C1->RXDR;
    }

    while (I2C1->CR2 & (1 << 13)); // Wait for START to clear
    
    return 0;
}

uint8_t i2c_write(uint8_t addr, uint8_t* data, uint8_t len) {
    I2C1->CR2 = 0;          // Clear first
    I2C1->CR2 = (addr << 1) // Set slave address
              | (len << 16) // Set byte count
              | (0 << 10)   // Set write direction
              | (1 << 13);  // Trigger START

    for (int i = 0; i < len; i++) {
        while (!(I2C1->ISR & (1 << 1))); // Wait for TX
        I2C1->TXDR = data[i];
    }

    while (!(I2C1->ISR & (1 << 6))); // Wait for transfer to complete

    I2C1->CR2 |= (1 << 14); // Trigger STOP
    
    return 0;
}

uint8_t i2c_read_register(uint8_t addr, uint8_t reg, uint8_t* value) {
    // Send register address
    i2c_write(addr, &reg, 1);

    // Read one byte back
    i2c_read(addr, value, 1);

    return 0;
}

uint8_t i2c_read_registers(uint8_t addr, uint8_t reg, uint8_t* buffer, uint8_t len) {
    i2c_write(addr, &reg, 1);
    i2c_read(addr, buffer, len);
    return 0;
}

uint8_t i2c_write_register(uint8_t addr, uint8_t reg, uint8_t value) {
    uint8_t data[2] = {reg, value};
    i2c_write(addr, data, 2);
    return 0;
}

void i2c_init() {
    i2c_enable_clocks();
    i2c_configure_pins();
    i2c_configure_peripheral();
}