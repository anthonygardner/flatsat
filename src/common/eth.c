#include "eth.h"
#include "uart.h"
#include "stm32f767xx.h"

struct {
    uint32_t status;    // RDES0
    uint32_t buff_len;  // RDES1
    uint32_t buff_addr; // RDES2
    uint32_t next_addr; // RDES3
} eth_rx_descriptor_t;

struct {
    uint32_t status;    // TDES0
    uint32_t buff_len;  // TDES1
    uint32_t buff_addr; // TDES2
    uint32_t next_addr; // TDES3
} eth_tx_descriptor_t;

static void eth_enable_clocks(void) {
    // Enable GPIO port clocks for reduced media independent interface (RMII) pins...
    // REF_CLK => PA1
    // TX_EN   => PG11
    // TXD0    => PG13
    // TXD1    => PB13
    // RXD0    => PC4
    // RXD1    => PC5
    // CRS_DV  => PA7
    
    // ...and clocks for management interface pins to talk to PHY
    // MDIO => PA2
    // MDC  => PC1

    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN
                  | RCC_AHB1ENR_GPIOBEN
                  | RCC_AHB1ENR_GPIOCEN
                  | RCC_AHB1ENR_GPIOGEN;

    // Enable SYSCFG clock
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;

    // Set RMII mode in SYSCFG
    SYSCFG->PMC |= SYSCFG_PMC_MII_RMII_SEL;

    // Enable Ethernet MAC clocks
    // ETHMAC (main MAC logic)
    // ETHMACTX (transmit path)
    // ETHMACRX (receive path)
    RCC->AHB1ENR |= RCC_AHB1ENR_ETHMACEN
                  | RCC_AHB1ENR_ETHMACTXEN
                  | RCC_AHB1ENR_ETHMACRXEN;
}

static void eth_configure_pins(void) {
    // Each pin needs:
    // 1. Mode set to alternate function (MAC peripheral will drive pins directly)
    // 2. Speed set high (pin voltage slew rate control)
    // 3. Pull set to none (prevents internal resistor interference)
    // 4. Alternate function number set to AF11 (ETH)

    // Port A: PA1 (REF_CLK), PA2 (MDIO), PA7 (CRS_DV)
    GPIOA->MODER &= ~((0b11 << 2)
                    | (0b11 << 4)
                    | (0b11 << 14)); // Clear

    GPIOA->MODER |= (0b10 << 2)
                  | (0b10 << 4)
                  | (0b10 << 14); // Set

    GPIOA->OSPEEDR |= (0b10 << 2)
                    | (0b10 << 4)
                    | (0b10 << 14);

    GPIOA->PUPDR &= ~((0b11 << 2)
                    | (0b11 << 4)
                    | (0b11 << 14));
    
    GPIOA->AFR[0] |= (0b1011 << 4)
                   | (0b1011 << 8)
                   | (0b1011 << 28);

    // Port B: PB13 (TXD1)
    GPIOB->MODER &= ~(0b11 << 26);

    GPIOB->MODER |= (0b10 << 26);

    GPIOB->OSPEEDR |= (0b10 << 26);

    GPIOB->PUPDR &= ~(0b11 << 26);

    GPIOB->AFR[1] |= (0b1011 << 20);

    // Port C: PC1 (MDC), PC4 (RXD0), PC5 (RXD1)
    GPIOC->MODER &= ~((0b11 << 2)
                    | (0b11 << 8)
                    | (0b11 << 10));

    GPIOC->MODER |= (0b10 << 2)
                  | (0b10 << 8)
                  | (0b10 << 10);

    GPIOC->OSPEEDR |= (0b10 << 2)
                    | (0b10 << 8)
                    | (0b10 << 10);

    GPIOC->PUPDR &= ~((0b11 << 2)
                    | (0b11 << 8)
                    | (0b11 << 10));

    GPIOC->AFR[0] |= (0b1011 << 4)
                   | (0b1011 << 16)
                   | (0b1011 << 20);

    // Port G: PG11 (TX_EN), PG13 (TXD0)
    GPIOG->MODER &= ~((0b11 << 22)
                    | (0b11 << 26));

    GPIOG->MODER |= (0b10 << 22)
                  | (0b10 << 26);

    GPIOG->OSPEEDR |= (0b10 << 22)
                    | (0b10 << 26);

    GPIOG->PUPDR &= ~((0b11 << 22)
                    | (0b11 << 26));

    GPIOG->AFR[1] |= (0b1011 << 12)
                   | (0b1011 << 20);
}

static void eth_reset_mac(void) {
    // Request reset
    ETH->DMABMR |= ETH_DMABMR_SR;

    // Poll hardware until the DMA bit clears
    while (ETH->DMABMR & ETH_DMABMR_SR);
}

static uint16_t eth_mdio_read(uint8_t phy_addr, uint8_t reg_addr) {
    // Wait until MII busy bit clears
    while (ETH->MACMIIAR & ETH_MACMIIAR_MB);

    // Write to MACMIIAR: PHY address, register address, clock range, read, busy
    ETH->MACMIIAR = (phy_addr << 11)
                  | (reg_addr << 6)
                  | (0b000 << 2)
                  | ETH_MACMIIAR_MB; // Setting busy triggers the read

    // Wait until busy clears again
    while (ETH->MACMIIAR & ETH_MACMIIAR_MB);

    // Read the result
    uint16_t result = ETH->MACMIIDR;
    return result;
}

static uint32_t eth_read_phy_id(void) {
    uint16_t reg2 = eth_mdio_read(0, 2); // PHY addr 0, register 2
    uint16_t reg3 = eth_mdio_read(0, 3); // PHY addr 0, register 3
    return (reg2 << 16) | reg3;
}

uint16_t eth_get_link_status(void) {
    // Read PHY register 1 (Basic Status Register)
    uint16_t bsr = eth_mdio_read(0, 1);

    // Bit 2 is Link Status: 0 = link down, 1 = link up
    return (bsr & (1 << 2)) != 0;
}

void eth_init(void) {
    eth_enable_clocks();
    eth_configure_pins();
    eth_reset_mac();
    
    uint32_t phy_id = eth_read_phy_id();
    uart_print_hex(phy_id);
}