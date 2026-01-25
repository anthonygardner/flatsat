#include "can.h"
#include "stm32f767xx.h"

void can_init(void) {
    // Enable GPIOD clock
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;
    
    // Enable CAN1 clock
    RCC->APB1ENR |= RCC_APB1ENR_CAN1EN;
    
    // Configure PD0 (RX) and PD1 (TX) as alternate function
    GPIOD->MODER &= ~(GPIO_MODER_MODER0 | GPIO_MODER_MODER1);
    GPIOD->MODER |= (0x2 << GPIO_MODER_MODER0_Pos) | (0x2 << GPIO_MODER_MODER1_Pos);
    
    // Set AF9 for CAN1
    GPIOD->AFR[0] &= ~((0xF << (0 * 4)) | (0xF << (1 * 4)));
    GPIOD->AFR[0] |= (9 << (0 * 4)) | (9 << (1 * 4));
    
    // High speed outputs
    GPIOD->OSPEEDR |= GPIO_OSPEEDR_OSPEEDR0 | GPIO_OSPEEDR_OSPEEDR1;
    
    // No pull-up/pull-down
    GPIOD->PUPDR &= ~(GPIO_PUPDR_PUPDR0 | GPIO_PUPDR_PUPDR1);
    
    // Enter CAN initialization mode
    CAN1->MCR |= CAN_MCR_INRQ;
    while (!(CAN1->MSR & CAN_MSR_INAK));
    
    // Exit sleep mode
    CAN1->MCR &= ~CAN_MCR_SLEEP;
    while (CAN1->MSR & CAN_MSR_SLAK);
    
    // Configure bit timing for 500kbps
    // APB1 = 16MHz (HSI)
    // Baud = 16MHz / (prescaler * (1 + BS1 + BS2))
    // 500000 = 16000000 / (2 * (1 + 13 + 2))
    // Prescaler = 2, BS1 = 13, BS2 = 2, SJW = 1
    CAN1->BTR = 0;
    CAN1->BTR |= (6 - 1) << CAN_BTR_BRP_Pos;
    CAN1->BTR |= (13 - 1) << CAN_BTR_TS1_Pos;
    CAN1->BTR |= (2 - 1) << CAN_BTR_TS2_Pos;
    CAN1->BTR |= (1 - 1) << CAN_BTR_SJW_Pos;
    
    // Leave initialization mode
    CAN1->MCR &= ~CAN_MCR_INRQ;
    while (CAN1->MSR & CAN_MSR_INAK);
    
    // Configure filter - accept all
    CAN1->FMR |= CAN_FMR_FINIT;
    CAN1->FA1R &= ~CAN_FA1R_FACT0;
    CAN1->FM1R &= ~CAN_FM1R_FBM0;
    CAN1->FS1R |= CAN_FS1R_FSC0;
    CAN1->sFilterRegister[0].FR1 = 0;
    CAN1->sFilterRegister[0].FR2 = 0;
    CAN1->FFA1R &= ~CAN_FFA1R_FFA0;
    CAN1->FA1R |= CAN_FA1R_FACT0;
    CAN1->FMR &= ~CAN_FMR_FINIT;
}

int can_receive(uint32_t *id, uint8_t *data, uint8_t *len) {
    if (!(CAN1->RF0R & CAN_RF0R_FMP0)) {
        return 0;
    }
    
    *id = (CAN1->sFIFOMailBox[0].RIR >> CAN_RI0R_STID_Pos) & 0x7FF;
    *len = CAN1->sFIFOMailBox[0].RDTR & 0x0F;
    
    uint32_t low = CAN1->sFIFOMailBox[0].RDLR;
    uint32_t high = CAN1->sFIFOMailBox[0].RDHR;
    data[0] = low & 0xFF;
    data[1] = (low >> 8) & 0xFF;
    data[2] = (low >> 16) & 0xFF;
    data[3] = (low >> 24) & 0xFF;
    data[4] = high & 0xFF;
    data[5] = (high >> 8) & 0xFF;
    data[6] = (high >> 16) & 0xFF;
    data[7] = (high >> 24) & 0xFF;
    
    CAN1->RF0R |= CAN_RF0R_RFOM0;
    
    return 1;
}

void can_transmit(uint32_t id, uint8_t *data, uint8_t len) {
    uint32_t timeout = 100000;

    // Wait for empty mailbox
    while (!(CAN1->TSR & CAN_TSR_TME0)) {
        if (--timeout == 0) {
            // Mailbox never freed - no ACK received
            return;
        }
    };
    
    // Set ID
    CAN1->sTxMailBox[0].TIR = (id << CAN_TI0R_STID_Pos);
    
    // Set length
    CAN1->sTxMailBox[0].TDTR = len & 0x0F;
    
    // Load data
    CAN1->sTxMailBox[0].TDLR = 
        (data[0]) | (data[1] << 8) | (data[2] << 16) | (data[3] << 24);
    CAN1->sTxMailBox[0].TDHR = 
        (data[4]) | (data[5] << 8) | (data[6] << 16) | (data[7] << 24);
    
    // Request transmission
    CAN1->sTxMailBox[0].TIR |= CAN_TI0R_TXRQ;
}