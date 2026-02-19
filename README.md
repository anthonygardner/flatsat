# Flatsat Testbed

## Purpose

This project explores the interface layer of three satellite subsystems: the primary onboard compute system (OBC), the attitude determination and control system (ADCS), and the electronic power system (EPS).

## System Architecture

```mermaid
graph TD
    Laptop -->|Ethernet| PoE[PoE Switch]
    PoE -->|Ethernet| RPi["Raspberry Pi (GCS)"]

    RPi -->|UART| OBC["Nucleo 1 (OBC)"]
    RPi -->|RS485-USB| OBC

    OBC -->|CAN| CT1[CAN Transceiver 1]
    OBC -->|I2C| MPU[MPU6050]

    ADCS["Nucleo 2 (ADCS)"] -->|CAN| CT2[CAN Transceiver 2]
    ADCS -->|PWM| Motor

    EPS["Nucleo 3 (EPS)"] -->|GPIO| Relay[Relay Module]
    EPS -->|CAN| CT3[CAN Transceiver 3]

    Relay -->|VCC| LED1[LED 1]
    Relay -->|VCC| LED2[LED 2]

    CT1 -->|CAN| Bus[CAN Bus]
    CT2 -->|CAN| Bus
    CT3 -->|CAN| Bus

    CT1 -->|USB-CAN| ExtCAN[USB-CAN Interface]
```

## Bill of Materials

Part | Amount | Vendor
-----|--------|-------
STMicroelectronics NUCLEO-F767ZI development board | 3x | Amazon
TJA1050 CAN Bus transceiver | 3x | Amazon
AOICRIE MPU-6050 3-axis IMU | 2x | Amazon
SUZLAZYR 2804 DC brushless motor | 1x | Amazon
BEMONOC 25GA370 150RPM DC geared motor | 1x | Amazon
Reland Sun L298N motor drive controller board | 1x | Amazon
SunFounder 4-channel 5V relay shield | 1x | Amazon
ATECC608 AES-128 encryption authentication chip | 1x | Amazon
NETGEAR 5-port managed Ethernet switch | 1x | Amazon