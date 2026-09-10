#pragma once

// Nano 2 (Control) Pin Definitions
// Safety-critical outputs: servos, parachute, stage2 ignition

#define LED0 2
#define BUTTON_1 3
#define LED1 4
#define PARACHUTE_PIN 5
#define LED2 6
#define BUTTON_2 7
#define LED3 8
#define STAGE2_IGNITION_PIN 9

// UART to Nano 1 uses hardware Serial (pins 0/1)
// Software Serial alternative pins if needed:
// #define UART_TX_PIN 10
// #define UART_RX_PIN 11

#define SERVO_0 A2
#define SERVO_1 A3
