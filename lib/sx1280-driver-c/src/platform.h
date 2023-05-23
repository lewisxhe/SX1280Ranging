#pragma once

#include <Arduino.h>
#include <Wire.h>

#define I2C_SDA                     18
#define I2C_SCL                     17

#define _RADIO_SCLK_PIN              5
#define _RADIO_MISO_PIN              3
#define _RADIO_MOSI_PIN              6
#define _RADIO_CS_PIN                7
#define _RADIO_DIO1_PIN              9
#define _RADIO_BUSY_PIN              36
#define _RADIO_RST_PIN               8

//!SX1276/78 module only
#define _RADIO_DIO0_PIN              9
#define _RADIO_DIO3_PIN              21
#define _RADIO_DIO4_PIN              10
#define _RADIO_DIO5_PIN              36
//! end

//! SX1280 module only
#define RADIO_RX_PIN                21
#define RADIO_TX_PIN                10
//! end
#define _BOARD_LED                   37

///

#define LED_TX_PORT                         -1
#define LED_RX_PORT                         -1
#define RADIO_BUSY_PORT                     -1
#define RADIO_nRESET_PORT                   -1
#define RADIO_DIO1_GPIO_Port                -1
#define RADIO_NSS_PORT                      -1
#define LED_RX_PIN                          -1

#define LED_TX_PIN                          _BOARD_LED
#define RADIO_BUSY_PIN                      _RADIO_BUSY_PIN
#define RADIO_nRESET_PIN                    _RADIO_RST_PIN
#define RADIO_DIO1_Pin                      _RADIO_DIO1_PIN
#define RADIO_NSS_PIN                       _RADIO_CS_PIN

#define __disable_irq()
#define __enable_irq()

#define HAL_GPIO_ReadPin(port,pin)          digitalRead(pin)
#define GpioWrite(port,pin,value)           digitalWrite(pin,value)
#define GpioRead(port,pin)                  digitalRead(pin)
#define GpioSetIrq(port,pin,tri,handler)    attachInterrupt(pin,handler,tri)
#define HAL_Delay(ms)                       delay(ms)

// #define SpiIn(tx,size)
// #define SpiInOut(tx,rx,size)
void SpiInOut( uint8_t *txBuffer, uint8_t *rxBuffer, uint16_t size );
void SpiIn( uint8_t *txBuffer, uint16_t size );

// void TimerCreateTimer(void *func, void *param, uint32_t millisec);
void TimerCreateTimer(void (*func)(), void *param, uint32_t millisec);
void TimerCancelTimer(void);

#define SetAntenna1()
#define SetAntenna2()
void SpiInit(void);
void SpiDeInit( void );

#define HAL_GetTick()   millis()
void hwInit();
