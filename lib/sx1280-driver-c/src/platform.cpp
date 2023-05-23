#include <SPI.h>
#include <Arduino.h>
#include <Wire.h>
#include "platform.h"

#define LTspeedMaximum  8000000
#define LTdataOrder     MSBFIRST
#define LTdataMode      SPI_MODE0

static hw_timer_t *timer = NULL;
static volatile SemaphoreHandle_t timerSemaphore;
static portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;


void checkBusy()
{
    // Serial.println(F("checkBusy() Revised"));
    // uint32_t startmS = millis();
    // do {
    //     if ( ((uint32_t) (millis() - startmS) > 9)) { //wait 10mS for busy to complete
    //         Serial.println(F("ERROR - Busy Timeout!"));
    //         break;
    //     }
    // } while (digitalRead(RADIO_BUSY_PIN));
}


/* SPI1 init function */
void SpiInit(void)
{
    // MX_SPI1_Init();
    // RadioSpiHandle = hspi1;

    SPI.begin(_RADIO_SCLK_PIN, _RADIO_MISO_PIN, _RADIO_MOSI_PIN);
    Wire.begin(I2C_SDA, I2C_SCL);



}

void SpiDeInit( void )
{
    // HAL_SPI_DeInit( &RadioSpiHandle );
    SPI.end();
}

/*!
 * @brief Sends txBuffer and receives rxBuffer
 *
 * @param [IN] txBuffer Byte to be sent
 * @param [OUT] rxBuffer Byte to be sent
 * @param [IN] size Byte to be sent
 */
void SpiInOut( uint8_t *txBuffer, uint8_t *rxBuffer, uint16_t size )
{

// #ifdef STM32L4XX_FAMILY
//     HAL_SPIEx_FlushRxFifo( &RadioSpiHandle ); // Comment For STM32L0XX and STM32L1XX Intégration, uncomment for STM32L4XX Intégration
// #endif
// #ifdef USE_DMA
//     blockingDmaFlag = true;
//     HAL_SPI_TransmitReceive_DMA( &SpiHandle, txBuffer, rxBuffer, size );
//     WAIT_FOR_BLOCKING_FLAG
// #else
//     // HAL_SPI_TransmitReceive( &RadioSpiHandle, txBuffer, rxBuffer, size, HAL_MAX_DELAY );
// #endif

    // Serial.println(F("readCommand() "));
    uint16_t i;
    checkBusy();
    SPI.beginTransaction(SPISettings(LTspeedMaximum, LTdataOrder, LTdataMode));
    digitalWrite(RADIO_NSS_PIN, LOW);
    for ( i = 0; i < size; i++ ) {
        *(rxBuffer + i) = SPI.transfer(txBuffer[i]);
    }
    digitalWrite(RADIO_NSS_PIN, HIGH);

    SPI.endTransaction();
}

//TX
void SpiIn( uint8_t *txBuffer, uint16_t size )
{
// #ifdef USE_DMA
//     blockingDmaFlag = true;
//     HAL_SPI_Transmit_DMA( &SpiHandle, txBuffer, size );
//     WAIT_FOR_BLOCKING_FLAG
// #else
//     // HAL_SPI_Transmit( &RadioSpiHandle, txBuffer, size, HAL_MAX_DELAY );
// #endif

// Serial.println(F("writeCommand() "));
    uint16_t index;
    checkBusy();
    SPI.beginTransaction(SPISettings(LTspeedMaximum, LTdataOrder, LTdataMode));
    digitalWrite(RADIO_NSS_PIN, LOW);
    for (index = 0; index < size; index++) {
        SPI.transfer(txBuffer[index]);
    }
    digitalWrite(RADIO_NSS_PIN, HIGH);
    SPI.endTransaction();
    checkBusy();
}


typedef void (*TimerFunc)(void *parms);
/* Function callback to call when timer has fired */
TimerFunc timerFunction;
/* Parameter of the function callback */
void *timerParam;


void hwInit()
{
    pinMode(LED_TX_PIN, OUTPUT);
    pinMode(RADIO_BUSY_PIN, INPUT);
    pinMode(RADIO_nRESET_PIN, OUTPUT);
    pinMode(RADIO_DIO1_Pin, INPUT);
    pinMode(RADIO_NSS_PIN, OUTPUT);
    pinMode(RADIO_RX_PIN, OUTPUT);
    pinMode(RADIO_TX_PIN, OUTPUT);
    // Create semaphore to inform us when the timer has fired
    timerSemaphore = xSemaphoreCreateBinary();
    SpiInit();
}

void ARDUINO_ISR_ATTR onTimer()
{
    // Increment the counter and set the time of ISR
    portENTER_CRITICAL_ISR(&timerMux);
    if (timerFunction) {
        timerFunction(timerParam);
    }
    portEXIT_CRITICAL_ISR(&timerMux);
    // Give a semaphore that we can check in the loop
    xSemaphoreGiveFromISR(timerSemaphore, NULL);
    // It is safe to use digitalRead/Write here if you want to toggle an output
}


void TimerCreateTimer(void (*func)(), void *param, uint32_t millisec)
{
    // Serial.println("TimerCreateTimer .....");
    if (timer) {
        return;
    }

    // Use 1st timer of 4 (counted from zero).
    // Set 80 divider for prescaler (see ESP32 Technical Reference Manual for more
    // info).
    timer = timerBegin(0, 80, true);

    // Attach onTimer function to our timer.
    timerAttachInterrupt(timer, &onTimer, true);

    // Set alarm to call onTimer function every second (value in microseconds).
    // Repeat the alarm (third parameter)
    timerAlarmWrite(timer, millisec *  1000, true);

    Serial.printf("TimerCreateTimer : %lu\n", millisec);

    // Start an alarm
    timerAlarmEnable(timer);

    timerFunction = (TimerFunc)func;
    timerParam = param;

}

/*!
 * \brief Cancel the current running timer.
 *
 */
void TimerCancelTimer(void)
{
    // Serial.println("TimerCancelTimer.");
    /* Disable the timer */
    timerDetachInterrupt(timer);
    timerEnd(timer);
    timer = NULL;
}
