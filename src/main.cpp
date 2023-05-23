#include "radio.h"
#include "sx1280.h"
#include "radio.h"
#include "utilities.h"
#include "demoRanging.h"
#include "RangingDisplay.h"
#include <U8g2lib.h>

U8G2_SSD1306_128X64_NONAME_F_HW_I2C *u8g2 = nullptr;

// #define DEMO_SETTING_ENTITY                          MASTER
#define DEMO_SETTING_ENTITY                          SLAVE

#define PROXBUFFERSIZE                                  20
uint16_t FwVersion = 0;

void setup()
{
    Serial.begin(115200);

    hwInit();

    Wire.beginTransmission(0x3C);
    if (Wire.endTransmission() == 0) {
        Serial.println("Started OLED");
        u8g2 = new U8G2_SSD1306_128X64_NONAME_F_HW_I2C(U8G2_R0, U8X8_PIN_NONE);
        u8g2->begin();
        u8g2->clearBuffer();
        u8g2->setFlipMode(0);
        u8g2->setFontMode(1); // Transparent
        u8g2->setDrawColor(1);
        u8g2->setFontDirection(0);
        u8g2->firstPage();
        u8g2->setFont(u8g2_font_inb19_mr);
        u8g2->drawStr(0, 30, "LilyGo");
        u8g2->drawHLine(2, 35, 47);
        u8g2->drawHLine(3, 36, 47);
        u8g2->drawVLine(45, 32, 12);
        u8g2->drawVLine(46, 33, 12);
        u8g2->setFont(u8g2_font_inb19_mf);
        u8g2->drawStr(58, 60, "LoRa");
        u8g2->sendBuffer();
        u8g2->setFont(u8g2_font_fur11_tf);
        delay(3000);
    }

    Serial.println("Start!");

    GpioWrite(LED_TX_PORT, LED_TX_PIN, 1);


    /* 1- Initialize the Ranging Application */
    RangingDemoInitApplication( DEMO_SETTING_ENTITY );

    RangingDemoSetRangingParameters( 10, DEMO_RNG_ADDR_2, DEMO_RNG_ANT_1, DEMO_RNG_UNIT_SEL_M );
    RangingDemoSetRadioParameters( LORA_SF6, LORA_BW_1600, LORA_CR_4_5, DEMO_CENTRAL_FREQ_PRESET2, DEMO_POWER_TX_MAX );

    Radio.Reset();
    FwVersion = Radio.GetFirmwareVersion();


    if (DEMO_SETTING_ENTITY == MASTER) {
        printf("Ranging Demo as Master , firmware %d \n\r", FwVersion);
        if (u8g2) {
            u8g2->clearBuffer();
            u8g2->setCursor(0, 16);
            u8g2->println( "RangingMaster");
            u8g2->sendBuffer();
        }
    } else {
        printf("Ranging Demo as Slave , firmware %d  \n\r", FwVersion);
        if (u8g2) {
            u8g2->clearBuffer();
            u8g2->setCursor(0, 16);
            u8g2->println( "RangingSlave");
            u8g2->sendBuffer();
        }
    }

    GpioWrite(LED_TX_PORT, LED_TX_PIN, 0);
}


void loop()
{
    RangingDemoStatus_t demoStatus;
    GpioWrite(LED_TX_PORT, LED_TX_PIN, 1);
    // Run the ranging demo.
    do {
        demoStatus = RangingDemoRun( );
        digitalWrite(_BOARD_LED, 1 - digitalRead(_BOARD_LED));
    } while ( demoStatus == DEMO_RANGING_RUNNING );

    GpioWrite(LED_TX_PORT, LED_TX_PIN, 0);

    // If master, display the ranging result.
    if ( DEMO_SETTING_ENTITY == MASTER ) {

        if (u8g2) {
            u8g2->clearBuffer();
            u8g2->setCursor(0, 16);
            u8g2->println( "RangingMaster");
            u8g2->setCursor(0, 32);
            u8g2->print("[");
            u8g2->print(millis() / 1000);
            u8g2->print("]:");
            u8g2->setCursor(0, 48);
            u8g2->print( "Distance:"); u8g2->print( 0); u8g2->print("meters(raw)");
            u8g2->sendBuffer();
        }
        Serial.println("-----------------------------");
        RangingDisplayUartOutputData( );
        RangingDisplayUartOutputDistance( );
        HAL_Delay(1000);
    }
    if ( demoStatus != DEMO_RANGING_TERMINATED ) {
        RangingDemoReset( );
        if (u8g2) {
            u8g2->clearBuffer();
            u8g2->setCursor(0, 16);
            u8g2->println( "RangingMaster");
            u8g2->setCursor(0, 32);
            u8g2->print("[");
            u8g2->print(millis() / 1000);
            u8g2->print("]:");
            u8g2->print( "RangingReset");
            u8g2->sendBuffer();
        }
    }

}







