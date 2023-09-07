/*B2 Logics by Tim Hebel 7b Sep 2023
- This code is designed for an ARduino NANO.
- I put the same code on two Nanos. One in the head and one in the top shell inside the neck ring.  I then drilled holes to run the wires to the Nano in the neck ring.
- Uses the NeoPatterns Library 3.1.1.  Be sure to use your library manager to add this library to your Arduino IDE
 
 *  Shows all patterns for strips rings and matrixes included in the NeoPattern MatrixNeoPattern and Snake library.
 *
 *  You need to install "Adafruit NeoPixel" library under "Tools -> Manage Libraries..." or "Ctrl+Shift+I" -> use "neoPixel" as filter string
 *
 *  Copyright (C) 2018  Armin Joachimsmeyer
 *  armin.joachimsmeyer@gmail.com
 *
 *  This file is part of NeoPatterns https://github.com/ArminJo/NeoPatterns.
 *
 *  NeoPatterns is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.

 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/gpl.html>.
 *
 */

#include <Arduino.h>
#include <NeoPatterns.hpp>

// #include <MatrixSnake.h>
// #if defined(__AVR__)
// #include "ADCUtils.h"
// #include <avr/power.h>
// #include <avr/pgmspace.h>

//#define ALL_PATTERN_ON_ONE_STRIP

// #define VCC_STOP_THRESHOLD_MILLIVOLT 3400   // We have voltage drop at the connectors, so the battery voltage is assumed higher, than the Arduino VCC.
// #define VCC_STOP_MIN_MILLIVOLT 3200         // We have voltage drop at the connectors, so the battery voltage is assumed higher, than the Arduino VCC.
// #define VCC_CHECK_PERIOD_MILLIS 2000         // Period of VCC checks
// #define VCC_STOP_PERIOD_REPETITIONS 9       // Shutdown after 9 times (18 seconds) VCC below VCC_STOP_THRESHOLD_MILLIVOLT or 1 time below VCC_STOP_MIN_MILLIVOLT
#define FALLING_STAR_DURATION 12
//#endif // (__AVR__)

// Which pin on the Arduino is connected to the NeoPixels?
//#define PIN_NEOPIXEL_BAR_24     2
#define PIN_NEOPIXEL_BAR_19     3 //19 HeadLogics
#define PIN_NEOPIXEL_MAIN_EYE        5 //32+16=48

#define PIN_NEOPIXEL_BAR_15F    4 //15
#define PIN_NEOPIXEL_BAR_44    6 //44 SmallEye
#define PIN_NEOPIXEL_BAR_15R    7//15

#define PIN_NEOPIXEL_BAR_40      8
#define MATRIX_NUMBER_OF_COLUMNS 5
#define MATRIX_NUMBER_OF_ROWS    8

// onComplete callback functions
void TestPatterns(NeoPatterns *aLedsPtr);
void OuterMainEye(NeoPatterns *aLedsPtr);
void InnerMainEye(NeoPatterns *aLedsPtr);
void SmallEye(NeoPatterns *aLedsPtr);
void HeadLogic(NeoPatterns *aLedsPtr);
// #ifdef ALL_PATTERN_ON_ONE_STRIP
// #define PIN_NEOPIXEL_MAIN_EYE        2
// NeoPatterns allPixel = NeoPatterns(54, PIN_NEOPIXEL_MAIN_EYE, NEO_GRB + NEO_KHZ800, &allPatternsRandomHandler);
// NeoPatterns bigRing32 = NeoPatterns(&allPixel, 0, 32, true, &allPatternsRandomHandler);
// NeoPatterns smallring16 = NeoPatterns(&allPixel, 35, 16, true, &TestPatterns);
// // NeoPatterns bar15F = NeoPatterns(&allPixel, 46, 12, true, &allPatternsRandomHandler);
// // NeoPatterns bar15R = NeoPatterns(&allPixel, 61, 16, true, &allPatternsRandomHandler);
// // NeoPatterns bar40 = NeoPatterns(&allPixel, 80, 24, true, &allPatternsRandomHandler);
// #else
// construct the NeoPatterns instances
NeoPatterns allPixel = NeoPatterns(54, PIN_NEOPIXEL_MAIN_EYE, NEO_GRB + NEO_KHZ800, &TestPatterns);
NeoPatterns bigRing32 = NeoPatterns(&allPixel, 0, 32, true, &OuterMainEye);
NeoPatterns smallring16 = NeoPatterns(&allPixel, 32, 16, true, &InnerMainEye);
NeoPatterns bar19 = NeoPatterns(19, PIN_NEOPIXEL_BAR_19, NEO_GRB + NEO_KHZ800, &HeadLogic);
NeoPatterns bar44 = NeoPatterns(44, PIN_NEOPIXEL_BAR_44, NEO_GRB + NEO_KHZ800, &SmallEye);
NeoPatterns bar15F = NeoPatterns(15, PIN_NEOPIXEL_BAR_15F, NEO_GRB + NEO_KHZ800, &TestPatterns);
NeoPatterns bar15R = NeoPatterns(15, PIN_NEOPIXEL_BAR_15R, NEO_GRB + NEO_KHZ800, &TestPatterns);
NeoPatterns bar40 = NeoPatterns(40, PIN_NEOPIXEL_BAR_40, NEO_GRB + NEO_KHZ800, &TestPatterns);
// #endif

/*
 * Specify your matrix geometry as 4th parameter.
 * ....BOTTOM ....RIGHT specify the position of the zeroth pixel.
 * See MatrixNeoPatterns.h for further explanation.
 */
// MatrixSnake NeoPixelMatrix = MatrixSnake(MATRIX_NUMBER_OF_COLUMNS, MATRIX_NUMBER_OF_ROWS, PIN_NEOPIXEL_BAR_40,
// NEO_MATRIX_BOTTOM | NEO_MATRIX_RIGHT | NEO_MATRIX_ROWS | NEO_MATRIX_PROGRESSIVE, NEO_GRB + NEO_KHZ800,
//         &MatrixAndSnakePatternsDemoHandler);

//void checkAndHandleVCCTooLow();

void setup() {
    pinMode(LED_BUILTIN, OUTPUT);

    Serial.begin(115200);
#if defined(__AVR_ATmega32U4__) || defined(SERIAL_USB) || defined(SERIAL_PORT_USBVIRTUAL)  || defined(ARDUINO_attiny3217)
    delay(2000); // To be able to connect Serial monitor after reset or power up and before first printout
#endif
    // Just to know which program is running on my Arduino
    Serial.println(F("START " __FILE__ " from " __DATE__ "\r\nUsing library version " VERSION_NEOPATTERNS));

    // This initializes the NeoPixel library and checks if enough memory was available
    // check the last object defined
    // if (!NeoPixelMatrix.begin(&Serial)) {
    //     // Blink forever
    //     while (true) {
    //         digitalWrite(LED_BUILTIN, HIGH);
    //         delay(500);
    //         digitalWrite(LED_BUILTIN, LOW);
    //         delay(500);
    //     }
    // }

// #if defined(__AVR__)
//     // setup ADC reference and channel
//     getVCCVoltageMillivoltSimple();

//     extern void *__brkval;
//     Serial.print(F("Free Ram/Stack[bytes]="));
//     Serial.println(SP - (uint16_t) __brkval);
// #endif
    bigRing32.begin();
    smallring16.begin();
    bar19.begin(5); // This initializes the NeoPixel library and sets the brightness to 5 out of 128.
    bar44.begin(5); // This initializes the NeoPixel library and sets the brightness to 5 out of 128.
    bar15F.begin(5); // This initializes the NeoPixel libraryand sets the brightness to 5 out of 128.
    bar15R.begin(5); // This initializes the NeoPixel libraryand sets the brightness to 5 out of 128.
    bar40.begin(5); // This initializes the NeoPixel libraryand sets the brightness to 5 out of 128.

    bigRing32.PixelFlags |= PIXEL_FLAG_GEOMETRY_CIRCLE;
    smallring16.PixelFlags |= PIXEL_FLAG_GEOMETRY_CIRCLE;
    // bar15F.PixelFlags |= PIXEL_FLAG_GEOMETRY_CIRCLE;
    // bar15R.PixelFlags |= PIXEL_FLAG_GEOMETRY_CIRCLE;
    // bar40.PixelFlags |= PIXEL_FLAG_GEOMETRY_CIRCLE;

    delay(300); // to avoid partial patterns at power up

    bigRing32.ColorWipe(COLOR32_RED, 50, 0, DIRECTION_DOWN);
    smallring16.ColorWipe(COLOR32_PURPLE, 50);
    bar15F.ColorWipe(COLOR32_PURPLE, 50);
    bar15R.ColorWipe(COLOR32_RED, 50, 0, DIRECTION_DOWN);
    bar40.ColorWipe(COLOR32_GREEN, 50);
    bar19.ColorWipe(COLOR32_BLUE, 50, 0, DIRECTION_DOWN);
    bar44.Stripes(COLOR32_BLUE, 5, COLOR32_RED, 3, 48, 50);
//    bar44.ScannerExtended(COLOR32_BLUE, 5, 50, 1,
//            FLAG_SCANNER_EXT_ROCKET | FLAG_SCANNER_EXT_VANISH_COMPLETE | FLAG_SCANNER_EXT_START_AT_BOTH_ENDS);
    // NeoPixelMatrix.clear(); // Clear matrix
    // NeoPixelMatrix.show();
    // NeoPixelMatrix.Delay(7000); // start later
    // setMatrixAndSnakePatternsDemoHandlerTickerText(F("I love NeoPixel"));

// #if defined(__AVR__)
//     /*
//      * Print voltage once on matrix
//      */
//     char sStringBufferForVCC[7] = "xxxxmV";
//     uint16_t tVCC = getVCCVoltageMillivoltSimple();
//     if (tVCC < 4300) {
//         itoa(tVCC, sStringBufferForVCC, 10);
//        // NeoPixelMatrix.Ticker(sStringBufferForVCC, NeoPatterns::Wheel(0), COLOR32_BLACK, 80, DIRECTION_LEFT);
//     }
// #endif // defined(__AVR__)

    Serial.println("started");
}

uint8_t sWheelPosition = 0; // hold the color index for the changing ticker colors

void loop() {
// #if defined(__AVR__)
//     checkAndHandleVCCTooLow();
// #endif // defined(__AVR__)
    bigRing32.update();
    smallring16.update();
    bar19.update();
    bar44.update();
    bar15F.update();
    bar15R.update();
    bar40.update();
    // if (NeoPixelMatrix.update()) {
    //     if (NeoPixelMatrix.ActivePattern == PATTERN_TICKER) {
    //         // change color of ticker after each update
    //         NeoPixelMatrix.Color1 = NeoPatterns::Wheel(sWheelPosition);
    //         sWheelPosition += 4;
    //     } else if (NeoPixelMatrix.ActivePattern == PATTERN_SNAKE) {
    //         if (NeoPixelMatrix.Index == 4) {
    //             NeoPixelMatrix.Direction = DIRECTION_LEFT;
    //         } else if (NeoPixelMatrix.Index == 8) {
    //             NeoPixelMatrix.Direction = DIRECTION_DOWN;
    //         }
    //     }
    // }
}

/*
 * Handler for testing patterns
 */
void TestPatterns(NeoPatterns *aLedsPtr) {
    static int8_t sState = 0;

    switch (sState) {
    case 0:
        aLedsPtr->ColorWipe(COLOR32_RED_HALF, 50);
        break;
    case 1:
        aLedsPtr->Delay(500);
        break;
    case 2:
        aLedsPtr->Heartbeat(COLOR32_GREEN_HALF, 50, 0);
        break;
    case 3:
        aLedsPtr->Delay(500);
        break;
    case 4:
        aLedsPtr->RainbowCycle(50, DIRECTION_UP);
        break;
    case 5:
        aLedsPtr->RainbowCycle(50, DIRECTION_DOWN);
        break;
    case 6:
        aLedsPtr->Delay(400);
        break;
    case 7:
        aLedsPtr->Fire(20, 400); // OK Fire(30, 260)is also OK
        break;
    case 8:
        // switch to random
        initMultipleFallingStars(aLedsPtr, COLOR32_WHITE_QUARTER, 7, 30, 3, &allPatternsRandomHandler);
        sState = -1; // Start from beginning
        break;
    default:
        Serial.println("ERROR");
        break;
    }

    Serial.print("TestPatterns: Pin=");
    Serial.print(aLedsPtr->getPin());
    Serial.print(" Length=");
    Serial.print(aLedsPtr->numPixels());
    Serial.print(" ActivePattern=");
    aLedsPtr->printPatternName(aLedsPtr->ActivePattern, &Serial);
    Serial.print("|");
    Serial.print(aLedsPtr->ActivePattern);
    Serial.print(" State=");
    Serial.println(sState);

    sState++;
}

void OuterMainEye(NeoPatterns *aLedsPtr){
  static int8_t sState = 0;

      switch (sState) {
    case 0:
        aLedsPtr->ColorWipe(COLOR32(32,0,0), 30,0, DIRECTION_UP);
        break;
    case 1:
        aLedsPtr->Delay(700);
        break;
    case 2:
      aLedsPtr->ColorWipe(COLOR32(16,0,0), 180, 0, DIRECTION_DOWN);

        break;
    case 3:
        aLedsPtr->Delay(random(500,1700));
        sState = -1;
        break;
      }

   sState++;
}
void InnerMainEye(NeoPatterns *aLedsPtr){
  aLedsPtr->Heartbeat(COLOR32(32,8,0), 120, 0);
}
void SmallEye(NeoPatterns *aLedsPtr){
  //aLedsPtr->Fire(20, 400);
  //PurpleWuarter COLOR32(64,0,64)
  //BllueQuarter COLOR32(0,0,64)
  aLedsPtr->Stripes(COLOR32(0,0,24), 5, COLOR32(4,0,8), 3, 48, 50);  
}
void HeadLogic(NeoPatterns *aLedsPtr){
    //aLedsPtr->RainbowCycle(50, DIRECTION_DOWN);
    initMultipleFallingStars(aLedsPtr, COLOR32_WHITE_QUARTER, 7, 30, 3, &allPatternsRandomHandler);
}
