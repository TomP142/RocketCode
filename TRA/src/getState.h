// Libraries
#include <Wire.h>
#include <PWMServo.h>
#include <SPI.h>
#include <SdFat.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <string>
#include <sstream>
#include <stdlib.h>
#include <RH_ASK.h>
#include <SPIFlash.h>
using namespace std;

// Compiling all together
#include <settings.h>
#include <startup.h>
#include <mainFunctions.h>

// State

// 1 - Starting up
// 2 - Ready for start
// 3 - Launch
// 4 - Accelerating
// 5 - Engine Off
// 6 - Apogee / Uncontrolled descend
// 7 - Start landing procedure
// 8 - Landed - Data Transfer

// WaitTime 2.0

int getState()
{
    // 1 - Starting up
    // 2 - Ready for start
    // 3 - Launch
    // 4 - Accelerating
    // 5 - Engine Off
    // 6 - Apogee / Uncontrolled descend
    // 7 - Start landing procedure
    // 8 - Landed - Data Transfer
    // 43 - Abort
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    currentMillisTime = millis();

    // Check for unusual state

    if (currentState == 43)
    {
        Serial.println("Aborting flight");
        Serial.print("Current state: ");
        Serial.println(currentState);
        abortFunc();
        return currentState;
    }

    // Check for liftoff
    if (currentState == 2)
    {
        if (a.acceleration.y > 0)
        {
            if (currentMillisTime >= liftOffMillisWaitTime)
            {
                currentState++;
                Serial.println("Lift off!");
                liftOffMillisTime = millis();
                Serial.print("Calibrated Altitude: ");
                Serial.println(CalibratedAltitude);
                Serial.print("Current State changed to: ");
                Serial.println(currentState);
            }
            else if (liftOffMillisWaitTime == 0)
            {
                liftOffMillisWaitTime = currentMillisTime + 100;
            }
        }
        else
        {
            liftOffMillisWaitTime = 0;
        }
    }

    // Check for ascent
    if (currentState == 3)
    {
        if (a.acceleration.y > -2)
        {
            if (bmp.readAltitude(BarPressure) >= CalibratedAltitude + 0.1)
            {
                currentState++;
                Serial.println("Detected ascent/engine on phase! Starting TVC.");
                Serial.print("Current State changed to: ");
                Serial.println(currentState);
            }
        }
    }

    // Check for engine off
    if (currentState == 4)
    {
        if (currentMillisTime >= engineOffMillisWaitTime)
        {
            acc2 = a.acceleration.y;
            if (acc1 > acc2)
            {
                currentState++;
                Serial.println("Engine Off phase detected. Losing acceleration.");
                Serial.print("Current State changed to: ");
                Serial.println(currentState);
            }
            else
            {
                acc1 = 0;
                acc2 = 0;
                engineOffMillisWaitTime = 3600000;
            }
        }
        else if (engineOffMillisWaitTime == 3600000)
        {
            acc1 = a.acceleration.y;
            engineOffMillisWaitTime = currentMillisTime + 100;
        }
    }

    // Check for uncontrolled descent
    if (currentState == 5)
    {
        if (currentMillisTime >= apogeeMillisWaitTime)
        {
            altitudeTwo = bmp.readAltitude(BarPressure) - CalibratedAltitude;
            if (altitudeOne > altitudeTwo)
            {
                currentState++;
                Serial.println("Apogee reached. Starting Uncontrolled descend!");
                Serial.print("Apogee recorded at ");
                Serial.print(altitudeOne);
                Serial.print("m AGL and ");
                Serial.print(altitudeOne + CalibratedAltitude);
                Serial.println("m ASL");
                Serial.print("Current State changed to: ");
                Serial.println(currentState);
            }
            else
            {
                altitudeOne = 0;
                altitudeTwo = 0;
                apogeeMillisWaitTime = 3600000;
            }
        }
        else if (apogeeMillisWaitTime == 3600000)
        {
            apogeeMillisWaitTime = currentMillisTime + 100;
            altitudeOne = bmp.readAltitude(BarPressure) - CalibratedAltitude;
        }
    }

    // Check if equation = currentTime then start second engine
    if (currentState == 6)
    {
        if (bmp.readAltitude(BarPressure) <= CalibratedAltitude + ParachutesDeployAltitude)
        {
            currentState++;
            Serial.println("Lighting second engine!");
            Serial.print("Current State changed to: ");
            Serial.println(currentState);
        }
    }

    // Check for landing
    if (currentState == 7)
    {
        if (currentMillisTime >= landedMillisWaitTime)
        {
            if (CalibratedAltitude > landedalt - 2)
            {
                currentState++;
                Serial.print("Rocket landed successfully ");
                Serial.print((millis() - liftOffMillisTime) / 1000);
                Serial.println("s after Lift off!");
                Serial.print("Current State changed to: ");
                Serial.println(currentState);
            }
            else
            {
                landedalt = bmp.readAltitude(BarPressure);
                landedMillisWaitTime = 3600000;
            }
        }
        else if (landedMillisWaitTime == 3600000)
        {
            landedalt = bmp.readAltitude(BarPressure);
            landedMillisWaitTime = currentMillisTime + 5000;
        }
    }

    return currentState;
}