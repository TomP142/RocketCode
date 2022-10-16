// Libraries
#include <Wire.h>
#include <PWMServo.h>
#include <SD.h>
#include <SPI.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <string>
#include <sstream>
#include <stdlib.h>
#include <RH_ASK.h>
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
// 7 - Parachutes off
// 8 - Landed
// 9 - Data Transfer ??

// WaitTime 2.0

int getState()
{
    // 1 - Starting up
    // 2 - Ready for start
    // 3 - Launch
    // 4 - Accelerating
    // 5 - Engine Off
    // 6 - Apogee / Uncontrolled descend
    // 7 - Parachutes off
    // 8 - Landed
    // 9 - Data Transfer ??
    // 43 - Abort Landing
    // 44 -
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

    if (currentState == 2)
    {
        { // Check if we didn't take off
            if (a.acceleration.y > 0)
            { // If we have enough acceleration for liftoff
                if (currentMillisTime >= liftOffMillisWaitTime)
                { // Check if we are accelerating for longer period of time
                    currentState++;
                    Serial.println("Lift off!");
                    liftOffMillisTime = millis();
                    Serial.print("Calibrated Altitude: ");
                    Serial.println(CalibratedAltitude);
                    Serial.print("Current State changed to: ");
                    Serial.println(currentState);
                    return currentState;
                }
                else if (liftOffMillisWaitTime == 0)
                {
                    liftOffMillisWaitTime = currentMillisTime + 300; // Set how much time the rocket must accelerate to sense lift off.
                }
            }
            else
            {
                liftOffMillisWaitTime = 0; // Reset timer if acceleration lower than expected
            }
        }
    }

    // Take off phase detected

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
                return currentState;
            }
        }
    }
    // Engine Off phase

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
                return currentState;
            }
            else // Acceleration is still increasing. Reset everything
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

    // Uncontrolled descend phase
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
                return currentState;
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

    // Parachute deployment phase

    if (currentState == 6)
    {
        if (bmp.readAltitude(BarPressure) <= CalibratedAltitude + ParachutesDeployAltitude)
        {
            currentState++;
            Serial.println("Parachutes deployment!");
            Serial.print("Current State changed to: ");
            Serial.println(currentState);
            return currentState;
        }
    }

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
                return currentState;
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
    return 0;
}
