#include <getState.h>

void setup()
{
  delay(2000);
  mainStartUp();
  servoCalibration();
  buzzerLED();
  sensorTesting();
  batSetup();
  pyroSetup(); // DO NOT TURN ON FOR LAUNCH COMMENT OUT !!!!!
  sdCardInit();
  radioStartup();
}

void loop()
{
  // Required functions
  getState();
  ledColor();
  if (currentState == 1) // Not full setup
  {
  }

  if (currentState == 2) // Waiting for launch
  {
    getRotData();
    allPyrosLow();
    //batVoltage();
    //slowDataLog();
  }

  if (currentState == 3) // Lift off
  {
    getRotData();
    TVCfunc();
    slowDataLog();
  }

  if (currentState == 4) // Engine on
  {
    getRotData();
    tvcStatus = 1;
    TVCfunc();
    shortDataLog();
  }

  if (currentState == 5) // Engine off
  {
    tvcStatus = 0;
    getRotData();
    shortDataLog();
  }

  if (currentState == 6) // Apogee / Uncontrolled descend
  {
    getRotData();
    shortDataLog();
  }

  if (currentState == 7) // Parachute deployment
  {
    fireParachute();
    getRotData();
    shortDataLog();
  }

  if (currentState == 8) // Landed
  {
    // After landing code
  }

  if (currentState == 9) // Data transfer to flash chip
  {
    // Nothing
  }
}
