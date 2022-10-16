// Startup

void mainStartUp()
{

    // MPU6050 startup
    Serial.begin(9600);
    Wire.begin();
    Wire.beginTransmission(MPU_addr);
    Wire.write(0x6B);
    Wire.write(0);
    Wire.endTransmission(true);

    Serial.begin(9600);
    bmp.begin();
    mpu.begin();
    Serial.println("");

    // Servo setup
    Serial.println("Servo Setup");

    Xaxis.attach(3);
    Yaxis.attach(4);
    Xaxis.write(0);
    Yaxis.write(0);
}

RH_ASK getDriver()
{
    return driver;
}

void radioStartup()
{
    Serial.println("Starting up Radio connection");
    RH_ASK driver(transmitSpeed, rxPin, txPin, 0);
    if (!driver.init())
#ifdef RH_HAVE_SERIAL
        Serial.println("RF init failed");
#else
        ;
#endif
}

// Calibration
void servoCalibration()
{
    Xaxis.write(calibration1);
    Yaxis.write(calibration1);
    if (Xaxis.read() == calibration1)
    {
        CalibrationServo += 1;
    }
    if (Yaxis.read() == calibration1)
    {
        CalibrationServo += 1;
    }

    delay(150);
    Xaxis.write(calibration2);
    Yaxis.write(calibration2);
    if (Xaxis.read() == calibration2)
    {
        CalibrationServo += 1;
    }
    if (Yaxis.read() == calibration2)
    {
        CalibrationServo += 1;
    }

    delay(150);
    Xaxis.write(calibration3);
    Yaxis.write(calibration3);
    if (Xaxis.read() == calibration3)
    {
        CalibrationServo += 1;
    }
    if (Yaxis.read() == calibration3)
    {
        CalibrationServo += 1;
    }

    delay(150);
    Xaxis.write(90);
    Yaxis.write(90);
    if (Xaxis.read() == 90)
    {
        CalibrationServo += 1;
    }
    if (Yaxis.read() == 90)
    {
        CalibrationServo += 1;
    }

    if (CalibrationServo == 8)
    {
        Serial.print("Servo calibrated correctly. Using angles: ");
        Serial.print(calibration1);
        Serial.print("° | ");
        Serial.print(calibration2);
        Serial.print("° | ");
        Serial.print(calibration3);
        Serial.print("°");
        Serial.println("");
        currentState++;
        Serial.print("Current State changed to: ");
        Serial.println(currentState);
    }
    else if (CalibrationServo != 8)
    {
        Serial.println("Error: Servo not calibrated properly!");
        Serial.println("");
    }
}
// Servo startup complete

// LED Test
void buzzerLED()
{
    pinMode(R_LED, OUTPUT);
    pinMode(G_LED, OUTPUT);
    pinMode(B_LED, OUTPUT);
    Serial.println("Testing LEDs");
    digitalWrite(R_LED, LOW);
    digitalWrite(G_LED, HIGH);
    digitalWrite(B_LED, HIGH);
    delay(1000);
    digitalWrite(R_LED, HIGH);
    digitalWrite(G_LED, LOW);
    delay(1000);
    digitalWrite(G_LED, HIGH);
    digitalWrite(B_LED, LOW);
    delay(1000);
    digitalWrite(B_LED, HIGH);
    Serial.println("Tested LEDs");

    // Buzzer test
    Serial.println("Testing Buzzer");
    tone(Buzzer, 2000);
    delay(50);
    noTone(Buzzer);
    delay(75);
    tone(Buzzer, 2000);
    delay(50);
    noTone(Buzzer);
    delay(200);
    tone(Buzzer, 1000);
    delay(50);
    noTone(Buzzer);
    delay(75);
    tone(Buzzer, 1000);
    delay(50);
    noTone(Buzzer);
    delay(400);

    tone(Buzzer, 1319);
    delay(50);
    noTone(Buzzer);
    delay(50);
    tone(Buzzer, 1760);
    delay(50);
    noTone(Buzzer);
    delay(50);
    tone(Buzzer, 2217);
    delay(50);
    noTone(Buzzer);
    delay(50);
    tone(Buzzer, 2637);
    delay(100);
    noTone(Buzzer);
    delay(100);
    tone(Buzzer, 2217);
    delay(50);
    noTone(Buzzer);
    delay(50);
    tone(Buzzer, 2637);
    delay(200);
    noTone(Buzzer);
    delay(400);
    Serial.println("Tested Buzzer");
}
// Testing MPU6050 (IMU)
void sensorTesting()
{
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    // Print IMU Values
    Serial.print("Acceleration X: ");
    Serial.print(a.acceleration.x);
    delay(100);
    Serial.print(", Y: ");
    Serial.print(a.acceleration.y);
    delay(100);
    Serial.print(", Z: ");
    Serial.print(a.acceleration.z);
    Serial.println(" m/s^2");
    delay(100);

    Serial.print("Rotation X: ");
    Serial.print(g.gyro.x);
    delay(100);
    Serial.print(", Y: ");
    Serial.print(g.gyro.y);
    delay(100);
    Serial.print(", Z: ");
    Serial.print(g.gyro.z);
    Serial.println(" rad/s");
    delay(100);

    Wire.beginTransmission(MPU_addr);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_addr, 14, true);
    AcX = Wire.read() << 8 | Wire.read();
    AcY = Wire.read() << 8 | Wire.read();
    AcZ = Wire.read() << 8 | Wire.read();
    int xAng = map(AcX, minVal, maxVal, -90, 90);
    int yAng = map(AcY, minVal, maxVal, -90, 90);
    int zAng = map(AcZ, minVal, maxVal, -90, 90);

    x = RAD_TO_DEG * (atan2(-yAng, -zAng) + PI);
    y = RAD_TO_DEG * (atan2(-xAng, -zAng) + PI);
    z = RAD_TO_DEG * (atan2(-yAng, -xAng) + PI);

    Serial.print("AngleX= ");
    Serial.println(x);

    Serial.print("AngleY= ");
    Serial.println(y);

    Serial.print("AngleZ= ");
    Serial.println(z);
    Serial.println("");
    delay(400);

    Serial.println("Tested MPU6050 (IMU)");
    delay(1000);
    // BMP280
    Serial.println("Testing BMP280 (Barometer)");
    bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                    Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                    Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                    Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                    Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */

    Serial.print(F("Temperature = "));
    Serial.print(bmp.readTemperature());
    Serial.println(" *C");
    delay(200);
    Serial.print(F("Pressure = "));
    Serial.print(bmp.readPressure());
    Serial.println(" Pa");
    delay(200);
    Serial.print(F("Approx altitude = "));
    Serial.print(bmp.readAltitude(BarPressure)); /* Adjusted to local forecast! */
    Serial.println(" m");
    delay(200);
}

// Pyros

void pyroSetup()
{
    Serial.println("Pyro 1 HIGH!");
    digitalWrite(Pyro1, HIGH);
    delay(1000);
    digitalWrite(Pyro1, LOW);
    delay(1000);
    Serial.println("Pyro 2 HIGH!");
    digitalWrite(Pyro2, HIGH);
    delay(1000);
    digitalWrite(Pyro2, LOW);
    delay(1000);
    Serial.println("Pyro 3 HIGH!");
    digitalWrite(Pyro3, HIGH);
    delay(1000);
    digitalWrite(Pyro3, LOW);
    delay(1000);
    Serial.println("Pyro 4 HIGH!");
    digitalWrite(Pyro4, HIGH);
    delay(1000);
    digitalWrite(Pyro4, LOW);
    delay(1000);
    Serial.println("All pyros completed!");
}

// SD Card

void sdCardInit()
{
    File TVC;
    const int chipSelect = 0;
    Serial.println("Initializing SD card...");

    if (!SD.begin(chipSelect))
    {
        Serial.println("SD card not found");
        currentState++;
        Serial.print("Current State changed to: ");
        Serial.println(currentState);
        return;
    }
    Serial.println("SD card setup done.");
    for (int i = 0; i <= 10; i++)
    {
        digitalWrite(B_LED, LOW);
        delay(300);
        digitalWrite(B_LED, HIGH);
        delay(300);
    }
    Serial.println("Startup complete!");
    currentState++;
    Serial.print("Current State changed to: ");
    Serial.println(currentState);
}