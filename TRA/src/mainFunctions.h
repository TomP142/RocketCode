// Vars
double xRight;
double zRight;

// Main Functions

void getRotData()
{
    bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                    Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                    Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                    Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                    Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */

    // MPU6050

    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    Wire.beginTransmission(MPU_addr);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom((uint8_t)MPU_addr, (uint8_t)14, (uint8_t) true); // Explicitly cast the arguments
    AcX = Wire.read() << 8 | Wire.read();
    AcY = Wire.read() << 8 | Wire.read();
    AcZ = Wire.read() << 8 | Wire.read();
    int xAng = map(AcX, minVal, maxVal, -90, 90);
    int yAng = map(AcY, minVal, maxVal, -90, 90);
    int zAng = map(AcZ, minVal, maxVal, -90, 90);

    x = RAD_TO_DEG * (atan2(-yAng, -zAng) + PI);
    y = RAD_TO_DEG * (atan2(-xAng, -zAng) + PI);
    z = RAD_TO_DEG * (atan2(-yAng, -xAng) + PI);

    // Serial.print("X "), Serial.print(x), Serial.print(" Y "), Serial.print(y), Serial.print(" Z "), Serial.println(z);
    if (currentState < 3)
    {
        DefaultX = x, DefaultY = y, DefaultZ = z, CalibratedAltitude = bmp.readAltitude(BarPressure);
    }
}

void fireParachute()
{
    if (firedParachute == 0)
    {

        Serial.println("Firing parachute now - Pyro 4");
        pyro4Fire = 1;
        digitalWrite(Pyro4, HIGH);
        if (parachuteFireMillisTime == 0)
        {
            parachuteFireMillisTime = millis();
        }
        if (parachuteFireMillisTime + 1000 >= millis())
        {
            digitalWrite(Pyro4, LOW);
            firedParachute = 1;
            pyro4Fire = 0;
        }
    }
}

char *translateChar(float val)
{
    char *sz = (char *)malloc(20 * sizeof(char)); // dynamically allocate memory

    if (sz == NULL)
    {
        // malloc failed, handle error
        return NULL;
    }

    int val_int = (int)val; // compute the integer part of the float
    float val_float = (abs(val) - abs(val_int)) * 100000;
    int val_fra = (int)val_float;

    sprintf(sz, "%d.%d", val_int, val_fra); //

    return sz;
}

void sendRadioMessage()
{
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    RH_ASK driver(transmitSpeed, rxPin, txPin, 0);

    unsigned long timeFromTakeOff = millis() - liftOffMillisTime;

    char *data[] = {
        translateChar(float(timeFromTakeOff)),
        translateChar(float(CommandX)),
        translateChar(float(CommandZ)),
        translateChar(float(bmp.readAltitude(BarPressure))),
        translateChar(float(a.acceleration.y * (double)((millis() - liftOffMillisTime) / 1000))),
        translateChar(float(a.acceleration.x)),
        translateChar(float(a.acceleration.y)),
        translateChar(float(a.acceleration.z)),
        translateChar(float(x)),
        translateChar(float(y)),
        translateChar(float(z)),
        translateChar(float(g.gyro.x)),
        translateChar(float(g.gyro.y)),
        translateChar(float(g.gyro.z)),
        translateChar(float(CommandX)),
        translateChar(float(tvcStatus)),
        translateChar(float(abortStatus)),
        translateChar(float(switchStatus)),
        translateChar(float(pyro1Fire)),
        translateChar(float(pyro2Fire)),
        translateChar(float(pyro3Fire)),
        translateChar(float(pyro4Fire))};

    char *msg = (char *)malloc(sizeof(char) * 1024); // ensure this size is sufficient
    strcpy(msg, data[0]);

    for (size_t i = 1; i < sizeof(data) / sizeof(char *); i++)
    {
        strcat(msg, data[i]);
        free(data[i]);
    }

    driver.send((uint8_t *)msg, strlen(msg));
    driver.waitPacketSent();

    free(msg);
}

void deployLegs()
{
    if (deployedLegs == 0)
    {

        Serial.println("Deploying legs - Pyro 1");
        pyro1Fire = 1;
        digitalWrite(Pyro1, HIGH);
        if (deployLegsMillisTime == 0)
        {
            deployLegsMillisTime = millis();
        }
        if (deployLegsMillisTime + 1000 >= millis())
        {
            digitalWrite(Pyro1, LOW);
            deployedLegs = 1;
            pyro1Fire = 0;
        }
    }
}

void abortFunc()
{

    // functions
    fireParachute();

    // Servo manipulation
    CommandX = 90;         // Reset servo
    CommandZ = 90;         // Reset servo
    Xaxis.write(CommandX); // Update servo X
    Yaxis.write(CommandZ); // Update servo Z
}

// PID handler
double handlePID(double currentState, double targetState)
{
    double currentUpdateError = targetState - currentState;

    // Safe Division
    long deltaTime = lastUpdateTime == defaultValue ? 1 : (millis() - lastUpdateTime) / 1000.0; // Convert to seconds
    double changeInError = lastUpdateError == -1 ? 0 : currentUpdateError - lastUpdateError;

    // Calculate PID Terms
    double proportion = currentUpdateError;
    double derivative = changeInError / deltaTime;
    integralCounter += changeInError * deltaTime;

    // Store values
    lastUpdateTime = millis();
    lastUpdateError = currentUpdateError;

    return proportion * Px + integralCounter * Ix + derivative * Dx + 90; // add PID together + add correct servo angle
}
// Thrust Vector Control
// Ensures the command is within the given angle limits
double constrainCommand(double command, double maxAngle)
{
    if (command < 90 - maxAngle)
    {
        return 90 - maxAngle;
    }
    else if (command > 90 + maxAngle)
    {
        return 90 + maxAngle;
    }
    return command;
}

double ascentTargetAngleHandler(int orientation)
{
    if (orientation == 0)
    {
        if (currentTrajectoryStep == 0)
        {
            trajectoryStartTime = millis();
            currentTrajectoryStep++;
        }
        else if (currentTrajectoryStep <= MAX_STEPS)
        {
            if (millis() >= trajectoryStartTime + trajectoryTimings[currentTrajectoryStep - 1])
            {
                currentTrajectoryStep++;
            }
            return trajectorySteps[currentTrajectoryStep - 1];
        }
    }
    if (orientation == 1)
    {
        return 0.0;
    }

    return 0.0;
}

void TVCfunc()
{
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    xRight = (x - DefaultX);
    zRight = (z - DefaultZ);

    if (abs(xRight - 90) >= abortAngle || abs(zRight - 90) >= abortAngle)
    {
        currentState = 43;
        Serial.println("Abort data detected");
        return;
    }

    CommandX = handlePID(xRight, ascentTargetAngleHandler(0));
    CommandZ = handlePID(zRight, ascentTargetAngleHandler(1));

    CommandX = constrainCommand(CommandX, maxTVCAngle);
    CommandZ = constrainCommand(CommandZ, maxTVCAngle);

    Serial.print("CommandX: "), Serial.print(CommandX), Serial.print(" CommandZ: "), Serial.println(CommandZ);

    Xaxis.write(CommandX);
    Yaxis.write(CommandZ);
}

// Landing

double landingTargetAngleHandler(int orientation)
{
    if (orientation == 0)
    {
        if (landingTrajectoryStartTime == 0)
        {
            landingCurrentTrajectoryStep = millis();
            landingTrajectoryStartTime++;
        }
        else if (landingTrajectoryStartTime <= MAX_STEPS)
        {
            if (millis() >= landingCurrentTrajectoryStep + landingSineTimings[landingTrajectoryStartTime - 1])
            {
                landingTrajectoryStartTime++;
            }
            return landingSineAngles[landingTrajectoryStartTime - 1];
        }
    }
    if (orientation == 1)
    {
        return 0.0;
    }

    return 0.0;
}

void landingFunc()
{
    // landing logic

    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    xRight = (x - DefaultX);
    zRight = (z - DefaultZ);

    if (abs(xRight - 90) >= abortAngle || abs(zRight - 90) >= abortAngle)
    {
        currentState = 43;
        Serial.println("Abort data detected");
        return;
    }

    CommandX = handlePID(xRight, landingTargetAngleHandler(0));
    CommandZ = handlePID(zRight, landingTargetAngleHandler(1));

    CommandX = constrainCommand(CommandX, maxTVCAngle);
    CommandZ = constrainCommand(CommandZ, maxTVCAngle);

    Serial.print("CommandX: "), Serial.print(CommandX), Serial.print(" CommandZ: "), Serial.println(CommandZ);

    Xaxis.write(CommandX);
    Yaxis.write(CommandZ);

    // close enough to ground to deploy legs
    if (bmp.readAltitude(BarPressure) <= legsDeployAltitude)
    {
        deployLegs();
    }
}

void ledColor()
{
    if (currentState == 1)
    {
        digitalWrite(R_LED, HIGH), digitalWrite(G_LED, LOW), digitalWrite(B_LED, HIGH);
    }
    if (currentState == 2)
    {
        digitalWrite(R_LED, HIGH), digitalWrite(G_LED, HIGH), digitalWrite(B_LED, LOW);
    }
    if (currentState == 3)
    {
        digitalWrite(R_LED, LOW), digitalWrite(G_LED, HIGH), digitalWrite(B_LED, HIGH);
    }
    if (currentState == 4)
    {
        digitalWrite(R_LED, LOW), digitalWrite(G_LED, LOW), digitalWrite(B_LED, HIGH);
    }
    if (currentState == 5)
    {
        digitalWrite(R_LED, LOW), digitalWrite(G_LED, HIGH), digitalWrite(B_LED, LOW);
    }
    if (currentState == 6)
    {
        digitalWrite(R_LED, HIGH), digitalWrite(G_LED, LOW), digitalWrite(B_LED, LOW);
    }
    if (currentState == 7)
    {
        digitalWrite(R_LED, HIGH), digitalWrite(G_LED, LOW), digitalWrite(B_LED, HIGH);
    }
    if (currentState == 8)
    {
        digitalWrite(R_LED, LOW), digitalWrite(G_LED, HIGH), digitalWrite(B_LED, HIGH);
        tone(Buzzer, 500);
        delay(50);
        noTone(Buzzer);
        delay(50);
    }
    if (currentState == 9)
    {
        digitalWrite(R_LED, LOW), digitalWrite(G_LED, HIGH), digitalWrite(B_LED, HIGH);
        tone(Buzzer, 500);
        delay(50);
        noTone(Buzzer);
        delay(50);
    }
    if (currentState == 43)
    {
        digitalWrite(R_LED, LOW), digitalWrite(G_LED, HIGH), digitalWrite(B_LED, HIGH);
        tone(Buzzer, 800);
        delay(50);
        noTone(Buzzer);
        delay(100);
    }
}

/*void slowDataLog()
{
    // Data required
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    const char *filename = "longDataLog.txt";

    if (SD.exists(filename))
    {
        longDataLog.open(filename, O_RDWR | O_APPEND);
    }
    else
    {
        longDataLog.open(filename, O_CREAT | O_RDWR | O_APPEND);
    }

    if (longDataLog.isOpen())
    {
        longDataLog.print("Millis, ");
        longDataLog.print(millis());
        longDataLog.print(", Time from Lift off, ");
        longDataLog.print(millis() - liftOffMillisTime);
        longDataLog.print(" , TVC X axis, ");
        longDataLog.print(CommandX);
        longDataLog.print(" , TVC Y axis, ");
        longDataLog.print(CommandZ);
        longDataLog.print(", Calibrated Altitude, ");
        longDataLog.print(CalibratedAltitude);
        longDataLog.print(" ,Current Altitude, ");
        longDataLog.print(bmp.readAltitude(BarPressure));
        longDataLog.print(", AngleX , ");
        longDataLog.print(x);
        longDataLog.print(" , AngleY , ");
        longDataLog.print(y);
        longDataLog.print(" ,AngleZ , ");
        longDataLog.print(z);
        longDataLog.print(", Gyro and IMU readings, ");
        longDataLog.print(" Acceleration X, ");
        longDataLog.print(a.acceleration.x);
        longDataLog.print(", Y, ");
        longDataLog.print(a.acceleration.y);
        longDataLog.print(", Z, ");
        longDataLog.print(a.acceleration.z);
        longDataLog.print(" , Rotation X, ");
        longDataLog.print(g.gyro.x);
        longDataLog.print(", Y, ");
        longDataLog.print(g.gyro.y);
        longDataLog.print(", Z, ");
        longDataLog.print(g.gyro.z);
        longDataLog.print("");
        longDataLog.print(", Current State, ");
        longDataLog.print(currentState);
        longDataLog.print(", TVC X axis converted, ");
        longDataLog.print(CommandX - DefaultX);
        longDataLog.print(", TVC Y axis converted, ");
        longDataLog.print(CommandZ - DefaultZ);
        longDataLog.print(", X axis servo angle, ");
        longDataLog.print(Xaxis.read());
        longDataLog.print(", Y axis servo angle, ");
        longDataLog.print(Yaxis.read());
        longDataLog.print(", Velocity, ");
        longDataLog.print(a.acceleration.y * (double)((millis() - liftOffMillisTime) / 1000));
        longDataLog.print(", tvcStatus, ");
        longDataLog.print(tvcStatus);
        longDataLog.print(", abortStatus, ");
        longDataLog.print(abortStatus);
        longDataLog.print(", switchStatus, ");
        longDataLog.print(switchStatus);
        longDataLog.print(", pyro1Fire, ");
        longDataLog.print(pyro1Fire);
        longDataLog.print(", pyro2Fire, ");
        longDataLog.print(pyro2Fire);
        longDataLog.print(", pyro3Fire, ");
        longDataLog.print(pyro3Fire);
        longDataLog.print(", pyro4Fire, ");
        longDataLog.println(pyro4Fire);
        longDataLog.println(", DefaultX, ");
        longDataLog.print(DefaultX);
        longDataLog.println(", DefaultY, ");
        longDataLog.print(DefaultY);
        longDataLog.println(", DefaultZ, ");
        longDataLog.println(DefaultZ);

        // close the file:
        longDataLog.close();
    }
    else
    {
        // if the file didn't open, print an error:
        Serial.println("Error opening longDataLog.txt");
    }
}*/

void transferDataFromFlashToSD()
{
    // Check if there is any data stored in flash memory
    if (currentAddress > 0)
    {
        const char *filename = "longDataLog.txt";

        // Open the SD card file in append mode

        if (SD.exists(filename))
        {
            dataFile.open(filename, O_RDWR | O_APPEND);
        }
        else
        {
            dataFile.open(filename, O_CREAT | O_RDWR | O_APPEND);
        }

        if (dataFile.isOpen())
        {
            // Read the data from flash memory and transfer it to the SD card
            byte dataBuffer[TRANSFER_BUFFER_SIZE];
            int bytesRead = 0;
            unsigned int totalBytesRead = 0;

            while (totalBytesRead < static_cast<unsigned int>(currentAddress))
            {
                // Calculate the number of bytes to read in this iteration
                unsigned int remainingBytes = static_cast<unsigned int>(currentAddress) - totalBytesRead;
                int bytesToRead = min(static_cast<int>(remainingBytes), TRANSFER_BUFFER_SIZE);

                // Read the data from flash memory
                flash.readBytes(totalBytesRead, dataBuffer, bytesToRead);

                // Write the data to the SD card
                dataFile.write(dataBuffer, bytesToRead);

                // Update the counters
                bytesRead = bytesToRead;
                totalBytesRead += bytesRead;
            }

            // Close the SD card file
            dataFile.close();
        }
        else
        {
            Serial.println("Error opening SD card file for data transfer!");
        }
    }
    else
    {
        Serial.println("No data available in flash memory for transfer!");
    }
}

void shortDataLog()
{
    // Data required
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    // Buffer for the log string
    String log;

    // Prepare the log string
    log += "Millis, ";
    log += millis();
    log += ", Time from Lift off, ";
    log += (millis() - liftOffMillisTime);
    log += " , TVC X axis, ";
    log += CommandX;
    log += " , TVC Y axis, ";
    log += CommandZ;
    log += ", Calibrated Altitude, ";
    log += CalibratedAltitude;
    log += " , Current Altitude, ";
    log += bmp.readAltitude(BarPressure);
    log += ", AngleX, ";
    log += x;
    log += " , AngleY, ";
    log += y;
    log += " , AngleZ, ";
    log += z;
    log += ", Acceleration X, ";
    log += a.acceleration.x;
    log += ", Y, ";
    log += a.acceleration.y;
    log += ", Z, ";
    log += a.acceleration.z;
    log += " , Rotation X, ";
    log += g.gyro.x;
    log += ", Y, ";
    log += g.gyro.y;
    log += ", Z, ";
    log += g.gyro.z;
    log += ", Current State, ";
    log += currentState;
    log += ", TVC X axis converted, ";
    log += (CommandX - DefaultX);
    log += ", TVC Y axis converted, ";
    log += (CommandZ - DefaultZ);
    log += ", X axis servo angle, ";
    log += Xaxis.read();
    log += ", Y axis servo angle, ";
    log += Yaxis.read();
    log += ", Velocity, ";
    log += (a.acceleration.y * ((double)(millis() - liftOffMillisTime) / 1000));
    log += ", tvcStatus, ";
    log += tvcStatus;
    log += ", abortStatus, ";
    log += abortStatus;
    log += ", switchStatus, ";
    log += switchStatus;
    log += ", pyro1Fire, ";
    log += pyro1Fire;
    log += ", pyro2Fire, ";
    log += pyro2Fire;
    log += ", pyro3Fire, ";
    log += pyro3Fire;
    log += ", pyro4Fire, ";
    log += pyro4Fire;
    log += ", DefaultX, ";
    log += DefaultX;
    log += ", DefaultY, ";
    log += DefaultY;
    log += ", DefaultZ, ";
    log += DefaultZ;

    // Convert the log string to a char array
    int strLen = log.length() + 1;
    char logArray[strLen];
    log.toCharArray(logArray, strLen);

    // Check if there is enough memory left for the new log
    if (currentAddress + strLen <= FLASH_MEMORY_SIZE)
    {
        // Save the log to flash
        flash.writeBytes(currentAddress, (byte *)logArray, strLen);

        // Verify written data
        byte verifyArray[strLen];
        flash.readBytes(currentAddress, verifyArray, strLen);

        // Compare written data with original data
        if (memcmp(logArray, verifyArray, strLen) != 0)
        {
            Serial.println("Failed to write to flash!");
        }

        // Move to the next address for the next log
        currentAddress += strLen;
    }
    else
    {
        // Handle the case where the flash memory is full
        Serial.println("Flash memory is full, could not write log!");
    }
}
void allPyrosLow()
{
    digitalWrite(Pyro1, LOW);
    digitalWrite(Pyro2, LOW);
    digitalWrite(Pyro3, LOW);
    digitalWrite(Pyro4, LOW);
}

void batSetup()
{
    pinMode(batPin, INPUT);
    analogReference(INTERNAL);
}

void batVoltage()
{
    V_R2 = analogRead(batPin) * resolutionVoltage;
    Vbatt = (V_R2) * (((R1 + R2) / R1));
    Vbatt_perc = 100 * (Vbatt - VbattMin) / (VbattMax - VbattMin);
    Serial.print(Vbatt);
    Serial.print("\t");
    Serial.print(V_R2);
    Serial.print("\t");
    Serial.println(analogRead(batPin));
}
