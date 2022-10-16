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

    //Serial.print("X "), Serial.print(x), Serial.print(" Y "), Serial.print(y), Serial.print(" Z "), Serial.println(z);
    if (currentState < 3)
    {
        DefaultX = x, DefaultY = y, DefaultZ = z, CalibratedAltitude = bmp.readAltitude(BarPressure);
    }
}

void fireParachute()
{
    if (firedParachute == 0) {
        
        Serial.println("Firing parachute now - Pyro 4");
        pyro4Fire = 1;
        digitalWrite(Pyro4, HIGH);
        if (parachuteFireMillisTime == 0) {
            parachuteFireMillisTime = millis();
        }
        if (parachuteFireMillisTime + 1000 >= millis()) {
            digitalWrite(Pyro4, LOW);
            firedParachute = 1;
            pyro4Fire = 0;
        }
    }
}

char* translateChar(float val) 
{ 
    char sz[20] = {' '} ;

    int val_int = (int) val;   // compute the integer part of the float

    float val_float = (abs(val) - abs(val_int)) * 100000;

    int val_fra = (int)val_float;

  sprintf (sz, "%d.%d", val_int, val_fra); //
  
  return sz;
}

void sendRadioMessage()
{
    delay(200);

    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    int messageStatus = 0;

    RH_ASK driver(transmitSpeed, rxPin, txPin, 0);

    unsigned long timeFromTakeOff =  millis() - liftOffMillisTime;

    char* XaxisTVC = translateChar(float(CommandX));
    char* ZaxisTVC = translateChar(float(CommandZ));

    char* currentAlt = translateChar(float(bmp.readAltitude(BarPressure)));
    char* velocity =  translateChar(float(a.acceleration.y * (double)((millis() - liftOffMillisTime) / 1000)));

    char* currentAngleX = translateChar(float(x));
    char* currentAngleY = translateChar(float(y));
    char* currentAngleZ = translateChar(float(z));

    char* accelerationX = translateChar(float(a.acceleration.x));
    char* accelerationY = translateChar(float(a.acceleration.y));
    char* accelerationZ = translateChar(float(a.acceleration.z));

    char* gyroX = translateChar(float(g.gyro.x));
    char* gyroY = translateChar(float(g.gyro.y));
    char* gyroZ = translateChar(float(g.gyro.z));

    char* tvcState = translateChar(float(tvcStatus));
    char* abortState = translateChar(float(abortStatus));
    char* switchState = translateChar(float(switchStatus));

    int pyro1State = pyro1Fire;
    int pyro2State = pyro2Fire;
    int pyro3State = pyro3Fire;
    int pyro4State = pyro4Fire;


    // Combine information + change to Char*

    if (messageStatus == 0) {
        messageStatus++;

        char *msg;
        strcat(msg, XaxisTVC); strcat(msg, ZaxisTVC); strcat(msg, accelerationX); strcat(msg, accelerationY); strcat(msg, accelerationZ); strcat(msg, currentAlt); strcat(msg, velocity); strcat(msg, translateChar(float(messageStatus))); strcat(msg, translateChar(float(timeFromTakeOff)));
        driver.send((uint8_t *)msg, strlen(msg));
        driver.waitPacketSent(); 

    } else if (messageStatus == 1) {
        messageStatus++;
        char *msg;
        strcat(msg, currentAngleX); strcat(msg, currentAngleY); strcat(msg, currentAngleZ); strcat(msg, gyroX); strcat(msg, gyroY); strcat(msg, gyroZ); strcat(msg, translateChar(float(messageStatus))); strcat(msg, translateChar(float(timeFromTakeOff)));
        driver.send((uint8_t *)msg, strlen(msg));
        driver.waitPacketSent();

    } else if (messageStatus == 2) {
        char *msg;
        strcat(msg, XaxisTVC); strcat(msg, tvcState); strcat(msg, abortState); strcat(msg, switchState); strcat(msg, translateChar(float(pyro1State))); strcat(msg, translateChar(float(pyro2State))); strcat(msg, translateChar(float(pyro3State))); strcat(msg, translateChar(float(pyro4State))); strcat(msg, translateChar(float(messageStatus))); strcat(msg, translateChar(float(timeFromTakeOff)));
        driver.send((uint8_t *)msg, strlen(msg));
        driver.waitPacketSent();
        messageStatus = 0;
    }
}

void abortFunc() 
{

    // functions
    fireParachute();

    // Servo manipulation
    CommandX = 90; // Reset servo
    CommandZ = 90; // Reset servo
    Xaxis.write(CommandX); // Update servo X
    Yaxis.write(CommandZ); // Update servo Z 
}

// PID handler
double handlePID(double currentState, double targetState) 
{
 
    double currentUpdateError = targetState - currentState;
 
    // Safe Division
    long deltaTime = lastUpdateTime == -1 ? 0 : millis() - lastUpdateTime;
    double changeInError = lastUpdateError == -1 ? 0 : currentUpdateError - lastUpdateError;;
 
    // Calculate PID Terms
    double proportion = currentUpdateError * Px;
    double derivative = changeInError / deltaTime;
    integralCounter += changeInError * deltaTime;
 
    // Store values
    lastUpdateTime = millis();
    lastUpdateError = currentUpdateError;

    // Sum Values
    double sum = 0;
    sum = proportion + integralCounter + derivative; // Add PID together
 
    return sum + 90; // return + add correct servo angle

}

// Thrust Vector Control
void TVCfunc()
{
    // Data required

    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    // TVC Here

    xRight = (x - DefaultX); // Get 90 degrees as default from 270
    zRight = (z - DefaultZ);

    if (xRight >= 135 || xRight <= 45 || zRight >= 135 || zRight <= 45) { // Check if we need to abort
        currentState = 43;
        Serial.println("Abort data detected");
    }
        /*float PCmdX = xRight * Px;  // Smoother angle change
        float PCmdZ = zRight * Px;
        ICmdX = 0; //ICmdX * 0.01 + xRight; Error calculation
        ICmdZ = 0; //ICmdZ * 0.01 + zRight;
        double DCmdX = g.gyro.x * Dx; // Rotation
        double DCmdZ = g.gyro.z * Dx;
        CommandX = (PCmdX + DCmdX + (ICmdX * Ix)) * 0.1 + 90; // Add everything together for X axis
        CommandZ = (PCmdZ + DCmdZ + (ICmdZ * Ix)) * 0.1 + 90; // Add everything together for Y axis*/

        CommandX = handlePID(xRight, targetAngle);
        CommandZ = handlePID(zRight, targetAngle);

    if (CommandX < 45) // Threshold 45° for X axis
    {
         CommandX = 45;
       }
    else if (CommandX > 135) // Threshold 45° for X axis
    {
            CommandX = 135;
    }
    if (CommandZ < 45) // Threshold 45° for Z axis
    {
            CommandZ = 45;
    }
    else if (CommandZ > 135) // Threshold 45° for Z axis
    {
         CommandZ = 135;
    }
    
    Serial.print("CommandX: "), Serial.print(CommandX), Serial.print(" CommandZ: "), Serial.println(CommandZ); // Print data for debugging
    //Serial.print("DCmdX: "), Serial.print(DCmdX), Serial.print("ICmdX: "), Serial.println(ICmdX);
    Xaxis.write(CommandX); // Update servo X
    Yaxis.write(CommandZ); // Update servo Z
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

void slowDataLog()
{
    // Data required

    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    // SD card
    longDataLog = SD.open("longDataLog.txt", FILE_WRITE);
    if (longDataLog)
    {
        longDataLog.print("Millis, "), longDataLog.print(millis()), longDataLog.print(", Time from Lift off, "), longDataLog.print(millis() - liftOffMillisTime), longDataLog.print(" , TVC X axis, "), longDataLog.print(CommandX), longDataLog.print(" , TVC Y axis:, "), longDataLog.print(CommandZ);
        longDataLog.print(", Calibrated Altitude, "), longDataLog.print(CalibratedAltitude), longDataLog.print(" ,Current Altitude, "), longDataLog.print(bmp.readAltitude(BarPressure)), longDataLog.print(", AngleX , "), longDataLog.print(x), longDataLog.print(" , AngleY , "), longDataLog.print(y), longDataLog.print(" ,AngleZ , "), longDataLog.print(z), longDataLog.print(", Gyro and IMU readings, "), longDataLog.print(" Acceleration X, "), longDataLog.print(a.acceleration.x), longDataLog.print(", Y, "), longDataLog.print(a.acceleration.y), longDataLog.print(", Z, "), longDataLog.print(a.acceleration.z), longDataLog.print(" , Rotation X, "), longDataLog.print(g.gyro.x), longDataLog.print(", Y, "), longDataLog.print(g.gyro.y), longDataLog.print(", Z, "), longDataLog.print(g.gyro.z), longDataLog.print("");
        longDataLog.print(", Current State, "), longDataLog.print(currentState), longDataLog.print(", TVC X axis converted, "), longDataLog.print(CommandX - DefaultX), longDataLog.print(", TVC Y axis converted, "), longDataLog.print(CommandZ - DefaultZ), longDataLog.print(", X axis servo angle, "), longDataLog.print(Xaxis.read()), longDataLog.print(", Y axis servo angle, "), longDataLog.print(Yaxis.read()), longDataLog.print(", Velocity, "), longDataLog.print(a.acceleration.y * (double)((millis() - liftOffMillisTime) / 1000));
        longDataLog.print(", tvcStatus, "), longDataLog.print(tvcStatus), longDataLog.print(", abortStatus, "), longDataLog.print(abortStatus), longDataLog.print(", switchStatus, "), longDataLog.print(switchStatus), longDataLog.print(", pyro1Fire, "), longDataLog.print(pyro1Fire), longDataLog.print(", pyro2Fire, "), longDataLog.print(pyro2Fire), longDataLog.print(", pyro3Fire, "), longDataLog.print(pyro3Fire), longDataLog.print(", pyro4Fire, "), longDataLog.println(pyro4Fire), longDataLog.println(", DefaultX, "), longDataLog.print(DefaultX), longDataLog.println(", DefaultY, "), longDataLog.print(DefaultY), longDataLog.println(", DefaultZ, "), longDataLog.println(DefaultZ);
        //close the file:
        longDataLog.close();
    }
    else
    {
        //if the file didn't open, print an error:
        //Serial.println("error opening TVC.txt");
    }
}

void shortDataLog()
{
    // Data required

    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    // SD card
    shortDataLogSD = SD.open("shortDataLog.txt", FILE_WRITE);
    if (shortDataLogSD)
    {
        shortDataLogSD.print(", Time from Lift off, "), shortDataLogSD.print(millis() - liftOffMillisTime), shortDataLogSD.print(" ,X axis, "), shortDataLogSD.print(CommandX), shortDataLogSD.print(" ,Z axis:, "), shortDataLogSD.print(CommandZ);
        shortDataLogSD.print(" Current Altitude, "), shortDataLogSD.print(bmp.readAltitude(BarPressure)), shortDataLogSD.print(", AngleX , "), shortDataLogSD.print(x), shortDataLogSD.print(" , AngleY , "), shortDataLogSD.print(y), shortDataLogSD.print(" ,AngleZ , "), shortDataLogSD.print(z), shortDataLogSD.print(", Gyro and IMU readings, "), shortDataLogSD.print(" Acceleration X, "), shortDataLogSD.print(a.acceleration.x), shortDataLogSD.print(", Y, "), shortDataLogSD.print(a.acceleration.y), shortDataLogSD.print(", Z, "), shortDataLogSD.print(a.acceleration.z), shortDataLogSD.print(" , Rotation X, "), shortDataLogSD.print(g.gyro.x), shortDataLogSD.print(", Y, "), shortDataLogSD.print(g.gyro.y), shortDataLogSD.print(", Z, "), shortDataLogSD.print(g.gyro.z), shortDataLogSD.print("");
        shortDataLogSD.print(", Current State, "), shortDataLogSD.print(currentState), shortDataLogSD.print(", TVC X axis converted, "), shortDataLogSD.print(CommandX - DefaultX), shortDataLogSD.print(", TVC Y axis converted, "), shortDataLogSD.print(CommandZ - DefaultZ), shortDataLogSD.print(", X axis servo angle, "), shortDataLogSD.print(Xaxis.read()), shortDataLogSD.print(", Y axis servo angle, "), shortDataLogSD.print(Yaxis.read()), shortDataLogSD.print(", Velocity, "), shortDataLogSD.print(a.acceleration.y * (double)((millis() - liftOffMillisTime) / 1000));
        shortDataLogSD.print(", tvcStatus, "), shortDataLogSD.print(tvcStatus), shortDataLogSD.print(", abortStatus, "), shortDataLogSD.print(abortStatus), shortDataLogSD.print(", switchStatus, "), shortDataLogSD.print(switchStatus), shortDataLogSD.print(", pyro1Fire, "), shortDataLogSD.print(pyro1Fire), shortDataLogSD.print(", pyro2Fire, "), shortDataLogSD.print(pyro2Fire), shortDataLogSD.print(", pyro3Fire, "), shortDataLogSD.print(pyro3Fire), shortDataLogSD.print(", pyro4Fire, "), shortDataLogSD.println(pyro4Fire);
        //close the file:
        shortDataLogSD.close();
    }
    else
    {
        //if the file didn't open, print an error:
        //Serial.println("error opening shortDataLog.txt");
    }
}

void allPyrosLow()
{
    digitalWrite(Pyro1, LOW);
    digitalWrite(Pyro2, LOW);
    digitalWrite(Pyro3, LOW);
    digitalWrite(Pyro4, LOW);
}

double Vbatt;
double Vbatt_perc;
double V_R2;
double VbattMax = 4.3;
double VbattMin = 2.7;
double resolutionVoltage = 0.00107422; // resolution = AREF / 1024 = 1.1V / 1024
double R1 = 20000;
double R2 = 100000;

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