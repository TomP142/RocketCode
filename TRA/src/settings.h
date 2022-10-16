// Time
int ttime = millis();
// BMP280 current barometric pressure
const float BarPressure = 1018.1;

// PID Values
double Px = 0.5;
double Ix = 0.2;
double Dx = 0.1;
 
double integralCounter = 0;
 
unsigned lastUpdateTime = -1;
double lastUpdateError = -1.0;

double targetAngle = 0.0;


// Servo Variables

PWMServo Xaxis;
PWMServo Yaxis;
int angle = 0;
int CalibrationServo = 0;
int calibration1 = random(180);
int calibration2 = random(180);
int calibration3 = random(180);
// Led and buzzer
int R_LED = 9;
int G_LED = 2;
int B_LED = 6;
int Buzzer = 10;

// TVC Values

int CommandX = 0;
int CommandZ = 0;

// Radio
int txPin = 11;
int rxPin = 12;
int transmitSpeed = 2000;
RH_ASK driver;

// BMP280
Adafruit_BMP280 bmp;
#define BMP_SCK (13)
#define BMP_MISO (12)
#define BMP_MOSI (11)
#define BMP_CS (10)

// MPU6050 IMU
Adafruit_MPU6050 mpu;
const int MPU_addr = 0x68;
int16_t AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ;
int minVal = 265;
int maxVal = 402;

double x;
double y;
double z;
float ICmdX;
float ICmdZ;

// Pyros

int Pyro1 = 20;
int Pyro2 = 21;
int Pyro3 = 22;
int Pyro4 = 23;

// State
int currentState = 0;

// Default values
double DefaultX = 0;
double DefaultY = 0;
double DefaultZ = 0;

// Variables
int acceleration = 0;

// SD CARD
File longDataLog;
File shortDataLogSD;
const int chipSelect = 0;

// Something
double CalibratedAltitude = 0;
double altitudeOne = 0;
double altitudeTwo = 0;

int acc1 = 0;
int acc2 = 0;

double landedalt = 0;
double maxalt = 0;
double minalt = 0;

int ParachutesDeployAltitude = 30;

unsigned long liftOffMillisTime = 0;
unsigned long currentMillisTime = 0;
unsigned long parachuteFireMillisTime = 0;

unsigned long liftOffMillisWaitTime = 3600000;
unsigned long engineOffMillisWaitTime = 3600000;
unsigned long apogeeMillisWaitTime = 3600000;
unsigned long landedMillisWaitTime = 3600000;

// Safe Checks
int tvcStatus = 0;
int abortStatus = 0;
int switchStatus = 0;
int firedParachute = 0;

int pyro1Fire = 0;
int pyro2Fire = 0;
int pyro3Fire = 0;
int pyro4Fire = 0;

//  Mass Values
double initialMass = 0; 

#define batPin A3 // Change to battery pin later