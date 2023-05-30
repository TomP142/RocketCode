// Time
int ttime = millis();
// BMP280 current barometric pressure
const float BarPressure = 1018.1;

// Flash chip

#define FLASH_CHIPSELECT 4        // adjust this to your CS pin!
#define FLASH_MEMORY_SIZE 8388608 // for W25Q64FVSSIQ which has 64Mbit = 8MBytes of storage
SPIFlash flash(FLASH_CHIPSELECT);

uint32_t currentAddress = 0;

// PID Values
double Px = 0.5;
double Ix = 0.2;
double Dx = 0.1;

double integralCounter = 0;

unsigned lastUpdateTime = -1;
double lastUpdateError = -1.0;

unsigned int defaultValue = -1;

// Trajectory table

const int MAX_STEPS = 4;

unsigned long trajectoryStartTime = 0;
int currentTrajectoryStep = 0;

double trajectorySteps[MAX_STEPS] = {0, 10, -5, 0};
double trajectoryTimings[MAX_STEPS] = {0, 1000, 1500, 1900};

// Landing trajectory
unsigned long landingTrajectoryStartTime = 0;
int landingCurrentTrajectoryStep = 0;

double landingSineAngles[MAX_STEPS] = {0, 5, -5, 0};
double landingSineTimings[MAX_STEPS] = {0, 500, 1200, 1900};
// Servo Variables

PWMServo Xaxis;
PWMServo Yaxis;
int angle = 0;
int CalibrationServo = 0;
int calibration1 = random(90);
int calibration2 = random(135);
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
int maxTVCAngle = 10;
int abortAngle = 45;

unsigned long trajectoryTiming = 0;

// SD Card
SdFat SD;
SdFile dataFile;
const size_t TRANSFER_BUFFER_SIZE = 512;

const int chipSelect = 0; // Change to SD card pin

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
unsigned long deployLegsMillisTime = 0;

unsigned long liftOffMillisWaitTime = 3600000;
unsigned long engineOffMillisWaitTime = 3600000;
unsigned long apogeeMillisWaitTime = 3600000;
unsigned long landedMillisWaitTime = 3600000;

// Safe Checks - 1 = true, 0 = false
int tvcStatus = 0;
int abortStatus = 0;
int switchStatus = 0;
int firedParachute = 0;
int deployedLegs = 0;

int pyro1Fire = 0;
int pyro2Fire = 0;
int pyro3Fire = 0;
int pyro4Fire = 0;

//  Mass Values
double initialMass = 0; // in kg

// Battery Values

double Vbatt;
double Vbatt_perc;
double V_R2;
double VbattMax = 4.3;
double VbattMin = 2.7;
double resolutionVoltage = 0.00107422; // resolution = AREF / 1024 = 1.1V / 1024
double R1 = 20000;
double R2 = 100000;
#define batPin A3 // Change to battery pin later

// Legs
double legsDeployAltitude = 5;