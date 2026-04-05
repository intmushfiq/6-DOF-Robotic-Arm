#include <Wire.h>
#include <MPU6050.h>
#include <QMC5883LCompass.h>
#include <Adafruit_PWMServoDriver.h>
#include <math.h>

//////////////////////////////////////////////////////////
// OBJECTS
//////////////////////////////////////////////////////////

MPU6050 mpuShoulder;
MPU6050 mpuElbow;
MPU6050 mpuWrist;

QMC5883LCompass compass;

Adafruit_PWMServoDriver pwm =
    Adafruit_PWMServoDriver();

//////////////////////////////////////////////////////////
// MUX SETTINGS
//////////////////////////////////////////////////////////

#define TCAADDR 0x70

void tselect(uint8_t i)
{
  if (i > 7) return;

  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);
  Wire.endTransmission();
delayMicroseconds(200);
}

//////////////////////////////////////////////////////////
// PWM CHANNELS
//////////////////////////////////////////////////////////

#define SHOULDER_SERVO1 0
#define SHOULDER_SERVO2 1
#define ELBOW_SERVO     2
#define WRIST_SERVO     3
#define GRIP_SERVO      4
#define BASE_SERVO      5

#define SERVOMIN 125
#define SERVOMAX 575

//////////////////////////////////////////////////////////
// BUFFERS
//////////////////////////////////////////////////////////

#define N 5

float pitchShoulderBuf[N];
float sidewaysBuf[N];
float pitchElbowBuf[N];

int idx = 0;

//////////////////////////////////////////////////////////
// WRIST FILTER
//////////////////////////////////////////////////////////

#define WINDOW_SIZE 5
#define WRIST_DEADBAND 2

float rollWindow[WINDOW_SIZE];
int indexWindow = 0;
float sumRoll = 0;
float previousFilteredRoll = 0;
bool targetSet = false;
//////////////////////////////////////////////////////////
// HALL SENSOR
//////////////////////////////////////////////////////////

const int hallPin = A0;

int farValue  = 545;
int nearValue = 877;

int openAngle  = 0; // Angle when gripper is open
int closeAngle = 40; // Angle when gripper is closed

const int samples = 10;

float alpha = 0.2;

int gripDeadband = 0;

float filteredValue = 0;
int lastGripAngle = 0;

//////////////////////////////////////////////////////////
// COMPASS VARIABLES
//////////////////////////////////////////////////////////

int currentHeading = 0;
int targetHeading = 0;
float baseServoAngle = 90;

const int numReadings = 5;
int headingBuffer[numReadings];
int bufferIndex = 0;

const int deadZone = 5;

//////////////////////////////////////////////////////////
// UTILITY FUNCTIONS
//////////////////////////////////////////////////////////

int angleToPWM(float angle)
{
  angle =
      constrain(angle, -90, 90);

  return map(
      angle,
      -90,
      90,
      SERVOMIN,
      SERVOMAX
  );
}

float average(float *arr)
{
  float sum = 0;

  for (int i = 0; i < N; i++)
      sum += arr[i];

  return sum / N;
}

//////////////////////////////////////////////////////////
// MPU READ
//////////////////////////////////////////////////////////

bool readMPU(
    MPU6050 &mpu,
    float &pitch1,
    float &pitch2,
    float &sideways)
{
  int16_t ax, ay, az;

  mpu.getAcceleration(
      &ax,
      &ay,
      &az
  );

  if (ax == 0 && ay == 0 && az == 0)
      return false;

  float ax_g = ax / 16384.0;
  float ay_g = ay / 16384.0;
  float az_g = az / 16384.0;

  pitch1 =
      atan2(
          ax_g,
          sqrt(
              ay_g * ay_g +
              az_g * az_g))
      * 180.0 / PI;

  pitch2 =
      atan2(
          ay_g,
          sqrt(
              ax_g * ax_g +
              az_g * az_g))
      * 180.0 / PI;

  sideways =
      atan2(
          az_g,
          sqrt(
              ax_g * ax_g +
              ay_g * ay_g))
      * 180.0 / PI;

  return true;
}

// ---------------- I2C RECOVERY ----------------

void checkI2C()
{
  if (Wire.getWireTimeoutFlag())
  {
    Serial.println("I2C timeout detected");

    Wire.clearWireTimeoutFlag();
  }
}

//////////////////////////////////////////////////////////
// HALL FILTER
//////////////////////////////////////////////////////////

int readMovingAverage()
{
  long total = 0;

  for (int i = 0; i < samples; i++)
  {
      total += analogRead(hallPin);
      delay(2);
  }

  return total / samples;
}

//////////////////////////////////////////////////////////
// COMPASS FUNCTIONS
//////////////////////////////////////////////////////////
float getAzimuth(float roll, float pitch)
{
    float roll_rad =
      roll * PI / 180.0;

    float pitch_rad =
        -pitch * PI / 180.0;

    tselect(6);

    compass.read();

    float Mx = compass.getX();
    float My = compass.getY();
    float Mz = compass.getZ();

    float Xh =
        Mx * cos(pitch_rad)
        + Mz * sin(pitch_rad);

    float Yh =
        Mx * sin(roll_rad) * sin(pitch_rad)
        + My * cos(roll_rad)
        - Mz * sin(roll_rad) * cos(pitch_rad);

    float azimuth =
        atan2(Yh, Xh);

    azimuth =
        azimuth * 180.0 / PI;

    if (azimuth < 0)
        azimuth += 360;

    return azimuth;
}






int getSmoothedHeading(float roll, float pitch)
{
  int rawHeading =
      getAzimuth(roll, pitch);

  // -------- IGNORE INVALID READ --------

  if (rawHeading == 0)
  {
      return currentHeading;
  }

  headingBuffer[bufferIndex] =
      rawHeading;

  bufferIndex =
      (bufferIndex + 1)
      % numReadings;

  long sum = 0;

  for (int i = 0; i < numReadings; i++)
      sum += headingBuffer[i];

  return sum / numReadings;
}

void setTargetHeading(float roll, float pitch)
{
  int sum = 0;

  for (int i = 0; i < 10; i++)
  {
    

      sum += getAzimuth(roll,pitch);

      delay(100);
  }

  targetHeading =
      sum / 10;
}

void updateBaseServo()
{
  int angleDiff =
      targetHeading -
      currentHeading;

  while (angleDiff > 180)
      angleDiff -= 360;

  while (angleDiff < -180)
      angleDiff += 360;

  if (abs(angleDiff) <= deadZone)
      return;

  int constrainedDiff =
      constrain(
          angleDiff,
          -90,
          90
      );

  int newServoAngle =
      map(
          constrainedDiff,
          -90,
          90,
          0,
          180
      );

  newServoAngle =
      constrain(
          newServoAngle,
          0,
          180
      );

  if (abs(
          newServoAngle -
          baseServoAngle)
      > 2)
  {
      baseServoAngle =
          newServoAngle;

      int pwmBase =
          map(
              baseServoAngle,
              0,
              180,
              SERVOMIN,
              SERVOMAX
          );

      pwm.setPWM(
          BASE_SERVO,
          0,
          pwmBase
      );
  }
}

//////////////////////////////////////////////////////////
// SETUP
//////////////////////////////////////////////////////////

void setup()
{
  Serial.begin(115200);

  Wire.begin();

  Wire.setWireTimeout(3000, true);

  pwm.begin();
  pwm.setPWMFreq(50);

  delay(500);

  tselect(0);
  mpuShoulder.initialize();

  delay(100);

  tselect(2);
  mpuElbow.initialize();

  delay(100);

  tselect(4);
  mpuWrist.initialize();

  delay(100);

  // Initialize wrist buffer

  for (int i = 0; i < WINDOW_SIZE; i++)
    rollWindow[i] = 0;
 tselect(6);
  compass.init();

  delay(2000);


  pwm.setPWM(
      BASE_SERVO,
      0,
      map(
          90,
          0,
          180,
          SERVOMIN,
          SERVOMAX
      )
  );
Wire.setClock(100000);
Wire.setWireTimeout(10000, true);

  filteredValue =
      readMovingAverage();

  Serial.println("System Ready");

}

//////////////////////////////////////////////////////////
// LOOP
//////////////////////////////////////////////////////////

void loop()
{
  checkI2C();
  //////////////////////////////////////////////////
  // SHOULDER
  //////////////////////////////////////////////////

  float pitch1 = 0;
  float sideways1 = 0;
  float pitch2 = 0;
  float dummy = 0;

  tselect(0);

  readMPU(
      mpuShoulder,
      pitch1,
      dummy,
      sideways1
  );

  //////////////////////////////////////////////////
  // ELBOW
  //////////////////////////////////////////////////

  tselect(2);

  readMPU(
      mpuElbow,
      dummy,
      pitch2,
      dummy
  );

  //////////////////////////////////////////////////
  // WRIST
  //////////////////////////////////////////////////

  tselect(4);

  int16_t ax, ay, az;
  int16_t gx, gy, gz;

  mpuWrist.getMotion6(
      &ax,
      &ay,
      &az,
      &gx,
      &gy,
      &gz
  );

  float roll =
      atan2(ay, az)
      * 180 / PI;

  sumRoll -= rollWindow[indexWindow];

  rollWindow[indexWindow] = roll;

  sumRoll += roll;

  indexWindow++;

  if (indexWindow >= WINDOW_SIZE)
      indexWindow = 0;

  float filteredRoll =
      sumRoll / WINDOW_SIZE;

  if (abs(
          filteredRoll -
          previousFilteredRoll)
      < WRIST_DEADBAND)
      filteredRoll =
          previousFilteredRoll;

  previousFilteredRoll =
      filteredRoll;

  //////////////////////////////////////////////////
  // HALL SENSOR
  //////////////////////////////////////////////////

  int rawValue =
      readMovingAverage();

  filteredValue =
      alpha * rawValue +
      (1 - alpha)
      * filteredValue;

  int sensorValue =
      constrain(
          filteredValue,
          farValue,
          nearValue
      );

  int gripAngle =
      map(
          sensorValue,
          farValue,
          nearValue,
          openAngle,
          closeAngle
      );

  if (abs(
          gripAngle -
          lastGripAngle)
      < gripDeadband)
      gripAngle =
          lastGripAngle;

  lastGripAngle =
      gripAngle;

  //////////////////////////////////////////////////
  // STORE
  //////////////////////////////////////////////////

  pitchShoulderBuf[idx] =
      pitch1;

  sidewaysBuf[idx] =
      sideways1;

  pitchElbowBuf[idx] =
      pitch2;

  idx++;

  if (idx >= N)
      idx = 0;

  //////////////////////////////////////////////////
  // SMOOTH
  //////////////////////////////////////////////////

  float pitchShoulder =
      average(
          pitchShoulderBuf
      );

  float sideways =
      average(
          sidewaysBuf
      ) - 45;
if (sideways < -41) {
  sideways = -41;
}

  float pitchElbow =
      average(
          pitchElbowBuf
      ) ;

  float elbowAngle =
      pitchElbow +
      pitchShoulder;

if (elbowAngle >20)
  {elbowAngle = 20;
}

if (elbowAngle <-72)
 { elbowAngle = -72;
}
  //////////////////////////////////////////////////
  // BASE (COMPASS)
  //////////////////////////////////////////////////
 if (!targetSet)
{
    setTargetHeading(
        filteredRoll,
        pitchElbow
    );

    targetSet = true;

    Serial.println("Target heading set");
}
static unsigned long lastUpdate = 0;
static unsigned long lastValidHeadingTime = 0;

int newHeading =
    getSmoothedHeading(
        filteredRoll,
        pitchElbow
    );

// -------- TRACK VALID HEADING --------

if (newHeading != 0)
{
    currentHeading =
        newHeading;

    lastValidHeadingTime =
        millis();
}

// -------- AUTO RECOVERY --------

if (millis() - lastValidHeadingTime > 1500)
{
    Serial.println("Compass reinitializing");

    tselect(6);
    compass.init();

    lastValidHeadingTime =
        millis();
}

// -------- SERVO UPDATE --------

if (millis() - lastUpdate > 40)
{
    updateBaseServo();

    lastUpdate =
        millis();
}
Serial.print("Heading raw: ");
Serial.println(currentHeading);
  //////////////////////////////////////////////////
  // PWM OUTPUT
  //////////////////////////////////////////////////

  pwm.setPWM(
      SHOULDER_SERVO1,
      0,
      angleToPWM(
          pitchShoulder
      )
  );

  pwm.setPWM(
      SHOULDER_SERVO2,
      0,
      angleToPWM(
          sideways
      )
  );

  pwm.setPWM(
      ELBOW_SERVO,
      0,
      angleToPWM(
          elbowAngle
      )
  );

  pwm.setPWM(
      WRIST_SERVO,
      0,
      angleToPWM(
          filteredRoll
      )
  );

  pwm.setPWM(
      GRIP_SERVO,
      0,
      map(
          gripAngle,
          0,
          180,
          SERVOMIN,
          SERVOMAX
      )
  );

  //////////////////////////////////////////////////
  // DEBUG
  //////////////////////////////////////////////////

  Serial.print("Shoulder: ");
  Serial.print(pitchShoulder);

  Serial.print("  sideways: ");
  Serial.print(sideways);
  
  Serial.print("  Elbow: ");
  Serial.print(pitchElbow);

  Serial.print("  Wrist: ");
  Serial.print(filteredRoll);

  Serial.print("  Grip: ");
  Serial.println(gripAngle);

   Serial.print("  Base: ");
  Serial.println(baseServoAngle);


  delay(50);
}
