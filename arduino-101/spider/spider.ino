#include <MadgwickAHRS.h>
#include <BLEAttribute.h>
#include <BLECentral.h>
#include <BLECharacteristic.h>
#include <BLECommon.h>
#include <BLEDescriptor.h>
#include <BLEPeripheral.h>
#include <BLEService.h>
#include <BLETypedCharacteristic.h>
#include <BLETypedCharacteristics.h>
#include <BLEUuid.h>
#include <CurieBLE.h>
#include <CurieIMU.h>

#include "globals.h"
#include "pin.h"
#include "SpiderMotor.h"
#include "task.h"

#define M_PI 3.1415926f


SpiderMotor motor;
enum TASK_ID {
  TASK_JOYSTICK = 1,
  TASK_STEPCOUNT = 2,
  TASK_HEADING = 3,
  TASK_BLE_SEND = 4,
  TASK_BLE_RECEIVE = 5,
  TASK_NUM
};
Task g_task[TASK_NUM];

/****************************
   BLE related section START
 ****************************/
#define BLE_UPDATE_INTERVAL 1000000
#define SPIDER_BLE_NAME "XpiderBLE"
#define SPIDER_UUID "19B10000-E8F2-537E-4F6C-D10476AB9339"
#define SPIDER_UUID_JOY "19B10000-E8F2-537E-4F6C-D10476AB9340"
#define SPIDER_UUID_STEP "19B10000-E8F2-537E-4F6C-D10476AB9341"
#define SPIDER_UUID_HEADING "19B10000-E8F2-537E-4F6C-D10476AB9342"
#define SPIDER_UUID_AUTOPILOT "19B10000-E8F2-537E-4F6C-D10476AB9349"
#define SPIDER_MAX_JOYSTICK_CHAR 4
#define SPIDER_MAX_IMU_CHAR 12
#define SPIDER_MAX_AUTOPILOT_CHAR 8
BLEPeripheral blePeripheral;
BLEService spiderService(SPIDER_UUID);
BLECharacteristic joystickChar(SPIDER_UUID_JOY, BLERead | BLEWrite, SPIDER_MAX_JOYSTICK_CHAR);
BLECharacteristic autopilotChar(SPIDER_UUID_AUTOPILOT, BLERead | BLEWrite, SPIDER_MAX_AUTOPILOT_CHAR);
BLELongCharacteristic stepCounterChar(SPIDER_UUID_STEP, BLERead | BLENotify);
BLEFloatCharacteristic headingChar(SPIDER_UUID_HEADING, BLERead | BLENotify);
/****************************
   BLE related section END
 ****************************/


Madgwick filter;
float accelScale, gyroScale;


void initializeIMU () {

/*
  CurieIMU.autoCalibrateGyroOffset();
  CurieIMU.autoCalibrateAccelerometerOffset(1, 0);
  CurieIMU.autoCalibrateAccelerometerOffset(2, 0);
  CurieIMU.autoCalibrateAccelerometerOffset(3, 1);
*/

  //setting the sample rate of the acelerometer and the gyro and the filter to 25Hz
  CurieIMU.begin();
  CurieIMU.setGyroRate(25);
  CurieIMU.setAccelerometerRate(25);
  filter.begin(25);

  //we set the accelerometer range to 2g and the gyro range to 250 °/s
  CurieIMU.setAccelerometerRange(2);
  CurieIMU.setGyroRange(250);

}


#define TASK_INTERVAL_JOYSTICK 50000
void taskReceiveJoystickMessage() {
  const unsigned char *rx_buffer;
  int forward;
  int rotate;
  if (joystickChar.written()) {
    rx_buffer = joystickChar.value();
    //TODO: process the joystick code here.
    rotate = ((int)rx_buffer[0] - 127) * 2;
    //Serial.print("rotate:");Serial.println(rotate);
    forward = ((int)rx_buffer[1] - 127) * 2;
    //Serial.print("forward:");Serial.println(forward);
    //process joystick control
    if (forward) {
      motor.walk(forward);
    } else {
      motor.stopWalking();
    }
    if (rotate) {
      motor.rotate(rotate);
    } else {
      motor.stopRotating();
    }
  }

}
#define TASK_INTERVAL_STEPCOUNTER 33000
static long g_lastStep = 0;
static long g_stepCounter = 0;
void taskStepCounter()
{
  long d = 0;
  if (g_stepCounter != g_lastStep) {
    d = motor.dir() * (g_stepCounter - g_lastStep);
    stepCounterChar.setValue(d);
    g_lastStep = g_stepCounter;
  }
}
void isr_step_count() {
  g_stepCounter += 1;
}


#define TASK_RATE_HEADING 25
unsigned short g_heading;
void taskHeadingUpdate() {

  int aix, aiy, aiz;
  int gix, giy, giz;
  float ax, ay, az;
  float gx, gy, gz;
  float roll, pitch, heading;

  // read raw data from CurieIMU
  CurieIMU.readMotionSensor(aix, aiy, aiz, gix, giy, giz);

  // convert from raw data to gravity and degrees/second units
  ax = convertRawAcceleration(aix);
  ay = convertRawAcceleration(aiy);
  az = convertRawAcceleration(aiz);
  gx = convertRawGyro(gix);
  gy = convertRawGyro(giy);
  gz = convertRawGyro(giz);



  // update the filter, which computes orientation
  filter.begin(g_task[TASK_HEADING].getRate());
  //filter.updateIMU(gix/100.0f, giy/100.0f, giz/100.0f, aix/100.0f, aiy/100.0f, aiz/100.0f);
  filter.updateIMU(gx, gy, gz, ax, ay, az);

  // print the heading, pitch and roll
  roll = filter.getRoll();
  pitch = filter.getPitch();
  heading = filter.getYaw();

  #if defined DEBUG_MODE  
    Serial.print("Orientation: ");
    Serial.println(heading);
  #endif

  g_heading = heading;

}

float convertRawAcceleration(int aRaw) {
  // since we are using 2G range
  // -2g maps to a raw value of -32768
  // +2g maps to a raw value of 32767

  float a = (aRaw * 2.0) / 32768.0;
  return a;
}

float convertRawGyro(int gRaw) {
  // since we are using 250 degrees/seconds range
  // -250 maps to a raw value of -32768
  // +250 maps to a raw value of 32767

  float g = (gRaw * 250.0) / 32768.0;
  return g;
}


#define TASK_RATE_BLE_SEND 5
void taskBLESend() {

  headingChar.setValue(g_heading);

}


/*
#define TASK_INTERVAL_AUTOPILOT 40000 //40ms 25hz
long g_autopilotDegree = 0;
void taskAutopilot() {
  const unsigned char *rx_buffer;
  unsigned char * p = (unsigned char*)(&g_autopilotDegree);
  unsigned char isRunning = 0;
  //update the dst degree
  if (autopilotChar.written()) {
    rx_buffer = autopilotChar.value();
    p[0] = rx_buffer[0];
    p[1] = rx_buffer[1];
    p[2] = rx_buffer[2];
    p[3] = rx_buffer[3];
    isRunning = rx_buffer[4];
    //Serial.print("dstDegree:");Serial.println(g_autopilotDegree);
    if (!isRunning) {
      motor.stopRotating();
      motor.stopWalking();
    }
  }
  //TODO:to rotate to the target degree
  if (isRunning) {
    //calc the dist ar
    int heading = ((int)g_heading + 720) % 360;
    float arc = (g_autopilotDegree - heading) * 3.1415f / 180.0f;
    double rotate = sin(arc) * 1.1f;
    double forward = cos(arc) * 1.2f;

    if (rotate > 1.0f) {
      rotate = 1.0f;
    }
    if (rotate < -1.0f) {
      rotate = -1.0f;
    }
    if (forward > 1.0f) {
      forward = 1.0f;
    }
    if (forward < -1.0f) {
      forward = -1.0f;
    }

    if (forward) {
      motor.walk(forward * 255);
    } else {
      motor.stopWalking();
    }
    if (rotate) {
      motor.rotate(rotate * 255);
    } else {
      motor.stopRotating();
    }
  }
}
*/

void setup() {
  Serial.begin(9600);


  blePeripheral.setLocalName(SPIDER_BLE_NAME);
  blePeripheral.setAdvertisedServiceUuid(spiderService.uuid());

  //add service and characteristic
  blePeripheral.addAttribute(spiderService);
  blePeripheral.addAttribute(autopilotChar);
  blePeripheral.addAttribute(joystickChar);
  blePeripheral.addAttribute(stepCounterChar);
  blePeripheral.addAttribute(headingChar);

  stepCounterChar.setValue(0);
  blePeripheral.begin();


  initializeIMU ();

  //setup task
  //g_task[TASK_JOYSTICK].init(TASK_INTERVAL_JOYSTICK, taskReceiveJoystickMessage);
  //g_task[TASK_STEPCOUNT].init(TASK_INTERVAL_STEPCOUNTER, taskStepCounter);
  //g_task[TASK_AUTOPILOT].init(TASK_INTERVAL_AUTOPILOT, taskAutopilot);

  
  g_task[TASK_HEADING].initialize(TASK_HEADING, TASK_RATE_HEADING, taskHeadingUpdate);
  g_task[TASK_BLE_SEND].initialize(TASK_BLE_SEND, TASK_RATE_BLE_SEND, taskBLESend);

}


unsigned long microsCumulated;
//unsigned long tickPerSecond;
//unsigned long averageTickPerSecond;
void schedule_tasks() {
  int i = 0;
  unsigned long microsNow = micros();
/*
  tickPerSecond++;
  if (microsNow - microsCumulated > 1000000)
    microsCumulated = microsNow;
    if (averageTickPerSecond == 0) {
      averageTickPerSecond = tickPerSecond;
    }
    else {
      averageTickPerSecond = (averageTickPerSecond+tickPerSecond)/2
    }
    tickPerSecond = 0;
*/  
  for (i = 0; i < TASK_NUM; i++) {
    g_task[i].trigger(microsNow);
  }
}


void loop() {
  BLECentral central = blePeripheral.central();

  // if a central is connected to peripheral:
  if (central) {

    #if defined DEBUG_MODE  
      Serial.print("Connected to central: ");
      Serial.println(central.address());
    #endif

    // while the central is still connected to peripheral:
    bool l_BLEConnected = central.connected();
    long microsPrevious = micros();
    while (l_BLEConnected) {

      schedule_tasks();

      if (micros() - microsPrevious > BLE_UPDATE_INTERVAL) {
          #if defined DEBUG_MODE  
            Serial.println("Still connected");
          #endif
        l_BLEConnected = central.connected();
        microsPrevious = micros();
      }
    }

  #if defined DEBUG_MODE  
    // when the central disconnects, print it out:
    Serial.print("Disconnected from central: ");
    Serial.println(central.address());
  #endif
  }
}








