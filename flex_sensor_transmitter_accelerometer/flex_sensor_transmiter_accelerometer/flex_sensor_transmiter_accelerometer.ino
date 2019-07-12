//ACCELEROMETER STUFF vvvv

// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps20.h"
//#include "MPU6050.h" // not necessary if using MotionApps include file

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu;
//MPU6050 mpu(0x69); // <-- use for AD0 high

#define OUTPUT_READABLE_YAWPITCHROLL

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// interrupt detection routine
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}

int x


// EVERYTHING ELSE vvv


#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

RF24 radio(7, 10); // CE, CSN         
const byte address[6] = "lerf2";     //Byte of array representing the address. This is the address where we will send the data. This should be same on the receiving side.

const int FLEX_PIN0 = A0; // Pin connected to voltage divider output
const int FLEX_PIN1 = A1;
const int FLEX_PIN2 = A2;
const int FLEX_PIN3 = A3;

// Measure the voltage at 5V and the actual resistance of your
// 47k resistor, and enter them below:
const float VCC = 5.00; // Measured voltage of Ardunio 5V line
const float R_DIV = 21000.0; // Measured resistance of 3.3k resistor

const float STRAIGHT_RESISTANCE0 = 8500.0; // resistance when straight
const float BEND_RESISTANCE0 = 17100; // resistance at 90 deg
const float STRAIGHT_RESISTANCE1 = 11000.0;
const float BEND_RESISTANCE1 = 18700;
const float STRAIGHT_RESISTANCE2 = 9500.0;
const float BEND_RESISTANCE2 = 15400;
const float STRAIGHT_RESISTANCE3 = 10200.0;
const float BEND_RESISTANCE3 = 19500;

//  Creating a struct in order to send the data
typedef struct{
  float A;
  float B;
  float C;
  float D;
  float roll;
}
A_t;

A_t angles;

void setup() {
//ACCELEROMETER STUFF vvv
  
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    // initialize serial communication
    // (115200 chosen because it is required for Teapot Demo output, but it's
    // really up to you depending on your project)
    Serial.begin(115200);
    while (!Serial); // wait for Leonardo enumeration, others continue immediately

    // NOTE: 8MHz or slower host processors, like the Teensy @ 3.3v or Ardunio
    // Pro Mini running at 3.3v, cannot handle this baud rate reliably due to
    // the baud timing being too misaligned with processor ticks. You must use
    // 38400 or slower in these cases, or use some kind of external separate
    // crystal solution for the UART timer.

    // initialize device
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();

    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        attachInterrupt(0, dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)8k
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")")); 
    }
// EVERYTHING ELSE vvv

  pinMode(FLEX_PIN0, INPUT);
  pinMode(FLEX_PIN1, INPUT);
  pinMode(FLEX_PIN2, INPUT);
  pinMode(FLEX_PIN3, INPUT);

  // This is for the NRF chips
  radio.begin();                  //Starting the Wireless communication
  radio.openWritingPipe(address); //Setting the address where we will send the data
  //radio.setPALevel(RF24_PA_MIN);  //You can set it as minimum or maximum depending on the distance between the transmitter and receiver.
  radio.stopListening();          //This sets the module as transmitter
}

void loop() {

// ACCELEROMETER STUFF

// if programming failed, don't try to do anything
   if (!dmpReady) return;

    // wait for MPU interrupt or extra packet(s) available
    while (!mpuInterrupt && fifoCount < packetSize){
      
    }
      // reset interrupt flag and get INT_STATUS byte
      mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & 0x02) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;

        #ifdef OUTPUT_READABLE_YAWPITCHROLL
            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            Serial.print("ypr\t");
            Serial.print(ypr[0] * 180/M_PI);
            Serial.print("\t");
            Serial.print(ypr[1] * 180/M_PI);
            Serial.print("\t");
            Serial.println(ypr[2] * 180/M_PI);
        #endif
    }

// EVERYTHING ELSE vvv

  // Read the ADC, and calculate voltage and resistance from it
  int flexADC0 = analogRead(FLEX_PIN0);
  float flexV0 = flexADC0 * VCC / 1023.0;
  float flexR0 = R_DIV * (VCC / flexV0 - 1.0);
  Serial.println("Resistance of Flex Sensor 0: " + String(flexR0) + " ohms");

  // Use the calculated resistance to estimate the sensor's
  // bend angle:
  float angle0 = map(flexR0, STRAIGHT_RESISTANCE0, BEND_RESISTANCE0,
                    0, 90.0); 

  Serial.println("Bend: " + String(angle0) + " degrees");
  Serial.println();
  
  int flexADC1 = analogRead(FLEX_PIN1);
  
  float flexV1 = flexADC1 * VCC / 1023.0;
  float flexR1 = R_DIV * (VCC / flexV1 - 1.0);
  Serial.println("Resistance of Flex Sensor 1: " + String(flexR1) + " ohms");

  float angle1 = map(flexR1, STRAIGHT_RESISTANCE1, BEND_RESISTANCE1,
                    0, 90.0);

  Serial.println("Bend: " + String(angle1) + " degrees");
  Serial.println();

  int flexADC2 = analogRead(FLEX_PIN2);
  float flexV2 = flexADC2 * VCC / 1023.0;
  float flexR2 = R_DIV * (VCC / flexV2 - 1.0);
  Serial.println("Resistance of Flex Sensor 2: " + String(flexR2) + " ohms");

  float angle2 = map(flexR2, STRAIGHT_RESISTANCE2, BEND_RESISTANCE2,
                    0, 90.0);

  Serial.println("Bend: " + String(angle2) + " degrees");
  Serial.println();

  int flexADC3 = analogRead(FLEX_PIN3);
  float flexV3 = flexADC3 * VCC / 1023.0;
  float flexR3 = R_DIV * (VCC / flexV3 - 1.0);
  Serial.println("Resistance of Flex Sensor 3: " + String(flexR3) + " ohms");

  float angle3 = map(flexR3, STRAIGHT_RESISTANCE3, BEND_RESISTANCE3,
                    0, 90.0);
  
  Serial.println("Bend: " + String(angle3) + " degrees");
  Serial.println();

  // This is assigning values in the struct
  angles.A = angle0;
  angles.B = angle1;
  angles.C = angle2;
  angles.D = angle3;
  angles.roll = ypr[2];

  // These are the beginnings of communication
  radio.write(&angles, sizeof(angles));


  if (angle0 > 45) {
    const char text0[] = "Flex sensor0 is bent";
    radio.write(&text0, sizeof(text0));
  }
  else {
    const char text0[] = "Flex sensor0 is straight";
    radio.write(&text0, sizeof(text0));
  }

  delay(5000);
}
