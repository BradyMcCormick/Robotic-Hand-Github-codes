/******************************************************************************
  Flex_Sensor_Example.ino
  Example sketch for SparkFun's flex sensors
  (https://www.sparkfun.com/products/10264)
  Jim Lindblom @ SparkFun Electronics
  April 28, 2016

  Create a voltage divider circuit combining a flex sensor with a 47k resistor.
  - The resistor should connect from A0 to GND.
  - The flex sensor should connect from A0 to 3.3V
  As the resistance of the flex sensor increases (meaning it's being bent), the
  voltage at A0 should decrease.

  Development environment specifics:
  Arduino 1.6.7
******************************************************************************/
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

RF24 radio(7, 10); // CE, CSN         
const byte address[6] = "00001";     //Byte of array representing the address. This is the address where we will send the data. This should be same on the receiving side.

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
const float STRAIGHT_RESISTANCE1 = 9900.0;
const float BEND_RESISTANCE1 = 19300;
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
}
A_t;

A_t angles;

void setup()
{
  Serial.begin(9600);
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

void loop()
{
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

  delay(50);
}
