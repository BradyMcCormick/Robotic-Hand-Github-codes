#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Servo.h>

RF24 radio(7, 10); // CE, CSN
const byte address[6] = "00001";

//  Creating a struct in order to send the data
typedef struct{
  float A;
  float B;
  float C;
  float D;
}
A_t;

A_t angles;

// This is for the Servos
Servo servo9;
Servo servo6;
Servo servo5;
Servo servo3;

void setup() {
Serial.begin(9600);
radio.begin();
radio.openReadingPipe(0, address);   //Setting the address at which we will receive the data
//radio.setPALevel(RF24_PA_MIN);       //You can set this as minimum or maximum depending on the distance between the transmitter and receiver.
radio.startListening();              //This sets the module as receiver

// This is for the Servos
servo9.attach(9);
servo6.attach(6);
servo5.attach(5);
servo3.attach(3);
}

void loop(){
char text0[] = ""; 
if (radio.available()) {            //Looking for the data.
    radio.read(&angles, sizeof(angles));
    radio.read(&text0, sizeof(text0));    //Reading the data
    Serial.println(text0);
}

// Printing struct data
Serial.print("angles.A = ");
Serial.print(angles.A);
Serial.print("angles.B = ");
Serial.print(angles.B);
Serial.print("angles.C = ");
Serial.print(angles.C);
Serial.print("angles.D = ");
Serial.print(angles.D);

// This is for the servos
  if (angles.A > 60) {
    servo9.write(180);
  }
  else {
    servo9.write(0);
  }

  if (angles.B > 60) {
    servo6.write(180);
  }
  else {
    servo6.write(0);
  }

  if (angles.C > 60) {
    servo5.write(180);
  }
  else {
    servo5.write(0);
  }

  if (angles.D > 60) {
    servo3.write(180);
  }
  else {
    servo3.write(0);
  }

delay(50);
}
