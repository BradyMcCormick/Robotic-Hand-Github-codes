#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Servo.h>

RF24 radio(7, 10); // CE, CSN
const byte address[6] = "lerf2";

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

// This is for the Servos
Servo servo9;
Servo servo6;
Servo servo5;
Servo servo3;

// This is for the stepper motor
int x; 

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

// This is for the stepper motor
  pinMode(7,OUTPUT); // Enable
  pinMode(A1,OUTPUT); // Step
  pinMode(4,OUTPUT); // Dir
  digitalWrite(6,LOW); // Set Enable low
}

void loop(){
char text0[] = ""; 
if (radio.available()) {            //Looking for the data.
    radio.read(&angles, sizeof(angles));
}

// Printing struct data
Serial.print("angles.A = ");
Serial.print(angles.A);
Serial.print("\n");
Serial.print("angles.B = ");
Serial.print(angles.B);
Serial.print("\n");
Serial.print("angles.C = ");
Serial.print(angles.C);
Serial.print("\n");
Serial.print("angles.D = ");
Serial.print(angles.D);
Serial.print("\n");
Serial.print("angles.roll = ");
Serial.print(angles.roll);
Serial.print("\n");

// This is for the servos

  if (angles.A > 180) {
    angles.A = 180;
  }
    servo9.write(180 - angles.A);
  
  if (angles.B > 180) {
    angles.B = 180;
  }
    servo6.write(180 - angles.B);
    
  if (angles.C > 180) {
    angles.C = 180;
  }
    servo5.write(180 - (angles.C*2));

  if (angles.D > 180) {
    angles.D = 180;
  }
    servo3.write(180 - angles.D);

// This is for the stepper motor
  digitalWrite(3,LOW); // Set Enable low

  if (angles.roll > 0) {
    digitalWrite(4,LOW); // Set Dir
  }
  else {
    digitalWrite(4,HIGH);
  }
  if (-50 < angles.roll < 50) {
    angles.roll = 0;
  }
  for(x = 0; x < angles.roll; x++) // step as many times as the roll is
  {
    digitalWrite(A1,HIGH); // Output high
    delay(1); // Wait
    digitalWrite(A1,LOW); // Output low
    delay(10); // Wait
  }
}
