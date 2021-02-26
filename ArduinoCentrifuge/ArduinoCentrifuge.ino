#include <PID_v1.h>

int userInput[1]; // array stores speed and duration
int mosfetPin = 5;
int tachoPin = A1;

unsigned long currentTime = 0; // time since centrifugation start
double currentRPM = 0; // var to keep track of speed
int initPower = 0; // value to write to analog pin
int writeDuty = 0; // duty cycle to send to MOSFET

bool tookInput = false; // has user provided input

// PID placeholder values (will change once we know more about PID)
int Kp = 1;
int Ki = 2;
int Kd = 3;
double Input = 50;
double Output = 500;
double Setpoint = 50;

PID v2PID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

void setup() {
  Serial.begin(9600);
  Serial.setTimeout(1);
  pinMode(mosfetPin, OUTPUT); // initialise digital pin 5 as MOSFET output
  pinMode(tachoPin, INPUT); // initialise A1 as tachometer input
  attachInterrupt(3, pin_ISR, CHANGE);
  
  v2PID.SetMode(AUTOMATIC); // configure PID (to be changed)
}

void loop() {
  while(Serial.available() >= 0 && !tookInput) {  // PySerial used to take 2 user input values
    for (int i = 0; i < 2; i++) {
        userInput[i] = Serial.readString().toInt();
        Serial.println(userInput[i]);
    }
    tookInput = true; // only take input once, will not enter this loop again
    currentTime = millis(); // begin timer
  }

  if (currentTime <= 15 * 60000)) {  // assume that the first 15 seconds = ramp up period
    currentRPM = calculateSpeed();
    if (currentRPM < userInput[0]) {
      initPower = initPower + 10; // increment voltage input if current speed is lower than desired speed
    }
    writeDuty = map(initPower, 0, 1023, 0, 255);
    analogWrite(mosfetPin, writeDuty); // write voltage to MOSFET gate
  } elif (currentTime <= (userInput[1] * 60000)) {  // not in ramp up phase
      v2PID.Compute(); // PID does its thing, unsure exactly how
      analogWrite(mosfetPin, Output);
  } else {  // end centrifugation
    analogWrite(mosfetPin, 0);
    currentTime = 0;
  }
}

double calculateSpeed() {
  tachoOutput = analogRead(tachoPin);
  // perform calculations to convert tachometer output to RPM (will require experimentation)
  // will eventually add a comparator between tachometer system and Arduino
  return tachoOutput;
}
