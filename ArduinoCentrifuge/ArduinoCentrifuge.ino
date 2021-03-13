#include <PID_v1.h>

float userInput[1]; // array stores speed and duration
int mosfetPin = 5;
int tachoPin = 3;

unsigned long currentTime = 0; // time since centrifugation start
unsigned long startTime = 0; // time since user input provided
unsigned long goalTime = 0; // user input duration
double currentRPM = 0; // var to keep track of speed
unsigned int rotationCount = 0; // var to count rotations
unsigned long last_interrupt_time = 0;

int initPower = 0; // value to write to analog pin
int writeDuty = 0; // duty cycle to send to MOSFET
boolean rampUp = true;

int numInputs = 0;

// PID parameters
double Setpoint;
double Input;
double Output;
double Kp = 0.1, Ki = 0.12, Kd = 0;

PID v2PID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

void setup() {
  Serial.begin(9600);
  Serial.setTimeout(1);
  pinMode(mosfetPin, OUTPUT); // initialise D5 as MOSFET output
  pinMode(tachoPin, INPUT); // initialise D3 as tachometer input

  v2PID.SetMode(AUTOMATIC);
  v2PID.SetTunings(Kp, Ki, Kd);
}

void loop() {  
  while (Serial.available() > 0) {  // PySerial used to take 2 user input values
    if (numInputs == 0) {  // speed
      String incomingString1 = Serial.readString();
      userInput[0] = incomingString1.toFloat();
      numInputs += 1;
    } else {  // duration
      String incomingString2 = Serial.readString();
      userInput[1] = incomingString2.toFloat();
      Setpoint = userInput[0];
      goalTime = userInput[1] * 60000;
      startTime = millis();
      break;
    }
  }

  currentTime = millis();
  int cnt = 0; // delete

  while ((currentTime - startTime) <= goalTime) {
      attachInterrupt(digitalPinToInterrupt(3), rotationSensed, RISING); // start counting rotations
      delay(1000); // count during this interval
      detachInterrupt(digitalPinToInterrupt(3)); // stop counting

      currentRPM = rotationCount * 30; // use rotation count to get RPM
      Input = currentRPM;
      if (Setpoint - currentRPM < 100 && rampUp) {  // ramp up time is not part of user-specified time
          rampUp = false;
          startTime = currentTime;
          currentTime = millis();
      }
      v2PID.Compute(); // PID calculation
      analogWrite(mosfetPin, Output); // write PWM output

      // can visualise PID values in Serial plotter
      Serial.print(Input);
      Serial.print(" ");
      Serial.print(Output);
      Serial.print(" ");  
      Serial.println(Setpoint);
      
      rotationCount = 0; // refresh counter
      currentTime = millis();
  }

  // at the end of user-specified time
  analogWrite(mosfetPin, 0);
  currentTime = 0;
  
}

void rotationSensed() {
    // enters ISR if comparator output goes from 0 to 5V
    unsigned long interrupt_time = millis();
    if (interrupt_time - last_interrupt_time > 10) {
      rotationCount += 1;
      //Serial.println(rotationCount);
      last_interrupt_time = interrupt_time;
    }
}
