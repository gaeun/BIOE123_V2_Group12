#include <PID_v1.h>

int userInput[1]; // array stores speed and duration
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
double Kp = 0.001, Ki = 0.1, Kd = 0;

PID v2PID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

void setup() {
  Serial.begin(9600);
  Serial.setTimeout(1);
  pinMode(mosfetPin, OUTPUT); // initialise D5 as MOSFET output
  pinMode(tachoPin, INPUT); // initialise D3 as tachometer input

  Setpoint = 1000; // default Setpoint, temporary
  v2PID.SetMode(AUTOMATIC);
  v2PID.SetTunings(Kp, Ki, Kd);
//  analogWrite(mosfetPin, 100); // write PWM output
}

void loop() {
  while (Serial.available() > 0) {  // PySerial used to take 2 user input values
    if (numInputs == 0) {  // speed
      int incomingString1 = Serial.readString().toInt();
      userInput[0] = incomingString1;
      numInputs += 1;
    } else {  // duration
      int incomingString2 = Serial.readString().toInt();
      userInput[1] = incomingString2;
      Setpoint = userInput[0];
      goalTime = userInput[1] * 60000;
      startTime = millis();
      break;
    }
  }

  currentTime = millis();

  while ((currentTime - startTime) <= goalTime) {
      attachInterrupt(digitalPinToInterrupt(3), rotationSensed, RISING); // start counting rotations
      delay(1000); // count during this interval
      detachInterrupt(digitalPinToInterrupt(3)); // stop counting

      currentRPM = (rotationCount * 60); // use rotation count to get RPM
      Input = currentRPM;
      if (Setpoint - currentRPM < 100 && rampUp) {
          rampUp = false;
          startTime = currentTime;
          currentTime = millis();
      }
      v2PID.Compute(); // PID calculation
//      Serial.println(Output);
      analogWrite(mosfetPin, Output); // write PWM output
      
      Serial.println(currentRPM); // PRINT FOR TESTING
      rotationCount = 0; // refresh counter
      currentTime = millis();
  }

  // at the end of user-specified time
  analogWrite(mosfetPin, 0);
  currentTime = 0;
}

void rotationSensed() {
    // enters ISR if comparator output goes from 0 to 5V
//    Serial.println("Launched");
    unsigned long interrupt_time = millis();
    if (interrupt_time - last_interrupt_time > 10) {
      rotationCount += 1;
      //Serial.println(rotationCount);
      last_interrupt_time = interrupt_time;
    }
}
