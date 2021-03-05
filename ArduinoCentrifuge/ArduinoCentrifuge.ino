#include <PID_v1.h>

int userInput[1]; // array stores speed and duration
int mosfetPin = 5;
int tachoPin = 3;

unsigned long currentTime = 0; // time since centrifugation start
double currentRPM = 0; // var to keep track of speed
unsigned int rotationCount = 0; // var to count rotations
volatile unsigned long startTime = 0; // marks start of one rotation
volatile unsigned long startTime = 0; // marks end of one rotation
volatile unsigned long rotationTime = 0; // time for one rotation
unsigned long delayTime = 2000; // interval for measuring speed, initialise to 2 sec

int initPower = 0; // value to write to analog pin
int writeDuty = 0; // duty cycle to send to MOSFET

bool tookInput = false; // has user provided input

// PID parameters
double Setpoint;
double Input;
double Output;
double Kp = 1, Ki = 2, Kd = 0.01;

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
  while (Serial.available() >= 0 && !tookInput) {  // PySerial used to take 2 user input values
    for (int i = 0; i < 2; i++) {
        userInput[i] = Serial.readString().toInt();
        Serial.println(userInput[i]);
    }
    Setpoint = userInput[0]; // user input RPM is desired state
    tookInput = true; // only take input once, will not enter this loop again
    currentTime = millis(); // begin timer
  }

  while (currentTime <= (userInput[1] * 60000)) {
      attachInterrupt(3, rotationSensed, RISING); // start counting rotations
      delay(delayTime); // count during this interval
      detachInterrupt(3); // stop counting

      currentRPM = calculateSpeed(startTime, endTime, rotationCount);
      Input = currentRPM;
      v2PID.Compute(); // PID calculation
      analogWrite(mosfetPin, Output); // write PWM output
      
      delayTime = endTime - startTime; // adjust delay time based on current speed
      rotationCount = 0; // refresh counter
  }

  // at the end of user-specified time
  analogWrite(mosfetPin, 0);
  currentTime = 0;
}

void rotationSensed() {
    // enters ISR if comparator output goes from 0 to 5V
    unsigned long curr = millis(); // start timer

    // get time between two jumps in signal, i.e. one rotation
    if (rotationCount == 0) {
      startTime = curr;
    } else {
      endTime = curr;
    }
    rotationCount += 1;
}

float calculateSpeed(startTime, endTime, rotationCount) {
  float timeDiff = endTime - startTime;
  float speedMillis = timeDiff / (rotationCount / 2) // time in ms for one rotation
  float rpm = 60000 / speedMillis // convert to RPM
  return rpm;
}
