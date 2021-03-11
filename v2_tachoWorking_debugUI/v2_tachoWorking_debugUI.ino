#include <PID_v1.h>

int userInput[1]; // array stores speed and duration
int mosfetPin = 5;
int tachoPin = 3;

unsigned long setupTime = 0; // time since program launched
unsigned long currentTime = 0; // time since centrifugation start
double currentRPM = 0; // var to keep track of speed
unsigned int rotationCount = 0; // var to count rotations
unsigned long last_interrupt_time = 0;

int initPower = 0; // value to write to analog pin
int writeDuty = 0; // duty cycle to send to MOSFET

int numInputs = 0;

// PID parameters
double Setpoint;
double Input;
double Output;
double Kp = 0.001, Ki = 0, Kd = 0;

PID v2PID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

void setup() {
  Serial.begin(9600);
  Serial.setTimeout(1);
  pinMode(mosfetPin, OUTPUT); // initialise D5 as MOSFET output
  pinMode(tachoPin, INPUT); // initialise D3 as tachometer input

  Setpoint = 1000;
  v2PID.SetMode(AUTOMATIC);
  v2PID.SetTunings(Kp, Ki, Kd);
  setupTime = millis(); // accounting for time it takes to set up
}

void loop() {
  while (Serial.available() > 0) {  // PySerial used to take 2 user input values
    if (numInputs == 0) {
      int incomingString1 = Serial.readString().toInt();
      Serial.println(incomingString1);
      numInputs += 1;
    } else {
      int incomingString2 = Serial.readString().toInt();
      Serial.println(incomingString2);
      currentTime = millis();
      break;
    }
//    Setpoint = userInput[0]; // user input RPM is desired state
//    tookInput = true; // only take input once, will not enter this loop again
//    currentTime = millis(); // begin timer
  }

  while ((currentTime - setupTime) <= 60000) { // (userInput[1] * 60000)
      attachInterrupt(digitalPinToInterrupt(3), rotationSensed, RISING); // start counting rotations
      delay(1000); // count during this interval
      detachInterrupt(digitalPinToInterrupt(3)); // stop counting

      currentRPM = (rotationCount * 60); // use rotation count to get RPM
      Input = currentRPM;
      v2PID.Compute();
      analogWrite(mosfetPin, Output);
      
//      Serial.print(Input);
//      Serial.print(" ");
//      Serial.print(Output); //scale based on how it looks in the plotter...different setups will have different ratios
//      Serial.print(" ");  
//      Serial.println(Setpoint);
//      Serial.println(currentRPM); // PRINT FOR TESTING
      rotationCount = 0; // refresh counter
  }

  // at the end of user-specified time
  analogWrite(mosfetPin, 0);
  currentTime = 0;
}

void rotationSensed() {
    // enters ISR if comparator output goes from 0 to 5V
    unsigned long interrupt_time = millis();
    if (interrupt_time - last_interrupt_time > 5) {
      rotationCount += 1;
      last_interrupt_time = interrupt_time;
    }
}
