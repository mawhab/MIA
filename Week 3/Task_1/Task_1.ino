#include <ros.h>
#include <std_msgs/Float32.h>

// motor driver pins
#define PWM PA7
#define DIRECTION PA6

// encoder pins
#define B PB12
#define A PB13

#define PPR 2048 // pulses per revolution

// function definitions
void calcVelocity();
void ISR_A();
void ISR_B();
void setMotor(int dir, int voltage);
void inputRead(const std_msgs::Float32 &speedmsg);
int calculatePID();


// ros variables
const char* topic = "speed"; // topic to subscribe to
ros::NodeHandle nh; //ros handler
ros::Subscriber<std_msgs::Float32> sub(topic, &inputRead); // subscriber for our topic

// encoder variables
long counter = 0; // counter for pulses
long prevCount = 0; // extra variable to check if velocity is 0
float velocity; // variable to hold velocity based on encoder readings
float prevVelocity; // variable to hold previous velocity, used in filter
float filteredVelocity = 0; // filtered velocity
long currTime; // current time
long prevTime_encoder; // previous time for use by encoder

float SPspeed; // our set point that we want to reach
float error; // error to be compensated
float errorI; // error to be compensated in integral
float errorD; // error to be compensated in differential
int outputV; // output voltage by PID
int outputD; // direction of output
float kp; // P constant
float ki; // I constant
float kd; // D constant
long prevTime_PID; // prev time for use by PID


void setup() {
  // setting up ros
  (nh.getHardware())->setPort(&Serial1); // setting serial port for ros
  (nh.getHardware())->setBaud(115200); // set baud rate for ros
  nh.initNode(); // initializing node handler
  nh.subscribe(sub); // subscribing to topic

  // setting PID constants
  kp= 0;
  ki = 0;
  kd = 0;

  // setting pin modes
  pinMode(A, INPUT);
  pinMode(B, INPUT);
  pinMode(PWM, OUTPUT);
  pinMode(DIRECTION, OUTPUT);

  // attaching interrupts
  attachInterrupt(digitalPinToInterrupt(A), ISR_A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(B), ISR_B, CHANGE);
}

void loop() {
  outputV = calculatePID(); // calculating PID values
  if(outputV<0){ // if value is negative adjust direction
    outputD = -1;
  }else{
    outputD = 1;
  }
  outputV = abs(outputV); // get magnitude of voltage
  
  if(outputV>255) // if output is higher than possible reduce it to max
    outputV = 255;
  
  setMotor(outputD, outputV); // set motor speed and direction
  
  nh.spinOnce();// node handler spin
}



//----------------------------FUNCTIONS---------------------------------

// calculate current velocity based on ecoder readings
void calcVelocity(int increment){
  currTime = micros(); // getting current time
  float dTime = (currTime - prevTime_encoder) / 1000000.0; // getting difference in time between this pulse and last pulse
  prevTime_encoder = currTime; // switching time
  velocity = increment / dTime; // getting velocity based on time between pulses
  velocity *= 60/PPR; // multiplying by 60 and dividing by PPR to get rpm instead of counts/second
}

// interrupt function for a
void ISR_A(){
  int increment;
  if(digitalRead(A)!=digitalRead(B)){
    increment = 1;
  }else{
    increment = -1;
  }
  counter += increment;
  calcVelocity(increment);
}

// interrupt function for b
void ISR_B(){
  int increment;
  if(digitalRead(A)!=digitalRead(B)){
    increment = -1;
  }else{
    increment = 1;
  }
  counter += increment;
  calcVelocity(increment);
}

// setting motor speed and direciton
void setMotor(int dir, int voltage){
  if(dir==1)
    digitalWrite(DIRECTION, HIGH);
  else if(dir==-1){
    digitalWrite(DIRECTION, LOW);
  }
  analogWrite(PWM, voltage);
}

// parsing message upon receiving from ros topic
void inputRead(const std_msgs::Float32 &speedmsg){
  SPspeed = speedmsg.data;
}

// calculates PID output value
int calculatePID(){
  currTime = micros(); // getting current time
  float dT = (currTime - prevTime_PID) / 1000000.0; // getting difference in time between last calculation
  prevTime_PID = currTime; // setting new prev time
  if(prevCount == counter){// if prevcount is equal to count then the encoder isnt moving
    velocity = 0; // set velocity to 0
    filteredVelocity = 0;// set prev filtered velocity to 0
  }
  
  filteredVelocity = 0.854*filteredVelocity + 0.0728*velocity + 0.0728*prevVelocity; // applying a low pass filter (25 Hz cutoff)
  prevVelocity = velocity;
  
  prevCount = counter; // switch counter vars
  error = SPspeed - filteredVelocity; // calculating error
  errorI += error * dT; // getting I 
  errorD = error / dT; // getting D
  return error*kp + errorI*ki + errorD*kd; // return error multiplied by constants
}
