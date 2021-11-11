#include <ros.h>
#include <std_msgs/Float32.h>

// motor driver pins
#define PWM PA0
#define DIRECTION PA1

// encoder pins
#define B PA4
#define A PA5

#define PPR 540 // pulses per revolution

// function definitions
void ISR_A();
void ISR_B();
void inputRead(const std_msgs::Float32 &speedmsg);
float calculatePID();
int mapPIDtoVolt(float rpm);


// ros variables
const char* topic = "speed"; // topic to subscribe to
ros::NodeHandle nh; //ros handler
ros::Subscriber<std_msgs::Float32> sub(topic, &inputRead); // subscriber for our topic

// encoder variables
volatile double counter = 0; // counter for pulses
double prevCount = 0; // extra variable to check if velocity is 0
float velocity; // variable to hold current motor speed based on encoder readings
long currTime; // current time

float SPspeed; // our set point that we want to reach
float error; // error to be compensated
float prevError=0; // to be used by differential term
float integralTerm; // error to be compensated in integral
float differentialTerm; // error to be compensated in differential
int outputV; // output voltage by PID
float kp; // P constant
float ki; // I constant
float kd; // D constant
long prevTime = 0; // prev time for use by PID

HardwareSerial Serial3(PB11, PB10);

void setup() {
  // setting up ros
//  (nh.getHardware())->setPort(&Serial1); // setting serial port for ros
//  (nh.getHardware())->setBaud(115200); // set baud rate for ros
//  nh.initNode(); // initializing node handler
//  nh.subscribe(sub); // subscribing to topic

  // setting PID constants
  kp = 10;
  ki = 0;
  kd = 0;
  SPspeed = 0;

  // setting pin modes
  pinMode(A, INPUT_PULLUP);
  pinMode(B, INPUT_PULLUP);
  pinMode(PWM, OUTPUT);
  pinMode(DIRECTION, OUTPUT);

  // attaching interrupts
  attachInterrupt(digitalPinToInterrupt(A), ISR_A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(B), ISR_B, CHANGE);

  Serial3.begin(9600);
}

void loop() {
  if(Serial3.available()){
    SPspeed = Serial3.readString().toInt();
  }
  
  outputV = mapPIDtoVolt(calculatePID()); // calculating PID values and mapping them to volt
  //Serial3.println(outputV);
  Serial3.println(counter);
  if (outputV < 0) { // if value is negative adjust direction
    digitalWrite(DIRECTION, LOW);
  } else {
    digitalWrite(DIRECTION, HIGH);
  }
  outputV = abs(outputV); // get magnitude of voltage 

  if (outputV > 65535) // if output is higher than possible reduce it to max
    outputV = 65535;

  
  analogWrite(PWM, outputV); // set motor speed
  //Serial3.println(counter);
  //Serial3.println(velocity);

  //delay(10);

  // nh.spinOnce();// node handler spin
}



//----------------------------FUNCTIONS---------------------------------

// interrupt function for a
void ISR_A() {
  if (digitalRead(A) != digitalRead(B)) {
    counter++;
  } else {
    counter--;
  }
}

// interrupt function for b
void ISR_B() {
  if (digitalRead(A) == digitalRead(B)) {
    counter++;
  } else {
    counter--;
  }
}


// map PID output (given in rpm) to PWM signal (given in voltage)
int mapPIDtoVolt(float rpm){
  return rpm * 65535 / 350;
}

// parsing message upon receiving from ros topic
void inputRead(const std_msgs::Float32 &speedmsg) {
  SPspeed = speedmsg.data;
}

// calculates PID output value
float calculatePID() {
  currTime = micros(); // getting current time
  float dT = (currTime - prevTime) / 1000000.0; // getting difference in time between last calculation
  prevTime = currTime; // setting new prev time

//  Serial3.print("before");
//  Serial3.println(velocity);
  velocity = 1.0*(counter) / dT; // get counts per second from encoder feedback
  velocity *= 60 / 540.0; // get rpm from counts/sec
  prevCount = counter; // setting new previous count
  counter = 0;
//  Serial3.print("after");
//  Serial3.println(velocity);

  if(SPspeed>350) // if given set point is higher than motor capabilities, reduce down
    SPspeed = 350;
    
  if(SPspeed<0) // if given set point is negative, set to 0
    SPspeed = 0;
    
  error = SPspeed - velocity; // calculating error
  integralTerm += error * dT; // getting I term
  differentialTerm = (error - prevError) / dT; // getting D term
  prevError = error; // setting prev error
  return error * kp + integralTerm * ki + differentialTerm * kd; // return error multiplied by constants
  
}
