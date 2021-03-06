#include <ros.h>
//#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>

#define PPR 2048 // pulses per revolution

// variables used for ros communication
ros::NodeHandle  nh;

std_msgs::Int32 str_msg;
ros::Publisher chatter("chatter", &str_msg);

//char encoder_data[60];


// setting encoder pins
#define B PB12
#define A PB13

// counter for pulses
long counter = 0;

// interrupt function for a
void ISR_A(){
  if(digitalRead(A)!=digitalRead(B)){
    counter++;
  }else{
    counter--;
  }
}

// interrupt function for b
void ISR_B(){
  if(digitalRead(A)!=digitalRead(B)){
    counter--;
  }else{
    counter++;
  }
}

void setup() {
  // initiailizing ros node and publisher
  (nh.getHardware())->setPort(&Serial1);
  (nh.getHardware())->setBaud(115200);
  nh.initNode();
  nh.advertise(chatter);
  // setting pin modes and ataching interrupts
  pinMode(A, INPUT_PULLUP);
  pinMode(B, INPUT_PULLUP);
  attachInterrupt(A, ISR_A, CHANGE);
  attachInterrupt(B, ISR_B, CHANGE);

}

void loop() {
  // getting degree from count relative to start
  float degree = counter / 4 / PPR * 360;

  // adding degree into encoder data string
  //sprintf(encoder_data, "Current degree of rotation of encoder: %.2f\n", degree);
  
  // adding encoder data to msg to be published
  str_msg.data = counter;

  // publishing msg
  chatter.publish(&str_msg);

  nh.spinOnce();
  delay(500);

}
