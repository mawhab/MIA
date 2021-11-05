#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps20.h"

#include "Wire.h"


MPU6050 mpu;


// Also note that yaw/pitch/roll angles suffer from gimbal lock (for
// more info, see: http://en.wikipedia.org/wiki/Gimbal_lock)


// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector



// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================
HardwareSerial Serial3(PB11, PB10);
void setup() {
  // join I2C bus (I2Cdev library doesn't do this automatically)
  Wire.begin();
  Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties


  // initialize serial communication
  Serial1.begin(9600);
  Serial3.begin(9600);
  Serial.begin(9600);

  // initialize device
  mpu.initialize();

  devStatus = mpu.dmpInitialize();


  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // Calibration Time: generate offsets and calibrate our MPU6050
    mpu.CalibrateGyro(6);
    // turn on the DMP, now that it's ready
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);


    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    Serial.println(F("DMP ready!"));
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }
}



// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {
  // if programming failed, don't try to do anything
  if (!dmpReady) return;
  // read a packet from FIFO
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet
    // display YawPitchRoll angles in degrees
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    Serial1.print("ypr\t");
    Serial1.print(ypr[0] * 180 / M_PI);
    Serial1.print("\t");
    Serial1.print(ypr[1] * 180 / M_PI);
    Serial1.print("\t");
    Serial1.println(ypr[2] * 180 / M_PI);

    Serial3.print("ypr\t");
    Serial3.print(ypr[0] * 180 / M_PI);
    Serial3.print("\t");
    Serial3.print(ypr[1] * 180 / M_PI);
    Serial3.print("\t");
    Serial3.println(ypr[2] * 180 / M_PI);
  }
}
