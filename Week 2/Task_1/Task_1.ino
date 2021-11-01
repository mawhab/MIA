#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps20.h"

#include "Wire.h"

#define IMU_ADDR 0x68

MPU6050 mpu;


// Also note that yaw/pitch/roll angles suffer from gimbal lock (for
// more info, see: http://en.wikipedia.org/wiki/Gimbal_lock)


// MPU control/status vars
float errorx, errory, errorz;
int16_t rawx, rawy, rawz;
bool dmpReady = false;  // set true if DMP init was successful
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// read 2 bytes from a give registry
uint16_t read2Bytes(uint8_t reg){
  uint16_t data=0;
  Wire.beginTransmission(IMU_ADDR);
  Wire.write(reg);
  Wire.endTransmission();
  Wire.requestFrom(IMU_ADDR, 2);
  while(Wire.available() < 2);
  data = Wire.read() << 8 | Wire.read();
  return data;
}

void offsetCalc(){
  /*Here we calculate the gyro data error before we start the loop
  I make the mean of 200 values, that should be enough*/
  for(int i=0; i<200; i++)
  {

    // request and read c and z values from register
    rawx = (read2Bytes(0x43)); 
    rawy = (read2Bytes(0x45));
    rawz = (read2Bytes(0x47));

    
    // add all values read to one varuable
    errorx += (rawx/131); 
    errory += (rawy/131);
    errorz += (rawz/131);
  }

  // get the mean error
  errorx /= 200;
  errory /= 200;
  errorz /= 200;
}



// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================
HardwareSerial Serial3(PB11, PB10);
void setup() {
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    // initialize serial communication
    Serial1.begin(9600);
    Serial3.begin(9600);
    Serial.begin(9600);

    // initialize device
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();

    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // wait for ready
    Serial.println(F("\nSend any character to begin DMP programming and demo: "));
    while (Serial.available() && Serial.read()); // empty buffer
    while (!Serial.available());                 // wait for data
    while (Serial.available() && Serial.read()); // empty buffer again

    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    offsetCalc(); // calculate offset before setting it

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(errorx);
    mpu.setYGyroOffset(errory);
    mpu.setZGyroOffset(errorz);

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
            Serial1.print(ypr[0] * 180/M_PI);
            Serial1.print("\t");
            Serial1.print(ypr[1] * 180/M_PI);
            Serial1.print("\t");
            Serial1.println(ypr[2] * 180/M_PI);

            Serial3.print("ypr\t");
            Serial3.print(ypr[0] * 180/M_PI);
            Serial3.print("\t");
            Serial3.print(ypr[1] * 180/M_PI);
            Serial3.print("\t");
            Serial3.println(ypr[2] * 180/M_PI);
    }
}
