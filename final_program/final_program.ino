//ELECTROMAGNETIC LEVITATION STABILIZATION APPARATUS
#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps20.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif
MPU6050 mpu;
#define OUTPUT_READABLE_YAWPITCHROLL
#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy-- is 6)
bool blinkState = false;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}
void setup() {
  pinMode(6, OUTPUT); 
  pinMode(9, OUTPUT);
  pinMode(10, OUTPUT);
  pinMode(11, OUTPUT);
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    // initialize serial communication
    // (115200 chosen because it is required for Teapot Demo output, but it's
    // really up to you depending on your project)
    Serial.begin(115200);
    while (!Serial); // wait for Leonardo enumeration, others continue immediately

    // NOTE: 8MHz or slower host processors, like the Teensy @ 3.3v or Ardunio
    // Pro Mini running at 3.3v, cannot handle this baud rate reliably due to
    // the baud timing being too misaligned with processor ticks. You must use
    // 38400 or slower in these cases, or use some kind of external separate
    // crystal solution for the UART timer.

    // initialize device
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();

    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // wait for ready
    //Serial.println(F("\nSend any character to begin DMP programming and demo: "));
    //while (Serial.available() && Serial.read()); // empty buffer
    //while (!Serial.available());                 // wait for data
    //while (Serial.available() && Serial.read()); // empty buffer again

    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(88);
    mpu.setYGyroOffset(42);
    mpu.setZGyroOffset(98);
    mpu.setZAccelOffset(1785); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        attachInterrupt(0, dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
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
    

    // configure LED for output
    pinMode(LED_PIN, OUTPUT);

}
double Out9=255;
double Out6=255;
double Out11=255;
double Out10=255;

void loop() {
  // put your main code here, to run repeatedly:
  while (true){
    mpuInterrupt = false;
      mpuIntStatus = mpu.getIntStatus();
  
      // get current FIFO count
      fifoCount = mpu.getFIFOCount();
  
      // check for overflow (this should never happen unless our code is too inefficient)
      if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
          // reset so we can continue cleanly
          mpu.resetFIFO();
          Serial.println(F("FIFO overflow!"));
  
      // otherwise, check for DMP data ready interrupt (this should happen frequently)
      } else if (mpuIntStatus & 0x02) {
          // wait for correct available data length, should be a VERY short wait
          while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
  
          // read a packet from FIFO
          mpu.getFIFOBytes(fifoBuffer, packetSize);
          
          // track FIFO count here in case there is > 1 packet available
          // (this lets us immediately read more without waiting for an interrupt)
          fifoCount -= packetSize;
      // display Euler angles in degrees
          mpu.dmpGetQuaternion(&q, fifoBuffer);
          mpu.dmpGetGravity(&gravity, &q);
          mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
          /*Serial.print(ypr[1] * 180/M_PI);
          Serial.print("\t");
          Serial.println(ypr[2] * 180/M_PI);
          Serial.print("\t");*/
          if (ypr[1] * 180/M_PI+4.3>5) {
            //Serial.print("y is pos");
            if (Out9>=20){
              Out9--;
            }
            if (Out11>=20){
              Out11--;
            }
            if (Out6<255){
              Out6++;            
            }
            if (Out10<255){
              Out10++;            
            }
          }
          else if (ypr[1] * 180/M_PI+4.3<-5) {
            //Serial.print("y is neg");
            if (Out6>=20){
              Out6--;
            }
            if (Out10>=20){
              Out10--;
            }
            if (Out9<255){
              Out9++;            
            }
            if (Out11<255){
              Out11++;            
            }
          }
          else if (ypr[2] * 180/M_PI+1.7>5) {
            //Serial.print("z is pos");
            if (Out6>=20){
              Out6--;
            }
            if (Out9>=20){
              Out9--;
            }
            if (Out10<255){
              Out10++;            
            }
            if (Out11<255){
              Out11++;            
            }
          }
          else if (ypr[2] * 180/M_PI+1.7<-5) {
            //Serial.print("z is neg");
            if (Out10>=20){
              Out10--;
            }
            if (Out11>=20){
              Out11--;
            }
            if (Out9<255){
              Out9++;            
            }
            if (Out6<255){
              Out6++;            
            }
          }
          else {
            //Out10=255;
            //Out6=200;
            //Out9=200;
            //Out11=255;
          }
      }
      analogWrite(9,Out9);
      analogWrite(6,Out6);
      analogWrite(10,Out10);
      analogWrite(11,Out11);
      //analogWrite(9,255);
      //analogWrite(6,255);
      //analogWrite(11,255);
      //analogWrite(10,255);
      Serial.print(ypr[1] * 180/M_PI+4.3);
      Serial.print("\t");
      Serial.print(ypr[2] * 180/M_PI+1.7);
      Serial.print("\t");
      Serial.print(Out6);
      Serial.print("\t");
      Serial.print(Out9);
      Serial.println("\t\n");
      Serial.print(Out10);
      Serial.print("\t");
      Serial.print(Out11);
  //    Serial.print("\t\n");
  }
}
