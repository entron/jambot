  #include "Wire.h"
  
  #include "I2Cdev.h"
  
  #include "MPU6050_6Axis_MotionApps20.h"
  
  #include "math.h"
  
  #include <PID_v1.h>
  
  //Define Variables we'll be connecting to
  double Setpoint, Input, Output;
  
  //Specify the links and initial tuning parameters
  PID myPID(&Input, &Output, &Setpoint,12,1,0, REVERSE);
  
  MPU6050 mpu;
  
  // MPU control/status vars
  bool dmpReady = false;  // set true if DMP init was successful
  uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
  uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
  uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
  uint16_t fifoCount;     // count of all bytes currently in FIFO
  uint8_t fifoBuffer[64]; // FIFO storage buffer
  
  // orientation/motion vars
  Quaternion q;           // [w, x, y, z]         quaternion container
  
  
  //Motor pins
  const int pinA1 = 9;
  const int pinB1 = 8;
  const int pinA2 = 11;
  const int pinB2 = 10;
  
  // ================================================================
  // ===               INTERRUPT DETECTION ROUTINE                ===
  // ================================================================
  
  volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
  void dmpDataReady() {
      mpuInterrupt = true;
  }
  
  // ================================================================
  // ===               SET MOTOR SPEED ROUTINE                ===
  // ================================================================
  int a1, b1, a2, b2;
  void setSpeed(int motorSpeed)
  {
    if (motorSpeed >= 0)
    {
      a1 = pinA1;
      a2 = pinA2;
      b1 = pinB1;
      b2 = pinB2;
    }
    else
    {
      a1 = pinB1;
      a2 = pinB2;
      b1 = pinA1;
      b2 = pinA2;
    }
    
    analogWrite(a1, 0);
    analogWrite(a2, 0);
    analogWrite(b1, abs(motorSpeed));
    analogWrite(b2, abs(motorSpeed));
  }
  
  
  // ================================================================
  // ===                      INITIAL SETUP                       ===
  // ================================================================
  
  void setup() {
      Wire.begin();
  
      // initialize serial communication
      // (115200 chosen because it is required for Teapot Demo output, but it's
      // really up to you depending on your project)
      Serial.begin(115200);
      delay(1000);
      while (!Serial); // wait for Leonardo enumeration, others continue immediately
  
      // initialize device
      Serial.println(F("Initializing I2C devices..."));
      mpu.initialize();
  
      // verify connection
      Serial.println(F("Testing device connections..."));
      Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
  
      // load and configure the DMP
      Serial.println(F("Initializing DMP..."));
      devStatus = mpu.dmpInitialize();
      
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
      
      
      //Setup motors
        pinMode(pinA1,OUTPUT);
        pinMode(pinB1,OUTPUT);
        pinMode(pinA2,OUTPUT);
        pinMode(pinB2,OUTPUT);
        
        //PID
        Setpoint = -4.25;
        myPID.SetMode(AUTOMATIC);
        myPID.SetOutputLimits(-255, 255);
  }
  
  
  
  // ================================================================
  // ===                    MAIN PROGRAM LOOP                     ===
  // ================================================================
  
  void loop() {
      // if programming failed, don't try to do anything
//      if (!dmpReady) 
//      {
//        Serial.println(F("Sensor not ready!"));
//        return;
//      }
  
      // reset interrupt flag and get INT_STATUS byte
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
  
          // display quaternion values in easy matrix form: w x y z
          mpu.dmpGetQuaternion(&q, fifoBuffer);
          //Serial.print("quat\t");
//             Serial.print(q.w);
//             Serial.print(",");
//             Serial.print(q.x);
//             Serial.print(",");
             Serial.print(q.y);
             Serial.print(",");
//             Serial.println(q.z);
          
          double theta = 2 * asin(q.y) * 180 / 3.1416;
          Serial.print(theta);
          Serial.print(",");
          if (abs(theta) < 20)
          {
              Input = theta;
      
              myPID.Compute();
              
              //int motorSpeed = (abs(Output) + 60) * Output/abs(Output);
              int motorSpeed = Output;

              
//              if (motorSpeed > 255)
//              {
//                motorSpeed = 255;
//              }
//              else if (motorSpeed < -255)
//              {
//                motorSpeed = -255;
//              }
              
              Serial.println(motorSpeed);
              
              setSpeed(motorSpeed);
          }
          else
          {
            setSpeed(0);
          }
   
      }
  }
