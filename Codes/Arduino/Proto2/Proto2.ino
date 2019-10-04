#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

MPU6050 mpu;

#define RunningLength 10
#define LMBPin 0
#define RMBPin 7
#define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards
#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
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

// my variables
uint8_t dellH;
uint8_t dellV;
uint8_t counter;
float ArrX[RunningLength];
float ArrY[RunningLength];
int8_t ArrLMB[RunningLength];
int8_t ArrRMB[RunningLength];
uint8_t iterator;
uint32_t sum;
float fsum;
uint8_t LMB;
uint8_t RMB;

float X;
float Y;


// Interrupt detection routine
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}

void setup() {
    // Initialization of my variables
    counter = 0;
    dellH = 25;
    dellV = 18;
    pinMode(LMBPin, INPUT);
    pinMode(RMBPin, INPUT);
    //counter = 15;
    X = 0;
    Y = 0;
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    // Zeros all arrays
    zeroRunningArrays();

    Serial.begin(115200);
    while (!Serial); // wait for Leonardo enumeration, others continue immediately

    // Initialization
    Serial.println(F("M Initializing I2C devices..."));
    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);

    // Verify connection
    Serial.println(F("M Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("M MPU6050 connection successful") : F("M MPU6050 connection failed"));

    // load and configure the DMP
    Serial.println(F("M Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788);

    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        Serial.println(F("M Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.print(F("M Enabling interrupt detection (Arduino external interrupt "));
        Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
        Serial.println(F("M )..."));
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("M DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("M DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F("M )"));

        // configure LED for output
        pinMode(LED_PIN, OUTPUT);
    }
    initializeArrays();
}

void loop() {
    // if programming failed, don't try to do anything
    if (!dmpReady) return;

    // wait for MPU interrupt or extra packet(s) available
    while (!mpuInterrupt && fifoCount < packetSize) {
        if (mpuInterrupt && fifoCount < packetSize) {
          // try to get out of the infinite loop 
          fifoCount = mpu.getFIFOCount();
        } 
    }

    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & _BV(MPU6050_INTERRUPT_FIFO_OFLOW_BIT)) || fifoCount >= 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        fifoCount = mpu.getFIFOCount();
        Serial.println(F("M FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & _BV(MPU6050_INTERRUPT_DMP_INT_BIT)) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;

        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
        ypr[1] = ypr[1] * 180/M_PI;
        ypr[2] = ypr[2] * 180/M_PI;
        X = ypr[1];
        Y = ypr[2];
        
        if(X > dellH){
            X = dellH;
        }
        else if(X < -dellH){
            X = -dellH;
        }

        if(Y > dellV){
            Y = dellV;
        }
        else if(Y < -dellV){
            Y = -dellV;
        }

        ArrX[counter] = X;
        ArrY[counter] = Y;

        if(analogRead(LMBPin) > 5){
            ArrLMB[counter] = 1;
        }
        else ArrLMB[counter] = 0;

        if(analogRead(RMBPin) > 5){
            ArrRMB[counter] = 1;
        }
        else ArrRMB[counter] = 0;

        counter = (counter + 1) % RunningLength;

        X = ((getRAvgArrX()+dellH)/(2*dellH)) * 65400;
        Y = ((getRAvgArrY()+dellV)/(2*dellV)) * 65400;
        LMB = getSumArrLMB();
        RMB = getSumArrRMB();
        Serial.print("G ");
        // Serial.print(ypr[0] * 180/M_PI);
        // Serial.print(" ");
        Serial.print(X);
        Serial.print(" ");
        Serial.print(Y);
        Serial.print(" ");
        Serial.print(LMB);
        Serial.print(" ");
        Serial.println(RMB);

        // Blink LED to indicate activity
        blinkState = !blinkState;
        digitalWrite(LED_PIN, blinkState);
  }

//  if(counter == 1){
//    if(analogRead(LMBPin) > 5){
//        Serial.println("L 1");
//    }
//    else Serial.println("L 0");
//  }
//  else if(counter == 2){
//    if(analogRead(RMBPin) > 5){
//        Serial.println("R 1");
//    }
//    else Serial.println("R 0");
//  }
//  counter = (counter + 1) % 3;
}

void zeroRunningArrays(){
    for(iterator = 0; iterator < RunningLength; iterator++){
        ArrLMB[iterator] = 0;
        ArrRMB[iterator] = 0;
        ArrX[iterator] = 0;
        ArrY[iterator] = 0;
    }
}

void initializeArrays(){
    for(iterator = 0; iterator < RunningLength; iterator++){
        // if programming failed, don't try to do anything
        if (!dmpReady) return;

        // wait for MPU interrupt or extra packet(s) available
        while (!mpuInterrupt && fifoCount < packetSize) {
            if (mpuInterrupt && fifoCount < packetSize) {
            // try to get out of the infinite loop 
            fifoCount = mpu.getFIFOCount();
            } 
        }

        // reset interrupt flag and get INT_STATUS byte
        mpuInterrupt = false;
        mpuIntStatus = mpu.getIntStatus();

        // get current FIFO count
        fifoCount = mpu.getFIFOCount();

        // check for overflow (this should never happen unless our code is too inefficient)
        if ((mpuIntStatus & _BV(MPU6050_INTERRUPT_FIFO_OFLOW_BIT)) || fifoCount >= 1024) {
            // reset so we can continue cleanly
            mpu.resetFIFO();
            fifoCount = mpu.getFIFOCount();
            Serial.println(F("M FIFO overflow!"));

        // otherwise, check for DMP data ready interrupt (this should happen frequently)
        } else if (mpuIntStatus & _BV(MPU6050_INTERRUPT_DMP_INT_BIT)) {
            // wait for correct available data length, should be a VERY short wait
            while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

            // read a packet from FIFO
            mpu.getFIFOBytes(fifoBuffer, packetSize);
            
            // track FIFO count here in case there is > 1 packet available
            // (this lets us immediately read more without waiting for an interrupt)
            fifoCount -= packetSize;

            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            ypr[1] = ypr[1] * 180/M_PI;
            ypr[2] = ypr[2] * 180/M_PI;
            X = ypr[1];
            Y = ypr[2];
            if(X > dellH){
                X = dellH;
            }
            else if(X < -dellH){
                X = -dellH;
            }

            if(Y > dellV){
                Y = dellV;
            }
            else if(Y < -dellV){
                Y = -dellV;
            }

            ArrX[iterator] = X;
            ArrY[iterator] = Y;

            if(analogRead(LMBPin) > 5){
                ArrLMB[iterator] = 1;
            }
            else ArrLMB[iterator] = 0;

            if(analogRead(RMBPin) > 5){
                ArrRMB[iterator] = 1;
            }
            else ArrRMB[iterator] = 0;
        }
    }
}

float getRAvgArrX(){
    fsum = 0;
    for(iterator = 0; iterator < RunningLength; iterator++){
        fsum += ArrX[iterator];
    }
    return fsum / RunningLength;
}

float getRAvgArrY(){
    fsum = 0;
    for(iterator = 0; iterator < RunningLength; iterator++){
        fsum += ArrY[iterator];
    }
    return fsum / RunningLength;
}

int8_t getSumArrLMB(){
    sum = 0;
    for(iterator = 0; iterator < RunningLength; iterator++){
        sum += ArrLMB[iterator];
    }
    return sum;
}

int8_t getSumArrRMB(){
    sum = 0;
    for(iterator = 0; iterator < RunningLength; iterator++){
        sum += ArrRMB[iterator];
    }
    return sum;
}
