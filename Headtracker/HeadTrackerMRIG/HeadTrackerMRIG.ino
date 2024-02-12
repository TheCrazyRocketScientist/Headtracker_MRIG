#include <LiquidCrystal_I2C.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#define redPin 3    //ACTIVITY LED
#define greenPin 14 //STATUS LED
#define buttonPin 13 
#define INTERRUPT_PIN 12
bool activity = false;
bool blinkState = false;
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer
Quaternion q;
VectorFloat gravity;
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
LiquidCrystal_I2C lcd(0x27,16,2);
MPU6050 mpu;
void setup() {
 pinMode(redPin,OUTPUT);
 pinMode(greenPin,OUTPUT);
 pinMode(buttonPin,INPUT);
 pinMode(INTERRUPT_PIN, INPUT);
 //LCD and LED Setup
 //lcd.begin();
 mpu.initialize();
 lcd.init();
 lcd.backlight();
 lcd.home();
 lcd.print(mpu.testConnection() ? F("Connected.") : F("Failed"));
 lcd.print(F("Ready."));
 delay(600);
devStatus = mpu.dmpInitialize(); //Sends a series of bytes to the sensor, dont understand it completely, but returns 0, after successfully setting up to the dmp.
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip
    // make sure it worked (returns 0 if so)
if (devStatus == 0) {
    // Calibration Time: generate offsets and calibrate our MPU6050
    mpu.CalibrateAccel(6); // Computes some sort of PID measurement, input is the number of iterations.
    mpu.CalibrateGyro(6); // ^^
    mpu.PrintActiveOffsets();
    mpu.setDMPEnabled(true);// sends enable bit to DMP
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();//Reads the interupt status bits, which revert to 0, after being read. In short, it tells us if the register is read.(I think)
    dmpReady = true;
    packetSize = mpu.dmpGetFIFOPacketSize();
}
    else {
      lcd.print(F("Sensor Setup Failed."));
    }
 delay(500);
 lcd.cursor();
 lcd.blink();
 while(digitalRead(buttonPin)){
  delay(250); //for debounce
 //ledBlink(redPin,20);
 //ledBlink(redPin,20);
  ledBlink(greenPin,50);
 }
 lcd.noBlink();
 lcd.clear();
 lcd.home();
 digitalWrite(greenPin,HIGH);
}
void dmpDataReady() {
    mpuInterrupt = true;
}


void ledBlink(unsigned short int pin,unsigned int interval){
  digitalWrite(pin,HIGH);
  delay(interval);
  digitalWrite(pin,LOW);
}

void loop() {
    // if programming failed, don't try to do anything
    if (!dmpReady) return ;
    // read a packet from FIFO
    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet , uses the GetCurrentFIFOPacket function, which manages the FIFO buffer of the DMP, Prevents 
      //potential buffer overflow, and resets
            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            lcd.print((float)(ypr[0] * 180/M_PI));
            lcd.print((float)(ypr[1] * 180/M_PI));
            lcd.print((float)(ypr[2] * 180/M_PI));
        // blink LED to indicate activity
        blinkState = !blinkState;
        digitalWrite(greenPin,blinkState);
}
}
