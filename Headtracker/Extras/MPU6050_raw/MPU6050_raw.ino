#include <LiquidCrystal_I2C.h>
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#define redPin 4    //ACTIVITY LED
#define greenPin 14 //STATUS LED
#define buttonPin 13 
#define OUTPUT_READABLE_ACCELGYRO
bool blinkState = false;
LiquidCrystal_I2C lcd(0x27,16,2);
MPU6050 accelgyro;
int16_t ax, ay, az;
int16_t gx, gy, gz;
void setup() {
 pinMode(redPin,OUTPUT);
 pinMode(greenPin,OUTPUT);
 pinMode(buttonPin,INPUT);
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif
    Serial.begin(38400);
    Serial.println("Initializing I2C devices...");
    Serial.println("Testing device connections...");
    Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

    accelgyro.initialize();
     lcd.init();
     lcd.backlight();
     lcd.home();
     lcd.print(accelgyro.testConnection() ? F("Connected.") : F("Failed"));
     lcd.print(F("Ready!."));
     delay(600);
     pinMode(greenPin, OUTPUT);
     pinMode(redPin, OUTPUT);
     pinMode(buttonPin, INPUT);
 while(digitalRead(buttonPin)){
   delay(250); //for debounce
   ledBlink(redPin,20);
   ledBlink(redPin,20);
    ledBlink(greenPin,50);
 }
     lcd.clear();
}
void ledBlink(unsigned short int pin,unsigned int interval){
  digitalWrite(pin,HIGH);
  delay(interval);
  digitalWrite(pin,LOW);
}

void loop() {
      // read raw accel/gyro measurements from device
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    // these methods (and a few others) are also available
    //accelgyro.getAcceleration(&ax, &ay, &az);
    //accelgyro.getRotation(&gx, &gy, &gz);

        lcd.setCursor(0,0);
        lcd.print("a");
        lcd.setCursor(3,0);
        lcd.print(ax);
        lcd.print(",");
        lcd.setCursor(6,0);
        lcd.print(ay);
        lcd.print(",");
        lcd.setCursor(9,0);
        lcd.print(az);
        lcd.setCursor(0,1);
        lcd.print("g");
        lcd.setCursor(3,1);
        lcd.print(gx);
        lcd.print(",");
        lcd.setCursor(6,1);
        lcd.print(gy);
        lcd.print(",");
        lcd.setCursor(9,1);
        lcd.print(gz);
    
        Serial.print("a/g:\t");
        Serial.print(ax); Serial.print("\t");
        Serial.print(ay); Serial.print("\t");
        Serial.print(az); Serial.print("\t");
        Serial.print(gx); Serial.print("\t");
        Serial.print(gy); Serial.print("\t");
        Serial.println(gz);
    blinkState = !blinkState;
    digitalWrite(greenPin, blinkState);
    delay(350);
    lcd.home();
}
