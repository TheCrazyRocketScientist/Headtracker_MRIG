#include <LiquidCrystal_I2C.h>
#include <Wire.h>

LiquidCrystal_I2C lcd(0x27, 16, 2);

void setup() {
     
lcd.home();
lcd.backlight();
lcd.init();
lcd.setCursor(0,0);
  

  
}

void loop() { // do nothing here 

  for (int pos = 0; pos < 10; pos++) {
  lcd.print(pos);
}
delay(100);
lcd.clear();
lcd.home();
pos += 10;
}
