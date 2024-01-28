//YWROBOT
//Compatible with the Arduino IDE 1.0
//Library version:1.1
#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(0x27, 20, 4); // set the LCD address to 0x27 for a 16 chars and 2 line display

void setup()
{
  lcd.init();                      // initialize the lcd
  // Print a message to the LCD.
printOnLcd("Hello World");
}

void printOnLcd(String c ) {
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print(c);

}
void loop()
{
    

}
