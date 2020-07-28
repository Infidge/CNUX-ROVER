#include <LiquidCrystal.h>
#include <Wire.h>
#include <MiCS6814-I2C.h>
int sensorValue;
LiquidCrystal lcd(7, 6, 5, 4, 3, 2);


void setup() {
  Serial.begin(9600);
  lcd.begin(16, 2);
  Wire.begin();
  lcd.setCursor(0, 0);
  lcd.print("   CNUX ROVER"); 
  delay(100);
  lcd.setCursor(0, 1);
  lcd.print("Init."); 
  delay(300);
  lcd.setCursor(0, 1);
  lcd.print("Init..."); 
  delay(600);
  lcd.setCursor(0, 1);
  lcd.print("Init.............."); 
  delay(1000);
  lcd.clear();
  
  }
  

void loop() {
  sensorValue = analogRead(0);       // read analog input pin 0
Serial.print("AirQua=");
Serial.print(sensorValue, DEC);               // prints the value read
Serial.println(" PPM");
lcd.setCursor(0,0);
lcd.print("Air.Q=");
lcd.print(sensorValue,DEC);
lcd.print(" PPM");
lcd.println("       "); 
lcd.print("  ");
delay(100);    
  delay(500);
  
}
