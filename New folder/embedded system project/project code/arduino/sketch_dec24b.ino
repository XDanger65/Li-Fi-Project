

#include <Wire.h> 
#include <LiquidCrystal_I2C.h>

// Set the LCD address to 0x27 for a 16 chars and 2 line display
LiquidCrystal_I2C lcd(0x27, 16, 2);
const int inputPin = A1; // Define the pin number for A1
void setup() {
  Serial.begin(9600);            // initialize UART with baud rate of 9600
	// initialize the LCD
	lcd.begin();

	// Turn on the blacklight and print a message.
	lcd.backlight();
 pinMode(inputPin, INPUT);

  
}

void loop() { 
   
   static int y = 1;
   if (y){
    lcd.print("Welcome everyone");  
    delay(6000);
    --y;
   }
   
   lcd.clear();
   lcd.setCursor(0,0);
   lcd.print("distance : ");
   lcd.setCursor(0,1);
   lcd.print("MS:");
   lcd.setCursor(8,1);
   lcd.print("Ss:");
  int i=0;
  while (Serial.available() > 0) {
 

    char receivedData = Serial.read();
     if (receivedData=='B'){
        lcd.clear();
      lcd.setCursor(2,0);
       lcd.print("system start");
       delay(3000);
       break;
    }
    if (receivedData=='D'){
      //
      lcd.setCursor(3,1);
      lcd.print("D");
      break;
    }
    if (receivedData=='N'){
      lcd.setCursor(3,1);
      lcd.print("NDT");
      //
      break;
    }
    if (receivedData=='Z')
    {
       lcd.setCursor(11,1);
       lcd.print("FIRE");
       break;
    }if (receivedData=='X'){
      lcd.setCursor(11,1);
       lcd.print("NF");
       break;
    }
    if (receivedData=='S'){
        lcd.clear();
      lcd.setCursor(0,0);
       lcd.print("system stoped");
       delay(7000);
       break;
    }
    
    //Serial.print(receivedData);
    lcd.setCursor(10+i,0);
    
    lcd.print(receivedData);
    i++;

      // read one byte from serial buffer and save to receivedData
    
  }
    delay(500);
}
