#include <SPI.h>
#include <Wire.h> 
#include <LiquidCrystal_I2C.h>
#include "DigiPot.h"

#define disp 6
#define clk 2
#define dt 3
#define sw 4
#define cs A0
#define TEA5767_mute_left_right  0x06
#define TEA5767_MUTE_FULL        0x80
#define TEA5767_ADC_LEVEL_MASK   0xF0
#define TEA5767_STEREO_MASK      0x80

// Set the LCD address to 0x27 for a 16 chars and 2 line display
LiquidCrystal_I2C lcd(0x27, 20, 4);

const int MODE_VOL = 1;
const int MODE_FRQ = 2;

volatile boolean TurnDetected = false;
volatile boolean up = false;
volatile boolean dispOn = true;
double frequency;
int mode;
const long timeout = 15000;
unsigned char frequencyH = 0;
unsigned char frequencyL = 0;
unsigned int frequencyB;
byte Vol;

volatile long tmr_lmt = 15000;
volatile long tmr_lst = millis();

byte old_stereo=0;
byte stereo=1;

byte old_mute=1;
byte mute=0;

byte old_signal_level=1;
byte signal_level=0;


void loop()
{ 
  if (millis() > (tmr_lst + tmr_lmt))
  {
    dispOn = false;
  } else
  {
    dispOn = true;
  }

  if (dispOn) //dispOn == Kommando
  {
    if (digitalRead(disp) == LOW) // Gegenwaertiger Status
    {
      // Wenn soll = AN (dispOn = HIGH) und ist = AUS (digitalRead(disp) == LOW) --> einschalten, sonst nichts tun
      lcd.display();
      lcd.backlight();
      digitalWrite(disp, HIGH);
    }
  } else
  {
    // Wenn soll = AUS und ist = AN --> ausschalten, sonst nichts tun
    if (digitalRead(disp) == HIGH)
    {
      lcd.noDisplay();
      lcd.noBacklight();
      digitalWrite(disp, LOW);
    }
  }
  
  if (digitalRead(sw) == LOW)
  {
    changeToNextMode();
    while(digitalRead(sw) == LOW)
    {
      // wait for release
    }
    arrow();
    DisplayData();
  }
  
  if (TurnDetected)
  {
    if (mode == MODE_FRQ)
    {
      changeFreuency();
    } else if (mode == MODE_VOL)
    {
      changeLevel();
    }
    TurnDetected = false;
  }
}

void changeMode(int newMode = 0)
{
  switch(newMode)
  {
    case 0: changeMode(MODE_VOL); break;
    case 1: mode = MODE_VOL; break;
    case 2: mode = MODE_FRQ; break;
    default: break;
  }
}

void changeToNextMode()
{
  if (mode == MODE_VOL)
  {
    changeMode(MODE_FRQ);
  } else if (mode == MODE_FRQ)
  {
    changeMode(MODE_VOL);
  }
}

void arrow()
{
  if (mode == MODE_FRQ)
  {
   lcd.setCursor(0,3);
   lcd.print(">");
   lcd.setCursor(0,2);
   lcd.print(" ");
  } else if (mode == MODE_VOL)
  {
   lcd.setCursor(0,2);
   lcd.print(">");
   lcd.setCursor(0,3);
   lcd.print(" ");
  } 
}

void isr0 ()  
{
  tmr_lst = millis();
  TurnDetected = true;
  up = (digitalRead(clk) == digitalRead(dt));
}

void setFrequency()  
{
  frequencyB = 4 * (frequency * 1000000 + 225000) / 32768;
  frequencyH = frequencyB >> 8;
  frequencyL = frequencyB & 0XFF;
  Wire.beginTransmission(0x60);
  Wire.write(frequencyH);
  Wire.write(frequencyL);
  Wire.write(0xB0);
  Wire.write(0x10);
  Wire.write((byte)0x00);
  Wire.endTransmission(); 
} 

void setVolume() 
{
 digitalWrite(cs, LOW);
 SPI.transfer(B00010001);
 SPI.transfer(Vol);
 digitalWrite(cs, HIGH);
}


void setup()
{
  Serial.begin(9600);
  pinMode(cs,OUTPUT);
  pinMode(clk,INPUT);
  pinMode(dt,INPUT);  
  pinMode(sw,INPUT);
  pinMode(disp, OUTPUT);
  digitalWrite(disp, LOW);
  mode = MODE_VOL;
  SPI.begin();
  Wire.begin();
	lcd.begin();
	lcd.backlight();
  frequency = 99.00; //starting Frequency
  Vol = 220; //starting Volume
  attachInterrupt (0, isr0, FALLING);
  setFrequency();
  setVolume();
  TEA5767_read_data();
  arrow();
  DisplayData();
}

void DisplayData()
{
  lcd.setCursor(0, 0);
  lcd.print("                    ");
  lcd.setCursor(0, 0);
  if (stereo)
  {
    lcd.print("Stereo ");
  } else
  {
    lcd.print("Mono   ");
  }
  lcd.print(String((int)signal_level));
  lcd.setCursor(0, 1);
  lcd.print("                    ");
  lcd.setCursor(0, 1);
  if (mode)
  {
    lcd.print("Mode ist TRUE");   
  } else
  {
    lcd.print("Mode ist FALSCH");
  }
  lcd.setCursor(1,2);
  lcd.print("                   ");
  lcd.setCursor(1,2);
  lcd.print("Vol:");
  lcd.print(100-Vol*100/255);
  lcd.print("%");
  lcd.setCursor(1, 3);
  lcd.print("                   ");
  lcd.setCursor(1, 3);
  lcd.print("Freq.: ");
  lcd.print(frequency);
  lcd.print(" MHz");  
}

void changeLevel()
{
  if(up)
  {
    Vol = Vol - 5;
    if (Vol <= 0)
    {
      Vol = 0;
    } 
  }
  else
  {
    Vol = Vol + 5;
    if (Vol >= 255){
      Vol = 255;
    } 
  }
  setVolume();
  DisplayData();
}

int TEA5767_read_data() 
{  
  unsigned char buf[5];
  memset (buf, 0, 5);
  
  Wire.requestFrom (0x60, 5); 

  if (Wire.available ()) {
    for (int i = 0; i < 5; i++) {
      buf[i] = Wire.read ();
    }
        
    stereo = (buf[2] & TEA5767_STEREO_MASK)?1:0;
    signal_level = ((buf[3] & TEA5767_ADC_LEVEL_MASK) >> 4);
    
    return 1;
  } 
  else return 0;
}


String value_to_string(int value)
{  
  String value_string = String(value / 100);
  value_string = value_string + '.' + ((value%100<10)?"0":"") + (value % 100);
  return value_string;
}

void changeFreuency()
{
  if(up)
  {
    if (frequency < 108.0)
    {
      frequency = frequency + 0.1;
    }
  } else
  {
    if (frequency > 87.6) 
    {
      frequency = frequency - 0.1;
    }
  }
  setFrequency();
  TEA5767_read_data();
  DisplayData();
}


