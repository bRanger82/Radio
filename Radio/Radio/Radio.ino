#include <SPI.h>
#include <Wire.h> 
#include <LiquidCrystal_I2C.h>

/* How many shift register chips are daisy-chained. */
#define NUMBER_OF_SHIFT_CHIPS   1

/* Width of data (how many ext lines). */
#define DATA_WIDTH   NUMBER_OF_SHIFT_CHIPS * 8

/* Width of pulse to trigger the shift register to read and latch. */
#define PULSE_WIDTH_USEC   5

/* Optional delay between shift register reads. */
#define POLL_DELAY_MSEC   250

/* You will need to change the "int" to "long" If the
 * NUMBER_OF_SHIFT_CHIPS is higher than 2.
*/
#define BYTES_VAL_T unsigned int

#define cs             A0
#define ploadPin        3  // Connects to Parallel load pin the 165
#define clockPin        4  // Connects to the Clock pin the 165
#define dataPin         5  // Connects to the Q7 pin the 165
#define disp            6
#define clockEnablePin 10  // Connects to Clock Enable pin the 165

BYTES_VAL_T pinValues;
BYTES_VAL_T oldPinValues;

#define FLOAT_LENGTH 4
#define DOUBLE_LENGTH 8
#define UINT_LENGTH 2


#define TEA5767_mute_left_right  0x06
#define TEA5767_MUTE_FULL        0x80
#define TEA5767_ADC_LEVEL_MASK   0xF0
#define TEA5767_STEREO_MASK      0x80
#define disk1 0x50               //Address of 24LC256 eeprom chip
const unsigned int ADDRESS_VOLUME = 8;
const unsigned int ADDRESS_FREQUE = 64;

// Set the LCD address to 0x27 for a 16 chars and 2 line display
LiquidCrystal_I2C lcd(0x27, 20, 4);


boolean dispOn = true;
float frequency = 99.0;
const long timeout = 15000;
unsigned char frequencyH = 0;
unsigned char frequencyL = 0;
unsigned int frequencyB;
byte Vol = 230;

volatile long tmr_lmt = 15000;
volatile long tmr_lst = millis();

byte old_stereo=0;
byte stereo=1;

byte old_mute=1;
byte mute=0;

byte old_signal_level=1;
byte signal_level=0;

void checkSerialinput()
{
  if (Serial.available() > 0)
  {
    dispOn = true;
    tmr_lst = millis();
    String s = Serial.readString();
    if (s.startsWith("UP"))
    {
      changeLevel(true);
    } else if (s.startsWith("DOWN"))
    {
      changeLevel(false);
    } else if (s.startsWith("FUP"))
    {
      changeFreuency(true);
    } else if (s.startsWith("FDOWN"))
    {
      changeFreuency(false);
    } else if (s.startsWith("OE3"))
    {
      frequency = 98.9;
      changeFreuency(true);
    } else if (s.startsWith("WELLE1"))
    {
      frequency = 101.7;
      changeFreuency(true);
    }
    Serial.print("START|");
    Serial.print("FREQ|");
    Serial.print(frequency);
    Serial.print("|VOL|");
    Serial.print(Vol);
    Serial.println("|EOF");
  }
}

float round_to_dp( float in_value, int decimal_place )
{
  float multiplier = powf( 10.0f, decimal_place );
  in_value = roundf( in_value * multiplier ) / multiplier;
  return in_value;
}

void loop()
{ 
  if (millis() > (tmr_lst + tmr_lmt))
  {
    if (dispOn)
    {
      /*
       * Every time the display turns off 
       * check if a value was changed
       * If yes save it to eeprom
       * This should avoid writing to eeprom too many (unnecessary) times
      */
      float tmpFreq = EEPROM_readFloat(disk1, ADDRESS_FREQUE);
      byte tmpVol  = readEEPROM(disk1, ADDRESS_VOLUME);

      if (round_to_dp(frequency, 2) != round_to_dp(tmpFreq, 2) || tmpVol != Vol)
      {
        Serial.print("freq: ");
        Serial.print(frequency);
        Serial.print(" - freq-tmp: ");
        Serial.print(tmpFreq);
        Serial.print(" ::: Vol: ");
        Serial.print(Vol);
        Serial.print(" - Vol_tmp: ");
        Serial.println(tmpVol);
        Serial.println("Save to EEPROM called!");
        saveToEEPROM();
      }
    }
    dispOn = false;
  } else
  {
    dispOn = true;
  }

  checkSerialinput();
  
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
  
  /* Read the state of all zones. */
  pinValues = read_shift_regs();

  /* If there was a chage in state, display which ones changed. */
  if(pinValues != oldPinValues)
  {
      display_pin_values();
      oldPinValues = pinValues;
      dispOn = true;
      tmr_lst = millis();
  }

  delay(POLL_DELAY_MSEC);
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
  delay(50);
} 

void saveToEEPROM()
{
  writeEEPROM(disk1, ADDRESS_VOLUME, Vol);
  delay(50);
  EEPROM_writeFloat(disk1, ADDRESS_FREQUE, frequency);
  delay(50);
}

void setVolume() 
{
 digitalWrite(cs, LOW);
 SPI.transfer(B00010001);
 SPI.transfer(Vol);
 digitalWrite(cs, HIGH);
 delay(50);
}

byte* floatToByteArray(float f) 
{
    byte* ret = malloc(4 * sizeof(byte));
    unsigned int asInt = *((int*)&f);

    int i;
    for (i = 0; i < 4; i++) {
        ret[i] = (asInt >> 8 * i) & 0xFF;
    }

    return ret;
}

void setup()
{
  
  Serial.begin(9600);
  SPI.begin();
  Wire.begin();

  /* Initialize our digital pins... */
  pinMode(cs,OUTPUT);
  pinMode(disp, OUTPUT);
  digitalWrite(disp, LOW);
  pinMode(ploadPin, OUTPUT);
  pinMode(clockEnablePin, OUTPUT);
  pinMode(clockPin, OUTPUT);
  pinMode(dataPin, INPUT);

  digitalWrite(clockPin, LOW);
  digitalWrite(ploadPin, HIGH);

  /* Read in and display the pin states at startup. */
  pinValues = read_shift_regs();
  oldPinValues = pinValues;
    
	lcd.begin();
	lcd.backlight();
  
  frequency = EEPROM_readFloat(disk1, ADDRESS_FREQUE);
  Vol = readEEPROM(disk1, ADDRESS_VOLUME);
  
  if (frequency < 87.6 || frequency > 108.0)
  {
    frequency = 99.0; //default/fall-back
  }
  if (Vol < 1 || Vol > 255)
  {
    Vol = 230; //default/fall-back
  }
  setFrequency();
  setVolume();
  TEA5767_read_data();
  DisplayData();
}


/* This function is essentially a "shift-in" routine reading the
 * serial Data from the shift register chips and representing
 * the state of those pins in an unsigned integer (or long).
*/
BYTES_VAL_T read_shift_regs()
{
    long bitVal;
    BYTES_VAL_T bytesVal = 0;

    /* Trigger a parallel Load to latch the state of the data lines,
    */
    digitalWrite(clockEnablePin, HIGH);
    digitalWrite(ploadPin, LOW);
    delayMicroseconds(PULSE_WIDTH_USEC);
    digitalWrite(ploadPin, HIGH);
    digitalWrite(clockEnablePin, LOW);

    /* Loop to read each bit value from the serial out line
     * of the SN74HC165N.
    */
    for(int i = 0; i < DATA_WIDTH; i++)
    {
        bitVal = digitalRead(dataPin);

        /* Set the corresponding bit in bytesVal.
        */
        bytesVal |= (bitVal << ((DATA_WIDTH-1) - i));

        /* Pulse the Clock (rising edge shifts the next bit).
        */
        digitalWrite(clockPin, HIGH);
        delayMicroseconds(PULSE_WIDTH_USEC);
        digitalWrite(clockPin, LOW);
    }

    return(bytesVal);
}

/* Dump the list of zones along with their current status.
*/
void display_pin_values()
{

    for(int i = 0; i < DATA_WIDTH; i++)
    {
      if((pinValues >> i) & 1) 
      {
        // I am high, do nothing  
      } else
      {
        switch(i)
        {
          case 0: break;
          case 1: frequency = (float)106.4; commitFrequency();break;
          case 2: frequency = (float)101.2; commitFrequency(); break;
          case 3: frequency = (float)99.0; commitFrequency(); break;
          case 4: changeLevel(true); break;
          case 5: changeLevel(false); break;
          case 6: changeFreuency(true); break;
          case 7: changeFreuency(false); break;
          default: Serial.println("Unknown Data Pin");
        }
      }
    }
}

// stored types
union storedFloat {
  float value;
  byte bytes[FLOAT_LENGTH];
};
union storedDouble {
  double value;
  byte bytes[DOUBLE_LENGTH];
};
union storedUnsignedInt {
  unsigned int value;
  byte bytes[UINT_LENGTH];
};

void EEPROM_writeBytes(int address, byte bytes[], unsigned int writeLength, unsigned int eeaddress)
{
  // save bytes
  Wire.beginTransmission(address);
  Wire.write((int)(eeaddress >> 8));   // MSB
  Wire.write((int)(eeaddress & 0xFF)); // LSB

  // send each byte
  for(int i = 0; i < writeLength; i++)
  {
    Wire.write(bytes[i]);
  }
  Wire.endTransmission();

  delay(50);

}

float EEPROM_readBytes(int address, unsigned int readLength, unsigned int eeaddress)
{
  // indicate starting position
  Wire.beginTransmission(address);
  Wire.write((int)(eeaddress >> 8));   // MSB
  Wire.write((int)(eeaddress & 0xFF)); // LSB
  Wire.endTransmission();

  Wire.requestFrom(address, readLength);
  // create structure
  union storedFloat s;
  int i = 0;
  while(Wire.available())
  {
    s.bytes[i++] = Wire.read();
  }

  return s.value;
}

// EEPROM methods
void EEPROM_writeFloat(int address, unsigned int eeaddress, float value)
{
  // cast value to union structure
  union storedFloat s;
  s.value = value;
  EEPROM_writeBytes(address, s.bytes, FLOAT_LENGTH, eeaddress);
}

float EEPROM_readFloat(int address, unsigned int eeaddress)
{
  return EEPROM_readBytes(address, FLOAT_LENGTH, eeaddress);
}

void writeEEPROM(int deviceaddress, unsigned int eeaddress, byte data ) 
{
  Wire.beginTransmission(deviceaddress);
  Wire.write((int)(eeaddress >> 8));   // MSB
  Wire.write((int)(eeaddress & 0xFF)); // LSB
  Wire.write(data);
  Wire.endTransmission();
 
  delay(25);
}
 
byte readEEPROM(int deviceaddress, unsigned int eeaddress ) 
{
  byte rdata = 0xFF;
 
  Wire.beginTransmission(deviceaddress);
  Wire.write((int)(eeaddress >> 8));   // MSB
  Wire.write((int)(eeaddress & 0xFF)); // LSB
  Wire.endTransmission();
 
  Wire.requestFrom(deviceaddress,1);
 
  if (Wire.available()) rdata = Wire.read();
 
  return rdata;
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

void changeLevel(bool up)
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

void commitFrequency()
{
  setFrequency();
  TEA5767_read_data();
  DisplayData();  
}

  
void changeFreuency(bool up)
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

  commitFrequency();
}

