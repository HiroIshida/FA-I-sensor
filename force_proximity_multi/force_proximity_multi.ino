/*
   Brought to you by Sparkfun (orignial code), the Correll Lab at the University of Colorado, Boulder
   and Robotic Materials Inc.

   This software is open source and can be used for any purpose.

   Written for Teensy LC
*/

/***** Library parameters ****/

#include <i2c_t3.h>     // Use <i2c_t3.h> for Teensy and <Wire.h> for Arduino
#include <math.h>

#define WIRE Wire1 // Use for Teensy boards
//#define WIRE Wire  // Use for Arduino


/***** USER PARAMETERS *****/

/********* From the datasheet of the I2C multiplexer ********

  A2 A1 A0
  L L L 112 (decimal), 70 (hexadecimal)
  L L H 113 (decimal), 71 (hexadecimal)
  L H L 114 (decimal), 72 (hexadecimal)
  L H H 115 (decimal), 73 (hexadecimal)
  H L L 116 (decimal), 74 (hexadecimal)
  H L H 117 (decimal), 75 (hexadecimal)
  H H L 118 (decimal), 76 (hexadecimal)
  H H H 119 (decimal), 77 (hexadecimal)
*/
int i2c_multi_id = 112;


int ir_current_ = 8;                     // range = [0, 20]. current = value * 10 mA
int ambient_light_measurement_rate_ = 7; // range = [0, 7]. 1, 2, 3, 4, 5, 6, 8, 10 samples per second
int averaging_function_ = 7;  // range [0, 7] measurements per run are 2**value, with range [1, 2**7 = 128]
int proximity_freq_ = 0; // range = [0 , 3]. 390.625kHz, 781.250kHz, 1.5625MHz, 3.125MHz
unsigned long time;

/***** GLOBAL CONSTANTS *****/
#define VCNL4010_ADDRESS 0x13
#define COMMAND_0 0x80  // starts measurements, relays data ready info
#define PRODUCT_ID 0x81  // product ID/revision ID, should read 0x21
#define IR_CURRENT 0x83  // sets IR current in steps of 10mA 0-200mA
#define AMBIENT_PARAMETER 0x84  // Configures ambient light measures
#define PROXIMITY_MOD 0x8F  // proximity modulator timing

#define LOOP_TIME 10  // loop duration in ms


// Touch/release detection
#define EA 0.3  // exponential average weight parameter / cut-off frequency for high-pass filter

/***** GLOBAL VARIABLES *****/
unsigned int proximity_value[8]; // current proximity reading
unsigned int average_value;   // low-pass filtered proximity reading
signed int  fa2;              // FA-II value;
signed int fa2derivative;     // Derivative of the FA-II value;
signed int fa2deriv_last;     // Last value of the derivative (for zero-crossing detection)
signed int sensitivity = 50;  // Sensitivity of touch/release detection, values closer to zero increase sensitivity

unsigned long start_time;
char cmd;

int  continuous_mode = 1;
int  single_shot = 0;
int  touch_analysis = 1;


void setup()
{
  Serial.begin(115200);
  WIRE.begin();
  delay(50);


  WIRE.beginTransmission(i2c_multi_id);
  WIRE.write(0);
  int errcode = WIRE.endTransmission();
  debug_endTransmission(errcode);
  delay(500);

  // initialize each IR sensor
  for (int i = 0; i < 8; i++) {
    if (i != 7) {
      Serial.println(i);
      delay(50);
      //specify IR sensor
      WIRE.beginTransmission(i2c_multi_id);
      WIRE.write(1 << i);
      debug_endTransmission(WIRE.endTransmission());
      writeByte(AMBIENT_PARAMETER, 0x7F);
      writeByte(IR_CURRENT, ir_current_);
      writeByte(PROXIMITY_MOD, 1); // 1 recommended by Vishay

      delay(50);
      byte temp = readByte(PRODUCT_ID);

      //byte proximityregister = readByte(IR_CURRENT);
      if (temp != 0x21) { // Product ID Should be 0x21 for the 4010 sensor
        Serial.print("IR sensor failed to initialize: id = ");
        Serial.print(i);
        Serial.print(". Should have returned 0x21 but returned ");
        Serial.println(temp, HEX);
      }
      else {
        Serial.print("Initialize ");
        Serial.print(i);
        Serial.println();
      }
      proximity_value[i] = readProximity();
      average_value = proximity_value[i];
      fa2 = 0;
    }
  }
}

void loop()
{


  time = millis();
  if (Serial.available() > 0) {
    // read the incoming byte:
    cmd = Serial.read();
    switch (cmd) {
      case 's' : single_shot = 1; break;
      case 't' : if (touch_analysis == 0) touch_analysis = 1; else touch_analysis = 0; break;
      case 'c' : if (continuous_mode == 0) continuous_mode = 1; else continuous_mode = 0; break;
      case 'h' : Serial.println("c: Toggle continuous mode");
        Serial.println("s: Single-shot measurement");
        Serial.println("t: Toggle touch/release analysis");
        break;
    }

  }

  // Read sensor values
  proximity_value[0] = readProximity();
  fa2deriv_last = fa2derivative;
  fa2derivative = (signed int) average_value - proximity_value[0] - fa2;
  fa2 = (signed int) average_value - proximity_value[0];

  if (continuous_mode || single_shot) {
    Serial.print(proximity_value[0]); Serial.print(","); Serial.print(fa2); //Serial.print(","); Serial.print(fa2derivative);

    if (touch_analysis) {
      Serial.print(",");
      if ((fa2deriv_last < -sensitivity && fa2derivative > sensitivity) || (fa2deriv_last > 50 && fa2derivative < -50)) { // zero crossing detected
        // Serial.print(proximity_value); Serial.print(","); Serial.print(fa2); Serial.print(","); Serial.println(fa2derivative);
        if (fa2 < -sensitivity) // minimum
        {
          Serial.print("T");
        }
        else if (fa2 > sensitivity) // maximum
        {
          Serial.print("R");
        }
      }
      else {
        Serial.print("0");
      }
    }
  }

  if (continuous_mode || single_shot) {
    single_shot = 0;
    Serial.println();
  }

  // Do this last
  average_value = EA * proximity_value[0] + (1 - EA) * average_value;
  while (millis() < time + LOOP_TIME); // enforce constant loop tim
}


unsigned int readProximity() {
  byte temp = readByte(0x80);
  writeByte(0x80, temp | 0x08);  // command the sensor to perform a proximity measure

  while (!(readByte(0x80) & 0x20)); // Wait for the proximity data ready bit to be set
  unsigned int data = readByte(0x87) << 8;
  data |= readByte(0x88);

  return data;
}

unsigned int readAmbient() {
  byte temp = readByte(0x80);
  writeByte(0x80, temp | 0x10);  // command the sensor to perform ambient measure

  while (!(readByte(0x80) & 0x40)); // wait for the proximity data ready bit to be set
  unsigned int data = readByte(0x85) << 8;
  data |= readByte(0x86);

  return data;
}

byte writeByte(byte address, byte data)
{
  WIRE.beginTransmission(VCNL4010_ADDRESS);
  WIRE.write(address);
  WIRE.write(data);
  return debug_endTransmission(WIRE.endTransmission());
}

byte readByte(byte address) {
  WIRE.beginTransmission(VCNL4010_ADDRESS);
  WIRE.write(address);

  debug_endTransmission(WIRE.endTransmission());
  WIRE.requestFrom(VCNL4010_ADDRESS, 1);
  while (!WIRE.available());
  byte data = WIRE.read();
  return data;
}

byte debug_endTransmission(int errcode)
{
  if (false)
  {
    switch (errcode)
    {
      // https://www.arduino.cc/en/Reference/WireEndTransmission
      case 0:
        Serial.println("CAVOK");
        break;
      case 1:
        Serial.println("data too long to fit in transmit buffer ");
        break;
      case 2:
        Serial.println("received NACK on transmit of address ");
        break;
      case 3:
        Serial.println("received NACK on transmit of data");
        break;
      case 4:
        Serial.println("other error");
        break;
    }
  }
  return errcode;
}

