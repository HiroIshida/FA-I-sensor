 /*
   This code demonstrates the force response of the RM Pressure Sensor eval kit.

   Press c to turn off continuous mode.
   Press h for help.

   This code requires Teensy Arduino to be installed: https://www.pjrc.com/teensy/td_download.html
   Select Teensy LC from the Board Menu. Load the code onto the Teensy.

   The output will be distance reading and a touch or release character.

   Brought to you by SparkFun (orignial code), the Correll Lab at the University
   of Colorado, Boulder and Robotic Materials Inc.

   This software is open source and can be used for any purpose.
*/

/***** Library parameters ****/
#include <i2c_t3.h>     // Use <i2c_t3.h> for Teensy and <Wire.h> for Arduino
#include <math.h>
#include <ros.h>
#include <force_proximity_ros/ProximityStamped.h>
#define WIRE Wire1


/***** ROS *****/
ros::NodeHandle  nh;
force_proximity_ros::ProximityStamped prx_msg0;
ros::Publisher prx_pub("proximity_sensor", &prx_msg0);

/***** USER PARAMETERS *****/

unsigned long time;

/***** GLOBAL CONSTANTS *****/
#define VCNL4040_ADDR 0x60 //7-bit unshifted I2C address of VCNL4040
//Command Registers have an upper byte and lower byte.
#define PS_CONF1 0x03
//#define PS_CONF2 //High byte of PS_CONF1
#define PS_CONF3 0x04
//#define PS_MS //High byte of PS_CONF3
#define PS_DATA_L 0x08

#define LOOP_TIME 10  // loop duration in ms

// Touch/release detection
#define EA 0.3  // exponential average weight parameter / cut-off frequency for high-pass filter

/***** GLOBAL VARIABLES *****/
unsigned int proximity_value0; // current proximity reading
unsigned int average_value0;   // low-pass filtered proximity reading

//Write a two byte value to a Command Register
void writeToCommandRegister(byte commandCode, byte lowVal, byte highVal)
{
  WIRE.beginTransmission(VCNL4040_ADDR);
  WIRE.write(commandCode);
  WIRE.write(lowVal); //Low byte of command
  WIRE.write(highVal); //High byte of command
  WIRE.endTransmission(); //Release bus
}

void startProxSensor()
{
  //Clear PS_SD to turn on proximity sensing
  //Integrate 8T, Clear PS_SD bit to begin reading
  //Set PS to 16-bit
  writeToCommandRegister(PS_CONF1, 0b00001110, 0b00001000); //Command register, low byte, high byte
}

void initVCNL4040()
{
  startProxSensor();

  delay(1);
  //Set the options for PS_CONF3 and PS_MS bytes
  //Set IR LED current to 75mA
  writeToCommandRegister(PS_CONF3, 0x00, 0b00000001);
}

void stopProxSensor()
{
  //Set PS_SD to turn off proximity sensing
  //Set PS_SD bit to stop reading
  writeToCommandRegister(PS_CONF1, 0b00000001, 0b00000000); //Command register, low byte, high byte
}

//Reads a two byte value from a command register
unsigned int readFromCommandRegister(byte commandCode)
{
  WIRE.beginTransmission(VCNL4040_ADDR);
  WIRE.write(commandCode);
  WIRE.endTransmission(false); //Send a restart command. Do not release bus.

  WIRE.requestFrom(VCNL4040_ADDR, 2); //Command codes have two bytes stored in them

  unsigned int data = WIRE.read();
  data |= WIRE.read() << 8;

  return (data);
}

unsigned int readProximity() {
  startProxSensor();
  unsigned int data = readFromCommandRegister(PS_DATA_L);
  return data;
}

void setup()
{
  nh.getHardware()->setBaud(57600);
  nh.initNode();
  nh.advertise(prx_pub);
  while(!nh.connected())
  {
    nh.spinOnce();
  }
  WIRE.begin();
  initVCNL4040();
  delay(10);
  proximity_value0 = readProximity();
  average_value0 = proximity_value0;
}

void loop()
{
  time = millis();

  // Read sensor values
  proximity_value0 = readProximity();
  prx_msg0.proximity.proximity = proximity_value0;
  prx_msg0.proximity.average = average_value0;

  prx_pub.publish(&prx_msg0);

  // Do this last
  average_value0 = EA * proximity_value0 + (1 - EA) * average_value0;
  while (millis() < time + LOOP_TIME); // enforce constant loop time
  nh.spinOnce();

}
