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
#include <vector>


/***** ROS *****/
ros::NodeHandle  nh;
force_proximity_ros::ProximityStamped prx_msg0;
ros::Publisher prx_pub0("proximity_sensor0", &prx_msg0);
force_proximity_ros::ProximityStamped prx_msg1;
ros::Publisher prx_pub1("proximity_sensor1", &prx_msg1);

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
unsigned int proximity_value1; // current proximity reading
unsigned int average_value1;   // low-pass filtered proximity reading

//Write a two byte value to a Command Register
void writeToCommandRegister(byte commandCode, byte lowVal, byte highVal)
{
  Wire.beginTransmission(VCNL4040_ADDR);
  Wire.write(commandCode);
  Wire.write(lowVal); //Low byte of command
  Wire.write(highVal); //High byte of command
  Wire.endTransmission(); //Release bus

  Wire1.beginTransmission(VCNL4040_ADDR);
  Wire1.write(commandCode);
  Wire1.write(lowVal); //Low byte of command
  Wire1.write(highVal); //High byte of command
  Wire1.endTransmission(); //Release bus
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
std::vector<unsigned int> readFromCommandRegister(byte commandCode)
{
  Wire.beginTransmission(VCNL4040_ADDR);
  Wire.write(commandCode);
  Wire.endTransmission(false); //Send a restart command. Do not release bus.
  Wire.requestFrom(VCNL4040_ADDR, 2); //Command codes have two bytes stored in them
  unsigned int data0 = Wire.read();
  data0 |= Wire.read() << 8;

  Wire1.beginTransmission(VCNL4040_ADDR);
  Wire1.write(commandCode);
  Wire1.endTransmission(false); //Send a restart command. Do not release bus.
  Wire1.requestFrom(VCNL4040_ADDR, 2); //Command codes have two bytes stored in them
  unsigned int data1 = Wire1.read();
  data1 |= Wire1.read() << 8;

  std::vector<unsigned int> data_vec{data0, data1};
  return data_vec;
}

std::vector<unsigned int> readProximity() {
  startProxSensor();
  auto data = readFromCommandRegister(PS_DATA_L);
  return data;
}

void setup()
{
  nh.getHardware()->setBaud(57600);
  nh.initNode();
  nh.advertise(prx_pub0);
  nh.advertise(prx_pub1);
  while(!nh.connected())
  {
    nh.spinOnce();
  }
  Wire.begin();
  Wire1.begin();
  initVCNL4040();
  delay(10);
  auto tmp = readProximity();
  proximity_value0 = tmp[0];
  proximity_value1 = tmp[1];
  average_value0 = proximity_value0;
  average_value1 = proximity_value1;
}

void loop()
{
  time = millis();

  // Read sensor values
  auto tmp = readProximity();
  proximity_value0 = tmp[0];
  proximity_value1 = tmp[1];
  prx_msg0.proximity.proximity = proximity_value0;
  prx_msg0.proximity.average = average_value0;
  prx_msg1.proximity.proximity = proximity_value1;
  prx_msg1.proximity.average = average_value1;

  prx_pub0.publish(&prx_msg0);
  prx_pub1.publish(&prx_msg1);

  // Do this last
  average_value0 = EA * proximity_value0 + (1 - EA) * average_value0;
  average_value1 = EA * proximity_value1 + (1 - EA) * average_value1;
  while (millis() < time + LOOP_TIME); // enforce constant loop time
  nh.spinOnce();

}
