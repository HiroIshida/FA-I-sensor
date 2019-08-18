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


/***** ROS *****/
ros::NodeHandle  nh;
force_proximity_ros::ProximityStamped prx_msg;
ros::Publisher prx_pub("proximity_sensor", &prx_msg);

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
/* TODO
signed int fa2;              // FA-II value;
signed int fa2derivative;     // Derivative of the FA-II value;
signed int fa2deriv_last;     // Last value of the derivative (for zero-crossing detection)
*/

class ProxSensor
{
  static const signed int sensitivity = 50;  // Sensitivity of touch/release detection, values closer to zero increase sensitivity
  public:
    unsigned int proximity_value; // current proximity reading
    unsigned int average_value;   // low-pass filtered proximity reading
    bool isInitial;

    ProxSensor(i2c_t3* wire_);
    void writeToCommandRegister(byte commandCode, byte lowVal, byte highVal);
    void startProxSensor();
    void initVCNL4040();
    void readProximity();
    unsigned int readFromCommandRegister(byte commandCode);
    i2c_t3* wire;
};

//Write a two byte value to a Command Register
void ProxSensor::writeToCommandRegister(byte commandCode, byte lowVal, byte highVal)
{
  wire->beginTransmission(VCNL4040_ADDR);
  wire->write(commandCode);
  wire->write(lowVal); //Low byte of command
  wire->write(highVal); //High byte of command
  wire->endTransmission(); //Release bus
}

void ProxSensor::startProxSensor()
{
  //Clear PS_SD to turn on proximity sensing
  //Integrate 8T, Clear PS_SD bit to begin reading
  //Set PS to 16-bit
  writeToCommandRegister(PS_CONF1, 0b00001110, 0b00001000); //Command register, low byte, high byte
}

void ProxSensor::initVCNL4040()
{
  startProxSensor();

  delay(1);
  //Set the options for PS_CONF3 and PS_MS bytes
  //Set IR LED current to 75mA
  writeToCommandRegister(PS_CONF3, 0x00, 0b00000001);
}

unsigned int ProxSensor::readFromCommandRegister(byte commandCode)
{
  Wire.beginTransmission(VCNL4040_ADDR);
  Wire.write(commandCode);
  Wire.endTransmission(false); //Send a restart command. Do not release bus.

  Wire.requestFrom(VCNL4040_ADDR, 2); //Command codes have two bytes stored in them

  unsigned int data = Wire.read();
  data |= Wire.read() << 8;

  return (data);
}

void ProxSensor::readProximity()
{
  startProxSensor();
  unsigned int data = readFromCommandRegister(PS_DATA_L);
  proximity_value = data;
  if(isInitial){
    average_value = proximity_value;
  }else{
    average_value = EA * proximity_value + (1 - EA) * average_value;
  }
}


ProxSensor::ProxSensor(i2c_t3* wire_){
  isInitial = true;
  wire = wire_;
  wire->begin();
  initVCNL4040();
  delay(10);
  readProximity();
}

ProxSensor prox(&Wire);

void setup()
{
  nh.getHardware()->setBaud(57600);
  nh.initNode();
  nh.advertise(prx_pub);
  while(!nh.connected())
  {
    nh.spinOnce();
  }
}

void loop()
{
  time = millis();
  prox.readProximity();
  prx_msg.proximity.proximity = prox.proximity_value;
  prx_msg.proximity.average = prox.average_value;
  prx_pub.publish(&prx_msg);
  while (millis() < time + LOOP_TIME); // enforce constant loop time
  nh.spinOnce();
}
