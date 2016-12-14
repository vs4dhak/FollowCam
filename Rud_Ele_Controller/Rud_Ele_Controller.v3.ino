#include <Servo.h>
#include <Wire.h>
#include <I2C.h>
#include <SoftwareSerial.h>
#include <EEPROM.h>
#include <SPI.h>
#include "SparkFunLSM9DS1.h"

//LSMDS91 Declarations
LSM9DS1 imu;
#define DECLINATION 4
#define LSM9DS1_M_CS  9 
#define LSM9DS1_AG_CS 10

//Input Pin Declarations
int const ELE_RX_PIN = 7;
int const RUD_RX_PIN = 8;
int const AUTO_RX_PIN = 4;

//Output Pin Declarations
int const ELE_TX_PIN = 6;
int const RUD_TX_PIN = 5;

//Delay Declaration
int const DELAY = 0;

//Input Variable Declarations
int ele_input;
int rud_input;
int auto_input;

//Output Variable Declarations
int ele_output;
int rud_output;

//Mode Variable Declarations
bool auto_mode = false;

//Timing Variable Declarations
int dt = 0;
int avg_dt = 0;
int t1 = 0;
int t2 = 0;

//PID Reference and Setpoint Variable Calculations
int x_tracking_setpoint = 160;
int x_tracking_actual = 0;

//Magnetometer Variable Declaration
float mag_x;
float mag_y;
float heading_actual;

//New PID Algo Variable Declarations
float const K_RI = 0.05; //0.04 
float const K_RP = 4; //4
float const K_RD = 0.5; //1
float RI, RP, RD;
int new_err;
int prev_err;

//RPI I2C Variable Declarations
#define SLAVE_ADDRESS 0x06
byte first_byte;
byte second_byte;

//PWM Declarations
Servo ele,rud;

void setup() 
{
  //Initializing the serial port with
  //9600 Baud Rate
  Serial.begin(9600);

  //Initializing the LSMDS91 IMU
  //init_imu();

  //Initializing the RPI I2C Communication
  //Channel
  init_rpi_i2c_comm();

  //Initializing the I/O pins
  init_pins();

  Serial.println("Done init.");
}

void loop()
{
  set_auto_mode();

  record_timing_data();

  read_ele_rud();

  //read_mag();

  set_outputs();
  
  send_outputs();

  //print_data();

  print_line();

  delay(DELAY);
}

//===================================Main Function===================================//

//This function contains the Rudder PID
//control algorithim.
int rud_pid(int rud_input)
{
   int d_err;
   
   //NOTE: RUD > 1480 => CCW
   //NOTE: RUD < 1480 => CW 
   //NOTE: HEADING < 0 => CW
   //NOTE: HEADING > 0 => CCW
  
   new_err = x_tracking_setpoint - x_tracking_actual;
   d_err = new_err - prev_err;
   prev_err = new_err;

   RP = new_err;
   RI = RI + new_err;
   RD = (d_err*1.0)/(avg_dt*1.0);

   int pid_output = (1500) + ( round(K_RI*RI) + K_RP*RP + round(K_RD*RD) );

   if (pid_output > 1970)
    pid_output = 1970;
   else 
      if (pid_output < 1000)
        pid_output = 1000;
  
   return pid_output;
}

//This function sets the Rudder and
//Elevator outputs. If auto mode is on
//then the rudder is set to the PID output.
//Otherwise the manual rudder output is set.
//Elevator is always set to the manual
//output.
void set_outputs()
{
  if (auto_mode == false)
  {
    ele_output = ele_input;
    rud_output = rud_input;
    reset_pid();
  }
  else
  if (auto_mode == true)
  {
    ele_output = ele_input;
    rud_output = rud_pid(rud_input);
  }
}

//This function takes in the degrees
//in float and returns an integer
//by getting the remainder when divided
//by 2
int degree_sections(float degree)
{
  int div_2;

  div_2 = degree / 2;

  return div_2*2;
}

//This function reads the data incoming from the RPI
void receiveData(int byteCount)
{ 
  if (Wire.available()) 
  {
    first_byte = Wire.read();
    second_byte = Wire.read();
    x_tracking_actual = first_byte;
    x_tracking_actual = (x_tracking_actual << 8) | second_byte;
  }
}

//This function resets the PID constants
//to zero.
void reset_pid()
{
    RI = 0;
    RP = 0;
    RD = 0;
    x_tracking_actual = 0;
}

//===================================Main Function===================================//

//===================================Print Function===================================//

//Prints data on serial
void print_data()
{
  //Mode Data
  Serial.print(" AUTO: ");
  Serial.print(auto_mode);

  //Input Data
  Serial.print(" ELE IN: ");
  Serial.print(ele_input);
  Serial.print(" RUD IN: ");
  Serial.print(rud_input);

  //Output Data
  Serial.print(" ELE OUT: ");
  Serial.print(ele_output);
  Serial.print(" RUD OUT: ");
  Serial.print(rud_output);

  //Magnetometer Data
  Serial.print(" Heading: ");
  Serial.print(heading_actual);

  //New PID Data
  Serial.print(" Tracking Act: ");
  Serial.print(x_tracking_actual);
  Serial.print(" KI: ");
  Serial.print(round(K_RI*RI));
  Serial.print(" KP: ");
  Serial.print(K_RP*RP);
  Serial.print(" KD: ");
  Serial.print(round(K_RD*RD)); 
}

//Prints a line so that the serial monitor
//will print the next data on the next line.
void print_line()
{
  Serial.println();
}

//===================================Print Function===================================//

//===================================Magnetometer Reading Functions===================================//

//This function reads the IMU and calculates the heading
void read_mag()
{
  imu.readMag();

  mag_x = imu.calcMag(imu.mx);
  mag_y = imu.calcMag(imu.my);

  mag_x = mag_x - 0.015;
  mag_y = mag_y - 0.015;

  heading_actual = degree_sections(atan2(mag_y,mag_x)*180/M_PI);
}

//===================================Magnetometer Reading Functions===================================//

//===================================Mode Setting Functions===================================//

//Reads the auto PWM signal and sets the
//auto mode variable appropriately.
void set_auto_mode()
{
  auto_input = pulseIn(AUTO_RX_PIN, HIGH);
  
  if (auto_input < 1500)
    auto_mode = false;
  else
  if (auto_input > 1500)
    auto_mode = true;
}

//===================================Mode Setting Functions===================================//

//===================================Reading Input and Sending Output Functions===================================//

//Reads the ele and ail PWM signals
void read_ele_rud()
{
  ele_input = pulseIn(ELE_RX_PIN, HIGH);
  rud_input = pulseIn(RUD_RX_PIN, HIGH);
}

//Sends the ele and rud outputs via PWM signals
void send_outputs()
{
  ele.writeMicroseconds(ele_output);
  rud.writeMicroseconds(rud_output);
}

//===================================Reading Input and Sending Output Functions===================================//

//===================================Timing Function===================================//

//Records the current timestamp,
//subtracts it from the previous timestamp
//to determine the loop time and then 
//averages it to determine the average loop time.
void record_timing_data()
{
  t2 = millis();

  dt = t2-t1;

  t1 = t2;

  avg_dt = (avg_dt + dt)/2;

  Serial.print("Curr T: ");
  Serial.print(t2);
  Serial.print(" Avg Cycle T: ");
  Serial.print(avg_dt);
}

//===================================Timing Function===================================//

//===================================Initialization Functions===================================//

//This function inializes the i2c bus to 
//communicate with the RPI
void init_rpi_i2c_comm()
{
  Wire.begin(SLAVE_ADDRESS);
  Wire.onReceive(receiveData);
  Serial.println("Done RPI I2C Initialization.");
}

//This function initializes each of the I/O pins
void init_pins()
{
  pinMode(ELE_RX_PIN, INPUT);
  pinMode(RUD_RX_PIN, INPUT);
  pinMode(AUTO_RX_PIN, INPUT);
  
  ele.attach(ELE_TX_PIN);
  rud.attach(RUD_TX_PIN);
}

//This function initializes the LSMDS91 IMU
void init_imu()
{
  imu.settings.device.commInterface = IMU_MODE_SPI;
  imu.settings.device.mAddress = LSM9DS1_M_CS;
  imu.settings.device.agAddress = LSM9DS1_AG_CS;

  if (!imu.begin())
  {
    Serial.println("Failed to communicate with LSM9DS1.");
    Serial.println("Double-check wiring.");
    Serial.println("Default settings in this sketch will " \
                  "work for an out of the box LSM9DS1 " \
                  "Breakout, but may need to be modified " \
                  "if the board jumpers are.");
    while (1);
  }

  Serial.println("Done LSMDS91 Initialization.");
}

//===================================Setup Functions===================================//
