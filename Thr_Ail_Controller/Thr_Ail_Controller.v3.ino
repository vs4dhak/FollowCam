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
int const THR_RX_PIN = 7;
int const AIL_RX_PIN = 8;
int const FS_RX_PIN = 3;
int const AUTO_RX_PIN = 2;

//Output Pin Declarations
int const THR_TX_PIN = 5;
int const AIL_TX_PIN = 6;

//Delay Declaration
int const DELAY = 0;

//Input Variable Declarations
int thr_input;
int ail_input;
int fs_input;
int auto_input;

//Output Variable Declarations
int thr_output;
int ail_output;

//Mode Variable Declarations
bool fs_mode = false;
bool auto_mode = false;

//Timing Variable Declarations
int dt = 0;
int avg_dt = 0;
int t1 = 0;
int t2 = 0;

//RPI I2C Variable Declarations
#define SLAVE_ADDRESS 0x04
byte first_byte;
byte second_byte;

//PID Reference and Setpoint Variable Calculations
int y_tracking_setpoint = 120;
int y_tracking_actual = 0;

//New PID Algo Variable Declarations
float ctrl_sig_array[4] = {0,0,0,0};
float err_sig_array[4] = {0,0,0,0};
float pid_output;

//Old PID Algo Variable Declarations
/*float const K_RI = 0; 
float const K_RP = 6;
float const K_RD = 0;
float RI, RP, RD;
int new_err;
int prev_err;*/

//PID Const Declarations
float uc_a = 2.264;
float uc_b = -1.565;
float uc_c = 0.3011;

float ec_a = 0.7138;
float ec_b = -1.39;
float ec_c = 0.6766;
float ec_d = 0.00004519;

//Y-Axis Acceleration Variable Declaration
float y_accel;

//PWM Declarations
Servo thr,ail;

void setup() 
{
  //Initializing the serial port with
  //9600 Baud Rate
  Serial.begin(9600);

  //Initializing the LSMDS91 IMU
  init_imu();

  //Initializing the RPI I2C Communication
  //Channel
  init_rpi_i2c_comm();

  //Initializing the I/O pins
  init_pins();

  Serial.println("Done init.");
}

void loop()
{
  set_fs_mode();

  set_auto_mode();

  record_timing_data();

  read_thr_ail();
  
  //read_accel();

  set_outputs();

  send_outputs();
  
  print_data();

  print_line();

  delay(DELAY);
}

//===================================Main Functions===================================//

//Sets the throttle and aileron outputs.
//First checks if failsafe mode is true or false.
//If the mode is true then the Throttle
//signals are set to zero. Otherwise
//they are set to the manual or automatic values 
//based on the status of the automatic mode variable.
void set_outputs()
{
  if (fs_mode == true)
    thr_output = 1000;
  else
  if (fs_mode == false)
  {
    if (auto_mode == true)
    {
      //shift_err_and_ctrl();
      thr_output = thr_input;//pid_output;
      //thr_output = thr_pid();

      ail_output = ail_input;  
    }
    else
    if (auto_mode == false)
    {
      thr_output = thr_input;
      ail_output = ail_input;
      
      //init_err_and_ctrl_ary();
      //reset_pid(); 
    } 
  }
}

//Contains the new PID Control Algorithim
void shift_err_and_ctrl()
{
  err_sig_array[4] = err_sig_array[3];
  err_sig_array[3] = err_sig_array[2];
  err_sig_array[2] = err_sig_array[1];
  err_sig_array[1] = err_sig_array[0];
  err_sig_array[0] = -(120-y_tracking_actual);

  ctrl_sig_array[3] = ctrl_sig_array[2];
  ctrl_sig_array[2] = ctrl_sig_array[1];
  ctrl_sig_array[1] = ctrl_sig_array[0];
  ctrl_sig_array[0] = uc_a*ctrl_sig_array[1] + uc_b*ctrl_sig_array[2] + uc_c*ctrl_sig_array[3] + 
                      ec_a*err_sig_array[1] + ec_b*err_sig_array[2] + ec_c*err_sig_array[3] + ec_d*err_sig_array[4];

  pid_output = 1200;//+((500/120)*ctrl_sig_array[0]);
 
  if (pid_output > 1995)
    pid_output = 1995;
  else 
    if (pid_output < 1010)
      pid_output = 1010;          
}

//This function initializes the parameters
//for the new Throttle PID Algorithim
void init_err_and_ctrl_ary()
{
  err_sig_array[4] = 0;
  err_sig_array[3] = 0;
  err_sig_array[2] = 0;
  err_sig_array[1] = 0;
  err_sig_array[0] = 0;

  ctrl_sig_array[3] = 0;
  ctrl_sig_array[2] = 0;
  ctrl_sig_array[1] = 0;
  ctrl_sig_array[0] = 0; 
}

//This function reads the data incoming from the RPI
void receiveData(int byteCount)
{ 
  if (Wire.available()) 
  {
    first_byte = Wire.read();
    second_byte = Wire.read();
    y_tracking_actual = first_byte;
    y_tracking_actual = (y_tracking_actual << 8) | second_byte;
    y_tracking_actual = 240 - y_tracking_actual;
  }
}

//This function contains the old Throttle PID
//control algorithim.
/*int thr_pid()
{
   int d_err;
  
   new_err = - (y_tracking_setpoint - y_tracking_actual);
   d_err = new_err - prev_err;
   prev_err = new_err;

   RP = new_err;
   RI = RI + new_err;
   RD = (d_err*1.0)/(avg_dt*1.0);

   int pid_output = (1400) + ( round(K_RI*RI) + round(K_RP*RP) );//+ round(K_RD*RD) );

   if (pid_output > 1990)
    pid_output = 1990;
   else 
      if (pid_output < 1010)
        pid_output = 1010;
  
   return pid_output;
}*/

//This function resets the parameters for the
//old PID algorithim
/*void reset_pid()
{
   RI = 0;
   RP = 0;
   RD = 0;
   y_tracking_actual = 0;
}*/

//===================================Main Function===================================//


//===================================Print Function===================================//

//Prints data on serial
void print_data()
{
  //Mode Data
  //Serial.print(" FS: ");
  //Serial.print(fs_mode);
  //Serial.print(" AUTO: ");
  //Serial.print(auto_mode);

  //Input Data
  /*Serial.print(" THR IN: ");
  Serial.print(thr_input);
  Serial.print(" AIL IN: ");
  Serial.print(ail_input);*/

  //Output Data
  Serial.print(" THR OUT: ");
  Serial.print(thr_output);
  //Serial.print(" AIL OUT: ");
  //Serial.print(ail_output);

  //Accelerometer Data
  //Serial.print(" Y Accel: ");
  //Serial.print(y_accel);

  //New PID Data
  Serial.print(" Setpoint: ");
  Serial.print(y_tracking_setpoint);
  Serial.print(" Reference: ");
  Serial.print(y_tracking_actual);
  Serial.print(" Error: ");
  Serial.print(err_sig_array[0]);
  Serial.print(" Ctrl 0: ");
  Serial.print(ctrl_sig_array[0]);
  Serial.print(" PID Out: ");
  Serial.print(pid_output);

  //Old PID Data
  /*Serial.print(" Setpoint: ");
  Serial.print(y_tracking_setpoint);
  Serial.print(" Reference: ");
  Serial.print(y_tracking_actual);
  Serial.print(" Error: ");
  Serial.print(new_err);
  Serial.print(" KI: ");
  Serial.print(round(K_RI*RI));
  Serial.print(" KP: ");
  Serial.print(round(K_RP*RP));
  Serial.print(" KD: ");
  Serial.print(round(K_RD*RD));*/
}

//Prints a line so that the serial monitor
//will print the next data on the next line.
void print_line()
{
  Serial.println();
}

//===================================Print Function===================================//

//===================================Accelerometer Reading Functions===================================//

void read_accel()
{
  imu.readAccel();

  y_accel = imu.calcAccel(imu.az);
}

//===================================Accelerometer Reading Functions===================================//


//===================================Reading Input and Sending Output Functions===================================//

//Reads the Throttle and Aileron PWM signals
void read_thr_ail()
{
  thr_input = pulseIn(THR_RX_PIN, HIGH);
  ail_input = pulseIn(AIL_RX_PIN, HIGH);
}

//Sends the throttle and ail outputs via PWM signals
void send_outputs()
{
  thr.writeMicroseconds(thr_output);
  ail.writeMicroseconds(ail_output);
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


//===================================Mode Setting Functions===================================//

//Readsthe fail safe PWM signal and sets the
//failsafe mode variable appropriately.
void set_fs_mode()
{
  fs_input = pulseIn(FS_RX_PIN, HIGH);
  
  if (fs_input < 1500)
    fs_mode = false;
  else
  if (fs_input > 1500)
    fs_mode = true;
}

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
  pinMode(THR_RX_PIN, INPUT);
  pinMode(AIL_RX_PIN, INPUT);
  pinMode(AUTO_RX_PIN, INPUT);
  pinMode(FS_RX_PIN, INPUT);

  thr.attach(THR_TX_PIN);
  ail.attach(AIL_TX_PIN);

  Serial.println("Done Input Output Pin Initialization.");
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

