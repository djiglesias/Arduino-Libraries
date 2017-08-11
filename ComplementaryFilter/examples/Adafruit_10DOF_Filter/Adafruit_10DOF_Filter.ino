/*******************************************************************************//**
  Complimentary Filter for Tracking Roll & Pitch

  Tracks the roll and pitch of an object using a complimentary filter on a 
  3-axis accelerometer and gyroscope breakout board from Adafruit (10DOF).
  Magnometer & barometer are excluded for increased rate of reading.

  Adapted from an existing source.

  @source https://www.adafruit.com/product/1604
  @source http://www.geekmomprojects.com/gyroscopes-and-accelerometers-on-a-chip/
  
  @author  Duncan Iglesias
  @company Arkbro Inc.
  @date    August 2017
  @link    https://github.com/djiglesias/Arduino-Libraries
**********************************************************************************/
#include <Adafruit_LSM303_U.h>
#include <Adafruit_L3GD20_U.h>
#include "ComplementaryFilter.h"

#define FLUSH                 20

Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(30301);
Adafruit_L3GD20_Unified       gyro  = Adafruit_L3GD20_Unified(20);

ComplementaryFilter<float>   comp_roll, comp_pitch;

/* Calibration Variables for Raw Data */
float bias_x_accel = 0;
float bias_y_accel = 0;
float bias_z_accel = 0;
float bias_x_gyro  = 0;
float bias_y_gyro  = 0;
unsigned long last_read_time;


/*******************************************************************************//**
  Calibrate Sensors
**********************************************************************************/
void calibrate_sensors() {
    
  /* Flush Initial Values from IMU */
  for (int i = 0; i < FLUSH; i++) {
    sensors_event_t accel_event; accel.getEvent(&accel_event);
    sensors_event_t gyro_event; gyro.getEvent(&gyro_event);  
    delay(10);
  }

  /* Collect Samples for Bias */
  for (int i = 0; i < FLUSH; i++) {
    sensors_event_t accel_event; accel.getEvent(&accel_event);
    sensors_event_t gyro_event; gyro.getEvent(&gyro_event); 
     
    bias_x_accel += accel_event.acceleration.x/FLUSH;
    bias_y_accel += accel_event.acceleration.y/FLUSH;
    bias_z_accel += accel_event.acceleration.z/FLUSH;
    bias_x_gyro  += gyro_event.gyro.x/FLUSH;
    bias_y_gyro  += gyro_event.gyro.y/FLUSH;
    
    delay(10);
  }
}


/*******************************************************************************//**
  Main Setup
**********************************************************************************/
void setup(void)
{
  /* Start Serial */
  Serial.begin(115200);

  /* Initialize & Calibrate the Sensors */
  if(!accel.begin()) Serial.println("LSM303 init failed.");  
  if(!gyro.begin())  Serial.println("L3GD20 init failed.");
  calibrate_sensors();  
  
}


/*******************************************************************************//**
  Main Loop
**********************************************************************************/
void loop(void)
{
  /* Read Raw Sensor Values & Update Time */
  sensors_event_t accel_event; accel.getEvent(&accel_event);
  sensors_event_t gyro_event;  gyro.getEvent(&gyro_event);
  unsigned long t_now = millis(); 
  float dt =(t_now - last_read_time)/1000.0;
  last_read_time = t_now;
  
  /* Condition Raw Values a& Pass to Compelementary Filters */
  float gyro_x = (gyro_event.gyro.x - bias_x_gyro)*dt*RAD_TO_DEG;
  float gyro_y = (gyro_event.gyro.y - bias_y_gyro)*dt*RAD_TO_DEG;
  float ax = accel_event.acceleration.x;
  float ay = accel_event.acceleration.y;
  float az = accel_event.acceleration.z;
  float accel_angle_y = atan(-ax/sqrt(pow(ay,2) + pow(az,2)))*RAD_TO_DEG;
  float accel_angle_x = atan(ay/sqrt(pow(ax,2) + pow(az,2)))*RAD_TO_DEG;
  float filt_roll = comp_roll.get_filtered_result(t_now, accel_angle_x, gyro_x);
  float filt_pitch = comp_pitch.get_filtered_result(t_now, accel_angle_y, gyro_y);

  /* Print Results to Serial Console */
  Serial.print("Roll: ");Serial.print(filt_roll,2);
  Serial.print("\t");
  Serial.print("Pitch: ");Serial.println(filt_pitch,2);
  
}

/*********************************************************************************/