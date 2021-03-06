/*******************************************************************************//**
  Automation Direct Stepper Motor Speed Test

  Communicates with stepper motor drivers from Automation Direct over RS 232. Can
  be used on either an Arduino Uno or Mega 2560 board type. Runs the motor at a
  series of different speeds in both directions.
  
  Compatible stepper driver unit types are:
        * STP-DRV-4035
        * STP-DRV-4850
        * STP-DRV-6575
        * STP-DRV-80100 (built & tested)

  Product Source:
  @source https://cdn.automationdirect.com/static/specs/surestepdrive.pdf
  
  @author  Duncan Iglesias
  @company Arkbro Inc.
  @date    August 2017
  @link    https://github.com/djiglesias/Arduino-Libraries
**********************************************************************************/
#include "StepperDriver.h"

/* Board Specific Communications */
#if defined(ARDUINO_AVR_UNO)
  #include <SoftwareSerial.h>
  #define SS_RX     10                    
  #define SS_TX     11                    
  SoftwareSerial *mtr_dvr = new SoftwareSerial(SS_RX,SS_TX);
#elif defined(ARDUINO_AVR_MEGA2560)
  #include <HardwareSerial.h>
  HardwareSerial *mtr_dvr = &Serial2;
#else
#error Unsupported hardware
#endif

/* Baud Rates for Serial Communication */
#define BAUD_STD      115200  // Baud rate on standard serial port.
#define BAUD_MTR      57600   // Baud rate on stepper driver port.

/* StepperDriver Object */
StepperDriver *stp_dvr;       // Global reference for stepper driver.

bool test_case = true;

/*******************************************************************************//**
  Main Setup
**********************************************************************************/
void setup() {
  /* Initialize Serial Ports */
  Serial.begin(BAUD_STD);
  mtr_dvr->begin(BAUD_MTR);

  /* Instantiate Stepper Driver */
  stp_dvr = new StepperDriver(mtr_dvr);
  
  /* Set Motor Parameters */
  stp_dvr->setJogSpeed(1.0);        // Motor Speed (m/s)
  stp_dvr->setJogAccel(10.0);       // Motor Acceleration (m/s/s)
  stp_dvr->setJogDecel(40.0);       // Motor Deceleration (m/s/s)
  stp_dvr->setHomeSpeed(1.0);       // Motor Speed (m/s)
  stp_dvr->setHomeAccel(10.0);      // Motor Acceleration (m/s/s)
  stp_dvr->setHomeDecel(40.0);      // Motor Deceleration (m/s/s)
  stp_dvr->setGearRatio(5.0);       // Gearbox Ratio
  stp_dvr->setPinionDiameter(0.10); // Pinion Diameter (m)
  stp_dvr->saveParams();            // Save parameters to driver 
  
}


/*******************************************************************************//**
  Main Loop
**********************************************************************************/
void loop() {

  if(test_case) {
 
    /* Run Connectivity Tests */
    Serial.println("----------------------------------------------------------");
    Serial.println("Stepper Motor Driver Test Suite");
    Serial.println("----------------------------------------------------------");
    Serial.println("Case 1: Jog Forward Intervals");
    {
      delay(1500);
      Serial.println(">> Jogging forwards @ 0.5km/h");
      stp_dvr->jog('f',0.5); delay(1000);
      Serial.println(">> Jogging forwards @ 1.0km/h");
      stp_dvr->jog('f',1.0); delay(1000);
      Serial.println(">> Jogging forwards @ 1.5km/h");
      stp_dvr->jog('f',1.5); delay(1000);
      Serial.println(">> Jogging forwards @ 1.0km/h");
      stp_dvr->jog('f',1.0); delay(1000);
      Serial.println(">> Jogging forwards @ 0.5km/h");
      stp_dvr->jog('f',0.5); delay(1000);
      Serial.println(">> Stop");
      stp_dvr->stop(); delay(1000);
    }
    Serial.println("----------------------------------------------------------");
    delay(1500);
    Serial.println("Case 2: Jog Backward Intervals");
    {
      delay(1500);
      Serial.println(">> Jogging backwards @ 0.5km/h");
      stp_dvr->jog('b',0.5); delay(1000);
      Serial.println(">> Jogging backwards @ 1.0km/h");
      stp_dvr->jog('b',1.0); delay(1000);
      Serial.println(">> Jogging backwards @ 1.5km/h");
      stp_dvr->jog('b',1.5); delay(1000);
      Serial.println(">> Jogging backwards @ 1.0km/h");
      stp_dvr->jog('b',1.0); delay(1000);
      Serial.println(">> Jogging backwards @ 0.5km/h");
      stp_dvr->jog('b',0.5); delay(1000);
      Serial.println(">> Stop");
      stp_dvr->stop(); delay(1000);
      
    }
    Serial.println("----------------------------------------------------------");
    delay(1500);
    Serial.println("Case 3: Sweeping Speed");
    {
      delay(1500);
      Serial.println(">> Increasing Forward Speed.");
      for (double i=0; i <= 1.5; i += 0.05) {
        stp_dvr->jog('f',i); delay(100);
      }
      Serial.println(">> Increasing Backward Speed.");
      for (double i=1.5; i >= -1.5; i -= 0.1) {
        stp_dvr->jog('f',i); delay(100);
      }
      Serial.println(">> Stopping.");
      for (double i=-1.5; i <= 0; i += 0.1) {
        stp_dvr->jog('f',i); delay(100);
      }
      stp_dvr->stop(); 
      delay(1500);     
    }
    Serial.println("----------------------------------------------------------");
    Serial.println("Case 4: Reset Home");
    {
      delay(1500);  
      Serial.println(">> Reset position."); stp_dvr->zero(); delay(1500);
      if (stp_dvr->getPosition() == 0)
        Serial.println(">> Success.");
      else
        Serial.println(">> Fail.");      
    }
    delay(1500);
    stp_dvr->mtrDisable();
    Serial.println("----------------------------------------------------------");
    Serial.println("End of Testing.");
    Serial.println("----------------------------------------------------------");
    
    test_case = false;
  } 
}
/*********************************************************************************/