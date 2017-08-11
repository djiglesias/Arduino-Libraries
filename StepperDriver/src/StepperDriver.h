/*******************************************************************************//**
  StepperDriver.h - Serial Stepper Driver - Version 1.
  Copyright (c) 2017 Duncan Iglesias.  All right reserved.

  Communicates with stepper motor drivers from Automation Direct over RS 232. 
  Compatible types are:
        * STP-DRV-4035
        * STP-DRV-4850
        * STP-DRV-6575
        * STP-DRV-80100 (built & tested)

  @source https://cdn.automationdirect.com --> [MAIN] 
  @source [MAIN]/static/specs/surestepdrive.pdf
  @source [MAIN]/static/manuals/surestepmanual/scl_manual.pdf
  
  @author  Duncan Iglesias
  @company Arkbro Inc.
  @date    August 2017
  @link    https://github.com/djiglesias/Arduino-Libraries

  Licence:
    This library is free software; you can redistribute it and/or modify it 
    under the terms of the GNU Lesser General Public License as published by
    the Free Software Foundation; either version 2.1 of the License, or (at
     your option) any later version.

    This library is distributed in the hope that it will be useful, but 
    WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
    or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU Lesser General Public 
    License for more details.

    You should have received a copy of the GNU Lesser General Public License 
    along with this library; if not, write to the Free Software Foundation, 
    Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA

**********************************************************************************/
#ifndef _STP_DRV_H
#define _STP_DVR_H

#include "Arduino.h"
#include "HashMap.h"

/* Determine Board Type */
#if defined(ARDUINO_AVR_UNO)        // Arduino Uno board.
  #include <SoftwareSerial.h>
  #define BOARD UNO
#elif defined(ARDUINO_AVR_MEGA2560) // Arduino Mega (2560) board.
  #include <HardwareSerial.h>
  #define BOARD MEGA
#else                               // Not a supported board.
  #error Unsupported hardware
#endif

/* Motor Kinematics */
#define INIT_JOG_ACC      10.00       // Default jog acceleration.
#define INIT_JOG_DEC      40.00       // Default jog deceleration.
#define INIT_JOG_VEL      1.00        // Defualt jog speed.
#define INIT_MTR_ACC      10.00       // Default autonomous acceleration.
#define INIT_MTR_DEC      40.00       // Default autonomous deceleration.
#define INIT_MTR_VEL      1.00        // Default autonomous speed.

/* Drive Train Parameters */
#define GEAR_RATIO        1.0         // Gear ratio of gearbox.
#define PINION_DIAMETER   0.1         // Diameter of pinon (m).
#define SCALE             1.0         // Erroneous scaling for drift error.
#define STEP_RESOLUTION   200         // Steps per revolution.

/* SCL Command Library */
#define ACCEL_RATE        "AC"        // Request acceleration rate (rev/s/s).
#define ALARM_CODE        "AL"        // Request alarm codes.
#define ALARM_RESET       "AR"        // Reset motor alarms.
#define BUFFER_STATUS     "BS"        // Request buffer status.
#define COMM_ERROR        "CE"        // Request communication error.
#define COMMENCE_JOG      "CJ"        // Commence motor into jog mode.
#define CHANGE_SPEED      "CS"        // Change speed while jogging.
#define DECEL_RATE        "DE"        // Set deceleration rate (rev/s/s).
#define DISTANCE          "DI"        // Set distance/position (steps).
#define IMMEDIATE_POS     "IP"        // Request immediate position.
#define JOG_ACCEL         "JA"        // Set jog acceleration (rev/s/s).
#define JOG_DISABLE       "JD"        // Disable jog mode.
#define JOG_ENABLE        "JE"        // Enable jog mode.
#define JOG_DECEL         "JL"        // Set jog deceleration (rev/s/s).
#define JOG_SPEED         "JS"        // Set jog speed (rev/s).
#define MOTOR_DISABLE     "MD"        // Disable current to motor.
#define MOTOR_ENABLE      "ME"        // Enable current to motor.
#define REQUEST_STATUS    "RS"        // Request motor status.
#define SAVE_PARAMS       "SA"        // Save current settings.
#define STATUS_CODE       "SC"        // Request status code.
#define STOP_JOG          "SJ"        // Stop jogging.
#define STOP_KILL_BUF     "SKD"       // Stop and kill buffer.
#define SET_POSITION      "SP"        // Set current position.
#define SEND_STRING       "SS"        // Send string after execution.
#define STOP_NOW          "ST"        // Stop motor now.
#define SET_VELOCITY      "VE"        // Set velocity.

/* Common Compound Commands */
#define CR                "\r"        // Carriage return character.
#define CMD_DISABLE       "MD\r"      // Disable current to the motor.
#define CMD_ENABLE        "ME\r"      // Enable current to the motor.
#define CMD_FORM          "IFD\r"     // Set decimal format for position.
#define CMD_GOTO          "FP\r"      // Feed to desired position.
#define CMD_HOME          "FP0\r"     // Feed to position zero.
#define CMD_JOG_VEL       "JA10.00\r" // Set jog acceleration.
#define CMD_JOG_REV       "JS2.00\r"  // Set jog speed.
#define CMD_JOG1          "JS0\r"     // Jog speed set to zero.
#define CMD_JOG2          "CJ\r"      // Continue jogging.
#define CMD_MTR_ACC       "AC10.00\r" // Set default acceleration rate.
#define CMD_MTR_DEC       "DE10.00\r" // Set default deceleration rate.
#define CMD_MTR_VEL       "VE4.00\r"  // Set default jog speed.
#define CMD_POLL          "IP\r"      // Immediate position.
#define CMD_RESET         "AR\r"      // Alarm code reset.
#define CMD_SAVE          "SA\r"      // Save parameter to non-volatile memory.
#define CMD_STOP          "SKD\r"     // Stop & kill buffer.
#define CMD_ZERO          "SP0\r"     // Set position.
#define ENDL              "\n"        // Newline character.
#define INIT_DIST         "DI-\r"     // Distance/position.
#define NL                "\n"        // Newline character.

/* Hash Map Parameters */
#define HASH_MAP_MS       11
#define HASH_MAP_CE       7
#define HASH_MAP_AL       17

/* Define Error Code Hash Maps */
HashType<uint16_t,char*> hashAL[HASH_MAP_AL]; 
HashType<uint8_t,char*>  hashCE[HASH_MAP_CE]; 
HashType<char,char*>     hashMS[HASH_MAP_MS]; 
HashMap <uint16_t,char*> hashMapAL = HashMap<uint16_t,char*>(hashAL, HASH_MAP_AL);
HashMap <uint8_t,char*>  hashMapCE = HashMap<uint8_t,char*>(hashCE, HASH_MAP_CE);
HashMap <char,char*>     hashMapMS = HashMap<char,char*>(hashMS, HASH_MAP_MS);

/* Motor Command Request */
enum MotorCommand {
  JOG_ACC = 0,
  JOG_DEC = 1,
  JOG_VEL = 2,
  MTR_ACC = 3,
  MTR_DEC = 4,
  MTR_VEL = 5,
  ALL_CMD = 6
} mtr_cmd;


/*******************************************************************************//**
  Stepper Driver Header File
**********************************************************************************/
class StepperDriver {

public:
  /* Board Specific Constructor */
  #if defined(ARDUINO_AVR_UNO)
  StepperDriver(SoftwareSerial *ss);
  #else
  StepperDriver(HardwareSerial *hs);
  #endif  
  ~StepperDriver();

  /* Motor Commands */
  void jog(char dir);
  void jog(char dir, double vel);        
  void feedToPosition(double pos);

  /* Common Motor Commands */ 
  inline void home()      {_comm->print(CMD_HOME);}
  inline void jogDisable(){_comm->print(JOG_DISABLE CR);}
  inline void mtrDisable(){_comm->print(CMD_DISABLE);}
  inline void mtrEnable() {_comm->print(CMD_ENABLE);}
  inline void saveParams(){_comm->print(CMD_SAVE);}
  inline void stop()      {sendCmd(CMD_STOP);moving = false;}
  inline void zero()      {_comm->print(CMD_ZERO);}
  
  /* Setter Member Functions */
  inline void inverseDirection(bool inv){direction = (inv) ? -1 : 1; }
  inline void setHomeAccel(double acc){mtr_accel = acc; update(MTR_ACC);}
  inline void setHomeDecel(double dec){mtr_decel = dec; update(MTR_DEC);}
  inline void setHomeSpeed(double vel){mtr_speed = vel; update(MTR_DEC);}
  inline void setGearRatio(double num){gear_ratio = num;}
  inline void setJogAccel(double acc){jog_accel = acc; update(JOG_ACC);}
  inline void setJogDecel(double dec){jog_decel = dec; update(JOG_DEC);}
  inline void setJogSpeed(double vel){jog_speed = vel; update(JOG_VEL);}
  inline void setPinionDiameter(double d){pinion_diameter = d;}
  inline void setStepResolution(int res){step_resolution = res;}
  inline void setScalingFactor(double num){scale = num;}

  /* Getter Member Functions */
  inline String getError(){return error;}
  inline double getJogSpeed(){return jog_speed;}
  inline double getJogAccel(){return jog_accel;}
  inline double getHomeSpeed(){return mtr_speed;}
  inline double getHomeAccel(){return mtr_accel;}
  inline double getHomeDecel(){return mtr_decel;}
  inline double getPosition(){return (position = stepToDist(sendCmd(CMD_POLL)));}
  inline double getGearRatio(){return gear_ratio;}
  inline double getPinionDiameter(){return pinion_diameter;}
  inline double getScalingFactor(){return scale;}
  inline int    getStepResolution(){return step_resolution;}
  inline bool   isActive(){return active;}
  inline bool   isMoving(){return moving;}

private:
  /* Motor Parameters */
  bool   active          = false;
  bool   moving          = false;
  double gear_ratio      = GEAR_RATIO;
  double jog_accel       = INIT_JOG_ACC;
  double jog_decel       = INIT_JOG_DEC;
  double jog_speed       = INIT_JOG_VEL;
  double mtr_accel       = INIT_MTR_ACC;
  double mtr_decel       = INIT_MTR_DEC;
  double mtr_speed       = INIT_MTR_VEL;
  double pinion_diameter = PINION_DIAMETER;
  double position        = 0.0;
  double scale           = SCALE;
  double step_resolution = STEP_RESOLUTION;
  int8_t direction       = 1;
  String error;

  /* Board Specific Communication */
  #if defined(ARDUINO_AVR_UNO)
  SoftwareSerial *_comm;
  #else
  HardwareSerial *_comm;
  #endif

  /* Motor Commands */
  double sendCmd(String command);
  void   getAlarmCode(String &error);
  void   getCommError(String &error);
  void   getMotorStatus(String &error);
  void   initErrorCodes();
  void   initialize();
  void   update(uint8_t cmd);
  
  /* Inline Initializer */
  inline void form(){_comm->print(CMD_FORM);}
  
  /* Kinematic Conversions */
  double linToRev(double lin);          // KPH to rev/s.
  double stepToDist(int step);     // Steps to meters.
  int    distToStep(double dist);  // Meters to steps.
};


/*******************************************************************************//**
  Stepper Driver Function Declarations
**********************************************************************************/
#if defined(ARDUINO_AVR_UNO)
StepperDriver::StepperDriver(SoftwareSerial *ss): _comm(ss), active(false) {
  initialize();
}
#else
StepperDriver::StepperDriver(HardwareSerial *hs): _comm(hs), active(false) {
  initialize();
}
#endif
StepperDriver::~StepperDriver() {
  if(_comm)
    _comm->end();
  delete _comm;  
}


/******************************************************************************//**
* Initialize Motor Driver
*
* Activates the motor and prepares it for communication requests.
**********************************************************************************/
void StepperDriver::initialize() {

  /* Initialize Once */
  if (isActive()) return;
  active = !active;

  /* Check Motor Status */
  initErrorCodes();
  getAlarmCode(error);
  getCommError(error);
  getMotorStatus(error);
  mtrEnable();

  /* Stop All Motion */
  stop(); 
  zero();
  form();

  /* Set Default Parameters */
  update(ALL_CMD);

  /* Flush Buffer */
  while (_comm->available() > 0) _comm->read();
}


/******************************************************************************//**
* Linear Speed to Angular Velocity Conversion
*
* Converts the inputed velocity/acceleration into revolutions per second given
* the current mechanical assembly of the motor (aka gear ratio and pinion
* diameter).
*
* @param  lin --> Double describing the speed (km/h) or acceleration (km/h/s).
* @output     --> Double describing the speed in revlutions per second.
**********************************************************************************/
double StepperDriver::linToRev(double lin) {

  return (lin * gear_ratio * scale) / (3.6 * PI * pinion_diameter);
}


/******************************************************************************//**
* Distance to Motor Step Conversion
*
* Converts the inputed distance to motor steps given the current mechanical
* assembly of the motor (aka gear ratio and sprocket diameter).
*
* @param  dist --> Double describing the target distance in meters.
* @output      --> Int describing the distance in motor steps.
**********************************************************************************/
int StepperDriver::distToStep(double dist) {

  return (dist * gear_ratio * step_resolution) / (PI * pinion_diameter * scale);
}


/******************************************************************************//**
* Distance to Motor Step Conversion
*
* Converts the inputed step count from the motor and converts it to a distance
* in meters given the current mechanical assembly of the motor (aka gear ratio
* and sprocket diameter).
*
* @param  step --> Int describing the motor step position.
* @output      --> Double describing the distance.
**********************************************************************************/
double StepperDriver::stepToDist(int step) {

  return (step * PI * pinion_diameter)/(gear_ratio * step_resolution) * scale;
}


/******************************************************************************//**
* Update Kinematics
*
* Recalculate the motor speeds and apply them to the speed controller.
*
* @param  cmd --> uint8_t describing what states to update.
**********************************************************************************/
void StepperDriver::update(uint8_t cmd) {

  /* Send Updated Parameters */
  if (cmd == JOG_ACC || cmd == ALL_CMD) {      // Jogging Acceleration.
    String msg = "JA";
    msg += String(linToRev(this->jog_accel), 2);
    msg += CR;
    _comm->print(msg);
  }
  if (cmd == JOG_DEC || cmd == ALL_CMD) {      // Jogging Deceleration.
    String msg = "JL";
    msg += String(linToRev(this->jog_decel), 2);
    msg += CR;
    _comm->print(msg);
  }
  if (cmd == JOG_VEL || cmd == ALL_CMD) {      // Jogging Speed.
    String msg = "JS";
    msg += String(linToRev(this->jog_speed), 2);
    msg += CR;
    _comm->print(msg);
  }
  if (cmd == MTR_ACC || cmd == ALL_CMD) {      // Point-to-Point Acceleration.
    String msg = "AC";
    msg += String(linToRev(this->mtr_accel), 2);
    msg += CR;
    _comm->print(msg);
  }
  if (cmd == MTR_DEC || cmd == ALL_CMD) {      // Point-to-Point Deceleration.
    String msg = "DE";
    msg += String(linToRev(this->mtr_decel), 2);
    msg += CR;
    _comm->print(msg);
  }
  if (cmd == MTR_VEL || cmd == ALL_CMD) {      // Point-to-Point Speed.
    String msg = "VE";
    msg += String(linToRev(this->mtr_speed), 2);
    msg += CR;
    _comm->print(msg);
  }
}


/******************************************************************************//**
* Motor Jog Request (Direction)
*
* Receives a request to jog and passes it to the stepper driver.
*
* @param dir --> Character indicating direction.
**********************************************************************************/
void StepperDriver::jog(char dir) {
  
  /* Update States */
  direction = (dir == 'f') ? -1 : 1; 

  /* Build & Send Message */
  if (isMoving()) {
    String cmd = "CS";
    cmd += String(direction * linToRev(this->jog_speed));
    cmd += CR;
    _comm->print(cmd);
  } else {
    _comm->print(CMD_JOG1);   // Set Zero Speed.
    _comm->print(CMD_JOG2);   // Enter Jogging Mode.
    String cmd = "CS";        // Select Direction.
    cmd += String(direction * linToRev(this->jog_speed));
    cmd += CR;
    _comm->print(cmd);
  }
  moving = true;
}


/******************************************************************************//**
* Motor Jog Request (Direction, Speed)
*
* Receives a request to jog and passes it to the stepper driver.
*
* @param dir --> Character indicating direction.
* @param vel --> Double indicating motor speed in kph.
**********************************************************************************/
void StepperDriver::jog(char dir, double vel) {

  /* Update Direction & Speed */
  direction = (dir == 'f') ? -1 : 1; 
  this->jog_speed = vel;

  /* Build & Send Message */
  if (isMoving()) {
    String cmd = "CS";
    cmd += String(direction * linToRev(this->jog_speed));
    cmd += CR;
    _comm->print(cmd);
  } else {
    _comm->print(CMD_JOG1);   // Set Zero Speed.
    _comm->print(CMD_JOG2);   // Enter Jogging Mode.
    String cmd = "CS";        // Select Direction.
    cmd += String(direction * linToRev(this->jog_speed));
    cmd += CR;
    _comm->print(cmd);
  }
  moving = true;
}


/******************************************************************************//**
* Feed to Motor Position
*
* Moves the motor to a requested position and stops upon reaching it.
*
* @param pos --> Double indicating requested position.
**********************************************************************************/
void StepperDriver::feedToPosition(double pos) {

  /* Build & Send Message */
  String cmd = "DI";
  cmd += String(distToStep(pos), 2);
  cmd += CR;
  _comm->print(cmd);
  Serial.print(cmd);
}


/******************************************************************************//**
* Send Command to Motor Driver
*
* Moves the motor to a requested position and stops upon reaching it.
*
* @param command  --> String command passed in by reference.
* @output         --> Double representing the returned value from the motor.
**********************************************************************************/
double StepperDriver::sendCmd(String command) {

  /* Send Command to Driver */
  _comm->print(command);
  delay(50);

  /* Parse Response */
  String resp;
  while (_comm->available() > 0 && _comm->read() != '=') {;}
  while (_comm->available() > 0 && _comm->peek() != CR) 
    resp += char(_comm->read());
  while (_comm->available() > 0) _comm->read();
  
  return resp.toDouble();
}


/******************************************************************************//**
* Query Alarm Codes
*
* Checks for alarm codes on start up and relays this information back to the main.
*
* @param  error --> Appends error codes to String passed in by reference.
**********************************************************************************/
void StepperDriver::getAlarmCode(String &error) {

  /* Query Alarm Codes */
  _comm->print(ALARM_RESET CR);
  _comm->print(ALARM_CODE CR);
  delay(10);

  /* Parse Response */
  String resp;
  while (_comm->available() > 0 | _comm->read() == CR) 
    resp += char(_comm->read());
  error += "Alarm Codes:\n>> ";
  error += hashMapAL.getValueOf(resp.toInt());
  error += NL;
}


/******************************************************************************//**
* Query Communication Error Codes
*
* Checks for comm errors on start up and relays this information back to the main.
*
* @param  error --> Appends error codes to String passed in by reference.
**********************************************************************************/
void StepperDriver::getCommError(String &error) {

  /* Query Alarm Codes */
  _comm->print(COMM_ERROR CR);
  delay(10);

  /* Parse Response */
  String resp;
  while (_comm->available() > 0 | _comm->read() == CR) 
    resp += char(_comm->read());
  error += "Comm Errors:\n>> ";
  error += hashMapCE.getValueOf(resp.toInt());
  error += NL;
}


/******************************************************************************//**
* Query Motor Status Codes
*
* Checks the motor state on start up and relays this information back to the main.
*
* @param  error --> Appends error codes to String passed in by reference.
**********************************************************************************/
void StepperDriver::getMotorStatus(String &error) {

  /* Query Alarm Codes */
  _comm->print(REQUEST_STATUS CR);
  delay(100);
  error += "Motor Status:";
  active = true;
  moving = false;

  /* Parse Response & Update States */
  while (_comm->available() > 0 && _comm->read() != '=') {;}
  while (_comm->available() > 0 && _comm->peek() != 13) {
    
    char code = _comm->read();

    /* Update Motor States */
    if (code == 'H' || code == 'J' || code == 'F' || code == 'M')
      moving = true;
    
    if (code == 'D' || code == 'E')
      active = false;

    error += NL;
    error += ">> ";
    error += hashMapMS.getValueOf(code);
  }
  while (_comm->available() > 0) _comm->read();
}


/******************************************************************************//**
* Initialize Hash Map
*
* Populates the hash maps with their respective error codes and motor status 
* messages as outline in the operators' manual.
**********************************************************************************/
void StepperDriver::initErrorCodes() {

  /* Alarm Codes (AL) */
  hashMapAL[0] (0x0000, "No alarms."              );
  hashMapAL[1] (0x0001, "Position limit."         );
  hashMapAL[2] (0x0002, "CCW limit."              );
  hashMapAL[3] (0x0004, "CW limit."               );
  hashMapAL[4] (0x0008, "Over temperature."       );
  hashMapAL[5] (0x0010, "Internal voltage."       );
  hashMapAL[6] (0x0020, "Over voltage."           );
  hashMapAL[7] (0x0040, "Under voltage."          );
  hashMapAL[8] (0x0080, "Over current."           );
  hashMapAL[9] (0x0100, "Open winding."           );
  hashMapAL[10](0x0200, "NOT USED."               );
  hashMapAL[11](0x0400, "Communication error."    );
  hashMapAL[12](0x0800, "Data save failed."       );
  hashMapAL[13](0x1000, "No move."                );
  hashMapAL[14](0x2000, "NOT USED."               );
  hashMapAL[15](0x4000, "NOT USED."               );
  hashMapAL[16](0x8000, "Motor disconnected."     );

  /* Communication Error (CE) Codes */
  hashMapCE[0] (0x0000, "No comm errors."         );
  hashMapCE[1] (0x0001, "Framing error."          );
  hashMapCE[2] (0x0002, "Noise error."            );
  hashMapCE[3] (0x0004, "Overrun error (Rx ovf)." );
  hashMapCE[4] (0x0008, "Rx buffer full."         );
  hashMapCE[5] (0x0010, "Tx buffer full."         );
  hashMapCE[6] (0x0020, "Bad SPI op-code."        );

  /* Motor Status (MS) Codes*/
  hashMapMS[0] ('A',    "Alarm Code is present."  );
  hashMapMS[1] ('D',    "Drive Disabled"          );
  hashMapMS[2] ('E',    "Drive Faulted."          );
  hashMapMS[3] ('H',    "Homing in progress."     );
  hashMapMS[4] ('J',    "Jogging in progress."    );
  hashMapMS[5] ('F',    "Motion in progress."     );
  hashMapMS[6] ('M',    "Motion in progress."     );
  hashMapMS[7] ('R',    "Ready to go."            );
  hashMapMS[8] ('S',    "Stopping a motion."      );
  hashMapMS[9] ('T',    "Wait Time."              );
  hashMapMS[10]('W',    "Wait Input."             );
}


#endif
/*********************************************************************************/