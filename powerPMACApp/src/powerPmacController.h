/********************************************
 *  powerPmacController.h 
 * 
 *  Power PMAC Asyn motor based on the 
 *  asynMotorController class and on Matthew
 *  Pearsons pmacController classes.
 * 
 *  Alan Greer
 *  21 Jan 2013
 * 
 ********************************************/

#ifndef powerPmacController_H
#define powerPmacController_H

#include "asynMotorController.h"
#include "asynMotorAxis.h"
#include "powerPmacAxis.h"
#include "powerPmacCSAxis.h"
#include "sshDriver.h"

#define PMAC_C_GlobalStatusString "PMAC_C_GLOBALSTATUS"
#define PMAC_C_CommsErrorString "PMAC_C_COMMSERROR"
#define PMAC_C_CPUString "PMAC_C_CPU"
#define PMAC_C_FirmwareString "PMAC_C_FIRMWARE"
#define PMAC_C_DateString "PMAC_C_DATE"
#define PMAC_C_PLCStatus0String "PMAC_C_PLCSTATUS0"
#define PMAC_C_PLCStatus1String "PMAC_C_PLCSTATUS1"
#define PMAC_C_PLCEnableString "PMAC_C_PLCENABLE"
#define PMAC_C_PLCDisableString "PMAC_C_PLCDISABLE"
#define PMAC_C_KillAxisString "PMAC_C_KILLAXIS"
#define PMAC_C_AxisStatus0String "PMAC_C_AXISSTATUS0"
#define PMAC_C_AxisStatus1String "PMAC_C_AXISSTATUS1"

class powerPmacController : public asynMotorController {

 public:
  powerPmacController(const char *portName, const char *lowLevelPortName, int lowLevelPortAddress, int numAxes, double movingPollPeriod, double idlePollPeriod);

  virtual ~powerPmacController();

  asynStatus printConnectedStatus(void);

  /* These are the methods that we override */
  asynStatus writeInt32(asynUser *pasynUser, epicsInt32 value);
  asynStatus writeFloat64(asynUser *pasynUser, epicsFloat64 value);
  void report(FILE *fp, int level);
  powerPmacAxis* getAxis(asynUser *pasynUser);
  powerPmacAxis* getAxis(int axisNo);
  asynStatus poll();

  //Set the axis scale factor.
  asynStatus pmacSetAxisScale(int axis, int scale);

  //Set the open loop encoder axis
  asynStatus pmacSetOpenLoopEncoderAxis(int axis, int encoder_axis);

 protected:
  powerPmacAxis **pAxes_;       /**< Array of pointers to axis objects */

  #define FIRST_PMAC_PARAM PMAC_C_GlobalStatus_
  int PMAC_C_GlobalStatus_;
  int PMAC_C_CommsError_;
  int PMAC_C_CPU_;
  int PMAC_C_Firmware_;
  int PMAC_C_Date_;
  int PMAC_C_PLCStatus0_;
  int PMAC_C_PLCStatus1_;
  int PMAC_C_PLCEnable_;
  int PMAC_C_PLCDisable_;
  int PMAC_C_KillAxis_;
  int PMAC_C_AxisStatus0_;
  int PMAC_C_AxisStatus1_;
  #define LAST_PMAC_PARAM PMAC_C_AxisStatus1_

 private:
  SSHDriver* lowLevelDriver_;
  asynUser* lowLevelPortUser_;
  epicsUInt32 debugFlag_;
  epicsUInt32 movesDeferred_;
  asynStatus lowLevelWriteRead(const char *command, char *response);
  int lowLevelPortConnect(const char *port, int addr, asynUser **ppasynUser, char *inputEos, char *outputEos);
  void debugFlow(const char *message);

  epicsUInt32 getGlobalStatus(void);

  //static class data members

  static const epicsUInt32 PMAC_MAXBUF_;
  static const epicsFloat64 PMAC_TIMEOUT_;


  static const epicsUInt32 PMAC_STATUS1_TRIGGER_MOVE;
  static const epicsUInt32 PMAC_STATUS1_HOMING;
  static const epicsUInt32 PMAC_STATUS1_NEG_LIMIT_SET;
  static const epicsUInt32 PMAC_STATUS1_POS_LIMIT_SET;
  static const epicsUInt32 PMAC_STATUS1_WARN_FOLLOW_ERR;
  static const epicsUInt32 PMAC_STATUS1_ERR_FOLLOW_ERR;
  static const epicsUInt32 PMAC_STATUS1_LIMIT_STOP;
  static const epicsUInt32 PMAC_STATUS1_AMP_FAULT;
  static const epicsUInt32 PMAC_STATUS1_SOFT_MINUS_LIMIT;
  static const epicsUInt32 PMAC_STATUS1_SOFT_PLUS_LIMIT;
  static const epicsUInt32 PMAC_STATUS1_I2T_AMP_FAULT;
  static const epicsUInt32 PMAC_STATUS1_HOME_COMPLETE;
  static const epicsUInt32 PMAC_STATUS1_DESIRED_VELOCITY_ZERO;
  static const epicsUInt32 PMAC_STATUS1_OPEN_LOOP;
  static const epicsUInt32 PMAC_STATUS1_AMP_ENABLED;
  static const epicsUInt32 PMAC_STATUS1_IN_POSITION;
  static const epicsUInt32 PMAC_STATUS1_BLOCK_REQUEST;
  static const epicsUInt32 PMAC_STATUS1_PHASED_MOTOR;


 /*Global status (?)*/
  static const epicsUInt32 PMAC_GSTATUS_NO_SYS_CLOCKS;             
  static const epicsUInt32 PMAC_GSTATUS_FACTORY_DEFAULT;         
  static const epicsUInt32 PMAC_GSTATUS_SYS_FILE_ERROR;              
  static const epicsUInt32 PMAC_GSTATUS_HWARE_CHANGE_DETECTED;        
  static const epicsUInt32 PMAC_GSTATUS_SAVED_CFG_LOAD_ERROR; 
  static const epicsUInt32 PMAC_GSTATUS_PROJECT_LOAD_ERROR;      
  static const epicsUInt32 PMAC_GSTATUS_POWER_ON_FAULT;      
  static const epicsUInt32 PMAC_GSTATUS_WATCHDOG_BIT1;          
  static const epicsUInt32 PMAC_GSTATUS_WATCHDOG_BIT0;          

  static const epicsUInt32 PMAC_HARDWARE_PROB;
  static const epicsUInt32 PMAX_AXIS_GENERAL_PROB1;
  static const epicsUInt32 PMAX_AXIS_GENERAL_PROB2;

  friend class powerPmacAxis;
  friend class powerPmacCSAxis;

};

#define NUM_PMAC_PARAMS (&LAST_PMAC_PARAM - &FIRST_PMAC_PARAM + 1)

#endif /* powerPmacController_H */
