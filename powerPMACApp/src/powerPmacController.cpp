/********************************************
 *  powerPmacController.cpp
 * 
 *  Power PMAC Asyn motor based on the 
 *  asynMotorController class and on Matthew
 *  Pearsons pmacController classes.
 * 
 *  Alan Greer
 *  21 Jan 2013
 * 
 ********************************************/


#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <errno.h>

#include <iostream>
using std::cout;
using std::endl;

#include <epicsTime.h>
#include <epicsThread.h>
#include <epicsExport.h>
#include <epicsString.h>
#include <iocsh.h>
#include <drvSup.h>
#include <registryFunction.h>

#include "asynOctetSyncIO.h"

#include "powerPmacController.h"

static const char *driverName = "powerPmacController";

const epicsUInt32 powerPmacController::PMAC_MAXBUF_ = 1024;
const epicsFloat64 powerPmacController::PMAC_TIMEOUT_ = 5.0;

const epicsUInt32 powerPmacController::PMAC_STATUS1_TRIGGER_MOVE           = (0x1<<31);
const epicsUInt32 powerPmacController::PMAC_STATUS1_HOMING                 = (0x1<<30);
const epicsUInt32 powerPmacController::PMAC_STATUS1_NEG_LIMIT_SET          = (0x1<<29);
const epicsUInt32 powerPmacController::PMAC_STATUS1_POS_LIMIT_SET          = (0x1<<28);
const epicsUInt32 powerPmacController::PMAC_STATUS1_WARN_FOLLOW_ERR        = (0x1<<27);
const epicsUInt32 powerPmacController::PMAC_STATUS1_ERR_FOLLOW_ERR         = (0x1<<26);
const epicsUInt32 powerPmacController::PMAC_STATUS1_LIMIT_STOP             = (0x1<<25);
const epicsUInt32 powerPmacController::PMAC_STATUS1_AMP_FAULT              = (0x1<<24);
const epicsUInt32 powerPmacController::PMAC_STATUS1_SOFT_MINUS_LIMIT       = (0x1<<23);
const epicsUInt32 powerPmacController::PMAC_STATUS1_SOFT_PLUS_LIMIT        = (0x1<<22);
const epicsUInt32 powerPmacController::PMAC_STATUS1_I2T_AMP_FAULT          = (0x1<<21);
const epicsUInt32 powerPmacController::PMAC_STATUS1_HOME_COMPLETE          = (0x1<<15);
const epicsUInt32 powerPmacController::PMAC_STATUS1_DESIRED_VELOCITY_ZERO  = (0x1<<14);
const epicsUInt32 powerPmacController::PMAC_STATUS1_OPEN_LOOP              = (0x1<<13);
const epicsUInt32 powerPmacController::PMAC_STATUS1_AMP_ENABLED            = (0x1<<12);
const epicsUInt32 powerPmacController::PMAC_STATUS1_IN_POSITION            = (0x1<<11);
const epicsUInt32 powerPmacController::PMAC_STATUS1_BLOCK_REQUEST          = (0x1<<9);
const epicsUInt32 powerPmacController::PMAC_STATUS1_PHASED_MOTOR           = (0x1<<8);

/*Global status (?)*/
const epicsUInt32 powerPmacController::PMAC_GSTATUS_NO_SYS_CLOCKS          = (0x1<<8);             
const epicsUInt32 powerPmacController::PMAC_GSTATUS_FACTORY_DEFAULT        = (0x1<<7);         
const epicsUInt32 powerPmacController::PMAC_GSTATUS_SYS_FILE_ERROR         = (0x1<<6);              
const epicsUInt32 powerPmacController::PMAC_GSTATUS_HWARE_CHANGE_DETECTED  = (0x1<<5);        
const epicsUInt32 powerPmacController::PMAC_GSTATUS_SAVED_CFG_LOAD_ERROR   = (0x1<<4); 
const epicsUInt32 powerPmacController::PMAC_GSTATUS_PROJECT_LOAD_ERROR     = (0x1<<3);      
const epicsUInt32 powerPmacController::PMAC_GSTATUS_POWER_ON_FAULT         = (0x1<<2);      
const epicsUInt32 powerPmacController::PMAC_GSTATUS_WATCHDOG_BIT1          = (0x1<<1);          
const epicsUInt32 powerPmacController::PMAC_GSTATUS_WATCHDOG_BIT0          = (0x1<<0);          

const epicsUInt32 powerPmacController::PMAC_HARDWARE_PROB = (PMAC_GSTATUS_NO_SYS_CLOCKS | PMAC_GSTATUS_FACTORY_DEFAULT | PMAC_GSTATUS_SYS_FILE_ERROR | PMAC_GSTATUS_HWARE_CHANGE_DETECTED | PMAC_GSTATUS_SAVED_CFG_LOAD_ERROR | PMAC_GSTATUS_PROJECT_LOAD_ERROR | PMAC_GSTATUS_POWER_ON_FAULT | PMAC_GSTATUS_WATCHDOG_BIT1 | PMAC_GSTATUS_WATCHDOG_BIT0);

const epicsUInt32 powerPmacController::PMAX_AXIS_GENERAL_PROB1 = PMAC_STATUS1_AMP_FAULT;
const epicsUInt32 powerPmacController::PMAX_AXIS_GENERAL_PROB2 = 0;


//C function prototypes, for the functions that can be called on IOC shell
extern "C" {
  asynStatus powerPmacCreateController(const char *portName, const char *lowLevelPortName, int lowLevelPortAddress, 
					 int numAxes, int movingPollPeriod, int idlePollPeriod);
  
  asynStatus pmacCreateAxis(const char *pmacName, int axis);

  asynStatus pmacCreateAxes(const char *pmacName, int numAxes);
  
  asynStatus pmacDisableLimitsCheck(const char *controller, int axis, int allAxes);
  
  asynStatus pmacSetAxisScale(const char *controller, int axis, int scale);
  
  asynStatus pmacSetOpenLoopEncoderAxis(const char *controller, int axis, int encoder_axis);

  
}

powerPmacController::powerPmacController(const char *portName, const char *lowLevelPortName, int lowLevelPortAddress, 
			       int numAxes, double movingPollPeriod, double idlePollPeriod)
  : asynMotorController(portName, numAxes+1, NUM_MOTOR_DRIVER_PARAMS,
			0, // No additional interfaces
			0, // No addition interrupt interfaces
			ASYN_CANBLOCK | ASYN_MULTIDEVICE, 
			1, // autoconnect
			0, 0)  // Default priority and stack size
{
  static const char *functionName = "powerPmacController::powerPmacController";

  debugFlow(functionName);

  //Initialize non static data members
  lowLevelPortUser_ = NULL;
  debugFlag_ = 0;
  movesDeferred_ = 0;

  pAxes_ = (powerPmacAxis **)(asynMotorController::pAxes_);

  // Create controller-specific parameters
  createParam(PMAC_C_GlobalStatusString,       asynParamInt32, &PMAC_C_GlobalStatus_);
  createParam(PMAC_C_CommsErrorString,         asynParamInt32, &PMAC_C_CommsError_);
  createParam(PMAC_C_CPUString,                asynParamOctet, &PMAC_C_CPU_);
  createParam(PMAC_C_FirmwareString,           asynParamOctet, &PMAC_C_Firmware_);
  createParam(PMAC_C_DateString,               asynParamOctet, &PMAC_C_Date_);
  createParam(PMAC_C_PLCStatus0String,         asynParamInt32, &PMAC_C_PLCStatus0_);
  createParam(PMAC_C_PLCStatus1String,         asynParamInt32, &PMAC_C_PLCStatus1_);
  createParam(PMAC_C_PLCEnableString,          asynParamInt32, &PMAC_C_PLCEnable_);
  createParam(PMAC_C_PLCDisableString,         asynParamInt32, &PMAC_C_PLCDisable_);
  // Create axis specific parameters
  createParam(PMAC_C_KillAxisString,           asynParamInt32, &PMAC_C_KillAxis_);
  createParam(PMAC_C_AxisStatus0String,        asynParamInt32, &PMAC_C_AxisStatus0_);
  createParam(PMAC_C_AxisStatus1String,        asynParamInt32, &PMAC_C_AxisStatus1_);

  // Connect our Asyn user to the low level port that is a parameter to this constructor
  if (lowLevelPortConnect(lowLevelPortName, lowLevelPortAddress, &lowLevelPortUser_, "\006", "\r") != asynSuccess) {
    printf("%s: Failed to connect to low level asynOctetSyncIO port %s\n", functionName, lowLevelPortName);
        setIntegerParam(PMAC_C_CommsError_, 1);
  } else {
    /* Create the poller thread for this controller
     * NOTE: at this point the axis objects don't yet exist, but the poller tolerates this */
    setIntegerParam(PMAC_C_CommsError_, 0);
  }

  startPoller(movingPollPeriod, idlePollPeriod, 10);

  callParamCallbacks();
 
}


powerPmacController::~powerPmacController(void) 
{
  //Destructor
}

/**
 * Connect to the underlying low level Asyn port that is used for comms.
 * This uses the asynOctetSyncIO interface, and also sets the input and output terminators.
 *
 * @port - Name of the port to connect to.
 * @addr - Address to connect to.
 * @ppasynUser - Pointer to the asyn user structure.
 * @inputEos - String input EOS.
 * @outputEos - String output EOS.
 */
int powerPmacController::lowLevelPortConnect(const char *port, int addr, asynUser **ppasynUser, const char *inputEos, const char *outputEos)
{
  asynStatus status = asynSuccess;
  char command[64] = {0};
  char response[64] = {0};
 
  static const char *functionName = "pmacController::lowLevelPortConnect";

  debugFlow(functionName);

  // Ensure axes cannot be created prior to connection completion
  this->lock();

  status = pasynOctetSyncIO->connect( port, addr, ppasynUser, NULL);
  if (status) {
    this->unlock();
    asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
	      "pmacController::motorAxisAsynConnect: unable to connect to port %s\n", 
	      port);
    return status;
  }

  status = pasynOctetSyncIO->setInputEos(*ppasynUser, inputEos, strlen(inputEos));

  // Read the CPU type
  sprintf(command, "cpu");
  status = lowLevelWriteRead(command, response, sizeof(response));
  if (response[0] != 0 && status == asynSuccess) {
    setStringParam (PMAC_C_CPU_,  response);
  }

  // Read the firmware version
  sprintf(command, "vers");
  status = lowLevelWriteRead(command, response, sizeof(response));
  if (response[0] != 0 && status == asynSuccess) {
    setStringParam (PMAC_C_Firmware_,  response);
  }

  // Read the firmware date
  sprintf(command, "date");
  status = lowLevelWriteRead(command, response, sizeof(response));
  if (response[0] != 0 && status == asynSuccess) {
    setStringParam (PMAC_C_Date_,  response);
  }

  // Unlock
  this->unlock();

  return status;
}

/**
 * Utilty function to print the connected status of the low level asyn port.
 */
asynStatus powerPmacController::printConnectedStatus()
{
  asynStatus status = asynSuccess;
  int asynManagerConnected = 0;
  
  if (lowLevelPortUser_) {
    status = pasynManager->isConnected(lowLevelPortUser_, &asynManagerConnected);
    if (status) {
      asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, "powerPmacController: Error calling pasynManager::isConnected.\n");
      return asynError;
    } else {
      printf("powerPmacController::printConnectedStatus: isConnected: %d\n", asynManagerConnected);
    }
  }
  return asynSuccess;
}

/**
 * Wrapper for asynOctetSyncIO write/read functions.
 * @param command - String command to send.
 * @response response - String response back.
 */
asynStatus powerPmacController::lowLevelWriteRead(const char *command, char *response, size_t maxlen)
{
  asynStatus status = asynSuccess;

   int eomReason;
   size_t nwrite = 0;
   size_t nread = 0;
   int commsError = 0;
   char sendString[2048];
   char recbuf[PMAC_MAXBUF_];
   size_t strlen_recbuf;

   memset(recbuf, 0, sizeof(recbuf));
   sprintf(sendString, "%s\n", command);
   static const char *functionName = "pmacController::lowLevelWriteRead";

   debugFlow(functionName);

   if (!lowLevelPortUser_) {
     setIntegerParam(this->motorStatusCommsError_, 1);
     return asynError;
   }

   asynPrint(lowLevelPortUser_, ASYN_TRACEIO_DRIVER, "%s: command: %s\n", functionName, command);
   debugFlow("Sending: ");
   debugFlow(command);
   //Make sure the low level port is connected before we attempt comms
   //Use the controller-wide param PMAC_C_CommsError_
   getIntegerParam(PMAC_C_CommsError_, &commsError);

   if (!commsError) {
     status = pasynOctetSyncIO->writeRead(lowLevelPortUser_ ,
					  sendString, strlen(sendString),
					  recbuf, sizeof(recbuf) - 1,
					  PMAC_TIMEOUT_,
					  &nwrite, &nread, &eomReason );
     strlen_recbuf = strlen(recbuf);
     if (strlen_recbuf > 1) {
       if (recbuf[strlen_recbuf-1] == '\n') {
         --strlen_recbuf;
         recbuf[strlen_recbuf] = '\0';
       }
       if (strlen_recbuf > 1) {
         if (recbuf[strlen_recbuf-1] == '\r') {
           --strlen_recbuf;
           recbuf[strlen_recbuf] = '\0';
         }
       }
     }
     if (strlen_recbuf > maxlen - 1) {
       asynPrint(lowLevelPortUser_, ASYN_TRACE_ERROR, "%s: Buffer too short in pasynOctetSyncIO->writeRead. command: %s recbuff: %s\n", functionName, command, recbuf);
       memcpy(response, recbuf, maxlen - 1);
       response[maxlen - 1] = '\0';
     } else {
       memcpy(response, recbuf, strlen_recbuf + 1); /* inlcude '\0' */
     }
     if (status) {
       asynPrint(lowLevelPortUser_, ASYN_TRACE_ERROR, "%s: Error from pasynOctetSyncIO->writeRead. command: %s\n", functionName, command);
       setIntegerParam(this->motorStatusCommsError_, 1);
       setIntegerParam(PMAC_C_CommsError_, 1);
     } else {
       setIntegerParam(this->motorStatusCommsError_, 0);
       setIntegerParam(PMAC_C_CommsError_, 0);
     }
   }

   asynPrint(lowLevelPortUser_, ASYN_TRACEIO_DRIVER, "%s: response: %s\n", functionName, response); 
   debugFlow("Response: ");
   debugFlow(response);

  return status;
}


void powerPmacController::debugFlow(const char *message)
{
  if (debugFlag_ == 1) {
#ifdef DEBUGFLOW_ESCPAPE
    size_t len = strlen(message);
    for (unsigned j = 0; j < len; j++){
      char ch =  message[j];
      if (isprint(ch)) {
        printf("%c", ch);
      } else if (ch == '\n') {
        printf("\\n");
      } else if (ch == '\r') {
        printf("\\r");
      } else {
        printf("\\%03o", ch);
      }
    }
    printf("\n");
#else
    printf("  %s\n", message);
#endif
  }
  asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, "%s\n", message);
}


void powerPmacController::report(FILE *fp, int level)
{
  int axis = 0;
  powerPmacAxis *pAxis = NULL;

  fprintf(fp, "pmac motor driver %s, numAxes=%d, moving poll period=%f, idle poll period=%f\n", 
          this->portName, numAxes_, movingPollPeriod_, idlePollPeriod_);

  if (level > 0) {
    for (axis=0; axis<numAxes_; axis++) {
      pAxis = getAxis(axis);
      if (pAxis){
        fprintf(fp, "  axis %d\n"
                    "    scale = %d\n", 
                pAxis->axisNo_,
                pAxis->scale_);
      }
    }
  }

  // Call the base class method
  asynMotorController::report(fp, level);
}


asynStatus powerPmacController::writeFloat64(asynUser *pasynUser, epicsFloat64 value)
{
  int function = pasynUser->reason;
  asynStatus status = asynError;
  powerPmacAxis *pAxis = NULL;
  char command[64] = {0};
  char response[64] = {0};
  //double encRatio = 1.0;
  //epicsInt32 encposition = 0;
	
  static const char *functionName = "powerPmacController::writeFloat64";

  debugFlow(functionName);

  pAxis = this->getAxis(pasynUser);
  if (!pAxis) {
    return asynError;
  }

  /* Set the parameter and readback in the parameter library. */
  status = pAxis->setDoubleParam(function, value);

  if (function == motorLowLimit_) {
    sprintf(command, "I%d14=%f", pAxis->axisNo_, value/pAxis->scale_);
    asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW,
	      "%s: Setting low limit on controller %s, axis %d to %f\n",
	      functionName, portName, pAxis->axisNo_, value);
  }
  else if (function == motorHighLimit_) {
    sprintf(command, "I%d13=%f", pAxis->axisNo_, value/pAxis->scale_);
    asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW,
	      "%s: Setting high limit on controller %s, axis %d to %f\n",
	      functionName, portName, pAxis->axisNo_, value);
  } 

  //Execute the command.
  if (command[0] != 0 && status == asynSuccess) {
    status = lowLevelWriteRead(command, response, sizeof(response));
  }

  //Call base class method
  //This will handle callCallbacks even if the function was handled here.
  status = asynMotorController::writeFloat64(pasynUser, value);

  return status;

}


asynStatus powerPmacController::writeInt32(asynUser *pasynUser, epicsInt32 value)
{
  int function = pasynUser->reason;
  asynStatus status = asynError;
  powerPmacAxis *pAxis = NULL;
  char command[64] = {0};
  char response[64] = {0};
  static const char *functionName = "powerPmacController::writeInt32";

  debugFlow(functionName);


  pAxis = this->getAxis(pasynUser);
  if (!pAxis) {
    // Maybe controller specific functions
    if (function == PMAC_C_PLCEnable_) {
      sprintf(command, "ENABLE PLC %d", value);
      asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, "%s: Enabling PLC %d on controller %s\n", functionName, value, portName);
    }

    if (function == PMAC_C_PLCDisable_) {
      sprintf(command, "DISABLE PLC %d", value);
      asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, "%s: Disabling PLC %d on controller %s\n", functionName, value, portName);
    }

    //Execute the command.
    if (command[0] != 0) {
      status = lowLevelWriteRead(command, response, sizeof(response));
    } else {
      // no controller specific functions, return error
      return asynError;
    }
  } else {
    /* Set the parameter and readback in the parameter library.  This may be overwritten when we read back the
     * status at the end, but that's OK */
    status = pAxis->setIntegerParam(function, value);

    // Only call the base class method if the functions are not controller specific
    //Call base class method
    //This will handle callCallbacks even if the function was handled here.
    status = asynMotorController::writeInt32(pasynUser, value);
  }
  return status;

}


/** Returns a pointer to an pmacAxis object.
  * Returns NULL if the axis number encoded in pasynUser is invalid.
  * \param[in] pasynUser asynUser structure that encodes the axis index number. */
powerPmacAxis* powerPmacController::getAxis(asynUser *pasynUser)
{
  int axisNo = 0;
    
  getAddress(pasynUser, &axisNo);
  return getAxis(axisNo);
}



/** Returns a pointer to an pmacAxis object.
  * Returns NULL if the axis number is invalid.
  * \param[in] axisNo Axis index number. */
powerPmacAxis* powerPmacController::getAxis(int axisNo)
{
  if ((axisNo < 0) || (axisNo >= numAxes_)) return NULL;
  return pAxes_[axisNo];
}


/** Polls the controller, rather than individual axis.*/
asynStatus powerPmacController::poll()
{
  epicsUInt32 globalStatus = 0;
  static const char *functionName = "powerPmacController::poll";

  debugFlow(functionName);

  if (!lowLevelPortUser_) {
    setIntegerParam(this->motorStatusCommsError_, 1);
    setIntegerParam(PMAC_C_CommsError_, 1);
    return asynError;
  }

  //Set any controller specific parameters. 
  //Some of these may be used by the axis poll to set axis problem bits.
  globalStatus = getGlobalStatus();
  setIntegerParam(this->PMAC_C_GlobalStatus_, ((globalStatus & PMAC_HARDWARE_PROB) != 0));
  
  callParamCallbacks();

  return asynSuccess;
}


/**
 * Read the PMAC global status integer (using a ??? )
 * @return int The global status integer (23 active bits)
 */
epicsUInt32 powerPmacController::getGlobalStatus(void)
{
  char command[1024];
  char response[PMAC_MAXBUF_];
  int nvals = 0;
  int plc[32];
  int plcStatus = 0;
  epicsUInt32 pmacStatus = 0;
  static const char *functionName = "powerPmacController::getGlobalStatus";

  debugFlow(functionName);

  // Changed for Power PMAC status updates
  sprintf(command, "?");
  if (lowLevelWriteRead(command, response, sizeof(response)) != asynSuccess) {
    asynPrint(lowLevelPortUser_, ASYN_TRACE_ERROR, "%s: Error reading global status. command: %s\n", functionName, command);
    setIntegerParam(this->motorStatusCommsError_, 1);
    setIntegerParam(PMAC_C_CommsError_, 1);
  } else {
    setIntegerParam(this->motorStatusCommsError_, 0);
    setIntegerParam(PMAC_C_CommsError_, 0);
  }
  
  nvals = sscanf(response, "%8x", &pmacStatus);

  // Read the status of PLC programs 0..31
  sprintf(command, "Plc[0].Running Plc[1].Running Plc[2].Running Plc[3].Running Plc[4].Running Plc[5].Running Plc[6].Running Plc[7].Running Plc[8].Running Plc[9].Running Plc[10].Running Plc[11].Running Plc[12].Running Plc[13].Running Plc[14].Running Plc[15].Running Plc[16].Running Plc[17].Running Plc[18].Running Plc[19].Running Plc[20].Running Plc[21].Running Plc[22].Running Plc[23].Running Plc[24].Running Plc[25].Running Plc[26].Running Plc[27].Running Plc[28].Running Plc[29].Running Plc[30].Running Plc[31].Running");
  if (lowLevelWriteRead(command, response, sizeof(response)) == asynSuccess) {
    sscanf(response, "Plc[0].Running=%d\nPlc[1].Running=%d\nPlc[2].Running=%d\nPlc[3].Running=%d\nPlc[4].Running=%d\nPlc[5].Running=%d\nPlc[6].Running=%d\nPlc[7].Running=%d\nPlc[8].Running=%d\nPlc[9].Running=%d\nPlc[10].Running=%d\nPlc[11].Running=%d\nPlc[12].Running=%d\nPlc[13].Running=%d\nPlc[14].Running=%d\nPlc[15].Running=%d\nPlc[16].Running=%d\nPlc[17].Running=%d\nPlc[18].Running=%d\nPlc[19].Running=%d\nPlc[20].Running=%d\nPlc[21].Running=%d\nPlc[22].Running=%d\nPlc[23].Running=%d\nPlc[24].Running=%d\nPlc[25].Running=%d\nPlc[26].Running=%d\nPlc[27].Running=%d\nPlc[28].Running=%d\nPlc[29].Running=%d\nPlc[30].Running=%d\nPlc[31].Running=%d\n", &plc[0], &plc[1], &plc[2], &plc[3], &plc[4], &plc[5], &plc[6], &plc[7], &plc[8], &plc[9], &plc[10], &plc[11], &plc[12], &plc[13], &plc[14], &plc[15], &plc[16], &plc[17], &plc[18], &plc[19], &plc[20], &plc[21], &plc[22], &plc[23], &plc[24], &plc[25], &plc[26], &plc[27], &plc[28], &plc[29], &plc[30], &plc[31]);
    // Record the first 16 PLC execution states
    plcStatus = plc[0];
    for (int index = 1; index <= 15; index++){
      plcStatus = plcStatus | (plc[index]<<index);
    }
    setIntegerParam (PMAC_C_PLCStatus0_,  plcStatus);
    // Record the second 16 PLC execution states
    plcStatus = plc[16];
    for (int index = 17; index <= 31; index++){
      plcStatus = plcStatus | (plc[index]<<(index-16));
    }
    setIntegerParam (PMAC_C_PLCStatus1_,  plcStatus);
  }

  return pmacStatus;

}

/**
 * Set the PMAC axis scale factor to increase resolution in the motor record.
 * Default value is 1.
 * @param axis Axis number to set the PMAC axis scale factor.
 * @param scale Scale factor to set
 */
asynStatus powerPmacController::pmacSetAxisScale(int axis, int scale) 
{
  powerPmacAxis *pA = NULL;
  static const char *functionName = "powerPmacController::pmacSetAxisScale";

  debugFlow(functionName);

  if (scale < 1) {
    asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, "%s: Error: scale factor must be >=1.\n", functionName);
    return asynError;
  }

  this->lock();
  pA = getAxis(axis);
  if (pA) {
    pA->scale_ = scale;
    asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, 
              "%s. Setting scale factor of %d on axis %d, on controller %s.\n", 
              functionName, pA->scale_, pA->axisNo_, portName);

  } else {
    asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, 
	      "%s: Error: axis %d has not been configured using pmacCreateAxis.\n", functionName, axis);
    return asynError;
  }
  this->unlock();
  return asynSuccess;
}


/**
 * If we have an open loop axis that has an encoder coming back on a different channel
 * then the encoder readback axis number can be set here. This ensures that the encoder
 * will be used for the position readback. It will also ensure that the encoder axis
 * is set correctly when performing a set position on the open loop axis.
 *
 * To use this function, the axis number used for the encoder must have been configured
 * already using pmacCreateAxis.
 *
 * @param controller The Asyn port name for the PMAC controller.
 * @param axis Axis number to set the PMAC axis scale factor.
 * @param encoder_axis The axis number that the encoder is fed into.  
 */
asynStatus powerPmacController::pmacSetOpenLoopEncoderAxis(int axis, int encoder_axis)
{
  powerPmacAxis *pA = NULL;
  static const char *functionName = "powerPmacController::pmacSetOpenLoopEncoderAxis";

  debugFlow(functionName);

  this->lock();
  pA = getAxis(axis);
  if (pA) {
    //Test that the encoder axis has also been configured
    if (getAxis(encoder_axis) == NULL) {
      asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, 
		"%s: Error: encoder axis %d has not been configured using pmacCreateAxis.\n", functionName, encoder_axis);
      return asynError;
    }
    pA->encoder_axis_ = encoder_axis;
    asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, 
              "%s. Setting encoder axis %d for axis %d, on controller %s.\n", 
              functionName, pA->encoder_axis_, pA->axisNo_, portName);

  } else {
    asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, 
	      "%s: Error: axis %d has not been configured using pmacCreateAxis.\n", functionName, axis);
    return asynError;
  }
  this->unlock();
  return asynSuccess;
}


/*************************************************************************************/
/** The following functions have C linkage, and can be called directly or from iocsh */

extern "C" {

/**
 * C wrapper for the powerPmacController constructor.
 * See powerPmacController::powerPmacController.
 *
 */
asynStatus powerPmacCreateController(const char *portName, const char *lowLevelPortName, int lowLevelPortAddress, 
				int numAxes, int movingPollPeriod, int idlePollPeriod)
{
    powerPmacController *ppmacController
      = new powerPmacController(portName, lowLevelPortName, lowLevelPortAddress, numAxes, movingPollPeriod/1000., idlePollPeriod/1000.);
    ppmacController = NULL;

    return asynSuccess;
}

/**
 * C wrapper for the pmacAxis constructor.
 * See pmacAxis::pmacAxis.
 *
 */
asynStatus pmacCreateAxis(const char *pmacName,         /* specify which controller by port name */
			  int axis)                    /* axis number (start from 1). */
{
  powerPmacController *pC;
  powerPmacAxis *pAxis;

  static const char *functionName = "pmacCreateAxis";
  pC = (powerPmacController*) findAsynPortDriver(pmacName);
  if (!pC) {
    printf("%s:%s: Error port %s not found\n",
           driverName, functionName, pmacName);
    return asynError;
  }
 
  pC->lock();
  pAxis = new powerPmacAxis(pC, axis);
  pAxis = NULL;
  pC->unlock();
  return asynSuccess;
}

/**
 * C Wrapper function for pmacAxis constructor.
 * See pmacAxis::pmacAxis.
 * This function allows creation of multiple pmacAxis objects with axis numbers 1 to numAxes.
 * @param pmacName Asyn port name for the controller (const char *)
 * @param numAxes The number of axes to create, starting at 1.
 *
 */
asynStatus pmacCreateAxes(const char *pmacName,        
			  int numAxes)                   
{
  powerPmacController *pC;
  powerPmacAxis *pAxis;

  static const char *functionName = "pmacCreateAxis";

  pC = (powerPmacController*) findAsynPortDriver(pmacName);
  if (!pC) {
    printf("%s:%s: Error port %s not found\n",
           driverName, functionName, pmacName);
    return asynError;
  }
  
  pC->lock();
  for (int axis=1; axis<=numAxes; axis++) {
    pAxis = new powerPmacAxis(pC, axis);
    pAxis = NULL;
  }
  pC->unlock();
  return asynSuccess;
}

/**
 * C wrapper for the pmacAxis constructor.
 * See pmacAxis::pmacAxis.
 *
 */
asynStatus pmacCreateCSAxis(const char *pmacName,   /* specify which controller by port name */
			                      int axis,               /* axis number (start from 1). */
			                      int csAxis,             /* coordinate system to use for this axis (start from 1). */
			                      int programNo,          /* motion program number to execute for moves. */
			                      const char *posVarName, /* name of variable to use to set position demand (eg P100). */
			                      const char *rdbVarName) /* name of variable to use to readback current position (eg P101). */
{
  powerPmacController *pC;
  powerPmacCSAxis *pAxis;

  static const char *functionName = "pmacCreateCSAxis";
  pC = (powerPmacController*) findAsynPortDriver(pmacName);
  if (!pC) {
    printf("%s:%s: Error port %s not found\n",
           driverName, functionName, pmacName);
    return asynError;
  }
 
  pC->lock();
  pAxis = new powerPmacCSAxis(pC, axis, csAxis, programNo, posVarName, rdbVarName);
  pAxis = NULL;
  pC->unlock();
  return asynSuccess;
}


/**
 * Set the PMAC axis scale factor to increase resolution in the motor record.
 * Default value is 1.
 * @param controller The Asyn port name for the PMAC controller.
 * @param axis Axis number to set the PMAC axis scale factor.
 * @param scale Scale factor to set
 */
asynStatus pmacSetAxisScale(const char *controller, int axis, int scale)
{
  powerPmacController *pC;
  static const char *functionName = "pmacSetAxisScale";

  pC = (powerPmacController*) findAsynPortDriver(controller);
  if (!pC) {
    cout << driverName << "::" << functionName << " Error: port " << controller << " not found." << endl;
    return asynError;
  }
    
  return pC->pmacSetAxisScale(axis, scale);
}

  
/**
 * If we have an open loop axis that has an encoder coming back on a different channel
 * then the encoder readback axis number can be set here. This ensures that the encoder
 * will be used for the position readback. It will also ensure that the encoder axis
 * is set correctly when performing a set position on the open loop axis.
 *
 * To use this function, the axis number used for the encoder must have been configured
 * already using pmacCreateAxis.
 *
 * @param controller The Asyn port name for the PMAC controller.
 * @param axis Axis number to set the PMAC axis scale factor.
 * @param encoder_axis The axis number that the encoder is fed into.  
 */
asynStatus pmacSetOpenLoopEncoderAxis(const char *controller, int axis, int encoder_axis)
{
  powerPmacController *pC;
  static const char *functionName = "pmacSetOpenLoopEncoderAxis";

  pC = (powerPmacController*) findAsynPortDriver(controller);
  if (!pC) {
    cout << driverName << "::" << functionName << " Error: port " << controller << " not found." << endl;
    return asynError;
  }
    
  return pC->pmacSetOpenLoopEncoderAxis(axis, encoder_axis);
}

/* Code for iocsh registration */

#ifdef vxWorks
#else

/* pmacCreateController */
static const iocshArg powerPmacCreateControllerArg0 = {"Controller port name", iocshArgString};
static const iocshArg powerPmacCreateControllerArg1 = {"Low level port name", iocshArgString};
static const iocshArg powerPmacCreateControllerArg2 = {"Low level port address", iocshArgInt};
static const iocshArg powerPmacCreateControllerArg3 = {"Number of axes", iocshArgInt};
static const iocshArg powerPmacCreateControllerArg4 = {"Moving poll rate (ms)", iocshArgInt};
static const iocshArg powerPmacCreateControllerArg5 = {"Idle poll rate (ms)", iocshArgInt};
static const iocshArg * const powerPmacCreateControllerArgs[] = {&powerPmacCreateControllerArg0,
							    &powerPmacCreateControllerArg1,
							    &powerPmacCreateControllerArg2,
							    &powerPmacCreateControllerArg3,
							    &powerPmacCreateControllerArg4,
							    &powerPmacCreateControllerArg5};
static const iocshFuncDef configpowerPmacCreateController = {"powerPmacCreateController", 6, powerPmacCreateControllerArgs};
static void configpowerPmacCreateControllerCallFunc(const iocshArgBuf *args)
{
  if (!args[0].sval){
    printf("powerPmacCreateController: failed - Controller port name must be specified\n");
    return;
  }
  if (!args[1].sval){
    printf("powerPmacCreateController: failed - Low level port name must be specified\n");
    return;
  }
  powerPmacCreateController(args[0].sval, args[1].sval, args[2].ival, args[3].ival, args[4].ival, args[5].ival);
}

/* pmacCreateAxis */
static const iocshArg pmacCreateAxisArg0 = {"Controller port name", iocshArgString};
static const iocshArg pmacCreateAxisArg1 = {"Axis number", iocshArgInt};
static const iocshArg * const pmacCreateAxisArgs[] = {&pmacCreateAxisArg0,
                                                     &pmacCreateAxisArg1};
static const iocshFuncDef configpmacAxis = {"pmacCreateAxis", 2, pmacCreateAxisArgs};

static void configpmacAxisCallFunc(const iocshArgBuf *args)
{
  if (args[0].sval){
    pmacCreateAxis(args[0].sval, args[1].ival);
  } else {
    printf("pmacCreateAxis: failed - Controller port name must be specified\n");
  }
}

/* pmacCreateAxes */
static const iocshArg pmacCreateAxesArg0 = {"Controller port name", iocshArgString};
static const iocshArg pmacCreateAxesArg1 = {"Num Axes", iocshArgInt};
static const iocshArg * const pmacCreateAxesArgs[] = {&pmacCreateAxesArg0,
                                                     &pmacCreateAxesArg1};
static const iocshFuncDef configpmacAxes = {"pmacCreateAxes", 2, pmacCreateAxesArgs};

static void configpmacAxesCallFunc(const iocshArgBuf *args)
{
  if (args[0].sval){
    pmacCreateAxes(args[0].sval, args[1].ival);
  } else {
    printf("pmacCreateAxes: failed - Controller port name must be specified\n");
  }
}

/* pmacCreateCSAxis */
static const iocshArg pmacCreateCSAxisArg0 = {"Controller port name", iocshArgString};
static const iocshArg pmacCreateCSAxisArg1 = {"Axis number", iocshArgInt};
static const iocshArg pmacCreateCSAxisArg2 = {"Coordinate System number", iocshArgInt};
static const iocshArg pmacCreateCSAxisArg3 = {"Motion program number", iocshArgInt};
static const iocshArg pmacCreateCSAxisArg4 = {"Demand variable name", iocshArgString};
static const iocshArg pmacCreateCSAxisArg5 = {"Readback variable name", iocshArgString};
static const iocshArg * const pmacCreateCSAxisArgs[] = {&pmacCreateCSAxisArg0,
                                                        &pmacCreateCSAxisArg1,
                                                        &pmacCreateCSAxisArg2,
                                                        &pmacCreateCSAxisArg3,
                                                        &pmacCreateCSAxisArg4,
                                                        &pmacCreateCSAxisArg5};
static const iocshFuncDef configpmacCSAxis = {"pmacCreateCSAxis", 6, pmacCreateCSAxisArgs};

static void configpmacCSAxisCallFunc(const iocshArgBuf *args)
{
  if (!args[0].sval){
    printf("pmacCreateCSAxis: failed - Controller port name must be specified\n");
    return;
  }
  if (!args[4].sval){
    printf("pmacCreateCSAxis: failed - Demand variable name must be specified\n");
    return;
  }
  if (!args[5].sval){
    printf("pmacCreateCSAxis: failed - Readback variable name must be specified\n");
    return;
  }
  pmacCreateCSAxis(args[0].sval, args[1].ival, args[2].ival, args[3].ival, args[4].sval, args[5].sval);
}


/* pmacSetAxisScale */
static const iocshArg pmacSetAxisScaleArg0 = {"Controller port name", iocshArgString};
static const iocshArg pmacSetAxisScaleArg1 = {"Axis number", iocshArgInt};
static const iocshArg pmacSetAxisScaleArg2 = {"Scale", iocshArgInt};
static const iocshArg * const pmacSetAxisScaleArgs[] = {&pmacSetAxisScaleArg0,
							      &pmacSetAxisScaleArg1,
							      &pmacSetAxisScaleArg2};
static const iocshFuncDef configpmacSetAxisScale = {"pmacSetAxisScale", 3, pmacSetAxisScaleArgs};

static void configpmacSetAxisScaleCallFunc(const iocshArgBuf *args)
{
  if (args[0].sval){
    pmacSetAxisScale(args[0].sval, args[1].ival, args[2].ival);
  } else {
    printf("pmacSetAxisScale: failed - Controller port name must be specified\n");
  }
}

/* pmacSetOpenLoopEncoderAxis */
static const iocshArg pmacSetOpenLoopEncoderAxisArg0 = {"Controller port name", iocshArgString};
static const iocshArg pmacSetOpenLoopEncoderAxisArg1 = {"Axis number", iocshArgInt};
static const iocshArg pmacSetOpenLoopEncoderAxisArg2 = {"Encoder Axis", iocshArgInt};
static const iocshArg * const pmacSetOpenLoopEncoderAxisArgs[] = {&pmacSetOpenLoopEncoderAxisArg0,
								  &pmacSetOpenLoopEncoderAxisArg1,
								  &pmacSetOpenLoopEncoderAxisArg2};
static const iocshFuncDef configpmacSetOpenLoopEncoderAxis = {"pmacSetOpenLoopEncoderAxis", 3, pmacSetOpenLoopEncoderAxisArgs};

static void configpmacSetOpenLoopEncoderAxisCallFunc(const iocshArgBuf *args)
{
  pmacSetOpenLoopEncoderAxis(args[0].sval, args[1].ival, args[2].ival);
}


static void powerPmacControllerRegister(void)
{
  iocshRegister(&configpowerPmacCreateController,   configpowerPmacCreateControllerCallFunc);
  iocshRegister(&configpmacAxis,                    configpmacAxisCallFunc);
  iocshRegister(&configpmacAxes,                    configpmacAxesCallFunc);
  iocshRegister(&configpmacCSAxis,                  configpmacCSAxisCallFunc);
  iocshRegister(&configpmacSetAxisScale,            configpmacSetAxisScaleCallFunc);
  iocshRegister(&configpmacSetOpenLoopEncoderAxis,  configpmacSetOpenLoopEncoderAxisCallFunc);
}
epicsExportRegistrar(powerPmacControllerRegister);

#endif

#ifdef vxWorks
  //VxWorks register functions
  epicsRegisterFunction(powerPmacCreateController);
  epicsRegisterFunction(pmacCreateAxis);
  epicsRegisterFunction(pmacCreateAxes);
  epicsRegisterFunction(pmacCreateCSAxis);
  epicsRegisterFunction(pmacSetAxisScale);
  epicsRegisterFunction(pmacSetOpenLoopEncoderAxis);
#endif
} // extern "C"

