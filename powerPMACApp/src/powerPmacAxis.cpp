/********************************************
 *  powerPmacAxis.cpp
 * 
 *  PMAC Asyn motor based on the 
 *  asynMotorAxis class and on Matthew
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

#include <epicsTime.h>
#include <epicsThread.h>
#include <epicsExport.h>
#include <epicsExit.h>
#include <epicsString.h>
#include <iocsh.h>

#include "powerPmacAxis.h"
#include "powerPmacController.h"

extern "C" void shutdownCallback(void *pPvt)
{
  powerPmacController *pC = static_cast<powerPmacController *>(pPvt);

  pC->lock();
  pC->shuttingDown_ = 1;
  pC->unlock();
}

// These are the powerPmacAxis class methods
powerPmacAxis::powerPmacAxis(powerPmacController *pC, int axisNo)
  :   asynMotorAxis(pC, axisNo),
      pC_(pC)
{
  static const char *functionName = "powerPmacAxis::powerPmacAxis";

  pC_->debugFlow(functionName); 

  //Initialize non-static data members
  setpointPosition_ = 0.0;
  encoderPosition_ = 0.0;
  currentVelocity_ = 0.0;
  velocity_ = 0.0;
  accel_ = 0.0;
  highLimit_ = 0.0;
  lowLimit_ = 0.0;
  limitsDisabled_ = 0;
  stepSize_ = 1; //Don't need?
  scale_ = 1;
  previous_position_ = 0.0;
  previous_direction_ = 0;
  amp_enabled_ = 0;
  fatal_following_ = 0;
  encoder_axis_ = 0;
  limitsCheckDisable_ = 0;
  errorPrintCount_ = 0;
  errorPrintFlag_ = 0;

  // Set an EPICS exit handler that will shut down polling before asyn kills the IP sockets
  epicsAtExit(shutdownCallback, pC_);

  //Do an initial poll to get some values from the PMAC
  if (getAxisInitialStatus() != asynSuccess) {
    asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR,
	      "%s: getAxisInitialStatus failed to return asynSuccess. Controller: %s, Axis: %d.\n", functionName, pC_->portName, axisNo_);
  }
  callParamCallbacks();

  // Wake up the poller task which will make it do a poll, 
  // updating values for this axis to use the new resolution (stepSize_)
  pC_->wakeupPoller();
 
}

asynStatus powerPmacAxis::getAxisInitialStatus(void)
{
  char command[pC_->PMAC_MAXBUF_];
  char response[pC_->PMAC_MAXBUF_];
  int cmdStatus = 0;
  double low_limit = 0.0;
  double high_limit = 0.0;
  double pgain = 0.0;
  double igain = 0.0;
  double dgain = 0.0;
  int dummy = 0;
  int nvals = 0;

  static const char *functionName = "powerPmacAxis::getAxisInitialStatus";

  //sprintf(command, "I%d13 I%d14 I%d30 I%d31 I%d33", axisNo_, axisNo_, axisNo_, axisNo_, axisNo_);
  sprintf(command, "Motor[%d].MaxPos Motor[%d].MinPos Motor[%d].Servo.Kp Motor[%d].Servo.Kvfb Motor[%d].Servo.Ki", axisNo_, axisNo_, axisNo_, axisNo_, axisNo_);
  cmdStatus = pC_->lowLevelWriteRead(command, response, sizeof(response));
  //nvals = sscanf(response, "I%d=%lf\nI%d=%lf\nI%d=%lf\nI%d=%lf\nI%d=%lf", &dummy, &high_limit, &dummy, &low_limit, &dummy, &pgain, &dummy, &dgain, &dummy, &igain);
  nvals = sscanf(response, "Motor[%d].MaxPos=%lf\nMotor[%d].MinPos=%lf\nMotor[%d].Servo.Kp=%lf\nMotor[%d].Servo.Kvfb=%lf\nMotor[%d].Servo.Ki=%lf", &dummy, &high_limit, &dummy, &low_limit, &dummy, &pgain, &dummy, &dgain, &dummy, &igain);

  if (cmdStatus || nvals != 10) {
    asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR, "%s: Error: initial status poll failed on axis %d.\n", functionName, axisNo_);
    return asynError;
  } else {
    setDoubleParam(pC_->motorLowLimit_,  low_limit*scale_);
    setDoubleParam(pC_->motorHighLimit_, high_limit*scale_);
    setDoubleParam(pC_->motorPgain_,     pgain);
    setDoubleParam(pC_->motorIgain_,     igain);
    setDoubleParam(pC_->motorDgain_,     dgain);
    setIntegerParam(pC_->motorStatusHasEncoder_, 1);
  }
  return asynSuccess;
}


powerPmacAxis::~powerPmacAxis() 
{
  //Destructor
}


asynStatus powerPmacAxis::move(double position, int relative, double min_velocity, double max_velocity, double acceleration)
{
  asynStatus status = asynError;
  static const char *functionName = "powerPmacAxis::move";

  pC_->debugFlow(functionName);  

  char acc_buff[32]="\0";
  char vel_buff[32]="\0";
  char command[128] = {0};
  char response[32] = {0};

  if (max_velocity != 0) {
    sprintf(vel_buff, "Motor[%d].JogSpeed=%f ", axisNo_, (max_velocity / (scale_ * 1000.0) ));
  }
  if (acceleration != 0) {
    if (max_velocity != 0) {
      sprintf(acc_buff, "Motor[%d].JogTa=%f ", axisNo_, (fabs(max_velocity/acceleration) * 1000.0));
    }
  }

  sprintf( command, "%s%s#%d %s%.2f", vel_buff, acc_buff, axisNo_, (relative?"J^":"J="), position/scale_ );

  status = pC_->lowLevelWriteRead(command, response, sizeof(response));
  
  return status;
}


 
asynStatus powerPmacAxis::home(double min_velocity, double max_velocity, double acceleration, int forwards)
{
  asynStatus status = asynError;
  char command[128] = {0};
  char response[128] = {0};
  static const char *functionName = "powerPmacAxis::home";

  pC_->debugFlow(functionName); 

  sprintf(command, "#%d HOME", axisNo_);
  
  status = pC_->lowLevelWriteRead(command, response, sizeof(response));

  return status;
}

asynStatus powerPmacAxis::moveVelocity(double min_velocity, double max_velocity, double acceleration)
{
  asynStatus status = asynError;
  char acc_buff[32] = "\0";
  char vel_buff[32] = "\0";
  char command[128] = {0};
  char response[32] = {0};
  static const char *functionName = "powerPmacAxis::moveVelocity";

  pC_->debugFlow(functionName);  

  if (max_velocity != 0) {
    sprintf(vel_buff, "Motor[%d].JogSpeed=%f ", axisNo_, (fabs(max_velocity) / (scale_ * 1000.0)));
  }
  if (acceleration != 0) {
    if (max_velocity != 0) {
      sprintf(acc_buff, "Motor[%d].JogTa=%f ", axisNo_, (fabs(max_velocity/acceleration) * 1000.0));
    }
  }
  sprintf(command, "%s%s#%d %s", vel_buff, acc_buff, axisNo_, (max_velocity < 0 ? "J-": "J+") );

  status = pC_->lowLevelWriteRead(command, response, sizeof(response));

  return status;
}


asynStatus powerPmacAxis::setPosition(double position)
{
  static const char *functionName = "powerPmacAxis::setPosition";
  
  pC_->debugFlow(functionName);  

  return asynSuccess;
}

asynStatus powerPmacAxis::stop(double acceleration)
{
  asynStatus status = asynError;
  static const char *functionName = "powerPmacAxis::stopAxis";

  pC_->debugFlow(functionName); 

  char command[128] = {0};
  char response[32] = {0};

  sprintf( command, "#%d J/",  axisNo_ );

  status = pC_->lowLevelWriteRead(command, response, sizeof(response));
  return status;
}

asynStatus powerPmacAxis::poll(bool *moving)
{
  int status = 0;
  int globalStatus = 0;
  char message[pC_->PMAC_MAXBUF_];
  static const char *functionName = "powerPmacAxis::poll";

  sprintf(message, "%s: Polling axis: %d", functionName, this->axisNo_);
  pC_->debugFlow(message); 

  if (!pC_->lowLevelPortUser_) {
    setIntegerParam(pC_->motorStatusCommsError_, 1);
    return asynError;
  }
  
  //Set axis problem bits based on the controller status (obtained in the controller poll).
  if (pC_->getIntegerParam(pC_->PMAC_C_GlobalStatus_, &globalStatus)) {
    asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR, "%s: Could not read controller %s global status.\n", functionName, pC_->portName);
  }
  setIntegerParam(pC_->motorStatusProblem_, globalStatus);
      
  //Now poll axis status
  if ((status = getAxisStatus(moving)) != asynSuccess) {
    asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR,
	      "%s: getAxisStatus failed to return asynSuccess. Controller: %s, Axis: %d.\n", functionName, pC_->portName, axisNo_);
  }
  
  callParamCallbacks();
  return status ? asynError : asynSuccess;
}



asynStatus powerPmacAxis::getAxisStatus(bool *moving)
{
  char command[pC_->PMAC_MAXBUF_];
  char response[pC_->PMAC_MAXBUF_];
  int cmdStatus = 0;;
  int motorOn = 0; 
  int dummy = 0; 
  int done = 0;
  double position = 0; 
  double enc_position = 0;
  int nvals = 0;
  epicsUInt32 status[2] = {0};
  int axisProblemFlag = 0;
  int limitsDisabledBit = 0;
  int errorPrintMin = 10000;
    
  /* Keep error messages from being printed each poll.*/
  if (errorPrintCount_ > errorPrintMin) {
    errorPrintCount_ = 0;
    errorPrintFlag_ = 0;
  }
  errorPrintCount_++;

  // Read all the status for this axis in one go
  if (encoder_axis_ != 0) {
    // Encoder position comes back on a different axis
    sprintf( command, "#%d ? P #%d P Motor[%d].ServoCtrl", axisNo_,  encoder_axis_, axisNo_);
  } else {
    // Encoder position comes back on this axis - note we initially read 
    // the following error into the position variable
    sprintf( command, "#%d ? F P Motor[%d].ServoCtrl", axisNo_, axisNo_);
  }
    
  cmdStatus = pC_->lowLevelWriteRead(command, response, sizeof(response));
  // Response parsed for PowerPMAC
  nvals = sscanf( response, "$%8x%8x\n%lf\n%lf\nMotor[%d].ServoCtrl=%d", &status[0], &status[1], &position, &enc_position, &dummy, &motorOn );
	
  if ( cmdStatus || nvals != 6) {
    asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR, "drvPmacAxisGetStatus: not all status values returned. Status: %d\nCommand :%s\nResponse:%s", cmdStatus, command, response );
  } else {
    // Store the status values
    setIntegerParam(pC_->PMAC_C_AxisStatus0_, status[0]);
    setIntegerParam(pC_->PMAC_C_AxisStatus1_, status[1]);

    int homeSignal = ((status[0] & pC_->PMAC_STATUS1_HOME_COMPLETE) != 0);
    int direction = 0;
      
    /* For closed loop axes, position is actually following error up to this point */ 
    if ( encoder_axis_ == 0 ) {
      position += enc_position;
    }
      
    position *= scale_;
    enc_position  *= scale_;
      
    setDoubleParam(pC_->motorPosition_, position);
    setDoubleParam(pC_->motorEncoderPosition_, enc_position);
      
    /* Use previous position and current position to calculate direction.*/
    if ((position - previous_position_) > 0) {
      direction = 1;
    } else if (position - previous_position_ == 0.0) {
      direction = previous_direction_;
    } else {
      direction = 0;
    }
    setIntegerParam(pC_->motorStatusDirection_, direction);
    /*Store position to calculate direction for next poll.*/
    previous_position_ = position;
    previous_direction_ = direction;

    done = (((status[0] & pC_->PMAC_STATUS1_IN_POSITION) != 0) || (motorOn == 0)); 
    /*If we are not done, but amp has been disabled, then set done (to stop when we get following errors).*/
    if ((done == 0) && ((status[0] & pC_->PMAC_STATUS1_AMP_ENABLED) == 0)) {
      done = 1;
    }

    if (!done) {
      *moving = true;
    } else {
      *moving = false;
    }

    setIntegerParam(pC_->motorStatusDone_, done);
    setIntegerParam(pC_->motorStatusHighLimit_, (((status[0] & pC_->PMAC_STATUS1_POS_LIMIT_SET) | (status[0] & pC_->PMAC_STATUS1_SOFT_PLUS_LIMIT)) != 0) );
    setIntegerParam(pC_->motorStatusHomed_, homeSignal);
    // If desired_vel_zero is false && motor activated (ix00=1) && amplifier enabled, set moving=1.
    setIntegerParam(pC_->motorStatusMoving_, ((status[0] & pC_->PMAC_STATUS1_DESIRED_VELOCITY_ZERO) == 0) && (motorOn != 0) && ((status[0] & pC_->PMAC_STATUS1_AMP_ENABLED) != 0) );
    setIntegerParam(pC_->motorStatusLowLimit_, (((status[0] & pC_->PMAC_STATUS1_NEG_LIMIT_SET) | (status[0] & pC_->PMAC_STATUS1_SOFT_MINUS_LIMIT))!=0) );
    setIntegerParam(pC_->motorStatusFollowingError_,((status[1] & pC_->PMAC_STATUS1_ERR_FOLLOW_ERR) != 0) );
    fatal_following_ = ((status[1] & pC_->PMAC_STATUS1_ERR_FOLLOW_ERR) != 0);

    axisProblemFlag = 0;
    /*Set any axis specific general problem bits.*/
    if ( ((status[0] & pC_->PMAX_AXIS_GENERAL_PROB1) != 0) || ((status[1] & pC_->PMAX_AXIS_GENERAL_PROB2) != 0) ) {
      axisProblemFlag = 1;
    }
    // Check limits disabled bit in ix24, and if we haven't intentially disabled limits
    // because we are homing, set the motorAxisProblem bit. Also check the limitsCheckDisable
    // flag, which the user can set to disable this feature.
    if (!limitsCheckDisable_) {
      // Check we haven't intentially disabled limits for homing.
      if (!limitsDisabled_) {
        sprintf( command, "Motor[%d].pLimits", axisNo_);
        cmdStatus = pC_->lowLevelWriteRead(command, response, sizeof(response));
        if (cmdStatus == asynSuccess) {
          sscanf(response, "Motor[%d].pLimits=%s", &dummy, command);
          if (strcmp(command, "0") == 0){
            limitsDisabledBit = 1;
          } else {
            limitsDisabledBit = 0;
          }
          if (limitsDisabledBit) {
            axisProblemFlag = 1;
            if (errorPrintFlag_ == 0) {
              asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR, "*** WARNING *** Limits are disabled on controller %s, axis %d\n", pC_->portName, axisNo_);
              errorPrintFlag_ = 1;
            }
	        }
        }
      }
    }
    setIntegerParam(pC_->motorStatusProblem_, axisProblemFlag);

    // Clear error print flag for this axis if problem has been removed.
    if (axisProblemFlag == 0) {
      errorPrintFlag_ = 0;
    }
  }

  // Set amplifier enabled bit.
  if ((status[0] & pC_->PMAC_STATUS1_AMP_ENABLED) != 0) {
    amp_enabled_ = 1;
  } else {
    amp_enabled_ = 0;
  }
  return asynSuccess;
}

asynStatus powerPmacAxis::setIntegerParam(int function, int value)
{
  char command[pC_->PMAC_MAXBUF_];
  char response[pC_->PMAC_MAXBUF_];
  asynStatus cmdStatus = asynError;
  asynStatus status = asynError; 

  if (function == pC_->PMAC_C_KillAxis_){
    // Kill the axis
    sprintf( command, "#%dk", axisNo_);
    cmdStatus = pC_->lowLevelWriteRead(command, response, sizeof(response));
    if (cmdStatus != asynSuccess){
      return asynError;
    }
  }
  //Call base class method
  status = asynMotorAxis::setIntegerParam(function, value);
  return status;
}
