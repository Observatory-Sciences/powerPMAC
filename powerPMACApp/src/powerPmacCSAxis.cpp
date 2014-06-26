/********************************************
 *  powerPmacCSAxis.cpp
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

#include "powerPmacCSAxis.h"
#include "powerPmacController.h"

extern "C" void shutdownCSCallback(void *pPvt)
{
  powerPmacController *pC = static_cast<powerPmacController *>(pPvt);

  pC->lock();
  pC->shuttingDown_ = 1;
  pC->unlock();
}

// These are the powerPmacCSAxis class methods
powerPmacCSAxis::powerPmacCSAxis(powerPmacController *pC, int axisNo, int csNo, int progNo, const char *demand, const char *readback)
  :   powerPmacAxis(pC, axisNo),
      coordSysNo_(csNo),
      motionProgNo_(progNo)
{
  static const char *functionName = "powerPmacCSAxis::powerPmacCSAxis";

  pC_->debugFlow(functionName); 

  // Record the demand and readback variable names
  strcpy(demandVar_, demand);
  strcpy(readbackVar_, readback);
  //Initialize non-static data members
  feedTime_ = 1000.0;
/*
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
  epicsAtExit(shutdownCSCallback, pC_);

  //Do an initial poll to get some values from the PMAC
  if (getAxisInitialStatus() != asynSuccess) {
    asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR,
	      "%s: getAxisInitialStatus failed to return asynSuccess. Controller: %s, Axis: %d.\n", functionName, pC_->portName, axisNo_);
  }
  callParamCallbacks();

  // Wake up the poller task which will make it do a poll, 
  // updating values for this axis to use the new resolution (stepSize_)
  pC_->wakeupPoller();
 */
}

asynStatus powerPmacCSAxis::getAxisInitialStatus(void)
{
  char command[pC_->PMAC_MAXBUF_];
  char response[pC_->PMAC_MAXBUF_];
  int cmdStatus = 0;
  int dummy = 0;
  int nvals = 0;

  static const char *functionName = "powerPmacCSAxis::getAxisInitialStatus";

  sprintf(command, "Coord[%d].FeedTime", coordSysNo_); 
  cmdStatus = pC_->lowLevelWriteRead(command, response);
  nvals = sscanf(response, "Coord[%d].FeedTime=%lf", &dummy, &feedTime_); 

  if (cmdStatus || nvals != 2) {
    asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR, "%s: Error: initial status poll failed on axis %d.\n", functionName, axisNo_);
    return asynError;
  }
  return asynSuccess;
}


powerPmacCSAxis::~powerPmacCSAxis() 
{
  //Destructor
}


asynStatus powerPmacCSAxis::move(double position, int relative, double min_velocity, double max_velocity, double acceleration)
{
  asynStatus status = asynError;
  static const char *functionName = "powerPmacCSAxis::move";
  double dmdpos = 0.0;

  pC_->debugFlow(functionName);  

  char acc_buff[32]="\0";
  char vel_buff[32]="\0";
  char command[128] = {0};
  char response[32] = {0};

  if (max_velocity != 0) {
    sprintf(vel_buff, "Coord[%d].Tm=-%f ", coordSysNo_, (max_velocity / scale_ * feedTime_ ));
  }
  if (acceleration != 0) {
    if (max_velocity != 0) {
      sprintf(acc_buff, "Coord[%d].Ta=%f ", coordSysNo_, (fabs(max_velocity/acceleration) * 1000.0));
    }
  }

  // Set the position variable to the position, then execute the motion program
  sprintf( command, "%s%s", vel_buff, acc_buff );
  status = pC_->lowLevelWriteRead(command, response);

  if (relative){
    dmdpos = previous_position_;
  }
  dmdpos += position;
  sprintf( command, "%s=%.2f", demandVar_, dmdpos/scale_ );
  status = pC_->lowLevelWriteRead(command, response);
  sprintf( command, "&%db%dr", coordSysNo_, motionProgNo_ );
  status = pC_->lowLevelWriteRead(command, response);
    
  return status;
}


 
asynStatus powerPmacCSAxis::home(double min_velocity, double max_velocity, double acceleration, int forwards)
{
  asynStatus status = asynError;
  static const char *functionName = "powerPmacCSAxis::home";

  pC_->debugFlow(functionName); 

  return status;
}

asynStatus powerPmacCSAxis::moveVelocity(double min_velocity, double max_velocity, double acceleration)
{
  asynStatus status = asynError;
  static const char *functionName = "powerPmacCSAxis::moveVelocity";

  pC_->debugFlow(functionName);  

  return status;
}


asynStatus powerPmacCSAxis::setPosition(double position)
{
  static const char *functionName = "powerPmacCSAxis::setPosition";
  
  pC_->debugFlow(functionName);  

  return asynSuccess;
}

asynStatus powerPmacCSAxis::stop(double acceleration)
{
  asynStatus status = asynError;
  static const char *functionName = "powerPmacCSAxis::stopAxis";

  pC_->debugFlow(functionName); 

  char command[128] = {0};
  char response[32] = {0};

  sprintf( command, "&%da",  coordSysNo_ );

  status = pC_->lowLevelWriteRead(command, response);
  return status;
}

asynStatus powerPmacCSAxis::poll(bool *moving)
{
  int status = 0;
  int globalStatus = 0;
  char message[pC_->PMAC_MAXBUF_];
  static const char *functionName = "powerPmacCSAxis::poll";

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



asynStatus powerPmacCSAxis::getAxisStatus(bool *moving)
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
  //int limitsDisabledBit = 0;
  int errorPrintMin = 10000;
    
  /* Keep error messages from being printed each poll.*/
  if (errorPrintCount_ > errorPrintMin) {
    errorPrintCount_ = 0;
    errorPrintFlag_ = 0;
  }
  errorPrintCount_++;
printf("Scale: %d\n", scale_);
  sprintf( command, "Coord[%d].Status[0] Coord[%d].Status[1] %s", coordSysNo_,  coordSysNo_, readbackVar_);    

  cmdStatus = pC_->lowLevelWriteRead(command, response);
  // Response parsed for PowerPMAC
  nvals = sscanf( response, "Coord[%d].Status[0]=$%x\nCoord[%d].Status[1]=$%x\n%[^=]=%lf", &dummy, &status[0], &dummy, &status[1], command, &position );
	
  if ( cmdStatus || nvals != 6) {
    asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR, "drvPmacCSAxisGetStatus: not all status values returned. Status: %d\nCommand :%s\nResponse:%s\n", cmdStatus, command, response );
  } else {
    // Store the status values
    setIntegerParam(pC_->PMAC_C_AxisStatus0_, status[0]);
    setIntegerParam(pC_->PMAC_C_AxisStatus1_, status[1]);

    int direction = 0;
      
    enc_position = position;  
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

    done = ((status[0] & pC_->PMAC_STATUS1_IN_POSITION) != 0); 
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
    setIntegerParam(pC_->motorStatusMoving_, ((status[0] & pC_->PMAC_STATUS1_DESIRED_VELOCITY_ZERO) == 0) && (motorOn != 0) && ((status[0] & pC_->PMAC_STATUS1_AMP_ENABLED) != 0) );
    setIntegerParam(pC_->motorStatusLowLimit_, (((status[0] & pC_->PMAC_STATUS1_NEG_LIMIT_SET) | (status[0] & pC_->PMAC_STATUS1_SOFT_MINUS_LIMIT))!=0) );
    setIntegerParam(pC_->motorStatusFollowingError_,((status[1] & pC_->PMAC_STATUS1_ERR_FOLLOW_ERR) != 0) );
    fatal_following_ = ((status[1] & pC_->PMAC_STATUS1_ERR_FOLLOW_ERR) != 0);

    axisProblemFlag = 0;
    /*Set any axis specific general problem bits.*/
    if ( ((status[0] & pC_->PMAX_AXIS_GENERAL_PROB1) != 0) || ((status[1] & pC_->PMAX_AXIS_GENERAL_PROB2) != 0) ) {
      axisProblemFlag = 1;
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
