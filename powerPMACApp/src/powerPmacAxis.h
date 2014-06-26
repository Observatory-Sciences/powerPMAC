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

#ifndef powerPmacAxis_H
#define powerPmacAxis_H

#include "asynMotorController.h"
#include "asynMotorAxis.h"

class powerPmacController;

class powerPmacAxis : public asynMotorAxis
{
  public:
  /* These are the methods we override from the base class */
  powerPmacAxis(powerPmacController *pController, int axisNo);
  virtual ~powerPmacAxis();
  virtual asynStatus move(double position, int relative, double min_velocity, double max_velocity, double acceleration);
  virtual asynStatus moveVelocity(double min_velocity, double max_velocity, double acceleration);
  virtual asynStatus home(double min_velocity, double max_velocity, double acceleration, int forwards);
  virtual asynStatus stop(double acceleration);
  virtual asynStatus poll(bool *moving);
  virtual asynStatus setPosition(double position);
  virtual asynStatus setIntegerParam(int function, int value);

  protected:
  powerPmacController *pC_;
  
  virtual asynStatus getAxisStatus(bool *moving);
  virtual asynStatus getAxisInitialStatus(void);

  double setpointPosition_;
  double encoderPosition_;
  double currentVelocity_;
  double velocity_;
  double accel_;
  double highLimit_;
  double lowLimit_;
  int limitsDisabled_;
  double stepSize_;
  int scale_;
  double previous_position_;
  int previous_direction_;
  int amp_enabled_;
  int fatal_following_;
  int encoder_axis_;
  int limitsCheckDisable_;
  int errorPrintCount_;
  int errorPrintFlag_;

  friend class powerPmacController;
};


#endif /* powerPmacAxis_H */
