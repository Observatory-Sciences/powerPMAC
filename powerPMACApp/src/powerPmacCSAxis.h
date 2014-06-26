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

#ifndef powerPmacCSAxis_H
#define powerPmacCSAxis_H

#include "asynMotorController.h"
#include "powerPmacAxis.h"

class powerPmacController;

class powerPmacCSAxis : public powerPmacAxis
{
  public:
  /* These are the methods we override from the base class */
  powerPmacCSAxis(powerPmacController *pController, int axisNo, int csNo, int progNo, const char *demand, const char *readback);
  virtual ~powerPmacCSAxis();
  asynStatus move(double position, int relative, double min_velocity, double max_velocity, double acceleration);
  asynStatus moveVelocity(double min_velocity, double max_velocity, double acceleration);
  asynStatus home(double min_velocity, double max_velocity, double acceleration, int forwards);
  asynStatus stop(double acceleration);
  asynStatus poll(bool *moving);
  asynStatus setPosition(double position);
  //asynStatus setIntegerParam(int function, int value);

  private:
  
  asynStatus getAxisStatus(bool *moving);
  asynStatus getAxisInitialStatus(void);

  int coordSysNo_;
  int motionProgNo_;
  double feedTime_;
  char demandVar_[256];
  char readbackVar_[256];

  friend class powerPmacController;
};


#endif /* powerPmacAxis_H */
