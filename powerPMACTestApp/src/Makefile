TOP=../..

include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================

#=============================
# build an ioc application
PROD_IOC = powerPMACTest

DBD += powerPMACTest.dbd
# powetPMACTest.dbd will be made up from these files:
powerPMACTest_DBD += base.dbd
powerPMACTest_DBD += asyn.dbd
powerPMACTest_DBD += motorRecord.dbd
powerPMACTest_DBD += motorSupport.dbd
powerPMACTest_DBD += powerPMACMotorSSH.dbd
powerPMACTest_DBD += drvAsynPowerPMACPort.dbd
powerPMACTest_DBD += stream.dbd

# <name>_registerRecordDeviceDriver.cpp will be created from <name>.dbd
powerPMACTest_SRCS += powerPMACTest_registerRecordDeviceDriver.cpp
powerPMACTest_SRCS_DEFAULT += powerPMACTestMain.cpp
powerPMACTest_SRCS_vxWorks += -nil-

#The following adds support from base/src/vxWorks
powerPMACTest_OBJS_vxWorks += $(EPICS_BASE_BIN)/vxComLibrary

# Include the following libraries in the build
powerPMACTest_LIBS += asyn
powerPMACTest_LIBS += motor
powerPMACTest_LIBS += powerPMACSupport
powerPMACTest_LIBS += stream
powerPMACTest_LIBS += $(EPICS_BASE_IOC_LIBS)

#===========================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE


