TOP=../..

include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE

#=============================
# Build the IOC support library

LIBRARY_IOC += powerPMACSupport

DBD += powerPMACMotorSSH.dbd
DBD += drvAsynPowerPMACPort.dbd

# Link locally-provided code into the support library,
powerPMACSupport_SRCS += sshDriver.cpp
powerPMACSupport_SRCS += powerPmacController.cpp
powerPMACSupport_SRCS += powerPmacAxis.cpp
powerPMACSupport_SRCS += powerPmacCSAxis.cpp
powerPMACSupport_SRCS += drvAsynPowerPMACPort.cpp

powerPMACSupport_LIBS += asyn
powerPMACSupport_LIBS += motor
powerPMACSupport_LIBS += ssh2
powerPMACSupport_LIBS += $(EPICS_BASE_IOC_LIBS)
ssh2_DIR = /usr/local/lib

#=============================
# Build the interactive shell test application

PROD_IOC = powerPMACShell

powerPMACShell_SRCS += interactive.cpp

# Link in the code from the support library
powerPMACShell_LIBS += powerPMACSupport
powerPMACShell_LIBS += asyn
powerPMACShell_LIBS += motor
powerPMACShell_LIBS += ssh2
ssh2_DIR = /usr/local/lib

# Finally link to the EPICS Base libraries
powerPMACShell_LIBS += $(EPICS_BASE_IOC_LIBS)


PROD_IOC += gpascii
gpascii_SRCS += test_gpascii.c

#=============================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

