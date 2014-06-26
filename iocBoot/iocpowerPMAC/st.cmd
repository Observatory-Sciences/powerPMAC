#!../../bin/linux-x86/powerPMACTest

## You may have to change powerPMACTest to something else
## everywhere it appears in this file

< envPaths
epicsEnvSet ("STREAM_PROTOCOL_PATH", ".:../protocols")

cd ${TOP}

## Register all support components
dbLoadDatabase "dbd/powerPMACTest.dbd"
powerPMACTest_registerRecordDeviceDriver pdbbase

## Load record instances
dbLoadRecords "db/powerPMACTest.db", ""
dbLoadRecords "db/testApp.db", ""

drvAsynPowerPMACPortConfigure("SSH1", "ppmac", "root", "deltatau", "0", "0", "0")
powerPmacCreateController("PPMAC1", "SSH1", "0", "32", "200", "1000")
pmacCreateAxis("PPMAC1", "1")
pmacCreateAxis("PPMAC1", "2")
pmacCreateAxis("PPMAC1", "3")
pmacCreateAxis("PPMAC1", "4")
pmacCreateAxis("PPMAC1", "5")
pmacCreateAxis("PPMAC1", "6")
pmacCreateAxis("PPMAC1", "7")
pmacCreateAxis("PPMAC1", "8")
pmacCreateCSAxis("PPMAC1", "9", "1", "1", "P1", "P2")

#asynSetTraceMask("SSH1", -1, 127)
#asynSetTraceIOMask("SSH1", -1, 127)

cd ${TOP}/iocBoot/${IOC}
iocInit

