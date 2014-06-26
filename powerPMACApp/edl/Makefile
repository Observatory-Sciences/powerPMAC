TOP=../..
include $(TOP)/configure/CONFIG

install:
	mkdir -p $(TOP)/data
	cp -f $(TOP)/powerPMACApp/edl/powerPMAC.edl $(TOP)/data/powerPMAC.edl
	cp -f $(TOP)/powerPMACApp/edl/motor.edl $(TOP)/data/motor.edl
	cp -f $(TOP)/powerPMACApp/edl/motor-commands.edl $(TOP)/data/motor-commands.edl
	cp -f $(TOP)/powerPMACApp/edl/motor-drive.edl $(TOP)/data/motor-drive.edl
	cp -f $(TOP)/powerPMACApp/edl/motor-limits.edl $(TOP)/data/motor-limits.edl
	cp -f $(TOP)/powerPMACApp/edl/motor-links.edl $(TOP)/data/motor-links.edl
	cp -f $(TOP)/powerPMACApp/edl/motor-misc.edl $(TOP)/data/motor-misc.edl
	cp -f $(TOP)/powerPMACApp/edl/motor-motion.edl $(TOP)/data/motor-motion.edl
	cp -f $(TOP)/powerPMACApp/edl/motor-resolution.edl $(TOP)/data/motor-resolution.edl
	cp -f $(TOP)/powerPMACApp/edl/motor-all.edl $(TOP)/data/motor-all.edl
	cp -f $(TOP)/powerPMACApp/edl/motor-calibration.edl $(TOP)/data/motor-calibration.edl
	cp -f $(TOP)/powerPMACApp/edl/motor-status.edl $(TOP)/data/motor-status.edl
	cp -f $(TOP)/powerPMACApp/edl/motor-servo.edl $(TOP)/data/motor-servo.edl
	cp -f $(TOP)/powerPMACApp/edl/motor-alarms.edl $(TOP)/data/motor-alarms.edl
	cp -f $(TOP)/powerPMACApp/edl/motor-cmd-confirm.edl $(TOP)/data/motor-cmd-confirm.edl

clean::
	rm -rf $(TOP)/data

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

