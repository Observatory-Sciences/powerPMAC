TOP=../..
include $(TOP)/configure/CONFIG

install:
	mkdir -p $(TOP)/data
	cp -f $(TOP)/powerPMACApp/opi/powerPMAC.opi $(TOP)/data/powerPMAC.opi
	cp -f $(TOP)/powerPMACApp/opi/motor.opi $(TOP)/data/motor.opi

clean::
	rm -rf $(TOP)/data

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

