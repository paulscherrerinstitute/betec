TOP=../..

include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================

#=============================
# Build the IOC application

PROD_IOC = bestec
# bestec.dbd will be created and installed
DBD += bestec.dbd

# bestec.dbd will be made up from these files:
bestec_DBD += base.dbd

# Include dbd files from all support applications:
bestec_DBD += asyn.dbd
bestec_DBD += busySupport.dbd
bestec_DBD += drvAsynIPPort.dbd
bestec_DBD += motorSupport.dbd
bestec_DBD += bestecSupport.dbd

# Add all the support libraries needed by this IOC
bestec_LIBS += asyn
bestec_LIBS += busy
bestec_LIBS += motor
bestec_LIBS += bestec

# bestec_registerRecordDeviceDriver.cpp derives from bestec.dbd
bestec_SRCS += bestec_registerRecordDeviceDriver.cpp

# Build the main IOC entry point on workstation OSs.
bestec_SRCS_DEFAULT += bestecMain.cpp
bestec_SRCS_vxWorks += -nil-

# Add support from base/src/vxWorks if needed
#bestec_OBJS_vxWorks += $(EPICS_BASE_BIN)/vxComLibrary

# Finally link to the EPICS Base libraries
bestec_LIBS += $(EPICS_BASE_IOC_LIBS)

#===========================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

