TOP=../..

include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================

#==================================================
# build a support library

LIBRARY_IOC += bestec

# install bestecSupport.dbd into <top>/dbd
DBD += bestecSupport.dbd

# specify all source files to be compiled and added to the library
bestec_SRCS += bestec.cpp

bestec_LIBS += asyn motor
bestec_LIBS += $(EPICS_BASE_IOC_LIBS)

#===========================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

