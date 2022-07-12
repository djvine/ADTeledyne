TOP=../..
include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE

# Need to figure this out; library is only shareable, so we say LIB_SYS_LIBS and it works
ifeq (linux-x86, $(findstring linux-x86, $(T_A)))
SHARED_LIBRARIES=NO
endif

LIBRARY_IOC_Linux += teledyne
LIB_SRCS += shadobox.cpp

DBD += teledyneSupport.dbd

# Add locally compiled object code
PROD_LIBS += teledyne

USR_INCLUDES += -I$(TOP)/teledyneSupport
USR_CPPFLAGS += -lGevApi -std=c++17 -g
USR_LDFLAGS += -pthread -g -lpthread

include $(ADCORE)/ADApp/commonLibraryMakefile

#=============================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

