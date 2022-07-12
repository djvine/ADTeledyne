TOP=..
include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================

# Note, the files in this directory were manually copied from the Andor SDK3 directories 
# for Linux and Windows.  The soft links for Linux were manually created.

INC += cordef.h
INC += corenv.h
INC += corposix.h
INC += cpackd.h
INC += cunpackd.h
INC += dynaqueue.h
INC += gevapi.h
INC += gev_legacy_api.h
INC += gev_linux.h
INC += gevoslib.h
INC += PFNC.h
INC += posixcmn.h

ifeq (win32-x86, $(findstring win32-x86, $(T_A)))
LIB_INSTALLS_WIN32    += ../os/win32-x86/atcorem.lib

else ifeq (windows-x64, $(findstring windows-x64, $(T_A)))
LIB_INSTALLS_WIN32    += ../os/windows-x64/atcorem.lib

#else ifeq (linux-x86_64, $(findstring linux-x86_64, $(T_A)))
#LIB_INSTALLS_Linux += ../os/linux-x86_64/xclib_x86_64.so
#LIB_INSTALLS_Linux += ../os/linux-x86_64/libTUCam.so.1
#LIB_INSTALLS_Linux += ../os/linux-x86_64/libTUCam.so.1.0
#LIB_INSTALLS_Linux += ../os/linux-x86_64/libTUCam.so.1.0.0

#else ifeq (linux-x86, $(findstring linux-x86, $(T_A)))
#LIB_INSTALLS_Linux += ../os/linux-x86_64/libTUCam.so
#LIB_INSTALLS_Linux += ../os/linux-x86_64/libTUCam.so.1
#LIB_INSTALLS_Linux += ../os/linux-x86_64/libTUCam.so.1.0
#LIB_INSTALLS_Linux += ../os/linux-x86_64/libTUCam.so.1.0.0

endif

#=============================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

