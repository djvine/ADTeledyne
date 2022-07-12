#include <iostream>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <vector>
#include <array>
#include <fstream>
#include <algorithm>
#include <math.h>
#include <thread>
#include <signal.h>

#include "cordef.h"
#include "gevapi.h"

#define NUM_TELEDYNE_PARAMS 0
#define MAX_NETIF 2
#define MAX_CAMERAS_PER_NETIF 32
#define MAX_CAMERAS (MAX_NETIF * MAX_CAMERAS_PER_NETIF)
#define NUM_BUFFERS 1

GEV_DEVICE_INTERFACE telCamInt_[MAX_CAMERAS] = {0};
GEV_CAMERA_HANDLE telCamHandle_ = NULL;
GEV_CAMERA_OPTIONS telCamOpt_ = {0};
GEVLIB_CONFIG_OPTIONS telLibOpt_ = {0};
GEV_STATUS telStatus;

int numCameras;
int camId_=0;
int value;
int numBuffers = NUM_BUFFERS;
unsigned char* bufAddress[NUM_BUFFERS];
bool exiting_ = false;
int type;
unsigned int height, width, format;
unsigned int maxHeight, maxWidth, maxDepth;
unsigned long int size, payload_size;
unsigned int pixFormat = 0;
unsigned int pixDepth = 0;
char synchModeStr[64] = {0};
uint16_t synchMode = 0;
uint16_t acqStart = 1;
uint16_t acqStop  = 0;
double exposureTime = 1.0;
char* serial_number = "2123820683";

int main(){
	// Get config options
	GevGetLibraryConfigOptions(&telLibOpt_);
	telLibOpt_.logLevel = GEV_LOG_LEVEL_NORMAL;
	GevSetLibraryConfigOptions(&telLibOpt_);

	telStatus = GevGetCameraList(telCamInt_, MAX_CAMERAS, &numCameras);

	printf("%d cameras on the network\n", numCameras);
	if (numCameras==0){
		printf("%s:%s: No cameras detected on network\n");
		exiting_ = true;
		return -1;
	}
	telStatus = GevOpenCameraBySN(serial_number, GevControlMode, &telCamHandle_);
	if (telStatus==GEVLIB_OK){
		char xmlFileName[1024] = {0};
		telStatus = GevGetGenICamXML_FileName( telCamHandle_, (int)sizeof(xmlFileName), xmlFileName);
		if (telStatus==GEVLIB_OK){
			printf("XML stored as: %s\n", xmlFileName);
		}
		char stringFeature[32] = {0};
		GevGetFeatureValueAsString(telCamHandle_, "DeviceVendorName", &type,  sizeof(stringFeature), stringFeature);
		unsigned int expTime = 1000000;
		unsigned int rbv;
		int type, telStatus;
		printf("setting SoftwareTrigIntTime %d\n", expTime);
		telStatus = GevSetFeatureValue(telCamHandle_, "SoftwareTrigIntTime", sizeof(expTime), &expTime);
		if (telStatus!=GEVLIB_OK){
				printf("Coud not set SoftwareTrigIntTime (status=%d)\n", telStatus);
		}
	    GevGetFeatureValue(telCamHandle_, "SoftwareTrigIntTime", &type, sizeof(rbv), &rbv);
		if (rbv!=value){
				printf( "setCameraFeature error, status=%d feature=SoftwareTrigIntTime value=%d rbv=%d\n",
					  telStatus, expTime, rbv);
		}
	}


	telStatus = GevAbortTransfer(telCamHandle_);
	telStatus = GevFreeTransfer(telCamHandle_);
	GevCloseCamera(&telCamHandle_);
	GevApiUninitialize();
	_CloseSocketAPI();
	printf("Done\n");
}

