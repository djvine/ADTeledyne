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

uint16_t sendTrigger(GEV_CAMERA_HANDLE camHandle);
void signal_callback_handler(int signum);


GEV_DEVICE_INTERFACE telCamInt_[MAX_CAMERAS] = {0};
GEV_CAMERA_HANDLE telCamHandle_ = NULL;
GEV_CAMERA_OPTIONS telCamOpt_ = {0};
GEVLIB_CONFIG_OPTIONS telLibOpt_ = {0};
GEV_STATUS telStatus;

int numCameras;
int camId_;
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
int expTime;

int main(){
	// Register signal handler
	signal(SIGINT, signal_callback_handler);

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
	telStatus = GevOpenCamera(&telCamInt_[camId_], GevControlMode, &telCamHandle_);
	if (telStatus==GEVLIB_OK){
		char xmlFileName[1024] = {0};
		telStatus = GevGetGenICamXML_FileName( telCamHandle_, (int)sizeof(xmlFileName), xmlFileName);
		if (telStatus==GEVLIB_OK){
			printf("XML stored as: %s\n", xmlFileName);
		}

		// Print device temperature
		double temperature = 0.0;
		GevGetFeatureValue(telCamHandle_, "DeviceTemperature", &type, sizeof(temperature), &temperature);
		printf("Device temp: %3.3fC\n", temperature);

		GevGetFeatureValue(telCamHandle_, "Width", &type, sizeof(width), &width);
		GevGetFeatureValue(telCamHandle_, "Height", &type, sizeof(height), &height);
		printf("Width: %d, Height: %d\n", width, height);
		GevGetFeatureValue(telCamHandle_, "PixelFormat", &type, sizeof(format), &format);
		switch (format){
			case 0x1100025:
				std::cout << "Pixel format: 14 bit mono unsigned" << std::endl;
				break;
			default:
				std::cout << "Unknown pixel format!" << std::endl;
		}
		char stringFeature[32];
		GevGetFeatureValue(telCamHandle_, "DeviceVendorName", &type, sizeof(stringFeature), &stringFeature);
		printf("Vendor: %s\n", stringFeature);
		GevGetFeatureValue(telCamHandle_, "DeviceModelName", &type, sizeof(stringFeature), &stringFeature);
		printf("Model: %s\n", stringFeature);
		GevGetFeatureValue(telCamHandle_, "DeviceVersion", &type, sizeof(stringFeature), &stringFeature);
		printf("Version: %s\n", stringFeature);
		GevGetFeatureValue(telCamHandle_, "DeviceProductIDBuild", &type, sizeof(stringFeature), &stringFeature);
		printf("Product ID: %s\n", stringFeature);
		GevGetFeatureValue(telCamHandle_, "DeviceID", &type, sizeof(stringFeature), &stringFeature);
		printf("Device ID: %s\n", stringFeature);
		GevGetFeatureValueAsString(telCamHandle_, "SynchronizationMode", &type, sizeof(synchModeStr), synchModeStr);
		GevGetFeatureValue(telCamHandle_, "SynchronizationMode", &type, sizeof(synchMode), &synchMode);
		printf("SynchMode string: %s\n", synchModeStr);
		printf("Synchmode: %d\n", synchMode);
		//int synchMode = 0x859;
		//GevSetFeatureValue(telCamHandle_, "SynchronizationMode", sizeof(synchMode), &synchMode);
		//GevGetFeatureValueAsString(telCamHandle_, "SynchronizationMode", &type, sizeof(synchModeStr), synchModeStr);
		GevGetFeatureValueAsString(telCamHandle_, "SynchronizationMode", "Snapshot");
		//GevGetFeatureValue(telCamHandle_, "SynchronizationMode", &type, sizeof(synchMode), &synchMode);
		//printf("SynchMode string: %s\n", synchModeStr);
		//printf("Synchmode: %d\n", synchMode);

		GevGetCameraInterfaceOptions(telCamHandle_, &telCamOpt_);
		telCamOpt_.heartbeat_timeout_ms = 5000;     // Disconnect detection (5 seconds)
		telCamOpt_.streamFrame_timeout_ms = 1001; // Internal timeout for frame reception
		telCamOpt_.streamNumFramesBuffered = 1;   // Buffer frames internally
		telCamOpt_.streamMemoryLimitMax = 64*2304*2940; // Adjust packet memory buffering limit
		telCamOpt_.streamPktSize = 9216;
		telCamOpt_.streamPktDelay = 10;

		int numCpus = _GetNumCpus();
		if (numCpus>1){
			telCamOpt_.streamThreadAffinity = numCpus-1;
			telCamOpt_.serverThreadAffinity = numCpus-2;
		}
		GevSetCameraInterfaceOptions(telCamHandle_, &telCamOpt_);

		maxHeight = height;
		maxWidth = width;
		maxDepth = GetPixelSizeInBytes(format);

		size = maxWidth*maxHeight*maxDepth;
		printf("allocate...");
		for (int i=0; i<numBuffers; i++){
			bufAddress[i] = (PUINT8)malloc(size);
			memset(bufAddress[i], 0, size);
		}
		printf("done\n");
		//GevAbortTransfer(telCamHandle_);
		//GevFreeTransfer(telCamHandle_);
		expTime = 500000;
		telStatus=GevSetFeatureValue(telCamHandle_, "FrameInterval", sizeof(expTime), &expTime);
		if (telStatus!=GEVLIB_OK){
				printf("Not set exposure time\n");
		}
		int expTimeRBV = 0;
		telStatus = GevGetFeatureValue(telCamHandle_, "FrameInterval", &type, sizeof(expTimeRBV), &expTimeRBV);
		printf("Exp time: %d, rbv: %d\n", expTime, expTimeRBV);
		telStatus = GevInitializeTransfer(telCamHandle_, SynchronousNextEmpty, size, numBuffers, bufAddress);
	} else {
		std::cerr << "Error: Couldn't open camera (" << telStatus << ")" <<  std::endl;
		exiting_ = true;
		return -2;
	}

	uint32_t readoutMode = 0;
	printf("Setting readout mode: %d ...\t", readoutMode);
	telStatus = GevSetFeatureValue(telCamHandle_, "ReadOutMode", sizeof(readoutMode), &readoutMode);
	if (telStatus==GEVLIB_OK){
		printf("Success\n");
	} else {
		printf("Failure\n");
	}
	int num_images = 30;
	telStatus = GevStartTransfer(telCamHandle_, num_images);

	std::string filename = "/home/david/shadobox.raw";
	uint16_t img_cnt = 0;
	double avg_exposure;
	while (img_cnt<num_images){
		printf("Press any jey to continue...\n");
		system("read");
		printf("Image counter: %d\n", img_cnt++);
		GEV_BUFFER_OBJECT *img = NULL;
		std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
		telStatus = GevWaitForNextFrame(telCamHandle_, &img, 5000000); // Timeout in ms, exposure + readout about 98 ms.
    	std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
		avg_exposure += std::chrono::duration_cast<std::chrono::microseconds>(end-begin).count()/1e6;
		std::cout << "Measured actual exposure time= " << std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count()/1e6 << "[s]" << std::endl;
		if (telStatus==GEVLIB_OK){
			std::ofstream rawFile(filename, std::ios::out | std::ios::binary);
			char val[2];
			uint32_t size = img->h*img->w*img->d;
			uint32_t num_pixels = img->h*img->w;
			//printf("Image state: %d\n", img->state);
			//printf("Image status: %d\n", img->status);
			//printf("Image format: %d\n", img->format);
			//printf("Image size: %d\n", size);
			char *p = (char *)img->address;
			for (auto i=0; i<num_pixels; i++){
				val[0] = p[2*i];
				val[1] = p[2*i+1];
				if (i<10){
						std::cout << *(reinterpret_cast<uint16_t*>(val)) << " ";
				}
				rawFile.write(val, sizeof(val));
			}
			std::cout << std::endl;
			rawFile.close();
		} else {
			std::cerr << "Error waiting for frame" << std::endl;
		}
		if (img!=NULL){
			GevReleaseFrame(telCamHandle_, img);
		}
		//double temperature = 0.0;
		//GevGetFeatureValue(telCamHandle_, "DeviceTemperature", &type, sizeof(temperature), &temperature);
		//printf("Device temp: %3.3fC\n", temperature);
	}
	avg_exposure /= num_images;
	printf("Avg exposure for %d imageis: %f [s]\n", num_images, avg_exposure);
	printf("Exposure time: %f, overhead: %f\n", (double)(expTime)/1e6, avg_exposure-(double)(expTime)/1e6);

	telStatus = GevAbortTransfer(telCamHandle_);
	telStatus = GevFreeTransfer(telCamHandle_);
	GevCloseCamera(&telCamHandle_);
	GevApiUninitialize();
	_CloseSocketAPI();
}

uint16_t sendTrigger(GEV_CAMERA_HANDLE camHandle){
	uint64_t readoutTime = 98581; //Shadobox s
	GEV_STATUS telStatusThread;
	telStatusThread = GevSetFeatureValue(camHandle, "SoftwareTrigger", sizeof(acqStart), &acqStart);
	//std::this_thread::sleep_for(std::chrono::microseconds(readoutTime+(uint64_t)(1000)));
	//telStatusThread = GevSetFeatureValue(camHandle, "SoftwareTrigger", sizeof(acqStop), &acqStop);
	return 0;
}
void signal_callback_handler(int signum){
	std::cout << "New exposure time [s]: " << std::endl;
	std::cin >> exposureTime;
	printf("Setting exposure time: %f ...\t", exposureTime);
	uint32_t expTimeMicroSec = (uint32_t)(1e6*exposureTime);
	telStatus = GevSetFeatureValue(telCamHandle_, "FrameInterval", sizeof(expTimeMicroSec), &expTimeMicroSec);
	if (telStatus==GEVLIB_OK){
		printf("Success\n");
	} else {
		printf("Failure\n");
	}
}
