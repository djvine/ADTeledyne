/* This is a driver for the Teledyne DALSA GigE-V family of cameras. 
 * Why not use the Genicam driver? I couldn't get it to work so I wrote this to
 * give some basic functionality.
 *
 * Author: D. J. Vine
 * Date: 18 August 2020
 */

#include <iostream>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <vector>
#include <array>
#include <fstream>
#include <algorithm>
#include <math.h>
#include <chrono>
#include <thread>

#include <epicsEvent.h>
#include <epicsTime.h>
#include <epicsThread.h>
#include <iocsh.h>
#include <epicsString.h>
#include <epicsExit.h>
#include <epicsExport.h>

#include "cordef.h"
#include "gevapi.h"

#include <ADDriver.h>

#define NUM_TELEDYNE_PARAMS 0
#define MAX_NETIF 2
#define MAX_CAMERAS_PER_NETIF 32
#define MAX_CAMERAS (MAX_NETIF * MAX_CAMERAS_PER_NETIF)
#define NUM_BUFFERS 1

static const char* driverName = "teledyne";

class teledyne : public ADDriver
{
	public:
		teledyne(const char* portName, int maxBuffers, size_t maxMemory, int priority, int stackSize, int camId, char* serial_number);

		/* Virtual methods to override from ADDriver */
		virtual asynStatus writeInt32( asynUser *pasynUser, epicsInt32 value) override;
		virtual asynStatus writeFloat64( asynUser *pasynUser, epicsFloat64 value) override;

		void imageGrabTask();
		void tempReadTask();
		void shutdown();
		void report(FILE *fp, int details);

	protected:
		int teleFirstParam;
#define FIRST_TELE_PARAM teleFirstParam
		int teleLastParam;
#define LAST_TELE_PARAM teleLastParam

	private:
		asynStatus grabImage(GEV_BUFFER_OBJECT *img);
		asynStatus startCapture();
		asynStatus stopCapture();


		asynStatus connectCamera();
		asynStatus disconnectCamera();
		asynStatus setExposureTime(double expTime);
		asynStatus setReadoutMode(uint32_t value);
		uint16_t median(std::vector<uint16_t> pixVals);
		double median(std::vector<double> pixVals);
		void defectPixelCorrection(std::vector<double> &img);
		void defectPixelCorrection(NDArray* pRaw_);
		void offsetAndGainCorrection(NDArray* pRaw_);
		void readCalibrationData(void);
		void endswap(uint16_t *objp);
		void saveRAWImage(NDArray* pRaw_, std::string filename);
		void saveRAWImage(std::vector<double> img, std::string filename);
		static void sendTrigger(double exposureTime, GEV_CAMERA_HANDLE telHandle);

		int exiting_;
		epicsEventId startEventId_;
		NDArray *pRaw_;
		int camId_;
		char* serial_number;

		// Device parameters
		double exposureTime = 1.0; // seconds

		GEV_DEVICE_INTERFACE telCamInt_[MAX_CAMERAS] = {0};
		GEV_CAMERA_HANDLE telCamHandle_ = NULL;
		GEV_CAMERA_OPTIONS telCamOpt_ = {0};
		GEVLIB_CONFIG_OPTIONS telLibOpt_ = {0};

		int numBuffers = NUM_BUFFERS;
		unsigned char* bufAddress[NUM_BUFFERS];

		// Shadobox correction images
		std::vector<std::array<uint16_t, 2>> defectPixelMap; // X,Y coordinate
		std::vector<std::array<uint16_t, 3>> defectColumnMap; // column, start_pixel, end_pixel
		std::vector<std::array<uint16_t, 3>> defectRowMap; // row, start_pixel, end_pixel
		std::vector<double> gainMap;
		std::vector<double> offsetMap;
		double gain_avg;
};

#define NUM_TELE_PARAMS ((int)(&LAST_TELE_PARAM-&FIRST_TELE_PARAM+1))

/* Configuration function to configure one camera
 *
 * This function needs to be called once for each camera used by the IOC. A
 * call to this function instantiates one object of the Tucsen class.
 * \param[in] portName asyn port to assign to the camera
 * \param[in] maxBuffers Maximum number of NDArray objects (image buffers) this
 *            driver is allowed to allocate.
 *            0 = unlimited
 * \param[in] maxMemory Maximum memort (in bytes) that this driver is allowed
 *            to allocate.
 *            0=unlimited
 * \param[in] priority The epics thread priority for this driver. 0= asyn
 *            default.
 * \param[in] stackSize The size of the stack of the EPICS port thread. 0=use
 *            asyn default.
 */
extern "C" int teledyneConfig(const char *portName, int maxBuffers, size_t maxMemory, int priority, int stackSize, int camId, char* serial_number)
{
	new teledyne(portName, maxBuffers, maxMemory, priority, stackSize, camId, serial_number);
	return asynSuccess;
}

static void c_shutdown(void *arg)
{
	teledyne *t = (teledyne *)arg;
	t->shutdown();
}

static void imageGrabTaskC(void *drvPvt)
{
	teledyne *t = (teledyne *)drvPvt;
	t->imageGrabTask();
}

static void tempReadTaskC(void *drvPvt)
{
	teledyne *t = (teledyne *)drvPvt;
	t->tempReadTask();
}

/* Constructor for Teledyne class */
teledyne::teledyne(const char *portName, int maxBuffers, size_t maxMemory, int priority, int stackSize, int camId, char* serial_number)
	: ADDriver( portName, 1, NUM_TELE_PARAMS, maxBuffers, maxMemory, asynEnumMask, asynEnumMask, ASYN_CANBLOCK | ASYN_MULTIDEVICE, 1, priority, stackSize),
	  exiting_(0), pRaw_(NULL), camId_(camId), serial_number(serial_number)
{
	static const char *functionName = "teledyne";

	asynStatus status;
	int traceMask = ASYN_TRACE_ERROR;
	pasynTrace->setTraceMask(pasynUserSelf, traceMask);


	readCalibrationData();
	status = connectCamera();

	if(status){
		asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
				"%s:%s: camera connection failed (%d)\n",
				driverName, functionName, status);
		report(stdout, 1);
		exit(1);
	}
	
	startEventId_ = epicsEventCreate(epicsEventEmpty);

	/* Launch image read task */
	asynPrint(pasynUserSelf, ASYN_TRACEIO_DRIVER,
			"%s:%s: create image read thread\n",
			driverName, functionName);
	epicsThreadCreate("TeledyneImageReadTask",
			epicsThreadPriorityMedium,
			epicsThreadGetStackSize(epicsThreadStackMedium),
			imageGrabTaskC, this);

	/* Launch image read task
	 * The GeVlib connection will be closed by the API if there is no
	 * communication for 10 seconds (default number). This task
	 * serves to keep the connectoin alive.
	 */
	asynPrint(pasynUserSelf, ASYN_TRACEIO_DRIVER,
			"%s:%s: create temp read thread\n",
			driverName, functionName);
	epicsThreadCreate("TeledyneTempReadTask",
			epicsThreadPriorityMedium,
			epicsThreadGetStackSize(epicsThreadStackMedium),
			tempReadTaskC, this);

	/* Launch shutdown task */
	epicsAtExit(c_shutdown, this);

	return;
}

void teledyne::shutdown(void)
{
	exiting_=1;
	disconnectCamera();
}

void teledyne::tempReadTask(void){
	double temp = 0.0;
	int type;
	while(!exiting_){
		GevGetFeatureValue(telCamHandle_, "DeviceTemperature", &type, sizeof(temp), &temp);
		lock();
		setDoubleParam(ADTemperatureActual, temp);
		callParamCallbacks();
		unlock();
		epicsThreadSleep(1.0);
	}
}

void teledyne::endswap(uint16_t *objp)
{
	unsigned char *memp = reinterpret_cast<unsigned char*>(objp);
	std::reverse(memp, memp+sizeof(uint16_t));
}

void teledyne::readCalibrationData(void)
{
	static const char* functionName = "readCalibrationData";

	std::string fname_pixmap="../../../../teledyneSupport/defect_correction_data/";
    fname_pixmap.append(serial_number);
    fname_pixmap.append("/");
    fname_pixmap.append(serial_number);
	fname_pixmap.append("_Pixel_Map");
	std::ifstream defectDataStream(fname_pixmap);
	std::string line;
	while (std::getline(defectDataStream, line))
	{
		std::cout << line << std::endl;
		if (line.size() && line[0]=='P'){ // Defect pixel
			uint16_t x_pix = std::stoi(line.substr(1, line.find_first_of(",")-1))-1;
			uint16_t y_pix = std::stoi(line.substr(line.find_first_of(",")+1, line.size()))-1;
			std::cout << "Defect pixel: " << x_pix << ", " << y_pix << std::endl;
			defectPixelMap.push_back({{x_pix, y_pix}});
		} else if (line.size() && line[0]=='C'){
			uint16_t c_num   = std::stoi(line.substr(1, line.find_first_of(" ")-1))-1;
			uint16_t c_start = std::stoi(line.substr(line.find_first_of(" ")+1, line.find_first_of("-")-1))-1;
			uint16_t c_end   = std::stoi(line.substr(line.find_first_of("-")+1, line.size()))-1;
			std::cout << "Defect column: " << c_num<< ": " << c_start << ", " << c_end << std::endl;
			defectColumnMap.push_back({{c_num, c_start, c_end}});
		}
	}

	std::string fname_gainmap="../../../../teledyneSupport/defect_correction_data/";
    fname_gainmap.append(serial_number);
    fname_gainmap.append("/");
    fname_gainmap.append(serial_number);
    fname_gainmap.append("_Gain_Image.raw");
	std::cout << "Gain image: " << fname_gainmap << std::endl;
	std::ifstream gainDataStream(fname_gainmap, std::ios::binary);
	uint16_t ivalue;
	double dvalue;
	while (gainDataStream.read(reinterpret_cast<char*>(&ivalue), sizeof(ivalue)))
	{
		// swap endianess
		endswap(&ivalue);
		dvalue = (double)ivalue;
		gainMap.push_back(dvalue);
	}
	defectPixelCorrection(gainMap);
	saveRAWImage(gainMap, "gain_map.raw");
	for (auto i=0; i<gainMap.size(); i++){
		gain_avg+=gainMap[i];
	}
	gain_avg /= gainMap.size();
	std::cout << "Gain average: " << gain_avg << std::endl;
	std::cout << std::endl << "Total elements: " << gainMap.size() << std::endl;
	for (auto i=0; i<gainMap.size(); i++){
		gainMap[i]/=gain_avg;
	}
	saveRAWImage(gainMap, "gain_map_norm.raw");
	
	std::string fname_offsetmap="../../../../teledyneSupport/defect_correction_data/";
    fname_offsetmap.append(serial_number);
    fname_offsetmap.append("/");
    fname_offsetmap.append(serial_number);
    fname_offsetmap.append("_Offset_Image.raw");
	std::ifstream offsetDataStream(fname_offsetmap, std::ios::binary);
	while (offsetDataStream.read(reinterpret_cast<char*>(&ivalue), sizeof(ivalue)))
	{
		endswap(&ivalue);
		dvalue = (double)ivalue;
		offsetMap.push_back(dvalue);
	}
	std::cout << "Pixel correct offset map" << std::endl;
	defectPixelCorrection(offsetMap);
	saveRAWImage(offsetMap, "offset_map.raw");
	std::cout << std::endl << "Total elements: " << offsetMap.size() << std::endl;

	std::cout << "Number defective pixels: " << defectPixelMap.size() << std::endl;
	std::cout << "Number defective columns: " << defectColumnMap.size() << std::endl;
}

asynStatus teledyne::connectCamera()
{
	static const char* functionName = "connectCamera";
	GEV_STATUS telStatus;
	int numCameras;
	int type;
	unsigned int height, width, format;
	unsigned long int size;
	char synchModeStr[64] = {0};

	GevGetLibraryConfigOptions(&telLibOpt_);
	telLibOpt_.logLevel = GEV_LOG_LEVEL_NORMAL;
	GevSetLibraryConfigOptions(&telLibOpt_);

	telStatus = GevGetCameraList(telCamInt_, MAX_CAMERAS, &numCameras);

	printf("%d cameras on the network\n", numCameras);
	if (numCameras==0){
		asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
				"%s:%s: No cameras detected on network\n",
				driverName, functionName);
		return asynError;
	}
	if (camId_>numCameras){
		asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
				"%s:%s: Requested camera id (%d) greater than number of devices detected (%d)\n",
				driverName, functionName, camId_, numCameras);
		return asynError;
	}

	asynPrint(pasynUserSelf, ASYN_TRACEIO_DRIVER,
			"%s:%s: opening camera\n",
			driverName, functionName);

	//telStatus = GevOpenCamera(&telCamInt_[camId_], GevControlMode, &telCamHandle_);
	telStatus = GevOpenCameraBySN(serial_number, GevControlMode, &telCamHandle_);
	if (telStatus==0){
		char xmlFileName[1024] = {0};
		telStatus = GevGetGenICamXML_FileName( telCamHandle_, (int)sizeof(xmlFileName), xmlFileName);
		if (telStatus==GEVLIB_OK){
			printf("XML stored as: %s\n", xmlFileName);
		}

		/* Set initial values for some parameters */
		setIntegerParam(NDDataType, NDUInt16);
		setIntegerParam(NDColorMode, NDColorModeMono);
		setIntegerParam(NDArraySizeZ, 0);
		setIntegerParam(ADMinX, 0);
		setIntegerParam(ADMinY, 0);
		GevGetFeatureValue(telCamHandle_, "Width", &type, sizeof(width), &width);
		setIntegerParam(ADSizeX, width);
		setIntegerParam(ADMaxSizeX, width);
		setIntegerParam(NDArraySizeX, width);
		GevGetFeatureValue(telCamHandle_, "Height", &type, sizeof(height), &height);
		setIntegerParam(ADSizeY, height);
		setIntegerParam(ADMaxSizeY, height);
		setIntegerParam(NDArraySizeY, height);
		GevGetFeatureValue(telCamHandle_, "PixelFormat", &type, sizeof(format), &format);
		switch (format){
			case 0x1100025:
				std::cout << "Pixel format: 14 bit mono unsigned" << std::endl;
				break;
			default:
				std::cout << "Unknown pixel format!" << std::endl;
		}
		setIntegerParam(NDArraySize, GetPixelSizeInBytes(format)*width*height);
		char stringFeature[32];
		GevGetFeatureValue(telCamHandle_, "DeviceVendorName", &type, sizeof(stringFeature), &stringFeature);
		setStringParam(ADManufacturer, stringFeature);
		memset(&stringFeature[0], 0, sizeof(stringFeature));
		GevGetFeatureValue(telCamHandle_, "DeviceModelName", &type, sizeof(stringFeature), &stringFeature);
		setStringParam(ADModel, stringFeature);
		memset(&stringFeature[0], 0, sizeof(stringFeature));
		GevGetFeatureValue(telCamHandle_, "DeviceVersion", &type, sizeof(stringFeature), &stringFeature);
		setStringParam(ADFirmwareVersion, stringFeature);
		memset(&stringFeature[0], 0, sizeof(stringFeature));
		GevGetFeatureValue(telCamHandle_, "DeviceProductIDBuild", &type, sizeof(stringFeature), &stringFeature);
		setStringParam(ADSDKVersion, stringFeature);
		memset(&stringFeature[0], 0, sizeof(stringFeature));
		GevGetFeatureValue(telCamHandle_, "DeviceID", &type, sizeof(stringFeature), &stringFeature);
		setStringParam(ADSerialNumber, stringFeature);
		GevGetFeatureValueAsString(telCamHandle_, "SynchronizationMode", &type, sizeof(synchModeStr), synchModeStr);
		GevSetFeatureValueAsString(telCamHandle_, "SynchronizationMode", "Snapshot");
		callParamCallbacks();

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

		uint32_t size = width*height*GetPixelSizeInBytes(format);
		for (int i=0; i<numBuffers; i++){
			bufAddress[i] = (PUINT8)malloc(size);
			memset(bufAddress[i], 0, size);
		}

		telStatus = GevInitializeTransfer(telCamHandle_, SynchronousNextEmpty, size, numBuffers, bufAddress);
	} else {
		std::cerr << "Error: Couldn't open camera (" << telStatus << ")" <<  std::endl;
		return asynError;
	}

	callParamCallbacks();
	return asynSuccess;
}

void teledyne::report(FILE *fp, int details)
{
	static const char* functionName = "report";
	ADDriver::report(fp, details);
	return;
}

asynStatus teledyne::disconnectCamera(void){
	static const char *functionName = "disconnectCamera";
	int acquiring;
	asynStatus status;
	GEV_STATUS telStatus;

	// Check if acquiring
	status = getIntegerParam(ADAcquire, &acquiring);

	// If necessary stop acquisition
	if (status==asynSuccess && acquiring){
		telStatus = GevStopTransfer(telCamHandle_);
		if (telStatus!=0){
			asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
					"%s:%s: unable to abort acquisition (%d)\n\n",
					driverName, functionName, 
					telStatus);
		}
	}
	GevCloseCamera(&telCamHandle_);
	GevApiUninitialize();
	_CloseSocketAPI();

	return asynSuccess;
}

asynStatus teledyne::setReadoutMode(uint32_t value){
	// Binning
	GEV_STATUS telStatus;
	telStatus = GevSetFeatureValue(telCamHandle_, "ReadOutMode", sizeof(value), &value);
	if (telStatus==GEVLIB_OK){
		return asynSuccess;
	} else {
		return asynError;
	}
}

asynStatus teledyne::startCapture()
{
	static const char* functionName = "startCapture";
	lock();
	asynPrint(pasynUserSelf, ASYN_TRACEIO_DRIVER,
			"%s:%s: Begin capture.\n",
			driverName, functionName);
	setIntegerParam(ADNumImagesCounter, 0);
	callParamCallbacks();
	unlock();
	epicsEventSignal(startEventId_);
	return asynSuccess;
}

asynStatus teledyne::stopCapture()
{
	static const char* functionName = "stopCapture";
	lock();
	asynPrint(pasynUserSelf, ASYN_TRACEIO_DRIVER,
			"%s:%s: Stop capture.\n",
			driverName, functionName);
	setIntegerParam(ADAcquire, 0);
	setIntegerParam(ADStatus, ADStatusIdle);
	callParamCallbacks();
	unlock();
	return asynSuccess;
}

asynStatus teledyne::writeInt32( asynUser *pasynUser, epicsInt32 value)
{
	static const char* functionName = "writeInt32";
	int function = pasynUser->reason;
	const char* paramName;
	asynStatus status;

	getParamName(function, &paramName);
	status = setIntegerParam(function, value);

	if (function==ADAcquire){
		if(value){
			status = startCapture();
		} else {
			status = stopCapture();
		}
	} else if ((function==ADBinX)||(function==ADBinY)){
		if (value<1) value=1;
		if (value>2) value=2;
		setIntegerParam(ADBinX, value);
		setIntegerParam(ADBinY, value);
		setReadoutMode(value-1);
	} else {
		if (function<FIRST_TELE_PARAM){
			status = ADDriver::writeInt32(pasynUser, value);
		}
	}

	asynPrint(pasynUserSelf, ASYN_TRACEIO_DRIVER,
		"%s:%s: function=%d, value=%d, status=%d\n",
		driverName, functionName, function, value, status);
	callParamCallbacks();
	return status;
}

asynStatus teledyne::writeFloat64(asynUser *pasynUser, epicsFloat64 value){
	static const char* functionName = "writeFloat64";
	asynStatus status;

	int function = pasynUser->reason;
	status = setDoubleParam(function, value);

	if (function==ADAcquireTime){
		if (value<0.013){
			value=0.013;
			setDoubleParam(ADAcquireTime, value);
		} else if (value>65.535999){
			value=65.535999;
			setDoubleParam(ADAcquireTime, 65.535999);
		}
		exposureTime = value;
	} else {
		if (function < FIRST_TELE_PARAM){
			status = ADDriver::writeFloat64(pasynUser, value);
		}
	}
	callParamCallbacks();
	return status;
}	

void teledyne::sendTrigger(double expTime, GEV_CAMERA_HANDLE telHandle){
	uint64_t readoutTime = 98581; //Shadobox s
	uint16_t acqStart = 1;
	uint16_t acqStop  = 0;
	GEV_STATUS telStatusThread;
	// This delay is required to make sure that main thread is in WaitForFrame before a trigger is sent
	//std::this_thread::sleep_for(std::chrono::microseconds(100));
	//std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
	telStatusThread = GevSetFeatureValue(telHandle, "SoftwareTrigger", sizeof(acqStart), &acqStart);
	std::this_thread::sleep_for(std::chrono::microseconds(readoutTime+(uint64_t)(1e6*expTime)));
	telStatusThread = GevSetFeatureValue(telHandle, "SoftwareTrigger", sizeof(acqStop), &acqStop);
    //std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
	//std::cout << "Measured set exposure time= " << std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count() << "[µs]" << std::endl;
	//printf("Thread: Done\n");
}

void teledyne::imageGrabTask(void){
	static const char* functionName = "imageGrabTask";
	int acquire;
	int imageMode;
	GEV_STATUS telStatus;
	int imageCounter;
	int numImages;
	int numImagesCounter;
	int arrayCallbacks;
	asynStatus status;
	uint32_t missedFrames = 0;

	lock();
	telStatus = GevStartTransfer(telCamHandle_, -1);
	while(1){
		/* Is acquisition active */
		getIntegerParam(ADAcquire, &acquire);

		/* If we are not acquiring wait for a semaphore  that is given when
		 * acquisition is started */
		if(!acquire){
			setIntegerParam(ADStatus, ADStatusIdle);
			callParamCallbacks();

			asynPrint(pasynUserSelf, ASYN_TRACEIO_DRIVER,
					"%s:%s: Waiting for acquisition to start.\n",
					driverName, functionName);
			unlock();
			epicsEventWait(startEventId_);
			lock();
			setIntegerParam(ADStatus, ADStatusAcquire);
			setIntegerParam(ADNumImagesCounter, 0);
			callParamCallbacks();
			asynPrint(pasynUserSelf, ASYN_TRACEIO_DRIVER,
					"%s:%s: Acquisition started.\n",
					driverName, functionName);
		}
		// Wait for new buffer
		unlock();
		GEV_BUFFER_OBJECT *img = NULL;
		std::thread t1(sendTrigger, exposureTime, telCamHandle_);
		//std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
		telStatus = GevWaitForNextFrame(telCamHandle_, &img, 1000000*exposureTime+500);
    	//std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
		//std::cout << "Measured actual exposure time= " << std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count() << "[µs]" << std::endl;
		t1.join();
		lock();

		if (telStatus==GEVLIB_OK){
			status = grabImage(img);
			if (status==asynError){
				if (pRaw_) pRaw_->release();
				pRaw_ = NULL;
				continue;
			}

			getIntegerParam(NDArrayCounter, &imageCounter);
			getIntegerParam(ADNumImages, &numImages);
			getIntegerParam(ADNumImagesCounter, &numImagesCounter);
			getIntegerParam(ADImageMode, &imageMode);
			getIntegerParam(NDArrayCallbacks, &arrayCallbacks);
			imageCounter++;
			numImagesCounter++;
			setIntegerParam(NDArrayCounter, imageCounter);
			setIntegerParam(ADNumImagesCounter, numImagesCounter);
			callParamCallbacks();

			if (arrayCallbacks){
		            doCallbacksGenericPointer(pRaw_, NDArrayData, 0);
			}

			if ((imageMode==ADImageSingle)||((imageMode==ADImageMultiple)&&(numImagesCounter>=numImages))){
				status = stopCapture();
			}
                
		} else {
			std::cerr << "Error waiting for next buffer (" << telStatus << ")" << std::endl;
			missedFrames++;
			printf("Missed frames: %d (Fuck you Teledyne!)\n", missedFrames);
		}
		if (pRaw_)pRaw_->release();
		pRaw_ = NULL;
		if (img!=NULL){
			GevReleaseFrame(telCamHandle_, img);
		}
	}
}

asynStatus teledyne::grabImage(GEV_BUFFER_OBJECT *img)
{
	static const char* functionName = "grabImage";
	size_t dims[2];
	NDDataType_t dataType;
	NDColorMode_t colorMode;
	unsigned int dataBufSize;
	int bitDepth;
	uint32_t size;

	setIntegerParam(ADStatus, ADStatusReadout);
	callParamCallbacks();

	dims[0] = img->w;
	dims[1] = img->h;
	bitDepth = img->d;
	size = dims[0]*dims[1]*bitDepth;

	dataType = NDUInt16;
	colorMode = NDColorModeMono;
	setIntegerParam(NDArraySizeX, dims[0]);
	setIntegerParam(NDArraySizeY, dims[1]);
	setIntegerParam(NDArraySize, dims[0]*dims[1]*bitDepth);
	setIntegerParam(NDDataType, dataType);
	setIntegerParam(NDColorMode, colorMode);
	callParamCallbacks();
	pRaw_ = pNDArrayPool->alloc(2, dims, dataType, 0, NULL);
	if (!pRaw_){
		setIntegerParam(ADStatus, ADStatusAborting);
		callParamCallbacks();
		asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
				"%s:%s: Unable to allocate buffer\n",
				driverName, functionName);
		setIntegerParam(ADAcquire, 0);
		return asynError;
	} else {
		memcpy(pRaw_->pData, img->address, size);
	}

	// Area detector bad pixel plugin now handles the following line
	//defectPixelCorrection(pRaw_);
	// Uncomment this line to apply gain and offset correction to each image
	//offsetAndGainCorrection(pRaw_);
	
	setIntegerParam(NDArrayCounter, img->id);
	if (pRaw_){
		pRaw_->uniqueId = img->id;
		updateTimeStamp(&pRaw_->epicsTS);
		pRaw_->timeStamp = pRaw_->epicsTS.secPastEpoch+pRaw_->epicsTS.nsec/1e9;
		getAttributes(pRaw_->pAttributeList);
		pRaw_->pAttributeList->add("ColorMode", "Color Mode", NDAttrInt32, &colorMode);
	}


	setIntegerParam(ADStatus, ADStatusIdle);
	callParamCallbacks();
	return asynSuccess;
}

void teledyne::saveRAWImage(std::vector<double> img, std::string filename){
	std::ofstream rawFile(filename, std::ios::out | std::ios::binary);
	char val[2];
	for (auto i=0; i<img.size(); i++){
		val[0] = (uint16_t)(round(img[i]))&0xFF;
		val[1] = ((uint16_t)(round(img[i]))>>8)&0xFF;
		rawFile.write(val, sizeof(val));
	}
	rawFile.close();
}

void teledyne::saveRAWImage(NDArray* pRaw_, std::string filename){
	std::ofstream rawFile(filename, std::ios::out | std::ios::binary);
	char val8[2];
	uint16_t val16;
	for (auto i=0; i<pRaw_->dataSize; i++){
		val16 = reinterpret_cast<uint16_t*>(pRaw_->pData)[i];
		val8[0] = val16&0xFF;
		val8[1] = (val16 >> 8) & 0xFF;
		rawFile.write(val8, sizeof(val8));
	}
	rawFile.close();
}

void teledyne::defectPixelCorrection(NDArray* pRaw_)
{
	char* functionName = "defectPixelCorrection";
	auto filter_kernel = 5;

	// Correct defective pixels
	std::vector<uint16_t> patch;

	uint16_t row_size = 2304;
	uint16_t col_size = 2940;
	for (auto i=0; i<defectPixelMap.size(); i++){
		patch.clear();
		uint16_t py = defectPixelMap[i][0];
		uint16_t px = defectPixelMap[i][1];
		for (auto j=0; j<filter_kernel; j++){
			for (auto k=0; k<filter_kernel; k++){
				int16_t mx = px-2+j;
				int16_t my = py-2+k;
				if ((mx<0) || (mx>col_size) || (my<0) || (my>row_size)){
					goto skip;
				}
				// Don't use if defect pixel
				for (auto l=0; l < defectPixelMap.size(); l++){
					if ((mx==defectPixelMap[l][1])&&(my==defectPixelMap[l][0])){
						goto skip;
					}
				}
				// Don't use if in defect column
				for (auto l=0; l<defectColumnMap.size(); ++l){
					if ((my==defectColumnMap[l][0])&&(mx>defectColumnMap[l][1])&&(mx<defectColumnMap[l][2])){
						goto skip;
					}
				}
				patch.push_back(reinterpret_cast<uint16_t*>(pRaw_->pData)[mx*row_size+my]);
skip:;
			}
		}
		reinterpret_cast<uint16_t*>(pRaw_->pData)[px*row_size+py] = median(patch);
	}

	// Correct defective columns
	std::vector<uint16_t> line;
	for (auto i=0; i<defectColumnMap.size(); i++){
		uint16_t cstart = defectColumnMap[i][1];
		uint16_t cend   = defectColumnMap[i][2];
		uint16_t py = defectColumnMap[i][0];
		for (uint16_t px=cstart; px<cend; px++){
			line.clear();
			for (auto j=0; j<filter_kernel; j++){
				uint16_t mx = px;
				uint16_t my = py-2+j;
				if ((mx<0) || (mx>col_size)){
					goto cskip;
				}
				// Don't use if defect pixel
				for (auto l=0; l<defectPixelMap.size(); l++){
					if ((mx==defectPixelMap[l][1])&&(my==defectPixelMap[l][0])){
						goto cskip;
					}
				}
				// Don't use if in defect column
				for (auto l=0; l<defectColumnMap.size(); l++){
					if ((my==defectColumnMap[l][0])&&(mx>defectColumnMap[l][1])&&(mx<defectColumnMap[l][2])){
						goto cskip;
					}
				}

				line.push_back(reinterpret_cast<uint16_t*>(pRaw_->pData)[mx*row_size+my]);
cskip:;
			}
			reinterpret_cast<uint16_t*>(pRaw_->pData)[px*row_size+py] = median(line);
		}
	}

}

void teledyne::defectPixelCorrection(std::vector<double> &img){
	char* functionName = "defectPixelCorrection";
	auto filter_kernel = 5;

	// Correct defective pixels
	std::vector<double> patch;

	uint16_t row_size = 2304;
	uint16_t col_size = 2940;
	for (auto i=0; i<defectPixelMap.size(); i++){
		patch.clear();
		uint16_t py = defectPixelMap[i][0];
		uint16_t px = defectPixelMap[i][1];
		for (auto j=0; j<filter_kernel; j++){
			for (auto k=0; k<filter_kernel; k++){
				int16_t mx = px-2+j;
				int16_t my = py-2+k;
				if ((mx<0) || (mx>col_size) || (my<0) || (my>row_size)){
					goto skip;
				}
				// Don't use if defect pixel
				for (auto l=0; l < defectPixelMap.size(); l++){
					if ((mx==defectPixelMap[l][1])&&(my==defectPixelMap[l][0])){
						goto skip;
					}
				}
				// Don't use if in defect column
				for (auto l=0; l<defectColumnMap.size(); ++l){
					if ((my==defectColumnMap[l][0])&&(mx>defectColumnMap[l][1])&&(mx<defectColumnMap[l][2])){
						goto skip;
					}
				}
				patch.push_back(img[mx*row_size+my]);
skip:;
			}
		}
		img[px*row_size+py] = median(patch);
	}

	// Correct defective columns
	std::vector<double> line;
	for (auto i=0; i<defectColumnMap.size(); i++){
		uint16_t cstart = defectColumnMap[i][1];
		uint16_t cend   = defectColumnMap[i][2];
		uint16_t py = defectColumnMap[i][0];
		for (uint16_t px=cstart; px<cend; px++){
			line.clear();
			for (auto j=0; j<filter_kernel; j++){
				uint16_t mx = px;
				uint16_t my = py-2+j;
				if ((mx<0) || (mx>col_size)){
					goto cskip;
				}
				// Don't use if defect pixel
				for (auto l=0; l<defectPixelMap.size(); l++){
					if ((mx==defectPixelMap[l][1])&&(my==defectPixelMap[l][0])){
						goto cskip;
					}
				}
				// Don't use if in defect column
				for (auto l=0; l<defectColumnMap.size(); l++){
					if ((my==defectColumnMap[l][0])&&(mx>defectColumnMap[l][1])&&(mx<defectColumnMap[l][2])){
						goto cskip;
					}
				}

				line.push_back(img[mx*row_size+my]);
cskip:;
			}
			img[px*row_size+py] = median(line);
		}
	}

}

void teledyne::offsetAndGainCorrection(NDArray* pRaw_)
{
	// subtract offset and correct gain
	double tmp = 0.0;
	for (auto i=0; i<pRaw_->dataSize; i++){
		tmp = reinterpret_cast<uint16_t*>(pRaw_->pData)[i]-offsetMap[i];
		tmp /= gainMap[i];
		reinterpret_cast<uint16_t*>(pRaw_->pData)[i] = (uint16_t)round(tmp);
	}
}
	
uint16_t teledyne::median(std::vector<uint16_t> pixVals){
	size_t size = pixVals.size();

	if (size==0){
		return 0;
	} else {
		std::sort(pixVals.begin(), pixVals.end());
		if (size%2==0){
			return (uint16_t)(0.5*(pixVals[size/2-1]+pixVals[size/2]));
		} else {
			return pixVals[size/2];
		}
	}
}

double teledyne::median(std::vector<double> pixVals){
	size_t size = pixVals.size();

	if (size==0){
		return 0;
	} else {
		std::sort(pixVals.begin(), pixVals.end());
		if (size%2==0){
			return 0.5*(pixVals[size/2-1]+pixVals[size/2]);
		} else {
			return pixVals[size/2];
		}
	}
}

static const iocshArg configArg0 = {"Port name",     iocshArgString};
static const iocshArg configArg1 = {"maxBuffers",    iocshArgInt};
static const iocshArg configArg2 = {"maxMemory",     iocshArgInt};
static const iocshArg configArg3 = {"priority",      iocshArgInt};
static const iocshArg configArg4 = {"stackSize",     iocshArgInt};
static const iocshArg configArg5 = {"camId",       	 iocshArgInt};
static const iocshArg configArg6 = {"serial_number", iocshArgString};
static const iocshArg * const configArgs [] = {&configArg0,
					       &configArg1,
					       &configArg2,
					       &configArg3,
					       &configArg4,
				           &configArg5,
						   &configArg6};
static const iocshFuncDef configTeledyne = {"teledyneConfig", 7, configArgs};
static void configCallFunc(const iocshArgBuf *args)
{
	teledyneConfig(args[0].sval, args[1].ival, args[2].ival,
	             args[3].ival, args[4].ival, args[5].ival, args[6].sval);
}

static void teledyneRegister(void)
{
	iocshRegister(&configTeledyne, configCallFunc);
}

extern "C" {
	epicsExportRegistrar(teledyneRegister);
}
