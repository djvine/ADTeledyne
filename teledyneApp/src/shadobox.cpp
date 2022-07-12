/* shaobox.cpp
 *
 * This is a driver for the Teledyn Shadobox camera.
 *
 * Author: David Vine
 * 		   Sigray
 *
 * Created:  Ausgust 6 2021
 *
 */

#include <stddef.h>
#include <stdlib.h>
#include <stdarg.h>
#include <math.h>
#include <stdio.h>
#include <errno.h>
#include <string.h>
#include <iostream>
#include <chrono>
#include <thread>

#include <epicsTime.h>
#include <epicsThread.h>
#include <epicsEvent.h>
#include <epicsMutex.h>
#include <epicsString.h>
#include <epicsStdio.h>
#include <epicsMutex.h>
#include <epicsExit.h>
#include <cantProceed.h>
#include <iocsh.h>

#include "ADDriver.h"
#include <epicsExport.h>
#include "shadobox.h"

static const char *driverName = "shadobox";

void shadobox::imageGrabTask(GEV_BUFFER_OBJECT *img)
{
		static const char* functionName = "imageGrabTask";
		size_t dims[2];
		uint16_t byteDepth;
		NDDataType_t dataType;
		NDColorMode_t colorMode;
		uint32_t numBytes;
		dataType = NDUInt16;
		colorMode = NDColorModeMono;
		bool verbose  = true;


		this->lock();
		dims[0] = img->w;
		dims[1] = img->h;
		byteDepth = img->d;
		numBytes = dims[0]*dims[1]*byteDepth;
		setIntegerParam(ADStatus, ADStatusReadout);

		setIntegerParam(NDArraySizeX, dims[0]);
		setIntegerParam(NDArraySizeY, dims[1]);
		setIntegerParam(NDArraySize, numBytes);
		setIntegerParam(NDDataType, dataType);
		setIntegerParam(NDColorMode, colorMode);
		callParamCallbacks();
		if (verbose) printf("GrabTask: Allocating pRaw_\n");
		pRaw_ = pNDArrayPool->alloc(2, dims, dataType, 0, NULL);
		if (!pRaw_){
				setIntegerParam(ADStatus, ADStatusAborting);
				callParamCallbacks();
				asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
								"%s:%s: Unable to allocate buffer\n",
								driverName, functionName);
				setIntegerParam(ADAcquire, 0);
		} else {
				memcpy(pRaw_->pData, img->address, numBytes);
		}

		setIntegerParam(NDArrayCounter, img->id);
		if (pRaw_){
				pRaw_->uniqueId = img->id;
				updateTimeStamp(&pRaw_->epicsTS);
				pRaw_->timeStamp = pRaw_->epicsTS.secPastEpoch+pRaw_->epicsTS.nsec/1e9;
				getAttributes(pRaw_->pAttributeList);
				pRaw_->pAttributeList->add("ColorMode", "Color Mode", NDAttrInt32, &colorMode);
		}

		callParamCallbacks();
		this->unlock();
}

void shadobox::shutdown(void){
		exiting_=1;
		disconnectCamera();
}

void shadobox::reconnectCamera(void){
		disconnectCamera();
		connectCamera();
}

asynStatus shadobox::disconnectCamera(void){
		int acquiring;
		asynStatus status;
		GEV_STATUS telStatus;

		// Check if acuiring
		status = getIntegerParam(ADAcquire, &acquiring);

		// If necessary stop acquisitiom
		if (status==asynSuccess && acquiring){
				telStatus = GevStopTransfer(telCamHandle_);
		}
		for (int i=0; i<numBuffers; i++){
			free(bufAddress[i]);
		}
		if (telCamHandle_){
				printf("Disconnecting camera... ");
				GevAbortTransfer(telCamHandle_);
				telStatus = GevFreeTransfer(telCamHandle_);
				GevCloseCamera(&telCamHandle_);
				GevApiUninitialize();
				_CloseSocketAPI();
				printf("done\n");
		}
		return asynSuccess;
}

static void c_shutdown(void *arg)
{
		shadobox *s = (shadobox *)arg;
		s->shutdown();
}

static void tempReadTaskC(void *drvPvt)
{
    shadobox *pPvt = (shadobox *)drvPvt;

    pPvt->tempReadTask();
}

static void mainDataTaskC(void *drvPvt)
{
    shadobox *pPvt = (shadobox *)drvPvt;

    pPvt->mainDataTask();
}

void shadobox::tempReadTask(){
		int type;
		double temp;
		 while(!exiting_){
				GevGetFeatureValue(telCamHandle_, "DeviceTemperature", &type, sizeof(temp), &temp);
				this->lock();
				setDoubleParam(ADTemperature, temp);
				setDoubleParam(ADTemperatureActual, temp);
				callParamCallbacks();
				this->unlock();
				sleep(1); // seconds
		 }
}

/** This thread interacts with the camera API to start/stop acquiring.
  * It implements the logic for single, multiple or continuous acquisition. */
void shadobox::mainDataTask()
{
    int status = asynSuccess;
    int imageCounter;
    int numImages, numImagesCounter;
    int imageMode, triggerMode;
    int arrayCallbacks;
    int acquire=0;
    double acquireTime;
    epicsTimeStamp startTime;
    const char *functionName = "imageGrabTask";
	GEV_STATUS telStatus;
	epicsEventStatus eventStatus;
	int missedFrames = 0;
	int shadSynch = 0;
	int disconnectCycles = 0;
	bool verbose = true;
	const int pollingTime = 100;
	int waitTimeOut, waitCycles, n;

    this->lock();
    /* Loop forever */
    while (1) {
        /* If we are not acquiring then wait for a semaphore that is given when acquisition is started */
        if (!acquire) {
			setIntegerParam(ADStatus, ADStatusIdle);
			callParamCallbacks();
          /* Release the lock while we wait for an event that says acquire has started, then lock again */
            asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW,
                "%s:%s: waiting for acquire to start\n", driverName, functionName);
            this->unlock();
			if (verbose) printf("MainTsk: Waiting for event\n");
            status = epicsEventWait(startEventId_);
			if (verbose) printf("MainTask: Got start event\n");
			missedFrames = 0;
			disconnectCycles = 0;
            this->lock();
			getIntegerParam(ADNumImages, &numImages);
			getIntegerParam(ADImageMode, &imageMode);
			getIntegerParam(ADTriggerMode, &triggerMode);
            setIntegerParam(ADNumImagesCounter, 0);
			setIntegerParam(ADStatus, ADStatusAcquire);
			getDoubleParam(ADAcquireTime, &acquireTime);
        	setStringParam(ADStatusMessage, "Acquiring");
		    getIntegerParam(ShadTrigMode, &shadSynch);
        	callParamCallbacks();
			waitTimeOut = (int)(2000*acquireTime+132);
			if (waitTimeOut<500) waitTimeOut = 500;
			waitCycles = waitTimeOut/pollingTime;
			if (imageMode == 0){ // Single
					numImages = 1;
			} else if (imageMode == 2){ // Continuous
					numImages = -1;
			}
			GevStartTransfer(telCamHandle_, numImages);
            acquire = 1;
        }

        epicsTimeGetCurrent(&startTime);

		GEV_BUFFER_OBJECT *img = NULL;
		/* If internal triggering send the trigger */
		if (triggerMode!=0){
            setStringParam(ADStatusMessage, "Wait for trigger");
		    callParamCallbacks();
		}
		if (shadSynch==0){ // Snapshot
			if (verbose) printf("MainTask: Sending trigger\n");
			setCameraFeature("SoftwareTrigger", 1);
		}

		if (verbose) printf("MainTask: Entering GevWaitForNextFrame with timeout: %d\n", waitTimeOut);
		auto t_start = std::chrono::high_resolution_clock::now();
		for (n=0; n<waitCycles; n++){
			this->unlock();
			telStatus = GevWaitForNextImage(telCamHandle_, &img, pollingTime);
			this->lock();
			if ((img!=NULL) && (status == GEVLIB_OK)){
				   if (verbose) printf("MainTask: Wait got image\n");
			   	   break;
			}
		    getIntegerParam(ADAcquire, &acquire);
			if (acquire==0){
				if (verbose) printf("MainTask: Wait got abort\n");
		   	   	break; // Acquisition stopped
			}
		}
		auto t_end = std::chrono::high_resolution_clock::now();

		double elapsed_exp = std::chrono::duration<double, std::milli>(t_end-t_start).count();
		printf("MainTask: Elapsed exposure time: %f ms\n", elapsed_exp);
		this->unlock();
		if ((img != NULL) && (telStatus == GEVLIB_OK) ){
				if (img->status==0){
					if (verbose) printf("MainTask: Procesing image\n");
					imageGrabTask(img);
				}
		}
		this->lock();

		getIntegerParam(ADAcquire, &acquire);
        if (!acquire){ // Stop was pressed during continuous acquisition
			printf("MainTask: Aborting\n");
	    	continue; 
		} else if (n>=waitCycles){
			printf("MainTask: Missed frame\n");
			if (pRaw_) pRaw_->release();
			pRaw_ = NULL;
			printf("MainTask: Reconnecting camera...");
			reconnectCamera();
			printf(" Done!\n");
			GevStartTransfer(telCamHandle_, numImages);
			continue;
		} else {
				setIntegerParam(ADStatus, ADStatusReadout);
				getIntegerParam(NDArrayCounter, &imageCounter);
				getIntegerParam(ADNumImages, &numImages);
				getIntegerParam(ADNumImagesCounter, &numImagesCounter);
				getIntegerParam(NDArrayCallbacks, &arrayCallbacks);
				imageCounter++;
				numImagesCounter++;
				setIntegerParam(NDArrayCounter, imageCounter);
				setIntegerParam(ADNumImagesCounter, numImagesCounter);
				/* Call the callbacks to update any changes */
				callParamCallbacks();

				if (arrayCallbacks) {
					/* Call the NDArray callback */
					asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW,
							  "%s:%s: calling imageData callback\n", driverName, functionName);
					doCallbacksGenericPointer(pRaw_, NDArrayData, 0);
				}
		} 

        /* See if acquisition is done */
        if ((imageMode == ADImageSingle) ||
            ((imageMode == ADImageMultiple) &&
             (numImagesCounter >= numImages))) {


            /* First do callback on ADStatus. */
            setStringParam(ADStatusMessage, "Waiting for acquisition");
            setIntegerParam(ADStatus, ADStatusIdle);
            callParamCallbacks();

            acquire = 0;
            setIntegerParam(ADAcquire, acquire);
            asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW,
                      "%s:%s: acquisition completed\n", driverName, functionName);
        }
		if (pRaw_){
				if (verbose) printf("MainTask: Releasing pRaw\n");
				pRaw_->release();
				pRaw_ = NULL;
		}
		if (img != NULL){
				GevReleaseImage(telCamHandle_, img);
		}

        /* Call the callbacks to update any changes */
        callParamCallbacks();
    }
}


/** Called when asyn clients call pasynInt32->write().
  * This function performs actions for some parameters, including ADAcquire, ADColorMode, etc.
  * For all parameters it sets the value in the parameter library and calls any registered callbacks..
  * \param[in] pasynUser pasynUser structure that encodes the reason and address.
  * \param[in] value Value to write. */
asynStatus shadobox::writeInt32(asynUser *pasynUser, epicsInt32 value)
{
    int function = pasynUser->reason;
    int adstatus;
    int acquiring;
    int imageMode;
    asynStatus status = asynSuccess;

    /* Ensure that ADStatus is set correctly before we set ADAcquire.*/
    getIntegerParam(ADStatus, &adstatus);
    getIntegerParam(ADAcquire, &acquiring);
    getIntegerParam(ADImageMode, &imageMode);
    if (function == ADAcquire) {
      if (value && !acquiring) {
        setStringParam(ADStatusMessage, "Acquiring data");
      }
      if (!value && acquiring) {
        setStringParam(ADStatusMessage, "Acquisition stopped");
        if (imageMode == ADImageContinuous) {
          setIntegerParam(ADStatus, ADStatusIdle);
        } else {
          setIntegerParam(ADStatus, ADStatusAborted);
        }
        setIntegerParam(ADStatus, ADStatusAcquire);
      }
    }
    callParamCallbacks();

    /* Set the parameter and readback in the parameter library.  This may be overwritten when we read back the
     * status at the end, but that's OK */
    status = setIntegerParam(function, value);

    /* For a real detector this is where the parameter is sent to the hardware */
    if (function == ADAcquire) {
        if (value && !acquiring) {
            /* Send an event to wake up the simulation task.
             * It won't actually start generating new images until we release the lock below */
            epicsEventSignal(startEventId_);
        }
        if (!value && acquiring) {
            /* This was a command to stop acquisition */
            /* Send the stop event */
            epicsEventSignal(stopEventId_);
			/* If we're GevWaitForFrame we need to abort that */
			GevStopTransfer(telCamHandle_);
        }
	} else if ((function == ADBinX) ||
			   (function == ADBinY)){
			if (value < 1) value = 1;
			if (value > 2) value = 2;
			int binX, binY;
			std::string rmode;
			getIntegerParam(ADBinX, &binX);
			getIntegerParam(ADBinY, &binY);
			if (value!=binX){
					setIntegerParam(ADBinX, value);
			}
			if (value!=binY){
					setIntegerParam(ADBinY, value);
			}
			if (value==2){
					rmode = "Binning";
			} else {
					rmode = "FullResolution";
			}
			setCameraFeature("ReadOutMode", rmode);
			updateSensorSize();
	} else if (function == ADMinX){
			if (value<0) value = 0;
			if (value>ADMaxSizeX-2) value = ADMaxSizeX-2;
			int sizeX, maxSizeX;
			getIntegerParam(ADSizeX, &sizeX);
			getIntegerParam(ADMaxSizeX, &maxSizeX);
			setCameraFeature("ROIStartH", value);
			if (value+sizeX>maxSizeX-1){
					sizeX = maxSizeX-1-value;
					setIntegerParam(ADSizeX, sizeX);
			}
			uint16_t stopX = value+sizeX;
			printf("Setting startH=%d, stopH=%d\n", value, stopX);
			setCameraFeature("ROIStopH", stopX);
	} else if (function == ADMinY){
			if (value<0) value = 0;
			if (value>ADMaxSizeY-2) value = ADMaxSizeY-2;
			int sizeY, maxSizeY;
			getIntegerParam(ADSizeY, &sizeY);
			getIntegerParam(ADMaxSizeY, &maxSizeY);
			setCameraFeature("ROIStartV", value);
			if (value+sizeY>maxSizeY-1){
					sizeY = maxSizeY-1-value;
					setIntegerParam(ADSizeY, sizeY);
			}
			int stopY = value+sizeY;
			printf("Setting startY=%d, stopY=%d\n", value, stopY);
			setCameraFeature("ROIStopV", stopY);
	} else if (function == ADSizeX){
			int maxSizeX, minX;
			getIntegerParam(ADMaxSizeX, &maxSizeX);
			getIntegerParam(ADMinX, &minX);
			if (value<1) value = 1;
			if (value>=maxSizeX-minX) value = maxSizeX-minX-1;
			setIntegerParam(ADSizeX, value);
			printf("roi stopH=%d\n", value);
			setCameraFeature("ROIStopH", value);
	} else if (function == ADSizeY){	
			int maxSizeY, minY;
			getIntegerParam(ADMaxSizeY, &maxSizeY);
			getIntegerParam(ADMinY, &minY);
			if (value<1) value = 1;
			if (value>=maxSizeY-minY) value = maxSizeY-minY-1;
			setIntegerParam(ADSizeY, value);
			printf("roi stopY=%d\n", value);
			setCameraFeature("ROIStopV", value);
	} else if (function == ShadReadOutMode){
			// 0 = Full Resolution, 1 = Binning, 2 = ROI
			setCameraFeature("ReadOutMode", value);
	} else if (function == ShadFullWell){
			setCameraFeature("FullWell", value);
	} else if (function == ShadTrigMode){
			if (value==0){
					setCameraFeature("SynchronizationMode", "Snapshot");
					printf("Synch mode: Snapshot\n");
			} else if (value==1){
					setCameraFeature("SynchronizationMode", "FreeRunning");
					printf("Synch mode: Free Running\n");
			}
	} else if (function == ShadGain){
			setCameraFeature("DigitalGain", value);
	} else if (function == ShadFFCEnable){
			setCameraFeature("FFCEnable", value);
	} else if (function == ShadFFCActive){
			setCameraFeature("FFCActive", value);
	} else if (function == ShadDPCEnable){
			setCameraFeature("DPCEnable", value);
	} else if (function == ShadDPCActive){
			setCameraFeature("DPCActive", value);
	} else if (function == ShadTurbo){
			setCameraFeature("transferTurboMode", value);
    } else {
        /* If this parameter belongs to a base class call its method */
        if (function < FIRST_SHAD_PARAM) status = ADDriver::writeInt32(pasynUser, value);
    }

    /* Do callbacks so higher layers see any changes */
    callParamCallbacks();

    if (status)
        asynPrint(pasynUser, ASYN_TRACE_ERROR,
              "%s:writeInt32 error, status=%d function=%d, value=%d\n",
              driverName, status, function, value);
    else
        asynPrint(pasynUser, ASYN_TRACEIO_DRIVER,
              "%s:writeInt32: function=%d, value=%d\n",
              driverName, function, value);
    return status;
}

/** Called when asyn clients call pasynFloat64->write().
  * This function performs actions for some parameters, including ADAcquireTime, ADGain, etc.
  * For all parameters it sets the value in the parameter library and calls any registered callbacks..
  * \param[in] pasynUser pasynUser structure that encodes the reason and address.
  * \param[in] value Value to write. */
asynStatus shadobox::writeFloat64(asynUser *pasynUser, epicsFloat64 value)
{
    int function = pasynUser->reason;
    asynStatus status = asynSuccess;

    /* Set the parameter and readback in the parameter library.  This may be overwritten when we read back the
     * status at the end, but that's OK */
    status = setDoubleParam(function, value);

    /* Changing any of the simulation parameters requires recomputing the base image */
	if (function == ADAcquireTime){
			if (value<25e-3) value = 25e-3;
			if (value>60.0) value = 60.0;
			setCameraFeature("FrameInterval", (int)(1e6*value));
			setCameraFeature("SoftwareTrigIntTime", (int)(1e6*value));
			status = setDoubleParam(ADAcquireTime, value);
			int type, frameInterval, trigIntTime;
			GevGetFeatureValue(telCamHandle_, "FrameInterval", &type, sizeof(frameInterval), &frameInterval);
			GevGetFeatureValue(telCamHandle_, "SoftwareTrigIntTime", &type, sizeof(trigIntTime), &trigIntTime);
			printf("Frame interval: %d\n", frameInterval);
			printf("Trig int time: %d\n", trigIntTime);
	} else {
        /* This parameter belongs to a base class call its method */
        status = ADDriver::writeFloat64(pasynUser, value);
    }

    /* Do callbacks so higher layers see any changes */
    callParamCallbacks();
    if (status)
        asynPrint(pasynUser, ASYN_TRACE_ERROR,
              "%s:writeFloat64 error, status=%d function=%d, value=%f\n",
              driverName, status, function, value);
    else
        asynPrint(pasynUser, ASYN_TRACEIO_DRIVER,
              "%s:writeFloat64: function=%d, value=%f\n",
              driverName, function, value);
    return status;
}

int shadobox::setCameraFeature(std::string feature, unsigned int value){
		int rbv, type, telStatus;
		telStatus = GevSetFeatureValue(telCamHandle_, feature.c_str(), sizeof(value), &value);
		if (telStatus!=GEVLIB_OK){
				printf("Coud not set %s (status=%d)\n", feature.c_str(), telStatus);
		}
	    GevGetFeatureValue(telCamHandle_, feature.c_str(), &type, sizeof(rbv), &rbv);
		if (rbv!=value){
				printf( "%s:setCameraFeature error, status=%d feature=%s value=%d rbv=%d\n",
					  driverName, telStatus, feature.c_str(), value, rbv);
				return asynError;
		}

		return asynSuccess;
}

int shadobox::setCameraFeature(std::string feature, int value){
		int rbv, type, telStatus;
		telStatus = GevSetFeatureValue(telCamHandle_, feature.c_str(), sizeof(value), &value);
		if (telStatus!=GEVLIB_OK){
				printf("Coud not set %s (status=%d)\n", feature.c_str(), telStatus);
		}
	    GevGetFeatureValue(telCamHandle_, feature.c_str(), &type, sizeof(rbv), &rbv);
		if (rbv!=value){
				printf( "%s:setCameraFeature error, status=%d feature=%s value=%d rbv=%d\n",
					  driverName, telStatus, feature.c_str(), value, rbv);
				return asynError;
		}

		return asynSuccess;
}

int shadobox::setCameraFeature(std::string feature, double value){
		int type, telStatus;
		double rbv;
		telStatus = GevSetFeatureValue(telCamHandle_, feature.c_str(), sizeof(value), &value);
		if (telStatus!=GEVLIB_OK){
				printf("Coud not set %s (status=%d)\n", feature.c_str(), telStatus);
		}
		GevGetFeatureValue(telCamHandle_, feature.c_str(), &type, sizeof(rbv), &rbv);
		if (std::abs(rbv-value)>1e-3){
				printf("%s:setCameraFeature error, status=%d feature=%s value=%f\n",
					  driverName, telStatus, feature.c_str(), value);
				return asynError;
		}
		return asynSuccess;
}

int shadobox::setCameraFeature(std::string feature, std::string value){
		int type, telStatus;
		std::string rbv;
		telStatus = GevSetFeatureValueAsString(telCamHandle_, feature.c_str(), value.c_str());
		if (telStatus!=GEVLIB_OK){
				printf("Could not set %s (status=%d)\n", feature.c_str(), telStatus);
		}
		GevGetFeatureValueAsString(telCamHandle_, feature.c_str(), &type, sizeof(rbv), &rbv[0]);
		if (strcmp(value.c_str(), rbv.c_str())!=0){
				printf("%s:setCameraFeature error, status=%d feature=%s value=%s rbv=%s\n",
					  driverName, telStatus, feature.c_str(), value.c_str(), rbv.c_str());
				return asynError;
		}
		return asynSuccess;
}

/** Report status of the driver.
  * Prints details about the driver if details>0.
  * It then calls the ADDriver::report() method.
  * \param[in] fp File pointed passed by caller where the output is written to.
  * \param[in] details If >0 then driver details are printed.
  */
void shadobox::report(FILE *fp, int details)
{

    fprintf(fp, "Simulation detector %s\n", this->portName);
    if (details > 0) {
        int nx, ny, dataType;
        getIntegerParam(ADSizeX, &nx);
        getIntegerParam(ADSizeY, &ny);
        getIntegerParam(NDDataType, &dataType);
        fprintf(fp, "  NX, NY:            %d  %d\n", nx, ny);
        fprintf(fp, "  Data type:         %d\n", dataType);
    }
    /* Invoke the base class method */
    ADDriver::report(fp, details);
}

int shadobox::connectCamera(){
	static const char* functionName = "connectCamera";
	GEV_STATUS telStatus;
	int numCameras = 0;
	int type, status;
	unsigned int height, width, format;
	unsigned long int size;
	char synchModeStr[64] = {0};
	char versionString[40] = {0};

	GevGetLibraryConfigOptions(&telLibOpt_);
	telLibOpt_.logLevel = GEV_LOG_LEVEL_NORMAL;
	GevSetLibraryConfigOptions(&telLibOpt_);

	telStatus = GevGetCameraList(telCamInt_, MAX_CAMERAS, &numCameras);

	printf("%s:%s: Discovered %d cameras on the network\n",
		driverName, functionName, numCameras);
	if (numCameras==0){
		std::this_thread::sleep_for(std::chrono::milliseconds(3000));
		printf("%s:%s: No cameras detected on network\n",
				driverName, functionName);
		return asynError;
	}

	telStatus = GevOpenCameraBySN(serial_number, GevControlMode, &telCamHandle_);

	if (telStatus==0){
		char xmlFileName[1024] = {0};
		telStatus = GevGetGenICamXML_FileName( telCamHandle_, (int)sizeof(xmlFileName), xmlFileName);
		if (telStatus==GEVLIB_OK){
			printf("XML stored as: %s\n", xmlFileName);
		}

		/* Set some default values for parameters */
		char stringFeature[32] = {0};
		GevGetFeatureValueAsString(telCamHandle_, "DeviceVendorName", &type,  sizeof(stringFeature), stringFeature);
		status =  setStringParam(ADManufacturer, stringFeature);
		memset(stringFeature, 0, sizeof(stringFeature));
		GevGetFeatureValueAsString(telCamHandle_, "DeviceModelName", &type, sizeof(stringFeature), stringFeature);
		status =  setStringParam(ADModel, stringFeature);
		setStringParam(ADSerialNumber, serial_number);
		memset(stringFeature, 0, sizeof(stringFeature));
		GevGetFeatureValueAsString(telCamHandle_, "DeviceVersion", &type, sizeof(stringFeature), stringFeature);
		setStringParam(ADFirmwareVersion, stringFeature);
		memset(stringFeature, 0, sizeof(stringFeature));
		epicsSnprintf(versionString, sizeof(versionString), "%d.%d.%d",
				  DRIVER_VERSION, DRIVER_REVISION, DRIVER_MODIFICATION);
		setStringParam(NDDriverVersion, versionString);
		GevGetFeatureValueAsString(telCamHandle_, "HardwareRevision", &type, sizeof(stringFeature), stringFeature);
		setStringParam(ADSDKVersion, stringFeature);

		uint32_t intFeature = 0;
		uint32_t width, height;
		GevGetFeatureValue(telCamHandle_, "Width", &type, sizeof(width), &width);
		status |= setIntegerParam(ADMaxSizeX, width);
		status |= setIntegerParam(ADSizeX, width);
		status |= setIntegerParam(NDArraySizeX, width);
		GevGetFeatureValue(telCamHandle_, "Height", &type, sizeof(height), &height);
		status |= setIntegerParam(ADMaxSizeY, height);
		status |= setIntegerParam(ADSizeY, height);
		status |= setIntegerParam(NDArraySizeY, height);
		status |= setIntegerParam(ADMinX, 0);
		status |= setIntegerParam(ADMinY, 0);
		status |= setIntegerParam(ADBinX, 1);
		status |= setIntegerParam(ADBinY, 1);
		status |= setIntegerParam(ADReverseX, 0);
		status |= setIntegerParam(ADReverseY, 0);
		status |= setIntegerParam(NDArraySize, 0);
		status |= setIntegerParam(NDDataType, NDInt16);
		status |= setIntegerParam(NDColorMode, NDColorModeMono);
		status |= setIntegerParam(NDArraySize, 2*width*height);
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
		bufSize = width*height*2;
		for (int i=0; i<numBuffers; i++){
			bufAddress[i] = (PUINT8)malloc(bufSize);
			memset(bufAddress[i], 0, bufSize);
		}
		telStatus = GevInitializeTransfer(telCamHandle_, SynchronousNextEmpty, bufSize, numBuffers, bufAddress);

	} else {
			asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
				"%s:%s: Could not open camera (Error: %d)\n",
				driverName, functionName, numCameras);
			this->disconnect(this->pasynUserSelf);
		return asynError;
	}

	callParamCallbacks();
	return asynSuccess;
}

void shadobox::updateSensorSize(void){
		int type, width, height;
		int status;
		GevGetFeatureValue(telCamHandle_, "Width", &type, sizeof(width), &width);
		status |= setIntegerParam(ADMaxSizeX, width);
		status |= setIntegerParam(ADSizeX, width);
		status |= setIntegerParam(NDArraySizeX, width);
		GevGetFeatureValue(telCamHandle_, "Height", &type, sizeof(height), &height);
		status |= setIntegerParam(ADMaxSizeY, height);
		status |= setIntegerParam(ADSizeY, height);
		status |= setIntegerParam(NDArraySizeY, height);
		status |= setIntegerParam(NDArraySize, 2*width*height);
		callParamCallbacks();
}

/** Constructor for shadobox; most parameters are simply passed to ADDriver::ADDriver.
  * After calling the base class constructor this method creates a thread to compute the simulated detector data,
  * and sets reasonable default values for parameters defined in this class, asynNDArrayDriver and ADDriver.
  * \param[in] portName The name of the asyn port driver to be created.
  * \param[in] serial_number The serial number used to connect to the shadobox
  * \param[in] maxBuffers The maximum number of NDArray buffers that the NDArrayPool for this driver is
  *            allowed to allocate. Set this to -1 to allow an unlimited number of buffers.
  * \param[in] maxMemory The maximum amount of memory that the NDArrayPool for this driver is
  *            allowed to allocate. Set this to -1 to allow an unlimited amount of memory.
  * \param[in] priority The thread priority for the asyn port driver thread if ASYN_CANBLOCK is set in asynFlags.
  * \param[in] stackSize The stack size for the asyn port driver thread if ASYN_CANBLOCK is set in asynFlags.
  */
shadobox::shadobox(const char *portName, char* serial_number, int maxBuffers, size_t maxMemory, int priority, int stackSize)

    : ADDriver(portName, 1, 0, maxBuffers, maxMemory,
               0, 0, /* No interfaces beyond those set in ADDriver.cpp */
               0, 1, /* ASYN_CANBLOCK=0, ASYN_MULTIDEVICE=0, autoConnect=1 */
               priority, stackSize),
	  pRaw_(NULL), serial_number(serial_number) 
{
    int status = asynSuccess;
    char versionString[20];
    const char *functionName = "shadobox";

    /* Create the epicsEvents for signaling to the simulate task when acquisition starts and stops */
    startEventId_ = epicsEventCreate(epicsEventEmpty);
    if (!startEventId_) {
        printf("%s:%s epicsEventCreate failure for start event\n",
            driverName, functionName);
        return;
    }
    stopEventId_ = epicsEventCreate(epicsEventEmpty);
    if (!stopEventId_) {
        printf("%s:%s epicsEventCreate failure for stop event\n",
            driverName, functionName);
        return;
    }

	createParam(ShadFullWellString, 		  asynParamInt32,	&ShadFullWell);
	createParam(ShadGainString,				  asynParamInt32,   &ShadGain);
	createParam(ShadTrigModeString,			  asynParamInt32,   &ShadTrigMode);
	createParam(ShadRetriesString,			  asynParamInt32,   &ShadRetries);
	createParam(ShadReadOutModeString,		  asynParamInt32,   &ShadReadOutMode);
	createParam(ShadTurboString,			  asynParamInt32,   &ShadTurbo);
	createParam(ShadFFCEnableString,		  asynParamInt32, 	&ShadFFCEnable);
	createParam(ShadFFCActiveString,		  asynParamInt32, 	&ShadFFCActive);
	createParam(ShadDPCEnableString,		  asynParamInt32,   &ShadDPCEnable);
	createParam(ShadDPCActiveString,		  asynParamInt32,   &ShadDPCActive);

	status = connectCamera();
	if (status){
			printf("Didn't connect to camera\n");
			return;
	}

    /* Create the thread that updates the images */
    status = (epicsThreadCreate("ShadoboxMainDataTask",
                                epicsThreadPriorityMedium,
                                epicsThreadGetStackSize(epicsThreadStackMedium),
                                (EPICSTHREADFUNC)mainDataTaskC,
                                this) == NULL);
    if (status) {
        printf("%s:%s epicsThreadCreate failure for image grab task\n",
            driverName, functionName);
        return;
    }

    /* Create the thread that updates the temperature*/
    status = (epicsThreadCreate("ShadoboxTempReadTask",
                                epicsThreadPriorityMedium,
                                epicsThreadGetStackSize(epicsThreadStackMedium),
                                (EPICSTHREADFUNC)tempReadTaskC,
                                this) == NULL);
    if (status) {
        printf("%s:%s epicsThreadCreate failure for temp read task\n",
            driverName, functionName);
        return;
    }


	epicsAtExit(c_shutdown, this);
	return;
}

/** Configuration command, called directly or from iocsh */
extern "C" int shadoboxConfig(const char *portName, char* serialNumber,
                                 int maxBuffers, int maxMemory, int priority, int stackSize)
{
    new shadobox(portName, serialNumber, 
                    (maxBuffers < 0) ? 0 : maxBuffers,
                    (maxMemory < 0) ? 0 : maxMemory,
                    priority, stackSize);
    return(asynSuccess);
}

/** Code for iocsh registration */
static const iocshArg shadoboxConfigArg0 = {"Port name", iocshArgString};
static const iocshArg shadoboxConfigArg1 = {"Serial Number", iocshArgString};
static const iocshArg shadoboxConfigArg2 = {"maxBuffers", iocshArgInt};
static const iocshArg shadoboxConfigArg3 = {"maxMemory", iocshArgInt};
static const iocshArg shadoboxConfigArg4 = {"priority", iocshArgInt};
static const iocshArg shadoboxConfigArg5 = {"stackSize", iocshArgInt};
static const iocshArg * const shadoboxConfigArgs[] =  {&shadoboxConfigArg0,
                                                       &shadoboxConfigArg1,
                                                       &shadoboxConfigArg2,
                                                       &shadoboxConfigArg3,
                                                       &shadoboxConfigArg4,
                                                       &shadoboxConfigArg5};
static const iocshFuncDef configShadobox = {"shadoboxConfig", 6, shadoboxConfigArgs};
static void configShadoboxCallFunc(const iocshArgBuf *args)
{
    shadoboxConfig(args[0].sval, args[1].sval, args[2].ival, args[3].ival,
                      args[4].ival, args[5].ival);
}

static void shadoboxRegister(void)
{

    iocshRegister(&configShadobox, configShadoboxCallFunc);
}

extern "C" {
epicsExportRegistrar(shadoboxRegister);
}
